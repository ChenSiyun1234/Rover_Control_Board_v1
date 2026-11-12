"""Single-active-instance DLL wrapper and deterministic UART/physics runner."""

import ctypes as C
import math
from pathlib import Path

import numpy as np

from host.axis_client import Client
from host.axis_client.identifiers import ParameterId, State
from host.axis_client.parameters import PARAMETER_IDS
from sim.trace import TraceColumn

from .params import Params
from .plant import Plant
from .sensors import Sensors

ROOT = Path(__file__).resolve().parents[1]
STATES = ("DISABLED", "IDLE", "MOVING", "SEATED", "RETRACTING", "FAULT", "ESTOP", "HOMING", "HOME_BACKOFF")


class Inputs(C.Structure):
    """Match the native C input ABI."""

    _fields_ = [
        ("now_ms", C.c_uint32),
        ("encoder_counts", C.c_int32),
        ("adc_code", C.c_int32),
        ("adc_new", C.c_bool),
        ("estop_ok", C.c_bool),
        ("limit_switch", C.c_bool),
    ]


class Outputs(C.Structure):
    """Match the native C driver output ABI."""

    _fields_ = [("current_cmd_A", C.c_float), ("drive_enable", C.c_bool),
                ("pwm_duty", C.c_float), ("dir", C.c_uint8)]


class Debug(C.Structure):
    """Expose the native C telemetry snapshot."""

    _fields_ = (
        [("state", C.c_uint32), ("fault_flags", C.c_uint32)]
        + [
            (x, C.c_float)
            for x in (
                "force_N",
                "force_raw_N",
                "carriage_mm",
                "server_est_mm",
                "vel_mm_s",
                "limit_N",
                "current_A",
            )
        ]
        + [
            (x, C.c_uint32)
            for x in ("now_ms", "crc_error_count", "frame_drop_count", "tare_samples")
        ]
        + [
            ("velocity_ref", C.c_float),
            ("fault_ms", C.c_uint32),
            ("detection_force_N", C.c_float),
        ]
    )


class Firmware:
    """Bind the single C controller instance in the SIL DLL."""

    def __init__(self):
        """Initialize the instance."""
        self.lib = C.CDLL(str(ROOT / "build/axis_fw.dll"))
        self.lib.fw_step.argtypes = [C.POINTER(Inputs), C.POINTER(Outputs)]
        self.lib.fw_uart_rx.argtypes = [C.POINTER(C.c_uint8), C.c_size_t]
        self.lib.fw_uart_tx.argtypes = [C.POINTER(C.c_uint8), C.c_size_t]
        self.lib.fw_uart_tx.restype = C.c_size_t
        self.lib.fw_get_debug.argtypes = [C.POINTER(Debug)]
        self.lib.fw_crc16.argtypes = [C.POINTER(C.c_uint8), C.c_size_t]
        self.lib.fw_crc16.restype = C.c_uint16
        self.lib.fw_init()
        self.out = Outputs()

    def write(self, data):
        """Queue bytes for the receiving endpoint."""
        buffer = (C.c_uint8 * len(data)).from_buffer_copy(data)
        self.lib.fw_uart_rx(buffer, len(data))

    def read(self):
        """Drain bytes waiting at the receiving endpoint."""
        buffer = (C.c_uint8 * 8192)()
        byte_count = self.lib.fw_uart_tx(buffer, len(buffer))
        return bytes(buffer[:byte_count])

    def step(self, now_ms, encoder_counts=0, adc_code=0, adc_ready=True, estop_ok=True, limit_switch=False):
        """Advance one simulation or control tick."""
        self.lib.fw_step(
            C.byref(Inputs(now_ms, encoder_counts, adc_code, adc_ready, estop_ok, limit_switch)),
            C.byref(self.out),
        )
        return self.out

    @property
    def debug(self):
        """Expose the native C telemetry snapshot."""
        debug = Debug()
        self.lib.fw_get_debug(C.byref(debug))
        return debug

    def crc(self, data):
        """Crc."""
        return self.lib.fw_crc16(
            (C.c_uint8 * len(data)).from_buffer_copy(data), len(data)
        )


class Sim:
    """Run the C firmware against deterministic mechanics, sensors and UART."""

    def __init__(self, config=None):
        """Initialize the instance."""
        self.config = config or {}
        physical = self.config.get("physical", {})
        self.parameters = Params(**physical)
        self.plant = self.config.get("plant_factory", Plant)(
            self.parameters, self.config.get("jam", False)
        )
        self.sensors = (self.config["sensor_factory"](self.parameters)
                        if "sensor_factory" in self.config else
                        Sensors(self.parameters, self.config.get("adc", "ADS1220_1000")))
        self.firmware = Firmware()
        self.now_ms = 0
        self.rx = bytearray()
        self.tx = bytearray()
        self.received = bytearray()
        self.rx_credit = self.tx_credit = 0.0
        self.client = Client(self)
        self.records = []
        self.plant_records = []
        self.peak_true_force_N = 0.0
        self.peak_current_A = 0.0
        self.drive_mode = self.config.get("params", {}).get("drive_mode", 0)
        self.first_contact_s = None
        self.first_limit_crossing_s = None
        self.first_current_limit_s = None
        self.diagnostic_limit_N = self.config.get("params", {}).get("free_limit_N", 60)
        self.client.set_param(ParameterId.LC_COUNTS_PER_N, self.sensors.counts_per_N)
        self.client.set_param(
            ParameterId.PUSHER_STIFFNESS_N_PER_MM, self.parameters.pusher_k
        )
        self.client.set_param(
            ParameterId.STALE_TIMEOUT_MS, max(5, math.ceil(3000 / self.sensors.rate))
        )
        for key, value in self.config.get("params", {}).items():
            parameter_id = key if isinstance(key, int) else PARAMETER_IDS[key]
            self.client.set_param(parameter_id, value)
        self.advance(math.ceil(self.sensors.window * 1000) + 30)

    def write(self, data):
        """Queue bytes for the receiving endpoint."""
        silent = self.config.get("host_silent_ms", float("inf"))
        if self.now_ms < silent:
            self.rx.extend(data)

    def read(self):
        """Drain bytes waiting at the receiving endpoint."""
        received_bytes = bytes(self.received)
        self.received.clear()
        return received_bytes

    def step(self):
        """Advance one simulation or control tick."""
        adc_ready = False
        for substep in range(10):
            time_s = (self.now_ms + (substep + 1) / 10) / 1000
            out = self.firmware.out
            if self.drive_mode == 1:
                self.plant.step_pwm(out.pwm_duty, out.dir, out.drive_enable)
            else:
                self.plant.step(out.current_cmd_A, out.drive_enable)
            self.peak_current_A = max(self.peak_current_A, abs(self.plant.current))
            self.plant_records.append((time_s, self.plant.jam_force()))
            # Scalar diagnostics retain 10 kHz events.
            self.peak_true_force_N = max(self.peak_true_force_N, self.plant.force)
            if self.plant.server_position_m * 1000 >= self.parameters.x_jam:
                if self.first_contact_s is None:
                    self.first_contact_s = time_s
            if self.plant.force > self.diagnostic_limit_N:
                if self.first_limit_crossing_s is None:
                    self.first_limit_crossing_s = time_s
            if abs(self.plant.current) >= 0.999 * self.parameters.max_current:
                if self.first_current_limit_s is None:
                    self.first_current_limit_s = time_s
            adc_ready = (
                self.sensors.step(
                    self.plant,
                    time_s,
                    time_s * 1000 >= self.config.get("adc_stop_ms", float("inf")),
                    time_s * 1000 >= self.config.get("adc_full_ms", float("inf")),
                )
                or adc_ready
            )
        self.now_ms += 1
        self.firmware.step(
            self.now_ms,
            self.sensors.encoder(self.plant) * getattr(self.plant, "encoder_sign", 1),
            self.sensors.code,
            adc_ready,
            self.now_ms < self.config.get("estop_ms", float("inf")),
            getattr(self.plant, "limit_switch", False),
        )
        self.rx_credit += self.parameters.baud / 10000
        byte_count = min(len(self.rx), int(self.rx_credit))
        self.rx_credit -= int(self.rx_credit)
        if byte_count:
            self.firmware.write(bytes(self.rx[:byte_count]))
            del self.rx[:byte_count]
        self.tx.extend(self.firmware.read())
        self.tx_credit += self.parameters.baud / 10000
        byte_count = min(len(self.tx), int(self.tx_credit))
        self.tx_credit -= int(self.tx_credit)
        self.received.extend(self.tx[:byte_count])
        del self.tx[:byte_count]
        debug = self.firmware.debug
        self.records.append(
            (
                self.now_ms / 1000,
                self.plant.carriage_position_m * 1000,
                self.plant.server_position_m * 1000,
                self.plant.force,
                debug.force_N,
                debug.state,
                debug.current_A,
                float(self.firmware.out.drive_enable),
                debug.vel_mm_s,
                debug.velocity_ref,
                debug.force_raw_N,
                debug.limit_N,
                debug.server_est_mm,
                self.plant.carriage_velocity_m_s * 1000,
                self.plant.current,
                self.plant.jam_force(),
                self.firmware.out.pwm_duty,
                self.firmware.out.dir,
                debug.carriage_mm,
                float(getattr(self.plant, "limit_switch", False)),
            )
        )

    def advance(self, ms, heartbeat=False):
        """Advance the requested milliseconds with optional host heartbeats."""
        for _ in range(ms):
            if heartbeat and self.now_ms % 50 == 0:
                self.client.ping()
            self.step()

    def prepare(self):
        """Enable the controller and wait for the stationary tare."""
        self.client.enable()
        self.advance(
            math.ceil(
                50 * max(1, 1000 / self.sensors.rate) + self.sensors.window * 1000
            )
            + 20,
            True,
        )
        if self.firmware.debug.tare_samples != 50:
            raise RuntimeError("Tare did not finish")

    def run_insert(self, max_ms=12000):
        """Run until seating, a fault, target completion, or the deadline."""
        self.prepare()
        self.client.start_insert(296)
        started = self.now_ms
        self.insertion_started_ms = started
        was_moving = False
        for _ in range(max_ms):
            if self.now_ms % 50 == 0:
                self.client.ping()
            self.step()
            state = self.firmware.debug.state
            if was_moving and state == State.IDLE:
                break
            was_moving |= state == State.MOVING
            if state in (State.SEATED, State.FAULT, State.ESTOP):
                break
        self.cycle_time = (self.now_ms - started) / 1000
        return self.summary()

    @property
    def trace(self):
        """Return the recorded millisecond samples as a NumPy array."""
        return np.asarray(self.records)

    @property
    def plant_trace(self):
        """Return 10 kHz time and rack jam force samples."""
        return np.asarray(self.plant_records)

    def summary(self):
        """Return seating and fault metrics at the current tick."""
        debug = self.firmware.debug
        return dict(
            final_state=STATES[debug.state],
            seat_position_mm=self.plant.server_position_m * 1000,
            peak_force_N=float(self.trace[:, TraceColumn.TRUE_FORCE_N].max()),
            cycle_time_s=getattr(self, "cycle_time", 0),
            detection_force_N=debug.detection_force_N,
            fault_flags=debug.fault_flags,
            fault_ms=debug.fault_ms,
        )
