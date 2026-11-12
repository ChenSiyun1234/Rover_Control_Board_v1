"""Schematic-derived FSR channel and sampled MCU pin, solved by ngspice."""
from pathlib import Path
import re
import math
import time

import numpy as np

from board.extract_fsr import REVISIONS
from hw.spice import simulate_released
from sim.fsr import FSRSensors, resistance, VREF, FULL_SCALE, static_channel

# DS12288 Rev 6, Table 66, pp.138-139: C_ADC typical 5 pF;
# sampling time up to 640.5 cycles. Assumed single-ADC clock: 60 MHz.
# https://www.st.com/resource/en/datasheet/stm32g474ce.pdf
C_ADC = 5e-12
ADC_CLOCK = 60e6
SAMPLE_TIME = 640.5 / ADC_CLOCK
# Table 54, p.128: FT_xx except FT_c, 0<Vin<=VDD: +/-100 nA.
# PE10 and PE13 are FT_a (Table 12). Assume worst sinking direction when enabled.
MAX_LEAKAGE = 100e-9
# R_ADC is NOT specified for this MCU: assume 1 kOhm; test 0.5..6 kOhm.
R_ADC = 1000
PERIOD = 1e-3  # Requested firmware sample period.


def front_end(revision):
    """Load the extracted subcircuit, changing only R26 for the proposed fix."""
    rev = REVISIONS["now" if revision == "fix" else revision]
    text = (Path(__file__).resolve().parents[1] / "board" / f"fsr_front_{rev}.cir").read_text()
    if revision == "fix":
        text, count = re.subn(r"(?m)^(R26\s+\S+\s+\S+)\s+\S+$", r"\g<1> 2.2k", text)
        assert count == 1
    return text


class SpiceFSRSensors(FSRSensors):
    """One transient per tick; every board capacitor and the ADC hold state persist."""

    def __init__(self, parameters, revision="now", sh_start="previous", r_adc=R_ADC,
                 leakage=False, sampling=True):
        super().__init__(parameters, revision)
        if sh_start not in ("previous", "ground"):
            raise ValueError("sh_start must be previous or ground")
        self.sh_start, self.r_adc = sh_start, r_adc
        self.leakage = MAX_LEAKAGE if leakage is True else float(leakage)
        self.sampling = sampling
        self.subcircuit = front_end(revision)
        self.gate = " gate" if revision == "prev" else ""
        # Extract capacitor endpoints, rather than assuming a particular board RC.
        ports = dict(sensor_a="a", sensor_b="b", adc="adc", v3v3="supply", gnd="0", gate="gate")
        self.cap_nodes = set()
        for line in self.subcircuit.splitlines():
            if line.startswith("C"):
                for node in line.split()[1:3]:
                    mapped = ports.get(node, "xfront." + node)
                    if mapped != "0":
                        self.cap_nodes.add(mapped)
        self.state = dict.fromkeys(self.cap_nodes, 0.0)
        self.hold = 0.0
        self.forces = []

    def circuit(self, forces, dc=False):
        # Each plant force applies over its just-completed plant interval. Near-
        # instantaneous PWL edges (assumed 1 ps) preserve the held-force waveform.
        dt = PERIOD / len(forces)
        conductances = [0.0 if math.isinf(f) else 1 / resistance(f) for f in forces]
        points = [f"0 {conductances[0]:.17g}"]
        for i, g in enumerate(conductances[1:], 1):
            points += [f"{i*dt:.17g} {conductances[i-1]:.17g}",
                       f"{i*dt+1e-12:.17g} {g:.17g}"]
        points.append(f"{PERIOD:.17g} {conductances[-1]:.17g}")
        lines = ["FSR sampled channel", self.subcircuit,
                 f"Vref supply 0 {VREF}", "Vgate gate 0 0",
                 f"Xfront a b adc supply 0{self.gate} fsr_front",
                 "Vconductance g 0 " + (str(conductances[0]) if dc else "PWL(" + " ".join(points) + ")"),
                 "Bfsr a b I=V(a,b)*V(g)", f"Ileak adc 0 {self.leakage}"]
        if self.sampling and not dc:
            start = PERIOD - SAMPLE_TIME
            # Assumed ideal acquisition switch: Ron=1mOhm, Roff=1e15Ohm;
            # the separate R_ADC supplies the assumed silicon resistance.
            lines += [f"Vsample sample 0 PWL(0 0 {start:.17g} 0 {start+1e-12:.17g} 1 {PERIOD} 1)",
                      "Sacquire adc input sample 0 acquisition",
                      ".model acquisition SW(Ron=0.001 Roff=1e15 Vt=0.5 Vh=0)",
                      f"Radc input hold {self.r_adc}", f"Cadc hold 0 {C_ADC}"]
        state = dict(self.state)
        if self.sampling:
            state["hold"] = self.hold if self.sh_start == "previous" else 0.0
        if dc:
            lines += [".op", ".save v(adc)"]
        else:
            lines += [".ic " + " ".join(f"v({n})={v:.17g}" for n, v in state.items()),
                      # Assumed numerical limits; resolve acquisition with adaptive steps.
                      ".options reltol=1e-6 abstol=1e-13 vntol=1e-8",
                      f".tran 10u {PERIOD} 0 10u uic",
                      ".save v(adc) " + " ".join(f"v({n})" for n in sorted(self.cap_nodes))
                      + (" v(hold)" if self.sampling else "")]
        return "\n".join(lines + [".end", ""])

    def period(self, forces):
        """Solve a complete sample period; infinity explicitly denotes an open sensor."""
        result = simulate_released(self.circuit(forces))
        for node in self.cap_nodes:
            self.state[node] = float(result[node][-1])
        self.voltage = float(result["adc"][-1])
        self.hold = float(result["hold"][-1]) if self.sampling else self.voltage
        return self.hold

    def dc(self, force):
        return float(simulate_released(self.circuit([force], dc=True))["adc"][-1])

    def step(self, plant, time_s, stop=False, full=False):
        self.forces.append(plant.force)
        if time_s + 1e-9 < self.next_ready:
            return False
        self.next_ready += 1 / self.rate
        self.period(self.forces)
        self.forces.clear()
        if stop:
            return False
        # Same Gaussian draw, clipping, rounding and full-scale short circuit as FSRSensors.
        self.code = FULL_SCALE if full else max(0, min(FULL_SCALE, round(
            FULL_SCALE * self.hold / VREF + self.rng.normal(0, 1.5))))
        self.codes.append(self.code)
        return True


def steady(parameters, force, revision, **options):
    """Converge the physical capacitor state to 1 nV between periods (assumed)."""
    sensor = SpiceFSRSensors(parameters, revision, **options)
    for _ in range(2000):  # Assumed ceiling: 2 s, >20 off-state time constants.
        before = np.array([*sensor.state.values(), sensor.hold])
        sensor.period([force])
        if np.max(np.abs(np.array([*sensor.state.values(), sensor.hold]) - before)) < 1e-9:
            return sensor.hold
    raise RuntimeError("Periodic FSR state did not converge")


def static_rows(parameters):
    for revision in ("now", "prev", "fix"):
        sensor = SpiceFSRSensors(parameters, revision)
        for force in (0, 5, 15, 25, 35):
            row = dict(revision=revision, force_N=force)
            values = dict(formula=static_channel(force, revision)["node_volts"],
                          spice_dc=sensor.dc(force),
                          previous=steady(parameters, force, revision),
                          ground=steady(parameters, force, revision, sh_start="ground"),
                          leakage=steady(parameters, force, revision, leakage=True))
            for key, voltage in values.items():
                row[key + "_V"] = voltage
                row[key + "_code"] = max(0, min(FULL_SCALE, round(FULL_SCALE*voltage/VREF)))
            yield row


def process_memory():
    """Current Windows working set and private bytes, not the high-water mark."""
    import ctypes as ct
    from ctypes import wintypes
    class Counters(ct.Structure):
        _fields_ = [("cb", wintypes.DWORD), ("PageFaultCount", wintypes.DWORD)] + [
            (n, ct.c_size_t) for n in ("PeakWorkingSetSize", "WorkingSetSize", "QuotaPeakPagedPoolUsage",
                                     "QuotaPagedPoolUsage", "QuotaPeakNonPagedPoolUsage",
                                     "QuotaNonPagedPoolUsage", "PagefileUsage", "PeakPagefileUsage", "PrivateUsage")]
    counters = Counters()
    counters.cb = ct.sizeof(counters)
    ct.windll.psapi.GetProcessMemoryInfo(ct.c_void_p(-1), ct.byref(counters), counters.cb)
    return dict(working_set=counters.WorkingSetSize, private_bytes=counters.PrivateUsage)


def benchmark(parameters, periods=10000):
    sensor = SpiceFSRSensors(parameters)
    for _ in range(100):  # Assumed DLL/allocator warmup; not included in the measured run.
        sensor.period([5]*10)
    readings = [dict(period=0, **process_memory())]
    started = time.perf_counter()
    for i in range(periods):
        sensor.period([5]*10)
        if (i+1) % 1000 == 0:
            readings.append(dict(period=i+1, **process_memory()))
    elapsed = time.perf_counter() - started
    return dict(periods=periods, elapsed_s=elapsed, seconds_per_simulated_second=elapsed/(periods*PERIOD),
                memory=readings)
