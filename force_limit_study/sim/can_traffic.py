"""Proposed CAN message table and open/closed-loop force transport.

All arm IDs, payloads, rates and phases are ASSUMED, except the team-given
second-bus topology. Main -> each of three steppers: 0x100+n, 8 bytes,
100 Hz; status -> main: 0x180+n, 8 bytes, 100 Hz. Force -> main: 0x080
or 0x700, little-endian ADC (2-byte unsigned FSR, 3-byte signed bridge),
requested send_ms. Arm bitrate 1 Mbit/s
(assumed equal to drive) or the requested 500 kbit/s sensitivity.
Shared bus adds DM IDs 1..6: 0x200+id, float32 little-endian velocity;
8-byte feedback encoding and 1 Mbit/s from the read-only can_dm.py manual
pp.5-7 notes. Feedback IDs 0x010+id are ASSUMED; identical master IDs are
a configuration question for the team, not an operating scenario here.
100 Hz command burst is ASSUMED as in that simulation.
Motor delay is ASSUMED 0.1 ms after command EOF, independent of ticks.
Feedback encoding below
uses representative p=0, v=1 rad/s, torque=1 Nm, temperatures=25 C, enabled.
Representative stepper position/speed/current and motor speed are ASSUMED.

ASSUMED driver behavior: a main-board application burst feeds the three
hardware slots as they free; no arbitrary inter-command timer. Nodes use
both independent 3-element Rx FIFOs: force in FIFO0, other accepted traffic
in FIFO1, both blocking (RM0440 reset). Main drains at each receive interrupt
(interrupt latency is not modelled), optionally once per 1 ms tick. Firmware
uses the newest force code at its 1 ms tick. Stepper drains retain ticks. No drift.
Node phase is seeded uniform [0,1) ms. The end-effector phase is quantized
upward to the requested 0.1 ms plant cadence, identically in open and closed
loop; other node clocks retain their continuous event-driven phase.
Applying the same three-Tx/two-three-element-Rx limits to DM motor nodes is
ASSUMED; the reference protocol notes do not identify their hardware RAM.
"""

from collections import deque
import csv
import math
from pathlib import Path
import struct

import numpy as np

from .can_bus import CanBus, Frame, frame_bits


class Traffic:
    def __init__(self, scenario="arm_bus", bitrate=1_000_000, priority="high",
                 send_ms=1, ber=0, seed=7, phase=None, background=True, adc_bytes=2,
                 rx_drain="isr"):
        if rx_drain not in ("isr", "tick"):
            raise ValueError("rx_drain must be isr or tick")
        self.bus = CanBus(bitrate, ber, seed)
        self.scenario, self.send_ms = scenario, send_ms
        self.adc_bytes = adc_bytes
        self.force_id = 0x080 if priority == "high" else 0x700
        self.main = self.bus.node("main", ([self.force_id],
                                  [*range(0x011, 0x017), 0x181, 0x182, 0x183]), 0)
        self.force = self.bus.node("end_effector", phase=phase)
        self.force.phase = math.ceil(self.force.phase * 10000 - 1e-12) / 10000
        self.records, self.held_ages = [], []
        self.held_times = []
        self.pending = None
        self.rx_drain = rx_drain
        if rx_drain == "isr":
            self.main.on_receive = lambda frame: self._receive_main()
        self.latest = None
        self.code = 0
        self.new = False
        self.burst = deque()
        self.force_frames = []
        self.main.on_sent = lambda frame: self._pump()
        self._periodic(.001, .001, self._drain_main, order=2)
        if background:
            for n in range(1, 4):
                node = self.bus.node(f"stepper{n}", ([], [0x100+n]))
                self._node_drain(node)
                self._periodic(.001 + node.phase, .010,
                               lambda n=n, node=node: self.bus.send(node, Frame(
                                   0x180+n, struct.pack("<ihh", 12000+n*200, 300, 800))))
            if scenario == "shared_bus":
                for n in range(1, 7):
                    node = self.bus.node(f"motor{n}", ([], [0x200+n]))
                    def received(frame, node=node, n=n):
                        # DM feedback fields quantized with ranges from can_dm.py.
                        data = bytes([0x10+n, 0x80, 0x00, 0x84, 0x48, 0xaa, 25, 25])
                        node.drain()
                        self.bus.schedule(frame.end + .0001,
                                          lambda: self.bus.send(node, Frame(0x010+n, data)))
                    node.on_receive = received
            self._periodic(.010, .010, self._commands)

    def _periodic(self, first, period, callback, order=1):
        def tick(index):
            callback()
            self.bus.schedule(first + (index+1)*period, lambda: tick(index+1), order)
        self.bus.schedule(first, lambda: tick(0), order)

    def _node_drain(self, node, handler=None):
        scheduled = False
        def drain():
            nonlocal scheduled
            scheduled = False
            for frame in node.drain():
                if handler:
                    handler(frame)
        def arrived(frame):
            nonlocal scheduled
            if not scheduled:
                scheduled = True
                tick = max(1, math.ceil((self.bus.now-node.phase)/.001-1e-10))
                self.bus.schedule(node.phase+tick*.001, drain, order=2)
        # Skip empty drains without changing the node's tick grid or FIFO waits.
        node.on_receive = arrived

    def _commands(self):
        for n in range(1, 4):
            self.burst.append(Frame(0x100+n, struct.pack("<ihh", 14000+n*200, 300, 1000)))
        if self.scenario == "shared_bus":
            for n in range(1, 7):
                self.burst.append(Frame(0x200+n, struct.pack("<f", 1.0 + n*.1)))
        self._pump()

    def _pump(self):
        while self.burst and len(self.main.tx) < 3 and not self.main.bus_off:
            self.bus.send(self.main, self.burst.popleft())

    def send_force(self, sample, code):
        frame = Frame(self.force_id, int(code).to_bytes(self.adc_bytes, "little",
                                                       signed=self.adc_bytes == 3), sample)
        self.force_frames.append(frame)
        self.bus.send(self.force, frame)

    def _receive_main(self):
        received = [f for f in self.main.drain() if f.can_id == self.force_id]
        if received:
            self.pending = received[-1]

    def _drain_main(self):
        if self.rx_drain == "tick":
            self._receive_main()
        self.new = self.pending is not None
        if self.new:
            frame, self.pending = self.pending, None
            self.code = int.from_bytes(frame.data, "little", signed=self.adc_bytes == 3)
            self.latest = frame
            self.records.append(dict(sample_s=frame.sample, frame_start_s=frame.start,
                                     frame_end_s=frame.end, use_s=self.bus.now,
                                     age_ms=(self.bus.now-frame.sample)*1000))
        if self.latest:
            self.held_ages.append((self.bus.now-self.latest.sample)*1000)
            self.held_times.append(self.bus.now)

    def stats(self, duration, start=0, baseline=None):
        # Firmware uses the held code on EVERY tick, including ticks with adc_new
        # false. Reporting only arrival ages would hide the send-period effect.
        ages = [a for t, a in zip(self.held_times, self.held_ages) if t > start]
        delivery_ages = [r["age_ms"] for r in self.records if r["use_s"] > start]
        frames = [f for f in self.force_frames if f.queued >= start]
        sizes = [frame_bits(f.can_id, f.data) for f in frames]
        # Clip a transmission crossing the observation boundary.
        pending_ends = [t for t, order, _, _ in self.bus.events if order == 0]
        spill = max(0, max(pending_ends, default=duration)-duration) if self.bus.active else 0
        result = dict(bus_load_pct=100*(self.bus.busy_time-spill)/duration,
                    force_frame_bits=float(np.mean(sizes)) if sizes else 0,
                    force_bits_min=min(sizes, default=0), force_bits_max=max(sizes, default=0),
                    wire_time_us=float(np.mean(sizes))/self.bus.bitrate*1e6 if sizes else 0,
                    age_mean_ms=float(np.mean(ages)) if ages else None,
                    age_p99_ms=float(np.percentile(ages, 99)) if ages else None,
                    age_max_ms=max(ages, default=None),
                    delivery_age_mean_ms=float(np.mean(delivery_ages)) if delivery_ages else None,
                    delivery_age_max_ms=max(delivery_ages, default=None),
                    held_age_mean_ms=float(np.mean(ages)) if ages else None,
                    held_age_max_ms=max(ages, default=None),
                    worst_arbitration_wait_ms=max((f.arbitration_wait for f in frames
                                                   if f.start is not None), default=0)*1000,
                    rx_fifo_losses=sum(n.rx_losses for n in self.bus.nodes),
                    tx_losses=sum(n.tx_losses for n in self.bus.nodes),
                    force_rx_losses=sum(n.force_rx_losses for n in self.bus.nodes),
                    force_tx_losses=sum(n.force_tx_losses for n in self.bus.nodes),
                    error_attempts=self.bus.errors,
                    bus_off_nodes=sum(n.bus_off for n in self.bus.nodes))
        if baseline is not None:
            result["bus_load_pct"] = (result["bus_load_pct"]*duration
                                      - baseline["bus_load_pct"]*start)/(duration-start)
            for key in result:
                if "losses" in key or key == "error_attempts":
                    result[key] -= baseline[key]
        return result


class CanBusSensors:
    """Sample continuously at plant resolution; transport only conversion events."""

    def __init__(self, sensor, **traffic):
        self.sensor = sensor
        # Assumed wire encoding: unsigned 12-bit FSR in two little-endian bytes;
        # bridge signed 24-bit two's complement in three bytes, without padding.
        traffic["adc_bytes"] = 2 if hasattr(sensor, "revision") else 3
        self.traffic = Traffic(**traffic)
        self.rate = sensor.rate / self.traffic.send_ms
        self.sensor.next_ready += self.traffic.force.phase
        self.code = sensor.code
        self.sample_index = 0
        self.sample_time = None

    def __getattr__(self, name):
        return getattr(self.sensor, name)

    def encoder(self, plant):
        return self.sensor.encoder(plant)

    def step(self, plant, time_s, stop=False, full=False):
        bus = self.traffic.bus
        bus.advance(time_s, inclusive=False)
        ready = self.sensor.step(plant, time_s, stop, full)
        if ready:
            self.sample_time = time_s
        # Sending is periodic even when ADC stops: send the held reading with its
        # original timestamp, so transport age exposes a failed acquisition node.
        node_tick = (time_s-self.traffic.force.phase)*1000
        if node_tick >= 1-1e-8 and abs(node_tick-round(node_tick)) < 1e-8:
            self.sample_index += 1
            if (self.sample_index-1) % self.traffic.send_ms == 0 and self.sample_time is not None:
                self.traffic.send_force(self.sample_time, self.sensor.code)
        self.traffic.new = False
        bus.advance(time_s)
        self.code = self.traffic.code
        return self.traffic.new


def latency_table(out):
    """Requested 20 s x 20 seeds; discard startup, report only (0.5, 20] s.

    ASSUMED open-loop force waveform: 0..40 N, 2 s sinusoid through the same
    2.2 kOhm DC FSR front end, rounded to 12-bit codes (no plant/RC/noise).
    """
    from .fsr import static_channel
    codes = [round(static_channel(20+20*math.sin(2*math.pi*i/2000), "fix")["ideal_code"])
             for i in range(20000)]
    rows = []
    for scenario in ("arm_bus", "shared_bus"):
        for bitrate in (500_000, 1_000_000):
            for priority in ("high", "low"):
                for send_ms in (1, 2, 5, 10, 20):
                    for ber, rx_drain in ((ber, drain) for ber in (0, 1e-4)
                                          for drain in (("isr", "tick") if scenario == "shared_bus"
                                                        else ("isr",))):
                        results, ages, held = [], [], []
                        for seed in range(20):
                            traffic = Traffic(scenario, bitrate, priority, send_ms, ber, seed,
                                              rx_drain=rx_drain)
                            phase = traffic.force.phase
                            for i in range(0, 20000, send_ms):
                                t = (i+1)/1000 + phase
                                if t > 20:
                                    break
                                traffic.bus.schedule(t, lambda i=i, t=t: traffic.send_force(t, codes[i]))
                            traffic.bus.advance(.5)
                            baseline = traffic.stats(.5)
                            traffic.bus.advance(20)
                            results.append(traffic.stats(20, start=.5, baseline=baseline))
                            window_ages = [a for t, a in zip(traffic.held_times, traffic.held_ages)
                                           if t > .5]
                            ages.extend(window_ages)
                            held.extend(window_ages)
                        row = dict(scenario=scenario, bitrate=bitrate, priority=priority,
                                   send_ms=send_ms, ber=ber, rx_drain=rx_drain,
                                   window_start_s=.5, window_end_s=20,
                                   duration_s=19.5, phase_seeds=20)
                        for key in results[0]:
                            values = [r[key] for r in results if r[key] is not None]
                            row[key] = (max(values) if key.endswith("max") or key.endswith("max_ms")
                                        or key == "worst_arbitration_wait_ms" else
                                        min(values) if key.endswith("min") else
                                        sum(values) if "losses" in key or key in ("error_attempts", "bus_off_nodes")
                                        else float(np.mean(values))) if values else None
                        row.update(age_mean_ms=float(np.mean(ages)) if ages else None,
                                   age_p99_ms=float(np.percentile(ages, 99)) if ages else None,
                                   held_age_mean_ms=float(np.mean(held)) if held else None)
                        rows.append(row)
                        print("can_latency", scenario, bitrate, priority, send_ms, ber, rx_drain,
                              flush=True)
    with (Path(out)/"can_latency.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Generate the open-loop CAN latency table without MuJoCo")
    parser.add_argument("out_dir", type=Path)
    output = parser.parse_args().out_dir
    output.mkdir(parents=True, exist_ok=True)
    latency_table(output)
