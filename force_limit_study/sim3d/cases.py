"""Sequential firmware experiments and replay artifacts for both MuJoCo fixtures."""

import argparse
import csv
from functools import partial
import json
from pathlib import Path

import numpy as np

from host.axis_client.identifiers import Fault, State
from sim.sil import Sim
from sim.bus import BusSensors
from sim.can_traffic import CanBusSensors, latency_table
from sim.sensors import Sensors
from sim.fsr import FSR_A, FSR_B, FRONT_ENDS, FSRSensors, static_channel
from sim.trace import TraceColumn as T
from .mj_plant import DCMotor, MjPlant
from .scene import ARM

# Assumed arm-scale firmware configuration; mechanical dimensions set the windows.
ARM_CONTROL = {
    # The base drive gives at most ARM.max_tip_force (about 58 N) at the hand.
    "free_limit_N": 15,  # Assumed: about a quarter of the tip force, above 4 N drawer friction.
    "mate_limit_N": 35,  # Assumed: about 60 % of the tip force, above the 20 N latch.
    "seat_force_N": 25,  # Assumed: closed-drawer detection between latch and limit.
    "approach_speed_mm_s": 30,  # Assumed cautious free tool travel speed.
    "insert_speed_mm_s": 10,  # Assumed slow engagement, same as rack default.
    "mate_start_mm": (ARM.gap+ARM.end-ARM.latch_length)*1000-2,
    # Assumed 2 mm envelope margin before the latch, allowing force-estimate noise.
    "seat_lo_mm": (ARM.gap+ARM.end)*1000-3,
    "seat_hi_mm": (ARM.gap+ARM.end)*1000+2,
    # Assumed -3/+2 mm window around the geometric stop. The hand swings on an arc, so by
    # the end of travel the pad is turned about 0.08 rad and its edge meets the drawer about
    # 1.2 mm before its centre would; -3 mm leaves room for that and the pad compliance.
    "slow_zone_mm": ARM.gap*1000-2,  # Assumed slow down 2 mm before first contact.
    "pusher_stiffness_N_per_mm": ARM.contact_k/1000,
    "counts_per_mm": ARM.counts_per_m/1000,  # Motor-shaft encoder mapped to tool arc.
    "accel_mm_s2": 100,  # Assumed gentle arm acceleration, one fifth of rack default.
    "backoff_mm": 2,  # Assumed existing safety unloading distance retained.
}

FSR_CASES = ("arm_nominal_fsr_now", "arm_jam_fsr_now",
             "arm_nominal_fsr_prev", "arm_jam_fsr_prev",
             "arm_jam_fsr_now_joint_enc", "arm_jam_fsr_now_joint_enc_lowpass",
             "arm_nominal_fsr_fix", "arm_jam_fsr_fix")
# Rigid arm approximation: M = J_yaw/r^2 + drawer mass and
# K_F = Kt*gear*efficiency/r. With L/R = 0.714 ms neglected at loop bandwidth,
# i = (12*duty - Ke*gear*v/r)/R, hence
# M*s^2 + (B_emf + K_F*12/R*Kp)*s + K_F*12/R*Ki,
# B_emf = K_F*Ke*gear/(R*r). As in DESIGN.md, match the current-PI
# feedback stiffness/damping terms: Kp_d = Kp_I*R/12, Ki_d = Ki_I*R/12.
# Back-EMF adds damping to this assumed starting tune; cycle time also depends
# on friction/contact/filter dynamics, all of which remain in the run.
PWM_CONTROL = dict(drive_mode=1, duty_kp=0.035*DCMotor.resistance_ohm/DCMotor.bus_V,
                   duty_ki=0.12*DCMotor.resistance_ohm/DCMotor.bus_V)
PWM_CASES = ("arm_nominal_fsr_fix_pwm", "arm_jam_fsr_fix_pwm")
WIRING_CASES = ("wiring_ok", "wiring_motor_swapped", "wiring_encoder_swapped",
                "wiring_both_swapped")
HOME_CASES = ("home_5", "home_20", "home_both_swapped")
# Requested bus sweep, appended after all existing case numbers. Loss is an
# independent 5% per-conversion assumption, with the wrapper's default seed 7.
BUS_CONFIGS = {
    f"bus_{family}_{suffix}": (base, delay, drop)
    for family, base in (("arm", "arm_jam_fsr_fix"), ("rack", "rack_jam"))
    for suffix, delay, drop in (
        *((f"{d}ms", d, 0) for d in (0, 1, 5, 10, 20)),
        ("10ms_drop5", 10, 0.05),
    )
}
BUS_CASES = tuple(BUS_CONFIGS)
SPICE_CASES = ("arm_nominal_fsr_now_spice", "arm_jam_fsr_now_spice",
               "arm_nominal_fsr_now_spice_shgnd", "arm_jam_fsr_now_spice_shgnd",
               "arm_jam_fsr_prev_spice", "arm_nominal_fsr_fix_spice",
               "arm_jam_fsr_fix_spice", "arm_jam_fsr_fix_spice_shgnd")
CASES = (("rack_nominal", "rack_jam", "rack_jam_hx711", "arm_nominal", "arm_jam")
         + FSR_CASES + PWM_CASES + WIRING_CASES + HOME_CASES + BUS_CASES + SPICE_CASES)
LEGACY_CASES = CASES
# Requested additions follow all 42 existing names and numbers. Assumed timeout
# tolerates one missed send plus one main-board drain; firmware minimum is 5 ms.
CAN_CONFIGS = {
    **{f"can_arm_{s}ms": ("arm_jam_fsr_fix", "arm_bus", s, "high", max(5, 2*s+1))
       for s in (1, 2, 5, 10, 20)},
    "can_arm_10ms_stale5": ("arm_jam_fsr_fix", "arm_bus", 10, "high", 5),
    **{f"can_shared_{s}ms_low": ("arm_jam_fsr_fix", "shared_bus", s, "low", max(5, 2*s+1))
       for s in (5, 10)},
    **{f"can_rack_{s}ms": ("rack_jam", "arm_bus", s, "high", max(5, 2*s+1))
       for s in (1, 5, 10, 20)},
}
CASES += tuple(CAN_CONFIGS)


def run_case(name, one_d=False, *, can_options=None):
    """Run through seating or through trip and the existing 1.5 s backoff interval."""
    can = CAN_CONFIGS.get(name)
    if can is not None:
        name, scenario, send_ms, priority, stale = can
    bus = BUS_CONFIGS.get(name)
    if bus is not None:
        name, delay_ms, drop = bus
    if name in HOME_CASES:
        return run_home(name)
    wiring = name in WIRING_CASES
    case = "arm" if wiring else name.split("_")[0]
    jam = "jam" in name
    config = {"jam": jam, "adc": "HX711_10" if "hx711" in name else "ADS1220_1000"}
    if not one_d:
        config["plant_factory"] = partial(MjPlant, case=case)
    if case == "arm":
        # Keep the physical diagnostic contact threshold in module coordinates.
        config["physical"] = {"pusher_k": ARM.contact_k/1000, "x_jam": ARM.jam_at*1000}
        config["params"] = ARM_CONTROL
    else:
        config["params"] = {"accel_mm_s2": 500}  # Existing experiment default.
    if "fsr" in name or wiring:
        revision = "prev" if "prev" in name else "fix" if "fix" in name or wiring else "now"
        counts = ARM.counts_per_m / (ARM.gear if "joint_enc" in name else 1)
        config["plant_factory"] = partial(MjPlant, case=case, counts_per_m=counts)
        config["sensor_factory"] = partial(FSRSensors, revision=revision)
        if name in SPICE_CASES:
            from sim.fsr_spice import SpiceFSRSensors
            config["sensor_factory"] = partial(SpiceFSRSensors, revision=revision,
                                                sh_start="ground" if "shgnd" in name else "previous")
        config["params"] = dict(ARM_CONTROL, force_sensor=1,
                                fsr_pulldown_ohm=FRONT_ENDS[revision].pulldown_ohm,
                                fsr_a=FSR_A, fsr_b=FSR_B, adc_full_scale=4095,
                                adc_vref=3.3, counts_per_mm=counts/1000)
        if "lowpass" in name:
            # Assumed 5 Hz: at 10 mm/s the joint encoder yields only ~18 counts/s;
            # 32 ms filter time smooths count impulses vs 2 ms at the default 80 Hz.
            # Only this filter changes relative to the unfiltered joint case.
            config["params"]["velocity_filter_hz"] = 5
    if name in PWM_CASES or wiring:
        config["params"].update(PWM_CONTROL)
    if wiring:
        motor_polarity = -1 if name in ("wiring_motor_swapped", "wiring_both_swapped") else 1
        encoder_sign = -1 if name in ("wiring_encoder_swapped", "wiring_both_swapped") else 1
        config["plant_factory"] = partial(MjPlant, case="arm", motor_polarity=motor_polarity,
                                          encoder_sign=encoder_sign)
    if bus is not None:
        factory = config.get("sensor_factory", partial(Sensors, model=config["adc"]))
        config["sensor_factory"] = lambda parameters: BusSensors(
            factory(parameters), delay_ms=delay_ms, drop=drop)
    if can is not None:
        config["params"] = dict(config["params"], stale_timeout_ms=stale)
        can_options = dict(scenario=scenario, send_ms=send_ms, priority=priority)
    if can_options is not None:
        factory = config.get("sensor_factory", partial(Sensors, model=config["adc"]))
        options = can_options.copy()
        config["sensor_factory"] = lambda parameters: CanBusSensors(factory(parameters), **options)
    sim = Sim(config)
    if case == "rack":
        sim.run_insert()
    else:
        sim.prepare()
        target = (ARM.gap+ARM.end)*1000+2  # Assumed 2 mm target allowance beyond stop.
        sim.client.start_insert(target)
        start = sim.now_ms
        sim.insertion_started_ms = start
        for _ in range(12000):
            if sim.now_ms % 50 == 0:
                sim.client.ping()
            sim.step()
            if sim.firmware.debug.state in (State.SEATED, State.FAULT, State.ESTOP):
                break
        sim.cycle_time = (sim.now_ms-start)/1000
    trip = sim.firmware.debug.fault_ms
    terminal_trace = sim.trace
    motion_rows = terminal_trace[terminal_trace[:, T.STATE] == State.MOVING]
    motion_start = float(motion_rows[0, T.TIME_S]) if len(motion_rows) else None
    motion_origin = float(motion_rows[0, T.CARRIAGE_MM]) if len(motion_rows) else 0.0
    wrong_way = max(0.0, motion_origin-float(terminal_trace[:, T.CARRIAGE_MM].min()))
    true_force_at_trip = sim.plant.force if trip else None
    terminal_state = sim.firmware.debug.state
    trip_position = sim.plant.carriage_position_m*1000
    if jam or trip:
        sim.advance(1500, True)
    summary = sim.summary()
    summary.update(peak_force_N=sim.peak_true_force_N,
                   carriage_mm=sim.plant.carriage_position_m*1000,
                   residual_force_N=sim.plant.force,
                   backoff_mm=trip_position-sim.plant.carriage_position_m*1000 if jam or trip else None,
                   trip_after_contact_ms=(trip/1000-sim.first_contact_s)*1000
                   if trip and sim.first_contact_s is not None else None)
    trace = sim.trace
    moving = trace[trace[:, T.STATE] == State.MOVING]
    summary.update(
        peak_current_A=sim.peak_current_A,  # Absolute peak at 10 kHz, including backoff.
        true_force_at_trip_N=true_force_at_trip,
        firmware_force_at_trip_N=summary["detection_force_N"] if trip else None,
        true_peak_force_N=sim.peak_true_force_N,
        debris_to_trip_ms=summary["trip_after_contact_ms"] if jam else None,
        timed_out=terminal_state not in (State.SEATED, State.FAULT, State.ESTOP),
        outcome=("tripped" if trip else "seated" if terminal_state == State.SEATED else "time_out"),
        max_abs_filtered_velocity_mm_s=float(np.max(np.abs(trace[:, T.MEASURED_VELOCITY_MM_S]))),
        max_abs_current_A=float(np.max(np.abs(trace[:, T.CURRENT_COMMAND_A]))),
        moving_velocity_error_rms_mm_s=float(np.sqrt(np.mean(
            (moving[:, T.MEASURED_VELOCITY_MM_S]-moving[:, T.TRUE_VELOCITY_MM_S])**2))) if len(moving) else None,
    )
    if summary["timed_out"]:
        summary["timeout_reason"] = "No seat or trip within the 12 s insertion time limit."
    if "fsr" in name or wiring:
        summary.update(fsr_revision={"prev": "e9beafa", "now": "5db1aab",
                                     "fix": "5db1aab with R26 = 2.2 kOhm (proposed)"}[revision],
                       encoder_mounting="joint" if "joint_enc" in name else "motor",
                       counts_per_mm=counts/1000,
                       velocity_filter_hz=config["params"].get("velocity_filter_hz", 80),
                       adc_full_scale_fraction=float(np.mean(
                           np.array(sim.sensors.codes) == 4095)))
    if name in PWM_CASES or wiring:
        summary.update(PWM_CONTROL, bus_V=DCMotor.bus_V,
                       assumed_motor_R_ohm=DCMotor.resistance_ohm,
                       assumed_motor_L_H=DCMotor.inductance_H)
    # Assumed reporting margins: 2 +/-0.5 mm backoff and under 1 N residual contact.
    summary["ok"] = (summary["final_state"] == "SEATED" if not jam else
                     summary["final_state"] == "FAULT" and summary["fault_flags"] == 1
                     and 1.5 < summary["backoff_mm"] < 2.5 and summary["residual_force_N"] < 1)
    if wiring:
        full_trace = sim.trace
        wrong_way_total = max(0.0, motion_origin-float(full_trace[:, T.CARRIAGE_MM].min()))
        summary.update(motor_polarity=motor_polarity, encoder_sign=encoder_sign,
                       motion_to_trip_ms=(trip/1000-motion_start)*1000 if trip and motion_start is not None else None,
                       wrong_way_mm=wrong_way_total, wrong_way_at_trip_mm=wrong_way if trip else None,
                       post_trip_hand_travel_mm=sim.plant.carriage_position_m*1000-trip_position if trip else None,
                       backoff_mm=None,  # Wiring faults brake without automatic backoff.
                       wiring_tripped=bool(summary["fault_flags"] & Fault.WIRING))
        summary["ok"] = (summary["fault_flags"] == Fault.WIRING if motor_polarity != encoder_sign
                         else summary["fault_flags"] == 0)
    if can is not None:
        summary.update(sim.sensors.traffic.stats(sim.now_ms/1000))
        summary.update(can_scenario=scenario, send_ms=send_ms, force_priority=priority,
                       bitrate=1_000_000, phase_seed=7,
                       node_phase_ms=sim.sensors.traffic.force.phase*1000,
                       stale_timeout_ms=stale,
                       stale_timeout_rule="default 5 ms" if stale == 5 and send_ms == 10 else
                       "max(5, 2*send_ms + 1): firmware minimum; one missed send plus drain",
                       rx_drain="isr", force_payload_bytes=sim.sensors.traffic.adc_bytes,
                       force_sensor="FSR fix" if case == "arm" else "ADS1220_1000")
        if case == "arm":
            summary["fsr_pulldown_ohm"] = 2200
    return sim, summary



def run_home(name, **home_parameters):
    """Home against a physical switch, retaining edge, stopped zero and retreat metrics."""
    swapped = name == "home_both_swapped"
    speed = 20 if name == "home_20" else 5
    polarity = -1 if swapped else 1
    config = {
        "plant_factory": partial(MjPlant, case="arm", motor_polarity=polarity,
                                 encoder_sign=polarity),
        "sensor_factory": partial(FSRSensors, revision="fix"),
        "physical": {"pusher_k": ARM.contact_k/1000, "x_jam": ARM.jam_at*1000},
        "params": dict(ARM_CONTROL, **PWM_CONTROL, force_sensor=1,
                       fsr_pulldown_ohm=FRONT_ENDS["fix"].pulldown_ohm,
                       fsr_a=FSR_A, fsr_b=FSR_B, adc_full_scale=4095, adc_vref=3.3,
                       home_speed_mm_s=speed),
    }
    config["params"].update(home_parameters)
    sim = Sim(config)
    sim.prepare()
    sim.client.home()
    started = sim.now_ms
    zero_position = zero_counts = None
    seen_homing = False
    # Assumed run budget: 30 s exceeds 60 mm at 5 mm/s plus acceleration and retreat.
    for _ in range(30000):
        if sim.now_ms % 50 == 0:
            sim.client.ping()
        sim.step()
        state = sim.firmware.debug.state
        seen_homing |= state == State.HOMING
        if state == State.HOME_BACKOFF and zero_position is None:
            zero_position = sim.plant.carriage_position_m*1000
            zero_counts = sim.sensors.encoder(sim.plant)*polarity
        if state in (State.FAULT, State.ESTOP) or (seen_homing and state == State.IDLE):
            break
    sim.cycle_time = (sim.now_ms-started)/1000
    trip_position = sim.plant.carriage_position_m*1000
    if state == State.FAULT:
        sim.advance(1500, True)  # Existing case convention: observe fault braking/backoff.
    summary = sim.summary()
    switch = sim.plant.limit_at_mm
    zero_error = zero_position-switch if zero_position is not None else None
    summary.update(
        home_speed_mm_s=speed, limit_at_mm=switch,
        home_debounce_ms=sim.config["params"].get("home_debounce_ms", 3),
        home_backoff_mm=sim.config["params"].get("home_backoff_mm", 1),
        home_max_travel_mm=sim.config["params"].get("home_max_travel_mm", 60),
        bounce_ms=sim.plant.bounce_ms, trip_position_mm=trip_position if state == State.FAULT else None,
        post_trip_hand_travel_mm=sim.plant.carriage_position_m*1000-trip_position
        if state == State.FAULT else None,
        time_to_switch_s=sim.plant.first_switch_s-started/1000
        if sim.plant.first_switch_s is not None else None,
        stopped_past_switch_mm=switch-float(sim.trace[:, T.CARRIAGE_MM].min())
        if zero_position is not None else None,
        zero_true_position_mm=zero_position, zero_encoder_counts=zero_counts,
        zero_error_mm=zero_error,
        zero_error_counts=zero_error*ARM.counts_per_m/1000 if zero_error is not None else None,
        carriage_mm=sim.plant.carriage_position_m*1000,
        firmware_carriage_mm=sim.firmware.debug.carriage_mm,
        peak_current_A=sim.peak_current_A, motor_polarity=polarity, encoder_sign=polarity,
        drive_mode=config["params"]["drive_mode"], fsr_pulldown_ohm=FRONT_ENDS["fix"].pulldown_ohm,
        encoder_mounting="motor", counts_per_mm=ARM.counts_per_m/1000,
        outcome="home_timeout" if summary["fault_flags"] & Fault.HOME_TIMEOUT else
                "homed" if state == State.IDLE else "fault" if state == State.FAULT else "time_out",
        timed_out=state not in (State.IDLE, State.FAULT, State.ESTOP),
    )
    summary["ok"] = (summary["fault_flags"] == Fault.HOME_TIMEOUT if swapped else
                     state == State.IDLE and summary["fault_flags"] == 0)
    return sim, summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out_dir", type=Path)
    out = parser.parse_args().out_dir
    out.mkdir(parents=True, exist_ok=True)
    summaries = {}
    comparison = []
    bus_rows = []
    can_rows = []
    channel_rows = [dict(revision=rev, force_N=force, **static_channel(force, rev))
                    for rev in FRONT_ENDS for force in (0.2, 0.5, 1, 2, 5, 10, 15, 20, 35, 50)]
    with (out/"fsr_channel.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(channel_rows[0]))
        writer.writeheader()
        writer.writerows(channel_rows)
    for name in CASES:
        sim, summary = run_case(name)
        if name in BUS_CONFIGS:
            base, delay, drop = BUS_CONFIGS[name]
            family = base.split("_")[0]
            baseline = summary if delay == 0 else summaries[f"bus_{family}_0ms"]
            overshoot = summary["true_peak_force_N"] - baseline["true_peak_force_N"]
            summary.update(family=family, delay_ms=delay, drop=drop,
                           overshoot_vs_0ms_N=overshoot)
            bus_rows.append(dict(
                family=family, delay_ms=delay, drop=drop,
                true_force_at_trip_N=summary["true_force_at_trip_N"],
                true_peak_force_N=summary["true_peak_force_N"],
                overshoot_vs_0ms_N=overshoot,
                contact_to_trip_ms=summary["trip_after_contact_ms"]))
        if name in CAN_CONFIGS:
            family = CAN_CONFIGS[name][0].split("_")[0]
            overshoot = summary["true_peak_force_N"] - summaries[f"bus_{family}_0ms"]["true_peak_force_N"]
            summary["overshoot_vs_0ms_N"] = overshoot
            can_rows.append(dict(case=name, **{key: summary[key] for key in (
                "final_state", "fault_flags", "true_force_at_trip_N", "true_peak_force_N",
                "overshoot_vs_0ms_N", "age_mean_ms", "age_max_ms", "held_age_max_ms",
                "rx_fifo_losses", "tx_losses", "bus_load_pct", "stale_timeout_ms")}))
            with (out/f"{name}_readings.csv").open("w", newline="", encoding="utf-8") as f:
                readings = sim.sensors.traffic.records
                writer = csv.DictWriter(f, fieldnames=list(readings[0]))
                writer.writeheader()
                writer.writerows(readings)
        summaries[name] = summary
        np.savetxt(out/f"{name}.csv", sim.trace, delimiter=",",
                   header=",".join(c.name.lower() for c in T), comments="")
        np.savez_compressed(out/f"{name}_qpos.npz",
                            time=np.array([t for t, _ in sim.plant.qpos_log]),
                            qpos=np.array([q for _, q in sim.plant.qpos_log]))
        print(name, json.dumps(summary), flush=True)
        if name.startswith("rack_jam"):
            del sim
            reference, reference_summary = run_case(name, one_d=True)
            np.savetxt(out/f"{name}_1d.csv", reference.trace, delimiter=",",
                       header=",".join(c.name.lower() for c in T), comments="")
            row = {"case": name}
            for label, result in (("1d", reference_summary), ("3d", summary)):
                for key in ("peak_force_N", "trip_after_contact_ms", "detection_force_N", "final_state", "peak_current_A", "fault_flags"):
                    row[f"{label}_{key}"] = result[key]
            comparison.append(row)
            del reference
        else:
            del sim
    with (out/"compare_1d.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(comparison[0]))
        writer.writeheader()
        writer.writerows(comparison)
    (out/"summary.json").write_text(json.dumps(summaries, indent=2), encoding="utf-8")
    with (out/"can_cases.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(can_rows[0]))
        writer.writeheader()
        writer.writerows(can_rows)
    with (out/"bus_delay.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(bus_rows[0]))
        writer.writeheader()
        writer.writerows(bus_rows)
    spice_rows = []
    fields = ("final_state", "fault_flags", "true_force_at_trip_N", "firmware_force_at_trip_N",
              "true_peak_force_N", "trip_after_contact_ms")
    for name in SPICE_CASES:
        twin = name.split("_spice")[0]
        for label in (twin, name):
            spice_rows.append(dict(case=label, **{key: summaries[label][key] for key in fields}))
    with (out/"fsr_spice.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(spice_rows[0]))
        writer.writeheader()
        writer.writerows(spice_rows)
    from sim.fsr_spice import static_rows, benchmark
    from sim.params import Params
    rows = list(static_rows(Params()))
    with (out/"fsr_spice_static.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    performance = benchmark(Params())
    (out/"fsr_spice_performance.json").write_text(json.dumps(performance, indent=2), encoding="utf-8")
    print("fsr_spice_performance", json.dumps(performance), flush=True)
    latency_table(out)


if __name__ == "__main__":
    main()
