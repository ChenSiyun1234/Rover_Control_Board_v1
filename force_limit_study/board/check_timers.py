"""Case E: exhaustively assign hardware timers to the extracted interfaces."""
import argparse
import csv
from itertools import combinations, product
from pathlib import Path

try:
    from .af_table import ENCODER_TIMERS, PINS
except ImportError:
    from af_table import ENCODER_TIMERS, PINS


def load(path):
    with Path(path).open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def label(option):
    return ", ".join(f"{net} {port} {timer}_{channel} AF{af}" for net, port, timer, channel, af in option)


def compatible(a, b):
    ta, tb = a[0][2], b[0][2]
    if ta != tb:
        return True
    # An encoder owns its counter; complementary and main PWM share CCRx.
    return len(a) == len(b) == 1 and a[0][3].rstrip("N") != b[0][3].rstrip("N")


def problem(rows):
    signals = {r["signal"]: r for r in rows}
    tasks, warnings = {}, []
    for ch in range(1, 7):
        a, b = f"ENC{ch}_A", f"ENC{ch}_B"
        if a in signals and b in signals:
            pa, pb = signals[a]["port"], signals[b]["port"]
            tasks[f"ENC{ch}"] = [
                ((a, pa, t, ca, fa), (b, pb, u, cb, fb))
                for t, ca, fa in PINS[pa]["timers"]
                for u, cb, fb in PINS[pb]["timers"]
                if t == u and t in ENCODER_TIMERS and {ca, cb} == {"CH1", "CH2"}
            ]
        else:
            single = signals.get(f"ENC_{ch}")
            if single:
                warnings.append(f"ENC{ch}: {single['signal']} {single['port']} is a single-wire input. The team notes "
                                f"describe it as a PWM-output absolute encoder, read by timer input capture.")
            else:
                tasks[f"ENC{ch}"] = []
        name = f"PWM{ch}"
        port = signals[name]["port"]
        tasks[name] = [((name, port, t, c, af),) for t, c, af in PINS[port]["timers"]]
    return dict(sorted(tasks.items())), warnings


def solutions(tasks, names):
    for choices in product(*(tasks[name] for name in names)):
        if all(compatible(a, b) for a, b in combinations(choices, 2)):
            yield dict(zip(names, choices))


def analyze(rows):
    tasks, warnings = problem(rows)
    names = list(tasks)
    assignments = list(solutions(tasks, names))
    # All inclusion-minimal impossible sets, including higher-order shortages.
    cores = []
    if not assignments:
        for size in range(1, len(names) + 1):
            for subset in combinations(names, size):
                if any(set(core) <= set(subset) for core in cores):
                    continue
                if next(solutions(tasks, subset), None) is None:
                    cores.append(subset)
    fsr = next(r for r in rows if r["signal"] == "FSR_ADC")
    limits = [(r["signal"], r["port"], PINS[r["port"]]["exti"]) for r in rows if r["signal"].startswith("LIM_SW")]
    return dict(tasks=tasks, warnings=warnings, assignments=assignments, cores=cores,
                adc=PINS[fsr["port"]]["adc"], fsr=fsr["port"], limits=limits,
                exti_ok=len({x[2] for x in limits}) == len(limits),
                feasible=bool(assignments) and not warnings)


def render(rows, revision):
    result = analyze(rows)
    tasks = result["tasks"]
    lines = [f"Case E: {revision}", f"All encoder direction decoding and PWM simultaneously: {'PASS' if result['feasible'] else 'FAIL'}",
             f"Timer allocation for paired encoders and PWM: {len(result['assignments'])} valid assignments.",
             "Single-wire (PWM-output absolute) encoders are excluded from quadrature allocation and listed separately.",
             "All Cartesian-product assignments searched; complementary outputs included.",
             "PWM channels on a free-running shared timer may share frequency; independent frequencies are not required here.",
             "No runtime, signal-integrity or hardware validation is implied.", "", "Candidates:"]
    for name, options in tasks.items():
        lines.append(name + ": " + (" OR ".join(label(o) for o in options) or "no CH1/CH2 encoder pair or PWM output"))
    lines += ["", *result["warnings"]]
    if result["assignments"]:
        lines += ["Valid assignment:"] + [label(o) for o in result["assignments"][0].values()]
    for core in result["cores"]:
        lines.append("Unavoidable conflict set: " + " + ".join(core))
        for name in core:
            lines.append("  " + name + ": " + " OR ".join(label(o) for o in tasks[name]))
        lines.append("  No mutually compatible choice; an encoder reserves the whole timer, and PWM outputs cannot share a CCR channel.")
    lines += ["", f"FSR_ADC {result['fsr']}: " + ", ".join(f"{a}_IN{c}" for a, c in result["adc"]),
              "Limit switches: " + ", ".join(f"{n} {p} EXTI{e}" for n, p, e in result["limits"]),
              f"Distinct EXTI lines: {'PASS' if result['exti_ok'] else 'FAIL'}", ""]
    return "\n".join(lines)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = render(load(args.csv), args.csv.stem.removeprefix("pins_"))
    if args.output:
        args.output.write_text(report, encoding="utf-8")
    else:
        print(report, end="")
