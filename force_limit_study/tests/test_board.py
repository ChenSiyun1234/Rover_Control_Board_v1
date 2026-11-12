"""Board extraction and case E regressions; no hardware or network required."""
from itertools import combinations
import os
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from board.af_table import PINS
from board.check_timers import analyze, compatible, load, render, solutions
from board.pin_map import extract, parse

# Folder holding rev_<commit>/single_board.kicad_pcb exports of the team's board repository.
# The KiCad files are not in this repository, so the extraction test needs this variable.
INPUTS = Path(os.environ.get("BOARD_KICAD_DIR", ""))
EXPECTED = {
    "5db1aab": {
        "PWM": "97:PE0 98:PE1 3:PE4 17:PC2 22:PA2 25:PA3",
        "DIR": "90:PB3 96:PB9 4:PE5 18:PC3 19:PF2 26:PA4",
        "ENC": "91:PB4 92:PB5 93:PB6 94:PB7 1:PE2 2:PE3 15:PC0 16:PC1",
        "LIM": "43:PE12 44:PE13 45:PE14",
        "OTHER": {"ENC_5": ("21", "PA1"), "ENC_6": ("20", "PA0"), "FSR_ADC": ("41", "PE10"), "LASER_SIG": ("42", "PE11")},
        "drivers": "J7 J9 J10 J14 J19 J22", "encoders": "J6 J8 J11 J15 J18 J21",
    },
    "e9beafa": {
        "PWM": "17:PC2 25:PA3 4:PE5 67:PC8 62:PD15 98:PE1",
        "DIR": "18:PC3 26:PA4 5:PE6 68:PC9 61:PD14 97:PE0",
        "ENC": "15:PC0 16:PC1 20:PA0 21:PA1 28:PA6 29:PA7 59:PD12 60:PD13 65:PC6 66:PC7 1:PE2 2:PE3",
        "LIM": "41:PE10 42:PE11 43:PE12",
        "OTHER": {"FSR_ADC": ("44", "PE13"), "FSR_SIG": ("45", "PE14"), "LASER_SIG": ("46", "PE15")},
        "drivers": "J38 J45 J46 J47 J48 J49", "encoders": "J39 J40 J41 J42 J43 J44",
    },
}


@pytest.mark.parametrize("revision", EXPECTED)
def test_expected_nets_and_pins(revision):
    expected = EXPECTED[revision]
    pins = dict(expected["OTHER"])
    for prefix in ("PWM", "DIR", "LIM"):
        for i, entry in enumerate(expected[prefix].split(), 1):
            pins[("LIM_SW" if prefix == "LIM" else prefix) + str(i)] = tuple(entry.split(":"))
    for i, entry in enumerate(expected["ENC"].split()):
        pins[f"ENC{i // 2 + 1}_{'AB'[i % 2]}"] = tuple(entry.split(":"))
    rows = load(ROOT / "board" / f"pins_{revision}.csv")
    assert {r["signal"]: (r["mcu_pin"], r["port"]) for r in rows} == pins
    assert len(rows) == len(pins)
    indexed = {r["signal"]: r for r in rows}
    for i, connector in enumerate(expected["drivers"].split(), 1):
        assert (indexed[f"PWM{i}"]["connector"], indexed[f"PWM{i}"]["connector_pin"]) == (connector, "3")
        assert (indexed[f"DIR{i}"]["connector"], indexed[f"DIR{i}"]["connector_pin"]) == (connector, "4")
    for i, connector in enumerate(expected["encoders"].split(), 1):
        for suffix in ("_A", "_B") if f"ENC{i}_A" in indexed else ("single",):
            name = f"ENC_{i}" if suffix == "single" else f"ENC{i}{suffix}"
            expected_pin = "2" if suffix == "single" else str((2 if revision == "5db1aab" else 1) + (suffix == "_B"))
            assert (indexed[name]["connector"], indexed[name]["connector_pin"]) == (connector, expected_pin)
    assert set(r["port"] for r in rows) <= PINS.keys()


@pytest.mark.parametrize("revision", EXPECTED)
def test_extraction_matches_csv(revision):
    if not (INPUTS / f"rev_{revision}").is_dir() or not os.environ.get("BOARD_KICAD_DIR"):
        pytest.skip("set BOARD_KICAD_DIR to the folder with the KiCad exports")
    assert extract(INPUTS / f"rev_{revision}") == load(ROOT / "board" / f"pins_{revision}.csv")


@pytest.mark.parametrize("revision", EXPECTED)
def test_case_e_report_and_minimal_conflicts(revision):
    rows = load(ROOT / "board" / f"pins_{revision}.csv")
    assert render(rows, revision) == (ROOT / "board" / f"check_{revision}.txt").read_text(encoding="utf-8")
    result = analyze(rows)
    assert not result["feasible"] and not result["assignments"]
    expected_cores = {
        "5db1aab": {("ENC1", "ENC3", "PWM3"), ("ENC1", "ENC3", "ENC4", "PWM4")},
        "e9beafa": {("ENC4", "PWM5"), ("ENC3", "ENC6", "PWM3"), ("ENC1", "ENC3", "ENC6", "PWM1"), ("ENC1", "ENC3", "PWM1", "PWM3"), ("ENC1", "ENC6", "PWM1", "PWM3")},
    }
    assert set(result["cores"]) == expected_cores[revision]
    for core in result["cores"]:
        assert next(solutions(result["tasks"], core), None) is None
        for subset in combinations(core, len(core) - 1):
            assert next(solutions(result["tasks"], subset), None) is not None
    assert result["exti_ok"]
    assert [e for _, _, e in result["limits"]] == ([12, 13, 14] if revision == "5db1aab" else [10, 11, 12])
    assert result["adc"] == ([("ADC3", 14), ("ADC4", 14), ("ADC5", 14)] if revision == "5db1aab" else [("ADC3", 3)])
    assert len(result["warnings"]) == (2 if revision == "5db1aab" else 0)


def test_timer_counter_and_complementary_channel_constraints():
    encoder = (("A", "PC0", "TIM1", "CH1", 2), ("B", "PC1", "TIM1", "CH2", 2))
    pwm = (("PWM", "PE13", "TIM1", "CH3", 2),)
    complement = (("PWM", "PE12", "TIM1", "CH3N", 2),)
    free = (("PWM", "PE14", "TIM1", "CH4", 2),)
    assert not compatible(encoder, pwm)
    assert not compatible(pwm, complement)
    assert compatible(pwm, free)
    assert list(solutions({"pwm": [pwm], "free": [free]}, ["pwm", "free"]))


def test_sexpression_quoted_parentheses():
    assert parse('(root (value "a (b) \\"c\\""))') == ["root", ["value", 'a (b) "c"']]
