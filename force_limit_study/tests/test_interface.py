"""Verify UART framing, command eligibility and documented hex examples."""

import random
import re
import struct
from pathlib import Path

import pytest

from host.axis_client import Reader, crc16, decode_status, encode
from host.axis_client.identifiers import Command, State
from host.axis_client.parameters import PARAMS
from sim.sil import Firmware


@pytest.fixture
def firmware():
    """Fw."""
    return Firmware()


def exchange(firmware, cmd, payload=b"", seq=17):
    """Exchange."""
    firmware.write(encode(seq, cmd, payload))
    return Reader().feed(firmware.read())


def ready(firmware):
    """Ready."""
    exchange(firmware, 2)
    for t in range(1, 61):
        firmware.step(t)


def test_crc_matches_ccitt_false_vector(firmware):
    """Check crc. Specification 01."""
    assert firmware.crc(b"123456789") == crc16(b"123456789") == 10673


def test_ping_echoes_sequence_and_versions(firmware):
    """Check ping. Specification 02."""
    result = exchange(firmware, 1, seq=73)[0]
    assert (result.seq, result.cmd, struct.unpack("<BH", result.payload)) == (
        73,
        129,
        (1, 256),
    )


def test_corrupt_crc_is_dropped_without_command_execution(firmware):
    """Check crc rejection. Specification 03."""
    b = bytearray(encode(1, 2))
    b[-1] ^= 1
    firmware.write(b)
    assert (
        not firmware.read()
        and firmware.debug.crc_error_count == 1
        and (firmware.debug.state == State.DISABLED)
    )


def test_parser_recovers_after_garbage_and_bad_length(firmware):
    """Check resync. Specification 04."""
    firmware.write(b"garbage\xa5junk\xa5\xa5Z\xff" + encode(9, 1))
    assert Reader().feed(firmware.read())[0].seq == 9


def test_parser_accepts_fragmented_and_concatenated_frames(firmware):
    """Check chunking. Specification 05."""
    for b in encode(1, 1):
        firmware.write(bytes([b]))
    firmware.write(encode(2, 1) + encode(3, 1))
    assert [x.seq for x in Reader().feed(firmware.read())] == [1, 2, 3]


def test_incomplete_frame_expires_after_interbyte_timeout(firmware):
    """Check timeout. Specification 06."""
    firmware.write(encode(1, 1)[:5])
    firmware.step(21)
    firmware.write(encode(2, 1))
    assert [x.seq for x in Reader().feed(firmware.read())] == [2]
    assert firmware.debug.frame_drop_count == 1


def test_oversize_frame_is_dropped(firmware):
    """Check length. Specification 07."""
    firmware.write(b"\xa5ZA")
    assert not firmware.read() and firmware.debug.frame_drop_count == 1


def test_unknown_command_and_bad_length_return_nacks(firmware):
    """Check nack. Specification 08."""
    assert exchange(firmware, 102)[0].payload == bytes((102, 1))
    assert exchange(firmware, 1, b"x")[0].payload == bytes((1, 2))


def test_request_and_response_code_space(firmware):
    """Reserved response codes cannot collide with or act as requests."""
    requests = [cmd for cmd in Command if cmd not in (Command.TELEMETRY, Command.NACK)]
    for cmd in requests:
        assert 0x01 <= cmd <= 0x7D
        assert cmd | 0x80 not in (Command.TELEMETRY, Command.NACK)
    for cmd in (Command.TELEMETRY, Command.NACK):
        frame = exchange(firmware, cmd)[0]
        assert (frame.seq, frame.cmd, frame.payload) == (17, Command.NACK, bytes((cmd, 1)))


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -1, 1001])
def test_numeric_validation_precedes_state_validation(firmware, value):
    """Check range and state. Specification 09."""
    assert exchange(firmware, 16, struct.pack("<f", 1))[0].payload == b"\x10\x05"
    assert exchange(firmware, 16, struct.pack("<f", value))[0].payload == b"\x10\x03"
    assert exchange(firmware, 64, struct.pack("<Bf", 3, value))[0].payload == b"@\x03"
    ready(firmware)
    firmware.step(61, adc_code=8388607)
    assert exchange(firmware, 16, struct.pack("<f", 10))[0].payload == b"\x10\x04"


def test_parameters_roundtrip_and_lock_during_motion(firmware):
    """Check params. Specification 10."""
    for parameter_id, (_, default, _, _, _) in enumerate(PARAMS, 1):
        assert (
            exchange(firmware, 64, struct.pack("<Bf", parameter_id, default))[0].cmd
            == 192
        )
        result = exchange(firmware, 65, bytes((parameter_id,)))[0]
        actual_id, value = struct.unpack("<Bf", result.payload)
        assert actual_id == parameter_id and value == pytest.approx(default)
    for cmd, parameters in [(65, b"\xff"), (64, struct.pack("<Bf", 0, 1))]:
        assert exchange(firmware, cmd, parameters)[0].payload == bytes((cmd, 6))
    ready(firmware)
    assert exchange(firmware, 16, struct.pack("<f", 100))[0].cmd == 144
    for parameter_id, (_, value, *_) in enumerate(PARAMS, 1):
        assert (
            exchange(firmware, 64, struct.pack("<Bf", parameter_id, value))[0].payload
            == b"@\x04"
        )


def test_telemetry_rate_and_status_encoding(firmware):
    """Check stream. Specification 11."""
    exchange(firmware, 80, struct.pack("<H", 10))
    result = Reader()
    frames = []
    for t in range(1, 1001):
        firmware.step(t)
        frames += result.feed(firmware.read())
    assert 98 <= len(frames) <= 102
    for frame in frames:
        debug = decode_status(frame.payload)
        assert (
            frame.cmd == Command.TELEMETRY
            and debug["state"] == 0
            and (debug["fault_flags"] == 0)
            and (abs(debug["force_N"]) < 0.1)
        )
        assert 0 < debug["now_ms"] <= 1000


@pytest.mark.parametrize("heartbeat", [False, True])
def test_heartbeat_prevents_motion_watchdog_fault(firmware, heartbeat):
    """Check watchdog. Specification 12."""
    ready(firmware)
    exchange(firmware, 16, struct.pack("<f", 100))
    for t in range(61, 301):
        if heartbeat and t % 50 == 0:
            exchange(firmware, 1)
        firmware.step(t)
    assert firmware.debug.state == (2 if heartbeat else 5)
    assert bool(firmware.debug.fault_flags & 8) == (not heartbeat)


def test_random_frames_preserve_parser_and_state_invariants(firmware):
    """Check fuzz. Specification 13."""
    rng = random.Random(413)
    reader = Reader()
    now = 0

    def check(data):
        """Check."""
        nonlocal now
        firmware.write(data)
        out = firmware.read()
        frames = reader.feed(out)
        assert b"".join((encode(f.seq, f.cmd, f.payload) for f in frames)) == out
        assert not reader.buffer
        assert firmware.debug.state in (0, 1)
        now += 1
        firmware.step(now)

    for _ in range(5000):
        check(rng.randbytes(rng.randrange(0, 80)))
    now += 21
    firmware.step(now)
    for _ in range(2000):
        cmd = rng.randrange(256)
        payload = rng.randbytes(rng.randrange(65))
        if cmd in (2, 16, 18) and (not payload):
            payload = b"x"
        check(encode(rng.randrange(256), cmd, payload))


def test_documented_hex_examples_match_codec():
    """Check document examples. Specification 14."""
    text = Path("docs/INTERFACE.md").read_text()
    examples = re.findall(
        "<!-- frame seq=(\\d+) cmd=(\\d+) payload=([0-9a-f]*) -->"
        "\\s*```hex\\s*([\\s0-9A-Fa-f]+)```",
        text,
    )
    assert len(examples) >= 4
    for seq, cmd, payload, hexbytes in examples:
        b = bytes.fromhex(hexbytes)
        result = Reader()
        frames = result.feed(b)
        assert len(frames) == 1 and (not result.buffer)
        assert (frames[0].seq, frames[0].cmd, frames[0].payload) == (
            int(seq),
            int(cmd),
            bytes.fromhex(payload),
        )
        assert encode(int(seq), int(cmd), bytes.fromhex(payload)) == b


def test_stop_retract_tare_and_disable(firmware):
    """Check stop retract tare and disable."""
    ready(firmware)
    assert exchange(firmware, 19)[0].cmd == 147
    assert exchange(firmware, 16, struct.pack("<f", 100))[0].payload == b"\x10\x04"
    for t in range(61, 111):
        firmware.step(t)
    assert exchange(firmware, 16, struct.pack("<f", 100))[0].cmd == 144
    assert exchange(firmware, 19)[0].payload == b"\x13\x04"
    assert exchange(firmware, 17)[0].cmd == 145 and firmware.debug.state == State.IDLE
    assert (
        exchange(firmware, 18, struct.pack("<f", 2))[0].cmd == 146
        and firmware.debug.state == State.RETRACTING
    )
    firmware.step(111, encoder_counts=-2000)
    assert firmware.debug.state == State.IDLE
    assert exchange(firmware, 3)[0].cmd == 131
    firmware.step(112, encoder_counts=-2000)
    assert (
        firmware.debug.state == State.DISABLED
        and (not firmware.out.drive_enable)
        and (firmware.out.current_cmd_A == 0)
    )


def test_disable_cannot_bypass_latched_fault(firmware):
    """Check disable cannot bypass latched fault."""
    ready(firmware)
    firmware.step(61, adc_code=8388607)
    assert firmware.debug.fault_flags & 4
    exchange(firmware, 3)
    assert exchange(firmware, 2)[0].payload == b"\x02\x04"
    assert exchange(firmware, 32)[0].payload == b" \x04"
    firmware.step(62)
    assert (
        exchange(firmware, 32)[0].cmd == 160
        and firmware.debug.state == State.IDLE
        and (not firmware.debug.fault_flags)
    )
