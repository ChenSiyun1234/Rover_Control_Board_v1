"""Fast bitstream, queue, error and firmware equivalence checks; no network."""

import math
import random
from types import SimpleNamespace

import numpy as np
import pytest

from sim.can_bus import CanBus, Frame, bits, crc15, frame_bits, frame_sequence
from sim.can_traffic import CanBusSensors, Traffic
from sim.fsr import FSRSensors
from sim.params import Params
from sim3d.cases import CAN_CONFIGS, LEGACY_CASES, run_case


def unstuff(sequence):
    raw, last, run, i = [], None, 0, 0
    while i < len(sequence):
        bit = sequence[i]
        raw.append(bit)
        run = run+1 if bit == last else 1
        last = bit
        i += 1
        if run == 5:
            assert sequence[i] == 1-bit
            last, run = sequence[i], 1
            i += 1
    return raw


def polynomial_remainder(sequence):
    """Independent long division, including the transmitted CRC, gives zero."""
    number = int("".join(map(str, sequence)), 2)
    while number.bit_length() > 15:
        number ^= 0xc599 << (number.bit_length()-16)
    return number


def test_published_crc_and_stuffing():
    # Manufacturer's published classical frame: ID 00A, DLC 1, data 81,
    # CRC 18DA; field order and stuffing illustrated on the same page.
    # https://wiki.seeedstudio.com/xiao_stm32c5_canfd_usage/
    raw = unstuff(frame_sequence(0x00a, b"\x81")[:-13])
    assert raw[:27] == [0] + bits(0x00a, 11) + [0, 0, 0] + bits(1, 4) + bits(0x81, 8)
    assert raw[-15:] == bits(0x18da, 15)
    assert polynomial_remainder(raw) == 0
    # Ken Tindell's hardware-tested example: both these complete frames have
    # exactly THREE stuff bits (not merely a CRC algorithm check string).
    # https://kentindell.github.io/2020/01/20/new-can-hacks/
    for data in (b"\x01", b"\xf0"):
        sequence = frame_sequence(0x123, data)
        raw = unstuff(sequence[:-13])
        assert len(sequence)-13-len(raw) == 3
        assert polynomial_remainder(raw) == 0
        assert frame_bits(0x123, data) == 58  # 47 + 8 + 3, including intermission.


def test_frame_bounds_and_payload_bits():
    rng = random.Random(17)  # Assumed repeatable coverage seed.
    for dlc in range(9):
        for _ in range(100):
            data = rng.randbytes(dlc)
            sequence = frame_sequence(rng.randrange(2048), data)
            # Davis et al. 2007, Eq. 1: g=34, base 47+8*DLC incl intermission.
            # https://www.cs.york.ac.uk/rts/publications/Davis2007a.html
            assert 47+8*dlc <= len(sequence) <= 47+8*dlc + (34+8*dlc-1)//4
            assert polynomial_remainder(unstuff(sequence[:-13])) == 0
            assert sequence[-13:] == [1, 0, 1] + [1]*10
    assert frame_bits(0, bytes(8)) != frame_bits(0, b"\x55"*8)


def test_arbitration_queue_and_no_preemption():
    bus = CanBus()
    a, b = bus.node("a"), bus.node("b")
    bus.send(a, Frame(0x500, bytes(8)))
    bus.send(a, Frame(0x400, bytes(8)))
    bus.send(b, Frame(0x300, bytes(8)))
    bus.advance(0)
    bus.schedule(.000001, lambda: bus.send(a, Frame(1, b"")))
    bus.advance(.002)
    assert [f.can_id for f in bus.history] == [0x300, 1, 0x400, 0x500]
    assert bus.history[0].start == 0
    assert bus.history[1].start == pytest.approx(frame_bits(0x300, bytes(8))/1e6)


def test_load_equals_bits_times_rate():
    bus = CanBus()
    node = bus.node("tx")
    count, period = 100, .002  # Assumed test fixture: 500 Hz for 0.2 s.
    for i in range(count):
        bus.schedule(i*period, lambda: bus.send(node, Frame(0x123, b"\x01\x02")))
    bus.advance(count*period)
    measured = bus.busy_time/(count*period)
    expected = frame_bits(0x123, b"\x01\x02")/period/bus.bitrate
    assert measured == pytest.approx(expected, rel=.01)


def test_blocking_fifo_fourth_lost_and_filters():
    bus = CanBus()
    receiver = bus.node("rx", ([1, 2, 3, 4], [8]))
    for i in (1, 2, 3, 4, 8, 9):
        node = bus.node(str(i))
        bus.send(node, Frame(i, b""))
    bus.advance(.001)
    assert receiver.rx_losses == 1
    assert [f.can_id for f in receiver.rx[0]] == [1, 2, 3]
    assert [f.can_id for f in receiver.rx[1]] == [8]
    assert len(receiver.drain()) == 4
    assert receiver.rx == [[], []]


def test_tx_full_counts_loss_including_active_buffer():
    bus = CanBus()
    node = bus.node("tx")
    for i in range(3):
        assert bus.send(node, Frame(i, b""))
    bus.advance(0)
    assert not bus.send(node, Frame(10, b""))
    assert node.tx_losses == 1


def test_error_retries_and_adds_recovery_time():
    bus = CanBus(ber=1e-4)
    draws = iter([0, 1, 1])  # Force the first attempt to fail, subsequent attempts succeed.
    bus.rng = SimpleNamespace(random=lambda: next(draws))
    node = bus.node("tx", phase=0)
    frame = Frame(0x123, b"\x01")
    bus.send(node, frame)
    bus.advance(.001)
    assert frame.attempts == 2 and bus.errors == 1 and node.tx_losses == 0
    assert bus.busy_time == pytest.approx((2*frame_bits(frame.can_id, frame.data)+17)/1e6)
    assert frame.end == pytest.approx(bus.busy_time-3e-6)


def test_failed_frame_arbitrates_again():
    bus = CanBus(ber=1e-4)
    draws = iter([0, 1, 1])
    bus.rng = SimpleNamespace(random=lambda: next(draws))
    a, b = bus.node("a", phase=0), bus.node("b", phase=0)
    bus.send(a, Frame(0x500, b""))
    bus.advance(0)
    bus.schedule(.000001, lambda: bus.send(b, Frame(1, b"")))
    bus.advance(.001)
    assert [f.can_id for f in bus.history] == [1, 0x500]


def test_same_id_collision_retries_and_passive_loser_yields():
    bus = CanBus()
    a, b = bus.node("a"), bus.node("b")
    bus.send(a, Frame(0, b"a"))
    bus.send(b, Frame(0, b"b"))
    bus.advance(.010)
    assert not a.bus_off and not b.bus_off
    assert bus.errors == 16
    assert [f.data for f in bus.history] == [b"a", b"b"]
    assert a.tec == 127 and b.tec == 135


def test_persistent_errors_go_bus_off_and_count_pending_loss():
    bus = CanBus(ber=1)
    node = bus.node("failed")
    bus.send(node, Frame(1, b""))
    bus.advance(.010)
    assert node.bus_off and node.tec == 256
    assert bus.bus_off_losses == node.tx_losses == 1
    assert not bus.history


def test_passive_suspend_does_not_block_other_nodes():
    bus = CanBus(ber=1e-4)
    draws = iter([0, 1, 1])
    bus.rng = SimpleNamespace(random=lambda: next(draws))
    a, b = bus.node("passive", phase=0), bus.node("active", phase=0)
    a.tec = 120  # Assumed fixture: next failure enters error-passive state.
    bus.send(a, Frame(1, b""))
    bus.send(b, Frame(2, b""))
    bus.advance(.002)
    assert [f.can_id for f in bus.history] == [2, 1]
    first_end = (frame_bits(1, b"")+17)/1e6
    assert bus.history[0].start == pytest.approx(first_end)
    assert bus.busy_time == pytest.approx((2*frame_bits(1, b"")+frame_bits(2, b"")+17)/1e6)


def test_node_phase_sample_send_drain_and_age():
    sensor = CanBusSensors(FSRSensors(Params(), "fix"), phase=.00035, send_ms=2,
                           background=False)
    plant = SimpleNamespace(force=1)
    ready = []
    for i in range(1, 71):
        if sensor.step(plant, i/10000):
            ready.append(i/10000)
    rows = sensor.traffic.records
    assert sensor.traffic.force.phase == .0004
    assert ready == [.002, .004, .006]
    assert [r["sample_s"] for r in rows] == [.0014, .0034, .0054]
    assert all(r["age_ms"] == pytest.approx(.6) for r in rows)
    assert all(r["sample_s"] <= r["frame_start_s"] < r["frame_end_s"] <= r["use_s"] for r in rows)
    stats = sensor.traffic.stats(.007)
    assert stats["age_max_ms"] == pytest.approx(1.6)
    assert stats["delivery_age_max_ms"] == pytest.approx(.6)


@pytest.mark.parametrize("base", ["arm_jam_fsr_fix", "rack_jam"])
def test_infinite_empty_can_matches_unwrapped(base):
    direct, expected = run_case(base)
    trace = direct.trace.copy()
    wrapped, actual = run_case(base, can_options=dict(bitrate=math.inf, phase=0,
                                                    send_ms=1, background=False))
    assert actual == expected
    np.testing.assert_array_equal(wrapped.trace, trace)


def test_legacy_numbers_preserved():
    assert len(LEGACY_CASES) == 42 and len(CAN_CONFIGS) == 12
    assert LEGACY_CASES[22] == "bus_arm_0ms"
    assert LEGACY_CASES[28] == "bus_rack_0ms"
    assert LEGACY_CASES[-1] == "arm_jam_fsr_fix_spice_shgnd"


def test_rack_can_uses_bridge_without_changing_local_encoder():
    from sim.sensors import Sensors
    sim, summary = run_case("can_rack_1ms")
    assert sim.sensors.encoder(sim.plant) == Sensors(sim.parameters).encoder(sim.plant)
    assert summary["fault_flags"] == 1
    assert sim.sensors.sensor.model == "ADS1220_1000"
    assert sim.sensors.sensor.rate == 1000
    assert summary["force_payload_bytes"] == 3
    assert all(len(f.data) == 3 for f in sim.sensors.traffic.force_frames)
    assert summary["stale_timeout_ms"] == 5


def test_distinct_motor_feedback_and_processing_after_eof():
    traffic = Traffic("shared_bus")
    traffic.bus.advance(.035)  # Assumed fixture: allow the third 10 ms burst to finish.
    commands = [f for f in traffic.bus.history if 0x201 <= f.can_id <= 0x206]
    replies = [f for f in traffic.bus.history if 0x011 <= f.can_id <= 0x016]
    assert len(commands) == len(replies) == 18
    assert {f.can_id for f in replies} == set(range(0x011, 0x017))
    for n in range(1, 7):
        for command, reply in zip([f for f in commands if f.can_id == 0x200+n],
                                  [f for f in replies if f.can_id == 0x010+n]):
            assert reply.queued == pytest.approx(command.end + .0001, abs=1e-12)
    assert traffic.bus.errors == 0
    assert all(not n.bus_off for n in traffic.bus.nodes)
    assert sum(n.rx_losses+n.tx_losses for n in traffic.bus.nodes) == 0


@pytest.mark.parametrize("rx_drain, losses", [("isr", 0), ("tick", 1)])
def test_main_receive_interrupt_or_tick_fifo_overflow(rx_drain, losses):
    traffic = Traffic(background=False, rx_drain=rx_drain)
    for can_id in range(0x011, 0x015):
        node = traffic.bus.node(str(can_id))
        traffic.bus.send(node, Frame(can_id, b""))
    traffic.bus.advance(.0009)
    assert traffic.main.rx_losses == losses
    assert len(traffic.main.rx[1]) == (0 if rx_drain == "isr" else 3)
    traffic.bus.advance(.001)
    assert traffic.main.rx == [[], []]


def test_isr_updates_pending_reading_before_firmware_tick():
    traffic = Traffic(background=False, phase=0)
    traffic.send_force(0, 123)
    traffic.bus.advance(.0009)
    assert traffic.main.rx == [[], []]
    assert traffic.pending is traffic.force_frames[0]
    assert not traffic.records and traffic.code == 0
    traffic.bus.advance(.001)
    assert traffic.code == 123 and traffic.new
    assert traffic.records[0]["use_s"] == .001


@pytest.mark.parametrize("code", [-8388608, -1, 0, 8388607])
def test_bridge_payload_signed_24_bit_roundtrip(code):
    traffic = Traffic(background=False, adc_bytes=3)
    traffic.send_force(0, code)
    traffic.bus.advance(.001)
    assert len(traffic.force_frames[0].data) == 3
    assert traffic.code == code
