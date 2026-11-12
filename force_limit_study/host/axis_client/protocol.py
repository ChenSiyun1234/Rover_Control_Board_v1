"""Independent little-endian UART codec."""

import struct
from dataclasses import dataclass


def crc16(data):
    """Return CRC-16/CCITT-FALSE over the supplied byte sequence."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


@dataclass(frozen=True)
class Frame:
    """A validated sequence, command and payload from one UART frame."""

    seq: int
    cmd: int
    payload: bytes


def encode(seq, cmd, payload=b""):
    """Serialize one bounded UART frame with a little-endian CRC."""
    if len(payload) > 64:
        raise ValueError("Payload exceeds 64 bytes")
    body = bytes((len(payload), seq, cmd)) + payload
    return b"\xa5Z" + body + struct.pack("<H", crc16(body))


class Reader:
    """Recover valid frames from fragmented or noisy byte streams."""

    def __init__(self):
        """Initialize the instance."""
        self.buffer = bytearray()

    def feed(self, data):
        """Consume new bytes and return any complete CRC-valid frames."""
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= 2:
            if self.buffer[:2] != b"\xa5Z":
                del self.buffer[0]
                continue
            if len(self.buffer) < 3:
                break
            payload_length = self.buffer[2]
            if payload_length > 64:
                del self.buffer[0]
                continue
            if len(self.buffer) < payload_length + 7:
                break
            frame = bytes(self.buffer[: payload_length + 7])
            if crc16(frame[2:-2]) != struct.unpack("<H", frame[-2:])[0]:
                del self.buffer[0]
                continue
            frames.append(Frame(frame[3], frame[4], frame[5:-2]))
            del self.buffer[: payload_length + 7]
        return frames


STATUS_FORMAT = "<BHfffffIHH"


def decode_status(payload):
    """Unpack the protocol version 1 telemetry payload into named fields."""
    return dict(
        zip(
            (
                "state",
                "fault_flags",
                "carriage_mm",
                "server_est_mm",
                "vel_mm_s",
                "force_N",
                "current_A",
                "now_ms",
                "crc_error_count",
                "frame_drop_count",
            ),
            struct.unpack(STATUS_FORMAT, payload),
        )
    )
