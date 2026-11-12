"""Nonblocking client for transports exposing write(bytes) and read()."""

import struct

from .identifiers import Command
from .protocol import Reader, decode_status, encode


class Client:
    """Send nonblocking requests through a byte-stream transport."""

    def __init__(self, transport):
        """Initialize the instance."""
        self.transport = transport
        self.seq = 0
        self.reader = Reader()

    def request(self, cmd, payload=b""):
        """Frame a request and return its sequence number without waiting for an ACK."""
        self.seq = self.seq + 1 & 255
        self.transport.write(encode(self.seq, cmd, payload))
        return self.seq

    def frames(self):
        """Yield complete replies currently available from the transport."""
        yield from self.reader.feed(self.transport.read())

    def ping(self):
        """Request versions and refresh the communication watchdog."""
        return self.request(Command.PING)

    def enable(self):
        """Request drive enable and a stationary 50-sample tare."""
        return self.request(Command.ENABLE)

    def disable(self):
        """Request torque removal while retaining latched fault flags."""
        return self.request(Command.DISABLE)

    def start_insert(self, target_mm=296):
        """Request insertion toward the absolute server target in millimetres."""
        return self.request(Command.START_INSERT, struct.pack("<f", target_mm))

    def home(self):
        """Seek the negative limit, zero at rest, then back off into IDLE."""
        return self.request(Command.HOME)

    def stop(self):
        """Request a hold at the current carriage position."""
        return self.request(Command.STOP)

    def retract(self, distance_mm):
        """Request a relative backward carriage move in millimetres."""
        return self.request(Command.RETRACT, struct.pack("<f", distance_mm))

    def tare(self):
        """Request a new stationary load-cell zero in IDLE."""
        return self.request(Command.TARE)

    def clear_fault(self):
        """Request fault reset after the cause and any backoff have cleared."""
        return self.request(Command.CLEAR_FAULT)

    def status(self):
        """Request a single telemetry status block."""
        return self.request(Command.GET_STATUS)

    def set_param(self, parameter_id, value):
        """Request a parameter write by its stable wire ID."""
        return self.request(Command.SET_PARAM, struct.pack("<Bf", parameter_id, value))

    def get_param(self, parameter_id):
        """Request a parameter value by its stable wire ID."""
        return self.request(Command.GET_PARAM, bytes((parameter_id,)))

    def stream(self, period_ms):
        """Set the telemetry period in milliseconds; zero disables streaming."""
        return self.request(Command.STREAM, struct.pack("<H", period_ms))

    @staticmethod
    def status_fields(frame):
        """Decode the fields of a status reply or telemetry frame."""
        return decode_status(frame.payload)
