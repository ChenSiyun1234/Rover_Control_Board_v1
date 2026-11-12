"""Classical CAN timing and STM32G4 queues; times are seconds, no clock drift.

Sources: Bosch CAN 2.0 part A (frame, CRC, stuffing, arbitration, error flags):
https://www.port.de/fileadmin/user_upload/Dateien_IST_fuer_Migration/CAN20A.pdf
RM0440 rev 9, 44.3.6 and Rx/Tx handling (fixed 212-word RAM, three Tx
elements, two three-element Rx FIFOs; reset FnOM=0 discards the new frame):
https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
Tx queue mode is selected explicitly (lowest ID), rather than reset FIFO mode.
Assumptions: all receivers ACK; one active error flag (6 bits), delimiter
8 bits, intermission 3 bits = 17 recovery bits. A BER failure consumes a full
attempt before recovery (conservative late detection, not a fault-location model).
TEC increments by 8 on failure, decrements by 1 on success; bus-off at 256;
no automatic recovery. The eight-bit error-passive suspension applies to the
sender only, not bus occupancy. Receiver error counters and physical fault
locations/error-flag superposition are outside this timing model.
"""

from dataclasses import dataclass, field
from functools import lru_cache
import heapq
import random


def bits(value, width):
    return [(value >> i) & 1 for i in range(width - 1, -1, -1)]


def crc15(sequence):
    """Bosch zero-initialized, MSB-first CRC over unstuffed SOF through data."""
    crc = 0
    for bit in sequence:
        feedback = bit ^ (crc >> 14)
        crc = (crc << 1) & 0x7fff
        if feedback:
            crc ^= 0x4599
    return crc


def frame_sequence(can_id, data):
    """Return actual acknowledged bus bits, including three-bit intermission."""
    if not 0 <= can_id <= 0x7ff or len(data) > 8:
        raise ValueError("Classical 11-bit CAN, at most eight data bytes required")
    head = [0] + bits(can_id, 11) + [0, 0, 0] + bits(len(data), 4)
    for byte in data:
        head.extend(bits(byte, 8))
    raw = head + bits(crc15(head), 15)
    stuffed, last, run = [], None, 0
    for bit in raw:
        stuffed.append(bit)
        run = run + 1 if bit == last else 1
        last = bit
        if run == 5:
            last, run = 1 - bit, 1
            stuffed.append(last)
    return stuffed + [1, 0, 1] + [1] * 10


@lru_cache(maxsize=16384)
def frame_bits(can_id, data):
    return len(frame_sequence(can_id, data))


@dataclass(eq=False)
class Frame:
    can_id: int
    data: bytes
    sample: float | None = None
    queued: float = 0
    start: float | None = None
    end: float | None = None
    attempts: int = 0
    ready_at: float = 0
    arbitration_wait: float = 0


@dataclass
class Node:
    name: str
    filters: tuple = ()
    phase: float = 0
    tx: list = field(default_factory=list)
    rx: list = field(default_factory=lambda: [[], []])
    rx_losses: int = 0
    tx_losses: int = 0
    force_rx_losses: int = 0
    force_tx_losses: int = 0
    tec: int = 0
    bus_off: bool = False
    suspend_until: float = 0
    on_receive: object = None
    on_sent: object = None

    def drain(self):
        frames = self.rx[0] + self.rx[1]
        self.rx = [[], []]
        return frames


class CanBus:
    """Event-driven, non-preemptive arbitration with three hardware Tx slots."""

    ERROR_BITS = 17

    def __init__(self, bitrate=1_000_000, ber=0, seed=7):
        self.bitrate, self.ber = bitrate, ber
        self.rng = random.Random(seed)
        self.nodes, self.events, self.history = [], [], []
        self.serial = 0
        self.now = self.busy_time = self.worst_wait = 0.0
        self.active = False
        self.errors = self.bus_off_losses = 0

    def node(self, name, filters=(), phase=None):
        node = Node(name, tuple(filters), self.rng.random() * .001 if phase is None else phase)
        self.nodes.append(node)
        return node

    def schedule(self, when, callback, order=1):
        self.serial += 1
        heapq.heappush(self.events, (when, order, self.serial, callback))

    def send(self, node, frame):
        frame.queued = self.now
        frame.ready_at = self.now
        if node.bus_off or len(node.tx) == 3:
            node.tx_losses += 1
            node.force_tx_losses += frame.sample is not None
            return False
        node.tx.append(frame)
        self.schedule(self.now, lambda: None)
        return True

    def _arbitrate(self):
        offers = [(min(n.tx, key=lambda f: f.can_id), n) for n in self.nodes
                  if n.tx and not n.bus_off and n.suspend_until <= self.now + 1e-12]
        if not offers or self.active:
            return
        lowest = min(f.can_id for f, _ in offers)
        participants = [(f, n) for f, n in offers if f.can_id == lowest]
        winners = participants[:]
        # Equal IDs with differing data collide after arbitration; never invent
        # an ordering between misconfigured nodes with identical IDs. An
        # error-passive recessive-bit loser stops without destroying the winner;
        # an active loser's dominant error flag destroys the whole attempt.
        sequences = ({f: frame_sequence(f.can_id, f.data) for f, _ in participants}
                     if len(participants) > 1 else {})
        passive_losers = []
        collision = False
        for position in range(min((len(s) for s in sequences.values()), default=0)):
            zeros = [(f, n) for f, n in winners if sequences[f][position] == 0]
            ones = [(f, n) for f, n in winners if sequences[f][position] == 1]
            if zeros and ones:
                if any(n.tec < 128 for _, n in ones):
                    collision = True
                    break
                passive_losers.extend(ones)
                winners = zeros
            if len(winners) == 1:
                break
        frame = winners[0][0]
        count = frame_bits(frame.can_id, frame.data)
        corrupted = collision or (self.ber > 0 and self.rng.random() < 1 - (1 - self.ber)**count)
        duration = (count + (self.ERROR_BITS if corrupted else 0)) / self.bitrate
        start = self.now
        for f, _ in participants:
            f.attempts += 1
            if f.start is None:
                f.start = start
            wait = start-f.ready_at
            f.arbitration_wait += wait
            self.worst_wait = max(self.worst_wait, wait)
        self.active = True

        def finish():
            self.active = False
            if corrupted:
                self.errors += 1
            for f, node in participants if corrupted else passive_losers:
                f.ready_at = self.now
                node.tec += 8
                if node.tec >= 128:
                    node.suspend_until = self.now + 8/self.bitrate
                    self.schedule(node.suspend_until, lambda: None)
                if node.tec >= 256:
                    node.bus_off = True
                    self.bus_off_losses += len(node.tx)
                    node.tx_losses += len(node.tx)
                    node.force_tx_losses += sum(x.sample is not None for x in node.tx)
                    node.tx.clear()
            if corrupted:
                return
            for f, sender in winners:
                f.end = self.now - 3 / self.bitrate  # End of EOF; intermission is bus occupancy.
                sender.tx.remove(f)
                sender.tec = max(0, sender.tec - 1)
                if sender.tec >= 128:
                    sender.suspend_until = self.now + 8/self.bitrate
                    self.schedule(sender.suspend_until, lambda: None)
                self.history.append(f)
                if sender.on_sent:
                    sender.on_sent(f)
            # Delivery is scheduled at EOF below; hardware slot releases after intermission.

        if not corrupted:
            def receive():
                frame.end = self.now
                for node in self.nodes:
                    if any(node is n for _, n in participants) or node.bus_off:
                        continue
                    for fifo, accepted in enumerate(node.filters):
                        if frame.can_id in accepted:
                            if len(node.rx[fifo]) == 3:
                                node.rx_losses += 1
                                node.force_rx_losses += frame.sample is not None
                            else:
                                node.rx[fifo].append(frame)
                                if node.on_receive:
                                    node.on_receive(frame)
                            break
            self.schedule(start + duration - 3 / self.bitrate, receive, 0)
        self.schedule(start + duration, finish, 0)
        self.busy_time += duration

    def advance(self, until, inclusive=True):
        """Process releases/drains at equal times before choosing the next winner."""
        while self.events and (self.events[0][0] <= until + 1e-12 if inclusive
                               else self.events[0][0] < until - 1e-12):
            when = self.events[0][0]
            self.now = when
            while self.events and self.events[0][0] <= when + 1e-12:
                while (self.events and self.events[0][0] <= when + 1e-12
                       and self.events[0][1] < 2):
                    heapq.heappop(self.events)[3]()
                self._arbitrate()
                if (self.events and self.events[0][0] <= when + 1e-12
                        and self.events[0][1] < 2):
                    continue  # Zero-duration transmission completes before a drain.
                while self.events and self.events[0][0] <= when + 1e-12:
                    heapq.heappop(self.events)[3]()
                self._arbitrate()
        self.now = until
        if inclusive:
            self._arbitrate()
