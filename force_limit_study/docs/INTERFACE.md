# Insertion axis UART interface, version 1

UART: 115200 baud, 8 data bits, no parity, one stop bit. Firmware tick: 1 ms.
All integers are unsigned unless specified; all multi-byte fields are little-endian.
Floating-point fields use IEEE-754 binary32. Positions are millimetres.

## Framing

`A5 5A | LEN u8 | SEQ u8 | CMD u8 | PAYLOAD LEN bytes | CRC u16 LE`

LEN counts payload bytes only, maximum 64. CRC-16/CCITT-FALSE covers LEN,
SEQ, CMD and PAYLOAD: polynomial 0x1021, initial 0xFFFF, no reflection,
xorout 0. Check vector ASCII `123456789` gives 0x29B1.
The byte-wise parser silently drops invalid CRCs and increments crc_error_count.
A corrupt frame has no trustworthy sequence or command to acknowledge.
LEN > 64 increments frame_drop_count. Inter-byte gaps > 20 ms discard incomplete
frames and increment frame_drop_count. It then searches for A5 5A; stray A5 bytes
are tolerated. Bytes within an in-progress legal-length payload are data, including
sync bytes. After a bad CRC, buffered bytes are rescanned for the next header. Counters in telemetry wrap modulo 65536.
A bounded 8191-byte transmit queue drops whole replies on overflow and increments
frame_drop_count; draining it regularly is the caller's responsibility.

Every valid CRC frame refreshes the communication watchdog, including NACKs.
Request CMDs are 0x01-0x7D; ACK CMDs are request CMD | 0x80 (0x81-0xFD) and echo SEQ.
NACK CMD is 0x7F; unsolicited TELEMETRY CMD is 0x7E with SEQ=0. Both are reserved from requests.
NACK payload is `original CMD u8, error u8`. No command deduplication: retries can repeat actions.

## Commands

| CMD | Name | Request payload | ACK payload |
|---|---|---|---|
| 0x01 | PING | empty | protocol_version u8=1, fw_version u16=0x0100 |
| 0x02 | ENABLE | empty | empty |
| 0x03 | DISABLE | empty | empty |
| 0x10 | START_INSERT | target_mm f32, (0,1000] | empty |
| 0x11 | STOP | empty | empty |
| 0x12 | RETRACT | distance_mm f32, (0,1000] | empty |
| 0x13 | TARE | empty | empty |
| 0x14 | HOME | empty | empty |
| 0x20 | CLEAR_FAULT | empty | empty |
| 0x30 | GET_STATUS | empty | status block below |
| 0x40 | SET_PARAM | id u8, value f32 | empty |
| 0x41 | GET_PARAM | id u8 | id u8, value f32 |
| 0x50 | STREAM | period_ms u16, 0 off, otherwise 1..65535 | empty |

ENABLE requires DISABLED, no latched faults, and a closed safety chain. It starts a stationary
50-conversion tare; motion is rejected until it completes. TARE requires IDLE and
speed <0.5 mm/s. START_INSERT and RETRACT require IDLE or SEATED. START_INSERT
from DISABLED returns NOT_ENABLED; from other states returns WRONG_STATE.
STOP captures the present holding position from MOVING/RETRACTING, or succeeds
in IDLE. DISABLE removes torque at the next tick and aborts motion; fault flags
remain latched. CLEAR_FAULT requires FAULT/ESTOP (or DISABLED with latched faults), a closed safety chain,
a non-saturated fresh sensor, force below the current envelope, and completed
backoff if active; it returns IDLE. Tare is not automatically repeated by clearing.
An E-stop still open overrides DISABLE on the next tick.
Changes to all parameters are rejected during MOVING and RETRACTING.
The framing layer delegates SET_PARAM/GET_PARAM to the parameter metadata table;
parameter meanings and bounds are owned by firmware/src/parameters.c. The matching
Python wire-ID and default/range table is in host/axis_client/.
Ranges are inclusive; NaN and infinities are rejected. Envelope enable is exactly
0 or 1. Invalid numeric inputs are checked before state eligibility.
The host must maintain consistent seat bounds and motion geometry; individual
parameter validation does not validate multi-parameter configurations.

## Errors and state values

Errors: UNKNOWN_CMD=1, BAD_LENGTH=2, OUT_OF_RANGE=3, WRONG_STATE=4,
NOT_ENABLED=5, BAD_PARAM_ID=6.
States: DISABLED=0, IDLE=1, MOVING=2, SEATED=3, RETRACTING=4, FAULT=5, ESTOP=6, HOMING=7, HOME_BACKOFF=8.
Fault bitmask: OVERFORCE=0x01, SENSOR_STALE=0x02, SENSOR_RANGE=0x04,
COMM_TIMEOUT=0x08, ESTOP=0x10, WIRING=0x20, HOME_TIMEOUT=0x40. Faults latch until eligible CLEAR_FAULT.

## Status block (31 bytes)

| Byte offset | Field | Encoding |
|---|---|---|
| 0 | state | u8 |
| 1 | fault flags | u16 |
| 3 | carriage_mm | f32 |
| 7 | server_est_mm | f32 |
| 11 | vel_mm_s | f32 |
| 15 | force_N | f32 |
| 19 | current_A | f32 |
| 23 | now_ms | u32 |
| 27 | crc_error_count | u16 |
| 29 | frame_drop_count | u16 |

STREAM emits this block once per period (1 ms granularity). Call fw_step once
per millisecond; rx/tx calls must be serialized with it. The MCU adapter must
serialize UART delivery with the control ISR or use its own interrupt-safe queue.

## Parameters

Defaults are firmware values. The SIL host configures calibration, stiffness and
stale timeout from its sensor configuration before enabling. Timeout is
max(5, ceil(3000 / ADC_rate)) ms; it measures time since the latest ready sample.

| ID | Name | Default | Allowed range | Unit |
|---|---|---|---|---|
| 1 | lc_counts_per_N | 10485.76 | 1..1000000.0 | counts/N |
| 2 | pusher_stiffness_N_per_mm | 200 | 1..1000 | N/mm |
| 3 | free_limit_N | 60 | 1..500 | N |
| 4 | mate_limit_N | 150 | 1..500 | N |
| 5 | seat_force_N | 110 | 1..500 | N |
| 6 | mate_start_mm | 278 | 0..1000 | mm |
| 7 | seat_lo_mm | 293 | 0..1000 | mm |
| 8 | seat_hi_mm | 297 | 0..1000 | mm |
| 9 | approach_speed_mm_s | 100 | 1..200 | mm/s |
| 10 | insert_speed_mm_s | 10 | 1..100 | mm/s |
| 11 | slow_zone_mm | 270 | 0..1000 | mm |
| 12 | force_filter_hz | 0 | 0..500 | Hz |
| 13 | stale_timeout_ms | 5 | 5..5000 | ms |
| 14 | backoff_mm | 2 | 0..20 | mm |
| 15 | comm_timeout_ms | 200 | 20..5000 | ms |
| 16 | envelope_enabled | 1 | 0..1 | bool |
| 17 | max_current_A | 8 | 0.1..8 | A |
| 18 | accel_mm_s2 | 500 | 10..3000 | mm/s^2 |
| 19 | velocity_kp | 0.035 | 0..1 | A/(mm/s) |
| 20 | velocity_ki | 0.12 | 0..10 | A/mm |
| 21 | velocity_filter_hz | 80 | 1..500 | Hz |
| 22 | backoff_speed_mm_s | 10 | 1..50 | mm/s |
| 23 | counts_per_mm | 819.2 | 1..100000 | counts/mm |
| 24 | position_kp | 5 | 0.1..50 | 1/s |
| 25 | force_sensor | 0 | 0..1 | enum |
| 26 | fsr_pulldown_ohm | 1000000 | 1..10000000 | ohm |
| 27 | fsr_a | 17394.59495 | 1..10000000 | ohm*N^b |
| 28 | fsr_b | 0.8503824633 | 0.1..3 | 1 |
| 29 | adc_full_scale | 4095 | 2..65535 | counts |
| 30 | adc_vref | 3.3 | 0.1..5 | V |
| 31 | drive_mode | 0 | 0 or 1 | enum |
| 32 | duty_kp | 0.002041666667 | 0..1 | duty/(mm/s) |
| 33 | duty_ki | 0.007 | 0..10 | duty/mm |
| 34 | wiring_check_mm_s | 2 | 0.1..200 | mm/s |
| 35 | wiring_check_ms | 50 | 1..5000 | ms |
| 36 | home_speed_mm_s | 5 | 1..50 | mm/s |
| 37 | home_debounce_ms | 3 | 1..100 | ms |
| 38 | home_backoff_mm | 1 | 0.01..20 | mm |
| 39 | home_max_travel_mm | 60 | 1..1000 | mm |

`drive_mode=0` retains the current PI, its gains and current clamp. Mode 1 uses
independent duty PI gains and anti-windup at signed duty [-1,1]; max_current_A
does not limit duty or motor current. Both modes share the motion profile.
While MOVING or RETRACTING, opposite signs of measured filtered velocity and
the actual reference, both magnitudes strictly above wiring_check_mm_s for
strictly longer than wiring_check_ms, latch WIRING. Any interrupted condition
resets the window. Assumed defaults of 2 mm/s and 50 ms reject near-zero ripple
and brief reversals well beyond the default velocity filter's ~2 ms time constant.
They require hardware commissioning. Two reversed signs can escape this check.

In mode 1, overforce braking commands duty 0 while enabled until the existing
speed threshold (<2 mm/s) permits the existing reverse/hold backoff phases.
Zero duty means zero average armature voltage and passive dynamic braking in
this model, with electrical current decay and back-EMF drag; it does not mean
instant stop or zero current. Sensor/communication/wiring faults command duty 0
without backoff. Wiring faults suppress the PI in both modes because feedback
direction cannot be trusted. E-stop/DISABLE remove drive enable; the model
assumes ideal open circuit then, omitting switching transients. Actual driver
pin timing and brake/coast behavior must be verified on hardware.

`force_sensor` is exactly 0 (bridge, default) or 1 (FSR divider). In FSR mode,
the firmware computes V = code / adc_full_scale * adc_vref,
R = fsr_pulldown_ohm * (adc_vref / V - 1), F = (fsr_a / R)^(1/fsr_b).
Code <= 0 returns zero. Codes >= adc_full_scale are assigned the largest finite
representable force: the inverse at adc_full_scale - 1. Thus the maximum is
`(fsr_a * (adc_full_scale - 1) / fsr_pulldown_ohm)^(1/fsr_b)`.
Quantisation and clipping can therefore hide further increases in true force.
The existing bridge signed-24-bit range check remains; full-scale FSR codes
are converted using this cap rather than creating a separate range trip.
Stationary tare averages converted newtons before subtracting the offset;
the force filter, envelope, seating and backoff logic are unchanged.

The FSR defaults assume an Interlink FSR 402, not an identified team part.
The log-log fit uses the typical curve from the official
[FSR 400 Series datasheet](https://www.interlinkelectronics.com/downloads/datasheets/fsr-400-series-datasheet.pdf),
page 3; digitised points and the fit are in `sim/fsr.py`. The specified range is
0.2-20 N; values above 20 N are assumed extrapolations. Hysteresis and drift
are ignored. ADC excitation, VDDA and VREF+ are assumed 3.3 V.
The current PCB (5db1aab) has a 1 MOhm pull-down and 10 nF shunt capacitor.
The previous PCB (e9beafa) has the same divider followed by 10 kOhm and 100 nF;
Q4 is assumed on with negligible resistance during operation. These channels
have identical DC calibration but different force-dependent RC time constants.

## C integration

Call fw_init once, then serialize fw_step, fw_uart_rx, fw_uart_tx and
fw_get_debug. The public inputs, outputs and debug structures use the native C
ABI; they are not wire packets. Encode telemetry fields explicitly as documented
above rather than sending structure bytes. The DLL contains one controller
instance, so a second initialization resets it.

`fw_outputs_t` fields in order are `float current_cmd_A`, `bool drive_enable`,
`float pwm_duty` (0..1), `uint8_t dir` (1 positive, 0 negative; zero duty uses 1).
Native alignment/padding applies; the ctypes wrapper matches it. Mode 0 zeros
the PWM fields (DIR=1); mode 1 zeros current_cmd_A and debug/status current_A.
The existing 31-byte UART status layout stays unchanged: current_A remains a
current command, never a measured motor current or a duty value. PWM/DIR are
adapter outputs and additional SIL trace columns, not new status bytes.
The MCU adapter maps these outputs to the board's PWM/DIR pins.

The SIL DLL also exports fw_probe_reference(float reference_mm_s), compiled only
with AXIS_SIL. This experiment hook directly replaces the moving velocity reference
to isolate the PI from trajectory acceleration. It is not a UART command and
is absent from the ARM build. It does not bypass force, E-stop or watchdog checks.

## Complete frame examples

These examples are decoded and checked by pytest.

PING request:

<!-- frame seq=7 cmd=1 payload= -->
```hex
A5 5A 00 07 01 2A 45
```

PING reply:

<!-- frame seq=7 cmd=129 payload=010001 -->
```hex
A5 5A 03 07 81 01 00 01 B9 2B
```

START_INSERT request (296 mm):

<!-- frame seq=8 cmd=16 payload=00009443 -->
```hex
A5 5A 04 08 10 00 00 94 43 98 95
```

START_INSERT NOT_ENABLED reply:

<!-- frame seq=8 cmd=127 payload=1005 -->
```hex
A5 5A 02 08 7F 10 05 A3 77
```

GET_PARAM request (free limit):

<!-- frame seq=9 cmd=65 payload=03 -->
```hex
A5 5A 01 09 41 03 7B 62
```

GET_PARAM reply (60 N):

<!-- frame seq=9 cmd=193 payload=0300007042 -->
```hex
A5 5A 05 09 C1 03 00 00 70 42 D1 27
```


Unsolicited telemetry (zeroed status block):

<!-- frame seq=0 cmd=126 payload=00000000000000000000000000000000000000000000000000000000000000 -->
```hex
A5 5A 1F 00 7E 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 2B EC
```

## Limit-switch homing

HOME (host `Client.home()`) has the same enable, tare and starting-state requirements
as START_INSERT. HOMING seeks in the negative encoder direction at home_speed_mm_s
with the existing acceleration limit. Logical active samples must persist for
ceil(home_debounce_ms) consecutive 1 ms ticks; any inactive sample resets the count.
The controller then commands a zero reference to brake. Below 0.5 mm/s (the existing
stationary threshold), it sets the current encoder coordinate to zero, enters
HOME_BACKOFF and moves positively by home_backoff_mm. Completion requires position
within one encoder count, speed below 0.5 mm/s and an inactive switch; it enters IDLE
holding the backoff target. Encoder velocity uses raw differences, so changing the
origin does not introduce a velocity impulse. The origin persists across subsequent
commands, DISABLE and fault clearing; fw_init resets it.

Assumed defaults: 5 mm/s for cautious switch approach; 3 ms to reject the assumed
2 ms closing bounce; 1 mm to clear the switch; 60 mm to allow twice the assumed
30 mm initial distance. If accumulated absolute encoder travel exceeds this search
limit before debounce succeeds, HOME_TIMEOUT latches and suppresses both PI outputs
without automatic backoff. This is a travel bound, not an elapsed-time watchdog;
a stalled axis without encoder travel is not detected by this flag.

STOP and DISABLE abort either homing state. Parameter writes lock, communication
supervision and the wiring check remain active in both homing states; force and
E-stop checks remain active as usual. Both current and PWM drive modes share this
flow. HOME_TIMEOUT requires the normal CLEAR_FAULT eligibility checks.

`fw_inputs_t` fields in order: uint32_t now_ms, int32_t encoder_counts,
int32_t adc_code, bool adc_new, bool estop_ok, bool limit_switch. limit_switch is
logical active, not the pin level. The assumed arm adapter uses a normally-open
contact to ground with a pull-up, so it inverts the active-low pin. The board's
100 Ohm series resistors alone do not establish switch type or pull-up provision.
The simulated base switch closes at hand arc travel <= -30 mm (assumed 30 mm behind
the start pose), with 2 ms seeded closing bounce (assumed seed 41). It uses physical
turntable travel independently of encoder sign and motor polarity.

Case metrics define zero_error_mm as the true coordinate assigned firmware zero
minus limit_at_mm; negative means beyond the switch. zero_error_counts multiplies
that signed error by the motor encoder counts/mm. stopped_past_switch_mm measures the deepest physical travel past the switch
during braking/reversal; it can differ slightly from the coordinate latched below
the stationary threshold. time_to_switch_s ends at the first physical closure, before bounce;
peak current covers the entire seek, braking and backoff cycle. The CSV adds
firmware_carriage_mm and limit_switch after all existing columns.

With the unchanged arm geometry and default 60 mm search bound, the double swap
reaches the drawer before reaching HOME_TIMEOUT and trips OVERFORCE instead.
The existing fault braking/backoff remains active; actual post-trip hand travel is
reported separately. The fast timeout test uses an explicitly assumed 30 mm
search bound, before drawer contact; the reported default case retains 60 mm.
