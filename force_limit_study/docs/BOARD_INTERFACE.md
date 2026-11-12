# Board interface

The MCU is STM32G474VBT6 in LQFP100. Connectivity and component values come from the PCB exports; MCU port names come from the schematic library symbol. Pin numbers below are physical U5 pins.

## Role assumptions

The six motor connectors are treated as paired, one connector for an encoder and one for an external motor controller. The driver interfaces are assumptions for this study: PWM + DIR for a DC driver and STEP + DIR for a stepper driver. They are not read from populated parts. The numbering is unspecified: channels 1-3 are assumed DC and 4-6 assumed stepper. On assumed stepper channels, the PCB net named PWM is treated as STEP.

## Reading the tables

Case E finds no full assignment in either revision. The peripheral column therefore lists eligible choices, not a chosen global assignment. DIR, FSR_SIG and LASER_SIG use GPIO; limits use EXTI. Encoder candidates include only matching CH1/CH2 pairs on an encoder-capable timer. A suffix N is a complementary PWM output and cannot serve as an encoder input. PWM outputs sharing a free-running timer may share a frequency; independent channel frequencies are outside this check.

Connector entries marked via a resistor are separate nets joined by that resistor. Entries marked switched are separate transistor output nodes, not direct MCU connections. The passive column lists only parts directly on the named signal net; surrounding circuitry is described below. Empty passive entries mean no resistor or capacitor directly on that net.

FT variants are specified as 5 V tolerant digital I/O; TT variants are only 3.6 V tolerant. This classification is not blanket permission to apply 5 V in analog mode or outside datasheet operating conditions. Encoder connectors provide 5V power, which does not establish encoder output voltage. PC1, PA0, PA1, PA6 and PA7 require particular attention if external encoder outputs are 5 V. PB4/PB6 have UCPD dead-battery pull-down behavior; PB4 also resets to a debug function (Table 12 notes 5-6, p. 72).

## Current revision (2026-11)

Export revision 5db1aab, dated 2026-11-02. No simultaneous assignment chosen.

| Channel | Assumed role | Connector.pin | Signal | MCU pin | Port | Peripheral / eligible timer channel | Notes / direct passives |
|---|---|---|---|---|---|---|---|
| 1 | DC | J7.3 | PWM1 | 97 | PE0 | TIM20_CH4N AF3 / TIM16_CH1 AF4 | - |
| 1 | DC | J7.4 | DIR1 | 90 | PB3 | GPIO output | - |
| 1 | DC | J6.2 | ENC1_A | 91 | PB4 | TIM3_CH1 AF2 | FT_c (5 V tolerant digital); Conflict set 1, 2 |
| 1 | DC | J6.3 | ENC1_B | 92 | PB5 | TIM3_CH2 AF2 | FT_f (5 V tolerant digital); Conflict set 1, 2 |
| 2 | DC | J9.3 | PWM2 | 98 | PE1 | TIM17_CH1 AF4 / TIM20_CH4 AF6 | - |
| 2 | DC | J9.4 | DIR2 | 96 | PB9 | GPIO output | - |
| 2 | DC | J8.2 | ENC2_A | 93 | PB6 | TIM4_CH1 AF2 | FT_c (5 V tolerant digital) |
| 2 | DC | J8.3 | ENC2_B | 94 | PB7 | TIM4_CH2 AF2 | FT_f (5 V tolerant digital) |
| 3 | DC | J10.3 | PWM3 | 3 | PE4 | TIM3_CH3 AF2 / TIM20_CH1N AF6 | Conflict set 1 |
| 3 | DC | J10.4 | DIR3 | 4 | PE5 | GPIO output | - |
| 3 | DC | J11.2 | ENC3_A | 1 | PE2 | TIM3_CH1 AF2 / TIM20_CH1 AF6 | FT (5 V tolerant digital); Conflict set 1, 2 |
| 3 | DC | J11.3 | ENC3_B | 2 | PE3 | TIM3_CH2 AF2 / TIM20_CH2 AF6 | FT (5 V tolerant digital); Conflict set 1, 2 |
| 4 | stepper | J14.3 | PWM4 | 17 | PC2 | TIM1_CH3 AF2 / TIM20_CH2 AF6 | Conflict set 2 |
| 4 | stepper | J14.4 | DIR4 | 18 | PC3 | GPIO output | - |
| 4 | stepper | J15.2 | ENC4_A | 15 | PC0 | TIM1_CH1 AF2 | FT_a (5 V tolerant digital); Conflict set 2 |
| 4 | stepper | J15.3 | ENC4_B | 16 | PC1 | TIM1_CH2 AF2 | TT_a (3.6 V tolerant; not 5 V tolerant); Conflict set 2 |
| 5 | stepper | J19.3 | PWM5 | 22 | PA2 | TIM2_CH3 AF1 / TIM5_CH3 AF2 / TIM15_CH1 AF9 | - |
| 5 | stepper | J19.4 | DIR5 | 19 | PF2 | GPIO output | - |
| 5 | stepper | J18.2 | ENC_5 | 21 | PA1 | TIM2_CH2 AF1 / TIM5_CH2 AF2 | Single wire: no direction decoding; TT_a (3.6 V tolerant; not 5 V tolerant) |
| 6 | stepper | J22.3 | PWM6 | 25 | PA3 | TIM2_CH4 AF1 / TIM5_CH4 AF2 / TIM15_CH2 AF9 | - |
| 6 | stepper | J22.4 | DIR6 | 26 | PA4 | GPIO output | - |
| 6 | stepper | J21.2 | ENC_6 | 20 | PA0 | TIM2_CH1 AF1 / TIM5_CH1 AF2 | Single wire: no direction decoding; TT_a (3.6 V tolerant; not 5 V tolerant) |
| - | - | J27.1 | FSR_ADC | 41 | PE10 | ADC3_IN14 / ADC4_IN14 / ADC5_IN14 | C32=10nF (1:/FSR_ADC; 2:GND); R26=1M (1:/FSR_ADC; 2:GND) |
| - | - | J30.1 | LASER_SIG | 42 | PE11 | GPIO output | via R35/Q1 (switched node, separate net); R35=1k (1:/LASER_SIG; 2:Net-(Q1-B)) |
| - | - | J26.2 | LIM_SW1 | 43 | PE12 | EXTI12 | via R23=100; R23=100 (1:Net-(J26-Pin_2); 2:/LIM_SW1) |
| - | - | J28.2 | LIM_SW2 | 44 | PE13 | EXTI13 | via R29=100; R29=100 (1:Net-(J28-Pin_2); 2:/LIM_SW2) |
| - | - | J31.2 | LIM_SW3 | 45 | PE14 | EXTI14 | via R33=100; R33=100 (1:Net-(J31-Pin_2); 2:/LIM_SW3) |

### Case E: 5db1aab

**FAIL: zero valid simultaneous timer assignments.** All inclusion-minimal impossible sets follow. Every larger impossible set contains at least one of these; choices and pins are enumerated in the table above.

1. ENC1 + ENC3 + PWM3.
2. ENC1 + ENC3 + ENC4 + PWM4.

If encoders 1 to 4 all use timer encoder mode, PWM3 and PWM4 have no hardware timer left: ENC1 needs TIM3, so ENC3 must use TIM20, and ENC4 needs TIM1; PWM3 can only use TIM3 or TIM20, and PWM4 only TIM1 or TIM20. Ways out include decoding one encoder in software from pin interrupts, or driving PWM3 and PWM4 in software or from other pins. This is a question for the team (the firmware may already plan for it), not a finding that the board cannot work. ENC_5 on PA1 and ENC_6 on PA0 are single-wire inputs; the team notes describe them as PWM-output absolute encoders, read by input capture (PA1: TIM2_CH2 or TIM5_CH2; PA0: TIM2_CH1 or TIM5_CH1).

## Previous revision (2026-09)

Export revision e9beafa, dated 2026-09-20. No simultaneous assignment chosen.

| Channel | Assumed role | Connector.pin | Signal | MCU pin | Port | Peripheral / eligible timer channel | Notes / direct passives |
|---|---|---|---|---|---|---|---|
| 1 | DC | J38.3 | PWM1 | 17 | PC2 | TIM1_CH3 AF2 / TIM20_CH2 AF6 | Conflict set 3, 4, 5 |
| 1 | DC | J38.4 | DIR1 | 18 | PC3 | GPIO output | - |
| 1 | DC | J39.1 | ENC1_A | 15 | PC0 | TIM1_CH1 AF2 | FT_a (5 V tolerant digital); Conflict set 3, 4, 5 |
| 1 | DC | J39.2 | ENC1_B | 16 | PC1 | TIM1_CH2 AF2 | TT_a (3.6 V tolerant; not 5 V tolerant); Conflict set 3, 4, 5 |
| 2 | DC | J45.3 | PWM2 | 25 | PA3 | TIM2_CH4 AF1 / TIM5_CH4 AF2 / TIM15_CH2 AF9 | - |
| 2 | DC | J45.4 | DIR2 | 26 | PA4 | GPIO output | - |
| 2 | DC | J40.1 | ENC2_A | 20 | PA0 | TIM2_CH1 AF1 / TIM5_CH1 AF2 | TT_a (3.6 V tolerant; not 5 V tolerant) |
| 2 | DC | J40.2 | ENC2_B | 21 | PA1 | TIM2_CH2 AF1 / TIM5_CH2 AF2 | TT_a (3.6 V tolerant; not 5 V tolerant) |
| 3 | DC | J46.3 | PWM3 | 4 | PE5 | TIM3_CH4 AF2 / TIM20_CH2N AF6 | Conflict set 2, 4, 5 |
| 3 | DC | J46.4 | DIR3 | 5 | PE6 | GPIO output | - |
| 3 | DC | J41.1 | ENC3_A | 28 | PA6 | TIM3_CH1 AF2 | TT_a (3.6 V tolerant; not 5 V tolerant); Conflict set 2, 3, 4 |
| 3 | DC | J41.2 | ENC3_B | 29 | PA7 | TIM3_CH2 AF2 | TT_a (3.6 V tolerant; not 5 V tolerant); Conflict set 2, 3, 4 |
| 4 | stepper | J47.3 | PWM4 | 67 | PC8 | TIM3_CH3 AF2 / HRTIM1_E_CH1 AF3 / TIM8_CH3 AF4 / TIM20_CH3 AF6 | - |
| 4 | stepper | J47.4 | DIR4 | 68 | PC9 | GPIO output | - |
| 4 | stepper | J42.1 | ENC4_A | 59 | PD12 | TIM4_CH1 AF2 | FT_a (5 V tolerant digital); Conflict set 1 |
| 4 | stepper | J42.2 | ENC4_B | 60 | PD13 | TIM4_CH2 AF2 | FT_a (5 V tolerant digital); Conflict set 1 |
| 5 | stepper | J48.3 | PWM5 | 62 | PD15 | TIM4_CH4 AF2 | Conflict set 1 |
| 5 | stepper | J48.4 | DIR5 | 61 | PD14 | GPIO output | - |
| 5 | stepper | J43.1 | ENC5_A | 65 | PC6 | TIM3_CH1 AF2 / TIM8_CH1 AF4 | FT_f (5 V tolerant digital) |
| 5 | stepper | J43.2 | ENC5_B | 66 | PC7 | TIM3_CH2 AF2 / TIM8_CH2 AF4 | FT_f (5 V tolerant digital) |
| 6 | stepper | J49.3 | PWM6 | 98 | PE1 | TIM17_CH1 AF4 / TIM20_CH4 AF6 | - |
| 6 | stepper | J49.4 | DIR6 | 97 | PE0 | GPIO output | - |
| 6 | stepper | J44.1 | ENC6_A | 1 | PE2 | TIM3_CH1 AF2 / TIM20_CH1 AF6 | FT (5 V tolerant digital); Conflict set 2, 3, 5 |
| 6 | stepper | J44.2 | ENC6_B | 2 | PE3 | TIM3_CH2 AF2 / TIM20_CH2 AF6 | FT (5 V tolerant digital); Conflict set 2, 3, 5 |
| - | - | J30.1 | FSR_ADC | 44 | PE13 | ADC3_IN3 | via R36=10k; C8=100nF (1:/FSR_ADC; 2:GND); R36=10k (1:/FSR_ADC; 2:Net-(J30-Pin_1)) |
| - | - | J30.2 | FSR_SIG | 45 | PE14 | GPIO output | via Q4 (switched node, separate net); R28=10k (1:/FSR_SIG; 2:v3v3) |
| - | - | J37.1 | LASER_SIG | 46 | PE15 | GPIO output | via Q5 (switched node, separate net); R30=10k (1:/LASER_SIG; 2:GND) |
| - | - | J31.1 | LIM_SW1 | 41 | PE10 | EXTI10 | via R38=100; R38=100 (1:Net-(J31-Pin_1); 2:/LIM_SW1) |
| - | - | J36.1 | LIM_SW2 | 42 | PE11 | EXTI11 | via R39=100; R39=100 (1:Net-(J36-Pin_1); 2:/LIM_SW2) |
| - | - | J33.1 | LIM_SW3 | 43 | PE12 | EXTI12 | via R40=100; R40=100 (1:Net-(J33-Pin_1); 2:/LIM_SW3) |

### Case E: e9beafa

**FAIL: zero valid simultaneous timer assignments.** All inclusion-minimal impossible sets follow. Every larger impossible set contains at least one of these; choices and pins are enumerated in the table above.

1. ENC4 + PWM5.
2. ENC3 + ENC6 + PWM3.
3. ENC1 + ENC3 + ENC6 + PWM1.
4. ENC1 + ENC3 + PWM1 + PWM3.
5. ENC1 + ENC6 + PWM1 + PWM3.

The conflict is reproduced independently from the schematic: ENC4 uses PD12/PD13, TIM4_CH1/CH2 AF2, while PWM5 on PD15 has only TIM4_CH4 AF2. The encoder-driven counter prevents independent PWM5. Other conflict sets remain even if that conflict is removed. ENC3 fixes TIM3; ENC6 then needs TIM20, leaving PWM3 without a free counter. PWM1 and PWM3 can also collide on TIM20 CCR2, because CH2 and CH2N share that compare channel. HRTIM1 CHE1 on PC8 is included as an alternative for PWM4; it does not remove these conflicts.

## Force input circuits

Current: J27.2 supplies v3v3 and J27.1 is FSR_ADC, directly connected to PE10 (U5.41). R26 = 1M and C32 = 10nF both connect that node to GND. There is no FSR_SIG net. If a resistive force sensor is connected across the two connector pins (assumed external sensor), it forms a divider with R26; increasing conductance raises the ADC node voltage. R26 and C32 alone have a nominal 10 ms product; the actual filter time depends on sensor resistance. PE10 can use ADC3, ADC4 or ADC5, channel 14, with no AF number.

Previous: J30.1 has R29 = 1M to GND and R36 = 10k in series to FSR_ADC at PE13 (U5.44). C8 = 100nF connects FSR_ADC to GND. J30.2 is supplied through Q4 (AO3401A): pad 2 is v3v3, pad 3 is J30.2, and gate pad 1 is FSR_SIG at PE14 (U5.45), pulled up to v3v3 by R28 = 10k. This is a switched sensor supply; low gate drive enables it and the pull-up biases it off. The external resistive sensor across J30.1/J30.2 is assumed. The ADC input is ADC3 channel 3. The R36/C8 product is 1 ms, but source resistance also affects settling; this is not a measured response.

Both channels are also generated as SPICE subcircuits from the schematic netlists by `board/extract_fsr.py`: `board/fsr_front_5db1aab.cir` and `board/fsr_front_e9beafa.cir`. Q4 has no SPICE model in the schematic, so it is a switch with the AO3401A datasheet's maximum Rds(on) at -2.5 V and typical threshold.

## Limit switches

Current: PE12/PE13/PE14 use EXTI12/13/14, respectively. LIM_SW1 reaches J26.2 through R23 = 100 ohm; LIM_SW2 reaches J28.2 through R29 = 100 ohm; LIM_SW3 reaches J31.2 through R33 = 100 ohm. Connector pin 1 is v3v3. R25/R30/R34 = 10k pull the respective connector-side signal nodes to GND. A switch bridging connector pins 1 and 2 is assumed active high.

Previous: PE10/PE11/PE12 use EXTI10/11/12. LIM_SW1 reaches J31.1 through R38 = 100 ohm; LIM_SW2 reaches J36.1 through R39 = 100 ohm; LIM_SW3 reaches J33.1 through R40 = 100 ohm. Connector pin 2 is GND. R18/R21/R24 = 1k pull the corresponding connector-side nodes to v3v3. A switch bridging pins 1 and 2 is assumed active low. Each revision has distinct EXTI lines, so this check passes; sharing an interrupt vector would not itself be an EXTI-line collision.

## Laser output

Current: LASER_SIG is PE11 (U5.42), connected through R35 = 1k to Q1 base pad 1. Q1 (MMBT3904LT1G) emitter pad 2 is GND and collector pad 3 reaches J30.1; J30.2 is v3v3. This is a transistor-switched low-side output, not a direct MCU laser supply. No base pull-down resistor is present on LASER_SIG in this export.

Previous: LASER_SIG is PE15 (U5.46), connected to Q5 (2N7002) gate pad 1, with R30 = 10k to GND. Source pad 2 is GND and drain pad 3 reaches J37.1; J37.2 is v3v3. This is also a switched low-side output. Load current and laser-module compatibility were not checked.

## Datasheet entries used

Source: [DS12288 Rev 6, November 2021](https://www.st.com/resource/en/datasheet/stm32g474ce.pdf). All pages below are printed page numbers. Table 13 supplies timer channel AFs; Table 12 supplies ADC inputs and I/O structure. Only ports used by either revision appear. Other serial, trigger, break and synchronization AFs are outside this table. HRTIM1_E/F CH1/2 denotes CHE1/2 and CHF1/2 waveform outputs. ADC functions do not use GPIO AF selection. EXTI line numbers equal GPIO bit numbers (section 3.17.2, p. 31).

| Port | Timer channel (AF) | Table 13 page | ADC inputs | I/O | Table 12 page | EXTI |
|---|---|---|---|---|---|---|
| PA0 | TIM2_CH1 (AF1), TIM5_CH1 (AF2) | 73 | ADC1_IN1, ADC2_IN1 | TT_a | 59 | 0 |
| PA1 | TIM2_CH2 (AF1), TIM5_CH2 (AF2), TIM15_CH1N (AF9) | 73 | ADC1_IN2, ADC2_IN2 | TT_a | 59 | 1 |
| PA2 | TIM2_CH3 (AF1), TIM5_CH3 (AF2), TIM15_CH1 (AF9) | 73 | ADC1_IN3 | FT_a | 59 | 2 |
| PA3 | TIM2_CH4 (AF1), TIM5_CH4 (AF2), TIM15_CH2 (AF9) | 73 | ADC1_IN4 | TT_a | 60 | 3 |
| PA4 | TIM3_CH2 (AF2) | 73 | ADC2_IN17 | TT_a | 60 | 4 |
| PA6 | TIM16_CH1 (AF1), TIM3_CH1 (AF2) | 73 | ADC2_IN3 | TT_a | 60 | 6 |
| PA7 | TIM17_CH1 (AF1), TIM3_CH2 (AF2), TIM8_CH1N (AF4), TIM1_CH1N (AF6) | 73 | ADC2_IN4 | TT_a | 60 | 7 |
| PB3 | TIM2_CH2 (AF1), TIM8_CH1N (AF4) | 74 | - | FT | 70 | 3 |
| PB4 | TIM16_CH1 (AF1), TIM3_CH1 (AF2), TIM8_CH2N (AF4) | 74 | - | FT_c | 70 | 4 |
| PB5 | TIM3_CH2 (AF2), TIM8_CH3N (AF3), TIM17_CH1 (AF10) | 74 | - | FT_f | 70 | 5 |
| PB6 | TIM16_CH1N (AF1), TIM4_CH1 (AF2), TIM8_CH1 (AF5) | 74 | - | FT_c | 71 | 6 |
| PB7 | TIM17_CH1N (AF1), TIM4_CH2 (AF2), TIM3_CH4 (AF10) | 74 | - | FT_f | 71 | 7 |
| PB9 | TIM17_CH1 (AF1), TIM4_CH4 (AF2), TIM8_CH3 (AF10), TIM1_CH3N (AF12) | 74 | - | FT_f | 71 | 9 |
| PC0 | TIM1_CH1 (AF2) | 75 | ADC1_IN6, ADC2_IN6 | FT_a | 59 | 0 |
| PC1 | TIM1_CH2 (AF2) | 75 | ADC1_IN7, ADC2_IN7 | TT_a | 59 | 1 |
| PC2 | TIM1_CH3 (AF2), TIM20_CH2 (AF6) | 75 | ADC1_IN8, ADC2_IN8 | FT_a | 59 | 2 |
| PC3 | TIM1_CH4 (AF2) | 75 | ADC1_IN9, ADC2_IN9 | TT_a | 59 | 3 |
| PC6 | TIM3_CH1 (AF2), TIM8_CH1 (AF4), HRTIM1_F_CH1 (AF13) | 75 | - | FT_f | 64 | 6 |
| PC7 | TIM3_CH2 (AF2), TIM8_CH2 (AF4), HRTIM1_F_CH2 (AF13) | 75 | - | FT_f | 65 | 7 |
| PC8 | TIM3_CH3 (AF2), HRTIM1_E_CH1 (AF3), TIM8_CH3 (AF4), TIM20_CH3 (AF6) | 75 | - | FT_f | 65 | 8 |
| PC9 | TIM3_CH4 (AF2), HRTIM1_E_CH2 (AF3), TIM8_CH4 (AF4) | 75 | - | FT_f | 65 | 9 |
| PD12 | TIM4_CH1 (AF2) | 76 | ADC3_IN9, ADC4_IN9, ADC5_IN9 | FT_a | 64 | 12 |
| PD13 | TIM4_CH2 (AF2) | 76 | ADC3_IN10, ADC4_IN10, ADC5_IN10 | FT_a | 64 | 13 |
| PD14 | TIM4_CH3 (AF2) | 76 | ADC3_IN11, ADC4_IN11, ADC5_IN11 | TT_a | 64 | 14 |
| PD15 | TIM4_CH4 (AF2) | 76 | - | FT_a | 64 | 15 |
| PE0 | TIM20_CH4N (AF3), TIM16_CH1 (AF4) | 77 | - | FT | 72 | 0 |
| PE1 | TIM17_CH1 (AF4), TIM20_CH4 (AF6) | 77 | - | FT | 72 | 1 |
| PE10 | TIM1_CH2N (AF2) | 77 | ADC3_IN14, ADC4_IN14, ADC5_IN14 | FT_a | 62 | 10 |
| PE11 | TIM1_CH2 (AF2) | 77 | ADC3_IN15, ADC4_IN15, ADC5_IN15 | FT_a | 62 | 11 |
| PE12 | TIM1_CH3N (AF2) | 77 | ADC3_IN16, ADC4_IN16, ADC5_IN16 | FT_a | 62 | 12 |
| PE13 | TIM1_CH3 (AF2) | 77 | ADC3_IN3 | FT_a | 62 | 13 |
| PE14 | TIM1_CH4 (AF2) | 77 | ADC4_IN1 | FT_a | 62 | 14 |
| PE15 | TIM1_CH4N (AF6) | 77 | ADC4_IN2 | FT_a | 63 | 15 |
| PE2 | TIM3_CH1 (AF2), TIM20_CH1 (AF6) | 77 | - | FT | 57 | 2 |
| PE3 | TIM3_CH2 (AF2), TIM20_CH2 (AF6) | 77 | - | FT | 57 | 3 |
| PE4 | TIM3_CH3 (AF2), TIM20_CH1N (AF6) | 77 | - | FT | 57 | 4 |
| PE5 | TIM3_CH4 (AF2), TIM20_CH2N (AF6) | 77 | - | FT | 57 | 5 |
| PE6 | TIM20_CH3N (AF6) | 77 | - | FT | 57 | 6 |
| PF2 | TIM20_CH3 (AF2) | 78 | - | FT | 59 | 2 |

Encoder mode and counter behavior were checked against [RM0440 Rev 9](https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf), chapters 29 and 30, including section 29.3.25 (p. 1207 onward) and the general-purpose encoder description (p. 1335). Capture/compare circuitry can still operate in encoder mode, but its counter follows the encoder; it does not provide an independent PWM timebase.

## Reproduction

Run pin_map.py with a revision folder and --output pointing to the desired CSV. Run check_timers.py with that CSV and optionally --output pointing to a text report. Both scripts use the Python standard library and make no network requests. The tests verify expected pins and connector assignments, re-extract both exports, reproduce both reports, and check that each reported conflict is minimal. No hardware runtime, voltage waveform, routing continuity or populated-board acceptance was tested.
