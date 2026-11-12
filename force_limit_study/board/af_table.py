"""Relevant timer outputs/inputs, ADC inputs and EXTI for the used ports only.

Source: DS12288 Rev 6 (November 2021), STM32G474xB/xC/xE.
https://www.st.com/resource/en/datasheet/stm32g474ce.pdf
Table 13, printed pp. 73-78: timer channel AFs (not other serial AFs,
break inputs, external triggers or synchronization signals).
Table 12: ADC and I/O structures, printed pages recorded below.
EXTI line = GPIO bit number: section 3.17.2, p. 31; no AF selection.
CHxN denotes a complementary OUTPUT, never an encoder input.
HRTIM1_E/F CH1/2 denote CHE1/2 and CHF1/2 waveform outputs.
"""

# port: (Table 13 page, Table 12 page, I/O structure, timer/channel/AF, ADC inputs)
RAW = {
    "PA0": (73, 59, "TT_a", "TIM2:1:1 TIM5:1:2", "ADC1:1 ADC2:1"),
    "PA1": (73, 59, "TT_a", "TIM2:2:1 TIM5:2:2 TIM15:1N:9", "ADC1:2 ADC2:2"),
    "PA2": (73, 59, "FT_a", "TIM2:3:1 TIM5:3:2 TIM15:1:9", "ADC1:3"),
    "PA3": (73, 60, "TT_a", "TIM2:4:1 TIM5:4:2 TIM15:2:9", "ADC1:4"),
    "PA4": (73, 60, "TT_a", "TIM3:2:2", "ADC2:17"),
    "PA6": (73, 60, "TT_a", "TIM16:1:1 TIM3:1:2", "ADC2:3"),
    "PA7": (73, 60, "TT_a", "TIM17:1:1 TIM3:2:2 TIM8:1N:4 TIM1:1N:6", "ADC2:4"),
    "PB3": (74, 70, "FT", "TIM2:2:1 TIM8:1N:4", ""),
    "PB4": (74, 70, "FT_c", "TIM16:1:1 TIM3:1:2 TIM8:2N:4", ""),
    "PB5": (74, 70, "FT_f", "TIM3:2:2 TIM8:3N:3 TIM17:1:10", ""),
    "PB6": (74, 71, "FT_c", "TIM16:1N:1 TIM4:1:2 TIM8:1:5", ""),
    "PB7": (74, 71, "FT_f", "TIM17:1N:1 TIM4:2:2 TIM3:4:10", ""),
    "PB9": (74, 71, "FT_f", "TIM17:1:1 TIM4:4:2 TIM8:3:10 TIM1:3N:12", ""),
    "PC0": (75, 59, "FT_a", "TIM1:1:2", "ADC1:6 ADC2:6"),
    "PC1": (75, 59, "TT_a", "TIM1:2:2", "ADC1:7 ADC2:7"),
    "PC2": (75, 59, "FT_a", "TIM1:3:2 TIM20:2:6", "ADC1:8 ADC2:8"),
    "PC3": (75, 59, "TT_a", "TIM1:4:2", "ADC1:9 ADC2:9"),
    "PC6": (75, 64, "FT_f", "TIM3:1:2 TIM8:1:4 HRTIM1_F:1:13", ""),
    "PC7": (75, 65, "FT_f", "TIM3:2:2 TIM8:2:4 HRTIM1_F:2:13", ""),
    "PC8": (75, 65, "FT_f", "TIM3:3:2 HRTIM1_E:1:3 TIM8:3:4 TIM20:3:6", ""),
    "PC9": (75, 65, "FT_f", "TIM3:4:2 HRTIM1_E:2:3 TIM8:4:4", ""),
    "PD12": (76, 64, "FT_a", "TIM4:1:2", "ADC3:9 ADC4:9 ADC5:9"),
    "PD13": (76, 64, "FT_a", "TIM4:2:2", "ADC3:10 ADC4:10 ADC5:10"),
    "PD14": (76, 64, "TT_a", "TIM4:3:2", "ADC3:11 ADC4:11 ADC5:11"),
    "PD15": (76, 64, "FT_a", "TIM4:4:2", ""),
    "PE0": (77, 72, "FT", "TIM20:4N:3 TIM16:1:4", ""),
    "PE1": (77, 72, "FT", "TIM17:1:4 TIM20:4:6", ""),
    "PE2": (77, 57, "FT", "TIM3:1:2 TIM20:1:6", ""),
    "PE3": (77, 57, "FT", "TIM3:2:2 TIM20:2:6", ""),
    "PE4": (77, 57, "FT", "TIM3:3:2 TIM20:1N:6", ""),
    "PE5": (77, 57, "FT", "TIM3:4:2 TIM20:2N:6", ""),
    "PE6": (77, 57, "FT", "TIM20:3N:6", ""),
    "PE10": (77, 62, "FT_a", "TIM1:2N:2", "ADC3:14 ADC4:14 ADC5:14"),
    "PE11": (77, 62, "FT_a", "TIM1:2:2", "ADC3:15 ADC4:15 ADC5:15"),
    "PE12": (77, 62, "FT_a", "TIM1:3N:2", "ADC3:16 ADC4:16 ADC5:16"),
    "PE13": (77, 62, "FT_a", "TIM1:3:2", "ADC3:3"),
    "PE14": (77, 62, "FT_a", "TIM1:4:2", "ADC4:1"),
    "PE15": (77, 63, "FT_a", "TIM1:4N:6", "ADC4:2"),
    "PF2": (78, 59, "FT", "TIM20:3:2", ""),
}

PINS = {
    port: dict(af_page=ap, pin_page=pp, io=io,
               timers=[(t, "CH" + c, int(af)) for t, c, af in (x.split(":") for x in timers.split())],
               adc=[(a, int(c)) for a, c in (x.split(":") for x in adc.split())],
               exti=int(port[2:]))
    for port, (ap, pp, io, timers, adc) in RAW.items()
}

# RM0440 Rev 9, chapters 29 (TIM1/8/20) and 30 (TIM2/3/4/5):
# encoder mode uses TI1/TI2 and the shared counter; TIM15/16/17 excluded.
ENCODER_TIMERS = {"TIM1", "TIM2", "TIM3", "TIM4", "TIM5", "TIM8", "TIM20"}
