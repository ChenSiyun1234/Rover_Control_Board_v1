"""SI mechanics, mm control coordinates. Sources are attached to every parameter."""

from dataclasses import dataclass

HX_URL = "https://cdn.sparkfun.com/assets/0/2/7/d/1/HX711_datasheet.pdf"
ADS_URL = "https://www.ti.com/lit/ds/symlink/ads1220.pdf"
# rate, settling periods, input-referred RMS noise V, input full scale V
# IDEAL_1000 is a synthetic reference, not a real ADC or a part recommendation.
ADC_MODELS = {
    # Synthetic reference: no averaging, bridge vibration or electrical noise.
    "IDEAL_1000": (1000, 0, 0.0, 2.048 / 128),
    # HX_URL, p3 Electrical Characteristics, gain 128, settling footnote.
    "HX711_10": (10, 4, 50e-9, 5 / 256),
    "HX711_80": (80, 4, 90e-9, 5 / 256),
    # ADS_URL, Table 7-1, gain 128; section 8.3.6 (single-cycle settling).
    "ADS1220_330": (330, 1, 0.34e-6, 2.048 / 128),
    "ADS1220_1000": (1000, 1, 0.70e-6, 2.048 / 128),
    # ADS_URL, Table 7-5, gain 128, turbo; section 8.3.6.
    "ADS1220_2000": (2000, 1, 0.82e-6, 2.048 / 128),
}


@dataclass
class Params:
    """Physical assumptions in SI units unless a field comment specifies otherwise."""

    lead: float = 0.010  # [assumed: 10 mm/rev screw]
    efficiency: float = 0.9  # [assumed: ball screw efficiency]
    kt: float = 0.1  # [assumed: 24 V servo class N m/A]
    rotor_j: float = 1.2e-5  # [assumed: small servo rotor kg m^2]
    max_current: float = 8  # [assumed: servo rating A]
    current_tau: float = 0.0005  # [assumed: driver current bandwidth]
    carriage_mass: float = 6  # [assumed: pusher carriage kg]
    coulomb: float = 10  # [assumed: guide friction N]
    friction_smoothing: float = 0.001  # [assumed: tanh smoothing m/s]
    viscous: float = 50  # [assumed: guide drag N/(m/s)]
    server_mass: float = 25  # [assumed: 1U-2U server kg]
    rail_mu: float = 0.1  # [assumed: sliding rail coefficient]
    gravity: float = 9.81  # [assumed: standard terrestrial acceleration m/s^2]
    pusher_k: float = 200  # [assumed: stiff load path N/mm; alternatives 20 and 5]
    damping_ratio: float = 0.05  # [assumed: lightly damped contact]
    mate_start: float = 280  # [assumed: connector begins at mm]
    mate_rise_end: float = 290  # [assumed: connector ramp ends at mm]
    connector_peak: float = 90  # [assumed: connector ramp N]
    connector_hold: float = 60  # [assumed: connector sliding N]
    end_stop: float = 295  # [assumed: rack travel mm]
    end_k: float = 500  # [assumed: end stop N/mm]
    x_jam: float = 150  # [assumed: optional misalignment mm]
    k_jam: float = 100  # [assumed: obstruction stiffness N/mm]
    encoder_lines: int = 2048  # [assumed: motor encoder lines/rev]
    quadrature: int = 4  # [assumed: x4 decoding]
    cell_rating: float = 500  # [assumed: typical rated load N]
    cell_mv_v: float = 2  # [assumed: typical bridge sensitivity mV/V]
    excitation: float = 5  # [assumed: bridge supply V]
    vibration_rms: float = (
        2  # [assumed: moving structural noise N RMS per 10 kHz sample]
    )
    vibration_amp: float = 5  # [assumed: structural mode N amplitude]
    vibration_hz: float = 150  # [assumed: structural resonance Hz]
    dt: float = 0.0001  # [assumed: requested 10 kHz integration]
    control_dt: float = 0.001  # [assumed: requested 1 kHz firmware tick]
    baud: int = 115200  # [assumed: UART 8N1 link]
    seed: int = 42  # [assumed: reproducible random stream]
