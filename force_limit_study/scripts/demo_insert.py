"""Run the nominal insertion entirely through the nonblocking UART client."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sim.sil import Sim


def main():
    """Print seating metrics and return an error if the axis does not seat."""
    result = Sim().run_insert()
    print(
        f"Final state: {result['final_state']}; "
        f"seat position: {result['seat_position_mm']:.3f} mm; "
        f"peak force: {result['peak_force_N']:.2f} N; "
        f"cycle time: {result['cycle_time_s']:.3f} s"
    )
    if result["final_state"] != "SEATED":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
