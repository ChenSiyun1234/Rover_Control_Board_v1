"""Two masses with unilateral Kelvin-Voigt contact and rack resistance."""

import math

from .params import Params


class Plant:
    """Integrate driver current, carriage motion and server contact in SI units."""

    def __init__(self, parameters=None, jam=False):
        """Initialize the instance."""
        self.parameters = parameters or Params()
        self.jam = jam
        self.carriage_position_m = self.server_position_m = (
            self.carriage_velocity_m_s
        ) = self.server_velocity_m_s = self.current = self.force = 0.0
        self.mass = (
            self.parameters.carriage_mass
            + self.parameters.rotor_j * (2 * math.pi / self.parameters.lead) ** 2
        )
        reduced_mass_kg = (
            self.mass
            * self.parameters.server_mass
            / (self.mass + self.parameters.server_mass)
        )
        self.damping = (
            2
            * self.parameters.damping_ratio
            * math.sqrt(self.parameters.pusher_k * 1000 * reduced_mass_kg)
        )

    def contact(self):
        """Return compressive spring and damper force; separation transmits none."""
        return (
            max(
                0,
                self.parameters.pusher_k
                * 1000
                * (self.carriage_position_m - self.server_position_m)
                + self.damping
                * (self.carriage_velocity_m_s - self.server_velocity_m_s),
            )
            if self.carriage_position_m > self.server_position_m
            else 0.0
        )

    def jam_force(self):
        """Return the unilateral server-to-rack jam spring force in newtons."""
        return (
            max(0.0, self.server_position_m * 1000 - self.parameters.x_jam)
            * self.parameters.k_jam
            if self.jam else 0.0
        )

    def step(self, current_command_A, enabled=True):
        """Advance one simulation or control tick."""
        parameters = self.parameters
        time_step_s = parameters.dt
        desired_current_A = (
            max(-parameters.max_current, min(parameters.max_current, current_command_A))
            if enabled
            else 0
        )
        self.current += (desired_current_A - self.current) * (
            1 - math.exp(-time_step_s / parameters.current_tau)
        )
        self.force = self.contact()
        drive_force_N = (
            self.current
            * parameters.kt
            * 2
            * math.pi
            * parameters.efficiency
            / parameters.lead
        )
        drag_force_N = (
            parameters.coulomb
            * math.tanh(self.carriage_velocity_m_s / parameters.friction_smoothing)
            + parameters.viscous * self.carriage_velocity_m_s
        )
        self.carriage_velocity_m_s += (
            (drive_force_N - drag_force_N - self.force) / self.mass * time_step_s
        )
        server_position_mm = self.server_position_m * 1000
        if server_position_mm < parameters.mate_start:
            connector_force_N = 0
        elif server_position_mm < parameters.mate_rise_end:
            connector_force_N = (
                parameters.connector_peak
                * (server_position_mm - parameters.mate_start)
                / (parameters.mate_rise_end - parameters.mate_start)
            )
        else:
            connector_force_N = parameters.connector_hold
        rack_force_N = (
            connector_force_N
            + max(0, server_position_mm - parameters.end_stop) * parameters.end_k
            + (
                self.jam_force()
            )
        )
        net_force_N = self.force - rack_force_N
        rail_friction_N = (
            parameters.rail_mu * parameters.server_mass * parameters.gravity
        )
        if (
            abs(self.server_velocity_m_s) < 1e-05
            and abs(net_force_N) <= rail_friction_N
        ):
            self.server_velocity_m_s = 0
        else:
            sign = (
                1
                if self.server_velocity_m_s > 0
                else -1
                if self.server_velocity_m_s < 0
                else 1
                if net_force_N > 0
                else -1
            )
            next_velocity_m_s = (
                self.server_velocity_m_s
                + (net_force_N - sign * rail_friction_N)
                / parameters.server_mass
                * time_step_s
            )
            if (
                next_velocity_m_s * self.server_velocity_m_s < 0
                and abs(net_force_N) <= rail_friction_N
            ):
                next_velocity_m_s = 0
            self.server_velocity_m_s = next_velocity_m_s
        self.carriage_position_m += self.carriage_velocity_m_s * time_step_s
        self.server_position_m += self.server_velocity_m_s * time_step_s
        self.force = self.contact()
