"""MuJoCo mechanics with current-mode and voltage-mode brushed motor paths."""

import math

import mujoco as mj
import numpy as np

from .scene import ARM, build


class DCMotor:
    """Average-voltage MD13S-like drive; no current loop or current clamp."""

    bus_V = 12.0  # Team document: arm supply bus, V.
    resistance_ohm = 0.7  # Assumed: 12/0.7 = 17.14 A stall, near team estimate.
    inductance_H = 0.0005  # Assumed armature inductance, H; not measured.
    ke = ARM.kt  # SI reciprocity: 0.04 V s/rad from the existing 0.04 N m/A.

    @classmethod
    def advance(cls, current, duty, direction, motor_speed, dt, enabled=True):
        voltage = duty*cls.bus_V*(1 if direction else -1) if enabled else 0.0
        # Exact electrical step at held shaft speed avoids Euler stiffness.
        # Disabled drive assumes ideal open circuit; switching transients omitted.
        if not enabled:
            return 0.0
        equilibrium = (voltage-cls.ke*motor_speed)/cls.resistance_ohm
        return equilibrium+(current-equilibrium)*math.exp(-dt*cls.resistance_ohm/cls.inductance_H)


class MjPlant:
    """Integrate joint dynamics in MuJoCo; expose linear tool/load coordinates."""

    def __init__(self, parameters, jam=False, case="arm", counts_per_m=None,
                 motor_polarity=1, encoder_sign=1, limit_at_mm=-30, bounce_ms=2, switch_seed=41):
        self.parameters, self.jam, self.case = parameters, jam, case
        self.motor_polarity, self.encoder_sign = motor_polarity, encoder_sign
        self.model, self.names = build(case, jam, parameters)
        self.data = mj.MjData(self.model)
        self.drive = self.model.joint(self.names["drive_joint"]).dofadr[0]
        self.load = self.model.joint(self.names["load_joint"]).dofadr[0]
        self.drive_q = self.model.joint(self.names["drive_joint"]).qposadr[0]
        self.load_q = self.model.joint(self.names["load_joint"]).qposadr[0]
        self.radius = self.names.get("radius", 1.0)
        # Assumed base switch: 30 mm behind start, NO contact to ground with pull-up.
        # Assumed 2 ms closing bounce; fixed seed 41 makes contact chatter repeatable.
        self.limit_at_mm, self.bounce_ms = limit_at_mm, bounce_ms
        self.switch_rng = np.random.default_rng(switch_seed)
        self.switch_closed_at_s = None
        self.first_switch_s = None
        self.limit_switch = False
        self.current = self.force = 0.0
        self.steps = 0
        self.qpos_log = [(0.0, self.data.qpos.copy())]
        self.contact_force = np.zeros(6)
        mj.mj_forward(self.model, self.data)
        if case == "arm":
            self.counts_per_m = ARM.counts_per_m if counts_per_m is None else counts_per_m
            self.contact_ids = {self.model.geom(n).id for n in self.names["contact_geoms"]}
            self.damping = self.names["damping"]
        else:
            p = parameters
            self.mass = p.carriage_mass + p.rotor_j*(2*math.pi/p.lead)**2
            reduced = self.mass*p.server_mass/(self.mass+p.server_mass)
            self.damping = 2*p.damping_ratio*math.sqrt(p.pusher_k*1000*reduced)

    @property
    def carriage_position_m(self):
        return float(self.data.qpos[self.drive_q]*self.radius)

    @property
    def carriage_velocity_m_s(self):
        return float(self.data.qvel[self.drive]*self.radius)

    @property
    def server_position_m(self):
        return float(self.data.qpos[self.load_q])

    @property
    def server_velocity_m_s(self):
        return float(self.data.qvel[self.load])

    def jam_force(self):
        """Unilateral obstacle spring; travel is module/server displacement."""
        if not self.jam:
            return 0.0
        if self.case == "arm":
            return max(0, self.server_position_m-ARM.jam_at)*ARM.jam_k
        return max(0, self.server_position_m*1000-self.parameters.x_jam)*self.parameters.k_jam

    def _rack_contact(self):
        overlap = self.carriage_position_m-self.server_position_m
        return max(0, self.parameters.pusher_k*1000*overlap + self.damping*(
            self.carriage_velocity_m_s-self.server_velocity_m_s)) if overlap > 0 else 0.0

    def _load_force(self, net, friction, mass):
        """Match Plant's sticking and zero-crossing Coulomb law through applied force."""
        v, dt = self.server_velocity_m_s, self.parameters.dt
        if abs(v) < 1e-5 and abs(net) <= friction:
            return -v*mass/dt
        sign = 1 if v > 0 else -1 if v < 0 else 1 if net > 0 else -1
        force = net-sign*friction
        if (v+force/mass*dt)*v < 0 and abs(net) <= friction:
            return -v*mass/dt
        return force

    def step(self, current_command_A, enabled=True):
        """Advance exactly one parameters.dt, with a first-order driver current lag."""
        p = self.parameters
        motor = ARM if self.case == "arm" else p
        desired = max(-motor.max_current, min(motor.max_current, current_command_A)) if enabled else 0
        desired *= self.motor_polarity if self.case == "arm" else 1
        self.current += (desired-self.current)*(1-math.exp(-p.dt/motor.current_tau))
        self._mechanical_step()

    def step_pwm(self, duty, direction, enabled=True):
        """Drive the arm with averaged terminal voltage; zero duty dynamically brakes."""
        if self.case != "arm":
            raise ValueError("The PWM motor model is configured only for the arm")
        # Express current/EMF in physical positive-torque coordinates; a lead
        # swap reverses the voltage applied in those coordinates.
        signed_direction = direction if self.motor_polarity == 1 else 1-direction
        self.current = DCMotor.advance(self.current, duty, signed_direction,
                                       self.data.qvel[self.drive]*ARM.gear,
                                       self.parameters.dt, enabled)
        self._mechanical_step()

    def _mechanical_step(self):
        p = self.parameters
        x, v = self.server_position_m, self.carriage_velocity_m_s
        self.data.qfrc_applied[:] = 0
        if self.case == "rack":
            self.force = self._rack_contact()
            drive = self.current*p.kt*2*math.pi*p.efficiency/p.lead
            drag = p.coulomb*math.tanh(v/p.friction_smoothing)+p.viscous*v
            self.data.qfrc_applied[self.drive] = drive-drag-self.force
            mm = x*1000
            connector = (0 if mm < p.mate_start else
                         p.connector_peak*(mm-p.mate_start)/(p.mate_rise_end-p.mate_start)
                         if mm < p.mate_rise_end else p.connector_hold)
            resistance = connector+max(0, mm-p.end_stop)*p.end_k+self.jam_force()
            self.data.qfrc_applied[self.load] = self._load_force(
                self.force-resistance, p.rail_mu*p.server_mass*p.gravity, p.server_mass)
        else:
            a = ARM
            torque = self.current*a.kt*a.gear*a.efficiency
            self.data.qfrc_applied[self.drive] = torque
            latch_start = a.end-a.latch_length
            latch = (0 if x < latch_start else a.latch_peak*(x-latch_start)/a.latch_length
                     if x < a.end else a.latch_hold)
            resistance = latch+max(0, x-a.end)*a.end_k+self.jam_force()
            # Native joint frictionloss supplies the module's sticking/sliding force.
            self.data.qfrc_applied[self.load] = -resistance
        mj.mj_step(self.model, self.data)
        if self.case == "rack":
            self.force = self._rack_contact()
        else:
            mj.mj_forward(self.model, self.data)
            self.force = self._normal_force()
        self.steps += 1
        if self.case == "arm":
            time_s = self.steps*p.dt
            if self.carriage_position_m*1000 <= self.limit_at_mm:
                if self.switch_closed_at_s is None:
                    self.switch_closed_at_s = time_s
                    if self.first_switch_s is None:
                        self.first_switch_s = time_s
                self.limit_switch = (time_s-self.switch_closed_at_s >= self.bounce_ms/1000
                                     or bool(self.switch_rng.integers(2)))
            else:
                self.switch_closed_at_s = None
                self.limit_switch = False
        if self.steps % 10 == 0:
            self.qpos_log.append((self.steps*p.dt, self.data.qpos.copy()))

    def _normal_force(self):
        total = 0.0
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            if {int(contact.geom1), int(contact.geom2)} == self.contact_ids:
                mj.mj_contactForce(self.model, self.data, i, self.contact_force)
                total += max(0.0, self.contact_force[0])
        return total
