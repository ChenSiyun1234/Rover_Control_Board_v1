"""Parked team rover and one selected, two-degree-of-freedom test fixture."""

from dataclasses import dataclass
import importlib.util
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco as mj

from sim.params import Params

ASSETS = Path(__file__).parent / "assets"


@dataclass(frozen=True)
class ArmParams:
    """Arm mechanics. Link lengths and masses come from the team's arm design sheet
    ("Better Approx. w/ CAD", before the arm was shortened); everything marked assumed
    is a study value, not measured hardware."""

    turret_mass: float = 2.0  # Assumed: turntable, driven spur gear and shoulder mount, kg.
    upper_mass: float = 1.4  # Design sheet: shoulder-to-elbow link, kg.
    elbow_actuator_mass: float = 0.77  # Design sheet: actuator at the elbow, kg.
    fore_mass: float = 0.96  # Design sheet: elbow-to-wrist link, kg.
    wrist_actuator_mass: float = 1.75  # Design sheet: actuator at the wrist, kg.
    tool_mass: float = 1.5  # Design sheet: hand, kg (no payload in this test).
    upper_length: float = 0.404  # Design sheet: shoulder to elbow, m.
    fore_length: float = 0.473  # Design sheet: elbow to wrist, m.
    tool_length: float = 0.075  # Design sheet: wrist to load, m.
    shoulder_height: float = 0.12  # Assumed turret height above the frame top, m.
    upper_deg: float = 78  # Assumed pose, read off the team's CAD picture.
    fore_deg: float = 18  # Assumed pose, read off the team's CAD picture.
    module_mass: float = 1.0  # Assumed drawer mass, kg.
    gap: float = 0.010  # Assumed initial pad-to-drawer clearance, m.
    friction: float = 4.0  # Assumed drawer slide friction, N.
    latch_peak: float = 20.0  # Assumed latch peak, about a third of the arm's tip force.
    latch_hold: float = 8.0  # Assumed resistance after the latch clicks, N.
    latch_length: float = 0.008  # Assumed final latch ramp length, m.
    end: float = 0.040  # Assumed drawer travel to closed, m.
    end_k: float = 500000.0  # Assumed rigid drawer stop, N/m (500 N/mm).
    jam_at: float = 0.020  # Assumed debris halfway along the drawer travel, m.
    jam_k: float = 20000.0  # Assumed compliant debris, N/m (20 N/mm).
    contact_k: float = 20000.0  # Assumed rubber pad on the hand, N/m (20 N/mm).
    damping_ratio: float = 0.05  # Assumed light damping, same ratio as rack.
    # Base drive: the design sheet lists a 550 kg cm class brushed DC gear motor on a
    # spur-gear turntable. Motor and gear values are assumed so that 17 A (the team's
    # 12 V current estimate) gives about 54 N m, i.e. 550 kg cm.
    kt: float = 0.04  # Assumed motor torque constant, N m/A.
    gear: float = 100.0  # Assumed total ratio, gearbox times turntable.
    efficiency: float = 0.8  # Assumed gear train efficiency.
    rotor_j: float = 5e-5  # Assumed rotor inertia of a 200 W class motor, kg m^2.
    current_tau: float = 0.0005  # Assumed current lag, same 0.5 ms as rack.
    max_current: float = 13.0  # Cytron MD13S continuous rating, A (team's driver choice).
    joint_friction: float = 1.0  # Assumed turntable bearing and gear friction, N m.
    encoder_counts: int = 8192  # REV-11-3174 quadrature counts/rev (x4 decoding).
    # https://www.revrobotics.com/rev-11-3174/ specifies 2048 cycles, 8192 counts.
    # Motor-shaft mounting remains assumed; joint mounting omits the gear factor.

    @property
    def radius(self):
        return (self.upper_length * math.cos(math.radians(self.upper_deg))
                + self.fore_length * math.cos(math.radians(self.fore_deg))
                + self.tool_length)

    @property
    def max_tip_force(self):
        return self.kt * self.gear * self.efficiency * self.max_current / self.radius

    @property
    def counts_per_m(self):
        return self.encoder_counts * self.gear / (2 * math.pi * self.radius)


ARM = ArmParams()


def _box(body, name, pos, size, mass=0, material="alu"):
    # Assumed visual cross sections and fixtures; only moving link mass affects physics.
    return body.add_geom(name=name, type=mj.mjtGeom.mjGEOM_BOX, pos=pos,
                         size=size, mass=mass, material=material,
                         contype=0, conaffinity=0)


def _load_look(path):
    """Load an optional display module with rover_visuals() and arm_visuals()."""
    spec = importlib.util.spec_from_file_location("sim3d_look", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


DEFAULT_LOOK = Path(__file__).parent / "look.py"


def build(case, jam=False, parameters=None, look=DEFAULT_LOOK):
    """Build only the selected mechanism; rack parameters can override Params defaults.

    `look` is a display module (default: sim3d/look.py, drawn from the team's assembly
    CAD picture). It only adds massless, non-colliding geoms, so the physics is the same
    with or without it; look=None keeps the plain display."""
    if case not in ("arm", "rack"):
        raise ValueError(f"Unknown case: {case}")
    p = parameters or Params()
    look = _load_look(look) if look else None
    tree = ET.parse(ASSETS / "rover_scene.xml")
    root = tree.getroot()
    if look is not None:
        # Keep only the chassis inertia and the control board; the module draws the rest.
        chassis_el = next(b for b in root.iter("body") if b.get("name") == "chassis")
        for child in list(chassis_el):
            if not (child.tag == "inertial"
                    or (child.tag == "body" and child.get("name") == "control_board")):
                chassis_el.remove(child)
    # Remove file-backed terrain BEFORE parsing with MjSpec: terrain.bin is unnecessary.
    for parent in root.iter():
        for child in list(parent):
            if (child.tag in ("joint", "freejoint", "hfield")
                    or (child.tag == "geom" and (
                        child.get("type") == "hfield"
                        or child.get("name", "")[4:].isdigit()
                        and child.get("name", "").startswith("rock")))):
                parent.remove(child)
    for tag in ("equality", "actuator", "sensor"):
        node = root.find(tag)
        if node is not None:
            root.remove(node)
    for mesh in list(root.find("asset")):
        if (mesh.tag == "mesh" and mesh.get("name", "").startswith("rock")
                and mesh.get("name", "")[4:].isdigit()):
            root.find("asset").remove(mesh)
    for geom in root.iter("geom"):
        geom.set("contype", "0")
        geom.set("conaffinity", "0")
    compiler = root.find("compiler")
    compiler.set("meshdir", str(ASSETS.resolve()))
    compiler.set("texturedir", str(ASSETS.resolve()))
    compiler.set("inertiafromgeom", "auto")
    spec = mj.MjSpec.from_string(ET.tostring(root, encoding="unicode"))
    spec.option.timestep = p.dt
    spec.option.integrator = mj.mjtIntegrator.mjINT_EULER
    # Assumed visual ground extent (8 m half-width) and 0.1 m display grid size.
    spec.worldbody.add_geom(name="ground", type=mj.mjtGeom.mjGEOM_PLANE,
                            size=[8, 8, .1], material="sand", contype=0, conaffinity=0)
    if look is not None:
        layout = look.rover_visuals(spec, spec.body("chassis"))
        if hasattr(look, "scene_visuals"):
            look.scene_visuals(spec)
        # Park the chassis so the wheels touch the ground; move the board to the open bay.
        spec.body("chassis").pos = [0, 0, -layout["wheel_contact_z"]]
        spec.body("control_board").pos = list(layout["board_pos"])
    # Assumed preliminary camera poses; renderer owner will tune these.
    spec.worldbody.add_camera(name="overview", pos=[2.5, -3, 1.8],
                              xyaxes=[.768, .640, 0, -.230, .276, .933])
    spec.worldbody.add_camera(name="rack_view", pos=[1.0, -2.8, 1.4],
                              xyaxes=[1, 0, 0, 0, .5, .866])
    # Markers on the board connectors this test uses (display group 5), at the footprint
    # positions in the team's KiCad file (5db1aab) relative to the board centre:
    # J7 motor 1 (PWM1, DIR1), J6 encoder 1, J27 force sensor.
    board = spec.body("control_board")
    for (x, y), rgba in (((-.0271, -.0343), [1, .45, .05, .8]), ((-.0271, -.0436), [.15, .5, 1, .8]),
                         ((.0467, -.0127), [.1, .85, .3, .8])):
        board.add_geom(type=mj.mjtGeom.mjGEOM_BOX, size=[.0075, .0045, .004], pos=[x, y, .004],
                       rgba=rgba, group=5, contype=0, conaffinity=0, mass=0)
    if case == "rack":
        # Assumed bench height 0.75 m and visual dimensions; travel physics uses Params.
        bench = spec.worldbody.add_body(name="bench", pos=[0, -1.3, .75])
        _box(bench, "bench_top", [.15, 0, -.08], [.6, .25, .025], material="slot")
        for x in (-.4, .7):  # four legs down to the ground
            for y in (-.22, .22):
                _box(bench, "", [x, y, -.4275], [.02, .02, .3225], material="alu")
        rack = bench.add_body(name="rack", pos=[p.end_stop / 1000, 0, 0])
        for y in (-.12, .12):
            _box(rack, "", [0, y, .30], [.015, .015, .38], material="motor")
            _box(rack, "", [.75, y, .30], [.015, .015, .38], material="motor")
            _box(rack, "", [.375, y, .66], [.39, .012, .012], material="motor")
            _box(bench, "", [.15, y, -.035], [.5, .012, .012], material="steel")
        carriage = bench.add_body(name="axis_carriage")
        carriage.add_joint(name="axis_slide", type=mj.mjtJoint.mjJNT_SLIDE,
                           axis=[1, 0, 0], armature=p.rotor_j * (2*math.pi/p.lead)**2)
        _box(carriage, "pusher", [-.05, 0, 0], [.05, .08, .04], p.carriage_mass, material="petg")
        _box(carriage, "load_cell", [-.015, 0, 0], [.015, .03, .025], material="steel")
        server = bench.add_body(name="server")
        server.add_joint(name="server_slide", type=mj.mjtJoint.mjJNT_SLIDE, axis=[1, 0, 0])
        _box(server, "server_case", [.10, 0, 0], [.10, .10, .04], p.server_mass, material="battery")
        if jam:
            _box(bench, "jam_obstacle", [p.x_jam/1000+.20, .11, 0], [.01, .02, .05], material="petg")
        names = dict(drive_joint="axis_slide", load_joint="server_slide")
    else:
        a = ARM
        blue, gray, dark, yellow = ([.33, .58, .82, 1], [.62, .63, .65, 1],
                                    [.20, .21, .23, 1], [.95, .72, .10, 1])

        def vis(body, kind, rgba, mass=0.0, group=4, **kw):
            # Display shapes follow the team's CAD picture; only the masses are physics.
            # Group 4 lets a renderer hide the arm and panel, e.g. to show the board below.
            return body.add_geom(type=kind, rgba=rgba, mass=mass, contype=0, conaffinity=0,
                                 group=group, **kw)

        BOX, CYL, CAP = mj.mjtGeom.mjGEOM_BOX, mj.mjtGeom.mjGEOM_CYLINDER, mj.mjtGeom.mjGEOM_CAPSULE

        def carrier(body, kind, m, **kw):
            # Invisible mass carrier (group 3): the physics does not depend on the display.
            return body.add_geom(type=kind, rgba=[0, 0, 0, 0], mass=m, contype=0, conaffinity=0,
                                 group=3, **kw)

        # Turntable on the frame top, near the middle, as in the team's CAD picture.
        x0 = .10
        base = spec.body("chassis").add_body(name="arm_base", pos=[x0, 0, .10])
        base.add_joint(name="arm_yaw", type=mj.mjtJoint.mjJNT_HINGE,
                       axis=[0, 0, 1], armature=a.rotor_j*a.gear**2,
                       frictionloss=a.joint_friction)
        up, fo = math.radians(a.upper_deg), math.radians(a.fore_deg)
        upper_end = [a.upper_length*math.cos(up), 0, a.upper_length*math.sin(up)]
        fore_end = [a.fore_length*math.cos(fo), 0, a.fore_length*math.sin(fo)]
        upper = base.add_body(name="upper_arm", pos=[0, 0, a.shoulder_height])
        fore = upper.add_body(name="forearm", pos=upper_end)
        tool = fore.add_body(name="arm_tool", pos=fore_end)
        carrier(base, CYL, a.turret_mass, size=[.09, .03, 0], pos=[0, 0, .03])
        carrier(upper, CAP, a.upper_mass, size=[.012, 0, 0], fromto=[0, 0, 0, *upper_end])
        carrier(fore, CYL, a.elbow_actuator_mass, size=[.055, .05, 0], euler=[90, 0, 0])
        carrier(fore, BOX, a.fore_mass, size=[.04, .008, 0],
                fromto=[0, .02, 0, fore_end[0], .02, fore_end[2]])
        carrier(tool, BOX, a.wrist_actuator_mass, size=[.035, .03, .035])
        carrier(tool, BOX, a.tool_mass, size=[a.tool_length/2, .018, .03],
                pos=[a.tool_length/2, 0, 0])
        if look is not None:
            look.arm_visuals(spec, base, upper, fore, tool, dict(
                upper_end=upper_end, fore_end=fore_end, tool_length=a.tool_length,
                shoulder_height=a.shoulder_height))
        else:
            vis(base, CYL, gray, size=[.09, .03, 0], pos=[0, 0, .03])
            vis(base, CYL, dark, size=[.11, .006, 0], pos=[0, 0, .006])  # driven spur gear
            vis(base, BOX, gray, size=[.03, .05, .05], pos=[0, 0, .09])  # shoulder mount
            for side in (-.035, .035):  # plain fallback display: two rails with braces
                vis(upper, CAP, blue, size=[.008, 0, 0],
                    fromto=[0, side, 0, upper_end[0], side, upper_end[2]])
            for k in range(6):
                s0, s1 = k/6, (k+1)/6
                vis(upper, CAP, blue, size=[.005, 0, 0],
                    fromto=[upper_end[0]*s0, -.035 if k % 2 else .035, upper_end[2]*s0,
                            upper_end[0]*s1, .035 if k % 2 else -.035, upper_end[2]*s1])
            vis(upper, CYL, yellow, size=[.045, .05, 0], euler=[90, 0, 0])  # shoulder motor
            vis(fore, CYL, dark, size=[.055, .05, 0], euler=[90, 0, 0])
            for side in (-.02, .02):
                vis(fore, BOX, gray, size=[.04, .008, 0],
                    fromto=[0, side, 0, fore_end[0], side, fore_end[2]])
            vis(fore, CYL, dark, size=[.035, .03, 0], euler=[90, 0, 0],
                pos=[fore_end[0]*.55, 0, fore_end[2]*.55])  # belt pulley on the forearm
            vis(tool, BOX, yellow, size=[.035, .03, .035])
            vis(tool, BOX, yellow, size=[a.tool_length/2, .018, .03], pos=[a.tool_length/2, 0, 0])
        _box(tool, "tool_pad", [a.tool_length, .022, 0], [.015, .004, .015], material="tpu")
        chassis_z = spec.body("chassis").pos[2]
        z = chassis_z + .10 + a.shoulder_height + upper_end[2] + fore_end[2]
        xm = x0 + a.radius
        y_front = .026 + a.gap  # drawer front face at the start
        wall_y = y_front + a.end + .03  # closed drawer front stands 30 mm proud (assumed)
        # A drawer on a mock lander panel, sliding along +world Y (tangent to the hand's arc).
        module = spec.worldbody.add_body(name="module", pos=[xm, y_front+.020, z])
        module.add_joint(name="module_slide", type=mj.mjtJoint.mjJNT_SLIDE,
                         axis=[0, 1, 0], frictionloss=a.friction)
        module.add_geom(name="module_case", type=BOX, size=[.030, .020, .020],
                        mass=a.module_mass, rgba=[.75, .76, .78, 0], contype=0, conaffinity=0)
        vis(module, BOX, [.22, .45, .78, 1], size=[.07, .004, .045], pos=[0, -.016, 0])  # front
        vis(module, BOX, [.12, .13, .15, 1], size=[.03, .006, .005], pos=[0, -.026, -.038])  # handle
        vis(module, BOX, [.70, .74, .80, 1], size=[.05, .07, .035], pos=[0, .058, 0])  # body
        panel = spec.worldbody.add_body(name="panel", pos=[xm, wall_y, 0])
        wall, frame, body = [.90, .90, .91, 1], [.30, .34, .40, 1], [.80, .81, .83, 1]
        # A mock lander equipment bay on four legs; the drawer opens out of its front face.
        w, h, depth = .26, .19, .24  # half width (x), half height (z), depth (y)
        for sx in (-1, 1):  # front face left and right of the 110 x 100 mm opening
            vis(panel, BOX, wall, size=[(w - .055) / 2, .0075, h], pos=[sx * (w + .055) / 2, .0075, z])
        for sz in (-1, 1):  # front face above and below the opening
            vis(panel, BOX, wall, size=[.055, .0075, (h - .05) / 2], pos=[0, .0075, z + sz * (h + .05) / 2])
        vis(panel, BOX, body, size=[w, depth / 2, h], pos=[0, .015 + depth / 2, z])
        vis(panel, BOX, frame, size=[w + .006, depth / 2 + .01, .008], pos=[0, .015 + depth / 2, z + h + .008])
        for sx in (-1, 1):
            for y in (.03, depth - .01):  # legs down to the ground
                vis(panel, BOX, [.74, .76, .79, 1], size=[.012, .012, (z - h) / 2],
                    pos=[sx * (w - .02), y, (z - h) / 2])
        # Drawer housing standing 25 mm out of the wall around the drawer front.
        for x in (-.085, .085):
            vis(panel, BOX, frame, size=[.01, .0125, .075], pos=[x, -.0125, z])
        for dz in (-.065, .065):
            vis(panel, BOX, frame, size=[.095, .0125, .01], pos=[0, -.0125, z + dz])
        if jam:
            # Debris inside the housing: the drawer front meets it after jam_at.
            debris_y0 = y_front + .008 + a.jam_at - wall_y
            vis(panel, BOX, [.98, .42, .05, 1], size=[.009, -debris_y0/2, .03],
                pos=[.063, debris_y0/2, z])
        # Direct solref is an acceleration law, not N/m. Calibrate using reduced
        # generalized mass at the initial contact point after compiling the tree.
        preliminary = spec.compile()
        data = mj.MjData(preliminary)
        mj.mj_forward(preliminary, data)
        inertia = data.M[0]  # First DOF is the sole yaw joint (all rover joints removed).
        effective_mass = inertia/a.radius**2
        reduced = effective_mass*a.module_mass/(effective_mass+a.module_mass)
        damping = 2*a.damping_ratio*math.sqrt(a.contact_k*reduced)
        # MuJoCo regularizes each contact using body_invweight0, rather than
        # the joint-space reduced mass. A yawed box touches along its leading
        # vertical edge (two points). Split the physical pad stiffness across
        # that manifold: sum(F) = 2*solref_k*penetration/((1-d)*diagA).
        inverse_weight = sum(preliminary.body_invweight0[preliminary.body(n).id, 0]
                             for n in ("arm_tool", "module"))
        impedance = .001  # Assumed low impedance approximates a compliant penalty spring.
        scale = (1-impedance)*inverse_weight/2
        # Account for impedance and the two-point contact patch compliance.
        # No tangential contact friction: the module has a separate rail law.
        spec.add_pair(name="pad_module", geomname1="tool_pad", geomname2="module_case",
                      condim=1, solimp=[impedance, impedance, .001, .5, 2],
                      solref=[-a.contact_k*scale, -damping*scale])
        names = dict(drive_joint="arm_yaw", load_joint="module_slide", radius=a.radius,
                     contact_geoms=("tool_pad", "module_case"), damping=damping)
    return spec.compile(), names
