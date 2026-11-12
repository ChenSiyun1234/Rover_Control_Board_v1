"""Display-only model of the parked team rover and its arm: no mass, no contacts, no joints.

Shapes follow the team's full-assembly CAD picture of 2026-11-01: a black 40 mm extrusion
box frame on orange posts, blue-grey truss rocker-bogies, a light-blue truss differential
bar, lugged wheels with blue rims and spoked hubs, and a grey plate arm with belts,
pulleys, stepper motors and a yellow wrist. Proportions are read off the picture, not
measured from CAD. Meshes are generated here with trimesh and shapely. The scene styling
(grid floor, gradient sky, headlight, soft shadows) follows the MuJoCo Menagerie scenes.
"""

import math

import mujoco as mj
import numpy as np
import trimesh
from shapely import affinity
from shapely.geometry import LineString, Point, Polygon, box as sbox
from shapely.ops import unary_union

BOX, CYL, CAP, MESH = (mj.mjtGeom.mjGEOM_BOX, mj.mjtGeom.mjGEOM_CYLINDER,
                       mj.mjtGeom.mjGEOM_CAPSULE, mj.mjtGeom.mjGEOM_MESH)

FRAME_X, FRAME_Y = 0.29, 0.21  # half length and half width of the box frame
TOP_Z, BOTTOM_Z = 0.10, -0.18  # top and bottom faces of the frame
RAIL = 0.02  # half section of the 40 mm extrusion
SIDE_Y = 0.262  # plane of the rocker-bogie plates
WHEEL_Y, WHEEL_Z = 0.385, -0.30  # wheel centres
WHEEL_R, WHEEL_HALF_W, LUG = 0.118, 0.068, 0.008
WHEEL_X = (-0.44, -0.07, 0.40)  # bogie rear, bogie front, rocker front
DIFF_X = -0.04  # differential bar pivot and rocker towers
ARM_XY = (0.10, 0.0)
BOARD_POS = (-0.16, 0.0, -0.06)
BASE_MOTOR = (ARM_XY[0] + 0.118, -0.045)  # base gear motor and pinion, fixed to the frame

LAYOUT = {
    "frame_top_z": TOP_Z,
    "arm_base_xy": ARM_XY,
    "board_pos": BOARD_POS,
    "wheel_contact_z": WHEEL_Z - WHEEL_R - LUG,
    "pose_deg": {"upper": 78, "fore": 18},
}

# name: rgba, specular, shininess, reflectance
MATERIALS = {
    "black": ([.07, .07, .08, 1], .55, .55, 0),
    "orange": ([.80, .45, .14, 1], .35, .45, 0),
    "deck": ([.17, .18, .20, 1], .25, .3, 0),
    "alu": ([.74, .76, .79, 1], .75, .65, .03),
    "steelblue": ([.43, .53, .63, 1], .45, .5, 0),
    "sky": ([.62, .81, .95, 1], .25, .35, 0),
    "navy": ([.14, .24, .56, 1], .5, .5, 0),
    "rubber": ([.05, .05, .055, 1], .08, .1, 0),
    "rim": ([.12, .23, .64, 1], .6, .6, 0),
    "silver": ([.84, .85, .87, 1], .9, .8, .05),
    "gray": ([.66, .67, .70, 1], .55, .5, 0),
    "dark": ([.19, .20, .22, 1], .5, .5, 0),
    "yellow": ([.96, .74, .12, 1], .45, .5, 0),
    "peg": ([.72, .70, .82, 1], .5, .5, 0),
    "belt": ([.09, .09, .10, 1], .15, .2, 0),
    "brass": ([.83, .66, .27, 1], .8, .7, 0),
    "red": ([.72, .10, .10, 1], .4, .4, 0),
    "label": ([.96, .76, .10, 1], .2, .2, 0),
    "wire_orange": ([1, .45, .05, 1], .3, .3, 0),
    "wire_blue": ([.15, .5, 1, 1], .3, .3, 0),
    "wire_green": ([.1, .85, .3, 1], .3, .3, 0),
}

_KITS = {}


class _Kit:
    """Materials and meshes registered once per spec."""

    def __init__(self, spec):
        self.spec, self.meshes = spec, set()
        for name, (rgba, specular, shininess, reflectance) in MATERIALS.items():
            spec.add_material(name="lk_" + name, rgba=rgba, specular=specular,
                              shininess=shininess, reflectance=reflectance)

    def mesh(self, name, make):
        if name not in self.meshes:
            tm = make()
            self.spec.add_mesh(name="lk_" + name, uservert=tm.vertices.flatten().tolist(),
                               userface=tm.faces.flatten().tolist())
            self.meshes.add(name)
        return "lk_" + name


def _kit(spec):
    if id(spec) not in _KITS:
        _KITS[id(spec)] = _Kit(spec)
    return _KITS[id(spec)]


def _g(body, kind, mat, group=0, **kw):
    return body.add_geom(type=kind, material="lk_" + mat, group=group, contype=0,
                         conaffinity=0, mass=0, **kw)


def _m(body, mesh, mat, group=0, pos=(0, 0, 0), quat=(1, 0, 0, 0)):
    return body.add_geom(type=MESH, meshname=mesh, material="lk_" + mat, group=group,
                         contype=0, conaffinity=0, mass=0, pos=list(pos), quat=list(quat))


def _quat_axis(axis, angle):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    return [math.cos(angle / 2), *(axis * math.sin(angle / 2))]


def _quat_z_to(d):
    """Quaternion turning +z onto direction d."""
    d = np.asarray(d, float) / np.linalg.norm(d)
    c = d[2]
    if c > 1 - 1e-9:
        return [1, 0, 0, 0]
    if c < -1 + 1e-9:
        return [0, 1, 0, 0]
    return _quat_axis(np.cross([0, 0, 1], d), math.acos(c))


# ---- 2-D outlines and meshes ---------------------------------------------------------

def _extrude(poly, thickness):
    """Extrude a shapely outline (metres) along z, centred on z = 0."""
    parts = list(poly.geoms) if hasattr(poly, "geoms") else [poly]
    m = trimesh.util.concatenate([trimesh.creation.extrude_polygon(p, thickness)
                                  for p in parts if p.area > 1e-9])
    m.apply_translation([0, 0, -thickness / 2])
    return m


def _in_xz(m):
    """Lay an outline drawn in (x, y) into the x-z plane, thickness along y."""
    m.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0]))
    return m


def _stadium(p0, p1, r):
    return LineString([p0, p1]).buffer(r, quad_segs=16)


def _tslot():
    """40 mm T-slot extrusion section with four slots and a centre bore."""
    cuts = [Point(0, 0).buffer(.0034, quad_segs=8)]
    for k in range(4):
        slot = unary_union([sbox(-.0041, .0135, .0041, .021), sbox(-.0082, .0115, .0082, .0165)])
        cuts.append(affinity.rotate(slot, 90 * k, origin=(0, 0)))
    outer = sbox(-RAIL, -RAIL, RAIL, RAIL).buffer(-.0015).buffer(.0015, quad_segs=4)
    return outer.difference(unary_union(cuts))


def _gear(teeth, r_tip, r_root, bore=0.0):
    pts = []
    for k in range(teeth):
        a = 2 * math.pi * k / teeth
        da = 2 * math.pi / teeth
        for frac, r in ((0.0, r_root), (0.18, r_tip), (0.50, r_tip), (0.68, r_root)):
            pts.append((r * math.cos(a + frac * da), r * math.sin(a + frac * da)))
    poly = Polygon(pts)
    return poly.difference(Point(0, 0).buffer(bore)) if bore else poly


def _truss(nodes, members, chord, web, boss, holes=()):
    """Plate truss: buffered members, round bosses at the nodes, optional bores."""
    shapes = []
    for i, j, kind in members:
        shapes.append(LineString([nodes[i], nodes[j]]).buffer(chord if kind == "c" else web,
                                                             cap_style="round"))
    for n in nodes.values():
        shapes.append(Point(n).buffer(boss, quad_segs=12))
    shape = unary_union(shapes)
    for n, r in holes:
        shape = shape.difference(Point(nodes[n]).buffer(r, quad_segs=10))
    return shape


def _belt(c0, r0, c1, r1, width):
    outer = unary_union([Point(c0).buffer(r0 + .003), Point(c1).buffer(r1 + .003)]).convex_hull
    inner = unary_union([Point(c0).buffer(r0), Point(c1).buffer(r1)]).convex_hull
    return _extrude(outer.difference(inner), width)


def _stepper(size, length):
    """NEMA-style motor: chamfered square body along +z with end caps and a shaft."""
    h, c = size / 2, size * 0.14
    body = Polygon([(-h + c, -h), (h - c, -h), (h, -h + c), (h, h - c), (h - c, h),
                    (-h + c, h), (-h, h - c), (-h, -h + c)])
    core = _extrude(body, length * 0.72)
    caps = [_extrude(body.buffer(-.0008), length * 0.14) for _ in range(2)]
    caps[0].apply_translation([0, 0, length * 0.43])
    caps[1].apply_translation([0, 0, -length * 0.43])
    return core, trimesh.util.concatenate(caps)


def _revolve_y(profile, sections=72):
    """Sweep a closed (radius, y) profile loop around the y axis into a closed surface."""
    pts = np.asarray(profile, float)
    n = len(pts)
    a = np.linspace(0, 2 * math.pi, sections, endpoint=False)
    verts = np.array([[r * math.cos(t), y, r * math.sin(t)] for t in a for r, y in pts])
    faces = []
    for i in range(sections):
        for j in range(n):
            p, q = i * n + j, i * n + (j + 1) % n
            p2, q2 = ((i + 1) % sections) * n + j, ((i + 1) % sections) * n + (j + 1) % n
            faces += [[p, q, q2], [p, q2, p2]]
    m = trimesh.Trimesh(verts, np.array(faces), process=True)
    m.fix_normals()
    return m


def _tyre():
    r_in, r_out, hw, rr = .080, WHEEL_R, WHEEL_HALF_W, .012
    prof = sbox(r_in, -hw, r_out, hw).buffer(-rr).buffer(rr, quad_segs=6)
    ring = _revolve_y(np.array(prof.exterior.coords)[:-1, :2])
    lugs = []
    n = 20
    for k in range(n):
        a = 2 * math.pi * k / n
        for side in (-1, 1):
            lug = trimesh.creation.box([.022, .056, LUG * 2])
            lug.apply_transform(trimesh.transformations.rotation_matrix(side * .35, [0, 0, 1]))
            off = (a + (math.pi / n if side > 0 else 0))
            lug.apply_translation([0, side * .032, 0])
            T = trimesh.transformations.rotation_matrix(off, [0, 1, 0])
            lug.apply_translation([0, 0, r_out + LUG * .4])
            lug.apply_transform(T)
            lugs.append(lug)
    return trimesh.util.concatenate([ring] + lugs)


def _hub():
    spokes = [Point(0, 0).buffer(.028, quad_segs=12)]
    for k in range(5):
        a = 2 * math.pi * k / 5
        arc = [(r * math.cos(a + .55 * (r - .02) / .06), r * math.sin(a + .55 * (r - .02) / .06))
               for r in np.linspace(.02, .074, 6)]
        spokes.append(LineString(arc).buffer(.0065, quad_segs=6))
    shape = unary_union(spokes).difference(Point(0, 0).buffer(.008))
    return _in_xz(_extrude(shape, .010))


# ---- rover --------------------------------------------------------------------------

def _rail(kit, body, p0, p1, mat="black", group=0):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    length = float(np.linalg.norm(p1 - p0))
    mesh = kit.mesh(f"rail_{length * 1000:.0f}", lambda: _extrude(_tslot(), length))
    _m(body, mesh, mat, group, pos=(p0 + p1) / 2, quat=_quat_z_to(p1 - p0))


def _frame(kit, ch):
    x, y, t, b = FRAME_X - RAIL, FRAME_Y - RAIL, TOP_Z - RAIL, BOTTOM_Z + RAIL
    for z in (t, b):
        for s in (-1, 1):
            _rail(kit, ch, (-FRAME_X, s * y, z), (FRAME_X, s * y, z))
            _rail(kit, ch, (s * x, -y + RAIL, z), (s * x, y - RAIL, z))
    _rail(kit, ch, (0, -y + RAIL, t), (0, y - RAIL, t))  # cross member under the arm plate
    for px in (-x, 0.0, x):
        for s in (-1, 1):
            _rail(kit, ch, (px, s * y, b + RAIL), (px, s * y, t - RAIL), "orange")
    # electronics tray, arm plate over the front half, battery and the arm's DC driver
    _g(ch, BOX, "deck", size=[FRAME_X - 2 * RAIL, FRAME_Y - 2 * RAIL, .003], pos=[0, 0, -.085])
    _g(ch, BOX, "alu", size=[FRAME_X / 2, FRAME_Y - .002, .003],
       pos=[FRAME_X / 2, 0, TOP_Z - .003])
    _g(ch, BOX, "black", size=[.09, .065, .035], pos=[.13, .06, -.047])
    _g(ch, BOX, "label", size=[.0905, .0655, .008], pos=[.13, .06, -.047])
    _g(ch, BOX, "red", size=[.037, .030, .0012], pos=[.20, -.10, -.075])  # DC driver board
    _g(ch, BOX, "dark", size=[.012, .022, .009], pos=[.22, -.10, -.065])
    for dx in (-.045, .045):  # standoffs under the battery strap
        _g(ch, CYL, "brass", size=[.003, .019], pos=[.13 + dx, -.03, -.063])


def _base_drive(kit, ch):
    x, y = BASE_MOTOR
    pinion = kit.mesh("pinion", lambda: _extrude(_gear(14, .024, .019, .004), .012))
    _m(ch, pinion, "dark", pos=(x, y, TOP_Z + .006))
    _g(ch, CYL, "gray", size=[.026, .03], pos=[x, y, TOP_Z - .03])  # gearbox under the plate
    _g(ch, CYL, "dark", size=[.024, .045], pos=[x, y, TOP_Z - .105])  # motor can


def _wires(ch):
    """Motor, encoder and force-sensor leads from the board connectors to the arm base."""
    bx, by, bz = BOARD_POS
    top = bz + .005
    # Each lead lies flat on the board until it clears the board edge (half size 50 mm),
    # so the top-down board view shows it plugged in rather than rising at the camera.
    for (cx, cy), exit_xy, mat, end in (
            ((-.0271, -.0343), (-.0271, -.064), "wire_orange", (BASE_MOTOR[0], BASE_MOTOR[1], -.03)),
            ((-.0271, -.0436), (-.0331, -.068), "wire_blue", (BASE_MOTOR[0], BASE_MOTOR[1] - .02, -.03)),
            ((.0467, -.0127), (.064, -.0127), "wire_green", (ARM_XY[0], ARM_XY[1] - .01, TOP_Z - .01))):
        ex, ey = bx + exit_xy[0], by + exit_xy[1]
        p = [(bx + cx, by + cy, top), (ex, ey, top), (ex, ey, -.03), (end[0] - .05, end[1], -.03), end]
        for a, b in zip(p, p[1:]):
            _g(ch, CAP, mat, size=[.0026, 0, 0], fromto=[*a, *b])


def _rocker_bogie(kit, ch, s):
    rocker = kit.mesh("rocker", lambda: _in_xz(_extrude(_truss(
        {"P": (DIFF_X, .02), "T": (DIFF_X, TOP_Z + .03), "F": (.40, -.30), "B": (-.25, -.18),
         "K": (.12, -.24), "Q": (.18, -.14), "U": (-.15, -.08)},
        [("T", "P", "c"), ("P", "F", "c"), ("P", "B", "c"), ("B", "K", "c"), ("K", "F", "c"),
         ("P", "K", "w"), ("Q", "K", "w"), ("U", "K", "w")],
        .013, .008, .02, holes=[("P", .008), ("B", .008), ("F", .009)]), .008)))
    bogie = kit.mesh("bogie", lambda: _in_xz(_extrude(_truss(
        {"B": (-.25, -.18), "R": (-.44, -.30), "S": (-.07, -.30), "J": (-.255, -.30)},
        [("B", "R", "c"), ("B", "S", "c"), ("R", "S", "c"), ("B", "J", "w")],
        .012, .008, .02, holes=[("B", .008), ("R", .009), ("S", .009)]), .008)))
    y = s * SIDE_Y
    _m(ch, rocker, "steelblue", pos=(0, y, 0))
    _m(ch, bogie, "steelblue", pos=(0, y + s * .012, 0))
    for px, pz in ((DIFF_X, .02), (-.25, -.18)):  # pivot shafts
        _g(ch, CYL, "silver", size=[.007, .03], pos=[px, y - s * .02, pz], euler=[90, 0, 0])
    for wx in WHEEL_X:
        _g(ch, CYL, "silver", size=[.009, .055], pos=[wx, s * (SIDE_Y + .06), WHEEL_Z],
           euler=[90, 0, 0])
        _g(ch, CYL, "dark", size=[.05, .022], pos=[wx, s * (WHEEL_Y - WHEEL_HALF_W - .018), WHEEL_Z],
           euler=[90, 0, 0])  # hub motor
        _wheel(kit, ch, wx, s)
    # link from the rocker tower to the end of the differential bar
    _g(ch, CAP, "navy", size=[.006, 0, 0],
       fromto=[DIFF_X, y, TOP_Z + .03, DIFF_X, s * .30, TOP_Z + .045])


def _wheel(kit, ch, x, s):
    tyre = kit.mesh("tyre", _tyre)
    hub = kit.mesh("hub", _hub)
    rim = kit.mesh("rim", lambda: _in_xz(_extrude(
        Point(0, 0).buffer(.081, quad_segs=24).difference(Point(0, 0).buffer(.066, quad_segs=24)), .012)))
    y = s * WHEEL_Y
    _m(ch, tyre, "rubber", pos=(x, y, WHEEL_Z))
    _g(ch, CYL, "dark", size=[.079, .05], pos=[x, y, WHEEL_Z], euler=[90, 0, 0])
    for face in (-1, 1):
        _m(ch, rim, "rim", pos=(x, y + face * (WHEEL_HALF_W - .012), WHEEL_Z))
        _m(ch, hub, "silver", pos=(x, y + face * (WHEEL_HALF_W - .016), WHEEL_Z))


def _differential(kit, ch):
    nodes = {f"a{k}": (-.022, -.30 + .1 * k) for k in range(7)}
    nodes.update({f"b{k}": (.022, -.25 + .1 * k) for k in range(6)})
    nodes["c"] = (0, 0)
    members = [(f"a{k}", f"a{k + 1}", "c") for k in range(6)]
    members += [(f"b{k}", f"b{k + 1}", "c") for k in range(5)]
    members += [(f"a{k}", f"b{k}", "w") for k in range(6)] + [(f"b{k}", f"a{k + 1}", "w") for k in range(6)]
    bar = kit.mesh("diffbar", lambda: _extrude(affinity.rotate(
        _truss(nodes, members, .007, .005, .009, holes=[("c", .006)]), 0), .008))
    # the outline is drawn with x across the bar and y along it, which matches the chassis axes
    _m(ch, bar, "sky", pos=(DIFF_X, 0, TOP_Z + .045))
    _g(ch, CYL, "silver", size=[.012, .022], pos=[DIFF_X, 0, TOP_Z + .022])  # centre pivot
    _g(ch, BOX, "dark", size=[.03, .03, .008], pos=[DIFF_X, 0, TOP_Z + .004])


def rover_visuals(spec, chassis):
    """Draw the parked rover on the chassis body; returns LAYOUT."""
    kit = _kit(spec)
    _frame(kit, chassis)
    _base_drive(kit, chassis)
    _wires(chassis)
    _differential(kit, chassis)
    for s in (-1, 1):
        _rocker_bogie(kit, chassis, s)
    return LAYOUT


# ---- arm ----------------------------------------------------------------------------

def _along(end, fraction, normal=0.0):
    length = math.hypot(end[0], end[2])
    ux, uz = end[0] / length, end[2] / length
    return end[0] * fraction - uz * normal, end[2] * fraction + ux * normal


def _plate(kit, name, end, r, holes):
    """Two-dimensional arm plate from (0, 0) to end in the x-z plane, with lightening slots."""
    def make():
        e = (end[0], end[2])
        shape = _stadium((0, 0), e, r)
        for f0, f1, w in holes:
            shape = shape.difference(_stadium((e[0] * f0, e[1] * f0), (e[0] * f1, e[1] * f1), w))
        for c in ((0, 0), e):
            shape = shape.difference(Point(c).buffer(.009, quad_segs=10))
        return _in_xz(_extrude(shape, .006))
    return kit.mesh(name, make)


def arm_visuals(spec, base, upper, fore, tool, dims):
    """Add the arm's look to the four bodies the caller built (all group 4)."""
    kit, g = _kit(spec), 4
    ue, fe, sh = dims["upper_end"], dims["fore_end"], dims.get("shoulder_height", .12)
    # turntable: driven spur gear, bearing ring, top disc
    gear = kit.mesh("ring_gear", lambda: _extrude(_gear(64, .104, .098, .07), .012))
    _m(base, gear, "dark", g, pos=(0, 0, .006))
    _g(base, CYL, "silver", g, size=[.072, .006], pos=[0, 0, .018])
    _g(base, CYL, "gray", g, size=[.085, .012], pos=[0, 0, .036])
    # shoulder yoke
    yoke = kit.mesh("yoke", lambda: _in_xz(_extrude(unary_union([
        sbox(-.055, .04, .055, sh), Point(0, sh).buffer(.055, quad_segs=16)]).difference(
        unary_union([Point(0, sh).buffer(.012), Point(0, sh - .05).buffer(.018)])), .008)))
    for y in (-.052, .052):
        _m(base, yoke, "gray", g, pos=(0, y, 0))
    _g(base, BOX, "gray", g, size=[.006, .048, .03], pos=[-.05, 0, .075])
    # shoulder drive on the near side: yellow planetary gearbox and a NEMA 23
    _g(base, CYL, "yellow", g, size=[.03, .022], pos=[0, -.078, sh], euler=[90, 0, 0])
    core, caps = _stepper(.057, .056)
    n23 = kit.mesh("nema23", lambda: core)
    n23c = kit.mesh("nema23_caps", lambda: caps)
    q = _quat_axis([1, 0, 0], math.pi / 2)
    _m(base, n23, "dark", g, pos=(0, -.128, sh), quat=q)
    _m(base, n23c, "silver", g, pos=(0, -.128, sh), quat=q)

    # upper arm: twin plates, standoffs, elbow motor and belt on the far side
    up = _plate(kit, "upper_plate", ue, .045, [(.18, .40, .016), (.55, .80, .016)])
    for y in (-.034, .034):
        _m(upper, up, "gray", g, pos=(0, y, 0))
    for f, n in ((.30, .028), (.30, -.028), (.68, .028), (.68, -.028), (.92, 0)):
        px, pz = _along(ue, f, n)
        _g(upper, CYL, "peg", g, size=[.006, .031], pos=[px, 0, pz], euler=[90, 0, 0])
    mx, mz = _along(ue, .25)
    _m(upper, n23, "dark", g, pos=(mx, .093, mz), quat=q)  # elbow motor
    _m(upper, n23c, "silver", g, pos=(mx, .093, mz), quat=q)
    # Motor mount: a plate on four standoffs from the arm plate, clear of the belt.
    uq = _quat_axis([0, 1, 0], -math.atan2(ue[2], ue[0]))
    _g(upper, BOX, "gray", g, size=[.034, .002, .034], pos=[mx, .063, mz], quat=uq)
    for df in (-.027, .027):
        for dn in (-.027, .027):
            px, pz = _along(ue, .25 + df / math.hypot(ue[0], ue[2]), dn)
            _g(upper, CYL, "peg", g, size=[.004, .0125], pos=[px, .0495, pz], euler=[90, 0, 0])
    small = kit.mesh("pulley_s", lambda: _in_xz(_extrude(_gear(18, .020, .0185, .004), .012)))
    big = kit.mesh("pulley_l", lambda: _in_xz(_extrude(_gear(48, .056, .0545, .012), .012)))
    _m(upper, small, "silver", g, pos=(mx, .051, mz))
    belt = kit.mesh("belt_upper", lambda: _in_xz(_belt((mx, mz), .020, (ue[0], ue[2]), .056, .010)))
    _m(upper, belt, "belt", g, pos=(0, .051, 0))

    # forearm: twin plates, elbow pulley, wrist motor and belt on the near side, yellow clamp
    fp = _plate(kit, "fore_plate", fe, .04, [(.16, .36, .014), (.46, .70, .014)])
    for y in (-.024, .024):
        _m(fore, fp, "gray", g, pos=(0, y, 0))
    _m(fore, big, "dark", g, pos=(0, .051, 0))
    _g(fore, CYL, "silver", g, size=[.012, .004], pos=[0, .061, 0], euler=[90, 0, 0])
    for f, n in ((.22, .026), (.22, -.026), (.58, .026), (.58, -.026)):
        px, pz = _along(fe, f, n)
        _g(fore, CYL, "peg", g, size=[.005, .021], pos=[px, 0, pz], euler=[90, 0, 0])
    wx, wz = _along(fe, .30)
    core17, caps17 = _stepper(.042, .040)
    n17 = kit.mesh("nema17", lambda: core17)
    n17c = kit.mesh("nema17_caps", lambda: caps17)
    _m(fore, n17, "dark", g, pos=(wx, -.058, wz), quat=q)
    _m(fore, n17c, "silver", g, pos=(wx, -.058, wz), quat=q)
    wsmall = kit.mesh("pulley_w", lambda: _in_xz(_extrude(_gear(16, .016, .0148, .003), .010)))
    _m(fore, wsmall, "silver", g, pos=(wx, -.033, wz))
    _m(fore, wsmall, "silver", g, pos=(fe[0] * .97, -.033, fe[2] * .97))
    wbelt = kit.mesh("belt_fore", lambda: _in_xz(_belt((wx, wz), .016, (fe[0] * .97, fe[2] * .97), .016, .008)))
    _m(fore, wbelt, "belt", g, pos=(0, -.033, 0))
    ang = math.atan2(fe[2], fe[0])
    cq = _quat_axis([0, 1, 0], -ang)
    cx, cz = _along(fe, .88)
    _g(fore, BOX, "yellow", g, size=[.042, .029, .03], pos=[cx, 0, cz], quat=cq)
    _g(fore, BOX, "yellow", g, size=[.016, .033, .036], pos=[*_along(fe, .74)[:1], 0, _along(fe, .74)[1]], quat=cq)

    # wrist and hand: kept inside y <= 0.018, |z| <= 0.04, x <= 0.09 (the pad is the caller's)
    _g(tool, BOX, "gray", g, size=[.024, .017, .03], pos=[.004, -.002, 0])
    _g(tool, CYL, "dark", g, size=[.018, .02], pos=[.004, -.004, 0], euler=[90, 0, 0])
    _g(tool, BOX, "sky", g, size=[.012, .016, .028], pos=[.040, -.004, 0])
    _g(tool, BOX, "dark", g, size=[.017, .016, .021], pos=[.068, -.004, 0])
    _g(tool, BOX, "silver", g, size=[.0008, .015, .02], pos=[.0858, -.004, 0])
    for z in (-.022, .022):  # gripper fingers on the far side from the pad
        _g(tool, BOX, "sky", g, size=[.022, .007, .006], pos=[.064, -.032, z])
        _g(tool, BOX, "dark", g, size=[.004, .009, .006], pos=[.084, -.036, z])
    # force-sensor lead from the pad back along the arm
    _g(tool, CAP, "wire_green", g, size=[.0022, 0, 0], fromto=[.075, .016, -.01, .0, .016, -.026])
    _g(fore, CAP, "wire_green", g, size=[.0022, 0, 0],
       fromto=[fe[0], .03, fe[2] - .026, *_along(fe, .05, -.03)[:1], .03, _along(fe, .05, -.03)[1]])
    _g(upper, CAP, "wire_green", g, size=[.0022, 0, 0],
       fromto=[ue[0] - .02, .04, ue[2], .02, .04, .01])


# ---- scene --------------------------------------------------------------------------

def scene_visuals(spec):
    """Light grid floor, soft gradient sky, headlight and shadows (Menagerie-style)."""
    spec.add_texture(name="lk_grid", type=mj.mjtTexture.mjTEXTURE_2D,
                     builtin=mj.mjtBuiltin.mjBUILTIN_CHECKER, mark=mj.mjtMark.mjMARK_EDGE,
                     rgb1=[.86, .87, .89], rgb2=[.83, .845, .865], markrgb=[.70, .72, .76],
                     width=300, height=300)
    floor = spec.add_material(name="lk_floor", texrepeat=[2, 2], texuniform=True, reflectance=.12)
    tex = list(floor.textures)
    tex[mj.mjtTextureRole.mjTEXROLE_RGB] = "lk_grid"
    floor.textures = tex
    spec.geom("ground").material = "lk_floor"
    sky = spec.texture("sky")
    sky.rgb1, sky.rgb2 = [.78, .85, .93], [.97, .97, .99]
    spec.visual.headlight.ambient = [.32, .32, .32]
    spec.visual.headlight.diffuse = [.45, .45, .45]
    spec.visual.headlight.specular = [.08, .08, .08]
    spec.visual.quality.shadowsize = 4096
    spec.visual.quality.offsamples = 8
    spec.visual.rgba.haze = [.93, .95, .98, 1]
    # The ground plane makes the model look kilometres wide to the shadow map; size it to
    # the rover and its test fixtures so the shadows stay sharp.
    spec.stat.extent = 1.6
    spec.stat.center = [0.2, -0.3, 0.6]
