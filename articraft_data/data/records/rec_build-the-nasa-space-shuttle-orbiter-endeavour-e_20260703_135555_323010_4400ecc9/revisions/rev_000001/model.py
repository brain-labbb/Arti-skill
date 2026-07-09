"""NASA Space Shuttle orbiter (Endeavour) in landing configuration.

Reference: photo of Endeavour seconds before touchdown — gear down, gear doors
open, payload-bay doors closed, white upper fuselage with black TPS underside,
black nose cap and wing leading edges, double-delta wing, tall swept fin with
split rudder/speed brake, three main engine bells plus two OMS pods, body flap.

Frame: +X = nose/forward, +Y = left/port, +Z = up. Ground plane at z=0.
Real-world scale: overall length ~37.5 m incl. engine bells, span ~23.2 m.
"""

from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoltPattern,
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    LatheGeometry,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireSidewall,
    TireTread,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    mesh_from_cadquery,
    mesh_from_geometry,
    superellipse_side_loft,
)

# ---------------------------------------------------------------- constants

SIDES = (("left", 1.0), ("right", -1.0))

FUSE_NOSE_X = 17.8
FUSE_TAIL_X = -16.9

# Payload bay / doors
BAY_X_FWD = 8.6
BAY_X_AFT = -7.4
BAY_LEN = BAY_X_FWD - BAY_X_AFT
BAY_X_MID = 0.5 * (BAY_X_FWD + BAY_X_AFT)
SILL_Y = 2.35          # door hinge half-width
SILL_Z = 5.90          # door hinge height
DOOR_CROWN_Z = 6.70    # closed-door top centerline
_h = DOOR_CROWN_Z - SILL_Z
DOOR_ARC_R = (SILL_Y * SILL_Y + _h * _h) / (2.0 * _h)   # mid-surface radius
DOOR_ARC_ZC = DOOR_CROWN_Z - DOOR_ARC_R                 # arc center height
DOOR_THICK = 0.10
DOOR_PHI_HINGE = math.atan2(SILL_Y, SILL_Z - DOOR_ARC_ZC)
DOOR_PHI_SEAM = math.asin(0.02 / DOOR_ARC_R)            # ~2 cm centerline seam
DOOR_PHI_TUCK = 0.04                                    # extra arc past hinge into sill

# Real payload-bay cavity carved into the hull top (hidden by the closed doors)
BAY_CUT_X_FWD = BAY_X_FWD - 0.2
BAY_CUT_X_AFT = BAY_X_AFT + 0.2
BAY_CUT_HALF_W = 2.12
BAY_FLOOR_Z = 4.15

# Wing planform per side: (|y|, x_leading, x_trailing, z_bottom, thickness)
WING_STATIONS = (
    (2.2, 11.5, -10.7, 3.02, 1.35),
    (3.0, 8.5, -10.7, 3.05, 0.95),
    (4.3, 3.6, -10.7, 3.10, 0.70),
    (7.5, -0.5, -10.7, 3.18, 0.45),
    (10.0, -3.6, -10.7, 3.25, 0.32),
    (11.6, -5.6, -10.7, 3.30, 0.22),
)
WING_TIP_Y = WING_STATIONS[-1][0]

# Elevons
ELEVON_HINGE_X = -10.55
ELEVON_TE_X = -12.75
ELEVON_Y_IN = 2.6
ELEVON_Y_OUT = 11.3
ELEVON_Z = 3.30

# Vertical stabilizer: (z, x_leading, thickness); rudder hinge line in x(z)
FIN_STATIONS = (
    (6.75, -10.2, 0.72),
    (9.5, -11.9, 0.55),
    (13.0, -13.9, 0.40),
    (16.4, -15.9, 0.26),
)
FIN_ROOT_Z = FIN_STATIONS[0][0]
FIN_TIP_Z = FIN_STATIONS[-1][0]


def rudder_hinge_x(z: float) -> float:
    """Rudder/speed-brake hinge line, slightly swept aft with height."""
    t = (z - 7.2) / (16.2 - 7.2)
    return -15.7 - 1.0 * t


RUDDER_ROOT_Z = 7.2
RUDDER_TIP_Z = 16.2
RUDDER_TE_ROOT_X = -17.0
RUDDER_TE_TIP_X = -17.85
RUDDER_PANEL_T = 0.17

# Body flap
FLAP_HINGE = (-16.75, 0.0, 3.45)
FLAP_CHORD = 2.35
FLAP_WIDTH = 5.7
FLAP_THICK = 0.34

# Landing gear
NOSE_GEAR_X = 13.3
NOSE_GEAR_TOP_Z = 3.30
NOSE_WHEEL_R = 0.41
NOSE_WHEEL_W = 0.22
NOSE_FORK_DROP = 1.55                       # strut frame -> fork (steering) frame
NOSE_AXLE_DROP = NOSE_GEAR_TOP_Z - NOSE_FORK_DROP - NOSE_WHEEL_R  # fork frame -> axle

MAIN_GEAR_X = -2.6
MAIN_GEAR_Y = 3.45
MAIN_GEAR_TOP_Z = 3.35
MAIN_WHEEL_R = 0.56
MAIN_WHEEL_W = 0.40
MAIN_AXLE_DROP = MAIN_GEAR_TOP_Z - MAIN_WHEEL_R   # strut frame -> axle (2.79)

# ---------------------------------------------------------------- helpers


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def _airfoil_loop(x_le, x_te, y, z_b, thick, samples=9):
    """Closed chordwise loop for a wing station at spanwise position y.

    LoftGeometry wants XY loops at constant z, so points are authored as
    (x, -z, y); rotate_x(-pi/2) afterwards maps them to world (x, y, z).
    Flat bottom at z_b, rounded top peaking mid-chord.
    """
    pts = []
    for i in range(samples):  # top surface, LE -> TE
        s = i / (samples - 1)
        x = _lerp(x_le, x_te, s)
        bump = (4.0 * s * (1.0 - s)) ** 0.7
        pts.append((x, -(z_b + max(0.02, thick * bump)), y))
    for i in range(samples):  # bottom surface, TE -> LE
        s = 1.0 - i / (samples - 1)
        x = _lerp(x_le, x_te, s)
        pts.append((x, -z_b, y))
    return pts


def _fin_loop(z, x_le, x_te, thick, samples=7):
    """Closed lens loop in the XY plane at height z (vertical fin)."""
    top = []
    bot = []
    for i in range(samples):
        s = i / (samples - 1)
        x = _lerp(x_le, x_te, s)
        w = max(0.015, 0.5 * thick * (4.0 * s * (1.0 - s)) ** 0.6)
        top.append((x, w, z))
        bot.append((x, -w, z))
    return top + bot[::-1]


def _door_arc_loop(x_local, side, r_out, r_in, phi_lo, phi_hi, n=14):
    """Bay-door cross-section (annular arc) in door-local frame.

    Door frame origin sits on the hinge line at (BAY_X_MID, side*SILL_Y,
    SILL_Z). World arc: y = side*R*sin(phi), z = zc + R*cos(phi).
    LoftGeometry wants XY loops at constant z, so points are authored as
    (-z, y, x_local); rotate_y(pi/2) afterwards maps them to (x_local, y, z).
    """
    outer = []
    inner = []
    for i in range(n):
        phi = _lerp(phi_lo, phi_hi, i / (n - 1))
        s, c = math.sin(phi), math.cos(phi)
        outer.append((-(DOOR_ARC_ZC + r_out * c - SILL_Z), side * r_out * s - side * SILL_Y, x_local))
        inner.append((-(DOOR_ARC_ZC + r_in * c - SILL_Z), side * r_in * s - side * SILL_Y, x_local))
    loop = outer + inner[::-1]
    if side < 0:
        loop = loop[::-1]  # keep outward winding on the mirrored side
    return loop


def _superellipse_wire(x, z0, z1, w, e, n=48):
    """Closed smooth superellipse cross-section wire in the YZ plane at x."""
    zc = 0.5 * (z0 + z1)
    hh = 0.5 * (z1 - z0)
    hw = 0.5 * w
    pts = []
    for i in range(n):
        t = 2.0 * math.pi * i / n
        c, s = math.cos(t), math.sin(t)
        y = hw * math.copysign(abs(c) ** (2.0 / e), c)
        z = zc + hh * math.copysign(abs(s) ** (2.0 / e), s)
        pts.append(cq.Vector(x, y, z))
    # periodic spline -> smooth hull skin (a polygon wire lofts into visible facets)
    return cq.Wire.assembleEdges([cq.Edge.makeSpline(pts, periodic=True)])


def _tube(radius, p0, p1, segments=20):
    """Cylinder mesh from p0 to p1 (both part-local points)."""
    dx, dy, dz = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    geom = CylinderGeometry(radius, length, radial_segments=segments)
    # cylinder starts along +Z centered at origin; aim it at (dx,dy,dz)
    yaw = math.atan2(dy, dx)
    pitch = math.acos(max(-1.0, min(1.0, dz / length)))
    geom.rotate_y(pitch)
    geom.rotate_z(yaw)
    geom.translate(0.5 * (p0[0] + p1[0]), 0.5 * (p0[1] + p1[1]), 0.5 * (p0[2] + p1[2]))
    return geom


# ---------------------------------------------------------------- model


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="space_shuttle_orbiter")

    white = model.material("tps_white", rgba=(0.92, 0.92, 0.94, 1.0))
    black = model.material("tps_black", rgba=(0.10, 0.10, 0.12, 1.0))
    charcoal = model.material("tps_charcoal", rgba=(0.17, 0.17, 0.19, 1.0))
    window = model.material("window_glass", rgba=(0.04, 0.04, 0.07, 1.0))
    silver = model.material("gear_silver", rgba=(0.72, 0.73, 0.75, 1.0))
    steel = model.material("nozzle_steel", rgba=(0.42, 0.42, 0.45, 1.0))
    dark_steel = model.material("nozzle_throat", rgba=(0.14, 0.14, 0.15, 1.0))
    tire_black = model.material("tire_rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    flag_red = model.material("flag_red", rgba=(0.70, 0.12, 0.15, 1.0))
    flag_blue = model.material("flag_blue", rgba=(0.10, 0.16, 0.42, 1.0))

    fuselage = model.part("fuselage")

    # --- main hull: superellipse side loft along +Y, rotated so +Y -> +X ---
    hull_sections = (
        (17.8, 4.32, 4.55, 0.36),
        (16.9, 3.95, 5.05, 1.70),
        (15.3, 3.55, 5.75, 3.15),
        (13.4, 3.30, 6.28, 4.35),
        (10.6, 3.10, 6.55, 4.90),
        (9.4, 3.05, 6.30, 5.10),
        (8.6, 3.05, 5.98, 5.20),
        (-7.4, 3.05, 5.98, 5.20),
        (-8.6, 3.10, 6.45, 5.30),
        (-10.5, 3.18, 6.90, 5.50),
        (-14.5, 3.35, 6.90, 5.20),
        (-16.9, 3.60, 6.75, 4.50),
    )
    hull_expo = (2.0, 2.0, 2.2, 2.5, 2.7, 3.0, 3.4, 3.4, 3.0, 2.8, 2.7, 2.5)
    # Solid CadQuery loft so a REAL payload-bay cavity can be cut into the
    # spine: without the cut the opened doors would only reveal sealed skin
    # (same reasoning as the CH-47 reference's hollow cargo hold).
    hull_solid = cq.Solid.makeLoft(
        [_superellipse_wire(x, z0, z1, w, e)
         for (x, z0, z1, w), e in zip(hull_sections, hull_expo)],
        ruled=True,
    )
    bay_len = BAY_CUT_X_FWD - BAY_CUT_X_AFT
    bay_cut = cq.Solid.makeBox(bay_len, 2.0 * BAY_CUT_HALF_W, 3.5).translate(
        (BAY_CUT_X_AFT, -BAY_CUT_HALF_W, BAY_FLOOR_Z)
    )
    hull_solid = hull_solid.cut(bay_cut)
    fuselage.visual(
        mesh_from_cadquery(cq.Workplane("XY").add(hull_solid), "hull"),
        material=white,
        name="hull",
    )

    # --- black TPS lower skirt (proud of the hull, rises toward the nose) ---
    belly_sections = (
        (17.82, 4.28, 4.60, 0.44),
        (16.9, 3.90, 4.95, 1.80),
        (15.3, 3.50, 4.70, 3.26),
        (13.4, 3.25, 4.18, 4.46),
        (10.6, 3.05, 3.88, 5.00),
        (8.6, 3.00, 3.74, 5.30),
        (-7.4, 3.00, 3.74, 5.30),
        (-10.5, 3.13, 3.92, 5.60),
        (-14.5, 3.30, 4.06, 5.30),
        (-16.9, 3.55, 4.42, 4.60),
    )
    belly = superellipse_side_loft(
        [(x, z0, z1, w) for (x, z0, z1, w) in belly_sections],
        exponents=(2.0, 2.0, 2.2, 2.4, 2.6, 2.8, 2.8, 2.6, 2.5, 2.4),
        segments=48,
    ).rotate_z(-math.pi / 2.0)
    fuselage.visual(mesh_from_geometry(belly, "belly_tiles"), material=black, name="belly_tiles")

    # --- black nose cap over the first metres of hull ---
    nose_sections = (
        (17.86, 4.26, 4.62, 0.50),
        (17.2, 3.90, 5.12, 1.80),
        (16.3, 3.75, 5.15, 2.40),
    )
    nose = superellipse_side_loft(
        [(x, z0, z1, w) for (x, z0, z1, w) in nose_sections],
        exponents=2.0,
        segments=40,
    ).rotate_z(-math.pi / 2.0)
    fuselage.visual(mesh_from_geometry(nose, "nose_cap"), material=black, name="nose_cap")

    # --- black chine band along the forward fuselage sides (nose -> strake) ---
    chine_sections = (
        (16.9, 4.05, 4.50, 1.86),
        (15.3, 3.95, 4.45, 3.24),
        (13.4, 3.85, 4.35, 4.44),
        (11.3, 3.80, 4.28, 4.86),
    )
    chine = superellipse_side_loft(
        [(x, z0, z1, w) for (x, z0, z1, w) in chine_sections],
        exponents=2.2,
        segments=40,
    ).rotate_z(-math.pi / 2.0)
    fuselage.visual(mesh_from_geometry(chine, "chine_band"), material=black, name="chine_band")

    # --- forward RCS panel (dark rectangle behind the nose cap, on top) ---
    fuselage.visual(
        Box((1.5, 1.9, 0.06)),
        origin=Origin(xyz=(15.7, 0.0, 5.42), rpy=(0.0, -0.30, 0.0)),
        material=charcoal,
        name="forward_rcs_panel",
    )
    # nose RCS thruster ports, three per side on the black chine
    for name, side in SIDES:
        for i in range(3):
            fuselage.visual(
                Box((0.26, 0.05, 0.20)),
                origin=Origin(xyz=(16.5 - 0.45 * i, side * (1.05 + 0.10 * i), 4.55)),
                material=charcoal,
                name=f"{name}_rcs_port_{i}",
            )

    # --- cockpit windshield panes on the forward cabin slope ---
    for i in range(6):
        t = (i - 2.5) / 2.5
        yaw = 0.55 * t
        fuselage.visual(
            Box((0.60, 0.06, 0.50)),
            origin=Origin(
                xyz=(14.95 - 0.55 * abs(t), 1.10 * t, 5.85),
                rpy=(0.0, -0.70, yaw),
            ),
            material=window,
            name=f"window_pane_{i}",
        )

    # --- circular crew hatch on the left forward fuselage ---
    fuselage.visual(
        Cylinder(radius=0.46, length=0.06),
        origin=Origin(xyz=(12.4, 2.24, 4.85), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="crew_hatch_ring",
    )

    # --- payload bay interior: dark floor liner + payload retention rails ---
    bay_len = BAY_CUT_X_FWD - BAY_CUT_X_AFT
    fuselage.visual(
        Box((bay_len - 0.1, 2.0 * BAY_CUT_HALF_W - 0.1, 0.08)),
        origin=Origin(xyz=(BAY_X_MID, 0.0, BAY_FLOOR_Z + 0.02)),
        material=charcoal,
        name="payload_bay_floor",
    )
    for name, side in SIDES:
        fuselage.visual(
            Box((bay_len - 0.3, 0.14, 0.12)),
            origin=Origin(xyz=(BAY_X_MID, side * 1.55, BAY_FLOOR_Z + 0.12)),
            material=silver,
            name=f"{name}_payload_rail",
        )

    # --- payload bay sill rails: doors hinge onto these ---
    for name, side in SIDES:
        fuselage.visual(
            Box((BAY_LEN, 0.34, 0.36)),
            origin=Origin(xyz=(BAY_X_MID, side * SILL_Y, SILL_Z - 0.14)),
            material=white,
            name=f"{name}_bay_sill",
        )

    # --- livery: US flag + round insignia on the left side, name strip right ---
    fuselage.visual(Box((0.80, 0.03, 0.34)), origin=Origin(xyz=(2.2, 2.56, 4.95)),
                    material=white, name="flag_field")
    for i in range(3):
        fuselage.visual(
            Box((0.80, 0.035, 0.045)),
            origin=Origin(xyz=(2.2, 2.56, 4.82 + 0.12 * i)),
            material=flag_red,
            name=f"flag_stripe_{i}",
        )
    fuselage.visual(Box((0.36, 0.04, 0.18)), origin=Origin(xyz=(2.42, 2.565, 5.02)),
                    material=flag_blue, name="flag_canton")
    fuselage.visual(
        Cylinder(radius=0.30, length=0.035),
        origin=Origin(xyz=(-3.4, 2.55, 4.95), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=flag_blue,
        name="insignia_roundel",
    )
    fuselage.visual(Box((3.4, 0.03, 0.30)), origin=Origin(xyz=(0.4, -2.56, 4.95)),
                    material=charcoal, name="name_strip")

    # --- wings: double-delta lofts + black leading-edge panels + black underside ---
    for name, side in SIDES:
        loops = []
        le_loops = []
        under_loops = []
        for (ay, x_le, x_te, z_b, th) in WING_STATIONS:
            y = side * ay
            loops.append(_airfoil_loop(x_le, x_te, y, z_b, th))
            le_loops.append(_airfoil_loop(x_le - 0.08, x_le + 1.2, y, z_b - 0.05, th * 0.85 + 0.06))
            under_loops.append(_airfoil_loop(x_le + 0.3, x_te, y, z_b - 0.035, 0.05))
        if side < 0:
            loops = [lp[::-1] for lp in loops]
            le_loops = [lp[::-1] for lp in le_loops]
            under_loops = [lp[::-1] for lp in under_loops]
        rot = -math.pi / 2.0
        fuselage.visual(mesh_from_geometry(LoftGeometry(loops).rotate_x(rot), f"{name}_wing"),
                        material=white, name=f"{name}_wing")
        fuselage.visual(mesh_from_geometry(LoftGeometry(le_loops).rotate_x(rot), f"{name}_wing_leading_edge"),
                        material=black, name=f"{name}_wing_leading_edge")
        fuselage.visual(mesh_from_geometry(LoftGeometry(under_loops).rotate_x(rot), f"{name}_wing_underside"),
                        material=black, name=f"{name}_wing_underside")

    # --- vertical stabilizer with dark leading-edge strip ---
    fin_loops = []
    fin_le_loops = []
    for (z, x_le, th) in FIN_STATIONS:
        x_te = rudder_hinge_x(z) + 0.12
        fin_loops.append(_fin_loop(z, x_le, x_te, th))
        fin_le_loops.append(_fin_loop(z, x_le - 0.04, x_le + 0.75, th * 0.82))
    fuselage.visual(mesh_from_geometry(LoftGeometry(fin_loops), "vertical_stabilizer"),
                    material=white, name="vertical_stabilizer")
    fuselage.visual(mesh_from_geometry(LoftGeometry(fin_le_loops), "fin_leading_edge"),
                    material=charcoal, name="fin_leading_edge")

    # --- OMS pods flanking the fin on the aft deck ---
    pod_sections = (
        (-11.8, 6.25, 6.75, 0.85),
        (-12.8, 6.15, 7.90, 2.00),
        (-14.5, 6.15, 8.15, 2.30),
        (-16.3, 6.20, 8.00, 2.10),
        (-17.3, 6.35, 7.50, 1.40),
    )
    for name, side in SIDES:
        pod = superellipse_side_loft(
            [(x, z0, z1, w) for (x, z0, z1, w) in pod_sections],
            exponents=2.2,
            segments=36,
        ).rotate_z(-math.pi / 2.0).translate(0.0, side * 1.75, 0.0)
        fuselage.visual(mesh_from_geometry(pod, f"{name}_oms_pod"),
                        material=white, name=f"{name}_oms_pod")
        oms_bell = ConeGeometry(0.42, 1.05, radial_segments=28)
        # cone apex is the +Z end; map +Z to forward/down so the bell opens aft/up
        oms_bell.rotate_y(math.radians(105.0))
        oms_bell.translate(-17.35, side * 1.75, 7.35)
        fuselage.visual(mesh_from_geometry(oms_bell, f"{name}_oms_nozzle"),
                        material=steel, name=f"{name}_oms_nozzle")

    # --- aft base heat shield + three main engine bells ---
    fuselage.visual(Box((0.5, 4.4, 3.1)), origin=Origin(xyz=(-17.0, 0.0, 5.0)),
                    material=steel, name="base_heat_shield")
    engine_mounts = ((0.0, 6.05, 12.0), (1.30, 4.45, 8.0), (-1.30, 4.45, 8.0))
    bell_profile = ((0.44, 0.0), (0.56, 0.35), (0.78, 1.0), (1.02, 1.85), (1.15, 2.55))
    for i, (ey, ez, pitch_deg) in enumerate(engine_mounts):
        bell = LatheGeometry(bell_profile, segments=36)
        # profile grows along +Z; map +Z to aft (-X) tipped up by pitch_deg
        bell.rotate_y(math.radians(-(90.0 - pitch_deg)))
        bell.translate(-16.95, ey, ez)
        fuselage.visual(mesh_from_geometry(bell, f"main_engine_nozzle_{i}"),
                        material=steel, name=f"main_engine_nozzle_{i}")
        # dark exit disc, oversized to meet the bell wall so it reads as the cavity
        throat = CylinderGeometry(1.12, 0.25, radial_segments=28)
        throat.rotate_y(math.radians(-(90.0 - pitch_deg)))
        dx = -2.30 * math.cos(math.radians(pitch_deg))
        dz = 2.30 * math.sin(math.radians(pitch_deg))
        throat.translate(-16.95 + dx, ey, ez + dz)
        fuselage.visual(mesh_from_geometry(throat, f"main_engine_throat_{i}"),
                        material=dark_steel, name=f"main_engine_throat_{i}")

    # ---------------------------------------------------------- bay doors
    # Closed at q=0 (as photographed); positive q swings each door up/outboard.
    for name, side in SIDES:
        door = model.part(f"{name}_payload_bay_door")
        loop_f = _door_arc_loop(BAY_LEN / 2.0, side, DOOR_ARC_R + DOOR_THICK / 2.0,
                                DOOR_ARC_R - DOOR_THICK / 2.0,
                                DOOR_PHI_HINGE + DOOR_PHI_TUCK, DOOR_PHI_SEAM)
        loop_a = _door_arc_loop(-BAY_LEN / 2.0, side, DOOR_ARC_R + DOOR_THICK / 2.0,
                                DOOR_ARC_R - DOOR_THICK / 2.0,
                                DOOR_PHI_HINGE + DOOR_PHI_TUCK, DOOR_PHI_SEAM)
        door.visual(mesh_from_geometry(LoftGeometry([loop_f, loop_a]).rotate_y(math.pi / 2.0),
                                       f"{name}_bay_door"),
                    material=white, name=f"{name}_door_shell")
        model.articulation(
            f"fuselage_to_{name}_payload_bay_door",
            ArticulationType.REVOLUTE,
            parent=fuselage,
            child=door,
            origin=Origin(xyz=(BAY_X_MID, side * SILL_Y, SILL_Z)),
            axis=(-side, 0.0, 0.0),
            motion_limits=MotionLimits(effort=400.0, velocity=0.3, lower=0.0, upper=2.6),
        )

    # ------------------------------------------------- split rudder panels
    # Both panels share the swept hinge line on the fin trailing edge.
    # Positive q deflects each panel toward its own side, so equal positive
    # values open the speed brake and opposite-sign values act as a rudder.
    hinge_root = (rudder_hinge_x(RUDDER_ROOT_Z), 0.0, RUDDER_ROOT_Z)
    hinge_tip = (rudder_hinge_x(RUDDER_TIP_Z), 0.0, RUDDER_TIP_Z)
    h_len = math.hypot(hinge_tip[0] - hinge_root[0], hinge_tip[2] - hinge_root[2])
    hinge_dir = ((hinge_tip[0] - hinge_root[0]) / h_len, 0.0,
                 (hinge_tip[2] - hinge_root[2]) / h_len)
    for name, side in SIDES:
        panel = model.part(f"{name}_rudder_panel")
        loops = []
        for (z, te_x) in ((RUDDER_ROOT_Z, RUDDER_TE_ROOT_X), (RUDDER_TIP_Z, RUDDER_TE_TIP_X)):
            hx = rudder_hinge_x(z)
            zl = z - RUDDER_ROOT_Z
            x0 = hx - hinge_root[0] + 0.35   # tucked into the fin ahead of the hinge
            x1 = te_x - hinge_root[0]
            loop = [
                (x0, 0.02 * side, zl),
                (x0, side * RUDDER_PANEL_T, zl),
                (x1, side * 0.35 * RUDDER_PANEL_T, zl),
                (x1, 0.01 * side, zl),
            ]
            if side < 0:
                loop = loop[::-1]
            loops.append(loop)
        panel.visual(mesh_from_geometry(LoftGeometry(loops), f"{name}_rudder_panel"),
                     material=white, name=f"{name}_rudder_slab")
        model.articulation(
            f"fin_to_{name}_rudder_panel",
            ArticulationType.REVOLUTE,
            parent=fuselage,
            child=panel,
            origin=Origin(xyz=hinge_root),
            axis=(-side * hinge_dir[0], 0.0, -side * hinge_dir[2]),
            motion_limits=MotionLimits(effort=200.0, velocity=0.8, lower=-0.45, upper=0.85),
        )

    # ------------------------------------------------------------ elevons
    # Hinged on the wing trailing edge; positive q raises the trailing edge.
    for name, side in SIDES:
        elevon = model.part(f"{name}_elevon")
        loops = []
        for ay, root_t in ((ELEVON_Y_IN, 0.30), (ELEVON_Y_OUT, 0.16)):
            y = side * ay - side * (ELEVON_Y_IN + ELEVON_Y_OUT) / 2.0
            x0 = 0.14                       # tucked into the wing ahead of the hinge
            x1 = ELEVON_TE_X - ELEVON_HINGE_X
            # authored as (x, -z, y); rotate_x(-pi/2) maps to (x, y, z)
            loop = [
                (x0, -root_t / 2.0, y),
                (x1, -0.03, y),
                (x1, 0.03, y),
                (x0, root_t / 2.0, y),
            ]
            loops.append(loop)
        if side < 0:
            loops = [lp[::-1] for lp in loops]
        elevon.visual(mesh_from_geometry(LoftGeometry(loops).rotate_x(-math.pi / 2.0),
                                         f"{name}_elevon"),
                      material=white, name=f"{name}_elevon_panel")
        model.articulation(
            f"wing_to_{name}_elevon",
            ArticulationType.REVOLUTE,
            parent=fuselage,
            child=elevon,
            origin=Origin(xyz=(ELEVON_HINGE_X, side * (ELEVON_Y_IN + ELEVON_Y_OUT) / 2.0, ELEVON_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=300.0, velocity=0.7, lower=-0.6, upper=0.4),
        )

    # ---------------------------------------------------------- body flap
    body_flap = model.part("body_flap")
    body_flap.visual(
        Box((FLAP_CHORD + 0.2, FLAP_WIDTH, FLAP_THICK)),
        origin=Origin(xyz=(-(FLAP_CHORD + 0.2) / 2.0 + 0.2, 0.0, 0.0)),
        material=black,
        name="body_flap_panel",
    )
    model.articulation(
        "fuselage_to_body_flap",
        ArticulationType.REVOLUTE,
        parent=fuselage,
        child=body_flap,
        origin=Origin(xyz=FLAP_HINGE),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=300.0, velocity=0.4, lower=-0.35, upper=0.45),
    )

    # ------------------------------------------------------ nose gear chain
    # Strut retracts forward into the belly well; q=0 is gear-down.
    nose_strut = model.part("nose_gear_strut")
    nose_strut.visual(mesh_from_geometry(_tube(0.14, (0.0, 0.0, 0.40), (0.0, 0.0, -1.10)),
                                         "nose_strut_tube"),
                      material=silver, name="nose_strut_tube")
    nose_strut.visual(mesh_from_geometry(_tube(0.10, (0.0, 0.0, -1.05), (0.0, 0.0, -NOSE_FORK_DROP - 0.15)),
                                         "nose_strut_piston"),
                      material=silver, name="nose_strut_piston")
    nose_strut.visual(mesh_from_geometry(_tube(0.07, (0.0, 0.0, -0.85), (0.95, 0.0, 0.30)),
                                         "nose_drag_brace"),
                      material=silver, name="nose_drag_brace")
    model.articulation(
        "fuselage_to_nose_gear_strut",
        ArticulationType.REVOLUTE,
        parent=fuselage,
        child=nose_strut,
        origin=Origin(xyz=(NOSE_GEAR_X, 0.0, NOSE_GEAR_TOP_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=800.0, velocity=0.4, lower=0.0, upper=1.6),
    )

    # Steerable fork spins about the strut axis.
    nose_fork = model.part("nose_gear_fork")
    nose_fork.visual(mesh_from_geometry(_tube(0.115, (0.0, 0.0, 0.30), (0.0, 0.0, -NOSE_AXLE_DROP)),
                                        "nose_fork_tube"),
                     material=silver, name="nose_fork_tube")
    nose_fork.visual(mesh_from_geometry(
        _tube(0.075, (0.0, -0.42, -NOSE_AXLE_DROP), (0.0, 0.42, -NOSE_AXLE_DROP)),
        "nose_axle"),
        material=steel, name="nose_axle")
    model.articulation(
        "nose_gear_strut_to_fork",
        ArticulationType.REVOLUTE,
        parent=nose_strut,
        child=nose_fork,
        origin=Origin(xyz=(0.0, 0.0, -NOSE_FORK_DROP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=150.0, velocity=1.0, lower=-0.35, upper=0.35),
    )

    def _wheel_visuals(part, prefix, tire_r, tire_w, rim_r):
        wheel = WheelGeometry(
            rim_r,
            tire_w * 0.72,
            rim=WheelRim(inner_radius=rim_r * 0.62, flange_height=0.02,
                         flange_thickness=0.012, bead_seat_depth=0.008),
            hub=WheelHub(radius=rim_r * 0.38, width=tire_w * 0.5, cap_style="domed",
                         bolt_pattern=BoltPattern(count=6, circle_diameter=rim_r * 0.55,
                                                  hole_diameter=0.022)),
            face=WheelFace(dish_depth=0.005, front_inset=0.003, rear_inset=0.003),
        )
        part.visual(mesh_from_geometry(wheel, f"{prefix}_rim"),
                    material=silver, name=f"{prefix}_rim")
        tire = TireGeometry(
            tire_r,
            tire_w,
            inner_radius=rim_r,
            carcass=TireCarcass(belt_width_ratio=0.62, sidewall_bulge=0.07),
            tread=TireTread(style="ribbed", depth=0.012, count=4, land_ratio=0.6),
            sidewall=TireSidewall(style="rounded", bulge=0.05),
        )
        part.visual(mesh_from_geometry(tire, f"{prefix}_tire"),
                    material=tire_black, name=f"{prefix}_tire")

    for name, side in SIDES:
        wheel_part = model.part(f"{name}_nose_wheel")
        _wheel_visuals(wheel_part, f"{name}_nose_wheel", NOSE_WHEEL_R, NOSE_WHEEL_W, 0.24)
        model.articulation(
            f"nose_axle_to_{name}_wheel",
            ArticulationType.CONTINUOUS,
            parent=nose_fork,
            child=wheel_part,
            # yaw the joint frame so the wheel's local +X spin axis lies along +Y
            origin=Origin(xyz=(0.0, side * 0.27, -NOSE_AXLE_DROP), rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=50.0, velocity=60.0),
        )

    # ------------------------------------------------------ main gear pair
    for name, side in SIDES:
        strut = model.part(f"{name}_main_gear_strut")
        strut.visual(mesh_from_geometry(_tube(0.17, (0.0, 0.0, 0.40), (0.0, 0.0, -1.85)),
                                        f"{name}_main_strut_tube"),
                     material=silver, name=f"{name}_main_strut_tube")
        strut.visual(mesh_from_geometry(
            _tube(0.12, (0.0, 0.0, -1.80), (0.0, 0.0, -MAIN_AXLE_DROP - 0.05)),
            f"{name}_main_strut_piston"),
            material=silver, name=f"{name}_main_strut_piston")
        strut.visual(mesh_from_geometry(_tube(0.08, (0.0, 0.0, -1.15), (0.95, 0.0, 0.30)),
                                        f"{name}_main_drag_brace"),
                     material=silver, name=f"{name}_main_drag_brace")
        strut.visual(mesh_from_geometry(
            _tube(0.08, (0.0, 0.0, -1.15), (0.0, -side * 0.95, 0.30)),
            f"{name}_main_side_brace"),
            material=silver, name=f"{name}_main_side_brace")
        strut.visual(mesh_from_geometry(
            _tube(0.095, (0.0, -0.50, -MAIN_AXLE_DROP), (0.0, 0.50, -MAIN_AXLE_DROP)),
            f"{name}_main_axle"),
            material=steel, name=f"{name}_main_axle")
        model.articulation(
            f"fuselage_to_{name}_main_gear_strut",
            ArticulationType.REVOLUTE,
            parent=fuselage,
            child=strut,
            origin=Origin(xyz=(MAIN_GEAR_X, side * MAIN_GEAR_Y, MAIN_GEAR_TOP_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=1500.0, velocity=0.4, lower=0.0, upper=1.55),
        )
        for wname, woff in (("outboard", side * 0.33), ("inboard", -side * 0.33)):
            wheel_part = model.part(f"{name}_{wname}_main_wheel")
            _wheel_visuals(wheel_part, f"{name}_{wname}_main", MAIN_WHEEL_R, MAIN_WHEEL_W, 0.33)
            model.articulation(
                f"{name}_main_axle_to_{wname}_wheel",
                ArticulationType.CONTINUOUS,
                parent=strut,
                child=wheel_part,
                origin=Origin(xyz=(0.0, woff, -MAIN_AXLE_DROP),
                              rpy=(0.0, 0.0, math.pi / 2.0)),
                axis=(1.0, 0.0, 0.0),
                motion_limits=MotionLimits(effort=200.0, velocity=60.0),
            )

    # ------------------------------------------------------- gear doors
    # All doors rest OPEN at q=0 (as photographed); positive q closes them.
    for name, side in SIDES:
        door = model.part(f"{name}_main_gear_door")
        door.visual(Box((2.7, 0.09, 1.85)),
                    origin=Origin(xyz=(0.0, 0.0, -0.83)),
                    material=black, name=f"{name}_main_door_panel")
        model.articulation(
            f"fuselage_to_{name}_main_gear_door",
            ArticulationType.REVOLUTE,
            parent=fuselage,
            child=door,
            origin=Origin(xyz=(MAIN_GEAR_X, side * 2.45, 3.12)),
            axis=(side, 0.0, 0.0),
            motion_limits=MotionLimits(effort=120.0, velocity=0.6, lower=0.0, upper=1.5),
        )
    for name, side in SIDES:
        door = model.part(f"{name}_nose_gear_door")
        door.visual(Box((1.9, 0.07, 1.15)),
                    origin=Origin(xyz=(0.0, 0.0, -0.47)),
                    material=black, name=f"{name}_nose_door_panel")
        model.articulation(
            f"fuselage_to_{name}_nose_gear_door",
            ArticulationType.REVOLUTE,
            parent=fuselage,
            child=door,
            origin=Origin(xyz=(NOSE_GEAR_X, side * 0.62, 3.42)),
            axis=(-side, 0.0, 0.0),
            motion_limits=MotionLimits(effort=80.0, velocity=0.8, lower=0.0, upper=1.5),
        )

    return model


# ---------------------------------------------------------------- tests


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fus = object_model.get_part("fuselage")

    # ---- intentional, local overlaps (declared in one batch) ----
    for name in ("left", "right"):
        for elem in (f"{name}_bay_sill", "hull"):
            ctx.allow_overlap(
                "fuselage", f"{name}_payload_bay_door", elem_a=elem,
                elem_b=f"{name}_door_shell",
                reason="Closed bay door tucks its hinge lip into the sill rail so the hinge line reads captured.",
            )
        ctx.allow_overlap(
            "fuselage", f"{name}_rudder_panel", elem_a="vertical_stabilizer",
            elem_b=f"{name}_rudder_slab",
            reason="Rudder/speed-brake panel leading edge is seated inside the fin trailing-edge cove.",
        )
        for elem in (f"{name}_wing", f"{name}_wing_underside", "belly_tiles"):
            ctx.allow_overlap(
                "fuselage", f"{name}_elevon", elem_a=elem,
                elem_b=f"{name}_elevon_panel",
                reason="Elevon leading edge is seated inside the wing trailing-edge cove on its hinge line.",
            )
        # Gear struts carry tube+piston+braces that all anchor up inside the
        # wells; the wheel hubs capture the through-axles. Their auto-split
        # mesh component names are not stable, so these mechanical pairs are
        # allowed at part level and proven by the stance/retraction checks.
        ctx.allow_overlap(
            "fuselage", f"{name}_main_gear_strut",
            reason="Main gear strut assembly anchors up inside the wing-root gear well.",
        )
        for elem in (f"{name}_wing", f"{name}_wing_underside", "belly_tiles", "hull"):
            ctx.allow_overlap(
                "fuselage", f"{name}_main_gear_door", elem_a=elem,
                elem_b=f"{name}_main_door_panel",
                reason="Open gear door keeps its hinge edge embedded in the well sill.",
            )
        for elem in ("hull", "belly_tiles"):
            ctx.allow_overlap(
                "fuselage", f"{name}_nose_gear_door", elem_a=elem,
                elem_b=f"{name}_nose_door_panel",
                reason="Open nose gear door keeps its hinge edge embedded in the well sill.",
            )
        ctx.allow_overlap(
            "nose_gear_fork", f"{name}_nose_wheel",
            reason="Wheel hub captures the through-axle (captured-pin fit).",
        )
        for wname in ("outboard", "inboard"):
            ctx.allow_overlap(
                f"{name}_main_gear_strut", f"{name}_{wname}_main_wheel",
                reason="Wheel hub captures the through-axle (captured-pin fit).",
            )
    for elem in ("hull", "belly_tiles", "base_heat_shield"):
        ctx.allow_overlap(
            "fuselage", "body_flap", elem_a=elem, elem_b="body_flap_panel",
            reason="Body flap forward edge is seated inside the aft fuselage hinge cove.",
        )
    ctx.allow_overlap(
        "fuselage", "nose_gear_strut",
        reason="Nose gear strut and drag brace anchor up inside the nose wheel well.",
    )
    ctx.allow_overlap(
        "nose_gear_strut", "nose_gear_fork",
        reason="Steering fork collar captures the oleo piston (captured-pin fit).",
    )
    ctx.allow_coplanar_surfaces(
        "left_rudder_panel", "right_rudder_panel",
        reason="The two speed-brake halves lie face to face when closed.",
    )

    # ---- identity and proportions ----
    aabb = ctx.part_world_aabb(fus)
    ctx.check("orbiter is real scale", aabb is not None and (aabb[1][0] - aabb[0][0]) > 30.0,
              details=f"fuselage aabb={aabb}")
    ctx.check("double-delta wing span", aabb is not None and (aabb[1][1] - aabb[0][1]) > 21.0,
              details=f"fuselage aabb={aabb}")
    ctx.check("fin reaches full height", aabb is not None and aabb[1][2] > 15.5,
              details=f"fuselage aabb={aabb}")
    belly = ctx.part_element_world_aabb(fus, elem="belly_tiles")
    ctx.check("black TPS wraps the underside", belly is not None and belly[0][2] < 3.1,
              details=f"belly aabb={belly}")
    for i in range(3):
        bell = ctx.part_element_world_aabb(fus, elem=f"main_engine_nozzle_{i}")
        ctx.check(f"main engine bell {i} protrudes aft", bell is not None and bell[0][0] < -18.3,
                  details=f"bell aabb={bell}")
    hull_box = ctx.part_element_world_aabb(fus, elem="hull")
    ctx.check("belly clearance over runway", hull_box is not None and hull_box[0][2] > 2.7,
              details=f"hull aabb={hull_box}")

    # ---- payload bay doors ----
    ld = object_model.get_part("left_payload_bay_door")
    rd = object_model.get_part("right_payload_bay_door")
    ctx.expect_gap(ld, rd, axis="y", min_gap=0.0, max_gap=0.10,
                   name="closed doors meet at the spine with a small seam")
    crown = ctx.part_element_world_aabb(ld, elem="left_door_shell")
    ctx.check("closed doors crown the mid fuselage",
              crown is not None and 6.4 < crown[1][2] < 7.1, details=f"door aabb={crown}")
    floor = ctx.part_element_world_aabb(fus, elem="payload_bay_floor")
    ctx.check("payload bay is a real recessed cavity",
              floor is not None and floor[1][2] < SILL_Z - 1.3,
              details=f"bay floor aabb={floor}")
    ld_joint = object_model.get_articulation("fuselage_to_left_payload_bay_door")
    rest = ctx.part_world_aabb(ld)
    with ctx.pose({ld_joint: 1.2}):
        opened = ctx.part_world_aabb(ld)
        ctx.check(
            "left bay door swings up and outboard",
            rest is not None and opened is not None
            and opened[1][2] > rest[1][2] + 0.8 and opened[1][1] > rest[1][1] + 0.4,
            details=f"rest={rest}, open={opened}",
        )

    # ---- split rudder / speed brake ----
    lr = object_model.get_part("left_rudder_panel")
    rr = object_model.get_part("right_rudder_panel")
    lr_joint = object_model.get_articulation("fin_to_left_rudder_panel")
    rr_joint = object_model.get_articulation("fin_to_right_rudder_panel")
    with ctx.pose({lr_joint: 0.6, rr_joint: 0.6}):
        la = ctx.part_world_aabb(lr)
        ra = ctx.part_world_aabb(rr)
        ctx.check(
            "speed brake halves split apart",
            la is not None and ra is not None and la[1][1] > 0.55 and ra[0][1] < -0.55,
            details=f"left={la}, right={ra}",
        )

    # ---- elevons ----
    le = object_model.get_part("left_elevon")
    le_joint = object_model.get_articulation("wing_to_left_elevon")
    rest = ctx.part_world_aabb(le)
    with ctx.pose({le_joint: 0.4}):
        up = ctx.part_world_aabb(le)
        ctx.check("left elevon trailing edge deflects up",
                  rest is not None and up is not None and up[1][2] > rest[1][2] + 0.4,
                  details=f"rest={rest}, up={up}")
    with ctx.pose({le_joint: -0.6}):
        down = ctx.part_world_aabb(le)
        ctx.check("left elevon trailing edge deflects down",
                  rest is not None and down is not None and down[0][2] < rest[0][2] - 0.6,
                  details=f"rest={rest}, down={down}")

    # ---- body flap ----
    flap = object_model.get_part("body_flap")
    flap_joint = object_model.get_articulation("fuselage_to_body_flap")
    rest = ctx.part_world_aabb(flap)
    with ctx.pose({flap_joint: 0.45}):
        up = ctx.part_world_aabb(flap)
        ctx.check("body flap trails up", rest is not None and up is not None
                  and up[1][2] > rest[1][2] + 0.5, details=f"rest={rest}, up={up}")

    # ---- landing gear: stance, retraction, steering ----
    wheel_parts = ["left_nose_wheel", "right_nose_wheel"]
    for name in ("left", "right"):
        for wname in ("outboard", "inboard"):
            wheel_parts.append(f"{name}_{wname}_main_wheel")
    for wp in wheel_parts:
        wa = ctx.part_world_aabb(object_model.get_part(wp))
        ctx.check(f"{wp} sits on the runway plane",
                  wa is not None and -0.04 < wa[0][2] < 0.08, details=f"{wp} aabb={wa}")

    nose_joint = object_model.get_articulation("fuselage_to_nose_gear_strut")
    lnw = object_model.get_part("left_nose_wheel")
    rest = ctx.part_world_aabb(lnw)
    with ctx.pose({nose_joint: 1.5}):
        upw = ctx.part_world_aabb(lnw)
        ctx.check("nose gear retracts up toward the belly",
                  rest is not None and upw is not None and upw[0][2] > rest[0][2] + 1.0,
                  details=f"rest={rest}, retracted={upw}")

    main_joint = object_model.get_articulation("fuselage_to_left_main_gear_strut")
    lmw = object_model.get_part("left_outboard_main_wheel")
    rest = ctx.part_world_aabb(lmw)
    with ctx.pose({main_joint: 1.5}):
        upw = ctx.part_world_aabb(lmw)
        ctx.check("left main gear retracts up into the wing well",
                  rest is not None and upw is not None and upw[0][2] > rest[0][2] + 1.0,
                  details=f"rest={rest}, retracted={upw}")

    steer = object_model.get_articulation("nose_gear_strut_to_fork")
    rnw = object_model.get_part("right_nose_wheel")
    l0 = ctx.part_world_position(lnw)
    r0 = ctx.part_world_position(rnw)
    with ctx.pose({steer: 0.3}):
        l1 = ctx.part_world_position(lnw)
        r1 = ctx.part_world_position(rnw)
        ctx.check(
            "nose wheels steer about the strut axis",
            None not in (l0, r0, l1, r1)
            and (l1[0] - l0[0]) < -0.03 and (r1[0] - r0[0]) > 0.03,
            details=f"left {l0}->{l1}, right {r0}->{r1}",
        )

    # ---- gear doors: open at rest, closable ----
    lmd = object_model.get_part("left_main_gear_door")
    rest = ctx.part_world_aabb(lmd)
    ctx.check("main gear door hangs open", rest is not None and rest[0][2] < 1.7,
              details=f"door aabb={rest}")
    lmd_joint = object_model.get_articulation("fuselage_to_left_main_gear_door")
    with ctx.pose({lmd_joint: 1.5}):
        closed = ctx.part_world_aabb(lmd)
        ctx.check("main gear door swings up closed",
                  rest is not None and closed is not None and closed[0][2] > rest[0][2] + 1.2,
                  details=f"open={rest}, closed={closed}")
    lnd = object_model.get_part("left_nose_gear_door")
    lnd_joint = object_model.get_articulation("fuselage_to_left_nose_gear_door")
    rest = ctx.part_world_aabb(lnd)
    with ctx.pose({lnd_joint: 1.4}):
        closed = ctx.part_world_aabb(lnd)
        ctx.check("nose gear door swings closed",
                  rest is not None and closed is not None and closed[0][2] > rest[0][2] + 0.7,
                  details=f"open={rest}, closed={closed}")

    return ctx.report()


object_model = build_object_model()
