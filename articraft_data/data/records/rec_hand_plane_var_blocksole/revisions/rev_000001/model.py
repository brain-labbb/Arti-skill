from __future__ import annotations

# Realistic articulated low-angle block plane (Handtools/Hand plane family fork).
#
# Variant change vs parent bench plane:
#   - SHORT low-profile body (stubby flat sole, ~155 mm long, ~52 mm wide)
#   - NO tall central casting — just low sidewalls forming an open iron trough
#   - LOW-angle bed (20 deg) instead of the parent's 45 deg frog
#   - Same clamp, grip, and depth-adjust functional layers preserved
#
# Real object identity:
#   - A cast-iron low-angle block plane with black japanned body, bright
#     machined sole edge, rosewood front knob and rear palm grip, an angled
#     steel cutting iron (blade + chipbreaker) bedded at 20 deg on the
#     integral bed, a polished steel lever cap clamping the iron, the lever
#     cap's flip cam lever lying flat on the cap spine, a small lateral-
#     adjustment lever riding on the iron's upper end, and a knurled brass
#     depth-adjustment wheel on a steel stud behind the bed.
#
# Articulated mechanism:
#   - PRIMARY: lever-cap cam lever flips up/down (REVOLUTE, axis Y)
#   - SECONDARY: lateral-adjustment lever rocks about the bed normal (REVOLUTE)
#   - TERTIARY: brass depth wheel spins on its stud (CONTINUOUS)
#
# Coordinate frame:
#   - Plane runs along +X (toe/front at +X, heel/rear at -X).
#   - Flat sole bottom at z = 0; up is +Z; width is along Y.
#   - Iron bed is a 20 deg plane. Helper coordinates: u runs up the bed
#     from the mouth (toward heel/+Z), m is the outward bed normal.

import math

import cadquery as cq
from cadquery.selectors import BoxSelector

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Key dimensions (a low-angle block plane is ~0.155 m long, ~0.052 m wide).
# ---------------------------------------------------------------------------
BODY_LEN = 0.155
BODY_WIDTH = 0.052
SOLE_THICK = 0.008       # machined sole plate thickness
WALL_HEIGHT = 0.020      # low sidewalls (no tall central casting)
WALL_THICK = 0.005       # sidewall thickness
SOLE_TOP_Z = SOLE_THICK  # top of the flat sole plate
WALL_TOP_Z = SOLE_TOP_Z + WALL_HEIGHT  # top of the sidewalls

MOUTH_X = 0.015          # blade mouth slot center from body center along +X
BED_DEG = 20.0           # low-angle frog bed for the iron
BED_RAD = math.radians(BED_DEG)
CQ_BED = math.cos(BED_RAD)  # ~0.940
SQ_BED = math.sin(BED_RAD)  # ~0.342

# Where the front knob and rear grip seat on the sole.
KNOB_X = BODY_LEN * 0.5 - 0.025   # ~0.053
TOTE_X = -BODY_LEN * 0.5 + 0.028  # ~-0.050

# Iron / lever-cap incline geometry. The iron is bedded at BED_DEG, its
# cutting edge at the mouth, rising toward the heel (-X).
IRON_EDGE_X = MOUTH_X
IRON_EDGE_Z = SOLE_TOP_Z + 0.0005


def _bed_pt(u: float, m: float) -> tuple[float, float, float]:
    """Body-frame point from bed coordinates.

    u: distance up the 20 deg bed from the cutting edge (toward heel & up).
    m: distance along the outward bed normal (off the iron face).
    """
    return (
        IRON_EDGE_X - CQ_BED * u + SQ_BED * m,
        0.0,
        IRON_EDGE_Z + SQ_BED * u + CQ_BED * m,
    )


# Iron stack thickness levels along the bed normal m:
#   blade: 0 .. 0.003, chipbreaker: 0.003 .. 0.006,
#   lever-cap plate: ~0.006 .. 0.009, lever-cap spine top: ~0.006 .. 0.013.
SPINE_TOP_M = 0.006 + 0.011

# Lever-cap cam pivot: pin sits just above the cap spine near its top end.
CAM_PIVOT = _bed_pt(0.058, 0.016)
# Lateral lever pivot: on the blade's exposed top face past the chipbreaker.
LAT_PIVOT = _bed_pt(0.068, 0.003)
# Depth wheel center: behind the bed, forward of the rear grip.
DEPTH_CENTER = _bed_pt(0.035, -0.005)
# Depth-wheel stud root (inside the body bed) and cap-screw position.
STUD_BASE = _bed_pt(0.028, -0.005)
SCREW_POS = _bed_pt(0.023, 0.0)

BED_NORMAL = (SQ_BED, 0.0, CQ_BED)   # bed normal m (lateral lever axis)
BED_UP = (-CQ_BED, 0.0, SQ_BED)      # up-the-bed direction u (depth wheel axis)


def _tilt_back(wp: cq.Workplane) -> cq.Workplane:
    """Rotate a flat -X-extending part to the bed angle.

    A plate authored extending toward -X (heel) from the origin, rotated by
    BED_DEG about +Y, rises up (+Z) and toward the heel (-X): the real bed pose.
    """
    return wp.rotate((0, 0, 0), (0, 1, 0), BED_DEG)


# ===========================================================================
# BODY LAYER GEOMETRY
# ===========================================================================


def _block_plane_body() -> cq.Workplane:
    """Cast-iron sole + low sidewalls forming an open iron trough.

    No tall central casting — just low sidewalls running the body length
    with an open top between them where the iron sits. The mouth is a
    narrow through-slot in the sole at the foot of the bed.
    """
    half = BODY_LEN * 0.5
    trough_w = BODY_WIDTH - 2 * WALL_THICK

    # Full sole plate
    sole = cq.Workplane("XY").box(
        BODY_LEN, BODY_WIDTH, SOLE_THICK, centered=(True, True, False)
    )

    # Low sidewalls running nearly the full body length
    setback = 0.006
    wall_len = BODY_LEN - 2 * setback
    left_wall = (
        cq.Workplane("XY")
        .box(wall_len, WALL_THICK, WALL_HEIGHT, centered=(True, True, False))
        .translate((0.0, BODY_WIDTH * 0.5 - WALL_THICK * 0.5, SOLE_TOP_Z))
    )
    right_wall = (
        cq.Workplane("XY")
        .box(wall_len, WALL_THICK, WALL_HEIGHT, centered=(True, True, False))
        .translate((0.0, -(BODY_WIDTH * 0.5 - WALL_THICK * 0.5), SOLE_TOP_Z))
    )

    body = sole.union(left_wall).union(right_wall)

    # Blade mouth: a narrow through-slot in the sole between the walls.
    # Cut only through the sole thickness, preserving the sidewalls above.
    mouth = (
        cq.Workplane("XY")
        .box(0.006, trough_w - 0.004, SOLE_THICK + 0.002, centered=(True, True, False))
        .translate((MOUTH_X, 0.0, -0.001))
    )
    body = body.cut(mouth)

    # Front knob boss (small raised pad on sole for the knob to seat on)
    knob_boss = (
        cq.Workplane("XY")
        .circle(0.015)
        .extrude(0.002)
        .translate((KNOB_X, 0.0, SOLE_TOP_Z))
    )
    body = body.union(knob_boss)

    # Rear grip boss (rectangular pad for the palm grip to seat on)
    grip_boss = (
        cq.Workplane("XY")
        .box(0.040, 0.030, 0.002, centered=(True, True, False))
        .translate((TOTE_X, 0.0, SOLE_TOP_Z))
    )
    body = body.union(grip_boss)

    # Round the outer toe (+X) and heel (-X) vertical corners of the sole.
    body = body.edges("|Z").edges(
        BoxSelector((half - 0.015, -0.05, -0.002), (half + 0.01, 0.05, SOLE_THICK + 0.001))
    ).fillet(0.005)
    body = body.edges("|Z").edges(
        BoxSelector((-half - 0.01, -0.05, -0.002), (-half + 0.015, 0.05, SOLE_THICK + 0.001))
    ).fillet(0.005)
    return body


def _frog() -> cq.Workplane:
    """Machined bed plate integral with the body casting.

    A thin ramp at the bed angle filling the trough floor where the iron
    seats. Represents the machined bed surface of a block plane (no
    separate frog casting). Short enough to avoid the rear grip.
    """
    ramp = (
        cq.Workplane("XY")
        .box(0.042, 0.036, 0.004, centered=(False, True, False))
        .translate((-0.044, 0.0, -0.004))
    )
    ramp = _tilt_back(ramp)
    ramp = ramp.translate((IRON_EDGE_X, 0.0, IRON_EDGE_Z + 0.0002))
    return ramp


def _depth_stud() -> cq.Workplane:
    """Steel stud the depth wheel spins on: rooted inside the body bed, along u."""
    stud = cq.Workplane("XY").circle(0.002).extrude(0.020)
    stud = stud.rotate((0, 0, 0), (0, 1, 0), -(90.0 - BED_DEG))
    return stud.translate(STUD_BASE)


def _cap_screw() -> cq.Workplane:
    """Lever-cap screw: shaft from inside the body up through the iron slot
    and the lever cap's kidney slot, with a disc head resting on the spine."""
    shaft = (
        cq.Workplane("XY")
        .circle(0.003)
        .extrude(0.025)
        .translate((0.0, 0.0, -0.008))
    )
    head = (
        cq.Workplane("XY")
        .circle(0.0045)
        .extrude(0.003)
        .translate((0.0, 0.0, 0.016))
    )
    screw = shaft.union(head)
    screw = screw.rotate((0, 0, 0), (0, 1, 0), 90.0 - BED_DEG)
    return screw.translate(SCREW_POS)


# ===========================================================================
# CUTTING LAYER GEOMETRY
# ===========================================================================


def _cutting_iron() -> cq.Workplane:
    """Steel cutting iron + chipbreaker, bedded at 20 deg in the body frame."""
    blade_len = 0.088
    blade_w = 0.038
    blade_t = 0.0030

    # Authored with the cutting edge at the origin, body extending toward -X.
    blade = (
        cq.Workplane("XY")
        .box(blade_len, blade_w, blade_t, centered=(False, True, False))
        .translate((-blade_len, 0.0, 0.0))
    )
    cap = (
        cq.Workplane("XY")
        .box(0.068, blade_w - 0.006, 0.0030, centered=(False, True, False))
        .translate((-0.074, 0.0, blade_t))
    )
    cap = cap.edges("|Y and >Z").fillet(0.0010)
    slot = (
        cq.Workplane("XY")
        .box(0.058, 0.008, 0.02, centered=(False, True, True))
        .translate((-0.074, 0.0, blade_t + 0.0015))
    )
    iron = blade.union(cap).cut(slot)

    # Tilt to the bed angle, then place the cutting edge at the mouth.
    iron = _tilt_back(iron)
    iron = iron.translate((IRON_EDGE_X, 0.0, IRON_EDGE_Z))
    return iron


# ===========================================================================
# CLAMP LAYER GEOMETRY
# ===========================================================================


def _lever_cap() -> cq.Workplane:
    """Polished steel lever cap, clamped flush onto the chipbreaker."""
    # Authored extending toward -X from near the lower end, matching the iron.
    plate = (
        cq.Workplane("XY")
        .box(0.075, 0.036, 0.0030, centered=(False, True, False))
        .translate((-0.078, 0.0, 0.0))
    )
    plate = plate.edges("|Z").fillet(0.008)
    spine = (
        cq.Workplane("XY")
        .box(0.060, 0.015, 0.007, centered=(False, True, False))
        .translate((-0.072, 0.0, 0.003))
    )
    spine = spine.edges("|X").fillet(0.003)
    cap = plate.union(spine)
    # Kidney slot through the cap for the clamp screw (mid-length).
    slot = (
        cq.Workplane("XY")
        .box(0.030, 0.008, 0.02, centered=(False, True, True))
        .translate((-0.040, 0.0, -0.005))
    )
    cap = cap.cut(slot)

    cap = _tilt_back(cap)
    # Seat flush on the chipbreaker (plate underside at m = 0.006).
    cap = cap.translate(_bed_pt(0.002, 0.006))
    return cap


def _cam_lever() -> cq.Workplane:
    """Lever-cap flip cam lever, authored about its pin at the world origin.

    Pin axis is Y. The pivot boss is captured on the cap pin (embedded ~1 mm
    into the spine) and the flat finger lies on the spine's top surface,
    extending down the bed when clamped. Positive joint q flips it up.
    """
    boss = (
        cq.Workplane("XZ")
        .circle(0.0045)
        .extrude(0.014)
        .translate((0.0, 0.007, 0.0))
    )
    finger = (
        cq.Workplane("XY")
        .box(0.028, 0.013, 0.0035, centered=(False, True, False))
        .translate((0.003, 0.0, -0.0035))
    )
    finger = finger.edges("|Z and >X").fillet(0.004)
    cam = boss.union(finger)
    # Local +X -> down the bed, local +Z -> bed normal m: the finger lies flat
    # on the cap spine at q = 0.
    cam = cam.rotate((0, 0, 0), (0, 1, 0), BED_DEG)
    return cam


# ===========================================================================
# ADJUST LAYER GEOMETRY
# ===========================================================================


def _lateral_lever() -> cq.Workplane:
    """Lateral-adjustment lever, authored about its pivot at the origin.

    A flat strip lying ON the blade's top face: a pivot boss (embedded into the
    blade), a stem running up the bed past the iron's end, and a thumb disc.
    Rocking about the bed normal slides it flat over the iron face.
    """
    boss = (
        cq.Workplane("XY")
        .circle(0.004)
        .extrude(0.003)
        .translate((0.0, 0.0, -0.001))
    )
    stem = (
        cq.Workplane("XY")
        .box(0.028, 0.005, 0.002, centered=(False, True, False))
        .translate((-0.028, 0.0, 0.0))
    )
    disc = (
        cq.Workplane("XY")
        .circle(0.006)
        .extrude(0.002)
        .translate((-0.028, 0.0, 0.0))
    )
    lever = boss.union(stem).union(disc)
    # Local -X -> up the bed, local +Z -> bed normal m.
    lever = lever.rotate((0, 0, 0), (0, 1, 0), BED_DEG)
    return lever


def _depth_wheel() -> cq.Workplane:
    """Knurled brass depth wheel, disc axis along the bed direction u."""
    wheel = (
        cq.Workplane("XY")
        .circle(0.009)
        .extrude(0.007)
        .translate((0.0, 0.0, -0.0035))
    )
    # Knurl notches around the rim (repeated sub-part loop).
    n_knurls = 24
    for i in range(n_knurls):
        a = 2.0 * math.pi * i / n_knurls
        notch = (
            cq.Workplane("XY")
            .box(0.0014, 0.0014, 0.011, centered=(True, True, True))
            .translate((0.009 * math.cos(a), 0.009 * math.sin(a), 0.0))
        )
        wheel = wheel.cut(notch)
    bore = (
        cq.Workplane("XY")
        .circle(0.0022)
        .extrude(0.025)
        .translate((0.0, 0.0, -0.0125))
    )
    wheel = wheel.cut(bore)
    # Local +Z -> up-the-bed direction u (the stud axis).
    wheel = wheel.rotate((0, 0, 0), (0, 1, 0), -(90.0 - BED_DEG))
    return wheel


# ===========================================================================
# GRIP LAYER GEOMETRY
# ===========================================================================


def _front_knob() -> cq.Workplane:
    """Rosewood front knob: a turned mushroom shape (smaller block-plane size)."""
    prof = [
        (0.0, 0.0),
        (0.015, 0.0),
        (0.015, 0.004),
        (0.010, 0.007),
        (0.008, 0.016),
        (0.011, 0.025),
        (0.013, 0.030),
        (0.012, 0.034),
        (0.008, 0.037),
        (0.0, 0.038),
    ]
    knob = (
        cq.Workplane("XZ")
        .polyline([(r, z) for r, z in prof])
        .close()
        .revolve(360, (0, 0, 0), (0, 1, 0))
    )
    return knob


def _rear_grip() -> cq.Workplane:
    """Rosewood rear palm grip: a rounded dome for block-plane use."""
    grip = (
        cq.Workplane("XY")
        .box(0.040, 0.030, 0.024, centered=(True, True, False))
    )
    # Heavy fillet on top edges to create a dome shape.
    grip = grip.edges(">Z").fillet(0.009)
    # Light fillet on vertical edges for smooth contour.
    grip = grip.edges("|Z").fillet(0.004)
    return grip


# ===========================================================================
# BUILD OBJECT MODEL
# ===========================================================================


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="block_plane")

    cast_iron = model.material("cast_iron", rgba=(0.10, 0.10, 0.11, 1.0))
    bright_steel = model.material("bright_steel", rgba=(0.78, 0.79, 0.82, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.70, 0.71, 0.74, 1.0))
    rosewood = model.material("rosewood", rgba=(0.40, 0.16, 0.10, 1.0))
    brass = model.material("brass", rgba=(0.80, 0.62, 0.22, 1.0))

    # ---- BODY LAYER: cast-iron body / sole (root part) ----
    body = model.part("plane_body")
    body.visual(
        mesh_from_cadquery(_block_plane_body(), "plane_body"),
        material=cast_iron,
        name="casting",
    )
    sole_edge = (
        cq.Workplane("XY")
        .box(BODY_LEN - 0.004, BODY_WIDTH + 0.0004, 0.0012, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.005)
    )
    body.visual(
        mesh_from_cadquery(sole_edge, "sole_edge"),
        material=bright_steel,
        name="sole_edge",
    )
    body.visual(
        mesh_from_cadquery(_frog(), "frog"),
        material=cast_iron,
        name="frog",
    )
    body.visual(
        mesh_from_cadquery(_depth_stud(), "depth_stud"),
        material=bright_steel,
        name="depth_stud",
    )
    body.visual(
        mesh_from_cadquery(_cap_screw(), "cap_screw"),
        material=bright_steel,
        name="cap_screw",
    )

    # ---- GRIP LAYER: front knob (rosewood, fixed, seated on sole near toe) ----
    knob = model.part("front_knob")
    knob.visual(
        mesh_from_cadquery(_front_knob(), "front_knob"),
        origin=Origin(xyz=(KNOB_X, 0.0, SOLE_TOP_Z)),
        material=rosewood,
        name="knob",
    )
    model.articulation(
        "body_to_front_knob",
        ArticulationType.FIXED,
        parent=body,
        child=knob,
        origin=Origin(),
    )

    # ---- GRIP LAYER: rear palm grip (rosewood, fixed) ----
    tote = model.part("rear_grip")
    tote.visual(
        mesh_from_cadquery(_rear_grip(), "rear_grip"),
        origin=Origin(xyz=(TOTE_X, 0.0, SOLE_TOP_Z)),
        material=rosewood,
        name="grip",
    )
    model.articulation(
        "body_to_rear_grip",
        ArticulationType.FIXED,
        parent=body,
        child=tote,
        origin=Origin(),
    )

    # ---- CUTTING LAYER: cutting iron (steel, fixed, bedded on the frog) ----
    iron = model.part("cutting_iron")
    iron.visual(
        mesh_from_cadquery(_cutting_iron(), "cutting_iron"),
        material=blade_steel,
        name="iron",
    )
    model.articulation(
        "body_to_cutting_iron",
        ArticulationType.FIXED,
        parent=body,
        child=iron,
        origin=Origin(),
    )

    # ---- CLAMP LAYER: lever cap (polished steel, fixed, clamped on iron) ----
    cap = model.part("lever_cap")
    cap.visual(
        mesh_from_cadquery(_lever_cap(), "lever_cap"),
        material=bright_steel,
        name="cap",
    )
    model.articulation(
        "body_to_lever_cap",
        ArticulationType.FIXED,
        parent=body,
        child=cap,
        origin=Origin(),
    )

    # ---- PRIMARY: lever-cap cam lever (REVOLUTE) ----
    cam = model.part("cam_lever")
    cam.visual(
        mesh_from_cadquery(_cam_lever(), "cam_lever"),
        origin=Origin(),
        material=bright_steel,
        name="cam",
    )
    model.articulation(
        "lever_cap_cam",
        ArticulationType.REVOLUTE,
        parent=cap,
        child=cam,
        origin=Origin(xyz=CAM_PIVOT),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=1.5),
    )

    # ---- SECONDARY: lateral adjustment lever (REVOLUTE) ----
    lat = model.part("lateral_lever")
    lat.visual(
        mesh_from_cadquery(_lateral_lever(), "lateral_lever"),
        origin=Origin(),
        material=bright_steel,
        name="lever",
    )
    model.articulation(
        "lateral_adjust",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lat,
        origin=Origin(xyz=LAT_PIVOT),
        axis=BED_NORMAL,
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-0.35, upper=0.35),
    )

    # ---- TERTIARY: brass depth-adjustment wheel (CONTINUOUS) ----
    wheel = model.part("depth_wheel")
    wheel.visual(
        mesh_from_cadquery(_depth_wheel(), "depth_wheel"),
        origin=Origin(),
        material=brass,
        name="wheel",
    )
    model.articulation(
        "depth_adjust",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wheel,
        origin=Origin(xyz=DEPTH_CENTER),
        axis=BED_UP,
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )

    return model


# ===========================================================================
# TESTS
# ===========================================================================


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("plane_body")
    knob = object_model.get_part("front_knob")
    tote = object_model.get_part("rear_grip")
    iron = object_model.get_part("cutting_iron")
    cap = object_model.get_part("lever_cap")
    cam = object_model.get_part("cam_lever")
    lat = object_model.get_part("lateral_lever")
    wheel = object_model.get_part("depth_wheel")

    cam_joint = object_model.get_articulation("lever_cap_cam")
    lat_joint = object_model.get_articulation("lateral_adjust")
    depth_joint = object_model.get_articulation("depth_adjust")

    # ---- Block-plane body proportion checks ----
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "body is short (block plane proportions)",
        body_aabb is not None
        and (body_aabb[1][0] - body_aabb[0][0]) < 0.170,
        details=f"body x-extent: {body_aabb}",
    )
    ctx.check(
        "body is low-profile (no tall central casting)",
        body_aabb is not None
        and body_aabb[1][2] < WALL_TOP_Z + 0.008,
        details=f"body max z: {body_aabb[1][2] if body_aabb else None}",
    )

    # ---- Joint type / axis contract ----
    ctx.check(
        "cam lever is revolute",
        cam_joint.articulation_type == ArticulationType.REVOLUTE,
        details=str(cam_joint.articulation_type),
    )
    ctx.check(
        "cam lever pivots about Y",
        abs(cam_joint.axis[1]) > 0.99 and abs(cam_joint.axis[0]) < 0.01,
        details=str(cam_joint.axis),
    )
    ctx.check(
        "lateral lever is revolute",
        lat_joint.articulation_type == ArticulationType.REVOLUTE,
        details=str(lat_joint.articulation_type),
    )
    ctx.check(
        "lateral lever rocks about the bed normal",
        lat_joint.axis[0] > 0.2
        and lat_joint.axis[2] > 0.8
        and abs(lat_joint.axis[1]) < 0.01,
        details=str(lat_joint.axis),
    )
    ctx.check(
        "depth wheel is continuous",
        depth_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=str(depth_joint.articulation_type),
    )

    # ---- Low-angle iron geometry ----
    def cx(aabb):
        return 0.5 * (aabb[0][0] + aabb[1][0])

    def cy(aabb):
        return 0.5 * (aabb[0][1] + aabb[1][1])

    iron_aabb = ctx.part_world_aabb(iron)
    ctx.check(
        "cutting iron reaches the mouth/sole",
        iron_aabb is not None and iron_aabb[0][2] < SOLE_TOP_Z + 0.004,
        details=str(iron_aabb),
    )
    ctx.check(
        "iron is low-angle (moderate z-extent, not steep 45 deg)",
        iron_aabb is not None
        and 0.015 < (iron_aabb[1][2] - iron_aabb[0][2]) < 0.055,
        details=str(iron_aabb),
    )
    ctx.check(
        "iron rises from the mouth toward the heel",
        iron_aabb is not None and iron_aabb[0][0] < MOUTH_X + 0.001,
        details=str(iron_aabb),
    )

    # ---- Grip placement ----
    knob_aabb = ctx.part_world_aabb(knob)
    tote_aabb = ctx.part_world_aabb(tote)
    ctx.check(
        "front knob is forward of rear grip",
        knob_aabb is not None
        and tote_aabb is not None
        and cx(knob_aabb) > cx(tote_aabb) + 0.06,
        details=f"knob_cx={cx(knob_aabb)}, tote_cx={cx(tote_aabb)}",
    )
    ctx.check(
        "front knob rises above the sole",
        knob_aabb is not None and knob_aabb[1][2] > 0.035,
        details=str(knob_aabb),
    )

    cap_aabb = ctx.part_world_aabb(cap)
    ctx.check(
        "lever cap sits above the sole over the iron",
        cap_aabb is not None and cap_aabb[1][2] > WALL_TOP_Z,
        details=str(cap_aabb),
    )

    # ---- Intentional captured / seated / bedded overlap allowances ----
    ctx.allow_overlap(
        iron, body, elem_a="iron", elem_b="frog",
        reason="The cutting iron is bedded flat against the integral bed plate (0.2 mm seat embed).",
    )
    ctx.allow_overlap(
        cap, body, elem_a="cap", elem_b="frog",
        reason="The lever cap's lower end seats down onto the bed/iron seat.",
    )
    ctx.allow_overlap(
        cap, body, elem_a="cap", elem_b="cap_screw",
        reason="The cap screw's head bears down onto the lever-cap spine to clamp the stack.",
    )
    ctx.allow_overlap(
        iron, body, elem_a="iron", elem_b="cap_screw",
        reason="The cap screw shaft passes through the iron's clamp slot to reach the lever cap.",
    )
    ctx.allow_overlap(
        cap, iron, elem_a="cap", elem_b="iron",
        reason="The lever cap clamps flat onto the cutting iron.",
    )
    ctx.allow_overlap(
        cam, cap, elem_a="cam", elem_b="cap",
        reason="The cam lever's boss is captured on the lever-cap clamp pin and its finger lies on the spine.",
    )
    ctx.allow_overlap(
        cam, iron, elem_a="cam", elem_b="iron",
        reason="The cam lever bears down onto the iron through the lever cap when clamped.",
    )
    ctx.allow_overlap(
        lat, iron, elem_a="lever", elem_b="iron",
        reason="The lateral-adjustment lever's boss is riveted into the iron and its stem slides on the iron face.",
    )
    ctx.allow_overlap(
        wheel, body, elem_a="wheel", elem_b="frog",
        reason="The brass depth wheel is a captured rotor seated against the bed on its stud.",
    )
    ctx.allow_overlap(
        wheel, body, elem_a="wheel", elem_b="depth_stud",
        reason="The depth wheel's bore rides on the steel mounting stud.",
    )
    ctx.allow_overlap(
        wheel, body, elem_a="wheel", elem_b="casting",
        reason="The depth wheel sits between the sidewalls on the sole, contacting the trough floor.",
    )
    ctx.allow_overlap(
        wheel, iron, elem_a="wheel", elem_b="iron",
        reason="The depth wheel's rim engages the iron's adjuster slot from below.",
    )
    ctx.allow_overlap(
        knob, body, elem_a="knob", elem_b="casting",
        reason="The front knob base is seated/bolted down onto the toe boss of the sole.",
    )
    ctx.allow_overlap(
        tote, body, elem_a="grip", elem_b="casting",
        reason="The rear palm grip base is seated/bolted down onto the heel boss of the sole.",
    )
    ctx.allow_overlap(
        iron, tote, elem_a="iron", elem_b="grip",
        reason="The bedded iron extends toward the heel past the grip's front face, as on a real block plane where the iron passes behind the rear grip.",
    )

    # ---- Connectivity / seating proofs ----
    ctx.expect_contact(knob, body, contact_tol=0.004, name="front knob touches body")
    ctx.expect_contact(tote, body, contact_tol=0.004, name="rear grip touches body")
    ctx.expect_contact(iron, body, contact_tol=0.002, name="iron bedded on frog")
    ctx.expect_contact(cap, iron, contact_tol=0.002, name="lever cap clamps iron")
    ctx.expect_contact(cam, cap, contact_tol=0.002, name="cam lever rides on lever cap")
    ctx.expect_contact(lat, iron, contact_tol=0.002, name="lateral lever rides on iron")
    ctx.expect_contact(wheel, iron, contact_tol=0.002, name="depth wheel engages iron")
    ctx.expect_contact(wheel, body, contact_tol=0.002, name="depth wheel mounted on stud")

    # ---- PRIMARY mechanism actuates correctly & stays seated ----
    with ctx.pose({cam_joint: 0.0}):
        rest_top = ctx.part_world_aabb(cam)[1][2]
    with ctx.pose({cam_joint: 1.5}):
        lifted_top = ctx.part_world_aabb(cam)[1][2]
    ctx.check(
        "flipping the cam lever raises it",
        lifted_top > rest_top + 0.01,
        details=f"rest_top={rest_top}, lifted_top={lifted_top}",
    )
    with ctx.pose({cam_joint: 1.5}):
        ctx.expect_contact(
            cam, cap, contact_tol=0.002, name="cam boss stays on cap pin when flipped"
        )

    # ---- SECONDARY mechanism actuates correctly & stays seated ----
    with ctx.pose({lat_joint: 0.0}):
        lat_cy0 = cy(ctx.part_world_aabb(lat))
    with ctx.pose({lat_joint: 0.35}):
        lat_cy1 = cy(ctx.part_world_aabb(lat))
        ctx.expect_contact(
            lat, iron, contact_tol=0.002, name="lateral lever stays on iron when rocked"
        )
    ctx.check(
        "rocking the lateral lever shifts it sideways",
        abs(lat_cy1 - lat_cy0) > 0.003,
        details=f"cy0={lat_cy0}, cy1={lat_cy1}",
    )

    # ---- TERTIARY mechanism ----
    wheel_aabb = ctx.part_world_aabb(wheel)
    ctx.check(
        "depth wheel tucked behind the bed under the iron",
        wheel_aabb is not None
        and 0.5 * (wheel_aabb[0][2] + wheel_aabb[1][2]) < WALL_TOP_Z + 0.020,
        details=str(wheel_aabb),
    )
    with ctx.pose({depth_joint: math.pi}):
        ctx.expect_contact(
            wheel, body, contact_tol=0.002, name="depth wheel stays on stud when spun"
        )

    return ctx.report()


object_model = build_object_model()
