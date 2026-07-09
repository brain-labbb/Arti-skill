from __future__ import annotations

# Realistic articulated Stanley-style No.4/No.5 bench hand plane (smoothing plane).
#
# Real object identity (from the reference image):
#   - A cast-iron bench plane with a black japanned body and a bright machined
#     sole edge, a rosewood front knob and rear tote (handle), an angled steel
#     cutting iron (blade + chipbreaker) seated on the frog, a polished steel
#     lever cap clamping the iron flush, the lever cap's flip cam lever lying
#     flat on the cap, a small lateral-adjustment lever riding on the iron's
#     upper end, and a knurled brass depth-adjustment wheel on a steel stud
#     behind the frog, under the iron.
#
# Articulated mechanism:
#   - PRIMARY: the lever-cap cam lever flips up/down about a horizontal pin to
#     clamp or release the cutting iron (REVOLUTE). Its pivot boss stays
#     captured on the cap pin at every joint angle.
#   - SECONDARY: the lateral-adjustment lever rocks about the bed normal so it
#     slides flat over the iron face while skewing the blade (REVOLUTE).
#   - TERTIARY: the brass depth wheel spins on its stud to advance/retract the
#     iron (CONTINUOUS). The stud is fixed in the frog and passes through the
#     wheel bore, so the wheel is physically mounted, not floating.
#
# Authoring convention:
#   - All geometry is authored directly in the final BODY frame (meters).
#   - The plane runs along +X (toe/front at +X, heel/rear at -X).
#   - The flat sole bottom sits on z = 0; up is +Z; width is along Y.
#   - The iron bed is a 45 deg plane. Helper coordinates: u runs up the bed
#     from the mouth (toward heel/+Z), m is the outward bed normal.
#   - Every fixed part's mesh is built in body-world coordinates with identity
#     articulation origins. Moving parts are authored about their pivot at the
#     local origin; the joint origin places the pivot in body-world.

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
# Key dimensions (a No.4 smoothing plane is ~0.245 m long, ~0.062 m wide).
# ---------------------------------------------------------------------------
BODY_LEN = 0.245
BODY_WIDTH = 0.062
SOLE_THICK = 0.010  # machined sole plate thickness
CHEEK_HEIGHT = 0.040  # solid cast mid-block height above the sole plate
CHEEK_THICK = 0.005  # minimum sidewall thickness beside the iron slot
SOLE_TOP_Z = SOLE_THICK  # top of the flat sole plate
BODY_TOP_Z = SOLE_TOP_Z + CHEEK_HEIGHT  # top of the cast mid block

MOUTH_X = 0.018  # blade mouth slot center, measured from center along +X
BED_DEG = 45.0  # frog bed angle for the iron
SQ = math.sqrt(0.5)  # sin/cos of the 45 deg bed angle

# Where the front knob and rear tote seat on the sole.
KNOB_X = BODY_LEN * 0.5 - 0.040
TOTE_X = -0.058  # forward foot of the rear tote; it leans back over the heel

# Iron / lever-cap incline geometry. The iron is bedded at BED_DEG, its cutting
# edge at the mouth, rising toward the heel (-X).
IRON_EDGE_X = MOUTH_X  # cutting edge x
IRON_EDGE_Z = SOLE_TOP_Z + 0.0005


def _bed_pt(u: float, m: float) -> tuple[float, float, float]:
    """Body-frame point from bed coordinates.

    u: distance up the 45 deg bed from the cutting edge (toward heel & up).
    m: distance along the outward bed normal (off the iron face).
    """
    return (
        IRON_EDGE_X - SQ * u + SQ * m,
        0.0,
        IRON_EDGE_Z + SQ * u + SQ * m,
    )


# Iron stack thickness levels measured along the bed normal m:
#   blade: 0 .. 0.003, chipbreaker: 0.003 .. 0.007,
#   lever-cap plate: ~0.0068 .. 0.0108, lever-cap spine top: ~0.0198.
SPINE_TOP_M = 0.0068 + 0.013

# Lever-cap cam pivot: pin sits just above the cap spine near its top end.
CAM_PIVOT = _bed_pt(0.094, 0.0238)
# Lateral lever pivot: on the blade's exposed top face past the chipbreaker.
LAT_PIVOT = _bed_pt(0.105, 0.003)
# Depth wheel center: behind the frog, under the iron's upper end.
DEPTH_CENTER = _bed_pt(0.085, -0.0085)
# Depth-wheel stud root (inside the frog body) and cap-screw position.
STUD_BASE = _bed_pt(0.068, -0.0085)
SCREW_POS = _bed_pt(0.0518, 0.0)

BED_NORMAL = (SQ, 0.0, SQ)  # bed normal m (lateral lever axis)
BED_UP = (-SQ, 0.0, SQ)  # up-the-bed direction u (depth wheel axis)


def _tilt_back(wp: cq.Workplane) -> cq.Workplane:
    """Rotate a flat -X-extending part to the bed angle.

    A plate authored extending toward -X (heel) from the origin, rotated +45 deg
    about +Y, rises up (+Z) and toward the heel (-X): the real frog/iron pose.
    """
    return wp.rotate((0, 0, 0), (0, 1, 0), BED_DEG)


def _bench_plane_body() -> cq.Workplane:
    """Cast-iron sole + SOLID mid block with a tight slanted iron slot.

    The mid section is a solid casting; the only openings are the 45 deg iron
    slot (sized to the iron + lever-cap stack with ~2 mm clearance), a narrow
    relief for the lever-cap spine, and the through mouth in the sole. The toe
    (+X) and heel (-X) ends stay as the low flat sole platform where the front
    knob and rear tote are bolted.
    """
    sole = cq.Workplane("XY").box(
        BODY_LEN, BODY_WIDTH, SOLE_THICK, centered=(True, True, False)
    )

    # Solid central block (no hollow cavity).
    block = (
        cq.Workplane("XY")
        .box(0.102, BODY_WIDTH, CHEEK_HEIGHT, centered=(True, True, False))
        .translate((0.001, 0.0, SOLE_TOP_Z))
    )

    # Slanted iron slot: a 45 deg channel hugging the iron + lever-cap stack.
    # m range -0.0123 (frog underside) .. +0.013 (2 mm over the cap plate).
    slot = (
        cq.Workplane("XY")
        .box(0.204, 0.048, 0.0253, centered=(False, True, False))
        .translate((-0.200, 0.0, -0.0123))
    )
    # Narrow extra relief where the raised lever-cap spine passes the block.
    relief = (
        cq.Workplane("XY")
        .box(0.190, 0.023, 0.0115, centered=(False, True, False))
        .translate((-0.200, 0.0, 0.010))
    )
    channel = _tilt_back(slot.union(relief)).translate(
        (IRON_EDGE_X, 0.0, IRON_EDGE_Z)
    )
    block = block.cut(channel)

    # Low raised bosses at the toe and heel that the knob/tote bolt onto.
    toe_boss = (
        cq.Workplane("XY")
        .circle(0.021)
        .extrude(0.0020)
        .translate((KNOB_X, 0.0, SOLE_TOP_Z))
    )
    heel_boss = (
        cq.Workplane("XY")
        .box(0.058, 0.030, 0.0020, centered=(True, True, False))
        .translate((TOTE_X - 0.026, 0.0, SOLE_TOP_Z))
    )
    body = sole.union(block).union(toe_boss).union(heel_boss)

    # Blade mouth: a narrow through-slot in the sole at the foot of the bed.
    mouth = (
        cq.Workplane("XY")
        .box(0.008, BODY_WIDTH - 0.018, SOLE_THICK + 0.02, centered=(True, True, True))
        .translate((MOUTH_X, 0.0, SOLE_THICK * 0.5))
    )
    body = body.cut(mouth)

    # Round only the outer toe (+X) and heel (-X) vertical corners of the sole.
    half = BODY_LEN * 0.5
    body = body.edges("|Z").edges(
        BoxSelector((half - 0.020, -0.05, -0.002), (half + 0.01, 0.05, 0.009))
    ).fillet(0.007)
    body = body.edges("|Z").edges(
        BoxSelector((-half - 0.01, -0.05, -0.002), (-half + 0.020, 0.05, 0.009))
    ).fillet(0.007)
    return body


def _frog() -> cq.Workplane:
    """Cast frog bed plate that fills the slot below the iron.

    A 12.5 mm thick 45 deg plate whose top face is the iron bed; it exactly
    fills the under-bed half of the body's iron slot, so the iron sits on a
    solid bed with no gaps. Seated with a 0.2 mm embed into the blade.
    """
    ramp = (
        cq.Workplane("XY")
        .box(0.077, 0.048, 0.0125, centered=(False, True, False))
        .translate((-0.079, 0.0, -0.0125))
    )
    ramp = _tilt_back(ramp)
    ramp = ramp.translate((IRON_EDGE_X, 0.0, IRON_EDGE_Z + 0.0003))
    return ramp


def _depth_stud() -> cq.Workplane:
    """Steel stud the depth wheel spins on: rooted inside the frog, along u."""
    stud = cq.Workplane("XY").circle(0.0023).extrude(0.024)
    stud = stud.rotate((0, 0, 0), (0, 1, 0), -45.0)
    return stud.translate(STUD_BASE)


def _cap_screw() -> cq.Workplane:
    """Lever-cap screw: shaft from inside the frog up through the iron slot
    and the lever cap's kidney slot, with a disc head resting on the spine."""
    shaft = (
        cq.Workplane("XY")
        .circle(0.0035)
        .extrude(0.030)
        .translate((0.0, 0.0, -0.010))
    )
    head = (
        cq.Workplane("XY")
        .circle(0.005)
        .extrude(0.0035)
        .translate((0.0, 0.0, 0.0195))
    )
    screw = shaft.union(head)
    screw = screw.rotate((0, 0, 0), (0, 1, 0), 45.0)
    return screw.translate(SCREW_POS)


def _cutting_iron() -> cq.Workplane:
    """Steel cutting iron + chipbreaker, bedded in the body frame."""
    blade_len = 0.120
    blade_w = BODY_WIDTH - 0.020
    blade_t = 0.0030

    # Authored with the cutting edge near the origin, body extending toward -X.
    # The extra 3 mm past the origin drops the edge into the sole mouth.
    blade = (
        cq.Workplane("XY")
        .box(blade_len + 0.003, blade_w, blade_t, centered=(False, True, False))
        .translate((-blade_len, 0.0, 0.0))
    )
    cap = (
        cq.Workplane("XY")
        .box(0.095, blade_w - 0.006, 0.0040, centered=(False, True, False))
        .translate((-0.100, 0.0, blade_t))
    )
    cap = cap.edges("|Y and >Z").fillet(0.0012)
    slot = (
        cq.Workplane("XY")
        .box(0.085, 0.009, 0.02, centered=(False, True, True))
        .translate((-0.100, 0.0, blade_t + 0.002))
    )
    iron = blade.union(cap).cut(slot)

    # Tilt to the bed angle (rises up and toward the heel), then put the cutting
    # edge at the mouth on the sole.
    iron = _tilt_back(iron)
    iron = iron.translate((IRON_EDGE_X, 0.0, IRON_EDGE_Z))
    return iron


def _lever_cap() -> cq.Workplane:
    """Polished steel lever cap, clamped flush onto the chipbreaker."""
    # Authored extending toward -X from near the lower end, matching the iron.
    plate = (
        cq.Workplane("XY")
        .box(0.100, 0.040, 0.0040, centered=(False, True, False))
        .translate((-0.100, 0.0, 0.0))
    )
    plate = plate.edges("|Z").fillet(0.010)
    spine = (
        cq.Workplane("XY")
        .box(0.086, 0.018, 0.009, centered=(False, True, False))
        .translate((-0.098, 0.0, 0.004))
    )
    spine = spine.edges("|X").fillet(0.003)
    cap = plate.union(spine)
    # Kidney slot through the cap for the clamp screw (mid-length).
    slot = (
        cq.Workplane("XY")
        .slot2D(0.038, 0.009, 0.0)
        .extrude(0.05)
        .translate((-0.050, 0.0, -0.01))
    )
    cap = cap.cut(slot)

    cap = _tilt_back(cap)
    # Seat flush on the chipbreaker (plate underside at m = 0.0068: a 0.2 mm
    # clamping embed into the chipbreaker's 0.007 top face).
    cap = cap.translate(_bed_pt(0.0018, 0.0068))
    return cap


def _cam_lever() -> cq.Workplane:
    """Lever-cap flip cam lever, authored about its pin at the world origin.

    Pin axis is Y. The pivot boss is captured on the cap pin (embedded ~1 mm
    into the spine) and the flat finger lies on the spine's top surface,
    extending down the bed when clamped. Positive joint q flips it up.
    """
    boss = (
        cq.Workplane("XZ")
        .circle(0.0050)
        .extrude(0.016)
        .translate((0.0, 0.008, 0.0))
    )
    finger = (
        cq.Workplane("XY")
        .box(0.032, 0.015, 0.0040, centered=(False, True, False))
        .translate((0.004, 0.0, -0.0042))
    )
    finger = finger.edges("|Z and >X").fillet(0.005)
    cam = boss.union(finger)
    # Local +X -> down the bed, local +Z -> bed normal m: the finger lies flat
    # on the cap spine at q = 0.
    cam = cam.rotate((0, 0, 0), (0, 1, 0), 45.0)
    return cam


def _lateral_lever() -> cq.Workplane:
    """Lateral-adjustment lever, authored about its pivot at the origin.

    A flat strip lying ON the blade's top face: a pivot boss (embedded into the
    blade), a stem running up the bed past the iron's end, and a thumb disc.
    Rocking about the bed normal slides it flat over the iron face.
    """
    boss = (
        cq.Workplane("XY")
        .circle(0.0045)
        .extrude(0.004)
        .translate((0.0, 0.0, -0.001))
    )
    stem = (
        cq.Workplane("XY")
        .box(0.034, 0.006, 0.0025, centered=(False, True, False))
        .translate((-0.034, 0.0, 0.0))
    )
    disc = (
        cq.Workplane("XY")
        .circle(0.0065)
        .extrude(0.0025)
        .translate((-0.034, 0.0, 0.0))
    )
    lever = boss.union(stem).union(disc)
    # Local -X -> up the bed, local +Z -> bed normal m.
    lever = lever.rotate((0, 0, 0), (0, 1, 0), 45.0)
    return lever


def _depth_wheel() -> cq.Workplane:
    """Knurled brass depth wheel, disc axis along the bed direction u."""
    wheel = (
        cq.Workplane("XY")
        .circle(0.010)
        .extrude(0.0075)
        .translate((0.0, 0.0, -0.00375))
    )
    n = 24
    for i in range(n):
        a = 2.0 * math.pi * i / n
        notch = (
            cq.Workplane("XY")
            .box(0.0016, 0.0016, 0.012, centered=(True, True, True))
            .translate((0.010 * math.cos(a), 0.010 * math.sin(a), 0.0))
        )
        wheel = wheel.cut(notch)
    bore = (
        cq.Workplane("XY")
        .circle(0.0024)
        .extrude(0.03)
        .translate((0.0, 0.0, -0.015))
    )
    wheel = wheel.cut(bore)
    # Local +Z -> up-the-bed direction u (the stud axis).
    wheel = wheel.rotate((0, 0, 0), (0, 1, 0), -45.0)
    return wheel


def _front_knob() -> cq.Workplane:
    """Rosewood front knob: a turned mushroom shape, base on z=0."""
    prof = [
        (0.0, 0.0),
        (0.020, 0.0),
        (0.020, 0.006),
        (0.013, 0.010),
        (0.011, 0.022),
        (0.014, 0.034),
        (0.018, 0.044),
        (0.018, 0.052),
        (0.012, 0.058),
        (0.0, 0.060),
    ]
    knob = (
        cq.Workplane("XZ")
        .polyline([(r, z) for r, z in prof])
        .close()
        .revolve(360, (0, 0, 0), (0, 1, 0))
    )
    return knob


def _rear_tote() -> cq.Workplane:
    """Rosewood rear tote (D-handle), base on z=0, leaning back toward -X."""
    outline = [
        (0.000, 0.000),
        (0.060, 0.000),
        (0.058, 0.018),
        (0.052, 0.030),
        (0.060, 0.050),
        (0.066, 0.078),
        (0.060, 0.110),
        (0.046, 0.124),
        (0.030, 0.122),
        (0.022, 0.104),
        (0.030, 0.082),
        (0.030, 0.060),
        (0.018, 0.046),
        (0.000, 0.034),
    ]
    handle = (
        cq.Workplane("XZ")
        .polyline(outline)
        .close()
        .extrude(0.024)
        .translate((0.0, 0.012, 0.0))
    )
    hole = (
        cq.Workplane("XZ")
        .center(0.040, 0.066)
        .ellipse(0.013, 0.026)
        .extrude(0.05)
        .translate((0.0, -0.01, 0.0))
    )
    handle = handle.cut(hole)
    handle = handle.edges("|Y").fillet(0.0015)
    # Mirror so the grip leans back (toward the heel, -X) and seat at TOTE_X.
    handle = handle.rotate((0, 0, 0), (0, 0, 1), 180.0)
    return handle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_hand_plane")

    cast_iron = model.material("cast_iron", rgba=(0.10, 0.10, 0.11, 1.0))
    bright_steel = model.material("bright_steel", rgba=(0.78, 0.79, 0.82, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.70, 0.71, 0.74, 1.0))
    rosewood = model.material("rosewood", rgba=(0.40, 0.16, 0.10, 1.0))
    brass = model.material("brass", rgba=(0.80, 0.62, 0.22, 1.0))

    # ----- Root: cast-iron body / sole (geometry already in body frame) -----
    body = model.part("plane_body")
    body.visual(
        mesh_from_cadquery(_bench_plane_body(), "plane_body"),
        material=cast_iron,
        name="casting",
    )
    sole_edge = (
        cq.Workplane("XY")
        .box(BODY_LEN - 0.006, BODY_WIDTH + 0.0006, 0.0015, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.009)
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

    # ----- Front knob (rosewood, fixed, seated on sole near the toe) -----
    knob = model.part("front_knob")
    knob.visual(
        mesh_from_cadquery(_front_knob(), "front_knob"),
        # Author identity-placed; position via the visual origin in body frame.
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

    # ----- Rear tote / handle (rosewood, fixed) -----
    tote = model.part("rear_tote")
    tote.visual(
        mesh_from_cadquery(_rear_tote(), "rear_tote"),
        origin=Origin(xyz=(TOTE_X, 0.0, SOLE_TOP_Z)),
        material=rosewood,
        name="tote",
    )
    model.articulation(
        "body_to_rear_tote",
        ArticulationType.FIXED,
        parent=body,
        child=tote,
        origin=Origin(),
    )

    # ----- Cutting iron (steel, fixed, bedded on the frog) -----
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

    # ----- Lever cap (polished steel, fixed, clamped flush on the iron) -----
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

    # ----- PRIMARY: lever-cap cam lever (REVOLUTE) -----
    # The cam flips up to release the iron, down to clamp it. The mesh is built
    # about the pin at its own origin; the joint origin places the pin on the
    # lever cap. The boss stays captured on the pin at every angle and the
    # finger lies flat on the cap spine at q = 0.
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
        # The cam is mounted on the lever cap; cap geometry is in body frame and
        # its part frame is identity, so the pivot in the cap frame == CAM_PIVOT.
        child=cam,
        origin=Origin(xyz=CAM_PIVOT),
        # Finger extends down-bed from the pin; -Y lifts it upward as q grows.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=1.5),
    )

    # ----- SECONDARY: lateral adjustment lever (REVOLUTE) -----
    # Rocks about the bed normal, so it slides flat over the iron face and
    # never lifts off its seat.
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

    # ----- TERTIARY: brass depth-adjustment wheel (CONTINUOUS) -----
    # Spins about its mounting stud (fixed in the frog), axis along the bed.
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


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("plane_body")
    knob = object_model.get_part("front_knob")
    tote = object_model.get_part("rear_tote")
    iron = object_model.get_part("cutting_iron")
    cap = object_model.get_part("lever_cap")
    cam = object_model.get_part("cam_lever")
    lat = object_model.get_part("lateral_lever")
    wheel = object_model.get_part("depth_wheel")

    cam_joint = object_model.get_articulation("lever_cap_cam")
    lat_joint = object_model.get_articulation("lateral_adjust")
    depth_joint = object_model.get_articulation("depth_adjust")

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
        lat_joint.axis[0] > 0.5
        and lat_joint.axis[2] > 0.5
        and abs(lat_joint.axis[1]) < 0.01,
        details=str(lat_joint.axis),
    )
    ctx.check(
        "depth wheel is continuous",
        depth_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=str(depth_joint.articulation_type),
    )

    # ---- Hero parts present & placed (use AABBs: fixed parts keep an identity
    #      part origin and carry placement only in their visual origins) ----
    def cx(aabb):
        return 0.5 * (aabb[0][0] + aabb[1][0])

    def cy(aabb):
        return 0.5 * (aabb[0][1] + aabb[1][1])

    knob_aabb = ctx.part_world_aabb(knob)
    tote_aabb = ctx.part_world_aabb(tote)
    ctx.check(
        "front knob is forward of rear tote",
        knob_aabb is not None
        and tote_aabb is not None
        and cx(knob_aabb) > cx(tote_aabb) + 0.10,
        details=f"knob_cx={cx(knob_aabb)}, tote_cx={cx(tote_aabb)}",
    )
    ctx.check(
        "front knob rises above the sole",
        knob_aabb is not None and knob_aabb[1][2] > 0.05,
        details=str(knob_aabb),
    )
    ctx.check(
        "rear tote is a tall handle",
        tote_aabb is not None and (tote_aabb[1][2] - tote_aabb[0][2]) > 0.09,
        details=str(tote_aabb),
    )

    iron_aabb = ctx.part_world_aabb(iron)
    ctx.check(
        "cutting iron reaches the mouth/sole",
        iron_aabb is not None and iron_aabb[0][2] < SOLE_TOP_Z + 0.004,
        details=str(iron_aabb),
    )
    ctx.check(
        "cutting iron is bedded at an incline",
        iron_aabb is not None and (iron_aabb[1][2] - iron_aabb[0][2]) > 0.06,
        details=str(iron_aabb),
    )
    # The iron's cutting edge is at the toe-ward (mouth) end; it rises toward
    # the heel, so its low point is forward (+X) of its high point.
    ctx.check(
        "iron rises from the mouth toward the heel",
        iron_aabb is not None and iron_aabb[0][0] < MOUTH_X + 0.001,
        details=str(iron_aabb),
    )

    cap_aabb = ctx.part_world_aabb(cap)
    ctx.check(
        "lever cap sits above the sole over the iron",
        cap_aabb is not None and cap_aabb[1][2] > BODY_TOP_Z,
        details=str(cap_aabb),
    )

    # ---- Intentional captured / seated / bedded overlap allowances ----
    # These are real mechanical fits, not stray collisions: the iron is bedded
    # on the frog, the lever cap and its cam clamp down onto the iron, the
    # lateral lever slides flat on the iron face, the cap screw pins the stack
    # to the frog, and the depth wheel is a captured rotor on its frog stud.
    ctx.allow_overlap(
        iron, body, elem_a="iron", elem_b="frog",
        reason="The cutting iron is bedded flat against the cast frog plate (0.2 mm seat embed).",
    )
    ctx.allow_overlap(
        cap, body, elem_a="cap", elem_b="frog",
        reason="The lever cap's lower end seats down onto the frog/iron seat.",
    )
    ctx.allow_overlap(
        cap, body, elem_a="cap", elem_b="cap_screw",
        reason="The cap screw's head bears down onto the lever-cap spine to clamp the stack.",
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
        reason="The brass depth wheel is a captured rotor seated against the frog on its stud.",
    )
    ctx.allow_overlap(
        wheel, body, elem_a="wheel", elem_b="depth_stud",
        reason="The depth wheel's bore rides on the steel mounting stud.",
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
        tote, body, elem_a="tote", elem_b="casting",
        reason="The rear tote foot is seated/bolted down onto the heel boss of the sole.",
    )
    ctx.allow_overlap(
        tote, iron, elem_a="tote", elem_b="iron",
        reason="The tote's lower throat sits just behind the bedded iron and shares a small seam.",
    )

    # ---- Connectivity / seating proofs ----
    # Knob and tote seat on top of the sole bosses (mounted, not floating).
    ctx.expect_contact(knob, body, contact_tol=0.004, name="front knob touches body")
    ctx.expect_contact(tote, body, contact_tol=0.004, name="rear tote touches body")
    # Iron bedded on the frog; lever cap clamps the iron; cam rides the cap.
    ctx.expect_contact(iron, body, contact_tol=0.002, name="iron bedded on frog")
    ctx.expect_contact(cap, iron, contact_tol=0.002, name="lever cap clamps iron")
    ctx.expect_contact(cam, cap, contact_tol=0.002, name="cam lever rides on lever cap")
    # Lateral lever rides the iron; depth wheel engages the iron's adjuster
    # slot and is mounted on its stud in the frog (its real grounding path).
    ctx.expect_contact(lat, iron, contact_tol=0.002, name="lateral lever rides on iron")
    ctx.expect_contact(wheel, iron, contact_tol=0.002, name="depth wheel engages iron")
    ctx.expect_contact(wheel, body, contact_tol=0.002, name="depth wheel mounted on frog stud")

    # ---- PRIMARY mechanism actuates correctly & stays seated ----
    # Flipping the cam (positive q) raises its free finger up and back, while
    # the pivot boss stays captured on the cap pin.
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
    # Rocking the lateral lever sweeps the whole lever sideways (in Y).
    with ctx.pose({lat_joint: 0.0}):
        lat_cy0 = cy(ctx.part_world_aabb(lat))
    with ctx.pose({lat_joint: 0.35}):
        lat_cy1 = cy(ctx.part_world_aabb(lat))
        ctx.expect_contact(
            lat, iron, contact_tol=0.002, name="lateral lever stays on iron when rocked"
        )
    ctx.check(
        "rocking the lateral lever shifts it sideways",
        abs(lat_cy1 - lat_cy0) > 0.005,
        details=f"cy0={lat_cy0}, cy1={lat_cy1}",
    )

    # ---- TERTIARY mechanism ----
    wheel_aabb = ctx.part_world_aabb(wheel)
    ctx.check(
        "depth wheel tucked behind the frog under the iron",
        wheel_aabb is not None
        and 0.5 * (wheel_aabb[0][2] + wheel_aabb[1][2]) < BODY_TOP_Z + 0.030,
        details=str(wheel_aabb),
    )
    with ctx.pose({depth_joint: math.pi}):
        ctx.expect_contact(
            wheel, body, contact_tol=0.002, name="depth wheel stays on stud when spun"
        )

    return ctx.report()


object_model = build_object_model()
