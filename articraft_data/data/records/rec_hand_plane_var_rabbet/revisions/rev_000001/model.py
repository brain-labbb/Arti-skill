from __future__ import annotations

# Realistic articulated rabbet/shoulder bench hand plane.
#
# Fork variant of the Stanley-style bench plane: a narrow rabbet/shoulder
# plane where one side cheek (+Y) is open so the cutting iron runs the full
# body width out to the side wall, replacing the closed-throat smoothing body.
#
# Real object identity:
#   - A cast-iron rabbet plane with a black japanned body and a bright machined
#     sole edge, a rosewood front knob and rear tote, an angled steel cutting
#     iron (blade + chipbreaker) seated on the frog at nearly full body width,
#     a polished steel lever cap clamping the iron flush, the lever cap's flip
#     cam lever lying flat on the cap, a small lateral-adjustment lever riding
#     on the iron's upper end, and a knurled brass depth-adjustment wheel on a
#     steel stud behind the frog.
#
# Body outline change vs. parent smoothing plane:
#   - Width narrowed from 62 mm to 30 mm (rabbet/shoulder proportions).
#   - The -Y side cheek is full height (35 mm above sole) for rigidity.
#   - The +Y side cheek is open: only a 3 mm lip remains so the iron's cutting
#     edge reaches the full body width for flush rabbet/shoulder cuts.
#
# Articulated mechanism (identical to parent):
#   - PRIMARY: lever-cap cam lever (REVOLUTE)
#   - SECONDARY: lateral-adjustment lever (REVOLUTE)
#   - TERTIARY: brass depth wheel (CONTINUOUS)
#
# Authoring convention:
#   - All geometry is authored directly in the final BODY frame (meters).
#   - The plane runs along +X (toe/front at +X, heel/rear at -X).
#   - The flat sole bottom sits on z = 0; up is +Z; width is along Y.
#   - The iron bed is a 45 deg plane. Helper coordinates: u runs up the bed
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
# Key dimensions (medium rabbet/shoulder plane: ~0.200 m long, ~0.030 m wide).
# ---------------------------------------------------------------------------
BODY_LEN = 0.200
BODY_WIDTH = 0.030
SOLE_THICK = 0.008
CHEEK_HEIGHT = 0.035        # closed side (-Y) cheek height above sole plate
OPEN_CHEEK_HEIGHT = 0.003   # open side (+Y) — thin lip only
SOLE_TOP_Z = SOLE_THICK
BODY_TOP_Z = SOLE_TOP_Z + CHEEK_HEIGHT

MOUTH_X = 0.015             # blade mouth slot center along +X
BED_DEG = 45.0
SQ = math.sqrt(0.5)

KNOB_X = BODY_LEN * 0.5 - 0.035
TOTE_X = -0.048

IRON_EDGE_X = MOUTH_X
IRON_EDGE_Z = SOLE_TOP_Z + 0.0005


def _bed_pt(u: float, m: float) -> tuple[float, float, float]:
    """Body-frame point from bed coordinates (u up the 45 deg bed, m normal)."""
    return (
        IRON_EDGE_X - SQ * u + SQ * m,
        0.0,
        IRON_EDGE_Z + SQ * u + SQ * m,
    )


# Iron stack thickness levels along the bed normal m.
SPINE_TOP_M = 0.0068 + 0.013

# Joint pivot / mount positions (bed coordinates → body frame).
CAM_PIVOT = _bed_pt(0.094, 0.0238)
LAT_PIVOT = _bed_pt(0.105, 0.003)
DEPTH_CENTER = _bed_pt(0.085, -0.0085)
STUD_BASE = _bed_pt(0.068, -0.0085)
SCREW_POS = _bed_pt(0.0518, 0.0)

BED_NORMAL = (SQ, 0.0, SQ)   # bed normal m (lateral lever axis)
BED_UP = (-SQ, 0.0, SQ)      # up-the-bed direction u (depth wheel axis)


def _tilt_back(wp: cq.Workplane) -> cq.Workplane:
    """Rotate a flat -X-extending part to the 45 deg bed angle."""
    return wp.rotate((0, 0, 0), (0, 1, 0), BED_DEG)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _rabbet_plane_body() -> cq.Workplane:
    """Cast-iron sole + asymmetric mid block for a rabbet/shoulder plane.

    The -Y side has a full-height cheek wall. The +Y side is open (thin lip
    only) so the cutting iron extends to the full body width for flush cuts.
    """
    sole = cq.Workplane("XY").box(
        BODY_LEN, BODY_WIDTH, SOLE_THICK, centered=(True, True, False)
    )

    # Full-width, full-height central block
    mid_len = 0.090
    block = (
        cq.Workplane("XY")
        .box(mid_len, BODY_WIDTH, CHEEK_HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.0, SOLE_TOP_Z))
    )

    # Cut away the +Y upper portion to create the open cheek.
    # Leaves only OPEN_CHEEK_HEIGHT lip on the +Y side.
    cut_h = CHEEK_HEIGHT - OPEN_CHEEK_HEIGHT + 0.002
    open_cut = (
        cq.Workplane("XY")
        .box(mid_len + 0.004, BODY_WIDTH * 0.5 + 0.002, cut_h,
             centered=(True, False, False))
        .translate((0.0, -0.001, SOLE_TOP_Z + OPEN_CHEEK_HEIGHT))
    )
    block = block.cut(open_cut)

    # 45 deg iron slot: body-width channel through both cheeks.
    slot = (
        cq.Workplane("XY")
        .box(0.180, BODY_WIDTH, 0.023, centered=(False, True, False))
        .translate((-0.180, 0.0, -0.0115))
    )
    # Narrow relief for the raised lever-cap spine.
    relief = (
        cq.Workplane("XY")
        .box(0.170, BODY_WIDTH - 0.006, 0.010, centered=(False, True, False))
        .translate((-0.170, 0.0, 0.008))
    )
    channel = _tilt_back(slot.union(relief)).translate(
        (IRON_EDGE_X, 0.0, IRON_EDGE_Z)
    )
    block = block.cut(channel)

    # Low bosses at the toe and heel for the knob/tote bolt-down.
    toe_boss = (
        cq.Workplane("XY")
        .circle(0.013)
        .extrude(0.002)
        .translate((KNOB_X, 0.0, SOLE_TOP_Z))
    )
    heel_boss = (
        cq.Workplane("XY")
        .box(0.046, 0.024, 0.002, centered=(True, True, False))
        .translate((TOTE_X - 0.021, 0.0, SOLE_TOP_Z))
    )
    body = sole.union(block).union(toe_boss).union(heel_boss)

    # Blade mouth: a narrow through-slot in the sole only (not extending above
    # the sole top), narrower than the body so the sole stays connected.
    mouth = (
        cq.Workplane("XY")
        .box(0.004, BODY_WIDTH - 0.006, SOLE_THICK + 0.002,
             centered=(True, True, False))
        .translate((MOUTH_X, 0.0, -0.001))
    )
    body = body.cut(mouth)

    # Fillet only the outer toe (+X) and heel (-X) vertical corners.
    half = BODY_LEN * 0.5
    body = body.edges("|Z").edges(
        BoxSelector((half - 0.015, -0.05, -0.002), (half + 0.01, 0.05, 0.009))
    ).fillet(0.005)
    body = body.edges("|Z").edges(
        BoxSelector((-half - 0.01, -0.05, -0.002), (-half + 0.015, 0.05, 0.009))
    ).fillet(0.005)
    return body


def _frog() -> cq.Workplane:
    """Cast frog bed plate, narrowed for the rabbet body."""
    ramp = (
        cq.Workplane("XY")
        .box(0.068, BODY_WIDTH - 0.006, 0.012, centered=(False, True, False))
        .translate((-0.070, 0.0, -0.012))
    )
    ramp = _tilt_back(ramp)
    ramp = ramp.translate((IRON_EDGE_X, 0.0, IRON_EDGE_Z + 0.0003))
    return ramp


def _depth_stud() -> cq.Workplane:
    """Steel stud the depth wheel spins on, rooted inside the frog."""
    stud = cq.Workplane("XY").circle(0.002).extrude(0.022)
    stud = stud.rotate((0, 0, 0), (0, 1, 0), -45.0)
    return stud.translate(STUD_BASE)


def _cap_screw() -> cq.Workplane:
    """Lever-cap screw: shaft through the iron slot with a disc head."""
    shaft = (
        cq.Workplane("XY")
        .circle(0.003)
        .extrude(0.028)
        .translate((0.0, 0.0, -0.010))
    )
    head = (
        cq.Workplane("XY")
        .circle(0.0045)
        .extrude(0.003)
        .translate((0.0, 0.0, 0.018))
    )
    screw = shaft.union(head)
    screw = screw.rotate((0, 0, 0), (0, 1, 0), 45.0)
    return screw.translate(SCREW_POS)


def _cutting_iron() -> cq.Workplane:
    """Steel cutting iron + chipbreaker at full body width for rabbet cuts."""
    blade_len = 0.110
    blade_w = BODY_WIDTH - 0.002   # nearly full body width (1 mm inset/side)
    blade_t = 0.003

    blade = (
        cq.Workplane("XY")
        .box(blade_len + 0.003, blade_w, blade_t, centered=(False, True, False))
        .translate((-blade_len, 0.0, 0.0))
    )
    cap = (
        cq.Workplane("XY")
        .box(0.085, blade_w - 0.006, 0.004, centered=(False, True, False))
        .translate((-0.090, 0.0, blade_t))
    )
    cap = cap.edges("|Y and >Z").fillet(0.001)
    slot = (
        cq.Workplane("XY")
        .box(0.075, 0.008, 0.02, centered=(False, True, True))
        .translate((-0.090, 0.0, blade_t + 0.002))
    )
    iron = blade.union(cap).cut(slot)
    iron = _tilt_back(iron)
    iron = iron.translate((IRON_EDGE_X, 0.0, IRON_EDGE_Z))
    return iron


def _lever_cap() -> cq.Workplane:
    """Polished steel lever cap, narrowed for the rabbet body."""
    plate = (
        cq.Workplane("XY")
        .box(0.100, BODY_WIDTH - 0.004, 0.004, centered=(False, True, False))
        .translate((-0.100, 0.0, 0.0))
    )
    plate = plate.edges("|Z").fillet(0.006)
    spine = (
        cq.Workplane("XY")
        .box(0.086, 0.014, 0.009, centered=(False, True, False))
        .translate((-0.098, 0.0, 0.004))
    )
    spine = spine.edges("|X").fillet(0.0025)
    cap = plate.union(spine)
    # Kidney slot for the clamp screw.
    slot = (
        cq.Workplane("XY")
        .box(0.038, 0.008, 0.02, centered=(False, True, True))
        .translate((-0.050, 0.0, -0.01))
    )
    cap = cap.cut(slot)
    cap = _tilt_back(cap)
    cap = cap.translate(_bed_pt(0.0018, 0.0068))
    return cap


def _cam_lever() -> cq.Workplane:
    """Lever-cap flip cam lever, authored about its pin at the world origin."""
    boss = (
        cq.Workplane("XZ")
        .circle(0.005)
        .extrude(0.016)
        .translate((0.0, 0.008, 0.0))
    )
    finger = (
        cq.Workplane("XY")
        .box(0.028, 0.013, 0.004, centered=(False, True, False))
        .translate((0.004, 0.0, -0.004))
    )
    finger = finger.edges("|Z and >X").fillet(0.004)
    cam = boss.union(finger)
    cam = cam.rotate((0, 0, 0), (0, 1, 0), 45.0)
    return cam


def _lateral_lever() -> cq.Workplane:
    """Lateral-adjustment lever, authored about its pivot at the origin."""
    boss = (
        cq.Workplane("XY")
        .circle(0.004)
        .extrude(0.0035)
        .translate((0.0, 0.0, -0.001))
    )
    stem = (
        cq.Workplane("XY")
        .box(0.030, 0.005, 0.002, centered=(False, True, False))
        .translate((-0.030, 0.0, 0.0))
    )
    disc = (
        cq.Workplane("XY")
        .circle(0.006)
        .extrude(0.002)
        .translate((-0.030, 0.0, 0.0))
    )
    lever = boss.union(stem).union(disc)
    lever = lever.rotate((0, 0, 0), (0, 1, 0), 45.0)
    return lever


def _depth_wheel() -> cq.Workplane:
    """Knurled brass depth wheel, smaller radius for the narrow body."""
    wheel = (
        cq.Workplane("XY")
        .circle(0.008)
        .extrude(0.008)
        .translate((0.0, 0.0, -0.004))
    )
    # Knurl notches around the rim (repeated sub-part pattern).
    n_knurls = 20
    for i in range(n_knurls):
        a = 2.0 * math.pi * i / n_knurls
        notch = (
            cq.Workplane("XY")
            .box(0.0014, 0.0014, 0.012, centered=(True, True, True))
            .translate((0.008 * math.cos(a), 0.008 * math.sin(a), 0.0))
        )
        wheel = wheel.cut(notch)
    bore = (
        cq.Workplane("XY")
        .circle(0.0019)
        .extrude(0.025)
        .translate((0.0, 0.0, -0.0125))
    )
    wheel = wheel.cut(bore)
    # Local +Z -> up-the-bed direction u (the stud axis).
    wheel = wheel.rotate((0, 0, 0), (0, 1, 0), -45.0)
    return wheel


def _front_knob() -> cq.Workplane:
    """Rosewood front knob (smaller for the narrow rabbet body)."""
    prof = [
        (0.0, 0.0),
        (0.014, 0.0),
        (0.014, 0.005),
        (0.010, 0.008),
        (0.008, 0.018),
        (0.011, 0.028),
        (0.013, 0.036),
        (0.013, 0.042),
        (0.009, 0.046),
        (0.0, 0.048),
    ]
    knob = (
        cq.Workplane("XZ")
        .polyline([(r, z) for r, z in prof])
        .close()
        .revolve(360, (0, 0, 0), (0, 1, 0))
    )
    return knob


def _rear_tote() -> cq.Workplane:
    """Rosewood rear tote (D-handle), narrower extrusion for the rabbet body."""
    outline = [
        (0.000, 0.000),
        (0.050, 0.000),
        (0.048, 0.016),
        (0.044, 0.026),
        (0.050, 0.044),
        (0.056, 0.068),
        (0.050, 0.096),
        (0.038, 0.108),
        (0.024, 0.106),
        (0.018, 0.090),
        (0.024, 0.072),
        (0.024, 0.052),
        (0.014, 0.040),
        (0.000, 0.030),
    ]
    handle = (
        cq.Workplane("XZ")
        .polyline(outline)
        .close()
        .extrude(0.018)
        .translate((0.0, -0.009, 0.0))
    )
    hole = (
        cq.Workplane("XZ")
        .center(0.034, 0.058)
        .ellipse(0.010, 0.020)
        .extrude(0.04)
        .translate((0.0, -0.02, 0.0))
    )
    handle = handle.cut(hole)
    handle = handle.edges("|Y").fillet(0.0012)
    # Mirror so the grip leans back toward the heel (-X).
    handle = handle.rotate((0, 0, 0), (0, 0, 1), 180.0)
    return handle


# ---------------------------------------------------------------------------
# Build model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rabbet_hand_plane")

    cast_iron = model.material("cast_iron", rgba=(0.10, 0.10, 0.11, 1.0))
    bright_steel = model.material("bright_steel", rgba=(0.78, 0.79, 0.82, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.70, 0.71, 0.74, 1.0))
    rosewood = model.material("rosewood", rgba=(0.40, 0.16, 0.10, 1.0))
    brass = model.material("brass", rgba=(0.80, 0.62, 0.22, 1.0))

    # ----- Root: cast-iron body / sole -----\
    body = model.part("plane_body")
    body.visual(
        mesh_from_cadquery(_rabbet_plane_body(), "plane_body"),
        material=cast_iron,
        name="casting",
    )
    sole_edge = (
        cq.Workplane("XY")
        .box(BODY_LEN - 0.006, BODY_WIDTH + 0.0006, 0.0015,
             centered=(True, True, False))
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

    # ----- Front knob (rosewood, fixed, seated on sole near the toe) -----\
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
        parent=body, child=knob,
        origin=Origin(),
    )

    # ----- Rear tote / handle (rosewood, fixed) -----\
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
        parent=body, child=tote,
        origin=Origin(),
    )

    # ----- Cutting iron (steel, fixed, bedded on the frog) -----\
    iron = model.part("cutting_iron")
    iron.visual(
        mesh_from_cadquery(_cutting_iron(), "cutting_iron"),
        material=blade_steel,
        name="iron",
    )
    model.articulation(
        "body_to_cutting_iron",
        ArticulationType.FIXED,
        parent=body, child=iron,
        origin=Origin(),
    )

    # ----- Lever cap (polished steel, fixed, clamped flush on the iron) -----\
    cap = model.part("lever_cap")
    cap.visual(
        mesh_from_cadquery(_lever_cap(), "lever_cap"),
        material=bright_steel,
        name="cap",
    )
    model.articulation(
        "body_to_lever_cap",
        ArticulationType.FIXED,
        parent=body, child=cap,
        origin=Origin(),
    )

    # ----- PRIMARY: lever-cap cam lever (REVOLUTE) -----\
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
        parent=cap, child=cam,
        origin=Origin(xyz=CAM_PIVOT),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=1.5),
    )

    # ----- SECONDARY: lateral adjustment lever (REVOLUTE) -----\
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
        parent=body, child=lat,
        origin=Origin(xyz=LAT_PIVOT),
        axis=BED_NORMAL,
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-0.35, upper=0.35),
    )

    # ----- TERTIARY: brass depth-adjustment wheel (CONTINUOUS) -----\
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
        parent=body, child=wheel,
        origin=Origin(xyz=DEPTH_CENTER),
        axis=BED_UP,
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

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

    # ---- Rabbet-plane-specific geometry claims ----
    body_aabb = ctx.part_world_aabb(body)
    body_width = body_aabb[1][1] - body_aabb[0][1]
    ctx.check(
        "body is narrow (rabbet/shoulder plane width)",
        body_width < 0.045,
        details=f"body_width={body_width:.4f}",
    )

    iron_aabb = ctx.part_world_aabb(iron)
    iron_width = iron_aabb[1][1] - iron_aabb[0][1]
    ctx.check(
        "cutting iron spans nearly full body width",
        iron_width > body_width * 0.80,
        details=f"iron_width={iron_width:.4f}, body_width={body_width:.4f}",
    )
    ctx.check(
        "iron extends to open side (+Y) edge",
        iron_aabb[1][1] > body_aabb[1][1] - 0.005,
        details=f"iron_max_y={iron_aabb[1][1]:.4f}, body_max_y={body_aabb[1][1]:.4f}",
    )
    ctx.check(
        "iron extends to closed side (-Y) edge",
        iron_aabb[0][1] < body_aabb[0][1] + 0.005,
        details=f"iron_min_y={iron_aabb[0][1]:.4f}, body_min_y={body_aabb[0][1]:.4f}",
    )
    ctx.check(
        "body rises to cheek height on closed side",
        body_aabb[1][2] > BODY_TOP_Z - 0.005,
        details=f"body_max_z={body_aabb[1][2]:.4f}, expected~{BODY_TOP_Z:.4f}",
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

    # ---- Hero parts present & placed ----
    def cx(aabb):
        return 0.5 * (aabb[0][0] + aabb[1][0])

    knob_aabb = ctx.part_world_aabb(knob)
    tote_aabb = ctx.part_world_aabb(tote)
    ctx.check(
        "front knob is forward of rear tote",
        knob_aabb is not None
        and tote_aabb is not None
        and cx(knob_aabb) > cx(tote_aabb) + 0.06,
        details=f"knob_cx={cx(knob_aabb)}, tote_cx={cx(tote_aabb)}",
    )
    ctx.check(
        "front knob rises above the sole",
        knob_aabb is not None and knob_aabb[1][2] > 0.04,
        details=str(knob_aabb),
    )
    ctx.check(
        "rear tote is a tall handle",
        tote_aabb is not None and (tote_aabb[1][2] - tote_aabb[0][2]) > 0.07,
        details=str(tote_aabb),
    )

    ctx.check(
        "cutting iron reaches the mouth/sole",
        iron_aabb is not None and iron_aabb[0][2] < SOLE_TOP_Z + 0.004,
        details=str(iron_aabb),
    )
    ctx.check(
        "cutting iron is bedded at an incline",
        iron_aabb is not None and (iron_aabb[1][2] - iron_aabb[0][2]) > 0.05,
        details=str(iron_aabb),
    )

    cap_aabb = ctx.part_world_aabb(cap)
    ctx.check(
        "lever cap sits above the sole over the iron",
        cap_aabb is not None and cap_aabb[1][2] > BODY_TOP_Z,
        details=str(cap_aabb),
    )

    # ---- Intentional captured / seated / bedded overlap allowances ----
    ctx.allow_overlap(
        iron, body, elem_a="iron", elem_b="frog",
        reason="The cutting iron is bedded flat against the cast frog plate (seat embed).",
    )
    ctx.allow_overlap(
        iron, body, elem_a="iron", elem_b="casting",
        reason="The cutting iron passes through the body's iron slot with minimal clearance.",
    )
    ctx.allow_overlap(
        cap, body, elem_a="cap", elem_b="frog",
        reason="The lever cap's lower end seats down onto the frog/iron seat.",
    )
    ctx.allow_overlap(
        cap, body, elem_a="cap", elem_b="casting",
        reason="The lever cap passes through the body's iron slot region.",
    )
    ctx.allow_overlap(
        cap, body, elem_a="cap", elem_b="cap_screw",
        reason="The cap screw's head bears down onto the lever-cap spine to clamp.",
    )
    ctx.allow_overlap(
        cap, iron, elem_a="cap", elem_b="iron",
        reason="The lever cap clamps flat onto the cutting iron.",
    )
    ctx.allow_overlap(
        cam, cap, elem_a="cam", elem_b="cap",
        reason="The cam lever's boss is captured on the lever-cap clamp pin.",
    )
    ctx.allow_overlap(
        cam, iron, elem_a="cam", elem_b="iron",
        reason="The cam lever bears down onto the iron through the lever cap.",
    )
    ctx.allow_overlap(
        lat, iron, elem_a="lever", elem_b="iron",
        reason="The lateral lever's boss is riveted into the iron and slides on the face.",
    )
    ctx.allow_overlap(
        wheel, body, elem_a="wheel", elem_b="frog",
        reason="The brass depth wheel is a captured rotor seated against the frog.",
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
        reason="The front knob base is seated/bolted onto the toe boss.",
    )
    ctx.allow_overlap(
        tote, body, elem_a="tote", elem_b="casting",
        reason="The rear tote foot is seated/bolted onto the heel boss.",
    )
    ctx.allow_overlap(
        tote, iron, elem_a="tote", elem_b="iron",
        reason="The tote's lower throat sits just behind the bedded iron.",
    )

    # ---- Connectivity / seating proofs ----
    ctx.expect_contact(knob, body, contact_tol=0.004, name="front knob touches body")
    ctx.expect_contact(tote, body, contact_tol=0.004, name="rear tote touches body")
    ctx.expect_contact(iron, body, contact_tol=0.002, name="iron bedded on frog")
    ctx.expect_contact(cap, iron, contact_tol=0.002, name="lever cap clamps iron")
    ctx.expect_contact(cam, cap, contact_tol=0.002, name="cam lever rides on lever cap")
    ctx.expect_contact(lat, iron, contact_tol=0.002, name="lateral lever rides on iron")
    ctx.expect_contact(wheel, iron, contact_tol=0.002, name="depth wheel engages iron")
    ctx.expect_contact(wheel, body, contact_tol=0.002, name="depth wheel on frog stud")

    # ---- PRIMARY mechanism: cam lever flips up ----
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
            cam, cap, contact_tol=0.005,
            name="cam boss stays on cap pin when flipped",
        )

    # ---- SECONDARY mechanism: lateral lever rocks sideways ----
    def cy(aabb):
        return 0.5 * (aabb[0][1] + aabb[1][1])

    with ctx.pose({lat_joint: 0.0}):
        lat_cy0 = cy(ctx.part_world_aabb(lat))
    with ctx.pose({lat_joint: 0.35}):
        lat_cy1 = cy(ctx.part_world_aabb(lat))
        ctx.expect_contact(
            lat, iron, contact_tol=0.002,
            name="lateral lever stays on iron when rocked",
        )
    ctx.check(
        "rocking the lateral lever shifts it sideways",
        abs(lat_cy1 - lat_cy0) > 0.003,
        details=f"cy0={lat_cy0}, cy1={lat_cy1}",
    )

    # ---- TERTIARY mechanism: depth wheel spins ----
    wheel_aabb = ctx.part_world_aabb(wheel)
    ctx.check(
        "depth wheel tucked behind the frog under the iron",
        wheel_aabb is not None
        and 0.5 * (wheel_aabb[0][2] + wheel_aabb[1][2]) < BODY_TOP_Z + 0.025,
        details=str(wheel_aabb),
    )
    with ctx.pose({depth_joint: math.pi}):
        ctx.expect_contact(
            wheel, body, contact_tol=0.002,
            name="depth wheel stays on stud when spun",
        )

    return ctx.report()


object_model = build_object_model()
