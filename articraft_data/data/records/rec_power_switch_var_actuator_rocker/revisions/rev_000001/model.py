from __future__ import annotations

# Wall-mounted toggle / draw-latch power switch, modeled from
# picture/Equipment/Power switch/001.png.
#
# Identity (from the reference):
#   A vertical rounded-rectangular faceplate with a raised perimeter border and a
#   recessed, chamfered-corner inner field. Near the top sits a raised louver pad
#   (four horizontal vent slots) flanked by two corner mounting screws. The centre
#   carries a wide rectangular rocker paddle in a shallow molded well.  The
#   paddle is the only moving child and see-saws about a central horizontal
#   X-axis pivot in the field. A recessed chamfered nameplate sits at the bottom.
#
# Coordinate convention (meters):
#   X = plate width (horizontal), Y = plate height (+Y up),
#   Z = depth out of the wall toward the viewer (+Z faces the room).
#
# Articulation:
#   ONE revolute joint = the rocker paddle pivoting about the horizontal
#   centreline (X). q=0 is neutral; positive q tips the top of the paddle
#   outward and the bottom inward like a real rocker switch.

import cadquery as cq

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
# Real-world dimensions (meters)
# ---------------------------------------------------------------------------
PLATE_W = 0.090          # plate width  (X)
PLATE_H = 0.130          # plate height (Y)
PLATE_T = 0.008          # plate thickness (Z); front face at Z=PLATE_T
PLATE_CORNER = 0.012     # rounded outer corner radius

# Recessed chamfered-corner inner field (leaves a raised perimeter border).
BORDER = 0.008           # border width in from the plate edge
FIELD_CHAMFER = 0.016    # chamfer of the inner field corners (octagonal look)
FIELD_DEPTH = 0.0018     # how deep the field is recessed into the front face

# Top louver / vent pad (a raised sub-panel carrying horizontal slots).
PAD_CY = PLATE_H / 2.0 - 0.024
PAD_W = 0.052
PAD_H = 0.028
PAD_PROUD = 0.0016                   # how far the pad stands above the field
LOUVER_W = 0.040
LOUVER_SLOT_H = 0.0016
LOUVER_SLOT_GAP = 0.0024
LOUVER_COUNT = 4
LOUVER_DEPTH = 0.0014

# Top mounting screws (flanking the louver pad).
SCREW_R = 0.0034
SCREW_HEAD_Z = 0.0012
SCREW_OFF_X = PLATE_W / 2.0 - 0.011
SCREW_OFF_Y = PLATE_H / 2.0 - 0.012

# Rocker well and paddle.
PADDLE_W = 0.058
PADDLE_H = 0.046
PADDLE_T = 0.0068
PADDLE_STANDOFF = 0.0012             # proud gap above the recessed field
PADDLE_CORNER = 0.006
PADDLE_CY = 0.000
PIVOT_Y = PADDLE_CY
PIVOT_Z = PLATE_T - FIELD_DEPTH      # actual field contact plane

WELL_W = PADDLE_W + 0.012
WELL_H = PADDLE_H + 0.012
WELL_INNER_W = PADDLE_W + 0.004
WELL_INNER_H = PADDLE_H + 0.004
WELL_CHAMFER = 0.007
WELL_PROUD = 0.0014
PIVOT_FOOT_W = PADDLE_W - 0.010
PIVOT_FOOT_H = 0.0040

# Bottom recessed nameplate (chamfered rectangle).
NAME_W = 0.044
NAME_H = 0.024
NAME_CY = -PLATE_H / 2.0 + 0.028
NAME_CHAMFER = 0.006
NAME_DEPTH = 0.0016

# Rocker throw: a real wall rocker only travels through a modest angle.
ROCK_LOWER = -0.24
ROCK_UPPER = 0.24


# ---------------------------------------------------------------------------
# Profile helper
# ---------------------------------------------------------------------------
def _chamfer_rect(w: float, h: float, c: float) -> list[tuple[float, float]]:
    """Centered rectangle with cut (chamfered) corners -> octagon profile."""
    hx, hy = w / 2.0, h / 2.0
    return [
        (-hx + c, -hy), (hx - c, -hy), (hx, -hy + c), (hx, hy - c),
        (hx - c, hy), (-hx + c, hy), (-hx, hy - c), (-hx, -hy + c),
    ]


# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------
def _register_materials(model: ArticulatedObject) -> None:
    model.material("plate_metal", color=(0.60, 0.62, 0.64))
    model.material("field_metal", color=(0.54, 0.56, 0.58))
    model.material("rivet_steel", color=(0.44, 0.46, 0.49))
    model.material("screw_zinc", color=(0.78, 0.80, 0.82))
    model.material("paddle_plastic", color=(0.70, 0.72, 0.73))
    model.material("paddle_shadow", color=(0.18, 0.19, 0.20))


# ---------------------------------------------------------------------------
# Faceplate (fixed root)
# ---------------------------------------------------------------------------
def _build_faceplate() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(PLATE_W, PLATE_H, PLATE_T, centered=(True, True, False))
        .edges("|Z")
        .fillet(PLATE_CORNER)
    )

    # Recessed chamfered inner field (leaves a raised perimeter border).
    field = _chamfer_rect(PLATE_W - 2.0 * BORDER, PLATE_H - 2.0 * BORDER, FIELD_CHAMFER)
    pocket = (
        cq.Workplane("XY")
        .workplane(offset=PLATE_T - FIELD_DEPTH)
        .polyline(field)
        .close()
        .extrude(FIELD_DEPTH + 0.001)
    )
    plate = plate.cut(pocket)

    # Bottom recessed nameplate (chamfered rectangle), cut deeper into the field,
    # leaving a raised inner border by cutting a smaller inner pocket deeper still.
    name = _chamfer_rect(NAME_W, NAME_H, NAME_CHAMFER)
    name_cut = (
        cq.Workplane("XY")
        .workplane(offset=PLATE_T - FIELD_DEPTH - NAME_DEPTH)
        .center(0.0, NAME_CY)
        .polyline(name)
        .close()
        .extrude(NAME_DEPTH + 0.001)
    )
    plate = plate.cut(name_cut)
    inner = _chamfer_rect(NAME_W - 0.010, NAME_H - 0.008, NAME_CHAMFER - 0.002)
    inner_cut = (
        cq.Workplane("XY")
        .workplane(offset=PLATE_T - FIELD_DEPTH - NAME_DEPTH - 0.0012)
        .center(0.0, NAME_CY)
        .polyline(inner)
        .close()
        .extrude(0.0014)
    )
    plate = plate.cut(inner_cut)

    # Four corner rivets on the nameplate border.
    name_face_z = PLATE_T - FIELD_DEPTH - NAME_DEPTH
    for sx in (-1, 1):
        for sy in (-1, 1):
            rivet = (
                cq.Workplane("XY")
                .workplane(offset=name_face_z - 0.0004)
                .center(sx * (NAME_W / 2.0 - 0.004), NAME_CY + sy * (NAME_H / 2.0 - 0.004))
                .circle(0.0015)
                .extrude(0.0008)
            )
            plate = plate.union(rivet)
    return plate


def _build_louver_pad() -> cq.Workplane:
    field_front = PLATE_T - FIELD_DEPTH
    pad = (
        cq.Workplane("XY")
        .workplane(offset=field_front)
        .center(0.0, PAD_CY)
        .box(PAD_W, PAD_H, FIELD_DEPTH + PAD_PROUD, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
    )
    pad_front = field_front + FIELD_DEPTH + PAD_PROUD
    band = LOUVER_COUNT * LOUVER_SLOT_H + (LOUVER_COUNT - 1) * LOUVER_SLOT_GAP
    y0 = PAD_CY + band / 2.0 - LOUVER_SLOT_H / 2.0
    for i in range(LOUVER_COUNT):
        cy = y0 - i * (LOUVER_SLOT_H + LOUVER_SLOT_GAP)
        slot = (
            cq.Workplane("XY")
            .workplane(offset=pad_front - LOUVER_DEPTH)
            .center(0.0, cy)
            .box(LOUVER_W, LOUVER_SLOT_H, LOUVER_DEPTH + 0.001, centered=(True, True, False))
        )
        pad = pad.cut(slot)
    return pad


def _build_rocker_well() -> cq.Workplane:
    """Raised chamfered well surrounding the rocker opening in the field."""
    field_front = PLATE_T - FIELD_DEPTH
    outer = _chamfer_rect(WELL_W, WELL_H, WELL_CHAMFER)
    inner = _chamfer_rect(
        WELL_INNER_W, WELL_INNER_H, max(0.002, WELL_CHAMFER - 0.004)
    )
    well = (
        cq.Workplane("XY")
        .workplane(offset=field_front)
        .center(0.0, PADDLE_CY)
        .polyline(outer)
        .close()
        .extrude(WELL_PROUD)
        .edges("|Z")
        .fillet(0.0016)
    )
    cutter = (
        cq.Workplane("XY")
        .workplane(offset=field_front - 0.0005)
        .center(0.0, PADDLE_CY)
        .polyline(inner)
        .close()
        .extrude(WELL_PROUD + 0.0012)
    )
    well = well.cut(cutter)

    # Two shallow shadow pockets above and below the pivot line make the well read
    # as an actual rocker recess without adding any fixed-decoration parts.
    for sy in (-1, 1):
        pocket = (
            cq.Workplane("XY")
            .workplane(offset=field_front + WELL_PROUD - 0.0004)
            .center(0.0, PADDLE_CY + sy * (PADDLE_H * 0.27))
            .box(PADDLE_W - 0.012, 0.0040, 0.0007, centered=(True, True, False))
            .edges("|Z")
            .fillet(0.0012)
        )
        well = well.cut(pocket)
    return well


def _build_screws() -> cq.Workplane:
    screws = None
    for sx in (-1, 1):
        head = (
            cq.Workplane("XY")
            .workplane(offset=PLATE_T - 0.0006)
            .center(sx * SCREW_OFF_X, SCREW_OFF_Y)
            .circle(SCREW_R)
            .extrude(SCREW_HEAD_Z + 0.0006)
        )
        slot_a = (
            cq.Workplane("XY")
            .workplane(offset=PLATE_T + SCREW_HEAD_Z - 0.0004)
            .center(sx * SCREW_OFF_X, SCREW_OFF_Y)
            .box(SCREW_R * 1.6, 0.0008, 0.001, centered=(True, True, False))
        )
        head = head.cut(slot_a)
        screws = head if screws is None else screws.union(head)
    return screws


# ---------------------------------------------------------------------------
# Rocker paddle (moving child, authored in the central X-axis pivot frame)
# ---------------------------------------------------------------------------
def _build_rocker_paddle() -> cq.Workplane:
    # Local frame: origin sits on the faceplate field at the real pivot line.
    # The narrow pivot foot touches the field at local z=0; the broad paddle
    # shell is stood off slightly so its top and bottom can see-saw into the well.
    shell = (
        cq.Workplane("XY")
        .workplane(offset=PADDLE_STANDOFF)
        .box(PADDLE_W, PADDLE_H, PADDLE_T, centered=(True, True, False))
        .edges("|Z")
        .fillet(PADDLE_CORNER)
        .faces(">Z")
        .edges()
        .fillet(0.0018)
    )

    # A shallow front recess leaves a molded rim; a transverse groove marks the
    # centre fulcrum line of the rocker.
    front_z = PADDLE_STANDOFF + PADDLE_T
    inset = (
        cq.Workplane("XY")
        .workplane(offset=front_z - 0.00045)
        .box(PADDLE_W - 0.010, PADDLE_H - 0.010, 0.0008, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0030)
    )
    groove = (
        cq.Workplane("XY")
        .workplane(offset=front_z - 0.00065)
        .box(PADDLE_W - 0.012, 0.0013, 0.0010, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0005)
    )
    shell = shell.cut(inset).cut(groove)

    foot = (
        cq.Workplane("XY")
        .box(PIVOT_FOOT_W, PIVOT_FOOT_H, PADDLE_STANDOFF + 0.00015, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0016)
    )
    return shell.union(foot)


def _build_top_mark() -> cq.Workplane:
    # Start slightly below the recessed paddle face so the dark inlay is a real
    # embedded island connected to the moving paddle geometry, not a floating decal.
    front_z = PADDLE_STANDOFF + PADDLE_T - 0.00065
    return (
        cq.Workplane("XY")
        .workplane(offset=front_z)
        .center(0.0, PADDLE_H * 0.25)
        .box(0.0022, 0.0100, 0.00095, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.00045)
    )


def _build_bottom_mark() -> cq.Workplane:
    front_z = PADDLE_STANDOFF + PADDLE_T - 0.00065
    return (
        cq.Workplane("XY")
        .workplane(offset=front_z)
        .center(0.0, -PADDLE_H * 0.25)
        .circle(0.0036)
        .circle(0.0023)
        .extrude(0.00095)
    )


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_rocker_power_switch")
    _register_materials(model)

    faceplate = model.part("faceplate")
    faceplate.visual(
        mesh_from_cadquery(_build_faceplate(), "faceplate_shell"),
        origin=Origin(),
        material="plate_metal",
        name="faceplate_shell",
    )
    faceplate.visual(
        mesh_from_cadquery(_build_louver_pad(), "louver_pad"),
        origin=Origin(),
        material="field_metal",
        name="louver_pad",
    )
    faceplate.visual(
        mesh_from_cadquery(_build_rocker_well(), "rocker_well"),
        origin=Origin(),
        material="field_metal",
        name="rocker_well",
    )
    faceplate.visual(
        mesh_from_cadquery(_build_screws(), "mount_screws"),
        origin=Origin(),
        material="screw_zinc",
        name="mount_screws",
    )

    paddle = model.part("rocker_paddle")
    paddle.visual(
        mesh_from_cadquery(_build_rocker_paddle(), "paddle_shell"),
        origin=Origin(),
        material="paddle_plastic",
        name="paddle_shell",
    )
    paddle.visual(
        mesh_from_cadquery(_build_top_mark(), "top_mark"),
        origin=Origin(),
        material="paddle_shadow",
        name="top_mark",
    )
    paddle.visual(
        mesh_from_cadquery(_build_bottom_mark(), "bottom_mark"),
        origin=Origin(),
        material="paddle_shadow",
        name="bottom_mark",
    )

    model.articulation(
        "plate_to_paddle",
        ArticulationType.REVOLUTE,
        parent=faceplate,
        child=paddle,
        origin=Origin(xyz=(0.0, PIVOT_Y, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=4.0, lower=ROCK_LOWER, upper=ROCK_UPPER
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    faceplate = object_model.get_part("faceplate")
    paddle = object_model.get_part("rocker_paddle")
    hinge = object_model.get_articulation("plate_to_paddle")

    ctx.check(
        "faceplate is the single root",
        faceplate in object_model.root_parts() and len(object_model.root_parts()) == 1,
        details=f"roots={[p.name for p in object_model.root_parts()]}",
    )
    ctx.check(
        "paddle is hinge child",
        hinge.child == "rocker_paddle" and hinge.parent == "faceplate",
        details=f"parent={hinge.parent}, child={hinge.child}",
    )
    ctx.check(
        "joint type is revolute",
        str(hinge.articulation_type).lower().endswith("revolute"),
        details=f"type={hinge.articulation_type}",
    )
    ax = tuple(float(a) for a in hinge.axis)
    ctx.check(
        "hinge axis is horizontal X",
        abs(ax[0]) > 0.99 and abs(ax[1]) < 1e-6 and abs(ax[2]) < 1e-6,
        details=f"axis={ax}",
    )

    # Faceplate reads as a vertical wall plate.
    plate_aabb = ctx.part_element_world_aabb(faceplate, elem="faceplate_shell")
    ctx.check("faceplate aabb present", plate_aabb is not None, details=str(plate_aabb))
    if plate_aabb is not None:
        pmin, pmax = plate_aabb
        pw = float(pmax[0] - pmin[0])
        ph = float(pmax[1] - pmin[1])
        pt = float(pmax[2] - pmin[2])
        ctx.check("plate width matches", abs(pw - PLATE_W) < 0.002, details=f"w={pw}")
        ctx.check("plate height matches", abs(ph - PLATE_H) < 0.002, details=f"h={ph}")
        ctx.check("plate is taller than wide", ph > pw + 0.02, details=f"w={pw}, h={ph}")
        ctx.check("plate is thin slab", pt < 0.013, details=f"t={pt}")

    # The replacement moving member is a wide rectangular rocker paddle, not a
    # roller bail: it is broad, nearly fills the well, and stands proud of the
    # recessed field.
    paddle_aabb = ctx.part_element_world_aabb(paddle, elem="paddle_shell")
    ctx.check("paddle aabb present", paddle_aabb is not None, details=str(paddle_aabb))
    if paddle_aabb is not None:
        rmin, rmax = paddle_aabb
        pw = float(rmax[0] - rmin[0])
        ph = float(rmax[1] - rmin[1])
        pf = float(rmax[2])
        ctx.check(
            "paddle is a wide rectangle",
            pw > 0.050 and ph > 0.038 and pw > ph,
            details=f"paddle_w={pw}, paddle_h={ph}",
        )
        ctx.check(
            "paddle stands proud of recessed field",
            pf > (PLATE_T - FIELD_DEPTH) + PADDLE_T,
            details=f"paddle_front_z={pf}",
        )

    well_aabb = ctx.part_element_world_aabb(faceplate, elem="rocker_well")
    ctx.check("rocker well aabb present", well_aabb is not None, details=str(well_aabb))
    if well_aabb is not None and paddle_aabb is not None:
        wmin, wmax = well_aabb
        pmin, pmax = paddle_aabb
        ctx.check(
            "well frames the paddle",
            float(wmax[0] - wmin[0]) > float(pmax[0] - pmin[0])
            and float(wmax[1] - wmin[1]) > float(pmax[1] - pmin[1]),
            details=f"well={well_aabb}, paddle={paddle_aabb}",
        )

    ctx.expect_contact(
        paddle,
        faceplate,
        elem_a="paddle_shell",
        elem_b="faceplate_shell",
        contact_tol=0.0002,
        name="paddle pivot foot touches field",
    )

    # The paddle sits in front of the plate and overlaps its footprint.
    ctx.expect_overlap(
        paddle,
        faceplate,
        axes="xy",
        elem_a="paddle_shell",
        elem_b="faceplate_shell",
        min_overlap=0.035,
        name="paddle overlaps plate footprint",
    )

    # Mechanism: the rocker see-saws about the X axis. The top marking moves
    # outward while the bottom marking moves inward at the positive limit.
    def _mark_front(elem_name: str):
        box = ctx.part_element_world_aabb(paddle, elem=elem_name)
        if box is None:
            return None
        mn, mx = box
        return float(mx[2])

    with ctx.pose({hinge: 0.0}):
        top_neutral = _mark_front("top_mark")
        bottom_neutral = _mark_front("bottom_mark")
    ctx.check(
        "rocker markings present",
        top_neutral is not None and bottom_neutral is not None,
        details=f"top={top_neutral}, bottom={bottom_neutral}",
    )

    with ctx.pose({hinge: ROCK_UPPER}):
        top_pressed = _mark_front("top_mark")
        bottom_pressed = _mark_front("bottom_mark")

    ctx.check(
        "positive throw tips top outward",
        top_neutral is not None and top_pressed is not None and top_pressed > top_neutral + 0.003,
        details=f"top_neutral={top_neutral}, top_pressed={top_pressed}",
    )
    ctx.check(
        "positive throw tips bottom inward",
        bottom_neutral is not None
        and bottom_pressed is not None
        and bottom_pressed < bottom_neutral - 0.0018,
        details=f"bottom_neutral={bottom_neutral}, bottom_pressed={bottom_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
