from __future__ import annotations

import math

# Wall-mounted toggle / draw-latch power switch, modeled from
# picture/Equipment/Power switch/001.png.
#
# Identity (from the reference):
#   A vertical rounded-rectangular faceplate with a raised perimeter border and a
#   recessed, chamfered-corner inner field. Near the top sits a raised louver pad
#   (four horizontal vent slots) flanked by two corner mounting screws. The centre
#   carries a short flip-toggle dolly lever: a single central lever pivots in a
#   raised boss on the faceplate. A recessed chamfered nameplate sits at the
#   bottom.
#
# Coordinate convention (meters):
#   X = plate width (horizontal), Y = plate height (+Y up),
#   Z = depth out of the wall toward the viewer (+Z faces the room).
#
# Articulation:
#   ONE revolute joint = the flip-toggle lever pivoting about the horizontal
#   boss axis (X). q=0 is the lever-up pose; positive q flips the dolly lever
#   down and slightly forward.

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

# Central raised boss (fixed faceplate feature holding the flip-toggle pivot).
BOSS_W = 0.034
BOSS_H = 0.046
BOSS_CY = 0.000
BOSS_PROUD = 0.0085                  # stands proud of the field/front face

# Pivot axis: horizontal X line in the visible raised boss.
PIVOT_Y = 0.004
TOGGLE_HUB_R = 0.0042
TOGGLE_HUB_LEN = 0.020               # short central moving hub, not a wide roller
BUSH_R = 0.0049
BUSH_EAR_LEN = 0.0055
BUSH_GAP = 0.0                     # boss ears visibly contact the moving hub
PIVOT_Z = PLATE_T + BOSS_PROUD + TOGGLE_HUB_R + 0.0003

# Flip-toggle dolly lever (moving child, short and central).
LEVER_LEN = 0.027
LEVER_STEM_R = 0.0024
LEVER_STEM_Z = 0.0030                # stem stands just forward of the pivot hub
LEVER_PAD_W = 0.014
LEVER_PAD_H = 0.010
LEVER_PAD_T = 0.006

# Bottom recessed nameplate (chamfered rectangle).
NAME_W = 0.044
NAME_H = 0.024
NAME_CY = -PLATE_H / 2.0 + 0.028
NAME_CHAMFER = 0.006
NAME_DEPTH = 0.0016

# Toggle throw: q=0 up; positive swing flips the lever down/forward.
ROCK_LOWER = math.radians(-10.0)
ROCK_UPPER = math.radians(9.0)


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
    model.material("keeper_steel", color=(0.56, 0.58, 0.61))
    model.material("rivet_steel", color=(0.44, 0.46, 0.49))
    model.material("screw_zinc", color=(0.78, 0.80, 0.82))
    model.material("boss_steel", color=(0.56, 0.58, 0.61))
    model.material("bushing_steel", color=(0.46, 0.48, 0.51))
    model.material("toggle_dark", color=(0.12, 0.13, 0.14))


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


def _build_toggle_boss() -> cq.Workplane:
    field_front = PLATE_T - FIELD_DEPTH
    boss_front = PLATE_T + BOSS_PROUD
    boss = (
        cq.Workplane("XY")
        .workplane(offset=field_front)
        .center(0.0, BOSS_CY)
        .box(BOSS_W, BOSS_H, FIELD_DEPTH + BOSS_PROUD, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
    )

    # A shallow front relief around the hub makes the child lever visibly sit in
    # the boss instead of being swallowed by a solid block.
    hub_relief = (
        cq.Workplane("XY")
        .workplane(offset=boss_front - 0.0010)
        .center(0.0, PIVOT_Y)
        .box(TOGGLE_HUB_LEN + 2.0 * BUSH_GAP, 2.0 * BUSH_R + 0.002, 0.0030, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.002)
    )
    boss = boss.cut(hub_relief)

    # Two visible side ears of the raised boss carry the horizontal pivot.  They
    # are not a roller-bail: the moving child is a single central dolly lever.
    for sx in (-1, 1):
        cx = sx * (TOGGLE_HUB_LEN / 2.0 + BUSH_GAP + BUSH_EAR_LEN / 2.0)
        ear = (
            cq.Workplane("YZ")
            .workplane(offset=cx - BUSH_EAR_LEN / 2.0)
            .center(PIVOT_Y, PIVOT_Z)
            .circle(BUSH_R)
            .extrude(BUSH_EAR_LEN)
        )
        bridge = (
            cq.Workplane("XY")
            .workplane(offset=boss_front - 0.0008)
            .center(cx, PIVOT_Y)
            .box(BUSH_EAR_LEN, 2.0 * BUSH_R, PIVOT_Z - boss_front + BUSH_R + 0.0010, centered=(True, True, False))
            .edges("|Z")
            .fillet(0.0015)
        )
        boss = boss.union(bridge).union(ear)

    # Two small rivets on the boss face, matching the reference's fixed plate
    # detailing while remaining inline with the non-moving faceplate layer.
    rivet_z = boss_front - 0.0004
    for sy in (-1, 1):
        rivet = (
            cq.Workplane("XY")
            .workplane(offset=rivet_z)
            .center(0.0, BOSS_CY + sy * 0.014)
            .circle(0.0020)
            .extrude(0.0008)
        )
        boss = boss.union(rivet)
    return boss


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
# Flip-toggle dolly lever (moving child, authored in the pivot frame)
# ---------------------------------------------------------------------------
def _build_toggle_lever() -> cq.Workplane:
    # Local frame: origin on the pivot axis. Local +Y is lever-up at q=0 and
    # local +Z points outward toward the user. The result is deliberately short
    # and central: one dolly lever, not the former wide roller-bail assembly.
    hub = (
        cq.Workplane("YZ")
        .workplane(offset=-TOGGLE_HUB_LEN / 2.0)
        .center(0.0, 0.0)
        .circle(TOGGLE_HUB_R)
        .extrude(TOGGLE_HUB_LEN)
    )
    stem = (
        cq.Workplane("XZ")
        .workplane(offset=0.0)
        .center(0.0, LEVER_STEM_Z)
        .circle(LEVER_STEM_R)
        .extrude(LEVER_LEN)
    )
    pad = (
        cq.Workplane("XY")
        .workplane(offset=LEVER_STEM_Z - LEVER_PAD_T / 2.0)
        .center(0.0, LEVER_LEN + LEVER_PAD_H / 2.0 - 0.0015)
        .box(LEVER_PAD_W, LEVER_PAD_H, LEVER_PAD_T, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.003)
    )
    neck = (
        cq.Workplane("XY")
        .workplane(offset=LEVER_STEM_Z - LEVER_PAD_T / 2.0)
        .center(0.0, LEVER_LEN - 0.0020)
        .box(LEVER_STEM_R * 2.0, 0.006, LEVER_PAD_T, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0015)
    )
    # A small solid web is hidden inside the rounded stem/pad envelope and makes
    # the moving lever one connected manufactured piece for exact connectivity.
    web = (
        cq.Workplane("XY")
        .workplane(offset=-0.0015)
        .center(0.0, LEVER_LEN / 2.0)
        .box(LEVER_STEM_R * 1.6, LEVER_LEN + 0.004, 0.0045, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0010)
    )
    return hub.union(stem).union(web).union(neck).union(pad)


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_toggle_power_switch")
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
        mesh_from_cadquery(_build_toggle_boss(), "raised_boss"),
        origin=Origin(),
        material="boss_steel",
        name="raised_boss",
    )
    faceplate.visual(
        mesh_from_cadquery(_build_screws(), "mount_screws"),
        origin=Origin(),
        material="screw_zinc",
        name="mount_screws",
    )

    toggle = model.part("toggle")
    toggle.visual(
        mesh_from_cadquery(_build_toggle_lever(), "toggle_lever"),
        origin=Origin(),
        material="toggle_dark",
        name="toggle_lever",
    )

    model.articulation(
        "plate_to_toggle",
        ArticulationType.REVOLUTE,
        parent=faceplate,
        child=toggle,
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
    toggle = object_model.get_part("toggle")
    hinge = object_model.get_articulation("plate_to_toggle")

    ctx.check(
        "faceplate is the single root",
        faceplate in object_model.root_parts() and len(object_model.root_parts()) == 1,
        details=f"roots={[p.name for p in object_model.root_parts()]}",
    )
    ctx.check(
        "toggle is hinge child",
        hinge.child == "toggle" and hinge.parent == "faceplate",
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
    ctx.check(
        "toggle throw is -10 to 9 degrees",
        abs(math.degrees(hinge.motion_limits.lower) + 10.0) < 1e-6
        and abs(math.degrees(hinge.motion_limits.upper) - 9.0) < 1e-6,
        details=(
            f"lower={math.degrees(hinge.motion_limits.lower)} "
            f"upper={math.degrees(hinge.motion_limits.upper)}"
        ),
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

    # The raised boss remains the visible fixed mount on the same plate face.
    boss_aabb = ctx.part_element_world_aabb(faceplate, elem="raised_boss")
    ctx.check("raised boss aabb present", boss_aabb is not None, details=str(boss_aabb))
    if boss_aabb is not None:
        bmin, bmax = boss_aabb
        bw = float(bmax[0] - bmin[0])
        bh = float(bmax[1] - bmin[1])
        ctx.check("boss is central and raised", float(bmax[2]) > PLATE_T + 0.008, details=f"z={float(bmax[2])}")
        ctx.check("boss is compact switch mount", bw < 0.050 and bh < 0.060, details=f"w={bw}, h={bh}")

    # The new child is one short central dolly lever, not the former wide roller bail.
    lever_aabb = ctx.part_element_world_aabb(toggle, elem="toggle_lever")
    ctx.check("toggle lever aabb present", lever_aabb is not None, details=str(lever_aabb))
    if lever_aabb is not None:
        lmin, lmax = lever_aabb
        lw = float(lmax[0] - lmin[0])
        lh = float(lmax[1] - lmin[1])
        ctx.check(
            "toggle is a short central lever",
            lw < 0.026 and lh < 0.070,
            details=f"lever_width={lw}, lever_height={lh}",
        )
        ctx.check(
            "toggle stands proud of the plate",
            float(lmax[2]) > PLATE_T + BOSS_PROUD + 0.002,
            details=f"lever_front_z={float(lmax[2])}",
        )

    # The toggle hub is intentionally captured in the raised boss on the plate.
    ctx.allow_overlap(
        faceplate,
        toggle,
        elem_a="raised_boss",
        elem_b="toggle_lever",
        reason="The toggle pivot hub is intentionally captured between the visible ears of the raised boss.",
    )

    # The lever sits in front of the raised boss and overlaps its footprint.
    ctx.expect_overlap(
        toggle,
        faceplate,
        axes="xy",
        elem_a="toggle_lever",
        elem_b="raised_boss",
        min_overlap=0.010,
        name="toggle overlaps raised boss footprint",
    )
    ctx.expect_within(
        toggle,
        faceplate,
        axes="x",
        inner_elem="toggle_lever",
        outer_elem="raised_boss",
        margin=0.001,
        name="toggle hub is centered between boss ears",
    )

    # Mechanism: throwing the switch swings the single lever about the X axis.
    # Track the lever at the two extreme poses -- a no-op joint cannot fake it.
    def _lever():
        box = ctx.part_element_world_aabb(toggle, elem="toggle_lever")
        if box is None:
            return None
        mn, mx = box
        return (float(mx[1]), float(mx[2]))  # (top_y, front_z)

    rest = _lever()
    ctx.check("lever present", rest is not None, details=str(rest))

    with ctx.pose({hinge: ROCK_LOWER}):
        lower_y, lower_z = _lever()
    with ctx.pose({hinge: ROCK_UPPER}):
        upper_y, upper_z = _lever()
    with ctx.pose({hinge: 0.0}):
        _up_y, up_z = _lever()

    swung = abs(lower_y - upper_y) > 0.0002 or abs(lower_z - upper_z) > 0.0005
    ctx.check(
        "throwing the toggle actually swings the lever",
        swung,
        details=(
            f"top_y lower={lower_y} upper={upper_y}; "
            f"front_z lower={lower_z} upper={upper_z}"
        ),
    )
    # Positive throw moves the lever forward -- proves the toggle pivots the
    # right way about the horizontal axis even with a compact 19 degree throw.
    ctx.check(
        "positive throw moves the toggle forward",
        upper_z > up_z,
        details=f"front_z up={up_z} upper={upper_z}",
    )

    return ctx.report()


object_model = build_object_model()
