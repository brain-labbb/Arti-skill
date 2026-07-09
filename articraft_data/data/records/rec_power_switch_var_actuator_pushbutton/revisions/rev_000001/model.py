from __future__ import annotations

# Wall-mounted momentary push-button power switch, forked from the
# picture/Equipment/Power switch/001.png.
#
# Identity (from the reference):
#   A vertical rounded-rectangular faceplate with a raised perimeter border and a
#   recessed, chamfered-corner inner field. Near the top sits a raised louver pad
#   (four horizontal vent slots) flanked by two corner mounting screws. The centre
#   carries one large round momentary push-button seated in a raised circular
#   bezel bore. A recessed chamfered nameplate sits at the bottom.
#
# Coordinate convention (meters):
#   X = plate width (horizontal), Y = plate height (+Y up),
#   Z = depth out of the wall toward the viewer (+Z faces the room).
#
# Articulation:
#   ONE prismatic joint = the button cap sliding inward along -Z into the wall
#   plate. q=0 is unpressed/proud; positive q depresses the cap.

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

# Central momentary button and fixed bezel bore.
BUTTON_CY = 0.000
BEZEL_OUTER_R = 0.0235
BEZEL_BORE_R = 0.0182
BEZEL_PROUD = 0.0048
BEZEL_FILLET = 0.0014
BUTTON_R = 0.0162
BUTTON_FACE_H = 0.0072
BUTTON_SKIRT_R = 0.0128
BUTTON_SKIRT_INSERT = 0.0042
BUTTON_TRAVEL = 0.0040

# Bottom recessed nameplate (chamfered rectangle).
NAME_W = 0.044
NAME_H = 0.024
NAME_CY = -PLATE_H / 2.0 + 0.028
NAME_CHAMFER = 0.006
NAME_DEPTH = 0.0016

# Button travel: q=0 unpressed; positive slides along the joint axis (-Z).
PRESS_LOWER = 0.0
PRESS_UPPER = BUTTON_TRAVEL


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
    model.material("bezel_metal", color=(0.50, 0.52, 0.54))
    model.material("bore_shadow", color=(0.12, 0.13, 0.14))
    model.material("button_dark", color=(0.08, 0.085, 0.09))
    model.material("button_highlight", color=(0.18, 0.19, 0.20))


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

    # True through-bore for the momentary button; the cap skirt slides in this
    # clear opening rather than into a solid proxy.
    button_bore = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .center(0.0, BUTTON_CY)
        .circle(BEZEL_BORE_R)
        .extrude(PLATE_T + BEZEL_PROUD + 0.004)
    )
    plate = plate.cut(button_bore)
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


def _build_button_bezel() -> cq.Workplane:
    """Raised annular bezel with an open bore around the moving button."""
    field_front = PLATE_T - FIELD_DEPTH
    bezel = (
        cq.Workplane("XY")
        .workplane(offset=field_front)
        .center(0.0, BUTTON_CY)
        .circle(BEZEL_OUTER_R)
        .circle(BEZEL_BORE_R)
        .extrude(FIELD_DEPTH + BEZEL_PROUD)
    )
    return bezel


def _build_bore_shadow() -> cq.Workplane:
    """Thin dark sleeve just inside the bezel bore, leaving the center open."""
    field_front = PLATE_T - FIELD_DEPTH
    sleeve_outer = BEZEL_BORE_R
    sleeve_inner = BUTTON_SKIRT_R
    return (
        cq.Workplane("XY")
        .workplane(offset=field_front - 0.0008)
        .center(0.0, BUTTON_CY)
        .circle(sleeve_outer)
        .circle(sleeve_inner)
        .extrude(BEZEL_PROUD + 0.0012)
    )


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
# Momentary push-button (moving child, authored in the bore/front-face frame)
# ---------------------------------------------------------------------------
def _build_button_cap() -> cq.Workplane:
    # Local frame: origin is on the visible front seating plane of the bezel bore.
    # The cap has a rounded proud face and a smaller rear skirt that stays inside
    # the through-bore through the whole press stroke.
    skirt = (
        cq.Workplane("XY")
        .workplane(offset=-BUTTON_SKIRT_INSERT)
        .circle(BUTTON_SKIRT_R)
        .extrude(BUTTON_SKIRT_INSERT + 0.0012)
    )
    face = (
        cq.Workplane("XY")
        .workplane(offset=0.0004)
        .circle(BUTTON_R)
        .extrude(BUTTON_FACE_H)
    )
    cap = skirt.union(face)

    # Shallow circular inset on the touch surface, giving the large cap the
    # molded, slightly dished look of a real momentary push button.
    groove = (
        cq.Workplane("XY")
        .workplane(offset=BUTTON_FACE_H + 0.0001)
        .circle(BUTTON_R * 0.68)
        .extrude(0.0010)
    )
    cap = cap.cut(groove)

    center_highlight = (
        cq.Workplane("XY")
        .workplane(offset=BUTTON_FACE_H - 0.00035)
        .circle(BUTTON_R * 0.22)
        .extrude(0.0005)
    )
    cap = cap.union(center_highlight)
    return cap


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_momentary_power_switch")
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
        mesh_from_cadquery(_build_button_bezel(), "button_bezel"),
        origin=Origin(),
        material="bezel_metal",
        name="button_bezel",
    )
    faceplate.visual(
        mesh_from_cadquery(_build_bore_shadow(), "bore_shadow"),
        origin=Origin(),
        material="bore_shadow",
        name="bore_shadow",
    )
    faceplate.visual(
        mesh_from_cadquery(_build_screws(), "mount_screws"),
        origin=Origin(),
        material="screw_zinc",
        name="mount_screws",
    )

    button = model.part("button_cap")
    button.visual(
        mesh_from_cadquery(_build_button_cap(), "button_cap"),
        origin=Origin(),
        material="button_dark",
        name="button_cap",
    )

    model.articulation(
        "plate_to_button",
        ArticulationType.PRISMATIC,
        parent=faceplate,
        child=button,
        origin=Origin(xyz=(0.0, BUTTON_CY, PLATE_T + BEZEL_PROUD)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=6.0, velocity=0.25, lower=PRESS_LOWER, upper=PRESS_UPPER
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    faceplate = object_model.get_part("faceplate")
    button = object_model.get_part("button_cap")
    slide = object_model.get_articulation("plate_to_button")

    ctx.check(
        "faceplate is the single root",
        faceplate in object_model.root_parts() and len(object_model.root_parts()) == 1,
        details=f"roots={[p.name for p in object_model.root_parts()]}",
    )
    ctx.check(
        "button is prismatic child",
        slide.child == "button_cap" and slide.parent == "faceplate",
        details=f"parent={slide.parent}, child={slide.child}",
    )
    ctx.check(
        "joint type is prismatic",
        str(slide.articulation_type).lower().endswith("prismatic"),
        details=f"type={slide.articulation_type}",
    )
    ax = tuple(float(a) for a in slide.axis)
    ctx.check(
        "button presses along negative Z",
        abs(ax[0]) < 1e-6 and abs(ax[1]) < 1e-6 and ax[2] < -0.99,
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

    # The fixed annular bezel is visible on the plate and the moving cap is a
    # single large round button seated inside its bore.
    bezel_aabb = ctx.part_element_world_aabb(faceplate, elem="button_bezel")
    cap_aabb = ctx.part_element_world_aabb(button, elem="button_cap")
    ctx.check("button bezel aabb present", bezel_aabb is not None, details=str(bezel_aabb))
    ctx.check("button cap aabb present", cap_aabb is not None, details=str(cap_aabb))
    if bezel_aabb is not None:
        bmin, bmax = bezel_aabb
        bw = float(bmax[0] - bmin[0])
        bh = float(bmax[1] - bmin[1])
        ctx.check(
            "bezel is a round bore surround",
            abs(bw - bh) < 0.003 and 0.040 < bw < 0.052,
            details=f"bezel_w={bw}, bezel_h={bh}",
        )
        ctx.check(
            "bezel stands proud of plate",
            float(bmax[2]) > PLATE_T + 0.003,
            details=f"bezel_front_z={float(bmax[2])}",
        )
    if cap_aabb is not None:
        cmin, cmax = cap_aabb
        cw = float(cmax[0] - cmin[0])
        ch = float(cmax[1] - cmin[1])
        ctx.check(
            "cap is one large round button",
            abs(cw - ch) < 0.003 and 0.028 < cw < 0.036,
            details=f"cap_w={cw}, cap_h={ch}",
        )
        ctx.check(
            "unpressed cap stands proud",
            float(cmax[2]) > PLATE_T + BEZEL_PROUD + 0.005,
            details=f"cap_front_z={float(cmax[2])}",
        )

    ctx.expect_overlap(
        button,
        faceplate,
        axes="xy",
        elem_a="button_cap",
        elem_b="faceplate_shell",
        min_overlap=0.010,
        name="button overlaps plate footprint",
    )
    ctx.expect_within(
        button,
        faceplate,
        axes="xy",
        inner_elem="button_cap",
        outer_elem="button_bezel",
        margin=0.002,
        name="button cap sits within bezel bore footprint",
    )
    ctx.expect_contact(
        button,
        faceplate,
        elem_a="button_cap",
        elem_b="bore_shadow",
        contact_tol=0.00005,
        name="button skirt is seated in visible bore sleeve",
    )

    # Mechanism: pressing the cap slides it inward along -Z. Track the cap front
    # at the two extreme poses -- a no-op or wrong-axis joint cannot fake it.
    def _cap_front():
        box = ctx.part_element_world_aabb(button, elem="button_cap")
        if box is None:
            return None
        mn, mx = box
        return float(mx[2])

    rest_z = _cap_front()
    ctx.check("button cap present", rest_z is not None, details=str(rest_z))

    with ctx.pose({slide: PRESS_UPPER}):
        pressed_z = _cap_front()
        ctx.expect_within(
            button,
            faceplate,
            axes="xy",
            inner_elem="button_cap",
            outer_elem="button_bezel",
            margin=0.002,
            name="pressed button stays centered in bezel",
        )
    with ctx.pose({slide: 0.0}):
        unpressed_z = _cap_front()

    ctx.check(
        "positive press moves cap inward",
        rest_z is not None
        and pressed_z is not None
        and unpressed_z is not None
        and pressed_z < unpressed_z - 0.003,
        details=f"front_z unpressed={unpressed_z}, pressed={pressed_z}",
    )
    ctx.check(
        "button travel matches short momentary stroke",
        abs((unpressed_z or 0.0) - (pressed_z or 0.0) - BUTTON_TRAVEL) < 0.001,
        details=f"travel={(unpressed_z or 0.0) - (pressed_z or 0.0)}",
    )

    return ctx.report()


object_model = build_object_model()
