from __future__ import annotations

# Rail-mounted control panel module (variant 2), modeled from the reference image.
#
# The reference shows a light-gray rounded plastic enclosure clamped onto two
# horizontal dark-metal rails. The face carries a recessed dark LCD window, three
# small indicator LEDs across the top, a column of horizontal ventilation/data
# slots on the left edge, and one knurled rotary selector knob at the bottom.
#
# Real mechanism: the bottom control has been replaced by a rotary selector knob.
# It is the primary articulated part and turns about the face normal (REVOLUTE).
# The two rails are carried by the rear clamp of the housing (FIXED), so the whole
# assembly has a single connected root.
#
# Frame convention (housing part frame, meters):
#   +X = right (along the rails)
#   +Y = out toward the viewer (the visible face normal)
#   +Z = up
# The housing front face is at +Y; the selector shaft spins around +Y.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    KnobGeometry,
    KnobGrip,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# --- Real-world dimensions (meters) -----------------------------------------

HOUSING_W = 0.088  # left-right extent of the enclosure
HOUSING_H = 0.104  # vertical extent of the enclosure
HOUSING_D = 0.030  # depth (front face thickness direction, along Y)
HOUSING_CORNER = 0.010

FACE_Y = HOUSING_D / 2.0  # front face plane (+Y)
BACK_Y = -HOUSING_D / 2.0

# Display window
DISP_W = 0.040
DISP_H = 0.030
DISP_CX = 0.006  # display centered slightly right of housing center
DISP_CZ = 0.004
DISP_RECESS = 0.004

# Indicator LEDs (three across the top)
LED_R = 0.0035
LED_Z = HOUSING_H / 2.0 - 0.014
LED_DX = 0.009
LED_CX = 0.006

# Vent / data slots (left column)
SLOT_W = 0.014
SLOT_H = 0.0022
SLOT_X = -HOUSING_W / 2.0 + 0.014
SLOT_Z0 = -0.014
SLOT_PITCH = 0.006
SLOT_COUNT = 5

# Rotary selector knob (single bottom control replacing the two push buttons)
KNOB_DIAMETER = 0.026
KNOB_HEIGHT = 0.013
KNOB_X = 0.006
KNOB_Z = -HOUSING_H / 2.0 + 0.020
KNOB_STEM_R = 0.0038
KNOB_STEM_L = 0.010
KNOB_TURN = math.radians(135.0)
POINTER_W = 0.0022
POINTER_L = 0.010
POINTER_T = 0.0007

# Rails
RAIL_R = 0.0055
RAIL_LEN = 0.34
RAIL_DZ = 0.020  # vertical spacing between the two rails

# Rear clamp block that grips the rails. It is deep enough to bridge from the
# inner front wall of the housing, through the open rear cavity, and out the
# back so its grooves can capture the rails.
CLAMP_W = 0.034
CLAMP_H = 0.058
CLAMP_D = 0.026
CLAMP_FRONT_Y = 0.007  # clamp front face embeds 0.002 into the front wall solid
CLAMP_CENTER_Y = CLAMP_FRONT_Y - CLAMP_D / 2.0
CLAMP_BACK_Y = CLAMP_CENTER_Y - CLAMP_D / 2.0  # groove plane (rear face)


# --- Materials ---------------------------------------------------------------

MAT_HOUSING = Material(name="panel_plastic", rgba=(0.74, 0.74, 0.71, 1.0))
MAT_DISPLAY = Material(name="lcd_glass", rgba=(0.16, 0.20, 0.18, 1.0))
MAT_LED = Material(name="indicator_led", rgba=(0.30, 0.32, 0.33, 1.0))
MAT_KNOB = Material(name="selector_knob", rgba=(0.18, 0.18, 0.17, 1.0))
MAT_POINTER = Material(name="selector_pointer", rgba=(0.86, 0.84, 0.78, 1.0))
MAT_RAIL = Material(name="steel_rail", rgba=(0.16, 0.16, 0.17, 1.0))
MAT_CLAMP = Material(name="rear_clamp", rgba=(0.60, 0.60, 0.58, 1.0))


# --- Housing geometry --------------------------------------------------------


# Axis mapping used throughout (CadQuery local frame == part-local frame):
#   X = width (left-right, along rails)
#   Y = depth (front face normal points +Y)
#   Z = height (up)
# cq box(dx, dy, dz) extents map directly to (X, Y, Z).
# A Z-axis cq cylinder rotated +90 deg about X has its axis along Y (front-back).
# A Z-axis cq cylinder rotated +90 deg about Y has its axis along X (left-right).


def _y_cyl(radius: float, length: float) -> cq.Workplane:
    """Cylinder whose axis runs along Y (front-back), centered at the origin."""
    return cq.Workplane("XY").cylinder(length, radius).rotate((0, 0, 0), (1, 0, 0), 90.0)


def _x_cyl(radius: float, length: float) -> cq.Workplane:
    """Cylinder whose axis runs along X (left-right), centered at the origin."""
    return cq.Workplane("XY").cylinder(length, radius).rotate((0, 0, 0), (0, 1, 0), 90.0)


def _build_housing_shape() -> cq.Workplane:
    """Rounded rectangular enclosure, hollowed at the back, with face features cut in."""
    # Solid rounded body: width X, depth Y, height Z. Vertical corner edges run
    # parallel to Z, so fillet the |Z edges to round the enclosure outline.
    body = (
        cq.Workplane("XY")
        .box(HOUSING_W, HOUSING_D, HOUSING_H)
        .edges("|Z")
        .fillet(HOUSING_CORNER)
    )

    # Hollow it from the back so it reads as a real shelled enclosure. The
    # cavity is shifted toward -Y so it opens out the back face only.
    back_cavity = (
        cq.Workplane("XY")
        .box(HOUSING_W - 0.010, HOUSING_D - 0.008, HOUSING_H - 0.010)
        .edges("|Z")
        .fillet(HOUSING_CORNER - 0.003)
        .translate((0.0, -0.006, 0.0))
    )
    body = body.cut(back_cavity.val())

    # Recessed display pocket cut into the front (+Y) face.
    disp_pocket = (
        cq.Workplane("XY")
        .box(DISP_W, DISP_RECESS * 2.0, DISP_H)
        .translate((DISP_CX, FACE_Y, DISP_CZ))
    )
    body = body.cut(disp_pocket.val())

    # Ventilation / data slots on the left of the face (long along X, thin along Z).
    for i in range(SLOT_COUNT):
        z = SLOT_Z0 + i * SLOT_PITCH
        slot = (
            cq.Workplane("XY")
            .box(SLOT_W, 0.008, SLOT_H)
            .translate((SLOT_X, FACE_Y, z))
        )
        body = body.cut(slot.val())

    # Small wells for the indicator LEDs so they read as inset dots.
    for k in (-1, 0, 1):
        led_well = _y_cyl(LED_R + 0.0008, 0.006).translate(
            (LED_CX + k * LED_DX, FACE_Y, LED_Z)
        )
        body = body.cut(led_well.val())

    # Central bore for the rotary selector shaft (axis along Y, through the face).
    shaft_bore = _y_cyl(KNOB_STEM_R + 0.0008, 0.026).translate(
        (KNOB_X, FACE_Y - 0.006, KNOB_Z)
    )
    body = body.cut(shaft_bore.val())

    return body


def _build_display_glass() -> cq.Workplane:
    """Dark LCD glass that sits inside the recessed pocket."""
    return (
        cq.Workplane("XY")
        .box(DISP_W - 0.003, 0.0035, DISP_H - 0.003)
        .edges("|Z")
        .fillet(0.0015)
        .translate((DISP_CX, FACE_Y - DISP_RECESS + 0.0010, DISP_CZ))
    )


def _build_display_bezel() -> cq.Workplane:
    """Thin raised frame around the display opening."""
    outer = (
        cq.Workplane("XY")
        .box(DISP_W + 0.006, 0.004, DISP_H + 0.006)
        .edges("|Z")
        .fillet(0.0012)
    )
    inner = cq.Workplane("XY").box(DISP_W - 0.001, 0.006, DISP_H - 0.001)
    frame = outer.cut(inner.val())
    return frame.translate((DISP_CX, FACE_Y - 0.0005, DISP_CZ))


def _build_led(k: int) -> cq.Workplane:
    """A single indicator LED dome seated in its well."""
    return (
        cq.Workplane("XY")
        .sphere(LED_R)
        .translate((LED_CX + k * LED_DX, FACE_Y - 0.0015, LED_Z))
    )


def _build_selector_knob_geometry() -> KnobGeometry:
    """Knurled selector cap aligned to local Z; mounted face remains at z=0."""
    return KnobGeometry(
        KNOB_DIAMETER,
        KNOB_HEIGHT,
        body_style="cylindrical",
        edge_radius=0.0008,
        grip=KnobGrip(
            style="knurled",
            count=36,
            depth=0.0008,
            helix_angle_deg=22.0,
        ),
        center=False,
    )


def _build_selector_stem() -> cq.Workplane:
    """Hidden shaft/stem that passes through the front bore and retains the knob."""
    return _y_cyl(KNOB_STEM_R, KNOB_STEM_L).translate((0.0, -KNOB_STEM_L / 2.0, 0.0))


def _build_pointer_mark() -> cq.Workplane:
    """Contrasting raised radial pointer mark on the front face of the knob."""
    pointer = (
        cq.Workplane("XY")
        .box(POINTER_W, POINTER_T, POINTER_L)
        .edges("|Y")
        .fillet(0.00035)
        .translate((0.0, KNOB_HEIGHT + POINTER_T / 2.0 - 0.00015, POINTER_L * 0.24))
    )
    return pointer


def _build_clamp_shape() -> cq.Workplane:
    """Rear clamp block with two grooves that capture the rails."""
    block = (
        cq.Workplane("XY")
        .box(CLAMP_W, CLAMP_D, CLAMP_H)
        .edges("|Y")
        .fillet(0.004)
    )
    # Two horizontal grooves (axis along X) for the rails, cut on the back (-Y).
    # The groove radius equals the rail radius so a rail seated at the groove
    # plane makes firm contact with the half-pipe pocket wall.
    for z in (RAIL_DZ / 2.0, -RAIL_DZ / 2.0):
        groove = _x_cyl(RAIL_R, CLAMP_W + 0.01).translate((0.0, -CLAMP_D / 2.0, z))
        block = block.cut(groove.val())
    return block


def _build_rail_shape() -> cq.Workplane:
    """One long horizontal rail bar (axis along X)."""
    return _x_cyl(RAIL_R, RAIL_LEN)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rail_mounted_control_panel")

    # --- Housing (root) -----------------------------------------------------
    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_build_housing_shape(), "housing_shell"),
        material=MAT_HOUSING,
        name="housing_shell",
    )
    housing.visual(
        mesh_from_cadquery(_build_display_bezel(), "display_bezel"),
        material=MAT_HOUSING,
        name="display_bezel",
    )
    housing.visual(
        mesh_from_cadquery(_build_display_glass(), "display_glass"),
        material=MAT_DISPLAY,
        name="display_glass",
    )
    for idx, k in enumerate((-1, 0, 1)):
        housing.visual(
            mesh_from_cadquery(_build_led(k), f"led_{idx}"),
            material=MAT_LED,
            name=f"led_{idx}",
        )

    # Rear clamp block that grips the rails. It bridges the housing front wall,
    # the rear cavity, and protrudes out the back, so it is solidly connected to
    # the housing shell and carries the rail grooves.
    housing.visual(
        mesh_from_cadquery(_build_clamp_shape(), "rear_clamp"),
        material=MAT_CLAMP,
        name="rear_clamp",
        origin=Origin(xyz=(0.0, CLAMP_CENTER_Y, 0.0)),
    )

    # --- Rails (each carried by the clamp grooves, FIXED) -------------------
    # The two rails are physically independent bars, so each is its own part
    # fixed into its groove on the rear clamp.
    # Seat the rail centerline at the groove plane, nudged 0.4 mm into the
    # half-pipe pocket so it makes firm contact with the clamp (a real seat).
    rail_center_y = CLAMP_BACK_Y + 0.0004
    for idx, z in enumerate((RAIL_DZ / 2.0, -RAIL_DZ / 2.0)):
        rname = "rail_top" if z > 0 else "rail_bottom"
        rail = model.part(rname)
        rail.visual(
            mesh_from_cadquery(_build_rail_shape(), f"{rname}_bar"),
            material=MAT_RAIL,
            name=f"{rname}_bar",
        )
        model.articulation(
            f"clamp_to_{rname}",
            ArticulationType.FIXED,
            parent=housing,
            child=rail,
            origin=Origin(xyz=(0.0, rail_center_y, z)),
        )

    # --- Rotary selector (REVOLUTE, the primary mechanism) ------------------
    selector = model.part("selector_knob")
    selector.visual(
        mesh_from_geometry(_build_selector_knob_geometry(), "selector_knob_shell"),
        # KnobGeometry is built along local +Z; rotate it so its axis becomes
        # local +Y and its mounting face lands on the panel face joint frame.
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=MAT_KNOB,
        name="selector_knob_shell",
    )
    selector.visual(
        mesh_from_cadquery(_build_selector_stem(), "selector_stem"),
        material=MAT_KNOB,
        name="selector_stem",
    )
    selector.visual(
        mesh_from_cadquery(_build_pointer_mark(), "pointer_mark"),
        material=MAT_POINTER,
        name="pointer_mark",
    )
    # Joint frame sits exactly on the visible face plane at the knob center.
    # Positive q follows the +Y face-normal rotation convention.
    model.articulation(
        "turn_selector",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=selector,
        origin=Origin(xyz=(KNOB_X, FACE_Y, KNOB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.35, velocity=4.0, lower=-KNOB_TURN, upper=KNOB_TURN
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    rail_top = object_model.get_part("rail_top")
    rail_bottom = object_model.get_part("rail_bottom")
    selector = object_model.get_part("selector_knob")
    turn_selector = object_model.get_articulation("turn_selector")
    clamp_to_rail_top = object_model.get_articulation("clamp_to_rail_top")
    clamp_to_rail_bottom = object_model.get_articulation("clamp_to_rail_bottom")

    # --- Joint type / axis claims ------------------------------------------
    ctx.check(
        "selector knob is revolute",
        str(turn_selector.articulation_type).endswith("REVOLUTE"),
        details=f"type={turn_selector.articulation_type}",
    )
    ctx.check(
        "selector axis is the face normal (+Y)",
        tuple(round(c, 3) for c in turn_selector.axis) == (0.0, 1.0, 0.0),
        details=f"axis={turn_selector.axis}",
    )
    ctx.check(
        "rails are fixed to the housing",
        str(clamp_to_rail_top.articulation_type).endswith("FIXED")
        and str(clamp_to_rail_bottom.articulation_type).endswith("FIXED"),
        details=f"top={clamp_to_rail_top.articulation_type}, "
        f"bottom={clamp_to_rail_bottom.articulation_type}",
    )
    ctx.check(
        "selector has realistic limited rotary travel",
        turn_selector.motion_limits is not None
        and turn_selector.motion_limits.lower is not None
        and turn_selector.motion_limits.upper is not None
        and -math.pi <= turn_selector.motion_limits.lower < -1.5
        and 1.5 < turn_selector.motion_limits.upper <= math.pi,
        details=(
            f"lower={getattr(turn_selector.motion_limits, 'lower', None)}, "
            f"upper={getattr(turn_selector.motion_limits, 'upper', None)}"
        ),
    )

    # --- Hero parts present and placed -------------------------------------
    # Display glass sits in the recessed pocket on the front (+Y) face.
    glass = housing.get_visual("display_glass")
    glass_aabb = ctx.part_element_world_aabb(housing, elem=glass)
    ctx.check(
        "display glass is present on the front face",
        glass_aabb is not None and glass_aabb[1][1] <= FACE_Y + 0.001,
        details=f"glass_aabb={glass_aabb}",
    )

    # Each rail spans well beyond the housing width (long horizontal bars).
    top_aabb = ctx.part_world_aabb(rail_top)
    bottom_aabb = ctx.part_world_aabb(rail_bottom)
    ctx.check(
        "rails extend well beyond the housing width",
        top_aabb is not None
        and bottom_aabb is not None
        and (top_aabb[1][0] - top_aabb[0][0]) > HOUSING_W * 2.0
        and (bottom_aabb[1][0] - bottom_aabb[0][0]) > HOUSING_W * 2.0,
        details=f"top={top_aabb}, bottom={bottom_aabb}",
    )
    # Rails sit behind the housing front face.
    ctx.check(
        "rails sit behind the housing face",
        top_aabb is not None
        and bottom_aabb is not None
        and top_aabb[1][1] < FACE_Y
        and bottom_aabb[1][1] < FACE_Y,
        details=f"top={top_aabb}, bottom={bottom_aabb}",
    )
    # The two rails are vertically separated.
    ctx.expect_origin_distance(
        rail_top,
        rail_bottom,
        axes="z",
        min_dist=0.012,
        name="two rails are separated vertically",
    )

    # The single rotary selector replaces the two old push buttons and stands
    # proud of the front face at the same bottom-center control location.
    selector_aabb = ctx.part_world_aabb(selector)
    ctx.check(
        "selector knob stands proud of the face",
        selector_aabb is not None and selector_aabb[1][1] > FACE_Y + KNOB_HEIGHT * 0.7,
        details=f"selector_aabb={selector_aabb}",
    )
    ctx.check(
        "old push-button parts were replaced",
        all(part.name not in ("button_left", "button_right") for part in object_model.parts),
        details=f"parts={[part.name for part in object_model.parts]}",
    )
    pointer_aabb = ctx.part_element_world_aabb(selector, elem="pointer_mark")
    ctx.check(
        "selector has a visible raised pointer mark",
        pointer_aabb is not None
        and selector_aabb is not None
        and pointer_aabb[1][1] >= selector_aabb[1][1] - POINTER_T * 1.5,
        details=f"pointer={pointer_aabb}, selector={selector_aabb}",
    )

    ctx.expect_overlap(
        selector,
        housing,
        axes="xz",
        elem_a="selector_stem",
        elem_b="housing_shell",
        min_overlap=KNOB_STEM_R * 1.4,
        name="selector stem passes through the front bore footprint",
    )

    # --- Mechanism actually rotates the selector ----------------------------
    rest_pointer = ctx.part_element_world_aabb(selector, elem="pointer_mark")
    with ctx.pose({turn_selector: KNOB_TURN}):
        turned_pointer = ctx.part_element_world_aabb(selector, elem="pointer_mark")
    ctx.check(
        "turning the selector moves the pointer around the face",
        rest_pointer is not None
        and turned_pointer is not None
        and (
            abs(turned_pointer[1][0] - rest_pointer[1][0]) > 0.003
            or abs(turned_pointer[0][2] - rest_pointer[0][2]) > 0.003
        ),
        details=f"rest={rest_pointer}, turned={turned_pointer}",
    )

    ctx.expect_contact(
        housing,
        selector,
        elem_b="selector_knob_shell",
        name="selector knob mounts on the front face",
    )
    # The rails are intentionally seated in the rear clamp grooves.
    ctx.allow_overlap(
        housing,
        rail_top,
        reason="The top rail is intentionally seated in the rear clamp groove.",
    )
    ctx.allow_overlap(
        housing,
        rail_bottom,
        reason="The bottom rail is intentionally seated in the rear clamp groove.",
    )
    ctx.expect_contact(
        housing,
        rail_top,
        name="top rail is seated against the clamp groove",
    )
    ctx.expect_contact(
        housing,
        rail_bottom,
        name="bottom rail is seated against the clamp groove",
    )

    return ctx.report()


object_model = build_object_model()
