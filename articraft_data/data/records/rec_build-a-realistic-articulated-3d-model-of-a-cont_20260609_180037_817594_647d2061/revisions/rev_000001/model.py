from __future__ import annotations

# Rail-mounted control panel module (variant 2), modeled from the reference image.
#
# The reference shows a light-gray rounded plastic enclosure clamped onto two
# horizontal dark-metal rails. The face carries a recessed dark LCD window, three
# small indicator LEDs across the top, a column of horizontal ventilation/data
# slots on the left edge, and two round push buttons at the bottom.
#
# Real mechanism: the two round buttons are momentary push buttons. They are the
# primary articulated parts and travel linearly into the face (PRISMATIC plunge).
# The two rails are carried by the rear clamp of the housing (FIXED), so the whole
# assembly has a single connected root.
#
# Frame convention (housing part frame, meters):
#   +X = right (along the rails)
#   +Y = out toward the viewer (the visible face normal)
#   +Z = up
# The housing front face is at +Y; buttons press toward -Y.

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
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

# Push buttons (two round, bottom)
BTN_R = 0.0095
BTN_Z = -HOUSING_H / 2.0 + 0.020
BTN_DX = 0.018
BTN_CX = 0.006
BTN_BODY_L = 0.010  # button cylinder length along Y
BTN_TRAVEL = 0.0030  # plunge depth
BTN_PROUD = 0.0035  # how far the rest button stands out from the face

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
MAT_BUTTON = Material(name="button_cap", rgba=(0.58, 0.58, 0.56, 1.0))
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

    # Bores for the two push-button barrels (axis along Y, through the face).
    for k in (-1, 1):
        bore = _y_cyl(BTN_R + 0.0010, 0.014).translate(
            (BTN_CX + k * BTN_DX, FACE_Y, BTN_Z)
        )
        body = body.cut(bore.val())

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


def _build_button_shape() -> cq.Workplane:
    """A round button cap with a short barrel, authored in the button part frame.

    The button part frame origin sits on the face plane (y=0 local). The cap
    crown stands proud toward +Y; the barrel extends back into -Y through the
    bore so the button stays captured at all travel.
    """
    crown = _y_cyl(BTN_R, 0.0035).translate((0.0, BTN_PROUD - 0.0035 / 2.0, 0.0))
    barrel = _y_cyl(BTN_R - 0.0014, BTN_BODY_L).translate((0.0, -BTN_BODY_L / 2.0, 0.0))
    # Domed top for a tactile look: a thin sphere cap above the crown.
    cap = (
        cq.Workplane("XY")
        .sphere(BTN_R)
        .translate((0.0, BTN_PROUD - 0.0035, 0.0))
        .intersect(
            cq.Workplane("XY")
            .box(BTN_R * 2.2, 0.004, BTN_R * 2.2)
            .translate((0.0, BTN_PROUD - 0.002, 0.0))
            .val()
        )
    )
    return crown.union(barrel.val()).union(cap.val())


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

    # --- Push buttons (PRISMATIC, the primary mechanism) --------------------
    for idx, k in enumerate((-1, 1)):
        bname = "button_left" if k == -1 else "button_right"
        btn = model.part(bname)
        btn.visual(
            mesh_from_cadquery(_build_button_shape(), f"{bname}_cap"),
            material=MAT_BUTTON,
            name=f"{bname}_cap",
        )
        # Joint frame on the face plane at the button center. Axis -Y means
        # positive q pushes the button into the housing (a real press).
        model.articulation(
            f"press_{bname}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=btn,
            origin=Origin(xyz=(BTN_CX + k * BTN_DX, FACE_Y, BTN_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0, velocity=0.05, lower=0.0, upper=BTN_TRAVEL
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    rail_top = object_model.get_part("rail_top")
    rail_bottom = object_model.get_part("rail_bottom")
    button_left = object_model.get_part("button_left")
    button_right = object_model.get_part("button_right")
    press_left = object_model.get_articulation("press_button_left")
    press_right = object_model.get_articulation("press_button_right")
    clamp_to_rail_top = object_model.get_articulation("clamp_to_rail_top")
    clamp_to_rail_bottom = object_model.get_articulation("clamp_to_rail_bottom")

    # --- Joint type / axis claims ------------------------------------------
    ctx.check(
        "buttons are prismatic",
        str(press_left.articulation_type).endswith("PRISMATIC")
        and str(press_right.articulation_type).endswith("PRISMATIC"),
        details=f"left={press_left.articulation_type}, right={press_right.articulation_type}",
    )
    ctx.check(
        "button press axis is into the face (-Y)",
        tuple(round(c, 3) for c in press_left.axis) == (0.0, -1.0, 0.0),
        details=f"axis={press_left.axis}",
    )
    ctx.check(
        "rails are fixed to the housing",
        str(clamp_to_rail_top.articulation_type).endswith("FIXED")
        and str(clamp_to_rail_bottom.articulation_type).endswith("FIXED"),
        details=f"top={clamp_to_rail_top.articulation_type}, "
        f"bottom={clamp_to_rail_bottom.articulation_type}",
    )
    ctx.check(
        "button travel is a realistic short plunge",
        press_left.motion_limits is not None
        and press_left.motion_limits.upper is not None
        and 0.0015 <= press_left.motion_limits.upper <= 0.006,
        details=f"upper={getattr(press_left.motion_limits, 'upper', None)}",
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

    # Buttons stand proud of the face at rest.
    bl_aabb = ctx.part_world_aabb(button_left)
    ctx.check(
        "button stands proud of the face at rest",
        bl_aabb is not None and bl_aabb[1][1] > FACE_Y + 0.0005,
        details=f"button_left_aabb={bl_aabb}",
    )

    # --- Mechanism actually moves the buttons -------------------------------
    rest_left = ctx.part_world_position(button_left)
    with ctx.pose({press_left: BTN_TRAVEL}):
        pressed_left = ctx.part_world_position(button_left)
    ctx.check(
        "pressing the left button moves it into the face (-Y)",
        rest_left is not None
        and pressed_left is not None
        and pressed_left[1] < rest_left[1] - 0.0015,
        details=f"rest={rest_left}, pressed={pressed_left}",
    )

    rest_right = ctx.part_world_position(button_right)
    with ctx.pose({press_right: BTN_TRAVEL}):
        pressed_right = ctx.part_world_position(button_right)
    ctx.check(
        "pressing the right button moves it into the face (-Y)",
        rest_right is not None
        and pressed_right is not None
        and pressed_right[1] < rest_right[1] - 0.0015,
        details=f"rest={rest_right}, pressed={pressed_right}",
    )

    # Buttons are horizontally separated (two distinct controls).
    ctx.expect_origin_distance(
        button_left,
        button_right,
        axes="x",
        min_dist=0.025,
        name="two buttons are separated horizontally",
    )

    # The button barrel is captured inside the housing bore (intentional nesting),
    # so allow that local overlap and prove the retention with contact.
    ctx.allow_overlap(
        housing,
        button_left,
        reason="The left button barrel is intentionally captured inside the housing bore.",
    )
    ctx.allow_overlap(
        housing,
        button_right,
        reason="The right button barrel is intentionally captured inside the housing bore.",
    )
    ctx.expect_contact(
        housing,
        button_left,
        name="left button is retained in the housing bore",
    )
    ctx.expect_contact(
        housing,
        button_right,
        name="right button is retained in the housing bore",
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
