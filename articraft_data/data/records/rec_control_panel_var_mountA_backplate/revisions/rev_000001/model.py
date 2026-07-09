from __future__ import annotations

# Wall-back-plate control panel module (variant 2), modeled from the reference image.
#
# The fork keeps the same light-gray rounded plastic front enclosure: recessed
# dark LCD window, three small indicator LEDs across the top, a column of
# horizontal ventilation/data slots on the left edge, and two round push buttons
# at the bottom.  The rear support has been changed from twin rails and a clamp
# to a flat rectangular wall back-plate with four screw-boss mounting tabs.
#
# Real mechanism: the two round buttons are momentary push buttons. They are the
# primary articulated parts and travel linearly into the face (PRISMATIC plunge).
# The enclosure is fixed onto the back-plate (FIXED), so the whole assembly has
# a single connected root.
#
# Frame convention (housing part frame, meters):
#   +X = right
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

# Wall mounting back-plate: flat rectangle behind the enclosure with four
# integrated screw-boss tabs. The back-plate part frame sits on the actual rear
# contact plane of the enclosure; its visual thickness extends toward -Y.
PLATE_W = 0.130
PLATE_H = 0.146
PLATE_T = 0.006
PLATE_CORNER = 0.008
MOUNT_BOSS_R = 0.0068
MOUNT_HOLE_R = 0.0022
MOUNT_BOSS_PROUD = 0.0020
MOUNT_TAB_X = PLATE_W / 2.0 - 0.012
MOUNT_TAB_Z = PLATE_H / 2.0 - 0.012
MOUNT_TAB_POSITIONS = (
    (-MOUNT_TAB_X, MOUNT_TAB_Z),
    (MOUNT_TAB_X, MOUNT_TAB_Z),
    (-MOUNT_TAB_X, -MOUNT_TAB_Z),
    (MOUNT_TAB_X, -MOUNT_TAB_Z),
)


# --- Materials ---------------------------------------------------------------

MAT_HOUSING = Material(name="panel_plastic", rgba=(0.74, 0.74, 0.71, 1.0))
MAT_DISPLAY = Material(name="lcd_glass", rgba=(0.16, 0.20, 0.18, 1.0))
MAT_LED = Material(name="indicator_led", rgba=(0.30, 0.32, 0.33, 1.0))
MAT_BUTTON = Material(name="button_cap", rgba=(0.58, 0.58, 0.56, 1.0))
MAT_PLATE = Material(name="wall_back_plate", rgba=(0.44, 0.44, 0.42, 1.0))


# --- Housing geometry --------------------------------------------------------


# Axis mapping used throughout (CadQuery local frame == part-local frame):
#   X = width (left-right)
#   Y = depth (front face normal points +Y)
#   Z = height (up)
# cq box(dx, dy, dz) extents map directly to (X, Y, Z).
# A Z-axis cq cylinder rotated +90 deg about X has its axis along Y (front-back).


def _y_cyl(radius: float, length: float) -> cq.Workplane:
    """Cylinder whose axis runs along Y (front-back), centered at the origin."""
    return cq.Workplane("XY").cylinder(length, radius).rotate((0, 0, 0), (1, 0, 0), 90.0)

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


def _build_wall_back_plate_shape() -> cq.Workplane:
    """Flat wall back-plate slab with four keyhole cuts.

    The local part frame is the mounting contact plane: y=0 touches the rear rim
    of the enclosure, and the plate thickness extends backward toward -Y.
    """
    plate = (
        cq.Workplane("XY")
        .box(PLATE_W, PLATE_T, PLATE_H)
        .edges("|Y")
        .fillet(PLATE_CORNER)
        .translate((0.0, -PLATE_T / 2.0, 0.0))
    )

    # Four keyhole openings through the flat plate align with the raised bosses.
    for x, z in MOUNT_TAB_POSITIONS:
        through_hole = _y_cyl(MOUNT_HOLE_R, PLATE_T + 0.004).translate(
            (x, -PLATE_T / 2.0, z)
        )
        plate = plate.cut(through_hole.val())

        slot = (
            cq.Workplane("XY")
            .box(MOUNT_HOLE_R * 1.35, PLATE_T + 0.004, 0.006)
            .translate((x, -PLATE_T / 2.0, z - 0.0038))
        )
        plate = plate.cut(slot.val())

    return plate


def _build_mount_tab_shape() -> cq.Workplane:
    """One raised screw-boss/keyhole tab, authored about a local tab center."""
    tab = _y_cyl(MOUNT_BOSS_R, MOUNT_BOSS_PROUD + 0.0002).translate(
        (0.0, MOUNT_BOSS_PROUD / 2.0 - 0.0001, 0.0)
    )
    through_hole = _y_cyl(MOUNT_HOLE_R, MOUNT_BOSS_PROUD + 0.006).translate(
        (0.0, MOUNT_BOSS_PROUD / 2.0, 0.0)
    )
    tab = tab.cut(through_hole.val())
    slot = (
        cq.Workplane("XY")
        .box(MOUNT_HOLE_R * 1.35, MOUNT_BOSS_PROUD + 0.006, 0.006)
        .translate((0.0, MOUNT_BOSS_PROUD / 2.0, -0.0038))
    )
    return tab.cut(slot.val())


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_plate_control_panel")

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

    # --- Wall back-plate (FIXED at the actual rear contact plane) -----------
    # This replaces the parent twin-rail rear clamp: the flat plate sits against
    # the enclosure's back rim and carries four integral screw/keyhole bosses.
    back_plate = model.part("back_plate")
    back_plate.visual(
        mesh_from_cadquery(_build_wall_back_plate_shape(), "wall_back_plate"),
        material=MAT_PLATE,
        name="wall_back_plate",
    )
    for idx, (x, z) in enumerate(MOUNT_TAB_POSITIONS):
        back_plate.visual(
            mesh_from_cadquery(_build_mount_tab_shape(), f"mounting_tab_{idx}"),
            material=MAT_PLATE,
            name=f"mounting_tab_{idx}",
            origin=Origin(xyz=(x, 0.0, z)),
        )
    model.articulation(
        "housing_to_back_plate",
        ArticulationType.FIXED,
        parent=housing,
        child=back_plate,
        origin=Origin(xyz=(0.0, BACK_Y, 0.0)),
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
    back_plate = object_model.get_part("back_plate")
    button_left = object_model.get_part("button_left")
    button_right = object_model.get_part("button_right")
    press_left = object_model.get_articulation("press_button_left")
    press_right = object_model.get_articulation("press_button_right")
    housing_to_back_plate = object_model.get_articulation("housing_to_back_plate")

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
        "back plate is fixed to the housing rear",
        str(housing_to_back_plate.articulation_type).endswith("FIXED"),
        details=f"type={housing_to_back_plate.articulation_type}",
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

    # The replacement rear support is a flat rectangular plate, not rails: it
    # extends beyond the housing on both X and Z and sits behind the enclosure.
    plate_aabb = ctx.part_world_aabb(back_plate)
    ctx.check(
        "back plate is wider and taller than the enclosure",
        plate_aabb is not None
        and (plate_aabb[1][0] - plate_aabb[0][0]) > HOUSING_W + 0.025
        and (plate_aabb[1][2] - plate_aabb[0][2]) > HOUSING_H + 0.030,
        details=f"plate_aabb={plate_aabb}",
    )
    ctx.check(
        "back plate sits behind the housing front face",
        plate_aabb is not None and plate_aabb[1][1] < FACE_Y,
        details=f"plate_aabb={plate_aabb}",
    )
    ctx.expect_contact(
        housing,
        back_plate,
        name="housing rear rim contacts the flat back plate",
    )
    ctx.check(
        "four screw boss mounting tabs are authored",
        len(MOUNT_TAB_POSITIONS) == 4
        and all(back_plate.get_visual(f"mounting_tab_{i}") is not None for i in range(4)),
        details=f"mount_tab_positions={MOUNT_TAB_POSITIONS}",
    )
    ctx.check(
        "no twin rail parts remain",
        not any("rail" in part.name for part in object_model.parts),
        details=f"parts={[part.name for part in object_model.parts]}",
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
    # The plate is intentionally a rear surface mount, not a hidden rail sleeve.
    ctx.expect_gap(
        housing,
        back_plate,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0005,
        negative_elem="wall_back_plate",
        name="back plate is flush against the rear mounting plane",
    )

    return ctx.report()


object_model = build_object_model()
