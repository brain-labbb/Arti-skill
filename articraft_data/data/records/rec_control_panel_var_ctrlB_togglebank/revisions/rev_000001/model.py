from __future__ import annotations

# Rail-mounted control panel module (variant 2), modeled from the reference image.
#
# The reference shows a light-gray rounded plastic enclosure clamped onto two
# horizontal dark-metal rails. The face carries a recessed dark LCD window, three
# small indicator LEDs across the top, a column of horizontal ventilation/data
# slots on the left edge, and a row of small toggle switch levers at the bottom.
#
# Real mechanism: the bottom controls are miniature toggle switches. Each bat
# pivots in a visible front-face bushing (REVOLUTE pitch) rather than plunging
# linearly. The two rails are carried by the rear clamp of the housing (FIXED),
# so the whole assembly has a single connected root.
#
# Frame convention (housing part frame, meters):
#   +X = right (along the rails)
#   +Y = out toward the viewer (the visible face normal)
#   +Z = up
# The housing front face is at +Y; toggle bats protrude toward +Y and pivot
# about a horizontal X-axis at the front face.

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

# Toggle switches (row of small pivoting bats, bottom)
SWITCH_COUNT = 3
SWITCH_Z = -HOUSING_H / 2.0 + 0.020
SWITCH_CX = 0.006
SWITCH_DX = 0.014
SWITCH_SOCKET_R = 0.0060
SWITCH_SOCKET_INNER_R = 0.0039
SWITCH_BALL_R = 0.0033
SWITCH_BAT_R = 0.0018
SWITCH_BAT_LEN = 0.023
SWITCH_THROW = 0.45  # about +/-26 degrees

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
MAT_SWITCH = Material(name="toggle_bat_metal", rgba=(0.50, 0.50, 0.48, 1.0))
MAT_SOCKET = Material(name="toggle_bushing_dark", rgba=(0.18, 0.18, 0.18, 1.0))
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

    # Round pivot holes for the toggle bushings (axis along Y, through the face).
    for idx in range(SWITCH_COUNT):
        x = SWITCH_CX + (idx - (SWITCH_COUNT - 1) / 2.0) * SWITCH_DX
        bore = _y_cyl(SWITCH_SOCKET_INNER_R, 0.014).translate(
            (x, FACE_Y, SWITCH_Z)
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


def _switch_x(idx: int) -> float:
    """Evenly spaced toggle-switch center on the housing front."""
    return SWITCH_CX + (idx - (SWITCH_COUNT - 1) / 2.0) * SWITCH_DX


def _build_switch_socket() -> cq.Workplane:
    """Raised annular bushing/washer fixed to the front face around a toggle."""
    outer = _y_cyl(SWITCH_SOCKET_R, 0.0024).translate((0.0, 0.0012, 0.0))
    inner = _y_cyl(SWITCH_SOCKET_INNER_R, 0.0040).translate((0.0, 0.0012, 0.0))
    washer = outer.cut(inner.val())
    # A small flat notch at the bottom gives the bushing a manufactured keyed look.
    notch = (
        cq.Workplane("XY")
        .box(SWITCH_SOCKET_R * 0.9, 0.0045, 0.0010)
        .translate((0.0, 0.0012, -SWITCH_SOCKET_R * 0.78))
    )
    return washer.cut(notch.val())


def _build_toggle_shape() -> cq.Workplane:
    """Movable toggle bat in its own joint frame.

    Local origin is the pivot point on the housing face. At q=0 the bat sticks
    straight outward along +Y; a revolute X-axis pitch tilts it up/down.
    """
    pivot = cq.Workplane("XY").sphere(SWITCH_BALL_R).translate(
        (0.0, SWITCH_BALL_R, 0.0)
    )
    # Slightly oversize shank nests in the bushing bore so the moving toggle is
    # visibly retained rather than floating in front of the face.
    shank = _y_cyl(SWITCH_SOCKET_INNER_R + 0.00015, 0.0032).translate(
        (0.0, 0.0010, 0.0)
    )
    stem = _y_cyl(SWITCH_BAT_R, SWITCH_BAT_LEN).translate(
        (0.0, SWITCH_BALL_R + SWITCH_BAT_LEN / 2.0, 0.0)
    )
    grip = _y_cyl(SWITCH_BAT_R * 1.25, 0.0050).translate(
        (0.0, SWITCH_BALL_R + SWITCH_BAT_LEN + 0.0025, 0.0)
    )
    tip = cq.Workplane("XY").sphere(SWITCH_BAT_R * 1.35).translate(
        (0.0, SWITCH_BALL_R + SWITCH_BAT_LEN + 0.0052, 0.0)
    )
    return shank.union(pivot.val()).union(stem.val()).union(grip.val()).union(tip.val())


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
    for idx in range(SWITCH_COUNT):
        housing.visual(
            mesh_from_cadquery(_build_switch_socket(), f"switch_socket_{idx}"),
            material=MAT_SOCKET,
            name=f"switch_socket_{idx}",
            origin=Origin(xyz=(_switch_x(idx), FACE_Y, SWITCH_Z)),
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

    # --- Toggle switches (REVOLUTE, the primary mechanism) ------------------
    for idx in range(SWITCH_COUNT):
        sname = f"switch_{idx}"
        sw = model.part(sname)
        sw.visual(
            mesh_from_cadquery(_build_toggle_shape(), f"{sname}_bat"),
            material=MAT_SWITCH,
            name=f"{sname}_bat",
        )
        # Joint frame is exactly on the front-face pivot point in the visible
        # bushing. Axis +X makes positive q tilt the outward bat upward.
        model.articulation(
            f"housing_to_{sname}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=sw,
            origin=Origin(xyz=(_switch_x(idx), FACE_Y, SWITCH_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.30, velocity=2.5, lower=-SWITCH_THROW, upper=SWITCH_THROW
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    rail_top = object_model.get_part("rail_top")
    rail_bottom = object_model.get_part("rail_bottom")
    switches = [object_model.get_part(f"switch_{idx}") for idx in range(SWITCH_COUNT)]
    switch_joints = [
        object_model.get_articulation(f"housing_to_switch_{idx}")
        for idx in range(SWITCH_COUNT)
    ]
    clamp_to_rail_top = object_model.get_articulation("clamp_to_rail_top")
    clamp_to_rail_bottom = object_model.get_articulation("clamp_to_rail_bottom")

    # --- Joint type / axis claims ------------------------------------------
    ctx.check(
        "toggle controls are revolute",
        all(str(j.articulation_type).endswith("REVOLUTE") for j in switch_joints),
        details=f"types={[j.articulation_type for j in switch_joints]}",
    )
    ctx.check(
        "toggle pivot axes are horizontal across the face",
        all(tuple(round(c, 3) for c in j.axis) == (1.0, 0.0, 0.0) for j in switch_joints),
        details=f"axes={[j.axis for j in switch_joints]}",
    )
    ctx.check(
        "rails are fixed to the housing",
        str(clamp_to_rail_top.articulation_type).endswith("FIXED")
        and str(clamp_to_rail_bottom.articulation_type).endswith("FIXED"),
        details=f"top={clamp_to_rail_top.articulation_type}, "
        f"bottom={clamp_to_rail_bottom.articulation_type}",
    )
    ctx.check(
        "toggle throw is a realistic small pitch",
        all(
            j.motion_limits is not None
            and j.motion_limits.lower is not None
            and j.motion_limits.upper is not None
            and -0.70 <= j.motion_limits.lower <= -0.20
            and 0.20 <= j.motion_limits.upper <= 0.70
            for j in switch_joints
        ),
        details=f"limits={[j.motion_limits for j in switch_joints]}",
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

    # Toggle bats and their visible bushings stand proud of the face at rest.
    mid_switch_aabb = ctx.part_world_aabb(switches[1])
    ctx.check(
        "toggle bat protrudes outward from the face",
        mid_switch_aabb is not None and mid_switch_aabb[1][1] > FACE_Y + 0.020,
        details=f"switch_1_aabb={mid_switch_aabb}",
    )
    socket_aabbs = [
        ctx.part_element_world_aabb(housing, elem=housing.get_visual(f"switch_socket_{idx}"))
        for idx in range(SWITCH_COUNT)
    ]
    ctx.check(
        "three visible toggle bushings are on the front face",
        all(aabb is not None and aabb[0][1] >= FACE_Y - 0.0001 for aabb in socket_aabbs),
        details=f"socket_aabbs={socket_aabbs}",
    )

    # --- Mechanism actually pivots the toggle bats --------------------------
    rest_mid_aabb = ctx.part_world_aabb(switches[1])
    with ctx.pose({switch_joints[1]: SWITCH_THROW}):
        tilted_mid_aabb = ctx.part_world_aabb(switches[1])
    ctx.check(
        "positive toggle motion tilts the bat upward",
        rest_mid_aabb is not None
        and tilted_mid_aabb is not None
        and tilted_mid_aabb[1][2] > rest_mid_aabb[1][2] + 0.006,
        details=f"rest={rest_mid_aabb}, tilted={tilted_mid_aabb}",
    )
    with ctx.pose({switch_joints[1]: -SWITCH_THROW}):
        lowered_mid_aabb = ctx.part_world_aabb(switches[1])
    ctx.check(
        "negative toggle motion tilts the bat downward",
        rest_mid_aabb is not None
        and lowered_mid_aabb is not None
        and lowered_mid_aabb[0][2] < rest_mid_aabb[0][2] - 0.006,
        details=f"rest={rest_mid_aabb}, lowered={lowered_mid_aabb}",
    )

    # Switches form a regular row of distinct controls.
    ctx.expect_origin_distance(
        switches[0],
        switches[1],
        axes="x",
        min_dist=SWITCH_DX - 0.001,
        max_dist=SWITCH_DX + 0.001,
        name="switches are evenly separated",
    )
    ctx.expect_origin_distance(
        switches[1],
        switches[2],
        axes="x",
        min_dist=SWITCH_DX - 0.001,
        max_dist=SWITCH_DX + 0.001,
        name="switch row has uniform pitch",
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
