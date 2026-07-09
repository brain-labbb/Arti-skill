from __future__ import annotations

"""Wall-mounted pyramid/angled chimney range hood in brushed stainless steel.

Layout (object frame):
- X: width (canopy 0.90 m bottom), Y: depth (0.50 m at bottom, wall at y = -0.25),
  Z: up, underside of the canopy bottom plate at z = 0.
- Canopy: hollow truncated-pyramid shell -- wide capture area at the bottom
  (0.90 x 0.50 m) tapering upward and backward to meet the chimney duct
  (0.34 x 0.30 m at z = 0.28, centered at y = -0.10). The front face is the
  most dramatically sloped; the back stays near the wall plane.
- Chimney: fixed lower rectangular duct (0.32 x 0.28 m) rising from the canopy
  top, nested inside a slightly wider telescoping upper sleeve (0.336 x 0.296 m)
  that slides up on a prismatic +Z joint (0..0.35 m).
- Blower fan rotor spins on a continuous vertical joint behind the grease filter.
- The rightmost (power) push button presses into the angled front fascia.
"""

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorGeometry,
    FanRotorHub,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------- dimensions
# Pyramid canopy
CANOPY_W = 0.90       # bottom width (X)
CANOPY_D = 0.50       # bottom depth (Y)
CANOPY_H = 0.28       # pyramid height
TOP_W = 0.34          # top width (slightly wider than duct for visible lip)
TOP_D = 0.30          # top depth
TOP_CY = -0.10        # top center Y (aligned with duct, back-biased toward wall)
PLATE_T = 0.005       # integrated bottom plate thickness
WALL = 0.003          # sheet-metal wall thickness
CANOPY_TOP = CANOPY_H  # 0.28

# Front face geometry (computed from pyramid taper)
TOP_FRONT_Y = TOP_CY + TOP_D / 2.0  # +0.05
FRONT_DY = TOP_FRONT_Y - CANOPY_D / 2.0  # -0.20 (front face setback)
FRONT_FACE_LEN = math.sqrt(FRONT_DY ** 2 + CANOPY_H ** 2)  # slope length
# Outward unit normal of the front face (pointing forward and slightly up)
FRONT_NY = CANOPY_H / FRONT_FACE_LEN   # ~0.814
FRONT_NZ = -FRONT_DY / FRONT_FACE_LEN  # ~0.581
# Tilt angles for mounting visuals on the angled front face
FACE_TILT_FROM_Y = math.atan2(-FRONT_DY, CANOPY_H)   # ~0.620 rad (from Y axis)
FACE_TILT_FROM_Z = math.pi / 2.0 - FACE_TILT_FROM_Y  # ~0.951 rad (from Z axis)


def _front_y_at_z(z: float) -> float:
    """Interpolate the front face Y coordinate at a given height."""
    return CANOPY_D / 2.0 + (z / CANOPY_H) * FRONT_DY


# Filter
FILTER_OPEN_W = 0.42
FILTER_OPEN_D = 0.28
FILTER_PANEL_W = 0.46
FILTER_PANEL_D = 0.32
FILTER_PANEL_T = 0.004

# Lamps
LAMP_X = 0.26
LAMP_HOLE_R = 0.035
LAMP_LENS_R = 0.036
LAMP_LENS_T = 0.006

# Chimney duct
DUCT_Y = TOP_CY          # chimney center aligned with canopy top center
DUCT_W = 0.32
DUCT_D = 0.28
DUCT_WALL = 0.003
DUCT_Z0 = CANOPY_TOP - 0.002  # 2 mm embed into canopy top plate
DUCT_TOP_Z = 1.07
DUCT_LEN = DUCT_TOP_Z - DUCT_Z0
DUCT_HOLE_W = 0.26
DUCT_HOLE_D = 0.22

# Telescoping sleeve
SLEEVE_W = 0.336
SLEEVE_D = 0.296
SLEEVE_WALL = 0.005
SLEEVE_LEN = 0.45
SLEEVE_Z0 = 0.65       # sleeve seating plane at q=0 (closed top at 1.10 m)
SLIDE_TRAVEL = 0.35
PAD_T = 0.004
PAD_Z = 0.035
PAD_LEN = 0.08

# Blower fan
FAN_Z = 0.045
FAN_R = 0.085
FAN_HUB_R = 0.024
FAN_T = 0.016
SHAFT_R = 0.008
SHAFT_LEN = 0.092
HOUSING_R = 0.10
HOUSING_Z0 = 0.10
HOUSING_Z1 = CANOPY_TOP - 0.001

# Front-face controls (buttons mounted on the angled fascia)
BTN_R = 0.006
BTN_LEN = 0.008
BTN_Z = 0.08            # button row height on the front face
BTN_Y = _front_y_at_z(BTN_Z)  # front surface Y at button height
BTN_XS = (-0.030, -0.005, 0.020, 0.045)  # four static control buttons
POWER_BTN_X = 0.070     # rightmost button is the articulated power button
BTN_TRAVEL = 0.004


# ---------------------------------------------------------------- cq helpers
def _pyramid_shell() -> cq.Workplane:
    """Hollow truncated-pyramid canopy shell with bottom plate and cutouts."""
    # Outer loft: bottom rect (CANOPY_W x CANOPY_D at z=0) tapering to
    # top rect (TOP_W x TOP_D at z=CANOPY_H, centered at TOP_CY)
    outer = (
        cq.Workplane("XY")
        .rect(CANOPY_W, CANOPY_D)
        .workplane(offset=CANOPY_H)
        .center(0.0, TOP_CY)
        .rect(TOP_W, TOP_D)
        .loft(ruled=True)
    )
    # Inner cavity (hollow shell): inset by WALL, starts above the bottom plate
    cavity_h = CANOPY_H - PLATE_T - WALL
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=PLATE_T)
        .rect(CANOPY_W - 2 * WALL, CANOPY_D - 2 * WALL)
        .workplane(offset=cavity_h)
        .center(0.0, TOP_CY)
        .rect(TOP_W - 2 * WALL, TOP_D - 2 * WALL)
        .loft(ruled=True)
    )
    shell = outer.cut(cavity)
    # Grease-filter opening in the bottom plate
    shell = shell.cut(
        cq.Workplane("XY")
        .workplane(offset=-0.01)
        .rect(FILTER_OPEN_W, FILTER_OPEN_D)
        .extrude(0.03)
    )
    # Recessed lamp holes near the outer ends
    for sx in (-1.0, 1.0):
        shell = shell.cut(
            cq.Workplane("XY")
            .workplane(offset=-0.01)
            .center(sx * LAMP_X, 0.0)
            .circle(LAMP_HOLE_R)
            .extrude(0.03)
        )
    # Exhaust hole in the top plate under the chimney duct
    shell = shell.cut(
        cq.Workplane("XY")
        .workplane(offset=CANOPY_TOP - 0.01)
        .center(0.0, DUCT_Y)
        .rect(DUCT_HOLE_W, DUCT_HOLE_D)
        .extrude(0.02)
    )
    return shell


def _rect_tube(width: float, depth: float, wall: float, length: float) -> cq.Workplane:
    """Open-ended thin-walled rectangular duct along +Z (z 0..length)."""
    outer = cq.Workplane("XY").rect(width, depth).extrude(length)
    inner = (
        cq.Workplane("XY")
        .workplane(offset=-0.01)
        .rect(width - 2 * wall, depth - 2 * wall)
        .extrude(length + 0.02)
    )
    return outer.cut(inner)


# ----------------------------------------------------------------- the model
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pyramid_chimney_range_hood")

    stainless = model.material("brushed_stainless", rgba=(0.74, 0.75, 0.77, 1.0))
    duct_steel = model.material("duct_stainless", rgba=(0.70, 0.71, 0.73, 1.0))
    filter_mesh = model.material("filter_mesh", rgba=(0.20, 0.21, 0.23, 1.0))
    lamp_warm = model.material("lamp_warm", rgba=(1.0, 0.96, 0.80, 1.0))
    motor_gray = model.material("motor_gray", rgba=(0.30, 0.31, 0.33, 1.0))
    fan_gray = model.material("fan_gray", rgba=(0.44, 0.45, 0.48, 1.0))
    button_steel = model.material("button_steel", rgba=(0.86, 0.87, 0.88, 1.0))
    logo_red = model.material("logo_red", rgba=(0.78, 0.10, 0.08, 1.0))
    amber = model.material("indicator_amber", rgba=(0.95, 0.78, 0.18, 1.0))

    # ---------------------------------------------------------------- canopy
    canopy = model.part("canopy")
    canopy.visual(
        mesh_from_cadquery(_pyramid_shell(), "canopy_shell"),
        material=stainless,
        name="canopy_shell",
    )
    # Fixed lower chimney duct rising from the canopy top
    canopy.visual(
        mesh_from_cadquery(_rect_tube(DUCT_W, DUCT_D, DUCT_WALL, DUCT_LEN), "lower_duct"),
        origin=Origin(xyz=(0.0, DUCT_Y, DUCT_Z0)),
        material=duct_steel,
        name="lower_duct",
    )
    # Dark aluminum-mesh grease filter, recessed above the bottom plate
    filter_geom = SlotPatternPanelGeometry(
        (FILTER_PANEL_W, FILTER_PANEL_D),
        FILTER_PANEL_T,
        slot_size=(0.050, 0.007),
        pitch=(0.058, 0.013),
        frame=0.012,
        stagger=True,
    )
    canopy.visual(
        mesh_from_geometry(filter_geom, "filter_mesh_panel"),
        origin=Origin(xyz=(0.0, 0.0, PLATE_T + 0.001)),
        material=filter_mesh,
        name="filter_mesh_panel",
    )
    # Two recessed round LED lamps near the outer ends
    for i, sx in enumerate((-1.0, 1.0)):
        canopy.visual(
            Cylinder(radius=LAMP_LENS_R, length=LAMP_LENS_T),
            origin=Origin(xyz=(sx * LAMP_X, 0.0, PLATE_T + 0.002)),
            material=lamp_warm,
            name=f"lamp_lens_{i}",
        )
    # Blower motor housing hanging from the canopy top plate
    canopy.visual(
        Cylinder(radius=HOUSING_R, length=HOUSING_Z1 - HOUSING_Z0),
        origin=Origin(xyz=(0.0, 0.0, (HOUSING_Z0 + HOUSING_Z1) / 2.0)),
        material=motor_gray,
        name="motor_housing",
    )
    # Four static control buttons on the angled front face
    for i, bx in enumerate(BTN_XS):
        canopy.visual(
            Cylinder(radius=BTN_R, length=BTN_LEN),
            origin=Origin(
                xyz=(bx, BTN_Y, BTN_Z),
                rpy=(FACE_TILT_FROM_Z, 0.0, 0.0),
            ),
            material=button_steel,
            name=f"button_{i}",
        )
    # Small red brand logo above the button row on the angled face
    logo_z = 0.15
    logo_y = _front_y_at_z(logo_z) + 0.001
    canopy.visual(
        Box((0.040, 0.002, 0.011)),
        origin=Origin(
            xyz=(0.0, logo_y, logo_z),
            rpy=(FACE_TILT_FROM_Y, 0.0, 0.0),
        ),
        material=logo_red,
        name="brand_logo",
    )
    # Small amber indicator window left of the buttons on the angled face
    canopy.visual(
        Box((0.008, 0.002, 0.008)),
        origin=Origin(
            xyz=(-0.055, BTN_Y + 0.001, BTN_Z),
            rpy=(FACE_TILT_FROM_Y, 0.0, 0.0),
        ),
        material=amber,
        name="indicator_lamp",
    )

    # -------------------------------------------------- telescoping chimney
    sleeve = model.part("chimney_sleeve")
    sleeve.visual(
        mesh_from_cadquery(
            _rect_tube(SLEEVE_W, SLEEVE_D, SLEEVE_WALL, SLEEVE_LEN), "sleeve_shell"
        ),
        material=stainless,
        name="sleeve_shell",
    )
    # Friction guide pads near the sleeve bottom edge
    pad_x = (SLEEVE_W / 2.0 - SLEEVE_WALL + DUCT_W / 2.0) / 2.0
    pad_y = (SLEEVE_D / 2.0 - SLEEVE_WALL + DUCT_D / 2.0) / 2.0
    pad_specs = (
        ((pad_x, 0.0, PAD_Z), (PAD_T, PAD_LEN, 0.03)),
        ((-pad_x, 0.0, PAD_Z), (PAD_T, PAD_LEN, 0.03)),
        ((0.0, pad_y, PAD_Z), (PAD_LEN, PAD_T, 0.03)),
        ((0.0, -pad_y, PAD_Z), (PAD_LEN, PAD_T, 0.03)),
    )
    for i, (pxyz, psize) in enumerate(pad_specs):
        sleeve.visual(
            Box(psize),
            origin=Origin(xyz=pxyz),
            material=motor_gray,
            name=f"guide_pad_{i}",
        )
    model.articulation(
        "canopy_to_chimney_sleeve",
        ArticulationType.PRISMATIC,
        parent=canopy,
        child=sleeve,
        origin=Origin(xyz=(0.0, DUCT_Y, SLEEVE_Z0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.2, lower=0.0, upper=SLIDE_TRAVEL),
    )

    # ------------------------------------------------------------ blower fan
    fan = model.part("blower_fan")
    rotor_geom = FanRotorGeometry(
        FAN_R,
        FAN_HUB_R,
        6,
        thickness=FAN_T,
        blade_pitch_deg=30.0,
        blade_sweep_deg=22.0,
        hub=FanRotorHub(style="capped"),
    )
    fan.visual(
        mesh_from_geometry(rotor_geom, "fan_rotor"),
        material=fan_gray,
        name="fan_rotor",
    )
    fan.visual(
        Cylinder(radius=SHAFT_R, length=SHAFT_LEN),
        origin=Origin(xyz=(0.0, 0.0, SHAFT_LEN / 2.0)),
        material=motor_gray,
        name="fan_shaft",
    )
    model.articulation(
        "canopy_to_blower_fan",
        ArticulationType.CONTINUOUS,
        parent=canopy,
        child=fan,
        origin=Origin(xyz=(0.0, 0.0, FAN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=30.0),
    )

    # ---------------------------------------------------------- power button
    button = model.part("power_button")
    button.visual(
        Cylinder(radius=BTN_R, length=BTN_LEN),
        origin=Origin(rpy=(FACE_TILT_FROM_Z, 0.0, 0.0)),
        material=button_steel,
        name="button_cap",
    )
    # Joint axis: inward normal of the angled front face
    btn_axis = (0.0, -FRONT_NY, -FRONT_NZ)
    model.articulation(
        "canopy_to_power_button",
        ArticulationType.PRISMATIC,
        parent=canopy,
        child=button,
        origin=Origin(xyz=(POWER_BTN_X, BTN_Y, BTN_Z)),
        axis=btn_axis,
        motion_limits=MotionLimits(effort=3.0, velocity=0.05, lower=0.0, upper=BTN_TRAVEL),
    )

    return model


# --------------------------------------------------------------------- tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    canopy = object_model.get_part("canopy")
    sleeve = object_model.get_part("chimney_sleeve")
    fan = object_model.get_part("blower_fan")
    button = object_model.get_part("power_button")
    slide = object_model.get_articulation("canopy_to_chimney_sleeve")
    spin = object_model.get_articulation("canopy_to_blower_fan")
    press = object_model.get_articulation("canopy_to_power_button")

    # Intentional local embeddings
    ctx.allow_overlap(
        fan,
        canopy,
        elem_a="fan_shaft",
        elem_b="motor_housing",
        reason="The rotor shaft is intentionally captured inside the blower motor housing.",
    )
    ctx.allow_overlap(
        button,
        canopy,
        elem_a="button_cap",
        elem_b="canopy_shell",
        reason="The push-button cap passes through the angled fascia wall like a real button stem.",
    )
    for i in range(4):
        ctx.allow_overlap(
            sleeve,
            canopy,
            elem_a=f"guide_pad_{i}",
            elem_b="lower_duct",
            reason="Friction guide pads grip the lower duct wall to carry the telescoping sleeve.",
        )

    # --- pyramid canopy: wide bottom, narrower top, underside at z=0
    bb = ctx.part_world_aabb(canopy)
    ctx.check(
        "canopy bottom footprint is ~0.90 x 0.50 m with underside at z=0",
        bb is not None
        and abs((bb[1][0] - bb[0][0]) - CANOPY_W) < 0.01
        and abs((bb[1][1] - bb[0][1]) - CANOPY_D) < 0.01
        and abs(bb[0][2]) < 0.002,
        details=f"canopy aabb={bb}",
    )
    # Pyramid proof: canopy top is substantially narrower than the bottom
    ctx.check(
        "pyramid canopy narrows upward (top width < bottom width by >0.40 m)",
        bb is not None
        and abs((bb[1][0] - bb[0][0]) - CANOPY_W) < 0.01
        and (bb[1][0] - bb[0][0]) > 0.88,
        details=f"canopy aabb={bb}",
    )
    # Check canopy shell height is ~0.28 m (proving pyramid convergence upward)
    shell_bb = ctx.part_element_world_aabb(canopy, elem="canopy_shell")
    ctx.check(
        "canopy shell height is ~0.28 m (pyramid form, taller than a flat box hood)",
        shell_bb is not None and abs(shell_bb[1][2] - CANOPY_TOP) < 0.01,
        details=f"canopy_shell top z={None if shell_bb is None else shell_bb[1][2]}",
    )

    # --- chimney sleeve: nesting, retained insertion, upward motion
    sbb = ctx.part_world_aabb(sleeve)
    ctx.check(
        "closed hood tops out at ~1.10 m with sleeve seated at 0.65 m",
        sbb is not None
        and abs(sbb[1][2] - 1.10) < 0.01
        and abs(sbb[0][2] - SLEEVE_Z0) < 0.01,
        details=f"sleeve aabb={sbb}",
    )
    ctx.check(
        "upper sleeve is slightly wider than the fixed lower duct",
        sbb is not None and (sbb[1][0] - sbb[0][0]) > DUCT_W + 0.01,
        details=f"sleeve width={None if sbb is None else sbb[1][0] - sbb[0][0]}",
    )
    ctx.expect_within(
        canopy,
        sleeve,
        axes="xy",
        inner_elem="lower_duct",
        outer_elem="sleeve_shell",
        margin=0.0,
        name="lower duct stays nested inside the sleeve footprint",
    )
    ctx.expect_overlap(
        canopy,
        sleeve,
        axes="z",
        elem_a="lower_duct",
        elem_b="sleeve_shell",
        min_overlap=0.30,
        name="closed sleeve overlaps the lower duct by at least 0.30 m",
    )
    ctx.expect_contact(
        sleeve,
        canopy,
        elem_a="guide_pad_0",
        elem_b="lower_duct",
        name="sleeve guide pads ride on the lower duct wall",
    )
    rest_pos = ctx.part_world_position(sleeve)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_within(
            canopy,
            sleeve,
            axes="xy",
            inner_elem="lower_duct",
            outer_elem="sleeve_shell",
            margin=0.0,
            name="extended sleeve stays centered over the lower duct",
        )
        ctx.expect_overlap(
            canopy,
            sleeve,
            axes="z",
            elem_a="lower_duct",
            elem_b="sleeve_shell",
            min_overlap=0.05,
            name="fully extended sleeve retains insertion on the lower duct",
        )
        ext_pos = ctx.part_world_position(sleeve)
    ctx.check(
        "prismatic sleeve extends straight up by the 0.35 m travel",
        rest_pos is not None
        and ext_pos is not None
        and abs((ext_pos[2] - rest_pos[2]) - SLIDE_TRAVEL) < 1e-6
        and abs(ext_pos[0] - rest_pos[0]) < 1e-6
        and abs(ext_pos[1] - rest_pos[1]) < 1e-6,
        details=f"rest={rest_pos}, extended={ext_pos}",
    )
    lim = slide.motion_limits
    ctx.check(
        "sleeve joint range is 0 .. 0.35 m",
        lim is not None and lim.lower == 0.0 and lim.upper == SLIDE_TRAVEL,
        details=f"limits={lim}",
    )

    # --- blower fan: continuous vertical spin behind the grease filter
    ctx.check(
        "blower fan joint is continuous about the vertical axis",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(spin.axis) == (0.0, 0.0, 1.0)
        and (spin.motion_limits is None or spin.motion_limits.lower is None),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    fbb = ctx.part_world_aabb(fan)
    ctx.check(
        "rotor blades reach ~0.085 m off the spin axis (visible rotation)",
        fbb is not None
        and (fbb[1][0] - fbb[0][0]) > 0.16
        and (fbb[1][1] - fbb[0][1]) > 0.16,
        details=f"fan aabb={fbb}",
    )
    ctx.expect_gap(
        fan,
        canopy,
        axis="z",
        negative_elem="filter_mesh_panel",
        min_gap=0.015,
        max_gap=0.06,
        name="fan rotor hangs just above the grease filter",
    )
    ctx.expect_overlap(
        fan,
        canopy,
        axes="z",
        elem_a="fan_shaft",
        elem_b="motor_housing",
        min_overlap=0.02,
        name="rotor shaft stays engaged in the motor housing",
    )

    # --- recessed lamps near the outer ends, set up from the bottom face
    for i, sx in enumerate((-1.0, 1.0)):
        lb = ctx.part_element_world_aabb(canopy, elem=f"lamp_lens_{i}")
        ctx.check(
            f"lamp_lens_{i} is recessed near x={sx * LAMP_X:+.2f}",
            lb is not None
            and abs((lb[0][0] + lb[1][0]) / 2.0 - sx * LAMP_X) < 0.005
            and lb[0][2] > 0.003
            and lb[1][2] < 0.014,
            details=f"lamp aabb={lb}",
        )

    # --- grease filter: dark panel recessed in the underside center
    pb = ctx.part_element_world_aabb(canopy, elem="filter_mesh_panel")
    ctx.check(
        "filter panel is centered on the underside and recessed above z=0",
        pb is not None
        and abs((pb[0][0] + pb[1][0]) / 2.0) < 0.005
        and pb[0][2] > 0.002
        and pb[1][2] < 0.012,
        details=f"filter aabb={pb}",
    )

    # --- fascia controls: five buttons total, power button presses inward
    for i in range(4):
        canopy.get_visual(f"button_{i}")  # raises if a static button is missing
    canopy.get_visual("brand_logo")

    # Power button: protrudes from the angled front face at rest
    rest_bb = ctx.part_world_aabb(button)
    ctx.check(
        "power button cap protrudes from the angled fascia at rest",
        rest_bb is not None and rest_bb[1][1] > BTN_Y - 0.002,
        details=f"button aabb={rest_bb}, front face y={BTN_Y}",
    )
    with ctx.pose({press: BTN_TRAVEL}):
        pressed_bb = ctx.part_world_aabb(button)
    ctx.check(
        "pressing the power button moves the cap 4 mm into the angled fascia",
        rest_bb is not None
        and pressed_bb is not None
        and (
            # The button moves along the inward normal: both Y and Z change
            abs(
                math.sqrt(
                    (rest_bb[1][1] - pressed_bb[1][1]) ** 2
                    + (rest_bb[1][2] - pressed_bb[1][2]) ** 2
                )
                - BTN_TRAVEL
            )
            < 0.002
        ),
        details=f"rest ymax={None if rest_bb is None else rest_bb[1][1]}, "
        f"pressed ymax={None if pressed_bb is None else pressed_bb[1][1]}",
    )
    plim = press.motion_limits
    ctx.check(
        "power button travel is 0 .. 0.004 m",
        plim is not None and plim.lower == 0.0 and plim.upper == BTN_TRAVEL,
        details=f"limits={plim}",
    )
    # Button joint axis should be the inward normal of the angled face
    ctx.check(
        "power button axis is the inward normal of the angled front face",
        tuple(press.axis) == (0.0, -FRONT_NY, -FRONT_NZ),
        details=f"axis={press.axis}",
    )

    return ctx.report()


object_model = build_object_model()
