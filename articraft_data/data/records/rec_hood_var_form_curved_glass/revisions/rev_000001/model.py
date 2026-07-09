from __future__ import annotations

"""Wall-mounted curved glass visor kitchen range hood.

The canopy is a modern curved tempered-glass visor over a slim stainless-steel
body. The glass sweeps from the body's lower front edge downward and outward
in a concave curve, creating the signature visor silhouette.

Layout (object frame):
- X: width (0.90 m), Y: depth (wall at y=-0.25), Z: up
- Slim metal body: 0.90 x 0.20 x 0.14 m at z 0.30..0.44
- Curved glass visor: 8 mm tempered glass, concave curve from
  (y=-0.05, z=0.30) outward to (y=+0.25, z=0.04), 0.90 m wide
- Chimney: fixed lower duct + telescoping upper sleeve (prismatic +Z, 0..0.35 m)
- Blower fan: continuous vertical spin inside the body behind the filter
- Power button: prismatic press into body front face (0..0.004 m)
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
    LoftSection,
    MotionLimits,
    Origin,
    SectionLoftSpec,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    section_loft,
)

# ---------------------------------------------------------------- dimensions
BODY_W = 0.90
BODY_D = 0.20
BODY_H = 0.14
BODY_Z0 = 0.30
BODY_Z1 = BODY_Z0 + BODY_H  # 0.44
BODY_Y_BACK = -0.25
BODY_Y_FRONT = BODY_Y_BACK + BODY_D  # -0.05
BODY_CY = (BODY_Y_BACK + BODY_Y_FRONT) / 2.0  # -0.15
BODY_CZ = (BODY_Z0 + BODY_Z1) / 2.0  # 0.37
WALL_T = 0.003

# Glass visor
GLASS_T = 0.008
GLASS_W = BODY_W
# Concave curve in YZ: (y, z) from body front-bottom outward to visor tip
GLASS_CURVE = [
    (-0.05, 0.30),
    (-0.01, 0.26),
    (0.04, 0.21),
    (0.09, 0.16),
    (0.14, 0.11),
    (0.19, 0.07),
    (0.25, 0.04),
]

# Filter on body underside
FILTER_OPEN_W = 0.42
FILTER_OPEN_D = 0.12
FILTER_PANEL_W = 0.46
FILTER_PANEL_D = 0.15
FILTER_PANEL_T = 0.004

# Lamps
LAMP_X = 0.30
LAMP_HOLE_R = 0.028
LAMP_LENS_R = 0.029
LAMP_LENS_T = 0.005

# Chimney duct
DUCT_W = 0.32
DUCT_D = 0.28
DUCT_WALL = 0.003
DUCT_Y = BODY_CY
DUCT_Z0 = BODY_Z1 - 0.002
DUCT_LEN = 0.66
DUCT_HOLE_W = 0.26
DUCT_HOLE_D = 0.22

# Telescoping sleeve
SLEEVE_W = 0.336
SLEEVE_D = 0.296
SLEEVE_WALL = 0.005
SLEEVE_LEN = 0.42
SLEEVE_Z0 = 0.72
SLIDE_TRAVEL = 0.35
PAD_T = 0.004
PAD_Z = 0.035
PAD_LEN = 0.08

# Blower fan (fits inside the slim body)
FAN_R = 0.050
FAN_HUB_R = 0.018
FAN_T = 0.012
SHAFT_R = 0.006
SHAFT_LEN = 0.065
HOUSING_R = 0.06
FAN_Z = BODY_Z0 + 0.035  # 0.335
HOUSING_Z0 = BODY_Z0 + 0.001  # motor housing contacts the bottom plate
HOUSING_Z1 = BODY_Z1 - 0.002

# Buttons on body front face
BTN_R = 0.006
BTN_LEN = 0.008
BTN_Y = BODY_Y_FRONT + 0.001
BTN_Z = BODY_CZ
BTN_XS = (-0.030, -0.005, 0.020, 0.045)
POWER_BTN_X = 0.070
BTN_TRAVEL = 0.004


# ---------------------------------------------------------------- cq helpers
def _metal_body_shell() -> cq.Workplane:
    """Hollow stainless-steel body with filter opening, lamp holes, and
    exhaust hole. Built in local coords centered at origin."""
    outer = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H)
    cavity = (
        cq.Workplane("XY")
        .box(BODY_W - 2 * WALL_T, BODY_D - 2 * WALL_T, BODY_H - 2 * WALL_T)
    )
    body = outer.cut(cavity)
    # Filter opening in bottom plate
    body = body.cut(
        cq.Workplane("XY")
        .workplane(offset=-BODY_H / 2.0 - 0.01)
        .rect(FILTER_OPEN_W, FILTER_OPEN_D)
        .extrude(WALL_T + 0.02)
    )
    # Recessed lamp holes near outer ends
    for sx in (-1.0, 1.0):
        body = body.cut(
            cq.Workplane("XY")
            .workplane(offset=-BODY_H / 2.0 - 0.01)
            .center(sx * LAMP_X, 0.0)
            .circle(LAMP_HOLE_R)
            .extrude(WALL_T + 0.02)
        )
    # Exhaust hole in top plate under chimney
    body = body.cut(
        cq.Workplane("XY")
        .workplane(offset=BODY_H / 2.0 - 0.01)
        .rect(DUCT_HOLE_W, DUCT_HOLE_D)
        .extrude(0.02)
    )
    return body


def _rect_tube(width: float, depth: float, wall: float, length: float) -> cq.Workplane:
    """Open-ended thin-walled rectangular duct along +Z."""
    outer = cq.Workplane("XY").rect(width, depth).extrude(length)
    inner = (
        cq.Workplane("XY")
        .workplane(offset=-0.01)
        .rect(width - 2 * wall, depth - 2 * wall)
        .extrude(length + 0.02)
    )
    return outer.cut(inner)


def _glass_visor_geometry():
    """Build the curved tempered-glass visor as a section loft through
    oriented rectangular cross-sections that follow the concave YZ curve."""
    hw = GLASS_W / 2.0
    ht = GLASS_T / 2.0
    n = len(GLASS_CURVE)

    raw_sections = []
    for i in range(n):
        y, z = GLASS_CURVE[i]
        # Tangent via central / forward / backward differences
        if i < n - 1:
            dy = GLASS_CURVE[i + 1][0] - y
            dz = GLASS_CURVE[i + 1][1] - z
        else:
            dy = y - GLASS_CURVE[i - 1][0]
            dz = z - GLASS_CURVE[i - 1][1]
        length = math.sqrt(dy * dy + dz * dz)
        ty, tz_dir = dy / length, dz / length

        # Normal perpendicular to tangent in YZ (rotated 90° CCW): (-tz, ty)
        ny, nz = -tz_dir, ty

        # Four corners of the thin rectangular section
        # Outer face offset +ht along normal, inner face -ht
        pts = (
            (-hw, y + ny * ht, z + nz * ht),
            (hw, y + ny * ht, z + nz * ht),
            (hw, y - ny * ht, z - nz * ht),
            (-hw, y - ny * ht, z - nz * ht),
        )
        raw_sections.append(LoftSection(points=tuple(pts)))

    spec = SectionLoftSpec(
        sections=tuple(raw_sections),
        cap=True,
        solid=True,
    )
    return section_loft(spec)


# ----------------------------------------------------------------- the model
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="curved_glass_visor_range_hood")

    stainless = model.material("brushed_stainless", rgba=(0.74, 0.75, 0.77, 1.0))
    duct_steel = model.material("duct_stainless", rgba=(0.70, 0.71, 0.73, 1.0))
    glass_tint = model.material("tempered_glass", rgba=(0.10, 0.13, 0.15, 1.0))
    filter_mesh = model.material("filter_mesh", rgba=(0.20, 0.21, 0.23, 1.0))
    lamp_warm = model.material("lamp_warm", rgba=(1.0, 0.96, 0.80, 1.0))
    motor_gray = model.material("motor_gray", rgba=(0.30, 0.31, 0.33, 1.0))
    fan_gray = model.material("fan_gray", rgba=(0.44, 0.45, 0.48, 1.0))
    button_steel = model.material("button_steel", rgba=(0.86, 0.87, 0.88, 1.0))
    logo_red = model.material("logo_red", rgba=(0.78, 0.10, 0.08, 1.0))
    indicator_amber = model.material("indicator_amber", rgba=(0.95, 0.78, 0.18, 1.0))

    # ---------------------------------------------------------------- canopy
    canopy = model.part("canopy")

    # Slim metal body shell (hollow box, positioned at body center)
    canopy.visual(
        mesh_from_cadquery(_metal_body_shell(), "body_shell"),
        origin=Origin(xyz=(0.0, BODY_CY, BODY_CZ)),
        material=stainless,
        name="body_shell",
    )

    # Curved glass visor (section loft, world-positioned sections)
    canopy.visual(
        mesh_from_geometry(_glass_visor_geometry(), "glass_visor"),
        material=glass_tint,
        name="glass_visor",
    )

    # Fixed lower chimney duct rising from the body top
    canopy.visual(
        mesh_from_cadquery(_rect_tube(DUCT_W, DUCT_D, DUCT_WALL, DUCT_LEN), "lower_duct"),
        origin=Origin(xyz=(0.0, DUCT_Y, DUCT_Z0)),
        material=duct_steel,
        name="lower_duct",
    )

    # Dark aluminum-mesh grease filter recessed above body bottom plate
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
        origin=Origin(xyz=(0.0, BODY_CY, BODY_Z0 + WALL_T + 0.001)),
        material=filter_mesh,
        name="filter_mesh_panel",
    )

    # Two recessed round LED lamps near the outer ends of the body underside
    for i, sx in enumerate((-1.0, 1.0)):
        canopy.visual(
            Cylinder(radius=LAMP_LENS_R, length=LAMP_LENS_T),
            origin=Origin(xyz=(sx * LAMP_X, BODY_CY, BODY_Z0 + WALL_T + 0.002)),
            material=lamp_warm,
            name=f"lamp_lens_{i}",
        )

    # Blower motor housing hanging from body top plate
    canopy.visual(
        Cylinder(radius=HOUSING_R, length=HOUSING_Z1 - HOUSING_Z0),
        origin=Origin(xyz=(0.0, BODY_CY, (HOUSING_Z0 + HOUSING_Z1) / 2.0)),
        material=motor_gray,
        name="motor_housing",
    )

    # Four static control buttons on the body front face
    for i, bx in enumerate(BTN_XS):
        canopy.visual(
            Cylinder(radius=BTN_R, length=BTN_LEN),
            origin=Origin(xyz=(bx, BTN_Y, BTN_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=button_steel,
            name=f"button_{i}",
        )

    # Small red brand logo above the button row
    canopy.visual(
        Box((0.040, 0.002, 0.011)),
        origin=Origin(xyz=(0.0, BODY_Y_FRONT + 0.001, BODY_Z0 + 0.10)),
        material=logo_red,
        name="brand_logo",
    )

    # Small amber indicator window left of the buttons
    canopy.visual(
        Box((0.008, 0.002, 0.008)),
        origin=Origin(xyz=(-0.055, BODY_Y_FRONT + 0.001, BTN_Z)),
        material=indicator_amber,
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
    # Friction guide pads bridging the sleeve/duct clearance
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
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.2, lower=0.0, upper=SLIDE_TRAVEL
        ),
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
        origin=Origin(xyz=(0.0, BODY_CY, FAN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=30.0),
    )

    # ---------------------------------------------------------- power button
    button = model.part("power_button")
    button.visual(
        Cylinder(radius=BTN_R, length=BTN_LEN),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=button_steel,
        name="button_cap",
    )
    model.articulation(
        "canopy_to_power_button",
        ArticulationType.PRISMATIC,
        parent=canopy,
        child=button,
        origin=Origin(xyz=(POWER_BTN_X, BTN_Y, BTN_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=0.05, lower=0.0, upper=BTN_TRAVEL
        ),
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
        elem_a="fan_rotor",
        elem_b="motor_housing",
        reason="The fan rotor is intentionally nested inside the motor housing proxy; the real housing is hollow around the rotor.",
    )
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
        elem_b="body_shell",
        reason="The push-button cap passes through the body front wall like a real button stem.",
    )
    for i in range(4):
        ctx.allow_overlap(
            sleeve,
            canopy,
            elem_a=f"guide_pad_{i}",
            elem_b="lower_duct",
            reason="Friction guide pads grip the lower duct wall to carry the telescoping sleeve.",
        )

    # --- glass visor: curved shape verification
    glass_bb = ctx.part_element_world_aabb(canopy, elem="glass_visor")
    ctx.check(
        "glass visor spans ~0.90 m width",
        glass_bb is not None
        and abs((glass_bb[1][0] - glass_bb[0][0]) - GLASS_W) < 0.02,
        details=f"glass visor aabb={glass_bb}",
    )
    ctx.check(
        "glass visor extends at least 0.25 m in depth (curved profile)",
        glass_bb is not None
        and (glass_bb[1][1] - glass_bb[0][1]) > 0.25,
        details=f"glass visor Y extent={None if glass_bb is None else glass_bb[1][1] - glass_bb[0][1]}",
    )
    ctx.check(
        "glass visor drops at least 0.20 m vertically (curved sweep)",
        glass_bb is not None
        and (glass_bb[1][2] - glass_bb[0][2]) > 0.20,
        details=f"glass visor Z extent={None if glass_bb is None else glass_bb[1][2] - glass_bb[0][2]}",
    )
    if glass_bb is not None:
        glass_extents = sorted(glass_bb[1][i] - glass_bb[0][i] for i in range(3))
        ctx.check(
            "glass visor is a curved panel (not a box block)",
            glass_extents[0] < glass_extents[2] * 0.5,
            details=f"sorted extents={glass_extents}",
        )

    # --- glass visor reaches down near z=0.04 at the front
    ctx.check(
        "glass visor front edge is near z=0.04",
        glass_bb is not None and abs(glass_bb[0][2] - 0.04) < 0.02,
        details=f"glass visor zmin={None if glass_bb is None else glass_bb[0][2]}",
    )

    # --- body shell: slim metal housing
    body_bb = ctx.part_element_world_aabb(canopy, elem="body_shell")
    ctx.check(
        "body shell is ~0.90 m wide, ~0.20 m deep, ~0.14 m tall",
        body_bb is not None
        and abs((body_bb[1][0] - body_bb[0][0]) - BODY_W) < 0.01
        and abs((body_bb[1][1] - body_bb[0][1]) - BODY_D) < 0.01
        and abs((body_bb[1][2] - body_bb[0][2]) - BODY_H) < 0.01,
        details=f"body aabb={body_bb}",
    )

    # --- overall hood height: closed at ~1.14 m
    sbb = ctx.part_world_aabb(sleeve)
    ctx.check(
        "closed hood tops out at ~1.14 m with sleeve seated",
        sbb is not None
        and abs(sbb[1][2] - (SLEEVE_Z0 + SLEEVE_LEN)) < 0.01,
        details=f"sleeve aabb={sbb}",
    )
    ctx.check(
        "upper sleeve is slightly wider than the fixed lower duct",
        sbb is not None and (sbb[1][0] - sbb[0][0]) > DUCT_W + 0.01,
        details=f"sleeve width={None if sbb is None else sbb[1][0] - sbb[0][0]}",
    )

    # --- telescoping chimney: nesting, retained insertion, upward motion
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
        min_overlap=0.25,
        name="closed sleeve overlaps the lower duct by at least 0.25 m",
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
            min_overlap=0.02,
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

    # --- blower fan: continuous vertical spin inside body
    ctx.check(
        "blower fan joint is continuous about the vertical axis",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(spin.axis) == (0.0, 0.0, 1.0)
        and (spin.motion_limits is None or spin.motion_limits.lower is None),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    fbb = ctx.part_world_aabb(fan)
    ctx.check(
        "rotor blades span at least 0.09 m across (visible through filter)",
        fbb is not None
        and (fbb[1][0] - fbb[0][0]) > 0.09
        and (fbb[1][1] - fbb[0][1]) > 0.09,
        details=f"fan aabb={fbb}",
    )
    ctx.expect_gap(
        fan,
        canopy,
        axis="z",
        negative_elem="filter_mesh_panel",
        min_gap=0.005,
        max_gap=0.06,
        name="fan rotor sits just above the grease filter",
    )
    ctx.expect_overlap(
        fan,
        canopy,
        axes="z",
        elem_a="fan_shaft",
        elem_b="motor_housing",
        min_overlap=0.01,
        name="rotor shaft stays engaged in the motor housing",
    )

    # --- recessed lamps near the outer ends on body underside
    for i, sx in enumerate((-1.0, 1.0)):
        lb = ctx.part_element_world_aabb(canopy, elem=f"lamp_lens_{i}")
        ctx.check(
            f"lamp_lens_{i} is recessed near x={sx * LAMP_X:+.2f} on body underside",
            lb is not None
            and abs((lb[0][0] + lb[1][0]) / 2.0 - sx * LAMP_X) < 0.005
            and lb[0][2] > BODY_Z0 - 0.005
            and lb[1][2] < BODY_Z0 + 0.015,
            details=f"lamp aabb={lb}",
        )

    # --- grease filter: dark panel recessed in body underside center
    pb = ctx.part_element_world_aabb(canopy, elem="filter_mesh_panel")
    ctx.check(
        "filter panel is centered on body underside and recessed above body bottom",
        pb is not None
        and abs((pb[0][0] + pb[1][0]) / 2.0) < 0.005
        and pb[0][2] > BODY_Z0 - 0.005
        and pb[1][2] < BODY_Z0 + 0.015,
        details=f"filter aabb={pb}",
    )

    # --- fascia controls: five buttons total, power button presses inward
    for i in range(4):
        canopy.get_visual(f"button_{i}")
    canopy.get_visual("brand_logo")
    rest_bb = ctx.part_world_aabb(button)
    ctx.check(
        "power button cap protrudes from body front face at rest",
        rest_bb is not None and rest_bb[1][1] > BODY_Y_FRONT - 0.002,
        details=f"button aabb={rest_bb}, body front y={BODY_Y_FRONT}",
    )
    with ctx.pose({press: BTN_TRAVEL}):
        pressed_bb = ctx.part_world_aabb(button)
    ctx.check(
        "pressing the power button moves the cap 4 mm into the body",
        rest_bb is not None
        and pressed_bb is not None
        and abs((rest_bb[1][1] - pressed_bb[1][1]) - BTN_TRAVEL) < 1e-6,
        details=f"rest ymax={None if rest_bb is None else rest_bb[1][1]}, "
        f"pressed ymax={None if pressed_bb is None else pressed_bb[1][1]}",
    )
    plim = press.motion_limits
    ctx.check(
        "power button travel is 0 .. 0.004 m",
        plim is not None and plim.lower == 0.0 and plim.upper == BTN_TRAVEL,
        details=f"limits={plim}",
    )

    return ctx.report()


object_model = build_object_model()
