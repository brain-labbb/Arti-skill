from __future__ import annotations

"""Wall-mounted T-style (box/linear) kitchen chimney range hood in brushed
stainless steel.

Layout (object frame):
- X: width (canopy 0.90 m), Y: depth (0.50 m, wall plane at y = -0.25),
  Z: up, underside of the canopy bottom plate at z = 0.
- Canopy: hollow stainless shell -- tapered skirt (z 0..0.06) lofting out to a
  0.90 x 0.50 x 0.12 m fascia box (z 0.06..0.18) with an integrated bottom
  plate carrying two recessed LED lamps and a dark slotted grease filter.
- Chimney: fixed lower rectangular duct (0.32 x 0.28 m) rising from the canopy
  top, nested inside a slightly wider telescoping upper sleeve
  (0.336 x 0.296 m) that slides up on a prismatic +Z joint (0..0.35 m).
- Blower fan rotor spins on a continuous vertical joint behind the filter.
- Horizontal slider control on the fascia: tab slides 60 mm (low/med/high).
"""

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
CANOPY_W = 0.90  # canopy width (X)
CANOPY_D = 0.50  # canopy depth (Y)
CANOPY_BOX_H = 0.12  # vertical fascia box height
SKIRT_H = 0.06  # tapered skirt height
SKIRT_BOT_W = 0.66  # skirt bottom opening width
SKIRT_BOT_D = 0.38  # skirt bottom opening depth
PLATE_T = 0.005  # integrated bottom plate thickness
WALL = 0.003  # sheet-metal wall thickness (box region)
CANOPY_TOP = SKIRT_H + CANOPY_BOX_H  # 0.18

FILTER_OPEN_W = 0.42  # filter cutout in the bottom plate
FILTER_OPEN_D = 0.28
FILTER_PANEL_W = 0.46  # filter panel (overlaps the plate rim)
FILTER_PANEL_D = 0.32
FILTER_PANEL_T = 0.004

LAMP_X = 0.26  # lamp centers near the outer ends
LAMP_HOLE_R = 0.035
LAMP_LENS_R = 0.036
LAMP_LENS_T = 0.006

DUCT_Y = -0.10  # chimney center (back face flush with canopy back wall)
DUCT_W = 0.32  # fixed lower duct outer width
DUCT_D = 0.28  # fixed lower duct outer depth
DUCT_WALL = 0.003
DUCT_LEN = 0.892  # lower duct length (z 0.178 .. 1.070)
DUCT_Z0 = CANOPY_TOP - 0.002  # 2 mm embed into the canopy top plate
DUCT_HOLE_W = 0.26  # exhaust hole in the canopy top plate
DUCT_HOLE_D = 0.22

SLEEVE_W = 0.336  # telescoping upper sleeve, slightly wider than the duct
SLEEVE_D = 0.296
SLEEVE_WALL = 0.005
SLEEVE_LEN = 0.45
SLEEVE_Z0 = 0.65  # sleeve seating plane at q=0 (closed top at 1.10 m)
SLIDE_TRAVEL = 0.35
PAD_T = 0.004  # friction guide pads bridging the sleeve/duct clearance
PAD_Z = 0.035  # pad center height in the sleeve local frame
PAD_LEN = 0.08  # pad length along the wall

FAN_Z = 0.045  # rotor mid-plane height
FAN_R = 0.085
FAN_HUB_R = 0.024
FAN_T = 0.016
SHAFT_R = 0.008
SHAFT_LEN = 0.092  # rotor center up into the motor housing
HOUSING_R = 0.10
HOUSING_Z0 = 0.105
HOUSING_Z1 = CANOPY_TOP - 0.001  # 2 mm embed into the canopy top plate

FASCIA_FRONT = CANOPY_D / 2.0  # y = +0.25

# --- horizontal slide control (low / med / high) on the fascia front
SLIDER_TRACK_W = 0.080  # track slot width (horizontal extent on fascia)
SLIDER_TRACK_H = 0.010  # track slot height
SLIDER_TRACK_DEPTH = 0.003  # slot recess into the fascia face
SLIDER_TAB_W = 0.022  # slider tab width
SLIDER_TAB_H = 0.014  # slider tab height
SLIDER_TAB_D = 0.008  # tab protrusion outward from the fascia face
SLIDER_STEM_W = 0.010  # stem width passing through the track slot
SLIDER_STEM_H = 0.008  # stem height
SLIDER_STEM_LEN = 0.006  # stem depth reaching into the fascia
SLIDER_TRAVEL = 0.060  # total slide travel (low → high)
SLIDER_X0 = -SLIDER_TRAVEL / 2.0  # tab center at q=0 (low end)
SLIDER_Z = SKIRT_H + 0.055  # control row height on the fascia
SLIDER_TICK_XS = (SLIDER_X0, 0.0, SLIDER_X0 + SLIDER_TRAVEL)  # low/med/high


# ---------------------------------------------------------------- cq helpers
def _canopy_shell() -> cq.Workplane:
    """Hollow canopy: tapered skirt + fascia box + integrated bottom plate."""
    outer = (
        cq.Workplane("XY")
        .rect(SKIRT_BOT_W, SKIRT_BOT_D)
        .workplane(offset=SKIRT_H)
        .rect(CANOPY_W, CANOPY_D)
        .workplane(offset=CANOPY_BOX_H)
        .rect(CANOPY_W, CANOPY_D)
        .loft(ruled=True)
    )
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=PLATE_T)
        .rect(SKIRT_BOT_W - 0.012, SKIRT_BOT_D - 0.012)
        .workplane(offset=SKIRT_H - PLATE_T + 0.0005)
        .rect(CANOPY_W - 2 * WALL, CANOPY_D - 2 * WALL)
        .workplane(offset=CANOPY_BOX_H - WALL - 0.0005)
        .rect(CANOPY_W - 2 * WALL, CANOPY_D - 2 * WALL)
        .loft(ruled=True)
    )
    shell = outer.cut(cavity)
    # grease-filter opening in the bottom plate
    shell = shell.cut(
        cq.Workplane("XY")
        .workplane(offset=-0.01)
        .rect(FILTER_OPEN_W, FILTER_OPEN_D)
        .extrude(0.03)
    )
    # recessed lamp holes near the outer ends
    for sx in (-1.0, 1.0):
        shell = shell.cut(
            cq.Workplane("XY")
            .workplane(offset=-0.01)
            .center(sx * LAMP_X, 0.0)
            .circle(LAMP_HOLE_R)
            .extrude(0.03)
        )
    # exhaust hole in the top plate, under the chimney duct
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
    model = ArticulatedObject(name="t_style_chimney_range_hood")

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
        mesh_from_cadquery(_canopy_shell(), "canopy_shell"),
        material=stainless,
        name="canopy_shell",
    )
    # fixed lower chimney duct rising from the canopy top
    canopy.visual(
        mesh_from_cadquery(_rect_tube(DUCT_W, DUCT_D, DUCT_WALL, DUCT_LEN), "lower_duct"),
        origin=Origin(xyz=(0.0, DUCT_Y, DUCT_Z0)),
        material=duct_steel,
        name="lower_duct",
    )
    # dark aluminum-mesh grease filter, recessed above the bottom plate
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
    # two recessed round LED lamps near the outer ends
    for i, sx in enumerate((-1.0, 1.0)):
        canopy.visual(
            Cylinder(radius=LAMP_LENS_R, length=LAMP_LENS_T),
            origin=Origin(xyz=(sx * LAMP_X, 0.0, PLATE_T + 0.002)),
            material=lamp_warm,
            name=f"lamp_lens_{i}",
        )
    # blower motor housing hanging from the canopy top plate
    canopy.visual(
        Cylinder(radius=HOUSING_R, length=HOUSING_Z1 - HOUSING_Z0),
        origin=Origin(xyz=(0.0, 0.0, (HOUSING_Z0 + HOUSING_Z1) / 2.0)),
        material=motor_gray,
        name="motor_housing",
    )
    # horizontal slider track slot on the fascia (tab slides along +X)
    canopy.visual(
        Box((SLIDER_TRACK_W, SLIDER_TRACK_DEPTH, SLIDER_TRACK_H)),
        origin=Origin(xyz=(0.0, FASCIA_FRONT - SLIDER_TRACK_DEPTH / 2.0, SLIDER_Z)),
        material=motor_gray,
        name="slider_track",
    )
    # three tick marks above the track indicating low / medium / high
    for i, tx in enumerate(SLIDER_TICK_XS):
        canopy.visual(
            Box((0.004, 0.002, 0.005)),
            origin=Origin(xyz=(tx, FASCIA_FRONT + 0.0005, SLIDER_Z + 0.014)),
            material=motor_gray,
            name=f"slider_tick_{i}",
        )
    # small red brand logo above the slider row
    canopy.visual(
        Box((0.040, 0.002, 0.011)),
        origin=Origin(xyz=(0.0, FASCIA_FRONT + 0.0005, SKIRT_H + 0.088)),
        material=logo_red,
        name="brand_logo",
    )
    # small amber indicator window left of the slider track
    canopy.visual(
        Box((0.008, 0.002, 0.008)),
        origin=Origin(xyz=(-0.055, FASCIA_FRONT + 0.0005, SLIDER_Z)),
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
    # friction guide pads near the sleeve bottom edge: they bridge the sliding
    # clearance and ride on the lower duct walls, carrying the sleeve.
    pad_x = (SLEEVE_W / 2.0 - SLEEVE_WALL + DUCT_W / 2.0) / 2.0  # 0.1615
    pad_y = (SLEEVE_D / 2.0 - SLEEVE_WALL + DUCT_D / 2.0) / 2.0  # 0.1415
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

    # ----------------------------------------------------------- slider tab
    slider = model.part("slider_tab")
    # tab body protruding outward from the fascia face
    slider.visual(
        Box((SLIDER_TAB_W, SLIDER_TAB_D, SLIDER_TAB_H)),
        origin=Origin(xyz=(0.0, SLIDER_TAB_D / 2.0, 0.0)),
        material=button_steel,
        name="slider_cap",
    )
    # stem passing through the track slot into the fascia
    slider.visual(
        Box((SLIDER_STEM_W, SLIDER_STEM_LEN, SLIDER_STEM_H)),
        origin=Origin(xyz=(0.0, -SLIDER_STEM_LEN / 2.0, 0.0)),
        material=motor_gray,
        name="slider_stem",
    )
    model.articulation(
        "canopy_to_slider_tab",
        ArticulationType.PRISMATIC,
        parent=canopy,
        child=slider,
        origin=Origin(xyz=(SLIDER_X0, FASCIA_FRONT + 0.001, SLIDER_Z)),
        axis=(1.0, 0.0, 0.0),  # positive q slides the tab to the right (high)
        motion_limits=MotionLimits(effort=3.0, velocity=0.1, lower=0.0, upper=SLIDER_TRAVEL),
    )

    return model


# --------------------------------------------------------------------- tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    canopy = object_model.get_part("canopy")
    sleeve = object_model.get_part("chimney_sleeve")
    fan = object_model.get_part("blower_fan")
    slider = object_model.get_part("slider_tab")
    slide = object_model.get_articulation("canopy_to_chimney_sleeve")
    spin = object_model.get_articulation("canopy_to_blower_fan")
    slider_joint = object_model.get_articulation("canopy_to_slider_tab")

    # Intentional local embeddings.
    ctx.allow_overlap(
        fan,
        canopy,
        elem_a="fan_shaft",
        elem_b="motor_housing",
        reason="The rotor shaft is intentionally captured inside the blower motor housing.",
    )
    ctx.allow_overlap(
        slider,
        canopy,
        elem_a="slider_stem",
        elem_b="canopy_shell",
        reason="The slider stem passes through the track slot in the fascia like a real slider mechanism.",
    )
    for i in range(4):
        ctx.allow_overlap(
            sleeve,
            canopy,
            elem_a=f"guide_pad_{i}",
            elem_b="lower_duct",
            reason="Friction guide pads grip the lower duct wall to carry the telescoping sleeve.",
        )

    # --- overall canopy envelope: ~0.90 wide, ~0.50 deep, underside at z=0
    bb = ctx.part_world_aabb(canopy)
    ctx.check(
        "canopy footprint is ~0.90 x 0.50 m with its underside at z=0",
        bb is not None
        and abs((bb[1][0] - bb[0][0]) - CANOPY_W) < 0.01
        and abs((bb[1][1] - bb[0][1]) - CANOPY_D) < 0.01
        and abs(bb[0][2]) < 0.002,
        details=f"canopy aabb={bb}",
    )
    sbb = ctx.part_world_aabb(sleeve)
    ctx.check(
        "closed hood tops out at ~1.10 m with the sleeve seated at 0.65 m",
        sbb is not None and abs(sbb[1][2] - 1.10) < 0.01 and abs(sbb[0][2] - SLEEVE_Z0) < 0.01,
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
    ctx.expect_within(
        canopy,
        fan,
        axes="xy",
        inner_elem="filter_mesh_panel",
        outer_elem=None,
        margin=0.20,
        name="fan sits behind the center filter panel",
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

    # --- fascia controls: horizontal slider track with three tick marks
    canopy.get_visual("slider_track")
    for i in range(3):
        canopy.get_visual(f"slider_tick_{i}")  # raises if a tick mark is missing
    canopy.get_visual("brand_logo")
    
    # slider tab protrudes from the fascia and slides horizontally
    rest_bb = ctx.part_world_aabb(slider)
    ctx.check(
        "slider tab protrudes from the fascia at rest (low position)",
        rest_bb is not None and rest_bb[1][1] > FASCIA_FRONT + 0.002,
        details=f"slider aabb={rest_bb}, fascia front y={FASCIA_FRONT}",
    )
    with ctx.pose({slider_joint: SLIDER_TRAVEL}):
        extended_bb = ctx.part_world_aabb(slider)
        ext_pos = ctx.part_world_position(slider)
    rest_pos = ctx.part_world_position(slider)
    ctx.check(
        "sliding the tab moves it 0.06 m to the right (high position)",
        rest_pos is not None
        and ext_pos is not None
        and abs((ext_pos[0] - rest_pos[0]) - SLIDER_TRAVEL) < 1e-6
        and abs(ext_pos[1] - rest_pos[1]) < 1e-6
        and abs(ext_pos[2] - rest_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, extended={ext_pos}",
    )
    slim = slider_joint.motion_limits
    ctx.check(
        "slider joint range is 0 .. 0.06 m (low/med/high)",
        slim is not None and slim.lower == 0.0 and slim.upper == SLIDER_TRAVEL,
        details=f"limits={slim}",
    )
    ctx.check(
        "slider joint is prismatic along +X axis",
        slider_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(slider_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={slider_joint.articulation_type}, axis={slider_joint.axis}",
    )

    return ctx.report()


object_model = build_object_model()
