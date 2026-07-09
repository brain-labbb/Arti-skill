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
  top, nested inside a dual-stage telescoping chimney -- two nested sleeves
  (sleeve_0: 0.336 x 0.296 m, sleeve_1: 0.352 x 0.312 m), each sliding on a
  prismatic +Z joint (0..0.35 m per stage) emitted via a for-i-in-range(2) loop.
- Blower fan rotor spins on a continuous vertical joint behind the filter.
- The rightmost (power) push button presses 4 mm into the fascia.
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

# Dual-stage telescoping chimney: each tuple is (outer_w, outer_d, wall, length).
# Stage 0 slides over the fixed duct; stage 1 slides over stage 0.
SLEEVE_DIMS = [
    (0.336, 0.296, 0.005, 0.45),  # sleeve_0 (inner stage)
    (0.352, 0.312, 0.005, 0.45),  # sleeve_1 (outer stage)
]
SLEEVE_Z0 = 0.65  # rest seating Z for the bottom of each sleeve (closed top at 1.10 m)
SLIDE_TRAVEL = 0.35  # per-stage travel (m)
PAD_T = 0.004  # friction guide pads bridging the sleeve/parent clearance
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
BTN_R = 0.006
BTN_LEN = 0.008
BTN_Y = FASCIA_FRONT + 0.0005  # cap mid-plane: spans y 0.2465 .. 0.2545
BTN_Z = SKIRT_H + 0.055  # button row height on the fascia
BTN_XS = (-0.030, -0.005, 0.020, 0.045)  # four static control buttons
POWER_BTN_X = 0.070  # rightmost button is the articulated power button
BTN_TRAVEL = 0.004


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


def _build_sleeve_stage(
    model: ArticulatedObject,
    stage_idx: int,
    parent_part,
    parent_outer_w: float,
    parent_outer_d: float,
    sleeve_material,
    pad_material,
    joint_name: str,
    joint_origin: Origin,
):
    """Build one telescoping sleeve stage with guide pads and a prismatic +Z joint.

    Each sleeve is a thin-walled open-ended rectangular tube along local +Z.
    Four friction guide pads near the bottom edge bridge the sliding clearance
    between this sleeve's inner wall and the parent's outer wall.
    """
    sw, sd, swall, slen = SLEEVE_DIMS[stage_idx]
    sleeve = model.part(f"sleeve_{stage_idx}")
    sleeve.visual(
        mesh_from_cadquery(
            _rect_tube(sw, sd, swall, slen), f"sleeve_shell_{stage_idx}"
        ),
        material=sleeve_material,
        name=f"sleeve_shell_{stage_idx}",
    )
    # Guide pads: centered between the sleeve inner wall and parent outer wall.
    inner_w = sw - 2 * swall
    inner_d = sd - 2 * swall
    pad_x = (inner_w / 2.0 + parent_outer_w / 2.0) / 2.0
    pad_y = (inner_d / 2.0 + parent_outer_d / 2.0) / 2.0
    pad_specs = (
        ((pad_x, 0.0, PAD_Z), (PAD_T, PAD_LEN, 0.03)),
        ((-pad_x, 0.0, PAD_Z), (PAD_T, PAD_LEN, 0.03)),
        ((0.0, pad_y, PAD_Z), (PAD_LEN, PAD_T, 0.03)),
        ((0.0, -pad_y, PAD_Z), (PAD_LEN, PAD_T, 0.03)),
    )
    for j, (pxyz, psize) in enumerate(pad_specs):
        sleeve.visual(
            Box(psize),
            origin=Origin(xyz=pxyz),
            material=pad_material,
            name=f"guide_pad_{stage_idx}_{j}",
        )
    model.articulation(
        joint_name,
        ArticulationType.PRISMATIC,
        parent=parent_part,
        child=sleeve,
        origin=joint_origin,
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.2, lower=0.0, upper=SLIDE_TRAVEL
        ),
    )
    return sleeve


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
    # four static control buttons on the fascia (power button is articulated)
    for i, bx in enumerate(BTN_XS):
        canopy.visual(
            Cylinder(radius=BTN_R, length=BTN_LEN),
            origin=Origin(xyz=(bx, BTN_Y, BTN_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=button_steel,
            name=f"button_{i}",
        )
    # small red brand logo above the button row
    canopy.visual(
        Box((0.040, 0.002, 0.011)),
        origin=Origin(xyz=(0.0, FASCIA_FRONT + 0.0005, SKIRT_H + 0.088)),
        material=logo_red,
        name="brand_logo",
    )
    # small amber indicator window left of the buttons
    canopy.visual(
        Box((0.008, 0.002, 0.008)),
        origin=Origin(xyz=(-0.055, FASCIA_FRONT + 0.0005, BTN_Z)),
        material=amber,
        name="indicator_lamp",
    )

    # ------------------------------------ dual-stage telescoping chimney
    # Build two nested sleeve stages via a for-i-in-range(2) loop.
    # Stage 0 (inner) slides over the fixed lower duct; stage 1 (outer) slides
    # over stage 0.  Each stage has its own prismatic +Z joint with 0..0.35 m
    # travel.  Sleeve materials vary slightly by stage for visual distinction.
    sleeve_materials = [
        model.material("sleeve_inner_steel", rgba=(0.72, 0.73, 0.75, 1.0)),
        model.material("sleeve_outer_steel", rgba=(0.76, 0.77, 0.79, 1.0)),
    ]
    sleeves = []
    slide_joints = []
    # Parent geometry for each stage: (part_ref, outer_w, outer_d)
    stage_parents = [
        (canopy, DUCT_W, DUCT_D),  # stage 0 rides on the fixed duct
    ]
    for i in range(2):
        if i == 0:
            parent_part, parent_ow, parent_od = stage_parents[0]
            jname = "canopy_to_sleeve_0"
            jorigin = Origin(xyz=(0.0, DUCT_Y, SLEEVE_Z0))
        else:
            parent_part = sleeves[i - 1]
            parent_ow = SLEEVE_DIMS[i - 1][0]
            parent_od = SLEEVE_DIMS[i - 1][1]
            jname = f"sleeve_{i - 1}_to_sleeve_{i}"
            # At q=0, sleeve_1 coincides with sleeve_0's origin.
            jorigin = Origin(xyz=(0.0, 0.0, 0.0))
        s = _build_sleeve_stage(
            model,
            stage_idx=i,
            parent_part=parent_part,
            parent_outer_w=parent_ow,
            parent_outer_d=parent_od,
            sleeve_material=sleeve_materials[i],
            pad_material=motor_gray,
            joint_name=jname,
            joint_origin=jorigin,
        )
        sleeves.append(s)
        slide_joints.append(jname)

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
        axis=(0.0, -1.0, 0.0),  # positive q presses the cap into the fascia
        motion_limits=MotionLimits(effort=3.0, velocity=0.05, lower=0.0, upper=BTN_TRAVEL),
    )

    return model


# --------------------------------------------------------------------- tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    canopy = object_model.get_part("canopy")
    sleeves = [object_model.get_part(f"sleeve_{i}") for i in range(2)]
    fan = object_model.get_part("blower_fan")
    button = object_model.get_part("power_button")
    slide_0 = object_model.get_articulation("canopy_to_sleeve_0")
    slide_1 = object_model.get_articulation("sleeve_0_to_sleeve_1")
    spin = object_model.get_articulation("canopy_to_blower_fan")
    press = object_model.get_articulation("canopy_to_power_button")

    # ---- intentional local embeddings
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
        reason="The push-button cap passes through the fascia wall like a real button stem.",
    )
    # Stage-0 guide pads embed into the fixed lower duct wall.
    for j in range(4):
        ctx.allow_overlap(
            sleeves[0],
            canopy,
            elem_a=f"guide_pad_0_{j}",
            elem_b="lower_duct",
            reason="Stage-0 friction guide pads grip the lower duct wall to carry the inner sleeve.",
        )
    # Stage-1 guide pads embed into sleeve_0's outer shell wall.
    for j in range(4):
        ctx.allow_overlap(
            sleeves[1],
            sleeves[0],
            elem_a=f"guide_pad_1_{j}",
            elem_b="sleeve_shell_0",
            reason="Stage-1 friction guide pads grip the stage-0 outer shell to carry the outer sleeve.",
        )
    # Concentric nesting: at rest, both sleeve shells overlap in the same Z range.
    ctx.allow_overlap(
        sleeves[0],
        sleeves[1],
        elem_a="sleeve_shell_0",
        elem_b="sleeve_shell_1",
        reason="Dual-stage telescoping sleeves are concentrically nested at rest.",
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

    # --- dual-stage chimney: both sleeves exist and are concentrically nested at rest
    for i in range(2):
        sw, sd, _, slen = SLEEVE_DIMS[i]
        sbb = ctx.part_world_aabb(sleeves[i])
        ctx.check(
            f"sleeve_{i} shell is {sw:.3f} x {sd:.3f} m at rest",
            sbb is not None
            and abs((sbb[1][0] - sbb[0][0]) - sw) < 0.01
            and abs((sbb[1][1] - sbb[0][1]) - sd) < 0.01,
            details=f"sleeve_{i} aabb={sbb}",
        )
    # Outer sleeve is wider than inner sleeve
    sbb0 = ctx.part_world_aabb(sleeves[0])
    sbb1 = ctx.part_world_aabb(sleeves[1])
    ctx.check(
        "sleeve_1 (outer) is wider than sleeve_0 (inner)",
        sbb0 is not None
        and sbb1 is not None
        and (sbb1[1][0] - sbb1[0][0]) > (sbb0[1][0] - sbb0[1][0]) + 0.01,
        details=f"sleeve_0 width={None if sbb0 is None else sbb0[1][0] - sbb0[0][0]}, "
        f"sleeve_1 width={None if sbb1 is None else sbb1[1][0] - sbb1[0][0]}",
    )
    # Inner sleeve is wider than the fixed duct
    ctx.check(
        "sleeve_0 (inner) is slightly wider than the fixed lower duct",
        sbb0 is not None and (sbb0[1][0] - sbb0[0][0]) > DUCT_W + 0.01,
        details=f"sleeve_0 width={None if sbb0 is None else sbb0[1][0] - sbb0[0][0]}",
    )

    # --- stage-0: nesting on duct, retained insertion, upward motion
    ctx.expect_within(
        canopy,
        sleeves[0],
        axes="xy",
        inner_elem="lower_duct",
        outer_elem="sleeve_shell_0",
        margin=0.0,
        name="lower duct stays nested inside sleeve_0 footprint",
    )
    ctx.expect_overlap(
        canopy,
        sleeves[0],
        axes="z",
        elem_a="lower_duct",
        elem_b="sleeve_shell_0",
        min_overlap=0.30,
        name="closed sleeve_0 overlaps the lower duct by at least 0.30 m",
    )
    ctx.expect_contact(
        sleeves[0],
        canopy,
        elem_a="guide_pad_0_0",
        elem_b="lower_duct",
        name="sleeve_0 guide pads ride on the lower duct wall",
    )
    rest0 = ctx.part_world_position(sleeves[0])
    with ctx.pose({slide_0: SLIDE_TRAVEL}):
        ctx.expect_within(
            canopy,
            sleeves[0],
            axes="xy",
            inner_elem="lower_duct",
            outer_elem="sleeve_shell_0",
            margin=0.0,
            name="extended sleeve_0 stays centered over the lower duct",
        )
        ctx.expect_overlap(
            canopy,
            sleeves[0],
            axes="z",
            elem_a="lower_duct",
            elem_b="sleeve_shell_0",
            min_overlap=0.05,
            name="fully extended sleeve_0 retains insertion on the lower duct",
        )
        ext0 = ctx.part_world_position(sleeves[0])
    ctx.check(
        "sleeve_0 prismatic joint extends straight up by 0.35 m",
        rest0 is not None
        and ext0 is not None
        and abs((ext0[2] - rest0[2]) - SLIDE_TRAVEL) < 1e-6
        and abs(ext0[0] - rest0[0]) < 1e-6
        and abs(ext0[1] - rest0[1]) < 1e-6,
        details=f"rest={rest0}, extended={ext0}",
    )

    # --- stage-1: nesting on sleeve_0, retained insertion, upward motion
    ctx.expect_within(
        sleeves[0],
        sleeves[1],
        axes="xy",
        inner_elem="sleeve_shell_0",
        outer_elem="sleeve_shell_1",
        margin=0.0,
        name="sleeve_0 stays nested inside sleeve_1 footprint",
    )
    ctx.expect_contact(
        sleeves[1],
        sleeves[0],
        elem_a="guide_pad_1_0",
        elem_b="sleeve_shell_0",
        name="sleeve_1 guide pads ride on the sleeve_0 outer wall",
    )
    rest1 = ctx.part_world_position(sleeves[1])
    with ctx.pose({slide_1: SLIDE_TRAVEL}):
        ctx.expect_within(
            sleeves[0],
            sleeves[1],
            axes="xy",
            inner_elem="sleeve_shell_0",
            outer_elem="sleeve_shell_1",
            margin=0.0,
            name="extended sleeve_1 stays centered over sleeve_0",
        )
        ext1 = ctx.part_world_position(sleeves[1])
    ctx.check(
        "sleeve_1 prismatic joint extends straight up by 0.35 m relative to sleeve_0",
        rest1 is not None
        and ext1 is not None
        and abs((ext1[2] - rest1[2]) - SLIDE_TRAVEL) < 1e-6
        and abs(ext1[0] - rest1[0]) < 1e-6
        and abs(ext1[1] - rest1[1]) < 1e-6,
        details=f"rest={rest1}, extended={ext1}",
    )

    # --- dual-stage combined: both joints extend, chimney rises by 2 * travel
    rest1_world = ctx.part_world_position(sleeves[1])
    with ctx.pose({slide_0: SLIDE_TRAVEL, slide_1: SLIDE_TRAVEL}):
        full_ext1 = ctx.part_world_position(sleeves[1])
    ctx.check(
        "dual-stage full extension raises sleeve_1 by 2 * 0.35 m above rest",
        rest1_world is not None
        and full_ext1 is not None
        and abs((full_ext1[2] - rest1_world[2]) - 2 * SLIDE_TRAVEL) < 1e-6,
        details=f"rest={rest1_world}, full_ext={full_ext1}",
    )

    # --- joint range checks for both stages
    for i, jnt in enumerate([slide_0, slide_1]):
        lim = jnt.motion_limits
        ctx.check(
            f"sleeve_{i} joint range is 0 .. {SLIDE_TRAVEL} m",
            lim is not None and lim.lower == 0.0 and lim.upper == SLIDE_TRAVEL,
            details=f"joint={jnt.name}, limits={lim}",
        )
        ctx.check(
            f"sleeve_{i} joint is prismatic about +Z",
            jnt.articulation_type == ArticulationType.PRISMATIC
            and tuple(jnt.axis) == (0.0, 0.0, 1.0),
            details=f"type={jnt.articulation_type}, axis={jnt.axis}",
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

    # --- fascia controls: five buttons total, power button presses inward
    for i in range(4):
        canopy.get_visual(f"button_{i}")  # raises if a static button is missing
    canopy.get_visual("brand_logo")
    rest_bb = ctx.part_world_aabb(button)
    ctx.check(
        "power button cap protrudes from the fascia at rest",
        rest_bb is not None and rest_bb[1][1] > FASCIA_FRONT + 0.002,
        details=f"button aabb={rest_bb}, fascia front y={FASCIA_FRONT}",
    )
    with ctx.pose({press: BTN_TRAVEL}):
        pressed_bb = ctx.part_world_aabb(button)
    ctx.check(
        "pressing the power button moves the cap 4 mm into the fascia",
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
