from __future__ import annotations

"""Stylized low-poly countertop drawer microwave oven.

Layout (world frame):
- x: width (0.52 m), y: depth (0.38 m, front face toward -Y), z: up (0.32 m total).
- Root `body`: chamfered pale blue-gray shell, hollow dark cooking cavity opening
  at the front-left three quarters, inset charcoal control panel on the right
  quarter, four dark block feet grounding the unit at z=0, guide rails on the
  inner cavity walls for the drawer slide.
- `drawer`: charcoal framed front panel with recessed near-black window and a
  proud full-height pale handle bar, plus a flat dark tray/platform extending
  backward into the cavity; prismatic slide along -Y, pulling out 0..0.22 m.
- `turntable`: glass plate on a drive hub with three coupler ribs, continuous
  spin about the vertical axis, riding on the drawer tray.
"""

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ----------------------------------------------------------------------------
# Global dimensions (meters)
# ----------------------------------------------------------------------------
BODY_W = 0.52
BODY_D = 0.38
TOTAL_H = 0.32

FOOT_H = 0.022
FOOT_SIZE = (0.06, 0.05, FOOT_H + 0.0005)  # tiny embed into the shell bottom
SHELL_H = TOTAL_H - FOOT_H  # 0.298
SHELL_CZ = FOOT_H + SHELL_H / 2.0  # 0.171
FRONT_Y = -BODY_D / 2.0  # -0.19

# Cooking cavity (cut through the front face, behind the drawer)
CAV_W = 0.32
CAV_H = 0.20
CAV_X = -0.04
CAV_Z0 = 0.07
CAV_Z1 = CAV_Z0 + CAV_H  # 0.27
CAV_BACK_Y = 0.09
LINER_T = 0.004

# Drawer front panel (covers left ~3/4 of the front face)
PANEL_W = 0.38
PANEL_H = 0.255
PANEL_T = 0.022
PANEL_CZ = 0.1725  # panel center Z (matches original door height)

# Drawer tray / flatbed platform
TRAY_W = 0.28
TRAY_DEPTH = 0.28
TRAY_T = 0.006
# Tray center Z in drawer local frame (places tray above cavity floor liner)
TRAY_LOCAL_Z = CAV_Z0 + 0.015 - PANEL_CZ  # ≈ -0.0815; tray bottom ~0.085 world Z

# Slide travel
SLIDE_TRAVEL = 0.22

# Guide rails on cavity walls (visible slide mechanism detail)
RAIL_W = 0.008
RAIL_H = 0.012
RAIL_DEPTH = 0.26
RAIL_CZ = 0.15  # rail center Z, well above the tray and turntable

# Control panel strip (right quarter, inset into the front face)
CPANEL_CX = 0.20
CPANEL_W = 0.088
CPANEL_H = 0.253

# Turntable (rides on drawer tray)
TT_Y_LOCAL = TRAY_DEPTH / 2.0  # centered on tray depth in drawer frame
HUB_R = 0.032
HUB_H = 0.012
PLATE_R = 0.13
PLATE_T = 0.008

# ----------------------------------------------------------------------------
# Materials
# ----------------------------------------------------------------------------
BODY_BLUE = Material(name="body_pale_blue_gray", rgba=(0.70, 0.76, 0.80, 1.0))
CHARCOAL = Material(name="charcoal", rgba=(0.17, 0.18, 0.20, 1.0))
NEAR_BLACK = Material(name="near_black", rgba=(0.055, 0.055, 0.065, 1.0))
CAVITY_DARK = Material(name="cavity_dark", rgba=(0.09, 0.09, 0.10, 1.0))
FOOT_GRAY = Material(name="foot_dark_gray", rgba=(0.21, 0.22, 0.24, 1.0))
PLATE_GLASS = Material(name="turntable_glass", rgba=(0.60, 0.64, 0.66, 1.0))
HUB_GRAY = Material(name="hub_gray", rgba=(0.25, 0.26, 0.28, 1.0))
RAIL_STEEL = Material(name="rail_steel", rgba=(0.45, 0.47, 0.50, 1.0))


def _shell_solid() -> cq.Workplane:
    """Chamfered outer shell with cavity opening and control-panel recess."""
    shell = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, SHELL_H)
        .translate((0.0, 0.0, SHELL_CZ))
        .edges()
        .chamfer(0.012)
    )
    # Cooking cavity: open at the front face, closed at the back wall.
    # The cut extends well past the front face to ensure a clean opening.
    cavity = (
        cq.Workplane("XY")
        .box(CAV_W, 0.40, CAV_H)
        .translate((CAV_X, -0.11, (CAV_Z0 + CAV_Z1) / 2.0))
    )
    shell = shell.cut(cavity)
    # Shallow recess for the control panel strip.
    recess = (
        cq.Workplane("XY")
        .box(CPANEL_W + 0.002, 0.012, CPANEL_H + 0.002)
        .translate((CPANEL_CX, FRONT_Y, PANEL_CZ))
    )
    shell = shell.cut(recess)
    return shell


def _drawer_front_solid() -> cq.Workplane:
    """Drawer front panel in the drawer local frame.

    The drawer frame origin is at the articulation point (center of the panel
    at the body front face).  The panel extends symmetrically in Z and forward
    in -Y from the origin.
    """
    panel_cy = -PANEL_T / 2.0
    frame = (
        cq.Workplane("XY")
        .box(PANEL_W, PANEL_T, PANEL_H)
        .translate((0.0, panel_cy, 0.0))
        .edges("|Y")
        .chamfer(0.003)
    )
    # Window pocket recessed into the front face (~0.009 deep).
    pocket_cy = -PANEL_T + 0.0045
    pocket = (
        cq.Workplane("XY")
        .box(0.30, 0.009, 0.18)
        .translate((0.0, pocket_cy, 0.0))
    )
    return frame.cut(pocket)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drawer_microwave_oven")

    # ------------------------------------------------------------------ body
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_shell_solid(), "microwave_shell"),
        material=BODY_BLUE,
        name="outer_shell",
    )

    # Dark cavity liner panels (slightly embedded into the cut shell walls).
    liner_y_c = -0.052
    liner_d = 0.276
    body.visual(
        Box((0.318, liner_d, LINER_T)),
        origin=Origin(xyz=(CAV_X, liner_y_c, CAV_Z0 + LINER_T / 2.0 - 0.0005)),
        material=CAVITY_DARK,
        name="cavity_floor",
    )
    body.visual(
        Box((0.318, liner_d, LINER_T)),
        origin=Origin(xyz=(CAV_X, liner_y_c, CAV_Z1 - LINER_T / 2.0 + 0.0005)),
        material=CAVITY_DARK,
        name="cavity_ceiling",
    )
    body.visual(
        Box((0.318, LINER_T, 0.198)),
        origin=Origin(xyz=(CAV_X, CAV_BACK_Y - LINER_T / 2.0 + 0.0005, 0.17)),
        material=CAVITY_DARK,
        name="cavity_back_wall",
    )
    body.visual(
        Box((LINER_T, liner_d, 0.198)),
        origin=Origin(xyz=(CAV_X - CAV_W / 2.0 + LINER_T / 2.0 - 0.0005, liner_y_c, 0.17)),
        material=CAVITY_DARK,
        name="cavity_left_wall",
    )
    body.visual(
        Box((LINER_T, liner_d, 0.198)),
        origin=Origin(xyz=(CAV_X + CAV_W / 2.0 - LINER_T / 2.0 + 0.0005, liner_y_c, 0.17)),
        material=CAVITY_DARK,
        name="cavity_right_wall",
    )

    # Guide rails on the inner cavity walls (slide mechanism detail).
    rail_y_c = (FRONT_Y + CAV_BACK_Y) / 2.0  # ≈ -0.05
    for i, x_sign in enumerate([-1.0, 1.0]):
        rail_cx = CAV_X + x_sign * (CAV_W / 2.0 - LINER_T - RAIL_W / 2.0 + 0.0005)
        body.visual(
            Box((RAIL_W, RAIL_DEPTH, RAIL_H)),
            origin=Origin(xyz=(rail_cx, rail_y_c, RAIL_CZ)),
            material=RAIL_STEEL,
            name=f"guide_rail_{i}",
        )

    # Inset charcoal control panel on the right quarter of the front face.
    body.visual(
        Box((CPANEL_W, 0.004, CPANEL_H)),
        origin=Origin(xyz=(CPANEL_CX, FRONT_Y + 0.0045, PANEL_CZ)),
        material=CHARCOAL,
        name="control_panel",
    )

    # Four dark block feet, grounding the unit at z=0.
    for i, (fx, fy) in enumerate(
        [(-0.20, -0.135), (0.20, -0.135), (-0.20, 0.135), (0.20, 0.135)]
    ):
        body.visual(
            Box(FOOT_SIZE),
            origin=Origin(xyz=(fx, fy, FOOT_SIZE[2] / 2.0)),
            material=FOOT_GRAY,
            name=f"foot_{i}",
        )

    # ---------------------------------------------------------------- drawer
    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_drawer_front_solid(), "drawer_front_panel"),
        material=CHARCOAL,
        name="drawer_frame",
    )
    # Near-black glass window seated at the bottom of the recess pocket.
    drawer.visual(
        Box((0.298, 0.004, 0.178)),
        origin=Origin(xyz=(0.0, -PANEL_T + 0.0105, 0.0)),
        material=NEAR_BLACK,
        name="window_glass",
    )
    # Full-height slim handle bar, proud of the drawer face on two standoffs.
    bar_x = PANEL_W / 2.0 - 0.028
    bar_y = -PANEL_T - 0.019
    drawer.visual(
        Box((0.018, 0.014, PANEL_H)),
        origin=Origin(xyz=(bar_x, bar_y, 0.0)),
        material=BODY_BLUE,
        name="handle_bar",
    )
    for i, dz in enumerate([0.108, -0.108]):
        drawer.visual(
            Box((0.018, 0.013, 0.024)),
            origin=Origin(xyz=(bar_x, -PANEL_T - 0.006, dz)),
            material=BODY_BLUE,
            name=f"handle_standoff_{i}",
        )
    # Flat tray / platform extending backward into the cavity.
    drawer.visual(
        Box((TRAY_W, TRAY_DEPTH, TRAY_T)),
        origin=Origin(xyz=(0.0, TRAY_DEPTH / 2.0, TRAY_LOCAL_Z)),
        material=CAVITY_DARK,
        name="drawer_tray",
    )
    # Thin tray lip edges (visual slide-rail contact strips on tray sides).
    for i, x_sign in enumerate([-1.0, 1.0]):
        lip_x = x_sign * (TRAY_W / 2.0 - 0.003)
        drawer.visual(
            Box((0.006, TRAY_DEPTH, 0.014)),
            origin=Origin(xyz=(lip_x, TRAY_DEPTH / 2.0, TRAY_LOCAL_Z + 0.004)),
            material=RAIL_STEEL,
            name=f"tray_lip_{i}",
        )

    # Drawer prismatic slide: axis toward -Y (pull toward user), 0..0.22 m.
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(CAV_X, FRONT_Y, PANEL_CZ)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0, velocity=0.5, lower=0.0, upper=SLIDE_TRAVEL
        ),
    )

    # ------------------------------------------------------------- turntable
    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=HUB_R, length=HUB_H),
        origin=Origin(xyz=(0.0, 0.0, HUB_H / 2.0)),
        material=HUB_GRAY,
        name="drive_hub",
    )
    # Three one-sided coupler ribs (off-axis features proving rotation).
    for k in range(3):
        ang = k * 2.0 * math.pi / 3.0
        r_c = 0.026
        turntable.visual(
            Box((0.04, 0.012, 0.008)),
            origin=Origin(
                xyz=(r_c * math.cos(ang), r_c * math.sin(ang), 0.008),
                rpy=(0.0, 0.0, ang),
            ),
            material=HUB_GRAY,
            name=f"coupler_rib_{k}",
        )
    # Thin circular glass plate resting on the hub.
    turntable.visual(
        Cylinder(radius=PLATE_R, length=PLATE_T),
        origin=Origin(xyz=(0.0, 0.0, HUB_H + PLATE_T / 2.0)),
        material=PLATE_GLASS,
        name="glass_plate",
    )

    # Turntable rides on the drawer tray (parent is drawer, not body).
    tt_origin_z = TRAY_LOCAL_Z + TRAY_T / 2.0
    model.articulation(
        "turntable_spin",
        ArticulationType.CONTINUOUS,
        parent=drawer,
        child=turntable,
        origin=Origin(xyz=(0.0, TT_Y_LOCAL, tt_origin_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    turntable = object_model.get_part("turntable")
    slide = object_model.get_articulation("drawer_slide")
    spin = object_model.get_articulation("turntable_spin")

    # ---- overall size and grounding -------------------------------------
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "body footprint matches ~0.52 x 0.38 m and total height ~0.32 m",
        body_aabb is not None
        and abs((body_aabb[1][0] - body_aabb[0][0]) - BODY_W) < 0.02
        and abs((body_aabb[1][1] - body_aabb[0][1]) - BODY_D) < 0.02
        and abs(body_aabb[1][2] - TOTAL_H) < 0.01,
        details=f"body aabb={body_aabb}",
    )
    ctx.check(
        "feet ground the unit at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-6,
        details=f"zmin={None if body_aabb is None else body_aabb[0][2]}",
    )

    # ---- drawer slide joint: prismatic, Y axis, 0..0.22 m range --------
    ctx.check(
        "drawer_slide is prismatic along -Y with 0..0.22 m travel",
        slide.articulation_type == ArticulationType.PRISMATIC
        and abs(slide.axis[1] - (-1.0)) < 1e-9
        and abs(slide.axis[0]) < 1e-9
        and abs(slide.axis[2]) < 1e-9
        and slide.motion_limits is not None
        and abs(slide.motion_limits.lower) < 1e-9
        and abs(slide.motion_limits.upper - SLIDE_TRAVEL) < 1e-6,
        details=f"axis={slide.axis}, limits={slide.motion_limits}",
    )

    # ---- closed drawer: front panel seated at the body front face -------
    drawer_aabb = ctx.part_world_aabb(drawer)
    ctx.check(
        "drawer front panel covers the left ~3/4 of the front face",
        drawer_aabb is not None
        and drawer_aabb[0][0] < CAV_X - CAV_W / 2.0 + 0.02
        and drawer_aabb[1][0] > CAV_X + CAV_W / 2.0 - 0.02,
        details=f"drawer aabb={drawer_aabb}",
    )
    # The drawer front panel back face contacts the body front face.
    # Element-level check: front panel sits at the body front face.
    frame_aabb = ctx.part_element_world_aabb(drawer, elem="drawer_frame")
    ctx.check(
        "closed drawer front panel seats at the body front face",
        frame_aabb is not None
        and abs(frame_aabb[1][1] - FRONT_Y) < 0.005,
        details=f"frame aabb={frame_aabb}, FRONT_Y={FRONT_Y}",
    )

    # Handle bar stands proud of the drawer front face.
    bar_aabb = ctx.part_element_world_aabb(drawer, elem="handle_bar")
    ctx.check(
        "handle bar stands proud of the drawer face and spans full panel height",
        bar_aabb is not None
        and bar_aabb[0][1] < FRONT_Y - PANEL_T - 0.010
        and (bar_aabb[1][2] - bar_aabb[0][2]) > 0.24,
        details=f"handle aabb={bar_aabb}",
    )
    # Window glass is recessed behind the drawer front face.
    glass_aabb = ctx.part_element_world_aabb(drawer, elem="window_glass")
    ctx.check(
        "window glass is recessed behind the drawer face",
        glass_aabb is not None and glass_aabb[0][1] > FRONT_Y - PANEL_T + 0.002,
        details=f"glass aabb={glass_aabb}",
    )

    # Control panel sits to the right of the drawer, inset into the front face.
    panel_aabb = ctx.part_element_world_aabb(body, elem="control_panel")
    ctx.check(
        "control panel strip sits right of the drawer and inset into the face",
        panel_aabb is not None
        and drawer_aabb is not None
        and panel_aabb[0][0] > drawer_aabb[1][0] - 0.005
        and panel_aabb[0][1] > FRONT_Y + 0.001,
        details=f"panel aabb={panel_aabb}",
    )

    # ---- drawer tray stays within cavity when closed ---------------------
    tray_aabb = ctx.part_element_world_aabb(drawer, elem="drawer_tray")
    ctx.check(
        "drawer tray fits within cavity opening width",
        tray_aabb is not None
        and tray_aabb[0][0] > CAV_X - CAV_W / 2.0
        and tray_aabb[1][0] < CAV_X + CAV_W / 2.0,
        details=f"tray aabb={tray_aabb}",
    )
    ctx.check(
        "drawer tray is above cavity floor when closed",
        tray_aabb is not None
        and tray_aabb[0][2] > CAV_Z0 + LINER_T - 0.002,
        details=f"tray zmin={None if tray_aabb is None else tray_aabb[0][2]}",
    )

    # ---- drawer tray nested inside cavity shell --------------------------
    # The shell mesh encloses the cavity void; the tray slides inside it.
    # This is an intentional nested-slider fit with a solid-proxy outer member.
    ctx.allow_overlap(
        body, drawer,
        elem_a="outer_shell",
        elem_b="drawer_tray",
        reason="The drawer tray slides inside the cooking cavity; the shell mesh encloses the cavity void as a solid-proxy outer member.",
    )
    for i in range(2):
        ctx.allow_overlap(
            body, drawer,
            elem_a="outer_shell",
            elem_b=f"tray_lip_{i}",
            reason="Tray side lip is a slide-rail contact strip nested inside the cavity shell.",
        )
    ctx.expect_within(
        drawer, body,
        axes="x",
        elem_a="drawer_tray",
        margin=0.005,
        name="tray stays within cavity width when closed",
    )
    ctx.expect_overlap(
        body, drawer,
        axes="y",
        elem_a="outer_shell",
        elem_b="drawer_tray",
        min_overlap=0.10,
        name="tray remains inserted in cavity when closed",
    )

    # ---- drawer opens: tray slides toward user --------------------------
    rest_pos = ctx.part_world_position(drawer)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        extended_pos = ctx.part_world_position(drawer)
        extended_aabb = ctx.part_world_aabb(drawer)
        ctx.check(
            "drawer pulls outward along -Y when slide is extended",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[1] < rest_pos[1] - 0.10,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )
        ctx.check(
            "extended drawer tray still retains insertion in the cavity",
            extended_aabb is not None
            and extended_aabb[1][1] > FRONT_Y + 0.01,
            details=f"extended aabb={extended_aabb}",
        )
        # Turntable moves with the drawer.
        tt_ext_aabb = ctx.part_world_aabb(turntable)
        ctx.check(
            "turntable moves with the drawer when extended",
            tt_ext_aabb is not None
            and tt_ext_aabb[0][1] < FRONT_Y - 0.05,
            details=f"turntable extended aabb={tt_ext_aabb}",
        )

    # ---- turntable: continuous spin, parented to drawer -----------------
    ctx.check(
        "turntable joint is continuous about a vertical axis",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and abs(spin.axis[2] - 1.0) < 1e-9
        and (spin.motion_limits is None or spin.motion_limits.lower is None),
        details=f"axis={spin.axis}, limits={spin.motion_limits}",
    )
    plate_aabb = ctx.part_element_world_aabb(turntable, elem="glass_plate")
    ctx.check(
        "glass plate is a thin disc on the drawer tray",
        plate_aabb is not None
        and (plate_aabb[1][2] - plate_aabb[0][2]) < 0.012,
        details=f"plate aabb={plate_aabb}",
    )
    ctx.expect_within(
        turntable,
        body,
        axes="x",
        margin=0.002,
        name="turntable stays within body width when closed",
    )

    # Off-axis coupler rib proves continuous rotation actually moves geometry.
    rib0_rest = ctx.part_element_world_aabb(turntable, elem="coupler_rib_0")
    with ctx.pose({spin: math.pi / 2.0}):
        rib0_spun = ctx.part_element_world_aabb(turntable, elem="coupler_rib_0")
    ctx.check(
        "spinning the turntable moves the off-axis coupler rib",
        rib0_rest is not None
        and rib0_spun is not None
        and abs(
            (rib0_spun[1][1] + rib0_spun[0][1]) / 2.0
            - (rib0_rest[1][1] + rib0_rest[0][1]) / 2.0
        )
        > 0.015,
        details=f"rib rest={rib0_rest}, spun={rib0_spun}",
    )

    # Guide rails exist on the body cavity walls (visible slide mechanism).
    rail0_aabb = ctx.part_element_world_aabb(body, elem="guide_rail_0")
    rail1_aabb = ctx.part_element_world_aabb(body, elem="guide_rail_1")
    ctx.check(
        "guide rails span most of the cavity depth on both walls",
        rail0_aabb is not None
        and rail1_aabb is not None
        and (rail0_aabb[1][1] - rail0_aabb[0][1]) > 0.20
        and (rail1_aabb[1][1] - rail1_aabb[0][1]) > 0.20,
        details=f"rail0={rail0_aabb}, rail1={rail1_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
