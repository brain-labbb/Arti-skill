from __future__ import annotations

"""Stylized low-poly countertop microwave oven with drop-down oven-style door.

Layout (world frame):
- x: width (0.52 m), y: depth (0.38 m, front face toward -Y), z: up (0.32 m total).
- Root `body`: chamfered pale blue-gray shell, hollow dark cooking cavity opening
  at the front-left three quarters, inset charcoal control panel on the right
  quarter, four dark block feet grounding the unit at z=0.
- `door`: charcoal framed panel with recessed near-black window and a proud
  horizontal handle bar at the top; revolute about a horizontal axis at its
  bottom front edge, swinging downward 0..90 deg (oven-style drop-down).
- `turntable`: glass plate on a drive hub with three coupler ribs, continuous
  spin about the vertical axis at the cavity floor center.
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

# Cooking cavity (cut through the front face, behind the door)
CAV_W = 0.32
CAV_H = 0.20
CAV_X = -0.04
CAV_Z0 = 0.07
CAV_Z1 = CAV_Z0 + CAV_H  # 0.27
CAV_BACK_Y = 0.09
LINER_T = 0.004

# Door (drop-down, covers the left ~3/4 of the front face)
DOOR_W = 0.38
DOOR_H = 0.255
DOOR_T = 0.022
DOOR_X0 = -0.245  # left edge of the door
DOOR_CX = DOOR_X0 + DOOR_W / 2.0  # door center X = -0.055
DOOR_CZ = 0.1725
DOOR_BOTTOM_Z = DOOR_CZ - DOOR_H / 2.0  # 0.045
DOOR_GAP = 0.0  # closed door seats in contact with the body front face
DOOR_BACK_Y = FRONT_Y - DOOR_GAP  # -0.19
DOOR_FRONT_Y = DOOR_BACK_Y - DOOR_T  # -0.212

# Control panel strip (right quarter, inset into the front face)
PANEL_CX = 0.20
PANEL_W = 0.088
PANEL_H = 0.253

# Turntable
TT_X = CAV_X
TT_Y = (FRONT_Y + CAV_BACK_Y) / 2.0  # -0.05
FLOOR_TOP_Z = CAV_Z0 + LINER_T - 0.0005  # top of the cavity floor liner: 0.0735
TT_Z = FLOOR_TOP_Z  # hub rests in contact with the floor liner
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
    cavity = (
        cq.Workplane("XY")
        .box(CAV_W, 0.30, CAV_H)
        .translate((CAV_X, -0.06, (CAV_Z0 + CAV_Z1) / 2.0))
    )
    shell = shell.cut(cavity)
    # Shallow recess for the control panel strip.
    recess = (
        cq.Workplane("XY")
        .box(PANEL_W + 0.002, 0.012, PANEL_H + 0.002)
        .translate((PANEL_CX, FRONT_Y, DOOR_CZ))
    )
    shell = shell.cut(recess)
    return shell


def _door_solid() -> cq.Workplane:
    """Door frame in the bottom-hinge local frame.

    Local frame: origin on the hinge axis at the bottom center of the door.
    Door extends along +Z (upward when closed), along -Y (front face outward),
    and along ±X (width centered on origin).
    """
    door = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_H)
        .translate((0.0, -DOOR_T / 2.0, DOOR_H / 2.0))
        .edges()
        .chamfer(0.003)
    )
    # Window pocket recessed into the front face (-Y face).
    pocket = (
        cq.Workplane("XY")
        .box(0.30, 0.016, 0.18)
        .translate((0.0, -DOOR_T + 0.007, DOOR_H / 2.0 + 0.015))
    )
    return door.cut(pocket)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_microwave_oven")

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

    # Inset charcoal control panel on the right quarter of the front face.
    body.visual(
        Box((PANEL_W, 0.004, PANEL_H)),
        origin=Origin(xyz=(PANEL_CX, FRONT_Y + 0.0045, DOOR_CZ)),
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

    # ------------------------------------------------------------------ door
    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_door_solid(), "microwave_door"),
        material=CHARCOAL,
        name="door_frame",
    )
    # Near-black glass window seated at the bottom of the recess pocket.
    door.visual(
        Box((0.298, 0.004, 0.178)),
        origin=Origin(xyz=(0.0, -0.009, DOOR_H / 2.0 + 0.015)),
        material=NEAR_BLACK,
        name="window_glass",
    )
    # Horizontal handle bar at the top of the door, proud of the front face.
    handle_bar_z = DOOR_H - 0.028  # local z, near top of door
    standoff_depth = 0.012
    bar_half_depth = 0.009
    handle_bar_y = -DOOR_T - standoff_depth - bar_half_depth  # local y
    door.visual(
        Box((0.30, bar_half_depth * 2, 0.014)),
        origin=Origin(xyz=(0.0, handle_bar_y, handle_bar_z)),
        material=BODY_BLUE,
        name="handle_bar",
    )
    # Two standoffs connecting the horizontal bar to the door face.
    standoff_y = -DOOR_T - standoff_depth / 2.0
    for i, sx in enumerate([-0.12, 0.12]):
        door.visual(
            Box((0.024, standoff_depth, 0.018)),
            origin=Origin(xyz=(sx, standoff_y, handle_bar_z)),
            material=BODY_BLUE,
            name=f"handle_standoff_{i}",
        )

    # Door hinge: horizontal axis along bottom front edge; positive q
    # drops the top of the door forward and down (oven-style), 0..90 degrees.
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(DOOR_CX, DOOR_BACK_Y, DOOR_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0,
            lower=0.0, upper=math.radians(90.0),
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

    model.articulation(
        "turntable_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=turntable,
        origin=Origin(xyz=(TT_X, TT_Y, TT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    turntable = object_model.get_part("turntable")
    hinge = object_model.get_articulation("door_hinge")
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

    # ---- door geometry and closed seating -------------------------------
    door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door covers the left ~3/4 of the front face",
        door_aabb is not None
        and abs(door_aabb[0][0] - DOOR_X0) < 0.005
        and abs(door_aabb[1][0] - (DOOR_X0 + DOOR_W)) < 0.005,
        details=f"door aabb={door_aabb}",
    )
    # Closed door seats against the body front face.
    ctx.expect_gap(
        body,
        door,
        axis="y",
        min_gap=0.0,
        max_gap=0.004,
        name="closed door seats against the body front with small clearance",
    )

    # Handle bar is horizontal at the top of the door, proud of the face.
    bar_aabb = ctx.part_element_world_aabb(door, elem="handle_bar")
    ctx.check(
        "handle bar is horizontal at the top, proud of the door face",
        bar_aabb is not None
        and (bar_aabb[1][0] - bar_aabb[0][0]) > 0.25  # spans most of door width
        and (bar_aabb[1][2] - bar_aabb[0][2]) < 0.02  # thin in Z (horizontal)
        and bar_aabb[0][2] > DOOR_CZ + DOOR_H / 2.0 - 0.06  # near the top
        and bar_aabb[0][1] < DOOR_FRONT_Y - 0.005,  # proud of door face
        details=f"handle aabb={bar_aabb}",
    )

    # Window glass is recessed behind the door front face.
    glass_aabb = ctx.part_element_world_aabb(door, elem="window_glass")
    ctx.check(
        "window glass is recessed behind the door face",
        glass_aabb is not None and glass_aabb[0][1] > DOOR_FRONT_Y + 0.004,
        details=f"glass aabb={glass_aabb}, door front y={DOOR_FRONT_Y}",
    )

    # Control panel sits to the right of the door, inset into the front face.
    panel_aabb = ctx.part_element_world_aabb(body, elem="control_panel")
    ctx.check(
        "control panel strip sits right of the door and inset into the face",
        panel_aabb is not None
        and door_aabb is not None
        and panel_aabb[0][0] > door_aabb[1][0]
        and panel_aabb[0][1] > FRONT_Y + 0.001,
        details=f"panel aabb={panel_aabb}",
    )

    # ---- door hinge: drop-down revolute about horizontal X axis ---------
    ctx.check(
        "door hinge is revolute about a horizontal (X) axis with 0..90 deg range",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and abs(hinge.axis[0] - 1.0) < 1e-9
        and abs(hinge.axis[1]) < 1e-9
        and abs(hinge.axis[2]) < 1e-9
        and hinge.motion_limits is not None
        and abs(hinge.motion_limits.lower - 0.0) < 1e-9
        and abs(hinge.motion_limits.upper - math.radians(90.0)) < 1e-6,
        details=f"axis={hinge.axis}, limits={hinge.motion_limits}",
    )
    ctx.check(
        "hinge origin sits at the door bottom edge",
        abs(hinge.origin.xyz[2] - DOOR_BOTTOM_Z) < 0.002,
        details=f"hinge z={hinge.origin.xyz[2]}, door bottom z={DOOR_BOTTOM_Z}",
    )

    closed_top_z = DOOR_BOTTOM_Z + DOOR_H  # ≈ 0.30

    with ctx.pose({hinge: math.radians(90.0)}):
        open_aabb = ctx.part_world_aabb(door)
        ctx.check(
            "open door drops down to a roughly horizontal position",
            open_aabb is not None
            and (open_aabb[1][2] - open_aabb[0][2]) < 0.12  # thin in Z
            and open_aabb[0][1] < FRONT_Y - 0.15,  # extends forward
            details=f"open door aabb={open_aabb}",
        )
        ctx.check(
            "open door top edge drops well below the closed height",
            open_aabb is not None and open_aabb[1][2] < closed_top_z - 0.15,
            details=f"open top z={open_aabb[1][2]}, closed top z={closed_top_z}",
        )
        ctx.check(
            "open door remains attached at the bottom hinge edge",
            open_aabb is not None
            and abs(open_aabb[1][1] - DOOR_BACK_Y) < 0.01,
            details=f"open door max y={open_aabb[1][1]}, hinge y={DOOR_BACK_Y}",
        )

    # ---- turntable: continuous spin, seated in the cavity ----------------
    ctx.check(
        "turntable joint is continuous about a vertical axis",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and abs(spin.axis[2] - 1.0) < 1e-9
        and (spin.motion_limits is None or spin.motion_limits.lower is None),
        details=f"axis={spin.axis}, limits={spin.motion_limits}",
    )
    plate_aabb = ctx.part_element_world_aabb(turntable, elem="glass_plate")
    ctx.check(
        "glass plate is a thin disc centered on the cavity floor center",
        plate_aabb is not None
        and abs((plate_aabb[1][0] + plate_aabb[0][0]) / 2.0 - TT_X) < 0.002
        and abs((plate_aabb[1][1] + plate_aabb[0][1]) / 2.0 - TT_Y) < 0.002
        and (plate_aabb[1][2] - plate_aabb[0][2]) < 0.012,
        details=f"plate aabb={plate_aabb}",
    )
    ctx.expect_within(
        turntable,
        body,
        axes="xy",
        margin=0.0,
        name="turntable stays inside the body footprint",
    )
    ctx.expect_gap(
        turntable,
        body,
        axis="z",
        min_gap=0.0,
        max_gap=0.002,
        positive_elem="drive_hub",
        negative_elem="cavity_floor",
        name="drive hub rests on the cavity floor",
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

    return ctx.report()


object_model = build_object_model()
