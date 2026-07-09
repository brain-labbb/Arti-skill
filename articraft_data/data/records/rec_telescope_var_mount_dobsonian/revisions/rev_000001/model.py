from __future__ import annotations

# Dobsonian-mounted refractor telescope.
# Frame:
#   - World Z is up. The ground board sits flat on the ground (z=0).
#   - The rocker box sits on the ground board and spins in azimuth about +Z.
#   - The optical tube is cradled between two tall side boards on the rocker box
#     and swings in altitude about the horizontal -Y axis at the altitude bearings.
#   - The tube points along local +X at rest; black dew-shield at +X (front),
#     brass focuser draw-tube at -X (rear).
# Articulations:
#   - azimuth_rotation: CONTINUOUS about +Z (ground_board -> rocker_box).
#   - tube_altitude:    REVOLUTE about -Y (rocker_box -> optical_tube).
#   - focuser_slide:    PRISMATIC along -X (optical_tube -> focuser_drawtube).

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# --- optical tube dimensions (kept from parent B) ---
TUBE_LEN = 0.300
TUBE_R = 0.030
TUBE_REAR_X = -0.135
TUBE_FRONT_X = 0.165

# --- Dobsonian rocker-box mount dimensions (meters) ---
GROUND_BOARD_R = 0.150
GROUND_BOARD_THICK = 0.020

BOTTOM_PLATE_SIZE = 0.160
BOTTOM_PLATE_THICK = 0.015

SIDE_BOARD_HEIGHT = 0.130
SIDE_BOARD_THICK = 0.012
SIDE_BOARD_LENGTH = 0.120
SIDE_SPACING = 0.050  # center to inner face of each side board

BEARING_R = 0.045
BEARING_THICK = 0.020  # spans from tube surface to side board inner face

FRONT_BRACE_HEIGHT = 0.060

ALTITUDE_AXIS_Z = BOTTOM_PLATE_THICK + SIDE_BOARD_HEIGHT  # 0.145 in rocker local


def _tube_shell() -> cq.Workplane:
    """Hollow optical tube lofted along local +X (kept identical to parent B)."""
    outer = (
        cq.Workplane("YZ")
        .workplane(offset=TUBE_REAR_X)
        .circle(TUBE_R * 0.92)
        .workplane(offset=0.020)
        .circle(TUBE_R)
        .workplane(offset=TUBE_FRONT_X - TUBE_REAR_X - 0.020)
        .circle(TUBE_R)
        .loft(ruled=True)
    )
    bore = (
        cq.Workplane("YZ")
        .workplane(offset=TUBE_FRONT_X - 0.090)
        .circle(TUBE_R - 0.004)
        .workplane(offset=0.120)
        .circle(TUBE_R - 0.004)
        .loft(ruled=True)
    )
    return outer.cut(bore)


def _altitude_bearing_shape() -> cq.Workplane:
    """Semicircular altitude bearing: half-disk in XZ plane, lower half (Z <= 0).

    The flat diameter sits at the altitude axis (Z=0); the curved edge hangs
    below, cradled between the rocker-box side boards.
    """
    R = BEARING_R
    t = BEARING_THICK
    disk = (
        cq.Workplane("XZ")
        .workplane(offset=-t / 2)
        .circle(R)
        .extrude(t)
    )
    # Remove the upper half (Z > 0) to leave the lower semicircle
    cutter = (
        cq.Workplane("XY")
        .rect(2 * R + 0.01, 2 * R + 0.01)
        .extrude(R + 0.01)
    )
    return disk.cut(cutter)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dobsonian_refractor_telescope")

    # ---- materials ----
    blue = model.material("tube_blue", rgba=(0.30, 0.42, 0.62, 1.0))
    white = model.material("tube_white", rgba=(0.90, 0.92, 0.95, 1.0))
    black = model.material("matte_black", rgba=(0.10, 0.10, 0.12, 1.0))
    brass = model.material("brass", rgba=(0.78, 0.62, 0.28, 1.0))
    plywood = model.material("plywood", rgba=(0.72, 0.58, 0.38, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.28, 0.30, 0.33, 1.0))
    formica = model.material("formica", rgba=(0.15, 0.15, 0.18, 1.0))

    # =================================================================
    # GROUND BOARD (root) — round plywood base that spins in azimuth
    # =================================================================
    ground_board = model.part("ground_board")

    ground_board.visual(
        Cylinder(radius=GROUND_BOARD_R, length=GROUND_BOARD_THICK),
        origin=Origin(xyz=(0.0, 0.0, GROUND_BOARD_THICK / 2)),
        material=plywood,
        name="ground_disk",
    )
    # Three formica glide pads on the underside
    for i in range(3):
        ang = i * 2.0 * math.pi / 3.0
        ground_board.visual(
            Cylinder(radius=0.014, length=0.003),
            origin=Origin(xyz=(0.10 * math.cos(ang), 0.10 * math.sin(ang), 0.0015)),
            material=formica,
            name=f"ground_pad_{i}",
        )
    # Centre azimuth pivot disk (proud of the ground board top)
    ground_board.visual(
        Cylinder(radius=0.025, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, GROUND_BOARD_THICK + 0.0015)),
        material=dark_metal,
        name="azimuth_pivot",
    )

    ground_board.inertial = Inertial.from_geometry(
        Cylinder(radius=GROUND_BOARD_R, length=GROUND_BOARD_THICK + 0.006),
        mass=2.5,
        origin=Origin(xyz=(0.0, 0.0, GROUND_BOARD_THICK / 2)),
    )

    # =================================================================
    # ROCKER BOX — square plywood box with two tall side boards and a
    #              low front brace; carries the altitude bearings.
    # =================================================================
    rocker = model.part("rocker_box")

    # Bottom plate
    rocker.visual(
        Box((BOTTOM_PLATE_SIZE, BOTTOM_PLATE_SIZE, BOTTOM_PLATE_THICK)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_PLATE_THICK / 2)),
        material=plywood,
        name="rocker_floor",
    )

    # Two tall side boards (the defining Dobsonian feature)
    for side, y_sign in (("left", 1.0), ("right", -1.0)):
        y_center = y_sign * (SIDE_SPACING + SIDE_BOARD_THICK / 2)
        rocker.visual(
            Box((SIDE_BOARD_LENGTH, SIDE_BOARD_THICK, SIDE_BOARD_HEIGHT)),
            origin=Origin(xyz=(0.0, y_center, BOTTOM_PLATE_THICK + SIDE_BOARD_HEIGHT / 2)),
            material=plywood,
            name=f"side_board_{side}",
        )

    # Front brace connecting the two side boards (low enough to clear tube travel)
    brace_width = 2.0 * (SIDE_SPACING + SIDE_BOARD_THICK)
    rocker.visual(
        Box((SIDE_BOARD_THICK, brace_width, FRONT_BRACE_HEIGHT)),
        origin=Origin(xyz=(
            SIDE_BOARD_LENGTH / 2 - SIDE_BOARD_THICK / 2,
            0.0,
            BOTTOM_PLATE_THICK + FRONT_BRACE_HEIGHT / 2,
        )),
        material=plywood,
        name="front_brace",
    )

    # Off-axis azimuth marker so the spin is visually detectable
    rocker.visual(
        Box((0.012, 0.010, 0.008)),
        origin=Origin(xyz=(0.062, 0.062, BOTTOM_PLATE_THICK + 0.004)),
        material=brass,
        name="azimuth_marker",
    )

    rocker.inertial = Inertial.from_geometry(
        Box((BOTTOM_PLATE_SIZE, 2.0 * (SIDE_SPACING + SIDE_BOARD_THICK),
             BOTTOM_PLATE_THICK + SIDE_BOARD_HEIGHT)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, (BOTTOM_PLATE_THICK + SIDE_BOARD_HEIGHT) / 2)),
    )

    # Azimuth joint: ground_board → rocker_box, CONTINUOUS about +Z
    model.articulation(
        "azimuth_rotation",
        ArticulationType.CONTINUOUS,
        parent=ground_board,
        child=rocker,
        origin=Origin(xyz=(0.0, 0.0, GROUND_BOARD_THICK)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0),
    )

    # =================================================================
    # OPTICAL TUBE — kept identical to parent B, plus altitude bearings
    # =================================================================
    tube = model.part("optical_tube")

    # CadQuery lofted hollow shell
    tube.visual(
        mesh_from_cadquery(_tube_shell(), "tube_shell"),
        material=white,
        name="tube_shell",
    )
    # Blue band wrapping the mid/rear of the tube
    tube.visual(
        Cylinder(radius=TUBE_R + 0.0015, length=0.150),
        origin=Origin(xyz=(-0.040, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blue,
        name="blue_band",
    )
    # Black dew shield at the front mouth
    tube.visual(
        Cylinder(radius=TUBE_R + 0.003, length=0.030),
        origin=Origin(xyz=(TUBE_FRONT_X - 0.013, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="dew_shield",
    )
    # Brass trim ring behind the dew shield
    tube.visual(
        Cylinder(radius=TUBE_R + 0.0035, length=0.008),
        origin=Origin(xyz=(TUBE_FRONT_X - 0.034, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="objective_ring",
    )
    # Brass cradle ring at the altitude pivot
    tube.visual(
        Cylinder(radius=TUBE_R + 0.004, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="cradle_ring",
    )
    # Brass focuser housing at the rear
    tube.visual(
        Cylinder(radius=0.016, length=0.030),
        origin=Origin(xyz=(TUBE_REAR_X + 0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="focuser_housing",
    )
    # Focus knob on the side
    tube.visual(
        Cylinder(radius=0.007, length=0.016),
        origin=Origin(xyz=(TUBE_REAR_X + 0.010, TUBE_R + 0.006, 0.0),
                       rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="focus_knob",
    )

    # Altitude bearings — semicircular trunnions on the tube sides
    bearing_cq = _altitude_bearing_shape()
    for side, y_sign in (("left", 1.0), ("right", -1.0)):
        y_offset = y_sign * (TUBE_R + BEARING_THICK / 2)
        tube.visual(
            mesh_from_cadquery(bearing_cq, f"altitude_bearing_{side}"),
            origin=Origin(xyz=(0.0, y_offset, 0.0)),
            material=formica,
            name=f"altitude_bearing_{side}",
        )

    tube.inertial = Inertial.from_geometry(
        Cylinder(radius=TUBE_R, length=TUBE_LEN),
        mass=0.50,
        origin=Origin(xyz=(0.5 * (TUBE_FRONT_X + TUBE_REAR_X), 0.0, 0.0),
                       rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    # Altitude joint: rocker_box → optical_tube, REVOLUTE about -Y
    model.articulation(
        "tube_altitude",
        ArticulationType.REVOLUTE,
        parent=rocker,
        child=tube,
        origin=Origin(xyz=(0.0, 0.0, ALTITUDE_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.0,
            lower=-math.radians(30.0),
            upper=math.radians(45.0),
        ),
    )

    # =================================================================
    # FOCUSER DRAW-TUBE — kept identical to parent B
    # =================================================================
    draw = model.part("focuser_drawtube")

    draw.visual(
        Cylinder(radius=0.010, length=0.090),
        origin=Origin(xyz=(0.030, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="drawtube_barrel",
    )
    draw.visual(
        Cylinder(radius=0.013, length=0.022),
        origin=Origin(xyz=(-0.026, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="eyepiece_body",
    )
    draw.visual(
        Cylinder(radius=0.009, length=0.010),
        origin=Origin(xyz=(-0.042, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="eyepiece_cup",
    )
    draw.inertial = Inertial.from_geometry(
        Box((0.130, 0.026, 0.026)),
        mass=0.04,
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
    )

    # Prismatic slide: +axis along -X so positive travel extends rearward
    model.articulation(
        "focuser_slide",
        ArticulationType.PRISMATIC,
        parent=tube,
        child=draw,
        origin=Origin(xyz=(TUBE_REAR_X + 0.010, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.05, lower=0.0, upper=0.030),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ground_board = object_model.get_part("ground_board")
    rocker = object_model.get_part("rocker_box")
    tube = object_model.get_part("optical_tube")
    draw = object_model.get_part("focuser_drawtube")

    az = object_model.get_articulation("azimuth_rotation")
    tilt = object_model.get_articulation("tube_altitude")
    slide = object_model.get_articulation("focuser_slide")

    # ---- Ground board: round disk resting on the ground ----
    ground_ab = ctx.part_world_aabb(ground_board)
    ctx.check(
        "ground_board sits on the ground (bottom near z=0)",
        ground_ab[0][2] < 0.008,
        details=f"min_z={ground_ab[0][2]}",
    )
    dx = ground_ab[1][0] - ground_ab[0][0]
    dy = ground_ab[1][1] - ground_ab[0][1]
    ctx.check(
        "ground_board is round (X and Y extents match)",
        abs(dx - dy) < 0.010,
        details=f"dx={dx:.4f} dy={dy:.4f}",
    )

    # ---- Rocker box sits on ground board ----
    # Allow the azimuth pivot disk nested under the rocker floor
    ctx.allow_overlap(
        ground_board, rocker,
        elem_a="azimuth_pivot", elem_b="rocker_floor",
        reason="The azimuth pivot disk is intentionally nested under the rocker floor as a flush bearing.",
    )
    ctx.expect_contact(
        ground_board, rocker,
        name="rocker_box seated on ground_board",
    )

    # ---- Side boards on opposite sides ----
    left_ab = ctx.part_element_world_aabb(rocker, elem="side_board_left")
    right_ab = ctx.part_element_world_aabb(rocker, elem="side_board_right")
    left_y = 0.5 * (left_ab[0][1] + left_ab[1][1])
    right_y = 0.5 * (right_ab[0][1] + right_ab[1][1])
    ctx.check(
        "rocker_box side boards on opposite sides of center",
        left_y > 0.02 and right_y < -0.02,
        details=f"left_y={left_y:.4f} right_y={right_y:.4f}",
    )

    # ---- Side boards reach the altitude axis (variant-specific proof) ----
    rocker_z = ctx.part_world_position(rocker)[2]
    ctx.check(
        "side_board_left top reaches altitude axis height (dobsonian variant)",
        left_ab[1][2] > rocker_z + ALTITUDE_AXIS_Z - 0.020,
        details=f"side_top_z={left_ab[1][2]:.4f} expected>{rocker_z + ALTITUDE_AXIS_Z - 0.020:.4f}",
    )

    # ---- Altitude bearings exist on the tube ----
    bl = ctx.part_element_world_aabb(tube, elem="altitude_bearing_left")
    br = ctx.part_element_world_aabb(tube, elem="altitude_bearing_right")
    ctx.check(
        "altitude_bearing_left and altitude_bearing_right present on optical_tube",
        bl is not None and br is not None,
        details="bearings found",
    )
    # Bearings on opposite Y sides of the tube
    bl_y = 0.5 * (bl[0][1] + bl[1][1])
    br_y = 0.5 * (br[0][1] + br[1][1])
    ctx.check(
        "altitude bearings on opposite tube sides",
        abs(bl_y - br_y) > 0.040,
        details=f"bearing_left_y={bl_y:.4f} bearing_right_y={br_y:.4f}",
    )

    # ---- Tube cradled between side boards ----
    ctx.expect_within(
        tube, rocker,
        axes="y",
        margin=0.010,
        name="tube stays within rocker side boards (Y)",
    )

    # ---- Azimuth: rocker swings about +Z ----
    m0_ab = ctx.part_element_world_aabb(rocker, elem="azimuth_marker")
    m0 = (0.5 * (m0_ab[0][0] + m0_ab[1][0]),
           0.5 * (m0_ab[0][1] + m0_ab[1][1]))
    with ctx.pose({az: math.pi / 2.0}):
        m1_ab = ctx.part_element_world_aabb(rocker, elem="azimuth_marker")
        m1 = (0.5 * (m1_ab[0][0] + m1_ab[1][0]),
               0.5 * (m1_ab[0][1] + m1_ab[1][1]))
    ctx.check(
        "azimuth_rotation swings rocker_box marker around vertical",
        math.hypot(m1[0] - m0[0], m1[1] - m0[1]) > 0.02,
        details=f"rest={m0} quarter={m1}",
    )

    # ---- Altitude: objective rises when tilted up ----
    front0 = ctx.part_element_world_aabb(tube, elem="dew_shield")
    front_z0 = 0.5 * (front0[0][2] + front0[1][2])
    with ctx.pose({tilt: math.radians(40.0)}):
        front_up = ctx.part_element_world_aabb(tube, elem="dew_shield")
        front_z_up = 0.5 * (front_up[0][2] + front_up[1][2])
    with ctx.pose({tilt: math.radians(-25.0)}):
        front_dn = ctx.part_element_world_aabb(tube, elem="dew_shield")
        front_z_dn = 0.5 * (front_dn[0][2] + front_dn[1][2])
    ctx.check(
        "tube_altitude up raises the objective end",
        front_z_up > front_z0 + 0.02,
        details=f"rest={front_z0:.4f} up={front_z_up:.4f}",
    )
    ctx.check(
        "tube_altitude down lowers the objective end",
        front_z_dn < front_z0 - 0.01,
        details=f"rest={front_z0:.4f} down={front_z_dn:.4f}",
    )

    # ---- Focuser draw-tube (kept identical to parent B) ----
    ctx.allow_overlap(
        draw, tube,
        elem_a="drawtube_barrel", elem_b="focuser_housing",
        reason="Draw-tube barrel intentionally inserted into focuser housing proxy.",
    )
    ctx.allow_overlap(
        draw, tube,
        elem_a="drawtube_barrel", elem_b="tube_shell",
        reason="Draw-tube barrel slides inside the solid rear of the tube shell proxy.",
    )
    ctx.allow_overlap(
        draw, tube,
        elem_a="drawtube_barrel", elem_b="blue_band",
        reason="Draw-tube barrel passes under the blue band wrap inside the tube.",
    )
    ctx.expect_within(
        draw, tube,
        axes="yz",
        inner_elem="drawtube_barrel",
        outer_elem="focuser_housing",
        margin=0.004,
        name="draw-tube centered in focuser housing",
    )
    ctx.expect_overlap(
        draw, tube,
        axes="x",
        elem_a="drawtube_barrel", elem_b="focuser_housing",
        min_overlap=0.010,
        name="draw-tube inserted in housing at rest",
    )
    draw_rear0 = ctx.part_world_aabb(draw)[0][0]
    with ctx.pose({slide: 0.030}):
        draw_rear1 = ctx.part_world_aabb(draw)[0][0]
        ctx.expect_overlap(
            draw, tube,
            axes="x",
            elem_a="drawtube_barrel", elem_b="focuser_housing",
            min_overlap=0.004,
            name="draw-tube stays captured when extended",
        )
    ctx.check(
        "focuser_slide extends draw-tube rearward",
        draw_rear1 < draw_rear0 - 0.02,
        details=f"rest={draw_rear0:.4f} extended={draw_rear1:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
