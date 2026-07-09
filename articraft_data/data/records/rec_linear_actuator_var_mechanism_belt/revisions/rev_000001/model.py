from __future__ import annotations

from math import cos, pi, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    WirePath,
    mesh_from_cadquery,
    mesh_from_geometry,
    sweep_profile_along_spline,
)


# ---------------------------------------------------------------------------
# Belt layout constants
# ---------------------------------------------------------------------------
_BELT_R = 0.020            # pulley pitch radius
_PULLEY_CX_L = -0.385     # drive-pulley centre X (inside left end plate)
_PULLEY_CX_R = 0.385      # idler-pulley centre X (inside right end plate)
_BELT_CZ = 0.080          # pulley / belt centre Z
_END_PLATE_X_L = -0.420
_END_PLATE_X_R = 0.420


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _cut_x_holes(body: cq.Workplane, thickness: float,
                 holes: list[tuple[float, float, float]]) -> cq.Workplane:
    """Cut circular through holes whose axes run along global X."""
    for y, z, radius in holes:
        cutter = (
            cq.Workplane("YZ")
            .center(y, z)
            .circle(radius)
            .extrude(thickness * 4.0, both=True)
        )
        body = body.cut(cutter)
    return body


def _plate_with_holes(
    *, thickness: float, width: float, height: float,
    holes: list[tuple[float, float, float]],
) -> cq.Workplane:
    plate = cq.Workplane("XY").box(thickness, width, height)
    plate = _cut_x_holes(plate, thickness, holes)
    return plate


def _motor_block() -> cq.Workplane:
    holes = [
        (0.0, 0.020, 0.018),
        (-0.055, -0.040, 0.006),
        (0.055, -0.040, 0.006),
        (-0.055, 0.052, 0.006),
        (0.055, 0.052, 0.006),
        (-0.066, 0.000, 0.004),
        (0.066, 0.000, 0.004),
    ]
    block = cq.Workplane("XY").box(0.100, 0.155, 0.135)
    return _cut_x_holes(block, 0.100, holes)


def _base_rail() -> cq.Workplane:
    """Long aluminum rail with shallow T-slot style grooves."""
    length, width, height = 0.830, 0.115, 0.050
    rail = cq.Workplane("XY").box(length, width, height)
    for y in (-0.036, 0.0, 0.036):
        groove = cq.Workplane("XY").box(length + 0.010, 0.012, 0.026).translate(
            (0.0, y, height / 2.0)
        )
        rail = rail.cut(groove)
    for y in (-width / 2.0, width / 2.0):
        slot = cq.Workplane("XY").box(length + 0.010, 0.010, 0.018).translate(
            (0.0, y, 0.002)
        )
        rail = rail.cut(slot)
    return rail


def _carriage_block() -> cq.Workplane:
    """Sliding carriage with guide-rod bores and horizontal belt passage."""
    block = cq.Workplane("XY").box(0.120, 0.142, 0.095)
    # Guide-rod and mounting-bolt holes (along X)
    holes = [
        (-0.045, 0.000, 0.0115),
        (0.045, 0.000, 0.0115),
        (-0.060, -0.028, 0.0048),
        (0.060, -0.028, 0.0048),
        (-0.060, 0.030, 0.0048),
        (0.060, 0.030, 0.0048),
    ]
    block = _cut_x_holes(block, 0.120, holes)

    # Horizontal belt-passage slot through the block at belt height.
    # Belt is at world z=0.080, carriage origin at z=0.1075, so local z=-0.0275.
    # Slot from local z=-0.0355 to z=-0.0195 (height 0.016).
    belt_slot = (
        cq.Workplane("XY")
        .box(0.130, 0.024, 0.016)
        .translate((0.0, -0.020, -0.0275))
    )
    block = block.cut(belt_slot)

    return block


def _pulley_cq() -> cq.Workplane:
    """Timing-belt pulley with flanges (axis along Z)."""
    R = 0.020
    h = 0.014
    flange_R = 0.024
    flange_h = 0.002
    body = cq.Workplane("XY").circle(R).extrude(h).translate((0, 0, -h / 2))
    top = cq.Workplane("XY").circle(flange_R).extrude(flange_h).translate(
        (0, 0, h / 2)
    )
    body = body.union(top)
    bot = cq.Workplane("XY").circle(flange_R).extrude(flange_h).translate(
        (0, 0, -h / 2 - flange_h)
    )
    body = body.union(bot)
    # Tooth grooves around circumference
    n_grooves = 20
    for i in range(n_grooves):
        a = 2.0 * pi * i / n_grooves
        cx_g = (R - 0.001) * cos(a)
        cy_g = (R - 0.001) * sin(a)
        groove = (
            cq.Workplane("XY")
            .transformed(
                offset=(cx_g, cy_g, 0.0),
                rotate=(0.0, 0.0, a * 180.0 / pi),
            )
            .box(0.004, 0.002, h - 0.002)
        )
        body = body.cut(groove)
    return body


def _belt_path_points() -> list[tuple[float, float, float]]:
    """Closed belt centre-line in the XY plane at z=_BELT_CZ."""
    R = _BELT_R
    wp = WirePath((_PULLEY_CX_L, -R, _BELT_CZ))
    wp.line_to((_PULLEY_CX_R, -R, _BELT_CZ))
    wp.arc(
        center=(_PULLEY_CX_R, 0.0, _BELT_CZ),
        normal=(0.0, 0.0, 1.0),
        angle=pi,
        segments=16,
    )
    wp.line_to((_PULLEY_CX_L, R, _BELT_CZ))
    wp.arc(
        center=(_PULLEY_CX_L, 0.0, _BELT_CZ),
        normal=(0.0, 0.0, 1.0),
        angle=pi,
        segments=16,
    )
    return wp.to_points()


def _belt_body_mesh():
    """Sweep a rectangular profile along the closed belt path."""
    pts = _belt_path_points()
    belt_w = 0.012
    belt_t = 0.003
    profile = [
        (-belt_w / 2, -belt_t / 2),
        (belt_w / 2, -belt_t / 2),
        (belt_w / 2, belt_t / 2),
        (-belt_w / 2, belt_t / 2),
    ]
    return sweep_profile_along_spline(
        pts,
        profile=profile,
        closed_spline=True,
        up_hint=(0.0, 0.0, 1.0),
        samples_per_segment=6,
    )


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="belt_driven_linear_actuator",
        meta={
            "category": "Robotics / Linear actuator",
            "run_note": (
                "Belt-driven variant: timing-belt loop over two end pulleys "
                "replaces the parent lead-screw drive. Carriage clamps to the "
                "front belt span; drive pulley rotates about Z."
            ),
        },
    )

    # --- Materials ---
    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    bright_aluminum = model.material(
        "polished_aluminum", rgba=(0.90, 0.92, 0.88, 1.0)
    )
    black = model.material("black_anodized", rgba=(0.015, 0.016, 0.016, 1.0))
    dark = model.material("dark_recess", rgba=(0.0, 0.0, 0.0, 1.0))
    steel = model.material("machined_steel", rgba=(0.82, 0.83, 0.80, 1.0))
    belt_mat = model.material(
        "black_timing_belt", rgba=(0.045, 0.045, 0.040, 1.0)
    )
    pulley_mat = model.material(
        "dark_pulley", rgba=(0.12, 0.12, 0.11, 1.0)
    )

    # ===================================================================
    # FRAME (root)
    # ===================================================================
    frame = model.part("frame")

    # Base rail
    frame.visual(
        mesh_from_cadquery(_base_rail(), "extruded_base_rail", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=aluminum,
        name="base_rail",
    )

    # End plates — holes for guide rods, pulley shafts, and mounting bolts
    end_holes = [
        (-0.045, 0.020, 0.0080),   # guide rod
        (0.045, 0.020, 0.0080),    # guide rod
        (0.000, 0.000, 0.0060),    # pulley shaft (Z-axis, through top)
        (-0.064, -0.043, 0.0060),
        (0.064, -0.043, 0.0060),
        (-0.064, 0.064, 0.0060),
        (0.064, 0.064, 0.0060),
    ]
    for index, x in enumerate((_END_PLATE_X_L, _END_PLATE_X_R)):
        frame.visual(
            mesh_from_cadquery(
                _plate_with_holes(
                    thickness=0.025, width=0.170, height=0.160, holes=end_holes
                ),
                f"end_plate_{index}_mesh",
                tolerance=0.0008,
            ),
            origin=Origin(xyz=(x, 0.0, 0.080)),
            material=black,
            name=f"end_plate_{index}",
        )

    # Motor block
    frame.visual(
        mesh_from_cadquery(_motor_block(), "motor_block_mesh", tolerance=0.0008),
        origin=Origin(xyz=(-0.4825, 0.0, 0.080)),
        material=black,
        name="motor_block",
    )

    # Guide rods
    for index, y in enumerate((-0.045, 0.045)):
        frame.visual(
            Cylinder(radius=0.0070, length=0.850),
            origin=Origin(xyz=(0.0, y, 0.100), rpy=(0.0, pi / 2.0, 0.0)),
            material=bright_aluminum,
            name=f"guide_rod_{index}",
        )
        for xi, xn in enumerate((_END_PLATE_X_L, _END_PLATE_X_R)):
            frame.visual(
                Cylinder(radius=0.0120, length=0.012),
                origin=Origin(xyz=(xn, y, 0.100), rpy=(0.0, pi / 2.0, 0.0)),
                material=steel,
                name=f"rod_clamp_{index}_{xi}",
            )

    # Pulley support bosses — short vertical pins on the rail top that
    # journal the pulleys. These connect the pulleys to the frame.
    boss_radius = 0.006
    boss_height = 0.020
    frame.visual(
        Cylinder(radius=boss_radius, length=boss_height),
        origin=Origin(xyz=(_PULLEY_CX_L, 0.0, 0.050 + boss_height / 2.0)),
        material=steel,
        name="pulley_boss_0",
    )
    frame.visual(
        Cylinder(radius=boss_radius, length=boss_height),
        origin=Origin(xyz=(_PULLEY_CX_R, 0.0, 0.050 + boss_height / 2.0)),
        material=steel,
        name="pulley_boss_1",
    )

    # Idler pulley (fixed to frame at right end)
    frame.visual(
        mesh_from_cadquery(_pulley_cq(), "idler_pulley_mesh", tolerance=0.0005),
        origin=Origin(xyz=(_PULLEY_CX_R, 0.0, _BELT_CZ)),
        material=pulley_mat,
        name="idler_pulley",
    )

    # --- Timing belt loop ---
    frame.visual(
        mesh_from_geometry(_belt_body_mesh(), "belt_body_mesh"),
        material=belt_mat,
        name="belt_body",
    )

    # Belt teeth — inner surface
    tooth_w = 0.005
    tooth_h = 0.002
    tooth_d = 0.008
    tooth_pitch = 0.028
    span_len = _PULLEY_CX_R - _PULLEY_CX_L

    # Front-span teeth (inner surface faces +Y)
    n_front = int(span_len / tooth_pitch)
    for i in range(n_front):
        x = _PULLEY_CX_L + (i + 0.5) * tooth_pitch
        y = -_BELT_R + 0.0015 + tooth_h / 2
        frame.visual(
            Box((tooth_w, tooth_h, tooth_d)),
            origin=Origin(xyz=(x, y, _BELT_CZ)),
            material=belt_mat,
            name=f"belt_tooth_f_{i}",
        )

    # Back-span teeth (inner surface faces -Y)
    for i in range(n_front):
        x = _PULLEY_CX_R - (i + 0.5) * tooth_pitch
        y = _BELT_R - 0.0015 - tooth_h / 2
        frame.visual(
            Box((tooth_w, tooth_h, tooth_d)),
            origin=Origin(xyz=(x, y, _BELT_CZ)),
            material=belt_mat,
            name=f"belt_tooth_b_{i}",
        )

    # Wrap teeth — right idler pulley
    n_wrap = 5
    for i in range(n_wrap):
        a = -pi / 2 + pi * (i + 0.5) / n_wrap
        r_t = _BELT_R - 0.0015 - tooth_h / 2
        x = _PULLEY_CX_R + r_t * cos(a)
        y = r_t * sin(a)
        frame.visual(
            Box((tooth_d, tooth_h, tooth_w)),
            origin=Origin(xyz=(x, y, _BELT_CZ), rpy=(0.0, 0.0, a - pi / 2)),
            material=belt_mat,
            name=f"belt_tooth_wr_{i}",
        )

    # Wrap teeth — left drive pulley
    for i in range(n_wrap):
        a = pi / 2 + pi * (i + 0.5) / n_wrap
        r_t = _BELT_R - 0.0015 - tooth_h / 2
        x = _PULLEY_CX_L + r_t * cos(a)
        y = r_t * sin(a)
        frame.visual(
            Box((tooth_d, tooth_h, tooth_w)),
            origin=Origin(xyz=(x, y, _BELT_CZ), rpy=(0.0, 0.0, a - pi / 2)),
            material=belt_mat,
            name=f"belt_tooth_wl_{i}",
        )

    # Top mounting holes on the rail
    for index, x in enumerate((-0.330, -0.210, -0.090, 0.090, 0.210, 0.330)):
        frame.visual(
            Cylinder(radius=0.0065, length=0.003),
            origin=Origin(xyz=(x, 0.0, 0.051)),
            material=dark,
            name=f"rail_hole_{index}",
        )

    # ===================================================================
    # CARRIAGE (prismatic child)
    # ===================================================================
    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(
            _carriage_block(), "carriage_block_mesh", tolerance=0.0008
        ),
        material=black,
        name="carriage_block",
    )

    # Socket-head fastener caps
    for index, (x, y) in enumerate(
        ((-0.042, -0.052), (0.042, -0.052), (-0.042, 0.052), (0.042, 0.052))
    ):
        carriage.visual(
            Cylinder(radius=0.0055, length=0.004),
            origin=Origin(xyz=(x, y, 0.0495)),
            material=steel,
            name=f"carriage_bolt_{index}",
        )

    # Visible steel belt-clamp plate pressing belt against the carriage
    # Positioned in the belt slot region, just below the belt.
    carriage.visual(
        Box((0.044, 0.028, 0.005)),
        origin=Origin(xyz=(0.0, -0.020, -0.031)),
        material=steel,
        name="belt_clamp",
    )

    # ===================================================================
    # DRIVE PULLEY (revolute child at motor end)
    # ===================================================================
    drive_pulley = model.part("drive_pulley")
    drive_pulley.visual(
        mesh_from_cadquery(_pulley_cq(), "drive_pulley_mesh", tolerance=0.0005),
        material=pulley_mat,
        name="pulley_body",
    )
    # Drive coupling stub — short cylinder reaching toward the motor below
    drive_pulley.visual(
        Cylinder(radius=0.005, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
        material=steel,
        name="drive_coupler",
    )

    # ===================================================================
    # ARTICULATIONS
    # ===================================================================
    model.articulation(
        "carriage_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(-0.160, 0.0, 0.1075)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0, velocity=0.50, lower=0.0, upper=0.390
        ),
    )

    model.articulation(
        "drive_pulley_spin",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=drive_pulley,
        origin=Origin(xyz=(_PULLEY_CX_L, 0.0, _BELT_CZ)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=20.0, lower=-6.283185307, upper=6.283185307
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    drive_pulley = object_model.get_part("drive_pulley")
    slide = object_model.get_articulation("carriage_slide")
    spin = object_model.get_articulation("drive_pulley_spin")

    # --- Part / joint existence ---
    ctx.check(
        "belt-driven actuator part set",
        all(
            object_model.get_part(n) is not None
            for n in ("frame", "carriage", "drive_pulley")
        ),
        details="Expected frame, sliding carriage, and drive_pulley parts.",
    )
    ctx.check(
        "drive mechanism joint types",
        slide.articulation_type == ArticulationType.PRISMATIC
        and spin.articulation_type == ArticulationType.REVOLUTE,
        details=f"slide={slide.articulation_type}, spin={spin.articulation_type}",
    )

    # --- Joint axes ---
    ctx.check(
        "carriage slide axis along rail",
        tuple(slide.axis) == (1.0, 0.0, 0.0),
        details=f"slide_axis={slide.axis}",
    )
    ctx.check(
        "drive_pulley_spin axis perpendicular to rail (Z)",
        tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"spin_axis={spin.axis}",
    )

    # --- Drive pulley at motor end ---
    dp_pos = ctx.part_world_position(drive_pulley)
    ctx.check(
        "drive_pulley at motor end",
        dp_pos is not None and dp_pos[0] < -0.30,
        details=f"drive_pulley position={dp_pos}",
    )

    # --- Belt clamp on carriage ---
    ctx.check(
        "belt_clamp visual on carriage",
        any(v.name == "belt_clamp" for v in carriage.visuals),
        details="Carriage must have a belt_clamp gripping the timing belt.",
    )

    # --- Belt body on frame ---
    ctx.check(
        "belt_body visual on frame",
        any(v.name == "belt_body" for v in frame.visuals),
        details="Frame must carry the closed timing-belt loop.",
    )

    # --- Idler pulley on frame at far end ---
    ctx.check(
        "idler_pulley on frame",
        any(v.name == "idler_pulley" for v in frame.visuals),
        details="Frame must have a fixed idler pulley at the far end.",
    )

    # --- Carriage stays on rail ---
    ctx.expect_within(
        carriage,
        frame,
        axes="x",
        inner_elem="carriage_block",
        outer_elem="base_rail",
        margin=0.010,
        name="carriage stays on rail at rest",
    )
    # Carriage is supported by guide rods, so there's a gap above the rail
    ctx.expect_gap(
        carriage,
        frame,
        axis="z",
        positive_elem="carriage_block",
        negative_elem="base_rail",
        min_gap=0.005,
        max_gap=0.020,
        name="carriage clears base rail (supported by guide rods)",
    )

    # --- Pose check: carriage travels along rail ---
    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.390}):
        ctx.expect_within(
            carriage,
            frame,
            axes="x",
            inner_elem="carriage_block",
            outer_elem="base_rail",
            margin=0.010,
            name="carriage remains on rail at full travel",
        )
        extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "positive slide extends carriage along rail",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.30,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    # --- Drive pulley boss seating ---
    # The drive pulley sits on the frame boss; the drive_coupler shaft
    # passes through the boss bearing to reach the motor below.
    ctx.allow_overlap(
        frame,
        drive_pulley,
        elem_a="pulley_boss_0",
        elem_b="drive_coupler",
        reason=(
            "The drive-pulley shaft (drive_coupler) passes through the "
            "frame bearing boss to reach the motor below."
        ),
    )
    ctx.expect_overlap(
        frame,
        drive_pulley,
        axes="z",
        elem_a="pulley_boss_0",
        elem_b="drive_coupler",
        min_overlap=0.010,
        name="drive shaft passes through bearing boss",
    )

    # --- Guide rods pass through carriage bores ---
    for rod_name in ("guide_rod_0", "guide_rod_1"):
        ctx.allow_overlap(
            carriage,
            frame,
            elem_a="carriage_block",
            elem_b=rod_name,
            reason=(
                f"The {rod_name} slides through the carriage block bore "
                "(linear bushing fit)."
            ),
        )
        ctx.expect_overlap(
            carriage,
            frame,
            axes="x",
            elem_a="carriage_block",
            elem_b=rod_name,
            min_overlap=0.100,
            name=f"{rod_name} passes through carriage bore",
        )

    return ctx.report()


object_model = build_object_model()
