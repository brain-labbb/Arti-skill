from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

TABLE_TILT = 0.50
HINGE_X = -0.36
HINGE_Z = 0.88


def _rot_y(point: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
    """Rotate a local tabletop point into the child link frame."""
    x, y, z = point
    c = math.cos(angle)
    s = math.sin(angle)
    return (c * x + s * z, y, -s * x + c * z)


def _table_origin(x: float, y: float, z: float, tilt: float = TABLE_TILT) -> Origin:
    return Origin(xyz=_rot_y((x, y, z), tilt), rpy=(0.0, tilt, 0.0))


def _box_between_xz(
    part,
    *,
    p0: tuple[float, float],
    p1: tuple[float, float],
    y: float,
    width_y: float,
    thickness: float,
    material,
    name: str,
) -> None:
    """Add a rectangular rail whose long axis runs between two XZ points."""
    x0, z0 = p0
    x1, z1 = p1
    dx = x1 - x0
    dz = z1 - z0
    length = math.hypot(dx, dz)
    # A box's local +X is rotated about Y; pitch beta gives world direction
    # (cos beta, 0, -sin beta).  The sign of the long axis is immaterial.
    beta = math.atan2(-dz, dx)
    part.visual(
        Box((length, width_y, thickness)),
        origin=Origin(xyz=((x0 + x1) / 2.0, y, (z0 + z1) / 2.0), rpy=(0.0, beta, 0.0)),
        material=material,
        name=name,
    )


def _annular_sector_geometry(
    *,
    center_x: float,
    center_z: float,
    radius: float,
    radial_width: float,
    y_center: float,
    y_thickness: float,
    start_angle: float,
    end_angle: float,
    segments: int = 36,
) -> MeshGeometry:
    """Build a connected curved metal strip in the XZ plane, extruded in Y."""
    geom = MeshGeometry()
    y0 = y_center - y_thickness / 2.0
    y1 = y_center + y_thickness / 2.0
    inner = radius - radial_width
    outer = radius + radial_width
    rings: list[tuple[int, int, int, int]] = []
    for i in range(segments + 1):
        t = start_angle + (end_angle - start_angle) * i / segments
        c = math.cos(t)
        s = math.sin(t)
        # outer-front, inner-front, outer-back, inner-back
        rings.append(
            (
                geom.add_vertex(center_x + outer * c, y0, center_z + outer * s),
                geom.add_vertex(center_x + inner * c, y0, center_z + inner * s),
                geom.add_vertex(center_x + outer * c, y1, center_z + outer * s),
                geom.add_vertex(center_x + inner * c, y1, center_z + inner * s),
            )
        )
    for i in range(segments):
        of0, inf0, ob0, ib0 = rings[i]
        of1, inf1, ob1, ib1 = rings[i + 1]
        # front and back broad faces
        geom.add_face(of0, of1, inf1)
        geom.add_face(of0, inf1, inf0)
        geom.add_face(ob0, ib1, ob1)
        geom.add_face(ob0, ib0, ib1)
        # outer and inner curved faces
        geom.add_face(of0, ob0, ob1)
        geom.add_face(of0, ob1, of1)
        geom.add_face(inf0, inf1, ib1)
        geom.add_face(inf0, ib1, ib0)
    # end caps
    for idx in (0, -1):
        of0, inf0, ob0, ib0 = rings[idx]
        geom.add_face(of0, inf0, ib0)
        geom.add_face(of0, ib0, ob0)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="tilting_wood_drafting_table",
        meta={
            "category": "Workspace / Drafting table",
            "reference_note": "Reference and category both indicate a wood drafting table; no classification mismatch suspected.",
        },
    )

    wood = model.material("warm_cherry_wood", rgba=(0.62, 0.28, 0.09, 1.0))
    dark_wood = model.material("dark_end_grain", rgba=(0.38, 0.15, 0.045, 1.0))
    light_wood = model.material("worn_edge_highlight", rgba=(0.86, 0.46, 0.18, 1.0))
    soft_grain = model.material("soft_cherry_grain", rgba=(0.50, 0.20, 0.060, 1.0))
    warm_grain = model.material("warm_cherry_grain_highlight", rgba=(0.74, 0.34, 0.11, 1.0))
    black_metal = model.material("blackened_steel", rgba=(0.015, 0.014, 0.012, 1.0))
    screw_metal = model.material("dark_screw_heads", rgba=(0.06, 0.055, 0.05, 1.0))

    support = model.part("support_frame")

    # Trestle feet, upright cheeks, and stretcher rails.  The pieces intentionally
    # touch or slightly interleave like glued/bolted wood joinery so the root part
    # reads as one grounded frame rather than separated planks.
    for suffix, y in (("near", -0.62), ("far", 0.62)):
        support.visual(
            Box((0.66, 0.09, 0.065)),
            origin=Origin(xyz=(-0.02, y, 0.0325)),
            material=wood,
            name=f"{suffix}_floor_foot",
        )
        support.visual(
            Box((0.18, 0.105, 0.18)),
            origin=Origin(xyz=(-0.22, y, 0.155)),
            material=dark_wood,
            name=f"{suffix}_foot_block",
        )
        support.visual(
            Box((0.078, 0.082, 0.78)),
            origin=Origin(xyz=(-0.22, y, 0.455)),
            material=wood,
            name=f"{suffix}_upright",
        )
        support.visual(
            Box((0.095, 0.09, 0.10)),
            origin=Origin(xyz=(-0.35, y, 0.825)),
            material=wood,
            name=f"{suffix}_hinge_cheek",
        )
        _box_between_xz(
            support,
            p0=(-0.23, 0.66),
            p1=(-0.38, 0.84),
            y=y,
            width_y=0.055,
            thickness=0.04,
            material=wood,
            name=f"{suffix}_rear_brace",
        )
        _box_between_xz(
            support,
            p0=(-0.19, 0.34),
            p1=(0.05, 0.54),
            y=y,
            width_y=0.045,
            thickness=0.04,
            material=dark_wood,
            name=f"{suffix}_front_brace",
        )
        support.visual(
            Box((0.31, 0.055, 0.050)),
            origin=Origin(xyz=(-0.075, y, 0.235)),
            material=wood,
            name=f"{suffix}_lower_side_rail",
        )

    support.visual(
        Box((0.06, 1.20, 0.055)),
        origin=Origin(xyz=(0.05, 0.0, 0.235)),
        material=wood,
        name="lower_front_stretcher",
    )
    support.visual(
        Box((0.055, 1.20, 0.05)),
        origin=Origin(xyz=(-0.32, 0.0, 0.215)),
        material=dark_wood,
        name="lower_rear_stretcher",
    )
    support.visual(
        Box((0.055, 1.20, 0.055)),
        origin=Origin(xyz=(-0.34, 0.0, 0.735)),
        material=wood,
        name="upper_cross_rail",
    )
    support.visual(
        Cylinder(radius=0.013, length=1.28),
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_metal,
        name="hinge_rod",
    )

    # Black metal ratchet quadrant on the visible side.  A chain of overlapping
    # short tangent plates gives the curved angle-setting sector visible in the
    # reference without making a free-floating arc.
    arc_center = (-0.34, 0.61)
    radius = 0.285
    arc_y = -0.665
    angles = [math.radians(a) for a in range(-112, 78, 10)]
    support.visual(
        mesh_from_geometry(
            _annular_sector_geometry(
                center_x=arc_center[0],
                center_z=arc_center[1],
                radius=radius,
                radial_width=0.018,
                y_center=arc_y,
                y_thickness=0.026,
                start_angle=math.radians(-112),
                end_angle=math.radians(78),
                segments=40,
            ),
            "ratchet_arc_plate",
        ),
        material=black_metal,
        name="ratchet_arc_plate",
    )
    for i, theta in enumerate(angles):
        x = arc_center[0] + radius * math.cos(theta)
        z = arc_center[1] + radius * math.sin(theta)
        if i % 2 == 0:
            support.visual(
                Cylinder(radius=0.0065, length=0.006),
                origin=Origin(xyz=(x, arc_y - 0.014, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=screw_metal,
                name=f"ratchet_hole_{i}",
            )
    support.visual(
        Box((0.12, 0.03, 0.030)),
        origin=Origin(xyz=(-0.33, arc_y, 0.845), rpy=(0.0, -0.10, 0.0)),
        material=black_metal,
        name="arc_top_mount",
    )
    support.visual(
        Box((0.11, 0.03, 0.030)),
        origin=Origin(xyz=(-0.23, arc_y, 0.44), rpy=(0.0, 0.18, 0.0)),
        material=black_metal,
        name="arc_lower_mount",
    )
    # Bushings beneath the rotatable hand knobs.
    support.visual(
        Cylinder(radius=0.027, length=0.018),
        origin=Origin(xyz=(-0.22, -0.668, 0.505), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_metal,
        name="lock_bushing",
    )
    support.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(-0.23, -0.668, 0.665), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_metal,
        name="angle_bushing",
    )
    for j, z in enumerate((0.31, 0.61, 0.74)):
        support.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(-0.22, -0.662, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=screw_metal,
            name=f"upright_screw_{j}",
        )

    tabletop = model.part("tabletop")
    depth = 0.82
    width = 1.12
    rear_clear = 0.060

    tabletop.visual(
        Box((depth, width, 0.036)),
        origin=_table_origin(rear_clear + depth / 2.0, 0.0, 0.0),
        material=wood,
        name="deck_panel",
    )
    # Built-up rim and lighter worn bevels around the board.
    tabletop.visual(
        Box((depth + 0.055, 0.038, 0.050)),
        origin=_table_origin(rear_clear + depth / 2.0, -width / 2.0 + 0.019, 0.018),
        material=dark_wood,
        name="near_side_raised_rim",
    )
    tabletop.visual(
        Box((depth + 0.055, 0.038, 0.050)),
        origin=_table_origin(rear_clear + depth / 2.0, width / 2.0 - 0.019, 0.018),
        material=dark_wood,
        name="far_side_raised_rim",
    )
    tabletop.visual(
        Box((0.046, width, 0.050)),
        origin=_table_origin(rear_clear + depth + 0.010, 0.0, 0.018),
        material=dark_wood,
        name="front_thick_lip",
    )
    tabletop.visual(
        Box((0.035, width, 0.040)),
        origin=_table_origin(rear_clear + 0.010, 0.0, 0.014),
        material=dark_wood,
        name="rear_hinge_lip",
    )
    tabletop.visual(
        Box((0.018, width - 0.18, 0.026)),
        origin=_table_origin(rear_clear + depth - 0.095, 0.0, 0.028),
        material=light_wood,
        name="paper_stop",
    )
    # Shallow inset-panel lines and a few restrained darker grain streaks.
    tabletop.visual(
        Box((0.006, width - 0.18, 0.004)),
        origin=_table_origin(rear_clear + 0.110, 0.0, 0.0195),
        material=dark_wood,
        name="rear_inset_line",
    )
    tabletop.visual(
        Box((0.006, width - 0.18, 0.004)),
        origin=_table_origin(rear_clear + depth - 0.120, 0.0, 0.0195),
        material=dark_wood,
        name="front_inset_line",
    )
    tabletop.visual(
        Box((depth - 0.22, 0.006, 0.004)),
        origin=_table_origin(rear_clear + depth / 2.0, -width / 2.0 + 0.080, 0.0195),
        material=dark_wood,
        name="near_inset_line",
    )
    tabletop.visual(
        Box((depth - 0.22, 0.006, 0.004)),
        origin=_table_origin(rear_clear + depth / 2.0, width / 2.0 - 0.080, 0.0195),
        material=dark_wood,
        name="far_inset_line",
    )
    for k, (x, y, length, width_y, material) in enumerate(
        (
            (0.19, -0.31, 0.38, 0.004, soft_grain),
            (0.27, -0.18, 0.46, 0.004, warm_grain),
            (0.35, -0.04, 0.40, 0.0035, soft_grain),
            (0.43, 0.15, 0.48, 0.004, warm_grain),
            (0.51, 0.30, 0.36, 0.0035, soft_grain),
            (0.60, -0.25, 0.32, 0.004, warm_grain),
            (0.68, -0.05, 0.28, 0.0035, soft_grain),
        )
    ):
        tabletop.visual(
            Box((length, width_y, 0.0016)),
            origin=_table_origin(rear_clear + x, y, 0.0185),
            material=material,
            name=f"wood_grain_{k}",
        )
    # Hinge collars rotate with the tabletop and capture the fixed metal rod.
    tabletop.visual(
        Cylinder(radius=0.024, length=0.12),
        origin=Origin(xyz=(0.0, -0.43, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_metal,
        name="near_hinge_barrel",
    )
    tabletop.visual(
        Cylinder(radius=0.024, length=0.12),
        origin=Origin(xyz=(0.0, 0.43, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_metal,
        name="far_hinge_barrel",
    )
    for suffix, y in (("near", -0.43), ("far", 0.43)):
        tabletop.visual(
            Box((0.080, 0.10, 0.010)),
            origin=Origin(xyz=(0.055, y, -0.002), rpy=(0.0, 0.12, 0.0)),
            material=black_metal,
            name=f"{suffix}_hinge_leaf",
        )
        tabletop.visual(
            Box((0.22, 0.030, 0.026)),
            origin=_table_origin(0.21, y, -0.030),
            material=black_metal,
            name=f"{suffix}_underside_angle_arm",
        )

    model.articulation(
        "frame_to_tabletop",
        ArticulationType.REVOLUTE,
        parent=support,
        child=tabletop,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.55, lower=-0.20, upper=0.65),
    )

    large_knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.070,
            0.030,
            body_style="lobed",
            base_diameter=0.045,
            top_diameter=0.060,
            crown_radius=0.002,
            grip=KnobGrip(style="ribbed", count=10, depth=0.002),
        ),
        "large_lock_knob",
    )
    small_knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.044,
            0.023,
            body_style="lobed",
            base_diameter=0.030,
            top_diameter=0.040,
            crown_radius=0.0015,
            grip=KnobGrip(style="ribbed", count=8, depth=0.0014),
        ),
        "angle_lock_knob",
    )

    lock_knob = model.part("lock_knob")
    lock_knob.visual(
        Cylinder(radius=0.010, length=0.032),
        origin=Origin(xyz=(0.0, -0.014, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=screw_metal,
        name="knob_stem",
    )
    lock_knob.visual(
        large_knob_mesh,
        origin=Origin(xyz=(0.0, -0.044, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_metal,
        name="lobed_grip",
    )
    angle_knob = model.part("angle_knob")
    angle_knob.visual(
        Cylinder(radius=0.007, length=0.026),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=screw_metal,
        name="angle_stem",
    )
    angle_knob.visual(
        small_knob_mesh,
        origin=Origin(xyz=(0.0, -0.034, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_metal,
        name="small_lobed_grip",
    )
    model.articulation(
        "support_to_lock_knob",
        ArticulationType.REVOLUTE,
        parent=support,
        child=lock_knob,
        origin=Origin(xyz=(-0.22, -0.668, 0.505)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "support_to_angle_knob",
        ArticulationType.REVOLUTE,
        parent=support,
        child=angle_knob,
        origin=Origin(xyz=(-0.23, -0.668, 0.665)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("support_frame")
    tabletop = object_model.get_part("tabletop")
    lock_knob = object_model.get_part("lock_knob")
    angle_knob = object_model.get_part("angle_knob")
    tilt = object_model.get_articulation("frame_to_tabletop")
    lock_joint = object_model.get_articulation("support_to_lock_knob")
    angle_joint = object_model.get_articulation("support_to_angle_knob")

    ctx.allow_overlap(
        frame,
        tabletop,
        elem_a="hinge_rod",
        elem_b="near_hinge_barrel",
        reason="The fixed steel hinge rod is intentionally captured inside the tabletop hinge barrel.",
    )
    ctx.allow_overlap(
        frame,
        tabletop,
        elem_a="hinge_rod",
        elem_b="far_hinge_barrel",
        reason="The fixed steel hinge rod is intentionally captured inside the tabletop hinge barrel.",
    )
    ctx.allow_overlap(
        frame,
        lock_knob,
        elem_a="lock_bushing",
        elem_b="knob_stem",
        reason="The hand-lock knob stem is intentionally seated through the side bushing.",
    )
    ctx.allow_overlap(
        frame,
        angle_knob,
        elem_a="angle_bushing",
        elem_b="angle_stem",
        reason="The small angle-lock stem is intentionally seated through the side bushing.",
    )

    ctx.expect_overlap(
        tabletop,
        frame,
        axes="y",
        elem_a="near_hinge_barrel",
        elem_b="hinge_rod",
        min_overlap=0.08,
        name="near hinge barrel captures rod along width",
    )
    ctx.expect_overlap(
        tabletop,
        frame,
        axes="y",
        elem_a="far_hinge_barrel",
        elem_b="hinge_rod",
        min_overlap=0.08,
        name="far hinge barrel captures rod along width",
    )
    ctx.expect_overlap(
        lock_knob,
        frame,
        axes="yz",
        elem_a="knob_stem",
        elem_b="lock_bushing",
        min_overlap=0.008,
        name="large lock knob stem is retained in bushing",
    )
    ctx.expect_overlap(
        angle_knob,
        frame,
        axes="yz",
        elem_a="angle_stem",
        elem_b="angle_bushing",
        min_overlap=0.006,
        name="small angle knob stem is retained in bushing",
    )
    ctx.expect_gap(
        tabletop,
        frame,
        axis="z",
        positive_elem="rear_hinge_lip",
        negative_elem="upper_cross_rail",
        min_gap=0.035,
        name="tilted tabletop clears upper support rail",
    )
    rest_center = ctx.part_world_aabb(tabletop)
    with ctx.pose({tilt: 0.55}):
        raised_center = ctx.part_world_aabb(tabletop)
        ctx.expect_gap(
            tabletop,
            frame,
            axis="z",
            positive_elem="deck_panel",
            negative_elem="lower_front_stretcher",
            min_gap=0.18,
            name="raised tabletop stays clear of lower stretcher",
        )
    ctx.check(
        "positive tilt joint raises the tabletop front edge",
        rest_center is not None
        and raised_center is not None
        and raised_center[0][2] > rest_center[0][2] + 0.06,
        details=f"rest_aabb={rest_center}, raised_aabb={raised_center}",
    )
    with ctx.pose({lock_joint: 0.75, angle_joint: -0.65}):
        ctx.expect_overlap(
            lock_knob,
            frame,
            axes="yz",
            elem_a="knob_stem",
            elem_b="lock_bushing",
            min_overlap=0.008,
            name="rotated large knob remains mounted",
        )
        ctx.expect_overlap(
            angle_knob,
            frame,
            axes="yz",
            elem_a="angle_stem",
            elem_b="angle_bushing",
            min_overlap=0.006,
            name="rotated small knob remains mounted",
        )

    return ctx.report()


object_model = build_object_model()
