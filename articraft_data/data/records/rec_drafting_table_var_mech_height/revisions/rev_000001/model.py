from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _beam_orientation(p0: tuple[float, float, float], p1: tuple[float, float, float]) -> tuple[Origin, float]:
    """Return an origin that aligns a local +Z beam from p0 to p1."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("beam endpoints must be distinct")
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    origin = Origin(
        xyz=((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5),
        rpy=(0.0, pitch, yaw),
    )
    return origin, length


def _rect_beam(part, p0, p1, size: float, material, name: str, overshoot: float = 0.018) -> None:
    origin, length = _beam_orientation(p0, p1)
    part.visual(
        Box((size, size, length + overshoot)),
        origin=origin,
        material=material,
        name=name,
    )


def _round_beam(part, p0, p1, radius: float, material, name: str, overshoot: float = 0.012) -> None:
    origin, length = _beam_orientation(p0, p1)
    part.visual(
        Cylinder(radius=radius, length=length + overshoot),
        origin=origin,
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="tilting_drafting_table",
        meta={"reference_category": "Workspace / Drafting table"},
    )

    black_metal = model.material("powder_coated_black_metal", rgba=(0.015, 0.017, 0.016, 1.0))
    worn_edges = model.material("slightly_worn_dark_edges", rgba=(0.05, 0.055, 0.052, 1.0))
    wood = model.material("warm_reddish_drafting_wood", rgba=(0.46, 0.20, 0.075, 1.0))
    wood_dark = model.material("darker_wood_grain", rgba=(0.20, 0.085, 0.035, 1.0))
    wood_mid = model.material("burnished_wood_grain", rgba=(0.34, 0.14, 0.050, 1.0))
    wood_highlight = model.material("subtle_amber_grain", rgba=(0.58, 0.25, 0.090, 1.0))
    grey = model.material("gunmetal_hardware", rgba=(0.19, 0.20, 0.19, 1.0))
    plastic = model.material("black_plastic_knobs", rgba=(0.01, 0.011, 0.012, 1.0))
    pale_panel = model.material("pale_drawer_faces", rgba=(0.78, 0.76, 0.68, 1.0))
    steel_tube = model.material("bright_steel_inner_tube", rgba=(0.14, 0.15, 0.14, 1.0))

    # Real-world scale: compact drafting table ~1.25 m wide, ~0.8 m deep.
    tube = 0.035
    outer_tube = 0.042  # outer sleeve cross-section (receives inner tube)
    inner_tube = 0.030  # inner sliding tube cross-section
    side_x = 0.55

    # ============================================================
    # BASE — grounded outer structure (floor runners + outer sleeves)
    # ============================================================
    base = model.part("base")

    for idx, sx in enumerate((-1.0, 1.0)):
        x = sx * side_x
        # Floor runners (horizontal base feet, front-to-back)
        _rect_beam(base, (x, -0.50, 0.040), (x, 0.50, 0.040), tube, black_metal, f"floor_runner_{idx}")
        # Outer front leg sleeve — lower portion of front leg, wider square tube
        _rect_beam(base, (x, -0.46, 0.035), (x, -0.378, 0.420), outer_tube, black_metal, f"outer_front_sleeve_{idx}")
        # Outer rear leg sleeve — lower portion of rear leg, wider square tube
        _rect_beam(base, (x, 0.46, 0.035), (x, 0.367, 0.459), outer_tube, black_metal, f"outer_rear_sleeve_{idx}")
        # Sleeve clamp collars — visible telescoping interface at top of each sleeve
        base.visual(
            Box((0.058, 0.058, 0.032)),
            origin=Origin(xyz=(x, -0.378, 0.434)),
            material=grey,
            name=f"front_sleeve_collar_{idx}",
        )
        base.visual(
            Box((0.058, 0.058, 0.032)),
            origin=Origin(xyz=(x, 0.367, 0.474)),
            material=grey,
            name=f"rear_sleeve_collar_{idx}",
        )

    # Cross-braces for base rigidity (at z=0.15 where outer sleeves are at y≈±0.435,
    # well below inner tube start at z=0.200 to avoid base↔carriage overlap)
    _rect_beam(base, (-0.55, -0.435, 0.15), (0.55, -0.435, 0.15), 0.028, black_metal, "front_lower_crossbar")
    _rect_beam(base, (-0.55, 0.435, 0.15), (0.55, 0.435, 0.15), 0.028, black_metal, "rear_lower_crossbar")

    # ============================================================
    # CARRIAGE — movable inner structure (inner tubes + upper frame)
    # ============================================================
    carriage = model.part("carriage")

    for idx, sx in enumerate((-1.0, 1.0)):
        x = sx * side_x
        # Inner front tube — slides inside outer front sleeve
        _rect_beam(carriage, (x, -0.425, 0.200), (x, -0.31, 0.735), inner_tube, steel_tube, f"inner_front_tube_{idx}")
        # Inner rear tube — slides inside outer rear sleeve
        _rect_beam(carriage, (x, 0.424, 0.200), (x, 0.29, 0.805), inner_tube, steel_tube, f"inner_rear_tube_{idx}")
        # Top side rails — connect front and rear inner tube tops
        _rect_beam(carriage, (x, -0.34, 0.730), (x, 0.34, 0.810), tube, black_metal, f"top_side_rail_{idx}")
        # Diagonal side braces within the carriage frame
        _rect_beam(carriage, (x, -0.34, 0.730), (x, 0.05, 0.400), 0.026, black_metal, f"side_brace_{idx}")

        # Ratchet/angle plates on both sides: toothed strip under the tilting top
        _rect_beam(carriage, (sx * 0.525, -0.20, 0.635), (sx * 0.525, 0.24, 0.845), 0.020, grey, f"ratchet_backbone_{idx}")
        for tooth in range(6):
            y = -0.15 + tooth * 0.065
            z = 0.66 + tooth * 0.032
            carriage.visual(
                Box((0.026, 0.035, 0.020)),
                origin=Origin(xyz=(sx * 0.525, y, z), rpy=(0.0, 0.0, 0.0)),
                material=grey,
                name=f"ratchet_tooth_{idx}_{tooth}",
            )

        # Side bushings that receive the lock-knob shafts
        carriage.visual(
            Cylinder(radius=0.030, length=0.034),
            origin=Origin(xyz=(sx * 0.552, -0.175, 0.455), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=grey,
            name=f"knob_socket_{idx}",
        )
        # Knob mount tabs
        carriage.visual(
            Box((0.020, 0.040, 0.100)),
            origin=Origin(xyz=(sx * 0.552, -0.175, 0.525)),
            material=grey,
            name=f"knob_mount_tab_{idx}",
        )

    # Cross-rails on the carriage
    _rect_beam(carriage, (-0.58, -0.365, 0.722), (0.58, -0.365, 0.722), tube, black_metal, "front_hinge_rail")
    _rect_beam(carriage, (-0.55, 0.325, 0.805), (0.55, 0.325, 0.805), tube, black_metal, "rear_top_rail")
    _rect_beam(carriage, (-0.52, -0.385, 0.655), (0.52, -0.385, 0.655), 0.030, black_metal, "tray_front_rail")
    _rect_beam(carriage, (-0.52, -0.205, 0.655), (0.52, -0.205, 0.655), 0.030, black_metal, "tray_rear_rail")

    # Front supply tray/cubby shelf mounted to the carriage
    carriage.visual(
        Box((1.08, 0.18, 0.020)),
        origin=Origin(xyz=(0.0, -0.295, 0.645)),
        material=black_metal,
        name="front_supply_tray",
    )
    for i, x in enumerate((-0.30, 0.0, 0.30)):
        carriage.visual(
            Box((0.018, 0.18, 0.065)),
            origin=Origin(xyz=(x, -0.295, 0.685)),
            material=black_metal,
            name=f"tray_divider_{i}",
        )
    for i, x in enumerate((-0.42, -0.14, 0.14, 0.42)):
        carriage.visual(
            Box((0.22, 0.012, 0.040)),
            origin=Origin(xyz=(x, -0.392, 0.682)),
            material=pale_panel,
            name=f"drawer_face_{i}",
        )

    # Hinge pin — captured steel rod through the table barrel
    carriage.visual(
        Cylinder(radius=0.012, length=1.18),
        origin=Origin(xyz=(0.0, -0.365, 0.755), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grey,
        name="hinge_pin",
    )
    for idx, sx in enumerate((-1.0, 1.0)):
        carriage.visual(
            Box((0.030, 0.050, 0.055)),
            origin=Origin(xyz=(sx * 0.585, -0.365, 0.745)),
            material=grey,
            name=f"hinge_end_bracket_{idx}",
        )

    # ============================================================
    # TABLETOP — tilting drawing surface
    # ============================================================
    tabletop = model.part("tabletop")
    tabletop.visual(
        Box((1.20, 0.76, 0.040)),
        origin=Origin(xyz=(0.0, 0.380, 0.035)),
        material=wood,
        name="wood_drawing_board",
    )
    # Dark perimeter lips
    tabletop.visual(
        Box((1.26, 0.045, 0.060)),
        origin=Origin(xyz=(0.0, 0.015, 0.070)),
        material=worn_edges,
        name="front_lip",
    )
    tabletop.visual(
        Box((1.24, 0.038, 0.050)),
        origin=Origin(xyz=(0.0, 0.765, 0.065)),
        material=worn_edges,
        name="rear_lip",
    )
    for idx, sx in enumerate((-1.0, 1.0)):
        tabletop.visual(
            Box((0.045, 0.78, 0.055)),
            origin=Origin(xyz=(sx * 0.625, 0.390, 0.065)),
            material=worn_edges,
            name=f"side_lip_{idx}",
        )
        tabletop.visual(
            Box((0.030, 0.52, 0.020)),
            origin=Origin(xyz=(sx * 0.555, 0.395, 0.065)),
            material=black_metal,
            name=f"paper_channel_{idx}",
        )

    # Front paper stop / parallel rule bar
    tabletop.visual(
        Box((1.04, 0.022, 0.020)),
        origin=Origin(xyz=(0.0, 0.105, 0.065)),
        material=black_metal,
        name="paper_stop",
    )
    # Tilt barrel (hinge cylinder on the tabletop side)
    tabletop.visual(
        Cylinder(radius=0.021, length=1.12),
        origin=Origin(xyz=(0.0, -0.006, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grey,
        name="tilt_barrel",
    )

    # Wood grain detail strips
    for i, (x, y, length, width_y, material) in enumerate(
        (
            (-0.12, 0.155, 0.78, 0.006, wood_mid),
            (-0.06, 0.195, 0.84, 0.004, wood_highlight),
            (-0.18, 0.240, 0.72, 0.005, wood_dark),
            (0.02, 0.285, 0.86, 0.004, wood_mid),
            (-0.08, 0.330, 0.80, 0.005, wood_highlight),
            (0.10, 0.375, 0.70, 0.004, wood_dark),
            (-0.04, 0.420, 0.82, 0.005, wood_mid),
            (0.05, 0.465, 0.76, 0.004, wood_highlight),
            (-0.16, 0.510, 0.68, 0.005, wood_dark),
            (0.00, 0.555, 0.84, 0.004, wood_mid),
            (-0.09, 0.605, 0.74, 0.005, wood_highlight),
            (0.12, 0.650, 0.62, 0.004, wood_dark),
        )
    ):
        tabletop.visual(
            Box((length, width_y, 0.0018)),
            origin=Origin(xyz=(x, y, 0.056), rpy=(0.0, 0.0, 0.025 * (-1 if i % 2 else 1))),
            material=material,
            name=f"wood_grain_{i}",
        )

    # ============================================================
    # ARTICULATIONS
    # ============================================================

    # 1. Prismatic height-adjust: base → carriage (vertical travel ~0.15 m)
    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.08, lower=0.0, upper=0.15),
    )

    # 2. Revolute tilt: carriage → tabletop
    tilt_origin = Origin(xyz=(0.0, -0.365, 0.755), rpy=(math.radians(20.0), 0.0, 0.0))
    model.articulation(
        "frame_to_tabletop",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=tabletop,
        origin=tilt_origin,
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.45, lower=-0.30, upper=0.55),
    )

    # 3. Lock knobs (children of carriage, spin in place)
    for idx, sx in enumerate((-1.0, 1.0)):
        knob = model.part(f"lock_knob_{idx}")
        knob.visual(
            Cylinder(radius=0.055, length=0.026),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=plastic,
            name="knob_disk",
        )
        knob.visual(
            Cylinder(radius=0.014, length=0.090),
            origin=Origin(xyz=(-sx * 0.045, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=grey,
            name="knob_shaft",
        )
        for k, angle in enumerate((0.0, math.pi / 3.0, 2.0 * math.pi / 3.0)):
            knob.visual(
                Box((0.012, 0.080, 0.010)),
                origin=Origin(xyz=(sx * 0.026, 0.0, 0.004), rpy=(0.0, math.pi / 2.0, angle)),
                material=grey,
                name=f"grip_rib_{k}",
            )
        model.articulation(
            f"frame_to_lock_knob_{idx}",
            ArticulationType.REVOLUTE,
            parent=carriage,
            child=knob,
            origin=Origin(xyz=(sx * 0.600, -0.175, 0.455)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-math.pi, upper=math.pi),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    tabletop = object_model.get_part("tabletop")
    height_joint = object_model.get_articulation("base_to_carriage")
    tilt = object_model.get_articulation("frame_to_tabletop")

    # ── Telescoping leg overlap allowances ──────────────────────
    # Inner tubes intentionally slide inside outer sleeves.
    for idx in (0, 1):
        ctx.allow_overlap(
            base, carriage,
            elem_a=f"outer_front_sleeve_{idx}",
            elem_b=f"inner_front_tube_{idx}",
            reason=f"Inner front tube {idx} is intentionally represented sliding inside the outer front sleeve.",
        )
        ctx.allow_overlap(
            base, carriage,
            elem_a=f"outer_rear_sleeve_{idx}",
            elem_b=f"inner_rear_tube_{idx}",
            reason=f"Inner rear tube {idx} is intentionally represented sliding inside the outer rear sleeve.",
        )
        # Sleeve collars wrap around the inner tubes at the clamp interface.
        ctx.allow_overlap(
            base, carriage,
            elem_a=f"front_sleeve_collar_{idx}",
            elem_b=f"inner_front_tube_{idx}",
            reason=f"Front sleeve collar {idx} wraps around the inner tube as a visible clamp.",
        )
        ctx.allow_overlap(
            base, carriage,
            elem_a=f"rear_sleeve_collar_{idx}",
            elem_b=f"inner_rear_tube_{idx}",
            reason=f"Rear sleeve collar {idx} wraps around the inner tube as a visible clamp.",
        )

    # ── Hinge pin / barrel (now carriage ↔ tabletop) ───────────
    ctx.allow_overlap(
        carriage, tabletop,
        elem_a="hinge_pin",
        elem_b="tilt_barrel",
        reason="The tabletop hinge barrel is intentionally represented around the carriage hinge pin.",
    )
    ctx.expect_overlap(
        carriage, tabletop,
        axes="x",
        elem_a="hinge_pin",
        elem_b="tilt_barrel",
        min_overlap=1.05,
        name="hinge barrel spans the fixed pin",
    )
    ctx.expect_overlap(
        carriage, tabletop,
        axes="y",
        elem_a="hinge_pin",
        elem_b="tilt_barrel",
        min_overlap=0.020,
        name="hinge barrel is coaxial with pin",
    )

    # ── Lock knob tests (now carriage ↔ lock_knob) ─────────────
    for idx in (0, 1):
        knob = object_model.get_part(f"lock_knob_{idx}")
        joint = object_model.get_articulation(f"frame_to_lock_knob_{idx}")
        ctx.allow_overlap(
            carriage, knob,
            elem_a=f"knob_socket_{idx}",
            elem_b="knob_shaft",
            reason="The lock-knob shaft intentionally seats in the carriage side bushing.",
        )
        ctx.expect_overlap(
            carriage, knob,
            axes="x",
            elem_a=f"knob_socket_{idx}",
            elem_b="knob_shaft",
            min_overlap=0.012,
            name=f"lock knob {idx} shaft enters side socket",
        )
        before = ctx.part_world_position(knob)
        with ctx.pose({joint: 1.2}):
            after = ctx.part_world_position(knob)
        ctx.check(
            f"lock knob {idx} spins in place",
            before is not None and after is not None and max(abs(before[i] - after[i]) for i in range(3)) < 1e-6,
            details=f"before={before}, after={after}",
        )

    # ── Tabletop tilt test ─────────────────────────────────────
    rest_aabb = ctx.part_world_aabb(tabletop)
    with ctx.pose({tilt: 0.45}):
        raised_aabb = ctx.part_world_aabb(tabletop)
    ctx.check(
        "positive tabletop tilt raises rear edge",
        rest_aabb is not None and raised_aabb is not None and raised_aabb[1][2] > rest_aabb[1][2] + 0.12,
        details=f"rest={rest_aabb}, raised={raised_aabb}",
    )
    ctx.expect_overlap(
        carriage, tabletop,
        axes="x",
        elem_b="wood_drawing_board",
        min_overlap=1.0,
        name="wide table top is carried by the carriage width",
    )

    # ── Height-adjust prismatic joint tests ─────────────────────
    # Inner tubes stay centered in outer sleeves on x-axis
    for idx in (0, 1):
        ctx.expect_within(
            carriage, base,
            axes="x",
            inner_elem=f"inner_front_tube_{idx}",
            outer_elem=f"outer_front_sleeve_{idx}",
            margin=0.008,
            name=f"inner front tube {idx} centered in outer sleeve (x)",
        )
        ctx.expect_within(
            carriage, base,
            axes="x",
            inner_elem=f"inner_rear_tube_{idx}",
            outer_elem=f"outer_rear_sleeve_{idx}",
            margin=0.008,
            name=f"inner rear tube {idx} centered in outer sleeve (x)",
        )

    # Retained insertion at rest: inner tubes overlap with outer sleeves on z
    for idx in (0, 1):
        ctx.expect_overlap(
            carriage, base,
            axes="z",
            elem_a=f"inner_front_tube_{idx}",
            elem_b=f"outer_front_sleeve_{idx}",
            min_overlap=0.15,
            name=f"front tube {idx} well-inserted in sleeve at rest",
        )
        ctx.expect_overlap(
            carriage, base,
            axes="z",
            elem_a=f"inner_rear_tube_{idx}",
            elem_b=f"outer_rear_sleeve_{idx}",
            min_overlap=0.15,
            name=f"rear tube {idx} well-inserted in sleeve at rest",
        )

    # Height joint raises the carriage by the configured travel
    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({height_joint: 0.15}):
        raised_pos = ctx.part_world_position(carriage)
    ctx.check(
        "base_to_carriage raises carriage by ~0.15 m",
        rest_pos is not None and raised_pos is not None
        and abs((raised_pos[2] - rest_pos[2]) - 0.15) < 0.002,
        details=f"rest_z={rest_pos[2] if rest_pos else None}, "
                f"raised_z={raised_pos[2] if raised_pos else None}",
    )

    # Retained insertion at max extension: tubes still inside sleeves
    with ctx.pose({height_joint: 0.15}):
        for idx in (0, 1):
            ctx.expect_overlap(
                carriage, base,
                axes="z",
                elem_a=f"inner_front_tube_{idx}",
                elem_b=f"outer_front_sleeve_{idx}",
                min_overlap=0.04,
                name=f"front tube {idx} still inserted at max height",
            )
            ctx.expect_overlap(
                carriage, base,
                axes="z",
                elem_a=f"inner_rear_tube_{idx}",
                elem_b=f"outer_rear_sleeve_{idx}",
                min_overlap=0.04,
                name=f"rear tube {idx} still inserted at max height",
            )

    return ctx.report()


object_model = build_object_model()
