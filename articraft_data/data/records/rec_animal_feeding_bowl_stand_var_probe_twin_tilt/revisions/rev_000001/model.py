from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _rect_profile(width: float, depth: float) -> list[tuple[float, float]]:
    hw = width / 2.0
    hd = depth / 2.0
    return [(-hw, -hd), (hw, -hd), (hw, hd), (-hw, hd)]


def _bowl_cup_mesh(name: str):
    # A thin, open stainless bowl that drops through the holder ring.  The
    # rolled rim is modeled as a separate torus so it can sit visibly on top of
    # the black stand ring without needing a collision allowance.
    outer = [
        (0.028, -0.090),
        (0.046, -0.084),
        (0.066, -0.065),
        (0.080, -0.034),
        (0.087, -0.006),
        (0.088, 0.004),
        (0.090, 0.0052),
        (0.095, 0.00942),
    ]
    inner = [
        (0.020, -0.078),
        (0.040, -0.073),
        (0.060, -0.056),
        (0.074, -0.028),
        (0.081, -0.004),
        (0.083, 0.004),
        (0.089, 0.00942),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer,
            inner,
            segments=72,
            lip_samples=5,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="raised_pet_bowl_stand")

    powder_black = model.material("powder_black", rgba=(0.02, 0.022, 0.020, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.82, 0.82, 0.78, 1.0))
    clamp_gray = model.material("clamp_gray", rgba=(0.23, 0.25, 0.24, 1.0))

    # Root: the one-piece black steel skid base and square upright post.
    base = model.part("base_frame")
    base.visual(
        Box((0.050, 0.540, 0.036)),
        origin=Origin(xyz=(-0.230, 0.0, 0.035)),
        material=powder_black,
        name="foot_0",
    )
    base.visual(
        Box((0.050, 0.540, 0.036)),
        origin=Origin(xyz=(0.230, 0.0, 0.035)),
        material=powder_black,
        name="foot_1",
    )
    base.visual(
        Box((0.510, 0.050, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=powder_black,
        name="base_crossbar",
    )
    base.visual(
        Box((0.036, 0.036, 0.420)),
        origin=Origin(xyz=(0.0, 0.0, 0.258)),
        material=powder_black,
        name="upright_post",
    )
    base.visual(
        Box((0.070, 0.090, 0.014)),
        origin=Origin(xyz=(-0.230, -0.250, 0.010)),
        material=dark_rubber,
        name="rubber_pad_0",
    )
    base.visual(
        Box((0.070, 0.090, 0.014)),
        origin=Origin(xyz=(-0.230, 0.250, 0.010)),
        material=dark_rubber,
        name="rubber_pad_1",
    )
    base.visual(
        Box((0.070, 0.090, 0.014)),
        origin=Origin(xyz=(0.230, -0.250, 0.010)),
        material=dark_rubber,
        name="rubber_pad_2",
    )
    base.visual(
        Box((0.070, 0.090, 0.014)),
        origin=Origin(xyz=(0.230, 0.250, 0.010)),
        material=dark_rubber,
        name="rubber_pad_3",
    )

    # Sliding collar and welded two-bowl ring bracket.  The collar is a true
    # hollow square loop clearanced around the upright post.
    carriage = model.part("height_carriage")
    carriage.visual(
        Box((0.098, 0.025, 0.086)),
        origin=Origin(xyz=(0.0, 0.0305, 0.0)),
        material=clamp_gray,
        name="collar_front",
    )
    carriage.visual(
        Box((0.098, 0.025, 0.086)),
        origin=Origin(xyz=(0.0, -0.0305, 0.0)),
        material=clamp_gray,
        name="collar_back",
    )
    carriage.visual(
        Box((0.025, 0.048, 0.086)),
        origin=Origin(xyz=(-0.0305, 0.0, 0.0)),
        material=clamp_gray,
        name="collar_side_0",
    )
    carriage.visual(
        Box((0.025, 0.048, 0.086)),
        origin=Origin(xyz=(0.0305, 0.0, 0.0)),
        material=clamp_gray,
        name="collar_side_1",
    )

    ring_mesh = mesh_from_geometry(
        TorusGeometry(0.096, 0.0048, radial_segments=20, tubular_segments=72),
        "bowl_holder_ring",
    )
    for x, visual_name in ((-0.168, "ring_0"), (0.168, "ring_1")):
        carriage.visual(
            ring_mesh,
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=powder_black,
            name=visual_name,
        )

    # Short welded tabs bridge each circular ring to the collar, avoiding a
    # floating hoop while keeping the bowl clearance open.
    carriage.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(-0.043, 0.043, 0.0), (-0.080, 0.065, 0.0), (-0.104, 0.076, 0.0)],
                radius=0.0045,
                samples_per_segment=8,
                radial_segments=12,
            ),
            "tab_0_tube",
        ),
        material=powder_black,
        name="tab_0",
    )
    carriage.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(0.043, 0.043, 0.0), (0.080, 0.065, 0.0), (0.104, 0.076, 0.0)],
                radius=0.0045,
                samples_per_segment=8,
                radial_segments=12,
            ),
            "tab_1_tube",
        ),
        material=powder_black,
        name="tab_1",
    )
    carriage.visual(
        Box((0.020, 0.032, 0.024)),
        origin=Origin(xyz=(0.0, 0.059, -0.020)),
        material=powder_black,
        name="clamp_backbone",
    )

    # Yoke pivot brackets: one vertical plate per side of each bowl, mounted
    # just outside the rim on the ±Y flanks.  They provide the trunnion
    # bearings for the independent Y-axis tilt mechanism.
    for idx, x in enumerate((-0.168, 0.168)):
        for bracket_idx, dy in enumerate((-0.108, 0.108)):
            carriage.visual(
                Box((0.014, 0.016, 0.050)),
                origin=Origin(xyz=(x, dy, 0.025)),
                material=powder_black,
                name=f"yoke_{idx}_{bracket_idx}",
            )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.295)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.08, lower=-0.110, upper=0.080),
    )

    # Two independently tiltable bowls on trunnion pivots.  Each bowl has a pair
    # of horizontal trunnion stubs that seat in the yoke bracket bearings,
    # allowing ~0–25° forward tilt for easier access.
    for idx, x in enumerate((-0.168, 0.168)):
        bowl = model.part(f"bowl_{idx}")
        bowl.visual(_bowl_cup_mesh(f"bowl_{idx}_cup"), material=stainless, name="cup")
        bowl.visual(
            mesh_from_geometry(
                TorusGeometry(0.099, 0.0050, radial_segments=18, tubular_segments=72),
                f"bowl_{idx}_rolled_rim",
            ),
            origin=Origin(xyz=(0.0, 0.0, 0.00942)),
            material=stainless,
            name="rim",
        )
        bowl.visual(
            Cylinder(radius=0.026, length=0.005),
            origin=Origin(xyz=(0.0, 0.0, -0.0875)),
            material=stainless,
            name="flat_bottom",
        )
        # Trunnion pivot stubs: horizontal pins along ±Y that ride in the
        # yoke bracket bearings, providing the visible tilt axis.
        for stub_idx, dy in enumerate((-0.094, 0.094)):
            bowl.visual(
                Cylinder(radius=0.005, length=0.024),
                origin=Origin(xyz=(0.0, dy, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=stainless,
                name=f"trunnion_{stub_idx}",
            )
        model.articulation(
            f"carriage_to_bowl_{idx}",
            ArticulationType.REVOLUTE,
            parent=carriage,
            child=bowl,
            origin=Origin(xyz=(x, 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=1.5,
                lower=0.0,
                upper=math.radians(25.0),
            ),
        )

    # Visible clamp knobs on the collar are rotary threaded controls.  They
    # clear the bowls by protruding fore/aft rather than along the bowl span.
    large_knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.052,
            0.026,
            body_style="lobed",
            base_diameter=0.034,
            top_diameter=0.046,
            crown_radius=0.0015,
            grip=KnobGrip(style="ribbed", count=8, depth=0.0012),
            bore=KnobBore(style="round", diameter=0.009),
        ),
        "large_clamp_knob",
    )
    small_knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.032,
            0.020,
            body_style="lobed",
            base_diameter=0.022,
            top_diameter=0.030,
            crown_radius=0.0010,
            grip=KnobGrip(style="ribbed", count=6, depth=0.0009),
            bore=KnobBore(style="round", diameter=0.006),
        ),
        "small_clamp_knob",
    )

    knob_0 = model.part("clamp_knob_0")
    knob_0.visual(
        Cylinder(radius=0.006, length=0.030),
        origin=Origin(xyz=(0.0, -0.015, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=powder_black,
        name="threaded_stem",
    )
    knob_0.visual(
        large_knob_mesh,
        origin=Origin(xyz=(0.0, -0.041, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=powder_black,
        name="hand_grip",
    )
    model.articulation(
        "carriage_to_knob_0",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=knob_0,
        origin=Origin(xyz=(0.0, -0.043, 0.010)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0),
    )

    knob_1 = model.part("clamp_knob_1")
    knob_1.visual(
        Cylinder(radius=0.0045, length=0.024),
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=powder_black,
        name="threaded_stem",
    )
    knob_1.visual(
        small_knob_mesh,
        origin=Origin(xyz=(0.0, 0.034, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=powder_black,
        name="hand_grip",
    )
    model.articulation(
        "carriage_to_knob_1",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=knob_1,
        origin=Origin(xyz=(0.0, 0.049, -0.012)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    carriage = object_model.get_part("height_carriage")
    bowl_0 = object_model.get_part("bowl_0")
    bowl_1 = object_model.get_part("bowl_1")
    height_joint = object_model.get_articulation("base_to_carriage")

    ctx.allow_overlap(
        "clamp_knob_1",
        "height_carriage",
        elem_a="threaded_stem",
        elem_b="clamp_backbone",
        reason="The small clamp knob's threaded stem is intentionally shown screwed through the collar backbone.",
    )
    for idx in range(2):
        for stub_idx in range(2):
            ctx.allow_overlap(
                f"bowl_{idx}",
                "height_carriage",
                elem_a=f"trunnion_{stub_idx}",
                elem_b=f"yoke_{idx}_{stub_idx}",
                reason="Trunnion pivot stub is intentionally captured in the yoke bracket bearing for tilt articulation.",
            )
        # The inner portion of each trunnion pin passes through the holder ring
        # torus on its way to the outer yoke bracket.
        ctx.allow_overlap(
            f"bowl_{idx}",
            "height_carriage",
            elem_a="trunnion_0",
            elem_b=f"ring_{idx}",
            reason="Trunnion pin passes through the holder ring bore on its way to the outer yoke bracket bearing.",
        )
        ctx.allow_overlap(
            f"bowl_{idx}",
            "height_carriage",
            elem_a="trunnion_1",
            elem_b=f"ring_{idx}",
            reason="Trunnion pin passes through the holder ring bore on its way to the outer yoke bracket bearing.",
        )

    ctx.expect_overlap(
        bowl_0,
        carriage,
        axes="xy",
        min_overlap=0.14,
        elem_a="rim",
        elem_b="ring_0",
        name="bowl 0 rim is seated over its holder ring",
    )
    ctx.expect_gap(
        bowl_0,
        carriage,
        axis="z",
        max_gap=0.001,
        max_penetration=0.001,
        positive_elem="rim",
        negative_elem="ring_0",
        name="bowl 0 rim rests just above holder ring",
    )
    ctx.expect_overlap(
        bowl_1,
        carriage,
        axes="xy",
        min_overlap=0.14,
        elem_a="rim",
        elem_b="ring_1",
        name="bowl 1 rim is seated over its holder ring",
    )
    ctx.expect_gap(
        bowl_1,
        carriage,
        axis="z",
        max_gap=0.001,
        max_penetration=0.001,
        positive_elem="rim",
        negative_elem="ring_1",
        name="bowl 1 rim rests just above holder ring",
    )
    ctx.expect_origin_distance(
        bowl_0,
        bowl_1,
        axes="x",
        min_dist=0.30,
        max_dist=0.36,
        name="two bowls keep a realistic side-by-side spacing",
    )

    # Tilt joint verification: each bowl has an independent REVOLUTE tilt joint
    # that allows ~0-25° forward tilt.  At max tilt both bowls must maintain
    # clearance with each other and with the carriage yoke brackets.
    tilt_0 = object_model.get_articulation("carriage_to_bowl_0")
    tilt_1 = object_model.get_articulation("carriage_to_bowl_1")
    max_tilt = math.radians(25.0)

    ctx.check(
        "bowl 0 tilt joint is revolute",
        tilt_0.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={tilt_0.articulation_type}",
    )
    ctx.check(
        "bowl 1 tilt joint is revolute",
        tilt_1.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={tilt_1.articulation_type}",
    )

    # At max tilt both bowls still maintain side-by-side X separation (no collision).
    with ctx.pose({tilt_0: max_tilt, tilt_1: max_tilt}):
        ctx.expect_origin_distance(
            bowl_0,
            bowl_1,
            axes="x",
            min_dist=0.20,
            name="tilted bowls maintain inter-bowl clearance at max tilt",
        )

    # Verify that tilting bowl 0 actually moves its rim in the expected direction.
    # The part origin is at the pivot center, so measure the rim visual instead.
    rest_rim_center = ctx.part_element_world_aabb(bowl_0, elem="rim")
    rest_rim_z = rest_rim_center[0][2]  # min Z of rim AABB
    with ctx.pose({tilt_0: max_tilt}):
        tilted_rim_center = ctx.part_element_world_aabb(bowl_0, elem="rim")
        tilted_rim_z = tilted_rim_center[0][2]
    ctx.check(
        "bowl 0 tilt moves the rim at max angle",
        abs(tilted_rim_z - rest_rim_z) > 0.005,
        details=f"rest_rim_z={rest_rim_z:.4f}, tilted_rim_z={tilted_rim_z:.4f}",
    )

    for pose_name, q in (("low adjustment", -0.110), ("rest adjustment", 0.0), ("high adjustment", 0.080)):
        with ctx.pose({height_joint: q}):
            ctx.expect_overlap(
                carriage,
                base,
                axes="z",
                min_overlap=0.075,
                elem_a="collar_front",
                elem_b="upright_post",
                name=f"{pose_name} collar remains engaged with the upright post",
            )
            ctx.expect_origin_distance(
                carriage,
                base,
                axes="xy",
                max_dist=0.001,
                name=f"{pose_name} upright stays centered inside collar footprint",
            )

    ctx.expect_overlap(
        "clamp_knob_1",
        carriage,
        axes="y",
        min_overlap=0.010,
        elem_a="threaded_stem",
        elem_b="clamp_backbone",
        name="small clamp screw remains engaged through the collar backbone",
    )

    rest_z = ctx.part_world_position(bowl_0)[2]
    with ctx.pose({height_joint: 0.080}):
        high_z = ctx.part_world_position(bowl_0)[2]
    ctx.check(
        "height adjustment raises the seated bowls",
        high_z > rest_z + 0.070,
        details=f"rest_z={rest_z:.3f}, high_z={high_z:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
