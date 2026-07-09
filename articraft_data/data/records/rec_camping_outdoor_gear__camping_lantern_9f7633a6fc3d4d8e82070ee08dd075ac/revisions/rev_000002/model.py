from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


BASE_HEIGHT = 0.105
SLIDE_TRAVEL = 0.070
LANTERN_RADIUS = 0.064


def _ring_profile_mesh(
    name: str,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    *,
    segments: int = 96,
):
    """Thin-walled revolved shell with visible lips/ridges."""
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=segments,
            start_cap="flat",
            end_cap="flat",
            lip_samples=4,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="compact_collapsible_camping_lantern",
        meta={
            "run_notes": (
                "Primary reference reads as a compact collapsible camping lantern; "
                "no category/image conflict suspected."
            )
        },
    )

    black = model.material("satin_black_plastic", rgba=(0.005, 0.005, 0.004, 1.0))
    dark = model.material("dark_graphite_rubber", rgba=(0.020, 0.020, 0.018, 1.0))
    clear = model.material("slightly_smoked_clear_plastic", rgba=(0.70, 0.92, 1.0, 0.32))
    chrome = model.material("polished_wire_steel", rgba=(0.82, 0.86, 0.86, 1.0))
    white = model.material("white_led_carrier", rgba=(0.91, 0.93, 0.90, 1.0))
    led = model.material("warm_led_lenses", rgba=(1.0, 0.86, 0.42, 1.0))
    screw = model.material("small_silver_fasteners", rgba=(0.72, 0.74, 0.72, 1.0))

    lower_base = model.part("lower_base")
    lower_base.visual(
        _ring_profile_mesh(
            "ribbed_lower_base_shell",
            [
                (0.056, 0.000),
                (0.064, 0.006),
                (0.064, 0.017),
                (0.059, 0.023),
                (0.059, 0.035),
                (0.062, 0.039),
                (0.062, 0.045),
                (0.058, 0.049),
                (0.058, 0.085),
                (0.062, 0.090),
                (0.062, 0.100),
                (0.058, 0.105),
            ],
            [
                (0.047, 0.006),
                (0.052, 0.014),
                (0.054, 0.095),
                (0.054, 0.105),
            ],
        ),
        material=black,
        name="base_shell",
    )
    # Vertical grip ribs on the lower canister, as in the reference photo.
    for i in range(18):
        angle = 2.0 * math.pi * i / 18
        r = 0.0625
        lower_base.visual(
            Box((0.006, 0.010, 0.047)),
            origin=Origin(
                xyz=(r * math.cos(angle), r * math.sin(angle), 0.054),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark,
            name=f"base_rib_{i}",
        )
    # Bottom castellated grip pads.
    for i in range(18):
        angle = 2.0 * math.pi * (i + 0.5) / 18
        r = 0.062
        lower_base.visual(
            Box((0.007, 0.014, 0.015)),
            origin=Origin(
                xyz=(r * math.cos(angle), r * math.sin(angle), 0.008),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark,
            name=f"foot_lug_{i}",
        )

    upper_lantern = model.part("upper_lantern")
    # Hidden telescoping skirt: retained inside the hollow base sleeve.
    upper_lantern.visual(
        _ring_profile_mesh(
            "sliding_inner_skirt",
            [(0.049, -0.095), (0.049, 0.000)],
            [(0.044, -0.095), (0.044, 0.000)],
            segments=72,
        ),
        material=dark,
        name="skirt_sleeve",
    )
    upper_lantern.visual(
        mesh_from_geometry(
            LatheGeometry(
                [(0.056, 0.000), (0.062, 0.000), (0.062, 0.003), (0.056, 0.003)],
                segments=96,
                closed=True,
            ),
            "base_rim_seating_lip",
        ),
        material=black,
        name="seat_lip",
    )
    upper_lantern.visual(
        _ring_profile_mesh(
            "lower_light_chamber_collar",
            [(0.051, 0.000), (0.060, 0.004), (0.060, 0.015), (0.052, 0.019)],
            [(0.042, 0.000), (0.045, 0.019)],
            segments=96,
        ),
        material=black,
        name="lower_collar",
    )
    # Transparent cylindrical chamber, built as a thin hollow tube rather than a solid plug.
    upper_lantern.visual(
        mesh_from_geometry(
            LatheGeometry(
                [(0.052, 0.015), (0.052, 0.137), (0.049, 0.137), (0.049, 0.015)],
                segments=96,
                closed=True,
            ),
            "transparent_light_chamber_tube",
        ),
        material=clear,
        name="clear_chamber",
    )
    # Subtle vertical clear facets/guards on the transparent tube.
    for i in range(4):
        angle = 2.0 * math.pi * i / 4 + math.pi / 4
        r = 0.0525
        upper_lantern.visual(
            Box((0.006, 0.004, 0.118)),
            origin=Origin(
                xyz=(r * math.cos(angle), r * math.sin(angle), 0.077),
                rpy=(0.0, 0.0, angle),
            ),
            material=clear,
            name=f"clear_post_{i}",
        )

    # Central LED strip and warm lens dots inside the chamber.
    upper_lantern.visual(
        Box((0.030, 0.010, 0.112)),
        origin=Origin(xyz=(0.0, -0.014, 0.078)),
        material=white,
        name="led_board",
    )
    upper_lantern.visual(
        Box((0.022, 0.014, 0.112)),
        origin=Origin(xyz=(0.0, -0.004, 0.078)),
        material=white,
        name="inner_reflector",
    )
    for row in range(5):
        z = 0.035 + row * 0.020
        for col, x in enumerate((-0.009, 0.009)):
            upper_lantern.visual(
                Cylinder(radius=0.0043, length=0.0025),
                origin=Origin(
                    xyz=(x, -0.0184, z),
                    rpy=(math.pi / 2, 0.0, 0.0),
                ),
                material=led,
                name=f"led_lens_{row}_{col}",
            )
            upper_lantern.visual(
                Sphere(radius=0.0021),
                origin=Origin(xyz=(x, -0.0202, z)),
                material=screw,
                name=f"led_core_{row}_{col}",
            )

    upper_lantern.visual(
        _ring_profile_mesh(
            "stepped_top_cap_shell",
            [
                (0.052, 0.132),
                (0.061, 0.138),
                (0.061, 0.154),
                (0.057, 0.160),
                (0.048, 0.165),
            ],
            [(0.020, 0.132), (0.020, 0.162), (0.000, 0.165)],
            segments=96,
        ),
        material=black,
        name="top_cap",
    )
    upper_lantern.visual(
        Cylinder(radius=0.038, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.1665)),
        material=dark,
        name="top_disc",
    )
    # Small raised tabs around the top cap.
    for i in range(6):
        angle = 2.0 * math.pi * i / 6
        r = 0.057
        upper_lantern.visual(
            Box((0.022, 0.026, 0.018)),
            origin=Origin(
                xyz=(r * math.cos(angle), r * math.sin(angle), 0.149),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark,
            name=f"cap_tab_{i}",
        )
    # Side hinge bosses for the wire bail. They sit just outside the handle ends.
    for side, x in enumerate((-0.0635, 0.0635)):
        upper_lantern.visual(
            Cylinder(radius=0.006, length=0.012),
            origin=Origin(xyz=(x, 0.0, 0.158), rpy=(0.0, math.pi / 2, 0.0)),
            material=chrome,
            name=f"handle_boss_{side}",
        )

    handle = model.part("wire_handle")
    handle.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (-0.056, 0.0, 0.000),
                    (-0.055, 0.0, 0.045),
                    (-0.044, 0.0, 0.102),
                    (-0.020, 0.0, 0.128),
                    (0.000, 0.0, 0.132),
                    (0.020, 0.0, 0.128),
                    (0.044, 0.0, 0.102),
                    (0.055, 0.0, 0.045),
                    (0.056, 0.0, 0.000),
                ],
                radius=0.0022,
                samples_per_segment=10,
                radial_segments=18,
                cap_ends=True,
            ),
            "fold_up_wire_bail",
        ),
        material=chrome,
        name="wire_bail",
    )
    handle.visual(
        Box((0.052, 0.006, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, 0.132)),
        material=chrome,
        name="top_grip_flat",
    )

    model.articulation(
        "base_to_lantern_slide",
        ArticulationType.PRISMATIC,
        parent=lower_base,
        child=upper_lantern,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.18, lower=0.0, upper=SLIDE_TRAVEL),
    )

    model.articulation(
        "lantern_to_handle_hinge",
        ArticulationType.REVOLUTE,
        parent=upper_lantern,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, 0.158)),
        axis=(1.0, 0.0, 0.0),
        # q=0 shows the carry handle upright like the reference; negative q folds it down.
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-1.45, upper=0.20),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower_base = object_model.get_part("lower_base")
    upper_lantern = object_model.get_part("upper_lantern")
    handle = object_model.get_part("wire_handle")
    slide = object_model.get_articulation("base_to_lantern_slide")
    hinge = object_model.get_articulation("lantern_to_handle_hinge")

    ctx.check(
        "image category matches lantern",
        "lantern" in object_model.name,
        details="Reference and category both appear to describe a compact camping lantern.",
    )
    ctx.expect_within(
        upper_lantern,
        lower_base,
        axes="xy",
        inner_elem="skirt_sleeve",
        outer_elem="base_shell",
        margin=0.002,
        name="telescoping skirt fits inside base sleeve",
    )
    ctx.expect_overlap(
        upper_lantern,
        lower_base,
        axes="z",
        elem_a="skirt_sleeve",
        elem_b="base_shell",
        min_overlap=0.055,
        name="collapsed slide remains retained",
    )
    ctx.expect_gap(
        upper_lantern,
        lower_base,
        axis="z",
        positive_elem="lower_collar",
        negative_elem="base_shell",
        max_gap=0.0015,
        max_penetration=0.001,
        name="lower collar seats on base rim",
    )
    ctx.expect_overlap(
        upper_lantern,
        lower_base,
        axes="xy",
        elem_a="clear_chamber",
        elem_b="base_shell",
        min_overlap=0.075,
        name="clear chamber is coaxial with cylindrical base",
    )

    rest_pos = ctx.part_world_position(upper_lantern)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        extended_pos = ctx.part_world_position(upper_lantern)
        ctx.expect_within(
            upper_lantern,
            lower_base,
            axes="xy",
            inner_elem="skirt_sleeve",
            outer_elem="base_shell",
            margin=0.002,
            name="extended slide stays coaxial",
        )
        ctx.expect_overlap(
            upper_lantern,
            lower_base,
            axes="z",
            elem_a="skirt_sleeve",
            elem_b="base_shell",
            min_overlap=0.020,
            name="extended slide retains insertion",
        )
    ctx.check(
        "lantern body slides upward",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.060,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    upright_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({hinge: -1.20}):
        folded_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "wire handle folds down on hinge",
        upright_aabb is not None
        and folded_aabb is not None
        and folded_aabb[1][2] < upright_aabb[1][2] - 0.035,
        details=f"upright={upright_aabb}, folded={folded_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
