from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _cylinder_origin_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    """Return an Origin/radius-length pair for a URDF cylinder between points."""

    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    mid = ((sx + ex) / 2.0, (sy + ey) / 2.0, (sz + ez) / 2.0)
    return Origin(xyz=mid, rpy=(0.0, pitch, yaw)), length


def _ergonomic_handle_mesh() -> object:
    """Rounded T grip with a short socket neck and shallow finger scallops."""

    grip = cq.Workplane("XY").box(0.165, 0.045, 0.034).edges().fillet(0.010)
    neck = (
        cq.Workplane("XY")
        .box(0.038, 0.036, 0.052)
        .edges("|Z")
        .fillet(0.009)
        .translate((0.0, 0.0, -0.036))
    )
    handle = grip.union(neck)

    # The underside of a molded cane grip is often lightly scalloped for fingers.
    for x in (-0.048, -0.016, 0.016, 0.048):
        cutter = (
            cq.Workplane("XZ")
            .center(x, -0.021)
            .circle(0.010)
            .extrude(0.070, both=True)
        )
        handle = handle.cut(cutter)

    return handle


def _hollow_tube_mesh(outer_radius: float, inner_radius: float, length: float) -> object:
    """A centered open-ended tube, used where the telescoping shaft is visible."""

    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="bronze_cane",
        meta={
            "description": (
                "A 90 cm walking cane with a slim bronze/copper shaft, black "
                "ergonomic T handle, and a compact stabilizing quad base."
            )
        },
    )

    copper = model.material("brushed_copper", rgba=(0.74, 0.34, 0.19, 1.0))
    dark_copper = model.material("shadowed_copper", rgba=(0.36, 0.15, 0.09, 1.0))
    black_plastic = model.material("black_molded_plastic", rgba=(0.005, 0.005, 0.006, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.0, 0.0, 0.0, 1.0))
    bright_metal = model.material("polished_collar_metal", rgba=(0.78, 0.72, 0.62, 1.0))

    lower = model.part("lower_assembly")
    upper = model.part("upper_assembly")

    joint_z = 0.330

    # The lower copper tube is slightly slimmer and continues up inside the
    # upper tube.  At q=0 it is visibly retained for about 15 cm, like a real
    # push-button telescoping cane.
    lower.visual(
        Cylinder(radius=0.0068, length=0.392),
        origin=Origin(xyz=(0.0, 0.0, 0.254)),
        material=copper,
        name="lower_tube",
    )
    lower.visual(
        Box((0.0014, 0.0010, 0.360)),
        origin=Origin(xyz=(0.0, -0.0070, 0.250)),
        material=dark_copper,
        name="lower_shadow_line",
    )

    # The upper member is an open-ended tube, not a solid rod, so the smaller
    # lower tube can nest inside it without a false collision.
    upper.visual(
        mesh_from_cadquery(
            _hollow_tube_mesh(outer_radius=0.0090, inner_radius=0.00675, length=0.555),
            "upper_tube",
            tolerance=0.0007,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.5775 - joint_z)),
        material=copper,
        name="upper_tube",
    )

    # Subtle darker longitudinal inlay makes the upper shaft read as brushed metal.
    upper.visual(
        Box((0.0018, 0.0010, 0.520)),
        origin=Origin(xyz=(0.0, -0.0092, 0.590 - joint_z)),
        material=dark_copper,
        name="upper_shadow_line",
    )

    # Small metal collars at the top and bottom where the tubes are captured.
    upper.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.855 - joint_z)),
        material=bright_metal,
        name="top_collar",
    )
    lower.visual(
        Cylinder(radius=0.013, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=dark_copper,
        name="base_socket",
    )

    # Ergonomic black T-shaped handle.  The neck overlaps the collar/shaft locally,
    # as a real molded handle socket would slide over the tube.
    upper.visual(
        mesh_from_cadquery(_ergonomic_handle_mesh(), "ergonomic_t_handle", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, 0.878 - joint_z)),
        material=black_plastic,
        name="handle_grip",
    )

    # Metal hub for the stabilizing foot bracket.
    lower.visual(
        Cylinder(radius=0.020, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=dark_copper,
        name="base_hub",
    )

    # Compact quad stabilizing base: four short metal feet with separate rubber tips.
    # Tip centers sit on a 12 cm footprint, with all repeated feet emitted by a loop.
    foot_radius = 0.006
    tip_radius = 0.012
    tip_height = 0.020
    foot_directions = ((1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0))
    for index, (ux, uy) in enumerate(foot_directions):
        foot_start = (0.010 * ux, 0.010 * uy, 0.040)
        foot_end = (0.055 * ux, 0.055 * uy, 0.020)
        foot_origin, foot_length = _cylinder_origin_between(foot_start, foot_end)
        lower.visual(
            Cylinder(radius=foot_radius, length=foot_length),
            origin=foot_origin,
            material=dark_copper,
            name=f"foot_{index}",
        )

        # Rubber tips are short ferrule pads, wider than the metal foot rods.
        lower.visual(
            Cylinder(radius=tip_radius, length=tip_height),
            origin=Origin(xyz=(0.055 * ux, 0.055 * uy, tip_height / 2.0)),
            material=rubber,
            name=f"rubber_tip_{index}",
        )

    # A molded black adjustment collar clamps around the telescoping overlap.
    upper.visual(
        mesh_from_cadquery(
            _hollow_tube_mesh(outer_radius=0.0128, inner_radius=0.0086, length=0.034),
            "adjustment_collar",
            tolerance=0.0007,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.330 - joint_z)),
        material=black_plastic,
        name="adjustment_collar",
    )
    upper.visual(
        Sphere(radius=0.004),
        origin=Origin(xyz=(0.0112, 0.0, 0.388 - joint_z)),
        material=black_plastic,
        name="spring_button",
    )

    model.articulation(
        "lower_to_upper",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, joint_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.18, lower=0.0, upper=0.12),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_assembly")
    upper = object_model.get_part("upper_assembly")
    slide = object_model.get_articulation("lower_to_upper")

    def combined_height() -> float | None:
        lower_aabb = ctx.part_world_aabb(lower)
        upper_aabb = ctx.part_world_aabb(upper)
        if lower_aabb is None or upper_aabb is None:
            return None
        min_z = min(lower_aabb[0][2], upper_aabb[0][2])
        max_z = max(lower_aabb[1][2], upper_aabb[1][2])
        return max_z - min_z

    height = combined_height()
    if height is None:
        ctx.fail("cane has measurable geometry", "No world AABB was available.")
    else:
        ctx.check(
            "overall height near walking cane scale",
            0.88 <= height <= 0.93,
            details=f"height={height:.3f} m",
        )

    tip_bounds = [
        ctx.part_element_world_aabb(lower, elem=f"rubber_tip_{index}")
        for index in range(4)
    ]
    if all(bounds is not None for bounds in tip_bounds):
        min_x = min(bounds[0][0] for bounds in tip_bounds if bounds is not None)
        max_x = max(bounds[1][0] for bounds in tip_bounds if bounds is not None)
        min_y = min(bounds[0][1] for bounds in tip_bounds if bounds is not None)
        max_y = max(bounds[1][1] for bounds in tip_bounds if bounds is not None)
        width_x = max_x - min_x
        width_y = max_y - min_y
        ctx.check(
            "compact stabilizing base footprint",
            0.115 <= max(width_x, width_y) <= 0.145,
            details=f"rubber-tip footprint=({width_x:.3f}, {width_y:.3f}) m",
        )
    else:
        ctx.fail("base rubber tips are measurable", f"tip_bounds={tip_bounds}")

    # Prompt-specific relationship checks on named elements.
    ctx.allow_overlap(
        lower,
        upper,
        elem_a="lower_tube",
        elem_b="upper_tube",
        reason=(
            "The lower cane tube is intentionally represented as a close sliding "
            "bearing fit inside the upper telescoping sleeve."
        ),
    )
    ctx.expect_overlap(
        upper,
        upper,
        axes="xy",
        elem_a="upper_tube",
        elem_b="top_collar",
        min_overlap=0.010,
        name="top collar captures upper tube",
    )
    ctx.expect_overlap(
        lower,
        lower,
        axes="xy",
        elem_a="lower_tube",
        elem_b="base_socket",
        min_overlap=0.010,
        name="base socket captures lower tube",
    )
    ctx.expect_within(
        lower,
        upper,
        axes="xy",
        inner_elem="lower_tube",
        outer_elem="upper_tube",
        margin=0.001,
        name="lower tube nests inside upper tube",
    )
    ctx.expect_overlap(
        lower,
        upper,
        axes="z",
        elem_a="lower_tube",
        elem_b="upper_tube",
        min_overlap=0.12,
        name="rest pose has telescoping insertion",
    )

    handle_aabb = ctx.part_element_world_aabb(upper, elem="handle_grip")
    if handle_aabb is not None:
        handle_lower, handle_upper = handle_aabb
        handle_span = handle_upper[0] - handle_lower[0]
        ctx.check(
            "handle is a broad T grip",
            handle_span >= 0.150,
            details=f"handle_x_span={handle_span:.3f} m",
        )

    rest_height = combined_height()
    rest_handle_aabb = ctx.part_element_world_aabb(upper, elem="handle_grip")
    with ctx.pose({slide: 0.12}):
        extended_height = combined_height()
        extended_handle_aabb = ctx.part_element_world_aabb(upper, elem="handle_grip")
        ctx.expect_within(
            lower,
            upper,
            axes="xy",
            inner_elem="lower_tube",
            outer_elem="upper_tube",
            margin=0.001,
            name="extended lower tube stays centered in upper tube",
        )
        ctx.expect_overlap(
            lower,
            upper,
            axes="z",
            elem_a="lower_tube",
            elem_b="upper_tube",
            min_overlap=0.025,
            name="extended pose retains tube insertion",
        )

    if (
        rest_height is not None
        and extended_height is not None
        and rest_handle_aabb is not None
        and extended_handle_aabb is not None
    ):
        rest_handle_top = rest_handle_aabb[1][2]
        extended_handle_top = extended_handle_aabb[1][2]
        ctx.check(
            "telescoping joint increases cane height",
            extended_height > rest_height + 0.10
            and extended_handle_top > rest_handle_top + 0.10,
            details=(
                f"rest_height={rest_height:.3f}, extended_height={extended_height:.3f}, "
                f"rest_handle_top={rest_handle_top:.3f}, extended_handle_top={extended_handle_top:.3f}"
            ),
        )
    else:
        ctx.fail(
            "telescoping joint increases cane height",
            "Could not measure rest/extended height or handle position.",
        )

    return ctx.report()


object_model = build_object_model()
