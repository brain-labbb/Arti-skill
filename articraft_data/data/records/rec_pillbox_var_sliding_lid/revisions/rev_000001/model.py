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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_W = 0.130
BASE_D = 0.100
BASE_BOTTOM_H = 0.004
WALL_H = 0.010
WALL_TOP_Z = BASE_BOTTOM_H + WALL_H
COVER_T = 0.003
RAIL_H = 0.002
COVER_BOTTOM_Z = WALL_TOP_Z + RAIL_H
COVER_W = BASE_W - 0.016
COVER_D = BASE_D - 0.014


def rounded_box_mesh(width: float, depth: float, height: float, radius: float, name: str):
    """Return a mesh-backed rounded rectangular solid centered on its local origin."""
    solid = cq.Workplane("XY").box(width, depth, height)
    if radius > 0:
        solid = solid.edges("|Z").fillet(min(radius, width * 0.45, depth * 0.45, height * 0.45))
    return mesh_from_cadquery(solid, name, tolerance=0.00035, angular_tolerance=0.12)


def add_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material,
    name: str,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def add_cylinder_x(part, radius: float, length: float, xyz, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def add_cover_digit(part, digit: int, cx: float, cy: float, material) -> None:
    """Raised white seven-segment number decal bonded into the cover top."""
    patterns = {
        1: "bc",
        2: "abged",
        3: "abgcd",
        4: "fgbc",
        5: "afgcd",
        6: "afgecd",
        7: "abc",
    }
    dw = 0.0055
    dh = 0.0080
    s = 0.00075
    z = COVER_T + 0.00012
    segs = {
        "a": ((dw, s, 0.00035), (cx, cy + dh * 0.50, z)),
        "b": ((s, dh * 0.50, 0.00035), (cx + dw * 0.50, cy + dh * 0.25, z)),
        "c": ((s, dh * 0.50, 0.00035), (cx + dw * 0.50, cy - dh * 0.25, z)),
        "d": ((dw, s, 0.00035), (cx, cy - dh * 0.50, z)),
        "e": ((s, dh * 0.50, 0.00035), (cx - dw * 0.50, cy - dh * 0.25, z)),
        "f": ((s, dh * 0.50, 0.00035), (cx - dw * 0.50, cy + dh * 0.25, z)),
        "g": ((dw, s, 0.00035), (cx, cy, z)),
    }
    for seg in patterns[digit]:
        size, xyz = segs[seg]
        add_box(part, size, xyz, material, f"cover_digit_{digit}_{seg}")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_7_day_pill_organizer")

    cream = model.material("warm_white_plastic", rgba=(0.94, 0.88, 0.68, 1.0))
    tray_blue = model.material("soft_blue_tray", rgba=(0.58, 0.74, 0.88, 1.0))
    white_print = model.material("white_print", rgba=(1.0, 1.0, 0.96, 1.0))
    cover_mat = model.material("frosted_translucent_cover", rgba=(0.82, 0.87, 0.91, 0.42))
    latch_mat = model.material("white_latch_plastic", rgba=(0.96, 0.96, 0.90, 1.0))
    grip_mat = model.material("grip_accent", rgba=(0.55, 0.70, 0.82, 0.80))

    # ── Base tray ──────────────────────────────────────────────────────
    base = model.part("base_tray")
    base.visual(
        rounded_box_mesh(BASE_W, BASE_D, BASE_BOTTOM_H, 0.009, "rounded_base_bottom"),
        origin=Origin(xyz=(0.0, 0.0, BASE_BOTTOM_H / 2.0)),
        material=cream,
        name="rounded_base_bottom",
    )

    # Raised outer tray rim
    rim_t = 0.005
    add_box(base, (BASE_W, rim_t, WALL_H), (0.0, BASE_D / 2.0 - rim_t / 2.0, BASE_BOTTOM_H + WALL_H / 2.0), cream, "rear_rim")
    add_box(base, (BASE_W, rim_t, WALL_H), (0.0, -BASE_D / 2.0 + rim_t / 2.0, BASE_BOTTOM_H + WALL_H / 2.0), cream, "front_rim")
    add_box(base, (rim_t, BASE_D - 2.0 * rim_t, WALL_H), (-BASE_W / 2.0 + rim_t / 2.0, 0.0, BASE_BOTTOM_H + WALL_H / 2.0), cream, "side_rim_0")
    add_box(base, (rim_t, BASE_D - 2.0 * rim_t, WALL_H), (BASE_W / 2.0 - rim_t / 2.0, 0.0, BASE_BOTTOM_H + WALL_H / 2.0), cream, "side_rim_1")

    # Seven compartment wells: 3 wider rear, 4 smaller front
    well_specs = [
        {"day": 1, "x": -0.039, "y": 0.018, "w": 0.034, "d": 0.035},
        {"day": 2, "x": 0.000, "y": 0.018, "w": 0.034, "d": 0.035},
        {"day": 3, "x": 0.039, "y": 0.018, "w": 0.034, "d": 0.035},
        {"day": 4, "x": -0.0435, "y": -0.024, "w": 0.026, "d": 0.035},
        {"day": 5, "x": -0.0145, "y": -0.024, "w": 0.026, "d": 0.035},
        {"day": 6, "x": 0.0145, "y": -0.024, "w": 0.026, "d": 0.035},
        {"day": 7, "x": 0.0435, "y": -0.024, "w": 0.026, "d": 0.035},
    ]

    wall_t = 0.0022
    floor_h = 0.0008
    for spec in well_specs:
        n = spec["day"]
        x = spec["x"]
        y = spec["y"]
        w = spec["w"]
        d = spec["d"]
        add_box(base, (w - 2.0 * wall_t, d - 2.0 * wall_t, floor_h), (x, y, BASE_BOTTOM_H + floor_h / 2.0), tray_blue, f"well_{n}_floor")
        add_box(base, (w, wall_t, WALL_H), (x, y + d / 2.0 - wall_t / 2.0, BASE_BOTTOM_H + WALL_H / 2.0), cream, f"well_{n}_rear_wall")
        add_box(base, (w, wall_t, WALL_H), (x, y - d / 2.0 + wall_t / 2.0, BASE_BOTTOM_H + WALL_H / 2.0), cream, f"well_{n}_front_wall")
        add_box(base, (wall_t, d, WALL_H), (x - w / 2.0 + wall_t / 2.0, y, BASE_BOTTOM_H + WALL_H / 2.0), cream, f"well_{n}_side_wall_0")
        add_box(base, (wall_t, d, WALL_H), (x + w / 2.0 - wall_t / 2.0, y, BASE_BOTTOM_H + WALL_H / 2.0), cream, f"well_{n}_side_wall_1")

    # Sliding rails: two raised strips on top of side rims guide the cover
    rail_w = 0.004
    rail_d = BASE_D - 0.016
    add_box(base, (rail_w, rail_d, RAIL_H), (-BASE_W / 2.0 + 0.008, 0.0, WALL_TOP_Z + RAIL_H / 2.0), cream, "slide_rail_0")
    add_box(base, (rail_w, rail_d, RAIL_H), (BASE_W / 2.0 - 0.008, 0.0, WALL_TOP_Z + RAIL_H / 2.0), cream, "slide_rail_1")

    # Front cover stop prevents the sliding cover from coming off the tray
    # Aligned with the front rim center in Y so it contacts the rim.
    add_box(base, (BASE_W - 0.024, rim_t, 0.005), (0.0, -BASE_D / 2.0 + rim_t / 2.0, WALL_TOP_Z + 0.0025), cream, "front_cover_stop")
    # Rear cover stop at the back edge, aligned with the rear rim.
    add_box(base, (BASE_W - 0.024, rim_t, 0.005), (0.0, BASE_D / 2.0 - rim_t / 2.0, WALL_TOP_Z + 0.0025), cream, "rear_cover_stop")

    # ── Sliding cover ──────────────────────────────────────────────────
    sliding_cover = model.part("sliding_cover")
    sliding_cover.visual(
        rounded_box_mesh(COVER_W, COVER_D, COVER_T, 0.008, "cover_panel"),
        origin=Origin(xyz=(0.0, 0.0, COVER_T / 2.0)),
        material=cover_mat,
        name="cover_panel",
    )

    # Two shallow grooves on the cover underside to ride on the rails
    add_box(sliding_cover, (rail_w + 0.001, COVER_D - 0.004, 0.001),
            (-COVER_W / 2.0 + 0.001, 0.0, -0.0005), cover_mat, "cover_groove_0")
    add_box(sliding_cover, (rail_w + 0.001, COVER_D - 0.004, 0.001),
            (COVER_W / 2.0 - 0.001, 0.0, -0.0005), cover_mat, "cover_groove_1")

    # Grip tab on the -X end for sliding
    add_box(sliding_cover, (0.014, 0.030, COVER_T + 0.003),
            (-COVER_W / 2.0 - 0.004, 0.0, COVER_T / 2.0), grip_mat, "grip_tab")
    # Grip ridges
    for i in range(3):
        add_box(sliding_cover, (0.010, 0.001, 0.001),
                (-COVER_W / 2.0 - 0.004, -0.008 + i * 0.008, COVER_T + 0.0016),
                grip_mat, f"grip_ridge_{i}")

    # Day number indicators on the cover top surface (visible through translucent cover)
    digit_y = 0.030
    digit_xs = [-0.044, -0.028, -0.012, 0.004, 0.020, 0.036, 0.048]
    for i, n in enumerate(range(1, 8)):
        add_cover_digit(sliding_cover, n, digit_xs[i], digit_y, white_print)

    # Prismatic joint: slides along +X to expose compartments progressively
    slide_joint = model.articulation(
        "base_to_sliding_cover",
        ArticulationType.PRISMATIC,
        parent=base,
        child=sliding_cover,
        origin=Origin(xyz=(0.0, 0.0, COVER_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.30, lower=0.0, upper=0.080),
    )

    # ── Front latch (re-parented from outer_lid to base_tray) ──────────
    latch = model.part("front_latch")
    add_box(latch, (0.030, 0.0045, 0.010), (0.0, -0.0024, -0.0050), latch_mat, "latch_tab")
    add_box(latch, (0.025, 0.0010, 0.0010), (0.0, -0.0050, -0.0018), cream, "grip_ridge_0")
    add_box(latch, (0.025, 0.0010, 0.0010), (0.0, -0.0050, -0.0040), cream, "grip_ridge_1")
    add_box(latch, (0.025, 0.0010, 0.0010), (0.0, -0.0050, -0.0062), cream, "grip_ridge_2")

    model.articulation(
        "base_to_front_latch",
        ArticulationType.REVOLUTE,
        parent=base,
        child=latch,
        origin=Origin(xyz=(0.0, -BASE_D / 2.0 + 0.003, WALL_TOP_Z + 0.004)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=3.0, lower=0.0, upper=0.85),
    )

    model.meta["slide_joint"] = slide_joint.name
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_tray")
    sliding_cover = object_model.get_part("sliding_cover")
    latch = object_model.get_part("front_latch")
    slide_joint = object_model.get_articulation("base_to_sliding_cover")
    latch_joint = object_model.get_articulation("base_to_front_latch")

    # Core variant assertions: sliding_cover and its prismatic joint exist
    ctx.check("sliding_cover part exists", sliding_cover is not None)
    ctx.check("base_to_sliding_cover joint exists", slide_joint is not None)

    # At q=0 the cover should overlap the base tray wells in XY
    ctx.expect_overlap(
        sliding_cover,
        base,
        axes="xy",
        min_overlap=0.028,
        elem_a="cover_panel",
        elem_b="well_1_floor",
        name="cover_overlaps_well_1_at_rest",
    )
    ctx.expect_overlap(
        sliding_cover,
        base,
        axes="xy",
        min_overlap=0.020,
        elem_a="cover_panel",
        elem_b="well_7_floor",
        name="cover_overlaps_well_7_at_rest",
    )

    # Cover should sit above the wall tops (cover sits on rails above the rim)
    ctx.expect_gap(
        sliding_cover,
        base,
        axis="z",
        min_gap=-0.002,
        max_gap=0.016,
        positive_elem="cover_panel",
        negative_elem="well_1_floor",
        name="cover_sits_above_well_floors",
    )

    # Positive q should slide cover in +X direction
    rest_pos = ctx.part_world_position(sliding_cover)
    with ctx.pose({slide_joint: 0.060}):
        slid_pos = ctx.part_world_position(sliding_cover)
    ctx.check(
        "positive_q_slides_cover_in_plus_x",
        rest_pos is not None
        and slid_pos is not None
        and slid_pos[0] > rest_pos[0] + 0.040,
        details=f"rest_x={rest_pos}, slid_x={slid_pos}",
    )

    # At max slide, the cover should expose the leftmost compartments
    with ctx.pose({slide_joint: 0.060}):
        # Cover center shifts right; well_1 on left should no longer overlap cover
        cover_aabb = ctx.part_element_world_aabb(sliding_cover, elem="cover_panel")
        well1_aabb = ctx.part_element_world_aabb(base, elem="well_1_floor")
    ctx.check(
        "slid_cover_exposes_left_wells",
        cover_aabb is not None
        and well1_aabb is not None
        and cover_aabb[0][0] > well1_aabb[1][0] - 0.005,
        details=f"cover_min_x={cover_aabb[0][0] if cover_aabb else None}, well1_max_x={well1_aabb[1][0] if well1_aabb else None}",
    )

    # Latch should flip outward when articulated
    latched_aabb = ctx.part_world_aabb(latch)
    with ctx.pose({latch_joint: 0.65}):
        released_aabb = ctx.part_world_aabb(latch)
    ctx.check(
        "front_latch_flips_out",
        latched_aabb is not None
        and released_aabb is not None
        and released_aabb[0][1] < latched_aabb[0][1] - 0.002,
        details=f"latched={latched_aabb}, released={released_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
