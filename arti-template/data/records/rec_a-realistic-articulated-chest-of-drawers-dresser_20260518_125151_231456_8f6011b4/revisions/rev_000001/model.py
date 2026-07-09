from __future__ import annotations

from math import pi

import cadquery as cq

# >>> USER_CODE_START
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


# Six-drawer dresser (chest of drawers): wooden carcass, two columns by three
# rows of drawers, each drawer running on a pair of ball-bearing prismatic
# glides, plus a pair of metal bar pull handles per drawer, a decorative top
# panel with rounded edges, and four short tapered feet.
#
# Coordinates: +Y is the drawer-pull direction; +Z is up; the world origin sits
# on the floor at the carcass plan centroid.

# --- Carcass shell -----------------------------------------------------------
CARCASS_WIDTH = 1.10
CARCASS_DEPTH = 0.48
CARCASS_HEIGHT = 0.90           # carcass shell height above the feet
FOOT_HEIGHT = 0.10

SIDE_THICKNESS = 0.020
BOTTOM_THICKNESS = 0.022
INNER_TOP_THICKNESS = 0.022
BACK_THICKNESS = 0.010
DIVIDER_THICKNESS = 0.020

# --- Decorative top panel (CadQuery mesh, filleted edges) --------------------
TOP_PANEL_THICKNESS = 0.028
TOP_PANEL_OVERHANG_X = 0.012
TOP_PANEL_OVERHANG_Y = 0.014
TOP_PANEL_SIDE_FILLET = 0.008
TOP_PANEL_TOP_FILLET = 0.006

# --- Tapered feet (CadQuery loft) --------------------------------------------
FOOT_TOP_SIZE = 0.060
FOOT_BOTTOM_SIZE = 0.034
FOOT_INSET = 0.025

# --- Drawer grid -------------------------------------------------------------
COLUMN_COUNT = 2
ROW_COUNT = 3
DRAWER_COUNT = COLUMN_COUNT * ROW_COUNT

INNER_WIDTH = CARCASS_WIDTH - 2.0 * SIDE_THICKNESS
COLUMN_INNER_WIDTH = (INNER_WIDTH - DIVIDER_THICKNESS) / COLUMN_COUNT
INNER_HEIGHT = CARCASS_HEIGHT - BOTTOM_THICKNESS - INNER_TOP_THICKNESS

DRAWER_REVEAL_VERT = 0.010
DRAWER_REVEAL_HORIZ = 0.004
DRAWER_FRONT_HEIGHT = (INNER_HEIGHT - (ROW_COUNT + 1) * DRAWER_REVEAL_VERT) / ROW_COUNT
DRAWER_FRONT_WIDTH = COLUMN_INNER_WIDTH - 2.0 * DRAWER_REVEAL_HORIZ
DRAWER_FRONT_THICKNESS = 0.022

# --- Drawer box --------------------------------------------------------------
DRAWER_BOX_DEPTH = 0.36
DRAWER_SIDE_THICKNESS = 0.014
DRAWER_BACK_THICKNESS = 0.012
DRAWER_BOTTOM_THICKNESS = 0.012
DRAWER_BOX_HEIGHT = DRAWER_FRONT_HEIGHT - 0.040
DRAWER_SIDE_HEIGHT = DRAWER_BOX_HEIGHT - DRAWER_BOTTOM_THICKNESS

# --- Ball-bearing slide rails -----------------------------------------------
GUIDE_THICKNESS = 0.012
GUIDE_LENGTH = 0.36
GUIDE_HEIGHT = 0.018
GUIDE_Y_OFFSET = -0.060          # carcass-frame Y center

RUNNER_THICKNESS = 0.018
RUNNER_LENGTH = 0.30
RUNNER_HEIGHT = 0.014
RUNNER_Y_OFFSET = -0.040         # drawer-frame Y center
RUNNER_Z_OFFSET = (
    -DRAWER_BOX_HEIGHT * 0.5 + DRAWER_BOTTOM_THICKNESS + 0.040 + RUNNER_HEIGHT * 0.5
)
# X offset of the runner center relative to the drawer center.
RUNNER_X_OFFSET_DRAWER = 0.0  # filled in after DRAWER_BOX_WIDTH is computed

# Guides and runners are positioned so the drawer-side runner outer face is
# flush against the carcass-side guide inner face. We derive the drawer box
# width from that constraint so it always lines up.
GUIDE_X_FROM_COLUMN_EDGE = GUIDE_THICKNESS * 0.5
RUNNER_X_FROM_COLUMN_EDGE = GUIDE_THICKNESS + RUNNER_THICKNESS * 0.5
DRAWER_BOX_WIDTH = COLUMN_INNER_WIDTH - 2.0 * (GUIDE_THICKNESS + RUNNER_THICKNESS)

# Drawer motion.
DRAWER_TRAVEL = 0.22
DRAWER_FRONT_PROUD = 0.002
DRAWER_JOINT_Y = (
    CARCASS_DEPTH * 0.5
    + DRAWER_FRONT_PROUD
    - DRAWER_BOX_DEPTH * 0.5
    - DRAWER_FRONT_THICKNESS
)

# --- Bar pull handles: pair per drawer --------------------------------------
HANDLE_BAR_LENGTH = 0.090
HANDLE_BAR_RADIUS = 0.0055
HANDLE_STEM_LENGTH = 0.030
HANDLE_STEM_RADIUS = 0.004
HANDLE_X_SPACING = 0.130          # horizontal spacing between the two bar centers
HANDLE_POST_SPACING = 0.062       # post-to-post spacing within one bar handle


def _column_center_x(col: int) -> float:
    inner_left = -CARCASS_WIDTH * 0.5 + SIDE_THICKNESS
    inner_right = -DIVIDER_THICKNESS * 0.5
    left_center = (inner_left + inner_right) * 0.5
    return left_center if col == 0 else -left_center


def _row_center_z(row: int) -> float:
    base = (
        FOOT_HEIGHT
        + BOTTOM_THICKNESS
        + DRAWER_REVEAL_VERT
        + DRAWER_FRONT_HEIGHT * 0.5
    )
    return base + row * (DRAWER_FRONT_HEIGHT + DRAWER_REVEAL_VERT)


def _drawer_name(row: int, col: int) -> str:
    return f"drawer_{row}_{col}"


def _make_top_panel_shape() -> object:
    width = CARCASS_WIDTH + 2.0 * TOP_PANEL_OVERHANG_X
    depth = CARCASS_DEPTH + 2.0 * TOP_PANEL_OVERHANG_Y
    shape = (
        cq.Workplane("XY")
        .box(width, depth, TOP_PANEL_THICKNESS)
        .edges("|Z")
        .fillet(TOP_PANEL_SIDE_FILLET)
        .edges(">Z")
        .fillet(TOP_PANEL_TOP_FILLET)
    )
    return shape


def _make_tapered_foot_shape() -> object:
    shape = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .rect(FOOT_TOP_SIZE, FOOT_TOP_SIZE)
        .workplane(offset=-FOOT_HEIGHT)
        .rect(FOOT_BOTTOM_SIZE, FOOT_BOTTOM_SIZE)
        .loft(combine=True)
    )
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="six_drawer_dresser")

    carcass_wood = model.material("carcass_wood", rgba=(0.34, 0.22, 0.13, 1.0))
    drawer_wood = model.material("drawer_wood", rgba=(0.51, 0.34, 0.20, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.66, 0.68, 0.71, 1.0))
    rail_metal = model.material("rail_metal", rgba=(0.55, 0.57, 0.60, 1.0))
    foot_wood = model.material("foot_wood", rgba=(0.20, 0.13, 0.07, 1.0))

    carcass = model.part("carcass")

    side_z_center = FOOT_HEIGHT + CARCASS_HEIGHT * 0.5
    carcass.visual(
        Box((SIDE_THICKNESS, CARCASS_DEPTH, CARCASS_HEIGHT)),
        origin=Origin(
            xyz=(-CARCASS_WIDTH * 0.5 + SIDE_THICKNESS * 0.5, 0.0, side_z_center)
        ),
        material=carcass_wood,
        name="left_side",
    )
    carcass.visual(
        Box((SIDE_THICKNESS, CARCASS_DEPTH, CARCASS_HEIGHT)),
        origin=Origin(
            xyz=(CARCASS_WIDTH * 0.5 - SIDE_THICKNESS * 0.5, 0.0, side_z_center)
        ),
        material=carcass_wood,
        name="right_side",
    )

    inner_between_sides = CARCASS_WIDTH - 2.0 * SIDE_THICKNESS
    carcass.visual(
        Box((inner_between_sides, CARCASS_DEPTH, BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, FOOT_HEIGHT + BOTTOM_THICKNESS * 0.5)),
        material=carcass_wood,
        name="bottom_panel",
    )
    carcass.visual(
        Box((inner_between_sides, CARCASS_DEPTH, INNER_TOP_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                FOOT_HEIGHT + CARCASS_HEIGHT - INNER_TOP_THICKNESS * 0.5,
            )
        ),
        material=carcass_wood,
        name="inner_top_panel",
    )

    back_z_min = FOOT_HEIGHT + BOTTOM_THICKNESS
    back_z_max = FOOT_HEIGHT + CARCASS_HEIGHT - INNER_TOP_THICKNESS
    back_height = back_z_max - back_z_min
    carcass.visual(
        Box((inner_between_sides, BACK_THICKNESS, back_height)),
        origin=Origin(
            xyz=(
                0.0,
                -CARCASS_DEPTH * 0.5 + BACK_THICKNESS * 0.5,
                (back_z_min + back_z_max) * 0.5,
            )
        ),
        material=carcass_wood,
        name="back_panel",
    )

    divider_height = back_height
    divider_y_extent = CARCASS_DEPTH - BACK_THICKNESS
    carcass.visual(
        Box((DIVIDER_THICKNESS, divider_y_extent, divider_height)),
        origin=Origin(
            xyz=(
                0.0,
                BACK_THICKNESS * 0.5,
                (back_z_min + back_z_max) * 0.5,
            )
        ),
        material=carcass_wood,
        name="center_divider",
    )

    top_panel_z = FOOT_HEIGHT + CARCASS_HEIGHT + TOP_PANEL_THICKNESS * 0.5
    carcass.visual(
        mesh_from_cadquery(_make_top_panel_shape(), "top_panel"),
        origin=Origin(xyz=(0.0, 0.0, top_panel_z)),
        material=carcass_wood,
        name="top_panel",
    )

    foot_x = CARCASS_WIDTH * 0.5 - FOOT_INSET - FOOT_TOP_SIZE * 0.5
    foot_y = CARCASS_DEPTH * 0.5 - FOOT_INSET - FOOT_TOP_SIZE * 0.5
    foot_positions = (
        (-foot_x, -foot_y, "foot_rear_left"),
        (foot_x, -foot_y, "foot_rear_right"),
        (-foot_x, foot_y, "foot_front_left"),
        (foot_x, foot_y, "foot_front_right"),
    )
    for fx, fy, fname in foot_positions:
        carcass.visual(
            mesh_from_cadquery(_make_tapered_foot_shape(), fname),
            origin=Origin(xyz=(fx, fy, FOOT_HEIGHT)),
            material=foot_wood,
            name=fname,
        )

    # Carcass-side glide guides for each drawer cell (outer = against side
    # panel, inner = against the center divider).
    for row in range(ROW_COUNT):
        z_center = _row_center_z(row)
        guide_z = z_center + RUNNER_Z_OFFSET
        for col in range(COLUMN_COUNT):
            cx = _column_center_x(col)
            outer_sign = -1 if col == 0 else 1
            inner_sign = -outer_sign
            outer_x = cx + outer_sign * (COLUMN_INNER_WIDTH * 0.5 - GUIDE_THICKNESS * 0.5)
            inner_x = cx + inner_sign * (COLUMN_INNER_WIDTH * 0.5 - GUIDE_THICKNESS * 0.5)
            carcass.visual(
                Box((GUIDE_THICKNESS, GUIDE_LENGTH, GUIDE_HEIGHT)),
                origin=Origin(xyz=(outer_x, GUIDE_Y_OFFSET, guide_z)),
                material=rail_metal,
                name=f"guide_{row}_{col}_outer",
            )
            carcass.visual(
                Box((GUIDE_THICKNESS, GUIDE_LENGTH, GUIDE_HEIGHT)),
                origin=Origin(xyz=(inner_x, GUIDE_Y_OFFSET, guide_z)),
                material=rail_metal,
                name=f"guide_{row}_{col}_inner",
            )

    carcass.inertial = Inertial.from_geometry(
        Box((CARCASS_WIDTH, CARCASS_DEPTH, CARCASS_HEIGHT)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, FOOT_HEIGHT + CARCASS_HEIGHT * 0.5)),
    )

    # Drawers.
    for row in range(ROW_COUNT):
        for col in range(COLUMN_COUNT):
            name = _drawer_name(row, col)
            drawer = model.part(name)

            drawer.visual(
                Box((DRAWER_BOX_WIDTH, DRAWER_BOX_DEPTH, DRAWER_BOTTOM_THICKNESS)),
                origin=Origin(
                    xyz=(
                        0.0,
                        0.0,
                        -DRAWER_BOX_HEIGHT * 0.5 + DRAWER_BOTTOM_THICKNESS * 0.5,
                    )
                ),
                material=drawer_wood,
                name="bottom",
            )
            side_z = (
                -DRAWER_BOX_HEIGHT * 0.5 + DRAWER_BOTTOM_THICKNESS + DRAWER_SIDE_HEIGHT * 0.5
            )
            drawer.visual(
                Box((DRAWER_SIDE_THICKNESS, DRAWER_BOX_DEPTH, DRAWER_SIDE_HEIGHT)),
                origin=Origin(
                    xyz=(
                        -DRAWER_BOX_WIDTH * 0.5 + DRAWER_SIDE_THICKNESS * 0.5,
                        0.0,
                        side_z,
                    )
                ),
                material=drawer_wood,
                name="left_side",
            )
            drawer.visual(
                Box((DRAWER_SIDE_THICKNESS, DRAWER_BOX_DEPTH, DRAWER_SIDE_HEIGHT)),
                origin=Origin(
                    xyz=(
                        DRAWER_BOX_WIDTH * 0.5 - DRAWER_SIDE_THICKNESS * 0.5,
                        0.0,
                        side_z,
                    )
                ),
                material=drawer_wood,
                name="right_side",
            )
            drawer.visual(
                Box((DRAWER_BOX_WIDTH, DRAWER_BACK_THICKNESS, DRAWER_SIDE_HEIGHT)),
                origin=Origin(
                    xyz=(
                        0.0,
                        -DRAWER_BOX_DEPTH * 0.5 + DRAWER_BACK_THICKNESS * 0.5,
                        side_z,
                    )
                ),
                material=drawer_wood,
                name="back",
            )

            front_center_y = DRAWER_BOX_DEPTH * 0.5 + DRAWER_FRONT_THICKNESS * 0.5
            drawer.visual(
                Box((DRAWER_FRONT_WIDTH, DRAWER_FRONT_THICKNESS, DRAWER_FRONT_HEIGHT)),
                origin=Origin(xyz=(0.0, front_center_y, 0.0)),
                material=drawer_wood,
                name="front",
            )

            stem_center_y = (
                DRAWER_BOX_DEPTH * 0.5
                + DRAWER_FRONT_THICKNESS
                + HANDLE_STEM_LENGTH * 0.5
            )
            bar_center_y = (
                DRAWER_BOX_DEPTH * 0.5
                + DRAWER_FRONT_THICKNESS
                + HANDLE_STEM_LENGTH
                + HANDLE_BAR_RADIUS
            )
            handle_specs = (
                ("left_handle", -HANDLE_X_SPACING * 0.5),
                ("right_handle", HANDLE_X_SPACING * 0.5),
            )
            for handle_name, hx in handle_specs:
                stem_0_x = hx - HANDLE_POST_SPACING * 0.5
                stem_1_x = hx + HANDLE_POST_SPACING * 0.5
                drawer.visual(
                    Cylinder(radius=HANDLE_STEM_RADIUS, length=HANDLE_STEM_LENGTH),
                    origin=Origin(
                        xyz=(stem_0_x, stem_center_y, 0.0),
                        rpy=(pi * 0.5, 0.0, 0.0),
                    ),
                    material=handle_metal,
                    name=f"{handle_name}_stem_0",
                )
                drawer.visual(
                    Cylinder(radius=HANDLE_STEM_RADIUS, length=HANDLE_STEM_LENGTH),
                    origin=Origin(
                        xyz=(stem_1_x, stem_center_y, 0.0),
                        rpy=(pi * 0.5, 0.0, 0.0),
                    ),
                    material=handle_metal,
                    name=f"{handle_name}_stem_1",
                )
                drawer.visual(
                    Cylinder(radius=HANDLE_BAR_RADIUS, length=HANDLE_BAR_LENGTH),
                    origin=Origin(
                        xyz=(hx, bar_center_y, 0.0),
                        rpy=(0.0, pi * 0.5, 0.0),
                    ),
                    material=handle_metal,
                    name=f"{handle_name}_bar",
                )

            runner_x = DRAWER_BOX_WIDTH * 0.5 + RUNNER_THICKNESS * 0.5
            drawer.visual(
                Box((RUNNER_THICKNESS, RUNNER_LENGTH, RUNNER_HEIGHT)),
                origin=Origin(xyz=(-runner_x, RUNNER_Y_OFFSET, RUNNER_Z_OFFSET)),
                material=rail_metal,
                name="runner_left",
            )
            drawer.visual(
                Box((RUNNER_THICKNESS, RUNNER_LENGTH, RUNNER_HEIGHT)),
                origin=Origin(xyz=(runner_x, RUNNER_Y_OFFSET, RUNNER_Z_OFFSET)),
                material=rail_metal,
                name="runner_right",
            )

            drawer.inertial = Inertial.from_geometry(
                Box((DRAWER_FRONT_WIDTH, DRAWER_BOX_DEPTH + 0.06, DRAWER_FRONT_HEIGHT)),
                mass=4.5,
                origin=Origin(xyz=(0.0, 0.03, 0.0)),
            )

            model.articulation(
                f"carcass_to_{name}",
                ArticulationType.PRISMATIC,
                parent=carcass,
                child=drawer,
                origin=Origin(
                    xyz=(_column_center_x(col), DRAWER_JOINT_Y, _row_center_z(row))
                ),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(
                    effort=22.0,
                    velocity=0.40,
                    lower=0.0,
                    upper=DRAWER_TRAVEL,
                ),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, seed=0)

    carcass = object_model.get_part("carcass")
    drawer_specs: list[tuple[int, int, object, object]] = []
    for row in range(ROW_COUNT):
        for col in range(COLUMN_COUNT):
            name = _drawer_name(row, col)
            drawer = object_model.get_part(name)
            slide = object_model.get_articulation(f"carcass_to_{name}")
            drawer_specs.append((row, col, drawer, slide))

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=72)

    carcass_aabb = ctx.part_world_aabb(carcass)
    if carcass_aabb is not None:
        cmin, cmax = carcass_aabb
        width = cmax[0] - cmin[0]
        depth = cmax[1] - cmin[1]
        height = cmax[2] - cmin[2]
        ctx.check(
            "dresser_realistic_width",
            1.05 <= width <= 1.20,
            f"expected dresser width near 1.10 m, got {width:.3f}",
        )
        ctx.check(
            "dresser_realistic_depth",
            0.44 <= depth <= 0.55,
            f"expected dresser depth near 0.50 m, got {depth:.3f}",
        )
        ctx.check(
            "dresser_realistic_height",
            0.95 <= height <= 1.15,
            f"expected dresser height near 1.03 m, got {height:.3f}",
        )
        ctx.check(
            "dresser_grounded_on_feet",
            abs(cmin[2]) <= 1e-5,
            f"dresser should rest on the floor, min z = {cmin[2]:.6f}",
        )

    # Decorative top overhangs the carcass on all four sides.
    top_panel = carcass.get_visual("top_panel")
    ctx.expect_overlap(
        carcass,
        carcass,
        axes=("x", "y"),
        elem_a=top_panel,
        elem_b=carcass.get_visual("inner_top_panel"),
        min_overlap=0.30,
        name="top_panel_covers_inner_top",
    )

    # Center divider connects bottom panel up to inner top panel.
    ctx.expect_contact(
        carcass,
        carcass,
        elem_a=carcass.get_visual("center_divider"),
        elem_b=carcass.get_visual("bottom_panel"),
        name="center_divider_seats_on_bottom",
    )
    ctx.expect_contact(
        carcass,
        carcass,
        elem_a=carcass.get_visual("center_divider"),
        elem_b=carcass.get_visual("inner_top_panel"),
        name="center_divider_meets_inner_top",
    )

    # Feet are mounted under the bottom panel.
    for fname in ("foot_rear_left", "foot_rear_right", "foot_front_left", "foot_front_right"):
        ctx.expect_contact(
            carcass,
            carcass,
            elem_a=carcass.get_visual(fname),
            elem_b=carcass.get_visual("bottom_panel"),
            name=f"{fname}_mounted_to_bottom",
        )

    for row, col, drawer, slide in drawer_specs:
        prefix = f"drawer_{row}_{col}"
        limits = slide.motion_limits

        ctx.check(
            f"{prefix}_uses_prismatic",
            slide.articulation_type == ArticulationType.PRISMATIC,
            f"expected prismatic, got {slide.articulation_type}",
        )
        ctx.check(
            f"{prefix}_slides_forward",
            tuple(slide.axis) == (0.0, 1.0, 0.0),
            f"expected axis (0, 1, 0), got {slide.axis}",
        )
        ctx.check(
            f"{prefix}_travel_realistic",
            limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and 0.18 <= limits.upper <= 0.26,
            f"expected travel 0.18-0.26 m, got {limits}",
        )

        front = drawer.get_visual("front")
        left_bar = drawer.get_visual("left_handle_bar")
        right_bar = drawer.get_visual("right_handle_bar")
        left_stems = (
            drawer.get_visual("left_handle_stem_0"),
            drawer.get_visual("left_handle_stem_1"),
        )
        right_stems = (
            drawer.get_visual("right_handle_stem_0"),
            drawer.get_visual("right_handle_stem_1"),
        )

        for stem in left_stems + right_stems:
            ctx.expect_contact(
                drawer,
                drawer,
                elem_a=stem,
                elem_b=front,
                name=f"{prefix}_{stem.name}_mounted_to_front",
            )
        for bar, stems in ((left_bar, left_stems), (right_bar, right_stems)):
            for stem in stems:
                ctx.expect_contact(
                    drawer,
                    drawer,
                    elem_a=bar,
                    elem_b=stem,
                    name=f"{prefix}_{bar.name}_seated_on_{stem.name}",
                )

        runner_left = drawer.get_visual("runner_left")
        runner_right = drawer.get_visual("runner_right")
        outer_guide = carcass.get_visual(f"guide_{row}_{col}_outer")
        inner_guide = carcass.get_visual(f"guide_{row}_{col}_inner")

        with ctx.pose({slide: 0.0}):
            if col == 0:
                ctx.expect_contact(
                    drawer, carcass, elem_a=runner_left, elem_b=outer_guide,
                    name=f"{prefix}_outer_runner_engages_outer_guide_closed",
                )
                ctx.expect_contact(
                    drawer, carcass, elem_a=runner_right, elem_b=inner_guide,
                    name=f"{prefix}_inner_runner_engages_inner_guide_closed",
                )
            else:
                ctx.expect_contact(
                    drawer, carcass, elem_a=runner_right, elem_b=outer_guide,
                    name=f"{prefix}_outer_runner_engages_outer_guide_closed",
                )
                ctx.expect_contact(
                    drawer, carcass, elem_a=runner_left, elem_b=inner_guide,
                    name=f"{prefix}_inner_runner_engages_inner_guide_closed",
                )
            ctx.expect_gap(
                drawer, drawer, axis="y", min_gap=0.020,
                positive_elem=left_bar, negative_elem=front,
                name=f"{prefix}_left_bar_proud_of_front_closed",
            )
            ctx.expect_gap(
                drawer, drawer, axis="y", min_gap=0.020,
                positive_elem=right_bar, negative_elem=front,
                name=f"{prefix}_right_bar_proud_of_front_closed",
            )
            ctx.expect_within(
                drawer, carcass, axes=("x", "z"),
                inner_elem=front,
                name=f"{prefix}_front_within_opening_closed",
            )
            ctx.fail_if_parts_overlap_in_current_pose(
                name=f"{prefix}_closed_pose_no_overlap"
            )
            ctx.fail_if_isolated_parts(name=f"{prefix}_closed_pose_no_floating")

        if limits is not None and limits.upper is not None:
            with ctx.pose({slide: limits.upper}):
                ctx.expect_gap(
                    drawer, carcass, axis="y", min_gap=0.18,
                    positive_elem=front,
                    name=f"{prefix}_opens_forward",
                )
                if col == 0:
                    ctx.expect_contact(
                        drawer, carcass, elem_a=runner_left, elem_b=outer_guide,
                        name=f"{prefix}_outer_runner_retains_guide_open",
                    )
                    ctx.expect_contact(
                        drawer, carcass, elem_a=runner_right, elem_b=inner_guide,
                        name=f"{prefix}_inner_runner_retains_guide_open",
                    )
                else:
                    ctx.expect_contact(
                        drawer, carcass, elem_a=runner_right, elem_b=outer_guide,
                        name=f"{prefix}_outer_runner_retains_guide_open",
                    )
                    ctx.expect_contact(
                        drawer, carcass, elem_a=runner_left, elem_b=inner_guide,
                        name=f"{prefix}_inner_runner_retains_guide_open",
                    )
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"{prefix}_open_pose_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"{prefix}_open_pose_no_floating")

    # Reveal between vertically stacked drawer fronts in each column.
    for col in range(COLUMN_COUNT):
        for row in range(ROW_COUNT - 1):
            lower = object_model.get_part(_drawer_name(row, col))
            upper = object_model.get_part(_drawer_name(row + 1, col))
            ctx.expect_gap(
                upper, lower, axis="z",
                min_gap=0.006, max_gap=0.016,
                positive_elem=upper.get_visual("front"),
                negative_elem=lower.get_visual("front"),
                name=f"reveal_col{col}_row{row}_to_{row + 1}",
            )

    # Reveal between drawer fronts of the two columns in each row.
    for row in range(ROW_COUNT):
        left = object_model.get_part(_drawer_name(row, 0))
        right = object_model.get_part(_drawer_name(row, 1))
        ctx.expect_gap(
            right, left, axis="x",
            min_gap=0.010, max_gap=0.040,
            positive_elem=right.get_visual("front"),
            negative_elem=left.get_visual("front"),
            name=f"divider_reveal_row{row}",
        )

    return ctx.report()


object_model = build_object_model()
# >>> USER_CODE_END
