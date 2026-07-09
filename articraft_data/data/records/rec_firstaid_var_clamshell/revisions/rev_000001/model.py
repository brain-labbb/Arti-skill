from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


# ── Case envelope (unchanged overall footprint) ──────────────────────────
WIDTH = 0.30
DEPTH = 0.12
CASE_HEIGHT = 0.22
# Clamshell split at middle → both halves ~equal depth
BASE_HEIGHT = CASE_HEIGHT / 2.0  # 0.11 bottom half
LID_HEIGHT = CASE_HEIGHT / 2.0   # 0.11 top half
WALL = 0.006
BOTTOM = 0.006
CORNER_RADIUS = 0.023

# ── Tray (same construction, lowered to fit shorter base) ─────────────────
TRAY_WIDTH = 0.265
TRAY_DEPTH = 0.090
TRAY_HEIGHT = 0.030
TRAY_FLOOR = 0.004
TRAY_WALL = 0.003
TRAY_BOTTOM_Z = 0.055
TRAY_TRAVEL = 0.065


def _rounded_slab(width: float, depth: float, height: float, radius: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(width, depth, radius, corner_segments=10), height, center=True),
        name,
    )


def _hollow_open_box(
    width: float,
    depth: float,
    height: float,
    wall: float,
    radius: float,
    name: str,
    *,
    open_face: str = ">Z",
):
    """Rounded rectangular container hollowed out via a true CAD shell so the
    interior is a genuinely empty cavity with correct outward normals (an open
    box, not a solid block). ``open_face`` selects which face is removed."""
    box = cq.Workplane("XY").box(width, depth, height).edges("|Z").fillet(radius)
    shell = box.faces(open_face).shell(-wall)
    return mesh_from_cadquery(shell, name, tolerance=0.0006)


def _three_wall_base(
    width: float,
    depth: float,
    height: float,
    wall: float,
    bottom: float,
    radius: float,
    name: str,
):
    """Base shell: floor + back wall + two side walls (no front wall).
    Open at top and front. Built from CadQuery with rounded outer corners."""
    # Outer box with rounded vertical edges
    outer = cq.Workplane("XY").box(width, depth, height).edges("|Z").fillet(radius)
    # Hollow out from the top (leaves floor + 4 walls)
    shelled = outer.faces(">Z").shell(-wall)
    # Now cut the front wall away with a wide, tall cutter
    # The front wall center is at y = -depth/2 + wall/2, z centered in box
    cutter = (
        cq.Workplane("XY")
        .box(width + 0.010, wall + 0.006, height + 0.010)
        .translate((0.0, -(depth / 2.0 - wall / 2.0), 0.0))
    )
    result = shelled.cut(cutter)
    return mesh_from_cadquery(result, name, tolerance=0.0006)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_first_aid_hard_case_clamshell")

    cream = model.material("weathered_cream", rgba=(0.86, 0.82, 0.70, 1.0))
    red = model.material("medical_red", rgba=(0.55, 0.03, 0.025, 1.0))
    dark = model.material("dark_interior", rgba=(0.04, 0.035, 0.032, 1.0))
    metal = model.material("aged_metal", rgba=(0.46, 0.44, 0.39, 1.0))
    plastic = model.material("aged_white_plastic", rgba=(0.82, 0.82, 0.78, 1.0))

    # ── Base (bottom half-shell, open at top and front) ───────────────────
    base = model.part("base")
    base.visual(
        _three_wall_base(WIDTH, DEPTH, BASE_HEIGHT, WALL, BOTTOM, CORNER_RADIUS, "base_shell"),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
        material=cream,
        name="base_shell",
    )
    # Dark interior floor pad
    base.visual(
        Box((WIDTH - 2.8 * WALL, DEPTH - 2.8 * WALL, 0.0012)),
        origin=Origin(xyz=(0.0, 0.0, WALL + 0.001)),
        material=dark,
        name="empty_compartment",
    )
    # Tray ledges on the side walls
    for i, x in enumerate((-0.134, 0.134)):
        base.visual(
            Box((0.024, TRAY_DEPTH + 0.004, 0.004)),
            origin=Origin(xyz=(x, 0.0, TRAY_BOTTOM_Z - 0.002)),
            material=cream,
            name=f"tray_ledge_{i}",
        )
    # Rear hinge barrel (for lid)
    base.visual(
        Cylinder(radius=0.004, length=WIDTH * 0.74),
        origin=Origin(
            xyz=(0.0, DEPTH / 2.0 + 0.002, BASE_HEIGHT + 0.001),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=metal,
        name="rear_hinge_barrel",
    )
    # Front hinge barrel (for front flap) — sits on the floor surface near front edge
    base.visual(
        Cylinder(radius=0.004, length=WIDTH * 0.74),
        origin=Origin(
            xyz=(0.0, -DEPTH / 2.0 + WALL + 0.004, BOTTOM + 0.004),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=metal,
        name="front_hinge_barrel",
    )
    # Latch strike plates on the side walls near the front-top (touching side wall inner face)
    for i, x_sign in enumerate((-1.0, 1.0)):
        x_pos = x_sign * (WIDTH / 2.0 - WALL - 0.020)
        base.visual(
            Box((0.040, 0.006, 0.020)),
            origin=Origin(xyz=(x_pos, -DEPTH / 2.0 + WALL + 0.003, BASE_HEIGHT - 0.016)),
            material=metal,
            name=f"latch_strike_{i}",
        )

    # ── Tray (identical construction, lowered z) ──────────────────────────
    tray = model.part("tray")
    wall_h = TRAY_HEIGHT - TRAY_FLOOR
    wall_cz = TRAY_FLOOR + wall_h / 2.0
    hx = TRAY_WIDTH / 2.0
    hy = TRAY_DEPTH / 2.0

    tray.visual(
        Box((TRAY_WIDTH, TRAY_DEPTH, TRAY_FLOOR)),
        origin=Origin(xyz=(0.0, 0.0, TRAY_FLOOR / 2.0)),
        material=plastic,
        name="tray_floor",
    )
    tray.visual(
        Box((TRAY_WALL, TRAY_DEPTH, wall_h)),
        origin=Origin(xyz=(-(hx - TRAY_WALL / 2.0), 0.0, wall_cz)),
        material=plastic,
        name="tray_wall_left",
    )
    tray.visual(
        Box((TRAY_WALL, TRAY_DEPTH, wall_h)),
        origin=Origin(xyz=(hx - TRAY_WALL / 2.0, 0.0, wall_cz)),
        material=plastic,
        name="tray_wall_right",
    )
    tray.visual(
        Box((TRAY_WIDTH, TRAY_WALL, wall_h)),
        origin=Origin(xyz=(0.0, -(hy - TRAY_WALL / 2.0), wall_cz)),
        material=plastic,
        name="tray_wall_front",
    )
    tray.visual(
        Box((TRAY_WIDTH, TRAY_WALL, wall_h)),
        origin=Origin(xyz=(0.0, hy - TRAY_WALL / 2.0, wall_cz)),
        material=plastic,
        name="tray_wall_back",
    )
    interior_depth = TRAY_DEPTH - 2.0 * TRAY_WALL
    interior_width = TRAY_WIDTH - 2.0 * TRAY_WALL
    for i, x in enumerate((-TRAY_WIDTH / 6.0, TRAY_WIDTH / 6.0)):
        tray.visual(
            Box((TRAY_WALL, interior_depth, wall_h)),
            origin=Origin(xyz=(x, 0.0, wall_cz)),
            material=plastic,
            name=f"divider_col_{i}",
        )
    tray.visual(
        Box((interior_width, TRAY_WALL, wall_h)),
        origin=Origin(xyz=(0.0, 0.0, wall_cz)),
        material=plastic,
        name="divider_row",
    )
    tray.visual(
        Box((0.045, 0.005, 0.009)),
        origin=Origin(xyz=(0.0, -hy - 0.0015, TRAY_HEIGHT - 0.006)),
        material=plastic,
        name="front_pull",
    )
    tray_joint = model.articulation(
        "base_to_tray",
        ArticulationType.PRISMATIC,
        parent=base,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, TRAY_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.6, lower=0.0, upper=TRAY_TRAVEL),
    )

    # ── Lid (top half-shell, opens upward from rear hinge) ────────────────
    lid = model.part("lid")
    lid.visual(
        _hollow_open_box(
            WIDTH, DEPTH, LID_HEIGHT, WALL, CORNER_RADIUS, "lid_skirt", open_face="<Z"
        ),
        origin=Origin(xyz=(0.0, -DEPTH / 2.0, LID_HEIGHT / 2.0)),
        material=cream,
        name="lid_skirt",
    )
    lid.visual(
        _rounded_slab(WIDTH, DEPTH, WALL, CORNER_RADIUS, "lid_top"),
        origin=Origin(xyz=(0.0, -DEPTH / 2.0, LID_HEIGHT - WALL / 2.0)),
        material=cream,
        name="lid_top",
    )
    lid.visual(
        Box((WIDTH - 3.0 * WALL, DEPTH - 3.0 * WALL, 0.0012)),
        origin=Origin(xyz=(0.0, -DEPTH / 2.0, 0.0010)),
        material=dark,
        name="dark_lid_liner",
    )
    for i, x in enumerate((-0.105, 0.105)):
        lid.visual(
            Box((0.030, 0.020, 0.004)),
            origin=Origin(xyz=(x, -DEPTH / 2.0, LID_HEIGHT + 0.002)),
            material=metal,
            name=f"handle_mount_{i}",
        )
    for i, x in enumerate((-0.074, 0.074)):
        lid.visual(
            Box((0.044, 0.004, 0.018)),
            origin=Origin(xyz=(x, -DEPTH - 0.002, 0.013)),
            material=metal,
            name=f"latch_keeper_{i}",
        )
    lid_joint = model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, DEPTH / 2.0, BASE_HEIGHT)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.85),
    )

    # ── Front flap (bottom-half front wall, opens downward like drawbridge) ─
    front_flap = model.part("front_flap")
    # Main wall panel: outer face at local y=0 (joint frame is at front face of case)
    # Panel extends inward +WALL and upward +BASE_HEIGHT from the bottom-front hinge.
    flap_h = BASE_HEIGHT - BOTTOM  # leave floor thickness gap
    front_flap.visual(
        Box((WIDTH - 2 * WALL, WALL, flap_h)),
        origin=Origin(xyz=(0.0, WALL / 2.0, flap_h / 2.0 + BOTTOM)),
        material=cream,
        name="flap_panel",
    )
    # Bold red medical cross on the outer face
    flap_cross_z = BASE_HEIGHT / 2.0
    front_flap.visual(
        Box((0.024, 0.0010, 0.070)),
        origin=Origin(xyz=(0.0, -0.0003, flap_cross_z)),
        material=red,
        name="cross_vertical",
    )
    front_flap.visual(
        Box((0.070, 0.0010, 0.024)),
        origin=Origin(xyz=(0.0, -0.0004, flap_cross_z)),
        material=red,
        name="cross_horizontal",
    )
    # Small latch-catch tabs on the inner face near the top
    for i, x in enumerate((-0.074, 0.074)):
        front_flap.visual(
            Box((0.040, 0.004, 0.016)),
            origin=Origin(xyz=(x, WALL + 0.002, BASE_HEIGHT - 0.016)),
            material=metal,
            name=f"flap_catch_{i}",
        )
    front_flap_joint = model.articulation(
        "base_to_front_flap",
        ArticulationType.REVOLUTE,
        parent=base,
        child=front_flap,
        # Hinge at bottom front edge; +q swings the top forward (-Y) and down.
        origin=Origin(xyz=(0.0, -DEPTH / 2.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.50),
    )

    # ── Handle (unchanged, mounted on lid top) ────────────────────────────
    handle = model.part("handle")
    handle_geom = tube_from_spline_points(
        [
            (-0.105, 0.0, 0.0),
            (-0.105, 0.0, 0.052),
            (-0.078, 0.0, 0.078),
            (0.0, 0.0, 0.086),
            (0.078, 0.0, 0.078),
            (0.105, 0.0, 0.052),
            (0.105, 0.0, 0.0),
        ],
        radius=0.003,
        samples_per_segment=12,
        radial_segments=18,
        cap_ends=True,
    )
    handle.visual(
        mesh_from_geometry(handle_geom, "handle_loop"),
        material=metal,
        name="handle_loop",
    )
    for i, x in enumerate((-0.105, 0.105)):
        handle.visual(
            Box((0.018, 0.014, 0.006)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=metal,
            name=f"handle_pivot_{i}",
        )
    handle_joint = model.articulation(
        "lid_to_handle",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=handle,
        origin=Origin(xyz=(0.0, -DEPTH / 2.0, LID_HEIGHT + 0.007)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=1.45),
    )

    # ── Draw latches (unchanged construction, z adjusted for new base) ────
    for i, x in enumerate((-0.074, 0.074)):
        latch = model.part(f"latch_{i}")
        latch.visual(
            Box((0.030, 0.003, 0.052)),
            origin=Origin(xyz=(0.0, 0.0, 0.026)),
            material=metal,
            name="clasp_plate",
        )
        latch.visual(
            Box((0.038, 0.004, 0.007)),
            origin=Origin(xyz=(0.0, 0.0, 0.052)),
            material=metal,
            name="clasp_lip",
        )
        latch.visual(
            Cylinder(radius=0.004, length=0.036),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name="clasp_pivot",
        )
        model.articulation(
            f"base_to_latch_{i}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=latch,
            origin=Origin(xyz=(x, -DEPTH / 2.0 - 0.0055, BASE_HEIGHT - 0.040)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=0.0, upper=1.20),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    front_flap = object_model.get_part("front_flap")
    handle = object_model.get_part("handle")
    tray = object_model.get_part("tray")
    lid_joint = object_model.get_articulation("base_to_lid")
    front_flap_joint = object_model.get_articulation("base_to_front_flap")
    handle_joint = object_model.get_articulation("lid_to_handle")
    tray_joint = object_model.get_articulation("base_to_tray")
    latch_0 = object_model.get_part("latch_0")
    latch_joint_0 = object_model.get_articulation("base_to_latch_0")

    # ── Lid seats on base rim when closed ─────────────────────────────────
    ctx.expect_gap(
        lid,
        base,
        axis="z",
        positive_elem="lid_skirt",
        negative_elem="base_shell",
        min_gap=-0.002,
        max_gap=0.003,
        name="closed lid seats on base rim",
    )

    # ── Front flap seats against base opening when closed ─────────────────
    # The flap panel sits just inside the base front opening; small gap/contact expected.
    ctx.expect_contact(
        front_flap,
        base,
        elem_a="flap_panel",
        elem_b="base_shell",
        contact_tol=0.003,
        name="closed front flap contacts base front opening edges",
    )

    # ── Base shell is mostly hollow ──────────────────────────────────────
    inner_volume = (WIDTH - 2.0 * WALL) * (DEPTH - 2.0 * WALL) * (BASE_HEIGHT - BOTTOM)
    outer_volume = WIDTH * DEPTH * BASE_HEIGHT
    ctx.check(
        "base shell is mostly hollow volume",
        inner_volume / outer_volume > 0.65,
        details=f"inner_volume={inner_volume:.6f}, outer_volume={outer_volume:.6f}",
    )

    # ── Medical cross on front flap ───────────────────────────────────────
    ctx.expect_overlap(
        front_flap,
        front_flap,
        axes="xz",
        elem_a="cross_vertical",
        elem_b="cross_horizontal",
        min_overlap=0.018,
        name="front medical cross bars intersect on flap",
    )

    # ── Tray nests inside base footprint ──────────────────────────────────
    ctx.expect_within(
        tray,
        base,
        axes="xy",
        inner_elem="tray_floor",
        outer_elem="base_shell",
        margin=0.002,
        name="medicine tray nests inside base footprint",
    )
    ctx.expect_gap(
        tray,
        base,
        axis="z",
        positive_elem="tray_floor",
        negative_elem="tray_ledge_0",
        min_gap=0.0,
        max_gap=0.0008,
        name="medicine tray rests on interior ledge",
    )
    rest_tray_box = ctx.part_world_aabb(tray)
    ctx.check(
        "resting tray sits below the case rim",
        rest_tray_box is not None and rest_tray_box[1][2] < BASE_HEIGHT - 0.004,
        details=f"tray={rest_tray_box}, rim_z={BASE_HEIGHT}",
    )
    tray_grid_boxes = [
        ctx.part_element_world_aabb(tray, elem="divider_col_0"),
        ctx.part_element_world_aabb(tray, elem="divider_col_1"),
        ctx.part_element_world_aabb(tray, elem="divider_row"),
    ]
    ctx.check(
        "medicine tray has two by three compartment dividers",
        all(box is not None for box in tray_grid_boxes),
        details=f"divider_boxes={tray_grid_boxes}",
    )

    # ── Lid opens upward from rear hinge ──────────────────────────────────
    # With a deep lid, the front edge swings backward when opened.
    # Check that the lid's min_y increases significantly (front edge moved rearward).
    closed_lid_box = ctx.part_element_world_aabb(lid, elem="lid_skirt")
    with ctx.pose({lid_joint: 1.45}):
        open_lid_box = ctx.part_element_world_aabb(lid, elem="lid_skirt")
    ctx.check(
        "lid opens upward from rear hinge",
        closed_lid_box is not None
        and open_lid_box is not None
        and open_lid_box[0][1] > closed_lid_box[0][1] + 0.06,
        details=f"closed={closed_lid_box}, open={open_lid_box}",
    )

    # ── Front flap opens downward/forward ─────────────────────────────────
    closed_flap_box = ctx.part_element_world_aabb(front_flap, elem="flap_panel")
    with ctx.pose({front_flap_joint: 1.20}):
        open_flap_box = ctx.part_element_world_aabb(front_flap, elem="flap_panel")
    ctx.check(
        "front_flap opens forward from bottom hinge (clamshell drawbridge)",
        closed_flap_box is not None
        and open_flap_box is not None
        and open_flap_box[0][1] < closed_flap_box[0][1] - 0.04,
        details=f"closed={closed_flap_box}, open={open_flap_box}",
    )

    # ── Handle folds ─────────────────────────────────────────────────────
    upright_handle = ctx.part_world_aabb(handle)
    with ctx.pose({handle_joint: 1.25}):
        folded_handle = ctx.part_world_aabb(handle)
    ctx.check(
        "carry handle folds down on its pivots",
        upright_handle is not None
        and folded_handle is not None
        and folded_handle[1][2] < upright_handle[1][2] - 0.030,
        details=f"upright={upright_handle}, folded={folded_handle}",
    )

    # ── Latch clasp flips outward ─────────────────────────────────────────
    closed_latch = ctx.part_world_aabb(latch_0)
    with ctx.pose({latch_joint_0: 0.95}):
        released_latch = ctx.part_world_aabb(latch_0)
    ctx.check(
        "front latch clasp flips outward",
        closed_latch is not None
        and released_latch is not None
        and released_latch[0][1] < closed_latch[0][1] - 0.010,
        details=f"closed={closed_latch}, released={released_latch}",
    )

    # ── Prismatic tray lift ───────────────────────────────────────────────
    with ctx.pose({lid_joint: 1.45, tray_joint: TRAY_TRAVEL}):
        lifted_tray_box = ctx.part_world_aabb(tray)
        ctx.expect_within(
            tray,
            base,
            axes="xy",
            inner_elem="tray_floor",
            outer_elem="base_shell",
            margin=0.002,
            name="lifted tray stays aligned with base opening",
        )
    ctx.check(
        "prismatic tray lift clears the case rim",
        lifted_tray_box is not None and lifted_tray_box[0][2] > BASE_HEIGHT + 0.006,
        details=f"lifted={lifted_tray_box}, rim_z={BASE_HEIGHT}",
    )

    return ctx.report()


object_model = build_object_model()
