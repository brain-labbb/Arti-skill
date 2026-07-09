from __future__ import annotations

import math

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
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


COVER_W = 0.255
BINDER_H = 0.310
SPINE_W = 0.060
BOARD_T = 0.006
PLATE_X = 0.0
PLATE_Z = 0.005
PLATE_W = 0.060
PLATE_L = 0.262
FIXED_HINGE_X = -0.012
MOVING_HINGE_X = 0.020
RING_TIP_X = 0.003
RING_TIP_Z = 0.058
LEVER_PIVOT_Z = 0.0215
RING_Y = (-0.105, 0.0, 0.105)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="white_three_ring_binder",
        meta={
            "run_note": "Reference and requested category both read as a white three-ring binder; no classification mismatch suspected."
        },
    )

    white_plastic = model.material("white_plastic", rgba=(0.93, 0.94, 0.92, 1.0))
    clear_plastic = model.material("clear_plastic", rgba=(0.86, 0.92, 0.96, 0.36))
    crease_gray = model.material("soft_crease_gray", rgba=(0.64, 0.66, 0.64, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.72, 0.70, 0.64, 1.0))
    shadow_metal = model.material("shadowed_metal", rgba=(0.40, 0.40, 0.36, 1.0))
    page_white = model.material("page_white", rgba=(0.97, 0.97, 0.94, 1.0))

    def rounded_panel_mesh(name: str, width: float, height: float, thickness: float, radius: float):
        return mesh_from_geometry(
            ExtrudeGeometry(
                rounded_rect_profile(width, height, radius, corner_segments=8),
                thickness,
                cap=True,
                center=True,
            ),
            name,
        )

    def ring_tube(name: str, points):
        return mesh_from_geometry(
            tube_from_spline_points(
                points,
                radius=0.0021,
                samples_per_segment=14,
                radial_segments=18,
                cap_ends=True,
                up_hint=(0.0, 1.0, 0.0),
            ),
            name,
        )

    # Root: the wide center spine board and molded plastic hinge crease lines.
    spine = model.part("spine")
    spine.visual(
        rounded_panel_mesh("spine_board_mesh", SPINE_W, BINDER_H, BOARD_T, 0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=white_plastic,
        name="spine_board",
    )
    for x, nm in ((-SPINE_W / 2, "front_crease"), (SPINE_W / 2, "back_crease")):
        spine.visual(
            Box((0.0022, BINDER_H - 0.020, 0.0012)),
            origin=Origin(xyz=(x, 0.0, BOARD_T / 2 + 0.0006)),
            material=crease_gray,
            name=nm,
        )
    spine.visual(
        Box((0.050, BINDER_H - 0.034, 0.0016)),
        origin=Origin(xyz=(0.0, 0.0, BOARD_T / 2 + 0.0008)),
        material=clear_plastic,
        name="inner_spine_sleeve",
    )

    # Hinged front and back covers: thin rounded boards with transparent inner sleeves.
    front_cover = model.part("front_cover")
    front_cover.visual(
        rounded_panel_mesh("front_cover_mesh", COVER_W, BINDER_H, BOARD_T, 0.009),
        origin=Origin(xyz=(-COVER_W / 2, 0.0, 0.0)),
        material=white_plastic,
        name="front_board",
    )
    front_cover.visual(
        Box((COVER_W - 0.030, BINDER_H - 0.040, 0.0012)),
        origin=Origin(xyz=(-COVER_W / 2 - 0.004, 0.0, BOARD_T / 2 + 0.0006)),
        material=clear_plastic,
        name="front_clear_pocket",
    )
    front_cover.visual(
        Box((0.004, BINDER_H - 0.030, 0.0020)),
        origin=Origin(xyz=(-0.006, 0.0, BOARD_T / 2 + 0.0010)),
        material=crease_gray,
        name="front_fold_reinforcement",
    )
    front_cover.visual(
        Box((0.0010, BINDER_H - 0.030, BOARD_T)),
        origin=Origin(xyz=(-0.0005, 0.0, 0.0)),
        material=white_plastic,
        name="front_hinge_web",
    )

    back_cover = model.part("back_cover")
    back_cover.visual(
        rounded_panel_mesh("back_cover_mesh", COVER_W, BINDER_H, BOARD_T, 0.009),
        origin=Origin(xyz=(COVER_W / 2, 0.0, 0.0)),
        material=white_plastic,
        name="back_board",
    )
    back_cover.visual(
        Box((COVER_W - 0.035, BINDER_H - 0.045, 0.0012)),
        origin=Origin(xyz=(COVER_W / 2 + 0.006, 0.0, BOARD_T / 2 + 0.0006)),
        material=clear_plastic,
        name="back_clear_pocket",
    )
    back_cover.visual(
        Box((0.004, BINDER_H - 0.030, 0.0020)),
        origin=Origin(xyz=(0.006, 0.0, BOARD_T / 2 + 0.0010)),
        material=crease_gray,
        name="back_fold_reinforcement",
    )
    back_cover.visual(
        Box((0.0010, BINDER_H - 0.030, BOARD_T)),
        origin=Origin(xyz=(0.0005, 0.0, 0.0)),
        material=white_plastic,
        name="back_hinge_web",
    )

    model.articulation(
        "spine_to_front_cover",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=front_cover,
        origin=Origin(xyz=(-SPINE_W / 2, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=0.0, upper=2.15),
    )
    model.articulation(
        "spine_to_back_cover",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=back_cover,
        origin=Origin(xyz=(SPINE_W / 2, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=0.0, upper=2.15),
    )

    # Metal ring mechanism fixed to the spine, with feet, rivets, and fixed ring halves.
    ring_bar = model.part("ring_bar")
    ring_bar.visual(
        Box((PLATE_W, PLATE_L, 0.004)),
        origin=Origin(xyz=(PLATE_X, 0.0, PLATE_Z)),
        material=satin_metal,
        name="metal_plate",
    )
    ring_bar.visual(
        Box((0.018, PLATE_L - 0.018, 0.005)),
        origin=Origin(xyz=(PLATE_X, 0.0, PLATE_Z + 0.0035)),
        material=satin_metal,
        name="raised_spine_channel",
    )
    # Fixed-side hinge rod and support blocks tie the fixed ring halves into the plate.
    ring_bar.visual(
        Cylinder(radius=0.0030, length=PLATE_L - 0.018),
        origin=Origin(xyz=(FIXED_HINGE_X, 0.0, 0.0175), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=shadow_metal,
        name="fixed_hinge_rod",
    )
    for i, y in enumerate(RING_Y):
        ring_bar.visual(
            Box((0.012, 0.015, 0.006)),
            origin=Origin(xyz=(FIXED_HINGE_X, y, 0.0115)),
            material=satin_metal,
            name=f"fixed_saddle_{i}",
        )
        ring_bar.visual(
            Box((0.011, 0.015, 0.0075)),
            origin=Origin(xyz=(MOVING_HINGE_X, y, 0.01075)),
            material=satin_metal,
            name=f"moving_saddle_{i}",
        )
        ring_bar.visual(
            Cylinder(radius=0.0034, length=0.0018),
            origin=Origin(xyz=(PLATE_X, y + 0.020, 0.0105)),
            material=shadow_metal,
            name=f"rivet_{i}",
        )
        ring_bar.visual(
            ring_tube(
                f"fixed_ring_half_{i}_mesh",
                [
                    (FIXED_HINGE_X, y, 0.0175),
                    (FIXED_HINGE_X - 0.004, y, 0.031),
                    (FIXED_HINGE_X - 0.0025, y, 0.047),
                    (RING_TIP_X, y, RING_TIP_Z),
                ],
            ),
            material=satin_metal,
            name=f"fixed_ring_half_{i}",
        )
    ring_bar.visual(
        Cylinder(radius=0.0085, length=0.011),
        origin=Origin(xyz=(PLATE_X, 0.0, 0.0150)),
        material=satin_metal,
        name="lever_center_standoff",
    )
    for name, y, width_y in (
        ("lower_lever_standoff", -0.126, 0.018),
        ("upper_lever_standoff", 0.126, 0.018),
    ):
        ring_bar.visual(
            Box((0.014, width_y, 0.011)),
            origin=Origin(xyz=(PLATE_X, y, 0.0150)),
            material=satin_metal,
            name=name,
        )

    model.articulation(
        "spine_to_ring_bar",
        ArticulationType.FIXED,
        parent=spine,
        child=ring_bar,
        origin=Origin(),
    )

    # One linked moving half assembly carries the three opening halves on a shared rod.
    ring_halves = model.part("ring_halves")
    ring_halves.visual(
        Cylinder(radius=0.0030, length=PLATE_L - 0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=shadow_metal,
        name="moving_hinge_rod",
    )
    for i, y in enumerate(RING_Y):
        ring_halves.visual(
            Box((0.008, 0.013, 0.008)),
            origin=Origin(xyz=(0.0, y, 0.0025)),
            material=satin_metal,
            name=f"moving_lug_{i}",
        )
        ring_halves.visual(
            ring_tube(
                f"moving_ring_half_{i}_mesh",
                [
                    (0.0, y, 0.0),
                    (0.0035, y, 0.014),
                    (0.0030, y, 0.034),
                    (RING_TIP_X - MOVING_HINGE_X, y, RING_TIP_Z - 0.0175),
                ],
            ),
            material=satin_metal,
            name=f"moving_ring_half_{i}",
        )

    model.articulation(
        "ring_bar_to_ring_halves",
        ArticulationType.REVOLUTE,
        parent=ring_bar,
        child=ring_halves,
        origin=Origin(xyz=(MOVING_HINGE_X, 0.0, 0.0175)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=0.62),
    )

    # End lever tabs are linked by a slim cam strip, as on a real binder mechanism.
    lever_tabs = model.part("lever_tabs")
    lever_tabs.visual(
        Box((0.011, PLATE_L - 0.018, 0.0022)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=shadow_metal,
        name="lever_linkage",
    )
    for i, y in enumerate((-0.126, 0.126)):
        lever_tabs.visual(
            Box((0.026, 0.020, 0.0032)),
            origin=Origin(xyz=(0.000, y, 0.0065), rpy=(0.26 if y > 0 else -0.26, 0.0, 0.0)),
            material=satin_metal,
            name=f"lever_tab_{i}",
        )
        lever_tabs.visual(
            Cylinder(radius=0.0025, length=0.012),
            origin=Origin(xyz=(0.0, y * 0.96, 0.003), rpy=(0.0, math.pi / 2, 0.0)),
            material=shadow_metal,
            name=f"lever_pivot_pin_{i}",
        )

    model.articulation(
        "ring_bar_to_lever_tabs",
        ArticulationType.REVOLUTE,
        parent=ring_bar,
        child=lever_tabs,
        origin=Origin(xyz=(PLATE_X, 0.0, LEVER_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.8,
            velocity=4.0,
            lower=-math.radians(5.0),
            upper=math.radians(5.0),
        ),
    )

    # A low, retained paper stack gives the rings clear functional context without adding props.
    paper_stack = model.part("paper_stack")
    paper_stack.visual(
        Box((0.170, 0.250, 0.006)),
        origin=Origin(xyz=(0.108, 0.0, BOARD_T / 2 + 0.0035)),
        material=page_white,
        name="paper_stack_block",
    )
    for i, y in enumerate(RING_Y):
        paper_stack.visual(
            Cylinder(radius=0.0060, length=0.0070),
            origin=Origin(xyz=(0.027, y, BOARD_T / 2 + 0.0070)),
            material=white_plastic,
            name=f"reinforced_hole_{i}",
        )
    model.articulation(
        "back_cover_to_paper_stack",
        ArticulationType.FIXED,
        parent=back_cover,
        child=paper_stack,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    spine = object_model.get_part("spine")
    front_cover = object_model.get_part("front_cover")
    back_cover = object_model.get_part("back_cover")
    ring_bar = object_model.get_part("ring_bar")
    ring_halves = object_model.get_part("ring_halves")
    lever_tabs = object_model.get_part("lever_tabs")
    paper_stack = object_model.get_part("paper_stack")

    front_hinge = object_model.get_articulation("spine_to_front_cover")
    back_hinge = object_model.get_articulation("spine_to_back_cover")
    ring_joint = object_model.get_articulation("ring_bar_to_ring_halves")
    lever_joint = object_model.get_articulation("ring_bar_to_lever_tabs")

    ctx.expect_contact(
        front_cover,
        spine,
        elem_a="front_board",
        elem_b="spine_board",
        contact_tol=0.001,
        name="front cover shares the spine hinge line",
    )
    ctx.expect_contact(
        back_cover,
        spine,
        elem_a="back_board",
        elem_b="spine_board",
        contact_tol=0.001,
        name="back cover shares the spine hinge line",
    )
    ctx.expect_overlap(
        ring_bar,
        spine,
        axes="y",
        min_overlap=0.24,
        elem_a="metal_plate",
        elem_b="spine_board",
        name="metal mechanism runs along the binder spine",
    )
    ctx.expect_within(
        ring_bar,
        spine,
        axes="y",
        margin=0.004,
        elem_a="metal_plate",
        elem_b="spine_board",
        name="ring plate fits within spine length",
    )
    plate_aabb = ctx.part_element_world_aabb(ring_bar, elem="metal_plate")
    spine_aabb = ctx.part_element_world_aabb(spine, elem="spine_board")
    ctx.check(
        "metal mechanism is centered on the spine",
        plate_aabb is not None
        and spine_aabb is not None
        and abs(((plate_aabb[0][0] + plate_aabb[1][0]) / 2) - ((spine_aabb[0][0] + spine_aabb[1][0]) / 2))
        < 0.001,
        details=f"plate={plate_aabb}, spine={spine_aabb}",
    )
    ctx.expect_overlap(
        paper_stack,
        back_cover,
        axes="xy",
        min_overlap=0.12,
        elem_a="paper_stack_block",
        elem_b="back_board",
        name="paper stack sits on the back cover",
    )
    for i in range(3):
        ctx.expect_overlap(
            ring_halves,
            ring_bar,
            axes="y",
            min_overlap=0.003,
            elem_a=f"moving_ring_half_{i}",
            elem_b=f"fixed_ring_half_{i}",
            name=f"ring {i} halves are aligned at the same station",
        )

    front_rest = ctx.part_world_aabb(front_cover)
    back_rest = ctx.part_world_aabb(back_cover)
    with ctx.pose({front_hinge: 1.25, back_hinge: 1.25}):
        front_closed = ctx.part_world_aabb(front_cover)
        back_closed = ctx.part_world_aabb(back_cover)
    ctx.check(
        "covers hinge upward from the flat-open binder",
        front_rest is not None
        and front_closed is not None
        and back_rest is not None
        and back_closed is not None
        and front_closed[1][2] > front_rest[1][2] + 0.17
        and back_closed[1][2] > back_rest[1][2] + 0.17,
        details=f"front_rest={front_rest}, front_closed={front_closed}, back_rest={back_rest}, back_closed={back_closed}",
    )

    closed_ring = ctx.part_element_world_aabb(ring_halves, elem="moving_ring_half_1")
    with ctx.pose({ring_joint: 0.60}):
        open_ring = ctx.part_element_world_aabb(ring_halves, elem="moving_ring_half_1")
    ctx.check(
        "ring halves swing outward to open",
        closed_ring is not None and open_ring is not None and open_ring[1][0] > closed_ring[1][0] + 0.020,
        details=f"closed={closed_ring}, open={open_ring}",
    )

    lever_rest = ctx.part_world_aabb(lever_tabs)
    with ctx.pose({lever_joint: math.radians(5.0)}):
        lever_moved = ctx.part_world_aabb(lever_tabs)
    ctx.check(
        "lever tabs rock on a revolute pivot",
        lever_rest is not None
        and lever_moved is not None
        and abs(lever_moved[1][2] - lever_rest[1][2]) > 0.002,
        details=f"rest={lever_rest}, moved={lever_moved}",
    )

    return ctx.report()


object_model = build_object_model()
