from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="whiteboard_easel",
        meta={
            "note": (
                "Reference and folder agree on a whiteboard easel; no classification "
                "mismatch suspected. Variant: four-post rigid floor stand replacing "
                "telescoping legs and wheeled H-base."
            )
        },
    )

    blue = model.material("powder_coated_blue", rgba=(0.0, 0.18, 0.38, 1.0))
    white = model.material("gloss_white_board", rgba=(0.96, 0.97, 0.94, 1.0))
    dark = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    metal = model.material("brushed_metal", rgba=(0.67, 0.70, 0.72, 1.0))
    grey = model.material("soft_grey_hardware", rgba=(0.24, 0.26, 0.28, 1.0))

    board = model.part("board_frame")

    # ── Board surface and frame rails (KEEP) ────────────────────────────
    board.visual(
        Box((0.685, 0.014, 1.040)),
        origin=Origin(xyz=(0.0, -0.018, 1.355)),
        material=white,
        name="white_surface",
    )
    board.visual(
        Box((0.805, 0.052, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, 1.920)),
        material=blue,
        name="top_rail",
    )
    board.visual(
        Box((0.785, 0.050, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.790)),
        material=blue,
        name="bottom_rail",
    )
    for side, x in (("side_0", -0.395), ("side_1", 0.395)):
        board.visual(
            Box((0.060, 0.052, 1.165)),
            origin=Origin(xyz=(x, 0.0, 1.355)),
            material=blue,
            name=f"{side}_rail",
        )

    # ── Marker tray (KEEP) ──────────────────────────────────────────────
    board.visual(
        Box((0.630, 0.085, 0.022)),
        origin=Origin(xyz=(0.0, -0.067, 0.730)),
        material=blue,
        name="marker_tray_shelf",
    )
    board.visual(
        Box((0.630, 0.018, 0.052)),
        origin=Origin(xyz=(0.0, -0.025, 0.755)),
        material=blue,
        name="tray_back_lip",
    )
    board.visual(
        Box((0.045, 0.045, 0.070)),
        origin=Origin(xyz=(-0.260, -0.045, 0.775)),
        material=blue,
        name="tray_bracket_0",
    )
    board.visual(
        Box((0.045, 0.045, 0.070)),
        origin=Origin(xyz=(0.260, -0.045, 0.775)),
        material=blue,
        name="tray_bracket_1",
    )

    # ── Four-post rigid floor stand ─────────────────────────────────────
    # Four vertical tubular steel posts arranged in a rectangle (2 front,
    # 2 rear), with rubber floor glides at each base.
    post_height = 1.95
    post_radius = 0.019
    post_specs = [
        (-0.44, 0.0),   # post_0: front left
        (0.44, 0.0),    # post_1: front right
        (-0.44, 0.40),  # post_2: rear left
        (0.44, 0.40),   # post_3: rear right
    ]
    for i, (px, py) in enumerate(post_specs):
        board.visual(
            Cylinder(radius=post_radius, length=post_height),
            origin=Origin(xyz=(px, py, post_height * 0.5)),
            material=blue,
            name=f"post_{i}",
        )
        # Rubber floor glide cap at each post base
        board.visual(
            Cylinder(radius=0.024, length=0.012),
            origin=Origin(xyz=(px, py, 0.006)),
            material=dark,
            name=f"glide_{i}",
        )

    # ── Crossbars forming a closed ladder frame ─────────────────────────
    bar_span = 0.90  # reaches into both front posts (past post center)

    # Front lower crossbar (KEEP by name)
    board.visual(
        Box((bar_span, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.390)),
        material=blue,
        name="lower_crossbar",
    )
    for idx, x in enumerate((-0.340, 0.340)):
        board.visual(
            Box((0.050, 0.024, 0.040)),
            origin=Origin(xyz=(x, 0.0, 0.390)),
            material=blue,
            name=f"crossbar_mount_{idx}",
        )

    # Front upper crossbar
    board.visual(
        Box((bar_span, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 1.880)),
        material=blue,
        name="upper_front_bar",
    )

    # Rear crossbars (lower and upper)
    board.visual(
        Box((bar_span, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, 0.40, 0.390)),
        material=blue,
        name="lower_rear_bar",
    )
    board.visual(
        Box((bar_span, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, 0.40, 1.880)),
        material=blue,
        name="upper_rear_bar",
    )

    # Side crossbars connecting front and rear posts (lower and upper)
    for i, x in enumerate((-0.44, 0.44)):
        board.visual(
            Box((0.035, 0.40, 0.035)),
            origin=Origin(xyz=(x, 0.20, 0.390)),
            material=blue,
            name=f"side_lower_bar_{i}",
        )
        board.visual(
            Box((0.035, 0.40, 0.035)),
            origin=Origin(xyz=(x, 0.20, 1.880)),
            material=blue,
            name=f"side_upper_bar_{i}",
        )

    # Small gusset plates at the post-to-crossbar junctions for realism
    for i, (px, py) in enumerate(post_specs):
        for z_lev, tag in ((0.390, "lower"), (1.880, "upper")):
            board.visual(
                Box((0.048, 0.048, 0.028)),
                origin=Origin(xyz=(px, py, z_lev)),
                material=blue,
                name=f"gusset_{i}_{tag}",
            )

    # ── Clamp knobs (KEEP, revolute joints) ─────────────────────────────
    # Board-locking clamp knobs on the outer face of each front post, used
    # to release the board for reversing on a classroom reversible easel.
    for idx, sign in enumerate((-1.0, 1.0)):
        knob = model.part(f"clamp_{idx}")
        knob.visual(
            Cylinder(radius=0.007, length=0.044),
            origin=Origin(xyz=(sign * 0.022, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name="threaded_stem",
        )
        knob.visual(
            Cylinder(radius=0.030, length=0.014),
            origin=Origin(xyz=(sign * 0.049, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name="thumb_wheel",
        )
        model.articulation(
            f"board_to_clamp_{idx}",
            ArticulationType.REVOLUTE,
            parent=board,
            child=knob,
            origin=Origin(xyz=(sign * 0.459, 0.0, 0.690)),
            axis=(sign, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.4, velocity=4.0, lower=-math.pi, upper=math.pi,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    board = object_model.get_part("board_frame")

    # ── Board surface aspect ratio (KEEP) ───────────────────────────────
    board_aabb = ctx.part_element_world_aabb(board, elem="white_surface")
    if board_aabb is not None:
        mn, mx = board_aabb
        width = mx[0] - mn[0]
        height = mx[2] - mn[2]
        ctx.check(
            "tall rectangular writing surface",
            0.60 <= width <= 0.75 and 0.95 <= height <= 1.12 and height / width > 1.40,
            details=f"width={width:.3f}, height={height:.3f}",
        )
    else:
        ctx.fail("tall rectangular writing surface", "white_surface AABB unavailable")

    # ── Four-post rigid floor frame (TARGET-specific checks) ────────────
    # Each post must be a tall vertical tube spanning from near floor to
    # near the top of the frame.
    for i in range(4):
        post_aabb = ctx.part_element_world_aabb(board, elem=f"post_{i}")
        if post_aabb is not None:
            mn, mx = post_aabb
            h = mx[2] - mn[2]
            w = mx[0] - mn[0]
            d = mx[1] - mn[1]
            ctx.check(
                f"post_{i} is a full-height vertical tube",
                h > 1.5 and w < 0.06 and d < 0.06,
                details=f"height={h:.3f}, x_width={w:.3f}, y_depth={d:.3f}",
            )
        else:
            ctx.fail(f"post_{i} geometry", "AABB unavailable")

        # Rubber glide seated at each post base
        ctx.expect_contact(
            board,
            board,
            elem_a=f"post_{i}",
            elem_b=f"glide_{i}",
            contact_tol=0.005,
            name=f"glide_{i} seated at post_{i} base",
        )

    # Front posts (0, 1) must flank the board and overlap its writing height
    for i in (0, 1):
        ctx.expect_overlap(
            board,
            board,
            axes="z",
            elem_a=f"post_{i}",
            elem_b="white_surface",
            min_overlap=0.80,
            name=f"front post_{i} spans the board writing height",
        )

    # Lower crossbar (KEEP) must connect the two front posts
    ctx.expect_overlap(
        board,
        board,
        axes="x",
        elem_a="lower_crossbar",
        elem_b="post_0",
        min_overlap=0.01,
        name="lower crossbar reaches front left post",
    )
    ctx.expect_overlap(
        board,
        board,
        axes="x",
        elem_a="lower_crossbar",
        elem_b="post_1",
        min_overlap=0.01,
        name="lower crossbar reaches front right post",
    )

    # Rear posts (2, 3) are set behind the board plane for stability
    for i in (2, 3):
        post_aabb = ctx.part_element_world_aabb(board, elem=f"post_{i}")
        board_center_y = -0.018  # white_surface y center
        if post_aabb is not None:
            post_y_min = post_aabb[0][1]
            ctx.check(
                f"rear post_{i} is behind the board plane",
                post_y_min > board_center_y + 0.10,
                details=f"post_y_min={post_y_min:.3f}",
            )

    # ── Clamp knob rotation (non-fixed joint requirement) ───────────────
    clamp_joint = object_model.get_articulation("board_to_clamp_0")
    clamp_0 = object_model.get_part("clamp_0")
    clamp_rest = ctx.part_world_aabb(clamp_0)
    with ctx.pose({clamp_joint: math.pi / 2}):
        clamp_rotated = ctx.part_world_aabb(clamp_0)
    ctx.check(
        "clamp knob rotates on front post",
        clamp_rest is not None and clamp_rotated is not None,
        details=f"rest={clamp_rest}, rotated={clamp_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
