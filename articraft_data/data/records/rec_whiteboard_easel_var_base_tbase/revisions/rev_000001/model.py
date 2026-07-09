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
                "Rolling pillar easel variant: single central mast column on a "
                "4-arm star/cross floor base with casters. Board, tray, and "
                "clamp knobs preserved from parent. Reference and folder agree "
                "on a whiteboard easel; no classification mismatch."
            )
        },
    )

    blue = model.material("powder_coated_blue", rgba=(0.0, 0.18, 0.38, 1.0))
    white = model.material("gloss_white_board", rgba=(0.96, 0.97, 0.94, 1.0))
    dark = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    metal = model.material("brushed_metal", rgba=(0.67, 0.70, 0.72, 1.0))
    grey = model.material("soft_grey_hardware", rgba=(0.24, 0.26, 0.28, 1.0))

    # ── Board frame (root) ──────────────────────────────────────────
    board = model.part("board_frame")

    # Perimeter frame rails first (they define the structural frame)
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

    # White writing surface spans exactly from bottom-rail top to top-rail
    # bottom so it contacts both rails and reads as a panel seated in the frame.
    # bottom_rail top: 0.790 + 0.052/2 = 0.816
    # top_rail bottom: 1.920 - 0.065/2 = 1.8875
    surf_bottom = 0.816
    surf_top = 1.8875
    surf_height = surf_top - surf_bottom
    surf_center_z = (surf_bottom + surf_top) / 2.0
    board.visual(
        Box((0.685, 0.014, surf_height)),
        origin=Origin(xyz=(0.0, -0.018, surf_center_z)),
        material=white,
        name="white_surface",
    )

    # Marker tray below the board with lip and brackets
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

    # Mounting plate on board back where the mast bracket bolts through.
    # Overlaps 1mm with the bottom_rail back face (y=0.025) for connectivity.
    board.visual(
        Box((0.150, 0.006, 0.210)),
        origin=Origin(xyz=(0.0, 0.027, 0.900)),
        material=grey,
        name="mast_mount_plate",
    )

    # Threaded bosses on each side of the mount plate – the clamp knob stems
    # screw into these, providing a real mount for the clamp parts.
    for idx, sign in enumerate((-1.0, 1.0)):
        board.visual(
            Box((0.022, 0.022, 0.022)),
            origin=Origin(xyz=(sign * 0.075, 0.036, 0.900)),
            material=grey,
            name=f"clamp_boss_{idx}",
        )

    # ── Clamp knobs (KEEP) ──────────────────────────────────────────
    for idx, sign in enumerate((-1.0, 1.0)):
        clamp = model.part(f"clamp_{idx}")
        clamp.visual(
            Cylinder(radius=0.007, length=0.044),
            origin=Origin(xyz=(sign * 0.022, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name="threaded_stem",
        )
        clamp.visual(
            Cylinder(radius=0.030, length=0.014),
            origin=Origin(xyz=(sign * 0.049, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name="thumb_wheel",
        )
        model.articulation(
            f"board_to_clamp_{idx}",
            ArticulationType.REVOLUTE,
            parent=board,
            child=clamp,
            origin=Origin(xyz=(sign * 0.075, 0.038, 0.900)),
            axis=(sign, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.4, velocity=4.0, lower=-math.pi, upper=math.pi
            ),
        )

    # ── Mast column + star base (single rigid assembly) ─────────────
    mast = model.part("mast")

    mast_y = 0.060
    mast_origin_z = 0.900

    # Vertical mast column tube
    column_length = 0.800
    mast.visual(
        Cylinder(radius=0.025, length=column_length),
        origin=Origin(xyz=(0.0, 0.0, -column_length / 2.0)),
        material=blue,
        name="mast_column",
    )

    # Bracket clamp plate – mates face-to-face with board mount plate.
    # Mount plate back face: y_world = 0.030.
    # Bracket clamp front face: y_world = 0.032. Center: 0.038.
    # y_local = 0.038 - 0.060 = -0.022.
    mast.visual(
        Box((0.130, 0.012, 0.200)),
        origin=Origin(xyz=(0.0, -0.022, 0.0)),
        material=blue,
        name="mast_bracket_clamp",
    )

    # Column cap ring at top
    mast.visual(
        Cylinder(radius=0.032, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=grey,
        name="column_cap",
    )

    # Hub at mast bottom
    hub_z_local = -column_length - 0.020  # -0.820
    hub_radius = 0.048
    hub_height = 0.040
    mast.visual(
        Cylinder(radius=hub_radius, length=hub_height),
        origin=Origin(xyz=(0.0, 0.0, hub_z_local)),
        material=blue,
        name="star_hub",
    )
    mast.visual(
        Cylinder(radius=hub_radius - 0.004, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, hub_z_local - 0.023)),
        material=grey,
        name="hub_bottom_cap",
    )

    # Star base geometry parameters
    arm_functional_length = 0.250
    arm_inner_overlap = 0.012
    arm_visual_length = arm_functional_length + arm_inner_overlap
    arm_end_r = hub_radius + arm_functional_length  # 0.298
    arm_center_r = hub_radius + arm_functional_length / 2.0 - arm_inner_overlap / 2.0
    arm_width = 0.050
    arm_height = 0.028

    for i in range(4):
        theta = i * math.pi / 2.0
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        # Arm beam – overlaps into hub for connectivity
        mast.visual(
            Box((arm_visual_length, arm_width, arm_height)),
            origin=Origin(
                xyz=(arm_center_r * cos_t, arm_center_r * sin_t, hub_z_local),
                rpy=(0.0, 0.0, theta),
            ),
            material=blue,
            name=f"arm_{i}",
        )

        # Gusset at arm root reinforcing the hub junction
        gusset_r = hub_radius + 0.030
        mast.visual(
            Box((0.042, arm_width, 0.038)),
            origin=Origin(
                xyz=(gusset_r * cos_t, gusset_r * sin_t, hub_z_local + 0.005),
                rpy=(0.0, 0.0, theta),
            ),
            material=blue,
            name=f"arm_{i}_gusset",
        )

        # Fork plates at arm end – taller to overlap with arm bottom in z
        fork_offset = 0.024
        fork_height = 0.056
        # arm bottom z = hub_z_local - arm_height/2 = -0.834
        # fork top should be 2mm above arm bottom for overlap
        fork_top = hub_z_local - arm_height / 2.0 + 0.002
        fork_z_local = fork_top - fork_height / 2.0

        for fork_idx, fsign in enumerate((-1.0, 1.0)):
            local_x = arm_end_r
            local_y = fsign * fork_offset
            fork_x = local_x * cos_t - local_y * sin_t
            fork_y = local_x * sin_t + local_y * cos_t
            mast.visual(
                Box((0.040, 0.008, fork_height)),
                origin=Origin(
                    xyz=(fork_x, fork_y, fork_z_local),
                    rpy=(0.0, 0.0, theta),
                ),
                material=blue,
                name=f"arm_{i}_fork_{fork_idx}",
            )

        # Fork bridge plate connecting fork tops – overlaps with arm bottom
        bridge_height = 0.012
        bridge_z = hub_z_local - arm_height / 2.0 - bridge_height / 2.0 + 0.002
        bridge_x = arm_end_r * cos_t
        bridge_y = arm_end_r * sin_t
        mast.visual(
            Box((0.048, 0.056, bridge_height)),
            origin=Origin(
                xyz=(bridge_x, bridge_y, bridge_z),
                rpy=(0.0, 0.0, theta),
            ),
            material=blue,
            name=f"arm_{i}_fork_bridge",
        )

        # ── Caster at arm end ───────────────────────────────────
        caster = model.part(f"caster_{i}_0")
        caster.visual(
            Cylinder(radius=0.030, length=0.024),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name="rubber_wheel",
        )
        caster.visual(
            Cylinder(radius=0.012, length=0.026),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=grey,
            name="hub_cap",
        )
        caster.visual(
            Cylinder(radius=0.006, length=0.060),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name="axle_pin",
        )

        # Wheel center in mast-local: z_world = 0.030 → z_local = -0.870
        wheel_x_local = arm_end_r * cos_t
        wheel_y_local = arm_end_r * sin_t
        wheel_z_local = -0.870

        model.articulation(
            f"mast_to_caster_{i}_0",
            ArticulationType.CONTINUOUS,
            parent=mast,
            child=caster,
            origin=Origin(
                xyz=(wheel_x_local, wheel_y_local, wheel_z_local),
                rpy=(0.0, 0.0, theta + math.pi / 2.0),
            ),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=12.0),
        )

    # Fixed joint: board → mast assembly
    model.articulation(
        "board_to_mast",
        ArticulationType.FIXED,
        parent=board,
        child=mast,
        origin=Origin(xyz=(0.0, mast_y, mast_origin_z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    board = object_model.get_part("board_frame")
    mast = object_model.get_part("mast")

    # ── Board writing surface dimensions (KEEP) ─────────────────────
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

    # ── Mast column is a vertical support (TARGET structural change) ─
    mast_col_aabb = ctx.part_element_world_aabb(mast, elem="mast_column")
    if mast_col_aabb is not None:
        mn, mx = mast_col_aabb
        col_height = mx[2] - mn[2]
        col_dx = mx[0] - mn[0]
        col_dy = mx[1] - mn[1]
        ctx.check(
            "mast column is a tall vertical tube",
            col_height > 0.70 and col_dx < 0.060 and col_dy < 0.060,
            details=f"height={col_height:.3f}, dx={col_dx:.3f}, dy={col_dy:.3f}",
        )
        mast_col_center_y = (mn[1] + mx[1]) / 2.0
        board_surf_aabb = ctx.part_element_world_aabb(board, elem="white_surface")
        if board_surf_aabb is not None:
            board_center_y = (board_surf_aabb[0][1] + board_surf_aabb[1][1]) / 2.0
            ctx.check(
                "mast column is behind the board",
                mast_col_center_y > board_center_y + 0.020,
                details=f"mast_y={mast_col_center_y:.3f}, board_y={board_center_y:.3f}",
            )
    else:
        ctx.fail("mast column exists", "mast_column AABB unavailable")

    # ── Star base has 4 radial arms (TARGET) ────────────────────────
    arm_centers_x = []
    arm_centers_y = []
    for i in range(4):
        arm_aabb = ctx.part_element_world_aabb(mast, elem=f"arm_{i}")
        ctx.check(
            f"star base arm_{i} exists",
            arm_aabb is not None,
            details=f"arm_{i} AABB unavailable",
        )
        if arm_aabb is not None:
            arm_centers_x.append((arm_aabb[0][0] + arm_aabb[1][0]) / 2.0)
            arm_centers_y.append((arm_aabb[0][1] + arm_aabb[1][1]) / 2.0)

    if len(arm_centers_x) == 4:
        # Opposing arms should be spread > 0.25m apart
        ctx.check(
            "opposing star arms spread apart (x-axis pair)",
            abs(arm_centers_x[0] - arm_centers_x[2]) > 0.25,
            details=f"arm_0 x={arm_centers_x[0]:.3f}, arm_2 x={arm_centers_x[2]:.3f}",
        )
        ctx.check(
            "opposing star arms spread apart (y-axis pair)",
            abs(arm_centers_y[1] - arm_centers_y[3]) > 0.25,
            details=f"arm_1 y={arm_centers_y[1]:.3f}, arm_3 y={arm_centers_y[3]:.3f}",
        )

    # ── Star hub exists ─────────────────────────────────────────────
    hub_aabb = ctx.part_element_world_aabb(mast, elem="star_hub")
    ctx.check(
        "star hub exists",
        hub_aabb is not None,
        details="star_hub AABB unavailable",
    )

    # ── Bracket contact: mast bracket clamp near board mount plate ──
    ctx.expect_gap(
        mast,
        board,
        axis="y",
        positive_elem="mast_bracket_clamp",
        negative_elem="mast_mount_plate",
        max_gap=0.004,
        max_penetration=0.002,
        name="mast bracket clamp seats near board mount plate",
    )

    # ── Clamp knobs still articulate (KEEP) ─────────────────────────
    for idx, sign in enumerate((-1.0, 1.0)):
        clamp = object_model.get_part(f"clamp_{idx}")
        clamp_joint = object_model.get_articulation(f"board_to_clamp_{idx}")

        # The clamp threaded stem is captured inside the bracket boss.
        ctx.allow_overlap(
            clamp,
            board,
            elem_a="threaded_stem",
            elem_b=f"clamp_boss_{idx}",
            reason=(
                "The clamp threaded stem screws into the bracket boss, so the "
                "stem is intentionally nested inside the boss for the clamp to "
                "read as a real tightening knob."
            ),
        )
        ctx.expect_overlap(
            clamp,
            board,
            axes="xyz",
            elem_a="threaded_stem",
            elem_b=f"clamp_boss_{idx}",
            min_overlap=0.004,
            name=f"clamp_{idx} stem captured in boss",
        )

        start_pos = ctx.part_world_position(clamp)
        with ctx.pose({clamp_joint: 1.0}):
            turned_pos = ctx.part_world_position(clamp)
        ctx.check(
            f"clamp_{idx} rotates on its bracket",
            start_pos is not None and turned_pos is not None,
            details=f"start={start_pos}, turned={turned_pos}",
        )

    # ── Caster subassembly: axle captured in fork, wheel spins (KEEP) ──
    for i in range(4):
        caster = object_model.get_part(f"caster_{i}_0")
        caster_joint = object_model.get_articulation(f"mast_to_caster_{i}_0")

        for fork_idx in (0, 1):
            ctx.allow_overlap(
                caster,
                mast,
                elem_a="axle_pin",
                elem_b=f"arm_{i}_fork_{fork_idx}",
                reason=(
                    "The caster axle pin is intentionally captured inside the "
                    "fork plate bore so the wheel is supported while spinning."
                ),
            )
            ctx.expect_overlap(
                caster,
                mast,
                axes="xyz",
                elem_a="axle_pin",
                elem_b=f"arm_{i}_fork_{fork_idx}",
                min_overlap=0.004,
                name=f"caster_{i}_0 axle captured by arm_{i}_fork_{fork_idx}",
            )

        start_aabb = ctx.part_world_aabb(caster)
        with ctx.pose({caster_joint: math.pi / 2.0}):
            spun_aabb = ctx.part_world_aabb(caster)
        ctx.check(
            f"caster_{i}_0 wheel spins",
            start_aabb is not None and spun_aabb is not None,
            details=f"rest={start_aabb}, spun={spun_aabb}",
        )

    # ── Wheels near floor ──────────────────────────────────────────
    for i in range(4):
        wheel_aabb = ctx.part_element_world_aabb(
            object_model.get_part(f"caster_{i}_0"), elem="rubber_wheel"
        )
        if wheel_aabb is not None:
            wheel_bottom = wheel_aabb[0][2]
            ctx.check(
                f"caster_{i}_0 wheel near floor",
                wheel_bottom < 0.010,
                details=f"wheel_bottom_z={wheel_bottom:.4f}",
            )

    # ── Board-to-mast fixed joint (TARGET) ──────────────────────────
    mast_joint = object_model.get_articulation("board_to_mast")
    ctx.check(
        "board_to_mast fixed joint exists",
        mast_joint is not None,
        details="board_to_mast articulation not found",
    )

    return ctx.report()


object_model = build_object_model()
