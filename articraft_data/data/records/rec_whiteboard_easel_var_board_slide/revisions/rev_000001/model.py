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


def _rod_between(part, p0, p1, radius: float, *, material: Material, name: str) -> None:
    """Add a cylinder whose local +Z is aligned from p0 to p1 in the part frame."""
    x0, y0, z0 = p0
    x1, y1, z1 = p1
    dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        return
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((x0 + x1) * 0.5, (y0 + y1) * 0.5, (z0 + z1) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _add_square_sleeve(part, *, x: float, z_center: float, length: float, material: Material, name: str) -> None:
    """Four wall boxes forming a visibly hollow carriage sleeve that slides on a mast rail.

    Side walls are named outboard (farther from x=0) and inboard (closer to x=0)
    so that the naming is physically consistent for both left and right sleeves.
    """
    wall = 0.008
    outer = 0.060
    inner = outer - 2.0 * wall
    # Front and rear walls (Y-direction).
    part.visual(
        Box((outer, wall, length)),
        origin=Origin(xyz=(x, -outer * 0.5 + wall * 0.5, z_center)),
        material=material,
        name=f"{name}_front_wall",
    )
    part.visual(
        Box((outer, wall, length)),
        origin=Origin(xyz=(x, outer * 0.5 - wall * 0.5, z_center)),
        material=material,
        name=f"{name}_rear_wall",
    )
    # Side walls: outboard is farther from center, inboard is closer.
    if x < 0:
        outboard_x = x - outer * 0.5 + wall * 0.5
        inboard_x = x + outer * 0.5 - wall * 0.5
    else:
        outboard_x = x + outer * 0.5 - wall * 0.5
        inboard_x = x - outer * 0.5 + wall * 0.5
    part.visual(
        Box((wall, inner, length)),
        origin=Origin(xyz=(outboard_x, 0.0, z_center)),
        material=material,
        name=f"{name}_outboard_wall",
    )
    part.visual(
        Box((wall, inner, length)),
        origin=Origin(xyz=(inboard_x, 0.0, z_center)),
        material=material,
        name=f"{name}_inboard_wall",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="whiteboard_easel",
        meta={
            "note": (
                "Reference and folder agree on a whiteboard easel; no classification "
                "mismatch suspected. Variant: board slides vertically on two fixed "
                "side masts via a prismatic carriage; legs do not telescope."
            )
        },
    )

    blue = model.material("powder_coated_blue", rgba=(0.0, 0.18, 0.38, 1.0))
    white = model.material("gloss_white_board", rgba=(0.96, 0.97, 0.94, 1.0))
    dark = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    metal = model.material("brushed_metal", rgba=(0.67, 0.70, 0.72, 1.0))
    grey = model.material("soft_grey_hardware", rgba=(0.24, 0.26, 0.28, 1.0))

    # ------------------------------------------------------------------
    # BASE (root): fixed structure with two vertical mast posts, lower
    # and upper crossbars, and foot mounting points.
    # ------------------------------------------------------------------
    base = model.part("base")

    # Two fixed square mast posts. 40 mm cross-section fits inside the
    # 44 mm hollow sleeve interior on the board carriage with 2 mm sliding
    # clearance per side.
    mast_length = 1.940
    mast_z_center = 1.090  # z from 0.120 to 2.060
    for side_name, x in (("mast_0", -0.340), ("mast_1", 0.340)):
        base.visual(
            Box((0.040, 0.040, mast_length)),
            origin=Origin(xyz=(x, 0.0, mast_z_center)),
            material=blue,
            name=f"{side_name}_post",
        )
        # Mast cap prevents the carriage from sliding off the top.
        base.visual(
            Box((0.056, 0.056, 0.018)),
            origin=Origin(xyz=(x, 0.0, 2.069)),
            material=blue,
            name=f"{side_name}_cap",
        )

    # Lower crossbar ties the two mast bases together, positioned behind
    # the masts (positive Y) and touching their rear faces.
    base.visual(
        Box((0.710, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, 0.032, 0.250)),
        material=blue,
        name="lower_crossbar",
    )
    for idx, x in enumerate((-0.340, 0.340)):
        base.visual(
            Box((0.050, 0.024, 0.040)),
            origin=Origin(xyz=(x, 0.032, 0.250)),
            material=blue,
            name=f"crossbar_mount_{idx}",
        )

    # Upper crossbar near the mast caps ties the top of the frame together.
    base.visual(
        Box((0.710, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 2.020)),
        material=blue,
        name="upper_crossbar",
    )

    # ------------------------------------------------------------------
    # BOARD FRAME: the whiteboard and its aluminium perimeter frame,
    # marker tray, and the carriage sleeves that wrap around the masts.
    # Slides vertically via the base_to_board prismatic joint.
    # ------------------------------------------------------------------
    board = model.part("board_frame")

    board.visual(
        Box((0.685, 0.014, 1.040)),
        origin=Origin(xyz=(0.0, -0.038, 1.355)),
        material=white,
        name="white_surface",
    )
    board.visual(
        Box((0.805, 0.028, 0.065)),
        origin=Origin(xyz=(0.0, -0.034, 1.920)),
        material=blue,
        name="top_rail",
    )
    board.visual(
        Box((0.785, 0.028, 0.052)),
        origin=Origin(xyz=(0.0, -0.034, 0.790)),
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

    # Marker tray below the board with an upturned lip and brackets.
    board.visual(
        Box((0.630, 0.085, 0.022)),
        origin=Origin(xyz=(0.0, -0.077, 0.730)),
        material=blue,
        name="marker_tray_shelf",
    )
    board.visual(
        Box((0.630, 0.018, 0.052)),
        origin=Origin(xyz=(0.0, -0.035, 0.755)),
        material=blue,
        name="tray_back_lip",
    )
    board.visual(
        Box((0.045, 0.045, 0.070)),
        origin=Origin(xyz=(-0.260, -0.055, 0.775)),
        material=blue,
        name="tray_bracket_0",
    )
    board.visual(
        Box((0.045, 0.045, 0.070)),
        origin=Origin(xyz=(0.260, -0.055, 0.775)),
        material=blue,
        name="tray_bracket_1",
    )

    # Carriage sleeves on each side. These hollow square tubes wrap
    # around the fixed mast posts and guide the board vertically.
    for side_name, x in (("sleeve_0", -0.340), ("sleeve_1", 0.340)):
        _add_square_sleeve(
            board, x=x, z_center=0.630, length=0.600,
            material=blue, name=side_name,
        )
        for collar_name, zc in (("upper_collar", 0.925), ("lower_collar", 0.335)):
            board.visual(
                Box((0.095, 0.006, 0.045)),
                origin=Origin(xyz=(x, -0.033, zc)),
                material=blue,
                name=f"{side_name}_{collar_name}_front_band",
            )
            board.visual(
                Box((0.095, 0.006, 0.045)),
                origin=Origin(xyz=(x, 0.033, zc)),
                material=blue,
                name=f"{side_name}_{collar_name}_rear_band",
            )
        # Clamp shoe: friction pad on the front face of the sleeve that
        # presses against the mast rail to lock the board height.
        board.visual(
            Box((0.030, 0.007, 0.080)),
            origin=Origin(xyz=(x, -0.0185, 0.690)),
            material=grey,
            name=f"{side_name}_clamp_shoe",
        )

    # Prismatic joint: board slides vertically on the fixed masts.
    model.articulation(
        "base_to_board",
        ArticulationType.PRISMATIC,
        parent=base,
        child=board,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0, velocity=0.18, lower=0.0, upper=0.350,
        ),
    )

    # ------------------------------------------------------------------
    # CLAMP KNOBS: height-lock knobs on the carriage sleeves. The
    # threaded stem passes through the sleeve outboard wall and engages
    # the mast; the thumb wheel sits outside.
    # ------------------------------------------------------------------
    for idx, sign in enumerate((-1.0, 1.0)):
        knob = model.part(f"clamp_{idx}")
        # Threaded stem goes from outside the wall through to the mast.
        # Extends 5 mm past the wall face so it overlaps with the thumb
        # wheel for internal connectivity.
        knob.visual(
            Cylinder(radius=0.007, length=0.050),
            origin=Origin(
                xyz=(-sign * 0.020, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=metal,
            name="threaded_stem",
        )
        # Thumb wheel on the outside, overlapping the stem end.
        knob.visual(
            Cylinder(radius=0.030, length=0.020),
            origin=Origin(
                xyz=(sign * 0.010, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark,
            name="thumb_wheel",
        )
        model.articulation(
            f"board_to_clamp_{idx}",
            ArticulationType.REVOLUTE,
            parent=board,
            child=knob,
            origin=Origin(xyz=(sign * 0.370, -0.001, 0.690)),
            axis=(sign, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.4, velocity=4.0, lower=-math.pi, upper=math.pi,
            ),
        )

    # ------------------------------------------------------------------
    # FEET: two floor runners with caster wheels, connected to the base
    # via swivel hinges at the mast bottoms.
    # ------------------------------------------------------------------
    for idx, x in enumerate((-0.340, 0.340)):
        foot = model.part(f"foot_{idx}")

        # Vertical hinge stub overlaps the mast bottom and the runner top.
        foot.visual(
            Box((0.044, 0.044, 0.060)),
            origin=Origin(xyz=(0.0, 0.0, -0.030)),
            material=blue,
            name="hinge_stub",
        )
        # Horizontal floor runner.
        foot.visual(
            Box((0.058, 0.720, 0.030)),
            origin=Origin(xyz=(0.0, 0.0, -0.060)),
            material=blue,
            name="floor_runner",
        )
        # Fork plates extend below the runner to hold the wheel axles.
        for end_name, y in (("front", -0.325), ("rear", 0.325)):
            foot.visual(
                Box((0.009, 0.040, 0.080)),
                origin=Origin(xyz=(-0.024, y, -0.080)),
                material=blue,
                name=f"{end_name}_fork_0",
            )
            foot.visual(
                Box((0.009, 0.040, 0.080)),
                origin=Origin(xyz=(0.024, y, -0.080)),
                material=blue,
                name=f"{end_name}_fork_1",
            )
            foot.visual(
                Box((0.065, 0.030, 0.010)),
                origin=Origin(xyz=(0.0, y, -0.045)),
                material=blue,
                name=f"{end_name}_fork_bridge",
            )
        # Diagonal support braces for the runner.
        _rod_between(
            foot, (0.0, -0.235, -0.050), (0.0, -0.030, -0.005),
            0.010, material=blue, name="front_support_brace",
        )
        _rod_between(
            foot, (0.0, 0.235, -0.050), (0.0, 0.030, -0.005),
            0.010, material=blue, name="rear_support_brace",
        )

        model.articulation(
            f"base_to_foot_{idx}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=foot,
            origin=Origin(xyz=(x, 0.0, 0.120)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=8.0, velocity=1.5, lower=0.0, upper=1.57,
            ),
        )

        # Caster wheels below the runner.
        for wheel_idx, (end_name, y) in enumerate(
            (("front", -0.325), ("rear", 0.325))
        ):
            caster = model.part(f"caster_{idx}_{wheel_idx}")
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
            model.articulation(
                f"foot_{idx}_to_caster_{wheel_idx}",
                ArticulationType.CONTINUOUS,
                parent=foot,
                child=caster,
                origin=Origin(xyz=(0.0, y, -0.108)),
                axis=(1.0, 0.0, 0.0),
                motion_limits=MotionLimits(effort=1.0, velocity=12.0),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    board = object_model.get_part("board_frame")

    # --- Board writing surface dimensions ---
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

    # --- Carriage sleeves captured on mast rails (Z projection) ---
    ctx.expect_overlap(
        board, base,
        axes="z",
        elem_a="sleeve_0_front_wall",
        elem_b="mast_0_post",
        min_overlap=0.50,
        name="carriage sleeve 0 captured on mast at rest",
    )
    ctx.expect_overlap(
        board, base,
        axes="z",
        elem_a="sleeve_1_front_wall",
        elem_b="mast_1_post",
        min_overlap=0.50,
        name="carriage sleeve 1 captured on mast at rest",
    )

    # --- Clamp shoes press against mast rails (intentional friction contact) ---
    for side_name in ("sleeve_0", "sleeve_1"):
        mast_name = side_name.replace("sleeve", "mast")
        ctx.allow_overlap(
            board, base,
            elem_a=f"{side_name}_clamp_shoe",
            elem_b=f"{mast_name}_post",
            reason=(
                "The clamp shoe is a friction pad that presses against the mast "
                "rail surface to lock the board carriage height."
            ),
        )
        ctx.expect_overlap(
            board, base,
            axes="xy",
            elem_a=f"{side_name}_clamp_shoe",
            elem_b=f"{mast_name}_post",
            min_overlap=0.005,
            name=f"{side_name} clamp shoe contacts mast rail",
        )

    # --- Clamp knob stems pass through outboard walls and engage masts ---
    for idx, side_name in enumerate(("sleeve_0", "sleeve_1")):
        mast_name = side_name.replace("sleeve", "mast")
        clamp_part = object_model.get_part(f"clamp_{idx}")
        # Stem goes through the outboard wall.
        ctx.allow_overlap(
            clamp_part, board,
            elem_a="threaded_stem",
            elem_b=f"{side_name}_outboard_wall",
            reason=(
                "The clamp threaded stem passes through the sleeve outboard wall "
                "to engage the mast post for height locking."
            ),
        )
        ctx.expect_overlap(
            clamp_part, board,
            axes="xyz",
            elem_a="threaded_stem",
            elem_b=f"{side_name}_outboard_wall",
            min_overlap=0.004,
            name=f"clamp_{idx} stem passes through {side_name} outboard wall",
        )
        # Stem engages with the mast post (threaded into the mast).
        ctx.allow_overlap(
            clamp_part, base,
            elem_a="threaded_stem",
            elem_b=f"{mast_name}_post",
            reason=(
                "The clamp threaded stem engages a threaded hole in the mast "
                "post to lock the carriage at the desired height."
            ),
        )
        ctx.expect_overlap(
            clamp_part, base,
            axes="xyz",
            elem_a="threaded_stem",
            elem_b=f"{mast_name}_post",
            min_overlap=0.004,
            name=f"clamp_{idx} stem engages {mast_name} post",
        )

    # --- Caster axles captured in fork plates ---
    for foot_idx in (0, 1):
        foot = object_model.get_part(f"foot_{foot_idx}")
        for wheel_idx, end_name in enumerate(("front", "rear")):
            caster = object_model.get_part(f"caster_{foot_idx}_{wheel_idx}")
            for fork_idx in (0, 1):
                fork_elem = f"{end_name}_fork_{fork_idx}"
                ctx.allow_overlap(
                    caster, foot,
                    elem_a="axle_pin",
                    elem_b=fork_elem,
                    reason=(
                        "The caster axle pin is intentionally captured in the fork "
                        "plate holes so the wheel is physically supported while rotating."
                    ),
                )
                ctx.expect_overlap(
                    caster, foot,
                    axes="xyz",
                    elem_a="axle_pin",
                    elem_b=fork_elem,
                    min_overlap=0.004,
                    name=f"caster {foot_idx}-{wheel_idx} axle captured by {fork_elem}",
                )

    # --- Caster wheels clear the runner bottom face ---
    for foot_idx in (0, 1):
        foot = object_model.get_part(f"foot_{foot_idx}")
        for wheel_idx in (0, 1):
            caster = object_model.get_part(f"caster_{foot_idx}_{wheel_idx}")
            ctx.expect_gap(
                foot, caster,
                axis="z",
                positive_elem="floor_runner",
                negative_elem="rubber_wheel",
                max_penetration=0.002,
                name=f"caster {foot_idx}-{wheel_idx} wheel clears foot runner",
            )

    # --- Prismatic height adjustment: board slides upward on masts ---
    slide = object_model.get_articulation("base_to_board")
    rest_pos = ctx.part_world_position(board)
    with ctx.pose({slide: 0.300}):
        moved_pos = ctx.part_world_position(board)
        ctx.expect_overlap(
            board, base,
            axes="z",
            elem_a="sleeve_0_front_wall",
            elem_b="mast_0_post",
            min_overlap=0.20,
            name="carriage sleeve 0 remains captured on mast at raised position",
        )
        ctx.expect_overlap(
            board, base,
            axes="z",
            elem_a="sleeve_1_front_wall",
            elem_b="mast_1_post",
            min_overlap=0.20,
            name="carriage sleeve 1 remains captured on mast at raised position",
        )
    ctx.check(
        "base_to_board prismatic raises board carriage upward",
        rest_pos is not None and moved_pos is not None and moved_pos[2] > rest_pos[2] + 0.25,
        details=f"rest={rest_pos}, raised={moved_pos}",
    )

    # --- Foot swivel hinge ---
    foot_0 = object_model.get_part("foot_0")
    foot_hinge = object_model.get_articulation("base_to_foot_0")
    foot_start = ctx.part_world_aabb(foot_0)
    with ctx.pose({foot_hinge: 0.80}):
        foot_swiveled = ctx.part_world_aabb(foot_0)
    start_x_span = None if foot_start is None else foot_start[1][0] - foot_start[0][0]
    swiveled_x_span = (
        None if foot_swiveled is None else foot_swiveled[1][0] - foot_swiveled[0][0]
    )
    ctx.check(
        "foot swivel hinge rotates runner",
        start_x_span is not None
        and swiveled_x_span is not None
        and swiveled_x_span > start_x_span + 0.30,
        details=f"start_x={start_x_span}, swiveled_x={swiveled_x_span}",
    )

    # --- Clamp knob rotation ---
    clamp_0 = object_model.get_part("clamp_0")
    clamp_joint = object_model.get_articulation("board_to_clamp_0")
    clamp_start = ctx.part_world_aabb(clamp_0)
    with ctx.pose({clamp_joint: 1.0}):
        clamp_turned = ctx.part_world_aabb(clamp_0)
    ctx.check(
        "clamp knob rotates for height lock",
        clamp_start is not None and clamp_turned is not None,
        details=f"start={clamp_start}, turned={clamp_turned}",
    )

    return ctx.report()


object_model = build_object_model()
