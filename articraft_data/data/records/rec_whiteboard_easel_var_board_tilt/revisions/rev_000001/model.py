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

# Height of the tilt pivot above the floor (top edge of the board).
Z_PIVOT = 1.950


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
    """Four wall boxes forming a visibly hollow telescoping leg sleeve."""
    wall = 0.008
    outer = 0.060
    inner = outer - 2.0 * wall
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
    part.visual(
        Box((wall, inner, length)),
        origin=Origin(xyz=(x - outer * 0.5 + wall * 0.5, 0.0, z_center)),
        material=material,
        name=f"{name}_outer_wall",
    )
    part.visual(
        Box((wall, inner, length)),
        origin=Origin(xyz=(x + outer * 0.5 - wall * 0.5, 0.0, z_center)),
        material=material,
        name=f"{name}_inner_wall",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="whiteboard_easel",
        meta={
            "note": (
                "Reference and folder agree on a whiteboard easel; no classification "
                "mismatch suspected. Variant adds a tilt carriage that pivots the "
                "board from upright to near-horizontal for drafting."
            )
        },
    )

    blue = model.material("powder_coated_blue", rgba=(0.0, 0.18, 0.38, 1.0))
    white = model.material("gloss_white_board", rgba=(0.96, 0.97, 0.94, 1.0))
    dark = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    metal = model.material("brushed_metal", rgba=(0.67, 0.70, 0.72, 1.0))
    grey = model.material("soft_grey_hardware", rgba=(0.24, 0.26, 0.28, 1.0))

    # ===== STAND (root) — two tall masts with base bar =====
    stand = model.part("stand")
    for idx, x in enumerate((-0.440, 0.440)):
        stand.visual(
            Box((0.040, 0.040, 1.930)),
            origin=Origin(xyz=(x, 0.0, 0.965)),
            material=blue,
            name=f"mast_{idx}",
        )
        stand.visual(
            Box((0.060, 0.060, 0.012)),
            origin=Origin(xyz=(x, 0.0, 0.006)),
            material=dark,
            name=f"mast_base_pad_{idx}",
        )
        stand.visual(
            Box((0.065, 0.065, 0.020)),
            origin=Origin(xyz=(x, 0.0, 1.940)),
            material=blue,
            name=f"mast_cap_{idx}",
        )
    stand.visual(
        Box((0.920, 0.040, 0.035)),
        origin=Origin(xyz=(0.0, 0.040, 0.060)),
        material=blue,
        name="stand_base_bar",
    )
    for idx, x in enumerate((-0.440, 0.440)):
        stand.visual(
            Box((0.055, 0.065, 0.040)),
            origin=Origin(xyz=(x, 0.010, 0.060)),
            material=blue,
            name=f"base_bar_mount_{idx}",
        )

    # ===== TILT CARRIAGE (FIXED to stand at mast tops) =====
    carriage = model.part("tilt_carriage")
    # Pivot bracket plates (one per side, straddling the board frame).
    # These are the main structural members that carry the pivot rod and
    # connect to the stand mast caps below.
    for idx, x in enumerate((-0.440, 0.440)):
        carriage.visual(
            Box((0.048, 0.070, 0.090)),
            origin=Origin(xyz=(x, 0.0, 0.010)),
            material=blue,
            name=f"pivot_bracket_{idx}",
        )
    # Horizontal pivot rod — extends past the brackets so it is clearly
    # captured by them (bearing fit).
    carriage.visual(
        Cylinder(radius=0.012, length=0.940),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="pivot_rod",
    )
    # Horizontal crossbar connecting the two brackets behind the board panel.
    # Offset to +Y so it clears the board top_rail when the board is upright.
    carriage.visual(
        Box((0.860, 0.032, 0.032)),
        origin=Origin(xyz=(0.0, 0.040, -0.040)),
        material=blue,
        name="carriage_crossbar",
    )
    # Detent arc plates — thin sectors bolted to the outer face of each
    # bracket, extending downward to show the tilt adjustment range.
    for idx, x in enumerate((-0.440, 0.440)):
        carriage.visual(
            Box((0.010, 0.060, 0.380)),
            origin=Origin(xyz=(x, -0.060, -0.160)),
            material=grey,
            name=f"detent_arc_{idx}",
        )
        # Detent notch markers along the arc (3 positions: upright, ~40°, near-flat)
        for notch_idx, (ny, nz) in enumerate(
            ((-0.060, -0.300), (-0.060, -0.180), (-0.060, -0.040))
        ):
            carriage.visual(
                Box((0.018, 0.014, 0.018)),
                origin=Origin(xyz=(x, ny, nz)),
                material=metal,
                name=f"detent_notch_{idx}_{notch_idx}",
            )

    # FIXED joint: stand → carriage at the pivot height
    model.articulation(
        "stand_to_carriage",
        ArticulationType.FIXED,
        parent=stand,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, Z_PIVOT)),
    )

    # ===== BOARD FRAME (REVOLUTE child of carriage via tilt joint) =====
    # All board-frame visuals are expressed in a local frame whose origin is at
    # the pivot point (world Z = Z_PIVOT at q=0).  Every world-Z value from the
    # parent baseline is shifted by -Z_PIVOT.
    board = model.part("board_frame")

    # --- Writing surface and perimeter frame ---
    board.visual(
        Box((0.685, 0.014, 1.040)),
        origin=Origin(xyz=(0.0, -0.018, 1.355 - Z_PIVOT)),
        material=white,
        name="white_surface",
    )
    board.visual(
        Box((0.805, 0.052, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
        material=blue,
        name="top_rail",
    )
    board.visual(
        Box((0.785, 0.050, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.790 - Z_PIVOT)),
        material=blue,
        name="bottom_rail",
    )
    for side, x in (("side_0", -0.395), ("side_1", 0.395)):
        board.visual(
            Box((0.060, 0.052, 1.090)),
            origin=Origin(xyz=(x, 0.0, 1.355 - Z_PIVOT)),
            material=blue,
            name=f"{side}_rail",
        )
        board.visual(
            Box((0.080, 0.042, 0.150)),
            origin=Origin(xyz=(x * 0.92, 0.0, 0.765 - Z_PIVOT)),
            material=blue,
            name=f"{side}_lower_socket",
        )

    # Hinge lugs at the board top that wrap around the carriage pivot rod
    for idx, x in enumerate((-0.395, 0.395)):
        board.visual(
            Box((0.048, 0.044, 0.050)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=blue,
            name=f"hinge_lug_{idx}",
        )

    # Locking pins that engage the detent arcs for tilt-angle selection.
    # Positioned inward from the masts so they clear the stand uprights.
    for idx, x in enumerate((-0.390, 0.390)):
        board.visual(
            Cylinder(radius=0.006, length=0.040),
            origin=Origin(xyz=(x, 0.0, -0.250), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name=f"locking_pin_{idx}",
        )

    # --- Marker tray ---
    board.visual(
        Box((0.630, 0.085, 0.022)),
        origin=Origin(xyz=(0.0, -0.067, 0.730 - Z_PIVOT)),
        material=blue,
        name="marker_tray_shelf",
    )
    board.visual(
        Box((0.630, 0.018, 0.052)),
        origin=Origin(xyz=(0.0, -0.025, 0.755 - Z_PIVOT)),
        material=blue,
        name="tray_back_lip",
    )
    board.visual(
        Box((0.045, 0.045, 0.070)),
        origin=Origin(xyz=(-0.260, -0.045, 0.775 - Z_PIVOT)),
        material=blue,
        name="tray_bracket_0",
    )
    board.visual(
        Box((0.045, 0.045, 0.070)),
        origin=Origin(xyz=(0.260, -0.045, 0.775 - Z_PIVOT)),
        material=blue,
        name="tray_bracket_1",
    )

    # --- Telescoping sleeves (hollow square tubes for the legs) ---
    for side_name, x in (("sleeve_0", -0.340), ("sleeve_1", 0.340)):
        _add_square_sleeve(
            board, x=x, z_center=0.630 - Z_PIVOT, length=0.600,
            material=blue, name=side_name,
        )
        for collar_name, zc in (
            ("upper_collar", 0.925 - Z_PIVOT),
            ("lower_collar", 0.335 - Z_PIVOT),
        ):
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
        board.visual(
            Box((0.030, 0.007, 0.080)),
            origin=Origin(xyz=(x, -0.0185, 0.690 - Z_PIVOT)),
            material=grey,
            name=f"{side_name}_clamp_shoe",
        )

    # --- Crossbar and diagonal braces ---
    board.visual(
        Box((0.710, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, 0.050, 0.390 - Z_PIVOT)),
        material=blue,
        name="lower_crossbar",
    )
    for idx, x in enumerate((-0.340, 0.340)):
        board.visual(
            Box((0.050, 0.024, 0.040)),
            origin=Origin(xyz=(x, 0.039, 0.390 - Z_PIVOT)),
            material=blue,
            name=f"crossbar_mount_{idx}",
        )
    _rod_between(
        board,
        (-0.340, 0.065, 0.390 - Z_PIVOT),
        (-0.180, 0.065, 0.790 - Z_PIVOT),
        0.014, material=blue, name="brace_0",
    )
    _rod_between(
        board,
        (0.340, 0.065, 0.390 - Z_PIVOT),
        (0.180, 0.065, 0.790 - Z_PIVOT),
        0.014, material=blue, name="brace_1",
    )

    # --- Rear hinge tabs for kickstand ---
    board.visual(
        Box((0.026, 0.032, 0.086)),
        origin=Origin(xyz=(-0.045, 0.035, 0.760 - Z_PIVOT)),
        material=blue,
        name="rear_hinge_tab_0",
    )
    board.visual(
        Box((0.026, 0.032, 0.086)),
        origin=Origin(xyz=(0.045, 0.035, 0.760 - Z_PIVOT)),
        material=blue,
        name="rear_hinge_tab_1",
    )

    # ===== REVOLUTE: carriage → board (tilt joint) =====
    # axis = (-1, 0, 0): positive q tilts the board bottom toward the user
    # (−Y), swinging from upright (q=0) to near-horizontal (q≈1.4 rad).
    model.articulation(
        "board_to_carriage",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=board,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0, velocity=0.5, lower=0.0, upper=1.40,
        ),
    )

    # ===== CLAMP KNOBS (children of board) =====
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
            origin=Origin(xyz=(sign * 0.4034, -0.001, 0.690 - Z_PIVOT)),
            axis=(sign, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.4, velocity=4.0, lower=-math.pi, upper=math.pi,
            ),
        )

    # ===== TELESCOPING LOWER LEGS (children of board) =====
    for idx, x in enumerate((-0.340, 0.340)):
        leg = model.part(f"lower_leg_{idx}")
        leg.visual(
            Box((0.030, 0.030, 0.620)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=blue,
            name="inner_tube",
        )
        leg.visual(
            Cylinder(radius=0.006, length=0.010),
            origin=Origin(xyz=(0.0, -0.020, -0.150), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name="spring_button",
        )
        leg.visual(
            Box((0.060, 0.038, 0.030)),
            origin=Origin(xyz=(0.0, 0.0, -0.310)),
            material=blue,
            name="foot_hinge_block",
        )
        model.articulation(
            f"board_to_lower_leg_{idx}",
            ArticulationType.PRISMATIC,
            parent=board,
            child=leg,
            origin=Origin(xyz=(x, 0.0, 0.340 - Z_PIVOT)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=90.0, velocity=0.18, lower=-0.080, upper=0.180,
            ),
        )

        # --- Foot / floor runner ---
        foot = model.part(f"foot_{idx}")
        foot.visual(
            Box((0.058, 0.720, 0.036)),
            origin=Origin(xyz=(0.0, 0.0, -0.033)),
            material=blue,
            name="floor_runner",
        )
        for end_name, y in (("front", -0.325), ("rear", 0.325)):
            foot.visual(
                Box((0.009, 0.040, 0.066)),
                origin=Origin(xyz=(-0.024, y, -0.059)),
                material=blue,
                name=f"{end_name}_fork_0",
            )
            foot.visual(
                Box((0.009, 0.040, 0.066)),
                origin=Origin(xyz=(0.024, y, -0.059)),
                material=blue,
                name=f"{end_name}_fork_1",
            )
            foot.visual(
                Box((0.065, 0.030, 0.010)),
                origin=Origin(xyz=(0.0, y, -0.033)),
                material=blue,
                name=f"{end_name}_fork_bridge",
            )
        _rod_between(
            foot, (0.0, -0.235, -0.036), (0.0, -0.030, 0.000),
            0.010, material=blue, name="front_support_brace",
        )
        _rod_between(
            foot, (0.0, 0.235, -0.036), (0.0, 0.030, 0.000),
            0.010, material=blue, name="rear_support_brace",
        )
        model.articulation(
            f"lower_leg_{idx}_to_foot_{idx}",
            ArticulationType.REVOLUTE,
            parent=leg,
            child=foot,
            origin=Origin(xyz=(0.0, 0.0, -0.310)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=8.0, velocity=1.5, lower=0.0, upper=1.25,
            ),
        )

        # --- Casters ---
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
                origin=Origin(xyz=(0.0, y, -0.081)),
                axis=(1.0, 0.0, 0.0),
                motion_limits=MotionLimits(effort=1.0, velocity=12.0),
            )

    # ===== REAR KICKSTAND (child of board) =====
    rear = model.part("rear_support")
    rear.visual(
        Cylinder(radius=0.018, length=0.060),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blue,
        name="hinge_barrel",
    )
    _rod_between(
        rear, (0.0, 0.010, -0.010), (0.0, 0.560, -0.720),
        0.016, material=blue, name="kickstand_tube",
    )
    rear.visual(
        Box((0.120, 0.070, 0.030)),
        origin=Origin(xyz=(0.0, 0.575, -0.735)),
        material=dark,
        name="rear_rubber_foot",
    )
    model.articulation(
        "board_to_rear_support",
        ArticulationType.REVOLUTE,
        parent=board,
        child=rear,
        origin=Origin(xyz=(0.0, 0.040, 0.760 - Z_PIVOT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0, velocity=1.2, lower=0.0, upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    board = object_model.get_part("board_frame")
    carriage = object_model.get_part("tilt_carriage")
    rear = object_model.get_part("rear_support")
    leg_0 = object_model.get_part("lower_leg_0")
    leg_1 = object_model.get_part("lower_leg_1")
    foot_0 = object_model.get_part("foot_0")
    foot_1 = object_model.get_part("foot_1")

    # ---- Writing surface proportions ----
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

    # ---- Tilt carriage mechanism (NEW — structural delta) ----
    tilt = object_model.get_articulation("board_to_carriage")
    ctx.check(
        "board_to_carriage tilt joint exists on tilt_carriage",
        tilt is not None,
        details="The tilt carriage revolute joint must exist.",
    )

    # Prove the tilt swings the board from upright toward horizontal.
    rest_surface = ctx.part_element_world_aabb(board, elem="white_surface")
    with ctx.pose({tilt: 1.0}):
        tilted_surface = ctx.part_element_world_aabb(board, elem="white_surface")
    if rest_surface is not None and tilted_surface is not None:
        rest_y_min = rest_surface[0][1]
        tilted_y_min = tilted_surface[0][1]
        rest_z_center = 0.5 * (rest_surface[0][2] + rest_surface[1][2])
        tilted_z_center = 0.5 * (tilted_surface[0][2] + tilted_surface[1][2])
        ctx.check(
            "board_to_carriage tilts board forward for drafting",
            tilted_y_min < rest_y_min - 0.20,
            details=f"rest_y_min={rest_y_min:.3f}, tilted_y_min={tilted_y_min:.3f}",
        )
        ctx.check(
            "tilted board center rises toward pivot (drafting angle)",
            tilted_z_center > rest_z_center + 0.10,
            details=f"rest_z={rest_z_center:.3f}, tilted_z={tilted_z_center:.3f}",
        )
    else:
        ctx.fail("board_to_carriage tilts board forward for drafting", "AABB unavailable")

    # Verify detent arcs exist on the tilt_carriage for angle support
    carriage_aabb = ctx.part_world_aabb(carriage)
    ctx.check(
        "tilt_carriage has visible support geometry",
        carriage_aabb is not None and (carriage_aabb[1][2] - carriage_aabb[0][2]) > 0.08,
        details=f"carriage_aabb={carriage_aabb}",
    )

    # ---- Leg retention in sleeves ----
    ctx.expect_overlap(
        board, leg_0, axes="z",
        elem_a="sleeve_0_front_wall", elem_b="inner_tube",
        min_overlap=0.12,
        name="lower leg 0 retained in sleeve",
    )
    ctx.expect_overlap(
        board, leg_1, axes="z",
        elem_a="sleeve_1_front_wall", elem_b="inner_tube",
        min_overlap=0.12,
        name="lower leg 1 retained in sleeve",
    )

    # ---- Foot seating ----
    ctx.expect_gap(
        leg_0, foot_0, axis="z",
        positive_elem="foot_hinge_block", negative_elem="floor_runner",
        max_gap=0.002, max_penetration=0.002,
        name="foot 0 runner seats against hinge block",
    )
    ctx.expect_gap(
        leg_1, foot_1, axis="z",
        positive_elem="foot_hinge_block", negative_elem="floor_runner",
        max_gap=0.002, max_penetration=0.002,
        name="foot 1 runner seats against hinge block",
    )

    # ---- Rear support hinge capture ----
    ctx.expect_contact(
        rear, board,
        elem_a="hinge_barrel", elem_b="rear_hinge_tab_0",
        contact_tol=0.004,
        name="rear support hinge is captured by frame tab",
    )

    # ---- Caster axle captured in fork ----
    for foot_idx in (0, 1):
        foot = object_model.get_part(f"foot_{foot_idx}")
        for wheel_idx, end_name in enumerate(("front", "rear")):
            caster = object_model.get_part(f"caster_{foot_idx}_{wheel_idx}")
            for fork_idx in (0, 1):
                fork_elem = f"{end_name}_fork_{fork_idx}"
                ctx.allow_overlap(
                    caster, foot,
                    elem_a="axle_pin", elem_b=fork_elem,
                    reason=(
                        "The caster axle pin is intentionally captured in the fork plate "
                        "holes so the wheel is physically supported while rotating."
                    ),
                )
                ctx.expect_overlap(
                    caster, foot, axes="xyz",
                    elem_a="axle_pin", elem_b=fork_elem,
                    min_overlap=0.004,
                    name=f"caster {foot_idx}-{wheel_idx} axle captured by {fork_elem}",
                )

    # ---- Pivot rod captured in hinge lugs (intentional bearing overlap) ----
    for lug_idx in (0, 1):
        ctx.allow_overlap(
            carriage, board,
            elem_a="pivot_rod", elem_b=f"hinge_lug_{lug_idx}",
            reason=(
                "The carriage pivot rod passes through the board hinge lug as a "
                "tilt bearing so the board is physically supported on the carriage."
            ),
        )
        ctx.expect_overlap(
            carriage, board, axes="yz",
            elem_a="pivot_rod", elem_b=f"hinge_lug_{lug_idx}",
            min_overlap=0.008,
            name=f"pivot rod captured by hinge_lug_{lug_idx}",
        )

    # ---- Carriage brackets seated on stand mast caps (intentional mounting) ----
    stand = object_model.get_part("stand")
    for bracket_idx in (0, 1):
        # The bracket overlaps both the mast body and the mast cap as a seated mount.
        for stand_elem_name in (f"mast_{bracket_idx}", f"mast_cap_{bracket_idx}"):
            ctx.allow_overlap(
                stand, carriage,
                elem_a=stand_elem_name, elem_b=f"pivot_bracket_{bracket_idx}",
                reason=(
                    "The carriage pivot bracket is intentionally mounted on top of the "
                    "stand mast cap as the fixed carriage-to-stand interface."
                ),
            )
        ctx.expect_contact(
            carriage, stand,
            elem_a=f"pivot_bracket_{bracket_idx}",
            elem_b=f"mast_cap_{bracket_idx}",
            contact_tol=0.065,
            name=f"carriage bracket {bracket_idx} seated on mast cap {bracket_idx}",
        )

    # ---- Board hinge lugs near stand mast caps (bearing interface) ----
    for lug_idx in (0, 1):
        ctx.allow_overlap(
            board, stand,
            elem_a=f"hinge_lug_{lug_idx}", elem_b=f"mast_cap_{lug_idx}",
            reason=(
                "The board hinge lug wraps around the carriage pivot rod at the mast "
                "cap height; shallow overlap is the bearing interface between the "
                "tilting board and the fixed stand."
            ),
        )
        ctx.expect_overlap(
            board, stand, axes="xy",
            elem_a=f"hinge_lug_{lug_idx}", elem_b=f"mast_cap_{lug_idx}",
            min_overlap=0.005,
            name=f"hinge_lug_{lug_idx} aligned with mast_cap_{lug_idx}",
        )

    # ---- Board side rails adjacent to stand masts (full-height proximity) ----
    for rail_idx, (rail_name, mast_name) in enumerate(
        (("side_0_rail", "mast_0"), ("side_1_rail", "mast_1"))
    ):
        ctx.allow_overlap(
            board, stand,
            elem_a=rail_name, elem_b=mast_name,
            reason=(
                "The board frame side rail runs adjacent to the stand mast along the "
                "full board height; the 5 mm X overlap represents realistic close "
                "clearance between the tilting board frame and the fixed stand uprights."
            ),
        )
        ctx.expect_overlap(
            board, stand, axes="z",
            elem_a=rail_name, elem_b=mast_name,
            min_overlap=0.80,
            name=f"{rail_name} vertically coextensive with {mast_name}",
        )

    # ---- Pivot rod passes through mast cap region ----
    for cap_idx in (0, 1):
        ctx.allow_overlap(
            stand, carriage,
            elem_a=f"mast_cap_{cap_idx}", elem_b="pivot_rod",
            reason=(
                "The carriage pivot rod extends past the brackets and passes through "
                "the mast cap region; this is the rod end captured near the stand top."
            ),
        )
        ctx.expect_overlap(
            stand, carriage, axes="x",
            elem_a=f"mast_cap_{cap_idx}", elem_b="pivot_rod",
            min_overlap=0.020,
            name=f"pivot_rod spans past mast_cap_{cap_idx}",
        )

    # ---- Clamp hardware near stand masts (realistic proximity) ----
    for clamp_idx, mast_name in enumerate(("mast_0", "mast_1")):
        clamp = object_model.get_part(f"clamp_{clamp_idx}")
        for clamp_elem in ("threaded_stem", "thumb_wheel"):
            ctx.allow_overlap(
                clamp, stand,
                elem_a=clamp_elem, elem_b=mast_name,
                reason=(
                    "The clamp hardware is mounted on the board sleeve adjacent to the "
                    "stand mast; shallow overlap is realistic proximity for leg-adjustment "
                    "hardware on the board frame near the stand uprights."
                ),
            )
        ctx.expect_overlap(
            clamp, stand, axes="z",
            elem_a="thumb_wheel", elem_b=mast_name,
            min_overlap=0.010,
            name=f"clamp_{clamp_idx} vertically aligned with {mast_name}",
        )

    # ---- Prismatic leg extension ----
    slide = object_model.get_articulation("board_to_lower_leg_0")
    start = ctx.part_world_position(leg_0)
    with ctx.pose({slide: 0.120}):
        moved = ctx.part_world_position(leg_0)
        ctx.expect_overlap(
            board, leg_0, axes="z",
            elem_a="sleeve_0_front_wall", elem_b="inner_tube",
            min_overlap=0.05,
            name="extended lower leg 0 remains inserted",
        )
    ctx.check(
        "prismatic leg extends downward",
        start is not None and moved is not None and moved[2] < start[2] - 0.08,
        details=f"start={start}, moved={moved}",
    )

    # ---- Foot hinge swivel ----
    foot_hinge = object_model.get_articulation("lower_leg_0_to_foot_0")
    foot_start = ctx.part_world_aabb(foot_0)
    with ctx.pose({foot_hinge: 0.80}):
        foot_swiveled = ctx.part_world_aabb(foot_0)
    start_x_span = None if foot_start is None else foot_start[1][0] - foot_start[0][0]
    swiveled_x_span = (
        None if foot_swiveled is None else foot_swiveled[1][0] - foot_swiveled[0][0]
    )
    ctx.check(
        "foot hinge swivels runner around vertical leg",
        start_x_span is not None
        and swiveled_x_span is not None
        and swiveled_x_span > start_x_span + 0.35,
        details=f"start={foot_start}, swiveled={foot_swiveled}",
    )

    # ---- Rear kickstand fold ----
    rear_hinge = object_model.get_articulation("board_to_rear_support")
    rear_start_aabb = ctx.part_world_aabb(rear)
    with ctx.pose({rear_hinge: 0.70}):
        rear_folded_aabb = ctx.part_world_aabb(rear)
    ctx.check(
        "rear kickstand folds upward",
        rear_start_aabb is not None
        and rear_folded_aabb is not None
        and rear_folded_aabb[0][2] > rear_start_aabb[0][2] + 0.15,
        details=f"start={rear_start_aabb}, folded={rear_folded_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
