from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    ExtrudeGeometry,
    Mimic,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


TABLE_X = 0.80
TABLE_Y = 0.54
TOP_Z = 0.51
SLAT_THICKNESS = 0.024
SLAT_COUNT = 9
SLAT_GAP = 0.006
SLAT_W = (TABLE_X - (SLAT_COUNT - 1) * SLAT_GAP) / SLAT_COUNT
LEG_TOP_Z = TOP_Z - 0.055
LEG_Y = TABLE_Y / 2.0 - 0.060

# Scissor base parameters
SCISSOR_MOUNT_X = 0.12
SCISSOR_ARM_LENGTH = 0.50
SCISSOR_DEPLOY_ANGLE = 0.50  # radians from vertical (~28.6°)
SCISSOR_FOLD_ANGLE = 1.05  # radians (~60°) upper limit
SCISSOR_TUBE_RADIUS = 0.012
ARM_Y_OFFSET = 0.016  # Y bypass gap between crossing arms


def _slat_mesh():
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(SLAT_W, TABLE_Y, radius=0.010, corner_segments=8),
            SLAT_THICKNESS,
            center=True,
        ),
        "rounded_aluminum_slat",
    )


def _capsule_x_mesh(name: str, radius: float, mid_length: float):
    geom = CapsuleGeometry(radius=radius, length=mid_length, radial_segments=18)
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _scissor_arm_mesh(name: str):
    """Vertical tube mesh for one scissor arm, along -Z from origin.

    The tube is aligned with the child frame Z axis so the SDK mesh AABB
    stays tight when the articulation rotates the arm.
    """
    return mesh_from_geometry(
        tube_from_spline_points(
            [(0.0, 0.0, 0.0), (0.0, 0.0, -SCISSOR_ARM_LENGTH)],
            radius=SCISSOR_TUBE_RADIUS,
            samples_per_segment=8,
            radial_segments=18,
            cap_ends=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="compact_folding_camp_table",
        meta={
            "run_notes": (
                "Primary image/category agree: a compact folding camp table. "
                "The carry bag in the reference is omitted so the single asset "
                "focuses on the articulated table. "
                "Variant: scissor/accordion collapsing base replaces four "
                "independent hinged legs. Crossed linkage arms fold flat under "
                "the slatted roll-top deck for compact carry."
            )
        },
    )

    mottled_aluminum = model.material("mottled_aluminum", rgba=(0.54, 0.55, 0.54, 1.0))
    dark_aluminum = model.material("dark_gap_shadow", rgba=(0.025, 0.026, 0.026, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.015, 0.014, 0.013, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.02, 0.019, 0.018, 1.0))
    screw_metal = model.material("screw_heads", rgba=(0.83, 0.82, 0.78, 1.0))

    slat_mesh = _slat_mesh()
    end_cap_mesh = _capsule_x_mesh(
        "rounded_slat_end_cap", radius=0.012, mid_length=max(0.001, SLAT_W - 0.024)
    )

    tabletop = model.part("tabletop")

    # Segmented aluminum slats
    x0 = -TABLE_X / 2.0 + SLAT_W / 2.0
    for i in range(SLAT_COUNT):
        x = x0 + i * (SLAT_W + SLAT_GAP)
        tabletop.visual(
            slat_mesh,
            origin=Origin(xyz=(x, 0.0, TOP_Z)),
            material=mottled_aluminum,
            name=f"slat_{i}",
        )
        for y, suffix in [(-1, "front"), (1, "rear")]:
            tabletop.visual(
                end_cap_mesh,
                origin=Origin(xyz=(x, y * (TABLE_Y / 2.0 + 0.012), TOP_Z - 0.003)),
                material=black_plastic,
                name=f"{suffix}_cap_{i}",
            )

    # Dark gap fillers between slats
    for i in range(1, SLAT_COUNT):
        x = -TABLE_X / 2.0 + i * SLAT_W + (i - 0.5) * SLAT_GAP
        tabletop.visual(
            Box((SLAT_GAP * 0.65, TABLE_Y + 0.012, 0.026)),
            origin=Origin(xyz=(x, 0.0, TOP_Z + 0.0005)),
            material=dark_aluminum,
            name=f"slat_gap_{i}",
        )

    # Black perimeter rails
    for y, name in [
        (-TABLE_Y / 2.0 - 0.016, "front_rail"),
        (TABLE_Y / 2.0 + 0.016, "rear_rail"),
    ]:
        tabletop.visual(
            Box((TABLE_X + 0.030, 0.030, 0.034)),
            origin=Origin(xyz=(0.0, y, TOP_Z - 0.012)),
            material=black_plastic,
            name=name,
        )
    for x, name in [
        (-TABLE_X / 2.0 - 0.016, "side_rail_0"),
        (TABLE_X / 2.0 + 0.016, "side_rail_1"),
    ]:
        tabletop.visual(
            Box((0.030, TABLE_Y + 0.026, 0.034)),
            origin=Origin(xyz=(x, 0.0, TOP_Z - 0.012)),
            material=black_plastic,
            name=name,
        )

    # Underside crossrails
    for y in (-0.16, 0.16):
        tabletop.visual(
            Box((TABLE_X - 0.070, 0.022, 0.030)),
            origin=Origin(xyz=(0.0, y, TOP_Z - 0.018)),
            material=black_plastic,
            name=f"underside_crossrail_{'front' if y < 0 else 'rear'}",
        )

    # Rounded corner connectors
    for idx, (x, y) in enumerate([(-1, -1), (1, -1), (1, 1), (-1, 1)]):
        tabletop.visual(
            Sphere(radius=0.030),
            origin=Origin(
                xyz=(
                    x * (TABLE_X / 2.0 + 0.004),
                    y * (TABLE_Y / 2.0 + 0.004),
                    TOP_Z - 0.012,
                )
            ),
            material=black_plastic,
            name=f"corner_connector_{idx}",
        )

    # Scissor pivot mount sockets (reused hinge_socket_i naming from parent).
    # Front pair and rear pair of crossed arms mount at these four points.
    scissor_mounts = [
        (-SCISSOR_MOUNT_X, -LEG_Y),  # 0: front-left
        (SCISSOR_MOUNT_X, -LEG_Y),   # 1: front-right
        (SCISSOR_MOUNT_X, LEG_Y),    # 2: rear-right
        (-SCISSOR_MOUNT_X, LEG_Y),   # 3: rear-left
    ]
    for i, (x, y) in enumerate(scissor_mounts):
        tabletop.visual(
            Box((0.050, 0.048, 0.058)),
            origin=Origin(xyz=(x, y, LEG_TOP_Z + 0.010)),
            material=black_plastic,
            name=f"hinge_socket_{i}",
        )
        # Vertical support rib connecting socket to tabletop underside slats.
        # Prevents the socket from reading as a disconnected geometry island.
        tabletop.visual(
            Box((0.024, 0.024, 0.034)),
            origin=Origin(xyz=(x, y, LEG_TOP_Z + 0.037)),
            material=black_plastic,
            name=f"socket_rib_{i}",
        )

    # Front rail details (screws and oval inserts)
    for i, x in enumerate([x0 + k * (SLAT_W + SLAT_GAP) for k in range(SLAT_COUNT)]):
        if i % 2 == 0:
            tabletop.visual(
                Box((0.042, 0.006, 0.014)),
                origin=Origin(xyz=(x, -TABLE_Y / 2.0 - 0.033, TOP_Z - 0.006)),
                material=black_plastic,
                name=f"front_oval_insert_{i}",
            )
        for dx in (-0.020, 0.020):
            tabletop.visual(
                Cylinder(radius=0.006, length=0.003),
                origin=Origin(
                    xyz=(x + dx, -TABLE_Y / 2.0 - 0.031, TOP_Z - 0.003),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=screw_metal,
                name=f"front_screw_{i}_{0 if dx < 0 else 1}",
            )

    # ── Scissor base: crossed linkage arms ──
    # Four scissor arms form two X-shaped pairs (front and rear).
    # Each arm's tube is aligned along child-frame -Z; the articulation
    # origin rpy tilts it to the deployed angle. Positive q folds flat.
    arm_y_offsets = [-ARM_Y_OFFSET, ARM_Y_OFFSET, ARM_Y_OFFSET, -ARM_Y_OFFSET]

    arm_configs = [
        # (index, mount_x, mount_y, dir_x, mimic_source_or_None, axis_y, rpy_pitch)
        # rpy_pitch tilts child -Z to the deployed direction:
        #   dir_x=+1 → pitch = -SCISSOR_DEPLOY_ANGLE
        #   dir_x=-1 → pitch = +SCISSOR_DEPLOY_ANGLE
        (0, -SCISSOR_MOUNT_X, -LEG_Y, +1.0, None, -1.0, -SCISSOR_DEPLOY_ANGLE),
        (1, SCISSOR_MOUNT_X, -LEG_Y, -1.0, "scissor_arm_0", 1.0, SCISSOR_DEPLOY_ANGLE),
        (2, SCISSOR_MOUNT_X, LEG_Y, -1.0, "scissor_arm_0", 1.0, SCISSOR_DEPLOY_ANGLE),
        (3, -SCISSOR_MOUNT_X, LEG_Y, +1.0, "scissor_arm_0", -1.0, -SCISSOR_DEPLOY_ANGLE),
    ]

    half_length = SCISSOR_ARM_LENGTH / 2.0

    for idx, mx, my, dir_x, mimic_src, axis_y, rpy_pitch in arm_configs:
        arm = model.part(f"scissor_arm_{idx}")
        y_off = arm_y_offsets[idx]

        # Main structural tube member (vertical in child frame)
        arm.visual(
            _scissor_arm_mesh(f"arm_tube_{idx}"),
            origin=Origin(xyz=(0.0, y_off, 0.0)),
            material=mottled_aluminum,
            name="arm_tube",
        )

        # Top pivot hub (captured inside tabletop hinge socket)
        arm.visual(
            Cylinder(radius=0.015, length=0.040),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=screw_metal,
            name="pivot_hub",
        )

        # Center scissor pivot bushing at arm midpoint
        arm.visual(
            Cylinder(radius=0.014, length=0.020),
            origin=Origin(
                xyz=(0.0, y_off, -half_length),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=black_plastic,
            name="scissor_pivot",
        )

        # Rubber foot at the bottom end
        arm.visual(
            Cylinder(radius=0.020, length=0.028),
            origin=Origin(xyz=(0.0, y_off, -SCISSOR_ARM_LENGTH)),
            material=rubber,
            name="rubber_foot",
        )

        # Cross stretcher bar connecting front-to-rear at crossing height.
        # Only on arm_0 — extends from front midpoint to rear crossing.
        if idx == 0:
            arm.visual(
                Cylinder(radius=0.010, length=2.0 * LEG_Y),
                origin=Origin(
                    xyz=(0.0, y_off + LEG_Y, -half_length),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=mottled_aluminum,
                name="cross_stretcher",
            )

        # Articulation with rpy to set deployed angle
        if mimic_src is None:
            model.articulation(
                f"tabletop_to_scissor_arm_{idx}",
                ArticulationType.REVOLUTE,
                parent=tabletop,
                child=arm,
                origin=Origin(
                    xyz=(mx, my, LEG_TOP_Z),
                    rpy=(0.0, rpy_pitch, 0.0),
                ),
                axis=(0.0, axis_y, 0.0),
                motion_limits=MotionLimits(
                    effort=18.0,
                    velocity=2.0,
                    lower=0.0,
                    upper=SCISSOR_FOLD_ANGLE,
                ),
            )
        else:
            model.articulation(
                f"tabletop_to_scissor_arm_{idx}",
                ArticulationType.REVOLUTE,
                parent=tabletop,
                child=arm,
                origin=Origin(
                    xyz=(mx, my, LEG_TOP_Z),
                    rpy=(0.0, rpy_pitch, 0.0),
                ),
                axis=(0.0, axis_y, 0.0),
                motion_limits=MotionLimits(
                    effort=18.0,
                    velocity=2.0,
                    lower=0.0,
                    upper=SCISSOR_FOLD_ANGLE,
                ),
                mimic=Mimic(
                    joint=f"tabletop_to_{mimic_src}",
                    multiplier=1.0,
                    offset=0.0,
                ),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tabletop = object_model.get_part("tabletop")

    # ── Tabletop integrity (unchanged from parent) ──
    ctx.check(
        "segmented_slat_tabletop",
        tabletop is not None
        and all(
            tabletop.get_visual(f"slat_{i}") is not None for i in range(SLAT_COUNT)
        ),
        "Expected nine separate rounded tabletop slats.",
    )

    aabb = ctx.part_world_aabb(tabletop)
    if aabb is not None:
        min_pt, max_pt = aabb
        ctx.check(
            "tabletop_width_realistic",
            0.78 <= float(max_pt[0] - min_pt[0]) <= 0.90,
            details=str(aabb),
        )
        ctx.check(
            "tabletop_depth_realistic",
            0.54 <= float(max_pt[1] - min_pt[1]) <= 0.64,
            details=str(aabb),
        )
    else:
        ctx.fail("tabletop_aabb_present", "Expected tabletop AABB.")

    # ── Scissor base structure ──
    ctx.check(
        "scissor_base_has_four_arms",
        all(
            object_model.get_part(f"scissor_arm_{i}") is not None
            and object_model.get_articulation(f"tabletop_to_scissor_arm_{i}")
            is not None
            for i in range(4)
        ),
        "Expected four scissor_arm parts with tabletop_to_scissor_arm articulations.",
    )

    # Each arm carries a rubber_foot (kept functional layer from parent).
    for i in range(4):
        arm = object_model.get_part(f"scissor_arm_{i}")
        ctx.check(
            f"scissor_arm_{i}_has_rubber_foot",
            arm.get_visual("rubber_foot") is not None,
            f"Expected rubber_foot visual on scissor_arm_{i}.",
        )

    # Front arm carries the cross stretcher (scissor frame rigidity).
    arm0_check = object_model.get_part("scissor_arm_0")
    ctx.check(
        "scissor_arm_0_has_cross_stretcher",
        arm0_check.get_visual("cross_stretcher") is not None,
        "Expected cross_stretcher visual on scissor_arm_0.",
    )

    # ── Pivot capture overlap (intentional embedding) ──
    for i in range(4):
        arm = object_model.get_part(f"scissor_arm_{i}")
        ctx.allow_overlap(
            tabletop,
            arm,
            elem_a=f"hinge_socket_{i}",
            elem_b="pivot_hub",
            reason="The scissor arm pivot hub is intentionally captured inside the mount socket.",
        )
        ctx.expect_overlap(
            arm,
            tabletop,
            axes="xy",
            elem_a="pivot_hub",
            elem_b=f"hinge_socket_{i}",
            min_overlap=0.016,
            name=f"scissor_arm_{i}_pivot_seated_in_socket",
        )
        ctx.allow_overlap(
            tabletop,
            arm,
            elem_a=f"hinge_socket_{i}",
            elem_b="arm_tube",
            reason="The scissor arm tube begins inside the mount socket, representing the captured tube end.",
        )
        ctx.expect_overlap(
            arm,
            tabletop,
            axes="xy",
            elem_a="arm_tube",
            elem_b=f"hinge_socket_{i}",
            min_overlap=0.008,
            name=f"scissor_arm_{i}_tube_inserted_in_socket",
        )

    # ── Cross stretcher overlaps with crossing arm pivots ──
    # The single front-to-rear stretcher on arm_0 passes through the
    # scissor pivot bushings on the other arms at the crossing points.
    # This represents the structural connection between scissor pairs.
    stretcher_pivot_pairs = [
        ("scissor_arm_0", "scissor_arm_1"),  # front crossing
        ("scissor_arm_0", "scissor_arm_3"),  # rear crossing
    ]
    for name_a, name_b in stretcher_pivot_pairs:
        part_a = object_model.get_part(name_a)
        part_b = object_model.get_part(name_b)
        ctx.allow_overlap(
            part_a,
            part_b,
            elem_a="cross_stretcher",
            elem_b="scissor_pivot",
            reason=(
                f"The {name_a} cross stretcher passes through the {name_b} scissor pivot "
                "at the crossing, representing the structural frame connection."
            ),
        )
    # The stretcher may also overlap with arm tubes at crossing points
    ctx.allow_overlap(
        object_model.get_part("scissor_arm_0"),
        object_model.get_part("scissor_arm_1"),
        elem_a="cross_stretcher",
        elem_b="arm_tube",
        reason="The arm_0 stretcher passes near the arm_1 tube at the front scissor crossing.",
    )
    ctx.allow_overlap(
        object_model.get_part("scissor_arm_0"),
        object_model.get_part("scissor_arm_3"),
        elem_a="cross_stretcher",
        elem_b="arm_tube",
        reason="The arm_0 stretcher passes near the arm_3 tube at the rear scissor crossing.",
    )

    # ── Scissor pivot proximity (crossing arms meet at center) ──
    arm0 = object_model.get_part("scissor_arm_0")
    arm1 = object_model.get_part("scissor_arm_1")
    ctx.expect_contact(
        arm0,
        arm1,
        elem_a="scissor_pivot",
        elem_b="scissor_pivot",
        contact_tol=0.040,
        name="front_scissor_pivots_at_crossing",
    )
    arm2 = object_model.get_part("scissor_arm_2")
    arm3 = object_model.get_part("scissor_arm_3")
    ctx.expect_contact(
        arm2,
        arm3,
        elem_a="scissor_pivot",
        elem_b="scissor_pivot",
        contact_tol=0.040,
        name="rear_scissor_pivots_at_crossing",
    )

    # ── Deployed pose: feet reach near ground ──
    primary_joint = object_model.get_articulation("tabletop_to_scissor_arm_0")
    rest_aabb = ctx.part_world_aabb(arm0)
    if rest_aabb is not None:
        rest_min, rest_max = rest_aabb
        ctx.check(
            "scissor_deployed_reaches_near_ground",
            float(rest_min[2]) < 0.060,
            details=f"deployed arm0 min_z={rest_min[2]:.4f}",
        )

    # ── Folded pose: scissor collapses flat under tabletop ──
    with ctx.pose({primary_joint: SCISSOR_FOLD_ANGLE}):
        folded_aabb = ctx.part_world_aabb(arm0)
    if rest_aabb is not None and folded_aabb is not None:
        rest_min, _rest_max = rest_aabb
        fold_min, fold_max = folded_aabb
        ctx.check(
            "scissor_folds_flat_under_tabletop",
            float(fold_min[2]) > float(rest_min[2]) + 0.20
            and float(fold_max[2]) < TOP_Z - 0.010,
            details=f"rest={rest_aabb}, folded={folded_aabb}",
        )

    # ── Verify the mechanism actually moves the foot upward when folding ──
    if rest_aabb is not None and folded_aabb is not None:
        rest_min, _ = rest_aabb
        fold_min, _ = folded_aabb
        ctx.check(
            "scissor_fold_raises_foot",
            float(fold_min[2]) > float(rest_min[2]) + 0.15,
            details=f"rest_min_z={rest_min[2]:.4f}, fold_min_z={fold_min[2]:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
