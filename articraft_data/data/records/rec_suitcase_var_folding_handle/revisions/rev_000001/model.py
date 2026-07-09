from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

# --- Dimensions (meters) -----------------------------------------------------
W = 0.46          # width  (X)
D = 0.34          # depth  (Y)
HB = 0.085        # body half height (Z)
HL = 0.075        # lid half height (Z)
T = 0.012         # shell wall thickness
SEAM_GAP = 0.0     # closed seam: lid walls contact body walls at z=HB
H = HB + HL       # total closed height

# Latch placement
LATCH_X = 0.13
LATCH_STANDOFF = 0.007
LATCH_W = 0.05
LATCH_T = 0.012
LATCH_H = 0.07

# Folding handle dimensions (on lid top surface)
HANDLE_ARM_LENGTH = 0.055   # arm length from pivot to grip center
HANDLE_ARM_W = 0.012        # arm bar width
HANDLE_ARM_H = 0.006        # arm bar thickness
HANDLE_PIVOT_X = 0.055      # half-spacing between the two arm pivots
HANDLE_GRIP_LENGTH = 0.13   # wooden grip bar length
HANDLE_GRIP_RADIUS = 0.009  # grip cylinder radius
HANDLE_MOUNT_W = 0.020      # pivot bracket width
HANDLE_MOUNT_D = 0.016      # pivot bracket depth (Y)
HANDLE_MOUNT_H = 0.020      # pivot bracket height above lid surface
HANDLE_PIVOT_Z = 0.012      # pivot axis height above lid outer surface


def _add_box_shell(part, *, width, depth, height, wall, material, z0=0.0, open_top=True):
    """Add a hollow rectangular shell: floor/ceiling panel + four walls.

    Pieces overlap at the edges so the part reads as one connected island.
    """
    cap_z = z0 + (wall / 2.0) if open_top else z0 + height - wall / 2.0
    part.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, cap_z)),
        material=material,
        name="shell_panel",
    )
    wall_h = height
    cz = z0 + height / 2.0
    part.visual(
        Box((width, wall, wall_h)),
        origin=Origin(xyz=(0.0, -(depth / 2.0 - wall / 2.0), cz)),
        material=material,
        name="wall_front",
    )
    part.visual(
        Box((width, wall, wall_h)),
        origin=Origin(xyz=(0.0, (depth / 2.0 - wall / 2.0), cz)),
        material=material,
        name="wall_back",
    )
    part.visual(
        Box((wall, depth, wall_h)),
        origin=Origin(xyz=(-(width / 2.0 - wall / 2.0), 0.0, cz)),
        material=material,
        name="wall_left",
    )
    part.visual(
        Box((wall, depth, wall_h)),
        origin=Origin(xyz=((width / 2.0 - wall / 2.0), 0.0, cz)),
        material=material,
        name="wall_right",
    )


def _add_corner_caps(part, *, width, depth, z_lo, z_hi, material, prefix):
    cap = 0.045
    capz = (z_lo + z_hi) / 2.0
    caph = (z_hi - z_lo)
    i = 0
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            part.visual(
                Box((cap, cap, caph)),
                origin=Origin(
                    xyz=(
                        sx * (width / 2.0 - cap / 2.0 + 0.004),
                        sy * (depth / 2.0 - cap / 2.0 + 0.004),
                        capz,
                    )
                ),
                material=material,
                name=f"{prefix}_corner_{i}",
            )
            i += 1


def _add_handle_arm(part, *, index, sign, material):
    """Add one swing arm of the folding handle in the handle local frame."""
    part.visual(
        Box((HANDLE_ARM_W, HANDLE_ARM_LENGTH, HANDLE_ARM_H)),
        origin=Origin(xyz=(sign * HANDLE_PIVOT_X, -HANDLE_ARM_LENGTH / 2.0, 0.0)),
        material=material,
        name=f"handle_arm_{index}",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_travel_suitcase")

    leather = model.material("leather_body", rgba=(0.32, 0.15, 0.10, 1.0))
    trim = model.material("leather_trim", rgba=(0.18, 0.09, 0.06, 1.0))
    corner = model.material("corner_cap", rgba=(0.22, 0.11, 0.07, 1.0))
    wood = model.material("wood_handle", rgba=(0.55, 0.27, 0.07, 1.0))
    metal = model.material("metal_latch", rgba=(0.58, 0.58, 0.62, 1.0))
    handle_hw = model.material("handle_hardware", rgba=(0.42, 0.42, 0.48, 1.0))

    # --- Body (root) ---------------------------------------------------------
    body = model.part("suitcase_body")
    _add_box_shell(
        body, width=W, depth=D, height=HB, wall=T, material=leather, z0=0.0, open_top=True
    )
    _add_corner_caps(
        body, width=W, depth=D, z_lo=0.0, z_hi=HB - 0.02, material=corner, prefix="body"
    )
    # vertical leather trim straps on the long faces
    for i, sx in enumerate((-0.16, 0.16)):
        body.visual(
            Box((0.03, D + 0.004, 0.012)),
            origin=Origin(xyz=(sx, 0.0, HB - 0.018)),
            material=trim,
            name=f"body_strap_{i}",
        )

    # latch hinge bosses: bridge the front face out to the proud latch plate
    plate_inner_y = -(D / 2.0) - LATCH_STANDOFF
    boss_outer_y = plate_inner_y
    boss_inner_y = -(D / 2.0) + 0.002
    boss_depth = boss_inner_y - boss_outer_y
    boss_cy = (boss_inner_y + boss_outer_y) / 2.0
    for i, (side, sx) in enumerate((("l", -LATCH_X), ("r", LATCH_X))):
        body.visual(
            Box((0.026, boss_depth, 0.020)),
            origin=Origin(xyz=(sx, boss_cy, HB - 0.018)),
            material=metal,
            name=f"latch_boss_{i}",
        )

    # --- Lid -----------------------------------------------------------------
    # Lid part frame sits on the rear hinge line at world (0, D/2, HB).
    # All lid geometry is authored in that local frame.
    lid = model.part("suitcase_lid")
    lid_panel_z = HL - T / 2.0
    wall_h = HL - T - SEAM_GAP
    wall_cz = SEAM_GAP + wall_h / 2.0
    # top panel
    lid.visual(
        Box((W, D, T)),
        origin=Origin(xyz=(0.0, -D / 2.0, lid_panel_z)),
        material=leather,
        name="shell_panel",
    )
    # front wall (world front, local -Y far)
    lid.visual(
        Box((W, T, wall_h)),
        origin=Origin(xyz=(0.0, -(D - T / 2.0), wall_cz)),
        material=leather,
        name="wall_front",
    )
    # back wall (near hinge)
    lid.visual(
        Box((W, T, wall_h)),
        origin=Origin(xyz=(0.0, -(T / 2.0), wall_cz)),
        material=leather,
        name="wall_back",
    )
    lid.visual(
        Box((T, D, wall_h)),
        origin=Origin(xyz=(-(W / 2.0 - T / 2.0), -D / 2.0, wall_cz)),
        material=leather,
        name="wall_left",
    )
    lid.visual(
        Box((T, D, wall_h)),
        origin=Origin(xyz=((W / 2.0 - T / 2.0), -D / 2.0, wall_cz)),
        material=leather,
        name="wall_right",
    )
    # lid corner caps (local frame)
    capm = 0.045
    for i, (sx, ly) in enumerate(
        [
            (-1.0, -(T / 2.0)),
            (1.0, -(T / 2.0)),
            (-1.0, -(D - T / 2.0)),
            (1.0, -(D - T / 2.0)),
        ]
    ):
        lid.visual(
            Box((capm, capm, HL - 0.02)),
            origin=Origin(
                xyz=(sx * (W / 2.0 - capm / 2.0 + 0.004), ly, (SEAM_GAP + HL - 0.02) / 2.0)
            ),
            material=corner,
            name=f"lid_corner_{i}",
        )
    # lid trim straps
    for i, sx in enumerate((-0.16, 0.16)):
        lid.visual(
            Box((0.03, D + 0.004, 0.012)),
            origin=Origin(xyz=(sx, -D / 2.0, SEAM_GAP + 0.010)),
            material=trim,
            name=f"lid_strap_{i}",
        )

    # Pivot brackets for the folding handle (on lid top surface, center)
    for i, sx in enumerate((-1.0, 1.0)):
        lid.visual(
            Box((HANDLE_MOUNT_W, HANDLE_MOUNT_D, HANDLE_MOUNT_H)),
            origin=Origin(
                xyz=(sx * HANDLE_PIVOT_X, -D / 2.0, HL + HANDLE_MOUNT_H / 2.0)
            ),
            material=handle_hw,
            name=f"handle_pivot_mount_{i}",
        )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, D / 2.0, HB)),
        # Closed lid extends along local -Y from the hinge; -X axis lifts the
        # free front edge upward for positive q.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=2.0),
    )

    # --- Folding handle (child of lid) ---------------------------------------
    # At q=0 the handle lies flat (arms extend along local -Y, horizontal).
    # Positive q swings the arms upward (from -Y toward +Z) for carrying.
    handle = model.part("folding_handle")
    for i, sx in enumerate((-1.0, 1.0)):
        _add_handle_arm(handle, index=i, sign=sx, material=handle_hw)
    # Pivot rod: thin axle connecting both arms at the pivot axis
    handle.visual(
        Cylinder(radius=0.004, length=2.0 * HANDLE_PIVOT_X + HANDLE_ARM_W),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_hw,
        name="handle_pivot_rod",
    )
    # Wooden grip bar at the arm far ends
    handle.visual(
        Cylinder(radius=HANDLE_GRIP_RADIUS, length=HANDLE_GRIP_LENGTH),
        origin=Origin(
            xyz=(0.0, -HANDLE_ARM_LENGTH, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=wood,
        name="handle_grip",
    )

    model.articulation(
        "handle_fold",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=handle,
        # Pivot axis at the top of the mount brackets, center of lid
        origin=Origin(xyz=(0.0, -D / 2.0, HL + HANDLE_PIVOT_Z)),
        # -X axis: positive q swings arms from -Y (flat) toward +Z (deployed)
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.3),
    )

    # --- Front latches -------------------------------------------------------
    # Each latch is a proud metal plate hinged at its base on the front face,
    # spanning the seam. q=0 is the closed/engaged (upright) state.
    latch_y = -(D / 2.0) - LATCH_STANDOFF - LATCH_T / 2.0
    for side, sx in (("left", -LATCH_X), ("right", LATCH_X)):
        latch = model.part(f"latch_{side}")
        latch.visual(
            Box((LATCH_W, LATCH_T, LATCH_H)),
            origin=Origin(xyz=(0.0, 0.0, LATCH_H / 2.0)),
            material=metal,
            name="latch_plate",
        )
        latch.visual(
            Box((0.018, LATCH_T + 0.004, 0.014)),
            origin=Origin(xyz=(0.0, -0.002, LATCH_H - 0.012)),
            material=metal,
            name="latch_catch",
        )
        model.articulation(
            f"latch_{side}_hinge",
            ArticulationType.REVOLUTE,
            parent=body,
            child=latch,
            # hinge at the plate base on the front face, near the seam
            origin=Origin(xyz=(sx, latch_y, HB - 0.018)),
            # plate extends local +Z when closed; +X axis flips it forward/down to release
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=1.4),
        )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("suitcase_body")
    lid = object_model.get_part("suitcase_lid")
    handle = object_model.get_part("folding_handle")
    latch_l = object_model.get_part("latch_left")
    latch_r = object_model.get_part("latch_right")
    lid_hinge = object_model.get_articulation("lid_hinge")
    handle_fold = object_model.get_articulation("handle_fold")
    latch_l_hinge = object_model.get_articulation("latch_left_hinge")

    # --- Allow intentional pivot overlaps (captured-pin pattern) -------------
    # The pivot rod and arm bases pass through the mount brackets at the hinge.
    for i in range(2):
        ctx.allow_overlap(
            lid,
            handle,
            elem_a=f"handle_pivot_mount_{i}",
            elem_b="handle_pivot_rod",
            reason="Pivot rod is captured inside the mount bracket as a hinge pin.",
        )
        ctx.allow_overlap(
            lid,
            handle,
            elem_a=f"handle_pivot_mount_{i}",
            elem_b=f"handle_arm_{i}",
            reason="Arm base pivots within the mount bracket at the hinge point.",
        )

    # --- Closed seam ---------------------------------------------------------
    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.0,
            max_gap=0.006,
            positive_elem="wall_front",
            negative_elem="wall_front",
            name="lid seats on body at the closed seam",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.20,
            name="closed lid footprint matches body footprint",
        )
        closed_lid_aabb = ctx.part_world_aabb(lid)

    # --- Lid opens upward ----------------------------------------------------
    with ctx.pose({lid_hinge: 1.8}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid swings up when opened",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.05,
        details=f"closed_top={closed_lid_aabb}, open_top={open_lid_aabb}",
    )

    # --- Folding handle mechanism --------------------------------------------
    # Handle exists with required visuals
    grip = handle.get_visual("handle_grip")
    rod = handle.get_visual("handle_pivot_rod")
    mount_0 = lid.get_visual("handle_pivot_mount_0")
    mount_1 = lid.get_visual("handle_pivot_mount_1")
    ctx.check(
        "folding handle has grip, pivot rod, and lid mounts",
        grip is not None and rod is not None and mount_0 is not None and mount_1 is not None,
        details="expected handle_grip, handle_pivot_rod on handle; mounts on lid",
    )

    # At q=0 (flat), handle grip sits near the lid top surface (not penetrating)
    with ctx.pose({handle_fold: 0.0}):
        ctx.expect_gap(
            handle,
            lid,
            axis="z",
            min_gap=-0.001,
            max_gap=0.025,
            positive_elem="handle_grip",
            negative_elem="shell_panel",
            name="folded handle grip rests near the lid top surface",
        )
        flat_grip_aabb = ctx.part_element_world_aabb(handle, elem="handle_grip")

    # At q=1.2 (deployed), grip rises well above the flat position
    with ctx.pose({handle_fold: 1.2}):
        deployed_grip_aabb = ctx.part_element_world_aabb(handle, elem="handle_grip")

    ctx.check(
        "folding handle swings up from flat for carrying",
        flat_grip_aabb is not None
        and deployed_grip_aabb is not None
        and deployed_grip_aabb[0][2] > flat_grip_aabb[0][2] + 0.025,
        details=f"flat_min_z={flat_grip_aabb}, deployed_min_z={deployed_grip_aabb}",
    )

    # Proof: pivot rod contacts the mounts (validates the pivot connection)
    ctx.expect_contact(
        handle,
        lid,
        elem_a="handle_pivot_rod",
        elem_b="handle_pivot_mount_0",
        name="pivot rod contacts left mount bracket",
    )
    ctx.expect_contact(
        handle,
        lid,
        elem_a="handle_pivot_rod",
        elem_b="handle_pivot_mount_1",
        name="pivot rod contacts right mount bracket",
    )

    # --- Two front latches ---------------------------------------------------
    ctx.check(
        "two front latches are present",
        latch_l is not None and latch_r is not None,
        details="expected latch_left and latch_right",
    )

    # Latch releases: opening the latch flips the catch down off the lid.
    with ctx.pose({latch_l_hinge: 0.0}):
        closed_latch_aabb = ctx.part_world_aabb(latch_l)
    with ctx.pose({latch_l_hinge: 1.3}):
        open_latch_aabb = ctx.part_world_aabb(latch_l)
    ctx.check(
        "front latch flips open to release the lid",
        closed_latch_aabb is not None
        and open_latch_aabb is not None
        and open_latch_aabb[1][2] < closed_latch_aabb[1][2] - 0.02,
        details=f"closed={closed_latch_aabb}, open={open_latch_aabb}",
    )

    return ctx.report()
