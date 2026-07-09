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
SEAM_GAP = 0.0015  # closed clearance between body rim and lid rim
H = HB + HL       # total closed height

# Handle / latch placement
LATCH_X = 0.13
LATCH_STANDOFF = 0.007  # latch plate standoff proud of the front face
LATCH_W = 0.05
LATCH_T = 0.012
LATCH_H = 0.07


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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_travel_suitcase")

    leather = model.material("leather_body", rgba=(0.32, 0.15, 0.10, 1.0))
    trim = model.material("leather_trim", rgba=(0.18, 0.09, 0.06, 1.0))
    corner = model.material("corner_cap", rgba=(0.22, 0.11, 0.07, 1.0))
    wood = model.material("wood_handle", rgba=(0.55, 0.27, 0.07, 1.0))
    metal = model.material("metal_latch", rgba=(0.58, 0.58, 0.62, 1.0))

    # --- Body (root) ---------------------------------------------------------
    body = model.part("suitcase_body")
    _add_box_shell(
        body, width=W, depth=D, height=HB, wall=T, material=leather, z0=0.0, open_top=True
    )
    _add_corner_caps(
        body, width=W, depth=D, z_lo=0.0, z_hi=HB - 0.02, material=corner, prefix="body"
    )
    # vertical leather trim straps on the long faces
    for sx in (-0.16, 0.16):
        body.visual(
            Box((0.03, D + 0.004, 0.012)),
            origin=Origin(xyz=(sx, 0.0, HB - 0.018)),
            material=trim,
            name=f"body_strap_{'l' if sx < 0 else 'r'}",
        )

    # carry handle on the front (-Y) face: wooden grip held by two loops
    grip_y = -(D / 2.0) - 0.022
    grip_z = HB - 0.006
    body.visual(
        Cylinder(radius=0.012, length=0.16),
        origin=Origin(xyz=(0.0, grip_y, grip_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wood,
        name="handle_grip",
    )
    for sx in (-0.06, 0.06):
        body.visual(
            Box((0.02, 0.030, 0.022)),
            origin=Origin(xyz=(sx, -(D / 2.0) - 0.008, grip_z)),
            material=trim,
            name=f"handle_loop_{'l' if sx < 0 else 'r'}",
        )

    # latch hinge bosses: bridge the front face out to the proud latch plate so
    # the latch is mechanically grounded (face contact at the hinge line).
    plate_inner_y = -(D / 2.0) - LATCH_STANDOFF  # inner face of the latch plate
    boss_outer_y = plate_inner_y                  # boss outer face meets plate inner face
    boss_inner_y = -(D / 2.0) + 0.002             # embed slightly into the front wall
    boss_depth = boss_inner_y - boss_outer_y
    boss_cy = (boss_inner_y + boss_outer_y) / 2.0
    for side, sx in (("l", -LATCH_X), ("r", LATCH_X)):
        body.visual(
            Box((0.026, boss_depth, 0.020)),
            origin=Origin(xyz=(sx, boss_cy, HB - 0.018)),
            material=metal,
            name=f"latch_boss_{side}",
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
    for sx in (-0.16, 0.16):
        lid.visual(
            Box((0.03, D + 0.004, 0.012)),
            origin=Origin(xyz=(sx, -D / 2.0, SEAM_GAP + 0.010)),
            material=trim,
            name=f"lid_strap_{'l' if sx < 0 else 'r'}",
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
    latch_l = object_model.get_part("latch_left")
    latch_r = object_model.get_part("latch_right")
    lid_hinge = object_model.get_articulation("lid_hinge")
    latch_l_hinge = object_model.get_articulation("latch_left_hinge")

    # Closed seam: lid rim sits just above the body rim, no penetration.
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

    # Lid opens upward: the lid sweeps above its closed height.
    with ctx.pose({lid_hinge: 1.8}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid swings up when opened",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.05,
        details=f"closed_top={closed_lid_aabb}, open_top={open_lid_aabb}",
    )

    # Handle grip exists and stands proud of the front face on the body.
    grip = body.get_visual("handle_grip")
    grip_aabb = ctx.part_element_world_aabb(body, elem="handle_grip")
    ctx.check(
        "carry handle grip is present on the front face",
        grip is not None and grip_aabb is not None and grip_aabb[0][1] < -(D / 2.0),
        details=f"grip_aabb={grip_aabb}",
    )

    # Two latches exist and are distinct.
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
