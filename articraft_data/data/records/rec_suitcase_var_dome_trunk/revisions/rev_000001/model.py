from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# --- Dimensions (meters) -----------------------------------------------------
W = 0.46          # width  (X)
D = 0.34          # depth  (Y)
HB = 0.085        # body height (Z)
DOME_H = 0.08     # dome peak height above rim
T = 0.012         # shell wall thickness
SEAM_GAP = 0.0015 # closed clearance between body rim and lid rim
DOME_RIM_Z = SEAM_GAP  # dome rim z in lid local frame

# Dome geometry: barrel-vault radius from chord=D and sagitta=DOME_H
_DOME_R = (D * D / 4.0 + DOME_H * DOME_H) / (2.0 * DOME_H)

# Handle / latch placement
LATCH_X = 0.13
LATCH_STANDOFF = 0.007
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


def _build_dome_shell(width, depth, dome_h, wall_t):
    """Build a barrel-vault (humpback) dome shell using CadQuery.

    The dome rim sits at z=0, peak at z=dome_h.
    The shell thickness is ABOVE the rim (inside the dome), so the
    outer surface touches z=0 at the rim edges and the inner surface
    is at z=wall_t at the rim. This ensures the dome does not extend
    below the rim plane.
    Extends from y=0 (back/hinge edge) to y=-depth (front edge).
    Centered on x=0 after extrusion.
    """
    profile = (
        cq.Workplane("YZ")
        .moveTo(0.0, 0.0)
        .threePointArc((-depth / 2.0, dome_h), (-depth, 0.0))
        .lineTo(-depth, wall_t)
        .threePointArc((-depth / 2.0, dome_h - wall_t), (0.0, wall_t))
        .close()
    )
    return profile.extrude(width).translate((-width / 2.0, 0.0, 0.0))


def _build_dome_endwall(depth, dome_h, x0, thickness):
    """Build a solid end wall that closes the barrel-vault dome on one side.

    The barrel vault only arches front-to-back, so its left/right ends are open
    crescents (the lid cannot actually cover the box there). This fills the
    crescent between the rim baseline (z=0) and the outer dome arch with a thin
    wall, so the closed lid fully encloses the body on every side.

    x0 is the lower-x face of the wall; the wall occupies [x0, x0+thickness].
    """
    profile = (
        cq.Workplane("YZ")
        .moveTo(0.0, 0.0)
        .threePointArc((-depth / 2.0, dome_h), (-depth, 0.0))
        .close()  # straight baseline back along z=0 -> filled tombstone
    )
    return profile.extrude(thickness).translate((x0, 0.0, 0.0))


def _dome_surface_z(y_local):
    """Return dome outer surface height above rim at a given local y.

    y_local is in the lid/dome frame (0 = back/hinge, -D = front).
    """
    y_center = -D / 2.0
    center_z = DOME_H - _DOME_R
    dy = y_local - y_center
    radicand = _DOME_R * _DOME_R - dy * dy
    if radicand < 0.0:
        return 0.0
    return center_z + math.sqrt(radicand)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_travel_suitcase")

    leather = model.material("leather_body", rgba=(0.32, 0.15, 0.10, 1.0))
    trim = model.material("leather_trim", rgba=(0.18, 0.09, 0.06, 1.0))
    corner = model.material("corner_cap", rgba=(0.22, 0.11, 0.07, 1.0))
    wood = model.material("wood_handle", rgba=(0.55, 0.27, 0.07, 1.0))
    metal = model.material("metal_latch", rgba=(0.58, 0.58, 0.62, 1.0))
    brass = model.material("brass_band", rgba=(0.72, 0.58, 0.22, 1.0))

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

    # latch hinge bosses: bridge the front face out to the proud latch plate
    plate_inner_y = -(D / 2.0) - LATCH_STANDOFF
    boss_outer_y = plate_inner_y
    boss_inner_y = -(D / 2.0) + 0.002
    boss_depth = boss_inner_y - boss_outer_y
    boss_cy = (boss_inner_y + boss_outer_y) / 2.0
    for side, sx in (("l", -LATCH_X), ("r", LATCH_X)):
        body.visual(
            Box((0.026, boss_depth, 0.020)),
            origin=Origin(xyz=(sx, boss_cy, HB - 0.018)),
            material=metal,
            name=f"latch_boss_{side}",
        )

    # --- Lid (domed humpback steamer-trunk) ----------------------------------
    lid = model.part("suitcase_lid")

    # Barrel-vault dome shell (CadQuery mesh)
    dome_cq = _build_dome_shell(W, D, DOME_H, T)
    lid.visual(
        mesh_from_cadquery(dome_cq, "dome_shell", tolerance=0.0005, angular_tolerance=0.08),
        origin=Origin(xyz=(0.0, 0.0, DOME_RIM_Z)),
        material=leather,
        name="dome_shell",
    )

    # End walls that close the open crescents on the left/right of the vault so
    # the lid fully covers the body. Each occupies the outermost shell-thickness
    # slab, flush above the matching body side wall.
    for i, x0 in enumerate((-W / 2.0, W / 2.0 - T)):
        end_cq = _build_dome_endwall(D, DOME_H, x0, T)
        lid.visual(
            mesh_from_cadquery(
                end_cq, f"dome_end_{i}", tolerance=0.0005, angular_tolerance=0.08
            ),
            origin=Origin(xyz=(0.0, 0.0, DOME_RIM_Z)),
            material=leather,
            name=f"dome_end_{i}",
        )

    # Lid corner caps at the dome rim (short protective bands)
    lid_cap_h = 0.028
    # Caps start just inside the dome shell thickness for connectivity
    lid_cap_z_bottom = DOME_RIM_Z + 0.002
    lid_cap_z_center = lid_cap_z_bottom + lid_cap_h / 2.0
    capm = 0.045
    for i, (sx, ly) in enumerate([
        (-1.0, -(T / 2.0)),
        (1.0, -(T / 2.0)),
        (-1.0, -(D - T / 2.0)),
        (1.0, -(D - T / 2.0)),
    ]):
        lid.visual(
            Box((capm, capm, lid_cap_h)),
            origin=Origin(
                xyz=(sx * (W / 2.0 - capm / 2.0 + 0.004), ly, lid_cap_z_center)
            ),
            material=corner,
            name=f"lid_corner_{i}",
        )

    # Brass bands across the dome (characteristic steamer-trunk feature)
    for i, frac in enumerate((0.28, 0.72)):
        band_y = -D * frac
        z_surf = _dome_surface_z(band_y)
        # Band straddles dome surface: center at surface, slight embed for connectivity
        band_z = DOME_RIM_Z + z_surf
        lid.visual(
            Box((W + 0.004, 0.022, 0.005)),
            origin=Origin(xyz=(0.0, band_y, band_z)),
            material=brass,
            name=f"dome_band_{i}",
        )

    # Leather straps along the dome (run front-to-back near peak)
    for i, sx in enumerate((-0.14, 0.14)):
        # Place near dome peak where surface is relatively flat
        strap_y = -D / 2.0
        z_surf = _dome_surface_z(strap_y)
        strap_z = DOME_RIM_Z + z_surf
        lid.visual(
            Box((0.028, D * 0.40, 0.005)),
            origin=Origin(xyz=(sx, strap_y, strap_z)),
            material=trim,
            name=f"lid_strap_{i}",
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
            origin=Origin(xyz=(sx, latch_y, HB - 0.018)),
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

    # --- Dome geometry checks ------------------------------------------------
    # Dome shell exists and has significant height (proves curved profile)
    dome_aabb = ctx.part_element_world_aabb(lid, elem="dome_shell")
    ctx.check(
        "dome shell has curved barrel-vault profile",
        dome_aabb is not None and (dome_aabb[1][2] - dome_aabb[0][2]) > 0.04,
        details=f"dome_aabb={dome_aabb}, expected height > 0.04m",
    )

    # Side end walls exist and close the open crescents on both sides.
    for i in range(2):
        end_aabb = ctx.part_element_world_aabb(lid, elem=f"dome_end_{i}")
        ctx.check(
            f"dome side end wall {i} closes the vault crescent",
            end_aabb is not None and (end_aabb[1][2] - end_aabb[0][2]) > DOME_H * 0.7,
            details=f"end_{i}_aabb={end_aabb}, expected height > {DOME_H * 0.7:.3f}",
        )

    # Closed-pose checks
    with ctx.pose({lid_hinge: 0.0}):
        # Lid fully spans the body footprint in width and depth (no open sides)
        closed_lid_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "closed lid spans the full body width and depth",
            closed_lid_aabb is not None
            and (closed_lid_aabb[1][0] - closed_lid_aabb[0][0]) > W * 0.98
            and (closed_lid_aabb[1][1] - closed_lid_aabb[0][1]) > D * 0.98,
            details=f"lid_aabb={closed_lid_aabb}, body W={W} D={D}",
        )
        # Dome peak rises well above body top (proves humpback shape)
        ctx.check(
            "dome peak rises above body top when closed",
            closed_lid_aabb is not None and closed_lid_aabb[1][2] > HB + 0.04,
            details=f"lid_top_z={closed_lid_aabb[1][2] if closed_lid_aabb else None}, body_top={HB}",
        )
        # Lid footprint still covers the body
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.20,
            name="closed dome lid footprint matches body",
        )
        # Dome rim sits just above body top at the front seam
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.0,
            max_gap=0.008,
            positive_elem="dome_shell",
            negative_elem="wall_front",
            name="dome rim seats above body at closed seam",
        )

    # Lid opens upward: dome sweeps above closed height
    with ctx.pose({lid_hinge: 1.8}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "dome lid swings up when opened",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.05,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    # Handle grip exists and stands proud of the front face
    grip = body.get_visual("handle_grip")
    grip_aabb = ctx.part_element_world_aabb(body, elem="handle_grip")
    ctx.check(
        "carry handle grip is present on the front face",
        grip is not None and grip_aabb is not None and grip_aabb[0][1] < -(D / 2.0),
        details=f"grip_aabb={grip_aabb}",
    )

    # Two latches exist and are distinct
    ctx.check(
        "two front latches are present",
        latch_l is not None and latch_r is not None,
        details="expected latch_left and latch_right",
    )

    # Latch releases: opening the latch flips the catch down
    with ctx.pose({latch_l_hinge: 0.0}):
        closed_latch_aabb = ctx.part_world_aabb(latch_l)
    with ctx.pose({latch_l_hinge: 1.3}):
        open_latch_aabb = ctx.part_world_aabb(latch_l)
    ctx.check(
        "front latch flips open to release the dome lid",
        closed_latch_aabb is not None
        and open_latch_aabb is not None
        and open_latch_aabb[1][2] < closed_latch_aabb[1][2] - 0.02,
        details=f"closed={closed_latch_aabb}, open={open_latch_aabb}",
    )

    return ctx.report()
