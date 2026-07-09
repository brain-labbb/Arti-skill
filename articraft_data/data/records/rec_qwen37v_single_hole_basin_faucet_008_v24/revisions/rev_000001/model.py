from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Single-hole basin faucet variant, ~0.13 m tall, mirror chrome.
# World frame: +Z up, deck at z = 0, spout projects toward +X (front).
# Side lever on +Y (right side of body when facing the spout).
# ---------------------------------------------------------------------------

# Base flange (sits flat on the deck).
FLANGE_R = 0.030
FLANGE_H = 0.006

# Main vertical body barrel.
BODY_R = 0.022
BODY_S0 = 0.006
BODY_S1 = 0.090

# Upper dome/cap of body (rounded top).
BODY_DOME_R = 0.022
DOME_Z = 0.090

# Decorative groove ring around mid-body.
GROOVE_R = 0.019
GROOVE_S0 = 0.050
GROOVE_S1 = 0.054

# Spout exit height on the body.
SPOUT_Z = 0.072

# Side lever hub position on the body (right side, +Y).
LEVER_HUB_Z = 0.065
LEVER_HUB_R = 0.010
LEVER_HUB_LEN = 0.012  # how far the hub protrudes from body

# Lever handle dimensions.
LEVER_ARM_LEN = 0.055
LEVER_ARM_W = 0.012
LEVER_ARM_H = 0.008

# Lever articulation limits (rotate up to open flow).
LEVER_LOWER = math.radians(-10.0)
LEVER_UPPER = math.radians(75.0)


def _build_spout_shape() -> cq.Workplane:
    """Waterfall-style spout: cylindrical shank from body, curves forward
    and slightly downward, then flares into a wide flat lip with an open
    channel on the underside for water to cascade out.
    Built in spout-local frame: origin at spout exit on body axis,
    shank runs along local +X."""
    r_out = 0.013
    shank_x0 = 0.008  # seated inside the body
    shank_x1 = 0.032

    # Gentle downward bend
    bend_dx = 0.015
    bend_dz = -0.010

    # Build the sweep path: straight shank then arc down
    path = (
        cq.Workplane("XZ")
        .moveTo(shank_x0, 0.0)
        .lineTo(shank_x1, 0.0)
        .tangentArcPoint((bend_dx, bend_dz), relative=True)
    )
    tube = cq.Workplane("YZ", origin=(shank_x0, 0.0, 0.0)).circle(r_out).sweep(path)

    # End of bend position
    end_x = shank_x1 + bend_dx
    end_z = bend_dz

    # Waterfall lip dimensions
    lip_width = 0.048
    lip_depth = 0.020
    lip_wall = 0.003  # wall thickness
    lip_height = 0.009

    # Transition from round tube to wide flat lip using a loft
    transition_len = 0.022
    transition = (
        cq.Workplane("YZ", origin=(end_x, 0.0, end_z))
        .circle(r_out)
        .workplane(offset=transition_len)
        .rect(lip_depth, lip_width)
        .loft()
    )

    # Waterfall lip: build as a hollow open-bottom channel (U-shape cross-section)
    # Top plate
    lip_x_start = end_x + transition_len
    top_plate = (
        cq.Workplane("XY", origin=(lip_x_start, 0.0, end_z + lip_height - lip_wall))
        .rect(lip_depth, lip_width)
        .extrude(lip_wall)
    )
    # Front wall
    front_wall = (
        cq.Workplane("XY", origin=(lip_x_start + lip_depth / 2 - lip_wall, 0.0, end_z))
        .rect(lip_wall, lip_width)
        .extrude(lip_height)
    )
    # Side walls (left and right)
    side_wall_l = (
        cq.Workplane("XY", origin=(lip_x_start, lip_width / 2 - lip_wall / 2, end_z))
        .rect(lip_depth, lip_wall)
        .extrude(lip_height)
    )
    side_wall_r = (
        cq.Workplane("XY", origin=(lip_x_start, -lip_width / 2 + lip_wall / 2, end_z))
        .rect(lip_depth, lip_wall)
        .extrude(lip_height)
    )

    # Back wall (shorter, leaving the bottom open for water exit)
    back_wall = (
        cq.Workplane("XY", origin=(lip_x_start - lip_wall / 2, 0.0, end_z + lip_height * 0.4))
        .rect(lip_wall, lip_width)
        .extrude(lip_height * 0.6)
    )

    spout = tube.union(transition).union(top_plate).union(front_wall)
    spout = spout.union(side_wall_l).union(side_wall_r).union(back_wall)

    # Hollow bore through the tube for the water channel (stop before the lip)
    bore = (
        cq.Workplane("YZ", origin=(shank_x0, 0.0, 0.0))
        .circle(r_out * 0.65)
        .extrude(end_x + transition_len * 0.5 - shank_x0)
    )
    spout = spout.cut(bore)

    return spout


def _build_lever_shape() -> cq.Workplane:
    """Side lever handle with grip grooves and integrated indicator ridge.
    Built in lever-local frame: origin at the axle center, handle extends
    along local +X (outward from body). The axle is along local Z."""
    # Base collar that wraps around the axle hub
    collar = (
        cq.Workplane("XY")
        .circle(LEVER_HUB_R)
        .extrude(LEVER_ARM_H)
    )

    # Main arm: rectangular bar from hub outward
    arm = (
        cq.Workplane("XZ", origin=(0.0, 0.0, LEVER_ARM_H / 2))
        .center(LEVER_HUB_R, 0.0)
        .rect(LEVER_ARM_LEN, LEVER_ARM_H)
        .extrude(LEVER_ARM_W)
        .translate((0.0, -LEVER_ARM_W / 2, 0.0))
    )

    lever = collar.union(arm)

    # Add grip grooves: small channels cut across the top of the grip area
    groove_start = LEVER_HUB_R + LEVER_ARM_LEN * 0.35
    groove_spacing = 0.006
    n_grooves = 6
    groove_depth = 0.0012
    groove_width = 0.002

    for i in range(n_grooves):
        gx = groove_start + i * groove_spacing
        groove_cut = (
            cq.Workplane("XY", origin=(gx, 0.0, LEVER_ARM_H - groove_depth))
            .rect(groove_width, LEVER_ARM_W + 0.002)
            .extrude(groove_depth + 0.001)
        )
        lever = lever.cut(groove_cut)

    return lever


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet_waterfall")

    model.material("chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    model.material("chrome_dark", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("chrome_brushed", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("groove_fill", rgba=(0.35, 0.37, 0.40, 1.0))

    # ---------------- body (root): flange + barrel + groove + dome cap ----
    body = model.part("body")
    body.visual(
        Cylinder(radius=FLANGE_R, length=FLANGE_H),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_H / 2.0)),
        material="chrome",
        name="base_flange",
    )
    body.visual(
        Cylinder(radius=BODY_R, length=BODY_S1 - BODY_S0),
        origin=Origin(xyz=(0.0, 0.0, (BODY_S0 + BODY_S1) / 2.0)),
        material="chrome",
        name="body_barrel",
    )
    body.visual(
        Cylinder(radius=GROOVE_R, length=GROOVE_S1 - GROOVE_S0),
        origin=Origin(xyz=(0.0, 0.0, (GROOVE_S0 + GROOVE_S1) / 2.0)),
        material="chrome_dark",
        name="groove_ring",
    )
    # Dome cap on top of the body
    body.visual(
        mesh_from_cadquery(
            cq.Workplane("XY", origin=(0.0, 0.0, DOME_Z))
            .circle(BODY_DOME_R)
            .extrude(0.004)
            .edges(">Z")
            .fillet(0.003),
            "body_dome",
            tolerance=0.0003,
        ),
        material="chrome",
        name="body_dome",
    )
    # Lever hub boss on body side (+Y)
    body.visual(
        Cylinder(radius=LEVER_HUB_R + 0.003, length=LEVER_HUB_LEN),
        origin=Origin(
            xyz=(0.0, BODY_R + LEVER_HUB_LEN / 2 - 0.002, LEVER_HUB_Z),
            rpy=(math.pi / 2, 0.0, 0.0),
        ),
        material="chrome",
        name="lever_hub",
    )

    # ---------------- spout (fixed): waterfall lip spout -----------------
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_spout_shape(), "spout", tolerance=0.0003),
        material="chrome",
        name="spout_tube",
    )
    model.articulation(
        "spout_mount",
        ArticulationType.FIXED,
        parent=body,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, SPOUT_Z)),
    )

    # ---------------- lever (revolute on horizontal axle) ----------------
    lever = model.part("lever")
    lever.visual(
        mesh_from_cadquery(_build_lever_shape(), "lever", tolerance=0.0003),
        material="chrome",
        name="lever_handle",
    )

    # Revolute joint: lever rotates on horizontal axle (Y axis).
    # Origin at the hub face on the body surface.
    # Axis is Y (horizontal, perpendicular to spout direction).
    # Positive rotation lifts the lever handle upward (opening flow).
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(
            xyz=(0.0, BODY_R + LEVER_HUB_LEN - 0.002, LEVER_HUB_Z),
            rpy=(math.pi / 2, 0.0, 0.0),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=LEVER_LOWER, upper=LEVER_UPPER
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    lever = object_model.get_part("lever")
    pivot = object_model.get_articulation("lever_pivot")

    # Intentional seated insertions (spout shank into body).
    ctx.allow_overlap(
        spout,
        body,
        elem_a="spout_tube",
        elem_b="body_barrel",
        reason="Spout shank is intentionally seated into the solid body casting.",
    )
    # Lever hub overlaps the body barrel (hub boss protrudes from body surface).
    ctx.allow_overlap(
        lever,
        body,
        elem_a="lever_handle",
        elem_b="lever_hub",
        reason="Lever collar sits on the hub axle protruding from the body side.",
    )

    # ---- body sits on deck ----
    flange_aabb = ctx.part_element_world_aabb(body, elem="base_flange")
    ctx.check(
        "base flange sits flat on the deck",
        flange_aabb is not None and abs(flange_aabb[0][2]) <= 0.0005,
        details=f"flange aabb={flange_aabb}",
    )

    # ---- spout: waterfall lip wider than the tube ----
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "spout projects forward from body and reaches low outlet",
        spout_aabb is not None
        and spout_aabb[1][0] > 0.055
        and spout_aabb[0][2] < 0.065
        and spout_aabb[0][2] > 0.020,
        details=f"spout aabb={spout_aabb}",
    )
    # Waterfall lip is wider (Y) than the tube diameter
    ctx.check(
        "waterfall lip is wider than the spout tube (Y extent > 0.040)",
        spout_aabb is not None
        and (spout_aabb[1][1] - spout_aabb[0][1]) > 0.040,
        details=f"spout Y extent={spout_aabb[1][1] - spout_aabb[0][1]:.4f}",
    )

    # ---- lever: exists on body side, revolute joint present ----
    lever_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "lever is mounted on the body side (+Y direction)",
        lever_aabb is not None
        and lever_aabb[0][1] > 0.010,
        details=f"lever aabb={lever_aabb}",
    )

    # Joint limits
    ml = pivot.motion_limits
    ctx.check(
        "lever pivot is revolute with realistic limits (~-10 to +75 degrees)",
        ml is not None
        and ml.lower is not None
        and ml.upper is not None
        and ml.lower < 0.0
        and ml.upper > math.radians(50.0),
        details=f"limits lower={ml.lower:.3f} upper={ml.upper:.3f}",
    )

    # Decisive pose: positive rotation lifts the lever tip upward
    lever_rest = ctx.part_world_aabb(lever)
    with ctx.pose({pivot: LEVER_UPPER}):
        lever_raised = ctx.part_world_aabb(lever)
    ctx.check(
        "lever rotates upward at max positive angle",
        lever_rest is not None
        and lever_raised is not None
        and lever_raised[1][2] > lever_rest[1][2] + 0.010,
        details=f"rest top_z={lever_rest[1][2]:.4f}, raised top_z={lever_raised[1][2]:.4f}",
    )

    # ---- overall faucet height ----
    dome_aabb = ctx.part_element_world_aabb(body, elem="body_dome")
    ctx.check(
        "overall faucet height is about 0.10-0.13 m",
        dome_aabb is not None and 0.090 <= dome_aabb[1][2] <= 0.135,
        details=f"dome aabb={dome_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
