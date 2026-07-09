from __future__ import annotations

# Flip-cap lipstick tube: round body with a hinged flip-cap (replaces pull-off).
# Frame: tube axis along +Z, base seated on z=0, cap hinges from the rear rim.
# Construction:
#   - tube_base (root): slim hollow tubular body with silver band + hinge lug.
#   - cap: flip-cap on a lateral revolute hinge at the rear top rim of the base.
#   - bullet_carrier: massless screw carrier; CONTINUOUS rotate about +Z.
#   - bullet: red lipstick bullet on a silver carrier cup; PRISMATIC rise about +Z.
# The twist-up screw chain is unchanged from the parent.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    mesh_from_cadquery,
)

# --- key dimensions (meters) ---
TUBE_R = 0.0105          # outer radius of the slim tube
TUBE_WALL = 0.0016
TUBE_BOTTOM_Z = 0.0
BASE_TOP_Z = 0.050       # top rim of the lower base section
BAND_Z = 0.0455          # center silver band height
INNER_R = TUBE_R - TUBE_WALL

CAP_R = TUBE_R + 0.0008  # cap sits on the base rim, slight clearance
CAP_LEN = 0.046          # cap height
CAP_WALL = 0.0014

BULLET_R = 0.0072        # red bullet radius
CUP_R = INNER_R - 0.0006 # silver carrier cup
CUP_LEN = 0.012
BULLET_LEN = 0.020       # straight portion below the angled tip
RISE_HEIGHT = 0.040      # twist-up travel

# Hinge geometry
HINGE_Y = -TUBE_R        # hinge at rear rim of the base
HINGE_Z = BASE_TOP_Z     # hinge at the top of the base
LUG_W = 0.008            # hinge lug width (X)
LUG_D = 0.004            # hinge lug depth (Y, protrudes outward)
LUG_H = 0.008            # hinge lug height (Z, protrudes above rim)
KNUCKLE_R = 0.0028       # hinge knuckle radius on cap
KNUCKLE_LEN = 0.007      # hinge knuckle length along X


def _tube_base_solid() -> cq.Workplane:
    """Hollow open-top tube: outer wall minus a bore that reaches the open top."""
    outer = (
        cq.Workplane("XY")
        .circle(TUBE_R)
        .extrude(BASE_TOP_Z)
    )
    bore = (
        cq.Workplane("XY")
        .workplane(offset=0.004)
        .circle(INNER_R)
        .extrude(BASE_TOP_Z)
    )
    return outer.cut(bore)


def _cap_shell_solid() -> cq.Workplane:
    """Hollow cap shell: open at bottom, closed at top."""
    outer = (
        cq.Workplane("XY")
        .circle(CAP_R)
        .extrude(CAP_LEN)
    )
    bore = (
        cq.Workplane("XY")
        .circle(CAP_R - CAP_WALL)
        .extrude(CAP_LEN - 0.004)  # leaves a closed top
    )
    return outer.cut(bore)


def _bullet_cup_solid() -> cq.Workplane:
    """Silver carrier cup that rides inside the tube bore."""
    return (
        cq.Workplane("XY")
        .circle(CUP_R)
        .extrude(CUP_LEN)
    )


def _bullet_red_solid() -> cq.Workplane:
    """Red lipstick column with an angled cut tip."""
    col_bottom = CUP_LEN
    column = (
        cq.Workplane("XY")
        .workplane(offset=col_bottom)
        .circle(BULLET_R)
        .extrude(BULLET_LEN)
    )
    tip = (
        cq.Workplane("XY")
        .workplane(offset=col_bottom + BULLET_LEN)
        .circle(BULLET_R)
        .extrude(0.010)
    )
    cutter = (
        cq.Workplane("XY")
        .workplane(offset=col_bottom + BULLET_LEN)
        .transformed(rotate=(20.0, 0.0, 0.0))
        .rect(0.05, 0.05)
        .extrude(0.03)
        .translate((0.0, 0.0, 0.004))
    )
    tip = tip.cut(cutter)
    return column.union(tip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lipstick_flip_cap")

    white = model.material("tube_white", rgba=(0.93, 0.93, 0.94, 1.0))
    silver = model.material("silver", rgba=(0.74, 0.75, 0.78, 1.0))
    red = model.material("lipstick_red", rgba=(0.86, 0.10, 0.10, 1.0))
    dark = model.material("marker_dark", rgba=(0.15, 0.15, 0.16, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.32, 0.33, 0.36, 1.0))

    # ---- tube_base (root): hollow body + silver band + hinge lug ----
    base = model.part("tube_base")
    base.visual(
        mesh_from_cadquery(_tube_base_solid(), "tube_body"),
        material=white,
        name="tube_body",
    )
    # Silver center band
    base.visual(
        Cylinder(radius=TUBE_R + 0.0006, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, BAND_Z)),
        material=silver,
        name="center_band",
    )
    # Hinge lug: visible boss at the rear top rim where the cap pivots
    base.visual(
        Box((LUG_W, LUG_D, LUG_H)),
        origin=Origin(xyz=(0.0, -(TUBE_R + LUG_D / 2.0 - 0.001), BASE_TOP_Z + LUG_H / 2.0)),
        material=gunmetal,
        name="hinge_lug",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=TUBE_R, length=BASE_TOP_Z),
        mass=0.030,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z / 2.0)),
    )

    # ---- cap: flip-cap on a lateral revolute hinge ----
    cap = model.part("cap")
    # Cap shell: hollow cup, open bottom, closed top.
    # Built with bottom at z=0, centered at y=0 in geometry space.
    # Visual origin offsets it so bottom sits at hinge z and center at tube axis.
    cap.visual(
        mesh_from_cadquery(_cap_shell_solid(), "cap_shell"),
        origin=Origin(xyz=(0.0, TUBE_R, 0.0)),
        material=white,
        name="cap_shell",
    )
    # Hinge knuckle: small cylinder around the hinge pin axis (X direction)
    cap.visual(
        Cylinder(radius=KNUCKLE_R, length=KNUCKLE_LEN),
        origin=Origin(xyz=(0.0, 0.0, LUG_H * 0.4), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="hinge_knuckle",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=CAP_R, length=CAP_LEN),
        mass=0.012,
        origin=Origin(xyz=(0.0, TUBE_R, CAP_LEN / 2.0)),
    )

    # Cap flip articulation: REVOLUTE at the rear top rim of the base.
    # Axis along +X (lateral), positive q swings the cap up and backward.
    model.articulation(
        "cap_flip",
        ArticulationType.REVOLUTE,
        parent=base,
        child=cap,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=2.4),
    )

    # ---- bullet_carrier: massless screw carrier, CONTINUOUS rotate about +Z ----
    carrier = model.part("bullet_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.004, 0.004, 0.004)), mass=1e-4)
    model.articulation(
        "bullet_twist",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    # ---- bullet: red lipstick on a silver carrier cup, PRISMATIC rise about +Z ----
    bullet = model.part("bullet")
    bullet.visual(
        mesh_from_cadquery(_bullet_cup_solid(), "carrier_cup"),
        material=silver,
        name="carrier_cup",
    )
    bullet.visual(
        mesh_from_cadquery(_bullet_red_solid(), "bullet_red"),
        material=red,
        name="bullet_red",
    )
    # Off-axis twist marker
    bullet.visual(
        Box((0.0016, 0.0016, 0.004)),
        origin=Origin(xyz=(CUP_R - 0.0006, 0.0, CUP_LEN / 2.0)),
        material=dark,
        name="twist_marker",
    )
    bullet.inertial = Inertial.from_geometry(
        Cylinder(radius=BULLET_R, length=CUP_LEN + BULLET_LEN),
        mass=0.004,
        origin=Origin(xyz=(0.0, 0.0, (CUP_LEN + BULLET_LEN) / 2.0)),
    )
    model.articulation(
        "bullet_rise",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=bullet,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=0.0, upper=RISE_HEIGHT),
    )

    return model


def run_tests():
    from sdk import TestContext

    ctx = TestContext(object_model)

    base = object_model.get_part("tube_base")
    cap = object_model.get_part("cap")
    carrier = object_model.get_part("bullet_carrier")
    bullet = object_model.get_part("bullet")
    cap_flip = object_model.get_articulation("cap_flip")
    twist = object_model.get_articulation("bullet_twist")
    rise = object_model.get_articulation("bullet_rise")

    # --- structural claims ---
    # 1. cap_flip is REVOLUTE (not prismatic like the parent)
    ctx.check(
        "cap joint is revolute (flip hinge)",
        cap_flip.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={cap_flip.articulation_type}",
    )
    # 2. Hinge axis is lateral (dominant X component)
    ax = cap_flip.axis
    ctx.check(
        "hinge axis is lateral (X-dominant)",
        abs(ax[0]) > 0.9 and abs(ax[1]) < 0.1 and abs(ax[2]) < 0.1,
        details=f"axis={ax}",
    )
    # 3. Hinge origin is at the rear top rim of the base
    ho = cap_flip.origin
    ctx.check(
        "hinge origin at rear top rim",
        ho.xyz[2] > BASE_TOP_Z - 0.003 and ho.xyz[1] < -0.005,
        details=f"hinge origin xyz={ho.xyz}",
    )

    # --- visible hinge lug on base ---
    lug = base.get_visual("hinge_lug")
    ctx.check(
        "hinge lug exists on tube_base",
        lug is not None,
        details="no hinge_lug visual found",
    )

    # --- hinge knuckle on cap ---
    knuckle = cap.get_visual("hinge_knuckle")
    ctx.check(
        "hinge knuckle exists on cap",
        knuckle is not None,
        details="no hinge_knuckle visual found",
    )

    # --- material/identity claims ---
    bullet_red = bullet.get_visual("bullet_red")
    tube_body = base.get_visual("tube_body")
    ctx.check(
        "bullet is red",
        tuple(bullet_red.material.rgba[:3]) == (0.86, 0.10, 0.10),
        details=f"bullet rgba={bullet_red.material.rgba}",
    )
    ctx.check(
        "tube body is white/silver (light)",
        min(tube_body.material.rgba[:3]) > 0.7,
        details=f"tube rgba={tube_body.material.rgba}",
    )

    # --- overlap allowances ---
    # Cap knuckle overlaps hinge lug at the pivot (intentional hinge embedding)
    ctx.allow_overlap(
        cap,
        base,
        elem_a="hinge_knuckle",
        elem_b="hinge_lug",
        reason="The hinge knuckle wraps around the hinge lug at the pivot point.",
    )
    # Cap shell may slightly contact base rim when closed
    ctx.allow_overlap(
        cap,
        base,
        elem_a="cap_shell",
        elem_b="tube_body",
        reason="The flip-cap shell seats onto the tube rim when closed.",
    )
    # Bullet/carrier nested inside the tube bore
    ctx.allow_overlap(
        bullet,
        base,
        elem_a="carrier_cup",
        elem_b="tube_body",
        reason="The carrier cup rides inside the tube bore (twist-up mechanism).",
    )
    ctx.allow_overlap(
        bullet,
        base,
        elem_a="bullet_red",
        elem_b="tube_body",
        reason="The red bullet starts nested inside the tube before being twisted up.",
    )
    # Cap encloses retracted bullet when closed
    ctx.allow_overlap(
        cap,
        bullet,
        reason="The closed cap encloses the retracted bullet.",
    )

    # --- cap covers the tube when closed (q=0) ---
    cap_shell_rest = ctx.part_element_world_aabb(cap, elem="cap_shell")
    ctx.check(
        "cap shell covers tube top when closed",
        cap_shell_rest is not None and cap_shell_rest[0][2] > BASE_TOP_Z - 0.005,
        details=f"cap shell rest aabb min={cap_shell_rest[0]}, max={cap_shell_rest[1]}",
    )
    # Cap shell center Y should be near 0 (over the tube) when closed
    rest_center_y = (cap_shell_rest[0][1] + cap_shell_rest[1][1]) / 2.0
    ctx.check(
        "closed cap shell is centered over tube",
        abs(rest_center_y) < 0.005,
        details=f"cap shell center Y={rest_center_y}",
    )

    # --- cap flips open (swings backward on the hinge) ---
    with ctx.pose({cap_flip: 2.0}):
        cap_shell_open = ctx.part_element_world_aabb(cap, elem="cap_shell")
        cap_open_aabb = ctx.part_world_aabb(cap)
    open_center_y = (cap_shell_open[0][1] + cap_shell_open[1][1]) / 2.0
    open_center_z = (cap_shell_open[0][2] + cap_shell_open[1][2]) / 2.0
    # Cap shell center should swing to the -Y side (behind the tube)
    ctx.check(
        "cap flips open (shell swings backward)",
        open_center_y < rest_center_y - 0.010,
        details=f"rest center Y={rest_center_y}, open center Y={open_center_y}",
    )
    # At open pose, the tube top opening should be exposed
    ctx.check(
        "open cap exposes tube top",
        cap_shell_open[0][1] < -TUBE_R - 0.002  # cap shell swung behind the tube
        or cap_shell_open[0][2] > BASE_TOP_Z + 0.010,  # or cap is well above rim
        details=f"open shell aabb min={cap_shell_open[0]}, max={cap_shell_open[1]}",
    )

    # --- twisting bullet_twist spins the bullet ---
    marker0 = ctx.part_element_world_aabb(bullet, elem="twist_marker")
    mc0 = ((marker0[0][0] + marker0[1][0]) / 2.0, (marker0[0][1] + marker0[1][1]) / 2.0)
    with ctx.pose({twist: math.pi / 2.0}):
        marker1 = ctx.part_element_world_aabb(bullet, elem="twist_marker")
        mc1 = ((marker1[0][0] + marker1[1][0]) / 2.0, (marker1[0][1] + marker1[1][1]) / 2.0)
    moved = math.hypot(mc1[0] - mc0[0], mc1[1] - mc0[1])
    ctx.check(
        "twist spins the bullet (marker moves)",
        moved > 0.003,
        details=f"marker rest={mc0}, twisted={mc1}, moved={moved}",
    )

    # --- bullet_rise raises the bullet out of the tube ---
    bull_rest = ctx.part_world_position(bullet)
    with ctx.pose({rise: RISE_HEIGHT}):
        bull_top_aabb = ctx.part_world_aabb(bullet)
        bull_up = ctx.part_world_position(bullet)
        bull_top_z = bull_top_aabb[1][2]
    ctx.check(
        "twist-up raises the bullet",
        bull_up is not None and bull_up[2] > bull_rest[2] + 0.030,
        details=f"rest={bull_rest}, raised={bull_up}",
    )
    ctx.check(
        "raised bullet emerges above the tube top",
        bull_top_z > BASE_TOP_Z + 0.005,
        details=f"bullet top z when raised={bull_top_z}",
    )

    return ctx.report()


object_model = build_object_model()
