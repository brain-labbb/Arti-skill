from __future__ import annotations

# Screw-thread twist-off cap variant of the slim round lipstick tube.
# Frame: tube axis along +Z, base seated on z=0, cap on top.
# Construction:
#   - tube_base (root): hollow body + narrower threaded neck + silver band.
#   - cap: screw-thread cap that TWISTS off (CONTINUOUS rotation about +Z).
#     Visible external thread ridges on the neck; matching internal ridges in the cap.
#   - bullet_carrier: massless screw carrier; CONTINUOUS rotate about +Z.
#   - bullet: red lipstick bullet on a silver carrier cup; PRISMATIC rise about +Z.
# The twist-up screw chain (continuous rotate + prismatic rise) is decoupled from
# the cap screw joint; both share the +Z axis but are independent.

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
INNER_R = TUBE_R - TUBE_WALL  # 0.0089

# Threaded neck (narrower section above the shoulder)
NECK_BOTTOM_Z = 0.042    # shoulder height
NECK_TOP_Z = 0.050       # top of the neck
NECK_R = 0.0095          # neck outer radius (narrower than tube)
NECK_LEN = NECK_TOP_Z - NECK_BOTTOM_Z  # 0.008

# Thread ridge geometry
THREAD_PITCH = 0.004     # 4 mm per turn
THREAD_TURNS = 2
THREAD_RIDGE_H = 0.0006  # radial protrusion
THREAD_RIDGE_W = 0.0010  # axial width of one ridge
THREAD_START_Z = NECK_BOTTOM_Z + 0.001  # threads start 1 mm above shoulder
THREAD_TOTAL_Z = THREAD_PITCH * THREAD_TURNS  # 0.008

# Silver center band (below the shoulder, at the parting line)
BAND_Z = NECK_BOTTOM_Z - 0.004  # 0.038

# Cap
CAP_R = TUBE_R + 0.0012   # 0.0117, cap outer radius
CAP_BORE_R = 0.0103       # clears thread ridges (outer=0.0101) but stops at tube shoulder
CAP_LEN = 0.046
CAP_REST_Z = NECK_BOTTOM_Z  # cap bottom sits on the shoulder

# Bullet
BULLET_R = 0.0072
CUP_R = INNER_R - 0.0006   # 0.0083
CUP_LEN = 0.012
BULLET_LEN = 0.020
RISE_HEIGHT = 0.040


# ---- geometry helpers ----

def _tube_base_solid() -> cq.Workplane:
    """Hollow tube body with a narrower neck above the shoulder."""
    # Main body cylinder
    outer = (
        cq.Workplane("XY")
        .circle(TUBE_R)
        .extrude(NECK_BOTTOM_Z)
    )
    # Neck section
    neck = (
        cq.Workplane("XY")
        .workplane(offset=NECK_BOTTOM_Z)
        .circle(NECK_R)
        .extrude(NECK_LEN)
    )
    body = outer.union(neck)
    # Bore from above the floor through the open top
    bore = (
        cq.Workplane("XY")
        .workplane(offset=0.004)
        .circle(INNER_R)
        .extrude(NECK_TOP_Z)
    )
    return body.cut(bore)


def _external_thread_ridges() -> cq.Workplane:
    """Helical thread ridges protruding from the neck surface (twistExtrude)."""
    r_inner = NECK_R - 0.0001   # slight embed into neck for connectivity
    r_outer = NECK_R + THREAD_RIDGE_H
    hw = THREAD_RIDGE_W / 2.0
    return (
        cq.Workplane("XY")
        .workplane(offset=THREAD_START_Z)
        .moveTo(r_inner, -hw)
        .lineTo(r_outer, -hw)
        .lineTo(r_outer, hw)
        .lineTo(r_inner, hw)
        .close()
        .twistExtrude(THREAD_TOTAL_Z, THREAD_TURNS * 360.0)
    )


def _cap_shell_solid() -> cq.Workplane:
    """Cap outer shell with bore (open bottom, closed top)."""
    outer = (
        cq.Workplane("XY")
        .circle(CAP_R)
        .extrude(CAP_LEN)
    )
    bore = (
        cq.Workplane("XY")
        .circle(CAP_BORE_R)
        .extrude(CAP_LEN - 0.004)  # leaves a 4 mm closed top
    )
    return outer.cut(bore)


def _internal_thread_ridges() -> cq.Workplane:
    """Internal thread ridges protruding inward from the cap bore wall."""
    r_outer = CAP_BORE_R + 0.0001  # slight embed into bore wall for connectivity
    r_inner = CAP_BORE_R - THREAD_RIDGE_H
    hw = THREAD_RIDGE_W / 2.0
    # Half-pitch axial offset so ridges interleave with external threads
    z_start = THREAD_PITCH / 2.0 + 0.001
    return (
        cq.Workplane("XY")
        .workplane(offset=z_start)
        .moveTo(r_inner, -hw)
        .lineTo(r_outer, -hw)
        .lineTo(r_outer, hw)
        .lineTo(r_inner, hw)
        .close()
        .twistExtrude(THREAD_TOTAL_Z, THREAD_TURNS * 360.0)
    )


def _bullet_cup_solid() -> cq.Workplane:
    cup = (
        cq.Workplane("XY")
        .circle(CUP_R)
        .extrude(CUP_LEN)
    )
    return cup


def _bullet_red_solid() -> cq.Workplane:
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


# ---- model ----

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lipstick_tube_screw_cap")

    white = model.material("tube_white", rgba=(0.93, 0.93, 0.94, 1.0))
    silver = model.material("silver", rgba=(0.74, 0.75, 0.78, 1.0))
    red = model.material("lipstick_red", rgba=(0.86, 0.10, 0.10, 1.0))
    dark = model.material("marker_dark", rgba=(0.15, 0.15, 0.16, 1.0))

    # ---- tube_base (root): hollow body + threaded neck + silver band ----
    base = model.part("tube_base")
    base.visual(
        mesh_from_cadquery(_tube_base_solid(), "tube_body"),
        material=white,
        name="tube_body",
    )
    base.visual(
        mesh_from_cadquery(_external_thread_ridges(), "neck_threads"),
        material=silver,
        name="neck_threads",
    )
    base.visual(
        Cylinder(radius=TUBE_R + 0.0006, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, BAND_Z)),
        material=silver,
        name="center_band",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=TUBE_R, length=NECK_TOP_Z),
        mass=0.030,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- cap: screw-thread twist-off (CONTINUOUS rotation about +Z) ----
    cap = model.part("cap")
    cap.visual(
        mesh_from_cadquery(_cap_shell_solid(), "cap_shell"),
        material=white,
        name="cap_shell",
    )
    cap.visual(
        mesh_from_cadquery(_internal_thread_ridges(), "cap_threads"),
        material=silver,
        name="cap_threads",
    )
    # Off-axis marker so rotation is measurable
    cap.visual(
        Box((0.002, 0.002, 0.004)),
        origin=Origin(xyz=(CAP_R - 0.002, 0.0, 0.010)),
        material=dark,
        name="cap_marker",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=CAP_R, length=CAP_LEN),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.0, CAP_LEN / 2.0)),
    )
    model.articulation(
        "cap_screw",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, CAP_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.0),
    )

    # ---- bullet_carrier: massless screw carrier, CONTINUOUS rotate ----
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

    # ---- bullet: red lipstick on silver carrier cup, PRISMATIC rise ----
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
    cap_screw = object_model.get_articulation("cap_screw")
    twist = object_model.get_articulation("bullet_twist")
    rise = object_model.get_articulation("bullet_rise")

    # --- material/identity claims ---
    bullet_red = bullet.get_visual("bullet_red")
    tube_body = base.get_visual("tube_body")
    band = base.get_visual("center_band")
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
    ctx.check(
        "center band is silver",
        abs(band.material.rgba[0] - 0.74) < 0.05,
        details=f"band rgba={band.material.rgba}",
    )

    # --- thread geometry existence ---
    neck_threads = base.get_visual("neck_threads")
    cap_threads = cap.get_visual("cap_threads")
    ctx.check(
        "neck has visible external thread ridges",
        neck_threads is not None,
        details="neck_threads visual missing from tube_base",
    )
    ctx.check(
        "cap has visible internal thread ridges",
        cap_threads is not None,
        details="cap_threads visual missing from cap",
    )

    # --- cap_screw is CONTINUOUS rotation about +Z ---
    ctx.check(
        "cap_screw is continuous rotation",
        cap_screw.joint_type == ArticulationType.CONTINUOUS,
        details=f"cap_screw type={cap_screw.joint_type}",
    )

    # --- overlap allowances ---
    # Thread meshing: internal cap ridges interleave with external neck ridges
    ctx.allow_overlap(
        cap, base,
        elem_a="cap_threads", elem_b="neck_threads",
        reason="Internal cap threads mesh with external neck threads (screw-thread interface).",
    )
    # Cap shell seats on tube body shoulder and surrounds threaded neck
    ctx.allow_overlap(
        cap, base,
        elem_a="cap_shell", elem_b="tube_body",
        reason="Cap shell seats on the tube shoulder and encloses the threaded neck.",
    )
    ctx.allow_overlap(
        cap, base,
        elem_a="cap_shell", elem_b="neck_threads",
        reason="Cap shell surrounds the external thread ridges on the neck.",
    )
    # Bullet/carrier inside tube bore
    ctx.allow_overlap(
        bullet, base,
        elem_a="carrier_cup", elem_b="tube_body",
        reason="The carrier cup rides inside the tube bore (twist-up mechanism).",
    )
    ctx.allow_overlap(
        bullet, base,
        elem_a="bullet_red", elem_b="tube_body",
        reason="The red bullet starts nested inside the tube before being twisted up.",
    )
    # Cap encloses the retracted bullet when closed
    ctx.allow_overlap(
        cap, bullet,
        reason="The closed cap encloses the retracted bullet.",
    )

    # --- cap rests at the shoulder ---
    cap_rest = ctx.part_world_position(cap)
    ctx.check(
        "cap rests at the tube shoulder",
        cap_rest is not None and abs(cap_rest[2] - CAP_REST_Z) < 0.002,
        details=f"cap origin={cap_rest}, expected z={CAP_REST_Z}",
    )

    # --- cap_screw rotates the cap (off-axis marker moves) ---
    marker0 = ctx.part_element_world_aabb(cap, elem="cap_marker")
    mc0 = ((marker0[0][0] + marker0[1][0]) / 2.0, (marker0[0][1] + marker0[1][1]) / 2.0)
    with ctx.pose({cap_screw: math.pi / 2.0}):
        marker1 = ctx.part_element_world_aabb(cap, elem="cap_marker")
        mc1 = ((marker1[0][0] + marker1[1][0]) / 2.0, (marker1[0][1] + marker1[1][1]) / 2.0)
    moved = math.hypot(mc1[0] - mc0[0], mc1[1] - mc0[1])
    ctx.check(
        "cap_screw rotates the cap (marker moves)",
        moved > 0.003,
        details=f"marker rest={mc0}, rotated={mc1}, moved={moved}",
    )

    # --- twisting bullet_twist spins the bullet ---
    bmarker0 = ctx.part_element_world_aabb(bullet, elem="twist_marker")
    bmc0 = ((bmarker0[0][0] + bmarker0[1][0]) / 2.0, (bmarker0[0][1] + bmarker0[1][1]) / 2.0)
    with ctx.pose({twist: math.pi / 2.0}):
        bmarker1 = ctx.part_element_world_aabb(bullet, elem="twist_marker")
        bmc1 = ((bmarker1[0][0] + bmarker1[1][0]) / 2.0, (bmarker1[0][1] + bmarker1[1][1]) / 2.0)
    bmoved = math.hypot(bmc1[0] - bmc0[0], bmc1[1] - bmc0[1])
    ctx.check(
        "twist spins the bullet (marker moves)",
        bmoved > 0.003,
        details=f"marker rest={bmc0}, twisted={bmc1}, moved={bmoved}",
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
        "raised bullet emerges above the neck top",
        bull_top_z > NECK_TOP_Z + 0.005,
        details=f"bullet top z when raised={bull_top_z}",
    )

    return ctx.report()


object_model = build_object_model()
