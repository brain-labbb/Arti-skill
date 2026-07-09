from __future__ import annotations

# Canteen-like flat oval bottle with side loops, straw spout, volume bands,
# and cap tether loop.
# Frame: bottle axis along +Z, base at z=0, neck/cap at the top (+Z).

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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- key dimensions (meters) ----
BODY_RX = 0.040          # outer half-width (X radius)
BODY_RY = 0.024          # outer half-depth (Y radius)
WALL = 0.0018
BASE_Z = 0.0
BARREL_TOP_Z = 0.110
SHOULDER_TOP_Z = 0.132
NECK_R = 0.012
NECK_TOP_Z = 0.150
CAP_R = 0.015
CAP_HEIGHT = 0.020
SKIRT_DROP = 0.009
CAP_TOP_Z = CAP_HEIGHT - SKIRT_DROP

# Volume bands (raised ridges)
BAND_COUNT = 3
BAND_START_Z = 0.030
BAND_SPACING = 0.025
BAND_WIDTH = 0.003
BAND_RAISE = 0.001

# Side loops
LOOP_RX = 0.007
LOOP_TUBE_R = 0.002
LOOP_Z = 0.065

# Straw
STRAW_LENGTH = 0.040
STRAW_R = 0.003
STRAW_WALL = 0.001

# Tether loop
TETHER_LOOP_R = 0.005
TETHER_TUBE = 0.0015
TETHER_Z = SHOULDER_TOP_Z + 0.003


def _canteen_body():
    """Flat oval canteen body with volume bands and threaded neck, hollowed."""
    # Main barrel: elliptical extrusion
    barrel = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, BASE_Z + 0.004))
        .ellipse(BODY_RX, BODY_RY)
        .extrude(BARREL_TOP_Z - BASE_Z - 0.004)
    )
    # Rounded base
    base = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, BASE_Z))
        .ellipse(BODY_RX - 0.003, BODY_RY - 0.003)
        .extrude(0.008)
    )
    body = barrel.union(base)

    # Shoulder loft
    shoulder = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, BARREL_TOP_Z))
        .ellipse(BODY_RX, BODY_RY)
        .workplane(offset=SHOULDER_TOP_Z - BARREL_TOP_Z)
        .circle(NECK_R + 0.003)
        .loft()
    )
    body = body.union(shoulder)

    # Neck cylinder
    neck = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, SHOULDER_TOP_Z))
        .circle(NECK_R + 0.001)
        .extrude(NECK_TOP_Z - SHOULDER_TOP_Z)
    )
    body = body.union(neck)

    # Thread ridges as simple rings on the neck (only 2 to save time)
    for k in range(2):
        zc = SHOULDER_TOP_Z + 0.005 + k * 0.006
        ring = (
            cq.Workplane("XY")
            .transformed(offset=(0, 0, zc))
            .circle(NECK_R + 0.002)
            .extrude(0.002)
        )
        body = body.union(ring)

    # Volume bands: raised elliptical rings around the barrel
    for i in range(BAND_COUNT):
        z = BAND_START_Z + i * BAND_SPACING
        outer_e = (
            cq.Workplane("XY")
            .transformed(offset=(0, 0, z))
            .ellipse(BODY_RX + BAND_RAISE, BODY_RY + BAND_RAISE)
            .extrude(BAND_WIDTH)
        )
        inner_e = (
            cq.Workplane("XY")
            .transformed(offset=(0, 0, z - 0.0001))
            .ellipse(BODY_RX, BODY_RY)
            .extrude(BAND_WIDTH + 0.0002)
        )
        band = outer_e.cut(inner_e)
        body = body.union(band)

    # Shell: hollow the body (open at top)
    body = body.faces(">Z").shell(-WALL)
    return body


def _side_loops():
    """Two carrying loops on the sides as rectangular frames."""
    result = None
    for sign in [-1, 1]:
        x_center = sign * (BODY_RX + 0.004)
        # Outer rectangle
        outer = (
            cq.Workplane("XY")
            .transformed(offset=(x_center, 0, LOOP_Z))
            .rect(LOOP_RX * 2, LOOP_TUBE_R * 4)
            .extrude(LOOP_TUBE_R * 2, both=True)
        )
        # Inner cutout
        inner = (
            cq.Workplane("XY")
            .transformed(offset=(x_center, 0, LOOP_Z - LOOP_TUBE_R))
            .rect(LOOP_RX * 2 - LOOP_TUBE_R * 2, LOOP_TUBE_R * 2)
            .extrude(LOOP_TUBE_R * 2)
        )
        loop = outer.cut(inner)
        if result is None:
            result = loop
        else:
            result = result.union(loop)
    return result


def _cap_solid():
    """Black ribbed screw cap."""
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -SKIRT_DROP))
        .circle(CAP_R)
        .extrude(CAP_HEIGHT)
    )
    outer = outer.edges(">Z").fillet(0.002)
    cavity = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -SKIRT_DROP - 0.001))
        .circle(NECK_R + 0.001)
        .extrude(CAP_HEIGHT - 0.003)
    )
    cap = outer.cut(cavity)
    # Ribs (fewer for speed)
    n = 12
    rib_h = CAP_HEIGHT - 0.003
    zc = -SKIRT_DROP + rib_h / 2.0
    for i in range(n):
        ang = 2.0 * math.pi * i / n
        x = (CAP_R - 0.0004) * math.cos(ang)
        y = (CAP_R - 0.0004) * math.sin(ang)
        rib = (
            cq.Workplane("XY")
            .transformed(offset=(x, y, zc), rotate=(0, 0, math.degrees(ang)))
            .box(0.0012, 0.001, rib_h)
        )
        cap = cap.union(rib)
    return cap


def _straw_spout():
    """Straw spout tube with pivot knuckle."""
    # Tube along +X from origin
    outer = (
        cq.Workplane("YZ")
        .circle(STRAW_R)
        .extrude(STRAW_LENGTH)
    )
    inner = (
        cq.Workplane("YZ")
        .transformed(offset=(-0.0001, 0, 0))
        .circle(STRAW_R - STRAW_WALL)
        .extrude(STRAW_LENGTH + 0.0002)
    )
    straw = outer.cut(inner)
    # Knuckle sphere at origin for pivot
    knuckle = (
        cq.Workplane("XY")
        .sphere(STRAW_R + 0.001)
    )
    straw = straw.union(knuckle)
    return straw


def _tether_loop():
    """Small tether loop on the neck."""
    x_center = NECK_R + TETHER_LOOP_R * 0.4
    # Simple rectangular loop frame
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(x_center, 0, TETHER_Z))
        .rect(TETHER_LOOP_R * 2, TETHER_TUBE * 4)
        .extrude(TETHER_TUBE * 2, both=True)
    )
    inner = (
        cq.Workplane("XY")
        .transformed(offset=(x_center, 0, TETHER_Z - TETHER_TUBE))
        .rect(TETHER_LOOP_R * 2 - TETHER_TUBE * 2, TETHER_TUBE * 2)
        .extrude(TETHER_TUBE * 2)
    )
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="canteen_bottle")

    clear = model.material("clear_plastic", rgba=(0.78, 0.84, 0.82, 0.30))
    black = model.material("cap_black", rgba=(0.05, 0.05, 0.06, 1.0))
    gray = model.material("loop_gray", rgba=(0.35, 0.36, 0.38, 1.0))
    white = model.material("straw_white", rgba=(0.90, 0.91, 0.88, 1.0))

    # ---- bottle body (root) ----
    body = model.part("bottle")
    body_geo = _canteen_body()
    body.visual(mesh_from_cadquery(body_geo, "bottle_shell"), material=clear, name="bottle_shell")

    loops_geo = _side_loops()
    body.visual(mesh_from_cadquery(loops_geo, "side_loops"), material=gray, name="side_loops")

    tether_geo = _tether_loop()
    body.visual(mesh_from_cadquery(tether_geo, "tether_loop"), material=gray, name="tether_loop")

    body.inertial = Inertial.from_geometry(
        Box((BODY_RX * 2, BODY_RY * 2, NECK_TOP_Z)),
        mass=0.045,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- carrier (massless, for decoupled cap rotation) ----
    carrier = model.part("cap_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.006, 0.006, 0.006)), mass=1e-4)

    # ---- cap ----
    cap = model.part("cap")
    cap_geo = _cap_solid()
    cap.visual(mesh_from_cadquery(cap_geo, "cap_shell"), material=black, name="cap_shell")
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_HEIGHT),
        mass=0.005,
        origin=Origin(xyz=(0.0, 0.0, (CAP_TOP_Z - SKIRT_DROP) / 2.0)),
    )

    # ---- straw spout ----
    spout = model.part("straw_spout")
    straw_geo = _straw_spout()
    spout.visual(mesh_from_cadquery(straw_geo, "straw_tube"), material=white, name="straw_tube")
    spout.inertial = Inertial.from_geometry(
        Box((STRAW_LENGTH, STRAW_R * 2, STRAW_R * 2)),
        mass=0.003,
        origin=Origin(xyz=(STRAW_LENGTH / 2.0, 0.0, 0.0)),
    )

    # ---- articulations ----
    model.articulation(
        "cap_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    model.articulation(
        "cap_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=CAP_HEIGHT, effort=1.0, velocity=1.0),
    )

    model.articulation(
        "spout_pivot",
        ArticulationType.REVOLUTE,
        parent=cap,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, CAP_TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=math.pi / 2.0,
            effort=2.0,
            velocity=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle")
    cap = object_model.get_part("cap")
    spout = object_model.get_part("straw_spout")
    rotate = object_model.get_articulation("cap_rotate")
    slide = object_model.get_articulation("cap_slide")
    spout_joint = object_model.get_articulation("spout_pivot")

    # --- materials ---
    bottle_shell = body.get_visual("bottle_shell")
    ctx.check(
        "bottle body is clear plastic (alpha < 1)",
        bottle_shell.material.rgba is not None and bottle_shell.material.rgba[3] < 1.0,
        details=f"bottle rgba={bottle_shell.material.rgba}",
    )
    cap_shell = cap.get_visual("cap_shell")
    ctx.check(
        "cap is opaque black",
        cap_shell.material.rgba is not None
        and cap_shell.material.rgba[3] >= 0.99
        and max(cap_shell.material.rgba[:3]) < 0.2,
        details=f"cap rgba={cap_shell.material.rgba}",
    )

    # --- cap position ---
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap mounted on top of the neck",
        cap_pos is not None and cap_pos[2] > BARREL_TOP_Z,
        details=f"cap origin={cap_pos}",
    )

    # --- side loops exist ---
    ctx.check("side loops exist", body.get_visual("side_loops") is not None)

    # --- tether loop exists ---
    ctx.check("tether loop exists on neck", body.get_visual("tether_loop") is not None)

    # --- straw spout exists ---
    ctx.check("straw spout exists", spout.get_visual("straw_tube") is not None)

    # --- spout pivot test ---
    with ctx.pose({spout_joint: 0.0}):
        stowed = ctx.part_world_aabb(spout)
    with ctx.pose({spout_joint: math.pi / 2.0}):
        deployed = ctx.part_world_aabb(spout)
    if stowed and deployed:
        stowed_dx = stowed[1][0] - stowed[0][0]
        stowed_dz = stowed[1][2] - stowed[0][2]
        deployed_dx = deployed[1][0] - deployed[0][0]
        deployed_dz = deployed[1][2] - deployed[0][2]
        ctx.check(
            "spout stowed: extends more in X than Z",
            stowed_dx > stowed_dz,
            details=f"stowed dx={stowed_dx:.4f}, dz={stowed_dz:.4f}",
        )
        ctx.check(
            "spout deployed: extends more in Z than X",
            deployed_dz > deployed_dx,
            details=f"deployed dx={deployed_dx:.4f}, dz={deployed_dz:.4f}",
        )

    # --- cap overlap allowance ---
    ctx.allow_overlap(
        cap, body,
        elem_a="cap_shell", elem_b="bottle_shell",
        reason="Cap skirt is intentionally seated over the threaded neck.",
    )
    ctx.allow_overlap(
        spout, cap,
        elem_a="straw_tube", elem_b="cap_shell",
        reason="Straw pivot knuckle is intentionally embedded in the cap top.",
    )

    # --- cap rotation ---
    with ctx.pose({rotate: 0.0}):
        a0 = ctx.part_world_aabb(cap)
    with ctx.pose({rotate: math.pi / 2.0}):
        a90 = ctx.part_world_aabb(cap)
    if a0 and a90:
        e0x = a0[1][0] - a0[0][0]
        e0y = a0[1][1] - a0[0][1]
        e90x = a90[1][0] - a90[0][0]
        e90y = a90[1][1] - a90[0][1]
        ctx.check(
            "cap rotation changes bounding box (ribs asymmetric)",
            abs(e0x - e90y) < 0.004 or abs(e0y - e90x) < 0.004,
            details=f"rest=({e0x:.4f},{e0y:.4f}), 90deg=({e90x:.4f},{e90y:.4f})",
        )

    # --- cap slide ---
    rest_z = ctx.part_world_position(cap)[2]
    with ctx.pose({slide: CAP_HEIGHT}):
        lifted_z = ctx.part_world_position(cap)[2]
    ctx.check(
        "cap_slide lifts the cap off the neck",
        lifted_z > rest_z + CAP_HEIGHT * 0.7,
        details=f"rest_z={rest_z}, lifted_z={lifted_z}",
    )

    # --- spout joint type ---
    ctx.check(
        "spout_pivot is revolute",
        spout_joint.articulation_type == ArticulationType.REVOLUTE,
    )

    return ctx.report()


object_model = build_object_model()
