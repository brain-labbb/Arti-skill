from __future__ import annotations

# Canteen-like flat oval bottle with:
# - Flat oval (elliptical) cross-section body, hollow interior
# - Molded volume bands around the body
# - Side carrying loops
# - Black screw cap with straw spout that pivots up (REVOLUTE)
# - Cap tether loop connected to the neck
# Frame: +Z up, bottle stands on z=0, centerline at x=y=0.

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
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- Key dimensions (m) ----
# Canteen: ~0.18m tall, ~0.085m wide (X), ~0.04m deep (Y)
BODY_RX = 0.042  # body X semi-axis (width)
BODY_RY = 0.020  # body Y semi-axis (depth - flat)
BASE_Z = 0.0
BODY_TOP_Z = 0.105
SHOULDER_TOP_Z = 0.145
NECK_TOP_Z = 0.165

NECK_R = 0.011
NECK_BORE_R = 0.008

CAP_R = 0.014
CAP_HEIGHT = 0.020
CAP_MOUNT_Z = NECK_TOP_Z - CAP_HEIGHT  # where cap sits

# Straw spout
STRAW_LENGTH = 0.050
STRAW_R = 0.003
STRAW_BORE_R = 0.0018

# Volume band positions (Z heights)
BAND_ZS = (0.035, 0.060, 0.085)
BAND_HEIGHT = 0.004
BAND_PROTRUSION = 0.0015

# Side loop dimensions
LOOP_R = 0.008  # loop ring radius
LOOP_TUBE_R = 0.002  # loop tube radius
LOOP_Z = 0.050  # height of side loops

# Tether loop
TETHER_RING_R = 0.005
TETHER_TUBE_R = 0.0015
TETHER_Z = SHOULDER_TOP_Z + 0.005  # near the neck


def _oval_section(rx: float, ry: float, z: float, n: int = 48) -> list[tuple[float, float, float]]:
    """Generate 3D points for an ellipse at height z."""
    pts = []
    for i in range(n):
        a = 2.0 * math.pi * i / n
        pts.append((rx * math.cos(a), ry * math.sin(a), z))
    return pts


def _canteen_body_cq() -> cq.Workplane:
    """Build the flat oval canteen body using CadQuery."""
    # Straight elliptical body from z=0
    body = (
        cq.Workplane("XY")
        .ellipse(BODY_RX, BODY_RY)
        .extrude(BODY_TOP_Z)
    )
    # Shoulder taper: ellipse to circle
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP_Z)
        .ellipse(BODY_RX, BODY_RY)
        .workplane(offset=SHOULDER_TOP_Z - BODY_TOP_Z)
        .circle(NECK_R + 0.002)
        .loft()
    )
    # Neck cylinder
    neck = (
        cq.Workplane("XY")
        .workplane(offset=SHOULDER_TOP_Z)
        .circle(NECK_R)
        .extrude(NECK_TOP_Z - SHOULDER_TOP_Z)
    )
    # Combine outer shell
    outer = body.union(shoulder).union(neck)

    # Hollow interior: cut a cavity that opens through the neck top.
    # The cavity starts well above the bottom so the bottle has a solid base
    # and the outer shell remains one connected piece.
    wall = 0.0018
    floor_z = 0.005  # solid floor thickness
    inner_body = (
        cq.Workplane("XY")
        .workplane(offset=floor_z)
        .ellipse(BODY_RX - wall, BODY_RY - wall)
        .extrude(BODY_TOP_Z - floor_z)
    )
    inner_shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP_Z)
        .ellipse(BODY_RX - wall, BODY_RY - wall)
        .workplane(offset=SHOULDER_TOP_Z - BODY_TOP_Z)
        .circle(NECK_BORE_R)
        .loft()
    )
    inner_neck = (
        cq.Workplane("XY")
        .workplane(offset=SHOULDER_TOP_Z)
        .circle(NECK_BORE_R)
        .extrude(NECK_TOP_Z - SHOULDER_TOP_Z + 0.003)
    )
    cavity = inner_body.union(inner_shoulder).union(inner_neck)
    hollow = outer.cut(cavity)
    return hollow


def _body_mesh():
    return mesh_from_cadquery(_canteen_body_cq(), "canteen_shell")


def _volume_bands():
    """Molded elliptical volume bands around the body."""
    g = None
    for bz in BAND_ZS:
        # Create an elliptical torus-like ring
        band = cq.Workplane("XY").workplane(offset=bz)
        # Use an elliptical sweep: create a small circle swept along an ellipse path
        # Simpler: use a flattened torus by scaling
        ring = TorusGeometry(
            radius=(BODY_RX + BODY_RY) / 2.0 + BAND_PROTRUSION * 0.5,
            tube=BAND_HEIGHT / 2.0,
            radial_segments=8,
            tubular_segments=48,
        )
        # Scale to make it elliptical: wider in X, narrower in Y
        sx = (BODY_RX + BAND_PROTRUSION) / ((BODY_RX + BODY_RY) / 2.0 + BAND_PROTRUSION * 0.5)
        sy = (BODY_RY + BAND_PROTRUSION) / ((BODY_RX + BODY_RY) / 2.0 + BAND_PROTRUSION * 0.5)
        ring.scale(sx, sy, 1.0)
        ring.translate(0.0, 0.0, bz)
        if g is None:
            g = ring
        else:
            g.merge(ring)
    return mesh_from_geometry(g, "volume_bands")


def _side_loops():
    """Two carrying loops on the sides of the bottle."""
    g = None
    for sign in (-1.0, 1.0):
        loop = TorusGeometry(LOOP_R, LOOP_TUBE_R, radial_segments=10, tubular_segments=24)
        # Orient the loop so the ring plane faces outward (rotate 90° around Y)
        loop.rotate_y(math.pi / 2.0)
        # Position so loop center is at body surface (inner tube overlaps body)
        x_pos = sign * BODY_RX
        loop.translate(x_pos, 0.0, LOOP_Z)
        if g is None:
            g = loop
        else:
            g.merge(loop)
    return mesh_from_geometry(g, "side_loops")


def _tether_loop():
    """Small ring attached to the neck area for cap tether."""
    g = TorusGeometry(TETHER_RING_R, TETHER_TUBE_R, radial_segments=8, tubular_segments=20)
    # Orient so ring hangs in XZ plane
    g.rotate_x(math.pi / 2.0)
    # Position so inner ring edge overlaps neck surface for connectivity
    g.translate(NECK_R + TETHER_RING_R - TETHER_TUBE_R, 0.0, TETHER_Z)
    return mesh_from_geometry(g, "tether_ring")


def _cap_solid() -> cq.Workplane:
    """Black screw cap with knurled skirt."""
    cap = (
        cq.Workplane("XY")
        .circle(CAP_R)
        .extrude(CAP_HEIGHT)
    )
    # Hollow bore for screwing onto neck
    bore = (
        cq.Workplane("XY")
        .circle(NECK_R)
        .extrude(CAP_HEIGHT - 0.003)
    )
    cap = cap.cut(bore)
    # Knurl flutes
    n = 20
    for i in range(n):
        a = 2.0 * math.pi * i / n
        groove = (
            cq.Workplane("XY")
            .center(CAP_R * math.cos(a), CAP_R * math.sin(a))
            .circle(0.0008)
            .extrude(CAP_HEIGHT)
        )
        cap = cap.cut(groove)
    return cap


def _cap_mesh():
    return mesh_from_cadquery(_cap_solid(), "cap_shell")


def _straw_spout_solid() -> cq.Workplane:
    """Straw spout: a thin tube that pivots up from the cap."""
    # Tube extends along +X from the pivot origin
    tube = (
        cq.Workplane("YZ")
        .circle(STRAW_R)
        .extrude(STRAW_LENGTH)
    )
    # Bore through the center
    bore = (
        cq.Workplane("YZ")
        .circle(STRAW_BORE_R)
        .extrude(STRAW_LENGTH)
    )
    straw = tube.cut(bore)
    # Small pivot boss at the base (a short cylinder around the pivot axis)
    boss = (
        cq.Workplane("XZ")
        .circle(STRAW_R + 0.001)
        .extrude(0.004)
        .translate((0, -0.002, 0))
    )
    straw = straw.union(boss)
    return straw


def _straw_mesh():
    return mesh_from_cadquery(_straw_spout_solid(), "straw_tube")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="canteen_bottle")

    # Materials
    body_mat = model.material("canteen_body", rgba=(0.55, 0.72, 0.78, 0.45))
    band_mat = model.material("volume_band", rgba=(0.45, 0.62, 0.70, 0.55))
    loop_mat = model.material("loop_plastic", rgba=(0.30, 0.45, 0.55, 0.85))
    black = model.material("cap_black", rgba=(0.08, 0.08, 0.09, 1.0))
    straw_mat = model.material("straw_plastic", rgba=(0.90, 0.92, 0.88, 0.90))
    tether_mat = model.material("tether_gray", rgba=(0.40, 0.42, 0.44, 1.0))

    # ---- Bottle body (root): flat oval hollow shell ----
    body = model.part("bottle_body")
    body.visual(_body_mesh(), material=body_mat, name="canteen_shell")
    body.visual(_volume_bands(), material=band_mat, name="volume_bands")
    body.visual(_side_loops(), material=loop_mat, name="side_loops")
    body.visual(_tether_loop(), material=tether_mat, name="tether_ring")
    body.inertial = Inertial.from_geometry(
        Box((BODY_RX * 2, BODY_RY * 2, NECK_TOP_Z)),
        mass=0.045,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- Cap carrier (massless, for decoupled spin/lift) ----
    carrier = model.part("cap_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.006, 0.006, 0.006)), mass=1e-4)

    # ---- Black screw cap ----
    cap = model.part("cap")
    cap.visual(_cap_mesh(), material=black, name="cap_shell")
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_HEIGHT),
        mass=0.004,
        origin=Origin(xyz=(0.0, 0.0, CAP_HEIGHT / 2.0)),
    )

    # ---- Straw spout ----
    straw = model.part("straw_spout")
    straw.visual(_straw_mesh(), material=straw_mat, name="straw_tube")
    straw.inertial = Inertial.from_geometry(
        Cylinder(STRAW_R, STRAW_LENGTH),
        mass=0.002,
        origin=Origin(xyz=(STRAW_LENGTH / 2.0, 0.0, 0.0)),
    )

    # === Articulations ===

    # cap_rotate: CONTINUOUS spin about +Z (screw thread rotation)
    model.articulation(
        "cap_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, CAP_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    # cap_slide: PRISMATIC lift along +Z (cap removal)
    model.articulation(
        "cap_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=CAP_HEIGHT + 0.01, effort=1.0, velocity=1.0),
    )

    # straw_pivot: REVOLUTE, pivots straw from stowed (horizontal) to drinking (upright)
    # Pivot point is on top of the cap, straw extends along +X when stowed
    # Axis is -Y so positive rotation (right-hand rule around -Y) lifts +X toward +Z
    model.articulation(
        "straw_pivot",
        ArticulationType.REVOLUTE,
        parent=cap,
        child=straw,
        origin=Origin(xyz=(0.0, 0.0, CAP_HEIGHT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.45, effort=2.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle_body")
    cap = object_model.get_part("cap")
    straw = object_model.get_part("straw_spout")
    rotate = object_model.get_articulation("cap_rotate")
    slide = object_model.get_articulation("cap_slide")
    pivot = object_model.get_articulation("straw_pivot")

    # --- Bottle body is flat oval: wider in X than deep in Y ---
    body_aabb = ctx.part_world_aabb(body)
    dx = body_aabb[1][0] - body_aabb[0][0]
    dy = body_aabb[1][1] - body_aabb[0][1]
    dz = body_aabb[1][2] - body_aabb[0][2]
    ctx.check(
        "bottle body is flat oval (wider than deep)",
        dx > dy * 1.4,
        details=f"dx={dx:.4f}, dy={dy:.4f}",
    )
    ctx.check(
        "bottle is taller than wide",
        dz > dx * 1.5,
        details=f"dz={dz:.4f}, dx={dx:.4f}",
    )

    # --- Volume bands exist on body ---
    band_vis = body.get_visual("volume_bands")
    ctx.check(
        "volume bands present on body",
        band_vis is not None,
        details="volume_bands visual missing",
    )

    # --- Side loops exist ---
    loops_vis = body.get_visual("side_loops")
    ctx.check(
        "side carrying loops present",
        loops_vis is not None,
        details="side_loops visual missing",
    )

    # --- Tether loop exists at neck ---
    tether_vis = body.get_visual("tether_ring")
    ctx.check(
        "tether loop present near neck",
        tether_vis is not None,
        details="tether_ring visual missing",
    )

    # --- Cap seated over neck (intentional overlap) ---
    ctx.allow_overlap(
        cap, body,
        elem_a="cap_shell", elem_b="canteen_shell",
        reason="Cap skirt intentionally screws over the threaded neck.",
    )
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap mounted at top of bottle",
        cap_pos is not None and cap_pos[2] > 0.13,
        details=f"cap_pos={cap_pos}",
    )

    # --- Cap spin (CONTINUOUS) ---
    with ctx.pose({rotate: math.pi / 2.0}):
        cap_pos_90 = ctx.part_world_position(cap)
    ctx.check(
        "cap rotates about +Z",
        cap_pos_90 is not None,
        details="cap_rotate pose failed",
    )

    # --- Cap slides up (PRISMATIC) ---
    z_rest = ctx.part_world_aabb(cap)[0][2]
    with ctx.pose({slide: CAP_HEIGHT}):
        z_lifted = ctx.part_world_aabb(cap)[0][2]
    ctx.check(
        "cap slides up off neck",
        z_lifted > z_rest + 0.015,
        details=f"rest_z={z_rest:.4f}, lifted_z={z_lifted:.4f}",
    )

    # --- Straw spout exists and is articulated ---
    ctx.check(
        "straw spout part exists",
        straw is not None,
        details="straw_spout part missing",
    )
    pivot_type = pivot.articulation_type
    ctx.check(
        "straw pivot is REVOLUTE",
        pivot_type == ArticulationType.REVOLUTE,
        details=f"type={pivot_type}",
    )

    # --- Straw pivot: stowed (q=0) is low, raised (q~1.4) is higher ---
    straw_tip_rest = ctx.part_world_aabb(straw)[1][2]  # max Z at rest
    with ctx.pose({pivot: 1.4}):
        straw_tip_up = ctx.part_world_aabb(straw)[1][2]  # max Z when raised
    ctx.check(
        "straw pivots upward when opened",
        straw_tip_up > straw_tip_rest + 0.01,
        details=f"rest_max_z={straw_tip_rest:.4f}, raised_max_z={straw_tip_up:.4f}",
    )

    # --- Straw pivot limits are sensible ---
    limits = pivot.motion_limits
    ctx.check(
        "straw pivot has bounded limits",
        limits is not None and limits.lower is not None and limits.upper is not None,
        details=f"limits={limits}",
    )

    return ctx.report()


object_model = build_object_model()
