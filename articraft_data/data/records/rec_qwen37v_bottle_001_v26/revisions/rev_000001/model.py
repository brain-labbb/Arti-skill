from __future__ import annotations

# Canteen-style flat oval bottle with side loops, safety collar, volume bands,
# and raised spiral-like neck threads. Frame: bottle axis along +Z, base at
# z=0, neck/cap at top (+Z). The black ribbed cap rides on the neck through a
# massless carrier with decoupled CONTINUOUS rotation and PRISMATIC lift.
# A safety collar ring rotates independently around the neck base.

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
BODY_RX = 0.040          # semi-axis X (wide direction of oval)
BODY_RY = 0.020          # semi-axis Y (narrow/flat direction)
WALL = 0.0018
BASE_Z = 0.0
BARREL_TOP_Z = 0.100
SHOULDER_TOP_Z = 0.124
NECK_R = 0.014
NECK_TOP_Z = 0.148
CAP_R = 0.0185
CAP_HEIGHT = 0.024

# Volume bands (molded ridges around the body)
BAND_Z_LIST = [0.032, 0.056, 0.080]
BAND_WIDTH = 0.004
BAND_PROTRUSION = 0.0013

# Side loops (attachment brackets)
LOOP_Z = 0.090
LOOP_MAJOR_R = 0.007
LOOP_MINOR_R = 0.0018

# Safety collar
COLLAR_Z0 = SHOULDER_TOP_Z + 0.001
COLLAR_HEIGHT = 0.006
COLLAR_OUTER_R = NECK_R + 0.004

# Cap layout
SKIRT_DROP = 0.012
CAP_TOP_Z = CAP_HEIGHT - SKIRT_DROP

# Neck threads (raised spiral-like ridges)
THREAD_DEPTH = 0.0018
THREAD_WIDTH = 0.0018
THREAD_SWEEP_DEG = 160
THREAD_RIDGE_COUNT = 3


def _build_bottle_body():
    """Flat oval canteen body with volume bands, side loops, and thread ridges."""
    # Rounded base: loft from smaller bottom ellipse to full barrel ellipse
    base_loft = (
        cq.Workplane("XY")
        .ellipse(BODY_RX - 0.005, BODY_RY - 0.004)
        .workplane(offset=0.008)
        .ellipse(BODY_RX, BODY_RY)
        .loft()
    )

    # Main barrel: elliptical cylinder
    barrel = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, 0.008))
        .ellipse(BODY_RX, BODY_RY)
        .extrude(BARREL_TOP_Z - 0.008)
    )

    # Shoulder: loft from barrel ellipse to neck circle
    shoulder_h = SHOULDER_TOP_Z - BARREL_TOP_Z
    shoulder = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, BARREL_TOP_Z))
        .ellipse(BODY_RX, BODY_RY)
        .workplane(offset=shoulder_h)
        .circle(NECK_R)
        .loft()
    )

    # Neck cylinder
    neck_h = NECK_TOP_Z - SHOULDER_TOP_Z
    neck = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, SHOULDER_TOP_Z))
        .circle(NECK_R)
        .extrude(neck_h)
    )

    body = base_loft.union(barrel).union(shoulder).union(neck)

    # Shell to make hollow (open at top)
    body = body.faces(">Z").shell(-WALL)

    # Volume bands: raised elliptical rings on the barrel exterior
    for bz in BAND_Z_LIST:
        band_outer = (
            cq.Workplane("XY")
            .transformed(offset=(0, 0, bz - BAND_WIDTH / 2))
            .ellipse(BODY_RX + BAND_PROTRUSION, BODY_RY + BAND_PROTRUSION)
            .extrude(BAND_WIDTH)
        )
        band_cut = (
            cq.Workplane("XY")
            .transformed(offset=(0, 0, bz - BAND_WIDTH / 2 - 0.001))
            .ellipse(BODY_RX - 0.0005, BODY_RY - 0.0005)
            .extrude(BAND_WIDTH + 0.002)
        )
        band = band_outer.cut(band_cut)
        body = body.union(band)

    # Spiral-like neck thread ridges: partial arcs staggered in angle
    thread_z0 = SHOULDER_TOP_Z + 0.005
    for k in range(THREAD_RIDGE_COUNT):
        zc = thread_z0 + k * 0.005
        # Create a rectangular profile in XZ plane and revolve partially around Z
        ridge = (
            cq.Workplane("XZ")
            .moveTo(NECK_R + THREAD_DEPTH * 0.4, zc)
            .rect(THREAD_DEPTH, THREAD_WIDTH)
            .revolve(THREAD_SWEEP_DEG, (0, 0), (0, 1))
        )
        # Rotate each ridge to a different start angle for spiral effect
        if k > 0:
            ridge = ridge.rotate((0, 0, 0), (0, 0, 1), k * 120)
        body = body.union(ridge)

    # Side loops: D-ring brackets on ±X sides near shoulder
    for sign in [-1, 1]:
        cx = sign * (BODY_RX + 0.001)
        # Upper attachment post
        post_up = (
            cq.Workplane("XY")
            .transformed(offset=(cx, 0, LOOP_Z + 0.006))
            .box(0.005, 0.005, 0.004)
        )
        # Lower attachment post
        post_dn = (
            cq.Workplane("XY")
            .transformed(offset=(cx, 0, LOOP_Z - 0.006))
            .box(0.005, 0.005, 0.004)
        )
        # Outer connecting bar (the loop part)
        outer_bar = (
            cq.Workplane("XY")
            .transformed(offset=(cx + sign * 0.006, 0, LOOP_Z))
            .box(0.003, 0.005, 0.016)
        )
        # Top connector
        top_conn = (
            cq.Workplane("XY")
            .transformed(offset=(cx + sign * 0.003, 0, LOOP_Z + 0.007))
            .box(0.008, 0.005, 0.003)
        )
        # Bottom connector
        bot_conn = (
            cq.Workplane("XY")
            .transformed(offset=(cx + sign * 0.003, 0, LOOP_Z - 0.007))
            .box(0.008, 0.005, 0.003)
        )
        body = body.union(post_up).union(post_dn).union(outer_bar).union(top_conn).union(bot_conn)

    return body


def _build_cap():
    """Black ribbed screw cap (same structure as original)."""
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -SKIRT_DROP))
        .circle(CAP_R)
        .extrude(CAP_HEIGHT)
    )
    outer = outer.edges(">Z").fillet(0.0025)
    cavity = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -SKIRT_DROP - 0.001))
        .circle(NECK_R + 0.0016)
        .extrude(CAP_HEIGHT - 0.004)
    )
    cap = outer.cut(cavity)
    # Vertical knurl ribs
    n = 24
    rib_h = CAP_HEIGHT - 0.004
    zc = -SKIRT_DROP + rib_h / 2.0
    for i in range(n):
        ang = 2.0 * math.pi * i / n
        x = (CAP_R - 0.0006) * math.cos(ang)
        y = (CAP_R - 0.0006) * math.sin(ang)
        rib = (
            cq.Workplane("XY")
            .transformed(offset=(x, y, zc), rotate=(0, 0, math.degrees(ang)))
            .box(0.0018, 0.0014, rib_h)
        )
        cap = cap.union(rib)
    return cap


def _build_safety_collar():
    """Tamper-evident collar ring with a small tear tab."""
    outer = (
        cq.Workplane("XY")
        .circle(COLLAR_OUTER_R)
        .extrude(COLLAR_HEIGHT)
    )
    inner = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -0.001))
        .circle(NECK_R + 0.0002)
        .extrude(COLLAR_HEIGHT + 0.002)
    )
    collar = outer.cut(inner)
    # Small tear/rotation indicator tab
    tab = (
        cq.Workplane("XY")
        .transformed(offset=(COLLAR_OUTER_R + 0.001, 0, COLLAR_HEIGHT / 2))
        .box(0.003, 0.004, COLLAR_HEIGHT * 0.7)
    )
    collar = collar.union(tab)
    return collar


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="canteen_bottle")

    # Materials
    clear = model.material("clear_pet", rgba=(0.78, 0.85, 0.82, 0.28))
    black = model.material("cap_black", rgba=(0.06, 0.06, 0.07, 1.0))
    collar_mat = model.material("collar_white", rgba=(0.88, 0.88, 0.85, 0.95))

    # ---- bottle body (root) ----
    body = model.part("bottle")
    body_geo = _build_bottle_body()
    body.visual(mesh_from_cadquery(body_geo, "bottle_shell"), material=clear, name="bottle_shell")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_RX, NECK_TOP_Z),
        mass=0.035,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- safety collar ----
    collar = model.part("safety_collar")
    collar_geo = _build_safety_collar()
    collar.visual(mesh_from_cadquery(collar_geo, "collar_ring"), material=collar_mat, name="collar_ring")
    collar.inertial = Inertial.from_geometry(
        Cylinder(COLLAR_OUTER_R, COLLAR_HEIGHT),
        mass=0.002,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_HEIGHT / 2.0)),
    )

    # ---- massless carrier for cap joints ----
    carrier = model.part("cap_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.008, 0.008, 0.008)), mass=1e-4)

    # ---- black ribbed cap ----
    cap = model.part("cap")
    cap_geo = _build_cap()
    cap.visual(mesh_from_cadquery(cap_geo, "cap_shell"), material=black, name="cap_shell")
    # Off-axis marker rib for spin detection
    cap.visual(
        Box((0.004, 0.006, 0.012)),
        origin=Origin(xyz=(CAP_R + 0.0015, 0.0, -SKIRT_DROP + 0.006)),
        material=black,
        name="cap_marker",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_HEIGHT),
        mass=0.006,
        origin=Origin(xyz=(0.0, 0.0, (CAP_TOP_Z - SKIRT_DROP) / 2.0)),
    )

    # ---- joints ----
    # Safety collar rotates around neck
    model.articulation(
        "collar_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_Z0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0),
    )

    # Cap rotation (continuous spin on threads)
    model.articulation(
        "cap_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    # Cap slide (prismatic lift off)
    model.articulation(
        "cap_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=CAP_HEIGHT, effort=1.0, velocity=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle")
    collar = object_model.get_part("safety_collar")
    cap = object_model.get_part("cap")
    collar_joint = object_model.get_articulation("collar_rotate")
    rotate = object_model.get_articulation("cap_rotate")
    slide = object_model.get_articulation("cap_slide")

    bottle_shell = body.get_visual("bottle_shell")
    cap_shell = cap.get_visual("cap_shell")
    collar_ring = collar.get_visual("collar_ring")

    # --- Material checks ---
    ctx.check(
        "bottle material is tinted-transparent (alpha < 1)",
        bottle_shell.material.rgba is not None and bottle_shell.material.rgba[3] < 1.0,
        details=f"bottle rgba={bottle_shell.material.rgba}",
    )
    ctx.check(
        "cap material is opaque black",
        cap_shell.material.rgba is not None
        and cap_shell.material.rgba[3] >= 0.99
        and max(cap_shell.material.rgba[:3]) < 0.2,
        details=f"cap rgba={cap_shell.material.rgba}",
    )

    # --- Oval cross-section: body X extent significantly larger than Y ---
    body_aabb = ctx.part_world_aabb(body)
    if body_aabb:
        mn, mx = body_aabb
        dx = mx[0] - mn[0]
        dy = mx[1] - mn[1]
        ctx.check(
            "bottle has flat oval cross-section (X wider than Y)",
            dx > dy * 1.4,
            details=f"dx={dx:.4f}, dy={dy:.4f}, ratio={dx/max(dy,1e-9):.2f}",
        )
    else:
        ctx.fail("bottle AABB", "could not compute body AABB")

    # --- Safety collar sits at neck base ---
    collar_pos = ctx.part_world_position(collar)
    ctx.check(
        "safety collar mounted at neck base",
        collar_pos is not None and SHOULDER_TOP_Z - 0.005 < collar_pos[2] < NECK_TOP_Z,
        details=f"collar origin={collar_pos}",
    )

    # Collar wraps around neck (overlap with body is intentional)
    ctx.allow_overlap(
        collar, body,
        elem_a="collar_ring", elem_b="bottle_shell",
        reason="Safety collar ring clips around the neck, intentionally overlapping the neck wall.",
    )

    # --- Collar rotates: tab moves on quarter turn ---
    collar_aabb_0 = None
    collar_aabb_90 = None
    with ctx.pose({collar_joint: 0.0}):
        collar_aabb_0 = ctx.part_world_aabb(collar)
    with ctx.pose({collar_joint: math.pi / 2.0}):
        collar_aabb_90 = ctx.part_world_aabb(collar)
    if collar_aabb_0 and collar_aabb_90:
        e0 = (collar_aabb_0[1][0] - collar_aabb_0[0][0],
               collar_aabb_0[1][1] - collar_aabb_0[0][1])
        e90 = (collar_aabb_90[1][0] - collar_aabb_90[0][0],
                collar_aabb_90[1][1] - collar_aabb_90[0][1])
        ctx.check(
            "collar rotation moves the tear tab (extents swap x<->y)",
            abs(e0[0] - e90[1]) < 0.004 and abs(e0[0] - e0[1]) > 0.001,
            details=f"rest extents xy=({e0[0]:.4f},{e0[1]:.4f}), quarter=({e90[0]:.4f},{e90[1]:.4f})",
        )

    # --- Cap sits on top of the neck ---
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap mounted on top of the neck",
        cap_pos is not None and cap_pos[2] > BARREL_TOP_Z,
        details=f"cap origin={cap_pos}",
    )

    # Cap skirt overlaps neck threads (intentional seated overlap)
    ctx.allow_overlap(
        cap, body,
        elem_a="cap_shell", elem_b="bottle_shell",
        reason="Cap skirt is intentionally seated over the threaded neck.",
    )

    # --- Cap rotation moves the off-axis marker ---
    with ctx.pose({rotate: 0.0}):
        m0 = ctx.part_world_aabb(cap)
    with ctx.pose({rotate: math.pi / 2.0}):
        m90 = ctx.part_world_aabb(cap)
    if m0 and m90:
        e0c = (m0[1][0] - m0[0][0], m0[1][1] - m0[0][1])
        e90c = (m90[1][0] - m90[0][0], m90[1][1] - m90[0][1])
        ctx.check(
            "cap rotation moves the off-axis marker (extents swap x<->y)",
            abs(e0c[0] - e90c[1]) < 0.002 and abs(e0c[0] - e0c[1]) > 0.0015,
            details=f"rest=({e0c[0]:.4f},{e0c[1]:.4f}), quarter=({e90c[0]:.4f},{e90c[1]:.4f})",
        )

    # --- Cap slide lifts the cap off the neck ---
    rest_z = ctx.part_world_position(cap)[2]
    with ctx.pose({slide: CAP_HEIGHT}):
        lifted_z = ctx.part_world_position(cap)[2]
    ctx.check(
        "cap_slide lifts the cap up off the neck",
        lifted_z > rest_z + CAP_HEIGHT * 0.8,
        details=f"rest_z={rest_z}, lifted_z={lifted_z}",
    )

    # --- At least one non-fixed joint exists ---
    joint_names = [a.name for a in object_model.articulations]
    non_fixed = [
        a for a in object_model.articulations
        if a.articulation_type in (ArticulationType.REVOLUTE, ArticulationType.CONTINUOUS, ArticulationType.PRISMATIC)
    ]
    ctx.check(
        "at least one non-fixed joint exists",
        len(non_fixed) >= 1,
        details=f"joints={joint_names}, non_fixed_count={len(non_fixed)}",
    )

    return ctx.report()


object_model = build_object_model()
