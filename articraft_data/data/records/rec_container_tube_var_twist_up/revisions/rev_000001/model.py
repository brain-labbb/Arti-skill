from __future__ import annotations

# Blue sunscreen twist-up stick tube (fork of squeeze tube).
#
# Frame:
#   - Same rounded-rectangle slab body as the parent (body axis along +Z).
#   - z=0 is the softly rounded bottom; the tube rises in +Z.
#   - The open neck and lift cap are replaced by a twist-up stick dispenser.
#
# Mechanism:
#   - twist_ring: CONTINUOUS rotation at the bottom; knurled white ring.
#   - platform: PRISMATIC along +Z inside the bore; flat product disk that
#     rises and lowers, child of the twist ring (parent-child chain coupling).
#
# The rounded-rectangle slab body and internal bore cavity are unchanged
# from the parent.

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

# ---- key dimensions (meters) — body identical to parent ----
TUBE_HALF_W = 0.0265   # half width in Y at the widest point
TUBE_HALF_T = 0.0140   # half thickness in X; rounded-rectangle slab depth
BODY_TOP_Z = 0.104
BODY_BOTTOM_Z = 0.0

NECK_INNER_R = 0.0051
CAVITY_BOTTOM_Z = 0.035
CAVITY_R = NECK_INNER_R * 1.10  # ~0.00561, matches parent bore

# ---- twist ring (at the bottom) ----
TWIST_RING_OUTER_R = 0.0095
TWIST_RING_INNER_R = 0.006
TWIST_RING_HEIGHT = 0.011
TWIST_RING_EMBED = 0.002   # top overlap into body bottom for mounting

# ---- platform (inside bore) ----
PLATFORM_R = CAVITY_R - 0.0005   # ~0.00511, clearance fit in bore
PLATFORM_THICKNESS = 0.003
PLATFORM_BOSS_R = PLATFORM_R * 0.65
PLATFORM_BOSS_H = 0.001
PLATFORM_TRAVEL = 0.050          # usable stroke inside cavity


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _oval_loop(z: float, half_t: float, half_w: float, n: int = 48):
    pts = []
    p = 3.4
    for i in range(n):
        a = 2.0 * math.pi * i / n
        c = math.cos(a)
        s = math.sin(a)
        x = half_t * math.copysign(abs(c) ** (2.0 / p), c)
        y = half_w * math.copysign(abs(s) ** (2.0 / p), s)
        pts.append((x, y, z))
    return pts


def _loop_xy(half_t: float, half_w: float):
    return [(p[0], p[1]) for p in _oval_loop(0.0, half_t, half_w)]


def _tube_body() -> cq.Workplane:
    """Rounded-rectangle slab body with through-bore cavity (identical to parent)."""
    sections = [
        (BODY_BOTTOM_Z, TUBE_HALF_T * 0.78, TUBE_HALF_W * 0.92),
        (0.010, TUBE_HALF_T * 0.96, TUBE_HALF_W * 0.99),
        (0.055, TUBE_HALF_T, TUBE_HALF_W),
        (0.092, TUBE_HALF_T * 0.99, TUBE_HALF_W * 0.99),
        (BODY_TOP_Z, TUBE_HALF_T * 0.94, TUBE_HALF_W * 0.96),
    ]
    wp = cq.Workplane("XY")
    prev = 0.0
    first = True
    for z, ht, hw in sections:
        off = z if first else z - prev
        wp = wp.workplane(offset=off)
        wp = wp.polyline(_loop_xy(ht, hw)).close()
        prev = z
        first = False
    body = wp.loft(ruled=False)
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=CAVITY_BOTTOM_Z)
        .circle(NECK_INNER_R * 1.10)
        .extrude(BODY_TOP_Z - CAVITY_BOTTOM_Z + 0.004)
    )
    return body.cut(cavity)


def _mouth_rim() -> cq.Workplane:
    """Short annular rim at the open mouth (replaces the parent's raised neck)."""
    rim_h = 0.004
    outer_r = CAVITY_R + 0.002
    inner_r = CAVITY_R
    rim = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP_Z - 0.001)
        .circle(outer_r)
        .circle(inner_r)
        .extrude(rim_h)
    )
    # Small lip flange at the top edge
    lip = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP_Z + rim_h - 0.001 - 0.0015)
        .circle(outer_r + 0.0008)
        .circle(inner_r)
        .extrude(0.0015)
    )
    return rim.union(lip)


def _twist_ring() -> cq.Workplane:
    """Knurled twist ring in the ring's local frame (origin at body bottom z=0)."""
    ring_bottom = -(TWIST_RING_HEIGHT - TWIST_RING_EMBED)  # below body
    ring_top = TWIST_RING_EMBED

    # Main annular ring
    ring = (
        cq.Workplane("XY")
        .workplane(offset=ring_bottom)
        .circle(TWIST_RING_OUTER_R)
        .circle(TWIST_RING_INNER_R)
        .extrude(TWIST_RING_HEIGHT)
    )

    # Top flange for visual mounting against body bottom
    flange = (
        cq.Workplane("XY")
        .workplane(offset=ring_top - 0.0015)
        .circle(TWIST_RING_OUTER_R + 0.001)
        .circle(TWIST_RING_INNER_R)
        .extrude(0.0015)
    )
    ring = ring.union(flange)

    # Bottom disk (with center hole for the drive shaft)
    bottom = (
        cq.Workplane("XY")
        .workplane(offset=ring_bottom)
        .circle(TWIST_RING_OUTER_R)
        .circle(TWIST_RING_INNER_R * 0.7)
        .extrude(0.001)
    )
    ring = ring.union(bottom)

    # Knurling: radial fins around the outer surface
    n_fins = 12
    fin_height = TWIST_RING_HEIGHT - 0.004
    for i in range(n_fins):
        angle_deg = 360.0 * i / n_fins
        fin = (
            cq.Workplane("XY")
            .workplane(offset=ring_bottom + 0.002)
            .transformed(rotate=(0, 0, angle_deg))
            .center(TWIST_RING_OUTER_R + 0.0005, 0)
            .rect(0.001, 0.0006)
            .extrude(fin_height)
        )
        ring = ring.union(fin)

    return ring


def _platform_disk() -> cq.Workplane:
    """Flat product platform disk in the platform's local frame (origin at joint)."""
    disk = (
        cq.Workplane("XY")
        .circle(PLATFORM_R)
        .extrude(PLATFORM_THICKNESS)
    )
    # Raised boss on top (product contact surface)
    boss = (
        cq.Workplane("XY")
        .workplane(offset=PLATFORM_THICKNESS)
        .circle(PLATFORM_BOSS_R)
        .extrude(PLATFORM_BOSS_H)
    )
    # Thin guide stem below (stays within bore at all positions)
    stem = (
        cq.Workplane("XY")
        .workplane(offset=-0.025)
        .circle(0.002)
        .extrude(0.025)
    )
    return disk.union(boss).union(stem)


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sunscreen_twist_stick")

    # Materials
    blue = model.material("tube_blue", rgba=(0.62, 0.80, 0.92, 1.0))
    white = model.material("ring_white", rgba=(0.95, 0.96, 0.97, 1.0))
    gray = model.material("platform_gray", rgba=(0.82, 0.83, 0.84, 1.0))
    label_white = model.material("label_white", rgba=(1.0, 1.0, 1.0, 1.0))

    # ---- body (root): rounded blue slab + mouth rim + bore cavity ----
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_tube_body(), "tube_body"),
        material=blue,
        name="tube_body",
    )
    body.visual(
        mesh_from_cadquery(_mouth_rim(), "mouth_rim"),
        material=blue,
        name="mouth_rim",
    )

    # Brand marks (same as parent, emitted via for-range loops).
    # Embedded at the body surface so they connect to the tube mesh.
    # The body surface curves inward at the mark Y position, so the marks
    # are made thicker in X to ensure overlap with the body solid.
    for i in range(8):
        body.visual(
            Box((0.002, 0.0023, 0.0060)),
            origin=Origin(
                xyz=(-0.013, TUBE_HALF_W * 0.52, 0.074 - i * 0.0065)
            ),
            material=label_white,
            name=f"vertical_brand_mark_{i}",
        )
    for i in range(3):
        body.visual(
            Box((0.002, 0.013, 0.0012)),
            origin=Origin(xyz=(-0.013, -0.002, 0.065 - i * 0.010)),
            material=label_white,
            name=f"small_front_label_{i}",
        )

    body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.126),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.063)),
    )

    # ---- twist ring: CONTINUOUS rotation at the body bottom ----
    twist_ring = model.part("twist_ring")
    twist_ring.visual(
        mesh_from_cadquery(_twist_ring(), "ring_shell"),
        material=white,
        name="ring_shell",
    )
    twist_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=TWIST_RING_OUTER_R, length=TWIST_RING_HEIGHT),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, -(TWIST_RING_HEIGHT - TWIST_RING_EMBED) * 0.5)),
    )

    # ---- platform: PRISMATIC carrier inside the bore ----
    platform = model.part("platform")
    platform.visual(
        mesh_from_cadquery(_platform_disk(), "platform_disk"),
        material=gray,
        name="platform_disk",
    )
    platform.inertial = Inertial.from_geometry(
        Cylinder(radius=PLATFORM_R, length=PLATFORM_THICKNESS + PLATFORM_BOSS_H),
        mass=0.003,
        origin=Origin(xyz=(0.0, 0.0, (PLATFORM_THICKNESS + PLATFORM_BOSS_H) * 0.5)),
    )

    # ---- articulations ----

    # Twist ring: CONTINUOUS rotation around Z at the body bottom face.
    # The platform is a child of the twist ring so it follows the ring's
    # rotation (representing the threaded elevator coupling).
    model.articulation(
        "body_to_twist_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=twist_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0),
    )

    # Platform: PRISMATIC along +Z inside the bore, child of twist ring.
    # The twist ring's rotation carries the platform azimuthally; the
    # prismatic joint controls vertical elevation (the real mechanism
    # converts twist rotation into linear travel via an internal thread).
    model.articulation(
        "twist_ring_to_platform",
        ArticulationType.PRISMATIC,
        parent=twist_ring,
        child=platform,
        origin=Origin(xyz=(0.0, 0.0, CAVITY_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.1,
            lower=0.0,
            upper=PLATFORM_TRAVEL,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    twist_ring = object_model.get_part("twist_ring")
    platform = object_model.get_part("platform")
    twist_joint = object_model.get_articulation("body_to_twist_ring")
    platform_joint = object_model.get_articulation("twist_ring_to_platform")

    # --- Body is still the same rounded-rectangle slab ---
    body_ext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "body is rounded-rectangle slab (wider in Y than thick in X)",
        body_ext[1] > body_ext[0] + 0.015,
        details=f"body extents (x,y,z)={body_ext}",
    )
    ctx.check(
        "body stands upright (tall)",
        body_ext[2] > 0.10,
        details=f"body z extent={body_ext[2]}",
    )
    tube_aabb = ctx.part_element_world_aabb(body, elem="tube_body")
    tube_x = tube_aabb[1][0] - tube_aabb[0][0]
    tube_y = tube_aabb[1][1] - tube_aabb[0][1]
    ctx.check(
        "blue body has broad rounded front and narrow side depth",
        0.025 < tube_x < 0.034 and tube_y > 0.048,
        details=f"tube_body x-depth={tube_x}, y-width={tube_y}",
    )

    # --- Bore cavity exists and reaches the top opening ---
    ctx.check(
        "bore cavity extends deep into body",
        CAVITY_BOTTOM_Z < BODY_TOP_Z - 0.050,
        details=f"cavity_bottom_z={CAVITY_BOTTOM_Z}, body_top_z={BODY_TOP_Z}",
    )

    # --- Mouth rim replaces the raised neck ---
    rim_aabb = ctx.part_element_world_aabb(body, elem="mouth_rim")
    ctx.check(
        "mouth rim sits at body top (replaces neck)",
        rim_aabb[0][2] > BODY_TOP_Z - 0.003 and rim_aabb[1][2] > BODY_TOP_Z,
        details=f"rim aabb={rim_aabb}",
    )

    # --- Twist ring is CONTINUOUS at body bottom ---
    ctx.check(
        "twist ring joint is CONTINUOUS",
        twist_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={twist_joint.articulation_type}",
    )
    ctx.check(
        "twist ring axis is Z",
        tuple(twist_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={twist_joint.axis}",
    )
    ring_aabb = ctx.part_world_aabb(twist_ring)
    ctx.check(
        "twist ring protrudes below body bottom",
        ring_aabb[0][2] < -0.004,
        details=f"ring bottom z={ring_aabb[0][2]}",
    )

    # --- Platform is PRISMATIC along Z, child of twist ring ---
    ctx.check(
        "platform joint is PRISMATIC along Z",
        platform_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(platform_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={platform_joint.articulation_type}, axis={platform_joint.axis}",
    )
    ctx.check(
        "platform is a child of the twist ring (elevating mechanism)",
        platform_joint.parent == "twist_ring",
        details=f"parent={platform_joint.parent}",
    )

    # --- Platform rises with positive prismatic travel ---
    with ctx.pose({platform_joint: 0.0}):
        rest_aabb = ctx.part_world_aabb(platform)
        rest_z = (rest_aabb[0][2] + rest_aabb[1][2]) * 0.5

    with ctx.pose({platform_joint: PLATFORM_TRAVEL}):
        extended_aabb = ctx.part_world_aabb(platform)
        extended_z = (extended_aabb[0][2] + extended_aabb[1][2]) * 0.5

    ctx.check(
        "platform rises with positive prismatic travel",
        extended_z > rest_z + PLATFORM_TRAVEL * 0.9,
        details=f"rest_z={rest_z:.6f}, extended_z={extended_z:.6f}, delta={extended_z - rest_z:.6f}",
    )

    # --- Platform stays within body on XY at rest and extended ---
    with ctx.pose({platform_joint: 0.0}):
        ctx.expect_within(
            platform, body,
            axes="xy",
            margin=0.002,
            name="platform stays within body bore on XY at rest",
        )

    with ctx.pose({platform_joint: PLATFORM_TRAVEL}):
        ctx.expect_within(
            platform, body,
            axes="xy",
            margin=0.002,
            name="platform stays within body bore on XY at full extension",
        )

    # --- Platform remains below body top at full extension ---
    with ctx.pose({platform_joint: PLATFORM_TRAVEL}):
        ext_plat_aabb = ctx.part_world_aabb(platform)
        ctx.check(
            "platform stays below body top at full extension",
            ext_plat_aabb[1][2] < BODY_TOP_Z + 0.005,
            details=f"platform top z={ext_plat_aabb[1][2]}, body_top_z={BODY_TOP_Z}",
        )

    # --- Twist ring mounting overlap ---
    ctx.allow_overlap(
        twist_ring, body,
        elem_a="ring_shell",
        elem_b="tube_body",
        reason="Twist ring flange is intentionally embedded 2mm into the body bottom for mounting.",
    )

    # --- Platform nested inside body bore ---
    ctx.allow_overlap(
        platform, body,
        elem_a="platform_disk",
        elem_b="tube_body",
        reason="Platform disk and guide stem are intentionally nested inside the body bore cavity.",
    )
    ctx.expect_within(
        platform, body,
        axes="xy",
        margin=0.001,
        name="platform disk is centered in the bore",
    )

    return ctx.report()


object_model = build_object_model()
