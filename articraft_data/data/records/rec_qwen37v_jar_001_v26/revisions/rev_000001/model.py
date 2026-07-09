from __future__ import annotations

# MASON JAR with two-piece lid (screw band ring + flat sealing disk).
# Frame: jar axis along +Z, base on z=0, centered at origin.
#
# The jar is a tall cylindrical glass body with a shoulder narrowing into a
# threaded neck/mouth.  Glass wall thickness is clearly visible at the rim.
# Prominent thread ridges protrude from the neck outer wall.
#
# Two-piece lid:
#   - ring (screw band): cylindrical band with knurled grip and an inward lip
#     at the top that clamps the disk against the rim.
#   - disk (flat lid): thin circular sealing disk that sits on the mouth rim.
#
# Parts:
#   body          - glass jar with threads (root)
#   ring_carrier  - massless rotation carrier
#   ring          - screw band
#   disk          - flat sealing disk
#
# Articulations:
#   ring_rotate   - CONTINUOUS about +Z (screw rotation of band)
#   ring_lift     - PRISMATIC along +Z (remove band from jar)
#   disk_lift     - PRISMATIC along +Z (remove disk from jar)

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
JAR_OUTER_R = 0.040           # outer radius of glass body (~80 mm dia)
JAR_BODY_H = 0.100            # cylindrical body height
WALL = 0.004                  # glass wall thickness
SHOULDER_H = 0.010            # shoulder transition height
NECK_OUTER_R = 0.033          # outer radius of threaded neck
NECK_H = 0.018                # threaded neck height
SHOULDER_TOP_Z = JAR_BODY_H + SHOULDER_H           # 0.110
RIM_TOP_Z = SHOULDER_TOP_Z + NECK_H                # 0.128

# Thread ridges
THREAD_COUNT = 4
THREAD_START_Z = SHOULDER_TOP_Z + 0.003
THREAD_SPACING = 0.0035

# Ring (screw band)
RING_OUTER_R = NECK_OUTER_R + 0.004   # 0.037
RING_INNER_R = NECK_OUTER_R + 0.001   # 0.034 – slips over threads
RING_H = 0.020                        # band height
RING_LIP_INNER_R = NECK_OUTER_R - 0.004  # 0.029 – inward lip at top
RING_LIP_DEPTH = 0.003                # top 3 mm has narrower opening

# Disk (flat sealing lid)
DISK_R = NECK_OUTER_R - 0.003         # 0.030
DISK_H = 0.002

# Ring seated position:
#   ring local z = 0 maps to world z = SHOULDER_TOP_Z = 0.110
#   ring local z = RING_H maps to world z = 0.130
# Carrier frame is at world z = RIM_TOP_Z = 0.128.
# ring_lift origin z in carrier frame = SHOULDER_TOP_Z - RIM_TOP_Z = -NECK_H
RING_LIFT_ORIGIN_Z = -NECK_H          # -0.018


# ──────────────────────── geometry builders ────────────────────────


def _jar_glass() -> cq.Workplane:
    """Hollow thick-walled mason jar body with shoulder and threaded neck.

    The revolved profile traces the outer wall up through a rounded shoulder
    into the narrower neck, across the rim top (showing glass wall thickness),
    then back down the inner wall to form a real open-topped cavity.
    """
    pts = [
        (0.0, 0.0),
        (JAR_OUTER_R, 0.0),
        (JAR_OUTER_R, JAR_BODY_H - 0.004),
        (JAR_OUTER_R - 0.003, JAR_BODY_H),
        (JAR_OUTER_R - 0.006, JAR_BODY_H + SHOULDER_H * 0.5),
        (NECK_OUTER_R, SHOULDER_TOP_Z),
        (NECK_OUTER_R, RIM_TOP_Z),
        (NECK_OUTER_R - WALL, RIM_TOP_Z),          # rim shows wall thickness
        (NECK_OUTER_R - WALL, SHOULDER_TOP_Z + 0.003),
        (JAR_OUTER_R - WALL, JAR_BODY_H - 0.004),
        (JAR_OUTER_R - WALL, WALL),
        (0.0, WALL),
        (0.0, 0.0),
    ]
    profile = cq.Workplane("XZ").polyline(pts).close()
    return profile.revolve(360.0, (0, 0, 0), (0, 1, 0))


def _neck_threads() -> cq.Workplane:
    """Prominent thread ridges protruding from the neck outer wall."""
    threads = None
    for i in range(THREAD_COUNT):
        z = THREAD_START_Z + i * THREAD_SPACING
        ring = (
            cq.Workplane("XY")
            .workplane(offset=z)
            .circle(NECK_OUTER_R + 0.0015)   # protrudes outward
            .circle(NECK_OUTER_R - 0.0005)    # thin ridge root
            .extrude(0.002)
        )
        threads = ring if threads is None else threads.union(ring)
    return threads


def _jar_body_bead() -> cq.Workplane:
    """Decorative horizontal bead near the shoulder – common on mason jars."""
    bead_z = JAR_BODY_H - 0.012
    return (
        cq.Workplane("XY")
        .workplane(offset=bead_z)
        .circle(JAR_OUTER_R + 0.0012)
        .circle(JAR_OUTER_R - 0.0008)
        .extrude(0.004)
    )


def _ring_band() -> cq.Workplane:
    """Screw band: cylindrical shell with inward lip at the top.

    The main bore (R < RING_INNER_R) runs from the bottom up to 3 mm below
    the top.  The lip bore (R < RING_LIP_INNER_R) cuts through the remaining
    top section, leaving an annular shelf that clamps the disk edge.
    """
    outer = cq.Workplane("XY").circle(RING_OUTER_R).extrude(RING_H)

    # Main bore – wide opening for most of the band height.
    bore_depth = RING_H - RING_LIP_DEPTH
    main_bore = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .circle(RING_INNER_R)
        .extrude(bore_depth + 0.001)
    )
    # Lip bore – narrower opening at the top.
    lip_bore = (
        cq.Workplane("XY")
        .workplane(offset=bore_depth)
        .circle(RING_LIP_INNER_R)
        .extrude(RING_H - bore_depth + 0.001)
    )
    return outer.cut(main_bore).cut(lip_bore)


def _ring_knurl() -> cq.Workplane:
    """Vertical grip ribs on the ring exterior."""
    ribs = None
    n = 32
    for i in range(n):
        ang = 2.0 * math.pi * i / n
        r = RING_OUTER_R + 0.0004
        rib = (
            cq.Workplane("XY")
            .workplane(offset=0.002)
            .center(r * math.cos(ang), r * math.sin(ang))
            .rect(0.0012, 0.0012)
            .extrude(RING_H - 0.004)
        )
        ribs = rib if ribs is None else ribs.union(rib)
    return ribs


def _disk_mesh():
    """Flat sealing disk with a very slight dome."""
    base = cq.Workplane("XY").circle(DISK_R).extrude(DISK_H * 0.6)
    dome = (
        cq.Workplane("XY")
        .workplane(offset=DISK_H * 0.6)
        .circle(DISK_R)
        .workplane(offset=DISK_H * 0.4)
        .circle(DISK_R * 0.88)
        .loft(ruled=False)
    )
    return mesh_from_cadquery(base.union(dome), "disk_shell")


# ──────────────────────── model ────────────────────────


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mason_jar")

    glass_green = model.material("glass_green", rgba=(0.60, 0.75, 0.55, 0.40))
    ring_gold = model.material("ring_gold", rgba=(0.76, 0.65, 0.35, 1.0))
    disk_silver = model.material("disk_silver", rgba=(0.78, 0.78, 0.80, 1.0))
    marker_red = model.material("marker_red", rgba=(0.80, 0.15, 0.10, 1.0))

    # ──── jar body (root): glass shell + neck threads + body bead ────
    body = model.part("body")
    glass = _jar_glass().union(_neck_threads()).union(_jar_body_bead())
    body.visual(
        mesh_from_cadquery(glass, "jar_glass"),
        material=glass_green,
        name="jar_glass",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(JAR_OUTER_R, RIM_TOP_Z),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP_Z * 0.5)),
    )

    # ──── ring carrier (massless, no visuals): rotates about +Z ────
    carrier = model.part("ring_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.008, 0.008, 0.008)), mass=1e-4)
    model.articulation(
        "ring_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    # ──── ring (screw band) ────
    ring = model.part("ring")
    ring.visual(
        mesh_from_cadquery(_ring_band(), "ring_band"),
        material=ring_gold,
        name="ring_band",
    )
    ring.visual(
        mesh_from_cadquery(_ring_knurl(), "ring_knurl"),
        material=ring_gold,
        name="ring_knurl",
    )
    # Off-axis red marker so rotation is visible.
    ring.visual(
        Box((0.004, 0.004, 0.003)),
        origin=Origin(xyz=(RING_OUTER_R - 0.005, 0.0, RING_H - 0.006)),
        material=marker_red,
        name="ring_marker",
    )
    ring.inertial = Inertial.from_geometry(
        Cylinder(RING_OUTER_R, RING_H),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.0, RING_H * 0.5)),
    )
    model.articulation(
        "ring_lift",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=ring,
        origin=Origin(xyz=(0.0, 0.0, RING_LIFT_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=RING_H + 0.020, effort=1.0, velocity=1.0,
        ),
    )

    # ──── disk (flat sealing lid) ────
    disk = model.part("disk")
    disk.visual(_disk_mesh(), material=disk_silver, name="disk_shell")
    disk.inertial = Inertial.from_geometry(
        Cylinder(DISK_R, DISK_H),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, DISK_H * 0.5)),
    )
    model.articulation(
        "disk_lift",
        ArticulationType.PRISMATIC,
        parent=body,
        child=disk,
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=0.040, effort=1.0, velocity=1.0,
        ),
    )

    return model


# ──────────────────────── helpers ────────────────────────


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


# ──────────────────────── tests ────────────────────────


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    carrier = object_model.get_part("ring_carrier")
    ring = object_model.get_part("ring")
    disk = object_model.get_part("disk")
    ring_rotate = object_model.get_articulation("ring_rotate")
    ring_lift = object_model.get_articulation("ring_lift")
    disk_lift = object_model.get_articulation("disk_lift")

    # ── intentional overlaps ──

    # Ring bore slips over the threaded neck (ring_band surrounds jar_glass
    # in the neck region with thread ridges engaging the bore wall).
    ctx.allow_overlap(
        ring, body,
        elem_a="ring_band", elem_b="jar_glass",
        reason="Ring band slips over the threaded neck – bore surrounds neck threads.",
    )

    # Ring top lip clamps the disk edge against the rim (small local embed).
    ctx.allow_overlap(
        ring, disk,
        elem_a="ring_band", elem_b="disk_shell",
        reason="Ring top lip clamps the disk edge against the mouth rim.",
    )

    # ── mason jar proportions: taller than wide ──
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jar is taller than wide (mason jar proportions)",
        bext[2] > bext[0] + 0.005 and bext[2] > bext[1] + 0.005,
        details=f"body extents={bext}",
    )

    # ── ring wraps around the neck (XY overlap with body) ──
    ctx.expect_overlap(
        ring, body, axes="xy", min_overlap=0.02,
        name="ring wraps around the neck",
    )

    # ── disk covers the mouth (XY overlap with body) ──
    ctx.expect_overlap(
        disk, body, axes="xy", min_overlap=0.02,
        name="disk covers the mouth",
    )

    # ── disk fits within ring footprint ──
    ctx.expect_within(
        disk, ring, axes="xy", margin=0.005,
        name="disk fits inside ring footprint",
    )

    # ── carrier has no visuals ──
    ctx.check(
        "ring_carrier has no visuals",
        len(carrier.visuals) == 0,
        details=f"carrier visuals={len(carrier.visuals)}",
    )

    # ── ring_rotate spins the ring (off-axis marker moves) ──
    marker0 = ctx.part_element_world_aabb(ring, elem="ring_marker")
    m0 = (
        (marker0[0][0] + marker0[1][0]) * 0.5,
        (marker0[0][1] + marker0[1][1]) * 0.5,
    )
    with ctx.pose({ring_rotate: math.pi / 2.0}):
        marker1 = ctx.part_element_world_aabb(ring, elem="ring_marker")
        m1 = (
            (marker1[0][0] + marker1[1][0]) * 0.5,
            (marker1[0][1] + marker1[1][1]) * 0.5,
        )
    moved = math.hypot(m1[0] - m0[0], m1[1] - m0[1])
    ctx.check(
        "ring_rotate spins the ring (marker moves)",
        moved > 0.01,
        details=f"rest={m0}, quarter-turn={m1}, moved={moved}",
    )

    # ── ring_lift removes ring from jar ──
    ring_rest_z = ctx.part_world_position(ring)[2]
    with ctx.pose({ring_lift: RING_H + 0.020}):
        ring_lifted_z = ctx.part_world_position(ring)[2]
        ctx.expect_gap(
            ring, body, axis="z", min_gap=0.0,
            positive_elem="ring_band", negative_elem="jar_glass",
            name="lifted ring clears the jar",
        )
    ctx.check(
        "ring_lift removes the ring from the jar",
        ring_lifted_z > ring_rest_z + RING_H * 0.5,
        details=f"rest_z={ring_rest_z}, lifted_z={ring_lifted_z}",
    )

    # ── disk_lift removes disk from jar ──
    disk_rest_z = ctx.part_world_position(disk)[2]
    with ctx.pose({disk_lift: 0.040}):
        disk_lifted_z = ctx.part_world_position(disk)[2]
        ctx.expect_gap(
            disk, body, axis="z", min_gap=0.0,
            positive_elem="disk_shell", negative_elem="jar_glass",
            name="lifted disk clears the jar",
        )
    ctx.check(
        "disk_lift removes the disk from the jar",
        disk_lifted_z > disk_rest_z + 0.02,
        details=f"rest_z={disk_rest_z}, lifted_z={disk_lifted_z}",
    )

    # ── disk sits on top of the jar (above rim) ──
    ctx.check(
        "disk sits at or above rim top",
        disk_rest_z is not None and disk_rest_z >= RIM_TOP_Z - 0.001,
        details=f"disk_rest_z={disk_rest_z}, rim_top={RIM_TOP_Z}",
    )

    # ── ring and disk are separate parts ──
    ctx.check(
        "ring and disk are distinct parts",
        ring.name != disk.name,
        details=f"ring={ring.name}, disk={disk.name}",
    )

    # ── at least one non-fixed joint exists ──
    non_fixed = [
        j for j in object_model.articulations
        if j.articulation_type != ArticulationType.FIXED
    ]
    ctx.check(
        "at least one non-fixed joint",
        len(non_fixed) >= 1,
        details=f"non_fixed={[j.name for j in non_fixed]}",
    )

    return ctx.report()


object_model = build_object_model()
