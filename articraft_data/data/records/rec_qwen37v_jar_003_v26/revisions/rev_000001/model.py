from __future__ import annotations

# Mason jar with two-piece lid (screw ring + flat disk).
# Frame: vertical axis +Z, jar centered on world origin, base on z=0.
#
# Parts:
#   - jar_body: clear glass jar, cylindrical body, shoulder taper, threaded neck,
#               hollow interior with glass wall thickness at mouth.
#   - lid_ring: metal screw band (annular ring) that threads onto the jar neck.
#   - lid_disk: flat metal disk that seals the jar mouth (sits under the ring).
#
# Articulations:
#   - body_to_ring: REVOLUTE around Z (unscrewing the band). Positive q rotates
#                   the ring counterclockwise (loosening). Limits 0..6.28 rad.
#   - body_to_disk: PRISMATIC along +Z (lifting the flat disk off). Limits 0..0.06 m.

import math
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Inertial,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ----- key dimensions (meters) -----
# Body
BODY_OD = 0.076        # outer diameter of jar body
BODY_RADIUS = BODY_OD / 2.0
BODY_H = 0.130         # total jar height
GLASS_WALL = 0.003     # glass wall thickness for body

# Shoulder / neck transition
SHOULDER_H = 0.012     # height of the shoulder taper
NECK_OD = 0.068        # outer diameter at the neck (slightly narrower)
NECK_RADIUS = NECK_OD / 2.0
NECK_H = 0.018         # height of the threaded neck section

# Mouth / rim
RIM_OD = 0.070         # outer diameter of the top rim (slightly wider than neck)
RIM_RADIUS = RIM_OD / 2.0
RIM_H = 0.004          # height of the thickened rim lip
MOUTH_WALL = 0.004     # glass wall thickness at mouth (requested)

# Thread ridges on exterior of neck
THREAD_COUNT = 3
THREAD_HEIGHT = 0.0015  # radial height of each thread ridge
THREAD_WIDTH = 0.0025   # vertical width of each thread ridge

# Lid ring (screw band)
RING_OD = 0.074        # outer diameter of ring
RING_ID = 0.060        # inner diameter of ring (clears neck)
RING_H = 0.016         # height of the ring band
RING_WALL = 0.002      # metal thickness

# Lid disk (flat lid)
DISK_OD = 0.064        # outer diameter of flat disk
DISK_H = 0.001         # thickness of flat disk

# Z positions (from bottom)
BODY_BOTTOM_Z = 0.0
SHOULDER_BOTTOM_Z = BODY_H - NECK_H - SHOULDER_H - RIM_H
NECK_BOTTOM_Z = BODY_H - NECK_H - RIM_H
RIM_BOTTOM_Z = BODY_H - RIM_H
JAR_TOP_Z = BODY_H


def _jar_body() -> cq.Workplane:
    """Build the glass mason jar body with shoulder, threaded neck, hollow interior."""
    # Revolve a profile for the outer jar shape
    # Profile points (r, z) from bottom center going up the right side
    # Body cylinder from z=0 to shoulder
    outer_profile = [
        (0.0, 0.0),
        (BODY_RADIUS, 0.0),
        (BODY_RADIUS, SHOULDER_BOTTOM_Z),
        (NECK_RADIUS, SHOULDER_BOTTOM_Z + SHOULDER_H),
        (NECK_RADIUS, NECK_BOTTOM_Z + NECK_H),
        (RIM_RADIUS, RIM_BOTTOM_Z),
        (RIM_RADIUS, JAR_TOP_Z),
        (0.0, JAR_TOP_Z),
    ]

    # Build outer solid via revolve
    outer = (
        cq.Workplane("XZ")
        .moveTo(outer_profile[0][0], outer_profile[0][1])
    )
    for pt in outer_profile[1:]:
        outer = outer.lineTo(pt[0], pt[1])
    outer = outer.close().revolve(360, (0, 0, 0), (0, 1, 0))

    # Build inner cavity (hollow interior, open at top)
    inner_r = BODY_RADIUS - GLASS_WALL
    inner_neck_r = NECK_RADIUS - MOUTH_WALL
    floor_z = GLASS_WALL  # solid glass floor

    inner_profile = [
        (0.0, floor_z),
        (inner_r, floor_z),
        (inner_r, SHOULDER_BOTTOM_Z),
        (inner_neck_r, SHOULDER_BOTTOM_Z + SHOULDER_H),
        (inner_neck_r, JAR_TOP_Z + 0.005),  # over-extrude to open through top
        (0.0, JAR_TOP_Z + 0.005),
    ]

    inner = (
        cq.Workplane("XZ")
        .moveTo(inner_profile[0][0], inner_profile[0][1])
    )
    for pt in inner_profile[1:]:
        inner = inner.lineTo(pt[0], pt[1])
    inner = inner.close().revolve(360, (0, 0, 0), (0, 1, 0))

    jar = outer.cut(inner)

    # Add thread ridges around the neck exterior
    for i in range(THREAD_COUNT):
        ridge_z = NECK_BOTTOM_Z + 0.003 + i * (THREAD_WIDTH + 0.002)
        if ridge_z + THREAD_WIDTH > NECK_BOTTOM_Z + NECK_H:
            break
        # Create a torus (ring) at the neck
        ridge = (
            cq.Workplane("XY")
            .workplane(offset=ridge_z + THREAD_WIDTH / 2.0)
            .circle(NECK_RADIUS + THREAD_HEIGHT / 2.0)
            .circle(NECK_RADIUS - 0.0005)
            .extrude(THREAD_WIDTH)
        )
        jar = jar.union(ridge)

    return jar


def _lid_ring() -> cq.Workplane:
    """Build the screw band (annular ring). Local frame: bottom at z=0."""
    # Outer cylinder minus inner bore
    outer = (
        cq.Workplane("XY")
        .circle(RING_OD / 2.0)
        .extrude(RING_H)
    )
    inner = (
        cq.Workplane("XY")
        .circle(RING_ID / 2.0)
        .extrude(RING_H)
    )
    ring = outer.cut(inner)

    # Add internal thread grooves (ridges on the inner surface that match jar threads)
    for i in range(THREAD_COUNT):
        groove_z = 0.002 + i * (THREAD_WIDTH + 0.002)
        if groove_z + THREAD_WIDTH > RING_H:
            break
        groove = (
            cq.Workplane("XY")
            .workplane(offset=groove_z)
            .circle(RING_ID / 2.0 + 0.0005)
            .circle(RING_ID / 2.0 - THREAD_HEIGHT / 2.0)
            .extrude(THREAD_WIDTH)
        )
        ring = ring.cut(groove)

    return ring


def _lid_disk() -> cq.Workplane:
    """Build the flat sealing disk. Local frame: bottom at z=0."""
    disk = (
        cq.Workplane("XY")
        .circle(DISK_OD / 2.0)
        .extrude(DISK_H)
    )
    return disk


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mason_jar")

    glass = model.material("clear_glass", rgba=(0.80, 0.88, 0.85, 0.30))
    metal_silver = model.material("tin_plate", rgba=(0.78, 0.76, 0.72, 1.0))
    metal_band = model.material("zinc_band", rgba=(0.72, 0.72, 0.68, 1.0))

    # ---- jar body (root): clear glass mason jar ----
    body = model.part("jar_body")
    body.visual(
        mesh_from_cadquery(_jar_body(), "jar_glass"),
        material=glass,
        name="jar_glass",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=BODY_RADIUS, length=BODY_H),
        mass=0.32,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # ---- lid disk: flat metal sealing disk ----
    # Sits on top of the jar mouth, under the ring
    disk = model.part("lid_disk")
    disk.visual(
        mesh_from_cadquery(_lid_disk(), "flat_lid"),
        material=metal_silver,
        name="flat_lid",
    )
    disk.inertial = Inertial.from_geometry(
        Cylinder(radius=DISK_OD / 2.0, length=DISK_H),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, DISK_H / 2.0)),
    )

    # ---- lid ring: screw band ----
    ring = model.part("lid_ring")
    ring.visual(
        mesh_from_cadquery(_lid_ring(), "screw_ring"),
        material=metal_band,
        name="screw_ring",
    )
    ring.inertial = Inertial.from_geometry(
        Cylinder(radius=RING_OD / 2.0, length=RING_H),
        mass=0.015,
        origin=Origin(xyz=(0.0, 0.0, RING_H / 2.0)),
    )

    # Articulation: body_to_disk - PRISMATIC along +Z (lift disk off jar)
    # At q=0 the disk sits on top of the jar rim.
    disk_seat_z = JAR_TOP_Z  # disk bottom sits on jar top rim
    model.articulation(
        "body_to_disk",
        ArticulationType.PRISMATIC,
        parent=body,
        child=disk,
        origin=Origin(xyz=(0.0, 0.0, disk_seat_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.3, lower=0.0, upper=0.06),
    )

    # Articulation: body_to_ring - REVOLUTE around Z (screw the band on/off)
    # At q=0 the ring is seated (screwed down) over the neck/disk.
    # The ring bottom sits slightly above the disk, encircling the neck.
    ring_seat_z = JAR_TOP_Z - RING_H + 0.002  # ring straddles rim and upper neck
    model.articulation(
        "body_to_ring",
        ArticulationType.REVOLUTE,
        parent=body,
        child=ring,
        origin=Origin(xyz=(0.0, 0.0, ring_seat_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=6.28),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("jar_body")
    disk = object_model.get_part("lid_disk")
    ring = object_model.get_part("lid_ring")
    ring_joint = object_model.get_articulation("body_to_ring")
    disk_joint = object_model.get_articulation("body_to_disk")

    # The ring sits around the neck/rim and partially overlaps the jar body
    # (thread engagement region). This is intentional mechanical fit.
    ctx.allow_overlap(
        ring,
        body,
        elem_a="screw_ring",
        elem_b="jar_glass",
        reason="Screw band threads onto the jar neck — intentional thread engagement overlap.",
    )

    # The disk sits flush on top of the jar rim — small contact overlap is intended.
    ctx.allow_overlap(
        disk,
        body,
        elem_a="flat_lid",
        elem_b="jar_glass",
        reason="Flat lid disk seated on the jar rim — intentional sealing contact.",
    )

    # ---- Jar body is round-section and tall ----
    body_aabb = ctx.part_world_aabb(body)
    body_dx = body_aabb[1][0] - body_aabb[0][0]
    body_dy = body_aabb[1][1] - body_aabb[0][1]
    body_dz = body_aabb[1][2] - body_aabb[0][2]
    ctx.check(
        "jar body is round in section (X ≈ Y)",
        abs(body_dx - body_dy) < 0.005,
        details=f"dx={body_dx:.4f}, dy={body_dy:.4f}",
    )
    ctx.check(
        "jar body is tall (height > 1.5× diameter)",
        body_dz > 1.5 * max(body_dx, body_dy),
        details=f"dz={body_dz:.4f}, max_xy={max(body_dx, body_dy):.4f}",
    )

    # ---- Disk is a wide flat piece (diameter >> thickness) ----
    disk_aabb = ctx.part_world_aabb(disk)
    disk_dx = disk_aabb[1][0] - disk_aabb[0][0]
    disk_dz = disk_aabb[1][2] - disk_aabb[0][2]
    ctx.check(
        "lid disk is flat (diameter >> thickness)",
        disk_dx > 10.0 * disk_dz,
        details=f"disk_dx={disk_dx:.4f}, disk_dz={disk_dz:.4f}",
    )

    # ---- Ring is annular (has hole, wider than tall) ----
    ring_aabb = ctx.part_world_aabb(ring)
    ring_dx = ring_aabb[1][0] - ring_aabb[0][0]
    ring_dz = ring_aabb[1][2] - ring_aabb[0][2]
    ctx.check(
        "lid ring is wider than tall",
        ring_dx > 2.0 * ring_dz,
        details=f"ring_dx={ring_dx:.4f}, ring_dz={ring_dz:.4f}",
    )

    # ---- At rest (q=0): disk sits on jar mouth, ring encircles neck ----
    ctx.expect_overlap(
        ring, body,
        axes="xy",
        min_overlap=0.010,
        name="ring footprint overlaps jar neck in XY",
    )
    ctx.expect_overlap(
        disk, body,
        axes="xy",
        min_overlap=0.020,
        name="disk footprint overlaps jar mouth in XY",
    )

    # Disk is at the top of the jar
    disk_pos = ctx.part_world_position(disk)
    ctx.check(
        "disk sits near top of jar",
        disk_pos is not None and disk_pos[2] > BODY_H * 0.85,
        details=f"disk_z={disk_pos[2] if disk_pos else None}",
    )

    # ---- Ring rotation (REVOLUTE): positive q rotates ring around Z ----
    rest_ring_pos = ctx.part_world_position(ring)
    with ctx.pose({ring_joint: 3.14}):
        rotated_ring_pos = ctx.part_world_position(ring)
    ctx.check(
        "ring rotation preserves Z position (revolute around Z)",
        rest_ring_pos is not None and rotated_ring_pos is not None
        and abs(rotated_ring_pos[2] - rest_ring_pos[2]) < 0.001,
        details=f"rest_z={rest_ring_pos[2] if rest_ring_pos else None}, "
                f"rotated_z={rotated_ring_pos[2] if rotated_ring_pos else None}",
    )

    # ---- Disk lifts off (PRISMATIC along +Z): positive q lifts disk upward ----
    rest_disk_z = ctx.part_world_position(disk)[2]
    with ctx.pose({disk_joint: 0.05}):
        lifted_disk_z = ctx.part_world_position(disk)[2]
        # When disk is lifted, it should clear the jar top
        ctx.expect_gap(
            disk, body,
            axis="z",
            min_gap=0.01,
            name="lifted disk clears jar top",
        )
    ctx.check(
        "disk lifts upward (Z increases with positive q)",
        lifted_disk_z > rest_disk_z + 0.04,
        details=f"rest_z={rest_disk_z:.4f}, lifted_z={lifted_disk_z:.4f}",
    )

    # ---- Ring is revolute (non-fixed), disk is prismatic (non-fixed) ----
    ctx.check(
        "ring joint is revolute",
        ring_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={ring_joint.articulation_type}",
    )
    ctx.check(
        "disk joint is prismatic",
        disk_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={disk_joint.articulation_type}",
    )

    # ---- Materials: glass body is translucent, metal parts are opaque ----
    glass_mat = body.get_visual("jar_glass").material
    disk_mat = disk.get_visual("flat_lid").material
    ring_mat = ring.get_visual("screw_ring").material
    ctx.check(
        "jar body is clear glass material",
        glass_mat is not None and getattr(glass_mat, "name", None) == "clear_glass",
        details=f"mat={getattr(glass_mat, 'name', None)}",
    )
    ctx.check(
        "disk and ring are distinct metal materials",
        disk_mat is not None and ring_mat is not None
        and getattr(disk_mat, "name", None) != getattr(ring_mat, "name", None),
        details=f"disk={getattr(disk_mat, 'name', None)}, ring={getattr(ring_mat, 'name', None)}",
    )

    return ctx.report()


object_model = build_object_model()
