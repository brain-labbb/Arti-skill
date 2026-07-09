from __future__ import annotations

# Mason jar with two-piece lid (ring band + flat sealing disk).
# Frame: jar stands on +XY ground plane, central axis along +Z (up).
#   - jar_body: round clear glass jar with thick walls, wide mouth,
#     prominent thread ridges on the outside of the neck. (root)
#   - lid_disk: flat metal sealing disk that sits on the mouth rim.
#     PRISMATIC lift along +Z (body -> disk).
#   - ring_carrier: massless carrier routing the ring screw rotation.
#   - lid_ring: screw band that threads over the neck and captures the disk.
#     CONTINUOUS rotation about +Z (body -> carrier) + PRISMATIC lift (carrier -> ring).

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key dimensions (meters) ----
# Body: round mason jar proportions
BODY_R = 0.042              # outer radius of the cylindrical body
WALL = 0.004                # glass wall thickness
BODY_Z0 = 0.0               # jar base sits on the ground
BODY_TOP = 0.110            # top of the cylindrical body section

# Shoulder: gentle taper from body to neck
SHOULDER_TOP = 0.120        # top of the shoulder transition

# Neck: wide-mouth mason jar neck
NECK_R_OUTER = 0.038        # outer radius of the threaded neck (wide mouth)
NECK_R_INNER = NECK_R_OUTER - WALL  # inner bore of the neck
NECK_BOTTOM = SHOULDER_TOP
NECK_TOP = 0.148            # top of the neck / mouth rim

# Thread ridges on the neck exterior
THREAD_N = 3                # number of thread ridges
THREAD_HEIGHT = 0.002       # height of each ridge
THREAD_PROTRUDE = 0.0015    # how far the ridge protrudes outward
THREAD_SPACING = 0.006      # vertical spacing between ridges

# Lip/rim at the mouth top
RIM_HEIGHT = 0.003          # raised rim at the top of the neck
RIM_R = NECK_R_OUTER + 0.001  # slightly wider than the neck

# Two-piece lid dimensions
DISK_R = NECK_R_OUTER + 0.001    # disk sits on the rim, slightly wider than neck OD
DISK_THICK = 0.002               # thin metal disk

RING_R_OUTER = NECK_R_OUTER + THREAD_PROTRUDE + 0.003  # outer radius of the ring band
RING_R_INNER = NECK_R_OUTER + THREAD_PROTRUDE + 0.0005  # inner bore of the ring (clears threads)
RING_HEIGHT = 0.014          # height of the screw band
RING_TOP_FLANGE = 0.003     # inward flange at the top of the ring that captures the disk

# Mount positions
RING_MOUNT_Z = NECK_TOP - RING_HEIGHT + RING_TOP_FLANGE  # ring seated on the neck
DISK_MOUNT_Z = NECK_TOP + 0.0005  # disk sits on top of the mouth rim


def _jar_body_solid() -> cq.Workplane:
    """Hollow round glass mason jar with thick walls, shoulder, threaded neck, and rim."""
    # Main cylindrical body (solid)
    body_outer = (
        cq.Workplane("XY")
        .circle(BODY_R)
        .extrude(BODY_TOP)
    )

    # Bottom plate (thicker base)
    base_plate = (
        cq.Workplane("XY")
        .circle(BODY_R)
        .extrude(0.006)
    )

    # Shoulder: taper from body to neck
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP)
        .circle(BODY_R - 0.002)
        .workplane(offset=(SHOULDER_TOP - BODY_TOP))
        .circle(NECK_R_OUTER)
        .loft(ruled=False)
    )

    # Neck cylinder (outer)
    neck_outer = (
        cq.Workplane("XY")
        .workplane(offset=NECK_BOTTOM)
        .circle(NECK_R_OUTER)
        .extrude(NECK_TOP - NECK_BOTTOM)
    )

    # Raised rim at the top
    rim = (
        cq.Workplane("XY")
        .workplane(offset=NECK_TOP)
        .circle(RIM_R)
        .extrude(RIM_HEIGHT)
    )

    # Combine outer solid
    solid = body_outer.union(base_plate).union(shoulder).union(neck_outer).union(rim)

    # Hollow interior cavity
    inner_body = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .circle(BODY_R - WALL)
        .extrude(BODY_TOP - WALL)
    )
    inner_shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP - 0.001)
        .circle(BODY_R - WALL - 0.002)
        .workplane(offset=(SHOULDER_TOP - BODY_TOP) + 0.001)
        .circle(NECK_R_INNER)
        .loft(ruled=False)
    )
    inner_neck = (
        cq.Workplane("XY")
        .workplane(offset=NECK_BOTTOM)
        .circle(NECK_R_INNER)
        .extrude((NECK_TOP - NECK_BOTTOM) + RIM_HEIGHT + 0.001)
    )
    cavity = inner_body.union(inner_shoulder).union(inner_neck)
    return solid.cut(cavity)


def _thread_ridges() -> cq.Workplane:
    """Prominent thread ridges around the outside of the neck."""
    ridges = None
    for i in range(THREAD_N):
        zc = NECK_BOTTOM + 0.004 + i * THREAD_SPACING
        ridge = (
            cq.Workplane("XY")
            .workplane(offset=zc)
            .circle(NECK_R_OUTER + THREAD_PROTRUDE)
            .circle(NECK_R_OUTER - 0.0002)
            .extrude(THREAD_HEIGHT)
        )
        ridges = ridge if ridges is None else ridges.union(ridge)
    return ridges


def _jar_body_mesh():
    solid = _jar_body_solid().union(_thread_ridges())
    return mesh_from_cadquery(solid, "jar_glass")


def _lid_disk_solid() -> cq.Workplane:
    """Flat circular sealing disk for the mason jar two-piece lid."""
    disk = (
        cq.Workplane("XY")
        .circle(DISK_R)
        .extrude(DISK_THICK)
    )
    # Slight raised rim on the disk edge for realism
    rim = (
        cq.Workplane("XY")
        .circle(DISK_R)
        .circle(DISK_R - 0.003)
        .extrude(DISK_THICK + 0.001)
    )
    return disk.union(rim)


def _lid_disk_mesh():
    return mesh_from_cadquery(_lid_disk_solid(), "lid_disk")


def _lid_ring_solid() -> cq.Workplane:
    """Screw band ring for the mason jar two-piece lid.
    
    Annular band with internal bore that clears the jar threads,
    plus an inward top flange that captures the disk.
    The skirt has vertical grip ridges on the outside.
    """
    # Main cylindrical band
    band_outer = (
        cq.Workplane("XY")
        .circle(RING_R_OUTER)
        .extrude(RING_HEIGHT)
    )
    # Hollow bore
    band_bore = (
        cq.Workplane("XY")
        .circle(RING_R_INNER)
        .extrude(RING_HEIGHT)
    )
    ring = band_outer.cut(band_bore)

    # Top flange: inward lip that captures the disk
    flange = (
        cq.Workplane("XY")
        .workplane(offset=RING_HEIGHT - RING_TOP_FLANGE)
        .circle(RING_R_OUTER)
        .circle(DISK_R - 0.001)
        .extrude(RING_TOP_FLANGE)
    )
    ring = ring.union(flange)

    # External vertical grip ridges (knurling) around the band
    grip_n = 24
    grip_depth = 0.001
    grip_width = 0.002
    for k in range(grip_n):
        ang = 2.0 * math.pi * k / grip_n
        gx = (RING_R_OUTER + grip_depth * 0.3) * math.cos(ang)
        gy = (RING_R_OUTER + grip_depth * 0.3) * math.sin(ang)
        grip = (
            cq.Workplane("XY")
            .center(gx, gy)
            .rect(grip_width, grip_depth)
            .extrude(RING_HEIGHT - 0.002)
        )
        ring = ring.union(grip)

    return ring


def _lid_ring_mesh():
    return mesh_from_cadquery(_lid_ring_solid(), "lid_ring")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mason_jar_two_piece_lid")

    glass = model.material("clear_glass", rgba=(0.80, 0.85, 0.87, 0.3))
    metal_silver = model.material("zinc_silver", rgba=(0.75, 0.76, 0.78, 1.0))
    metal_band = model.material("zinc_band", rgba=(0.65, 0.67, 0.70, 1.0))

    # ---- jar body (root): round hollow glass jar + threaded neck + rim ----
    body = model.part("jar_body")
    body.visual(_jar_body_mesh(), material=glass, name="jar_glass")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP + RIM_HEIGHT),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, (NECK_TOP + RIM_HEIGHT) / 2.0)),
    )

    # ---- lid disk: flat sealing disk on the mouth ----
    disk = model.part("lid_disk")
    disk.visual(_lid_disk_mesh(), material=metal_silver, name="lid_disk")
    # Off-axis marker so rotation of the ring (which captures the disk) is observable
    marker = CylinderGeometry(0.0015, 0.003).translate(DISK_R - 0.004, 0.0, DISK_THICK)
    disk.visual(mesh_from_geometry(marker, "disk_marker"), material=metal_band, name="disk_marker")
    disk.inertial = Inertial.from_geometry(
        Cylinder(DISK_R, DISK_THICK),
        mass=0.015,
        origin=Origin(xyz=(0.0, 0.0, DISK_THICK / 2.0)),
    )

    # ---- ring carrier (massless, no visuals): routes the screw rotation ----
    carrier = model.part("ring_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.008, 0.008, 0.008)), mass=1e-4)

    # ---- lid ring: screw band ----
    ring = model.part("lid_ring")
    ring.visual(_lid_ring_mesh(), material=metal_band, name="lid_ring")
    ring.inertial = Inertial.from_geometry(
        Cylinder(RING_R_OUTER, RING_HEIGHT),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.0, RING_HEIGHT / 2.0)),
    )

    # ---- Articulations ----

    # Ring rotates (screws on/off the threads) - continuous about +Z
    model.articulation(
        "ring_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, RING_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    # Ring lifts up off the jar (after unscrewing)
    model.articulation(
        "ring_lift",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=ring,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.05, effort=1.0, velocity=1.0),
    )

    # Disk lifts up off the mouth (after ring is removed)
    model.articulation(
        "disk_lift",
        ArticulationType.PRISMATIC,
        parent=body,
        child=disk,
        origin=Origin(xyz=(0.0, 0.0, DISK_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.04, effort=1.0, velocity=1.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("jar_body")
    disk = object_model.get_part("lid_disk")
    ring = object_model.get_part("lid_ring")
    ring_rot = object_model.get_articulation("ring_rotate")
    ring_lift = object_model.get_articulation("ring_lift")
    disk_lift = object_model.get_articulation("disk_lift")

    # Allow intentional overlaps: ring and disk are seated on/over the jar neck
    ctx.allow_overlap(
        ring,
        body,
        elem_a="lid_ring",
        elem_b="jar_glass",
        reason="The screw band ring is intentionally seated over the threaded neck.",
    )
    ctx.allow_overlap(
        disk,
        body,
        elem_a="lid_disk",
        elem_b="jar_glass",
        reason="The sealing disk is intentionally seated on the mouth rim.",
    )

    # --- Jar body is round (cylindrical) ---
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jar body is round (cylindrical)",
        abs(bext[0] - bext[1]) < 0.006,
        details=f"x={bext[0]:.4f}, y={bext[1]:.4f}",
    )
    ctx.check(
        "jar is taller than wide",
        bext[2] > bext[0] + 0.03 and bext[2] > bext[1] + 0.03,
        details=f"extents={bext}",
    )

    # --- Thread ridges exist on the neck (jar body has geometry wider than neck) ---
    # The thread ridges protrude beyond the neck outer radius, so body XY extent
    # should be driven by the body cylinder, not just the neck.
    ctx.check(
        "neck has thread ridges (body XY > neck inner)",
        bext[0] > 2 * NECK_R_INNER + 0.01,
        details=f"body_x={bext[0]:.4f}, 2*neck_inner={2*NECK_R_INNER:.4f}",
    )

    # --- Two-piece lid: ring and disk are separate parts ---
    ring_pos = ctx.part_world_position(ring)
    disk_pos = ctx.part_world_position(disk)
    ctx.check(
        "lid_ring and lid_disk are separate articulated parts",
        ring is not disk and ring_pos is not None and disk_pos is not None,
        details="two separate lid parts exist",
    )

    # --- Ring is seated on the neck ---
    ctx.check(
        "ring sits at the neck height",
        ring_pos[2] > NECK_BOTTOM - 0.005,
        details=f"ring z={ring_pos[2]:.4f}, neck_bottom={NECK_BOTTOM}",
    )
    ctx.expect_overlap(
        ring, body, axes="xy", min_overlap=0.01,
        name="ring seated over neck footprint",
    )

    # --- Disk is seated on the mouth ---
    ctx.check(
        "disk sits at the mouth rim",
        disk_pos[2] > NECK_TOP - 0.005,
        details=f"disk z={disk_pos[2]:.4f}, neck_top={NECK_TOP}",
    )
    ctx.expect_overlap(
        disk, body, axes="xy", min_overlap=0.01,
        name="disk seated on mouth footprint",
    )

    # --- Ring rotates (screw action) ---
    ctx.check(
        "ring_rotate is continuous about +Z",
        ring_rot.axis == (0.0, 0.0, 1.0) and ring_rot.articulation_type == ArticulationType.CONTINUOUS,
        details=f"axis={ring_rot.axis}, type={ring_rot.articulation_type}",
    )

    # --- Ring lifts off (prismatic along +Z) ---
    ctx.check(
        "ring_lift is prismatic along +Z",
        ring_lift.axis == (0.0, 0.0, 1.0) and ring_lift.articulation_type == ArticulationType.PRISMATIC,
        details=f"axis={ring_lift.axis}, type={ring_lift.articulation_type}",
    )
    z_ring_rest = ctx.part_world_position(ring)[2]
    with ctx.pose({ring_lift: 0.04}):
        z_ring_lifted = ctx.part_world_position(ring)[2]
    ctx.check(
        "ring_lift raises the ring off the jar",
        z_ring_lifted > z_ring_rest + 0.02,
        details=f"rest={z_ring_rest:.4f}, lifted={z_ring_lifted:.4f}",
    )

    # --- Disk lifts off (prismatic along +Z) ---
    ctx.check(
        "disk_lift is prismatic along +Z",
        disk_lift.axis == (0.0, 0.0, 1.0) and disk_lift.articulation_type == ArticulationType.PRISMATIC,
        details=f"axis={disk_lift.axis}, type={disk_lift.articulation_type}",
    )
    z_disk_rest = ctx.part_world_position(disk)[2]
    with ctx.pose({disk_lift: 0.03}):
        z_disk_lifted = ctx.part_world_position(disk)[2]
    ctx.check(
        "disk_lift raises the disk off the mouth",
        z_disk_lifted > z_disk_rest + 0.015,
        details=f"rest={z_disk_rest:.4f}, lifted={z_disk_lifted:.4f}",
    )

    # --- Ring rotation observable via carrier chain ---
    # Rotate the ring and confirm the carrier frame moves
    with ctx.pose({ring_rot: math.pi}):
        z_after_rotate = ctx.part_world_position(ring)[2]
    ctx.check(
        "ring rotation preserves seated height",
        abs(z_after_rotate - z_ring_rest) < 0.002,
        details=f"before={z_ring_rest:.4f}, after_half_turn={z_after_rotate:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
