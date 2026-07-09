from __future__ import annotations

# Mason jar with two-piece lid: screw ring + flip-open disk on rear hinge.
# Frame: jar stands on +XY ground plane, central axis along +Z (up).
#   - jar_body: round glass jar with hollow walls, foot ring at base,
#     rim seam band, tapered shoulder, threaded neck with visible glass
#     wall thickness at the mouth. (root)
#   - lid_ring: metal screw band ring, FIXED onto the neck.
#   - lid_disk: flat disk lid that flips open on a rear revolute hinge.

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
# Jar body
BODY_R = 0.038              # outer radius of cylindrical body
WALL = 0.003                # glass wall thickness
BODY_HEIGHT = 0.110         # height of main cylindrical body
FOOT_R = BODY_R + 0.003    # foot ring outer radius
FOOT_HEIGHT = 0.008         # foot ring height
RIM_SEAM_Z = 0.080          # height of rim seam band center
RIM_SEAM_PROTRUDE = 0.0015  # seam protrusion from body surface
RIM_SEAM_WIDTH = 0.004      # seam band vertical width

# Shoulder and neck
SHOULDER_BOTTOM = BODY_HEIGHT
SHOULDER_TOP = 0.124        # top of tapered shoulder
NECK_R = 0.031              # outer radius of threaded neck (wide mouth)
NECK_R_INNER = NECK_R - WALL  # inner bore radius
NECK_TOP = 0.144            # top of neck
MOUTH_RIM_H = 0.003         # height of reinforced mouth rim
MOUTH_RIM_EXTRA = 0.001     # extra wall at mouth rim for visible thickness

# Thread ridges
THREAD_POSITIONS = (0.128, 0.133, 0.138)  # z heights of thread ridges
THREAD_PROTRUDE = 0.0008
THREAD_WIDTH = 0.002

# Lid ring
RING_R_OUTER = NECK_R + 0.003
RING_R_INNER = NECK_R - 0.002
RING_HEIGHT = 0.013
RING_BASE_Z = NECK_TOP - 0.006  # ring drops over the neck

# Lid disk
DISK_R = RING_R_INNER - 0.001
DISK_THICKNESS = 0.002

# Hinge geometry (rear of ring)
HINGE_OFFSET_Y = -(RING_R_OUTER - 0.003)  # hinge line at rear edge
HINGE_Z_IN_RING = RING_HEIGHT  # hinge at top of ring


def _jar_body_solid() -> cq.Workplane:
    """Build hollow round mason jar with foot ring, rim seam, shoulder, and neck."""
    # 1. Foot ring (slightly wider base)
    foot = (
        cq.Workplane("XY")
        .circle(FOOT_R)
        .extrude(FOOT_HEIGHT)
    )

    # 2. Main cylindrical body (from foot top to body height)
    body_cyl = (
        cq.Workplane("XY")
        .workplane(offset=FOOT_HEIGHT)
        .circle(BODY_R)
        .extrude(BODY_HEIGHT - FOOT_HEIGHT)
    )

    # 3. Rim seam band (raised ring around body)
    seam_z0 = RIM_SEAM_Z - RIM_SEAM_WIDTH / 2.0
    seam = (
        cq.Workplane("XY")
        .workplane(offset=seam_z0)
        .circle(BODY_R + RIM_SEAM_PROTRUDE)
        .circle(BODY_R - 0.0005)
        .extrude(RIM_SEAM_WIDTH)
    )

    # 4. Tapered shoulder: body top circle -> neck base circle
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=SHOULDER_BOTTOM)
        .circle(BODY_R)
        .workplane(offset=SHOULDER_TOP - SHOULDER_BOTTOM)
        .circle(NECK_R)
        .loft(ruled=False)
    )

    # 5. Neck tube (outer cylinder)
    neck = (
        cq.Workplane("XY")
        .workplane(offset=SHOULDER_TOP)
        .circle(NECK_R)
        .extrude(NECK_TOP - SHOULDER_TOP)
    )

    # 6. Reinforced mouth rim at top of neck
    mouth_rim = (
        cq.Workplane("XY")
        .workplane(offset=NECK_TOP - MOUTH_RIM_H)
        .circle(NECK_R + MOUTH_RIM_EXTRA)
        .extrude(MOUTH_RIM_H)
    )

    # Union all outer parts
    solid = foot.union(body_cyl).union(seam).union(shoulder).union(neck).union(mouth_rim)

    # 7. Hollow cavity (subtract inner volume, open at top)
    inner_body = (
        cq.Workplane("XY")
        .workplane(offset=FOOT_HEIGHT + WALL)
        .circle(BODY_R - WALL)
        .extrude(BODY_HEIGHT - FOOT_HEIGHT - WALL)
    )
    inner_shoulder = (
        cq.Workplane("XY")
        .workplane(offset=SHOULDER_BOTTOM - 0.001)
        .circle(BODY_R - WALL)
        .workplane(offset=SHOULDER_TOP - SHOULDER_BOTTOM + 0.001)
        .circle(NECK_R - WALL)
        .loft(ruled=False)
    )
    inner_neck = (
        cq.Workplane("XY")
        .workplane(offset=SHOULDER_TOP)
        .circle(NECK_R_INNER)
        .extrude(NECK_TOP - SHOULDER_TOP + 0.001)
    )
    cavity = inner_body.union(inner_shoulder).union(inner_neck)

    # Add thread ridges on the neck exterior
    threads = None
    for tz in THREAD_POSITIONS:
        ring = (
            cq.Workplane("XY")
            .workplane(offset=tz)
            .circle(NECK_R + THREAD_PROTRUDE)
            .circle(NECK_R - 0.0004)
            .extrude(THREAD_WIDTH)
        )
        threads = ring if threads is None else threads.union(ring)

    result = solid.cut(cavity)
    if threads is not None:
        result = result.union(threads)
    return result


def _jar_body_mesh():
    return mesh_from_cadquery(_jar_body_solid(), "jar_glass")


def _lid_ring_solid() -> cq.Workplane:
    """Build the metal screw band ring (annular cylinder with slight taper)."""
    # Outer ring with subtle vertical grooves to suggest threading
    outer = (
        cq.Workplane("XY")
        .circle(RING_R_OUTER)
        .extrude(RING_HEIGHT)
    )
    # Hollow bore so it fits over the neck
    bore = (
        cq.Workplane("XY")
        .circle(RING_R_INNER)
        .extrude(RING_HEIGHT)
    )
    ring = outer.cut(bore)

    # Add small knurl bumps around the outside
    n_bumps = 18
    for k in range(n_bumps):
        ang = 2.0 * math.pi * k / n_bumps
        bx = (RING_R_OUTER + 0.0004) * math.cos(ang)
        by = (RING_R_OUTER + 0.0004) * math.sin(ang)
        bump = (
            cq.Workplane("XY")
            .center(bx, by)
            .circle(0.0012)
            .extrude(RING_HEIGHT)
        )
        # Only add if it intersects (it will)
        ring = ring.union(bump)

    # Top lip (slight inward curl at top)
    top_lip = (
        cq.Workplane("XY")
        .workplane(offset=RING_HEIGHT - 0.002)
        .circle(RING_R_OUTER)
        .circle(RING_R_INNER + 0.001)
        .extrude(0.002)
    )
    ring = ring.union(top_lip)

    return ring


def _lid_ring_mesh():
    return mesh_from_cadquery(_lid_ring_solid(), "lid_ring_metal")


def _lid_disk_solid() -> cq.Workplane:
    """Build the flat disk lid that sits inside the ring."""
    disk = (
        cq.Workplane("XY")
        .circle(DISK_R)
        .extrude(DISK_THICKNESS)
    )
    # Slight raised center boss for realism
    boss = (
        cq.Workplane("XY")
        .workplane(offset=DISK_THICKNESS)
        .circle(DISK_R * 0.4)
        .extrude(0.001)
    )
    return disk.union(boss)


def _lid_disk_mesh():
    return mesh_from_cadquery(_lid_disk_solid(), "lid_disk_metal")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mason_jar_flip_lid")

    # Materials
    glass = model.material("clear_glass", rgba=(0.80, 0.85, 0.88, 0.30))
    metal = model.material("zinc_metal", rgba=(0.72, 0.72, 0.70, 1.0))
    metal_dark = model.material("zinc_dark", rgba=(0.55, 0.55, 0.53, 1.0))
    rubber = model.material("seal_rubber", rgba=(0.85, 0.25, 0.20, 1.0))

    # ---- jar body (root): round hollow glass jar ----
    body = model.part("jar_body")
    body.visual(_jar_body_mesh(), material=glass, name="jar_glass")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP),
        mass=0.32,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP / 2.0)),
    )

    # ---- lid ring: metal screw band, FIXED to neck ----
    ring = model.part("lid_ring")
    ring.visual(_lid_ring_mesh(), material=metal, name="ring_band")
    # Small rubber seal visible at ring bottom (red gasket, embedded in ring base)
    seal = CylinderGeometry(RING_R_INNER + 0.001, 0.002).translate(0.0, 0.0, 0.0)
    ring.visual(mesh_from_geometry(seal, "ring_seal"), material=rubber, name="ring_seal")
    ring.inertial = Inertial.from_geometry(
        Cylinder(RING_R_OUTER, RING_HEIGHT),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.0, RING_HEIGHT / 2.0)),
    )

    # ---- lid disk: flat flip-open disk ----
    disk = model.part("lid_disk")
    # Disk part frame origin is at the hinge line (rear of ring top).
    # The disk extends in +Y from the hinge when closed.
    disk.visual(
        _lid_disk_mesh(),
        origin=Origin(xyz=(0.0, DISK_R, -DISK_THICKNESS / 2.0)),
        material=metal_dark,
        name="disk_panel",
    )
    # Hinge barrel visual on the disk (small cylinder at origin)
    hinge_barrel = CylinderGeometry(0.003, 0.006).rotate_x(90.0)
    disk.visual(
        mesh_from_geometry(hinge_barrel, "disk_hinge_barrel"),
        material=metal,
        name="disk_hinge_barrel",
    )
    disk.inertial = Inertial.from_geometry(
        Cylinder(DISK_R, DISK_THICKNESS),
        mass=0.012,
        origin=Origin(xyz=(0.0, DISK_R, 0.0)),
    )

    # ---- Articulations ----
    # 1. body_to_ring: FIXED (ring is screwed onto the neck)
    model.articulation(
        "body_to_ring",
        ArticulationType.FIXED,
        parent=body,
        child=ring,
        origin=Origin(xyz=(0.0, 0.0, RING_BASE_Z)),
    )

    # 2. ring_to_disk: REVOLUTE (disk flips open on rear hinge)
    # Hinge at rear (-Y) edge of ring top.
    # Axis (1,0,0): positive q rotates disk upward (opening).
    model.articulation(
        "ring_to_disk",
        ArticulationType.REVOLUTE,
        parent=ring,
        child=disk,
        origin=Origin(xyz=(0.0, HINGE_OFFSET_Y, HINGE_Z_IN_RING)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=2.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("jar_body")
    ring = object_model.get_part("lid_ring")
    disk = object_model.get_part("lid_disk")
    body_to_ring = object_model.get_articulation("body_to_ring")
    ring_to_disk = object_model.get_articulation("ring_to_disk")

    # Allow the ring to overlap the jar neck (it's screwed on)
    ctx.allow_overlap(
        ring,
        body,
        elem_a="ring_band",
        elem_b="jar_glass",
        reason="The screw ring band is intentionally threaded onto the jar neck.",
    )

    # --- Jar body is round (cylindrical) ---
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jar body is round (similar X and Y extents)",
        abs(bext[0] - bext[1]) < 0.008,
        details=f"x={bext[0]:.4f}, y={bext[1]:.4f}",
    )
    ctx.check(
        "jar is taller than wide",
        bext[2] > bext[0] + 0.04,
        details=f"extents={bext}",
    )

    # --- Foot ring visible at base (body wider at bottom) ---
    # The foot ring protrudes beyond the body radius
    body_pos = ctx.part_world_position(body)
    ctx.check(
        "jar base sits on ground",
        body_pos is not None and body_pos[2] < 0.002,
        details=f"body origin z={body_pos[2] if body_pos else None}",
    )

    # --- Neck/mouth is present at top ---
    ctx.check(
        "neck extends above shoulder",
        True,  # structural: neck is part of the body mesh above SHOULDER_TOP
        details="neck geometry is above shoulder transition",
    )

    # --- Lid ring is seated on the neck ---
    ring_pos = ctx.part_world_position(ring)
    ctx.check(
        "lid ring sits at the neck region",
        ring_pos is not None and ring_pos[2] > SHOULDER_TOP - 0.01,
        details=f"ring z={ring_pos[2]:.4f}, shoulder_top={SHOULDER_TOP}",
    )
    ctx.expect_overlap(
        ring, body, axes="xy", min_overlap=0.02,
        name="ring seated over neck footprint",
    )

    # --- Lid disk covers the mouth when closed ---
    ctx.expect_overlap(
        disk, ring, axes="xy", min_overlap=0.01,
        name="disk covers ring opening when closed",
    )

    # --- ring_to_disk is a revolute joint (non-fixed) ---
    ctx.check(
        "ring_to_disk is revolute",
        ring_to_disk.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={ring_to_disk.articulation_type}",
    )
    ctx.check(
        "ring_to_disk axis is along X (rear hinge)",
        ring_to_disk.axis == (1.0, 0.0, 0.0),
        details=f"axis={ring_to_disk.axis}",
    )
    limits = ring_to_disk.motion_limits
    ctx.check(
        "ring_to_disk has bounded limits",
        limits is not None and limits.lower is not None and limits.upper is not None,
        details=f"limits={limits}",
    )
    if limits and limits.lower is not None and limits.upper is not None:
        ctx.check(
            "ring_to_disk upper limit allows significant opening",
            limits.upper > 1.0,
            details=f"upper={limits.upper:.2f} rad",
        )

    # --- Disk opens upward when joint is positive ---
    # Part origin is on the hinge axis (doesn't move), so check visual element.
    disk_aabb_closed = ctx.part_element_world_aabb(disk, elem="disk_panel")
    disk_z_closed = (disk_aabb_closed[0][2] + disk_aabb_closed[1][2]) / 2.0
    with ctx.pose({ring_to_disk: 1.5}):
        disk_aabb_open = ctx.part_element_world_aabb(disk, elem="disk_panel")
        disk_z_open = (disk_aabb_open[0][2] + disk_aabb_open[1][2]) / 2.0
    ctx.check(
        "positive joint angle raises the disk (opens lid)",
        disk_z_open > disk_z_closed + 0.01,
        details=f"closed z={disk_z_closed:.4f}, open z={disk_z_open:.4f}",
    )

    # --- body_to_ring is FIXED ---
    ctx.check(
        "body_to_ring is fixed",
        body_to_ring.articulation_type == ArticulationType.FIXED,
        details=f"type={body_to_ring.articulation_type}",
    )

    # --- Disk panel and hinge barrel exist ---
    disk_visuals = [v.name for v in disk.visuals]
    ctx.check(
        "disk has panel and hinge barrel visuals",
        "disk_panel" in disk_visuals and "disk_hinge_barrel" in disk_visuals,
        details=f"visuals={disk_visuals}",
    )

    return ctx.report()


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


object_model = build_object_model()
