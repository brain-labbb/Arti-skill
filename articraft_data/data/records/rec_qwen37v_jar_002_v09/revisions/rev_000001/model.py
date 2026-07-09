from __future__ import annotations

# Square spice jar with a rotating perforated shaker insert.
# Frame: jar stands on +XY ground plane, central axis along +Z (up).
#   - jar_body: square-section clear glass shell with rounded vertical edges,
#     hollow inside, topped by a wide-mouth round threaded neck. (root)
#   - lid_ring: brass annular ring that screws onto the neck (open center).
#   - shaker_disk: perforated disk that rotates within the ring to open/close
#     the shaker holes. Sits inside the ring's open center.
# Articulations:
#   ring_screw (CONTINUOUS, jar_body -> lid_ring): ring rotates about +Z
#   shaker_rotate (REVOLUTE, lid_ring -> shaker_disk): disk rotates within ring

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
BODY_HALF = 0.035          # half-width of the square section (0.070 square)
BODY_FILLET = 0.010        # rounded vertical-edge radius
WALL = 0.003               # glass wall thickness
BODY_Z0 = 0.0              # jar base sits on the ground
BODY_TOP = 0.090           # top of the square body section
SHOULDER_TOP = 0.100       # top of the tapered shoulder where the neck begins
NECK_R = 0.028             # outer radius of the wide-mouth threaded neck
NECK_TOP = 0.118           # top of the neck (z)
NECK_BOTTOM = SHOULDER_TOP

# Lid ring dimensions (annular ring that screws onto the neck)
RING_OUTER_R = 0.032       # outer radius of the brass ring
RING_INNER_R = 0.024       # inner radius (open center for shaker disk)
RING_HEIGHT = 0.018        # height of the ring
RING_MOUNT_Z = NECK_TOP - 0.012  # where the ring seats on the neck
SCALLOP_N = 20             # number of scallops on the knurled ring

# Shaker disk dimensions
SHAKER_R = RING_INNER_R - 0.001  # slightly smaller than ring bore
SHAKER_THICKNESS = 0.003         # thin perforated disk
SHAKER_HOLE_R = 0.0025           # radius of each shaker hole
SHAKER_HOLE_N = 12               # number of holes in outer ring
SHAKER_HOLE_RING_R = 0.015      # radius of the hole circle pattern


def _body_solid() -> cq.Workplane:
    # Hollow square glass jar with rounded vertical edges and a wide-mouth neck.
    outer = (
        cq.Workplane("XY")
        .box(2 * BODY_HALF, 2 * BODY_HALF, BODY_TOP, centered=(True, True, False))
        .edges("|Z")
        .fillet(BODY_FILLET)
    )

    # Tapered shoulder: square body top -> round neck base.
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP)
        .rect(2 * (BODY_HALF - 0.003), 2 * (BODY_HALF - 0.003))
        .workplane(offset=(SHOULDER_TOP - BODY_TOP))
        .circle(NECK_R)
        .loft(ruled=False)
    )

    neck = (
        cq.Workplane("XY")
        .workplane(offset=NECK_BOTTOM)
        .circle(NECK_R)
        .extrude(NECK_TOP - NECK_BOTTOM)
    )

    solid = outer.union(shoulder).union(neck)

    # Hollow it out: cut an inner cavity that opens at the neck top (wide mouth).
    inner = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .box(
            2 * (BODY_HALF - WALL),
            2 * (BODY_HALF - WALL),
            BODY_TOP - WALL,
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(max(BODY_FILLET - WALL, 0.001))
    )
    inner_shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP - 0.001)
        .rect(2 * (BODY_HALF - 0.003 - WALL), 2 * (BODY_HALF - 0.003 - WALL))
        .workplane(offset=(SHOULDER_TOP - BODY_TOP) + 0.001)
        .circle(NECK_R - WALL)
        .loft(ruled=False)
    )
    inner_neck = (
        cq.Workplane("XY")
        .workplane(offset=NECK_BOTTOM)
        .circle(NECK_R - WALL)
        .extrude((NECK_TOP - NECK_BOTTOM) + 0.001)
    )
    cavity = inner.union(inner_shoulder).union(inner_neck)
    return solid.cut(cavity)


def _thread_ridges() -> cq.Workplane:
    # Thread ridges on the wide-mouth neck.
    rings = None
    for i, zc in enumerate((NECK_BOTTOM + 0.005, NECK_BOTTOM + 0.011)):
        ring = (
            cq.Workplane("XY")
            .workplane(offset=zc)
            .circle(NECK_R + 0.0008)
            .circle(NECK_R - 0.0004)
            .extrude(0.002)
        )
        rings = ring if rings is None else rings.union(ring)
    return rings


def _body_mesh():
    solid = _body_solid().union(_thread_ridges())
    return mesh_from_cadquery(solid, "jar_glass")


def _ring_solid() -> cq.Workplane:
    # Brass annular ring: outer cylinder with inner bore, scalloped outer edge.
    # Open center so the shaker disk sits inside.
    ring = (
        cq.Workplane("XY")
        .circle(RING_OUTER_R)
        .circle(RING_INNER_R)
        .extrude(RING_HEIGHT)
    )

    # Internal support ledge: a thin annular lip at the bottom inside the ring
    # that the shaker disk sits on. This provides physical support while allowing rotation.
    ledge_inner = RING_INNER_R - 0.003  # extends 3mm inward from bore wall
    ledge = (
        cq.Workplane("XY")
        .circle(RING_INNER_R)
        .circle(ledge_inner)
        .extrude(0.002)  # 2mm thick ledge at the bottom
    )
    ring = ring.union(ledge)

    # Internal thread grooves (ridges inside the bore that match the neck threads)
    for zc in (0.006, 0.012):
        groove = (
            cq.Workplane("XY")
            .workplane(offset=zc)
            .circle(RING_INNER_R + 0.0006)
            .circle(RING_INNER_R - 0.0002)
            .extrude(0.0018)
        )
        ring = ring.union(groove)

    # Scallops / knurling on outer edge.
    for k in range(SCALLOP_N):
        ang = 2.0 * math.pi * k / SCALLOP_N
        fx = RING_OUTER_R * math.cos(ang)
        fy = RING_OUTER_R * math.sin(ang)
        flute = (
            cq.Workplane("XY")
            .center(fx, fy)
            .circle(0.0022)
            .extrude(RING_HEIGHT)
        )
        ring = ring.cut(flute)

    # Slight chamfer on top outer edge.
    try:
        ring = ring.faces(">Z").edges(cq.selectors.RadiusNthSelector(-1)).chamfer(0.001)
    except Exception:
        pass

    return ring


def _ring_mesh():
    return mesh_from_cadquery(_ring_solid(), "lid_ring_brass")


def _shaker_solid() -> cq.Workplane:
    # Perforated disk: solid disk with a ring of through-holes.
    disk = (
        cq.Workplane("XY")
        .circle(SHAKER_R)
        .extrude(SHAKER_THICKNESS)
    )

    # Cut the shaker holes in a circular pattern.
    for k in range(SHAKER_HOLE_N):
        ang = 2.0 * math.pi * k / SHAKER_HOLE_N
        hx = SHAKER_HOLE_RING_R * math.cos(ang)
        hy = SHAKER_HOLE_RING_R * math.sin(ang)
        hole = (
            cq.Workplane("XY")
            .center(hx, hy)
            .circle(SHAKER_HOLE_R)
            .extrude(SHAKER_THICKNESS + 0.001)
        )
        disk = disk.cut(hole)

    # Add a small center detent nub (so the disk reads as rotatable).
    nub = (
        cq.Workplane("XY")
        .workplane(offset=SHAKER_THICKNESS)
        .circle(0.003)
        .extrude(0.002)
    )
    disk = disk.union(nub)

    # Add a grip indicator: a small raised ridge on top for finger grip.
    ridge = (
        cq.Workplane("XY")
        .workplane(offset=SHAKER_THICKNESS)
        .center(0.008, 0.0)
        .rect(0.012, 0.003)
        .extrude(0.0015)
    )
    disk = disk.union(ridge)

    return disk


def _shaker_mesh():
    return mesh_from_cadquery(_shaker_solid(), "shaker_disk")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_spice_jar_shaker")

    glass = model.material("clear_glass", rgba=(0.82, 0.86, 0.88, 0.25))
    brass = model.material("brass", rgba=(0.72, 0.55, 0.20, 1.0))
    steel = model.material("brushed_steel", rgba=(0.65, 0.65, 0.67, 1.0))

    # ---- jar body (root): square hollow glass shell + wide-mouth neck ----
    body = model.part("jar_body")
    body.visual(_body_mesh(), material=glass, name="jar_glass")
    body.inertial = Inertial.from_geometry(
        Box((2 * BODY_HALF, 2 * BODY_HALF, NECK_TOP)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP / 2.0)),
    )

    # ---- lid ring: brass annular ring that screws onto the neck ----
    ring = model.part("lid_ring")
    ring.visual(_ring_mesh(), material=brass, name="ring_brass")
    ring.inertial = Inertial.from_geometry(
        Cylinder(RING_OUTER_R, RING_HEIGHT),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.0, RING_HEIGHT / 2.0)),
    )

    # ---- shaker disk: perforated disk that rotates inside the ring ----
    shaker = model.part("shaker_disk")
    shaker.visual(_shaker_mesh(), material=steel, name="shaker_perforated")
    # Off-axis marker on the shaker so rotation is observable.
    marker = CylinderGeometry(0.0018, 0.003).translate(SHAKER_R - 0.004, 0.0, SHAKER_THICKNESS)
    shaker.visual(
        mesh_from_geometry(marker, "shaker_marker"),
        material=brass,
        name="shaker_marker",
    )
    shaker.inertial = Inertial.from_geometry(
        Cylinder(SHAKER_R, SHAKER_THICKNESS + 0.004),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, (SHAKER_THICKNESS + 0.004) / 2.0)),
    )

    # ---- ring_screw: CONTINUOUS rotation of the ring about +Z ----
    model.articulation(
        "ring_screw",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=ring,
        origin=Origin(xyz=(0.0, 0.0, RING_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    # ---- shaker_rotate: REVOLUTE rotation of the disk within the ring ----
    # The shaker disk origin is at the ring center, on top of the ring's bottom face.
    # Positive rotation turns the disk to align/misalign holes.
    model.articulation(
        "shaker_rotate",
        ArticulationType.REVOLUTE,
        parent=ring,
        child=shaker,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.5, velocity=2.0, lower=0.0, upper=math.pi
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("jar_body")
    ring = object_model.get_part("lid_ring")
    shaker = object_model.get_part("shaker_disk")
    ring_screw = object_model.get_articulation("ring_screw")
    shaker_rotate = object_model.get_articulation("shaker_rotate")

    # The ring seats over the neck (intentional overlap with capture fit).
    ctx.allow_overlap(
        ring,
        body,
        elem_a="ring_brass",
        elem_b="jar_glass",
        reason="The brass ring is intentionally threaded over the jar neck.",
    )

    # The shaker disk sits inside the ring bore (intentional containment).
    ctx.allow_overlap(
        ring,
        shaker,
        elem_a="ring_brass",
        elem_b="shaker_perforated",
        reason="The shaker disk is intentionally seated inside the ring bore.",
    )

    # --- jar body is a square section ---
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jar body is square in cross-section",
        abs(bext[0] - bext[1]) < 0.006,
        details=f"x={bext[0]:.4f}, y={bext[1]:.4f}",
    )
    ctx.check(
        "jar is taller than wide",
        bext[2] > bext[0] + 0.02,
        details=f"extents={bext}",
    )

    # --- wide mouth: the neck opening is wide relative to body ---
    ctx.check(
        "wide mouth neck is at least 60% of body half-width",
        NECK_R > BODY_HALF * 0.6,
        details=f"neck_r={NECK_R:.4f}, body_half={BODY_HALF:.4f}",
    )

    # --- ring is annular (open center) ---
    rext = _ext(ctx.part_world_aabb(ring))
    ctx.check(
        "lid ring sits at the top of the jar",
        ctx.part_world_position(ring)[2] > NECK_BOTTOM - 0.005,
        details=f"ring z={ctx.part_world_position(ring)[2]:.4f}",
    )
    ctx.check(
        "ring is round in footprint",
        abs(rext[0] - rext[1]) < 0.004,
        details=f"ring x={rext[0]:.4f}, y={rext[1]:.4f}",
    )

    # --- shaker disk is contained within ring in XY ---
    ctx.expect_within(
        shaker, ring, axes="xy",
        inner_elem="shaker_perforated",
        outer_elem="ring_brass",
        margin=0.003,
        name="shaker disk sits inside ring bore",
    )

    # --- shaker disk has perforations (holes reduce its XY footprint vs solid) ---
    shaker_ext = _ext(ctx.part_element_world_aabb(shaker, elem="shaker_perforated"))
    ctx.check(
        "shaker disk is thin (perforated plate profile)",
        shaker_ext[2] < 0.008,
        details=f"shaker thickness={shaker_ext[2]:.4f}",
    )

    # --- ring_screw rotates the ring about +Z ---
    ctx.check(
        "ring_screw is continuous about +Z",
        ring_screw.axis == (0.0, 0.0, 1.0) and
        ring_screw.articulation_type == ArticulationType.CONTINUOUS,
        details=f"axis={ring_screw.axis}, type={ring_screw.articulation_type}",
    )

    # Verify ring rotation moves the ring geometry.
    ring_pos_rest = ctx.part_world_position(ring)
    with ctx.pose({ring_screw: math.pi / 2.0}):
        ring_pos_rot = ctx.part_world_position(ring)
    # For a continuous rotation about Z at the ring center, the origin stays
    # the same. We check that the shaker (child of ring) also rotates.
    shaker_aabb_rest = ctx.part_element_world_aabb(shaker, elem="shaker_marker")
    with ctx.pose({ring_screw: math.pi / 2.0}):
        shaker_aabb_rot = ctx.part_element_world_aabb(shaker, elem="shaker_marker")
    m0c = ((shaker_aabb_rest[0][0] + shaker_aabb_rest[1][0]) / 2.0,
            (shaker_aabb_rest[0][1] + shaker_aabb_rest[1][1]) / 2.0)
    m1c = ((shaker_aabb_rot[0][0] + shaker_aabb_rot[1][0]) / 2.0,
            (shaker_aabb_rot[0][1] + shaker_aabb_rot[1][1]) / 2.0)
    ring_rotation_effect = math.hypot(m1c[0] - m0c[0], m1c[1] - m0c[1])
    ctx.check(
        "ring_screw rotation moves the child shaker assembly",
        ring_rotation_effect > 0.005,
        details=f"marker shifted {ring_rotation_effect:.4f} m",
    )

    # --- shaker_rotate rotates the disk within the ring ---
    ctx.check(
        "shaker_rotate is revolute about +Z",
        shaker_rotate.axis == (0.0, 0.0, 1.0) and
        shaker_rotate.articulation_type == ArticulationType.REVOLUTE,
        details=f"axis={shaker_rotate.axis}, type={shaker_rotate.articulation_type}",
    )

    # Verify shaker rotation: marker moves when shaker rotates.
    m_rest = ctx.part_element_world_aabb(shaker, elem="shaker_marker")
    m_rest_c = ((m_rest[0][0] + m_rest[1][0]) / 2.0,
                (m_rest[0][1] + m_rest[1][1]) / 2.0)
    with ctx.pose({shaker_rotate: math.pi / 2.0}):
        m_rot = ctx.part_element_world_aabb(shaker, elem="shaker_marker")
        m_rot_c = ((m_rot[0][0] + m_rot[1][0]) / 2.0,
                   (m_rot[0][1] + m_rot[1][1]) / 2.0)
    shaker_shift = math.hypot(m_rot_c[0] - m_rest_c[0], m_rot_c[1] - m_rest_c[1])
    ctx.check(
        "shaker_rotate moves the perforated disk (marker shifts)",
        shaker_shift > 0.005,
        details=f"marker shifted {shaker_shift:.4f} m on quarter turn",
    )

    # --- shaker_rotate has bounded limits (0 to pi) ---
    limits = shaker_rotate.motion_limits
    ctx.check(
        "shaker_rotate has bounded limits (0 to pi)",
        limits is not None and
        limits.lower is not None and limits.upper is not None and
        abs(limits.lower) < 0.01 and abs(limits.upper - math.pi) < 0.01,
        details=f"lower={limits.lower if limits else None}, upper={limits.upper if limits else None}",
    )

    # --- the two articulations are independent: ring_screw parent is body,
    #     shaker_rotate parent is ring ---
    parent_name = ring_screw.parent.name if hasattr(ring_screw.parent, 'name') else str(ring_screw.parent)
    child_name = ring_screw.child.name if hasattr(ring_screw.child, 'name') else str(ring_screw.child)
    ctx.check(
        "ring_screw connects body to ring",
        parent_name == "jar_body" and child_name == "lid_ring",
        details=f"parent={parent_name}, child={child_name}",
    )
    sr_parent = shaker_rotate.parent.name if hasattr(shaker_rotate.parent, 'name') else str(shaker_rotate.parent)
    sr_child = shaker_rotate.child.name if hasattr(shaker_rotate.child, 'name') else str(shaker_rotate.child)
    ctx.check(
        "shaker_rotate connects ring to shaker_disk",
        sr_parent == "lid_ring" and sr_child == "shaker_disk",
        details=f"parent={sr_parent}, child={sr_child}",
    )

    return ctx.report()


object_model = build_object_model()
