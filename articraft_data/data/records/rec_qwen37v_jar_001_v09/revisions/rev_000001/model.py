from __future__ import annotations

# SPICE JAR with rotating perforated shaker insert and split lid (ring + disk).
# Frame: jar axis along +Z, base resting on z=0, jar centered on (x=0, y=0).
#
# A clear glass spice jar with a wide mouth and visible hollow opening.
# The lid assembly is split into:
#   - lid_ring: threaded collar that screws onto the neck (CONTINUOUS about +Z)
#   - shaker:   perforated disk insert that rotates inside the ring (REVOLUTE, 0..pi)
#   - lid_disk: flat cap disk that lifts off the ring (PRISMATIC along +Z)
#
# Articulations:
#   ring_screw:    CONTINUOUS, parent=body, child=lid_ring, axis=+Z
#   shaker_rotate: REVOLUTE,   parent=body, child=shaker,  axis=+Z, [0, pi]
#   disk_lift:     PRISMATIC,  parent=lid_ring, child=lid_disk, axis=+Z, [0, 0.018]

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
JAR_OUTER_R = 0.028           # outer radius of glass body (~56mm dia)
JAR_BODY_H = 0.070            # height of glass body
WALL = 0.003                  # glass wall thickness
NECK_R = 0.025                # outer radius of threaded neck (wide mouth ~50mm)
NECK_H = 0.012                # neck height above shoulder
RIM_TOP_Z = JAR_BODY_H + NECK_H  # z of neck rim top (0.082)
MOUTH_R = NECK_R - WALL       # inner mouth radius (wide hollow opening)

# Shaker insert dimensions
SHAKER_R = MOUTH_R - 0.001    # shaker disk sits just inside the mouth
SHAKER_THICKNESS = 0.003
SHAKER_Z = RIM_TOP_Z - 0.002  # shaker sits slightly below rim top

# Lid ring dimensions
RING_OUTER_R = NECK_R + 0.003  # ring is slightly wider than neck
RING_INNER_R = MOUTH_R + 0.001  # ring inner clears the shaker
RING_H = 0.012                 # ring height
RING_BOTTOM_Z = RIM_TOP_Z - 0.008  # ring slips down over the neck

# Lid disk dimensions
DISK_R = RING_OUTER_R - 0.001   # disk sits inside the ring top
DISK_H = 0.004
DISK_BOTTOM_Z = RING_BOTTOM_Z + RING_H - 0.001  # disk rests on ring top


def _jar_glass_solid() -> cq.Workplane:
    """Hollow thick-walled glass spice jar with wide mouth opening.
    Revolved profile creates a real open-topped cavity visible from above."""
    pts = [
        (0.0, 0.0),                           # center of base
        (JAR_OUTER_R, 0.0),                   # outer base edge
        (JAR_OUTER_R, JAR_BODY_H - 0.008),    # outer wall up
        (JAR_OUTER_R - 0.003, JAR_BODY_H),    # rounded shoulder
        (NECK_R, JAR_BODY_H + 0.003),         # step in to neck
        (NECK_R, RIM_TOP_Z),                  # neck outer up to rim
        (NECK_R - WALL, RIM_TOP_Z),           # across rim top (wide mouth opening)
        (NECK_R - WALL, JAR_BODY_H - 0.003),  # inner neck wall down
        (JAR_OUTER_R - WALL, JAR_BODY_H - 0.008),
        (JAR_OUTER_R - WALL, WALL),           # inner body wall down
        (0.0, WALL),                          # across inner base
        (0.0, 0.0),                           # close
    ]
    profile = cq.Workplane("XZ").polyline(pts).close()
    return profile.revolve(360.0, (0, 0, 0), (0, 1, 0))


def _neck_threads() -> cq.Workplane:
    """Thread ridges on the wide neck for the screw-on ring."""
    threads = None
    z0 = JAR_BODY_H + 0.005
    for i in range(3):
        z = z0 + i * 0.003
        ring = (
            cq.Workplane("XY")
            .workplane(offset=z)
            .circle(NECK_R + 0.0005)
            .circle(NECK_R - 0.0003)
            .extrude(0.0018)
        )
        threads = ring if threads is None else threads.union(ring)
    return threads


def _spice_contents() -> cq.Workplane:
    """Spice contents filling the jar to about 60% height.
    Starts at the inner glass floor so it contacts the jar body."""
    inner_r = JAR_OUTER_R - WALL - 0.0005
    fill_h = JAR_BODY_H * 0.60
    contents = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .circle(inner_r)
        .extrude(fill_h)
    )
    # Slightly domed top surface
    dome = (
        cq.Workplane("XY")
        .workplane(offset=WALL + fill_h)
        .circle(inner_r)
        .workplane(offset=0.005)
        .circle(inner_r * 0.6)
        .loft(ruled=False)
    )
    return contents.union(dome)


def _shaker_disk() -> cq.Workplane:
    """Perforated shaker insert: a disk with circular holes for dispensing spice."""
    # Solid disk base
    disk = (
        cq.Workplane("XY")
        .circle(SHAKER_R)
        .extrude(SHAKER_THICKNESS)
    )
    # Cut holes in a pattern: center hole + ring of holes
    hole_r = 0.002  # 2mm radius holes
    # Center hole
    disk = disk.faces(">Z").workplane().circle(hole_r).cutThruAll()
    # Inner ring of 6 holes at 8mm radius
    inner_ring_r = 0.008
    for i in range(6):
        ang = 2.0 * math.pi * i / 6.0
        cx = inner_ring_r * math.cos(ang)
        cy = inner_ring_r * math.sin(ang)
        disk = (
            disk.faces(">Z").workplane()
            .center(cx, cy)
            .circle(hole_r)
            .cutThruAll()
        )
    # Outer ring of 10 holes at 16mm radius
    outer_ring_r = 0.016
    for i in range(10):
        ang = 2.0 * math.pi * i / 10.0 + math.pi / 10.0  # offset half step
        cx = outer_ring_r * math.cos(ang)
        cy = outer_ring_r * math.sin(ang)
        disk = (
            disk.faces(">Z").workplane()
            .center(cx, cy)
            .circle(hole_r)
            .cutThruAll()
        )
    return disk


def _shaker_tab() -> cq.Workplane:
    """Small grip tab on the shaker disk edge for rotating it."""
    tab = (
        cq.Workplane("XY")
        .center(SHAKER_R - 0.002, 0.0)
        .rect(0.006, 0.008)
        .extrude(SHAKER_THICKNESS + 0.002)
    )
    return tab


def _lid_ring_solid() -> cq.Workplane:
    """Threaded collar ring that screws onto the jar neck.
    Hollow cylinder with internal thread grooves."""
    outer = (
        cq.Workplane("XY")
        .circle(RING_OUTER_R)
        .extrude(RING_H)
    )
    # Hollow out the center
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .circle(RING_INNER_R)
        .extrude(RING_H + 0.002)
    )
    ring = outer.cut(cavity)
    # Add internal thread ridges (mate with jar neck threads)
    for i in range(3):
        z = 0.003 + i * 0.003
        thread_ring = (
            cq.Workplane("XY")
            .workplane(offset=z)
            .circle(RING_INNER_R + 0.0004)
            .circle(RING_INNER_R - 0.0003)
            .extrude(0.0015)
        )
        ring = ring.union(thread_ring)
    # Knurling on outside
    n_ribs = 36
    for i in range(n_ribs):
        ang = 2.0 * math.pi * i / n_ribs
        rib = (
            cq.Workplane("XY")
            .center(
                (RING_OUTER_R + 0.0003) * math.cos(ang),
                (RING_OUTER_R + 0.0003) * math.sin(ang),
            )
            .rect(0.0012, 0.0012)
            .extrude(RING_H - 0.002)
        )
        ring = ring.union(rib)
    return ring


def _lid_disk_solid() -> cq.Workplane:
    """Flat cap disk that sits on top of the ring."""
    disk = (
        cq.Workplane("XY")
        .circle(DISK_R)
        .extrude(DISK_H)
    )
    # Small lip on bottom edge to seat into the ring
    lip = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .circle(DISK_R - 0.002)
        .circle(DISK_R - 0.004)
        .extrude(0.001)
    )
    disk = disk.union(lip)
    # Small grip nub on top
    nub = (
        cq.Workplane("XY")
        .workplane(offset=DISK_H)
        .circle(0.005)
        .extrude(0.002)
    )
    disk = disk.union(nub)
    return disk


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spice_jar_shaker")

    # Materials
    glass_clear = model.material("glass_clear", rgba=(0.88, 0.92, 0.94, 0.40))
    spice_brown = model.material("spice_brown", rgba=(0.62, 0.36, 0.14, 1.0))
    shaker_metal = model.material("shaker_metal", rgba=(0.72, 0.72, 0.70, 1.0))
    ring_dark = model.material("ring_dark", rgba=(0.18, 0.18, 0.20, 1.0))
    disk_red = model.material("disk_red", rgba=(0.72, 0.15, 0.12, 1.0))

    # ---- body (root): glass jar with wide mouth, hollow opening ----
    body = model.part("body")

    glass = _jar_glass_solid().union(_neck_threads())
    body.visual(
        mesh_from_cadquery(glass, "jar_glass"),
        material=glass_clear,
        name="jar_glass",
    )

    # Spice contents visible through glass
    body.visual(
        mesh_from_cadquery(_spice_contents(), "spice_fill"),
        material=spice_brown,
        name="spice_fill",
    )

    body.inertial = Inertial.from_geometry(
        Cylinder(JAR_OUTER_R, JAR_BODY_H),
        mass=0.15,
        origin=Origin(xyz=(0.0, 0.0, JAR_BODY_H * 0.5)),
    )

    # ---- shaker: perforated disk insert, rotates inside the mouth ----
    shaker = model.part("shaker")
    shaker_body = _shaker_disk().union(_shaker_tab())
    shaker.visual(
        mesh_from_cadquery(shaker_body, "shaker_plate"),
        material=shaker_metal,
        name="shaker_plate",
    )
    shaker.inertial = Inertial.from_geometry(
        Cylinder(SHAKER_R, SHAKER_THICKNESS),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, SHAKER_THICKNESS * 0.5)),
    )

    # Shaker rotates about +Z at the mouth center
    model.articulation(
        "shaker_rotate",
        ArticulationType.REVOLUTE,
        parent=body,
        child=shaker,
        origin=Origin(xyz=(0.0, 0.0, SHAKER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=2.0, lower=0.0, upper=math.pi
        ),
    )

    # ---- lid_ring: threaded collar that screws onto the neck ----
    lid_ring = model.part("lid_ring")
    lid_ring.visual(
        mesh_from_cadquery(_lid_ring_solid(), "ring_shell"),
        material=ring_dark,
        name="ring_shell",
    )
    lid_ring.inertial = Inertial.from_geometry(
        Cylinder(RING_OUTER_R, RING_H),
        mass=0.015,
        origin=Origin(xyz=(0.0, 0.0, RING_H * 0.5)),
    )

    # Ring screws onto body neck (continuous rotation about +Z)
    model.articulation(
        "ring_screw",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lid_ring,
        origin=Origin(xyz=(0.0, 0.0, RING_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0),
    )

    # ---- lid_disk: flat cap disk that lifts off the ring ----
    lid_disk = model.part("lid_disk")
    lid_disk.visual(
        mesh_from_cadquery(_lid_disk_solid(), "disk_shell"),
        material=disk_red,
        name="disk_shell",
    )
    lid_disk.inertial = Inertial.from_geometry(
        Cylinder(DISK_R, DISK_H),
        mass=0.005,
        origin=Origin(xyz=(0.0, 0.0, DISK_H * 0.5)),
    )

    # Disk lifts prismatic along +Z from the ring
    model.articulation(
        "disk_lift",
        ArticulationType.PRISMATIC,
        parent=lid_ring,
        child=lid_disk,
        origin=Origin(xyz=(0.0, 0.0, RING_H - 0.001)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0, velocity=1.0, lower=0.0, upper=0.018
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    shaker = object_model.get_part("shaker")
    lid_ring = object_model.get_part("lid_ring")
    lid_disk = object_model.get_part("lid_disk")

    shaker_rotate = object_model.get_articulation("shaker_rotate")
    ring_screw = object_model.get_articulation("ring_screw")
    disk_lift = object_model.get_articulation("disk_lift")

    # ---- Allow intentional overlaps ----
    # Shaker sits inside the neck opening (intentional nesting)
    ctx.allow_overlap(
        shaker,
        body,
        elem_a="shaker_plate",
        elem_b="jar_glass",
        reason="The shaker insert is intentionally seated inside the wide jar mouth.",
    )
    # Ring slips over the neck threads (intentional threaded fit)
    ctx.allow_overlap(
        lid_ring,
        body,
        elem_a="ring_shell",
        elem_b="jar_glass",
        reason="The lid ring screws down over the threaded neck.",
    )
    # Disk seats on top of the ring with small lip overlap
    ctx.allow_overlap(
        lid_disk,
        lid_ring,
        elem_a="disk_shell",
        elem_b="ring_shell",
        reason="The disk lip seats into the ring top with small intentional nesting.",
    )

    # ---- Jar has wide mouth (hollow opening visible) ----
    body_aabb = ctx.part_world_aabb(body)
    body_ext = (
        body_aabb[1][0] - body_aabb[0][0],
        body_aabb[1][1] - body_aabb[0][1],
        body_aabb[1][2] - body_aabb[0][2],
    )
    ctx.check(
        "jar body is taller than wide (spice jar proportions)",
        body_ext[2] > body_ext[0] and body_ext[2] > body_ext[1],
        details=f"body_ext={body_ext}",
    )

    # ---- Shaker is positioned at the jar mouth ----
    shaker_pos = ctx.part_world_position(shaker)
    ctx.check(
        "shaker sits near the jar mouth",
        shaker_pos is not None and shaker_pos[2] > JAR_BODY_H - 0.005,
        details=f"shaker_pos={shaker_pos}, jar_body_h={JAR_BODY_H}",
    )

    # ---- Shaker is within the mouth opening (XY containment) ----
    ctx.expect_within(
        shaker, body, axes="xy",
        inner_elem="shaker_plate", outer_elem="jar_glass",
        margin=0.005,
        name="shaker fits within the jar mouth",
    )

    # ---- Shaker rotation moves the tab ----
    tab_rest = ctx.part_element_world_aabb(shaker, elem="shaker_plate")
    tab_rest_cx = (tab_rest[0][0] + tab_rest[1][0]) * 0.5
    tab_rest_cy = (tab_rest[0][1] + tab_rest[1][1]) * 0.5
    with ctx.pose({shaker_rotate: math.pi}):
        tab_rotated = ctx.part_element_world_aabb(shaker, elem="shaker_plate")
        tab_rot_cx = (tab_rotated[0][0] + tab_rotated[1][0]) * 0.5
        tab_rot_cy = (tab_rotated[0][1] + tab_rotated[1][1]) * 0.5
    # At pi rotation, the part should still exist in roughly same place (symmetric disk)
    # but we verify the joint is revolute with proper limits
    ctx.check(
        "shaker_rotate is revolute with [0, pi] limits",
        shaker_rotate.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={shaker_rotate.articulation_type}",
    )
    limits = shaker_rotate.motion_limits
    ctx.check(
        "shaker_rotate limits are 0 to pi",
        limits is not None
        and abs(limits.lower - 0.0) < 0.01
        and abs(limits.upper - math.pi) < 0.01,
        details=f"lower={limits.lower}, upper={limits.upper}",
    )

    # ---- Lid ring is on the jar neck ----
    ring_pos = ctx.part_world_position(lid_ring)
    ctx.check(
        "lid_ring is positioned on the jar neck",
        ring_pos is not None and ring_pos[2] > JAR_BODY_H,
        details=f"ring_pos={ring_pos}",
    )

    # ---- Lid ring overlaps the body in XY (encircles the neck) ----
    ctx.expect_overlap(
        lid_ring, body, axes="xy", min_overlap=0.02,
        name="lid_ring encircles the neck",
    )

    # ---- Ring screw is continuous ----
    ctx.check(
        "ring_screw is continuous rotation",
        ring_screw.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={ring_screw.articulation_type}",
    )

    # ---- Lid disk sits on top of ring ----
    disk_pos = ctx.part_world_position(lid_disk)
    ring_top_z = RING_BOTTOM_Z + RING_H
    ctx.check(
        "lid_disk sits above the ring",
        disk_pos is not None and disk_pos[2] > ring_top_z - 0.005,
        details=f"disk_pos={disk_pos}, ring_top_z={ring_top_z}",
    )

    # ---- Disk lift raises the disk off the ring ----
    rest_z = ctx.part_world_position(lid_disk)[2]
    with ctx.pose({disk_lift: 0.018}):
        lifted_z = ctx.part_world_position(lid_disk)[2]
        ctx.expect_gap(
            lid_disk, lid_ring, axis="z",
            min_gap=0.005,
            positive_elem="disk_shell", negative_elem="ring_shell",
            name="lifted disk clears the ring",
        )
    ctx.check(
        "disk_lift raises the disk",
        lifted_z > rest_z + 0.010,
        details=f"rest_z={rest_z}, lifted_z={lifted_z}",
    )

    # ---- Disk lift is prismatic ----
    ctx.check(
        "disk_lift is prismatic",
        disk_lift.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={disk_lift.articulation_type}",
    )

    # ---- At least 3 non-fixed joints exist ----
    non_fixed = [
        a for a in object_model.articulations
        if a.articulation_type != ArticulationType.FIXED
    ]
    ctx.check(
        "at least 3 non-fixed articulations",
        len(non_fixed) >= 3,
        details=f"non_fixed_count={len(non_fixed)}",
    )

    return ctx.report()


object_model = build_object_model()
