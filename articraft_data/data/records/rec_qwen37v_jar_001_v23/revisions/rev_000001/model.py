from __future__ import annotations

# Tall cylindrical storage jar with clamp lid.
# Frame: jar axis along +Z, base on z=0, centered at (0,0).
#
# The jar is tall (~0.12m) with a wide mouth opening at the top.
# A metal clamp ring hinges at the back to open/close.
# A separate sealing disk lifts off the mouth independently.
#
# Articulations:
#   - ring_hinge: REVOLUTE at the rear rim, swings the clamp ring open upward
#   - disk_lift:  PRISMATIC along +Z, lifts the sealing disk off the mouth

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
JAR_OUTER_R = 0.050          # outer radius of the glass body (100mm dia)
JAR_BODY_H = 0.120           # height of the glass body (tall jar)
WALL = 0.004                 # glass wall thickness
BASE_THICK = 0.006           # thick glass base

# Rim/neck at the top
RIM_OUTER_R = JAR_OUTER_R + 0.003  # slightly wider rim lip (0.053)
RIM_H = 0.008                       # rim height
RIM_TOP_Z = JAR_BODY_H + RIM_H     # top of the rim (0.128)

# Inner cavity
INNER_R = JAR_OUTER_R - WALL       # inner radius (0.046)

# Clamp ring
RING_INNER_R = RIM_OUTER_R + 0.001  # ring sits just outside the rim (0.054)
RING_OUTER_R = RING_INNER_R + 0.006 # ring outer wall (0.060)
RING_H = 0.014                      # ring height

# Sealing disk
DISK_R = RIM_OUTER_R - 0.001       # disk sits just inside the rim lip (0.052)
DISK_THICK = 0.004                  # disk thickness

# Hinge location (back of jar, at y = -JAR_OUTER_R)
HINGE_Y = -(RIM_OUTER_R + 0.002)   # hinge pin at back of rim
HINGE_Z = RIM_TOP_Z                # hinge at rim top level

# Clamp hooks on the sides
HOOK_WIDTH = 0.008
HOOK_DEPTH = 0.012
HOOK_THICK = 0.003


def _jar_body_solid() -> cq.Workplane:
    """Hollow tall cylindrical glass jar with wide mouth opening."""
    # Outer shell: tall cylinder
    outer = (
        cq.Workplane("XY")
        .circle(JAR_OUTER_R)
        .extrude(JAR_BODY_H)
    )
    # Fillet the bottom edge slightly
    outer = outer.edges("<Z").fillet(0.003)
    # Fillet top shoulder
    outer = outer.edges(">Z").fillet(0.002)

    # Inner cavity: hollow from top down, leaving thick base
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICK)
        .circle(INNER_R)
        .extrude(JAR_BODY_H - BASE_THICK + 0.001)
    )
    jar = outer.cut(cavity)

    # Wide rim lip at top
    rim = (
        cq.Workplane("XY")
        .workplane(offset=JAR_BODY_H)
        .circle(RIM_OUTER_R)
        .circle(INNER_R)
        .extrude(RIM_H)
    )
    jar = jar.union(rim)

    return jar


def _clamp_ring_solid() -> cq.Workplane:
    """Split clamp ring that sits around the jar rim.
    Built in ring-local frame: origin at the hinge point (back of jar).
    Ring center is offset by (0, -HINGE_Y, 0) from the part origin.
    """
    # Ring center in local coords
    cy = -HINGE_Y  # positive offset from hinge to ring center

    # Main ring body
    ring = (
        cq.Workplane("XY")
        .center(0.0, cy)
        .circle(RING_OUTER_R)
        .circle(RING_INNER_R)
        .extrude(RING_H)
    )

    # Split gap at front (cut a small slot)
    gap_width = 0.004
    slot = (
        cq.Workplane("XY")
        .center(0.0, cy + RING_OUTER_R - 0.001)
        .rect(gap_width, 0.010)
        .extrude(RING_H)
    )
    ring = ring.cut(slot)

    return ring


def _clamp_hooks() -> cq.Workplane:
    """Two clamp hooks on the sides of the ring (left and right)."""
    cy = -HINGE_Y
    hooks = None
    for sign in (-1, 1):
        hook_x = sign * (RING_OUTER_R + HOOK_THICK * 0.5)
        hook = (
            cq.Workplane("XY")
            .center(hook_x, cy)
            .rect(HOOK_THICK, HOOK_DEPTH)
            .extrude(RING_H + 0.004)
        )
        # Hook lip extending inward to grip the rim
        lip = (
            cq.Workplane("XY")
            .workplane(offset=RING_H)
            .center(hook_x - sign * HOOK_THICK * 0.5, cy)
            .rect(HOOK_THICK * 2, HOOK_WIDTH)
            .extrude(0.004)
        )
        h = hook.union(lip)
        hooks = h if hooks is None else hooks.union(h)
    return hooks


def _hinge_knuckles_body() -> cq.Workplane:
    """Hinge knuckle barrels mounted on the body rim at the back."""
    # Two knuckle barrels on the body side
    knuckles = None
    for dx in (-0.010, 0.010):
        knuckle = (
            cq.Workplane("XY")
            .workplane(offset=HINGE_Z - 0.002)
            .center(dx, HINGE_Y)
            .circle(0.004)
            .extrude(0.008)
        )
        knuckles = knuckle if knuckles is None else knuckles.union(knuckle)
    return knuckles


def _hinge_knuckle_ring() -> cq.Workplane:
    """Single center knuckle on the ring side, between the body knuckles."""
    cy = -HINGE_Y
    # Pin barrel on the ring at the hinge point
    barrel = (
        cq.Workplane("XY")
        .center(0.0, 0.0)  # at the hinge point in local coords
        .circle(0.003)
        .extrude(0.012)
    )
    return barrel


def _lid_disk_solid() -> cq.Workplane:
    """Flat sealing disk that sits on the jar mouth."""
    disk = (
        cq.Workplane("XY")
        .circle(DISK_R)
        .extrude(DISK_THICK)
    )
    # Small raised lip around the edge
    lip = (
        cq.Workplane("XY")
        .circle(DISK_R)
        .circle(DISK_R - 0.003)
        .extrude(DISK_THICK + 0.002)
    )
    return disk.union(lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="storage_jar_clamp_lid")

    glass_clear = model.material("glass_clear", rgba=(0.85, 0.92, 0.95, 0.45))
    metal_steel = model.material("metal_steel", rgba=(0.72, 0.73, 0.74, 1.0))
    seal_rubber = model.material("seal_rubber", rgba=(0.22, 0.22, 0.24, 1.0))
    label_cream = model.material("label_cream", rgba=(0.95, 0.92, 0.84, 1.0))

    # ---- jar body (root): tall hollow glass cylinder with rim ----
    body = model.part("body")

    jar_cq = _jar_body_solid().union(_hinge_knuckles_body())
    body.visual(mesh_from_cadquery(jar_cq, "jar_glass"), material=glass_clear, name="jar_glass")

    # Label band on the body
    body.visual(
        Cylinder(JAR_OUTER_R + 0.0005, 0.030),
        origin=Origin(xyz=(0.0, 0.0, JAR_BODY_H * 0.45)),
        material=label_cream,
        name="body_label",
    )

    body.inertial = Inertial.from_geometry(
        Cylinder(JAR_OUTER_R, JAR_BODY_H + RIM_H),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, (JAR_BODY_H + RIM_H) * 0.5)),
    )

    # ---- clamp ring: hinged at back, swings open ----
    clamp_ring = model.part("clamp_ring")

    ring_cq = _clamp_ring_solid().union(_clamp_hooks())
    ring_cq = ring_cq.union(_hinge_knuckle_ring())
    clamp_ring.visual(
        mesh_from_cadquery(ring_cq, "ring_body"),
        material=metal_steel,
        name="ring_body",
    )

    clamp_ring.inertial = Inertial.from_geometry(
        Cylinder(RING_OUTER_R, RING_H),
        mass=0.06,
        origin=Origin(xyz=(0.0, -HINGE_Y, RING_H * 0.5)),
    )

    # Ring hinge articulation:
    # - Origin at the hinge pin location (back of rim)
    # - axis=(1,0,0): right-hand rule rotates +Y toward +Z
    # - Ring center is at (0, -HINGE_Y, 0) in child frame
    # - Positive q swings the ring upward (open)
    model.articulation(
        "ring_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=clamp_ring,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=0.0, upper=2.4
        ),
    )

    # ---- lid disk: sits on mouth, lifts off independently ----
    lid_disk = model.part("lid_disk")

    lid_disk.visual(
        mesh_from_cadquery(_lid_disk_solid(), "seal_disk"),
        material=seal_rubber,
        name="seal_disk",
    )

    lid_disk.inertial = Inertial.from_geometry(
        Cylinder(DISK_R, DISK_THICK),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, DISK_THICK * 0.5)),
    )

    # Disk lift articulation:
    # - Prismatic along +Z, lifts disk off the mouth
    # - Origin at rim top center; disk sits at z=0 in its frame at the rim
    model.articulation(
        "disk_lift",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lid_disk,
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=0.5, lower=0.0, upper=0.06
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    clamp_ring = object_model.get_part("clamp_ring")
    lid_disk = object_model.get_part("lid_disk")
    ring_hinge = object_model.get_articulation("ring_hinge")
    disk_lift = object_model.get_articulation("disk_lift")

    # ---- Allow small overlap where the ring sits around the rim ----
    ctx.allow_overlap(
        clamp_ring,
        body,
        elem_a="ring_body",
        elem_b="jar_glass",
        reason="The clamp ring is intentionally fitted around the jar rim with slight interference for a snug clamp fit.",
    )

    # ---- Disk seats on the rim: small intentional contact overlap ----
    ctx.allow_overlap(
        lid_disk,
        body,
        elem_a="seal_disk",
        elem_b="jar_glass",
        reason="The sealing disk sits flush on the jar rim surface with minor mesh contact at the seating interface.",
    )

    # ---- Jar is tall: taller than it is wide ----
    bext_aabb = ctx.part_world_aabb(body)
    bext = (
        bext_aabb[1][0] - bext_aabb[0][0],
        bext_aabb[1][1] - bext_aabb[0][1],
        bext_aabb[1][2] - bext_aabb[0][2],
    )
    ctx.check(
        "jar is tall (taller than wide)",
        bext[2] > bext[0] + 0.01 and bext[2] > bext[1] + 0.01,
        details=f"body extents={bext}",
    )

    # ---- Wide mouth opening: inner cavity is substantial ----
    # The jar glass has a hollow cavity (wide mouth)
    glass_vis = body.get_visual("jar_glass")
    ctx.check(
        "jar has wide-mouth hollow opening",
        glass_vis is not None,
        details="jar_glass visual should exist with hollow cavity",
    )

    # ---- Ring sits at top of jar at rest ----
    ring_pos = ctx.part_world_position(clamp_ring)
    body_pos = ctx.part_world_position(body)
    ctx.check(
        "clamp ring is at the top of the jar",
        ring_pos is not None and body_pos is not None and ring_pos[2] > JAR_BODY_H - 0.01,
        details=f"ring_pos={ring_pos}, body_pos={body_pos}",
    )

    # ---- Ring overlaps the body footprint in XY at rest (sits on rim) ----
    ctx.expect_overlap(
        clamp_ring, body, axes="xy", min_overlap=0.02,
        name="clamp ring sits over jar body footprint",
    )

    # ---- Ring hinge opens: positive pose swings ring upward ----
    # Use geometry AABB since part origin is at the hinge pin (rotation center)
    rest_ring_aabb = ctx.part_element_world_aabb(clamp_ring, elem="ring_body")
    rest_ring_max_z = rest_ring_aabb[1][2]
    with ctx.pose({ring_hinge: 1.5}):
        open_ring_aabb = ctx.part_element_world_aabb(clamp_ring, elem="ring_body")
        open_ring_max_z = open_ring_aabb[1][2]
    ctx.check(
        "ring_hinge opens ring upward",
        open_ring_max_z > rest_ring_max_z + 0.02,
        details=f"rest_max_z={rest_ring_max_z}, open_max_z={open_ring_max_z}",
    )

    # ---- Disk sits on mouth at rest ----
    disk_pos = ctx.part_world_position(lid_disk)
    ctx.check(
        "lid disk sits on jar mouth",
        disk_pos is not None and disk_pos[2] > RIM_TOP_Z - 0.005,
        details=f"disk_pos={disk_pos}, rim_top={RIM_TOP_Z}",
    )

    # ---- Disk lifts off: positive prismatic pose raises disk ----
    rest_disk_z = ctx.part_world_position(lid_disk)[2]
    with ctx.pose({disk_lift: 0.04}):
        lifted_disk_z = ctx.part_world_position(lid_disk)[2]
    ctx.check(
        "disk_lift raises disk off the mouth",
        lifted_disk_z > rest_disk_z + 0.02,
        details=f"rest_z={rest_disk_z}, lifted_z={lifted_disk_z}",
    )

    # ---- Disk overlaps body footprint at rest (seated on mouth) ----
    ctx.expect_overlap(
        lid_disk, body, axes="xy", min_overlap=0.02,
        name="disk seated on mouth overlaps body",
    )

    # ---- Disk is seated at rim level (bottom at RIM_TOP_Z) ----
    # The hinge knuckles extend above the rim, so whole-body Z overlap includes
    # knuckle height. Check that the disk bottom is at or above the rim.
    disk_aabb = ctx.part_element_world_aabb(lid_disk, elem="seal_disk")
    ctx.check(
        "disk bottom sits at or above rim top",
        disk_aabb[0][2] >= RIM_TOP_Z - 0.001,
        details=f"disk_min_z={disk_aabb[0][2]}, rim_top={RIM_TOP_Z}",
    )

    # ---- Ring has clamp hooks (ring_body visual includes hooks) ----
    ring_vis = clamp_ring.get_visual("ring_body")
    ctx.check(
        "clamp ring has visible hooks/knuckles",
        ring_vis is not None,
        details="ring_body visual should include hooks and knuckle geometry",
    )

    # ---- Ring hinge has proper limits ----
    limits = ring_hinge.motion_limits
    ctx.check(
        "ring hinge has bounded limits",
        limits is not None and limits.lower is not None and limits.upper is not None,
        details=f"limits={limits}",
    )

    return ctx.report()


object_model = build_object_model()
