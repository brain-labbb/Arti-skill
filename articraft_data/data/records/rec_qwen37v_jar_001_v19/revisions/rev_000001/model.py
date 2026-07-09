from __future__ import annotations

# SPICE JAR with rotating perforated shaker insert and flip lid.
# Frame: jar axis along +Z, base resting on z=0, jar centered on (x=0, y=0).
#
# Parts:
#   - jar_body (root): glass jar with wide mouth and threaded neck
#   - shaker_insert: perforated disk that rotates inside the neck (CONTINUOUS)
#   - flip_lid: flip-top lid on a rear revolute hinge
#   - gasket_ring: rubber gasket seated under the lid (fixed to body)
#
# Articulations:
#   - shaker_rotate: CONTINUOUS spin of the shaker about +Z at the neck top
#   - lid_hinge: REVOLUTE hinge at the rear rim, opens upward/backward

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
JAR_OUTER_R = 0.025          # outer radius of glass body (~0.05 m dia)
JAR_BODY_H = 0.070           # height of the glass body
WALL = 0.003                 # glass wall thickness
NECK_R = 0.022               # outer radius of the threaded neck (wide mouth)
NECK_H = 0.012               # neck height above shoulder
RIM_TOP_Z = JAR_BODY_H + NECK_H  # z of the neck rim top (0.082)

# Shaker insert dimensions
SHAKER_R = NECK_R - WALL - 0.001  # fits inside the neck inner wall
SHAKER_THICK = 0.0025        # disk thickness
SHAKER_HOLE_R = 0.0015       # hole radius
SHAKER_HOLE_COUNT = 12       # number of holes in ring pattern
SHAKER_RING_R = SHAKER_R * 0.6  # radius of hole ring pattern

# Flip lid dimensions
LID_R = NECK_R + 0.002       # slightly wider than neck for overhang
LID_THICK = 0.004            # lid thickness
LID_TAB_H = 0.006            # front tab height for grip

# Gasket dimensions
GASKET_R = NECK_R + 0.001    # slightly wider than neck for overhang
GASKET_THICK = 0.003         # rubber thickness
GASKET_INNER_R = NECK_R - WALL  # inner edge matches neck inner wall


def _jar_glass_solid() -> cq.Workplane:
    """Hollow thick-walled glass spice jar: tall body, shoulder into wide neck."""
    pts = [
        (0.0, 0.0),                           # center of base
        (JAR_OUTER_R, 0.0),                   # outer base edge
        (JAR_OUTER_R, JAR_BODY_H - 0.008),    # outer wall up
        (JAR_OUTER_R - 0.005, JAR_BODY_H),    # rounded outer shoulder
        (NECK_R, JAR_BODY_H + 0.003),         # step into neck
        (NECK_R, RIM_TOP_Z),                  # neck outer up to rim
        (NECK_R - WALL, RIM_TOP_Z),           # across rim top (wide mouth opening)
        (NECK_R - WALL, JAR_BODY_H - 0.002),  # inner neck wall down
        (JAR_OUTER_R - WALL, JAR_BODY_H - 0.008),
        (JAR_OUTER_R - WALL, WALL),           # inner body wall down
        (0.0, WALL),                          # across inner base
        (0.0, 0.0),                           # close
    ]
    profile = cq.Workplane("XZ").polyline(pts).close()
    jar = profile.revolve(360.0, (0, 0, 0), (0, 1, 0))

    # Inner ledge ring for the shaker to rest on (small shelf inside the neck)
    ledge_z = RIM_TOP_Z - SHAKER_THICK - 0.001  # just below where shaker sits
    ledge = (
        cq.Workplane("XY")
        .workplane(offset=ledge_z)
        .circle(NECK_R - WALL + 0.0001)
        .circle(NECK_R - WALL - 0.003)
        .extrude(0.002)
    )
    jar = jar.union(ledge)

    return jar


def _neck_threads() -> cq.Workplane:
    """Thread ridges on the neck outer wall for screw-cap appearance."""
    threads = None
    z0 = JAR_BODY_H + 0.005
    for i in range(3):
        z = z0 + i * 0.0025
        ring = (
            cq.Workplane("XY")
            .workplane(offset=z)
            .circle(NECK_R + 0.0005)
            .circle(NECK_R - 0.0003)
            .extrude(0.0014)
        )
        threads = ring if threads is None else threads.union(ring)
    return threads


def _shaker_disk() -> cq.Workplane:
    """Perforated shaker insert: circular disk with a ring of holes."""
    # Base disk
    disk = (
        cq.Workplane("XY")
        .circle(SHAKER_R)
        .extrude(SHAKER_THICK)
    )
    # Cut holes in a ring pattern
    for i in range(SHAKER_HOLE_COUNT):
        angle = 2.0 * math.pi * i / SHAKER_HOLE_COUNT
        cx = SHAKER_RING_R * math.cos(angle)
        cy = SHAKER_RING_R * math.sin(angle)
        hole = (
            cq.Workplane("XY")
            .workplane(offset=-0.001)
            .center(cx, cy)
            .circle(SHAKER_HOLE_R)
            .extrude(SHAKER_THICK + 0.002)
        )
        disk = disk.cut(hole)
    # Add a center hole too
    center_hole = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .circle(SHAKER_HOLE_R * 1.2)
        .extrude(SHAKER_THICK + 0.002)
    )
    disk = disk.cut(center_hole)
    # Asymmetric grip ridge on top: extends beyond disk radius in +X
    grip = (
        cq.Workplane("XY")
        .workplane(offset=SHAKER_THICK)
        .center(SHAKER_R * 0.5, 0.0)  # offset toward +X
        .rect(SHAKER_R * 1.4, 0.003)  # extends well beyond disk edge
        .extrude(0.002)
    )
    disk = disk.union(grip)
    return disk


def _flip_lid_solid() -> cq.Workplane:
    """Flip-top lid: flat disk cap with a front tab for opening grip.
    
    Authored in the lid part frame where origin is at the hinge pin (rear of jar).
    The lid extends forward along +Y to cover the jar opening.
    """
    # The hinge is at the rear edge. The jar opening center is at y ≈ +NECK_R
    # from the hinge. The lid disk covers the opening.
    lid_center_y = NECK_R + 0.001  # distance from hinge to opening center

    # Main lid disk
    lid = (
        cq.Workplane("XY")
        .center(0.0, lid_center_y)
        .circle(LID_R)
        .extrude(LID_THICK)
    )
    # Front tab for grip (at the front edge, +Y side)
    tab = (
        cq.Workplane("XY")
        .center(0.0, lid_center_y + LID_R + 0.003)
        .rect(0.012, 0.008)
        .extrude(LID_TAB_H)
    )
    lid = lid.union(tab)
    # Small rim around the underside for sealing contact
    rim = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .center(0.0, lid_center_y)
        .circle(LID_R - 0.001)
        .circle(LID_R - 0.003)
        .extrude(0.001)
    )
    lid = lid.union(rim)
    # Hinge ear at the rear (connects to the barrel)
    ear = (
        cq.Workplane("XY")
        .center(0.0, 0.0)
        .rect(0.008, 0.006)
        .extrude(LID_THICK + 0.002)
    )
    lid = lid.union(ear)
    return lid


def _gasket_ring() -> cq.Workplane:
    """Rubber gasket ring that sits on the jar rim under the lid."""
    ring = (
        cq.Workplane("XY")
        .circle(GASKET_R)
        .circle(GASKET_INNER_R)
        .extrude(GASKET_THICK)
    )
    return ring


def _hinge_barrel() -> cq.Workplane:
    """Hinge bracket and barrel at the rear of the jar rim, connected to the neck wall."""
    hinge_z = RIM_TOP_Z + GASKET_THICK  # z of the hinge pin center
    rear_y = -(NECK_R + 0.001)  # rear position

    # Vertical bracket plate that connects to the neck outer wall
    bracket = (
        cq.Workplane("XY")
        .workplane(offset=RIM_TOP_Z - 0.006)
        .center(0.0, rear_y)
        .rect(0.010, 0.004)
        .extrude(hinge_z - (RIM_TOP_Z - 0.006) + 0.003)
    )
    # Horizontal barrel cylinder along Y, centered on the bracket
    barrel = (
        cq.Workplane("XZ")
        .workplane(offset=rear_y - 0.006)
        .center(0.0, hinge_z)
        .circle(0.002)
        .extrude(0.012)
    )
    return bracket.union(barrel)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spice_jar")

    glass_clear = model.material("glass_clear", rgba=(0.88, 0.92, 0.95, 0.45))
    metal_silver = model.material("metal_silver", rgba=(0.72, 0.74, 0.76, 1.0))
    plastic_white = model.material("plastic_white", rgba=(0.94, 0.94, 0.92, 1.0))
    rubber_dark = model.material("rubber_dark", rgba=(0.15, 0.15, 0.18, 1.0))
    label_cream = model.material("label_cream", rgba=(0.96, 0.93, 0.82, 1.0))

    # ---- jar body (root): glass shell + neck threads + gasket + hinge barrel ----
    body = model.part("jar_body")

    glass = _jar_glass_solid().union(_neck_threads())
    body.visual(
        mesh_from_cadquery(glass, "jar_glass"),
        material=glass_clear,
        name="jar_glass",
    )

    # Gasket ring seated on the rim top
    gasket = _gasket_ring()
    body.visual(
        mesh_from_cadquery(gasket, "gasket_ring"),
        material=rubber_dark,
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP_Z)),
        name="gasket_ring",
    )

    # Hinge barrel at the rear of the rim (bracket connects to neck wall)
    hinge_barrel = _hinge_barrel()
    body.visual(
        mesh_from_cadquery(hinge_barrel, "hinge_barrel"),
        material=metal_silver,
        name="hinge_barrel",
    )

    # Label on body
    body.visual(
        Cylinder(JAR_OUTER_R + 0.0003, 0.025),
        origin=Origin(xyz=(0.0, 0.0, JAR_BODY_H * 0.45)),
        material=label_cream,
        name="spice_label",
    )

    body.inertial = Inertial.from_geometry(
        Cylinder(JAR_OUTER_R, JAR_BODY_H),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, JAR_BODY_H * 0.5)),
    )

    # ---- shaker insert: perforated disk rotating inside the neck ----
    shaker = model.part("shaker_insert")
    shaker.visual(
        mesh_from_cadquery(_shaker_disk(), "shaker_disk"),
        material=plastic_white,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="shaker_disk",
    )
    shaker.inertial = Inertial.from_geometry(
        Cylinder(SHAKER_R, SHAKER_THICK),
        mass=0.005,
        origin=Origin(xyz=(0.0, 0.0, SHAKER_THICK * 0.5)),
    )

    # Shaker rotates about +Z at the rim top (sits just below the rim)
    model.articulation(
        "shaker_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=shaker,
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP_Z - SHAKER_THICK)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=2.0),
    )

    # ---- flip lid: hinged at rear, opens upward ----
    lid = model.part("flip_lid")
    lid.visual(
        mesh_from_cadquery(_flip_lid_solid(), "lid_shell"),
        material=metal_silver,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="lid_shell",
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(LID_R, LID_THICK),
        mass=0.015,
        origin=Origin(xyz=(0.0, NECK_R + 0.001, LID_THICK * 0.5)),
    )

    # Revolute hinge at the rear of the rim.
    # Origin at the hinge pin center (rear of rim, above gasket).
    # Axis along +X: right-hand rule curls +Y toward +Z, so positive q
    # lifts the front edge of the lid upward and backward.
    # At q=0 the lid is closed (flat on top of the jar, extending +Y).
    hinge_z = RIM_TOP_Z + GASKET_THICK
    rear_y = -(NECK_R + 0.001)
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, rear_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0,
            lower=0.0, upper=2.4,  # ~137 degrees open
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("jar_body")
    shaker = object_model.get_part("shaker_insert")
    lid = object_model.get_part("flip_lid")
    shaker_joint = object_model.get_articulation("shaker_rotate")
    hinge = object_model.get_articulation("lid_hinge")

    # Allow small intentional overlap: lid seats on gasket when closed
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_shell",
        elem_b="gasket_ring",
        reason="The lid compresses against the gasket ring when closed for a seal.",
    )

    # ---- jar is taller than wide (spice jar proportions) ----
    body_aabb = ctx.part_world_aabb(body)
    dx = body_aabb[1][0] - body_aabb[0][0]
    dz = body_aabb[1][2] - body_aabb[0][2]
    ctx.check(
        "jar is taller than wide (spice jar)",
        dz > dx + 0.01,
        details=f"dx={dx:.4f}, dz={dz:.4f}",
    )

    # ---- shaker insert exists and has holes (perforated) ----
    shaker_vis = shaker.get_visual("shaker_disk")
    ctx.check(
        "shaker insert exists",
        shaker_vis is not None,
        details="shaker_disk visual not found",
    )

    # ---- shaker_rotate spins the shaker: asymmetric grip shifts AABB ----
    shaker_aabb_0 = ctx.part_element_world_aabb(shaker, elem="shaker_disk")
    # At rest the grip bar extends toward +X, making the max_x larger
    rest_max_x = shaker_aabb_0[1][0]
    with ctx.pose({shaker_joint: math.pi / 2.0}):
        shaker_aabb_1 = ctx.part_element_world_aabb(shaker, elem="shaker_disk")
        # After 90° rotation the grip bar extends toward +Y instead
        rotated_max_x = shaker_aabb_1[1][0]
    ctx.check(
        "shaker_rotate changes the shaker footprint (grip rotates)",
        abs(rotated_max_x - rest_max_x) > 0.001,
        details=f"rest_max_x={rest_max_x:.4f}, rotated_max_x={rotated_max_x:.4f}",
    )

    # ---- shaker sits inside the neck (within the body footprint) ----
    ctx.expect_within(
        shaker, body, axes="xy",
        inner_elem="shaker_disk", outer_elem="jar_glass",
        margin=0.001,
        name="shaker insert stays within the jar neck",
    )

    # ---- lid hinge opens upward: the front edge of the lid rises ----
    lid_rest_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    rest_max_z = lid_rest_aabb[1][2]  # top of closed lid
    with ctx.pose({hinge: 1.5}):  # ~86 degrees open
        lid_open_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
        open_max_z = lid_open_aabb[1][2]  # top of opened lid
        open_min_z = lid_open_aabb[0][2]  # bottom of opened lid (was the front edge)
    ctx.check(
        "lid hinge opens the lid upward",
        open_max_z > rest_max_z + 0.01,
        details=f"rest_max_z={rest_max_z:.4f}, open_max_z={open_max_z:.4f}",
    )

    # ---- lid closed position: seats near the rim ----
    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            lid, body, axes="xy", min_overlap=0.01,
            name="closed lid overlaps the jar body in XY",
        )

    # ---- gasket ring exists ----
    gasket_vis = body.get_visual("gasket_ring")
    ctx.check(
        "gasket ring exists on the jar",
        gasket_vis is not None,
        details="gasket_ring visual not found",
    )

    # ---- gasket sits at the rim top (above the jar glass) ----
    ctx.expect_gap(
        lid, body,
        axis="z",
        min_gap=-0.005, max_gap=0.010,
        positive_elem="lid_shell", negative_elem="jar_glass",
        name="lid sits near the rim when closed",
    )

    # ---- hinge barrel exists (rear hinge support) ----
    hinge_vis = body.get_visual("hinge_barrel")
    ctx.check(
        "hinge barrel exists at rear of jar",
        hinge_vis is not None,
        details="hinge_barrel visual not found",
    )

    # ---- lid hinge is revolute with proper limits ----
    ctx.check(
        "lid_hinge is revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={hinge.articulation_type}",
    )
    limits = hinge.motion_limits
    ctx.check(
        "lid_hinge has bounded limits",
        limits is not None and limits.lower is not None and limits.upper is not None,
        details=f"limits={limits}",
    )

    # ---- wide mouth: neck inner opening is clearly visible ----
    # The inner neck radius (NECK_R - WALL) should be substantial relative to body
    mouth_r = NECK_R - WALL
    ctx.check(
        "wide mouth opening (inner neck > 60% of body radius)",
        mouth_r > JAR_OUTER_R * 0.6,
        details=f"mouth_r={mouth_r:.4f}, body_r={JAR_OUTER_R:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
