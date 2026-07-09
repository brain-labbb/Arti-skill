from __future__ import annotations

# Spice jar variant: glass jar with wide mouth, rotating perforated shaker
# insert, screw-on lid with hinge knuckles, and clamp hooks on the neck.
#
# Frame: jar axis along +Z, base resting on z=0, jar centered on (x=0, y=0).
#
# Parts:
#   body (root): glass jar shell with wide-mouth hollow opening, clamp hooks
#   shaker: perforated disc insert seated in the mouth
#   lid: screw-on metal cap with hinge knuckles
#
# Articulations:
#   shaker_rotate: CONTINUOUS, body -> shaker, +Z axis at mouth level
#   lid_rotate:    CONTINUOUS, body -> lid, +Z axis at rim top

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
NECK_R = 0.024                # outer radius of threaded neck (wide mouth ~48mm)
NECK_H = 0.010                # neck height above shoulder
RIM_TOP_Z = JAR_BODY_H + NECK_H  # z of the neck rim (0.080)

SHAKER_R = NECK_R - WALL            # shaker disc rim contacts inner neck wall (friction fit)
SHAKER_Z = JAR_BODY_H + 0.002      # shaker sits just above the body interior

LID_OUTER_R = 0.030           # lid outer radius (slightly wider than neck)
LID_H = 0.016                 # lid total height
LID_SKIRT_BOTTOM_Z = -0.008   # lid-local: skirt drops below rim top
LID_TOP_Z = LID_SKIRT_BOTTOM_Z + LID_H  # lid-local top


def _jar_glass_solid() -> cq.Workplane:
    """Hollow thick-walled glass spice jar with wide mouth opening.
    Revolve of half-profile in XZ about Z axis. The inner cavity is open at top."""
    pts = [
        (0.0, 0.0),
        (JAR_OUTER_R, 0.0),
        (JAR_OUTER_R, JAR_BODY_H - 0.008),
        (JAR_OUTER_R - 0.003, JAR_BODY_H - 0.002),
        (NECK_R + 0.002, JAR_BODY_H),
        (NECK_R, JAR_BODY_H + 0.002),
        (NECK_R, RIM_TOP_Z),
        (NECK_R - WALL, RIM_TOP_Z),
        (NECK_R - WALL, JAR_BODY_H + 0.002),
        (NECK_R - WALL - 0.002, JAR_BODY_H),
        (JAR_OUTER_R - WALL, JAR_BODY_H - 0.006),
        (JAR_OUTER_R - WALL, WALL),
        (0.0, WALL),
        (0.0, 0.0),
    ]
    profile = cq.Workplane("XZ").polyline(pts).close()
    return profile.revolve(360.0, (0, 0, 0), (0, 1, 0))


def _neck_threads() -> cq.Workplane:
    """Thread ridges on the neck exterior for screw-on lid."""
    threads = None
    z0 = JAR_BODY_H + 0.003
    for i in range(3):
        z = z0 + i * 0.0025
        ring = (
            cq.Workplane("XY")
            .workplane(offset=z)
            .circle(NECK_R + 0.0005)
            .circle(NECK_R - 0.0003)
            .extrude(0.0012)
        )
        threads = ring if threads is None else threads.union(ring)
    return threads


def _clamp_hooks() -> cq.Workplane:
    """Four small L-shaped clamp hooks on the neck exterior that engage the lid."""
    hooks = None
    hook_w = 0.004
    hook_h = 0.006
    hook_d = 0.002
    hook_z = JAR_BODY_H + 0.004
    for i in range(4):
        ang = 2.0 * math.pi * i / 4.0 + math.pi / 4.0
        cx = (NECK_R + 0.001) * math.cos(ang)
        cy = (NECK_R + 0.001) * math.sin(ang)
        # Vertical tab
        tab = (
            cq.Workplane("XY")
            .workplane(offset=hook_z)
            .center(cx, cy)
            .rect(hook_w, hook_d)
            .extrude(hook_h)
        )
        # Horizontal lip at top of tab
        lip = (
            cq.Workplane("XY")
            .workplane(offset=hook_z + hook_h - hook_d)
            .center(cx * 1.02, cy * 1.02)
            .rect(hook_w, hook_d + 0.002)
            .extrude(hook_d)
        )
        hook = tab.union(lip)
        hooks = hook if hooks is None else hooks.union(hook)
    return hooks


def _shaker_disc() -> cq.Workplane:
    """Perforated shaker disc: flat disc with holes for spice dispensing."""
    # Solid disc base
    disc = (
        cq.Workplane("XY")
        .circle(SHAKER_R)
        .extrude(0.002)
    )
    # Add a raised rim around the edge
    rim = (
        cq.Workplane("XY")
        .circle(SHAKER_R)
        .circle(SHAKER_R - 0.002)
        .extrude(0.004)
    )
    disc = disc.union(rim)

    # Cut holes in a circular pattern - 3 rings of holes
    hole_r = 0.0015  # hole radius (3mm diameter)
    # Center hole
    center_hole = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .circle(hole_r)
        .extrude(0.006)
    )
    disc = disc.cut(center_hole)

    # Inner ring: 6 holes at r=0.007
    for i in range(6):
        ang = 2.0 * math.pi * i / 6.0
        hx = 0.007 * math.cos(ang)
        hy = 0.007 * math.sin(ang)
        hole = (
            cq.Workplane("XY")
            .workplane(offset=-0.001)
            .center(hx, hy)
            .circle(hole_r)
            .extrude(0.006)
        )
        disc = disc.cut(hole)

    # Outer ring: 10 holes at r=0.014
    for i in range(10):
        ang = 2.0 * math.pi * i / 10.0
        hx = 0.014 * math.cos(ang)
        hy = 0.014 * math.sin(ang)
        hole = (
            cq.Workplane("XY")
            .workplane(offset=-0.001)
            .center(hx, hy)
            .circle(hole_r)
            .extrude(0.006)
        )
        disc = disc.cut(hole)

    return disc


def _lid_solid() -> cq.Workplane:
    """Screw-on metal lid: cylindrical cup that caps over the neck."""
    outer = (
        cq.Workplane("XY")
        .workplane(offset=LID_SKIRT_BOTTOM_Z)
        .circle(LID_OUTER_R)
        .extrude(LID_H)
    )
    outer = outer.edges(">Z").fillet(0.002)
    # Hollow cavity to seat over the neck
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=LID_SKIRT_BOTTOM_Z - 0.001)
        .circle(NECK_R + 0.0005)
        .extrude(LID_H - 0.004)
    )
    return outer.cut(cavity)


def _lid_knurl() -> cq.Workplane:
    """Knurled grip ridges around the lid skirt."""
    ribs = None
    n = 36
    band_z = LID_SKIRT_BOTTOM_Z + 0.002
    band_h = LID_H - 0.006
    for i in range(n):
        ang = 2.0 * math.pi * i / n
        rib = (
            cq.Workplane("XY")
            .workplane(offset=band_z)
            .center(
                (LID_OUTER_R - 0.0003) * math.cos(ang),
                (LID_OUTER_R - 0.0003) * math.sin(ang),
            )
            .rect(0.0012, 0.0012)
            .extrude(band_h)
        )
        ribs = rib if ribs is None else ribs.union(rib)
    return ribs


def _hinge_knuckles() -> cq.Workplane:
    """Two small hinge knuckles on the lid edge for visual detail."""
    knuckles = None
    knuckle_r = 0.0025
    knuckle_h = 0.006
    for i in range(2):
        ang = math.pi * 0.4 + i * math.pi * 0.2
        kx = (LID_OUTER_R + knuckle_r * 0.3) * math.cos(ang)
        ky = (LID_OUTER_R + knuckle_r * 0.3) * math.sin(ang)
        kz = LID_SKIRT_BOTTOM_Z + LID_H * 0.4
        knuckle = (
            cq.Workplane("XZ")
            .workplane(offset=ky)
            .center(kx, kz)
            .circle(knuckle_r)
            .extrude(knuckle_h)
        )
        # Rotate to align properly - use a cylinder approach instead
        knuckle = (
            cq.Workplane("XY")
            .workplane(offset=kz - knuckle_r)
            .center(kx, ky)
            .circle(knuckle_r)
            .extrude(knuckle_r * 2)
        )
        knuckles = knuckle if knuckles is None else knuckles.union(knuckle)
    return knuckles


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spice_jar")

    glass_amber = model.material("glass_amber", rgba=(0.72, 0.55, 0.30, 0.50))
    metal_silver = model.material("metal_silver", rgba=(0.75, 0.76, 0.78, 1.0))
    shaker_white = model.material("shaker_white", rgba=(0.92, 0.91, 0.88, 1.0))
    hook_metal = model.material("hook_metal", rgba=(0.60, 0.62, 0.64, 1.0))
    spice_brown = model.material("spice_brown", rgba=(0.55, 0.35, 0.15, 1.0))

    # ---- jar body (root): glass shell + neck threads + clamp hooks ----
    body = model.part("body")

    glass = _jar_glass_solid().union(_neck_threads())
    body.visual(mesh_from_cadquery(glass, "jar_glass"), material=glass_amber, name="jar_glass")

    # Clamp hooks on the neck
    hooks = _clamp_hooks()
    body.visual(mesh_from_cadquery(hooks, "clamp_hooks"), material=hook_metal, name="clamp_hooks")

    # Spice content visible inside the jar (brown mass filling lower half)
    # Touches the inner floor and inner walls for connectivity
    spice_fill = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .circle(JAR_OUTER_R - WALL)
        .extrude(JAR_BODY_H * 0.45)
    )
    body.visual(mesh_from_cadquery(spice_fill, "spice_fill"), material=spice_brown, name="spice_fill")

    body.inertial = Inertial.from_geometry(
        Cylinder(JAR_OUTER_R, JAR_BODY_H),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, JAR_BODY_H * 0.5)),
    )

    # ---- shaker insert: perforated disc that rotates in the mouth ----
    shaker = model.part("shaker")
    shaker_disc = _shaker_disc()
    shaker.visual(
        mesh_from_cadquery(shaker_disc, "shaker_disc"),
        material=shaker_white,
        name="shaker_disc",
    )
    # Small orientation marker on the shaker
    shaker.visual(
        Box((0.003, 0.002, 0.003)),
        origin=Origin(xyz=(SHAKER_R - 0.004, 0.0, 0.003)),
        material=hook_metal,
        name="shaker_marker",
    )
    shaker.inertial = Inertial.from_geometry(
        Cylinder(SHAKER_R, 0.004),
        mass=0.01,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
    )

    # Shaker rotate: continuous rotation about +Z at the mouth
    model.articulation(
        "shaker_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=shaker,
        origin=Origin(xyz=(0.0, 0.0, SHAKER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=2.0),
    )

    # ---- lid: screw-on metal cap with hinge knuckles ----
    lid = model.part("lid")
    lid_shell = _lid_solid()
    lid.visual(mesh_from_cadquery(lid_shell, "lid_shell"), material=metal_silver, name="lid_shell")

    knurl = _lid_knurl()
    lid.visual(mesh_from_cadquery(knurl, "lid_knurl"), material=metal_silver, name="lid_knurl")

    knuckles = _hinge_knuckles()
    lid.visual(mesh_from_cadquery(knuckles, "hinge_knuckles"), material=hook_metal, name="hinge_knuckles")

    lid.inertial = Inertial.from_geometry(
        Cylinder(LID_OUTER_R, LID_H),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.0, LID_SKIRT_BOTTOM_Z + LID_H * 0.5)),
    )

    # Lid rotate: continuous screw joint about +Z at rim top
    model.articulation(
        "lid_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.5),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    shaker = object_model.get_part("shaker")
    lid = object_model.get_part("lid")
    shaker_rotate = object_model.get_articulation("shaker_rotate")
    lid_rotate = object_model.get_articulation("lid_rotate")

    # ---- Allow intentional overlaps ----
    # Lid skirt seats over the neck rim
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_shell",
        elem_b="jar_glass",
        reason="The lid skirt is intentionally slipped down over the threaded neck rim for screw-on engagement.",
    )

    # Shaker disc sits inside the mouth opening
    ctx.allow_overlap(
        shaker,
        body,
        elem_a="shaker_disc",
        elem_b="jar_glass",
        reason="The shaker insert is intentionally seated inside the jar mouth opening.",
    )

    # ---- Jar is taller than wide (spice jar proportions) ----
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jar is taller than wide (spice jar proportions)",
        bext[2] > bext[0] and bext[2] > bext[1],
        details=f"body extents={bext}",
    )

    # ---- Wide mouth: neck opening is substantial ----
    # The neck inner radius should be at least 60% of the body outer radius
    neck_inner_r = NECK_R - WALL
    ctx.check(
        "wide mouth opening (neck inner radius >= 60% of body radius)",
        neck_inner_r >= JAR_OUTER_R * 0.60,
        details=f"neck_inner_r={neck_inner_r}, jar_outer_r={JAR_OUTER_R}",
    )

    # ---- Lid sits on top of the jar ----
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid is on top of the jar",
        lid_pos is not None and lid_pos[2] > RIM_TOP_Z - 0.001,
        details=f"lid_pos={lid_pos}, rim_top={RIM_TOP_Z}",
    )
    ctx.expect_overlap(
        lid, body, axes="xy", min_overlap=0.015, name="lid caps the neck"
    )

    # ---- Shaker sits in the mouth ----
    shaker_pos = ctx.part_world_position(shaker)
    ctx.check(
        "shaker is in the jar mouth",
        shaker_pos is not None and shaker_pos[2] > JAR_BODY_H - 0.005,
        details=f"shaker_pos={shaker_pos}",
    )
    ctx.expect_within(
        shaker, body, axes="xy",
        inner_elem="shaker_disc", outer_elem="jar_glass",
        margin=0.005,
        name="shaker fits within jar mouth",
    )

    # ---- Shaker rotate: disc marker moves when rotated ----
    marker0 = ctx.part_element_world_aabb(shaker, elem="shaker_marker")
    m0 = ((marker0[0][0] + marker0[1][0]) * 0.5, (marker0[0][1] + marker0[1][1]) * 0.5)
    with ctx.pose({shaker_rotate: math.pi / 2.0}):
        marker1 = ctx.part_element_world_aabb(shaker, elem="shaker_marker")
        m1 = ((marker1[0][0] + marker1[1][0]) * 0.5, (marker1[0][1] + marker1[1][1]) * 0.5)
    moved = math.hypot(m1[0] - m0[0], m1[1] - m0[1])
    ctx.check(
        "shaker_rotate spins the shaker (marker moves)",
        moved > 0.005,
        details=f"marker rest={m0}, quarter-turn={m1}, moved={moved}",
    )

    # ---- Lid rotate: lid rotates on continuous screw joint ----
    lid_knurl0 = ctx.part_element_world_aabb(lid, elem="lid_knurl")
    lk0_center = ((lid_knurl0[0][0] + lid_knurl0[1][0]) * 0.5,
                  (lid_knurl0[0][1] + lid_knurl0[1][1]) * 0.5)
    with ctx.pose({lid_rotate: math.pi}):
        lid_knurl1 = ctx.part_element_world_aabb(lid, elem="lid_knurl")
        # The knurling is symmetric, so check hinge knuckles instead
        hk0 = ctx.part_element_world_aabb(lid, elem="hinge_knuckles")
        hk0_cx = (hk0[0][0] + hk0[1][0]) * 0.5
    # Check that lid_rotate articulation exists and is CONTINUOUS
    ctx.check(
        "lid_rotate is a continuous joint",
        lid_rotate.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={lid_rotate.articulation_type}",
    )

    # ---- Clamp hooks exist on the body ----
    hooks_vis = body.get_visual("clamp_hooks")
    ctx.check(
        "clamp hooks are present on the jar body",
        hooks_vis is not None,
        details="clamp_hooks visual not found",
    )

    # ---- Hinge knuckles exist on the lid ----
    knuckles_vis = lid.get_visual("hinge_knuckles")
    ctx.check(
        "hinge knuckles are present on the lid",
        knuckles_vis is not None,
        details="hinge_knuckles visual not found",
    )

    # ---- Both joints are non-fixed ----
    ctx.check(
        "shaker_rotate is non-fixed",
        shaker_rotate.articulation_type != ArticulationType.FIXED,
        details=f"type={shaker_rotate.articulation_type}",
    )
    ctx.check(
        "lid_rotate is non-fixed",
        lid_rotate.articulation_type != ArticulationType.FIXED,
        details=f"type={lid_rotate.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
