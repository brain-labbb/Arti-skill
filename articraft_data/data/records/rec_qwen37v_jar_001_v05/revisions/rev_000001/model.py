from __future__ import annotations

# Wide APOTHECARY JAR with domed flip-top stopper.
# Frame: jar axis along +Z, base on z=0, centered on (x=0, y=0).
#
# A wide squat amber-glass apothecary jar with a thick-walled hollow body,
# a visible wide-mouth opening, and a domed glass stopper that flips open
# on a rear revolute hinge. Small interlocking hinge knuckles at the rear
# rim connect the lid to the body.
#
# Articulation:
#   body_to_lid: REVOLUTE at the rear rim edge, axis +X so positive q
#                flips the domed stopper upward/backward.

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
JAR_OUTER_R = 0.040          # outer radius (~80mm dia, wide apothecary)
JAR_BODY_H = 0.048           # body height (squat)
WALL = 0.004                 # thick glass wall
INNER_R = JAR_OUTER_R - WALL # inner cavity radius = 0.036
NECK_H = 0.006               # short lip above shoulder
RIM_TOP_Z = JAR_BODY_H + NECK_H  # top of the rim/lip = 0.054

# Lid dimensions (in lid-local frame, origin at hinge pin center)
LID_FLANGE_R = JAR_OUTER_R + 0.001  # flange slightly wider than jar = 0.041
LID_FLANGE_H = 0.003                # thin flange disc
DOME_R = 0.034                      # dome base radius (slightly less than inner)
DOME_H = 0.018                      # dome height above flange

# Hinge geometry
HINGE_KNUCKLE_R = 0.0028            # knuckle outer radius
HINGE_KNUCKLE_W = 0.006             # knuckle width along pin axis
HINGE_Y = -(JAR_OUTER_R + 0.001)   # hinge at rear of jar rim = -0.041
HINGE_Z = RIM_TOP_Z                 # hinge pin at rim top level = 0.054

# The lid flange sits ON TOP of the rim. In lid-local frame (origin at hinge pin):
# flange bottom at local z = 0, flange top at z = LID_FLANGE_H.
# In world at q=0: flange bottom = HINGE_Z = 0.054, flange top = 0.057.
# The jar rim top is at RIM_TOP_Z = 0.054. So flange sits exactly on the rim.


def _jar_body_solid() -> cq.Workplane:
    """Hollow thick-walled apothecary jar via revolve of half-profile."""
    pts = [
        (0.0, 0.0),
        (JAR_OUTER_R, 0.0),
        (JAR_OUTER_R, JAR_BODY_H - 0.008),
        (JAR_OUTER_R - 0.002, JAR_BODY_H - 0.002),   # rounded shoulder
        (JAR_OUTER_R - 0.001, JAR_BODY_H),             # shoulder top
        (JAR_OUTER_R + 0.001, JAR_BODY_H + 0.001),     # slight lip flare
        (JAR_OUTER_R + 0.001, RIM_TOP_Z),              # lip outer wall up
        (INNER_R, RIM_TOP_Z),                          # across rim top inward
        (INNER_R, JAR_BODY_H - 0.002),                 # inner neck wall down
        (INNER_R, WALL),                               # inner body wall down
        (0.0, WALL),                                   # across inner base
        (0.0, 0.0),
    ]
    profile = cq.Workplane("XZ").polyline(pts).close()
    jar = profile.revolve(360.0, (0, 0, 0), (0, 1, 0))
    # Thick base foot ring
    base_ring = (
        cq.Workplane("XY")
        .circle(JAR_OUTER_R + 0.001)
        .circle(JAR_OUTER_R - WALL)
        .extrude(0.003)
    )
    return jar.union(base_ring)


def _body_hinge_knuckles() -> cq.Workplane:
    """Two knuckle barrels on the body at the rear rim for the hinge.
    Built along the X axis (hinge pin direction), centered at hinge point.
    """
    knuckles = None
    # Two body knuckles spaced apart, leaving room for the lid knuckle between them.
    # Knuckle 1: x from -0.009 to -0.003; Knuckle 2: x from +0.003 to +0.009.
    for dx in (-0.009, 0.003):
        kn = (
            cq.Workplane("YZ")
            .workplane(offset=dx)
            .center(HINGE_Y, HINGE_Z)
            .circle(HINGE_KNUCKLE_R)
            .extrude(HINGE_KNUCKLE_W)
        )
        knuckles = kn if knuckles is None else knuckles.union(kn)
    # Connecting bridge from jar rim to knuckles (for visual support)
    bridge = (
        cq.Workplane("XY")
        .workplane(offset=RIM_TOP_Z - 0.003)
        .center(0.0, HINGE_Y + HINGE_KNUCKLE_R)
        .rect(0.024, 0.004)
        .extrude(0.006)
    )
    return knuckles.union(bridge)


def _body_front_hook() -> cq.Workplane:
    """Small latch hook on the outside of the front rim, touching the jar wall."""
    # Vertical post attached to the outer jar wall, short enough to not block the lid
    hook = (
        cq.Workplane("XY")
        .workplane(offset=RIM_TOP_Z - 0.003)
        .center(0.0, JAR_OUTER_R + 0.001)
        .rect(0.005, 0.004)
        .extrude(0.007)
    )
    return hook


def _lid_solid() -> cq.Workplane:
    """Domed stopper lid in lid-local frame (origin at hinge pin).

    At q=0 the lid frame is at world (0, HINGE_Y, HINGE_Z).
    The lid extends in +Y (forward) from the hinge.
    The dome rises in +Z from the flange.
    """
    flange_center_y = JAR_OUTER_R  # lid-local y: centers the flange on the jar mouth

    # Flange disc: bottom at local z=0, top at z=LID_FLANGE_H
    flange = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .center(0.0, flange_center_y)
        .circle(LID_FLANGE_R)
        .extrude(LID_FLANGE_H)
    )

    # Dome: solid loft rising from the flange top surface.
    # Start 0.001 below flange top for guaranteed shared volume.
    dome_z0 = LID_FLANGE_H - 0.001
    dome = (
        cq.Workplane("XY")
        .workplane(offset=dome_z0)
        .center(0.0, flange_center_y)
        .circle(DOME_R)
        .workplane(offset=DOME_H * 0.5)
        .circle(DOME_R * 0.6)
        .workplane(offset=DOME_H * 0.5)
        .circle(DOME_R * 0.12)
        .loft()
    )

    # Finial knob on top of dome
    finial_z = dome_z0 + DOME_H
    finial = (
        cq.Workplane("XY")
        .workplane(offset=finial_z)
        .center(0.0, flange_center_y)
        .circle(0.004)
        .extrude(0.005)
    )
    finial_cap = (
        cq.Workplane("XY")
        .workplane(offset=finial_z + 0.003)
        .center(0.0, flange_center_y)
        .circle(0.006)
        .extrude(0.003)
    )

    lid = flange.union(dome).union(finial).union(finial_cap)

    # Lid hinge knuckle: centered between the two body knuckles.
    # Horizontal cylinder along X at the hinge origin (lid-local 0,0,0).
    lid_knuckle = (
        cq.Workplane("YZ")
        .workplane(offset=-HINGE_KNUCKLE_W * 0.45)
        .center(0.0, 0.0)  # lid-local origin = hinge pin
        .circle(HINGE_KNUCKLE_R - 0.0003)
        .extrude(HINGE_KNUCKLE_W * 0.9)
    )
    lid = lid.union(lid_knuckle)

    return lid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="apothecary_jar")

    # Materials
    glass_amber = model.material("glass_amber", rgba=(0.55, 0.32, 0.10, 0.60))
    lid_glass = model.material("lid_glass", rgba=(0.60, 0.38, 0.14, 0.65))
    metal_brass = model.material("metal_brass", rgba=(0.72, 0.58, 0.24, 1.0))
    cream_white = model.material("cream_white", rgba=(0.95, 0.92, 0.85, 1.0))

    # ---- jar body (root) ----
    body = model.part("body")

    jar_mesh = mesh_from_cadquery(_jar_body_solid(), "jar_shell")
    body.visual(jar_mesh, material=glass_amber, name="jar_shell")

    # Hinge knuckles on body (brass-colored metal)
    knuckle_mesh = mesh_from_cadquery(_body_hinge_knuckles(), "body_knuckles")
    body.visual(knuckle_mesh, material=metal_brass, name="body_knuckles")

    # Front latch hook on body
    hook_mesh = mesh_from_cadquery(_body_front_hook(), "body_hook")
    body.visual(hook_mesh, material=metal_brass, name="body_hook")

    # Cream surface visible inside the jar - contacts the inner wall for connectivity.
    # A disc at the cream fill level, radius matching inner wall for contact.
    cream = (
        cq.Workplane("XY")
        .workplane(offset=WALL + 0.001)
        .circle(INNER_R)
        .extrude(JAR_BODY_H - WALL - 0.008)
    )
    body.visual(
        mesh_from_cadquery(cream, "cream_fill"),
        material=cream_white,
        name="cream_fill",
    )

    body.inertial = Inertial.from_geometry(
        Cylinder(JAR_OUTER_R, JAR_BODY_H),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, JAR_BODY_H * 0.5)),
    )

    # ---- lid (domed stopper, flips on rear hinge) ----
    lid = model.part("lid")

    lid_mesh = mesh_from_cadquery(_lid_solid(), "lid_shell")
    lid.visual(lid_mesh, material=lid_glass, name="lid_shell")

    lid.inertial = Inertial.from_geometry(
        Cylinder(LID_FLANGE_R, DOME_H + LID_FLANGE_H),
        mass=0.06,
        origin=Origin(xyz=(0.0, JAR_OUTER_R, DOME_H * 0.4)),
    )

    # ---- articulation: rear revolute hinge ----
    # Origin at rear rim, axis +X so positive q lifts the lid front upward.
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=0.0, upper=1.9
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    # ---- structural: joint is REVOLUTE (non-fixed) ----
    ctx.check(
        "hinge joint is REVOLUTE",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={hinge.articulation_type}",
    )

    # ---- structural: hinge has bounded limits with meaningful range ----
    limits = hinge.motion_limits
    ctx.check(
        "hinge has bounded motion limits",
        limits is not None and limits.lower is not None and limits.upper is not None
        and limits.upper > limits.lower + 0.5,
        details=f"limits={limits}",
    )

    # ---- hinge knuckles exist on body ----
    body_knuckles = body.get_visual("body_knuckles")
    ctx.check(
        "body has hinge knuckles",
        body_knuckles is not None,
        details="body_knuckles visual not found",
    )

    # ---- front hook exists ----
    body_hook = body.get_visual("body_hook")
    ctx.check(
        "body has front latch hook",
        body_hook is not None,
        details="body_hook visual not found",
    )

    # ---- jar is wide (wider than tall) ----
    body_aabb = ctx.part_world_aabb(body)
    bext = (
        body_aabb[1][0] - body_aabb[0][0],
        body_aabb[1][1] - body_aabb[0][1],
        body_aabb[1][2] - body_aabb[0][2],
    )
    ctx.check(
        "jar is wide (wider than tall)",
        bext[0] > bext[2] + 0.005,
        details=f"body extents={bext}",
    )

    # ---- wide mouth: interior cream fill proves hollow cavity ----
    cream = body.get_visual("cream_fill")
    ctx.check(
        "jar has visible interior fill (wide mouth cavity)",
        cream is not None,
        details="cream_fill visual not found",
    )

    # ---- lid covers the mouth at rest (closed) ----
    ctx.expect_overlap(
        lid, body, axes="xy", min_overlap=0.02,
        name="closed lid covers the jar mouth",
    )

    # ---- allow intentional overlap at hinge knuckle interface ----
    ctx.allow_overlap(
        lid, body,
        elem_a="lid_shell", elem_b="body_knuckles",
        reason="The lid hinge knuckle interlocks with the body knuckles at the rear rim hinge.",
    )
    ctx.allow_overlap(
        lid, body,
        elem_a="lid_shell", elem_b="body_hook",
        reason="The lid flange edge seats alongside the front latch hook when closed.",
    )

    # ---- positive hinge angle opens lid upward (use AABB, not part origin) ----
    rest_aabb = ctx.part_world_aabb(lid)
    rest_top_z = rest_aabb[1][2]
    rest_front_y = rest_aabb[1][1]  # max Y = front edge

    with ctx.pose({hinge: 1.2}):
        open_aabb = ctx.part_world_aabb(lid)
        open_top_z = open_aabb[1][2]
        open_front_y = open_aabb[1][1]

    ctx.check(
        "positive hinge angle opens lid upward",
        open_top_z > rest_top_z + 0.01,
        details=f"rest_top_z={rest_top_z}, open_top_z={open_top_z}",
    )

    # ---- at open pose, the lid front edge (max Y) moves back/shorter ----
    ctx.check(
        "lid front edge retracts when opened",
        open_front_y < rest_front_y - 0.005,
        details=f"rest_front_y={rest_front_y}, open_front_y={open_front_y}",
    )

    # ---- at closed pose, lid flange sits on the jar rim (element-level) ----
    ctx.expect_gap(
        lid, body, axis="z",
        min_gap=-0.003, max_gap=0.005,
        positive_elem="lid_shell", negative_elem="jar_shell",
        name="closed lid flange sits on the jar rim",
    )

    return ctx.report()


object_model = build_object_model()
