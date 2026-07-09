from __future__ import annotations

# Stainless steel spring snap-hook carabiner — OVAL (egg / round-ended) variant.
#
# Frame: a symmetric OVAL loop in the X-Z plane, +Z up. The body is one thick
# round bar bent into an OPEN hook whose outline (if closed) would form a
# left-right symmetric oval taller than wide, with the SAME rounded radius at
# the top bend and the bottom bend.
#   - The GATE is the straight LEFT side (-X). It is the ONLY bar on that span:
#     the body is an OPEN hook there. The gate bridges from the bottom hinge
#     rivet up to the nose.
#   - The opposite long side (the curved "spine") is the +X side.
#   - The NOSE is the body's hook tip at the top of the gate-side; the closed
#     gate's latch notch seats against it.
#
# The body centerline is generated as two equal-radius semicircular arcs (top
# and bottom) joined by straight spine and gate-side runs, so both bends are
# equally rounded.
#
# Articulation:
#   - gate: REVOLUTE about the hinge rivet at the BOTTOM of the gate-side. The
#     gate is a straight rounded bar flush with the loop's -X line, latch notch
#     at the TOP. Pin axis = loop normal (Y), gate swings in the X-Z plane.
#     Opening swings the gate top INWARD (toward +X, into the loop) ~0..28 deg;
#     spring-return closed at 0.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---- key dimensions (meters) ----
BAR_R = 0.0042  # round bar radius (~0.008 m dia)
HALF_W = 0.022  # half width of the loop (bar-center to bar-center span ~0.044)
TOP_Z = 0.100  # top of bar centerline at the wide bend
EYE_Z = 0.012  # eye bottom (bar centerline)
GATE_X = -HALF_W  # the gate / left straight side sits on the -X line

# Gate-side straight run: the gate spans between the hinge rivet (bottom) and the nose (top).
GATE_HINGE_Z = 0.028  # hinge pin height on the gate-side (bottom rivet)
GATE_NOSE_Z = 0.082  # nose / latch height (just below the top bend)
GATE_LEN = GATE_NOSE_Z - GATE_HINGE_Z  # straight gate length ~0.054
GATE_R = 0.0038  # gate bar radius (a touch slimmer than the body bar)


def _oval_centerline_pts(n_arc: int = 14):
    """Generate the symmetric-oval centerline as a list of (x, y, z) tuples.

    The outline is two semicircular arcs of equal radius R = HALF_W joined by
    straight runs on the +X spine and -X gate sides.  Both the top bend and
    the bottom bend have the same radius, making the oval left-right symmetric
    and equally rounded at both ends.

    Walk order (open hook, NOT a closed loop):
        nose tip → top bend arc → spine straight → bottom bend arc → hinge boss
    The straight -X span between nose and hinge is intentionally EMPTY — that
    gap is filled by the gate.
    """
    R = HALF_W                       # bend radius (same for top & bottom)
    z_top_c = TOP_Z - R              # top-arc centre height
    z_bot_c = EYE_Z + R              # bottom-arc centre height

    pts: list[tuple[float, float, float]] = []

    # -- free end #1: nose tip (top of gate side, slightly inboard & above arc) --
    pts.append((GATE_X * 0.95, 0.0, GATE_NOSE_Z + 0.002))
    pts.append((GATE_X * 0.88, 0.0, GATE_NOSE_Z - 0.002))

    # -- top bend arc: 170° → 10° (gate side over the top to spine side) --
    for i in range(n_arc + 1):
        t = i / n_arc
        angle = math.radians(170.0 - t * 160.0)   # 170° → 10°
        x = R * math.cos(angle)
        z = z_top_c + R * math.sin(angle)
        pts.append((x, 0.0, z))

    # -- spine straight mid-point (+X side) --
    pts.append((HALF_W, 0.0, 0.5 * (z_top_c + z_bot_c)))

    # -- bottom bend arc: -10° → -170° (spine side under the bottom to gate side) --
    for i in range(n_arc + 1):
        t = i / n_arc
        angle = math.radians(-10.0 - t * 160.0)   # -10° → -170°
        x = R * math.cos(angle)
        z = z_bot_c + R * math.sin(angle)
        pts.append((x, 0.0, z))

    # -- free end #2: hinge boss (bottom of gate side) --
    pts.append((GATE_X * 0.88, 0.0, GATE_HINGE_Z + 0.003))
    pts.append((GATE_X, 0.0, GATE_HINGE_Z))

    return pts


def _body_hook_mesh():
    # One continuous round bar bent into the symmetric oval open-hook outline.
    pts = _oval_centerline_pts(n_arc=14)
    return tube_from_spline_points(
        pts,
        radius=BAR_R,
        closed_spline=False,
        samples_per_segment=22,
        radial_segments=22,
    )


def _nose_lug_mesh():
    # The nose: a short steel hook lip at the top of the gate-side, reaching slightly DOWN
    # and INWARD from the body's nose tip so the closed gate's latch notch seats against it.
    # Sits just below the wide top bend; this is the catch the gate latches into.
    lug = (
        cq.Workplane("XZ")
        .center(GATE_X + 0.0022, GATE_NOSE_Z - 0.001)
        .rect(0.0075, 0.006)
        .extrude(BAR_R * 0.95, both=True)
    )
    return mesh_from_cadquery(lug, "nose_lug")


def _gate_body_mesh():
    # Straight rounded gate bar + bottom hinge boss, authored in the GATE LOCAL frame:
    # hinge pin at the local origin, bar running up +Z to the nose. The bar sits at local
    # x=0, i.e. flush on the loop's -X line, so closed it completes the pear outline. Built
    # entirely in CadQuery so the bar + boss fuse into one solid.
    # Main gate bar: a vertical capsule cylinder along +Z, flush on the gate line (x=0).
    bar = (
        cq.Workplane("XY")
        .circle(GATE_R)
        .extrude(GATE_LEN)
    )
    bar = bar.union(
        cq.Workplane("XY").sphere(GATE_R)
    ).union(
        cq.Workplane("XY").workplane(offset=GATE_LEN).sphere(GATE_R)
    )
    # Hinge boss (knuckle) at the bottom, wrapping the loop's bar where the pin passes
    # through. Pin axis = local Y (the loop normal).
    boss = (
        cq.Workplane("XZ")
        .circle(GATE_R * 1.55)
        .extrude(0.0048, both=True)
    )
    gate = bar.union(boss)
    return mesh_from_cadquery(gate, "gate_bar")


def _gate_latch_mesh():
    # Latch notch at the top of the gate: a small lip that hooks INWARD (+x) and slightly up
    # so it catches the nose lug when closed. Authored in the GATE LOCAL frame (top at
    # z=GATE_LEN). Separate named visual so tests can target the latch/nose seating.
    hook = (
        cq.Workplane("XZ")
        .center(GATE_R * 0.7, GATE_LEN + GATE_R * 0.3)
        .rect(GATE_R * 2.2, GATE_R * 1.9)
        .extrude(GATE_R * 1.2, both=True)
    )
    return mesh_from_cadquery(hook, "gate_latch")


def _hinge_pin_mesh():
    # Rivet/pin through the gate boss; pin axis along Y (the loop normal). Flush rivet.
    pin = (
        cq.Workplane("XZ")
        .circle(GATE_R * 0.55)
        .extrude(0.0062, both=True)
    )
    return mesh_from_cadquery(pin, "hinge_pin")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="snap_hook_carabiner")

    steel = model.material("satin_steel", rgba=(0.74, 0.76, 0.78, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.55, 0.57, 0.60, 1.0))

    # ---- body (root): the bent-bar OPEN hook + nose lug ----
    body = model.part("body")
    body.visual(mesh_from_geometry(_body_hook_mesh(), "body_loop"), material=steel, name="body_loop")
    body.visual(_nose_lug_mesh(), material=steel, name="nose_lug")
    body.inertial = Inertial.from_geometry(
        Box((0.052, 0.012, 0.100)), mass=0.085, origin=Origin(xyz=(0.0, 0.0, 0.052))
    )

    # ---- gate: revolute about the bottom hinge rivet, swings inward (+X) to open ----
    gate = model.part("gate")
    gate.visual(_gate_body_mesh(), material=steel, name="gate_bar")
    gate.visual(_gate_latch_mesh(), material=steel, name="gate_latch")
    gate.visual(_hinge_pin_mesh(), material=dark_steel, name="hinge_pin")
    # Gate is a slim bar ~0.054 tall; modest mass. Authored about the hinge at local origin.
    gate.inertial = Inertial.from_geometry(
        Box((0.008, 0.008, 0.058)), mass=0.012, origin=Origin(xyz=(0.0, 0.0, 0.027))
    )
    # Hinge at the bottom rivet of the gate-side. Axis = local Y (the loop normal) so the
    # gate top swings in the X-Z plane. Positive angle pushes the top toward +X (into the
    # loop), opening the gap between the gate top and the nose. Closed at 0 (spring-return).
    model.articulation(
        "gate_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=gate,
        origin=Origin(xyz=(GATE_X, 0.0, GATE_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=math.radians(28.0),
            effort=1.0,
            velocity=3.0,
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    gate = object_model.get_part("gate")
    hinge = object_model.get_articulation("gate_hinge")

    # The closed gate latch seats into the nose lug: small intentional capture overlap.
    ctx.allow_overlap(
        gate,
        body,
        elem_a="gate_latch",
        elem_b="nose_lug",
        reason="Closed snap-hook gate latch hooks into the nose lug (spring-seated capture).",
    )
    ctx.allow_overlap(
        gate,
        body,
        elem_a="gate_latch",
        elem_b="body_loop",
        reason="Gate latch wraps around the body bar at the nose where it seats closed (seated capture).",
    )
    # The hinge boss at the bottom of the gate wraps the loop's bar where the pin passes
    # through, so the gate reads as pinned to the body.
    ctx.allow_overlap(
        gate,
        body,
        elem_a="gate_bar",
        elem_b="body_loop",
        reason="Gate hinge boss wraps the loop bar at the pin (mounted hinge capture).",
    )
    ctx.allow_overlap(
        gate,
        body,
        elem_a="hinge_pin",
        elem_b="body_loop",
        reason="Hinge pin passes through the loop bar at the bottom of the gate-side.",
    )

    # ---- body is an oval loop, taller than wide, symmetric about Z-mid ----
    body_aabb = ctx.part_world_aabb(body)
    bext = _ext(body_aabb)
    ctx.check(
        "oval body is taller than wide",
        bext[2] > bext[0] + 0.02,
        details=f"body extents (x,y,z)={bext}",
    )
    ctx.check(
        "body spans roughly the full carabiner height",
        bext[2] > 0.085,
        details=f"body z-extent={bext[2]}",
    )
    # Oval symmetry: the bounding-box Z centre should sit near the midpoint of
    # the nominal top/bottom heights.  A pear shape shifts the centre upward
    # because the wide top dominates; a symmetric oval keeps it centred.
    z_mid_box = 0.5 * (body_aabb[0][2] + body_aabb[1][2])
    z_mid_nominal = 0.5 * (TOP_Z + EYE_Z)
    ctx.check(
        "oval body is vertically symmetric (top and bottom bends equal)",
        abs(z_mid_box - z_mid_nominal) < 0.008,
        details=f"box z-centre={z_mid_box:.4f}, nominal mid={z_mid_nominal:.4f}",
    )
    # The body X-extent should be centred near x=0 (left-right symmetric oval).
    x_mid_box = 0.5 * (body_aabb[0][0] + body_aabb[1][0])
    ctx.check(
        "oval body is left-right symmetric about x=0",
        abs(x_mid_box) < 0.004,
        details=f"box x-centre={x_mid_box:.4f}",
    )

    # ---- the gate top (latch) meets the nose when closed (contact / capture) ----
    ctx.expect_contact(
        gate,
        body,
        elem_a="gate_latch",
        elem_b="nose_lug",
        contact_tol=0.0015,
        name="closed gate latch meets the nose lug",
    )

    # ---- gate is hinged near the bottom: hinge origin sits low, near the eye end ----
    gate_pos = ctx.part_world_position(gate)
    ctx.check(
        "gate hinge is near the bottom of the loop",
        gate_pos is not None and gate_pos[2] < 0.045,
        details=f"gate hinge world pos={gate_pos}",
    )

    # ---- opening swings the gate TOP inward (toward +X) and AWAY from the nose ----
    # Closed: measure the gate latch element AABB center in X and its top position.
    closed_latch = ctx.part_element_world_aabb(gate, elem="gate_latch")
    nose_aabb = ctx.part_element_world_aabb(body, elem="nose_lug")
    closed_cx = 0.5 * (closed_latch[0][0] + closed_latch[1][0])
    nose_cx = 0.5 * (nose_aabb[0][0] + nose_aabb[1][0])
    closed_gap = abs(closed_cx - nose_cx)

    with ctx.pose({hinge: math.radians(28.0)}):
        open_latch = ctx.part_element_world_aabb(gate, elem="gate_latch")
        open_cx = 0.5 * (open_latch[0][0] + open_latch[1][0])
    open_gap = abs(open_cx - nose_cx)

    ctx.check(
        "open gate top swings inward toward +X (into the loop)",
        open_cx > closed_cx + 0.004,
        details=f"closed latch x={closed_cx}, open latch x={open_cx}",
    )
    ctx.check(
        "opening moves the latch away from the nose lug",
        open_gap > closed_gap + 0.003,
        details=f"closed gap={closed_gap}, open gap={open_gap}",
    )

    return ctx.report()


object_model = build_object_model()
