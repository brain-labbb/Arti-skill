from __future__ import annotations

# Stainless steel spring snap-hook carabiner with WIRE GATE (firefighter / snap-hook style).
#
# Frame: the pear/teardrop loop lies in the X-Z plane, +Z up.
#   - Wide rounded TOP of the loop is at high +Z; the small EYE is at the bottom (low +Z).
#   - The GATE is the straight LEFT side (-X). It is the ONLY bar on that span: the body
#     is an OPEN hook there. The gate bridges from the bottom hinge rivet up to the nose.
#   - The opposite long side (the curved "spine") is the +X side.
#   - The NOSE is the body's hook tip at the top of the gate-side, just below the wide top
#     bend; the closed gate's latch notch seats against it.
#
# The body is one thick round bar bent into an OPEN hook (NOT a closed loop): nose tip ->
# over the top -> down the +X spine -> around the eye -> up the -X side a short way to the
# hinge boss. The straight -X span between the hinge and the nose is EMPTY metal: that gap
# is what the gate fills, and what opens when the gate swings in.
#
# WIRE GATE: a single thin spring-steel wire (~2.6 mm dia) bent into a narrow elongated
# hairpin U-loop. Two close-spaced parallel wire legs joined by a rounded bend at the nose
# end. Much lighter and thinner than a solid bar gate, with a visible open wire loop rather
# than a fat capsule. A small flat hinge plate at the bottom captures the pivot pin.
#
# Articulation:
#   - gate: REVOLUTE about the hinge rivet at the BOTTOM of the gate-side. The wire gate
#     sits flush on the loop's -X line. The pin axis is the loop normal (Y), so the gate
#     swings in the X-Z plane. Opening swings the gate top INWARD (toward +X, into the
#     loop) ~0..28 deg, opening a gap between the gate top and the nose; spring-return
#     closed at 0.

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
GATE_R = 0.0038  # gate bar radius (a touch slimmer than the body bar) -- kept for hinge boss sizing

# Wire gate dimensions: a thin spring-steel wire bent into a narrow hairpin U-loop
WIRE_R = 0.0013  # wire radius (~2.6 mm dia spring steel wire)
WIRE_LEG_SPACING = 0.0052  # center-to-center Y spacing between the two parallel wire legs
WIRE_BEND_R = WIRE_LEG_SPACING / 2  # U-bend radius at the nose end


def _body_hook_mesh():
    # OPEN hook centerline in the X-Z plane (y=0). One bar, two free ends:
    #   free end #1 = the NOSE tip (top of the gate-side), free end #2 = the HINGE boss
    #   (bottom of the gate-side). The straight -X span between them is intentionally EMPTY
    #   (that gap is the gate). Walk: nose tip -> up over the wide top -> down the +X spine
    #   -> around the eye -> up the -X side to the hinge boss.
    pts = [
        (GATE_X * 0.92, 0.0, GATE_NOSE_Z + 0.002),  # nose tip (free end), slightly inboard
        (GATE_X * 0.80, 0.0, 0.092),  # nose curving up toward the apex
        (0.0, 0.0, TOP_Z),  # top apex (wide bend)
        (HALF_W * 0.80, 0.0, TOP_Z - 0.004),  # spine shoulder
        (HALF_W, 0.0, 0.070),  # spine, widest
        (HALF_W * 0.92, 0.0, 0.044),  # spine mid
        (HALF_W * 0.55, 0.0, 0.023),  # spine sweeping into the eye
        (HALF_W * 0.18, 0.0, EYE_Z),  # eye bottom (spine side)
        (-HALF_W * 0.18, 0.0, EYE_Z),  # eye bottom (gate side)
        (GATE_X * 0.62, 0.0, 0.024),  # gate side rising
        (GATE_X, 0.0, GATE_HINGE_Z),  # hinge boss (free end #2)
    ]
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


def _wire_gate_mesh():
    # Wire gate: a single thin spring-steel wire bent into a narrow elongated hairpin U-loop.
    # Authored in the GATE LOCAL frame: hinge pin at the local origin, wire legs running up
    # +Z to the nose, joined by a rounded U-bend at the top. Two close-spaced parallel legs
    # in the Y direction, connected at the top by the bend. The whole gate reads as one
    # continuous piece of bent wire.
    half_s = WIRE_LEG_SPACING / 2
    bend_top = GATE_LEN - WIRE_BEND_R * 0.15  # legs reach nearly to the full gate length

    # Path: left leg bottom -> left leg top -> U-bend apex -> right leg top -> right leg bottom
    # Add intermediate points along the U-bend for a smooth curve
    n_bend = 8
    bend_pts = []
    for i in range(n_bend + 1):
        t = i / n_bend  # 0..1 across the bend
        angle = math.pi * t  # 0 to pi
        y = -half_s * math.cos(angle)  # -half_s to +half_s
        z = bend_top + WIRE_BEND_R * math.sin(angle) * 0.85
        bend_pts.append((0.0, y, z))

    pts = [
        (0.0, -half_s, 0.002),  # left leg bottom (just above hinge pin center)
        (0.0, -half_s, GATE_LEN * 0.35),  # left leg lower-mid
        (0.0, -half_s, GATE_LEN * 0.70),  # left leg upper-mid
    ] + bend_pts + [
        (0.0, half_s, GATE_LEN * 0.70),  # right leg upper-mid
        (0.0, half_s, GATE_LEN * 0.35),  # right leg lower-mid
        (0.0, half_s, 0.002),  # right leg bottom
    ]

    wire_mesh = tube_from_spline_points(
        pts,
        radius=WIRE_R,
        closed_spline=False,
        samples_per_segment=16,
        radial_segments=14,
        cap_ends=True,
    )

    # Small hinge capture plate at the bottom: a thin plate that bridges the two wire legs
    # at the hinge pin, giving the pin something to pass through. This is the flat tab that
    # real wire gate carabiners have at the pivot end.
    plate = (
        cq.Workplane("XZ")
        .center(0.0, 0.001)
        .rect(WIRE_R * 3.0, WIRE_R * 4.0)
        .extrude(half_s + WIRE_R, both=True)
    )
    plate_mesh = mesh_from_cadquery(plate, "wire_hinge_plate")

    return wire_mesh, plate_mesh


def _wire_gate_latch_mesh():
    # Wire gate latch: the rounded U-bend tip at the top of the wire loop that seats against
    # the nose lug when closed. A small spherical cap at the apex of the bend represents the
    # wire contact point. Authored in the GATE LOCAL frame (top at z≈GATE_LEN).
    cap = (
        cq.Workplane("XY")
        .workplane(offset=GATE_LEN + WIRE_BEND_R * 0.6)
        .sphere(WIRE_R * 1.6)
    )
    return mesh_from_cadquery(cap, "wire_latch_tip")


def _hinge_pin_mesh():
    # Rivet/pin through the wire gate hinge plate and body boss; pin axis along Y (the loop
    # normal). Slightly smaller rivet than the solid gate version, sized for the wire gate.
    pin = (
        cq.Workplane("XZ")
        .circle(WIRE_R * 1.1)
        .extrude(WIRE_LEG_SPACING / 2 + WIRE_R + 0.001, both=True)
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

    # ---- gate: wire gate, revolute about the bottom hinge rivet, swings inward (+X) to open ----
    gate = model.part("gate")
    wire_mesh, hinge_plate_mesh = _wire_gate_mesh()
    gate.visual(mesh_from_geometry(wire_mesh, "wire_loop"), material=steel, name="wire_loop")
    gate.visual(hinge_plate_mesh, material=steel, name="wire_hinge_plate")
    gate.visual(_wire_gate_latch_mesh(), material=steel, name="wire_latch_tip")
    gate.visual(_hinge_pin_mesh(), material=dark_steel, name="hinge_pin")
    # Wire gate is very light — thin steel wire hairpin, much lighter than a solid bar.
    gate.inertial = Inertial.from_geometry(
        Box((0.004, 0.006, 0.054)), mass=0.004, origin=Origin(xyz=(0.0, 0.0, 0.027))
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

    # The closed wire gate latch tip seats into the nose lug: small intentional capture overlap.
    ctx.allow_overlap(
        gate,
        body,
        elem_a="wire_latch_tip",
        elem_b="nose_lug",
        reason="Closed wire gate latch tip hooks into the nose lug (spring-seated capture).",
    )
    # The wire hinge plate at the bottom wraps the loop's bar where the pin passes through.
    ctx.allow_overlap(
        gate,
        body,
        elem_a="wire_hinge_plate",
        elem_b="body_loop",
        reason="Wire gate hinge plate wraps the loop bar at the pin (mounted hinge capture).",
    )
    ctx.allow_overlap(
        gate,
        body,
        elem_a="hinge_pin",
        elem_b="body_loop",
        reason="Hinge pin passes through the loop bar at the bottom of the gate-side.",
    )

    # ---- body is an open hook, taller than wide, with the eye at the bottom ----
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "body hook is taller than wide",
        bext[2] > bext[0] + 0.02,
        details=f"body extents (x,y,z)={bext}",
    )
    ctx.check(
        "body spans roughly the full carabiner height",
        bext[2] > 0.085,
        details=f"body z-extent={bext[2]}",
    )

    # ---- the wire gate is visibly thinner than the body bar ----
    wire_loop_aabb = ctx.part_element_world_aabb(gate, elem="wire_loop")
    wire_ext = _ext(wire_loop_aabb)
    body_ext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "wire gate Y-span is wider than a single wire (two legs visible)",
        wire_ext[1] > WIRE_LEG_SPACING * 0.6,
        details=f"wire loop Y-extent={wire_ext[1]}",
    )
    ctx.check(
        "wire gate X-thickness is much thinner than body bar",
        wire_ext[0] < BAR_R * 3.5,
        details=f"wire loop X-extent={wire_ext[0]}, body bar_r={BAR_R}",
    )

    # ---- the gate top (latch tip) meets the nose when closed (contact / capture) ----
    ctx.expect_contact(
        gate,
        body,
        elem_a="wire_latch_tip",
        elem_b="nose_lug",
        contact_tol=0.002,
        name="closed wire gate latch tip meets the nose lug",
    )

    # ---- gate is hinged near the bottom: hinge origin sits low, near the eye end ----
    gate_pos = ctx.part_world_position(gate)
    ctx.check(
        "gate hinge is near the bottom of the loop",
        gate_pos is not None and gate_pos[2] < 0.045,
        details=f"gate hinge world pos={gate_pos}",
    )

    # ---- opening swings the gate TOP inward (toward +X) and AWAY from the nose ----
    closed_latch = ctx.part_element_world_aabb(gate, elem="wire_latch_tip")
    nose_aabb = ctx.part_element_world_aabb(body, elem="nose_lug")
    closed_cx = 0.5 * (closed_latch[0][0] + closed_latch[1][0])
    nose_cx = 0.5 * (nose_aabb[0][0] + nose_aabb[1][0])
    closed_gap = abs(closed_cx - nose_cx)

    with ctx.pose({hinge: math.radians(28.0)}):
        open_latch = ctx.part_element_world_aabb(gate, elem="wire_latch_tip")
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
