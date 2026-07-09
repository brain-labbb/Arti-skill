from __future__ import annotations

# Stainless steel spring snap-hook carabiner (firefighter / snap-hook style).
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
# Articulation:
#   - gate: REVOLUTE about the hinge rivet at the BOTTOM of the gate-side. The gate is a
#     straight rounded bar flush with the loop's -X line, latch notch at the TOP. The pin
#     axis is the loop normal (Y), so the gate swings in the X-Z plane. Opening swings the
#     gate top INWARD (toward +X, into the loop) ~0..28 deg, opening a gap between the gate
#     top and the nose; spring-return closed at 0.

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

# Screw-lock sleeve: hollow knurled cylinder sliding on the gate bar.
SLEEVE_IR = GATE_R + 0.0004  # inner radius (clearance over gate bar)
SLEEVE_OR = GATE_R * 2.4  # outer radius (~0.0091, visible knurled barrel)
SLEEVE_LEN = 0.015  # sleeve length along gate axis
# Travel range on gate local +Z: unlocked low to locked high (bridging nose seam).
SLEEVE_HOME_Z = 0.014  # sleeve center Z on gate when unlocked (low)
SLEEVE_LOCK_Z = GATE_LEN - 0.002  # sleeve center Z on gate when locked (high, past gate top)
SLEEVE_TRAVEL = SLEEVE_LOCK_Z - SLEEVE_HOME_Z  # prismatic travel ~0.038


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


def _lock_sleeve_mesh():
    # Hollow knurled cylinder that slides on the gate bar. Authored in the SLEEVE LOCAL
    # frame: centered at origin, axis along +Z, spanning z = -SLEEVE_LEN/2 .. +SLEEVE_LEN/2.
    # The inner bore clears the gate bar; the outer surface has circumferential knurl rings.
    half = SLEEVE_LEN / 2.0
    # Outer cylinder
    outer = (
        cq.Workplane("XY")
        .circle(SLEEVE_OR)
        .extrude(SLEEVE_LEN)
        .translate((0.0, 0.0, -half))
    )
    # Inner bore (hollow)
    bore = (
        cq.Workplane("XY")
        .circle(SLEEVE_IR)
        .extrude(SLEEVE_LEN + 0.002)
        .translate((0.0, 0.0, -half - 0.001))
    )
    sleeve = outer.cut(bore)
    # Knurl rings: small circumferential ridges on the outer surface for grip.
    n_rings = 6
    ring_h = SLEEVE_LEN / (n_rings * 2.5)
    ring_r = SLEEVE_OR + 0.0004  # slightly proud of the outer surface
    for i in range(n_rings):
        z_center = -half + SLEEVE_LEN * (i + 0.5) / n_rings
        ring = (
            cq.Workplane("XY")
            .workplane(offset=z_center - ring_h / 2.0)
            .circle(ring_r)
            .circle(SLEEVE_OR - 0.0002)
            .extrude(ring_h)
        )
        sleeve = sleeve.union(ring)
    # Chamfer the top and bottom edges slightly for a finished look
    return mesh_from_cadquery(sleeve, "lock_sleeve")


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

    # ---- screw-lock sleeve: prismatic slide on the gate bar ----
    # A hollow knurled barrel that rides coaxially on the gate. Parent is the gate so
    # the sleeve travels with the gate when it swings. Sliding UP (gate-local +Z) to the
    # locked position bridges the gate-nose seam and blocks the gate from opening; sliding
    # DOWN clears the seam and frees the gate.
    lock_sleeve = model.part("lock_sleeve")
    lock_sleeve.visual(_lock_sleeve_mesh(), material=dark_steel, name="lock_sleeve_barrel")
    lock_sleeve.inertial = Inertial.from_geometry(
        Box((SLEEVE_OR * 2, SLEEVE_OR * 2, SLEEVE_LEN)),
        mass=0.006,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    # Prismatic joint: parent=gate, child=lock_sleeve. Axis is gate-local +Z (up the bar).
    # At q=0 the sleeve center is at SLEEVE_HOME_Z on the gate; at q=SLEEVE_TRAVEL it is at
    # SLEEVE_LOCK_Z, extending past the gate top to bridge the nose seam.
    model.articulation(
        "sleeve_slide",
        ArticulationType.PRISMATIC,
        parent=gate,
        child=lock_sleeve,
        origin=Origin(xyz=(0.0, 0.0, SLEEVE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SLEEVE_TRAVEL,
            effort=2.0,
            velocity=0.1,
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
    lock_sleeve = object_model.get_part("lock_sleeve")
    hinge = object_model.get_articulation("gate_hinge")
    sleeve_slide = object_model.get_articulation("sleeve_slide")

    # The closed gate latch seats into the nose lug: small intentional capture overlap.
    ctx.allow_overlap(
        gate,
        body,
        elem_a="gate_latch",
        elem_b="nose_lug",
        reason="Closed snap-hook gate latch hooks into the nose lug (spring-seated capture).",
    )
    # The latch also contacts the body loop bar near the nose where the hook tip curves in.
    ctx.allow_overlap(
        gate,
        body,
        elem_a="gate_latch",
        elem_b="body_loop",
        reason="Gate latch lip seats against the body loop bar at the nose hook (latch capture region).",
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
    # The lock sleeve is a hollow tube wrapped around the gate bar, sliding coaxially.
    ctx.allow_overlap(
        gate,
        lock_sleeve,
        elem_a="gate_bar",
        elem_b="lock_sleeve_barrel",
        reason="Lock sleeve is a hollow barrel riding on the gate bar (prismatic slide capture).",
    )
    # The sleeve has a small sliding-fit clearance over the gate bar; allow it as isolated
    # since it is mechanically retained on the gate by the prismatic joint.
    ctx.allow_isolated_part(
        lock_sleeve,
        reason="Lock sleeve slides on the gate bar with a small clearance gap; retained by prismatic joint.",
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

    # ---- screw-lock sleeve: slides along the gate bar axis ----
    # At rest (q=0, unlocked): sleeve center is low on the gate, below the gate top.
    sleeve_rest_pos = ctx.part_world_position(lock_sleeve)
    gate_top_z = GATE_HINGE_Z + GATE_LEN  # world Z of gate top when gate is closed
    ctx.check(
        "unlocked sleeve sits below the gate top",
        sleeve_rest_pos is not None and sleeve_rest_pos[2] < gate_top_z - 0.004,
        details=f"sleeve world pos={sleeve_rest_pos}, gate_top_z={gate_top_z}",
    )

    # At max travel (locked): sleeve extends past the gate top, bridging the nose seam.
    with ctx.pose({sleeve_slide: SLEEVE_TRAVEL}):
        sleeve_locked_pos = ctx.part_world_position(lock_sleeve)
    ctx.check(
        "locked sleeve extends to or past the gate top (bridges nose seam)",
        sleeve_locked_pos is not None and sleeve_locked_pos[2] >= gate_top_z - 0.003,
        details=f"locked sleeve pos={sleeve_locked_pos}, gate_top_z={gate_top_z}",
    )

    # Sleeve moves upward along Z when driven to locked position.
    ctx.check(
        "sleeve prismatic travel moves upward along Z",
        sleeve_rest_pos is not None
        and sleeve_locked_pos is not None
        and sleeve_locked_pos[2] > sleeve_rest_pos[2] + 0.020,
        details=f"rest={sleeve_rest_pos}, locked={sleeve_locked_pos}",
    )

    # Sleeve remains coaxial with the gate (stays centered on the gate bar in XY).
    ctx.expect_within(
        lock_sleeve,
        gate,
        axes="xy",
        margin=0.006,
        name="sleeve stays coaxial with the gate bar",
    )

    # The sleeve part exists and has a visible barrel.
    sleeve_aabb = ctx.part_world_aabb(lock_sleeve)
    sleeve_ext = _ext(sleeve_aabb) if sleeve_aabb else (0, 0, 0)
    ctx.check(
        "lock sleeve has visible barrel dimensions",
        sleeve_ext[2] > 0.010 and max(sleeve_ext[0], sleeve_ext[1]) > 0.014,
        details=f"sleeve extents (x,y,z)={sleeve_ext}",
    )

    return ctx.report()


object_model = build_object_model()
