from __future__ import annotations

# Stainless steel spring snap-hook carabiner — D-shape (climbing-D) variant.
#
# Frame: the symmetric D-shape loop lies in the X-Z plane, +Z up.
#   - The outline reads as a capital letter D standing up: flat back on the gate side (-X)
#     and a bowed spine on the opposite side (+X), taller than wide.
#   - The SPINE (+X side) is a long nearly-straight bar with a slight outward bow and
#     rounded top and bottom corners.
#   - The GATE SIDE (-X) is the flat back of the D. The straight span between the bottom
#     hinge rivet and the top nose is the GATE GAP (open — the gate fills it).
#   - The TOP and BOTTOM are tight quarter-circle bends connecting the spine to the gate
#     side.
#   - The NOSE is the body's hook tip at the top of the gate-side; the closed gate's latch
#     notch seats against it.
#   - The EYE region is at the bottom of the D (low +Z).
#
# The body is one thick round bar bent into an OPEN hook (NOT a closed loop): nose tip ->
# top quarter-circle bend -> down the +X spine -> bottom quarter-circle bend -> up the -X
# side a short way to the hinge boss. The straight -X span between the hinge and the nose
# is EMPTY metal: that gap is what the gate fills, and what opens when the gate swings in.
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


def _body_hook_mesh():
    # D-shape OPEN hook centerline in the X-Z plane (y=0). One bar, two free ends:
    #   free end #1 = the NOSE tip (top of the gate-side), free end #2 = the HINGE boss
    #   (bottom of the gate-side). The straight -X span between them is intentionally EMPTY
    #   (that gap is the gate). Walk: nose tip -> top quarter-circle bend -> down the +X
    #   spine (nearly straight, slight bow) -> bottom quarter-circle bend -> up the -X side
    #   to the hinge boss.
    #
    # D-shape strategy: closely-spaced control points at the four corners create tight
    # quarter-circle bends; widely-spaced points along the spine keep it nearly straight.
    # The gate side (-X) runs straight at x=GATE_X between the nose and hinge.
    CR = 0.009  # effective corner radius for the tight D bends
    SPINE_BOW = 0.002  # outward bow of the spine at mid-height

    pts = [
        # === NOSE TIP (free end #1) ===
        (GATE_X + 0.002, 0.0, GATE_NOSE_Z + 0.002),   # nose tip, slightly inboard

        # === GATE SIDE UPPER → TOP-LEFT CORNER ===
        (GATE_X, 0.0, 0.090),                           # gate side straight upper
        (GATE_X + CR * 0.3, 0.0, TOP_Z - CR),          # corner entry
        (-HALF_W * 0.25, 0.0, TOP_Z - 0.002),          # corner mid

        # === TOP OF D ===
        (0.0, 0.0, TOP_Z),                              # top apex

        # === TOP-RIGHT CORNER ===
        (HALF_W * 0.25, 0.0, TOP_Z - 0.002),           # corner mid
        (HALF_W - CR * 0.3, 0.0, TOP_Z - CR),          # corner exit
        (HALF_W, 0.0, TOP_Z - CR - 0.004),             # spine entry

        # === SPINE (long nearly-straight run, slight outward bow) ===
        (HALF_W + SPINE_BOW * 0.7, 0.0, 0.074),        # spine upper
        (HALF_W + SPINE_BOW, 0.0, 0.058),               # spine mid (max bow)
        (HALF_W + SPINE_BOW * 0.7, 0.0, 0.042),        # spine lower

        # === BOTTOM-RIGHT CORNER ===
        (HALF_W, 0.0, EYE_Z + CR + 0.006),             # spine exit
        (HALF_W - CR * 0.3, 0.0, EYE_Z + CR),          # corner entry
        (HALF_W * 0.25, 0.0, EYE_Z + 0.003),           # corner mid

        # === BOTTOM OF D ===
        (0.0, 0.0, EYE_Z),                              # bottom apex

        # === BOTTOM-LEFT CORNER ===
        (-HALF_W * 0.25, 0.0, EYE_Z + 0.003),          # corner mid
        (GATE_X + CR * 0.3, 0.0, EYE_Z + CR),          # corner exit
        (GATE_X, 0.0, EYE_Z + CR + 0.006),             # gate side straight lower

        # === GATE SIDE LOWER → HINGE BOSS ===
        (GATE_X, 0.0, 0.024),                           # gate side lower
        (GATE_X, 0.0, GATE_HINGE_Z),                    # hinge boss (free end #2)
    ]
    return tube_from_spline_points(
        pts,
        radius=BAR_R,
        closed_spline=False,
        samples_per_segment=24,
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
    # x=0, i.e. flush on the loop's -X line, so closed it completes the D outline. Built
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
    model = ArticulatedObject(name="d_shape_carabiner")

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
    # The gate latch also slightly overlaps the body loop tube near the nose region,
    # because the latch hooks inward past the body bar as it seats against the frame.
    ctx.allow_overlap(
        gate,
        body,
        elem_a="gate_latch",
        elem_b="body_loop",
        reason="Gate latch hooks inward past the body frame tube at the nose (seated capture).",
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

    # ---- D-shape body: taller than wide, with the eye at the bottom ----
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "D-shape body is taller than wide",
        bext[2] > bext[0] + 0.025,
        details=f"body extents (x,y,z)={bext}",
    )
    ctx.check(
        "body spans roughly the full carabiner height",
        bext[2] > 0.080,
        details=f"body z-extent={bext[2]}",
    )
    # ---- D-shape proportions: width-to-height ratio consistent with climbing D ----
    ratio = bext[0] / bext[2] if bext[2] > 0 else 0.0
    ctx.check(
        "D-shape width/height ratio is in climbing-D range (0.35–0.65)",
        0.35 < ratio < 0.65,
        details=f"width/height ratio={ratio:.3f}",
    )
    # ---- Spine bow: the body X extent slightly exceeds 2*HALF_W (spine bows outward) ----
    ctx.check(
        "spine bows outward beyond the nominal half-width",
        bext[0] > 2 * HALF_W + 0.001,
        details=f"body x-extent={bext[0]:.4f}, 2*HALF_W={2*HALF_W:.4f}",
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
    # Proof check for body_loop latch overlap: latch stays within the nose region in XY
    ctx.expect_overlap(
        gate,
        body,
        axes="xy",
        elem_a="gate_latch",
        elem_b="nose_lug",
        min_overlap=0.001,
        name="gate latch overlaps the nose lug region in XY (seated capture proof)",
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
