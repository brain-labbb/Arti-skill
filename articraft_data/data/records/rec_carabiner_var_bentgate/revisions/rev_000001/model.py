from __future__ import annotations

# Stainless steel spring snap-hook carabiner with BENT KEY-LOCK gate.
#
# Frame: same pear/teardrop loop as the parent, lying in the X-Z plane, +Z up.
#   - Wide rounded TOP at high +Z; small EYE at the bottom.
#   - The GATE is the LEFT side (-X). The body is an OPEN hook there.
#   - The NOSE is the body's hook tip at the top of the gate-side.
#
# Gate: BENT key-lock style (replaces the parent's straight bar).
#   - Straight bar from the bottom hinge rivet up to ~68% of the gate span.
#   - Distinct bend inward (toward +X, into the loop) at ~40 deg from vertical.
#   - Bent section continues to the tip, which carries a KEY tongue extending
#     back toward the nose (-X direction in gate local frame).
#   - The NOSE lug has a matching SLOT groove that receives the key tongue.
#   - Closing interface: positive bent hook-and-slot (key seats into nose slot).
#   - Gate swings its top inward (+X) to open; spring-returns closed at q=0.
#
# Articulation:
#   - gate_hinge: REVOLUTE about the bottom hinge rivet. Axis = Y (loop normal).
#     Positive q swings the gate top inward (+X), opening the gap. 0..28 deg.

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
BAR_R = 0.0042       # body round bar radius
HALF_W = 0.022       # half width of the loop
TOP_Z = 0.100        # top of bar centerline at the wide bend
EYE_Z = 0.012        # eye bottom (bar centerline)
GATE_X = -HALF_W     # gate / left straight side sits on the -X line

# Gate-side geometry: hinge rivet (bottom) to nose (top)
GATE_HINGE_Z = 0.028
GATE_NOSE_Z = 0.082
GATE_LEN = GATE_NOSE_Z - GATE_HINGE_Z   # ~0.054
GATE_R = 0.0038       # gate bar radius

# Bent gate parameters
BEND_ANGLE = math.radians(40)   # bend from vertical, toward +X (inward)
BEND_FRAC = 0.68                # fraction of gate length where bend starts
BEND_Z = GATE_LEN * BEND_FRAC   # height of bend point in gate local (~0.037)
BENT_LEN = GATE_LEN * 0.36      # length of the bent section (~0.019)

# Key tongue parameters (extends from bent tip toward nose, -X in gate local)
KEY_LEN = 0.006                 # tongue extent in X
KEY_W = GATE_R * 1.6            # tongue width in Y (~0.006)
KEY_H = GATE_R * 1.8            # tongue height in Z (~0.007)

# Precomputed tip position in gate local frame
_TIP_X = math.sin(BEND_ANGLE) * BENT_LEN
_TIP_Z = BEND_Z + math.cos(BEND_ANGLE) * BENT_LEN

# Key center in gate local frame
_KEY_LOCAL_X = _TIP_X - KEY_LEN / 2
_KEY_LOCAL_Z = _TIP_Z

# Key world position when gate is closed (q=0)
_KEY_WORLD_X = GATE_X + _KEY_LOCAL_X
_KEY_WORLD_Z = GATE_HINGE_Z + _KEY_LOCAL_Z


def _body_hook_mesh():
    """Open hook centerline in X-Z plane. Identical to the parent baseline."""
    pts = [
        (GATE_X * 0.92, 0.0, GATE_NOSE_Z + 0.002),   # nose tip (free end)
        (GATE_X * 0.80, 0.0, 0.092),                   # nose curving up
        (0.0, 0.0, TOP_Z),                              # top apex
        (HALF_W * 0.80, 0.0, TOP_Z - 0.004),           # spine shoulder
        (HALF_W, 0.0, 0.070),                           # spine widest
        (HALF_W * 0.92, 0.0, 0.044),                   # spine mid
        (HALF_W * 0.55, 0.0, 0.023),                   # spine into eye
        (HALF_W * 0.18, 0.0, EYE_Z),                   # eye bottom (spine)
        (-HALF_W * 0.18, 0.0, EYE_Z),                  # eye bottom (gate)
        (GATE_X * 0.62, 0.0, 0.024),                   # gate side rising
        (GATE_X, 0.0, GATE_HINGE_Z),                   # hinge boss (free end)
    ]
    return tube_from_spline_points(
        pts,
        radius=BAR_R,
        closed_spline=False,
        samples_per_segment=22,
        radial_segments=22,
    )


def _nose_slot_mesh():
    """Nose lug with a slot groove to receive the bent gate's key tongue.

    The lug bridges from the body bar nose region to the key seating position.
    A rectangular channel is cut through the inner portion, creating a groove
    open on the +X face so the key tongue slides in from the gate side.
    """
    # Lug body: centered to bridge body bar and key position
    lug_dx = 0.014
    lug_dy = BAR_R * 2.4
    lug_dz = 0.012
    lug_cx = -0.015
    lug_cz = _KEY_WORLD_Z

    lug = (
        cq.Workplane("XY")
        .box(lug_dx, lug_dy, lug_dz)
        .translate((lug_cx, 0.0, lug_cz))
    )

    # Slot: rectangular channel slightly larger than the key for clearance.
    # Positioned at the key world position so the key seats into the groove.
    slot_dx = KEY_LEN + 0.002
    slot_dy = KEY_W + 0.002
    slot_dz = KEY_H + 0.002
    slot = (
        cq.Workplane("XY")
        .box(slot_dx, slot_dy, slot_dz)
        .translate((_KEY_WORLD_X, 0.0, _KEY_WORLD_Z))
    )

    lug_with_slot = lug.cut(slot)
    return mesh_from_cadquery(lug_with_slot, "nose_slot")


def _gate_body_mesh():
    """Bent gate bar: straight section + bend + angled section + hinge boss.

    Authored in the gate local frame: hinge pin at local origin, bar running
    up +Z. At BEND_Z the bar bends inward (+X) by BEND_ANGLE and continues
    to the tip. The hinge boss wraps the loop bar at the bottom.
    """
    # Straight section: vertical cylinder from z=0 to z=BEND_Z
    straight = (
        cq.Workplane("XY")
        .circle(GATE_R)
        .extrude(BEND_Z)
    )
    # Bottom cap sphere at hinge
    straight = straight.union(cq.Workplane("XY").sphere(GATE_R))

    # Bend transition sphere at (0, 0, BEND_Z)
    bend_sphere = (
        cq.Workplane("XY")
        .sphere(GATE_R)
        .translate((0.0, 0.0, BEND_Z))
    )

    # Bent section: cylinder along Z, then rotated and translated
    bent = (
        cq.Workplane("XY")
        .circle(GATE_R)
        .extrude(BENT_LEN)
    )
    # Rotate around Y axis by BEND_ANGLE (tilts toward +X)
    bent = bent.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), math.degrees(BEND_ANGLE))
    # Position so base is at the bend point
    bent = bent.translate((0.0, 0.0, BEND_Z))

    # Tip sphere at the end of the bent section
    tip_sphere = (
        cq.Workplane("XY")
        .sphere(GATE_R)
        .translate((_TIP_X, 0.0, _TIP_Z))
    )

    # Hinge boss (knuckle) at the bottom, wrapping the loop bar.
    # Pin axis = local Y (loop normal).
    boss = (
        cq.Workplane("XZ")
        .circle(GATE_R * 1.55)
        .extrude(0.0048, both=True)
    )

    gate = straight.union(bend_sphere).union(bent).union(tip_sphere).union(boss)
    return mesh_from_cadquery(gate, "gate_bar")


def _gate_key_mesh():
    """Key tongue at the bent tip: extends toward the nose (-X in gate local).

    This is the 'key' that seats into the nose slot groove when the gate
    closes, forming the positive hook-and-slot lock. Authored in gate local
    frame.
    """
    key = (
        cq.Workplane("XY")
        .box(KEY_LEN, KEY_W, KEY_H)
        .translate((_KEY_LOCAL_X, 0.0, _KEY_LOCAL_Z))
    )
    return mesh_from_cadquery(key, "gate_key")


def _hinge_pin_mesh():
    """Rivet/pin through the gate boss. Pin axis along Y (loop normal)."""
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

    # ---- body (root): the bent-bar open hook + nose slot lug ----
    body = model.part("body")
    body.visual(
        mesh_from_geometry(_body_hook_mesh(), "body_loop"),
        material=steel,
        name="body_loop",
    )
    body.visual(_nose_slot_mesh(), material=steel, name="nose_slot")
    body.inertial = Inertial.from_geometry(
        Box((0.052, 0.012, 0.100)),
        mass=0.085,
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
    )

    # ---- gate: bent key-lock style, revolute about bottom hinge rivet ----
    gate = model.part("gate")
    gate.visual(_gate_body_mesh(), material=steel, name="gate_bar")
    gate.visual(_gate_key_mesh(), material=steel, name="gate_key")
    gate.visual(_hinge_pin_mesh(), material=dark_steel, name="hinge_pin")
    gate.inertial = Inertial.from_geometry(
        Box((0.014, 0.008, 0.058)),
        mass=0.013,
        origin=Origin(xyz=(0.003, 0.0, 0.027)),
    )

    # Hinge at the bottom rivet of the gate-side. Axis = Y (loop normal) so the
    # gate top swings in the X-Z plane. Positive angle pushes the top toward +X
    # (into the loop), opening the gap between the gate tip and the nose.
    # Closed at 0 (spring-return).
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

    # ---- overlap allowances ----

    # Key tongue seats within the nose slot groove: close-fit mesh interface.
    ctx.allow_overlap(
        gate,
        body,
        elem_a="gate_key",
        elem_b="nose_slot",
        reason=(
            "Closed key-lock gate: key tongue seats within the nose slot groove "
            "(close-fit hook-and-slot capture)."
        ),
    )
    # Hinge boss wraps the loop bar at the pin (mounted hinge capture).
    ctx.allow_overlap(
        gate,
        body,
        elem_a="gate_bar",
        elem_b="body_loop",
        reason="Gate hinge boss wraps the loop bar at the pin (mounted hinge capture).",
    )
    # Hinge pin passes through the loop bar.
    ctx.allow_overlap(
        gate,
        body,
        elem_a="hinge_pin",
        elem_b="body_loop",
        reason="Hinge pin passes through the loop bar at the bottom of the gate-side.",
    )
    # Bent gate bar tip passes close to the nose slot lug when closed.
    ctx.allow_overlap(
        gate,
        body,
        elem_a="gate_bar",
        elem_b="nose_slot",
        reason=(
            "Bent gate bar tip sphere passes through the nose slot groove opening "
            "when closed (key-lock seating geometry)."
        ),
    )

    # ---- body is an open hook, taller than wide, eye at bottom ----
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

    # ---- gate key seats into nose slot when closed ----
    ctx.expect_contact(
        gate,
        body,
        elem_a="gate_key",
        elem_b="nose_slot",
        contact_tol=0.002,
        name="closed gate key seats into nose slot",
    )

    # ---- gate hinge is near the bottom of the loop ----
    gate_pos = ctx.part_world_position(gate)
    ctx.check(
        "gate hinge is near the bottom of the loop",
        gate_pos is not None and gate_pos[2] < 0.045,
        details=f"gate hinge world pos={gate_pos}",
    )

    # ---- gate has a visible bend: key tip is offset +X from the straight bar ----
    gate_bar_aabb = ctx.part_element_world_aabb(gate, elem="gate_bar")
    gate_key_aabb = ctx.part_element_world_aabb(gate, elem="gate_key")
    bar_min_x = gate_bar_aabb[0][0]
    key_max_x = gate_key_aabb[1][0]
    ctx.check(
        "gate has a visible bend: key tip is inward (+X) of the bar's outermost -X edge",
        key_max_x > bar_min_x + 0.008,
        details=f"bar min_x={bar_min_x:.5f}, key max_x={key_max_x:.5f}",
    )

    # ---- opening swings the gate key inward (+X) and away from the nose ----
    closed_key_aabb = ctx.part_element_world_aabb(gate, elem="gate_key")
    nose_aabb = ctx.part_element_world_aabb(body, elem="nose_slot")
    closed_key_cx = 0.5 * (closed_key_aabb[0][0] + closed_key_aabb[1][0])
    nose_cx = 0.5 * (nose_aabb[0][0] + nose_aabb[1][0])
    closed_gap = abs(closed_key_cx - nose_cx)

    with ctx.pose({hinge: math.radians(28.0)}):
        open_key_aabb = ctx.part_element_world_aabb(gate, elem="gate_key")
        open_key_cx = 0.5 * (open_key_aabb[0][0] + open_key_aabb[1][0])
    open_gap = abs(open_key_cx - nose_cx)

    ctx.check(
        "open gate key swings inward toward +X (into the loop)",
        open_key_cx > closed_key_cx + 0.004,
        details=f"closed key x={closed_key_cx:.5f}, open key x={open_key_cx:.5f}",
    )
    ctx.check(
        "opening moves the key away from the nose slot",
        open_gap > closed_gap + 0.003,
        details=f"closed gap={closed_gap:.5f}, open gap={open_gap:.5f}",
    )

    return ctx.report()


object_model = build_object_model()
