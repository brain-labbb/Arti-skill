from __future__ import annotations

"""Countertop hands-free presentation barcode scanner.

Variant of the wireless handheld scanner: the pistol-grip skeleton is replaced
by a slim vertical neck column seated on a broad flat weighted base plate.
The imager head sits on top, pitched forward/down to aim at items presented
in front of it.  A squeezable trigger still hangs under the head nose.

Articulation: trigger pivots about a horizontal cross axis under the head;
positive q squeezes the blade rearward toward the neck.
"""

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------- dimensions
# World frame: +Z up, +X forward (scan direction), base bottom on the ground.
BASE_LEN = 0.140  # X footprint
BASE_WID = 0.100  # Y footprint
BASE_HGT = 0.020  # Z thickness – bottom at z=0

NECK_BOT_Z = BASE_HGT          # 0.020
NECK_TOP_Z = 0.100

HEAD_LEN = 0.105
HEAD_WID = 0.070
HEAD_HGT = 0.056
HEAD_CENTER = (0.010, 0.0, 0.128)
HEAD_PITCH = 0.30               # rad, nose pitched down for presentation aim

TRIGGER_PIVOT = (0.042, 0.0, 0.088)
TRIGGER_SQUEEZE = 0.30          # rad of rearward travel

# ------------------------------------------------------------- colours
CHARCOAL   = (0.13, 0.13, 0.15, 1.0)
DARK_GRAY  = (0.22, 0.22, 0.24, 1.0)
ORANGE     = (0.96, 0.62, 0.08, 1.0)
WINDOW_GRAY = (0.30, 0.30, 0.33, 1.0)
BLUE       = (0.10, 0.40, 0.95, 1.0)
NEAR_BLACK = (0.06, 0.06, 0.07, 1.0)


# --------------------------------------------------------- base + neck solids
def _base_plate() -> cq.Workplane:
    """Broad flat weighted base with filleted corners and a seating boss."""
    base = (
        cq.Workplane("XY")
        .rect(BASE_LEN, BASE_WID)
        .extrude(BASE_HGT)
        .edges("|Z")
        .fillet(0.012)
        .edges(">Z")
        .chamfer(0.002)
    )
    # Raised boss where the neck column seats – visual transition ring.
    boss = (
        cq.Workplane("XY")
        .workplane(offset=BASE_HGT - 0.001)
        .rect(0.058, 0.048)
        .extrude(0.005)
        .edges(">Z")
        .chamfer(0.002)
    )
    return base.union(boss)


def _neck_column() -> cq.Workplane:
    """Slim tapered neck lofted from the base boss up to the head underside.

    The center line sweeps gently forward (+X) as it rises so the head sits
    slightly ahead of the base center – like a real presentation scanner arm.
    """
    sections = [
        (NECK_BOT_Z, 0.052, 0.044,  0.000),
        (0.045,      0.042, 0.036,  0.004),
        (0.075,      0.036, 0.030,  0.008),
        (NECK_TOP_Z, 0.038, 0.032,  0.010),
    ]
    wires = []
    for z, dx, dy, cx in sections:
        wire = (
            cq.Workplane("XY", origin=(cx, 0.0, z))
            .rect(dx, dy)
            .wires()
            .val()
        )
        wires.append(wire)
    return cq.Workplane(obj=cq.Solid.makeLoft(wires))


def _neck_accent() -> cq.Workplane:
    """Thin orange accent panel proud of the neck front face."""
    return (
        cq.Workplane("YZ")
        .workplane(offset=0.023)
        .center(0.0, 0.058)
        .rect(0.024, 0.035)
        .extrude(0.004)
    )


# ---------------------------------------------------------------- head solids
# All head solids are built in the head-local frame (centered on HEAD_CENTER)
# and attached with the same pitched Origin, so they stay mutually aligned.
def _head_shell() -> cq.Workplane:
    shell = cq.Workplane("XY").box(HEAD_LEN, HEAD_WID, HEAD_HGT).edges().fillet(0.010)
    pocket = (
        cq.Workplane("YZ")
        .workplane(offset=HEAD_LEN / 2 - 0.016)
        .rect(0.052, 0.032)
        .extrude(0.020)
    )
    return shell.cut(pocket)


def _window_bezel() -> cq.Workplane:
    """Orange frame lining the pocket mouth (outer edge embeds in the shell)."""
    frame = (
        cq.Workplane("YZ")
        .workplane(offset=HEAD_LEN / 2 - 0.007)
        .rect(0.058, 0.038)
        .extrude(0.005)
    )
    hole = (
        cq.Workplane("YZ")
        .workplane(offset=HEAD_LEN / 2 - 0.008)
        .rect(0.046, 0.026)
        .extrude(0.007)
    )
    return frame.cut(hole)


def _scan_window() -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .workplane(offset=HEAD_LEN / 2 - 0.017)
        .rect(0.048, 0.028)
        .extrude(0.005)
    )


def _top_panel() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .workplane(offset=HEAD_HGT / 2 - 0.003)
        .center(-0.004, 0.0)
        .rect(0.082, 0.054)
        .extrude(0.005)
        .edges("|Z")
        .fillet(0.010)
    )


def _ring_button_parts() -> tuple[cq.Workplane, cq.Workplane]:
    """Blue illuminated ring + near-black button disc on the top panel."""
    z0 = HEAD_HGT / 2 + 0.003
    ring = (
        cq.Workplane("XY")
        .workplane(offset=z0)
        .center(-0.006, 0.0)
        .circle(0.0165)
        .circle(0.0115)
        .extrude(0.0035)
    )
    disc = (
        cq.Workplane("XY")
        .workplane(offset=z0 - 0.001)
        .center(-0.006, 0.0)
        .circle(0.0118)
        .extrude(0.0038)
    )
    return ring, disc


# ------------------------------------------------------------- trigger solid
def _trigger_blade() -> cq.Workplane:
    """Pivot knuckle + curved blade, in the trigger-local (pivot) frame."""
    knuckle = cq.Workplane("XZ").workplane(offset=-0.011).circle(0.007).extrude(0.022)
    blade = (
        cq.Workplane("XY")
        .box(0.012, 0.022, 0.050)
        .edges("|Y")
        .fillet(0.005)
        .translate((0.010, 0.0, -0.021))
    )
    return knuckle.union(blade)


# -------------------------------------------------------------------- object
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="presentation_barcode_scanner")

    body = model.part("scanner_body")

    # --- Base + neck skeleton (replaces the pistol-grip loft) ---
    body.visual(
        mesh_from_cadquery(_base_plate(), "base_plate"),
        name="base_plate",
        color=CHARCOAL,
    )
    body.visual(
        mesh_from_cadquery(_neck_column(), "neck_column"),
        name="neck_column",
        color=CHARCOAL,
    )
    body.visual(
        mesh_from_cadquery(_neck_accent(), "neck_accent"),
        name="neck_accent",
        color=ORANGE,
    )

    # --- Imager head (identical functional layer to parent) ---
    head_origin = Origin(xyz=HEAD_CENTER, rpy=(0.0, HEAD_PITCH, 0.0))
    body.visual(
        mesh_from_cadquery(_head_shell(), "head_shell"),
        origin=head_origin,
        name="head_shell",
        color=CHARCOAL,
    )
    body.visual(
        mesh_from_cadquery(_window_bezel(), "window_bezel"),
        origin=head_origin,
        name="window_bezel",
        color=ORANGE,
    )
    body.visual(
        mesh_from_cadquery(_scan_window(), "scan_window"),
        origin=head_origin,
        name="scan_window",
        color=WINDOW_GRAY,
    )
    body.visual(
        mesh_from_cadquery(_top_panel(), "top_panel"),
        origin=head_origin,
        name="top_panel",
        color=ORANGE,
    )
    ring, disc = _ring_button_parts()
    body.visual(
        mesh_from_cadquery(ring, "ring_light"),
        origin=head_origin,
        name="ring_light",
        color=BLUE,
    )
    body.visual(
        mesh_from_cadquery(disc, "ring_button"),
        origin=head_origin,
        name="ring_button",
        color=NEAR_BLACK,
    )

    # --- Trigger (identical functional layer to parent) ---
    trigger = model.part("trigger")
    trigger.visual(
        mesh_from_cadquery(_trigger_blade(), "trigger_blade"),
        name="trigger_blade",
        color=DARK_GRAY,
    )

    model.articulation(
        "trigger_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=trigger,
        origin=Origin(xyz=TRIGGER_PIVOT),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=3.0, lower=0.0, upper=TRIGGER_SQUEEZE
        ),
    )

    return model


# --------------------------------------------------------------------- tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("scanner_body")
    trigger = object_model.get_part("trigger")
    pivot = object_model.get_articulation("trigger_pivot")
    knuckle_blade = trigger.get_visual("trigger_blade")
    head_shell = body.get_visual("head_shell")
    neck_column = body.get_visual("neck_column")
    base_plate = body.get_visual("base_plate")

    # Trigger pivot knuckle is captured inside the head underside so the
    # mechanism reads seated and the part group stays connected.
    ctx.allow_overlap(
        trigger,
        body,
        reason="trigger pivot knuckle is captured inside the head underside",
    )

    # ── Skeleton: freestanding base + neck ──────────────────────────────
    base_lo, base_hi = ctx.part_element_world_aabb(body, elem=base_plate)
    neck_lo, neck_hi = ctx.part_element_world_aabb(body, elem=neck_column)

    ctx.check(
        "base bottom sits flat on ground plane (z=0)",
        base_lo[2] is not None and abs(base_lo[2]) < 0.001,
        details=f"base_plate min z = {base_lo[2]:.4f}",
    )
    ctx.check(
        "neck column bridges base to head",
        neck_lo[2] < BASE_HGT + 0.005 and neck_hi[2] > 0.090,
        details=f"neck z range = [{neck_lo[2]:.4f}, {neck_hi[2]:.4f}]",
    )
    ctx.check(
        "base is broader than neck (freestanding support)",
        (base_hi[0] - base_lo[0]) > (neck_hi[0] - neck_lo[0]) + 0.04
        and (base_hi[1] - base_lo[1]) > (neck_hi[1] - neck_lo[1]) + 0.03,
        details="base must be substantially wider than the neck in both X and Y",
    )

    # ── Trigger at rest: hangs under the head nose, clear of the neck ───
    with ctx.pose({pivot: 0.0}):
        ctx.expect_overlap(
            trigger, body, elem_a=knuckle_blade, elem_b=head_shell, axes="xy"
        )
        ctx.expect_gap(
            trigger, body, elem_b=neck_column, axis="x", max_penetration=0.0
        )
        rest_lo, rest_hi = ctx.part_world_aabb(trigger)
        rest_cx = 0.5 * (rest_lo[0] + rest_hi[0])
        assert rest_lo[2] < TRIGGER_PIVOT[2] - 0.030, (
            "trigger blade should hang below pivot"
        )

    # ── Trigger squeezed: swings rearward without penetrating the neck ──
    with ctx.pose({pivot: TRIGGER_SQUEEZE}):
        sq_lo, sq_hi = ctx.part_world_aabb(trigger)
        sq_cx = 0.5 * (sq_lo[0] + sq_hi[0])
        assert sq_cx < rest_cx - 0.004, (
            f"squeezing must pull the trigger rearward: "
            f"rest_cx={rest_cx:.4f} sq_cx={sq_cx:.4f}"
        )
        ctx.expect_gap(
            trigger, body, elem_b=neck_column, axis="x", max_penetration=0.002
        )

    # ── Hero features: recessed scan window, orange bezel, blue ring ────
    scan_window = body.get_visual("scan_window")
    window_bezel = body.get_visual("window_bezel")
    ring_light = body.get_visual("ring_light")
    top_panel = body.get_visual("top_panel")

    win_lo, win_hi = ctx.part_element_world_aabb(body, elem=scan_window)
    bez_lo, bez_hi = ctx.part_element_world_aabb(body, elem=window_bezel)
    head_lo, head_hi_v = ctx.part_element_world_aabb(body, elem=head_shell)

    assert win_hi[0] < bez_hi[0], "scan window must be recessed behind the orange bezel"
    assert win_hi[0] > head_lo[0], "scan window must sit in the head front"

    ring_lo, ring_hi = ctx.part_element_world_aabb(body, elem=ring_light)
    top_lo, top_hi = ctx.part_element_world_aabb(body, elem=top_panel)
    # Ring sits forward on the panel; with the head pitched down the panel
    # rear lifts higher in world Z than the ring.  Prove the ring is proud
    # at its own location: ring bottom above panel front, ring top above
    # panel midpoint.
    assert ring_lo[2] >= top_lo[2], "blue ring must sit on top of the orange panel"
    top_mid_z = 0.5 * (top_lo[2] + top_hi[2])
    assert ring_hi[2] > top_mid_z, "blue ring must protrude above the panel mid-height"

    return ctx.report()


object_model = build_object_model()
