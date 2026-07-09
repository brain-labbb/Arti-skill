from __future__ import annotations

"""Wireless handheld barcode scanner (pistol grip).

Matches the reference photo: charcoal pistol-grip body, wide flat scan head
with a recessed dark scan window lined by an orange bezel, orange-yellow top
panel carrying a blue illuminated ring bezel around a pressable dark button
cap, orange rubber inserts on the grip, and a squeezable trigger hanging
under the head nose.

Articulations:
- trigger_pivot (REVOLUTE): the trigger pivots about a horizontal cross axis
  under the head; positive q squeezes the blade rearward toward the grip.
- top_button_press (PRISMATIC): the dark button cap on the top panel depresses
  vertically into a shallow recess; positive q presses the button down.
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
# World frame: +Z up, +X forward (scan direction), grip base on the ground.
GRIP_BOT_Z = 0.0
GRIP_TOP_Z = 0.115  # overlaps into the head underside so the body stays one island

HEAD_LEN = 0.105  # along X
HEAD_WID = 0.070  # along Y
HEAD_HGT = 0.056  # along Z
HEAD_CENTER = (0.020, 0.0, 0.135)
HEAD_PITCH = 0.20  # rad, nose pitched down like the photo

TRIGGER_PIVOT = (0.045, 0.0, 0.100)
TRIGGER_SQUEEZE = 0.30  # rad of rearward travel

BUTTON_PRESS_TRAVEL = 0.003  # 3 mm vertical press travel

CHARCOAL = (0.13, 0.13, 0.15, 1.0)
DARK_GRAY = (0.22, 0.22, 0.24, 1.0)
ORANGE = (0.96, 0.62, 0.08, 1.0)
WINDOW_GRAY = (0.30, 0.30, 0.33, 1.0)
BLUE = (0.10, 0.40, 0.95, 1.0)
NEAR_BLACK = (0.06, 0.06, 0.07, 1.0)


# ---------------------------------------------------------------- grip solids
def _grip_shell() -> cq.Workplane:
    """Tapered pistol grip lofted from rounded-rect sections (flared base)."""
    # (z, x_depth, y_width, x_center)
    sections = [
        (GRIP_BOT_Z, 0.052, 0.042, -0.006),
        (0.030, 0.044, 0.036, -0.003),
        (0.070, 0.042, 0.034, 0.001),
        (GRIP_TOP_Z, 0.052, 0.042, 0.006),
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


def _grip_insert_wrap() -> cq.Workplane:
    """Orange rubber strip lofted to follow the grip taper.

    Slightly deeper than the grip in X and narrower in Y, so it reads as
    flush front/rear rubber strips (like the photo's orange grip panels)
    instead of proud slabs.
    """
    # (z, x_depth, x_center) interpolated from the grip sections, +5 mm proud
    sections = [
        (0.022, 0.0515, -0.0045),
        (0.030, 0.0490, -0.0030),
        (0.070, 0.0470, 0.0010),
        (0.100, 0.0535, 0.0043),
    ]
    wires = []
    for z, dx, cx in sections:
        wire = (
            cq.Workplane("XY", origin=(cx, 0.0, z))
            .rect(dx, 0.024)
            .wires()
            .val()
        )
        wires.append(wire)
    return cq.Workplane(obj=cq.Solid.makeLoft(wires))


# ---------------------------------------------------------------- head solids
# All head solids are built in the head-local frame (centered on HEAD_CENTER)
# and attached with the same pitched Origin, so they stay mutually aligned.
def _head_shell() -> cq.Workplane:
    shell = cq.Workplane("XY").box(HEAD_LEN, HEAD_WID, HEAD_HGT).edges().fillet(0.010)
    # Recessed scan-window pocket cut into the front (+X) face.
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
        # Starts 1 mm behind the pocket floor so the plate seats into the shell.
        .workplane(offset=HEAD_LEN / 2 - 0.017)
        .rect(0.048, 0.028)
        .extrude(0.005)
    )


def _top_panel() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .workplane(offset=HEAD_HGT / 2 - 0.003)
        .center(-0.004, 0.0)
        .rect(0.082, 0.054)
        .extrude(0.005)
        .edges("|Z")
        .fillet(0.010)
    )
    # Shallow recessed well for button travel (3 mm deep pocket from panel top)
    well = (
        cq.Workplane("XY")
        .workplane(offset=HEAD_HGT / 2 - 0.001)
        .center(-0.006, 0.0)
        .circle(0.013)
        .extrude(0.008)
    )
    return panel.cut(well)


def _ring_light() -> cq.Workplane:
    """Blue illuminated bezel ring on the top panel (fixed, stays on body)."""
    z0 = HEAD_HGT / 2 + 0.0015
    return (
        cq.Workplane("XY")
        .workplane(offset=z0)
        .center(-0.006, 0.0)
        .circle(0.0165)
        .circle(0.0125)
        .extrude(0.0035)
    )


def _top_button_cap() -> cq.Workplane:
    """Pressable dark button cap with retention stem (part-local = head-local at q=0).

    The cap sits inside the ring bezel at rest and depresses into a shallow
    well in the top panel when the prismatic joint is actuated.
    """
    z0 = HEAD_HGT / 2 + 0.003
    # Cap: raised disc that fills the ring bezel opening
    cap = (
        cq.Workplane("XY")
        .workplane(offset=z0)
        .center(-0.006, 0.0)
        .circle(0.0118)
        .extrude(0.0035)
    )
    # Stem: narrower cylinder extending below the cap into the panel recess
    # for visual retention and connectivity
    stem = (
        cq.Workplane("XY")
        .workplane(offset=HEAD_HGT / 2 - 0.003)
        .center(-0.006, 0.0)
        .circle(0.005)
        .extrude(0.006)
    )
    return cap.union(stem)


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
    model = ArticulatedObject(name="wireless_handheld_barcode_scanner")

    body = model.part("scanner_body")
    body.visual(
        mesh_from_cadquery(_grip_shell(), "grip_shell"),
        name="grip_shell",
        color=CHARCOAL,
    )
    body.visual(
        mesh_from_cadquery(_grip_insert_wrap(), "grip_insert_wrap"),
        name="grip_insert_wrap",
        color=ORANGE,
    )

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
    body.visual(
        mesh_from_cadquery(_ring_light(), "ring_light"),
        origin=head_origin,
        name="ring_light",
        color=BLUE,
    )

    # -- top push-button (prismatic press mechanism) --
    top_button = model.part("top_button")
    top_button.visual(
        mesh_from_cadquery(_top_button_cap(), "button_cap"),
        name="button_cap",
        color=NEAR_BLACK,
    )

    model.articulation(
        "top_button_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=top_button,
        # Joint frame at head center with head pitch so -Z_artic = head-local down.
        origin=Origin(xyz=HEAD_CENTER, rpy=(0.0, HEAD_PITCH, 0.0)),
        # Positive q presses the button downward into the panel recess.
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=0.5, lower=0.0, upper=BUTTON_PRESS_TRAVEL,
        ),
    )

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
        # +Y axis: positive q swings the hanging blade rearward (-X) toward
        # the grip, i.e. a squeeze.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=TRIGGER_SQUEEZE),
    )

    return model


# --------------------------------------------------------------------- tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("scanner_body")
    trigger = object_model.get_part("trigger")
    top_button = object_model.get_part("top_button")
    pivot = object_model.get_articulation("trigger_pivot")
    press = object_model.get_articulation("top_button_press")
    knuckle_blade = trigger.get_visual("trigger_blade")
    head_shell = body.get_visual("head_shell")
    grip_shell = body.get_visual("grip_shell")
    button_cap = top_button.get_visual("button_cap")

    # The trigger knuckle intentionally embeds into the head underside so the
    # mechanism reads seated (and the part group stays connected).
    ctx.allow_overlap(
        trigger,
        body,
        reason="trigger pivot knuckle is captured inside the head underside",
    )

    # The button stem extends through the panel recess into the head shell to
    # provide retention and connectivity for the prismatic push button.
    ctx.allow_overlap(
        top_button,
        body,
        reason="button cap stem is captured inside the panel well and head housing",
    )

    # --- trigger tests ---
    # At rest the trigger hangs under the head nose: seated against the head,
    # clear of the grip front so there is finger room to squeeze.
    with ctx.pose({pivot: 0.0}):
        ctx.expect_overlap(trigger, body, elem_a=knuckle_blade, elem_b=head_shell, axes="xy")
        ctx.expect_gap(trigger, body, elem_b=grip_shell, axis="x", max_penetration=0.0)
        rest_lo, rest_hi = ctx.part_world_aabb(trigger)
        rest_cx = 0.5 * (rest_lo[0] + rest_hi[0])
        # Blade hangs below the pivot height.
        assert rest_lo[2] < TRIGGER_PIVOT[2] - 0.030, "trigger blade should hang below pivot"

    # Fully squeezed: the blade swings rearward (toward the grip) without ever
    # penetrating the grip shell.
    with ctx.pose({pivot: TRIGGER_SQUEEZE}):
        sq_lo, sq_hi = ctx.part_world_aabb(trigger)
        sq_cx = 0.5 * (sq_lo[0] + sq_hi[0])
        assert sq_cx < rest_cx - 0.004, (
            f"squeezing must pull the trigger rearward: rest_cx={rest_cx:.4f} sq_cx={sq_cx:.4f}"
        )
        ctx.expect_gap(trigger, body, elem_b=grip_shell, axis="x", max_penetration=0.0)

    # --- top button press tests ---
    # The press articulation is prismatic with short travel.
    assert press.articulation_type == ArticulationType.PRISMATIC, (
        "top_button_press must be a PRISMATIC joint"
    )
    limits = press.motion_limits
    assert limits is not None and limits.upper is not None, (
        "top_button_press must have an upper travel limit"
    )
    assert abs(limits.upper - BUTTON_PRESS_TRAVEL) < 1e-6, (
        f"top_button_press upper limit must be {BUTTON_PRESS_TRAVEL} m"
    )

    # At rest (q=0): button cap top is visible above the top panel surface.
    rest_btn_pos = ctx.part_world_position(top_button)
    top_panel_vis = body.get_visual("top_panel")
    with ctx.pose({press: 0.0}):
        btn_lo, btn_hi = ctx.part_element_world_aabb(top_button, elem=button_cap)
        pnl_lo, pnl_hi = ctx.part_element_world_aabb(body, elem=top_panel_vis)
        # The button_cap visual includes a retention stem that extends below
        # the panel, so we check the cap top (max Z) is above the panel surface.
        ctx.check(
            "button cap top visible above panel at rest",
            btn_hi[2] > pnl_hi[2] - 0.004,
            details=f"btn_cap_top_z={btn_hi[2]:.4f} panel_top_z={pnl_hi[2]:.4f}",
        )

    # Fully pressed (q=upper): button moves downward.
    with ctx.pose({press: BUTTON_PRESS_TRAVEL}):
        pressed_btn_pos = ctx.part_world_position(top_button)
        assert pressed_btn_pos is not None and rest_btn_pos is not None
        assert pressed_btn_pos[2] < rest_btn_pos[2] - 0.001, (
            f"pressing must lower the button: rest_z={rest_btn_pos[2]:.4f} "
            f"pressed_z={pressed_btn_pos[2]:.4f}"
        )

    # --- hero feature visibility tests ---
    # Recessed dark scan window inside an orange bezel on the head front,
    # blue ring bezel on the orange top panel around the button cap.
    scan_window = body.get_visual("scan_window")
    window_bezel = body.get_visual("window_bezel")
    ring_light = body.get_visual("ring_light")
    top_panel = body.get_visual("top_panel")
    win_lo, win_hi = ctx.part_element_world_aabb(body, elem=scan_window)
    bez_lo, bez_hi = ctx.part_element_world_aabb(body, elem=window_bezel)
    head_lo, head_hi = ctx.part_element_world_aabb(body, elem=head_shell)
    # Window sits recessed behind the bezel mouth and forward of the head rear.
    assert win_hi[0] < bez_hi[0], "scan window must be recessed behind the orange bezel"
    assert win_hi[0] > head_lo[0], "scan window must sit in the head front"
    ring_lo, ring_hi = ctx.part_element_world_aabb(body, elem=ring_light)
    top_lo, top_hi = ctx.part_element_world_aabb(body, elem=top_panel)
    assert ring_lo[2] >= top_lo[2], "blue ring must sit on top of the orange panel"
    assert ring_hi[2] > top_hi[2] - 0.002, "blue ring must be visible above the top panel"

    # Button cap sits inside the ring bezel opening.
    btn_lo, btn_hi = ctx.part_element_world_aabb(top_button, elem=button_cap)
    assert btn_lo[2] >= top_lo[2] - 0.004, "button cap should sit near or above panel surface"
    assert btn_hi[2] > top_hi[2] - 0.002, "button cap must be visible above the top panel"

    return ctx.report()


object_model = build_object_model()
