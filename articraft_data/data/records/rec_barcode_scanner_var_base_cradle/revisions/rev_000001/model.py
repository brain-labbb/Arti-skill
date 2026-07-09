from __future__ import annotations

"""Wireless handheld barcode scanner docked in its charging cradle.

The scanner seats nose-up in a dark charging/presentation cradle with a
weighted foot plate and an upright saddle cup that cradles the grip front
with an integrated head-support plate for the scan head nose.

Scanner: charcoal pistol-grip body, wide flat scan head with recessed dark
scan window lined by an orange bezel, orange-yellow top panel with blue
illuminated ring button, orange rubber grip inserts, squeezable trigger.

Articulation: the trigger pivots about a horizontal cross axis under the head;
positive q squeezes the blade rearward toward the grip.
"""

import math

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
# World frame: +Z up, cradle foot plate bottom is the ground interface (z=0).

# --- Scanner geometry (built in scanner_body frame, preserved from parent) ---
GRIP_BOT_Z = 0.0
GRIP_TOP_Z = 0.115

HEAD_LEN = 0.105
HEAD_WID = 0.070
HEAD_HGT = 0.056
HEAD_CENTER = (0.020, 0.0, 0.135)
HEAD_PITCH = 0.20  # rad, nose pitched down

TRIGGER_PIVOT = (0.045, 0.0, 0.100)
TRIGGER_SQUEEZE = 0.30  # rad rearward travel

# --- Dock pose: scanner_body frame placed in cradle_base (world) frame ---
DOCK_PITCH = -0.38  # rad (~22° back tilt, nose-up presentation)
DOCK_XYZ = (0.008, 0.0, 0.020)

# --- Cradle dimensions ---
FOOT_X = 0.140
FOOT_Y = 0.090
FOOT_Z = 0.012

# --- Colors ---
CHARCOAL = (0.13, 0.13, 0.15, 1.0)
DARK_GRAY = (0.22, 0.22, 0.24, 1.0)
ORANGE = (0.96, 0.62, 0.08, 1.0)
WINDOW_GRAY = (0.30, 0.30, 0.33, 1.0)
BLUE = (0.10, 0.40, 0.95, 1.0)
NEAR_BLACK = (0.06, 0.06, 0.07, 1.0)
CRADLE_DARK = (0.14, 0.14, 0.16, 1.0)
CRADLE_MID = (0.22, 0.22, 0.24, 1.0)
LED_GREEN = (0.15, 0.80, 0.25, 1.0)


# ---------------------------------------------------------------- grip solids
def _grip_shell() -> cq.Workplane:
    """Tapered pistol grip lofted from rounded-rect sections (flared base)."""
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
    """Orange rubber strip lofted to follow the grip taper."""
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
    """Orange frame lining the pocket mouth."""
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


# ---------------------------------------------------------------- cradle solids
def _cradle_foot_plate() -> cq.Workplane:
    """Weighted flat foot plate, bottom face at z=0 (ground interface)."""
    return (
        cq.Workplane("XY")
        .box(FOOT_X, FOOT_Y, FOOT_Z)
        .edges("|Z")
        .fillet(0.006)
        .translate((0.0, 0.0, FOOT_Z / 2))
    )


def _cradle_saddle() -> cq.Workplane:
    """Upright U-channel saddle cup with integrated head-support plate.

    Built in vertical frame, then rotated to DOCK_PITCH and placed on the
    foot plate.  The head-support plate extends forward from risers on the
    channel side walls so the head chin rests on it from above.
    """
    h = 0.088  # channel height
    wall_t = 0.005
    channel_iw = 0.048  # inner Y width (accepts the ~42 mm grip)
    channel_id = 0.034  # inner X depth
    floor_t = 0.005

    total_ow = channel_iw + 2 * wall_t  # 0.058
    total_od = channel_id + wall_t  # 0.039

    # Back wall (at -X side of the channel)
    back = (
        cq.Workplane("XY")
        .box(wall_t, total_ow, h)
        .edges("|Z")
        .fillet(0.002)
        .translate((-total_od / 2 + wall_t / 2, 0.0, h / 2))
    )

    # Side walls
    side_xc = -total_od / 2 + wall_t + channel_id / 2  # ≈ 0.002
    side_y = total_ow / 2 - wall_t / 2  # ≈ 0.0265
    left = (
        cq.Workplane("XY")
        .box(channel_id, wall_t, h)
        .edges("|Z")
        .fillet(0.002)
        .translate((side_xc, side_y, h / 2))
    )
    right = (
        cq.Workplane("XY")
        .box(channel_id, wall_t, h)
        .edges("|Z")
        .fillet(0.002)
        .translate((side_xc, -side_y, h / 2))
    )

    # Floor plate
    floor_plate = (
        cq.Workplane("XY")
        .box(total_od, total_ow, floor_t)
        .edges("|Z")
        .fillet(0.002)
        .translate((0.0, 0.0, floor_t / 2))
    )

    saddle = back.union(left).union(right).union(floor_plate)

    # --- Head support structure: risers + plate ---
    riser_h = 0.012
    riser_t = 0.008

    riser_l = (
        cq.Workplane("XY")
        .box(riser_t, riser_t, riser_h)
        .translate((side_xc, side_y, h + riser_h / 2))
    )
    riser_r = (
        cq.Workplane("XY")
        .box(riser_t, riser_t, riser_h)
        .translate((side_xc, -side_y, h + riser_h / 2))
    )

    plate_len = 0.035
    plate_w = 0.065
    plate_t = 0.006

    head_plate = (
        cq.Workplane("XY")
        .box(plate_len, plate_w, plate_t)
        .edges("|Z")
        .fillet(0.002)
        .translate((side_xc + plate_len / 2, 0.0, h + riser_h + plate_t / 2))
    )

    saddle = saddle.union(riser_l).union(riser_r).union(head_plate)

    # Rotate to dock angle about Y
    saddle = saddle.rotate((0, 0, 0), (0, 1, 0), math.degrees(DOCK_PITCH))

    # Place on foot plate: saddle base overlaps the foot plate top for
    # connectivity.
    saddle = saddle.translate((DOCK_XYZ[0] - 0.008, 0.0, FOOT_Z - 0.001))

    return saddle


def _charge_led() -> cq.Workplane:
    """Small green indicator LED on the foot plate front edge."""
    return (
        cq.Workplane("XY")
        .workplane(offset=FOOT_Z - 0.001)  # embeds 1 mm into foot plate
        .center(FOOT_X / 2 - 0.012, 0.0)
        .circle(0.004)
        .extrude(0.003)
    )


# -------------------------------------------------------------------- object
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wireless_barcode_scanner_docked")

    # --- cradle_base (root part, ground interface) ---
    cradle = model.part("cradle_base")
    cradle.visual(
        mesh_from_cadquery(_cradle_foot_plate(), "foot_plate"),
        name="foot_plate",
        color=CRADLE_DARK,
    )
    cradle.visual(
        mesh_from_cadquery(_cradle_saddle(), "saddle_cup"),
        name="saddle_cup",
        color=CRADLE_DARK,
    )
    cradle.visual(
        mesh_from_cadquery(_charge_led(), "charge_led"),
        name="charge_led",
        color=LED_GREEN,
    )

    # --- scanner_body (child of cradle via FIXED dock articulation) ---
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

    # --- trigger (child of scanner_body, articulation preserved from parent) ---
    trigger = model.part("trigger")
    trigger.visual(
        mesh_from_cadquery(_trigger_blade(), "trigger_blade"),
        name="trigger_blade",
        color=DARK_GRAY,
    )

    # --- articulations ---
    # FIXED: cradle_base -> scanner_body (positions scanner nose-up in dock)
    model.articulation(
        "cradle_to_scanner",
        ArticulationType.FIXED,
        parent=cradle,
        child=body,
        origin=Origin(xyz=DOCK_XYZ, rpy=(0.0, DOCK_PITCH, 0.0)),
    )

    # REVOLUTE: scanner_body -> trigger (preserved from parent)
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
    cradle = object_model.get_part("cradle_base")
    body = object_model.get_part("scanner_body")
    trigger = object_model.get_part("trigger")
    pivot = object_model.get_articulation("trigger_pivot")

    foot_plate = cradle.get_visual("foot_plate")
    saddle_cup = cradle.get_visual("saddle_cup")
    grip_shell = body.get_visual("grip_shell")
    head_shell = body.get_visual("head_shell")
    scan_window = body.get_visual("scan_window")
    window_bezel = body.get_visual("window_bezel")
    ring_light = body.get_visual("ring_light")
    top_panel = body.get_visual("top_panel")
    trigger_blade = trigger.get_visual("trigger_blade")

    # --- cradle_base ground interface ---
    foot_lo, foot_hi = ctx.part_element_world_aabb(cradle, elem=foot_plate)
    ctx.check(
        "cradle_base foot plate bottom is the ground interface",
        foot_lo[2] < 0.002,
        details=f"foot plate min z={foot_lo[2]:.4f}, expected ~0",
    )

    # --- scanner seats nose-up in the cradle ---
    grip_lo, grip_hi = ctx.part_element_world_aabb(body, elem=grip_shell)
    win_lo, win_hi = ctx.part_element_world_aabb(body, elem=scan_window)
    ctx.check(
        "scanner seats nose-up: scan window above grip base",
        win_hi[2] > grip_hi[2] + 0.020,
        details=f"scan_window top z={win_hi[2]:.4f}, grip top z={grip_hi[2]:.4f}",
    )

    # --- saddle cup cradles the grip (XY footprint overlap) ---
    ctx.expect_overlap(
        cradle,
        body,
        elem_a=saddle_cup,
        elem_b=grip_shell,
        axes="xy",
        name="saddle cup cradles the scanner grip",
    )

    # --- saddle head-support plate near the head chin region (Z proximity) ---
    saddle_lo, saddle_hi = ctx.part_element_world_aabb(cradle, elem=saddle_cup)
    head_lo, head_hi = ctx.part_element_world_aabb(body, elem=head_shell)
    ctx.check(
        "saddle head-support plate rises into the head region",
        saddle_hi[2] > head_lo[2] + 0.005,
        details=f"saddle top z={saddle_hi[2]:.4f}, head bottom z={head_lo[2]:.4f}",
    )

    # --- scanner features preserved ---
    bez_lo, bez_hi = ctx.part_element_world_aabb(body, elem=window_bezel)
    ctx.check(
        "scan window recessed behind orange bezel",
        win_hi[0] < bez_hi[0],
        details=f"window front x={win_hi[0]:.4f}, bezel front x={bez_hi[0]:.4f}",
    )
    # Ring button sits on the top panel (XY footprint overlap)
    ctx.expect_overlap(
        body,
        body,
        elem_a=ring_light,
        elem_b=top_panel,
        axes="xy",
        name="blue ring sits on the top panel footprint",
    )

    # --- trigger articulation preserved ---
    # The trigger knuckle intentionally embeds into the head underside so the
    # mechanism reads seated (and the part group stays connected).
    ctx.allow_overlap(
        trigger,
        body,
        reason="trigger pivot knuckle is captured inside the head underside",
    )

    # Allow overlap where the grip seats against the saddle U-channel walls
    ctx.allow_overlap(
        cradle,
        body,
        elem_a=saddle_cup,
        elem_b=grip_shell,
        reason="grip seats inside the saddle U-channel walls for docked support",
    )

    # The orange grip inserts are slightly proud of the grip shell and sit
    # inside the cradle channel walls
    ctx.allow_overlap(
        cradle,
        body,
        elem_a=saddle_cup,
        elem_b="grip_insert_wrap",
        reason="rubber grip inserts sit flush inside the saddle channel",
    )

    # Allow overlap where the head-support plate cradles the head chin
    ctx.allow_overlap(
        cradle,
        body,
        elem_a=saddle_cup,
        elem_b=head_shell,
        reason="integrated head-support plate cradles the scanner head chin from below",
    )

    with ctx.pose({pivot: 0.0}):
        ctx.expect_overlap(
            trigger, body, elem_a=trigger_blade, elem_b=head_shell, axes="xy"
        )
        rest_lo_t, rest_hi_t = ctx.part_world_aabb(trigger)
        rest_front_x = rest_hi_t[0]
        # Trigger blade hangs below the pivot point in world Z
        pivot_pos = ctx.part_world_position(trigger)
        ctx.check(
            "trigger blade hangs below pivot",
            rest_lo_t[2] < pivot_pos[2] - 0.015,
            details=f"blade bottom z={rest_lo_t[2]:.4f}, pivot z={pivot_pos[2]:.4f}",
        )

    with ctx.pose({pivot: TRIGGER_SQUEEZE}):
        sq_lo_t, sq_hi_t = ctx.part_world_aabb(trigger)
        sq_front_x = sq_hi_t[0]
        ctx.check(
            "squeeze pulls trigger rearward (front X decreases)",
            sq_front_x < rest_front_x - 0.002,
            details=f"rest front x={rest_front_x:.4f}, squeezed front x={sq_front_x:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
