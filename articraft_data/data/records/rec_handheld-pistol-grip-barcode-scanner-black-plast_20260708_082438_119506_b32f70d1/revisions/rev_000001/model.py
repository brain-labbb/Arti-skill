"""Handheld pistol-grip barcode scanner.

Matches the reference photo: a black gun-style handheld scanner with a wide
rounded scan head on top, a dark recessed scan window emitting a red laser
line, a blue rubber trigger on the front of the raked grip, a white spec label
under the head, and a coiled black cable exiting the base of the handle.

The trigger is the primary articulation: a REVOLUTE squeeze toward the grip.
"""

from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------- constants
# World frame: +X = forward (the direction the scan window faces), +Z = up.
# The scanner stands on the flared base of its grip at z = 0.

HEAD_TILT = -0.30  # rad about +Y; negative pitches the head nose upward
HEAD_POS = (0.012, 0.0, 0.150)  # world position of the scan-head center

HEAD_LEN = 0.100  # local x (front-back)
HEAD_WID = 0.066  # local y
HEAD_HGT = 0.048  # local z

POCKET_WID = 0.044
POCKET_HGT = 0.026
POCKET_FLOOR_X = 0.032  # local x of the recess floor (front face at x=0.050)

TRIGGER_PIVOT = (0.022, 0.0, 0.121)  # world position of the trigger hinge
TRIGGER_SQUEEZE = 0.22  # rad of squeeze travel

BLACK_PLASTIC = Material("black_plastic", rgba=(0.08, 0.08, 0.09, 1.0))
DARK_GLASS = Material("dark_scan_glass", rgba=(0.03, 0.03, 0.04, 1.0))
LASER_RED = Material("laser_red", rgba=(1.0, 0.08, 0.05, 1.0))
TRIGGER_BLUE = Material("trigger_blue", rgba=(0.13, 0.20, 0.75, 1.0))
LABEL_WHITE = Material("label_white", rgba=(0.92, 0.92, 0.90, 1.0))
CABLE_BLACK = Material("cable_black", rgba=(0.05, 0.05, 0.06, 1.0))


def _head_local_to_world(p: tuple[float, float, float]) -> tuple[float, float, float]:
    """Rotate a scan-head local point by HEAD_TILT about +Y, then translate."""
    c, s = math.cos(HEAD_TILT), math.sin(HEAD_TILT)
    x = p[0] * c + p[2] * s
    z = -p[0] * s + p[2] * c
    return (HEAD_POS[0] + x, HEAD_POS[1] + p[1], HEAD_POS[2] + z)


def _grip_solid() -> cq.Workplane:
    """Raked pistol grip: loft of elliptical sections, flared at the base."""
    # (z, center_x, half_depth_x, half_width_y)
    sections = [
        (0.000, -0.030, 0.027, 0.022),  # flared standing base
        (0.018, -0.027, 0.021, 0.017),
        (0.060, -0.016, 0.019, 0.0155),  # slim waist of the grip
        (0.100, -0.004, 0.021, 0.017),
        (0.128, 0.006, 0.026, 0.021),  # neck blending into the head
    ]
    wp = cq.Workplane("XY")
    prev_z, prev_cx = 0.0, 0.0
    for z, cx, rx, ry in sections:
        wp = wp.workplane(offset=z - prev_z).center(cx - prev_cx, 0.0).ellipse(rx, ry)
        prev_z, prev_cx = z, cx
    return wp.loft(ruled=False)


def _head_solid() -> cq.Workplane:
    """Rounded scan head with the front scan-window pocket cut in."""
    head = cq.Workplane("XY").box(HEAD_LEN, HEAD_WID, HEAD_HGT).edges().fillet(0.012)
    # Pocket overshoots the front face so the cut leaves a clean recess.
    pocket = (
        cq.Workplane("XY")
        .box(0.030, POCKET_WID, POCKET_HGT)
        .translate((POCKET_FLOOR_X + 0.015, 0.0, 0.0))
    )
    return head.cut(pocket)


def _trigger_solid() -> cq.Workplane:
    """Blue trigger pad, built in the trigger frame (pivot at the origin).

    The pad hangs below the pivot and bulges forward (+X); a short stem
    reaches back (-X) into the grip shell.
    """
    pad = (
        cq.Workplane("XY")
        .box(0.014, 0.024, 0.034)
        .translate((0.007, 0.0, -0.018))
        .edges()
        .fillet(0.0055)
    )
    stem = cq.Workplane("XY").box(0.016, 0.016, 0.010).translate((-0.008, 0.0, -0.005))
    return pad.union(stem)


def _cable_points() -> list[tuple[float, float, float]]:
    """Lead-out from the grip base into a standing coil behind the scanner."""
    pts: list[tuple[float, float, float]] = [
        (-0.045, 0.000, 0.010),  # embedded inside the base flare
        (-0.058, 0.002, 0.016),
        (-0.070, 0.006, 0.026),
    ]
    coil_center = (-0.088, 0.0)
    coil_r = 0.016
    turns = 3.0
    top_z, bottom_z = 0.030, 0.006
    n = 60
    for i in range(n + 1):
        t = i / n
        a = 0.55 + 2.0 * math.pi * turns * t
        pts.append(
            (
                coil_center[0] + coil_r * math.cos(a),
                coil_center[1] + coil_r * math.sin(a),
                top_z + (bottom_z - top_z) * t,
            )
        )
    return pts


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="handheld_barcode_scanner")

    # ------------------------------------------------------------- body
    body = model.part("scanner_body")

    body.visual(
        mesh_from_cadquery(_grip_solid(), "grip_loft"),
        origin=Origin(),
        material=BLACK_PLASTIC,
        name="grip_shell",
    )

    body.visual(
        mesh_from_cadquery(_head_solid(), "scan_head"),
        origin=Origin(xyz=HEAD_POS, rpy=(0.0, HEAD_TILT, 0.0)),
        material=BLACK_PLASTIC,
        name="scan_head",
    )

    # Dark glass window plate seated on the pocket floor (slightly embedded
    # into the floor so it reads as one assembly, clearly recessed from the
    # front rim of the head).
    window_center = _head_local_to_world((0.0335, 0.0, 0.001))
    body.visual(
        Box((0.004, 0.042, 0.024)),
        origin=Origin(xyz=window_center, rpy=(0.0, HEAD_TILT, 0.0)),
        material=DARK_GLASS,
        name="scan_window",
    )

    # Thin red laser line across the middle of the window.
    laser_center = _head_local_to_world((0.0362, 0.0, 0.0))
    body.visual(
        Box((0.002, 0.034, 0.005)),
        origin=Origin(xyz=laser_center, rpy=(0.0, HEAD_TILT, 0.0)),
        material=LASER_RED,
        name="laser_line",
    )

    # White spec label on the underside of the head (as in the photo).
    label_center = _head_local_to_world((-0.018, 0.0, -0.0245))
    body.visual(
        Box((0.034, 0.028, 0.002)),
        origin=Origin(xyz=label_center, rpy=(0.0, HEAD_TILT, 0.0)),
        material=LABEL_WHITE,
        name="spec_label",
    )

    # Coiled cable exiting the base of the handle.
    coil_mesh = tube_from_spline_points(
        _cable_points(),
        radius=0.0035,
        samples_per_segment=6,
        radial_segments=14,
        cap_ends=True,
    )
    body.visual(
        mesh_from_geometry(coil_mesh, "cable_coil"),
        origin=Origin(),
        material=CABLE_BLACK,
        name="cable_coil",
    )

    # ------------------------------------------------------------ trigger
    trigger = model.part("trigger")
    trigger.visual(
        mesh_from_cadquery(_trigger_solid(), "trigger_pad"),
        origin=Origin(),
        material=TRIGGER_BLUE,
        name="trigger_pad",
    )

    model.articulation(
        "grip_to_trigger",
        ArticulationType.REVOLUTE,
        parent=body,
        child=trigger,
        origin=Origin(xyz=TRIGGER_PIVOT),
        # Pad hangs below the pivot, so positive rotation about +Y swings the
        # pad tip backward (-X): a squeeze toward the grip.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=4.0, lower=0.0, upper=TRIGGER_SQUEEZE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("scanner_body")
    trigger = object_model.get_part("trigger")
    joint = object_model.get_articulation("grip_to_trigger")

    # The trigger stem/pad intentionally seats into the grip shell.
    ctx.allow_overlap(
        trigger,
        body,
        reason="Trigger stem passes through a slot in the grip front; the pad "
        "seats against the grip shell like a real trigger.",
    )

    # -------- identity / layout checks (rest pose)
    with ctx.pose({joint: 0.0}):
        head_aabb = ctx.part_element_world_aabb(body, elem="scan_head")
        grip_aabb = ctx.part_element_world_aabb(body, elem="grip_shell")
        window_aabb = ctx.part_element_world_aabb(body, elem="scan_window")
        coil_aabb = ctx.part_element_world_aabb(body, elem="cable_coil")
        trig_rest = ctx.part_world_aabb(trigger)

        # Scan head sits on top of the grip and is wider than it.
        ctx.check(
            "head_tops_the_grip",
            head_aabb[1][2] > grip_aabb[1][2] + 0.02,
            f"head z_max {head_aabb[1][2]:.3f} vs grip z_max {grip_aabb[1][2]:.3f}",
        )
        head_w = head_aabb[1][1] - head_aabb[0][1]
        grip_w = grip_aabb[1][1] - grip_aabb[0][1]
        ctx.check(
            "head_wider_than_grip",
            head_w > grip_w + 0.01,
            f"head width {head_w:.3f} vs grip width {grip_w:.3f}",
        )

        # Scan window is recessed behind the front rim of the head, and the
        # red laser line sits inside the window opening.
        ctx.check(
            "window_recessed_in_head",
            window_aabb[1][0] < head_aabb[1][0] - 0.005,
            f"window x_max {window_aabb[1][0]:.3f} vs head x_max {head_aabb[1][0]:.3f}",
        )
        ctx.expect_within(
            body,
            body,
            axes="yz",
            inner_elem="laser_line",
            outer_elem="scan_window",
            name="laser_line_inside_window",
        )

        # Coiled cable sits low, behind the grip base.
        ctx.check(
            "coil_below_grip_top",
            coil_aabb[1][2] < 0.06,
            f"coil z_max {coil_aabb[1][2]:.3f}",
        )
        ctx.check(
            "coil_behind_grip",
            coil_aabb[0][0] < grip_aabb[0][0],
            f"coil x_min {coil_aabb[0][0]:.3f} vs grip x_min {grip_aabb[0][0]:.3f}",
        )

        # Trigger pad protrudes forward of the grip front at rest.
        ctx.check(
            "trigger_proud_of_grip",
            trig_rest[1][0] > grip_aabb[1][0] + 0.002,
            f"trigger x_max {trig_rest[1][0]:.3f} vs grip x_max {grip_aabb[1][0]:.3f}",
        )
        # Trigger stays seated against the grip (contact, given the allowance).
        ctx.expect_contact(trigger, body, name="trigger_seated_on_grip")

    # -------- articulation: squeezing the trigger pulls the pad backward
    with ctx.pose({joint: TRIGGER_SQUEEZE}):
        trig_squeezed = ctx.part_world_aabb(trigger)

    rest_cx = 0.5 * (trig_rest[0][0] + trig_rest[1][0])
    sq_cx = 0.5 * (trig_squeezed[0][0] + trig_squeezed[1][0])
    ctx.check(
        "trigger_squeezes_backward",
        rest_cx - sq_cx > 0.003,
        f"trigger AABB center x moved {sq_cx - rest_cx:+.4f} m (want <= -0.003, backward)",
    )
    # The squeeze is a rotation about the pivot: the top (hinge end) of the
    # trigger should barely move while the tip swings.
    ctx.check(
        "trigger_pivot_end_stays",
        abs(trig_squeezed[1][2] - trig_rest[1][2]) < 0.004,
        f"trigger z_max moved {trig_squeezed[1][2] - trig_rest[1][2]:+.4f} m",
    )

    return ctx.report()


object_model = build_object_model()
