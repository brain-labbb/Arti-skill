"""Inline wand/CCD barcode scanner (wand-skeleton variant).

Single straight-barrel inline scanner body held like a thick pen or marker,
with a recessed scan window on the front nose emitting a red laser line, a
small side thumb trigger on the barrel, a white spec label on the barrel
underside, and a coiled black cable exiting the rear end cap.

Structural delta vs. parent: the separate scan_head + raked grip are
collapsed into one inline barrel.  Scan window and laser line move onto
the front nose face; the trigger becomes a small side thumb button; the
cable exits the rear end cap.

The trigger is the primary articulation: a REVOLUTE press inward from
the barrel side.
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
# World frame: +X = forward (scan direction), +Z = up.
# Barrel centered at origin; front nose at +X, rear end at -X.

BARREL_L = 0.180        # barrel length along X  (180 mm)
BARREL_W = 0.036        # barrel width  along Y  ( 36 mm)
BARREL_H = 0.032        # barrel height along Z  ( 32 mm)
BARREL_FILLET = 0.008   # long-edge corner fillet

RECESS_W = 0.026        # scan-window pocket width  (Y)
RECESS_H = 0.016        # scan-window pocket height (Z)
RECESS_DEPTH = 0.005    # pocket depth into front face

TRIGGER_PIVOT = (0.015, BARREL_W / 2, 0.002)   # on the +Y face
TRIGGER_PRESS = 0.22    # rad of inward press travel

# ---- materials
BLACK_PLASTIC = Material("black_plastic", rgba=(0.08, 0.08, 0.09, 1.0))
DARK_GLASS = Material("dark_scan_glass", rgba=(0.03, 0.03, 0.04, 1.0))
LASER_RED = Material("laser_red", rgba=(1.0, 0.08, 0.05, 1.0))
TRIGGER_BLUE = Material("trigger_blue", rgba=(0.13, 0.20, 0.75, 1.0))
LABEL_WHITE = Material("label_white", rgba=(0.92, 0.92, 0.90, 1.0))
CABLE_BLACK = Material("cable_black", rgba=(0.05, 0.05, 0.06, 1.0))
CAP_GREY = Material("end_cap_grey", rgba=(0.12, 0.12, 0.13, 1.0))


# -------------------------------------------------------------- geometry
def _barrel_solid() -> cq.Workplane:
    """Single inline barrel: rounded-rect cross section, front scan-window pocket."""
    barrel = (
        cq.Workplane("XY")
        .box(BARREL_L, BARREL_W, BARREL_H)
        .edges("|Z")
        .fillet(BARREL_FILLET)
    )
    # Cut scan-window pocket into the front (+X) face.
    # Cutter straddles the front face so the boolean leaves a clean recess.
    cutter = (
        cq.Workplane("XY")
        .box(RECESS_DEPTH * 2, RECESS_W, RECESS_H)
        .translate((BARREL_L / 2, 0.0, 0.0))
    )
    return barrel.cut(cutter)


def _trigger_solid() -> cq.Workplane:
    """Side thumb trigger, pivot at origin.

    The pad hangs below the pivot (-Z) and protrudes outward (+Y);
    a short stem reaches inward (-Y) through the barrel wall.
    """
    pad = (
        cq.Workplane("XY")
        .box(0.016, 0.006, 0.022)
        .translate((0.0, 0.004, -0.011))
    )
    stem = (
        cq.Workplane("XY")
        .box(0.012, 0.010, 0.010)
        .translate((0.0, -0.003, -0.005))
    )
    return pad.union(stem)


def _cable_points() -> list[tuple[float, float, float]]:
    """Lead-out from the barrel rear into a hanging coil behind the scanner."""
    rear_x = -BARREL_L / 2
    pts: list[tuple[float, float, float]] = [
        (rear_x, 0.000, 0.000),
        (rear_x - 0.012, 0.002, -0.008),
        (rear_x - 0.025, 0.005, -0.018),
    ]
    coil_cx = rear_x - 0.042
    coil_r = 0.016
    turns = 3.0
    top_z = -0.012
    bottom_z = -0.042
    n = 60
    for i in range(n + 1):
        t = i / n
        a = 0.55 + 2.0 * math.pi * turns * t
        pts.append(
            (
                coil_cx + coil_r * math.cos(a),
                coil_r * math.sin(a),
                top_z + (bottom_z - top_z) * t,
            )
        )
    return pts


# -------------------------------------------------------------- model
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="handheld_barcode_scanner")

    # --- scanner_body: single inline barrel ---
    body = model.part("scanner_body")

    body.visual(
        mesh_from_cadquery(_barrel_solid(), "barrel_body"),
        origin=Origin(),
        material=BLACK_PLASTIC,
        name="barrel_shell",
    )

    # Rear end cap (slightly inset, lighter grey to read as a separate moulding)
    body.visual(
        Box((0.004, BARREL_W - 0.008, BARREL_H - 0.008)),
        origin=Origin(xyz=(-BARREL_L / 2 + 0.002, 0.0, 0.0)),
        material=CAP_GREY,
        name="rear_cap",
    )

    # Scan window: dark glass plate seated in the front recess
    window_x = BARREL_L / 2 - RECESS_DEPTH + 0.001
    body.visual(
        Box((0.003, RECESS_W - 0.004, RECESS_H - 0.004)),
        origin=Origin(xyz=(window_x, 0.0, 0.0)),
        material=DARK_GLASS,
        name="scan_window",
    )

    # Laser line: thin red stripe across the scan window
    body.visual(
        Box((0.002, RECESS_W - 0.008, 0.004)),
        origin=Origin(xyz=(window_x + 0.002, 0.0, 0.0)),
        material=LASER_RED,
        name="laser_line",
    )

    # Spec label: white rectangle on the barrel underside
    body.visual(
        Box((0.050, 0.022, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, -BARREL_H / 2)),
        material=LABEL_WHITE,
        name="spec_label",
    )

    # Coiled cable exiting the rear end cap
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

    # --- trigger: side thumb trigger ---
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
        # axis = -X: positive q swings the pad inward (-Y) toward the barrel.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=4.0, lower=0.0, upper=TRIGGER_PRESS
        ),
    )

    return model


# -------------------------------------------------------------- tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("scanner_body")
    trigger = object_model.get_part("trigger")
    joint = object_model.get_articulation("grip_to_trigger")

    # Trigger stem passes through the barrel side wall; pad seats against the
    # barrel surface.  Scoped intentional overlap.
    ctx.allow_overlap(
        trigger,
        body,
        reason="Trigger stem passes through a slot in the barrel side wall; "
        "the pad seats against the barrel shell like a real thumb trigger.",
    )

    # -------- identity / layout checks (rest pose) --------
    with ctx.pose({joint: 0.0}):
        barrel_aabb = ctx.part_element_world_aabb(body, elem="barrel_shell")
        window_aabb = ctx.part_element_world_aabb(body, elem="scan_window")
        coil_aabb = ctx.part_element_world_aabb(body, elem="cable_coil")
        label_aabb = ctx.part_element_world_aabb(body, elem="spec_label")
        trig_rest = ctx.part_world_aabb(trigger)

        # Structural delta: one inline barrel (not pistol-grip head-over-grip).
        # The barrel must be much longer along the scan axis (X) than tall (Z).
        barrel_dx = barrel_aabb[1][0] - barrel_aabb[0][0]
        barrel_dz = barrel_aabb[1][2] - barrel_aabb[0][2]
        ctx.check(
            "inline_barrel_skeleton",
            barrel_dx > barrel_dz * 3,
            f"barrel dx={barrel_dx:.3f} vs dz={barrel_dz:.3f} (want dx > 3*dz)",
        )

        # Scan window is at the front nose of the barrel.
        ctx.check(
            "window_at_front_nose",
            window_aabb[1][0] > barrel_aabb[1][0] - 0.015,
            f"window x_max {window_aabb[1][0]:.3f} vs barrel x_max {barrel_aabb[1][0]:.3f}",
        )

        # Laser line is within the scan window footprint (YZ).
        ctx.expect_within(
            body,
            body,
            axes="yz",
            inner_elem="laser_line",
            outer_elem="scan_window",
            name="laser_inside_window",
        )

        # Spec label on the barrel underside.
        barrel_cz = 0.5 * (barrel_aabb[0][2] + barrel_aabb[1][2])
        label_cz = 0.5 * (label_aabb[0][2] + label_aabb[1][2])
        ctx.check(
            "label_on_underside",
            label_cz < barrel_cz - 0.005,
            f"label z_center {label_cz:.3f} vs barrel z_center {barrel_cz:.3f}",
        )

        # Cable coil behind the barrel.
        ctx.check(
            "cable_behind_barrel",
            coil_aabb[0][0] < barrel_aabb[0][0] - 0.005,
            f"coil x_min {coil_aabb[0][0]:.3f} vs barrel x_min {barrel_aabb[0][0]:.3f}",
        )

        # Trigger on the +Y side, proud of the barrel surface.
        ctx.check(
            "trigger_side_proud",
            trig_rest[1][1] > barrel_aabb[1][1] - 0.002,
            f"trigger y_max {trig_rest[1][1]:.3f} vs barrel y_max {barrel_aabb[1][1]:.3f}",
        )
        ctx.expect_contact(trigger, body, name="trigger_seated_on_barrel")

    # -------- articulation: pressing the trigger moves it inward --------
    with ctx.pose({joint: TRIGGER_PRESS}):
        trig_pressed = ctx.part_world_aabb(trigger)

    # The trigger Y-center shifts inward (toward the barrel) when pressed.
    rest_cy = 0.5 * (trig_rest[0][1] + trig_rest[1][1])
    pressed_cy = 0.5 * (trig_pressed[0][1] + trig_pressed[1][1])
    ctx.check(
        "trigger_presses_inward",
        rest_cy - pressed_cy > 0.0005,
        f"trigger Y center shifted {pressed_cy - rest_cy:+.4f} m "
        f"(want <= -0.0005, inward toward barrel)",
    )

    return ctx.report()


object_model = build_object_model()
