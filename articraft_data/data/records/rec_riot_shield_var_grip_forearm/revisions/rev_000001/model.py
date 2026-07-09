from __future__ import annotations

"""Folding two-panel soft-armor riot shield (G-FOLD style).

Faithful to the reference image: a matte gray ballistic-fabric front panel with
dark charcoal edge binding, a black rectangular logo patch near the top, a cast
aluminum carry-handle bracket bolted on with four dome-head carriage bolts
(open rectangular grip window), a black webbing pull strap along the bottom
edge, grommet eyelets near the bottom binding, and a second dark fabric panel
joined at the top by a full-width fabric fold hinge. At rest the shield sits
partially folded (A-frame tent, as photographed); the fold articulation unfolds
it flat into a full-height deployed shield or folds it closed.
"""

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# --- Overall dimensions (meters) -------------------------------------------
PANEL_W = 0.50  # panel width (Y)
PANEL_H = 0.55  # single panel height (Z)
PANEL_T = 0.030  # soft-armor panel thickness (X)
TRIM_W = 0.024  # edge binding tape width
TRIM_T = 0.037  # binding wraps slightly proud of both faces

HINGE_GAP = 0.040  # panel slabs stand off the fold axis; the fabric roll spans it
HINGE_Z = PANEL_H + HINGE_GAP  # fold line sits above the front panel top edge
SLEEVE_R = 0.042  # fat fabric fold roll wrapping the hinge line
BIND_H = 0.52  # binding tapes stop short of the fold line

# Rest pose: rear panel folded back by FOLD0 from straight-down (A-frame tent).
FOLD0 = 0.85
FOLD_LOWER = -0.62  # fold nearly closed (panels almost face to face)
FOLD_UPPER = math.pi - FOLD0  # unfold flat: rear panel extends straight up

# Handle bracket
PLATE_W = 0.21  # Y
PLATE_H = 0.15  # Z
PLATE_T = 0.016  # X
HANDLE_Z = 0.27  # bracket center height on the front panel

# Forearm cradle cuff (upper plate region): open half-cylinder C-channel
CRADLE_R = 0.040  # outer radius
CRADLE_T = 0.006  # wall thickness
CRADLE_L = 0.15   # length along Y
CRADLE_Z = 0.035  # center height on plate (local Z)

# Grip bar on standoffs (lower plate region)
GRIP_BAR_R = 0.013
GRIP_BAR_L = 0.14
STANDOFF_R = 0.009
STANDOFF_H = 0.032
GRIP_Z = -0.030   # center height on plate (local Z)

HANDLE_EMBED = 0.002  # small overlap into plate for mesh connectivity

BOLT_R = 0.010
BOLT_Y = 0.088
BOLT_Z = 0.060


def _rounded_plate() -> cq.Workplane:
    """Cast aluminum mounting plate with rounded corners (handle part frame)."""
    plate = cq.Workplane("YZ").rect(PLATE_W, PLATE_H).extrude(PLATE_T)
    try:
        plate = plate.edges("|X").fillet(0.018)
    except ValueError:
        pass
    return plate


def _forearm_cradle() -> cq.Workplane:
    """Open half-cylinder forearm cradle cuff (C-channel, opening outward).

    Built as the -X half of a thin-walled tube with axis along Y.
    The flat cut face sits flush against the mounting plate; the curved
    back stands proud so the user's forearm clips in from outside.
    """
    R = CRADLE_R
    t = CRADLE_T
    L = CRADLE_L

    # Full tube along Y, centered at origin (XZ extrudes in -Y)
    outer = cq.Workplane("XZ", origin=(0.0, L / 2.0, 0.0)).circle(R).extrude(L)
    inner = (
        cq.Workplane("XZ", origin=(0.0, L / 2.0 + 0.001, 0.0))
        .circle(R - t)
        .extrude(L + 0.002)
    )
    tube = outer.cut(inner)

    # Remove the +X half → C-channel with opening facing +X (outward)
    cutter = (
        cq.Workplane("XY")
        .box(R + 0.02, L + 0.04, 2.0 * R + 0.04)
        .translate(((R + 0.02) / 2.0 - 0.001, 0.0, 0.0))
    )
    half = tube.cut(cutter)
    try:
        half = half.edges("|X").fillet(0.002)
    except Exception:
        pass
    return half


def _grip_bar_assembly() -> cq.Workplane:
    """Cylindrical grip bar on two rectangular bracket standoffs.

    Each bracket is a flat plate extending from the mounting face outward
    to overlap with the bar cylinder, ensuring a single connected mesh.
    """
    bar_r = GRIP_BAR_R
    bar_l = GRIP_BAR_L
    s_h = STANDOFF_H
    s_y = 0.050  # bracket Y-offset from center
    bracket_y_w = 0.012  # bracket width in Y
    bracket_z_h = 0.022  # bracket height in Z

    # Horizontal bar along Y, centered at x = s_h (XZ extrudes in -Y)
    bar = (
        cq.Workplane("XZ", origin=(s_h, bar_l / 2.0, 0.0))
        .circle(bar_r)
        .extrude(bar_l)
    )

    # Two flat bracket standoffs extending into the bar for overlap
    for dy in (-s_y, s_y):
        bracket = (
            cq.Workplane("YZ", origin=(0.0, dy, 0.0))
            .rect(bracket_y_w, bracket_z_h)
            .extrude(s_h + bar_r * 0.5)
        )
        bar = bar.union(bracket)

    return bar


def _add_binding(part, height: float, z_center: float, end_z: float) -> None:
    """Bindings run the panel sides but stop short of the fold line."""
    """Charcoal binding tape along both side edges plus the free end edge."""
    for i in range(2):
        sign = -1.0 if i == 0 else 1.0
        part.visual(
            Box((TRIM_T, TRIM_W, height)),
            origin=Origin(xyz=(0.0, sign * (PANEL_W / 2.0 - TRIM_W / 2.0), z_center)),
            material="charcoal_binding",
            name=f"side_binding_{i}",
        )
    part.visual(
        Box((TRIM_T, PANEL_W, TRIM_W)),
        origin=Origin(xyz=(0.0, 0.0, end_z)),
        material="charcoal_binding",
        name="end_binding",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_riot_shield")

    model.material("gray_ballistic_fabric", rgba=(0.60, 0.61, 0.62, 1.0))
    model.material("dark_ballistic_fabric", rgba=(0.11, 0.11, 0.12, 1.0))
    model.material("charcoal_binding", rgba=(0.17, 0.17, 0.18, 1.0))
    model.material("patch_black", rgba=(0.04, 0.04, 0.045, 1.0))
    model.material("patch_white", rgba=(0.93, 0.93, 0.92, 1.0))
    model.material("cast_aluminum", rgba=(0.63, 0.64, 0.66, 1.0))
    model.material("bolt_steel", rgba=(0.24, 0.24, 0.26, 1.0))
    model.material("webbing_black", rgba=(0.07, 0.07, 0.08, 1.0))
    model.material("grommet_metal", rgba=(0.45, 0.45, 0.47, 1.0))
    model.material("grip_rubber", rgba=(0.12, 0.12, 0.13, 1.0))

    # --- Front panel (root): gray face with logo patch, strap, grommets ----
    front = model.part("front_panel")
    front.visual(
        Box((PANEL_T, PANEL_W, PANEL_H)),
        origin=Origin(xyz=(0.0, 0.0, PANEL_H / 2.0)),
        material="gray_ballistic_fabric",
        name="front_slab",
    )
    _add_binding(front, BIND_H, BIND_H / 2.0, TRIM_W / 2.0)

    # Fabric fold sleeve wrapping the hinge line at the top edge.
    front.visual(
        Cylinder(radius=SLEEVE_R, length=PANEL_W),
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="charcoal_binding",
        name="hinge_sleeve",
    )

    # Black rectangular logo patch with a white lettering bar near the top.
    front.visual(
        Box((0.006, 0.20, 0.11)),
        origin=Origin(xyz=(PANEL_T / 2.0 + 0.002, 0.0, 0.435)),
        material="patch_black",
        name="logo_patch",
    )
    front.visual(
        Box((0.003, 0.15, 0.034)),
        origin=Origin(xyz=(PANEL_T / 2.0 + 0.0055, 0.0, 0.435)),
        material="patch_white",
        name="logo_lettering_bar",
    )

    # Black webbing pull strap along the bottom front edge.
    front.visual(
        Box((0.012, 0.16, 0.022)),
        origin=Origin(xyz=(PANEL_T / 2.0 + 0.004, 0.0, 0.030)),
        material="webbing_black",
        name="bottom_pull_strap",
    )

    # Grommet eyelets punched through the bottom binding.
    for i in range(3):
        front.visual(
            Cylinder(radius=0.008, length=PANEL_T + 0.010),
            origin=Origin(
                xyz=(0.0, -0.15 + 0.15 * i, TRIM_W / 2.0 + 0.010),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material="grommet_metal",
            name=f"grommet_{i}",
        )

    # --- Carry-handle bracket (bolted rigid mount on the front face) -------
    handle = model.part("carry_handle")
    handle.visual(
        mesh_from_cadquery(_rounded_plate(), "handle_plate", tolerance=0.0012),
        material="cast_aluminum",
        name="mount_plate",
    )
    # Forearm cradle cuff: open C-channel higher on the plate
    handle.visual(
        mesh_from_cadquery(_forearm_cradle(), "handle_forearm_cradle", tolerance=0.0012),
        origin=Origin(xyz=(PLATE_T + CRADLE_R - HANDLE_EMBED, 0.0, CRADLE_Z)),
        material="cast_aluminum",
        name="forearm_cradle",
    )

    # Grip bar on standoffs: lower on the plate
    handle.visual(
        mesh_from_cadquery(_grip_bar_assembly(), "handle_grip_bar", tolerance=0.0012),
        origin=Origin(xyz=(PLATE_T - HANDLE_EMBED, 0.0, GRIP_Z)),
        material="grip_rubber",
        name="grip_bar",
    )
    # Four dome-head carriage bolts at the plate corners.
    corners = ((-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0))
    for i, (sy, sz) in enumerate(corners):
        y = sy * BOLT_Y
        z = sz * BOLT_Z
        handle.visual(
            Cylinder(radius=BOLT_R, length=0.016),
            origin=Origin(xyz=(PLATE_T + 0.002, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="bolt_steel",
            name=f"bolt_shank_{i}",
        )
        handle.visual(
            Sphere(radius=BOLT_R),
            origin=Origin(xyz=(PLATE_T + 0.010, y, z)),
            material="bolt_steel",
            name=f"bolt_dome_{i}",
        )

    model.articulation(
        "panel_to_handle",
        ArticulationType.FIXED,
        parent=front,
        child=handle,
        origin=Origin(xyz=(PANEL_T / 2.0, 0.0, HANDLE_Z)),
    )

    # --- Rear panel joined by the top fold hinge ----------------------------
    rear = model.part("rear_panel")
    rear.visual(
        Box((PANEL_T, PANEL_W, PANEL_H)),
        origin=Origin(xyz=(0.0, 0.0, -(HINGE_GAP + PANEL_H / 2.0))),
        material="dark_ballistic_fabric",
        name="rear_slab",
    )
    _add_binding(
        rear,
        BIND_H,
        -(HINGE_GAP + PANEL_H - BIND_H / 2.0),
        -(HINGE_GAP + PANEL_H - TRIM_W / 2.0),
    )

    # Fold hinge: at q=0 the rear panel leans down-back in the photographed
    # A-frame tent pose. Positive q unfolds the shield toward flat/deployed.
    model.articulation(
        "panel_fold",
        ArticulationType.REVOLUTE,
        parent=front,
        child=rear,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z), rpy=(0.0, FOLD0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.0, lower=FOLD_LOWER, upper=FOLD_UPPER),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_panel")
    rear = object_model.get_part("rear_panel")
    handle = object_model.get_part("carry_handle")
    fold = object_model.get_articulation("panel_fold")

    # The fabric fold sleeve wraps the hinge line and hugs the rear panel edge.
    ctx.allow_overlap(
        "front_panel",
        "rear_panel",
        elem_a="hinge_sleeve",
        elem_b="rear_slab",
        reason="fabric fold sleeve wraps the hinge line and hugs the rear panel top edge",
    )
    for i in range(2):
        ctx.allow_overlap(
            "front_panel",
            "rear_panel",
            elem_a="hinge_sleeve",
            elem_b=f"side_binding_{i}",
            reason="fold sleeve fabric wraps over the rear panel side binding at the hinge line",
        )

    # Prompt-specific presence checks.
    ctx.check(
        "logo_patch_present",
        front.get_visual("logo_patch") is not None,
        "front panel must carry the black logo patch",
    )
    # Structural delta: carry_handle has forearm_cradle + grip_bar, not the old grip_frame
    _old_grip_frame_gone = True
    try:
        handle.get_visual("grip_frame")
        _old_grip_frame_gone = False
    except Exception:
        pass
    ctx.check(
        "carry_handle_forearm_cradle_and_grip_bar",
        handle.get_visual("forearm_cradle") is not None
        and handle.get_visual("grip_bar") is not None
        and _old_grip_frame_gone,
        "carry_handle must have forearm cradle cuff + grip bar on standoffs",
    )
    # Forearm cradle is above the grip bar on the plate
    ctx.expect_gap(
        handle,
        handle,
        axis="z",
        positive_elem="forearm_cradle",
        negative_elem="grip_bar",
        min_gap=0.005,
        name="forearm_cradle_above_grip_bar",
    )
    ctx.check(
        "four_bolts_present",
        all(handle.get_visual(f"bolt_dome_{i}") is not None for i in range(4)),
        "handle bracket must be bolted with four dome-head bolts",
    )

    # Handle bracket is mounted flush on the front panel face and stays inside
    # the panel footprint.
    ctx.expect_contact(handle, front, elem_a="mount_plate", elem_b="front_slab")
    ctx.expect_within(handle, front, axes="yz", inner_elem="mount_plate", outer_elem="front_slab")

    # Fold hinge continuity: the sleeve stays engaged with the rear panel edge.
    ctx.expect_contact(front, rear, elem_a="hinge_sleeve", elem_b="rear_slab")

    # Usable fold travel spans nearly-closed -> flat deployment (over 2.5 rad).
    limits = fold.motion_limits
    ctx.check(
        "fold_travel_range",
        limits is not None and (limits.upper - limits.lower) > 2.5,
        "fold hinge must travel from nearly closed to fully deployed",
    )

    # Deployed pose: rear panel unfolds flat above the front panel (coplanar).
    with ctx.pose({fold: FOLD_UPPER}):
        ctx.expect_overlap(
            rear,
            front,
            axes="xy",
            min_overlap=0.02,
            elem_a="rear_slab",
            elem_b="front_slab",
            name="deployed_panels_coplanar",
        )

    return ctx.report()


object_model = build_object_model()
