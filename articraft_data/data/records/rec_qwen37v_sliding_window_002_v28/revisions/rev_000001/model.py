from __future__ import annotations

# Corner-lift sliding window variant: white vinyl frame with one fixed sash,
# one sliding sash (with corner-lift vent panel), an independent insect screen
# on a shallow prismatic track, roller blocks under the sliding sash, and a
# sill lip with drainage slots.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     - width  -> X
#     - height -> Z   (sill near z=0)
#     - frame depth / glazing thickness / slide-normal -> Y
#   Glass plane is the X-Z plane. q=0 reads SHUT for slide and screen.
#   Positive slide q moves the right sash toward fixed left sash (-X) to open.
#   Positive screen q moves the screen in -X independently.
#   Positive vent q tilts the vent panel inward (top edge hinges, bottom lifts).

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Absolute dimensions (meters)
# ---------------------------------------------------------------------------

TOTAL_W = 1.52            # overall window width along X
TOTAL_H = 1.72            # overall height along Z (sill at z=0, head at z=TOTAL_H)

FRAME_FACE = 0.085        # outer frame member face width (chunky vinyl)
FRAME_DEPTH = 0.140       # deep box section along Y (thick patio-slider profile)

MEETING_OVERLAP = 0.040   # the two sash stiles overlap by this much at center

SASH_FACE = 0.075         # sash perimeter rail/stile face width
SASH_DEPTH = 0.060        # sash depth along Y
GLASS_T = 0.008           # glazing thickness along Y

# Y layout: frame box centered on y=0. Fixed sash in the rear glazing plane;
# sliding sash sits proud toward +Y so it passes in front of the fixed sash.
FIXED_SASH_Y = -0.028     # rear glazing plane center (Y)
SLIDE_SASH_Y = 0.044      # sliding sash proud toward +Y (front track)

REBATE = 0.005            # glass tucks under the sash lip by this much

# Latch (cam lock) hardware
LATCH_PLATE_W = 0.028
LATCH_PLATE_H = 0.075
LATCH_PLATE_T = 0.010
LATCH_LEVER_LEN = 0.045
LATCH_LEVER_R = 0.006

# Sill lip and drainage slots
SILL_LIP_H = 0.018        # sill lip height (vertical protrusion)
SILL_LIP_DEPTH = 0.025    # sill lip depth along Y (exterior side, toward +Y)
DRAIN_SLOT_W = 0.040      # drainage slot width
DRAIN_SLOT_H = 0.010      # drainage slot height (through the sill lip)
DRAIN_SLOT_DEPTH = 0.030  # drainage slot depth along Y (cuts through lip)
NUM_DRAIN_SLOTS = 4       # number of drainage slots across the sill

# Roller blocks under sliding sash
ROLLER_W = 0.030          # roller block width (X)
ROLLER_H = 0.012          # roller block height (Z)
ROLLER_D = 0.025          # roller block depth (Y)
ROLLER_INSET_X = 0.10     # how far from sash center the rollers sit

# Vent panel dimensions (corner-lift vent at top of sliding sash)
VENT_W = 0.30             # vent panel width
VENT_H = 0.20             # vent panel height
VENT_DEPTH = 0.020        # vent panel frame depth
VENT_FRAME_W = 0.025      # vent frame member width
VENT_GLASS_T = 0.006      # vent glass thickness

# Insect screen dimensions
SCREEN_W = 1.30           # screen width (nearly full inner width)
SCREEN_H = 1.59           # screen height (extends into head/sill tracks)
SCREEN_FRAME_W = 0.025    # screen frame member width
SCREEN_FRAME_DEPTH = 0.015  # screen frame depth (thin)
SCREEN_MESH_T = 0.002     # screen mesh thickness
SCREEN_Y = -0.064         # screen sits behind (more negative Y) the fixed sash

METAL_RGBA = (0.74, 0.76, 0.79, 1.0)
SCREEN_MESH_RGBA = (0.30, 0.32, 0.30, 0.60)  # dark grey semi-transparent mesh
ROLLER_RGBA = (0.25, 0.25, 0.27, 1.0)  # dark nylon roller

# ---------------------------------------------------------------------------
# Derived layout
# ---------------------------------------------------------------------------

HALF_W = TOTAL_W / 2.0
INNER_X0 = -HALF_W + FRAME_FACE
INNER_X1 = HALF_W - FRAME_FACE
INNER_Z0 = FRAME_FACE
INNER_Z1 = TOTAL_H - FRAME_FACE
INNER_W = INNER_X1 - INNER_X0
INNER_H = INNER_Z1 - INNER_Z0

SASH_OPENING_W = (INNER_W + MEETING_OVERLAP) / 2.0
SASH_OPENING_H = INNER_H

FIXED_OPEN_CX = INNER_X0 + SASH_OPENING_W / 2.0
SLIDE_OPEN_CX = INNER_X1 - SASH_OPENING_W / 2.0
MID_CZ = (INNER_Z0 + INNER_Z1) / 2.0

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)
GLASS_RGBA = (0.52, 0.60, 0.66, 0.30)


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery), authored directly in meters.
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float, y_center: float, depth: float) -> cq.Workplane:
    """Axis-aligned box spanning [x0,x1] x [z0,z1] in the X-Z plane, centered on
    y_center with the given Y depth."""
    return (
        cq.Workplane("XY")
        .transformed(offset=((x0 + x1) / 2.0, y_center, (z0 + z1) / 2.0))
        .box(x1 - x0, depth, z1 - z0)
    )


def _build_frame_shape() -> cq.Workplane:
    """Static outer frame: thick slab with two sash openings, hollow perimeter,
    plus a sill lip on the exterior (+Y side) with drainage slots cut through."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    cut_depth = FRAME_DEPTH + 0.02
    # Single clear opening spanning the inner region
    opening = _slab(INNER_X0, INNER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    frame = outer.cut(opening)

    # Sill lip: a horizontal ledge protruding from the sill bottom on the
    # exterior (+Y) side, running the full width.
    sill_lip_y_center = FRAME_DEPTH / 2.0 + SILL_LIP_DEPTH / 2.0
    sill_lip = _slab(
        -HALF_W, HALF_W,
        0.0, SILL_LIP_H,
        sill_lip_y_center,
        SILL_LIP_DEPTH,
    )
    frame = frame.union(sill_lip)

    # Drainage slots: small rectangular cuts through the sill lip, evenly spaced.
    slot_spacing = INNER_W / (NUM_DRAIN_SLOTS + 1)
    for i in range(NUM_DRAIN_SLOTS):
        slot_cx = INNER_X0 + slot_spacing * (i + 1)
        slot_cutter = (
            cq.Workplane("XY")
            .transformed(offset=(slot_cx, sill_lip_y_center, SILL_LIP_H / 2.0))
            .box(DRAIN_SLOT_W, DRAIN_SLOT_DEPTH + 0.01, DRAIN_SLOT_H)
        )
        frame = frame.cut(slot_cutter)

    return frame


def _build_sash_shape() -> cq.Workplane:
    """One sash ring in its OWN local frame, centered on local origin."""
    ow = SASH_OPENING_W
    oh = SASH_OPENING_H
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SASH_DEPTH)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH + 0.02)
    return outer.cut(opening)


def _build_sash_glass_shape() -> cq.Workplane:
    """Single clear pane filling the sash opening (sash-local frame)."""
    ow = SASH_OPENING_W + 2 * REBATE
    oh = SASH_OPENING_H + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_vent_frame_shape() -> cq.Workplane:
    """Vent panel frame ring in its own local frame. Origin at the top-center
    of the vent (the hinge edge). The vent extends downward (-Z) and is
    centered on X."""
    out_w = VENT_W
    out_h = VENT_H
    # Outer slab: from z=-out_h to z=0 (hinge at top)
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h, 0.0, 0.0, VENT_DEPTH)
    # Inner opening
    in_w = out_w - 2 * VENT_FRAME_W
    in_h = out_h - 2 * VENT_FRAME_W
    opening = _slab(-in_w / 2.0, in_w / 2.0, -out_h + VENT_FRAME_W, -VENT_FRAME_W, 0.0, VENT_DEPTH + 0.01)
    return outer.cut(opening)


def _build_vent_glass_shape() -> cq.Workplane:
    """Vent glass pane in vent-local frame (origin at hinge top-center)."""
    in_w = VENT_W - 2 * VENT_FRAME_W + 2 * REBATE
    in_h = VENT_H - 2 * VENT_FRAME_W + 2 * REBATE
    return _slab(-in_w / 2.0, in_w / 2.0, -VENT_H + VENT_FRAME_W, -VENT_FRAME_W, 0.0, VENT_GLASS_T)


def _build_screen_frame_shape() -> cq.Workplane:
    """Insect screen frame in its own local frame, centered on origin.
    A thin rectangular frame ring."""
    out_w = SCREEN_W
    out_h = SCREEN_H
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SCREEN_FRAME_DEPTH)
    in_w = out_w - 2 * SCREEN_FRAME_W
    in_h = out_h - 2 * SCREEN_FRAME_W
    opening = _slab(-in_w / 2.0, in_w / 2.0, -in_h / 2.0, in_h / 2.0, 0.0, SCREEN_FRAME_DEPTH + 0.01)
    return outer.cut(opening)


def _build_screen_mesh_shape() -> cq.Workplane:
    """Thin screen mesh panel filling the screen frame opening."""
    in_w = SCREEN_W - 2 * SCREEN_FRAME_W
    in_h = SCREEN_H - 2 * SCREEN_FRAME_W
    return _slab(-in_w / 2.0, in_w / 2.0, -in_h / 2.0, in_h / 2.0, 0.0, SCREEN_MESH_T)


# ---------------------------------------------------------------------------
# Part builders
# ---------------------------------------------------------------------------

def _add_sash(model: ArticulatedObject, name: str) -> None:
    """Add a sash part (vinyl ring + clear glass) in its own local frame."""
    sash = model.part(name)
    sash.visual(
        mesh_from_cadquery(_build_sash_shape(), f"{name}_vinyl"),
        material="vinyl",
        name=f"{name}_vinyl",
    )
    sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), f"{name}_glass"),
        material="glass",
        name=f"{name}_glass",
    )


def _add_latch(model: ArticulatedObject, sash_name: str) -> None:
    """Add cam-latch hardware on the sliding sash's meeting stile."""
    sash = model.get_part(sash_name)

    stile_x = -SASH_OPENING_W / 2.0 - SASH_FACE / 2.0
    face_y = SASH_DEPTH / 2.0
    plate_y = face_y + LATCH_PLATE_T / 2.0

    sash.visual(
        Box((LATCH_PLATE_W, LATCH_PLATE_T, LATCH_PLATE_H)),
        origin=Origin(xyz=(stile_x, plate_y, 0.0)),
        material="metal",
        name=f"{sash_name}_latch_plate",
    )
    lever_y = face_y + LATCH_PLATE_T + LATCH_LEVER_LEN / 2.0
    sash.visual(
        Cylinder(radius=LATCH_LEVER_R, length=LATCH_LEVER_LEN),
        origin=Origin(xyz=(stile_x, lever_y, -0.008), rpy=(1.5707963, 0.0, 0.0)),
        material="metal",
        name=f"{sash_name}_latch_lever",
    )


def _add_rollers(model: ArticulatedObject, sash_name: str) -> None:
    """Add two small roller blocks at the bottom of the sliding sash."""
    sash = model.get_part(sash_name)
    # Rollers sit at the bottom of the sash (local z = -SASH_OPENING_H/2 - SASH_FACE)
    # and protrude slightly below.
    roller_z = -SASH_OPENING_H / 2.0 - SASH_FACE - ROLLER_H / 2.0
    roller_y = 0.0  # centered in sash depth

    for i, xoff in enumerate([-ROLLER_INSET_X, ROLLER_INSET_X]):
        sash.visual(
            Box((ROLLER_W, ROLLER_D, ROLLER_H)),
            origin=Origin(xyz=(xoff, roller_y, roller_z)),
            material="roller",
            name=f"{sash_name}_roller_{i}",
        )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_lift_sliding_window")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("metal", rgba=METAL_RGBA)
    model.material("screen_mesh", rgba=SCREEN_MESH_RGBA)
    model.material("roller", rgba=ROLLER_RGBA)

    # --- Static outer frame (root) with sill lip and drainage slots ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="vinyl",
        name="frame_shell",
    )

    # --- Fixed (left) sash ---
    _add_sash(model, "fixed_sash")

    # --- Sliding (right) sash with latch and rollers ---
    _add_sash(model, "sliding_sash")
    _add_latch(model, "sliding_sash")
    _add_rollers(model, "sliding_sash")

    # --- Vent panel (corner-lift, child of sliding sash) ---
    vent = model.part("vent_panel")
    vent.visual(
        mesh_from_cadquery(_build_vent_frame_shape(), "vent_frame"),
        material="vinyl",
        name="vent_frame",
    )
    vent.visual(
        mesh_from_cadquery(_build_vent_glass_shape(), "vent_glass"),
        material="glass",
        name="vent_glass",
    )

    # --- Insect screen (independent prismatic on frame) ---
    screen = model.part("insect_screen")
    screen.visual(
        mesh_from_cadquery(_build_screen_frame_shape(), "screen_frame"),
        material="vinyl",
        name="screen_frame",
    )
    screen.visual(
        mesh_from_cadquery(_build_screen_mesh_shape(), "screen_mesh"),
        material="screen_mesh",
        name="screen_mesh",
    )

    # === Articulations ===

    # FIXED left sash seated in the rear glazing plane.
    model.articulation(
        "frame_to_fixed_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="fixed_sash",
        origin=Origin(xyz=(FIXED_OPEN_CX, FIXED_SASH_Y, MID_CZ)),
    )

    # SLIDING right sash: PRISMATIC along X. Positive q slides left (-X) to open.
    slide_travel = SASH_OPENING_W * 0.90
    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="sliding_sash",
        origin=Origin(xyz=(SLIDE_OPEN_CX, SLIDE_SASH_Y, MID_CZ)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5, lower=0.0, upper=slide_travel),
    )

    # VENT PANEL: REVOLUTE corner-lift. The vent is a child of the sliding sash.
    # Vent local frame has origin at top-center (hinge line). The hinge axis is
    # along X (horizontal, at the top edge of the vent). Positive rotation tilts
    # the bottom of the vent inward (-Y direction) for ventilation.
    # Place the vent at the upper portion of the sliding sash, near the top rail.
    vent_hinge_z_in_sash = SASH_OPENING_H / 2.0 - VENT_H / 2.0  # upper region
    vent_hinge_x_in_sash = 0.0  # centered on sash
    vent_hinge_y_in_sash = 0.0  # centered in sash depth (overlaps glass for contact)

    # The articulation origin is in the PARENT (sliding_sash) local frame.
    # The child (vent_panel) frame has origin at its top edge (hinge line).
    # axis=(1,0,0): positive q rotates around +X, which tilts bottom of vent
    # toward -Y (inward) since the vent hangs below the hinge.
    model.articulation(
        "sash_to_vent",
        ArticulationType.REVOLUTE,
        parent="sliding_sash",
        child="vent_panel",
        origin=Origin(xyz=(vent_hinge_x_in_sash, vent_hinge_y_in_sash, vent_hinge_z_in_sash)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=0.0, upper=0.52),
    )
    # INSECT SCREEN: PRISMATIC along X, on its own track behind the fixed sash.
    # Screen center is at the frame center in X/Z, at SCREEN_Y in depth.
    screen_travel = INNER_W * 0.45  # shallow travel
    model.articulation(
        "frame_to_screen",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="insect_screen",
        origin=Origin(xyz=(0.0, SCREEN_Y, MID_CZ)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.3, lower=0.0, upper=screen_travel),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    fixed_sash = object_model.get_part("fixed_sash")
    sliding_sash = object_model.get_part("sliding_sash")
    vent_panel = object_model.get_part("vent_panel")
    insect_screen = object_model.get_part("insect_screen")

    slide = object_model.get_articulation("frame_to_sliding_sash")
    vent_joint = object_model.get_articulation("sash_to_vent")
    screen_joint = object_model.get_articulation("frame_to_screen")

    # --- Intentional overlaps ---
    # Glass tucks under the vinyl sash lip on each sash (captured glass).
    for nm in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_vinyl",
            reason="Clear pane is rebated under the sash lip so it reads captured, not floating.",
        )
    # Each sash ring laps the frame opening edge (glazing rebate / track).
    for nm in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_vinyl",
            reason=f"{nm} ring is rebated into the frame opening / head-sill track (seated capture).",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_glass",
            reason=f"{nm} glass is rebated under the frame opening lip (captured glazing).",
        )
    # Latch keeper plate seated onto sliding sash.
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="sliding_sash_latch_plate",
        elem_b="sliding_sash_vinyl",
        reason="Latch keeper plate is seated onto the sliding-sash meeting-stile face.",
    )
    # Roller blocks seated under the sliding sash bottom rail.
    for i in range(2):
        ctx.allow_overlap(
            "sliding_sash", "sliding_sash",
            elem_a=f"sliding_sash_roller_{i}",
            elem_b="sliding_sash_vinyl",
            reason="Roller block is seated against the bottom rail of the sliding sash.",
        )
        # Rollers run in the sill track groove of the frame.
        ctx.allow_overlap(
            "frame", "sliding_sash",
            elem_a="frame_shell",
            elem_b=f"sliding_sash_roller_{i}",
            reason=f"Roller {i} runs in the sill track of the frame (seated in track groove).",
        )
    # Vent panel seated on the sliding sash front face.
    ctx.allow_overlap(
        "sliding_sash", "vent_panel",
        elem_a="sliding_sash_vinyl",
        elem_b="vent_frame",
        reason="Vent panel frame is seated against the sliding sash front face (mounted hinge contact).",
    )
    # Vent panel overlaps with sliding sash glass (vent replaces glass in that corner).
    ctx.allow_overlap(
        "sliding_sash", "vent_panel",
        elem_a="sliding_sash_glass",
        elem_b="vent_frame",
        reason="Vent panel is inset into the sliding sash glazing area (vent replaces glass in upper corner).",
    )
    # Insect screen runs in frame tracks at head and sill.
    ctx.allow_overlap(
        "frame", "insect_screen",
        elem_a="frame_shell",
        elem_b="screen_frame",
        reason="Insect screen frame runs in the head/sill tracks of the outer frame (captured track fit).",
    )
    # Insect screen passes behind the fixed sash in a rear track.
    ctx.allow_overlap(
        "fixed_sash", "insect_screen",
        elem_a="fixed_sash_vinyl",
        elem_b="screen_frame",
        reason="Insect screen passes behind the fixed sash in a rear screen track (close clearance).",
    )
    # Vent panel glass captured in vent frame
    ctx.allow_overlap(
        "vent_panel", "vent_panel",
        elem_a="vent_glass",
        elem_b="vent_frame",
        reason="Vent glass is rebated under the vent frame lip (captured glazing).",
    )
    # Screen mesh captured in screen frame
    ctx.allow_overlap(
        "insect_screen", "insect_screen",
        elem_a="screen_mesh",
        elem_b="screen_frame",
        reason="Screen mesh is seated within the screen frame (captured mesh).",
    )

    # --- Closed pose (q=0): window reads SHUT ---
    with ctx.pose({slide: 0.0, vent_joint: 0.0, screen_joint: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        f_aabb = ctx.part_world_aabb(fixed_sash)
        s_aabb = ctx.part_world_aabb(sliding_sash)

        # Frame spans the full width.
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        sash_w = s_aabb[1][0] - s_aabb[0][0]
        ctx.check(
            "frame spans wider than a single sash",
            frame_w > sash_w + 0.40,
            details=f"frame_w={frame_w:.3f}, sash_w={sash_w:.3f}",
        )
        # Sill near floor.
        ctx.check(
            "sill sits near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"frame zmin={frame_aabb[0][2]:.4f}",
        )
        # Two sashes side by side.
        fx = (f_aabb[0][0] + f_aabb[1][0]) / 2.0
        sx = (s_aabb[0][0] + s_aabb[1][0]) / 2.0
        ctx.check(
            "fixed sash left of sliding sash",
            fx < sx,
            details=f"fixed_x={fx:.3f}, sliding_x={sx:.3f}",
        )
        # Sliding sash proud of fixed sash.
        fy = (f_aabb[0][1] + f_aabb[1][1]) / 2.0
        sy = (s_aabb[0][1] + s_aabb[1][1]) / 2.0
        ctx.check(
            "sliding sash proud of fixed sash",
            sy > fy + 0.02,
            details=f"sliding_y={sy:.3f}, fixed_y={fy:.3f}",
        )
        # Both sashes seated in frame opening.
        ctx.expect_overlap(
            fixed_sash, frame, axes="xz", min_overlap=0.03,
            name="fixed sash seated in frame opening",
        )
        ctx.expect_overlap(
            sliding_sash, frame, axes="xz", min_overlap=0.03,
            name="sliding sash seated in frame opening",
        )

        # --- Roller blocks exist and are at the bottom of the sliding sash ---
        roller0_aabb = ctx.part_element_world_aabb(sliding_sash, elem="sliding_sash_roller_0")
        roller1_aabb = ctx.part_element_world_aabb(sliding_sash, elem="sliding_sash_roller_1")
        sash_bottom = s_aabb[0][2]
        ctx.check(
            "roller_0 at or below sash bottom",
            roller0_aabb[0][2] <= sash_bottom + 0.005,
            details=f"roller0_zmin={roller0_aabb[0][2]:.4f}, sash_bottom={sash_bottom:.4f}",
        )
        ctx.check(
            "roller_1 at or below sash bottom",
            roller1_aabb[0][2] <= sash_bottom + 0.005,
            details=f"roller1_zmin={roller1_aabb[0][2]:.4f}, sash_bottom={sash_bottom:.4f}",
        )
        # Two rollers separated in X
        r0_cx = (roller0_aabb[0][0] + roller0_aabb[1][0]) / 2.0
        r1_cx = (roller1_aabb[0][0] + roller1_aabb[1][0]) / 2.0
        ctx.check(
            "rollers separated horizontally",
            abs(r1_cx - r0_cx) > 0.10,
            details=f"r0_cx={r0_cx:.3f}, r1_cx={r1_cx:.3f}",
        )

        # --- Vent panel exists and is at the upper region of the sliding sash ---
        vent_aabb = ctx.part_world_aabb(vent_panel)
        vent_cz = (vent_aabb[0][2] + vent_aabb[1][2]) / 2.0
        sash_cz = (s_aabb[0][2] + s_aabb[1][2]) / 2.0
        ctx.check(
            "vent panel is in upper region of sliding sash",
            vent_cz > sash_cz + 0.20,
            details=f"vent_cz={vent_cz:.3f}, sash_cz={sash_cz:.3f}",
        )
        # Vent panel at rest (q=0) should overlap the sliding sash in XY
        ctx.expect_overlap(
            vent_panel, sliding_sash, axes="xy", min_overlap=0.01,
            name="vent panel overlaps sliding sash footprint at rest",
        )

        # --- Insect screen exists behind the sashes ---
        screen_aabb = ctx.part_world_aabb(insect_screen)
        screen_cy = (screen_aabb[0][1] + screen_aabb[1][1]) / 2.0
        ctx.check(
            "insect screen behind fixed sash in Y",
            screen_cy < fy,
            details=f"screen_y={screen_cy:.3f}, fixed_sash_y={fy:.3f}",
        )
        # Screen is large enough to cover most of the opening
        screen_w = screen_aabb[1][0] - screen_aabb[0][0]
        screen_h = screen_aabb[1][2] - screen_aabb[0][2]
        ctx.check(
            "screen width covers most of the inner opening",
            screen_w > INNER_W * 0.70,
            details=f"screen_w={screen_w:.3f}, inner_w={INNER_W:.3f}",
        )
        ctx.check(
            "screen height covers most of the inner opening",
            screen_h > INNER_H * 0.70,
            details=f"screen_h={screen_h:.3f}, inner_h={INNER_H:.3f}",
        )

        rest_sx = sx
        rest_sz = (s_aabb[0][2] + s_aabb[1][2]) / 2.0

    # --- Sliding sash opens toward fixed sash (-X) ---
    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel, vent_joint: 0.0, screen_joint: 0.0}):
        s_open = ctx.part_world_aabb(sliding_sash)
        open_sx = (s_open[0][0] + s_open[1][0]) / 2.0
        ctx.check(
            "sliding sash opens toward fixed sash (-X)",
            abs((rest_sx - open_sx) - travel) < 0.02 and open_sx < rest_sx - 0.30,
            details=f"rest_x={rest_sx:.3f}, open_x={open_sx:.3f}, travel={travel:.3f}",
        )
        # Pure horizontal slide (no Z change).
        open_sz = (s_open[0][2] + s_open[1][2]) / 2.0
        ctx.check(
            "slide is purely horizontal",
            abs(open_sz - rest_sz) < 0.02,
            details=f"open_z={open_sz:.3f}, rest_z={rest_sz:.3f}",
        )
        # Retained insertion: sliding sash stays within frame X span.
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained within frame X span at full travel",
            s_open[0][0] > f_aabb[0][0] - 1e-4 and s_open[1][0] < f_aabb[1][0] + 1e-4,
            details=f"sash x=[{s_open[0][0]:.3f},{s_open[1][0]:.3f}] frame x=[{f_aabb[0][0]:.3f},{f_aabb[1][0]:.3f}]",
        )
        ctx.expect_overlap(
            sliding_sash, frame, axes="z", min_overlap=0.10,
            name="sash retains vertical engagement with head/sill track",
        )

    # --- Vent panel tilts open (corner-lift) ---
    with ctx.pose({slide: 0.0, vent_joint: vent_joint.motion_limits.upper}):
        vent_open_aabb = ctx.part_world_aabb(vent_panel)
        vent_open_cy = (vent_open_aabb[0][1] + vent_open_aabb[1][1]) / 2.0
        # At rest the vent is at sliding_sash Y; opened it should shift in Y
        # (the bottom tilts inward).
        # Compare with closed vent Y position
        with ctx.pose({slide: 0.0, vent_joint: 0.0}):
            vent_closed_aabb = ctx.part_world_aabb(vent_panel)
            vent_closed_cy = (vent_closed_aabb[0][1] + vent_closed_aabb[1][1]) / 2.0
        ctx.check(
            "vent panel tilts when opened (Y center shifts)",
            abs(vent_open_cy - vent_closed_cy) > 0.005,
            details=f"closed_y={vent_closed_cy:.4f}, open_y={vent_open_cy:.4f}",
        )

    # --- Insect screen slides independently ---
    screen_travel = screen_joint.motion_limits.upper
    with ctx.pose({slide: 0.0, vent_joint: 0.0, screen_joint: 0.0}):
        screen_rest_aabb = ctx.part_world_aabb(insect_screen)
        screen_rest_cx = (screen_rest_aabb[0][0] + screen_rest_aabb[1][0]) / 2.0

    with ctx.pose({slide: 0.0, vent_joint: 0.0, screen_joint: screen_travel}):
        screen_open_aabb = ctx.part_world_aabb(insect_screen)
        screen_open_cx = (screen_open_aabb[0][0] + screen_open_aabb[1][0]) / 2.0

    ctx.check(
        "insect screen slides in -X when driven",
        screen_open_cx < screen_rest_cx - 0.05,
        details=f"rest_cx={screen_rest_cx:.3f}, open_cx={screen_open_cx:.3f}, travel={screen_travel:.3f}",
    )
    ctx.check(
        "screen travel matches expected displacement",
        abs((screen_rest_cx - screen_open_cx) - screen_travel) < 0.02,
        details=f"displacement={screen_rest_cx - screen_open_cx:.3f}, travel={screen_travel:.3f}",
    )

    # --- Frame has sill lip geometry (extends below inner_z0 on exterior) ---
    frame_aabb = ctx.part_world_aabb(frame)
    # The sill lip protrudes on the +Y side at the bottom
    # Check that frame extends to at least the outer boundary in Y beyond FRAME_DEPTH/2
    frame_max_y = frame_aabb[1][1]
    ctx.check(
        "frame extends beyond base depth for sill lip",
        frame_max_y > FRAME_DEPTH / 2.0 + 0.005,
        details=f"frame_max_y={frame_max_y:.4f}, expected>{FRAME_DEPTH / 2.0 + 0.005:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
