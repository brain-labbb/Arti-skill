from __future__ import annotations

# Corner-lift sliding window: three-panel horizontal sliding window with white
# vinyl frame and colonial divided-lite grilles. Center sash slides sideways
# (prismatic) along the head/sill track.
#
# Variant features over the base three-panel slider:
#   - Small vent panel at bottom-right corner of center sash (revolute hinge
#     along top edge; tilts outward for corner-lift ventilation).
#   - Thumb-turn latch at the meeting rail of the center sash (revolute around
#     Y; arm swings from engaged-horizontal to disengaged-vertical).
#   - Two roller blocks at the bottom of the center sash (ride on sill track).
#   - Visible overlap stile on the meeting edge (left stile) of the center sash
#     that extends past the sash back face, showing where panes cross.
#
# Coordinate convention:
#   +Z up, window stands vertically.
#     width  -> X
#     height -> Z   (sill near z=0)
#     depth  -> Y   (glass plane is X-Z)
#   Window reads SHUT at q=0 for all joints.

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

TOTAL_W = 3.00
TOTAL_H = 1.50

FRAME_FACE = 0.070
MULLION_FACE = 0.060
FRAME_DEPTH = 0.110

SIDE_LITE_W = 0.85
CENTER_LITE_W = 1.04

SASH_FACE = 0.055
SASH_DEPTH = 0.055
GLASS_T = 0.008

GRILLE_COLS = 4
GRILLE_ROWS = 5
MUNTIN_T = 0.020
MUNTIN_DEPTH = 0.020

FIXED_LITE_Y = -0.020
SLIDE_SASH_Y = 0.052

REBATE = 0.005

# --- Variant-specific dimensions ---
VENT_W = 0.16           # vent panel width
VENT_H = 0.13           # vent panel height
VENT_DEPTH = 0.028      # vent panel frame depth (Y)
VENT_FRAME_W = 0.018    # vent panel frame rail/stile width

LATCH_ARM_L = 0.050     # latch arm length
LATCH_ARM_W = 0.014     # latch arm width
LATCH_ARM_T = 0.006     # latch arm thickness (Y)
LATCH_BASE_R = 0.012    # latch pivot boss radius
LATCH_BASE_H = 0.012    # latch pivot boss height (Y)

ROLLER_W = 0.028        # roller block width (X)
ROLLER_H = 0.016        # roller block height (Z)
ROLLER_D = 0.022        # roller block depth (Y)

OVERLAP_STILE_W = 0.028   # overlap stile face width (X)
OVERLAP_STILE_EXT = 0.022 # overlap stile extension past sash back face (-Y)

# ---------------------------------------------------------------------------
# Derived layout
# ---------------------------------------------------------------------------

HALF_W = TOTAL_W / 2.0

INNER_X0 = -HALF_W + FRAME_FACE
INNER_X1 = HALF_W - FRAME_FACE
INNER_Z0 = FRAME_FACE
INNER_Z1 = TOTAL_H - FRAME_FACE

LEFT_X0 = INNER_X0
LEFT_X1 = LEFT_X0 + SIDE_LITE_W
MUL0_X0 = LEFT_X1
MUL0_X1 = MUL0_X0 + MULLION_FACE
CENTER_X0 = MUL0_X1
CENTER_X1 = CENTER_X0 + CENTER_LITE_W
MUL1_X0 = CENTER_X1
MUL1_X1 = MUL1_X0 + MULLION_FACE
RIGHT_X0 = MUL1_X1
RIGHT_X1 = INNER_X1

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)
GLASS_RGBA = (0.50, 0.58, 0.64, 0.32)
METAL_RGBA = (0.35, 0.36, 0.38, 1.0)   # dark satin metal for hardware


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery)
# ---------------------------------------------------------------------------

def _slab(x0, x1, z0, z1, y_center, depth):
    w = x1 - x0
    h = z1 - z0
    cx = (x0 + x1) / 2.0
    cz = (z0 + z1) / 2.0
    return (
        cq.Workplane("XY")
        .transformed(offset=(cx, y_center, cz))
        .box(w, depth, h)
    )


def _build_frame_shape():
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    cut_depth = FRAME_DEPTH + 0.02
    left_cut = _slab(LEFT_X0, LEFT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    center_cut = _slab(CENTER_X0, CENTER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    right_cut = _slab(RIGHT_X0, RIGHT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    return outer.cut(left_cut).cut(center_cut).cut(right_cut)


def _build_sash_grille_shape(opening_w, opening_h):
    ow = opening_w
    oh = opening_h
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE

    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SASH_DEPTH)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH + 0.02)
    ring = outer.cut(opening)

    bars = None
    for c in range(1, GRILLE_COLS):
        frac = c / GRILLE_COLS
        x = -ow / 2.0 + frac * ow
        bar = _slab(
            x - MUNTIN_T / 2.0, x + MUNTIN_T / 2.0,
            -oh / 2.0, oh / 2.0,
            0.0, MUNTIN_DEPTH,
        )
        bars = bar if bars is None else bars.union(bar)

    for r in range(1, GRILLE_ROWS):
        frac = r / GRILLE_ROWS
        z = -oh / 2.0 + frac * oh
        bar = _slab(
            -ow / 2.0, ow / 2.0,
            z - MUNTIN_T / 2.0, z + MUNTIN_T / 2.0,
            0.0, MUNTIN_DEPTH,
        )
        bars = bar if bars is None else bars.union(bar)

    return ring if bars is None else ring.union(bars)


def _build_sash_glass_shape(opening_w, opening_h):
    ow = opening_w + 2 * REBATE
    oh = opening_h + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_vent_panel_shape():
    """Vent panel frame + glass in its own local frame.
    Hinge line at Z=0 (top edge). Panel extends downward to Z=-VENT_H.
    Centered on X=0, depth centered on Y=0.
    """
    fw = VENT_FRAME_W
    # Outer frame slab
    outer = _slab(
        -VENT_W / 2.0, VENT_W / 2.0,
        -VENT_H, 0.0,
        0.0, VENT_DEPTH,
    )
    # Cut the inner opening (leaving the frame rails/stiles)
    inner_w = VENT_W - 2 * fw
    inner_h = VENT_H - 2 * fw
    opening = _slab(
        -inner_w / 2.0, inner_w / 2.0,
        -VENT_H + fw, -fw,
        0.0, VENT_DEPTH + 0.01,
    )
    frame = outer.cut(opening)
    return frame


def _build_vent_glass_shape():
    """Glass pane for the vent panel, in the same vent-local frame."""
    fw = VENT_FRAME_W
    inner_w = VENT_W - 2 * fw + 2 * REBATE
    inner_h = VENT_H - 2 * fw + 2 * REBATE
    cy = 0.0
    cz = (-VENT_H + fw + (-fw)) / 2.0  # center of opening in Z
    return _slab(
        -inner_w / 2.0, inner_w / 2.0,
        cz - inner_h / 2.0, cz + inner_h / 2.0,
        cy, GLASS_T,
    )


def _build_latch_shape():
    """Thumb-turn latch in its own local frame.
    Pivot at origin. Boss extends along +Y. Arm extends along +X from boss mid-height.
    Built as one connected solid: boss block + arm block overlap in X/Y/Z.
    """
    # Boss: rectangular pivot block extending along +Y from origin
    boss = _slab(
        -LATCH_BASE_R, LATCH_BASE_R,
        -LATCH_BASE_R, LATCH_BASE_R,
        LATCH_BASE_H / 2.0,
        LATCH_BASE_H,
    )
    # Arm: rectangular bar extending along +X from the boss.
    # Arm Y center matches boss Y center so they share volume.
    # Arm starts at X = -LATCH_BASE_R/2 (inside the boss) for solid overlap.
    arm = _slab(
        -LATCH_BASE_R / 2.0, LATCH_ARM_L,
        -LATCH_ARM_W / 2.0, LATCH_ARM_W / 2.0,
        LATCH_BASE_H / 2.0,
        LATCH_ARM_T,
    )
    return boss.union(arm)


# ---------------------------------------------------------------------------
# Part builders
# ---------------------------------------------------------------------------

def _add_lite(model, name, opening_w, opening_h):
    lite = model.part(name)
    lite.visual(
        mesh_from_cadquery(_build_sash_grille_shape(opening_w, opening_h), f"{name}_vinyl"),
        material="vinyl",
        name=f"{name}_vinyl",
    )
    lite.visual(
        mesh_from_cadquery(_build_sash_glass_shape(opening_w, opening_h), f"{name}_glass"),
        material="glass",
        name=f"{name}_glass",
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model():
    span = SIDE_LITE_W + MULLION_FACE + CENTER_LITE_W + MULLION_FACE + SIDE_LITE_W
    inner_w = INNER_X1 - INNER_X0
    assert abs(span - inner_w) < 1e-6, f"lite layout {span} != inner width {inner_w}"

    model = ArticulatedObject(name="corner_lift_sliding_window")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("metal", rgba=METAL_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="vinyl",
        name="frame_shell",
    )

    opening_h = INNER_Z1 - INNER_Z0

    # --- Two FIXED side lites ---
    _add_lite(model, "left_lite", SIDE_LITE_W, opening_h)
    _add_lite(model, "right_lite", SIDE_LITE_W, opening_h)

    # --- CENTER sliding sash ---
    center_sash = model.part("center_sash")
    center_sash.visual(
        mesh_from_cadquery(
            _build_sash_grille_shape(CENTER_LITE_W, opening_h),
            "center_sash_vinyl",
        ),
        material="vinyl",
        name="center_sash_vinyl",
    )
    center_sash.visual(
        mesh_from_cadquery(
            _build_sash_glass_shape(CENTER_LITE_W, opening_h),
            "center_sash_glass",
        ),
        material="glass",
        name="center_sash_glass",
    )

    # --- Overlap stile on meeting edge (left stile) of center sash ---
    # A vertical strip that extends past the sash back face (-Y), showing where
    # the sliding sash overlaps the adjacent fixed lite.
    sash_half_h = opening_h / 2.0 + SASH_FACE
    overlap_stile_depth = SASH_DEPTH + OVERLAP_STILE_EXT
    overlap_stile_y = -OVERLAP_STILE_EXT / 2.0  # extends past back face
    overlap_stile_x = -(CENTER_LITE_W / 2.0 + SASH_FACE / 2.0)
    center_sash.visual(
        Box((OVERLAP_STILE_W, overlap_stile_depth, 2 * sash_half_h)),
        origin=Origin(xyz=(overlap_stile_x, overlap_stile_y, 0.0)),
        material="vinyl",
        name="overlap_stile",
    )

    # --- Two roller blocks at the bottom of the center sash ---
    roller_z = -(opening_h / 2.0 + SASH_FACE)  # bottom of sash
    roller_x_left = -CENTER_LITE_W / 3.0
    roller_x_right = CENTER_LITE_W / 3.0
    center_sash.visual(
        Box((ROLLER_W, ROLLER_D, ROLLER_H)),
        origin=Origin(xyz=(roller_x_left, 0.0, roller_z - ROLLER_H / 2.0)),
        material="metal",
        name="roller_left",
    )
    center_sash.visual(
        Box((ROLLER_W, ROLLER_D, ROLLER_H)),
        origin=Origin(xyz=(roller_x_right, 0.0, roller_z - ROLLER_H / 2.0)),
        material="metal",
        name="roller_right",
    )

    # --- Vent panel (child of center_sash, revolute hinge) ---
    vent_panel = model.part("vent_panel")
    vent_panel.visual(
        mesh_from_cadquery(_build_vent_panel_shape(), "vent_frame"),
        material="vinyl",
        name="vent_frame",
    )
    vent_panel.visual(
        mesh_from_cadquery(_build_vent_glass_shape(), "vent_glass"),
        material="glass",
        name="vent_glass",
    )

    # --- Latch (child of center_sash, revolute thumb-turn) ---
    latch = model.part("latch")
    latch.visual(
        mesh_from_cadquery(_build_latch_shape(), "latch_body"),
        material="metal",
        name="latch_body",
    )

    # --- Articulation layout ---
    left_cx = (LEFT_X0 + LEFT_X1) / 2.0
    right_cx = (RIGHT_X0 + RIGHT_X1) / 2.0
    center_cx = (CENTER_X0 + CENTER_X1) / 2.0
    mid_cz = (INNER_Z0 + INNER_Z1) / 2.0

    # FIXED side lites
    model.articulation(
        "frame_to_left_lite",
        ArticulationType.FIXED,
        parent="frame",
        child="left_lite",
        origin=Origin(xyz=(left_cx, FIXED_LITE_Y, mid_cz)),
    )
    model.articulation(
        "frame_to_right_lite",
        ArticulationType.FIXED,
        parent="frame",
        child="right_lite",
        origin=Origin(xyz=(right_cx, FIXED_LITE_Y, mid_cz)),
    )

    # CENTER sliding sash: PRISMATIC along +X
    slide_travel = SIDE_LITE_W * 0.92
    model.articulation(
        "frame_to_center_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="center_sash",
        origin=Origin(xyz=(center_cx, SLIDE_SASH_Y, mid_cz)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.5, lower=0.0, upper=slide_travel,
        ),
    )

    # Vent panel: REVOLUTE hinge along top edge.
    # In the center_sash local frame, the hinge is near the bottom-right corner.
    # The vent panel extends downward from the hinge. Positive q tilts the
    # bottom edge outward (+Y) via axis=(1,0,0) right-hand rule.
    vent_hinge_x = CENTER_LITE_W / 2.0 - VENT_W / 2.0 - 0.03
    vent_hinge_z = -(opening_h / 2.0) + VENT_H + 0.02
    model.articulation(
        "sash_to_vent",
        ArticulationType.REVOLUTE,
        parent="center_sash",
        child="vent_panel",
        origin=Origin(xyz=(vent_hinge_x, 0.0, vent_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=1.0, lower=0.0, upper=0.55,
        ),
    )

    # Latch: REVOLUTE thumb-turn at the meeting rail (left stile of sash).
    # Pivot on the front face of the sash. Arm extends along +X when engaged.
    # Positive q rotates arm from +X toward -Z (disengaged) via axis=(0,1,0).
    latch_x = -(CENTER_LITE_W / 2.0)
    latch_y = SASH_DEPTH / 2.0
    latch_z = 0.0
    model.articulation(
        "sash_to_latch",
        ArticulationType.REVOLUTE,
        parent="center_sash",
        child="latch",
        origin=Origin(xyz=(latch_x, latch_y, latch_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0, velocity=2.0, lower=0.0, upper=1.57,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests():
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    left_lite = object_model.get_part("left_lite")
    right_lite = object_model.get_part("right_lite")
    center_sash = object_model.get_part("center_sash")
    vent_panel = object_model.get_part("vent_panel")
    latch = object_model.get_part("latch")

    slide = object_model.get_articulation("frame_to_center_sash")
    vent_hinge = object_model.get_articulation("sash_to_vent")
    latch_joint = object_model.get_articulation("sash_to_latch")

    # --- Joint type checks ---
    ctx.check(
        "slide is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"got {slide.articulation_type}",
    )
    ctx.check(
        "vent hinge is revolute",
        vent_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"got {vent_hinge.articulation_type}",
    )
    ctx.check(
        "latch joint is revolute",
        latch_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"got {latch_joint.articulation_type}",
    )

    # --- Overlap allowances ---
    # Glass rebated under sash/muntin lip on each sash.
    for nm in ("left_lite", "right_lite"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_vinyl",
            reason="Clear pane rebated under sash/muntin lip (captured glass).",
        )
    ctx.allow_overlap(
        "center_sash", "center_sash",
        elem_a="center_sash_glass",
        elem_b="center_sash_vinyl",
        reason="Center sash glass rebated under sash/muntin lip (captured glass).",
    )
    # Overlap stile is part of the center sash and intentionally overlaps the
    # sash vinyl at the meeting edge.
    ctx.allow_overlap(
        "center_sash", "center_sash",
        elem_a="overlap_stile",
        elem_b="center_sash_vinyl",
        reason="Overlap stile is an integral extension of the sash meeting stile.",
    )
    # Fixed lites rebated into frame openings.
    for nm in ("left_lite", "right_lite"):
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_vinyl",
            reason=f"{nm} rebated into frame opening (seated capture).",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_glass",
            reason=f"{nm} glass rebated under frame lip (captured glazing).",
        )
    # Center sash rides the head/sill track and laps the frame face.
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell",
        elem_b="center_sash_vinyl",
        reason="Center sash rides the head/sill track (slider capture).",
    )
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell",
        elem_b="center_sash_glass",
        reason="Center sash glass laps the head/sill track lip (captured glazing).",
    )
    # Vent panel glass rebated under vent frame lip.
    ctx.allow_overlap(
        "vent_panel", "vent_panel",
        elem_a="vent_glass",
        elem_b="vent_frame",
        reason="Vent glass rebated under vent panel frame lip.",
    )
    # Vent panel sits on the center sash face when closed; small overlap at hinge.
    ctx.allow_overlap(
        "center_sash", "vent_panel",
        elem_a="center_sash_vinyl",
        elem_b="vent_frame",
        reason="Vent panel is hinged to the sash face; small overlap at the hinge mounting.",
    )
    ctx.allow_overlap(
        "center_sash", "vent_panel",
        elem_a="center_sash_glass",
        elem_b="vent_frame",
        reason="Vent panel frame laps the sash glass at the mounting region.",
    )
    # Latch mounts on the sash front face; boss embeds slightly.
    ctx.allow_overlap(
        "center_sash", "latch",
        elem_a="center_sash_vinyl",
        elem_b="latch_body",
        reason="Latch pivot boss is surface-mounted on the sash meeting rail.",
    )
    # Overlap stile extends past the sash back face into the frame/mullion region
    # to show where the sliding sash crosses the adjacent fixed lite.
    ctx.allow_overlap(
        "center_sash", "frame",
        elem_a="overlap_stile",
        elem_b="frame_shell",
        reason="Overlap stile extends past the sash back face into the mullion region; this is the visible crossing where panes overlap.",
    )
    ctx.allow_overlap(
        "center_sash", "left_lite",
        elem_a="overlap_stile",
        elem_b="left_lite_vinyl",
        reason="Overlap stile laps the adjacent fixed lite sash at the meeting edge; visible crossing where panes overlap.",
    )
    # Vent panel sits within the sash opening; its glass and frame lap the sash glass.
    ctx.allow_overlap(
        "center_sash", "vent_panel",
        elem_a="center_sash_glass",
        elem_b="vent_glass",
        reason="Vent panel glass is coplanar with the sash glass within the opening region.",
    )
    ctx.allow_overlap(
        "frame", "vent_panel",
        elem_a="frame_shell",
        elem_b="vent_frame",
        reason="Vent panel frame sits within the sash opening near the frame sill/mullion edge; small seating overlap at the perimeter.",
    )
    # Roller blocks sit on the sill track and lap the frame sill face.
    ctx.allow_overlap(
        "center_sash", "frame",
        elem_a="roller_left",
        elem_b="frame_shell",
        reason="Left roller rides on the sill track; intentional contact/seating on the frame sill.",
    )
    ctx.allow_overlap(
        "center_sash", "frame",
        elem_a="roller_right",
        elem_b="frame_shell",
        reason="Right roller rides on the sill track; intentional contact/seating on the frame sill.",
    )

    # --- Rest pose (all joints at 0) ---
    with ctx.pose({slide: 0.0, vent_hinge: 0.0, latch_joint: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        c_aabb = ctx.part_world_aabb(center_sash)

        # Frame spans wider than center sash.
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        center_w = c_aabb[1][0] - c_aabb[0][0]
        ctx.check(
            "frame spans wider than center sash",
            frame_w > center_w + 1.5,
            details=f"frame_w={frame_w:.3f}, center_w={center_w:.3f}",
        )

        # Sill near z=0.
        ctx.check(
            "sill near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"frame zmin={frame_aabb[0][2]:.4f}",
        )

        # Vent panel is near the bottom-right of the center sash.
        vent_aabb = ctx.part_world_aabb(vent_panel)
        sash_aabb = ctx.part_world_aabb(center_sash)
        vent_cx = (vent_aabb[0][0] + vent_aabb[1][0]) / 2.0
        sash_cx = (sash_aabb[0][0] + sash_aabb[1][0]) / 2.0
        ctx.check(
            "vent panel on right side of sash",
            vent_cx > sash_cx + 0.1,
            details=f"vent_cx={vent_cx:.3f}, sash_cx={sash_cx:.3f}",
        )
        vent_cz = (vent_aabb[0][2] + vent_aabb[1][2]) / 2.0
        sash_cz = (sash_aabb[0][2] + sash_aabb[1][2]) / 2.0
        ctx.check(
            "vent panel near bottom of sash",
            vent_cz < sash_cz - 0.1,
            details=f"vent_cz={vent_cz:.3f}, sash_cz={sash_cz:.3f}",
        )

        # Latch is on the meeting rail (left side) of the sash.
        latch_aabb = ctx.part_world_aabb(latch)
        latch_cx = (latch_aabb[0][0] + latch_aabb[1][0]) / 2.0
        ctx.check(
            "latch on left (meeting) side of sash",
            latch_cx < sash_cx - 0.1,
            details=f"latch_cx={latch_cx:.3f}, sash_cx={sash_cx:.3f}",
        )

        rest_cx = sash_cx
        rest_cz = sash_cz

    # --- Vent panel opens outward (tilts in +Y from sash face) ---
    vent_open_q = vent_hinge.motion_limits.upper
    with ctx.pose({slide: 0.0, vent_hinge: vent_open_q, latch_joint: 0.0}):
        vent_open_aabb = ctx.part_world_aabb(vent_panel)
        vent_open_y_max = vent_open_aabb[1][1]

    with ctx.pose({slide: 0.0, vent_hinge: 0.0, latch_joint: 0.0}):
        vent_closed_aabb = ctx.part_world_aabb(vent_panel)
        vent_closed_y_max = vent_closed_aabb[1][1]

    ctx.check(
        "vent panel tilts outward when opened",
        vent_open_y_max > vent_closed_y_max + 0.01,
        details=f"closed_y_max={vent_closed_y_max:.4f}, open_y_max={vent_open_y_max:.4f}",
    )

    # --- Latch rotates (arm swings from engaged to disengaged) ---
    latch_open_q = latch_joint.motion_limits.upper
    with ctx.pose({slide: 0.0, vent_hinge: 0.0, latch_joint: 0.0}):
        latch_rest_aabb = ctx.part_world_aabb(latch)
        latch_rest_dx = latch_rest_aabb[1][0] - latch_rest_aabb[0][0]

    with ctx.pose({slide: 0.0, vent_hinge: 0.0, latch_joint: latch_open_q}):
        latch_open_aabb = ctx.part_world_aabb(latch)
        latch_open_dz = latch_open_aabb[1][2] - latch_open_aabb[0][2]

    # At rest the arm is horizontal (wide in X); at 90° it's vertical (tall in Z).
    ctx.check(
        "latch arm rotates from horizontal to vertical",
        latch_open_dz > latch_rest_dx * 0.6,
        details=f"rest_dx={latch_rest_dx:.4f}, open_dz={latch_open_dz:.4f}",
    )

    # --- Center sash slides sideways ---
    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel, vent_hinge: 0.0, latch_joint: 0.0}):
        c_open = ctx.part_world_aabb(center_sash)
        open_cx = (c_open[0][0] + c_open[1][0]) / 2.0
        ctx.check(
            "center sash slides along +X by ~travel",
            abs((open_cx - rest_cx) - travel) < 0.02,
            details=f"rest_cx={rest_cx:.3f}, open_cx={open_cx:.3f}, travel={travel:.3f}",
        )
        # Retained insertion: sash stays within frame X span.
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained within frame X span at full travel",
            c_open[1][0] < f_aabb[1][0] + 1e-4 and c_open[0][0] > f_aabb[0][0] - 1e-4,
            details=f"sash x=[{c_open[0][0]:.3f},{c_open[1][0]:.3f}] frame x=[{f_aabb[0][0]:.3f},{f_aabb[1][0]:.3f}]",
        )
        ctx.expect_overlap(
            center_sash, frame,
            axes="z",
            min_overlap=0.10,
            name="sash retains vertical engagement with head/sill track",
        )

    # --- Overlap stile is visible on meeting edge ---
    stile_vis = center_sash.get_visual("overlap_stile")
    ctx.check(
        "overlap stile visual exists on center sash",
        stile_vis is not None,
        details="overlap_stile visual not found",
    )

    # --- Roller blocks exist ---
    rl = center_sash.get_visual("roller_left")
    rr = center_sash.get_visual("roller_right")
    ctx.check(
        "roller_left visual exists",
        rl is not None,
        details="roller_left visual not found",
    )
    ctx.check(
        "roller_right visual exists",
        rr is not None,
        details="roller_right visual not found",
    )

    return ctx.report()


object_model = build_object_model()
