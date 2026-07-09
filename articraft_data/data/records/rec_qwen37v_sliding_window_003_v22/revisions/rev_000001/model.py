from __future__ import annotations

# Three-panel horizontal sliding window with white frame.
# Wider fixed center pane, sliding right sash with roller blocks,
# revolute latch at meeting rail, sill lip with drainage slots.
#
# Variant of the double-hung sash window, forked into a sliding window sibling.
#
# Coordinate convention:
#   +Z is up. Window stands vertically: height along +Z, width along X,
#   depth along Y. The sill sits at z=0; the head at z=WIN_H.
#   Glass plane is the X-Z plane.
#
# Layout (X):
#   |jamb| left fixed |mullion| center fixed (wider) |mullion| sliding sash |jamb|
#
# Articulation:
#   - SLIDING SASH: PRISMATIC, axis (-1,0,0), positive q slides LEFT (opens).
#   - LATCH: REVOLUTE, axis (0,0,1), rotates at the meeting rail.

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Absolute dimensions (meters)
# ---------------------------------------------------------------------------

WIN_W = 1.20           # overall window width (X)
WIN_H = 1.00           # overall window height (Z)
FRAME_FACE = 0.055     # outer frame member face width
FRAME_DEPTH = 0.110    # outer frame jamb depth (Y)

# Clear opening inside the outer frame
OPEN_W = WIN_W - 2 * FRAME_FACE
OPEN_H = WIN_H - 2 * FRAME_FACE
OPEN_X0 = -WIN_W / 2.0 + FRAME_FACE
OPEN_X1 = WIN_W / 2.0 - FRAME_FACE
OPEN_Z0 = FRAME_FACE
OPEN_Z1 = WIN_H - FRAME_FACE
OPEN_CZ = (OPEN_Z0 + OPEN_Z1) / 2.0

# Mullions separating the three panels
MULLION_W = 0.040

# Panel widths within the clear opening
_avail = OPEN_W - 2 * MULLION_W
LEFT_W = _avail * 0.25
CENTER_W = _avail * 0.50
RIGHT_W = _avail - LEFT_W - CENTER_W

# Section X boundaries
left_x0 = OPEN_X0
left_x1 = left_x0 + LEFT_W
center_x0 = left_x1 + MULLION_W
center_x1 = center_x0 + CENTER_W
right_x0 = center_x1 + MULLION_W
right_x1 = OPEN_X1

# Section centers
left_cx = (left_x0 + left_x1) / 2.0
center_cx = (center_x0 + center_x1) / 2.0
right_cx = (right_x0 + right_x1) / 2.0

# Sliding sash dimensions
SASH_W = RIGHT_W - 0.008
SASH_H = OPEN_H - 0.008
SASH_RAIL = 0.036
SASH_DEPTH = 0.030
GLASS_T = 0.005
GLASS_REBATE = 0.005
SASH_Y = -0.022       # sash rides inward of the fixed glass plane

# Travel: sash slides left, overlapping the center section
MAX_TRAVEL = RIGHT_W + MULLION_W - 0.010

# Sill lip
SILL_LIP_EXT = 0.028
SILL_LIP_THICK = 0.012

# Drainage slots
DRAIN_COUNT = 4
DRAIN_W = 0.032

# Track grooves
TRACK_GROOVE_DEPTH = 0.010
TRACK_GROOVE_W = SASH_DEPTH + 0.008

# Roller blocks
ROLLER_W = 0.022
ROLLER_D = 0.016
ROLLER_H = 0.010

# Latch
LATCH_BASE = (0.022, 0.014, 0.036)
LATCH_LEVER = (0.046, 0.010, 0.012)

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

FRAME_RGBA = (0.945, 0.945, 0.945, 1.0)
SASH_RGBA = (0.965, 0.965, 0.965, 1.0)
GLASS_RGBA = (0.30, 0.36, 0.42, 0.34)
HARDWARE_RGBA = (0.72, 0.73, 0.75, 1.0)
ROLLER_RGBA = (0.22, 0.22, 0.24, 1.0)


# ---------------------------------------------------------------------------
# Frame geometry (CadQuery)
# ---------------------------------------------------------------------------

def _build_frame_shape() -> cq.Workplane:
    """Outer perimeter frame with two mullions, track grooves, sill lip,
    and drainage slots."""
    # Solid outer slab
    shape = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, WIN_H / 2.0))
        .box(WIN_W, FRAME_DEPTH, WIN_H)
    )

    # Cut three rectangular openings (leaves head, sill, jambs, two mullions)
    for cx, w in [(left_cx, LEFT_W), (center_cx, CENTER_W), (right_cx, RIGHT_W)]:
        cut = (
            cq.Workplane("XY")
            .transformed(offset=(cx, 0.0, OPEN_CZ))
            .box(w, FRAME_DEPTH + 0.02, OPEN_H)
        )
        shape = shape.cut(cut)

    # Sill lip: thin shelf extending outward (+Y) from the frame bottom
    lip_y = FRAME_DEPTH / 2.0 + SILL_LIP_EXT / 2.0
    lip = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, lip_y, SILL_LIP_THICK / 2.0))
        .box(WIN_W, SILL_LIP_EXT, SILL_LIP_THICK)
    )
    shape = shape.union(lip)

    # Drainage slots cut through the sill lip
    spacing = WIN_W / (DRAIN_COUNT + 1)
    for i in range(DRAIN_COUNT):
        dx = -WIN_W / 2.0 + (i + 1) * spacing
        slot = (
            cq.Workplane("XY")
            .transformed(offset=(dx, lip_y, SILL_LIP_THICK / 2.0))
            .box(DRAIN_W, SILL_LIP_EXT + 0.01, SILL_LIP_THICK + 0.008)
        )
        shape = shape.cut(slot)

    # Track grooves in sill top and head bottom for the sliding sash
    track_x0 = center_x0
    track_x1 = OPEN_X1
    track_cx = (track_x0 + track_x1) / 2.0
    track_len = track_x1 - track_x0

    # Bottom track groove (cut into sill top surface)
    bt = (
        cq.Workplane("XY")
        .transformed(offset=(track_cx, SASH_Y, OPEN_Z0 - TRACK_GROOVE_DEPTH / 2.0))
        .box(track_len, TRACK_GROOVE_W, TRACK_GROOVE_DEPTH)
    )
    shape = shape.cut(bt)

    # Top track groove (cut into head bottom surface)
    tt = (
        cq.Workplane("XY")
        .transformed(offset=(track_cx, SASH_Y, OPEN_Z1 + TRACK_GROOVE_DEPTH / 2.0))
        .box(track_len, TRACK_GROOVE_W, TRACK_GROOVE_DEPTH)
    )
    shape = shape.cut(tt)

    return shape


# ---------------------------------------------------------------------------
# Sash frame geometry (CadQuery)
# ---------------------------------------------------------------------------

def _build_sash_frame_shape() -> cq.Workplane:
    """Sliding sash perimeter frame: box with inner cutout for glass."""
    w, h, d = SASH_W, SASH_H, SASH_DEPTH
    r = SASH_RAIL

    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(w, d, h)
    )

    inner_w = w - 2 * r
    inner_h = h - 2 * r
    cutout = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(inner_w, d + 0.02, inner_h)
    )

    return outer.cut(cutout)


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_panel_sliding_window")

    model.material("frame", rgba=FRAME_RGBA)
    model.material("sash", rgba=SASH_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("hardware", rgba=HARDWARE_RGBA)
    model.material("roller", rgba=ROLLER_RGBA)

    # --- Frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="frame",
        name="frame_shell",
    )

    # Fixed left glass pane (rebated into frame)
    lg_w = LEFT_W + 2 * GLASS_REBATE
    lg_h = OPEN_H + 2 * GLASS_REBATE
    frame.visual(
        Box((lg_w, GLASS_T, lg_h)),
        origin=Origin(xyz=(left_cx, 0.0, OPEN_CZ)),
        material="glass",
        name="left_fixed_glass",
    )

    # Fixed center glass pane (wider, rebated into frame)
    cg_w = CENTER_W + 2 * GLASS_REBATE
    cg_h = OPEN_H + 2 * GLASS_REBATE
    frame.visual(
        Box((cg_w, GLASS_T, cg_h)),
        origin=Origin(xyz=(center_cx, 0.0, OPEN_CZ)),
        material="glass",
        name="center_fixed_glass",
    )

    # --- Sliding sash ---
    sash = model.part("sliding_sash")
    sash.visual(
        mesh_from_cadquery(_build_sash_frame_shape(), "sash_frame"),
        material="sash",
        name="sash_frame",
    )

    # Sash glass (captured under rails)
    sash_inner_w = SASH_W - 2 * SASH_RAIL
    sash_inner_h = SASH_H - 2 * SASH_RAIL
    sash.visual(
        Box((sash_inner_w + 0.008, GLASS_T, sash_inner_h + 0.008)),
        origin=Origin(xyz=(0.0, 0.0, SASH_H / 2.0)),
        material="glass",
        name="sash_glass",
    )

    # Roller blocks at the bottom of the sash
    roller_z = -ROLLER_H / 2.0
    sash.visual(
        Box((ROLLER_W, ROLLER_D, ROLLER_H)),
        origin=Origin(xyz=(-SASH_W / 2.0 + 0.032, 0.0, roller_z)),
        material="roller",
        name="roller_left",
    )
    sash.visual(
        Box((ROLLER_W, ROLLER_D, ROLLER_H)),
        origin=Origin(xyz=(SASH_W / 2.0 - 0.032, 0.0, roller_z)),
        material="roller",
        name="roller_right",
    )

    # Latch base plate (mounted on sash left stile, doesn't rotate)
    latch_px = -SASH_W / 2.0 + SASH_RAIL / 2.0
    latch_py = -SASH_DEPTH / 2.0 - LATCH_BASE[1] / 2.0 + 0.004
    latch_pz = SASH_H / 2.0
    sash.visual(
        Box(LATCH_BASE),
        origin=Origin(xyz=(latch_px, latch_py, latch_pz)),
        material="hardware",
        name="latch_base",
    )

    # --- Latch lever (separate part, rotates on revolute joint) ---
    latch = model.part("latch")
    latch.visual(
        Box(LATCH_LEVER),
        origin=Origin(xyz=(-LATCH_LEVER[0] / 2.0, 0.0, 0.0)),
        material="hardware",
        name="latch_lever",
    )

    # ----- Articulations -----

    # Sliding sash: prismatic, positive q slides left (opens)
    model.articulation(
        "frame_to_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="sliding_sash",
        origin=Origin(xyz=(right_cx, SASH_Y, OPEN_Z0 + 0.004)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0, velocity=0.3, lower=0.0, upper=MAX_TRAVEL
        ),
    )

    # Latch: revolute, rotates at meeting rail
    model.articulation(
        "sash_to_latch",
        ArticulationType.REVOLUTE,
        parent="sliding_sash",
        child="latch",
        origin=Origin(xyz=(latch_px, latch_py, latch_pz)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=2.0, lower=0.0, upper=1.2
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    sash = object_model.get_part("sliding_sash")
    latch = object_model.get_part("latch")
    j_slide = object_model.get_articulation("frame_to_sash")
    j_latch = object_model.get_articulation("sash_to_latch")

    # --- Intentional overlaps ---

    # Glass panes rebated into frame/sash (captured glass)
    ctx.allow_overlap(
        "frame", "frame",
        elem_a="left_fixed_glass", elem_b="frame_shell",
        reason="Left fixed glass is rebated into the frame rabbet (captured glazing).",
    )
    ctx.allow_overlap(
        "frame", "frame",
        elem_a="center_fixed_glass", elem_b="frame_shell",
        reason="Center fixed glass is rebated into the frame rabbet (captured glazing).",
    )
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="sash_glass", elem_b="sash_frame",
        reason="Sash glass is rebated under the sash rails (captured glass).",
    )

    # Sash rides in frame track grooves
    ctx.allow_overlap(
        "frame", "sliding_sash",
        reason="Sliding sash stiles ride in the frame track grooves (retained insertion).",
    )

    # Latch base seated on sash stile
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="latch_base", elem_b="sash_frame",
        reason="Latch base is mounted (seated) onto the sash meeting-rail stile.",
    )

    # Rollers seated in sash bottom rail
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="roller_left", elem_b="sash_frame",
        reason="Left roller block is seated into the sash bottom rail.",
    )
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="roller_right", elem_b="sash_frame",
        reason="Right roller block is seated into the sash bottom rail.",
    )

    # Latch lever pivots on the latch base plate
    ctx.allow_overlap(
        "latch", "sliding_sash",
        elem_a="latch_lever", elem_b="latch_base",
        reason="Latch lever pivots on the latch base plate (seated pivot hardware).",
    )
    # Latch lever pivot seated in sash stile and engages frame mullion
    ctx.allow_overlap(
        "latch", "sliding_sash",
        elem_a="latch_lever", elem_b="sash_frame",
        reason="Latch lever pivot is seated in the sash stile (captured pivot pin).",
    )
    ctx.allow_overlap(
        "latch", "frame",
        elem_a="latch_lever", elem_b="frame_shell",
        reason="Latch lever engages the frame mullion when locked at the meeting rail.",
    )

    # --- Structure: three-panel proportions ---

    f_aabb = ctx.part_world_aabb(frame)
    f_w = f_aabb[1][0] - f_aabb[0][0]
    f_h = f_aabb[1][2] - f_aabb[0][2]
    ctx.check(
        "frame wider than tall (slider proportions)",
        f_w > f_h,
        details=f"width={f_w:.3f}, height={f_h:.3f}",
    )

    # Sill near z=0
    ctx.check(
        "frame sill near z=0",
        abs(f_aabb[0][2]) < 0.015 and f_h > 0.8,
        details=f"z_min={f_aabb[0][2]:.3f}, height={f_h:.3f}",
    )

    # Center pane wider than left pane
    lg_aabb = ctx.part_element_world_aabb(frame, elem="left_fixed_glass")
    cg_aabb = ctx.part_element_world_aabb(frame, elem="center_fixed_glass")
    if lg_aabb and cg_aabb:
        lg_w = lg_aabb[1][0] - lg_aabb[0][0]
        cg_w = cg_aabb[1][0] - cg_aabb[0][0]
        ctx.check(
            "center pane wider than left pane",
            cg_w > lg_w + 0.10,
            details=f"left_w={lg_w:.3f}, center_w={cg_w:.3f}",
        )

    # Sill lip extends beyond frame depth (+Y)
    ctx.check(
        "sill lip extends beyond frame depth",
        f_aabb[1][1] > FRAME_DEPTH / 2.0 + SILL_LIP_EXT * 0.7,
        details=f"frame y_max={f_aabb[1][1]:.4f}, expected>{FRAME_DEPTH / 2.0 + SILL_LIP_EXT * 0.7:.4f}",
    )

    # --- Closed pose (q=0): sash in right section ---
    with ctx.pose({j_slide: 0.0}):
        s_aabb = ctx.part_world_aabb(sash)
        s_cx = (s_aabb[0][0] + s_aabb[1][0]) / 2.0
        ctx.check(
            "sash in right section when closed",
            s_cx > center_x1 - 0.01,
            details=f"sash_cx={s_cx:.3f}, center_right={center_x1:.3f}",
        )
        # Sash within frame opening in Z
        ctx.check(
            "sash within frame height when closed",
            s_aabb[0][2] >= -0.005 and s_aabb[1][2] <= WIN_H + 0.005,
            details=f"sash z=({s_aabb[0][2]:.3f}, {s_aabb[1][2]:.3f})",
        )
        rest_cx = s_cx

    # --- Open pose: sash slides left ---
    travel = MAX_TRAVEL * 0.85
    with ctx.pose({j_slide: travel}):
        s_aabb = ctx.part_world_aabb(sash)
        s_cx = (s_aabb[0][0] + s_aabb[1][0]) / 2.0
        ctx.check(
            "sash slides left when opened",
            s_cx < rest_cx - travel * 0.8,
            details=f"rest_cx={rest_cx:.3f}, open_cx={s_cx:.3f}, travel={travel:.3f}",
        )
        # Sash retained in frame (Z overlap)
        ctx.expect_overlap(
            sash, frame, axes="z", min_overlap=0.05,
            name="sash retained in frame when open",
        )

    # --- Latch rotation ---
    with ctx.pose({j_slide: 0.0, j_latch: 0.0}):
        latch_aabb_0 = ctx.part_world_aabb(latch)
    with ctx.pose({j_slide: 0.0, j_latch: 1.0}):
        latch_aabb_1 = ctx.part_world_aabb(latch)

    if latch_aabb_0 and latch_aabb_1:
        cx0 = (latch_aabb_0[0][0] + latch_aabb_0[1][0]) / 2.0
        cx1 = (latch_aabb_1[0][0] + latch_aabb_1[1][0]) / 2.0
        cy0 = (latch_aabb_0[0][1] + latch_aabb_0[1][1]) / 2.0
        cy1 = (latch_aabb_1[0][1] + latch_aabb_1[1][1]) / 2.0
        ctx.check(
            "latch rotates when actuated",
            abs(cx0 - cx1) > 0.004 or abs(cy0 - cy1) > 0.004,
            details=f"latch center ({cx0:.4f},{cy0:.4f}) -> ({cx1:.4f},{cy1:.4f})",
        )

    # --- Rollers at sash bottom ---
    rl_aabb = ctx.part_element_world_aabb(sash, elem="roller_left")
    rr_aabb = ctx.part_element_world_aabb(sash, elem="roller_right")
    sf_aabb = ctx.part_element_world_aabb(sash, elem="sash_frame")
    if rl_aabb and sf_aabb:
        ctx.check(
            "left roller at sash bottom",
            rl_aabb[0][2] < sf_aabb[0][2] + 0.006,
            details=f"roller_z_min={rl_aabb[0][2]:.4f}, sash_z_min={sf_aabb[0][2]:.4f}",
        )
    if rr_aabb and sf_aabb:
        ctx.check(
            "right roller at sash bottom",
            rr_aabb[0][2] < sf_aabb[0][2] + 0.006,
            details=f"roller_z_min={rr_aabb[0][2]:.4f}, sash_z_min={sf_aabb[0][2]:.4f}",
        )

    # --- Rollers spaced apart (two distinct blocks) ---
    if rl_aabb and rr_aabb:
        rl_cx = (rl_aabb[0][0] + rl_aabb[1][0]) / 2.0
        rr_cx = (rr_aabb[0][0] + rr_aabb[1][0]) / 2.0
        ctx.check(
            "rollers spaced apart horizontally",
            abs(rr_cx - rl_cx) > SASH_W * 0.5,
            details=f"left_cx={rl_cx:.3f}, right_cx={rr_cx:.3f}",
        )

    # --- Joint type checks ---
    ctx.check(
        "frame_to_sash is prismatic",
        j_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={j_slide.articulation_type}",
    )
    ctx.check(
        "sash_to_latch is revolute",
        j_latch.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={j_latch.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
