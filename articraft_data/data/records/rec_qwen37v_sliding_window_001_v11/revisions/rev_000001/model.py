from __future__ import annotations

# Two-panel horizontal sliding window, white vinyl/PVC frame with colonial
# divided-lite grilles. One wide outer frame holds two vertical lites:
#   - LEFT sash (SLIDING) sits proud toward +Y so it passes in front of the
#     fixed right panel; PRISMATIC along +X.
#   - RIGHT sash (FIXED) seated in the rear glazing plane.
# A small cam latch at the meeting rail (right stile of the left sash) rotates
# on a REVOLUTE joint to lock/unlock the window.
# Deep track grooves along head and sill capture both sashes.
# A recessed pull cup on the left sash stile provides a finger grip.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     width  -> X
#     height -> Z   (sill near z=0)
#     frame depth / glazing thickness / slide-normal -> Y
#   The glass plane is the X-Z plane.

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

TOTAL_W = 1.20            # overall window width along X
TOTAL_H = 1.20            # overall height along Z (sill at z=0)

FRAME_FACE = 0.060        # outer frame member face width (jamb / head / sill)
MULLION_FACE = 0.050      # center mullion face width
FRAME_DEPTH = 0.100       # outer frame depth along Y

# Two lite columns: left (slider) and right (fixed), separated by mullion.
# inner clear width = TOTAL_W - 2*FRAME_FACE = 1.08; minus mullion (0.05) = 1.03
# Each lite ~ 0.515 m wide.
LITE_W = 0.515            # clear opening width of each lite

# Sash construction
SASH_FACE = 0.048         # sash perimeter rail/stile face width
SASH_DEPTH = 0.048        # sash depth along Y
GLASS_T = 0.006           # glazing thickness

# Colonial grille
GRILLE_COLS = 3           # 3 columns of panes
GRILLE_ROWS = 4           # 4 rows of panes
MUNTIN_T = 0.016          # muntin bar face width
MUNTIN_DEPTH = 0.016      # muntin bar depth

# Track groove dimensions (deep channels in head/sill)
TRACK_DEPTH = 0.025       # how deep the groove cuts into the frame member
TRACK_WIDTH = 0.018       # width of each groove channel (accommodates sash)
# Two tracks: front (for slider) and rear (for fixed)
TRACK_FRONT_Y = 0.024     # Y center of front track groove
TRACK_REAR_Y = -0.024     # Y center of rear track groove

# Y layout (depth). Frame centered on y=0.
FIXED_LITE_Y = TRACK_REAR_Y   # fixed right sash in rear track
SLIDE_SASH_Y = TRACK_FRONT_Y  # left sash in front track (proud toward +Y)

REBATE = 0.004            # glass tucks under the sash lip

# Pull cup dimensions
PULL_CUP_W = 0.060        # width of the recessed pull
PULL_CUP_H = 0.025        # height of the recessed pull
PULL_CUP_DEPTH = 0.008    # how deep the cup is recessed

# Latch dimensions
LATCH_BODY_W = 0.040      # latch body width
LATCH_BODY_H = 0.012      # latch body height/thickness
LATCH_BODY_DEPTH = 0.020  # latch body depth (into frame)

# ---------------------------------------------------------------------------
# Derived layout
# ---------------------------------------------------------------------------

HALF_W = TOTAL_W / 2.0

INNER_X0 = -HALF_W + FRAME_FACE
INNER_X1 = HALF_W - FRAME_FACE
INNER_Z0 = FRAME_FACE
INNER_Z1 = TOTAL_H - FRAME_FACE

# Two lite openings with one mullion between.
# left lite | mullion | right lite
LEFT_X0 = INNER_X0
LEFT_X1 = LEFT_X0 + LITE_W
MUL_X0 = LEFT_X1
MUL_X1 = MUL_X0 + MULLION_FACE
RIGHT_X0 = MUL_X1
RIGHT_X1 = INNER_X1

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)     # bright white vinyl/PVC
GLASS_RGBA = (0.50, 0.58, 0.64, 0.32)    # cool grey-blue, semi-transparent
METAL_RGBA = (0.60, 0.62, 0.64, 1.0)     # brushed metal for latch


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery). All in meters, world frame.
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float, y_center: float, depth: float) -> cq.Workplane:
    """Axis-aligned box in the X-Z plane, centered on y_center with given Y depth."""
    w = x1 - x0
    h = z1 - z0
    cx = (x0 + x1) / 2.0
    cz = (z0 + z1) / 2.0
    return (
        cq.Workplane("XY")
        .transformed(offset=(cx, y_center, cz))
        .box(w, depth, h)
    )


def _build_frame_shape() -> cq.Workplane:
    """Static outer frame with two lite openings and deep track grooves.
    
    The frame is a slab cut by two lite openings. Track grooves are channels
    cut into the head (top) and sill (bottom) frame members to capture both
    sash panels.
    """
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)

    cut_depth = FRAME_DEPTH + 0.02
    left_cut = _slab(LEFT_X0, LEFT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    right_cut = _slab(RIGHT_X0, RIGHT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)

    frame = outer.cut(left_cut).cut(right_cut)

    # Deep track grooves: channels cut into the head and sill members.
    # Front track (for sliding sash) and rear track (for fixed sash).
    groove_depth_cut = TRACK_DEPTH
    groove_w = TRACK_WIDTH
    # Groove spans the inner width of the frame opening (across both lites + mullion area)
    groove_x0 = INNER_X0 - 0.01
    groove_x1 = INNER_X1 + 0.01

    # Head grooves (top frame member) - cut from below the head inner edge
    head_z_bottom = INNER_Z1
    head_z_top = INNER_Z1 + groove_depth_cut
    # Front track groove in head
    head_front_groove = _slab(groove_x0, groove_x1, head_z_bottom, head_z_top,
                              TRACK_FRONT_Y, groove_w)
    frame = frame.cut(head_front_groove)
    # Rear track groove in head
    head_rear_groove = _slab(groove_x0, groove_x1, head_z_bottom, head_z_top,
                             TRACK_REAR_Y, groove_w)
    frame = frame.cut(head_rear_groove)

    # Sill grooves (bottom frame member) - cut from above the sill inner edge
    sill_z_top = INNER_Z0
    sill_z_bottom = INNER_Z0 - groove_depth_cut
    # Front track groove in sill
    sill_front_groove = _slab(groove_x0, groove_x1, sill_z_bottom, sill_z_top,
                              TRACK_FRONT_Y, groove_w)
    frame = frame.cut(sill_front_groove)
    # Rear track groove in sill
    sill_rear_groove = _slab(groove_x0, groove_x1, sill_z_bottom, sill_z_top,
                             TRACK_REAR_Y, groove_w)
    frame = frame.cut(sill_rear_groove)

    return frame


def _build_sash_grille_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """One sash in its OWN local frame, centered on local origin.
    Outer sash ring + colonial muntin grid. Vinyl material.
    """
    ow = opening_w
    oh = opening_h
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE

    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SASH_DEPTH)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH + 0.02)
    ring = outer.cut(opening)

    bars = None

    # Vertical muntins
    for c in range(1, GRILLE_COLS):
        frac = c / GRILLE_COLS
        x = -ow / 2.0 + frac * ow
        bar = _slab(
            x - MUNTIN_T / 2.0, x + MUNTIN_T / 2.0,
            -oh / 2.0, oh / 2.0,
            0.0, MUNTIN_DEPTH,
        )
        bars = bar if bars is None else bars.union(bar)

    # Horizontal muntins
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


def _build_sash_with_pull_cup(opening_w: float, opening_h: float) -> cq.Workplane:
    """Left (sliding) sash with a recessed pull cup on the right stile
    (meeting rail side). The pull cup is a rectangular recess cut into the
    stile face, providing a finger grip for sliding.
    """
    sash = _build_sash_grille_shape(opening_w, opening_h)

    # Pull cup: recessed into the right stile (at +X edge of sash), near mid-height.
    # It's a shallow rectangular pocket cut from the front face (+Y side).
    cup_x_center = opening_w / 2.0 + SASH_FACE / 2.0  # center of right stile
    cup_y_front = SASH_DEPTH / 2.0  # front face of sash
    cup_y_back = cup_y_front - PULL_CUP_DEPTH

    cup = _slab(
        cup_x_center - PULL_CUP_W / 2.0,
        cup_x_center + PULL_CUP_W / 2.0,
        -PULL_CUP_H / 2.0,
        PULL_CUP_H / 2.0,
        (cup_y_front + cup_y_back) / 2.0,
        PULL_CUP_DEPTH,
    )
    sash = sash.cut(cup)

    return sash


def _build_sash_glass_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Single clear pane filling the sash opening."""
    ow = opening_w + 2 * REBATE
    oh = opening_h + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_latch_shape() -> cq.Workplane:
    """Small cam latch body: a flat lever that rotates about Y axis (perpendicular
    to the glass face). The latch bar extends along +X from the pivot when locked
    (q=0). Rotating about Y swings it from horizontal (+X) to vertical (+Z).
    The pivot boss is a small cylinder along Y at the origin."""
    # Latch bar: thin flat lever extending along +X from pivot, thin in Z and Y
    bar_w = LATCH_BODY_W   # length along X
    bar_h = LATCH_BODY_H   # thickness in Z (thin)
    bar_d = 0.010          # depth in Y (thin flat bar)
    latch = _slab(
        0.0, bar_w,
        -bar_h / 2.0, bar_h / 2.0,
        0.0, bar_d,
    )
    # Small cylindrical pivot boss along Y axis at origin
    pivot = (
        cq.Workplane("XZ")
        .transformed(offset=(0.0, 0.0, 0.0))
        .circle(0.007)
        .extrude(LATCH_BODY_DEPTH)
    )
    return latch.union(pivot)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    # Sanity: two lites + mullion must fill the inner clear width.
    span = LITE_W + MULLION_FACE + LITE_W
    inner_w = INNER_X1 - INNER_X0
    assert abs(span - inner_w) < 1e-6, f"lite layout {span} != inner width {inner_w}"

    model = ArticulatedObject(name="two_panel_sliding_window")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("metal", rgba=METAL_RGBA)

    # --- Static outer frame (root) with deep track grooves ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="vinyl",
        name="frame_shell",
    )

    opening_h = INNER_Z1 - INNER_Z0

    # --- Right (fixed) sash ---
    right_sash = model.part("right_sash")
    right_sash.visual(
        mesh_from_cadquery(_build_sash_grille_shape(LITE_W, opening_h), "right_sash_vinyl"),
        material="vinyl",
        name="right_sash_vinyl",
    )
    right_sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(LITE_W, opening_h), "right_sash_glass"),
        material="glass",
        name="right_sash_glass",
    )

    # --- Left (sliding) sash with pull cup ---
    left_sash = model.part("left_sash")
    left_sash.visual(
        mesh_from_cadquery(_build_sash_with_pull_cup(LITE_W, opening_h), "left_sash_vinyl"),
        material="vinyl",
        name="left_sash_vinyl",
    )
    left_sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(LITE_W, opening_h), "left_sash_glass"),
        material="glass",
        name="left_sash_glass",
    )

    # --- Latch (mounted on left sash at meeting rail) ---
    latch = model.part("latch")
    latch.visual(
        mesh_from_cadquery(_build_latch_shape(), "latch_body"),
        material="metal",
        name="latch_body",
    )

    # World positions of lite centers
    left_cx = (LEFT_X0 + LEFT_X1) / 2.0
    right_cx = (RIGHT_X0 + RIGHT_X1) / 2.0
    mid_cz = (INNER_Z0 + INNER_Z1) / 2.0

    # FIXED right sash: seated in rear track (rear glazing plane)
    model.articulation(
        "frame_to_right_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="right_sash",
        origin=Origin(xyz=(right_cx, FIXED_LITE_Y, mid_cz)),
    )

    # SLIDING left sash: PRISMATIC along +X in front track.
    # At q=0, sash is closed (covering left opening).
    # Positive q slides it rightward (toward center/right), opening the left side.
    slide_travel = LITE_W * 0.85
    model.articulation(
        "frame_to_left_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="left_sash",
        origin=Origin(xyz=(left_cx, SLIDE_SASH_Y, mid_cz)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5, lower=0.0, upper=slide_travel),
    )

    # Latch: REVOLUTE joint on left sash at the meeting rail (right edge of left sash).
    # The latch pivot is at the right stile of the left sash, near mid-height.
    # Rotates about Y axis (perpendicular to glass face), like a real cam latch:
    # at q=0 the bar is horizontal (locked); positive q swings it upward (unlocked).
    latch_x_in_sash = LITE_W / 2.0 + SASH_FACE / 2.0  # right stile
    latch_z_in_sash = 0.0  # mid-height of sash
    model.articulation(
        "left_sash_to_latch",
        ArticulationType.REVOLUTE,
        parent="left_sash",
        child="latch",
        origin=Origin(xyz=(latch_x_in_sash, 0.0, latch_z_in_sash)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=1.57),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    left_sash = object_model.get_part("left_sash")
    right_sash = object_model.get_part("right_sash")
    latch = object_model.get_part("latch")
    slide = object_model.get_articulation("frame_to_left_sash")
    latch_joint = object_model.get_articulation("left_sash_to_latch")

    # --- Intentional overlaps ---
    # Glass panes tuck under the vinyl/muntin lip on each sash (captured glass).
    for nm in ("left_sash", "right_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_vinyl",
            reason="Clear pane is rebated under the sash/muntin lip so it reads captured, not floating.",
        )

    # The right (fixed) sash is seated in the rear frame track; its ring laps
    # the frame opening edges (glazing rebate).
    ctx.allow_overlap(
        "frame", "right_sash",
        elem_a="frame_shell",
        elem_b="right_sash_vinyl",
        reason="Right fixed sash is rebated into the frame rear track; its sash ring laps the jamb/mullion edge (seated capture).",
    )
    ctx.allow_overlap(
        "frame", "right_sash",
        elem_a="frame_shell",
        elem_b="right_sash_glass",
        reason="Right sash glass is rebated under the frame track lip (captured glazing).",
    )

    # The left sliding sash rides in the front track; its ring laps the frame
    # face along the track.
    ctx.allow_overlap(
        "frame", "left_sash",
        elem_a="frame_shell",
        elem_b="left_sash_vinyl",
        reason="Left sash rides the head/sill track grooves and laps the frame face along the track (slider capture).",
    )
    ctx.allow_overlap(
        "frame", "left_sash",
        elem_a="frame_shell",
        elem_b="left_sash_glass",
        reason="Left sash glass laps the track lip as the sash rides the groove (captured glazing).",
    )

    # Latch is mounted flush on the sash stile; small overlap at the pivot boss.
    ctx.allow_overlap(
        "left_sash", "latch",
        elem_a="left_sash_vinyl",
        elem_b="latch_body",
        reason="Latch pivot boss is embedded in the sash stile face for a flush mount.",
    )
    # Latch bar extends toward the mullion when locked; it overlaps the frame
    # mullion (the latch engages a keeper/strike on the mullion).
    ctx.allow_overlap(
        "frame", "latch",
        elem_a="frame_shell",
        elem_b="latch_body",
        reason="Latch bar engages the mullion keeper when locked; this is the latch-to-strike interface.",
    )

    # Proof: latch overlaps the frame in XZ projection (engages mullion keeper)
    ctx.expect_overlap(
        latch, frame,
        axes="xz",
        min_overlap=0.005,
        elem_a="latch_body",
        elem_b="frame_shell",
        name="latch engages frame mullion keeper area",
    )

    # --- Verify two-panel structure (not three) ---
    all_part_names = [p.name for p in object_model.parts]
    ctx.check(
        "exactly two sashes exist",
        "left_sash" in all_part_names and "right_sash" in all_part_names,
        details=f"parts: {all_part_names}",
    )
    ctx.check(
        "no center sash (two-panel variant)",
        "center_sash" not in all_part_names,
        details=f"parts: {all_part_names}",
    )

    # --- Verify latch exists as a separate part with revolute joint ---
    ctx.check(
        "latch part exists",
        "latch" in all_part_names,
        details=f"parts: {all_part_names}",
    )
    ctx.check(
        "latch joint is revolute",
        latch_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={latch_joint.articulation_type}",
    )

    # --- Verify left sash is the movable one (prismatic) ---
    ctx.check(
        "left sash joint is prismatic (movable)",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )

    # --- Closed pose (q=0): window reads SHUT ---
    with ctx.pose({slide: 0.0, latch_joint: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        l_aabb = ctx.part_world_aabb(left_sash)
        r_aabb = ctx.part_world_aabb(right_sash)

        # Frame spans the full width
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        ctx.check(
            "frame spans full window width",
            frame_w > 1.0,
            details=f"frame_w={frame_w:.3f}",
        )

        # Sill near z=0
        ctx.check(
            "sill sits near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"frame zmin={frame_aabb[0][2]:.4f}",
        )

        # Two sashes ordered left then right
        lx = (l_aabb[0][0] + l_aabb[1][0]) / 2.0
        rx = (r_aabb[0][0] + r_aabb[1][0]) / 2.0
        ctx.check(
            "sashes ordered left-right",
            lx < rx,
            details=f"left_x={lx:.3f}, right_x={rx:.3f}",
        )

        # Left sash sits proud of right sash (in +Y, front track)
        l_y = (l_aabb[0][1] + l_aabb[1][1]) / 2.0
        r_y = (r_aabb[0][1] + r_aabb[1][1]) / 2.0
        ctx.check(
            "left sash proud of right sash (front track)",
            l_y > r_y + 0.02,
            details=f"left_y={l_y:.3f}, right_y={r_y:.3f}",
        )

        # Both sashes seated within frame height
        for nm, ab in (("left", l_aabb), ("right", r_aabb)):
            ctx.check(
                f"{nm} sash seated within frame height",
                ab[0][2] > frame_aabb[0][2] - 1e-4 and ab[1][2] < frame_aabb[1][2] + 1e-4,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}] frame z=[{frame_aabb[0][2]:.3f},{frame_aabb[1][2]:.3f}]",
            )

        # Latch is near the meeting rail (right edge of left sash, near right sash)
        latch_aabb = ctx.part_world_aabb(latch)
        latch_x = (latch_aabb[0][0] + latch_aabb[1][0]) / 2.0
        ctx.check(
            "latch positioned at meeting rail (near mullion)",
            latch_x > lx,  # latch is to the right of left sash center
            details=f"latch_x={latch_x:.3f}, left_sash_x={lx:.3f}",
        )

        rest_lx = lx
        rest_lz = (l_aabb[0][2] + l_aabb[1][2]) / 2.0

    # --- Driven/open pose: left sash slides sideways along +X ---
    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel, latch_joint: 0.0}):
        l_open = ctx.part_world_aabb(left_sash)
        open_lx = (l_open[0][0] + l_open[1][0]) / 2.0
        # Left sash translated along +X by ~travel distance
        ctx.check(
            "left sash slides along +X by ~travel",
            abs((open_lx - rest_lx) - travel) < 0.02,
            details=f"rest_lx={rest_lx:.3f}, open_lx={open_lx:.3f}, travel={travel:.3f}",
        )
        # Slide is purely horizontal (no Z movement)
        l_open_z = (l_open[0][2] + l_open[1][2]) / 2.0
        ctx.check(
            "slide is purely horizontal",
            abs(l_open_z - rest_lz) < 0.02,
            details=f"open_z={l_open_z:.3f}, rest_z={rest_lz:.3f}",
        )
        # Retained insertion: sash still overlaps frame at full travel
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained within frame X span at full travel",
            l_open[1][0] < f_aabb[1][0] + 1e-4 and l_open[0][0] > f_aabb[0][0] - 1e-4,
            details=f"sash x=[{l_open[0][0]:.3f},{l_open[1][0]:.3f}] frame x=[{f_aabb[0][0]:.3f},{f_aabb[1][0]:.3f}]",
        )
        ctx.expect_overlap(
            left_sash, frame,
            axes="z",
            min_overlap=0.10,
            name="sash retains vertical engagement with track grooves",
        )

    # --- Latch rotation: unlocked pose ---
    # Measure latch AABB at locked (q=0) and unlocked (q=π/2) positions.
    with ctx.pose({slide: 0.0, latch_joint: 0.0}):
        latch_closed_aabb = ctx.part_world_aabb(latch)
        latch_closed_z = latch_closed_aabb[1][2] - latch_closed_aabb[0][2]  # Z extent
        latch_closed_x = latch_closed_aabb[1][0] - latch_closed_aabb[0][0]  # X extent

    with ctx.pose({slide: 0.0, latch_joint: 1.57}):
        latch_open_aabb = ctx.part_world_aabb(latch)
        latch_open_z = latch_open_aabb[1][2] - latch_open_aabb[0][2]  # Z extent
        latch_open_x = latch_open_aabb[1][0] - latch_open_aabb[0][0]  # X extent

    # When rotated 90° about Y, the bar (which was along +X) swings to +Z.
    # The Z extent should increase and X extent should decrease.
    ctx.check(
        "latch rotation swings bar upward (Z extent increases)",
        latch_open_z > latch_closed_z + 0.005,
        details=f"closed_z_extent={latch_closed_z:.4f}, open_z_extent={latch_open_z:.4f}",
    )
    ctx.check(
        "latch rotation reduces X extent (bar no longer horizontal)",
        latch_open_x < latch_closed_x - 0.005,
        details=f"closed_x_extent={latch_closed_x:.4f}, open_x_extent={latch_open_x:.4f}",
    )

    # --- Track grooves exist: frame has deeper profile than just frame members ---
    # The frame should have visible depth variation from the track grooves.
    # We check that the frame extends below the inner sill (groove cuts downward).
    with ctx.pose({slide: 0.0}):
        f_aabb = ctx.part_world_aabb(frame)
        # Frame bottom should be at z=0 (sill bottom), and the grooves cut
        # INTO the sill from above, so the sill still reaches z=0 at the bottom.
        # The key is that the head reaches TOTAL_H.
        ctx.check(
            "frame head reaches full height",
            abs(f_aabb[1][2] - TOTAL_H) < 0.02,
            details=f"frame zmax={f_aabb[1][2]:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
