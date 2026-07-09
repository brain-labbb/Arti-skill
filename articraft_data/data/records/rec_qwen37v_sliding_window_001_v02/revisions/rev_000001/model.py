from __future__ import annotations

# Three-panel vertical-sliding window variant. White vinyl/PVC frame with
# colonial divided-lite grilles. Three vertical columns: narrow left fixed lite,
# wider center fixed lite, and a right column split by a horizontal meeting
# rail into an upper fixed lite and a lower sash that slides upward on a
# vertical prismatic joint. Two tiny roller blocks at the bottom of the moving
# sash and a visible overlap stile extending above the sash on the inner edge
# where the panes cross.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     - width  -> X
#     - height -> Z   (sill near z=0)
#     - frame depth / glazing thickness / slide-normal -> Y
#   The glass plane is the X-Z plane. The window reads SHUT at q=0; driving the
#   prismatic joint slides the lower sash upward (+Z), staying retained in the
#   frame track.

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

# ---------------------------------------------------------------------------
# Absolute dimensions (meters)
# ---------------------------------------------------------------------------

TOTAL_W = 3.00            # overall window width along X
TOTAL_H = 1.50            # overall height along Z (sill at z=0, head at z=TOTAL_H)

FRAME_FACE = 0.070        # outer frame member face width (jamb / head / sill)
MULLION_FACE = 0.060      # intermediate mullion face width
FRAME_DEPTH = 0.110       # outer frame depth along Y (chunky vinyl box section)

# Three lite columns: left narrow | mullion | center wide | mullion | right narrow
LEFT_LITE_W = 0.70        # clear opening width of left lite
CENTER_LITE_W = 1.34      # clear opening width of center lite (wider)
RIGHT_LITE_W = 0.70       # clear opening width of right column

# Sash construction
SASH_FACE = 0.025         # sash perimeter rail/stile face width (in-plane)
SASH_DEPTH = 0.055        # sash depth along Y
GLASS_T = 0.008           # glazing thickness along Y

# Colonial grille (divided lite): muntin bars
MUNTIN_T = 0.020          # muntin bar face width (in-plane)
MUNTIN_DEPTH = 0.020      # muntin bar depth along Y

# Y layout (depth). Frame box centered on y=0. Fixed lites sit in the rear
# glazing plane; the lower sliding sash sits proud toward +Y so it can slide
# upward in front of the upper right lite without colliding.
FIXED_LITE_Y = -0.020     # fixed lites: rear glazing plane center (Y)
SLIDE_SASH_Y = 0.052      # lower sash sits proud toward +Y (passes in front)

REBATE = 0.005            # glass tucks under the sash/muntin lip by this much

# Meeting rail: horizontal bar dividing the right column into upper and lower
MEETING_RAIL_H = 0.045    # height of the horizontal meeting rail bar
SPLIT_FRAC = 0.55         # lower sash gets 55% of opening height

# Roller blocks at the bottom of the sliding sash
ROLLER_W = 0.030          # roller block width along X
ROLLER_D = 0.018          # roller block depth along Y
ROLLER_H = 0.015          # roller block height along Z

# Overlap stile: vertical fin on the inner edge of the sash that extends above
# the top rail, overlapping the meeting rail / upper lite area
STILE_W = 0.030           # stile face width along X
STILE_D = 0.025           # stile depth along Y
STILE_EXTEND = 0.055      # how far the stile extends above the sash top rail

# Slide travel (upward)
SLIDE_TRAVEL = 0.45       # meters of upward travel

# ---------------------------------------------------------------------------
# Derived layout
# ---------------------------------------------------------------------------

HALF_W = TOTAL_W / 2.0

# Inner clear region (inside the outer head/sill/jambs)
INNER_X0 = -HALF_W + FRAME_FACE
INNER_X1 = HALF_W - FRAME_FACE
INNER_Z0 = FRAME_FACE
INNER_Z1 = TOTAL_H - FRAME_FACE

# Three lite columns laid left -> center -> right with two mullions between.
LEFT_X0 = INNER_X0
LEFT_X1 = LEFT_X0 + LEFT_LITE_W
MUL0_X0 = LEFT_X1
MUL0_X1 = MUL0_X0 + MULLION_FACE
CENTER_X0 = MUL0_X1
CENTER_X1 = CENTER_X0 + CENTER_LITE_W
MUL1_X0 = CENTER_X1
MUL1_X1 = MUL1_X0 + MULLION_FACE
RIGHT_X0 = MUL1_X1
RIGHT_X1 = INNER_X1

# Right column Z split (meeting rail divides it)
OPENING_H = INNER_Z1 - INNER_Z0
MEETING_RAIL_Z0 = INNER_Z0 + SPLIT_FRAC * OPENING_H
MEETING_RAIL_Z1 = MEETING_RAIL_Z0 + MEETING_RAIL_H

RIGHT_LOWER_Z0 = INNER_Z0
RIGHT_LOWER_Z1 = MEETING_RAIL_Z0
RIGHT_UPPER_Z0 = MEETING_RAIL_Z1
RIGHT_UPPER_Z1 = INNER_Z1

LOWER_SASH_OPENING_H = RIGHT_LOWER_Z1 - RIGHT_LOWER_Z0
UPPER_RIGHT_OPENING_H = RIGHT_UPPER_Z1 - RIGHT_UPPER_Z0

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)     # bright white vinyl/PVC
GLASS_RGBA = (0.50, 0.58, 0.64, 0.32)    # cool grey-blue, semi-transparent
ROLLER_RGBA = (0.28, 0.28, 0.30, 1.0)    # dark grey nylon rollers


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery). All authored directly in meters, world frame.
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float, y_center: float, depth: float) -> cq.Workplane:
    """Axis-aligned box spanning [x0,x1] x [z0,z1] in the X-Z plane, centered on
    y_center with the given Y depth."""
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
    """Static outer frame: a full slab cut by the three lite openings (right
    column split into upper and lower by the meeting rail), leaving head, sill,
    two jambs, two intermediate mullions, and the meeting rail as one solid."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)

    cut_depth = FRAME_DEPTH + 0.02  # through-cut clearance in Y
    left_cut = _slab(LEFT_X0, LEFT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    center_cut = _slab(CENTER_X0, CENTER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    right_upper_cut = _slab(RIGHT_X0, RIGHT_X1, RIGHT_UPPER_Z0, RIGHT_UPPER_Z1, 0.0, cut_depth)
    right_lower_cut = _slab(RIGHT_X0, RIGHT_X1, RIGHT_LOWER_Z0, RIGHT_LOWER_Z1, 0.0, cut_depth)

    return outer.cut(left_cut).cut(center_cut).cut(right_upper_cut).cut(right_lower_cut)


def _build_sash_grille_shape(
    opening_w: float,
    opening_h: float,
    grille_cols: int = 4,
    grille_rows: int = 5,
) -> cq.Workplane:
    """One sash built in its OWN local frame, centered on local origin.
    Outer sash slab cut by the clear opening, then the colonial muntin grid
    (vertical + horizontal bars) unioned back in across the opening.
    Returns the vinyl (frame + muntins) workplane. Glass is built separately.
    """
    ow = opening_w
    oh = opening_h
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE

    # Outer sash slab.
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SASH_DEPTH)
    # Hollow it: cut the clear opening (glass region).
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH + 0.02)
    ring = outer.cut(opening)

    # Colonial muntin grid across the opening.
    bars = None

    # Vertical muntins.
    for c in range(1, grille_cols):
        frac = c / grille_cols
        x = -ow / 2.0 + frac * ow
        bar = _slab(
            x - MUNTIN_T / 2.0, x + MUNTIN_T / 2.0,
            -oh / 2.0, oh / 2.0,
            0.0, MUNTIN_DEPTH,
        )
        bars = bar if bars is None else bars.union(bar)

    # Horizontal muntins.
    for r in range(1, grille_rows):
        frac = r / grille_rows
        z = -oh / 2.0 + frac * oh
        bar = _slab(
            -ow / 2.0, ow / 2.0,
            z - MUNTIN_T / 2.0, z + MUNTIN_T / 2.0,
            0.0, MUNTIN_DEPTH,
        )
        bars = bar if bars is None else bars.union(bar)

    return ring if bars is None else ring.union(bars)


def _build_sash_glass_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Single clear pane filling the sash opening, in the same sash-local frame."""
    ow = opening_w + 2 * REBATE
    oh = opening_h + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_lower_sash_vinyl() -> cq.Workplane:
    """Lower sliding sash vinyl shape (sash ring + grille + roller blocks +
    overlap stile) in the sash-local frame centered on the sash opening center."""
    ow = RIGHT_LITE_W
    oh = LOWER_SASH_OPENING_H

    # Base sash ring + grille
    sash = _build_sash_grille_shape(ow, oh, grille_cols=3, grille_rows=4)

    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE

    # --- Roller blocks: two small blocks at the bottom of the sash on the back
    # face, where the sash rides the sill track. ---
    roller_z_center = -out_h / 2.0 + ROLLER_H / 2.0  # centered on sash bottom edge
    roller_y_center = -SASH_DEPTH / 2.0 - ROLLER_D / 4.0  # half-protruding from back face

    left_roller_x = -(ow / 2.0 - ROLLER_W / 2.0 - 0.010)
    right_roller_x = +(ow / 2.0 - ROLLER_W / 2.0 - 0.010)

    lr = _slab(
        left_roller_x - ROLLER_W / 2.0, left_roller_x + ROLLER_W / 2.0,
        roller_z_center - ROLLER_H / 2.0, roller_z_center + ROLLER_H / 2.0,
        roller_y_center, ROLLER_D,
    )
    rr = _slab(
        right_roller_x - ROLLER_W / 2.0, right_roller_x + ROLLER_W / 2.0,
        roller_z_center - ROLLER_H / 2.0, roller_z_center + ROLLER_H / 2.0,
        roller_y_center, ROLLER_D,
    )
    sash = sash.union(lr).union(rr)

    # --- Overlap stile: vertical fin on the inner (left) edge of the sash that
    # extends above the top rail, overlapping the meeting rail / upper lite. ---
    stile_x_center = -(ow / 2.0 + SASH_FACE / 2.0)  # on the left stile
    stile_z_bottom = out_h / 2.0 - STILE_W  # starts just inside the top rail
    stile_z_top = out_h / 2.0 + STILE_EXTEND  # extends above the sash
    stile_z_center = (stile_z_bottom + stile_z_top) / 2.0
    stile_h = stile_z_top - stile_z_bottom

    stile = _slab(
        stile_x_center - STILE_W / 2.0, stile_x_center + STILE_W / 2.0,
        stile_z_center - stile_h / 2.0, stile_z_center + stile_h / 2.0,
        0.0, STILE_D,
    )
    sash = sash.union(stile)

    return sash


def _build_lower_sash_rollers() -> cq.Workplane:
    """Two roller blocks as separate visuals (dark material) in the sash-local frame."""
    ow = RIGHT_LITE_W
    oh = LOWER_SASH_OPENING_H
    out_h = oh + 2 * SASH_FACE

    roller_z_center = -out_h / 2.0 + ROLLER_H / 2.0
    roller_y_center = -SASH_DEPTH / 2.0 - ROLLER_D / 4.0

    left_roller_x = -(ow / 2.0 - ROLLER_W / 2.0 - 0.010)
    right_roller_x = +(ow / 2.0 - ROLLER_W / 2.0 - 0.010)

    lr = _slab(
        left_roller_x - ROLLER_W / 2.0, left_roller_x + ROLLER_W / 2.0,
        roller_z_center - ROLLER_H / 2.0, roller_z_center + ROLLER_H / 2.0,
        roller_y_center, ROLLER_D,
    )
    rr = _slab(
        right_roller_x - ROLLER_W / 2.0, right_roller_x + ROLLER_W / 2.0,
        roller_z_center - ROLLER_H / 2.0, roller_z_center + ROLLER_H / 2.0,
        roller_y_center, ROLLER_D,
    )
    return lr.union(rr)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    # Sanity: the three lites + two mullions must fill the inner clear width.
    span = LEFT_LITE_W + MULLION_FACE + CENTER_LITE_W + MULLION_FACE + RIGHT_LITE_W
    inner_w = INNER_X1 - INNER_X0
    assert abs(span - inner_w) < 1e-6, f"lite layout {span} != inner width {inner_w}"

    model = ArticulatedObject(name="three_panel_vertical_sliding_window")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("roller", rgba=ROLLER_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="vinyl",
        name="frame_shell",
    )

    # --- Left fixed lite ---
    left_lite = model.part("left_lite")
    left_lite.visual(
        mesh_from_cadquery(
            _build_sash_grille_shape(LEFT_LITE_W, OPENING_H, grille_cols=3, grille_rows=5),
            "left_lite_vinyl",
        ),
        material="vinyl",
        name="left_lite_vinyl",
    )
    left_lite.visual(
        mesh_from_cadquery(_build_sash_glass_shape(LEFT_LITE_W, OPENING_H), "left_lite_glass"),
        material="glass",
        name="left_lite_glass",
    )

    # --- Center fixed lite (wider) ---
    center_lite = model.part("center_lite")
    center_lite.visual(
        mesh_from_cadquery(
            _build_sash_grille_shape(CENTER_LITE_W, OPENING_H, grille_cols=5, grille_rows=5),
            "center_lite_vinyl",
        ),
        material="vinyl",
        name="center_lite_vinyl",
    )
    center_lite.visual(
        mesh_from_cadquery(_build_sash_glass_shape(CENTER_LITE_W, OPENING_H), "center_lite_glass"),
        material="glass",
        name="center_lite_glass",
    )

    # --- Upper right fixed lite ---
    upper_right = model.part("upper_right_lite")
    upper_right.visual(
        mesh_from_cadquery(
            _build_sash_grille_shape(RIGHT_LITE_W, UPPER_RIGHT_OPENING_H, grille_cols=3, grille_rows=3),
            "upper_right_vinyl",
        ),
        material="vinyl",
        name="upper_right_vinyl",
    )
    upper_right.visual(
        mesh_from_cadquery(
            _build_sash_glass_shape(RIGHT_LITE_W, UPPER_RIGHT_OPENING_H),
            "upper_right_glass",
        ),
        material="glass",
        name="upper_right_glass",
    )

    # --- Lower sliding sash (with roller blocks and overlap stile) ---
    lower_sash = model.part("lower_sash")
    lower_sash.visual(
        mesh_from_cadquery(_build_lower_sash_vinyl(), "lower_sash_vinyl"),
        material="vinyl",
        name="lower_sash_vinyl",
    )
    lower_sash.visual(
        mesh_from_cadquery(_build_lower_sash_rollers(), "lower_sash_rollers"),
        material="roller",
        name="lower_sash_rollers",
    )
    lower_sash.visual(
        mesh_from_cadquery(
            _build_sash_glass_shape(RIGHT_LITE_W, LOWER_SASH_OPENING_H),
            "lower_sash_glass",
        ),
        material="glass",
        name="lower_sash_glass",
    )

    # --- World positions for articulation origins ---
    left_cx = (LEFT_X0 + LEFT_X1) / 2.0
    center_cx = (CENTER_X0 + CENTER_X1) / 2.0
    right_cx = (RIGHT_X0 + RIGHT_X1) / 2.0
    full_mid_cz = (INNER_Z0 + INNER_Z1) / 2.0
    upper_right_cz = (RIGHT_UPPER_Z0 + RIGHT_UPPER_Z1) / 2.0
    lower_sash_cz = (RIGHT_LOWER_Z0 + RIGHT_LOWER_Z1) / 2.0

    # FIXED: left lite seated in the rear glazing plane
    model.articulation(
        "frame_to_left_lite",
        ArticulationType.FIXED,
        parent="frame",
        child="left_lite",
        origin=Origin(xyz=(left_cx, FIXED_LITE_Y, full_mid_cz)),
    )

    # FIXED: center lite (wider) seated in the rear glazing plane
    model.articulation(
        "frame_to_center_lite",
        ArticulationType.FIXED,
        parent="frame",
        child="center_lite",
        origin=Origin(xyz=(center_cx, FIXED_LITE_Y, full_mid_cz)),
    )

    # FIXED: upper right lite seated in the rear glazing plane
    model.articulation(
        "frame_to_upper_right",
        ArticulationType.FIXED,
        parent="frame",
        child="upper_right_lite",
        origin=Origin(xyz=(right_cx, FIXED_LITE_Y, upper_right_cz)),
    )

    # PRISMATIC: lower sash slides upward along +Z. Joint origin at the sash
    # closed (rest) center, proud of the fixed lites in +Y. Positive q slides
    # the sash upward by SLIDE_TRAVEL meters. The sash stays retained in the
    # frame track at full travel.
    model.articulation(
        "frame_to_lower_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="lower_sash",
        origin=Origin(xyz=(right_cx, SLIDE_SASH_Y, lower_sash_cz)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.4, lower=0.0, upper=SLIDE_TRAVEL,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    left_lite = object_model.get_part("left_lite")
    center_lite = object_model.get_part("center_lite")
    upper_right = object_model.get_part("upper_right_lite")
    lower_sash = object_model.get_part("lower_sash")
    slide = object_model.get_articulation("frame_to_lower_sash")

    # --- Intentional overlaps ---
    # Glass panes tuck under the vinyl/muntin lip on each sash (captured glass).
    for nm in ("left_lite", "center_lite", "upper_right_lite"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass" if nm != "upper_right_lite" else "upper_right_glass",
            elem_b=f"{nm}_vinyl" if nm != "upper_right_lite" else "upper_right_vinyl",
            reason="Clear pane is rebated under the sash/muntin lip so it reads captured, not floating.",
        )
    ctx.allow_overlap(
        "lower_sash", "lower_sash",
        elem_a="lower_sash_glass",
        elem_b="lower_sash_vinyl",
        reason="Clear pane is rebated under the sash/muntin lip so it reads captured.",
    )
    # Roller blocks are seated in the sash bottom rail (intentional capture).
    ctx.allow_overlap(
        "lower_sash", "lower_sash",
        elem_a="lower_sash_rollers",
        elem_b="lower_sash_vinyl",
        reason="Roller blocks are seated in the sash bottom rail (captured hardware).",
    )
    # Fixed lites rebated into the frame opening (seated capture).
    ctx.allow_overlap(
        "frame", "left_lite",
        elem_a="frame_shell", elem_b="left_lite_vinyl",
        reason="Left fixed lite is rebated into the frame opening (seated capture).",
    )
    ctx.allow_overlap(
        "frame", "center_lite",
        elem_a="frame_shell", elem_b="center_lite_vinyl",
        reason="Center fixed lite is rebated into the frame opening (seated capture).",
    )
    ctx.allow_overlap(
        "frame", "upper_right_lite",
        elem_a="frame_shell", elem_b="upper_right_vinyl",
        reason="Upper right fixed lite is rebated into the frame opening (seated capture).",
    )
    # Glass rebated under frame opening lip for fixed lites.
    ctx.allow_overlap(
        "frame", "left_lite",
        elem_a="frame_shell", elem_b="left_lite_glass",
        reason="Left lite glass rebated under frame opening lip (captured glazing).",
    )
    ctx.allow_overlap(
        "frame", "center_lite",
        elem_a="frame_shell", elem_b="center_lite_glass",
        reason="Center lite glass rebated under frame opening lip (captured glazing).",
    )
    ctx.allow_overlap(
        "frame", "upper_right_lite",
        elem_a="frame_shell", elem_b="upper_right_glass",
        reason="Upper right lite glass rebated under frame opening lip (captured glazing).",
    )
    # Lower sash rides the frame track and laps the frame face (slider capture).
    ctx.allow_overlap(
        "frame", "lower_sash",
        elem_a="frame_shell", elem_b="lower_sash_vinyl",
        reason="Lower sash rides the frame track and laps the frame face (slider capture).",
    )
    ctx.allow_overlap(
        "frame", "lower_sash",
        elem_a="frame_shell", elem_b="lower_sash_glass",
        reason="Lower sash glass laps the frame track lip (captured glazing).",
    )
    ctx.allow_overlap(
        "frame", "lower_sash",
        elem_a="frame_shell", elem_b="lower_sash_rollers",
        reason="Roller blocks ride in the frame sill track (track engagement).",
    )
    # Overlap stile on the lower sash intentionally extends into the meeting rail
    # zone, overlapping the upper right lite bottom rail (visible overlap where
    # panes cross).
    ctx.allow_overlap(
        "lower_sash", "upper_right_lite",
        elem_a="lower_sash_vinyl", elem_b="upper_right_vinyl",
        reason="Overlap stile extends above the sash top rail into the meeting rail zone, overlapping the upper right lite (visible stile crossing).",
    )
    ctx.expect_overlap(
        lower_sash, upper_right,
        axes="z",
        elem_a="lower_sash_vinyl", elem_b="upper_right_vinyl",
        min_overlap=0.010,
        name="overlap stile crosses the meeting rail zone at rest",
    )

    # --- Structural checks ---

    # Center pane is wider than left and right panes.
    frame_aabb = ctx.part_world_aabb(frame)
    l_aabb = ctx.part_world_aabb(left_lite)
    c_aabb = ctx.part_world_aabb(center_lite)
    r_aabb = ctx.part_world_aabb(upper_right)
    s_aabb = ctx.part_world_aabb(lower_sash)

    left_w = l_aabb[1][0] - l_aabb[0][0]
    center_w = c_aabb[1][0] - c_aabb[0][0]
    right_w = r_aabb[1][0] - r_aabb[0][0]
    ctx.check(
        "center pane wider than side panes",
        center_w > left_w + 0.20 and center_w > right_w + 0.20,
        details=f"left_w={left_w:.3f}, center_w={center_w:.3f}, right_w={right_w:.3f}",
    )

    # Frame spans the full window.
    frame_w = frame_aabb[1][0] - frame_aabb[0][0]
    ctx.check(
        "frame spans full width",
        frame_w > 2.5,
        details=f"frame_w={frame_w:.3f}",
    )
    ctx.check(
        "sill sits near z=0",
        abs(frame_aabb[0][2]) < 0.02,
        details=f"frame zmin={frame_aabb[0][2]:.4f}",
    )
    ctx.check(
        "head reaches full height",
        abs(frame_aabb[1][2] - TOTAL_H) < 0.02,
        details=f"frame zmax={frame_aabb[1][2]:.4f}",
    )

    # Three columns ordered left -> center -> right.
    lx = (l_aabb[0][0] + l_aabb[1][0]) / 2.0
    cx = (c_aabb[0][0] + c_aabb[1][0]) / 2.0
    rx = (r_aabb[0][0] + r_aabb[1][0]) / 2.0
    ctx.check(
        "columns ordered left-center-right",
        lx < cx < rx,
        details=f"left_x={lx:.3f}, center_x={cx:.3f}, right_x={rx:.3f}",
    )

    # Lower sash is in the right column (overlaps upper right in X).
    sx = (s_aabb[0][0] + s_aabb[1][0]) / 2.0
    ctx.check(
        "lower sash in right column",
        abs(sx - rx) < 0.10,
        details=f"sash_x={sx:.3f}, upper_right_x={rx:.3f}",
    )

    # Lower sash sits proud of upper right lite in +Y.
    ur_y = (r_aabb[0][1] + r_aabb[1][1]) / 2.0
    s_y = (s_aabb[0][1] + s_aabb[1][1]) / 2.0
    ctx.check(
        "lower sash proud of upper right lite",
        s_y > ur_y + 0.02,
        details=f"sash_y={s_y:.3f}, upper_right_y={ur_y:.3f}",
    )

    # Upper right lite center is above lower sash center in Z (at rest).
    # The overlap stile intentionally extends above the sash top rail into the
    # meeting rail zone, so we compare centers rather than extents.
    ur_cz = (r_aabb[0][2] + r_aabb[1][2]) / 2.0
    s_cz = (s_aabb[0][2] + s_aabb[1][2]) / 2.0
    ctx.check(
        "upper right lite above lower sash at rest",
        ur_cz > s_cz + 0.10,
        details=f"upper_right_cz={ur_cz:.3f}, sash_cz={s_cz:.3f}",
    )

    # Fixed lites seated inside frame height.
    for nm, ab in (("left", l_aabb), ("center", c_aabb), ("upper_right", r_aabb)):
        ctx.check(
            f"{nm} lite seated within frame height",
            ab[0][2] > frame_aabb[0][2] - 1e-4 and ab[1][2] < frame_aabb[1][2] + 1e-4,
            details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}]",
        )

    # Lower sash seated within frame height at rest.
    ctx.check(
        "lower sash seated within frame height at rest",
        s_aabb[0][2] > frame_aabb[0][2] - 1e-4 and s_aabb[1][2] < frame_aabb[1][2] + 1e-4,
        details=f"sash z=[{s_aabb[0][2]:.3f},{s_aabb[1][2]:.3f}]",
    )

    # Fixed lites seated in frame openings (projected overlap).
    ctx.expect_overlap(left_lite, frame, axes="xz", min_overlap=0.03,
                       name="left lite seated in frame opening")
    ctx.expect_overlap(center_lite, frame, axes="xz", min_overlap=0.03,
                       name="center lite seated in frame opening")
    ctx.expect_overlap(upper_right, frame, axes="xz", min_overlap=0.03,
                       name="upper right lite seated in frame opening")

    # --- Rest pose (q=0): window reads SHUT ---
    with ctx.pose({slide: 0.0}):
        rest_sash_aabb = ctx.part_world_aabb(lower_sash)
        rest_sash_cz = (rest_sash_aabb[0][2] + rest_sash_aabb[1][2]) / 2.0
        rest_sash_cx = (rest_sash_aabb[0][0] + rest_sash_aabb[1][0]) / 2.0

    # --- Open pose: lower sash slides upward along +Z ---
    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel}):
        open_aabb = ctx.part_world_aabb(lower_sash)
        open_cz = (open_aabb[0][2] + open_aabb[1][2]) / 2.0
        open_cx = (open_aabb[0][0] + open_aabb[1][0]) / 2.0

        # Sash center translated upward by ~travel distance.
        ctx.check(
            "lower sash slides upward by ~travel",
            abs((open_cz - rest_sash_cz) - travel) < 0.02,
            details=f"rest_cz={rest_sash_cz:.3f}, open_cz={open_cz:.3f}, travel={travel:.3f}",
        )
        # Sash does not move horizontally (pure vertical slide).
        ctx.check(
            "slide is purely vertical",
            abs(open_cx - rest_sash_cx) < 0.02,
            details=f"rest_cx={rest_sash_cx:.3f}, open_cx={open_cx:.3f}",
        )
        # Sash retained within frame at full travel.
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained within frame Z span at full travel",
            open_aabb[1][2] < f_aabb[1][2] + 1e-4,
            details=f"sash zmax={open_aabb[1][2]:.3f}, frame zmax={f_aabb[1][2]:.3f}",
        )
        # Sash still overlaps the frame in X (retained in track).
        ctx.expect_overlap(
            lower_sash, frame,
            axes="x",
            min_overlap=0.05,
            name="sash retains horizontal engagement with frame track",
        )

    return ctx.report()


object_model = build_object_model()
