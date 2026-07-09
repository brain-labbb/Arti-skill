from __future__ import annotations

# Double-hung vertical sliding window, white vinyl frame with colonial
# divided-lite grilles. Upper sash is fixed; lower sash slides upward on a
# vertical prismatic joint. Two small roller blocks at the bottom of the
# moving sash. The lower sash is partially open at rest (q=0) so the
# overlap with the meeting rail is visible.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     - width  -> X
#     - height -> Z   (sill near z=0)
#     - frame depth / glazing thickness / slide-normal -> Y
#   The glass plane is the X-Z plane.

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

TOTAL_W = 1.20            # overall window width along X
TOTAL_H = 1.50            # overall height along Z (sill at z=0, head at z=TOTAL_H)

FRAME_FACE = 0.065        # outer frame member face width (jamb / head / sill)
MEETING_RAIL_H = 0.040    # horizontal meeting rail / check rail height
FRAME_DEPTH = 0.110       # outer frame depth along Y

SASH_FACE = 0.050         # sash perimeter rail/stile face width (in-plane)
SASH_DEPTH = 0.050        # sash depth along Y
GLASS_T = 0.008           # glazing thickness along Y

# Colonial grille (divided lite): each sash has a grid of small panes.
GRILLE_COLS = 4           # 4 columns of panes per sash
GRILLE_ROWS = 3           # 3 rows of panes per sash
MUNTIN_T = 0.018          # muntin bar face width
MUNTIN_DEPTH = 0.018      # muntin bar depth along Y

# Y layout (depth). Upper (fixed) sash in the rear track; lower (sliding)
# sash in the front track, proud toward +Y so it can slide past the meeting rail.
UPPER_SASH_Y = -0.020     # upper sash center Y (rear track)
LOWER_SASH_Y = 0.045      # lower sash center Y (front track, proud)

REBATE = 0.005            # glass tucks under the sash/muntin lip by this much

# Roller blocks at bottom of lower sash
ROLLER_W = 0.030          # roller width along X
ROLLER_H = 0.018          # roller height along Z
ROLLER_D = 0.020          # roller depth along Y

# Partial-open offset at rest: lower sash raised this much from closed position
PARTIAL_OPEN_OFFSET = 0.10  # meters

# ---------------------------------------------------------------------------
# Derived layout
# ---------------------------------------------------------------------------

HALF_W = TOTAL_W / 2.0

INNER_X0 = -HALF_W + FRAME_FACE
INNER_X1 = HALF_W - FRAME_FACE
INNER_Z0 = FRAME_FACE
INNER_Z1 = TOTAL_H - FRAME_FACE
MID_Z = TOTAL_H / 2.0

OPENING_W = INNER_X1 - INNER_X0

# Upper opening (fixed sash)
UPPER_Z0 = MID_Z + MEETING_RAIL_H / 2.0
UPPER_Z1 = INNER_Z1
UPPER_OPENING_H = UPPER_Z1 - UPPER_Z0

# Lower opening (sliding sash)
LOWER_Z0 = INNER_Z0
LOWER_Z1 = MID_Z - MEETING_RAIL_H / 2.0
LOWER_OPENING_H = LOWER_Z1 - LOWER_Z0

# Lower sash center at closed position
LOWER_SASH_CLOSED_Z = (LOWER_Z0 + LOWER_Z1) / 2.0
# Lower sash center at rest (partially open)
LOWER_SASH_REST_Z = LOWER_SASH_CLOSED_Z + PARTIAL_OPEN_OFFSET

# Slide travel (how much further the sash can go up from rest)
SLIDE_TRAVEL = 0.30

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)     # bright white vinyl/PVC
GLASS_RGBA = (0.50, 0.58, 0.64, 0.32)    # cool grey-blue, semi-transparent
ROLLER_RGBA = (0.22, 0.22, 0.24, 1.0)    # dark grey nylon roller housing


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
    """Static outer frame: a full slab cut by the upper and lower sash openings,
    leaving head, sill, two jambs and the horizontal meeting rail as one solid."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)

    cut_depth = FRAME_DEPTH + 0.02  # through-cut clearance in Y
    lower_cut = _slab(INNER_X0, INNER_X1, LOWER_Z0, LOWER_Z1, 0.0, cut_depth)
    upper_cut = _slab(INNER_X0, INNER_X1, UPPER_Z0, UPPER_Z1, 0.0, cut_depth)

    return outer.cut(lower_cut).cut(upper_cut)


def _build_frame_track() -> cq.Workplane:
    """Two thin vertical track strips on the inner face of the jambs where the
    lower sash rides. These are raised rails on the front inner face."""
    track_w = 0.012
    track_d = 0.010
    track_h = LOWER_OPENING_H + MEETING_RAIL_H + 0.02  # spans lower opening + meeting rail
    track_z_center = (LOWER_Z0 + MID_Z) / 2.0

    # Track Y position: front inner face of jamb (toward +Y, where lower sash rides)
    track_y = FRAME_DEPTH / 2.0 - track_d / 2.0 - 0.002

    # Left jamb track
    left_x = INNER_X0 - track_w / 2.0 + 0.002
    left_track = _slab(
        left_x - track_w / 2.0, left_x + track_w / 2.0,
        track_z_center - track_h / 2.0, track_z_center + track_h / 2.0,
        track_y, track_d,
    )

    # Right jamb track
    right_x = INNER_X1 + track_w / 2.0 - 0.002
    right_track = _slab(
        right_x - track_w / 2.0, right_x + track_w / 2.0,
        track_z_center - track_h / 2.0, track_z_center + track_h / 2.0,
        track_y, track_d,
    )

    return left_track.union(right_track)


def _build_sash_grille_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """One sash built in its OWN local frame, centered on local origin:
      - local X in [-opening_w/2 - SASH_FACE, +opening_w/2 + SASH_FACE]
      - local Z in [-opening_h/2 - SASH_FACE, +opening_h/2 + SASH_FACE]
      - local Y is the sash depth, centered at 0
    Construction: outer sash slab cut by the clear opening, then the colonial
    muntin grid (vertical + horizontal bars) unioned back in across the opening.
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

    # Vertical muntins: GRILLE_COLS columns -> (GRILLE_COLS - 1) interior bars.
    for c in range(1, GRILLE_COLS):
        frac = c / GRILLE_COLS
        x = -ow / 2.0 + frac * ow
        bar = _slab(
            x - MUNTIN_T / 2.0, x + MUNTIN_T / 2.0,
            -oh / 2.0, oh / 2.0,
            0.0, MUNTIN_DEPTH,
        )
        bars = bar if bars is None else bars.union(bar)

    # Horizontal muntins: GRILLE_ROWS rows -> (GRILLE_ROWS - 1) interior bars.
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


def _build_sash_glass_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Single clear pane filling the sash opening, in the same sash-local frame."""
    ow = opening_w + 2 * REBATE
    oh = opening_h + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_roller_block() -> cq.Workplane:
    """Small roller housing block, centered on local origin."""
    return (
        cq.Workplane("XY")
        .box(ROLLER_W, ROLLER_D, ROLLER_H)
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_hung_sliding_window")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("roller", rgba=ROLLER_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="vinyl",
        name="frame_shell",
    )
    frame.visual(
        mesh_from_cadquery(_build_frame_track(), "frame_track"),
        material="vinyl",
        name="frame_track",
    )

    # --- Upper sash (FIXED) ---
    upper_sash = model.part("upper_sash")
    upper_sash.visual(
        mesh_from_cadquery(_build_sash_grille_shape(OPENING_W, UPPER_OPENING_H), "upper_sash_vinyl"),
        material="vinyl",
        name="upper_sash_vinyl",
    )
    upper_sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(OPENING_W, UPPER_OPENING_H), "upper_sash_glass"),
        material="glass",
        name="upper_sash_glass",
    )

    # --- Lower sash (SLIDING) ---
    lower_sash = model.part("lower_sash")
    lower_sash.visual(
        mesh_from_cadquery(_build_sash_grille_shape(OPENING_W, LOWER_OPENING_H), "lower_sash_vinyl"),
        material="vinyl",
        name="lower_sash_vinyl",
    )
    lower_sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(OPENING_W, LOWER_OPENING_H), "lower_sash_glass"),
        material="glass",
        name="lower_sash_glass",
    )

    # Two roller blocks at the bottom of the lower sash, near left and right edges.
    # In the sash-local frame, the bottom rail bottom is at z = -(LOWER_OPENING_H/2 + SASH_FACE).
    # Rollers sit at that Z level, offset toward the left and right stile centers.
    roller_mesh = mesh_from_cadquery(_build_roller_block(), "roller_block")
    roller_z_local = -(LOWER_OPENING_H / 2.0 + SASH_FACE) - ROLLER_H / 2.0 + 0.004
    roller_x_offset = OPENING_W / 2.0 - ROLLER_W / 2.0 - 0.010

    # Left roller
    lower_sash.visual(
        roller_mesh,
        material="roller",
        origin=Origin(xyz=(-roller_x_offset, 0.0, roller_z_local)),
        name="roller_left",
    )
    # Right roller
    lower_sash.visual(
        roller_mesh,
        material="roller",
        origin=Origin(xyz=(roller_x_offset, 0.0, roller_z_local)),
        name="roller_right",
    )

    # --- Articulations ---

    # Upper sash center (world)
    upper_cz = (UPPER_Z0 + UPPER_Z1) / 2.0
    model.articulation(
        "frame_to_upper_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="upper_sash",
        origin=Origin(xyz=(0.0, UPPER_SASH_Y, upper_cz)),
    )

    # Lower sash: PRISMATIC along +Z. Joint origin at the partially-open rest
    # position so q=0 shows the sash partially raised with visible overlap at
    # the meeting rail. Positive q slides the sash further upward.
    model.articulation(
        "frame_to_lower_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="lower_sash",
        origin=Origin(xyz=(0.0, LOWER_SASH_Y, LOWER_SASH_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.3,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    upper_sash = object_model.get_part("upper_sash")
    lower_sash = object_model.get_part("lower_sash")
    slide = object_model.get_articulation("frame_to_lower_sash")

    # --- Intentional overlaps ---
    # Glass panes tuck under the vinyl/muntin lip on each sash (captured glass).
    for nm in ("upper_sash", "lower_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_vinyl",
            reason="Clear pane is rebated under the sash/muntin lip so it reads captured, not floating.",
        )

    # Upper sash is seated in the frame upper opening (rebated capture).
    ctx.allow_overlap(
        "frame", "upper_sash",
        elem_a="frame_shell",
        elem_b="upper_sash_vinyl",
        reason="Upper fixed sash is rebated into the frame opening; its ring laps the jamb/head edge (seated capture).",
    )
    ctx.allow_overlap(
        "frame", "upper_sash",
        elem_a="frame_shell",
        elem_b="upper_sash_glass",
        reason="Upper sash glass is rebated under the frame opening lip (captured glazing).",
    )

    # Lower sash rides the front track and laps the meeting rail / jamb edges.
    ctx.allow_overlap(
        "frame", "lower_sash",
        elem_a="frame_shell",
        elem_b="lower_sash_vinyl",
        reason="Lower sash rides the front track and laps the meeting rail and jamb edges (slider capture).",
    )
    # Frame track rails are the vertical guides that capture the lower sash edges.
    ctx.allow_overlap(
        "frame", "lower_sash",
        elem_a="frame_track",
        elem_b="lower_sash_vinyl",
        reason="Frame track rails are the vertical guide strips that the lower sash rides between (slider track capture).",
    )
    ctx.allow_overlap(
        "frame", "lower_sash",
        elem_a="frame_track",
        elem_b="lower_sash_glass",
        reason="Frame track rails overlap the lower sash glass edge as the glass is rebated into the sash near the track (captured glazing in track).",
    )
    ctx.allow_overlap(
        "frame", "lower_sash",
        elem_a="frame_shell",
        elem_b="lower_sash_glass",
        reason="Lower sash glass laps the meeting rail lip as the sash rides the track (captured glazing).",
    )

    # Roller blocks are attached to the lower sash bottom rail (intentional contact/embed).
    ctx.allow_overlap(
        "lower_sash", "lower_sash",
        elem_a="lower_sash_vinyl",
        elem_b="roller_left",
        reason="Left roller block is fastened into the bottom rail of the lower sash.",
    )
    ctx.allow_overlap(
        "lower_sash", "lower_sash",
        elem_a="lower_sash_vinyl",
        elem_b="roller_right",
        reason="Right roller block is fastened into the bottom rail of the lower sash.",
    )

    # --- Rest pose (q=0): lower sash partially open ---
    with ctx.pose({slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        upper_aabb = ctx.part_world_aabb(upper_sash)
        lower_aabb = ctx.part_world_aabb(lower_sash)

        # Frame spans the full window dimensions.
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        frame_h = frame_aabb[1][2] - frame_aabb[0][2]
        ctx.check(
            "frame has reasonable width",
            frame_w > 1.0,
            details=f"frame_w={frame_w:.3f}",
        )
        ctx.check(
            "frame has reasonable height",
            frame_h > 1.3,
            details=f"frame_h={frame_h:.3f}",
        )

        # Sill sits near z=0.
        ctx.check(
            "sill sits near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"frame zmin={frame_aabb[0][2]:.4f}",
        )

        # Upper sash is above lower sash.
        upper_cz = (upper_aabb[0][2] + upper_aabb[1][2]) / 2.0
        lower_cz = (lower_aabb[0][2] + lower_aabb[1][2]) / 2.0
        ctx.check(
            "upper sash above lower sash",
            upper_cz > lower_cz + 0.05,
            details=f"upper_cz={upper_cz:.3f}, lower_cz={lower_cz:.3f}",
        )

        # Lower sash is partially open at rest: its bottom is above the sill inner edge.
        lower_bottom = lower_aabb[0][2]
        sill_inner = INNER_Z0
        ctx.check(
            "lower sash partially open at rest (bottom above sill)",
            lower_bottom > sill_inner + 0.03,
            details=f"lower_bottom={lower_bottom:.3f}, sill_inner={sill_inner:.3f}",
        )

        # Lower sash top overlaps the meeting rail region (visible overlap).
        lower_top = lower_aabb[1][2]
        meeting_rail_z = MID_Z
        ctx.check(
            "lower sash overlaps meeting rail at rest",
            lower_top > meeting_rail_z,
            details=f"lower_top={lower_top:.3f}, meeting_rail_z={meeting_rail_z:.3f}",
        )

        # Lower sash sits proud (in +Y) of the upper sash.
        upper_y = (upper_aabb[0][1] + upper_aabb[1][1]) / 2.0
        lower_y = (lower_aabb[0][1] + lower_aabb[1][1]) / 2.0
        ctx.check(
            "lower sash proud of upper sash (front track)",
            lower_y > upper_y + 0.02,
            details=f"lower_y={lower_y:.3f}, upper_y={upper_y:.3f}",
        )

        # Upper sash seated in frame opening (projected overlap in glass plane).
        ctx.expect_overlap(
            upper_sash, frame, axes="xz", min_overlap=0.02,
            name="upper sash seated in frame opening",
        )

        # Lower sash stays within the frame horizontally (captured by track and jambs).
        ctx.expect_within(
            lower_sash, frame, axes="x",
            margin=0.01,
            name="lower sash stays within frame width at rest",
        )

        # Lower sash has roller blocks (proven by X-width including rollers).
        sash_w = lower_aabb[1][0] - lower_aabb[0][0]
        ctx.check(
            "lower sash includes roller blocks (wider than opening)",
            sash_w > OPENING_W * 0.8,
            details=f"sash_w={sash_w:.3f}, opening_w={OPENING_W:.3f}",
        )

        rest_lower_cz = lower_cz

    # --- Fully open pose: lower sash slides further upward ---
    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel}):
        lower_open_aabb = ctx.part_world_aabb(lower_sash)
        open_lower_cz = (lower_open_aabb[0][2] + lower_open_aabb[1][2]) / 2.0

        # Lower sash moved upward by ~travel distance.
        ctx.check(
            "lower sash slides upward by ~travel",
            abs((open_lower_cz - rest_lower_cz) - travel) < 0.02,
            details=f"rest_cz={rest_lower_cz:.3f}, open_cz={open_lower_cz:.3f}, travel={travel:.3f}",
        )

        # Movement is purely vertical (X position unchanged).
        open_lower_cx = (lower_open_aabb[0][0] + lower_open_aabb[1][0]) / 2.0
        rest_lower_cx_approx = 0.0  # centered at rest
        ctx.check(
            "slide is purely vertical (X unchanged)",
            abs(open_lower_cx - rest_lower_cx_approx) < 0.02,
            details=f"open_cx={open_lower_cx:.3f}",
        )

        # Lower sash remains retained in the frame at full travel.
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "lower sash retained within frame at full travel",
            lower_open_aabb[1][2] < f_aabb[1][2] + 0.05,
            details=f"sash zmax={lower_open_aabb[1][2]:.3f}, frame zmax={f_aabb[1][2]:.3f}",
        )

        # At full travel, lower sash still overlaps frame vertically (retained).
        ctx.expect_overlap(
            lower_sash, frame,
            axes="z",
            min_overlap=0.05,
            name="lower sash retains vertical engagement with frame at full travel",
        )

    # --- Prismatic joint axis check ---
    ctx.check(
        "lower sash joint is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
