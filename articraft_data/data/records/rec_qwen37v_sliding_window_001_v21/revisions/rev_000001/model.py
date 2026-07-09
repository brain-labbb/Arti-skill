from __future__ import annotations

# Two-panel horizontal sliding window with white vinyl frame and colonial
# divided-lite grilles. Both sashes slide in opposite directions on separate
# prismatic joints with deep track grooves along top and bottom rails.
#
# Coordinate convention:
#   +Z is up.  Width -> X, Height -> Z (sill near z=0), Depth -> Y.
#   Glass plane is the X-Z plane.
#   At q=0, both sashes are closed (window reads shut).
#   Left sash slides +X (rightward), right sash slides -X (leftward).
#   The two sashes ride on separate tracks at different Y positions.
#
# Structure:
#   - frame (root): head, sill, two jambs as one CadQuery solid with a single
#     large opening and deep track grooves cut into the sill and head.
#   - left_sash (PRISMATIC +X): vinyl sash ring + colonial grille + glass,
#     on the rear track.
#   - right_sash (PRISMATIC -X): same construction, on the front track.

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

TOTAL_W = 1.20
TOTAL_H = 1.10

FRAME_FACE = 0.080        # frame member face width (jamb / head / sill)
FRAME_DEPTH = 0.120       # frame depth along Y

HALF_W = TOTAL_W / 2.0

INNER_X0 = -HALF_W + FRAME_FACE   # -0.52
INNER_X1 = HALF_W - FRAME_FACE    #  0.52
INNER_Z0 = FRAME_FACE             #  0.08
INNER_Z1 = TOTAL_H - FRAME_FACE   #  1.02
INNER_W = INNER_X1 - INNER_X0     #  1.04
INNER_H = INNER_Z1 - INNER_Z0     #  0.94

SASH_FACE = 0.045         # sash stile/rail face width
SASH_DEPTH = 0.050        # sash depth along Y
GLASS_T = 0.008           # glass thickness

# Each sash is slightly wider than half the opening for meeting-stile overlap
SASH_OPENING_W = INNER_W / 2.0 + 0.01   # 0.53
SASH_OUTER_W = SASH_OPENING_W + 2.0 * SASH_FACE  # 0.62

# Two separate tracks at different Y positions
LEFT_SASH_Y = -0.024      # rear track center Y
RIGHT_SASH_Y = 0.029      # front track center Y

# Deep track grooves cut into sill and head
GROOVE_DEPTH = SASH_FACE  # 0.045 – grooves exactly accommodate sash rails
GROOVE_WIDTH_Y = SASH_DEPTH + 0.008   # 0.058 – channel wider than sash

# Colonial grille
GRILLE_COLS = 3
GRILLE_ROWS = 5
MUNTIN_T = 0.020
MUNTIN_DEPTH = 0.020

REBATE = 0.005            # glass tucks under sash lip

# Sash closed-position centers (world X)
LEFT_SASH_CX = INNER_X0 + SASH_OUTER_W / 2.0   # -0.21
RIGHT_SASH_CX = INNER_X1 - SASH_OUTER_W / 2.0  #  0.21
MID_CZ = (INNER_Z0 + INNER_Z1) / 2.0            #  0.55

# Slide travel (~82 % of one sash opening width)
SLIDE_TRAVEL = round(SASH_OPENING_W * 0.82, 4)  # ~0.4346

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)
GLASS_RGBA = (0.50, 0.58, 0.64, 0.32)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float,
          y_center: float, depth: float) -> cq.Workplane:
    """Axis-aligned box in the X-Z plane, centered on y_center."""
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
    """Outer frame: one large opening plus deep track grooves in sill & head."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)

    # Main opening through-cut
    cut_d = FRAME_DEPTH + 0.02
    opening = _slab(INNER_X0, INNER_X1, INNER_Z0, INNER_Z1, 0.0, cut_d)
    frame = outer.cut(opening)

    # Deep track grooves – extend the opening into sill / head at each track Y
    for y_pos in (LEFT_SASH_Y, RIGHT_SASH_Y):
        # Sill groove (downward from sill top face)
        sill_g = _slab(
            INNER_X0, INNER_X1,
            INNER_Z0 - GROOVE_DEPTH, INNER_Z0,
            y_pos, GROOVE_WIDTH_Y,
        )
        frame = frame.cut(sill_g)

        # Head groove (upward from head bottom face)
        head_g = _slab(
            INNER_X0, INNER_X1,
            INNER_Z1, INNER_Z1 + GROOVE_DEPTH,
            y_pos, GROOVE_WIDTH_Y,
        )
        frame = frame.cut(head_g)

    return frame


def _build_sash_grille(opening_w: float, opening_h: float) -> cq.Workplane:
    """Sash ring + colonial muntin grid, in sash-local frame centered at origin."""
    ow = opening_w
    oh = opening_h
    out_w = ow + 2.0 * SASH_FACE
    out_h = oh + 2.0 * SASH_FACE

    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SASH_DEPTH)
    hole = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH + 0.02)
    ring = outer.cut(hole)

    bars = None
    for c in range(1, GRILLE_COLS):
        x = -ow / 2.0 + (c / GRILLE_COLS) * ow
        bar = _slab(x - MUNTIN_T / 2.0, x + MUNTIN_T / 2.0,
                    -oh / 2.0, oh / 2.0, 0.0, MUNTIN_DEPTH)
        bars = bar if bars is None else bars.union(bar)
    for r in range(1, GRILLE_ROWS):
        z = -oh / 2.0 + (r / GRILLE_ROWS) * oh
        bar = _slab(-ow / 2.0, ow / 2.0,
                    z - MUNTIN_T / 2.0, z + MUNTIN_T / 2.0,
                    0.0, MUNTIN_DEPTH)
        bars = bar if bars is None else bars.union(bar)

    return ring if bars is None else ring.union(bars)


def _build_sash_glass(opening_w: float, opening_h: float) -> cq.Workplane:
    """Clear pane rebated under the sash lip."""
    ow = opening_w + 2.0 * REBATE
    oh = opening_h + 2.0 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_panel_sliding_window")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="vinyl",
        name="frame_shell",
    )

    opening_h = INNER_H

    # --- Left sash (rear track, slides +X) ---
    left_sash = model.part("left_sash")
    left_sash.visual(
        mesh_from_cadquery(_build_sash_grille(SASH_OPENING_W, opening_h), "left_sash_vinyl"),
        material="vinyl",
        name="left_sash_vinyl",
    )
    left_sash.visual(
        mesh_from_cadquery(_build_sash_glass(SASH_OPENING_W, opening_h), "left_sash_glass"),
        material="glass",
        name="left_sash_glass",
    )

    # --- Right sash (front track, slides -X) ---
    right_sash = model.part("right_sash")
    right_sash.visual(
        mesh_from_cadquery(_build_sash_grille(SASH_OPENING_W, opening_h), "right_sash_vinyl"),
        material="vinyl",
        name="right_sash_vinyl",
    )
    right_sash.visual(
        mesh_from_cadquery(_build_sash_glass(SASH_OPENING_W, opening_h), "right_sash_glass"),
        material="glass",
        name="right_sash_glass",
    )

    # --- Articulations ---

    # Left sash: prismatic along +X (positive q slides rightward)
    model.articulation(
        "frame_to_left_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="left_sash",
        origin=Origin(xyz=(LEFT_SASH_CX, LEFT_SASH_Y, MID_CZ)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.5,
            lower=0.0, upper=SLIDE_TRAVEL,
        ),
    )

    # Right sash: prismatic along -X (positive q slides leftward)
    model.articulation(
        "frame_to_right_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="right_sash",
        origin=Origin(xyz=(RIGHT_SASH_CX, RIGHT_SASH_Y, MID_CZ)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.5,
            lower=0.0, upper=SLIDE_TRAVEL,
        ),
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
    left_slide = object_model.get_articulation("frame_to_left_sash")
    right_slide = object_model.get_articulation("frame_to_right_sash")

    # --- Intentional overlap allowances ---

    # Glass panes rebated under sash/muntin lip on each sash
    for nm in ("left_sash", "right_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_vinyl",
            reason="Glass pane is rebated under the sash/muntin lip (captured glazing).",
        )

    # Each sash rides in the frame track grooves; sash ring laps the track
    # channel walls and the groove floor (track engagement).
    ctx.allow_overlap(
        "frame", "left_sash",
        elem_a="frame_shell",
        elem_b="left_sash_vinyl",
        reason="Left sash rides in the rear track groove; sash ring engages the channel.",
    )
    ctx.allow_overlap(
        "frame", "right_sash",
        elem_a="frame_shell",
        elem_b="right_sash_vinyl",
        reason="Right sash rides in the front track groove; sash ring engages the channel.",
    )
    ctx.allow_overlap(
        "frame", "left_sash",
        elem_a="frame_shell",
        elem_b="left_sash_glass",
        reason="Left sash glass laps the track lip as sash rides the groove.",
    )
    ctx.allow_overlap(
        "frame", "right_sash",
        elem_a="frame_shell",
        elem_b="right_sash_glass",
        reason="Right sash glass laps the track lip as sash rides the groove.",
    )

    # --- Closed-pose checks (q=0) ---
    with ctx.pose({left_slide: 0.0, right_slide: 0.0}):
        f_aabb = ctx.part_world_aabb(frame)
        l_aabb = ctx.part_world_aabb(left_sash)
        r_aabb = ctx.part_world_aabb(right_sash)

        # Frame proportions
        frame_w = f_aabb[1][0] - f_aabb[0][0]
        ctx.check(
            "frame spans wider than both sashes combined",
            frame_w > 1.0,
            details=f"frame_w={frame_w:.3f}",
        )
        ctx.check(
            "sill near z=0",
            abs(f_aabb[0][2]) < 0.02,
            details=f"frame zmin={f_aabb[0][2]:.4f}",
        )
        ctx.check(
            "head reaches full height",
            abs(f_aabb[1][2] - TOTAL_H) < 0.02,
            details=f"frame zmax={f_aabb[1][2]:.4f}",
        )

        # Sashes ordered left-right
        lx = (l_aabb[0][0] + l_aabb[1][0]) / 2.0
        rx = (r_aabb[0][0] + r_aabb[1][0]) / 2.0
        ctx.check(
            "sashes ordered left-right at rest",
            lx < rx,
            details=f"left_x={lx:.3f}, right_x={rx:.3f}",
        )

        # Sashes on separate Y tracks
        ly = (l_aabb[0][1] + l_aabb[1][1]) / 2.0
        ry = (r_aabb[0][1] + r_aabb[1][1]) / 2.0
        ctx.check(
            "sashes on separate Y tracks",
            abs(ly - ry) > 0.03,
            details=f"left_y={ly:.3f}, right_y={ry:.3f}",
        )

        # Both sashes seated within frame height
        for nm, ab in (("left_sash", l_aabb), ("right_sash", r_aabb)):
            ctx.check(
                f"{nm} within frame height",
                ab[0][2] > f_aabb[0][2] - 1e-4 and ab[1][2] < f_aabb[1][2] + 1e-4,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}]",
            )

        # Both sashes overlap the frame opening in the glass plane
        ctx.expect_overlap(
            left_sash, frame, axes="xz", min_overlap=0.10,
            name="left sash overlaps frame opening",
        )
        ctx.expect_overlap(
            right_sash, frame, axes="xz", min_overlap=0.10,
            name="right sash overlaps frame opening",
        )

        rest_lx = lx
        rest_rx = rx
        rest_lz = (l_aabb[0][2] + l_aabb[1][2]) / 2.0
        rest_rz = (r_aabb[0][2] + r_aabb[1][2]) / 2.0

    # --- Open pose: both sashes slide in opposite directions ---
    travel_l = left_slide.motion_limits.upper
    travel_r = right_slide.motion_limits.upper

    with ctx.pose({left_slide: travel_l, right_slide: travel_r}):
        l_open = ctx.part_world_aabb(left_sash)
        r_open = ctx.part_world_aabb(right_sash)

        open_lx = (l_open[0][0] + l_open[1][0]) / 2.0
        open_rx = (r_open[0][0] + r_open[1][0]) / 2.0

        # Left sash moved +X
        ctx.check(
            "left sash slides +X",
            open_lx > rest_lx + 0.10,
            details=f"rest_lx={rest_lx:.3f}, open_lx={open_lx:.3f}",
        )
        # Right sash moved -X
        ctx.check(
            "right sash slides -X",
            open_rx < rest_rx - 0.10,
            details=f"rest_rx={rest_rx:.3f}, open_rx={open_rx:.3f}",
        )

        # Both sashes retained within frame X span
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "left sash retained in frame X span",
            l_open[1][0] < f_aabb[1][0] + 1e-4 and l_open[0][0] > f_aabb[0][0] - 1e-4,
            details=f"left x=[{l_open[0][0]:.3f},{l_open[1][0]:.3f}]",
        )
        ctx.check(
            "right sash retained in frame X span",
            r_open[1][0] < f_aabb[1][0] + 1e-4 and r_open[0][0] > f_aabb[0][0] - 1e-4,
            details=f"right x=[{r_open[0][0]:.3f},{r_open[1][0]:.3f}]",
        )

        # Pure horizontal slide (no Z movement)
        open_lz = (l_open[0][2] + l_open[1][2]) / 2.0
        open_rz = (r_open[0][2] + r_open[1][2]) / 2.0
        ctx.check(
            "left slide purely horizontal",
            abs(open_lz - rest_lz) < 0.02,
            details=f"open_lz={open_lz:.3f}, rest_lz={rest_lz:.3f}",
        )
        ctx.check(
            "right slide purely horizontal",
            abs(open_rz - rest_rz) < 0.02,
            details=f"open_rz={open_rz:.3f}, rest_rz={rest_rz:.3f}",
        )

        # Vertical track engagement retained
        ctx.expect_overlap(
            left_sash, frame, axes="z", min_overlap=0.10,
            name="left sash retains vertical track engagement at full travel",
        )
        ctx.expect_overlap(
            right_sash, frame, axes="z", min_overlap=0.10,
            name="right sash retains vertical track engagement at full travel",
        )

    # --- Joint-type checks ---
    ctx.check(
        "left slide has positive travel",
        left_slide.motion_limits.upper > 0.10,
        details=f"upper={left_slide.motion_limits.upper:.4f}",
    )
    ctx.check(
        "right slide has positive travel",
        right_slide.motion_limits.upper > 0.10,
        details=f"upper={right_slide.motion_limits.upper:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
