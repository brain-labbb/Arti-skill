from __future__ import annotations

# Two-panel horizontal sliding window (dual-slider variant), white vinyl frame.
# BOTH sashes slide in opposite directions on separate prismatic joints:
#   - left_sash:  slides right (+X) to open, rear track
#   - right_sash: slides left  (-X) to open, front track
# The sashes stack over each other at center when both are open (separated in Y
# on their respective tracks). Deep track grooves are cut into the head (top
# rail) and sill (bottom rail) of the frame at each sash's Y-track position.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     width  -> X,  height -> Z (sill near z=0),  depth -> Y
#   Glass plane is the X-Z plane. q=0 reads SHUT for both sashes.
#
# Structure:
#   - frame (root): head, sill, two jambs as a hollow slab, plus deep track
#     grooves on the inner sill and head faces at each sash Y-track.
#   - left_sash (PRISMATIC +X): vinyl sash ring + glass, rear track; latch on
#     meeting (right) stile.
#   - right_sash (PRISMATIC -X): vinyl sash ring + glass, front track.

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

SASH_FACE = 0.075         # sash perimeter rail/stile face width (chunky)
SASH_DEPTH = 0.060        # sash depth along Y
GLASS_T = 0.008           # glazing thickness along Y

# Y layout: frame box centered on y=0. Left sash in the rear track;
# right sash sits proud toward +Y (front track) so it passes in front.
LEFT_SASH_Y = -0.028      # rear track center (Y)
RIGHT_SASH_Y = 0.044      # front track proud toward +Y

REBATE = 0.005            # glass tucks under the sash lip by this much

# Track groove dimensions (deep channels in head and sill)
GROOVE_W_Y = 0.068        # groove width along Y (slightly wider than sash depth)
GROOVE_DEPTH = 0.020      # groove depth cut into the frame rail (Z direction)

# Latch (cam lock) hardware
LATCH_PLATE_W = 0.028
LATCH_PLATE_H = 0.075
LATCH_PLATE_T = 0.010
LATCH_LEVER_LEN = 0.045
LATCH_LEVER_R = 0.006

METAL_RGBA = (0.74, 0.76, 0.79, 1.0)

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

# Closed-pose centers (world X) of each sash
LEFT_OPEN_CX = INNER_X0 + SASH_OPENING_W / 2.0          # left
RIGHT_OPEN_CX = INNER_X1 - SASH_OPENING_W / 2.0          # right
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
    """Static outer frame: thick hollow slab with the big sash opening, plus
    deep track grooves cut into the sill top face and head bottom face at
    each sash Y-track position."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    cut_depth = FRAME_DEPTH + 0.02
    # One clear opening spanning the whole inner region.
    opening = _slab(INNER_X0, INNER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    frame = outer.cut(opening)

    # --- Deep track grooves in sill (top face of sill, cut downward) ---
    # Sill top face is at z = INNER_Z0. Grooves cut from there downward into
    # the sill body. Each groove is centered on the sash Y-track position.
    for track_y in (LEFT_SASH_Y, RIGHT_SASH_Y):
        groove = _slab(
            INNER_X0, INNER_X1,
            INNER_Z0 - GROOVE_DEPTH, INNER_Z0,
            track_y, GROOVE_W_Y,
        )
        frame = frame.cut(groove)

    # --- Deep track grooves in head (bottom face of head, cut upward) ---
    # Head bottom face is at z = INNER_Z1. Grooves cut from there upward.
    for track_y in (LEFT_SASH_Y, RIGHT_SASH_Y):
        groove = _slab(
            INNER_X0, INNER_X1,
            INNER_Z1, INNER_Z1 + GROOVE_DEPTH,
            track_y, GROOVE_W_Y,
        )
        frame = frame.cut(groove)

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
    """Single clear pane filling the sash opening, rebated under the sash lip."""
    ow = SASH_OPENING_W + 2 * REBATE
    oh = SASH_OPENING_H + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


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
    """Add cam-latch hardware on the sash meeting stile at mid-height.
    For the left sash, the meeting stile is the RIGHT stile (local +X side)."""
    sash = model.get_part(sash_name)

    # Meeting stile center: right side of left sash local frame
    stile_x = SASH_OPENING_W / 2.0 + SASH_FACE / 2.0
    face_y = SASH_DEPTH / 2.0
    plate_y = face_y + LATCH_PLATE_T / 2.0

    sash.visual(
        Box((LATCH_PLATE_W, LATCH_PLATE_T, LATCH_PLATE_H)),
        origin=Origin(xyz=(stile_x, plate_y, 0.0)),
        material="metal",
        name=f"{sash_name}_latch_plate",
    )
    # Lever is a horizontal thumb-turn extending along X (in the window plane),
    # positioned just proud of the plate front face. This avoids protruding in Y
    # toward the other sash on its separate track.
    lever_y = face_y + LATCH_PLATE_T + LATCH_LEVER_R
    sash.visual(
        Cylinder(radius=LATCH_LEVER_R, length=LATCH_LEVER_LEN),
        origin=Origin(xyz=(stile_x, lever_y, 0.0), rpy=(0.0, 1.5707963, 0.0)),
        material="metal",
        name=f"{sash_name}_latch_lever",
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_slider_window")
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

    # --- Both sashes slide ---
    _add_sash(model, "left_sash")
    _add_sash(model, "right_sash")
    _add_latch(model, "left_sash")

    # Travel limited so both sashes stay within the frame X span at full open.
    # Each sash slides toward the center on its own track, stacking over the
    # other sash (separated in Y).
    sash_out_w = SASH_OPENING_W + 2 * SASH_FACE
    # Left sash slides +X: right edge at (LEFT_OPEN_CX + t + sash_out_w/2) must stay ≤ HALF_W
    left_max = HALF_W - LEFT_OPEN_CX - sash_out_w / 2.0 - 0.005
    # Right sash slides -X: left edge at (RIGHT_OPEN_CX - t - sash_out_w/2) must stay ≥ -HALF_W
    right_max = RIGHT_OPEN_CX + HALF_W - sash_out_w / 2.0 - 0.005
    slide_travel = min(left_max, right_max, SASH_OPENING_W * 0.85)

    # LEFT sash: PRISMATIC along +X. Positive q slides it right to open,
    # stacking over the right sash on the rear track.
    model.articulation(
        "frame_to_left_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="left_sash",
        origin=Origin(xyz=(LEFT_OPEN_CX, LEFT_SASH_Y, MID_CZ)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5, lower=0.0, upper=slide_travel),
    )

    # RIGHT sash: PRISMATIC along -X. Positive q slides it left to open,
    # stacking over the left sash on the front track.
    model.articulation(
        "frame_to_right_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="right_sash",
        origin=Origin(xyz=(RIGHT_OPEN_CX, RIGHT_SASH_Y, MID_CZ)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5, lower=0.0, upper=slide_travel),
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

    # --- Intentional overlaps ---
    # Glass tucks under the vinyl sash lip on each sash (captured glass).
    for nm in ("left_sash", "right_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_vinyl",
            reason="Clear pane is rebated under the sash lip so it reads captured, not floating.",
        )
    # Each sash ring laps the frame opening edge (seated in track grooves).
    for nm in ("left_sash", "right_sash"):
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_vinyl",
            reason=f"{nm} ring is seated in the frame head/sill track grooves (captured sash).",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_glass",
            reason=f"{nm} glass is rebated under the frame opening lip (captured glazing).",
        )
    # Latch keeper plate seated on left sash meeting stile face.
    ctx.allow_overlap(
        "left_sash", "left_sash",
        elem_a="left_sash_latch_plate",
        elem_b="left_sash_vinyl",
        reason="Latch keeper plate is seated onto the left-sash meeting-stile face (mounted hardware).",
    )

    # --- Closed pose (q=0 for both): window reads SHUT ---
    with ctx.pose({left_slide: 0.0, right_slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        l_aabb = ctx.part_world_aabb(left_sash)
        r_aabb = ctx.part_world_aabb(right_sash)

        # Frame spans full width, wider than one sash.
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        sash_w = r_aabb[1][0] - r_aabb[0][0]
        ctx.check(
            "frame spans wider than a single sash",
            frame_w > sash_w + 0.40,
            details=f"frame_w={frame_w:.3f}, sash_w={sash_w:.3f}",
        )
        # Sill near floor, head at full height.
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
        # Left sash is to the left of right sash.
        lx = (l_aabb[0][0] + l_aabb[1][0]) / 2.0
        rx = (r_aabb[0][0] + r_aabb[1][0]) / 2.0
        ctx.check(
            "left sash is left of right sash",
            lx < rx,
            details=f"left_x={lx:.3f}, right_x={rx:.3f}",
        )
        # Both sashes seated within frame height.
        for nm, ab in (("left", l_aabb), ("right", r_aabb)):
            ctx.check(
                f"{nm} sash seated within frame height",
                ab[0][2] > frame_aabb[0][2] - 1e-4 and ab[1][2] < frame_aabb[1][2] + 1e-4,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}]",
            )
        # Right sash sits proud (+Y) of left sash (front vs rear track).
        ly = (l_aabb[0][1] + l_aabb[1][1]) / 2.0
        ry = (r_aabb[0][1] + r_aabb[1][1]) / 2.0
        ctx.check(
            "right sash proud of left sash (front track)",
            ry > ly + 0.02,
            details=f"right_y={ry:.3f}, left_y={ly:.3f}",
        )
        # Both sashes seated in frame opening.
        ctx.expect_overlap(
            left_sash, frame, axes="xz", min_overlap=0.03,
            name="left sash seated in frame opening",
        )
        ctx.expect_overlap(
            right_sash, frame, axes="xz", min_overlap=0.03,
            name="right sash seated in frame opening",
        )

        # Latch on left sash meeting (right) stile, mid-height, front face.
        latch_aabb = ctx.part_element_world_aabb(left_sash, elem="left_sash_latch_plate")
        latch_cx = (latch_aabb[0][0] + latch_aabb[1][0]) / 2.0
        latch_cz = (latch_aabb[0][2] + latch_aabb[1][2]) / 2.0
        latch_cy = (latch_aabb[0][1] + latch_aabb[1][1]) / 2.0
        ctx.check(
            "latch on left sash meeting (right) stile",
            latch_cx > lx,
            details=f"latch_x={latch_cx:.3f}, left_center_x={lx:.3f}",
        )
        ctx.check(
            "latch near mid-height",
            abs(latch_cz - MID_CZ) < 0.20,
            details=f"latch_z={latch_cz:.3f}, mid_z={MID_CZ:.3f}",
        )
        ctx.check(
            "latch stands off front sash face",
            latch_cy > ly,
            details=f"latch_y={latch_cy:.3f}, sash_y={ly:.3f}",
        )

        rest_lx = lx
        rest_rx = rx
        rest_lz = (l_aabb[0][2] + l_aabb[1][2]) / 2.0
        rest_rz = (r_aabb[0][2] + r_aabb[1][2]) / 2.0

    # --- Open pose: both sashes slide toward center (opposite directions) ---
    travel = left_slide.motion_limits.upper
    with ctx.pose({left_slide: travel, right_slide: travel}):
        l_open = ctx.part_world_aabb(left_sash)
        r_open = ctx.part_world_aabb(right_sash)
        open_lx = (l_open[0][0] + l_open[1][0]) / 2.0
        open_rx = (r_open[0][0] + r_open[1][0]) / 2.0

        # Left sash moved in +X (right)
        ctx.check(
            "left sash opens toward right (+X)",
            open_lx > rest_lx + 0.20,
            details=f"rest_lx={rest_lx:.3f}, open_lx={open_lx:.3f}",
        )
        # Right sash moved in -X (left)
        ctx.check(
            "right sash opens toward left (-X)",
            open_rx < rest_rx - 0.20,
            details=f"rest_rx={rest_rx:.3f}, open_rx={open_rx:.3f}",
        )
        # Sashes moved in opposite directions
        ctx.check(
            "sashes slide in opposite directions",
            open_lx > rest_lx and open_rx < rest_rx,
            details=f"left_delta={open_lx - rest_lx:.3f}, right_delta={open_rx - rest_rx:.3f}",
        )
        # Pure horizontal slide (no Z change) for both.
        open_lz = (l_open[0][2] + l_open[1][2]) / 2.0
        open_rz = (r_open[0][2] + r_open[1][2]) / 2.0
        ctx.check(
            "left slide is purely horizontal",
            abs(open_lz - rest_lz) < 0.02,
            details=f"open_lz={open_lz:.3f}, rest_lz={rest_lz:.3f}",
        )
        ctx.check(
            "right slide is purely horizontal",
            abs(open_rz - rest_rz) < 0.02,
            details=f"open_rz={open_rz:.3f}, rest_rz={rest_rz:.3f}",
        )
        # Retained insertion: both sashes stay within frame X span.
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "left sash retained within frame X span at full travel",
            l_open[0][0] > f_aabb[0][0] - 1e-4 and l_open[1][0] < f_aabb[1][0] + 1e-4,
            details=f"sash x=[{l_open[0][0]:.3f},{l_open[1][0]:.3f}] frame x=[{f_aabb[0][0]:.3f},{f_aabb[1][0]:.3f}]",
        )
        ctx.check(
            "right sash retained within frame X span at full travel",
            r_open[0][0] > f_aabb[0][0] - 1e-4 and r_open[1][0] < f_aabb[1][0] + 1e-4,
            details=f"sash x=[{r_open[0][0]:.3f},{r_open[1][0]:.3f}] frame x=[{f_aabb[0][0]:.3f},{f_aabb[1][0]:.3f}]",
        )
        ctx.expect_overlap(
            left_sash, frame, axes="z", min_overlap=0.10,
            name="left sash retains vertical engagement with track grooves",
        )
        ctx.expect_overlap(
            right_sash, frame, axes="z", min_overlap=0.10,
            name="right sash retains vertical engagement with track grooves",
        )

    # --- Both joints are non-fixed (prismatic) ---
    ctx.check(
        "left sash joint is prismatic",
        left_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={left_slide.articulation_type}",
    )
    ctx.check(
        "right sash joint is prismatic",
        right_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={right_slide.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
