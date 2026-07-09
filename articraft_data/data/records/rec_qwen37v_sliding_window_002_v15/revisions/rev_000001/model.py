from __future__ import annotations

# Variant: thick aluminum frame sliding window with deep track grooves.
# Forked from white-vinyl parent into a distinct aluminum sibling.
# Deep U-channel track grooves are cut into the head and sill rails,
# with integrated wear rails at groove floors. One sash slides left-right
# on a prismatic joint in the front track.
#
# Coordinate convention:
#   +Z up, window stands vertically
#   width  -> X
#   height -> Z (sill near z=0)
#   depth  -> Y (glass plane is X-Z)
#   q=0 is closed; positive q slides the right sash toward -X (open).

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
TOTAL_H = 1.72            # overall height along Z

FRAME_FACE = 0.095        # thick aluminum rail face width (chunky)
FRAME_DEPTH = 0.160       # deep box section for dual-track profile

# Deep track grooves cut into the sill and head rails
TRACK_GROOVE_W = 0.030    # groove width along Y
TRACK_GROOVE_D = 0.032    # groove depth into rail (along Z)

# Wear rail: thin raised strip at groove floor (part of the frame solid)
WEAR_RAIL_T = 0.004       # wear rail thickness protruding from groove floor

MEETING_OVERLAP = 0.040   # the two sash stiles overlap at center

SASH_FACE = 0.068         # sash rail/stile face width
SASH_DEPTH = 0.050        # sash depth along Y
GLASS_T = 0.008           # glazing thickness

# Y layout: two parallel tracks in the deep frame
FIXED_SASH_Y = -0.030     # rear track center (fixed sash)
SLIDE_SASH_Y = 0.038      # front track center (sliding sash)

REBATE = 0.005            # glass tucks under the sash lip

# Latch hardware
LATCH_PLATE_W = 0.028
LATCH_PLATE_H = 0.075
LATCH_PLATE_T = 0.010
LATCH_LEVER_LEN = 0.045
LATCH_LEVER_R = 0.006

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

ALUMINUM_RGBA = (0.72, 0.74, 0.78, 1.0)   # brushed aluminum
GLASS_RGBA = (0.52, 0.60, 0.66, 0.30)     # cool grey-blue, semi-transparent
METAL_RGBA = (0.45, 0.47, 0.50, 1.0)      # dark anodized metal (latch)

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

# Groove X extent (slightly beyond inner width for sash travel)
GROOVE_X0 = INNER_X0 - 0.003
GROOVE_X1 = INNER_X1 + 0.003


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery), authored in meters.
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float, y_center: float, depth: float) -> cq.Workplane:
    """Axis-aligned box spanning [x0,x1] x [z0,z1] centered on y_center."""
    return (
        cq.Workplane("XY")
        .transformed(offset=((x0 + x1) / 2.0, y_center, (z0 + z1) / 2.0))
        .box(x1 - x0, depth, z1 - z0)
    )


def _build_frame_shape() -> cq.Workplane:
    """Thick aluminum frame with deep track grooves in sill and head rails.

    The frame is built as a solid slab cut by:
    1. The main sash opening (full inner clear region)
    2. Four deep track grooves (2 in sill, 2 in head) - one per track per rail
    Then thin wear rails are added at each groove floor as part of the same
    solid, representing the nylon/UHMW track wear strips.
    """
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    cut_depth = FRAME_DEPTH + 0.02
    # Main clear opening spanning the full inner region
    opening = _slab(INNER_X0, INNER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    frame = outer.cut(opening)

    # Deep track grooves in sill (bottom rail) - one per track
    for ty in (FIXED_SASH_Y, SLIDE_SASH_Y):
        sill_groove = _slab(
            GROOVE_X0, GROOVE_X1,
            INNER_Z0 - TRACK_GROOVE_D, INNER_Z0,
            ty, TRACK_GROOVE_W,
        )
        frame = frame.cut(sill_groove)

    # Deep track grooves in head (top rail) - one per track
    for ty in (FIXED_SASH_Y, SLIDE_SASH_Y):
        head_groove = _slab(
            GROOVE_X0, GROOVE_X1,
            INNER_Z1, INNER_Z1 + TRACK_GROOVE_D,
            ty, TRACK_GROOVE_W,
        )
        frame = frame.cut(head_groove)

    # Wear rails: thin raised strips at each groove floor/ceiling.
    # These are part of the same CadQuery solid (unioned), so no
    # disconnected geometry islands.
    groove_xw = GROOVE_X1 - GROOVE_X0
    for ty in (FIXED_SASH_Y, SLIDE_SASH_Y):
        # Sill wear rail: sits on the groove floor, protruding upward
        sill_rail_z0 = INNER_Z0 - TRACK_GROOVE_D
        sill_rail_z1 = sill_rail_z0 + WEAR_RAIL_T
        sill_rail = _slab(
            GROOVE_X0, GROOVE_X1,
            sill_rail_z0, sill_rail_z1,
            ty, TRACK_GROOVE_W,
        )
        frame = frame.union(sill_rail)

        # Head wear rail: sits at groove ceiling, protruding downward
        head_rail_z1 = INNER_Z1 + TRACK_GROOVE_D
        head_rail_z0 = head_rail_z1 - WEAR_RAIL_T
        head_rail = _slab(
            GROOVE_X0, GROOVE_X1,
            head_rail_z0, head_rail_z1,
            ty, TRACK_GROOVE_W,
        )
        frame = frame.union(head_rail)

    return frame


def _build_sash_shape() -> cq.Workplane:
    """Sash ring in its own local frame, centered at origin."""
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


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="aluminum_sliding_window")
    model.material("aluminum", rgba=ALUMINUM_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("metal", rgba=METAL_RGBA)

    # --- Static outer frame (root) with deep track grooves + wear rails ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="aluminum",
        name="frame_shell",
    )

    # --- Fixed (left) sash ---
    fixed_sash = model.part("fixed_sash")
    fixed_sash.visual(
        mesh_from_cadquery(_build_sash_shape(), "fixed_sash_frame"),
        material="aluminum",
        name="fixed_sash_frame",
    )
    fixed_sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "fixed_sash_glass"),
        material="glass",
        name="fixed_sash_glass",
    )

    # --- Sliding (right) sash with latch ---
    sliding_sash = model.part("sliding_sash")
    sliding_sash.visual(
        mesh_from_cadquery(_build_sash_shape(), "sliding_sash_frame"),
        material="aluminum",
        name="sliding_sash_frame",
    )
    sliding_sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "sliding_sash_glass"),
        material="glass",
        name="sliding_sash_glass",
    )

    # Latch keeper plate on the sliding sash meeting (inner/left) stile
    stile_x = -SASH_OPENING_W / 2.0 - SASH_FACE / 2.0
    face_y = SASH_DEPTH / 2.0
    plate_y = face_y + LATCH_PLATE_T / 2.0
    sliding_sash.visual(
        Box((LATCH_PLATE_W, LATCH_PLATE_T, LATCH_PLATE_H)),
        origin=Origin(xyz=(stile_x, plate_y, 0.0)),
        material="metal",
        name="sliding_sash_latch_plate",
    )
    # Lever arm (thumb-turn cam)
    lever_y = face_y + LATCH_PLATE_T + LATCH_LEVER_LEN / 2.0
    sliding_sash.visual(
        Cylinder(radius=LATCH_LEVER_R, length=LATCH_LEVER_LEN),
        origin=Origin(xyz=(stile_x, lever_y, -0.008), rpy=(1.5707963, 0.0, 0.0)),
        material="metal",
        name="sliding_sash_latch_lever",
    )

    # FIXED joint for left sash (seated in rear track)
    model.articulation(
        "frame_to_fixed_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="fixed_sash",
        origin=Origin(xyz=(FIXED_OPEN_CX, FIXED_SASH_Y, MID_CZ)),
    )

    # PRISMATIC joint for right sash: slides along X in the front track.
    # axis=(-1,0,0) so positive q OPENS (slides left toward fixed sash).
    slide_travel = SASH_OPENING_W * 0.88
    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="sliding_sash",
        origin=Origin(xyz=(SLIDE_OPEN_CX, SLIDE_SASH_Y, MID_CZ)),
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
    fixed_sash = object_model.get_part("fixed_sash")
    sliding_sash = object_model.get_part("sliding_sash")
    slide = object_model.get_articulation("frame_to_sliding_sash")

    # --- Intentional overlap allowances ---

    # Glass rebated under sash lip on each sash
    for nm in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_frame",
            reason="Clear pane is rebated under the sash lip (captured glazing).",
        )

    # Sash frames seated in the frame track grooves
    for nm in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_frame",
            reason=f"{nm} is rebated into the frame track groove (seated capture).",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_glass",
            reason=f"{nm} glass is rebated under the frame opening lip.",
        )

    # Latch mounted on sash stile
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="sliding_sash_latch_plate",
        elem_b="sliding_sash_frame",
        reason="Latch keeper plate is seated on the sliding-sash meeting stile.",
    )

    # --- Frame dimension checks (thick aluminum rails) ---
    frame_aabb = ctx.part_world_aabb(frame)
    frame_w = frame_aabb[1][0] - frame_aabb[0][0]
    frame_h = frame_aabb[1][2] - frame_aabb[0][2]
    frame_d = frame_aabb[1][1] - frame_aabb[0][1]

    ctx.check(
        "frame width matches aluminum slider",
        abs(frame_w - TOTAL_W) < 0.02,
        details=f"frame_w={frame_w:.3f}, expected={TOTAL_W:.3f}",
    )
    ctx.check(
        "frame height matches aluminum slider",
        abs(frame_h - TOTAL_H) < 0.02,
        details=f"frame_h={frame_h:.3f}, expected={TOTAL_H:.3f}",
    )
    ctx.check(
        "thick aluminum frame depth (deep dual-track profile)",
        frame_d >= FRAME_DEPTH - 0.01,
        details=f"frame_d={frame_d:.3f}, min_expected={FRAME_DEPTH - 0.01:.3f}",
    )
    ctx.check(
        "thick frame rail face (>= 80mm)",
        FRAME_FACE >= 0.080,
        details=f"FRAME_FACE={FRAME_FACE:.3f}",
    )

    # --- Deep track groove checks ---
    # The grooves are cut INTO the sill/head rails (internal cuts within the
    # frame body), so the frame AABB still spans z=0..TOTAL_H. We verify the
    # groove design parameters and that the sill/head rails are deep enough
    # to contain the grooves.
    ctx.check(
        "deep track grooves (>= 25mm)",
        TRACK_GROOVE_D >= 0.025,
        details=f"TRACK_GROOVE_D={TRACK_GROOVE_D:.3f}",
    )
    ctx.check(
        "sill rail deep enough for groove (groove fits within rail)",
        FRAME_FACE >= TRACK_GROOVE_D + 0.02,
        details=f"FRAME_FACE={FRAME_FACE:.3f}, TRACK_GROOVE_D={TRACK_GROOVE_D:.3f}",
    )
    ctx.check(
        "head rail deep enough for groove (groove fits within rail)",
        FRAME_FACE >= TRACK_GROOVE_D + 0.02,
        details=f"FRAME_FACE={FRAME_FACE:.3f}, TRACK_GROOVE_D={TRACK_GROOVE_D:.3f}",
    )
    # Sill and head grooves are inside the rail (groove floor > frame bottom,
    # groove ceiling < frame top).
    sill_groove_floor = INNER_Z0 - TRACK_GROOVE_D
    head_groove_ceiling = INNER_Z1 + TRACK_GROOVE_D
    ctx.check(
        "sill groove floor above frame bottom",
        sill_groove_floor > 0.0,
        details=f"sill_groove_floor={sill_groove_floor:.4f}",
    )
    ctx.check(
        "head groove ceiling below frame top",
        head_groove_ceiling < TOTAL_H,
        details=f"head_groove_ceiling={head_groove_ceiling:.4f}, TOTAL_H={TOTAL_H:.4f}",
    )

    # Two distinct tracks at different Y positions
    ctx.check(
        "dual tracks with Y separation",
        SLIDE_SASH_Y - FIXED_SASH_Y >= 0.050,
        details=f"track_separation={SLIDE_SASH_Y - FIXED_SASH_Y:.3f}",
    )

    # --- Prismatic joint checks ---
    ctx.check(
        "sliding joint is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )
    ctx.check(
        "prismatic joint has non-trivial travel",
        slide.motion_limits.upper > 0.30,
        details=f"upper={slide.motion_limits.upper:.3f}",
    )

    # Closed pose (q=0): window reads SHUT
    with ctx.pose({slide: 0.0}):
        f_aabb = ctx.part_world_aabb(fixed_sash)
        s_aabb = ctx.part_world_aabb(sliding_sash)

        # Fixed sash on left, sliding sash on right
        fx = (f_aabb[0][0] + f_aabb[1][0]) / 2.0
        sx = (s_aabb[0][0] + s_aabb[1][0]) / 2.0
        ctx.check(
            "fixed sash left of sliding sash",
            fx < sx,
            details=f"fixed_x={fx:.3f}, sliding_x={sx:.3f}",
        )

        # Sliding sash in front track (higher Y than fixed sash)
        fy = (f_aabb[0][1] + f_aabb[1][1]) / 2.0
        sy = (s_aabb[0][1] + s_aabb[1][1]) / 2.0
        ctx.check(
            "sliding sash in front track (proud of fixed)",
            sy > fy + 0.02,
            details=f"sliding_y={sy:.3f}, fixed_y={fy:.3f}",
        )

        # Both sashes seated within frame opening
        ctx.expect_overlap(
            fixed_sash, frame, axes="xz", min_overlap=0.03,
            name="fixed sash seated in frame track",
        )
        ctx.expect_overlap(
            sliding_sash, frame, axes="xz", min_overlap=0.03,
            name="sliding sash seated in frame track",
        )

        # Sashes within frame height (including groove extension)
        for nm, ab in (("fixed", f_aabb), ("sliding", s_aabb)):
            ctx.check(
                f"{nm} sash within frame height",
                ab[0][2] > frame_aabb[0][2] - 0.01 and ab[1][2] < frame_aabb[1][2] + 0.01,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}]",
            )

        rest_sx = sx
        rest_sz = (s_aabb[0][2] + s_aabb[1][2]) / 2.0

    # Open pose: sliding sash slides toward fixed sash (-X)
    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel}):
        s_open = ctx.part_world_aabb(sliding_sash)
        open_sx = (s_open[0][0] + s_open[1][0]) / 2.0

        # Positive q moves sash in -X (toward fixed sash)
        ctx.check(
            "sliding sash opens toward -X",
            open_sx < rest_sx - 0.30,
            details=f"rest_x={rest_sx:.3f}, open_x={open_sx:.3f}, travel={travel:.3f}",
        )

        # Pure horizontal slide (no Z change)
        open_sz = (s_open[0][2] + s_open[1][2]) / 2.0
        ctx.check(
            "pure horizontal slide (no Z drift)",
            abs(open_sz - rest_sz) < 0.02,
            details=f"open_z={open_sz:.3f}, rest_z={rest_sz:.3f}",
        )

        # Retained insertion: sash stays within frame X span
        ctx.check(
            "sash retained within frame at full travel",
            s_open[0][0] > frame_aabb[0][0] - 0.01 and s_open[1][0] < frame_aabb[1][0] + 0.01,
            details=f"sash x=[{s_open[0][0]:.3f},{s_open[1][0]:.3f}]",
        )
        ctx.expect_overlap(
            sliding_sash, frame, axes="z", min_overlap=0.10,
            name="sash retains track engagement at full travel",
        )

    return ctx.report()


object_model = build_object_model()
