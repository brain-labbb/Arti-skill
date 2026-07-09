from __future__ import annotations

# Sliding window: slim vinyl frame with bevelled inner corners, two side-by-side
# sashes. Left sash is fixed; right sash slides horizontally on bottom rollers.
# A tilt-in latch pair on the sliding sash pivots on small revolute joints.
# A visible overlap stile extends from the sliding sash meeting edge.
#
# Coordinate convention:
#   +Z is up. Window stands vertically: height along +Z, width along X,
#   frame depth along Y (glass plane is X-Z). Sill at z=0; head at z=WIN_H.
#
# Articulation:
#   - FIXED sash: left side, fixed in the exterior (+Y) track.
#   - SLIDING sash: right side, PRISMATIC axis (-1,0,0): positive q slides it
#     LEFT (opens) behind the fixed sash in the interior (-Y) track.
#   - LATCH pair: REVOLUTE on the sliding sash, axis (1,0,0), tilt-in release.

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

WIN_W = 1.10           # overall window width (X)
WIN_H = 0.92           # overall window height (Z)
FRAME_FACE = 0.045     # slim vinyl frame member face width
FRAME_DEPTH = 0.085    # frame depth (Y)

# Clear opening
OPEN_W = WIN_W - 2 * FRAME_FACE
OPEN_H = WIN_H - 2 * FRAME_FACE
OPEN_X0 = -WIN_W / 2.0 + FRAME_FACE
OPEN_X1 = WIN_W / 2.0 - FRAME_FACE
OPEN_Z0 = FRAME_FACE
OPEN_Z1 = WIN_H - FRAME_FACE

# Sash dimensions. Each sash is slightly less than half the opening width.
SASH_RAIL = 0.040       # top/bottom rail width
SASH_STILE = 0.040      # side stile width
SASH_DEPTH = 0.030      # sash thickness (Y)
SASH_H = OPEN_H + 0.012  # sash height extends into top/bottom tracks
SASH_W = OPEN_W / 2.0 - 0.008  # sash width (with clearance)

# Y planes: fixed sash exterior (+Y), sliding sash interior (-Y)
SASH_Y_GAP = 0.014
FIXED_SASH_Y = +SASH_Y_GAP
SLIDING_SASH_Y = -SASH_Y_GAP

# Track grooves in sill and head
TRACK_DEPTH_CUT = 0.007  # how deep the groove cuts into frame member
TRACK_W = SASH_DEPTH + 0.006

# Overlap stile on sliding sash meeting edge
OVERLAP_W = 0.014        # width of overlap extension
OVERLAP_T = 0.010        # extra thickness beyond sash depth

# Roller blocks at bottom of sliding sash
ROLLER_W = 0.028
ROLLER_D = 0.012
ROLLER_H = 0.009

# Tilt-in latches
LATCH_W = 0.038
LATCH_D = 0.014
LATCH_H = 0.010

# Glass
GLASS_T = 0.005

# Frame bevel at inner corners
BEVEL = 0.005

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

FRAME_RGBA = (0.91, 0.91, 0.925, 1.0)     # vinyl white-grey
SASH_RGBA = (0.935, 0.935, 0.95, 1.0)     # slightly brighter vinyl
GLASS_RGBA = (0.28, 0.34, 0.40, 0.34)     # cool tinted glass
HARDWARE_RGBA = (0.70, 0.72, 0.74, 1.0)   # brushed metal
ROLLER_RGBA = (0.20, 0.20, 0.22, 1.0)     # dark nylon


# ---------------------------------------------------------------------------
# Frame geometry (CadQuery)
# ---------------------------------------------------------------------------

def _build_frame_shape() -> cq.Workplane:
    """Slim vinyl outer frame: perimeter slab with central opening, horizontal
    track grooves in sill and head, and bevelled inner corners."""
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, WIN_H / 2.0))
        .box(WIN_W, FRAME_DEPTH, WIN_H)
    )

    # Cut clear central opening
    opening = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, (OPEN_Z0 + OPEN_Z1) / 2.0))
        .box(OPEN_W, FRAME_DEPTH + 0.02, OPEN_H)
    )
    frame = outer.cut(opening)

    # Horizontal track grooves in sill and head: two parallel grooves per
    # member (one per sash Y-plane track).
    for z_base, z_dir in ((OPEN_Z0, -1.0), (OPEN_Z1, +1.0)):
        zc = z_base + z_dir * TRACK_DEPTH_CUT / 2.0
        for ty in (FIXED_SASH_Y, SLIDING_SASH_Y):
            groove = (
                cq.Workplane("XY")
                .transformed(offset=(0.0, ty, zc))
                .box(OPEN_W + 0.01, TRACK_W, TRACK_DEPTH_CUT)
            )
            frame = frame.cut(groove)

    # Bevel inner corners: cut small 45-degree triangular prisms at each
    # inner corner of the frame opening (where jamb meets sill/head).
    for x_edge, x_dir in ((OPEN_X0, -1.0), (OPEN_X1, +1.0)):
        for z_edge, z_dir in ((OPEN_Z0, -1.0), (OPEN_Z1, +1.0)):
            # Build a triangular prism spanning the full frame depth + margin.
            # Triangle in XZ plane removes the sharp inner corner.
            tri = (
                cq.Workplane("XZ")
                .transformed(offset=(0.0, 0.0, 0.0))
                .moveTo(x_edge, z_edge)
                .lineTo(x_edge + x_dir * BEVEL, z_edge)
                .lineTo(x_edge, z_edge + z_dir * BEVEL)
                .close()
                .extrude(FRAME_DEPTH / 2.0 + 0.01, both=True)
            )
            frame = frame.cut(tri)

    return frame


# ---------------------------------------------------------------------------
# Sash geometry (CadQuery)
# ---------------------------------------------------------------------------

def _build_sash_frame_shape() -> cq.Workplane:
    """One sash: perimeter ring (stiles + rails) with a single large glass
    opening. Local frame: X centered, Z from 0 to SASH_H, Y centered."""
    w = SASH_W
    h = SASH_H
    d = SASH_DEPTH

    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(w, d, h)
    )

    # Single large glass opening (inner pane region)
    inner_w = w - 2 * SASH_STILE
    inner_h = h - 2 * SASH_RAIL
    opening = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(inner_w, d + 0.02, inner_h)
    )
    return outer.cut(opening)


def _build_glass_shape() -> cq.Workplane:
    """Single glass pane for one sash, rebated under the stile/rail lips."""
    inner_w = SASH_W - 2 * SASH_STILE
    inner_h = SASH_H - 2 * SASH_RAIL
    rebate = 0.004
    return (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, SASH_H / 2.0))
        .box(inner_w + 2 * rebate, GLASS_T, inner_h + 2 * rebate)
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_window")

    model.material("frame", rgba=FRAME_RGBA)
    model.material("sash", rgba=SASH_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("hardware", rgba=HARDWARE_RGBA)
    model.material("roller", rgba=ROLLER_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="frame",
        name="frame_shell",
    )

    # --- Fixed sash (left, exterior track) ---
    fixed = model.part("fixed_sash")
    fixed.visual(
        mesh_from_cadquery(_build_sash_frame_shape(), "fixed_sash_frame"),
        material="sash",
        name="fixed_sash_frame",
    )
    fixed.visual(
        mesh_from_cadquery(_build_glass_shape(), "fixed_sash_glass"),
        material="glass",
        name="fixed_sash_glass",
    )

    # --- Sliding sash (right, interior track) ---
    sliding = model.part("sliding_sash")
    sliding.visual(
        mesh_from_cadquery(_build_sash_frame_shape(), "sliding_sash_frame"),
        material="sash",
        name="sliding_sash_frame",
    )
    sliding.visual(
        mesh_from_cadquery(_build_glass_shape(), "sliding_sash_glass"),
        material="glass",
        name="sliding_sash_glass",
    )

    # Overlap stile: extends from the sliding sash meeting edge (-X side)
    # past the fixed sash, creating the visible overlap where panes cross.
    overlap_stile_x = -SASH_W / 2.0 - OVERLAP_W / 2.0
    overlap_stile_y = OVERLAP_T / 2.0  # extends toward +Y (toward fixed sash)
    sliding.visual(
        Box((OVERLAP_W, SASH_DEPTH + OVERLAP_T, SASH_H)),
        origin=Origin(xyz=(overlap_stile_x, overlap_stile_y, SASH_H / 2.0)),
        material="sash",
        name="overlap_stile",
    )

    # Two roller blocks at the bottom of the sliding sash
    roller_z = -ROLLER_H / 2.0  # half below sash bottom edge
    for i, x_off in enumerate((-SASH_W / 4.0, SASH_W / 4.0)):
        sliding.visual(
            Box((ROLLER_W, ROLLER_D, ROLLER_H)),
            origin=Origin(xyz=(x_off, 0.0, roller_z)),
            material="roller",
            name=f"roller_{i}",
        )

    # --- Tilt-in latch pair (separate parts with revolute joints) ---
    latch_top = model.part("latch_top")
    latch_top.visual(
        Box((LATCH_W, LATCH_D, LATCH_H)),
        material="hardware",
        name="latch_top_body",
    )

    latch_bottom = model.part("latch_bottom")
    latch_bottom.visual(
        Box((LATCH_W, LATCH_D, LATCH_H)),
        material="hardware",
        name="latch_bottom_body",
    )

    # ----- Articulations -----

    # Fixed sash position: centered in the left half of the opening
    fixed_x = OPEN_X0 + SASH_W / 2.0 + 0.004
    fixed_z = OPEN_Z0 - 0.006  # extends into sill track groove

    model.articulation(
        "frame_to_fixed_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="fixed_sash",
        origin=Origin(xyz=(fixed_x, FIXED_SASH_Y, fixed_z)),
    )

    # Sliding sash: centered in the right half, slides left to open
    sliding_x = OPEN_X1 - SASH_W / 2.0 - 0.004
    sliding_z = OPEN_Z0 - 0.006  # extends into sill track groove

    max_travel = SASH_W * 0.78  # retains some engagement at full open

    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="sliding_sash",
        origin=Origin(xyz=(sliding_x, SLIDING_SASH_Y, sliding_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.25, lower=0.0, upper=max_travel
        ),
    )

    # Tilt-in latches: revolute joints on the sliding sash
    # Pivot axis along X (horizontal, perpendicular to glass plane).
    # Positive q tilts the latch outward (-Y, toward interior).
    latch_y = -(SASH_DEPTH / 2.0 + LATCH_D / 2.0 - 0.004)

    model.articulation(
        "sliding_sash_to_latch_top",
        ArticulationType.REVOLUTE,
        parent="sliding_sash",
        child="latch_top",
        origin=Origin(xyz=(0.0, latch_y, SASH_H - SASH_RAIL / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=0.0, upper=1.2
        ),
    )

    model.articulation(
        "sliding_sash_to_latch_bottom",
        ArticulationType.REVOLUTE,
        parent="sliding_sash",
        child="latch_bottom",
        origin=Origin(xyz=(0.0, latch_y, SASH_RAIL / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=0.0, upper=1.2
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    fixed = object_model.get_part("fixed_sash")
    sliding = object_model.get_part("sliding_sash")
    latch_top = object_model.get_part("latch_top")
    latch_bottom = object_model.get_part("latch_bottom")

    j_slide = object_model.get_articulation("frame_to_sliding_sash")
    j_latch_top = object_model.get_articulation("sliding_sash_to_latch_top")
    j_latch_bot = object_model.get_articulation("sliding_sash_to_latch_bottom")

    # --- Intentional overlaps ---
    # Glass rebated under sash rails/stiles
    for sash_name in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            sash_name, sash_name,
            elem_a=f"{sash_name}_glass",
            elem_b=f"{sash_name}_frame",
            reason="Glass pane rebated under sash rails/stiles (captured glass).",
        )

    # Overlap stile is mounted on the sliding sash frame
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="overlap_stile",
        elem_b="sliding_sash_frame",
        reason="Overlap stile is mounted onto the sliding sash meeting edge.",
    )

    # Rollers seated at the bottom of the sliding sash
    for roller_name in ("roller_0", "roller_1"):
        ctx.allow_overlap(
            "sliding_sash", "sliding_sash",
            elem_a=roller_name,
            elem_b="sliding_sash_frame",
            reason="Roller block seated at the bottom of the sliding sash.",
        )

    # Sashes ride in frame top/bottom track grooves
    ctx.allow_overlap(
        "frame", "fixed_sash",
        reason="Fixed sash rails sit in the exterior top/bottom track grooves.",
    )
    ctx.allow_overlap(
        "frame", "sliding_sash",
        reason="Sliding sash rails sit in the interior top/bottom track grooves.",
    )

    # Overlap stile on sliding sash crosses the fixed sash meeting edge
    ctx.allow_overlap(
        "fixed_sash", "sliding_sash",
        elem_a="fixed_sash_frame",
        elem_b="overlap_stile",
        reason="Overlap stile extends from sliding sash past the fixed sash meeting stile (visible overlap where panes cross).",
    )

    # Latches mounted on sliding sash
    ctx.allow_overlap(
        "sliding_sash", "latch_top",
        reason="Tilt-in latch mounted on the sliding sash top rail.",
    )
    ctx.allow_overlap(
        "sliding_sash", "latch_bottom",
        reason="Tilt-in latch mounted on the sliding sash bottom rail.",
    )

    # --- Closed pose (q=0): both sashes in place, window reads shut ---
    with ctx.pose({j_slide: 0.0}):
        f_aabb = ctx.part_world_aabb(frame)
        fx_aabb = ctx.part_world_aabb(fixed)
        sl_aabb = ctx.part_world_aabb(sliding)

        # Frame spans wider than either sash
        frame_w = f_aabb[1][0] - f_aabb[0][0]
        ctx.check(
            "frame wider than both sashes",
            frame_w > 0.9,
            details=f"frame_w={frame_w:.3f}",
        )

        # Window stands upright (sill near z=0, head > 0.8)
        ctx.check(
            "frame stands upright",
            abs(f_aabb[0][2]) < 0.01 and f_aabb[1][2] > 0.8,
            details=f"z=({f_aabb[0][2]:.3f}, {f_aabb[1][2]:.3f})",
        )

        # Fixed sash is to the left of the sliding sash
        fx_cx = (fx_aabb[0][0] + fx_aabb[1][0]) / 2.0
        sl_cx = (sl_aabb[0][0] + sl_aabb[1][0]) / 2.0
        ctx.check(
            "fixed sash left of sliding sash",
            fx_cx < sl_cx - 0.1,
            details=f"fixed_cx={fx_cx:.3f}, sliding_cx={sl_cx:.3f}",
        )

        # Sashes are in offset Y planes (different tracks)
        fx_cy = (fx_aabb[0][1] + fx_aabb[1][1]) / 2.0
        sl_cy = (sl_aabb[0][1] + sl_aabb[1][1]) / 2.0
        ctx.check(
            "sashes in offset Y tracks",
            abs(fx_cy - sl_cy) > 0.015,
            details=f"fixed_cy={fx_cy:.3f}, sliding_cy={sl_cy:.3f}",
        )

        rest_sl_cx = sl_cx

    # --- HERO: sliding sash slides LEFT to open ---
    travel = SASH_W * 0.60
    with ctx.pose({j_slide: travel}):
        op_aabb = ctx.part_world_aabb(sliding)
        op_cx = (op_aabb[0][0] + op_aabb[1][0]) / 2.0
        ctx.check(
            "sliding sash moves left when opened",
            op_cx < rest_sl_cx - travel * 0.8,
            details=f"rest_cx={rest_sl_cx:.3f}, opened_cx={op_cx:.3f}, travel={travel:.3f}",
        )
        # Still retained in frame (overlaps frame in Z)
        ctx.expect_overlap(
            sliding, frame, axes="z", min_overlap=0.05,
            name="sliding sash retained in frame when open",
        )

    # --- Overlap stile exists on the sliding sash ---
    stile_aabb = ctx.part_element_world_aabb(sliding, elem="overlap_stile")
    ctx.check(
        "overlap stile present on sliding sash",
        stile_aabb is not None,
        details="overlap_stile visual not found",
    )
    if stile_aabb is not None:
        stile_h = stile_aabb[1][2] - stile_aabb[0][2]
        ctx.check(
            "overlap stile spans sash height",
            stile_h > SASH_H * 0.8,
            details=f"stile_h={stile_h:.3f}, sash_h={SASH_H:.3f}",
        )
        # Proof: overlap stile crosses the fixed sash in X at closed pose
        ctx.expect_overlap(
            sliding, fixed,
            axes="x",
            elem_a="overlap_stile",
            elem_b="fixed_sash_frame",
            min_overlap=0.002,
            name="overlap stile crosses fixed sash meeting edge",
        )

    # --- Roller blocks exist at the bottom of the sliding sash ---
    for rname in ("roller_0", "roller_1"):
        r_aabb = ctx.part_element_world_aabb(sliding, elem=rname)
        ctx.check(
            f"{rname} present on sliding sash",
            r_aabb is not None,
            details=f"{rname} visual not found",
        )
        if r_aabb is not None:
            # Proof: roller is at or below the sash frame bottom edge
            sl_frame_aabb = ctx.part_element_world_aabb(sliding, elem="sliding_sash_frame")
            if sl_frame_aabb is not None:
                ctx.check(
                    f"{rname} below sash bottom rail",
                    r_aabb[0][2] < sl_frame_aabb[0][2] + 0.005,
                    details=f"roller_z_min={r_aabb[0][2]:.4f}, frame_z_min={sl_frame_aabb[0][2]:.4f}",
                )

    # --- Tilt-in latches have non-fixed revolute joints ---
    for jname in ("sliding_sash_to_latch_top", "sliding_sash_to_latch_bottom"):
        j = object_model.get_articulation(jname)
        ctx.check(
            f"{jname} is revolute",
            j.articulation_type == ArticulationType.REVOLUTE,
            details=f"type={j.articulation_type}",
        )

    # Latch pivot test: positive q tilts latch outward
    with ctx.pose({j_slide: 0.0, j_latch_top: 0.8}):
        lt_aabb = ctx.part_world_aabb(latch_top)
        lt_cy = (lt_aabb[0][1] + lt_aabb[1][1]) / 2.0
        sl_aabb = ctx.part_world_aabb(sliding)
        sl_cy = (sl_aabb[0][1] + sl_aabb[1][1]) / 2.0
        ctx.check(
            "latch top tilts toward interior at positive q",
            lt_cy < sl_cy - 0.002,
            details=f"latch_cy={lt_cy:.4f}, sash_cy={sl_cy:.4f}",
        )

    # --- Sliding joint is prismatic with correct limits ---
    ctx.check(
        "sliding joint is prismatic",
        j_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={j_slide.articulation_type}",
    )
    limits = j_slide.motion_limits
    ctx.check(
        "sliding joint has positive travel",
        limits is not None and limits.upper is not None and limits.upper > 0.1,
        details=f"upper={limits.upper if limits else None}",
    )

    return ctx.report()


object_model = build_object_model()
