from __future__ import annotations

# Two-panel horizontal sliding window VARIANT (fork of parent patio-slider).
# White vinyl frame, one FIXED sash (left) + one SLIDING sash (right).
#
# Variant 20 changes from parent:
#   1. Sliding sash partially open at rest (q=0), showing visible overlap
#   2. Insect screen on independent shallow prismatic joint (exterior track)
#   3. Two roller blocks at the bottom of the moving sash
#   4. Visible overlap stile fin where the two sash meeting stiles cross
#
# Coordinate convention:
#   +Z up, window stands vertically
#     width  -> X,  height -> Z (sill near z=0),  frame depth -> Y
#   Glass plane is X-Z. q=0 reads PARTIALLY OPEN (variant). Positive prismatic
#   q slides the sash further toward the fixed sash (-X) via axis=(-1,0,0).

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

TOTAL_W = 1.52
TOTAL_H = 1.72

FRAME_FACE = 0.085
FRAME_DEPTH = 0.140

MEETING_OVERLAP = 0.040

SASH_FACE = 0.075
SASH_DEPTH = 0.060
GLASS_T = 0.008

FIXED_SASH_Y = -0.028
SLIDE_SASH_Y = 0.044

REBATE = 0.005

# Latch hardware
LATCH_PLATE_W = 0.028
LATCH_PLATE_H = 0.075
LATCH_PLATE_T = 0.010
LATCH_LEVER_LEN = 0.045
LATCH_LEVER_R = 0.006

# --- Variant-specific dimensions ---

# Sliding sash is partially open at q=0 by this offset (meters toward -X)
PARTIAL_OPEN = 0.22

# Overlap stile fin: a thin vertical bar on the sliding sash meeting stile
# that extends toward the fixed sash plane, making the overlap crossing visible
OVERLAP_FIN_W = 0.014        # fin width along X
OVERLAP_FIN_EMBED = 0.003    # how far the fin embeds into the sash back face (for connectivity)
OVERLAP_FIN_EXTEND = 0.010   # how far the fin extends past the sash back face toward fixed sash

# Roller blocks at bottom of sliding sash
ROLLER_W = 0.030
ROLLER_H = 0.014
ROLLER_D = 0.016

# Insect screen (exterior track, independent prismatic joint)
SCREEN_FRAME_W = 0.022       # screen frame member face width
SCREEN_FRAME_D = 0.018       # screen frame depth (Y thickness)
SCREEN_Y = -0.070            # screen track center Y (exterior side, behind fixed sash)
SCREEN_TRAVEL = 0.35          # shallow independent slide
SCREEN_MESH_T = 0.002        # screen mesh thickness

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
SLIDE_OPEN_CX_CLOSED = INNER_X1 - SASH_OPENING_W / 2.0
# At q=0 the sash is already partially open: shift origin toward -X
SLIDE_OPEN_CX = SLIDE_OPEN_CX_CLOSED - PARTIAL_OPEN
MID_CZ = (INNER_Z0 + INNER_Z1) / 2.0

# Screen dimensions (covers roughly one sash width)
SCREEN_OPEN_W = SASH_OPENING_W - 0.010
SCREEN_OPEN_H = SASH_OPENING_H - 0.010
# Screen centered on the right half at rest (covers the opening behind sliding sash)
SCREEN_CX = SLIDE_OPEN_CX_CLOSED

# Slide travel remaining from partially-open position
slide_travel = SASH_OPENING_W * 0.90 - PARTIAL_OPEN

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)
GLASS_RGBA = (0.52, 0.60, 0.66, 0.30)
METAL_RGBA = (0.74, 0.76, 0.79, 1.0)
SCREEN_RGBA = (0.30, 0.33, 0.36, 0.50)       # dark grey mesh, semi-transparent
ALUMINUM_RGBA = (0.68, 0.71, 0.74, 1.0)       # anodized aluminum screen frame
ROLLER_RGBA = (0.18, 0.18, 0.20, 1.0)         # dark nylon roller blocks


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery)
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float, y_center: float, depth: float) -> cq.Workplane:
    """Axis-aligned box in the X-Z plane, centered on y_center with given Y depth."""
    return (
        cq.Workplane("XY")
        .transformed(offset=((x0 + x1) / 2.0, y_center, (z0 + z1) / 2.0))
        .box(x1 - x0, depth, z1 - z0)
    )


def _build_frame_shape() -> cq.Workplane:
    """Static outer frame: thick slab cut by the full inner opening."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    cut_depth = FRAME_DEPTH + 0.02
    opening = _slab(INNER_X0, INNER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    return outer.cut(opening)


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
    """Clear pane filling the sash opening, rebated under the sash lip."""
    ow = SASH_OPENING_W + 2 * REBATE
    oh = SASH_OPENING_H + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_overlap_fin_shape() -> cq.Workplane:
    """Overlap stile fin on the sliding sash's meeting (left) stile.
    A thin vertical bar that extends from the sash back face toward the fixed sash
    plane, making the overlap crossing clearly visible.
    In sash-local frame: the meeting stile center is at x = -SASH_OPENING_W/2 - SASH_FACE/2.
    The fin embeds slightly into the sash back face (for mesh connectivity) and
    extends past it toward -Y (toward the fixed sash).
    The fin height stays within the frame opening to avoid frame overlap."""
    stile_x = -SASH_OPENING_W / 2.0 - SASH_FACE / 2.0
    fin_x0 = stile_x - OVERLAP_FIN_W / 2.0
    fin_x1 = stile_x + OVERLAP_FIN_W / 2.0
    # Fin extends from slightly inside the sash back face (-Y side) toward -Y
    fin_y0 = -(SASH_DEPTH / 2.0) + OVERLAP_FIN_EMBED   # embed for connectivity
    fin_y1 = -(SASH_DEPTH / 2.0) - OVERLAP_FIN_EXTEND   # extend past back face
    # Keep fin within frame opening in Z (avoid sill/head collision)
    # Frame opening Z in sash-local: [-INNER_H/2, INNER_H/2] with small margin
    fin_z0 = -INNER_H / 2.0 + 0.005
    fin_z1 = INNER_H / 2.0 - 0.005
    fin_depth = abs(fin_y1 - fin_y0)
    return _slab(fin_x0, fin_x1, fin_z0, fin_z1, (fin_y0 + fin_y1) / 2.0, fin_depth)


def _build_screen_frame_shape() -> cq.Workplane:
    """Screen frame ring in its own local frame, centered at origin.
    Thin aluminum frame for the insect screen panel."""
    ow = SCREEN_OPEN_W
    oh = SCREEN_OPEN_H
    out_w = ow + 2 * SCREEN_FRAME_W
    out_h = oh + 2 * SCREEN_FRAME_W
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SCREEN_FRAME_D)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SCREEN_FRAME_D + 0.01)
    return outer.cut(opening)


def _build_screen_mesh_shape() -> cq.Workplane:
    """Thin screen mesh panel filling the screen frame opening."""
    return _slab(
        -SCREEN_OPEN_W / 2.0, SCREEN_OPEN_W / 2.0,
        -SCREEN_OPEN_H / 2.0, SCREEN_OPEN_H / 2.0,
        0.0, SCREEN_MESH_T,
    )


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


def _add_overlap_fin(model: ArticulatedObject, sash_name: str) -> None:
    """Add the visible overlap stile fin on the sliding sash meeting stile."""
    sash = model.get_part(sash_name)
    stile_x = -SASH_OPENING_W / 2.0 - SASH_FACE / 2.0
    sash.visual(
        mesh_from_cadquery(_build_overlap_fin_shape(), f"{sash_name}_overlap_fin"),
        material="vinyl",
        name=f"{sash_name}_overlap_fin",
    )


def _add_roller_blocks(model: ArticulatedObject, sash_name: str) -> None:
    """Add two small roller blocks at the bottom rail of the sliding sash."""
    sash = model.get_part(sash_name)
    # Bottom rail center Z in sash-local frame
    bottom_z = -SASH_OPENING_H / 2.0 - SASH_FACE / 2.0
    # Roller sits at the bottom of the rail, touching the sill track
    roller_z = -SASH_OPENING_H / 2.0 - SASH_FACE + ROLLER_H / 2.0
    # Y center: roller sits in the sill track groove (centered on sash depth)
    roller_y = 0.0
    # Two rollers at 1/4 and 3/4 of the sash opening width
    quarter_x = SASH_OPENING_W / 4.0
    for i, x_off in enumerate((-quarter_x, quarter_x)):
        sash.visual(
            Box((ROLLER_W, ROLLER_D, ROLLER_H)),
            origin=Origin(xyz=(x_off, roller_y, roller_z)),
            material="roller",
            name=f"{sash_name}_roller_{i}",
        )


def _add_insect_screen(model: ArticulatedObject) -> None:
    """Add the insect screen part (aluminum frame + mesh fill)."""
    screen = model.part("insect_screen")
    screen.visual(
        mesh_from_cadquery(_build_screen_frame_shape(), "screen_frame"),
        material="aluminum",
        name="screen_frame",
    )
    screen.visual(
        mesh_from_cadquery(_build_screen_mesh_shape(), "screen_mesh"),
        material="screen_mesh",
        name="screen_mesh",
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_window_variant")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("metal", rgba=METAL_RGBA)
    model.material("aluminum", rgba=ALUMINUM_RGBA)
    model.material("screen_mesh", rgba=SCREEN_RGBA)
    model.material("roller", rgba=ROLLER_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="vinyl",
        name="frame_shell",
    )

    # --- Fixed (left) + sliding (right) sashes ---
    _add_sash(model, "fixed_sash")
    _add_sash(model, "sliding_sash")
    _add_latch(model, "sliding_sash")
    _add_overlap_fin(model, "sliding_sash")
    _add_roller_blocks(model, "sliding_sash")

    # --- Insect screen ---
    _add_insect_screen(model)

    # FIXED left sash seated in the rear glazing plane
    model.articulation(
        "frame_to_fixed_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="fixed_sash",
        origin=Origin(xyz=(FIXED_OPEN_CX, FIXED_SASH_Y, MID_CZ)),
    )

    # SLIDING right sash: PRISMATIC along X. At q=0 the sash is already
    # partially open (PARTIAL_OPEN offset baked into the origin).
    # axis=(-1,0,0) so positive q slides further toward -X (more open).
    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="sliding_sash",
        origin=Origin(xyz=(SLIDE_OPEN_CX, SLIDE_SASH_Y, MID_CZ)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5, lower=0.0, upper=slide_travel),
    )

    # INSECT SCREEN: independent shallow PRISMATIC along X on the exterior track.
    # axis=(-1,0,0) so positive q slides the screen left (same direction convention).
    model.articulation(
        "frame_to_screen",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="insect_screen",
        origin=Origin(xyz=(SCREEN_CX, SCREEN_Y, MID_CZ)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.4, lower=0.0, upper=SCREEN_TRAVEL),
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
    screen = object_model.get_part("insect_screen")
    slide = object_model.get_articulation("frame_to_sliding_sash")
    screen_slide = object_model.get_articulation("frame_to_screen")

    # --- Verify variant-specific parts exist ---
    ctx.check(
        "insect_screen part exists",
        screen is not None,
        details="insect_screen part not found",
    )
    ctx.check(
        "sliding sash has overlap fin",
        sliding_sash.get_visual("sliding_sash_overlap_fin") is not None,
        details="sliding_sash_overlap_fin visual not found",
    )
    ctx.check(
        "sliding sash has roller block 0",
        sliding_sash.get_visual("sliding_sash_roller_0") is not None,
        details="sliding_sash_roller_0 visual not found",
    )
    ctx.check(
        "sliding sash has roller block 1",
        sliding_sash.get_visual("sliding_sash_roller_1") is not None,
        details="sliding_sash_roller_1 visual not found",
    )

    # --- Verify non-fixed joints exist ---
    ctx.check(
        "sliding sash has prismatic joint",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"got {slide.articulation_type}",
    )
    ctx.check(
        "screen has prismatic joint",
        screen_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"got {screen_slide.articulation_type}",
    )
    ctx.check(
        "screen joint has limits",
        screen_slide.motion_limits is not None
        and screen_slide.motion_limits.upper is not None
        and screen_slide.motion_limits.upper > 0.0,
        details="screen prismatic joint must have positive upper limit",
    )

    # --- Intentional overlaps ---
    # Glass rebated under sash lip
    for nm in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_vinyl",
            reason="Clear pane is rebated under the sash lip (captured glazing).",
        )
    # Sash rings rebated into frame opening / track
    for nm in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_vinyl",
            reason=f"{nm} ring is rebated into the frame opening / head-sill track.",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_glass",
            reason=f"{nm} glass is rebated under the frame opening lip.",
        )
    # Latch keeper plate seated on sash
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="sliding_sash_latch_plate",
        elem_b="sliding_sash_vinyl",
        reason="Latch keeper plate seated onto the sliding-sash meeting-stile face.",
    )
    # Overlap fin extends from sash into the gap between sash planes
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="sliding_sash_overlap_fin",
        elem_b="sliding_sash_vinyl",
        reason="Overlap fin is mounted on and extends from the sash meeting stile.",
    )
    # Roller blocks embedded slightly in the bottom rail
    for i in (0, 1):
        ctx.allow_overlap(
            "sliding_sash", "sliding_sash",
            elem_a=f"sliding_sash_roller_{i}",
            elem_b="sliding_sash_vinyl",
            reason=f"Roller block {i} is seated into the bottom rail of the sliding sash.",
        )
    # Screen frame and mesh overlap
    ctx.allow_overlap(
        "insect_screen", "insect_screen",
        elem_a="screen_mesh",
        elem_b="screen_frame",
        reason="Screen mesh is captured inside the screen frame ring.",
    )
    # Screen sits in the frame track (small rebate into frame)
    ctx.allow_overlap(
        "frame", "insect_screen",
        elem_a="frame_shell",
        elem_b="screen_frame",
        reason="Screen frame rides in the exterior frame track channel.",
    )
    ctx.allow_overlap(
        "frame", "insect_screen",
        elem_a="frame_shell",
        elem_b="screen_mesh",
        reason="Screen mesh passes through the frame track region.",
    )

    # --- Rest pose (q=0): sash is PARTIALLY OPEN ---
    with ctx.pose({slide: 0.0, screen_slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        f_aabb = ctx.part_world_aabb(fixed_sash)
        s_aabb = ctx.part_world_aabb(sliding_sash)
        scr_aabb = ctx.part_world_aabb(screen)

        # Frame proportions
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        sash_w = s_aabb[1][0] - s_aabb[0][0]
        ctx.check(
            "frame spans wider than a single sash",
            frame_w > sash_w + 0.40,
            details=f"frame_w={frame_w:.3f}, sash_w={sash_w:.3f}",
        )
        ctx.check(
            "sill sits near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"frame zmin={frame_aabb[0][2]:.4f}",
        )

        # Fixed sash left, sliding sash right
        fx = (f_aabb[0][0] + f_aabb[1][0]) / 2.0
        sx = (s_aabb[0][0] + s_aabb[1][0]) / 2.0
        ctx.check(
            "fixed sash left of sliding sash",
            fx < sx,
            details=f"fixed_x={fx:.3f}, sliding_x={sx:.3f}",
        )

        # Sliding sash is partially open: its center should be offset from
        # the fully-closed position (SLIDE_OPEN_CX_CLOSED) by at least PARTIAL_OPEN - tol
        ctx.check(
            "sliding sash partially open at rest",
            sx < SLIDE_OPEN_CX_CLOSED - PARTIAL_OPEN * 0.5,
            details=f"sliding_x={sx:.3f}, closed_x={SLIDE_OPEN_CX_CLOSED:.3f}",
        )

        # Sliding sash proud of fixed sash in Y
        fy = (f_aabb[0][1] + f_aabb[1][1]) / 2.0
        sy = (s_aabb[0][1] + s_aabb[1][1]) / 2.0
        ctx.check(
            "sliding sash proud of fixed sash",
            sy > fy + 0.02,
            details=f"sliding_y={sy:.3f}, fixed_y={fy:.3f}",
        )

        # Overlap fin is between the two sash Y planes (bridges the gap)
        fin_aabb = ctx.part_element_world_aabb(sliding_sash, elem="sliding_sash_overlap_fin")
        fin_cy = (fin_aabb[0][1] + fin_aabb[1][1]) / 2.0
        ctx.check(
            "overlap fin bridges sash Y gap",
            fin_cy < sy and fin_cy > fy - 0.02,
            details=f"fin_y={fin_cy:.3f}, sliding_y={sy:.3f}, fixed_y={fy:.3f}",
        )

        # Roller blocks at the bottom of the sliding sash
        for i in (0, 1):
            r_aabb = ctx.part_element_world_aabb(sliding_sash, elem=f"sliding_sash_roller_{i}")
            r_cz = (r_aabb[0][2] + r_aabb[1][2]) / 2.0
            s_bottom = s_aabb[0][2]
            ctx.check(
                f"roller_{i} near bottom of sliding sash",
                r_cz < s_bottom + 0.04,
                details=f"roller_z={r_cz:.3f}, sash_bottom={s_bottom:.3f}",
            )

        # Insect screen is on the exterior (-Y) side, behind the fixed sash
        scr_cy = (scr_aabb[0][1] + scr_aabb[1][1]) / 2.0
        ctx.check(
            "screen on exterior side of window",
            scr_cy < fy - 0.01,
            details=f"screen_y={scr_cy:.3f}, fixed_y={fy:.3f}",
        )
        # Screen is roughly the right height (within frame)
        scr_h = scr_aabb[1][2] - scr_aabb[0][2]
        ctx.check(
            "screen spans most of the window height",
            scr_h > INNER_H * 0.80,
            details=f"screen_h={scr_h:.3f}, inner_h={INNER_H:.3f}",
        )

        # Both sashes seated in frame
        ctx.expect_overlap(
            fixed_sash, frame, axes="xz", min_overlap=0.03,
            name="fixed sash seated in frame opening",
        )
        ctx.expect_overlap(
            sliding_sash, frame, axes="xz", min_overlap=0.03,
            name="sliding sash seated in frame opening",
        )

        rest_sx = sx

    # --- Sliding sash further open pose ---
    travel = slide.motion_limits.upper
    if travel > 0.01:
        with ctx.pose({slide: travel, screen_slide: 0.0}):
            s_open = ctx.part_world_aabb(sliding_sash)
            open_sx = (s_open[0][0] + s_open[1][0]) / 2.0
            # Sash moved further toward -X
            ctx.check(
                "sliding sash opens further toward -X",
                open_sx < rest_sx - 0.05,
                details=f"rest_x={rest_sx:.3f}, open_x={open_sx:.3f}",
            )
            # Pure horizontal slide (no Z change)
            open_sz = (s_open[0][2] + s_open[1][2]) / 2.0
            rest_sz = MID_CZ
            ctx.check(
                "slide is purely horizontal",
                abs(open_sz - rest_sz) < 0.02,
                details=f"open_z={open_sz:.3f}, rest_z={rest_sz:.3f}",
            )
            # Retained within frame X span
            f_aabb = ctx.part_world_aabb(frame)
            ctx.check(
                "sash retained within frame X span at full travel",
                s_open[0][0] > f_aabb[0][0] - 1e-4 and s_open[1][0] < f_aabb[1][0] + 1e-4,
                details=f"sash x=[{s_open[0][0]:.3f},{s_open[1][0]:.3f}]",
            )

    # --- Screen slides independently ---
    with ctx.pose({slide: 0.0, screen_slide: SCREEN_TRAVEL}):
        scr_moved = ctx.part_world_aabb(screen)
        moved_cx = (scr_moved[0][0] + scr_moved[1][0]) / 2.0
        # Screen moved in -X (same axis convention)
        ctx.check(
            "screen slides independently in -X",
            moved_cx < SCREEN_CX - 0.10,
            details=f"screen_rest_x={SCREEN_CX:.3f}, moved_x={moved_cx:.3f}",
        )
        # Screen stays on exterior side
        moved_cy = (scr_moved[0][1] + scr_moved[1][1]) / 2.0
        ctx.check(
            "screen stays on exterior track while sliding",
            abs(moved_cy - SCREEN_Y) < 0.02,
            details=f"screen_y={moved_cy:.3f}, expected={SCREEN_Y:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
