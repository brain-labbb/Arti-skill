from __future__ import annotations

# Variant 29: Two-panel horizontal sliding window with insect screen panel,
# white vinyl frame with deep track grooves and a recessed pull cup.
#
# Structure:
#   - frame (static root): deep box-section vinyl profile with three parallel
#     track grooves cut into the head and sill inner faces (screen / fixed /
#     sliding tracks). One large opening, no fixed mullion bar.
#   - fixed_sash (left, FIXED): vinyl sash ring + clear glass, middle track.
#   - sliding_sash (right, PRISMATIC): vinyl sash ring + clear glass, front
#     track; carries the cam-latch and a recessed pull cup on the meeting stile.
#   - insect_screen (FIXED): thin aluminium screen frame + mesh panel, seated
#     in the outermost (exterior) track groove.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     - width  -> X
#     - height -> Z   (sill near z=0)
#     - frame depth / glazing thickness / slide-normal -> Y
#   Glass plane is the X-Z plane. q=0 reads SHUT. Driving the prismatic joint
#   slides the right sash sideways toward the fixed left sash (-X) to open.

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
FRAME_DEPTH = 0.155       # deep box section along Y (3-track patio-slider profile)

MEETING_OVERLAP = 0.040   # the two sash stiles overlap by this much at center

SASH_FACE = 0.075         # sash perimeter rail/stile face width (chunky)
SASH_DEPTH = 0.050        # sash depth along Y
GLASS_T = 0.008           # glazing thickness along Y

# Y layout: frame box centered on y=0.
# Screen in outermost (exterior, -Y) track, fixed sash in middle, sliding sash
# in front (interior, +Y) track. Gaps keep all three clear of each other.
SCREEN_Y = -0.062         # exterior screen track center (Y)
FIXED_SASH_Y = -0.025     # rear glazing plane center (Y)
SLIDE_SASH_Y = 0.040      # sliding sash proud toward +Y (front track)

# Screen panel
SCREEN_DEPTH = 0.014      # screen frame depth along Y (thin aluminium)
SCREEN_FRAME_W = 0.030    # screen frame member face width
SCREEN_FRAME_MARGIN = 0.0    # screen frame seats flush against frame track grooves
SCREEN_MESH_T = 0.002     # screen mesh sheet thickness

# Track channel geometry: raised lips on the inner sill/head face form U-channels
# that grip the screen and sash frames. Each track has two parallel lips.
TRACK_LIP_W = 0.006        # each lip thickness (Y)
TRACK_LIP_H = 0.010        # lip height (protrudes into opening from sill/head)
TRACK_LIP_GAP = 0.000      # tight fit: panel edges contact channel walls

# Recessed pull cup on sliding sash meeting stile
PULL_CUP_W = 0.050        # cup width (X)
PULL_CUP_H = 0.025        # cup height (Z)
PULL_CUP_D = 0.008        # cup depth (Y, into the stile face)

# Latch (cam lock) hardware
LATCH_PLATE_W = 0.028
LATCH_PLATE_H = 0.075
LATCH_PLATE_T = 0.010
LATCH_LEVER_LEN = 0.045
LATCH_LEVER_R = 0.006

REBATE = 0.005            # glass tucks under the sash lip by this much

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

# Screen frame outer dims fit inside the frame opening with margin.
SCREEN_OUT_W = INNER_W - 2 * SCREEN_FRAME_MARGIN
SCREEN_OUT_H = INNER_H - 2 * SCREEN_FRAME_MARGIN
SCREEN_OPENING_W = SCREEN_OUT_W - 2 * SCREEN_FRAME_W
SCREEN_OPENING_H = SCREEN_OUT_H - 2 * SCREEN_FRAME_W

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)
GLASS_RGBA = (0.52, 0.60, 0.66, 0.30)
METAL_RGBA = (0.74, 0.76, 0.79, 1.0)
SCREEN_FRAME_RGBA = (0.78, 0.80, 0.82, 1.0)   # satin aluminium
SCREEN_MESH_RGBA = (0.18, 0.20, 0.22, 0.38)    # dark fibreglass mesh
PULL_CUP_RGBA = (0.14, 0.14, 0.16, 1.0)        # dark recessed cup


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
    """Static outer frame: thick slab cut by one large sash opening, with
    raised track lips on the inner sill/head faces forming U-channels for
    the screen, fixed sash, and sliding sash panels."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    cut_depth = FRAME_DEPTH + 0.02
    opening = _slab(INNER_X0, INNER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    frame = outer.cut(opening)

    # Track lips: raised bars on the inner sill/head faces that form
    # U-channels gripping the screen and sash frame edges.
    # For each track, two parallel lips flank the panel in Y.
    for track_y, panel_depth in (
        (SCREEN_Y, SCREEN_DEPTH),
        (FIXED_SASH_Y, SASH_DEPTH),
        (SLIDE_SASH_Y, SASH_DEPTH),
    ):
        half_d = panel_depth / 2.0 + TRACK_LIP_GAP
        # Outer lip (exterior side)
        lip_outer_y = track_y - half_d - TRACK_LIP_W / 2.0
        # Inner lip (interior side)
        lip_inner_y = track_y + half_d + TRACK_LIP_W / 2.0

        for lip_y in (lip_outer_y, lip_inner_y):
            # Sill lip: protrudes upward from z=INNER_Z0
            sill_lip = _slab(
                INNER_X0, INNER_X1,
                INNER_Z0, INNER_Z0 + TRACK_LIP_H,
                lip_y, TRACK_LIP_W,
            )
            frame = frame.union(sill_lip)
            # Head lip: protrudes downward from z=INNER_Z1
            head_lip = _slab(
                INNER_X0, INNER_X1,
                INNER_Z1 - TRACK_LIP_H, INNER_Z1,
                lip_y, TRACK_LIP_W,
            )
            frame = frame.union(head_lip)

    return frame


def _build_sash_shape() -> cq.Workplane:
    """One sash ring in its own local frame, centered on origin."""
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


def _build_screen_frame_shape() -> cq.Workplane:
    """Screen frame ring in its own local frame, centered on origin."""
    outer = _slab(
        -SCREEN_OUT_W / 2.0, SCREEN_OUT_W / 2.0,
        -SCREEN_OUT_H / 2.0, SCREEN_OUT_H / 2.0,
        0.0, SCREEN_DEPTH,
    )
    opening = _slab(
        -SCREEN_OPENING_W / 2.0, SCREEN_OPENING_W / 2.0,
        -SCREEN_OPENING_H / 2.0, SCREEN_OPENING_H / 2.0,
        0.0, SCREEN_DEPTH + 0.02,
    )
    return outer.cut(opening)


def _build_screen_mesh_shape() -> cq.Workplane:
    """Thin mesh panel filling the screen opening."""
    return _slab(
        -SCREEN_OPENING_W / 2.0, SCREEN_OPENING_W / 2.0,
        -SCREEN_OPENING_H / 2.0, SCREEN_OPENING_H / 2.0,
        0.0, SCREEN_MESH_T,
    )


def _build_pull_cup_shape() -> cq.Workplane:
    """Recessed pull cup: a shallow hollow tray in sash-aligned frame
    (X=width, Y=depth, Z=height). Cavity opens on the +Y face."""
    w, h, d = PULL_CUP_W, PULL_CUP_H, PULL_CUP_D
    t = 0.003  # wall thickness
    outer = cq.Workplane("XY").box(w, d, h)
    cavity = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, t / 2.0, 0.0))
        .box(w - 2 * t, d - t, h - 2 * t)
    )
    return outer.cut(cavity)


# ---------------------------------------------------------------------------
# Part builders
# ---------------------------------------------------------------------------

def _add_sash(model: ArticulatedObject, name: str) -> None:
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


def _add_pull_cup(model: ArticulatedObject, sash_name: str) -> None:
    """Recessed pull cup on the meeting stile, below the latch."""
    sash = model.get_part(sash_name)
    stile_x = -SASH_OPENING_W / 2.0 - SASH_FACE / 2.0
    face_y = SASH_DEPTH / 2.0
    # Cup center: front face flush with sash face, body recessed inward
    cup_y = face_y - PULL_CUP_D / 2.0
    cup_z = -0.18  # well below mid-height (comfortable finger-pull position)

    sash.visual(
        mesh_from_cadquery(_build_pull_cup_shape(), f"{sash_name}_pull_cup"),
        origin=Origin(xyz=(stile_x, cup_y, cup_z)),
        material="pull_cup",
        name=f"{sash_name}_pull_cup",
    )


def _add_screen(model: ArticulatedObject) -> None:
    screen = model.part("insect_screen")
    screen.visual(
        mesh_from_cadquery(_build_screen_frame_shape(), "screen_frame"),
        material="screen_frame",
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
    model = ArticulatedObject(name="sliding_window_with_screen")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("metal", rgba=METAL_RGBA)
    model.material("screen_frame", rgba=SCREEN_FRAME_RGBA)
    model.material("screen_mesh", rgba=SCREEN_MESH_RGBA)
    model.material("pull_cup", rgba=PULL_CUP_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="vinyl",
        name="frame_shell",
    )

    # --- Sashes ---
    _add_sash(model, "fixed_sash")
    _add_sash(model, "sliding_sash")
    _add_latch(model, "sliding_sash")
    _add_pull_cup(model, "sliding_sash")

    # --- Insect screen ---
    _add_screen(model)

    # FIXED left sash seated in the middle track.
    model.articulation(
        "frame_to_fixed_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="fixed_sash",
        origin=Origin(xyz=(FIXED_OPEN_CX, FIXED_SASH_Y, MID_CZ)),
    )

    # SLIDING right sash: PRISMATIC along X. Positive q slides left (-X).
    slide_travel = SASH_OPENING_W * 0.90
    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="sliding_sash",
        origin=Origin(xyz=(SLIDE_OPEN_CX, SLIDE_SASH_Y, MID_CZ)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5, lower=0.0, upper=slide_travel),
    )

    # FIXED screen panel in the outermost (exterior) track.
    model.articulation(
        "frame_to_screen",
        ArticulationType.FIXED,
        parent="frame",
        child="insect_screen",
        origin=Origin(xyz=(0.0, SCREEN_Y, MID_CZ)),
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

    # --- Intentional overlaps ---
    # Glass rebated under sash lip on each sash.
    for nm in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_vinyl",
            reason="Clear pane is rebated under the sash lip so it reads captured.",
        )
    # Each sash ring laps the frame opening edge (seated in track groove).
    for nm in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_vinyl",
            reason=f"{nm} ring is rebated into the frame track (seated capture).",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_glass",
            reason=f"{nm} glass is rebated under the frame opening lip.",
        )
    # Latch keeper plate seated on sash stile.
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="sliding_sash_latch_plate",
        elem_b="sliding_sash_vinyl",
        reason="Latch keeper plate is seated onto the sliding-sash meeting-stile face.",
    )
    # Pull cup recessed into sash stile.
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="sliding_sash_pull_cup",
        elem_b="sliding_sash_vinyl",
        reason="Pull cup is recessed into the sliding-sash meeting-stile face.",
    )
    # Screen frame seated in its own track groove within the frame opening.
    ctx.allow_overlap(
        "frame", "insect_screen",
        elem_a="frame_shell",
        elem_b="screen_frame",
        reason="Screen frame is seated in the exterior track groove of the frame.",
    )
    ctx.allow_overlap(
        "insect_screen", "insect_screen",
        elem_a="screen_mesh",
        elem_b="screen_frame",
        reason="Screen mesh is captured in the screen frame opening.",
    )

    # --- Closed pose (q=0): window reads SHUT ---
    with ctx.pose({slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        f_aabb = ctx.part_world_aabb(fixed_sash)
        s_aabb = ctx.part_world_aabb(sliding_sash)
        screen_aabb = ctx.part_world_aabb(screen)

        # Frame spans the full width and is wider than a single sash.
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        sash_w = s_aabb[1][0] - s_aabb[0][0]
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
        # Two sashes side by side: fixed on the left, sliding on the right.
        fx = (f_aabb[0][0] + f_aabb[1][0]) / 2.0
        sx = (s_aabb[0][0] + s_aabb[1][0]) / 2.0
        ctx.check(
            "fixed sash left of sliding sash",
            fx < sx,
            details=f"fixed_x={fx:.3f}, sliding_x={sx:.3f}",
        )
        # Both sashes seated within the frame height.
        for nm, ab in (("fixed", f_aabb), ("sliding", s_aabb)):
            ctx.check(
                f"{nm} sash seated within frame height",
                ab[0][2] > frame_aabb[0][2] - 1e-4 and ab[1][2] < frame_aabb[1][2] + 1e-4,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}]",
            )
        # Sliding sash sits proud (in +Y) of the fixed sash.
        fy = (f_aabb[0][1] + f_aabb[1][1]) / 2.0
        sy = (s_aabb[0][1] + s_aabb[1][1]) / 2.0
        ctx.check(
            "sliding sash proud of fixed sash",
            sy > fy + 0.02,
            details=f"sliding_y={sy:.3f}, fixed_y={fy:.3f}",
        )
        # Both sashes seated in the frame opening.
        ctx.expect_overlap(
            fixed_sash, frame, axes="xz", min_overlap=0.03,
            name="fixed sash seated in frame opening",
        )
        ctx.expect_overlap(
            sliding_sash, frame, axes="xz", min_overlap=0.03,
            name="sliding sash seated in frame opening",
        )

        # --- Screen panel checks ---
        screen_cy = (screen_aabb[0][1] + screen_aabb[1][1]) / 2.0
        # Screen is in separate exterior track (most negative Y).
        ctx.check(
            "screen in separate track on exterior side of fixed sash",
            screen_cy < fy - 0.005,
            details=f"screen_y={screen_cy:.3f}, fixed_y={fy:.3f}",
        )
        ctx.check(
            "screen in separate track on exterior side of sliding sash",
            screen_cy < sy - 0.020,
            details=f"screen_y={screen_cy:.3f}, sliding_y={sy:.3f}",
        )
        # Screen within the frame opening (X-Z projection).
        ctx.expect_within(
            screen, frame, axes="xz", margin=0.01,
            name="screen within frame opening in X-Z",
        )
        # Screen spans most of the opening.
        screen_w = screen_aabb[1][0] - screen_aabb[0][0]
        ctx.check(
            "screen covers most of the opening width",
            screen_w > INNER_W * 0.80,
            details=f"screen_w={screen_w:.3f}, inner_w={INNER_W:.3f}",
        )

        # --- Pull cup checks ---
        pull_aabb = ctx.part_element_world_aabb(sliding_sash, elem="sliding_sash_pull_cup")
        pull_cx = (pull_aabb[0][0] + pull_aabb[1][0]) / 2.0
        pull_cz = (pull_aabb[0][2] + pull_aabb[1][2]) / 2.0
        # Pull cup on meeting stile (left of sash center).
        ctx.check(
            "pull cup on sliding sash meeting stile",
            pull_cx < sx,
            details=f"pull_x={pull_cx:.3f}, sliding_cx={sx:.3f}",
        )
        # Pull cup below mid-height.
        ctx.check(
            "pull cup below mid-height",
            pull_cz < MID_CZ - 0.05,
            details=f"pull_z={pull_cz:.3f}, mid_z={MID_CZ:.3f}",
        )
        # Latch above pull cup.
        latch_aabb = ctx.part_element_world_aabb(sliding_sash, elem="sliding_sash_latch_plate")
        latch_cz = (latch_aabb[0][2] + latch_aabb[1][2]) / 2.0
        ctx.check(
            "pull cup below latch",
            pull_cz < latch_cz - 0.05,
            details=f"pull_z={pull_cz:.3f}, latch_z={latch_cz:.3f}",
        )

        rest_sx = sx
        rest_sz = (s_aabb[0][2] + s_aabb[1][2]) / 2.0

    # --- Driven/open pose: sliding sash slides toward fixed sash (-X) ---
    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel}):
        s_open = ctx.part_world_aabb(sliding_sash)
        open_sx = (s_open[0][0] + s_open[1][0]) / 2.0
        # Positive q moves the sash in -X to open.
        ctx.check(
            "sliding sash opens toward fixed sash (-X)",
            abs((rest_sx - open_sx) - travel) < 0.02 and open_sx < rest_sx - 0.30,
            details=f"rest_x={rest_sx:.3f}, open_x={open_sx:.3f}, travel={travel:.3f}",
        )
        # Pure horizontal slide (no Z change).
        open_sz = (s_open[0][2] + s_open[1][2]) / 2.0
        ctx.check(
            "slide is purely horizontal",
            abs(open_sz - rest_sz) < 0.02,
            details=f"open_z={open_sz:.3f}, rest_z={rest_sz:.3f}",
        )
        # Retained insertion: sash stays within the frame X span at full travel.
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained within frame X span at full travel",
            s_open[0][0] > f_aabb[0][0] - 1e-4 and s_open[1][0] < f_aabb[1][0] + 1e-4,
            details=f"sash x=[{s_open[0][0]:.3f},{s_open[1][0]:.3f}] frame x=[{f_aabb[0][0]:.3f},{f_aabb[1][0]:.3f}]",
        )
        ctx.expect_overlap(
            sliding_sash, frame, axes="z", min_overlap=0.10,
            name="sash retains vertical engagement with head/sill track",
        )

    # --- Prismatic joint metadata check ---
    ctx.check(
        "slide joint is prismatic with positive travel",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide.motion_limits is not None
        and slide.motion_limits.upper is not None
        and slide.motion_limits.upper > 0.0,
        details=f"type={slide.articulation_type}, upper={slide.motion_limits.upper if slide.motion_limits else None}",
    )

    return ctx.report()


object_model = build_object_model()
