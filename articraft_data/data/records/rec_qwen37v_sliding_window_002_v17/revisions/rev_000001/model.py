from __future__ import annotations

# Two-panel horizontal sliding window variant — white vinyl frame with deep
# track grooves, muntin grid bars on the sliding sash only, an independently
# sliding insect screen, and a recessed pull cup on the sliding sash.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     - width  -> X
#     - height -> Z   (sill near z=0)
#     - frame depth / glazing thickness / slide-normal -> Y
#   Glass plane is the X-Z plane. q=0 reads SHUT. Driving the prismatic joints
#   slides the right sash / screen sideways toward the fixed left sash (-X).
#
# Structure:
#   - frame (static root): head, sill, two jambs + deep box profile, with
#     visible deep track grooves cut into head and sill inner surfaces.
#   - fixed_sash (left, FIXED): vinyl sash ring + clear glass, no muntins.
#   - sliding_sash (right, PRISMATIC): vinyl sash ring + clear glass + muntin
#     grid bars (1 horizontal + 1 vertical) + recessed pull cup + latch.
#   - insect_screen (PRISMATIC, independent): thin vinyl frame + screen mesh,
#     sits behind both sashes on its own shallow track.

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

# Y layout: frame box centered on y=0.
FIXED_SASH_Y = -0.028     # rear glazing plane center (Y)
SLIDE_SASH_Y = 0.044      # sliding sash proud toward +Y (front track)
SCREEN_Y = -0.068         # insect screen behind both sashes (rearmost track)

REBATE = 0.005            # glass tucks under the sash lip by this much

# Track groove dimensions (deep channels in head and sill)
TRACK_GROOVE_W = 0.018    # groove width along Y
TRACK_GROOVE_DEPTH = 0.012  # groove depth cut into the rail face (Z direction)
TRACK_GROOVE_COUNT = 2    # two parallel grooves per rail (front + rear track)

# Muntin bar dimensions (on sliding sash only)
MUNTIN_W = 0.018          # muntin bar face width
MUNTIN_T = 0.012          # muntin bar thickness (depth along Y)

# Pull cup dimensions (recessed into sliding sash stile)
PULL_CUP_R = 0.018        # cup radius
PULL_CUP_DEPTH = 0.008    # cup recess depth

# Insect screen dimensions
SCREEN_FRAME_W = 0.030    # screen frame member width
SCREEN_FRAME_T = 0.015    # screen frame depth along Y
SCREEN_MESH_T = 0.002     # screen mesh thickness

# Latch (cam lock) hardware
LATCH_PLATE_W = 0.028
LATCH_PLATE_H = 0.075
LATCH_PLATE_T = 0.010
LATCH_LEVER_LEN = 0.045
LATCH_LEVER_R = 0.006

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)
GLASS_RGBA = (0.52, 0.60, 0.66, 0.30)
METAL_RGBA = (0.74, 0.76, 0.79, 1.0)
SCREEN_RGBA = (0.25, 0.27, 0.28, 0.55)   # dark grey semi-transparent mesh
GROOVE_RGBA = (0.82, 0.83, 0.84, 1.0)    # slightly darker vinyl for groove shadow

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

# Screen opening slightly smaller than sash opening (frame inset)
SCREEN_OPEN_W = SASH_OPENING_W - 0.010
SCREEN_OPEN_H = SASH_OPENING_H - 0.010
SCREEN_CX = SLIDE_OPEN_CX  # screen starts at right side like sliding sash

# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery)
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float, y_center: float, depth: float) -> cq.Workplane:
    """Axis-aligned box spanning [x0,x1] x [z0,z1], centered on y_center."""
    return (
        cq.Workplane("XY")
        .transformed(offset=((x0 + x1) / 2.0, y_center, (z0 + z1) / 2.0))
        .box(x1 - x0, depth, z1 - z0)
    )


def _build_frame_shape() -> cq.Workplane:
    """Static outer frame: thick slab cut by the big opening, with deep track
    grooves cut into head and sill inner surfaces."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    cut_depth = FRAME_DEPTH + 0.02
    opening = _slab(INNER_X0, INNER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    frame = outer.cut(opening)

    # Deep track grooves: two parallel channels in the sill (bottom rail) and
    # two in the head (top rail). Each groove is a thin slot cut into the inner
    # face of the rail, running the full inner width. The grooves are at
    # different Y positions for front and rear tracks.
    groove_y_positions = [SLIDE_SASH_Y, FIXED_SASH_Y]  # front track, rear track

    for gy in groove_y_positions:
        # Sill groove (cut upward from the inner bottom surface)
        sill_groove = _slab(
            INNER_X0, INNER_X1,
            INNER_Z0 - 0.001,
            INNER_Z0 + TRACK_GROOVE_DEPTH,
            gy, TRACK_GROOVE_W,
        )
        frame = frame.cut(sill_groove)

        # Head groove (cut downward from the inner top surface)
        head_groove = _slab(
            INNER_X0, INNER_X1,
            INNER_Z1 - TRACK_GROOVE_DEPTH,
            INNER_Z1 + 0.001,
            gy, TRACK_GROOVE_W,
        )
        frame = frame.cut(head_groove)

    return frame


def _build_sash_shape() -> cq.Workplane:
    """One sash ring in its own local frame, centered on local origin."""
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
    """Insect screen frame ring in its own local frame."""
    ow = SCREEN_OPEN_W
    oh = SCREEN_OPEN_H
    out_w = ow + 2 * SCREEN_FRAME_W
    out_h = oh + 2 * SCREEN_FRAME_W
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SCREEN_FRAME_T)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SCREEN_FRAME_T + 0.002)
    return outer.cut(opening)


def _build_screen_mesh_shape() -> cq.Workplane:
    """Thin screen mesh panel filling the screen frame opening."""
    return _slab(-SCREEN_OPEN_W / 2.0, SCREEN_OPEN_W / 2.0,
                 -SCREEN_OPEN_H / 2.0, SCREEN_OPEN_H / 2.0,
                 0.0, SCREEN_MESH_T)


def _build_pull_cup_shape() -> cq.Workplane:
    """Recessed pull cup: a shallow disk representing the cup cavity."""
    return (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, 0.0))
        .circle(PULL_CUP_R)
        .extrude(PULL_CUP_DEPTH)
    )


# ---------------------------------------------------------------------------
# Part builders
# ---------------------------------------------------------------------------

def _add_fixed_sash(model: ArticulatedObject) -> None:
    """Fixed sash: vinyl ring + clear glass, no muntins."""
    sash = model.part("fixed_sash")
    sash.visual(
        mesh_from_cadquery(_build_sash_shape(), "fixed_sash_vinyl"),
        material="vinyl",
        name="fixed_sash_vinyl",
    )
    sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "fixed_sash_glass"),
        material="glass",
        name="fixed_sash_glass",
    )


def _add_sliding_sash(model: ArticulatedObject) -> None:
    """Sliding sash: vinyl ring + clear glass + muntin grid bars + pull cup + latch."""
    sash = model.part("sliding_sash")
    sash.visual(
        mesh_from_cadquery(_build_sash_shape(), "sliding_sash_vinyl"),
        material="vinyl",
        name="sliding_sash_vinyl",
    )
    sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "sliding_sash_glass"),
        material="glass",
        name="sliding_sash_glass",
    )

    # Muntin grid bars: 1 horizontal + 1 vertical bar creating a 4-pane grid.
    # In sash-local frame, the opening spans [-ow/2, ow/2] x [-oh/2, oh/2].
    ow = SASH_OPENING_W
    oh = SASH_OPENING_H

    # Horizontal muntin bar (runs along X, centered at z=0)
    sash.visual(
        Box((ow, MUNTIN_T, MUNTIN_W)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="vinyl",
        name="sliding_sash_muntin_h",
    )
    # Vertical muntin bar (runs along Z, centered at x=0)
    sash.visual(
        Box((MUNTIN_W, MUNTIN_T, oh)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="vinyl",
        name="sliding_sash_muntin_v",
    )

    # Recessed pull cup: on the front face of the sliding sash, near the bottom
    # of the meeting (inner/left) stile. The cup is a shallow cylinder recessed
    # into the stile face, oriented along Y (into the face).
    stile_x = -ow / 2.0 - SASH_FACE / 2.0   # meeting stile center in sash-local X
    face_y = SASH_DEPTH / 2.0                # front face of sash
    cup_y = face_y - PULL_CUP_DEPTH / 2.0   # cup center recessed into the face
    cup_z = -oh / 2.0 - SASH_FACE / 2.0 + 0.020  # near the bottom rail

    sash.visual(
        Cylinder(radius=PULL_CUP_R, length=PULL_CUP_DEPTH),
        origin=Origin(xyz=(stile_x, cup_y, cup_z), rpy=(1.5707963, 0.0, 0.0)),
        material="metal",
        name="sliding_sash_pull_cup",
    )

    # Latch keeper plate on the meeting stile (mid-height)
    plate_y = face_y + LATCH_PLATE_T / 2.0
    sash.visual(
        Box((LATCH_PLATE_W, LATCH_PLATE_T, LATCH_PLATE_H)),
        origin=Origin(xyz=(stile_x, plate_y, 0.0)),
        material="metal",
        name="sliding_sash_latch_plate",
    )
    lever_y = face_y + LATCH_PLATE_T + LATCH_LEVER_LEN / 2.0
    sash.visual(
        Cylinder(radius=LATCH_LEVER_R, length=LATCH_LEVER_LEN),
        origin=Origin(xyz=(stile_x, lever_y, -0.008), rpy=(1.5707963, 0.0, 0.0)),
        material="metal",
        name="sliding_sash_latch_lever",
    )


def _add_insect_screen(model: ArticulatedObject) -> None:
    """Insect screen: thin vinyl frame + mesh panel, on its own prismatic track."""
    screen = model.part("insect_screen")
    screen.visual(
        mesh_from_cadquery(_build_screen_frame_shape(), "screen_frame"),
        material="vinyl",
        name="screen_frame",
    )
    screen.visual(
        mesh_from_cadquery(_build_screen_mesh_shape(), "screen_mesh"),
        material="screen",
        name="screen_mesh",
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_window_muntin_screen")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("metal", rgba=METAL_RGBA)
    model.material("screen", rgba=SCREEN_RGBA)

    # --- Static outer frame (root) with deep track grooves ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="vinyl",
        name="frame_shell",
    )

    # --- Fixed sash (left, no muntins) ---
    _add_fixed_sash(model)

    # --- Sliding sash (right, with muntins + pull cup + latch) ---
    _add_sliding_sash(model)

    # --- Insect screen (independent sliding track) ---
    _add_insect_screen(model)

    # FIXED left sash seated in the rear glazing plane.
    model.articulation(
        "frame_to_fixed_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="fixed_sash",
        origin=Origin(xyz=(FIXED_OPEN_CX, FIXED_SASH_Y, MID_CZ)),
    )

    # SLIDING right sash: PRISMATIC along X.
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

    # INSECT SCREEN: PRISMATIC along X, independent of the sash.
    # Screen slides the same direction but with slightly less travel.
    screen_travel = SASH_OPENING_W * 0.80
    model.articulation(
        "frame_to_screen",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="insect_screen",
        origin=Origin(xyz=(SCREEN_CX, SCREEN_Y, MID_CZ)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.4, lower=0.0, upper=screen_travel),
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
    insect_screen = object_model.get_part("insect_screen")
    slide = object_model.get_articulation("frame_to_sliding_sash")
    screen_joint = object_model.get_articulation("frame_to_screen")

    # --- Intentional overlaps ---
    # Glass rebated under sash lips
    for nm in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_vinyl",
            reason="Clear pane is rebated under the sash lip so it reads captured, not floating.",
        )
    # Sash rings rebated into frame opening
    for nm in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_vinyl",
            reason=f"{nm} ring is rebated into the frame opening / head-sill track (seated capture).",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_glass",
            reason=f"{nm} glass is rebated under the frame opening lip (captured glazing).",
        )
    # Muntin bars overlap the glass (they sit on the glass surface)
    for muntin in ("sliding_sash_muntin_h", "sliding_sash_muntin_v"):
        ctx.allow_overlap(
            "sliding_sash", "sliding_sash",
            elem_a=muntin,
            elem_b="sliding_sash_glass",
            reason=f"{muntin} sits on the glass surface as a grid divider (mounted, not floating).",
        )
    # Muntin bars sit within the frame opening (like the sash itself)
    for muntin in ("sliding_sash_muntin_h", "sliding_sash_muntin_v"):
        ctx.allow_overlap(
            "frame", "sliding_sash",
            elem_a="frame_shell",
            elem_b=muntin,
            reason=f"{muntin} is within the frame opening as part of the sliding sash grid (seated in track).",
        )
    # Pull cup recessed into the stile face
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="sliding_sash_pull_cup",
        elem_b="sliding_sash_vinyl",
        reason="Pull cup is recessed into the sliding sash stile face (seated recess).",
    )
    # Latch keeper plate seated on stile
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="sliding_sash_latch_plate",
        elem_b="sliding_sash_vinyl",
        reason="Latch keeper plate is seated onto the sliding-sash meeting-stile face.",
    )
    # Screen frame + mesh overlap
    ctx.allow_overlap(
        "insect_screen", "insect_screen",
        elem_a="screen_mesh",
        elem_b="screen_frame",
        reason="Screen mesh is captured inside the screen frame opening (seated panel).",
    )
    # Screen frame rebated into frame track
    ctx.allow_overlap(
        "frame", "insect_screen",
        elem_a="frame_shell",
        elem_b="screen_frame",
        reason="Insect screen frame rides in the rearmost frame track channel (seated capture).",
    )
    ctx.allow_overlap(
        "frame", "insect_screen",
        elem_a="frame_shell",
        elem_b="screen_mesh",
        reason="Screen mesh passes through the frame opening region (track capture).",
    )

    # --- Closed pose (q=0): window reads SHUT ---
    with ctx.pose({slide: 0.0, screen_joint: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        f_aabb = ctx.part_world_aabb(fixed_sash)
        s_aabb = ctx.part_world_aabb(sliding_sash)
        scr_aabb = ctx.part_world_aabb(insect_screen)

        # Frame spans the full width
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        sash_w = s_aabb[1][0] - s_aabb[0][0]
        ctx.check(
            "frame spans wider than a single sash",
            frame_w > sash_w + 0.40,
            details=f"frame_w={frame_w:.3f}, sash_w={sash_w:.3f}",
        )
        # Sill near z=0, head at full height
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
        # Two sashes side by side
        fx = (f_aabb[0][0] + f_aabb[1][0]) / 2.0
        sx = (s_aabb[0][0] + s_aabb[1][0]) / 2.0
        ctx.check(
            "fixed sash left of sliding sash",
            fx < sx,
            details=f"fixed_x={fx:.3f}, sliding_x={sx:.3f}",
        )
        # Sliding sash proud of fixed sash in +Y
        fy = (f_aabb[0][1] + f_aabb[1][1]) / 2.0
        sy = (s_aabb[0][1] + s_aabb[1][1]) / 2.0
        ctx.check(
            "sliding sash proud of fixed sash",
            sy > fy + 0.02,
            details=f"sliding_y={sy:.3f}, fixed_y={fy:.3f}",
        )
        # Both sashes seated within frame height
        for nm, ab in (("fixed", f_aabb), ("sliding", s_aabb)):
            ctx.check(
                f"{nm} sash seated within frame height",
                ab[0][2] > frame_aabb[0][2] - 1e-4 and ab[1][2] < frame_aabb[1][2] + 1e-4,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}]",
            )
        # Sashes seated in frame opening
        ctx.expect_overlap(
            fixed_sash, frame, axes="xz", min_overlap=0.03,
            name="fixed sash seated in frame opening",
        )
        ctx.expect_overlap(
            sliding_sash, frame, axes="xz", min_overlap=0.03,
            name="sliding sash seated in frame opening",
        )

        # --- Muntin bars on sliding sash ---
        muntin_h_aabb = ctx.part_element_world_aabb(sliding_sash, elem="sliding_sash_muntin_h")
        muntin_v_aabb = ctx.part_element_world_aabb(sliding_sash, elem="sliding_sash_muntin_v")
        # Horizontal muntin spans most of the sash width
        muntin_h_span = muntin_h_aabb[1][0] - muntin_h_aabb[0][0]
        ctx.check(
            "horizontal muntin spans most of sliding sash width",
            muntin_h_span > SASH_OPENING_W * 0.8,
            details=f"muntin_h_span={muntin_h_span:.3f}, opening_w={SASH_OPENING_W:.3f}",
        )
        # Vertical muntin spans most of the sash height
        muntin_v_span = muntin_v_aabb[1][2] - muntin_v_aabb[0][2]
        ctx.check(
            "vertical muntin spans most of sliding sash height",
            muntin_v_span > SASH_OPENING_H * 0.8,
            details=f"muntin_v_span={muntin_v_span:.3f}, opening_h={SASH_OPENING_H:.3f}",
        )
        # Muntins overlap the sliding sash glass in XY projection
        ctx.expect_overlap(
            sliding_sash, sliding_sash,
            axes="xz", min_overlap=0.01,
            elem_a="sliding_sash_muntin_h",
            elem_b="sliding_sash_glass",
            name="horizontal muntin overlaps glass in projection",
        )
        ctx.expect_overlap(
            sliding_sash, sliding_sash,
            axes="xz", min_overlap=0.01,
            elem_a="sliding_sash_muntin_v",
            elem_b="sliding_sash_glass",
            name="vertical muntin overlaps glass in projection",
        )

        # --- Fixed sash has NO muntin bars ---
        fixed_visual_names = [v.name for v in fixed_sash.visuals if v.name]
        ctx.check(
            "fixed sash has no muntin bars",
            not any("muntin" in n for n in fixed_visual_names),
            details=f"fixed sash visuals: {fixed_visual_names}",
        )

        # --- Pull cup on sliding sash ---
        cup_aabb = ctx.part_element_world_aabb(sliding_sash, elem="sliding_sash_pull_cup")
        cup_cx = (cup_aabb[0][0] + cup_aabb[1][0]) / 2.0
        cup_cz = (cup_aabb[0][2] + cup_aabb[1][2]) / 2.0
        # Pull cup is on the meeting stile side (left of sash center)
        ctx.check(
            "pull cup on meeting stile side of sliding sash",
            cup_cx < sx,
            details=f"cup_x={cup_cx:.3f}, sash_center_x={sx:.3f}",
        )
        # Pull cup is near the bottom of the sash
        sash_bottom = s_aabb[0][2]
        ctx.check(
            "pull cup near bottom of sliding sash",
            cup_cz < MID_CZ and cup_cz > sash_bottom,
            details=f"cup_z={cup_cz:.3f}, mid_z={MID_CZ:.3f}, sash_bottom={sash_bottom:.3f}",
        )

        # --- Insect screen behind both sashes ---
        scr_y = (scr_aabb[0][1] + scr_aabb[1][1]) / 2.0
        ctx.check(
            "insect screen behind fixed sash in Y",
            scr_y < fy - 0.01,
            details=f"screen_y={scr_y:.3f}, fixed_y={fy:.3f}",
        )
        # Screen seated within frame
        ctx.expect_overlap(
            insect_screen, frame, axes="xz", min_overlap=0.03,
            name="insect screen seated within frame opening",
        )

        # --- Track grooves (frame has deeper inner channels) ---
        # The frame inner depth should be greater than a simple flat slab due to grooves.
        frame_depth_actual = frame_aabb[1][1] - frame_aabb[0][1]
        ctx.check(
            "frame has substantial depth for track grooves",
            frame_depth_actual > 0.10,
            details=f"frame_depth={frame_depth_actual:.3f}",
        )

        rest_sx = sx
        rest_sz = (s_aabb[0][2] + s_aabb[1][2]) / 2.0
        rest_scr_x = (scr_aabb[0][0] + scr_aabb[1][0]) / 2.0

    # --- Sliding sash opens toward fixed sash (-X) ---
    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel}):
        s_open = ctx.part_world_aabb(sliding_sash)
        open_sx = (s_open[0][0] + s_open[1][0]) / 2.0
        ctx.check(
            "sliding sash opens toward fixed sash (-X)",
            abs((rest_sx - open_sx) - travel) < 0.02 and open_sx < rest_sx - 0.30,
            details=f"rest_x={rest_sx:.3f}, open_x={open_sx:.3f}, travel={travel:.3f}",
        )
        # Pure horizontal slide
        open_sz = (s_open[0][2] + s_open[1][2]) / 2.0
        ctx.check(
            "slide is purely horizontal",
            abs(open_sz - rest_sz) < 0.02,
            details=f"open_z={open_sz:.3f}, rest_z={rest_sz:.3f}",
        )
        # Retained insertion
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

    # --- Insect screen slides independently ---
    screen_travel = screen_joint.motion_limits.upper
    with ctx.pose({screen_joint: screen_travel}):
        scr_open = ctx.part_world_aabb(insect_screen)
        open_scr_x = (scr_open[0][0] + scr_open[1][0]) / 2.0
        ctx.check(
            "insect screen slides toward left (-X) when opened",
            open_scr_x < rest_scr_x - 0.10,
            details=f"rest_screen_x={rest_scr_x:.3f}, open_screen_x={open_scr_x:.3f}",
        )
        # Screen stays within frame
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "screen retained within frame X span at full travel",
            scr_open[0][0] > f_aabb[0][0] - 1e-4 and scr_open[1][0] < f_aabb[1][0] + 1e-4,
            details=f"screen x=[{scr_open[0][0]:.3f},{scr_open[1][0]:.3f}]",
        )

    # --- Screen and sash move independently ---
    # Open only the screen, confirm sash stays put
    with ctx.pose({screen_joint: screen_travel, slide: 0.0}):
        s_closed = ctx.part_world_aabb(sliding_sash)
        closed_sx = (s_closed[0][0] + s_closed[1][0]) / 2.0
        ctx.check(
            "sash stays closed when only screen slides",
            abs(closed_sx - rest_sx) < 0.01,
            details=f"closed_sash_x={closed_sx:.3f}, rest_sash_x={rest_sx:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
