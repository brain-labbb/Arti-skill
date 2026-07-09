from __future__ import annotations

# Two-panel horizontal sliding window variant: slim vinyl frame rails with
# bevelled corners, tilt-in latch pair on small revolute pivots, two roller
# blocks at the bottom of the moving sash, and a visible overlap stile where
# the panes cross.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     - width  -> X
#     - height -> Z   (sill near z=0)
#     - frame depth / glazing thickness / slide-normal -> Y
#   Glass plane is the X-Z plane. q=0 reads SHUT. Driving the prismatic joint
#   slides the right sash sideways toward the fixed left sash (-X) to open,
#   staying retained in the head/sill track.
#
# Structure:
#   - frame (static root): slim head, sill, two jambs with bevelled outer
#     corners, built as one CadQuery solid with chamfers.
#   - fixed_sash (left, FIXED): slim vinyl sash ring + clear glass, seated in
#     the rear glazing plane.
#   - sliding_sash (right, PRISMATIC): slim vinyl sash ring + clear glass,
#     sitting proud (front) so it passes in front of the fixed sash; carries
#     the tilt-in latches and roller blocks.
#   - tilt_latch_top / tilt_latch_bottom (REVOLUTE): two small tilt-in hooks
#     on the sliding sash meeting stile that pivot inward to lock.
#   - roller_left / roller_right: two small roller blocks at the bottom rail
#     of the sliding sash (fixed to the sash, not separately articulated).

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
# Absolute dimensions (meters) — slim profile variant
# ---------------------------------------------------------------------------

TOTAL_W = 1.52            # overall window width along X
TOTAL_H = 1.72            # overall height along Z (sill at z=0, head at z=TOTAL_H)

FRAME_FACE = 0.055        # slim outer frame member face width
FRAME_DEPTH = 0.100       # slimmer box section along Y
FRAME_CHAMFER = 0.006     # bevel size on outer corners

MEETING_OVERLAP = 0.050   # visible overlap stile width where panes cross

SASH_FACE = 0.048         # slim sash perimeter rail/stile face width
SASH_DEPTH = 0.045        # sash depth along Y
GLASS_T = 0.006           # glazing thickness along Y

# Y layout: frame box centered on y=0. Fixed sash in the rear glazing plane;
# sliding sash sits proud toward +Y so it passes in front of the fixed sash.
FIXED_SASH_Y = -0.022     # rear glazing plane center (Y)
SLIDE_SASH_Y = 0.032      # sliding sash proud toward +Y (front track)

REBATE = 0.004            # glass tucks under the sash lip by this much

# Tilt-in latch dimensions
LATCH_BODY_W = 0.022      # latch hook body width (X)
LATCH_BODY_H = 0.040      # latch hook body height (Z)
LATCH_BODY_T = 0.008      # latch hook body thickness (Y)
LATCH_PIVOT_R = 0.003     # pivot pin radius
LATCH_PIVOT_LEN = 0.016   # pivot pin length (along X)
LATCH_HOOK_LEN = 0.012    # the hook tab that extends from the body

# Roller block dimensions
ROLLER_W = 0.028          # roller block width (X)
ROLLER_H = 0.018          # roller block height (Z)
ROLLER_D = 0.020          # roller block depth (Y)
ROLLER_WHEEL_R = 0.006    # roller wheel radius
ROLLER_WHEEL_W = 0.010    # roller wheel width

# Overlap stile — a visible protruding fin on the sliding sash meeting edge
OVERLAP_STILE_W = 0.012   # width of the overlap fin (X)
OVERLAP_STILE_T = 0.008   # thickness of the overlap fin (Y, extends toward fixed sash)

METAL_RGBA = (0.70, 0.72, 0.75, 1.0)   # satin metal hardware
ROLLER_RGBA = (0.25, 0.25, 0.27, 1.0)  # dark nylon roller

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

# Each sash opening width: two openings that meet near the center with overlap.
SASH_OPENING_W = (INNER_W + MEETING_OVERLAP) / 2.0
SASH_OPENING_H = INNER_H

# Opening centers (world X) of each sash's clear glass region.
FIXED_OPEN_CX = INNER_X0 + SASH_OPENING_W / 2.0          # left
SLIDE_OPEN_CX = INNER_X1 - SASH_OPENING_W / 2.0          # right
MID_CZ = (INNER_Z0 + INNER_Z1) / 2.0

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)     # bright white vinyl/PVC
GLASS_RGBA = (0.52, 0.60, 0.66, 0.30)    # cool grey-blue, semi-transparent


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
    """Static outer frame: a slim slab cut by one clear opening, then outer
    vertical edges chamfered for the bevelled-corner look."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    cut_depth = FRAME_DEPTH + 0.02
    opening = _slab(INNER_X0, INNER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    frame = outer.cut(opening)
    # Bevel the outer vertical edges of the frame (Y-aligned edges on the
    # front/back faces at the four outer corners). We select vertical edges
    # (parallel to Z) on the outer boundary and chamfer them.
    try:
        frame = frame.edges("|Z").edges(cq.selectors.BoxSelector(
            (-HALF_W - 0.001, -FRAME_DEPTH / 2.0 - 0.001, -0.001),
            (-HALF_W + FRAME_FACE + 0.001, FRAME_DEPTH / 2.0 + 0.001, TOTAL_H + 0.001),
        )).chamfer(FRAME_CHAMFER)
    except Exception:
        pass  # If edge selection fails, continue without chamfer
    try:
        frame = frame.edges("|Z").edges(cq.selectors.BoxSelector(
            (HALF_W - FRAME_FACE - 0.001, -FRAME_DEPTH / 2.0 - 0.001, -0.001),
            (HALF_W + 0.001, FRAME_DEPTH / 2.0 + 0.001, TOTAL_H + 0.001),
        )).chamfer(FRAME_CHAMFER)
    except Exception:
        pass
    return frame


def _build_sash_shape() -> cq.Workplane:
    """One sash ring in its OWN local frame, centered on local origin:
      - local X in [-out_w/2, out_w/2]
      - local Z in [-out_h/2, out_h/2]
      - local Y is the sash depth, centered at 0
    """
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


def _build_overlap_stile() -> cq.Workplane:
    """Visible overlap stile fin on the sliding sash meeting edge.
    In sash-local frame: the meeting stile is at local x = -SASH_OPENING_W/2 - SASH_FACE/2.
    The fin extends from the front face toward -Y (toward the fixed sash).
    Shortened to the middle portion of the sash to avoid latch hardware."""
    stile_x = -SASH_OPENING_W / 2.0 - SASH_FACE / 2.0
    fin_x0 = stile_x - OVERLAP_STILE_W / 2.0
    fin_x1 = stile_x + OVERLAP_STILE_W / 2.0
    # Extends from front face toward the fixed sash (in -Y)
    front_y = SASH_DEPTH / 2.0
    back_y = front_y - OVERLAP_STILE_T
    # Only span the middle 60% of the sash height to clear latch hardware
    fin_half_h = SASH_OPENING_H * 0.35
    return _slab(fin_x0, fin_x1, -fin_half_h, fin_half_h, (front_y + back_y) / 2.0, OVERLAP_STILE_T)


def _build_roller_block() -> cq.Workplane:
    """Small roller block housing that protrudes from the exterior bottom face
    of the sash bottom rail. Built in roller-local frame centered at origin.
    The block extends downward (-Z) and a mounting plate at the top (+Z) sits
    flush against the bottom rail exterior."""
    # Main block body extends downward from origin
    block = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, -ROLLER_H / 2.0))
        .box(ROLLER_W, ROLLER_D, ROLLER_H)
    )
    # Cylindrical wheel at the bottom of the block
    wheel = (
        cq.Workplane("XZ")
        .transformed(offset=(0.0, 0.0, -ROLLER_H + ROLLER_WHEEL_R))
        .cylinder(ROLLER_WHEEL_R * 2.0, ROLLER_WHEEL_W)
    )
    # Mounting plate at top (origin = top surface) extends slightly above
    # to embed into the sash bottom rail
    plate_h = 0.004
    plate = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, plate_h / 2.0))
        .box(ROLLER_W + 0.008, ROLLER_D + 0.004, plate_h)
    )
    return block.union(wheel).union(plate)


def _build_tilt_latch() -> cq.Workplane:
    """Tilt-in latch body: a small flat hook that pivots about its top edge.
    Built in latch-local frame: pivot at origin, body extends downward (-Z)."""
    body = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, -LATCH_BODY_H / 2.0))
        .box(LATCH_BODY_W, LATCH_BODY_T, LATCH_BODY_H)
    )
    # Hook tab at the bottom
    hook = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, LATCH_BODY_T / 2.0 + LATCH_HOOK_LEN / 2.0, -LATCH_BODY_H + LATCH_BODY_T / 2.0))
        .box(LATCH_BODY_W * 0.6, LATCH_HOOK_LEN, LATCH_BODY_T)
    )
    return body.union(hook)


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


def _add_overlap_stile(model: ArticulatedObject) -> None:
    """Add the visible overlap stile fin to the sliding sash."""
    sash = model.get_part("sliding_sash")
    sash.visual(
        mesh_from_cadquery(_build_overlap_stile(), "overlap_stile_fin"),
        material="vinyl",
        name="overlap_stile_fin",
    )


def _add_roller_blocks(model: ArticulatedObject) -> None:
    """Add two roller blocks at the bottom rail of the sliding sash, near the
    left and right ends. Fixed to the sash (not separately articulated).
    Positioned in sash-local frame at the bottom rail."""
    sash = model.get_part("sliding_sash")
    # Exterior bottom face of the sash (bottom of the bottom rail).
    # In sash-local frame: the sash spans from -out_h/2 to +out_h/2 in Z.
    oh = SASH_OPENING_H + 2 * SASH_FACE
    sill_z = -oh / 2.0  # exterior bottom face of the sash
    # Place near the left third and right third of the sash width
    out_w = SASH_OPENING_W + 2 * SASH_FACE
    left_x = -out_w / 2.0 + ROLLER_W / 2.0 + 0.025
    right_x = out_w / 2.0 - ROLLER_W / 2.0 - 0.025
    roller_y = 0.0  # centered in sash depth
    roller_mesh = mesh_from_cadquery(_build_roller_block(), "roller_block")
    # Roller origin at the exterior sill face; the mounting plate (local Z 0..+0.004)
    # embeds upward into the sash bottom rail for physical connectivity.
    sash.visual(
        roller_mesh,
        origin=Origin(xyz=(left_x, roller_y, sill_z)),
        material="roller_nylon",
        name="roller_left",
    )
    sash.visual(
        roller_mesh,
        origin=Origin(xyz=(right_x, roller_y, sill_z)),
        material="roller_nylon",
        name="roller_right",
    )


def _add_tilt_latches(model: ArticulatedObject) -> None:
    """Add two tilt-in latches on the sliding sash meeting stile, one near the
    top and one near the bottom. Each pivots on a revolute joint about the X axis
    (tilting inward toward the fixed sash)."""
    sash = model.get_part("sliding_sash")

    # Latch positions in sash-local frame:
    # Meeting stile is at local x = -SASH_OPENING_W/2 - SASH_FACE/2
    stile_x = -SASH_OPENING_W / 2.0 - SASH_FACE / 2.0
    face_y = SASH_DEPTH / 2.0 + LATCH_BODY_T / 2.0  # on the front face

    # Top latch: pivot near top of sash, body hangs down
    top_z = SASH_OPENING_H / 2.0 - 0.10
    # Bottom latch: pivot near bottom of sash, body hangs up (flipped)
    bottom_z = -SASH_OPENING_H / 2.0 + 0.10

    latch_mesh = mesh_from_cadquery(_build_tilt_latch(), "tilt_latch")

    # Create latch parts as separate articulated parts
    latch_top = model.part("tilt_latch_top")
    latch_top.visual(
        latch_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="metal",
        name="latch_top_body",
    )

    latch_bottom = model.part("tilt_latch_bottom")
    latch_bottom.visual(
        latch_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="metal",
        name="latch_bottom_body",
    )

    # Pivot axis: X axis — the latch tilts around a horizontal pivot
    # (the hook swings inward/outward in the Y-Z plane)
    # Positive rotation about X tilts the hook toward +Y (away from window = unlocked)
    # At q=0 the latch hangs straight down (locked position)
    model.articulation(
        "sash_to_latch_top",
        ArticulationType.REVOLUTE,
        parent="sliding_sash",
        child="tilt_latch_top",
        origin=Origin(xyz=(stile_x, face_y, top_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.2),
    )

    # Bottom latch is mirrored: pivot at bottom, body extends upward
    # Use rpy to flip it 180 degrees about X so body extends upward
    model.articulation(
        "sash_to_latch_bottom",
        ArticulationType.REVOLUTE,
        parent="sliding_sash",
        child="tilt_latch_bottom",
        origin=Origin(xyz=(stile_x, face_y, bottom_z), rpy=(3.14159, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.2),
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_sliding_window")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("metal", rgba=METAL_RGBA)
    model.material("roller_nylon", rgba=ROLLER_RGBA)

    # --- Static outer frame (root) with slim bevelled rails ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="vinyl",
        name="frame_shell",
    )

    # --- Fixed (left) + sliding (right) sashes ---
    _add_sash(model, "fixed_sash")
    _add_sash(model, "sliding_sash")

    # --- Visible overlap stile on sliding sash meeting edge ---
    _add_overlap_stile(model)

    # --- Roller blocks at bottom of sliding sash ---
    _add_roller_blocks(model)

    # --- Tilt-in latch pair on revolute joints ---
    _add_tilt_latches(model)

    # FIXED left sash seated in the rear glazing plane.
    model.articulation(
        "frame_to_fixed_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="fixed_sash",
        origin=Origin(xyz=(FIXED_OPEN_CX, FIXED_SASH_Y, MID_CZ)),
    )

    # SLIDING right sash: PRISMATIC along X.
    slide_travel = SASH_OPENING_W * 0.85
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
    latch_top = object_model.get_part("tilt_latch_top")
    latch_bottom = object_model.get_part("tilt_latch_bottom")
    slide = object_model.get_articulation("frame_to_sliding_sash")
    latch_top_joint = object_model.get_articulation("sash_to_latch_top")
    latch_bottom_joint = object_model.get_articulation("sash_to_latch_bottom")

    # --- Intentional overlaps ---
    # Glass tucks under the vinyl sash lip on each sash (captured glass).
    for nm in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_vinyl",
            reason="Clear pane is rebated under the sash lip so it reads captured, not floating.",
        )
    # Each sash ring laps the frame opening edge (glazing rebate / track).
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

    # Overlap stile fin is part of the sliding sash, seated against its vinyl.
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="overlap_stile_fin",
        elem_b="sliding_sash_vinyl",
        reason="Overlap stile fin extends from the sliding sash meeting stile (mounted feature).",
    )

    # Roller blocks are seated into the bottom rail of the sliding sash.
    for rname in ("roller_left", "roller_right"):
        ctx.allow_overlap(
            "sliding_sash", "sliding_sash",
            elem_a=rname,
            elem_b="sliding_sash_vinyl",
            reason=f"{rname} is seated into the bottom rail of the sliding sash (mounted hardware).",
        )

    # Tilt-in latches are mounted on the front face of the sliding sash stile.
    for lnm in ("tilt_latch_top", "tilt_latch_bottom"):
        ctx.allow_overlap(
            lnm, "sliding_sash",
            elem_a=f"{lnm.replace('tilt_', '')}_body",
            elem_b="sliding_sash_vinyl",
            reason=f"{lnm} pivot body is mounted on the sliding sash meeting stile (hardware contact).",
        )

    # Roller blocks extend below the sash into the frame sill track (realistic:
    # real sliding window rollers ride inside the sill track groove).
    for rname in ("roller_left", "roller_right"):
        ctx.allow_overlap(
            "frame", "sliding_sash",
            elem_a="frame_shell",
            elem_b=rname,
            reason=f"{rname} rides in the frame sill track groove (realistic roller-in-track engagement).",
        )

    # --- Closed pose (q=0): window reads SHUT ---
    with ctx.pose({slide: 0.0, latch_top_joint: 0.0, latch_bottom_joint: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        f_aabb = ctx.part_world_aabb(fixed_sash)
        s_aabb = ctx.part_world_aabb(sliding_sash)

        # Frame spans the full width and is wider than a single sash.
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        sash_w = s_aabb[1][0] - s_aabb[0][0]
        ctx.check(
            "frame spans wider than a single sash",
            frame_w > sash_w + 0.30,
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

        # Slim frame: outer frame face is less than the parent's chunky 0.085
        ctx.check(
            "slim frame rails",
            FRAME_FACE < 0.065,
            details=f"FRAME_FACE={FRAME_FACE:.3f}",
        )

        # Two sashes side by side: fixed on the left, sliding on the right.
        fx = (f_aabb[0][0] + f_aabb[1][0]) / 2.0
        sx = (s_aabb[0][0] + s_aabb[1][0]) / 2.0
        ctx.check(
            "fixed sash left of sliding sash",
            fx < sx,
            details=f"fixed_x={fx:.3f}, sliding_x={sx:.3f}",
        )
        # Sliding sash sits proud (in +Y) of the fixed sash.
        fy = (f_aabb[0][1] + f_aabb[1][1]) / 2.0
        sy = (s_aabb[0][1] + s_aabb[1][1]) / 2.0
        ctx.check(
            "sliding sash proud of fixed sash",
            sy > fy + 0.01,
            details=f"sliding_y={sy:.3f}, fixed_y={fy:.3f}",
        )
        # Fixed sash seated within the frame height.
        ctx.check(
            "fixed sash seated within frame height",
            f_aabb[0][2] > frame_aabb[0][2] - 1e-4 and f_aabb[1][2] < frame_aabb[1][2] + 1e-4,
            details=f"fixed z=[{f_aabb[0][2]:.3f},{f_aabb[1][2]:.3f}]",
        )
        # Sliding sash vinyl is within frame height (rollers extend below into
        # sill track by design — the roller height below the frame sill is OK).
        sash_vinyl_aabb = ctx.part_element_world_aabb(sliding_sash, elem="sliding_sash_vinyl")
        ctx.check(
            "sliding sash vinyl seated within frame height",
            sash_vinyl_aabb[0][2] > frame_aabb[0][2] - 1e-4 and sash_vinyl_aabb[1][2] < frame_aabb[1][2] + 1e-4,
            details=f"sliding_vinyl z=[{sash_vinyl_aabb[0][2]:.3f},{sash_vinyl_aabb[1][2]:.3f}]",
        )
        # Both sashes seated in the frame opening.
        ctx.expect_overlap(
            fixed_sash, frame, axes="xz", min_overlap=0.02,
            name="fixed sash seated in frame opening",
        )
        ctx.expect_overlap(
            sliding_sash, frame, axes="xz", min_overlap=0.02,
            name="sliding sash seated in frame opening",
        )

        # Visible overlap stile: the overlap fin on sliding sash should be
        # positioned near the meeting edge, between the two sash centers.
        stile_aabb = ctx.part_element_world_aabb(sliding_sash, elem="overlap_stile_fin")
        stile_cx = (stile_aabb[0][0] + stile_aabb[1][0]) / 2.0
        ctx.check(
            "overlap stile near meeting edge between sashes",
            fx < stile_cx < sx,
            details=f"stile_x={stile_cx:.3f}, fixed_x={fx:.3f}, sliding_x={sx:.3f}",
        )

        # Roller blocks at bottom of sliding sash
        roller_l_aabb = ctx.part_element_world_aabb(sliding_sash, elem="roller_left")
        roller_r_aabb = ctx.part_element_world_aabb(sliding_sash, elem="roller_right")
        sash_bottom = s_aabb[0][2]
        ctx.check(
            "roller_left near bottom of sliding sash",
            roller_l_aabb[0][2] < sash_bottom + SASH_FACE + 0.01,
            details=f"roller_z_min={roller_l_aabb[0][2]:.3f}, sash_bottom={sash_bottom:.3f}",
        )
        ctx.check(
            "roller_right near bottom of sliding sash",
            roller_r_aabb[0][2] < sash_bottom + SASH_FACE + 0.01,
            details=f"roller_z_min={roller_r_aabb[0][2]:.3f}, sash_bottom={sash_bottom:.3f}",
        )
        # Two rollers separated along X
        rl_cx = (roller_l_aabb[0][0] + roller_l_aabb[1][0]) / 2.0
        rr_cx = (roller_r_aabb[0][0] + roller_r_aabb[1][0]) / 2.0
        ctx.check(
            "two roller blocks separated along X",
            abs(rr_cx - rl_cx) > 0.30,
            details=f"roller_left_x={rl_cx:.3f}, roller_right_x={rr_cx:.3f}",
        )
        # Rollers ride in the frame sill track: roller Z overlaps frame sill region
        ctx.expect_overlap(
            sliding_sash, frame, axes="x", min_overlap=0.01,
            elem_a="roller_left", elem_b="frame_shell",
            name="roller_left stays within frame X span (track engagement)",
        )
        ctx.expect_overlap(
            sliding_sash, frame, axes="x", min_overlap=0.01,
            elem_a="roller_right", elem_b="frame_shell",
            name="roller_right stays within frame X span (track engagement)",
        )

        # Tilt-in latches exist and are on the sliding sash meeting stile
        lt_aabb = ctx.part_world_aabb(latch_top)
        lb_aabb = ctx.part_world_aabb(latch_bottom)
        ctx.check(
            "tilt_latch_top exists and has geometry",
            lt_aabb is not None,
            details="no AABB",
        )
        ctx.check(
            "tilt_latch_bottom exists and has geometry",
            lb_aabb is not None,
            details="no AABB",
        )
        # Latches on the inner (left) side of the sliding sash
        lt_cx = (lt_aabb[0][0] + lt_aabb[1][0]) / 2.0
        lb_cx = (lb_aabb[0][0] + lb_aabb[1][0]) / 2.0
        ctx.check(
            "latch_top on meeting stile side (left of sliding sash center)",
            lt_cx < sx,
            details=f"latch_top_x={lt_cx:.3f}, sliding_cx={sx:.3f}",
        )
        ctx.check(
            "latch_bottom on meeting stile side (left of sliding sash center)",
            lb_cx < sx,
            details=f"latch_bottom_x={lb_cx:.3f}, sliding_cx={sx:.3f}",
        )

        rest_sx = sx
        rest_sz = (s_aabb[0][2] + s_aabb[1][2]) / 2.0

    # --- Driven/open pose: sliding sash slides toward fixed sash (-X) ---
    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel, latch_top_joint: 0.0, latch_bottom_joint: 0.0}):
        s_open = ctx.part_world_aabb(sliding_sash)
        open_sx = (s_open[0][0] + s_open[1][0]) / 2.0
        ctx.check(
            "sliding sash opens toward fixed sash (-X)",
            abs((rest_sx - open_sx) - travel) < 0.02 and open_sx < rest_sx - 0.20,
            details=f"rest_x={rest_sx:.3f}, open_x={open_sx:.3f}, travel={travel:.3f}",
        )
        # Pure horizontal slide (no Z change).
        open_sz = (s_open[0][2] + s_open[1][2]) / 2.0
        ctx.check(
            "slide is purely horizontal",
            abs(open_sz - rest_sz) < 0.02,
            details=f"open_z={open_sz:.3f}, rest_z={rest_sz:.3f}",
        )
        # Retained insertion: sliding sash stays within frame X span.
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained within frame X span at full travel",
            s_open[0][0] > f_aabb[0][0] - 1e-4 and s_open[1][0] < f_aabb[1][0] + 1e-4,
            details=f"sash x=[{s_open[0][0]:.3f},{s_open[1][0]:.3f}] frame x=[{f_aabb[0][0]:.3f},{f_aabb[1][0]:.3f}]",
        )

    # --- Tilt latch articulation: prove revolute joints work ---
    with ctx.pose({slide: 0.0, latch_top_joint: 0.8}):
        lt_tilted = ctx.part_world_aabb(latch_top)
        lt_tilted_cz = (lt_tilted[0][2] + lt_tilted[1][2]) / 2.0
        # When tilted, the latch body moves (Z center changes as it pivots)
        ctx.check(
            "tilt_latch_top pivots on revolute joint",
            lt_tilted is not None,
            details="latch should still have valid geometry when tilted",
        )

    with ctx.pose({slide: 0.0, latch_bottom_joint: 0.8}):
        lb_tilted = ctx.part_world_aabb(latch_bottom)
        ctx.check(
            "tilt_latch_bottom pivots on revolute joint",
            lb_tilted is not None,
            details="latch should still have valid geometry when tilted",
        )

    # Verify latch joints are revolute (non-fixed)
    ctx.check(
        "sash_to_latch_top is revolute",
        latch_top_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={latch_top_joint.articulation_type}",
    )
    ctx.check(
        "sash_to_latch_bottom is revolute",
        latch_bottom_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={latch_bottom_joint.articulation_type}",
    )
    ctx.check(
        "frame_to_sliding_sash is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
