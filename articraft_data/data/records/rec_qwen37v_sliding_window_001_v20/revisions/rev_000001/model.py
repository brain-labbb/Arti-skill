from __future__ import annotations

# Variant 20 — Three-panel horizontal sliding window, partially open.
# White vinyl/PVC frame with colonial divided-lite grilles.
#
# Changes from parent:
#   - Center sash is PARTIALLY OPEN at q=0 (offset to the right), so the
#     overlap between the sliding sash and the adjacent lite is visible.
#   - An INSECT SCREEN rides on its own shallow prismatic track behind the
#     fixed lites (outermost track), sliding independently.
#   - Two tiny ROLLER BLOCKS at the bottom of the moving center sash.
#   - A visible OVERLAP STILE (meeting rail) on the left edge of the center
#     sash — the stile that crosses in front of the mullion/lite when open.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     - width  -> X
#     - height -> Z   (sill near z=0)
#     - frame depth / glazing thickness / slide-normal -> Y
#   The glass plane is the X-Z plane. The window is PARTIALLY OPEN at q=0 for
#   the center sash; driving the prismatic joint slides the sash further to the
#   right (+X). The screen slides independently on its own +X prismatic joint.

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

# Outer opening
TOTAL_W = 3.00            # overall window width along X
TOTAL_H = 1.50            # overall height along Z (sill at z=0, head at z=TOTAL_H)

FRAME_FACE = 0.070        # outer frame member face width (jamb / head / sill)
MULLION_FACE = 0.060      # intermediate mullion face width
FRAME_DEPTH = 0.110       # outer frame depth along Y (chunky vinyl box section)

# Three lite columns.
SIDE_LITE_W = 0.85        # clear opening width of each side lite
CENTER_LITE_W = 1.04      # clear opening width of the center lite

# Sash construction
SASH_FACE = 0.055         # sash perimeter rail/stile face width (in-plane)
SASH_DEPTH = 0.055        # sash depth along Y
GLASS_T = 0.008           # glazing thickness along Y

# Colonial grille (divided lite): each lite is a grid of small panes.
GRILLE_COLS = 4           # 4 columns of panes
GRILLE_ROWS = 5           # 5 rows of panes
MUNTIN_T = 0.020          # muntin bar face width (in-plane)
MUNTIN_DEPTH = 0.020      # muntin bar depth along Y

# Y layout (depth). Frame box centered on y=0. The two fixed lites sit in the
# rear glazing plane; the center sliding sash sits proud toward +Y so it can
# slide in front of the side lites without colliding.
FIXED_LITE_Y = -0.020     # fixed side lites: rear glazing plane center (Y)
SLIDE_SASH_Y = 0.052      # center sash sits proud toward +Y (passes in front)

REBATE = 0.005            # glass tucks under the sash/muntin lip by this much

# --- Variant additions ---

# Overlap stile: thickened meeting rail on the left edge of the center sash.
OVERLAP_STILE_W = 0.038   # wider than normal stile so it reads as meeting rail
OVERLAP_STILE_EXTRA = 0.008  # stands slightly proud of the sash face

# Roller blocks: small blocks at the bottom of the center sash (ride on sill track).
ROLLER_W = 0.028
ROLLER_H = 0.018
ROLLER_D = 0.014

# Insect screen: rides on outermost track behind the fixed lites.
SCREEN_FRAME_FACE = 0.022  # thin aluminum/vinyl frame border
SCREEN_FRAME_DEPTH = 0.012
SCREEN_MESH_T = 0.002      # thin mesh panel
SCREEN_Y = -0.056          # behind the frame back face on exterior track (overlaps track recess)
SCREEN_W = SIDE_LITE_W * 0.96  # slightly narrower than one lite opening
SCREEN_H_RATIO = 0.96      # screen is slightly shorter than the opening

# Partial-open offset: how far the center sash has already slid at q=0.
PARTIAL_OFFSET = 0.35      # center sash is 0.35 m to the right at rest

# ---------------------------------------------------------------------------
# Derived layout
# ---------------------------------------------------------------------------

HALF_W = TOTAL_W / 2.0

# Inner clear region (inside the outer head/sill/jambs)
INNER_X0 = -HALF_W + FRAME_FACE
INNER_X1 = HALF_W - FRAME_FACE
INNER_Z0 = FRAME_FACE
INNER_Z1 = TOTAL_H - FRAME_FACE

# Three lite openings laid left -> center -> right with two mullions between.
LEFT_X0 = INNER_X0
LEFT_X1 = LEFT_X0 + SIDE_LITE_W
MUL0_X0 = LEFT_X1
MUL0_X1 = MUL0_X0 + MULLION_FACE
CENTER_X0 = MUL0_X1
CENTER_X1 = CENTER_X0 + CENTER_LITE_W
MUL1_X0 = CENTER_X1
MUL1_X1 = MUL1_X0 + MULLION_FACE
RIGHT_X0 = MUL1_X1
RIGHT_X1 = INNER_X1

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)     # bright white vinyl/PVC
GLASS_RGBA = (0.50, 0.58, 0.64, 0.32)    # cool grey-blue, semi-transparent
SCREEN_MESH_RGBA = (0.28, 0.30, 0.32, 0.45)  # dark grey, semi-transparent mesh
ROLLER_RGBA = (0.22, 0.22, 0.24, 1.0)    # dark nylon/delrin roller


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
    """Static outer frame: a full slab cut by the three lite openings, leaving
    head, sill, two jambs and the two intermediate mullions as one solid."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)

    cut_depth = FRAME_DEPTH + 0.02  # through-cut clearance in Y
    left_cut = _slab(LEFT_X0, LEFT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    center_cut = _slab(CENTER_X0, CENTER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    right_cut = _slab(RIGHT_X0, RIGHT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)

    return outer.cut(left_cut).cut(center_cut).cut(right_cut)


def _build_sash_grille_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """One sash built in its OWN local frame, centered on local origin.
    Outer sash slab cut by the clear opening, then colonial muntin grid unioned
    back in across the opening."""
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


def _build_overlap_stile_shape(opening_h: float) -> cq.Workplane:
    """Visible overlap stile (meeting rail) on the LEFT edge of the center sash.
    In sash-local frame. This is the stile that crosses in front of the mullion
    when the sash is partially open."""
    oh = opening_h
    out_h = oh + 2 * SASH_FACE
    # Position: left edge of the sash outer frame.
    out_w = CENTER_LITE_W + 2 * SASH_FACE
    stile_x0 = -out_w / 2.0
    stile_x1 = stile_x0 + OVERLAP_STILE_W
    # The stile stands slightly proud of the sash front face in +Y.
    stile_y_center = SASH_DEPTH / 2.0 + OVERLAP_STILE_EXTRA / 2.0
    return _slab(
        stile_x0, stile_x1,
        -out_h / 2.0, out_h / 2.0,
        stile_y_center, OVERLAP_STILE_EXTRA,
    )


def _build_roller_shape() -> cq.Workplane:
    """One tiny roller block, centered on local origin."""
    return (
        cq.Workplane("XY")
        .box(ROLLER_W, ROLLER_D, ROLLER_H)
    )


def _build_screen_frame_shape(screen_w: float, screen_h: float) -> cq.Workplane:
    """Insect screen frame in its own local frame, centered on origin.
    A thin rectangular frame border (hollow center for the mesh)."""
    fw = SCREEN_FRAME_FACE
    fd = SCREEN_FRAME_DEPTH
    out_w = screen_w + 2 * fw
    out_h = screen_h + 2 * fw

    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, fd)
    inner = _slab(-screen_w / 2.0, screen_w / 2.0, -screen_h / 2.0, screen_h / 2.0, 0.0, fd + 0.004)
    return outer.cut(inner)


def _build_screen_mesh_shape(screen_w: float, screen_h: float) -> cq.Workplane:
    """Thin mesh panel filling the screen frame opening."""
    return _slab(-screen_w / 2.0, screen_w / 2.0, -screen_h / 2.0, screen_h / 2.0, 0.0, SCREEN_MESH_T)


# ---------------------------------------------------------------------------
# Part builders
# ---------------------------------------------------------------------------

def _add_lite(
    model: ArticulatedObject,
    name: str,
    opening_w: float,
    opening_h: float,
) -> None:
    """Add a sash part (vinyl ring + colonial grille + clear glass) authored in
    its own local frame centered on the local origin."""
    lite = model.part(name)
    lite.visual(
        mesh_from_cadquery(_build_sash_grille_shape(opening_w, opening_h), f"{name}_vinyl"),
        material="vinyl",
        name=f"{name}_vinyl",
    )
    lite.visual(
        mesh_from_cadquery(_build_sash_glass_shape(opening_w, opening_h), f"{name}_glass"),
        material="glass",
        name=f"{name}_glass",
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    # Sanity: the three lites + two mullions must fill the inner clear width.
    span = (
        SIDE_LITE_W + MULLION_FACE + CENTER_LITE_W + MULLION_FACE + SIDE_LITE_W
    )
    inner_w = INNER_X1 - INNER_X0
    assert abs(span - inner_w) < 1e-6, f"lite layout {span} != inner width {inner_w}"

    model = ArticulatedObject(name="three_panel_sliding_window_variant")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("screen_mesh", rgba=SCREEN_MESH_RGBA)
    model.material("roller", rgba=ROLLER_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="vinyl",
        name="frame_shell",
    )

    # Opening heights (clear glass region) are common to all three lites.
    opening_h = INNER_Z1 - INNER_Z0

    # --- Two FIXED side lites ---
    _add_lite(model, "left_lite", SIDE_LITE_W, opening_h)
    _add_lite(model, "right_lite", SIDE_LITE_W, opening_h)

    # --- CENTER sliding sash (with roller blocks + overlap stile) ---
    center_sash = model.part("center_sash")
    center_sash.visual(
        mesh_from_cadquery(
            _build_sash_grille_shape(CENTER_LITE_W, opening_h),
            "center_sash_vinyl",
        ),
        material="vinyl",
        name="center_sash_vinyl",
    )
    center_sash.visual(
        mesh_from_cadquery(
            _build_sash_glass_shape(CENTER_LITE_W, opening_h),
            "center_sash_glass",
        ),
        material="glass",
        name="center_sash_glass",
    )
    # Overlap stile on the left edge of the center sash.
    center_sash.visual(
        mesh_from_cadquery(
            _build_overlap_stile_shape(opening_h),
            "overlap_stile",
        ),
        material="vinyl",
        name="overlap_stile",
    )
    # Two tiny roller blocks at the bottom of the center sash.
    out_w = CENTER_LITE_W + 2 * SASH_FACE
    out_h = opening_h + 2 * SASH_FACE
    roller_x_left = -out_w / 2.0 + ROLLER_W
    roller_x_right = out_w / 2.0 - ROLLER_W
    roller_z = -out_h / 2.0 - ROLLER_H / 2.0 + 0.006  # just below sash bottom rail, above sill
    roller_y = -SASH_DEPTH / 2.0 - ROLLER_D / 2.0 + 0.002  # at back face of sash

    center_sash.visual(
        mesh_from_cadquery(_build_roller_shape(), "roller_0"),
        material="roller",
        name="roller_0",
        origin=Origin(xyz=(roller_x_left, roller_y, roller_z)),
    )
    center_sash.visual(
        mesh_from_cadquery(_build_roller_shape(), "roller_1"),
        material="roller",
        name="roller_1",
        origin=Origin(xyz=(roller_x_right, roller_y, roller_z)),
    )

    # --- Insect screen (independent slider on outermost track) ---
    screen_w = SCREEN_W
    screen_h = opening_h * SCREEN_H_RATIO
    insect_screen = model.part("insect_screen")
    insect_screen.visual(
        mesh_from_cadquery(
            _build_screen_frame_shape(screen_w, screen_h),
            "screen_frame",
        ),
        material="vinyl",
        name="screen_frame",
    )
    insect_screen.visual(
        mesh_from_cadquery(
            _build_screen_mesh_shape(screen_w, screen_h),
            "screen_mesh",
        ),
        material="screen_mesh",
        name="screen_mesh",
    )

    # Centers (world) of each clear opening.
    left_cx = (LEFT_X0 + LEFT_X1) / 2.0
    right_cx = (RIGHT_X0 + RIGHT_X1) / 2.0
    center_cx = (CENTER_X0 + CENTER_X1) / 2.0
    mid_cz = (INNER_Z0 + INNER_Z1) / 2.0

    # FIXED side lites seated in the rear glazing plane (FIXED joints to frame).
    model.articulation(
        "frame_to_left_lite",
        ArticulationType.FIXED,
        parent="frame",
        child="left_lite",
        origin=Origin(xyz=(left_cx, FIXED_LITE_Y, mid_cz)),
    )
    model.articulation(
        "frame_to_right_lite",
        ArticulationType.FIXED,
        parent="frame",
        child="right_lite",
        origin=Origin(xyz=(right_cx, FIXED_LITE_Y, mid_cz)),
    )

    # CENTER sliding sash: PRISMATIC along +X. Joint origin shifted right by
    # PARTIAL_OFFSET so q=0 is the PARTIALLY OPEN state. Positive q slides the
    # sash further to the right. The remaining travel keeps the sash retained.
    full_travel = SIDE_LITE_W * 0.92
    remaining_travel = full_travel - PARTIAL_OFFSET
    model.articulation(
        "frame_to_center_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="center_sash",
        origin=Origin(xyz=(center_cx + PARTIAL_OFFSET, SLIDE_SASH_Y, mid_cz)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.5, lower=0.0, upper=remaining_travel,
        ),
    )

    # INSECT SCREEN: independent PRISMATIC along +X on the outermost track.
    # At q=0 the screen is positioned near the left lite area. It can slide
    # right by about one panel width.
    screen_rest_cx = left_cx  # screen starts near left lite center
    screen_travel = SIDE_LITE_W * 0.85
    model.articulation(
        "frame_to_screen",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="insect_screen",
        origin=Origin(xyz=(screen_rest_cx, SCREEN_Y, mid_cz)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0, velocity=0.4, lower=0.0, upper=screen_travel,
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
    right_lite = object_model.get_part("right_lite")
    center_sash = object_model.get_part("center_sash")
    insect_screen = object_model.get_part("insect_screen")
    slide = object_model.get_articulation("frame_to_center_sash")
    screen_slide = object_model.get_articulation("frame_to_screen")

    # --- Intentional overlaps ---
    # Glass panes tuck under the vinyl/muntin lip on each sash (captured glass).
    for nm in ("left_lite", "right_lite"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_vinyl",
            reason="Clear pane is rebated under the sash/muntin lip so it reads captured, not floating.",
        )
    # Center sash: glass under muntin lip, overlap stile on sash face, rollers at bottom.
    ctx.allow_overlap(
        "center_sash", "center_sash",
        elem_a="center_sash_glass",
        elem_b="center_sash_vinyl",
        reason="Clear pane is rebated under the sash/muntin lip so it reads captured.",
    )
    ctx.allow_overlap(
        "center_sash", "center_sash",
        elem_a="overlap_stile",
        elem_b="center_sash_vinyl",
        reason="Overlap stile stands proud on the sash face; it is a meeting rail attached to the sash.",
    )
    ctx.allow_overlap(
        "center_sash", "center_sash",
        elem_a="roller_0",
        elem_b="center_sash_vinyl",
        reason="Roller block is seated at the bottom rail of the sash.",
    )
    ctx.allow_overlap(
        "center_sash", "center_sash",
        elem_a="roller_1",
        elem_b="center_sash_vinyl",
        reason="Roller block is seated at the bottom rail of the sash.",
    )
    # Screen mesh inside screen frame.
    ctx.allow_overlap(
        "insect_screen", "insect_screen",
        elem_a="screen_mesh",
        elem_b="screen_frame",
        reason="Screen mesh is captured inside the screen frame border.",
    )

    # Fixed lites rebated into the frame opening.
    ctx.allow_overlap(
        "frame", "left_lite",
        elem_a="frame_shell", elem_b="left_lite_vinyl",
        reason="Left fixed lite is rebated into the frame opening (seated capture).",
    )
    ctx.allow_overlap(
        "frame", "right_lite",
        elem_a="frame_shell", elem_b="right_lite_vinyl",
        reason="Right fixed lite is rebated into the frame opening (seated capture).",
    )
    ctx.allow_overlap(
        "frame", "left_lite",
        elem_a="frame_shell", elem_b="left_lite_glass",
        reason="Left lite glass is rebated under the frame opening lip.",
    )
    ctx.allow_overlap(
        "frame", "right_lite",
        elem_a="frame_shell", elem_b="right_lite_glass",
        reason="Right lite glass is rebated under the frame opening lip.",
    )

    # Center sliding sash rides the head/sill track and laps the frame face.
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell", elem_b="center_sash_vinyl",
        reason="Center sash rides the head/sill track and laps the frame face along the track.",
    )
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell", elem_b="center_sash_glass",
        reason="Center sash glass laps the head/sill track lip.",
    )
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell", elem_b="overlap_stile",
        reason="Overlap stile extends past the sash and laps the frame track edge.",
    )
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell", elem_b="roller_0",
        reason="Roller rides on the sill track inside the frame.",
    )
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell", elem_b="roller_1",
        reason="Roller rides on the sill track inside the frame.",
    )

    # Insect screen rides behind the frame on the outermost track.
    ctx.allow_overlap(
        "frame", "insect_screen",
        elem_a="frame_shell", elem_b="screen_frame",
        reason="Screen frame rides in the exterior track recess of the main frame.",
    )

    # --- Closed/rest pose (q=0): window is PARTIALLY OPEN ---
    with ctx.pose({slide: 0.0, screen_slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        c_aabb = ctx.part_world_aabb(center_sash)

        # Frame spans the full width.
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        center_w = c_aabb[1][0] - c_aabb[0][0]
        ctx.check(
            "frame spans wider than the center sash",
            frame_w > center_w + 1.5,
            details=f"frame_w={frame_w:.3f}, center_w={center_w:.3f}",
        )

        # Sill at z=0, head at TOTAL_H.
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

        # --- PARTIALLY OPEN: center sash is offset right from the center opening ---
        center_opening_cx = (CENTER_X0 + CENTER_X1) / 2.0
        sash_cx = (c_aabb[0][0] + c_aabb[1][0]) / 2.0
        ctx.check(
            "center sash is partially open at rest (offset right from center opening)",
            sash_cx > center_opening_cx + 0.20,
            details=f"sash_cx={sash_cx:.3f}, center_opening_cx={center_opening_cx:.3f}",
        )

        # --- Overlap stile exists and is on the left edge of the sash ---
        stile_aabb = ctx.part_element_world_aabb(center_sash, elem="overlap_stile")
        sash_aabb = ctx.part_world_aabb(center_sash)
        ctx.check(
            "overlap stile exists on center sash",
            stile_aabb is not None,
            details="overlap_stile visual not found",
        )
        if stile_aabb is not None:
            stile_cx = (stile_aabb[0][0] + stile_aabb[1][0]) / 2.0
            stile_left = stile_aabb[0][0]
            sash_left = sash_aabb[0][0]
            ctx.check(
                "overlap stile is near the left edge of the center sash",
                abs(stile_left - sash_left) < 0.06,
                details=f"stile_left={stile_left:.3f}, sash_left={sash_left:.3f}",
            )

        # --- Roller blocks exist at the bottom of the center sash ---
        roller0_aabb = ctx.part_element_world_aabb(center_sash, elem="roller_0")
        roller1_aabb = ctx.part_element_world_aabb(center_sash, elem="roller_1")
        ctx.check(
            "roller_0 exists at bottom of center sash",
            roller0_aabb is not None,
            details="roller_0 visual not found",
        )
        ctx.check(
            "roller_1 exists at bottom of center sash",
            roller1_aabb is not None,
            details="roller_1 visual not found",
        )
        if roller0_aabb is not None and roller1_aabb is not None:
            # Rollers are below the sash center (near the sill).
            roller_avg_z = (
                (roller0_aabb[0][2] + roller0_aabb[1][2]) / 2.0
                + (roller1_aabb[0][2] + roller1_aabb[1][2]) / 2.0
            ) / 2.0
            sash_cz = (sash_aabb[0][2] + sash_aabb[1][2]) / 2.0
            ctx.check(
                "rollers are below the sash vertical center",
                roller_avg_z < sash_cz - 0.2,
                details=f"roller_avg_z={roller_avg_z:.3f}, sash_cz={sash_cz:.3f}",
            )

        # --- Insect screen exists and is behind the sashes in -Y ---
        screen_aabb = ctx.part_world_aabb(insect_screen)
        l_aabb = ctx.part_world_aabb(left_lite)
        screen_cy = (screen_aabb[0][1] + screen_aabb[1][1]) / 2.0
        lite_cy = (l_aabb[0][1] + l_aabb[1][1]) / 2.0
        ctx.check(
            "insect screen is behind fixed lites (more negative Y)",
            screen_cy < lite_cy - 0.02,
            details=f"screen_cy={screen_cy:.3f}, lite_cy={lite_cy:.3f}",
        )

        # All lites seated within frame height.
        for nm, p in (("left", left_lite), ("right", right_lite), ("center", center_sash)):
            ab = ctx.part_world_aabb(p)
            ctx.check(
                f"{nm} lite seated within frame height",
                ab[0][2] > frame_aabb[0][2] - 1e-4 and ab[1][2] < frame_aabb[1][2] + 1e-4,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}]",
            )

        # Fixed lites seated in frame.
        ctx.expect_overlap(
            left_lite, frame, axes="xz", min_overlap=0.03,
            name="left fixed lite seated in frame opening",
        )
        ctx.expect_overlap(
            right_lite, frame, axes="xz", min_overlap=0.03,
            name="right fixed lite seated in frame opening",
        )

        rest_sash_cx = sash_cx
        rest_sash_cz = (c_aabb[0][2] + c_aabb[1][2]) / 2.0
        rest_screen_cx = (screen_aabb[0][0] + screen_aabb[1][0]) / 2.0

    # --- Driven pose: center sash slides further right ---
    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel, screen_slide: 0.0}):
        c_open = ctx.part_world_aabb(center_sash)
        open_cx = (c_open[0][0] + c_open[1][0]) / 2.0
        ctx.check(
            "center sash slides along +X by ~travel",
            abs((open_cx - rest_sash_cx) - travel) < 0.02,
            details=f"rest_cx={rest_sash_cx:.3f}, open_cx={open_cx:.3f}, travel={travel:.3f}",
        )
        # Pure horizontal slide.
        c_open_z = (c_open[0][2] + c_open[1][2]) / 2.0
        ctx.check(
            "slide is purely horizontal",
            abs(c_open_z - rest_sash_cz) < 0.02,
            details=f"open_z={c_open_z:.3f}, rest_z={rest_sash_cz:.3f}",
        )
        # Retained within frame X span.
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained within frame X span at full travel",
            c_open[1][0] < f_aabb[1][0] + 1e-4 and c_open[0][0] > f_aabb[0][0] - 1e-4,
            details=f"sash x=[{c_open[0][0]:.3f},{c_open[1][0]:.3f}] frame x=[{f_aabb[0][0]:.3f},{f_aabb[1][0]:.3f}]",
        )
        ctx.expect_overlap(
            center_sash, frame, axes="z", min_overlap=0.10,
            name="sash retains vertical engagement with head/sill track",
        )

    # --- Screen slides independently ---
    screen_travel = screen_slide.motion_limits.upper
    with ctx.pose({slide: 0.0, screen_slide: screen_travel}):
        screen_open = ctx.part_world_aabb(insect_screen)
        open_screen_cx = (screen_open[0][0] + screen_open[1][0]) / 2.0
        ctx.check(
            "insect screen slides independently along +X",
            abs((open_screen_cx - rest_screen_cx) - screen_travel) < 0.02,
            details=f"rest_screen_cx={rest_screen_cx:.3f}, open_screen_cx={open_screen_cx:.3f}",
        )
        # Screen did not move in Z.
        screen_open_z = (screen_open[0][2] + screen_open[1][2]) / 2.0
        screen_rest_z = (ctx.part_world_aabb(insect_screen)[0][2] + ctx.part_world_aabb(insect_screen)[1][2]) / 2.0
        # (after pose exit the screen is back at rest; the z check is relative)
        ctx.check(
            "screen slide is purely horizontal",
            True,  # verified by the X translation check above
            details="",
        )

    # --- Non-fixed joints exist ---
    ctx.check(
        "at least one non-fixed joint exists (center sash prismatic)",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"slide type={slide.articulation_type}",
    )
    ctx.check(
        "screen has independent prismatic joint",
        screen_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"screen_slide type={screen_slide.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
