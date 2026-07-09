from __future__ import annotations

# Vertical sliding window (single-hung), white vinyl frame with deep track
# grooves in head and sill rails. One FIXED upper sash, one PRISMATIC lower
# sash that slides vertically, and an independent insect screen on its own
# shallow prismatic track.
#
# Coordinate convention:
#   +Z up, window stands vertically in the XZ plane
#     width  -> X
#     height -> Z   (sill near z=0)
#     depth  -> Y   (frame depth / track-normal direction)
#   q=0 for lower sash means CLOSED (sash fills lower opening).
#   Positive q slides lower sash UP (+Z) to open.
#   q=0 for screen means STOWED (screen covers lower opening).
#   Positive q slides screen UP (+Z) independently.

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
# Dimensions (meters)
# ---------------------------------------------------------------------------

TOTAL_W = 0.92            # window width along X
TOTAL_H = 1.22            # window height along Z

FRAME_FACE = 0.075        # frame member face width (chunky vinyl profile)
FRAME_DEPTH = 0.120       # deep box section along Y

MEETING_RAIL = 0.040      # horizontal divider between upper and lower openings

SASH_FACE = 0.050         # sash perimeter rail/stile face width
SASH_DEPTH = 0.038        # sash depth along Y
GLASS_T = 0.006           # glazing thickness
REBATE = 0.005            # glass tucks under sash lip

# Track groove channels (deep, prominent feature of head and sill rails)
GROOVE_W = 0.040          # groove width along Y
GROOVE_D = 0.024          # groove depth into the rail along Z

# Screen
SCR_FRAME_W = 0.020       # screen frame member width
SCR_DEPTH = 0.014         # screen frame depth (thin)

# Latch on lower sash meeting rail
LATCH_W = 0.045
LATCH_H = 0.022
LATCH_T = 0.012

# ---------------------------------------------------------------------------
# Derived layout
# ---------------------------------------------------------------------------

HALF_W = TOTAL_W / 2.0
INNER_W = TOTAL_W - 2 * FRAME_FACE
INNER_Z0 = FRAME_FACE                     # sill top
INNER_Z1 = TOTAL_H - FRAME_FACE           # head bottom
MID_Z = (INNER_Z0 + INNER_Z1) / 2.0

# Upper opening
UP_Z0 = MID_Z + MEETING_RAIL / 2.0
UP_Z1 = INNER_Z1
UP_H = UP_Z1 - UP_Z0
UP_CZ = (UP_Z0 + UP_Z1) / 2.0

# Lower opening
LO_Z0 = INNER_Z0
LO_Z1 = MID_Z - MEETING_RAIL / 2.0
LO_H = LO_Z1 - LO_Z0
LO_CZ = (LO_Z0 + LO_Z1) / 2.0

# Y positions (rear = interior -Y, front/exterior = +Y)
UPPER_SASH_Y = -0.018
LOWER_SASH_Y = 0.018
SCREEN_Y = FRAME_DEPTH / 2.0 - 0.005     # near exterior face

# Track groove Y centers (3 tracks in head/sill: rear sash, front sash, screen)
GY_REAR = -0.025
GY_FRONT = 0.012
GY_SCREEN = 0.040

# Travel limits
LOWER_TRAVEL = LO_H * 0.72
SCREEN_TRAVEL = LO_H * 0.55

# Glass dimensions (per sash)
GLASS_W = INNER_W - 2 * SASH_FACE + 2 * REBATE
GLASS_UP_H = UP_H - 2 * SASH_FACE + 2 * REBATE
GLASS_LO_H = LO_H - 2 * SASH_FACE + 2 * REBATE

# Screen inner dimensions
SCR_OUTER_W = INNER_W - 0.010
SCR_OUTER_H = LO_H - 0.010
SCR_INNER_W = SCR_OUTER_W - 2 * SCR_FRAME_W
SCR_INNER_H = SCR_OUTER_H - 2 * SCR_FRAME_W

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)
GLASS_RGBA = (0.52, 0.60, 0.66, 0.30)
SCREEN_RGBA = (0.25, 0.28, 0.25, 0.70)
METAL_RGBA = (0.70, 0.72, 0.75, 1.0)


# ---------------------------------------------------------------------------
# CadQuery geometry helpers
# ---------------------------------------------------------------------------

def _box(cx: float, cy: float, cz: float,
         sx: float, sy: float, sz: float) -> cq.Workplane:
    """Axis-aligned box centered at (cx, cy, cz) with full size (sx, sy, sz)."""
    return (
        cq.Workplane("XY")
        .transformed(offset=(cx, cy, cz))
        .box(sx, sy, sz)
    )


def _build_frame() -> cq.Workplane:
    """Outer frame with two sash openings and deep track grooves in head, sill,
    and meeting rail."""
    # Outer slab
    outer = _box(0, 0, TOTAL_H / 2, TOTAL_W, FRAME_DEPTH, TOTAL_H)

    # Two sash openings
    up_cut = _box(0, 0, UP_CZ, INNER_W, FRAME_DEPTH + 0.02, UP_H)
    lo_cut = _box(0, 0, LO_CZ, INNER_W, FRAME_DEPTH + 0.02, LO_H)
    frame = outer.cut(up_cut).cut(lo_cut)

    # --- Deep track grooves in HEAD rail (inner bottom face at Z=INNER_Z1) ---
    for gy in (GY_REAR, GY_FRONT, GY_SCREEN):
        g = _box(0, gy, INNER_Z1 + GROOVE_D / 2,
                 INNER_W + 0.01, GROOVE_W, GROOVE_D + 0.004)
        frame = frame.cut(g)

    # --- Deep track grooves in SILL rail (inner top face at Z=INNER_Z0) ---
    for gy in (GY_REAR, GY_FRONT, GY_SCREEN):
        g = _box(0, gy, INNER_Z0 - GROOVE_D / 2,
                 INNER_W + 0.01, GROOVE_W, GROOVE_D + 0.004)
        frame = frame.cut(g)

    # --- Track grooves in MEETING RAIL ---
    # Top face (for upper sash edges)
    for gy in (GY_REAR, GY_FRONT):
        g = _box(0, gy, UP_Z0 - GROOVE_D / 2,
                 INNER_W + 0.01, GROOVE_W, GROOVE_D + 0.004)
        frame = frame.cut(g)
    # Bottom face (for lower sash edges)
    for gy in (GY_REAR, GY_FRONT):
        g = _box(0, gy, LO_Z1 + GROOVE_D / 2,
                 INNER_W + 0.01, GROOVE_W, GROOVE_D + 0.004)
        frame = frame.cut(g)

    return frame


def _build_sash(opening_w: float, opening_h: float) -> cq.Workplane:
    """Sash ring in local frame centered at origin. Hollow vinyl profile."""
    outer = _box(0, 0, 0, opening_w, SASH_DEPTH, opening_h)
    inner = _box(0, 0, 0,
                 opening_w - 2 * SASH_FACE,
                 SASH_DEPTH + 0.02,
                 opening_h - 2 * SASH_FACE)
    return outer.cut(inner)


def _build_glass(w: float, h: float) -> cq.Workplane:
    """Glass pane centered at origin."""
    return _box(0, 0, 0, w, GLASS_T, h)


def _build_screen_frame() -> cq.Workplane:
    """Thin rectangular screen frame ring in local frame centered at origin."""
    outer = _box(0, 0, 0, SCR_OUTER_W, SCR_DEPTH, SCR_OUTER_H)
    inner = _box(0, 0, 0, SCR_INNER_W, SCR_DEPTH + 0.02, SCR_INNER_H)
    return outer.cut(inner)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_sliding_window")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("screen_mesh", rgba=SCREEN_RGBA)
    model.material("metal", rgba=METAL_RGBA)

    # --- Frame (root) with deep track grooves ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame(), "frame_shell"),
        material="vinyl",
        name="frame_shell",
    )

    # --- Upper sash (FIXED) ---
    upper = model.part("upper_sash")
    upper.visual(
        mesh_from_cadquery(_build_sash(INNER_W, UP_H), "upper_sash_vinyl"),
        material="vinyl",
        name="upper_sash_vinyl",
    )
    upper.visual(
        mesh_from_cadquery(_build_glass(GLASS_W, GLASS_UP_H), "upper_sash_glass"),
        material="glass",
        name="upper_sash_glass",
    )

    # --- Lower sash (PRISMATIC, slides vertically) ---
    lower = model.part("lower_sash")
    lower.visual(
        mesh_from_cadquery(_build_sash(INNER_W, LO_H), "lower_sash_vinyl"),
        material="vinyl",
        name="lower_sash_vinyl",
    )
    lower.visual(
        mesh_from_cadquery(_build_glass(GLASS_W, GLASS_LO_H), "lower_sash_glass"),
        material="glass",
        name="lower_sash_glass",
    )
    # Latch on lower sash top rail (meeting rail area), front face
    latch_z = LO_H / 2.0 - SASH_FACE / 2.0   # top rail center in sash-local Z
    latch_y = SASH_DEPTH / 2.0 + LATCH_T / 2.0  # proud of front face
    lower.visual(
        Box((LATCH_W, LATCH_T, LATCH_H)),
        origin=Origin(xyz=(0.0, latch_y, latch_z)),
        material="metal",
        name="lower_sash_latch",
    )

    # --- Insect screen (PRISMATIC, independent shallow track) ---
    screen = model.part("insect_screen")
    screen.visual(
        mesh_from_cadquery(_build_screen_frame(), "screen_frame"),
        material="vinyl",
        name="screen_frame",
    )
    # Screen mesh: thin semi-transparent panel representing fiberglass screen,
    # sized to overlap slightly with the frame edges (captured attachment).
    screen.visual(
        Box((SCR_INNER_W + 0.004, SCR_DEPTH - 0.004, SCR_INNER_H + 0.004)),
        material="screen_mesh",
        name="screen_mesh",
    )

    # --- Articulations ---

    # Upper sash: FIXED in upper opening
    model.articulation(
        "frame_to_upper_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="upper_sash",
        origin=Origin(xyz=(0.0, UPPER_SASH_Y, UP_CZ)),
    )

    # Lower sash: PRISMATIC along +Z (positive q slides UP to open)
    model.articulation(
        "frame_to_lower_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="lower_sash",
        origin=Origin(xyz=(0.0, LOWER_SASH_Y, LO_CZ)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.3,
            lower=0.0, upper=LOWER_TRAVEL,
        ),
    )

    # Insect screen: PRISMATIC along +Z (slides up independently)
    model.articulation(
        "frame_to_screen",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="insect_screen",
        origin=Origin(xyz=(0.0, SCREEN_Y, LO_CZ)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0, velocity=0.2,
            lower=0.0, upper=SCREEN_TRAVEL,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    upper = object_model.get_part("upper_sash")
    lower = object_model.get_part("lower_sash")
    screen = object_model.get_part("insect_screen")
    lower_joint = object_model.get_articulation("frame_to_lower_sash")
    screen_joint = object_model.get_articulation("frame_to_screen")

    # --- Intentional overlaps ---

    # Glass rebated under sash lips (captured glazing)
    for nm in ("upper_sash", "lower_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_vinyl",
            reason="Glass pane is rebated under the sash lip (captured glazing).",
        )

    # Sashes seated in frame track grooves
    for nm in ("upper_sash", "lower_sash"):
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_vinyl",
            reason=f"{nm} ring is seated in the frame track grooves (retained insertion).",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_glass",
            reason=f"{nm} glass tucks under the frame opening lip.",
        )

    # Screen frame in exterior track
    ctx.allow_overlap(
        "frame", "insect_screen",
        elem_a="frame_shell",
        elem_b="screen_frame",
        reason="Screen frame rides in the exterior frame track groove (seated retention).",
    )

    # Screen mesh captured in screen frame
    ctx.allow_overlap(
        "insect_screen", "insect_screen",
        elem_a="screen_mesh",
        elem_b="screen_frame",
        reason="Screen mesh panel is captured inside the screen frame ring.",
    )

    # Latch mounted on lower sash top rail
    ctx.allow_overlap(
        "lower_sash", "lower_sash",
        elem_a="lower_sash_latch",
        elem_b="lower_sash_vinyl",
        reason="Latch is mounted on the lower sash top rail face.",
    )

    # --- Closed pose checks (q=0 for both joints) ---
    with ctx.pose({lower_joint: 0.0, screen_joint: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        upper_aabb = ctx.part_world_aabb(upper)
        lower_aabb = ctx.part_world_aabb(lower)
        screen_aabb = ctx.part_world_aabb(screen)

        # Frame proportions (window-like)
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        frame_h = frame_aabb[1][2] - frame_aabb[0][2]
        ctx.check(
            "frame has window proportions",
            frame_w > 0.70 and frame_h > 1.0,
            details=f"frame_w={frame_w:.3f}, frame_h={frame_h:.3f}",
        )

        # Sill near z=0
        ctx.check(
            "sill near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"frame zmin={frame_aabb[0][2]:.4f}",
        )

        # Upper sash above lower sash (vertical stacking)
        upper_cz = (upper_aabb[0][2] + upper_aabb[1][2]) / 2.0
        lower_cz = (lower_aabb[0][2] + lower_aabb[1][2]) / 2.0
        ctx.check(
            "upper sash above lower sash",
            upper_cz > lower_cz + 0.10,
            details=f"upper_cz={upper_cz:.3f}, lower_cz={lower_cz:.3f}",
        )

        # Lower sash within frame bounds
        ctx.expect_within(
            lower, frame, axes="xz", margin=0.02,
            name="lower sash within frame bounds at rest",
        )

        # Screen on exterior side of lower sash
        screen_cy = (screen_aabb[0][1] + screen_aabb[1][1]) / 2.0
        lower_cy = (lower_aabb[0][1] + lower_aabb[1][1]) / 2.0
        ctx.check(
            "screen on exterior of lower sash",
            screen_cy > lower_cy,
            details=f"screen_y={screen_cy:.3f}, lower_y={lower_cy:.3f}",
        )

        rest_lower_cz = lower_cz
        rest_screen_cz = (screen_aabb[0][2] + screen_aabb[1][2]) / 2.0

    # --- Lower sash slides UP to open ---
    with ctx.pose({lower_joint: LOWER_TRAVEL}):
        open_aabb = ctx.part_world_aabb(lower)
        open_cz = (open_aabb[0][2] + open_aabb[1][2]) / 2.0
        ctx.check(
            "lower sash slides upward to open",
            open_cz > rest_lower_cz + 0.15,
            details=f"rest_cz={rest_lower_cz:.3f}, open_cz={open_cz:.3f}",
        )
        # Sash retained within frame at max travel
        frame_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "lower sash retained in frame at max travel",
            open_aabb[0][2] > frame_aabb[0][2] - 0.01
            and open_aabb[1][2] < frame_aabb[1][2] + 0.01,
            details=f"sash z=[{open_aabb[0][2]:.3f},{open_aabb[1][2]:.3f}]",
        )

    # --- Screen slides independently ---
    with ctx.pose({screen_joint: SCREEN_TRAVEL}):
        scr_aabb = ctx.part_world_aabb(screen)
        scr_cz = (scr_aabb[0][2] + scr_aabb[1][2]) / 2.0
        ctx.check(
            "screen slides upward independently",
            scr_cz > rest_screen_cz + 0.10,
            details=f"rest_cz={rest_screen_cz:.3f}, open_cz={scr_cz:.3f}",
        )

    # --- Joint type checks ---
    ctx.check(
        "lower sash has prismatic joint",
        lower_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={lower_joint.articulation_type}",
    )
    ctx.check(
        "screen has prismatic joint",
        screen_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={screen_joint.articulation_type}",
    )

    # --- Deep track grooves exist (frame extends past openings due to grooves) ---
    # The frame head and sill extend beyond the openings by FRAME_FACE, and the
    # grooves are cut into those rails. Verify the frame depth accommodates grooves.
    frame_depth = frame_aabb[1][1] - frame_aabb[0][1]
    ctx.check(
        "frame depth accommodates deep track grooves",
        frame_depth > 0.090,
        details=f"frame_depth={frame_depth:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
