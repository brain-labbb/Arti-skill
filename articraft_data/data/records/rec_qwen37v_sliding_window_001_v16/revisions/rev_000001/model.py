from __future__ import annotations

# Two-panel vertical sliding window (single-hung variant). Slim white vinyl
# frame with bevelled (chamfered) outer corners, colonial divided-lite grilles.
# Upper lite is FIXED; lower sash slides UPWARD on a vertical prismatic joint.
# Two tiny roller blocks at the bottom of the moving sash. Sill lip with
# drainage slots (weep holes) as real geometry.
#
# Coordinate convention:
#   +Z up, sill near z=0, window stands vertically in the X-Z plane.
#   +Y toward the interior (lower sash track); -Y toward exterior (upper lite).
#   Width -> X, height -> Z, depth -> Y.

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
TOTAL_W = 1.20              # overall window width
TOTAL_H = 1.50              # overall window height
FRAME_FACE = 0.050          # slim frame rail/stile face width
FRAME_DEPTH = 0.090         # frame depth along Y
MEETING_RAIL_H = 0.050     # horizontal meeting rail height
CHAMFER = 0.008             # bevelled corner chamfer size

CLEAR_W = TOTAL_W - 2 * FRAME_FACE  # 1.10 clear opening width

# Vertical layout: sill | lower opening | meeting rail | upper opening | head
LOWER_Z0 = FRAME_FACE                               # 0.050
MEETING_Z0 = (TOTAL_H - MEETING_RAIL_H) / 2.0      # 0.725
MEETING_Z1 = MEETING_Z0 + MEETING_RAIL_H            # 0.775
UPPER_Z1 = TOTAL_H - FRAME_FACE                     # 1.45
LOWER_Z1 = MEETING_Z0                               # 0.725
UPPER_Z0 = MEETING_Z1                               # 0.775

UPPER_H = UPPER_Z1 - UPPER_Z0                       # 0.675
LOWER_H = LOWER_Z1 - LOWER_Z0                       # 0.675

# Sash construction
SASH_FACE = 0.040           # sash rail/stile face width
SASH_DEPTH = 0.040          # sash profile depth along Y
GLASS_T = 0.006             # glass thickness
REBATE = 0.004              # glass tucks under sash lip

# Colonial grille
GRILLE_COLS = 3
GRILLE_ROWS = 4
MUNTIN_T = 0.015
MUNTIN_DEPTH = 0.018

# Y layout: upper lite on exterior track, lower sash on interior track
UPPER_Y = -0.025
LOWER_Y = +0.025

# Sill lip and drainage
SILL_LIP_DEPTH = 0.035
SILL_LIP_T = 0.015
SLOT_W = 0.025
NUM_SLOTS = 3

# Roller blocks (at bottom of moving sash)
ROLLER_W = 0.025
ROLLER_D = 0.020
ROLLER_H = 0.015
ROLLER_EMBED = 0.003        # recess into sash bottom rail

# Travel
TRAVEL = 0.40               # vertical slide travel

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------
VINYL_RGBA = (0.93, 0.94, 0.95, 1.0)
GLASS_RGBA = (0.50, 0.58, 0.64, 0.32)
ROLLER_RGBA = (0.22, 0.22, 0.25, 1.0)   # dark grey nylon


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float,
          y_center: float, depth: float) -> cq.Workplane:
    """Axis-aligned box spanning [x0,x1] x [z0,z1] in X-Z, centered on y_center."""
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
    """Outer frame: slim box with chamfered vertical edges, two lite openings
    (upper and lower), plus a sill lip with drainage slots."""
    # 1. Outer box, bottom at z=0, centered on X and Y.
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, TOTAL_H / 2.0))
        .box(TOTAL_W, FRAME_DEPTH, TOTAL_H)
    )
    # 2. Bevelled corners: chamfer the 4 outer vertical edges.
    outer = outer.edges("|Z").chamfer(CHAMFER)

    # 3. Cut upper opening.
    upper_cut = _slab(-CLEAR_W / 2.0, CLEAR_W / 2.0,
                      UPPER_Z0, UPPER_Z1, 0.0, FRAME_DEPTH + 0.02)
    outer = outer.cut(upper_cut)

    # 4. Cut lower opening.
    lower_cut = _slab(-CLEAR_W / 2.0, CLEAR_W / 2.0,
                      LOWER_Z0, LOWER_Z1, 0.0, FRAME_DEPTH + 0.02)
    outer = outer.cut(lower_cut)

    # 5. Sill lip: extends outward (+Y) from the front face at the sill.
    lip_y_center = FRAME_DEPTH / 2.0 + SILL_LIP_DEPTH / 2.0
    sill_lip = _slab(-TOTAL_W / 2.0, TOTAL_W / 2.0,
                     0.0, SILL_LIP_T,
                     lip_y_center, SILL_LIP_DEPTH)

    # 6. Drainage slots (weep holes) cut through the sill lip.
    slot_spacing = TOTAL_W / (NUM_SLOTS + 1)
    for i in range(NUM_SLOTS):
        sx = -TOTAL_W / 2.0 + (i + 1) * slot_spacing
        slot = _slab(sx - SLOT_W / 2.0, sx + SLOT_W / 2.0,
                     -0.005, SILL_LIP_T + 0.005,
                     lip_y_center, SILL_LIP_DEPTH + 0.02)
        sill_lip = sill_lip.cut(slot)

    outer = outer.union(sill_lip)
    return outer


def _build_sash_vinyl(sash_w: float, sash_h: float) -> cq.Workplane:
    """Sash frame (rails + stiles) with colonial muntin grille, in local frame
    centered on origin. Returns the vinyl workplane."""
    glass_w = sash_w - 2.0 * SASH_FACE
    glass_h = sash_h - 2.0 * SASH_FACE

    # Outer sash slab.
    outer = _slab(-sash_w / 2.0, sash_w / 2.0,
                  -sash_h / 2.0, sash_h / 2.0, 0.0, SASH_DEPTH)
    # Hollow: cut the glass opening.
    opening = _slab(-glass_w / 2.0, glass_w / 2.0,
                    -glass_h / 2.0, glass_h / 2.0, 0.0, SASH_DEPTH + 0.02)
    ring = outer.cut(opening)

    # Colonial muntin grid.
    bars = None
    for c in range(1, GRILLE_COLS):
        x = -glass_w / 2.0 + (c / GRILLE_COLS) * glass_w
        bar = _slab(x - MUNTIN_T / 2.0, x + MUNTIN_T / 2.0,
                    -glass_h / 2.0, glass_h / 2.0, 0.0, MUNTIN_DEPTH)
        bars = bar if bars is None else bars.union(bar)
    for r in range(1, GRILLE_ROWS):
        z = -glass_h / 2.0 + (r / GRILLE_ROWS) * glass_h
        bar = _slab(-glass_w / 2.0, glass_w / 2.0,
                    z - MUNTIN_T / 2.0, z + MUNTIN_T / 2.0,
                    0.0, MUNTIN_DEPTH)
        bars = bar if bars is None else bars.union(bar)

    return ring if bars is None else ring.union(bars)


def _build_sash_glass(sash_w: float, sash_h: float) -> cq.Workplane:
    """Single glass pane rebated under the sash lip, in sash-local frame."""
    glass_w = sash_w - 2.0 * SASH_FACE + 2.0 * REBATE
    glass_h = sash_h - 2.0 * SASH_FACE + 2.0 * REBATE
    return _slab(-glass_w / 2.0, glass_w / 2.0,
                 -glass_h / 2.0, glass_h / 2.0, 0.0, GLASS_T)


def _build_rollers(sash_w: float, sash_h: float) -> cq.Workplane:
    """Two small roller blocks at the bottom of the sash, in sash-local frame.
    Each roller is recessed slightly into the bottom rail for connectivity."""
    roller_z_center = (-sash_h / 2.0 - ROLLER_H / 2.0 + ROLLER_EMBED)
    x_offset = sash_w / 2.0 - ROLLER_W / 2.0 - 0.025

    left = (
        cq.Workplane("XY")
        .transformed(offset=(-x_offset, 0.0, roller_z_center))
        .box(ROLLER_W, ROLLER_D, ROLLER_H)
    )
    right = (
        cq.Workplane("XY")
        .transformed(offset=(x_offset, 0.0, roller_z_center))
        .box(ROLLER_W, ROLLER_D, ROLLER_H)
    )
    return left.union(right)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_sliding_window")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("roller", rgba=ROLLER_RGBA)

    # --- Frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="vinyl",
        name="frame_shell",
    )

    # Sash dimensions (both lites use the same clear width)
    sash_w = CLEAR_W

    # --- Upper lite (FIXED) ---
    upper = model.part("upper_lite")
    upper.visual(
        mesh_from_cadquery(_build_sash_vinyl(sash_w, UPPER_H), "upper_vinyl"),
        material="vinyl",
        name="upper_lite_vinyl",
    )
    upper.visual(
        mesh_from_cadquery(_build_sash_glass(sash_w, UPPER_H), "upper_glass"),
        material="glass",
        name="upper_lite_glass",
    )

    # --- Lower sash (SLIDING) ---
    lower = model.part("lower_sash")
    lower.visual(
        mesh_from_cadquery(_build_sash_vinyl(sash_w, LOWER_H), "lower_vinyl"),
        material="vinyl",
        name="lower_sash_vinyl",
    )
    lower.visual(
        mesh_from_cadquery(_build_sash_glass(sash_w, LOWER_H), "lower_glass"),
        material="glass",
        name="lower_sash_glass",
    )
    lower.visual(
        mesh_from_cadquery(_build_rollers(sash_w, LOWER_H), "rollers"),
        material="roller",
        name="rollers",
    )

    # Centers for placement
    upper_cz = (UPPER_Z0 + UPPER_Z1) / 2.0   # 1.1125
    lower_cz = (LOWER_Z0 + LOWER_Z1) / 2.0   # 0.3875

    # FIXED upper lite in exterior track
    model.articulation(
        "frame_to_upper_lite",
        ArticulationType.FIXED,
        parent="frame",
        child="upper_lite",
        origin=Origin(xyz=(0.0, UPPER_Y, upper_cz)),
    )

    # PRISMATIC lower sash: slides upward (+Z) on interior track
    model.articulation(
        "frame_to_lower_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="lower_sash",
        origin=Origin(xyz=(0.0, LOWER_Y, lower_cz)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0, velocity=0.4,
            lower=0.0, upper=TRAVEL,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    upper_lite = object_model.get_part("upper_lite")
    lower_sash = object_model.get_part("lower_sash")
    slide = object_model.get_articulation("frame_to_lower_sash")

    # --- Intentional overlaps ---

    # Glass rebated under sash/muntin lip on each lite.
    for nm in ("upper_lite", "lower_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass" if nm == "upper_lite" else f"{nm.replace('sash', 'sash')}_glass",
            elem_b=f"{nm}_vinyl" if nm == "upper_lite" else f"{nm.replace('sash', 'sash')}_vinyl",
            reason="Glass pane rebated under sash/muntin lip (captured glazing).",
        )
    # Simpler element names:
    ctx.allow_overlap(
        "upper_lite", "upper_lite",
        elem_a="upper_lite_glass", elem_b="upper_lite_vinyl",
        reason="Upper lite glass rebated under sash/muntin lip.",
    )
    ctx.allow_overlap(
        "lower_sash", "lower_sash",
        elem_a="lower_sash_glass", elem_b="lower_sash_vinyl",
        reason="Lower sash glass rebated under sash/muntin lip.",
    )

    # Roller blocks recessed into sash bottom rail.
    ctx.allow_overlap(
        "lower_sash", "lower_sash",
        elem_a="rollers", elem_b="lower_sash_vinyl",
        reason="Roller blocks recessed into the sash bottom rail for mounting.",
    )

    # Upper lite seated in frame opening (sash laps frame rebate).
    ctx.allow_overlap(
        "frame", "upper_lite",
        elem_a="frame_shell", elem_b="upper_lite_vinyl",
        reason="Upper lite sash is seated in the frame opening rebate.",
    )
    ctx.allow_overlap(
        "frame", "upper_lite",
        elem_a="frame_shell", elem_b="upper_lite_glass",
        reason="Upper lite glass extends into frame rebate zone.",
    )

    # Lower sash rides in frame track (sash laps jamb track edges).
    ctx.allow_overlap(
        "frame", "lower_sash",
        elem_a="frame_shell", elem_b="lower_sash_vinyl",
        reason="Lower sash rides the jamb track; stiles lap the frame track edges.",
    )
    ctx.allow_overlap(
        "frame", "lower_sash",
        elem_a="frame_shell", elem_b="lower_sash_glass",
        reason="Lower sash glass laps the frame track lip.",
    )

    # Rollers protrude below sash into sill track (roller in track representation).
    ctx.allow_overlap(
        "frame", "lower_sash",
        elem_a="frame_shell", elem_b="rollers",
        reason="Roller blocks protrude into the sill track recess (roller-in-track).",
    )

    # --- Joint type and mechanism checks ---

    ctx.check(
        "slide joint is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )

    # --- Rest pose (q=0): window reads closed ---
    with ctx.pose({slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        upper_aabb = ctx.part_world_aabb(upper_lite)
        lower_aabb = ctx.part_world_aabb(lower_sash)

        # Frame spans the full window dimensions.
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        frame_h = frame_aabb[1][2] - frame_aabb[0][2]
        ctx.check(
            "frame width matches design",
            abs(frame_w - TOTAL_W) < 0.02,
            details=f"frame_w={frame_w:.3f}",
        )
        ctx.check(
            "frame height matches design",
            abs(frame_h - TOTAL_H) < 0.02,
            details=f"frame_h={frame_h:.3f}",
        )
        ctx.check(
            "sill near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"frame zmin={frame_aabb[0][2]:.4f}",
        )

        # Sill lip extends beyond the main frame depth in +Y.
        ctx.check(
            "sill lip extends beyond frame front",
            frame_aabb[1][1] > FRAME_DEPTH / 2.0 + 0.010,
            details=f"frame ymax={frame_aabb[1][1]:.4f}, expected>{FRAME_DEPTH / 2.0 + 0.010:.4f}",
        )

        # Upper lite above lower sash.
        upper_cz = (upper_aabb[0][2] + upper_aabb[1][2]) / 2.0
        lower_cz = (lower_aabb[0][2] + lower_aabb[1][2]) / 2.0
        ctx.check(
            "upper lite above lower sash",
            upper_cz > lower_cz + 0.10,
            details=f"upper_cz={upper_cz:.3f}, lower_cz={lower_cz:.3f}",
        )

        # Both lites within frame height.
        for nm, ab in (("upper", upper_aabb), ("lower", lower_aabb)):
            ctx.check(
                f"{nm} lite within frame height",
                ab[0][2] > frame_aabb[0][2] - 0.005 and ab[1][2] < frame_aabb[1][2] + 0.005,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}]",
            )

        # Lower sash on interior track (proud in +Y relative to upper lite).
        upper_y = (upper_aabb[0][1] + upper_aabb[1][1]) / 2.0
        lower_y = (lower_aabb[0][1] + lower_aabb[1][1]) / 2.0
        ctx.check(
            "lower sash on interior track (proud in +Y)",
            lower_y > upper_y + 0.01,
            details=f"lower_y={lower_y:.3f}, upper_y={upper_y:.3f}",
        )

        # Roller blocks near sash bottom.
        roller_aabb = ctx.part_element_world_aabb(lower_sash, elem="rollers")
        ctx.check(
            "rollers exist and are near sash bottom",
            roller_aabb is not None and roller_aabb[0][2] < lower_aabb[0][2] + 0.020,
            details=f"roller_zmin={roller_aabb[0][2]:.4f}, sash_zmin={lower_aabb[0][2]:.4f}" if roller_aabb else "no rollers",
        )

        rest_lower_z = lower_cz

    # --- Driven pose: lower sash slides upward ---
    with ctx.pose({slide: TRAVEL}):
        raised_aabb = ctx.part_world_aabb(lower_sash)
        raised_cz = (raised_aabb[0][2] + raised_aabb[1][2]) / 2.0

        ctx.check(
            "lower sash slides upward by travel",
            abs((raised_cz - rest_lower_z) - TRAVEL) < 0.02,
            details=f"rest_z={rest_lower_z:.3f}, raised_z={raised_cz:.3f}, travel={TRAVEL:.3f}",
        )

        # Sash does not move horizontally.
        rest_lower_x = 0.0  # sash is centered
        raised_x = (raised_aabb[0][0] + raised_aabb[1][0]) / 2.0
        ctx.check(
            "slide is purely vertical (no X drift)",
            abs(raised_x - rest_lower_x) < 0.02,
            details=f"raised_x={raised_x:.3f}",
        )

        # Sash retained within frame at full travel.
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained within frame height at full travel",
            raised_aabb[1][2] < f_aabb[1][2] + 0.01,
            details=f"sash zmax={raised_aabb[1][2]:.3f}, frame zmax={f_aabb[1][2]:.3f}",
        )

        ctx.expect_overlap(
            lower_sash, frame,
            axes="x",
            min_overlap=0.50,
            name="sash retains horizontal engagement with frame at full travel",
        )

    return ctx.report()


object_model = build_object_model()
