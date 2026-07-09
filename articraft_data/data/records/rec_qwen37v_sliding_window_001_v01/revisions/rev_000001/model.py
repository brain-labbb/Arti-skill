from __future__ import annotations

"""Two-panel horizontal sliding window with white vinyl frame, colonial
divided-lite grilles, deep track grooves in head/sill rails, and rubber
gasket strips around glass panes. Left sash slides sideways on a prismatic
joint along the head/sill track.

Coordinate convention:
  +Z up, window stands vertically
  width  -> X
  height -> Z (sill near z=0)
  depth  -> Y (glass plane is X-Z)

Structure:
  frame (root): outer vinyl frame with deep track grooves in head/sill
  right_lite: fixed sash (rear track) with grille, glass, gasket
  left_sash: sliding sash (front track) with grille, glass, gasket
"""

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

# ── Dimensions (meters) ─────────────────────────────────────────────────────

TOTAL_W = 1.50
TOTAL_H = 1.50

FRAME_FACE = 0.075        # frame member face width (head/sill/jambs)
FRAME_DEPTH = 0.120       # frame depth along Y

INNER_W = TOTAL_W - 2 * FRAME_FACE   # 1.350
LEFT_LITE_W = INNER_W * 0.50         # 0.675
RIGHT_LITE_W = INNER_W - LEFT_LITE_W # 0.675

SASH_FACE = 0.045         # sash rail/stile face width
SASH_DEPTH = 0.050        # sash depth along Y
GLASS_T = 0.008           # glass thickness

# Track grooves (deep channels in head and sill for sash guidance)
TRACK_GROOVE_DEPTH = 0.025
TRACK_GROOVE_WIDTH = 0.025

# Colonial grille (divided lite)
GRILLE_COLS = 3
GRILLE_ROWS = 4
MUNTIN_T = 0.018
MUNTIN_DEPTH = 0.018

# Y layout (depth): fixed lite in rear track, slider in front track
FIXED_LITE_Y = -0.025     # right fixed lite (rear track)
SLIDE_SASH_Y = 0.040      # left sliding sash (front track, proud)

# Rubber gasket strips
GASKET_W = 0.008          # gasket visible strip width
GASKET_T = 0.005          # gasket thickness

REBATE = 0.005            # glass tucks under sash lip

# ── Materials ───────────────────────────────────────────────────────────────

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)
GLASS_RGBA = (0.50, 0.58, 0.64, 0.32)
RUBBER_RGBA = (0.12, 0.12, 0.13, 1.0)

# ── Derived layout ──────────────────────────────────────────────────────────

HALF_W = TOTAL_W / 2.0
INNER_X0 = -HALF_W + FRAME_FACE
INNER_X1 = HALF_W - FRAME_FACE
INNER_Z0 = FRAME_FACE
INNER_Z1 = TOTAL_H - FRAME_FACE

LEFT_X0 = INNER_X0
LEFT_X1 = LEFT_X0 + LEFT_LITE_W
RIGHT_X0 = LEFT_X1
RIGHT_X1 = INNER_X1


# ── Geometry helpers ────────────────────────────────────────────────────────

def _slab(x0: float, x1: float, z0: float, z1: float,
          y_center: float, depth: float) -> cq.Workplane:
    """Axis-aligned box in X-Z plane, centered on y_center."""
    w = x1 - x0
    h = z1 - z0
    return (
        cq.Workplane("XY")
        .transformed(offset=((x0 + x1) / 2.0, y_center, (z0 + z1) / 2.0))
        .box(w, depth, h)
    )


def _build_frame_shape() -> cq.Workplane:
    """Outer frame with deep track grooves cut into head and sill."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)

    # Main opening (single large opening for two panels, no mullion)
    cut_d = FRAME_DEPTH + 0.02
    frame = outer.cut(_slab(INNER_X0, INNER_X1, INNER_Z0, INNER_Z1, 0.0, cut_d))

    # Track grooves: two parallel channels in sill and head
    for ty in (FIXED_LITE_Y, SLIDE_SASH_Y):
        # Sill groove (cut downward from sill top surface)
        frame = frame.cut(_slab(
            INNER_X0, INNER_X1,
            INNER_Z0 - TRACK_GROOVE_DEPTH, INNER_Z0 + 0.002,
            ty, TRACK_GROOVE_WIDTH,
        ))
        # Head groove (cut upward from head bottom surface)
        frame = frame.cut(_slab(
            INNER_X0, INNER_X1,
            INNER_Z1 - 0.002, INNER_Z1 + TRACK_GROOVE_DEPTH,
            ty, TRACK_GROOVE_WIDTH,
        ))

    return frame


def _build_sash_grille_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Sash frame ring + colonial muntin grille, in sash-local frame."""
    ow = opening_w
    oh = opening_h
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE

    outer = _slab(-out_w / 2, out_w / 2, -out_h / 2, out_h / 2, 0.0, SASH_DEPTH)
    hole = _slab(-ow / 2, ow / 2, -oh / 2, oh / 2, 0.0, SASH_DEPTH + 0.02)
    ring = outer.cut(hole)

    bars = None
    for c in range(1, GRILLE_COLS):
        x = -ow / 2 + (c / GRILLE_COLS) * ow
        bar = _slab(x - MUNTIN_T / 2, x + MUNTIN_T / 2,
                    -oh / 2, oh / 2, 0.0, MUNTIN_DEPTH)
        bars = bar if bars is None else bars.union(bar)
    for r in range(1, GRILLE_ROWS):
        z = -oh / 2 + (r / GRILLE_ROWS) * oh
        bar = _slab(-ow / 2, ow / 2,
                    z - MUNTIN_T / 2, z + MUNTIN_T / 2,
                    0.0, MUNTIN_DEPTH)
        bars = bar if bars is None else bars.union(bar)

    return ring if bars is None else ring.union(bars)


def _build_glass_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Glass pane in sash-local frame (extends REBATE under sash lip)."""
    ow = opening_w + 2 * REBATE
    oh = opening_h + 2 * REBATE
    return _slab(-ow / 2, ow / 2, -oh / 2, oh / 2, 0.0, GLASS_T)


def _build_gasket_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Rubber gasket ring around glass perimeter in sash-local frame.
    Sits at the interface between the sash frame and the glass pane."""
    ow = opening_w
    oh = opening_h
    iw = ow - 2 * GASKET_W
    ih = oh - 2 * GASKET_W

    outer = _slab(-ow / 2, ow / 2, -oh / 2, oh / 2, 0.0, GASKET_T)
    inner_cut = _slab(-iw / 2, iw / 2, -ih / 2, ih / 2, 0.0, GASKET_T + 0.002)
    return outer.cut(inner_cut)


# ── Model ───────────────────────────────────────────────────────────────────

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_panel_sliding_window")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("rubber", rgba=RUBBER_RGBA)

    # ── Frame (root) with track grooves ──
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="vinyl",
        name="frame_shell",
    )

    opening_h = INNER_Z1 - INNER_Z0

    # ── Right fixed lite (rear track) ──
    rl = model.part("right_lite")
    rl.visual(
        mesh_from_cadquery(_build_sash_grille_shape(RIGHT_LITE_W, opening_h), "right_vinyl"),
        material="vinyl", name="right_vinyl",
    )
    rl.visual(
        mesh_from_cadquery(_build_glass_shape(RIGHT_LITE_W, opening_h), "right_glass"),
        material="glass", name="right_glass",
    )
    rl.visual(
        mesh_from_cadquery(_build_gasket_shape(RIGHT_LITE_W, opening_h), "right_gasket"),
        material="rubber", name="right_gasket",
    )

    # ── Left sliding sash (front track) ──
    ls = model.part("left_sash")
    ls.visual(
        mesh_from_cadquery(_build_sash_grille_shape(LEFT_LITE_W, opening_h), "left_vinyl"),
        material="vinyl", name="left_vinyl",
    )
    ls.visual(
        mesh_from_cadquery(_build_glass_shape(LEFT_LITE_W, opening_h), "left_glass"),
        material="glass", name="left_glass",
    )
    ls.visual(
        mesh_from_cadquery(_build_gasket_shape(LEFT_LITE_W, opening_h), "left_gasket"),
        material="rubber", name="left_gasket",
    )

    # ── Articulations ──
    right_cx = (RIGHT_X0 + RIGHT_X1) / 2.0
    left_cx = (LEFT_X0 + LEFT_X1) / 2.0
    mid_cz = (INNER_Z0 + INNER_Z1) / 2.0

    # Fixed right lite seated in rear track
    model.articulation(
        "frame_to_right_lite",
        ArticulationType.FIXED,
        parent="frame",
        child="right_lite",
        origin=Origin(xyz=(right_cx, FIXED_LITE_Y, mid_cz)),
    )

    # Left sliding sash: prismatic along +X
    # Positive q slides sash rightward, exposing the left side of the opening
    slide_travel = LEFT_LITE_W * 0.90
    model.articulation(
        "frame_to_left_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="left_sash",
        origin=Origin(xyz=(left_cx, SLIDE_SASH_Y, mid_cz)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.5,
            lower=0.0, upper=slide_travel,
        ),
    )

    return model


# ── Tests ───────────────────────────────────────────────────────────────────

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    right_lite = object_model.get_part("right_lite")
    left_sash = object_model.get_part("left_sash")
    slide = object_model.get_articulation("frame_to_left_sash")

    # ── Intentional overlap allowances ──

    # Glass rebated under sash lip (captured glazing) — within each sash part
    for part_nm, glass_nm, vinyl_nm in (
        ("right_lite", "right_glass", "right_vinyl"),
        ("left_sash", "left_glass", "left_vinyl"),
    ):
        ctx.allow_overlap(
            part_nm, part_nm,
            elem_a=glass_nm, elem_b=vinyl_nm,
            reason="Glass pane is rebated under the sash frame lip (captured glazing).",
        )

    # Gasket wraps glass perimeter — within each sash part
    for part_nm, glass_nm, gasket_nm in (
        ("right_lite", "right_glass", "right_gasket"),
        ("left_sash", "left_glass", "left_gasket"),
    ):
        ctx.allow_overlap(
            part_nm, part_nm,
            elem_a=glass_nm, elem_b=gasket_nm,
            reason="Rubber gasket wraps the glass perimeter; intentional seated capture.",
        )

    # Gasket at perimeter may overlap muntin bar crossings — within each sash
    for part_nm, gasket_nm, vinyl_nm in (
        ("right_lite", "right_gasket", "right_vinyl"),
        ("left_sash", "left_gasket", "left_vinyl"),
    ):
        ctx.allow_overlap(
            part_nm, part_nm,
            elem_a=gasket_nm, elem_b=vinyl_nm,
            reason="Gasket strip sits at the sash perimeter where muntin bars cross; seated rubber seal.",
        )

    # Fixed right lite sash seated in frame opening (rear track capture)
    ctx.allow_overlap(
        "frame", "right_lite",
        elem_a="frame_shell", elem_b="right_vinyl",
        reason="Right fixed lite sash is rebated into the frame opening; seated capture in rear track.",
    )
    ctx.allow_overlap(
        "frame", "right_lite",
        elem_a="frame_shell", elem_b="right_glass",
        reason="Right lite glass is rebated under the frame opening lip (captured glazing).",
    )

    # Left sliding sash rides in front track and laps frame along the track
    ctx.allow_overlap(
        "frame", "left_sash",
        elem_a="frame_shell", elem_b="left_vinyl",
        reason="Left sash rides the head/sill track and laps the frame face along the track (slider capture).",
    )
    ctx.allow_overlap(
        "frame", "left_sash",
        elem_a="frame_shell", elem_b="left_glass",
        reason="Left sash glass laps the head/sill track lip as the proud sash rides the track.",
    )
    ctx.allow_overlap(
        "frame", "left_sash",
        elem_a="frame_shell", elem_b="left_gasket",
        reason="Left sash gasket sits within the frame opening at the track interface (seated rubber seal).",
    )

    # Right fixed lite gasket also sits within the frame opening
    ctx.allow_overlap(
        "frame", "right_lite",
        elem_a="frame_shell", elem_b="right_gasket",
        reason="Right lite gasket sits within the frame opening at the rear track (seated rubber seal).",
    )

    # ── Prompt-specific checks ──

    # Two-panel structure: exactly two sash parts plus frame
    ctx.check(
        "two-panel structure exists",
        right_lite is not None and left_sash is not None and frame is not None,
        details="frame, right_lite, left_sash must all exist",
    )

    # Gasket visuals exist on each sash
    for part_nm, gasket_nm in (("right_lite", "right_gasket"), ("left_sash", "left_gasket")):
        p = object_model.get_part(part_nm)
        gasket_vis = p.get_visual(gasket_nm) if p else None
        ctx.check(
            f"{part_nm} has rubber gasket strip",
            gasket_vis is not None,
            details=f"expected gasket visual '{gasket_nm}' on part '{part_nm}'",
        )

    # Prismatic joint exists and is non-fixed
    ctx.check(
        "left sash has prismatic joint",
        slide is not None and slide.articulation_type == ArticulationType.PRISMATIC,
        details="frame_to_left_sash must be PRISMATIC",
    )
    ctx.check(
        "prismatic joint has positive travel",
        slide.motion_limits.upper > 0.05,
        details=f"upper={slide.motion_limits.upper:.3f}",
    )

    # ── Closed pose (q=0): window reads SHUT ──
    with ctx.pose({slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        l_aabb = ctx.part_world_aabb(left_sash)
        r_aabb = ctx.part_world_aabb(right_lite)

        # Frame spans full window
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        ctx.check(
            "frame spans full width",
            abs(frame_w - TOTAL_W) < 0.02,
            details=f"frame_w={frame_w:.3f}, expected={TOTAL_W:.3f}",
        )
        ctx.check(
            "sill near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"frame zmin={frame_aabb[0][2]:.4f}",
        )
        ctx.check(
            "head reaches full height",
            abs(frame_aabb[1][2] - TOTAL_H) < 0.02,
            details=f"frame zmax={frame_aabb[1][2]:.4f}",
        )

        # Two panels ordered left-right
        lx = (l_aabb[0][0] + l_aabb[1][0]) / 2.0
        rx = (r_aabb[0][0] + r_aabb[1][0]) / 2.0
        ctx.check(
            "panels ordered left-right",
            lx < rx,
            details=f"left_x={lx:.3f}, right_x={rx:.3f}",
        )

        # Both sashes seated within frame height
        for nm, ab in (("left_sash", l_aabb), ("right_lite", r_aabb)):
            ctx.check(
                f"{nm} within frame height",
                ab[0][2] > frame_aabb[0][2] - 0.01 and ab[1][2] < frame_aabb[1][2] + 0.01,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}]",
            )

        # Left sash sits proud of right lite (front track vs rear track)
        l_y = (l_aabb[0][1] + l_aabb[1][1]) / 2.0
        r_y = (r_aabb[0][1] + r_aabb[1][1]) / 2.0
        ctx.check(
            "left sash proud of right lite (front track)",
            l_y > r_y + 0.02,
            details=f"left_y={l_y:.3f}, right_y={r_y:.3f}",
        )

        # Fixed lite seated in frame opening (projected overlap)
        ctx.expect_overlap(
            right_lite, frame, axes="xz", min_overlap=0.03,
            name="right fixed lite seated in frame opening",
        )

        rest_lx = lx
        rest_lz = (l_aabb[0][2] + l_aabb[1][2]) / 2.0

    # ── Open pose: left sash slides right ──
    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel}):
        l_open = ctx.part_world_aabb(left_sash)
        open_lx = (l_open[0][0] + l_open[1][0]) / 2.0

        # Sash translated along +X by ~travel
        ctx.check(
            "left sash slides along +X",
            abs((open_lx - rest_lx) - travel) < 0.02,
            details=f"rest_lx={rest_lx:.3f}, open_lx={open_lx:.3f}, travel={travel:.3f}",
        )

        # Slide is purely horizontal (no Z drift)
        open_lz = (l_open[0][2] + l_open[1][2]) / 2.0
        ctx.check(
            "slide is purely horizontal",
            abs(open_lz - rest_lz) < 0.02,
            details=f"open_z={open_lz:.3f}, rest_z={rest_lz:.3f}",
        )

        # Sash retained within frame at full travel
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained within frame X span",
            l_open[1][0] < f_aabb[1][0] + 1e-4 and l_open[0][0] > f_aabb[0][0] - 1e-4,
            details=f"sash x=[{l_open[0][0]:.3f},{l_open[1][0]:.3f}] frame x=[{f_aabb[0][0]:.3f},{f_aabb[1][0]:.3f}]",
        )
        ctx.expect_overlap(
            left_sash, frame, axes="z", min_overlap=0.10,
            name="sash retains vertical engagement with head/sill track",
        )

    return ctx.report()


object_model = build_object_model()
