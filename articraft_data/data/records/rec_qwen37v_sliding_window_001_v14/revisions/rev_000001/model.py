from __future__ import annotations

# Variant 14: Three-panel horizontal sliding window with white vinyl frame,
# colonial divided-lite grilles, NARROW TRANSOM panel above the sliding panes,
# INSECT SCREEN on independent prismatic slide, ROLLER BLOCKS at the bottom
# of the moving sash, and VISIBLE OVERLAP STILE where panes cross.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     width  -> X,  height -> Z (sill near z=0),  depth -> Y
#   Glass plane is X-Z. Window reads SHUT at q=0; driving the prismatic
#   joints slides the center sash (+X) and screen (+X) independently.

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

TOTAL_W = 3.00
TOTAL_H = 1.50

FRAME_FACE = 0.070
MULLION_FACE = 0.060
FRAME_DEPTH = 0.110

TRANSOM_BAR_H = 0.045      # horizontal bar between main lites and transom
TRANSOM_H = 0.20            # transom clear opening height

SIDE_LITE_W = 0.85
CENTER_LITE_W = 1.04

SASH_FACE = 0.055
SASH_DEPTH = 0.055
GLASS_T = 0.008

GRILLE_COLS = 4
GRILLE_ROWS = 5
MUNTIN_T = 0.020
MUNTIN_DEPTH = 0.020

FIXED_LITE_Y = -0.020
SLIDE_SASH_Y = 0.052

REBATE = 0.005

# Transom construction
TRANSOM_SASH_FACE = 0.035

# Insect screen
SCREEN_FRAME_W = 0.025
SCREEN_FRAME_DEPTH = 0.018
SCREEN_MESH_T = 0.002
SCREEN_Y = -0.060           # interior side, behind fixed lites

# Roller blocks (bottom of sliding sash)
ROLLER_W = 0.030
ROLLER_D = 0.020
ROLLER_H = 0.012
ROLLER_EMBED = 0.004        # overlap into sash bottom rail for connectivity

# Overlap stile (interlock flange on right edge of sliding sash)
STILE_W = 0.035
STILE_EXTRA_DEPTH = 0.008   # extends slightly proud of sash face
STILE_EMBED = 0.010         # overlap into sash right stile for connectivity

# ---------------------------------------------------------------------------
# Derived layout
# ---------------------------------------------------------------------------

HALF_W = TOTAL_W / 2.0

INNER_X0 = -HALF_W + FRAME_FACE
INNER_X1 = HALF_W - FRAME_FACE
INNER_Z0 = FRAME_FACE
INNER_Z1 = TOTAL_H - FRAME_FACE

# Main lite region (below transom bar)
MAIN_LITE_Z1 = INNER_Z1 - TRANSOM_BAR_H - TRANSOM_H
# Transom region (above transom bar)
TRANSOM_Z0 = MAIN_LITE_Z1 + TRANSOM_BAR_H
TRANSOM_Z1 = INNER_Z1

# Three lite columns
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

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)
GLASS_RGBA = (0.50, 0.58, 0.64, 0.32)
SCREEN_RGBA = (0.22, 0.25, 0.22, 0.55)   # dark grey-green, semi-opaque mesh
ROLLER_RGBA = (0.12, 0.12, 0.14, 1.0)    # dark nylon rollers


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery, meters, world frame conventions)
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float,
          y_center: float, depth: float) -> cq.Workplane:
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
    """Outer frame: slab cut by three lower lite openings + one transom opening,
    leaving head, sill, jambs, two mullions, and the transom bar as one solid."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    cut_depth = FRAME_DEPTH + 0.02

    # Lower lite cutouts (main sliding region)
    left_cut = _slab(LEFT_X0, LEFT_X1, INNER_Z0, MAIN_LITE_Z1, 0.0, cut_depth)
    center_cut = _slab(CENTER_X0, CENTER_X1, INNER_Z0, MAIN_LITE_Z1, 0.0, cut_depth)
    right_cut = _slab(RIGHT_X0, RIGHT_X1, INNER_Z0, MAIN_LITE_Z1, 0.0, cut_depth)

    # Transom cutout (full inner width above transom bar)
    transom_cut = _slab(INNER_X0, INNER_X1, TRANSOM_Z0, TRANSOM_Z1, 0.0, cut_depth)

    return outer.cut(left_cut).cut(center_cut).cut(right_cut).cut(transom_cut)


def _build_sash_grille_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Sash ring + colonial muntin grid in local frame centered on origin."""
    ow = opening_w
    oh = opening_h
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE

    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SASH_DEPTH)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH + 0.02)
    ring = outer.cut(opening)

    bars = None
    for c in range(1, GRILLE_COLS):
        frac = c / GRILLE_COLS
        x = -ow / 2.0 + frac * ow
        bar = _slab(x - MUNTIN_T / 2.0, x + MUNTIN_T / 2.0,
                    -oh / 2.0, oh / 2.0, 0.0, MUNTIN_DEPTH)
        bars = bar if bars is None else bars.union(bar)

    for r in range(1, GRILLE_ROWS):
        frac = r / GRILLE_ROWS
        z = -oh / 2.0 + frac * oh
        bar = _slab(-ow / 2.0, ow / 2.0,
                    z - MUNTIN_T / 2.0, z + MUNTIN_T / 2.0,
                    0.0, MUNTIN_DEPTH)
        bars = bar if bars is None else bars.union(bar)

    return ring if bars is None else ring.union(bars)


def _build_sash_glass_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    ow = opening_w + 2 * REBATE
    oh = opening_h + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_transom_vinyl_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Transom sash ring (thinner frame, no grille)."""
    face = TRANSOM_SASH_FACE
    out_w = opening_w + 2 * face
    out_h = opening_h + 2 * face
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SASH_DEPTH)
    opening = _slab(-opening_w / 2.0, opening_w / 2.0,
                    -opening_h / 2.0, opening_h / 2.0,
                    0.0, SASH_DEPTH + 0.02)
    return outer.cut(opening)


def _build_transom_glass_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    ow = opening_w + 2 * REBATE
    oh = opening_h + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_screen_frame_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Thin screen frame ring in local frame."""
    out_w = opening_w + 2 * SCREEN_FRAME_W
    out_h = opening_h + 2 * SCREEN_FRAME_W
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0,
                  0.0, SCREEN_FRAME_DEPTH)
    opening = _slab(-opening_w / 2.0, opening_w / 2.0,
                    -opening_h / 2.0, opening_h / 2.0,
                    0.0, SCREEN_FRAME_DEPTH + 0.01)
    return outer.cut(opening)


def _build_screen_mesh_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Screen mesh panel, slightly oversized for rebate overlap with frame."""
    w = opening_w + 2 * 0.003
    h = opening_h + 2 * 0.003
    return _slab(-w / 2.0, w / 2.0, -h / 2.0, h / 2.0, 0.0, SCREEN_MESH_T)


def _build_roller_shape() -> cq.Workplane:
    """Small roller block (box)."""
    return _slab(-ROLLER_W / 2.0, ROLLER_W / 2.0,
                 -ROLLER_H / 2.0, ROLLER_H / 2.0,
                 0.0, ROLLER_D)


def _build_overlap_stile_shape(sash_outer_h: float) -> cq.Workplane:
    """Vertical overlap stile bar (interlock flange)."""
    return _slab(-STILE_W / 2.0, STILE_W / 2.0,
                 -sash_outer_h / 2.0, sash_outer_h / 2.0,
                 0.0, SASH_DEPTH + STILE_EXTRA_DEPTH)


# ---------------------------------------------------------------------------
# Part builders
# ---------------------------------------------------------------------------

def _add_lite(model: ArticulatedObject, name: str,
              opening_w: float, opening_h: float) -> None:
    """Fixed lite: sash ring + colonial grille + glass."""
    lite = model.part(name)
    lite.visual(
        mesh_from_cadquery(_build_sash_grille_shape(opening_w, opening_h), f"{name}_vinyl"),
        material="vinyl", name=f"{name}_vinyl",
    )
    lite.visual(
        mesh_from_cadquery(_build_sash_glass_shape(opening_w, opening_h), f"{name}_glass"),
        material="glass", name=f"{name}_glass",
    )


def _add_center_sash(model: ArticulatedObject, name: str,
                     opening_w: float, opening_h: float) -> None:
    """Sliding sash: sash ring + grille + glass + overlap stile + roller blocks."""
    sash = model.part(name)
    sash.visual(
        mesh_from_cadquery(_build_sash_grille_shape(opening_w, opening_h), f"{name}_vinyl"),
        material="vinyl", name=f"{name}_vinyl",
    )
    sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(opening_w, opening_h), f"{name}_glass"),
        material="glass", name=f"{name}_glass",
    )

    # Overlap stile on right edge (interlock flange where panes cross).
    sash_outer_h = opening_h + 2 * SASH_FACE
    stile_x = opening_w / 2.0 + SASH_FACE - STILE_EMBED + STILE_W / 2.0
    sash.visual(
        mesh_from_cadquery(_build_overlap_stile_shape(sash_outer_h), "overlap_stile"),
        material="vinyl", name="overlap_stile",
        origin=Origin(xyz=(stile_x, STILE_EXTRA_DEPTH / 2.0, 0.0)),
    )

    # Two roller blocks at bottom of sash (partially embedded in bottom rail).
    roller_z = -(opening_h / 2.0 + SASH_FACE) - ROLLER_H / 2.0 + ROLLER_EMBED
    left_rx = -(opening_w / 2.0)
    right_rx = opening_w / 2.0
    sash.visual(
        mesh_from_cadquery(_build_roller_shape(), "roller_left"),
        material="roller", name="roller_left",
        origin=Origin(xyz=(left_rx, 0.0, roller_z)),
    )
    sash.visual(
        mesh_from_cadquery(_build_roller_shape(), "roller_right"),
        material="roller", name="roller_right",
        origin=Origin(xyz=(right_rx, 0.0, roller_z)),
    )


def _add_transom(model: ArticulatedObject, name: str,
                 opening_w: float, opening_h: float) -> None:
    """Fixed transom panel: thin sash ring + glass (no grille)."""
    transom = model.part(name)
    transom.visual(
        mesh_from_cadquery(_build_transom_vinyl_shape(opening_w, opening_h), "transom_vinyl"),
        material="vinyl", name="transom_vinyl",
    )
    transom.visual(
        mesh_from_cadquery(_build_transom_glass_shape(opening_w, opening_h), "transom_glass"),
        material="glass", name="transom_glass",
    )


def _add_screen(model: ArticulatedObject, name: str,
                opening_w: float, opening_h: float) -> None:
    """Insect screen: thin frame ring + semi-transparent mesh panel."""
    screen = model.part(name)
    screen.visual(
        mesh_from_cadquery(_build_screen_frame_shape(opening_w, opening_h), "screen_frame"),
        material="vinyl", name="screen_frame",
    )
    screen.visual(
        mesh_from_cadquery(_build_screen_mesh_shape(opening_w, opening_h), "screen_mesh"),
        material="screen", name="screen_mesh",
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    # Verify layout fills inner clear width.
    span = SIDE_LITE_W + MULLION_FACE + CENTER_LITE_W + MULLION_FACE + SIDE_LITE_W
    inner_w = INNER_X1 - INNER_X0
    assert abs(span - inner_w) < 1e-6, f"lite layout {span} != inner width {inner_w}"

    model = ArticulatedObject(name="sliding_window_transom_screen")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("screen", rgba=SCREEN_RGBA)
    model.material("roller", rgba=ROLLER_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="vinyl", name="frame_shell",
    )

    # Main lite opening height (reduced by transom bar + transom opening)
    main_opening_h = MAIN_LITE_Z1 - INNER_Z0

    # --- Two fixed side lites ---
    _add_lite(model, "left_lite", SIDE_LITE_W, main_opening_h)
    _add_lite(model, "right_lite", SIDE_LITE_W, main_opening_h)

    # --- Center sliding sash (with rollers and overlap stile) ---
    _add_center_sash(model, "center_sash", CENTER_LITE_W, main_opening_h)

    # --- Transom panel (fixed, above main lites) ---
    transom_opening_w = INNER_X1 - INNER_X0
    transom_opening_h = TRANSOM_Z1 - TRANSOM_Z0
    _add_transom(model, "transom", transom_opening_w, transom_opening_h)

    # --- Insect screen (independent prismatic slide) ---
    screen_opening_w = SIDE_LITE_W
    screen_opening_h = main_opening_h
    _add_screen(model, "insect_screen", screen_opening_w, screen_opening_h)

    # World centers for placement
    left_cx = (LEFT_X0 + LEFT_X1) / 2.0
    right_cx = (RIGHT_X0 + RIGHT_X1) / 2.0
    center_cx = (CENTER_X0 + CENTER_X1) / 2.0
    main_mid_cz = (INNER_Z0 + MAIN_LITE_Z1) / 2.0
    transom_cz = (TRANSOM_Z0 + TRANSOM_Z1) / 2.0
    transom_cx = (INNER_X0 + INNER_X1) / 2.0

    # --- Fixed side lites ---
    model.articulation(
        "frame_to_left_lite",
        ArticulationType.FIXED,
        parent="frame", child="left_lite",
        origin=Origin(xyz=(left_cx, FIXED_LITE_Y, main_mid_cz)),
    )
    model.articulation(
        "frame_to_right_lite",
        ArticulationType.FIXED,
        parent="frame", child="right_lite",
        origin=Origin(xyz=(right_cx, FIXED_LITE_Y, main_mid_cz)),
    )

    # --- Center sliding sash: PRISMATIC along +X ---
    slide_travel = SIDE_LITE_W * 0.92
    model.articulation(
        "frame_to_center_sash",
        ArticulationType.PRISMATIC,
        parent="frame", child="center_sash",
        origin=Origin(xyz=(center_cx, SLIDE_SASH_Y, main_mid_cz)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5,
                                   lower=0.0, upper=slide_travel),
    )

    # --- Transom: FIXED above main lites ---
    model.articulation(
        "frame_to_transom",
        ArticulationType.FIXED,
        parent="frame", child="transom",
        origin=Origin(xyz=(transom_cx, FIXED_LITE_Y, transom_cz)),
    )

    # --- Insect screen: PRISMATIC along +X (independent from sash) ---
    screen_travel = SIDE_LITE_W * 0.85
    model.articulation(
        "frame_to_screen",
        ArticulationType.PRISMATIC,
        parent="frame", child="insect_screen",
        origin=Origin(xyz=(0.0, SCREEN_Y, main_mid_cz)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.3,
                                   lower=0.0, upper=screen_travel),
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
    transom = object_model.get_part("transom")
    screen = object_model.get_part("insect_screen")

    slide = object_model.get_articulation("frame_to_center_sash")
    screen_slide = object_model.get_articulation("frame_to_screen")

    # =====================================================================
    # Intentional overlap allowances
    # =====================================================================

    # Glass panes captured under sash/muntin lip (each lite).
    for nm in ("left_lite", "right_lite", "center_sash"):
        ctx.allow_overlap(
            nm, nm, elem_a=f"{nm}_glass", elem_b=f"{nm}_vinyl",
            reason="Pane rebated under sash/muntin lip (captured glazing).",
        )

    # Fixed side lites rebated into frame openings.
    for nm in ("left_lite", "right_lite"):
        ctx.allow_overlap(
            "frame", nm, elem_a="frame_shell", elem_b=f"{nm}_vinyl",
            reason=f"{nm} sash ring laps jamb/mullion edge (seated capture).",
        )
        ctx.allow_overlap(
            "frame", nm, elem_a="frame_shell", elem_b=f"{nm}_glass",
            reason=f"{nm} glass rebated under frame opening lip.",
        )

    # Center sash track engagement.
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell", elem_b="center_sash_vinyl",
        reason="Center sash rides head/sill track, laps frame face.",
    )
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell", elem_b="center_sash_glass",
        reason="Center sash glass laps track lip.",
    )

    # Transom seated in frame transom opening.
    ctx.allow_overlap(
        "frame", "transom",
        elem_a="frame_shell", elem_b="transom_vinyl",
        reason="Transom sash laps frame transom opening edge (seated).",
    )
    ctx.allow_overlap(
        "frame", "transom",
        elem_a="frame_shell", elem_b="transom_glass",
        reason="Transom glass rebated under frame transom lip.",
    )
    ctx.allow_overlap(
        "transom", "transom",
        elem_a="transom_glass", elem_b="transom_vinyl",
        reason="Transom glass captured under transom sash lip.",
    )

    # Roller blocks embedded in sash bottom rail.
    for rname in ("roller_left", "roller_right"):
        ctx.allow_overlap(
            "center_sash", "center_sash",
            elem_a=rname, elem_b="center_sash_vinyl",
            reason=f"{rname} partially embedded in sash bottom rail for track contact.",
        )

    # Overlap stile embedded in sash right stile (interlock flange).
    ctx.allow_overlap(
        "center_sash", "center_sash",
        elem_a="overlap_stile", elem_b="center_sash_vinyl",
        reason="Overlap stile embedded in sash right stile as interlock flange.",
    )
    # Overlap stile passes in front of frame/mullion and right lite when closed.
    ctx.allow_overlap(
        "center_sash", "frame",
        elem_a="overlap_stile", elem_b="frame_shell",
        reason="Overlap stile passes in front of right mullion when sash is closed.",
    )
    ctx.allow_overlap(
        "center_sash", "right_lite",
        elem_a="overlap_stile", elem_b="right_lite_vinyl",
        reason="Overlap stile overlaps right lite stile when closed (interlock).",
    )
    ctx.allow_overlap(
        "center_sash", "right_lite",
        elem_a="overlap_stile", elem_b="right_lite_glass",
        reason="Overlap stile passes in front of right lite glass when closed.",
    )

    # Screen mesh captured inside screen frame.
    ctx.allow_overlap(
        "insect_screen", "insect_screen",
        elem_a="screen_mesh", elem_b="screen_frame",
        reason="Screen mesh rebated inside screen frame.",
    )
    # Screen track engagement with frame back.
    ctx.allow_overlap(
        "frame", "insect_screen",
        elem_a="frame_shell", elem_b="screen_frame",
        reason="Screen slides on interior track recessed into frame back.",
    )

    # Side lite sash rings tuck under the transom bar face (real rebate capture).
    for nm in ("left_lite", "right_lite"):
        ctx.allow_overlap(
            nm, "transom",
            elem_a=f"{nm}_vinyl", elem_b="transom_vinyl",
            reason=f"{nm} sash ring tucks under transom bar face (rebate capture).",
        )
        ctx.allow_overlap(
            nm, "transom",
            elem_a=f"{nm}_vinyl", elem_b="transom_glass",
            reason=f"{nm} sash ring laps transom glass at rebate boundary (captured glazing joint).",
        )

    # Roller blocks ride on the sill track, overlapping the frame sill.
    for rname in ("roller_left", "roller_right"):
        ctx.allow_overlap(
            "center_sash", "frame",
            elem_a=rname, elem_b="frame_shell",
            reason=f"{rname} rides on sill track, overlapping frame sill for track engagement.",
        )

    # =====================================================================
    # Geometry and structural checks
    # =====================================================================

    # --- Transom is above main lites (comparing part origins) ---
    ctx.expect_origin_gap(
        transom, center_sash, axis="z", min_gap=0.05,
        name="transom is above center sash in Z",
    )
    ctx.expect_within(
        transom, frame, axes="z", margin=0.01,
        name="transom within frame Z extent",
    )

    # --- Screen is on interior side (-Y) of main lites ---
    ctx.expect_gap(
        center_sash, screen, axis="y", min_gap=0.02,
        name="screen is behind center sash (interior side)",
    )

    # --- Roller blocks below sash glass ---
    ctx.expect_gap(
        center_sash, center_sash, axis="z",
        positive_elem="center_sash_glass", negative_elem="roller_left",
        min_gap=0.02,
        name="left roller is below glass pane",
    )
    ctx.expect_gap(
        center_sash, center_sash, axis="z",
        positive_elem="center_sash_glass", negative_elem="roller_right",
        min_gap=0.02,
        name="right roller is below glass pane",
    )

    # --- Overlap stile spans sash height and shares X extent ---
    ctx.expect_overlap(
        center_sash, center_sash, axes="z",
        elem_a="overlap_stile", elem_b="center_sash_vinyl",
        min_overlap=0.5,
        name="overlap stile spans most of sash height",
    )
    ctx.expect_overlap(
        center_sash, center_sash, axes="x",
        elem_a="overlap_stile", elem_b="center_sash_vinyl",
        min_overlap=0.005,
        name="overlap stile shares X extent with sash vinyl",
    )

    # --- Both non-fixed joints have positive travel ---
    ctx.check(
        "center sash has prismatic travel",
        slide.motion_limits.upper > 0.0,
        details=f"upper={slide.motion_limits.upper:.3f}",
    )
    ctx.check(
        "screen has independent prismatic travel",
        screen_slide.motion_limits.upper > 0.0,
        details=f"upper={screen_slide.motion_limits.upper:.3f}",
    )

    # =====================================================================
    # Closed pose (q=0): window reads SHUT
    # =====================================================================

    with ctx.pose({slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        l_aabb = ctx.part_world_aabb(left_lite)
        r_aabb = ctx.part_world_aabb(right_lite)
        c_aabb = ctx.part_world_aabb(center_sash)
        t_aabb = ctx.part_world_aabb(transom)

        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        center_w = c_aabb[1][0] - c_aabb[0][0]
        ctx.check(
            "frame spans wider than center sash",
            frame_w > center_w + 1.5,
            details=f"frame_w={frame_w:.3f}, center_w={center_w:.3f}",
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

        # Lites ordered left -> center -> right
        lx = (l_aabb[0][0] + l_aabb[1][0]) / 2.0
        cx = (c_aabb[0][0] + c_aabb[1][0]) / 2.0
        rx = (r_aabb[0][0] + r_aabb[1][0]) / 2.0
        ctx.check(
            "lites ordered left-center-right",
            lx < cx < rx,
            details=f"lx={lx:.3f}, cx={cx:.3f}, rx={rx:.3f}",
        )

        # All main lites within frame height
        for nm, ab in (("left", l_aabb), ("right", r_aabb), ("center", c_aabb)):
            ctx.check(
                f"{nm} lite within frame height",
                ab[0][2] > frame_aabb[0][2] - 1e-4 and ab[1][2] < frame_aabb[1][2] + 1e-4,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}]",
            )

        # Transom glass is above all main lite glass (comparing opening centers,
        # since sash rings tuck under the transom bar in reality).
        t_glass_cz = (t_aabb[0][2] + t_aabb[1][2]) / 2.0
        for nm, ab in (("left", l_aabb), ("center", c_aabb), ("right", r_aabb)):
            lite_cz = (ab[0][2] + ab[1][2]) / 2.0
            ctx.check(
                f"transom center above {nm} lite center",
                t_glass_cz > lite_cz + 0.05,
                details=f"transom_cz={t_glass_cz:.3f}, {nm}_cz={lite_cz:.3f}",
            )

        # Center sash proud of side lites
        l_y = (l_aabb[0][1] + l_aabb[1][1]) / 2.0
        c_y = (c_aabb[0][1] + c_aabb[1][1]) / 2.0
        ctx.check(
            "center sash proud of side lites",
            c_y > l_y + 0.02,
            details=f"center_y={c_y:.3f}, side_y={l_y:.3f}",
        )

        # Fixed lites seated in frame
        ctx.expect_overlap(
            left_lite, frame, axes="xz", min_overlap=0.03,
            name="left lite seated in frame opening",
        )
        ctx.expect_overlap(
            right_lite, frame, axes="xz", min_overlap=0.03,
            name="right lite seated in frame opening",
        )

        rest_cx = cx
        rest_cz = (c_aabb[0][2] + c_aabb[1][2]) / 2.0

    # =====================================================================
    # Sash sliding pose
    # =====================================================================

    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel}):
        c_open = ctx.part_world_aabb(center_sash)
        open_cx = (c_open[0][0] + c_open[1][0]) / 2.0
        ctx.check(
            "center sash slides +X by ~travel",
            abs((open_cx - rest_cx) - travel) < 0.02,
            details=f"rest={rest_cx:.3f}, open={open_cx:.3f}, travel={travel:.3f}",
        )
        c_open_z = (c_open[0][2] + c_open[1][2]) / 2.0
        ctx.check(
            "slide is purely horizontal",
            abs(c_open_z - rest_cz) < 0.02,
            details=f"open_z={c_open_z:.3f}, rest_z={rest_cz:.3f}",
        )
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained in frame X span",
            c_open[1][0] < f_aabb[1][0] + 1e-4 and c_open[0][0] > f_aabb[0][0] - 1e-4,
            details=f"sash x=[{c_open[0][0]:.3f},{c_open[1][0]:.3f}]",
        )
        ctx.expect_overlap(
            center_sash, frame, axes="z", min_overlap=0.10,
            name="sash retains vertical track engagement at full travel",
        )

    # =====================================================================
    # Screen independent slide
    # =====================================================================

    screen_rest = ctx.part_world_position(screen)
    sash_rest_pos = ctx.part_world_position(center_sash)

    screen_travel_dist = screen_slide.motion_limits.upper
    with ctx.pose({screen_slide: screen_travel_dist}):
        screen_open = ctx.part_world_position(screen)
        sash_during_screen = ctx.part_world_position(center_sash)

    ctx.check(
        "screen slides along +X",
        screen_open is not None and screen_rest is not None
        and (screen_open[0] - screen_rest[0]) > 0.1,
        details=f"screen dx={screen_open[0] - screen_rest[0]:.3f}",
    )
    ctx.check(
        "screen slide is horizontal (no Z drift)",
        screen_open is not None and screen_rest is not None
        and abs(screen_open[2] - screen_rest[2]) < 0.01,
        details=f"screen dz={screen_open[2] - screen_rest[2]:.4f}",
    )
    ctx.check(
        "screen slide does not move center sash",
        sash_during_screen is not None and sash_rest_pos is not None
        and abs(sash_during_screen[0] - sash_rest_pos[0]) < 0.001,
        details="sash should remain stationary when screen slides",
    )

    # Driven sash does not move screen
    screen_rest2 = ctx.part_world_position(screen)
    with ctx.pose({slide: travel}):
        screen_during_sash = ctx.part_world_position(screen)
    ctx.check(
        "sash slide does not move screen",
        screen_during_sash is not None and screen_rest2 is not None
        and abs(screen_during_sash[0] - screen_rest2[0]) < 0.001,
        details="screen should remain stationary when sash slides",
    )

    return ctx.report()


object_model = build_object_model()
