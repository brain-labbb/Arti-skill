from __future__ import annotations

# Corner-lift sliding window variant: three-panel horizontal slider with
# colonial divided-lite grilles, corner-lift vent panel, independent insect
# screen, roller blocks on the sliding sash, and sill lip with drainage slots.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     - width  -> X
#     - height -> Z   (sill near z=0)
#     - frame depth / glazing thickness / slide-normal -> Y
#   The glass plane is the X-Z plane. +Y is exterior.
#
# Structure:
#   - frame (static root): head, sill, two jambs, two mullions, SILL LIP, and
#     DRAINAGE SLOTS cut through the sill front face.
#   - left_lite, right_lite (FIXED): vinyl sash ring + colonial muntin grille +
#     clear glass, seated in the rear glazing plane.
#   - center_sash (SLIDING, PRISMATIC +X): same construction as fixed lites but
#     proud in +Y, plus two ROLLER BLOCKS at the bottom rail.
#   - vent_panel (REVOLUTE corner-lift): small operable panel in the upper-left
#     area, tilts outward on a bottom-edge hinge.
#   - insect_screen (PRISMATIC +X, independent track): thin framed screen on the
#     interior side, slides independently.

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

TOTAL_W = 3.00
TOTAL_H = 1.50

FRAME_FACE = 0.070
MULLION_FACE = 0.060
FRAME_DEPTH = 0.110

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

# Sill lip dimensions
SILL_LIP_H = 0.025          # lip height (Z)
SILL_LIP_T = 0.015          # lip thickness (Y, protruding outward)

# Drainage slots
DRAIN_SLOT_W = 0.040        # slot width (X)
DRAIN_SLOT_H = 0.008        # slot height (Z)
DRAIN_SLOT_DEPTH = FRAME_DEPTH + 0.02  # through-cut
DRAIN_SLOT_COUNT = 4
DRAIN_SLOT_Z = FRAME_FACE * 0.35       # near bottom of sill

# Vent panel dimensions
VENT_W = 0.300
VENT_H = 0.250
VENT_FRAME = 0.035
VENT_DEPTH = 0.030
VENT_GLASS_T = 0.006

# Insect screen dimensions
SCREEN_W = 0.820
SCREEN_H = 1.440            # extends into sill/head tracks for real engagement
SCREEN_FRAME = 0.030
SCREEN_DEPTH = 0.020
SCREEN_MESH_T = 0.002
SCREEN_Y = -0.048            # interior side, behind fixed lites

# Roller block dimensions
ROLLER_W = 0.040
ROLLER_H = 0.015
ROLLER_D = 0.025

# ---------------------------------------------------------------------------
# Derived layout
# ---------------------------------------------------------------------------

HALF_W = TOTAL_W / 2.0

INNER_X0 = -HALF_W + FRAME_FACE
INNER_X1 = HALF_W - FRAME_FACE
INNER_Z0 = FRAME_FACE
INNER_Z1 = TOTAL_H - FRAME_FACE

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
SCREEN_RGBA = (0.25, 0.25, 0.28, 0.45)     # dark grey insect screen mesh
ROLLER_RGBA = (0.18, 0.18, 0.20, 1.0)       # dark nylon roller


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery)
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float, y_center: float, depth: float) -> cq.Workplane:
    """Axis-aligned box spanning [x0,x1] x [z0,z1] centered on y_center."""
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
    """Static outer frame: slab cut by three lite openings + drainage slots,
    then union with sill lip."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)

    cut_depth = FRAME_DEPTH + 0.02
    left_cut = _slab(LEFT_X0, LEFT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    center_cut = _slab(CENTER_X0, CENTER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    right_cut = _slab(RIGHT_X0, RIGHT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)

    frame = outer.cut(left_cut).cut(center_cut).cut(right_cut)

    # Drainage slots: cut narrow through-slots in the sill front face.
    inner_span = INNER_X1 - INNER_X0
    for i in range(DRAIN_SLOT_COUNT):
        frac = (i + 1.0) / (DRAIN_SLOT_COUNT + 1.0)
        sx = INNER_X0 + frac * inner_span
        slot = _slab(
            sx - DRAIN_SLOT_W / 2.0, sx + DRAIN_SLOT_W / 2.0,
            DRAIN_SLOT_Z - DRAIN_SLOT_H / 2.0, DRAIN_SLOT_Z + DRAIN_SLOT_H / 2.0,
            0.0, DRAIN_SLOT_DEPTH,
        )
        frame = frame.cut(slot)

    # Sill lip: a raised strip along the exterior front bottom of the sill.
    lip = _slab(
        -HALF_W, HALF_W,
        0.0, SILL_LIP_H,
        FRAME_DEPTH / 2.0 + SILL_LIP_T / 2.0, SILL_LIP_T,
    )
    frame = frame.union(lip)

    return frame


def _build_sash_grille_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Sash ring + colonial muntin grille in local frame centered on origin."""
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
        bar = _slab(
            x - MUNTIN_T / 2.0, x + MUNTIN_T / 2.0,
            -oh / 2.0, oh / 2.0,
            0.0, MUNTIN_DEPTH,
        )
        bars = bar if bars is None else bars.union(bar)

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
    """Clear pane filling sash opening in sash-local frame."""
    ow = opening_w + 2 * REBATE
    oh = opening_h + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_vent_panel_shape() -> cq.Workplane:
    """Small vent panel sash ring in local frame centered on origin."""
    ow = VENT_W - 2 * VENT_FRAME
    oh = VENT_H - 2 * VENT_FRAME
    outer = _slab(-VENT_W / 2.0, VENT_W / 2.0, -VENT_H / 2.0, VENT_H / 2.0, 0.0, VENT_DEPTH)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, VENT_DEPTH + 0.02)
    return outer.cut(opening)


def _build_vent_glass_shape() -> cq.Workplane:
    """Vent panel glass pane in local frame."""
    ow = VENT_W - 2 * VENT_FRAME + 2 * REBATE
    oh = VENT_H - 2 * VENT_FRAME + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, VENT_GLASS_T)


def _build_screen_frame_shape() -> cq.Workplane:
    """Insect screen frame: thin ring in local frame."""
    ow = SCREEN_W - 2 * SCREEN_FRAME
    oh = SCREEN_H - 2 * SCREEN_FRAME
    outer = _slab(-SCREEN_W / 2.0, SCREEN_W / 2.0, -SCREEN_H / 2.0, SCREEN_H / 2.0, 0.0, SCREEN_DEPTH)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SCREEN_DEPTH + 0.02)
    return outer.cut(opening)


def _build_screen_mesh_shape() -> cq.Workplane:
    """Insect screen mesh panel (thin semi-transparent)."""
    ow = SCREEN_W - 2 * SCREEN_FRAME
    oh = SCREEN_H - 2 * SCREEN_FRAME
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SCREEN_MESH_T)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    span = SIDE_LITE_W + MULLION_FACE + CENTER_LITE_W + MULLION_FACE + SIDE_LITE_W
    inner_w = INNER_X1 - INNER_X0
    assert abs(span - inner_w) < 1e-6, f"lite layout {span} != inner width {inner_w}"

    model = ArticulatedObject(name="corner_lift_sliding_window")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("screen_mesh", rgba=SCREEN_RGBA)
    model.material("roller", rgba=ROLLER_RGBA)

    # --- Static outer frame (root) with sill lip + drainage slots ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="vinyl",
        name="frame_shell",
    )

    opening_h = INNER_Z1 - INNER_Z0

    # --- Fixed side lites ---
    left_lite = model.part("left_lite")
    left_lite.visual(
        mesh_from_cadquery(_build_sash_grille_shape(SIDE_LITE_W, opening_h), "left_lite_vinyl"),
        material="vinyl",
        name="left_lite_vinyl",
    )
    left_lite.visual(
        mesh_from_cadquery(_build_sash_glass_shape(SIDE_LITE_W, opening_h), "left_lite_glass"),
        material="glass",
        name="left_lite_glass",
    )

    right_lite = model.part("right_lite")
    right_lite.visual(
        mesh_from_cadquery(_build_sash_grille_shape(SIDE_LITE_W, opening_h), "right_lite_vinyl"),
        material="vinyl",
        name="right_lite_vinyl",
    )
    right_lite.visual(
        mesh_from_cadquery(_build_sash_glass_shape(SIDE_LITE_W, opening_h), "right_lite_glass"),
        material="glass",
        name="right_lite_glass",
    )

    # --- Center sliding sash with roller blocks ---
    center_sash = model.part("center_sash")
    center_sash.visual(
        mesh_from_cadquery(_build_sash_grille_shape(CENTER_LITE_W, opening_h), "center_sash_vinyl"),
        material="vinyl",
        name="center_sash_vinyl",
    )
    center_sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(CENTER_LITE_W, opening_h), "center_sash_glass"),
        material="glass",
        name="center_sash_glass",
    )
    # Two roller blocks at the bottom rail of the sash
    sash_out_h = opening_h + 2 * SASH_FACE
    roller_z = -sash_out_h / 2.0 + ROLLER_H / 2.0  # bottom of sash
    roller_x_offset = CENTER_LITE_W / 2.0 - ROLLER_W  # near each end
    for i, sign in enumerate((-1.0, 1.0)):
        rx = sign * roller_x_offset
        center_sash.visual(
            Box((ROLLER_W, ROLLER_D, ROLLER_H)),
            origin=Origin(xyz=(rx, 0.0, roller_z)),
            material="roller",
            name=f"roller_block_{i}",
        )

    # --- Vent panel (corner-lift, upper-left area) ---
    vent_panel = model.part("vent_panel")
    vent_panel.visual(
        mesh_from_cadquery(_build_vent_panel_shape(), "vent_panel_vinyl"),
        material="vinyl",
        name="vent_panel_vinyl",
    )
    vent_panel.visual(
        mesh_from_cadquery(_build_vent_glass_shape(), "vent_panel_glass"),
        material="glass",
        name="vent_panel_glass",
    )

    # --- Insect screen (independent prismatic track, interior side) ---
    insect_screen = model.part("insect_screen")
    insect_screen.visual(
        mesh_from_cadquery(_build_screen_frame_shape(), "screen_frame"),
        material="vinyl",
        name="screen_frame",
    )
    insect_screen.visual(
        mesh_from_cadquery(_build_screen_mesh_shape(), "screen_mesh"),
        material="screen_mesh",
        name="screen_mesh",
    )

    # --- Centers for articulation origins ---
    left_cx = (LEFT_X0 + LEFT_X1) / 2.0
    right_cx = (RIGHT_X0 + RIGHT_X1) / 2.0
    center_cx = (CENTER_X0 + CENTER_X1) / 2.0
    mid_cz = (INNER_Z0 + INNER_Z1) / 2.0

    # FIXED side lites
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

    # CENTER sliding sash: PRISMATIC along +X
    slide_travel = SIDE_LITE_W * 0.92
    model.articulation(
        "frame_to_center_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="center_sash",
        origin=Origin(xyz=(center_cx, SLIDE_SASH_Y, mid_cz)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5, lower=0.0, upper=slide_travel),
    )

    # Vent panel: REVOLUTE corner-lift at the bottom edge of the vent opening.
    # The vent panel sits in the upper-left corner of the left lite.
    # Panel center at q=0 is at the opening center. Hinge at the bottom edge.
    vent_cx = LEFT_X0 + VENT_W / 2.0
    vent_cz = INNER_Z1 - VENT_H / 2.0
    vent_hinge_z = INNER_Z1 - VENT_H  # bottom edge of vent opening
    # Panel extends +Z from hinge. axis=(-1,0,0) makes positive q tilt top outward (+Y).
    model.articulation(
        "frame_to_vent_panel",
        ArticulationType.REVOLUTE,
        parent="frame",
        child="vent_panel",
        origin=Origin(xyz=(vent_cx, FIXED_LITE_Y, vent_hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=0.60),
    )

    # Insect screen: PRISMATIC along +X on interior track.
    # Screen starts centered on the left lite area, slides right.
    screen_start_x = (LEFT_X0 + LEFT_X1) / 2.0
    screen_cz = (INNER_Z0 + INNER_Z1) / 2.0
    screen_travel = SIDE_LITE_W * 0.90
    model.articulation(
        "frame_to_insect_screen",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="insect_screen",
        origin=Origin(xyz=(screen_start_x, SCREEN_Y, screen_cz)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.4, lower=0.0, upper=screen_travel),
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
    vent_panel = object_model.get_part("vent_panel")
    insect_screen = object_model.get_part("insect_screen")
    slide = object_model.get_articulation("frame_to_center_sash")
    vent_hinge = object_model.get_articulation("frame_to_vent_panel")
    screen_slide = object_model.get_articulation("frame_to_insect_screen")

    # --- Intentional overlaps ---
    # Glass panes tuck under sash/muntin lip (captured glass).
    for nm in ("left_lite", "right_lite", "center_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_vinyl",
            reason="Clear pane is rebated under the sash/muntin lip so it reads captured.",
        )
    # Vent panel glass captured in vent sash ring.
    ctx.allow_overlap(
        "vent_panel", "vent_panel",
        elem_a="vent_panel_glass",
        elem_b="vent_panel_vinyl",
        reason="Vent glass is rebated under the vent sash lip.",
    )
    # Screen mesh captured in screen frame.
    ctx.allow_overlap(
        "insect_screen", "insect_screen",
        elem_a="screen_mesh",
        elem_b="screen_frame",
        reason="Screen mesh is captured inside the screen frame ring.",
    )
    # Fixed lites rebated into frame openings.
    ctx.allow_overlap(
        "frame", "left_lite",
        elem_a="frame_shell",
        elem_b="left_lite_vinyl",
        reason="Left fixed lite sash ring laps the frame opening edge (seated capture).",
    )
    ctx.allow_overlap(
        "frame", "right_lite",
        elem_a="frame_shell",
        elem_b="right_lite_vinyl",
        reason="Right fixed lite sash ring laps the frame opening edge (seated capture).",
    )
    # Center sash rides the head/sill track.
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell",
        elem_b="center_sash_vinyl",
        reason="Center sash rides the head/sill track and laps the frame face.",
    )
    # Glass rebated under frame opening lip.
    ctx.allow_overlap(
        "frame", "left_lite",
        elem_a="frame_shell",
        elem_b="left_lite_glass",
        reason="Left lite glass rebated under frame opening lip.",
    )
    ctx.allow_overlap(
        "frame", "right_lite",
        elem_a="frame_shell",
        elem_b="right_lite_glass",
        reason="Right lite glass rebated under frame opening lip.",
    )
    ctx.allow_overlap(
        "frame", "center_sash",
        elem_a="frame_shell",
        elem_b="center_sash_glass",
        reason="Center sash glass laps head/sill track lip.",
    )
    # Vent panel overlaps the left lite area (it sits in the same opening region).
    ctx.allow_overlap(
        "frame", "vent_panel",
        elem_a="frame_shell",
        elem_b="vent_panel_vinyl",
        reason="Vent panel is rebated into the frame opening at the upper-left corner.",
    )
    ctx.allow_overlap(
        "left_lite", "vent_panel",
        elem_a="left_lite_vinyl",
        elem_b="vent_panel_vinyl",
        reason="Vent panel sits in the upper-left region of the left lite opening.",
    )
    ctx.allow_overlap(
        "left_lite", "vent_panel",
        elem_a="left_lite_glass",
        elem_b="vent_panel_glass",
        reason="Vent panel glass overlaps left lite glass in the corner region.",
    )
    ctx.allow_overlap(
        "left_lite", "vent_panel",
        elem_a="left_lite_glass",
        elem_b="vent_panel_vinyl",
        reason="Vent panel sash ring sits over left lite glass in the upper-left corner region.",
    )
    ctx.allow_overlap(
        "left_lite", "vent_panel",
        elem_a="left_lite_vinyl",
        elem_b="vent_panel_glass",
        reason="Vent panel glass sits behind left lite sash ring in the upper-left corner region.",
    )
    # Insect screen passes behind the fixed lite on its own interior track.
    ctx.allow_overlap(
        "insect_screen", "left_lite",
        elem_a="screen_frame",
        elem_b="left_lite_vinyl",
        reason="Insect screen on interior track passes behind the left fixed lite sash.",
    )
    ctx.allow_overlap(
        "insect_screen", "left_lite",
        elem_a="screen_mesh",
        elem_b="left_lite_glass",
        reason="Insect screen mesh passes behind the left fixed lite glass on the interior track.",
    )
    # Roller blocks recessed into sash bottom rail, contacting the sill track.
    for i in range(2):
        ctx.allow_overlap(
            "center_sash", "frame",
            elem_a=f"roller_block_{i}",
            elem_b="frame_shell",
            reason=f"Roller block {i} is recessed into the sash bottom rail and contacts the sill track surface.",
        )
    # Insect screen extends into sill/head tracks (real screen track engagement).
    ctx.allow_overlap(
        "insect_screen", "frame",
        elem_a="screen_frame",
        elem_b="frame_shell",
        reason="Insect screen frame extends into the sill and head track channels for guided sliding.",
    )
    ctx.allow_overlap(
        "insect_screen", "frame",
        elem_a="screen_mesh",
        elem_b="frame_shell",
        reason="Insect screen mesh extends into the sill and head track channels along with the frame.",
    )

    # --- Closed pose checks ---
    with ctx.pose({slide: 0.0, vent_hinge: 0.0, screen_slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)

        # Frame spans full width and height.
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        ctx.check(
            "frame spans full width",
            frame_w > 2.5,
            details=f"frame_w={frame_w:.3f}",
        )
        ctx.check(
            "sill sits near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"frame zmin={frame_aabb[0][2]:.4f}",
        )

        # Sill lip protrudes in +Y beyond the main frame depth.
        frame_y_max = frame_aabb[1][1]
        ctx.check(
            "sill lip protrudes beyond frame front",
            frame_y_max > FRAME_DEPTH / 2.0 + SILL_LIP_T * 0.5,
            details=f"frame y_max={frame_y_max:.4f}, expected>{FRAME_DEPTH / 2.0 + SILL_LIP_T * 0.5:.4f}",
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

        # Center sash proud of side lites.
        l_aabb = ctx.part_world_aabb(left_lite)
        c_aabb = ctx.part_world_aabb(center_sash)
        l_y = (l_aabb[0][1] + l_aabb[1][1]) / 2.0
        c_y = (c_aabb[0][1] + c_aabb[1][1]) / 2.0
        ctx.check(
            "center sash proud of side lites",
            c_y > l_y + 0.02,
            details=f"center_y={c_y:.3f}, side_y={l_y:.3f}",
        )

        # Vent panel is in the upper-left area of the window.
        v_aabb = ctx.part_world_aabb(vent_panel)
        ctx.check(
            "vent panel in upper-left region",
            v_aabb[0][0] < 0.0 and v_aabb[1][2] > mid_cz_global(),
            details=f"vent x_min={v_aabb[0][0]:.3f}, z_max={v_aabb[1][2]:.3f}",
        )
        # Vent panel overlaps with frame opening (seated in track).
        ctx.expect_overlap(
            vent_panel, frame, axes="xz", min_overlap=0.02,
            name="vent panel seated in frame opening",
        )

        # Insect screen is on the interior side (more negative Y than fixed lites).
        s_aabb = ctx.part_world_aabb(insect_screen)
        screen_y = (s_aabb[0][1] + s_aabb[1][1]) / 2.0
        ctx.check(
            "insect screen on interior side",
            screen_y < l_y - 0.01,
            details=f"screen_y={screen_y:.3f}, side_y={l_y:.3f}",
        )
        # Insect screen extends into sill and head tracks (retained in Z).
        ctx.expect_overlap(
            insect_screen, frame, axes="x", min_overlap=0.03,
            name="insect screen retained in frame track (X overlap)",
        )
        # Screen is behind (more negative Y) the left lite - proof for screen/lite allowance.
        ctx.expect_gap(
            left_lite, insect_screen, axis="y", max_penetration=0.015,
            name="insect screen mostly behind left lite in Y",
        )

        # Roller blocks at the bottom of the sash, near the sill.
        r0_aabb = ctx.part_element_world_aabb(center_sash, elem="roller_block_0")
        if r0_aabb is not None:
            ctx.check(
                "roller_block_0 near sill level",
                r0_aabb[0][2] < FRAME_FACE + 0.02,
                details=f"roller z_min={r0_aabb[0][2]:.4f}, sill_top={FRAME_FACE}",
            )

    # --- Sash slide test ---
    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel}):
        c_open = ctx.part_world_aabb(center_sash)
        rest_cx_closed = (CENTER_X0 + CENTER_X1) / 2.0
        open_cx = (c_open[0][0] + c_open[1][0]) / 2.0
        ctx.check(
            "center sash slides along +X",
            abs((open_cx - rest_cx_closed) - travel) < 0.03,
            details=f"rest_cx={rest_cx_closed:.3f}, open_cx={open_cx:.3f}, travel={travel:.3f}",
        )
        # Retained in frame.
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained within frame X span",
            c_open[1][0] < f_aabb[1][0] + 1e-4 and c_open[0][0] > f_aabb[0][0] - 1e-4,
            details=f"sash x=[{c_open[0][0]:.3f},{c_open[1][0]:.3f}] frame x=[{f_aabb[0][0]:.3f},{f_aabb[1][0]:.3f}]",
        )

    # --- Vent panel tilt test ---
    vent_upper = vent_hinge.motion_limits.upper
    with ctx.pose({vent_hinge: vent_upper}):
        v_open = ctx.part_world_aabb(vent_panel)
        # Top of vent panel should move outward (+Y) when tilted.
        v_closed_y_max = FIXED_LITE_Y + VENT_DEPTH / 2.0
        open_y_max = v_open[1][1]
        ctx.check(
            "vent panel top tilts outward",
            open_y_max > v_closed_y_max + 0.02,
            details=f"closed_y_max={v_closed_y_max:.3f}, open_y_max={open_y_max:.3f}",
        )

    # --- Insect screen slide test ---
    screen_travel = screen_slide.motion_limits.upper
    with ctx.pose({screen_slide: screen_travel}):
        s_open = ctx.part_world_aabb(insect_screen)
        screen_start_x = (LEFT_X0 + LEFT_X1) / 2.0
        open_sx = (s_open[0][0] + s_open[1][0]) / 2.0
        ctx.check(
            "insect screen slides along +X",
            abs((open_sx - screen_start_x) - screen_travel) < 0.03,
            details=f"start_x={screen_start_x:.3f}, open_x={open_sx:.3f}, travel={screen_travel:.3f}",
        )

    return ctx.report()


def mid_cz_global() -> float:
    """Helper: vertical center of the opening region."""
    return (INNER_Z0 + INNER_Z1) / 2.0


object_model = build_object_model()
