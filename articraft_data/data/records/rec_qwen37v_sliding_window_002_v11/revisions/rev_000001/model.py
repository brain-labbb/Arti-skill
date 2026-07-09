from __future__ import annotations

# Two-panel horizontal sliding window variant: white vinyl frame with deep
# track grooves, LEFT sash movable (prismatic), RIGHT sash fixed.
# A small cam latch with a rotating thumb-turn lever is mounted at the meeting
# rail on the movable sash.  A recessed pull cup is on the movable sash face.
#
# Coordinate convention:
#   +Z up, window stands vertically
#   width -> X, height -> Z, frame depth -> Y
#   q=0 for slide reads SHUT.  Positive q slides left sash toward +X
#   (toward the fixed right sash) to open, staying retained in the tracks.
#   q=0 for latch reads LOCKED (lever vertical).  Positive q rotates the
#   thumb-turn around Y so the lever sweeps in the X-Z plane.

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

FRAME_FACE = 0.085         # outer frame member face width (chunky vinyl)
FRAME_DEPTH = 0.140        # deep box section along Y

MEETING_OVERLAP = 0.040    # sash stile overlap at center

SASH_FACE = 0.075          # sash rail/stile face width
SASH_DEPTH = 0.060         # sash depth along Y
GLASS_T = 0.008

# Y layout: fixed sash rear, sliding sash proud (front)
FIXED_SASH_Y = -0.028
SLIDE_SASH_Y = 0.044

REBATE = 0.005

# Deep track grooves (cut into head and sill)
GROOVE_DEPTH = 0.018       # channel depth along Z
GROOVE_WY = SASH_DEPTH + 0.008   # channel width along Y (0.068)

# Latch hardware (thumb-turn cam latch)
LATCH_PLATE_W = 0.025
LATCH_PLATE_H = 0.050
LATCH_PLATE_T = 0.008
LATCH_LEVER_W = 0.008      # thin paddle width (X)
LATCH_LEVER_D = 0.015      # paddle depth (Y, proud of plate)
LATCH_LEVER_H = 0.028      # paddle height (Z, the sweeping dimension)

# Recessed pull cup
CUP_W = 0.055
CUP_H = 0.035
CUP_D = 0.012

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

# LEFT sash = sliding, RIGHT sash = fixed  (variant swap from parent)
SLIDE_OPEN_CX = INNER_X0 + SASH_OPENING_W / 2.0    # left
FIXED_OPEN_CX = INNER_X1 - SASH_OPENING_W / 2.0    # right
MID_CZ = (INNER_Z0 + INNER_Z1) / 2.0

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)
GLASS_RGBA = (0.52, 0.60, 0.66, 0.30)
METAL_RGBA = (0.74, 0.76, 0.79, 1.0)
DARK_RGBA = (0.22, 0.22, 0.25, 1.0)       # dark plastic for pull cup / track liners


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery)
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float,
          y_center: float, depth: float) -> cq.Workplane:
    """Axis-aligned box in the X-Z plane, centered on y_center."""
    return (
        cq.Workplane("XY")
        .transformed(offset=((x0 + x1) / 2.0, y_center, (z0 + z1) / 2.0))
        .box(x1 - x0, depth, z1 - z0)
    )


def _build_frame_shape() -> cq.Workplane:
    """Outer frame slab with one big opening + deep track grooves in sill/head."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    cut_depth = FRAME_DEPTH + 0.02
    opening = _slab(INNER_X0, INNER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    result = outer.cut(opening)

    # Deep track grooves: cut channels into the sill top and head bottom
    gx0 = INNER_X0 + 0.01
    gx1 = INNER_X1 - 0.01
    for z_lo, z_hi in [
        (INNER_Z0 - GROOVE_DEPTH, INNER_Z0),             # sill grooves
        (INNER_Z1, INNER_Z1 + GROOVE_DEPTH),              # head grooves
    ]:
        for y_c in (SLIDE_SASH_Y, FIXED_SASH_Y):
            groove = _slab(gx0, gx1, z_lo, z_hi, y_c, GROOVE_WY)
            result = result.cut(groove)
    return result


def _build_sash_shape() -> cq.Workplane:
    """Hollow sash ring in its own local frame."""
    ow = SASH_OPENING_W
    oh = SASH_OPENING_H
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SASH_DEPTH)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH + 0.02)
    return outer.cut(opening)


def _build_sash_glass_shape() -> cq.Workplane:
    ow = SASH_OPENING_W + 2 * REBATE
    oh = SASH_OPENING_H + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_pull_cup_shape() -> cq.Workplane:
    """Recessed pull cup: a shallow rectangular dish open on the +Y face."""
    wall = 0.004
    outer = cq.Workplane("XY").box(CUP_W, CUP_D, CUP_H)
    pocket = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, wall, 0.0))
        .box(CUP_W - 2 * wall, CUP_D, CUP_H - 2 * wall)
    )
    return outer.cut(pocket)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_window_left_movable")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("metal", rgba=METAL_RGBA)
    model.material("dark_plastic", rgba=DARK_RGBA)

    # ---- Frame (root) ----
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="vinyl",
        name="frame_shell",
    )

    # Track groove liner visuals (dark strips seated in the groove channels).
    # They extend slightly into the frame solid for geometric connectivity.
    groove_ext = 0.004   # overlap into frame solid below/above the cut
    groove_vis_h = GROOVE_DEPTH + groove_ext
    groove_vis_w = GROOVE_WY * 0.85
    groove_vis_xw = INNER_W * 0.90

    # Sill grooves
    sill_z = INNER_Z0 - (GROOVE_DEPTH + groove_ext) / 2.0
    frame.visual(
        Box((groove_vis_xw, groove_vis_w, groove_vis_h)),
        origin=Origin(xyz=(0.0, SLIDE_SASH_Y, sill_z)),
        material="dark_plastic",
        name="sill_groove_front",
    )
    frame.visual(
        Box((groove_vis_xw, groove_vis_w, groove_vis_h)),
        origin=Origin(xyz=(0.0, FIXED_SASH_Y, sill_z)),
        material="dark_plastic",
        name="sill_groove_rear",
    )

    # Head grooves
    head_z = INNER_Z1 + (GROOVE_DEPTH + groove_ext) / 2.0
    frame.visual(
        Box((groove_vis_xw, groove_vis_w, groove_vis_h)),
        origin=Origin(xyz=(0.0, SLIDE_SASH_Y, head_z)),
        material="dark_plastic",
        name="head_groove_front",
    )
    frame.visual(
        Box((groove_vis_xw, groove_vis_w, groove_vis_h)),
        origin=Origin(xyz=(0.0, FIXED_SASH_Y, head_z)),
        material="dark_plastic",
        name="head_groove_rear",
    )

    # ---- Fixed sash (RIGHT, rear track) ----
    fixed = model.part("fixed_sash")
    fixed.visual(
        mesh_from_cadquery(_build_sash_shape(), "fixed_sash_vinyl"),
        material="vinyl", name="fixed_sash_vinyl",
    )
    fixed.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "fixed_sash_glass"),
        material="glass", name="fixed_sash_glass",
    )

    # ---- Sliding sash (LEFT, front track) ----
    slider = model.part("sliding_sash")
    slider.visual(
        mesh_from_cadquery(_build_sash_shape(), "sliding_sash_vinyl"),
        material="vinyl", name="sliding_sash_vinyl",
    )
    slider.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "sliding_sash_glass"),
        material="glass", name="sliding_sash_glass",
    )

    # Latch base plate (fixed to the sash – does NOT rotate with the latch part)
    meeting_stile_x = SASH_OPENING_W / 2.0 + SASH_FACE / 2.0   # right stile of left sash
    plate_y = SASH_DEPTH / 2.0 + LATCH_PLATE_T / 2.0
    slider.visual(
        Box((LATCH_PLATE_W, LATCH_PLATE_T, LATCH_PLATE_H)),
        origin=Origin(xyz=(meeting_stile_x, plate_y, 0.0)),
        material="metal",
        name="latch_base",
    )

    # Recessed pull cup on sliding sash front face, seated on the bottom rail
    cup_x = 0.0                                  # centered on the bottom rail
    cup_y = SASH_DEPTH / 2.0 - 0.002            # embedded into rail for connectivity
    cup_z = -SASH_OPENING_H / 2.0 - SASH_FACE / 2.0  # on the bottom rail
    slider.visual(
        mesh_from_cadquery(_build_pull_cup_shape(), "pull_cup"),
        origin=Origin(xyz=(cup_x, cup_y, cup_z)),
        material="dark_plastic",
        name="pull_cup",
    )

    # ---- Latch (separate part – rotating thumb-turn lever) ----
    latch = model.part("latch")
    latch.visual(
        Box((LATCH_LEVER_W, LATCH_LEVER_D, LATCH_LEVER_H)),
        origin=Origin(xyz=(0.0, LATCH_LEVER_D / 2.0, 0.0)),
        material="metal",
        name="latch_lever",
    )

    # ---- Articulations ----

    # Fixed sash (right, rear)
    model.articulation(
        "frame_to_fixed_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="fixed_sash",
        origin=Origin(xyz=(FIXED_OPEN_CX, FIXED_SASH_Y, MID_CZ)),
    )

    # Sliding sash (left, front, prismatic +X to open)
    slide_travel = SASH_OPENING_W * 0.90
    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="sliding_sash",
        origin=Origin(xyz=(SLIDE_OPEN_CX, SLIDE_SASH_Y, MID_CZ)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.5, lower=0.0, upper=slide_travel
        ),
    )

    # Latch revolute joint: on the meeting stile of the sliding sash.
    # Axis along Y so the lever sweeps in the X-Z plane (thumb-turn motion).
    latch_origin_x = meeting_stile_x
    latch_origin_y = SASH_DEPTH / 2.0 + LATCH_PLATE_T   # at the plate front face
    model.articulation(
        "sash_to_latch",
        ArticulationType.REVOLUTE,
        parent="sliding_sash",
        child="latch",
        origin=Origin(xyz=(latch_origin_x, latch_origin_y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=1.5, lower=0.0, upper=1.2
        ),
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
    latch = object_model.get_part("latch")
    slide = object_model.get_articulation("frame_to_sliding_sash")
    latch_joint = object_model.get_articulation("sash_to_latch")

    # --- Allowances ---
    # Glass rebated under sash lip on each sash
    for nm in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass", elem_b=f"{nm}_vinyl",
            reason="Clear pane is rebated under the sash lip (captured glazing).",
        )
    # Sash rings rebated into the frame opening / track
    for nm in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell", elem_b=f"{nm}_vinyl",
            reason=f"{nm} ring is rebated into the frame track (seated capture).",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell", elem_b=f"{nm}_glass",
            reason=f"{nm} glass rebated under frame opening lip.",
        )

    # Latch base plate seated on sliding sash meeting stile
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="latch_base", elem_b="sliding_sash_vinyl",
        reason="Latch base plate is seated onto the sliding sash meeting stile face.",
    )

    # Pull cup recessed into the sash front face
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="pull_cup", elem_b="sliding_sash_vinyl",
        reason="Pull cup is recessed into the sliding sash front face (seated mount).",
    )

    # Track groove liners seated in the frame groove channels
    for groove_name in (
        "sill_groove_front", "sill_groove_rear",
        "head_groove_front", "head_groove_rear",
    ):
        ctx.allow_overlap(
            "frame", "frame",
            elem_a="frame_shell", elem_b=groove_name,
            reason=f"Track groove liner sits inside the frame groove cut ({groove_name}).",
        )

    # Sash rails sit in the track grooves (groove liner touches sash vinyl)
    ctx.allow_overlap(
        "frame", "sliding_sash",
        elem_a="sill_groove_front", elem_b="sliding_sash_vinyl",
        reason="Sliding sash bottom rail sits in the front sill track groove.",
    )
    ctx.allow_overlap(
        "frame", "sliding_sash",
        elem_a="head_groove_front", elem_b="sliding_sash_vinyl",
        reason="Sliding sash top rail sits in the front head track groove.",
    )
    ctx.allow_overlap(
        "frame", "fixed_sash",
        elem_a="sill_groove_rear", elem_b="fixed_sash_vinyl",
        reason="Fixed sash bottom rail sits in the rear sill track groove.",
    )
    ctx.allow_overlap(
        "frame", "fixed_sash",
        elem_a="head_groove_rear", elem_b="fixed_sash_vinyl",
        reason="Fixed sash top rail sits in the rear head track groove.",
    )

    # Glass panes extend into the track groove region (realistic glazing capture)
    ctx.allow_overlap(
        "frame", "sliding_sash",
        elem_a="head_groove_front", elem_b="sliding_sash_glass",
        reason="Sliding sash glass extends into the head track groove (glazing capture).",
    )
    ctx.allow_overlap(
        "frame", "sliding_sash",
        elem_a="sill_groove_front", elem_b="sliding_sash_glass",
        reason="Sliding sash glass extends into the sill track groove (glazing capture).",
    )
    ctx.allow_overlap(
        "frame", "fixed_sash",
        elem_a="head_groove_rear", elem_b="fixed_sash_glass",
        reason="Fixed sash glass extends into the head track groove (glazing capture).",
    )
    ctx.allow_overlap(
        "frame", "fixed_sash",
        elem_a="sill_groove_rear", elem_b="fixed_sash_glass",
        reason="Fixed sash glass extends into the sill track groove (glazing capture).",
    )

    # --- Joint type checks ---
    ctx.check(
        "slide joint is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )
    ctx.check(
        "latch joint is revolute",
        latch_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={latch_joint.articulation_type}",
    )
    ctx.check(
        "fixed sash joint is fixed",
        object_model.get_articulation("frame_to_fixed_sash").articulation_type
        == ArticulationType.FIXED,
        details="frame_to_fixed_sash must be FIXED",
    )

    # --- Closed pose (q=0): window reads SHUT ---
    with ctx.pose({slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        f_aabb = ctx.part_world_aabb(fixed_sash)
        s_aabb = ctx.part_world_aabb(sliding_sash)
        l_aabb = ctx.part_world_aabb(latch)

        # Sliding sash on the LEFT, fixed on the RIGHT
        sx = (s_aabb[0][0] + s_aabb[1][0]) / 2.0
        fx = (f_aabb[0][0] + f_aabb[1][0]) / 2.0
        ctx.check(
            "sliding sash on left side",
            sx < fx,
            details=f"slide_x={sx:.3f}, fixed_x={fx:.3f}",
        )

        # Frame dimensions
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        sash_w = s_aabb[1][0] - s_aabb[0][0]
        ctx.check(
            "frame spans wider than a single sash",
            frame_w > sash_w + 0.40,
            details=f"frame_w={frame_w:.3f}, sash_w={sash_w:.3f}",
        )
        ctx.check(
            "sill near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"zmin={frame_aabb[0][2]:.4f}",
        )
        ctx.check(
            "head reaches full height",
            abs(frame_aabb[1][2] - TOTAL_H) < 0.02,
            details=f"zmax={frame_aabb[1][2]:.4f}",
        )

        # Both sashes seated within frame height
        for nm, ab in (("fixed", f_aabb), ("sliding", s_aabb)):
            ctx.check(
                f"{nm} sash within frame height",
                ab[0][2] > frame_aabb[0][2] - 1e-4 and ab[1][2] < frame_aabb[1][2] + 1e-4,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}]",
            )

        # Sliding sash proud (front, +Y) of fixed sash
        sy = (s_aabb[0][1] + s_aabb[1][1]) / 2.0
        fy = (f_aabb[0][1] + f_aabb[1][1]) / 2.0
        ctx.check(
            "sliding sash proud of fixed sash",
            sy > fy + 0.02,
            details=f"slide_y={sy:.3f}, fixed_y={fy:.3f}",
        )

        # Both sashes seated in frame opening
        ctx.expect_overlap(
            fixed_sash, frame, axes="xz", min_overlap=0.03,
            name="fixed sash seated in frame opening",
        )
        ctx.expect_overlap(
            sliding_sash, frame, axes="xz", min_overlap=0.03,
            name="sliding sash seated in frame opening",
        )

        # Latch on the meeting stile (right side of the left sash)
        latch_cx = (l_aabb[0][0] + l_aabb[1][0]) / 2.0
        ctx.check(
            "latch on meeting stile (right of sliding sash center)",
            latch_cx > sx,
            details=f"latch_x={latch_cx:.3f}, sash_x={sx:.3f}",
        )
        latch_cz = (l_aabb[0][2] + l_aabb[1][2]) / 2.0
        ctx.check(
            "latch near mid-height",
            abs(latch_cz - MID_CZ) < 0.20,
            details=f"latch_z={latch_cz:.3f}, mid_z={MID_CZ:.3f}",
        )

        # Pull cup on the sliding sash front face
        cup_aabb = ctx.part_element_world_aabb(sliding_sash, elem="pull_cup")
        cup_cy = (cup_aabb[0][1] + cup_aabb[1][1]) / 2.0
        ctx.check(
            "pull cup on sash front face",
            cup_cy > sy,
            details=f"cup_y={cup_cy:.3f}, sash_y={sy:.3f}",
        )

        # Track groove liners positioned correctly
        sill_gf = ctx.part_element_world_aabb(frame, elem="sill_groove_front")
        sill_gr = ctx.part_element_world_aabb(frame, elem="sill_groove_rear")
        head_gf = ctx.part_element_world_aabb(frame, elem="head_groove_front")
        head_gr = ctx.part_element_world_aabb(frame, elem="head_groove_rear")
        ctx.check(
            "sill front groove below opening",
            sill_gf[1][2] <= INNER_Z0 + 0.002,
            details=f"groove_zmax={sill_gf[1][2]:.4f}",
        )
        ctx.check(
            "head front groove above opening",
            head_gf[0][2] >= INNER_Z1 - 0.002,
            details=f"groove_zmin={head_gf[0][2]:.4f}",
        )
        sill_gf_y = (sill_gf[0][1] + sill_gf[1][1]) / 2.0
        sill_gr_y = (sill_gr[0][1] + sill_gr[1][1]) / 2.0
        ctx.check(
            "sill grooves at distinct Y (front/rear tracks)",
            abs(sill_gf_y - sill_gr_y) > 0.03,
            details=f"front_y={sill_gf_y:.3f}, rear_y={sill_gr_y:.3f}",
        )

        # Glass panes are captured in the track grooves (vertical overlap)
        ctx.expect_overlap(
            sliding_sash, frame, axes="z",
            elem_a="sliding_sash_glass", elem_b="sill_groove_front",
            min_overlap=0.003,
            name="sliding sash glass captured in sill groove",
        )
        ctx.expect_overlap(
            sliding_sash, frame, axes="z",
            elem_a="sliding_sash_glass", elem_b="head_groove_front",
            min_overlap=0.003,
            name="sliding sash glass captured in head groove",
        )
        ctx.expect_overlap(
            fixed_sash, frame, axes="z",
            elem_a="fixed_sash_glass", elem_b="sill_groove_rear",
            min_overlap=0.003,
            name="fixed sash glass captured in sill groove",
        )
        ctx.expect_overlap(
            fixed_sash, frame, axes="z",
            elem_a="fixed_sash_glass", elem_b="head_groove_rear",
            min_overlap=0.003,
            name="fixed sash glass captured in head groove",
        )

        rest_sx = sx
        rest_sz = (s_aabb[0][2] + s_aabb[1][2]) / 2.0

    # --- Open pose: sliding sash slides toward +X (right, toward fixed sash) ---
    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel}):
        s_open = ctx.part_world_aabb(sliding_sash)
        open_sx = (s_open[0][0] + s_open[1][0]) / 2.0
        ctx.check(
            "sliding sash opens toward +X",
            open_sx > rest_sx + 0.30,
            details=f"rest_x={rest_sx:.3f}, open_x={open_sx:.3f}",
        )
        open_sz = (s_open[0][2] + s_open[1][2]) / 2.0
        ctx.check(
            "slide is purely horizontal",
            abs(open_sz - rest_sz) < 0.02,
            details=f"open_z={open_sz:.3f}, rest_z={rest_sz:.3f}",
        )
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained in frame at full travel",
            s_open[0][0] > f_aabb[0][0] - 1e-4 and s_open[1][0] < f_aabb[1][0] + 1e-4,
            details=f"sash x=[{s_open[0][0]:.3f},{s_open[1][0]:.3f}] "
                    f"frame x=[{f_aabb[0][0]:.3f},{f_aabb[1][0]:.3f}]",
        )
        ctx.expect_overlap(
            sliding_sash, frame, axes="z", min_overlap=0.10,
            name="sash retains head/sill track engagement",
        )

    # --- Latch rotation test ---
    latch_upper = latch_joint.motion_limits.upper
    with ctx.pose({latch_joint: 0.0}):
        latch_rest = ctx.part_world_aabb(latch)
        rest_xspan = latch_rest[1][0] - latch_rest[0][0]
        rest_zspan = latch_rest[1][2] - latch_rest[0][2]

    with ctx.pose({latch_joint: latch_upper}):
        latch_open = ctx.part_world_aabb(latch)
        open_xspan = latch_open[1][0] - latch_open[0][0]
        open_zspan = latch_open[1][2] - latch_open[0][2]

    ctx.check(
        "latch rotation changes X-Z profile (thumb-turn sweeps)",
        abs(open_xspan - rest_xspan) > 0.005 or abs(open_zspan - rest_zspan) > 0.005,
        details=f"rest_xspan={rest_xspan:.4f} zspan={rest_zspan:.4f}, "
                f"open_xspan={open_xspan:.4f} zspan={open_zspan:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
