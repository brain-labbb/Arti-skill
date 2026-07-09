from __future__ import annotations

# Horizontal sliding window: white frame with two side-by-side six-lite sashes
# that slide horizontally in opposite directions on separate tracks.
# Right sash is partially open at rest, showing visible overlap with the left
# sash. Two roller blocks at the bottom of the right (moving) sash.
# Sill lip with drainage slots on the exterior sill face.
#
# Coordinate convention:
#   +Z is up. Window stands vertically: height along +Z, width along X, frame
#   depth / glazing thickness along Y (the glass plane is the X-Z plane).
#   Sill sits at z=0; head at z=WIN_H.
#
# Articulation (horizontal slider):
#   - LEFT sash is PRISMATIC, axis (+1,0,0): positive q slides it RIGHT (opens
#     the left side). Rides on the interior (-Y) track.
#   - RIGHT sash is PRISMATIC, axis (-1,0,0): positive q slides it LEFT (opens
#     the right side). Rides on the exterior (+Y) track.
#   At rest (q=0), the right sash is partially open (origin placed at a
#   partially-slid position) so the overlap region is visible.

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

WIN_W = 1.20          # overall window width (X) — landscape slider proportions
WIN_H = 1.00          # overall window height (Z), sill at z=0
FRAME_FACE = 0.060    # outer frame member face width (X/Z)
FRAME_DEPTH = 0.110   # outer frame jamb depth (Y)

# Clear opening inside the outer frame
OPEN_W = WIN_W - 2 * FRAME_FACE   # 1.080
OPEN_H = WIN_H - 2 * FRAME_FACE   # 0.880
OPEN_X0 = -OPEN_W / 2.0           # -0.540
OPEN_X1 = OPEN_W / 2.0            # +0.540
OPEN_Z0 = FRAME_FACE               # 0.060
OPEN_Z1 = WIN_H - FRAME_FACE      # 0.940

# Sash geometry. Each sash is wider than half the opening so they overlap at
# the meeting stile when closed.
SASH_W = OPEN_W * 0.52            # ~0.562 — covers just over half
SASH_RAIL = 0.050                 # sash perimeter member width (stile/rail)
SASH_DEPTH = 0.034                # sash thickness (Y)
SASH_H = OPEN_H - 0.010           # 0.870 — near full opening height
GLASS_T = 0.006                   # glazing thickness (Y)

# Y planes: left sash rides interior (-Y), right sash rides exterior (+Y),
# offset so they pass each other at the meeting stile.
SASH_Y_GAP = 0.018
FRONT_SASH_Y = -SASH_Y_GAP        # left sash (interior track)
REAR_SASH_Y = +SASH_Y_GAP         # right sash (exterior track)

# Muntin grid: 2 columns x 3 rows per sash -> 6 lites (tall narrow sashes).
MUNTIN_W = 0.020
N_COLS = 2
N_ROWS = 3

# Track channels in head and sill (horizontal grooves where sashes slide).
TRACK_W = 0.020
TRACK_DEPTH = 0.012

# Sill lip (exterior flange at the bottom of the frame).
SILL_LIP_DEPTH = 0.040            # how far it extends outward (+Y)
SILL_LIP_THICK = 0.015            # vertical thickness (Z)

# Drainage slots cut through the sill lip.
N_DRAIN_SLOTS = 3
DRAIN_SLOT_W = 0.040              # slot width (X)
DRAIN_SLOT_D = 0.008              # slot depth (Y, narrow slot)

# Roller blocks at the bottom of the right sash.
ROLLER_W = 0.030
ROLLER_D = 0.024
ROLLER_H = 0.010

# Sash closed-pose center X positions.
LEFT_CLOSED_X = OPEN_X0 + SASH_W / 2.0    # left sash centered in left half
RIGHT_CLOSED_X = OPEN_X1 - SASH_W / 2.0   # right sash centered in right half

# Partial-open offset for right sash at rest (q=0 is already partially open).
PARTIAL_OPEN = 0.12
RIGHT_REST_X = RIGHT_CLOSED_X - PARTIAL_OPEN

# Sash bottom Z: raised above sill to accommodate roller blocks.
SASH_BOTTOM_Z = OPEN_Z0 + 0.012

# Latch on the left sash meeting stile.
LATCH_BODY = (0.022, 0.026, 0.050)
LATCH_LEVER = (0.010, 0.012, 0.040)

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

FRAME_RGBA = (0.945, 0.945, 0.945, 1.0)    # white painted frame
SASH_RGBA = (0.965, 0.965, 0.965, 1.0)     # white sash (slightly brighter)
GLASS_RGBA = (0.30, 0.36, 0.42, 0.34)      # cool dark-tinted glass
ROLLER_RGBA = (0.22, 0.22, 0.24, 1.0)       # dark nylon rollers
LOCK_RGBA = (0.86, 0.87, 0.89, 1.0)        # brushed metal latch


# ---------------------------------------------------------------------------
# Static outer frame geometry (CadQuery)
# ---------------------------------------------------------------------------

def _build_frame_shape() -> cq.Workplane:
    """White outer frame: perimeter slab with the central opening cut out,
    plus horizontal track channels in the sill and head where the sashes slide.

    World frame: opening centered on X=0, Z from 0 (sill) to WIN_H (head).
    """
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, WIN_H / 2.0))
        .box(WIN_W, FRAME_DEPTH, WIN_H)
    )

    opening = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, (OPEN_Z0 + OPEN_Z1) / 2.0))
        .box(OPEN_W, FRAME_DEPTH + 0.02, OPEN_H)
    )
    frame = outer.cut(opening)

    # Horizontal track grooves in the sill (top face, cut downward).
    # Two grooves: one per sash track plane.
    for ty in (FRONT_SASH_Y, REAR_SASH_Y):
        groove = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, ty, OPEN_Z0 - TRACK_DEPTH / 2.0))
            .box(OPEN_W + 0.01, TRACK_W, TRACK_DEPTH)
        )
        frame = frame.cut(groove)

    # Horizontal track grooves in the head (bottom face, cut upward).
    for ty in (FRONT_SASH_Y, REAR_SASH_Y):
        groove = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, ty, OPEN_Z1 + TRACK_DEPTH / 2.0))
            .box(OPEN_W + 0.01, TRACK_W, TRACK_DEPTH)
        )
        frame = frame.cut(groove)

    return frame


def _build_sill_lip_shape() -> cq.Workplane:
    """Sill lip: horizontal flange extending outward (+Y) from the sill bottom,
    with drainage slots cut through it.

    The lip overlaps the frame sill by a few mm so it reads as integrally joined.
    """
    overlap = 0.004  # penetrate into the frame sill for connectivity
    lip_y_center = FRAME_DEPTH / 2.0 + SILL_LIP_DEPTH / 2.0 - overlap / 2.0
    lip_y_extent = SILL_LIP_DEPTH + overlap

    lip = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, lip_y_center, SILL_LIP_THICK / 2.0))
        .box(WIN_W, lip_y_extent, SILL_LIP_THICK)
    )

    # Cut drainage slots (vertical through-holes in the lip).
    slot_spacing = WIN_W / (N_DRAIN_SLOTS + 1)
    for i in range(N_DRAIN_SLOTS):
        sx = -WIN_W / 2.0 + slot_spacing * (i + 1)
        slot = (
            cq.Workplane("XY")
            .transformed(offset=(sx, lip_y_center, SILL_LIP_THICK / 2.0))
            .box(DRAIN_SLOT_W, DRAIN_SLOT_D, SILL_LIP_THICK + 0.01)
        )
        lip = lip.cut(slot)

    return lip


# ---------------------------------------------------------------------------
# Sash frame geometry (CadQuery): perimeter ring + 6-lite muntin grid
# ---------------------------------------------------------------------------

def _build_sash_frame_shape() -> cq.Workplane:
    """One sash: perimeter ring plus a 2x3 muntin grid, built as a slab with
    six rectangular lite openings cut, leaving a true muntin lattice.

    Authored in the sash-local frame:
      - local X runs -SASH_W/2 .. +SASH_W/2
      - local Z runs 0 .. SASH_H (bottom rail at z=0)
      - local Y is the sash thickness, centered at y=0.
    """
    w = SASH_W
    h = SASH_H
    r = SASH_RAIL
    d = SASH_DEPTH

    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(w, d, h)
    )

    in_x0, in_x1 = -w / 2.0 + r, w / 2.0 - r
    in_z0, in_z1 = r, h - r
    inner_w = in_x1 - in_x0
    inner_h = in_z1 - in_z0

    col_lines = [in_x0 + (i + 1) * inner_w / N_COLS for i in range(N_COLS - 1)]
    row_lines = [in_z0 + (j + 1) * inner_h / N_ROWS for j in range(N_ROWS - 1)]

    x_edges = [in_x0] + col_lines + [in_x1]
    z_edges = [in_z0] + row_lines + [in_z1]
    half_m = MUNTIN_W / 2.0

    sash = outer
    for ci in range(N_COLS):
        for ri in range(N_ROWS):
            lx0 = x_edges[ci] + (half_m if ci > 0 else 0.0)
            lx1 = x_edges[ci + 1] - (half_m if ci < N_COLS - 1 else 0.0)
            lz0 = z_edges[ri] + (half_m if ri > 0 else 0.0)
            lz1 = z_edges[ri + 1] - (half_m if ri < N_ROWS - 1 else 0.0)
            lite = (
                cq.Workplane("XY")
                .transformed(offset=((lx0 + lx1) / 2.0, 0.0, (lz0 + lz1) / 2.0))
                .box(lx1 - lx0, d + 0.02, lz1 - lz0)
            )
            sash = sash.cut(lite)

    return sash


def _build_sash_glass_shape() -> cq.Workplane:
    """Six thin glass panes filling the lite openings, rebated under the
    muntin/rail lips so the glass reads as captured, not floating."""
    w = SASH_W
    h = SASH_H
    r = SASH_RAIL
    rebate = 0.005

    in_x0, in_x1 = -w / 2.0 + r, w / 2.0 - r
    in_z0, in_z1 = r, h - r
    inner_w = in_x1 - in_x0
    inner_h = in_z1 - in_z0

    col_lines = [in_x0 + (i + 1) * inner_w / N_COLS for i in range(N_COLS - 1)]
    row_lines = [in_z0 + (j + 1) * inner_h / N_ROWS for j in range(N_ROWS - 1)]

    x_edges = [in_x0] + col_lines + [in_x1]
    z_edges = [in_z0] + row_lines + [in_z1]
    half_m = MUNTIN_W / 2.0

    panes = None
    for ci in range(N_COLS):
        for ri in range(N_ROWS):
            lx0 = x_edges[ci] + (half_m if ci > 0 else 0.0) - rebate
            lx1 = x_edges[ci + 1] - (half_m if ci < N_COLS - 1 else 0.0) + rebate
            lz0 = z_edges[ri] + (half_m if ri > 0 else 0.0) - rebate
            lz1 = z_edges[ri + 1] - (half_m if ri < N_ROWS - 1 else 0.0) + rebate
            pane = (
                cq.Workplane("XY")
                .transformed(offset=((lx0 + lx1) / 2.0, 0.0, (lz0 + lz1) / 2.0))
                .box(lx1 - lx0, GLASS_T, lz1 - lz0)
            )
            panes = pane if panes is None else panes.union(pane)

    return panes


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_window")

    model.material("frame", rgba=FRAME_RGBA)
    model.material("sash", rgba=SASH_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("roller", rgba=ROLLER_RGBA)
    model.material("lock", rgba=LOCK_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="frame",
        name="frame_shell",
    )
    frame.visual(
        mesh_from_cadquery(_build_sill_lip_shape(), "sill_lip"),
        material="frame",
        name="sill_lip",
    )

    # --- Left sash (front/interior track) ---
    left = model.part("left_sash")
    left.visual(
        mesh_from_cadquery(_build_sash_frame_shape(), "left_sash_frame"),
        material="sash",
        name="left_sash_frame",
    )
    left.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "left_sash_glass"),
        material="glass",
        name="left_sash_glass",
    )
    # Latch on the meeting stile (right stile of left sash), interior face.
    latch_x = SASH_W / 2.0 - SASH_RAIL / 2.0
    latch_z = SASH_H / 2.0
    latch_y = -(SASH_DEPTH / 2.0 + LATCH_BODY[1] / 2.0 - 0.004)
    left.visual(
        Box(LATCH_BODY),
        origin=Origin(xyz=(latch_x, latch_y, latch_z)),
        material="lock",
        name="left_sash_latch_body",
    )
    left.visual(
        Box(LATCH_LEVER),
        origin=Origin(xyz=(latch_x, latch_y - LATCH_BODY[1] / 2.0, latch_z)),
        material="lock",
        name="left_sash_latch_lever",
    )

    # --- Right sash (rear/exterior track, partially open at rest) ---
    right = model.part("right_sash")
    right.visual(
        mesh_from_cadquery(_build_sash_frame_shape(), "right_sash_frame"),
        material="sash",
        name="right_sash_frame",
    )
    right.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "right_sash_glass"),
        material="glass",
        name="right_sash_glass",
    )
    # Two roller blocks at the bottom of the right sash.
    roller_x_offset = SASH_W / 2.0 - ROLLER_W / 2.0 - 0.040
    roller_z = -(ROLLER_H / 2.0 - 0.003)  # embed 3mm into sash bottom rail for mounting
    for idx, rx in enumerate((-roller_x_offset, roller_x_offset)):
        right.visual(
            Box((ROLLER_W, ROLLER_D, ROLLER_H)),
            origin=Origin(xyz=(rx, 0.0, roller_z)),
            material="roller",
            name=f"roller_{idx}",
        )

    # ----- Articulations (horizontal slider) -----
    # Both sashes are authored with their bottom rail at local z=0 and centered
    # in X. The joint origin is placed at each sash's rest world position.
    # q=0 is the rest state: left sash closed, right sash partially open.

    # LEFT sash: slides RIGHT (+X) to open.
    model.articulation(
        "frame_to_left_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="left_sash",
        origin=Origin(xyz=(LEFT_CLOSED_X, FRONT_SASH_Y, SASH_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.25, lower=0.0, upper=SASH_W * 0.45
        ),
    )

    # RIGHT sash: slides LEFT (-X) to open. Origin is at the partially-open
    # rest position so q=0 shows the visible overlap with the left sash.
    model.articulation(
        "frame_to_right_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="right_sash",
        origin=Origin(xyz=(RIGHT_REST_X, REAR_SASH_Y, SASH_BOTTOM_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.25, lower=0.0, upper=0.20
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    left = object_model.get_part("left_sash")
    right = object_model.get_part("right_sash")
    j_left = object_model.get_articulation("frame_to_left_sash")
    j_right = object_model.get_articulation("frame_to_right_sash")

    # --- Intentional overlaps ---
    # Glass panes tuck under the sash muntin/rail lips (captured glazing).
    for sash_name in ("left_sash", "right_sash"):
        ctx.allow_overlap(
            sash_name, sash_name,
            elem_a=f"{sash_name}_glass",
            elem_b=f"{sash_name}_frame",
            reason="Glass panes are rebated under the sash rails/muntins so they read as captured, not floating.",
        )
    # Each sash rides in the head/sill track grooves cut into the frame.
    ctx.allow_overlap(
        "frame", "left_sash",
        reason="Left sash rides in the interior track grooves in the frame head and sill.",
    )
    ctx.allow_overlap(
        "frame", "right_sash",
        reason="Right sash rides in the exterior track grooves in the frame head and sill.",
    )
    # The two sashes overlap in X at the meeting region (different Y planes).
    ctx.allow_overlap(
        "left_sash", "right_sash",
        reason="Sashes overlap in X projection at the meeting stile region; they ride in offset Y track planes.",
    )
    # Sill lip is integrally joined to the frame sill (small overlap for connectivity).
    ctx.allow_overlap(
        "frame", "frame",
        elem_a="sill_lip",
        elem_b="frame_shell",
        reason="Sill lip is integrally joined to the frame sill with a small structural overlap.",
    )
    # Latch body seated on left sash meeting stile.
    ctx.allow_overlap(
        "left_sash", "left_sash",
        elem_a="left_sash_latch_body",
        elem_b="left_sash_frame",
        reason="Latch body is mounted (seated) onto the left sash meeting stile.",
    )
    # Roller blocks mounted at the bottom of the right sash.
    for roller_name in ("roller_0", "roller_1"):
        ctx.allow_overlap(
            "right_sash", "right_sash",
            elem_a=roller_name,
            elem_b="right_sash_frame",
            reason="Roller block is mounted at the bottom rail of the right sash.",
        )

    # --- Rest pose (q=0): left sash closed, right sash partially open ---
    with ctx.pose({j_left: 0.0, j_right: 0.0}):
        f_aabb = ctx.part_world_aabb(frame)
        lo_aabb = ctx.part_world_aabb(left)
        ro_aabb = ctx.part_world_aabb(right)

        frame_w = f_aabb[1][0] - f_aabb[0][0]
        frame_h = f_aabb[1][2] - f_aabb[0][2]

        # Slider proportions: frame is wider than tall.
        ctx.check(
            "frame wider than tall (slider proportions)",
            frame_w > frame_h + 0.10,
            details=f"frame_w={frame_w:.3f}, frame_h={frame_h:.3f}",
        )
        # Sill near z=0.
        ctx.check(
            "frame sill near z=0",
            abs(f_aabb[0][2]) < 0.02 and f_aabb[1][2] > 0.80,
            details=f"frame z=({f_aabb[0][2]:.3f}, {f_aabb[1][2]:.3f})",
        )
        # Sashes within frame opening width.
        ctx.check(
            "sashes within frame width",
            lo_aabb[0][0] > f_aabb[0][0] - 0.01 and lo_aabb[1][0] < f_aabb[1][0] + 0.01
            and ro_aabb[0][0] > f_aabb[0][0] - 0.01 and ro_aabb[1][0] < f_aabb[1][0] + 0.01,
            details=f"left x=({lo_aabb[0][0]:.3f},{lo_aabb[1][0]:.3f}) right x=({ro_aabb[0][0]:.3f},{ro_aabb[1][0]:.3f})",
        )
        # Sashes side by side (left sash center is to the left of right sash center).
        lo_cx = (lo_aabb[0][0] + lo_aabb[1][0]) / 2.0
        ro_cx = (ro_aabb[0][0] + ro_aabb[1][0]) / 2.0
        ctx.check(
            "sashes side by side (left of right)",
            lo_cx < ro_cx - 0.05,
            details=f"left_cx={lo_cx:.3f}, right_cx={ro_cx:.3f}",
        )
        # Right sash partially open: right edge well inside the frame opening.
        ctx.check(
            "right sash partially open at rest",
            ro_aabb[1][0] < f_aabb[1][0] - FRAME_FACE - 0.05,
            details=f"right_edge={ro_aabb[1][0]:.3f}, frame_right_inner={f_aabb[1][0] - FRAME_FACE:.3f}",
        )
        # Sashes overlap in X (visible overlap region at the meeting stile).
        ctx.expect_overlap(
            left, right, axes="x", min_overlap=0.05,
            name="sashes have visible X overlap at rest",
        )
        # Sashes on offset Y tracks.
        lo_cy = (lo_aabb[0][1] + lo_aabb[1][1]) / 2.0
        ro_cy = (ro_aabb[0][1] + ro_aabb[1][1]) / 2.0
        ctx.check(
            "sashes on offset Y tracks",
            abs(lo_cy - ro_cy) > 0.020,
            details=f"left_cy={lo_cy:.3f}, right_cy={ro_cy:.3f}",
        )
        # Sill lip extends frame in +Y beyond base depth.
        ctx.check(
            "sill lip extends frame in +Y",
            f_aabb[1][1] > FRAME_DEPTH / 2.0 + SILL_LIP_DEPTH * 0.5,
            details=f"frame_y_max={f_aabb[1][1]:.3f}, expected>{FRAME_DEPTH / 2.0 + SILL_LIP_DEPTH * 0.5:.3f}",
        )

        rest_lo_cx = lo_cx
        rest_ro_cx = ro_cx

    # --- HERO: left sash slides RIGHT (+X) when opened ---
    travel_l = SASH_W * 0.35
    with ctx.pose({j_left: travel_l}):
        op = ctx.part_world_aabb(left)
        op_cx = (op[0][0] + op[1][0]) / 2.0
        ctx.check(
            "left sash slides right when opened",
            op_cx > rest_lo_cx + travel_l * 0.8,
            details=f"rest_cx={rest_lo_cx:.3f}, opened_cx={op_cx:.3f}, travel={travel_l:.3f}",
        )
        ctx.expect_overlap(
            left, frame, axes="x", min_overlap=0.05,
            name="left sash retained in frame when open",
        )

    # --- HERO: right sash slides further LEFT (-X) when opened ---
    travel_r = 0.10
    with ctx.pose({j_right: travel_r}):
        op = ctx.part_world_aabb(right)
        op_cx = (op[0][0] + op[1][0]) / 2.0
        ctx.check(
            "right sash slides left when opened further",
            op_cx < rest_ro_cx - travel_r * 0.8,
            details=f"rest_cx={rest_ro_cx:.3f}, opened_cx={op_cx:.3f}, travel={travel_r:.3f}",
        )
        ctx.expect_overlap(
            right, frame, axes="x", min_overlap=0.05,
            name="right sash retained in frame when open",
        )

    # --- Roller blocks exist on right sash and are near the bottom ---
    r0_aabb = ctx.part_element_world_aabb(right, elem="roller_0")
    r1_aabb = ctx.part_element_world_aabb(right, elem="roller_1")
    ctx.check(
        "roller_0 exists on right sash",
        r0_aabb is not None,
        details="roller_0 visual not found",
    )
    ctx.check(
        "roller_1 exists on right sash",
        r1_aabb is not None,
        details="roller_1 visual not found",
    )
    if r0_aabb is not None and r1_aabb is not None:
        ro_aabb = ctx.part_world_aabb(right)
        ctx.check(
            "rollers near bottom of right sash",
            r0_aabb[0][2] < ro_aabb[0][2] + 0.04 and r1_aabb[0][2] < ro_aabb[0][2] + 0.04,
            details=f"roller0_z_min={r0_aabb[0][2]:.3f}, roller1_z_min={r1_aabb[0][2]:.3f}, sash_z_min={ro_aabb[0][2]:.3f}",
        )
        # Rollers are separated in X (one near each end of the sash bottom).
        r0_cx = (r0_aabb[0][0] + r0_aabb[1][0]) / 2.0
        r1_cx = (r1_aabb[0][0] + r1_aabb[1][0]) / 2.0
        ctx.check(
            "rollers separated in X along sash bottom",
            abs(r1_cx - r0_cx) > 0.20,
            details=f"roller0_cx={r0_cx:.3f}, roller1_cx={r1_cx:.3f}",
        )

    # --- Both joints are prismatic (non-fixed) ---
    ctx.check(
        "left sash joint is prismatic",
        j_left.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={j_left.articulation_type}",
    )
    ctx.check(
        "right sash joint is prismatic",
        j_right.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={j_right.articulation_type}",
    )

    # --- Sill lip visual exists ---
    sill_lip_aabb = ctx.part_element_world_aabb(frame, elem="sill_lip")
    ctx.check(
        "sill lip visual exists",
        sill_lip_aabb is not None,
        details="sill_lip visual not found on frame",
    )

    return ctx.report()


object_model = build_object_model()
