from __future__ import annotations

# Horizontal sliding window variant: white frame, two side-by-side six-lite
# sashes that slide horizontally. One sash is partially open at rest so the
# overlap stile is visible. Insect screen on an independent shallow prismatic
# track. Two tiny roller blocks at the bottom of the sliding sash.
#
# Coordinate convention:
#   +Z is up. Window stands vertically: height along +Z, width along X, frame
#   depth along Y (glass plane is the X-Z plane). The sill sits at z=0.
#
# Articulation:
#   - FIXED sash (left, exterior track): rigid FIXED joint.
#   - SLIDING sash (right, interior track): PRISMATIC axis (1,0,0), positive q
#     slides it to the right (further open). q=0 is the partially-open rest
#     pose; lower limit closes it, upper limit opens it more.
#   - INSECT screen: PRISMATIC axis (1,0,0), slides independently on the
#     interior-most track.

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

WIN_W = 1.20          # overall window width (X)
WIN_H = 1.00          # overall window height (Z), sill at z=0
FRAME_FACE = 0.058    # outer frame member face width (X/Z)
FRAME_DEPTH = 0.110   # outer frame jamb depth (Y)

# Clear opening inside the outer frame
OPEN_W = WIN_W - 2 * FRAME_FACE   # 1.084
OPEN_H = WIN_H - 2 * FRAME_FACE   # 0.884
OPEN_X0 = -OPEN_W / 2.0
OPEN_X1 = OPEN_W / 2.0
OPEN_Z0 = FRAME_FACE
OPEN_Z1 = WIN_H - FRAME_FACE

# Sash geometry. Each sash is slightly wider than half the opening so they
# overlap at the meeting stile in the center.
SASH_W = 0.580                  # sash width
SASH_H = OPEN_H - 0.008        # sash height (top/bottom clearance)
SASH_D = 0.032                  # sash thickness (Y)
RAIL = 0.045                    # normal stile/rail width
MTG_STILE = 0.055              # wider meeting stile for overlap visibility
MUNTIN = 0.020                  # muntin bar face width
GLASS_T = 0.006                 # glazing thickness
NCOLS, NROWS = 3, 2            # six-lite grid per sash

# Y planes: fixed sash on exterior (+Y), sliding sash on interior (-Y)
Y_GAP = 0.018                   # half the gap between the two sash planes
FIXED_Y = +Y_GAP
SLIDE_Y = -Y_GAP
SCREEN_Y = -0.050              # screen track (most interior)

# Closed-pose sash X centers
FIXED_X = OPEN_X0 + SASH_W / 2.0           # left half of opening
SLIDE_X0 = OPEN_X1 - SASH_W / 2.0         # right half of opening (closed)

# Partial-open rest offset: at q=0 the sliding sash is already partly open
PARTIAL = 0.050
SLIDE_XR = SLIDE_X0 + PARTIAL             # rest-pose X center

# Overlap at the meeting stile (closed)
# fixed right edge = FIXED_X + SASH_W/2 = OPEN_X0 + SASH_W
# slide left edge (closed) = SLIDE_X0 - SASH_W/2 = OPEN_X1 - SASH_W
# overlap = fixed_right - slide_left = 2*SASH_W - OPEN_W
# OVERLAP = 2*0.580 - 1.084 = 0.076 (76 mm at closed, 26 mm at rest)

# Track channels in head/sill
TRACK_W = 0.016
TRACK_D = 0.028

# Roller blocks (two, at bottom of sliding sash)
ROLLER_SZ = (0.025, 0.012, 0.010)   # (X, Y, Z)

# Overlap stile lip on fixed sash meeting stile
LIP_W = 0.012    # lip width along X
LIP_T = 0.008    # lip thickness along Y (extends toward interior)

# Insect screen
SCR_FW = OPEN_W - 0.012      # screen frame width
SCR_FH = OPEN_H + 0.024      # screen frame height (extends into sill/head grooves)
SCR_FD = 0.014               # screen frame depth (Y)
SCR_RAIL = 0.028             # screen frame perimeter member width
SCR_T = 0.002                # screen mesh thickness
SCREEN_ORIGIN_Z = OPEN_Z0 - 0.020   # lowered so screen bottom enters sill groove

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

FRAME_RGBA = (0.945, 0.945, 0.945, 1.0)
SASH_RGBA = (0.965, 0.965, 0.965, 1.0)
GLASS_RGBA = (0.30, 0.36, 0.42, 0.34)
SCR_FRAME_RGBA = (0.82, 0.82, 0.80, 1.0)
SCR_MESH_RGBA = (0.45, 0.47, 0.43, 0.55)
ROLLER_RGBA = (0.22, 0.22, 0.24, 1.0)


# ---------------------------------------------------------------------------
# Frame geometry (CadQuery)
# ---------------------------------------------------------------------------

def _frame_shape() -> cq.Workplane:
    """White outer frame: perimeter slab with central opening, plus shallow
    track grooves in the head and sill for the two sash planes and screen."""
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, WIN_H / 2.0))
        .box(WIN_W, FRAME_DEPTH, WIN_H)
    )
    hole = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, (OPEN_Z0 + OPEN_Z1) / 2.0))
        .box(OPEN_W, FRAME_DEPTH + 0.02, OPEN_H)
    )
    frame = outer.cut(hole)

    # Sash track grooves in sill (bottom) and head (top)
    groove_half = TRACK_D / 2.0
    for ty in (FIXED_Y, SLIDE_Y):
        # Sill groove: opens from the inner face (z = OPEN_Z0) downward
        sg = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, ty, OPEN_Z0 - groove_half))
            .box(OPEN_W + 0.01, TRACK_W, TRACK_D)
        )
        frame = frame.cut(sg)
        # Head groove: opens from the inner face (z = OPEN_Z1) upward
        hg = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, ty, OPEN_Z1 + groove_half))
            .box(OPEN_W + 0.01, TRACK_W, TRACK_D)
        )
        frame = frame.cut(hg)

    # Screen track (shallower, narrower grooves)
    scr_gd = TRACK_D * 0.6
    scr_gw = TRACK_W * 0.8
    for zc in (OPEN_Z0 - scr_gd / 2.0, OPEN_Z1 + scr_gd / 2.0):
        sg = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, SCREEN_Y, zc))
            .box(OPEN_W + 0.01, scr_gw, scr_gd)
        )
        frame = frame.cut(sg)

    return frame


# ---------------------------------------------------------------------------
# Sash frame geometry (CadQuery): perimeter ring + six-lite muntin grid
# ---------------------------------------------------------------------------

def _sash_frame_shape(meeting_side: str, with_lip: bool = False) -> cq.Workplane:
    """One sash frame: perimeter ring plus a 3x2 muntin grid.

    meeting_side: 'left' or 'right' — which stile is the wider meeting stile.
    with_lip: if True, add a thin overlap lip on the meeting stile that
    extends toward the interior (-Y) so the overlap reads visibly.

    Authored in the sash-local frame:
      - local X runs -SASH_W/2 .. +SASH_W/2
      - local Z runs 0 .. SASH_H (bottom rail at z=0)
      - local Y is the sash thickness, centered at y=0.
    """
    w, h, d = SASH_W, SASH_H, SASH_D
    lw = MTG_STILE if meeting_side == "left" else RAIL
    rw = MTG_STILE if meeting_side == "right" else RAIL

    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(w, d, h)
    )

    ix0, ix1 = -w / 2.0 + lw, w / 2.0 - rw
    iz0, iz1 = RAIL, h - RAIL
    iw = ix1 - ix0
    ih = iz1 - iz0

    col_lines = [ix0 + (i + 1) * iw / NCOLS for i in range(NCOLS - 1)]
    row_lines = [iz0 + (j + 1) * ih / NROWS for j in range(NROWS - 1)]
    xe = [ix0] + col_lines + [ix1]
    ze = [iz0] + row_lines + [iz1]
    hm = MUNTIN / 2.0

    sash = outer
    for ci in range(NCOLS):
        for ri in range(NROWS):
            x0 = xe[ci] + (hm if ci > 0 else 0.0)
            x1 = xe[ci + 1] - (hm if ci < NCOLS - 1 else 0.0)
            z0 = ze[ri] + (hm if ri > 0 else 0.0)
            z1 = ze[ri + 1] - (hm if ri < NROWS - 1 else 0.0)
            cut = (
                cq.Workplane("XY")
                .transformed(offset=((x0 + x1) / 2.0, 0.0, (z0 + z1) / 2.0))
                .box(x1 - x0, d + 0.02, z1 - z0)
            )
            sash = sash.cut(cut)

    # Overlap lip: thin fin on the meeting stile extending toward -Y
    if with_lip:
        sx = (w / 2.0 - MTG_STILE / 2.0) if meeting_side == "right" else (-w / 2.0 + MTG_STILE / 2.0)
        lip = (
            cq.Workplane("XY")
            .transformed(offset=(sx, -(d / 2.0 + LIP_T / 2.0), h / 2.0))
            .box(LIP_W, LIP_T, h - 2.0 * RAIL)
        )
        sash = sash.union(lip)

    return sash


def _sash_glass_shape(meeting_side: str) -> cq.Workplane:
    """Six thin glass panes filling the lite openings, rebated under the
    muntin/rail lips so the glass reads as captured."""
    w, h = SASH_W, SASH_H
    lw = MTG_STILE if meeting_side == "left" else RAIL
    rw = MTG_STILE if meeting_side == "right" else RAIL
    rebate = 0.005

    ix0, ix1 = -w / 2.0 + lw, w / 2.0 - rw
    iz0, iz1 = RAIL, h - RAIL
    iw = ix1 - ix0
    ih = iz1 - iz0

    col_lines = [ix0 + (i + 1) * iw / NCOLS for i in range(NCOLS - 1)]
    row_lines = [iz0 + (j + 1) * ih / NROWS for j in range(NROWS - 1)]
    xe = [ix0] + col_lines + [ix1]
    ze = [iz0] + row_lines + [iz1]
    hm = MUNTIN / 2.0

    panes = None
    for ci in range(NCOLS):
        for ri in range(NROWS):
            x0 = xe[ci] + (hm if ci > 0 else 0.0) - rebate
            x1 = xe[ci + 1] - (hm if ci < NCOLS - 1 else 0.0) + rebate
            z0 = ze[ri] + (hm if ri > 0 else 0.0) - rebate
            z1 = ze[ri + 1] - (hm if ri < NROWS - 1 else 0.0) + rebate
            pane = (
                cq.Workplane("XY")
                .transformed(offset=((x0 + x1) / 2.0, 0.0, (z0 + z1) / 2.0))
                .box(x1 - x0, GLASS_T, z1 - z0)
            )
            panes = pane if panes is None else panes.union(pane)
    return panes


# ---------------------------------------------------------------------------
# Screen frame geometry (CadQuery): thin perimeter ring
# ---------------------------------------------------------------------------

def _screen_frame_shape() -> cq.Workplane:
    """Thin perimeter frame for the insect screen."""
    w, h, d, r = SCR_FW, SCR_FH, SCR_FD, SCR_RAIL
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(w, d, h)
    )
    inner = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(w - 2.0 * r, d + 0.02, h - 2.0 * r)
    )
    return outer.cut(inner)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_window")

    model.material("frame", rgba=FRAME_RGBA)
    model.material("sash", rgba=SASH_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("scr_frame", rgba=SCR_FRAME_RGBA)
    model.material("scr_mesh", rgba=SCR_MESH_RGBA)
    model.material("roller", rgba=ROLLER_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_frame_shape(), "frame"),
        material="frame",
        name="frame_shell",
    )

    # --- Fixed sash (left, exterior +Y track) ---
    fixed = model.part("fixed_sash")
    fixed.visual(
        mesh_from_cadquery(_sash_frame_shape("right", with_lip=True), "fixed_sash_frame"),
        material="sash",
        name="fixed_sash_frame",
    )
    fixed.visual(
        mesh_from_cadquery(_sash_glass_shape("right"), "fixed_sash_glass"),
        material="glass",
        name="fixed_sash_glass",
    )

    # --- Sliding sash (right, interior -Y track) ---
    sliding = model.part("sliding_sash")
    sliding.visual(
        mesh_from_cadquery(_sash_frame_shape("left"), "sliding_sash_frame"),
        material="sash",
        name="sliding_sash_frame",
    )
    sliding.visual(
        mesh_from_cadquery(_sash_glass_shape("left"), "sliding_sash_glass"),
        material="glass",
        name="sliding_sash_glass",
    )

    # Roller blocks at the bottom of the sliding sash
    roller_z = -ROLLER_SZ[2] / 2.0   # half below sash bottom (local z=0)
    for i, rx in enumerate((-SASH_W / 2.0 + 0.050, SASH_W / 2.0 - 0.050)):
        sliding.visual(
            Box(ROLLER_SZ),
            origin=Origin(xyz=(rx, 0.0, roller_z)),
            material="roller",
            name=f"roller_{i}",
        )

    # --- Insect screen (interior-most track) ---
    screen = model.part("screen")
    screen.visual(
        mesh_from_cadquery(_screen_frame_shape(), "screen_frame"),
        material="scr_frame",
        name="screen_frame",
    )

    # Screen mesh: thin panel in the X-Z plane (vertical), centered in frame
    mesh_w = SCR_FW - 2.0 * SCR_RAIL
    mesh_h = SCR_FH - 2.0 * SCR_RAIL
    screen.visual(
        Box((mesh_w, SCR_T, mesh_h)),
        origin=Origin(xyz=(0.0, 0.0, SCR_FH / 2.0)),
        material="scr_mesh",
        name="screen_mesh",
    )

    # ----- Articulations -----

    # Fixed sash: rigid mount to frame
    model.articulation(
        "frame_to_fixed",
        ArticulationType.FIXED,
        parent="frame",
        child="fixed_sash",
        origin=Origin(xyz=(FIXED_X, FIXED_Y, OPEN_Z0 + 0.004)),
    )

    # Sliding sash: prismatic along +X. q=0 is the partially-open rest pose.
    # Positive q slides it further open (to the right).
    model.articulation(
        "frame_to_sliding",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="sliding_sash",
        origin=Origin(xyz=(SLIDE_XR, SLIDE_Y, OPEN_Z0 + 0.004)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.30,
            lower=-PARTIAL,       # negative q closes (back to closed pose)
            upper=0.25,           # positive q opens further
        ),
    )

    # Insect screen: prismatic along +X, independent track
    model.articulation(
        "frame_to_screen",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="screen",
        origin=Origin(xyz=(0.0, SCREEN_Y, SCREEN_ORIGIN_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.40,
            lower=0.0,
            upper=OPEN_W * 0.75,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    fixed = object_model.get_part("fixed_sash")
    sliding = object_model.get_part("sliding_sash")
    screen = object_model.get_part("screen")
    j_slide = object_model.get_articulation("frame_to_sliding")
    j_screen = object_model.get_articulation("frame_to_screen")

    # --- Intentional overlaps ---
    # Glass rebated under sash rails/muntins
    for sn in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            sn, sn,
            elem_a=f"{sn}_glass",
            elem_b=f"{sn}_frame",
            reason="Glass panes are rebated under the sash rails/muntins so they read as captured.",
        )

    # Sashes ride in frame track grooves
    ctx.allow_overlap(
        "frame", "fixed_sash",
        reason="Fixed sash stiles are retained in the exterior jamb track grooves.",
    )
    ctx.allow_overlap(
        "frame", "sliding_sash",
        reason="Sliding sash stiles ride in the interior track grooves; sash stacks behind the frame jamb at full travel.",
    )

    # Screen rides in screen track grooves
    ctx.allow_overlap(
        "frame", "screen",
        reason="Screen frame rides in the shallow screen track grooves.",
    )

    # Sashes overlap at the meeting stile (offset Y planes)
    ctx.allow_overlap(
        "fixed_sash", "sliding_sash",
        reason="Sashes overlap at the meeting stile; they ride in offset Y planes so the overlap stile reads visibly.",
    )

    # Roller blocks seated into the sliding sash bottom rail
    for rn in ("roller_0", "roller_1"):
        ctx.allow_overlap(
            "sliding_sash", "sliding_sash",
            elem_a=rn,
            elem_b="sliding_sash_frame",
            reason=f"{rn} is seated into the sliding sash bottom rail.",
        )

    # Screen mesh panel sits inside the screen frame opening
    ctx.allow_overlap(
        "screen", "screen",
        elem_a="screen_mesh",
        elem_b="screen_frame",
        reason="Screen mesh panel is captured inside the screen frame opening.",
    )

    # --- Rest pose (q=0): partially open, overlap stile visible ---
    with ctx.pose({j_slide: 0.0}):
        fa = ctx.part_world_aabb(frame)
        fxa = ctx.part_world_aabb(fixed)
        sa = ctx.part_world_aabb(sliding)

        # Frame is wider than a single sash
        frame_w = fa[1][0] - fa[0][0]
        sash_w = sa[1][0] - sa[0][0]
        ctx.check(
            "frame wider than sash",
            frame_w > sash_w + 0.05,
            details=f"frame_w={frame_w:.3f}, sash_w={sash_w:.3f}",
        )

        # Sill near z=0
        ctx.check(
            "sill near z=0",
            abs(fa[0][2]) < 0.01 and fa[1][2] > 0.7,
            details=f"frame z=({fa[0][2]:.3f},{fa[1][2]:.3f})",
        )

        # Sashes side by side: fixed left, sliding right
        fcx = (fxa[0][0] + fxa[1][0]) / 2.0
        scx = (sa[0][0] + sa[1][0]) / 2.0
        ctx.check(
            "fixed sash left of sliding sash",
            fcx < scx - 0.10,
            details=f"fixed_cx={fcx:.3f}, slide_cx={scx:.3f}",
        )

        # Overlap at meeting stile (partially open but still overlapping)
        ctx.check(
            "overlap stile visible at rest",
            fxa[1][0] >= sa[0][0] - 0.001,
            details=f"fixed_right={fxa[1][0]:.4f}, slide_left={sa[0][0]:.4f}",
        )

        # Sashes ride in offset Y planes
        fcy = (fxa[0][1] + fxa[1][1]) / 2.0
        scy = (sa[0][1] + sa[1][1]) / 2.0
        ctx.check(
            "sashes in offset Y planes",
            abs(fcy - scy) > 0.015,
            details=f"fixed_cy={fcy:.3f}, slide_cy={scy:.3f}",
        )

        rest_scx = scx
        rest_fixed_right = fxa[1][0]
        rest_slide_left = sa[0][0]

    # --- Sliding sash opens further (positive q) ---
    with ctx.pose({j_slide: 0.20}):
        op = ctx.part_world_aabb(sliding)
        op_scx = (op[0][0] + op[1][0]) / 2.0
        ctx.check(
            "sliding sash moves right when opened",
            op_scx > rest_scx + 0.15,
            details=f"rest_cx={rest_scx:.3f}, open_cx={op_scx:.3f}",
        )
        # Retained in frame height
        ctx.expect_overlap(
            sliding, frame, axes="z", min_overlap=0.05,
            name="sliding sash retained in frame height when open",
        )

    # --- Sliding sash closes (negative q = -PARTIAL) ---
    with ctx.pose({j_slide: -PARTIAL}):
        cl = ctx.part_world_aabb(sliding)
        cl_fx = ctx.part_world_aabb(fixed)
        # At closed, overlap is larger than at rest
        cl_overlap = cl_fx[1][0] - cl[0][0]
        ctx.check(
            "sashes overlap more when fully closed",
            cl_overlap > (rest_fixed_right - rest_slide_left) + 0.02,
            details=f"closed_overlap={cl_overlap:.4f}, rest_overlap={rest_fixed_right - rest_slide_left:.4f}",
        )

    # --- Screen slides independently ---
    with ctx.pose({j_screen: 0.0}):
        rest_scr = ctx.part_world_aabb(screen)
        rest_scr_cx = (rest_scr[0][0] + rest_scr[1][0]) / 2.0

    with ctx.pose({j_screen: 0.40}):
        op_scr = ctx.part_world_aabb(screen)
        op_scr_cx = (op_scr[0][0] + op_scr[1][0]) / 2.0
        ctx.check(
            "screen slides right independently",
            op_scr_cx > rest_scr_cx + 0.30,
            details=f"rest_scr_cx={rest_scr_cx:.3f}, open_scr_cx={op_scr_cx:.3f}",
        )

    # --- Roller blocks exist and are positioned at sash bottom ---
    r0 = ctx.part_element_world_aabb(sliding, elem="roller_0")
    r1 = ctx.part_element_world_aabb(sliding, elem="roller_1")
    ctx.check("roller_0 exists", r0 is not None)
    ctx.check("roller_1 exists", r1 is not None)
    if r0 is not None and r1 is not None:
        # Rollers are near the sash bottom (low Z)
        ctx.check(
            "rollers near sash bottom",
            r0[0][2] < 0.20 and r1[0][2] < 0.20,
            details=f"r0_z_min={r0[0][2]:.3f}, r1_z_min={r1[0][2]:.3f}",
        )
        # Rollers are separated in X (one near each end of the sash)
        r0_cx = (r0[0][0] + r0[1][0]) / 2.0
        r1_cx = (r1[0][0] + r1[1][0]) / 2.0
        ctx.check(
            "rollers separated in X",
            abs(r1_cx - r0_cx) > 0.30,
            details=f"r0_cx={r0_cx:.3f}, r1_cx={r1_cx:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
