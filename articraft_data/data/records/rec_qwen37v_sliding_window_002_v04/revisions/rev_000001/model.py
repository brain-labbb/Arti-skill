from __future__ import annotations

# Sliding window variant: two-panel horizontal sliding window with narrow
# transom panel above, white vinyl frame. One FIXED sash (left) + one
# SLIDING sash (right) on a prismatic joint. Sill lip with drainage slots.
# Two roller blocks at the bottom of the moving sash. Cam-latch handle on
# the sliding sash meeting stile.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     width  -> X,  height -> Z (sill near z=0),  depth -> Y
#   Glass plane is X-Z. q=0 reads SHUT. Driving the prismatic joint slides
#   the right sash toward the fixed left sash (-X) to open.
#
# Structure:
#   - frame (root): head, sill, jambs, transom bar + sill lip w/ drainage
#   - fixed_sash (left, FIXED): vinyl ring + clear glass
#   - sliding_sash (right, PRISMATIC): vinyl ring + glass + latch + rollers
#   - transom (FIXED): narrow glass panel above the sliding panes

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

TOTAL_W = 1.52            # overall window width along X
TOTAL_H = 1.72            # overall height along Z

FRAME_FACE = 0.085        # outer frame member face width
FRAME_DEPTH = 0.140       # deep box section along Y

MEETING_OVERLAP = 0.040   # the two sash stiles overlap at center

SASH_FACE = 0.075         # sash perimeter rail/stile face width
SASH_DEPTH = 0.060        # sash depth along Y
GLASS_T = 0.008           # glazing thickness

# Y layout: frame centered on y=0. Fixed sash rear; sliding sash proud (+Y).
FIXED_SASH_Y = -0.028
SLIDE_SASH_Y = 0.044

REBATE = 0.005            # glass tucks under sash lip

# Transom panel
TRANSOM_H = 0.20          # transom glass height
TRANSOM_BAR_H = 0.08      # horizontal divider bar between transom and main opening

# Sill lip and drainage slots
SILL_LIP_EXTEND = 0.030   # sill lip protrusion beyond frame front face (+Y)
SILL_LIP_H = 0.022        # sill lip height
DRAIN_W = 0.035           # drainage slot width
DRAIN_H = 0.010           # drainage slot height
DRAIN_COUNT = 3           # number of drainage slots

# Roller blocks
ROLLER_W = 0.025          # roller block width (X)
ROLLER_H = 0.012          # roller block height (Z)
ROLLER_D = 0.018          # roller block depth (Y)

# Latch hardware
LATCH_PLATE_W = 0.028
LATCH_PLATE_H = 0.075
LATCH_PLATE_T = 0.010
LATCH_LEVER_LEN = 0.045
LATCH_LEVER_R = 0.006

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)
GLASS_RGBA = (0.52, 0.60, 0.66, 0.30)
METAL_RGBA = (0.74, 0.76, 0.79, 1.0)
ROLLER_RGBA = (0.22, 0.22, 0.25, 1.0)   # dark nylon/metal roller

# ---------------------------------------------------------------------------
# Derived layout
# ---------------------------------------------------------------------------

HALF_W = TOTAL_W / 2.0
INNER_X0 = -HALF_W + FRAME_FACE
INNER_X1 = HALF_W - FRAME_FACE
INNER_Z0 = FRAME_FACE
INNER_Z1 = TOTAL_H - FRAME_FACE
INNER_W = INNER_X1 - INNER_X0

# Transom / main opening split
TRANSOM_BAR_Z0 = INNER_Z1 - TRANSOM_H - TRANSOM_BAR_H   # bottom of transom bar
TRANSOM_BAR_Z1 = TRANSOM_BAR_Z0 + TRANSOM_BAR_H          # top of transom bar
MAIN_OPEN_Z0 = INNER_Z0                                    # bottom of main sash opening
MAIN_OPEN_Z1 = TRANSOM_BAR_Z0                              # top of main sash opening
MAIN_OPEN_H = MAIN_OPEN_Z1 - MAIN_OPEN_Z0
TRANSOM_OPEN_Z0 = TRANSOM_BAR_Z1                           # bottom of transom opening
TRANSOM_OPEN_Z1 = INNER_Z1                                 # top of transom opening
TRANSOM_OPEN_H = TRANSOM_OPEN_Z1 - TRANSOM_OPEN_Z0
MAIN_MID_CZ = (MAIN_OPEN_Z0 + MAIN_OPEN_Z1) / 2.0
TRANSOM_CZ = (TRANSOM_OPEN_Z0 + TRANSOM_OPEN_Z1) / 2.0

# Sash opening dimensions (based on main opening)
SASH_OPENING_W = (INNER_W + MEETING_OVERLAP) / 2.0
SASH_OPENING_H = MAIN_OPEN_H
SASH_OUT_H = SASH_OPENING_H + 2 * SASH_FACE   # sash ring outer height

# Sash opening centers (world X)
FIXED_OPEN_CX = INNER_X0 + SASH_OPENING_W / 2.0
SLIDE_OPEN_CX = INNER_X1 - SASH_OPENING_W / 2.0

# Sill lip Y layout
SILL_LIP_Y0 = FRAME_DEPTH / 2.0
SILL_LIP_YC = SILL_LIP_Y0 + SILL_LIP_EXTEND / 2.0

# Roller positions in sash-local frame
ROLLER_LOCAL_Z = -SASH_OUT_H / 2.0               # centered at sash bottom face
ROLLER_X_OFFSET = SASH_OPENING_W / 3.0            # symmetric left/right

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
    """Outer frame: hollow slab cut by main sash opening + transom opening,
    with sill lip (protruding shelf) and drainage slot through-cuts."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    cut_depth = FRAME_DEPTH + 0.02

    # Main sash opening (lower, large)
    main_cut = _slab(INNER_X0, INNER_X1, MAIN_OPEN_Z0, MAIN_OPEN_Z1, 0.0, cut_depth)
    # Transom opening (upper, narrow)
    transom_cut = _slab(INNER_X0, INNER_X1, TRANSOM_OPEN_Z0, TRANSOM_OPEN_Z1, 0.0, cut_depth)
    frame = outer.cut(main_cut).cut(transom_cut)

    # Sill lip: protruding shelf at bottom-front (+Y), with slight overlap
    # into the frame body for a clean boolean union.
    lip_overlap = 0.006
    lip_yc = SILL_LIP_YC - lip_overlap / 2.0
    lip_depth = SILL_LIP_EXTEND + lip_overlap
    sill_lip = _slab(-HALF_W, HALF_W, 0.0, SILL_LIP_H, lip_yc, lip_depth)
    frame = frame.union(sill_lip)

    # Drainage slots: 3 rectangular through-cuts in the sill lip
    for i in range(DRAIN_COUNT):
        xp = INNER_X0 + (i + 0.5) * (INNER_W / DRAIN_COUNT)
        slot = _slab(
            xp - DRAIN_W / 2.0, xp + DRAIN_W / 2.0,
            SILL_LIP_H * 0.25, SILL_LIP_H * 0.75,
            lip_yc, lip_depth + 0.02,
        )
        frame = frame.cut(slot)

    return frame


def _build_sash_shape() -> cq.Workplane:
    """Hollow sash ring in its own local frame (centered at origin)."""
    ow = SASH_OPENING_W
    oh = SASH_OPENING_H
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SASH_DEPTH)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH + 0.02)
    return outer.cut(opening)


def _build_sash_glass_shape() -> cq.Workplane:
    """Clear pane rebated under the sash lip (sash-local frame)."""
    ow = SASH_OPENING_W + 2 * REBATE
    oh = SASH_OPENING_H + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_transom_glass_shape() -> cq.Workplane:
    """Fixed transom glass panel (transom-local frame, centered at origin).
    Sized to overlap into the frame rebate so it reads captured, not floating."""
    rebate_overlap = 0.003   # glass embeds into the frame groove by this much
    w = INNER_W + 2 * rebate_overlap
    h = TRANSOM_OPEN_H + 2 * rebate_overlap
    return _slab(-w / 2.0, w / 2.0, -h / 2.0, h / 2.0, 0.0, GLASS_T)


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


def _add_latch(model: ArticulatedObject, sash_name: str) -> None:
    """Cam-latch hardware on the sliding sash's meeting stile."""
    sash = model.get_part(sash_name)
    stile_x = -SASH_OPENING_W / 2.0 - SASH_FACE / 2.0
    face_y = SASH_DEPTH / 2.0
    plate_y = face_y + LATCH_PLATE_T / 2.0

    sash.visual(
        Box((LATCH_PLATE_W, LATCH_PLATE_T, LATCH_PLATE_H)),
        origin=Origin(xyz=(stile_x, plate_y, 0.0)),
        material="metal",
        name=f"{sash_name}_latch_plate",
    )
    lever_y = face_y + LATCH_PLATE_T + LATCH_LEVER_LEN / 2.0
    sash.visual(
        Cylinder(radius=LATCH_LEVER_R, length=LATCH_LEVER_LEN),
        origin=Origin(xyz=(stile_x, lever_y, -0.008), rpy=(1.5707963, 0.0, 0.0)),
        material="metal",
        name=f"{sash_name}_latch_lever",
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_window_with_transom")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("metal", rgba=METAL_RGBA)
    model.material("roller", rgba=ROLLER_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="vinyl",
        name="frame_shell",
    )

    # --- Fixed (left) + sliding (right) sashes ---
    _add_sash(model, "fixed_sash")
    _add_sash(model, "sliding_sash")
    _add_latch(model, "sliding_sash")

    # --- Roller blocks on sliding sash bottom ---
    sliding = model.get_part("sliding_sash")
    for i, sign in enumerate((-1, 1)):
        sliding.visual(
            Box((ROLLER_W, ROLLER_D, ROLLER_H)),
            origin=Origin(xyz=(sign * ROLLER_X_OFFSET, 0.0, ROLLER_LOCAL_Z)),
            material="roller",
            name=f"roller_{i}",
        )

    # --- Transom (fixed glass panel above sashes) ---
    transom = model.part("transom")
    transom.visual(
        mesh_from_cadquery(_build_transom_glass_shape(), "transom_glass"),
        material="glass",
        name="transom_glass",
    )

    # FIXED left sash seated in the rear glazing plane.
    model.articulation(
        "frame_to_fixed_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="fixed_sash",
        origin=Origin(xyz=(FIXED_OPEN_CX, FIXED_SASH_Y, MAIN_MID_CZ)),
    )

    # SLIDING right sash: PRISMATIC along X. Positive q opens (slides left).
    slide_travel = SASH_OPENING_W * 0.90
    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="sliding_sash",
        origin=Origin(xyz=(SLIDE_OPEN_CX, SLIDE_SASH_Y, MAIN_MID_CZ)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5, lower=0.0, upper=slide_travel),
    )

    # FIXED transom panel above.
    model.articulation(
        "frame_to_transom",
        ArticulationType.FIXED,
        parent="frame",
        child="transom",
        origin=Origin(xyz=(0.0, 0.0, TRANSOM_CZ)),
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
    transom = object_model.get_part("transom")
    slide = object_model.get_articulation("frame_to_sliding_sash")

    # --- Intentional overlaps ---
    # Glass rebated under sash lip
    for nm in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass", elem_b=f"{nm}_vinyl",
            reason="Clear pane is rebated under the sash lip (captured glazing).",
        )
    # Sash rings and glass rebated into frame tracks
    for nm in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell", elem_b=f"{nm}_vinyl",
            reason=f"{nm} ring is rebated into the frame opening / head-sill track.",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell", elem_b=f"{nm}_glass",
            reason=f"{nm} glass is rebated under the frame opening lip.",
        )
    # Latch seated on sash stile
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="sliding_sash_latch_plate", elem_b="sliding_sash_vinyl",
        reason="Latch keeper plate seated on the sliding-sash meeting stile.",
    )
    # Transom glass rebated into frame groove (captured fixed panel)
    ctx.allow_overlap(
        "frame", "transom",
        elem_a="frame_shell", elem_b="transom_glass",
        reason="Transom glass is rebated into the frame groove (captured fixed panel).",
    )

    # Roller blocks: half-embedded in sash bottom rail, riding in sill track
    for rn in ("roller_0", "roller_1"):
        ctx.allow_overlap(
            "sliding_sash", "sliding_sash",
            elem_a=rn, elem_b="sliding_sash_vinyl",
            reason=f"{rn} is mounted in the sliding sash bottom rail (half-embedded).",
        )
        ctx.allow_overlap(
            "frame", "sliding_sash",
            elem_a="frame_shell", elem_b=rn,
            reason=f"{rn} rides in the sill track, nested in the frame sill body.",
        )

    # --- Closed pose (q=0) ---
    with ctx.pose({slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        f_aabb = ctx.part_world_aabb(fixed_sash)
        s_aabb = ctx.part_world_aabb(sliding_sash)
        t_aabb = ctx.part_world_aabb(transom)

        # Frame spans
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        sash_w = s_aabb[1][0] - s_aabb[0][0]
        ctx.check(
            "frame wider than single sash",
            frame_w > sash_w + 0.40,
            details=f"frame_w={frame_w:.3f}, sash_w={sash_w:.3f}",
        )
        ctx.check(
            "sill near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"zmin={frame_aabb[0][2]:.4f}",
        )
        ctx.check(
            "head at full height",
            abs(frame_aabb[1][2] - TOTAL_H) < 0.02,
            details=f"zmax={frame_aabb[1][2]:.4f}",
        )

        # Fixed sash left, sliding sash right
        fx = (f_aabb[0][0] + f_aabb[1][0]) / 2.0
        sx = (s_aabb[0][0] + s_aabb[1][0]) / 2.0
        ctx.check(
            "fixed sash left of sliding sash",
            fx < sx,
            details=f"fixed_x={fx:.3f}, sliding_x={sx:.3f}",
        )

        # Sashes within frame height
        for nm, ab in (("fixed", f_aabb), ("sliding", s_aabb)):
            ctx.check(
                f"{nm} sash within frame height",
                ab[0][2] > frame_aabb[0][2] - 1e-4 and ab[1][2] < frame_aabb[1][2] + 1e-4,
                details=f"z=[{ab[0][2]:.3f},{ab[1][2]:.3f}]",
            )

        # Sliding sash proud of fixed sash (+Y)
        fy = (f_aabb[0][1] + f_aabb[1][1]) / 2.0
        sy = (s_aabb[0][1] + s_aabb[1][1]) / 2.0
        ctx.check(
            "sliding sash proud of fixed sash",
            sy > fy + 0.02,
            details=f"slide_y={sy:.3f}, fixed_y={fy:.3f}",
        )

        # Sashes seated in frame
        ctx.expect_overlap(
            fixed_sash, frame, axes="xz", min_overlap=0.03,
            name="fixed sash seated in frame opening",
        )
        ctx.expect_overlap(
            sliding_sash, frame, axes="xz", min_overlap=0.03,
            name="sliding sash seated in frame opening",
        )

        # --- Transom panel above main sash region ---
        ctx.check(
            "transom above main sash openings",
            t_aabb[0][2] > MAIN_OPEN_Z1 - 0.02,
            details=f"transom_zmin={t_aabb[0][2]:.4f}, main_top={MAIN_OPEN_Z1:.4f}",
        )
        ctx.check(
            "transom within frame height",
            t_aabb[1][2] < frame_aabb[1][2] + 1e-4,
            details=f"transom_zmax={t_aabb[1][2]:.4f}",
        )
        ctx.expect_overlap(
            transom, frame, axes="x", min_overlap=0.30,
            name="transom spans within frame width",
        )
        ctx.expect_contact(
            transom, frame,
            elem_a="transom_glass", elem_b="frame_shell",
            contact_tol=0.005,
            name="transom glass contacts frame rebate",
        )

        # --- Sill lip protrudes beyond frame front face ---
        ctx.check(
            "sill lip protrudes in +Y beyond frame front",
            frame_aabb[1][1] > FRAME_DEPTH / 2.0 + 0.010,
            details=f"frame_ymax={frame_aabb[1][1]:.4f}, expected>{FRAME_DEPTH / 2.0 + 0.010:.4f}",
        )

        # --- Roller blocks at sash bottom ---
        for rn in ("roller_0", "roller_1"):
            raabb = ctx.part_element_world_aabb(sliding_sash, elem=rn)
            ctx.check(
                f"{rn} at sash bottom",
                abs(raabb[1][2] - s_aabb[0][2]) < 0.020,
                details=f"roller_top={raabb[1][2]:.4f}, sash_bottom={s_aabb[0][2]:.4f}",
            )
            ctx.check(
                f"{rn} within frame X span",
                raabb[0][0] > frame_aabb[0][0] - 1e-4 and raabb[1][0] < frame_aabb[1][0] + 1e-4,
                details=f"roller_x=[{raabb[0][0]:.3f},{raabb[1][0]:.3f}]",
            )

        # Two roller blocks separated horizontally
        r0_aabb = ctx.part_element_world_aabb(sliding_sash, elem="roller_0")
        r1_aabb = ctx.part_element_world_aabb(sliding_sash, elem="roller_1")
        r0_cx = (r0_aabb[0][0] + r0_aabb[1][0]) / 2.0
        r1_cx = (r1_aabb[0][0] + r1_aabb[1][0]) / 2.0
        ctx.check(
            "rollers separated horizontally",
            abs(r1_cx - r0_cx) > 0.10,
            details=f"r0_x={r0_cx:.3f}, r1_x={r1_cx:.3f}",
        )

        # Latch checks
        latch_aabb = ctx.part_element_world_aabb(sliding_sash, elem="sliding_sash_latch_plate")
        latch_cx = (latch_aabb[0][0] + latch_aabb[1][0]) / 2.0
        latch_cz = (latch_aabb[0][2] + latch_aabb[1][2]) / 2.0
        latch_cy = (latch_aabb[0][1] + latch_aabb[1][1]) / 2.0
        ctx.check(
            "latch on inner stile",
            latch_cx < sx,
            details=f"latch_x={latch_cx:.3f}, sash_x={sx:.3f}",
        )
        ctx.check(
            "latch near mid-height",
            abs(latch_cz - MAIN_MID_CZ) < 0.20,
            details=f"latch_z={latch_cz:.3f}, mid_z={MAIN_MID_CZ:.3f}",
        )
        ctx.check(
            "latch proud of sash face",
            latch_cy > sy,
            details=f"latch_y={latch_cy:.3f}, sash_y={sy:.3f}",
        )

        rest_sx = sx
        rest_sz = (s_aabb[0][2] + s_aabb[1][2]) / 2.0

    # --- Open pose: sash slides in -X ---
    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel}):
        s_open = ctx.part_world_aabb(sliding_sash)
        open_sx = (s_open[0][0] + s_open[1][0]) / 2.0
        ctx.check(
            "sash opens toward fixed (-X)",
            abs((rest_sx - open_sx) - travel) < 0.02 and open_sx < rest_sx - 0.30,
            details=f"rest={rest_sx:.3f}, open={open_sx:.3f}, travel={travel:.3f}",
        )
        open_sz = (s_open[0][2] + s_open[1][2]) / 2.0
        ctx.check(
            "pure horizontal slide (no Z change)",
            abs(open_sz - rest_sz) < 0.02,
            details=f"open_z={open_sz:.3f}, rest_z={rest_sz:.3f}",
        )
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained within frame X at full travel",
            s_open[0][0] > f_aabb[0][0] - 1e-4 and s_open[1][0] < f_aabb[1][0] + 1e-4,
            details=f"sash_x=[{s_open[0][0]:.3f},{s_open[1][0]:.3f}]",
        )
        ctx.expect_overlap(
            sliding_sash, frame, axes="z", min_overlap=0.10,
            name="sash retains track engagement at full travel",
        )

    return ctx.report()


object_model = build_object_model()
