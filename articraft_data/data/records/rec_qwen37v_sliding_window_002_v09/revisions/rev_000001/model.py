from __future__ import annotations

# Sliding window variant: two-panel horizontal sliding window with insect screen
# panel in a separate exterior track, tilt-in latch pair on revolute joints, and
# deep track grooves along top/bottom rails. White vinyl frame.
#
# Coordinate convention:
#   +Z up, window vertical. +Y interior, -Y exterior.
#   width -> X, height -> Z, depth -> Y.
#   Sill near z=0, head at z=TOTAL_H.
#
# Structure:
#   frame (root): deep vinyl box with hollow opening + track groove rails
#     unioned into sill/head inner surfaces (visible deep grooves).
#   fixed_sash (FIXED, left): vinyl ring + glass in rear track.
#   sliding_sash (PRISMATIC, right): vinyl ring + glass + cam latch in front track.
#   insect_screen (FIXED): thin aluminum frame + mesh panel in exterior track.
#   tilt_latch_top (REVOLUTE): pivoting latch tab on screen top edge.
#   tilt_latch_bottom (REVOLUTE): pivoting latch tab on screen bottom edge.

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

FRAME_FACE = 0.085
FRAME_DEPTH = 0.170       # deepened to accommodate exterior screen track

MEETING_OVERLAP = 0.040

SASH_FACE = 0.075
SASH_DEPTH = 0.060
GLASS_T = 0.008

# Y layout: +Y interior, -Y exterior
SCREEN_Y = -0.068          # exterior screen track center
SCREEN_DEPTH = 0.015
FIXED_SASH_Y = -0.028      # rear glazing plane
SLIDE_SASH_Y = 0.044       # front track (proud)

REBATE = 0.005

# Screen frame
SCREEN_FRAME_FACE = 0.022
SCREEN_MESH_T = 0.002

# Tilt latch (on screen frame)
TILT_LATCH_W = 0.035
TILT_LATCH_T = 0.006
TILT_LATCH_TAB_LEN = 0.030

# Track groove rails
GROOVE_RAIL_W = 0.006
GROOVE_RAIL_H = 0.014

# Cam latch (on sliding sash)
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
SCREEN_RGBA = (0.35, 0.38, 0.40, 0.55)       # dark semi-transparent mesh
SCREEN_FRAME_RGBA = (0.86, 0.87, 0.88, 1.0)   # light aluminum screen frame

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

FIXED_OPEN_CX = INNER_X0 + SASH_OPENING_W / 2.0
SLIDE_OPEN_CX = INNER_X1 - SASH_OPENING_W / 2.0
MID_CZ = (INNER_Z0 + INNER_Z1) / 2.0

SCREEN_HALF_H = (SASH_OPENING_H + 2 * SCREEN_FRAME_FACE) / 2.0


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery)
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float, y_center: float, depth: float) -> cq.Workplane:
    """Axis-aligned box spanning [x0,x1]x[z0,z1], centered on y_center."""
    return (
        cq.Workplane("XY")
        .transformed(offset=((x0 + x1) / 2.0, y_center, (z0 + z1) / 2.0))
        .box(x1 - x0, depth, z1 - z0)
    )


def _groove_rail_params():
    """Return slab params for all track groove rails (sill + head)."""
    rails = []
    sill_z0, sill_z1 = FRAME_FACE, FRAME_FACE + GROOVE_RAIL_H
    head_z0, head_z1 = TOTAL_H - FRAME_FACE - GROOVE_RAIL_H, TOTAL_H - FRAME_FACE

    sash_lo = SLIDE_SASH_Y - SASH_DEPTH / 2.0
    sash_hi = SLIDE_SASH_Y + SASH_DEPTH / 2.0
    scr_lo = SCREEN_Y - SCREEN_DEPTH / 2.0
    scr_hi = SCREEN_Y + SCREEN_DEPTH / 2.0

    for z0, z1 in [(sill_z0, sill_z1), (head_z0, head_z1)]:
        # Sash track walls
        rails.append((INNER_X0, INNER_X1, z0, z1,
                       sash_lo - GROOVE_RAIL_W / 2.0, GROOVE_RAIL_W))
        rails.append((INNER_X0, INNER_X1, z0, z1,
                       sash_hi + GROOVE_RAIL_W / 2.0, GROOVE_RAIL_W))
        # Screen track walls
        rails.append((INNER_X0, INNER_X1, z0, z1,
                       scr_lo - GROOVE_RAIL_W / 2.0, GROOVE_RAIL_W))
        rails.append((INNER_X0, INNER_X1, z0, z1,
                       scr_hi + GROOVE_RAIL_W / 2.0, GROOVE_RAIL_W))
    return rails


def _build_frame_shape() -> cq.Workplane:
    """Static outer frame with hollow opening + unioned track groove rails."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    cut_depth = FRAME_DEPTH + 0.02
    opening = _slab(INNER_X0, INNER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    frame = outer.cut(opening)
    for params in _groove_rail_params():
        frame = frame.union(_slab(*params))
    return frame


def _build_sash_shape() -> cq.Workplane:
    ow, oh = SASH_OPENING_W, SASH_OPENING_H
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SASH_DEPTH)
    inner = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH + 0.02)
    return outer.cut(inner)


def _build_sash_glass_shape() -> cq.Workplane:
    ow = SASH_OPENING_W + 2 * REBATE
    oh = SASH_OPENING_H + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_screen_frame_shape() -> cq.Workplane:
    ow, oh = SASH_OPENING_W, SASH_OPENING_H
    out_w = ow + 2 * SCREEN_FRAME_FACE
    out_h = oh + 2 * SCREEN_FRAME_FACE
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SCREEN_DEPTH)
    inner = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SCREEN_DEPTH + 0.01)
    return outer.cut(inner)


def _build_screen_mesh_shape() -> cq.Workplane:
    # Rebated slightly under the screen frame lip (like captured glazing)
    w = SASH_OPENING_W + 2 * REBATE
    h = SASH_OPENING_H + 2 * REBATE
    return _slab(-w / 2.0, w / 2.0, -h / 2.0, h / 2.0, 0.0, SCREEN_MESH_T)


# ---------------------------------------------------------------------------
# Part builders
# ---------------------------------------------------------------------------

def _add_sash(model: ArticulatedObject, name: str) -> None:
    sash = model.part(name)
    sash.visual(
        mesh_from_cadquery(_build_sash_shape(), f"{name}_vinyl"),
        material="vinyl", name=f"{name}_vinyl",
    )
    sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), f"{name}_glass"),
        material="glass", name=f"{name}_glass",
    )


def _add_cam_latch(model: ArticulatedObject, sash_name: str) -> None:
    sash = model.get_part(sash_name)
    stile_x = -SASH_OPENING_W / 2.0 - SASH_FACE / 2.0
    face_y = SASH_DEPTH / 2.0
    plate_y = face_y + LATCH_PLATE_T / 2.0
    sash.visual(
        Box((LATCH_PLATE_W, LATCH_PLATE_T, LATCH_PLATE_H)),
        origin=Origin(xyz=(stile_x, plate_y, 0.0)),
        material="metal", name=f"{sash_name}_latch_plate",
    )
    lever_y = face_y + LATCH_PLATE_T + LATCH_LEVER_LEN / 2.0
    sash.visual(
        Cylinder(radius=LATCH_LEVER_R, length=LATCH_LEVER_LEN),
        origin=Origin(xyz=(stile_x, lever_y, -0.008), rpy=(1.5707963, 0.0, 0.0)),
        material="metal", name=f"{sash_name}_latch_lever",
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_window_with_screen")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("metal", rgba=METAL_RGBA)
    model.material("screen_mesh", rgba=SCREEN_RGBA)
    model.material("screen_frame", rgba=SCREEN_FRAME_RGBA)

    # --- Static frame (root) with track grooves ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="vinyl", name="frame_shell",
    )

    # --- Fixed + sliding sashes ---
    _add_sash(model, "fixed_sash")
    _add_sash(model, "sliding_sash")
    _add_cam_latch(model, "sliding_sash")

    # --- Insect screen panel in exterior track ---
    screen = model.part("insect_screen")
    screen.visual(
        mesh_from_cadquery(_build_screen_frame_shape(), "screen_frame_shell"),
        material="screen_frame", name="screen_frame_shell",
    )
    screen.visual(
        mesh_from_cadquery(_build_screen_mesh_shape(), "screen_mesh"),
        material="screen_mesh", name="screen_mesh_panel",
    )

    # --- Tilt-in latch pair (on screen frame) ---
    top_latch = model.part("tilt_latch_top")
    top_latch.visual(
        Box((TILT_LATCH_W, TILT_LATCH_T, TILT_LATCH_TAB_LEN)),
        origin=Origin(xyz=(0.0, 0.0, -TILT_LATCH_TAB_LEN / 2.0)),
        material="metal", name="latch_tab",
    )

    bottom_latch = model.part("tilt_latch_bottom")
    bottom_latch.visual(
        Box((TILT_LATCH_W, TILT_LATCH_T, TILT_LATCH_TAB_LEN)),
        origin=Origin(xyz=(0.0, 0.0, TILT_LATCH_TAB_LEN / 2.0)),
        material="metal", name="latch_tab",
    )

    # --- Articulations ---

    # Fixed sash
    model.articulation(
        "frame_to_fixed_sash", ArticulationType.FIXED,
        parent="frame", child="fixed_sash",
        origin=Origin(xyz=(FIXED_OPEN_CX, FIXED_SASH_Y, MID_CZ)),
    )

    # Sliding sash: prismatic along -X to open
    slide_travel = SASH_OPENING_W * 0.90
    model.articulation(
        "frame_to_sliding_sash", ArticulationType.PRISMATIC,
        parent="frame", child="sliding_sash",
        origin=Origin(xyz=(SLIDE_OPEN_CX, SLIDE_SASH_Y, MID_CZ)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5,
                                   lower=0.0, upper=slide_travel),
    )

    # Screen: fixed in exterior track
    model.articulation(
        "frame_to_screen", ArticulationType.FIXED,
        parent="frame", child="insect_screen",
        origin=Origin(xyz=(SLIDE_OPEN_CX, SCREEN_Y, MID_CZ)),
    )

    # Tilt latch top: pivot on screen frame top rail interior face.
    # Tab hangs down from the pivot, contacting the frame rail.
    # Revolute around X: positive q tilts tab toward +Y (interior).
    latch_top_pivot_z = SCREEN_HALF_H - SCREEN_FRAME_FACE / 2.0
    model.articulation(
        "screen_to_tilt_latch_top", ArticulationType.REVOLUTE,
        parent="insect_screen", child="tilt_latch_top",
        origin=Origin(xyz=(0.0, SCREEN_DEPTH / 2.0, latch_top_pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0,
                                   lower=0.0, upper=1.2),
    )

    # Tilt latch bottom: pivot on screen frame bottom rail interior face.
    # Tab extends up from the pivot, contacting the frame rail.
    # Revolute around -X: positive q tilts tab toward +Y (interior).
    latch_bot_pivot_z = -(SCREEN_HALF_H - SCREEN_FRAME_FACE / 2.0)
    model.articulation(
        "screen_to_tilt_latch_bottom", ArticulationType.REVOLUTE,
        parent="insect_screen", child="tilt_latch_bottom",
        origin=Origin(xyz=(0.0, SCREEN_DEPTH / 2.0, latch_bot_pivot_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0,
                                   lower=0.0, upper=1.2),
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
    screen = object_model.get_part("insect_screen")
    top_latch = object_model.get_part("tilt_latch_top")
    bottom_latch = object_model.get_part("tilt_latch_bottom")

    slide = object_model.get_articulation("frame_to_sliding_sash")
    latch_top_j = object_model.get_articulation("screen_to_tilt_latch_top")
    latch_bot_j = object_model.get_articulation("screen_to_tilt_latch_bottom")

    # ---- Overlap allowances ----

    # Glass rebated under sash lips
    for nm in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            nm, nm, elem_a=f"{nm}_glass", elem_b=f"{nm}_vinyl",
            reason="Clear pane rebated under the sash lip (captured glazing).",
        )
    # Sashes rebated into frame tracks
    for nm in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            "frame", nm, elem_a="frame_shell", elem_b=f"{nm}_vinyl",
            reason=f"{nm} ring rebated into frame head/sill track (seated capture).",
        )
        ctx.allow_overlap(
            "frame", nm, elem_a="frame_shell", elem_b=f"{nm}_glass",
            reason=f"{nm} glass rebated under frame lip (captured glazing).",
        )
    # Cam latch plate seated on sliding sash stile
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="sliding_sash_latch_plate", elem_b="sliding_sash_vinyl",
        reason="Cam latch keeper plate seated on sliding-sash meeting stile face.",
    )
    # Screen frame rebated into frame track groove
    ctx.allow_overlap(
        "frame", "insect_screen",
        elem_a="frame_shell", elem_b="screen_frame_shell",
        reason="Screen frame is seated in the exterior track groove of the frame.",
    )
    ctx.allow_overlap(
        "frame", "insect_screen",
        elem_a="frame_shell", elem_b="screen_mesh_panel",
        reason="Screen mesh is within the frame track region.",
    )
    # Screen mesh captured in screen frame
    ctx.allow_overlap(
        "insect_screen", "insect_screen",
        elem_a="screen_mesh_panel", elem_b="screen_frame_shell",
        reason="Screen mesh panel is captured inside the screen frame border.",
    )
    # Tilt latches mounted on screen frame (tabs proud of face)
    for latch_name in ("tilt_latch_top", "tilt_latch_bottom"):
        ctx.allow_overlap(
            "insect_screen", latch_name,
            elem_a="screen_frame_shell", elem_b="latch_tab",
            reason=f"{latch_name} tab is mounted proud of the screen frame interior face.",
        )
        # Latch tabs sit in the frame track region (screen is seated in the
        # frame, so tabs naturally overlap with frame groove rails).
        ctx.allow_overlap(
            "frame", latch_name,
            elem_a="frame_shell", elem_b="latch_tab",
            reason=f"{latch_name} tab overlaps frame groove rail region because screen is seated in the frame track.",
        )

    # ---- Rest-pose (q=0) checks ----

    with ctx.pose({slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        f_aabb = ctx.part_world_aabb(fixed_sash)
        s_aabb = ctx.part_world_aabb(sliding_sash)

        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        sash_w = s_aabb[1][0] - s_aabb[0][0]
        ctx.check("frame wider than single sash", frame_w > sash_w + 0.40,
                  details=f"frame_w={frame_w:.3f}, sash_w={sash_w:.3f}")
        ctx.check("sill near z=0", abs(frame_aabb[0][2]) < 0.02,
                  details=f"zmin={frame_aabb[0][2]:.4f}")
        ctx.check("head at full height", abs(frame_aabb[1][2] - TOTAL_H) < 0.02,
                  details=f"zmax={frame_aabb[1][2]:.4f}")

        fx = (f_aabb[0][0] + f_aabb[1][0]) / 2.0
        sx = (s_aabb[0][0] + s_aabb[1][0]) / 2.0
        ctx.check("fixed sash left of sliding sash", fx < sx,
                  details=f"fixed_x={fx:.3f}, sliding_x={sx:.3f}")

        fy = (f_aabb[0][1] + f_aabb[1][1]) / 2.0
        sy = (s_aabb[0][1] + s_aabb[1][1]) / 2.0
        ctx.check("sliding sash proud of fixed sash", sy > fy + 0.02,
                  details=f"sliding_y={sy:.3f}, fixed_y={fy:.3f}")

        ctx.expect_overlap(fixed_sash, frame, axes="xz", min_overlap=0.03,
                           name="fixed sash seated in frame opening")
        ctx.expect_overlap(sliding_sash, frame, axes="xz", min_overlap=0.03,
                           name="sliding sash seated in frame opening")

        # Latch checks
        latch_aabb = ctx.part_element_world_aabb(sliding_sash, elem="sliding_sash_latch_plate")
        latch_cx = (latch_aabb[0][0] + latch_aabb[1][0]) / 2.0
        latch_cz = (latch_aabb[0][2] + latch_aabb[1][2]) / 2.0
        latch_cy = (latch_aabb[0][1] + latch_aabb[1][1]) / 2.0
        ctx.check("latch on inner stile", latch_cx < sx,
                  details=f"latch_x={latch_cx:.3f}, sash_x={sx:.3f}")
        ctx.check("latch near mid-height", abs(latch_cz - MID_CZ) < 0.20,
                  details=f"latch_z={latch_cz:.3f}, mid_z={MID_CZ:.3f}")
        ctx.check("latch proud of sash face", latch_cy > sy,
                  details=f"latch_y={latch_cy:.3f}, sash_y={sy:.3f}")

        rest_sx = sx
        rest_sz = (s_aabb[0][2] + s_aabb[1][2]) / 2.0

    # ---- Sliding sash open pose ----

    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel}):
        s_open = ctx.part_world_aabb(sliding_sash)
        open_sx = (s_open[0][0] + s_open[1][0]) / 2.0
        ctx.check("sash opens toward fixed (-X)",
                  abs((rest_sx - open_sx) - travel) < 0.02 and open_sx < rest_sx - 0.30,
                  details=f"rest={rest_sx:.3f}, open={open_sx:.3f}, travel={travel:.3f}")
        open_sz = (s_open[0][2] + s_open[1][2]) / 2.0
        ctx.check("slide purely horizontal", abs(open_sz - rest_sz) < 0.02,
                  details=f"open_z={open_sz:.3f}, rest_z={rest_sz:.3f}")
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check("sash retained in frame at full travel",
                  s_open[0][0] > f_aabb[0][0] - 1e-4 and s_open[1][0] < f_aabb[1][0] + 1e-4,
                  details=f"sash x=[{s_open[0][0]:.3f},{s_open[1][0]:.3f}]")
        ctx.expect_overlap(sliding_sash, frame, axes="z", min_overlap=0.10,
                           name="sash retains head/sill track engagement")

    # ---- Screen panel checks ----

    scr_aabb = ctx.part_world_aabb(screen)
    scr_cy = (scr_aabb[0][1] + scr_aabb[1][1]) / 2.0
    scr_w = scr_aabb[1][0] - scr_aabb[0][0]
    scr_h = scr_aabb[1][2] - scr_aabb[0][2]

    ctx.check("screen in exterior track (more -Y than fixed sash)",
              scr_cy < fy - 0.005,
              details=f"screen_y={scr_cy:.3f}, fixed_y={fy:.3f}")
    ctx.check("screen within frame depth",
              scr_aabb[0][1] > frame_aabb[0][1] - 0.01
              and scr_aabb[1][1] < frame_aabb[1][1] + 0.01,
              details=f"screen y=[{scr_aabb[0][1]:.3f},{scr_aabb[1][1]:.3f}]")
    ctx.check("screen has meaningful width",
              scr_w > 0.30,
              details=f"screen_w={scr_w:.3f}")
    ctx.check("screen has meaningful height",
              scr_h > 0.50,
              details=f"screen_h={scr_h:.3f}")

    # ---- Tilt latch existence and articulation ----

    ctx.check("tilt_latch_top has revolute joint",
              latch_top_j.articulation_type == ArticulationType.REVOLUTE,
              details=f"type={latch_top_j.articulation_type}")
    ctx.check("tilt_latch_bottom has revolute joint",
              latch_bot_j.articulation_type == ArticulationType.REVOLUTE,
              details=f"type={latch_bot_j.articulation_type}")

    # Latch top: positive q tilts tab toward +Y (interior).
    with ctx.pose({latch_top_j: 0.0}):
        top_rest_y = (ctx.part_world_aabb(top_latch)[0][1]
                      + ctx.part_world_aabb(top_latch)[1][1]) / 2.0
    with ctx.pose({latch_top_j: 0.8}):
        top_tilt_y = (ctx.part_world_aabb(top_latch)[0][1]
                      + ctx.part_world_aabb(top_latch)[1][1]) / 2.0
    ctx.check("top latch tilts toward interior (+Y)",
              top_tilt_y > top_rest_y + 0.003,
              details=f"rest_y={top_rest_y:.4f}, tilt_y={top_tilt_y:.4f}")

    # Latch bottom: same direction.
    with ctx.pose({latch_bot_j: 0.0}):
        bot_rest_y = (ctx.part_world_aabb(bottom_latch)[0][1]
                      + ctx.part_world_aabb(bottom_latch)[1][1]) / 2.0
    with ctx.pose({latch_bot_j: 0.8}):
        bot_tilt_y = (ctx.part_world_aabb(bottom_latch)[0][1]
                      + ctx.part_world_aabb(bottom_latch)[1][1]) / 2.0
    ctx.check("bottom latch tilts toward interior (+Y)",
              bot_tilt_y > bot_rest_y + 0.003,
              details=f"rest_y={bot_rest_y:.4f}, tilt_y={bot_tilt_y:.4f}")

    # ---- Track groove depth check ----
    # The frame shape includes groove rails; confirm frame depth is real.
    ctx.check("frame has deep profile for track grooves",
              frame_aabb[1][1] - frame_aabb[0][1] > 0.12,
              details=f"frame_depth={frame_aabb[1][1] - frame_aabb[0][1]:.3f}")

    # ---- Tilt latch contact with screen frame (mounting proof) ----
    ctx.expect_contact(
        top_latch, screen,
        elem_a="latch_tab", elem_b="screen_frame_shell",
        name="top latch tab contacts screen frame (mounted)",
    )
    ctx.expect_contact(
        bottom_latch, screen,
        elem_a="latch_tab", elem_b="screen_frame_shell",
        name="bottom latch tab contacts screen frame (mounted)",
    )

    return ctx.report()


object_model = build_object_model()
