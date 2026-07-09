from __future__ import annotations

# Two-panel horizontal sliding window (patio-slider style), white vinyl frame
# with a chunky/thick profile. One FIXED sash (left) + one SLIDING sash (right).
# Large clear glass, no grille. A small metal cam-latch handle is mounted on the
# sliding sash's meeting stile.
#
# Coordinate convention (per brief):
#   +Z is up. Window stands vertically.
#     - width  -> X
#     - height -> Z   (sill near z=0)
#     - frame depth / glazing thickness / slide-normal -> Y
#   Glass plane is the X-Z plane. q=0 reads SHUT. Driving the prismatic joint
#   slides the right sash sideways toward the fixed left sash (-X) to open,
#   staying retained in the head/sill track.
#
# Structure:
#   - frame (static root): head, sill, two jambs + the deep box profile, built
#     as one CadQuery solid: a thick slab cut by the two sash openings, leaving
#     a true hollow perimeter + center mullion track.
#   - fixed_sash (left, FIXED): vinyl sash ring + clear glass, seated in the rear
#     glazing plane.
#   - sliding_sash (right, PRISMATIC): vinyl sash ring + clear glass, sitting
#     proud (front) so it can pass in front of the fixed sash; carries the latch.
#   - The latch is a small metal keeper plate + lever on the sliding sash's
#     meeting (inner) stile, mounted on the sash (a real part, not floating).

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
TOTAL_H = 1.72            # overall height along Z (sill at z=0, head at z=TOTAL_H)

FRAME_FACE = 0.085        # outer frame member face width (chunky vinyl)
FRAME_DEPTH = 0.140       # deep box section along Y (thick patio-slider profile)

# Two equal sash openings split by a small center meeting region. There is no
# real mullion bar in a 2-lite slider; the two sashes overlap at a meeting
# stile. We size two openings that together fill the inner clear width with a
# small central overlap allowance.
MEETING_OVERLAP = 0.040   # the two sash stiles overlap by this much at center

SASH_FACE = 0.075         # sash perimeter rail/stile face width (chunky)
SASH_DEPTH = 0.060        # sash depth along Y
GLASS_T = 0.008           # glazing thickness along Y

# Y layout: frame box centered on y=0. Fixed sash in the rear glazing plane;
# sliding sash sits proud toward +Y so it passes in front of the fixed sash.
FIXED_SASH_Y = -0.028     # rear glazing plane center (Y)
SLIDE_SASH_Y = 0.044      # sliding sash proud toward +Y (front track)
# Fixed sash front face  = FIXED_SASH_Y + SASH_DEPTH/2 = 0.002
# Sliding sash back face = SLIDE_SASH_Y - SASH_DEPTH/2 = 0.014
# -> ~12 mm air gap in Y; the proud sash glides in front of the fixed sash
#    so the two sash rings never interpenetrate (they only overlap in the
#    projected X footprint at the meeting stile, as a real slider does).

REBATE = 0.005            # glass tucks under the sash lip by this much

# Latch (cam lock) hardware
LATCH_PLATE_W = 0.028     # keeper plate face width (X)
LATCH_PLATE_H = 0.075     # keeper plate height (Z)
LATCH_PLATE_T = 0.010     # keeper plate thickness (Y, stands off the sash face)
LATCH_LEVER_LEN = 0.045   # lever arm length
LATCH_LEVER_R = 0.006     # lever arm radius

METAL_RGBA = (0.74, 0.76, 0.79, 1.0)   # brushed metal latch

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

# Each sash opening width: two openings that meet near the center with overlap.
# opening width = (inner_w + meeting_overlap) / 2  so the two stiles overlap.
SASH_OPENING_W = (INNER_W + MEETING_OVERLAP) / 2.0
SASH_OPENING_H = INNER_H

# Opening centers (world X) of each sash's clear glass region.
FIXED_OPEN_CX = INNER_X0 + SASH_OPENING_W / 2.0          # left
SLIDE_OPEN_CX = INNER_X1 - SASH_OPENING_W / 2.0          # right
MID_CZ = (INNER_Z0 + INNER_Z1) / 2.0

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)     # bright white vinyl/PVC
GLASS_RGBA = (0.52, 0.60, 0.66, 0.30)    # cool grey-blue, semi-transparent


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery), authored directly in meters.
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float, y_center: float, depth: float) -> cq.Workplane:
    """Axis-aligned box spanning [x0,x1] x [z0,z1] in the X-Z plane, centered on
    y_center with the given Y depth (local Y == world Y, local Z == world Z)."""
    return (
        cq.Workplane("XY")
        .transformed(offset=((x0 + x1) / 2.0, y_center, (z0 + z1) / 2.0))
        .box(x1 - x0, depth, z1 - z0)
    )


def _build_frame_shape() -> cq.Workplane:
    """Static outer frame: a thick slab cut by the two sash openings, leaving a
    true hollow perimeter (head, sill, jambs) plus the deep box section."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    cut_depth = FRAME_DEPTH + 0.02
    # One clear opening spanning the whole inner region (the two sashes sit in
    # front of / behind this single big opening; there is no fixed mullion bar
    # in a 2-lite slider).
    opening = _slab(INNER_X0, INNER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    return outer.cut(opening)


def _build_sash_shape() -> cq.Workplane:
    """One sash ring built in its OWN local frame, centered on local origin:
      - local X in [-out_w/2, out_w/2], out_w = opening + 2*SASH_FACE
      - local Z in [-out_h/2, out_h/2]
      - local Y is the sash depth, centered at 0
    A solid slab cut by the clear opening -> a true hollow sash ring.
    """
    ow = SASH_OPENING_W
    oh = SASH_OPENING_H
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SASH_DEPTH)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH + 0.02)
    return outer.cut(opening)


def _build_sash_glass_shape() -> cq.Workplane:
    """Single clear pane filling the sash opening (sash-local frame), rebated
    slightly under the sash lip so it reads captured."""
    ow = SASH_OPENING_W + 2 * REBATE
    oh = SASH_OPENING_H + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


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
    """Add the cam-latch hardware on the sliding sash's meeting (inner/left)
    stile, at mid-height. Authored in the sash-local frame: the meeting stile is
    at local x = -SASH_OPENING_W/2 - SASH_FACE/2. The keeper plate stands off the
    front sash face (+Y) and the lever arm sits on it -> a real mounted part."""
    sash = model.get_part(sash_name)

    stile_x = -SASH_OPENING_W / 2.0 - SASH_FACE / 2.0   # local center of meeting stile
    face_y = SASH_DEPTH / 2.0                            # front face of the sash
    plate_y = face_y + LATCH_PLATE_T / 2.0               # plate centered just proud of the face

    # Keeper plate seated on the stile front face.
    sash.visual(
        Box((LATCH_PLATE_W, LATCH_PLATE_T, LATCH_PLATE_H)),
        origin=Origin(xyz=(stile_x, plate_y, 0.0)),
        material="metal",
        name=f"{sash_name}_latch_plate",
    )
    # Lever arm: a short cylinder standing off the plate (the thumb-turn cam
    # lever), pointing along +Y then angled down -> model as a stub along +Y.
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
    model = ArticulatedObject(name="two_panel_sliding_window")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("metal", rgba=METAL_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="vinyl",
        name="frame_shell",
    )

    # --- Fixed (left) + sliding (right) sashes ---
    _add_sash(model, "fixed_sash")
    _add_sash(model, "sliding_sash")
    _add_latch(model, "sliding_sash")

    # FIXED left sash seated in the rear glazing plane.
    model.articulation(
        "frame_to_fixed_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="fixed_sash",
        origin=Origin(xyz=(FIXED_OPEN_CX, FIXED_SASH_Y, MID_CZ)),
    )

    # SLIDING right sash: PRISMATIC along X. Joint origin at the sash seated
    # (closed) center, proud of the fixed sash in +Y. The sash slides toward the
    # fixed (left) sash to open, so positive q must move it in -X. We choose
    # axis=(-1,0,0) so positive q OPENS (slides left). The sash keeps overlapping
    # the head/sill track and meeting region at full travel (retained insertion).
    slide_travel = SASH_OPENING_W * 0.90   # ~one sash width
    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="sliding_sash",
        origin=Origin(xyz=(SLIDE_OPEN_CX, SLIDE_SASH_Y, MID_CZ)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5, lower=0.0, upper=slide_travel),
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
    slide = object_model.get_articulation("frame_to_sliding_sash")

    # --- Intentional overlaps ---
    # Glass tucks under the vinyl sash lip on each sash (captured glass).
    for nm in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_vinyl",
            reason="Clear pane is rebated under the sash lip so it reads captured, not floating.",
        )
    # Each sash ring laps the frame opening edge (the glazing rebate / track that
    # captures the sash in the deep vinyl frame). Allow vinyl-frame + glass-frame.
    for nm in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_vinyl",
            reason=f"{nm} ring is rebated into the frame opening / head-sill track (seated capture).",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_glass",
            reason=f"{nm} glass is rebated under the frame opening lip (captured glazing).",
        )
    # The latch keeper plate is seated onto the sliding sash front stile face.
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="sliding_sash_latch_plate",
        elem_b="sliding_sash_vinyl",
        reason="Latch keeper plate is seated onto the sliding-sash meeting-stile face (mounted, not floating).",
    )

    # --- Closed pose (q=0): window reads SHUT ---
    with ctx.pose({slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        f_aabb = ctx.part_world_aabb(fixed_sash)
        s_aabb = ctx.part_world_aabb(sliding_sash)

        # Frame spans the full width and is wider than a single sash.
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        sash_w = s_aabb[1][0] - s_aabb[0][0]
        ctx.check(
            "frame spans wider than a single sash",
            frame_w > sash_w + 0.40,
            details=f"frame_w={frame_w:.3f}, sash_w={sash_w:.3f}",
        )
        # Sill near floor, head at full height.
        ctx.check(
            "sill sits near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"frame zmin={frame_aabb[0][2]:.4f}",
        )
        ctx.check(
            "head reaches full height",
            abs(frame_aabb[1][2] - TOTAL_H) < 0.02,
            details=f"frame zmax={frame_aabb[1][2]:.4f}",
        )
        # Two sashes side by side: fixed on the left, sliding on the right.
        fx = (f_aabb[0][0] + f_aabb[1][0]) / 2.0
        sx = (s_aabb[0][0] + s_aabb[1][0]) / 2.0
        ctx.check(
            "fixed sash left of sliding sash",
            fx < sx,
            details=f"fixed_x={fx:.3f}, sliding_x={sx:.3f}",
        )
        # Both sashes seated within the frame height.
        for nm, ab in (("fixed", f_aabb), ("sliding", s_aabb)):
            ctx.check(
                f"{nm} sash seated within frame height",
                ab[0][2] > frame_aabb[0][2] - 1e-4 and ab[1][2] < frame_aabb[1][2] + 1e-4,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}]",
            )
        # Sliding sash sits proud (in +Y) of the fixed sash.
        fy = (f_aabb[0][1] + f_aabb[1][1]) / 2.0
        sy = (s_aabb[0][1] + s_aabb[1][1]) / 2.0
        ctx.check(
            "sliding sash proud of fixed sash",
            sy > fy + 0.02,
            details=f"sliding_y={sy:.3f}, fixed_y={fy:.3f}",
        )
        # Both sashes are seated in the frame opening (rebate overlap proof).
        ctx.expect_overlap(
            fixed_sash, frame, axes="xz", min_overlap=0.03,
            name="fixed sash seated in frame opening",
        )
        ctx.expect_overlap(
            sliding_sash, frame, axes="xz", min_overlap=0.03,
            name="sliding sash seated in frame opening",
        )

        # Latch is on the sliding sash's meeting (inner/left) stile, mid-height,
        # standing off the front face.
        latch_aabb = ctx.part_element_world_aabb(sliding_sash, elem="sliding_sash_latch_plate")
        latch_cx = (latch_aabb[0][0] + latch_aabb[1][0]) / 2.0
        latch_cz = (latch_aabb[0][2] + latch_aabb[1][2]) / 2.0
        latch_cy = (latch_aabb[0][1] + latch_aabb[1][1]) / 2.0
        ctx.check(
            "latch on sliding sash inner (meeting) stile",
            latch_cx < sx,  # inner stile is on the left side of the right sash
            details=f"latch_x={latch_cx:.3f}, sliding_center_x={sx:.3f}",
        )
        ctx.check(
            "latch near mid-height",
            abs(latch_cz - MID_CZ) < 0.20,
            details=f"latch_z={latch_cz:.3f}, mid_z={MID_CZ:.3f}",
        )
        ctx.check(
            "latch stands off the front sash face",
            latch_cy > sy,
            details=f"latch_y={latch_cy:.3f}, sash_y={sy:.3f}",
        )

        rest_sx = sx
        rest_sz = (s_aabb[0][2] + s_aabb[1][2]) / 2.0

    # --- Driven/open pose: sliding sash slides toward fixed sash (-X) ---
    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel}):
        s_open = ctx.part_world_aabb(sliding_sash)
        open_sx = (s_open[0][0] + s_open[1][0]) / 2.0
        # Positive q moves the sash in -X (toward the fixed left sash) to open.
        ctx.check(
            "sliding sash opens toward fixed sash (-X)",
            abs((rest_sx - open_sx) - travel) < 0.02 and open_sx < rest_sx - 0.30,
            details=f"rest_x={rest_sx:.3f}, open_x={open_sx:.3f}, travel={travel:.3f}",
        )
        # Pure horizontal slide (no Z change).
        open_sz = (s_open[0][2] + s_open[1][2]) / 2.0
        ctx.check(
            "slide is purely horizontal",
            abs(open_sz - rest_sz) < 0.02,
            details=f"open_z={open_sz:.3f}, rest_z={rest_sz:.3f}",
        )
        # Retained insertion: sliding sash stays within the frame X span and
        # keeps engaging the head/sill track at full travel.
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained within frame X span at full travel",
            s_open[0][0] > f_aabb[0][0] - 1e-4 and s_open[1][0] < f_aabb[1][0] + 1e-4,
            details=f"sash x=[{s_open[0][0]:.3f},{s_open[1][0]:.3f}] frame x=[{f_aabb[0][0]:.3f},{f_aabb[1][0]:.3f}]",
        )
        ctx.expect_overlap(
            sliding_sash, frame, axes="z", min_overlap=0.10,
            name="sash retains vertical engagement with head/sill track",
        )

    return ctx.report()


object_model = build_object_model()
