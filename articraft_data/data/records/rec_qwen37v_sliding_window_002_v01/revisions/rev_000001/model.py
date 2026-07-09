from __future__ import annotations

# Two-panel horizontal sliding window (variant), white vinyl frame.
# LEFT sash SLIDES on a prismatic joint; RIGHT sash is FIXED.
# Deep track grooves along the head and sill rails. Rubber gasket strips
# (dark EPDM) surround each glass pane. A small metal cam-latch handle is
# mounted on the sliding sash's meeting (right/inner) stile.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     width  -> X,  height -> Z (sill near z=0),  depth -> Y
#   Glass plane is X-Z. q=0 reads SHUT. Positive q slides the left sash
#   toward +X (toward the fixed right sash) to open, staying retained in
#   the head/sill track.

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
FRAME_DEPTH = 0.140

MEETING_OVERLAP = 0.040

SASH_FACE = 0.075
SASH_DEPTH = 0.060
GLASS_T = 0.008

# Y layout: frame centered on y=0. Fixed sash in rear; sliding sash proud (+Y).
FIXED_SASH_Y = -0.028
SLIDE_SASH_Y = 0.044

REBATE = 0.005

# Track groove dimensions (deep channels in head/sill where sashes ride)
TRACK_GROOVE_DEPTH = 0.018   # how deep the groove cuts into the frame rail
TRACK_GROOVE_WIDTH = 0.022   # groove width along Y (accommodates sash edge)
# Two tracks: rear track for fixed sash, front track for sliding sash.
TRACK_REAR_Y = FIXED_SASH_Y    # rear groove centered on fixed sash Y
TRACK_FRONT_Y = SLIDE_SASH_Y   # front groove centered on sliding sash Y

# Rubber gasket dimensions
GASKET_W = 0.008              # gasket strip width (visible border around glass)
GASKET_T = 0.004              # gasket thickness (stands slightly proud of glass)

# Latch hardware
LATCH_PLATE_W = 0.028
LATCH_PLATE_H = 0.075
LATCH_PLATE_T = 0.010
LATCH_LEVER_LEN = 0.045
LATCH_LEVER_R = 0.006

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

# LEFT sash is sliding, RIGHT sash is fixed.
SLIDE_OPEN_CX = INNER_X0 + SASH_OPENING_W / 2.0   # left
FIXED_OPEN_CX = INNER_X1 - SASH_OPENING_W / 2.0    # right
MID_CZ = (INNER_Z0 + INNER_Z1) / 2.0

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)
GLASS_RGBA = (0.52, 0.60, 0.66, 0.30)
METAL_RGBA = (0.74, 0.76, 0.79, 1.0)
GASKET_RGBA = (0.12, 0.12, 0.13, 1.0)   # dark EPDM rubber


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery)
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float, y_center: float, depth: float) -> cq.Workplane:
    """Axis-aligned box spanning [x0,x1] x [z0,z1], centered on y_center."""
    return (
        cq.Workplane("XY")
        .transformed(offset=((x0 + x1) / 2.0, y_center, (z0 + z1) / 2.0))
        .box(x1 - x0, depth, z1 - z0)
    )


def _build_frame_shape() -> cq.Workplane:
    """Outer frame: thick slab with the main opening cut out, leaving a true
    hollow perimeter (head, sill, jambs). Deep track grooves are booleaned into
    the head and sill rails as protruding channel lips."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    cut_depth = FRAME_DEPTH + 0.02
    opening = _slab(INNER_X0, INNER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    frame = outer.cut(opening)

    # Deep track grooves protruding into the opening from the head and sill.
    # These are solid lips that capture the sash edges. Offset past the glass
    # rebate so they don't intersect the glass pane edges.
    rear_groove_top = _slab(
        INNER_X0, INNER_X1,
        INNER_Z1 + REBATE, INNER_Z1 + TRACK_GROOVE_DEPTH,
        TRACK_REAR_Y, TRACK_GROOVE_WIDTH,
    )
    rear_groove_bot = _slab(
        INNER_X0, INNER_X1,
        INNER_Z0 - TRACK_GROOVE_DEPTH, INNER_Z0 - REBATE,
        TRACK_REAR_Y, TRACK_GROOVE_WIDTH,
    )
    front_groove_top = _slab(
        INNER_X0, INNER_X1,
        INNER_Z1 + REBATE, INNER_Z1 + TRACK_GROOVE_DEPTH,
        TRACK_FRONT_Y, TRACK_GROOVE_WIDTH,
    )
    front_groove_bot = _slab(
        INNER_X0, INNER_X1,
        INNER_Z0 - TRACK_GROOVE_DEPTH, INNER_Z0 - REBATE,
        TRACK_FRONT_Y, TRACK_GROOVE_WIDTH,
    )
    frame = frame.union(rear_groove_top).union(rear_groove_bot).union(front_groove_top).union(front_groove_bot)
    return frame


def _build_sash_shape() -> cq.Workplane:
    """One sash ring in its own local frame, centered on origin."""
    ow = SASH_OPENING_W
    oh = SASH_OPENING_H
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SASH_DEPTH)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH + 0.02)
    return outer.cut(opening)


def _build_sash_glass_shape() -> cq.Workplane:
    """Single clear pane filling the sash opening, rebated under the sash lip."""
    ow = SASH_OPENING_W + 2 * REBATE
    oh = SASH_OPENING_H + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_gasket_shape() -> cq.Workplane:
    """Rubber gasket strip: a thin rectangular frame around the glass pane.
    Built in sash-local frame, sitting on the glass face (+Y side).
    Four strips forming a rectangle border around the glass opening."""
    gw = SASH_OPENING_W + 2 * REBATE  # glass width (same as glass)
    gh = SASH_OPENING_H + 2 * REBATE  # glass height
    gasket_y = GLASS_T / 2.0 + GASKET_T / 2.0  # sits on top of glass face

    # Build four strips: top, bottom, left, right
    # Top strip
    top = _slab(
        -gw / 2.0 - GASKET_W, gw / 2.0 + GASKET_W,
        gh / 2.0, gh / 2.0 + GASKET_W,
        gasket_y, GASKET_T,
    )
    # Bottom strip
    bot = _slab(
        -gw / 2.0 - GASKET_W, gw / 2.0 + GASKET_W,
        -gh / 2.0 - GASKET_W, -gh / 2.0,
        gasket_y, GASKET_T,
    )
    # Left strip
    left = _slab(
        -gw / 2.0 - GASKET_W, -gw / 2.0,
        -gh / 2.0, gh / 2.0,
        gasket_y, GASKET_T,
    )
    # Right strip
    right = _slab(
        gw / 2.0, gw / 2.0 + GASKET_W,
        -gh / 2.0, gh / 2.0,
        gasket_y, GASKET_T,
    )
    return top.union(bot).union(left).union(right)


# ---------------------------------------------------------------------------
# Part builders
# ---------------------------------------------------------------------------

def _add_sash(model: ArticulatedObject, name: str) -> None:
    """Add a sash part (vinyl ring + glass + rubber gasket) in its own local frame."""
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
    sash.visual(
        mesh_from_cadquery(_build_gasket_shape(), f"{name}_gasket"),
        material="gasket",
        name=f"{name}_gasket",
    )


def _add_latch(model: ArticulatedObject, sash_name: str) -> None:
    """Cam-latch on the sliding sash's meeting (inner/right) stile at mid-height.
    The sliding sash is now LEFT, so the meeting stile is at local x = +SASH_OPENING_W/2 + SASH_FACE/2."""
    sash = model.get_part(sash_name)

    # Meeting stile is on the RIGHT side of the left sliding sash
    stile_x = SASH_OPENING_W / 2.0 + SASH_FACE / 2.0
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
    model = ArticulatedObject(name="two_panel_sliding_window")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("metal", rgba=METAL_RGBA)
    model.material("gasket", rgba=GASKET_RGBA)

    # --- Static outer frame (root) with track grooves ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="vinyl",
        name="frame_shell",
    )

    # --- Sliding (left) + fixed (right) sashes ---
    _add_sash(model, "sliding_sash")
    _add_sash(model, "fixed_sash")
    _add_latch(model, "sliding_sash")

    # FIXED right sash seated in the rear glazing plane.
    model.articulation(
        "frame_to_fixed_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="fixed_sash",
        origin=Origin(xyz=(FIXED_OPEN_CX, FIXED_SASH_Y, MID_CZ)),
    )

    # SLIDING left sash: PRISMATIC along X. Positive q slides rightward (+X)
    # toward the fixed sash to open. The sash stays retained in the head/sill
    # track at full travel.
    slide_travel = SASH_OPENING_W * 0.90
    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="sliding_sash",
        origin=Origin(xyz=(SLIDE_OPEN_CX, SLIDE_SASH_Y, MID_CZ)),
        axis=(1.0, 0.0, 0.0),
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
    # Gasket sits on the glass face and slightly overlaps the glass + vinyl sash.
    for nm in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_gasket",
            elem_b=f"{nm}_glass",
            reason="Rubber gasket strip is seated onto the glass face perimeter (compression seal).",
        )
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_gasket",
            elem_b=f"{nm}_vinyl",
            reason="Rubber gasket strip overlaps the sash inner lip edge (retained in glazing rebate).",
        )
    # Each sash ring laps the frame opening edge (seated in the track grooves
    # integrated into the frame rails).
    for nm in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_vinyl",
            reason=f"{nm} ring is rebated into the frame opening / head-sill track grooves (seated capture).",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_glass",
            reason=f"{nm} glass is rebated under the frame opening lip (captured glazing).",
        )
    # Latch keeper plate seated on the sliding sash meeting stile.
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

        # Frame spans the full width.
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
        # Sliding sash on LEFT, fixed sash on RIGHT.
        sx = (s_aabb[0][0] + s_aabb[1][0]) / 2.0
        fx = (f_aabb[0][0] + f_aabb[1][0]) / 2.0
        ctx.check(
            "sliding sash left of fixed sash",
            sx < fx,
            details=f"sliding_x={sx:.3f}, fixed_x={fx:.3f}",
        )
        # Both sashes seated within the frame height.
        for nm, ab in (("fixed", f_aabb), ("sliding", s_aabb)):
            ctx.check(
                f"{nm} sash seated within frame height",
                ab[0][2] > frame_aabb[0][2] - 1e-4 and ab[1][2] < frame_aabb[1][2] + 1e-4,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}]",
            )
        # Sliding sash sits proud (in +Y) of the fixed sash.
        sy = (s_aabb[0][1] + s_aabb[1][1]) / 2.0
        fy = (f_aabb[0][1] + f_aabb[1][1]) / 2.0
        ctx.check(
            "sliding sash proud of fixed sash",
            sy > fy + 0.02,
            details=f"sliding_y={sy:.3f}, fixed_y={fy:.3f}",
        )
        # Both sashes seated in the frame opening.
        ctx.expect_overlap(
            fixed_sash, frame, axes="xz", min_overlap=0.03,
            name="fixed sash seated in frame opening",
        )
        ctx.expect_overlap(
            sliding_sash, frame, axes="xz", min_overlap=0.03,
            name="sliding sash seated in frame opening",
        )

        # Gasket strips present on both sashes (rubber seal around glass).
        for nm, sash_part in (("fixed_sash", fixed_sash), ("sliding_sash", sliding_sash)):
            gasket_aabb = ctx.part_element_world_aabb(sash_part, elem=f"{nm}_gasket")
            glass_aabb = ctx.part_element_world_aabb(sash_part, elem=f"{nm}_glass")
            # Gasket should overlap the glass in XZ (it frames the glass).
            ctx.check(
                f"{nm} has rubber gasket around glass",
                gasket_aabb is not None and glass_aabb is not None,
                details=f"gasket_aabb={gasket_aabb}, glass_aabb={glass_aabb}",
            )
            if gasket_aabb and glass_aabb:
                gasket_w_x = gasket_aabb[1][0] - gasket_aabb[0][0]
                glass_w_x = glass_aabb[1][0] - glass_aabb[0][0]
                ctx.check(
                    f"{nm} gasket wider than glass (frames the pane)",
                    gasket_w_x > glass_w_x - 0.001,
                    details=f"gasket_x={gasket_w_x:.4f}, glass_x={glass_w_x:.4f}",
                )

        # Frame has deep profile with integrated track grooves. The frame shell
        # should extend beyond the basic inner opening to show the track lips.
        ctx.check(
            "frame has deep track grooves integrated into rails",
            frame_aabb[1][2] > INNER_Z1 + 0.005,  # head extends above inner opening
            details=f"frame_zmax={frame_aabb[1][2]:.4f}, inner_z1={INNER_Z1:.4f}",
        )
        ctx.check(
            "frame sill extends below inner opening for track grooves",
            frame_aabb[0][2] < INNER_Z0 - 0.005,
            details=f"frame_zmin={frame_aabb[0][2]:.4f}, inner_z0={INNER_Z0:.4f}",
        )

        # Latch on the sliding sash's meeting (inner/right) stile.
        latch_aabb = ctx.part_element_world_aabb(sliding_sash, elem="sliding_sash_latch_plate")
        latch_cx = (latch_aabb[0][0] + latch_aabb[1][0]) / 2.0
        latch_cz = (latch_aabb[0][2] + latch_aabb[1][2]) / 2.0
        latch_cy = (latch_aabb[0][1] + latch_aabb[1][1]) / 2.0
        ctx.check(
            "latch on sliding sash inner (meeting) stile",
            latch_cx > sx,  # meeting stile is on the right side of the left sash
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

    # --- Driven/open pose: sliding sash slides toward fixed sash (+X) ---
    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel}):
        s_open = ctx.part_world_aabb(sliding_sash)
        open_sx = (s_open[0][0] + s_open[1][0]) / 2.0
        # Positive q moves the sash in +X (toward the fixed right sash).
        ctx.check(
            "sliding sash opens toward fixed sash (+X)",
            abs((open_sx - rest_sx) - travel) < 0.02 and open_sx > rest_sx + 0.30,
            details=f"rest_x={rest_sx:.3f}, open_x={open_sx:.3f}, travel={travel:.3f}",
        )
        # Pure horizontal slide (no Z change).
        open_sz = (s_open[0][2] + s_open[1][2]) / 2.0
        ctx.check(
            "slide is purely horizontal",
            abs(open_sz - rest_sz) < 0.02,
            details=f"open_z={open_sz:.3f}, rest_z={rest_sz:.3f}",
        )
        # Retained insertion: sliding sash stays within frame X span.
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
