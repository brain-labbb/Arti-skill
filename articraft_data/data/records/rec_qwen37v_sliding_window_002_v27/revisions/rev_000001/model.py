from __future__ import annotations

# Dual-slide horizontal sliding window variant, white vinyl frame.
# - Two sashes slide in OPPOSITE directions on separate prismatic joints
#   (rear sash slides +X, front sash slides -X).
# - Muntin grid bars (3×3 = 2 vertical + 2 horizontal) on the FRONT sash only.
# - Deep track grooves cut into the head and sill rails for the dual-track
#   system.
# - Cam-latch handle on the front sash meeting stile.
#
# Coordinate convention:
#   +Z is up, window stands vertically.
#     width  -> X
#     height -> Z  (sill near z=0)
#     depth  -> Y  (frame depth / slide-normal)
#   Glass plane is X-Z. q=0 reads SHUT. Driving either prismatic joint slides
#   its sash horizontally past the other, staying retained in the head/sill
#   track grooves.

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
TOTAL_H = 1.72            # overall height along Z (sill near z=0)

FRAME_FACE = 0.085        # outer frame member face width (chunky vinyl)
FRAME_DEPTH = 0.140       # deep box section along Y (thick patio-slider profile)

MEETING_OVERLAP = 0.040   # the two sash stiles overlap by this much at center

SASH_FACE = 0.075         # sash perimeter rail/stile face width (chunky)
SASH_DEPTH = 0.060        # sash depth along Y
GLASS_T = 0.008           # glazing thickness along Y

# Y layout: rear track at -Y, front track at +Y
REAR_SASH_Y = -0.028      # rear track center (Y)
FRONT_SASH_Y = 0.044      # front track center (Y)

REBATE = 0.005            # glass tucks under the sash lip by this much

# Deep track grooves along head and sill
GROOVE_W = 0.025          # groove channel width along Y
GROOVE_D = 0.020          # groove depth into sill/head (visible channel)

# Muntin grid bars (3×3 colonial pattern on front sash only)
MUNTIN_W = 0.012          # bar face width
MUNTIN_T = 0.008          # bar thickness (proud of glass face)
MUNTIN_INSET = 0.010      # bars inset from sash opening edge (captured by glazing bead)

# Latch (cam lock) hardware
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

REAR_OPEN_CX = INNER_X0 + SASH_OPENING_W / 2.0       # left
FRONT_OPEN_CX = INNER_X1 - SASH_OPENING_W / 2.0      # right
MID_CZ = (INNER_Z0 + INNER_Z1) / 2.0

# Muntin grid positions (3×3 grid: 2 vertical + 2 horizontal bars)
# Vertical bars divide opening into 3 equal columns
_ow = SASH_OPENING_W
_oh = SASH_OPENING_H
MUNTIN_V_X = [
    -_ow / 2.0 + _ow / 3.0,     # -ow/6
    -_ow / 2.0 + 2.0 * _ow / 3.0,  # +ow/6
]
MUNTIN_H_Z = [
    -_oh / 2.0 + _oh / 3.0,     # -oh/6
    -_oh / 2.0 + 2.0 * _oh / 3.0,  # +oh/6
]
# Muntin Y: back face seated on glass front face
MUNTIN_Y = GLASS_T / 2.0 + MUNTIN_T / 2.0

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)
GLASS_RGBA = (0.52, 0.60, 0.66, 0.30)
METAL_RGBA = (0.74, 0.76, 0.79, 1.0)


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery), authored directly in meters.
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float, y_center: float, depth: float) -> cq.Workplane:
    """Axis-aligned box spanning [x0,x1] x [z0,z1] in the X-Z plane, centered on
    y_center with the given Y depth."""
    return (
        cq.Workplane("XY")
        .transformed(offset=((x0 + x1) / 2.0, y_center, (z0 + z1) / 2.0))
        .box(x1 - x0, depth, z1 - z0)
    )


def _build_frame_shape() -> cq.Workplane:
    """Static outer frame: thick slab cut by the opening + deep dual-track
    grooves in the sill (bottom rail) and head (top rail)."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    cut_depth = FRAME_DEPTH + 0.02
    opening = _slab(INNER_X0, INNER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    frame = outer.cut(opening)

    # Deep track grooves in sill (bottom rail) – two parallel channels
    eps = 0.002
    for track_y in (REAR_SASH_Y, FRONT_SASH_Y):
        groove = _slab(
            INNER_X0, INNER_X1,
            INNER_Z0 - GROOVE_D, INNER_Z0 + eps,
            track_y, GROOVE_W,
        )
        frame = frame.cut(groove)

    # Deep track grooves in head (top rail) – two parallel channels
    for track_y in (REAR_SASH_Y, FRONT_SASH_Y):
        groove = _slab(
            INNER_X0, INNER_X1,
            INNER_Z1 - eps, INNER_Z1 + GROOVE_D,
            track_y, GROOVE_W,
        )
        frame = frame.cut(groove)

    return frame


def _build_sash_shape() -> cq.Workplane:
    """One sash ring in its OWN local frame, centered on origin."""
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


# ---------------------------------------------------------------------------
# Part builders
# ---------------------------------------------------------------------------

def _add_sash(model: ArticulatedObject, name: str, *, with_muntins: bool = False) -> None:
    """Add a sash part (vinyl ring + clear glass), optionally with muntin grid."""
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
    if with_muntins:
        _add_muntins(sash, name)


def _add_muntins(sash, name: str) -> None:
    """Add 3×3 muntin grid bars (2 vertical + 2 horizontal) on the sash glass."""
    oh = SASH_OPENING_H
    ow = SASH_OPENING_W
    # Bars are inset from the sash opening edge (captured by glazing bead)
    bar_len_x = ow - 2 * MUNTIN_INSET
    bar_len_z = oh - 2 * MUNTIN_INSET

    # Vertical bars (inset height, thin along X)
    for i, vx in enumerate(MUNTIN_V_X):
        sash.visual(
            Box((MUNTIN_W, MUNTIN_T, bar_len_z)),
            origin=Origin(xyz=(vx, MUNTIN_Y, 0.0)),
            material="vinyl",
            name=f"{name}_muntin_v{i}",
        )

    # Horizontal bars (inset width, thin along Z)
    for i, hz in enumerate(MUNTIN_H_Z):
        sash.visual(
            Box((bar_len_x, MUNTIN_T, MUNTIN_W)),
            origin=Origin(xyz=(0.0, MUNTIN_Y, hz)),
            material="vinyl",
            name=f"{name}_muntin_h{i}",
        )


def _add_latch(model: ArticulatedObject, sash_name: str) -> None:
    """Add the cam-latch hardware on the sash's meeting (inner/left) stile."""
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
    model = ArticulatedObject(name="dual_slide_sliding_window")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("metal", rgba=METAL_RGBA)

    # --- Static outer frame (root) with track grooves ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="vinyl",
        name="frame_shell",
    )

    # --- Rear sash (rear track, no muntins, slides +X) ---
    _add_sash(model, "rear_sash", with_muntins=False)

    # --- Front sash (front track, muntin grid, slides -X) ---
    _add_sash(model, "front_sash", with_muntins=True)
    _add_latch(model, "front_sash")

    slide_travel = SASH_OPENING_W * 0.90

    # Rear sash: PRISMATIC along +X (positive q slides rightward to open).
    model.articulation(
        "frame_to_rear_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="rear_sash",
        origin=Origin(xyz=(REAR_OPEN_CX, REAR_SASH_Y, MID_CZ)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5, lower=0.0, upper=slide_travel),
    )

    # Front sash: PRISMATIC along -X (positive q slides leftward to open).
    model.articulation(
        "frame_to_front_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="front_sash",
        origin=Origin(xyz=(FRONT_OPEN_CX, FRONT_SASH_Y, MID_CZ)),
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
    rear_sash = object_model.get_part("rear_sash")
    front_sash = object_model.get_part("front_sash")
    rear_slide = object_model.get_articulation("frame_to_rear_sash")
    front_slide = object_model.get_articulation("frame_to_front_sash")

    # --- Muntin grid checks ---
    front_viz_names = [v.name for v in front_sash.visuals]
    rear_viz_names = [v.name for v in rear_sash.visuals]

    ctx.check(
        "front sash has muntin grid bars",
        any("muntin" in n for n in front_viz_names),
        details=f"front_sash visuals: {front_viz_names}",
    )
    ctx.check(
        "rear sash has no muntin bars (muntins on front sash only)",
        not any("muntin" in n for n in rear_viz_names),
        details=f"rear_sash visuals: {rear_viz_names}",
    )
    muntin_count = sum(1 for n in front_viz_names if "muntin" in n)
    ctx.check(
        "front sash has 4 muntin bars (2v + 2h for 3×3 grid)",
        muntin_count == 4,
        details=f"muntin count: {muntin_count}",
    )

    # --- Joint type checks: both sashes are prismatic (not fixed) ---
    ctx.check(
        "rear sash joint is prismatic",
        rear_slide.articulation_type == ArticulationType.PRISMATIC,
    )
    ctx.check(
        "front sash joint is prismatic",
        front_slide.articulation_type == ArticulationType.PRISMATIC,
    )

    # --- Intentional overlaps ---
    # Glass captured under sash lip
    for nm in ("rear_sash", "front_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_vinyl",
            reason="Clear pane is rebated under the sash lip (captured glazing).",
        )
    # Sashes seated in frame tracks
    for nm in ("rear_sash", "front_sash"):
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_vinyl",
            reason=f"{nm} ring is rebated into the frame track (seated capture).",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_glass",
            reason=f"{nm} glass is rebated under the frame opening lip.",
        )

    # Muntin bars seated on glass face (snap-in grille)
    muntin_names = [n for n in front_viz_names if "muntin" in n]
    for m in muntin_names:
        ctx.allow_overlap(
            "front_sash", "front_sash",
            elem_a=m,
            elem_b="front_sash_glass",
            reason="Muntin bar seated on glass face (snap-in grille).",
        )
    # Muntin bar crossings (vertical × horizontal)
    v_names = sorted(n for n in muntin_names if "_v" in n)
    h_names = sorted(n for n in muntin_names if "_h" in n)
    for v in v_names:
        for h in h_names:
            ctx.allow_overlap(
                "front_sash", "front_sash",
                elem_a=v, elem_b=h,
                reason="Muntin bars cross at grid intersections (snap-in grille pattern).",
            )

    # Latch keeper plate seated on front sash stile face
    ctx.allow_overlap(
        "front_sash", "front_sash",
        elem_a="front_sash_latch_plate",
        elem_b="front_sash_vinyl",
        reason="Latch keeper plate is seated onto the front-sash meeting-stile face.",
    )

    # --- Closed pose (q=0): window reads SHUT ---
    with ctx.pose({rear_slide: 0.0, front_slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        r_aabb = ctx.part_world_aabb(rear_sash)
        f_aabb = ctx.part_world_aabb(front_sash)

        # Frame spans full window
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        ctx.check(
            "frame spans full width",
            frame_w > 1.4,
            details=f"frame_w={frame_w:.3f}",
        )
        # Sill near z=0, head at full height
        ctx.check(
            "sill near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"frame zmin={frame_aabb[0][2]:.4f}",
        )
        ctx.check(
            "head at full height",
            abs(frame_aabb[1][2] - TOTAL_H) < 0.02,
            details=f"frame zmax={frame_aabb[1][2]:.4f}",
        )

        # Rear sash on left, front sash on right at rest
        rx = (r_aabb[0][0] + r_aabb[1][0]) / 2.0
        fx = (f_aabb[0][0] + f_aabb[1][0]) / 2.0
        ctx.check(
            "rear sash left of front sash at rest",
            rx < fx,
            details=f"rear_x={rx:.3f}, front_x={fx:.3f}",
        )

        # Front sash proud of rear sash (+Y)
        ry = (r_aabb[0][1] + r_aabb[1][1]) / 2.0
        fy = (f_aabb[0][1] + f_aabb[1][1]) / 2.0
        ctx.check(
            "front sash proud of rear sash",
            fy > ry + 0.02,
            details=f"front_y={fy:.3f}, rear_y={ry:.3f}",
        )

        # Both sashes seated within frame height
        for nm, ab in (("rear", r_aabb), ("front", f_aabb)):
            ctx.check(
                f"{nm} sash seated within frame height",
                ab[0][2] > frame_aabb[0][2] - 1e-4 and ab[1][2] < frame_aabb[1][2] + 1e-4,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}]",
            )

        # Both sashes overlap the frame opening
        ctx.expect_overlap(rear_sash, frame, axes="xz", min_overlap=0.03,
                           name="rear sash seated in frame opening")
        ctx.expect_overlap(front_sash, frame, axes="xz", min_overlap=0.03,
                           name="front sash seated in frame opening")

        rest_rx = rx
        rest_fx = fx
        rest_rz = (r_aabb[0][2] + r_aabb[1][2]) / 2.0
        rest_fz = (f_aabb[0][2] + f_aabb[1][2]) / 2.0

    # --- Rear sash opens in +X direction ---
    travel = rear_slide.motion_limits.upper
    with ctx.pose({rear_slide: travel, front_slide: 0.0}):
        r_open = ctx.part_world_aabb(rear_sash)
        open_rx = (r_open[0][0] + r_open[1][0]) / 2.0
        ctx.check(
            "rear sash slides +X when opened",
            open_rx > rest_rx + 0.30,
            details=f"rest_x={rest_rx:.3f}, open_x={open_rx:.3f}",
        )
        # Pure horizontal slide
        open_rz = (r_open[0][2] + r_open[1][2]) / 2.0
        ctx.check(
            "rear sash slide is purely horizontal",
            abs(open_rz - rest_rz) < 0.02,
            details=f"open_z={open_rz:.3f}, rest_z={rest_rz:.3f}",
        )
        # Retained in frame
        f_aabb_open = ctx.part_world_aabb(frame)
        ctx.check(
            "rear sash retained within frame X span at full travel",
            r_open[0][0] > f_aabb_open[0][0] - 1e-4 and r_open[1][0] < f_aabb_open[1][0] + 1e-4,
            details=f"sash x=[{r_open[0][0]:.3f},{r_open[1][0]:.3f}]",
        )
        ctx.expect_overlap(rear_sash, frame, axes="z", min_overlap=0.10,
                           name="rear sash retains vertical track engagement at full travel")

    # --- Front sash opens in -X direction (opposite) ---
    with ctx.pose({rear_slide: 0.0, front_slide: travel}):
        f_open = ctx.part_world_aabb(front_sash)
        open_fx = (f_open[0][0] + f_open[1][0]) / 2.0
        ctx.check(
            "front sash slides -X when opened (opposite to rear)",
            open_fx < rest_fx - 0.30,
            details=f"rest_x={rest_fx:.3f}, open_x={open_fx:.3f}",
        )
        # Pure horizontal slide
        open_fz = (f_open[0][2] + f_open[1][2]) / 2.0
        ctx.check(
            "front sash slide is purely horizontal",
            abs(open_fz - rest_fz) < 0.02,
            details=f"open_z={open_fz:.3f}, rest_z={rest_fz:.3f}",
        )
        # Retained in frame
        ctx.check(
            "front sash retained within frame X span at full travel",
            f_open[0][0] > f_aabb_open[0][0] - 1e-4 and f_open[1][0] < f_aabb_open[1][0] + 1e-4,
            details=f"sash x=[{f_open[0][0]:.3f},{f_open[1][0]:.3f}]",
        )
        ctx.expect_overlap(front_sash, frame, axes="z", min_overlap=0.10,
                           name="front sash retains vertical track engagement at full travel")

    # --- Opposite-direction proof: both sashes opened simultaneously ---
    with ctx.pose({rear_slide: travel, front_slide: travel}):
        r_both = ctx.part_world_aabb(rear_sash)
        f_both = ctx.part_world_aabb(front_sash)
        both_rx = (r_both[0][0] + r_both[1][0]) / 2.0
        both_fx = (f_both[0][0] + f_both[1][0]) / 2.0
        ctx.check(
            "sashes swap sides when both fully opened",
            both_rx > both_fx,
            details=f"rear_x={both_rx:.3f}, front_x={both_fx:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
