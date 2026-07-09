from __future__ import annotations

# Vertical-sliding (single-hung style) window, white vinyl frame.
# Upper sash is FIXED in the rear track; lower sash SLIDES UPWARD on a
# vertical prismatic joint in the front (proud) track.  Two small roller
# blocks are mounted at the bottom of the moving sash.  A cam-latch is
# mounted on the lower sash's meeting (top) rail.
#
# Coordinate convention:
#   +Z is up, window stands vertically.
#     width  -> X
#     height -> Z   (sill near z=0, head at z=TOTAL_H)
#     depth  -> Y   (glazing plane is X-Z)
#   q=0 reads SHUT.  Positive q slides the lower sash UPWARD (+Z).
#   The two sashes overlap at the meeting rail (visible crossing in Y).

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

FRAME_FACE = 0.085        # outer frame member face width (chunky vinyl)
FRAME_DEPTH = 0.140       # deep box section along Y

MEETING_OVERLAP = 0.040   # Z overlap of the two sashes at the meeting rail

SASH_FACE = 0.065         # sash perimeter rail/stile face width
SASH_DEPTH = 0.055        # sash depth along Y
GLASS_T = 0.008           # glazing thickness along Y

# Y layout: upper sash in the rear track, lower sash proud toward +Y so it
# slides in front of the upper sash at the meeting rail.
UPPER_SASH_Y = -0.028
LOWER_SASH_Y = 0.044

REBATE = 0.005            # glass tucks under the sash lip

# Roller blocks (small nylon housing blocks at the bottom of the lower sash)
ROLLER_W = 0.035          # roller block width  (X)
ROLLER_H = 0.018          # roller block height (Z)
ROLLER_D = 0.025          # roller block depth  (Y)

# Latch hardware (cam lock on the meeting / top rail of the lower sash)
LATCH_PLATE_W = 0.030
LATCH_PLATE_H = 0.050
LATCH_PLATE_T = 0.010
LATCH_LEVER_LEN = 0.040
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
MID_Z = (INNER_Z0 + INNER_Z1) / 2.0

# Each sash height: half the inner height plus half the meeting overlap so
# the two sashes overlap by MEETING_OVERLAP at the meeting rail.
UPPER_SASH_H = INNER_H / 2.0 + MEETING_OVERLAP / 2.0
LOWER_SASH_H = INNER_H / 2.0 + MEETING_OVERLAP / 2.0

# Sash outer width spans the full inner opening.
SASH_OUTER_W = INNER_W

# World Z centers of each sash at the closed (q=0) pose.
UPPER_SASH_CZ = INNER_Z1 - UPPER_SASH_H / 2.0
LOWER_SASH_CZ = INNER_Z0 + LOWER_SASH_H / 2.0

# Glass clear opening inside each sash ring.
UPPER_GLASS_W = SASH_OUTER_W - 2.0 * SASH_FACE + 2.0 * REBATE
UPPER_GLASS_H = UPPER_SASH_H - 2.0 * SASH_FACE + 2.0 * REBATE
LOWER_GLASS_W = SASH_OUTER_W - 2.0 * SASH_FACE + 2.0 * REBATE
LOWER_GLASS_H = LOWER_SASH_H - 2.0 * SASH_FACE + 2.0 * REBATE

# Slide travel: the lower sash can slide up until its top approaches the head.
SLIDE_TRAVEL = min(LOWER_SASH_H * 0.80, INNER_Z1 - (LOWER_SASH_CZ + LOWER_SASH_H / 2.0) - 0.01)

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)
GLASS_RGBA = (0.52, 0.60, 0.66, 0.30)
METAL_RGBA = (0.74, 0.76, 0.79, 1.0)
ROLLER_RGBA = (0.22, 0.22, 0.25, 1.0)   # dark nylon roller blocks


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery)
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float,
          y_center: float, depth: float) -> cq.Workplane:
    """Axis-aligned box spanning [x0,x1] x [z0,z1], centered on y_center."""
    return (
        cq.Workplane("XY")
        .transformed(offset=((x0 + x1) / 2.0, y_center, (z0 + z1) / 2.0))
        .box(x1 - x0, depth, z1 - z0)
    )


def _build_frame_shape() -> cq.Workplane:
    """Outer frame: thick slab with the single large inner opening."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    cut_depth = FRAME_DEPTH + 0.02
    opening = _slab(INNER_X0, INNER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    return outer.cut(opening)


def _build_sash_ring(width: float, height: float) -> cq.Workplane:
    """Sash ring: outer slab minus the clear glass opening."""
    outer = _slab(-width / 2.0, width / 2.0, -height / 2.0, height / 2.0,
                  0.0, SASH_DEPTH)
    inner_w = width - 2.0 * SASH_FACE
    inner_h = height - 2.0 * SASH_FACE
    opening = _slab(-inner_w / 2.0, inner_w / 2.0,
                    -inner_h / 2.0, inner_h / 2.0,
                    0.0, SASH_DEPTH + 0.02)
    return outer.cut(opening)


def _build_glass_pane(width: float, height: float) -> cq.Workplane:
    """Single clear glass pane centered on the sash local frame."""
    return _slab(-width / 2.0, width / 2.0,
                 -height / 2.0, height / 2.0,
                 0.0, GLASS_T)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_sliding_window")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("metal", rgba=METAL_RGBA)
    model.material("roller", rgba=ROLLER_RGBA)

    # ---- Static outer frame (root) ----
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="vinyl",
        name="frame_shell",
    )

    # ---- Upper sash (FIXED, rear track) ----
    upper = model.part("upper_sash")
    upper.visual(
        mesh_from_cadquery(_build_sash_ring(SASH_OUTER_W, UPPER_SASH_H),
                           "upper_sash_vinyl"),
        material="vinyl",
        name="upper_sash_vinyl",
    )
    upper.visual(
        mesh_from_cadquery(_build_glass_pane(UPPER_GLASS_W, UPPER_GLASS_H),
                           "upper_sash_glass"),
        material="glass",
        name="upper_sash_glass",
    )

    # ---- Lower sash (SLIDING, front track) ----
    lower = model.part("lower_sash")
    lower.visual(
        mesh_from_cadquery(_build_sash_ring(SASH_OUTER_W, LOWER_SASH_H),
                           "lower_sash_vinyl"),
        material="vinyl",
        name="lower_sash_vinyl",
    )
    lower.visual(
        mesh_from_cadquery(_build_glass_pane(LOWER_GLASS_W, LOWER_GLASS_H),
                           "lower_sash_glass"),
        material="glass",
        name="lower_sash_glass",
    )

    # Two roller blocks at the bottom of the lower sash (near left & right edges).
    roller_inset = 0.030   # distance from sash edge to roller center
    roller_x = SASH_OUTER_W / 2.0 - ROLLER_W / 2.0 - roller_inset
    roller_z = -LOWER_SASH_H / 2.0   # centered on the bottom edge of the sash
    lower.visual(
        Box((ROLLER_W, ROLLER_D, ROLLER_H)),
        origin=Origin(xyz=(-roller_x, 0.0, roller_z)),
        material="roller",
        name="roller_left",
    )
    lower.visual(
        Box((ROLLER_W, ROLLER_D, ROLLER_H)),
        origin=Origin(xyz=(roller_x, 0.0, roller_z)),
        material="roller",
        name="roller_right",
    )

    # Latch on the lower sash meeting (top) rail, centered horizontally.
    face_y = SASH_DEPTH / 2.0
    plate_y = face_y + LATCH_PLATE_T / 2.0
    latch_z = LOWER_SASH_H / 2.0 - SASH_FACE / 2.0   # top rail center
    lower.visual(
        Box((LATCH_PLATE_W, LATCH_PLATE_T, LATCH_PLATE_H)),
        origin=Origin(xyz=(0.0, plate_y, latch_z)),
        material="metal",
        name="latch_plate",
    )
    lever_y = face_y + LATCH_PLATE_T + LATCH_LEVER_LEN / 2.0
    lower.visual(
        Cylinder(radius=LATCH_LEVER_R, length=LATCH_LEVER_LEN),
        origin=Origin(xyz=(0.0, lever_y, latch_z), rpy=(1.5707963, 0.0, 0.0)),
        material="metal",
        name="latch_lever",
    )

    # ---- Articulations ----

    # Upper sash: FIXED in the rear track.
    model.articulation(
        "frame_to_upper_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="upper_sash",
        origin=Origin(xyz=(0.0, UPPER_SASH_Y, UPPER_SASH_CZ)),
    )

    # Lower sash: PRISMATIC along +Z (slides upward to open).
    # Lower limit is 0.10 m so the rest pose shows the sash partially raised,
    # making the meeting-rail overlap clearly visible.
    model.articulation(
        "frame_to_lower_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="lower_sash",
        origin=Origin(xyz=(0.0, LOWER_SASH_Y, LOWER_SASH_CZ)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.5,
            lower=0.10, upper=SLIDE_TRAVEL,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    upper = object_model.get_part("upper_sash")
    lower = object_model.get_part("lower_sash")
    slide = object_model.get_articulation("frame_to_lower_sash")

    # ---- Intentional-overlap allowances ----

    # Glass rebated under each sash lip (captured glazing).
    for nm in ("upper_sash", "lower_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass", elem_b=f"{nm}_vinyl",
            reason="Glass pane rebated under sash lip (captured glazing).",
        )

    # Each sash ring seated in the frame track.
    for nm in ("upper_sash", "lower_sash"):
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell", elem_b=f"{nm}_vinyl",
            reason=f"{nm} ring is seated in the frame jamb track.",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell", elem_b=f"{nm}_glass",
            reason=f"{nm} glass captured inside the frame opening.",
        )

    # Roller blocks embedded in the lower sash bottom rail and seated in the
    # frame sill track.
    for roller in ("roller_left", "roller_right"):
        ctx.allow_overlap(
            "lower_sash", "lower_sash",
            elem_a=roller, elem_b="lower_sash_vinyl",
            reason=f"{roller} is seated into the lower sash bottom rail.",
        )
        ctx.allow_overlap(
            "frame", "lower_sash",
            elem_a="frame_shell", elem_b=roller,
            reason=f"{roller} rides in the frame sill track (seated roller).",
        )

    # Latch plate seated on the lower sash meeting rail.
    ctx.allow_overlap(
        "lower_sash", "lower_sash",
        elem_a="latch_plate", elem_b="lower_sash_vinyl",
        reason="Latch keeper plate seated on the lower sash meeting (top) rail.",
    )

    # ---- Rest pose (q = lower limit = 0.10 m, sash partially raised) ----
    rest_q = slide.motion_limits.lower
    with ctx.pose({slide: rest_q}):
        frame_aabb = ctx.part_world_aabb(frame)
        upper_aabb = ctx.part_world_aabb(upper)
        lower_aabb = ctx.part_world_aabb(lower)

        # Frame proportions.
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        frame_h = frame_aabb[1][2] - frame_aabb[0][2]
        ctx.check(
            "frame spans full window width",
            frame_w > 1.40,
            details=f"frame_w={frame_w:.3f}",
        )
        ctx.check(
            "frame spans full window height",
            abs(frame_h - TOTAL_H) < 0.02,
            details=f"frame_h={frame_h:.3f}",
        )
        ctx.check(
            "sill near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"zmin={frame_aabb[0][2]:.4f}",
        )

        # Upper sash above lower sash.
        upper_cz = (upper_aabb[0][2] + upper_aabb[1][2]) / 2.0
        lower_cz = (lower_aabb[0][2] + lower_aabb[1][2]) / 2.0
        ctx.check(
            "upper sash above lower sash",
            upper_cz > lower_cz + 0.10,
            details=f"upper_z={upper_cz:.3f}, lower_z={lower_cz:.3f}",
        )

        # Lower sash proud of upper sash (front track).
        upper_cy = (upper_aabb[0][1] + upper_aabb[1][1]) / 2.0
        lower_cy = (lower_aabb[0][1] + lower_aabb[1][1]) / 2.0
        ctx.check(
            "lower sash proud of upper sash (front track)",
            lower_cy > upper_cy + 0.02,
            details=f"lower_y={lower_cy:.3f}, upper_y={upper_cy:.3f}",
        )

        # Meeting rail overlap: lower sash top extends above upper sash bottom.
        upper_bottom_z = upper_aabb[0][2]
        lower_top_z = lower_aabb[1][2]
        ctx.check(
            "meeting rail Z overlap visible at rest",
            lower_top_z > upper_bottom_z + 0.01,
            details=f"upper_bottom={upper_bottom_z:.3f}, lower_top={lower_top_z:.3f}",
        )

        # Visible gap at sill because sash is partially raised.
        sill_z = frame_aabb[0][2] + FRAME_FACE
        gap_at_sill = lower_aabb[0][2] - sill_z
        ctx.check(
            "visible gap at sill (sash partially open)",
            gap_at_sill > 0.05,
            details=f"gap={gap_at_sill:.3f}",
        )

        # Lower sash within frame height.
        for nm, ab in (("upper", upper_aabb), ("lower", lower_aabb)):
            ctx.check(
                f"{nm} sash within frame height",
                ab[0][2] > frame_aabb[0][2] - 1e-4 and ab[1][2] < frame_aabb[1][2] + 1e-4,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}]",
            )

        # Sashes overlap in X projection with frame.
        ctx.expect_overlap(
            upper, frame, axes="xz", min_overlap=0.03,
            name="upper sash seated in frame opening",
        )
        ctx.expect_overlap(
            lower, frame, axes="xz", min_overlap=0.03,
            name="lower sash seated in frame opening",
        )

        # Roller blocks at the bottom of the lower sash.
        rl = ctx.part_element_world_aabb(lower, elem="roller_left")
        rr = ctx.part_element_world_aabb(lower, elem="roller_right")
        ctx.check(
            "roller_left near bottom of lower sash",
            rl[0][2] < lower_aabb[0][2] + 0.025,
            details=f"roller_zmin={rl[0][2]:.4f}, sash_zmin={lower_aabb[0][2]:.4f}",
        )
        ctx.check(
            "roller_right near bottom of lower sash",
            rr[0][2] < lower_aabb[0][2] + 0.025,
            details=f"roller_zmin={rr[0][2]:.4f}, sash_zmin={lower_aabb[0][2]:.4f}",
        )
        # Rollers on opposite sides of the sash.
        rl_cx = (rl[0][0] + rl[1][0]) / 2.0
        rr_cx = (rr[0][0] + rr[1][0]) / 2.0
        ctx.check(
            "two rollers on opposite sides of lower sash",
            rr_cx - rl_cx > 0.50,
            details=f"left_x={rl_cx:.3f}, right_x={rr_cx:.3f}",
        )

        # Rollers seated near sill track.
        for rname, r_aabb in (("roller_left", rl), ("roller_right", rr)):
            ctx.check(
                f"{rname} raised above sill (sash partially open)",
                r_aabb[0][2] > sill_z - 0.005,
                details=f"roller_zmin={r_aabb[0][2]:.4f}, sill_z={sill_z:.4f}",
            )

        # Latch on the meeting (top) rail of the lower sash.
        latch_aabb = ctx.part_element_world_aabb(lower, elem="latch_plate")
        latch_cz = (latch_aabb[0][2] + latch_aabb[1][2]) / 2.0
        ctx.check(
            "latch on lower sash top (meeting) rail",
            latch_cz > lower_cz + 0.10,
            details=f"latch_z={latch_cz:.3f}, lower_center_z={lower_cz:.3f}",
        )

        rest_lower_cz = lower_cz

    # ---- Partially open pose: lower sash slides upward ----
    partial_q = SLIDE_TRAVEL * 0.45
    with ctx.pose({slide: partial_q}):
        lower_open = ctx.part_world_aabb(lower)
        open_cz = (lower_open[0][2] + lower_open[1][2]) / 2.0

        # Lower sash moved upward.
        ctx.check(
            "lower sash moves upward when opened",
            open_cz > rest_lower_cz + 0.05,
            details=f"rest_z={rest_lower_cz:.3f}, open_z={open_cz:.3f}",
        )

        # Pure vertical slide (no X change).
        lower_rest_aabb = ctx.part_world_aabb(lower)  # already at partial pose
        lower_cx_open = (lower_open[0][0] + lower_open[1][0]) / 2.0
        ctx.check(
            "slide is purely vertical (no X drift)",
            abs(lower_cx_open) < 0.02,
            details=f"lower_cx={lower_cx_open:.3f}",
        )

        # Meeting rail overlap increases when opened.
        upper_aabb2 = ctx.part_world_aabb(upper)
        overlap_z = lower_open[1][2] - upper_aabb2[0][2]
        ctx.check(
            "meeting rail overlap increases when partially opened",
            overlap_z > MEETING_OVERLAP + 0.05,
            details=f"overlap={overlap_z:.3f}",
        )

        # Lower sash retained within frame at partial open.
        frame_aabb2 = ctx.part_world_aabb(frame)
        ctx.check(
            "lower sash retained within frame at partial open",
            lower_open[1][2] < frame_aabb2[1][2] + 0.01,
            details=f"sash_top={lower_open[1][2]:.3f}, frame_top={frame_aabb2[1][2]:.3f}",
        )

        # Gap at sill visible when partially open.
        gap_at_sill = lower_open[0][2] - frame_aabb2[0][2]
        ctx.check(
            "visible gap at sill when partially open",
            gap_at_sill > 0.05,
            details=f"gap={gap_at_sill:.3f}",
        )

    # ---- Prismatic joint properties ----
    ctx.check(
        "slide joint is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )
    ctx.check(
        "slide axis is vertical (+Z)",
        abs(slide.axis[2] - 1.0) < 1e-6 and abs(slide.axis[0]) < 1e-6 and abs(slide.axis[1]) < 1e-6,
        details=f"axis={slide.axis}",
    )
    ctx.check(
        "slide has positive travel range",
        slide.motion_limits.upper > 0.10,
        details=f"upper={slide.motion_limits.upper:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
