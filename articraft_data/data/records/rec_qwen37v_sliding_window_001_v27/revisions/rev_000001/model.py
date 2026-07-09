from __future__ import annotations

# Variant 27: Double-sliding window with two movable sashes on separate tracks.
#
# Structure:
#   - frame (root): head, sill, two jambs, two intermediate mullions, built as
#     one CadQuery solid with DEEP TRACK GROOVES cut into the head and sill
#     inner faces for two parallel sash tracks.
#   - center_panel (FIXED): thin fixed panel with plain glass, no muntin grille.
#   - front_sash (PRISMATIC +X): colonial muntin grille, rides the front track
#     (proud, +Y). At rest covers the left opening; slides right to open.
#   - rear_sash (PRISMATIC -X): colonial muntin grille, rides the rear track
#     (recessed, -Y). At rest covers the right opening; slides left to open.
#
# Coordinate convention:
#   +Z up, window stands vertically.
#   width  -> X,  height -> Z,  depth / glazing normal -> Y.
#   Glass plane is X-Z. q=0 reads SHUT; driving the prismatic joints slides
#   both sashes toward the center on separate Y-depth tracks.

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
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
SASH_DEPTH = 0.040
GLASS_T = 0.008

CENTER_PANEL_DEPTH = 0.018  # thin fixed panel

# Two parallel tracks at different Y depths
FRONT_TRACK_Y = 0.032
REAR_TRACK_Y = -0.032

# Deep track grooves cut into head/sill inner faces
GROOVE_W = 0.046
GROOVE_D = 0.020

# Colonial muntin grille (movable sashes only)
GRILLE_COLS = 4
GRILLE_ROWS = 5
MUNTIN_T = 0.020
MUNTIN_DEPTH = 0.016

REBATE = 0.005

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
TRACK_RGBA = (0.82, 0.83, 0.85, 1.0)  # slightly darker vinyl for track grooves


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float,
          y_center: float, depth: float) -> cq.Workplane:
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
    """Outer frame: head, sill, jambs, two mullions + deep track grooves."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)

    cut_depth = FRAME_DEPTH + 0.02
    left_cut = _slab(LEFT_X0, LEFT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    center_cut = _slab(CENTER_X0, CENTER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    right_cut = _slab(RIGHT_X0, RIGHT_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)

    frame = outer.cut(left_cut).cut(center_cut).cut(right_cut)

    # Deep track grooves in SILL (cut upward from sill inner face at Z=INNER_Z0)
    sill_gz0 = INNER_Z0 - GROOVE_D
    sill_gz1 = INNER_Z0 + 0.002
    fg_sill = _slab(INNER_X0, INNER_X1, sill_gz0, sill_gz1,
                    FRONT_TRACK_Y, GROOVE_W)
    rg_sill = _slab(INNER_X0, INNER_X1, sill_gz0, sill_gz1,
                    REAR_TRACK_Y, GROOVE_W)

    # Deep track grooves in HEAD (cut downward from head inner face at Z=INNER_Z1)
    head_gz0 = INNER_Z1 - 0.002
    head_gz1 = INNER_Z1 + GROOVE_D
    fg_head = _slab(INNER_X0, INNER_X1, head_gz0, head_gz1,
                    FRONT_TRACK_Y, GROOVE_W)
    rg_head = _slab(INNER_X0, INNER_X1, head_gz0, head_gz1,
                    REAR_TRACK_Y, GROOVE_W)

    frame = frame.cut(fg_sill).cut(rg_sill).cut(fg_head).cut(rg_head)
    return frame


def _build_track_rail_shape() -> cq.Workplane:
    """Raised guide rails between the track grooves on sill and head.
    Thin raised strips between and beside the groove channels that read as
    the sash guide rails visible in the track profile."""
    rail_h = 0.006  # rail height above sill/head face
    rail_w = 0.008  # rail width along Y

    rails = None

    # Center rail between the two grooves (on sill inner face)
    center_rail_y = 0.0
    # Sill center rail (just above sill face)
    sill_cr = _slab(INNER_X0, INNER_X1,
                    INNER_Z0, INNER_Z0 + rail_h,
                    center_rail_y, rail_w)
    rails = sill_cr

    # Head center rail (just below head face)
    head_cr = _slab(INNER_X0, INNER_X1,
                    INNER_Z1 - rail_h, INNER_Z1,
                    center_rail_y, rail_w)
    rails = rails.union(head_cr)

    # Outer rails (front side, between front groove and frame face)
    outer_front_y = FRONT_TRACK_Y + GROOVE_W / 2.0 + rail_w / 2.0
    if outer_front_y + rail_w / 2.0 < FRAME_DEPTH / 2.0 - 0.002:
        sill_of = _slab(INNER_X0, INNER_X1,
                        INNER_Z0, INNER_Z0 + rail_h,
                        outer_front_y, rail_w)
        head_of = _slab(INNER_X0, INNER_X1,
                        INNER_Z1 - rail_h, INNER_Z1,
                        outer_front_y, rail_w)
        rails = rails.union(sill_of).union(head_of)

    # Outer rails (rear side)
    outer_rear_y = REAR_TRACK_Y - GROOVE_W / 2.0 - rail_w / 2.0
    if outer_rear_y - rail_w / 2.0 > -FRAME_DEPTH / 2.0 + 0.002:
        sill_or = _slab(INNER_X0, INNER_X1,
                        INNER_Z0, INNER_Z0 + rail_h,
                        outer_rear_y, rail_w)
        head_or = _slab(INNER_X0, INNER_X1,
                        INNER_Z1 - rail_h, INNER_Z1,
                        outer_rear_y, rail_w)
        rails = rails.union(sill_or).union(head_or)

    return rails


def _build_sash_grille_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Sash ring + colonial muntin grille, in sash-local frame centered at origin."""
    ow = opening_w
    oh = opening_h
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE

    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0,
                  0.0, SASH_DEPTH)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0,
                    0.0, SASH_DEPTH + 0.02)
    ring = outer.cut(opening)

    bars = None

    # Vertical muntins
    for c in range(1, GRILLE_COLS):
        frac = c / GRILLE_COLS
        x = -ow / 2.0 + frac * ow
        bar = _slab(x - MUNTIN_T / 2.0, x + MUNTIN_T / 2.0,
                    -oh / 2.0, oh / 2.0, 0.0, MUNTIN_DEPTH)
        bars = bar if bars is None else bars.union(bar)

    # Horizontal muntins
    for r in range(1, GRILLE_ROWS):
        frac = r / GRILLE_ROWS
        z = -oh / 2.0 + frac * oh
        bar = _slab(-ow / 2.0, ow / 2.0,
                    z - MUNTIN_T / 2.0, z + MUNTIN_T / 2.0,
                    0.0, MUNTIN_DEPTH)
        bars = bar if bars is None else bars.union(bar)

    return ring if bars is None else ring.union(bars)


def _build_sash_glass_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Clear pane filling the sash opening, in sash-local frame."""
    ow = opening_w + 2 * REBATE
    oh = opening_h + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_fixed_panel_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Fixed panel vinyl ring (no grille), in panel-local frame."""
    ow = opening_w
    oh = opening_h
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE

    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0,
                  0.0, CENTER_PANEL_DEPTH)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0,
                    0.0, CENTER_PANEL_DEPTH + 0.02)
    return outer.cut(opening)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    span = SIDE_LITE_W + MULLION_FACE + CENTER_LITE_W + MULLION_FACE + SIDE_LITE_W
    inner_w = INNER_X1 - INNER_X0
    assert abs(span - inner_w) < 1e-6, f"layout {span} != inner {inner_w}"

    model = ArticulatedObject(name="double_sliding_window")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("track", rgba=TRACK_RGBA)

    # --- Frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="vinyl",
        name="frame_shell",
    )
    frame.visual(
        mesh_from_cadquery(_build_track_rail_shape(), "track_rails"),
        material="track",
        name="track_rails",
    )

    opening_h = INNER_Z1 - INNER_Z0

    # --- Center fixed panel (plain glass, no muntin grille) ---
    center_panel = model.part("center_panel")
    center_panel.visual(
        mesh_from_cadquery(
            _build_fixed_panel_shape(CENTER_LITE_W, opening_h),
            "center_panel_vinyl",
        ),
        material="vinyl",
        name="center_panel_vinyl",
    )
    center_panel.visual(
        mesh_from_cadquery(
            _build_sash_glass_shape(CENTER_LITE_W, opening_h),
            "center_panel_glass",
        ),
        material="glass",
        name="center_panel_glass",
    )

    # --- Front sash (front track, slides right +X) with muntin grille ---
    front_sash = model.part("front_sash")
    front_sash.visual(
        mesh_from_cadquery(
            _build_sash_grille_shape(SIDE_LITE_W, opening_h),
            "front_sash_vinyl",
        ),
        material="vinyl",
        name="front_sash_vinyl",
    )
    front_sash.visual(
        mesh_from_cadquery(
            _build_sash_glass_shape(SIDE_LITE_W, opening_h),
            "front_sash_glass",
        ),
        material="glass",
        name="front_sash_glass",
    )

    # --- Rear sash (rear track, slides left -X) with muntin grille ---
    rear_sash = model.part("rear_sash")
    rear_sash.visual(
        mesh_from_cadquery(
            _build_sash_grille_shape(SIDE_LITE_W, opening_h),
            "rear_sash_vinyl",
        ),
        material="vinyl",
        name="rear_sash_vinyl",
    )
    rear_sash.visual(
        mesh_from_cadquery(
            _build_sash_glass_shape(SIDE_LITE_W, opening_h),
            "rear_sash_glass",
        ),
        material="glass",
        name="rear_sash_glass",
    )

    # --- Part world positions ---
    left_cx = (LEFT_X0 + LEFT_X1) / 2.0
    right_cx = (RIGHT_X0 + RIGHT_X1) / 2.0
    center_cx = (CENTER_X0 + CENTER_X1) / 2.0
    mid_cz = (INNER_Z0 + INNER_Z1) / 2.0

    # Center fixed panel: FIXED joint at center opening, Y=0
    model.articulation(
        "frame_to_center_panel",
        ArticulationType.FIXED,
        parent="frame",
        child="center_panel",
        origin=Origin(xyz=(center_cx, 0.0, mid_cz)),
    )

    # Front sash: PRISMATIC +X on front track (+Y)
    travel = SIDE_LITE_W * 0.85
    model.articulation(
        "frame_to_front_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="front_sash",
        origin=Origin(xyz=(left_cx, FRONT_TRACK_Y, mid_cz)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.5, lower=0.0, upper=travel,
        ),
    )

    # Rear sash: PRISMATIC -X on rear track (-Y)
    model.articulation(
        "frame_to_rear_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="rear_sash",
        origin=Origin(xyz=(right_cx, REAR_TRACK_Y, mid_cz)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.5, lower=0.0, upper=travel,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    center_panel = object_model.get_part("center_panel")
    front_sash = object_model.get_part("front_sash")
    rear_sash = object_model.get_part("rear_sash")
    front_slide = object_model.get_articulation("frame_to_front_sash")
    rear_slide = object_model.get_articulation("frame_to_rear_sash")
    center_joint = object_model.get_articulation("frame_to_center_panel")

    # --- Verify articulation types ---
    ctx.check(
        "front_sash has prismatic joint",
        front_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={front_slide.articulation_type}",
    )
    ctx.check(
        "rear_sash has prismatic joint",
        rear_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={rear_slide.articulation_type}",
    )
    ctx.check(
        "center_panel is fixed",
        center_joint.articulation_type == ArticulationType.FIXED,
        details=f"type={center_joint.articulation_type}",
    )

    # --- Verify opposite slide directions ---
    fa = front_slide.axis
    ra = rear_slide.axis
    ctx.check(
        "front and rear sash slide in opposite X directions",
        fa[0] * ra[0] < 0.0,
        details=f"front_axis={fa}, rear_axis={ra}",
    )

    # --- Intentional overlaps ---
    # Glass rebated under sash/muntin lip on movable sashes
    for nm in ("front_sash", "rear_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_vinyl",
            reason="Glass pane rebated under sash/muntin lip (captured glazing).",
        )
    # Center panel glass rebated under panel ring
    ctx.allow_overlap(
        "center_panel", "center_panel",
        elem_a="center_panel_glass",
        elem_b="center_panel_vinyl",
        reason="Center panel glass rebated under fixed panel ring (captured glazing).",
    )
    # Frame-to-sash track engagement (sash rails ride in head/sill grooves)
    ctx.allow_overlap(
        "frame", "front_sash",
        elem_a="frame_shell",
        elem_b="front_sash_vinyl",
        reason="Front sash top/bottom rails engage the head/sill track grooves (track capture).",
    )
    ctx.allow_overlap(
        "frame", "rear_sash",
        elem_a="frame_shell",
        elem_b="rear_sash_vinyl",
        reason="Rear sash top/bottom rails engage the head/sill track grooves (track capture).",
    )
    # Glass rebated under frame opening lip (captured glazing at jamb/mullion/head/sill)
    ctx.allow_overlap(
        "frame", "front_sash",
        elem_a="frame_shell",
        elem_b="front_sash_glass",
        reason="Front sash glass is rebated under the frame opening lip (captured glazing).",
    )
    ctx.allow_overlap(
        "frame", "rear_sash",
        elem_a="frame_shell",
        elem_b="rear_sash_glass",
        reason="Rear sash glass is rebated under the frame opening lip (captured glazing).",
    )
    # Frame-to-center-panel seating
    ctx.allow_overlap(
        "frame", "center_panel",
        reason="Center fixed panel is rebated into the frame opening (fixed lite capture).",
    )
    ctx.allow_overlap(
        "frame", "center_panel",
        elem_a="frame_shell",
        elem_b="center_panel_glass",
        reason="Center panel glass rebated under frame opening lip (captured glazing).",
    )
    # Track rails sit on the frame sill/head surfaces
    ctx.allow_overlap(
        "frame", "frame",
        elem_a="frame_shell",
        elem_b="track_rails",
        reason="Track guide rails are integral to the head/sill track profile.",
    )

    # --- Closed pose (q=0): window reads SHUT ---
    with ctx.pose({front_slide: 0.0, rear_slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        f_aabb = ctx.part_world_aabb(front_sash)
        r_aabb = ctx.part_world_aabb(rear_sash)
        c_aabb = ctx.part_world_aabb(center_panel)

        # Frame spans the full width
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        ctx.check(
            "frame spans full window width",
            frame_w > 2.5,
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

        # Sashes on separate Y tracks
        f_y = (f_aabb[0][1] + f_aabb[1][1]) / 2.0
        r_y = (r_aabb[0][1] + r_aabb[1][1]) / 2.0
        ctx.check(
            "sashes on separate Y tracks",
            abs(f_y - r_y) > 0.04,
            details=f"front_y={f_y:.3f}, rear_y={r_y:.3f}",
        )
        ctx.check(
            "front sash on +Y side of rear sash",
            f_y > r_y + 0.04,
            details=f"front_y={f_y:.3f}, rear_y={r_y:.3f}",
        )

        # Sashes flanking center panel: front(left), center, rear(right)
        fx = (f_aabb[0][0] + f_aabb[1][0]) / 2.0
        cx = (c_aabb[0][0] + c_aabb[1][0]) / 2.0
        rx = (r_aabb[0][0] + r_aabb[1][0]) / 2.0
        ctx.check(
            "closed: front_sash left, center panel, rear_sash right",
            fx < cx < rx,
            details=f"front_x={fx:.3f}, center_x={cx:.3f}, rear_x={rx:.3f}",
        )

        # All parts within frame height
        for nm, ab in [("front_sash", f_aabb), ("rear_sash", r_aabb),
                       ("center_panel", c_aabb)]:
            ctx.check(
                f"{nm} within frame height",
                ab[0][2] > frame_aabb[0][2] - 1e-4
                and ab[1][2] < frame_aabb[1][2] + 1e-4,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}]",
            )

        # Sashes retained in frame X span
        for nm, ab in [("front_sash", f_aabb), ("rear_sash", r_aabb)]:
            ctx.check(
                f"{nm} within frame X at rest",
                ab[0][0] > frame_aabb[0][0] - 0.01
                and ab[1][0] < frame_aabb[1][0] + 0.01,
                details=f"{nm} x=[{ab[0][0]:.3f},{ab[1][0]:.3f}]",
            )

        # Proof: glass panes seated within the frame opening (XZ projection)
        ctx.expect_overlap(
            front_sash, frame, axes="xz", min_overlap=0.50,
            elem_a="front_sash_glass", elem_b="frame_shell",
            name="front sash glass seated in frame opening",
        )
        ctx.expect_overlap(
            rear_sash, frame, axes="xz", min_overlap=0.50,
            elem_a="rear_sash_glass", elem_b="frame_shell",
            name="rear sash glass seated in frame opening",
        )

        rest_fx = fx
        rest_rx = rx
        rest_fz = (f_aabb[0][2] + f_aabb[1][2]) / 2.0
        rest_rz = (r_aabb[0][2] + r_aabb[1][2]) / 2.0

    # --- Open pose: both sashes slide toward center ---
    travel = front_slide.motion_limits.upper
    with ctx.pose({front_slide: travel, rear_slide: travel}):
        f_open = ctx.part_world_aabb(front_sash)
        r_open = ctx.part_world_aabb(rear_sash)

        open_fx = (f_open[0][0] + f_open[1][0]) / 2.0
        open_rx = (r_open[0][0] + r_open[1][0]) / 2.0

        # Front sash slid right (+X)
        ctx.check(
            "front sash slides right (+X)",
            open_fx > rest_fx + 0.3,
            details=f"rest_fx={rest_fx:.3f}, open_fx={open_fx:.3f}, travel={travel:.3f}",
        )

        # Rear sash slid left (-X)
        ctx.check(
            "rear sash slides left (-X)",
            open_rx < rest_rx - 0.3,
            details=f"rest_rx={rest_rx:.3f}, open_rx={open_rx:.3f}, travel={travel:.3f}",
        )

        # Pure horizontal slide (no Z change)
        open_fz = (f_open[0][2] + f_open[1][2]) / 2.0
        open_rz = (r_open[0][2] + r_open[1][2]) / 2.0
        ctx.check(
            "front sash slide is purely horizontal",
            abs(open_fz - rest_fz) < 0.02,
            details=f"rest_fz={rest_fz:.3f}, open_fz={open_fz:.3f}",
        )
        ctx.check(
            "rear sash slide is purely horizontal",
            abs(open_rz - rest_rz) < 0.02,
            details=f"rest_rz={rest_rz:.3f}, open_rz={open_rz:.3f}",
        )

        # Both sashes retained within frame at full travel
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "front sash retained in frame at full travel",
            f_open[0][0] > f_aabb[0][0] - 0.01
            and f_open[1][0] < f_aabb[1][0] + 0.01,
            details=f"front x=[{f_open[0][0]:.3f},{f_open[1][0]:.3f}]",
        )
        ctx.check(
            "rear sash retained in frame at full travel",
            r_open[0][0] > f_aabb[0][0] - 0.01
            and r_open[1][0] < f_aabb[1][0] + 0.01,
            details=f"rear x=[{r_open[0][0]:.3f},{r_open[1][0]:.3f}]",
        )

        # Vertical engagement with head/sill track maintained
        ctx.expect_overlap(
            front_sash, frame, axes="z", min_overlap=0.10,
            name="front sash retains vertical track engagement",
        )
        ctx.expect_overlap(
            rear_sash, frame, axes="z", min_overlap=0.10,
            name="rear sash retains vertical track engagement",
        )

    return ctx.report()


object_model = build_object_model()
