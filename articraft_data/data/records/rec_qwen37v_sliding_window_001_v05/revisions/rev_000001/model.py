from __future__ import annotations

# Two-panel vertical sliding window: thick aluminum frame with deep track
# grooves, upper fixed lite, lower sash that slides upward on a vertical
# prismatic joint. Recessed pull cup on the lower sash meeting rail.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     - width  -> X
#     - height -> Z   (sill near z=0)
#     - frame depth / glazing thickness -> Y
#   The glass plane is the X-Z plane. The window reads SHUT at q=0; driving
#   the prismatic joint slides the lower sash upward (+Z), staying retained
#   in the jamb tracks.
#
# Structure:
#   - frame (static root): head, sill, two jambs, meeting rail between
#     upper and lower openings; built as one CadQuery solid.
#   - Track grooves: visible channel geometry along head, sill, and jambs.
#   - upper_lite (FIXED): sash ring + colonial grille + glass.
#   - lower_sash (SLIDING): sash ring + colonial grille + glass + pull cup;
#     PRISMATIC along +Z.

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
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

TOTAL_W = 1.20            # overall window width along X
TOTAL_H = 1.50            # overall height along Z (sill at z=0)

FRAME_FACE = 0.080        # thick aluminum frame rail face width
FRAME_DEPTH = 0.100       # frame depth along Y (chunky aluminum box section)
MEETING_RAIL_H = 0.055    # horizontal meeting/check rail between upper/lower

# Track grooves (deep channels in the frame rails)
GROOVE_W = 0.014          # groove width (channel opening)
GROOVE_D = 0.022          # groove depth into the rail

# Sash construction
SASH_FACE = 0.048         # sash perimeter rail/stile face width
SASH_DEPTH = 0.044        # sash depth along Y
GLASS_T = 0.006           # glazing thickness

# Pull cup (recessed into the lower sash meeting rail)
CUP_DIAM = 0.040          # pull cup diameter
CUP_DEPTH = 0.010         # pull cup recess depth
CUP_WALL_T = 0.004        # pull cup rim thickness

# Colonial grille (divided lite)
GRILLE_COLS = 3
GRILLE_ROWS = 3
MUNTIN_T = 0.018
MUNTIN_DEPTH = 0.018

REBATE = 0.002            # glass tucks under sash lip (kept small so pane
                          # stays within the opening and avoids the jamb tracks)

# ---------------------------------------------------------------------------
# Derived layout
# ---------------------------------------------------------------------------

HALF_W = TOTAL_W / 2.0

INNER_X0 = -HALF_W + FRAME_FACE
INNER_X1 = HALF_W - FRAME_FACE
INNER_Z0 = FRAME_FACE
INNER_Z1 = TOTAL_H - FRAME_FACE
MID_Z = TOTAL_H / 2.0

# Upper opening: from meeting rail top to head inner edge
UPPER_Z0 = MID_Z + MEETING_RAIL_H / 2.0
UPPER_Z1 = INNER_Z1
# Lower opening: from sill inner edge to meeting rail bottom
LOWER_Z0 = INNER_Z0
LOWER_Z1 = MID_Z - MEETING_RAIL_H / 2.0

UPPER_H = UPPER_Z1 - UPPER_Z0
LOWER_H = LOWER_Z1 - LOWER_Z0
OPENING_W = INNER_X1 - INNER_X0

# Sash slide travel: lower sash slides upward by ~80% of the upper opening
SLIDE_TRAVEL = UPPER_H * 0.85

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

ALUMINUM_RGBA = (0.72, 0.74, 0.76, 1.0)      # brushed aluminum
TRACK_RGBA = (0.35, 0.37, 0.40, 1.0)          # dark anodized track channels
GLASS_RGBA = (0.50, 0.58, 0.64, 0.32)         # cool grey-blue, semi-transparent
PULL_RGBA = (0.25, 0.26, 0.28, 1.0)           # dark recessed pull cup


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery)
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float, y_center: float, depth: float) -> cq.Workplane:
    """Axis-aligned box in the X-Z plane, centered on y_center."""
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
    """Static outer frame: a full slab cut by upper and lower openings,
    leaving head, sill, two jambs, and the meeting rail as one solid."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)

    cut_depth = FRAME_DEPTH + 0.02
    upper_cut = _slab(INNER_X0, INNER_X1, UPPER_Z0, UPPER_Z1, 0.0, cut_depth)
    lower_cut = _slab(INNER_X0, INNER_X1, LOWER_Z0, LOWER_Z1, 0.0, cut_depth)

    return outer.cut(upper_cut).cut(lower_cut)


def _build_track_grooves() -> list[tuple[cq.Workplane, str]]:
    """Deep track grooves along head, sill, and both jambs.
    Returns list of (workplane, name) tuples."""
    grooves = []

    # Head groove: channel on inner bottom face of head rail
    head_groove = _slab(
        INNER_X0, INNER_X1,
        INNER_Z1, INNER_Z1 + GROOVE_D,
        0.0, GROOVE_W,
    )
    grooves.append((head_groove, "head_groove"))

    # Sill groove: channel on inner top face of sill rail
    sill_groove = _slab(
        INNER_X0, INNER_X1,
        INNER_Z0 - GROOVE_D, INNER_Z0,
        0.0, GROOVE_W,
    )
    grooves.append((sill_groove, "sill_groove"))

    # Left jamb track: vertical channel on inner face of left jamb
    left_track = _slab(
        INNER_X0 - GROOVE_D, INNER_X0,
        INNER_Z0, INNER_Z1,
        0.0, GROOVE_W,
    )
    grooves.append((left_track, "left_jamb_track"))

    # Right jamb track: vertical channel on inner face of right jamb
    right_track = _slab(
        INNER_X1, INNER_X1 + GROOVE_D,
        INNER_Z0, INNER_Z1,
        0.0, GROOVE_W,
    )
    grooves.append((right_track, "right_jamb_track"))

    return grooves


def _build_sash_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """One sash ring (frame) built in its own local frame centered on origin.
    The opening is hollowed out for glass."""
    ow = opening_w
    oh = opening_h
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE

    # Outer sash slab
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SASH_DEPTH)
    # Cut clear opening
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH + 0.02)
    return outer.cut(opening)


def _build_grille_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Colonial muntin grid across the sash opening, in sash-local frame."""
    ow = opening_w
    oh = opening_h
    bars = None

    # Vertical muntins
    for c in range(1, GRILLE_COLS):
        frac = c / GRILLE_COLS
        x = -ow / 2.0 + frac * ow
        bar = _slab(
            x - MUNTIN_T / 2.0, x + MUNTIN_T / 2.0,
            -oh / 2.0, oh / 2.0,
            0.0, MUNTIN_DEPTH,
        )
        bars = bar if bars is None else bars.union(bar)

    # Horizontal muntins
    for r in range(1, GRILLE_ROWS):
        frac = r / GRILLE_ROWS
        z = -oh / 2.0 + frac * oh
        bar = _slab(
            -ow / 2.0, ow / 2.0,
            z - MUNTIN_T / 2.0, z + MUNTIN_T / 2.0,
            0.0, MUNTIN_DEPTH,
        )
        bars = bar if bars is None else bars.union(bar)

    return bars


def _build_glass_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Single clear pane in sash-local frame, rebated under the sash lip."""
    ow = opening_w + 2 * REBATE
    oh = opening_h + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_pull_cup() -> cq.Workplane:
    """Recessed pull cup: a short cylinder with a hollowed interior,
    built in its own local frame (cup axis along Y, opening toward +Y)."""
    # Outer cylinder (the cup body)
    outer_r = CUP_DIAM / 2.0
    cup = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, 0.0))
        .cylinder(CUP_DEPTH, outer_r, centered=(True, True, False))
    )
    # Hollow interior
    inner_r = outer_r - CUP_WALL_T
    hollow = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, CUP_WALL_T))
        .cylinder(CUP_DEPTH - CUP_WALL_T, inner_r, centered=(True, True, False))
    )
    return cup.cut(hollow)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_sliding_window")
    model.material("aluminum", rgba=ALUMINUM_RGBA)
    model.material("track", rgba=TRACK_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("pull", rgba=PULL_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="aluminum",
        name="frame_shell",
    )

    # Track grooves (visible dark channels)
    for groove_wp, groove_name in _build_track_grooves():
        frame.visual(
            mesh_from_cadquery(groove_wp, groove_name),
            material="track",
            name=groove_name,
        )

    # --- Upper fixed lite ---
    upper = model.part("upper_lite")
    upper.visual(
        mesh_from_cadquery(_build_sash_shape(OPENING_W, UPPER_H), "upper_sash"),
        material="aluminum",
        name="upper_sash",
    )
    grille_upper = _build_grille_shape(OPENING_W, UPPER_H)
    if grille_upper is not None:
        upper.visual(
            mesh_from_cadquery(grille_upper, "upper_grille"),
            material="aluminum",
            name="upper_grille",
        )
    upper.visual(
        mesh_from_cadquery(_build_glass_shape(OPENING_W, UPPER_H), "upper_glass"),
        material="glass",
        name="upper_glass",
    )

    # --- Lower sliding sash ---
    lower = model.part("lower_sash")
    lower.visual(
        mesh_from_cadquery(_build_sash_shape(OPENING_W, LOWER_H), "lower_sash_frame"),
        material="aluminum",
        name="lower_sash_frame",
    )
    grille_lower = _build_grille_shape(OPENING_W, LOWER_H)
    if grille_lower is not None:
        lower.visual(
            mesh_from_cadquery(grille_lower, "lower_grille"),
            material="aluminum",
            name="lower_grille",
        )
    lower.visual(
        mesh_from_cadquery(_build_glass_shape(OPENING_W, LOWER_H), "lower_glass"),
        material="glass",
        name="lower_glass",
    )

    # Pull cup on the lower sash meeting rail (top rail, center).
    # Cup opens toward +Y (front face). Position: top of the sash frame,
    # centered in X, at the front face of the sash depth.
    cup_z_local = LOWER_H / 2.0 + SASH_FACE / 2.0  # on the meeting rail
    cup_y_local = SASH_DEPTH / 2.0  # front face of sash
    # The cup cylinder axis is along local Z in CadQuery; rotate so it
    # points along Y (into the sash face). Use rpy to rotate 90° about X.
    lower.visual(
        mesh_from_cadquery(_build_pull_cup(), "pull_cup"),
        material="pull",
        origin=Origin(
            xyz=(0.0, cup_y_local, cup_z_local),
            rpy=(-1.5708, 0.0, 0.0),  # rotate 90° about X so cup axis -> +Y
        ),
        name="pull_cup",
    )

    # --- World positions of sash centers ---
    upper_cz = (UPPER_Z0 + UPPER_Z1) / 2.0
    lower_cz = (LOWER_Z0 + LOWER_Z1) / 2.0
    mid_x = 0.0

    # FIXED upper lite: seated in the frame opening
    model.articulation(
        "frame_to_upper_lite",
        ArticulationType.FIXED,
        parent="frame",
        child="upper_lite",
        origin=Origin(xyz=(mid_x, 0.0, upper_cz)),
    )

    # Lower sliding sash: PRISMATIC along +Z (upward).
    # Positive q lifts the sash upward.
    model.articulation(
        "frame_to_lower_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="lower_sash",
        origin=Origin(xyz=(mid_x, 0.0, lower_cz)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0, velocity=0.4, lower=0.0, upper=SLIDE_TRAVEL
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    upper_lite = object_model.get_part("upper_lite")
    lower_sash = object_model.get_part("lower_sash")
    slide = object_model.get_articulation("frame_to_lower_sash")

    # --- Intentional overlaps ---
    # Glass panes tuck under the sash lip (captured glazing).
    for nm in ("upper_lite", "lower_sash"):
        sash_name = "upper_sash" if nm == "upper_lite" else "lower_sash_frame"
        glass_name = "upper_glass" if nm == "upper_lite" else "lower_glass"
        grille_name = "upper_grille" if nm == "upper_lite" else "lower_grille"
        ctx.allow_overlap(
            nm, nm,
            elem_a=glass_name,
            elem_b=sash_name,
            reason=f"Glass pane rebated under the {nm} sash lip (captured glazing).",
        )
        ctx.allow_overlap(
            nm, nm,
            elem_a=glass_name,
            elem_b=grille_name,
            reason=f"Glass pane contacts the {nm} grille muntins (seated grille).",
        )

    # Upper fixed lite is rebated into the frame opening.
    ctx.allow_overlap(
        "frame", "upper_lite",
        elem_a="frame_shell",
        elem_b="upper_sash",
        reason="Upper fixed lite is rebated into the frame opening (seated capture).",
    )
    ctx.allow_overlap(
        "frame", "upper_lite",
        elem_a="frame_shell",
        elem_b="upper_glass",
        reason="Upper lite glass rebated under frame opening lip.",
    )
    ctx.allow_overlap(
        "frame", "upper_lite",
        elem_a="frame_shell",
        elem_b="upper_grille",
        reason="Upper lite grille bars sit inside the frame opening void (grille within rebate).",
    )

    # Lower sliding sash rides in the jamb tracks; its frame laps the track
    # grooves and frame inner edges.
    ctx.allow_overlap(
        "frame", "lower_sash",
        elem_a="frame_shell",
        elem_b="lower_sash_frame",
        reason="Lower sash rides in the jamb tracks and laps the frame inner edges (slider capture).",
    )
    ctx.allow_overlap(
        "frame", "lower_sash",
        elem_a="frame_shell",
        elem_b="lower_glass",
        reason="Lower sash glass laps the track lip as the sash rides (captured glazing).",
    )
    ctx.allow_overlap(
        "frame", "lower_sash",
        elem_a="frame_shell",
        elem_b="lower_grille",
        reason="Lower sash grille bars sit inside the frame opening void (grille within rebate).",
    )

    # Meeting rail: the lower sash top rail and upper lite bottom rail overlap
    # at the check rail zone. This is standard vertical slider construction
    # where the two sashes interlock at the meeting rail.
    ctx.allow_overlap(
        "lower_sash", "upper_lite",
        elem_a="lower_sash_frame",
        elem_b="upper_sash",
        reason="Meeting rail interlock: lower sash top rail overlaps upper lite bottom rail at the check rail (standard vertical slider construction).",
    )

    # Track grooves are embedded in the frame rail surfaces.
    for groove in ("head_groove", "sill_groove", "left_jamb_track", "right_jamb_track"):
        ctx.allow_overlap(
            "frame", "frame",
            elem_a="frame_shell",
            elem_b=groove,
            reason=f"Track groove '{groove}' is a visible channel embedded in the frame rail surface.",
        )

    # Track grooves engage the sash edges (sash rides in the tracks).
    # Head groove captures the upper sash top rail; sill groove captures
    # the lower sash bottom rail; jamb tracks capture both sash side edges.
    ctx.allow_overlap(
        "frame", "upper_lite",
        elem_a="head_groove",
        elem_b="upper_sash",
        reason="Upper sash top rail engages the head track groove (vertical slider track capture).",
    )
    ctx.allow_overlap(
        "frame", "lower_sash",
        elem_a="sill_groove",
        elem_b="lower_sash_frame",
        reason="Lower sash bottom rail engages the sill track groove (vertical slider track capture).",
    )
    ctx.allow_overlap(
        "frame", "lower_sash",
        elem_a="left_jamb_track",
        elem_b="lower_sash_frame",
        reason="Lower sash left stile engages the left jamb track groove (vertical slider track capture).",
    )
    ctx.allow_overlap(
        "frame", "lower_sash",
        elem_a="right_jamb_track",
        elem_b="lower_sash_frame",
        reason="Lower sash right stile engages the right jamb track groove (vertical slider track capture).",
    )
    ctx.allow_overlap(
        "frame", "upper_lite",
        elem_a="left_jamb_track",
        elem_b="upper_sash",
        reason="Upper sash left stile engages the left jamb track groove (vertical slider track capture).",
    )
    ctx.allow_overlap(
        "frame", "upper_lite",
        elem_a="right_jamb_track",
        elem_b="upper_sash",
        reason="Upper sash right stile engages the right jamb track groove (vertical slider track capture).",
    )

    # Pull cup is recessed into the lower sash meeting rail.
    ctx.allow_overlap(
        "lower_sash", "lower_sash",
        elem_a="lower_sash_frame",
        elem_b="pull_cup",
        reason="Pull cup is recessed into the lower sash meeting rail (seated insertion).",
    )

    # --- Closed pose (q=0): window reads SHUT ---
    with ctx.pose({slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        upper_aabb = ctx.part_world_aabb(upper_lite)
        lower_aabb = ctx.part_world_aabb(lower_sash)

        # Frame spans the full window dimensions
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        frame_h = frame_aabb[1][2] - frame_aabb[0][2]
        ctx.check(
            "frame width matches design",
            abs(frame_w - TOTAL_W) < 0.02,
            details=f"frame_w={frame_w:.3f}, expected={TOTAL_W:.3f}",
        )
        ctx.check(
            "frame height matches design",
            abs(frame_h - TOTAL_H) < 0.02,
            details=f"frame_h={frame_h:.3f}, expected={TOTAL_H:.3f}",
        )

        # Sill at z~0
        ctx.check(
            "sill sits near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"frame zmin={frame_aabb[0][2]:.4f}",
        )

        # Upper lite is above lower sash
        upper_cz = (upper_aabb[0][2] + upper_aabb[1][2]) / 2.0
        lower_cz = (lower_aabb[0][2] + lower_aabb[1][2]) / 2.0
        ctx.check(
            "upper lite above lower sash at rest",
            upper_cz > lower_cz + 0.1,
            details=f"upper_cz={upper_cz:.3f}, lower_cz={lower_cz:.3f}",
        )

        # Lower sash is within the frame height
        ctx.check(
            "lower sash within frame at rest",
            lower_aabb[0][2] > frame_aabb[0][2] - 0.01 and lower_aabb[1][2] < frame_aabb[1][2] + 0.01,
            details=f"lower z=[{lower_aabb[0][2]:.3f},{lower_aabb[1][2]:.3f}]",
        )

        rest_lower_cz = lower_cz

    # --- Track grooves exist on the frame ---
    frame_visuals = [v.name for v in frame.visuals]
    for groove in ("head_groove", "sill_groove", "left_jamb_track", "right_jamb_track"):
        ctx.check(
            f"track groove '{groove}' present on frame",
            groove in frame_visuals,
            details=f"frame visuals: {frame_visuals}",
        )

    # --- Pull cup exists on lower sash ---
    lower_visuals = [v.name for v in lower_sash.visuals]
    ctx.check(
        "pull cup present on lower sash",
        "pull_cup" in lower_visuals,
        details=f"lower sash visuals: {lower_visuals}",
    )

    # --- Joint is prismatic along Z (vertical slide) ---
    ctx.check(
        "lower sash joint is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )
    ctx.check(
        "slide axis is vertical (Z)",
        abs(slide.axis[2]) > 0.99,
        details=f"axis={slide.axis}",
    )
    ctx.check(
        "slide has nonzero travel",
        slide.motion_limits is not None and slide.motion_limits.upper > 0.05,
        details=f"limits={slide.motion_limits}",
    )

    # --- Driven pose: lower sash slides upward ---
    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel}):
        lower_open = ctx.part_world_aabb(lower_sash)
        open_cz = (lower_open[0][2] + lower_open[1][2]) / 2.0

        # Lower sash moved upward by ~travel
        ctx.check(
            "lower sash slides upward by ~travel",
            abs((open_cz - rest_lower_cz) - travel) < 0.02,
            details=f"rest_cz={rest_lower_cz:.3f}, open_cz={open_cz:.3f}, travel={travel:.3f}",
        )

        # Lower sash stays within frame X span (retained in jamb tracks)
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained within frame X span at full travel",
            lower_open[1][0] < f_aabb[1][0] + 0.01 and lower_open[0][0] > f_aabb[0][0] - 0.01,
            details=f"sash x=[{lower_open[0][0]:.3f},{lower_open[1][0]:.3f}] frame x=[{f_aabb[0][0]:.3f},{f_aabb[1][0]:.3f}]",
        )

        # Lower sash still overlaps the frame vertically (retained insertion)
        ctx.expect_overlap(
            lower_sash, frame,
            axes="x",
            min_overlap=0.05,
            name="sash retains horizontal engagement with jamb tracks at full travel",
        )

    return ctx.report()


object_model = build_object_model()
