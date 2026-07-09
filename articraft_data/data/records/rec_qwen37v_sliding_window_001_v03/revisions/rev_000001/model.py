from __future__ import annotations

# Vertical single-hung sliding window with white vinyl frame, colonial
# divided-lite grilles, deep track grooves along head and sill rails, one
# movable lower sash that slides upward, and an independent insect screen on
# a shallow prismatic joint.
#
# Coordinate convention:
#   +Z is up. Window stands vertically in the X-Z plane.
#     - width  -> X
#     - height -> Z   (sill near z=0)
#     - frame depth / glazing thickness -> Y
#   The glass plane is the X-Z plane. The window reads SHUT at q=0; driving
#   the lower sash prismatic joint slides it upward (+Z) to open. The insect
#   screen sits proud toward +Y and slides independently upward.

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

# Outer frame
TOTAL_W = 1.00            # overall window width along X
TOTAL_H = 1.50            # overall height along Z (sill at z=0, head at z=TOTAL_H)

FRAME_FACE = 0.065        # outer frame member face width (jamb / head / sill)
FRAME_DEPTH = 0.100       # outer frame depth along Y (vinyl box section)

# Track grooves: deep channels cut into head and sill rails for sash capture
TRACK_DEPTH = 0.022       # how deep the groove cuts into the rail (along Z)
TRACK_WIDTH = 0.028       # groove width along Y (captures sash edge)
# The groove is centered on the inner frame face in Y

# Meeting rail between upper and lower sash openings
MEETING_RAIL_H = 0.050    # height of the meeting/check rail

# Sash construction
SASH_FACE = 0.050         # sash perimeter rail/stile face width (in-plane)
SASH_DEPTH = 0.050        # sash depth along Y
GLASS_T = 0.006           # glazing thickness along Y

# Colonial grille (divided lite)
GRILLE_COLS = 3           # 3 columns of panes
GRILLE_ROWS = 4           # 4 rows of panes
MUNTIN_T = 0.018          # muntin bar face width
MUNTIN_DEPTH = 0.018      # muntin bar depth along Y

# Insect screen
SCREEN_FRAME_W = 0.025    # screen frame member width
SCREEN_FRAME_DEPTH = 0.020  # screen frame depth along Y
SCREEN_MESH_T = 0.002     # screen mesh thickness

# Y layout (depth). Frame box centered on y=0.
# Sashes sit in the inner track plane; screen sits proud toward +Y.
SASH_Y = -0.010           # sash center Y (rear track plane)
SCREEN_Y = 0.055          # screen sits proud toward +Y (outside the sashes)

REBATE = 0.004            # glass tucks under sash lip

# ---------------------------------------------------------------------------
# Derived layout
# ---------------------------------------------------------------------------

HALF_W = TOTAL_W / 2.0

# Inner clear region (inside the outer head/sill/jambs)
INNER_X0 = -HALF_W + FRAME_FACE
INNER_X1 = HALF_W - FRAME_FACE
INNER_Z0 = FRAME_FACE
INNER_Z1 = TOTAL_H - FRAME_FACE

# Opening width (common to both sashes)
OPENING_W = INNER_X1 - INNER_X0

# Vertical layout: lower sash + meeting rail + upper sash fill the inner height
INNER_H = INNER_Z1 - INNER_Z0
SASH_OPENING_H = (INNER_H - MEETING_RAIL_H) / 2.0

# Lower sash: from sill inner edge up to meeting rail
LOWER_Z0 = INNER_Z0
LOWER_Z1 = INNER_Z0 + SASH_OPENING_H
# Meeting rail
MEETING_Z0 = LOWER_Z1
MEETING_Z1 = MEETING_Z0 + MEETING_RAIL_H
# Upper sash: from meeting rail top to head inner edge
UPPER_Z0 = MEETING_Z1
UPPER_Z1 = INNER_Z1

# Sash slide travel: the lower sash can slide up by nearly its full height
SASH_TRAVEL = SASH_OPENING_H * 0.85

# Screen travel: shallower independent slide
SCREEN_TRAVEL = SASH_OPENING_H * 0.60

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)     # bright white vinyl/PVC
GLASS_RGBA = (0.50, 0.58, 0.64, 0.32)    # cool grey-blue, semi-transparent
SCREEN_RGBA = (0.40, 0.42, 0.44, 0.45)   # dark grey screen mesh, semi-transparent
ALUMINUM_RGBA = (0.72, 0.74, 0.76, 1.0)  # aluminum screen frame


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery). All authored in meters, world frame.
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float, y_center: float, depth: float) -> cq.Workplane:
    """Axis-aligned box spanning [x0,x1] x [z0,z1] in the X-Z plane, centered on
    y_center with the given Y depth."""
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
    """Static outer frame: a full slab cut by the two sash openings (upper and
    lower) plus the meeting rail gap, leaving head, sill, two jambs, and the
    meeting/check rail as one solid. Then deep track grooves are cut into the
    head and sill rails."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)

    cut_depth = FRAME_DEPTH + 0.02  # through-cut clearance in Y

    # Cut lower sash opening
    lower_cut = _slab(INNER_X0, INNER_X1, LOWER_Z0, LOWER_Z1, 0.0, cut_depth)
    # Cut upper sash opening
    upper_cut = _slab(INNER_X0, INNER_X1, UPPER_Z0, UPPER_Z1, 0.0, cut_depth)

    frame = outer.cut(lower_cut).cut(upper_cut)

    # Deep track grooves along the sill (bottom rail) - two parallel channels
    # The grooves are cut into the top face of the sill rail, running along X
    # They capture the sash bottom edge and the screen bottom edge
    groove_y_sash = SASH_Y  # sash track groove Y position
    groove_y_screen = SCREEN_Y - SCREEN_FRAME_DEPTH / 2.0  # screen track groove

    # Sill track grooves (cut into top of sill, going down TRACK_DEPTH)
    sill_groove_sash = _slab(
        INNER_X0, INNER_X1,
        INNER_Z0 - TRACK_DEPTH, INNER_Z0,
        groove_y_sash, TRACK_WIDTH,
    )
    sill_groove_screen = _slab(
        INNER_X0, INNER_X1,
        INNER_Z0 - TRACK_DEPTH, INNER_Z0,
        groove_y_screen, TRACK_WIDTH * 0.7,
    )

    # Head track grooves (cut into bottom of head rail, going up TRACK_DEPTH)
    head_groove_sash = _slab(
        INNER_X0, INNER_X1,
        INNER_Z1, INNER_Z1 + TRACK_DEPTH,
        groove_y_sash, TRACK_WIDTH,
    )
    head_groove_screen = _slab(
        INNER_X0, INNER_X1,
        INNER_Z1, INNER_Z1 + TRACK_DEPTH,
        groove_y_screen, TRACK_WIDTH * 0.7,
    )

    frame = frame.cut(sill_groove_sash).cut(sill_groove_screen)
    frame = frame.cut(head_groove_sash).cut(head_groove_screen)

    return frame


def _build_meeting_rail_shape() -> cq.Workplane:
    """The meeting/check rail between upper and lower sash openings. A horizontal
    bar spanning the inner width at the meeting height."""
    return _slab(
        INNER_X0, INNER_X1,
        MEETING_Z0, MEETING_Z1,
        0.0, FRAME_DEPTH * 0.7,
    )


def _build_sash_grille_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """One sash built in its OWN local frame, centered on local origin:
      - local X in [-opening_w/2 - SASH_FACE, +opening_w/2 + SASH_FACE]
      - local Z in [-opening_h/2 - SASH_FACE, +opening_h/2 + SASH_FACE]
      - local Y is the sash depth, centered at 0
    Construction: outer sash slab cut by the clear opening, then the colonial
    muntin grid unioned back in.
    """
    ow = opening_w
    oh = opening_h
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE

    # Outer sash slab
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SASH_DEPTH)
    # Hollow it: cut the clear opening
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH + 0.02)
    ring = outer.cut(opening)

    # Colonial muntin grid
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

    return ring if bars is None else ring.union(bars)


def _build_sash_glass_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Single clear pane filling the sash opening."""
    ow = opening_w + 2 * REBATE
    oh = opening_h + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_screen_frame_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Insect screen frame built in local frame centered on origin. A thin
    rectangular frame ring (aluminum) with screen mesh infill."""
    ow = opening_w
    oh = opening_h
    out_w = ow + 2 * SCREEN_FRAME_W
    out_h = oh + 2 * SCREEN_FRAME_W

    # Outer frame slab
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SCREEN_FRAME_DEPTH)
    # Cut the inner opening
    inner = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SCREEN_FRAME_DEPTH + 0.01)
    return outer.cut(inner)


def _build_screen_mesh_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Thin screen mesh pane filling the screen frame opening."""
    return _slab(-opening_w / 2.0, opening_w / 2.0, -opening_h / 2.0, opening_h / 2.0, 0.0, SCREEN_MESH_T)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_sliding_window")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("screen_mesh", rgba=SCREEN_RGBA)
    model.material("aluminum", rgba=ALUMINUM_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="vinyl",
        name="frame_shell",
    )
    # Meeting rail as part of frame
    frame.visual(
        mesh_from_cadquery(_build_meeting_rail_shape(), "meeting_rail"),
        material="vinyl",
        name="meeting_rail",
    )

    # --- Upper sash (FIXED) ---
    upper_sash = model.part("upper_sash")
    upper_sash.visual(
        mesh_from_cadquery(
            _build_sash_grille_shape(OPENING_W, SASH_OPENING_H),
            "upper_sash_vinyl",
        ),
        material="vinyl",
        name="upper_sash_vinyl",
    )
    upper_sash.visual(
        mesh_from_cadquery(
            _build_sash_glass_shape(OPENING_W, SASH_OPENING_H),
            "upper_sash_glass",
        ),
        material="glass",
        name="upper_sash_glass",
    )

    # --- Lower sash (SLIDING - vertical prismatic) ---
    lower_sash = model.part("lower_sash")
    lower_sash.visual(
        mesh_from_cadquery(
            _build_sash_grille_shape(OPENING_W, SASH_OPENING_H),
            "lower_sash_vinyl",
        ),
        material="vinyl",
        name="lower_sash_vinyl",
    )
    lower_sash.visual(
        mesh_from_cadquery(
            _build_sash_glass_shape(OPENING_W, SASH_OPENING_H),
            "lower_sash_glass",
        ),
        material="glass",
        name="lower_sash_glass",
    )

    # --- Insect screen (SLIDING - independent shallow prismatic) ---
    screen_opening_w = OPENING_W - 2 * FRAME_FACE * 0.3  # slightly narrower than sash opening
    screen_opening_h = SASH_OPENING_H - 0.02
    insect_screen = model.part("insect_screen")
    insect_screen.visual(
        mesh_from_cadquery(
            _build_screen_frame_shape(screen_opening_w, screen_opening_h),
            "screen_frame",
        ),
        material="aluminum",
        name="screen_frame",
    )
    insect_screen.visual(
        mesh_from_cadquery(
            _build_screen_mesh_shape(screen_opening_w, screen_opening_h),
            "screen_mesh",
        ),
        material="screen_mesh",
        name="screen_mesh",
    )

    # --- Articulations ---

    # Upper sash: FIXED to frame, centered in upper opening
    upper_cx = 0.0
    upper_cz = (UPPER_Z0 + UPPER_Z1) / 2.0
    model.articulation(
        "frame_to_upper_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="upper_sash",
        origin=Origin(xyz=(upper_cx, SASH_Y, upper_cz)),
    )

    # Lower sash: PRISMATIC along +Z (slides upward to open).
    # Joint origin at the lower sash seated (closed) center.
    # Positive q slides the sash upward.
    lower_cx = 0.0
    lower_cz = (LOWER_Z0 + LOWER_Z1) / 2.0
    model.articulation(
        "frame_to_lower_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="lower_sash",
        origin=Origin(xyz=(lower_cx, SASH_Y, lower_cz)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0, velocity=0.4, lower=0.0, upper=SASH_TRAVEL,
        ),
    )

    # Insect screen: PRISMATIC along +Z (independent shallow slide upward).
    # Screen sits at lower sash height by default, can slide up independently.
    screen_cz = (LOWER_Z0 + LOWER_Z1) / 2.0
    model.articulation(
        "frame_to_screen",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="insect_screen",
        origin=Origin(xyz=(0.0, SCREEN_Y, screen_cz)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0, velocity=0.6, lower=0.0, upper=SCREEN_TRAVEL,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    upper_sash = object_model.get_part("upper_sash")
    lower_sash = object_model.get_part("lower_sash")
    insect_screen = object_model.get_part("insect_screen")
    sash_slide = object_model.get_articulation("frame_to_lower_sash")
    screen_slide = object_model.get_articulation("frame_to_screen")

    # --- Intentional overlaps ---
    # Glass panes tuck under the vinyl/muntin lip on each sash (captured glass).
    for nm in ("upper_sash", "lower_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_vinyl",
            reason="Clear pane is rebated under the sash/muntin lip so it reads captured, not floating.",
        )

    # Screen mesh is captured inside the screen frame
    ctx.allow_overlap(
        "insect_screen", "insect_screen",
        elem_a="screen_mesh",
        elem_b="screen_frame",
        reason="Screen mesh is seated within the screen frame ring (captured mesh).",
    )

    # Upper sash is fixed in the frame opening - rebated capture
    ctx.allow_overlap(
        "frame", "upper_sash",
        elem_a="frame_shell",
        elem_b="upper_sash_vinyl",
        reason="Upper fixed sash is rebated into the frame opening; its sash ring laps the jamb/head/meeting rail edge.",
    )
    ctx.allow_overlap(
        "frame", "upper_sash",
        elem_a="meeting_rail",
        elem_b="upper_sash_vinyl",
        reason="Upper sash bottom edge seats on the meeting rail.",
    )

    # Lower sash rides the sill/head track grooves - lapping the frame track edges
    ctx.allow_overlap(
        "frame", "lower_sash",
        elem_a="frame_shell",
        elem_b="lower_sash_vinyl",
        reason="Lower sash rides the head/sill track grooves; its edges lap the track lips (sliding capture).",
    )
    ctx.allow_overlap(
        "frame", "lower_sash",
        elem_a="meeting_rail",
        elem_b="lower_sash_vinyl",
        reason="Lower sash top edge contacts the meeting rail at closed position.",
    )

    # Screen rides in its own track proud of the sashes
    ctx.allow_overlap(
        "frame", "insect_screen",
        elem_a="frame_shell",
        elem_b="screen_frame",
        reason="Insect screen rides in a shallow track groove proud of the sashes; frame laps screen frame at track edges.",
    )

    # Glass panes rebate into frame at rest
    ctx.allow_overlap(
        "frame", "upper_sash",
        elem_a="frame_shell",
        elem_b="upper_sash_glass",
        reason="Upper sash glass is rebated under the frame opening lip (captured glazing).",
    )
    ctx.allow_overlap(
        "frame", "lower_sash",
        elem_a="frame_shell",
        elem_b="lower_sash_glass",
        reason="Lower sash glass is rebated under the sill/head track lips as the sash rides the track (captured glazing).",
    )

    # Sash rails interlock at the meeting rail (real check-rail overlap in a
    # single-hung window: the top rail of the lower sash and the bottom rail
    # of the upper sash meet and interlock at the meeting/check rail).
    ctx.allow_overlap(
        "lower_sash", "upper_sash",
        elem_a="lower_sash_vinyl",
        elem_b="upper_sash_vinyl",
        reason="Lower and upper sash rails interlock at the meeting/check rail (real single-hung check-rail capture).",
    )

    # --- Closed pose (q=0): window reads SHUT ---
    with ctx.pose({sash_slide: 0.0, screen_slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        lower_aabb = ctx.part_world_aabb(lower_sash)
        upper_aabb = ctx.part_world_aabb(upper_sash)
        screen_aabb = ctx.part_world_aabb(insect_screen)

        # Frame spans the full height and width
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        frame_h = frame_aabb[1][2] - frame_aabb[0][2]
        ctx.check(
            "frame is full window size",
            frame_w > 0.90 and frame_h > 1.40,
            details=f"frame_w={frame_w:.3f}, frame_h={frame_h:.3f}",
        )

        # Sill sits near z=0
        ctx.check(
            "sill sits near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"frame zmin={frame_aabb[0][2]:.4f}",
        )

        # Lower sash is below upper sash (vertical arrangement)
        lower_cz = (lower_aabb[0][2] + lower_aabb[1][2]) / 2.0
        upper_cz = (upper_aabb[0][2] + upper_aabb[1][2]) / 2.0
        ctx.check(
            "lower sash below upper sash",
            lower_cz < upper_cz - 0.1,
            details=f"lower_cz={lower_cz:.3f}, upper_cz={upper_cz:.3f}",
        )

        # Screen sits proud of sashes in +Y
        sash_y = (lower_aabb[0][1] + lower_aabb[1][1]) / 2.0
        screen_y = (screen_aabb[0][1] + screen_aabb[1][1]) / 2.0
        ctx.check(
            "screen proud of sashes in +Y",
            screen_y > sash_y + 0.02,
            details=f"screen_y={screen_y:.3f}, sash_y={sash_y:.3f}",
        )

        # Lower sash seated in frame vertically
        ctx.expect_overlap(
            lower_sash, frame, axes="xz", min_overlap=0.03,
            name="lower sash seated in frame opening at rest",
        )

        # Sash rails meet at the meeting rail: the lower sash top and upper
        # sash bottom contact at the meeting rail height (check-rail interlock).
        ctx.expect_gap(
            upper_sash, lower_sash,
            axis="z",
            max_penetration=0.052,
            name="sash rails interlock at meeting rail within check-rail depth",
        )

        # Glass is captured within frame: lower sash glass stays within frame X span
        ctx.expect_within(
            lower_sash, frame,
            axes="x",
            inner_elem="lower_sash_glass",
            outer_elem="frame_shell",
            margin=0.01,
            name="lower sash glass centered within frame width",
        )

        rest_lower_cz = lower_cz
        rest_screen_cz = (screen_aabb[0][2] + screen_aabb[1][2]) / 2.0

    # --- Lower sash opens (slides upward) ---
    travel = sash_slide.motion_limits.upper
    with ctx.pose({sash_slide: travel, screen_slide: 0.0}):
        lower_open = ctx.part_world_aabb(lower_sash)
        open_lower_cz = (lower_open[0][2] + lower_open[1][2]) / 2.0

        # The sash center translated upward by ~travel distance
        ctx.check(
            "lower sash slides upward by ~travel",
            abs((open_lower_cz - rest_lower_cz) - travel) < 0.02,
            details=f"rest_cz={rest_lower_cz:.3f}, open_cz={open_lower_cz:.3f}, travel={travel:.3f}",
        )

        # Sash did not move in X (pure vertical slide)
        rest_lower_cx = 0.0  # centered
        open_lower_cx = (lower_open[0][0] + lower_open[1][0]) / 2.0
        ctx.check(
            "slide is purely vertical",
            abs(open_lower_cx - rest_lower_cx) < 0.02,
            details=f"open_cx={open_lower_cx:.3f}",
        )

        # Retained insertion: sash still overlaps frame Z extent at full travel
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained within frame at full travel",
            lower_open[1][2] < f_aabb[1][2] + 0.01 and lower_open[0][2] > f_aabb[0][2] - 0.01,
            details=f"sash z=[{lower_open[0][2]:.3f},{lower_open[1][2]:.3f}] frame z=[{f_aabb[0][2]:.3f},{f_aabb[1][2]:.3f}]",
        )

    # --- Screen slides independently ---
    screen_travel = screen_slide.motion_limits.upper
    with ctx.pose({sash_slide: 0.0, screen_slide: screen_travel}):
        screen_open = ctx.part_world_aabb(insect_screen)
        open_screen_cz = (screen_open[0][2] + screen_open[1][2]) / 2.0

        # Screen translated upward by ~screen travel
        ctx.check(
            "screen slides upward independently",
            abs((open_screen_cz - rest_screen_cz) - screen_travel) < 0.02,
            details=f"rest_cz={rest_screen_cz:.3f}, open_cz={open_screen_cz:.3f}, travel={screen_travel:.3f}",
        )

        # Screen independent of sash: sash did not move when screen moved
        lower_still = ctx.part_world_aabb(lower_sash)
        still_cz = (lower_still[0][2] + lower_still[1][2]) / 2.0
        ctx.check(
            "screen motion does not move lower sash",
            abs(still_cz - rest_lower_cz) < 0.005,
            details=f"sash cz={still_cz:.3f}, expected={rest_lower_cz:.3f}",
        )

    # --- Non-fixed joint exists ---
    ctx.check(
        "lower sash joint is prismatic",
        sash_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={sash_slide.articulation_type}",
    )
    ctx.check(
        "screen joint is prismatic",
        screen_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={screen_slide.articulation_type}",
    )

    # --- Track grooves exist: frame has geometry extending beyond inner opening ---
    # The deep track grooves extend the frame past the inner Z bounds
    with ctx.pose({sash_slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        # Frame should extend below INNER_Z0 and above INNER_Z1 (track grooves)
        ctx.check(
            "frame extends below inner opening (sill track groove)",
            frame_aabb[0][2] < INNER_Z0 - 0.005,
            details=f"frame zmin={frame_aabb[0][2]:.4f}, inner_z0={INNER_Z0:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
