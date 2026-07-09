from __future__ import annotations

# Vertical sash-style sliding window (double-hung), white vinyl frame with
# colonial divided-lite grilles. Two sashes slide vertically in opposite
# directions on separate prismatic joints: the upper sash slides DOWN, the
# lower sash slides UP. Deep track grooves are cut into the side jambs and
# the head/sill. Rubber gasket strips surround each glass pane.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     - width  -> X
#     - height -> Z   (sill near z=0)
#     - frame depth / glazing thickness / slide-normal -> Y
#   The glass plane is the X-Z plane. The window reads SHUT at q=0 for both
#   joints. Driving the upper sash joint slides it down; driving the lower
#   sash joint slides it up.

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

TOTAL_W = 1.20            # overall window width along X
TOTAL_H = 1.50            # overall window height along Z (sill at z=0)

FRAME_FACE = 0.070        # outer frame member face width (jamb / head / sill)
FRAME_DEPTH = 0.110       # outer frame depth along Y

# Track grooves — deep channels cut into jambs and head/sill
TRACK_WIDTH = 0.028       # track channel width in Y
TRACK_DEPTH = 0.022       # how far the channel cuts into the frame member

# Sash construction
SASH_FACE = 0.050         # sash perimeter rail/stile face width
SASH_DEPTH = 0.040        # sash depth along Y
GLASS_T = 0.006           # glazing thickness along Y
GASKET_W = 0.010          # rubber gasket strip visible width
GASKET_T = 0.003          # gasket thickness (proud of glass face)

# Colonial grille (divided lite)
GRILLE_COLS = 3           # 3 columns of panes
GRILLE_ROWS = 4           # 4 rows of panes
MUNTIN_T = 0.018          # muntin bar face width
MUNTIN_DEPTH = 0.018      # muntin bar depth along Y

REBATE = 0.005            # glass tucks under sash lip by this much

# Slide travel for each sash
SLIDE_TRAVEL = 0.35       # meters of vertical travel

# ---------------------------------------------------------------------------
# Derived layout
# ---------------------------------------------------------------------------

HALF_W = TOTAL_W / 2.0

INNER_X0 = -HALF_W + FRAME_FACE
INNER_X1 = HALF_W - FRAME_FACE
INNER_Z0 = FRAME_FACE
INNER_Z1 = TOTAL_H - FRAME_FACE

OPENING_W = INNER_X1 - INNER_X0   # clear width inside frame
OPENING_H = INNER_Z1 - INNER_Z0   # clear height inside frame

# Two sashes cover the opening with a small meeting-rail overlap.
MEETING_OVERLAP = 0.025
SASH_TOTAL_H = (OPENING_H + MEETING_OVERLAP) / 2.0
SASH_GLASS_H = SASH_TOTAL_H - 2.0 * SASH_FACE
SASH_W = OPENING_W                # sash spans full opening width
GLASS_W = SASH_W - 2.0 * SASH_FACE  # glass clear width inside sash frame

# Track Y positions — two parallel tracks offset in Y so sashes pass each other
REAR_TRACK_Y = -0.025       # upper sash rides in the rear track
FRONT_TRACK_Y = 0.025       # lower sash rides in the front track

# Sash center-Z at rest (q=0)
UPPER_SASH_CZ = INNER_Z1 - SASH_TOTAL_H / 2.0
LOWER_SASH_CZ = INNER_Z0 + SASH_TOTAL_H / 2.0

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)       # bright white vinyl
GLASS_RGBA = (0.50, 0.58, 0.64, 0.32)       # cool grey-blue, semi-transparent
GASKET_RGBA = (0.15, 0.15, 0.15, 1.0)       # dark rubber black


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery). All in meters, world frame.
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float,
          y_center: float, depth: float) -> cq.Workplane:
    """Axis-aligned box spanning [x0,x1] x [z0,z1] in the X-Z plane,
    centered on y_center with the given Y depth."""
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
    """Outer frame with deep track grooves.
    A full slab cut by one large opening, then track channels are cut into
    the jamb inner faces and head/sill inner faces for both front and rear
    sash tracks."""
    # Main slab
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)

    # Cut the main opening (one large rectangular opening)
    cut_d = FRAME_DEPTH + 0.02
    opening_cut = _slab(INNER_X0, INNER_X1, INNER_Z0, INNER_Z1, 0.0, cut_d)
    frame = outer.cut(opening_cut)

    # Track grooves on LEFT jamb (cut into jamb body from the inner face).
    # Each groove is a narrow vertical slot.
    for ty in (REAR_TRACK_Y, FRONT_TRACK_Y):
        groove = _slab(
            INNER_X0 - TRACK_DEPTH, INNER_X0,
            INNER_Z0, INNER_Z1,
            ty, TRACK_WIDTH,
        )
        frame = frame.cut(groove)

    # Track grooves on RIGHT jamb (mirror)
    for ty in (REAR_TRACK_Y, FRONT_TRACK_Y):
        groove = _slab(
            INNER_X1, INNER_X1 + TRACK_DEPTH,
            INNER_Z0, INNER_Z1,
            ty, TRACK_WIDTH,
        )
        frame = frame.cut(groove)

    # Track grooves on HEAD (horizontal channels cut into the head from below)
    for ty in (REAR_TRACK_Y, FRONT_TRACK_Y):
        groove = _slab(
            INNER_X0, INNER_X1,
            INNER_Z1, INNER_Z1 + TRACK_DEPTH,
            ty, TRACK_WIDTH,
        )
        frame = frame.cut(groove)

    # Track grooves on SILL (horizontal channels cut into the sill from above)
    for ty in (REAR_TRACK_Y, FRONT_TRACK_Y):
        groove = _slab(
            INNER_X0, INNER_X1,
            INNER_Z0 - TRACK_DEPTH, INNER_Z0,
            ty, TRACK_WIDTH,
        )
        frame = frame.cut(groove)

    return frame


def _build_sash_vinyl_shape(glass_w: float, glass_h: float) -> cq.Workplane:
    """One sash frame (stiles + rails) with colonial muntin grille, built in
    its own local frame centered on the local origin.

    Returns the vinyl workplane (frame ring + muntins). Glass and gasket are
    built separately.
    """
    out_w = glass_w + 2.0 * SASH_FACE
    out_h = glass_h + 2.0 * SASH_FACE

    # Outer sash slab
    outer = _slab(-out_w / 2.0, out_w / 2.0,
                  -out_h / 2.0, out_h / 2.0,
                  0.0, SASH_DEPTH)
    # Hollow: cut the clear glass opening
    opening = _slab(-glass_w / 2.0, glass_w / 2.0,
                    -glass_h / 2.0, glass_h / 2.0,
                    0.0, SASH_DEPTH + 0.02)
    ring = outer.cut(opening)

    # Colonial muntin grid across the opening
    bars = None

    # Vertical muntins
    for c in range(1, GRILLE_COLS):
        frac = c / GRILLE_COLS
        x = -glass_w / 2.0 + frac * glass_w
        bar = _slab(
            x - MUNTIN_T / 2.0, x + MUNTIN_T / 2.0,
            -glass_h / 2.0, glass_h / 2.0,
            0.0, MUNTIN_DEPTH,
        )
        bars = bar if bars is None else bars.union(bar)

    # Horizontal muntins
    for r in range(1, GRILLE_ROWS):
        frac = r / GRILLE_ROWS
        z = -glass_h / 2.0 + frac * glass_h
        bar = _slab(
            -glass_w / 2.0, glass_w / 2.0,
            z - MUNTIN_T / 2.0, z + MUNTIN_T / 2.0,
            0.0, MUNTIN_DEPTH,
        )
        bars = bar if bars is None else bars.union(bar)

    return ring if bars is None else ring.union(bars)


def _build_sash_glass_shape(glass_w: float, glass_h: float) -> cq.Workplane:
    """Single clear pane filling the sash glass opening, centered at y=0."""
    ow = glass_w + 2.0 * REBATE
    oh = glass_h + 2.0 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_sash_gasket_shape(glass_w: float, glass_h: float) -> cq.Workplane:
    """Rubber gasket ring around the glass perimeter. Sits on the front face
    of the glass as a visible dark border strip."""
    # Outer edge matches the glass pane (with rebate)
    ow = glass_w + 2.0 * REBATE
    oh = glass_h + 2.0 * REBATE
    # Inner edge is inset by the gasket visible width
    iw = ow - 2.0 * GASKET_W
    ih = oh - 2.0 * GASKET_W

    # Gasket sits on the front face of the glass
    gasket_y = GLASS_T / 2.0 + GASKET_T / 2.0

    outer = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0,
                  gasket_y, GASKET_T)
    inner_cut = _slab(-iw / 2.0, iw / 2.0, -ih / 2.0, ih / 2.0,
                      gasket_y, GASKET_T + 0.002)
    return outer.cut(inner_cut)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_sliding_window")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("gasket", rgba=GASKET_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="vinyl",
        name="frame_shell",
    )

    # --- Upper sash (slides DOWN) ---
    upper = model.part("upper_sash")
    upper.visual(
        mesh_from_cadquery(
            _build_sash_vinyl_shape(GLASS_W, SASH_GLASS_H),
            "upper_sash_vinyl",
        ),
        material="vinyl",
        name="upper_sash_vinyl",
    )
    upper.visual(
        mesh_from_cadquery(
            _build_sash_glass_shape(GLASS_W, SASH_GLASS_H),
            "upper_sash_glass",
        ),
        material="glass",
        name="upper_sash_glass",
    )
    upper.visual(
        mesh_from_cadquery(
            _build_sash_gasket_shape(GLASS_W, SASH_GLASS_H),
            "upper_sash_gasket",
        ),
        material="gasket",
        name="upper_sash_gasket",
    )

    # --- Lower sash (slides UP) ---
    lower = model.part("lower_sash")
    lower.visual(
        mesh_from_cadquery(
            _build_sash_vinyl_shape(GLASS_W, SASH_GLASS_H),
            "lower_sash_vinyl",
        ),
        material="vinyl",
        name="lower_sash_vinyl",
    )
    lower.visual(
        mesh_from_cadquery(
            _build_sash_glass_shape(GLASS_W, SASH_GLASS_H),
            "lower_sash_glass",
        ),
        material="glass",
        name="lower_sash_glass",
    )
    lower.visual(
        mesh_from_cadquery(
            _build_sash_gasket_shape(GLASS_W, SASH_GLASS_H),
            "lower_sash_gasket",
        ),
        material="gasket",
        name="lower_sash_gasket",
    )

    # --- Articulations ---

    # Upper sash: PRISMATIC along -Z (positive q slides the sash DOWN).
    # The sash is in the rear track at REAR_TRACK_Y.
    model.articulation(
        "frame_to_upper_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="upper_sash",
        origin=Origin(xyz=(0.0, REAR_TRACK_Y, UPPER_SASH_CZ)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.5,
            lower=0.0, upper=SLIDE_TRAVEL,
        ),
    )

    # Lower sash: PRISMATIC along +Z (positive q slides the sash UP).
    # The sash is in the front track at FRONT_TRACK_Y.
    model.articulation(
        "frame_to_lower_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="lower_sash",
        origin=Origin(xyz=(0.0, FRONT_TRACK_Y, LOWER_SASH_CZ)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.5,
            lower=0.0, upper=SLIDE_TRAVEL,
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
    upper_slide = object_model.get_articulation("frame_to_upper_sash")
    lower_slide = object_model.get_articulation("frame_to_lower_sash")

    # --- Intentional overlap allowances ---

    # Glass panes are rebated under the sash frame lip on each sash.
    for nm in ("upper_sash", "lower_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_vinyl",
            reason="Glass pane is rebated under the sash rail/stile lip so it reads captured, not floating.",
        )

    # Gasket strips sit on the glass front face and overlap the sash frame edge.
    for nm in ("upper_sash", "lower_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_gasket",
            elem_b=f"{nm}_glass",
            reason="Rubber gasket strip sits on the glass front face as a perimeter seal bead.",
        )
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_gasket",
            elem_b=f"{nm}_vinyl",
            reason="Rubber gasket strip overlaps the sash frame inner edge as a glazing seal.",
        )

    # Sashes ride in the frame track grooves; the sash stiles lap the jamb
    # track walls (captured in the tracks).
    ctx.allow_overlap(
        "frame", "upper_sash",
        elem_a="frame_shell",
        elem_b="upper_sash_vinyl",
        reason="Upper sash stiles ride in the rear track grooves; sash ring laps the jamb track walls.",
    )
    ctx.allow_overlap(
        "frame", "lower_sash",
        elem_a="frame_shell",
        elem_b="lower_sash_vinyl",
        reason="Lower sash stiles ride in the front track grooves; sash ring laps the jamb track walls.",
    )

    # Sash glass is rebated past the sash opening and may lap the frame head/sill
    # groove lip at rest (captured glazing at the track extremes).
    ctx.allow_overlap(
        "frame", "upper_sash",
        elem_a="frame_shell",
        elem_b="upper_sash_glass",
        reason="Upper sash glass laps the head track groove lip when seated at rest.",
    )
    ctx.allow_overlap(
        "frame", "lower_sash",
        elem_a="frame_shell",
        elem_b="lower_sash_glass",
        reason="Lower sash glass laps the sill track groove lip when seated at rest.",
    )

    # --- Closed pose (q=0): window reads SHUT ---
    with ctx.pose({upper_slide: 0.0, lower_slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        u_aabb = ctx.part_world_aabb(upper_sash)
        l_aabb = ctx.part_world_aabb(lower_sash)

        # Frame spans the full window dimensions.
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        frame_h = frame_aabb[1][2] - frame_aabb[0][2]
        ctx.check(
            "frame has correct width",
            abs(frame_w - TOTAL_W) < 0.02,
            details=f"frame_w={frame_w:.3f}, expected={TOTAL_W:.3f}",
        )
        ctx.check(
            "frame has correct height",
            abs(frame_h - TOTAL_H) < 0.02,
            details=f"frame_h={frame_h:.3f}, expected={TOTAL_H:.3f}",
        )

        # Sill near z=0.
        ctx.check(
            "sill sits near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"frame zmin={frame_aabb[0][2]:.4f}",
        )

        # Upper sash is above the lower sash at rest.
        u_cz = (u_aabb[0][2] + u_aabb[1][2]) / 2.0
        l_cz = (l_aabb[0][2] + l_aabb[1][2]) / 2.0
        ctx.check(
            "upper sash above lower sash at rest",
            u_cz > l_cz + 0.10,
            details=f"upper_cz={u_cz:.3f}, lower_cz={l_cz:.3f}",
        )

        # Both sashes seated within the frame height.
        for nm, ab in (("upper", u_aabb), ("lower", l_aabb)):
            ctx.check(
                f"{nm} sash seated within frame height",
                ab[0][2] > frame_aabb[0][2] - 1e-4 and ab[1][2] < frame_aabb[1][2] + 1e-4,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}] frame z=[{frame_aabb[0][2]:.3f},{frame_aabb[1][2]:.3f}]",
            )

        # Sashes are on different Y tracks (rear vs front).
        u_y = (u_aabb[0][1] + u_aabb[1][1]) / 2.0
        l_y = (l_aabb[0][1] + l_aabb[1][1]) / 2.0
        ctx.check(
            "sashes on separate Y tracks",
            abs(u_y - l_y) > 0.03,
            details=f"upper_y={u_y:.3f}, lower_y={l_y:.3f}",
        )

        # Both sashes overlap the frame footprint in X (seated in the opening).
        ctx.expect_overlap(
            upper_sash, frame, axes="x", min_overlap=0.50,
            name="upper sash seated in frame opening (X)",
        )
        ctx.expect_overlap(
            lower_sash, frame, axes="x", min_overlap=0.50,
            name="lower sash seated in frame opening (X)",
        )

        # Gasket visuals exist (proof check for gasket geometry).
        for nm in ("upper_sash", "lower_sash"):
            gasket_vis = object_model.get_part(nm).get_visual(f"{nm}_gasket")
            ctx.check(
                f"{nm} has rubber gasket strip",
                gasket_vis is not None,
                details=f"missing gasket visual on {nm}",
            )

        # Record rest positions for driven-pose checks.
        rest_upper_cz = u_cz
        rest_lower_cz = l_cz

    # --- Driven pose: upper sash slides DOWN ---
    with ctx.pose({upper_slide: SLIDE_TRAVEL}):
        u_open = ctx.part_world_aabb(upper_sash)
        open_upper_cz = (u_open[0][2] + u_open[1][2]) / 2.0
        ctx.check(
            "upper sash slides down by ~travel",
            abs((rest_upper_cz - open_upper_cz) - SLIDE_TRAVEL) < 0.02,
            details=f"rest_cz={rest_upper_cz:.3f}, open_cz={open_upper_cz:.3f}, travel={SLIDE_TRAVEL:.3f}",
        )
        # Sash remains within the frame at full travel.
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "upper sash retained in frame at full travel",
            u_open[0][2] > f_aabb[0][2] - 0.01 and u_open[1][2] < f_aabb[1][2] + 0.01,
            details=f"sash z=[{u_open[0][2]:.3f},{u_open[1][2]:.3f}] frame z=[{f_aabb[0][2]:.3f},{f_aabb[1][2]:.3f}]",
        )

    # --- Driven pose: lower sash slides UP ---
    with ctx.pose({lower_slide: SLIDE_TRAVEL}):
        l_open = ctx.part_world_aabb(lower_sash)
        open_lower_cz = (l_open[0][2] + l_open[1][2]) / 2.0
        ctx.check(
            "lower sash slides up by ~travel",
            abs((open_lower_cz - rest_lower_cz) - SLIDE_TRAVEL) < 0.02,
            details=f"rest_cz={rest_lower_cz:.3f}, open_cz={open_lower_cz:.3f}, travel={SLIDE_TRAVEL:.3f}",
        )
        # Sash remains within the frame at full travel.
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "lower sash retained in frame at full travel",
            l_open[0][2] > f_aabb[0][2] - 0.01 and l_open[1][2] < f_aabb[1][2] + 0.01,
            details=f"sash z=[{l_open[0][2]:.3f},{l_open[1][2]:.3f}] frame z=[{f_aabb[0][2]:.3f},{f_aabb[1][2]:.3f}]",
        )

    # --- Both sashes driven simultaneously (opposite directions) ---
    with ctx.pose({upper_slide: SLIDE_TRAVEL, lower_slide: SLIDE_TRAVEL}):
        u_both = ctx.part_world_aabb(upper_sash)
        l_both = ctx.part_world_aabb(lower_sash)
        u_both_cz = (u_both[0][2] + u_both[1][2]) / 2.0
        l_both_cz = (l_both[0][2] + l_both[1][2]) / 2.0
        # The sashes have moved toward each other (meeting in the middle region).
        ctx.check(
            "sashes converge when both driven",
            u_both_cz < rest_upper_cz and l_both_cz > rest_lower_cz,
            details=f"upper={u_both_cz:.3f}<{rest_upper_cz:.3f}, lower={l_both_cz:.3f}>{rest_lower_cz:.3f}",
        )

    # --- Joint type verification ---
    ctx.check(
        "upper sash joint is prismatic",
        upper_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"got {upper_slide.articulation_type}",
    )
    ctx.check(
        "lower sash joint is prismatic",
        lower_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"got {lower_slide.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
