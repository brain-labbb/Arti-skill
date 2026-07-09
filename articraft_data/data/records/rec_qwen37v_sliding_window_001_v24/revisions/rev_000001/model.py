from __future__ import annotations

# Variant 24: Three-panel sliding window with narrow transom above.
# Two sashes slide in opposite directions on separate prismatic joints,
# each with two tiny roller blocks at the bottom.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     - width  -> X
#     - height -> Z   (sill near z=0)
#     - frame depth / glazing thickness / slide-normal -> Y
#   The glass plane is the X-Z plane. The window reads SHUT at q=0;
#   driving the prismatic joints slides both sashes toward center (overlapping
#   on separate Y tracks) to expose the left and right sides of the opening.
#
# Structure:
#   - frame (static root): head, sill, two jambs, horizontal transom bar.
#     The transom opening is at top; the main slider opening is below.
#   - transom (FIXED): narrow fixed lite with colonial grille, seated in the
#     transom opening.
#   - left_sash (SLIDING): sash with colonial grille, on the front track (+Y),
#     slides RIGHT (+X) to overlap the right sash.
#   - right_sash (SLIDING): sash with colonial grille, on the rear track (-Y),
#     slides LEFT (-X) to overlap the left sash.
#   - Each sash has two tiny roller blocks at its bottom, riding the sill track.

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Absolute dimensions (meters)
# ---------------------------------------------------------------------------

TOTAL_W = 3.00            # overall window width along X
TOTAL_H = 1.80            # overall height along Z (taller for transom)

FRAME_FACE = 0.070        # outer frame member face width (jamb / head / sill)
FRAME_DEPTH = 0.110       # outer frame depth along Y

TRANSOM_H = 0.28          # transom opening height
HORIZ_BAR = 0.120         # horizontal bar between transom and slider opening

# Sash construction
SASH_FACE = 0.050         # sash perimeter rail/stile face width (in-plane)
SASH_DEPTH = 0.050        # sash depth along Y
GLASS_T = 0.008           # glazing thickness along Y

# Colonial grille (divided lite): each sash is a grid of small panes.
GRILLE_COLS = 3           # 3 columns of panes per sash
GRILLE_ROWS = 5           # 5 rows of panes per sash
MUNTIN_T = 0.018          # muntin bar face width (in-plane)
MUNTIN_DEPTH = 0.018      # muntin bar depth along Y

# Transom grille (simpler: 4 cols x 2 rows)
TRANSOM_GRILLE_COLS = 6
TRANSOM_GRILLE_ROWS = 2
TRANSOM_MUNTIN_T = 0.015

# Y layout (depth). Two sashes on separate tracks:
#   front track (left sash): +Y offset
#   rear track (right sash): -Y offset
# This lets them pass each other when sliding.
LEFT_SASH_Y = 0.030       # left sash front track center
RIGHT_SASH_Y = -0.030     # right sash rear track center
TRANSOM_Y = 0.0           # transom centered in frame depth

# Roller blocks
ROLLER_W = 0.030          # roller width along X
ROLLER_H = 0.018          # roller height along Z
ROLLER_D = 0.025          # roller depth along Y

REBATE = 0.005            # glass tucks under the sash/muntin lip

# ---------------------------------------------------------------------------
# Derived layout
# ---------------------------------------------------------------------------

HALF_W = TOTAL_W / 2.0

# Inner clear region (inside the outer head/sill/jambs)
INNER_X0 = -HALF_W + FRAME_FACE
INNER_X1 = HALF_W - FRAME_FACE
INNER_W = INNER_X1 - INNER_X0  # clear inner width

# Transom opening
TRANSOM_Z0 = TOTAL_H - FRAME_FACE - TRANSOM_H
TRANSOM_Z1 = TOTAL_H - FRAME_FACE

# Horizontal bar between transom and slider
BAR_Z0 = TRANSOM_Z0 - HORIZ_BAR
BAR_Z1 = TRANSOM_Z0

# Main slider opening
SLIDER_Z0 = FRAME_FACE
SLIDER_Z1 = BAR_Z0
SLIDER_H = SLIDER_Z1 - SLIDER_Z0  # height of slider opening

# Each sash covers roughly half the inner width
SASH_OPENING_W = INNER_W / 2.0

# Sash center positions at rest (q=0):
LEFT_SASH_CX = INNER_X0 + SASH_OPENING_W / 2.0
RIGHT_SASH_CX = INNER_X1 - SASH_OPENING_W / 2.0

# Transom center
TRANSOM_CX = 0.0
TRANSOM_CZ = (TRANSOM_Z0 + TRANSOM_Z1) / 2.0

# Slider center Z
SLIDER_CZ = (SLIDER_Z0 + SLIDER_Z1) / 2.0

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)     # bright white vinyl/PVC
GLASS_RGBA = (0.50, 0.58, 0.64, 0.32)    # cool grey-blue, semi-transparent
ROLLER_RGBA = (0.25, 0.25, 0.28, 1.0)    # dark grey nylon roller


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery). All authored directly in meters, world frame.
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
    """Static outer frame: a full slab cut by the transom opening and the main
    slider opening, leaving head, sill, two jambs, and the horizontal transom bar."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)

    cut_depth = FRAME_DEPTH + 0.02
    # Transom opening (top)
    transom_cut = _slab(INNER_X0, INNER_X1, TRANSOM_Z0, TRANSOM_Z1, 0.0, cut_depth)
    # Main slider opening (bottom) - one wide opening, no mullion
    slider_cut = _slab(INNER_X0, INNER_X1, SLIDER_Z0, SLIDER_Z1, 0.0, cut_depth)

    return outer.cut(transom_cut).cut(slider_cut)


def _build_sash_grille_shape(opening_w: float, opening_h: float,
                             grille_cols: int, grille_rows: int,
                             muntin_t: float) -> cq.Workplane:
    """One sash built in its OWN local frame, centered on local origin.
    Outer sash slab cut by the clear opening, then colonial muntin grid unioned in."""
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
    for c in range(1, grille_cols):
        frac = c / grille_cols
        x = -ow / 2.0 + frac * ow
        bar = _slab(
            x - muntin_t / 2.0, x + muntin_t / 2.0,
            -oh / 2.0, oh / 2.0,
            0.0, MUNTIN_DEPTH,
        )
        bars = bar if bars is None else bars.union(bar)

    # Horizontal muntins
    for r in range(1, grille_rows):
        frac = r / grille_rows
        z = -oh / 2.0 + frac * oh
        bar = _slab(
            -ow / 2.0, ow / 2.0,
            z - muntin_t / 2.0, z + muntin_t / 2.0,
            0.0, MUNTIN_DEPTH,
        )
        bars = bar if bars is None else bars.union(bar)

    return ring if bars is None else ring.union(bars)


def _build_sash_glass_shape(opening_w: float, opening_h: float) -> cq.Workplane:
    """Single clear pane filling the sash opening, in the sash-local frame."""
    ow = opening_w + 2 * REBATE
    oh = opening_h + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_roller_shape() -> cq.Workplane:
    """Tiny roller block: small box in its own local frame centered at origin."""
    return (
        cq.Workplane("XY")
        .box(ROLLER_W, ROLLER_D, ROLLER_H)
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="transom_dual_slider_window")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("roller", rgba=ROLLER_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="vinyl",
        name="frame_shell",
    )

    # --- Transom (fixed lite at top) ---
    transom = model.part("transom")
    transom_opening_w = INNER_W
    transom_opening_h = TRANSOM_H
    transom.visual(
        mesh_from_cadquery(
            _build_sash_grille_shape(transom_opening_w, transom_opening_h,
                                     TRANSOM_GRILLE_COLS, TRANSOM_GRILLE_ROWS,
                                     TRANSOM_MUNTIN_T),
            "transom_vinyl",
        ),
        material="vinyl",
        name="transom_vinyl",
    )
    transom.visual(
        mesh_from_cadquery(_build_sash_glass_shape(transom_opening_w, transom_opening_h), "transom_glass"),
        material="glass",
        name="transom_glass",
    )

    # --- Left sash (front track, slides RIGHT +X) ---
    left_sash = model.part("left_sash")
    left_sash.visual(
        mesh_from_cadquery(
            _build_sash_grille_shape(SASH_OPENING_W, SLIDER_H,
                                     GRILLE_COLS, GRILLE_ROWS, MUNTIN_T),
            "left_sash_vinyl",
        ),
        material="vinyl",
        name="left_sash_vinyl",
    )
    left_sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(SASH_OPENING_W, SLIDER_H), "left_sash_glass"),
        material="glass",
        name="left_sash_glass",
    )

    # --- Right sash (rear track, slides LEFT -X) ---
    right_sash = model.part("right_sash")
    right_sash.visual(
        mesh_from_cadquery(
            _build_sash_grille_shape(SASH_OPENING_W, SLIDER_H,
                                     GRILLE_COLS, GRILLE_ROWS, MUNTIN_T),
            "right_sash_vinyl",
        ),
        material="vinyl",
        name="right_sash_vinyl",
    )
    right_sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(SASH_OPENING_W, SLIDER_H), "right_sash_glass"),
        material="glass",
        name="right_sash_glass",
    )

    # --- Roller blocks (2 per sash, FIXED to their sash) ---
    roller_mesh = _build_roller_shape()
    for sash_name in ("left_sash", "right_sash"):
        for i in range(2):
            roller = model.part(f"{sash_name}_roller_{i}")
            roller.visual(
                mesh_from_cadquery(roller_mesh, f"{sash_name}_roller_{i}_body"),
                material="roller",
                name=f"{sash_name}_roller_{i}_body",
            )

    # -----------------------------------------------------------------------
    # Articulations
    # -----------------------------------------------------------------------

    # Transom: FIXED, centered in the transom opening
    model.articulation(
        "frame_to_transom",
        ArticulationType.FIXED,
        parent="frame",
        child="transom",
        origin=Origin(xyz=(TRANSOM_CX, TRANSOM_Y, TRANSOM_CZ)),
    )

    # Left sash: PRISMATIC along +X (slides right to overlap right sash)
    slide_travel = SASH_OPENING_W * 0.85
    model.articulation(
        "frame_to_left_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="left_sash",
        origin=Origin(xyz=(LEFT_SASH_CX, LEFT_SASH_Y, SLIDER_CZ)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5, lower=0.0, upper=slide_travel),
    )

    # Right sash: PRISMATIC along -X (slides left to overlap left sash)
    model.articulation(
        "frame_to_right_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="right_sash",
        origin=Origin(xyz=(RIGHT_SASH_CX, RIGHT_SASH_Y, SLIDER_CZ)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5, lower=0.0, upper=slide_travel),
    )

    # Roller blocks: FIXED to their respective sashes.
    # Positioned at the bottom of each sash, near left and right edges.
    roller_z_offset = -(SLIDER_H / 2.0 + SASH_FACE) - ROLLER_H / 2.0 + 0.005
    roller_x_offset = SASH_OPENING_W / 2.0 - ROLLER_W
    for i, x_off in enumerate([-roller_x_offset, roller_x_offset]):
        # Left sash rollers
        model.articulation(
            f"left_sash_to_roller_{i}",
            ArticulationType.FIXED,
            parent="left_sash",
            child=f"left_sash_roller_{i}",
            origin=Origin(xyz=(x_off, 0.0, roller_z_offset)),
        )
        # Right sash rollers
        model.articulation(
            f"right_sash_to_roller_{i}",
            ArticulationType.FIXED,
            parent="right_sash",
            child=f"right_sash_roller_{i}",
            origin=Origin(xyz=(x_off, 0.0, roller_z_offset)),
        )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    transom = object_model.get_part("transom")
    left_sash = object_model.get_part("left_sash")
    right_sash = object_model.get_part("right_sash")
    left_slide = object_model.get_articulation("frame_to_left_sash")
    right_slide = object_model.get_articulation("frame_to_right_sash")

    # --- Intentional overlaps ---
    # Glass panes tuck under the vinyl/muntin lip on each sash/transom.
    for nm in ("transom", "left_sash", "right_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_vinyl",
            reason="Clear pane is rebated under the sash/muntin lip so it reads captured, not floating.",
        )

    # Transom seated in the frame opening (rebate capture).
    ctx.allow_overlap(
        "frame", "transom",
        elem_a="frame_shell",
        elem_b="transom_vinyl",
        reason="Transom is rebated into the frame opening; its sash ring laps the jamb/head edge (seated capture).",
    )
    ctx.allow_overlap(
        "frame", "transom",
        elem_a="frame_shell",
        elem_b="transom_glass",
        reason="Transom glass is rebated under the frame opening lip (captured glazing).",
    )

    # Sliding sashes ride tracks and lap the frame face along the track.
    ctx.allow_overlap(
        "frame", "left_sash",
        elem_a="frame_shell",
        elem_b="left_sash_vinyl",
        reason="Left sash rides the head/sill track and laps the frame face along the track (slider capture).",
    )
    ctx.allow_overlap(
        "frame", "left_sash",
        elem_a="frame_shell",
        elem_b="left_sash_glass",
        reason="Left sash glass laps the head/sill track lip (captured glazing).",
    )
    ctx.allow_overlap(
        "frame", "right_sash",
        elem_a="frame_shell",
        elem_b="right_sash_vinyl",
        reason="Right sash rides the head/sill track and laps the frame face along the track (slider capture).",
    )
    ctx.allow_overlap(
        "frame", "right_sash",
        elem_a="frame_shell",
        elem_b="right_sash_glass",
        reason="Right sash glass laps the head/sill track lip (captured glazing).",
    )

    # Rollers sit at the bottom of each sash and contact the sill track.
    # Small intentional overlap between roller and sash frame bottom.
    for sash_nm in ("left_sash", "right_sash"):
        for i in range(2):
            ctx.allow_overlap(
                sash_nm, f"{sash_nm}_roller_{i}",
                elem_a=f"{sash_nm}_vinyl",
                elem_b=f"{sash_nm}_roller_{i}_body",
                reason=f"Roller block is seated against the bottom rail of the {sash_nm} (mounted hardware).",
            )

    # --- Transom geometry checks ---
    ctx.expect_gap(
        transom, left_sash,
        axis="z",
        min_gap=0.0,
        name="transom is above the left sash",
    )
    ctx.expect_overlap(
        transom, frame, axes="xz", min_overlap=0.03,
        name="transom seated in frame opening",
    )

    # --- Closed pose (q=0): window reads SHUT ---
    with ctx.pose({left_slide: 0.0, right_slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        l_aabb = ctx.part_world_aabb(left_sash)
        r_aabb = ctx.part_world_aabb(right_sash)

        # Frame spans the full width
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        ctx.check(
            "frame spans full window width",
            frame_w > 2.8,
            details=f"frame_w={frame_w:.3f}",
        )

        # Frame bottom at sill near z=0
        ctx.check(
            "sill sits near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"frame zmin={frame_aabb[0][2]:.4f}",
        )

        # Frame top at full height
        ctx.check(
            "head reaches full height",
            abs(frame_aabb[1][2] - TOTAL_H) < 0.02,
            details=f"frame zmax={frame_aabb[1][2]:.4f}",
        )

        # Left sash is to the left of right sash at rest
        lx = (l_aabb[0][0] + l_aabb[1][0]) / 2.0
        rx = (r_aabb[0][0] + r_aabb[1][0]) / 2.0
        ctx.check(
            "sashes ordered left-right at rest",
            lx < rx,
            details=f"left_x={lx:.3f}, right_x={rx:.3f}",
        )

        # Both sashes within frame height
        for nm, ab in (("left_sash", l_aabb), ("right_sash", r_aabb)):
            ctx.check(
                f"{nm} seated within frame height",
                ab[0][2] > frame_aabb[0][2] - 1e-4 and ab[1][2] < frame_aabb[1][2] + 1e-4,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}]",
            )

        # Sashes are on separate Y tracks
        l_y = (l_aabb[0][1] + l_aabb[1][1]) / 2.0
        r_y = (r_aabb[0][1] + r_aabb[1][1]) / 2.0
        ctx.check(
            "sashes on separate Y tracks",
            abs(l_y - r_y) > 0.02,
            details=f"left_y={l_y:.3f}, right_y={r_y:.3f}",
        )

        rest_lx = lx
        rest_rx = rx
        rest_lz = (l_aabb[0][2] + l_aabb[1][2]) / 2.0
        rest_rz = (r_aabb[0][2] + r_aabb[1][2]) / 2.0

    # --- Driven pose: both sashes slide toward center ---
    travel = left_slide.motion_limits.upper
    with ctx.pose({left_slide: travel, right_slide: travel}):
        l_open = ctx.part_world_aabb(left_sash)
        r_open = ctx.part_world_aabb(right_sash)
        open_lx = (l_open[0][0] + l_open[1][0]) / 2.0
        open_rx = (r_open[0][0] + r_open[1][0]) / 2.0

        # Left sash moved to the right (+X)
        ctx.check(
            "left sash slides along +X by ~travel",
            abs((open_lx - rest_lx) - travel) < 0.02,
            details=f"rest_lx={rest_lx:.3f}, open_lx={open_lx:.3f}, travel={travel:.3f}",
        )

        # Right sash moved to the left (-X)
        ctx.check(
            "right sash slides along -X by ~travel",
            abs((rest_rx - open_rx) - travel) < 0.02,
            details=f"rest_rx={rest_rx:.3f}, open_rx={open_rx:.3f}, travel={travel:.3f}",
        )

        # Both sashes remain at same Z (pure horizontal slide)
        open_lz = (l_open[0][2] + l_open[1][2]) / 2.0
        open_rz = (r_open[0][2] + r_open[1][2]) / 2.0
        ctx.check(
            "left slide is purely horizontal",
            abs(open_lz - rest_lz) < 0.02,
            details=f"open_z={open_lz:.3f}, rest_z={rest_lz:.3f}",
        )
        ctx.check(
            "right slide is purely horizontal",
            abs(open_rz - rest_rz) < 0.02,
            details=f"open_z={open_rz:.3f}, rest_z={rest_rz:.3f}",
        )

        # Retained: sashes still within frame X span at full travel
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "left sash retained within frame X span at full travel",
            l_open[1][0] < f_aabb[1][0] + 1e-4 and l_open[0][0] > f_aabb[0][0] - 1e-4,
            details=f"sash x=[{l_open[0][0]:.3f},{l_open[1][0]:.3f}]",
        )
        ctx.check(
            "right sash retained within frame X span at full travel",
            r_open[1][0] < f_aabb[1][0] + 1e-4 and r_open[0][0] > f_aabb[0][0] - 1e-4,
            details=f"sash x=[{r_open[0][0]:.3f},{r_open[1][0]:.3f}]",
        )

        # Sashes retain vertical engagement with track
        ctx.expect_overlap(
            left_sash, frame, axes="z", min_overlap=0.10,
            name="left sash retains vertical engagement at full travel",
        )
        ctx.expect_overlap(
            right_sash, frame, axes="z", min_overlap=0.10,
            name="right sash retains vertical engagement at full travel",
        )

    # --- Roller block checks ---
    for sash_nm, slide_joint in [("left_sash", left_slide), ("right_sash", right_slide)]:
        for i in range(2):
            roller_nm = f"{sash_nm}_roller_{i}"
            roller = object_model.get_part(roller_nm)
            sash = object_model.get_part(sash_nm)
            # Roller is below the sash
            ctx.expect_gap(
                sash, roller,
                axis="z",
                min_gap=-0.020,
                max_gap=0.010,
                name=f"{roller_nm} is at bottom of {sash_nm}",
            )
            # Roller overlaps sash in XY (mounted on it)
            ctx.expect_overlap(
                roller, sash, axes="xy", min_overlap=0.005,
                name=f"{roller_nm} mounted under {sash_nm}",
            )

    return ctx.report()


object_model = build_object_model()
