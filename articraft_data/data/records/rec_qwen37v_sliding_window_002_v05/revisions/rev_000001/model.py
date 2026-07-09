from __future__ import annotations

# Vertical sliding window: thick aluminum frame with deep track grooves
# in the head and sill rails. Upper sash is FIXED; lower sash slides
# UPWARD on a vertical prismatic joint. Recessed pull cup on the lower
# sash bottom rail.
#
# Coordinate convention:
#   +Z up, window stands vertically
#   X = width, Z = height (sill near z=0), Y = frame depth
#
# q=0: lower sash CLOSED (bottom of opening)
# q>0: lower sash slides UP along +Z
#
# Structure:
#   - frame (root): thick aluminum frame, hollow opening, 4 track grooves
#   - upper_sash (FIXED): upper half of opening, rear track
#   - lower_sash (PRISMATIC +Z): lower half, front track, pull cup

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
# Dimensions (meters)
# ---------------------------------------------------------------------------

TOTAL_W = 1.52            # overall width (X)
TOTAL_H = 1.72            # overall height (Z)

FRAME_FACE = 0.090        # thick aluminum rail/stile face width
FRAME_DEPTH = 0.140       # deep box section (Y)

# Deep track grooves (two parallel channels in head and sill)
GROOVE_W = 0.024          # groove width along Y
GROOVE_D = 0.030          # groove depth cut into rail along Z

REAR_TRACK_Y = -0.026     # rear groove center (upper sash track)
FRONT_TRACK_Y = 0.030     # front groove center (lower sash track)

SASH_FACE = 0.058         # sash rail/stile face width
SASH_DEPTH = 0.040        # sash depth along Y
GLASS_T = 0.006           # glass thickness

REBATE = 0.004            # glass rebate under sash lip

# Pull cup on lower sash bottom rail
CUP_W = 0.090             # cup width (X)
CUP_H = 0.028             # cup height (Z)
CUP_D = 0.014             # cup depth/thickness (Y)

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

ALUMINUM_RGBA = (0.72, 0.74, 0.77, 1.0)       # brushed aluminum
DARK_TRACK_RGBA = (0.28, 0.30, 0.33, 1.0)     # dark anodized track
GLASS_RGBA = (0.55, 0.64, 0.70, 0.25)         # clear glass, slight tint
CUP_RGBA = (0.22, 0.24, 0.27, 1.0)            # dark anodized pull cup

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

SASH_OUTER_W = INNER_W                   # sash fills opening width
SASH_OUTER_H = INNER_H / 2.0            # each sash is half the opening
GLASS_W = SASH_OUTER_W - 2 * SASH_FACE  # clear glass width
GLASS_H = SASH_OUTER_H - 2 * SASH_FACE  # clear glass height

UPPER_CZ = MID_Z + SASH_OUTER_H / 2.0   # upper sash center Z
LOWER_CZ = MID_Z - SASH_OUTER_H / 2.0   # lower sash center Z (closed)

SLIDE_TRAVEL = SASH_OUTER_H * 0.82       # max upward travel


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery, meters)
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float,
          yc: float, depth: float) -> cq.Workplane:
    """Axis-aligned box in XZ plane, centered at yc in Y."""
    return (
        cq.Workplane("XY")
        .transformed(offset=((x0 + x1) / 2.0, yc, (z0 + z1) / 2.0))
        .box(x1 - x0, depth, z1 - z0)
    )


def _build_frame() -> cq.Workplane:
    """Thick aluminum frame: outer slab with main opening + 4 deep track grooves
    (two in head, two in sill)."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    # Main opening (through-cut in Y)
    opening = _slab(INNER_X0, INNER_X1, INNER_Z0, INNER_Z1, 0.0,
                    FRAME_DEPTH + 0.02)
    frame = outer.cut(opening)

    # Head grooves: cut upward from the bottom face of the head rail
    for ty in (REAR_TRACK_Y, FRONT_TRACK_Y):
        groove = _slab(INNER_X0, INNER_X1,
                       INNER_Z1 - 0.001, INNER_Z1 + GROOVE_D,
                       ty, GROOVE_W + 0.002)
        frame = frame.cut(groove)

    # Sill grooves: cut downward from the top face of the sill rail
    for ty in (REAR_TRACK_Y, FRONT_TRACK_Y):
        groove = _slab(INNER_X0, INNER_X1,
                       INNER_Z0 - GROOVE_D, INNER_Z0 + 0.001,
                       ty, GROOVE_W + 0.002)
        frame = frame.cut(groove)

    return frame


def _build_sash_ring() -> cq.Workplane:
    """Hollow sash ring in its own local frame (centered at origin)."""
    ow, oh = SASH_OUTER_W, SASH_OUTER_H
    cw, ch = GLASS_W, GLASS_H
    outer = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH)
    inner = _slab(-cw / 2.0, cw / 2.0, -ch / 2.0, ch / 2.0, 0.0,
                  SASH_DEPTH + 0.02)
    return outer.cut(inner)


def _build_glass() -> cq.Workplane:
    """Glass pane in sash-local frame, rebated under the sash lip."""
    gw = GLASS_W + 2 * REBATE
    gh = GLASS_H + 2 * REBATE
    return _slab(-gw / 2.0, gw / 2.0, -gh / 2.0, gh / 2.0, 0.0, GLASS_T)


def _build_pull_cup() -> cq.Workplane:
    """Recessed pull cup: a plate with a finger-grip pocket cut from the front.
    Built in local frame: X=width, Y=depth(thin), Z=height."""
    plate = (
        cq.Workplane("XY")
        .box(CUP_W, CUP_D, CUP_H)
    )
    # Finger-grip pocket recessed from the +Y face
    pocket_w = CUP_W - 0.020
    pocket_h = CUP_H - 0.008
    pocket_d = CUP_D * 0.55
    pocket = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, CUP_D * 0.25, 0.0))
        .box(pocket_w, pocket_d, pocket_h)
    )
    return plate.cut(pocket)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_sliding_window")
    model.material("aluminum", rgba=ALUMINUM_RGBA)
    model.material("dark_track", rgba=DARK_TRACK_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("cup_metal", rgba=CUP_RGBA)

    # ── Frame (root) ────────────────────────────────────────────────
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame(), "frame_shell"),
        material="aluminum",
        name="frame_shell",
    )

    # Dark track strips visible in head and sill groove channels
    for suffix, ty in [("rear", REAR_TRACK_Y), ("front", FRONT_TRACK_Y)]:
        # Head track strip (sits proud into opening by 1 mm for visibility)
        frame.visual(
            Box((INNER_W * 0.98, GROOVE_W, GROOVE_D + 0.002)),
            origin=Origin(xyz=(0.0, ty,
                               INNER_Z1 + (GROOVE_D - 0.002) / 2.0)),
            material="dark_track",
            name=f"head_track_{suffix}",
        )
        # Sill track strip
        frame.visual(
            Box((INNER_W * 0.98, GROOVE_W, GROOVE_D + 0.002)),
            origin=Origin(xyz=(0.0, ty,
                               INNER_Z0 - (GROOVE_D - 0.002) / 2.0)),
            material="dark_track",
            name=f"sill_track_{suffix}",
        )

    # ── Upper sash (FIXED, rear track) ──────────────────────────────
    upper = model.part("upper_sash")
    upper.visual(
        mesh_from_cadquery(_build_sash_ring(), "upper_sash_ring"),
        material="aluminum",
        name="upper_sash_ring",
    )
    upper.visual(
        mesh_from_cadquery(_build_glass(), "upper_sash_glass"),
        material="glass",
        name="upper_sash_glass",
    )

    # ── Lower sash (PRISMATIC, front track, slides upward) ──────────
    lower = model.part("lower_sash")
    lower.visual(
        mesh_from_cadquery(_build_sash_ring(), "lower_sash_ring"),
        material="aluminum",
        name="lower_sash_ring",
    )
    lower.visual(
        mesh_from_cadquery(_build_glass(), "lower_sash_glass"),
        material="glass",
        name="lower_sash_glass",
    )

    # Pull cup on the bottom rail, front face, centered
    bottom_rail_z = -SASH_OUTER_H / 2.0 + SASH_FACE / 2.0
    cup_face_y = SASH_DEPTH / 2.0 + CUP_D / 2.0  # proud of front face
    lower.visual(
        mesh_from_cadquery(_build_pull_cup(), "pull_cup"),
        origin=Origin(xyz=(0.0, cup_face_y, bottom_rail_z)),
        material="cup_metal",
        name="pull_cup",
    )

    # ── Articulations ───────────────────────────────────────────────

    # Upper sash: fixed in rear track
    model.articulation(
        "frame_to_upper_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="upper_sash",
        origin=Origin(xyz=(0.0, REAR_TRACK_Y, UPPER_CZ)),
    )

    # Lower sash: prismatic along +Z (slides upward)
    model.articulation(
        "frame_to_lower_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="lower_sash",
        origin=Origin(xyz=(0.0, FRONT_TRACK_Y, LOWER_CZ)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0, velocity=0.3,
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
    upper = object_model.get_part("upper_sash")
    lower = object_model.get_part("lower_sash")
    slide = object_model.get_articulation("frame_to_lower_sash")

    # ── Intentional overlaps ────────────────────────────────────────

    # Glass rebated under sash lips (captured glazing)
    for nm in ("upper_sash", "lower_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_ring",
            reason="Glass pane rebated under sash lip (captured glazing).",
        )

    # Sashes seated in frame tracks
    for nm in ("upper_sash", "lower_sash"):
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_ring",
            reason=f"{nm} ring seated in frame track channel (retained sash).",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_glass",
            reason=f"{nm} glass within frame opening rebate.",
        )

    # Track strip visuals seated in frame groove channels
    for suffix in ("rear", "front"):
        for loc in ("head", "sill"):
            ctx.allow_overlap(
                "frame", "frame",
                elem_a="frame_shell",
                elem_b=f"{loc}_track_{suffix}",
                reason=f"Track strip seated in frame {loc} groove channel.",
            )

    # Pull cup mounted on lower sash bottom rail
    ctx.allow_overlap(
        "lower_sash", "lower_sash",
        elem_a="pull_cup",
        elem_b="lower_sash_ring",
        reason="Pull cup mounted on lower sash bottom rail front face.",
    )

    # ── Closed pose (q=0) ──────────────────────────────────────────

    with ctx.pose({slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        upper_aabb = ctx.part_world_aabb(upper)
        lower_aabb = ctx.part_world_aabb(lower)

        # Frame proportions
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        frame_h = frame_aabb[1][2] - frame_aabb[0][2]
        ctx.check(
            "frame spans full width",
            frame_w > 1.40,
            details=f"w={frame_w:.3f}",
        )
        ctx.check(
            "frame spans full height",
            frame_h > 1.60,
            details=f"h={frame_h:.3f}",
        )
        ctx.check(
            "sill near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"zmin={frame_aabb[0][2]:.4f}",
        )

        # Upper sash above lower sash (vertical arrangement)
        upper_cz = (upper_aabb[0][2] + upper_aabb[1][2]) / 2.0
        lower_cz = (lower_aabb[0][2] + lower_aabb[1][2]) / 2.0
        ctx.check(
            "upper sash above lower sash",
            upper_cz > lower_cz + 0.30,
            details=f"upper_z={upper_cz:.3f}, lower_z={lower_cz:.3f}",
        )

        # Sashes in different Y planes (front/back tracks)
        upper_cy = (upper_aabb[0][1] + upper_aabb[1][1]) / 2.0
        lower_cy = (lower_aabb[0][1] + lower_aabb[1][1]) / 2.0
        ctx.check(
            "lower sash in front track, upper in rear",
            lower_cy > upper_cy + 0.02,
            details=f"lower_y={lower_cy:.3f}, upper_y={upper_cy:.3f}",
        )

        # Pull cup near bottom of lower sash
        cup_aabb = ctx.part_element_world_aabb(lower, elem="pull_cup")
        cup_cz = (cup_aabb[0][2] + cup_aabb[1][2]) / 2.0
        ctx.check(
            "pull cup near bottom of lower sash",
            cup_cz < lower_cz,
            details=f"cup_z={cup_cz:.3f}, sash_center_z={lower_cz:.3f}",
        )

        # Track grooves present in head and sill
        head_rear = ctx.part_element_world_aabb(frame, elem="head_track_rear")
        sill_front = ctx.part_element_world_aabb(frame, elem="sill_track_front")
        ctx.check(
            "head track near top of opening",
            head_rear[0][2] > INNER_Z1 - 0.01,
            details=f"head_track_zmin={head_rear[0][2]:.3f}",
        )
        ctx.check(
            "sill track near bottom of opening",
            sill_front[1][2] < INNER_Z0 + 0.01,
            details=f"sill_track_zmax={sill_front[1][2]:.3f}",
        )

        # Sashes seated within frame
        for nm, ab in (("upper", upper_aabb), ("lower", lower_aabb)):
            ctx.check(
                f"{nm} sash within frame height",
                ab[0][2] > frame_aabb[0][2] - 1e-4
                and ab[1][2] < frame_aabb[1][2] + 1e-4,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}]",
            )

        # Both sashes overlap frame in XZ (seated in tracks)
        ctx.expect_overlap(
            upper, frame, axes="xz", min_overlap=0.03,
            name="upper sash seated in frame opening",
        )
        ctx.expect_overlap(
            lower, frame, axes="xz", min_overlap=0.03,
            name="lower sash seated in frame opening",
        )

        rest_lower_cz = lower_cz

    # ── Open pose: lower sash slides upward ─────────────────────────

    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel}):
        open_aabb = ctx.part_world_aabb(lower)
        open_cz = (open_aabb[0][2] + open_aabb[1][2]) / 2.0

        # Lower sash moved upward
        ctx.check(
            "lower sash slides upward",
            open_cz > rest_lower_cz + 0.30,
            details=f"rest_z={rest_lower_cz:.3f}, open_z={open_cz:.3f}",
        )

        # Pure vertical slide (no X drift)
        open_cx = (open_aabb[0][0] + open_aabb[1][0]) / 2.0
        ctx.check(
            "slide is purely vertical (no X drift)",
            abs(open_cx) < 0.02,
            details=f"open_x={open_cx:.3f}",
        )

        # Sash retained within frame at max travel
        ctx.check(
            "lower sash retained within frame at max travel",
            open_aabb[1][2] < frame_aabb[1][2] + 0.01,
            details=(
                f"sash_zmax={open_aabb[1][2]:.3f}, "
                f"frame_zmax={frame_aabb[1][2]:.3f}"
            ),
        )

        # Retains X engagement with frame
        ctx.expect_overlap(
            lower, frame, axes="x", min_overlap=0.50,
            name="lower sash retains X engagement at max travel",
        )

    # ── Joint verification ──────────────────────────────────────────

    ctx.check(
        "slide joint is prismatic (non-fixed)",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )
    ctx.check(
        "slide has positive upward travel",
        slide.motion_limits.upper is not None
        and slide.motion_limits.upper > 0.1,
        details=f"upper={slide.motion_limits.upper}",
    )

    return ctx.report()


object_model = build_object_model()
