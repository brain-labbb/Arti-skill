from __future__ import annotations

# Vertical double-hung sliding window, white vinyl frame.
# Two sashes slide vertically in opposite directions on separate prismatic joints:
#   - Lower sash slides upward (+Z) to open
#   - Upper sash slides downward (-Z) to open
# Deep track grooves along head and sill rails, plus vertical jamb channels.
# Rubber gasket strips around each glass pane.
# Cam-latch handle on lower sash meeting rail.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     width  -> X
#     height -> Z  (sill near z=0)
#     depth  -> Y
#   Glass plane is X-Z. q=0 reads SHUT (both sashes closed at mid-height).

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

TOTAL_W = 1.20
TOTAL_H = 1.50

FRAME_FACE = 0.075          # frame member face width
FRAME_DEPTH = 0.120         # frame depth along Y

SASH_FACE = 0.055           # sash rail/stile face width
SASH_DEPTH = 0.040          # sash depth along Y
GLASS_T = 0.006             # glass thickness
REBATE = 0.004              # glass tucks under sash lip

GROOVE_DEPTH = 0.022        # head/sill groove depth (into rail)
GROOVE_DEPTH_JAMB = 0.016   # jamb groove depth (into jamb)
GROOVE_WIDTH = SASH_DEPTH + 0.006  # groove accepts sash rail
GROOVE_EMBED = 0.010        # sash rail extends into groove

GASKET_W = 0.008            # gasket strip width
GASKET_T = 0.004            # gasket strip thickness

LATCH_BASE_W = 0.050
LATCH_BASE_H = 0.022
LATCH_BASE_T = 0.008
LATCH_LEVER_LEN = 0.035
LATCH_LEVER_R = 0.005

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)
GLASS_RGBA = (0.52, 0.60, 0.66, 0.30)
GASKET_RGBA = (0.12, 0.12, 0.12, 1.0)   # dark rubber
METAL_RGBA = (0.74, 0.76, 0.79, 1.0)

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

# Sash outer dimensions: each covers half the inner height + groove embedment
SASH_OUTER_W = INNER_W + 2.0 * GROOVE_DEPTH_JAMB * 0.6
SASH_OUTER_H = INNER_H / 2.0 + GROOVE_EMBED

# Glass opening within each sash
GLASS_OPEN_W = SASH_OUTER_W - 2.0 * SASH_FACE
GLASS_OPEN_H = SASH_OUTER_H - 2.0 * SASH_FACE

# Y positions for the two tracks (upper rear, lower front)
UPPER_SASH_Y = -(GROOVE_WIDTH / 2.0 + 0.004)
LOWER_SASH_Y = +(GROOVE_WIDTH / 2.0 + 0.004)

# Closed sash center-Z positions
UPPER_SASH_CZ = MID_Z + INNER_H / 4.0 + GROOVE_EMBED / 2.0
LOWER_SASH_CZ = MID_Z - INNER_H / 4.0 - GROOVE_EMBED / 2.0

# Travel
SASH_TRAVEL = INNER_H * 0.38


# ---------------------------------------------------------------------------
# CadQuery geometry helpers
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float,
          y_center: float, depth: float) -> cq.Workplane:
    """Axis-aligned box in X-Z plane, centered at y_center with given Y depth."""
    return (
        cq.Workplane("XY")
        .transformed(offset=((x0 + x1) / 2.0, y_center, (z0 + z1) / 2.0))
        .box(x1 - x0, depth, z1 - z0)
    )


def _build_frame_shape() -> cq.Workplane:
    """Outer frame: head, sill, jambs with deep track grooves."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    opening = _slab(INNER_X0, INNER_X1, INNER_Z0, INNER_Z1, 0.0, FRAME_DEPTH + 0.02)
    frame = outer.cut(opening)

    for yc in (UPPER_SASH_Y, LOWER_SASH_Y):
        # Head grooves (channels in bottom face of head rail)
        frame = frame.cut(
            _slab(INNER_X0, INNER_X1, INNER_Z1, INNER_Z1 + GROOVE_DEPTH, yc, GROOVE_WIDTH)
        )
        # Sill grooves (channels in top face of sill rail)
        frame = frame.cut(
            _slab(INNER_X0, INNER_X1, INNER_Z0 - GROOVE_DEPTH, INNER_Z0, yc, GROOVE_WIDTH)
        )
        # Left jamb grooves (vertical channels)
        frame = frame.cut(
            _slab(INNER_X0 - GROOVE_DEPTH_JAMB, INNER_X0, INNER_Z0, INNER_Z1, yc, GROOVE_WIDTH)
        )
        # Right jamb grooves (vertical channels)
        frame = frame.cut(
            _slab(INNER_X1, INNER_X1 + GROOVE_DEPTH_JAMB, INNER_Z0, INNER_Z1, yc, GROOVE_WIDTH)
        )
    return frame


def _build_sash_shape() -> cq.Workplane:
    """Sash ring in local frame, centered at origin."""
    ow = SASH_OUTER_W
    oh = SASH_OUTER_H
    outer = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH)
    hole = _slab(-GLASS_OPEN_W / 2.0, GLASS_OPEN_W / 2.0,
                 -GLASS_OPEN_H / 2.0, GLASS_OPEN_H / 2.0,
                 0.0, SASH_DEPTH + 0.02)
    return outer.cut(hole)


def _build_glass_shape() -> cq.Workplane:
    """Glass pane in sash-local frame, rebated under the sash lip."""
    w = GLASS_OPEN_W + 2.0 * REBATE
    h = GLASS_OPEN_H + 2.0 * REBATE
    return _slab(-w / 2.0, w / 2.0, -h / 2.0, h / 2.0, 0.0, GLASS_T)


def _build_gasket_shape() -> cq.Workplane:
    """Rubber gasket frame on front face of glass, in sash-local frame."""
    outer_w = GLASS_OPEN_W + 2.0 * GASKET_W
    outer_h = GLASS_OPEN_H + 2.0 * GASKET_W
    y_front = GLASS_T / 2.0 + GASKET_T / 2.0
    outer = _slab(-outer_w / 2.0, outer_w / 2.0,
                  -outer_h / 2.0, outer_h / 2.0, y_front, GASKET_T)
    hole = _slab(-GLASS_OPEN_W / 2.0, GLASS_OPEN_W / 2.0,
                 -GLASS_OPEN_H / 2.0, GLASS_OPEN_H / 2.0,
                 y_front, GASKET_T + 0.004)
    return outer.cut(hole)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_sliding_window")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("gasket", rgba=GASKET_RGBA)
    model.material("metal", rgba=METAL_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="vinyl",
        name="frame_shell",
    )

    # --- Upper sash (rear track, slides downward to open) ---
    upper = model.part("upper_sash")
    upper.visual(
        mesh_from_cadquery(_build_sash_shape(), "upper_sash_vinyl"),
        material="vinyl",
        name="upper_sash_vinyl",
    )
    upper.visual(
        mesh_from_cadquery(_build_glass_shape(), "upper_sash_glass"),
        material="glass",
        name="upper_sash_glass",
    )
    upper.visual(
        mesh_from_cadquery(_build_gasket_shape(), "upper_sash_gasket"),
        material="gasket",
        name="upper_sash_gasket",
    )

    # --- Lower sash (front track, slides upward to open) ---
    lower = model.part("lower_sash")
    lower.visual(
        mesh_from_cadquery(_build_sash_shape(), "lower_sash_vinyl"),
        material="vinyl",
        name="lower_sash_vinyl",
    )
    lower.visual(
        mesh_from_cadquery(_build_glass_shape(), "lower_sash_glass"),
        material="glass",
        name="lower_sash_glass",
    )
    lower.visual(
        mesh_from_cadquery(_build_gasket_shape(), "lower_sash_gasket"),
        material="gasket",
        name="lower_sash_gasket",
    )

    # Latch on lower sash meeting rail (top rail)
    meeting_z = SASH_OUTER_H / 2.0 - SASH_FACE / 2.0
    face_y = SASH_DEPTH / 2.0
    plate_y = face_y + LATCH_BASE_T / 2.0
    lower.visual(
        Box((LATCH_BASE_W, LATCH_BASE_T, LATCH_BASE_H)),
        origin=Origin(xyz=(0.0, plate_y, meeting_z)),
        material="metal",
        name="lower_latch_plate",
    )
    lever_y = face_y + LATCH_BASE_T + LATCH_LEVER_LEN / 2.0
    lower.visual(
        Cylinder(radius=LATCH_LEVER_R, length=LATCH_LEVER_LEN),
        origin=Origin(xyz=(0.0, lever_y, meeting_z), rpy=(1.5708, 0.0, 0.0)),
        material="metal",
        name="lower_latch_lever",
    )

    # --- Articulations ---

    # Upper sash: PRISMATIC along -Z (positive q slides downward)
    model.articulation(
        "frame_to_upper_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="upper_sash",
        origin=Origin(xyz=(0.0, UPPER_SASH_Y, UPPER_SASH_CZ)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.3,
                                   lower=0.0, upper=SASH_TRAVEL),
    )

    # Lower sash: PRISMATIC along +Z (positive q slides upward)
    model.articulation(
        "frame_to_lower_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="lower_sash",
        origin=Origin(xyz=(0.0, LOWER_SASH_Y, LOWER_SASH_CZ)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.3,
                                   lower=0.0, upper=SASH_TRAVEL),
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

    # --- Overlap allowances ---
    for nm in ("upper_sash", "lower_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass", elem_b=f"{nm}_vinyl",
            reason="Glass pane rebated under sash lip (captured glazing).",
        )
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_gasket", elem_b=f"{nm}_vinyl",
            reason="Rubber gasket seated between glass edge and sash frame pocket.",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell", elem_b=f"{nm}_vinyl",
            reason=f"{nm} rails/stiles seat in frame track grooves (retained capture).",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell", elem_b=f"{nm}_glass",
            reason=f"{nm} glass within frame opening rebate region.",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell", elem_b=f"{nm}_gasket",
            reason=f"{nm} gasket seated at frame rebate edge.",
        )

    # Latch mounted on lower sash meeting rail
    ctx.allow_overlap(
        "lower_sash", "lower_sash",
        elem_a="lower_latch_plate", elem_b="lower_sash_vinyl",
        reason="Latch plate seated on lower sash meeting rail front face.",
    )

    # --- Closed pose (q=0): window reads SHUT ---
    with ctx.pose({lower_slide: 0.0, upper_slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        upper_aabb = ctx.part_world_aabb(upper_sash)
        lower_aabb = ctx.part_world_aabb(lower_sash)

        # Frame proportions
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        frame_h = frame_aabb[1][2] - frame_aabb[0][2]
        ctx.check(
            "frame width ~1.2m",
            abs(frame_w - TOTAL_W) < 0.02,
            details=f"frame_w={frame_w:.3f}",
        )
        ctx.check(
            "frame height ~1.5m",
            abs(frame_h - TOTAL_H) < 0.02,
            details=f"frame_h={frame_h:.3f}",
        )
        ctx.check(
            "sill near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"frame zmin={frame_aabb[0][2]:.4f}",
        )

        # Sashes stacked vertically (upper above lower)
        upper_cz = (upper_aabb[0][2] + upper_aabb[1][2]) / 2.0
        lower_cz = (lower_aabb[0][2] + lower_aabb[1][2]) / 2.0
        ctx.check(
            "upper sash above lower sash",
            upper_cz > lower_cz + 0.20,
            details=f"upper_z={upper_cz:.3f}, lower_z={lower_cz:.3f}",
        )

        # Sashes on different Y tracks
        upper_cy = (upper_aabb[0][1] + upper_aabb[1][1]) / 2.0
        lower_cy = (lower_aabb[0][1] + lower_aabb[1][1]) / 2.0
        ctx.check(
            "sashes on separate Y tracks",
            abs(upper_cy - lower_cy) > 0.02,
            details=f"upper_y={upper_cy:.4f}, lower_y={lower_cy:.4f}",
        )

        # Both sashes within frame X span
        ctx.expect_within(upper_sash, frame, axes="x", margin=0.01,
                          name="upper sash within frame X")
        ctx.expect_within(lower_sash, frame, axes="x", margin=0.01,
                          name="lower sash within frame X")

        # Sashes seated within frame height
        for nm, ab in (("upper", upper_aabb), ("lower", lower_aabb)):
            ctx.check(
                f"{nm} sash within frame height",
                ab[0][2] > frame_aabb[0][2] - 0.015 and ab[1][2] < frame_aabb[1][2] + 0.015,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}]",
            )

        # Gasket visible on each sash
        for nm in ("upper_sash", "lower_sash"):
            gasket_aabb = ctx.part_element_world_aabb(
                object_model.get_part(nm), elem=f"{nm}_gasket"
            )
            ctx.check(
                f"{nm} has gasket strip",
                gasket_aabb is not None,
                details="gasket visual missing",
            )

        # Latch on lower sash meeting rail (near mid-height, front face)
        latch_aabb = ctx.part_element_world_aabb(lower_sash, elem="lower_latch_plate")
        latch_cz = (latch_aabb[0][2] + latch_aabb[1][2]) / 2.0
        latch_cy = (latch_aabb[0][1] + latch_aabb[1][1]) / 2.0
        ctx.check(
            "latch near meeting rail (mid-height)",
            abs(latch_cz - MID_Z) < 0.10,
            details=f"latch_z={latch_cz:.3f}, mid_z={MID_Z:.3f}",
        )
        ctx.check(
            "latch on front face of lower sash",
            latch_cy > lower_cy,
            details=f"latch_y={latch_cy:.3f}, sash_y={lower_cy:.3f}",
        )

        rest_lower_cz = lower_cz
        rest_upper_cz = upper_cz

    # --- Lower sash opens upward ---
    with ctx.pose({lower_slide: SASH_TRAVEL}):
        open_lower = ctx.part_world_aabb(lower_sash)
        open_cz = (open_lower[0][2] + open_lower[1][2]) / 2.0
        ctx.check(
            "lower sash slides upward (+Z)",
            open_cz > rest_lower_cz + 0.10,
            details=f"rest_z={rest_lower_cz:.3f}, open_z={open_cz:.3f}",
        )
        # Retained within frame
        ctx.expect_overlap(
            lower_sash, frame, axes="x", min_overlap=0.10,
            name="lower sash retained in frame at max travel",
        )
        # Sash stays within frame Z
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "lower sash top below head at max travel",
            open_lower[1][2] < f_aabb[1][2] + 0.02,
            details=f"sash_zmax={open_lower[1][2]:.3f}, frame_zmax={f_aabb[1][2]:.3f}",
        )

    # --- Upper sash opens downward ---
    with ctx.pose({upper_slide: SASH_TRAVEL}):
        open_upper = ctx.part_world_aabb(upper_sash)
        open_upper_cz = (open_upper[0][2] + open_upper[1][2]) / 2.0
        ctx.check(
            "upper sash slides downward (-Z)",
            open_upper_cz < rest_upper_cz - 0.10,
            details=f"rest_z={rest_upper_cz:.3f}, open_z={open_upper_cz:.3f}",
        )
        ctx.expect_overlap(
            upper_sash, frame, axes="x", min_overlap=0.10,
            name="upper sash retained in frame at max travel",
        )

    # --- Prismatic joint checks ---
    ctx.check(
        "lower sash joint is prismatic",
        lower_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={lower_slide.articulation_type}",
    )
    ctx.check(
        "upper sash joint is prismatic",
        upper_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={upper_slide.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
