from __future__ import annotations

# Two-panel horizontal sliding window variant, white vinyl frame.
# BOTH sashes slide on separate prismatic joints in OPPOSITE directions:
#   - rear_sash (rear track, -Y): slides RIGHT (+X) to open
#   - front_sash (front track, +Y): slides LEFT (-X) to open
# At rest (q=0), the rear sash is partially open to show the overlap.
# Two tiny roller blocks at the bottom of each moving sash.
# Sill lip protrudes forward; drainage slots cut through the sill.
# A cam-latch handle is mounted on the front sash meeting stile.
#
# Coordinate convention:
#   +Z up, window stands vertically
#   width  -> X
#   height -> Z  (sill near z=0)
#   depth  -> Y  (glazing plane / slide-normal)

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

FRAME_FACE = 0.085         # outer frame member face width
FRAME_DEPTH = 0.140        # deep box section along Y

MEETING_OVERLAP = 0.040    # sash stile overlap at center when closed

SASH_FACE = 0.075          # sash rail/stile face width
SASH_DEPTH = 0.060         # sash depth along Y
GLASS_T = 0.008            # glazing thickness

# Y track layout: rear sash behind, front sash proud
REAR_SASH_Y = -0.028
FRONT_SASH_Y = 0.044

REBATE = 0.005             # glass tucks under sash lip

# Sill lip: thin shelf protruding forward from sill bottom
SILL_LIP_EXTEND = 0.030    # forward extent in Y
SILL_LIP_THICK = 0.012     # thickness in Z

# Drainage slots (weep holes) in the sill
DRAIN_SLOT_W = 0.040       # slot width (X)
DRAIN_SLOT_H = 0.008       # slot height (Z)
NUM_DRAIN_SLOTS = 3

# Roller blocks at sash bottoms
ROLLER_W = 0.025           # width (X)
ROLLER_H = 0.010           # height (Z)
ROLLER_D = 0.018           # depth (Y)

# Latch hardware
LATCH_PLATE_W = 0.028
LATCH_PLATE_H = 0.075
LATCH_PLATE_T = 0.010
LATCH_LEVER_LEN = 0.045
LATCH_LEVER_R = 0.006

# Partial opening at rest: rear sash is slid this far right at q=0
PARTIAL_OPEN = 0.20

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)
GLASS_RGBA = (0.52, 0.60, 0.66, 0.30)
METAL_RGBA = (0.74, 0.76, 0.79, 1.0)
ROLLER_RGBA = (0.30, 0.30, 0.32, 1.0)   # dark grey nylon

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

# Closed-position world X centers for each sash
REAR_CLOSED_CX = INNER_X0 + SASH_OPENING_W / 2.0
FRONT_CLOSED_CX = INNER_X1 - SASH_OPENING_W / 2.0
MID_CZ = (INNER_Z0 + INNER_Z1) / 2.0

# Rear sash at rest is partially open (shifted right by PARTIAL_OPEN)
REAR_REST_CX = REAR_CLOSED_CX + PARTIAL_OPEN

# Travel limits: sash must remain within frame X span at full travel
_SASH_HALF_W = (SASH_OPENING_W + 2 * SASH_FACE) / 2.0
REAR_TRAVEL = min(
    SASH_OPENING_W * 0.80,
    HALF_W - _SASH_HALF_W - REAR_REST_CX - 0.005,
)
FRONT_TRAVEL = min(
    SASH_OPENING_W * 0.90,
    FRONT_CLOSED_CX - (-HALF_W + _SASH_HALF_W) - 0.005,
)


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery)
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float,
          y_center: float, depth: float) -> cq.Workplane:
    """Axis-aligned box in X-Z plane, centered on y_center with Y depth."""
    return (
        cq.Workplane("XY")
        .transformed(offset=((x0 + x1) / 2.0, y_center, (z0 + z1) / 2.0))
        .box(x1 - x0, depth, z1 - z0)
    )


def _build_frame_shape() -> cq.Workplane:
    """Frame: thick hollow slab with sill lip and drainage slots."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    cut_depth = FRAME_DEPTH + 0.02
    opening = _slab(INNER_X0, INNER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    frame = outer.cut(opening)

    # Sill lip: thin shelf protruding forward (+Y) from the sill bottom
    sill_front_y = FRAME_DEPTH / 2.0
    lip_y_center = sill_front_y + SILL_LIP_EXTEND / 2.0
    lip = _slab(
        -HALF_W, HALF_W,
        0.0, SILL_LIP_THICK,
        lip_y_center, SILL_LIP_EXTEND,
    )
    frame = frame.union(lip)

    # Drainage slots: small rectangular weep holes cut through the sill
    slot_spacing = INNER_W / (NUM_DRAIN_SLOTS + 1)
    slot_z_center = FRAME_FACE * 0.30
    for i in range(NUM_DRAIN_SLOTS):
        sx = INNER_X0 + slot_spacing * (i + 1)
        slot = _slab(
            sx - DRAIN_SLOT_W / 2.0, sx + DRAIN_SLOT_W / 2.0,
            slot_z_center - DRAIN_SLOT_H / 2.0,
            slot_z_center + DRAIN_SLOT_H / 2.0,
            sill_front_y,
            SILL_LIP_EXTEND + 0.02,  # cuts through lip and sill face
        )
        frame = frame.cut(slot)

    return frame


def _build_sash_shape() -> cq.Workplane:
    """Hollow sash ring in local frame (origin at sash center)."""
    ow = SASH_OPENING_W
    oh = SASH_OPENING_H
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SASH_DEPTH)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH + 0.02)
    return outer.cut(opening)


def _build_sash_glass_shape() -> cq.Workplane:
    """Clear pane, slightly rebated under the sash lip."""
    ow = SASH_OPENING_W + 2 * REBATE
    oh = SASH_OPENING_H + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_panel_sliding_window_variant")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("metal", rgba=METAL_RGBA)
    model.material("roller", rgba=ROLLER_RGBA)

    # --- Frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="vinyl",
        name="frame_shell",
    )

    # --- Rear sash (rear track, slides +X to open) ---
    rear = model.part("rear_sash")
    rear.visual(
        mesh_from_cadquery(_build_sash_shape(), "rear_sash_vinyl"),
        material="vinyl",
        name="rear_sash_vinyl",
    )
    rear.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "rear_sash_glass"),
        material="glass",
        name="rear_sash_glass",
    )
    # Two roller blocks at bottom of rear sash
    _add_rollers(rear, "rear")

    # --- Front sash (front track, slides -X to open) ---
    front = model.part("front_sash")
    front.visual(
        mesh_from_cadquery(_build_sash_shape(), "front_sash_vinyl"),
        material="vinyl",
        name="front_sash_vinyl",
    )
    front.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "front_sash_glass"),
        material="glass",
        name="front_sash_glass",
    )
    # Two roller blocks at bottom of front sash
    _add_rollers(front, "front")

    # Latch on front sash meeting stile
    _add_latch(front)

    # --- Articulations ---
    # Rear sash: PRISMATIC along +X. Origin at partially-open rest position.
    # Positive q slides further right to open.
    model.articulation(
        "frame_to_rear_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="rear_sash",
        origin=Origin(xyz=(REAR_REST_CX, REAR_SASH_Y, MID_CZ)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.5,
            lower=0.0, upper=REAR_TRAVEL,
        ),
    )

    # Front sash: PRISMATIC along -X. Origin at closed position.
    # Positive q slides left to open.
    model.articulation(
        "frame_to_front_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="front_sash",
        origin=Origin(xyz=(FRONT_CLOSED_CX, FRONT_SASH_Y, MID_CZ)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.5,
            lower=0.0, upper=FRONT_TRAVEL,
        ),
    )

    return model


def _add_rollers(part, prefix: str) -> None:
    """Add two tiny roller blocks at the bottom rail of a sash part."""
    # Bottom outer edge of sash: z = -(SASH_OPENING_H/2 + SASH_FACE)
    bottom_z = -(SASH_OPENING_H / 2.0 + SASH_FACE)
    # Roller center: partially embedded (3mm) below sash bottom
    roller_cz = bottom_z - ROLLER_H / 2.0 + 0.003
    # X positions: near corners of the bottom rail
    roller_span = SASH_OPENING_W * 0.38
    for i, sign in enumerate((-1, 1)):
        part.visual(
            Box((ROLLER_W, ROLLER_D, ROLLER_H)),
            origin=Origin(xyz=(sign * roller_span, 0.0, roller_cz)),
            material="roller",
            name=f"{prefix}_roller_{i}",
        )


def _add_latch(part) -> None:
    """Cam-latch hardware on the front sash meeting stile."""
    stile_x = -SASH_OPENING_W / 2.0 - SASH_FACE / 2.0
    face_y = SASH_DEPTH / 2.0
    plate_y = face_y + LATCH_PLATE_T / 2.0

    # Keeper plate
    part.visual(
        Box((LATCH_PLATE_W, LATCH_PLATE_T, LATCH_PLATE_H)),
        origin=Origin(xyz=(stile_x, plate_y, 0.0)),
        material="metal",
        name="front_sash_latch_plate",
    )
    # Lever arm
    lever_y = face_y + LATCH_PLATE_T + LATCH_LEVER_LEN / 2.0
    part.visual(
        Cylinder(radius=LATCH_LEVER_R, length=LATCH_LEVER_LEN),
        origin=Origin(xyz=(stile_x, lever_y, -0.008), rpy=(1.5707963, 0.0, 0.0)),
        material="metal",
        name="front_sash_latch_lever",
    )


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

    # --- Intentional-overlap allowances ---
    for nm in ("rear_sash", "front_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass", elem_b=f"{nm}_vinyl",
            reason="Clear pane rebated under sash lip (captured glazing).",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell", elem_b=f"{nm}_vinyl",
            reason=f"{nm} ring rebated into frame head/sill track (seated capture).",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell", elem_b=f"{nm}_glass",
            reason=f"{nm} glass rebated under frame opening lip.",
        )
        pfx = nm.replace("_sash", "")
        for i in range(2):
            ctx.allow_overlap(
                nm, nm,
                elem_a=f"{pfx}_roller_{i}", elem_b=f"{nm}_vinyl",
                reason="Roller block seated into sash bottom rail.",
            )

    ctx.allow_overlap(
        "front_sash", "front_sash",
        elem_a="front_sash_latch_plate", elem_b="front_sash_vinyl",
        reason="Latch keeper plate seated on front sash meeting stile.",
    )

    # --- Rest-pose checks (q=0 for both joints) ---
    frame_aabb = ctx.part_world_aabb(frame)
    rear_aabb = ctx.part_world_aabb(rear_sash)
    front_aabb = ctx.part_world_aabb(front_sash)

    # Frame proportions
    frame_w = frame_aabb[1][0] - frame_aabb[0][0]
    ctx.check("frame spans full width",
              frame_w > 1.4, details=f"frame_w={frame_w:.3f}")
    ctx.check("sill near z=0",
              abs(frame_aabb[0][2]) < 0.02, details=f"zmin={frame_aabb[0][2]:.4f}")
    ctx.check("head at full height",
              abs(frame_aabb[1][2] - TOTAL_H) < 0.02,
              details=f"zmax={frame_aabb[1][2]:.4f}")

    # Rear sash is partially open at rest
    rear_cx = (rear_aabb[0][0] + rear_aabb[1][0]) / 2.0
    ctx.check(
        "rear sash partially open at rest",
        rear_cx > REAR_CLOSED_CX + PARTIAL_OPEN * 0.4,
        details=f"rear_cx={rear_cx:.3f}, closed_cx={REAR_CLOSED_CX:.3f}",
    )

    # Front sash is at closed position
    front_cx = (front_aabb[0][0] + front_aabb[1][0]) / 2.0
    ctx.check(
        "front sash at closed position",
        abs(front_cx - FRONT_CLOSED_CX) < 0.02,
        details=f"front_cx={front_cx:.3f}, expected={FRONT_CLOSED_CX:.3f}",
    )

    # Visible overlap between the two sashes at rest
    ctx.expect_overlap(
        rear_sash, front_sash, axes="x", min_overlap=0.01,
        name="sash overlap visible at rest",
    )

    # Sashes on separate Y tracks
    rear_cy = (rear_aabb[0][1] + rear_aabb[1][1]) / 2.0
    front_cy = (front_aabb[0][1] + front_aabb[1][1]) / 2.0
    ctx.check(
        "sashes on separate Y tracks",
        front_cy > rear_cy + 0.02,
        details=f"front_y={front_cy:.3f}, rear_y={rear_cy:.3f}",
    )

    # Both sashes retained within frame
    ctx.expect_overlap(rear_sash, frame, axes="xz", min_overlap=0.03,
                       name="rear sash retained in frame")
    ctx.expect_overlap(front_sash, frame, axes="xz", min_overlap=0.03,
                       name="front sash retained in frame")

    # Sill lip extends beyond main frame depth
    ctx.check(
        "sill lip extends forward beyond frame front face",
        frame_aabb[1][1] > FRAME_DEPTH / 2.0 + 0.01,
        details=f"frame ymax={frame_aabb[1][1]:.3f}, expected>{FRAME_DEPTH / 2.0 + 0.01:.3f}",
    )

    rest_rear_cx = rear_cx
    rest_front_cx = front_cx

    # --- Rear sash slides further right (+X) ---
    with ctx.pose({rear_slide: REAR_TRAVEL}):
        rear_open_aabb = ctx.part_world_aabb(rear_sash)
        open_cx = (rear_open_aabb[0][0] + rear_open_aabb[1][0]) / 2.0
        ctx.check(
            "rear sash slides right (+X) to open further",
            open_cx > rest_rear_cx + 0.1,
            details=f"rest={rest_rear_cx:.3f}, open={open_cx:.3f}",
        )
        # Retained within frame X span
        ctx.check(
            "rear sash retained in frame at full travel",
            rear_open_aabb[0][0] > frame_aabb[0][0] - 0.01
            and rear_open_aabb[1][0] < frame_aabb[1][0] + 0.01,
            details=f"sash x=[{rear_open_aabb[0][0]:.3f},{rear_open_aabb[1][0]:.3f}]",
        )
        ctx.expect_overlap(
            rear_sash, frame, axes="z", min_overlap=0.10,
            name="rear sash retains vertical track engagement",
        )

    # --- Front sash slides left (-X) ---
    with ctx.pose({front_slide: FRONT_TRAVEL}):
        front_open_aabb = ctx.part_world_aabb(front_sash)
        open_fcx = (front_open_aabb[0][0] + front_open_aabb[1][0]) / 2.0
        ctx.check(
            "front sash slides left (-X) to open",
            open_fcx < rest_front_cx - 0.1,
            details=f"rest={rest_front_cx:.3f}, open={open_fcx:.3f}",
        )
        ctx.expect_overlap(
            front_sash, frame, axes="z", min_overlap=0.10,
            name="front sash retains vertical track engagement",
        )

    # --- Roller blocks exist on both sashes ---
    for pfx, sash_part in (("rear", rear_sash), ("front", front_sash)):
        for i in range(2):
            elem_name = f"{pfx}_roller_{i}"
            roller_aabb = ctx.part_element_world_aabb(sash_part, elem=elem_name)
            sash_aabb = ctx.part_world_aabb(sash_part)
            ctx.check(
                f"{pfx} roller {i} at sash bottom",
                roller_aabb[0][2] < sash_aabb[0][2] + 0.05,
                details=f"roller zmin={roller_aabb[0][2]:.3f}, sash zmin={sash_aabb[0][2]:.3f}",
            )

    return ctx.report()


object_model = build_object_model()
