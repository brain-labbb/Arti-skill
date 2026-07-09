from __future__ import annotations

# Two-panel horizontal sliding window variant: thick ALUMINUM frame with deep
# track grooves, rubber gasket strips around glass panes, and a small latch
# on a REVOLUTE joint at the meeting rail.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     - width  -> X
#     - height -> Z   (sill near z=0)
#     - frame depth / glazing thickness / slide-normal -> Y
#   Glass plane is the X-Z plane. q=0 reads SHUT. Driving the prismatic joint
#   slides the right sash sideways toward the fixed left sash (-X) to open.
#
# Structure:
#   - frame (root): thick aluminum head, sill, jambs with deep track grooves
#     cut into head and sill rails for sash guidance.
#   - fixed_sash (left, FIXED): aluminum sash ring + glass + rubber gasket
#   - sliding_sash (right, PRISMATIC): aluminum sash ring + glass + rubber gasket
#   - latch (REVOLUTE on sliding_sash): small cam latch at meeting rail

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
TOTAL_H = 1.72            # overall height along Z (sill at z=0, head at z=TOTAL_H)

FRAME_FACE = 0.095        # thick aluminum frame member face width
FRAME_DEPTH = 0.155       # deep aluminum box section along Y

# Track groove dimensions (deep channels in head and sill for sash guidance)
TRACK_GROOVE_W = 0.018    # groove width along Y (depth of channel)
TRACK_GROOVE_D = 0.020    # groove depth into the rail (along Z for sill/head)
# Two parallel grooves per rail (front track + rear track for the two sashes)
TRACK_SPACING = 0.048     # center-to-center Y spacing between front/rear tracks

MEETING_OVERLAP = 0.040   # the two sash stiles overlap by this much at center

SASH_FACE = 0.068         # sash perimeter rail/stile face width (aluminum)
SASH_DEPTH = 0.055        # sash depth along Y
GLASS_T = 0.006           # glazing thickness along Y

# Gasket strip dimensions (rubber seal around glass pane)
GASKET_W = 0.008          # gasket strip width (visible face)
GASKET_T = 0.004          # gasket strip thickness (stands off glass surface slightly)

# Y layout: frame box centered on y=0. Fixed sash in rear; sliding sash proud.
FIXED_SASH_Y = -0.030     # rear glazing plane center (Y)
SLIDE_SASH_Y = 0.042      # sliding sash proud toward +Y (front track)

REBATE = 0.005            # glass tucks under the sash lip by this much

# Latch (cam lock) hardware - separate part with revolute joint
LATCH_BODY_W = 0.024      # latch body width (X)
LATCH_BODY_H = 0.050      # latch body height (Z)
LATCH_BODY_T = 0.012      # latch body thickness (Y)
LATCH_LEVER_LEN = 0.040   # lever arm length
LATCH_LEVER_R = 0.005     # lever arm radius
LATCH_PIVOT_R = 0.006     # pivot shaft radius

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

FIXED_OPEN_CX = INNER_X0 + SASH_OPENING_W / 2.0
SLIDE_OPEN_CX = INNER_X1 - SASH_OPENING_W / 2.0
MID_CZ = (INNER_Z0 + INNER_Z1) / 2.0

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

ALUMINUM_RGBA = (0.72, 0.74, 0.76, 1.0)    # brushed aluminum frame/sash
GLASS_RGBA = (0.52, 0.60, 0.66, 0.30)       # cool grey-blue, semi-transparent
RUBBER_RGBA = (0.12, 0.12, 0.13, 1.0)       # dark rubber gasket
METAL_RGBA = (0.60, 0.62, 0.65, 1.0)        # darker metal latch hardware


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery), authored directly in meters.
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float, y_center: float, depth: float) -> cq.Workplane:
    """Axis-aligned box spanning [x0,x1] x [z0,z1] in X-Z, centered on y_center."""
    return (
        cq.Workplane("XY")
        .transformed(offset=((x0 + x1) / 2.0, y_center, (z0 + z1) / 2.0))
        .box(x1 - x0, depth, z1 - z0)
    )


def _build_frame_shape() -> cq.Workplane:
    """Static outer frame: thick aluminum slab cut by the sash opening,
    with deep track grooves cut into head and sill rails."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    cut_depth = FRAME_DEPTH + 0.02
    # Main opening
    opening = _slab(INNER_X0, INNER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    frame = outer.cut(opening)

    # Deep track grooves in the SILL (bottom rail, z near FRAME_FACE)
    # Two parallel grooves for front and rear sash tracks
    for y_off in (-TRACK_SPACING / 2.0, TRACK_SPACING / 2.0):
        groove = _slab(
            INNER_X0, INNER_X1,
            INNER_Z0 - TRACK_GROOVE_D, INNER_Z0,
            y_off, TRACK_GROOVE_W,
        )
        frame = frame.cut(groove)

    # Deep track grooves in the HEAD (top rail, z near TOTAL_H - FRAME_FACE)
    for y_off in (-TRACK_SPACING / 2.0, TRACK_SPACING / 2.0):
        groove = _slab(
            INNER_X0, INNER_X1,
            INNER_Z1, INNER_Z1 + TRACK_GROOVE_D,
            y_off, TRACK_GROOVE_W,
        )
        frame = frame.cut(groove)

    return frame


def _build_sash_shape() -> cq.Workplane:
    """One sash ring (aluminum) in its own local frame, centered on origin."""
    ow = SASH_OPENING_W
    oh = SASH_OPENING_H
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SASH_DEPTH)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH + 0.02)
    return outer.cut(opening)


def _build_sash_glass_shape() -> cq.Workplane:
    """Single clear pane filling the sash opening (sash-local frame)."""
    ow = SASH_OPENING_W + 2 * REBATE
    oh = SASH_OPENING_H + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_gasket_shape() -> cq.Workplane:
    """Rubber gasket frame around the glass pane (sash-local frame).
    A thin rectangular ring that sits at the glass edge, slightly proud."""
    glass_w = SASH_OPENING_W + 2 * REBATE
    glass_h = SASH_OPENING_H + 2 * REBATE
    outer_w = glass_w + 2 * GASKET_W
    outer_h = glass_h + 2 * GASKET_W
    # Build outer rectangle minus inner rectangle to get a ring
    outer = _slab(-outer_w / 2.0, outer_w / 2.0, -outer_h / 2.0, outer_h / 2.0, 0.0, GASKET_T)
    inner = _slab(-glass_w / 2.0, glass_w / 2.0, -glass_h / 2.0, glass_h / 2.0, 0.0, GASKET_T + 0.002)
    return outer.cut(inner)


def _build_latch_shape() -> cq.Workplane:
    """Latch: a single fused solid combining body block and lever arm.
    The lever extends along +Y from the body front face, sharing material
    to ensure one connected mesh."""
    body = cq.Workplane("XY").box(LATCH_BODY_W, LATCH_BODY_T, LATCH_BODY_H)
    # Lever: cylinder along Y, starting from body front face and extending outward.
    # Center the cylinder so it overlaps into the body slightly for a boolean union.
    lever_cy = LATCH_BODY_T / 2.0 + LATCH_LEVER_LEN / 2.0 - 0.003
    lever_len = LATCH_LEVER_LEN + 0.006  # extends 3mm into body for union
    lever = (
        cq.Workplane("XZ")
        .transformed(offset=(0.0, lever_cy, 0.0))
        .circle(LATCH_LEVER_R)
        .extrude(lever_len / 2.0, both=True)
    )
    return body.union(lever)


# ---------------------------------------------------------------------------
# Part builders
# ---------------------------------------------------------------------------

def _add_sash(model: ArticulatedObject, name: str) -> None:
    """Add a sash part (aluminum ring + glass + rubber gasket) in its own local frame."""
    sash = model.part(name)
    sash.visual(
        mesh_from_cadquery(_build_sash_shape(), f"{name}_frame"),
        material="aluminum",
        name=f"{name}_frame",
    )
    sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), f"{name}_glass"),
        material="glass",
        name=f"{name}_glass",
    )
    sash.visual(
        mesh_from_cadquery(_build_gasket_shape(), f"{name}_gasket"),
        material="rubber",
        name=f"{name}_gasket",
    )


def _add_latch(model: ArticulatedObject) -> None:
    """Add the latch as a separate part with a revolute joint on the sliding sash.
    The latch sits on the meeting (inner/left) stile of the sliding sash at mid-height.
    It rotates around the Z axis (vertical) to lock/unlock."""
    latch = model.part("latch")
    latch.visual(
        mesh_from_cadquery(_build_latch_shape(), "latch"),
        material="metal_dark",
        name="latch_mesh",
    )

    # Position latch on the sliding sash meeting stile
    # In sliding sash local frame: meeting stile center at x = -SASH_OPENING_W/2 - SASH_FACE/2
    stile_x = -SASH_OPENING_W / 2.0 - SASH_FACE / 2.0
    face_y = SASH_DEPTH / 2.0 + LATCH_BODY_T / 2.0  # proud of sash front face

    # The latch part origin is at the pivot point. We place it on the sash stile.
    # The articulation will then position it in the sash frame.
    model.articulation(
        "sash_to_latch",
        ArticulationType.REVOLUTE,
        parent="sliding_sash",
        child="latch",
        # Origin in sliding sash local frame: meeting stile, mid-height, front face
        origin=Origin(xyz=(stile_x, face_y, 0.0)),
        # Rotate around Z axis (vertical) - the latch thumb-turn
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=1.57),
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="aluminum_sliding_window")
    model.material("aluminum", rgba=ALUMINUM_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("rubber", rgba=RUBBER_RGBA)
    model.material("metal_dark", rgba=METAL_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame_shell"),
        material="aluminum",
        name="frame_shell",
    )

    # --- Fixed (left) + sliding (right) sashes ---
    _add_sash(model, "fixed_sash")
    _add_sash(model, "sliding_sash")

    # --- Latch (revolute on sliding sash) ---
    _add_latch(model)

    # FIXED left sash seated in the rear glazing plane.
    model.articulation(
        "frame_to_fixed_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="fixed_sash",
        origin=Origin(xyz=(FIXED_OPEN_CX, FIXED_SASH_Y, MID_CZ)),
    )

    # SLIDING right sash: PRISMATIC along X.
    slide_travel = SASH_OPENING_W * 0.90
    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="sliding_sash",
        origin=Origin(xyz=(SLIDE_OPEN_CX, SLIDE_SASH_Y, MID_CZ)),
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
    fixed_sash = object_model.get_part("fixed_sash")
    sliding_sash = object_model.get_part("sliding_sash")
    latch = object_model.get_part("latch")
    slide = object_model.get_articulation("frame_to_sliding_sash")
    latch_joint = object_model.get_articulation("sash_to_latch")

    # --- Intentional overlaps ---
    # Glass and gasket tuck under the sash frame lip (captured glazing).
    for nm in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_frame",
            reason="Glass pane is rebated under the aluminum sash lip (captured glazing).",
        )
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_gasket",
            elem_b=f"{nm}_frame",
            reason="Rubber gasket is seated against the sash frame inner lip (compression seal).",
        )
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_gasket",
            elem_b=f"{nm}_glass",
            reason="Gasket wraps the glass edge (contact seal).",
        )

    # Sash rings lap the frame opening edge (seated in tracks).
    for nm in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_frame",
            reason=f"{nm} ring is seated in the frame track groove (track capture).",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_glass",
            reason=f"{nm} glass is within the frame opening (rebated glazing).",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_gasket",
            reason=f"{nm} gasket sits within the frame opening perimeter (seal contact).",
        )

    # Latch is mounted on the sliding sash stile face.
    ctx.allow_overlap(
        "sliding_sash", "latch",
        elem_a="sliding_sash_frame",
        elem_b="latch_mesh",
        reason="Latch is surface-mounted on the sliding sash meeting stile.",
    )

    # --- Closed pose (q=0): window reads SHUT ---
    with ctx.pose({slide: 0.0, latch_joint: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        f_aabb = ctx.part_world_aabb(fixed_sash)
        s_aabb = ctx.part_world_aabb(sliding_sash)

        # Frame spans the full width and is wider than a single sash.
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        sash_w = s_aabb[1][0] - s_aabb[0][0]
        ctx.check(
            "frame spans wider than a single sash",
            frame_w > sash_w + 0.40,
            details=f"frame_w={frame_w:.3f}, sash_w={sash_w:.3f}",
        )
        # Sill near floor, head at full height.
        ctx.check(
            "sill sits near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"frame zmin={frame_aabb[0][2]:.4f}",
        )
        ctx.check(
            "head reaches full height",
            abs(frame_aabb[1][2] - TOTAL_H) < 0.02,
            details=f"frame zmax={frame_aabb[1][2]:.4f}",
        )
        # Two sashes side by side.
        fx = (f_aabb[0][0] + f_aabb[1][0]) / 2.0
        sx = (s_aabb[0][0] + s_aabb[1][0]) / 2.0
        ctx.check(
            "fixed sash left of sliding sash",
            fx < sx,
            details=f"fixed_x={fx:.3f}, sliding_x={sx:.3f}",
        )
        # Both sashes seated within frame height.
        for nm, ab in (("fixed", f_aabb), ("sliding", s_aabb)):
            ctx.check(
                f"{nm} sash seated within frame height",
                ab[0][2] > frame_aabb[0][2] - 1e-4 and ab[1][2] < frame_aabb[1][2] + 1e-4,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}]",
            )
        # Sliding sash sits proud of fixed sash.
        fy = (f_aabb[0][1] + f_aabb[1][1]) / 2.0
        sy = (s_aabb[0][1] + s_aabb[1][1]) / 2.0
        ctx.check(
            "sliding sash proud of fixed sash",
            sy > fy + 0.02,
            details=f"sliding_y={sy:.3f}, fixed_y={fy:.3f}",
        )
        # Both sashes seated in frame opening.
        ctx.expect_overlap(
            fixed_sash, frame, axes="xz", min_overlap=0.03,
            name="fixed sash seated in frame opening",
        )
        ctx.expect_overlap(
            sliding_sash, frame, axes="xz", min_overlap=0.03,
            name="sliding sash seated in frame opening",
        )

        # --- Prompt-specific: rubber gaskets present around glass ---
        fixed_gasket_aabb = ctx.part_element_world_aabb(fixed_sash, elem="fixed_sash_gasket")
        slide_gasket_aabb = ctx.part_element_world_aabb(sliding_sash, elem="sliding_sash_gasket")
        ctx.check(
            "fixed sash has rubber gasket around glass",
            fixed_gasket_aabb is not None,
            details="fixed_sash_gasket visual missing",
        )
        ctx.check(
            "sliding sash has rubber gasket around glass",
            slide_gasket_aabb is not None,
            details="sliding_sash_gasket visual missing",
        )
        # Gasket spans nearly the full glass height
        if fixed_gasket_aabb:
            gasket_h = fixed_gasket_aabb[1][2] - fixed_gasket_aabb[0][2]
            ctx.check(
                "fixed gasket spans most of sash height",
                gasket_h > SASH_OPENING_H * 0.9,
                details=f"gasket_h={gasket_h:.3f}, opening_h={SASH_OPENING_H:.3f}",
            )

        # --- Prompt-specific: latch at meeting rail ---
        latch_aabb = ctx.part_world_aabb(latch)
        latch_cx = (latch_aabb[0][0] + latch_aabb[1][0]) / 2.0
        latch_cz = (latch_aabb[0][2] + latch_aabb[1][2]) / 2.0
        ctx.check(
            "latch on sliding sash inner (meeting) stile",
            latch_cx < sx,
            details=f"latch_x={latch_cx:.3f}, sliding_center_x={sx:.3f}",
        )
        ctx.check(
            "latch near mid-height",
            abs(latch_cz - MID_CZ) < 0.20,
            details=f"latch_z={latch_cz:.3f}, mid_z={MID_CZ:.3f}",
        )

        rest_sx = sx
        rest_sz = (s_aabb[0][2] + s_aabb[1][2]) / 2.0

    # --- Driven/open pose: sliding sash slides toward fixed sash (-X) ---
    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel, latch_joint: 0.0}):
        s_open = ctx.part_world_aabb(sliding_sash)
        open_sx = (s_open[0][0] + s_open[1][0]) / 2.0
        ctx.check(
            "sliding sash opens toward fixed sash (-X)",
            abs((rest_sx - open_sx) - travel) < 0.02 and open_sx < rest_sx - 0.30,
            details=f"rest_x={rest_sx:.3f}, open_x={open_sx:.3f}, travel={travel:.3f}",
        )
        # Pure horizontal slide.
        open_sz = (s_open[0][2] + s_open[1][2]) / 2.0
        ctx.check(
            "slide is purely horizontal",
            abs(open_sz - rest_sz) < 0.02,
            details=f"open_z={open_sz:.3f}, rest_z={rest_sz:.3f}",
        )
        # Retained insertion.
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained within frame X span at full travel",
            s_open[0][0] > f_aabb[0][0] - 1e-4 and s_open[1][0] < f_aabb[1][0] + 1e-4,
            details=f"sash x=[{s_open[0][0]:.3f},{s_open[1][0]:.3f}] frame x=[{f_aabb[0][0]:.3f},{f_aabb[1][0]:.3f}]",
        )
        ctx.expect_overlap(
            sliding_sash, frame, axes="z", min_overlap=0.10,
            name="sash retains vertical engagement with head/sill track",
        )

    # --- Prompt-specific: latch revolute joint articulation test ---
    # Verify the latch rotates when driven (non-fixed joint).
    with ctx.pose({slide: 0.0, latch_joint: 0.0}):
        latch_rest = ctx.part_world_aabb(latch)
        rest_latch_w = latch_rest[1][0] - latch_rest[0][0]
        rest_latch_d = latch_rest[1][1] - latch_rest[0][1]

    with ctx.pose({slide: 0.0, latch_joint: 1.2}):
        latch_rot = ctx.part_world_aabb(latch)
        rot_latch_w = latch_rot[1][0] - latch_rot[0][0]
        rot_latch_d = latch_rot[1][1] - latch_rot[0][1]

    # When rotated ~70 degrees, the AABB extents should change significantly
    # (the long dimension swaps from X to Y or vice versa).
    ctx.check(
        "latch rotates on revolute joint (AABB changes)",
        abs(rot_latch_w - rest_latch_w) > 0.005 or abs(rot_latch_d - rest_latch_d) > 0.005,
        details=f"rest_w={rest_latch_w:.4f}, rot_w={rot_latch_w:.4f}, rest_d={rest_latch_d:.4f}, rot_d={rot_latch_d:.4f}",
    )

    # --- Prompt-specific: frame has deep track grooves (thick aluminum) ---
    # The frame should be thick (deep profile) - check frame depth
    frame_depth = frame_aabb[1][1] - frame_aabb[0][1]
    ctx.check(
        "frame has thick aluminum profile (deep section)",
        frame_depth > 0.12,
        details=f"frame_depth={frame_depth:.3f}",
    )
    # Frame face width should be thick
    frame_h = frame_aabb[1][2] - frame_aabb[0][2]
    ctx.check(
        "frame spans full window height",
        frame_h > 1.6,
        details=f"frame_h={frame_h:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
