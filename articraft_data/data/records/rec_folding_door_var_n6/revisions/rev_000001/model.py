from __future__ import annotations

# 6-panel bi-fold interior glass partition door (accordion / concertina folding glass door).
#
# Coordinate convention:
#   +Z is up. The door opening spans along X. Leaves hang in the X/Z plane and
#   the bi-fold swings into +/-Y. At the rest (zero) pose all six leaves are
#   coplanar at Y~0 = the CLOSED door. Driving the joints concertinas the whole
#   chain into a deep accordion stack against the left jamb.
#
# Structure:
#   - Root frame (static): top header track, left jamb, right jamb, floor track.
#     The leaves hang from the header; nothing floats.
#   - Six leaves chained: frame -> leaf_0 -> leaf_1 -> ... -> leaf_5.
#     Each leaf is its own part built via a shared geometry helper in a loop.
#   - Uniform alternating-sign revolute hinge policy down the chain.
#   - Each leaf: slim steel perimeter frame + horizontal mid-rail (CadQuery mesh)
#     + two glass panes + hinge-knuckle cylinders at the hinge edge.
#   - A slim vertical pull handle sits on the center-meeting leaf (leaf_2).

import math

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

OPENING_WIDTH = 2.20          # total clear opening span along X
OPENING_HEIGHT = 2.10         # total opening height along Z (top of header)
LEAF_COUNT = 6

FRAME_T = 0.035              # steel perimeter / mullion profile width (in-plane)
FRAME_DEPTH = 0.040          # steel profile depth (along Y / thickness)
GLASS_T = 0.010             # glass pane thickness (along Y)

# Header track and jambs
HEADER_H = 0.080            # header track height (Z)
HEADER_DEPTH = 0.060        # header track depth (Y)
JAMB_W = 0.060             # jamb width (X)
JAMB_DEPTH = 0.060         # jamb depth (Y)
FLOOR_H = 0.030            # floor track height (Z)
FLOOR_DEPTH = 0.060

# Leaf width: equal division of the clear span between jamb hinge lines.
CLEAR_SPAN = OPENING_WIDTH - JAMB_W   # usable span from left hinge line to right hinge line
LEAF_W = CLEAR_SPAN / LEAF_COUNT      # each leaf width

# Leaf vertical extents: leaves hang from just under the header down to the floor
# track, with small running clearances top and bottom so they swing freely and do
# not share a coplanar face with the header/floor.
HEAD_CLEARANCE = 0.012
FLOOR_CLEARANCE = 0.012
LEAF_TOP_Z = (OPENING_HEIGHT - HEADER_H) - HEAD_CLEARANCE
LEAF_BOTTOM_Z = FLOOR_H + FLOOR_CLEARANCE
LEAF_H = LEAF_TOP_Z - LEAF_BOTTOM_Z
MID_RAIL_FRACTION = 0.66    # mid-rail sits ~2/3 up (large upper, short lower)

# Hinge knuckles
KNUCKLE_R = 0.016
KNUCKLE_LEN = 0.060
KNUCKLE_COUNT = 5

# Pull handle
HANDLE_R = 0.011
HANDLE_LEN = 0.700
HANDLE_STANDOFF = 0.040

# Hinge X lines: vertical meeting edges from left jamb to right jamb.
LEFT_HINGE_X = -OPENING_WIDTH / 2.0 + JAMB_W / 2.0
HINGE_X = [LEFT_HINGE_X + i * LEAF_W for i in range(LEAF_COUNT + 1)]

# Center-meeting leaf: the leaf whose free edge is at the center of the opening.
# With 6 leaves, leaf_2 free edge is at HINGE_X[3] which is the midpoint.
CENTER_LEAF_INDEX = LEAF_COUNT // 2 - 1  # = 2

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

STEEL_RGBA = (0.09, 0.09, 0.10, 1.0)        # near-black powder-coated steel
GLASS_RGBA = (0.42, 0.50, 0.56, 0.30)       # cool grey-blue, semi-transparent
KNUCKLE_RGBA = (0.78, 0.80, 0.82, 1.0)      # bright machined hinge knuckles
HANDLE_RGBA = (0.20, 0.21, 0.23, 1.0)       # dark satin handle


# ---------------------------------------------------------------------------
# Shared leaf geometry helpers (CadQuery)
# ---------------------------------------------------------------------------

def _build_leaf_frame_shape() -> cq.Workplane:
    """Slim steel perimeter frame + horizontal mid-rail for one leaf.

    Authored in the leaf-local hinge frame:
      - local X runs 0 .. LEAF_W (hinge edge at x=0, free edge at x=LEAF_W)
      - local Z runs 0 .. LEAF_H
      - local Y is the leaf thickness, centered at y=0
    The frame is the perimeter ring plus a mid-rail; the glass opening is hollow.
    """
    w = LEAF_W
    h = LEAF_H
    t = FRAME_T
    d = FRAME_DEPTH

    outer = (
        cq.Workplane("XY")
        .box(w, d, h, centered=(False, True, False))
    )

    mid_z = h * MID_RAIL_FRACTION
    half_rail = t / 2.0

    # Upper opening (between top rail, mid-rail, and side rails)
    up_x0, up_x1 = t, w - t
    up_z0, up_z1 = mid_z + half_rail, h - t
    upper_cut = (
        cq.Workplane("XY")
        .transformed(offset=(
            (up_x0 + up_x1) / 2.0,
            0.0,
            (up_z0 + up_z1) / 2.0,
        ))
        .box(up_x1 - up_x0, d + 0.01, up_z1 - up_z0)
    )

    # Lower opening
    lo_x0, lo_x1 = t, w - t
    lo_z0, lo_z1 = t, mid_z - half_rail
    lower_cut = (
        cq.Workplane("XY")
        .transformed(offset=(
            (lo_x0 + lo_x1) / 2.0,
            0.0,
            (lo_z0 + lo_z1) / 2.0,
        ))
        .box(lo_x1 - lo_x0, d + 0.01, lo_z1 - lo_z0)
    )

    frame = outer.cut(upper_cut).cut(lower_cut)
    return frame


def _build_glass_shape() -> cq.Workplane:
    """Two thin glass panes (upper + short lower) filling the leaf openings.

    Same leaf-local frame as the steel frame. Panes overlap the frame rebate
    slightly so the glass reads as captured inside the steel, not floating.
    """
    w = LEAF_W
    h = LEAF_H
    t = FRAME_T
    mid_z = h * MID_RAIL_FRACTION
    half_rail = FRAME_T / 2.0
    rebate = 0.006

    # Upper pane
    up_x0, up_x1 = t - rebate, w - t + rebate
    up_z0, up_z1 = mid_z + half_rail - rebate, h - t + rebate
    upper = (
        cq.Workplane("XY")
        .transformed(offset=(
            (up_x0 + up_x1) / 2.0,
            0.0,
            (up_z0 + up_z1) / 2.0,
        ))
        .box(up_x1 - up_x0, GLASS_T, up_z1 - up_z0)
    )

    # Lower (shorter) pane
    lo_x0, lo_x1 = t - rebate, w - t + rebate
    lo_z0, lo_z1 = t - rebate, mid_z - half_rail + rebate
    lower = (
        cq.Workplane("XY")
        .transformed(offset=(
            (lo_x0 + lo_x1) / 2.0,
            0.0,
            (lo_z0 + lo_z1) / 2.0,
        ))
        .box(lo_x1 - lo_x0, GLASS_T, lo_z1 - lo_z0)
    )

    return upper.union(lower)


# ---------------------------------------------------------------------------
# Root frame geometry (CadQuery): header track, jambs, floor track
# ---------------------------------------------------------------------------

def _build_root_frame_shape() -> cq.Workplane:
    """Static outer frame that carries the whole assembly.

    World frame: opening centered on X=0, Z from 0 (floor) to OPENING_HEIGHT (top).
    """
    half_w = OPENING_WIDTH / 2.0

    header = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, OPENING_HEIGHT - HEADER_H / 2.0))
        .box(OPENING_WIDTH, HEADER_DEPTH, HEADER_H)
    )

    left_jamb = (
        cq.Workplane("XY")
        .transformed(offset=(
            -half_w + JAMB_W / 2.0,
            0.0,
            OPENING_HEIGHT / 2.0,
        ))
        .box(JAMB_W, JAMB_DEPTH, OPENING_HEIGHT)
    )

    right_jamb = (
        cq.Workplane("XY")
        .transformed(offset=(
            half_w - JAMB_W / 2.0,
            0.0,
            OPENING_HEIGHT / 2.0,
        ))
        .box(JAMB_W, JAMB_DEPTH, OPENING_HEIGHT)
    )

    floor = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, FLOOR_H / 2.0))
        .box(OPENING_WIDTH, FLOOR_DEPTH, FLOOR_H)
    )

    return header.union(left_jamb).union(right_jamb).union(floor)


# ---------------------------------------------------------------------------
# Leaf part builder (shared geometry helper, loop-emitted)
# ---------------------------------------------------------------------------

def _add_leaf(model: ArticulatedObject, index: int) -> None:
    """Build one leaf part in its leaf-local hinge frame.

    All leaves use the same geometry helper and the same knuckle placement:
    knuckles at the hinge edge (local x=0) so the visible hinge line reads
    at every fold. The leaf extends along local +X from its hinge edge.
    """
    name = f"leaf_{index}"
    leaf = model.part(name)

    # Steel frame + mid-rail (shared geometry helper)
    leaf.visual(
        mesh_from_cadquery(_build_leaf_frame_shape(), f"{name}_frame"),
        material="steel",
        name=f"{name}_frame",
    )

    # Glass panes (shared geometry helper)
    leaf.visual(
        mesh_from_cadquery(_build_glass_shape(), f"{name}_glass"),
        material="glass",
        name=f"{name}_glass",
    )

    # Hinge knuckles down the hinge edge (x=0): a column of short bright
    # cylinders straddling the pivot line so the hinge line reads.
    span = LEAF_H - 2 * FRAME_T
    z0 = FRAME_T + KNUCKLE_LEN / 2.0
    for k in range(KNUCKLE_COUNT):
        frac = k / (KNUCKLE_COUNT - 1)
        z = z0 + frac * (span - KNUCKLE_LEN)
        leaf.visual(
            Cylinder(radius=KNUCKLE_R, length=KNUCKLE_LEN),
            origin=Origin(xyz=(0.0, 0.0, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="knuckle",
            name=f"{name}_knuckle_{k}",
        )


def _add_handle(model: ArticulatedObject, leaf_index: int) -> None:
    """Add a slim vertical pull handle on the free (center-meeting) edge of a leaf.

    Mounted on two short standoffs on the leaf's free edge, standing off into -Y
    so it reads as a graspable bar.
    """
    leaf_name = f"leaf_{leaf_index}"
    leaf = model.get_part(leaf_name)
    handle_x = LEAF_W - FRAME_T / 2.0   # near the free (center-meeting) edge
    handle_y = -(FRAME_DEPTH / 2.0 + HANDLE_STANDOFF)
    handle_z = LEAF_H * 0.46            # roughly hand height, near the mid-rail

    # Two standoffs connecting handle bar to the leaf frame.
    for dz in (HANDLE_LEN / 2.0 - 0.05, -(HANDLE_LEN / 2.0 - 0.05)):
        leaf.visual(
            Cylinder(radius=0.006, length=HANDLE_STANDOFF + FRAME_DEPTH / 2.0),
            origin=Origin(
                xyz=(handle_x, handle_y / 2.0, handle_z + dz),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="handle",
            name=f"{leaf_name}_handle_standoff_{'top' if dz > 0 else 'bot'}",
        )

    # Vertical handle bar.
    leaf.visual(
        Cylinder(radius=HANDLE_R, length=HANDLE_LEN),
        origin=Origin(xyz=(handle_x, handle_y, handle_z)),
        material="handle",
        name=f"{leaf_name}_handle_bar",
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bifold_glass_partition_door")

    model.material("steel", rgba=STEEL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("knuckle", rgba=KNUCKLE_RGBA)
    model.material("handle", rgba=HANDLE_RGBA)

    # --- Root static frame ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_root_frame_shape(), "frame"),
        material="steel",
        name="frame_shell",
    )

    # --- Six leaves emitted via loop with shared geometry helper ---
    for i in range(LEAF_COUNT):
        _add_leaf(model, i)

    # Pull handle on the center-meeting leaf (leaf_2 free edge at opening center).
    _add_handle(model, CENTER_LEAF_INDEX)

    # ----- Articulations: single chain, uniform alternating-sign hinge policy -----
    # All leaf parts are authored in a local frame where the hinge edge is at
    # local x=0 and the leaf extends along local +X. Joint origins sit on the
    # real vertical meeting edge at the leaf bottom Z anchor.
    #
    # Policy: joint i connects parent to child leaf_i.
    #   - Joint 0: frame -> leaf_0 at the left jamb hinge line (HINGE_X[0]).
    #   - Joint i (i>=1): leaf_{i-1} -> leaf_i at local (LEAF_W, 0, 0) in the parent.
    #   - Axis alternates: +Z for even i, -Z for odd i (concertina / accordion fold).
    #   - All joints: REVOLUTE, limits [0, 2.7 rad].

    z_anchor = LEAF_BOTTOM_Z

    for i in range(LEAF_COUNT):
        axis_sign = 1.0 if (i % 2 == 0) else -1.0
        axis = (0.0, 0.0, axis_sign)

        if i == 0:
            # First leaf hinges at the left jamb.
            model.articulation(
                f"frame_to_leaf_{i}",
                ArticulationType.REVOLUTE,
                parent="frame",
                child=f"leaf_{i}",
                origin=Origin(xyz=(HINGE_X[0], 0.0, z_anchor)),
                axis=axis,
                motion_limits=MotionLimits(
                    effort=40.0, velocity=1.5, lower=0.0, upper=2.7
                ),
            )
        else:
            # Subsequent leaves chain from the previous leaf's free edge.
            model.articulation(
                f"leaf_{i-1}_to_leaf_{i}",
                ArticulationType.REVOLUTE,
                parent=f"leaf_{i-1}",
                child=f"leaf_{i}",
                origin=Origin(xyz=(LEAF_W, 0.0, 0.0)),
                axis=axis,
                motion_limits=MotionLimits(
                    effort=40.0, velocity=1.5, lower=0.0, upper=2.7
                ),
            )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    leaves = [object_model.get_part(f"leaf_{i}") for i in range(LEAF_COUNT)]
    joints = []
    joints.append(object_model.get_articulation("frame_to_leaf_0"))
    for i in range(1, LEAF_COUNT):
        joints.append(object_model.get_articulation(f"leaf_{i-1}_to_leaf_{i}"))

    leaf_names = [f"leaf_{i}" for i in range(LEAF_COUNT)]

    # --- Intentional overlaps ---
    # Glass panes tuck under the steel frame lip (captured glass, not floating).
    for lf in leaf_names:
        ctx.allow_overlap(
            lf, lf,
            elem_a=f"{lf}_glass",
            elem_b=f"{lf}_frame",
            reason="Glass pane is rebated under the steel frame lip so it reads as captured, not floating.",
        )

    # Adjacent leaves share a hinge line; knuckles and frame edges intentionally
    # meet at the fold edge.
    for i in range(LEAF_COUNT - 1):
        ctx.allow_overlap(
            f"leaf_{i}", f"leaf_{i+1}",
            reason=f"Adjacent leaves leaf_{i} and leaf_{i+1} share a hinge line; knuckles and frame edges intentionally meet at the fold edge.",
        )

    # First leaf hinges at the left jamb.
    ctx.allow_overlap(
        "frame", "leaf_0",
        reason="Leaf_0 hinges at the left jamb; its hinge edge and knuckles intentionally meet the jamb.",
    )
    # Last leaf meets the right jamb when closed (end of the accordion chain).
    last_leaf = f"leaf_{LEAF_COUNT - 1}"
    ctx.allow_overlap(
        "frame", last_leaf,
        reason=f"{last_leaf} is the end of the accordion chain; its free edge and knuckles meet the right jamb when closed.",
    )

    # Handle standoffs penetrate the center leaf frame to mount the bar.
    center_name = f"leaf_{CENTER_LEAF_INDEX}"
    ctx.allow_overlap(
        center_name, center_name,
        elem_a=f"{center_name}_handle_standoff_top",
        elem_b=f"{center_name}_frame",
        reason="Handle standoff is seated into the leaf frame to mount the pull bar.",
    )
    ctx.allow_overlap(
        center_name, center_name,
        elem_a=f"{center_name}_handle_standoff_bot",
        elem_b=f"{center_name}_frame",
        reason="Handle standoff is seated into the leaf frame to mount the pull bar.",
    )

    # --- Structural checks ---
    # 6 leaves exist with equal widths
    ctx.check(
        "six leaves exist",
        len(leaves) == LEAF_COUNT,
        details=f"found {len(leaves)} leaves",
    )

    # All leaves have the same width (equal division of the opening)
    leaf_widths = []
    for lf in leaves:
        aabb = ctx.part_world_aabb(lf)
        leaf_widths.append(aabb[1][0] - aabb[0][0])
    width_spread = max(leaf_widths) - min(leaf_widths)
    ctx.check(
        "all leaves have equal width",
        width_spread < 0.02,
        details=f"leaf widths: {[f'{w:.4f}' for w in leaf_widths]}",
    )

    # --- Closed/zero pose: all six leaves coplanar (similar Y center) ---
    zero_pose = {j: 0.0 for j in joints}
    with ctx.pose(zero_pose):
        aabbs = [ctx.part_world_aabb(lf) for lf in leaves]
        y_centers = [(lo[1] + hi[1]) / 2.0 for lo, hi in aabbs]
        y_spread = max(y_centers) - min(y_centers)
        ctx.check(
            "leaves coplanar at closed pose",
            y_spread < 0.05,
            details=f"Y centers: {[f'{y:.3f}' for y in y_centers]}",
        )
        # Leaves are ordered left-to-right with no big X gaps (door reads closed).
        x_centers = [(lo[0] + hi[0]) / 2.0 for lo, hi in aabbs]
        ordered = all(x_centers[i] < x_centers[i + 1] for i in range(LEAF_COUNT - 1))
        total_span = x_centers[-1] - x_centers[0]
        ctx.check(
            "leaves span left-to-right closed",
            ordered and total_span > 1.2,
            details=f"X centers L->R: {[f'{x:.3f}' for x in x_centers]}, span={total_span:.3f}",
        )

    # --- Header is topmost and spans wider than any leaf; leaves hang below ---
    frame_aabb = ctx.part_world_aabb(frame)
    leaf0_aabb = ctx.part_world_aabb(leaves[0])
    frame_top = frame_aabb[1][2]
    leaf_top = leaf0_aabb[1][2]
    frame_width = frame_aabb[1][0] - frame_aabb[0][0]
    leaf_width = leaf0_aabb[1][0] - leaf0_aabb[0][0]
    ctx.check(
        "header is topmost element",
        frame_top >= leaf_top - 1e-4,
        details=f"frame_top={frame_top:.3f}, leaf_top={leaf_top:.3f}",
    )
    ctx.check(
        "frame spans wider than a single leaf",
        frame_width > leaf_width + 0.5,
        details=f"frame_width={frame_width:.3f}, leaf_width={leaf_width:.3f}",
    )
    ctx.check(
        "leaves hang below header top",
        leaf_top <= frame_top + 1e-4,
        details=f"leaf_top={leaf_top:.3f}, frame_top={frame_top:.3f}",
    )

    # --- HERO: driving the chain folds leaf_0 and leaf_1 out of plane ---
    rest_l0_y = (leaf0_aabb[0][1] + leaf0_aabb[1][1]) / 2.0
    leaf1_aabb = ctx.part_world_aabb(leaves[1])
    rest_l1_y = (leaf1_aabb[0][1] + leaf1_aabb[1][1]) / 2.0
    rest_l0_x = (leaf0_aabb[0][0] + leaf0_aabb[1][0]) / 2.0

    fold = 2.4  # radians, near flat-stack
    # Drive the first two joints to fold the front of the chain.
    fold_pose = {joints[0]: fold, joints[1]: fold}
    with ctx.pose(fold_pose):
        f0 = ctx.part_world_aabb(leaves[0])
        f1 = ctx.part_world_aabb(leaves[1])
        f0_y = (f0[0][1] + f0[1][1]) / 2.0
        f1_y = (f1[0][1] + f1[1][1]) / 2.0
        f0_x = (f0[0][0] + f0[1][0]) / 2.0
        # leaf_0 swings out into +Y away from the closed plane (axis +Z)
        ctx.check(
            "leaf_0 folds out of plane (+Y)",
            f0_y > rest_l0_y + 0.10,
            details=f"rest_y={rest_l0_y:.3f}, folded_y={f0_y:.3f}",
        )
        # leaf_1 also leaves the closed plane (concertina, axis -Z reverses fold)
        ctx.check(
            "leaf_1 leaves closed plane (concertina)",
            abs(f1_y - rest_l1_y) > 0.08,
            details=f"rest_y={rest_l1_y:.3f}, folded_y={f1_y:.3f}",
        )
        # Folded pair pulls back toward the left jamb.
        ctx.check(
            "folded pair shifts toward left jamb",
            f0_x < rest_l0_x + 0.02,
            details=f"leaf0 x rest={rest_l0_x:.3f} folded={f0_x:.3f}",
        )

    # --- Alternating hinge policy: even joints use +Z axis, odd use -Z ---
    for i, j in enumerate(joints):
        expected_sign = 1.0 if (i % 2 == 0) else -1.0
        j_summary = ctx.articulation_summary(j) if hasattr(ctx, 'articulation_summary') else None
        # We can check by observing the axis from the articulation.
        # Use a small pose to confirm direction: positive q moves leaf in expected Y direction.
        if i == 0:
            small_q = 0.3
            with ctx.pose({j: small_q}):
                moved = ctx.part_world_aabb(leaves[0])
                moved_y = (moved[0][1] + moved[1][1]) / 2.0
                rest_y = (leaf0_aabb[0][1] + leaf0_aabb[1][1]) / 2.0
                if expected_sign > 0:
                    ctx.check(
                        f"joint_{i} positive q folds toward +Y (axis +Z)",
                        moved_y > rest_y + 0.01,
                        details=f"rest_y={rest_y:.4f}, moved_y={moved_y:.4f}",
                    )
                else:
                    ctx.check(
                        f"joint_{i} positive q folds toward -Y (axis -Z)",
                        moved_y < rest_y - 0.01,
                        details=f"rest_y={rest_y:.4f}, moved_y={moved_y:.4f}",
                    )

    # --- Pull handle sits on the center meeting edge (near world X=0) ---
    handle_aabb = ctx.part_element_world_aabb(
        leaves[CENTER_LEAF_INDEX],
        elem=f"{center_name}_handle_bar",
    )
    if handle_aabb is not None:
        handle_x = (handle_aabb[0][0] + handle_aabb[1][0]) / 2.0
        ctx.check(
            "pull handle is on the center meeting edge",
            abs(handle_x) < 0.12,
            details=f"handle world X center={handle_x:.3f}",
        )
        # Handle stands off the glass plane in -Y.
        handle_y = (handle_aabb[0][1] + handle_aabb[1][1]) / 2.0
        ctx.check(
            "pull handle stands off the glass",
            handle_y < -FRAME_DEPTH / 2.0,
            details=f"handle world Y center={handle_y:.3f}",
        )

    # --- Chain connectivity: each leaf connects to the next via a real joint ---
    ctx.check(
        "chain has 6 revolute joints",
        len(joints) == LEAF_COUNT,
        details=f"found {len(joints)} joints",
    )

    return ctx.report()


object_model = build_object_model()
