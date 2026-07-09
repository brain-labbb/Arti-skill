from __future__ import annotations

# 8-panel accordion interior glass partition door (continuous bi-fold chain).
#
# Coordinate convention:
#   +Z is up. The door opening spans along X. Leaves hang in the X/Z plane and
#   the accordion swings into +/-Y. At the rest (zero) pose all eight leaves are
#   coplanar at Y~0 = the CLOSED door. Driving the joints concertinas the chain
#   into a compact zig-zag stack.
#
# Structure:
#   - Root frame (static): top header track, left jamb, right jamb, floor track.
#   - Eight leaves in one continuous chain: frame -> leaf_0 -> leaf_1 -> ... -> leaf_7.
#   - Each leaf is its own part: a slim steel perimeter frame + horizontal mid-rail
#     (one CadQuery mesh) + a thin semi-transparent glass pane + hinge-knuckle
#     cylinders down the free vertical edge.
#   - A slim vertical pull handle sits on the center leaf (leaf_3) near X=0.
#   - All joints are revolute with alternating-sign axes for accordion folding.

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
LEAF_COUNT = 8                # 8 narrow hinged leaves in one accordion chain

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

# Hinge span: leaves fill from left jamb center to right jamb center.
LEFT_HINGE_X = -OPENING_WIDTH / 2.0 + JAMB_W / 2.0   # -1.07
HINGE_SPAN = 2.0 * abs(LEFT_HINGE_X)                  # 2.14
LEAF_W = HINGE_SPAN / LEAF_COUNT                      # 0.2675

# Leaf vertical extents: leaves hang from just under the header down to the floor
# track, with small running clearances top and bottom.
HEAD_CLEARANCE = 0.012
FLOOR_CLEARANCE = 0.012
LEAF_TOP_Z = (OPENING_HEIGHT - HEADER_H) - HEAD_CLEARANCE     # 2.008
LEAF_BOTTOM_Z = FLOOR_H + FLOOR_CLEARANCE                     # 0.042
LEAF_H = LEAF_TOP_Z - LEAF_BOTTOM_Z           # leaf overall height
MID_RAIL_FRACTION = 0.66                       # mid-rail sits ~2/3 up

# Hinge knuckles
KNUCKLE_R = 0.016
KNUCKLE_LEN = 0.060
KNUCKLE_COUNT = 5

# Pull handle
HANDLE_R = 0.011
HANDLE_LEN = 0.700
HANDLE_STANDOFF = 0.040

# Hinge X lines (vertical meeting edges) measured from left jamb.
HINGE_X = [LEFT_HINGE_X + i * LEAF_W for i in range(LEAF_COUNT + 1)]

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

STEEL_RGBA = (0.09, 0.09, 0.10, 1.0)        # near-black powder-coated steel
GLASS_RGBA = (0.42, 0.50, 0.56, 0.30)       # cool grey-blue, semi-transparent
KNUCKLE_RGBA = (0.78, 0.80, 0.82, 1.0)      # bright machined hinge knuckles
HANDLE_RGBA = (0.20, 0.21, 0.23, 1.0)       # dark satin handle


# ---------------------------------------------------------------------------
# Shared geometry helpers (CadQuery)
# ---------------------------------------------------------------------------

def _build_leaf_frame_shape() -> cq.Workplane:
    """Slim steel perimeter frame + horizontal mid-rail for one leaf.

    Authored in leaf-local hinge frame:
      - local X runs 0 .. LEAF_W (hinge edge at x=0, free edge at x=LEAF_W)
      - local Z runs 0 .. LEAF_H
      - local Y is the leaf thickness, centered at y=0
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

    # Upper glass opening
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

    # Lower glass opening
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

    return outer.cut(upper_cut).cut(lower_cut)


def _build_glass_shape() -> cq.Workplane:
    """Two thin glass panes (upper + lower) filling the leaf frame openings."""
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

    # Lower pane
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


def _build_root_frame_shape() -> cq.Workplane:
    """Static outer frame: header track, left jamb, right jamb, floor track."""
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
# Leaf part builder (shared helper for all 8 leaves)
# ---------------------------------------------------------------------------

def _add_leaf(model: ArticulatedObject, name: str) -> None:
    """Build one leaf part in its leaf-local hinge frame.

    The leaf's hinge edge is at local x=0; the leaf extends to local x=LEAF_W.
    Knuckles run down the free vertical edge (x=LEAF_W) — the meeting edge
    with the next leaf (or the right jamb for the last leaf).
    """
    leaf = model.part(name)

    # Steel frame + mid-rail
    leaf.visual(
        mesh_from_cadquery(_build_leaf_frame_shape(), f"{name}_frame"),
        material="steel",
        name=f"{name}_frame",
    )

    # Glass panes
    leaf.visual(
        mesh_from_cadquery(_build_glass_shape(), f"{name}_glass"),
        material="glass",
        name=f"{name}_glass",
    )

    # Hinge knuckles down the free edge
    span = LEAF_H - 2 * FRAME_T
    z0 = FRAME_T + KNUCKLE_LEN / 2.0
    for k in range(KNUCKLE_COUNT):
        frac = k / (KNUCKLE_COUNT - 1)
        z = z0 + frac * (span - KNUCKLE_LEN)
        leaf.visual(
            Cylinder(radius=KNUCKLE_R, length=KNUCKLE_LEN),
            origin=Origin(xyz=(LEAF_W, 0.0, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="knuckle",
            name=f"{name}_knuckle_{k}",
        )


def _add_handle(model: ArticulatedObject, leaf_name: str) -> None:
    """Add a slim vertical pull handle on a leaf's free (meeting) edge."""
    leaf = model.get_part(leaf_name)
    handle_x = LEAF_W - FRAME_T / 2.0
    handle_y = -(FRAME_DEPTH / 2.0 + HANDLE_STANDOFF)
    handle_z = LEAF_H * 0.46

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
    model = ArticulatedObject(name="accordion_glass_partition_door")

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

    # --- Eight leaves in one continuous accordion chain ---
    for i in range(LEAF_COUNT):
        _add_leaf(model, f"leaf_{i}")

    # Center pull handle on the leaf whose free edge is nearest X=0 (leaf_3).
    # HINGE_X[4] = 0.0 is the free edge of leaf_3.
    _add_handle(model, "leaf_3")

    # ----- Articulations: one continuous chain with alternating-sign axes -----
    # All leaf parts are authored in a local frame where the hinge edge is at
    # local x=0 and the leaf extends along local +X. The joint origin is placed
    # at the real vertical hinge line, at the leaf bottom Z anchor.

    z_anchor = LEAF_BOTTOM_Z

    # Joint 0: frame -> leaf_0 (hinge at left jamb, HINGE_X[0])
    # Even-index joints use +Z axis; odd-index joints use -Z axis.
    # This alternating sign produces the accordion zig-zag when all q > 0.
    for i in range(LEAF_COUNT):
        child_name = f"leaf_{i}"
        axis_sign = 1 if i % 2 == 0 else -1

        if i == 0:
            # Frame -> first leaf: world-space origin at left jamb hinge line
            model.articulation(
                "frame_to_leaf_0",
                ArticulationType.REVOLUTE,
                parent="frame",
                child=child_name,
                origin=Origin(xyz=(HINGE_X[0], 0.0, z_anchor)),
                axis=(0.0, 0.0, float(axis_sign)),
                motion_limits=MotionLimits(
                    effort=40.0, velocity=1.5, lower=0.0, upper=2.5,
                ),
            )
        else:
            # leaf_{i-1} -> leaf_i: origin at parent's free edge (local x=LEAF_W)
            parent_name = f"leaf_{i - 1}"
            model.articulation(
                f"{parent_name}_to_{child_name}",
                ArticulationType.REVOLUTE,
                parent=parent_name,
                child=child_name,
                origin=Origin(xyz=(LEAF_W, 0.0, 0.0)),
                axis=(0.0, 0.0, float(axis_sign)),
                motion_limits=MotionLimits(
                    effort=40.0, velocity=1.5, lower=0.0, upper=2.5,
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
    leaf_names = [f"leaf_{i}" for i in range(LEAF_COUNT)]

    # Collect all articulations
    joints = []
    joints.append(object_model.get_articulation("frame_to_leaf_0"))
    for i in range(1, LEAF_COUNT):
        joints.append(
            object_model.get_articulation(f"leaf_{i - 1}_to_leaf_{i}")
        )

    # --- Verify structure: 8 leaves and 8 joints ---
    ctx.check(
        "eight leaves exist",
        len(leaves) == LEAF_COUNT,
        details=f"found {len(leaves)} leaves",
    )
    ctx.check(
        "eight articulations exist",
        len(joints) == LEAF_COUNT,
        details=f"found {len(joints)} joints",
    )

    # --- Verify alternating axis signs ---
    axis_signs = []
    for j in joints:
        ax = j.axis
        axis_signs.append(1 if ax[2] > 0 else -1)
    expected_signs = [1 if i % 2 == 0 else -1 for i in range(LEAF_COUNT)]
    ctx.check(
        "alternating hinge axis signs",
        axis_signs == expected_signs,
        details=f"got {axis_signs}, expected {expected_signs}",
    )

    # --- Intentional overlaps ---
    # Glass panes are rebated under the steel frame lip.
    for lf in leaf_names:
        ctx.allow_overlap(
            lf, lf,
            elem_a=f"{lf}_glass",
            elem_b=f"{lf}_frame",
            reason="Glass pane is rebated under the steel frame lip so it reads as captured, not floating.",
        )

    # Adjacent leaves share a hinge line; knuckles and frame edges intentionally meet.
    for i in range(LEAF_COUNT - 1):
        ctx.allow_overlap(
            leaf_names[i], leaf_names[i + 1],
            reason=f"Adjacent leaves leaf_{i} and leaf_{i+1} share a hinge line; knuckles and frame edges intentionally meet at the fold edge.",
        )

    # First leaf hinges at the left jamb.
    ctx.allow_overlap(
        "frame", "leaf_0",
        reason="Leaf_0 hinges at the left jamb; its hinge edge and knuckles intentionally meet the jamb.",
    )

    # Last leaf's free edge knuckles meet the right jamb.
    ctx.allow_overlap(
        "frame", f"leaf_{LEAF_COUNT - 1}",
        reason=f"Leaf_{LEAF_COUNT-1} free-edge knuckles meet the right jamb at the terminal hinge.",
    )

    # Handle standoffs penetrate the leaf_3 frame to mount the bar.
    ctx.allow_overlap(
        "leaf_3", "leaf_3",
        elem_a="leaf_3_handle_standoff_top",
        elem_b="leaf_3_frame",
        reason="Handle standoff is seated into the leaf frame to mount the pull bar.",
    )
    ctx.allow_overlap(
        "leaf_3", "leaf_3",
        elem_a="leaf_3_handle_standoff_bot",
        elem_b="leaf_3_frame",
        reason="Handle standoff is seated into the leaf frame to mount the pull bar.",
    )

    # --- Closed/zero pose: all eight leaves coplanar ---
    zero_pose = {j: 0.0 for j in joints}
    with ctx.pose(zero_pose):
        aabbs = [ctx.part_world_aabb(lf) for lf in leaves]
        y_centers = [(lo[1] + hi[1]) / 2.0 for lo, hi in aabbs]
        ctx.check(
            "leaves coplanar at closed pose",
            max(y_centers) - min(y_centers) < 0.05,
            details=f"Y centers: {[f'{y:.3f}' for y in y_centers]}",
        )
        # Leaves are ordered left-to-right with no big X gaps.
        x_centers = [(lo[0] + hi[0]) / 2.0 for lo, hi in aabbs]
        ordered = all(x_centers[i] < x_centers[i + 1] for i in range(LEAF_COUNT - 1))
        total_span = x_centers[-1] - x_centers[0]
        ctx.check(
            "leaves span left-to-right closed",
            ordered and total_span > 1.4,
            details=f"ordered={ordered}, span={total_span:.3f}",
        )

    # --- Header is topmost and spans wider than a single leaf ---
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

    # --- HERO: driving all joints folds the accordion chain ---
    # At rest, record leaf positions.
    rest_x_centers = [(lo[0] + hi[0]) / 2.0 for lo, hi in aabbs]
    rest_y_centers = y_centers[:]
    rest_last_x = rest_x_centers[-1]

    fold = 1.8  # radians, moderate fold
    fold_pose = {j: fold for j in joints}
    with ctx.pose(fold_pose):
        folded_aabbs = [ctx.part_world_aabb(lf) for lf in leaves]
        folded_y = [(lo[1] + hi[1]) / 2.0 for lo, hi in folded_aabbs]
        folded_x = [(lo[0] + hi[0]) / 2.0 for lo, hi in folded_aabbs]

        # Leaves zig-zag out of the closed plane (no longer coplanar).
        folded_y_spread = max(folded_y) - min(folded_y)
        ctx.check(
            "accordion: leaves zig-zag out of closed plane",
            folded_y_spread > 0.20,
            details=f"folded Y spread={folded_y_spread:.3f}, values={[f'{y:.3f}' for y in folded_y]}",
        )

        # The chain compresses: the last leaf moves leftward (toward first leaf).
        folded_last_x = folded_x[-1]
        ctx.check(
            "accordion chain compresses in X",
            folded_last_x < rest_last_x - 0.10,
            details=f"rest_last_x={rest_last_x:.3f}, folded_last_x={folded_last_x:.3f}",
        )

    # --- Pull handle sits near center (X≈0) on leaf_3 ---
    leaf_3 = object_model.get_part("leaf_3")
    handle_aabb = ctx.part_element_world_aabb(leaf_3, elem="leaf_3_handle_bar")
    if handle_aabb is not None:
        handle_x = (handle_aabb[0][0] + handle_aabb[1][0]) / 2.0
        ctx.check(
            "pull handle is near center",
            abs(handle_x) < 0.15,
            details=f"handle world X center={handle_x:.3f}",
        )
        # Handle stands off the glass plane in -Y.
        handle_y = (handle_aabb[0][1] + handle_aabb[1][1]) / 2.0
        ctx.check(
            "pull handle stands off the glass",
            handle_y < -FRAME_DEPTH / 2.0,
            details=f"handle world Y center={handle_y:.3f}",
        )

    # --- Equal leaf widths (regular placement) ---
    leaf_widths = [
        (aabbs[i][1][0] - aabbs[i][0][0]) for i in range(LEAF_COUNT)
    ]
    max_width_diff = max(leaf_widths) - min(leaf_widths)
    ctx.check(
        "all leaves have equal width",
        max_width_diff < 0.005,
        details=f"widths={[f'{w:.4f}' for w in leaf_widths]}",
    )

    return ctx.report()


object_model = build_object_model()
