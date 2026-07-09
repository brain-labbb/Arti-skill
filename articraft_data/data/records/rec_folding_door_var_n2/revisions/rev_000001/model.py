from __future__ import annotations

# 2-leaf bi-fold interior glass partition door.
#
# Coordinate convention:
#   +Z is up. The door opening spans along X. Leaves hang in the X/Z plane and
#   the bi-fold swings into +/-Y. At the rest (zero) pose both leaves are
#   coplanar at Y~0 = the CLOSED door. Driving the joints concertinas the pair
#   flat against the left jamb.
#
# Structure:
#   - Root frame (static): top header track, left jamb, right jamb, floor track.
#   - Two leaves emitted via for-i-in-range loop with a shared geometry helper:
#     leaf_0 hinged at the left jamb, leaf_1 hinged at the shared edge with
#     leaf_0. Uniform alternating-sign revolute hinge policy concertinas the pair.
#   - Each leaf: slim steel perimeter frame + horizontal mid-rail (CadQuery) +
#     semi-transparent glass panes + hinge knuckle cylinders on vertical edges.
#   - Pull handle on leaf_1 trailing (free) edge.

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

OPENING_WIDTH = 1.20          # total clear opening span along X
OPENING_HEIGHT = 2.10         # total opening height along Z (top of header)
LEAF_COUNT = 2                # single 2-leaf bi-fold pair

FRAME_T = 0.035               # steel perimeter / mullion profile width (in-plane)
FRAME_DEPTH = 0.040           # steel profile depth (along Y / thickness)
GLASS_T = 0.010               # glass pane thickness (along Y)

# Header track and jambs
HEADER_H = 0.080              # header track height (Z)
HEADER_DEPTH = 0.060          # header track depth (Y)
JAMB_W = 0.060                # jamb width (X)
JAMB_DEPTH = 0.060            # jamb depth (Y)
FLOOR_H = 0.030               # floor track height (Z)
FLOOR_DEPTH = 0.060

# Leaf vertical extents: leaves hang from just under the header down to the floor
# track, with small running clearances top and bottom so they swing freely.
HEAD_CLEARANCE = 0.012
FLOOR_CLEARANCE = 0.012
LEAF_TOP_Z = (OPENING_HEIGHT - HEADER_H) - HEAD_CLEARANCE
LEAF_BOTTOM_Z = FLOOR_H + FLOOR_CLEARANCE
LEAF_H = LEAF_TOP_Z - LEAF_BOTTOM_Z
MID_RAIL_FRACTION = 0.66      # mid-rail sits ~2/3 up (large upper, short lower)

# Leaf width: equal leaves summing to the clear span between jambs
CLEAR_SPAN = OPENING_WIDTH - 2 * JAMB_W
LEAF_W = CLEAR_SPAN / LEAF_COUNT   # each leaf width

# Hinge knuckles
KNUCKLE_R = 0.016
KNUCKLE_LEN = 0.060
KNUCKLE_COUNT = 5

# Pull handle
HANDLE_R = 0.011
HANDLE_LEN = 0.700
HANDLE_STANDOFF = 0.040

# Left jamb hinge X and successive hinge X lines.
LEFT_HINGE_X = -OPENING_WIDTH / 2.0 + JAMB_W / 2.0
HINGE_X = [LEFT_HINGE_X + i * LEAF_W for i in range(LEAF_COUNT + 1)]

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

STEEL_RGBA = (0.09, 0.09, 0.10, 1.0)
GLASS_RGBA = (0.42, 0.50, 0.56, 0.30)
KNUCKLE_RGBA = (0.78, 0.80, 0.82, 1.0)
HANDLE_RGBA = (0.20, 0.21, 0.23, 1.0)


# ---------------------------------------------------------------------------
# Shared geometry helpers (one per leaf shape, reused by loop)
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

    return outer.cut(upper_cut).cut(lower_cut)


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
    """Static outer frame: header track, left jamb, right jamb, floor track.

    World frame: opening centered on X=0, Z from 0 (floor) to OPENING_HEIGHT.
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
# Leaf part builder (shared helper used by the emission loop)
# ---------------------------------------------------------------------------

def _add_leaf(
    model: ArticulatedObject,
    name: str,
    *,
    knuckle_at_hinge_edge: bool = False,
    knuckle_at_free_edge: bool = True,
) -> None:
    """Build one leaf part in its leaf-local hinge frame.

    The leaf's hinge edge is at local x=0; the leaf extends to local x=LEAF_W.
    Knuckles are placed on the requested vertical edges so the hinge lines read.
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

    # Hinge knuckles: columns of short bright cylinders on the selected edges
    span = LEAF_H - 2 * FRAME_T
    z0 = FRAME_T + KNUCKLE_LEN / 2.0

    edges: list[float] = []
    if knuckle_at_hinge_edge:
        edges.append(0.0)
    if knuckle_at_free_edge:
        edges.append(LEAF_W)

    knuckle_idx = 0
    for edge_x in edges:
        for j in range(KNUCKLE_COUNT):
            frac = j / (KNUCKLE_COUNT - 1)
            z = z0 + frac * (span - KNUCKLE_LEN)
            leaf.visual(
                Cylinder(radius=KNUCKLE_R, length=KNUCKLE_LEN),
                origin=Origin(xyz=(edge_x, 0.0, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material="knuckle",
                name=f"{name}_knuckle_{knuckle_idx}",
            )
            knuckle_idx += 1


def _add_handle(model: ArticulatedObject, leaf_name: str) -> None:
    """Add a slim vertical pull handle on the free (trailing) edge of a leaf.

    Mounted on two short standoffs, standing off into -Y so it reads as a
    graspable bar.
    """
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

    # --- Emit leaves via loop with shared geometry helper ---
    # leaf_0: knuckles at hinge edge (left jamb) and free edge (shared hinge)
    # leaf_1: knuckles at hinge edge (shared hinge with leaf_0)
    for i in range(LEAF_COUNT):
        name_i = f"leaf_{i}"
        _add_leaf(
            model,
            name_i,
            knuckle_at_hinge_edge=True,
            knuckle_at_free_edge=(i < LEAF_COUNT - 1),
        )

    # Pull handle on leaf_1 trailing (free) edge
    _add_handle(model, "leaf_1")

    # ----- Articulations: uniform alternating-sign revolute hinge policy -----
    # All leaf parts are authored in a local frame where the hinge edge is at
    # local x=0 and the leaf extends along local +X. The joint origin is placed
    # on the real vertical meeting edge, at the leaf vertical base, so folding
    # happens about the visible knuckle line.
    z_anchor = LEAF_BOTTOM_Z

    for i in range(LEAF_COUNT):
        name_i = f"leaf_{i}"
        # Alternating sign: even joints use +Z axis, odd use -Z axis.
        # This concertinas the pair: leaf_0 swings toward +Y, leaf_1 folds back.
        sign = 1.0 if i % 2 == 0 else -1.0

        if i == 0:
            # frame -> leaf_0: hinge on the LEFT JAMB (HINGE_X[0]).
            model.articulation(
                "jamb_to_leaf_0",
                ArticulationType.REVOLUTE,
                parent="frame",
                child=name_i,
                origin=Origin(xyz=(HINGE_X[0], 0.0, z_anchor)),
                axis=(0.0, 0.0, sign),
                motion_limits=MotionLimits(
                    effort=40.0, velocity=1.5, lower=0.0, upper=2.7,
                ),
            )
        else:
            # leaf_{i-1} -> leaf_i: hinge on the shared vertical edge.
            # In leaf_{i-1}'s local frame, the shared edge is at local x=LEAF_W.
            model.articulation(
                f"leaf_{i - 1}_to_{name_i}",
                ArticulationType.REVOLUTE,
                parent=f"leaf_{i - 1}",
                child=name_i,
                origin=Origin(xyz=(LEAF_W, 0.0, 0.0)),
                axis=(0.0, 0.0, sign),
                motion_limits=MotionLimits(
                    effort=40.0, velocity=1.5, lower=0.0, upper=2.7,
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
    leaf_0, leaf_1 = leaves[0], leaves[1]

    j_0 = object_model.get_articulation("jamb_to_leaf_0")
    j_1 = object_model.get_articulation("leaf_0_to_leaf_1")

    # --- Structural: exactly 2 leaves and 2 non-fixed revolute hinges ---
    has_leaf_2 = True
    try:
        object_model.get_part("leaf_2")
    except Exception:
        has_leaf_2 = False
    ctx.check(
        "exactly 2 leaf parts exist",
        all(
            object_model.get_part(f"leaf_{i}") is not None
            for i in range(LEAF_COUNT)
        )
        and not has_leaf_2,
        details="leaf_0 and leaf_1 must exist; leaf_2 must not",
    )
    ctx.check(
        "two revolute articulations",
        j_0 is not None and j_1 is not None,
        details="jamb_to_leaf_0 and leaf_0_to_leaf_1 must exist",
    )

    # --- Intentional overlaps ---
    # Glass panes tuck under the steel frame lip (captured glass, not floating).
    for i in range(LEAF_COUNT):
        lf = f"leaf_{i}"
        ctx.allow_overlap(
            lf, lf,
            elem_a=f"{lf}_glass",
            elem_b=f"{lf}_frame",
            reason="Glass pane is rebated under the steel frame lip so it reads as captured, not floating.",
        )
    # Hinge knuckles straddle the vertical meeting edges between adjacent leaves
    # and at the left jamb; that capture is the visible hinge line.
    ctx.allow_overlap(
        "leaf_0", "leaf_1",
        reason="Adjacent leaves share a hinge line; knuckles and frame edges intentionally meet at the fold edge.",
    )
    ctx.allow_overlap(
        "frame", "leaf_0",
        reason="Leaf_0 hinges at the left jamb; its hinge edge and knuckles intentionally meet the jamb.",
    )
    # Handle standoffs penetrate the leaf_1 frame to mount the bar.
    ctx.allow_overlap(
        "leaf_1", "leaf_1",
        elem_a="leaf_1_handle_standoff_top",
        elem_b="leaf_1_frame",
        reason="Handle standoff is seated into the leaf frame to mount the pull bar.",
    )
    ctx.allow_overlap(
        "leaf_1", "leaf_1",
        elem_a="leaf_1_handle_standoff_bot",
        elem_b="leaf_1_frame",
        reason="Handle standoff is seated into the leaf frame to mount the pull bar.",
    )

    # --- Closed/zero pose: both leaves coplanar, ordered left-to-right ---
    with ctx.pose({j_0: 0.0, j_1: 0.0}):
        aabbs = [ctx.part_world_aabb(lf) for lf in leaves]
        y_centers = [(lo[1] + hi[1]) / 2.0 for lo, hi in aabbs]
        ctx.check(
            "leaves coplanar at closed pose",
            max(y_centers) - min(y_centers) < 0.05,
            details=f"Y centers: {y_centers}",
        )
        x_centers = [(lo[0] + hi[0]) / 2.0 for lo, hi in aabbs]
        ctx.check(
            "leaf_0 left of leaf_1 at closed",
            x_centers[0] < x_centers[1],
            details=f"X centers: leaf_0={x_centers[0]:.3f}, leaf_1={x_centers[1]:.3f}",
        )
        # Leaves together span most of the clear opening
        total_span = aabbs[1][1][0] - aabbs[0][0][0]
        ctx.check(
            "leaves span the clear opening",
            total_span > CLEAR_SPAN * 0.85,
            details=f"leaf span={total_span:.3f}, clear_span={CLEAR_SPAN:.3f}",
        )

    # --- Header is topmost element, frame wider than a single leaf ---
    frame_aabb = ctx.part_world_aabb(frame)
    leaf0_aabb = ctx.part_world_aabb(leaf_0)
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
        frame_width > leaf_width + 0.1,
        details=f"frame_width={frame_width:.3f}, leaf_width={leaf_width:.3f}",
    )
    ctx.check(
        "leaves hang below header top",
        leaf_top <= frame_top + 1e-4,
        details=f"leaf_top={leaf_top:.3f}, frame_top={frame_top:.3f}",
    )

    # --- Each leaf has glass + frame + knuckle visuals ---
    for i in range(LEAF_COUNT):
        lf_name = f"leaf_{i}"
        lf = object_model.get_part(lf_name)
        visual_names = [v.name for v in lf.visuals]
        ctx.check(
            f"{lf_name} has frame and glass",
            f"{lf_name}_frame" in visual_names and f"{lf_name}_glass" in visual_names,
            details=f"visuals: {visual_names}",
        )
        ctx.check(
            f"{lf_name} has hinge knuckles",
            any("knuckle" in vn for vn in visual_names),
            details=f"visuals: {visual_names}",
        )

    # --- HERO: driving both joints folds the pair toward +Y (concertina) ---
    rest_y0 = (leaf0_aabb[0][1] + leaf0_aabb[1][1]) / 2.0
    leaf1_aabb = ctx.part_world_aabb(leaf_1)
    rest_y1 = (leaf1_aabb[0][1] + leaf1_aabb[1][1]) / 2.0
    rest_x0 = (leaf0_aabb[0][0] + leaf0_aabb[1][0]) / 2.0

    fold = 2.2  # radians, near flat-stack against the jamb
    with ctx.pose({j_0: fold, j_1: fold}):
        f0 = ctx.part_world_aabb(leaf_0)
        f1 = ctx.part_world_aabb(leaf_1)
        f0_y = (f0[0][1] + f0[1][1]) / 2.0
        f1_y = (f1[0][1] + f1[1][1]) / 2.0
        f0_x = (f0[0][0] + f0[1][0]) / 2.0

        # leaf_0 swings out into +Y away from the closed plane
        ctx.check(
            "leaf_0 folds out of plane (+Y)",
            f0_y > rest_y0 + 0.10,
            details=f"rest_y={rest_y0:.3f}, folded_y={f0_y:.3f}",
        )
        # Concertina: leaf_1 folds back relative to leaf_0 (different Y)
        ctx.check(
            "leaf_1 concertinas (zigzag relative to leaf_0)",
            abs(f1_y - f0_y) > 0.04,
            details=f"leaf_0_y={f0_y:.3f}, leaf_1_y={f1_y:.3f}",
        )
        # The pair pulls back toward the left jamb (X decreases)
        ctx.check(
            "folded pair pulls toward left jamb",
            f0_x < rest_x0 + 0.05,
            details=f"leaf_0 x rest={rest_x0:.3f} folded={f0_x:.3f}",
        )
        # leaf_1 max_x retracts from its open position
        ctx.check(
            "leaf_1 retracts from open span",
            f1[1][0] < leaf1_aabb[1][0] - 0.10,
            details=f"leaf1 max_x rest={leaf1_aabb[1][0]:.3f} folded={f1[1][0]:.3f}",
        )

    # --- Pull handle on leaf_1 trailing edge ---
    handle_aabb = ctx.part_element_world_aabb(leaf_1, elem="leaf_1_handle_bar")
    if handle_aabb is not None:
        handle_x = (handle_aabb[0][0] + handle_aabb[1][0]) / 2.0
        handle_y = (handle_aabb[0][1] + handle_aabb[1][1]) / 2.0
        ctx.check(
            "handle is on leaf_1 trailing edge (right side)",
            handle_x > 0.0,
            details=f"handle world X center={handle_x:.3f}",
        )
        ctx.check(
            "handle stands off the glass plane",
            handle_y < -FRAME_DEPTH / 2.0,
            details=f"handle world Y center={handle_y:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
