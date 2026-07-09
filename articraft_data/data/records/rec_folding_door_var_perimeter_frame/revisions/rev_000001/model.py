from __future__ import annotations

# 4-panel bi-fold interior glass partition door (accordion / concertina folding glass door).
#
# Suspension variant: full perimeter cased frame with header track + threshold
# rail, capturing leaf top and bottom edges in U-channels at both rails.
#
# Coordinate convention:
#   +Z is up. The door opening spans along X. Leaves hang in the X/Z plane and
#   the bi-fold swings into +/-Y. At the rest (zero) pose all four leaves are
#   coplanar at Y~0 = the CLOSED door. Driving the joints concertinas the left
#   pair flat against the left jamb and the right pair flat against the right jamb.
#
# Structure:
#   - Root frame (static): full perimeter cased frame — top header track with
#     U-channel groove on bottom face, left jamb, right jamb, threshold rail with
#     matching U-channel groove on top face. Leaf top and bottom edges are captured
#     in both channels; pivot guide pins at each leaf's hinge edge ride in the grooves.
#   - Each leaf is its own part: a slim steel perimeter frame + horizontal mid-rail
#     (one CadQuery mesh) + a thin semi-transparent glass pane + hinge-knuckle
#     cylinders down the vertical meeting edge + pivot pins at top and bottom.
#   - The center leaf carries a slim vertical pull handle on small standoffs.

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
LEAF_COUNT = 4
LEAF_W = 0.535                # each leaf width (4 * 0.535 = 2.14, fits inside jambs)

FRAME_T = 0.035              # steel perimeter / mullion profile width (in-plane)
FRAME_DEPTH = 0.040          # steel profile depth (along Y / thickness)
GLASS_T = 0.010             # glass pane thickness (along Y)

# Header track, threshold rail, and jambs (full perimeter cased frame)
HEADER_H = 0.080            # header track height (Z)
HEADER_DEPTH = 0.060        # header track depth (Y)
JAMB_W = 0.060             # jamb width (X)
JAMB_DEPTH = 0.060         # jamb depth (Y)
FLOOR_H = 0.060            # threshold rail height (Z) — matches header profile
FLOOR_DEPTH = 0.060        # threshold rail depth (Y)

# U-channel grooves in header (bottom face) and threshold (top face) that
# capture the leaf top and bottom edges for full perimeter guided suspension.
CHANNEL_DEPTH = 0.028      # how far the groove extends into the rail (Z)
CHANNEL_W = FRAME_DEPTH + 0.010   # groove width (Y) — leaf frame slides inside

# Leaf vertical extents: top and bottom edges extend into the header and
# threshold channels respectively, so the leaves are captured at both rails.
HEADER_BOTTOM_Z = OPENING_HEIGHT - HEADER_H   # 2.02
THRESHOLD_TOP_Z = FLOOR_H                     # 0.06
LEAF_TOP_Z = HEADER_BOTTOM_Z + CHANNEL_DEPTH - 0.005     # leaf top inside header channel
LEAF_BOTTOM_Z = THRESHOLD_TOP_Z - CHANNEL_DEPTH + 0.005  # leaf bottom inside threshold channel
LEAF_H = LEAF_TOP_Z - LEAF_BOTTOM_Z           # leaf overall height
MID_RAIL_FRACTION = 0.66                       # mid-rail sits ~2/3 up (large upper, short lower)

# Pivot pins at the hinge edge of each leaf (top and bottom) — the visible
# hardware that rides in the header and threshold channels.
PIVOT_PIN_R = 0.008        # pivot pin radius
PIVOT_PIN_LEN = 0.016      # pivot pin length (vertical)
PIVOT_PIN_EMBED = 0.005    # how much of the pin is embedded in the leaf frame

# Hinge knuckles
KNUCKLE_R = 0.016
KNUCKLE_LEN = 0.060
KNUCKLE_COUNT = 5

# Pull handle
HANDLE_R = 0.011
HANDLE_LEN = 0.700
HANDLE_STANDOFF = 0.040    # how far the handle stands off the glass (Y)

# Left jamb inner face X (where leaf0 hinges) and successive hinge X lines.
# Leaves span from the left jamb inner edge to the right jamb inner edge.
LEFT_HINGE_X = -OPENING_WIDTH / 2.0 + JAMB_W / 2.0   # leaf0 hinge at left jamb center line
# Hinge X lines (vertical meeting edges) measured from left jamb:
HINGE_X = [LEFT_HINGE_X + i * LEAF_W for i in range(LEAF_COUNT + 1)]
# HINGE_X[0] = left jamb, [1] = leaf0|leaf1 edge, [2] = center (leaf1|leaf2),
# [3] = leaf2|leaf3 edge, [4] = right jamb.

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

STEEL_RGBA = (0.09, 0.09, 0.10, 1.0)        # near-black powder-coated steel
GLASS_RGBA = (0.42, 0.50, 0.56, 0.30)       # cool grey-blue, semi-transparent
KNUCKLE_RGBA = (0.78, 0.80, 0.82, 1.0)      # bright machined hinge knuckles
HANDLE_RGBA = (0.20, 0.21, 0.23, 1.0)       # dark satin handle


# ---------------------------------------------------------------------------
# Leaf steel frame geometry (CadQuery)
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

    # Outer solid slab, then subtract the two glass openings to leave a frame ring
    # plus the dividing mid-rail.
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
    rebate = 0.006  # glass tucks under the frame lip by this much on each edge

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
    """Static outer frame that carries the whole assembly (full perimeter cased frame).

    World frame: opening centered on X=0, Z from 0 (floor) to OPENING_HEIGHT (top).
    The header has a U-channel groove on its bottom face and the threshold has a
    matching U-channel on its top face. These channels capture the leaf top and
    bottom edges for full-perimeter guided suspension.
    """
    half_w = OPENING_WIDTH / 2.0

    # Top header track spans the full opening width, sits at the very top.
    header = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, OPENING_HEIGHT - HEADER_H / 2.0))
        .box(OPENING_WIDTH, HEADER_DEPTH, HEADER_H)
    )

    # Cut U-channel groove into header bottom face for leaf top-edge capture.
    header_channel = (
        cq.Workplane("XY")
        .transformed(offset=(
            0.0,
            0.0,
            HEADER_BOTTOM_Z + CHANNEL_DEPTH / 2.0,
        ))
        .box(OPENING_WIDTH - JAMB_W, CHANNEL_W, CHANNEL_DEPTH)
    )
    header = header.cut(header_channel)

    # Left jamb (vertical), inner face at -half_w + JAMB_W
    left_jamb = (
        cq.Workplane("XY")
        .transformed(offset=(
            -half_w + JAMB_W / 2.0,
            0.0,
            OPENING_HEIGHT / 2.0,
        ))
        .box(JAMB_W, JAMB_DEPTH, OPENING_HEIGHT)
    )

    # Right jamb
    right_jamb = (
        cq.Workplane("XY")
        .transformed(offset=(
            half_w - JAMB_W / 2.0,
            0.0,
            OPENING_HEIGHT / 2.0,
        ))
        .box(JAMB_W, JAMB_DEPTH, OPENING_HEIGHT)
    )

    # Threshold rail at floor — matches header profile for a cased frame.
    threshold = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, FLOOR_H / 2.0))
        .box(OPENING_WIDTH, FLOOR_DEPTH, FLOOR_H)
    )

    # Cut U-channel groove into threshold top face for leaf bottom-edge capture.
    threshold_channel = (
        cq.Workplane("XY")
        .transformed(offset=(
            0.0,
            0.0,
            THRESHOLD_TOP_Z - CHANNEL_DEPTH / 2.0,
        ))
        .box(OPENING_WIDTH - JAMB_W, CHANNEL_W, CHANNEL_DEPTH)
    )
    threshold = threshold.cut(threshold_channel)

    return header.union(left_jamb).union(right_jamb).union(threshold)


# ---------------------------------------------------------------------------
# Leaf part builder
# ---------------------------------------------------------------------------

def _add_leaf(
    model: ArticulatedObject,
    name: str,
    *,
    knuckle_at_free_edge: bool,
) -> None:
    """Build one leaf part in its leaf-local hinge frame.

    The leaf's hinge edge is at local x=0; the leaf extends to local x=LEAF_W.
    Knuckles run down the free vertical edge (x=LEAF_W) when knuckle_at_free_edge
    is True, otherwise down the hinge edge (x=0). This places the bright hinge
    knuckles on the real meeting edges between panels (image 003).
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

    # Hinge knuckles down a vertical edge (a column of short bright cylinders
    # straddling the meeting edge so the hinge line reads).
    edge_x = LEAF_W if knuckle_at_free_edge else 0.0
    span = LEAF_H - 2 * FRAME_T
    z0 = FRAME_T + KNUCKLE_LEN / 2.0
    for i in range(KNUCKLE_COUNT):
        frac = i / (KNUCKLE_COUNT - 1)
        z = z0 + frac * (span - KNUCKLE_LEN)
        leaf.visual(
            Cylinder(radius=KNUCKLE_R, length=KNUCKLE_LEN),
            origin=Origin(xyz=(edge_x, 0.0, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="knuckle",
            name=f"{name}_knuckle_{i}",
        )

    # Pivot guide pins at top and bottom of the hinge edge — these ride inside
    # the header and threshold U-channels for full-perimeter guided suspension.
    pivot_exposed = PIVOT_PIN_LEN - PIVOT_PIN_EMBED
    # Top pivot: extends upward from leaf top into the header channel.
    leaf.visual(
        Cylinder(radius=PIVOT_PIN_R, length=PIVOT_PIN_LEN),
        origin=Origin(xyz=(edge_x, 0.0, LEAF_H + pivot_exposed / 2.0)),
        material="knuckle",
        name=f"{name}_pivot_top",
    )
    # Bottom pivot: extends downward from leaf bottom into the threshold channel.
    leaf.visual(
        Cylinder(radius=PIVOT_PIN_R, length=PIVOT_PIN_LEN),
        origin=Origin(xyz=(edge_x, 0.0, -pivot_exposed / 2.0)),
        material="knuckle",
        name=f"{name}_pivot_bot",
    )


def _add_handle(model: ArticulatedObject, leaf_name: str) -> None:
    """Add a slim vertical pull handle on the center-meeting edge of a leaf.

    Mounted on two short standoffs on the leaf's free (center-meeting) edge,
    standing off into -Y so it reads as a graspable bar (image 001/002/003).
    """
    leaf = model.get_part(leaf_name)
    handle_x = LEAF_W - FRAME_T / 2.0   # near the center-meeting edge
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

    # --- Four leaves ---
    # Left bi-fold stack: frame -> leaf_0 (hinge at left jamb) -> leaf_1.
    # Right bi-fold stack: frame -> leaf_3 (hinge at right jamb) -> leaf_2.
    # Knuckles read on the vertical meeting edges between panels.
    _add_leaf(model, "leaf_0", knuckle_at_free_edge=True)   # leaf0|leaf1 edge
    _add_leaf(model, "leaf_1", knuckle_at_free_edge=False)  # hinge edge = leaf0|leaf1
    _add_leaf(model, "leaf_2", knuckle_at_free_edge=False)  # hinge edge = leaf2|leaf3
    _add_leaf(model, "leaf_3", knuckle_at_free_edge=True)   # leaf2|leaf3 edge

    # Center pull handle on the two center-meeting leaves (leaf_1 free edge meets
    # leaf_2 free edge at X=0). Put the slim vertical bar there.
    _add_handle(model, "leaf_1")

    # ----- Articulations -----
    # All leaf parts are authored in a local frame where the hinge edge is at
    # local x=0 and the leaf extends along local +X. The joint origin is placed
    # on the real vertical meeting edge (world X = HINGE_X[...]), at the leaf
    # vertical mid-height, so folding happens about the visible knuckle line.

    z_anchor = LEAF_BOTTOM_Z  # leaf local z=0 maps here in world

    # LEFT STACK -------------------------------------------------------------
    # frame -> leaf_0: hinge on the LEFT JAMB (HINGE_X[0]). Leaf extends +X.
    # Positive q about +Z swings the free edge toward +Y (folds out of plane).
    model.articulation(
        "left_jamb_to_leaf_0",
        ArticulationType.REVOLUTE,
        parent="frame",
        child="leaf_0",
        origin=Origin(xyz=(HINGE_X[0], 0.0, z_anchor)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=0.0, upper=2.7),
    )
    # leaf_0 -> leaf_1: hinge on the shared edge (leaf0 free edge = HINGE_X[1]).
    # In leaf_0's local frame that edge is at local x=LEAF_W. leaf_1 extends +X.
    # Opposite fold sign so the pair concertinas (zig-zag), not co-rotate.
    model.articulation(
        "leaf_0_to_leaf_1",
        ArticulationType.REVOLUTE,
        parent="leaf_0",
        child="leaf_1",
        origin=Origin(xyz=(LEAF_W, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=0.0, upper=2.7),
    )

    # RIGHT STACK (mirrored) -------------------------------------------------
    # frame -> leaf_3: hinge on the RIGHT JAMB (HINGE_X[4]). leaf_3 is authored so
    # its hinge edge is at local x=0 and it extends along local +X; we mount it
    # rotated 180 deg about Z so its body reaches back toward the center (-X world).
    model.articulation(
        "right_jamb_to_leaf_3",
        ArticulationType.REVOLUTE,
        parent="frame",
        child="leaf_3",
        origin=Origin(xyz=(HINGE_X[4], 0.0, z_anchor), rpy=(0.0, 0.0, math.pi)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=0.0, upper=2.7),
    )
    # leaf_3 -> leaf_2: shared edge at leaf_3 local x=LEAF_W (world HINGE_X[3]).
    model.articulation(
        "leaf_3_to_leaf_2",
        ArticulationType.REVOLUTE,
        parent="leaf_3",
        child="leaf_2",
        origin=Origin(xyz=(LEAF_W, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=0.0, upper=2.7),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    leaf_0 = object_model.get_part("leaf_0")
    leaf_1 = object_model.get_part("leaf_1")
    leaf_2 = object_model.get_part("leaf_2")
    leaf_3 = object_model.get_part("leaf_3")

    j_l0 = object_model.get_articulation("left_jamb_to_leaf_0")
    j_l1 = object_model.get_articulation("leaf_0_to_leaf_1")
    j_r3 = object_model.get_articulation("right_jamb_to_leaf_3")
    j_r2 = object_model.get_articulation("leaf_3_to_leaf_2")

    # --- Intentional overlaps ---
    # Glass panes tuck under the steel frame lip (captured glass, not floating).
    for lf in ("leaf_0", "leaf_1", "leaf_2", "leaf_3"):
        ctx.allow_overlap(
            lf,
            lf,
            elem_a=f"{lf}_glass",
            elem_b=f"{lf}_frame",
            reason="Glass pane is rebated under the steel frame lip so it reads as captured, not floating.",
        )
    # Hinge knuckles straddle the vertical meeting edges between adjacent leaves
    # and the jambs; that capture is the visible hinge line.
    ctx.allow_overlap(
        "leaf_0", "leaf_1",
        reason="Adjacent leaves share a hinge line; knuckles and frame edges intentionally meet at the fold edge.",
    )
    ctx.allow_overlap(
        "leaf_2", "leaf_3",
        reason="Adjacent leaves share a hinge line; knuckles and frame edges intentionally meet at the fold edge.",
    )
    ctx.allow_overlap(
        "frame", "leaf_0",
        reason="Leaf_0 hinges at the left jamb; its hinge edge and knuckles intentionally meet the jamb.",
    )
    ctx.allow_overlap(
        "frame", "leaf_3",
        reason="Leaf_3 hinges at the right jamb; its hinge edge and knuckles intentionally meet the jamb.",
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

    # Pivot guide pins at leaf hinge edges ride inside the header and threshold
    # U-channels. Small overlap with channel wall edges is the captured-pin fit.
    for lf in ("leaf_0", "leaf_1", "leaf_2", "leaf_3"):
        for pin_name in (f"{lf}_pivot_top", f"{lf}_pivot_bot"):
            ctx.allow_overlap(
                "frame", lf,
                elem_a="frame_shell",
                elem_b=pin_name,
                reason="Pivot guide pin rides inside the header/threshold U-channel; small edge overlap is the captured-pin fit.",
            )

    # --- Closed/zero pose: all four leaves coplanar (similar Y center) ---
    with ctx.pose({j_l0: 0.0, j_l1: 0.0, j_r3: 0.0, j_r2: 0.0}):
        aabbs = {
            "leaf_0": ctx.part_world_aabb(leaf_0),
            "leaf_1": ctx.part_world_aabb(leaf_1),
            "leaf_2": ctx.part_world_aabb(leaf_2),
            "leaf_3": ctx.part_world_aabb(leaf_3),
        }
        y_centers = {
            k: (lo[1] + hi[1]) / 2.0 for k, (lo, hi) in aabbs.items()
        }
        y_vals = list(y_centers.values())
        ctx.check(
            "leaves coplanar at closed pose",
            max(y_vals) - min(y_vals) < 0.05,
            details=f"Y centers: {y_centers}",
        )
        # Leaves are ordered left-to-right with no big X gaps (door reads closed).
        x_centers = {
            k: (lo[0] + hi[0]) / 2.0 for k, (lo, hi) in aabbs.items()
        }
        ordered = [x_centers["leaf_0"], x_centers["leaf_1"], x_centers["leaf_2"], x_centers["leaf_3"]]
        ctx.check(
            "leaves span left-to-right closed",
            ordered[0] < ordered[1] < ordered[2] < ordered[3]
            and (ordered[3] - ordered[0]) > 1.4,
            details=f"X centers L->R: {ordered}",
        )

    # --- Header is topmost and spans wider than any leaf; leaves hang below ---
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
        frame_width > leaf_width + 0.5,
        details=f"frame_width={frame_width:.3f}, leaf_width={leaf_width:.3f}",
    )
    ctx.check(
        "leaves hang below header top",
        leaf_top <= frame_top + 1e-4,
        details=f"leaf_top={leaf_top:.3f}, frame_top={frame_top:.3f}",
    )

    # --- Cased-frame suspension: threshold rail + captured leaf edges ---
    # The frame extends to the floor (threshold rail is part of the frame).
    frame_bot = frame_aabb[0][2]
    ctx.check(
        "threshold rail reaches the floor",
        frame_bot < 0.01,
        details=f"frame bottom Z={frame_bot:.3f}",
    )

    # Each leaf's top and bottom edges are captured inside the header and
    # threshold channels respectively (leaf Z extents lie within the frame Z).
    for lf_name, lf_part in [("leaf_0", leaf_0), ("leaf_1", leaf_1),
                              ("leaf_2", leaf_2), ("leaf_3", leaf_3)]:
        lf_aabb = ctx.part_world_aabb(lf_part)
        lf_top = lf_aabb[1][2]
        lf_bot = lf_aabb[0][2]
        ctx.check(
            f"{lf_name} top captured in header channel",
            lf_top > HEADER_BOTTOM_Z and lf_top < OPENING_HEIGHT,
            details=f"leaf_top_z={lf_top:.3f}, header_channel=[{HEADER_BOTTOM_Z:.3f}, {OPENING_HEIGHT:.3f}]",
        )
        ctx.check(
            f"{lf_name} bottom captured in threshold channel",
            lf_bot > 0.0 and lf_bot < THRESHOLD_TOP_Z,
            details=f"leaf_bot_z={lf_bot:.3f}, threshold_channel=[0.0, {THRESHOLD_TOP_Z:.3f}]",
        )

    # Pivot guide pins exist on each leaf (top and bottom).
    for lf_name in ("leaf_0", "leaf_1", "leaf_2", "leaf_3"):
        lf_part = object_model.get_part(lf_name)
        top_aabb = ctx.part_element_world_aabb(lf_part, elem=f"{lf_name}_pivot_top")
        bot_aabb = ctx.part_element_world_aabb(lf_part, elem=f"{lf_name}_pivot_bot")
        ctx.check(
            f"{lf_name} has top pivot pin",
            top_aabb is not None and top_aabb[1][2] > HEADER_BOTTOM_Z,
            details=f"pivot_top max_z={top_aabb[1][2] if top_aabb else 'missing'}",
        )
        ctx.check(
            f"{lf_name} has bottom pivot pin",
            bot_aabb is not None and bot_aabb[0][2] < THRESHOLD_TOP_Z,
            details=f"pivot_bot min_z={bot_aabb[0][2] if bot_aabb else 'missing'}",
        )

    # --- HERO: driving the left stack folds leaf_0 and leaf_1 toward +Y ---
    rest_l0_y = (leaf0_aabb[0][1] + leaf0_aabb[1][1]) / 2.0
    leaf1_aabb = ctx.part_world_aabb(leaf_1)
    rest_l1_y = (leaf1_aabb[0][1] + leaf1_aabb[1][1]) / 2.0
    rest_l0_x = (leaf0_aabb[0][0] + leaf0_aabb[1][0]) / 2.0

    fold = 2.4  # radians, near flat-stack against the jamb
    with ctx.pose({j_l0: fold, j_l1: fold}):
        f0 = ctx.part_world_aabb(leaf_0)
        f1 = ctx.part_world_aabb(leaf_1)
        f0_y = (f0[0][1] + f0[1][1]) / 2.0
        f1_y = (f1[0][1] + f1[1][1]) / 2.0
        f0_x = (f0[0][0] + f0[1][0]) / 2.0
        # leaf_0 swings out into +Y away from the closed plane
        ctx.check(
            "left stack leaf_0 folds out of plane (+Y)",
            f0_y > rest_l0_y + 0.12,
            details=f"rest_y={rest_l0_y:.3f}, folded_y={f0_y:.3f}",
        )
        # leaf_1 also leaves the closed plane (concertina), and the pair pulls
        # back toward the left jamb (leaf_1 X moves left of its open position).
        ctx.check(
            "left stack leaf_1 leaves closed plane",
            abs(f1_y - rest_l1_y) > 0.10,
            details=f"rest_y={rest_l1_y:.3f}, folded_y={f1_y:.3f}",
        )
        ctx.check(
            "left stack folds back toward left jamb",
            f0_x < rest_l0_x + 0.02 and f1[1][0] < leaf1_aabb[1][0] - 0.10,
            details=f"leaf0 x rest={rest_l0_x:.3f} folded={f0_x:.3f}; leaf1 max_x rest={leaf1_aabb[1][0]:.3f} folded={f1[1][0]:.3f}",
        )

    # --- HERO mirror: driving the right stack folds leaf_3 and leaf_2 toward -Y ---
    leaf3_aabb = ctx.part_world_aabb(leaf_3)
    leaf2_aabb = ctx.part_world_aabb(leaf_2)
    rest_l3_y = (leaf3_aabb[0][1] + leaf3_aabb[1][1]) / 2.0
    rest_l2_y = (leaf2_aabb[0][1] + leaf2_aabb[1][1]) / 2.0
    rest_l3_x = (leaf3_aabb[0][0] + leaf3_aabb[1][0]) / 2.0
    with ctx.pose({j_r3: fold, j_r2: fold}):
        f3 = ctx.part_world_aabb(leaf_3)
        f2 = ctx.part_world_aabb(leaf_2)
        f3_y = (f3[0][1] + f3[1][1]) / 2.0
        f2_y = (f2[0][1] + f2[1][1]) / 2.0
        f3_x = (f3[0][0] + f3[1][0]) / 2.0
        # Right jamb mounted at rpy=pi about Z, so its +axis fold pushes into -Y.
        ctx.check(
            "right stack leaf_3 folds out of plane (-Y)",
            f3_y < rest_l3_y - 0.12,
            details=f"rest_y={rest_l3_y:.3f}, folded_y={f3_y:.3f}",
        )
        ctx.check(
            "right stack leaf_2 leaves closed plane",
            abs(f2_y - rest_l2_y) > 0.10,
            details=f"rest_y={rest_l2_y:.3f}, folded_y={f2_y:.3f}",
        )
        ctx.check(
            "right stack folds back toward right jamb",
            f3_x > rest_l3_x - 0.02 and f2[0][0] > leaf2_aabb[0][0] + 0.10,
            details=f"leaf3 x rest={rest_l3_x:.3f} folded={f3_x:.3f}; leaf2 min_x rest={leaf2_aabb[0][0]:.3f} folded={f2[0][0]:.3f}",
        )

    # --- Pull handle sits on the center meeting edge (near world X=0) ---
    handle_aabb = ctx.part_element_world_aabb(leaf_1, elem="leaf_1_handle_bar")
    if handle_aabb is not None:
        handle_x = (handle_aabb[0][0] + handle_aabb[1][0]) / 2.0
        ctx.check(
            "pull handle is on the center meeting edge",
            abs(handle_x) < 0.10,
            details=f"handle world X center={handle_x:.3f}",
        )
        # Handle stands off the glass plane in -Y.
        handle_y = (handle_aabb[0][1] + handle_aabb[1][1]) / 2.0
        ctx.check(
            "pull handle stands off the glass",
            handle_y < -FRAME_DEPTH / 2.0,
            details=f"handle world Y center={handle_y:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
