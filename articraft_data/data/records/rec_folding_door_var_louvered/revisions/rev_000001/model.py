from __future__ import annotations

# 4-panel bi-fold interior louver partition door (accordion / concertina folding louver door).
#
# Fork of the glass partition door: the leaf infill is changed from semi-transparent
# glass panes to horizontal louvered wood slats filling each leaf opening. Everything
# else (leaf count, top-hung track, hinge chain, center pull handle) is identical.
#
# Coordinate convention:
#   +Z is up. The door opening spans along X. Leaves hang in the X/Z plane and
#   the bi-fold swings into +/-Y. At the rest (zero) pose all four leaves are
#   coplanar at Y~0 = the CLOSED door. Driving the joints concertinas the left
#   pair flat against the left jamb and the right pair flat against the right jamb.
#
# Structure:
#   - Root frame (static): top header track, left jamb, right jamb, floor track.
#     The leaves hang from the header; nothing floats.
#   - Each leaf is its own part: a slim steel perimeter frame + horizontal mid-rail
#     (one CadQuery mesh) + horizontal louver slats (shared CadQuery mesh emitted
#     per leaf via a for loop) + hinge-knuckle cylinders down the vertical
#     meeting edge.
#   - The two center leaves carry a slim vertical pull handle on small standoffs.

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

# Header track and jambs
HEADER_H = 0.080            # header track height (Z)
HEADER_DEPTH = 0.060        # header track depth (Y)
JAMB_W = 0.060             # jamb width (X)
JAMB_DEPTH = 0.060         # jamb depth (Y)
FLOOR_H = 0.030            # floor track height (Z)
FLOOR_DEPTH = 0.060

# Leaf vertical extents: leaves hang from just under the header down to the floor
# track, with small running clearances top and bottom so they swing freely and do
# not share a coplanar face with the header/floor.
HEAD_CLEARANCE = 0.012                          # gap below header bottom
FLOOR_CLEARANCE = 0.012                          # gap above floor track top
LEAF_TOP_Z = (OPENING_HEIGHT - HEADER_H) - HEAD_CLEARANCE     # 2.008
LEAF_BOTTOM_Z = FLOOR_H + FLOOR_CLEARANCE                     # 0.042
LEAF_H = LEAF_TOP_Z - LEAF_BOTTOM_Z           # leaf overall height
MID_RAIL_FRACTION = 0.66                       # mid-rail sits ~2/3 up (large lower, shorter upper)

# Hinge knuckles
KNUCKLE_R = 0.016
KNUCKLE_LEN = 0.060
KNUCKLE_COUNT = 5

# Pull handle
HANDLE_R = 0.011
HANDLE_LEN = 0.700
HANDLE_STANDOFF = 0.040    # how far the handle stands off the leaf face (Y)

# Left jamb inner face X (where leaf0 hinges) and successive hinge X lines.
LEFT_HINGE_X = -OPENING_WIDTH / 2.0 + JAMB_W / 2.0
HINGE_X = [LEFT_HINGE_X + i * LEAF_W for i in range(LEAF_COUNT + 1)]
# HINGE_X[0] = left jamb, [1] = leaf0|leaf1 edge, [2] = center (leaf1|leaf2),
# [3] = leaf2|leaf3 edge, [4] = right jamb.

# ---------------------------------------------------------------------------
# Louver slat dimensions
# ---------------------------------------------------------------------------

SLAT_FACE = 0.072           # slat board face width (72 mm, plantation-shutter scale)
SLAT_THICK = 0.005          # slat board thickness (5 mm)
SLAT_ANGLE_DEG = 35.0       # tilt from horizontal (degrees) – angled for privacy
SLAT_PITCH_TARGET = 0.062   # target vertical center-to-center pitch (m)

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

STEEL_RGBA = (0.09, 0.09, 0.10, 1.0)        # near-black powder-coated steel
WOOD_RGBA = (0.72, 0.55, 0.35, 1.0)         # warm natural oak / honey wood
KNUCKLE_RGBA = (0.78, 0.80, 0.82, 1.0)      # bright machined hinge knuckles
HANDLE_RGBA = (0.20, 0.21, 0.23, 1.0)       # dark satin handle


# ---------------------------------------------------------------------------
# Shared slat geometry helper
# ---------------------------------------------------------------------------

def _build_slat_shape(width: float) -> cq.Workplane:
    """One louver slat: a thin plank tilted at the louver angle.

    Authored centered at the local origin:
      - X spans the given *width* (the clear opening span)
      - the plank face is rotated about the X axis by SLAT_ANGLE_DEG so the
        +Y edge is higher than the -Y edge (standard louver tilt)
    """
    return (
        cq.Workplane("XY")
        .transformed(rotate=(SLAT_ANGLE_DEG, 0.0, 0.0))
        .box(width, SLAT_FACE, SLAT_THICK)
    )


# ---------------------------------------------------------------------------
# Leaf steel frame geometry (CadQuery) – unchanged from glass parent
# ---------------------------------------------------------------------------

def _build_leaf_frame_shape() -> cq.Workplane:
    """Slim steel perimeter frame + horizontal mid-rail for one leaf.

    Authored in the leaf-local hinge frame:
      - local X runs 0 .. LEAF_W (hinge edge at x=0, free edge at x=LEAF_W)
      - local Z runs 0 .. LEAF_H
      - local Y is the leaf thickness, centered at y=0
    The frame is the perimeter ring plus a mid-rail; the infill openings are hollow.
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
# Leaf part builder
# ---------------------------------------------------------------------------

def _add_leaf(
    model: ArticulatedObject,
    name: str,
    *,
    knuckle_at_free_edge: bool,
    slat_mesh,
) -> None:
    """Build one leaf part in its leaf-local hinge frame.

    The leaf's hinge edge is at local x=0; the leaf extends to local x=LEAF_W.
    Knuckles run down the free vertical edge (x=LEAF_W) when knuckle_at_free_edge
    is True, otherwise down the hinge edge (x=0).

    Louver slats fill the upper and lower frame openings (separated by the
    mid-rail), emitted via a for loop with equal vertical spacing.
    """
    leaf = model.part(name)

    # Steel frame + mid-rail
    leaf.visual(
        mesh_from_cadquery(_build_leaf_frame_shape(), f"{name}_frame"),
        material="steel",
        name=f"{name}_frame",
    )

    # ---- Louver slats filling the upper and lower openings ----
    mid_z = LEAF_H * MID_RAIL_FRACTION
    half_rail = FRAME_T / 2.0
    slat_center_x = LEAF_W / 2.0  # slat mesh is centered, so place at leaf center

    # Upper opening slats
    up_z0 = mid_z + half_rail
    up_z1 = LEAF_H - FRAME_T
    upper_h = up_z1 - up_z0
    n_upper = max(3, round(upper_h / SLAT_PITCH_TARGET))
    upper_pitch = upper_h / n_upper
    for i in range(n_upper):
        z = up_z0 + upper_pitch * (i + 0.5)
        leaf.visual(
            slat_mesh,
            origin=Origin(xyz=(slat_center_x, 0.0, z)),
            material="wood",
            name=f"{name}_slat_upper_{i}",
        )

    # Lower opening slats
    lo_z0 = FRAME_T
    lo_z1 = mid_z - half_rail
    lower_h = lo_z1 - lo_z0
    n_lower = max(3, round(lower_h / SLAT_PITCH_TARGET))
    lower_pitch = lower_h / n_lower
    for i in range(n_lower):
        z = lo_z0 + lower_pitch * (i + 0.5)
        leaf.visual(
            slat_mesh,
            origin=Origin(xyz=(slat_center_x, 0.0, z)),
            material="wood",
            name=f"{name}_slat_lower_{i}",
        )

    # Hinge knuckles down a vertical edge
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


def _add_handle(model: ArticulatedObject, leaf_name: str) -> None:
    """Add a slim vertical pull handle on the center-meeting edge of a leaf."""
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
    model = ArticulatedObject(name="bifold_louver_partition_door")

    model.material("steel", rgba=STEEL_RGBA)
    model.material("wood", rgba=WOOD_RGBA)
    model.material("knuckle", rgba=KNUCKLE_RGBA)
    model.material("handle", rgba=HANDLE_RGBA)

    # Shared slat geometry – one mesh asset instanced by all leaf visuals.
    slat_width = LEAF_W - 2 * FRAME_T
    slat_mesh = mesh_from_cadquery(_build_slat_shape(slat_width), "louver_slat")

    # --- Root static frame ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_root_frame_shape(), "frame"),
        material="steel",
        name="frame_shell",
    )

    # --- Four leaves ---
    _add_leaf(model, "leaf_0", knuckle_at_free_edge=True, slat_mesh=slat_mesh)
    _add_leaf(model, "leaf_1", knuckle_at_free_edge=False, slat_mesh=slat_mesh)
    _add_leaf(model, "leaf_2", knuckle_at_free_edge=False, slat_mesh=slat_mesh)
    _add_leaf(model, "leaf_3", knuckle_at_free_edge=True, slat_mesh=slat_mesh)

    # Center pull handle on the center-meeting leaf.
    _add_handle(model, "leaf_1")

    # ----- Articulations (identical to glass parent) -----
    z_anchor = LEAF_BOTTOM_Z

    # LEFT STACK
    model.articulation(
        "left_jamb_to_leaf_0",
        ArticulationType.REVOLUTE,
        parent="frame",
        child="leaf_0",
        origin=Origin(xyz=(HINGE_X[0], 0.0, z_anchor)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=0.0, upper=2.7),
    )
    model.articulation(
        "leaf_0_to_leaf_1",
        ArticulationType.REVOLUTE,
        parent="leaf_0",
        child="leaf_1",
        origin=Origin(xyz=(LEAF_W, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=0.0, upper=2.7),
    )

    # RIGHT STACK (mirrored)
    model.articulation(
        "right_jamb_to_leaf_3",
        ArticulationType.REVOLUTE,
        parent="frame",
        child="leaf_3",
        origin=Origin(xyz=(HINGE_X[4], 0.0, z_anchor), rpy=(0.0, 0.0, math.pi)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=0.0, upper=2.7),
    )
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
    # Hinge knuckles straddle the vertical meeting edges between adjacent leaves.
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

    # ---- Louver slat verification ----

    # Compute expected slat counts (same formula as builder)
    mid_z = LEAF_H * MID_RAIL_FRACTION
    half_rail = FRAME_T / 2.0
    up_z0 = mid_z + half_rail
    up_z1 = LEAF_H - FRAME_T
    upper_h = up_z1 - up_z0
    n_upper = max(3, round(upper_h / SLAT_PITCH_TARGET))

    lo_z0 = FRAME_T
    lo_z1 = mid_z - half_rail
    lower_h = lo_z1 - lo_z0
    n_lower = max(3, round(lower_h / SLAT_PITCH_TARGET))

    # Each leaf has louver slats in both openings
    for lf_name in ("leaf_0", "leaf_1", "leaf_2", "leaf_3"):
        lf = object_model.get_part(lf_name)
        # Verify first and last upper slat exist
        aabb_first = ctx.part_element_world_aabb(lf, elem=f"{lf_name}_slat_upper_0")
        aabb_last = ctx.part_element_world_aabb(lf, elem=f"{lf_name}_slat_upper_{n_upper - 1}")
        ctx.check(
            f"{lf_name} has upper louver slats",
            aabb_first is not None and aabb_last is not None,
            details=f"expected {n_upper} upper slats",
        )
        # Verify first and last lower slat exist
        aabb_lo_first = ctx.part_element_world_aabb(lf, elem=f"{lf_name}_slat_lower_0")
        aabb_lo_last = ctx.part_element_world_aabb(lf, elem=f"{lf_name}_slat_lower_{n_lower - 1}")
        ctx.check(
            f"{lf_name} has lower louver slats",
            aabb_lo_first is not None and aabb_lo_last is not None,
            details=f"expected {n_lower} lower slats",
        )

    # No glass visuals remain on any leaf
    for lf_name in ("leaf_0", "leaf_1", "leaf_2", "leaf_3"):
        lf = object_model.get_part(lf_name)
        aabb_glass = ctx.part_element_world_aabb(lf, elem=f"{lf_name}_glass")
        ctx.check(
            f"{lf_name} has no glass pane",
            aabb_glass is None,
            details="glass visual still present",
        )

    # Slats are tilted: Z extent of a slat should be significantly larger than
    # its raw thickness (proves the louver angle is applied).
    aabb_slat = ctx.part_element_world_aabb(leaf_0, elem="leaf_0_slat_upper_0")
    if aabb_slat is not None:
        slat_z_extent = aabb_slat[1][2] - aabb_slat[0][2]
        ctx.check(
            "slats are tilted (Z extent exceeds raw thickness)",
            slat_z_extent > SLAT_THICK * 3.0,
            details=f"Z extent={slat_z_extent:.4f}, expected > {SLAT_THICK * 3:.4f}",
        )

    # Upper slats on leaf_0 are evenly spaced (equal vertical pitch)
    z_centers_upper = []
    for i in range(n_upper):
        aabb = ctx.part_element_world_aabb(leaf_0, elem=f"leaf_0_slat_upper_{i}")
        if aabb is not None:
            z_centers_upper.append((aabb[0][2] + aabb[1][2]) / 2.0)
    if len(z_centers_upper) >= 3:
        pitches = [
            z_centers_upper[i + 1] - z_centers_upper[i]
            for i in range(len(z_centers_upper) - 1)
        ]
        avg_pitch = sum(pitches) / len(pitches)
        max_dev = max(abs(p - avg_pitch) for p in pitches)
        ctx.check(
            "leaf_0 upper slats evenly spaced",
            max_dev < 0.005,
            details=f"avg_pitch={avg_pitch:.4f}, max_deviation={max_dev:.4f}",
        )

    # Lower slats on leaf_0 are evenly spaced
    z_centers_lower = []
    for i in range(n_lower):
        aabb = ctx.part_element_world_aabb(leaf_0, elem=f"leaf_0_slat_lower_{i}")
        if aabb is not None:
            z_centers_lower.append((aabb[0][2] + aabb[1][2]) / 2.0)
    if len(z_centers_lower) >= 3:
        pitches = [
            z_centers_lower[i + 1] - z_centers_lower[i]
            for i in range(len(z_centers_lower) - 1)
        ]
        avg_pitch = sum(pitches) / len(pitches)
        max_dev = max(abs(p - avg_pitch) for p in pitches)
        ctx.check(
            "leaf_0 lower slats evenly spaced",
            max_dev < 0.005,
            details=f"avg_pitch={avg_pitch:.4f}, max_deviation={max_dev:.4f}",
        )

    # Slats fill the opening: first lower slat is near the bottom rail, last
    # upper slat is near the top rail. Use world Z (leaf is mounted at LEAF_BOTTOM_Z).
    if z_centers_lower and z_centers_upper:
        leaf_z_base = LEAF_BOTTOM_Z
        bottom_rail_top_world = leaf_z_base + FRAME_T
        top_rail_bottom_world = leaf_z_base + LEAF_H - FRAME_T
        ctx.check(
            "slats span the leaf opening vertically",
            (z_centers_lower[0] - bottom_rail_top_world) < 0.06
            and (top_rail_bottom_world - z_centers_upper[-1]) < 0.06,
            details=f"lowest_slat_z={z_centers_lower[0]:.3f}, highest_slat_z={z_centers_upper[-1]:.3f}",
        )

    # --- Closed/zero pose: all four leaves coplanar ---
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

    # --- Header is topmost and spans wider than any leaf ---
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

    # --- HERO: left stack folds ---
    rest_l0_y = (leaf0_aabb[0][1] + leaf0_aabb[1][1]) / 2.0
    leaf1_aabb = ctx.part_world_aabb(leaf_1)
    rest_l1_y = (leaf1_aabb[0][1] + leaf1_aabb[1][1]) / 2.0
    rest_l0_x = (leaf0_aabb[0][0] + leaf0_aabb[1][0]) / 2.0

    fold = 2.4
    with ctx.pose({j_l0: fold, j_l1: fold}):
        f0 = ctx.part_world_aabb(leaf_0)
        f1 = ctx.part_world_aabb(leaf_1)
        f0_y = (f0[0][1] + f0[1][1]) / 2.0
        f1_y = (f1[0][1] + f1[1][1]) / 2.0
        f0_x = (f0[0][0] + f0[1][0]) / 2.0
        ctx.check(
            "left stack leaf_0 folds out of plane (+Y)",
            f0_y > rest_l0_y + 0.12,
            details=f"rest_y={rest_l0_y:.3f}, folded_y={f0_y:.3f}",
        )
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

    # --- HERO mirror: right stack folds ---
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

    # --- Pull handle on center meeting edge ---
    handle_aabb = ctx.part_element_world_aabb(leaf_1, elem="leaf_1_handle_bar")
    if handle_aabb is not None:
        handle_x = (handle_aabb[0][0] + handle_aabb[1][0]) / 2.0
        ctx.check(
            "pull handle is on the center meeting edge",
            abs(handle_x) < 0.10,
            details=f"handle world X center={handle_x:.3f}",
        )
        handle_y = (handle_aabb[0][1] + handle_aabb[1][1]) / 2.0
        ctx.check(
            "pull handle stands off the leaf face",
            handle_y < -FRAME_DEPTH / 2.0,
            details=f"handle world Y center={handle_y:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
