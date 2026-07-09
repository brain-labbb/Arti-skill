from __future__ import annotations

# Passenger elevator landing entrance — CENTER-OPENING TELESCOPIC FOUR-LEAF doors.
#
# Coordinate convention (Z-up, meters):
#   +X = wall width        (doors slide along X)
#   +Y = wall thickness / depth, going back into the shaft
#   +Z = height            (ground/floor at z = 0)
#
# Y layout (front -> back):
#   buttons/digit proud    y < 0
#   front track (inner)    y in [-0.040, 0.000]
#   rear track (outer)     y in [ 0.008, 0.048]
#   granite wall front     y = 0
#   granite wall body      y in [0.000, 0.150]
#   shaft recess           y in [~0, 0.350]
#
# The dark granite wall surround is the fixed root. Four brushed-stainless
# telescopic leaves slide apart along X: two per side on front and rear tracks.
# Inner (fast) leaves meet at the center seam; outer (slow) leaves sit beside
# them on the rear track. Indicator, call panel, and sill are separate parts
# fixed to the wall.

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Dimensions
# ---------------------------------------------------------------------------

# Granite wall slab: front face at y=0, body into +Y.
WALL_W = 3.00
WALL_H = 2.70
WALL_D = 0.15

# Doorway opening cut through the wall (wider for 4-leaf telescopic).
OPEN_W = 1.80
OPEN_H = 2.15
OPEN_HALF = OPEN_W / 2.0  # 0.90

# Door leaves (brushed stainless). 4 leaves, each ~quarter of the opening.
LEAF_W = OPEN_W / 4.0  # 0.45
LEAF_BOTTOM = 0.018
LEAF_TOP = OPEN_H
LEAF_H = LEAF_TOP - LEAF_BOTTOM
LEAF_T = 0.040

# Two-track Y layout: inner (fast) leaves on front track, outer (slow) on rear.
TRACK_GAP = 0.008
FRONT_TRACK_CY = -0.020  # inner leaf center Y
REAR_TRACK_CY = FRONT_TRACK_CY + LEAF_T + TRACK_GAP  # 0.028, outer leaf center Y

# Jamb / steel frame lining the opening.
JAMB_T = 0.045
JAMB_D = 0.12

# Prismatic travel: inner (fast) leaves travel farther than outer (slow).
INNER_TRAVEL = 0.82
OUTER_TRAVEL = 0.42

# Indicator box above the doorway.
IND_W = 0.34
IND_H = 0.13
IND_D = 0.06
IND_CZ = OPEN_H + 0.18

# Hall call-button plate (right of the opening).
PLATE_W = 0.085
PLATE_H = 0.16
PLATE_EMBED = 0.012
PLATE_CX = OPEN_HALF + 0.16
PLATE_CZ = 1.10

# Threshold sill at the floor.
SILL_W = OPEN_W + 0.10
SILL_FRONT_Y = -0.14
SILL_BACK_Y = 0.03
SILL_TOP = LEAF_BOTTOM

# Shaft recess behind the doors.
SHAFT_W = OPEN_W
SHAFT_H = OPEN_H
SHAFT_FRONT_Y = 0.0
SHAFT_BACK_Y = 0.35

# ---------------------------------------------------------------------------
# Colors / materials
# ---------------------------------------------------------------------------
GRANITE = (0.17, 0.17, 0.20, 1.0)
STEEL = (0.72, 0.73, 0.75, 1.0)
STEEL_DARK = (0.55, 0.56, 0.58, 1.0)
INDICATOR_BLACK = (0.04, 0.04, 0.05, 1.0)
RED_LED = (0.88, 0.10, 0.10, 1.0)
SHAFT_DARK = (0.05, 0.05, 0.06, 1.0)
BUTTON_STEEL = (0.80, 0.81, 0.83, 1.0)


# ---------------------------------------------------------------------------
# Geometry builders (CadQuery, authored directly in meters)
# ---------------------------------------------------------------------------
def _box_xyz(xc: float, xs: float, yc: float, ys: float, zc: float, zs: float) -> cq.Workplane:
    """Axis-aligned box from explicit center+size on each axis."""
    return cq.Workplane("XY").box(xs, ys, zs).translate((xc, yc, zc))


def _granite_wall_shape() -> cq.Workplane:
    """Dark granite slab with a real rectangular doorway cut through it."""
    slab = _box_xyz(0.0, WALL_W, WALL_D / 2.0, WALL_D, WALL_H / 2.0, WALL_H)
    opening = _box_xyz(
        0.0, OPEN_W, WALL_D / 2.0, WALL_D + 0.04, OPEN_H / 2.0 + 0.05, OPEN_H + 0.10
    )
    return slab.cut(opening)


def _jamb_shape() -> cq.Workplane:
    """Brushed-steel U-jamb lining the doorway opening (two sides + head)."""
    outer_w = OPEN_W + 2.0 * JAMB_T
    outer_top = OPEN_H + JAMB_T
    frame = _box_xyz(0.0, outer_w, JAMB_D / 2.0, JAMB_D, outer_top / 2.0, outer_top)
    clear = _box_xyz(
        0.0, OPEN_W, JAMB_D / 2.0, JAMB_D + 0.04, OPEN_H / 2.0 + 0.05, OPEN_H + 0.10
    )
    band = frame.cut(clear)
    below = _box_xyz(0.0, outer_w + 0.05, JAMB_D / 2.0, JAMB_D + 0.05, -0.05, 0.10)
    return band.cut(below)


def _leaf_shape(side: float, leaf_idx: int) -> cq.Workplane:
    """A single brushed-stainless telescopic door leaf.

    side: -1 (left) or +1 (right).
    leaf_idx: 0 = inner (fast, front track), 1 = outer (slow, rear track).

    At q=0 (closed), all four leaves together span the full opening.
    Inner leaves have their inner edge at x=0 (center seam).
    Outer leaves sit adjacent to inner leaves on the jamb side.
    """
    # Closed-position center X for this leaf.
    if leaf_idx == 0:
        # Inner leaf: inner edge at center (x=0).
        cx = side * (LEAF_W / 2.0)
    else:
        # Outer leaf: adjacent to inner leaf, toward the jamb.
        cx = side * (OPEN_HALF - LEAF_W / 2.0)

    cz = (LEAF_BOTTOM + LEAF_TOP) / 2.0

    # Y track: inner leaves on front track, outer on rear track.
    if leaf_idx == 0:
        cy = FRONT_TRACK_CY
    else:
        cy = REAR_TRACK_CY

    body = _box_xyz(cx, LEAF_W, cy, LEAF_T, cz, LEAF_H)
    return body


def _shaft_recess_shape() -> cq.Workplane:
    """Hollow elevator-car interior visible through the doorway."""
    depth = SHAFT_BACK_Y - SHAFT_FRONT_Y
    t = 0.018
    w = SHAFT_W
    h = SHAFT_H

    back = _box_xyz(0.0, w, depth - t / 2.0, t, h / 2.0, h)
    left = _box_xyz(-w / 2.0 + t / 2.0, t, depth / 2.0, depth, h / 2.0, h)
    right = _box_xyz(w / 2.0 - t / 2.0, t, depth / 2.0, depth, h / 2.0, h)
    ceil_ = _box_xyz(0.0, w, depth / 2.0, depth, h - t / 2.0, t)
    floor_ = _box_xyz(0.0, w, depth / 2.0, depth, t / 2.0, t)

    return back.union(left).union(right).union(ceil_).union(floor_)


def _sill_shape() -> cq.Workplane:
    """Stainless threshold sill at the floor with longitudinal door-track grooves."""
    yc = (SILL_FRONT_Y + SILL_BACK_Y) / 2.0
    yd = SILL_BACK_Y - SILL_FRONT_Y
    base = _box_xyz(0.0, SILL_W, yc, yd, SILL_TOP / 2.0, SILL_TOP)
    # Two sets of grooves: one for front track, one for rear track.
    for track_cy in (FRONT_TRACK_CY, REAR_TRACK_CY):
        for gi in range(2):
            gy = track_cy + (gi - 0.5) * 0.018
            groove = _box_xyz(
                0.0, SILL_W + 0.02, gy, 0.008,
                SILL_TOP - 0.004 + 0.0005, 0.008,
            )
            base = base.cut(groove)
    return base


def _indicator_box_shape() -> cq.Workplane:
    """Recessed black indicator housing embedded in the granite above the door."""
    box = _box_xyz(0.0, IND_W, IND_D / 2.0, IND_D, IND_CZ, IND_H)
    pocket = _box_xyz(0.0, IND_W - 0.04, 0.006, 0.014, IND_CZ, IND_H - 0.04)
    return box.cut(pocket)


def _digit_shape() -> cq.Workplane:
    """Red digit '1' plus a small up-arrow on the indicator face."""
    plate_y_front = -0.003
    plate_y_back = 0.015
    plate_y_c = (plate_y_front + plate_y_back) / 2.0
    plate_y_d = plate_y_back - plate_y_front
    digit = _box_xyz(0.060, 0.014, plate_y_c, plate_y_d, IND_CZ, 0.070)
    serif = _box_xyz(0.060, 0.034, plate_y_c, plate_y_d, IND_CZ - 0.035, 0.012)
    digit = digit.union(serif)
    top_flag = (
        cq.Workplane("XZ")
        .box(0.032, plate_y_d, 0.011)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -28)
        .translate((0.048, plate_y_c, IND_CZ + 0.029))
    )
    digit = digit.union(top_flag)
    arrow = (
        cq.Workplane("XZ")
        .polyline([(-0.085, -0.030), (-0.035, -0.030), (-0.060, 0.030)])
        .close()
        .extrude(plate_y_d)
        .translate((0.0, plate_y_back, IND_CZ))
    )
    return digit.union(arrow)


def _call_plate_shape() -> cq.Workplane:
    """Small stainless hall call-button plate, embedded flush in the wall front."""
    yc = (PLATE_EMBED - 0.002) / 2.0
    yd = PLATE_EMBED + 0.002
    plate = _box_xyz(PLATE_CX, PLATE_W, yc, yd, PLATE_CZ, PLATE_H)
    plate = plate.edges("|Y").fillet(0.006)
    return plate


def _call_buttons_shape() -> cq.Workplane:
    """Two round call buttons (up/down) proud of the call plate, toward -Y."""
    btn_r = 0.016
    btn_len = 0.014
    y_back = 0.003
    up = (
        cq.Workplane("XZ")
        .circle(btn_r)
        .extrude(btn_len)
        .translate((PLATE_CX, y_back, PLATE_CZ + 0.038))
    )
    down = (
        cq.Workplane("XZ")
        .circle(btn_r)
        .extrude(btn_len)
        .translate((PLATE_CX, y_back, PLATE_CZ - 0.038))
    )
    return up.union(down)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="elevator_landing_entrance")

    model.material("granite", rgba=GRANITE)
    model.material("steel", rgba=STEEL)
    model.material("steel_dark", rgba=STEEL_DARK)
    model.material("indicator_black", rgba=INDICATOR_BLACK)
    model.material("red_led", rgba=RED_LED)
    model.material("shaft_dark", rgba=SHAFT_DARK)
    model.material("button_steel", rgba=BUTTON_STEEL)

    # --- Fixed root: the granite wall surround, its jamb, and the shaft recess. ---
    wall = model.part("wall_surround")
    wall.visual(
        mesh_from_cadquery(_granite_wall_shape(), "granite_slab"),
        material="granite",
        name="granite_slab",
    )
    wall.visual(
        mesh_from_cadquery(_jamb_shape(), "door_jamb"),
        material="steel_dark",
        name="door_jamb",
    )
    wall.visual(
        mesh_from_cadquery(_shaft_recess_shape(), "shaft_recess"),
        material="shaft_dark",
        name="shaft_recess",
    )

    # Indicator: distinct part, embedded in (and fixed to) the wall.
    indicator = model.part("indicator")
    indicator.visual(
        mesh_from_cadquery(_indicator_box_shape(), "indicator_box"),
        material="indicator_black",
    )
    indicator.visual(
        mesh_from_cadquery(_digit_shape(), "indicator_digit"),
        material="red_led",
    )

    # Hall call panel: distinct part, embedded beside the opening.
    call_panel = model.part("call_panel")
    call_panel.visual(
        mesh_from_cadquery(_call_plate_shape(), "call_plate"),
        material="steel",
    )
    call_panel.visual(
        mesh_from_cadquery(_call_buttons_shape(), "call_buttons"),
        material="button_steel",
    )

    # Threshold sill: distinct part, butting into the wall front at the floor.
    sill = model.part("sill")
    sill.visual(
        mesh_from_cadquery(_sill_shape(), "sill_track"),
        material="steel",
    )

    # --- Four telescopic door leaves (brushed stainless). ---
    # side_defs: (side_sign, side_name, axis_x)
    # leaf_defs: (leaf_idx, leaf_label, travel)
    side_defs = [(-1.0, "left", -1.0), (1.0, "right", 1.0)]
    leaf_defs = [(0, "inner", INNER_TRAVEL), (1, "outer", OUTER_TRAVEL)]

    door_parts = {}
    door_joints = {}

    for side_sign, side_name, axis_x in side_defs:
        for leaf_idx, leaf_label, travel in leaf_defs:
            part_name = f"{side_name}_{leaf_label}"
            joint_name = f"wall_to_{part_name}"
            visual_name = f"{side_name}_{leaf_label}_leaf"

            leaf_part = model.part(part_name)
            leaf_part.visual(
                mesh_from_cadquery(_leaf_shape(side_sign, leaf_idx), visual_name),
                material="steel",
                name=visual_name,
            )
            door_parts[part_name] = leaf_part

            # Prismatic joint: lower=CLOSED, upper=OPEN.
            # Left side axis = -X (opens toward -X), right side axis = +X.
            jnt = model.articulation(
                joint_name,
                ArticulationType.PRISMATIC,
                parent=wall,
                child=leaf_part,
                origin=Origin(xyz=(0.0, 0.0, 0.0)),
                axis=(axis_x, 0.0, 0.0),
                motion_limits=MotionLimits(
                    lower=0.0, upper=travel, effort=400.0, velocity=0.5
                ),
            )
            door_joints[joint_name] = jnt

    # --- Fixed mounts of the surround sub-parts to the wall root. ---
    model.articulation(
        "wall_to_indicator",
        ArticulationType.FIXED,
        parent=wall,
        child=indicator,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "wall_to_call_panel",
        ArticulationType.FIXED,
        parent=wall,
        child=call_panel,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "wall_to_sill",
        ArticulationType.FIXED,
        parent=wall,
        child=sill,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall = object_model.get_part("wall_surround")
    indicator = object_model.get_part("indicator")
    call_panel = object_model.get_part("call_panel")
    sill = object_model.get_part("sill")

    # Retrieve the four door leaves and their joints.
    left_inner = object_model.get_part("left_inner")
    left_outer = object_model.get_part("left_outer")
    right_inner = object_model.get_part("right_inner")
    right_outer = object_model.get_part("right_outer")

    j_li = object_model.get_articulation("wall_to_left_inner")
    j_lo = object_model.get_articulation("wall_to_left_outer")
    j_ri = object_model.get_articulation("wall_to_right_inner")
    j_ro = object_model.get_articulation("wall_to_right_outer")

    all_leaves = [left_inner, left_outer, right_inner, right_outer]
    all_joints = [j_li, j_lo, j_ri, j_ro]
    closed_pose = {j: 0.0 for j in all_joints}
    open_pose = {
        j_li: INNER_TRAVEL,
        j_lo: OUTER_TRAVEL,
        j_ri: INNER_TRAVEL,
        j_ro: OUTER_TRAVEL,
    }

    # --- Intentional overlaps: seated mounts and rear-track leaves. ---
    ctx.allow_overlap(
        sill, wall,
        reason="The threshold sill butts into the wall front so it is physically attached at the floor.",
    )
    ctx.allow_overlap(
        indicator, wall,
        reason="The indicator housing is recessed into the granite above the doorway.",
    )
    ctx.allow_overlap(
        call_panel, wall,
        reason="The call plate is flush-mounted (embedded) into the granite beside the doorway.",
    )
    # Outer (rear-track) leaves sit partially inside the wall body and shaft
    # recess when closed — this is the rear door pocket, a real telescopic
    # elevator feature where outer leaves ride behind the wall front plane.
    ctx.allow_overlap(
        left_outer, wall,
        elem_a="left_outer_leaf", elem_b="granite_slab",
        reason="Outer leaf rides on the rear track behind the wall front plane, nested inside the granite wall pocket.",
    )
    ctx.allow_overlap(
        left_outer, wall,
        elem_a="left_outer_leaf", elem_b="shaft_recess",
        reason="Outer leaf rides on the rear track behind the wall front plane, nested inside the shaft pocket.",
    )
    ctx.allow_overlap(
        right_outer, wall,
        elem_a="right_outer_leaf", elem_b="granite_slab",
        reason="Outer leaf rides on the rear track behind the wall front plane, nested inside the granite wall pocket.",
    )
    ctx.allow_overlap(
        right_outer, wall,
        elem_a="right_outer_leaf", elem_b="shaft_recess",
        reason="Outer leaf rides on the rear track behind the wall front plane, nested inside the shaft pocket.",
    )

    # Prove fixed mounts contact the wall.
    ctx.expect_contact(indicator, wall, name="indicator is seated in the wall")
    ctx.expect_contact(call_panel, wall, name="call panel is seated in the wall")
    ctx.expect_contact(sill, wall, name="sill is seated against the wall")

    # --- Closed pose (q=0): four leaves cover the full opening. ---
    with ctx.pose(closed_pose):
        # Inner leaves meet at the center seam.
        ctx.expect_gap(
            right_inner, left_inner,
            axis="x", min_gap=-0.002, max_gap=0.004,
            name="closed inner leaves meet at center seam",
        )
        # Outer leaves are adjacent to inner leaves (no large gap between them).
        ctx.expect_gap(
            left_inner, left_outer,
            axis="x", min_gap=-0.002, max_gap=0.006,
            name="closed left outer leaf is adjacent to left inner leaf",
        )
        ctx.expect_gap(
            right_outer, right_inner,
            axis="x", min_gap=-0.002, max_gap=0.006,
            name="closed right outer leaf is adjacent to right inner leaf",
        )

        # Total coverage: from leftmost outer to rightmost outer spans the opening.
        li_aabb = ctx.part_world_aabb(left_inner)
        lo_aabb = ctx.part_world_aabb(left_outer)
        ri_aabb = ctx.part_world_aabb(right_inner)
        ro_aabb = ctx.part_world_aabb(right_outer)

        leftmost_x = min(lo_aabb[0][0], li_aabb[0][0])
        rightmost_x = max(ro_aabb[1][0], ri_aabb[1][0])
        total_w = rightmost_x - leftmost_x
        ctx.check(
            "closed four leaves cover the opening width",
            total_w >= OPEN_W - 0.01,
            details=f"covered_width={total_w:.4f} vs opening={OPEN_W}",
        )

        # All leaves span the opening height.
        for leaf_name, aabb in [("left_inner", li_aabb), ("left_outer", lo_aabb),
                                 ("right_inner", ri_aabb), ("right_outer", ro_aabb)]:
            ctx.check(
                f"{leaf_name} spans the opening height",
                aabb[0][2] < 0.03 and aabb[1][2] > OPEN_H - 0.02,
                details=f"{leaf_name}_z=({aabb[0][2]:.3f},{aabb[1][2]:.3f})",
            )

        # Symmetry: left pair and right pair are mirror images about x=0.
        left_cx = (lo_aabb[0][0] + li_aabb[1][0]) / 2.0
        right_cx = (ri_aabb[0][0] + ro_aabb[1][0]) / 2.0
        ctx.check(
            "left and right leaf pairs are symmetric about center",
            abs(left_cx + right_cx) < 0.02,
            details=f"left_pair_cx={left_cx:.4f}, right_pair_cx={right_cx:.4f}",
        )

    # --- Open pose (upper): leaves nest at the two jambs. ---
    with ctx.pose(open_pose):
        li_o = ctx.part_world_aabb(left_inner)
        lo_o = ctx.part_world_aabb(left_outer)
        ri_o = ctx.part_world_aabb(right_inner)
        ro_o = ctx.part_world_aabb(right_outer)

        # Clear opening: gap between leftmost-right-edge and rightmost-left-edge.
        left_clear = max(li_o[1][0], lo_o[1][0])
        right_clear = min(ri_o[0][0], ro_o[0][0])
        clear_gap = right_clear - left_clear
        ctx.check(
            "open leaves clear most of the opening",
            clear_gap > 0.80 * OPEN_W,
            details=f"clear_gap={clear_gap:.4f} vs opening={OPEN_W}",
        )

        # Left leaves nest at the left jamb (all left-side X < 0).
        ctx.check(
            "left leaves nest at the left jamb",
            li_o[1][0] < -0.4 and lo_o[1][0] < -0.4,
            details=f"left_inner_max_x={li_o[1][0]:.3f}, left_outer_max_x={lo_o[1][0]:.3f}",
        )
        # Right leaves nest at the right jamb (all right-side X > 0).
        ctx.check(
            "right leaves nest at the right jamb",
            ri_o[0][0] > 0.4 and ro_o[0][0] > 0.4,
            details=f"right_inner_min_x={ri_o[0][0]:.3f}, right_outer_min_x={ro_o[0][0]:.3f}",
        )

    # --- Decisive directional: inner leaves travel farther than outer leaves. ---
    with ctx.pose(closed_pose):
        li_closed_x = ctx.part_world_position(left_inner)[0]
        lo_closed_x = ctx.part_world_position(left_outer)[0]
        ri_closed_x = ctx.part_world_position(right_inner)[0]
        ro_closed_x = ctx.part_world_position(right_outer)[0]
    with ctx.pose(open_pose):
        li_open_x = ctx.part_world_position(left_inner)[0]
        lo_open_x = ctx.part_world_position(left_outer)[0]
        ri_open_x = ctx.part_world_position(right_inner)[0]
        ro_open_x = ctx.part_world_position(right_outer)[0]

    left_inner_travel = li_closed_x - li_open_x
    left_outer_travel = lo_closed_x - lo_open_x
    right_inner_travel = ri_open_x - ri_closed_x
    right_outer_travel = ro_open_x - ro_closed_x

    ctx.check(
        "left inner leaf travels farther than left outer leaf",
        left_inner_travel > left_outer_travel + 0.10,
        details=f"inner={left_inner_travel:.3f}m, outer={left_outer_travel:.3f}m",
    )
    ctx.check(
        "right inner leaf travels farther than right outer leaf",
        right_inner_travel > right_outer_travel + 0.10,
        details=f"inner={right_inner_travel:.3f}m, outer={right_outer_travel:.3f}m",
    )

    # Left side moves toward -X, right side toward +X.
    ctx.check(
        "left leaves open toward -X",
        li_open_x < li_closed_x - 0.3 and lo_open_x < lo_closed_x - 0.1,
        details=f"LI {li_closed_x:.3f}->{li_open_x:.3f}, LO {lo_closed_x:.3f}->{lo_open_x:.3f}",
    )
    ctx.check(
        "right leaves open toward +X",
        ri_open_x > ri_closed_x + 0.3 and ro_open_x > ro_closed_x + 0.1,
        details=f"RI {ri_closed_x:.3f}->{ri_open_x:.3f}, RO {ro_closed_x:.3f}->{ro_open_x:.3f}",
    )

    # --- Front and rear tracks are at different Y depths. ---
    with ctx.pose(closed_pose):
        li_y = ctx.part_world_aabb(left_inner)
        lo_y = ctx.part_world_aabb(left_outer)
    ctx.check(
        "inner and outer leaves are on different Y tracks",
        lo_y[0][1] > li_y[0][1] + 0.01,
        details=f"inner_ymin={li_y[0][1]:.3f}, outer_ymin={lo_y[0][1]:.3f}",
    )

    # --- Surround features in the right place. ---
    sill_aabb = ctx.part_world_aabb(sill)
    ctx.check(
        "sill sits at the floor (z ~ 0)",
        sill_aabb[0][2] < 0.005 and sill_aabb[1][2] < 0.05,
        details=f"sill_z=({sill_aabb[0][2]:.4f},{sill_aabb[1][2]:.4f})",
    )

    ind_aabb = ctx.part_world_aabb(indicator)
    ctx.check(
        "indicator is above the doorway opening",
        ind_aabb[0][2] > OPEN_H,
        details=f"indicator_zmin={ind_aabb[0][2]:.4f} vs opening_h={OPEN_H}",
    )

    plate_aabb = ctx.part_world_aabb(call_panel)
    plate_cx = (plate_aabb[0][0] + plate_aabb[1][0]) / 2.0
    plate_cz = (plate_aabb[0][2] + plate_aabb[1][2]) / 2.0
    ctx.check(
        "call panel is beside the opening at hand height",
        plate_cx > OPEN_HALF and 0.9 < plate_cz < 1.3,
        details=f"plate_cx={plate_cx:.3f}, plate_cz={plate_cz:.3f}",
    )

    wall_aabb = ctx.part_world_aabb(wall)
    ctx.check(
        "wall surround is large and grounded",
        wall_aabb[0][2] < 0.01
        and (wall_aabb[1][0] - wall_aabb[0][0]) > OPEN_W + 0.5
        and (wall_aabb[1][2] - wall_aabb[0][2]) > OPEN_H,
        details=f"wall_x=({wall_aabb[0][0]:.2f},{wall_aabb[1][0]:.2f}) wall_z=({wall_aabb[0][2]:.2f},{wall_aabb[1][2]:.2f})",
    )

    return ctx.report()


object_model = build_object_model()
