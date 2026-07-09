from __future__ import annotations

# Surgical / operating table with four articulated mattress sections.
#
# Identity (from reference image):
#   A pedestal operating table. A heavy beige cross-foot carries a grey central
#   column. The column supports a four-section padded top: a fixed seat/center
#   pad, a back/torso pad at the head end, a mid/thigh pad, and a foot/calf pad.
#   The back tilts up (back-raise / Trendelenburg), the mid and foot fold down
#   (leg break), each on its own revolute hinge. A horseshoe head support and
#   two curved tubular arm-support rails complete the assembly.
#
# Coordinate convention:
#   +Z is up. Table long axis along +/-X (head +X, foot -X). +Y patient left.
#   Column rises along +Z from cross-foot at floor (z=0).
#
# Sections (head to foot):
#   section_0 = back   (head end, REVOLUTE, tilts up)
#   section_1 = seat   (center, FIXED to column)
#   section_2 = mid    (thigh, REVOLUTE, folds down)
#   section_3 = foot   (calf, REVOLUTE, folds down, chained to mid)

import math

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
# Dimensions (meters)
# ---------------------------------------------------------------------------

# Padded top: four sections + head support.
TOP_W = 0.500          # cushion width (Y)
PAD_T = 0.060          # cushion thickness (Z)
RAIL_T = 0.022         # stainless side rail thickness (square-ish)
RAIL_H = 0.075         # side rail height (Z)
FRAME_W = TOP_W + 2 * RAIL_T  # outer frame width including side rails

# Four mattress section lengths along X.
BACK_LEN = 0.640       # section_0: back/torso (head end)
SEAT_LEN = 0.420       # section_1: fixed center/seat
MID_LEN = 0.280        # section_2: mid/thigh
FOOT_LEN = 0.280       # section_3: foot/calf

HEAD_LEN = 0.260       # horseshoe head support reach (X)

# Top sits at this height (top surface of the seat cushion).
TABLE_TOP_Z = 0.980
SEAT_DECK_Z = TABLE_TOP_Z - PAD_T   # top of the steel seat deck under the pad

# Steel deck plate thickness (Z).
DECK_T = 0.030

# Cross-foot (beige base on the floor).
CROSS_FOOT_LEN = 0.820
CROSS_FOOT_W = 0.560
CROSS_FOOT_H = 0.110

# Central column. Rises from inside the foot up into the seat deck underside.
COL_BASE_Z = CROSS_FOOT_H - 0.060
COL_TOP_Z = SEAT_DECK_Z - DECK_T + 0.018
COL_W = 0.150
COL_TAPER = 0.190

# Head support (horseshoe).
HS_TUBE_R = 0.013
HS_OPENING = 0.150
HS_PAD_T = 0.045

# Arm support rails (curved tubular).
ARM_TUBE_R = 0.011

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

PAD_RGBA = (0.16, 0.16, 0.18, 1.0)       # dark grey upholstery
STEEL_RGBA = (0.78, 0.80, 0.82, 1.0)     # brushed stainless steel
COL_RGBA = (0.55, 0.56, 0.58, 1.0)       # painted grey column
FOOT_RGBA = (0.74, 0.66, 0.52, 1.0)      # beige / tan base
TUBE_RGBA = (0.80, 0.82, 0.84, 1.0)      # bright tubular accessory steel

# ---------------------------------------------------------------------------
# Section layout constants
# ---------------------------------------------------------------------------

N_SECTIONS = 4
SECTION_LENGTHS = [BACK_LEN, SEAT_LEN, MID_LEN, FOOT_LEN]

# Hinged sections: (section_idx, parent_section_idx, hinge_xyz_in_parent, upper_rad)
# All use the same axis (0, -1, 0) for a uniform per-section revolute policy.
HINGED_SECTIONS = [
    (0, 1, (SEAT_LEN / 2.0, 0.0, 0.0), 1.05),     # back tilts up
    (2, 1, (-SEAT_LEN / 2.0, 0.0, 0.0), 1.40),    # mid folds down
    (3, 2, (-MID_LEN, 0.0, 0.0), 1.20),            # foot folds down (chained)
]

HINGE_AXIS = (0.0, -1.0, 0.0)


# ---------------------------------------------------------------------------
# CadQuery helpers
# ---------------------------------------------------------------------------

def _cushion_shape(length: float) -> cq.Workplane:
    """Padded cushion slab with rounded soft top. Section-local frame: deck top
    at z=0, cushion extends to +PAD_T, X over [-length/2, +length/2]."""
    pad = (
        cq.Workplane("XY")
        .box(length, TOP_W, PAD_T, centered=(True, True, False))
        .edges("|X")
        .fillet(0.018)
        .edges(">Z")
        .fillet(0.012)
    )
    return pad


def _side_rails_shape(length: float) -> cq.Workplane:
    """Two stainless side rails framing a cushion section. Section-local frame,
    deck top at z=0. Rails run along X on both +/-Y edges."""
    y_off = TOP_W / 2.0 + RAIL_T / 2.0
    rail_r = (
        cq.Workplane("XY")
        .box(length, RAIL_T, RAIL_H, centered=(True, True, False))
        .translate((0.0, y_off, 0.0))
        .edges("|X")
        .fillet(0.006)
    )
    rail_l = (
        cq.Workplane("XY")
        .box(length, RAIL_T, RAIL_H, centered=(True, True, False))
        .translate((0.0, -y_off, 0.0))
        .edges("|X")
        .fillet(0.006)
    )
    return rail_r.union(rail_l)


def _deck_shape(length: float) -> cq.Workplane:
    """Steel deck plate under a cushion section. Top at z=0, extends to -DECK_T."""
    deck = (
        cq.Workplane("XY")
        .box(length, TOP_W, DECK_T, centered=(True, True, True))
        .translate((0.0, 0.0, -DECK_T / 2.0))
        .edges("|X")
        .fillet(0.004)
    )
    return deck


def _column_shape() -> cq.Workplane:
    """Tapered grey pedestal column, base at z=COL_BASE_Z to z=COL_TOP_Z."""
    h = COL_TOP_Z - COL_BASE_Z
    col = (
        cq.Workplane("XY")
        .workplane(offset=COL_BASE_Z)
        .rect(COL_TAPER, COL_TAPER)
        .workplane(offset=h * 0.45)
        .rect(COL_W, COL_W)
        .workplane(offset=h * 0.55)
        .rect(COL_W, COL_W)
        .loft(combine=True)
    )
    collar = (
        cq.Workplane("XY")
        .box(COL_W + 0.018, COL_W + 0.018, 0.030, centered=(True, True, False))
        .translate((0.0, 0.0, COL_TOP_Z - 0.060))
        .edges("|Z")
        .fillet(0.006)
    )
    return col.union(collar)


def _foot_shape() -> cq.Workplane:
    """Beige cross-foot base, z=0..CROSS_FOOT_H."""
    foot = (
        cq.Workplane("XY")
        .box(CROSS_FOOT_LEN, CROSS_FOOT_W, CROSS_FOOT_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.030)
    )
    return foot


def _tube_from_path(points, radius: float) -> cq.Workplane:
    """Swept circular tube along a 3D spline path."""
    edge_pts = [cq.Vector(*p) for p in points]
    path = cq.Workplane(obj=cq.Edge.makeSpline(edge_pts)).wire()
    p0 = points[0]
    p1 = points[1]
    tangent = cq.Vector(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]).normalized()
    profile = (
        cq.Workplane(
            cq.Plane(origin=cq.Vector(*p0), normal=tangent)
        )
        .circle(radius)
    )
    return profile.sweep(path, transition="round")


def _horseshoe_shape():
    """Horseshoe head support. Open mouth faces +X. Returns (tube, pad)."""
    r = HS_OPENING / 2.0 + HS_TUBE_R
    leg = HEAD_LEN * 0.9
    path = [
        (leg * 0.50, r, 0.0),
        (-leg * 0.20, r, 0.0),
        (-leg * 0.45, r * 0.60, 0.0),
        (-leg * 0.52, 0.0, 0.0),
        (-leg * 0.45, -r * 0.60, 0.0),
        (-leg * 0.20, -r, 0.0),
        (leg * 0.50, -r, 0.0),
    ]
    tube = _tube_from_path(path, HS_TUBE_R)
    pad = (
        cq.Workplane("XY")
        .box(leg * 0.50, 2.0 * r - 2.0 * HS_TUBE_R, HS_PAD_T, centered=(True, True, False))
        .translate((-leg * 0.10, 0.0, HS_TUBE_R - HS_PAD_T / 2.0))
        .edges("|X")
        .fillet(0.012)
    )
    return tube, pad


def _arm_rail_shape(sign: float) -> cq.Workplane:
    """One curved tubular arm-support rail. sign=+1 (patient left) or -1."""
    y0 = sign * (FRAME_W / 2.0 - RAIL_T / 2.0)
    pts = [
        (0.10, y0, 0.005),
        (0.04, y0 + sign * 0.03, 0.12),
        (-0.02, y0 + sign * 0.07, 0.22),
        (0.06, y0 + sign * 0.05, 0.30),
        (0.17, y0 - sign * 0.01, 0.26),
    ]
    return _tube_from_path(pts, ARM_TUBE_R)


# ---------------------------------------------------------------------------
# Shared section geometry helper
# ---------------------------------------------------------------------------

def _add_section_visuals(part, sec_name: str, sec_len: float, extends: int):
    """Add deck, rails, and pad visuals to a section part.

    `extends` controls the local-frame offset:
      +1 : pad extends from hinge at origin toward +X (back section)
       0 : centered at origin (fixed seat)
      -1 : pad extends from hinge at origin toward -X (mid, foot)
    """
    if extends > 0:
        cx = sec_len / 2.0
    elif extends < 0:
        cx = -sec_len / 2.0
    else:
        cx = 0.0

    part.visual(
        mesh_from_cadquery(_deck_shape(sec_len), f"{sec_name}_deck"),
        origin=Origin(xyz=(cx, 0.0, 0.0)),
        material="steel",
        name=f"{sec_name}_deck",
    )
    part.visual(
        mesh_from_cadquery(_side_rails_shape(sec_len), f"{sec_name}_rails"),
        origin=Origin(xyz=(cx, 0.0, -DECK_T)),
        material="steel",
        name=f"{sec_name}_rails",
    )
    part.visual(
        mesh_from_cadquery(_cushion_shape(sec_len), f"{sec_name}_pad"),
        origin=Origin(xyz=(cx, 0.0, 0.0)),
        material="pad",
        name=f"{sec_name}_pad",
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="surgical_bed")

    model.material("pad", rgba=PAD_RGBA)
    model.material("steel", rgba=STEEL_RGBA)
    model.material("column", rgba=COL_RGBA)
    model.material("foot", rgba=FOOT_RGBA)
    model.material("tube", rgba=TUBE_RGBA)

    # ---- Root: cross-foot base + column (fixed support) -------------------
    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_foot_shape(), "foot_block"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="foot",
        name="foot_block",
    )
    base.visual(
        mesh_from_cadquery(_column_shape(), "column_post"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="column",
        name="column_post",
    )

    # ---- Four mattress sections via loop ----------------------------------
    # Each section part gets a shared deck + cushion + rail geometry set.
    # extends: +1 = head end (+X from hinge), 0 = centered, -1 = foot end (-X)
    extends_dirs = [+1, 0, -1, -1]
    section_parts = {}

    for i in range(N_SECTIONS):
        sec_name = f"section_{i}"
        sec_len = SECTION_LENGTHS[i]
        sec = model.part(sec_name)
        section_parts[i] = sec
        _add_section_visuals(sec, sec_name, sec_len, extends_dirs[i])

    # Seat (section_1) extras: two curved arm-support rails.
    seat = section_parts[1]
    seat.visual(
        mesh_from_cadquery(_arm_rail_shape(+1.0), "arm_rail_l"),
        origin=Origin(xyz=(0.0, 0.0, -DECK_T)),
        material="tube",
        name="arm_rail_left",
    )
    seat.visual(
        mesh_from_cadquery(_arm_rail_shape(-1.0), "arm_rail_r"),
        origin=Origin(xyz=(0.0, 0.0, -DECK_T)),
        material="tube",
        name="arm_rail_right",
    )

    # ---- Fixed mount: seat onto column ------------------------------------
    model.articulation(
        "base_to_seat",
        ArticulationType.FIXED,
        parent=base,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, SEAT_DECK_Z)),
    )

    # ---- Revolute joints for hinged sections (uniform policy) -------------
    for child_idx, parent_idx, hinge_xyz, upper_rad in HINGED_SECTIONS:
        child_name = f"section_{child_idx}"
        parent_name = f"section_{parent_idx}"
        model.articulation(
            f"{parent_name}_to_{child_name}",
            ArticulationType.REVOLUTE,
            parent=section_parts[parent_idx],
            child=section_parts[child_idx],
            origin=Origin(xyz=hinge_xyz),
            axis=HINGE_AXIS,
            motion_limits=MotionLimits(
                effort=200.0, velocity=0.5, lower=0.0, upper=upper_rad
            ),
        )

    # ---- Head support (horseshoe), mounted on section_0 (back) ------------
    head = model.part("head")
    hs_tube, hs_pad = _horseshoe_shape()
    head.visual(
        mesh_from_cadquery(hs_tube, "head_tube"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="tube",
        name="head_tube",
    )
    head.visual(
        mesh_from_cadquery(hs_pad, "head_pad"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="pad",
        name="head_pad",
    )
    back = section_parts[0]
    model.articulation(
        "back_to_head",
        ArticulationType.FIXED,
        parent=back,
        child=head,
        origin=Origin(xyz=(BACK_LEN + HEAD_LEN * 0.30, 0.0, PAD_T + HS_TUBE_R)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    sections = [object_model.get_part(f"section_{i}") for i in range(N_SECTIONS)]
    head = object_model.get_part("head")

    seat = sections[1]
    back = sections[0]
    mid = sections[2]
    foot = sections[3]

    seat_to_back = object_model.get_articulation("section_1_to_section_0")
    seat_to_mid = object_model.get_articulation("section_1_to_section_2")
    mid_to_foot = object_model.get_articulation("section_2_to_section_3")

    # --- Column / seat overlap allowance -----------------------------------
    ctx.allow_overlap(
        base,
        seat,
        elem_a="column_post",
        elem_b="section_1_deck",
        reason="Column top is seated into the seat deck underside as the pedestal mount.",
    )
    ctx.expect_overlap(
        base,
        seat,
        axes="xy",
        elem_a="column_post",
        elem_b="section_1_deck",
        min_overlap=0.10,
        name="column carries the seat deck",
    )

    # --- All four sections exist -------------------------------------------
    for i in range(N_SECTIONS):
        ctx.check(
            f"section_{i} exists",
            sections[i] is not None,
            details=f"section_{i} not found",
        )

    # --- Three revolute joints with correct type and axis ------------------
    for joint_name, joint_obj, label in [
        ("section_1_to_section_0", seat_to_back, "back"),
        ("section_1_to_section_2", seat_to_mid, "mid"),
        ("section_2_to_section_3", mid_to_foot, "foot"),
    ]:
        ctx.check(
            f"{label} joint is revolute",
            joint_obj.articulation_type == ArticulationType.REVOLUTE,
            details=str(joint_obj.articulation_type),
        )
        ctx.check(
            f"{label} hinge axis is Y",
            abs(joint_obj.axis[1]) > 0.9
            and abs(joint_obj.axis[0]) < 1e-6
            and abs(joint_obj.axis[2]) < 1e-6,
            details=str(joint_obj.axis),
        )

    # --- Structural placement at rest --------------------------------------
    seat_pad_aabb = ctx.part_element_world_aabb(seat, elem="section_1_pad")
    ctx.check(
        "seat cushion top at table height",
        seat_pad_aabb is not None and abs(seat_pad_aabb[1][2] - TABLE_TOP_Z) < 0.03,
        details=str(seat_pad_aabb),
    )

    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base sits on floor",
        base_aabb is not None and base_aabb[0][2] < 0.01,
        details=str(base_aabb),
    )

    head_aabb = ctx.part_world_aabb(head)
    ctx.check(
        "head support at head end",
        head_aabb is not None and head_aabb[0][0] > SEAT_LEN / 2.0,
        details=str(head_aabb),
    )

    # --- Sections meet at rest (continuous flat table) ---------------------
    ctx.expect_gap(
        back,
        seat,
        axis="x",
        max_gap=0.02,
        max_penetration=0.02,
        positive_elem="section_0_pad",
        negative_elem="section_1_pad",
        name="back pad meets seat at rest",
    )
    ctx.expect_gap(
        seat,
        mid,
        axis="x",
        max_gap=0.02,
        max_penetration=0.02,
        positive_elem="section_1_pad",
        negative_elem="section_2_pad",
        name="mid pad meets seat at rest",
    )
    ctx.expect_gap(
        mid,
        foot,
        axis="x",
        max_gap=0.02,
        max_penetration=0.02,
        positive_elem="section_2_pad",
        negative_elem="section_3_pad",
        name="foot pad meets mid at rest",
    )

    # --- Decisive pose checks: sections actually move ---------------------
    # Back tilts up.
    back_tip_rest = ctx.part_element_world_aabb(back, elem="section_0_pad")
    with ctx.pose({seat_to_back: 0.90}):
        back_tip_up = ctx.part_element_world_aabb(back, elem="section_0_pad")
    ctx.check(
        "back tilts up when actuated",
        back_tip_rest is not None
        and back_tip_up is not None
        and back_tip_up[1][2] > back_tip_rest[1][2] + 0.15,
        details=f"rest_maxz={back_tip_rest[1][2] if back_tip_rest else None}, "
        f"posed_maxz={back_tip_up[1][2] if back_tip_up else None}",
    )

    # Mid folds down.
    mid_tip_rest = ctx.part_element_world_aabb(mid, elem="section_2_pad")
    with ctx.pose({seat_to_mid: 1.20}):
        mid_tip_down = ctx.part_element_world_aabb(mid, elem="section_2_pad")
    ctx.check(
        "mid folds down when actuated",
        mid_tip_rest is not None
        and mid_tip_down is not None
        and mid_tip_down[0][2] < mid_tip_rest[0][2] - 0.08,
        details=f"rest_minz={mid_tip_rest[0][2] if mid_tip_rest else None}, "
        f"posed_minz={mid_tip_down[0][2] if mid_tip_down else None}",
    )

    # Foot folds down (independent of mid).
    foot_tip_rest = ctx.part_element_world_aabb(foot, elem="section_3_pad")
    with ctx.pose({mid_to_foot: 1.00}):
        foot_tip_down = ctx.part_element_world_aabb(foot, elem="section_3_pad")
    ctx.check(
        "foot folds down when actuated",
        foot_tip_rest is not None
        and foot_tip_down is not None
        and foot_tip_down[0][2] < foot_tip_rest[0][2] - 0.08,
        details=f"rest_minz={foot_tip_rest[0][2] if foot_tip_rest else None}, "
        f"posed_minz={foot_tip_down[0][2] if foot_tip_down else None}",
    )

    # --- Arm rails present ------------------------------------------------
    arm_l = ctx.part_element_world_aabb(seat, elem="arm_rail_left")
    arm_r = ctx.part_element_world_aabb(seat, elem="arm_rail_right")
    ctx.check(
        "two arm rails present and raised above deck",
        arm_l is not None
        and arm_r is not None
        and arm_l[1][2] > SEAT_DECK_Z + 0.10
        and arm_r[1][2] > SEAT_DECK_Z + 0.10,
        details=f"arm_l={arm_l}, arm_r={arm_r}",
    )

    return ctx.report()


object_model = build_object_model()
