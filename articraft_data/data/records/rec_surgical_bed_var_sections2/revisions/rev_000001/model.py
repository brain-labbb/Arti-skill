from __future__ import annotations

# Surgical / operating table — fork variant: two-section mattress top.
#
# Identity (from reference image):
#   A pedestal operating table. A heavy beige cross-foot carries a grey central
#   column. The column supports a two-section padded top: a fixed seat/center
#   pad and a back/torso pad at the head end on a revolute hinge, plus a
#   horseshoe head support and two curved tubular arm-support rails. The dark
#   grey upholstered cushions sit in stainless side frames.
#
# Coordinate convention:
#   +Z is up. The table long axis runs along +/-X (head end toward +X, foot end
#   toward -X). +Y is the patient's left. The column rises along +Z from the
#   cross-foot at the floor (z=0).
#
# Articulation:
#   - The BACK section (section_0) is hinged at the seat head edge and tilts UP
#     (back-raise / Trendelenburg) -> REVOLUTE. Emitted via a for-i-in-range
#     loop with shared geometry helpers and a uniform joint policy.
#   - The head support and arm rails are rigid accessories mounted on the
#     back section and seat frame respectively.

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

# Padded top: split into fixed seat + hinged back section.
TOP_W = 0.500          # cushion width (Y)
PAD_T = 0.060          # cushion thickness (Z)
RAIL_T = 0.022         # stainless side rail thickness (square-ish)
RAIL_H = 0.075         # side rail height (Z)
FRAME_W = TOP_W + 2 * RAIL_T  # outer frame width including side rails

SEAT_LEN = 0.420       # fixed center/seat section length (X)
BACK_LEN = 0.640       # back/torso section length (X)  (head end)
HEAD_LEN = 0.260       # horseshoe head support reach (X)

# Top sits at this height (top surface of the seat cushion).
TABLE_TOP_Z = 0.980
SEAT_DECK_Z = TABLE_TOP_Z - PAD_T   # top of the steel seat deck under the pad

# Cross-foot (beige base on the floor)
FOOT_LEN = 0.820       # along X
FOOT_W = 0.560         # along Y
FOOT_H = 0.110         # height of the beige block (Z), sits on floor at z=0

# Seat steel deck thickness is needed for the column-top clearance below.
DECK_T = 0.030         # steel deck plate thickness (Z)

# Central column. It rises from well inside the foot (overlapping it for a solid
# structural joint) up into the seat deck underside (also overlapping it).
COL_BASE_Z = FOOT_H - 0.060         # start inside the foot block (solid overlap)
COL_TOP_Z = SEAT_DECK_Z - DECK_T + 0.018  # top reaches up into the seat deck
COL_W = 0.150                       # column cross-section (square-ish)
COL_TAPER = 0.190                   # wider near the base

# Head support (horseshoe)
HS_TUBE_R = 0.013
HS_OPENING = 0.150     # gap of the horseshoe (Y)
HS_PAD_T = 0.045

# Arm support rails (curved tubular)
ARM_TUBE_R = 0.011

# ---------------------------------------------------------------------------
# Hinged section configuration (for-loop policy)
# ---------------------------------------------------------------------------

# Each entry defines one hinged mattress section: its length, the hinge X
# position on the seat edge, the extension direction (+1 = head/+X, -1 = foot/
# -X), and the upper motion limit. The for-loop emits section_i parts with
# shared geometry helpers and a uniform revolute joint policy hinged off seat.
HINGED_SECTIONS = [
    {
        "length": BACK_LEN,
        "hinge_x": SEAT_LEN / 2.0,
        "direction": +1,
        "upper": 1.05,
    },
]

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

PAD_RGBA = (0.16, 0.16, 0.18, 1.0)       # dark grey upholstery
STEEL_RGBA = (0.78, 0.80, 0.82, 1.0)     # brushed stainless steel
COL_RGBA = (0.55, 0.56, 0.58, 1.0)       # painted grey column
FOOT_RGBA = (0.74, 0.66, 0.52, 1.0)      # beige / tan base
TUBE_RGBA = (0.80, 0.82, 0.84, 1.0)      # bright tubular accessory steel


# ---------------------------------------------------------------------------
# CadQuery helpers
# ---------------------------------------------------------------------------

def _cushion_shape(length: float) -> cq.Workplane:
    """A padded cushion: a slab with a rounded soft top, built in section-local
    frame with the deck top at z=0, cushion extends up to +PAD_T, X over
    [-length/2, +length/2], Y over [-TOP_W/2, +TOP_W/2]."""
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
    """Two stainless side rails framing a cushion section, in section-local
    frame, deck top at z=0. Rails run along X on both +/-Y edges."""
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
    """Steel deck plate under a cushion section (section-local frame). Sits just
    below the cushion: top at z=0, extends down to -DECK_T."""
    deck = (
        cq.Workplane("XY")
        .box(length, TOP_W, DECK_T, centered=(True, True, True))
        .translate((0.0, 0.0, -DECK_T / 2.0))
        .edges("|X")
        .fillet(0.004)
    )
    return deck


def _column_shape() -> cq.Workplane:
    """Tapered grey pedestal column in world-Z frame, base at z=COL_BASE_Z rising
    to z=COL_TOP_Z. Wider near the bottom. Authored at its final world Z so it
    can be fused with the foot into one connected base solid."""
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
    # Decorative recessed band collar near the top.
    collar = (
        cq.Workplane("XY")
        .box(COL_W + 0.018, COL_W + 0.018, 0.030, centered=(True, True, False))
        .translate((0.0, 0.0, COL_TOP_Z - 0.060))
        .edges("|Z")
        .fillet(0.006)
    )
    return col.union(collar)


def _foot_shape() -> cq.Workplane:
    """Beige cross-foot base, sitting on the floor (z=0..FOOT_H). A wide
    rectangular block with softened vertical corners that reads as the heavy
    stabilizing base. Kept as a clean single watertight solid."""
    foot = (
        cq.Workplane("XY")
        .box(FOOT_LEN, FOOT_W, FOOT_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.030)
    )
    return foot


def _tube_from_path(points, radius: float) -> cq.Workplane:
    """Build a swept circular tube along a 3D spline path with CadQuery.

    `points` is a list of (x, y, z) centerline points. Returns a solid
    Workplane. The profile circle is placed perpendicular to the path start so
    the sweep follows the spline cleanly.
    """
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
    """Horseshoe (U-shaped) head support. Built in head-local frame: the open
    mouth faces +X (toward the back pad) and the closed curve faces -X. Returns
    (tube_solid, pad_solid). Reference z=0 at the tube centerline plane."""
    r = HS_OPENING / 2.0 + HS_TUBE_R
    leg = HEAD_LEN * 0.9
    # U centerline path in the XY plane (z=0): legs run along X, bend at -X.
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
    # A thin contoured pad bridging the closed (-X) end of the U.
    pad = (
        cq.Workplane("XY")
        .box(leg * 0.50, 2.0 * r - 2.0 * HS_TUBE_R, HS_PAD_T, centered=(True, True, False))
        .translate((-leg * 0.10, 0.0, HS_TUBE_R - HS_PAD_T / 2.0))
        .edges("|X")
        .fillet(0.012)
    )
    return tube, pad


def _arm_rail_shape(sign: float) -> cq.Workplane:
    """One curved tubular arm-support rail that arcs up and outward from a seat
    side rail, like the curved metal arms in the image. Built in seat-local
    frame; sign = +1 (patient left) or -1 (right). Starts at the side rail."""
    y0 = sign * (FRAME_W / 2.0 - RAIL_T / 2.0)
    pts = [
        (0.10, y0, 0.005),
        (0.04, y0 + sign * 0.03, 0.12),
        (-0.02, y0 + sign * 0.07, 0.22),
        (0.06, y0 + sign * 0.05, 0.30),
        (0.17, y0 - sign * 0.01, 0.26),
    ]
    return _tube_from_path(pts, ARM_TUBE_R)


def _build_section_visuals(part, name: str, length: float, direction: int):
    """Shared geometry helper: attach deck, side rails, and cushion visuals to
    a hinged section part. The part frame sits at the hinge line; visuals extend
    along `direction * +X` from there."""
    x_off = direction * length / 2.0
    part.visual(
        mesh_from_cadquery(_deck_shape(length), f"{name}_deck"),
        origin=Origin(xyz=(x_off, 0.0, 0.0)),
        material="steel",
        name=f"{name}_deck",
    )
    part.visual(
        mesh_from_cadquery(_side_rails_shape(length), f"{name}_rails"),
        origin=Origin(xyz=(x_off, 0.0, -DECK_T)),
        material="steel",
        name=f"{name}_rails",
    )
    part.visual(
        mesh_from_cadquery(_cushion_shape(length), f"{name}_pad"),
        origin=Origin(xyz=(x_off, 0.0, 0.0)),
        material="pad",
        name=f"{name}_pad",
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
    # The beige cross-foot carries the grey column. The column base socket sits
    # well down inside the foot block (solid interpenetration) so the two visuals
    # form one connected pedestal casting, not a floating column island.
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

    # ---- Seat / center section (fixed to column) --------------------------
    # Seat-local frame: deck top at world z = SEAT_DECK_Z, centered at x=0.
    seat = model.part("seat")
    seat.visual(
        mesh_from_cadquery(_deck_shape(SEAT_LEN), "seat_deck"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="steel",
        name="seat_deck",
    )
    seat.visual(
        mesh_from_cadquery(_side_rails_shape(SEAT_LEN), "seat_rails"),
        origin=Origin(xyz=(0.0, 0.0, -DECK_T)),
        material="steel",
        name="seat_rails",
    )
    seat.visual(
        mesh_from_cadquery(_cushion_shape(SEAT_LEN), "seat_pad"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="pad",
        name="seat_pad",
    )

    # Two curved arm-support rails mounted on the seat side frames.
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

    # Mount seat onto the column (fixed).
    model.articulation(
        "column_to_seat",
        ArticulationType.FIXED,
        parent=base,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, SEAT_DECK_Z)),
    )

    # ---- Hinged mattress sections (for-loop policy) -----------------------
    # Each section gets deck + rails + cushion via the shared helper, and a
    # uniform revolute joint hinged off the seat at the configured hinge_x.
    section_parts = []
    section_joints = []
    for i, sec in enumerate(HINGED_SECTIONS):
        name = f"section_{i}"
        section = model.part(name)
        _build_section_visuals(section, name, sec["length"], sec["direction"])

        # Revolute hinge at the seat edge. The section extends along
        # direction * +X from the hinge. Axis -Y lifts the free edge upward
        # for positive q when direction = +1 (back-raise).
        joint_name = f"seat_to_{name}"
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=seat,
            child=section,
            origin=Origin(xyz=(sec["hinge_x"], 0.0, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=200.0, velocity=0.5, lower=0.0, upper=sec["upper"],
            ),
        )
        section_parts.append(section)
        section_joints.append(joint_name)

    # ---- Head support (horseshoe), mounted on section_0 -------------------
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

    # Fixed to the head end of section_0. The pad top is at section-local
    # z = PAD_T; the horseshoe tube centerline sits just above it.
    model.articulation(
        "section_0_to_head",
        ArticulationType.FIXED,
        parent=section_parts[0],
        child=head,
        origin=Origin(xyz=(BACK_LEN + HEAD_LEN * 0.30, 0.0, PAD_T + HS_TUBE_R)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    seat = object_model.get_part("seat")
    section_0 = object_model.get_part("section_0")
    head = object_model.get_part("head")

    seat_to_section_0 = object_model.get_articulation("seat_to_section_0")

    # The grey column top is seated up into the underside of the steel seat deck
    # to form a solid pedestal mount. That small embedment is intentional.
    ctx.allow_overlap(
        base,
        seat,
        elem_a="column_post",
        elem_b="seat_deck",
        reason="Column top is seated into the seat deck underside as the pedestal mount.",
    )
    ctx.expect_overlap(
        base,
        seat,
        axes="xy",
        elem_a="column_post",
        elem_b="seat_deck",
        min_overlap=0.10,
        name="column carries the seat deck",
    )

    # --- Loop-emitted section: joint type / axis claims --------------------
    ctx.check(
        "section_0 joint is revolute",
        seat_to_section_0.articulation_type == ArticulationType.REVOLUTE,
        details=str(seat_to_section_0.articulation_type),
    )
    ctx.check(
        "section_0 hinge axis is Y",
        abs(seat_to_section_0.axis[1]) > 0.9
        and abs(seat_to_section_0.axis[0]) < 1e-6
        and abs(seat_to_section_0.axis[2]) < 1e-6,
        details=str(seat_to_section_0.axis),
    )

    # --- Structural placement at rest ------------------------------------
    # The padded top is well above the floor: the seat cushion top sits at table
    # height.
    seat_pad_aabb = ctx.part_element_world_aabb(seat, elem="seat_pad")
    ctx.check(
        "seat cushion top at table height",
        seat_pad_aabb is not None and abs(seat_pad_aabb[1][2] - TABLE_TOP_Z) < 0.03,
        details=str(seat_pad_aabb),
    )

    # Base reaches the floor (cross-foot at z=0).
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base sits on floor",
        base_aabb is not None and base_aabb[0][2] < 0.01,
        details=str(base_aabb),
    )

    # Head support is at the head (+X) end, above section_0 pad.
    head_aabb = ctx.part_world_aabb(head)
    ctx.check(
        "head support at head end",
        head_aabb is not None and head_aabb[0][0] > SEAT_LEN / 2.0,
        details=str(head_aabb),
    )

    # Section_0 pad meets seat head edge at rest (continuous flat table).
    ctx.expect_gap(
        section_0,
        seat,
        axis="x",
        max_gap=0.02,
        max_penetration=0.02,
        positive_elem="section_0_pad",
        negative_elem="seat_pad",
        name="section_0 pad meets seat at rest",
    )

    # --- Decisive pose check: section_0 actually tilts up -------------------
    sec0_tip_rest = ctx.part_element_world_aabb(section_0, elem="section_0_pad")
    with ctx.pose({seat_to_section_0: 0.90}):
        sec0_tip_up = ctx.part_element_world_aabb(section_0, elem="section_0_pad")
    ctx.check(
        "section_0 tilts up when actuated",
        sec0_tip_rest is not None
        and sec0_tip_up is not None
        and sec0_tip_up[1][2] > sec0_tip_rest[1][2] + 0.15,
        details=f"rest_maxz={sec0_tip_rest[1][2] if sec0_tip_rest else None}, "
        f"posed_maxz={sec0_tip_up[1][2] if sec0_tip_up else None}",
    )

    # Arm rails present on the seat (curved tubular accessories).
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

    # --- No leg section exists (fork variant claim) -------------------------
    part_names = [p.name for p in object_model.parts]
    ctx.check(
        "no separate leg section in this variant",
        "leg" not in part_names,
        details=f"parts={part_names}",
    )

    # --- Section_0 has the expected visuals from the shared helper -----------
    sec0_visual_names = [v.name for v in section_0.visuals]
    ctx.check(
        "section_0 has deck, rails, and pad from shared helper",
        "section_0_deck" in sec0_visual_names
        and "section_0_rails" in sec0_visual_names
        and "section_0_pad" in sec0_visual_names,
        details=str(sec0_visual_names),
    )

    return ctx.report()


object_model = build_object_model()
