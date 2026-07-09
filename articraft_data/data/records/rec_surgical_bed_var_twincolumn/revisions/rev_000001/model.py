from __future__ import annotations

# Surgical / operating table.
#
# Identity (from reference image):
#   A twin-column H-frame operating table. A heavy beige floor beam carries two
#   grey vertical columns bridged by a cross member. The twin columns support a
#   multi-section padded top: a fixed seat/center pad, a back/torso pad at the
#   head end, a leg/foot pad at the foot end, plus a horseshoe head support and
#   two curved tubular arm-support rails. The dark grey upholstered cushions sit
#   in stainless side frames.
#
# Coordinate convention:
#   +Z is up. The table long axis runs along +/-X (head end toward +X, foot end
#   toward -X). +Y is the patient's left. The column rises along +Z from the
#   cross-foot at the floor (z=0).
#
# Articulation (what actually moves, reasoned from the image):
#   - The LEG/FOOT section is hinged at the seat front edge and folds DOWN. In
#     the reference photo it is clearly broken downward at an angle -> REVOLUTE.
#   - The BACK/TORSO section is hinged at the seat head edge and tilts UP
#     (Trendelenburg / back-raise) -> REVOLUTE. Operating tables articulate both.
#   These are the two primary section joints. The head support and arm rails are
#   rigid accessories mounted on the back section / seat frame.

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

# Padded top: full length ~1.95 m, split into three sections + head support.
TOP_W = 0.500          # cushion width (Y)
PAD_T = 0.060          # cushion thickness (Z)
RAIL_T = 0.022         # stainless side rail thickness (square-ish)
RAIL_H = 0.075         # side rail height (Z)
FRAME_W = TOP_W + 2 * RAIL_T  # outer frame width including side rails

SEAT_LEN = 0.420       # fixed center/seat section length (X)
BACK_LEN = 0.640       # back/torso section length (X)  (head end)
LEG_LEN = 0.560        # leg/foot section length (X)     (foot end)
HEAD_LEN = 0.260       # horseshoe head support reach (X)

# Top sits at this height (top surface of the seat cushion).
TABLE_TOP_Z = 0.980
SEAT_DECK_Z = TABLE_TOP_Z - PAD_T   # top of the steel seat deck under the pad

# H-frame twin-column base (replaces single pedestal).
# A shared beige floor beam runs along Y; two grey columns rise from it, bridged
# by a horizontal cross member, both meeting the underside of the seat deck.
BEAM_LEN = 0.720       # floor beam length along Y
BEAM_W = 0.180         # floor beam width along X
BEAM_H = 0.100         # floor beam height (Z), sits on floor at z=0

# Seat steel deck thickness is needed for the column-top clearance below.
DECK_T = 0.030         # steel deck plate thickness (Z)

# Twin columns. Each rises from inside the beam (structural overlap) up into
# the seat deck underside (also overlapping for a solid mount).
COL_SIZE = 0.105       # column cross-section (square)
COL_SPACING = 0.460    # center-to-center distance between columns (Y)
COL_BASE_Z = BEAM_H - 0.040   # start inside the beam (solid overlap)
COL_TOP_Z = SEAT_DECK_Z - DECK_T + 0.018  # top reaches into the seat deck
N_COLUMNS = 2

# Cross member bridges the two columns at roughly 38% height.
CROSS_Z = 0.400        # center height of cross member (Z)
CROSS_H = 0.055        # cross member vertical size (Z)
CROSS_D = 0.075        # cross member depth along X

# Head support (horseshoe)
HS_TUBE_R = 0.013
HS_OPENING = 0.150     # gap of the horseshoe (Y)
HS_PAD_T = 0.045

# Arm support rails (curved tubular)
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


def _beam_shape() -> cq.Workplane:
    """Beige floor beam sitting on the floor (z=0..BEAM_H). A wide rectangular
    block running along Y with softened vertical corners, reading as the heavy
    stabilizing base that carries both columns."""
    beam = (
        cq.Workplane("XY")
        .box(BEAM_W, BEAM_LEN, BEAM_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.022)
        .edges(">Z")
        .fillet(0.006)
    )
    return beam


def _column_shape_hframe() -> cq.Workplane:
    """One grey vertical column in world-Z frame. Tapered from a wider base to a
    narrower top, built at its final world Z so it overlaps the beam at the
    bottom and the seat deck at the top. Authored centered at (x=0, y=0) so
    the visual origin translates it to the correct Y position."""
    h = COL_TOP_Z - COL_BASE_Z
    base_size = COL_SIZE * 1.18
    top_size = COL_SIZE
    col = (
        cq.Workplane("XY")
        .workplane(offset=COL_BASE_Z)
        .rect(base_size, base_size)
        .workplane(offset=h * 0.40)
        .rect(top_size, top_size)
        .workplane(offset=h * 0.60)
        .rect(top_size, top_size)
        .loft(combine=True)
    )
    # Decorative recessed collar near the top.
    collar = (
        cq.Workplane("XY")
        .box(top_size + 0.016, top_size + 0.016, 0.025, centered=(True, True, False))
        .translate((0.0, 0.0, COL_TOP_Z - 0.050))
        .edges("|Z")
        .fillet(0.005)
    )
    return col.union(collar)


def _cross_member_shape() -> cq.Workplane:
    """Horizontal cross member bridging the two columns at CROSS_Z. Runs along Y
    between the column centerlines. Authored at world coordinates."""
    span = COL_SPACING + COL_SIZE * 0.4  # extend slightly into each column
    member = (
        cq.Workplane("XY")
        .box(CROSS_D, span, CROSS_H, centered=(True, True, False))
        .translate((0.0, 0.0, CROSS_Z - CROSS_H / 2.0))
        .edges("|Y")
        .fillet(0.008)
        .edges(">Z")
        .fillet(0.004)
    )
    return member


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

    # ---- Root: H-frame twin-column base (fixed support) -------------------
    # A shared beige floor beam carries two grey vertical columns bridged by a
    # cross member. Column bases sit inside the beam (structural overlap) and
    # column tops reach into the seat deck underside (also overlapping).
    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_beam_shape(), "foot_beam"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="foot",
        name="foot_beam",
    )
    # Two grey columns placed symmetrically along Y via a loop.
    for i in range(N_COLUMNS):
        y_sign = -1.0 + 2.0 * i  # i=0 -> -1, i=1 -> +1
        y_pos = y_sign * (COL_SPACING / 2.0)
        base.visual(
            mesh_from_cadquery(_column_shape_hframe(), f"column_{i}"),
            origin=Origin(xyz=(0.0, y_pos, 0.0)),
            material="column",
            name=f"column_{i}",
        )
    base.visual(
        mesh_from_cadquery(_cross_member_shape(), "cross_member"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="column",
        name="cross_member",
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

    # ---- Back / torso section (head end, tilts up) ------------------------
    # Back-local frame: hinge line at the seat head edge. Pad extends along +X
    # from x=0 (hinge) to x=+BACK_LEN. Deck top at z=0.
    back = model.part("back")
    back.visual(
        mesh_from_cadquery(_deck_shape(BACK_LEN), "back_deck"),
        origin=Origin(xyz=(BACK_LEN / 2.0, 0.0, 0.0)),
        material="steel",
        name="back_deck",
    )
    back.visual(
        mesh_from_cadquery(_side_rails_shape(BACK_LEN), "back_rails"),
        origin=Origin(xyz=(BACK_LEN / 2.0, 0.0, -DECK_T)),
        material="steel",
        name="back_rails",
    )
    back.visual(
        mesh_from_cadquery(_cushion_shape(BACK_LEN), "back_pad"),
        origin=Origin(xyz=(BACK_LEN / 2.0, 0.0, 0.0)),
        material="pad",
        name="back_pad",
    )

    # Hinge at the seat head edge (world x = +SEAT_LEN/2). The pad extends along
    # +X from the hinge, so axis -Y lifts the free (head) edge upward for
    # positive q (back-raise).
    model.articulation(
        "seat_to_back",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=back,
        origin=Origin(xyz=(SEAT_LEN / 2.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=200.0, velocity=0.5, lower=0.0, upper=1.05),
    )

    # ---- Head support (horseshoe), mounted on the back section ------------
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

    # Fixed to the head end of the back section. The back pad top is at
    # back-local z = PAD_T; the horseshoe tube centerline sits just above it.
    model.articulation(
        "back_to_head",
        ArticulationType.FIXED,
        parent=back,
        child=head,
        origin=Origin(xyz=(BACK_LEN + HEAD_LEN * 0.30, 0.0, PAD_T + HS_TUBE_R)),
    )

    # ---- Leg / foot section (foot end, folds down) ------------------------
    # Leg-local frame: hinge line at the seat foot edge. Pad extends along -X
    # from x=0 (hinge) to x=-LEG_LEN. Deck top at z=0.
    leg = model.part("leg")
    leg.visual(
        mesh_from_cadquery(_deck_shape(LEG_LEN), "leg_deck"),
        origin=Origin(xyz=(-LEG_LEN / 2.0, 0.0, 0.0)),
        material="steel",
        name="leg_deck",
    )
    leg.visual(
        mesh_from_cadquery(_side_rails_shape(LEG_LEN), "leg_rails"),
        origin=Origin(xyz=(-LEG_LEN / 2.0, 0.0, -DECK_T)),
        material="steel",
        name="leg_rails",
    )
    leg.visual(
        mesh_from_cadquery(_cushion_shape(LEG_LEN), "leg_pad"),
        origin=Origin(xyz=(-LEG_LEN / 2.0, 0.0, 0.0)),
        material="pad",
        name="leg_pad",
    )

    # Hinge at the seat foot edge (world x = -SEAT_LEN/2). The pad extends along
    # -X from the hinge, so axis -Y rotates the free (foot) edge DOWNWARD for
    # positive q (the leg section folds down, matching the reference photo).
    model.articulation(
        "seat_to_leg",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=leg,
        origin=Origin(xyz=(-SEAT_LEN / 2.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=200.0, velocity=0.5, lower=0.0, upper=1.40),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    seat = object_model.get_part("seat")
    back = object_model.get_part("back")
    leg = object_model.get_part("leg")
    head = object_model.get_part("head")

    seat_to_back = object_model.get_articulation("seat_to_back")
    seat_to_leg = object_model.get_articulation("seat_to_leg")

    # The two grey column tops are seated up into the underside of the steel seat
    # frame to form the H-frame mount. The columns penetrate both the deck plate
    # and the side rails to form a solid structural connection.
    for i in range(N_COLUMNS):
        ctx.allow_overlap(
            base,
            seat,
            elem_a=f"column_{i}",
            elem_b="seat_deck",
            reason="Column top is seated into the seat deck underside as part of the H-frame twin-column mount.",
        )
        ctx.allow_overlap(
            base,
            seat,
            elem_a=f"column_{i}",
            elem_b="seat_rails",
            reason="Column top penetrates the seat side rails to reach the seat frame underside as the H-frame support.",
        )
    ctx.expect_overlap(
        base,
        seat,
        axes="xy",
        elem_a="column_0",
        elem_b="seat_deck",
        min_overlap=0.05,
        name="column_0 carries the seat deck",
    )
    ctx.expect_overlap(
        base,
        seat,
        axes="xy",
        elem_a="column_1",
        elem_b="seat_deck",
        min_overlap=0.05,
        name="column_1 carries the seat deck",
    )

    # --- H-frame twin-column base structure -------------------------------
    # Verify the H-frame has two separate columns (not a single pedestal).
    col0_aabb = ctx.part_element_world_aabb(base, elem="column_0")
    col1_aabb = ctx.part_element_world_aabb(base, elem="column_1")
    ctx.check(
        "H-frame has two separate columns",
        col0_aabb is not None and col1_aabb is not None,
        details=f"col0={col0_aabb}, col1={col1_aabb}",
    )
    
    # Columns are symmetrically placed about the centerline (Y axis).
    if col0_aabb and col1_aabb:
        col0_center_y = (col0_aabb[0][1] + col0_aabb[1][1]) / 2.0
        col1_center_y = (col1_aabb[0][1] + col1_aabb[1][1]) / 2.0
        ctx.check(
            "columns are symmetrically positioned",
            abs(col0_center_y + col1_center_y) < 0.02,  # sum should be near zero
            details=f"col0_y={col0_center_y:.3f}, col1_y={col1_center_y:.3f}",
        )
        ctx.check(
            "columns are spaced apart",
            abs(col1_center_y - col0_center_y) > 0.30,  # at least 30cm apart
            details=f"spacing={abs(col1_center_y - col0_center_y):.3f}m",
        )
    
    # Cross member bridges the two columns.
    cross_aabb = ctx.part_element_world_aabb(base, elem="cross_member")
    ctx.check(
        "cross member present between columns",
        cross_aabb is not None and cross_aabb[1][1] - cross_aabb[0][1] > 0.30,
        details=f"cross_span={cross_aabb[1][1] - cross_aabb[0][1]:.3f}m" if cross_aabb else "missing",
    )
    
    # Columns reach the same height (both support the seat deck).
    if col0_aabb and col1_aabb:
        ctx.check(
            "both columns reach seat deck height",
            abs(col0_aabb[1][2] - col1_aabb[1][2]) < 0.02,
            details=f"col0_top={col0_aabb[1][2]:.3f}, col1_top={col1_aabb[1][2]:.3f}",
        )

    # --- Joint type / axis claims ----------------------------------------
    ctx.check(
        "back joint is revolute",
        seat_to_back.articulation_type == ArticulationType.REVOLUTE,
        details=str(seat_to_back.articulation_type),
    )
    ctx.check(
        "leg joint is revolute",
        seat_to_leg.articulation_type == ArticulationType.REVOLUTE,
        details=str(seat_to_leg.articulation_type),
    )
    ctx.check(
        "back hinge axis is Y",
        abs(seat_to_back.axis[1]) > 0.9
        and abs(seat_to_back.axis[0]) < 1e-6
        and abs(seat_to_back.axis[2]) < 1e-6,
        details=str(seat_to_back.axis),
    )
    ctx.check(
        "leg hinge axis is Y",
        abs(seat_to_leg.axis[1]) > 0.9
        and abs(seat_to_leg.axis[0]) < 1e-6
        and abs(seat_to_leg.axis[2]) < 1e-6,
        details=str(seat_to_leg.axis),
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

    # Head support is at the head (+X) end, above the back pad.
    head_aabb = ctx.part_world_aabb(head)
    ctx.check(
        "head support at head end",
        head_aabb is not None and head_aabb[0][0] > SEAT_LEN / 2.0,
        details=str(head_aabb),
    )

    # Sections line up: at rest the back pad meets the seat head edge and the leg
    # pad meets the seat foot edge (continuous flat table).
    ctx.expect_gap(
        back,
        seat,
        axis="x",
        max_gap=0.02,
        max_penetration=0.02,
        positive_elem="back_pad",
        negative_elem="seat_pad",
        name="back pad meets seat at rest",
    )
    ctx.expect_gap(
        seat,
        leg,
        axis="x",
        max_gap=0.02,
        max_penetration=0.02,
        positive_elem="seat_pad",
        negative_elem="leg_pad",
        name="leg pad meets seat at rest",
    )

    # --- Decisive pose checks: the sections actually move --------------------
    leg_rest = ctx.part_world_position(leg)
    leg_tip_rest = ctx.part_element_world_aabb(leg, elem="leg_pad")
    with ctx.pose({seat_to_leg: 1.30}):
        leg_tip_down = ctx.part_element_world_aabb(leg, elem="leg_pad")
    ctx.check(
        "leg folds down when actuated",
        leg_tip_rest is not None
        and leg_tip_down is not None
        and leg_tip_down[0][2] < leg_tip_rest[0][2] - 0.15,
        details=f"rest_minz={leg_tip_rest[0][2] if leg_tip_rest else None}, "
        f"posed_minz={leg_tip_down[0][2] if leg_tip_down else None}",
    )

    back_tip_rest = ctx.part_element_world_aabb(back, elem="back_pad")
    with ctx.pose({seat_to_back: 0.90}):
        back_tip_up = ctx.part_element_world_aabb(back, elem="back_pad")
    ctx.check(
        "back tilts up when actuated",
        back_tip_rest is not None
        and back_tip_up is not None
        and back_tip_up[1][2] > back_tip_rest[1][2] + 0.15,
        details=f"rest_maxz={back_tip_rest[1][2] if back_tip_rest else None}, "
        f"posed_maxz={back_tip_up[1][2] if back_tip_up else None}",
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

    return ctx.report()


object_model = build_object_model()
