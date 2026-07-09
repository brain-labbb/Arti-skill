from __future__ import annotations

# Surgical / operating table — flat arm board variant.
#
# Identity:
#   A pedestal operating table. A heavy beige cross-foot carries a grey central
#   column. The column supports a multi-section padded top: a fixed seat/center
#   pad, a back/torso pad at the head end, a leg/foot pad at the foot end, plus
#   a simple flat head pad and two flat padded arm boards on short swing
#   brackets at the seat sides. The dark grey upholstered cushions sit in
#   stainless side frames.
#
# Coordinate convention:
#   +Z is up. The table long axis runs along +/-X (head end toward +X, foot end
#   toward -X). +Y is the patient's left. The column rises along +Z from the
#   cross-foot at the floor (z=0).
#
# Articulation:
#   - The LEG/FOOT section is hinged at the seat front edge and folds DOWN
#     -> REVOLUTE.
#   - The BACK/TORSO section is hinged at the seat head edge and tilts UP
#     -> REVOLUTE.
#   The flat head pad and arm boards are rigid accessories mounted on the back
#   section / seat frame (no separate joints).

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

# Padded top: full length ~1.82 m, split into three sections + head pad.
TOP_W = 0.500          # cushion width (Y)
PAD_T = 0.060          # cushion thickness (Z)
RAIL_T = 0.022         # stainless side rail thickness
RAIL_H = 0.075         # side rail height (Z)
FRAME_W = TOP_W + 2 * RAIL_T  # outer frame width including side rails

SEAT_LEN = 0.420       # fixed center/seat section length (X)
BACK_LEN = 0.640       # back/torso section length (X)  (head end)
LEG_LEN = 0.560        # leg/foot section length (X)     (foot end)

# Top sits at this height (top surface of the seat cushion).
TABLE_TOP_Z = 0.980
SEAT_DECK_Z = TABLE_TOP_Z - PAD_T   # top of the steel seat deck under the pad

# Cross-foot (beige base on the floor)
FOOT_LEN = 0.820       # along X
FOOT_W = 0.560         # along Y
FOOT_H = 0.110         # height of the beige block (Z), sits on floor at z=0

# Seat steel deck thickness
DECK_T = 0.030         # steel deck plate thickness (Z)

# Central column
COL_BASE_Z = FOOT_H - 0.060         # start inside the foot block
COL_TOP_Z = SEAT_DECK_Z - DECK_T + 0.018  # top reaches up into the seat deck
COL_W = 0.150                       # column cross-section
COL_TAPER = 0.190                   # wider near the base

# Flat head pad (replaces horseshoe)
HEAD_PAD_LEN = 0.220   # along X
HEAD_PAD_W = 0.240     # along Y

# Flat padded arm boards on swing brackets (replace curved tubular rails)
ARM_BOARD_LEN = 0.250  # pad length along X
ARM_BOARD_W = 0.120    # pad width along Y
BRACKET_REACH = 0.090  # how far the bracket extends outward from the rail
BRACKET_W = 0.050      # bracket arm width (along X)
BRACKET_T = 0.010      # bracket arm thickness (along Z)
BRACKET_POST_H = 0.035 # vertical post height

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

PAD_RGBA = (0.16, 0.16, 0.18, 1.0)       # dark grey upholstery
STEEL_RGBA = (0.78, 0.80, 0.82, 1.0)     # brushed stainless steel
COL_RGBA = (0.55, 0.56, 0.58, 1.0)       # painted grey column
FOOT_RGBA = (0.74, 0.66, 0.52, 1.0)      # beige / tan base


# ---------------------------------------------------------------------------
# CadQuery helpers
# ---------------------------------------------------------------------------

def _cushion_shape(length: float, width: float = TOP_W) -> cq.Workplane:
    """A padded cushion: a slab with a rounded soft top, built in section-local
    frame with the deck top at z=0, cushion extends up to +PAD_T, X over
    [-length/2, +length/2], Y over [-width/2, +width/2]."""
    f_side = min(0.018, width * 0.14, PAD_T * 0.45)
    f_top = min(0.012, width * 0.08, length * 0.08)
    pad = (
        cq.Workplane("XY")
        .box(length, width, PAD_T, centered=(True, True, False))
        .edges("|X")
        .fillet(f_side)
        .edges(">Z")
        .fillet(f_top)
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
    to z=COL_TOP_Z. Wider near the bottom."""
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
    """Beige cross-foot base, sitting on the floor (z=0..FOOT_H)."""
    foot = (
        cq.Workplane("XY")
        .box(FOOT_LEN, FOOT_W, FOOT_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.030)
    )
    return foot


def _arm_board_bracket(sign: float) -> cq.Workplane:
    """Short swing bracket arm for one arm board. Built in seat-local frame
    with deck top at z=0. sign=+1 for patient left (+Y), -1 for right (-Y).
    The bracket extends outward from the side rail with a horizontal arm and
    a vertical post at the outer end."""
    y_rail_outer = sign * (FRAME_W / 2.0)
    # Horizontal arm: starts inside the rail for connectivity, extends outward.
    y_start = y_rail_outer - sign * RAIL_T
    y_end = y_rail_outer + sign * BRACKET_REACH
    y_center = (y_start + y_end) / 2.0
    arm_len = abs(y_end - y_start)
    arm = (
        cq.Workplane("XY")
        .box(BRACKET_W, arm_len, BRACKET_T, centered=(True, True, False))
        .translate((0.0, y_center, 0.0))
        .edges("|Z").fillet(0.004)
    )
    # Vertical post at outer end
    post = (
        cq.Workplane("XY")
        .box(BRACKET_W, BRACKET_W, BRACKET_POST_H, centered=(True, True, False))
        .translate((0.0, y_end, 0.0))
        .edges("|Z").fillet(0.003)
    )
    # Small gusset rib connecting post base to arm for rigidity look
    gusset = (
        cq.Workplane("XY")
        .box(BRACKET_W * 0.6, BRACKET_REACH * 0.35, BRACKET_T * 1.5,
             centered=(True, True, False))
        .translate((0.0, y_end - sign * BRACKET_REACH * 0.18, 0.0))
        .edges("|Z").fillet(0.003)
    )
    return arm.union(post).union(gusset)


def _arm_board_pad(sign: float) -> cq.Workplane:
    """Flat padded arm board that sits on top of the swing bracket, in
    seat-local frame (deck top at z=0). The pad overlaps with the bracket
    post for visual connectivity."""
    y_rail_outer = sign * (FRAME_W / 2.0)
    pad_y_center = y_rail_outer + sign * (ARM_BOARD_W / 2.0 + 0.010)
    # Start pad slightly below post top for overlap connectivity.
    pad_z_base = BRACKET_POST_H - 0.005
    pad = (
        cq.Workplane("XY")
        .box(ARM_BOARD_LEN, ARM_BOARD_W, PAD_T, centered=(True, True, False))
        .translate((0.0, pad_y_center, pad_z_base))
        .edges("|X").fillet(0.010)
        .edges(">Z").fillet(0.006)
    )
    return pad


def _head_pad_deck_shape() -> cq.Workplane:
    """Steel deck plate under the head pad plus a connecting bridge that
    reaches back toward the back section. Built in local frame centered at
    x=0, deck top at z=0."""
    deck = (
        cq.Workplane("XY")
        .box(HEAD_PAD_LEN, HEAD_PAD_W, DECK_T, centered=(True, True, True))
        .translate((0.0, 0.0, -DECK_T / 2.0))
        .edges("|X").fillet(0.004)
    )
    # Connecting bridge: extends from the deck toward -X to overlap with the
    # back section's deck/rails.
    bridge_len = 0.050
    bridge_w = min(HEAD_PAD_W, TOP_W * 0.40)
    bridge = (
        cq.Workplane("XY")
        .box(bridge_len, bridge_w, DECK_T, centered=(True, True, True))
        .translate((-HEAD_PAD_LEN / 2.0 - bridge_len / 2.0, 0.0, -DECK_T / 2.0))
        .edges("|Z").fillet(0.003)
    )
    # Small mounting boss that protrudes above z=0 for pad connectivity.
    boss = (
        cq.Workplane("XY")
        .box(HEAD_PAD_LEN * 0.50, HEAD_PAD_W * 0.50, 0.008,
             centered=(True, True, False))
        .edges("|Z").fillet(0.004)
    )
    return deck.union(bridge).union(boss)


def _head_pad_cushion_shape() -> cq.Workplane:
    """Simple flat head pad cushion, centered at origin with deck top at z=0."""
    return _cushion_shape(HEAD_PAD_LEN, HEAD_PAD_W)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="surgical_bed")

    model.material("pad", rgba=PAD_RGBA)
    model.material("steel", rgba=STEEL_RGBA)
    model.material("column", rgba=COL_RGBA)
    model.material("foot", rgba=FOOT_RGBA)

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

    # ---- Seat / center section (fixed to column) --------------------------
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

    # Two flat padded arm boards on swing brackets, one per side.
    for i, sign in enumerate((+1.0, -1.0)):
        seat.visual(
            mesh_from_cadquery(_arm_board_bracket(sign), f"arm_bracket_{i}"),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material="steel",
            name=f"arm_bracket_{i}",
        )
        seat.visual(
            mesh_from_cadquery(_arm_board_pad(sign), f"arm_pad_{i}"),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material="pad",
            name=f"arm_pad_{i}",
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

    # Flat head pad inlined on the back section (no separate part).
    head_x_center = BACK_LEN + HEAD_PAD_LEN / 2.0
    back.visual(
        mesh_from_cadquery(_head_pad_deck_shape(), "head_pad_deck"),
        origin=Origin(xyz=(head_x_center, 0.0, 0.0)),
        material="steel",
        name="head_pad_deck",
    )
    back.visual(
        mesh_from_cadquery(_head_pad_cushion_shape(), "head_pad"),
        origin=Origin(xyz=(head_x_center, 0.0, 0.0)),
        material="pad",
        name="head_pad",
    )

    # Hinge at the seat head edge (world x = +SEAT_LEN/2). The pad extends
    # along +X from the hinge, so axis -Y lifts the free (head) edge upward
    # for positive q (back-raise).
    model.articulation(
        "seat_to_back",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=back,
        origin=Origin(xyz=(SEAT_LEN / 2.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=200.0, velocity=0.5, lower=0.0, upper=1.05),
    )

    # ---- Leg / foot section (foot end, folds down) ------------------------
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

    # Hinge at the seat foot edge (world x = -SEAT_LEN/2). The pad extends
    # along -X from the hinge, so axis -Y rotates the free (foot) edge
    # DOWNWARD for positive q (the leg section folds down).
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

    seat_to_back = object_model.get_articulation("seat_to_back")
    seat_to_leg = object_model.get_articulation("seat_to_leg")

    # The grey column top is seated up into the underside of the steel seat
    # deck to form a solid pedestal mount.
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
    seat_pad_aabb = ctx.part_element_world_aabb(seat, elem="seat_pad")
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

    # Flat head pad is at the head (+X) end of the back section, above deck.
    head_pad_aabb = ctx.part_element_world_aabb(back, elem="head_pad")
    ctx.check(
        "flat head pad at head end",
        head_pad_aabb is not None and head_pad_aabb[0][0] > SEAT_LEN / 2.0,
        details=str(head_pad_aabb),
    )

    # Sections line up: at rest the back pad meets the seat head edge and the
    # leg pad meets the seat foot edge (continuous flat table).
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

    # --- Arm boards: two flat padded boards on swing brackets --------------
    arm_pad_0 = ctx.part_element_world_aabb(seat, elem="arm_pad_0")
    arm_pad_1 = ctx.part_element_world_aabb(seat, elem="arm_pad_1")
    ctx.check(
        "two flat arm pads present and raised above deck",
        arm_pad_0 is not None
        and arm_pad_1 is not None
        and arm_pad_0[1][2] > SEAT_DECK_Z + 0.02
        and arm_pad_1[1][2] > SEAT_DECK_Z + 0.02,
        details=f"arm_pad_0={arm_pad_0}, arm_pad_1={arm_pad_1}",
    )
    # Arm pads extend beyond the seat side rails (outboard in Y).
    seat_aabb = ctx.part_world_aabb(seat)
    ctx.check(
        "arm pads extend outboard of the seat frame",
        arm_pad_0 is not None
        and arm_pad_1 is not None
        and seat_aabb is not None
        and (arm_pad_0[1][1] > FRAME_W / 2.0 + 0.03
             or arm_pad_0[0][1] < -(FRAME_W / 2.0 + 0.03))
        and (arm_pad_1[1][1] > FRAME_W / 2.0 + 0.03
             or arm_pad_1[0][1] < -(FRAME_W / 2.0 + 0.03)),
        details=f"frame_half={FRAME_W / 2.0}, pad_0={arm_pad_0}, pad_1={arm_pad_1}",
    )

    # Head pad is a flat cushion (not a horseshoe): it should be roughly as
    # wide as specified and centered on Y.
    ctx.check(
        "head pad is a flat rectangular cushion",
        head_pad_aabb is not None
        and (head_pad_aabb[1][1] - head_pad_aabb[0][1]) > HEAD_PAD_W * 0.70
        and (head_pad_aabb[1][0] - head_pad_aabb[0][0]) > HEAD_PAD_LEN * 0.70,
        details=str(head_pad_aabb),
    )

    return ctx.report()


object_model = build_object_model()
