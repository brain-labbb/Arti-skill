from __future__ import annotations

# Surgical / operating table — variant with side safety rails and IV pole.
#
# Identity (from reference image):
#   A pedestal operating table. A heavy beige cross-foot carries a grey central
#   column. The column supports a multi-section padded top: a fixed seat/center
#   pad, a back/torso pad at the head end, a leg/foot pad at the foot end. Two
#   straight stainless safety rails run along both long deck edges and a
#   vertical IV pole rises from one corner of the seat frame. The dark grey
#   upholstered cushions sit in stainless side frames.
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
#   These are the two primary section joints. The safety rails and IV pole are
#   rigid accessories mounted on the seat frame.

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

# Padded top: full length ~1.95 m, split into three sections.
TOP_W = 0.500          # cushion width (Y)
PAD_T = 0.060          # cushion thickness (Z)
RAIL_T = 0.022         # stainless side rail thickness (square-ish)
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

# Seat steel deck thickness is needed for the column-top clearance below.
DECK_T = 0.030         # steel deck plate thickness (Z)

# Central column. It rises from well inside the foot (overlapping it for a solid
# structural joint) up into the seat deck underside (also overlapping it).
COL_BASE_Z = FOOT_H - 0.060         # start inside the foot block (solid overlap)
COL_TOP_Z = SEAT_DECK_Z - DECK_T + 0.018  # top reaches up into the seat deck
COL_W = 0.150                       # column cross-section (square-ish)
COL_TAPER = 0.190                   # wider near the base

# Safety rails (straight stainless bars along both long deck edges)
SAFETY_RAIL_LEN = SEAT_LEN * 0.92   # rail bar length (X)
SAFETY_RAIL_H = 0.200               # top of rail above deck surface (Z)
SAFETY_RAIL_BAR_R = 0.012           # rail bar radius
SAFETY_RAIL_POST_R = 0.010          # vertical support post radius
SAFETY_RAIL_POST_N = 3              # number of vertical posts per rail

# IV pole (vertical stainless tube from one corner of the seat frame)
IV_POLE_R = 0.014                   # pole tube radius
IV_POLE_H = 1.500                   # pole height above seat deck
IV_HOOK_R = 0.008                   # hook tube radius
IV_HOOK_LEN = 0.070                 # hook reach outward
IV_HOOK_N = 4                       # number of hooks at the top

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


def _safety_rail_shape() -> cq.Workplane:
    """One straight stainless safety rail: a horizontal bar supported by
    vertical posts, built in seat-local frame with the deck top at z=0.
    The rail bar sits at z=SAFETY_RAIL_H above the deck and runs along X.
    Posts connect the bar down to the deck surface (z=0)."""
    # Horizontal top bar — sweep a circle along X
    bar_z = SAFETY_RAIL_H - SAFETY_RAIL_BAR_R
    bar = (
        cq.Workplane("YZ")
        .workplane(offset=-SAFETY_RAIL_LEN / 2.0)
        .circle(SAFETY_RAIL_BAR_R)
        .extrude(SAFETY_RAIL_LEN)
    )
    bar = bar.translate((0.0, 0.0, bar_z))
    # Vertical support posts evenly spaced along the bar
    result = bar
    post_h = SAFETY_RAIL_H - SAFETY_RAIL_BAR_R
    xs = [
        -SAFETY_RAIL_LEN * 0.38,
        0.0,
        SAFETY_RAIL_LEN * 0.38,
    ][:SAFETY_RAIL_POST_N]
    for px in xs:
        post = (
            cq.Workplane("XY")
            .circle(SAFETY_RAIL_POST_R)
            .extrude(post_h)
            .translate((px, 0.0, 0.0))
        )
        result = result.union(post)
    return result


def _iv_pole_shape() -> cq.Workplane:
    """IV pole: a vertical stainless tube with curved hooks at the top, built in
    seat-local frame with the deck top at z=0. The pole rises along +Z from the
    deck surface. Hooks arc outward and downward at the top."""
    # Main vertical tube
    pole = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, IV_POLE_H / 2.0))
        .cylinder(IV_POLE_H, IV_POLE_R, centered=(True, True, True))
    )
    # Base mounting flange — a short wider collar at the bottom
    flange = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, 0.015))
        .cylinder(0.030, IV_POLE_R * 1.8, centered=(True, True, True))
    )
    result = pole.union(flange)

    # Curved hooks at the top, evenly distributed around the pole
    for i in range(IV_HOOK_N):
        angle = 2.0 * math.pi * i / IV_HOOK_N
        # Each hook: a short arc that curves outward and downward from the pole top
        hook_pts = []
        n_seg = 8
        for s in range(n_seg + 1):
            t = s / n_seg
            # Arc from vertical to ~60 degrees outward
            a = t * math.pi * 0.45
            r_out = IV_HOOK_LEN * math.sin(a)
            z_drop = -IV_HOOK_LEN * (1.0 - math.cos(a))
            hook_pts.append(cq.Vector(
                r_out * math.cos(angle),
                r_out * math.sin(angle),
                IV_POLE_H + z_drop,
            ))
        path = cq.Workplane(obj=cq.Edge.makeSpline(hook_pts)).wire()
        p0 = hook_pts[0]
        p1 = hook_pts[1]
        tangent = (p1 - p0).normalized()
        profile = (
            cq.Workplane(cq.Plane(origin=p0, normal=tangent))
            .circle(IV_HOOK_R)
        )
        hook_solid = profile.sweep(path, transition="round")
        result = result.union(hook_solid)

    # Small cap ball at the very top
    cap = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, IV_POLE_H + 0.012))
        .sphere(0.014)
    )
    result = result.union(cap)

    return result


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

    # Straight stainless safety rails along both long deck edges.
    # Mounted on the seat frame, one per side (+Y and -Y).
    safety_rail_mesh = mesh_from_cadquery(_safety_rail_shape(), "safety_rail")
    y_rail_offset = FRAME_W / 2.0 - RAIL_T / 2.0
    for i in range(2):
        sign = 1.0 if i == 0 else -1.0
        seat.visual(
            safety_rail_mesh,
            origin=Origin(xyz=(0.0, sign * y_rail_offset, -DECK_T)),
            material="tube",
            name=f"safety_rail_{i}",
        )

    # IV pole rising from the head-left corner of the seat frame.
    iv_x = SEAT_LEN / 2.0 - 0.030
    iv_y = FRAME_W / 2.0 - RAIL_T / 2.0
    seat.visual(
        mesh_from_cadquery(_iv_pole_shape(), "iv_pole"),
        origin=Origin(xyz=(iv_x, iv_y, -DECK_T)),
        material="tube",
        name="iv_pole",
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

    seat_to_back = object_model.get_articulation("seat_to_back")
    seat_to_leg = object_model.get_articulation("seat_to_leg")

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

    # --- Safety rails and IV pole --------------------------------------------
    # Two safety rails present on opposite sides of the seat, standing well above
    # the mattress surface.
    rail_0 = ctx.part_element_world_aabb(seat, elem="safety_rail_0")
    rail_1 = ctx.part_element_world_aabb(seat, elem="safety_rail_1")
    ctx.check(
        "two safety rails present and raised above deck",
        rail_0 is not None
        and rail_1 is not None
        and rail_0[1][2] > SEAT_DECK_Z + 0.12
        and rail_1[1][2] > SEAT_DECK_Z + 0.12,
        details=f"rail_0={rail_0}, rail_1={rail_1}",
    )
    # Rails are on opposite Y sides of the seat center.
    ctx.check(
        "safety rails on opposite sides of bed",
        rail_0 is not None
        and rail_1 is not None
        and rail_0[0][1] > 0.0
        and rail_1[1][1] < 0.0,
        details=f"rail_0_y_range=[{rail_0[0][1]:.3f}, {rail_0[1][1]:.3f}], "
        f"rail_1_y_range=[{rail_1[0][1]:.3f}, {rail_1[1][1]:.3f}]",
    )

    # IV pole present, rising tall from the seat frame corner.
    iv_aabb = ctx.part_element_world_aabb(seat, elem="iv_pole")
    ctx.check(
        "IV pole present and tall",
        iv_aabb is not None
        and iv_aabb[1][2] > SEAT_DECK_Z + IV_POLE_H * 0.85,
        details=str(iv_aabb),
    )

    # No horseshoe head support part exists.
    head_parts = [p for p in object_model.parts if p.name == "head"]
    ctx.check(
        "no horseshoe head support part",
        len(head_parts) == 0,
        details=f"found {len(head_parts)} head parts",
    )

    return ctx.report()


object_model = build_object_model()
