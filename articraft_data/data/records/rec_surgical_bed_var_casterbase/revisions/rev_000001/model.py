from __future__ import annotations

# Surgical / operating table — wheeled trolley base variant.
#
# Identity:
#   A hospital surgical bed on a wheeled trolley base. A low rectangular steel
#   chassis carries four swivel casters at the corners and a central column
#   rising to the multi-section padded top. The top has: a fixed seat/center
#   pad, a back/torso pad hinged at the head end (REVOLUTE, tilts up), a
#   leg/foot pad hinged at the foot end (REVOLUTE, folds down), a horseshoe
#   head support, and two curved tubular arm-support rails.
#
# Coordinate convention:
#   +Z up. Long axis along +/-X (head +X, foot -X). +Y patient left.
#   Floor at z=0. Casters touch floor; chassis rides above.
#
# Articulation:
#   - LEG/FOOT section: REVOLUTE at seat foot edge, folds DOWN.
#   - BACK/TORSO section: REVOLUTE at seat head edge, tilts UP.
#   - Four casters: FIXED to chassis at corners.

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

# Padded top (identical to parent)
TOP_W = 0.500          # cushion width (Y)
PAD_T = 0.060          # cushion thickness (Z)
RAIL_T = 0.022         # stainless side rail thickness
RAIL_H = 0.075         # side rail height (Z)
FRAME_W = TOP_W + 2 * RAIL_T  # outer frame width including side rails

SEAT_LEN = 0.420       # fixed center/seat section length (X)
BACK_LEN = 0.640       # back/torso section length (X)  (head end)
LEG_LEN = 0.560        # leg/foot section length (X)     (foot end)
HEAD_LEN = 0.260       # horseshoe head support reach (X)

# Top sits at this height (top surface of the seat cushion).
TABLE_TOP_Z = 0.980
SEAT_DECK_Z = TABLE_TOP_Z - PAD_T   # top of the steel seat deck under the pad
DECK_T = 0.030         # steel deck plate thickness (Z)

# --- Trolley base (replaces pedestal cross-foot) ---
CHASSIS_LEN = 0.720    # along X
CHASSIS_W = 0.520      # along Y
CHASSIS_H = 0.050      # chassis plate height (Z)

# Caster (100 mm hospital swivel caster)
CASTER_WHEEL_R = 0.050
CASTER_WHEEL_W = 0.030
CASTER_FORK_H = 0.080  # axle center to fork crown top
CASTER_FORK_T = 0.005  # fork leg plate thickness
CASTER_HOUSING_R = 0.025
CASTER_HOUSING_H = 0.035
CASTER_PLATE_SIZE = 0.058
CASTER_PLATE_T = 0.008

CASTER_TOTAL_H = (CASTER_WHEEL_R + CASTER_FORK_H
                  + CASTER_HOUSING_H + CASTER_PLATE_T)  # 0.173

CHASSIS_BOT_Z = CASTER_TOTAL_H                 # 0.173
CHASSIS_TOP_Z = CHASSIS_BOT_Z + CHASSIS_H      # 0.223

# Caster inset from chassis edges
CASTER_INSET_X = 0.065
CASTER_INSET_Y = 0.055

# Central column (from chassis to seat deck underside)
COL_W = 0.140
COL_BASE_Z = CHASSIS_TOP_Z - 0.015  # embeds 15 mm into chassis for connection
COL_TOP_Z = SEAT_DECK_Z - DECK_T + 0.018  # penetrates 18 mm into seat deck

# Head support (horseshoe)
HS_TUBE_R = 0.013
HS_OPENING = 0.150
HS_PAD_T = 0.045

# Arm support rails (curved tubular)
ARM_TUBE_R = 0.011

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

PAD_RGBA = (0.16, 0.16, 0.18, 1.0)          # dark grey upholstery
STEEL_RGBA = (0.78, 0.80, 0.82, 1.0)        # brushed stainless steel
COL_RGBA = (0.55, 0.56, 0.58, 1.0)          # painted grey column
CHASSIS_RGBA = (0.36, 0.38, 0.42, 1.0)      # dark steel chassis
CASTER_METAL_RGBA = (0.72, 0.74, 0.76, 1.0) # zinc-plated caster steel
WHEEL_RGBA = (0.12, 0.12, 0.13, 1.0)        # dark rubber wheel
TUBE_RGBA = (0.80, 0.82, 0.84, 1.0)         # bright tubular accessory steel


# ---------------------------------------------------------------------------
# CadQuery geometry helpers
# ---------------------------------------------------------------------------

def _cushion_shape(length: float) -> cq.Workplane:
    """Padded cushion slab with rounded soft top. Section-local frame: deck top
    at z=0, cushion extends up to +PAD_T, X over [-length/2, +length/2]."""
    return (
        cq.Workplane("XY")
        .box(length, TOP_W, PAD_T, centered=(True, True, False))
        .edges("|X")
        .fillet(0.018)
        .edges(">Z")
        .fillet(0.012)
    )


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
    """Steel deck plate under a cushion section. Top at z=0, extends down to
    -DECK_T."""
    return (
        cq.Workplane("XY")
        .box(length, TOP_W, DECK_T, centered=(True, True, True))
        .translate((0.0, 0.0, -DECK_T / 2.0))
        .edges("|X")
        .fillet(0.004)
    )


def _chassis_shape() -> cq.Workplane:
    """Low rectangular trolley chassis plate with a shallow perimeter lip on top
    and mounting bosses at the caster corners. Built in world-Z frame: plate
    from z=CHASSIS_BOT_Z to z=CHASSIS_TOP_Z."""
    plate = (
        cq.Workplane("XY")
        .box(CHASSIS_LEN, CHASSIS_W, CHASSIS_H, centered=(True, True, False))
        .translate((0.0, 0.0, CHASSIS_BOT_Z))
        .edges("|Z")
        .fillet(0.018)
        .edges(">Z")
        .fillet(0.004)
    )
    return plate


def _column_shape() -> cq.Workplane:
    """Tapered grey column from chassis to seat deck underside. Authored at
    final world Z."""
    h = COL_TOP_Z - COL_BASE_Z
    col_bot_w = COL_W + 0.030
    col = (
        cq.Workplane("XY")
        .workplane(offset=COL_BASE_Z)
        .rect(col_bot_w, col_bot_w)
        .workplane(offset=h * 0.35)
        .rect(COL_W, COL_W)
        .workplane(offset=h * 0.65)
        .rect(COL_W, COL_W)
        .loft(combine=True)
    )
    # Decorative collar near the top
    collar = (
        cq.Workplane("XY")
        .box(COL_W + 0.020, COL_W + 0.020, 0.025, centered=(True, True, False))
        .translate((0.0, 0.0, COL_TOP_Z - 0.050))
        .edges("|Z")
        .fillet(0.005)
    )
    return col.union(collar)


def _caster_metal_shape() -> cq.Workplane:
    """Swivel caster metal parts in caster-local frame (mounting plate top at
    z=0, everything extends below). Includes: mounting plate, swivel housing,
    fork crown, two fork legs, and an axle pin that bridges the fork through the
    wheel hub for metal↔wheel connectivity."""
    z_plate_bot = -CASTER_PLATE_T
    z_housing_bot = z_plate_bot - CASTER_HOUSING_H
    z_axle = z_housing_bot - CASTER_FORK_H

    # Mounting plate (square, bolted to chassis underside)
    plate = (
        cq.Workplane("XY")
        .box(CASTER_PLATE_SIZE, CASTER_PLATE_SIZE, CASTER_PLATE_T,
             centered=(True, True, False))
        .translate((0.0, 0.0, z_plate_bot))
        .edges("|Z")
        .fillet(0.003)
    )

    # Swivel housing / raceway cylinder
    housing = (
        cq.Workplane("XY")
        .circle(CASTER_HOUSING_R)
        .extrude(CASTER_HOUSING_H + 0.003)
        .translate((0.0, 0.0, z_housing_bot))
    )

    # Fork crown block
    crown_w = CASTER_WHEEL_W + 2.0 * (CASTER_FORK_T + 0.006)
    crown = (
        cq.Workplane("XY")
        .box(crown_w, 0.040, 0.014, centered=(True, True, False))
        .translate((0.0, 0.0, z_housing_bot - 0.011))
        .edges("|Z")
        .fillet(0.004)
    )

    # Fork legs: two plates straddling the wheel along Y
    fork_y = CASTER_WHEEL_W / 2.0 + CASTER_FORK_T / 2.0 + 0.002
    leg_top = z_housing_bot - 0.009
    leg_bot = z_axle - 0.006
    leg_h = leg_top - leg_bot
    leg_cz = (leg_top + leg_bot) / 2.0
    leg_w = 0.034

    leg_r = (
        cq.Workplane("XY")
        .box(leg_w, CASTER_FORK_T, leg_h, centered=(True, True, True))
        .translate((0.0, fork_y, leg_cz))
        .edges("|Y")
        .fillet(0.002)
    )
    leg_l = (
        cq.Workplane("XY")
        .box(leg_w, CASTER_FORK_T, leg_h, centered=(True, True, True))
        .translate((0.0, -fork_y, leg_cz))
        .edges("|Y")
        .fillet(0.002)
    )

    # Axle pin through fork legs and wheel hub (ensures metal↔wheel connectivity)
    axle_half = fork_y + CASTER_FORK_T / 2.0 + 0.003
    axle = (
        cq.Workplane("XY")
        .circle(0.005)
        .extrude(axle_half * 2.0)
        .translate((0.0, 0.0, -axle_half))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((0.0, 0.0, z_axle))
    )

    return plate.union(housing).union(crown).union(leg_r).union(leg_l).union(axle)


def _caster_wheel_shape() -> cq.Workplane:
    """Caster rubber wheel in caster-local frame. Cylinder along Y (rolling
    axis) centered at the axle position below the mounting plate."""
    z_axle = -(CASTER_PLATE_T + CASTER_HOUSING_H + CASTER_FORK_H)

    wheel = (
        cq.Workplane("XY")
        .circle(CASTER_WHEEL_R)
        .extrude(CASTER_WHEEL_W)
        .translate((0.0, 0.0, -CASTER_WHEEL_W / 2.0))
        .edges()
        .fillet(0.005)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((0.0, 0.0, z_axle))
    )
    return wheel


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
    """Horseshoe (U-shaped) head support. Head-local frame: open mouth faces +X
    (toward back pad), closed curve faces -X. Returns (tube_solid, pad_solid)."""
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
        .box(leg * 0.50, 2.0 * r - 2.0 * HS_TUBE_R, HS_PAD_T,
             centered=(True, True, False))
        .translate((-leg * 0.10, 0.0, HS_TUBE_R - HS_PAD_T / 2.0))
        .edges("|X")
        .fillet(0.012)
    )
    return tube, pad


def _arm_rail_shape(sign: float) -> cq.Workplane:
    """One curved tubular arm-support rail. Seat-local frame; sign = +1 (patient
    left) or -1 (right)."""
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
    model.material("chassis", rgba=CHASSIS_RGBA)
    model.material("caster_metal", rgba=CASTER_METAL_RGBA)
    model.material("wheel", rgba=WHEEL_RGBA)
    model.material("tube", rgba=TUBE_RGBA)

    # ---- Root: trolley chassis + column ------------------------------------
    chassis = model.part("chassis")
    chassis.visual(
        mesh_from_cadquery(_chassis_shape(), "chassis_frame"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="chassis",
        name="chassis_frame",
    )
    chassis.visual(
        mesh_from_cadquery(_column_shape(), "column_post"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="column",
        name="column_post",
    )

    # ---- Four swivel casters at chassis corners (FIXED) --------------------
    caster_positions = [
        (+CHASSIS_LEN / 2.0 - CASTER_INSET_X, +CHASSIS_W / 2.0 - CASTER_INSET_Y),
        (+CHASSIS_LEN / 2.0 - CASTER_INSET_X, -CHASSIS_W / 2.0 + CASTER_INSET_Y),
        (-CHASSIS_LEN / 2.0 + CASTER_INSET_X, +CHASSIS_W / 2.0 - CASTER_INSET_Y),
        (-CHASSIS_LEN / 2.0 + CASTER_INSET_X, -CHASSIS_W / 2.0 + CASTER_INSET_Y),
    ]

    metal_cq = _caster_metal_shape()
    wheel_cq = _caster_wheel_shape()

    for i in range(4):
        cx, cy = caster_positions[i]
        caster = model.part(f"caster_{i}")
        caster.visual(
            mesh_from_cadquery(metal_cq, "caster_metal"),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material="caster_metal",
            name="caster_metal",
        )
        caster.visual(
            mesh_from_cadquery(wheel_cq, "caster_wheel"),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material="wheel",
            name="caster_wheel",
        )
        model.articulation(
            f"chassis_to_caster_{i}",
            ArticulationType.FIXED,
            parent=chassis,
            child=caster,
            origin=Origin(xyz=(cx, cy, CHASSIS_BOT_Z)),
        )

    # ---- Seat / center section (fixed to chassis via column) ---------------
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

    model.articulation(
        "chassis_to_seat",
        ArticulationType.FIXED,
        parent=chassis,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, SEAT_DECK_Z)),
    )

    # ---- Back / torso section (head end, tilts up) -------------------------
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

    model.articulation(
        "seat_to_back",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=back,
        origin=Origin(xyz=(SEAT_LEN / 2.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=200.0, velocity=0.5, lower=0.0, upper=1.05),
    )

    # ---- Head support (horseshoe), mounted on back section -----------------
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

    model.articulation(
        "back_to_head",
        ArticulationType.FIXED,
        parent=back,
        child=head,
        origin=Origin(xyz=(BACK_LEN + HEAD_LEN * 0.30, 0.0, PAD_T + HS_TUBE_R)),
    )

    # ---- Leg / foot section (foot end, folds down) -------------------------
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

    chassis = object_model.get_part("chassis")
    seat = object_model.get_part("seat")
    back = object_model.get_part("back")
    leg = object_model.get_part("leg")
    head = object_model.get_part("head")

    seat_to_back = object_model.get_articulation("seat_to_back")
    seat_to_leg = object_model.get_articulation("seat_to_leg")

    # Column top is seated into the seat deck underside (intentional embed).
    ctx.allow_overlap(
        chassis,
        seat,
        elem_a="column_post",
        elem_b="seat_deck",
        reason="Column top is seated into the seat deck underside as the trolley mount.",
    )
    ctx.expect_overlap(
        chassis,
        seat,
        axes="xy",
        elem_a="column_post",
        elem_b="seat_deck",
        min_overlap=0.10,
        name="column carries the seat deck",
    )

    # --- Caster checks -------------------------------------------------------
    for i in range(4):
        caster_part = object_model.get_part(f"caster_{i}")
        wheel_aabb = ctx.part_element_world_aabb(caster_part, elem="caster_wheel")
        ctx.check(
            f"caster_{i} wheel reaches floor",
            wheel_aabb is not None and wheel_aabb[0][2] < 0.005,
            details=str(wheel_aabb),
        )

    # Chassis is elevated above the floor by the casters.
    chassis_aabb = ctx.part_world_aabb(chassis)
    ctx.check(
        "chassis elevated above floor by casters",
        chassis_aabb is not None and chassis_aabb[0][2] > 0.10,
        details=str(chassis_aabb),
    )

    # --- Joint type / axis claims --------------------------------------------
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

    # --- Structural placement at rest ----------------------------------------
    seat_pad_aabb = ctx.part_element_world_aabb(seat, elem="seat_pad")
    ctx.check(
        "seat cushion top at table height",
        seat_pad_aabb is not None and abs(seat_pad_aabb[1][2] - TABLE_TOP_Z) < 0.03,
        details=str(seat_pad_aabb),
    )

    # Head support at the head (+X) end, above the back pad.
    head_aabb = ctx.part_world_aabb(head)
    ctx.check(
        "head support at head end",
        head_aabb is not None and head_aabb[0][0] > SEAT_LEN / 2.0,
        details=str(head_aabb),
    )

    # Sections line up at rest: continuous flat table.
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

    # --- Decisive pose checks ------------------------------------------------
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

    # Arm rails present on the seat.
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
