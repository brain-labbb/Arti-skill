from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Single-hole basin faucet sibling, ~0.13 m tall, polished chrome.
# World frame: +Z up, deck at z = 0, spout projects toward +X at rest.
# Vertical cylindrical column body on an oval rubber gasket at deck.
# Spout swivels around the vertical body axis; top cap rotates for temperature.
# ---------------------------------------------------------------------------

# Base gasket (oval, sits flat on the deck).
GASKET_RX = 0.035   # semi-major axis (X/Y)
GASKET_RY = 0.028   # semi-minor axis
GASKET_H = 0.004

# Main body column (vertical).
COLUMN_R = 0.022
COLUMN_H = 0.100
COLUMN_Z0 = GASKET_H  # column sits on top of gasket

# Decorative groove ring near the upper third.
GROOVE_Z = COLUMN_Z0 + COLUMN_H * 0.72
GROOVE_H = 0.004
GROOVE_R = COLUMN_R + 0.001

# Upper neck (slightly stepped-in above groove).
NECK_R = 0.019
NECK_Z0 = GROOVE_Z + GROOVE_H
NECK_H = 0.020

# Spout mount height on the column.
SPOUT_MOUNT_Z = COLUMN_Z0 + COLUMN_H * 0.55

# Push cap on top of the column (temperature adjustment).
CAP_Z0 = COLUMN_Z0 + COLUMN_H  # cap sits at column top
CAP_R = 0.025
CAP_H = 0.012

# Spout dimensions.
SPOUT_R = 0.012  # outer tube radius
SPOUT_SHANK_LEN = 0.050  # straight section projecting forward
SPOUT_DROP = 0.025  # how far the end drops below the mount point
SPOUT_SWIVEL_LIMIT = math.radians(120.0)  # ±120 degrees

TURN_LIMIT = math.radians(60.0)


def _build_spout_shape() -> cq.Workplane:
    """Spout with gentle downward curve and real hollow outlet.

    Built in spout-local frame: origin at mount point, shank runs along +X,
    curves gently downward. The outlet end has a hollowed bore."""
    r_out = SPOUT_R
    shank_start = COLUMN_R + 0.004  # start just outside the column surface
    shank_end = shank_start + SPOUT_SHANK_LEN
    bend_r = SPOUT_DROP  # bend radius equals the drop
    end_x = shank_end + bend_r * 0.6
    end_z = -SPOUT_DROP

    # Path: straight section then gentle downward arc.
    path = (
        cq.Workplane("XZ")
        .moveTo(shank_start, 0.0)
        .lineTo(shank_end, 0.0)
        .tangentArcPoint((bend_r * 0.6, -bend_r), relative=True)
    )

    # Sweep the tube along the path.
    tube = cq.Workplane("YZ", origin=(shank_start, 0.0, 0.0)).circle(r_out).sweep(path)

    # Flared outlet rim at the curved end.
    flare = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z + 0.005))
        .circle(r_out + 0.001)
        .workplane(offset=-0.008)
        .circle(r_out + 0.005)
        .loft()
    )
    spout = tube.union(flare)

    # Hollow bore through the outlet mouth (real open outlet).
    bore = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z - 0.008))
        .circle(r_out - 0.003)
        .extrude(0.016)
    )
    spout = spout.cut(bore)

    # Swivel sleeve: a hollow cylinder that fits over the column, providing
    # the mechanical swivel bearing. Intentionally overlaps the column.
    sleeve_outer = COLUMN_R + 0.005
    sleeve_inner = COLUMN_R - 0.001  # slightly under column R for tight fit
    sleeve_h = 0.016
    sleeve = (
        cq.Workplane("XY", origin=(0.0, 0.0, -sleeve_h / 2.0))
        .circle(sleeve_outer)
        .circle(sleeve_inner)
        .extrude(sleeve_h)
    )
    # Bridge block connecting the sleeve to the spout tube.
    bridge_len = shank_start - sleeve_outer + 0.004
    bridge = (
        cq.Workplane("YZ", origin=(sleeve_outer - 0.002, 0.0, 0.0))
        .rect(2 * r_out + 0.004, sleeve_h * 0.8)
        .extrude(bridge_len)
    )
    spout = spout.union(sleeve).union(bridge)

    return spout


def _build_oval_gasket() -> cq.Workplane:
    """Oval rubber base gasket using an elliptical extrusion."""
    gasket = (
        cq.Workplane("XY")
        .ellipse(GASKET_RX, GASKET_RY)
        .extrude(GASKET_H)
    )
    # Cut a center hole for the column to pass through.
    hole = (
        cq.Workplane("XY")
        .circle(COLUMN_R - 0.001)
        .extrude(GASKET_H)
    )
    return gasket.cut(hole)


def _build_cap_shape() -> cq.Workplane:
    """Flat round temperature cap with a brushed top surface and
    a small indicator bump on the rim."""
    # Main cap disc.
    disc = cq.Workplane("XY").circle(CAP_R).extrude(CAP_H)
    # Soften top edge.
    disc = disc.edges(">Z").fillet(0.001)
    # Add a small grip ring on the side for visual detail.
    grip = (
        cq.Workplane("XY", origin=(0.0, 0.0, CAP_H * 0.3))
        .circle(CAP_R + 0.001)
        .circle(CAP_R - 0.001)
        .extrude(CAP_H * 0.4)
    )
    cap = disc.union(grip)
    # Small indicator bump on the rim (part of the same mesh).
    indicator = (
        cq.Workplane("YZ", origin=(CAP_R + 0.001, 0.0, CAP_H * 0.5))
        .circle(0.002)
        .extrude(0.003)
    )
    cap = cap.union(indicator)
    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    model.material("chrome_brushed", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("chrome_dark", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("rubber_gasket", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("index_mark", rgba=(0.80, 0.20, 0.20, 1.0))

    # ---------------- body (root): column + gasket + groove + neck ---------
    body = model.part("body")
    # Oval base gasket.
    body.visual(
        mesh_from_cadquery(_build_oval_gasket(), "base_gasket", tolerance=0.0003),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="rubber_gasket",
        name="base_gasket",
    )
    # Main cylindrical column.
    body.visual(
        Cylinder(radius=COLUMN_R, length=COLUMN_H),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_Z0 + COLUMN_H / 2.0)),
        material="chrome",
        name="column",
    )
    # Decorative groove ring.
    body.visual(
        Cylinder(radius=GROOVE_R, length=GROOVE_H),
        origin=Origin(xyz=(0.0, 0.0, GROOVE_Z + GROOVE_H / 2.0)),
        material="chrome_dark",
        name="groove_ring",
    )
    # Upper neck (slightly stepped-in).
    body.visual(
        Cylinder(radius=NECK_R, length=NECK_H),
        origin=Origin(xyz=(0.0, 0.0, NECK_Z0 + NECK_H / 2.0)),
        material="chrome",
        name="neck",
    )

    # ---------------- spout (swivels around vertical axis) -----------------
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_spout_shape(), "spout", tolerance=0.0003),
        material="chrome",
        name="spout_tube",
    )
    # Spout swivel: revolute around Z axis at the mount height.
    # The spout local frame has the shank along +X; swivel rotates it
    # around the column's vertical axis.
    model.articulation(
        "spout_swivel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, SPOUT_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=1.0,
            lower=-SPOUT_SWIVEL_LIMIT,
            upper=SPOUT_SWIVEL_LIMIT,
        ),
    )

    # ---------------- temperature cap (revolute on top) --------------------
    cap = model.part("cap")
    cap.visual(
        mesh_from_cadquery(_build_cap_shape(), "cap", tolerance=0.0003),
        material="chrome_brushed",
        name="cap_disc",
    )

    # Cap rotates around Z axis for temperature adjustment.
    model.articulation(
        "cap_turn",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, CAP_Z0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-TURN_LIMIT,
            upper=TURN_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    cap = object_model.get_part("cap")
    swivel = object_model.get_articulation("spout_swivel")
    turn = object_model.get_articulation("cap_turn")

    # Intentional swivel sleeve overlap: the spout sleeve fits over the column
    # as the swivel bearing surface.
    ctx.allow_overlap(
        spout,
        body,
        elem_a="spout_tube",
        elem_b="column",
        reason="Spout swivel sleeve intentionally fits over the column as a bearing surface.",
    )

    # ---- oval base gasket is present and wider than the column ------------
    gasket_aabb = ctx.part_element_world_aabb(body, elem="base_gasket")
    column_aabb = ctx.part_element_world_aabb(body, elem="column")
    ctx.check(
        "oval gasket extends beyond the column footprint",
        gasket_aabb is not None
        and column_aabb is not None
        and (gasket_aabb[1][0] - gasket_aabb[0][0]) > (column_aabb[1][0] - column_aabb[0][0]) + 0.010
        and (gasket_aabb[1][1] - gasket_aabb[0][1]) > (column_aabb[1][1] - column_aabb[0][1]) + 0.004,
        details=f"gasket={gasket_aabb}, column={column_aabb}",
    )

    # ---- gasket sits flat on the deck ------------------------------------
    ctx.check(
        "gasket sits flat on the deck",
        gasket_aabb is not None and abs(gasket_aabb[0][2]) <= 0.001,
        details=f"gasket aabb={gasket_aabb}",
    )

    # ---- column is vertical (centered over gasket) -----------------------
    ctx.check(
        "column is vertically centered over the gasket",
        gasket_aabb is not None
        and column_aabb is not None
        and abs((column_aabb[0][0] + column_aabb[1][0]) / 2.0 - (gasket_aabb[0][0] + gasket_aabb[1][0]) / 2.0) < 0.003,
        details=f"gasket={gasket_aabb}, column={column_aabb}",
    )

    # Proof check: sleeve overlaps the column on the Z axis (swivel bearing seated).
    ctx.expect_overlap(
        spout,
        body,
        axes="z",
        elem_a="spout_tube",
        elem_b="column",
        min_overlap=0.008,
        name="spout sleeve seated on column vertically",
    )

    # ---- spout projects forward and curves downward ----------------------
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "spout projects forward from the column and drops down",
        spout_aabb is not None
        and spout_aabb[1][0] > 0.040  # reaches forward
        and spout_aabb[0][2] < SPOUT_MOUNT_Z - 0.005,  # outlet below mount
        details=f"spout aabb={spout_aabb}",
    )

    # ---- spout outlet is below the mount point ---------------------------
    spout_tube_aabb = ctx.part_element_world_aabb(spout, elem="spout_tube")
    ctx.check(
        "spout outlet end is below the mount height",
        spout_tube_aabb is not None
        and spout_tube_aabb[0][2] < SPOUT_MOUNT_Z - 0.010,
        details=f"spout_tube aabb={spout_tube_aabb}",
    )

    # ---- spout swivel joint limits are correct ---------------------------
    sl = swivel.motion_limits
    ctx.check(
        "spout swivel limits are ±120 degrees",
        sl is not None
        and sl.lower is not None
        and sl.upper is not None
        and abs(sl.lower + SPOUT_SWIVEL_LIMIT) < 1e-6
        and abs(sl.upper - SPOUT_SWIVEL_LIMIT) < 1e-6,
        details=f"limits={sl}",
    )

    # ---- cap turn joint limits are correct -------------------------------
    tl = turn.motion_limits
    ctx.check(
        "cap turn limits are ±60 degrees",
        tl is not None
        and tl.lower is not None
        and tl.upper is not None
        and abs(tl.lower + TURN_LIMIT) < 1e-6
        and abs(tl.upper - TURN_LIMIT) < 1e-6,
        details=f"limits={tl}",
    )

    # ---- cap sits at top of column ----------------------------------------
    cap_aabb = ctx.part_world_aabb(cap)
    ctx.check(
        "cap sits above the column top",
        cap_aabb is not None
        and column_aabb is not None
        and cap_aabb[0][2] >= column_aabb[1][2] - 0.005,
        details=f"cap={cap_aabb}, column={column_aabb}",
    )

    # ---- decisive pose: spout swivels to the side -------------------------
    spout_rest_aabb = ctx.part_element_world_aabb(spout, elem="spout_tube")
    with ctx.pose({swivel: SPOUT_SWIVEL_LIMIT}):
        spout_swung_aabb = ctx.part_element_world_aabb(spout, elem="spout_tube")
    ctx.check(
        "spout swivels laterally when rotated",
        spout_rest_aabb is not None
        and spout_swung_aabb is not None
        and abs(
            (spout_swung_aabb[0][1] + spout_swung_aabb[1][1]) / 2.0
            - (spout_rest_aabb[0][1] + spout_rest_aabb[1][1]) / 2.0
        ) > 0.015,
        details=f"rest_center_y={(spout_rest_aabb[0][1] + spout_rest_aabb[1][1]) / 2.0}, swung_center_y={(spout_swung_aabb[0][1] + spout_swung_aabb[1][1]) / 2.0}",
    )

    # ---- decisive pose: cap rotation swings the indicator bump ------------
    # The indicator bump is part of the cap_disc mesh on the +X side.
    # When the cap rotates around Z, the bump center should move in Y.
    cap_rest_aabb = ctx.part_element_world_aabb(cap, elem="cap_disc")
    with ctx.pose({turn: TURN_LIMIT}):
        cap_turned_aabb = ctx.part_element_world_aabb(cap, elem="cap_disc")
    ctx.check(
        "turning the cap swings the indicator bump laterally",
        cap_rest_aabb is not None
        and cap_turned_aabb is not None
        and abs(cap_turned_aabb[1][1] - cap_rest_aabb[1][1]) > 0.005,
        details=f"rest_y_max={cap_rest_aabb[1][1] if cap_rest_aabb else None}, turned_y_max={cap_turned_aabb[1][1] if cap_turned_aabb else None}",
    )

    # ---- overall faucet height is plausible -------------------------------
    ctx.check(
        "overall faucet height is about 0.12 to 0.14 m",
        cap_aabb is not None
        and 0.105 <= cap_aabb[1][2] <= 0.145,
        details=f"cap top={cap_aabb[1][2] if cap_aabb else None}",
    )

    return ctx.report()


object_model = build_object_model()
