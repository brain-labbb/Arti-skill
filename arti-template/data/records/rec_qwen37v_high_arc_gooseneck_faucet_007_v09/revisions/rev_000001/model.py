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
# Variant 09: High-arc gooseneck faucet sibling
#
# Changes from parent monobloc mixer tap:
# - Faceted neck with visible segmented bends (polyline path instead of arc)
# - Temperature ring rotates around the pedestal (revolute about Z)
# - Visible cold/hot tick marks as geometry protrusions on the column
# - Retain tall arcing gooseneck silhouette and faucet identity
# ---------------------------------------------------------------------------

# Layout (world frame, deck plane at z = 0):
# - +X is the front of the tap (the direction the gooseneck reaches over the
#   sink), +Z is up.

# Base + column
BASE_DISC_R = 0.042
BASE_DISC_H = 0.008
COLUMN_R = 0.020  # 0.04 m diameter per prompt
COLUMN_TOP = 0.132  # shaft reaches 2 mm into the collar for connectivity

# Cross valve cylinder
CROSS_Z = 0.085
CROSS_R = 0.0225  # 0.045 m diameter per prompt
CROSS_TUBE_LEN = 0.170
CAP_LEN = 0.005
CAP_R = 0.0235
CAP_Y = CROSS_TUBE_LEN / 2.0 + CAP_LEN / 2.0  # 0.0875

# Pin levers (lever-local frame at the valve axis center)
LEVER_Y = 0.058
BOSS_R = 0.010
BOSS_LEN = 0.016
BOSS_Z = 0.026
PIN_R = 0.006  # 0.012 m diameter
PIN_LEN = 0.100
PIN_Z0 = 0.032

# Swivel collar + gooseneck (spout-local frame at the collar top)
COLLAR_R = 0.0215
COLLAR_LEN = 0.010
SWIVEL_Z = 0.140

TUBE_R = 0.015
ARC_R = 0.072
RISER_TOP = 0.153
REACH_X = 2.0 * ARC_R  # 0.144 m horizontal reach
DROP_END = 0.124

SLEEVE_R = 0.0165
SLEEVE_LEN = 0.028
AERATOR_R = 0.0118
AERATOR_LEN = 0.003

APEX_WORLD = SWIVEL_Z + RISER_TOP + ARC_R + TUBE_R  # 0.380 m

SWIVEL_LIMIT = math.radians(110.0)
RING_LIMIT = math.radians(90.0)

# --- Temperature ring (around the pedestal column) ---
RING_Z_BASE = 0.040  # ring bottom at z=0.040
RING_HEIGHT = 0.012  # ring spans z 0.040..0.052
RING_OUTER_R = 0.028  # slightly wider than the column
RING_INNER_R = COLUMN_R + 0.0005  # small clearance for rotation
RING_INDICATOR_R = 0.030  # small protruding indicator bump on the ring

# --- Tick marks on the column body (cold/hot geometry marks) ---
TICK_Z_CENTER = 0.046  # midway through the ring height
TICK_WIDTH = 0.003  # thin tick mark
TICK_HEIGHT = 0.008  # vertical extent
TICK_DEPTH = 0.003  # protrusion from column surface
# Cold tick at +Y side (angle 0 from +Y), Hot tick at -Y side
TICK_COLD_ANGLE = 0.0  # +Y direction
TICK_HOT_ANGLE = math.pi  # -Y direction


def _faceted_gooseneck_shape() -> cq.Workplane:
    """Faceted swan-neck tube: straight segments with visible angular bends.

    Replaces the smooth threePointArc with a polyline of ~6 straight segments
    approximating the arc, producing visible facet lines at each bend.
    """
    n_arc_segments = 6

    # Build polyline points in XZ plane
    points = [(0.0, 0.0), (0.0, RISER_TOP)]

    # Arc segments
    for i in range(1, n_arc_segments + 1):
        angle = math.pi * i / n_arc_segments
        x = ARC_R * (1.0 - math.cos(angle))
        z = RISER_TOP + ARC_R * math.sin(angle)
        points.append((x, z))

    # Drop leg endpoint
    points.append((REACH_X, DROP_END))

    # Build polyline path
    wp = cq.Workplane("XZ").moveTo(points[0][0], points[0][1])
    for pt in points[1:]:
        wp = wp.lineTo(pt[0], pt[1])

    return cq.Workplane("XY").circle(TUBE_R).sweep(wp)


def _temperature_ring_shape() -> cq.Workplane:
    """Annular ring that wraps around the column pedestal."""
    return (
        cq.Workplane("XY")
        .circle(RING_OUTER_R)
        .circle(RING_INNER_R)
        .extrude(RING_HEIGHT)
    )


def _ring_indicator_shape() -> cq.Workplane:
    """Small protruding indicator bump on the temperature ring outer surface."""
    # A thin radial tab at the ring midheight, protruding outward
    return (
        cq.Workplane("XY")
        .transformed(offset=(RING_OUTER_R + TICK_DEPTH / 2.0, 0.0, RING_HEIGHT / 2.0))
        .box(TICK_DEPTH, TICK_WIDTH, TICK_HEIGHT)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_arc_gooseneck_faucet")

    gloss_black = model.material("veined_gloss_black", rgba=(0.075, 0.075, 0.085, 1.0))
    matte_black = model.material("matte_black", rgba=(0.035, 0.035, 0.038, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    outlet_dark = model.material("outlet_dark", rgba=(0.10, 0.10, 0.10, 1.0))
    tick_white = model.material("tick_mark", rgba=(0.80, 0.82, 0.85, 1.0))
    ring_chrome = model.material("ring_chrome", rgba=(0.78, 0.80, 0.82, 1.0))

    # ------------------------------------------------------------------ column
    column = model.part("body_column")
    column.visual(
        Cylinder(radius=BASE_DISC_R, length=BASE_DISC_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_DISC_H / 2.0)),
        material=chrome,
        name="base_disc",
    )
    column.visual(
        Cylinder(radius=COLUMN_R, length=COLUMN_TOP - 0.004),
        origin=Origin(xyz=(0.0, 0.0, (COLUMN_TOP + 0.004) / 2.0)),
        material=gloss_black,
        name="column_shaft",
    )
    # Horizontal cross valve cylinder through the column, left-right (Y axis).
    column.visual(
        Cylinder(radius=CROSS_R, length=CROSS_TUBE_LEN),
        origin=Origin(xyz=(0.0, 0.0, CROSS_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gloss_black,
        name="cross_tube",
    )
    column.visual(
        Cylinder(radius=CAP_R, length=CAP_LEN),
        origin=Origin(xyz=(0.0, CAP_Y, CROSS_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="valve_end_cap_0",
    )
    column.visual(
        Cylinder(radius=CAP_R, length=CAP_LEN),
        origin=Origin(xyz=(0.0, -CAP_Y, CROSS_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="valve_end_cap_1",
    )
    # Thin chrome collar ring separating the column from the swivel spout.
    column.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z - COLLAR_LEN / 2.0)),
        material=chrome,
        name="swivel_collar",
    )

    # --- Cold / hot tick marks as geometry on the column pedestal ---
    # Cold tick: small protruding box at +Y side of column
    tick_cold_y = COLUMN_R + TICK_DEPTH / 2.0
    column.visual(
        Box((TICK_WIDTH, TICK_DEPTH, TICK_HEIGHT)),
        origin=Origin(xyz=(0.0, tick_cold_y, TICK_Z_CENTER)),
        material=tick_white,
        name="tick_cold",
    )
    # Hot tick: small protruding box at -Y side of column
    column.visual(
        Box((TICK_WIDTH, TICK_DEPTH, TICK_HEIGHT)),
        origin=Origin(xyz=(0.0, -tick_cold_y, TICK_Z_CENTER)),
        material=tick_white,
        name="tick_hot",
    )

    # ------------------------------------------------------- temperature ring
    temp_ring = model.part("temperature_ring")
    # Part frame sits at the articulation origin (z=RING_Z_BASE world).
    # Visual origins are relative to the part frame.
    temp_ring.visual(
        mesh_from_cadquery(_temperature_ring_shape(), "ring_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=ring_chrome,
        name="ring_body",
    )
    # Small indicator bump on the ring outer surface at +X (front-facing at rest)
    temp_ring.visual(
        Box((TICK_DEPTH, TICK_WIDTH, TICK_HEIGHT)),
        origin=Origin(xyz=(RING_OUTER_R + TICK_DEPTH / 2.0, 0.0, RING_HEIGHT / 2.0)),
        material=ring_chrome,
        name="ring_indicator",
    )

    # Ring rotates about the vertical column axis
    model.articulation(
        "ring_rotate",
        ArticulationType.REVOLUTE,
        parent=column,
        child=temp_ring,
        origin=Origin(xyz=(0.0, 0.0, RING_Z_BASE)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=-RING_LIMIT, upper=RING_LIMIT
        ),
    )

    # --------------------------------------------------------------- gooseneck
    spout = model.part("gooseneck_spout")
    spout.visual(
        mesh_from_cadquery(_faceted_gooseneck_shape(), "gooseneck_tube"),
        material=gloss_black,
        name="gooseneck_tube",
    )
    spout.visual(
        Cylinder(radius=SLEEVE_R, length=SLEEVE_LEN),
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END + SLEEVE_LEN / 2.0)),
        material=chrome,
        name="tip_sleeve",
    )
    spout.visual(
        Cylinder(radius=AERATOR_R, length=AERATOR_LEN),
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END - 0.001)),
        material=outlet_dark,
        name="outlet_aerator",
    )
    model.articulation(
        "spout_swivel",
        ArticulationType.REVOLUTE,
        parent=column,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=1.5, lower=-SWIVEL_LIMIT, upper=SWIVEL_LIMIT
        ),
    )

    # ------------------------------------------------------------- pin levers
    for idx, y_sign in ((0, 1.0), (1, -1.0)):
        lever = model.part(f"pin_lever_{idx}")
        lever.visual(
            Cylinder(radius=BOSS_R, length=BOSS_LEN),
            origin=Origin(xyz=(0.0, 0.0, BOSS_Z)),
            material=gloss_black,
            name="lever_boss",
        )
        lever.visual(
            Cylinder(radius=PIN_R, length=PIN_LEN),
            origin=Origin(xyz=(0.0, 0.0, PIN_Z0 + PIN_LEN / 2.0)),
            material=gloss_black,
            name="lever_pin",
        )
        model.articulation(
            f"lever_pivot_{idx}",
            ArticulationType.REVOLUTE,
            parent=column,
            child=lever,
            origin=Origin(xyz=(0.0, y_sign * LEVER_Y, CROSS_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=5.0, velocity=2.0, lower=-math.pi / 2.0, upper=0.0
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    column = object_model.get_part("body_column")
    spout = object_model.get_part("gooseneck_spout")
    lever_0 = object_model.get_part("pin_lever_0")
    lever_1 = object_model.get_part("pin_lever_1")
    temp_ring = object_model.get_part("temperature_ring")

    swivel = object_model.get_articulation("spout_swivel")
    pivot_0 = object_model.get_articulation("lever_pivot_0")
    pivot_1 = object_model.get_articulation("lever_pivot_1")
    ring_joint = object_model.get_articulation("ring_rotate")

    # Intentional seated insertions
    ctx.allow_overlap(
        lever_0,
        column,
        elem_a="lever_boss",
        elem_b="cross_tube",
        reason="Lever boss intentionally seats a few mm into the valve cylinder.",
    )
    ctx.allow_overlap(
        lever_1,
        column,
        elem_a="lever_boss",
        elem_b="cross_tube",
        reason="Lever boss intentionally seats a few mm into the valve cylinder.",
    )

    # --- Variant 09: temperature ring wraps around the column pedestal ---
    # The ring inner bore is slightly larger than the column, so the ring
    # body overlaps with the column shaft in 3D. This is intentional: the
    # ring is meant to encircle the column.
    ctx.allow_overlap(
        temp_ring,
        column,
        elem_a="ring_body",
        elem_b="column_shaft",
        reason="Temperature ring encircles the column; the annular bore intentionally overlaps the shaft.",
    )

    # ----- grounding, scale, proportions -----
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "tap grounded on the deck plane",
        col_aabb is not None and abs(col_aabb[0][2]) <= 0.002,
        details=f"column aabb={col_aabb}",
    )
    disc = ctx.part_element_world_aabb(column, elem="base_disc")
    ctx.check(
        "single chrome base disc sits on the deck (wide, thin)",
        disc is not None
        and 0.080 <= (disc[1][0] - disc[0][0]) <= 0.090
        and (disc[1][2] - disc[0][2]) <= 0.010,
        details=f"base disc aabb={disc}",
    )
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "gooseneck apex near 0.38 m",
        spout_aabb is not None and 0.370 <= spout_aabb[1][2] <= 0.392,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "gooseneck arcs forward over the sink (+X reach)",
        spout_aabb is not None and spout_aabb[1][0] >= 0.120,
        details=f"spout aabb={spout_aabb}",
    )

    # ----- VARIANT 09: faceted neck with visible segmented bends -----
    tube = ctx.part_element_world_aabb(spout, elem="gooseneck_tube")
    ctx.check(
        "faceted gooseneck tube exists with high-arc silhouette",
        tube is not None
        and tube[1][2] >= 0.350  # apex is high
        and tube[1][0] >= 0.120  # reaches forward
        and tube[0][2] <= SWIVEL_Z + 0.005,  # starts near collar
        details=f"tube aabb={tube}",
    )

    # ----- VARIANT 09: temperature ring geometry and articulation -----
    ring_aabb = ctx.part_world_aabb(temp_ring)
    ctx.check(
        "temperature ring exists on the pedestal between base and cross valve",
        ring_aabb is not None
        and ring_aabb[0][2] >= BASE_DISC_H - 0.001
        and ring_aabb[1][2] <= CROSS_Z - CROSS_R + 0.005,
        details=f"ring aabb={ring_aabb}",
    )
    ring_body = ctx.part_element_world_aabb(temp_ring, elem="ring_body")
    ctx.check(
        "temperature ring is wider than the column (annular form)",
        ring_body is not None
        and (ring_body[1][0] - ring_body[0][0]) > 2.0 * COLUMN_R + 0.005,
        details=f"ring body aabb={ring_body}",
    )

    # Ring joint verification
    ctx.check(
        "ring_rotate is revolute about the vertical axis with -90..+90 deg limits",
        ring_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(ring_joint.axis) == (0.0, 0.0, 1.0)
        and ring_joint.motion_limits is not None
        and abs(ring_joint.motion_limits.lower + RING_LIMIT) < 1e-6
        and abs(ring_joint.motion_limits.upper - RING_LIMIT) < 1e-6,
    )

    # Ring pose check: rotating the ring moves the indicator sideways
    rest_indicator = ctx.part_element_world_aabb(temp_ring, elem="ring_indicator")
    with ctx.pose({ring_joint: math.pi / 2.0}):
        rotated_indicator = ctx.part_element_world_aabb(temp_ring, elem="ring_indicator")
    ctx.check(
        "temperature ring rotation moves the indicator from front to side",
        rest_indicator is not None
        and rotated_indicator is not None
        and abs(rotated_indicator[0][1] - rest_indicator[0][1]) > 0.01,
        details=f"rest={rest_indicator}, rotated={rotated_indicator}",
    )

    # ----- VARIANT 09: cold/hot tick marks as geometry -----
    tick_cold = ctx.part_element_world_aabb(column, elem="tick_cold")
    tick_hot = ctx.part_element_world_aabb(column, elem="tick_hot")
    ctx.check(
        "cold tick mark exists as geometry on the column pedestal",
        tick_cold is not None
        and tick_cold[1][2] - tick_cold[0][2] >= 0.005
        and tick_cold[0][2] >= BASE_DISC_H
        and tick_cold[1][2] <= CROSS_Z,
        details=f"tick_cold aabb={tick_cold}",
    )
    ctx.check(
        "hot tick mark exists as geometry on the column pedestal",
        tick_hot is not None
        and tick_hot[1][2] - tick_hot[0][2] >= 0.005
        and tick_hot[0][2] >= BASE_DISC_H
        and tick_hot[1][2] <= CROSS_Z,
        details=f"tick_hot aabb={tick_hot}",
    )
    ctx.check(
        "cold and hot ticks are on opposite sides of the column",
        tick_cold is not None
        and tick_hot is not None
        and 0.5 * (tick_cold[0][1] + tick_cold[1][1]) > 0.0
        and 0.5 * (tick_hot[0][1] + tick_hot[1][1]) < 0.0,
        details=f"cold={tick_cold}, hot={tick_hot}",
    )

    # ----- cross valve cylinder with flat black end caps -----
    cross = ctx.part_element_world_aabb(column, elem="cross_tube")
    cap_0 = ctx.part_element_world_aabb(column, elem="valve_end_cap_0")
    cap_1 = ctx.part_element_world_aabb(column, elem="valve_end_cap_1")
    ctx.check(
        "cross-cylinder is ~0.045 m diameter at mid-column height",
        cross is not None
        and 0.043 <= (cross[1][2] - cross[0][2]) <= 0.047
        and 0.06 <= 0.5 * (cross[0][2] + cross[1][2]) <= 0.11,
        details=f"cross aabb={cross}",
    )
    ctx.check(
        "valve assembly spans ~0.18 m end-to-end cap face to cap face",
        cap_0 is not None
        and cap_1 is not None
        and 0.178 <= (cap_0[1][1] - cap_1[0][1]) <= 0.182,
        details=f"cap_0={cap_0}, cap_1={cap_1}",
    )

    # ----- chrome collar ring; spout seats on it -----
    collar = ctx.part_element_world_aabb(column, elem="swivel_collar")
    ctx.check(
        "thin chrome collar sits above the cross and below the spout",
        collar is not None
        and cross is not None
        and collar[0][2] >= cross[1][2]
        and spout_aabb is not None
        and collar[1][2] <= spout_aabb[0][2] + 1e-6,
        details=f"collar={collar}, cross_top={cross[1][2] if cross else None}",
    )
    ctx.expect_contact(
        spout,
        column,
        elem_a="gooseneck_tube",
        elem_b="swivel_collar",
        contact_tol=0.002,
        name="gooseneck riser seats on the chrome collar",
    )

    # ----- chrome tip sleeve with downward outlet -----
    sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    aerator = ctx.part_element_world_aabb(spout, elem="outlet_aerator")
    ctx.check(
        "chrome tip sleeve wraps the spout drop leg with a downward outlet",
        sleeve is not None
        and aerator is not None
        and tube is not None
        and 0.24 <= sleeve[0][2] <= 0.29
        and aerator[0][2] < sleeve[0][2]
        and abs(0.5 * (sleeve[0][0] + sleeve[1][0]) - REACH_X) <= 0.005,
        details=f"sleeve={sleeve}, aerator={aerator}",
    )

    # ----- pin levers: geometry and seating -----
    for lever, name in ((lever_0, "pin_lever_0"), (lever_1, "pin_lever_1")):
        pin = ctx.part_element_world_aabb(lever, elem="lever_pin")
        ctx.check(
            f"{name} pin is slim (0.012 m dia) and ~0.10 m long, vertical at rest",
            pin is not None
            and 0.010 <= (pin[1][0] - pin[0][0]) <= 0.014
            and 0.095 <= (pin[1][2] - pin[0][2]) <= 0.105,
            details=f"pin aabb={pin}",
        )
        ctx.expect_overlap(
            lever,
            column,
            axes="z",
            elem_a="lever_boss",
            elem_b="cross_tube",
            min_overlap=0.003,
            name=f"{name} boss seats into the valve cylinder",
        )

    # ----- joint plan: spout swivel -----
    ctx.check(
        "spout swivel is revolute -110..+110 deg about the vertical column axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and tuple(swivel.axis) == (0.0, 0.0, 1.0)
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + SWIVEL_LIMIT) < 1e-6
        and abs(swivel.motion_limits.upper - SWIVEL_LIMIT) < 1e-6,
    )

    # ----- lever pivots -----
    for pivot, name in ((pivot_0, "lever_pivot_0"), (pivot_1, "lever_pivot_1")):
        ctx.check(
            f"{name} is revolute -90..0 deg about the valve's left-right axis",
            pivot.articulation_type == ArticulationType.REVOLUTE
            and tuple(pivot.axis) == (0.0, -1.0, 0.0)
            and pivot.motion_limits is not None
            and abs(pivot.motion_limits.lower + math.pi / 2.0) < 1e-6
            and abs(pivot.motion_limits.upper) < 1e-6
            and pivot.mimic is None,
        )

    # ----- swivel pose: spout outlet sweeps sideways -----
    rest_sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    with ctx.pose({swivel: 1.0}):
        sw_sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    ctx.check(
        "spout swivel carries the outlet sideways about the vertical axis",
        rest_sleeve is not None
        and sw_sleeve is not None
        and abs(0.5 * (rest_sleeve[0][1] + rest_sleeve[1][1])) < 1e-6
        and 0.5 * (sw_sleeve[0][1] + sw_sleeve[1][1]) > 0.08,
        details=f"rest={rest_sleeve}, swiveled={sw_sleeve}",
    )

    # ----- Non-fixed joint count check -----
    all_joints = [swivel, pivot_0, pivot_1, ring_joint]
    non_fixed = [
        j for j in all_joints
        if j.articulation_type
        in (ArticulationType.REVOLUTE, ArticulationType.PRISMATIC, ArticulationType.CONTINUOUS)
    ]
    ctx.check(
        "at least one non-fixed joint exists",
        len(non_fixed) >= 1,
        details=f"non-fixed joints: {len(non_fixed)}",
    )

    return ctx.report()


object_model = build_object_model()
