from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Variant 25: High-arc gooseneck faucet (forked from gloss-black monobloc
# kitchen mixer tap). ~0.38 m tall, deck-mounted.
#
# Structural changes from parent:
# - Stepped cylindrical pedestal with a broad escutcheon (replaces flat disc)
# - Removable circular deck plate under the base (separate part)
# - Small top flow knob rotates independently (REVOLUTE about Z)
# - Visible cold/hot tick marks as geometry (small raised boxes)
#
# Layout (world frame, deck plane at z = 0):
# - +X is front (gooseneck reach direction), +Z is up.
# - A removable chrome deck plate sits on the deck.
# - On top: broad chrome escutcheon ring, two-step cylindrical pedestal,
#   then the gloss-black column rises.
# - Horizontal cross-cylinder passes through forming two valve bodies with
#   flat black end caps.
# - Slim pin levers on each valve body (kept from parent).
# - Chrome collar ring separates column from gooseneck spout.
# - High swan-neck gooseneck arcs up to ~0.38 m apex, chrome tip + outlet.
# - Small flow knob on top of the column rotates about vertical axis.
# - Cold (blue tick) and hot (red tick) marks on the column body.
# ---------------------------------------------------------------------------

# Deck plate (removable)
DECK_PLATE_R = 0.065
DECK_PLATE_H = 0.004

# Stepped pedestal + escutcheon
ESCUTCHEON_R = 0.055
ESCUTCHEON_H = 0.005
STEP1_R = 0.038
STEP1_H = 0.012
STEP2_R = 0.028
STEP2_H = 0.010

# Z stacking from deck surface
DECK_TOP = DECK_PLATE_H  # 0.004
ESC_Z0 = DECK_TOP  # escutcheon starts at deck top
ESC_ZC = ESC_Z0 + ESCUTCHEON_H / 2.0  # 0.0065
STEP1_Z0 = ESC_Z0 + ESCUTCHEON_H  # 0.009
STEP1_ZC = STEP1_Z0 + STEP1_H / 2.0  # 0.015
STEP2_Z0 = STEP1_Z0 + STEP1_H  # 0.021
STEP2_ZC = STEP2_Z0 + STEP2_H / 2.0  # 0.026
COLUMN_Z0 = STEP2_Z0 + STEP2_H  # 0.031

# Column
COLUMN_R = 0.020
COLUMN_TOP = 0.136  # shaft top

# Cross valve cylinder
CROSS_Z = 0.090
CROSS_R = 0.0225
CROSS_TUBE_LEN = 0.170
CAP_LEN = 0.005
CAP_R = 0.0235
CAP_Y = CROSS_TUBE_LEN / 2.0 + CAP_LEN / 2.0

# Pin levers
LEVER_Y = 0.058
BOSS_R = 0.010
BOSS_LEN = 0.016
BOSS_Z = 0.026
PIN_R = 0.006
PIN_LEN = 0.100
PIN_Z0 = 0.032

# Swivel collar + gooseneck
COLLAR_R = 0.0215
COLLAR_LEN = 0.010
SWIVEL_Z = 0.144

TUBE_R = 0.015
ARC_R = 0.072
RISER_TOP = 0.153
REACH_X = 2.0 * ARC_R  # 0.144
DROP_END = 0.124

SLEEVE_R = 0.0165
SLEEVE_LEN = 0.028
AERATOR_R = 0.0118
AERATOR_LEN = 0.003

APEX_WORLD = SWIVEL_Z + RISER_TOP + ARC_R + TUBE_R  # ~0.384 m

SWIVEL_LIMIT = math.radians(110.0)

# Tick marks (cold/hot indicators)
TICK_W = 0.003  # width of tick mark
TICK_H = 0.012  # height of tick mark
TICK_D = 0.002  # depth (protrusion from column surface)
TICK_Z = CROSS_Z - 0.020  # just below the cross valve

# Flow knob (mounted on front face of column, just below collar)
KNOB_DIA = 0.018
KNOB_HEIGHT = 0.010
KNOB_MOUNT_X = COLUMN_R  # front face of column
KNOB_MOUNT_Z = 0.124  # centered between cross top and collar bottom


def _gooseneck_shape() -> cq.Workplane:
    """Swan-neck tube: straight riser, high semicircular arc, short drop leg."""
    path = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, RISER_TOP)
        .threePointArc((ARC_R, RISER_TOP + ARC_R), (REACH_X, RISER_TOP))
        .lineTo(REACH_X, DROP_END)
    )
    return cq.Workplane("XY").circle(TUBE_R).sweep(path)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_arc_gooseneck_faucet_v25")

    gloss_black = model.material("veined_gloss_black", rgba=(0.075, 0.075, 0.085, 1.0))
    matte_black = model.material("matte_black", rgba=(0.035, 0.035, 0.038, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    outlet_dark = model.material("outlet_dark", rgba=(0.10, 0.10, 0.10, 1.0))
    cold_blue = model.material("cold_indicator", rgba=(0.15, 0.25, 0.70, 1.0))
    hot_red = model.material("hot_indicator", rgba=(0.70, 0.15, 0.15, 1.0))

    # ------------------------------------------------------------------ root: body_column
    column = model.part("body_column")

    # Stepped pedestal with broad escutcheon
    column.visual(
        Cylinder(radius=ESCUTCHEON_R, length=ESCUTCHEON_H),
        origin=Origin(xyz=(0.0, 0.0, ESC_ZC)),
        material=chrome,
        name="escutcheon",
    )
    column.visual(
        Cylinder(radius=STEP1_R, length=STEP1_H),
        origin=Origin(xyz=(0.0, 0.0, STEP1_ZC)),
        material=gloss_black,
        name="pedestal_step_1",
    )
    column.visual(
        Cylinder(radius=STEP2_R, length=STEP2_H),
        origin=Origin(xyz=(0.0, 0.0, STEP2_ZC)),
        material=gloss_black,
        name="pedestal_step_2",
    )

    # Column shaft from top of step 2 to collar
    shaft_len = COLUMN_TOP - COLUMN_Z0
    column.visual(
        Cylinder(radius=COLUMN_R, length=shaft_len),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_Z0 + shaft_len / 2.0)),
        material=gloss_black,
        name="column_shaft",
    )

    # Horizontal cross valve cylinder through the column (Y axis)
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

    # Cold/hot tick marks as geometry (small raised boxes on column surface)
    # Cold mark at +Y side, hot mark at -Y side
    tick_r = COLUMN_R + TICK_D / 2.0  # center of tick protrusion
    column.visual(
        Box((TICK_D, TICK_W, TICK_H)),
        origin=Origin(xyz=(0.0, tick_r, TICK_Z)),
        material=cold_blue,
        name="cold_tick",
    )
    column.visual(
        Box((TICK_D, TICK_W, TICK_H)),
        origin=Origin(xyz=(0.0, -tick_r, TICK_Z)),
        material=hot_red,
        name="hot_tick",
    )

    # Chrome collar ring separating column from spout
    column.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z - COLLAR_LEN / 2.0)),
        material=chrome,
        name="swivel_collar",
    )

    # ---------------------------------------------------------- deck plate (removable)
    deck = model.part("deck_plate")
    deck.visual(
        Cylinder(radius=DECK_PLATE_R, length=DECK_PLATE_H),
        origin=Origin(xyz=(0.0, 0.0, DECK_PLATE_H / 2.0)),
        material=chrome,
        name="deck_disc",
    )
    # Fixed articulation: deck plate is mounted under the body
    model.articulation(
        "deck_mount",
        ArticulationType.FIXED,
        parent=column,
        child=deck,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --------------------------------------------------------------- gooseneck
    spout = model.part("gooseneck_spout")
    spout.visual(
        mesh_from_cadquery(_gooseneck_shape(), "gooseneck_tube"),
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

    # ------------------------------------------------------------- flow knob (front face)
    flow_knob = model.part("flow_knob")
    knob_geom = KnobGeometry(
        KNOB_DIA,
        KNOB_HEIGHT,
        body_style="cylindrical",
        grip=KnobGrip(style="fluted", count=14, depth=0.0008),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0005),
        center=False,
    )
    # Knob part frame is at the articulation origin (KNOB_MOUNT_X, 0, KNOB_MOUNT_Z).
    # Visual origins are relative to the part frame.
    # Knob mounted on front face, oriented horizontally (local Z = world +X)
    flow_knob.visual(
        mesh_from_geometry(knob_geom, "flow_knob_body"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gloss_black,
        name="flow_knob_mesh",
    )
    # Small mounting boss connecting knob to column face (embeds into column surface)
    flow_knob.visual(
        Cylinder(radius=0.005, length=0.008),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="knob_boss",
    )
    model.articulation(
        "knob_rotate",
        ArticulationType.REVOLUTE,
        parent=column,
        child=flow_knob,
        origin=Origin(xyz=(KNOB_MOUNT_X, 0.0, KNOB_MOUNT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=-math.pi, upper=math.pi
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    column = object_model.get_part("body_column")
    deck = object_model.get_part("deck_plate")
    spout = object_model.get_part("gooseneck_spout")
    lever_0 = object_model.get_part("pin_lever_0")
    lever_1 = object_model.get_part("pin_lever_1")
    flow_knob = object_model.get_part("flow_knob")

    swivel = object_model.get_articulation("spout_swivel")
    pivot_0 = object_model.get_articulation("lever_pivot_0")
    pivot_1 = object_model.get_articulation("lever_pivot_1")
    knob_joint = object_model.get_articulation("knob_rotate")
    deck_joint = object_model.get_articulation("deck_mount")

    # Intentional seated insertions: lever boss embeds into valve cylinder
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

    # Pre-fetch key element AABBs used across multiple test sections
    cross = ctx.part_element_world_aabb(column, elem="cross_tube")
    collar = ctx.part_element_world_aabb(column, elem="swivel_collar")
    spout_aabb = ctx.part_world_aabb(spout)

    # ----- Variant 25: stepped pedestal with broad escutcheon
    esc = ctx.part_element_world_aabb(column, elem="escutcheon")
    step1 = ctx.part_element_world_aabb(column, elem="pedestal_step_1")
    step2 = ctx.part_element_world_aabb(column, elem="pedestal_step_2")
    ctx.check(
        "broad escutcheon ring at the base (wider than pedestal steps)",
        esc is not None
        and step1 is not None
        and (esc[1][0] - esc[0][0]) > (step1[1][0] - step1[0][0]),
        details=f"esc={esc}, step1={step1}",
    )
    ctx.check(
        "stepped pedestal: step 1 wider than step 2",
        step1 is not None
        and step2 is not None
        and (step1[1][0] - step1[0][0]) > (step2[1][0] - step2[0][0]),
        details=f"step1={step1}, step2={step2}",
    )
    ctx.check(
        "pedestal steps stack vertically above escutcheon",
        esc is not None
        and step1 is not None
        and step2 is not None
        and step1[0][2] >= esc[0][2] - 0.001
        and step2[0][2] >= step1[1][2] - 0.001,
        details=f"esc={esc}, step1={step1}, step2={step2}",
    )

    # ----- Variant 25: removable circular deck plate under the base
    deck_aabb = ctx.part_world_aabb(deck)
    ctx.check(
        "removable deck plate is a broad circular disc at the deck plane",
        deck_aabb is not None
        and abs(deck_aabb[0][2]) <= 0.001
        and (deck_aabb[1][0] - deck_aabb[0][0]) >= 0.12,
        details=f"deck aabb={deck_aabb}",
    )
    ctx.check(
        "deck_mount is a FIXED joint connecting body to deck plate",
        deck_joint.articulation_type == ArticulationType.FIXED,
    )
    ctx.expect_gap(
        column,
        deck,
        axis="z",
        max_penetration=0.001,
        name="body column sits on or above the deck plate",
    )

    # ----- Variant 25: cold/hot tick marks as geometry
    cold = ctx.part_element_world_aabb(column, elem="cold_tick")
    hot = ctx.part_element_world_aabb(column, elem="hot_tick")
    ctx.check(
        "cold tick mark exists as geometry on the column",
        cold is not None and (cold[1][2] - cold[0][2]) >= 0.008,
        details=f"cold tick aabb={cold}",
    )
    ctx.check(
        "hot tick mark exists as geometry on the column",
        hot is not None and (hot[1][2] - hot[0][2]) >= 0.008,
        details=f"hot tick aabb={hot}",
    )
    ctx.check(
        "cold and hot ticks are on opposite sides of the column (Y axis)",
        cold is not None
        and hot is not None
        and 0.5 * (cold[0][1] + cold[1][1]) > 0.0
        and 0.5 * (hot[0][1] + hot[1][1]) < 0.0,
        details=f"cold={cold}, hot={hot}",
    )

    # ----- Variant 25: flow knob on front face with independent rotation
    knob_aabb = ctx.part_world_aabb(flow_knob)
    shaft_aabb = ctx.part_element_world_aabb(column, elem="column_shaft")
    ctx.check(
        "flow knob mounted on the front face of the column (above cross, below collar)",
        knob_aabb is not None
        and cross is not None
        and collar is not None
        and knob_aabb[1][0] > COLUMN_R
        and knob_aabb[0][2] > cross[1][2]
        and knob_aabb[1][2] < collar[0][2] + 0.002,
        details=f"knob aabb={knob_aabb}",
    )
    ctx.check(
        "knob_rotate is REVOLUTE about the front-facing axis",
        knob_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(knob_joint.axis) == (1.0, 0.0, 0.0)
        and knob_joint.motion_limits is not None
        and knob_joint.motion_limits.lower < 0.0
        and knob_joint.motion_limits.upper > 0.0,
    )

    # ----- Gooseneck silhouette preserved
    ctx.check(
        "gooseneck apex near 0.38 m",
        spout_aabb is not None and 0.372 <= spout_aabb[1][2] <= 0.395,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "gooseneck arcs forward over the sink (+X reach)",
        spout_aabb is not None and spout_aabb[1][0] >= 0.150,
        details=f"spout aabb={spout_aabb}",
    )

    # ----- Column grounded and scale
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "faucet body grounded near the deck plane",
        col_aabb is not None and col_aabb[0][2] <= 0.012,
        details=f"column aabb={col_aabb}",
    )
    ctx.check(
        "vertical column is ~0.04 m diameter",
        shaft_aabb is not None and 0.038 <= (shaft_aabb[1][0] - shaft_aabb[0][0]) <= 0.042,
        details=f"column shaft aabb={shaft_aabb}",
    )

    # ----- Cross valve cylinder with flat black end caps
    cap_0 = ctx.part_element_world_aabb(column, elem="valve_end_cap_0")
    cap_1 = ctx.part_element_world_aabb(column, elem="valve_end_cap_1")
    ctx.check(
        "cross-cylinder is ~0.045 m diameter",
        cross is not None
        and 0.043 <= (cross[1][2] - cross[0][2]) <= 0.047,
        details=f"cross aabb={cross}",
    )
    ctx.check(
        "valve assembly spans ~0.18 m end-to-end",
        cap_0 is not None
        and cap_1 is not None
        and 0.178 <= (cap_0[1][1] - cap_1[0][1]) <= 0.182,
        details=f"cap_0={cap_0}, cap_1={cap_1}",
    )

    # ----- Chrome collar + spout seating
    ctx.check(
        "thin chrome collar sits above the cross and below spout base",
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
        contact_tol=0.001,
        name="gooseneck riser seats on the chrome collar",
    )

    # ----- Chrome tip sleeve with downward outlet
    sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    aerator = ctx.part_element_world_aabb(spout, elem="outlet_aerator")
    ctx.check(
        "chrome tip sleeve wraps the spout drop leg with a downward outlet",
        sleeve is not None
        and aerator is not None
        and aerator[0][2] < sleeve[0][2],
        details=f"sleeve={sleeve}, aerator={aerator}",
    )

    # ----- Pin levers: geometry and seating
    for lever, name in ((lever_0, "pin_lever_0"), (lever_1, "pin_lever_1")):
        pin = ctx.part_element_world_aabb(lever, elem="lever_pin")
        ctx.check(
            f"{name} pin is slim and vertical at rest",
            pin is not None
            and 0.010 <= (pin[1][0] - pin[0][0]) <= 0.014
            and 0.098 <= (pin[1][2] - pin[0][2]) <= 0.102,
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

    # ----- Joint plan: spout swivel
    ctx.check(
        "spout swivel is revolute about the vertical column axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and tuple(swivel.axis) == (0.0, 0.0, 1.0)
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + SWIVEL_LIMIT) < 1e-6
        and abs(swivel.motion_limits.upper - SWIVEL_LIMIT) < 1e-6,
    )

    # ----- Lever joints
    for pivot, name in ((pivot_0, "lever_pivot_0"), (pivot_1, "lever_pivot_1")):
        ctx.check(
            f"{name} is revolute about the valve left-right axis",
            pivot.articulation_type == ArticulationType.REVOLUTE
            and tuple(pivot.axis) == (0.0, -1.0, 0.0)
            and pivot.motion_limits is not None
            and abs(pivot.motion_limits.lower + math.pi / 2.0) < 1e-6
            and abs(pivot.motion_limits.upper) < 1e-6,
        )

    # ----- Swivel pose: spout sweeps sideways
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

    return ctx.report()


object_model = build_object_model()
