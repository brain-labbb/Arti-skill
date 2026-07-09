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
# High-arc gooseneck faucet variant: faceted segmented neck, pull-out spray
# head on prismatic hose, visible hot/cold tick marks as geometry, removable
# circular deck plate under the base.
#
# Layout (world frame, deck plane at z = 0):
# - +X is front (spout reach), +Z is up.
# - A chrome deck plate sits below the base disc.
# - Chrome base disc, gloss-black column (0.04 m dia), horizontal cross
#   valve cylinder (0.045 m dia, 0.18 m end-to-end) with flat end caps.
# - Tick marks (cold=blue, neutral=black, hot=red) on each cap outer face.
# - Two pin levers (revolute about valve Y axis, -90..0 deg).
# - Chrome collar separates column from faceted gooseneck spout.
# - Spout arcs up and over via a polyline (polygonal) path for visible
#   segmented bends, apex ~0.38 m.
# - Pull-out spray head docks at the spout tip; prismatic hose joint lets
#   it slide downward 0..0.06 m.
# ---------------------------------------------------------------------------

# Base + column
BASE_DISC_R = 0.042
BASE_DISC_H = 0.008
COLUMN_R = 0.020
COLUMN_TOP = 0.132

# Cross valve cylinder
CROSS_Z = 0.085
CROSS_R = 0.0225
CROSS_TUBE_LEN = 0.170
CAP_LEN = 0.005
CAP_R = 0.0235
CAP_Y = CROSS_TUBE_LEN / 2.0 + CAP_LEN / 2.0  # 0.0875

# Pin levers
LEVER_Y = 0.058
BOSS_R = 0.010
BOSS_LEN = 0.016
BOSS_Z = 0.026
PIN_R = 0.006
PIN_LEN = 0.100
PIN_Z0 = 0.032

# Swivel collar
COLLAR_R = 0.0215
COLLAR_LEN = 0.010
SWIVEL_Z = 0.140

# Faceted gooseneck (spout-local frame at collar top, z = 0)
TUBE_R = 0.015
ARC_R = 0.072
RISER_TOP = 0.153
REACH_X = 2.0 * ARC_R  # 0.144
DROP_END = 0.124
N_ARC_SEGMENTS = 8  # polygonal arc segments for visible faceting

SLEEVE_R = 0.0165
SLEEVE_LEN = 0.028

APEX_WORLD = SWIVEL_Z + RISER_TOP + ARC_R + TUBE_R  # 0.380

SWIVEL_LIMIT = math.radians(110.0)

# Deck plate (new)
DECK_PLATE_R = 0.058
DECK_PLATE_H = 0.004

# Spray head (new)
SPRAY_COLLAR_R = 0.016
SPRAY_COLLAR_LEN = 0.006
SPRAY_BODY_R = 0.013
SPRAY_BODY_LEN = 0.040
SPRAY_OUTLET_R = 0.010
SPRAY_OUTLET_LEN = 0.003
HOSE_TRAVEL = 0.060

# Tick marks (new)
TICK_W = 0.004   # X width
TICK_T = 0.0015  # Y protrusion from cap face
TICK_L = 0.007   # Z length


def _faceted_gooseneck_shape() -> cq.Workplane:
    """Faceted swan-neck tube built from a polyline path (polygonal arc)."""
    pts = [(0.0, 0.0), (0.0, RISER_TOP)]
    for i in range(1, N_ARC_SEGMENTS):
        theta = math.pi * i / N_ARC_SEGMENTS
        x = ARC_R * (1.0 - math.cos(theta))
        z = RISER_TOP + ARC_R * math.sin(theta)
        pts.append((x, z))
    pts.append((REACH_X, RISER_TOP))
    pts.append((REACH_X, DROP_END))

    # Build path with explicit moveTo/lineTo for reliable wire construction
    path = cq.Workplane("XZ").moveTo(pts[0][0], pts[0][1])
    for p in pts[1:]:
        path = path.lineTo(p[0], p[1])
    return cq.Workplane("XY").circle(TUBE_R).sweep(path)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_arc_gooseneck_faucet")

    # Materials
    gloss_black = model.material("veined_gloss_black", rgba=(0.075, 0.075, 0.085, 1.0))
    matte_black = model.material("matte_black", rgba=(0.035, 0.035, 0.038, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    outlet_dark = model.material("outlet_dark", rgba=(0.10, 0.10, 0.10, 1.0))
    hot_red = model.material("hot_indicator", rgba=(0.70, 0.15, 0.15, 1.0))
    cold_blue = model.material("cold_indicator", rgba=(0.15, 0.20, 0.70, 1.0))

    # ---- deck plate (removable circular plate under the base) ----
    deck_plate = model.part("deck_plate")
    deck_plate.visual(
        Cylinder(radius=DECK_PLATE_R, length=DECK_PLATE_H),
        origin=Origin(xyz=(0.0, 0.0, -DECK_PLATE_H / 2.0)),
        material=chrome,
        name="deck_plate_disc",
    )

    # ---- body column ----
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
    # Horizontal cross valve cylinder (Y axis)
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
    # Chrome collar ring
    column.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z - COLLAR_LEN / 2.0)),
        material=chrome,
        name="swivel_collar",
    )

    # ---- tick marks on valve end cap outer faces ----
    cap_face_y_right = CAP_Y + CAP_LEN / 2.0 + TICK_T / 2.0
    cap_face_y_left = -(CAP_Y + CAP_LEN / 2.0 + TICK_T / 2.0)
    tick_z_offsets = [-0.010, 0.0, 0.010]
    tick_materials = [cold_blue, gloss_black, hot_red]

    for idx, (dz, mat) in enumerate(zip(tick_z_offsets, tick_materials)):
        column.visual(
            Box((TICK_W, TICK_T, TICK_L)),
            origin=Origin(xyz=(0.0, cap_face_y_right, CROSS_Z + dz)),
            material=mat,
            name=f"tick_right_{idx}",
        )
        column.visual(
            Box((TICK_W, TICK_T, TICK_L)),
            origin=Origin(xyz=(0.0, cap_face_y_left, CROSS_Z + dz)),
            material=mat,
            name=f"tick_left_{idx}",
        )

    # Fixed joint: deck plate under column base
    model.articulation(
        "deck_mount",
        ArticulationType.FIXED,
        parent=column,
        child=deck_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # ---- faceted gooseneck spout ----
    spout = model.part("gooseneck_spout")
    spout.visual(
        mesh_from_cadquery(_faceted_gooseneck_shape(), "faceted_gooseneck_tube"),
        material=gloss_black,
        name="gooseneck_tube",
    )
    # Chrome tip sleeve at the drop leg end
    spout.visual(
        Cylinder(radius=SLEEVE_R, length=SLEEVE_LEN),
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END + SLEEVE_LEN / 2.0)),
        material=chrome,
        name="tip_sleeve",
    )

    # Spout swivel about vertical column axis
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

    # ---- pull-out spray head on prismatic hose joint ----
    spray = model.part("spray_head")
    spray.visual(
        Cylinder(radius=SPRAY_COLLAR_R, length=SPRAY_COLLAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, -SPRAY_COLLAR_LEN / 2.0)),
        material=chrome,
        name="spray_collar",
    )
    spray.visual(
        Cylinder(radius=SPRAY_BODY_R, length=SPRAY_BODY_LEN),
        origin=Origin(xyz=(0.0, 0.0, -(SPRAY_COLLAR_LEN + SPRAY_BODY_LEN / 2.0))),
        material=gloss_black,
        name="spray_body",
    )
    spray.visual(
        Cylinder(radius=SPRAY_OUTLET_R, length=SPRAY_OUTLET_LEN),
        origin=Origin(
            xyz=(0.0, 0.0, -(SPRAY_COLLAR_LEN + SPRAY_BODY_LEN + SPRAY_OUTLET_LEN / 2.0))
        ),
        material=outlet_dark,
        name="spray_outlet",
    )

    # Prismatic hose joint: spray head slides out downward from spout tip
    model.articulation(
        "spray_hose",
        ArticulationType.PRISMATIC,
        parent=spout,
        child=spray,
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=10.0, velocity=0.5, lower=0.0, upper=HOSE_TRAVEL
        ),
    )

    # ---- pin levers (preserved from parent) ----
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
    spray = object_model.get_part("spray_head")
    deck = object_model.get_part("deck_plate")
    lever_0 = object_model.get_part("pin_lever_0")
    lever_1 = object_model.get_part("pin_lever_1")

    swivel = object_model.get_articulation("spout_swivel")
    hose = object_model.get_articulation("spray_hose")
    pivot_0 = object_model.get_articulation("lever_pivot_0")
    pivot_1 = object_model.get_articulation("lever_pivot_1")
    deck_mount = object_model.get_articulation("deck_mount")

    # -- intentional overlaps --
    ctx.allow_overlap(
        lever_0, column, elem_a="lever_boss", elem_b="cross_tube",
        reason="Lever boss intentionally seats into the valve cylinder.",
    )
    ctx.allow_overlap(
        lever_1, column, elem_a="lever_boss", elem_b="cross_tube",
        reason="Lever boss intentionally seats into the valve cylinder.",
    )
    ctx.allow_overlap(
        spout, spray,
        elem_a="gooseneck_tube", elem_b="spray_collar",
        reason="Spray collar docks into the spout tube tip as a seated pull-out interface.",
    )

    # =============================================================
    # VARIANT-SPECIFIC CHECKS
    # =============================================================

    # 1. Faceted neck: gooseneck preserved silhouette with segmented bends
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "gooseneck apex near 0.38 m (high-arc silhouette preserved)",
        spout_aabb is not None and 0.370 <= spout_aabb[1][2] <= 0.390,
        details=f"spout aabb={spout_aabb}",
    )
    tube = ctx.part_element_world_aabb(spout, elem="gooseneck_tube")
    ctx.check(
        "faceted gooseneck tube spans the full arc reach (+X)",
        tube is not None and tube[1][0] >= 0.130,
        details=f"tube aabb={tube}",
    )

    # 2. Spray head on prismatic hose joint
    ctx.check(
        "spray_hose is prismatic with 0..0.06 m travel along -Z",
        hose.articulation_type == ArticulationType.PRISMATIC
        and tuple(hose.axis) == (0.0, 0.0, -1.0)
        and hose.motion_limits is not None
        and abs(hose.motion_limits.lower) < 1e-6
        and abs(hose.motion_limits.upper - HOSE_TRAVEL) < 1e-6,
    )

    # Spray head docks at spout tip at rest
    spray_aabb = ctx.part_world_aabb(spray)
    sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    ctx.check(
        "spray head docks below the spout tip sleeve at rest",
        spray_aabb is not None and sleeve is not None
        and abs(spray_aabb[1][2] - sleeve[0][2]) <= 0.003,
        details=f"spray_top={spray_aabb[1][2] if spray_aabb else None}, "
                f"sleeve_bottom={sleeve[0][2] if sleeve else None}",
    )

    # Prismatic extension moves spray head downward
    rest_spray_pos = ctx.part_world_position(spray)
    with ctx.pose({hose: HOSE_TRAVEL}):
        ext_spray_pos = ctx.part_world_position(spray)
    ctx.check(
        "spray head extends downward when hose is pulled out",
        rest_spray_pos is not None and ext_spray_pos is not None
        and ext_spray_pos[2] < rest_spray_pos[2] - 0.04,
        details=f"rest={rest_spray_pos}, extended={ext_spray_pos}",
    )

    # Prove the docking collar separates from the spout sleeve when extended
    with ctx.pose({hose: HOSE_TRAVEL}):
        ctx.expect_gap(
            spout, spray,
            axis="z",
            positive_elem="tip_sleeve",
            negative_elem="spray_collar",
            min_gap=0.03,
            name="spray collar clears the spout sleeve when fully extended",
        )

    # 3. Cold/hot tick marks as geometry
    tick_names = [
        "tick_right_0", "tick_right_1", "tick_right_2",
        "tick_left_0", "tick_left_1", "tick_left_2",
    ]
    ticks_found = all(
        ctx.part_element_world_aabb(column, elem=name) is not None
        for name in tick_names
    )
    ctx.check(
        "cold/hot tick marks present as geometry on both valve caps",
        ticks_found,
    )

    # Ticks protrude from outer cap faces
    tick_r0 = ctx.part_element_world_aabb(column, elem="tick_right_0")
    tick_l0 = ctx.part_element_world_aabb(column, elem="tick_left_0")
    cap_0 = ctx.part_element_world_aabb(column, elem="valve_end_cap_0")
    cap_1 = ctx.part_element_world_aabb(column, elem="valve_end_cap_1")
    ctx.check(
        "tick marks sit on the outer faces of the valve end caps",
        tick_r0 is not None and cap_0 is not None
        and tick_l0 is not None and cap_1 is not None
        and tick_r0[0][1] >= cap_0[1][1] - 0.002
        and tick_l0[1][1] <= cap_1[0][1] + 0.002,
        details=f"tick_r0={tick_r0}, cap_0={cap_0}, tick_l0={tick_l0}, cap_1={cap_1}",
    )

    # 4. Removable circular deck plate
    ctx.check(
        "deck_mount is a fixed joint",
        deck_mount.articulation_type == ArticulationType.FIXED,
    )
    deck_aabb = ctx.part_world_aabb(deck)
    disc = ctx.part_element_world_aabb(column, elem="base_disc")
    ctx.check(
        "deck plate is wider than the base disc and sits at/below it",
        deck_aabb is not None and disc is not None
        and (deck_aabb[1][0] - deck_aabb[0][0]) > (disc[1][0] - disc[0][0]) + 0.02
        and deck_aabb[1][2] <= disc[0][2] + 0.002,
        details=f"deck={deck_aabb}, disc={disc}",
    )
    ctx.expect_contact(
        deck, column,
        elem_a="deck_plate_disc", elem_b="base_disc",
        contact_tol=0.002,
        name="deck plate contacts base disc at deck plane",
    )

    # =============================================================
    # PRESERVED PARENT CHECKS
    # =============================================================

    # Grounding and scale
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "tap grounded on the deck plane",
        col_aabb is not None and abs(col_aabb[0][2]) <= 0.002,
        details=f"column aabb={col_aabb}",
    )
    shaft = ctx.part_element_world_aabb(column, elem="column_shaft")
    ctx.check(
        "vertical column is ~0.04 m diameter",
        shaft is not None and 0.038 <= (shaft[1][0] - shaft[0][0]) <= 0.042,
        details=f"column shaft aabb={shaft}",
    )
    ctx.check(
        "gooseneck arcs forward over the sink (+X reach)",
        spout_aabb is not None and spout_aabb[1][0] >= 0.130,
        details=f"spout aabb={spout_aabb}",
    )

    # Cross valve cylinder with flat end caps
    cross = ctx.part_element_world_aabb(column, elem="cross_tube")
    cap_0_aabb = ctx.part_element_world_aabb(column, elem="valve_end_cap_0")
    cap_1_aabb = ctx.part_element_world_aabb(column, elem="valve_end_cap_1")
    ctx.check(
        "cross-cylinder is ~0.045 m diameter at mid-column height",
        cross is not None
        and 0.043 <= (cross[1][2] - cross[0][2]) <= 0.047
        and 0.06 <= 0.5 * (cross[0][2] + cross[1][2]) <= 0.11,
        details=f"cross aabb={cross}",
    )
    ctx.check(
        "valve assembly spans ~0.18 m end-to-end",
        cap_0_aabb is not None and cap_1_aabb is not None
        and 0.178 <= (cap_0_aabb[1][1] - cap_1_aabb[0][1]) <= 0.182,
        details=f"cap_0={cap_0_aabb}, cap_1={cap_1_aabb}",
    )

    # Chrome collar and spout seating
    collar = ctx.part_element_world_aabb(column, elem="swivel_collar")
    ctx.check(
        "thin chrome collar sits above the cross and below the spout",
        collar is not None and cross is not None
        and collar[0][2] >= cross[1][2]
        and spout_aabb is not None
        and collar[1][2] <= spout_aabb[0][2] + 1e-6,
        details=f"collar={collar}, cross_top={cross[1][2] if cross else None}",
    )
    ctx.expect_contact(
        spout, column,
        elem_a="gooseneck_tube", elem_b="swivel_collar",
        contact_tol=0.002,
        name="gooseneck riser seats on the chrome collar",
    )

    # Joint plan: swivel preserved
    ctx.check(
        "spout swivel is revolute ±110° about vertical column axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and tuple(swivel.axis) == (0.0, 0.0, 1.0)
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + SWIVEL_LIMIT) < 1e-6
        and abs(swivel.motion_limits.upper - SWIVEL_LIMIT) < 1e-6,
    )

    # Lever pivots preserved
    for pivot, name in ((pivot_0, "lever_pivot_0"), (pivot_1, "lever_pivot_1")):
        ctx.check(
            f"{name} is revolute -90..0° about the valve left-right axis",
            pivot.articulation_type == ArticulationType.REVOLUTE
            and tuple(pivot.axis) == (0.0, -1.0, 0.0)
            and pivot.motion_limits is not None
            and abs(pivot.motion_limits.lower + math.pi / 2.0) < 1e-6
            and abs(pivot.motion_limits.upper) < 1e-6
            and pivot.mimic is None,
        )

    # Lever seating
    for lever, name in ((lever_0, "pin_lever_0"), (lever_1, "pin_lever_1")):
        ctx.expect_overlap(
            lever, column,
            axes="z", elem_a="lever_boss", elem_b="cross_tube",
            min_overlap=0.003,
            name=f"{name} boss seats into the valve cylinder",
        )

    # Lever pose: tilt toward user
    rest_0 = ctx.part_world_aabb(lever_0)
    rest_1 = ctx.part_world_aabb(lever_1)
    with ctx.pose({pivot_0: -math.pi / 2.0}):
        tilted_0 = ctx.part_world_aabb(lever_0)
        still_1 = ctx.part_world_aabb(lever_1)
    ctx.check(
        "lever 0 tilts from vertical toward the user at q=-90°",
        rest_0 is not None and tilted_0 is not None
        and tilted_0[1][0] > rest_0[1][0] + 0.08
        and tilted_0[1][2] < CROSS_Z + 0.03,
        details=f"rest={rest_0}, tilted={tilted_0}",
    )
    ctx.check(
        "lever 1 is independent of lever 0",
        rest_1 is not None and still_1 is not None
        and abs(still_1[1][2] - rest_1[1][2]) < 1e-9,
        details=f"rest={rest_1}, while_0_tilted={still_1}",
    )

    # Swivel pose: spout sweeps sideways
    rest_sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    with ctx.pose({swivel: 1.0}):
        sw_sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    ctx.check(
        "spout swivel carries the tip sleeve sideways about the vertical axis",
        rest_sleeve is not None and sw_sleeve is not None
        and abs(0.5 * (rest_sleeve[0][1] + rest_sleeve[1][1])) < 1e-6
        and 0.5 * (sw_sleeve[0][1] + sw_sleeve[1][1]) > 0.08,
        details=f"rest={rest_sleeve}, swiveled={sw_sleeve}",
    )

    return ctx.report()


object_model = build_object_model()
