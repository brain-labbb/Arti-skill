from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Variant 04 – high-arc gooseneck faucet with side-mounted mixer body.
#
# Layout (world frame, deck at z = 0, +X = user-facing / spout reach):
#
# - Chrome base disc on the deck; gloss-black column (0.04 m dia) rises on Z.
# - A side-mounted mixer body (horizontal cylinder, 0.048 m dia, 0.060 m
#   long) projects to the +Y side of the column at z ≈ 0.085, connected by
#   a short neck.  A flat matte-black end cap closes the outer end.
# - A single slim pin lever (0.012 m dia, 0.10 m long) rises from the top
#   of the mixer housing.  Revolute joint about the mixer's Y axis; q in
#   [-90°, 0°] tilts the pin from vertical toward the user (+X).
# - A thin chrome collar ring separates the column from the high-arc
#   gooseneck spout (swivel revolute, -110°..+110° about Z).
# - The gooseneck tube arcs up to an apex ≈ 0.38 m and drops to a ribbed
#   chrome spray-head sleeve (5 shallow circumferential ribs) with a
#   distinct hollow outlet tube below.
# ---------------------------------------------------------------------------

# Base + column
BASE_DISC_R = 0.042
BASE_DISC_H = 0.008
COLUMN_R = 0.020
COLUMN_TOP = 0.132

# Side mixer body
MIXER_Z = 0.085
MIXER_R = 0.024
MIXER_LEN = 0.060
MIXER_Y = 0.060
NECK_R = 0.014
NECK_LEN = 0.024
NECK_Y = 0.030
MIXER_CAP_R = 0.025
MIXER_CAP_LEN = 0.004
MIXER_CAP_Y = MIXER_Y + MIXER_LEN / 2.0 + MIXER_CAP_LEN / 2.0

# Side lever (lever-local frame at pivot)
LEVER_BOSS_R = 0.010
LEVER_BOSS_LEN = 0.016
LEVER_BOSS_Z = 0.005
LEVER_PIN_R = 0.006
LEVER_PIN_LEN = 0.100
LEVER_PIN_Z0 = 0.010
LEVER_PIVOT_Z = MIXER_Z + MIXER_R  # 0.109
LEVER_PIVOT_Y = MIXER_Y

# Collar + gooseneck
COLLAR_R = 0.0215
COLLAR_LEN = 0.010
SWIVEL_Z = 0.140

TUBE_R = 0.015
ARC_R = 0.072
RISER_TOP = 0.153
REACH_X = 2.0 * ARC_R  # 0.144
DROP_END = 0.124

# Ribbed sleeve
SLEEVE_R = 0.0165
SLEEVE_LEN = 0.028
RIB_R = 0.0178
RIB_H = 0.0015
RIB_COUNT = 5

# Hollow outlet
OUTLET_OUTER_R = 0.013
OUTLET_BORE_R = 0.008
OUTLET_LEN = 0.012

APEX_WORLD = SWIVEL_Z + RISER_TOP + ARC_R + TUBE_R
SWIVEL_LIMIT = math.radians(110.0)


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


def _ribbed_sleeve_shape() -> cq.Workplane:
    """Chrome tip sleeve with shallow circumferential ribs fused as one solid."""
    body = cq.Workplane("XY").circle(SLEEVE_R).extrude(SLEEVE_LEN)
    spacing = SLEEVE_LEN / (RIB_COUNT + 1)
    for i in range(RIB_COUNT):
        z_c = spacing * (i + 1)
        rib = (
            cq.Workplane("XY")
            .workplane(offset=z_c - RIB_H / 2.0)
            .circle(RIB_R)
            .extrude(RIB_H)
        )
        body = body.union(rib)
    return body


def _hollow_outlet_shape() -> cq.Workplane:
    """Short annular tube with a visible bore – the hollow outlet opening."""
    return (
        cq.Workplane("XY")
        .circle(OUTLET_OUTER_R)
        .circle(OUTLET_BORE_R)
        .extrude(OUTLET_LEN)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_arc_gooseneck_faucet_v04")

    gloss_black = model.material("veined_gloss_black", rgba=(0.075, 0.075, 0.085, 1.0))
    matte_black = model.material("matte_black", rgba=(0.035, 0.035, 0.038, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    outlet_dark = model.material("outlet_dark", rgba=(0.04, 0.04, 0.04, 1.0))

    # ---- body_column (root) -----------------------------------------------
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
    column.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z - COLLAR_LEN / 2.0)),
        material=chrome,
        name="swivel_collar",
    )

    # Side mixer body (neck + housing + end cap)
    column.visual(
        Cylinder(radius=NECK_R, length=NECK_LEN),
        origin=Origin(xyz=(0.0, NECK_Y, MIXER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gloss_black,
        name="mixer_neck",
    )
    column.visual(
        Cylinder(radius=MIXER_R, length=MIXER_LEN),
        origin=Origin(xyz=(0.0, MIXER_Y, MIXER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gloss_black,
        name="mixer_housing",
    )
    column.visual(
        Cylinder(radius=MIXER_CAP_R, length=MIXER_CAP_LEN),
        origin=Origin(xyz=(0.0, MIXER_CAP_Y, MIXER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="mixer_end_cap",
    )

    # ---- gooseneck_spout --------------------------------------------------
    spout = model.part("gooseneck_spout")

    spout.visual(
        mesh_from_cadquery(_gooseneck_shape(), "gooseneck_tube"),
        material=gloss_black,
        name="gooseneck_tube",
    )
    # Ribbed chrome spray-head sleeve (local z = 0 at sleeve bottom)
    spout.visual(
        mesh_from_cadquery(_ribbed_sleeve_shape(), "ribbed_sleeve"),
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END)),
        material=chrome,
        name="ribbed_sleeve",
    )
    # Hollow outlet tube hanging below the sleeve
    spout.visual(
        mesh_from_cadquery(_hollow_outlet_shape(), "hollow_outlet"),
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END - OUTLET_LEN)),
        material=outlet_dark,
        name="hollow_outlet",
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

    # ---- side_lever -------------------------------------------------------
    lever = model.part("side_lever")

    lever.visual(
        Cylinder(radius=LEVER_BOSS_R, length=LEVER_BOSS_LEN),
        origin=Origin(xyz=(0.0, 0.0, LEVER_BOSS_Z)),
        material=gloss_black,
        name="lever_boss",
    )
    lever.visual(
        Cylinder(radius=LEVER_PIN_R, length=LEVER_PIN_LEN),
        origin=Origin(xyz=(0.0, 0.0, LEVER_PIN_Z0 + LEVER_PIN_LEN / 2.0)),
        material=gloss_black,
        name="lever_pin",
    )

    model.articulation(
        "side_lever_pivot",
        ArticulationType.REVOLUTE,
        parent=column,
        child=lever,
        origin=Origin(xyz=(0.0, LEVER_PIVOT_Y, LEVER_PIVOT_Z)),
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
    lever = object_model.get_part("side_lever")

    swivel = object_model.get_articulation("spout_swivel")
    lever_pivot = object_model.get_articulation("side_lever_pivot")

    # Intentional seated insertion: lever boss embeds into mixer housing top.
    ctx.allow_overlap(
        lever,
        column,
        elem_a="lever_boss",
        elem_b="mixer_housing",
        reason="Lever boss intentionally seats a few mm into the mixer housing top.",
    )

    # ----- grounding and scale
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "faucet grounded on deck plane",
        col_aabb is not None and abs(col_aabb[0][2]) <= 0.002,
        details=f"column aabb={col_aabb}",
    )
    disc = ctx.part_element_world_aabb(column, elem="base_disc")
    ctx.check(
        "single chrome base disc sits on the deck (one deck hole)",
        disc is not None
        and 0.080 <= (disc[1][0] - disc[0][0]) <= 0.090
        and (disc[1][2] - disc[0][2]) <= 0.010,
        details=f"base disc aabb={disc}",
    )

    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "high-arc gooseneck apex near 0.38 m",
        spout_aabb is not None and 0.370 <= spout_aabb[1][2] <= 0.390,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "gooseneck arcs forward over the sink (+X reach)",
        spout_aabb is not None and spout_aabb[1][0] >= 0.140,
        details=f"spout aabb={spout_aabb}",
    )

    # ----- side-mounted mixer body
    housing = ctx.part_element_world_aabb(column, elem="mixer_housing")
    neck = ctx.part_element_world_aabb(column, elem="mixer_neck")
    cap = ctx.part_element_world_aabb(column, elem="mixer_end_cap")
    ctx.check(
        "side-mounted mixer housing projects to the side of the column",
        housing is not None
        and neck is not None
        and housing[0][1] > 0.015
        and 0.040 <= (housing[1][2] - housing[0][2]) <= 0.055,
        details=f"housing={housing}, neck={neck}",
    )
    ctx.check(
        "mixer housing closed with a flat end cap",
        cap is not None
        and housing is not None
        and cap[0][1] >= housing[1][1] - 0.002,
        details=f"cap={cap}, housing={housing}",
    )

    # ----- chrome collar ring
    collar = ctx.part_element_world_aabb(column, elem="swivel_collar")
    ctx.expect_contact(
        spout,
        column,
        elem_a="gooseneck_tube",
        elem_b="swivel_collar",
        contact_tol=0.001,
        name="gooseneck riser seats on the chrome collar",
    )

    # ----- ribbed spray head sleeve
    sleeve = ctx.part_element_world_aabb(spout, elem="ribbed_sleeve")
    ctx.check(
        "ribbed spray-head sleeve at spout tip with visible ribs",
        sleeve is not None
        and 0.033 <= (sleeve[1][0] - sleeve[0][0]) <= 0.040
        and 0.025 <= (sleeve[1][2] - sleeve[0][2]) <= 0.032,
        details=f"ribbed sleeve aabb={sleeve}",
    )

    # ----- distinct hollow outlet opening
    outlet = ctx.part_element_world_aabb(spout, elem="hollow_outlet")
    ctx.check(
        "distinct hollow outlet tube hangs below the ribbed sleeve",
        outlet is not None
        and sleeve is not None
        and outlet[1][2] <= sleeve[0][2] + 0.002
        and 0.009 <= (outlet[1][2] - outlet[0][2]) <= 0.016
        and 0.022 <= (outlet[1][0] - outlet[0][0]) <= 0.030,
        details=f"outlet={outlet}, sleeve={sleeve}",
    )

    # ----- side lever geometry and seating
    pin = ctx.part_element_world_aabb(lever, elem="lever_pin")
    ctx.check(
        "single side lever pin is slim (0.012 m dia) and 0.10 m long",
        pin is not None
        and 0.010 <= (pin[1][0] - pin[0][0]) <= 0.014
        and 0.098 <= (pin[1][2] - pin[0][2]) <= 0.102,
        details=f"lever pin aabb={pin}",
    )
    ctx.check(
        "side lever rises from the top of the mixer housing",
        pin is not None
        and housing is not None
        and pin[0][2] >= housing[1][2] - 0.012
        and pin[0][1] > 0.020,
        details=f"pin={pin}, housing={housing}",
    )
    ctx.expect_overlap(
        lever,
        column,
        axes="z",
        elem_a="lever_boss",
        elem_b="mixer_housing",
        min_overlap=0.002,
        name="lever boss seats into mixer housing",
    )

    # ----- joint plan
    ctx.check(
        "spout swivel is revolute -110..+110 deg about vertical column axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and tuple(swivel.axis) == (0.0, 0.0, 1.0)
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + SWIVEL_LIMIT) < 1e-6
        and abs(swivel.motion_limits.upper - SWIVEL_LIMIT) < 1e-6,
    )
    ctx.check(
        "side lever pivot is revolute -90..0 deg about horizontal Y axis",
        lever_pivot.articulation_type == ArticulationType.REVOLUTE
        and tuple(lever_pivot.axis) == (0.0, -1.0, 0.0)
        and lever_pivot.motion_limits is not None
        and abs(lever_pivot.motion_limits.lower + math.pi / 2.0) < 1e-6
        and abs(lever_pivot.motion_limits.upper) < 1e-6
        and lever_pivot.mimic is None,
    )

    # ----- lever pose: full -90 deg tilt brings the pin toward the user (+X)
    rest_lev = ctx.part_world_aabb(lever)
    with ctx.pose({lever_pivot: -math.pi / 2.0}):
        tilted_lev = ctx.part_world_aabb(lever)
    ctx.check(
        "side lever tilts from vertical to horizontal toward the user at q=-90 deg",
        rest_lev is not None
        and tilted_lev is not None
        and tilted_lev[1][0] > rest_lev[1][0] + 0.08
        and tilted_lev[1][2] < LEVER_PIVOT_Z + 0.03,
        details=f"rest={rest_lev}, tilted={tilted_lev}",
    )

    # ----- swivel pose: spout sweeps sideways
    rest_sl = ctx.part_element_world_aabb(spout, elem="ribbed_sleeve")
    with ctx.pose({swivel: 1.0}):
        sw_sl = ctx.part_element_world_aabb(spout, elem="ribbed_sleeve")
    ctx.check(
        "spout swivel sweeps the spray head sideways about the vertical axis",
        rest_sl is not None
        and sw_sl is not None
        and abs(0.5 * (rest_sl[0][1] + rest_sl[1][1])) < 0.005
        and 0.5 * (sw_sl[0][1] + sw_sl[1][1]) > 0.08,
        details=f"rest={rest_sl}, swiveled={sw_sl}",
    )

    return ctx.report()


object_model = build_object_model()
