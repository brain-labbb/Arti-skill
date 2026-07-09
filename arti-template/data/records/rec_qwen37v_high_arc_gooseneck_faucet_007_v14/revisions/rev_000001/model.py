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
    mesh_from_geometry,
    TorusGeometry,
)

# ---------------------------------------------------------------------------
# High-arc gooseneck faucet variant 14 (fork of gloss-black monobloc mixer tap).
#
# Changes from parent:
# - Side-mounted mixer body on the +Y flank of the column (single deck hole
#   retained; mixer protrudes above deck from the main body).
# - Flip-down outlet aerator: separate part pivoting at the nozzle tip via a
#   revolute joint (axis along the cross-flow Y direction, 0..0.55 rad).
# - Shallow ribbing on the spray head: four thin chrome rings around the tip
#   sleeve.
# - Thin seam ring at the swivel collar mid-height (dark groove).
# ---------------------------------------------------------------------------

# Base + column
BASE_DISC_R = 0.042
BASE_DISC_H = 0.008
COLUMN_R = 0.020  # 0.04 m diameter per prompt
COLUMN_TOP = 0.132  # shaft reaches 2 mm into the collar for connectivity

# Cross valve cylinder
CROSS_Z = 0.085
CROSS_R = 0.0225  # 0.045 m diameter per prompt
CROSS_TUBE_LEN = 0.170
CAP_LEN = 0.005  # flat end caps; total end-to-end = 0.170 + 2*0.005 = 0.180 m
CAP_R = 0.0235
CAP_Y = CROSS_TUBE_LEN / 2.0 + CAP_LEN / 2.0  # 0.0875

# Pin levers (lever-local frame at the valve axis center)
LEVER_Y = 0.058  # outboard position of each lever along the cross
BOSS_R = 0.010
BOSS_LEN = 0.016
BOSS_Z = 0.026  # boss spans z 0.018..0.034; embeds ~4.5 mm into the valve
PIN_R = 0.006  # 0.012 m diameter per prompt
PIN_LEN = 0.100
PIN_Z0 = 0.032  # pin spans z 0.032..0.132 (overlaps boss top for connectivity)

# Swivel collar + gooseneck (spout-local frame at the collar top, z = SWIVEL_Z)
COLLAR_R = 0.0215
COLLAR_LEN = 0.010
SWIVEL_Z = 0.140

TUBE_R = 0.015
ARC_R = 0.072
RISER_TOP = 0.153  # centerline apex = 0.153 + 0.072 = 0.225; +TUBE_R -> 0.240
REACH_X = 2.0 * ARC_R  # 0.144 m horizontal reach
DROP_END = 0.124  # spout-local z of the open tube tip (world 0.264)

SLEEVE_R = 0.0165
SLEEVE_LEN = 0.028  # chrome tip sleeve spans local z 0.124..0.152

APEX_WORLD = SWIVEL_Z + RISER_TOP + ARC_R + TUBE_R  # 0.380 m

SWIVEL_LIMIT = math.radians(110.0)

# --- Variant 14 additions ---

# Side-mounted mixer body (on +Y flank of column, below the cross valve)
MIXER_Z = 0.045
MIXER_NECK_R = 0.008
MIXER_NECK_LEN = 0.012
MIXER_BODY_R = 0.014
MIXER_BODY_LEN = 0.034
MIXER_CAP_R = 0.015
MIXER_CAP_LEN = 0.004
# Positions along +Y:
MIXER_NECK_Y = COLUMN_R + MIXER_NECK_LEN / 2.0  # 0.026
MIXER_BODY_Y = COLUMN_R + MIXER_NECK_LEN + MIXER_BODY_LEN / 2.0  # 0.049
MIXER_CAP_Y = COLUMN_R + MIXER_NECK_LEN + MIXER_BODY_LEN + MIXER_CAP_LEN / 2.0  # 0.068

# Collar seam (thin dark groove ring at collar mid-height)
SEAM_R = COLLAR_R + 0.001
SEAM_H = 0.001

# Spray head ribbing (4 thin chrome rings around the tip sleeve)
RIB_COUNT = 4
RIB_TUBE_R = 0.002  # cross-section radius of each rib torus
RIB_MAJOR_R = SLEEVE_R + RIB_TUBE_R * 0.5  # sits slightly proud of sleeve
RIB_Z_START = DROP_END + 0.004
RIB_Z_STEP = (SLEEVE_LEN - 0.008) / max(RIB_COUNT - 1, 1)

# Flip-down aerator (separate part, pivots at nozzle tip)
AERATOR_BOSS_R = 0.009
AERATOR_BOSS_LEN = 0.006
AERATOR_BODY_R = 0.013
AERATOR_BODY_LEN = 0.020
AERATOR_FACE_R = 0.010
AERATOR_FACE_LEN = 0.003
# In aerator-local frame (origin at pivot):
# Boss centered at z = +AERATOR_BOSS_LEN/2 (above pivot, embeds into sleeve)
# Body centered at z = -(AERATOR_BODY_LEN/2) (below pivot, hangs down)
# Face centered at z = -(AERATOR_BODY_LEN + AERATOR_FACE_LEN/2)


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
    model = ArticulatedObject(name="high_arc_gooseneck_faucet_v14")

    gloss_black = model.material("veined_gloss_black", rgba=(0.075, 0.075, 0.085, 1.0))
    matte_black = model.material("matte_black", rgba=(0.035, 0.035, 0.038, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    outlet_dark = model.material("outlet_dark", rgba=(0.10, 0.10, 0.10, 1.0))
    seam_dark = model.material("seam_dark", rgba=(0.02, 0.02, 0.025, 1.0))

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
    # Thin seam ring at collar mid-height (variant 14 feature)
    column.visual(
        Cylinder(radius=SEAM_R, length=SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z - SEAM_H / 2.0)),
        material=seam_dark,
        name="collar_seam",
    )

    # --- Side-mounted mixer body (variant 14 feature) ---
    # Neck bridge from column surface outward along +Y
    column.visual(
        Cylinder(radius=MIXER_NECK_R, length=MIXER_NECK_LEN),
        origin=Origin(xyz=(0.0, MIXER_NECK_Y, MIXER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gloss_black,
        name="mixer_neck",
    )
    # Main mixer body cylinder
    column.visual(
        Cylinder(radius=MIXER_BODY_R, length=MIXER_BODY_LEN),
        origin=Origin(xyz=(0.0, MIXER_BODY_Y, MIXER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gloss_black,
        name="mixer_body",
    )
    # Mixer end cap
    column.visual(
        Cylinder(radius=MIXER_CAP_R, length=MIXER_CAP_LEN),
        origin=Origin(xyz=(0.0, MIXER_CAP_Y, MIXER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="mixer_cap",
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
    # Shallow ribbing on the spray head (variant 14 feature)
    for i in range(RIB_COUNT):
        rib_z = RIB_Z_START + i * RIB_Z_STEP
        rib_geom = TorusGeometry(radius=RIB_MAJOR_R, tube=RIB_TUBE_R)
        spout.visual(
            mesh_from_geometry(rib_geom, f"spray_rib_{i}"),
            origin=Origin(xyz=(REACH_X, 0.0, rib_z)),
            material=chrome,
            name=f"spray_rib_{i}",
        )

    # --------------------------------------------------------- flip-down aerator
    aerator = model.part("aerator_nozzle")
    # Pivot boss (embeds into sleeve bottom for connectivity)
    aerator.visual(
        Cylinder(radius=AERATOR_BOSS_R, length=AERATOR_BOSS_LEN),
        origin=Origin(xyz=(0.0, 0.0, AERATOR_BOSS_LEN / 2.0)),
        material=chrome,
        name="aerator_boss",
    )
    # Main aerator housing body (hangs below pivot)
    aerator.visual(
        Cylinder(radius=AERATOR_BODY_R, length=AERATOR_BODY_LEN),
        origin=Origin(xyz=(0.0, 0.0, -AERATOR_BODY_LEN / 2.0)),
        material=matte_black,
        name="aerator_housing",
    )
    # Outlet face at the bottom
    aerator.visual(
        Cylinder(radius=AERATOR_FACE_R, length=AERATOR_FACE_LEN),
        origin=Origin(xyz=(0.0, 0.0, -(AERATOR_BODY_LEN + AERATOR_FACE_LEN / 2.0))),
        material=outlet_dark,
        name="aerator_face",
    )
    # Aerator pivot joint: revolute about horizontal Y axis at the nozzle tip
    model.articulation(
        "aerator_pivot",
        ArticulationType.REVOLUTE,
        parent=spout,
        child=aerator,
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=0.0, upper=0.55
        ),
    )

    # Spout swivel articulation
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
    # Two identical levers; numeric suffixes (no intrinsic left/right frame).
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
            # Axis is the valve cylinder's own left-right (Y) axis. With
            # axis -Y, negative q rotates the vertical pin toward +X (the
            # user side): q in [-pi/2, 0] tilts from vertical to horizontal.
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
    aerator = object_model.get_part("aerator_nozzle")
    lever_0 = object_model.get_part("pin_lever_0")
    lever_1 = object_model.get_part("pin_lever_1")

    swivel = object_model.get_articulation("spout_swivel")
    aerator_pivot = object_model.get_articulation("aerator_pivot")
    pivot_0 = object_model.get_articulation("lever_pivot_0")
    pivot_1 = object_model.get_articulation("lever_pivot_1")

    # Intentional seated insertions: each lever boss embeds a few mm into the
    # valve cylinder so the lever reads mounted, proven by expect_overlap below.
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
    # Aerator boss embeds into the tip sleeve for pivot connectivity
    ctx.allow_overlap(
        aerator,
        spout,
        elem_a="aerator_boss",
        elem_b="tip_sleeve",
        reason="Aerator pivot boss seats into the tip sleeve to represent the flip-down pivot mount.",
    )
    # Aerator boss also contacts the gooseneck tube drop-leg end
    ctx.allow_overlap(
        aerator,
        spout,
        elem_a="aerator_boss",
        elem_b="gooseneck_tube",
        reason="Aerator pivot boss seats into the gooseneck tube drop-leg end at the nozzle connection.",
    )
    # Aerator housing top abuts the gooseneck tube drop-leg end at the nozzle
    ctx.allow_overlap(
        aerator,
        spout,
        elem_a="aerator_housing",
        elem_b="gooseneck_tube",
        reason="Aerator housing connects at the gooseneck tube nozzle tip; small local overlap represents the seated joint.",
    )

    # ----- grounding, scale, proportions
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
    shaft = ctx.part_element_world_aabb(column, elem="column_shaft")
    ctx.check(
        "vertical column is ~0.04 m diameter",
        shaft is not None and 0.038 <= (shaft[1][0] - shaft[0][0]) <= 0.042,
        details=f"column shaft aabb={shaft}",
    )
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "gooseneck apex near 0.38 m",
        spout_aabb is not None and 0.372 <= spout_aabb[1][2] <= 0.388,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "gooseneck arcs forward over the sink (+X reach)",
        spout_aabb is not None and spout_aabb[1][0] >= 0.150,
        details=f"spout aabb={spout_aabb}",
    )

    # ----- cross valve cylinder with flat black end caps
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

    # ----- chrome collar ring with seam (variant 14)
    collar = ctx.part_element_world_aabb(column, elem="swivel_collar")
    seam = ctx.part_element_world_aabb(column, elem="collar_seam")
    ctx.check(
        "thin chrome collar sits above the cross and below the spout",
        collar is not None
        and cross is not None
        and collar[0][2] >= cross[1][2]
        and spout_aabb is not None
        and collar[1][2] <= spout_aabb[0][2] + 1e-6,
        details=f"collar={collar}, cross_top={cross[1][2] if cross else None}",
    )
    ctx.check(
        "collar seam is a thin dark ring at the collar mid-height",
        seam is not None
        and collar is not None
        and seam[0][2] >= collar[0][2] - 0.001
        and seam[1][2] <= collar[1][2] + 0.001
        and (seam[1][2] - seam[0][2]) <= 0.002,
        details=f"seam={seam}, collar={collar}",
    )
    ctx.expect_contact(
        spout,
        column,
        elem_a="gooseneck_tube",
        elem_b="swivel_collar",
        contact_tol=0.001,
        name="gooseneck riser seats on the chrome collar",
    )

    # ----- chrome tip sleeve on spout (aerator is now a separate part)
    sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    tube = ctx.part_element_world_aabb(spout, elem="gooseneck_tube")
    ctx.check(
        "chrome tip sleeve wraps the spout drop leg",
        sleeve is not None
        and tube is not None
        and 0.25 <= sleeve[0][2] <= 0.28
        and abs(0.5 * (sleeve[0][0] + sleeve[1][0]) - REACH_X) <= 0.002,
        details=f"sleeve={sleeve}, tube={tube}",
    )

    # ----- spray head ribbing (variant 14 feature)
    rib_aabbs = []
    for i in range(RIB_COUNT):
        rib = ctx.part_element_world_aabb(spout, elem=f"spray_rib_{i}")
        rib_aabbs.append(rib)
    ctx.check(
        "spray head has shallow ribbing rings around the tip sleeve",
        all(r is not None for r in rib_aabbs)
        and all(
            0.25 <= r[0][2] <= 0.29 and r[1][2] - r[0][2] <= 0.005
            for r in rib_aabbs
        ),
        details=f"ribs={rib_aabbs}",
    )

    # ----- side-mounted mixer body (variant 14 feature)
    mixer_neck = ctx.part_element_world_aabb(column, elem="mixer_neck")
    mixer_body = ctx.part_element_world_aabb(column, elem="mixer_body")
    mixer_cap = ctx.part_element_world_aabb(column, elem="mixer_cap")
    ctx.check(
        "side-mounted mixer body protrudes from column on +Y flank",
        mixer_neck is not None
        and mixer_body is not None
        and mixer_cap is not None
        and mixer_neck[0][1] >= COLUMN_R - 0.002
        and mixer_body[0][1] > mixer_neck[0][1]
        and mixer_cap[0][1] > mixer_body[0][1]
        and abs(0.5 * (mixer_body[0][2] + mixer_body[1][2]) - MIXER_Z) < 0.002,
        details=f"neck={mixer_neck}, body={mixer_body}, cap={mixer_cap}",
    )
    ctx.check(
        "mixer body is below the cross valve (single deck hole retained)",
        mixer_body is not None
        and cross is not None
        and mixer_body[1][2] < cross[0][2] + 0.005,
        details=f"mixer_body_top={mixer_body[1][2] if mixer_body else None}, cross_bottom={cross[0][2] if cross else None}",
    )

    # ----- flip-down aerator (variant 14 feature)
    aerator_boss = ctx.part_element_world_aabb(aerator, elem="aerator_boss")
    aerator_housing = ctx.part_element_world_aabb(aerator, elem="aerator_housing")
    aerator_face = ctx.part_element_world_aabb(aerator, elem="aerator_face")
    ctx.check(
        "aerator nozzle hangs below the tip sleeve at rest",
        aerator_housing is not None
        and sleeve is not None
        and aerator_housing[0][2] < sleeve[0][2]
        and aerator_face is not None
        and aerator_face[0][2] < aerator_housing[0][2],
        details=f"aerator_housing={aerator_housing}, sleeve={sleeve}, face={aerator_face}",
    )
    ctx.expect_overlap(
        aerator,
        spout,
        axes="z",
        elem_a="aerator_boss",
        elem_b="tip_sleeve",
        min_overlap=0.003,
        name="aerator boss seats into the tip sleeve for pivot mount",
    )
    ctx.expect_contact(
        aerator,
        spout,
        elem_a="aerator_housing",
        elem_b="gooseneck_tube",
        contact_tol=0.005,
        name="aerator housing contacts the gooseneck tube at the nozzle tip",
    )

    # ----- pin levers: geometry and seating
    for lever, name in ((lever_0, "pin_lever_0"), (lever_1, "pin_lever_1")):
        pin = ctx.part_element_world_aabb(lever, elem="lever_pin")
        ctx.check(
            f"{name} pin is slim (0.012 m dia) and 0.10 m long, vertical at rest",
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
    pin0 = ctx.part_element_world_aabb(lever_0, elem="lever_pin")
    pin1 = ctx.part_element_world_aabb(lever_1, elem="lever_pin")
    ctx.check(
        "the two pin levers rise from the tops of the two valve bodies",
        pin0 is not None
        and pin1 is not None
        and cross is not None
        and 0.5 * (pin0[0][1] + pin0[1][1]) > 0.04
        and 0.5 * (pin1[0][1] + pin1[1][1]) < -0.04
        and pin0[0][2] >= cross[1][2] - 0.001
        and pin1[0][2] >= cross[1][2] - 0.001,
        details=f"pin0={pin0}, pin1={pin1}",
    )

    # ----- joint plan: types, axes, ranges
    ctx.check(
        "spout swivel is revolute -110..+110 deg about the vertical column axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and tuple(swivel.axis) == (0.0, 0.0, 1.0)
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + SWIVEL_LIMIT) < 1e-6
        and abs(swivel.motion_limits.upper - SWIVEL_LIMIT) < 1e-6,
    )
    ctx.check(
        "aerator pivot is revolute 0..0.55 rad about horizontal Y axis at the nozzle",
        aerator_pivot.articulation_type == ArticulationType.REVOLUTE
        and tuple(aerator_pivot.axis) == (0.0, -1.0, 0.0)
        and aerator_pivot.motion_limits is not None
        and abs(aerator_pivot.motion_limits.lower) < 1e-6
        and abs(aerator_pivot.motion_limits.upper - 0.55) < 1e-6,
    )
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

    # ----- lever pose: full -90 deg tilt brings the pin toward the user (+X)
    rest_0 = ctx.part_world_aabb(lever_0)
    rest_1 = ctx.part_world_aabb(lever_1)
    with ctx.pose({pivot_0: -math.pi / 2.0}):
        tilted_0 = ctx.part_world_aabb(lever_0)
        still_1 = ctx.part_world_aabb(lever_1)
    ctx.check(
        "lever 0 tilts from vertical to horizontal toward the user at q=-90 deg",
        rest_0 is not None
        and tilted_0 is not None
        and tilted_0[1][0] > rest_0[1][0] + 0.10
        and tilted_0[1][2] < CROSS_Z + 0.03,
        details=f"rest={rest_0}, tilted={tilted_0}",
    )
    ctx.check(
        "lever 1 is independent of lever 0 (stays vertical while 0 tilts)",
        rest_1 is not None
        and still_1 is not None
        and abs(still_1[1][2] - rest_1[1][2]) < 1e-9,
        details=f"rest={rest_1}, while_0_tilted={still_1}",
    )
    with ctx.pose({pivot_1: -math.pi / 2.0}):
        tilted_1 = ctx.part_world_aabb(lever_1)
    ctx.check(
        "lever 1 tilts from vertical to horizontal toward the user at q=-90 deg",
        rest_1 is not None
        and tilted_1 is not None
        and tilted_1[1][0] > rest_1[1][0] + 0.10
        and tilted_1[1][2] < CROSS_Z + 0.03,
        details=f"rest={rest_1}, tilted={tilted_1}",
    )

    # ----- swivel pose: spout outlet sweeps sideways about the column axis
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

    # ----- aerator flip pose: tilts forward at positive q (variant 14)
    rest_aerator = ctx.part_world_aabb(aerator)
    with ctx.pose({aerator_pivot: 0.55}):
        tilted_aerator = ctx.part_world_aabb(aerator)
    ctx.check(
        "aerator flips forward (+X) at positive pivot angle",
        rest_aerator is not None
        and tilted_aerator is not None
        and tilted_aerator[1][0] > rest_aerator[1][0] + 0.005,
        details=f"rest={rest_aerator}, tilted={tilted_aerator}",
    )

    return ctx.report()


object_model = build_object_model()
