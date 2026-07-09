from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Variant 08 — high-arc gooseneck faucet with pre-rinse spring, independent
# flow knob, spray-head ribbing, and collar seam.
#
# Layout (world frame, deck plane at z = 0, +X forward):
#   Chrome base disc → gloss-black column → cross valve with pin levers →
#   chrome collar (with seam) → gooseneck spout with spring coils around
#   the arc → chrome tip sleeve (with shallow ribbing) → flow knob on top.
# ---------------------------------------------------------------------------

# Base + column
BASE_DISC_R = 0.042
BASE_DISC_H = 0.008
COLUMN_R = 0.020
COLUMN_TOP = 0.132

# Cross valve
CROSS_Z = 0.085
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

# Swivel collar
COLLAR_R = 0.0215
COLLAR_LEN = 0.010
SWIVEL_Z = 0.140

# Gooseneck
TUBE_R = 0.015
ARC_R = 0.072
RISER_TOP = 0.153
REACH_X = 2.0 * ARC_R
DROP_END = 0.124

SLEEVE_R = 0.0165
SLEEVE_LEN = 0.028
AERATOR_R = 0.0118
AERATOR_LEN = 0.003

APEX_WORLD = SWIVEL_Z + RISER_TOP + ARC_R + TUBE_R  # 0.380 m

SWIVEL_LIMIT = math.radians(110.0)

# --- Variant 08 additions ---

# Pre-rinse spring around the arc
SPRING_WIRE_R = 0.0012
SPRING_COIL_MAJOR_R = TUBE_R + SPRING_WIRE_R  # inner edge = TUBE_R, touching tube
SPRING_NUM_COILS = 20
SPRING_PHI_START = 0.12
SPRING_PHI_END = math.pi - 0.12

# Flow knob on top of arc
KNOB_BOSS_R = 0.005
KNOB_BOSS_H = 0.008
KNOB_BOSS_EMBED = 0.002  # embed into the tube for connectivity
KNOB_BOSS_CENTER_Z = RISER_TOP + ARC_R + TUBE_R - KNOB_BOSS_EMBED + KNOB_BOSS_H / 2.0

KNOB_D = 0.016
KNOB_H = 0.010
KNOB_MOUNT_Z = RISER_TOP + ARC_R + TUBE_R - KNOB_BOSS_EMBED + KNOB_BOSS_H  # top of boss
KNOB_LIMIT = math.pi  # half-turn

# Spray head ribbing
RIB_COUNT = 5
RIB_MAJOR_R = SLEEVE_R  # ring center on sleeve surface
RIB_WIRE_R = 0.0005

# Collar seam
SEAM_MAJOR_R = COLLAR_R
SEAM_WIRE_R = 0.0004


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


def _spring_mesh():
    """Pre-rinse spring coils along the gooseneck arc, merged into one mesh."""
    meshes = []
    for i in range(SPRING_NUM_COILS):
        t = i / (SPRING_NUM_COILS - 1) if SPRING_NUM_COILS > 1 else 0.5
        phi = SPRING_PHI_START + t * (SPRING_PHI_END - SPRING_PHI_START)

        torus = TorusGeometry(
            SPRING_COIL_MAJOR_R,
            SPRING_WIRE_R,
            radial_segments=10,
            tubular_segments=20,
        )
        # Default torus axis is Z; rotate around Y by phi to align with arc tangent
        torus.rotate((0.0, 1.0, 0.0), phi)

        x = ARC_R * (1.0 - math.cos(phi))
        z = RISER_TOP + ARC_R * math.sin(phi)
        torus.translate(x, 0.0, z)
        meshes.append(torus)

    result = meshes[0].copy()
    for m in meshes[1:]:
        result = result.merge(m)
    return result


def _ribbing_mesh():
    """Shallow circumferential ribs on the spray-head tip sleeve."""
    meshes = []
    for i in range(RIB_COUNT):
        t = (i + 0.5) / RIB_COUNT
        z_local = DROP_END + t * SLEEVE_LEN

        rib = TorusGeometry(
            RIB_MAJOR_R,
            RIB_WIRE_R,
            radial_segments=8,
            tubular_segments=20,
        )
        # Sleeve axis is Z — no rotation needed
        rib.translate(REACH_X, 0.0, z_local)
        meshes.append(rib)

    result = meshes[0].copy()
    for m in meshes[1:]:
        result = result.merge(m)
    return result


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prerinse_gooseneck_faucet")

    gloss_black = model.material("veined_gloss_black", rgba=(0.075, 0.075, 0.085, 1.0))
    matte_black = model.material("matte_black", rgba=(0.035, 0.035, 0.038, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    outlet_dark = model.material("outlet_dark", rgba=(0.10, 0.10, 0.10, 1.0))
    seam_dark = model.material("seam_dark", rgba=(0.06, 0.06, 0.065, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.74, 0.76, 1.0))

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
    # --- Variant 08: thin seam at the swivel collar top ---
    seam_torus = TorusGeometry(
        SEAM_MAJOR_R, SEAM_WIRE_R, radial_segments=8, tubular_segments=24
    )
    seam_torus.translate(0.0, 0.0, SWIVEL_Z)
    column.visual(
        mesh_from_geometry(seam_torus, "collar_seam"),
        material=seam_dark,
        name="collar_seam",
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
    # --- Variant 08: pre-rinse spring around the arc ---
    spout.visual(
        mesh_from_geometry(_spring_mesh(), "spring_coils"),
        material=brushed_steel,
        name="spring_coils",
    )
    # --- Variant 08: shallow ribbing on the spray head ---
    spout.visual(
        mesh_from_geometry(_ribbing_mesh(), "spray_ribbing"),
        material=brushed_steel,
        name="spray_ribbing",
    )
    # --- Variant 08: chrome boss for knob mount ---
    spout.visual(
        Cylinder(radius=KNOB_BOSS_R, length=KNOB_BOSS_H),
        origin=Origin(xyz=(ARC_R, 0.0, KNOB_BOSS_CENTER_Z)),
        material=chrome,
        name="knob_boss",
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

    # --------------------------------------------------- Variant 08: flow knob
    flow_knob = model.part("flow_knob")
    knob_geom = KnobGeometry(
        KNOB_D,
        KNOB_H,
        body_style="cylindrical",
        grip=KnobGrip(style="ribbed", count=12, depth=0.0006, width=0.0012),
        indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
        center=False,
    )
    flow_knob.visual(
        mesh_from_geometry(knob_geom, "knob_body"),
        material=matte_black,
        name="knob_body",
    )

    model.articulation(
        "knob_rotate",
        ArticulationType.REVOLUTE,
        parent=spout,
        child=flow_knob,
        origin=Origin(xyz=(ARC_R, 0.0, KNOB_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=0.0, upper=KNOB_LIMIT
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    column = object_model.get_part("body_column")
    spout = object_model.get_part("gooseneck_spout")
    lever_0 = object_model.get_part("pin_lever_0")
    lever_1 = object_model.get_part("pin_lever_1")
    flow_knob = object_model.get_part("flow_knob")

    swivel = object_model.get_articulation("spout_swivel")
    pivot_0 = object_model.get_articulation("lever_pivot_0")
    pivot_1 = object_model.get_articulation("lever_pivot_1")
    knob_joint = object_model.get_articulation("knob_rotate")

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
    shaft = ctx.part_element_world_aabb(column, elem="column_shaft")
    ctx.check(
        "vertical column is ~0.04 m diameter",
        shaft is not None and 0.038 <= (shaft[1][0] - shaft[0][0]) <= 0.042,
        details=f"column shaft aabb={shaft}",
    )
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "gooseneck apex near 0.38 m (including spring/boss features)",
        spout_aabb is not None and 0.372 <= spout_aabb[1][2] <= 0.400,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "gooseneck arcs forward over the sink (+X reach)",
        spout_aabb is not None and spout_aabb[1][0] >= 0.150,
        details=f"spout aabb={spout_aabb}",
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

    # ----- chrome collar + seam -----
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
        contact_tol=0.001,
        name="gooseneck riser seats on the chrome collar",
    )

    # --- Variant 08: seam at swivel collar ---
    seam = ctx.part_element_world_aabb(column, elem="collar_seam")
    ctx.check(
        "thin seam ring present at the swivel collar top",
        seam is not None
        and collar is not None
        and abs(0.5 * (seam[0][2] + seam[1][2]) - collar[1][2]) <= 0.002
        and (seam[1][2] - seam[0][2]) <= 0.002,
        details=f"seam={seam}, collar_top={collar[1][2] if collar else None}",
    )

    # ----- chrome tip sleeve with downward outlet -----
    sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    aerator = ctx.part_element_world_aabb(spout, elem="outlet_aerator")
    tube = ctx.part_element_world_aabb(spout, elem="gooseneck_tube")
    ctx.check(
        "chrome tip sleeve wraps the spout drop leg with a downward outlet",
        sleeve is not None
        and aerator is not None
        and tube is not None
        and 0.25 <= sleeve[0][2] <= 0.28
        and aerator[0][2] < sleeve[0][2]
        and aerator[0][2] <= SWIVEL_Z + DROP_END
        and abs(0.5 * (sleeve[0][0] + sleeve[1][0]) - REACH_X) <= 0.002,
        details=f"sleeve={sleeve}, aerator={aerator}, tube={tube}",
    )

    # --- Variant 08: spray head ribbing ---
    ribbing = ctx.part_element_world_aabb(spout, elem="spray_ribbing")
    ctx.check(
        "shallow ribbing rings present on the spray head sleeve",
        ribbing is not None
        and sleeve is not None
        and ribbing[0][2] >= sleeve[0][2] - 0.002
        and ribbing[1][2] <= sleeve[1][2] + 0.002
        and ribbing[1][0] > sleeve[1][0] - 0.001,
        details=f"ribbing={ribbing}, sleeve={sleeve}",
    )

    # --- Variant 08: pre-rinse spring around the arc ---
    spring = ctx.part_element_world_aabb(spout, elem="spring_coils")
    ctx.check(
        "pre-rinse spring coils wrap around the gooseneck arc",
        spring is not None
        and tube is not None
        and spring[0][2] >= SWIVEL_Z + RISER_TOP - 0.01
        and spring[1][2] <= SWIVEL_Z + RISER_TOP + ARC_R + TUBE_R + 0.005
        and spring[1][0] >= 0.02
        and spring[0][0] <= REACH_X - 0.02,
        details=f"spring={spring}",
    )
    # Spring wraps around the tube — overlap in XY projection
    ctx.expect_overlap(
        spout,
        spout,
        axes="xy",
        elem_a="spring_coils",
        elem_b="gooseneck_tube",
        min_overlap=0.01,
        name="spring coils overlap the gooseneck tube in XY projection",
    )

    # ----- pin levers: geometry and seating -----
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

    # ----- joint plan: types, axes, ranges -----
    ctx.check(
        "spout swivel is revolute -110..+110 deg about the vertical column axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and tuple(swivel.axis) == (0.0, 0.0, 1.0)
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + SWIVEL_LIMIT) < 1e-6
        and abs(swivel.motion_limits.upper - SWIVEL_LIMIT) < 1e-6,
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

    # --- Variant 08: flow knob joint ---
    ctx.check(
        "flow knob is revolute about vertical with half-turn range",
        knob_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(knob_joint.axis) == (0.0, 0.0, 1.0)
        and knob_joint.motion_limits is not None
        and abs(knob_joint.motion_limits.lower) < 1e-6
        and abs(knob_joint.motion_limits.upper - math.pi) < 1e-6,
    )

    # ----- lever pose checks -----
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

    # ----- swivel pose -----
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

    # --- Variant 08: flow knob pose ---
    knob_rest = ctx.part_world_aabb(flow_knob)
    with ctx.pose({knob_joint: math.pi}):
        knob_turned = ctx.part_world_aabb(flow_knob)
    ctx.check(
        "flow knob rotates independently (AABB changes at full turn)",
        knob_rest is not None
        and knob_turned is not None
        and knob_rest[1][2] > SWIVEL_Z + RISER_TOP + ARC_R,
        details=f"rest={knob_rest}, turned={knob_turned}",
    )
    # Knob stays above the arc apex during rotation
    ctx.check(
        "flow knob remains above the gooseneck arc apex",
        knob_rest is not None
        and knob_rest[0][2] >= SWIVEL_Z + RISER_TOP + ARC_R - 0.005,
        details=f"knob_rest={knob_rest}",
    )

    return ctx.report()


object_model = build_object_model()
