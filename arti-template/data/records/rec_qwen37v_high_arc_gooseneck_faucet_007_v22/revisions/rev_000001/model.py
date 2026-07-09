from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Variant 22 — high-arc gooseneck faucet variant (fork of parent monobloc tap).
#
# Changes from parent:
#   1. Spout is lower (apex ~0.33 m vs 0.38 m) and wider (reach 0.16 m vs
#      0.144 m) with a flattened oval tube cross-section.
#   2. A small top flow knob at the spout apex rotates independently about
#      the vertical axis (revolute, -180..+180 deg).
#   3. Shallow chrome rib rings on the spray head.
#   4. A distinct hollow annular outlet opening at the spray head bottom.
#
# Retained from parent:
#   - Chrome base disc, gloss-black column, cross valve cylinder with flat
#     end caps, chrome swivel collar, two pin levers.
#   - Spout swivel (revolute about Z, ±110°) and lever pivots (revolute
#     about -Y, -90..0°).
# ---------------------------------------------------------------------------

# --- Column and base (unchanged from parent) ---
BASE_DISC_R = 0.042
BASE_DISC_H = 0.008
COLUMN_R = 0.020
COLUMN_TOP = 0.132

CROSS_Z = 0.085
CROSS_R = 0.0225
CROSS_TUBE_LEN = 0.170
CAP_LEN = 0.005
CAP_R = 0.0235
CAP_Y = CROSS_TUBE_LEN / 2.0 + CAP_LEN / 2.0

LEVER_Y = 0.058
BOSS_R = 0.010
BOSS_LEN = 0.016
BOSS_Z = 0.026
PIN_R = 0.006
PIN_LEN = 0.100
PIN_Z0 = 0.032

COLLAR_R = 0.0215
COLLAR_LEN = 0.010
SWIVEL_Z = 0.140

# --- Variant 22: lower, wider gooseneck with flattened oval tube ---
RISER_H = 0.10       # riser height in spout-local frame
ARC_R = 0.080        # arc radius (semicircle in XZ)
REACH_X = 2.0 * ARC_R  # 0.16 m horizontal reach (wider than parent's 0.144)
DROP_END = 0.06      # drop-leg end z in spout-local (world z = 0.200)

# Flattened oval cross-section: wider laterally (Y), flatter in bend plane (X)
OVAL_A = 0.009       # semi-axis in bend plane (XZ) — 18 mm total
OVAL_B = 0.014       # semi-axis lateral (Y) — 28 mm total

APEX_WORLD = SWIVEL_Z + RISER_H + ARC_R + OVAL_A  # ≈ 0.329 m

# --- Spray head ---
SPRAY_R = 0.016
SPRAY_LEN = 0.035
RIB_COUNT = 4
RIB_THICKNESS = 0.0015
RIB_OUTER = SPRAY_R + 0.001   # ribs sit 1 mm proud of the spray body

# --- Hollow outlet ---
OUTLET_OUTER_R = 0.014
OUTLET_INNER_R = 0.008
OUTLET_LEN = 0.008

# --- Flow knob ---
KNOB_DIAMETER = 0.020
KNOB_HEIGHT = 0.012
STEM_R = 0.004
STEM_LEN = 0.006

SWIVEL_LIMIT = math.radians(110.0)


# ----- geometry builders ---------------------------------------------------

def _gooseneck_shape() -> cq.Workplane:
    """Lower, wider gooseneck with flattened oval cross-section."""
    path = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, RISER_H)
        .threePointArc((ARC_R, RISER_H + ARC_R), (REACH_X, RISER_H))
        .lineTo(REACH_X, DROP_END)
    )
    # ellipse(a1, a2): a1 along X (bend plane, narrower), a2 along Y (wider)
    return cq.Workplane("XY").ellipse(OVAL_A, OVAL_B).sweep(path)


def _spray_head_shape() -> cq.Workplane:
    """Spray head body — slightly wider cylinder at the drop end."""
    return cq.Workplane("XY").circle(SPRAY_R).extrude(SPRAY_LEN)


def _rib_ring_shape() -> cq.Workplane:
    """Thin annular rib ring for spray head ribbing."""
    return (
        cq.Workplane("XY")
        .circle(RIB_OUTER)
        .circle(SPRAY_R)
        .extrude(RIB_THICKNESS)
    )


def _hollow_outlet_shape() -> cq.Workplane:
    """Annular outlet ring — hollow opening visible from below."""
    return (
        cq.Workplane("XY")
        .circle(OUTLET_OUTER_R)
        .circle(OUTLET_INNER_R)
        .extrude(OUTLET_LEN)
    )


# ----- model ---------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_arc_gooseneck_faucet_v22")

    gloss_black = model.material("veined_gloss_black", rgba=(0.075, 0.075, 0.085, 1.0))
    matte_black = model.material("matte_black", rgba=(0.035, 0.035, 0.038, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    outlet_dark = model.material("outlet_dark", rgba=(0.04, 0.04, 0.04, 1.0))
    spray_chrome = model.material("spray_chrome", rgba=(0.78, 0.80, 0.82, 1.0))

    # ----------------------------------------------------------------- column
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
    column.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z - COLLAR_LEN / 2.0)),
        material=chrome,
        name="swivel_collar",
    )

    # ------------------------------------------------------- gooseneck spout
    spout = model.part("gooseneck_spout")
    spout.visual(
        mesh_from_cadquery(_gooseneck_shape(), "gooseneck_tube"),
        material=gloss_black,
        name="gooseneck_tube",
    )

    # Spray head body at the drop-leg end
    spray_z0 = DROP_END - SPRAY_LEN  # spout-local z of spray head bottom
    spout.visual(
        mesh_from_cadquery(_spray_head_shape(), "spray_head_body"),
        origin=Origin(xyz=(REACH_X, 0.0, spray_z0)),
        material=spray_chrome,
        name="spray_head_body",
    )

    # Shallow ribbing — four thin chrome rings on the spray head
    for i in range(RIB_COUNT):
        z_pos = spray_z0 + 0.005 + i * 0.008
        spout.visual(
            mesh_from_cadquery(_rib_ring_shape(), f"spray_rib_{i}"),
            origin=Origin(xyz=(REACH_X, 0.0, z_pos)),
            material=chrome,
            name=f"spray_rib_{i}",
        )

    # Hollow outlet ring below the spray head
    outlet_z0 = spray_z0 - OUTLET_LEN
    spout.visual(
        mesh_from_cadquery(_hollow_outlet_shape(), "outlet_ring"),
        origin=Origin(xyz=(REACH_X, 0.0, outlet_z0)),
        material=chrome,
        name="outlet_ring",
    )
    # Dark interior showing the hollow opening
    spout.visual(
        Cylinder(radius=OUTLET_INNER_R, length=0.004),
        origin=Origin(xyz=(REACH_X, 0.0, outlet_z0 + 0.002)),
        material=outlet_dark,
        name="outlet_hollow",
    )

    # Knob mounting stem at the spout apex — embeds 2 mm into the tube for
    # connectivity (the tube surface is curved; exact contact is unreliable).
    stem_base_z = RISER_H + ARC_R + OVAL_A  # top of tube at apex
    stem_embed = 0.002
    spout.visual(
        Cylinder(radius=STEM_R, length=STEM_LEN + stem_embed),
        origin=Origin(xyz=(ARC_R, 0.0, stem_base_z - stem_embed + (STEM_LEN + stem_embed) / 2.0)),
        material=chrome,
        name="knob_stem",
    )

    # Spout swivel articulation (same as parent)
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

    # ----------------------------------------------------------- flow knob
    flow_knob = model.part("flow_knob")
    knob_geom = KnobGeometry(
        KNOB_DIAMETER,
        KNOB_HEIGHT,
        body_style="cylindrical",
        grip=KnobGrip(style="ribbed", count=12, depth=0.0008),
        center=False,  # mounting face at z = 0
    )
    flow_knob.visual(
        mesh_from_geometry(knob_geom, "flow_knob_body"),
        material=matte_black,
        name="flow_knob_body",
    )

    model.articulation(
        "knob_rotate",
        ArticulationType.REVOLUTE,
        parent=spout,
        child=flow_knob,
        origin=Origin(xyz=(ARC_R, 0.0, stem_base_z + STEM_LEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=-math.pi, upper=math.pi
        ),
    )

    # --------------------------------------------------------- pin levers
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


# ----- tests ---------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    column = object_model.get_part("body_column")
    spout = object_model.get_part("gooseneck_spout")
    flow_knob = object_model.get_part("flow_knob")
    lever_0 = object_model.get_part("pin_lever_0")
    lever_1 = object_model.get_part("pin_lever_1")

    swivel = object_model.get_articulation("spout_swivel")
    knob_joint = object_model.get_articulation("knob_rotate")
    pivot_0 = object_model.get_articulation("lever_pivot_0")
    pivot_1 = object_model.get_articulation("lever_pivot_1")

    # Intentional overlap allowances (same as parent)
    ctx.allow_overlap(
        lever_0, column,
        elem_a="lever_boss", elem_b="cross_tube",
        reason="Lever boss seats a few mm into the valve cylinder.",
    )
    ctx.allow_overlap(
        lever_1, column,
        elem_a="lever_boss", elem_b="cross_tube",
        reason="Lever boss seats a few mm into the valve cylinder.",
    )

    # ===== Variant 22 specific checks =====

    # 1. Spout apex is lower than parent (0.38 m) — should be ~0.32–0.35 m
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "gooseneck apex is lower than parent (0.30–0.35 m vs 0.38 m)",
        spout_aabb is not None and 0.30 <= spout_aabb[1][2] <= 0.35,
        details=f"spout aabb={spout_aabb}",
    )

    # 2. Spout reach is wider than parent (0.144 m)
    ctx.check(
        "gooseneck reach is wider than parent (>= 0.15 m vs 0.144 m)",
        spout_aabb is not None and spout_aabb[1][0] >= 0.15,
        details=f"spout aabb={spout_aabb}",
    )

    # 3. Flattened oval tube — lateral span >= 0.025 m (2*OVAL_B = 0.028)
    tube = ctx.part_element_world_aabb(spout, elem="gooseneck_tube")
    ctx.check(
        "tube has flattened oval section (lateral span >= 0.025 m)",
        tube is not None and (tube[1][1] - tube[0][1]) >= 0.025,
        details=f"tube aabb={tube}",
    )

    # 4. Flow knob is a non-fixed revolute joint with meaningful range
    ctx.check(
        "flow knob is a non-fixed revolute joint with range > 1 rad",
        knob_joint.articulation_type == ArticulationType.REVOLUTE
        and knob_joint.motion_limits is not None
        and knob_joint.motion_limits.lower is not None
        and knob_joint.motion_limits.upper is not None
        and (knob_joint.motion_limits.upper - knob_joint.motion_limits.lower) > 1.0,
    )

    # 5. Flow knob at the spout apex (top of faucet)
    knob_aabb = ctx.part_world_aabb(flow_knob)
    ctx.check(
        "flow knob sits at or near the spout apex",
        knob_aabb is not None and spout_aabb is not None
        and knob_aabb[0][2] >= spout_aabb[1][2] - 0.025,
        details=f"knob={knob_aabb}, spout={spout_aabb}",
    )

    # 6. Spray head ribs exist
    for i in range(RIB_COUNT):
        rib = ctx.part_element_world_aabb(spout, elem=f"spray_rib_{i}")
        ctx.check(
            f"spray rib {i} exists on the spray head",
            rib is not None,
        )

    # 7. Hollow outlet — ring and dark interior are present and ordered
    outlet_ring = ctx.part_element_world_aabb(spout, elem="outlet_ring")
    outlet_hollow = ctx.part_element_world_aabb(spout, elem="outlet_hollow")
    ctx.check(
        "distinct hollow outlet opening below the spray head",
        outlet_ring is not None and outlet_hollow is not None
        and outlet_ring[0][2] < outlet_hollow[1][2]
        and outlet_hollow[0][2] < outlet_ring[1][2],
        details=f"ring={outlet_ring}, hollow={outlet_hollow}",
    )

    # 8. Knob rotation is independent (does not move the spout)
    with ctx.pose({knob_joint: 1.0}):
        spout_posed = ctx.part_world_aabb(spout)
    ctx.check(
        "knob rotation is independent (spout position unchanged)",
        spout_posed is not None and spout_aabb is not None
        and abs(spout_posed[1][0] - spout_aabb[1][0]) < 1e-6
        and abs(spout_posed[1][2] - spout_aabb[1][2]) < 1e-6,
    )

    # 9. At least one non-fixed joint present
    all_joints = [swivel, knob_joint, pivot_0, pivot_1]
    ctx.check(
        "at least one non-fixed articulated joint exists",
        any(j.articulation_type != ArticulationType.FIXED for j in all_joints),
    )

    # ===== Retained parent checks =====

    # Grounding
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "tap grounded on deck plane",
        col_aabb is not None and abs(col_aabb[0][2]) <= 0.002,
        details=f"column aabb={col_aabb}",
    )

    # Chrome base disc
    disc = ctx.part_element_world_aabb(column, elem="base_disc")
    ctx.check(
        "single chrome base disc on the deck",
        disc is not None
        and 0.080 <= (disc[1][0] - disc[0][0]) <= 0.090
        and (disc[1][2] - disc[0][2]) <= 0.010,
        details=f"base disc aabb={disc}",
    )

    # Column diameter
    shaft = ctx.part_element_world_aabb(column, elem="column_shaft")
    ctx.check(
        "vertical column is ~0.04 m diameter",
        shaft is not None and 0.038 <= (shaft[1][0] - shaft[0][0]) <= 0.042,
        details=f"column shaft aabb={shaft}",
    )

    # Spout arcs forward
    ctx.check(
        "gooseneck arcs forward over the sink (+X reach)",
        spout_aabb is not None and spout_aabb[1][0] >= 0.14,
        details=f"spout aabb={spout_aabb}",
    )

    # Collar below spout
    collar = ctx.part_element_world_aabb(column, elem="swivel_collar")
    ctx.check(
        "chrome collar sits below the spout",
        collar is not None and spout_aabb is not None
        and collar[1][2] <= spout_aabb[0][2] + 0.002,
        details=f"collar={collar}, spout={spout_aabb}",
    )
    ctx.expect_contact(
        spout, column,
        elem_a="gooseneck_tube", elem_b="swivel_collar",
        contact_tol=0.002,
        name="gooseneck riser seats on the chrome collar",
    )

    # Cross valve
    cross = ctx.part_element_world_aabb(column, elem="cross_tube")
    ctx.check(
        "cross-cylinder at mid-column height",
        cross is not None and 0.06 <= 0.5 * (cross[0][2] + cross[1][2]) <= 0.11,
        details=f"cross aabb={cross}",
    )

    # Spout swivel carries spray head sideways
    rest_spray = ctx.part_element_world_aabb(spout, elem="spray_head_body")
    with ctx.pose({swivel: 1.0}):
        sw_spray = ctx.part_element_world_aabb(spout, elem="spray_head_body")
    ctx.check(
        "spout swivel carries the spray head sideways about Z",
        rest_spray is not None and sw_spray is not None
        and abs(0.5 * (rest_spray[0][1] + rest_spray[1][1])) < 1e-3
        and abs(0.5 * (sw_spray[0][1] + sw_spray[1][1])) > 0.05,
        details=f"rest={rest_spray}, swiveled={sw_spray}",
    )

    # Swivel joint properties
    ctx.check(
        "spout swivel is revolute ±110° about vertical axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and tuple(swivel.axis) == (0.0, 0.0, 1.0)
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + SWIVEL_LIMIT) < 1e-6
        and abs(swivel.motion_limits.upper - SWIVEL_LIMIT) < 1e-6,
    )

    # Lever checks
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
            lever, column, axes="z",
            elem_a="lever_boss", elem_b="cross_tube",
            min_overlap=0.003,
            name=f"{name} boss seats into valve cylinder",
        )

    # Lever pivot properties
    for pivot, name in ((pivot_0, "lever_pivot_0"), (pivot_1, "lever_pivot_1")):
        ctx.check(
            f"{name} is revolute -90..0° about valve Y axis",
            pivot.articulation_type == ArticulationType.REVOLUTE
            and tuple(pivot.axis) == (0.0, -1.0, 0.0)
            and pivot.motion_limits is not None
            and abs(pivot.motion_limits.lower + math.pi / 2.0) < 1e-6
            and abs(pivot.motion_limits.upper) < 1e-6,
        )

    return ctx.report()


object_model = build_object_model()
