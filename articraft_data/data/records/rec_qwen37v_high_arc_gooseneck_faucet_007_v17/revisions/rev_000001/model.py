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
# High-arc gooseneck faucet variant (~0.38 m tall) with:
# - Pull-down spray head nested into the gooseneck mouth (prismatic, 0..80 mm)
# - Flip-down outlet aerator pivoting at the nozzle (revolute, 0..45 deg)
# - Visible cold/hot tick marks as raised geometry on valve end caps
# - Two small chrome mounting collars on the pedestal
#
# Layout (world frame, deck plane at z = 0):
# - +X is the front (spout reach direction), +Z is up.
# - Chrome base disc on deck; gloss-black column (0.04 m dia) rises on Z.
# - Two chrome mounting collars on pedestal below the cross valve.
# - Horizontal cross-cylinder (0.045 m dia, 0.18 m end-to-end) at z ≈ 0.085.
# - Hot/cold tick marks as raised geometry on each valve end cap.
# - Pin levers (0.012 m dia, 0.10 m) from each valve body top.
# - Chrome collar ring at z ≈ 0.135 separates column from gooseneck.
# - Gooseneck arcs to apex at z ≈ 0.38, ending in chrome tip sleeve.
# - Pull-down spray head nests into the sleeve (prismatic along -Z).
# - Flip-down aerator at spray head bottom (revolute about -Y).
# - Spout swivels about vertical axis (revolute ±110 deg).
# ---------------------------------------------------------------------------

# --- Base + column ---
BASE_DISC_R = 0.042
BASE_DISC_H = 0.008
COLUMN_R = 0.020
COLUMN_TOP = 0.132

# --- Mounting collars on pedestal ---
MOUNT_COLLAR_R = 0.023
MOUNT_COLLAR_LEN = 0.005
MOUNT_COLLAR_Z0 = 0.022  # lower collar center
MOUNT_COLLAR_Z1 = 0.045  # upper collar center

# --- Cross valve cylinder ---
CROSS_Z = 0.085
CROSS_R = 0.0225
CROSS_TUBE_LEN = 0.170
CAP_LEN = 0.005
CAP_R = 0.0235
CAP_Y = CROSS_TUBE_LEN / 2.0 + CAP_LEN / 2.0  # 0.0875

# --- Tick marks on valve end caps ---
TICK_SX = 0.002   # X extent
TICK_SY = 0.002   # Y extent (proud of cap face)
TICK_SZ = 0.008   # Z extent (vertical line)

# --- Pin levers ---
LEVER_Y = 0.058
BOSS_R = 0.010
BOSS_LEN = 0.016
BOSS_Z = 0.026
PIN_R = 0.006
PIN_LEN = 0.100
PIN_Z0 = 0.032

# --- Swivel collar + gooseneck ---
COLLAR_R = 0.0215
COLLAR_LEN = 0.010
SWIVEL_Z = 0.140

TUBE_R = 0.015
ARC_R = 0.072
RISER_TOP = 0.153
REACH_X = 2.0 * ARC_R  # 0.144 m horizontal reach
DROP_END = 0.124        # spout-local z of open tube tip

SLEEVE_R = 0.0165
SLEEVE_LEN = 0.028

APEX_WORLD = SWIVEL_Z + RISER_TOP + ARC_R + TUBE_R  # 0.380 m
SWIVEL_LIMIT = math.radians(110.0)

# --- Pull-down spray head ---
SPRAY_FACE_R = 0.012
SPRAY_FACE_LEN = 0.004
SPRAY_GRIP_R = 0.015
SPRAY_GRIP_LEN = 0.016
SPRAY_SHOULDER_R = 0.016
SPRAY_SHOULDER_LEN = 0.004
SPRAY_NEST_R = 0.013
SPRAY_NEST_LEN = 0.012

PULL_DOWN_MAX = 0.080  # 80 mm travel

# --- Flip-down aerator ---
AERATOR_R = 0.010
AERATOR_LEN = 0.003
AERATOR_FLIP_LIMIT = math.radians(45.0)


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


def _spray_head_body() -> cq.Workplane:
    """Stepped spray head: face plate, grip, shoulder, nest (into sleeve)."""
    z_face_bot = -(SPRAY_SHOULDER_LEN + SPRAY_GRIP_LEN + SPRAY_FACE_LEN)
    z_grip_bot = z_face_bot + SPRAY_FACE_LEN
    z_shoulder_bot = z_grip_bot + SPRAY_GRIP_LEN
    z_nest_bot = z_shoulder_bot + SPRAY_SHOULDER_LEN

    face = (
        cq.Workplane("XY")
        .workplane(offset=z_face_bot)
        .circle(SPRAY_FACE_R)
        .extrude(SPRAY_FACE_LEN)
    )
    grip = (
        cq.Workplane("XY")
        .workplane(offset=z_grip_bot)
        .circle(SPRAY_GRIP_R)
        .extrude(SPRAY_GRIP_LEN)
    )
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=z_shoulder_bot)
        .circle(SPRAY_SHOULDER_R)
        .extrude(SPRAY_SHOULDER_LEN)
    )
    nest = (
        cq.Workplane("XY")
        .workplane(offset=z_nest_bot)
        .circle(SPRAY_NEST_R)
        .extrude(SPRAY_NEST_LEN)
    )
    return face.union(grip).union(shoulder).union(nest)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_arc_gooseneck_faucet")

    # Materials
    gloss_black = model.material("veined_gloss_black", rgba=(0.075, 0.075, 0.085, 1.0))
    matte_black = model.material("matte_black", rgba=(0.035, 0.035, 0.038, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    outlet_dark = model.material("outlet_dark", rgba=(0.10, 0.10, 0.10, 1.0))
    hot_mark = model.material("hot_indicator", rgba=(0.70, 0.15, 0.12, 1.0))
    cold_mark = model.material("cold_indicator", rgba=(0.12, 0.20, 0.70, 1.0))

    # --------------------------------------------------------------- column
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
    # Mounting collars on pedestal
    column.visual(
        Cylinder(radius=MOUNT_COLLAR_R, length=MOUNT_COLLAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, MOUNT_COLLAR_Z0)),
        material=chrome,
        name="mount_collar_0",
    )
    column.visual(
        Cylinder(radius=MOUNT_COLLAR_R, length=MOUNT_COLLAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, MOUNT_COLLAR_Z1)),
        material=chrome,
        name="mount_collar_1",
    )
    # Cross valve cylinder
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
    # Hot/cold tick marks on valve end cap faces
    column.visual(
        Box((TICK_SX, TICK_SY, TICK_SZ)),
        origin=Origin(xyz=(0.0, CAP_Y + CAP_LEN / 2.0, CROSS_Z)),
        material=hot_mark,
        name="hot_tick",
    )
    column.visual(
        Box((TICK_SX, TICK_SY, TICK_SZ)),
        origin=Origin(xyz=(0.0, -(CAP_Y + CAP_LEN / 2.0), CROSS_Z)),
        material=cold_mark,
        name="cold_tick",
    )
    # Chrome collar ring
    column.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z - COLLAR_LEN / 2.0)),
        material=chrome,
        name="swivel_collar",
    )

    # ---------------------------------------------------------- gooseneck spout
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

    # -------------------------------------------------------- pull-down spray head
    spray_head = model.part("spray_head")
    spray_head.visual(
        mesh_from_cadquery(_spray_head_body(), "spray_body"),
        material=gloss_black,
        name="spray_body",
    )
    # Prismatic joint: parent=spout, child=spray_head
    # Origin at bottom of tip sleeve in spout frame; axis -Z so positive q
    # pulls the spray head downward.
    model.articulation(
        "spray_pull_down",
        ArticulationType.PRISMATIC,
        parent=spout,
        child=spray_head,
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=15.0, velocity=0.5, lower=0.0, upper=PULL_DOWN_MAX
        ),
    )

    # ------------------------------------------------------- flip-down aerator
    aerator = model.part("outlet_aerator")
    aerator.visual(
        Cylinder(radius=AERATOR_R, length=AERATOR_LEN),
        # Disc center offset from pivot in aerator frame: extends in +X
        origin=Origin(xyz=(AERATOR_R, 0.0, 0.0)),
        material=chrome,
        name="aerator_disc",
    )
    # Revolute joint: parent=spray_head, child=aerator
    # Pivot at the back (-X) edge of the spray head bottom face.
    # axis = -Y so positive q drops the +X edge downward (flip open).
    spray_face_bot = -(SPRAY_SHOULDER_LEN + SPRAY_GRIP_LEN + SPRAY_FACE_LEN)
    model.articulation(
        "aerator_flip",
        ArticulationType.REVOLUTE,
        parent=spray_head,
        child=aerator,
        origin=Origin(xyz=(-AERATOR_R, 0.0, spray_face_bot)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=0.0, upper=AERATOR_FLIP_LIMIT
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
    spray_head = object_model.get_part("spray_head")
    aerator = object_model.get_part("outlet_aerator")
    lever_0 = object_model.get_part("pin_lever_0")
    lever_1 = object_model.get_part("pin_lever_1")

    swivel = object_model.get_articulation("spout_swivel")
    pull_down = object_model.get_articulation("spray_pull_down")
    aerator_flip = object_model.get_articulation("aerator_flip")
    pivot_0 = object_model.get_articulation("lever_pivot_0")
    pivot_1 = object_model.get_articulation("lever_pivot_1")

    # --- Intentional overlaps ---
    # Lever bosses seat into valve cylinder
    ctx.allow_overlap(
        lever_0, column,
        elem_a="lever_boss", elem_b="cross_tube",
        reason="Lever boss intentionally seats into the valve cylinder.",
    )
    ctx.allow_overlap(
        lever_1, column,
        elem_a="lever_boss", elem_b="cross_tube",
        reason="Lever boss intentionally seats into the valve cylinder.",
    )
    # Spray head nest portion inside tip sleeve and gooseneck tube drop leg
    ctx.allow_overlap(
        spray_head, spout,
        elem_a="spray_body", elem_b="tip_sleeve",
        reason="Spray head nest portion intentionally docks inside the tip sleeve.",
    )
    ctx.allow_overlap(
        spray_head, spout,
        elem_a="spray_body", elem_b="gooseneck_tube",
        reason="Spray head nest portion sits inside the gooseneck tube drop leg when docked.",
    )
    # Aerator disc seated against spray head face when closed
    ctx.allow_overlap(
        aerator, spray_head,
        elem_a="aerator_disc", elem_b="spray_body",
        reason="Aerator disc seats flush against the spray head outlet face when closed.",
    )

    # ===== Grounding, scale, proportions =====
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "faucet grounded on deck plane",
        col_aabb is not None and abs(col_aabb[0][2]) <= 0.002,
        details=f"column aabb={col_aabb}",
    )
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "gooseneck apex near 0.38 m (high-arc silhouette)",
        spout_aabb is not None and 0.372 <= spout_aabb[1][2] <= 0.388,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "gooseneck arcs forward over the sink (+X reach)",
        spout_aabb is not None and spout_aabb[1][0] >= 0.140,
        details=f"spout aabb={spout_aabb}",
    )

    # ===== Mounting collars =====
    mc0 = ctx.part_element_world_aabb(column, elem="mount_collar_0")
    mc1 = ctx.part_element_world_aabb(column, elem="mount_collar_1")
    ctx.check(
        "two mounting collars exist on pedestal",
        mc0 is not None and mc1 is not None,
        details=f"mc0={mc0}, mc1={mc1}",
    )
    shaft = ctx.part_element_world_aabb(column, elem="column_shaft")
    base = ctx.part_element_world_aabb(column, elem="base_disc")
    cross = ctx.part_element_world_aabb(column, elem="cross_tube")
    ctx.check(
        "mounting collars are between base disc and cross valve",
        mc0 is not None and mc1 is not None
        and base is not None and cross is not None
        and mc0[0][2] >= base[1][2] - 0.001
        and mc1[1][2] <= cross[0][2] + 0.001
        and mc0[1][2] < mc1[0][2],
        details=f"mc0={mc0}, mc1={mc1}, base_top={base[1][2] if base else None}, cross_bot={cross[0][2] if cross else None}",
    )
    ctx.check(
        "mounting collars are wider than column shaft",
        mc0 is not None and shaft is not None
        and (mc0[1][0] - mc0[0][0]) > (shaft[1][0] - shaft[0][0]) - 0.001,
        details=f"mc0_dx={mc0[1][0] - mc0[0][0] if mc0 else None}, shaft_dx={shaft[1][0] - shaft[0][0] if shaft else None}",
    )

    # ===== Tick marks =====
    hot = ctx.part_element_world_aabb(column, elem="hot_tick")
    cold = ctx.part_element_world_aabb(column, elem="cold_tick")
    ctx.check(
        "hot and cold tick marks exist as geometry",
        hot is not None and cold is not None,
        details=f"hot={hot}, cold={cold}",
    )
    ctx.check(
        "tick marks are on opposite sides of the column (Y axis)",
        hot is not None and cold is not None
        and 0.5 * (hot[0][1] + hot[1][1]) > 0.04
        and 0.5 * (cold[0][1] + cold[1][1]) < -0.04,
        details=f"hot_y={0.5*(hot[0][1]+hot[1][1]) if hot else None}, cold_y={0.5*(cold[0][1]+cold[1][1]) if cold else None}",
    )
    ctx.check(
        "tick marks are near the cross valve height",
        hot is not None and cold is not None and cross is not None
        and abs(0.5 * (hot[0][2] + hot[1][2]) - CROSS_Z) < 0.015
        and abs(0.5 * (cold[0][2] + cold[1][2]) - CROSS_Z) < 0.015,
        details=f"hot_z={hot}, cold_z={cold}",
    )

    # ===== Spray head geometry =====
    spray_aabb = ctx.part_world_aabb(spray_head)
    sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    ctx.check(
        "spray head docks at the gooseneck mouth (near sleeve bottom)",
        spray_aabb is not None and sleeve is not None
        and abs(spray_aabb[1][2] - sleeve[0][2]) < 0.015,
        details=f"spray={spray_aabb}, sleeve={sleeve}",
    )
    ctx.expect_overlap(
        spray_head, spout,
        axes="z",
        elem_a="spray_body", elem_b="tip_sleeve",
        min_overlap=0.005,
        name="spray head nest portion inserted into tip sleeve",
    )

    # ===== Spray head pull-down joint =====
    ctx.check(
        "spray_pull_down is prismatic along -Z with 0..80mm travel",
        pull_down.articulation_type == ArticulationType.PRISMATIC
        and tuple(pull_down.axis) == (0.0, 0.0, -1.0)
        and pull_down.motion_limits is not None
        and abs(pull_down.motion_limits.lower) < 1e-6
        and abs(pull_down.motion_limits.upper - PULL_DOWN_MAX) < 1e-6,
    )
    # Pull-down pose: spray head moves downward
    rest_spray = ctx.part_world_aabb(spray_head)
    rest_spray_pos = ctx.part_world_position(spray_head)
    with ctx.pose({pull_down: PULL_DOWN_MAX}):
        extended_spray = ctx.part_world_aabb(spray_head)
        extended_spray_pos = ctx.part_world_position(spray_head)
    ctx.check(
        "spray head pulls downward when prismatic q increases",
        rest_spray_pos is not None and extended_spray_pos is not None
        and extended_spray_pos[2] < rest_spray_pos[2] - 0.05,
        details=f"rest={rest_spray_pos}, extended={extended_spray_pos}",
    )

    # ===== Aerator geometry and flip joint =====
    aerator_aabb = ctx.part_world_aabb(aerator)
    ctx.check(
        "aerator exists at the spray head nozzle",
        aerator_aabb is not None and spray_aabb is not None
        and aerator_aabb[0][2] <= spray_aabb[0][2] + 0.005,
        details=f"aerator={aerator_aabb}, spray={spray_aabb}",
    )
    ctx.check(
        "aerator_flip is revolute about +Y with 0..45 deg range",
        aerator_flip.articulation_type == ArticulationType.REVOLUTE
        and tuple(aerator_flip.axis) == (0.0, 1.0, 0.0)
        and aerator_flip.motion_limits is not None
        and abs(aerator_flip.motion_limits.lower) < 1e-6
        and abs(aerator_flip.motion_limits.upper - AERATOR_FLIP_LIMIT) < 1e-6,
    )
    # Flip pose: aerator front edge drops when q increases
    rest_aer = ctx.part_element_world_aabb(aerator, elem="aerator_disc")
    with ctx.pose({aerator_flip: AERATOR_FLIP_LIMIT}):
        flipped_aer = ctx.part_element_world_aabb(aerator, elem="aerator_disc")
    ctx.check(
        "aerator front edge drops when flipped open",
        rest_aer is not None and flipped_aer is not None
        and flipped_aer[0][2] < rest_aer[0][2] - 0.002,
        details=f"rest={rest_aer}, flipped={flipped_aer}",
    )

    # ===== Spout swivel joint =====
    ctx.check(
        "spout swivel is revolute ±110 deg about vertical axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and tuple(swivel.axis) == (0.0, 0.0, 1.0)
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + SWIVEL_LIMIT) < 1e-6
        and abs(swivel.motion_limits.upper - SWIVEL_LIMIT) < 1e-6,
    )
    rest_sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    with ctx.pose({swivel: 1.0}):
        sw_sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    ctx.check(
        "spout swivel carries outlet sideways about vertical axis",
        rest_sleeve is not None and sw_sleeve is not None
        and abs(0.5 * (rest_sleeve[0][1] + rest_sleeve[1][1])) < 1e-6
        and 0.5 * (sw_sleeve[0][1] + sw_sleeve[1][1]) > 0.08,
        details=f"rest={rest_sleeve}, swiveled={sw_sleeve}",
    )

    # ===== Lever joints =====
    for pivot, name in ((pivot_0, "lever_pivot_0"), (pivot_1, "lever_pivot_1")):
        ctx.check(
            f"{name} is revolute -90..0 deg about valve Y axis",
            pivot.articulation_type == ArticulationType.REVOLUTE
            and tuple(pivot.axis) == (0.0, -1.0, 0.0)
            and pivot.motion_limits is not None
            and abs(pivot.motion_limits.lower + math.pi / 2.0) < 1e-6
            and abs(pivot.motion_limits.upper) < 1e-6,
        )

    # Lever boss seating
    for lever, name in ((lever_0, "pin_lever_0"), (lever_1, "pin_lever_1")):
        ctx.expect_overlap(
            lever, column,
            axes="z",
            elem_a="lever_boss", elem_b="cross_tube",
            min_overlap=0.003,
            name=f"{name} boss seats into valve cylinder",
        )

    # Lever tilt check
    rest_0 = ctx.part_world_aabb(lever_0)
    with ctx.pose({pivot_0: -math.pi / 2.0}):
        tilted_0 = ctx.part_world_aabb(lever_0)
    ctx.check(
        "lever 0 tilts toward user (+X) at full negative angle",
        rest_0 is not None and tilted_0 is not None
        and tilted_0[1][0] > rest_0[1][0] + 0.08,
        details=f"rest={rest_0}, tilted={tilted_0}",
    )

    return ctx.report()


object_model = build_object_model()
