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
# Variant 20: High-arc gooseneck faucet sibling.
#
# Changes from parent (gloss-black monobloc mixer tap):
# 1. Rear support strut from base disc to collar area (behind column)
# 2. Flip-down outlet aerator pivoting at the nozzle tip
# 3. Shallow ribbing rings on the spray head (tip sleeve area)
# 4. Thin seam ring at the swivel collar
#
# Preserved: tall gooseneck silhouette, chrome base disc, valve cross-cylinder,
# pin levers, spout swivel, gloss-black veined finish.
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
SWIVEL_Z = 0.140

TUBE_R = 0.015
ARC_R = 0.072
RISER_TOP = 0.153
REACH_X = 2.0 * ARC_R
DROP_END = 0.124

SLEEVE_R = 0.0165
SLEEVE_LEN = 0.028

APEX_WORLD = SWIVEL_Z + RISER_TOP + ARC_R + TUBE_R

SWIVEL_LIMIT = math.radians(110.0)

# ---- Variant 20 additions ----

# Rear support strut (behind column, from base to collar)
STRUT_R = 0.005
STRUT_X = -0.026
STRUT_Z0 = BASE_DISC_H
STRUT_Z1 = SWIVEL_Z - COLLAR_LEN
STRUT_LEN = STRUT_Z1 - STRUT_Z0

# Collar seam (thin dark ring at collar top)
SEAM_R = COLLAR_R + 0.001
SEAM_H = 0.0015
SEAM_Z = SWIVEL_Z  # at top of collar

# Spray head ribbing (3 raised rings on tip sleeve)
RIB_R = SLEEVE_R + 0.001
RIB_H = 0.0015
N_RIBS = 3
RIB_Z_START = DROP_END + 0.004
RIB_Z_STEP = 0.008

# Flip-down aerator
AERATOR_BODY_R = 0.012
AERATOR_BODY_LEN = 0.010
AERATOR_FLANGE_R = 0.014
AERATOR_FLANGE_LEN = 0.002
AERATOR_FLIP_LIMIT = math.radians(40.0)


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
    model = ArticulatedObject(name="high_arc_gooseneck_faucet")

    gloss_black = model.material("veined_gloss_black", rgba=(0.075, 0.075, 0.085, 1.0))
    matte_black = model.material("matte_black", rgba=(0.035, 0.035, 0.038, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    outlet_dark = model.material("outlet_dark", rgba=(0.10, 0.10, 0.10, 1.0))
    seam_dark = model.material("seam_dark", rgba=(0.02, 0.02, 0.025, 1.0))
    rib_chrome = model.material("rib_chrome", rgba=(0.75, 0.77, 0.80, 1.0))

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
    # Horizontal cross valve cylinder
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
    # --- Variant 20: rear support strut ---
    column.visual(
        Cylinder(radius=STRUT_R, length=STRUT_LEN),
        origin=Origin(xyz=(STRUT_X, 0.0, STRUT_Z0 + STRUT_LEN / 2.0)),
        material=gloss_black,
        name="rear_strut",
    )
    # --- Variant 20: collar seam ring ---
    column.visual(
        Cylinder(radius=SEAM_R, length=SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, SEAM_Z + SEAM_H / 2.0)),
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
    # --- Variant 20: spray head ribbing (3 raised rings on sleeve) ---
    for i in range(N_RIBS):
        rib_z = RIB_Z_START + i * RIB_Z_STEP
        spout.visual(
            Cylinder(radius=RIB_R, length=RIB_H),
            origin=Origin(xyz=(REACH_X, 0.0, rib_z)),
            material=rib_chrome,
            name=f"spray_rib_{i}",
        )

    # ------------------------------------------------------- flip-down aerator
    aerator = model.part("aerator_cap")
    # Main aerator body - cylinder hanging below the nozzle
    aerator.visual(
        Cylinder(radius=AERATOR_BODY_R, length=AERATOR_BODY_LEN),
        origin=Origin(xyz=(0.0, 0.0, -AERATOR_BODY_LEN / 2.0)),
        material=outlet_dark,
        name="aerator_body",
    )
    # Flange at top for hinge attachment
    aerator.visual(
        Cylinder(radius=AERATOR_FLANGE_R, length=AERATOR_FLANGE_LEN),
        origin=Origin(xyz=(0.0, 0.0, AERATOR_FLANGE_LEN / 2.0)),
        material=chrome,
        name="aerator_flange",
    )

    # Flip-down joint: hinge at front (+X) edge of nozzle bottom
    # In spout-local frame, joint at (REACH_X + AERATOR_FLANGE_R, 0, DROP_END)
    # Aerator local frame origin is at hinge; body extends downward (-Z) and
    # inward (-X) so at rest it's centered under the nozzle.
    # Wait - the aerator part origin is at the joint. In aerator local frame,
    # the body hangs at (0, 0, -body_len/2). At rest this is directly below hinge.
    # The hinge is at the front edge, so the aerator hangs slightly forward.
    # Let me offset: put hinge at center of nozzle, body below.
    model.articulation(
        "aerator_flip",
        ArticulationType.REVOLUTE,
        parent=spout,
        child=aerator,
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END)),
        # Axis along Y: positive q tilts bottom toward +X (user/sink side)
        # with axis=(0, -1, 0), rotation by positive angle takes -Z toward +X
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=0.0, upper=AERATOR_FLIP_LIMIT
        ),
    )

    # ---------------------------------------------------------- spout swivel
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
    aerator = object_model.get_part("aerator_cap")
    lever_0 = object_model.get_part("pin_lever_0")
    lever_1 = object_model.get_part("pin_lever_1")

    swivel = object_model.get_articulation("spout_swivel")
    pivot_0 = object_model.get_articulation("lever_pivot_0")
    pivot_1 = object_model.get_articulation("lever_pivot_1")
    flip = object_model.get_articulation("aerator_flip")

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
    # Aerator flange overlaps spout sleeve at the hinge - intentional seated mount
    ctx.allow_overlap(
        aerator,
        spout,
        elem_a="aerator_flange",
        elem_b="tip_sleeve",
        reason="Aerator flange seats against the tip sleeve at the hinge mount.",
    )
    # Aerator body nests just inside the gooseneck tube drop-leg opening
    ctx.allow_overlap(
        aerator,
        spout,
        elem_a="aerator_body",
        elem_b="gooseneck_tube",
        reason="Aerator body sits at the open end of the gooseneck tube drop leg, partially nested inside the tube bore.",
    )

    # ----- grounding and scale -----
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "tap grounded on the deck plane",
        col_aabb is not None and abs(col_aabb[0][2]) <= 0.002,
        details=f"column aabb={col_aabb}",
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

    # ----- Variant 20: rear support strut -----
    strut = ctx.part_element_world_aabb(column, elem="rear_strut")
    ctx.check(
        "rear support strut rises from base to collar area behind column",
        strut is not None
        and strut[0][2] <= BASE_DISC_H + 0.002
        and strut[1][2] >= SWIVEL_Z - COLLAR_LEN - 0.005
        and strut[1][0] < -0.020,
        details=f"strut aabb={strut}",
    )

    # ----- Variant 20: collar seam -----
    seam = ctx.part_element_world_aabb(column, elem="collar_seam")
    collar = ctx.part_element_world_aabb(column, elem="swivel_collar")
    ctx.check(
        "thin seam ring visible at the swivel collar",
        seam is not None
        and collar is not None
        and abs(seam[0][2] - collar[1][2]) <= 0.003
        and (seam[1][2] - seam[0][2]) <= 0.003,
        details=f"seam={seam}, collar={collar}",
    )

    # ----- Variant 20: spray head ribbing -----
    ribs_found = 0
    for i in range(N_RIBS):
        rib = ctx.part_element_world_aabb(spout, elem=f"spray_rib_{i}")
        if rib is not None and (rib[1][2] - rib[0][2]) <= 0.003:
            ribs_found += 1
    ctx.check(
        f"shallow ribbing on spray head ({N_RIBS} rings on tip sleeve)",
        ribs_found == N_RIBS,
        details=f"ribs found={ribs_found}/{N_RIBS}",
    )

    # ----- Variant 20: flip-down aerator joint -----
    ctx.check(
        "aerator_flip is revolute with limits 0..40 deg about horizontal axis",
        flip.articulation_type == ArticulationType.REVOLUTE
        and flip.motion_limits is not None
        and abs(flip.motion_limits.lower) < 1e-6
        and flip.motion_limits.upper > 0.5
        and tuple(flip.axis) == (0.0, -1.0, 0.0),
    )

    # Aerator exists below the nozzle at rest
    aerator_body = ctx.part_element_world_aabb(aerator, elem="aerator_body")
    sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    ctx.check(
        "aerator body hangs below the tip sleeve at rest",
        aerator_body is not None
        and sleeve is not None
        and aerator_body[1][2] < sleeve[0][2] + 0.002,
        details=f"aerator_body={aerator_body}, sleeve={sleeve}",
    )

    # Proof: aerator body is retained at the tube opening, centered in XY
    ctx.expect_overlap(
        aerator,
        spout,
        axes="xy",
        elem_a="aerator_body",
        elem_b="tip_sleeve",
        min_overlap=0.010,
        name="aerator body is centered within the tip sleeve bore",
    )
    ctx.expect_gap(
        spout,
        aerator,
        axis="z",
        positive_elem="tip_sleeve",
        negative_elem="aerator_body",
        max_gap=0.003,
        max_penetration=0.002,
        name="aerator body hangs just below the tip sleeve",
    )

    # Pose check: aerator flips downward (bottom swings toward +X)
    rest_aerator = ctx.part_element_world_aabb(aerator, elem="aerator_body")
    with ctx.pose({flip: AERATOR_FLIP_LIMIT}):
        flipped_aerator = ctx.part_element_world_aabb(aerator, elem="aerator_body")
    ctx.check(
        "aerator flip tilts the body forward (+X) at max angle",
        rest_aerator is not None
        and flipped_aerator is not None
        and flipped_aerator[1][0] > rest_aerator[1][0] + 0.003,
        details=f"rest={rest_aerator}, flipped={flipped_aerator}",
    )

    # ----- spout swivel joint -----
    ctx.check(
        "spout swivel is revolute -110..+110 deg about the vertical column axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and tuple(swivel.axis) == (0.0, 0.0, 1.0)
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + SWIVEL_LIMIT) < 1e-6
        and abs(swivel.motion_limits.upper - SWIVEL_LIMIT) < 1e-6,
    )

    # ----- lever joints -----
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

    # ----- spout swivel pose -----
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

    # ----- gooseneck collar contact -----
    ctx.expect_contact(
        spout,
        column,
        elem_a="gooseneck_tube",
        elem_b="swivel_collar",
        contact_tol=0.001,
        name="gooseneck riser seats on the chrome collar",
    )

    return ctx.report()


object_model = build_object_model()
