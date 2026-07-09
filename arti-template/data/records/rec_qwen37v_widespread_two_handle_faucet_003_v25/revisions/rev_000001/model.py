from __future__ import annotations

"""Matte-black widespread two-handle wall-mounted bathroom faucet.

Three-piece widespread layout on a wall-mount bracket (total spread 0.30 m):
- center: gooseneck spout on a wall-mounted column, swiveling about the column
  vertical axis (revolute, -45..+45 deg),
- hot (left) and cold (right): valve columns with T-style lever handles that
  tilt forward-back (revolute about the horizontal X axis, -90..+90 deg).

Visible stem collars sit under each handle as trim rings. Separate hot (red)
and cold (blue) cap disks are modeled as geometry indicators on the stem tops.
All faucet surfaces matte black. Modeled at true scale in meters; wall face
at y = 0, faucet projects along +Y, Z up.
"""

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- wall-mount bracket --------------------------------------------------
BRACKET_W = 0.38        # width along X (spread axis)
BRACKET_SHELF_D = 0.09  # shelf depth projecting from wall along +Y
BRACKET_SHELF_T = 0.015 # shelf thickness along Z
BRACKET_BACK_H = 0.26   # backplate height along Z
BRACKET_BACK_T = 0.012  # backplate thickness into wall along -Y
SHELF_TOP_Z = 0.20      # top of shelf = faucet mounting surface

SPREAD_HALF = 0.15      # valve columns at x = ±0.15

# ---- center spout piece --------------------------------------------------
SPOUT_FLANGE_R = 0.042
SPOUT_FLANGE_H = 0.012
SPOUT_COL_R = 0.025
SPOUT_COL_H = 0.12

# ---- gooseneck (spout part frame, origin at column top) -------------------
TUBE_R = 0.0155
RISER_EMBED = 0.03
RISER_TOP = 0.14
ARC_R = 0.062
HOOK_DEG = -12.0
COLLAR_R = 0.020
COLLAR_H = 0.016

# ---- valve pieces ---------------------------------------------------------
VALVE_FLANGE_R = 0.036
VALVE_FLANGE_H = 0.010
VALVE_COL_R = 0.0225
VALVE_COL_H = 0.10

# ---- T-lever (lever part frame, origin at valve column top) ---------------
STEM_R = 0.009
STEM_EMBED = 0.015
STEM_TOP = 0.045
BAR_R = 0.0095
BAR_LEN = 0.12
BAR_FWD_OFF = 0.025     # bar center offset along +Y so it overhangs toward user

# ---- stem collar (variant feature) ----------------------------------------
STEM_COLLAR_R = 0.028   # wider than column for a visible trim ring
STEM_COLLAR_H = 0.008

# ---- cap disks (variant feature) ------------------------------------------
CAP_R = 0.012
CAP_H = 0.003

# ---- derived gooseneck geometry -------------------------------------------
ARC_END_Y = ARC_R + ARC_R * math.cos(math.radians(HOOK_DEG))
ARC_END_Z = RISER_TOP + ARC_R * math.sin(math.radians(HOOK_DEG))
AERATOR_LEN = 0.016
AERATOR_R = 0.017
_TX = math.sin(math.radians(HOOK_DEG))
_TZ = -math.cos(math.radians(HOOK_DEG))
AERATOR_CY = ARC_END_Y + _TX * (AERATOR_LEN / 2 - 0.004)
AERATOR_CZ = ARC_END_Z + _TZ * (AERATOR_LEN / 2 - 0.004)


def _gooseneck_solid() -> cq.Workplane:
    """Swept gooseneck tube: straight riser + forward-down arc."""
    path = (
        cq.Workplane("YZ")
        .moveTo(0.0, -RISER_EMBED)
        .lineTo(0.0, RISER_TOP)
        .threePointArc((ARC_R, RISER_TOP + ARC_R), (ARC_END_Y, ARC_END_Z))
    )
    profile = cq.Workplane("XY").workplane(offset=-RISER_EMBED).circle(TUBE_R)
    return profile.sweep(path, isFrenet=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_wall_mount_faucet")

    matte_black = model.material("matte_black", rgba=(0.07, 0.07, 0.07, 1.0))
    hot_red = model.material("hot_red", rgba=(0.78, 0.08, 0.08, 1.0))
    cold_blue = model.material("cold_blue", rgba=(0.10, 0.25, 0.82, 1.0))

    # ============================================================= wall mount
    wall_mount = model.part("wall_mount")
    # Vertical backplate against the wall (face at y=0, body behind wall)
    wall_mount.visual(
        Box((BRACKET_W, BRACKET_BACK_T, BRACKET_BACK_H)),
        origin=Origin(xyz=(0.0, -BRACKET_BACK_T / 2, BRACKET_BACK_H / 2)),
        material=matte_black,
        name="backplate",
    )
    # Horizontal shelf projecting from wall; overlaps backplate 3 mm for
    # within-part connectivity.
    shelf_d_total = BRACKET_SHELF_D + 0.003
    wall_mount.visual(
        Box((BRACKET_W, shelf_d_total, BRACKET_SHELF_T)),
        origin=Origin(xyz=(
            0.0,
            (BRACKET_SHELF_D - 0.003) / 2,
            SHELF_TOP_Z - BRACKET_SHELF_T / 2,
        )),
        material=matte_black,
        name="shelf",
    )

    # ======================================================= center spout base
    spout_base = model.part("spout_base")
    spout_base.visual(
        Cylinder(radius=SPOUT_FLANGE_R, length=SPOUT_FLANGE_H),
        origin=Origin(xyz=(0.0, 0.0, SPOUT_FLANGE_H / 2)),
        material=matte_black,
        name="base_flange",
    )
    spout_base.visual(
        Cylinder(radius=SPOUT_COL_R, length=SPOUT_COL_H),
        origin=Origin(xyz=(0.0, 0.0, SPOUT_COL_H / 2)),
        material=matte_black,
        name="base_column",
    )

    model.articulation(
        "wall_to_spout_base",
        ArticulationType.FIXED,
        parent=wall_mount,
        child=spout_base,
        origin=Origin(xyz=(0.0, BRACKET_SHELF_D / 2, SHELF_TOP_Z)),
    )

    # ========================================================== gooseneck spout
    gooseneck_spout = model.part("gooseneck_spout")
    gooseneck_spout.visual(
        mesh_from_cadquery(_gooseneck_solid(), "gooseneck_tube"),
        material=matte_black,
        name="spout_tube",
    )
    gooseneck_spout.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_H),
        origin=Origin(xyz=(0.0, 0.0, COLLAR_H / 2)),
        material=matte_black,
        name="swivel_collar",
    )
    gooseneck_spout.visual(
        Cylinder(radius=AERATOR_R, length=AERATOR_LEN),
        origin=Origin(
            xyz=(0.0, AERATOR_CY, AERATOR_CZ),
            rpy=(math.radians(HOOK_DEG), 0.0, 0.0),
        ),
        material=matte_black,
        name="aerator",
    )

    model.articulation(
        "spout_swivel",
        ArticulationType.REVOLUTE,
        parent=spout_base,
        child=gooseneck_spout,
        origin=Origin(xyz=(0.0, 0.0, SPOUT_COL_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0, velocity=2.0,
            lower=-math.pi / 4, upper=math.pi / 4,
        ),
    )

    # =================================================== hot / cold valve cols
    def _valve_column(name: str):
        col = model.part(name)
        col.visual(
            Cylinder(radius=VALVE_FLANGE_R, length=VALVE_FLANGE_H),
            origin=Origin(xyz=(0.0, 0.0, VALVE_FLANGE_H / 2)),
            material=matte_black,
            name="valve_flange",
        )
        col.visual(
            Cylinder(radius=VALVE_COL_R, length=VALVE_COL_H),
            origin=Origin(xyz=(0.0, 0.0, VALVE_COL_H / 2)),
            material=matte_black,
            name="valve_body",
        )
        return col

    # ========================================= lever handles (variant geometry)
    def _lever_handle(name: str, cap_material):
        lever = model.part(name)
        # Vertical stem (embeds into column bore below joint)
        lever.visual(
            Cylinder(radius=STEM_R, length=STEM_TOP + STEM_EMBED),
            origin=Origin(xyz=(0.0, 0.0, (STEM_TOP - STEM_EMBED) / 2)),
            material=matte_black,
            name="lever_stem",
        )
        # Stem collar: visible trim ring at the base of the handle
        lever.visual(
            Cylinder(radius=STEM_COLLAR_R, length=STEM_COLLAR_H),
            origin=Origin(xyz=(0.0, 0.0, STEM_COLLAR_H / 2)),
            material=matte_black,
            name="stem_collar",
        )
        # Horizontal T-bar along Y (forward direction), off-center toward user
        lever.visual(
            Cylinder(radius=BAR_R, length=BAR_LEN),
            origin=Origin(
                xyz=(0.0, BAR_FWD_OFF, STEM_TOP),
                rpy=(math.pi / 2, 0.0, 0.0),
            ),
            material=matte_black,
            name="lever_bar",
        )
        # Bar end caps (spheres)
        for end in (-1.0, 1.0):
            lever.visual(
                Sphere(radius=BAR_R),
                origin=Origin(xyz=(
                    0.0,
                    BAR_FWD_OFF + end * BAR_LEN / 2,
                    STEM_TOP,
                )),
                material=matte_black,
                name=f"bar_cap_{'front' if end > 0 else 'rear'}",
            )
        # Cap disk: hot/cold indicator on stem top (geometry, not text)
        lever.visual(
            Cylinder(radius=CAP_R, length=CAP_H),
            origin=Origin(xyz=(0.0, 0.0, STEM_TOP + 0.001 + CAP_H / 2)),
            material=cap_material,
            name="cap_disk",
        )
        return lever

    hot_valve_column = _valve_column("hot_valve_column")
    cold_valve_column = _valve_column("cold_valve_column")
    hot_lever = _lever_handle("hot_lever", hot_red)
    cold_lever = _lever_handle("cold_lever", cold_blue)

    # Fixed mounts on wall bracket shelf
    model.articulation(
        "wall_to_hot_valve",
        ArticulationType.FIXED,
        parent=wall_mount,
        child=hot_valve_column,
        origin=Origin(xyz=(-SPREAD_HALF, BRACKET_SHELF_D / 2, SHELF_TOP_Z)),
    )
    model.articulation(
        "wall_to_cold_valve",
        ArticulationType.FIXED,
        parent=wall_mount,
        child=cold_valve_column,
        origin=Origin(xyz=(SPREAD_HALF, BRACKET_SHELF_D / 2, SHELF_TOP_Z)),
    )

    # Lever articulations: forward-back tilt about horizontal X axis
    for joint_name, parent_col, child_lever in (
        ("hot_lever_tilt", hot_valve_column, hot_lever),
        ("cold_lever_tilt", cold_valve_column, cold_lever),
    ):
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=parent_col,
            child=child_lever,
            origin=Origin(xyz=(0.0, 0.0, VALVE_COL_H)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0, velocity=2.0,
                lower=-math.pi / 2, upper=math.pi / 2,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall_mount = object_model.get_part("wall_mount")
    spout_base = object_model.get_part("spout_base")
    gooseneck = object_model.get_part("gooseneck_spout")
    hot_col = object_model.get_part("hot_valve_column")
    cold_col = object_model.get_part("cold_valve_column")
    hot_lever = object_model.get_part("hot_lever")
    cold_lever = object_model.get_part("cold_lever")

    spout_swivel = object_model.get_articulation("spout_swivel")
    hot_tilt = object_model.get_articulation("hot_lever_tilt")
    cold_tilt = object_model.get_articulation("cold_lever_tilt")

    spout_tube = gooseneck.get_visual("spout_tube")
    base_column = spout_base.get_visual("base_column")
    aerator = gooseneck.get_visual("aerator")

    # --- intentional hidden engagements ------------------------------------
    ctx.allow_overlap(
        gooseneck, spout_base,
        elem_a=spout_tube, elem_b=base_column,
        reason="gooseneck riser tube seats 30 mm into the base column bore",
    )
    for lever, col in ((hot_lever, hot_col), (cold_lever, cold_col)):
        ctx.allow_overlap(
            lever, col,
            elem_a=lever.get_visual("lever_stem"),
            elem_b=col.get_visual("valve_body"),
            reason="lever stem seats 15 mm into the valve cartridge bore",
        )
        ctx.allow_overlap(
            lever, col,
            elem_a=lever.get_visual("stem_collar"),
            elem_b=col.get_visual("valve_body"),
            reason="stem collar trim ring seats against the valve column top face",
        )

    # === variant check: wall mount is root, no sink deck ====================
    ctx.check(
        "wall_mount_is_root",
        wall_mount is not None,
        "wall_mount part must exist",
    )
    has_deck = False
    try:
        object_model.get_part("sink_deck")
        has_deck = True
    except Exception:
        pass
    ctx.check("no_sink_deck", not has_deck,
              "sink_deck should not exist in wall-mounted variant")

    # === variant check: stem collars present under each handle ==============
    for lever in (hot_lever, cold_lever):
        collar = lever.get_visual("stem_collar")
        ctx.check(
            f"{lever.name}_has_stem_collar",
            collar is not None,
            f"stem_collar visual missing on {lever.name}",
        )

    # === variant check: hot/cold cap disks as geometry, not text ============
    hot_cap = hot_lever.get_visual("cap_disk")
    cold_cap = cold_lever.get_visual("cap_disk")
    hot_mat = hot_cap.material if isinstance(hot_cap.material, str) else hot_cap.material.name
    cold_mat = cold_cap.material if isinstance(cold_cap.material, str) else cold_cap.material.name
    ctx.check("hot_cap_is_red", hot_mat == "hot_red", f"material={hot_mat}")
    ctx.check("cold_cap_is_blue", cold_mat == "cold_blue", f"material={cold_mat}")

    # === joint plan: handle tilts are horizontal revolute (forward-back) ====
    for joint in (hot_tilt, cold_tilt):
        ctx.check(
            f"{joint.name}_is_x_revolute",
            str(joint.joint_type).lower().endswith("revolute")
            and tuple(joint.axis) == (1.0, 0.0, 0.0),
            f"axis={joint.axis}",
        )
        ml = joint.motion_limits
        ctx.check(
            f"{joint.name}_range_pm90",
            ml is not None
            and abs(ml.lower + math.pi / 2) < 1e-6
            and abs(ml.upper - math.pi / 2) < 1e-6,
            f"lower={ml.lower} upper={ml.upper}",
        )

    # Spout swivel remains vertical revolute
    ctx.check(
        "spout_swivel_is_z_revolute",
        str(spout_swivel.joint_type).lower().endswith("revolute")
        and tuple(spout_swivel.axis) == (0.0, 0.0, 1.0),
        f"axis={spout_swivel.axis}",
    )
    ml_s = spout_swivel.motion_limits
    ctx.check(
        "spout_swivel_range_pm45",
        ml_s is not None
        and abs(ml_s.lower + math.pi / 4) < 1e-6
        and abs(ml_s.upper - math.pi / 4) < 1e-6,
        f"lower={ml_s.lower} upper={ml_s.upper}",
    )

    # === widespread 0.30 m spread on the wall bracket =======================
    hot_pos = ctx.part_world_position(hot_col)
    cold_pos = ctx.part_world_position(cold_col)
    spout_pos = ctx.part_world_position(spout_base)
    ctx.check(
        "widespread_0p30_spread",
        abs(hot_pos[0] + 0.15) < 1e-3
        and abs(cold_pos[0] - 0.15) < 1e-3
        and abs(spout_pos[0]) < 1e-3,
        f"hot_x={hot_pos[0]} cold_x={cold_pos[0]} spout_x={spout_pos[0]}",
    )

    # === articulation behavior: lever tilts forward-back ====================
    # At q=0 the bar spans Y (forward direction); at q=+π/2 the bar has
    # rotated to span Z, proving forward-back tilt about X axis.
    with ctx.pose({hot_tilt: 0.0}):
        bar0 = ctx.part_element_world_aabb(hot_lever, elem=hot_lever.get_visual("lever_bar"))
    with ctx.pose({hot_tilt: math.pi / 2}):
        bar90 = ctx.part_element_world_aabb(hot_lever, elem=hot_lever.get_visual("lever_bar"))
    span_y0 = bar0[1][1] - bar0[0][1]
    span_z0 = bar0[1][2] - bar0[0][2]
    span_y90 = bar90[1][1] - bar90[0][1]
    span_z90 = bar90[1][2] - bar90[0][2]
    ctx.check(
        "hot_lever_tilts_forward_back",
        span_y0 > 0.08 and span_z90 > 0.08 and span_z0 < 0.04 and span_y90 < 0.04,
        f"q=0 y_span={span_y0:.3f} z_span={span_z0:.3f} "
        f"q=π/2 y_span={span_y90:.3f} z_span={span_z90:.3f}",
    )

    # === spout swivel proof =================================================
    with ctx.pose({spout_swivel: 0.0}):
        tip0 = ctx.part_element_world_aabb(gooseneck, elem=aerator)
    with ctx.pose({spout_swivel: math.pi / 4}):
        tip45 = ctx.part_element_world_aabb(gooseneck, elem=aerator)
    tip0_cx = 0.5 * (tip0[0][0] + tip0[1][0])
    tip45_cx = 0.5 * (tip45[0][0] + tip45[1][0])
    tip45_cz = 0.5 * (tip45[0][2] + tip45[1][2])
    tip0_cz = 0.5 * (tip0[0][2] + tip0[1][2])
    ctx.check(
        "spout_swivels_laterally",
        tip45_cx < tip0_cx - 0.03 and abs(tip45_cz - tip0_cz) < 1e-3,
        f"rest_x={tip0_cx:.3f} 45deg_x={tip45_cx:.3f} "
        f"rest_z={tip0_cz:.3f} 45deg_z={tip45_cz:.3f}",
    )

    # === gooseneck arc height ==============================================
    neck_aabb = ctx.part_world_aabb(gooseneck)
    arc_top_z = neck_aabb[1][2]
    shelf_z = SHELF_TOP_Z
    ctx.check(
        "gooseneck_rises_above_shelf",
        0.25 < (arc_top_z - shelf_z) < 0.40,
        f"arc top {arc_top_z:.3f} m, shelf {shelf_z:.3f} m",
    )

    return ctx.report()


object_model = build_object_model()
