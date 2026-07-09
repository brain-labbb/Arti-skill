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
# Single-hole basin faucet with lever handle, ~0.13 m tall, mirror chrome.
# Variant 29 of the self-closing pillar tap family.
# World frame: +Z up, deck at z = 0, spout points toward +X (front).
# The body leans BACK a few degrees, i.e. its long axis tilts toward -X.
# ---------------------------------------------------------------------------

TILT = math.radians(6.0)
SIN_T = math.sin(TILT)
COS_T = math.cos(TILT)

# Base flange (sits flat on the deck).
FLANGE_R = 0.030
FLANGE_H = 0.006

# Main body barrel.
BODY_R = 0.025
BODY_S0 = 0.006
BODY_S1 = 0.0725

# Thin recessed separation groove ring around the upper third.
GROOVE_R = 0.0215
GROOVE_S0 = 0.0705
GROOVE_S1 = 0.0760

# Stepped-in upper neck above the groove.
NECK_R = 0.0228
NECK_S0 = 0.0740
NECK_S1 = 0.104

# Lever pivot station (top of neck).
LEVER_PIVOT_S = NECK_S1

# Lever geometry
LEVER_PIVOT_R = 0.010  # pivot boss radius
LEVER_PIVOT_H = 0.008  # pivot boss height above neck top
LEVER_ARM_W = 0.012    # lever arm width
LEVER_ARM_H = 0.006    # lever arm thickness
LEVER_ARM_L = 0.045    # lever arm length from pivot center

LEVER_TILT_LIMIT = math.radians(40.0)  # max upward tilt

# Spout exit station on the body axis.
SPOUT_S = 0.050

# Spout collar seam
COLLAR_R = 0.018
COLLAR_H = 0.005

# Aerator (sized to seat inside the spout bore wall)
AERATOR_R = 0.0145
AERATOR_H = 0.004


def _axis_point(s: float) -> tuple[float, float, float]:
    """World position of the tilted body axis at axial station s."""
    return (-s * SIN_T, 0.0, s * COS_T)


def _tilted(s: float) -> Origin:
    """Origin on the body axis at station s, z-axis aligned with the axis."""
    return Origin(xyz=_axis_point(s), rpy=(0.0, -TILT, 0.0))


def _build_spout_shape() -> cq.Workplane:
    """Hollow chrome spout: straight shank, smooth downward bend, flared
    open outlet rim with a real hollow bore. Built in spout-local frame whose
    origin sits on the body axis at SPOUT_S; the shank runs along local +X."""
    r_out = 0.015
    shank_x0 = 0.010  # seated ~15 mm inside the body casting
    shank_x1 = 0.035
    bend = 0.028  # bend radius; end heads straight down
    end_x = shank_x1 + bend
    end_z = -bend

    path = (
        cq.Workplane("XZ")
        .moveTo(shank_x0, 0.0)
        .lineTo(shank_x1, 0.0)
        .tangentArcPoint((bend, -bend), relative=True)
    )
    tube = cq.Workplane("YZ", origin=(shank_x0, 0.0, 0.0)).circle(r_out).sweep(path)

    # Flared outlet skirt around the down-turned end.
    flare = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z + 0.006))
        .circle(0.0148)
        .workplane(offset=-0.010)
        .circle(0.0185)
        .loft()
    )
    spout = tube.union(flare)

    # Tapered bore opening the outlet mouth (real hollow outlet rim).
    bore = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z - 0.008))
        .circle(0.0155)
        .workplane(offset=0.020)
        .circle(0.011)
        .loft()
    )
    return spout.cut(bore)


def _build_collar_shape() -> cq.Workplane:
    """Collar seam ring at the spout-body junction. Built in the same local
    frame as the spout."""
    collar = (
        cq.Workplane("YZ", origin=(0.010, 0.0, 0.0))
        .circle(COLLAR_R)
        .extrude(COLLAR_H)
    )
    # Cut a hole through the center so the spout shank passes through
    bore = (
        cq.Workplane("YZ", origin=(0.008, 0.0, 0.0))
        .circle(0.014)
        .extrude(COLLAR_H + 0.004)
    )
    return collar.cut(bore)


def _build_aerator_shape() -> cq.Workplane:
    """Circular aerator insert: a thin ring with a recessed mesh-like disc.
    Built in local frame with z=0 at the disc bottom, centered at origin."""
    # Outer ring
    outer = cq.Workplane("XY").circle(AERATOR_R).extrude(AERATOR_H)
    # Central bore (slightly smaller than outer to leave a ring wall)
    inner_r = AERATOR_R - 0.0015
    bore = cq.Workplane("XY").circle(inner_r).extrude(AERATOR_H)
    ring = outer.cut(bore)
    # Inner mesh disc - slightly recessed, touching the ring inner wall
    mesh_disc = (
        cq.Workplane("XY", origin=(0.0, 0.0, 0.0005))
        .circle(inner_r)
        .extrude(AERATOR_H - 0.001)
    )
    return ring.union(mesh_disc)


def _build_lever_shape() -> cq.Workplane:
    """Lever handle: a flat bar extending from a cylindrical pivot boss.
    Local frame: pivot center at origin, lever extends along +X, z is thickness."""
    # Pivot boss cylinder
    boss = (
        cq.Workplane("XY", origin=(0.0, 0.0, -LEVER_PIVOT_H / 2.0))
        .circle(LEVER_PIVOT_R)
        .extrude(LEVER_PIVOT_H)
    )
    # Lever arm - flat bar extending from pivot center
    arm_start_x = LEVER_PIVOT_R * 0.5
    arm = (
        cq.Workplane("XY")
        .transformed(offset=(arm_start_x + LEVER_ARM_L / 2.0, 0.0, 0.0))
        .box(LEVER_ARM_L, LEVER_ARM_W, LEVER_ARM_H)
    )
    lever = boss.union(arm)
    return lever


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet_lever")

    model.material("chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    model.material("chrome_dark", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("chrome_brushed", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("aerator_mesh", rgba=(0.55, 0.57, 0.60, 1.0))
    model.material("collar_accent", rgba=(0.35, 0.37, 0.40, 1.0))

    # ---------------- body (root): flange + barrel + groove + neck ---------
    body = model.part("body")
    body.visual(
        Cylinder(radius=FLANGE_R, length=FLANGE_H),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_H / 2.0)),
        material="chrome",
        name="base_flange",
    )
    body.visual(
        Cylinder(radius=BODY_R, length=BODY_S1 - BODY_S0),
        origin=_tilted((BODY_S0 + BODY_S1) / 2.0),
        material="chrome",
        name="body_barrel",
    )
    body.visual(
        Cylinder(radius=GROOVE_R, length=GROOVE_S1 - GROOVE_S0),
        origin=_tilted((GROOVE_S0 + GROOVE_S1) / 2.0),
        material="chrome_dark",
        name="groove_ring",
    )
    body.visual(
        Cylinder(radius=NECK_R, length=NECK_S1 - NECK_S0),
        origin=_tilted((NECK_S0 + NECK_S1) / 2.0),
        material="chrome",
        name="body_neck",
    )

    # ---------------- spout (fixed): swept hollow tube + flared outlet -----
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_spout_shape(), "spout", tolerance=0.0003),
        material="chrome",
        name="spout_tube",
    )
    # Collar seam ring at the spout-body junction (detachable look)
    spout.visual(
        mesh_from_cadquery(_build_collar_shape(), "collar", tolerance=0.0003),
        material="collar_accent",
        name="spout_collar",
    )
    model.articulation(
        "spout_mount",
        ArticulationType.FIXED,
        parent=body,
        child=spout,
        origin=Origin(xyz=_axis_point(SPOUT_S)),
    )

    # ---------------- aerator (fixed to spout) ----------------------------
    aerator = model.part("aerator")
    aerator.visual(
        mesh_from_cadquery(_build_aerator_shape(), "aerator", tolerance=0.0003),
        material="aerator_mesh",
        name="aerator_disc",
    )
    # The aerator sits at the spout outlet mouth. The spout end is at
    # (end_x, 0, end_z) in spout-local frame = (0.063, 0, -0.028).
    # In world that's offset from SPOUT_S axis point.
    spout_end_local = (0.063, 0.0, -0.025)
    spout_origin_world = _axis_point(SPOUT_S)
    aerator_world = (
        spout_origin_world[0] + spout_end_local[0],
        spout_origin_world[1] + spout_end_local[1],
        spout_origin_world[2] + spout_end_local[2],
    )
    model.articulation(
        "aerator_mount",
        ArticulationType.FIXED,
        parent=spout,
        child=aerator,
        origin=Origin(xyz=spout_end_local),
    )

    # ---------------- pivot carrier (side-to-side swing on body top) ------
    carrier = model.part("pivot_carrier")
    carrier.visual(
        Cylinder(radius=LEVER_PIVOT_R + 0.002, length=LEVER_PIVOT_H),
        origin=Origin(xyz=(0.0, 0.0, LEVER_PIVOT_H / 2.0)),
        material="chrome",
        name="carrier_boss",
    )
    lever_origin = _axis_point(LEVER_PIVOT_S)
    model.articulation(
        "lever_swing",
        ArticulationType.REVOLUTE,
        parent=body,
        child=carrier,
        origin=Origin(xyz=lever_origin, rpy=(0.0, -TILT, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0,
            lower=-math.radians(45.0), upper=math.radians(45.0),
        ),
    )

    # ---------------- lever (up/down tilt on carrier) ---------------------
    lever = model.part("lever")
    lever.visual(
        mesh_from_cadquery(_build_lever_shape(), "lever", tolerance=0.0003),
        material="chrome",
        name="lever_handle",
    )
    # Lever arm extends along +X from the carrier origin.
    # axis=(0,-1,0): right-hand rule around -Y makes +X rotate toward +Z (up).
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=carrier,
        child=lever,
        origin=Origin(xyz=(0.0, 0.0, LEVER_PIVOT_H)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=0.0, upper=LEVER_TILT_LIMIT
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    aerator = object_model.get_part("aerator")
    carrier = object_model.get_part("pivot_carrier")
    lever = object_model.get_part("lever")
    lever_pivot = object_model.get_articulation("lever_pivot")
    lever_swing = object_model.get_articulation("lever_swing")

    # Intentional seated insertions (solid proxies, scoped per element).
    ctx.allow_overlap(
        spout,
        body,
        elem_a="spout_tube",
        elem_b="body_barrel",
        reason="Spout shank is intentionally seated ~15 mm into the solid body casting.",
    )
    ctx.allow_overlap(
        spout,
        body,
        elem_a="spout_collar",
        elem_b="body_barrel",
        reason="Collar seam ring wraps the spout-body junction surface.",
    )
    ctx.allow_overlap(
        aerator,
        spout,
        elem_a="aerator_disc",
        elem_b="spout_tube",
        reason="Aerator disc is seated inside the spout outlet mouth.",
    )
    ctx.allow_overlap(
        carrier,
        body,
        elem_a="carrier_boss",
        elem_b="body_neck",
        reason="Pivot carrier boss sits on top of the neck, small overlap at mounting interface.",
    )
    ctx.allow_overlap(
        lever,
        carrier,
        elem_a="lever_handle",
        elem_b="carrier_boss",
        reason="Lever pivot boss mounts on top of the carrier boss, small overlap at pivot interface.",
    )

    # ---- hero geometry: flange seated on deck, body leaning back ----------
    flange_aabb = ctx.part_element_world_aabb(body, elem="base_flange")
    ctx.check(
        "base flange sits flat on the deck",
        flange_aabb is not None and abs(flange_aabb[0][2]) <= 0.0005,
        details=f"flange aabb={flange_aabb}",
    )
    neck_aabb = ctx.part_element_world_aabb(body, elem="body_neck")
    ctx.check(
        "body leans back (neck offset toward -X behind the flange center)",
        neck_aabb is not None and (neck_aabb[0][0] + neck_aabb[1][0]) / 2.0 < -0.005,
        details=f"neck aabb={neck_aabb}",
    )

    # ---- spout: projects forward from the body and curves down ------------
    ctx.expect_overlap(
        spout,
        body,
        axes="xz",
        elem_a="spout_tube",
        elem_b="body_barrel",
        min_overlap=0.005,
        name="spout shank stays seated in the body",
    )
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "spout reaches forward and droops to a low open outlet above the deck",
        spout_aabb is not None
        and spout_aabb[1][0] > 0.060
        and spout_aabb[0][2] < 0.025
        and spout_aabb[0][2] > 0.008,
        details=f"spout aabb={spout_aabb}",
    )

    # ---- collar seam: visible ring at the spout-body junction ------------
    collar_aabb = ctx.part_element_world_aabb(spout, elem="spout_collar")
    ctx.check(
        "collar seam exists at the spout-body junction",
        collar_aabb is not None
        and (collar_aabb[1][1] - collar_aabb[0][1]) > 0.020,
        details=f"collar aabb={collar_aabb}",
    )

    # ---- aerator: separate circular insert at the spout mouth ------------
    aerator_aabb = ctx.part_world_aabb(aerator)
    ctx.check(
        "aerator insert exists near the spout outlet",
        aerator_aabb is not None
        and aerator_aabb[0][2] > 0.0
        and aerator_aabb[0][2] < 0.040,
        details=f"aerator aabb={aerator_aabb}",
    )
    ctx.expect_overlap(
        aerator,
        spout,
        axes="xy",
        min_overlap=0.005,
        name="aerator overlaps spout footprint in XY (seated in mouth)",
    )

    # ---- lever: handle on top of body ------------------------------------
    lever_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "lever handle sits above the body neck",
        lever_aabb is not None
        and neck_aabb is not None
        and lever_aabb[0][2] >= neck_aabb[0][2] - 0.005,
        details=f"lever aabb={lever_aabb}, neck aabb={neck_aabb}",
    )

    # ---- overall height --------------------------------------------------
    ctx.check(
        "overall faucet height is about 0.13 m",
        lever_aabb is not None and 0.105 <= lever_aabb[1][2] <= 0.145,
        details=f"lever aabb={lever_aabb}",
    )

    # ---- articulation: lever pivot limits --------------------------------
    ll = lever_pivot.motion_limits
    ctx.check(
        "lever pivot limits allow 0 to ~40 degree upward tilt",
        ll is not None
        and ll.lower is not None
        and ll.upper is not None
        and abs(ll.lower) < 1e-9
        and 0.5 <= ll.upper <= 1.0,
        details=f"limits={ll}",
    )

    # ---- articulation: lever swing limits --------------------------------
    sl = lever_swing.motion_limits
    ctx.check(
        "lever swing limits allow ±45 degree side-to-side rotation",
        sl is not None
        and sl.lower is not None
        and sl.upper is not None
        and abs(sl.lower + math.radians(45.0)) < 1e-6
        and abs(sl.upper - math.radians(45.0)) < 1e-6,
        details=f"limits={sl}",
    )

    # ---- decisive pose: lever tilts upward when actuated -----------------
    lever_rest_aabb = ctx.part_element_world_aabb(lever, elem="lever_handle")
    with ctx.pose({lever_pivot: LEVER_TILT_LIMIT}):
        lever_tilted_aabb = ctx.part_element_world_aabb(lever, elem="lever_handle")
    ctx.check(
        "lever tilts upward when actuated (tip rises)",
        lever_rest_aabb is not None
        and lever_tilted_aabb is not None
        and lever_tilted_aabb[1][2] > lever_rest_aabb[1][2] + 0.003,
        details=f"rest_max_z={lever_rest_aabb[1][2] if lever_rest_aabb else None}, "
                f"tilted_max_z={lever_tilted_aabb[1][2] if lever_tilted_aabb else None}",
    )

    # ---- decisive pose: lever swings side-to-side ------------------------
    lever_rest_aabb = ctx.part_element_world_aabb(lever, elem="lever_handle")
    swing_angle = math.radians(30.0)
    with ctx.pose({lever_swing: swing_angle}):
        lever_swing_aabb = ctx.part_element_world_aabb(lever, elem="lever_handle")
    ctx.check(
        "lever swings side-to-side (Y position changes)",
        lever_rest_aabb is not None
        and lever_swing_aabb is not None
        and abs((lever_swing_aabb[0][1] + lever_swing_aabb[1][1]) / 2.0
                - (lever_rest_aabb[0][1] + lever_rest_aabb[1][1]) / 2.0) > 0.005,
        details=f"rest_y_center={(lever_rest_aabb[0][1] + lever_rest_aabb[1][1]) / 2.0 if lever_rest_aabb else None}, "
                f"swing_y_center={(lever_swing_aabb[0][1] + lever_swing_aabb[1][1]) / 2.0 if lever_swing_aabb else None}",
    )

    # ---- lever joints are revolute (non-fixed) ---------------------------
    ctx.check(
        "lever pivot is a revolute joint (non-fixed articulation)",
        lever_pivot.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={lever_pivot.articulation_type}",
    )
    ctx.check(
        "lever swing is a revolute joint (non-fixed articulation)",
        lever_swing.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={lever_swing.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
