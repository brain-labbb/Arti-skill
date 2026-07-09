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
# Single-hole basin faucet with lever handle, ~0.13 m tall, mirror chrome.
# World frame: +Z up, deck at z = 0, spout points toward +X (front).
# The body leans BACK a few degrees, i.e. its long axis tilts toward -X.
# All "s" values below are axial stations in meters measured along that
# tilted body axis from the deck.
# ---------------------------------------------------------------------------

TILT = math.radians(6.0)
SIN_T = math.sin(TILT)
COS_T = math.cos(TILT)

# Base flange (sits flat on the deck).
FLANGE_R = 0.030
FLANGE_H = 0.006

# Raised circular collar around the base (variant feature).
COLLAR_R = 0.033
COLLAR_H = 0.010
COLLAR_S0 = FLANGE_H
COLLAR_S1 = COLLAR_S0 + COLLAR_H

# Main body barrel.
BODY_R = 0.025
BODY_S0 = COLLAR_S1
BODY_S1 = 0.0725

# Thin recessed separation groove ring around the upper third.
GROOVE_R = 0.0215
GROOVE_S0 = 0.0705
GROOVE_S1 = 0.0760

# Stepped-in upper neck above the groove.
NECK_R = 0.0228
NECK_S0 = 0.0740
NECK_S1 = 0.104

# Spout exit station on the body axis.
SPOUT_S = 0.050

# Lever cartridge (pivot dome on top of neck, rotates for temperature).
CARTRIDGE_R = 0.016
CARTRIDGE_H = 0.016
CARTRIDGE_S0 = NECK_S1
CARTRIDGE_S1 = CARTRIDGE_S0 + CARTRIDGE_H

# Lever arm dimensions (extends along +Y from the cartridge pivot).
LEVER_ARM_LEN = 0.072
LEVER_ARM_R = 0.007
LEVER_ARM_OFFSET_Y = 0.012  # arm root offset from pivot center

# Grip groove parameters (thin rings along the lever arm grip zone).
GRIP_GROOVE_R = LEVER_ARM_R + 0.0003  # slightly proud of the arm surface
GRIP_GROOVE_H = 0.0012
GRIP_GROOVE_COUNT = 5
GRIP_START_Y = 0.030  # first groove Y offset from lever origin
GRIP_SPACING = 0.008  # spacing between grooves

# Joint limits.
LEVER_LIFT_MAX = math.radians(40.0)   # 0 = closed, max = full flow
LEVER_TURN_LIMIT = math.radians(45.0)  # -45 to +45 for temperature


def _axis_point(s: float) -> tuple[float, float, float]:
    """World position of the tilted body axis at axial station s."""
    return (-s * SIN_T, 0.0, s * COS_T)


def _tilted(s: float) -> Origin:
    """Origin on the body axis at station s, z-axis aligned with the axis."""
    return Origin(xyz=_axis_point(s), rpy=(0.0, -TILT, 0.0))


def _build_spout_shape() -> cq.Workplane:
    """Hollow chrome spout: straight shank, smooth downward bend, flared
    open outlet rim. Built in spout-local frame whose origin sits on the
    body axis at SPOUT_S; the shank runs along local +X."""
    r_out = 0.015
    shank_x0 = 0.010  # seated ~15 mm inside the body casting
    shank_x1 = 0.035
    bend = 0.028  # bend radius; end heads straight down at (0.063, -0.028)
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
        cq.Workplane("XY", origin=(end_x, 0.0, end_z - 0.006))
        .circle(0.0155)
        .workplane(offset=0.018)
        .circle(0.011)
        .loft()
    )
    return spout.cut(bore)


def _build_lever_arm_shape() -> cq.Workplane:
    """Lever arm: cylindrical bar extending along +Y from the pivot.
    Lever-local frame: origin at the pivot center, arm extends along +Y.
    Built as a single body to avoid boolean-union mesh islands."""
    y_start = LEVER_ARM_OFFSET_Y
    y_end = LEVER_ARM_OFFSET_Y + LEVER_ARM_LEN
    # Main arm: circle at far end, extrude back toward root (along -Y normal).
    arm = (
        cq.Workplane("XZ", origin=(0.0, y_end, 0.0))
        .circle(LEVER_ARM_R)
        .extrude(LEVER_ARM_LEN)
    )
    # Fillet both ends for a finished look.
    arm = arm.edges().fillet(LEVER_ARM_R * 0.35)
    return arm


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    model.material("chrome_dark", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("chrome_brushed", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("grip_groove", rgba=(0.30, 0.32, 0.35, 1.0))

    # ---------------- body (root): flange + collar + barrel + groove + neck -
    body = model.part("body")
    body.visual(
        Cylinder(radius=FLANGE_R, length=FLANGE_H),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_H / 2.0)),
        material="chrome",
        name="base_flange",
    )
    # Raised circular collar around the base (variant feature).
    body.visual(
        mesh_from_cadquery(
            _build_collar_shape(), "base_collar", tolerance=0.0003,
        ),
        origin=_tilted((COLLAR_S0 + COLLAR_S1) / 2.0),
        material="chrome",
        name="base_collar",
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
    model.articulation(
        "spout_mount",
        ArticulationType.FIXED,
        parent=body,
        child=spout,
        origin=Origin(xyz=_axis_point(SPOUT_S)),
    )

    # ---------------- lever cartridge (revolute: side-to-side temperature) -
    cartridge = model.part("lever_cartridge")
    cartridge.visual(
        mesh_from_cadquery(
            _build_cartridge_shape(), "lever_cartridge", tolerance=0.0003,
        ),
        origin=Origin(),
        material="chrome",
        name="cartridge_dome",
    )
    # Joint frame on the body axis at the neck top; local +z runs up the
    # tilted axis. Axis +z makes positive q rotate the lever side-to-side
    # around the body long axis (temperature adjustment).
    model.articulation(
        "lever_turn",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cartridge,
        origin=Origin(xyz=_axis_point(CARTRIDGE_S0), rpy=(0.0, -TILT, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=2.0,
            lower=-LEVER_TURN_LIMIT, upper=LEVER_TURN_LIMIT,
        ),
    )

    # ---------------- lever handle (revolute: lift up for flow) -----------
    lever = model.part("lever_handle")
    lever.visual(
        mesh_from_cadquery(
            _build_lever_arm_shape(), "lever_handle", tolerance=0.0003,
        ),
        origin=Origin(),
        material="chrome",
        name="lever_arm",
    )
    # Root ball where arm meets the pivot (overlaps arm root for connectivity).
    lever.visual(
        Cylinder(radius=LEVER_ARM_R * 1.3, length=LEVER_ARM_R * 2.0),
        origin=Origin(
            xyz=(0.0, LEVER_ARM_OFFSET_Y, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="chrome",
        name="lever_root_ball",
    )
    # Tip cap at the grip end (overlaps arm tip for connectivity).
    lever.visual(
        Cylinder(radius=LEVER_ARM_R * 1.1, length=LEVER_ARM_R * 1.5),
        origin=Origin(
            xyz=(0.0, LEVER_ARM_OFFSET_Y + LEVER_ARM_LEN, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="chrome_brushed",
        name="lever_tip_cap",
    )
    # Grip grooves: thin dark rings along the arm grip zone.
    # Each ring slightly overlaps the arm shaft for connectivity.
    for i in range(GRIP_GROOVE_COUNT):
        y_pos = GRIP_START_Y + i * GRIP_SPACING
        lever.visual(
            Cylinder(radius=GRIP_GROOVE_R, length=GRIP_GROOVE_H),
            origin=Origin(
                xyz=(0.0, y_pos, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="grip_groove",
            name=f"grip_groove_{i}",
        )
    # Lift joint: cartridge carries the lever; axis along +X in cartridge
    # frame so positive q lifts the arm upward (away from the faucet body).
    # The lever arm extends along +Y in its own frame; rotating around X
    # tilts the +Y arm upward toward +Z.
    model.articulation(
        "lever_lift",
        ArticulationType.REVOLUTE,
        parent=cartridge,
        child=lever,
        origin=Origin(xyz=(0.0, 0.0, CARTRIDGE_H * 0.6)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=1.5,
            lower=0.0, upper=LEVER_LIFT_MAX,
        ),
    )

    return model


def _build_collar_shape() -> cq.Workplane:
    """Raised circular collar: a short ring wider than the body barrel,
    with a filleted top edge. Built around local Z axis, centered at z=0."""
    h = COLLAR_H
    collar = (
        cq.Workplane("XY")
        .circle(COLLAR_R)
        .extrude(h)
    )
    # Chamfer the top outer edge for a finished look.
    collar = collar.edges(">Z").fillet(0.0015)
    # Offset so center is at z=0 (visual origin convention).
    return collar.translate((0.0, 0.0, -h / 2.0))


def _build_cartridge_shape() -> cq.Workplane:
    """Lever cartridge pivot dome: a short cylinder with a domed top,
    built around local Z axis, bottom at z=0."""
    r = CARTRIDGE_R
    h = CARTRIDGE_H
    body_cyl = cq.Workplane("XY").circle(r).extrude(h)
    dome = (
        cq.Workplane("XY", origin=(0.0, 0.0, h))
        .circle(r)
        .workplane(offset=r * 0.4)
        .circle(r * 0.5)
        .loft()
    )
    cartridge = body_cyl.union(dome)
    # Small chamfer at the base where it meets the neck.
    cartridge = cartridge.edges("<Z").chamfer(0.001)
    return cartridge


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    cartridge = object_model.get_part("lever_cartridge")
    lever = object_model.get_part("lever_handle")
    turn = object_model.get_articulation("lever_turn")
    lift = object_model.get_articulation("lever_lift")

    # Intentional seated insertions (solid proxies, scoped per element).
    ctx.allow_overlap(
        spout,
        body,
        elem_a="spout_tube",
        elem_b="body_barrel",
        reason="Spout shank is intentionally seated ~15 mm into the solid body casting.",
    )
    ctx.allow_overlap(
        cartridge,
        body,
        elem_a="cartridge_dome",
        elem_b="body_neck",
        reason="Cartridge base nests into the neck top at the tilted interface.",
    )
    ctx.allow_overlap(
        lever,
        cartridge,
        elem_a="lever_arm",
        elem_b="cartridge_dome",
        reason="Lever arm root is intentionally nested inside the cartridge dome pivot.",
    )
    ctx.allow_overlap(
        lever,
        cartridge,
        elem_a="lever_root_ball",
        elem_b="cartridge_dome",
        reason="Lever root ball is seated inside the cartridge dome for pivot support.",
    )

    # ---- hero geometry: flange seated on deck, collar present -------------
    flange_aabb = ctx.part_element_world_aabb(body, elem="base_flange")
    ctx.check(
        "base flange sits flat on the deck",
        flange_aabb is not None and abs(flange_aabb[0][2]) <= 0.0005,
        details=f"flange aabb={flange_aabb}",
    )
    collar_aabb = ctx.part_element_world_aabb(body, elem="base_collar")
    ctx.check(
        "raised collar sits above the flange around the base",
        collar_aabb is not None
        and flange_aabb is not None
        and collar_aabb[0][2] >= flange_aabb[0][2] - 0.001
        and collar_aabb[1][2] > flange_aabb[1][2] - 0.001,
        details=f"collar aabb={collar_aabb}, flange aabb={flange_aabb}",
    )
    ctx.check(
        "collar is wider than the body barrel",
        collar_aabb is not None
        and (collar_aabb[1][1] - collar_aabb[0][1]) > 2.0 * BODY_R + 0.005,
        details=f"collar aabb={collar_aabb}",
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

    # ---- lever cartridge: sits above the neck -----------------------------
    cartridge_aabb = ctx.part_world_aabb(cartridge)
    ctx.check(
        "lever cartridge sits at the top of the neck",
        cartridge_aabb is not None
        and neck_aabb is not None
        and cartridge_aabb[0][2] >= neck_aabb[1][2] - 0.005,
        details=f"cartridge aabb={cartridge_aabb}, neck aabb={neck_aabb}",
    )

    # ---- lever handle: extends from the cartridge -------------------------
    lever_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "lever handle extends laterally from the cartridge",
        lever_aabb is not None
        and cartridge_aabb is not None
        and (lever_aabb[1][1] - lever_aabb[0][1]) > 0.040,
        details=f"lever aabb={lever_aabb}, cartridge aabb={cartridge_aabb}",
    )

    # ---- grip grooves present on the lever --------------------------------
    grip_viz = [v for v in lever.visuals if v.name and v.name.startswith("grip_groove_")]
    ctx.check(
        f"lever has {GRIP_GROOVE_COUNT} grip grooves on the handle",
        len(grip_viz) == GRIP_GROOVE_COUNT,
        details=f"found {len(grip_viz)} groove visuals",
    )

    # ---- overall faucet height --------------------------------------------
    ctx.check(
        "overall faucet height is about 0.12 to 0.14 m",
        lever_aabb is not None and 0.115 <= lever_aabb[1][2] <= 0.145,
        details=f"lever aabb={lever_aabb}",
    )

    # ---- articulation limits match the prompt ------------------------------
    tl = turn.motion_limits
    ctx.check(
        "lever turn limits are -45 to +45 degrees for temperature",
        tl is not None
        and tl.lower is not None
        and tl.upper is not None
        and abs(tl.lower + LEVER_TURN_LIMIT) < 1e-6
        and abs(tl.upper - LEVER_TURN_LIMIT) < 1e-6,
        details=f"limits={tl}",
    )
    ll = lift.motion_limits
    ctx.check(
        "lever lift limits are 0 to 40 degrees for flow control",
        ll is not None
        and ll.lower is not None
        and ll.upper is not None
        and abs(ll.lower) < 1e-9
        and abs(ll.upper - LEVER_LIFT_MAX) < 1e-6,
        details=f"limits={ll}",
    )

    # ---- decisive poses: lift raises the lever arm, turn swings it --------
    arm_rest_aabb = ctx.part_element_world_aabb(lever, elem="lever_arm")
    with ctx.pose({lift: LEVER_LIFT_MAX}):
        arm_lifted_aabb = ctx.part_element_world_aabb(lever, elem="lever_arm")
    ctx.check(
        "lifting the lever raises the arm tip upward (positive Z movement)",
        arm_rest_aabb is not None
        and arm_lifted_aabb is not None
        and arm_lifted_aabb[1][2] > arm_rest_aabb[1][2] + 0.010,
        details=f"rest_max_z={arm_rest_aabb[1][2]}, lifted_max_z={arm_lifted_aabb[1][2]}",
    )

    arm_turn_rest_aabb = ctx.part_element_world_aabb(lever, elem="lever_arm")
    with ctx.pose({turn: LEVER_TURN_LIMIT}):
        arm_turned_aabb = ctx.part_element_world_aabb(lever, elem="lever_arm")
    ctx.check(
        "turning the lever swings the arm side-to-side (X displacement)",
        arm_turn_rest_aabb is not None
        and arm_turned_aabb is not None
        and abs(arm_turned_aabb[0][0] - arm_turn_rest_aabb[0][0]) > 0.020,
        details=f"rest_min_x={arm_turn_rest_aabb[0][0]}, turned_min_x={arm_turned_aabb[0][0]}",
    )

    return ctx.report()


object_model = build_object_model()
