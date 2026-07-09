from __future__ import annotations

# Aerosol SPRAY PAINT CAN with a clip-on pistol-grip handle adapter.
# The can body, crimped dome, and lift-off dust cap are IDENTICAL to the parent.
# The press-down nozzle button is REPLACED by a pistol-grip adapter that clamps
# around the can's upper straight wall and carries a finger trigger on a
# REVOLUTE joint, with a linkage arm pressing the valve stem down.
#
# Frame: can axis along +Z. Base rim at z=0, body rises in +Z, crimped dome
# and valve stem at the top. Cylinder ~0.20 m tall, ~0.065 m diameter.
#
# Articulations:
#   - cap_lift:      PRISMATIC, dust cap lifts UP (+Z, 0.090 m) — IDENTICAL
#   - grip_mount:    FIXED, pistol grip clamps around can body — NEW
#   - trigger_pull:  REVOLUTE, finger trigger swings on Y-axis — NEW

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key dimensions (meters) ----
CAN_R = 0.0325  # body radius (~0.065 m diameter)
BODY_BOTTOM = 0.0  # base plane
BODY_TOP = 0.150  # where the straight wall ends and the dome begins
DOME_TOP = 0.176  # top of the crimped dome (mounting cup rim)
VALVE_TOP = 0.182  # top of the central valve stem
CAP_BOTTOM = 0.150  # dust cap lower rim
CAP_TOP = 0.210  # dust cap top

# ---- pistol grip adapter dimensions ----
GRIP_CENTER_Z = 0.120  # clamp ring center height, safely on straight wall
GRIP_CLAMP_H = 0.034  # clamp ring height (0.103 to 0.137), below shoulder
GRIP_HALF_H = GRIP_CLAMP_H / 2.0
GRIP_INNER_R = CAN_R + 0.0020  # clear label/can wall, avoids visual z-fighting
GRIP_OUTER_R = CAN_R + 0.0070  # firm but slimmer clamp wall

# Linkage geometry (in grip local frame, origin at clamp center)
LINKAGE_ARM_H = DOME_TOP - (GRIP_CENTER_Z + GRIP_HALF_H)  # 0.176-0.150 = 0.026
LINKAGE_ARM_X = -(GRIP_OUTER_R + 0.003)  # arm on -X (back) side

# Trigger pivot in grip local frame. Keep it outboard of the can wall so the
# lever sweep has a real air gap instead of grazing the bottle.
TRIGGER_PIVOT_X = -(GRIP_OUTER_R + 0.012)
TRIGGER_PIVOT_Z = -GRIP_HALF_H - 0.008
TRIGGER_SWING_MAX = 0.26
HANDLE_W = 0.024  # chunkier so it reads as a hand grip, not a thin bar
HANDLE_D = 0.044
HANDLE_CX = TRIGGER_PIVOT_X - 0.030

# Pistol-grip rake: the handle rakes backward (its bottom swings to -X) about
# its top edge so the hand and knuckles clear the can, like a real spray-gun
# "can gun" handle. The top stays put, so the bracket/webs still meet it.
BACK_PLATE_H = 0.034
HANDLE_H = 0.072
HANDLE_TOP_Z = -GRIP_HALF_H - BACK_PLATE_H + 0.003  # grip-local z of handle top
HANDLE_RAKE_DEG = 13.0
HANDLE_RAKE_SIN = math.sin(math.radians(HANDLE_RAKE_DEG))


# ---------------------------------------------------------------------------
# Geometry builders
# ---------------------------------------------------------------------------


def _rake(solid: cq.Workplane) -> cq.Workplane:
    """Rake a sub-solid backward about a Y-axis through the handle top, giving
    the handle a pistol-grip angle without moving where it joins the bracket."""
    return solid.rotate(
        (HANDLE_CX, -1.0, HANDLE_TOP_Z),
        (HANDLE_CX, 1.0, HANDLE_TOP_Z),
        HANDLE_RAKE_DEG,
    )


def _can_body_solid() -> cq.Workplane:
    """Can body: bottom rim, straight wall, crimped dome, central valve stem.
    IDENTICAL to parent."""
    body = cq.Workplane("XY").circle(CAN_R + 0.0012).extrude(0.006)
    body = body.union(
        cq.Workplane("XY").workplane(offset=0.005).circle(CAN_R).extrude(BODY_TOP - 0.005)
    )
    dome = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP - 0.001)
        .circle(CAN_R)
        .workplane(offset=0.006)
        .circle(CAN_R - 0.001)
        .workplane(offset=0.004)
        .circle(CAN_R - 0.006)
        .workplane(offset=0.017)
        .circle(0.014)
        .loft(ruled=False)
    )
    body = body.union(dome)
    body = body.union(
        cq.Workplane("XY")
        .workplane(offset=DOME_TOP - 0.001)
        .circle(0.006)
        .extrude(VALVE_TOP - DOME_TOP)
    )
    return body


def _dust_cap_solid() -> cq.Workplane:
    """Hollow dust cap with a rear service slot for the trigger linkage."""
    h = CAP_TOP - CAP_BOTTOM
    outer = cq.Workplane("XY").circle(CAN_R + 0.0035).extrude(h - 0.010)
    top = (
        cq.Workplane("XY")
        .workplane(offset=h - 0.011)
        .circle(CAN_R + 0.0035)
        .workplane(offset=0.007)
        .circle(CAN_R - 0.004)
        .workplane(offset=0.004)
        .circle(0.010)
        .loft(ruled=False)
    )
    cap = outer.union(top)
    bore = cq.Workplane("XY").workplane(offset=-0.001).circle(CAN_R - 0.0006).extrude(h - 0.014)
    cap = cap.cut(bore)
    # The clip-on pistol adapter has an external rear linkage. The cap only
    # needs a slim notch at its lower back wall for the linkage bridge to pass
    # and for the cap to slide up over it — not a full-height window through the
    # dome (which read as a broken cap).
    rear_slot = (
        cq.Workplane("XY")
        .workplane(offset=-0.002)
        .center(-(CAN_R + 0.003), 0)
        .rect(0.026, 0.018)
        .extrude(0.034)
    )
    cap = cap.cut(rear_slot)
    return cap


def _pistol_grip_solid() -> cq.Workplane:
    """Pistol grip adapter body in local coordinates.
    Origin at clamp ring center on can axis. +Z up. C-opening faces +X."""
    half_h = GRIP_HALF_H  # 0.020

    # --- 1. Clamp collar: C-shaped ring, open on +X ---
    collar = cq.Workplane("XY").workplane(offset=-half_h).circle(GRIP_OUTER_R).extrude(GRIP_CLAMP_H)
    collar = collar.cut(
        cq.Workplane("XY")
        .workplane(offset=-half_h - 0.001)
        .circle(GRIP_INNER_R)
        .extrude(GRIP_CLAMP_H + 0.002)
    )
    # C-opening on +X side
    collar = collar.cut(
        cq.Workplane("XY")
        .workplane(offset=-half_h - 0.001)
        .center(GRIP_OUTER_R * 0.55, 0)
        .rect(GRIP_OUTER_R * 1.2, GRIP_OUTER_R * 0.90)
        .extrude(GRIP_CLAMP_H + 0.002)
    )

    # --- 2. Back plate: connects collar to handle and linkage ---
    # A flat vertical plate on the -X side that overlaps the collar wall
    back_plate_x = -GRIP_OUTER_R + 0.004  # inner face at -0.0365
    back_plate_w = 0.010
    back_plate_h = 0.034
    back_plate = (
        cq.Workplane("XY")
        .workplane(offset=-half_h - back_plate_h + 0.001)
        .center(back_plate_x - back_plate_w / 2.0, 0)
        .box(back_plate_w, 0.030, back_plate_h + half_h + 0.002, centered=(True, True, False))
    )

    # --- 3. Handle body (raked back like a pistol grip) ---
    handle_h = HANDLE_H
    handle_cx = HANDLE_CX
    handle = (
        cq.Workplane("XY")
        .workplane(offset=-half_h - back_plate_h - handle_h + 0.001)
        .center(handle_cx, 0)
        .box(HANDLE_W, HANDLE_D, handle_h + 0.002, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.007)
    )
    handle = _rake(handle)

    # Two side webs connect the collar/back plate to the rear handle while
    # leaving a central channel for the trigger sweep.
    web_front_x = back_plate_x - back_plate_w
    web_back_x = handle_cx + HANDLE_W / 2.0 + 0.006  # overshoot to bridge the rake gap
    web_span = abs(web_front_x - web_back_x) + 0.002
    web_cx = (web_front_x + web_back_x) / 2.0
    web_z = -half_h - back_plate_h - 0.004
    side_web_plus = (
        cq.Workplane("XY")
        .workplane(offset=web_z)
        .center(web_cx, 0.014)
        .box(web_span, 0.006, 0.010, centered=(True, True, False))
    )
    side_web_minus = (
        cq.Workplane("XY")
        .workplane(offset=web_z)
        .center(web_cx, -0.014)
        .box(web_span, 0.006, 0.010, centered=(True, True, False))
    )

    # --- 4. Trigger guard front bar ---
    guard_x = -GRIP_OUTER_R - 0.004  # forward stop, still outside the can wall
    guard_front = (
        cq.Workplane("XY")
        .workplane(offset=-half_h - back_plate_h - 0.038)
        .center(guard_x, 0)
        .box(0.006, 0.026, 0.038, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.002)
    )
    # Guard bottom bridge — reach back to the raked handle bottom.
    guard_z = -half_h - back_plate_h - 0.038
    handle_x_at_guard = handle_cx - (HANDLE_TOP_Z - guard_z) * HANDLE_RAKE_SIN
    guard_bridge = (
        cq.Workplane("XY")
        .workplane(offset=guard_z)
        .center((handle_x_at_guard + guard_x) / 2.0, 0)
        .box(abs(guard_x - handle_x_at_guard) + 0.008, 0.026, 0.005, centered=(True, True, False))
    )

    # --- 5. Linkage arm: from collar top going up to dome level on -X side ---
    arm_x = -(GRIP_OUTER_R + 0.002)
    arm_h = LINKAGE_ARM_H  # 0.026
    linkage_arm = (
        cq.Workplane("XY")
        .workplane(offset=-0.005)  # start deep inside collar
        .center(arm_x, 0)
        .box(0.012, 0.014, half_h + arm_h + 0.005, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.002)
    )

    # --- 6. Linkage bridge: horizontal bar from arm top over dome to valve ---
    bridge_z = half_h + arm_h - 0.006
    bridge_len = abs(arm_x) + 0.006  # spans from arm to over valve stem
    bridge = (
        cq.Workplane("XY")
        .workplane(offset=bridge_z)
        .center(arm_x / 2.0 + 0.002, 0)
        .box(bridge_len, 0.012, 0.005, centered=(True, True, False))
    )

    # --- 7. Plunger head over valve stem ---
    plunger = cq.Workplane("XY").workplane(offset=bridge_z).circle(0.006).extrude(0.008)

    # --- 8. Trigger pivot ears: two tabs from back plate for trigger mount ---
    # Ears straddle y=0 at the trigger pivot z level
    ear_w = 0.014
    ear_t = 0.008  # thickness in Y
    ear_gap = 0.005  # inner face at y = ±ear_gap — boss at ±0.010 overlaps by 0.005
    ear_depth = 0.014  # height of ear
    # Position ears overlapping the back plate at the pivot location
    ear_z_top = TRIGGER_PIVOT_Z + 0.005
    ear_plus = (
        cq.Workplane("XY")
        .workplane(offset=ear_z_top - ear_depth)
        .center(TRIGGER_PIVOT_X, ear_gap + ear_t / 2.0)
        .box(ear_w, ear_t, ear_depth, centered=(True, True, False))
    )
    ear_minus = (
        cq.Workplane("XY")
        .workplane(offset=ear_z_top - ear_depth)
        .center(TRIGGER_PIVOT_X, -(ear_gap + ear_t / 2.0))
        .box(ear_w, ear_t, ear_depth, centered=(True, True, False))
    )

    # Union all pieces
    grip = collar.union(back_plate).union(handle)
    grip = grip.union(side_web_plus).union(side_web_minus)
    grip = grip.union(guard_front).union(guard_bridge)
    grip = grip.union(linkage_arm).union(bridge).union(plunger)
    grip = grip.union(ear_plus).union(ear_minus)
    return grip


def _grip_rib_solid(z_offset: float) -> cq.Workplane:
    """Single handle grip rib on the handle back face in grip local frame.
    Split into two short side pads so the trigger never sweeps through bars."""
    rib_x = HANDLE_CX - HANDLE_W / 2.0 - 0.002
    total_y = 0.044
    slot_y = 0.026
    side_y = (total_y - slot_y) / 2.0
    offset_y = slot_y / 2.0 + side_y / 2.0
    rib_plus = (
        cq.Workplane("XY")
        .workplane(offset=z_offset)
        .center(rib_x, offset_y)
        .box(0.004, side_y, 0.004, centered=(True, True, False))
        .edges(">Z")
        .fillet(0.001)
    )
    rib_minus = (
        cq.Workplane("XY")
        .workplane(offset=z_offset)
        .center(rib_x, -offset_y)
        .box(0.004, side_y, 0.004, centered=(True, True, False))
        .edges(">Z")
        .fillet(0.001)
    )
    return _rake(rib_plus.union(rib_minus))


def _trigger_solid() -> cq.Workplane:
    """Curved trigger BLADE in local coordinates. Origin at the pivot; the
    blade hangs down (-Z) and curls slightly forward (+X, toward the finger),
    flaring into a finger pad at the tip — a real trigger, not a square nub."""
    blade = (
        cq.Workplane("XY")
        .workplane(offset=0.004)
        .rect(0.007, 0.016)
        .workplane(offset=-0.024)
        .rect(0.007, 0.018)
        .workplane(offset=-0.020)
        .center(0.001, 0)
        .rect(0.008, 0.020)
        .workplane(offset=-0.018)
        .center(0.003, 0)
        .rect(0.010, 0.022)
        .loft(ruled=False)
    )
    # Pivot boss cylinder along Y — seated between the grip ears.
    boss = cq.Workplane("XZ").workplane(offset=-0.010).circle(0.005).extrude(0.020)
    return blade.union(boss)


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spray_paint_can_pistol_grip")

    # Materials
    label_mat = model.material("label_splatter", rgba=(0.90, 0.90, 0.92, 1.0))
    metal_mat = model.material("metal", rgba=(0.78, 0.80, 0.83, 1.0))
    dark_cap_mat = model.material("dark_grey_cap", rgba=(0.20, 0.21, 0.20, 1.0))
    grip_mat = model.material("grip_black", rgba=(0.10, 0.10, 0.12, 1.0))
    trigger_mat = model.material("trigger_steel", rgba=(0.26, 0.27, 0.30, 1.0))

    # ---- can body (root) — IDENTICAL to parent ----
    body = model.part("can_body")
    body.visual(
        mesh_from_cadquery(_can_body_solid(), "can_body"),
        material=metal_mat,
        name="can_body",
    )
    label_band = CylinderGeometry(CAN_R + 0.0004, 0.105, radial_segments=64)
    label_band.translate(0.0, 0.0, 0.012 + 0.105 / 2.0)
    body.visual(
        mesh_from_geometry(label_band, "label_band"),
        material=label_mat,
        name="label_band",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(CAN_R, 0.176),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
    )

    # ---- pistol grip adapter: FIXED child of can body ----
    grip = model.part("pistol_grip")
    grip.visual(
        mesh_from_cadquery(_pistol_grip_solid(), "pistol_grip_body"),
        material=grip_mat,
        name="pistol_grip_body",
    )
    # Handle grip ribs — repeated decoration via for-loop, raked with the handle
    # Handle z range: ~-0.125 to -0.051 in grip local frame
    rib_z_offsets = [-0.058 - i * 0.014 for i in range(4)]
    for i, z_off in enumerate(rib_z_offsets):
        grip.visual(
            mesh_from_cadquery(_grip_rib_solid(z_off), f"grip_rib_{i}"),
            material=grip_mat,
            name=f"grip_rib_{i}",
        )
    grip.inertial = Inertial.from_geometry(
        Box((0.060, 0.030, 0.120)),
        mass=0.080,
        origin=Origin(xyz=(-0.020, 0.0, -0.030)),
    )
    model.articulation(
        "grip_mount",
        ArticulationType.FIXED,
        parent=body,
        child=grip,
        origin=Origin(xyz=(0.0, 0.0, GRIP_CENTER_Z)),
    )

    # ---- trigger: REVOLUTE child of pistol grip ----
    trigger = model.part("trigger")
    trigger.visual(
        mesh_from_cadquery(_trigger_solid(), "trigger_lever"),
        material=trigger_mat,
        name="trigger_lever",
    )
    trigger.inertial = Inertial.from_geometry(
        Box((0.012, 0.022, 0.062)),
        mass=0.008,
        origin=Origin(xyz=(0.002, 0.0, -0.027)),
    )
    model.articulation(
        "trigger_pull",
        ArticulationType.REVOLUTE,
        parent=grip,
        child=trigger,
        origin=Origin(xyz=(TRIGGER_PIVOT_X, 0.0, TRIGGER_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=0.0,
            upper=TRIGGER_SWING_MAX,
        ),
    )

    # ---- dust cap: IDENTICAL to parent ----
    cap = model.part("dust_cap")
    cap.visual(
        mesh_from_cadquery(_dust_cap_solid(), "dust_cap"),
        material=dark_cap_mat,
        name="dust_cap",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAN_R + 0.0035, CAP_TOP - CAP_BOTTOM),
        mass=0.015,
        origin=Origin(xyz=(0.0, 0.0, (CAP_TOP - CAP_BOTTOM) / 2.0)),
    )
    model.articulation(
        "cap_lift",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, CAP_BOTTOM)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=0.2,
            lower=0.0,
            upper=0.090,
        ),
    )

    return model


object_model = build_object_model()


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("can_body")
    grip = object_model.get_part("pistol_grip")
    trigger = object_model.get_part("trigger")
    cap = object_model.get_part("dust_cap")

    cap_lift = object_model.get_articulation("cap_lift")
    trigger_pull = object_model.get_articulation("trigger_pull")

    # ---- can body is a tall cylinder (unchanged) ----
    bmn, bmx = ctx.part_world_aabb(body)
    height = bmx[2] - bmn[2]
    dia_x = bmx[0] - bmn[0]
    dia_y = bmx[1] - bmn[1]
    ctx.check(
        "can body is a tall cylinder (~0.18 m, ~0.065 m dia)",
        height > 0.16 and 0.05 < dia_x < 0.08 and 0.05 < dia_y < 0.08 and height > dia_x * 2.0,
        details=f"height={height:.3f}, dia_x={dia_x:.3f}, dia_y={dia_y:.3f}",
    )

    # ---- pistol grip is mounted on the can upper body ----
    grip_pos = ctx.part_world_position(grip)
    ctx.check(
        "pistol grip is mounted on the can upper body",
        grip_pos is not None and 0.10 < grip_pos[2] < 0.16,
        details=f"grip origin z={grip_pos[2]:.3f}",
    )

    # ---- grip has a visible handle extending below the clamp ----
    gmn, gmx = ctx.part_world_aabb(grip)
    handle_span = GRIP_CENTER_Z - GRIP_HALF_H - gmn[2]
    ctx.check(
        "grip handle extends well below the clamp (>0.04 m)",
        handle_span > 0.04,
        details=f"handle span below clamp={handle_span:.3f} m",
    )

    # ---- grip clamp wraps around the can (intentional overlap at clamp bore) ----
    ctx.allow_overlap(
        grip,
        body,
        elem_a="pistol_grip_body",
        elem_b="can_body",
        reason="The grip clamp collar wraps around the can body wall as a seated clamp fit.",
    )
    ctx.allow_overlap(
        grip,
        body,
        elem_a="pistol_grip_body",
        elem_b="label_band",
        reason="The grip clamp collar sleeves over the label band region on the can wall.",
    )
    ctx.expect_contact(
        grip,
        body,
        elem_a="pistol_grip_body",
        elem_b="can_body",
        name="grip clamp contacts the can body",
    )

    # ---- cap has a rear slot, so the linkage and cap do not need to share volume ----

    # ---- trigger pivot boss embedded in grip pivot ears (bearing fit) ----
    ctx.allow_overlap(
        grip,
        trigger,
        elem_a="pistol_grip_body",
        elem_b="trigger_lever",
        reason="Trigger pivot boss is seated between the grip pivot ears as a bearing fit.",
    )
    ctx.expect_contact(
        grip,
        trigger,
        name="trigger contacts grip at pivot mount",
    )

    # ---- trigger is positioned between handle and can body ----
    trig_pos = ctx.part_world_position(trigger)
    trig_mn, trig_mx = ctx.part_world_aabb(trigger)
    ctx.check(
        "trigger is on the back side of the can (negative X from axis)",
        trig_pos[0] < -0.010 and trig_pos[0] > -0.080,
        details=f"trigger_x={trig_pos[0]:.4f}",
    )

    # ---- trigger swings on its REVOLUTE joint (use AABB since origin is at pivot) ----
    rest_mn, rest_mx = ctx.part_world_aabb(trigger)
    rest_x_min = rest_mn[0]
    with ctx.pose({trigger_pull: TRIGGER_SWING_MAX}):
        pull_mn, pull_mx = ctx.part_world_aabb(trigger)
    pulled_x_min = pull_mn[0]
    ctx.check(
        "trigger swings when pulled (AABB shifts in X)",
        abs(pulled_x_min - rest_x_min) > 0.008,
        details=f"rest_x_min={rest_x_min:.4f}, pulled_x_min={pulled_x_min:.4f}",
    )

    # The trigger no longer sweeps through the can wall or the handle ribs.
    clearance_samples = (0.0, TRIGGER_SWING_MAX * 0.5, TRIGGER_SWING_MAX)
    can_negative_x = bmn[0]
    trigger_clears_can = True
    trigger_clears_ribs = True
    clearance_details = []
    rib_details = []
    for angle in clearance_samples:
        with ctx.pose({trigger_pull: angle}):
            trig_mn, trig_mx = ctx.part_element_world_aabb(trigger, elem="trigger_lever")
            clearance = can_negative_x - trig_mx[0]
            trigger_clears_can = trigger_clears_can and clearance > 0.006
            clearance_details.append(f"{angle:.2f}:{clearance:.4f}")

            for rib_index in range(4):
                rib_mn, rib_mx = ctx.part_element_world_aabb(
                    grip,
                    elem=f"grip_rib_{rib_index}",
                )
                separated = (
                    trig_mx[0] < rib_mn[0]
                    or trig_mn[0] > rib_mx[0]
                    or trig_mx[1] < rib_mn[1]
                    or trig_mn[1] > rib_mx[1]
                    or trig_mx[2] < rib_mn[2]
                    or trig_mn[2] > rib_mx[2]
                )
                trigger_clears_ribs = trigger_clears_ribs and separated
                if not separated:
                    rib_details.append(
                        f"angle={angle:.2f} rib={rib_index} "
                        f"trig=({trig_mn}, {trig_mx}) rib=({rib_mn}, {rib_mx})"
                    )
    ctx.check(
        "trigger sweep stays outside the can body",
        trigger_clears_can,
        details="clearance at samples " + ", ".join(clearance_details),
    )
    ctx.check(
        "trigger sweep does not pass through handle grip ribs",
        trigger_clears_ribs,
        details="; ".join(rib_details) if rib_details else "all sampled poses clear",
    )

    # ---- trigger at rest overlaps grip footprint in XY ----
    ctx.expect_overlap(
        trigger,
        grip,
        axes="xy",
        min_overlap=0.004,
        name="trigger overlaps grip footprint in XY",
    )

    # ---- dust cap: seated over the can top at rest ----
    ctx.allow_overlap(
        cap,
        body,
        elem_a="dust_cap",
        elem_b="can_body",
        reason="Dust cap intentionally sleeves over the can top shoulder when seated.",
    )
    cap_rest_mn, cap_rest_mx = ctx.part_world_aabb(cap)
    ctx.check(
        "dust cap is seated over the can top at rest",
        cap_rest_mn[2] < 0.16 and cap_rest_mx[2] > 0.20,
        details=f"cap z range=({cap_rest_mn[2]:.3f}, {cap_rest_mx[2]:.3f})",
    )

    # ---- dust cap lifts straight up a LARGE amount ----
    cap_rest_pos_z = ctx.part_world_position(cap)[2]
    with ctx.pose({cap_lift: 0.090}):
        cap_up_mn, cap_up_mx = ctx.part_world_aabb(cap)
        cap_up_pos = ctx.part_world_position(cap)
    ctx.check(
        "dust cap lifts straight up by ~0.09 m",
        cap_up_pos[2] - cap_rest_pos_z > 0.085,
        details=f"cap rose from z={cap_rest_pos_z:.3f} to z={cap_up_pos[2]:.3f}",
    )
    ctx.check(
        "cap lift is purely vertical (no lateral drift)",
        abs(cap_up_pos[0]) < 0.005 and abs(cap_up_pos[1]) < 0.005,
        details=f"lifted cap xy=({cap_up_pos[0]:.4f}, {cap_up_pos[1]:.4f})",
    )

    return ctx.report()
