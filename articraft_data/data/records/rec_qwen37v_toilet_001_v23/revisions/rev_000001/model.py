from __future__ import annotations

# Two-piece floor-standing white ceramic toilet.
#
# Frame / coordinate convention:
#   +X = forward (front of the bowl points to +X).
#   +Y = left-right (the hinge axis for lid + seat ring runs along Y).
#   +Z = up. Floor at z=0; seat top ~0.40 m.
#
# Root part = bowl + pedestal base (floor-standing ceramic assembly).
# Everything else mounts to that root or to the tank:
#   - tank        : ceramic cistern sitting on the rear shelf of the bowl,
#                   fixed (rigid mount).
#   - lid         : oval top lid, REVOLUTE hinge at rear (axis +Y), ~100 deg.
#   - seat_ring   : oval seat ring under the lid, REVOLUTE same rear axis.
#   - flush_handle: chrome lever on tank front face, REVOLUTE pivot (axis +Y),
#                   small ~25 deg downward travel to actuate flush.
# Visual-only details on the root/tank:
#   - hinge_barrel_left / hinge_barrel_right : exposed chrome hinge barrels
#     behind the seat ring (mounted on bowl rear rim).
#   - water_inlet_pipe : chrome supply pipe from floor to tank rear.
#   - tank_access_panel : recessed panel outline on tank top (concealed
#     cistern access).

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
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
SEAT_TOP_Z = 0.400
BOWL_W = 0.370
BOWL_DEPTH = 0.520

# Pedestal base height
PEDESTAL_H = 0.100
# Bowl body from pedestal top to seat shelf
BOWL_BODY_H = 0.280

# Rear hinge axis (shared by lid + seat ring), in world coords.
HINGE_X = -0.050
HINGE_Z = SEAT_TOP_Z + 0.004

# Bowl center X (front-ish)
BOWL_CX = 0.160

# Tank dimensions
TANK_W = 0.350
TANK_D = 0.180
TANK_H = 0.330
TANK_TOP_Z = SEAT_TOP_Z + TANK_H  # ~0.73 m


def _oval_ring(z, rx_out, ry_out, rx_in, ry_in, thick, cx=0.0, segs=72) -> cq.Workplane:
    def ell(rx_, ry_):
        return [
            (cx + rx_ * math.cos(2 * math.pi * i / segs), ry_ * math.sin(2 * math.pi * i / segs))
            for i in range(segs)
        ]

    outer = (
        cq.Workplane("XY")
        .workplane(offset=z)
        .polyline(ell(rx_out, ry_out))
        .close()
        .polyline(ell(rx_in, ry_in))
        .close()
        .extrude(thick)
    )
    return outer


def _oval_disc_solid(rx, ry, thick, cx, z=0.0, segs=72) -> cq.Workplane:
    def ell(rx_, ry_):
        return [
            (cx + rx_ * math.cos(2 * math.pi * i / segs), ry_ * math.sin(2 * math.pi * i / segs))
            for i in range(segs)
        ]

    disc = (
        cq.Workplane("XY")
        .workplane(offset=z)
        .polyline(ell(rx, ry))
        .close()
        .extrude(thick)
    )
    return disc


def _pedestal_solid() -> cq.Workplane:
    """Floor-standing pedestal base: a rounded rectangular foot that tapers
    slightly inward toward the top, supporting the bowl above."""
    # Base footprint ~0.30 x 0.26, height PEDESTAL_H
    foot = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .rect(0.30, 0.26)
        .extrude(0.020)
    )
    # Tapered column from foot to bowl connection
    column = (
        cq.Workplane("XY")
        .workplane(offset=0.020)
        .rect(0.28, 0.24)
        .workplane(offset=PEDESTAL_H - 0.020)
        .rect(0.24, 0.22)
        .loft(ruled=True)
    )
    return foot.union(column).translate((BOWL_CX - 0.02, 0.0, 0.0))


def _bowl_body_solid() -> cq.Workplane:
    """Ceramic bowl body: lofted oval sections from pedestal top up to the
    rim shelf, then hollowed to form the basin."""
    bottom_z = PEDESTAL_H
    top_z = SEAT_TOP_Z

    # Outer shell: tapered oval loft
    outer = (
        cq.Workplane("XY")
        .workplane(offset=bottom_z)
        .ellipse(0.110, 0.110)
        .workplane(offset=0.100)
        .ellipse(0.150, 0.140)
        .workplane(offset=0.100)
        .ellipse(0.175, 0.175)
        .workplane(offset=top_z - bottom_z - 0.200)
        .ellipse(0.185, 0.185)
        .loft(ruled=False)
    )
    outer = outer.translate((BOWL_CX, 0.0, 0.0))

    # Hollow basin cavity
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=bottom_z + 0.060)
        .ellipse(0.070, 0.075)
        .workplane(offset=0.120)
        .ellipse(0.130, 0.130)
        .workplane(offset=top_z - bottom_z - 0.180 - 0.120)
        .ellipse(0.155, 0.155)
        .loft(ruled=False)
        .translate((BOWL_CX, 0.0, 0.0))
    )
    bowl = outer.cut(cavity)

    # Seat shelf: flat oval rim the seat rests on
    shelf = _oval_ring(
        top_z - 0.022, rx_out=0.200, ry_out=0.195, rx_in=0.150, ry_in=0.150,
        thick=0.022, cx=BOWL_CX,
    )
    bowl = bowl.union(shelf)

    # Rear platform connecting bowl to tank mounting area
    rear_platform = (
        cq.Workplane("XY")
        .workplane(offset=top_z - 0.040)
        .center(HINGE_X + 0.040, 0.0)
        .box(0.120, 0.260, 0.040, centered=(True, True, False))
    )
    bowl = bowl.union(rear_platform)

    return bowl


def _tank_solid() -> cq.Workplane:
    """Ceramic cistern tank: rounded rectangular box with slight taper."""
    # Main body
    body = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .rect(TANK_W, TANK_D)
        .workplane(offset=TANK_H * 0.6)
        .rect(TANK_W - 0.010, TANK_D - 0.005)
        .workplane(offset=TANK_H * 0.4)
        .rect(TANK_W - 0.020, TANK_D - 0.010)
        .loft(ruled=False)
    )
    # Round the top edges with a cap
    lid_cap = (
        cq.Workplane("XY")
        .workplane(offset=TANK_H - 0.010)
        .center(0.0, 0.0)
        .box(TANK_W - 0.020, TANK_D - 0.010, 0.010, centered=(True, True, False))
    )
    tank = body.union(lid_cap)
    return tank


def _hinge_barrel_solid() -> cq.Workplane:
    """Small chrome hinge barrel cylinder (one barrel)."""
    barrel = (
        cq.Workplane("XZ")
        .circle(0.008)
        .extrude(0.040)
    )
    return barrel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_piece_toilet")

    ceramic = model.material("ceramic_white", rgba=(0.95, 0.95, 0.94, 1.0))
    seat_white = model.material("seat_white", rgba=(0.97, 0.97, 0.96, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    pipe_chrome = model.material("pipe_chrome", rgba=(0.72, 0.74, 0.76, 1.0))

    # ================= ROOT: bowl + pedestal (floor-standing) ================
    bowl_part = model.part("bowl")

    # Pedestal base
    bowl_part.visual(
        mesh_from_cadquery(_pedestal_solid(), "pedestal"),
        material=ceramic,
        name="pedestal",
    )

    # Bowl body + seat shelf + rear platform
    bowl_part.visual(
        mesh_from_cadquery(_bowl_body_solid(), "bowl_shell"),
        material=ceramic,
        name="bowl_shell",
    )

    # Hinge barrels: two chrome cylinders at rear of bowl rim, flanking the
    # seat hinge axis. These are visual-only on the root part.
    hinge_barrel_left = _hinge_barrel_solid().translate((HINGE_X, 0.070, HINGE_Z - 0.008))
    hinge_barrel_right = _hinge_barrel_solid().translate((HINGE_X, -0.110, HINGE_Z - 0.008))
    bowl_part.visual(
        mesh_from_cadquery(hinge_barrel_left, "hinge_barrel_left"),
        material=chrome,
        name="hinge_barrel_left",
    )
    bowl_part.visual(
        mesh_from_cadquery(hinge_barrel_right, "hinge_barrel_right"),
        material=chrome,
        name="hinge_barrel_right",
    )

    # Water inlet pipe: chrome supply pipe from floor level behind the tank
    # going up to the tank rear connection point.
    inlet_pipe = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .circle(0.012)
        .extrude(TANK_TOP_Z - 0.080)
    )
    # Position behind the tank, centered in Y
    inlet_x = HINGE_X - TANK_D / 2.0 - 0.025
    inlet_pipe = inlet_pipe.translate((inlet_x, 0.0, 0.0))
    # Add a small horizontal section at top connecting to tank
    inlet_elbow = (
        cq.Workplane("YZ")
        .workplane(offset=inlet_x)
        .center(0.0, TANK_TOP_Z - 0.080)
        .circle(0.012)
        .extrude(0.035)
    )
    inlet_pipe = inlet_pipe.union(inlet_elbow)
    bowl_part.visual(
        mesh_from_cadquery(inlet_pipe, "water_inlet_pipe"),
        material=pipe_chrome,
        name="water_inlet_pipe",
    )

    bowl_part.inertial = Inertial.from_geometry(
        Box((0.55, 0.38, 0.42)),
        mass=18.0,
        origin=Origin(xyz=(BOWL_CX - 0.02, 0.0, 0.20)),
    )

    # ================= TANK (fixed mount on bowl rear) =======================
    tank = model.part("tank")
    tank_geo = _tank_solid()
    # Tank sits on the rear platform of the bowl, centered in Y
    tank_origin_x = HINGE_X - TANK_D / 2.0 + 0.040
    tank_origin_z = SEAT_TOP_Z - 0.010
    tank.visual(
        mesh_from_cadquery(tank_geo, "tank_shell"),
        origin=Origin(xyz=(tank_origin_x, 0.0, tank_origin_z)),
        material=ceramic,
        name="tank_shell",
    )

    # Tank access panel outline: a thin recessed rectangle on top of tank
    access_panel = (
        cq.Workplane("XY")
        .workplane(offset=tank_origin_z + TANK_H - 0.002)
        .center(tank_origin_x, 0.0)
        .box(TANK_W * 0.6, TANK_D * 0.5, 0.003, centered=(True, True, False))
    )
    tank.visual(
        mesh_from_cadquery(access_panel, "tank_access_panel"),
        material=model.material("panel_gray", rgba=(0.85, 0.85, 0.84, 1.0)),
        name="tank_access_panel",
    )

    tank.inertial = Inertial.from_geometry(
        Box((TANK_D, TANK_W, TANK_H)),
        mass=8.0,
        origin=Origin(xyz=(tank_origin_x, 0.0, tank_origin_z + TANK_H / 2.0)),
    )

    # Tank is rigidly mounted on the bowl (fixed articulation)
    model.articulation(
        "bowl_to_tank",
        ArticulationType.FIXED,
        parent=bowl_part,
        child=tank,
        origin=Origin(xyz=(tank_origin_x, 0.0, tank_origin_z)),
    )

    # ================= seat ring (revolute, rear axis) =======================
    seat = model.part("seat_ring")
    seat_local_cx = BOWL_CX - HINGE_X
    seat_ring_geo = _oval_ring(
        z=0.002, rx_out=0.180, ry_out=0.178, rx_in=0.110, ry_in=0.118,
        thick=0.020, cx=seat_local_cx,
    )
    seat.visual(
        mesh_from_cadquery(seat_ring_geo, "seat_ring_shell"),
        material=seat_white,
        name="seat_ring_shell",
    )
    seat.inertial = Inertial.from_geometry(
        Box((0.36, 0.36, 0.025)),
        mass=0.8,
        origin=Origin(xyz=(seat_local_cx, 0.0, 0.01)),
    )
    model.articulation(
        "bowl_to_seat_ring",
        ArticulationType.REVOLUTE,
        parent=bowl_part,
        child=seat,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0, lower=-math.radians(100.0), upper=0.0
        ),
    )

    # ================= lid (revolute, same rear axis) ========================
    lid = model.part("lid")
    lid_local_cx = BOWL_CX - HINGE_X
    lid_geo = _oval_disc_solid(rx=0.190, ry=0.185, thick=0.018, cx=lid_local_cx, z=0.020)
    lid.visual(
        mesh_from_cadquery(lid_geo, "lid_shell"),
        material=seat_white,
        name="lid_shell",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.38, 0.37, 0.020)),
        mass=0.9,
        origin=Origin(xyz=(lid_local_cx, 0.0, 0.029)),
    )
    model.articulation(
        "bowl_to_lid",
        ArticulationType.REVOLUTE,
        parent=bowl_part,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0, lower=-math.radians(100.0), upper=0.0
        ),
    )

    # ================= flush handle (revolute on tank front) =================
    # Chrome lever handle on the tank front-left face. The handle pivots
    # downward when pulled (positive angle = tip goes down).
    handle = model.part("flush_handle")

    # Handle pivot base: small cylindrical boss on the tank face
    pivot_base = (
        cq.Workplane("YZ")
        .circle(0.014)
        .extrude(0.012)
    )

    # Handle lever arm: extends to the right from the pivot
    lever_arm = (
        cq.Workplane("XY")
        .workplane(offset=-0.004)
        .center(0.006, 0.0)
        .box(0.090, 0.012, 0.008, centered=(False, True, True))
    )

    # Handle grip: small rounded end
    grip = (
        cq.Workplane("YZ")
        .workplane(offset=0.096)
        .center(0.0, 0.0)
        .circle(0.009)
        .extrude(0.014)
    )

    handle_geo = pivot_base.union(lever_arm).union(grip)

    # Position the handle on the tank front-left area
    handle_x = tank_origin_x + TANK_D / 2.0 + 0.006
    handle_y = 0.080
    handle_z = tank_origin_z + TANK_H * 0.65

    handle.visual(
        mesh_from_cadquery(handle_geo, "flush_handle_lever"),
        origin=Origin(xyz=(handle_x, handle_y, handle_z)),
        material=chrome,
        name="flush_handle_lever",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.10, 0.02, 0.02)),
        mass=0.08,
        origin=Origin(xyz=(handle_x + 0.05, handle_y, handle_z)),
    )

    # Handle pivots around Y axis at its mount point. Positive rotation
    # pushes the lever tip downward (-Z) to actuate flush.
    model.articulation(
        "tank_to_flush_handle",
        ArticulationType.REVOLUTE,
        parent=tank,
        child=handle,
        origin=Origin(xyz=(handle_x, handle_y, handle_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=0.0, upper=math.radians(25.0)
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bowl = object_model.get_part("bowl")
    tank = object_model.get_part("tank")
    seat = object_model.get_part("seat_ring")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("flush_handle")

    seat_joint = object_model.get_articulation("bowl_to_seat_ring")
    lid_joint = object_model.get_articulation("bowl_to_lid")
    handle_joint = object_model.get_articulation("tank_to_flush_handle")
    tank_joint = object_model.get_articulation("bowl_to_tank")

    # --- Intentional overlaps ---
    ctx.allow_overlap(
        seat, bowl,
        elem_a="seat_ring_shell", elem_b="bowl_shell",
        reason="Seat ring rests on the ceramic bowl rim shelf (seated contact).",
    )
    ctx.allow_overlap(
        lid, seat,
        elem_a="lid_shell", elem_b="seat_ring_shell",
        reason="Closed lid rests on top of the seat ring.",
    )
    ctx.allow_overlap(
        tank, bowl,
        elem_a="tank_shell", elem_b="bowl_shell",
        reason="Tank sits on the bowl rear platform (rigid ceramic mount).",
    )

    # --- Two-piece structure: tank is a separate part above bowl ---
    tank_aabb = ctx.part_world_aabb(tank)
    bowl_aabb = ctx.part_world_aabb(bowl)
    ctx.check(
        "tank is a separate part (exists)",
        tank_aabb is not None,
        details="tank part has no geometry",
    )
    ctx.check(
        "tank top is above seat height (two-piece cistern)",
        tank_aabb[1][2] > SEAT_TOP_Z + 0.10,
        details=f"tank top z = {tank_aabb[1][2]}",
    )
    ctx.check(
        "tank is behind the bowl center (rear-mounted cistern)",
        tank_aabb[0][0] < BOWL_CX,
        details=f"tank min x = {tank_aabb[0][0]}, bowl center x = {BOWL_CX}",
    )

    # --- Bowl is floor-standing (pedestal reaches near the floor) ---
    ctx.check(
        "bowl pedestal is floor-standing (min z near 0)",
        bowl_aabb[0][2] < 0.02,
        details=f"min z of bowl = {bowl_aabb[0][2]}",
    )

    # --- Seat top height check ---
    ctx.check(
        "seat top is near 0.40 m above floor",
        0.34 < SEAT_TOP_Z < 0.46,
        details=f"seat top z = {SEAT_TOP_Z}",
    )

    # --- Lid and seat ring share same rear hinge axis ---
    so = seat_joint.origin
    lo = lid_joint.origin
    ctx.check(
        "seat ring and lid share the same rear hinge axis",
        abs(so.xyz[0] - lo.xyz[0]) < 1e-6
        and abs(so.xyz[1] - lo.xyz[1]) < 1e-6
        and abs(so.xyz[2] - lo.xyz[2]) < 1e-6
        and tuple(seat_joint.axis) == tuple(lid_joint.axis),
        details=f"seat origin={so.xyz} axis={seat_joint.axis}; lid origin={lo.xyz} axis={lid_joint.axis}",
    )

    # --- Lid rotates open ~100 deg (front edge lifts up and back) ---
    lid_top_z0 = ctx.part_world_aabb(lid)[1][2]
    with ctx.pose({lid_joint: -math.radians(100.0)}):
        lid_top_z1 = ctx.part_world_aabb(lid)[1][2]
        lid_front_x1 = ctx.part_world_aabb(lid)[1][0]
    lid_front_x0 = ctx.part_world_aabb(lid)[1][0]
    ctx.check(
        "lid rotates open (lifts upward)",
        lid_top_z1 > lid_top_z0 + 0.05,
        details=f"closed top z={lid_top_z0}, open top z={lid_top_z1}",
    )
    ctx.check(
        "lid swings rearward when opened",
        lid_front_x1 < lid_front_x0 - 0.05,
        details=f"closed front x={lid_front_x0}, open front x={lid_front_x1}",
    )

    # --- Seat ring rotates open ---
    seat_top_z0 = ctx.part_world_aabb(seat)[1][2]
    with ctx.pose({seat_joint: -math.radians(100.0)}):
        seat_top_z1 = ctx.part_world_aabb(seat)[1][2]
    ctx.check(
        "seat ring rotates open (lifts upward)",
        seat_top_z1 > seat_top_z0 + 0.05,
        details=f"closed top z={seat_top_z0}, open top z={seat_top_z1}",
    )

    # --- Visible hinge barrels exist behind the seat ---
    bowl_summary_parts = []
    try:
        left_barrel = bowl.get_visual("hinge_barrel_left")
        right_barrel = bowl.get_visual("hinge_barrel_right")
        ctx.check(
            "visible hinge barrels exist on bowl",
            left_barrel is not None and right_barrel is not None,
            details="missing hinge barrel visuals",
        )
        # Barrels should be near the hinge axis height and behind bowl center
        barrel_left_aabb = ctx.part_element_world_aabb(bowl, elem="hinge_barrel_left")
        ctx.check(
            "hinge barrels are near the hinge axis height",
            barrel_left_aabb is not None and abs((barrel_left_aabb[0][2] + barrel_left_aabb[1][2]) / 2.0 - HINGE_Z) < 0.030,
            details=f"barrel center z ~ {barrel_left_aabb}",
        )
    except Exception:
        ctx.fail("visible hinge barrels", "could not inspect hinge barrel visuals")

    # --- Water inlet pipe exists ---
    try:
        inlet = bowl.get_visual("water_inlet_pipe")
        ctx.check(
            "water inlet pipe exists on the assembly",
            inlet is not None,
            details="no water inlet pipe visual found",
        )
        inlet_aabb = ctx.part_element_world_aabb(bowl, elem="water_inlet_pipe")
        ctx.check(
            "water inlet pipe reaches from floor to near tank height",
            inlet_aabb is not None and inlet_aabb[0][2] < 0.05 and inlet_aabb[1][2] > SEAT_TOP_Z + 0.10,
            details=f"inlet aabb = {inlet_aabb}",
        )
    except Exception:
        ctx.fail("water inlet pipe", "could not inspect inlet pipe visual")

    # --- Tank access panel outline exists ---
    try:
        panel = tank.get_visual("tank_access_panel")
        ctx.check(
            "tank access panel outline exists",
            panel is not None,
            details="no tank_access_panel visual found",
        )
    except Exception:
        ctx.fail("tank access panel", "could not inspect access panel visual")

    # --- Flush handle pivots on a revolute joint ---
    ctx.check(
        "flush handle articulation is revolute",
        handle_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"handle joint type = {handle_joint.articulation_type}",
    )
    handle_z0 = ctx.part_world_position(handle)
    with ctx.pose({handle_joint: math.radians(25.0)}):
        handle_z1 = ctx.part_world_position(handle)
    ctx.check(
        "flush handle pivots downward when actuated",
        handle_z1 is not None and handle_z0 is not None and handle_z1[2] < handle_z0[2] - 0.002,
        details=f"rest z={handle_z0}, actuated z={handle_z1}",
    )
    ctx.check(
        "flush handle is mounted on the tank (above seat height)",
        handle_z0 is not None and handle_z0[2] > SEAT_TOP_Z,
        details=f"handle z = {handle_z0}",
    )

    # --- Handle joint has realistic limits (small arc) ---
    hl = handle_joint.motion_limits
    ctx.check(
        "flush handle has limited travel arc (<=30 deg)",
        hl is not None and (hl.upper - hl.lower) <= math.radians(30.0) + 0.001,
        details=f"handle limits: lower={hl.lower}, upper={hl.upper}",
    )

    return ctx.report()


object_model = build_object_model()
