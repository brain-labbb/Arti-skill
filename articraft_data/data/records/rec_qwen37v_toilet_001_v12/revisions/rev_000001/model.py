from __future__ import annotations

# One-piece floor-standing white ceramic toilet with integrated rounded tank.
#
# Frame / coordinate convention:
#   +X = forward (front of the bowl points to +X).
#   +Y = left-right.
#   +Z = up. The floor is at z=0; the seat top sits ~0.40 m above the floor.
#
# Root part = the one-piece ceramic body (bowl + pedestal + integrated tank).
# Everything else mounts to that root:
#   - lid          : oval top lid, REVOLUTE hinge at the rear (axis +Y), ~100 deg.
#   - seat_ring    : oval seat ring under the lid, REVOLUTE hinge sharing the
#                    SAME rear axis as the lid (concentric), ~100 deg.
#   - flush_handle : chrome lever on the right side of the tank, REVOLUTE
#                    pivot axis along Y, pushes down ~25 deg to flush.
# Floor bolt caps are fixed visuals on the body at the base.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
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
BOWL_W = 0.360
BOWL_DEPTH = 0.520
BOWL_CX = 0.140  # bowl center X
TANK_CX = -0.060  # tank center X (behind bowl)
TANK_TOP_Z = 0.720
TANK_W = 0.360
TANK_DEPTH = 0.180

# Rear hinge axis (shared by lid + seat ring), in world coords.
HINGE_X = -0.010
HINGE_Z = SEAT_TOP_Z + 0.004


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


def _oval_disc_solid(rx, ry, thick, cx, segs=72) -> cq.Workplane:
    def ell(rx_, ry_):
        return [
            (cx + rx_ * math.cos(2 * math.pi * i / segs), ry_ * math.sin(2 * math.pi * i / segs))
            for i in range(segs)
        ]

    disc = (
        cq.Workplane("XY")
        .polyline(ell(rx, ry))
        .close()
        .extrude(thick)
    )
    return disc


def _oval_disc_ring_solid(rx_out, ry_out, rx_in, ry_in, thick, cx, segs=80) -> cq.Workplane:
    def ell(rx_, ry_):
        return [
            (cx + rx_ * math.cos(2 * math.pi * i / segs), ry_ * math.sin(2 * math.pi * i / segs))
            for i in range(segs)
        ]

    ring = (
        cq.Workplane("XY")
        .polyline(ell(rx_out, ry_out))
        .close()
        .polyline(ell(rx_in, ry_in))
        .close()
        .extrude(thick)
    )
    return ring


def _pedestal_solid() -> cq.Workplane:
    """Tapered pedestal from floor to bowl shelf level."""
    # Base is wider, tapers slightly upward.
    # Bottom at z=0, top at z = SEAT_TOP_Z - 0.02 (just under the shelf)
    bot_z = 0.0
    top_z = SEAT_TOP_Z - 0.020

    def rect_pts(hw, hd, segs=4):
        """Rounded rectangle approximation with 4 sides."""
        # Use an ellipse for smoother pedestal shape
        pts = []
        n = 48
        for i in range(n):
            a = 2.0 * math.pi * i / n
            pts.append((hw * math.cos(a), hd * math.sin(a)))
        return pts

    ped = (
        cq.Workplane("XY")
        .workplane(offset=bot_z)
        .polyline(rect_pts(0.140, 0.120))
        .close()
        .workplane(offset=top_z - bot_z)
        .polyline(rect_pts(0.160, 0.145))
        .close()
        .loft(ruled=False)
    )
    # Center pedestal under the bowl
    ped = ped.translate((BOWL_CX - 0.02, 0.0, 0.0))
    return ped


def _bowl_solid() -> cq.Workplane:
    """The ceramic bowl: oval body with hollowed basin, sitting on the pedestal."""
    cx = BOWL_CX
    top_z = SEAT_TOP_Z
    bottom_z = SEAT_TOP_Z - 0.280

    # Outer body: lofted oval from narrower bottom to wider rim
    outer = (
        cq.Workplane("XY")
        .workplane(offset=bottom_z)
        .ellipse(0.080, 0.085)
        .workplane(offset=0.100)
        .ellipse(0.140, 0.130)
        .workplane(offset=0.100)
        .ellipse(0.170, 0.170)
        .workplane(offset=0.080)
        .ellipse(0.180, 0.180)
        .loft(ruled=False)
    )
    outer = outer.translate((cx, 0.0, 0.0))

    # Hollow basin cavity cut from top
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=top_z - 0.180)
        .ellipse(0.090, 0.095)
        .workplane(offset=0.120)
        .ellipse(0.148, 0.148)
        .workplane(offset=0.065)
        .ellipse(0.155, 0.155)
        .loft(ruled=False)
        .translate((cx, 0.0, 0.0))
    )
    bowl = outer.cut(cavity)

    # Seat shelf: flat oval rim at top
    shelf = _oval_ring(
        top_z - 0.022, rx_out=0.195, ry_out=0.190, rx_in=0.148, ry_in=0.148,
        thick=0.022, cx=cx,
    )
    bowl = bowl.union(shelf)
    return bowl


def _tank_solid() -> cq.Workplane:
    """Rounded integrated tank behind the bowl."""
    cx = TANK_CX
    bot_z = 0.220  # tank starts above the pedestal midpoint
    top_z = TANK_TOP_Z

    # Tank body: rounded rectangular shape via loft with oval cross-sections
    # Bottom section (narrower), mid section (widest), top section (slightly tapered)
    tank = (
        cq.Workplane("XY")
        .workplane(offset=bot_z)
        .ellipse(TANK_DEPTH * 0.42, TANK_W * 0.42)
        .workplane(offset=0.120)
        .ellipse(TANK_DEPTH * 0.50, TANK_W * 0.50)
        .workplane(offset=(top_z - bot_z) - 0.180)
        .ellipse(TANK_DEPTH * 0.48, TANK_W * 0.50)
        .workplane(offset=0.060)
        .ellipse(TANK_DEPTH * 0.40, TANK_W * 0.46)
        .loft(ruled=False)
    )
    tank = tank.translate((cx, 0.0, 0.0))

    # Tank lid cap: a slightly wider flat disc on top
    cap = (
        cq.Workplane("XY")
        .workplane(offset=top_z - 0.005)
        .center(cx, 0.0)
        .ellipse(TANK_DEPTH * 0.50, TANK_W * 0.51)
        .extrude(0.018)
    )
    tank = tank.union(cap)
    return tank


def _body_connector_solid() -> cq.Workplane:
    """Smooth connector between bowl back and tank front so they read as one piece."""
    # A box-like bridge filling the gap between bowl rear and tank front
    bridge_x = (BOWL_CX + TANK_CX) / 2.0
    bridge = (
        cq.Workplane("XY")
        .workplane(offset=SEAT_TOP_Z - 0.200)
        .center(bridge_x, 0.0)
        .box(BOWL_CX - TANK_CX + 0.06, 0.260, 0.240, centered=(True, True, False))
    )
    return bridge


def _one_piece_body() -> cq.Workplane:
    """Full one-piece ceramic body: pedestal + bowl + tank + connector."""
    body = _pedestal_solid()
    body = body.union(_bowl_solid())
    body = body.union(_tank_solid())
    body = body.union(_body_connector_solid())
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="one_piece_toilet")

    ceramic = model.material("ceramic_white", rgba=(0.95, 0.95, 0.94, 1.0))
    seat_white = model.material("seat_white", rgba=(0.97, 0.97, 0.96, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    cap_white = model.material("cap_white", rgba=(0.92, 0.92, 0.91, 1.0))

    # ================= ROOT: one-piece ceramic body =================
    body = model.part("body")

    body.visual(
        mesh_from_cadquery(_one_piece_body(), "body_shell"),
        material=ceramic,
        name="body_shell",
    )

    # Floor bolt caps: two small dome caps at the base front
    for side, y_pos in [("bolt_cap_0", -0.095), ("bolt_cap_1", 0.095)]:
        cap_geo = (
            cq.Workplane("XY")
            .workplane(offset=0.0)
            .center(BOWL_CX - 0.02, y_pos)
            .circle(0.016)
            .extrude(0.012)
        )
        # Add a dome on top
        dome = (
            cq.Workplane("XY")
            .workplane(offset=0.012)
            .center(BOWL_CX - 0.02, y_pos)
            .sphere(0.016)
        )
        # Cut the bottom half of the sphere
        cutter = (
            cq.Workplane("XY")
            .workplane(offset=-0.010)
            .center(BOWL_CX - 0.02, y_pos)
            .box(0.050, 0.050, 0.024, centered=(True, True, False))
        )
        cap_geo = cap_geo.union(dome.cut(cutter))
        body.visual(
            mesh_from_cadquery(cap_geo, side),
            material=cap_white,
            name=side,
        )

    body.inertial = Inertial.from_geometry(
        Box((0.56, 0.38, 0.72)),
        mass=35.0,
        origin=Origin(xyz=(0.05, 0.0, 0.36)),
    )

    # ================= seat ring (revolute, rear axis) =================
    seat = model.part("seat_ring")
    seat_local_cx = BOWL_CX - HINGE_X
    seat_ring_geo = _oval_disc_ring_solid(
        rx_out=0.180,
        ry_out=0.178,
        rx_in=0.110,
        ry_in=0.118,
        thick=0.020,
        cx=seat_local_cx,
    )
    seat.visual(
        mesh_from_cadquery(seat_ring_geo.translate((0, 0, 0.002)), "seat_ring_shell"),
        material=seat_white,
        name="seat_ring_shell",
    )
    seat.inertial = Inertial.from_geometry(
        Box((0.36, 0.36, 0.025)),
        mass=0.8,
        origin=Origin(xyz=(seat_local_cx, 0.0, 0.01)),
    )
    model.articulation(
        "body_to_seat_ring",
        ArticulationType.REVOLUTE,
        parent=body,
        child=seat,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0, lower=-math.radians(100.0), upper=0.0
        ),
    )

    # ================= lid (revolute, SAME rear axis) =================
    lid = model.part("lid")
    lid_local_cx = BOWL_CX - HINGE_X
    lid_geo = _oval_disc_solid(rx=0.190, ry=0.185, thick=0.018, cx=lid_local_cx)
    lid.visual(
        mesh_from_cadquery(lid_geo.translate((0, 0, 0.020)), "lid_shell"),
        material=seat_white,
        name="lid_shell",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.38, 0.37, 0.020)),
        mass=0.9,
        origin=Origin(xyz=(lid_local_cx, 0.0, 0.029)),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0, lower=-math.radians(100.0), upper=0.0
        ),
    )

    # ================= flush handle (revolute on tank side) =================
    # Chrome lever on the right side (+Y) of the tank front face.
    # Pivot axis along Y. Handle extends outward along +X from the pivot.
    # Pushing the handle down (negative rotation about Y axis) actuates flush.
    handle = model.part("flush_handle")

    # Handle geometry: a small chrome lever arm
    # Pivot is at the tank surface; handle extends ~0.06m outward
    handle_pivot_x = TANK_CX + TANK_DEPTH * 0.45  # near tank front face
    handle_pivot_y = TANK_W * 0.48  # right side of tank
    handle_pivot_z = TANK_TOP_Z - 0.080  # near top of tank

    # Lever: a rounded bar extending in +X direction from pivot
    lever = (
        cq.Workplane("XY")
        .workplane(offset=-0.006)
        .center(0.030, 0.0)
        .box(0.065, 0.018, 0.012, centered=(True, True, True))
    )
    # Small cylindrical pivot boss at the origin
    pivot_boss = (
        cq.Workplane("XY")
        .workplane(offset=-0.008)
        .circle(0.010)
        .extrude(0.016)
    )
    handle_geo = lever.union(pivot_boss)
    # Round the end of the lever
    lever_tip = (
        cq.Workplane("XY")
        .workplane(offset=-0.006)
        .center(0.062, 0.0)
        .sphere(0.009)
    )
    tip_cutter = (
        cq.Workplane("XY")
        .workplane(offset=-0.020)
        .center(0.062, 0.0)
        .box(0.030, 0.030, 0.020, centered=(True, True, False))
    )
    handle_geo = handle_geo.union(lever_tip.cut(tip_cutter))

    handle.visual(
        mesh_from_cadquery(handle_geo, "flush_handle_lever"),
        material=chrome,
        name="flush_handle_lever",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.07, 0.02, 0.015)),
        mass=0.08,
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_flush_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(handle_pivot_x, handle_pivot_y, handle_pivot_z)),
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

    body = object_model.get_part("body")
    seat = object_model.get_part("seat_ring")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("flush_handle")

    seat_joint = object_model.get_articulation("body_to_seat_ring")
    lid_joint = object_model.get_articulation("body_to_lid")
    handle_joint = object_model.get_articulation("body_to_flush_handle")

    # --- Intentional overlaps ---
    ctx.allow_overlap(
        seat, body,
        elem_a="seat_ring_shell", elem_b="body_shell",
        reason="Seat ring rests on the ceramic bowl rim shelf (seated contact).",
    )
    ctx.allow_overlap(
        lid, seat,
        elem_a="lid_shell", elem_b="seat_ring_shell",
        reason="Closed lid rests on top of the seat ring.",
    )
    ctx.allow_overlap(
        handle, body,
        elem_a="flush_handle_lever", elem_b="body_shell",
        reason="Flush handle pivot boss is seated into the tank surface.",
    )

    # --- One-piece toilet is floor-standing (body reaches the floor). ---
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "toilet body reaches the floor (floor-standing)",
        body_aabb[0][2] < 0.01,
        details=f"min z of body = {body_aabb[0][2]}",
    )

    # --- Integrated tank extends above the seat. ---
    body_top_z = body_aabb[1][2]
    ctx.check(
        "integrated tank extends above the seat level",
        body_top_z > SEAT_TOP_Z + 0.20,
        details=f"body top z = {body_top_z}, seat z = {SEAT_TOP_Z}",
    )

    # --- Seat top near 0.40 m ---
    ctx.check(
        "seat top is near 0.40 m above floor",
        0.34 < SEAT_TOP_Z < 0.46,
        details=f"seat top z = {SEAT_TOP_Z}",
    )

    # --- Lid sits above the seat ring when closed. ---
    seat_z = ctx.part_world_aabb(seat)[1][2]
    lid_z = ctx.part_world_aabb(lid)[0][2]
    ctx.check(
        "lid sits above the seat ring when closed",
        lid_z >= seat_z - 0.005,
        details=f"lid bottom z={lid_z}, seat top z={seat_z}",
    )

    # --- Lid rotates open ~100 deg (front edge lifts up). ---
    lid_top_z0 = ctx.part_world_aabb(lid)[1][2]
    with ctx.pose({lid_joint: -math.radians(100.0)}):
        lid_top_z1 = ctx.part_world_aabb(lid)[1][2]
    ctx.check(
        "lid rotates open (lifts upward)",
        lid_top_z1 > lid_top_z0 + 0.05,
        details=f"closed top z={lid_top_z0}, open top z={lid_top_z1}",
    )

    # --- Seat ring rotates open. ---
    seat_top_z0 = ctx.part_world_aabb(seat)[1][2]
    with ctx.pose({seat_joint: -math.radians(100.0)}):
        seat_top_z1 = ctx.part_world_aabb(seat)[1][2]
    ctx.check(
        "seat ring rotates open (lifts upward)",
        seat_top_z1 > seat_top_z0 + 0.05,
        details=f"closed top z={seat_top_z0}, open top z={seat_top_z1}",
    )

    # --- Seat ring and lid share the same rear hinge axis. ---
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

    # --- Flush handle is a revolute joint that pivots. ---
    ctx.check(
        "flush handle joint is revolute",
        handle_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"handle joint type = {handle_joint.articulation_type}",
    )

    # --- Flush handle pivots downward when actuated. ---
    handle_z0 = ctx.part_world_aabb(handle)[1][2]  # top of handle at rest
    handle_x0 = ctx.part_world_position(handle)[0]
    with ctx.pose({handle_joint: math.radians(25.0)}):
        handle_z1 = ctx.part_world_aabb(handle)[0][2]  # bottom of handle when pushed
        handle_x1 = ctx.part_world_position(handle)[0]
    ctx.check(
        "flush handle pivots (position changes with actuation)",
        abs(handle_z1 - handle_z0) > 0.005 or abs(handle_x1 - handle_x0) > 0.005,
        details=f"rest: z_top={handle_z0}, x={handle_x0}; actuated: z_bot={handle_z1}, x={handle_x1}",
    )

    # --- Flush handle is mounted on the tank (above seat level, behind bowl). ---
    handle_pos = ctx.part_world_position(handle)
    ctx.check(
        "flush handle is mounted high on the tank",
        handle_pos[2] > SEAT_TOP_Z + 0.10,
        details=f"handle z = {handle_pos[2]}",
    )

    # --- Floor bolt caps exist at the base. ---
    bolt_cap_0 = body.get_visual("bolt_cap_0")
    bolt_cap_1 = body.get_visual("bolt_cap_1")
    ctx.check(
        "floor bolt cap 0 exists on body",
        bolt_cap_0 is not None,
        details="bolt_cap_0 visual not found on body",
    )
    ctx.check(
        "floor bolt cap 1 exists on body",
        bolt_cap_1 is not None,
        details="bolt_cap_1 visual not found on body",
    )

    return ctx.report()


object_model = build_object_model()
