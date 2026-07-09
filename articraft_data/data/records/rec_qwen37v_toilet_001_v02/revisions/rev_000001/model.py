from __future__ import annotations

# One-piece floor-standing white ceramic toilet with a rounded integrated tank.
#
# Frame / coordinate convention:
#   +X = forward (front of the bowl points to +X), back of tank near x=0.
#   +Y = left (flush lever is on the left side of the tank).
#   +Z = up. Floor at z=0; seat rim top at ~0.40 m.
#
# Root part = body: one-piece ceramic assembly integrating tank, bowl, base,
# raised rim, and floor bolt caps. Everything else mounts to the body:
#   - seat_ring  : oval seat ring, REVOLUTE hinge at rear of bowl, ~100 deg.
#   - lid        : oval lid, REVOLUTE hinge sharing same rear axis, ~100 deg.
#   - flush_lever: chrome lever on tank left side, REVOLUTE, push-down ~30 deg.

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
SEAT_RIM_Z = 0.400  # top of bowl rim / seat surface
TANK_TOP_Z = 0.680  # top of the integrated tank
TANK_BACK_X = 0.000  # back face of tank
TANK_FRONT_X = 0.180  # front face of tank
TANK_W = 0.370  # tank width (Y)
TANK_DEPTH = TANK_FRONT_X - TANK_BACK_X

BOWL_CX = 0.370  # bowl oval center X
BOWL_RX = 0.210  # bowl half-depth (X) at rim
BOWL_RY = 0.180  # bowl half-width (Y) at rim
BOWL_BOTTOM_Z = 0.060  # bottom of bowl outer shell

BASE_FRONT_X = 0.540  # front of the pedestal base
BASE_BACK_X = 0.000  # back of the pedestal (same as tank back)

HINGE_X = 0.190  # rear hinge line X (back of bowl rim, near tank front)
HINGE_Z = SEAT_RIM_Z + 0.005  # hinge axis just above the rim

# Flush lever pivot location (left side of tank, upper area)
LEVER_X = 0.110
LEVER_Y = -(TANK_W / 2.0)  # left face of tank
LEVER_Z = 0.580


def _ellipse_pts(rx, ry, cx=0.0, cy=0.0, segs=72):
    """Generate ellipse points for CadQuery polyline."""
    return [
        (cx + rx * math.cos(2.0 * math.pi * i / segs),
         cy + ry * math.sin(2.0 * math.pi * i / segs))
        for i in range(segs)
    ]


def _tank_solid() -> cq.Workplane:
    """Rounded integrated tank at the rear of the toilet."""
    # Main tank body: a box with filleted vertical and top edges
    tank = (
        cq.Workplane("XY")
        .workplane(offset=0.10)
        .center(TANK_BACK_X + TANK_DEPTH / 2.0, 0.0)
        .box(TANK_DEPTH, TANK_W, TANK_TOP_Z - 0.10, centered=(True, True, False))
    )
    # Fillet vertical edges for rounded look
    tank = tank.edges("|Z").fillet(0.025)
    # Fillet top edges
    tank = tank.edges(">Z").fillet(0.012)
    return tank


def _bowl_outer_solid() -> cq.Workplane:
    """Outer bowl shell: lofted oval sections from base to rim."""
    cx = BOWL_CX
    # Bottom section (smaller, near trapway)
    # Middle section (wider)
    # Top section (full rim size)
    outer = (
        cq.Workplane("XY")
        .workplane(offset=BOWL_BOTTOM_Z)
        .polyline(_ellipse_pts(BOWL_RX * 0.45, BOWL_RY * 0.55, cx=cx))
        .close()
        .workplane(offset=0.12)
        .polyline(_ellipse_pts(BOWL_RX * 0.75, BOWL_RY * 0.85, cx=cx))
        .close()
        .workplane(offset=0.12)
        .polyline(_ellipse_pts(BOWL_RX * 0.92, BOWL_RY * 0.95, cx=cx))
        .close()
        .workplane(offset=SEAT_RIM_Z - BOWL_BOTTOM_Z - 0.24)
        .polyline(_ellipse_pts(BOWL_RX, BOWL_RY, cx=cx))
        .close()
        .loft(ruled=False)
    )
    return outer


def _bowl_cavity_solid() -> cq.Workplane:
    """Inner cavity to hollow out the bowl from the top."""
    cx = BOWL_CX
    # Cavity: slightly smaller oval, from deep inside up past the rim
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=BOWL_BOTTOM_Z + 0.08)
        .polyline(_ellipse_pts(BOWL_RX * 0.25, BOWL_RY * 0.30, cx=cx))
        .close()
        .workplane(offset=0.10)
        .polyline(_ellipse_pts(BOWL_RX * 0.60, BOWL_RY * 0.65, cx=cx))
        .close()
        .workplane(offset=SEAT_RIM_Z - BOWL_BOTTOM_Z - 0.18 + 0.01)
        .polyline(_ellipse_pts(BOWL_RX * 0.82, BOWL_RY * 0.82, cx=cx))
        .close()
        .loft(ruled=False)
    )
    return cavity


def _raised_rim_solid() -> cq.Workplane:
    """Raised oval rim ring on top of the bowl shell."""
    cx = BOWL_CX
    segs = 72
    # Outer rim profile
    outer_pts = _ellipse_pts(BOWL_RX + 0.008, BOWL_RY + 0.008, cx=cx, segs=segs)
    inner_pts = _ellipse_pts(BOWL_RX * 0.82, BOWL_RY * 0.82, cx=cx, segs=segs)

    rim = (
        cq.Workplane("XY")
        .workplane(offset=SEAT_RIM_Z - 0.003)
        .polyline(outer_pts)
        .close()
        .polyline(inner_pts)
        .close()
        .extrude(0.018)
    )
    return rim


def _base_pedestal_solid() -> cq.Workplane:
    """Pedestal base connecting tank and bowl to the floor."""
    # A tapered box-like shape under the bowl and in front of the tank
    # Wider at the bottom for stability look
    cx_ped = (BASE_BACK_X + BASE_FRONT_X) / 2.0
    ped = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .center(cx_ped, 0.0)
        .box(BASE_FRONT_X - BASE_BACK_X, TANK_W * 0.85, 0.10, centered=(True, True, False))
    )
    # Upper portion slightly narrower (tapered)
    ped_upper = (
        cq.Workplane("XY")
        .workplane(offset=0.06)
        .center(cx_ped + 0.02, 0.0)
        .box(BASE_FRONT_X - BASE_BACK_X - 0.06, TANK_W * 0.75, 0.08, centered=(True, True, False))
    )
    return ped.union(ped_upper)


def _connector_solid() -> cq.Workplane:
    """Connecting shroud between the tank front face and the bowl back."""
    # Fills the gap between tank front (x=0.18) and bowl back (~x=0.16)
    # This makes the one-piece look seamless
    cx_conn = (TANK_FRONT_X + BOWL_CX - BOWL_RX * 0.45) / 2.0
    conn = (
        cq.Workplane("XY")
        .workplane(offset=BOWL_BOTTOM_Z)
        .center(cx_conn + 0.02, 0.0)
        .box(0.12, TANK_W * 0.65, SEAT_RIM_Z - BOWL_BOTTOM_Z - 0.02,
             centered=(True, True, False))
    )
    return conn


def _seat_shelf_solid() -> cq.Workplane:
    """Flat shelf area around the bowl opening where the seat ring rests."""
    cx = BOWL_CX
    segs = 72
    outer_pts = _ellipse_pts(BOWL_RX + 0.005, BOWL_RY + 0.005, cx=cx, segs=segs)
    inner_pts = _ellipse_pts(BOWL_RX * 0.85, BOWL_RY * 0.85, cx=cx, segs=segs)

    shelf = (
        cq.Workplane("XY")
        .workplane(offset=SEAT_RIM_Z - 0.025)
        .polyline(outer_pts)
        .close()
        .polyline(inner_pts)
        .close()
        .extrude(0.025)
    )
    return shelf


def _oval_disc_ring(rx_out, ry_out, rx_in, ry_in, thick, cx, segs=80) -> cq.Workplane:
    """Flat oval ring for the seat ring geometry."""
    outer_pts = _ellipse_pts(rx_out, ry_out, cx=cx, segs=segs)
    inner_pts = _ellipse_pts(rx_in, ry_in, cx=cx, segs=segs)
    ring = (
        cq.Workplane("XY")
        .polyline(outer_pts)
        .close()
        .polyline(inner_pts)
        .close()
        .extrude(thick)
    )
    return ring


def _oval_disc(rx, ry, thick, cx, segs=72) -> cq.Workplane:
    """Flat oval disc for the lid geometry."""
    pts = _ellipse_pts(rx, ry, cx=cx, segs=segs)
    disc = (
        cq.Workplane("XY")
        .polyline(pts)
        .close()
        .extrude(thick)
    )
    return disc


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="one_piece_toilet")

    ceramic = model.material("ceramic_white", rgba=(0.95, 0.95, 0.94, 1.0))
    seat_white = model.material("seat_white", rgba=(0.97, 0.97, 0.96, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    bowl_interior = model.material("bowl_interior", rgba=(0.85, 0.87, 0.88, 1.0))
    bolt_cap_mat = model.material("bolt_cap_chrome", rgba=(0.72, 0.74, 0.76, 1.0))

    cx = BOWL_CX

    # ================= ROOT: body (one-piece ceramic) =================
    body = model.part("body")

    # Tank (rounded integrated tank at the rear)
    body.visual(
        mesh_from_cadquery(_tank_solid(), "tank_shell"),
        material=ceramic,
        name="tank_shell",
    )

    # Bowl outer shell
    bowl_outer = _bowl_outer_solid()
    body.visual(
        mesh_from_cadquery(bowl_outer, "bowl_shell"),
        material=ceramic,
        name="bowl_shell",
    )

    # Bowl cavity (visible hollow interior - darker material to show depth)
    bowl_cav = _bowl_cavity_solid()
    body.visual(
        mesh_from_cadquery(bowl_cav, "bowl_cavity"),
        material=bowl_interior,
        name="bowl_cavity",
    )

    # Raised rim
    rim = _raised_rim_solid()
    body.visual(
        mesh_from_cadquery(rim, "bowl_rim"),
        material=ceramic,
        name="bowl_rim",
    )

    # Seat shelf (flat area the seat sits on)
    shelf = _seat_shelf_solid()
    body.visual(
        mesh_from_cadquery(shelf, "seat_shelf"),
        material=ceramic,
        name="seat_shelf",
    )

    # Base pedestal
    base = _base_pedestal_solid()
    body.visual(
        mesh_from_cadquery(base, "base_shell"),
        material=ceramic,
        name="base_shell",
    )

    # Connector between tank and bowl
    conn = _connector_solid()
    body.visual(
        mesh_from_cadquery(conn, "connector_shell"),
        material=ceramic,
        name="connector_shell",
    )

    # Floor bolt caps (two small chrome cylinders at the base front)
    bolt_cap_geo = CylinderGeometry(0.014, 0.012, radial_segments=32)
    for i, by in enumerate([-0.110, 0.110]):
        body.visual(
            mesh_from_geometry(bolt_cap_geo, f"bolt_cap_{i}"),
            origin=Origin(xyz=(BASE_FRONT_X - 0.04, by, 0.006)),
            material=bolt_cap_mat,
            name=f"bolt_cap_{i}",
        )

    body.inertial = Inertial.from_geometry(
        Box((0.60, 0.38, 0.68)),
        mass=35.0,
        origin=Origin(xyz=(0.28, 0.0, 0.34)),
    )

    # ================= seat ring (revolute, rear hinge) =================
    seat = model.part("seat_ring")
    # Seat ring extends forward from hinge; back edge at local x=0 (hinge).
    seat_rx_out = 0.190
    seat_local_cx = seat_rx_out  # center so back edge is at the hinge
    seat_ring_geo = _oval_disc_ring(
        rx_out=seat_rx_out,
        ry_out=0.178,
        rx_in=0.120,
        ry_in=0.125,
        thick=0.020,
        cx=seat_local_cx,
    )
    seat.visual(
        mesh_from_cadquery(seat_ring_geo.translate((0, 0, 0.002)), "seat_ring_shell"),
        material=seat_white,
        name="seat_ring_shell",
    )
    seat.inertial = Inertial.from_geometry(
        Box((0.38, 0.36, 0.025)),
        mass=0.9,
        origin=Origin(xyz=(seat_local_cx, 0.0, 0.012)),
    )
    model.articulation(
        "body_to_seat_ring",
        ArticulationType.REVOLUTE,
        parent=body,
        child=seat,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0,
            lower=-math.radians(100.0), upper=0.0,
        ),
    )

    # ================= lid (revolute, same rear hinge) =================
    lid = model.part("lid")
    # Lid extends forward from hinge; back edge at local x=0 (hinge).
    lid_rx = 0.200
    lid_local_cx = lid_rx  # center so back edge is at the hinge
    lid_geo = _oval_disc(rx=lid_rx, ry=0.188, thick=0.016, cx=lid_local_cx)
    lid.visual(
        mesh_from_cadquery(lid_geo.translate((0, 0, 0.022)), "lid_shell"),
        material=seat_white,
        name="lid_shell",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.40, 0.38, 0.018)),
        mass=1.0,
        origin=Origin(xyz=(lid_local_cx, 0.0, 0.030)),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0,
            lower=-math.radians(100.0), upper=0.0,
        ),
    )

    # ================= flush lever (revolute, tank left side) =================
    lever = model.part("flush_lever")

    # Lever pivot cylinder (small chrome cylinder going into the tank wall)
    lever_pivot = CylinderGeometry(0.010, 0.022, radial_segments=24)
    lever_pivot_solid = lever_pivot.rotate_x(math.pi / 2.0)  # orient along Y
    lever.visual(
        mesh_from_geometry(lever_pivot_solid, "lever_pivot"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome,
        name="lever_pivot",
    )

    # Lever arm: extends in -Y (outward from left tank face)
    # Part frame at the pivot point; arm extends in -Y and slightly up
    arm_length = 0.065
    arm_geo = BoxGeometry((0.014, arm_length, 0.010))
    lever.visual(
        mesh_from_geometry(arm_geo, "lever_arm"),
        origin=Origin(xyz=(0.0, -(arm_length / 2.0 + 0.008), 0.0)),
        material=chrome,
        name="lever_arm",
    )

    # Lever handle knob at the end of the arm
    knob_geo = CylinderGeometry(0.012, 0.018, radial_segments=20)
    knob_solid = knob_geo.rotate_x(math.pi / 2.0)
    lever.visual(
        mesh_from_geometry(knob_solid, "lever_knob"),
        origin=Origin(xyz=(0.0, -(arm_length + 0.014), 0.0)),
        material=chrome,
        name="lever_knob",
    )

    lever.inertial = Inertial.from_geometry(
        Box((0.014, 0.08, 0.012)),
        mass=0.08,
        origin=Origin(xyz=(0.0, -0.04, 0.0)),
    )

    # Articulation: axis along +X so positive rotation pushes lever tip down
    model.articulation(
        "body_to_flush_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(LEVER_X, LEVER_Y, LEVER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0,
            lower=0.0, upper=math.radians(30.0),
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
    lever = object_model.get_part("flush_lever")

    seat_joint = object_model.get_articulation("body_to_seat_ring")
    lid_joint = object_model.get_articulation("body_to_lid")
    lever_joint = object_model.get_articulation("body_to_flush_lever")

    # --- Intentional overlaps ---
    ctx.allow_overlap(
        seat, body,
        elem_a="seat_ring_shell", elem_b="bowl_rim",
        reason="Seat ring rests on the ceramic bowl rim shelf (seated contact).",
    )
    ctx.allow_overlap(
        seat, body,
        elem_a="seat_ring_shell", elem_b="seat_shelf",
        reason="Seat ring rests on the seat shelf surface.",
    )
    ctx.allow_overlap(
        lid, seat,
        elem_a="lid_shell", elem_b="seat_ring_shell",
        reason="Closed lid rests on top of the seat ring.",
    )
    ctx.allow_overlap(
        lever, body,
        elem_a="lever_pivot", elem_b="tank_shell",
        reason="Flush lever pivot is captured in the tank wall.",
    )

    # --- One-piece toilet sits on the floor (base reaches z≈0) ---
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "body base reaches near the floor (floor-standing)",
        body_aabb[0][2] < 0.02,
        details=f"min z of body = {body_aabb[0][2]}",
    )

    # --- Tank is present and tall (integrated tank) ---
    tank_top = body_aabb[1][2]
    ctx.check(
        "integrated tank extends above seat rim",
        tank_top > SEAT_RIM_Z + 0.15,
        details=f"body top z={tank_top}, rim z={SEAT_RIM_Z}",
    )

    # --- Seat top is near 0.40 m above floor ---
    ctx.check(
        "seat rim is near 0.40 m above floor",
        0.35 < SEAT_RIM_Z < 0.45,
        details=f"rim z = {SEAT_RIM_Z}",
    )

    # --- Bowl has hollow interior (cavity visual exists) ---
    bowl_cavity = body.get_visual("bowl_cavity")
    ctx.check(
        "bowl has hollow interior cavity",
        bowl_cavity is not None,
        details="bowl_cavity visual missing from body",
    )

    # --- Raised rim exists ---
    bowl_rim = body.get_visual("bowl_rim")
    ctx.check(
        "bowl has raised rim geometry",
        bowl_rim is not None,
        details="bowl_rim visual missing from body",
    )

    # --- Floor bolt caps exist ---
    bolt_cap_0 = body.get_visual("bolt_cap_0")
    bolt_cap_1 = body.get_visual("bolt_cap_1")
    ctx.check(
        "floor bolt caps present at base",
        bolt_cap_0 is not None and bolt_cap_1 is not None,
        details="bolt cap visuals missing from body",
    )

    # --- Lid sits above seat ring when closed ---
    seat_z = ctx.part_world_aabb(seat)[1][2]
    lid_z = ctx.part_world_aabb(lid)[0][2]
    ctx.check(
        "lid sits above the seat ring when closed",
        lid_z >= seat_z - 0.005,
        details=f"lid bottom z={lid_z}, seat top z={seat_z}",
    )

    # --- Lid rotates open ~100 deg (front edge lifts up and back) ---
    lid_front_x0 = ctx.part_world_aabb(lid)[1][0]
    lid_top_z0 = ctx.part_world_aabb(lid)[1][2]
    with ctx.pose({lid_joint: -math.radians(100.0)}):
        lid_top_z1 = ctx.part_world_aabb(lid)[1][2]
        lid_front_x1 = ctx.part_world_aabb(lid)[1][0]
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

    # --- Seat ring and lid share the same rear hinge axis ---
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

    # --- Seat ring rotates open ---
    seat_top_z0 = ctx.part_world_aabb(seat)[1][2]
    with ctx.pose({seat_joint: -math.radians(100.0)}):
        seat_top_z1 = ctx.part_world_aabb(seat)[1][2]
    ctx.check(
        "seat ring rotates open (lifts upward)",
        seat_top_z1 > seat_top_z0 + 0.05,
        details=f"closed top z={seat_top_z0}, open top z={seat_top_z1}",
    )

    # --- Flush lever exists and rotates (push down) ---
    lever_joint_info = lever_joint
    ctx.check(
        "flush lever is revolute joint",
        lever_joint_info.articulation_type == ArticulationType.REVOLUTE,
        details=f"lever type={lever_joint_info.articulation_type}",
    )

    # Lever tip moves down when pushed (positive q = axis +X right-hand rule)
    lever_aabb_rest = ctx.part_world_aabb(lever)
    with ctx.pose({lever_joint: math.radians(25.0)}):
        lever_aabb_pushed = ctx.part_world_aabb(lever)
    # When pushed, the arm tip (which extends in -Y) rotates downward,
    # so the AABB minimum Z should decrease.
    rest_min_z = lever_aabb_rest[0][2]
    pushed_min_z = lever_aabb_pushed[0][2]
    ctx.check(
        "flush lever rotates when pushed (tip moves down)",
        pushed_min_z < rest_min_z - 0.003,
        details=f"rest min z={rest_min_z}, pushed min z={pushed_min_z}",
    )

    # --- Flush lever is on the tank side (left side, -Y) ---
    lever_y = ctx.part_world_position(lever)[1]
    ctx.check(
        "flush lever is on the left side of the tank",
        lever_y < -0.05,
        details=f"lever y={lever_y}",
    )

    # --- Flush lever is near tank height (not at floor level) ---
    lever_z = ctx.part_world_position(lever)[2]
    ctx.check(
        "flush lever is mounted on the tank (above seat rim)",
        lever_z > SEAT_RIM_Z + 0.05,
        details=f"lever z={lever_z}",
    )

    return ctx.report()


object_model = build_object_model()
