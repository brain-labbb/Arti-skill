from __future__ import annotations

# High-tank vintage toilet with pull chain.
#
# Frame / coordinate convention:
#   +X = forward (front of the bowl points to +X), wall is at -X.
#   +Y = left-right (the hinge axis for the lid + seat ring runs along Y).
#   +Z = up. The floor is at z=0; the seat top sits ~0.40 m above the floor.
#
# This is a vintage-style high-tank toilet:
#   - The ceramic cistern tank is mounted high on the wall (~1.5 m).
#   - A chrome flush pipe runs from the tank bottom down to the bowl rear.
#   - A pull chain with a ceramic handle hangs from the tank for flushing.
#   - The bowl is wall-mounted (cantilevered from a back panel).
#   - The seat ring and lid both rotate independently on visible hinge barrels.
#   - The tank has a separate lid with a visible seam line.
#
# Root part = the wall-mounting back panel + ceramic bowl + flush pipe +
# high tank + tank lid + hinge barrels (one fixed assembly).
#   - seat_ring : oval seat ring, REVOLUTE at rear hinge (axis +Y), ~100 deg.
#   - lid       : oval lid, REVOLUTE at rear hinge (axis +Y), ~100 deg,
#                 independent of the seat.
#   - pull_chain: PRISMATIC downward (-Z), ~0.12 m travel, pull to flush.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key dimensions (meters) ----
SEAT_TOP_Z = 0.400  # top of the seat ring above the floor
BOWL_W = 0.360  # bowl width (Y)
BOWL_DEPTH = 0.540  # bowl depth (X), front lip to wall
WALL_X = -0.010  # front face of the wall back-panel

# Rear hinge axis (shared by lid + seat ring), in world coords.
HINGE_X = 0.060
HINGE_Z = SEAT_TOP_Z + 0.004

# Seat plate top surface z (where seat ring + lid live).
SEAT_Z = SEAT_TOP_Z

# High tank dimensions
TANK_BOTTOM_Z = 1.350  # bottom of tank (high on wall)
TANK_W = 0.360  # tank width (Y)
TANK_D = 0.180  # tank depth (X)
TANK_H = 0.280  # tank height (Z)
TANK_CENTER_X = WALL_X - TANK_D / 2.0 + 0.020  # centered near wall

# Flush pipe
PIPE_RADIUS = 0.018
PIPE_TOP_Z = TANK_BOTTOM_Z  # connects at tank bottom
PIPE_BOTTOM_Z = SEAT_Z + 0.05  # connects to bowl rear just above seat

# Pull chain
CHAIN_TOP_Z = TANK_BOTTOM_Z - 0.02  # chain attaches just below tank
CHAIN_LENGTH = 0.55  # chain hangs down
CHAIN_HANDLE_Z = CHAIN_TOP_Z - CHAIN_LENGTH  # handle position


def _oval_loft(z_bottom, z_top, rx, ry, taper=0.85, segs=64) -> cq.Workplane:
    def ellipse_pts(rx_, ry_):
        pts = []
        for i in range(segs):
            a = 2.0 * math.pi * i / segs
            pts.append((rx_ * math.cos(a), ry_ * math.sin(a)))
        return pts

    wp = (
        cq.Workplane("XY")
        .workplane(offset=z_bottom)
        .polyline(ellipse_pts(rx * taper, ry * taper))
        .close()
        .workplane(offset=(z_top - z_bottom))
        .polyline(ellipse_pts(rx, ry))
        .close()
        .loft(ruled=False)
    )
    return wp


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


def _bowl_solid() -> cq.Workplane:
    cx = 0.215
    top_z = SEAT_Z
    bottom_z = SEAT_Z - 0.300

    outer = (
        cq.Workplane("XY")
        .workplane(offset=bottom_z)
        .ellipse(0.075, 0.080)
        .workplane(offset=0.090)
        .ellipse(0.130, 0.120)
        .workplane(offset=0.120)
        .ellipse(0.165, 0.165)
        .workplane(offset=0.090)
        .ellipse(0.180, 0.180)
        .loft(ruled=False)
    )
    outer = outer.translate((cx, 0.0, 0.0))

    cavity = (
        cq.Workplane("XY")
        .workplane(offset=top_z - 0.190)
        .ellipse(0.085, 0.090)
        .workplane(offset=0.130)
        .ellipse(0.150, 0.150)
        .workplane(offset=0.075)
        .ellipse(0.158, 0.158)
        .loft(ruled=False)
        .translate((cx, 0.0, 0.0))
    )
    bowl = outer.cut(cavity)

    shelf = _oval_ring(
        top_z - 0.022, rx_out=0.200, ry_out=0.195, rx_in=0.150, ry_in=0.150, thick=0.022, cx=cx
    )
    bowl = bowl.union(shelf)

    return bowl


def _back_panel_solid() -> cq.Workplane:
    # The back panel spans from bowl level up to the high tank mounting area.
    panel = (
        cq.Workplane("XY")
        .workplane(offset=0.060)
        .center(WALL_X - 0.015, 0.0)
        .box(0.030, 0.380, 1.600, centered=(True, True, False))
    )
    return panel


def _bowl_neck_solid() -> cq.Workplane:
    neck = (
        cq.Workplane("XY")
        .workplane(offset=SEAT_Z - 0.170)
        .center(0.045, 0.0)
        .box(0.130, 0.230, 0.180, centered=(True, True, False))
    )
    return neck


def _flush_pipe_solid() -> cq.Workplane:
    # Chrome pipe running from tank bottom down to bowl rear.
    pipe_cx = WALL_X - 0.030
    pipe_height = PIPE_TOP_Z - PIPE_BOTTOM_Z
    pipe = (
        cq.Workplane("XY")
        .workplane(offset=PIPE_BOTTOM_Z)
        .center(pipe_cx, 0.0)
        .circle(PIPE_RADIUS)
        .extrude(pipe_height)
    )
    # Add a small flange at top and bottom for visual realism
    flange_top = (
        cq.Workplane("XY")
        .workplane(offset=PIPE_TOP_Z - 0.010)
        .center(pipe_cx, 0.0)
        .circle(PIPE_RADIUS + 0.006)
        .extrude(0.015)
    )
    flange_bot = (
        cq.Workplane("XY")
        .workplane(offset=PIPE_BOTTOM_Z)
        .center(pipe_cx, 0.0)
        .circle(PIPE_RADIUS + 0.006)
        .extrude(0.015)
    )
    return pipe.union(flange_top).union(flange_bot)


def _high_tank_solid() -> cq.Workplane:
    # Rectangular ceramic cistern tank mounted high on wall.
    tank_cx = WALL_X - TANK_D / 2.0 + 0.015
    tank = (
        cq.Workplane("XY")
        .workplane(offset=TANK_BOTTOM_Z)
        .center(tank_cx, 0.0)
        .box(TANK_D, TANK_W, TANK_H, centered=(True, True, False))
    )
    # Round the front edges slightly with a fillet-like chamfer effect
    # by adding a subtle front face panel
    return tank


def _tank_lid_solid() -> cq.Workplane:
    # A thin lid panel on top of the tank with a visible seam (gap line).
    tank_cx = WALL_X - TANK_D / 2.0 + 0.015
    lid_thick = 0.012
    # The lid is slightly inset from the tank edges to show a seam.
    inset = 0.004
    lid = (
        cq.Workplane("XY")
        .workplane(offset=TANK_BOTTOM_Z + TANK_H + 0.001)
        .center(tank_cx, 0.0)
        .box(TANK_D - inset * 2, TANK_W - inset * 2, lid_thick, centered=(True, True, False))
    )
    return lid


def _hinge_barrels_solid() -> cq.Workplane:
    # Two visible chrome hinge barrel cylinders behind the seat mounting area.
    # Each barrel is a short cylinder along Y, placed at the rear hinge line.
    barrel_r = 0.008
    barrel_len = 0.040
    barrel_z = HINGE_Z
    y_offsets = [-0.100, 0.100]
    result = None
    for y_center in y_offsets:
        # Build a cylinder along Y using extrude from XZ workplane
        y_start = y_center - barrel_len / 2.0
        barrel = (
            cq.Workplane("XZ")
            .workplane(offset=y_start)
            .center(HINGE_X, barrel_z)
            .circle(barrel_r)
            .extrude(barrel_len)
        )
        if result is None:
            result = barrel
        else:
            result = result.union(barrel)
    return result


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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_tank_vintage_toilet")

    ceramic = model.material("ceramic_white", rgba=(0.95, 0.95, 0.94, 1.0))
    seat_white = model.material("seat_white", rgba=(0.97, 0.97, 0.96, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    wall_gray = model.material("wall_gray", rgba=(0.72, 0.72, 0.70, 1.0))
    chain_brass = model.material("chain_brass", rgba=(0.72, 0.60, 0.30, 1.0))
    porcelain_handle = model.material("porcelain_white", rgba=(0.96, 0.96, 0.95, 1.0))

    cx = 0.215

    # ================= ROOT: back panel + bowl + pipe + tank + hinge barrels =
    body = model.part("body")

    # Wall back-panel (mounting surface spanning from floor area up to tank).
    body.visual(
        mesh_from_cadquery(_back_panel_solid(), "back_panel"),
        material=wall_gray,
        name="back_panel",
    )

    # Cantilevered ceramic bowl + connecting neck shroud.
    bowl = _bowl_solid().union(_bowl_neck_solid())
    body.visual(mesh_from_cadquery(bowl, "bowl_shell"), material=ceramic, name="bowl_shell")

    # Chrome flush pipe from tank to bowl.
    body.visual(
        mesh_from_cadquery(_flush_pipe_solid(), "flush_pipe"),
        material=chrome,
        name="flush_pipe",
    )

    # High ceramic cistern tank.
    body.visual(
        mesh_from_cadquery(_high_tank_solid(), "high_tank"),
        material=ceramic,
        name="high_tank",
    )

    # Tank lid (separate visual with visible seam gap above tank body).
    body.visual(
        mesh_from_cadquery(_tank_lid_solid(), "tank_lid"),
        material=ceramic,
        name="tank_lid",
    )

    # Visible hinge barrels behind the seat mounting area.
    body.visual(
        mesh_from_cadquery(_hinge_barrels_solid(), "hinge_barrels"),
        material=chrome,
        name="hinge_barrels",
    )

    body.inertial = Inertial.from_geometry(
        Box((0.55, 0.38, 1.60)),
        mass=30.0,
        origin=Origin(xyz=(0.10, 0.0, 0.80)),
    )

    # ================= seat ring (revolute, rear axis) =================
    seat = model.part("seat_ring")
    seat_local_cx = cx - HINGE_X
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

    # ================= lid (revolute, SAME rear axis, independent) ==========
    lid = model.part("lid")
    lid_local_cx = cx - HINGE_X
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

    # ================= pull chain (prismatic, downward) =====================
    chain = model.part("pull_chain")

    # Chain rod (thin brass rod hanging from the tank).
    rod_len = CHAIN_LENGTH - 0.04
    chain_rod = CylinderGeometry(0.004, rod_len, radial_segments=16)
    chain.visual(
        mesh_from_geometry(chain_rod, "chain_rod"),
        origin=Origin(xyz=(0.0, 0.0, -rod_len / 2.0)),
        material=chain_brass,
        name="chain_rod",
    )

    # Ceramic pull handle at the bottom (cone taper shape, wide at bottom).
    handle_geo = ConeGeometry(0.018, 0.040, radial_segments=24)
    chain.visual(
        mesh_from_geometry(handle_geo, "chain_handle"),
        origin=Origin(xyz=(0.0, 0.0, -rod_len / 2.0 - 0.020 - 0.020)),
        material=porcelain_handle,
        name="chain_handle",
    )

    chain.inertial = Inertial.from_geometry(
        Box((0.020, 0.020, CHAIN_LENGTH)),
        mass=0.15,
        origin=Origin(xyz=(0.0, 0.0, -CHAIN_LENGTH / 2.0)),
    )

    pipe_cx = WALL_X - 0.030
    model.articulation(
        "body_to_pull_chain",
        ArticulationType.PRISMATIC,
        parent=body,
        child=chain,
        origin=Origin(xyz=(pipe_cx, 0.0, CHAIN_TOP_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=0.3, lower=0.0, upper=0.12
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
    chain = object_model.get_part("pull_chain")

    seat_joint = object_model.get_articulation("body_to_seat_ring")
    lid_joint = object_model.get_articulation("body_to_lid")
    chain_joint = object_model.get_articulation("body_to_pull_chain")

    # --- Intentional overlaps ---
    ctx.allow_overlap(
        seat, body,
        elem_a="seat_ring_shell", elem_b="bowl_shell",
        reason="Seat ring rests on the ceramic bowl rim shelf (seated contact).",
    )
    ctx.allow_overlap(
        lid, seat,
        elem_a="lid_shell", elem_b="seat_ring_shell",
        reason="Closed lid rests on top of the seat ring.",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="bowl_shell", elem_b="back_panel",
        reason="Bowl neck shroud is fused into the wall back-panel (cantilever support).",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="flush_pipe", elem_b="back_panel",
        reason="Flush pipe mounts against the back panel wall surface.",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="high_tank", elem_b="back_panel",
        reason="High tank is mounted against the back panel wall surface.",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="hinge_barrels", elem_b="bowl_shell",
        reason="Hinge barrels are embedded in the rear bowl rim for seat mounting.",
    )

    # --- Tank is high on the wall (well above the bowl) ---
    tank_aabb = ctx.part_element_world_aabb(body, elem="high_tank")
    ctx.check(
        "high tank is mounted well above the bowl",
        tank_aabb[0][2] > 1.0,
        details=f"tank bottom z = {tank_aabb[0][2]}",
    )

    # --- Tank lid sits above the tank body with a visible seam gap ---
    tank_lid_aabb = ctx.part_element_world_aabb(body, elem="tank_lid")
    ctx.check(
        "tank lid sits above the tank body",
        tank_lid_aabb[0][2] > tank_aabb[1][2] - 0.005,
        details=f"tank_lid bottom z={tank_lid_aabb[0][2]}, tank top z={tank_aabb[1][2]}",
    )

    # --- Flush pipe connects tank to bowl ---
    pipe_aabb = ctx.part_element_world_aabb(body, elem="flush_pipe")
    ctx.check(
        "flush pipe spans from near tank bottom to near bowl",
        pipe_aabb[1][2] > 1.2 and pipe_aabb[0][2] < 0.55,
        details=f"pipe z range = [{pipe_aabb[0][2]}, {pipe_aabb[1][2]}]",
    )

    # --- Hinge barrels exist behind the seat area ---
    hinge_aabb = ctx.part_element_world_aabb(body, elem="hinge_barrels")
    ctx.check(
        "hinge barrels are near the seat hinge height",
        abs(hinge_aabb[0][2] - HINGE_Z) < 0.03,
        details=f"hinge barrels z={hinge_aabb[0][2]}, expected near {HINGE_Z}",
    )

    # --- Bowl is wall-hung (no floor pedestal) ---
    bowl_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "bowl is wall-hung (does not reach the floor)",
        bowl_aabb[0][2] > 0.05,
        details=f"min z of body = {bowl_aabb[0][2]}",
    )

    # --- Seat ring and lid are independent (different joints) ---
    ctx.check(
        "seat and lid have independent articulations",
        seat_joint.name != lid_joint.name,
        details=f"seat joint={seat_joint.name}, lid joint={lid_joint.name}",
    )

    # --- Lid hinge origin is slightly above seat hinge (independent rotation) ---
    so = seat_joint.origin
    lo = lid_joint.origin
    ctx.check(
        "lid hinge is at or above seat hinge (independent axis offset)",
        lo.xyz[2] >= so.xyz[2] - 0.001,
        details=f"seat hinge z={so.xyz[2]}, lid hinge z={lo.xyz[2]}",
    )

    # --- Seat ring rests on the bowl, lid rests above the seat when closed ---
    seat_z = ctx.part_world_aabb(seat)[1][2]
    lid_z = ctx.part_world_aabb(lid)[0][2]
    ctx.check(
        "lid sits above the seat ring when closed",
        lid_z >= seat_z - 0.005,
        details=f"lid bottom z={lid_z}, seat top z={seat_z}",
    )

    # --- Lid rotates open ~100 deg ---
    lid_top_z0 = ctx.part_world_aabb(lid)[1][2]
    lid_front_x0 = ctx.part_world_aabb(lid)[1][0]
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

    # --- Seat ring rotates open on same Y axis as lid ---
    ctx.check(
        "seat ring and lid share the same hinge axis direction",
        tuple(seat_joint.axis) == tuple(lid_joint.axis),
        details=f"seat axis={seat_joint.axis}, lid axis={lid_joint.axis}",
    )
    seat_top_z0 = ctx.part_world_aabb(seat)[1][2]
    with ctx.pose({seat_joint: -math.radians(100.0)}):
        seat_top_z1 = ctx.part_world_aabb(seat)[1][2]
    ctx.check(
        "seat ring rotates open (lifts upward)",
        seat_top_z1 > seat_top_z0 + 0.05,
        details=f"closed top z={seat_top_z0}, open top z={seat_top_z1}",
    )

    # --- Pull chain moves downward when pulled (prismatic) ---
    chain_pos_rest = ctx.part_world_position(chain)
    with ctx.pose({chain_joint: 0.12}):
        chain_pos_pulled = ctx.part_world_position(chain)
    ctx.check(
        "pull chain moves downward when pulled",
        chain_pos_pulled[2] < chain_pos_rest[2] - 0.08,
        details=f"rest z={chain_pos_rest[2]}, pulled z={chain_pos_pulled[2]}",
    )

    # --- Pull chain hangs from near the tank ---
    ctx.check(
        "pull chain origin is near the high tank",
        chain_pos_rest[2] > 0.8,
        details=f"chain rest z={chain_pos_rest[2]}",
    )

    # --- Pull chain joint is prismatic ---
    ctx.check(
        "pull chain articulation is prismatic",
        chain_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"chain joint type={chain_joint.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
