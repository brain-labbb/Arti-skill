from __future__ import annotations

# High-tank vintage toilet with pull chain, bidet nozzle, visible hinge barrels,
# and water inlet pipe.
#
# Frame / coordinate convention:
#   +X = forward (front of the bowl points to +X), wall is at -X.
#   +Y = left-right (the hinge axis for the lid + seat ring runs along Y).
#   +Z = up. The floor is at z=0; the seat top sits ~0.40 m above the floor.
#
# Root part = pedestal + ceramic bowl + wall back-panel + high cistern tank +
# water inlet pipe + hinge barrels (one fixed ceramic/plumbing assembly).
# Articulated children:
#   - lid           : oval top lid, REVOLUTE hinge at the rear (axis +Y), ~100 deg.
#   - seat_ring     : oval seat ring under the lid, REVOLUTE hinge sharing the
#                     SAME rear axis as the lid (concentric), ~100 deg.
#   - bidet_nozzle  : small spray nozzle that slides forward on a PRISMATIC
#                     joint (+X) from the rear of the bowl, ~0.08m travel.
#   - pull_handle   : chain pull handle below the tank, PRISMATIC (-Z, pull
#                     downward) ~0.04m travel to actuate flush.

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
SEAT_TOP_Z = 0.400  # top of the seat ring above the floor
BOWL_W = 0.360  # bowl width (Y)
BOWL_DEPTH = 0.500  # bowl depth (X), front lip to wall
WALL_X = -0.010  # front face of the wall back-panel

# Rear hinge axis (shared by lid + seat ring), in world coords.
HINGE_X = 0.060
HINGE_Z = SEAT_TOP_Z + 0.004

# Seat plate top surface z (where seat ring + lid live).
SEAT_Z = SEAT_TOP_Z

# High tank dimensions
TANK_BOTTOM_Z = 1.200  # bottom of tank above floor
TANK_W = 0.340  # tank width (Y)
TANK_D = 0.150  # tank depth (X)
TANK_H = 0.220  # tank height (Z)
TANK_CENTER_X = WALL_X - TANK_D / 2.0 + 0.020  # tank center X


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


def _pedestal_solid() -> cq.Workplane:
    """Floor-standing pedestal column supporting the bowl."""
    cx = 0.200
    # Tapered pedestal: wider at base, narrower at top
    pedestal = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .box(0.200, 0.220, 0.010, centered=(True, True, False))  # base plate
    )
    # Main column
    column = (
        cq.Workplane("XY")
        .workplane(offset=0.010)
        .center(cx - 0.02, 0.0)
        .box(0.160, 0.180, 0.180, centered=(True, True, False))
    )
    pedestal = pedestal.union(column)
    return pedestal.translate((cx, 0.0, 0.0))


def _bowl_solid() -> cq.Workplane:
    """The ceramic bowl: rounded body tapering from seat shelf to drain, hollowed."""
    cx = 0.220  # bowl center X
    top_z = SEAT_Z
    bottom_z = SEAT_Z - 0.250

    # Outer body lofted oval sections
    outer = (
        cq.Workplane("XY")
        .workplane(offset=bottom_z)
        .ellipse(0.070, 0.075)
        .workplane(offset=0.080)
        .ellipse(0.120, 0.110)
        .workplane(offset=0.100)
        .ellipse(0.160, 0.160)
        .workplane(offset=0.070)
        .ellipse(0.175, 0.175)
        .loft(ruled=False)
    )
    outer = outer.translate((cx, 0.0, 0.0))

    # Hollow the basin
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=top_z - 0.170)
        .ellipse(0.080, 0.085)
        .workplane(offset=0.110)
        .ellipse(0.140, 0.140)
        .workplane(offset=0.065)
        .ellipse(0.150, 0.150)
        .loft(ruled=False)
        .translate((cx, 0.0, 0.0))
    )
    bowl = outer.cut(cavity)

    # Seat shelf: flat oval ceramic rim around the basin opening
    shelf = _oval_ring(
        top_z - 0.022, rx_out=0.195, ry_out=0.190, rx_in=0.145, ry_in=0.145, thick=0.022, cx=cx
    )
    bowl = bowl.union(shelf)

    return bowl


def _back_panel_solid() -> cq.Workplane:
    """Wall mounting panel spanning from floor area up to the high tank."""
    panel = (
        cq.Workplane("XY")
        .workplane(offset=0.050)
        .center(WALL_X - 0.015, 0.0)
        .box(0.030, 0.360, 1.350, centered=(True, True, False))
    )
    return panel


def _bowl_neck_solid() -> cq.Workplane:
    """Ceramic shroud connecting back of bowl into the back panel."""
    neck = (
        cq.Workplane("XY")
        .workplane(offset=SEAT_Z - 0.160)
        .center(0.050, 0.0)
        .box(0.120, 0.220, 0.170, centered=(True, True, False))
    )
    return neck


def _high_tank_solid() -> cq.Workplane:
    """High-mounted ceramic cistern tank."""
    tank = (
        cq.Workplane("XY")
        .workplane(offset=TANK_BOTTOM_Z)
        .center(TANK_CENTER_X, 0.0)
        .box(TANK_D, TANK_W, TANK_H, centered=(True, True, False))
    )
    # Add a slight rounded cap on top (box with filleted top edges via union)
    cap = (
        cq.Workplane("XY")
        .workplane(offset=TANK_BOTTOM_Z + TANK_H - 0.010)
        .center(TANK_CENTER_X, 0.0)
        .box(TANK_D - 0.010, TANK_W - 0.010, 0.025, centered=(True, True, False))
    )
    tank = tank.union(cap)
    return tank


def _water_inlet_pipe_solid() -> cq.Workplane:
    """Visible water inlet/supply pipe from tank bottom down to bowl rear."""
    pipe_radius = 0.015
    # Vertical pipe section from tank bottom down to bowl rear connection
    pipe_top_z = TANK_BOTTOM_Z
    pipe_bottom_z = SEAT_Z + 0.020
    pipe_x = TANK_CENTER_X + 0.040

    pipe = (
        cq.Workplane("XY")
        .workplane(offset=pipe_bottom_z)
        .center(pipe_x, 0.0)
        .circle(pipe_radius)
        .extrude(pipe_top_z - pipe_bottom_z)
    )

    # Horizontal connection into the bowl rear
    connector = (
        cq.Workplane("XZ")
        .workplane(offset=0.0)  # at y=0
        .center(pipe_x, pipe_bottom_z + 0.015)
        .circle(pipe_radius)
        .extrude(-(pipe_x - 0.060))  # goes from pipe X toward bowl rear
    )

    pipe = pipe.union(connector)
    return pipe


def _hinge_barrel_solid(y_offset: float) -> cq.Workplane:
    """A visible hinge barrel cylinder at the rear of the seat mounting."""
    barrel_r = 0.010
    barrel_len = 0.030
    barrel = (
        cq.Workplane("XZ")
        .workplane(offset=y_offset - barrel_len / 2.0)
        .center(HINGE_X, HINGE_Z)
        .circle(barrel_r)
        .extrude(barrel_len)
    )
    return barrel


def _concealed_panel_outline_solid() -> cq.Workplane:
    """A subtle recessed outline on the wall panel showing concealed cistern access."""
    # Thin raised-border rectangle on the wall panel face
    outline = (
        cq.Workplane("XY")
        .workplane(offset=0.650)
        .center(WALL_X + 0.001, 0.0)
        .box(0.003, 0.250, 0.350, centered=(True, True, False))
    )
    return outline


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
    brass = model.material("brass", rgba=(0.72, 0.58, 0.28, 1.0))
    pipe_chrome = model.material("pipe_chrome", rgba=(0.70, 0.72, 0.74, 1.0))

    cx = 0.220  # bowl center

    # ================= ROOT: pedestal + bowl + wall panel + tank + pipe + barrels =========
    body = model.part("body")

    # Wall back-panel
    body.visual(
        mesh_from_cadquery(_back_panel_solid(), "back_panel"),
        material=wall_gray,
        name="back_panel",
    )

    # Pedestal + bowl + connecting neck
    bowl_assembly = _pedestal_solid().union(_bowl_solid()).union(_bowl_neck_solid())
    body.visual(
        mesh_from_cadquery(bowl_assembly, "bowl_shell"),
        material=ceramic,
        name="bowl_shell",
    )

    # High cistern tank
    body.visual(
        mesh_from_cadquery(_high_tank_solid(), "high_tank"),
        material=ceramic,
        name="high_tank",
    )

    # Water inlet pipe (chrome pipe from tank to bowl)
    body.visual(
        mesh_from_cadquery(_water_inlet_pipe_solid(), "water_inlet_pipe"),
        material=pipe_chrome,
        name="water_inlet_pipe",
    )

    # Concealed panel outline (access panel indicator on wall)
    body.visual(
        mesh_from_cadquery(_concealed_panel_outline_solid(), "concealed_panel"),
        material=wall_gray,
        name="concealed_panel",
    )

    # Visible hinge barrels (two, one on each side of the seat rear)
    hinge_barrel_left = _hinge_barrel_solid(-0.090)
    hinge_barrel_right = _hinge_barrel_solid(0.090)
    barrels = hinge_barrel_left.union(hinge_barrel_right)
    body.visual(
        mesh_from_cadquery(barrels, "hinge_barrels"),
        material=chrome,
        name="hinge_barrels",
    )

    body.inertial = Inertial.from_geometry(
        Box((0.55, 0.38, 1.50)),
        mass=35.0,
        origin=Origin(xyz=(0.15, 0.0, 0.60)),
    )

    # ================= seat ring (revolute, rear axis) =================
    seat = model.part("seat_ring")
    seat_local_cx = cx - HINGE_X
    seat_ring_geo = _oval_disc_ring_solid(
        rx_out=0.175,
        ry_out=0.173,
        rx_in=0.105,
        ry_in=0.113,
        thick=0.020,
        cx=seat_local_cx,
    )
    seat.visual(
        mesh_from_cadquery(seat_ring_geo.translate((0, 0, 0.002)), "seat_ring_shell"),
        material=seat_white,
        name="seat_ring_shell",
    )
    seat.inertial = Inertial.from_geometry(
        Box((0.35, 0.35, 0.025)),
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
    lid_local_cx = cx - HINGE_X
    lid_geo = _oval_disc_solid(rx=0.185, ry=0.180, thick=0.018, cx=lid_local_cx)
    lid.visual(
        mesh_from_cadquery(lid_geo.translate((0, 0, 0.020)), "lid_shell"),
        material=seat_white,
        name="lid_shell",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.37, 0.36, 0.020)),
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

    # ================= bidet nozzle (prismatic, slides forward +X) =========
    nozzle = model.part("bidet_nozzle")
    # Small cylindrical nozzle body, ~0.06m long tube
    nozzle_body = CylinderGeometry(0.008, 0.060, radial_segments=24)
    # Rotate so cylinder axis is along X (default is Z)
    nozzle_body = nozzle_body.rotate_y(math.pi / 2.0)
    # Nozzle tip (slightly wider spray head)
    nozzle_tip = CylinderGeometry(0.012, 0.015, radial_segments=24)
    nozzle_tip = nozzle_tip.rotate_y(math.pi / 2.0)

    nozzle.visual(
        mesh_from_geometry(nozzle_body, "nozzle_tube"),
        material=pipe_chrome,
        name="nozzle_tube",
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
    )
    nozzle.visual(
        mesh_from_geometry(nozzle_tip, "nozzle_tip"),
        material=chrome,
        name="nozzle_tip",
        origin=Origin(xyz=(0.065, 0.0, 0.0)),
    )
    nozzle.inertial = Inertial.from_geometry(
        Box((0.080, 0.024, 0.024)), mass=0.05
    )

    # Nozzle origin at the rear of the bowl interior; slides forward (+X)
    nozzle_origin_x = 0.080  # rear of bowl
    nozzle_origin_z = SEAT_Z - 0.060  # just below seat level inside bowl
    model.articulation(
        "body_to_bidet_nozzle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=nozzle,
        origin=Origin(xyz=(nozzle_origin_x, 0.0, nozzle_origin_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0, velocity=0.05, lower=0.0, upper=0.080
        ),
    )

    # ================= pull chain handle (prismatic, pull downward -Z) =========
    pull = model.part("pull_handle")
    # Chain handle: a small brass knob/pull with a short chain stub
    handle_knob = CylinderGeometry(0.014, 0.025, radial_segments=24)
    # chain segment above the knob
    chain_stub = CylinderGeometry(0.003, 0.040, radial_segments=12)

    pull.visual(
        mesh_from_geometry(handle_knob, "pull_knob"),
        material=brass,
        name="pull_knob",
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
    )
    pull.visual(
        mesh_from_geometry(chain_stub, "chain_stub"),
        material=chrome,
        name="chain_stub",
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )
    pull.inertial = Inertial.from_geometry(
        Box((0.028, 0.028, 0.065)), mass=0.08
    )

    # Pull handle hangs from the bottom of the tank; pull downward (-Z)
    pull_origin_z = TANK_BOTTOM_Z - 0.010
    model.articulation(
        "body_to_pull_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=pull,
        origin=Origin(xyz=(TANK_CENTER_X, 0.0, pull_origin_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=0.10, lower=0.0, upper=0.040
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
    nozzle = object_model.get_part("bidet_nozzle")
    pull = object_model.get_part("pull_handle")

    seat_joint = object_model.get_articulation("body_to_seat_ring")
    lid_joint = object_model.get_articulation("body_to_lid")
    nozzle_joint = object_model.get_articulation("body_to_bidet_nozzle")
    pull_joint = object_model.get_articulation("body_to_pull_handle")

    # --- Intentional overlaps: seated nesting of seat-on-rim and lid-on-seat. ---
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
        nozzle, body,
        elem_a="nozzle_tube", elem_b="bowl_shell",
        reason="Bidet nozzle is nested inside the bowl cavity, retracted into the rear wall.",
    )
    ctx.allow_overlap(
        body, pull,
        elem_a="high_tank", elem_b="chain_stub",
        reason="Chain stub inserts into the tank bottom (captured chain mount).",
    )
    ctx.allow_overlap(
        body, seat,
        elem_a="hinge_barrels", elem_b="seat_ring_shell",
        reason="Hinge barrels are captured at the seat ring pivot point (hinge embedding).",
    )

    # --- High tank exists above the bowl ---
    tank_aabb = ctx.part_element_world_aabb(body, elem="high_tank")
    bowl_aabb = ctx.part_element_world_aabb(body, elem="bowl_shell")
    ctx.check(
        "high tank is mounted above the bowl",
        tank_aabb[0][2] > 0.80,
        details=f"tank min z = {tank_aabb[0][2]}",
    )
    ctx.check(
        "tank bottom is well above seat level",
        tank_aabb[0][2] > SEAT_Z + 0.50,
        details=f"tank min z = {tank_aabb[0][2]}, seat z = {SEAT_Z}",
    )

    # --- Bowl is floor-standing (pedestal reaches near floor) ---
    ctx.check(
        "bowl pedestal reaches the floor",
        bowl_aabb[0][2] < 0.05,
        details=f"bowl min z = {bowl_aabb[0][2]}",
    )

    # --- Seat top is near 0.40 m above floor ---
    ctx.check(
        "seat top is near 0.40 m above floor",
        0.34 < SEAT_Z < 0.46,
        details=f"seat top z = {SEAT_Z}",
    )

    # --- Water inlet pipe is visible between tank and bowl ---
    pipe_aabb = ctx.part_element_world_aabb(body, elem="water_inlet_pipe")
    ctx.check(
        "water inlet pipe spans from near tank down to bowl level",
        pipe_aabb[1][2] > 1.0 and pipe_aabb[0][2] < 0.60,
        details=f"pipe z range = [{pipe_aabb[0][2]}, {pipe_aabb[1][2]}]",
    )

    # --- Hinge barrels exist behind the seat area ---
    barrel_aabb = ctx.part_element_world_aabb(body, elem="hinge_barrels")
    ctx.check(
        "hinge barrels are near the seat hinge height",
        abs(barrel_aabb[0][2] - HINGE_Z) < 0.030 or abs(barrel_aabb[1][2] - HINGE_Z) < 0.030,
        details=f"barrel z range = [{barrel_aabb[0][2]}, {barrel_aabb[1][2]}], hinge z = {HINGE_Z}",
    )
    ctx.check(
        "hinge barrels are at the rear of the bowl",
        barrel_aabb[1][0] < 0.12,
        details=f"barrel max x = {barrel_aabb[1][0]}",
    )

    # --- Lid sits above the seat ring when closed ---
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

    # --- Seat ring rotates on the SAME axis as the lid (concentric hinge) ---
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
    seat_top_z0 = ctx.part_world_aabb(seat)[1][2]
    with ctx.pose({seat_joint: -math.radians(100.0)}):
        seat_top_z1 = ctx.part_world_aabb(seat)[1][2]
    ctx.check(
        "seat ring rotates open (lifts upward)",
        seat_top_z1 > seat_top_z0 + 0.05,
        details=f"closed top z={seat_top_z0}, open top z={seat_top_z1}",
    )

    # --- Bidet nozzle slides forward (prismatic +X) ---
    nozzle_x0 = ctx.part_world_position(nozzle)[0]
    with ctx.pose({nozzle_joint: 0.080}):
        nozzle_x1 = ctx.part_world_position(nozzle)[0]
    ctx.check(
        "bidet nozzle slides forward when extended",
        nozzle_x1 > nozzle_x0 + 0.050,
        details=f"retracted x={nozzle_x0}, extended x={nozzle_x1}",
    )
    ctx.check(
        "bidet nozzle is mounted inside the bowl (below seat level)",
        ctx.part_world_position(nozzle)[2] < SEAT_Z,
        details=f"nozzle z={ctx.part_world_position(nozzle)[2]}",
    )

    # --- Pull handle is below the tank (hanging chain) ---
    pull_z = ctx.part_world_position(pull)[2]
    ctx.check(
        "pull handle hangs below the high tank",
        pull_z < TANK_BOTTOM_Z,
        details=f"pull z={pull_z}, tank bottom z={TANK_BOTTOM_Z}",
    )
    # Pull handle moves downward when actuated
    pull_z0 = ctx.part_world_position(pull)[2]
    with ctx.pose({pull_joint: 0.040}):
        pull_z1 = ctx.part_world_position(pull)[2]
    ctx.check(
        "pull handle moves downward when pulled",
        pull_z1 < pull_z0 - 0.020,
        details=f"rest z={pull_z0}, pulled z={pull_z1}",
    )

    # --- Concealed panel outline exists on the wall ---
    panel_aabb = ctx.part_element_world_aabb(body, elem="concealed_panel")
    ctx.check(
        "concealed panel outline is on the wall above the bowl",
        panel_aabb[0][2] > 0.50 and panel_aabb[0][0] < WALL_X + 0.020,
        details=f"panel z range = [{panel_aabb[0][2]}, {panel_aabb[1][2]}]",
    )

    # --- Proof checks for intentional overlaps ---
    # Nozzle is within the bowl footprint (XY containment)
    ctx.expect_within(
        nozzle, body,
        axes="xy",
        inner_elem="nozzle_tube",
        outer_elem="bowl_shell",
        margin=0.010,
        name="bidet nozzle stays within bowl footprint",
    )

    # Chain stub is near the tank (contact/seated)
    ctx.expect_contact(
        body, pull,
        elem_a="high_tank", elem_b="chain_stub",
        contact_tol=0.015,
        name="chain stub connects to tank bottom",
    )

    # Hinge barrels are at the seat hinge pivot (contact)
    ctx.expect_contact(
        body, seat,
        elem_a="hinge_barrels", elem_b="seat_ring_shell",
        contact_tol=0.012,
        name="hinge barrels contact the seat ring at the pivot",
    )

    return ctx.report()


object_model = build_object_model()
