from __future__ import annotations

# Commercial flushometer toilet with exposed supply pipe and flush lever.
#
# Frame / coordinate convention:
#   +X = forward (front of the bowl points to +X), wall is at -X.
#   +Y = left-right (the hinge axis for the lid + seat ring runs along Y).
#   +Z = up. The floor is at z=0; the seat top sits ~0.40 m above the floor.
#
# Root part = the floor-standing ceramic bowl with pedestal base.
# Everything else mounts to or near that root:
#   - lid          : oval top lid, REVOLUTE hinge at the rear (axis +Y), ~100 deg.
#   - seat_ring    : oval seat ring under the lid, REVOLUTE hinge sharing the
#                    SAME rear axis as the lid (concentric), ~100 deg.
#   - flush_lever  : chrome lever handle on the flushometer valve side,
#                    REVOLUTE ~30 deg downward to activate flush.
#   - bolt_cap_0/1 : floor bolt caps at the base (fixed, decorative).
# The chrome flushometer valve body and supply pipe are fixed visual geometry
# on the root part; the flush lever rotates on the valve.

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
BOWL_DEPTH = 0.540  # bowl depth (X), front lip to wall
WALL_X = 0.000  # wall face

# Rear hinge axis (shared by lid + seat ring), in world coords.
HINGE_X = 0.060
HINGE_Z = SEAT_TOP_Z + 0.004

# Seat plate top surface z (where seat ring + lid live).
SEAT_Z = SEAT_TOP_Z

# Flushometer valve location
VALVE_X = -0.040  # slightly behind the bowl rear
VALVE_Z = 0.580  # valve center height
VALVE_Y = 0.120  # offset to one side for the lever


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


def _bowl_with_pedestal() -> cq.Workplane:
    """Floor-standing commercial bowl with pedestal base down to the floor."""
    cx = 0.215  # bowl center X
    top_z = SEAT_Z
    # The bowl body from seat level down
    bottom_z = SEAT_Z - 0.200

    # Outer body - lofted oval sections from mid-body up to the rim
    outer = (
        cq.Workplane("XY")
        .workplane(offset=bottom_z)
        .ellipse(0.100, 0.110)
        .workplane(offset=0.080)
        .ellipse(0.140, 0.140)
        .workplane(offset=0.060)
        .ellipse(0.165, 0.165)
        .workplane(offset=0.060)
        .ellipse(0.180, 0.180)
        .loft(ruled=False)
    )
    outer = outer.translate((cx, 0.0, 0.0))

    # Hollow the basin
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=top_z - 0.170)
        .ellipse(0.090, 0.095)
        .workplane(offset=0.120)
        .ellipse(0.150, 0.150)
        .workplane(offset=0.055)
        .ellipse(0.158, 0.158)
        .loft(ruled=False)
        .translate((cx, 0.0, 0.0))
    )
    bowl = outer.cut(cavity)

    # Seat shelf: flat oval ceramic rim
    shelf = _oval_ring(
        top_z - 0.022, rx_out=0.200, ry_out=0.195, rx_in=0.150, ry_in=0.150, thick=0.022, cx=cx
    )
    bowl = bowl.union(shelf)

    # Pedestal / base: tapers from the bowl bottom down to the floor
    pedestal = (
        cq.Workplane("XY")
        .workplane(offset=0.000)
        .center(cx, 0.0)
        .box(0.160, 0.220, 0.001, centered=(True, True, False))  # floor plate
    )
    # Taper column from floor up to bowl bottom
    ped_col = (
        cq.Workplane("XY")
        .workplane(offset=0.005)
        .center(cx, 0.0)
        .box(0.140, 0.200, bottom_z - 0.005, centered=(True, True, False))
    )
    # Skirt connecting pedestal top to bowl bottom (wider transition)
    skirt = (
        cq.Workplane("XY")
        .workplane(offset=bottom_z - 0.040)
        .center(cx, 0.0)
        .box(0.200, 0.240, 0.060, centered=(True, True, False))
    )
    bowl = bowl.union(pedestal).union(ped_col).union(skirt)

    # Rear shroud connecting bowl back to near-wall area (for pipe connection look)
    rear_shroud = (
        cq.Workplane("XY")
        .workplane(offset=SEAT_Z - 0.160)
        .center(0.045, 0.0)
        .box(0.110, 0.160, 0.160, centered=(True, True, False))
    )
    bowl = bowl.union(rear_shroud)

    return bowl


def _flushometer_valve() -> cq.Workplane:
    """Chrome flushometer valve body - cylindrical with inlet/outlet."""
    # Main valve body: vertical cylinder
    valve_body = (
        cq.Workplane("XY")
        .workplane(offset=VALVE_Z - 0.060)
        .center(VALVE_X, 0.0)
        .circle(0.032)
        .extrude(0.120)
    )
    # Top cap (rounded)
    top_cap = (
        cq.Workplane("XY")
        .workplane(offset=VALVE_Z + 0.060)
        .center(VALVE_X, 0.0)
        .circle(0.034)
        .extrude(0.015)
    )
    # Bottom connector (outlet to bowl)
    bottom_connector = (
        cq.Workplane("XY")
        .workplane(offset=VALVE_Z - 0.080)
        .center(VALVE_X, 0.0)
        .circle(0.022)
        .extrude(0.025)
    )
    # Side boss for lever mount
    lever_boss = (
        cq.Workplane("XY")
        .workplane(offset=VALVE_Z - 0.020)
        .center(VALVE_X, VALVE_Y)
        .circle(0.018)
        .extrude(0.025)
    )
    valve = valve_body.union(top_cap).union(bottom_connector).union(lever_boss)
    return valve


def _supply_pipe() -> cq.Workplane:
    """Exposed chrome supply pipe from valve upward and to the wall."""
    # Vertical pipe from valve top up to wall entry
    pipe_vert = (
        cq.Workplane("XY")
        .workplane(offset=VALVE_Z + 0.075)
        .center(VALVE_X, 0.0)
        .circle(0.016)
        .extrude(0.500)  # goes up to ~1.075m
    )
    # Horizontal stub from valve to wall
    pipe_horiz = (
        cq.Workplane("XZ")
        .workplane(offset=0.0)
        .center(VALVE_Z + 0.0, WALL_X - 0.020)
        .circle(0.016)
        .extrude(abs(VALVE_X) + 0.020)
    )
    # Wall flange
    flange = (
        cq.Workplane("XY")
        .workplane(offset=VALVE_Z + 0.570)
        .center(VALVE_X, 0.0)
        .circle(0.028)
        .extrude(0.012)
    )
    pipe = pipe_vert.union(pipe_horiz).union(flange)
    return pipe


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
    model = ArticulatedObject(name="flushometer_toilet")

    ceramic = model.material("ceramic_white", rgba=(0.95, 0.95, 0.94, 1.0))
    seat_white = model.material("seat_white", rgba=(0.97, 0.97, 0.96, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    bolt_cap = model.material("bolt_cap_brass", rgba=(0.72, 0.65, 0.35, 1.0))

    cx = 0.215

    # ================= ROOT: floor-standing bowl + pedestal + valve + pipe ===
    body = model.part("body")

    # Floor-standing ceramic bowl with pedestal base
    bowl_geo = _bowl_with_pedestal()
    body.visual(mesh_from_cadquery(bowl_geo, "bowl_shell"), material=ceramic, name="bowl_shell")

    # Chrome flushometer valve body (fixed on the root)
    valve_geo = _flushometer_valve()
    body.visual(mesh_from_cadquery(valve_geo, "flushometer_valve"), material=chrome, name="flushometer_valve")

    # Exposed supply pipe (fixed on the root)
    pipe_geo = _supply_pipe()
    body.visual(mesh_from_cadquery(pipe_geo, "supply_pipe"), material=chrome, name="supply_pipe")

    body.inertial = Inertial.from_geometry(
        Box((0.55, 0.38, 0.45)),
        mass=28.0,
        origin=Origin(xyz=(0.18, 0.0, SEAT_Z - 0.10)),
    )

    # ================= Floor bolt caps (fixed visuals on body) ===============
    # Two bolt caps at the base, one on each side of the pedestal
    for i, cap_y in enumerate([-0.085, 0.085]):
        cap_geo = CylinderGeometry(0.014, 0.012, radial_segments=24)
        body.visual(
            mesh_from_geometry(cap_geo, f"bolt_cap_{i}"),
            origin=Origin(xyz=(cx, cap_y, 0.006)),
            material=bolt_cap,
            name=f"bolt_cap_{i}",
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

    # ================= lid (revolute, SAME rear axis) =================
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

    # ================= Flush lever (revolute on valve side) ==================
    # The lever rotates downward (~30 deg) to activate the flush.
    # Lever origin is at the valve's side boss, axis along +Y so positive
    # rotation pushes the lever tip downward (-Z).
    lever = model.part("flush_lever")

    # Lever arm: a flat elongated piece extending from the valve boss outward (+Y)
    lever_arm = (
        cq.Workplane("XY")
        .center(0.0, 0.040)
        .box(0.016, 0.090, 0.012, centered=(True, True, True))
    )
    # Lever handle (wider grip at the end)
    lever_handle = (
        cq.Workplane("XY")
        .center(0.0, 0.085)
        .box(0.022, 0.025, 0.018, centered=(True, True, True))
    )
    lever_geo = lever_arm.union(lever_handle)
    lever.visual(
        mesh_from_cadquery(lever_geo, "flush_lever_arm"),
        material=chrome,
        name="flush_lever_arm",
    )
    lever.inertial = Inertial.from_geometry(
        Box((0.016, 0.100, 0.018)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.040, 0.0)),
    )

    # Articulation: lever pivots at the valve boss
    # Origin at the boss center, axis = +Y so positive q rotates tip down
    lever_origin_world = (VALVE_X, VALVE_Y + 0.012, VALVE_Z - 0.008)
    model.articulation(
        "body_to_flush_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=lever_origin_world),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=4.0, lower=0.0, upper=math.radians(30.0)
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
        lever, body,
        elem_a="flush_lever_arm", elem_b="flushometer_valve",
        reason="Flush lever pivots on the valve side boss (captured pivot).",
    )

    # --- Bowl is floor-standing (pedestal reaches near the floor). ---
    bowl_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "bowl is floor-standing (pedestal reaches near floor)",
        bowl_aabb[0][2] < 0.02,
        details=f"min z of body = {bowl_aabb[0][2]}",
    )
    ctx.check(
        "seat top is near 0.40 m above floor",
        0.34 < SEAT_Z < 0.46,
        details=f"seat top z = {SEAT_Z}",
    )

    # --- Flushometer valve is above the bowl (visible exposed hardware). ---
    valve_center_z = VALVE_Z
    ctx.check(
        "flushometer valve is above the seat (exposed commercial valve)",
        valve_center_z > SEAT_Z + 0.05,
        details=f"valve z = {valve_center_z}, seat z = {SEAT_Z}",
    )

    # --- Supply pipe extends well above the valve (visible exposed pipe). ---
    pipe_max_z = VALVE_Z + 0.075 + 0.500  # top of vertical pipe
    ctx.check(
        "exposed supply pipe extends above the valve",
        pipe_max_z > VALVE_Z + 0.30,
        details=f"pipe top z = {pipe_max_z}",
    )

    # --- Floor bolt caps are near the floor at the bowl base. ---
    # Bolt caps are visuals on body, check their z position
    bolt_cap_0 = body.get_visual("bolt_cap_0")
    bolt_cap_1 = body.get_visual("bolt_cap_1")
    ctx.check(
        "floor bolt cap 0 is near the floor",
        bolt_cap_0.origin.xyz[2] < 0.020,
        details=f"bolt_cap_0 z = {bolt_cap_0.origin.xyz[2]}",
    )
    ctx.check(
        "floor bolt cap 1 is near the floor",
        bolt_cap_1.origin.xyz[2] < 0.020,
        details=f"bolt_cap_1 z = {bolt_cap_1.origin.xyz[2]}",
    )
    # Bolt caps are on opposite sides of the pedestal
    ctx.check(
        "bolt caps are on opposite sides of the pedestal",
        bolt_cap_0.origin.xyz[1] * bolt_cap_1.origin.xyz[1] < 0,
        details=f"cap0 y={bolt_cap_0.origin.xyz[1]}, cap1 y={bolt_cap_1.origin.xyz[1]}",
    )

    # --- Seat ring rests on the bowl, lid rests above the seat when closed. ---
    seat_z = ctx.part_world_aabb(seat)[1][2]
    lid_z = ctx.part_world_aabb(lid)[0][2]
    ctx.check(
        "lid sits above the seat ring when closed",
        lid_z >= seat_z - 0.005,
        details=f"lid bottom z={lid_z}, seat top z={seat_z}",
    )

    # --- Lid rotates open ~100 deg (front edge lifts up and back). ---
    lid_top_z0 = ctx.part_world_aabb(lid)[1][2]
    with ctx.pose({lid_joint: -math.radians(100.0)}):
        lid_top_z1 = ctx.part_world_aabb(lid)[1][2]
    ctx.check(
        "lid rotates open (lifts upward)",
        lid_top_z1 > lid_top_z0 + 0.05,
        details=f"closed top z={lid_top_z0}, open top z={lid_top_z1}",
    )

    # --- Seat ring rotates on the SAME axis as the lid (concentric hinge). ---
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

    # --- Flush lever rotates downward to activate flush. ---
    lever_pos_rest = ctx.part_world_position(lever)
    with ctx.pose({lever_joint: math.radians(30.0)}):
        lever_pos_actuated = ctx.part_world_position(lever)
    ctx.check(
        "flush lever rotates when actuated (tip moves downward)",
        lever_pos_actuated[2] < lever_pos_rest[2] - 0.005,
        details=f"rest z={lever_pos_rest[2]}, actuated z={lever_pos_actuated[2]}",
    )
    ctx.check(
        "flush lever joint is revolute",
        lever_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type = {lever_joint.articulation_type}",
    )
    ctx.check(
        "flush lever has limited rotation range",
        lever_joint.motion_limits.upper > 0.0 and lever_joint.motion_limits.upper < math.radians(45.0),
        details=f"upper limit = {lever_joint.motion_limits.upper} rad",
    )

    # --- Flush lever is mounted on the valve side (above the bowl). ---
    lever_z = ctx.part_world_position(lever)[2]
    ctx.check(
        "flush lever is mounted above seat height (on the valve)",
        lever_z > SEAT_Z,
        details=f"lever z = {lever_z}",
    )

    return ctx.report()


object_model = build_object_model()
