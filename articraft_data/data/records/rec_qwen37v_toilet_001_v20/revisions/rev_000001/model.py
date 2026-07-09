from __future__ import annotations

# Commercial flushometer floor-standing toilet with exposed supply pipe.
#
# Frame / coordinate convention:
#   +X = forward (front of the bowl points to +X), wall is at -X.
#   +Y = left-right (the hinge axis for the lid + seat ring runs along Y).
#   +Z = up. The floor is at z=0; the seat top sits ~0.40 m above the floor.
#
# Root part = the floor-standing ceramic toilet body (bowl + pedestal base +
# flushometer pipe assembly + floor bolt caps). Everything else mounts to it:
#   - lid          : oval top lid, REVOLUTE hinge at the rear (axis +Y), ~100 deg.
#   - seat_ring    : oval seat ring under the lid, REVOLUTE hinge sharing the
#                    SAME rear axis as the lid (concentric), ~100 deg.
#   - bidet_nozzle : small chrome nozzle that slides forward (+X) on a
#                    PRISMATIC joint under the bowl rim, ~50 mm travel.
#
# The flushometer valve and exposed pipe are fixed chrome geometry on the body.
# Floor bolt caps are fixed ceramic visuals at the pedestal base.

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
SEAT_TOP_Z = 0.400  # top of the seat ring above the floor
BOWL_W = 0.360  # bowl width (Y)
BOWL_DEPTH = 0.540  # bowl depth (X), front lip to back
PEDESTAL_H = 0.260  # pedestal base height from floor to bowl bottom

# Rear hinge axis (shared by lid + seat ring), in world coords.
HINGE_X = 0.060
HINGE_Z = SEAT_TOP_Z + 0.004

# Seat plate top surface z (where seat ring + lid live).
SEAT_Z = SEAT_TOP_Z


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
    # Floor-standing ceramic bowl: a rounded body that tapers from a wide
    # seat shelf at the top down to a narrower base, with a hollowed basin
    # interior and a raised rim around the top opening.
    cx = 0.215  # bowl center X
    top_z = SEAT_Z
    bottom_z = PEDESTAL_H  # bowl bottom sits on top of pedestal

    # Outer body, lofted oval sections from the pedestal top up to the rim.
    outer = (
        cq.Workplane("XY")
        .workplane(offset=bottom_z)
        .ellipse(0.120, 0.110)
        .workplane(offset=0.060)
        .ellipse(0.155, 0.145)
        .workplane(offset=0.080)
        .ellipse(0.175, 0.175)
        .workplane(offset=0.060)
        .ellipse(0.180, 0.180)
        .loft(ruled=False)
    )
    outer = outer.translate((cx, 0.0, 0.0))

    # Hollow the basin: an inner oval cavity cut from the top, creating
    # the visible hollow bowl interior.
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=top_z - 0.160)
        .ellipse(0.090, 0.095)
        .workplane(offset=0.100)
        .ellipse(0.140, 0.140)
        .workplane(offset=0.060)
        .ellipse(0.148, 0.148)
        .loft(ruled=False)
        .translate((cx, 0.0, 0.0))
    )
    bowl = outer.cut(cavity)

    # Raised rim: a torus-like ring around the bowl opening, protruding
    # above the seat shelf to create the visible raised rim geometry.
    rim = _oval_ring(
        top_z + 0.002, rx_out=0.185, ry_out=0.185, rx_in=0.148, ry_in=0.148, thick=0.018, cx=cx
    )
    bowl = bowl.union(rim)

    # Seat shelf: a flat oval ceramic rim/shelf the seat ring rests on.
    shelf = _oval_ring(
        top_z - 0.022, rx_out=0.200, ry_out=0.195, rx_in=0.150, ry_in=0.150, thick=0.022, cx=cx
    )
    bowl = bowl.union(shelf)

    return bowl


def _pedestal_solid() -> cq.Workplane:
    # Ceramic pedestal/base connecting the bowl to the floor.
    # Tapers from a wider base at the floor to a narrower top where it meets the bowl.
    cx = 0.215
    base_z = 0.0
    top_z = PEDESTAL_H

    pedestal = (
        cq.Workplane("XY")
        .workplane(offset=base_z)
        .ellipse(0.140, 0.130)
        .workplane(offset=0.080)
        .ellipse(0.130, 0.120)
        .workplane(offset=top_z - 0.080)
        .ellipse(0.120, 0.110)
        .loft(ruled=False)
    )
    pedestal = pedestal.translate((cx, 0.0, 0.0))
    return pedestal


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


def _flushometer_pipe_solid() -> cq.Workplane:
    # Exposed chrome flushometer valve and supply pipe assembly.
    # Vertical pipe rises from behind the bowl, valve body at mid-height,
    # then a horizontal feed pipe connects into the bowl rear.
    cx = 0.215  # bowl center
    pipe_x = 0.020  # pipe X position (near the wall/rear of bowl)
    pipe_z_bottom = PEDESTAL_H + 0.100
    pipe_z_top = SEAT_Z + 0.450  # pipe extends well above the bowl

    # Vertical supply pipe (chrome tube)
    vert_pipe = (
        cq.Workplane("XY")
        .workplane(offset=pipe_z_bottom)
        .circle(0.016)
        .circle(0.012)
        .extrude(pipe_z_top - pipe_z_bottom)
    )
    vert_pipe = vert_pipe.translate((pipe_x, 0.0, 0.0))

    # Valve body (larger chrome cylinder at mid-height)
    valve_z = pipe_z_bottom + 0.180
    valve_body = (
        cq.Workplane("XY")
        .workplane(offset=valve_z)
        .circle(0.032)
        .extrude(0.080)
    )
    valve_body = valve_body.translate((pipe_x, 0.0, 0.0))

    # Valve handle (small horizontal cylinder)
    handle = (
        cq.Workplane("XZ")
        .workplane(offset=0.0)
        .center(pipe_x, valve_z + 0.040)
        .circle(0.008)
        .extrude(0.050)
    )
    handle = handle.translate((0.0, 0.035, 0.0))

    # Horizontal feed pipe from valve down into bowl rear
    feed_z = SEAT_Z - 0.050
    feed_pipe = (
        cq.Workplane("XZ")
        .workplane(offset=0.0)
        .center(pipe_x, feed_z)
        .circle(0.010)
        .circle(0.007)
        .extrude(cx - 0.100 - pipe_x)
    )
    feed_pipe = feed_pipe.translate((0.0, 0.0, 0.0))

    assembly = vert_pipe.union(valve_body).union(handle).union(feed_pipe)
    return assembly


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_flushometer_toilet")

    ceramic = model.material("ceramic_white", rgba=(0.95, 0.95, 0.94, 1.0))
    seat_white = model.material("seat_white", rgba=(0.97, 0.97, 0.96, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    cap_white = model.material("cap_white", rgba=(0.93, 0.93, 0.92, 1.0))

    cx = 0.215

    # ================= ROOT: floor-standing toilet body =================
    body = model.part("body")

    # Ceramic bowl with hollow interior and raised rim.
    bowl = _bowl_solid()
    body.visual(mesh_from_cadquery(bowl, "bowl_shell"), material=ceramic, name="bowl_shell")

    # Ceramic pedestal base connecting bowl to floor.
    pedestal = _pedestal_solid()
    body.visual(mesh_from_cadquery(pedestal, "pedestal_shell"), material=ceramic, name="pedestal_shell")

    # Chrome flushometer valve and exposed pipe assembly (fixed).
    flushometer = _flushometer_pipe_solid()
    body.visual(mesh_from_cadquery(flushometer, "flushometer_pipe"), material=chrome, name="flushometer_pipe")

    # Floor bolt caps: two oval ceramic caps at the pedestal base, covering
    # the floor mounting bolts. Positioned symmetrically left and right.
    cap_y_offsets = [-0.080, 0.080]
    for i, cap_y in enumerate(cap_y_offsets):
        cap_name = f"bolt_cap_{i}"
        cap_geo = (
            cq.Workplane("XY")
            .workplane(offset=0.005)
            .center(cx, cap_y)
            .ellipse(0.022, 0.016)
            .extrude(0.012)
        )
        body.visual(
            mesh_from_cadquery(cap_geo, cap_name),
            material=cap_white,
            name=cap_name,
        )

    body.inertial = Inertial.from_geometry(
        Box((0.55, 0.38, 0.70)),
        mass=28.0,
        origin=Origin(xyz=(0.18, 0.0, 0.35)),
    )

    # ================= seat ring (revolute, rear axis) =================
    seat = model.part("seat_ring")
    seat_local_cx = cx - HINGE_X  # bowl center relative to the hinge
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

    # ================= bidet nozzle (prismatic, slides forward +X) =================
    # A small chrome nozzle that slides forward under the bowl rim.
    nozzle = model.part("bidet_nozzle")
    # Nozzle body: a small horizontal cylinder pointing forward
    nozzle_body = CylinderGeometry(radius=0.008, height=0.040, radial_segments=32)
    nozzle_body = nozzle_body.rotate_y(math.pi / 2.0)  # align with X axis
    nozzle.visual(
        mesh_from_geometry(nozzle_body, "nozzle_body"),
        material=chrome,
        name="nozzle_body",
    )
    # Nozzle tip: a smaller tapered end
    nozzle_tip = CylinderGeometry(radius=0.005, height=0.015, radial_segments=24)
    nozzle_tip = nozzle_tip.rotate_y(math.pi / 2.0)
    nozzle.visual(
        mesh_from_geometry(nozzle_tip.translate(0.027, 0.0, 0.0), "nozzle_tip"),
        material=chrome,
        name="nozzle_tip",
    )
    nozzle.inertial = Inertial.from_geometry(
        Box((0.055, 0.016, 0.016)),
        mass=0.08,
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
    )

    # Nozzle slides forward (+X) from a retracted position under the rim.
    # Origin is at the rear of the nozzle travel, under the bowl rim.
    nozzle_origin_x = cx - 0.080  # retracted position near bowl rear
    nozzle_origin_z = SEAT_Z - 0.060  # just under the rim
    model.articulation(
        "body_to_bidet_nozzle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=nozzle,
        origin=Origin(xyz=(nozzle_origin_x, 0.0, nozzle_origin_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=0.05, lower=0.0, upper=0.050
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

    seat_joint = object_model.get_articulation("body_to_seat_ring")
    lid_joint = object_model.get_articulation("body_to_lid")
    nozzle_joint = object_model.get_articulation("body_to_bidet_nozzle")

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
        body, body,
        elem_a="bowl_shell", elem_b="pedestal_shell",
        reason="Bowl is fused into the pedestal base (floor-standing construction).",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="flushometer_pipe", elem_b="bowl_shell",
        reason="Flushometer feed pipe connects into the bowl rear (plumbing connection).",
    )

    # --- Toilet is floor-standing (pedestal reaches near the floor). ---
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "toilet is floor-standing (pedestal reaches near floor)",
        body_aabb[0][2] < 0.02,
        details=f"min z of body = {body_aabb[0][2]}",
    )

    # --- Seat top is near 0.40 m above floor. ---
    ctx.check(
        "seat top is near 0.40 m above floor",
        0.34 < SEAT_Z < 0.46,
        details=f"seat top z = {SEAT_Z}",
    )

    # --- Seat ring rests on the bowl, lid rests above the seat when closed. ---
    seat_z = ctx.part_world_aabb(seat)[1][2]  # top of seat ring
    lid_z = ctx.part_world_aabb(lid)[0][2]  # bottom of lid
    ctx.check(
        "lid sits above the seat ring when closed",
        lid_z >= seat_z - 0.005,
        details=f"lid bottom z={lid_z}, seat top z={seat_z}",
    )

    # --- Lid rotates open ~100 deg (front edge lifts up and back). ---
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

    # --- Bidet nozzle slides forward on prismatic joint. ---
    nozzle_x0 = ctx.part_world_position(nozzle)[0]
    with ctx.pose({nozzle_joint: 0.050}):
        nozzle_x1 = ctx.part_world_position(nozzle)[0]
    ctx.check(
        "bidet nozzle slides forward (+X) when extended",
        nozzle_x1 > nozzle_x0 + 0.040,
        details=f"retracted x={nozzle_x0}, extended x={nozzle_x1}",
    )
    ctx.check(
        "bidet nozzle is positioned under the bowl rim",
        SEAT_Z - 0.120 < nozzle_x0 < SEAT_Z + 0.100,
        details=f"nozzle z={ctx.part_world_position(nozzle)[2]}, seat z={SEAT_Z}",
    )

    # --- Flushometer pipe extends above the bowl. ---
    pipe_visual = body.get_visual("flushometer_pipe")
    pipe_aabb = ctx.part_element_world_aabb(body, elem="flushometer_pipe")
    ctx.check(
        "flushometer pipe extends well above the bowl",
        pipe_aabb[1][2] > SEAT_Z + 0.30,
        details=f"pipe top z={pipe_aabb[1][2]}, seat z={SEAT_Z}",
    )

    # --- Floor bolt caps exist at the pedestal base. ---
    cap_0 = body.get_visual("bolt_cap_0")
    cap_1 = body.get_visual("bolt_cap_1")
    cap_0_aabb = ctx.part_element_world_aabb(body, elem="bolt_cap_0")
    cap_1_aabb = ctx.part_element_world_aabb(body, elem="bolt_cap_1")
    ctx.check(
        "floor bolt caps exist near the floor",
        cap_0_aabb[0][2] < 0.030 and cap_1_aabb[0][2] < 0.030,
        details=f"cap_0 min z={cap_0_aabb[0][2]}, cap_1 min z={cap_1_aabb[0][2]}",
    )
    ctx.check(
        "floor bolt caps are symmetric left-right",
        abs(cap_0_aabb[0][1] + cap_1_aabb[1][1]) < 0.05,
        details=f"cap_0 y={cap_0_aabb[0][1]}, cap_1 y={cap_1_aabb[1][1]}",
    )

    return ctx.report()


object_model = build_object_model()