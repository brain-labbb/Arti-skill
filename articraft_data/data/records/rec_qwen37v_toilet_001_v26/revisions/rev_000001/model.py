from __future__ import annotations

# Tankless smart toilet with rear service pod, floor-standing.
#
# Frame / coordinate convention:
#   +X = forward (front of the bowl points to +X).
#   +Y = left-right (hinge axis for lid + seat ring runs along Y).
#   +Z = up. The floor is at z=0; the seat top sits ~0.40 m above the floor.
#
# Root part = the ceramic body (bowl + pedestal base + rear service pod).
# Everything else mounts to that root:
#   - lid          : oval top lid, REVOLUTE hinge at the rear (axis +Y), ~100 deg.
#   - seat_ring    : oval seat ring under the lid, REVOLUTE hinge sharing the
#                    SAME rear axis as the lid (concentric), ~100 deg.
#   - flush_handle : flushometer-style lever handle on the service pod side,
#                    REVOLUTE pivot axis along Y, pushes down ~30 deg.
#   - bolt_cap_0, bolt_cap_1 : floor bolt caps at the pedestal base (fixed
#                    visuals on the body).
#
# The bowl has a hollow interior cavity with a raised rim visible at the top.

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
BOWL_DEPTH = 0.540  # bowl depth (X), front lip to rear
BOWL_CX = 0.220  # bowl center X

# Rear hinge axis (shared by lid + seat ring), in world coords.
HINGE_X = 0.020
HINGE_Z = SEAT_TOP_Z + 0.004

# Service pod dimensions
POD_W = 0.300  # width (Y)
POD_D = 0.180  # depth (X)
POD_H = 0.520  # height (Z)
POD_X = -0.060  # center X of pod (behind bowl)
POD_Z = 0.0  # pod sits on floor


def _oval_ring(z, rx_out, ry_out, rx_in, ry_in, thick, cx=0.0, segs=72) -> cq.Workplane:
    """A flat oval ring (annular ellipse) of given thickness centered at (cx,0,z)."""
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


def _bowl_with_rim_and_cavity() -> cq.Workplane:
    """The ceramic bowl: outer shell with hollow interior and raised rim."""
    cx = BOWL_CX
    top_z = SEAT_TOP_Z
    bottom_z = SEAT_TOP_Z - 0.280

    # Outer body: lofted oval sections from rounded bottom up to the rim shelf.
    outer = (
        cq.Workplane("XY")
        .workplane(offset=bottom_z)
        .ellipse(0.070, 0.075)
        .workplane(offset=0.080)
        .ellipse(0.125, 0.115)
        .workplane(offset=0.110)
        .ellipse(0.160, 0.160)
        .workplane(offset=0.090)
        .ellipse(0.178, 0.178)
        .loft(ruled=False)
    )
    outer = outer.translate((cx, 0.0, 0.0))

    # Hollow basin cavity: cut from inside, leaving walls ~15-20mm thick.
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=top_z - 0.200)
        .ellipse(0.065, 0.070)
        .workplane(offset=0.110)
        .ellipse(0.130, 0.130)
        .workplane(offset=0.060)
        .ellipse(0.140, 0.140)
        .loft(ruled=False)
        .translate((cx, 0.0, 0.0))
    )
    bowl = outer.cut(cavity)

    # Raised rim: a torus-like ring at the top of the bowl, slightly proud of
    # the seat shelf. This is the visible "lip" of the bowl opening.
    rim_height = 0.018
    rim = _oval_ring(
        top_z - 0.002,
        rx_out=0.185, ry_out=0.182,
        rx_in=0.148, ry_in=0.148,
        thick=rim_height,
        cx=cx,
    )
    bowl = bowl.union(rim)

    # Seat shelf: a flat oval ceramic rim the seat ring rests on.
    shelf = _oval_ring(
        top_z - 0.024, rx_out=0.198, ry_out=0.193, rx_in=0.155, ry_in=0.155, thick=0.024, cx=cx
    )
    bowl = bowl.union(shelf)

    return bowl


def _pedestal_base() -> cq.Workplane:
    """Floor-standing pedestal connecting bowl to floor."""
    cx = BOWL_CX
    # Tapered base: wider at floor, narrower at bowl connection.
    base = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .box(0.260, 0.300, 0.010, centered=(False, True, False))
        .translate((cx - 0.130, 0.0, 0.0))
    )
    # Vertical column connecting base to bowl underside
    column = (
        cq.Workplane("XY")
        .workplane(offset=0.010)
        .center(cx, 0.0)
        .box(0.200, 0.240, SEAT_TOP_Z - 0.280 - 0.010, centered=(True, True, False))
    )
    return base.union(column)


def _service_pod() -> cq.Workplane:
    """Rear service pod housing electronics and flush valve."""
    # Rectangular pod with slightly rounded appearance (chamfered box).
    pod = (
        cq.Workplane("XY")
        .workplane(offset=POD_Z)
        .center(POD_X, 0.0)
        .box(POD_D, POD_W, POD_H, centered=(True, True, False))
    )
    # Add a top cap that's slightly domed (flat box for simplicity, with slight bevel)
    cap = (
        cq.Workplane("XY")
        .workplane(offset=POD_Z + POD_H)
        .center(POD_X, 0.0)
        .box(POD_D - 0.020, POD_W - 0.020, 0.015, centered=(True, True, False))
    )
    return pod.union(cap)


def _pod_to_bowl_connector() -> cq.Workplane:
    """Ceramic connector/shroud between service pod and bowl rear."""
    # Bridges the gap between the pod front face and the bowl rear.
    conn_x = POD_X + POD_D / 2.0
    conn_end_x = BOWL_CX - 0.100
    conn_depth = conn_end_x - conn_x
    if conn_depth < 0.01:
        conn_depth = 0.04
    mid_x = conn_x + conn_depth / 2.0
    conn = (
        cq.Workplane("XY")
        .workplane(offset=SEAT_TOP_Z - 0.200)
        .center(mid_x, 0.0)
        .box(conn_depth, 0.220, 0.200, centered=(True, True, False))
    )
    return conn


def _floor_bolt_cap(y_offset: float) -> cq.Workplane:
    """Small chrome dome cap covering a floor mounting bolt."""
    cap = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .center(BOWL_CX, y_offset)
        .cylinder(0.018, 0.012)
    )
    # Small dome on top
    dome = (
        cq.Workplane("XY")
        .workplane(offset=0.012)
        .center(BOWL_CX, y_offset)
        .sphere(0.012)
    )
    # Cut sphere in half (keep top half)
    cut_box = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .center(BOWL_CX, y_offset)
        .box(0.040, 0.040, 0.024, centered=(True, True, False))
    )
    dome = dome.cut(cut_box.translate((0, 0, -0.024)))
    return cap.union(dome)


def _flush_handle_solid() -> cq.Workplane:
    """Flushometer-style lever handle, built in handle-local frame.
    Pivot axis is at local origin along Y. Handle arm extends in +X.
    """
    # Pivot boss (cylinder around the pivot axis)
    boss = (
        cq.Workplane("XZ")
        .center(0.0, 0.0)
        .circle(0.015)
        .extrude(0.030)
        .translate((0.0, -0.015, 0.0))
    )
    # Handle arm: a tapered lever extending in +X direction from pivot
    arm = (
        cq.Workplane("XY")
        .workplane(offset=-0.006)
        .center(0.060, 0.0)
        .box(0.120, 0.018, 0.012, centered=(True, True, False))
    )
    # Handle grip end: slightly wider rounded end
    grip = (
        cq.Workplane("XY")
        .workplane(offset=-0.008)
        .center(0.120, 0.0)
        .box(0.030, 0.026, 0.016, centered=(True, True, False))
    )
    handle = boss.union(arm).union(grip)
    return handle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tankless_smart_toilet")

    ceramic = model.material("ceramic_white", rgba=(0.95, 0.95, 0.94, 1.0))
    seat_white = model.material("seat_white", rgba=(0.97, 0.97, 0.96, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.30, 0.30, 0.32, 1.0))

    cx = BOWL_CX

    # ================= ROOT: ceramic body (bowl + pedestal + service pod) =====
    body = model.part("body")

    # Bowl with hollow interior and raised rim.
    bowl_geo = _bowl_with_rim_and_cavity()
    body.visual(mesh_from_cadquery(bowl_geo, "bowl_shell"), material=ceramic, name="bowl_shell")

    # Pedestal base (floor-standing support).
    pedestal = _pedestal_base()
    body.visual(mesh_from_cadquery(pedestal, "pedestal"), material=ceramic, name="pedestal")

    # Rear service pod.
    pod = _service_pod()
    body.visual(mesh_from_cadquery(pod, "service_pod"), material=ceramic, name="service_pod")

    # Connector between pod and bowl.
    connector = _pod_to_bowl_connector()
    body.visual(mesh_from_cadquery(connector, "pod_connector"), material=ceramic, name="pod_connector")

    # Floor bolt caps (chrome, fixed visuals on body).
    bolt_cap_geo_0 = _floor_bolt_cap(0.100)
    body.visual(mesh_from_cadquery(bolt_cap_geo_0, "bolt_cap_0"), material=chrome, name="bolt_cap_0")
    bolt_cap_geo_1 = _floor_bolt_cap(-0.100)
    body.visual(mesh_from_cadquery(bolt_cap_geo_1, "bolt_cap_1"), material=chrome, name="bolt_cap_1")

    # Small status LED indicator on service pod front (smart toilet detail).
    led_x = POD_X + POD_D / 2.0 + 0.002
    body.visual(
        Box((0.004, 0.040, 0.008)),
        origin=Origin(xyz=(led_x, 0.0, POD_H * 0.75)),
        material=dark_gray,
        name="status_panel",
    )

    body.inertial = Inertial.from_geometry(
        Box((0.65, 0.38, 0.55)),
        mass=30.0,
        origin=Origin(xyz=(0.12, 0.0, SEAT_TOP_Z - 0.15)),
    )

    # ================= seat ring (revolute, rear axis) =================
    seat = model.part("seat_ring")
    seat_local_cx = cx - HINGE_X  # bowl center relative to the hinge
    seat_ring_geo = _oval_ring(
        z=0.002,
        rx_out=0.178, ry_out=0.176,
        rx_in=0.110, ry_in=0.118,
        thick=0.020,
        cx=seat_local_cx,
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
    lid_geo = _oval_disc_solid(rx=0.188, ry=0.183, thick=0.016, cx=lid_local_cx)
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

    # ================= flush handle (revolute, on service pod side) ===========
    # The handle is mounted on the right side (+Y) of the service pod,
    # near the top. It pivots around the Y axis like a flushometer lever.
    handle = model.part("flush_handle")
    handle_geo = _flush_handle_solid()
    handle.visual(
        mesh_from_cadquery(handle_geo, "handle_lever"),
        material=chrome,
        name="handle_lever",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.14, 0.03, 0.02)),
        mass=0.15,
        origin=Origin(xyz=(0.06, 0.0, 0.0)),
    )
    # Mount on the right side of the service pod, near top.
    handle_mount_x = POD_X
    handle_mount_y = POD_W / 2.0 + 0.015
    handle_mount_z = POD_H * 0.80
    model.articulation(
        "body_to_flush_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(handle_mount_x, handle_mount_y, handle_mount_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=-math.radians(35.0), upper=math.radians(5.0)
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
        elem_a="bowl_shell", elem_b="pedestal",
        reason="Bowl shell sits on top of the pedestal base (structural connection).",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="service_pod", elem_b="pod_connector",
        reason="Pod connector bridges into the service pod face (structural weld).",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="bowl_shell", elem_b="pod_connector",
        reason="Pod connector bridges into the bowl rear (structural connection).",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="bolt_cap_0", elem_b="pedestal",
        reason="Floor bolt cap is seated into the pedestal base.",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="bolt_cap_1", elem_b="pedestal",
        reason="Floor bolt cap is seated into the pedestal base.",
    )

    # --- Toilet is floor-standing (pedestal reaches the floor). ---
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "toilet is floor-standing (reaches near the floor)",
        body_aabb[0][2] < 0.02,
        details=f"min z of body = {body_aabb[0][2]}",
    )

    # --- Seat top is near 0.40 m above floor. ---
    ctx.check(
        "seat top is near 0.40 m above floor",
        0.34 < SEAT_TOP_Z < 0.46,
        details=f"seat top z = {SEAT_TOP_Z}",
    )

    # --- Rear service pod exists and is behind the bowl. ---
    pod_aabb = ctx.part_element_world_aabb(body, elem="service_pod")
    bowl_aabb = ctx.part_element_world_aabb(body, elem="bowl_shell")
    ctx.check(
        "service pod is behind the bowl (pod center X < bowl center X)",
        (pod_aabb[0][0] + pod_aabb[1][0]) / 2.0 < (bowl_aabb[0][0] + bowl_aabb[1][0]) / 2.0 - 0.05,
        details=f"pod center x={(pod_aabb[0][0] + pod_aabb[1][0]) / 2.0}, bowl center x={(bowl_aabb[0][0] + bowl_aabb[1][0]) / 2.0}",
    )
    ctx.check(
        "service pod has substantial height (housing for electronics)",
        pod_aabb[1][2] - pod_aabb[0][2] > 0.30,
        details=f"pod height = {pod_aabb[1][2] - pod_aabb[0][2]}",
    )

    # --- Floor bolt caps exist at the base. ---
    cap0_aabb = ctx.part_element_world_aabb(body, elem="bolt_cap_0")
    cap1_aabb = ctx.part_element_world_aabb(body, elem="bolt_cap_1")
    ctx.check(
        "bolt cap 0 is near the floor",
        cap0_aabb[0][2] < 0.02,
        details=f"cap0 min z = {cap0_aabb[0][2]}",
    )
    ctx.check(
        "bolt cap 1 is near the floor",
        cap1_aabb[0][2] < 0.02,
        details=f"cap1 min z = {cap1_aabb[0][2]}",
    )
    # Bolt caps are on opposite sides of center (Y axis).
    cap0_y = (cap0_aabb[0][1] + cap0_aabb[1][1]) / 2.0
    cap1_y = (cap1_aabb[0][1] + cap1_aabb[1][1]) / 2.0
    ctx.check(
        "bolt caps are on opposite sides of centerline",
        cap0_y * cap1_y < 0 and abs(cap0_y - cap1_y) > 0.10,
        details=f"cap0 y={cap0_y}, cap1 y={cap1_y}",
    )

    # --- Hollow bowl interior: the bowl shell should have an internal cavity. ---
    # Verify the bowl has raised rim geometry (rim top > seat shelf top).
    bowl_shell_aabb = ctx.part_element_world_aabb(body, elem="bowl_shell")
    ctx.check(
        "bowl shell top is at or above seat level (raised rim present)",
        bowl_shell_aabb[1][2] >= SEAT_TOP_Z - 0.005,
        details=f"bowl top z={bowl_shell_aabb[1][2]}, seat z={SEAT_TOP_Z}",
    )

    # --- Lid sits above the seat ring when closed. ---
    seat_z = ctx.part_world_aabb(seat)[1][2]
    lid_z = ctx.part_world_aabb(lid)[0][2]
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

    # --- Flush handle pivots on revolute joint. ---
    ctx.check(
        "flush handle joint is revolute",
        handle_joint.type == ArticulationType.REVOLUTE,
        details=f"handle joint type = {handle_joint.type}",
    )
    handle_z0 = ctx.part_world_aabb(handle)[0][2]  # bottom of handle at rest
    handle_tip_x0 = ctx.part_world_aabb(handle)[1][0]  # frontmost x at rest
    with ctx.pose({handle_joint: -math.radians(30.0)}):
        handle_z1 = ctx.part_world_aabb(handle)[0][2]
        handle_tip_x1 = ctx.part_world_aabb(handle)[1][0]
    ctx.check(
        "flush handle pivots downward when pushed",
        handle_z1 < handle_z0 - 0.010,
        details=f"rest bottom z={handle_z0}, pushed bottom z={handle_z1}",
    )

    # --- Flush handle is mounted on the service pod (high, near pod top). ---
    handle_pos = ctx.part_world_position(handle)
    ctx.check(
        "flush handle is mounted high on the service pod",
        handle_pos[2] > POD_H * 0.60,
        details=f"handle z={handle_pos[2]}, pod top={POD_H}",
    )

    return ctx.report()


object_model = build_object_model()
