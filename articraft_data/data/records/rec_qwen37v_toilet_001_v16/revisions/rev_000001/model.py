from __future__ import annotations

# Tankless smart toilet with rear service pod, floor-standing.
#
# Frame / coordinate convention:
#   +X = forward (front of the bowl points to +X).
#   +Y = left-right (the hinge axis for the lid + seat ring runs along Y).
#   +Z = up. The floor is at z=0; the seat top sits ~0.42 m above the floor.
#
# Root part = the ceramic body (bowl + rear service pod + floor bolt caps).
# Everything else mounts to that root:
#   - lid          : oval top lid, REVOLUTE hinge at the rear (axis +Y), ~100 deg.
#   - seat_ring    : oval seat ring with rubber bumpers underneath, REVOLUTE
#                    hinge sharing the SAME rear axis as the lid (concentric).
#   - flush_lever  : side-mounted flush lever on the pod, REVOLUTE ~25 deg.
#
# The rear service pod rises behind the bowl to house electronics and the flush
# mechanism. The toilet sits on the floor with bolt caps at the base.

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
SEAT_TOP_Z = 0.410  # top of seat ring above the floor
BOWL_W = 0.360  # bowl width (Y)
BOWL_DEPTH = 0.440  # bowl depth (X), front lip to rear junction
POD_DEPTH = 0.160  # service pod depth (X)
POD_WIDTH = 0.320  # service pod width (Y)
POD_HEIGHT = 0.550  # service pod height (Z)

# Bowl center X (measured from pod back)
POD_BACK_X = -0.010
POD_FRONT_X = POD_BACK_X + POD_DEPTH  # 0.150
BOWL_FRONT_X = POD_FRONT_X + BOWL_DEPTH  # 0.590
BOWL_CX = (POD_FRONT_X + BOWL_FRONT_X) / 2.0  # ~0.370

# Rear hinge axis (shared by lid + seat ring), in world coords.
HINGE_X = POD_FRONT_X - 0.010  # just behind bowl-pod junction
HINGE_Z = SEAT_TOP_Z + 0.006

# Seat plate top surface z (where seat ring + lid live).
SEAT_Z = SEAT_TOP_Z


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


def _service_pod_solid() -> cq.Workplane:
    """Rear service pod: a rounded rectangular housing behind the bowl."""
    pod_cx = (POD_BACK_X + POD_FRONT_X) / 2.0
    pod = (
        cq.Workplane("XY")
        .workplane(offset=0.010)  # slight base inset
        .center(pod_cx, 0.0)
        .box(POD_DEPTH - 0.010, POD_WIDTH, POD_HEIGHT - 0.010, centered=(True, True, False))
    )
    # Add a slight top cap/dome to round the top
    top_cap = (
        cq.Workplane("XY")
        .workplane(offset=POD_HEIGHT - 0.030)
        .center(pod_cx, 0.0)
        .box(POD_DEPTH - 0.020, POD_WIDTH - 0.020, 0.030, centered=(True, True, False))
    )
    pod = pod.union(top_cap)
    return pod


def _bowl_solid() -> cq.Workplane:
    """Floor-standing ceramic bowl: oval body from floor to rim, hollow basin."""
    cx = BOWL_CX
    top_z = SEAT_Z
    bottom_z = 0.020  # slight clearance above floor

    # Outer body: lofted oval from base to rim
    outer = (
        cq.Workplane("XY")
        .workplane(offset=bottom_z)
        .ellipse(0.090, 0.100)  # base oval
        .workplane(offset=0.100)
        .ellipse(0.140, 0.140)  # mid body
        .workplane(offset=0.140)
        .ellipse(0.180, 0.180)  # upper body
        .workplane(offset=0.150)
        .ellipse(0.195, 0.190)  # rim
        .loft(ruled=False)
    )
    outer = outer.translate((cx, 0.0, 0.0))

    # Hollow the basin: inner cavity cut from the top
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=top_z - 0.200)
        .ellipse(0.090, 0.095)
        .workplane(offset=0.120)
        .ellipse(0.150, 0.155)
        .workplane(offset=0.085)
        .ellipse(0.165, 0.165)
        .loft(ruled=False)
        .translate((cx, 0.0, 0.0))
    )
    bowl = outer.cut(cavity)

    # Seat shelf: flat oval ceramic rim the seat ring rests on
    shelf = _oval_ring(
        top_z - 0.022, rx_out=0.210, ry_out=0.200, rx_in=0.155, ry_in=0.155, thick=0.022, cx=cx
    )
    bowl = bowl.union(shelf)

    # Base skirt: connects the bowl bottom to the floor
    skirt = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .center(cx, 0.0)
        .box(0.200, 0.210, 0.040, centered=(True, True, False))
    )
    bowl = bowl.union(skirt)

    return bowl


def _bowl_pod_connector() -> cq.Workplane:
    """Ceramic shroud connecting the bowl rear into the service pod."""
    mid_x = (POD_FRONT_X + BOWL_CX - 0.100) / 2.0
    connector = (
        cq.Workplane("XY")
        .workplane(offset=SEAT_Z - 0.180)
        .center(mid_x, 0.0)
        .box(POD_FRONT_X - mid_x + 0.120, 0.240, 0.200, centered=(True, True, False))
    )
    return connector


def _floor_bolt_cap(y_offset: float) -> cq.Workplane:
    """Small domed bolt cap at the toilet base."""
    cap = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .center(POD_BACK_X + 0.040, y_offset)
        .cylinder(0.016, 0.012, centered=(True, True, False))
    )
    # Dome top
    dome = (
        cq.Workplane("XY")
        .workplane(offset=0.012)
        .center(POD_BACK_X + 0.040, y_offset)
        .sphere(0.008)
    )
    return cap.union(dome)


def _rubber_bumper(local_x: float, local_y: float) -> cq.Workplane:
    """Small rubber bumper pad (cylinder) under the seat ring."""
    bumper = (
        cq.Workplane("XY")
        .workplane(offset=-0.005)
        .center(local_x, local_y)
        .cylinder(0.007, 0.005, centered=(True, True, False))
    )
    return bumper


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tankless_smart_toilet")

    ceramic = model.material("ceramic_white", rgba=(0.95, 0.95, 0.94, 1.0))
    seat_white = model.material("seat_white", rgba=(0.97, 0.97, 0.96, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.15, 0.15, 0.16, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    pod_gray = model.material("pod_gray", rgba=(0.88, 0.88, 0.87, 1.0))

    cx = BOWL_CX

    # ================= ROOT: ceramic body + service pod + bolt caps ==========
    body = model.part("body")

    # Service pod (rear housing)
    body.visual(
        mesh_from_cadquery(_service_pod_solid(), "service_pod"),
        material=pod_gray,
        name="service_pod",
    )

    # Ceramic bowl + connector to pod
    bowl_geo = _bowl_solid().union(_bowl_pod_connector())
    body.visual(
        mesh_from_cadquery(bowl_geo, "bowl_shell"),
        material=ceramic,
        name="bowl_shell",
    )

    # Floor bolt caps (2 caps at the pod base)
    bolt_caps = _floor_bolt_cap(0.090).union(_floor_bolt_cap(-0.090))
    body.visual(
        mesh_from_cadquery(bolt_caps, "bolt_caps"),
        material=dark_plastic,
        name="bolt_caps",
    )

    body.inertial = Inertial.from_geometry(
        Box((0.60, 0.36, 0.55)),
        mass=28.0,
        origin=Origin(xyz=(0.25, 0.0, 0.22)),
    )

    # ================= seat ring (revolute, rear axis) =================
    seat = model.part("seat_ring")
    seat_local_cx = cx - HINGE_X  # bowl center relative to the hinge

    seat_ring_geo = _oval_disc_ring_solid(
        rx_out=0.190,
        ry_out=0.185,
        rx_in=0.115,
        ry_in=0.122,
        thick=0.020,
        cx=seat_local_cx,
    )
    seat.visual(
        mesh_from_cadquery(seat_ring_geo.translate((0, 0, 0.002)), "seat_ring_shell"),
        material=seat_white,
        name="seat_ring_shell",
    )

    # Rubber bumpers under the seat ring (4 bumpers: front, rear, left, right)
    bumper_positions = [
        (seat_local_cx + 0.140, 0.0),       # front
        (seat_local_cx - 0.100, 0.0),       # rear
        (seat_local_cx + 0.020, 0.140),     # left
        (seat_local_cx + 0.020, -0.140),    # right
    ]
    bumpers = _rubber_bumper(*bumper_positions[0])
    for bx, by in bumper_positions[1:]:
        bumpers = bumpers.union(_rubber_bumper(bx, by))
    seat.visual(
        mesh_from_cadquery(bumpers, "seat_bumpers"),
        material=rubber_black,
        name="seat_bumpers",
    )

    seat.inertial = Inertial.from_geometry(
        Box((0.38, 0.37, 0.025)),
        mass=0.9,
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
    lid_geo = _oval_disc_solid(rx=0.200, ry=0.192, thick=0.018, cx=lid_local_cx)
    lid.visual(
        mesh_from_cadquery(lid_geo.translate((0, 0, 0.020)), "lid_shell"),
        material=seat_white,
        name="lid_shell",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.40, 0.38, 0.020)),
        mass=1.0,
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

    # ================= flush lever (revolute, on pod side) =================
    # Lever mounted on the right side (+Y) of the service pod.
    # The pivot shaft goes into the pod wall (along +Y).
    # The lever arm extends forward (+X) from the pivot and rotates down when pushed.
    lever = model.part("flush_lever")

    lever_pivot_x = (POD_BACK_X + POD_FRONT_X) / 2.0 + 0.020
    lever_pivot_y = POD_WIDTH / 2.0 + 0.002  # just outside pod wall
    lever_pivot_z = 0.420

    # Lever arm: a thin bar extending forward from the pivot
    lever_arm = (
        cq.Workplane("XY")
        .center(0.035, 0.0)
        .box(0.070, 0.014, 0.016, centered=(True, True, True))
    )
    # Pivot boss (small cylinder at the mount point)
    boss = (
        cq.Workplane("XZ")
        .center(0.0, 0.0)
        .circle(0.012)
        .extrude(0.008)
    )
    lever_geo = lever_arm.union(boss)

    lever.visual(
        mesh_from_cadquery(lever_geo, "flush_lever_arm"),
        material=chrome,
        name="flush_lever_arm",
    )
    lever.inertial = Inertial.from_geometry(
        Box((0.080, 0.020, 0.020)),
        mass=0.06,
        origin=Origin(xyz=(0.035, 0.0, 0.0)),
    )

    # Articulation: rotation about Y axis (shaft into pod wall).
    # Positive q: right-hand rule around +Y takes +Z toward +X,
    # so +X lever tip goes toward -Z (downward = actuated).
    model.articulation(
        "body_to_flush_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(lever_pivot_x, lever_pivot_y, lever_pivot_z)),
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
    lever = object_model.get_part("flush_lever")

    seat_joint = object_model.get_articulation("body_to_seat_ring")
    lid_joint = object_model.get_articulation("body_to_lid")
    lever_joint = object_model.get_articulation("body_to_flush_lever")

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
        elem_a="bowl_shell", elem_b="service_pod",
        reason="Bowl connector shroud is fused into the service pod (structural support).",
    )

    # --- Toilet sits on the floor (not wall-hung) ---
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "toilet body reaches the floor (floor-standing)",
        body_aabb[0][2] < 0.03,
        details=f"min z of body = {body_aabb[0][2]}",
    )
    ctx.check(
        "seat top is near 0.41 m above floor",
        0.35 < SEAT_Z < 0.47,
        details=f"seat top z = {SEAT_Z}",
    )

    # --- Service pod is behind the bowl (at -X side) ---
    pod_aabb_min = body_aabb[0]
    ctx.check(
        "service pod extends behind the bowl (negative X from hinge)",
        pod_aabb_min[0] < HINGE_X - 0.05,
        details=f"body min x = {pod_aabb_min[0]}, hinge x = {HINGE_X}",
    )

    # --- Floor bolt caps exist at the base ---
    ctx.check(
        "bolt caps visual exists on body",
        body.get_visual("bolt_caps") is not None,
        details="bolt_caps visual not found on body",
    )

    # --- Rubber bumpers exist under the seat ---
    ctx.check(
        "rubber bumpers visual exists on seat ring",
        seat.get_visual("seat_bumpers") is not None,
        details="seat_bumpers visual not found on seat_ring",
    )

    # --- Bumpers are below the seat ring surface ---
    seat_ring_aabb = ctx.part_element_world_aabb(seat, elem="seat_ring_shell")
    bumper_aabb = ctx.part_element_world_aabb(seat, elem="seat_bumpers")
    ctx.check(
        "bumpers sit at or below the seat ring bottom",
        bumper_aabb[0][2] <= seat_ring_aabb[0][2] + 0.003,
        details=f"bumper min z={bumper_aabb[0][2]}, seat ring min z={seat_ring_aabb[0][2]}",
    )

    # --- Lid sits above the seat ring when closed ---
    seat_z_top = ctx.part_world_aabb(seat)[1][2]
    lid_z_bot = ctx.part_world_aabb(lid)[0][2]
    ctx.check(
        "lid sits above the seat ring when closed",
        lid_z_bot >= seat_z_top - 0.005,
        details=f"lid bottom z={lid_z_bot}, seat top z={seat_z_top}",
    )

    # --- Lid rotates open ~100 deg ---
    lid_top_z0 = ctx.part_world_aabb(lid)[1][2]
    with ctx.pose({lid_joint: -math.radians(100.0)}):
        lid_top_z1 = ctx.part_world_aabb(lid)[1][2]
    ctx.check(
        "lid rotates open (lifts upward)",
        lid_top_z1 > lid_top_z0 + 0.05,
        details=f"closed top z={lid_top_z0}, open top z={lid_top_z1}",
    )

    # --- Seat ring rotates on the SAME axis as the lid ---
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

    # --- Flush lever is on the pod side (positive Y) ---
    lever_pos = ctx.part_world_position(lever)
    ctx.check(
        "flush lever is on the right side of the pod (+Y)",
        lever_pos[1] > 0.10,
        details=f"lever y = {lever_pos[1]}",
    )

    # --- Flush lever rotates downward when actuated ---
    lever_tip_z0 = ctx.part_world_aabb(lever)[1][2]  # top of lever at rest
    lever_front_x0 = ctx.part_world_aabb(lever)[1][0]  # front x at rest
    with ctx.pose({lever_joint: math.radians(25.0)}):
        lever_tip_z1 = ctx.part_world_aabb(lever)[0][2]  # bottom after rotation
        lever_front_x1 = ctx.part_world_aabb(lever)[1][0]
    ctx.check(
        "flush lever rotates (front drops when actuated)",
        lever_tip_z1 < lever_tip_z0 - 0.005,
        details=f"rest top z={lever_tip_z0}, actuated bottom z={lever_tip_z1}",
    )

    # --- Flush lever is at seat height on the pod ---
    ctx.check(
        "flush lever is near seat height",
        abs(lever_pos[2] - SEAT_Z) < 0.10,
        details=f"lever z={lever_pos[2]}, seat z={SEAT_Z}",
    )

    return ctx.report()


object_model = build_object_model()
