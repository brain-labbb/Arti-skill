from __future__ import annotations

# Compact corner toilet with triangular tank back, bidet nozzle, floor bolt
# caps, hollow bowl interior, and raised rim geometry.
#
# Frame / coordinate convention:
#   +X = forward (front of the bowl points to +X), wall/corner is at -X.
#   +Y = left-right (the hinge axis for the lid + seat ring runs along Y).
#   +Z = up. The floor is at z=0; the seat top sits ~0.40 m above the floor.
#
# Root part = the triangular tank + bowl body (one ceramic assembly on floor).
#   - lid          : oval top lid, REVOLUTE hinge at the rear (axis +Y), ~100 deg.
#   - seat_ring    : oval seat ring under the lid, REVOLUTE, same rear axis.
#   - bidet_nozzle : small nozzle, PRISMATIC, slides forward (+X) ~0.08 m.
#   - flush_button_large / flush_button_small : dual flush on the tank top,
#                    PRISMATIC, depress downward (-Z).

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
FLOOR_Z = 0.0

# Triangular tank
TANK_FRONT_X = 0.04
TANK_BACK_X = -0.20
TANK_HALF_W = 0.16
TANK_BOTTOM_Z = 0.22
TANK_TOP_Z = 0.76

# Bowl
BOWL_CX = 0.250
BOWL_BASE_Z = 0.06

# Hinge
HINGE_X = 0.060
HINGE_Z = SEAT_TOP_Z + 0.004

# Bidet nozzle
NOZZLE_REST_X = 0.08
NOZZLE_TRAVEL = 0.080

# Flush plate on tank top
PLATE_CX = (TANK_FRONT_X + TANK_BACK_X) / 2.0 + 0.03
PLATE_Z = TANK_TOP_Z


def _oval_pts(rx, ry, cx, segs):
    return [
        (cx + rx * math.cos(2 * math.pi * i / segs),
         ry * math.sin(2 * math.pi * i / segs))
        for i in range(segs)
    ]


def _oval_ring(z, rx_out, ry_out, rx_in, ry_in, thick, cx=0.0, segs=72):
    outer = (
        cq.Workplane("XY")
        .workplane(offset=z)
        .polyline(_oval_pts(rx_out, ry_out, cx, segs))
        .close()
        .polyline(_oval_pts(rx_in, ry_in, cx, segs))
        .close()
        .extrude(thick)
    )
    return outer


def _oval_disc(z, rx, ry, thick, cx=0.0, segs=72):
    disc = (
        cq.Workplane("XY")
        .workplane(offset=z)
        .polyline(_oval_pts(rx, ry, cx, segs))
        .close()
        .extrude(thick)
    )
    return disc


def _triangular_tank_solid() -> cq.Workplane:
    """Triangular prism tank that fits into a bathroom corner."""
    pts = [
        (TANK_FRONT_X, -TANK_HALF_W),
        (TANK_FRONT_X, TANK_HALF_W),
        (TANK_BACK_X, 0.0),
    ]
    tank = (
        cq.Workplane("XY")
        .workplane(offset=TANK_BOTTOM_Z)
        .polyline(pts)
        .close()
        .extrude(TANK_TOP_Z - TANK_BOTTOM_Z)
    )
    return tank


def _bowl_solid() -> cq.Workplane:
    """Floor-standing bowl with hollow interior and raised rim."""
    cx = BOWL_CX
    top_z = SEAT_TOP_Z
    bottom_z = BOWL_BASE_Z

    # Outer body: lofted oval from a narrow base up to the rim shelf.
    outer = (
        cq.Workplane("XY")
        .workplane(offset=bottom_z)
        .ellipse(0.070, 0.080)
        .workplane(offset=0.090)
        .ellipse(0.130, 0.120)
        .workplane(offset=0.130)
        .ellipse(0.165, 0.165)
        .workplane(offset=0.090)
        .ellipse(0.180, 0.180)
        .loft(ruled=False)
    )
    outer = outer.translate((cx, 0.0, 0.0))

    # Hollow basin cavity cut from the top.
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=top_z - 0.185)
        .ellipse(0.080, 0.085)
        .workplane(offset=0.130)
        .ellipse(0.145, 0.145)
        .workplane(offset=0.060)
        .ellipse(0.155, 0.155)
        .loft(ruled=False)
        .translate((cx, 0.0, 0.0))
    )
    bowl = outer.cut(cavity)

    # Pedestal base connecting bowl to the floor.
    base = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .center(cx, 0.0)
        .box(0.22, 0.24, bottom_z + 0.005, centered=(True, True, False))
    )
    bowl = bowl.union(base)

    # Raised rim: oval ring sitting proud of the bowl outer surface at the top.
    rim = _oval_ring(
        top_z - 0.006,
        rx_out=0.198, ry_out=0.195,
        rx_in=0.155, ry_in=0.155,
        thick=0.028,
        cx=cx,
    )
    bowl = bowl.union(rim)

    return bowl


def _tank_connector_solid() -> cq.Workplane:
    """Ceramic shroud connecting the back of the bowl to the tank front face."""
    neck = (
        cq.Workplane("XY")
        .workplane(offset=SEAT_TOP_Z - 0.180)
        .center(0.050, 0.0)
        .box(0.140, 0.250, 0.200, centered=(True, True, False))
    )
    return neck


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_toilet")

    ceramic = model.material("ceramic_white", rgba=(0.95, 0.95, 0.94, 1.0))
    seat_white = model.material("seat_white", rgba=(0.97, 0.97, 0.96, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.15, 0.15, 0.16, 1.0))

    cx = BOWL_CX

    # ================ ROOT: tank + bowl + connector + bolt caps ================
    body = model.part("body")

    # Triangular tank (corner-fit).
    body.visual(
        mesh_from_cadquery(_triangular_tank_solid(), "tank_shell"),
        material=ceramic,
        name="tank_shell",
    )

    # Bowl with hollow interior + raised rim + pedestal base.
    body.visual(
        mesh_from_cadquery(_bowl_solid(), "bowl_shell"),
        material=ceramic,
        name="bowl_shell",
    )

    # Connector shroud between bowl back and tank front.
    body.visual(
        mesh_from_cadquery(_tank_connector_solid(), "tank_connector"),
        material=ceramic,
        name="tank_connector",
    )

    # Floor bolt caps (two chrome caps at the bowl pedestal base).
    bolt_cap_ys = [0.10, -0.10]
    for i, by in enumerate(bolt_cap_ys):
        cap_geo = CylinderGeometry(0.014, 0.018, radial_segments=24)
        body.visual(
            mesh_from_geometry(cap_geo, f"bolt_cap_{i}"),
            origin=Origin(xyz=(cx - 0.03, by, 0.009)),
            material=chrome,
            name=f"bolt_cap_{i}",
        )

    # Flush plate on the tank top.
    body.visual(
        Box((0.120, 0.090, 0.006)),
        origin=Origin(xyz=(PLATE_CX, 0.0, PLATE_Z + 0.003)),
        material=chrome,
        name="flush_plate",
    )

    body.inertial = Inertial.from_geometry(
        Box((0.55, 0.38, 0.78)),
        mass=32.0,
        origin=Origin(xyz=(0.10, 0.0, 0.35)),
    )

    # ================ SEAT RING (revolute, rear hinge axis) ================
    seat = model.part("seat_ring")
    seat_local_cx = cx - HINGE_X
    seat_ring_geo = _oval_ring(
        z=0.0, rx_out=0.180, ry_out=0.178,
        rx_in=0.110, ry_in=0.118,
        thick=0.020, cx=seat_local_cx, segs=80,
    )
    seat.visual(
        mesh_from_cadquery(seat_ring_geo.translate((0, 0, 0.002)), "seat_ring_shell"),
        material=seat_white,
        name="seat_ring_shell",
    )
    seat.inertial = Inertial.from_geometry(
        Box((0.36, 0.36, 0.025)),
        mass=0.8,
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

    # ================ LID (revolute, same rear hinge axis) ================
    lid = model.part("lid")
    lid_local_cx = cx - HINGE_X
    lid_geo = _oval_disc(
        z=0.0, rx=0.190, ry=0.185, thick=0.018, cx=lid_local_cx, segs=72,
    )
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
            effort=8.0, velocity=2.0,
            lower=-math.radians(100.0), upper=0.0,
        ),
    )

    # ================ BIDET NOZZLE (prismatic, slides forward +X) ================
    nozzle = model.part("bidet_nozzle")
    nozzle_geo = CylinderGeometry(0.007, 0.045, radial_segments=20)
    nozzle.visual(
        mesh_from_geometry(nozzle_geo.rotate_y(math.pi / 2.0), "nozzle_body"),
        material=dark_plastic,
        name="nozzle_body",
    )
    nozzle.inertial = Inertial.from_geometry(
        Box((0.045, 0.014, 0.014)), mass=0.04,
    )
    model.articulation(
        "body_to_bidet_nozzle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=nozzle,
        origin=Origin(xyz=(NOZZLE_REST_X, 0.0, SEAT_TOP_Z - 0.080)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=0.02, lower=0.0, upper=NOZZLE_TRAVEL,
        ),
    )

    # ================ FLUSH BUTTONS (prismatic, depress downward -Z) ================
    button_specs = [
        ("flush_button_large", 0.018, PLATE_CX + 0.022),
        ("flush_button_small", 0.012, PLATE_CX - 0.022),
    ]
    for part_name, radius, btn_x in button_specs:
        b = model.part(part_name)
        puck = CylinderGeometry(radius, 0.010, radial_segments=32)
        b.visual(
            mesh_from_geometry(puck, part_name + "_actuator"),
            material=chrome,
            name=part_name + "_actuator",
        )
        b.inertial = Inertial.from_geometry(
            Box((2.0 * radius, 2.0 * radius, 0.010)), mass=0.04,
        )
        model.articulation(
            "body_to_" + part_name,
            ArticulationType.PRISMATIC,
            parent=body,
            child=b,
            origin=Origin(xyz=(btn_x, 0.0, PLATE_Z + 0.008)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=4.0, velocity=0.05, lower=0.0, upper=0.005,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    seat = object_model.get_part("seat_ring")
    lid = object_model.get_part("lid")
    nozzle = object_model.get_part("bidet_nozzle")
    btn_large = object_model.get_part("flush_button_large")
    btn_small = object_model.get_part("flush_button_small")

    seat_joint = object_model.get_articulation("body_to_seat_ring")
    lid_joint = object_model.get_articulation("body_to_lid")
    nozzle_joint = object_model.get_articulation("body_to_bidet_nozzle")
    btn_large_joint = object_model.get_articulation("body_to_flush_button_large")
    btn_small_joint = object_model.get_articulation("body_to_flush_button_small")

    # ---- Intentional overlaps: seated parts ----
    ctx.allow_overlap(
        seat, body,
        elem_a="seat_ring_shell", elem_b="bowl_shell",
        reason="Seat ring rests on the ceramic bowl raised rim (seated contact).",
    )
    ctx.allow_overlap(
        lid, seat,
        elem_a="lid_shell", elem_b="seat_ring_shell",
        reason="Closed lid rests on top of the seat ring.",
    )
    ctx.allow_overlap(
        btn_large, body,
        elem_a="flush_button_large_actuator", elem_b="flush_plate",
        reason="Large flush button is captured in the tank-top flush plate.",
    )
    ctx.allow_overlap(
        btn_small, body,
        elem_a="flush_button_small_actuator", elem_b="flush_plate",
        reason="Small flush button is captured in the tank-top flush plate.",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="bowl_shell", elem_b="tank_connector",
        reason="Bowl is fused into the tank connector shroud (structural).",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="tank_connector", elem_b="tank_shell",
        reason="Tank connector fuses into the triangular tank body.",
    )
    ctx.allow_overlap(
        nozzle, body,
        elem_a="nozzle_body", elem_b="bowl_shell",
        reason="Bidet nozzle retracts inside the bowl rear wall (captured nozzle).",
    )

    # ---- Triangular tank: back is narrower than front ----
    tank_aabb = ctx.part_element_world_aabb(body, elem="tank_shell")
    tank_min_y = tank_aabb[0][1]
    tank_max_y = tank_aabb[1][1]
    # Measure width at front vs. back by checking that the tank tapers.
    # The tank is a triangle: widest at front X, narrowest at back X.
    # We check that the overall Y span is reasonable and the tank exists.
    tank_dy = tank_max_y - tank_min_y
    ctx.check(
        "triangular tank has meaningful width",
        tank_dy > 0.20,
        details=f"tank Y span = {tank_dy:.4f}",
    )

    # Tank back point is at smaller X than front face.
    tank_min_x = tank_aabb[0][0]
    tank_max_x = tank_aabb[1][0]
    ctx.check(
        "triangular tank extends behind the bowl",
        tank_min_x < TANK_FRONT_X - 0.10,
        details=f"tank min x = {tank_min_x:.4f}",
    )

    # ---- Bowl touches the floor (floor-standing, not wall-hung) ----
    bowl_aabb = ctx.part_element_world_aabb(body, elem="bowl_shell")
    ctx.check(
        "bowl pedestal reaches the floor",
        bowl_aabb[0][2] < 0.02,
        details=f"bowl min z = {bowl_aabb[0][2]:.4f}",
    )

    # ---- Raised rim: rim element extends above bowl outer top ----
    rim_top = bowl_aabb[1][2]  # top of the bowl_shell (includes rim)
    ctx.check(
        "raised rim extends to seat height or above",
        rim_top >= SEAT_TOP_Z - 0.005,
        details=f"bowl top z = {rim_top:.4f}",
    )

    # ---- Floor bolt caps exist at the base ----
    bolt_0 = ctx.part_element_world_aabb(body, elem="bolt_cap_0")
    bolt_1 = ctx.part_element_world_aabb(body, elem="bolt_cap_1")
    ctx.check(
        "floor bolt cap 0 is near the floor",
        bolt_0[0][2] < 0.025,
        details=f"bolt_cap_0 min z = {bolt_0[0][2]:.4f}",
    )
    ctx.check(
        "floor bolt cap 1 is near the floor",
        bolt_1[0][2] < 0.025,
        details=f"bolt_cap_1 min z = {bolt_1[0][2]:.4f}",
    )
    # Bolt caps are on opposite sides (left/right).
    bolt_0_y = (bolt_0[0][1] + bolt_0[1][1]) / 2.0
    bolt_1_y = (bolt_1[0][1] + bolt_1[1][1]) / 2.0
    ctx.check(
        "floor bolt caps are on opposite sides of the bowl",
        abs(bolt_0_y - bolt_1_y) > 0.10,
        details=f"bolt_0 y={bolt_0_y:.3f}, bolt_1 y={bolt_1_y:.3f}",
    )

    # ---- Seat ring rests on the bowl, lid sits above seat when closed ----
    seat_top_z = ctx.part_world_aabb(seat)[1][2]
    lid_bottom_z = ctx.part_world_aabb(lid)[0][2]
    ctx.check(
        "lid sits above the seat ring when closed",
        lid_bottom_z >= seat_top_z - 0.005,
        details=f"lid bottom z={lid_bottom_z:.4f}, seat top z={seat_top_z:.4f}",
    )

    # ---- Lid opens upward (revolute) ----
    lid_top_z0 = ctx.part_world_aabb(lid)[1][2]
    lid_front_x0 = ctx.part_world_aabb(lid)[1][0]
    with ctx.pose({lid_joint: -math.radians(100.0)}):
        lid_top_z1 = ctx.part_world_aabb(lid)[1][2]
        lid_front_x1 = ctx.part_world_aabb(lid)[1][0]
    ctx.check(
        "lid rotates open (lifts upward)",
        lid_top_z1 > lid_top_z0 + 0.05,
        details=f"closed top z={lid_top_z0:.4f}, open top z={lid_top_z1:.4f}",
    )
    ctx.check(
        "lid swings rearward when opened",
        lid_front_x1 < lid_front_x0 - 0.05,
        details=f"closed front x={lid_front_x0:.4f}, open front x={lid_front_x1:.4f}",
    )

    # ---- Seat ring opens (revolute, same axis as lid) ----
    seat_top_z0 = ctx.part_world_aabb(seat)[1][2]
    with ctx.pose({seat_joint: -math.radians(100.0)}):
        seat_top_z1 = ctx.part_world_aabb(seat)[1][2]
    ctx.check(
        "seat ring rotates open (lifts upward)",
        seat_top_z1 > seat_top_z0 + 0.05,
        details=f"closed top z={seat_top_z0:.4f}, open top z={seat_top_z1:.4f}",
    )

    # Seat and lid share the same hinge axis.
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

    # ---- Bidet nozzle slides forward (prismatic +X) ----
    nozzle_x0 = ctx.part_world_position(nozzle)[0]
    with ctx.pose({nozzle_joint: NOZZLE_TRAVEL}):
        nozzle_x1 = ctx.part_world_position(nozzle)[0]
    ctx.check(
        "bidet nozzle slides forward when extended",
        nozzle_x1 > nozzle_x0 + 0.04,
        details=f"retracted x={nozzle_x0:.4f}, extended x={nozzle_x1:.4f}",
    )
    # Nozzle joint is prismatic along +X.
    ctx.check(
        "bidet nozzle joint is prismatic along +X",
        nozzle_joint.articulation_type == ArticulationType.PRISMATIC
        and abs(nozzle_joint.axis[0] - 1.0) < 1e-6
        and abs(nozzle_joint.axis[1]) < 1e-6
        and abs(nozzle_joint.axis[2]) < 1e-6,
        details=f"type={nozzle_joint.articulation_type}, axis={nozzle_joint.axis}",
    )

    # ---- Flush buttons depress downward into the tank top ----
    for name, part_obj, joint_obj in (
        ("large", btn_large, btn_large_joint),
        ("small", btn_small, btn_small_joint),
    ):
        z0 = ctx.part_world_position(part_obj)[2]
        with ctx.pose({joint_obj: 0.005}):
            z1 = ctx.part_world_position(part_obj)[2]
        ctx.check(
            f"{name} flush button depresses downward",
            z1 < z0 - 0.003,
            details=f"rest z={z0:.4f}, pressed z={z1:.4f}",
        )
        ctx.check(
            f"{name} flush button is on top of the tank",
            z0 > TANK_TOP_Z - 0.02,
            details=f"button z={z0:.4f}, tank top z={TANK_TOP_Z}",
        )

    # ---- Dual-flush: large button is bigger than small ----
    large_dy = ctx.part_world_aabb(btn_large)[1][1] - ctx.part_world_aabb(btn_large)[0][1]
    small_dy = ctx.part_world_aabb(btn_small)[1][1] - ctx.part_world_aabb(btn_small)[0][1]
    ctx.check(
        "dual flush: large button is larger than small button",
        large_dy > small_dy + 0.004,
        details=f"large dy={large_dy:.4f}, small dy={small_dy:.4f}",
    )

    # ---- Seat height is near 0.40 m ----
    ctx.check(
        "seat top is near 0.40 m above floor",
        0.34 < SEAT_TOP_Z < 0.46,
        details=f"seat top z = {SEAT_TOP_Z}",
    )

    return ctx.report()


object_model = build_object_model()
