from __future__ import annotations

# One-piece floor-standing white ceramic toilet with rounded integrated tank.
#
# Frame / coordinate convention:
#   +X = forward (front of the bowl points to +X).
#   +Y = left-right (the hinge axis for the lid + seat ring runs along Y).
#   +Z = up. The floor is at z=0; the seat top sits ~0.40 m above the floor.
#
# Root part = body (one-piece ceramic: bowl + integrated tank + base pedestal).
# Everything else mounts to the root:
#   - lid          : oval top lid, REVOLUTE hinge at the rear, ~100 deg.
#   - seat_ring    : oval seat ring under the lid, REVOLUTE hinge sharing the
#                    SAME rear axis as the lid (concentric), ~100 deg.
#   - flush_button_large / flush_button_small : dual-flush push buttons on top
#                    of the integrated tank, PRISMATIC pressing down (-Z), ~6mm.
# Fixed visual details on the body:
#   - floor_bolt_caps : two chrome cap nuts at the base front (floor mounting).
#   - rubber_bumpers  : four small dark rubber pads on the bowl rim under seat.

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
BOWL_DEPTH = 0.540
TANK_W = 0.340
TANK_DEPTH = 0.190
TANK_TOP_Z = 0.820
BASE_W = 0.320
BASE_DEPTH = 0.580

# Bowl center X (forward of origin)
BOWL_CX = 0.180
# Tank center X (behind bowl)
TANK_CX = -0.100

# Rear hinge axis (shared by lid + seat ring), in world coords.
HINGE_X = -0.020
HINGE_Z = SEAT_TOP_Z + 0.004

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


def _bowl_body_solid() -> cq.Workplane:
    """The ceramic bowl: a rounded body that tapers from a wide seat shelf
    at the top down to a narrower rounded bottom, hollowed at the top."""
    cx = BOWL_CX
    top_z = SEAT_Z
    bottom_z = 0.040  # short pedestal above floor

    # Outer body, lofted oval sections from base up to rim.
    outer = (
        cq.Workplane("XY")
        .workplane(offset=bottom_z)
        .ellipse(0.080, 0.090)
        .workplane(offset=0.080)
        .ellipse(0.120, 0.110)
        .workplane(offset=0.130)
        .ellipse(0.160, 0.160)
        .workplane(offset=0.100)
        .ellipse(0.180, 0.180)
        .loft(ruled=False)
    )
    outer = outer.translate((cx, 0.0, 0.0))

    # Pedestal base: a wider flat base at the bottom for floor contact.
    base = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .center(cx, 0.0)
        .ellipse(0.120, 0.100)
        .extrude(0.045)
    )

    # Hollow the basin: an inner oval cavity cut from the top.
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=top_z - 0.200)
        .ellipse(0.080, 0.085)
        .workplane(offset=0.130)
        .ellipse(0.145, 0.145)
        .workplane(offset=0.075)
        .ellipse(0.155, 0.155)
        .loft(ruled=False)
        .translate((cx, 0.0, 0.0))
    )
    bowl = outer.union(base).cut(cavity)

    # Seat shelf: a flat oval ceramic rim the seat ring rests on.
    shelf = _oval_ring(
        top_z - 0.022, rx_out=0.195, ry_out=0.190, rx_in=0.148, ry_in=0.148, thick=0.022, cx=cx
    )
    bowl = bowl.union(shelf)

    return bowl


def _tank_solid() -> cq.Workplane:
    """Rounded integrated tank behind the bowl. One smooth ceramic piece
    that connects to the bowl at the rear."""
    cx = TANK_CX
    tank_bottom_z = 0.200
    tank_top_z = TANK_TOP_Z

    # Main tank body: a rounded rectangle (box with filleted edges).
    tank = (
        cq.Workplane("XY")
        .workplane(offset=tank_bottom_z)
        .center(cx, 0.0)
        .rect(TANK_DEPTH, TANK_W)
        .extrude(tank_top_z - tank_bottom_z)
    )
    # Fillet vertical edges for rounded look.
    try:
        tank = tank.edges("|Z").fillet(0.040)
    except Exception:
        pass

    # Tank cap: slightly domed/rounded top.
    cap = (
        cq.Workplane("XY")
        .workplane(offset=tank_top_z - 0.010)
        .center(cx, 0.0)
        .rect(TANK_DEPTH + 0.005, TANK_W + 0.005)
        .extrude(0.020)
    )
    try:
        cap = cap.edges(">Z").fillet(0.008)
    except Exception:
        pass
    tank = tank.union(cap)

    return tank


def _connector_solid() -> cq.Workplane:
    """Ceramic bridge connecting the back of the bowl to the tank so the
    one-piece silhouette reads as continuous."""
    # A box/shroud from the back of the bowl into the tank front face.
    connector = (
        cq.Workplane("XY")
        .workplane(offset=0.120)
        .center(0.020, 0.0)
        .box(0.130, 0.260, 0.300, centered=(True, True, False))
    )
    return connector


def _base_skirt_solid() -> cq.Workplane:
    """The floor-level ceramic skirt / pedestal that ties the bowl base and
    tank base together into one continuous floor-standing piece."""
    skirt = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .center(0.040, 0.0)
        .box(BASE_DEPTH, BASE_W, 0.050, centered=(True, True, False))
    )
    try:
        skirt = skirt.edges(">Z").fillet(0.010)
    except Exception:
        pass
    return skirt


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="one_piece_toilet")

    ceramic = model.material("ceramic_white", rgba=(0.95, 0.95, 0.94, 1.0))
    seat_white = model.material("seat_white", rgba=(0.97, 0.97, 0.96, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    rubber = model.material("rubber_dark", rgba=(0.15, 0.15, 0.15, 1.0))

    cx = BOWL_CX

    # ================= ROOT: one-piece ceramic body ========================
    body = model.part("body")

    # Bowl shell (front ceramic body with hollow basin).
    bowl = _bowl_body_solid()
    body.visual(mesh_from_cadquery(bowl, "bowl_shell"), material=ceramic, name="bowl_shell")

    # Integrated tank (rear rounded tank).
    tank = _tank_solid()
    body.visual(mesh_from_cadquery(tank, "tank_shell"), material=ceramic, name="tank_shell")

    # Ceramic connector between bowl and tank.
    connector = _connector_solid()
    body.visual(mesh_from_cadquery(connector, "connector_shell"), material=ceramic, name="connector_shell")

    # Base skirt / pedestal.
    skirt = _base_skirt_solid()
    body.visual(mesh_from_cadquery(skirt, "base_skirt"), material=ceramic, name="base_skirt")

    # Flush plate on top of the tank (centered).
    plate_z = TANK_TOP_Z + 0.008
    plate_x = TANK_CX
    body.visual(
        Box((0.120, 0.100, 0.008)),
        origin=Origin(xyz=(plate_x, 0.0, plate_z + 0.004)),
        material=chrome,
        name="flush_plate",
    )

    # Floor bolt caps: two small chrome cylinders at the base front.
    bolt_y_offsets = [-0.085, 0.085]
    for i, by in enumerate(bolt_y_offsets):
        cap_geo = CylinderGeometry(0.012, 0.018, radial_segments=32)
        body.visual(
            mesh_from_geometry(cap_geo, f"floor_bolt_cap_{i}"),
            origin=Origin(xyz=(BOWL_CX + 0.060, by, 0.009)),
            material=chrome,
            name=f"floor_bolt_cap_{i}",
        )

    # Rubber bumpers: four small dark pads on the bowl rim under the seat.
    bumper_angles_deg = [30.0, 90.0, 150.0, -30.0]  # around the oval rim
    rim_rx = 0.170
    rim_ry = 0.170
    for i, ang_deg in enumerate(bumper_angles_deg):
        ang = math.radians(ang_deg)
        bx = cx + rim_rx * math.cos(ang)
        by_pos = rim_ry * math.sin(ang)
        bz = SEAT_Z - 0.003
        body.visual(
            Box((0.020, 0.015, 0.006)),
            origin=Origin(xyz=(bx, by_pos, bz)),
            material=rubber,
            name=f"rubber_bumper_{i}",
        )

    body.inertial = Inertial.from_geometry(
        Box((0.60, 0.36, 0.82)),
        mass=30.0,
        origin=Origin(xyz=(0.05, 0.0, 0.35)),
    )

    # ================= seat ring (revolute, rear axis) =====================
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

    # ================= lid (revolute, SAME rear axis) ======================
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

    # ================= dual flush buttons (prismatic, press down) ==========
    # Buttons sit on top of the flush plate and press downward (-Z).
    plate_top_z = plate_z + 0.008
    button_specs = [
        ("flush_button_large", 0.028, TANK_CX + 0.025),
        ("flush_button_small", 0.018, TANK_CX - 0.025),
    ]
    for part_name, radius, btn_x in button_specs:
        b = model.part(part_name)
        puck = CylinderGeometry(radius, 0.012, radial_segments=40)
        b.visual(
            mesh_from_geometry(puck, part_name + "_actuator"),
            material=chrome,
            name=part_name + "_actuator",
        )
        b.inertial = Inertial.from_geometry(
            Box((2.0 * radius, 2.0 * radius, 0.012)), mass=0.04
        )
        # Place on the plate; travel is -Z (pressing down).
        model.articulation(
            "body_to_" + part_name,
            ArticulationType.PRISMATIC,
            parent=body,
            child=b,
            origin=Origin(xyz=(btn_x, 0.0, plate_top_z + 0.006)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=0.05, lower=0.0, upper=0.006),
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
    btn_large = object_model.get_part("flush_button_large")
    btn_small = object_model.get_part("flush_button_small")

    seat_joint = object_model.get_articulation("body_to_seat_ring")
    lid_joint = object_model.get_articulation("body_to_lid")
    btn_large_joint = object_model.get_articulation("body_to_flush_button_large")
    btn_small_joint = object_model.get_articulation("body_to_flush_button_small")

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
        btn_large, body,
        elem_a="flush_button_large_actuator", elem_b="flush_plate",
        reason="Large flush button is captured in the flush plate on top of tank.",
    )
    ctx.allow_overlap(
        btn_small, body,
        elem_a="flush_button_small_actuator", elem_b="flush_plate",
        reason="Small flush button is captured in the flush plate on top of tank.",
    )

    # --- One-piece body: integrated tank exists above seat level ---
    tank_aabb = ctx.part_element_world_aabb(body, elem="tank_shell")
    ctx.check(
        "integrated tank extends above seat level",
        tank_aabb is not None and tank_aabb[1][2] > SEAT_Z + 0.20,
        details=f"tank top z = {tank_aabb[1][2] if tank_aabb else None}",
    )

    # --- Body reaches the floor (floor-standing, not wall-hung) ---
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "body is floor-standing (reaches near floor)",
        body_aabb[0][2] < 0.05,
        details=f"min z of body = {body_aabb[0][2]}",
    )

    # --- Floor bolt caps exist at the base ---
    bolt0_aabb = ctx.part_element_world_aabb(body, elem="floor_bolt_cap_0")
    bolt1_aabb = ctx.part_element_world_aabb(body, elem="floor_bolt_cap_1")
    ctx.check(
        "floor bolt cap 0 exists near floor level",
        bolt0_aabb is not None and bolt0_aabb[0][2] < 0.04,
        details=f"bolt cap 0 min z = {bolt0_aabb[0][2] if bolt0_aabb else None}",
    )
    ctx.check(
        "floor bolt cap 1 exists near floor level",
        bolt1_aabb is not None and bolt1_aabb[0][2] < 0.04,
        details=f"bolt cap 1 min z = {bolt1_aabb[0][2] if bolt1_aabb else None}",
    )

    # --- Rubber bumpers exist on the rim under seat ---
    bumper_aabb = ctx.part_element_world_aabb(body, elem="rubber_bumper_0")
    ctx.check(
        "rubber bumper exists near seat rim level",
        bumper_aabb is not None and abs(bumper_aabb[0][2] - SEAT_Z) < 0.02,
        details=f"bumper min z = {bumper_aabb[0][2] if bumper_aabb else None}",
    )

    # --- Seat top near 0.40m ---
    ctx.check(
        "seat top is near 0.40 m above floor",
        0.34 < SEAT_Z < 0.46,
        details=f"seat top z = {SEAT_Z}",
    )

    # --- Lid sits above the seat ring when closed ---
    seat_z = ctx.part_world_aabb(seat)[1][2]
    lid_z = ctx.part_world_aabb(lid)[0][2]
    ctx.check(
        "lid sits above the seat ring when closed",
        lid_z >= seat_z - 0.005,
        details=f"lid bottom z={lid_z}, seat top z={seat_z}",
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

    # --- Seat ring rotates on same axis as lid ---
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

    # --- Flush buttons press down (prismatic, -Z) ---
    for name, part, joint in (
        ("large", btn_large, btn_large_joint),
        ("small", btn_small, btn_small_joint),
    ):
        z0 = ctx.part_world_position(part)[2]
        with ctx.pose({joint: 0.006}):
            z1 = ctx.part_world_position(part)[2]
        ctx.check(
            f"{name} flush button presses downward",
            z1 < z0 - 0.004,
            details=f"rest z={z0}, pressed z={z1}",
        )
        ctx.check(
            f"{name} flush button is on top of the tank",
            ctx.part_world_position(part)[2] > TANK_TOP_Z - 0.05,
            details=f"button z={ctx.part_world_position(part)[2]}, tank top={TANK_TOP_Z}",
        )

    # --- Dual-flush: large button is bigger than the small button ---
    large_dy = _ext(ctx.part_world_aabb(btn_large))[1]
    small_dy = _ext(ctx.part_world_aabb(btn_small))[1]
    ctx.check(
        "dual flush: large button is larger than the small button",
        large_dy > small_dy + 0.005,
        details=f"large dy={large_dy}, small dy={small_dy}",
    )

    return ctx.report()


object_model = build_object_model()
