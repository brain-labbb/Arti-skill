from __future__ import annotations

# Child-size training toilet (floor-standing, white ceramic).
#
# Frame / coordinate convention:
#   +X = forward (front of the bowl points to +X), tank/wall at -X.
#   +Y = left-right (the hinge axis for the lid + seat ring runs along Y).
#   +Z = up. The floor is at z=0; the seat top sits ~0.28 m above the floor.
#
# Root part = one-piece ceramic body: pedestal base + bowl + tank.
# Articulated parts:
#   - lid          : oval top lid, REVOLUTE hinge at the rear (axis +Y), ~100 deg.
#   - seat_ring    : oval seat ring under the lid, REVOLUTE hinge sharing the
#                    SAME rear axis as the lid (concentric), ~100 deg.
#   - flush_button : single round chrome push button on the tank top,
#                    PRISMATIC ~5 mm downward (-Z travel).
# Static features on root:
#   - Floor bolt caps (two) at the front base, covering mounting hardware.

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

# ---- key dimensions (meters) ---- child-size training toilet
SEAT_TOP_Z = 0.280  # top of the seat ring above the floor (child height)
BOWL_W = 0.280  # bowl width (Y)
BOWL_DEPTH = 0.400  # bowl depth (X), front lip to tank
BASE_W = 0.300  # pedestal base width (Y)
BASE_DEPTH = 0.420  # pedestal base depth (X)
TANK_W = 0.260  # tank width (Y)
TANK_DEPTH = 0.140  # tank depth (X)
TANK_HEIGHT = 0.220  # tank height above seat level

# Bowl center X (measured from origin at back of base)
BOWL_CX = 0.220

# Rear hinge axis (shared by lid + seat ring), in world coords.
HINGE_X = 0.040
HINGE_Z = SEAT_TOP_Z + 0.004

# Seat plate top surface z (where seat ring + lid live).
SEAT_Z = SEAT_TOP_Z

# Base bottom at floor
BASE_BOTTOM_Z = 0.0


def _oval_loft(z_bottom, z_top, rx, ry, taper=0.85, segs=64) -> cq.Workplane:
    """Vertical oval loft (ellipse in XY) between two z levels, tapering."""
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
    """Flat oval ring (annular ellipse) of given thickness."""
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
    """The ceramic bowl: rounded body tapering from seat shelf to base, hollowed."""
    cx = BOWL_CX
    top_z = SEAT_Z
    bottom_z = SEAT_Z - 0.200  # shallower bowl for child size

    # Outer body: lofted oval sections from rounded bottom up to rim.
    outer = (
        cq.Workplane("XY")
        .workplane(offset=bottom_z)
        .ellipse(0.060, 0.065)
        .workplane(offset=0.060)
        .ellipse(0.100, 0.095)
        .workplane(offset=0.080)
        .ellipse(0.130, 0.130)
        .workplane(offset=0.060)
        .ellipse(0.140, 0.140)
        .loft(ruled=False)
    )
    outer = outer.translate((cx, 0.0, 0.0))

    # Hollow the basin.
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=top_z - 0.140)
        .ellipse(0.065, 0.070)
        .workplane(offset=0.090)
        .ellipse(0.110, 0.110)
        .workplane(offset=0.055)
        .ellipse(0.118, 0.118)
        .loft(ruled=False)
        .translate((cx, 0.0, 0.0))
    )
    bowl = outer.cut(cavity)

    # Seat shelf: flat oval ceramic rim at the top of the bowl.
    shelf = _oval_ring(
        top_z - 0.018, rx_out=0.155, ry_out=0.150, rx_in=0.110, ry_in=0.110, thick=0.018, cx=cx
    )
    bowl = bowl.union(shelf)

    return bowl


def _pedestal_solid() -> cq.Workplane:
    """Pedestal base: tapered column from floor up to bowl support height."""
    cx = BOWL_CX
    # Base footprint is wider at the floor, narrower at the top.
    # Use a loft from floor rectangle to top oval.
    base_bottom = 0.0
    base_top = SEAT_Z - 0.190  # just below bowl bottom

    # Floor footprint: rounded rectangle approximation using a box + fillets
    # Simpler: use a box for the base and blend with the bowl
    pedestal = (
        cq.Workplane("XY")
        .workplane(offset=base_bottom)
        .center(cx, 0.0)
        .box(BASE_DEPTH, BASE_W, base_top - base_bottom, centered=(True, True, False))
    )

    # Round the front edges slightly for a child-friendly look
    # Add a slight taper by unioning with a smaller top box
    top_cap = (
        cq.Workplane("XY")
        .workplane(offset=base_top - 0.020)
        .center(cx, 0.0)
        .box(BASE_DEPTH * 0.85, BASE_W * 0.90, 0.020, centered=(True, True, False))
    )
    pedestal = pedestal.union(top_cap)

    # Front skirt extension: a rounded front face
    front_skirt = (
        cq.Workplane("XY")
        .workplane(offset=base_bottom)
        .center(cx + BASE_DEPTH * 0.42, 0.0)
        .box(0.040, BASE_W * 0.80, base_top - base_bottom - 0.030, centered=(True, True, False))
    )
    pedestal = pedestal.union(front_skirt)

    return pedestal


def _tank_solid() -> cq.Workplane:
    """Rear cistern/tank: rectangular with rounded top, behind the bowl."""
    tank_x = 0.0 + TANK_DEPTH / 2.0  # centered near x=0
    tank_bottom = SEAT_Z - 0.040  # tank starts just below seat level
    tank_top = tank_bottom + TANK_HEIGHT

    tank = (
        cq.Workplane("XY")
        .workplane(offset=tank_bottom)
        .center(tank_x, 0.0)
        .box(TANK_DEPTH, TANK_W, TANK_HEIGHT, centered=(True, True, False))
    )

    # Rounded cap on top
    cap = (
        cq.Workplane("XY")
        .workplane(offset=tank_top - 0.015)
        .center(tank_x, 0.0)
        .box(TANK_DEPTH + 0.010, TANK_W + 0.010, 0.015, centered=(True, True, False))
    )
    tank = tank.union(cap)

    return tank


def _neck_solid() -> cq.Workplane:
    """Ceramic neck connecting back of bowl into the tank."""
    neck = (
        cq.Workplane("XY")
        .workplane(offset=SEAT_Z - 0.140)
        .center(0.080, 0.0)
        .box(0.100, 0.200, 0.150, centered=(True, True, False))
    )
    return neck


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


def _bolt_cap_solid() -> cq.Workplane:
    """Small domed bolt cap that covers a floor mounting bolt."""
    cap = (
        cq.Workplane("XY")
        .circle(0.018)
        .extrude(0.008)
    )
    # Add a dome top
    dome = (
        cq.Workplane("XY")
        .workplane(offset=0.008)
        .circle(0.018)
        .workplane(offset=0.010)
        .circle(0.002)
        .loft()
    )
    cap = cap.union(dome)
    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="child_training_toilet")

    ceramic = model.material("ceramic_white", rgba=(0.95, 0.95, 0.94, 1.0))
    seat_white = model.material("seat_white", rgba=(0.97, 0.97, 0.96, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    cap_white = model.material("cap_white", rgba=(0.92, 0.92, 0.90, 1.0))

    cx = BOWL_CX

    # ================= ROOT: pedestal + bowl + tank =================
    body = model.part("body")

    # Pedestal base (reaches the floor).
    pedestal = _pedestal_solid()
    body.visual(
        mesh_from_cadquery(pedestal, "pedestal"),
        material=ceramic,
        name="pedestal",
    )

    # Bowl + connecting neck.
    bowl = _bowl_solid().union(_neck_solid())
    body.visual(mesh_from_cadquery(bowl, "bowl_shell"), material=ceramic, name="bowl_shell")

    # Tank/cistern at the back.
    tank = _tank_solid()
    body.visual(mesh_from_cadquery(tank, "tank"), material=ceramic, name="tank")

    # Floor bolt caps at the front base (two caps, left and right).
    bolt_cap_y_offset = 0.090
    bolt_cap_x = cx + BASE_DEPTH * 0.38
    bolt_cap_z = 0.0

    cap_geo = _bolt_cap_solid()
    body.visual(
        mesh_from_cadquery(cap_geo.translate((bolt_cap_x, bolt_cap_y_offset, bolt_cap_z)), "bolt_cap_right"),
        material=cap_white,
        name="bolt_cap_right",
    )
    body.visual(
        mesh_from_cadquery(cap_geo.translate((bolt_cap_x, -bolt_cap_y_offset, bolt_cap_z)), "bolt_cap_left"),
        material=cap_white,
        name="bolt_cap_left",
    )

    body.inertial = Inertial.from_geometry(
        Box((0.45, 0.30, 0.50)),
        mass=12.0,
        origin=Origin(xyz=(0.18, 0.0, SEAT_Z - 0.10)),
    )

    # ================= seat ring (revolute, rear axis) =================
    seat = model.part("seat_ring")
    seat_local_cx = cx - HINGE_X
    seat_ring_geo = _oval_disc_ring_solid(
        rx_out=0.140,
        ry_out=0.138,
        rx_in=0.080,
        ry_in=0.088,
        thick=0.016,
        cx=seat_local_cx,
    )
    seat.visual(
        mesh_from_cadquery(seat_ring_geo.translate((0, 0, 0.002)), "seat_ring_shell"),
        material=seat_white,
        name="seat_ring_shell",
    )
    seat.inertial = Inertial.from_geometry(
        Box((0.28, 0.28, 0.020)),
        mass=0.5,
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
            effort=5.0, velocity=2.0, lower=-math.radians(100.0), upper=0.0
        ),
    )

    # ================= lid (revolute, SAME rear axis) =================
    lid = model.part("lid")
    lid_local_cx = cx - HINGE_X
    lid_geo = _oval_disc_solid(rx=0.148, ry=0.145, thick=0.014, cx=lid_local_cx)
    lid.visual(
        mesh_from_cadquery(lid_geo.translate((0, 0, 0.018)), "lid_shell"),
        material=seat_white,
        name="lid_shell",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.30, 0.29, 0.016)),
        mass=0.5,
        origin=Origin(xyz=(lid_local_cx, 0.0, 0.025)),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=-math.radians(100.0), upper=0.0
        ),
    )

    # ================= flush button (prismatic, downward into tank) =========
    flush_btn = model.part("flush_button")
    btn_radius = 0.020
    puck = CylinderGeometry(btn_radius, 0.012, radial_segments=40)
    flush_btn.visual(
        mesh_from_geometry(puck, "flush_button_actuator"),
        material=chrome,
        name="flush_button_actuator",
    )
    flush_btn.inertial = Inertial.from_geometry(
        Box((2.0 * btn_radius, 2.0 * btn_radius, 0.012)), mass=0.03
    )
    # Place on tank top center; travel is -Z (downward into tank).
    tank_top_z = SEAT_Z - 0.040 + TANK_HEIGHT
    model.articulation(
        "body_to_flush_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=flush_btn,
        origin=Origin(xyz=(TANK_DEPTH / 2.0, 0.0, tank_top_z + 0.006)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.05, lower=0.0, upper=0.005),
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
    flush_btn = object_model.get_part("flush_button")

    seat_joint = object_model.get_articulation("body_to_seat_ring")
    lid_joint = object_model.get_articulation("body_to_lid")
    flush_joint = object_model.get_articulation("body_to_flush_button")

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
        flush_btn, body,
        elem_a="flush_button_actuator", elem_b="tank",
        reason="Flush button is captured in the tank top face.",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="bowl_shell", elem_b="pedestal",
        reason="Bowl is fused into the pedestal base (one-piece ceramic body).",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="bowl_shell", elem_b="tank",
        reason="Neck connects bowl to tank (one-piece ceramic body).",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="bolt_cap_right", elem_b="pedestal",
        reason="Bolt cap sits on the pedestal base surface.",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="bolt_cap_left", elem_b="pedestal",
        reason="Bolt cap sits on the pedestal base surface.",
    )

    # --- Child-size: seat height is lower than a standard adult toilet. ---
    ctx.check(
        "child-size: seat height is below 0.34 m",
        SEAT_TOP_Z < 0.34,
        details=f"seat top z = {SEAT_TOP_Z}",
    )

    # --- Pedestal reaches the floor (floor-standing, not wall-hung). ---
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "pedestal reaches the floor (floor-standing toilet)",
        body_aabb[0][2] < 0.02,
        details=f"min z of body = {body_aabb[0][2]}",
    )

    # --- Floor bolt caps exist at the base front. ---
    ctx.check(
        "bolt_cap_right visual exists on body",
        body.get_visual("bolt_cap_right") is not None,
        details="bolt_cap_right not found",
    )
    ctx.check(
        "bolt_cap_left visual exists on body",
        body.get_visual("bolt_cap_left") is not None,
        details="bolt_cap_left not found",
    )

    # Bolt caps are near the floor
    bolt_cap_right_aabb = ctx.part_element_world_aabb(body, elem="bolt_cap_right")
    bolt_cap_left_aabb = ctx.part_element_world_aabb(body, elem="bolt_cap_left")
    ctx.check(
        "bolt caps are near the floor (z < 0.05 m)",
        bolt_cap_right_aabb[1][2] < 0.05 and bolt_cap_left_aabb[1][2] < 0.05,
        details=f"right top z={bolt_cap_right_aabb[1][2]}, left top z={bolt_cap_left_aabb[1][2]}",
    )

    # Bolt caps are symmetric about Y centerline
    right_y = (bolt_cap_right_aabb[0][1] + bolt_cap_right_aabb[1][1]) / 2.0
    left_y = (bolt_cap_left_aabb[0][1] + bolt_cap_left_aabb[1][1]) / 2.0
    ctx.check(
        "bolt caps are symmetric about centerline",
        abs(right_y + left_y) < 0.01,
        details=f"right_y={right_y}, left_y={left_y}",
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
    lid_front_x0 = ctx.part_world_aabb(lid)[1][0]
    lid_top_z0 = ctx.part_world_aabb(lid)[1][2]
    with ctx.pose({lid_joint: -math.radians(100.0)}):
        lid_top_z1 = ctx.part_world_aabb(lid)[1][2]
        lid_front_x1 = ctx.part_world_aabb(lid)[1][0]
    ctx.check(
        "lid rotates open (lifts upward)",
        lid_top_z1 > lid_top_z0 + 0.03,
        details=f"closed top z={lid_top_z0}, open top z={lid_top_z1}",
    )
    ctx.check(
        "lid swings rearward when opened",
        lid_front_x1 < lid_front_x0 - 0.03,
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
        seat_top_z1 > seat_top_z0 + 0.03,
        details=f"closed top z={seat_top_z0}, open top z={seat_top_z1}",
    )

    # --- Flush button depresses (prismatic downward into tank). ---
    btn_z0 = ctx.part_world_position(flush_btn)[2]
    with ctx.pose({flush_joint: 0.005}):
        btn_z1 = ctx.part_world_position(flush_btn)[2]
    ctx.check(
        "flush button depresses downward",
        btn_z1 < btn_z0 - 0.003,
        details=f"rest z={btn_z0}, pressed z={btn_z1}",
    )
    ctx.check(
        "flush button is on the tank (above seat level)",
        btn_z0 > SEAT_Z,
        details=f"button z={btn_z0}, seat z={SEAT_Z}",
    )

    # --- Tank exists above seat level at the back. ---
    tank_aabb = ctx.part_element_world_aabb(body, elem="tank")
    ctx.check(
        "tank extends above seat level",
        tank_aabb[1][2] > SEAT_Z + 0.10,
        details=f"tank top z={tank_aabb[1][2]}",
    )

    # --- Non-fixed joints exist (revolute seat hinge at minimum). ---
    ctx.check(
        "seat hinge is a non-fixed revolute joint",
        seat_joint.articulation_type == ArticulationType.REVOLUTE
        and seat_joint.motion_limits.upper > seat_joint.motion_limits.lower,
        details=f"type={seat_joint.articulation_type}, limits=({seat_joint.motion_limits.lower}, {seat_joint.motion_limits.upper})",
    )

    return ctx.report()


object_model = build_object_model()
