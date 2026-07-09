from __future__ import annotations

# Two-piece white ceramic toilet with separate tank and bowl, soft-close
# hinged seat ring and independently rotating lid, visible hinge barrels,
# tank lid seam, and chrome dual-button flush on the tank top.
#
# Frame / coordinate convention:
#   +X = forward (front of the bowl points to +X).
#   +Y = left-right (hinge axis for lid + seat ring runs along Y).
#   +Z = up. Floor at z=0; seat rim ~0.40 m above floor.
#
# Part hierarchy:
#   bowl_body (root): floor-standing ceramic bowl with pedestal, rear bridge
#     wall connecting to tank, and hinge barrel mounts.
#     tank (FIXED): separate ceramic cistern bolted to rear of bowl.
#       tank_lid (FIXED): ceramic lid with visible seam on top of tank.
#       flush_button_large (PRISMATIC): chrome full-flush button on tank lid.
#       flush_button_small (PRISMATIC): chrome half-flush button on tank lid.
#     seat_ring (REVOLUTE): oval seat ring hinged at rear of bowl rim.
#       lid (REVOLUTE): oval lid hinged independently above the seat ring.

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
FLOOR_Z = 0.0
RIM_Z = 0.395           # top of bowl rim / seat shelf
BOWL_W = 0.370          # bowl outer width (Y)
BOWL_FRONT_X = 0.410    # front-most point of the bowl
BOWL_REAR_X = -0.020    # rear of the bowl at rim level
TANK_FRONT_X = -0.050   # front face of the tank
TANK_REAR_X = -0.230    # rear face of the tank
TANK_W = 0.420          # tank width (Y)
TANK_BOTTOM_Z = 0.180   # bottom of the tank body
TANK_TOP_Z = 0.750      # top of the tank body
TANK_LID_TOP_Z = 0.778  # top of the tank lid

# Hinge axis location (rear of bowl rim)
HINGE_X = -0.010
HINGE_Z = RIM_Z + 0.006

# Bowl center X (approximate geometric center of the oval bowl)
BOWL_CX = (BOWL_FRONT_X + BOWL_REAR_X) / 2.0  # ~0.195


def _ellipse_pts(rx, ry, cx, segs=72):
    """Generate ellipse points in XY plane centered at (cx, 0)."""
    return [
        (cx + rx * math.cos(2.0 * math.pi * i / segs),
         ry * math.sin(2.0 * math.pi * i / segs))
        for i in range(segs)
    ]


def _oval_ring(z, rx_out, ry_out, rx_in, ry_in, thick, cx=0.0, segs=72):
    """Flat oval annular ring of given thickness at height z."""
    outer = (
        cq.Workplane("XY")
        .workplane(offset=z)
        .polyline(_ellipse_pts(rx_out, ry_out, cx, segs))
        .close()
        .polyline(_ellipse_pts(rx_in, ry_in, cx, segs))
        .close()
        .extrude(thick)
    )
    return outer


def _oval_solid(z, rx, ry, thick, cx=0.0, segs=72):
    """Solid oval disc of given thickness at height z."""
    disc = (
        cq.Workplane("XY")
        .workplane(offset=z)
        .polyline(_ellipse_pts(rx, ry, cx, segs))
        .close()
        .extrude(thick)
    )
    return disc


def _bowl_body_solid():
    """Floor-standing ceramic bowl: pedestal base + bowl body + rear bridge wall."""
    cx = BOWL_CX

    # --- Pedestal base: tapered oval column from floor up ---
    pedestal = (
        cq.Workplane("XY")
        .workplane(offset=FLOOR_Z)
        .polyline(_ellipse_pts(0.130, 0.100, cx, segs=64))
        .close()
        .workplane(offset=0.06)
        .polyline(_ellipse_pts(0.145, 0.115, cx, segs=64))
        .close()
        .workplane(offset=0.06)
        .polyline(_ellipse_pts(0.155, 0.130, cx, segs=64))
        .close()
        .loft(ruled=False)
    )

    # --- Bowl body: from pedestal top up to rim, widening oval ---
    body_lower = (
        cq.Workplane("XY")
        .workplane(offset=0.12)
        .polyline(_ellipse_pts(0.155, 0.130, cx, segs=64))
        .close()
        .workplane(offset=0.10)
        .polyline(_ellipse_pts(0.175, 0.155, cx, segs=64))
        .close()
        .workplane(offset=0.08)
        .polyline(_ellipse_pts(0.185, 0.175, cx, segs=64))
        .close()
        .workplane(offset=0.095)
        .polyline(_ellipse_pts(0.185, 0.185, cx, segs=64))
        .close()
        .loft(ruled=False)
    )

    # --- Rim shelf: flat ring at the top of the bowl ---
    shelf = _oval_ring(
        RIM_Z - 0.022,
        rx_out=0.200, ry_out=0.195,
        rx_in=0.148, ry_in=0.152,
        thick=0.022, cx=cx,
    )

    # --- Hollow basin cavity ---
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=RIM_Z - 0.200)
        .polyline(_ellipse_pts(0.080, 0.085, cx, segs=64))
        .close()
        .workplane(offset=0.120)
        .polyline(_ellipse_pts(0.140, 0.145, cx, segs=64))
        .close()
        .workplane(offset=0.085)
        .polyline(_ellipse_pts(0.148, 0.152, cx, segs=64))
        .close()
        .loft(ruled=False)
    )

    # --- Rear bridge wall: connects back of bowl to the tank ---
    # This is the ceramic wall that extends from the rear of the bowl upward
    # to meet the tank front face. It carries the bolt-on connection.
    bridge = (
        cq.Workplane("XY")
        .workplane(offset=0.12)
        .center((BOWL_REAR_X + TANK_FRONT_X) / 2.0, 0.0)
        .box(
            abs(TANK_FRONT_X - BOWL_REAR_X) + 0.04,
            0.260,
            RIM_Z - 0.12 + 0.06,
            centered=(True, True, False),
        )
    )

    # --- Rear mounting shelf: a wider flat area at rim level where tank bolts ---
    mount_shelf = (
        cq.Workplane("XY")
        .workplane(offset=RIM_Z - 0.030)
        .center((BOWL_REAR_X + TANK_FRONT_X) / 2.0 - 0.01, 0.0)
        .box(0.12, 0.32, 0.030, centered=(True, True, False))
    )

    bowl = pedestal.union(body_lower).union(shelf).union(bridge).union(mount_shelf)
    bowl = bowl.cut(cavity)
    return bowl


def _tank_body_solid():
    """Separate ceramic tank cistern with slightly rounded edges."""
    tank_cx = (TANK_FRONT_X + TANK_REAR_X) / 2.0  # center X of tank
    tank_h = TANK_TOP_Z - TANK_BOTTOM_Z
    tank_d = abs(TANK_REAR_X - TANK_FRONT_X)

    # Main tank body as a box with filleted vertical edges
    tank = (
        cq.Workplane("XY")
        .workplane(offset=TANK_BOTTOM_Z)
        .center(tank_cx, 0.0)
        .box(tank_d, TANK_W, tank_h, centered=(True, True, False))
    )
    # Fillet the four vertical edges for a softer ceramic look
    tank = tank.edges("|Z").fillet(0.020)

    return tank


def _tank_lid_solid():
    """Thin ceramic tank lid, slightly larger than tank body for overhang."""
    tank_cx = (TANK_FRONT_X + TANK_REAR_X) / 2.0
    tank_d = abs(TANK_REAR_X - TANK_FRONT_X)
    lid_thick = 0.025
    overhang = 0.012

    lid = (
        cq.Workplane("XY")
        .workplane(offset=TANK_TOP_Z + 0.002)  # 2mm gap = visible seam
        .center(tank_cx, 0.0)
        .box(tank_d + overhang * 2, TANK_W + overhang * 2, lid_thick,
             centered=(True, True, False))
    )
    lid = lid.edges("|Z").fillet(0.015)
    return lid


def _hinge_barrel_solid(y_offset):
    """One visible hinge barrel: a chrome cylinder at the rear of the bowl rim."""
    barrel_r = 0.010
    barrel_len = 0.035
    barrel = (
        cq.Workplane("XZ")
        .workplane(offset=y_offset)
        .center(HINGE_X, HINGE_Z)
        .circle(barrel_r)
        .extrude(barrel_len)
        .translate((0, -barrel_len / 2.0, 0))
    )
    return barrel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_piece_toilet")

    ceramic = model.material("ceramic_white", rgba=(0.95, 0.95, 0.94, 1.0))
    seat_white = model.material("seat_white", rgba=(0.97, 0.97, 0.96, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    hinge_chrome = model.material("hinge_chrome", rgba=(0.70, 0.72, 0.74, 1.0))

    cx = BOWL_CX

    # ================= ROOT: bowl body =================
    body = model.part("bowl_body")

    # Ceramic bowl (pedestal + basin + bridge + mounting shelf)
    bowl_geo = _bowl_body_solid()
    body.visual(
        mesh_from_cadquery(bowl_geo, "bowl_shell"),
        material=ceramic,
        name="bowl_shell",
    )

    # Visible hinge barrels at the rear of the bowl rim (two chrome cylinders)
    hinge_barrels = _hinge_barrel_solid(0.065).union(_hinge_barrel_solid(-0.065))
    body.visual(
        mesh_from_cadquery(hinge_barrels, "hinge_barrels"),
        material=hinge_chrome,
        name="hinge_barrels",
    )

    body.inertial = Inertial.from_geometry(
        Box((0.50, 0.38, 0.42)),
        mass=25.0,
        origin=Origin(xyz=(0.15, 0.0, 0.20)),
    )

    # ================= tank (FIXED to bowl body) =================
    tank = model.part("tank")
    tank_geo = _tank_body_solid()
    tank.visual(
        mesh_from_cadquery(tank_geo, "tank_shell"),
        material=ceramic,
        name="tank_shell",
    )
    tank.inertial = Inertial.from_geometry(
        Box((0.18, 0.42, 0.57)),
        mass=12.0,
        origin=Origin(xyz=((TANK_FRONT_X + TANK_REAR_X) / 2.0, 0.0,
                           (TANK_BOTTOM_Z + TANK_TOP_Z) / 2.0)),
    )
    model.articulation(
        "bowl_to_tank",
        ArticulationType.FIXED,
        parent=body,
        child=tank,
        origin=Origin(xyz=((TANK_FRONT_X + TANK_REAR_X) / 2.0, 0.0,
                           (TANK_BOTTOM_Z + TANK_TOP_Z) / 2.0)),
    )

    # ================= tank lid (FIXED to tank, visible seam gap) =================
    tank_lid = model.part("tank_lid")
    tank_lid_geo = _tank_lid_solid()
    tank_lid.visual(
        mesh_from_cadquery(tank_lid_geo, "tank_lid_shell"),
        material=ceramic,
        name="tank_lid_shell",
    )
    tank_lid.inertial = Inertial.from_geometry(
        Box((0.21, 0.44, 0.025)),
        mass=1.5,
        origin=Origin(xyz=((TANK_FRONT_X + TANK_REAR_X) / 2.0, 0.0,
                           TANK_TOP_Z + 0.014)),
    )
    model.articulation(
        "tank_to_tank_lid",
        ArticulationType.FIXED,
        parent=tank,
        child=tank_lid,
        origin=Origin(xyz=((TANK_FRONT_X + TANK_REAR_X) / 2.0, 0.0,
                           TANK_TOP_Z + 0.002)),
    )

    # ================= flush buttons (PRISMATIC on tank lid top) =================
    tank_cx = (TANK_FRONT_X + TANK_REAR_X) / 2.0
    button_z = TANK_LID_TOP_Z

    button_specs = [
        ("flush_button_large", 0.028, tank_cx + 0.032),
        ("flush_button_small", 0.018, tank_cx - 0.032),
    ]
    for part_name, radius, btn_x in button_specs:
        b = model.part(part_name)
        # Chrome puck standing on the tank lid top, depresses downward (-Z)
        puck = CylinderGeometry(radius, 0.012, radial_segments=40)
        b.visual(
            mesh_from_geometry(puck, part_name + "_actuator"),
            material=chrome,
            name=part_name + "_actuator",
        )
        b.inertial = Inertial.from_geometry(
            Box((2.0 * radius, 2.0 * radius, 0.012)), mass=0.04
        )
        model.articulation(
            "tank_to_" + part_name,
            ArticulationType.PRISMATIC,
            parent=tank_lid,
            child=b,
            origin=Origin(xyz=(btn_x, 0.0, button_z + 0.006)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=0.05, lower=0.0, upper=0.006),
        )

    # ================= seat ring (REVOLUTE at rear hinge) =================
    seat = model.part("seat_ring")
    seat_local_cx = cx - HINGE_X
    seat_ring_geo = _oval_disc_ring_solid(
        rx_out=0.185,
        ry_out=0.182,
        rx_in=0.112,
        ry_in=0.120,
        thick=0.018,
        cx=seat_local_cx,
    )
    seat.visual(
        mesh_from_cadquery(seat_ring_geo.translate((0, 0, 0.002)), "seat_ring_shell"),
        material=seat_white,
        name="seat_ring_shell",
    )
    seat.inertial = Inertial.from_geometry(
        Box((0.37, 0.36, 0.022)),
        mass=0.8,
        origin=Origin(xyz=(seat_local_cx, 0.0, 0.011)),
    )
    model.articulation(
        "bowl_to_seat_ring",
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

    # ================= lid (REVOLUTE from seat ring, independent) =================
    lid = model.part("lid")
    lid_local_cx = cx - HINGE_X
    lid_geo = _oval_solid(z=0.0, rx=0.192, ry=0.188, thick=0.016, cx=lid_local_cx)
    lid.visual(
        mesh_from_cadquery(lid_geo.translate((0, 0, 0.022)), "lid_shell"),
        material=seat_white,
        name="lid_shell",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.38, 0.37, 0.018)),
        mass=0.9,
        origin=Origin(xyz=(lid_local_cx, 0.0, 0.030)),
    )
    model.articulation(
        "seat_to_lid",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0,
            lower=-math.radians(100.0), upper=0.0,
        ),
    )

    return model


def _oval_disc_ring_solid(rx_out, ry_out, rx_in, ry_in, thick, cx, segs=80):
    """Flat oval annular ring used for the seat ring."""
    ring = (
        cq.Workplane("XY")
        .polyline(_ellipse_pts(rx_out, ry_out, cx, segs))
        .close()
        .polyline(_ellipse_pts(rx_in, ry_in, cx, segs))
        .close()
        .extrude(thick)
    )
    return ring


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bowl_body")
    tank = object_model.get_part("tank")
    tank_lid = object_model.get_part("tank_lid")
    seat = object_model.get_part("seat_ring")
    lid = object_model.get_part("lid")
    btn_large = object_model.get_part("flush_button_large")
    btn_small = object_model.get_part("flush_button_small")

    seat_joint = object_model.get_articulation("bowl_to_seat_ring")
    lid_joint = object_model.get_articulation("seat_to_lid")
    btn_large_joint = object_model.get_articulation("tank_to_flush_button_large")
    btn_small_joint = object_model.get_articulation("tank_to_flush_button_small")
    tank_joint = object_model.get_articulation("bowl_to_tank")
    tank_lid_joint = object_model.get_articulation("tank_to_tank_lid")

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
        elem_a="bowl_shell", elem_b="hinge_barrels",
        reason="Hinge barrels are mounted into the rear bowl rim (embedded fasteners).",
    )
    ctx.allow_overlap(
        btn_large, tank_lid,
        elem_a="flush_button_large_actuator", elem_b="tank_lid_shell",
        reason="Large flush button is captured in the tank lid face.",
    )
    ctx.allow_overlap(
        btn_small, tank_lid,
        elem_a="flush_button_small_actuator", elem_b="tank_lid_shell",
        reason="Small flush button is captured in the tank lid face.",
    )

    # --- Two-piece structure: tank and bowl are separate parts ---
    ctx.check(
        "tank and bowl_body are separate parts",
        tank is not body,
        details="tank must be a distinct part from bowl_body",
    )

    # --- Tank is behind the bowl ---
    tank_aabb = ctx.part_world_aabb(tank)
    bowl_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "tank is behind the bowl (tank max X < bowl max X)",
        tank_aabb[1][0] < bowl_aabb[1][0] - 0.05,
        details=f"tank max x={tank_aabb[1][0]:.3f}, bowl max x={bowl_aabb[1][0]:.3f}",
    )

    # --- Bowl reaches near the floor (floor-standing, not wall-hung) ---
    ctx.check(
        "bowl stands on the floor (min z near 0)",
        bowl_aabb[0][2] < 0.02,
        details=f"bowl min z={bowl_aabb[0][2]:.3f}",
    )

    # --- Tank lid has visible seam (gap between tank top and lid bottom) ---
    tank_shell_aabb = ctx.part_element_world_aabb(tank, elem="tank_shell")
    tank_lid_shell_aabb = ctx.part_element_world_aabb(tank_lid, elem="tank_lid_shell")
    ctx.check(
        "tank lid sits above tank body (visible seam gap)",
        tank_lid_shell_aabb[0][2] > tank_shell_aabb[1][2] - 0.005,
        details=(
            f"tank_lid bottom z={tank_lid_shell_aabb[0][2]:.4f}, "
            f"tank top z={tank_shell_aabb[1][2]:.4f}"
        ),
    )

    # --- Visible hinge barrels exist on the bowl body ---
    hinge_aabb = ctx.part_element_world_aabb(body, elem="hinge_barrels")
    ctx.check(
        "hinge barrels are present at the rear of the bowl",
        hinge_aabb is not None and hinge_aabb[0][2] > RIM_Z - 0.02,
        details=f"hinge barrels AABB={hinge_aabb}",
    )

    # --- Seat ring rests on the bowl rim ---
    seat_aabb = ctx.part_world_aabb(seat)
    ctx.expect_gap(
        seat, body,
        axis="z",
        min_gap=-0.005,
        max_gap=0.015,
        positive_elem="seat_ring_shell",
        negative_elem="bowl_shell",
        name="seat ring rests on bowl rim (z gap near zero)",
    )

    # --- Lid sits above the seat ring when closed ---
    lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid sits above the seat ring when closed",
        lid_aabb[0][2] >= seat_aabb[1][2] - 0.008,
        details=f"lid bottom z={lid_aabb[0][2]:.4f}, seat top z={seat_aabb[1][2]:.4f}",
    )

    # --- Seat ring rotates open (revolute, lifts upward) ---
    seat_top_z0 = ctx.part_world_aabb(seat)[1][2]
    with ctx.pose({seat_joint: -math.radians(100.0)}):
        seat_top_z1 = ctx.part_world_aabb(seat)[1][2]
    ctx.check(
        "seat ring rotates open (lifts upward)",
        seat_top_z1 > seat_top_z0 + 0.05,
        details=f"closed top z={seat_top_z0:.3f}, open top z={seat_top_z1:.3f}",
    )

    # --- Lid rotates independently from the seat (its own revolute joint) ---
    lid_top_z0 = ctx.part_world_aabb(lid)[1][2]
    with ctx.pose({lid_joint: -math.radians(100.0)}):
        lid_top_z1 = ctx.part_world_aabb(lid)[1][2]
    ctx.check(
        "lid rotates independently (lifts upward on its own joint)",
        lid_top_z1 > lid_top_z0 + 0.05,
        details=f"closed top z={lid_top_z0:.3f}, open top z={lid_top_z1:.3f}",
    )

    # --- Lid joint parent is seat_ring (lid hinged to seat, not directly to bowl) ---
    ctx.check(
        "lid articulation parent is the seat ring (independent above seat)",
        lid_joint.parent.name == "seat_ring",
        details=f"lid parent={lid_joint.parent.name}",
    )

    # --- Seat hinge axis runs along Y (lateral) ---
    ctx.check(
        "seat hinge axis runs along Y",
        abs(seat_joint.axis[1]) > 0.9,
        details=f"seat axis={seat_joint.axis}",
    )

    # --- Flush buttons on tank top, depress downward ---
    for name, part_obj, joint in (
        ("large", btn_large, btn_large_joint),
        ("small", btn_small, btn_small_joint),
    ):
        z0 = ctx.part_world_position(part_obj)[2]
        with ctx.pose({joint: 0.006}):
            z1 = ctx.part_world_position(part_obj)[2]
        ctx.check(
            f"{name} flush button depresses downward",
            z1 < z0 - 0.004,
            details=f"rest z={z0:.4f}, pressed z={z1:.4f}",
        )
        ctx.check(
            f"{name} flush button is on top of the tank (above bowl rim)",
            z0 > RIM_Z + 0.20,
            details=f"button z={z0:.4f}",
        )

    # --- Dual flush: large button is bigger than small button ---
    large_dy = _ext(ctx.part_world_aabb(btn_large))[1]
    small_dy = _ext(ctx.part_world_aabb(btn_small))[1]
    ctx.check(
        "dual flush: large button is larger than small button",
        large_dy > small_dy + 0.005,
        details=f"large dy={large_dy:.4f}, small dy={small_dy:.4f}",
    )

    # --- Tank is connected to bowl body (not floating) ---
    ctx.check(
        "tank articulation connects to bowl_body",
        tank_joint.parent.name == "bowl_body",
        details=f"tank parent={tank_joint.parent.name}",
    )

    # --- Seat top is near expected comfort height ---
    ctx.check(
        "seat rim is near 0.40 m above floor",
        0.34 < RIM_Z < 0.46,
        details=f"rim z={RIM_Z}",
    )

    return ctx.report()


object_model = build_object_model()
