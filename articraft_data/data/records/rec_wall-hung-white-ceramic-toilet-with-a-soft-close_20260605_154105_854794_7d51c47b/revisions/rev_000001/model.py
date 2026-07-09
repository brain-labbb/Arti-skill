from __future__ import annotations

# Wall-hung white ceramic toilet with a soft-close seat.
#
# Frame / coordinate convention:
#   +X = forward (front of the bowl points to +X), wall is at -X.
#   +Y = left-right (the hinge axis for the lid + seat ring runs along Y).
#   +Z = up. The floor is at z=0; the seat top sits ~0.40 m above the floor.
#
# Root part = the wall-mounting back panel + the cantilevered ceramic bowl
# (they are one fixed ceramic+wall assembly so nothing floats). Everything
# else mounts to that root:
#   - lid          : oval top lid, REVOLUTE hinge at the rear (axis +Y), ~100 deg.
#   - seat_ring    : oval seat ring under the lid, REVOLUTE hinge sharing the
#                    SAME rear axis as the lid (concentric), ~100 deg.
#   - flush_button_large / flush_button_small : a dual-flush actuator pair
#                    (one big = full flush, one small = half flush) of round
#                    chrome push buttons on the centered wall flush plate, each
#                    PRISMATIC ~6 mm into the wall (-X travel).
# The chrome flush plate is fixed visual geometry centered on the back panel;
# the bowl cantilevers from the back panel; both flush buttons depress into the
# wall independently.

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
WALL_X = -0.010  # front face of the wall back-panel

# Rear hinge axis (shared by lid + seat ring), in world coords.
HINGE_X = 0.060
HINGE_Z = SEAT_TOP_Z + 0.004

# Seat plate top surface z (where seat ring + lid live).
SEAT_Z = SEAT_TOP_Z


def _oval_loft(z_bottom, z_top, rx, ry, taper=0.85, segs=64) -> cq.Workplane:
    # A vertical oval (ellipse in the XY plane) lofted between two z levels,
    # tapering slightly. rx along X, ry along Y. Centered at (cx, 0).
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
    # A flat oval ring (annular ellipse) of given thickness centered at (cx,0,z).
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
    # The cantilevered ceramic bowl: a rounded body that tapers from a wide
    # seat shelf at the top down to a narrower rounded bottom, hollowed at the
    # top to read as a real open ceramic bowl.
    cx = 0.215  # bowl center X (front-of-wall + half depth-ish)
    top_z = SEAT_Z
    bottom_z = SEAT_Z - 0.300

    # Outer body, lofted oval sections from a rounded bottom up to the rim.
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

    # Hollow the basin: an inner oval cavity cut from the top.
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

    # Seat shelf: a flat oval ceramic rim/shelf the seat ring rests on, around
    # the basin opening at the top of the bowl.
    shelf = _oval_ring(
        top_z - 0.022, rx_out=0.200, ry_out=0.195, rx_in=0.150, ry_in=0.150, thick=0.022, cx=cx
    )
    bowl = bowl.union(shelf)

    return bowl


def _back_panel_solid() -> cq.Workplane:
    # Thin wall mounting back-panel the bowl cantilevers from. Spans behind the
    # bowl and up to where the flush plate mounts.
    panel = (
        cq.Workplane("XY")
        .workplane(offset=0.060)
        .center(WALL_X - 0.015, 0.0)
        .box(0.030, 0.380, 0.760, centered=(True, True, False))
    )
    return panel


def _bowl_neck_solid() -> cq.Workplane:
    # Ceramic shroud / neck connecting the back of the bowl into the back panel
    # so the cantilever reads as solid, supported construction (no gap).
    neck = (
        cq.Workplane("XY")
        .workplane(offset=SEAT_Z - 0.170)
        .center(0.045, 0.0)
        .box(0.130, 0.230, 0.180, centered=(True, True, False))
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_hung_toilet")

    ceramic = model.material("ceramic_white", rgba=(0.95, 0.95, 0.94, 1.0))
    seat_white = model.material("seat_white", rgba=(0.97, 0.97, 0.96, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    wall_gray = model.material("wall_gray", rgba=(0.72, 0.72, 0.70, 1.0))

    cx = 0.215

    # ================= ROOT: back panel + cantilevered bowl =================
    body = model.part("body")

    # Wall back-panel (where everything mounts).
    body.visual(
        mesh_from_cadquery(_back_panel_solid(), "back_panel"),
        material=wall_gray,
        name="back_panel",
    )

    # Cantilevered ceramic bowl + connecting neck shroud into the panel.
    bowl = _bowl_solid().union(_bowl_neck_solid())
    body.visual(mesh_from_cadquery(bowl, "bowl_shell"), material=ceramic, name="bowl_shell")

    # Chrome flush plate (rectangular) fixed on the wall panel, centered on the
    # wall above the bowl (the back-panel spans Y in [-0.190, 0.190], so y=0 is
    # the wall centerline).
    plate_x = WALL_X
    plate_y = 0.0
    plate_z = 0.620
    body.visual(
        Box((0.012, 0.150, 0.230)),
        origin=Origin(xyz=(plate_x - 0.006, plate_y, plate_z)),
        material=chrome,
        name="flush_plate",
    )

    body.inertial = Inertial.from_geometry(
        Box((0.55, 0.38, 0.55)),
        mass=22.0,
        origin=Origin(xyz=(0.18, 0.0, SEAT_Z - 0.12)),
    )

    # ================= seat ring (revolute, rear axis) =================
    # Built in LID/SEAT-local frame with hinge axis at local origin; the
    # articulation origin places the axis in the world.
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

    # ================= dual flush buttons (both prismatic into wall) =========
    # Dual-flush actuator pair on the centered plate: a LARGE button (full
    # flush) and a SMALL button (half flush), side by side. Each round chrome
    # puck stands proud of the plate and depresses ~6 mm into the wall (-X).
    button_specs = [
        ("flush_button_large", 0.036, plate_y + 0.038),
        ("flush_button_small", 0.022, plate_y - 0.042),
    ]
    for part_name, radius, btn_y in button_specs:
        b = model.part(part_name)
        puck = CylinderGeometry(radius, 0.014, radial_segments=40).rotate_y(math.pi / 2.0)
        b.visual(
            mesh_from_geometry(puck, part_name + "_actuator"),
            material=chrome,
            name=part_name + "_actuator",
        )
        b.inertial = Inertial.from_geometry(
            Box((0.014, 2.0 * radius, 2.0 * radius)), mass=0.05
        )
        # Place at the plate front face; travel is -X (into wall).
        model.articulation(
            "body_to_" + part_name,
            ArticulationType.PRISMATIC,
            parent=body,
            child=b,
            origin=Origin(xyz=(plate_x + 0.006, btn_y, plate_z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=0.05, lower=0.0, upper=0.006),
        )

    return model


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
        btn_large, body,
        elem_a="flush_button_large_actuator", elem_b="flush_plate",
        reason="Large (full flush) push actuator is captured in the flush plate face.",
    )
    ctx.allow_overlap(
        btn_small, body,
        elem_a="flush_button_small_actuator", elem_b="flush_plate",
        reason="Small (half flush) push actuator is captured in the flush plate face.",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="bowl_shell", elem_b="back_panel",
        reason="Bowl neck shroud is fused into the wall back-panel (cantilever support).",
    )

    # --- Bowl is cantilevered off the wall plate (no floor pedestal). ---
    bowl_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "bowl is wall-hung (does not reach the floor)",
        bowl_aabb[0][2] > 0.05,
        details=f"min z of body = {bowl_aabb[0][2]}",
    )
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

    # --- Both flush buttons depress (prismatic into the wall, -X). ---
    for name, part, joint in (
        ("large", btn_large, btn_large_joint),
        ("small", btn_small, btn_small_joint),
    ):
        x0 = ctx.part_world_position(part)[0]
        with ctx.pose({joint: 0.006}):
            x1 = ctx.part_world_position(part)[0]
        ctx.check(
            f"{name} flush button depresses into the wall",
            x1 < x0 - 0.004,
            details=f"rest x={x0}, pressed x={x1}",
        )
        ctx.check(
            f"{name} flush button is mounted high on the wall (above the bowl)",
            ctx.part_world_position(part)[2] > SEAT_Z,
            details=f"button z={ctx.part_world_position(part)[2]}",
        )

    # --- Dual-flush: large button is bigger than the small button. ---
    large_dy = _ext(ctx.part_world_aabb(btn_large))[1]
    small_dy = _ext(ctx.part_world_aabb(btn_small))[1]
    ctx.check(
        "dual flush: large button is larger than the small button",
        large_dy > small_dy + 0.005,
        details=f"large dy={large_dy}, small dy={small_dy}",
    )

    # --- Flush plate/buttons are centered on the wall (near the centerline). ---
    large_y = ctx.part_world_position(btn_large)[1]
    small_y = ctx.part_world_position(btn_small)[1]
    ctx.check(
        "flush buttons straddle the wall centerline (plate centered)",
        abs((large_y + small_y) / 2.0) < 0.05 and large_y > small_y,
        details=f"large y={large_y}, small y={small_y}",
    )

    return ctx.report()


object_model = build_object_model()
