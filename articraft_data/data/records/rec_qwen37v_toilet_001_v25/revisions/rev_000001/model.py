from __future__ import annotations

# Elongated modern wall-hung white ceramic toilet with skirted base.
#
# Frame / coordinate convention:
#   +X = forward (front of the bowl points to +X), wall is at -X.
#   +Y = left-right (the hinge axis for the lid + seat ring runs along Y).
#   +Z = up. The floor is at z=0; the seat top sits ~0.40 m above the floor.
#
# Root part = wall back panel + rear concealed-cistern pod + cantilevered
# elongated skirted ceramic bowl (one fixed assembly).
#
# Articulated parts:
#   - lid          : elongated oval lid, REVOLUTE hinge at rear (axis +Y), ~100 deg.
#   - seat_ring    : elongated oval seat ring, REVOLUTE hinge sharing the same
#                    rear axis (concentric), ~100 deg.
#   - flush_button_large / flush_button_small : dual-flush push buttons on top
#                    of the rear pod, PRISMATIC pressing down (-Z) ~6 mm.
#
# Visible hinge barrels are modeled as small cylinders on the seat ring and lid
# near the shared hinge axis. The rear pod has a visible lid seam panel line.

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
BOWL_W = 0.370  # bowl width (Y) - slightly wider for elongated modern
BOWL_DEPTH = 0.580  # elongated bowl depth (X), front lip to wall
WALL_X = -0.010  # front face of the wall back-panel

# Rear hinge axis (shared by lid + seat ring), in world coords.
HINGE_X = 0.050
HINGE_Z = SEAT_TOP_Z + 0.004

# Rear pod (concealed cistern housing) dimensions
POD_X = WALL_X - 0.015  # rear of pod
POD_FRONT_X = 0.070  # front face of pod
POD_W = 0.380  # pod width (Y)
POD_TOP_Z = 0.820  # top of pod
POD_BOTTOM_Z = 0.100  # bottom of pod

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
    # Elongated modern ceramic bowl: a rounded body that tapers from a wide
    # seat shelf at the top down to a narrower rounded bottom, hollowed at the
    # top to read as a real open ceramic bowl. Elongated along X.
    cx = 0.250  # bowl center X (shifted forward for elongated)
    top_z = SEAT_Z
    bottom_z = SEAT_Z - 0.280

    # Outer body, lofted oval sections from a rounded bottom up to the rim.
    # More elongated (larger rx vs ry) for modern elongated form.
    outer = (
        cq.Workplane("XY")
        .workplane(offset=bottom_z)
        .ellipse(0.080, 0.075)
        .workplane(offset=0.080)
        .ellipse(0.140, 0.115)
        .workplane(offset=0.110)
        .ellipse(0.185, 0.160)
        .workplane(offset=0.090)
        .ellipse(0.200, 0.175)
        .loft(ruled=False)
    )
    outer = outer.translate((cx, 0.0, 0.0))

    # Hollow the basin: an inner oval cavity cut from the top.
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=top_z - 0.180)
        .ellipse(0.090, 0.080)
        .workplane(offset=0.120)
        .ellipse(0.160, 0.140)
        .workplane(offset=0.070)
        .ellipse(0.175, 0.155)
        .loft(ruled=False)
        .translate((cx, 0.0, 0.0))
    )
    bowl = outer.cut(cavity)

    # Seat shelf: a flat oval ceramic rim/shelf the seat ring rests on.
    shelf = _oval_ring(
        top_z - 0.022, rx_out=0.215, ry_out=0.190, rx_in=0.165, ry_in=0.148, thick=0.022, cx=cx
    )
    bowl = bowl.union(shelf)

    return bowl


def _skirt_solid() -> cq.Workplane:
    # Skirted base: smooth enclosed underside panel that hides trapway.
    # A gently curved panel below the bowl that reads as a clean skirted design.
    cx = 0.250
    skirt_top = SEAT_Z - 0.280
    skirt_bottom = SEAT_Z - 0.360

    skirt = (
        cq.Workplane("XY")
        .workplane(offset=skirt_bottom)
        .ellipse(0.060, 0.065)
        .workplane(offset=(skirt_top - skirt_bottom))
        .ellipse(0.080, 0.075)
        .loft(ruled=False)
    )
    skirt = skirt.translate((cx, 0.0, 0.0))

    # Smooth transition collar connecting skirt top to bowl bottom
    collar = (
        cq.Workplane("XY")
        .workplane(offset=skirt_top - 0.010)
        .ellipse(0.082, 0.078)
        .workplane(offset=0.020)
        .ellipse(0.080, 0.075)
        .loft(ruled=False)
    )
    collar = collar.translate((cx, 0.0, 0.0))
    skirt = skirt.union(collar)

    return skirt


def _rear_pod_solid() -> cq.Workplane:
    # Rear pod/concealed cistern housing: a boxy rounded shell that sits against
    # the wall above and behind the bowl. Modern concealed-cistern toilets have
    # this visible pod.
    pod_cx = (POD_X + POD_FRONT_X) / 2.0
    pod_dx = POD_FRONT_X - POD_X
    pod_dz = POD_TOP_Z - POD_BOTTOM_Z

    # Main pod body with slightly rounded edges
    pod = (
        cq.Workplane("XY")
        .workplane(offset=POD_BOTTOM_Z)
        .center(pod_cx, 0.0)
        .box(pod_dx, POD_W, pod_dz, centered=(True, True, False))
    )

    return pod


def _pod_seam_solid() -> cq.Workplane:
    # Tank lid seam: a thin raised line/panel gap on top of the pod suggesting
    # a removable cistern access panel.
    pod_cx = (POD_X + POD_FRONT_X) / 2.0
    pod_dx = POD_FRONT_X - POD_X

    # A thin rectangular strip inset from the pod top edges
    seam = (
        cq.Workplane("XY")
        .workplane(offset=POD_TOP_Z - 0.001)
        .center(pod_cx, 0.0)
        .box(pod_dx - 0.030, POD_W - 0.040, 0.003, centered=(True, True, False))
    )
    return seam


def _back_panel_solid() -> cq.Workplane:
    # Thin wall mounting back-panel behind the pod and bowl.
    panel = (
        cq.Workplane("XY")
        .workplane(offset=0.050)
        .center(WALL_X - 0.015, 0.0)
        .box(0.030, 0.400, 0.800, centered=(True, True, False))
    )
    return panel


def _bowl_neck_solid() -> cq.Workplane:
    # Ceramic shroud / neck connecting the back of the bowl into the rear pod
    # so the cantilever reads as solid, supported construction.
    neck = (
        cq.Workplane("XY")
        .workplane(offset=SEAT_Z - 0.160)
        .center(0.040, 0.0)
        .box(0.120, 0.240, 0.170, centered=(True, True, False))
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


def _hinge_barrel(length, radius=0.010) -> cq.Workplane:
    # A small cylindrical hinge barrel oriented along Y axis.
    barrel = (
        cq.Workplane("XZ")
        .center(0.0, 0.0)
        .circle(radius)
        .extrude(length)
    )
    # Center it along Y
    barrel = barrel.translate((0.0, -length / 2.0, 0.0))
    return barrel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_hung_toilet_elongated")

    ceramic = model.material("ceramic_white", rgba=(0.95, 0.95, 0.94, 1.0))
    seat_white = model.material("seat_white", rgba=(0.97, 0.97, 0.96, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    wall_gray = model.material("wall_gray", rgba=(0.72, 0.72, 0.70, 1.0))
    seam_dark = model.material("seam_gray", rgba=(0.55, 0.55, 0.54, 1.0))
    hinge_chrome = model.material("hinge_chrome", rgba=(0.70, 0.72, 0.74, 1.0))

    cx_bowl = 0.250

    # ================= ROOT: back panel + rear pod + bowl + skirt =================
    body = model.part("body")

    # Wall back-panel.
    body.visual(
        mesh_from_cadquery(_back_panel_solid(), "back_panel"),
        material=wall_gray,
        name="back_panel",
    )

    # Rear pod (concealed cistern housing).
    body.visual(
        mesh_from_cadquery(_rear_pod_solid(), "rear_pod"),
        material=ceramic,
        name="rear_pod",
    )

    # Pod seam (tank lid seam visible on top of pod).
    body.visual(
        mesh_from_cadquery(_pod_seam_solid(), "pod_seam"),
        material=seam_dark,
        name="pod_seam",
    )

    # Elongated ceramic bowl + skirted base + connecting neck.
    bowl_assembly = _bowl_solid().union(_skirt_solid()).union(_bowl_neck_solid())
    body.visual(
        mesh_from_cadquery(bowl_assembly, "bowl_shell"),
        material=ceramic,
        name="bowl_shell",
    )

    # Chrome flush plate on top of the rear pod (horizontal orientation).
    plate_x = (POD_X + POD_FRONT_X) / 2.0
    plate_y = 0.0
    plate_z = POD_TOP_Z + 0.002
    body.visual(
        Box((0.100, 0.150, 0.008)),
        origin=Origin(xyz=(plate_x, plate_y, plate_z)),
        material=chrome,
        name="flush_plate",
    )

    body.inertial = Inertial.from_geometry(
        Box((0.55, 0.40, 0.75)),
        mass=25.0,
        origin=Origin(xyz=(0.18, 0.0, 0.35)),
    )

    # ================= seat ring (revolute, rear axis) =================
    seat = model.part("seat_ring")
    seat_local_cx = cx_bowl - HINGE_X
    seat_ring_geo = _oval_disc_ring_solid(
        rx_out=0.195,
        ry_out=0.175,
        rx_in=0.125,
        ry_in=0.135,
        thick=0.020,
        cx=seat_local_cx,
    )
    seat.visual(
        mesh_from_cadquery(seat_ring_geo.translate((0, 0, 0.002)), "seat_ring_shell"),
        material=seat_white,
        name="seat_ring_shell",
    )

    # Visible hinge barrels behind the seat (two small chrome cylinders).
    barrel_left = _hinge_barrel(0.040, radius=0.009).translate((-0.005, -0.055, 0.0))
    barrel_right = _hinge_barrel(0.040, radius=0.009).translate((-0.005, 0.015, 0.0))
    seat_barrels = barrel_left.union(barrel_right)
    seat.visual(
        mesh_from_cadquery(seat_barrels, "seat_hinge_barrels"),
        material=hinge_chrome,
        name="seat_hinge_barrels",
    )

    seat.inertial = Inertial.from_geometry(
        Box((0.39, 0.36, 0.025)),
        mass=0.85,
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
    lid_local_cx = cx_bowl - HINGE_X
    lid_geo = _oval_disc_solid(rx=0.205, ry=0.185, thick=0.016, cx=lid_local_cx)
    lid.visual(
        mesh_from_cadquery(lid_geo.translate((0, 0, 0.020)), "lid_shell"),
        material=seat_white,
        name="lid_shell",
    )

    # Visible hinge barrels behind the lid (two small chrome cylinders, slightly
    # offset from seat barrels to show concentric hinge arrangement).
    lid_barrel_left = _hinge_barrel(0.035, radius=0.008).translate((-0.005, -0.050, 0.002))
    lid_barrel_right = _hinge_barrel(0.035, radius=0.008).translate((-0.005, 0.015, 0.002))
    lid_barrels = lid_barrel_left.union(lid_barrel_right)
    lid.visual(
        mesh_from_cadquery(lid_barrels, "lid_hinge_barrels"),
        material=hinge_chrome,
        name="lid_hinge_barrels",
    )

    lid.inertial = Inertial.from_geometry(
        Box((0.41, 0.37, 0.020)),
        mass=0.9,
        origin=Origin(xyz=(lid_local_cx, 0.0, 0.028)),
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

    # ================= dual flush buttons (prismatic, press DOWN -Z) ==========
    # Dual-flush actuator pair on top of the rear pod: LARGE (full flush) and
    # SMALL (half flush), side by side. Each round chrome puck sits on top of
    # the plate and presses downward (-Z travel).
    button_specs = [
        ("flush_button_large", 0.030, plate_x + 0.022),
        ("flush_button_small", 0.020, plate_x - 0.022),
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
            Box((2.0 * radius, 2.0 * radius, 0.012)), mass=0.05
        )
        # Place at the plate top; travel is -Z (pressing down).
        model.articulation(
            "body_to_" + part_name,
            ArticulationType.PRISMATIC,
            parent=body,
            child=b,
            origin=Origin(xyz=(btn_x, plate_y, plate_z + 0.010)),
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
        reason="Large flush push actuator sits on top of the flush plate (seated contact).",
    )
    ctx.allow_overlap(
        btn_small, body,
        elem_a="flush_button_small_actuator", elem_b="flush_plate",
        reason="Small flush push actuator sits on top of the flush plate (seated contact).",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="bowl_shell", elem_b="back_panel",
        reason="Bowl neck shroud is fused into the wall back-panel (cantilever support).",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="bowl_shell", elem_b="rear_pod",
        reason="Bowl neck connects into the rear pod (structural connection).",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="rear_pod", elem_b="back_panel",
        reason="Rear pod is mounted against the wall back-panel.",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="pod_seam", elem_b="rear_pod",
        reason="Pod seam is an inset detail on the pod top surface.",
    )

    # --- Elongated bowl: depth should be clearly more than width. ---
    body_aabb = ctx.part_world_aabb(body)
    # The bowl extends well forward from the wall
    ctx.check(
        "bowl is elongated (depth > width)",
        BOWL_DEPTH > BOWL_W + 0.15,
        details=f"bowl depth={BOWL_DEPTH}, bowl width={BOWL_W}",
    )

    # --- Skirted base: bowl has a smooth enclosed underside. ---
    ctx.check(
        "skirted base extends below the main bowl body",
        SEAT_Z - 0.360 < SEAT_Z - 0.280,
        details="skirt bottom is below bowl bottom",
    )

    # --- Rear pod panel seam is visible on top of pod. ---
    ctx.check(
        "rear pod exists (concealed cistern housing)",
        POD_TOP_Z > SEAT_Z + 0.20,
        details=f"pod top z={POD_TOP_Z}, seat z={SEAT_Z}",
    )

    # --- Bowl is wall-hung (does not reach the floor). ---
    bowl_min_z = SEAT_Z - 0.360  # skirt bottom
    ctx.check(
        "bowl is wall-hung (does not reach the floor)",
        bowl_min_z > 0.02,
        details=f"min z of bowl+skirt = {bowl_min_z}",
    )
    ctx.check(
        "seat top is near 0.40 m above floor",
        0.34 < SEAT_Z < 0.46,
        details=f"seat top z = {SEAT_Z}",
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

    # --- Hinge barrels are visible behind the seat/lid. ---
    ctx.check(
        "seat has visible hinge barrels",
        seat.get_visual("seat_hinge_barrels") is not None,
        details="seat_hinge_barrels visual not found",
    )
    ctx.check(
        "lid has visible hinge barrels",
        lid.get_visual("lid_hinge_barrels") is not None,
        details="lid_hinge_barrels visual not found",
    )

    # --- Both flush buttons press down (prismatic, -Z axis). ---
    for name, part_obj, joint in (
        ("large", btn_large, btn_large_joint),
        ("small", btn_small, btn_small_joint),
    ):
        z0 = ctx.part_world_position(part_obj)[2]
        with ctx.pose({joint: 0.006}):
            z1 = ctx.part_world_position(part_obj)[2]
        ctx.check(
            f"{name} flush button presses downward",
            z1 < z0 - 0.004,
            details=f"rest z={z0}, pressed z={z1}",
        )
        ctx.check(
            f"{name} flush button is on top of rear pod (above seat)",
            ctx.part_world_position(part_obj)[2] > SEAT_Z + 0.20,
            details=f"button z={ctx.part_world_position(part_obj)[2]}",
        )
        # Verify prismatic axis is downward
        ctx.check(
            f"{name} flush button axis is downward (-Z)",
            joint.axis[2] < -0.9,
            details=f"joint axis={joint.axis}",
        )

    # --- Dual-flush: large button is bigger than the small button. ---
    large_dx = _ext(ctx.part_world_aabb(btn_large))[0]
    small_dx = _ext(ctx.part_world_aabb(btn_small))[0]
    ctx.check(
        "dual flush: large button is larger than the small button",
        large_dx > small_dx + 0.005,
        details=f"large dx={large_dx}, small dx={small_dx}",
    )

    # --- Flush buttons are on top of the pod (near centerline). ---
    large_x = ctx.part_world_position(btn_large)[0]
    small_x = ctx.part_world_position(btn_small)[0]
    ctx.check(
        "flush buttons are positioned on the pod top",
        abs((large_x + small_x) / 2.0 - (POD_X + POD_FRONT_X) / 2.0) < 0.03,
        details=f"large x={large_x}, small x={small_x}, pod center={(POD_X + POD_FRONT_X) / 2.0}",
    )

    return ctx.report()


object_model = build_object_model()
