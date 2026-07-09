from __future__ import annotations

# Bidet-toilet combo: wall-hung white ceramic toilet with a soft-close seat
# ring and lid, visible hinge barrels behind the seat, a side-mounted
# flushometer lever handle (revolute pivot), and a bidet control knob
# (continuous rotation) on the right side.
#
# Frame / coordinate convention:
#   +X = forward (front of the bowl points to +X), wall is at -X.
#   +Y = left-right (+Y is left, -Y is right when facing the toilet).
#   +Z = up. The floor is at z=0; the seat top sits ~0.40 m above the floor.
#
# Root part = the wall-mounting back panel + the cantilevered ceramic bowl +
# fixed hinge barrels + bidet nozzle housing.
# Articulated children:
#   - lid           : oval top lid, REVOLUTE hinge at the rear (axis +Y), ~100 deg.
#   - seat_ring     : oval seat ring under the lid, REVOLUTE same rear axis.
#   - flush_handle  : side lever handle on the right (-Y), REVOLUTE pivot to
#                     pull downward (~45 deg travel).
#   - bidet_knob    : rotary control knob on the right side, CONTINUOUS rotation.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Inertial,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
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
WALL_X = -0.010

# Rear hinge axis (shared by lid + seat ring), in world coords.
HINGE_X = 0.060
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


def _bowl_solid() -> cq.Workplane:
    cx = 0.215
    top_z = SEAT_Z
    bottom_z = SEAT_Z - 0.300

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

    shelf = _oval_ring(
        top_z - 0.022, rx_out=0.200, ry_out=0.195, rx_in=0.150, ry_in=0.150, thick=0.022, cx=cx
    )
    bowl = bowl.union(shelf)

    return bowl


def _back_panel_solid() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .workplane(offset=0.060)
        .center(WALL_X - 0.015, 0.0)
        .box(0.030, 0.380, 0.760, centered=(True, True, False))
    )
    return panel


def _bowl_neck_solid() -> cq.Workplane:
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


def _hinge_barrel_solid() -> cq.Workplane:
    """A visible hinge barrel: a short cylinder with knuckle detail, oriented
    along Y (the seat hinge axis). Built around local origin, axis along Y."""
    barrel_len = 0.045
    barrel_r = 0.012
    # Main barrel cylinder along Y
    barrel = (
        cq.Workplane("XZ")
        .workplane(offset=-barrel_len / 2.0)
        .circle(barrel_r)
        .extrude(barrel_len)
    )
    # Add knuckle rings at each end for visual detail
    knuckle = (
        cq.Workplane("XZ")
        .workplane(offset=-barrel_len / 2.0)
        .circle(barrel_r + 0.003)
        .extrude(0.008)
    )
    knuckle2 = (
        cq.Workplane("XZ")
        .workplane(offset=barrel_len / 2.0 - 0.008)
        .circle(barrel_r + 0.003)
        .extrude(0.008)
    )
    return barrel.union(knuckle).union(knuckle2)


def _flush_handle_solid() -> cq.Workplane:
    """A flushometer-style lever handle: a flat lever arm extending outward
    from a pivot boss. Built in handle-local frame with pivot at origin,
    arm extending along +X (outward from wall), handle grip at the end."""
    # Pivot boss (cylindrical base that mounts to the wall plate)
    boss = (
        cq.Workplane("XY")
        .circle(0.018)
        .extrude(0.015)
    )
    # Lever arm: flat bar extending outward (along +X from pivot)
    arm = (
        cq.Workplane("XY")
        .workplane(offset=0.003)
        .center(0.050, 0.0)
        .box(0.100, 0.018, 0.010, centered=(True, True, False))
    )
    # Grip end: slightly thicker rounded end
    grip = (
        cq.Workplane("XY")
        .workplane(offset=0.002)
        .center(0.098, 0.0)
        .box(0.024, 0.024, 0.014, centered=(True, True, False))
    )
    return boss.union(arm).union(grip)


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bidet_toilet_combo")

    ceramic = model.material("ceramic_white", rgba=(0.95, 0.95, 0.94, 1.0))
    seat_white = model.material("seat_white", rgba=(0.97, 0.97, 0.96, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    wall_gray = model.material("wall_gray", rgba=(0.72, 0.72, 0.70, 1.0))
    dark_chrome = model.material("dark_chrome", rgba=(0.55, 0.57, 0.60, 1.0))

    cx = 0.215

    # ================= ROOT: back panel + bowl + hinge barrels ==============
    body = model.part("body")

    # Wall back-panel
    body.visual(
        mesh_from_cadquery(_back_panel_solid(), "back_panel"),
        material=wall_gray,
        name="back_panel",
    )

    # Cantilevered ceramic bowl + connecting neck shroud
    bowl = _bowl_solid().union(_bowl_neck_solid())
    body.visual(mesh_from_cadquery(bowl, "bowl_shell"), material=ceramic, name="bowl_shell")

    # Bidet nozzle housing: small raised bump at the rear of the bowl shelf
    # (where the bidet spray nozzle is housed in a real bidet-toilet combo).
    # Positioned below the seat ring to avoid interference.
    nozzle_housing = (
        cq.Workplane("XY")
        .workplane(offset=SEAT_Z - 0.032)
        .center(HINGE_X + 0.020, 0.0)
        .box(0.040, 0.045, 0.020, centered=(True, True, False))
    )
    body.visual(
        mesh_from_cadquery(nozzle_housing, "bidet_nozzle_housing"),
        material=ceramic,
        name="bidet_nozzle_housing",
    )

    # Flush plate (rectangular chrome plate on the wall, above the bowl)
    plate_x = WALL_X
    plate_y = 0.0
    plate_z = 0.620
    body.visual(
        Box((0.012, 0.150, 0.230)),
        origin=Origin(xyz=(plate_x - 0.006, plate_y, plate_z)),
        material=chrome,
        name="flush_plate",
    )

    # Visible hinge barrels: two barrel cylinders at the rear of the seat
    # shelf, where the seat and lid pivot. These are fixed to the body.
    hinge_barrel_geo = _hinge_barrel_solid()
    # Left barrel (at +Y side)
    body.visual(
        mesh_from_cadquery(
            hinge_barrel_geo.translate((HINGE_X, 0.120, HINGE_Z)),
            "hinge_barrel_left",
        ),
        material=dark_chrome,
        name="hinge_barrel_left",
    )
    # Right barrel (at -Y side)
    body.visual(
        mesh_from_cadquery(
            hinge_barrel_geo.translate((HINGE_X, -0.120, HINGE_Z)),
            "hinge_barrel_right",
        ),
        material=dark_chrome,
        name="hinge_barrel_right",
    )

    # Side control panel: a small flat panel on the right side of the bowl
    # body where the bidet control knob mounts. Extends from the bowl neck
    # area outward to provide a mounting surface.
    knob_mount_x = 0.100
    knob_mount_y = -0.175
    knob_mount_z = SEAT_Z - 0.040
    side_panel = (
        cq.Workplane("XY")
        .workplane(offset=knob_mount_z - 0.035)
        .center(knob_mount_x, knob_mount_y + 0.015)
        .box(0.060, 0.030, 0.070, centered=(True, True, False))
    )
    body.visual(
        mesh_from_cadquery(side_panel, "side_control_panel"),
        material=ceramic,
        name="side_control_panel",
    )

    body.inertial = Inertial.from_geometry(
        Box((0.55, 0.38, 0.55)),
        mass=24.0,
        origin=Origin(xyz=(0.18, 0.0, SEAT_Z - 0.12)),
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

    # ================= flush handle (revolute lever on the right side) ======
    # A flushometer-style lever handle mounted on the right side of the flush
    # plate. It pivots downward (rotates around Z axis) to activate the flush.
    flush_handle = model.part("flush_handle")
    handle_geo = _flush_handle_solid()
    # The handle is built with arm along +X, boss at origin. Rotate it so the
    # arm points outward from the wall (+X direction) and is oriented
    # horizontally. We rotate it to face right (-Y) from the plate.
    flush_handle.visual(
        mesh_from_cadquery(
            handle_geo.rotate((0, 0, 0), (0, 0, 1), -90).translate((0.0, 0.0, 0.0)),
            "flush_handle_shell",
        ),
        material=chrome,
        name="flush_handle_shell",
    )
    flush_handle.inertial = Inertial.from_geometry(
        Box((0.12, 0.03, 0.02)),
        mass=0.15,
        origin=Origin(xyz=(0.05, 0.0, 0.007)),
    )
    # Articulation: pivot at the right edge of the flush plate.
    # The handle extends outward along +Y (to the right of the toilet when
    # facing it). Axis is +X so positive rotation pulls the handle down.
    # Actually, let's use axis along X (pointing into/out of wall) so that
    # the lever swings in the YZ plane. But the handle arm extends along -Y
    # (to the right side). Let's use axis along +X so positive q rotates
    # the handle tip downward (-Z).
    handle_pivot_x = 0.008  # forward of back panel front face (x=-0.01)
    handle_pivot_y = -0.090  # right side of the flush plate
    handle_pivot_z = plate_z
    model.articulation(
        "body_to_flush_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flush_handle,
        origin=Origin(xyz=(handle_pivot_x, handle_pivot_y, handle_pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=0.0, upper=math.radians(45.0)
        ),
    )

    # ================= bidet control knob (continuous rotation, right side) ==
    # A rotary control knob on the right side of the bowl body, used to
    # control bidet water pressure and spray mode.
    bidet_knob = model.part("bidet_knob")
    knob_geo = KnobGeometry(
        0.032,
        0.018,
        body_style="skirted",
        top_diameter=0.026,
        skirt=KnobSkirt(0.038, 0.005, flare=0.06, chamfer=0.001),
        grip=KnobGrip(style="fluted", count=16, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
    )
    # The knob is built around local Z axis. Rotate to face outward along -Y
    # (right side of toilet).
    knob_mesh = mesh_from_geometry(knob_geo, "bidet_knob_shell")
    bidet_knob.visual(
        knob_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="bidet_knob_shell",
    )
    bidet_knob.inertial = Inertial.from_geometry(
        Box((0.04, 0.04, 0.02)),
        mass=0.06,
    )
    # Mount the knob on the right side control panel (at -Y), at a
    # comfortable height near the seat level. The knob protrudes from the
    # outer face of the side panel (-Y direction).
    knob_mount_x_final = knob_mount_x
    knob_mount_y_final = knob_mount_y - 0.015  # outer face of the side panel
    knob_mount_z_final = knob_mount_z
    model.articulation(
        "body_to_bidet_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=bidet_knob,
        origin=Origin(xyz=(knob_mount_x_final, knob_mount_y_final, knob_mount_z_final)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    seat = object_model.get_part("seat_ring")
    lid = object_model.get_part("lid")
    flush_handle = object_model.get_part("flush_handle")
    bidet_knob = object_model.get_part("bidet_knob")

    seat_joint = object_model.get_articulation("body_to_seat_ring")
    lid_joint = object_model.get_articulation("body_to_lid")
    handle_joint = object_model.get_articulation("body_to_flush_handle")
    knob_joint = object_model.get_articulation("body_to_bidet_knob")

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
        elem_a="bowl_shell", elem_b="back_panel",
        reason="Bowl neck shroud is fused into the wall back-panel (cantilever support).",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="bidet_nozzle_housing", elem_b="bowl_shell",
        reason="Bidet nozzle housing is fused into the bowl rear shelf.",
    )
    ctx.allow_overlap(
        flush_handle, body,
        elem_a="flush_handle_shell", elem_b="flush_plate",
        reason="Flush handle pivot boss is captured in the flush plate mount.",
    )

    # --- Wall-hung (no floor pedestal) ---
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

    # --- Lid sits above seat when closed ---
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

    # --- Seat ring rotates on the same axis as the lid ---
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

    # --- Visible hinge barrels exist behind the seat ---
    body_visuals = [v.name for v in body.visuals]
    ctx.check(
        "visible hinge barrel (left) exists on the body",
        "hinge_barrel_left" in body_visuals,
        details=f"body visuals: {body_visuals}",
    )
    ctx.check(
        "visible hinge barrel (right) exists on the body",
        "hinge_barrel_right" in body_visuals,
        details=f"body visuals: {body_visuals}",
    )
    # Hinge barrels are near the hinge axis
    hinge_barrel_left_aabb = ctx.part_element_world_aabb(body, elem="hinge_barrel_left")
    ctx.check(
        "hinge barrel left is near the hinge axis height",
        hinge_barrel_left_aabb is not None
        and abs((hinge_barrel_left_aabb[0][2] + hinge_barrel_left_aabb[1][2]) / 2.0 - HINGE_Z) < 0.02,
        details=f"barrel center z={hinge_barrel_left_aabb}",
    )

    # --- Flush handle pivots (revolute joint, non-fixed) ---
    ctx.check(
        "flush handle has a revolute articulation",
        handle_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"joint type={handle_joint.articulation_type}",
    )
    handle_z0 = ctx.part_world_aabb(flush_handle)[1][2]  # top of handle at rest
    with ctx.pose({handle_joint: math.radians(30.0)}):
        handle_z1 = ctx.part_world_aabb(flush_handle)[1][2]
    ctx.check(
        "flush handle pivots downward when actuated",
        handle_z1 < handle_z0 - 0.01,
        details=f"rest top z={handle_z0}, actuated top z={handle_z1}",
    )
    # Handle is on the right side (-Y)
    handle_y = ctx.part_world_position(flush_handle)[1]
    ctx.check(
        "flush handle is on the right side of the toilet",
        handle_y < -0.03,
        details=f"handle y={handle_y}",
    )

    # --- Bidet control knob exists and rotates (continuous joint) ---
    ctx.check(
        "bidet knob has a continuous articulation",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint type={knob_joint.articulation_type}",
    )
    ctx.check(
        "bidet knob is on the right side of the toilet",
        ctx.part_world_position(bidet_knob)[1] < -0.03,
        details=f"knob y={ctx.part_world_position(bidet_knob)[1]}",
    )
    # Knob is near seat height (accessible)
    knob_z = ctx.part_world_position(bidet_knob)[2]
    ctx.check(
        "bidet knob is at an accessible height near the seat",
        SEAT_Z - 0.15 < knob_z < SEAT_Z + 0.05,
        details=f"knob z={knob_z}",
    )

    # --- Bidet nozzle housing exists ---
    ctx.check(
        "bidet nozzle housing exists on the body",
        "bidet_nozzle_housing" in body_visuals,
        details=f"body visuals: {body_visuals}",
    )

    # --- Flush plate is centered on the wall ---
    plate_aabb = ctx.part_element_world_aabb(body, elem="flush_plate")
    plate_center_y = (plate_aabb[0][1] + plate_aabb[1][1]) / 2.0
    ctx.check(
        "flush plate is centered on the wall (y~0)",
        abs(plate_center_y) < 0.02,
        details=f"flush plate center y={plate_center_y}",
    )

    return ctx.report()


object_model = build_object_model()
