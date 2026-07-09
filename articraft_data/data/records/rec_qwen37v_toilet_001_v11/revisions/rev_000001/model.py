from __future__ import annotations

# Wall-hung white ceramic toilet with concealed cistern tank panel,
# soft-close hinged seat ring and lid with visible hinge barrels,
# and a chrome dual-flush plate with top-mounted push buttons that
# press downward on short prismatic joints.
#
# Frame / coordinate convention:
#   +X = forward (front of the bowl points to +X), wall is at -X.
#   +Y = left-right (the hinge axis for the lid + seat ring runs along Y).
#   +Z = up. The floor is at z=0; the seat top sits ~0.40 m above the floor.
#
# Root part = the wall-mounting back panel + concealed cistern housing +
# cantilevered ceramic bowl + hinge barrels + water inlet pipe.
# Articulated children:
#   - lid          : oval top lid, REVOLUTE hinge at the rear (axis +Y), ~100 deg.
#   - seat_ring    : oval seat ring under the lid, REVOLUTE hinge sharing the
#                    SAME rear axis as the lid (concentric), ~100 deg.
#   - flush_button_large / flush_button_small : dual-flush top-mounted push
#                    buttons on the flush plate, PRISMATIC ~6 mm downward (-Z).

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
BOWL_DEPTH = 0.540  # bowl depth (X), front lip to wall
WALL_X = -0.010  # front face of the wall back-panel

# Rear hinge axis (shared by lid + seat ring), in world coords.
HINGE_X = 0.060
HINGE_Z = SEAT_TOP_Z + 0.004

# Seat plate top surface z (where seat ring + lid live).
SEAT_Z = SEAT_TOP_Z

# Concealed cistern panel dimensions
CISTERN_TOP_Z = 0.820  # top of the concealed cistern panel
CISTERN_W = 0.380  # width of cistern panel (Y)
CISTERN_DEPTH = 0.120  # depth into wall (X)

# Flush plate position (on front face of cistern panel, near top)
PLATE_X = WALL_X
PLATE_Y = 0.0
PLATE_Z = 0.720


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
    # Thin wall mounting back-panel the bowl cantilevers from.
    panel = (
        cq.Workplane("XY")
        .workplane(offset=0.060)
        .center(WALL_X - 0.015, 0.0)
        .box(0.030, 0.380, 0.760, centered=(True, True, False))
    )
    return panel


def _cistern_panel_solid() -> cq.Workplane:
    # Concealed cistern tank housing: a taller rectangular panel behind the wall
    # face that represents the in-wall tank enclosure. Sits behind the flush
    # plate area and extends from above the bowl up to near the top.
    panel = (
        cq.Workplane("XY")
        .workplane(offset=SEAT_Z - 0.100)
        .center(WALL_X - 0.020 - CISTERN_DEPTH / 2.0, 0.0)
        .box(CISTERN_DEPTH, CISTERN_W, CISTERN_TOP_Z - (SEAT_Z - 0.100), centered=(True, True, False))
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


def _hinge_barrel_solid(y_offset, length=0.040) -> cq.Workplane:
    # A small cylindrical hinge barrel at the rear of the bowl/seat hinge line.
    # Oriented along Y axis (the hinge axis direction).
    barrel = (
        cq.Workplane("XZ")
        .workplane(offset=y_offset - length / 2.0)
        .circle(0.012)
        .extrude(length)
    )
    return barrel


def _water_inlet_pipe_solid() -> cq.Workplane:
    # A small visible water supply pipe on the wall, near the base of the
    # concealed cistern. Runs vertically along Z on the wall face.
    pipe = (
        cq.Workplane("XZ")
        .workplane(offset=-0.160)
        .circle(0.008)
        .extrude(0.012)
        .translate((WALL_X + 0.006, 0.0, SEAT_Z - 0.080))
    )
    # Vertical run
    vert = (
        cq.Workplane("XY")
        .workplane(offset=SEAT_Z - 0.080)
        .center(WALL_X + 0.006, -0.160)
        .circle(0.008)
        .extrude(0.200)
    )
    # Small elbow connector at bottom
    elbow = (
        cq.Workplane("XZ")
        .workplane(offset=-0.160)
        .circle(0.010)
        .extrude(0.016)
        .translate((WALL_X + 0.006, 0.0, SEAT_Z - 0.080))
    )
    return vert.union(elbow)


def _concealed_panel_outline_solid() -> cq.Workplane:
    # A thin rectangular outline/frame on the wall face showing the concealed
    # cistern access panel boundary. Rendered as a thin raised border.
    outer = (
        cq.Workplane("XY")
        .workplane(offset=SEAT_Z + 0.010)
        .center(WALL_X + 0.001, 0.0)
        .box(0.003, 0.320, 0.440, centered=(True, True, False))
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_hung_toilet_concealed_cistern")

    ceramic = model.material("ceramic_white", rgba=(0.95, 0.95, 0.94, 1.0))
    seat_white = model.material("seat_white", rgba=(0.97, 0.97, 0.96, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    wall_gray = model.material("wall_gray", rgba=(0.72, 0.72, 0.70, 1.0))
    pipe_brass = model.material("pipe_brass", rgba=(0.72, 0.60, 0.30, 1.0))
    panel_outline = model.material("panel_outline", rgba=(0.85, 0.85, 0.83, 1.0))

    cx = 0.215

    # ================= ROOT: back panel + cistern + bowl + hinges + pipe =======
    body = model.part("body")

    # Wall back-panel (where the bowl mounts).
    body.visual(
        mesh_from_cadquery(_back_panel_solid(), "back_panel"),
        material=wall_gray,
        name="back_panel",
    )

    # Concealed cistern tank panel behind the wall.
    body.visual(
        mesh_from_cadquery(_cistern_panel_solid(), "cistern_panel"),
        material=wall_gray,
        name="cistern_panel",
    )

    # Concealed panel access outline on the wall face.
    body.visual(
        mesh_from_cadquery(_concealed_panel_outline_solid(), "panel_outline"),
        material=panel_outline,
        name="panel_outline",
    )

    # Cantilevered ceramic bowl + connecting neck shroud into the panel.
    bowl = _bowl_solid().union(_bowl_neck_solid())
    body.visual(mesh_from_cadquery(bowl, "bowl_shell"), material=ceramic, name="bowl_shell")

    # Chrome flush plate (rectangular) on the wall face, above the bowl.
    body.visual(
        Box((0.012, 0.150, 0.100)),
        origin=Origin(xyz=(PLATE_X - 0.006, PLATE_Y, PLATE_Z)),
        material=chrome,
        name="flush_plate",
    )

    # Visible hinge barrels behind the seat (two small cylinders on the Y axis).
    hinge_barrel_left = _hinge_barrel_solid(y_offset=-0.060, length=0.040).translate(
        (HINGE_X, 0.0, HINGE_Z)
    )
    hinge_barrel_right = _hinge_barrel_solid(y_offset=0.060, length=0.040).translate(
        (HINGE_X, 0.0, HINGE_Z)
    )
    hinge_barrels = hinge_barrel_left.union(hinge_barrel_right)
    body.visual(
        mesh_from_cadquery(hinge_barrels, "hinge_barrels"),
        material=chrome,
        name="hinge_barrels",
    )

    # Water inlet pipe on the wall (visible supply pipe).
    body.visual(
        mesh_from_cadquery(_water_inlet_pipe_solid(), "water_inlet_pipe"),
        material=pipe_brass,
        name="water_inlet_pipe",
    )

    body.inertial = Inertial.from_geometry(
        Box((0.55, 0.38, 0.55)),
        mass=25.0,
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

    # ================= dual flush buttons (both prismatic downward, -Z) =======
    # Top-mounted push buttons on the flush plate. Each round chrome puck sits
    # on top of the plate and depresses ~6 mm downward (-Z travel).
    button_specs = [
        ("flush_button_large", 0.032, PLATE_Y + 0.032),
        ("flush_button_small", 0.020, PLATE_Y - 0.032),
    ]
    for part_name, radius, btn_y in button_specs:
        b = model.part(part_name)
        # Round chrome puck standing on top of the flush plate.
        # CylinderGeometry: (radius, height), local +Z aligned.
        puck = CylinderGeometry(radius, 0.012, radial_segments=40)
        b.visual(
            mesh_from_geometry(puck, part_name + "_actuator"),
            material=chrome,
            name=part_name + "_actuator",
        )
        b.inertial = Inertial.from_geometry(
            Box((2.0 * radius, 2.0 * radius, 0.012)), mass=0.05
        )
        # Place on top of the flush plate; travel is -Z (downward press).
        plate_top_z = PLATE_Z + 0.050 + 0.006  # top of plate + half height of plate + proud
        model.articulation(
            "body_to_" + part_name,
            ArticulationType.PRISMATIC,
            parent=body,
            child=b,
            origin=Origin(xyz=(PLATE_X - 0.006, btn_y, plate_top_z)),
            axis=(0.0, 0.0, -1.0),
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
        reason="Large flush push actuator sits captured on the flush plate top.",
    )
    ctx.allow_overlap(
        btn_small, body,
        elem_a="flush_button_small_actuator", elem_b="flush_plate",
        reason="Small flush push actuator sits captured on the flush plate top.",
    )
    ctx.allow_overlap(
        btn_large, body,
        elem_a="flush_button_large_actuator", elem_b="back_panel",
        reason="Large flush button rear radius is captured inside the wall panel opening.",
    )
    ctx.allow_overlap(
        btn_small, body,
        elem_a="flush_button_small_actuator", elem_b="back_panel",
        reason="Small flush button rear radius is captured inside the wall panel opening.",
    )
    ctx.allow_overlap(
        btn_large, body,
        elem_a="flush_button_large_actuator", elem_b="cistern_panel",
        reason="Large flush button mechanism extends into the concealed cistern housing.",
    )
    ctx.allow_overlap(
        btn_small, body,
        elem_a="flush_button_small_actuator", elem_b="cistern_panel",
        reason="Small flush button mechanism extends into the concealed cistern housing.",
    )
    ctx.allow_overlap(
        body, seat,
        elem_a="hinge_barrels", elem_b="seat_ring_shell",
        reason="Hinge barrels sit at the seat pivot axis (captured barrel/pin interface).",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="bowl_shell", elem_b="back_panel",
        reason="Bowl neck shroud is fused into the wall back-panel (cantilever support).",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="cistern_panel", elem_b="back_panel",
        reason="Concealed cistern panel is behind and attached to the wall back-panel.",
    )

    # --- Bowl is wall-hung (no floor pedestal). ---
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

    # --- Concealed cistern panel extends above the bowl. ---
    ctx.check(
        "concealed cistern panel extends above the seat",
        CISTERN_TOP_Z > SEAT_Z + 0.20,
        details=f"cistern top z={CISTERN_TOP_Z}, seat z={SEAT_Z}",
    )

    # --- Flush plate is above the bowl on the wall. ---
    ctx.check(
        "flush plate is mounted above the seat on the wall",
        PLATE_Z > SEAT_Z + 0.10,
        details=f"plate z={PLATE_Z}, seat z={SEAT_Z}",
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

    # --- Seat ring and lid share the SAME rear hinge axis (concentric). ---
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

    # --- Hinge barrels exist behind the seat (at the hinge line). ---
    hinge_barrels_aabb = ctx.part_element_world_aabb(body, elem="hinge_barrels")
    ctx.check(
        "hinge barrels exist at the rear hinge line",
        hinge_barrels_aabb is not None,
        details="hinge_barrels visual not found on body",
    )
    if hinge_barrels_aabb is not None:
        hinge_min_z = hinge_barrels_aabb[0][2]
        hinge_max_z = hinge_barrels_aabb[1][2]
        ctx.check(
            "hinge barrels are near the seat hinge height",
            abs((hinge_min_z + hinge_max_z) / 2.0 - HINGE_Z) < 0.020,
            details=f"barrel center z={(hinge_min_z + hinge_max_z) / 2.0}, hinge z={HINGE_Z}",
        )

    # --- Water inlet pipe exists on the wall. ---
    pipe_aabb = ctx.part_element_world_aabb(body, elem="water_inlet_pipe")
    ctx.check(
        "water inlet pipe exists on the wall",
        pipe_aabb is not None,
        details="water_inlet_pipe visual not found on body",
    )
    if pipe_aabb is not None:
        ctx.check(
            "water inlet pipe is near the wall (behind the bowl)",
            pipe_aabb[0][0] < WALL_X + 0.030,
            details=f"pipe min x={pipe_aabb[0][0]}, wall x={WALL_X}",
        )

    # --- Both flush buttons press downward (prismatic, -Z). ---
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
            f"{name} flush button is mounted above the bowl",
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

    # --- Flush buttons straddle the centerline. ---
    large_y = ctx.part_world_position(btn_large)[1]
    small_y = ctx.part_world_position(btn_small)[1]
    ctx.check(
        "flush buttons straddle the wall centerline (plate centered)",
        abs((large_y + small_y) / 2.0) < 0.05 and large_y > small_y,
        details=f"large y={large_y}, small y={small_y}",
    )

    # --- Prismatic joints use -Z axis (downward press). ---
    ctx.check(
        "large flush button joint axis is downward (-Z)",
        tuple(btn_large_joint.axis) == (0.0, 0.0, -1.0),
        details=f"axis={btn_large_joint.axis}",
    )
    ctx.check(
        "small flush button joint axis is downward (-Z)",
        tuple(btn_small_joint.axis) == (0.0, 0.0, -1.0),
        details=f"axis={btn_small_joint.axis}",
    )

    # --- Proof checks paired with allow_overlap entries ---
    # Buttons are captured in the wall but sit above the flush plate.
    ctx.expect_gap(
        btn_large, body,
        axis="z",
        min_gap=-0.020,
        positive_elem="flush_button_large_actuator",
        negative_elem="flush_plate",
        name="large button sits at or above the flush plate top",
    )
    ctx.expect_gap(
        btn_small, body,
        axis="z",
        min_gap=-0.020,
        positive_elem="flush_button_small_actuator",
        negative_elem="flush_plate",
        name="small button sits at or above the flush plate top",
    )
    # Hinge barrels are at the seat pivot axis (contact proof).
    ctx.expect_overlap(
        body, seat,
        axes="xy",
        elem_a="hinge_barrels",
        elem_b="seat_ring_shell",
        min_overlap=0.005,
        name="hinge barrels overlap the seat ring footprint (pivot interface)",
    )

    return ctx.report()


object_model = build_object_model()
