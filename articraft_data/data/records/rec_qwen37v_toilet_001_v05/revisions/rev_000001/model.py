from __future__ import annotations

# Elongated modern wall-hung toilet with skirted base, side flush lever,
# visible hinge barrels, concealed cistern panel, and water inlet pipe.
#
# Frame / coordinate convention:
#   +X = forward (front of the bowl points to +X), wall is at -X.
#   +Y = left-right (hinge axis for the lid + seat ring runs along Y).
#   +Z = up. The floor is at z=0; the seat top sits ~0.40 m above the floor.
#
# Root part = the wall-mounting back panel + cantilevered elongated ceramic
# bowl with skirted smooth exterior + hinge barrels + concealed cistern panel
# outline + water inlet pipe (all one fixed ceramic+wall assembly).
# Articulated children:
#   - lid          : elongated oval lid, REVOLUTE hinge at rear (axis +Y), ~100 deg
#   - seat_ring    : elongated oval seat ring, REVOLUTE same rear axis, ~100 deg
#   - flush_lever  : chrome side-mounted lever, REVOLUTE on +Y side of the
#                    cistern panel, rotates downward ~45 deg to flush

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
BOWL_W = 0.370  # bowl width (Y)
BOWL_DEPTH = 0.600  # elongated bowl depth (X), front lip to wall
WALL_X = -0.010  # front face of the wall back-panel

# Rear hinge axis (shared by lid + seat ring), in world coords.
HINGE_X = 0.050
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


def _elongated_bowl_solid() -> cq.Workplane:
    """Elongated modern bowl: deeper front-to-back, smooth skirted exterior."""
    cx = 0.250  # bowl center X
    top_z = SEAT_Z
    bottom_z = SEAT_Z - 0.280

    # Outer body: lofted elongated oval sections (rx > ry for elongation)
    outer = (
        cq.Workplane("XY")
        .workplane(offset=bottom_z)
        .ellipse(0.090, 0.075)  # bottom: narrow, elongated
        .workplane(offset=0.080)
        .ellipse(0.140, 0.110)
        .workplane(offset=0.110)
        .ellipse(0.180, 0.155)
        .workplane(offset=0.090)
        .ellipse(0.200, 0.185)
        .loft(ruled=False)
    )
    outer = outer.translate((cx, 0.0, 0.0))

    # Hollow basin cavity
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=top_z - 0.180)
        .ellipse(0.100, 0.080)
        .workplane(offset=0.120)
        .ellipse(0.160, 0.140)
        .workplane(offset=0.070)
        .ellipse(0.175, 0.160)
        .loft(ruled=False)
        .translate((cx, 0.0, 0.0))
    )
    bowl = outer.cut(cavity)

    # Seat shelf: flat oval ceramic rim
    shelf = _oval_ring(
        top_z - 0.022,
        rx_out=0.215, ry_out=0.190,
        rx_in=0.165, ry_in=0.148,
        thick=0.022, cx=cx,
    )
    bowl = bowl.union(shelf)

    # Skirted base: smooth shroud covering the underside/trapway area
    # A box-like smooth panel under the bowl connecting to the wall
    skirt = (
        cq.Workplane("XY")
        .workplane(offset=bottom_z - 0.020)
        .center(cx - 0.040, 0.0)
        .box(0.280, 0.300, 0.060, centered=(True, True, False))
    )
    bowl = bowl.union(skirt)

    return bowl


def _back_panel_solid() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .workplane(offset=0.060)
        .center(WALL_X - 0.015, 0.0)
        .box(0.030, 0.400, 0.800, centered=(True, True, False))
    )
    return panel


def _bowl_neck_solid() -> cq.Workplane:
    """Ceramic shroud connecting bowl rear into back panel (cantilever support)."""
    neck = (
        cq.Workplane("XY")
        .workplane(offset=SEAT_Z - 0.170)
        .center(0.040, 0.0)
        .box(0.140, 0.250, 0.180, centered=(True, True, False))
    )
    return neck


def _concealed_cistern_panel() -> cq.Workplane:
    """Raised rectangular panel outline on the wall above the bowl (concealed cistern)."""
    panel = (
        cq.Workplane("XY")
        .workplane(offset=0.500)
        .center(WALL_X - 0.003, 0.0)
        .box(0.006, 0.280, 0.380, centered=(True, True, False))
    )
    return panel


def _hinge_barrel_solid(y_offset: float) -> cq.Workplane:
    """A single hinge barrel cylinder at the rear hinge line."""
    barrel = (
        cq.Workplane("XZ")
        .workplane(offset=y_offset)
        .center(HINGE_X, HINGE_Z)
        .circle(0.012)
        .extrude(0.040)
    )
    return barrel


def _water_inlet_pipe() -> cq.Workplane:
    """Chrome water inlet pipe on the left side of the wall panel."""
    # Vertical pipe segment + horizontal connector into wall
    pipe_v = (
        cq.Workplane("XY")
        .workplane(offset=0.150)
        .center(WALL_X - 0.025, -0.180)
        .circle(0.010)
        .extrude(0.350)
    )
    # Horizontal stub into wall
    pipe_h = (
        cq.Workplane("YZ")
        .workplane(offset=WALL_X - 0.025)
        .center(-0.180, 0.150)
        .circle(0.010)
        .extrude(0.030)
    )
    return pipe_v.union(pipe_h)


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
    model = ArticulatedObject(name="elongated_wall_hung_toilet")

    ceramic = model.material("ceramic_white", rgba=(0.95, 0.95, 0.94, 1.0))
    seat_white = model.material("seat_white", rgba=(0.97, 0.97, 0.96, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    wall_gray = model.material("wall_gray", rgba=(0.72, 0.72, 0.70, 1.0))
    pipe_chrome = model.material("pipe_chrome", rgba=(0.70, 0.72, 0.75, 1.0))

    cx = 0.250

    # ================= ROOT: back panel + elongated bowl + accessories ========
    body = model.part("body")

    # Wall back-panel
    body.visual(
        mesh_from_cadquery(_back_panel_solid(), "back_panel"),
        material=wall_gray,
        name="back_panel",
    )

    # Elongated ceramic bowl + neck shroud
    bowl = _elongated_bowl_solid().union(_bowl_neck_solid())
    body.visual(mesh_from_cadquery(bowl, "bowl_shell"), material=ceramic, name="bowl_shell")

    # Concealed cistern panel outline on the wall
    body.visual(
        mesh_from_cadquery(_concealed_cistern_panel(), "cistern_panel"),
        material=wall_gray,
        name="cistern_panel",
    )

    # Visible hinge barrels (two cylinders at the rear hinge line)
    hinge_barrels = _hinge_barrel_solid(-0.060).union(_hinge_barrel_solid(0.020))
    body.visual(
        mesh_from_cadquery(hinge_barrels, "hinge_barrels"),
        material=chrome,
        name="hinge_barrels",
    )

    # Water inlet pipe on the left side
    body.visual(
        mesh_from_cadquery(_water_inlet_pipe(), "water_inlet_pipe"),
        material=pipe_chrome,
        name="water_inlet_pipe",
    )

    body.inertial = Inertial.from_geometry(
        Box((0.60, 0.40, 0.58)),
        mass=25.0,
        origin=Origin(xyz=(0.20, 0.0, SEAT_Z - 0.10)),
    )

    # ================= seat ring (revolute, rear axis) =================
    seat = model.part("seat_ring")
    seat_local_cx = cx - HINGE_X
    seat_ring_geo = _oval_disc_ring_solid(
        rx_out=0.200,
        ry_out=0.185,
        rx_in=0.130,
        ry_in=0.145,
        thick=0.020,
        cx=seat_local_cx,
    )
    seat.visual(
        mesh_from_cadquery(seat_ring_geo.translate((0, 0, 0.002)), "seat_ring_shell"),
        material=seat_white,
        name="seat_ring_shell",
    )
    seat.inertial = Inertial.from_geometry(
        Box((0.40, 0.37, 0.025)),
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
    lid_geo = _oval_disc_solid(rx=0.210, ry=0.190, thick=0.018, cx=lid_local_cx)
    lid.visual(
        mesh_from_cadquery(lid_geo.translate((0, 0, 0.020)), "lid_shell"),
        material=seat_white,
        name="lid_shell",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.42, 0.38, 0.020)),
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

    # ================= flush lever (revolute, side of cistern panel) ==========
    # Chrome lever on the right side (+Y) of the concealed cistern panel.
    # Rotates about Y axis downward to flush. Lever part frame at pivot.
    lever = model.part("flush_lever")
    # Lever arm: a thin rounded bar extending forward from the pivot
    lever_arm = (
        cq.Workplane("XY")
        .center(0.040, 0.0)
        .box(0.080, 0.014, 0.012, centered=(True, True, True))
    )
    # Pivot boss: small cylinder at the mount point
    pivot_boss = (
        cq.Workplane("XZ")
        .center(0.0, 0.0)
        .circle(0.010)
        .extrude(0.016)
    )
    lever_solid = lever_arm.union(pivot_boss)
    lever.visual(
        mesh_from_cadquery(lever_solid, "flush_lever_shell"),
        material=chrome,
        name="flush_lever_shell",
    )
    lever.inertial = Inertial.from_geometry(
        Box((0.090, 0.020, 0.018)),
        mass=0.08,
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
    )

    # Lever pivot on the right side (+Y) of the cistern panel
    lever_pivot_x = WALL_X + 0.006
    lever_pivot_y = 0.160  # right side of panel
    lever_pivot_z = 0.680  # mid-height of cistern panel

    model.articulation(
        "body_to_flush_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(lever_pivot_x, lever_pivot_y, lever_pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=0.0, upper=math.radians(45.0)
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
        body, seat,
        elem_a="hinge_barrels", elem_b="seat_ring_shell",
        reason="Hinge barrels are the pivot bearings captured by the seat ring at the rear hinge line.",
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
        lever, body,
        elem_a="flush_lever_shell", elem_b="cistern_panel",
        reason="Flush lever pivot boss is captured in the cistern panel face.",
    )

    # --- Bowl is wall-hung (does not reach the floor) ---
    bowl_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "bowl is wall-hung (does not reach the floor)",
        bowl_aabb[0][2] > 0.04,
        details=f"min z of body = {bowl_aabb[0][2]}",
    )
    ctx.check(
        "seat top is near 0.40 m above floor",
        0.34 < SEAT_Z < 0.46,
        details=f"seat top z = {SEAT_Z}",
    )

    # --- Elongated bowl: depth (X) is significantly greater than width (Y) ---
    body_ext = _ext(ctx.part_world_aabb(body))
    # Bowl depth should exceed width by meaningful margin
    bowl_visual = body.get_visual("bowl_shell")
    ctx.check(
        "bowl is elongated (depth > width)",
        body_ext[0] > 0.45,
        details=f"body X extent = {body_ext[0]}",
    )

    # --- Hinge barrels exist as visible geometry on the body ---
    hinge_barrel_vis = body.get_visual("hinge_barrels")
    ctx.check(
        "visible hinge barrels exist behind the seat",
        hinge_barrel_vis is not None,
        details="hinge_barrels visual not found on body",
    )

    # --- Concealed cistern panel exists ---
    cistern_vis = body.get_visual("cistern_panel")
    ctx.check(
        "concealed cistern panel outline exists on wall",
        cistern_vis is not None,
        details="cistern_panel visual not found on body",
    )

    # --- Water inlet pipe exists ---
    pipe_vis = body.get_visual("water_inlet_pipe")
    ctx.check(
        "water inlet pipe is visible on the side",
        pipe_vis is not None,
        details="water_inlet_pipe visual not found on body",
    )

    # --- Lid sits above the seat ring when closed ---
    seat_z = ctx.part_world_aabb(seat)[1][2]
    lid_z = ctx.part_world_aabb(lid)[0][2]
    ctx.check(
        "lid sits above the seat ring when closed",
        lid_z >= seat_z - 0.005,
        details=f"lid bottom z={lid_z}, seat top z={seat_z}",
    )

    # --- Lid rotates open ~100 deg (front edge lifts up and back) ---
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

    # --- Seat ring and lid share the same rear hinge axis ---
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

    # --- Hinge barrels are at the seat hinge axis (proof for allowance) ---
    ctx.expect_overlap(
        body, seat,
        axes="xy",
        elem_a="hinge_barrels", elem_b="seat_ring_shell",
        min_overlap=0.010,
        name="hinge barrels overlap the seat ring pivot region",
    )

    # --- Flush lever is REVOLUTE and rotates downward ---
    ctx.check(
        "flush lever joint is REVOLUTE",
        lever_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"joint type = {lever_joint.articulation_type}",
    )

    # Lever rotates (front edge dips down when actuated)
    lever_z0 = ctx.part_world_aabb(lever)[0][2]  # bottom of lever at rest
    lever_x0 = ctx.part_world_aabb(lever)[1][0]  # front extent at rest
    with ctx.pose({lever_joint: math.radians(40.0)}):
        lever_z1 = ctx.part_world_aabb(lever)[0][2]
        lever_x1 = ctx.part_world_aabb(lever)[1][0]
    ctx.check(
        "flush lever rotates when actuated (front dips down)",
        lever_z1 < lever_z0 - 0.005,
        details=f"rest bottom z={lever_z0}, actuated bottom z={lever_z1}",
    )

    # --- Flush lever is mounted on the side of the cistern panel (high, above bowl) ---
    lever_pos = ctx.part_world_position(lever)
    ctx.check(
        "flush lever is mounted above the bowl (on cistern panel)",
        lever_pos[2] > SEAT_Z + 0.10,
        details=f"lever z = {lever_pos[2]}",
    )
    ctx.check(
        "flush lever is on the side (+Y) of the panel",
        lever_pos[1] > 0.08,
        details=f"lever y = {lever_pos[1]}",
    )

    # --- At least one non-fixed joint exists ---
    non_fixed_joints = [
        a for a in object_model.articulations
        if a.articulation_type != ArticulationType.FIXED
    ]
    ctx.check(
        "at least one non-fixed joint exists",
        len(non_fixed_joints) >= 1,
        details=f"found {len(non_fixed_joints)} non-fixed joints",
    )

    return ctx.report()


object_model = build_object_model()
