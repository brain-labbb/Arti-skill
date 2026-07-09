from __future__ import annotations

# Bidet-toilet combo: wall-hung white ceramic toilet with soft-close seat/lid,
# side bidet control knob, flush lever on the tank side, visible hinge barrels,
# and a rear pod with tank-lid seam.
#
# Frame / coordinate convention:
#   +X = forward (front of the bowl points to +X), wall is at -X.
#   +Y = left-right (the hinge axis for the lid + seat ring runs along Y).
#   +Z = up. The floor is at z=0; the seat top sits ~0.40 m above the floor.
#
# Root part = the wall-mounting back panel + rear pod/tank enclosure + the
# cantilevered ceramic bowl + hinge barrels (one fixed ceramic+wall assembly).
# Articulated children:
#   - lid          : oval top lid, REVOLUTE hinge at the rear (axis +Y), ~100 deg.
#   - seat_ring    : oval seat ring under the lid, REVOLUTE same rear axis.
#   - flush_lever  : REVOLUTE lever on the right side (+Y) of the rear pod,
#                    pivots downward to flush, ~35 deg travel.
#   - bidet_knob   : CONTINUOUS rotary control knob on the right side of the
#                    bowl body for bidet spray adjustment.
#   - flush_button_large / flush_button_small : wall-mounted dual-flush
#                    actuator pair on the chrome flush plate (PRISMATIC).

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
SEAT_TOP_Z = 0.400  # top of the seat ring above the floor
BOWL_W = 0.360  # bowl width (Y)
BOWL_DEPTH = 0.540  # bowl depth (X), front lip to wall
WALL_X = -0.010  # front face of the wall back-panel

# Rear hinge axis (shared by lid + seat ring), in world coords.
HINGE_X = 0.060
HINGE_Z = SEAT_TOP_Z + 0.004

# Seat plate top surface z (where seat ring + lid live).
SEAT_Z = SEAT_TOP_Z

# Rear pod/tank enclosure dimensions
POD_W = 0.360  # pod width (Y)
POD_D = 0.120  # pod depth (X)
POD_H = 0.340  # pod height (Z)
POD_BOTTOM_Z = SEAT_Z - 0.040  # pod starts just below seat level
POD_CENTER_X = WALL_X - POD_D / 2.0 + 0.010  # slightly in front of wall


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


def _rear_pod_solid() -> cq.Workplane:
    """Rear pod/tank enclosure above and behind the bowl against the wall."""
    pod = (
        cq.Workplane("XY")
        .workplane(offset=POD_BOTTOM_Z)
        .center(POD_CENTER_X, 0.0)
        .box(POD_D, POD_W, POD_H, centered=(True, True, False))
    )
    return pod


def _pod_seam_solid() -> cq.Workplane:
    """Thin groove near the top of the rear pod to read as a tank-lid seam."""
    seam_z = POD_BOTTOM_Z + POD_H - 0.030
    seam = (
        cq.Workplane("XY")
        .workplane(offset=seam_z)
        .center(POD_CENTER_X + POD_D / 2.0 + 0.001, 0.0)
        .box(0.003, POD_W - 0.020, 0.003, centered=(True, True, False))
    )
    return seam


def _hinge_barrel_solid(y_offset: float) -> cq.Workplane:
    """A single hinge barrel cylinder at the rear hinge position."""
    barrel_len = 0.040
    barrel_r = 0.010
    barrel = (
        cq.Workplane("XZ")
        .workplane(offset=y_offset - barrel_len / 2.0)
        .center(HINGE_X, HINGE_Z)
        .circle(barrel_r)
        .extrude(barrel_len)
    )
    return barrel


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


def _flush_lever_solid() -> cq.Workplane:
    """A small flush lever arm that pivots on the right side of the rear pod."""
    # Lever arm: a thin rounded bar extending outward from the pivot
    lever = (
        cq.Workplane("XY")
        .workplane(offset=-0.006)
        .center(0.030, 0.0)
        .box(0.065, 0.014, 0.012, centered=(True, True, False))
    )
    # Pivot boss: a small cylinder at the origin
    boss = (
        cq.Workplane("XY")
        .workplane(offset=-0.008)
        .center(0.0, 0.0)
        .circle(0.012)
        .extrude(0.016)
    )
    return lever.union(boss)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bidet_toilet_combo")

    ceramic = model.material("ceramic_white", rgba=(0.95, 0.95, 0.94, 1.0))
    seat_white = model.material("seat_white", rgba=(0.97, 0.97, 0.96, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    wall_gray = model.material("wall_gray", rgba=(0.72, 0.72, 0.70, 1.0))
    dark_seam = model.material("seam_dark", rgba=(0.55, 0.55, 0.53, 1.0))
    knob_black = model.material("knob_black", rgba=(0.15, 0.15, 0.16, 1.0))

    cx = 0.215

    # ================= ROOT: back panel + bowl + rear pod + hinge barrels =====
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

    # Rear pod/tank enclosure
    body.visual(
        mesh_from_cadquery(_rear_pod_solid(), "rear_pod"),
        material=ceramic,
        name="rear_pod",
    )

    # Tank-lid seam (dark groove near top of pod)
    body.visual(
        mesh_from_cadquery(_pod_seam_solid(), "pod_seam"),
        material=dark_seam,
        name="pod_seam",
    )

    # Visible hinge barrels behind the seat (two barrels flanking the hinge axis)
    barrel_left = _hinge_barrel_solid(y_offset=-0.080)
    barrel_right = _hinge_barrel_solid(y_offset=0.080)
    body.visual(
        mesh_from_cadquery(barrel_left, "hinge_barrel_left"),
        material=chrome,
        name="hinge_barrel_left",
    )
    body.visual(
        mesh_from_cadquery(barrel_right, "hinge_barrel_right"),
        material=chrome,
        name="hinge_barrel_right",
    )

    # Chrome flush plate (wall-mounted, centered above the bowl)
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

    # ================= flush lever (revolute, on right side of rear pod) ======
    # The lever pivots on the right side (+Y) of the rear pod.
    # Pivot axis is along X so the lever swings downward (positive q = push down).
    lever = model.part("flush_lever")
    lever_geo = _flush_lever_solid()
    lever.visual(
        mesh_from_cadquery(lever_geo, "flush_lever_arm"),
        material=chrome,
        name="flush_lever_arm",
    )
    lever.inertial = Inertial.from_geometry(
        Box((0.07, 0.02, 0.02)),
        mass=0.08,
        origin=Origin(xyz=(0.03, 0.0, 0.0)),
    )
    # Lever pivot at the right side of the rear pod
    lever_pivot_x = POD_CENTER_X + POD_D / 2.0
    lever_pivot_y = POD_W / 2.0 + 0.005
    lever_pivot_z = POD_BOTTOM_Z + POD_H - 0.060
    model.articulation(
        "body_to_flush_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(lever_pivot_x, lever_pivot_y, lever_pivot_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=4.0, velocity=3.0, lower=0.0, upper=math.radians(35.0)
        ),
    )

    # ================= bidet control knob (continuous, right side of bowl) ====
    bidet = model.part("bidet_knob")
    knob_geo = KnobGeometry(
        0.032,
        0.018,
        body_style="skirted",
        top_diameter=0.026,
        skirt=KnobSkirt(0.038, 0.005, flare=0.06, chamfer=0.001),
        grip=KnobGrip(style="fluted", count=16, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
    )
    bidet.visual(
        mesh_from_geometry(knob_geo, "bidet_knob_cap"),
        material=knob_black,
        name="bidet_knob_cap",
    )
    bidet.inertial = Inertial.from_geometry(
        Box((0.04, 0.04, 0.02)),
        mass=0.04,
    )
    # Mount on the right side (+Y) of the bowl body, near the seat level
    knob_mount_x = 0.100
    knob_mount_y = BOWL_W / 2.0 + 0.010
    knob_mount_z = SEAT_Z - 0.060
    model.articulation(
        "body_to_bidet_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=bidet,
        origin=Origin(
            xyz=(knob_mount_x, knob_mount_y, knob_mount_z),
            rpy=(math.radians(90.0), 0.0, 0.0),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0),
    )

    # ================= dual flush buttons (both prismatic into wall) ==========
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
    lever = object_model.get_part("flush_lever")
    bidet = object_model.get_part("bidet_knob")
    btn_large = object_model.get_part("flush_button_large")
    btn_small = object_model.get_part("flush_button_small")

    seat_joint = object_model.get_articulation("body_to_seat_ring")
    lid_joint = object_model.get_articulation("body_to_lid")
    lever_joint = object_model.get_articulation("body_to_flush_lever")
    bidet_joint = object_model.get_articulation("body_to_bidet_knob")
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
    ctx.allow_overlap(
        body, body,
        elem_a="rear_pod", elem_b="back_panel",
        reason="Rear pod enclosure is mounted against the wall back-panel.",
    )
    ctx.allow_overlap(
        lever, body,
        elem_a="flush_lever_arm", elem_b="rear_pod",
        reason="Flush lever pivot boss is mounted into the rear pod side face.",
    )
    ctx.allow_overlap(
        bidet, body,
        elem_a="bidet_knob_cap", elem_b="bowl_shell",
        reason="Bidet control knob is mounted into the bowl body side panel.",
    )

    # --- Bowl is wall-hung (no floor pedestal) ---
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

    # --- Seat ring and lid share the same hinge axis ---
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

    # --- Seat ring rotates open ---
    seat_top_z0 = ctx.part_world_aabb(seat)[1][2]
    with ctx.pose({seat_joint: -math.radians(100.0)}):
        seat_top_z1 = ctx.part_world_aabb(seat)[1][2]
    ctx.check(
        "seat ring rotates open (lifts upward)",
        seat_top_z1 > seat_top_z0 + 0.05,
        details=f"closed top z={seat_top_z0}, open top z={seat_top_z1}",
    )

    # --- Flush lever rotates (push down to flush) ---
    lever_pos0 = ctx.part_world_position(lever)
    with ctx.pose({lever_joint: math.radians(35.0)}):
        lever_pos1 = ctx.part_world_position(lever)
    ctx.check(
        "flush lever rotates when actuated",
        lever_pos1 is not None and lever_pos0 is not None
        and abs(lever_pos1[0] - lever_pos0[0]) + abs(lever_pos1[2] - lever_pos0[2]) > 0.005,
        details=f"rest={lever_pos0}, actuated={lever_pos1}",
    )
    ctx.check(
        "flush lever is on the right side of the toilet (+Y)",
        lever_pos0[1] > 0.10,
        details=f"lever y={lever_pos0[1]}",
    )

    # --- Bidet control knob is on the right side and is a continuous joint ---
    knob_pos = ctx.part_world_position(bidet)
    ctx.check(
        "bidet knob is on the right side of the bowl (+Y)",
        knob_pos[1] > 0.10,
        details=f"knob y={knob_pos[1]}",
    )
    ctx.check(
        "bidet knob joint is continuous or revolute (non-fixed)",
        bidet_joint.articulation_type in (ArticulationType.CONTINUOUS, ArticulationType.REVOLUTE),
        details=f"joint type={bidet_joint.articulation_type}",
    )

    # --- Rear pod with seam is present on the body ---
    body_visuals = [v.name for v in body.visuals]
    ctx.check(
        "rear pod panel is present on body",
        "rear_pod" in body_visuals,
        details=f"body visuals={body_visuals}",
    )
    ctx.check(
        "tank-lid seam is visible on body",
        "pod_seam" in body_visuals,
        details=f"body visuals={body_visuals}",
    )

    # --- Visible hinge barrels are present ---
    ctx.check(
        "hinge barrels are visible behind the seat",
        "hinge_barrel_left" in body_visuals and "hinge_barrel_right" in body_visuals,
        details=f"body visuals={body_visuals}",
    )

    # --- Flush buttons still work ---
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

    return ctx.report()


object_model = build_object_model()
