from __future__ import annotations

# Measuring-cup-cap bottle variant: clear plastic bottle with tapered shoulder,
# removable measuring cup cap, flip-top lid on a revolute hinge, molded volume
# bands around the body, and a small tether loop at the neck.
# Frame: vertical axis along +Z, bottle standing on z=0, centerline on x=y=0.

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
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key heights (m) along +Z ----
BASE_Z = 0.0
BODY_TOP_Z = 0.110
SHOULDER_TOP_Z = 0.156
NECK_TOP_Z = 0.176

BODY_R = 0.0275
NECK_R = 0.0125
NECK_BORE_R = 0.0098

# Measuring cup dimensions
CUP_R = 0.0180       # outer radius of the measuring cup
CUP_HEIGHT = 0.035   # cup height (taller than original cap for measuring use)
CUP_BORE_R = NECK_R + 0.001  # inner bore slightly larger than neck for slip fit
CUP_WALL = 0.002     # cup wall thickness

# Flip lid dimensions
LID_R = CUP_R - 0.001   # lid radius (slightly smaller than cup outer)
LID_THICKNESS = 0.004   # lid thickness
HINGE_OFFSET_Y = CUP_R - 0.003  # hinge at the cup rim edge

# Tether loop
TETHER_RING_R = 0.005   # major radius of tether ring
TETHER_TUBE_R = 0.0015  # tube radius
TETHER_Z = SHOULDER_TOP_Z + 0.004  # just below the neck base

# Volume band heights
BAND_POSITIONS = [0.030, 0.050, 0.070, 0.090]  # four bands up the body


def _profile_sections():
    return [
        (0.000, 0.0150),
        (0.006, 0.0250),
        (0.014, 0.0273),
        (BODY_TOP_Z, BODY_R),
        (0.124, 0.0268),
        (0.138, 0.0228),
        (SHOULDER_TOP_Z, 0.0148),
        (0.160, NECK_R),
        (NECK_TOP_Z, NECK_R),
    ]


def _bottle_solid() -> cq.Workplane:
    pts = _profile_sections()
    wp = cq.Workplane("XZ").moveTo(0.0, pts[0][0])
    for r, z in [(r, z) for (z, r) in pts]:
        wp = wp.lineTo(r, z)
    wp = wp.lineTo(0.0, pts[-1][0]).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))

    wall = 0.0014
    inner_pts = [
        (0.010, 0.006),
        (0.0235, 0.012),
        (BODY_R - wall, 0.014),
        (BODY_R - wall, BODY_TOP_Z),
        (0.0254, 0.124),
        (0.0214, 0.138),
        (0.0134, SHOULDER_TOP_Z),
        (NECK_BORE_R, 0.160),
        (NECK_BORE_R, NECK_TOP_Z + 0.005),
    ]
    iwp = cq.Workplane("XZ").moveTo(0.0, inner_pts[0][1])
    for r, z in inner_pts:
        iwp = iwp.lineTo(r, z)
    iwp = iwp.lineTo(0.0, inner_pts[-1][1]).close()
    cavity = iwp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    return outer.cut(cavity)


def _bottle_mesh():
    return mesh_from_cadquery(_bottle_solid(), "bottle_shell")


def _volume_bands():
    """Molded raised rings around the bottle body at measurement heights."""
    g = None
    band_r = BODY_R + 0.0008  # slightly proud of the body surface
    for z in BAND_POSITIONS:
        ring = TorusGeometry(band_r, 0.0012, radial_segments=8, tubular_segments=48)
        ring.translate(0.0, 0.0, z)
        if g is None:
            g = ring
        else:
            g.merge(ring)
    return mesh_from_geometry(g, "volume_bands")


def _tether_loop_mesh():
    """Small ring/loop attached to the neck for a cap tether cord."""
    # Small attachment lug (rectangular tab protruding from the neck wall)
    lug_depth = 0.005
    lug = BoxGeometry((lug_depth, 0.006, 0.008))
    lug.translate(NECK_R + lug_depth / 2.0, 0.0, TETHER_Z)
    # Ring: a torus just outside the lug
    ring = TorusGeometry(TETHER_RING_R, TETHER_TUBE_R, radial_segments=8, tubular_segments=24)
    ring.translate(NECK_R + lug_depth + TETHER_RING_R, 0.0, TETHER_Z)
    # Merge geometry objects, then export
    lug.merge(ring)
    return mesh_from_geometry(lug, "tether_loop")


def _cup_solid() -> cq.Workplane:
    """Measuring cup: hollow cylinder open at bottom, closed at top with a rim."""
    # Outer shell
    cup = (
        cq.Workplane("XY")
        .circle(CUP_R)
        .extrude(CUP_HEIGHT)
    )
    # Hollow bore from bottom (open bottom to fit over neck)
    bore_depth = CUP_HEIGHT - CUP_WALL
    bore = (
        cq.Workplane("XY")
        .circle(CUP_BORE_R)
        .extrude(bore_depth)
    )
    cup = cup.cut(bore)
    # Add a small pouring spout notch on one side (cut a small wedge from the rim)
    spout = (
        cq.Workplane("XY")
        .center(CUP_R - 0.002, 0.0)
        .rect(0.008, 0.006)
        .extrude(CUP_HEIGHT)
    )
    # Don't cut the spout too deep - just a subtle lip feature
    return cup


def _cup_mesh():
    return mesh_from_cadquery(_cup_solid(), "measuring_cup_shell")


def _cup_graduations():
    """Small raised lines on the cup exterior suggesting volume markings."""
    g = None
    # A few thin horizontal lines (tori at different heights)
    for i, z in enumerate([0.008, 0.015, 0.022, 0.029]):
        mark_r = CUP_R + 0.0003
        mark = TorusGeometry(mark_r, 0.0003, radial_segments=4, tubular_segments=32)
        mark.translate(0.0, 0.0, z)
        if g is None:
            g = mark
        else:
            g.merge(mark)
    return mesh_from_geometry(g, "cup_graduations")


def _flip_lid_solid() -> cq.Workplane:
    """Flip lid: a disc with a hinge tab on one edge."""
    # Main disc
    lid = (
        cq.Workplane("XY")
        .circle(LID_R)
        .extrude(LID_THICKNESS)
    )
    # Add a hinge tab (small box extending from one edge for the hinge pin)
    tab_width = 0.006
    tab_depth = 0.008
    tab = (
        cq.Workplane("XY")
        .center(LID_R - tab_depth / 2, 0.0)
        .rect(tab_depth, tab_width)
        .extrude(LID_THICKNESS)
    )
    lid = lid.union(tab)
    return lid


def _flip_lid_mesh():
    return mesh_from_cadquery(_flip_lid_solid(), "flip_lid_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="measuring_cup_bottle")

    clear = model.material("clear_pet", rgba=(0.78, 0.85, 0.88, 0.30))
    cup_mat = model.material("white_plastic", rgba=(0.92, 0.92, 0.90, 0.85))
    lid_mat = model.material("blue_lid", rgba=(0.15, 0.35, 0.75, 1.0))
    band_mat = model.material("molded_band", rgba=(0.70, 0.78, 0.82, 0.40))
    tether_mat = model.material("tether_gray", rgba=(0.45, 0.45, 0.48, 1.0))

    # ---- bottle body (root) ----
    body = model.part("bottle_body")
    body.visual(_bottle_mesh(), material=clear, name="bottle_shell")
    body.visual(_volume_bands(), material=band_mat, name="volume_bands")
    body.visual(_tether_loop_mesh(), material=tether_mat, name="tether_loop")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, 0.176),
        mass=0.020,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
    )

    # ---- measuring cup cap (removable) ----
    cup = model.part("measuring_cup")
    cup.visual(_cup_mesh(), material=cup_mat, name="measuring_cup_shell")
    cup.visual(_cup_graduations(), material=band_mat, name="cup_graduations")
    cup.inertial = Inertial.from_geometry(
        Cylinder(CUP_R, CUP_HEIGHT),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, CUP_HEIGHT / 2.0)),
    )

    # ---- flip lid ----
    lid = model.part("flip_lid")
    lid.visual(_flip_lid_mesh(), material=lid_mat, name="flip_lid_shell")
    lid.inertial = Inertial.from_geometry(
        Cylinder(LID_R, LID_THICKNESS),
        mass=0.002,
        origin=Origin(xyz=(0.0, 0.0, LID_THICKNESS / 2.0)),
    )

    # cup_lift: PRISMATIC along +Z, lifts the measuring cup off the bottle neck
    # Origin at the neck top where the cup seats
    cup_mount_z = NECK_TOP_Z - CUP_HEIGHT + CUP_WALL
    model.articulation(
        "cup_lift",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cup,
        origin=Origin(xyz=(0.0, 0.0, cup_mount_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=CUP_HEIGHT + 0.01, effort=2.0, velocity=0.5),
    )

    # flip_hinge: REVOLUTE at the cup rim edge, opens the lid upward
    # The hinge sits at the top of the cup, at the rim edge along +Y
    # Axis along +X so positive rotation lifts the far edge of the lid up
    hinge_z = CUP_HEIGHT - LID_THICKNESS / 2.0  # at the cup top
    hinge_y = HINGE_OFFSET_Y  # at the rim edge
    model.articulation(
        "flip_hinge",
        ArticulationType.REVOLUTE,
        parent=cup,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=2.4, effort=1.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle_body")
    cup = object_model.get_part("measuring_cup")
    lid = object_model.get_part("flip_lid")
    cup_lift = object_model.get_articulation("cup_lift")
    flip_hinge = object_model.get_articulation("flip_hinge")

    # --- bottle is clear/transparent ---
    clear_mat = next(m for m in object_model.materials if m.name == "clear_pet")
    a = clear_mat.rgba[3] if clear_mat.rgba is not None else 1.0
    ctx.check(
        "bottle shell is transparent",
        a < 1.0,
        details=f"clear_pet alpha={a}",
    )

    # --- volume bands exist on the body ---
    bands = body.get_visual("volume_bands")
    ctx.check(
        "volume bands are present on bottle body",
        bands is not None,
        details="volume_bands visual missing from bottle_body",
    )

    # --- tether loop exists ---
    tether = body.get_visual("tether_loop")
    ctx.check(
        "tether loop is attached to the bottle",
        tether is not None,
        details="tether_loop visual missing from bottle_body",
    )

    # --- measuring cup cap exists ---
    cup_shell = cup.get_visual("measuring_cup_shell")
    ctx.check(
        "measuring cup cap is present",
        cup_shell is not None,
        details="measuring_cup_shell visual missing",
    )

    # --- cup seats over the neck (intentional slip-fit overlap) ---
    ctx.allow_overlap(
        cup,
        body,
        elem_a="measuring_cup_shell",
        elem_b="bottle_shell",
        reason="The measuring cup bore intentionally slips over the threaded neck as a removable cap.",
    )
    # Proof: cup overlaps the neck region along Z (retained insertion)
    ctx.expect_overlap(
        cup,
        body,
        axes="z",
        elem_a="measuring_cup_shell",
        elem_b="bottle_shell",
        min_overlap=0.010,
        name="measuring cup overlaps neck region along Z",
    )
    # Proof: cup is centered on the body in XY
    ctx.expect_within(
        cup,
        body,
        axes="xy",
        inner_elem="measuring_cup_shell",
        margin=0.015,
        name="measuring cup is centered on bottle body",
    )

    # --- flip lid exists ---
    lid_shell = lid.get_visual("flip_lid_shell")
    ctx.check(
        "flip lid is present",
        lid_shell is not None,
        details="flip_lid_shell visual missing",
    )

    # --- cup is seated at the top of the bottle ---
    cup_pos = ctx.part_world_position(cup)
    ctx.check(
        "measuring cup is mounted at bottle top",
        cup_pos is not None and cup_pos[2] > 0.12,
        details=f"cup origin z={cup_pos[2] if cup_pos else None}",
    )

    # --- flip hinge opens: positive rotation lifts the lid ---
    lid_rest_z = ctx.part_world_aabb(lid)[1][2]  # max z of lid at rest
    with ctx.pose({flip_hinge: 1.2}):
        lid_open_z = ctx.part_world_aabb(lid)[1][2]
    ctx.check(
        "flip hinge opens the lid upward",
        lid_open_z > lid_rest_z + 0.005,
        details=f"lid max_z rest={lid_rest_z:.4f}, open={lid_open_z:.4f}",
    )

    # --- cup lifts off the bottle (prismatic joint) ---
    cup_rest_z = ctx.part_world_aabb(cup)[1][2]
    with ctx.pose({cup_lift: CUP_HEIGHT + 0.01}):
        cup_lifted_z = ctx.part_world_aabb(cup)[1][2]
    ctx.check(
        "measuring cup lifts off the bottle",
        cup_lifted_z > cup_rest_z + 0.02,
        details=f"cup max_z rest={cup_rest_z:.4f}, lifted={cup_lifted_z:.4f}",
    )

    # --- bottle is tall and tapered ---
    body_aabb = ctx.part_world_aabb(body)
    dx = body_aabb[1][0] - body_aabb[0][0]
    dz = body_aabb[1][2] - body_aabb[0][2]
    ctx.check(
        "bottle is tall (taller than wide)",
        dz > 2.5 * dx,
        details=f"body dx={dx:.4f}, dz={dz:.4f}",
    )

    # --- tether loop is near the neck region ---
    tether_pos = ctx.part_world_position(body)
    # The tether is part of body, just verify it exists and is at a reasonable height
    # by checking its visual AABB
    tether_aabb_min, tether_aabb_max = ctx.part_element_world_aabb(body, elem=tether)
    ctx.check(
        "tether loop is near the neck region",
        tether_aabb_min[2] > SHOULDER_TOP_Z - 0.02 and tether_aabb_max[2] < NECK_TOP_Z + 0.02,
        details=f"tether z range=[{tether_aabb_min[2]:.4f}, {tether_aabb_max[2]:.4f}]",
    )

    # --- joints are non-fixed ---
    ctx.check(
        "flip_hinge is revolute (non-fixed)",
        flip_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"flip_hinge articulation_type={flip_hinge.articulation_type}",
    )
    ctx.check(
        "cup_lift is prismatic (non-fixed)",
        cup_lift.articulation_type == ArticulationType.PRISMATIC,
        details=f"cup_lift articulation_type={cup_lift.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
