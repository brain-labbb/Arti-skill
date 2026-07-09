from __future__ import annotations

# Clear plastic bottle variant with:
# - Swing-top stopper pivoting on side hinge arms (REVOLUTE)
# - Removable measuring cup cap (PRISMATIC)
# - Molded volume bands around the body
# - Raised spiral-like neck thread ridges
# Frame: vertical axis along +Z, bottle standing on z=0, centerline on x=y=0.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
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
BODY_TOP_Z = 0.110
SHOULDER_TOP_Z = 0.156
NECK_TOP_Z = 0.176

BODY_R = 0.0275
NECK_R = 0.0125
NECK_BORE_R = 0.0098

# Swing-top stopper
PIVOT_Z = NECK_TOP_Z - 0.010  # collar height on neck
ARM_DISC_Z = 0.014  # vertical offset from pivot to disc center (closed)
STOPPER_DISC_R = 0.0105
STOPPER_DISC_H = 0.004
WIRE_R = 0.0012
ARM_SPREAD_X = 0.010  # arm X offset from centerline

# Measuring cup
CUP_OUTER_R = 0.019
CUP_INNER_R = 0.015
CUP_HEIGHT = 0.032
CUP_TOP_WALL = 0.003
CUP_MOUNT_Z = NECK_TOP_Z - 0.002

# Volume bands
BAND_Z_LIST = [0.035, 0.055, 0.075]
BAND_MAJOR_R = BODY_R + 0.0003
BAND_TUBE_R = 0.0012

# Neck threads (spiral-like ridges)
THREAD_Z_LIST = [0.160, 0.164, 0.168, 0.172]
THREAD_MAJOR_R = NECK_R + 0.0002
THREAD_TUBE_R = 0.0010


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


def _bottle_solid():
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


def _volume_bands_mesh():
    g = None
    for z in BAND_Z_LIST:
        band = TorusGeometry(BAND_MAJOR_R, BAND_TUBE_R, radial_segments=8, tubular_segments=48)
        band.translate(0.0, 0.0, z)
        if g is None:
            g = band
        else:
            g.merge(band)
    return mesh_from_geometry(g, "volume_bands")


def _neck_threads_mesh():
    g = None
    offsets = [(0.0004, 0.0), (0.0, 0.0004), (-0.0004, 0.0), (0.0, -0.0004)]
    for i, z in enumerate(THREAD_Z_LIST):
        ring = TorusGeometry(THREAD_MAJOR_R, THREAD_TUBE_R, radial_segments=8, tubular_segments=40)
        dx, dy = offsets[i % len(offsets)]
        ring.translate(dx, dy, z)
        if g is None:
            g = ring
        else:
            g.merge(ring)
    return mesh_from_geometry(g, "neck_threads")


def _collar_ring_mesh():
    collar = (
        cq.Workplane("XY")
        .workplane(offset=-0.003)
        .circle(NECK_R + 0.0015)
        .circle(NECK_R - 0.0005)
        .extrude(0.006)
    )
    return mesh_from_cadquery(collar, "collar_ring")


def _stopper_solid():
    arm_height = ARM_DISC_Z

    # Left arm wire (vertical)
    left_arm = (
        cq.Workplane("XY")
        .center(ARM_SPREAD_X, 0)
        .circle(WIRE_R)
        .extrude(arm_height)
    )
    # Right arm wire
    right_arm = (
        cq.Workplane("XY")
        .center(-ARM_SPREAD_X, 0)
        .circle(WIRE_R)
        .extrude(arm_height)
    )
    # Left tab (horizontal connector to collar at pivot)
    # Tabs reach into collar ring but stay inside the neck surface to avoid
    # overlap with the bottle shell.
    tab_outer = NECK_R - 0.0003  # just inside neck surface
    tab_width = tab_outer - ARM_SPREAD_X
    left_tab = (
        cq.Workplane("XY")
        .center(ARM_SPREAD_X + tab_width / 2, 0)
        .rect(tab_width, WIRE_R * 2.5)
        .extrude(WIRE_R * 2.5)
    )
    right_tab = (
        cq.Workplane("XY")
        .center(-(ARM_SPREAD_X + tab_width / 2), 0)
        .rect(tab_width, WIRE_R * 2.5)
        .extrude(WIRE_R * 2.5)
    )
    # Lever bar connecting arms at top
    lever_w = ARM_SPREAD_X * 2 + WIRE_R * 2
    lever = (
        cq.Workplane("XY")
        .center(0, 0)
        .workplane(offset=arm_height - WIRE_R)
        .rect(lever_w, WIRE_R * 2.5)
        .extrude(WIRE_R * 2.5)
    )
    # Disc (seals the opening), slightly overlapping lever for connectivity
    disc = (
        cq.Workplane("XY")
        .workplane(offset=arm_height - 0.001)
        .circle(STOPPER_DISC_R)
        .extrude(STOPPER_DISC_H)
    )
    # Sealing gasket ring on disc underside
    seal = (
        cq.Workplane("XY")
        .workplane(offset=arm_height - 0.001)
        .circle(STOPPER_DISC_R * 0.85)
        .circle(STOPPER_DISC_R * 0.60)
        .extrude(0.001)
    )
    result = left_arm.union(right_arm).union(left_tab).union(right_tab)
    result = result.union(lever).union(disc).union(seal)
    return result


def _stopper_mesh():
    return mesh_from_cadquery(_stopper_solid(), "stopper_body")


def _cup_solid():
    cup = (
        cq.Workplane("XY")
        .circle(CUP_OUTER_R)
        .extrude(CUP_HEIGHT)
    )
    bore = (
        cq.Workplane("XY")
        .circle(CUP_INNER_R)
        .extrude(CUP_HEIGHT - CUP_TOP_WALL)
    )
    cup = cup.cut(bore)
    # Internal friction ring at bottom: tighter bore contacts the neck
    # for a secure seated fit (slight intentional interference).
    friction_inner = NECK_R - 0.0003  # 0.0122, slight interference with neck at 0.0125
    friction_ring = (
        cq.Workplane("XY")
        .circle(CUP_INNER_R)
        .circle(friction_inner)
        .extrude(0.004)
    )
    cup = cup.union(friction_ring)
    # Measurement line ridges on outside
    for z_line in [0.008, 0.016, 0.024]:
        ring = (
            cq.Workplane("XY")
            .workplane(offset=z_line)
            .circle(CUP_OUTER_R + 0.0005)
            .circle(CUP_OUTER_R - 0.0002)
            .extrude(0.0006)
        )
        cup = cup.union(ring)
    return cup


def _cup_mesh():
    return mesh_from_cadquery(_cup_solid(), "cup_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_top_bottle")

    # Materials
    clear = model.material("clear_pet", rgba=(0.78, 0.85, 0.88, 0.25))
    clear_neck = model.material("clear_neck", rgba=(0.72, 0.80, 0.84, 0.30))
    band_mat = model.material("band_frosted", rgba=(0.88, 0.90, 0.92, 0.45))
    collar_mat = model.material("collar_clear", rgba=(0.75, 0.82, 0.85, 0.40))
    ceramic = model.material("stopper_ceramic", rgba=(0.93, 0.91, 0.87, 1.0))
    cup_mat = model.material("cup_blue", rgba=(0.68, 0.80, 0.92, 0.30))

    # ---- Bottle body (root) ----
    body = model.part("bottle_body")
    body.visual(_bottle_mesh(), material=clear, name="bottle_shell")
    body.visual(_volume_bands_mesh(), material=band_mat, name="volume_bands")
    body.visual(_neck_threads_mesh(), material=clear_neck, name="neck_threads")
    body.visual(_collar_ring_mesh(), origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
                material=collar_mat, name="collar_ring")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, 0.176),
        mass=0.022,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
    )

    # ---- Swing-top stopper ----
    stopper = model.part("swing_stopper")
    stopper.visual(
        _stopper_mesh(),
        material=ceramic,
        name="stopper_body",
    )
    stopper.inertial = Inertial.from_geometry(
        Cylinder(STOPPER_DISC_R, ARM_DISC_Z + STOPPER_DISC_H),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, ARM_DISC_Z / 2)),
    )

    # ---- Measuring cup ----
    cup = model.part("measuring_cup")
    cup.visual(
        _cup_mesh(),
        material=cup_mat,
        name="cup_shell",
    )
    cup.inertial = Inertial.from_geometry(
        Cylinder(CUP_OUTER_R, CUP_HEIGHT),
        mass=0.005,
        origin=Origin(xyz=(0.0, 0.0, CUP_HEIGHT / 2)),
    )

    # ---- Articulations ----

    # Swing-top hinge: REVOLUTE about Y at the collar pivot height.
    # At q=0 (closed), disc seals the neck opening.
    # Positive q swings disc toward +X and upward (opening).
    model.articulation(
        "stopper_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stopper,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0,
            lower=0.0, upper=1.8,
        ),
    )

    # Measuring cup: PRISMATIC lift along +Z off the bottle top.
    model.articulation(
        "cup_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cup,
        origin=Origin(xyz=(0.0, 0.0, CUP_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=1.0,
            lower=0.0, upper=0.060,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle_body")
    stopper = object_model.get_part("swing_stopper")
    cup = object_model.get_part("measuring_cup")
    hinge = object_model.get_articulation("stopper_hinge")
    slide = object_model.get_articulation("cup_slide")

    # --- Bottle is clear/translucent ---
    clear_mat = next(m for m in object_model.materials if m.name == "clear_pet")
    a = clear_mat.rgba[3] if clear_mat.rgba is not None else 1.0
    ctx.check("bottle shell is transparent", a < 1.0, details=f"alpha={a}")

    # --- Volume bands exist ---
    vb = body.get_visual("volume_bands")
    ctx.check("volume bands exist on body", vb is not None, details="missing volume_bands visual")

    # --- Neck threads exist ---
    nt = body.get_visual("neck_threads")
    ctx.check("neck thread ridges exist", nt is not None, details="missing neck_threads visual")

    # --- Collar ring exists (hinge attachment) ---
    cr = body.get_visual("collar_ring")
    ctx.check("collar ring exists at pivot", cr is not None, details="missing collar_ring visual")

    # --- Stopper hinge is REVOLUTE with proper limits ---
    ctx.check(
        "stopper hinge is revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={hinge.articulation_type}",
    )
    hinge_limits = hinge.motion_limits
    ctx.check(
        "stopper hinge has bounded range",
        hinge_limits is not None and hinge_limits.lower is not None and hinge_limits.upper is not None
        and hinge_limits.upper > 0.5,
        details=f"limits={hinge_limits}",
    )

    # --- Cup slide is PRISMATIC ---
    ctx.check(
        "cup slide is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )

    # --- Allow intentional overlap: stopper tabs captured at hinge pivot ---
    ctx.allow_overlap(
        stopper, body,
        elem_a="stopper_body", elem_b="collar_ring",
        reason="Swing-top arm tabs pivot within the collar ring at the hinge attachment.",
    )
    ctx.allow_overlap(
        stopper, body,
        elem_a="stopper_body", elem_b="bottle_shell",
        reason="Swing-top arm tabs embed in the thin neck wall at the hinge pivot, captured by the collar ring.",
    )

    # --- Stopper mounted near the top of the bottle ---
    stopper_pos = ctx.part_world_position(stopper)
    ctx.check(
        "stopper mounted near bottle top",
        stopper_pos is not None and stopper_pos[2] > 0.14,
        details=f"stopper origin={stopper_pos}",
    )

    # --- Swing stopper opens: disc moves in X when hinge rotates ---
    disc_z_rest = None
    disc_x_rest = None
    # Get disc position at rest (closed)
    stopper_aabb_rest = ctx.part_world_aabb(stopper)
    if stopper_aabb_rest is not None:
        mn, mx = stopper_aabb_rest
        disc_x_rest = (mn[0] + mx[0]) / 2.0
        disc_z_rest = mx[2]  # top of stopper

    with ctx.pose({hinge: 1.2}):
        stopper_aabb_open = ctx.part_world_aabb(stopper)
        if stopper_aabb_open is not None:
            mn_o, mx_o = stopper_aabb_open
            disc_x_open = (mn_o[0] + mx_o[0]) / 2.0
            disc_z_open = mx_o[2]

    if disc_x_rest is not None:
        x_shift = abs(disc_x_open - disc_x_rest)
        ctx.check(
            "stopper swings open (disc moves laterally)",
            x_shift > 0.005,
            details=f"rest_x={disc_x_rest:.4f}, open_x={disc_x_open:.4f}, shift={x_shift:.4f}",
        )

    # --- Measuring cup slides off ---
    cup_z_rest = ctx.part_world_aabb(cup)[0][2]
    with ctx.pose({slide: 0.040}):
        cup_z_up = ctx.part_world_aabb(cup)[0][2]
    ctx.check(
        "measuring cup lifts off bottle",
        cup_z_up > cup_z_rest + 0.030,
        details=f"cup bottom rest={cup_z_rest:.4f}, lifted={cup_z_up:.4f}",
    )

    # --- Cup is mounted at the top ---
    cup_pos = ctx.part_world_position(cup)
    ctx.check(
        "measuring cup mounted at bottle top",
        cup_pos is not None and cup_pos[2] > 0.15,
        details=f"cup origin={cup_pos}",
    )

    # --- Cup fits over neck (allow overlap for bore clearance) ---
    ctx.allow_overlap(
        cup, body,
        elem_a="cup_shell", elem_b="bottle_shell",
        reason="Measuring cup bore slides over the bottle neck when seated.",
    )
    ctx.allow_overlap(
        cup, stopper,
        elem_a="cup_shell", elem_b="stopper_body",
        reason="Measuring cup bore surrounds the closed swing-top stopper.",
    )

    # --- Cup stays within neck region XY at rest ---
    ctx.expect_within(
        cup, body, axes="xy",
        margin=0.012,
        name="cup centered on bottle at rest",
    )

    # --- Proof: stopper contacts the body at the pivot (hinge capture) ---
    ctx.expect_contact(
        stopper, body,
        contact_tol=0.003,
        name="stopper contacts body at hinge pivot",
    )

    # --- Proof: cup contacts body at friction ring ---
    ctx.expect_contact(
        cup, body,
        elem_a="cup_shell", elem_b="bottle_shell",
        contact_tol=0.002,
        name="cup friction ring contacts neck",
    )

    # --- Proof: cup overlaps stopper in XY (cup surrounds closed stopper) ---
    ctx.expect_overlap(
        cup, stopper, axes="xy",
        min_overlap=0.005,
        name="cup bore surrounds closed stopper in XY",
    )

    # --- Tapered shoulder ---
    ctx.check(
        "tapered shoulder narrows toward top",
        NECK_R < BODY_R * 0.6,
        details=f"neck_r={NECK_R}, body_r={BODY_R}",
    )

    return ctx.report()


object_model = build_object_model()
