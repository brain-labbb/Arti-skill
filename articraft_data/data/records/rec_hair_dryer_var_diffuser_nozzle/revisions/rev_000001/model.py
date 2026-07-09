from __future__ import annotations

# Pink compact hair dryer with round bowl diffuser attachment.
# Frame: barrel axis along +X (front/diffuser at +X, rear intake at -X),
# barrel centerline at z=0, handle hanging down (-Z).
# Articulations:
#   - bowl diffuser: CONTINUOUS spin about the barrel axis
#   - two slide switches on the handle: PRISMATIC fore/aft travel
#   - power cord + plug: FIXED (flexible cable, modeled as one drooping tube)

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CapsuleGeometry,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

BARREL_FRONT_X = 0.175
NOZZLE_MOUNT_X = 0.163  # diffuser back sleeve overlaps the barrel front lip

# Diffuser geometry constants (relative to diffuser part frame at NOZZLE_MOUNT_X)
BOWL_R = 0.056
FACE_X = 0.027
FACE_THICKNESS = 0.003
FINGER_RING_R = 0.036
FINGER_R = 0.004
FINGER_LEN = 0.014
N_FINGERS = 10


def _loft(sections) -> cq.Workplane:
    # sections: list of ("circle", x, r) or ("rect", x, w, h) along +X (YZ planes).
    wp = cq.Workplane("YZ")
    prev = 0.0
    for i, s in enumerate(sections):
        x = s[1]
        off = x if i == 0 else x - prev
        if i > 0 or abs(off) > 1e-12:
            wp = wp.workplane(offset=off)
        if s[0] == "circle":
            wp = wp.circle(s[2])
        else:
            wp = wp.rect(s[2], s[3])
        prev = x
    return wp.loft(ruled=False)


def _barrel_solid() -> cq.Workplane:
    # Hollow housing: outer loft minus a slightly smaller inner loft that pokes
    # past both ends, so the barrel reads as a real open-ended shell.
    outer = _loft(
        [
            ("circle", 0.0, 0.037),
            ("circle", 0.030, 0.042),
            ("circle", 0.100, 0.040),
            ("circle", 0.150, 0.033),
            ("circle", 0.175, 0.030),
        ]
    )
    inner = _loft(
        [
            ("circle", -0.006, 0.034),
            ("circle", 0.030, 0.039),
            ("circle", 0.100, 0.037),
            ("circle", 0.150, 0.030),
            ("circle", 0.181, 0.027),
        ]
    )
    return outer.cut(inner)


def _handle_solid() -> cq.Workplane:
    # Tapered, filleted handle dropping from under the barrel.
    handle = (
        cq.Workplane("XY")
        .center(0.063, 0.0)
        .rect(0.050, 0.030)
        .workplane(offset=-0.055)
        .rect(0.046, 0.030)
        .workplane(offset=-0.060)
        .rect(0.040, 0.032)
        .loft(ruled=False)
    )
    return handle


def _diffuser_bowl_solid() -> cq.Workplane:
    """Bowl shell with perforated face plate (CadQuery solid)."""
    # Outer shell: sleeve → flared body → rim
    outer = _loft(
        [
            ("circle", -0.010, 0.034),
            ("circle", 0.004, 0.035),
            ("circle", 0.018, 0.050),
            ("circle", 0.030, BOWL_R),
        ]
    )
    # Inner void: hollows the interior, open at both ends
    inner = _loft(
        [
            ("circle", -0.016, 0.031),
            ("circle", 0.004, 0.031),
            ("circle", 0.018, 0.044),
            ("circle", 0.035, BOWL_R - 0.006),
        ]
    )
    shell = outer.cut(inner)

    # Face plate: thin disk closing the front of the bowl
    face = (
        cq.Workplane("YZ")
        .workplane(offset=FACE_X)
        .circle(BOWL_R - 0.003)
        .extrude(FACE_THICKNESS)
    )

    # Perforation holes through the face plate
    hole_points_inner = [
        (0.020 * math.cos(2 * math.pi * i / 6), 0.020 * math.sin(2 * math.pi * i / 6))
        for i in range(6)
    ]
    hole_points_outer = [
        (0.042 * math.cos(2 * math.pi * i / 8 + 0.2), 0.042 * math.sin(2 * math.pi * i / 8 + 0.2))
        for i in range(8)
    ]
    all_holes = hole_points_inner + hole_points_outer

    hole_cutters = (
        cq.Workplane("YZ")
        .workplane(offset=FACE_X - 0.005)
        .pushPoints(all_holes)
        .circle(0.004)
        .extrude(FACE_THICKNESS + 0.010)
    )
    center_hole = (
        cq.Workplane("YZ")
        .workplane(offset=FACE_X - 0.005)
        .circle(0.007)
        .extrude(FACE_THICKNESS + 0.010)
    )
    face = face.cut(hole_cutters).cut(center_hole)

    return shell.union(face)


def _finger_mesh():
    """Single finger capsule extending along +X from origin."""
    finger = CapsuleGeometry(FINGER_R, FINGER_LEN, radial_segments=12, height_segments=4)
    # CapsuleGeometry extends along Z centered; rotate to extend along X
    finger.rotate_y(-math.pi / 2.0)
    # Shift so base embeds slightly into the face plate
    half_span = FINGER_LEN / 2.0 + FINGER_R
    finger.translate(FACE_X + FACE_THICKNESS + half_span - 0.003, 0.0, 0.0)
    return finger


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hair_dryer")

    shell_pink = model.material("shell_pink", rgba=(0.96, 0.71, 0.78, 1.0))
    dark = model.material("dark_gray", rgba=(0.24, 0.24, 0.26, 1.0))
    switch_gray = model.material("switch_gray", rgba=(0.32, 0.32, 0.34, 1.0))

    # ---- body (root): barrel + handle + rear filter cap + switch housing ----
    body = model.part("body")

    body_shell = _barrel_solid().union(_handle_solid())
    body.visual(mesh_from_cadquery(body_shell, "body_shell"), material=shell_pink, name="body_shell")

    # Rear intake filter cap with concentric grille ribs.
    cap = CylinderGeometry(0.037, 0.012, radial_segments=48).rotate_y(math.pi / 2.0)
    cap.translate(-0.004, 0.0, 0.0)
    for rr in (0.014, 0.022, 0.030):
        ring = TorusGeometry(rr, 0.0016, radial_segments=10, tubular_segments=40).rotate_y(math.pi / 2.0)
        ring.translate(-0.011, 0.0, 0.0)
        cap.merge(ring)
    body.visual(mesh_from_geometry(cap, "rear_filter"), material=dark, name="rear_filter")

    # Switch housing plate on the +Y broad face of the handle.
    body.visual(
        Box((0.032, 0.005, 0.062)),
        origin=Origin(xyz=(0.060, 0.0145, -0.032)),
        material=dark,
        name="switch_housing",
    )

    body.inertial = Inertial.from_geometry(
        Box((0.20, 0.085, 0.085)), mass=0.45, origin=Origin(xyz=(0.085, 0.0, 0.0))
    )

    # ---- bowl diffuser: spins about the barrel axis ----
    diffuser = model.part("diffuser")
    diffuser.visual(
        mesh_from_cadquery(_diffuser_bowl_solid(), "diffuser_bowl"),
        material=dark,
        name="bowl_shell",
    )
    # Fingers: repeated capsule prongs on the bowl face
    for i in range(N_FINGERS):
        angle = 2.0 * math.pi * i / N_FINGERS
        fy = FINGER_RING_R * math.cos(angle)
        fz = FINGER_RING_R * math.sin(angle)
        finger = _finger_mesh()
        finger.translate(0.0, fy, fz)
        diffuser.visual(
            mesh_from_geometry(finger, f"finger_{i}"),
            material=dark,
            name=f"finger_{i}",
        )
    diffuser.inertial = Inertial.from_geometry(
        Box((0.06, 0.112, 0.112)), mass=0.05, origin=Origin(xyz=(0.025, 0.0, 0.0))
    )
    model.articulation(
        "barrel_to_diffuser",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=diffuser,
        origin=Origin(xyz=(NOZZLE_MOUNT_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=6.0),
    )

    # ---- two slide switches (power + heat) sliding fore/aft on the handle ----
    for name, sz in (("power_switch", -0.018), ("heat_switch", -0.046)):
        sw = model.part(name)
        sw.visual(
            Box((0.013, 0.007, 0.011)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=switch_gray,
            name=f"{name}_nub",
        )
        sw.inertial = Inertial.from_geometry(Box((0.013, 0.007, 0.011)), mass=0.003)
        model.articulation(
            f"body_to_{name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=sw,
            origin=Origin(xyz=(0.060, 0.0205, sz)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.1, lower=-0.007, upper=0.007),
        )

    # ---- power cord + plug: one drooping flexible cable, fixed to the handle ----
    cord_pts = [
        (0.045, 0.0, -0.116),
        (0.052, 0.0, -0.150),
        (0.030, 0.0, -0.190),
        (-0.020, 0.0, -0.208),
        (-0.060, 0.0, -0.208),
        (-0.090, 0.0, -0.208),
    ]
    cord = tube_from_spline_points(cord_pts, radius=0.0042, samples_per_segment=16, radial_segments=14)
    # Strain-relief sleeve where the cord exits the handle base (overlaps the cord top).
    relief = CylinderGeometry(0.0085, 0.024).translate(0.045, 0.0, -0.118)
    cord.merge(relief)
    # Plug body (cord runs into it) + two pins at the cord end.
    plug = BoxGeometry((0.040, 0.028, 0.022)).translate(-0.090, 0.0, -0.208)
    cord.merge(plug)
    for py in (-0.008, 0.008):
        pin = CylinderGeometry(0.0035, 0.020).rotate_y(math.pi / 2.0)
        pin.translate(-0.119, py, -0.208)
        cord.merge(pin)

    power_cord = model.part("power_cord")
    power_cord.visual(mesh_from_geometry(cord, "power_cord"), material=dark, name="cord_shell")
    power_cord.inertial = Inertial.from_geometry(Box((0.18, 0.03, 0.12)), mass=0.06)
    model.articulation(
        "body_to_cord",
        ArticulationType.FIXED,
        parent=body,
        child=power_cord,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    diffuser = object_model.get_part("diffuser")
    power = object_model.get_part("power_switch")
    heat = object_model.get_part("heat_switch")
    cord = object_model.get_part("power_cord")
    spin = object_model.get_articulation("barrel_to_diffuser")
    power_joint = object_model.get_articulation("body_to_power_switch")

    # Diffuser is at the front and stays seated over the barrel lip (slight capture).
    ctx.allow_overlap(
        diffuser,
        body,
        elem_a="bowl_shell",
        elem_b="body_shell",
        reason="Diffuser back rim is intentionally slipped over the barrel front lip.",
    )
    ctx.expect_overlap(
        diffuser, body, axes="x", min_overlap=0.006, name="diffuser seated over barrel front"
    )
    dif_pos = ctx.part_world_position(diffuser)
    ctx.check(
        "diffuser mounted at the barrel front",
        dif_pos is not None and dif_pos[0] > 0.15,
        details=f"diffuser origin={dif_pos}",
    )

    # The bowl is round, so spinning does not change the overall AABB; but the
    # individual finger positions must rotate with the diffuser.
    f0 = object_model.get_part("diffuser").get_visual("finger_0")
    rest_center = ctx.part_element_world_aabb(diffuser, elem=f0)
    with ctx.pose({spin: math.pi}):
        spun_center = ctx.part_element_world_aabb(diffuser, elem=f0)
    ctx.check(
        "diffuser spin rotates finger_0",
        rest_center is not None and spun_center is not None
        and (abs(rest_center[0][1] - spun_center[0][1]) > 0.005
             or abs(rest_center[0][2] - spun_center[0][2]) > 0.005),
        details=f"rest_yz={rest_center[0][1:]}, spun_yz={spun_center[0][1:]}",
    )

    # Diffuser face plate has perforations (the face is wider than tall → bowl shape).
    ext = _ext(ctx.part_world_aabb(diffuser))
    ctx.check(
        "diffuser bowl is round",
        abs(ext[1] - ext[2]) < 0.010,
        details=f"bowl extents={ext}",
    )

    # Both switches sit on the housing and slide along the barrel direction.
    ctx.expect_contact(power, body, name="power switch rests on housing")
    ctx.expect_contact(heat, body, name="heat switch rests on housing")
    rest_x = ctx.part_world_position(power)[0]
    with ctx.pose({power_joint: 0.007}):
        slid_x = ctx.part_world_position(power)[0]
    ctx.check(
        "power switch slides forward",
        slid_x > rest_x + 0.004,
        details=f"rest_x={rest_x}, slid_x={slid_x}",
    )

    # Cord/plug is physically attached at the handle base (strain relief plugs in).
    ctx.allow_overlap(
        body,
        cord,
        elem_a="body_shell",
        elem_b="cord_shell",
        reason="Strain-relief sleeve and cord top intentionally enter the handle base.",
    )
    ctx.expect_contact(cord, body, name="cord attached to handle")

    return ctx.report()


object_model = build_object_model()
