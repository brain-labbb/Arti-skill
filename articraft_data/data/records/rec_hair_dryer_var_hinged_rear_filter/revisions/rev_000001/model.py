from __future__ import annotations

# Pink compact hair dryer.
# Frame: barrel axis along +X (front/nozzle at +X, rear intake at -X),
# barrel centerline at z=0, handle hanging down (-Z).
# Articulations:
#   - concentrator nozzle: CONTINUOUS spin about the barrel axis (orient the slot)
#   - two slide switches on the handle: PRISMATIC fore/aft travel
#   - power cord + plug: FIXED (flexible cable, modeled as one drooping tube)

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
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

BARREL_FRONT_X = 0.175
NOZZLE_MOUNT_X = 0.163  # nozzle back sleeve overlaps the barrel front lip


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


def _perforated_disc() -> cq.Workplane:
    """Circular lint-screen disc with concentric perforated-hole pattern.
    Built in the YZ plane at x=0. The disc center is at local (0, 0, -R)
    so the top edge touches the hinge origin at local z = 0.
    The disc is centered on x=0 so its front face contacts the barrel rear rim.
    """
    R = 0.035
    thickness = 0.003
    disc = (
        cq.Workplane("YZ")
        .center(0, -R)
        .circle(R)
        .extrude(thickness)
    )
    # Shift so the disc is centered on x=0 (from -t/2 to +t/2)
    disc = disc.translate((-thickness / 2, 0, 0))
    hole_pts = []
    for ring_r, count in [(0.010, 5), (0.018, 8), (0.026, 12)]:
        for i in range(count):
            angle = 2 * math.pi * i / count
            hole_pts.append((ring_r * math.cos(angle), ring_r * math.sin(angle)))
    disc = disc.faces(">X").workplane().pushPoints(hole_pts).hole(0.004)
    return disc


def _hinge_knuckle(y_offset: float):
    """Small hinge-knuckle cylinder oriented along Y at the barrel rear top.
    Positioned to straddle the barrel rear rim surface for physical contact.
    """
    knuckle = CylinderGeometry(0.003, 0.008, radial_segments=16)
    knuckle.rotate_x(math.pi / 2.0)
    # Center the knuckle on the barrel rear face (x=0) at the barrel top (z≈0.037)
    knuckle.translate(0.0, y_offset, 0.037)
    return knuckle


def _nozzle_mesh():
    # Concentrator: round back (slips over the barrel lip) lofted to a wide thin
    # slot, hollowed so air passes through.
    outer = _loft(
        [
            ("circle", 0.0, 0.033),
            ("rect", 0.028, 0.070, 0.030),
            ("rect", 0.052, 0.082, 0.013),
        ]
    )
    inner = _loft(
        [
            ("circle", -0.006, 0.030),
            ("rect", 0.028, 0.064, 0.024),
            ("rect", 0.058, 0.078, 0.008),
        ]
    )
    noz = outer.cut(inner)
    return mesh_from_cadquery(noz, "nozzle")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hair_dryer")

    shell_pink = model.material("shell_pink", rgba=(0.96, 0.71, 0.78, 1.0))
    dark = model.material("dark_gray", rgba=(0.24, 0.24, 0.26, 1.0))
    switch_gray = model.material("switch_gray", rgba=(0.32, 0.32, 0.34, 1.0))

    # ---- body (root): barrel + handle + rear filter cap + switch housing ----
    body = model.part("body")

    body_shell = _barrel_solid().union(_handle_solid())
    body.visual(mesh_from_cadquery(body_shell, "body_shell"), material=shell_pink, name="body_shell")

    # Rear hinge mount: two knuckle barrels for the flip-open lint screen.
    for i in range(2):
        dy = -0.010 + i * 0.020
        body.visual(
            mesh_from_geometry(_hinge_knuckle(dy), f"hinge_knuckle_{i}"),
            material=dark,
            name=f"hinge_knuckle_{i}",
        )

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

    # ---- lint screen: flip-open perforated disc hinged at the barrel rear top ----
    lint_screen = model.part("lint_screen")
    lint_screen.visual(
        mesh_from_cadquery(_perforated_disc(), "screen_disc"),
        material=dark,
        name="screen_disc",
    )
    lint_screen.visual(
        Box((0.005, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=dark,
        name="hinge_tab",
    )
    model.articulation(
        "body_to_lint_screen",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lint_screen,
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.5),
    )

    # ---- concentrator nozzle: spins about the barrel axis ----
    nozzle = model.part("nozzle")
    nozzle.visual(_nozzle_mesh(), material=dark, name="nozzle_shell")
    nozzle.inertial = Inertial.from_geometry(
        Box((0.05, 0.082, 0.07)), mass=0.04, origin=Origin(xyz=(0.025, 0.0, 0.0))
    )
    model.articulation(
        "barrel_to_nozzle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=nozzle,
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
    nozzle = object_model.get_part("nozzle")
    lint_screen = object_model.get_part("lint_screen")
    power = object_model.get_part("power_switch")
    heat = object_model.get_part("heat_switch")
    cord = object_model.get_part("power_cord")
    spin = object_model.get_articulation("barrel_to_nozzle")
    hinge = object_model.get_articulation("body_to_lint_screen")
    power_joint = object_model.get_articulation("body_to_power_switch")

    # Lint screen: hinged at the barrel rear top, perforated grille covers the intake.
    ctx.allow_overlap(
        body,
        lint_screen,
        elem_a="hinge_knuckle_0",
        elem_b="hinge_tab",
        reason="Hinge knuckle and tab intentionally interleave at the pivot.",
    )
    ctx.allow_overlap(
        body,
        lint_screen,
        elem_a="hinge_knuckle_1",
        elem_b="hinge_tab",
        reason="Hinge knuckle and tab intentionally interleave at the pivot.",
    )
    ctx.expect_contact(
        lint_screen, body, elem_a="screen_disc", elem_b="body_shell",
        name="lint screen disc seated at barrel rear",
    )
    screen_pos = ctx.part_world_position(lint_screen)
    ctx.check(
        "lint screen mounted at barrel rear",
        screen_pos is not None and screen_pos[0] < 0.01,
        details=f"screen origin={screen_pos}",
    )
    closed_aabb = ctx.part_world_aabb(lint_screen)
    with ctx.pose({hinge: 1.2}):
        open_aabb = ctx.part_world_aabb(lint_screen)
    ctx.check(
        "lint screen swings open away from barrel rear",
        closed_aabb is not None and open_aabb is not None
        and open_aabb[0][0] < closed_aabb[0][0] - 0.015,
        details=f"closed_min_x={closed_aabb[0][0]:.4f}, open_min_x={open_aabb[0][0]:.4f}",
    )

    # Nozzle is at the front and stays seated over the barrel lip (slight capture).
    ctx.allow_overlap(
        nozzle,
        body,
        elem_a="nozzle_shell",
        elem_b="body_shell",
        reason="Concentrator back rim is intentionally slipped over the barrel front lip.",
    )
    ctx.expect_overlap(
        nozzle, body, axes="x", min_overlap=0.006, name="nozzle seated over barrel front"
    )
    noz_pos = ctx.part_world_position(nozzle)
    ctx.check(
        "nozzle mounted at the barrel front",
        noz_pos is not None and noz_pos[0] > 0.15,
        details=f"nozzle origin={noz_pos}",
    )

    # Spinning the nozzle reorients the flat slot: wide axis flips from Y to Z.
    ext0 = _ext(ctx.part_world_aabb(nozzle))
    ctx.check(
        "slot is horizontal at rest",
        ext0[1] > ext0[2] + 0.005,
        details=f"rest extents={ext0}",
    )
    with ctx.pose({spin: math.pi / 2.0}):
        ext90 = _ext(ctx.part_world_aabb(nozzle))
    ctx.check(
        "nozzle spin rotates the slot toward vertical",
        ext90[2] > ext90[1] + 0.005,
        details=f"quarter-turn extents={ext90}",
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
