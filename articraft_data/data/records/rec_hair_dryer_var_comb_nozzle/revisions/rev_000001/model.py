from __future__ import annotations

# Pink compact hair dryer with comb pick nozzle.
# Frame: barrel axis along +X (front/nozzle at +X, rear intake at -X),
# barrel centerline at z=0, handle hanging down (-Z).
# Articulations:
#   - comb pick nozzle: CONTINUOUS spin about the barrel axis (orient the teeth)
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


def _comb_nozzle_body():
    # Comb nozzle: a short round collar seats over the barrel lip, then flattens
    # into a narrow slit duct. Teeth are separate straight pegs on the lower lip.
    outer = _loft(
        [
            ("circle", 0.0, 0.032),
            ("circle", 0.010, 0.030),
            ("rect", 0.025, 0.052, 0.024),
            ("rect", 0.050, 0.046, 0.016),
        ]
    )
    inner = _loft(
        [
            ("circle", -0.006, 0.028),
            ("circle", 0.010, 0.026),
            ("rect", 0.025, 0.044, 0.017),
            ("rect", 0.056, 0.038, 0.010),
        ]
    )
    noz = outer.cut(inner)
    return mesh_from_cadquery(noz, "nozzle_body")


def _comb_lip_rail():
    # Low rectangular rail directly under the slit; teeth start at this rail.
    rail = (
        cq.Workplane("XY")
        .box(0.018, 0.050, 0.006, centered=(True, True, True))
        .translate((0.043, 0.0, -0.014))
    )
    return mesh_from_cadquery(rail, "comb_lip_rail")


def _make_tooth():
    """Single comb tooth: short straight cylindrical peg pointing downward (-Z)."""
    tooth = CylinderGeometry(0.0018, 0.020, radial_segments=10)
    tooth.translate(0.0, 0.0, -0.010)
    return tooth


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

    # ---- comb pick nozzle: spins about the barrel axis ----
    nozzle = model.part("nozzle")
    nozzle.visual(_comb_nozzle_body(), material=dark, name="nozzle_body")
    nozzle.visual(_comb_lip_rail(), material=dark, name="comb_lip_rail")

    # Row of evenly spaced comb teeth on the lower front lip of the duct.
    num_teeth = 11
    tooth_y_start = -0.021
    tooth_y_end = 0.021
    tooth_geom = _make_tooth()
    for i in range(num_teeth):
        t = i / (num_teeth - 1)
        ty = tooth_y_start + t * (tooth_y_end - tooth_y_start)
        nozzle.visual(
            mesh_from_geometry(tooth_geom.copy(), f"tooth_{i}"),
            origin=Origin(xyz=(0.043, ty, -0.017)),
            material=dark,
            name=f"tooth_{i}",
        )

    nozzle.inertial = Inertial.from_geometry(
        Box((0.07, 0.05, 0.06)), mass=0.04, origin=Origin(xyz=(0.030, 0.0, -0.01))
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
    power = object_model.get_part("power_switch")
    heat = object_model.get_part("heat_switch")
    cord = object_model.get_part("power_cord")
    spin = object_model.get_articulation("barrel_to_nozzle")
    power_joint = object_model.get_articulation("body_to_power_switch")

    # Comb nozzle collar is at the front and slips over the barrel lip (slight capture).
    ctx.allow_overlap(
        nozzle,
        body,
        elem_a="nozzle_body",
        elem_b="body_shell",
        reason="Comb nozzle collar back rim is intentionally slipped over the barrel front lip.",
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

    # Comb teeth are short straight pegs attached to the lower lip, not dangling cones.
    tooth_0_aabb = ctx.part_element_world_aabb(nozzle, elem="tooth_0")
    tooth_5_aabb = ctx.part_element_world_aabb(nozzle, elem="tooth_5")
    rail_aabb = ctx.part_element_world_aabb(nozzle, elem="comb_lip_rail")
    tooth_0_ext = _ext(tooth_0_aabb) if tooth_0_aabb else None
    ctx.check(
        "comb teeth attach to lower nozzle lip",
        tooth_0_aabb is not None and rail_aabb is not None
        and rail_aabb[0][2] - 0.001 <= tooth_0_aabb[1][2] <= rail_aabb[1][2] + 0.001
        and tooth_0_aabb[0][2] < rail_aabb[0][2] - 0.012,
        details=f"tooth_0 z={tooth_0_aabb if tooth_0_aabb else None}, "
        f"rail z={rail_aabb if rail_aabb else None}",
    )
    ctx.check(
        "comb teeth are straight and short",
        tooth_0_ext is not None
        and tooth_0_ext[2] < 0.026
        and tooth_0_ext[0] < 0.006
        and tooth_0_ext[1] < 0.006,
        details=f"tooth_0 extents={tooth_0_ext}",
    )

    # Teeth are evenly spaced across the lower outlet edge.
    first = ctx.part_element_world_aabb(nozzle, elem="tooth_0")
    last = ctx.part_element_world_aabb(nozzle, elem="tooth_10")
    mid = tooth_5_aabb
    if first and last and mid:
        span = last[0][1] - first[0][1]
        mid_offset = mid[0][1] - first[0][1]
        ctx.check(
            "teeth span evenly across nozzle lip",
            span > 0.035 and 0.4 < mid_offset / span < 0.6,
            details=f"span={span:.4f}, mid_offset={mid_offset:.4f}",
        )

    # Spinning the nozzle reorients the teeth: they hang downward at rest,
    # point sideways after a quarter turn.
    ext0 = _ext(ctx.part_world_aabb(nozzle))
    ctx.check(
        "teeth hang downward at rest",
        ext0[2] > ext0[1] + 0.004,
        details=f"rest extents={ext0}",
    )
    with ctx.pose({spin: math.pi / 2.0}):
        ext90 = _ext(ctx.part_world_aabb(nozzle))
    ctx.check(
        "nozzle spin rotates teeth toward horizontal",
        ext90[1] > ext90[2] + 0.004,
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
