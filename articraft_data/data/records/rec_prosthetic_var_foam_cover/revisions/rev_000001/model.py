from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


BLUE = Material("glossy_clinical_blue", rgba=(0.04, 0.30, 0.80, 1.0))
DARK = Material("matte_black_metal", rgba=(0.026, 0.030, 0.038, 1.0))
RUBBER = Material("dark_rubber_sole", rgba=(0.016, 0.016, 0.018, 1.0))
METAL = Material("brushed_silver_pylon", rgba=(0.62, 0.64, 0.66, 1.0))
FASTENER = Material("small_silver_fasteners", rgba=(0.80, 0.80, 0.78, 1.0))
RED = Material("bright_red_spring", rgba=(0.84, 0.05, 0.04, 1.0))
COSMESIS_FOAM = Material("matte_cosmesis_foam", rgba=(0.85, 0.74, 0.63, 1.0))


def _socket_shell() -> cq.Workplane:
    """Tapered open below-knee socket shell, authored upright in meters."""
    outer = (
        cq.Workplane("XY")
        .circle(0.041)
        .workplane(offset=0.135)
        .circle(0.060)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=0.024)
        .circle(0.029)
        .workplane(offset=0.123)
        .circle(0.052)
        .loft(combine=True)
    )
    shell = outer.cut(inner)
    try:
        shell = shell.edges().fillet(0.003)
    except Exception:
        pass
    return shell


def _foam_cuff_ring() -> cq.Workplane:
    """Chunky padded black donut/cuff around the socket rim."""
    ring = cq.Workplane("XY").circle(0.073).extrude(0.036)
    bore = cq.Workplane("XY").circle(0.050).extrude(0.040)
    try:
        ring = ring.cut(bore).edges().fillet(0.006)
    except Exception:
        ring = ring.cut(bore)
    return ring


def _cosmesis_cover_shape() -> cq.Workplane:
    """Lifelike cosmetic foam cosmesis — a hollow leg-shaped shell over the pylon.

    Four elliptical cross-sections taper from a wider calf region to a narrower
    ankle.  The inner cavity extends through both end caps so the cover reads
    as an open sleeve enclosing the telescoping tubes.
    """
    # Outer shell: lofted ellipses at key heights (authored in-place in meters)
    outer = (
        cq.Workplane("XY")
        .workplane(offset=0.185)
        .ellipse(0.026, 0.030)    # ankle trim-line: 52 × 60 mm
        .workplane(offset=0.075)   # z ≈ 0.260
        .ellipse(0.038, 0.044)    # lower calf: 76 × 88 mm
        .workplane(offset=0.075)   # z ≈ 0.335
        .ellipse(0.040, 0.048)    # upper calf: 80 × 96 mm
        .workplane(offset=0.040)   # z ≈ 0.375
        .ellipse(0.036, 0.042)    # proximal trim: 72 × 84 mm
        .loft(combine=True)
    )
    # Inner cavity: slightly smaller, extends through both end caps
    w = 0.005  # 5 mm foam wall
    inner = (
        cq.Workplane("XY")
        .workplane(offset=0.180)   # 5 mm below outer bottom
        .ellipse(0.026 - w, 0.030 - w)
        .workplane(offset=0.080)   # z ≈ 0.260
        .ellipse(0.038 - w, 0.044 - w)
        .workplane(offset=0.075)   # z ≈ 0.335
        .ellipse(0.040 - w, 0.048 - w)
        .workplane(offset=0.045)   # z ≈ 0.380, 5 mm above outer top
        .ellipse(0.036 - w, 0.042 - w)
        .loft(combine=True)
    )
    shell = outer.cut(inner)
    try:
        shell = shell.edges().fillet(0.002)
    except Exception:
        pass
    return shell


def _blue_cuff_band() -> cq.Workplane:
    """Thin glossy blue accent strap recessed into the black foam cuff."""
    band = cq.Workplane("XY").circle(0.074).extrude(0.010)
    bore = cq.Workplane("XY").circle(0.066).extrude(0.012)
    try:
        band = band.cut(bore).edges().fillet(0.0015)
    except Exception:
        band = band.cut(bore)
    return band


def _clamp_collar() -> cq.Workplane:
    """Split-looking height-adjustment collar with side tightening ears."""
    collar = cq.Workplane("XY").circle(0.020).extrude(0.024)
    bore = cq.Workplane("XY").circle(0.0115).extrude(0.028)
    ear = _rounded_box((0.018, 0.030, 0.018), 0.003).translate((0.0, 0.024, 0.012))
    slot = cq.Workplane("XY").box(0.006, 0.020, 0.030).translate((0.0, 0.014, 0.012))
    try:
        return collar.cut(bore).union(ear).cut(slot).edges().fillet(0.0015)
    except Exception:
        return collar.cut(bore).union(ear).cut(slot)


def _aabb_center(aabb) -> tuple[float, float, float]:
    """Center of a world AABB given as ((minx, miny, minz), (maxx, maxy, maxz))."""
    lo, hi = aabb
    return ((lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5, (lo[2] + hi[2]) * 0.5)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    shape = cq.Workplane("XY").box(*size)
    try:
        shape = shape.edges("|Z").fillet(radius)
    except Exception:
        pass
    return shape


def _soft_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """Box with every edge rounded for a smooth, less blocky moulded look.

    Falls back to vertical-edge-only rounding (or a plain box) if the all-edge
    fillet is too aggressive for the given proportions.
    """
    # An all-edge fillet must stay below half the smallest box dimension or OCC
    # yields a null shape, so clamp the radius (with a little margin) first.
    safe = min(radius, 0.45 * min(size))
    for r in (safe, safe * 0.6, safe * 0.35):
        if r <= 1.0e-4:
            break
        try:
            shape = cq.Workplane("XY").box(*size).edges().fillet(r)
            shape.val().Volume()  # force evaluation; raises on a null/invalid shape
            return shape
        except Exception:
            continue
    try:
        return cq.Workplane("XY").box(*size).edges("|Z").fillet(min(radius, 0.45 * min(size[0], size[1])))
    except Exception:
        return cq.Workplane("XY").box(*size)


def _tube_mesh(points: list[tuple[float, float, float]], radius: float, name: str, *, samples: int = 12):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=samples,
            radial_segments=18,
            cap_ends=True,
        ),
        name,
    )


def _grab_handle_mesh():
    """Black inverted-U bail handle, authored in its own pivot frame.

    The pivot axis runs along Y through the two feet (the handle-local origin),
    so the loop swings fore/aft when the ``handle_pivot`` joint rotates.
    World feet sit at (0.060, +/-0.034, 0.424); here they are at the origin.
    """
    points = [
        (0.000, -0.034, 0.000),
        (0.014, -0.036, 0.036),
        (0.014, 0.000, 0.062),
        (0.014, 0.036, 0.036),
        (0.000, 0.034, 0.000),
    ]
    return _tube_mesh(points, 0.006, "front_grab_handle", samples=18)


def _spring_coil_mesh():
    """Helical red spring riding on top of the foot between toe pod and midfoot.

    Authored in the FOOT link frame so the spring belongs to, and moves with,
    the carbon foot rather than sitting behind the heel/ankle.
    """
    p0 = (0.130, 0.0, -0.074)  # lower/front eyelet at the toe pod
    p1 = (0.050, 0.0, -0.036)  # upper/rear eyelet on the midfoot bracket
    turns = 4.5
    coil_radius = 0.0108
    points: list[tuple[float, float, float]] = []
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dz * dz)
    ux = dx / length
    uz = dz / length
    # e1 is side-to-side; e2 lies in the XZ plane, perpendicular to the spring axis.
    e1 = (0.0, 1.0, 0.0)
    e2 = (-uz, 0.0, ux)
    steps = 112
    for i in range(steps + 1):
        t = i / steps
        theta = 2.0 * math.pi * turns * t
        cx = p0[0] + dx * t
        cy = p0[1] + dy * t
        cz = p0[2] + dz * t
        points.append(
            (
                cx + coil_radius * math.sin(theta) * e2[0],
                cy + coil_radius * math.cos(theta) * e1[1],
                cz + coil_radius * math.sin(theta) * e2[2],
            )
        )
    return _tube_mesh(points, 0.0026, "red_coil_spring", samples=3)


def _sole_shape() -> cq.Workplane:
    """Sporty smoothly-rockered rubber sole, rounded at heel and toe with a
    slight toe-spring lift, in the foot local frame."""
    sole = _soft_box((0.248, 0.090, 0.027), 0.016).translate((0.045, 0.0, -0.112))
    toe_rocker = (
        _soft_box((0.058, 0.086, 0.022), 0.015)
        .rotate((0, 0, 0), (0, 1, 0), 11.0)
        .translate((0.158, 0.0, -0.103))
    )
    heel_pad = _soft_box((0.060, 0.086, 0.022), 0.015).translate((-0.060, 0.0, -0.103))
    return sole.union(toe_rocker).union(heel_pad)


def _foot_upper_shape() -> cq.Workplane:
    """Open blue foot frame with toe pod, side rails, and ankle pedestal.

    The middle of the foot is intentionally open so the red spring bridges from
    the toe pod to the midfoot support without sinking into a solid upper shell.
    """
    base = _soft_box((0.206, 0.072, 0.018), 0.012).translate((0.042, 0.0, -0.092))
    side_rail_l = _soft_box((0.156, 0.018, 0.024), 0.008).translate((0.040, 0.032, -0.076))
    side_rail_r = _soft_box((0.156, 0.018, 0.024), 0.008).translate((0.040, -0.032, -0.076))
    # Rounded, slightly lifted toe box (toe-spring read).
    toe_cap = (
        _soft_box((0.078, 0.066, 0.026), 0.015)
        .rotate((0, 0, 0), (0, 1, 0), 7.0)
        .translate((0.144, 0.0, -0.080))
    )
    midfoot_bridge = _soft_box((0.054, 0.060, 0.032), 0.012).translate((0.006, 0.0, -0.052))
    # Central pedestal carrying the ankle hub.
    ankle_pedestal = _rounded_box((0.054, 0.052, 0.082), 0.012).translate((-0.018, 0.0, -0.041))
    # Rounded raised heel counter at the back.
    heel_counter = _soft_box((0.066, 0.062, 0.046), 0.018).translate((-0.046, 0.0, -0.066))
    return (
        base.union(side_rail_l)
        .union(side_rail_r)
        .union(toe_cap)
        .union(midfoot_bridge)
        .union(ankle_pedestal)
        .union(heel_counter)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_below_knee_prosthetic_leg")

    leg = model.part("leg")
    foot = model.part("foot")
    handle = model.part("handle")

    # Upper socket: glossy blue cup capped by the prominent padded black cuff.
    leg.visual(
        mesh_from_cadquery(_socket_shell(), "blue_socket_shell", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, 0.407)),
        material=BLUE,
        name="socket_shell",
    )
    leg.visual(
        mesh_from_cadquery(_foam_cuff_ring(), "black_foam_cuff", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, 0.525)),
        material=DARK,
        name="foam_cuff",
    )
    leg.visual(
        mesh_from_cadquery(_blue_cuff_band(), "blue_cuff_band", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, 0.537)),
        material=BLUE,
        name="cuff_blue_band",
    )
    leg.visual(
        Cylinder(radius=0.033, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.405)),
        material=BLUE,
        name="distal_blue_collar",
    )
    leg.visual(
        Cylinder(radius=0.023, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.387)),
        material=DARK,
        name="distal_adapter",
    )
    leg.visual(
        Box((0.022, 0.084, 0.020)),
        origin=Origin(xyz=(0.053, 0.0, 0.422)),
        material=BLUE,
        name="handle_mount",
    )
    # The bail handle is its own pivoting link (see handle_pivot below); its
    # feet are captured inside the widened mount bracket.
    handle.visual(
        _grab_handle_mesh(),
        material=DARK,
        name="grab_handle",
    )

    # Polished telescoping pylon with a clamp collar, matching the stilt-like reference.
    leg.visual(
        Cylinder(radius=0.0085, length=0.225),
        origin=Origin(xyz=(0.0, 0.0, 0.410)),
        material=METAL,
        name="inner_tube",
    )
    leg.visual(
        Cylinder(radius=0.0120, length=0.215),
        origin=Origin(xyz=(0.0, 0.0, 0.2575)),
        material=METAL,
        name="outer_tube",
    )
    leg.visual(
        mesh_from_cadquery(_clamp_collar(), "height_adjust_clamp", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, 0.314)),
        material=BLUE,
        name="height_clamp",
    )
    leg.visual(
        Cylinder(radius=0.004, length=0.032),
        origin=Origin(xyz=(0.0, 0.042, 0.326), rpy=(math.pi / 2, 0.0, 0.0)),
        material=FASTENER,
        name="clamp_screw",
    )
    leg.visual(
        Cylinder(radius=0.017, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.385)),
        material=DARK,
        name="upper_tube_collar",
    )
    leg.visual(
        Cylinder(radius=0.017, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material=DARK,
        name="lower_tube_collar",
    )
    # Lifelike cosmetic foam cover enclosing the exposed pylon between socket and ankle.
    leg.visual(
        mesh_from_cadquery(_cosmesis_cover_shape(), "cosmesis_cover", tolerance=0.0008),
        material=COSMESIS_FOAM,
        name="cosmesis_cover",
    )
    leg.visual(
        mesh_from_cadquery(_soft_box((0.067, 0.074, 0.030), 0.011), "ankle_blue_block", tolerance=0.0008),
        origin=Origin(xyz=(-0.008, 0.0, 0.166)),
        material=BLUE,
        name="ankle_blue_block",
    )
    # Two small bolt heads detailing the front face of the ankle block.
    leg.visual(
        Cylinder(radius=0.0042, length=0.008),
        origin=Origin(xyz=(0.026, 0.020, 0.170), rpy=(0.0, math.pi / 2, 0.0)),
        material=FASTENER,
        name="ankle_bolt_0",
    )
    leg.visual(
        Cylinder(radius=0.0042, length=0.008),
        origin=Origin(xyz=(0.026, -0.020, 0.170), rpy=(0.0, math.pi / 2, 0.0)),
        material=FASTENER,
        name="ankle_bolt_1",
    )

    # Fork/yoke around the ankle pitch axis.  The cheeks sit outside the foot lug.
    leg.visual(
        Box((0.047, 0.012, 0.071)),
        origin=Origin(xyz=(-0.005, 0.043, 0.126)),
        material=DARK,
        name="ankle_yoke_0",
    )
    leg.visual(
        Cylinder(radius=0.017, length=0.006),
        origin=Origin(xyz=(-0.005, 0.051, 0.126), rpy=(math.pi / 2, 0.0, 0.0)),
        material=BLUE,
        name="blue_pivot_washer_0",
    )
    leg.visual(
        Cylinder(radius=0.0045, length=0.007),
        origin=Origin(xyz=(-0.005, 0.055, 0.126), rpy=(math.pi / 2, 0.0, 0.0)),
        material=FASTENER,
        name="pivot_screw_0",
    )
    leg.visual(
        Box((0.047, 0.012, 0.071)),
        origin=Origin(xyz=(-0.005, -0.043, 0.126)),
        material=DARK,
        name="ankle_yoke_1",
    )
    leg.visual(
        Cylinder(radius=0.017, length=0.006),
        origin=Origin(xyz=(-0.005, -0.051, 0.126), rpy=(math.pi / 2, 0.0, 0.0)),
        material=BLUE,
        name="blue_pivot_washer_1",
    )
    leg.visual(
        Cylinder(radius=0.0045, length=0.007),
        origin=Origin(xyz=(-0.005, -0.055, 0.126), rpy=(math.pi / 2, 0.0, 0.0)),
        material=FASTENER,
        name="pivot_screw_1",
    )
    leg.visual(
        mesh_from_cadquery(_soft_box((0.073, 0.096, 0.020), 0.008), "yoke_bridge", tolerance=0.0008),
        origin=Origin(xyz=(-0.005, 0.0, 0.158)),
        material=DARK,
        name="yoke_bridge",
    )
    # Foot local frame is exactly on the ankle pitch axis.
    foot.visual(
        mesh_from_cadquery(_sole_shape(), "black_rubber_sole", tolerance=0.0008),
        material=RUBBER,
        name="sole",
    )
    foot.visual(
        mesh_from_cadquery(_foot_upper_shape(), "blue_foot_frame", tolerance=0.0008),
        material=BLUE,
        name="foot_frame",
    )
    foot.visual(
        Cylinder(radius=0.017, length=0.052),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=DARK,
        name="ankle_hub",
    )
    # Brushed-metal bearing race banding the centre of the dark ankle hub.
    foot.visual(
        Cylinder(radius=0.0185, length=0.014),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=METAL,
        name="ankle_bearing_race",
    )
    foot.visual(
        Cylinder(radius=0.007, length=0.074),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=FASTENER,
        name="ankle_pin",
    )
    foot.visual(Box((0.018, 0.078, 0.004)), origin=Origin(xyz=(-0.050, 0.0, -0.126)), material=RUBBER, name="tread_0")
    foot.visual(Box((0.018, 0.078, 0.004)), origin=Origin(xyz=(-0.010, 0.0, -0.126)), material=RUBBER, name="tread_1")
    foot.visual(Box((0.018, 0.078, 0.004)), origin=Origin(xyz=(0.030, 0.0, -0.126)), material=RUBBER, name="tread_2")
    foot.visual(Box((0.018, 0.078, 0.004)), origin=Origin(xyz=(0.070, 0.0, -0.126)), material=RUBBER, name="tread_3")
    foot.visual(Box((0.018, 0.078, 0.004)), origin=Origin(xyz=(0.110, 0.0, -0.126)), material=RUBBER, name="tread_4")
    foot.visual(Box((0.018, 0.078, 0.004)), origin=Origin(xyz=(0.150, 0.0, -0.126)), material=RUBBER, name="tread_5")

    # Red energy-return coil sits on the foot top between the lifted toe pod and midfoot body.
    foot.visual(
        Box((0.038, 0.034, 0.018)),
        origin=Origin(xyz=(0.130, 0.0, -0.076)),
        material=BLUE,
        name="spring_lower_mount",
    )
    foot.visual(
        Cylinder(radius=0.0065, length=0.030),
        origin=Origin(xyz=(0.130, 0.0, -0.071), rpy=(math.pi / 2, 0.0, 0.0)),
        material=FASTENER,
        name="spring_lower_pin",
    )
    foot.visual(
        Box((0.034, 0.034, 0.020)),
        origin=Origin(xyz=(0.050, 0.0, -0.036)),
        material=BLUE,
        name="spring_upper_mount",
    )
    foot.visual(
        Cylinder(radius=0.0065, length=0.030),
        origin=Origin(xyz=(0.050, 0.0, -0.037), rpy=(math.pi / 2, 0.0, 0.0)),
        material=FASTENER,
        name="spring_upper_pin",
    )
    foot.visual(
        _spring_coil_mesh(),
        material=RED,
        name="red_coil_spring",
    )

    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=leg,
        child=foot,
        origin=Origin(xyz=(-0.005, 0.0, 0.126)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.35, upper=0.35, effort=18.0, velocity=4.0),
    )

    # The black bail handle folds about the Y axis through its two mount feet.
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=leg,
        child=handle,
        origin=Origin(xyz=(0.060, 0.0, 0.424)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.6, effort=8.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    leg = object_model.get_part("leg")
    foot = object_model.get_part("foot")
    handle = object_model.get_part("handle")
    ankle = object_model.get_articulation("ankle_pitch")
    handle_pivot = object_model.get_articulation("handle_pivot")

    # The bail handle feet intentionally seat into the neighbouring link mount.
    ctx.allow_overlap(
        leg,
        handle,
        elem_a="handle_mount",
        elem_b="grab_handle",
        reason="bail-handle feet pivot captured inside the leg mount bracket",
    )

    socket_aabb = ctx.part_element_world_aabb(leg, elem="socket_shell")
    cuff_aabb = ctx.part_element_world_aabb(leg, elem="foam_cuff")
    handle_aabb = ctx.part_element_world_aabb(handle, elem="grab_handle")
    spring_aabb = ctx.part_element_world_aabb(foot, elem="red_coil_spring")
    lower_mount_aabb = ctx.part_element_world_aabb(foot, elem="spring_lower_mount")
    upper_mount_aabb = ctx.part_element_world_aabb(foot, elem="spring_upper_mount")
    clamp_aabb = ctx.part_element_world_aabb(leg, elem="height_clamp")
    inner_tube_aabb = ctx.part_element_world_aabb(leg, elem="inner_tube")
    outer_tube_aabb = ctx.part_element_world_aabb(leg, elem="outer_tube")
    sole_aabb = ctx.part_element_world_aabb(foot, elem="sole")

    ctx.check(
        "single device scale is close to 55 cm tall",
        cuff_aabb is not None
        and sole_aabb is not None
        and 0.54 <= cuff_aabb[1][2] - sole_aabb[0][2] <= 0.63,
        details=f"cuff={cuff_aabb}, sole={sole_aabb}",
    )
    ctx.check(
        "blue socket remains about 12 cm across",
        socket_aabb is not None
        and 0.112 <= socket_aabb[1][0] - socket_aabb[0][0] <= 0.125
        and 0.112 <= socket_aabb[1][1] - socket_aabb[0][1] <= 0.125,
        details=f"socket={socket_aabb}",
    )
    ctx.check(
        "chunky black foam cuff wraps the top of the blue socket",
        cuff_aabb is not None
        and socket_aabb is not None
        and cuff_aabb[0][2] < socket_aabb[1][2] < cuff_aabb[1][2]
        and (cuff_aabb[1][0] - cuff_aabb[0][0]) > (socket_aabb[1][0] - socket_aabb[0][0]) + 0.020
        and (cuff_aabb[1][1] - cuff_aabb[0][1]) > (socket_aabb[1][1] - socket_aabb[0][1]) + 0.020,
        details=f"cuff={cuff_aabb}, socket={socket_aabb}",
    )
    ctx.check(
        "front black grab handle is present below the cuff",
        handle_aabb is not None
        and socket_aabb is not None
        and cuff_aabb is not None
        and handle_aabb[0][2] < cuff_aabb[0][2]
        and handle_aabb[1][0] > socket_aabb[1][0] + 0.015
        and handle_aabb[1][1] - handle_aabb[0][1] > 0.075
        and handle_aabb[1][2] - handle_aabb[0][2] > 0.060,
        details=f"handle={handle_aabb}, socket={socket_aabb}, cuff={cuff_aabb}",
    )
    ctx.check(
        "height-adjustment clamp collars the telescoping metal strut",
        clamp_aabb is not None
        and inner_tube_aabb is not None
        and outer_tube_aabb is not None
        and inner_tube_aabb[0][2] < (clamp_aabb[0][2] + clamp_aabb[1][2]) * 0.5 < inner_tube_aabb[1][2]
        and outer_tube_aabb[0][2] < (clamp_aabb[0][2] + clamp_aabb[1][2]) * 0.5 < outer_tube_aabb[1][2]
        and (clamp_aabb[1][0] - clamp_aabb[0][0]) > (outer_tube_aabb[1][0] - outer_tube_aabb[0][0]) + 0.010,
        details=f"clamp={clamp_aabb}, inner={inner_tube_aabb}, outer={outer_tube_aabb}",
    )
    toe_aabb = ctx.part_element_world_aabb(foot, elem="tread_5")
    mid_tread_aabb = ctx.part_element_world_aabb(foot, elem="tread_2")
    ctx.check(
        "prominent red coil spring rides on top of the foot between toe pod and midfoot",
        spring_aabb is not None
        and toe_aabb is not None
        and mid_tread_aabb is not None
        and 0.035 <= spring_aabb[0][2] <= 0.060
        and 0.085 <= spring_aabb[1][2] <= 0.120
        and mid_tread_aabb[1][0] < spring_aabb[0][0] < spring_aabb[1][0] < toe_aabb[1][0]
        and spring_aabb[0][1] < -0.004
        and spring_aabb[1][1] > 0.004
        and (spring_aabb[1][1] - spring_aabb[0][1]) < 0.045,
        details=f"spring={spring_aabb}, mid_tread={mid_tread_aabb}, toe={toe_aabb}",
    )
    ctx.check(
        "red coil spring is captured between forefoot lower and upper mounts",
        spring_aabb is not None
        and lower_mount_aabb is not None
        and upper_mount_aabb is not None
        and lower_mount_aabb[0][2] - 0.004 <= spring_aabb[0][2] <= lower_mount_aabb[1][2] + 0.004
        and upper_mount_aabb[0][2] - 0.004 <= spring_aabb[1][2] <= upper_mount_aabb[1][2] + 0.014
        and spring_aabb[1][0] > lower_mount_aabb[0][0]
        and lower_mount_aabb[1][0] > spring_aabb[0][0]
        and spring_aabb[1][0] > upper_mount_aabb[0][0]
        and upper_mount_aabb[1][0] > spring_aabb[0][0]
        and spring_aabb[1][1] > lower_mount_aabb[0][1]
        and lower_mount_aabb[1][1] > spring_aabb[0][1],
        details=f"spring={spring_aabb}, lower_mount={lower_mount_aabb}, upper_mount={upper_mount_aabb}",
    )
    ctx.expect_contact(
        foot,
        leg,
        elem_a="ankle_pin",
        elem_b="ankle_yoke_0",
        contact_tol=0.0005,
        name="ankle pin is captured by one yoke cheek",
    )
    ctx.expect_contact(
        foot,
        leg,
        elem_a="ankle_pin",
        elem_b="ankle_yoke_1",
        contact_tol=0.0005,
        name="ankle pin is captured by the opposite yoke cheek",
    )

    rest_toe = ctx.part_element_world_aabb(foot, elem="tread_5")
    with ctx.pose({ankle: 0.30}):
        pitched_toe = ctx.part_element_world_aabb(foot, elem="tread_5")
        ctx.expect_contact(
            foot,
            leg,
            elem_a="ankle_pin",
            elem_b="ankle_yoke_0",
            contact_tol=0.0005,
            name="dorsiflexed ankle remains on the yoke pin axis",
        )

    ctx.check(
        "revolute ankle pitch lifts the toe side of the foot",
        rest_toe is not None and pitched_toe is not None and pitched_toe[1][2] > rest_toe[1][2] + 0.040,
        details=f"rest={rest_toe}, pitched={pitched_toe}",
    )

    # The foot-borne coil must actually travel with the foot when the ankle pitches.
    rest_spring = ctx.part_element_world_aabb(foot, elem="red_coil_spring")
    rest_spring_c = None if rest_spring is None else _aabb_center(rest_spring)
    with ctx.pose({ankle: 0.30}):
        pitched_spring = ctx.part_element_world_aabb(foot, elem="red_coil_spring")
    pitched_spring_c = None if pitched_spring is None else _aabb_center(pitched_spring)
    spring_travel = (
        0.0
        if rest_spring_c is None or pitched_spring_c is None
        else math.dist(rest_spring_c, pitched_spring_c)
    )
    ctx.check(
        "red coil spring is bound to the foot and travels with ankle pitch",
        spring_travel > 0.005,
        details=f"spring_travel={spring_travel:.4f}",
    )

    # The black bail handle must rotate on its pivot.
    rest_handle = ctx.part_element_world_aabb(handle, elem="grab_handle")
    rest_handle_c = None if rest_handle is None else _aabb_center(rest_handle)
    with ctx.pose({handle_pivot: 1.2}):
        rotated_handle = ctx.part_element_world_aabb(handle, elem="grab_handle")
    rotated_handle_c = None if rotated_handle is None else _aabb_center(rotated_handle)
    handle_swing = (
        0.0
        if rest_handle_c is None or rotated_handle_c is None
        else math.dist(rest_handle_c, rotated_handle_c)
    )
    ctx.check(
        "black bail handle rotates about its mount pivot",
        handle_swing > 0.025,
        details=f"handle_swing={handle_swing:.4f}",
    )

    # --- Cosmesis cover (variant-specific assertion) ---
    cosmesis_aabb = ctx.part_element_world_aabb(leg, elem="cosmesis_cover")
    ctx.check(
        "cosmesis_cover wraps the pylon between socket and ankle",
        cosmesis_aabb is not None
        and socket_aabb is not None
        and sole_aabb is not None
        and cosmesis_aabb[1][2] < socket_aabb[0][2] + 0.025
        and cosmesis_aabb[0][2] > sole_aabb[0][2] + 0.060
        and (cosmesis_aabb[1][0] - cosmesis_aabb[0][0]) > 0.060
        and (cosmesis_aabb[1][1] - cosmesis_aabb[0][1]) > 0.060,
        details=f"cosmesis={cosmesis_aabb}, socket={socket_aabb}, sole={sole_aabb}",
    )
    ctx.check(
        "cosmesis_cover has a tapered leg-shaped calf profile",
        cosmesis_aabb is not None
        and (cosmesis_aabb[1][1] - cosmesis_aabb[0][1]) >= 0.080
        and (cosmesis_aabb[1][0] - cosmesis_aabb[0][0]) >= 0.060
        and cosmesis_aabb[1][2] - cosmesis_aabb[0][2] > 0.150,
        details=f"cosmesis={cosmesis_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
