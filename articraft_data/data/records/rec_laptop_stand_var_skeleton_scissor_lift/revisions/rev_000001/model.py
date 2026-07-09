from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rounded_plate_mesh(name: str, length: float, width: float, thickness: float, radius: float):
    return _mesh(
        name,
        ExtrudeGeometry(
            rounded_rect_profile(length, width, radius, corner_segments=10),
            thickness,
            center=True,
        ),
    )


# ── Scissor geometry constants ──────────────────────────────────────────
L = 0.220            # link length
HALF_L = L / 2.0
LINK_W = 0.022       # link visible width (Z when horizontal)
LINK_T = 0.005       # link side-to-side thickness (Y)
Y_INNER = 0.038      # inner link pair Y offset
Y_OUTER = 0.058      # outer link pair Y offset
PIVOT_Z = 0.068      # lower pivot height above turntable origin
REST = 0.25          # rest angle (rad from horizontal) baked into geometry

# Pre-computed baked positions in part frames
_CX = HALF_L * math.cos(REST)   # cross-joint X in inner frame
_CZ = HALF_L * math.sin(REST)   # cross-joint Z in inner frame
_TX = -_CX                       # tray-level X in outer frame  (upper end)
_TZ = _CZ                        # tray-level Z in outer frame


def _bar_from_origin(
    name: str,
    *,
    length: float,
    width: float,
    thickness: float,
    y_offset: float,
    angle: float,
):
    """Bar extending from local origin along +X rotated by *angle* in XZ."""
    g = ExtrudeGeometry(
        rounded_rect_profile(length, width, width * 0.40, corner_segments=8),
        thickness,
        center=True,
    )
    g.rotate_x(math.pi / 2.0)          # bar along X, width→Z, thickness→Y
    g.translate(length / 2.0, 0.0, 0.0)  # centre at L/2
    g.rotate_y(-angle)                   # tilt in XZ plane
    g.translate(0.0, y_offset, 0.0)
    return _mesh(name, g)


def _bar_centered(
    name: str,
    *,
    length: float,
    width: float,
    thickness: float,
    y_offset: float,
    angle: float,
):
    """Bar centred at local origin, extending ±L/2 at *angle* in XZ."""
    g = ExtrudeGeometry(
        rounded_rect_profile(length, width, width * 0.40, corner_segments=8),
        thickness,
        center=True,
    )
    g.rotate_x(math.pi / 2.0)
    g.rotate_y(-angle)
    g.translate(0.0, y_offset, 0.0)
    return _mesh(name, g)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="scissor_lift_laptop_stand")

    # ── materials ──────────────────────────────────────────────────────
    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.017, 1.0))
    soft_black = model.material("soft_black", rgba=(0.055, 0.057, 0.060, 1.0))
    groove_black = model.material("groove_black", rgba=(0.0, 0.0, 0.0, 1.0))
    hardware = model.material("brushed_silver", rgba=(0.76, 0.76, 0.74, 1.0))
    laptop_silver = model.material("thin_laptop_silver", rgba=(0.80, 0.82, 0.83, 1.0))

    # ====================================================================
    # BASE  (KEEP)
    # ====================================================================
    base = model.part("base")
    base.visual(
        _rounded_plate_mesh("rounded_base_plate", 0.34, 0.24, 0.016, 0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=matte_black,
        name="base_plate",
    )
    for side, y in enumerate((-0.078, 0.078)):
        base.visual(
            Box((0.240, 0.006, 0.0022)),
            origin=Origin(xyz=(0.0, y, 0.0149)),
            material=groove_black,
            name=f"side_groove_{side}",
        )
    for side, x in enumerate((-0.126, 0.126)):
        base.visual(
            Box((0.006, 0.156, 0.0022)),
            origin=Origin(xyz=(x, 0.0, 0.0149)),
            material=groove_black,
            name=f"end_groove_{side}",
        )

    # ====================================================================
    # TURNTABLE  (modified for scissor lower pivot + slide rail)
    # ====================================================================
    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.090, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=soft_black,
        name="outer_rotation_disk",
    )
    turntable.visual(
        Cylinder(radius=0.070, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=matte_black,
        name="upper_turntable_disk",
    )
    turntable.visual(
        Cylinder(radius=0.046, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=matte_black,
        name="short_pedestal",
    )
    # Lower fixed-pivot brackets for inner link pair
    for side, y in enumerate((-Y_INNER, Y_INNER)):
        turntable.visual(
            Box((0.034, 0.020, 0.036)),
            origin=Origin(xyz=(0.0, y, PIVOT_Z)),
            material=matte_black,
            name=f"base_lug_{side}",
        )
        turntable.visual(
            Cylinder(radius=0.018, length=0.004),
            origin=Origin(xyz=(0.0, y * 1.15, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hardware,
            name=f"base_pivot_washer_{side}",
        )
    # Lower horizontal slide rail (connects to pedestal for connectivity)
    turntable.visual(
        Box((0.170, 0.096, 0.008)),
        origin=Origin(xyz=(0.085, 0.0, PIVOT_Z - 0.018)),
        material=groove_black,
        name="lower_slide_rail",
    )

    # ====================================================================
    # SCISSOR INNER — inner crossed link pair  (loop-emitted)
    # ====================================================================
    scissor_inner = model.part("scissor_inner")
    for i, y in enumerate((-Y_INNER, Y_INNER)):
        scissor_inner.visual(
            _bar_from_origin(
                f"inner_link_bar_{i}",
                length=L, width=LINK_W, thickness=LINK_T,
                y_offset=y, angle=REST,
            ),
            material=matte_black,
            name=f"inner_link_{i}",
        )
    # Lower pivot shaft at origin
    shaft_span = Y_INNER * 2.0 + LINK_T + 0.016
    scissor_inner.visual(
        Cylinder(radius=0.008, length=shaft_span),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="base_shaft",
    )
    # Center crossing shaft — spans full width to engage outer links
    center_shaft_span = Y_OUTER * 2.0 + 0.004
    scissor_inner.visual(
        Cylinder(radius=0.008, length=center_shaft_span),
        origin=Origin(xyz=(_CX, 0.0, _CZ), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="center_shaft",
    )
    for i, y in enumerate((-(Y_INNER + 0.005), Y_INNER + 0.005)):
        scissor_inner.visual(
            Cylinder(radius=0.015, length=0.005),
            origin=Origin(xyz=(_CX, y, _CZ), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hardware,
            name=f"inner_center_washer_{i}",
        )

    # ====================================================================
    # SCISSOR OUTER — outer crossed link pair  (loop-emitted)
    # ====================================================================
    OUTER_ANGLE = math.pi - REST   # baked angle for outer links
    scissor_outer = model.part("scissor_outer")
    for i, y in enumerate((-Y_OUTER, Y_OUTER)):
        scissor_outer.visual(
            _bar_centered(
                f"outer_link_bar_{i}",
                length=L, width=LINK_W, thickness=LINK_T,
                y_offset=y, angle=OUTER_ANGLE,
            ),
            material=matte_black,
            name=f"outer_link_{i}",
        )
    # Center pivot washers — overlap with center shaft for connectivity
    for i, y in enumerate((-(Y_OUTER - 0.001), Y_OUTER - 0.001)):
        scissor_outer.visual(
            Cylinder(radius=0.017, length=0.008),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hardware,
            name=f"outer_center_washer_{i}",
        )
    # Upper pivot (tray-end) shaft at baked tray-level position
    shaft_span_outer = Y_OUTER * 2.0 + LINK_T + 0.016
    scissor_outer.visual(
        Cylinder(radius=0.008, length=shaft_span_outer),
        origin=Origin(xyz=(_TX, 0.0, _TZ), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="tray_shaft",
    )
    for i, y in enumerate((-(Y_OUTER + 0.002), Y_OUTER + 0.002)):
        scissor_outer.visual(
            Cylinder(radius=0.017, length=0.006),
            origin=Origin(xyz=(_TX, y, _TZ), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hardware,
            name=f"tray_pivot_washer_{i}",
        )
    # Lower slide rollers at the opposite end of outer links
    _SX = -_TX   # slide-end X in outer frame
    _SZ = -_TZ   # slide-end Z in outer frame
    for i, y in enumerate((-Y_OUTER, Y_OUTER)):
        scissor_outer.visual(
            Cylinder(radius=0.011, length=0.010),
            origin=Origin(xyz=(_SX, y, _SZ), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hardware,
            name=f"lower_slide_roller_{i}",
        )

    # ====================================================================
    # SLIDER CARRIAGE  (counter-rotates to keep tray level)
    # ====================================================================
    slider_carriage = model.part("slider_carriage")
    cw = Y_OUTER + 0.010
    # Plate sits above lugs to capture pivot
    slider_carriage.visual(
        Box((0.060, cw * 2.0, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=matte_black,
        name="carriage_plate",
    )
    # Lugs extend down from plate to capture the tray shaft at origin
    for side, y in enumerate((-0.044, 0.044)):
        slider_carriage.visual(
            Box((0.026, 0.018, 0.028)),
            origin=Origin(xyz=(0.0, y, 0.002)),
            material=matte_black,
            name=f"carriage_lug_{side}",
        )
        slider_carriage.visual(
            Cylinder(radius=0.014, length=0.005),
            origin=Origin(xyz=(0.0, y * 1.15, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hardware,
            name=f"carriage_bushing_{side}",
        )

    # ====================================================================
    # UPPER TRAY  (KEEP)
    # ====================================================================
    tray = model.part("upper_tray")
    for side, y in enumerate((-0.044, 0.044)):
        tray.visual(
            Box((0.032, 0.018, 0.020)),
            origin=Origin(xyz=(0.0, y, 0.020)),
            material=matte_black,
            name=f"tray_lug_{side}",
        )
    tray.visual(
        Box((0.082, 0.230, 0.014)),
        origin=Origin(xyz=(0.022, 0.0, 0.032)),
        material=matte_black,
        name="front_tray_rail",
    )
    for side, y in enumerate((-0.092, 0.092)):
        tray.visual(
            Box((0.300, 0.018, 0.014)),
            origin=Origin(xyz=(-0.110, y, 0.032)),
            material=matte_black,
            name=f"side_tray_rail_{side}",
        )
    tray.visual(
        _rounded_plate_mesh("thin_laptop_plate", 0.300, 0.215, 0.006, 0.014),
        origin=Origin(xyz=(-0.110, 0.0, 0.042)),
        material=laptop_silver,
        name="laptop_plate",
    )
    for side, y in enumerate((-0.066, 0.066)):
        tray.visual(
            Box((0.022, 0.058, 0.066)),
            origin=Origin(xyz=(0.056, y, 0.072)),
            material=soft_black,
            name=f"front_lip_{side}",
        )
        tray.visual(
            Box((0.034, 0.064, 0.014)),
            origin=Origin(xyz=(0.044, y, 0.044)),
            material=soft_black,
            name=f"lip_foot_{side}",
        )
    # Underside slide rail (touches tray lugs for connectivity)
    tray.visual(
        Box((0.080, 0.100, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=groove_black,
        name="upper_slide_rail",
    )

    # ====================================================================
    # ARTICULATIONS
    # ====================================================================

    # 1. Turntable yaw  (KEEP)
    model.articulation(
        "turntable_yaw",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0),
    )

    # 2. Scissor spread — driving revolute that sets lift height.
    #    Positive q rotates inner links further upward from the baked rest angle.
    model.articulation(
        "scissor_spread",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=scissor_inner,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.0, lower=0.0, upper=0.85),
    )

    # 3. Scissor cross — counter-rotates outer links to maintain the X.
    #    Mimic: q_cross = -2 * q_spread
    model.articulation(
        "scissor_cross",
        ArticulationType.REVOLUTE,
        parent=scissor_inner,
        child=scissor_outer,
        origin=Origin(xyz=(_CX, 0.0, _CZ)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.0, lower=-1.70, upper=0.0),
        mimic=Mimic(joint="scissor_spread", multiplier=-2.0, offset=0.0),
    )

    # 4. Tray level — counter-rotates carriage so the tray stays horizontal.
    #    Mimic: q_level = +1 * q_spread
    model.articulation(
        "tray_level",
        ArticulationType.REVOLUTE,
        parent=scissor_outer,
        child=slider_carriage,
        origin=Origin(xyz=(_TX, 0.0, _TZ)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.0, lower=0.0, upper=0.85),
        mimic=Mimic(joint="scissor_spread", multiplier=1.0, offset=0.0),
    )

    # 5. Tray tilt — user-adjustable forward / back  (KEEP)
    model.articulation(
        "tray_tilt",
        ArticulationType.REVOLUTE,
        parent=slider_carriage,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.2, lower=-0.35, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    turntable = object_model.get_part("turntable")
    inner = object_model.get_part("scissor_inner")
    outer = object_model.get_part("scissor_outer")
    carriage = object_model.get_part("slider_carriage")
    tray = object_model.get_part("upper_tray")

    spread = object_model.get_articulation("scissor_spread")
    cross = object_model.get_articulation("scissor_cross")
    level = object_model.get_articulation("tray_level")
    tilt = object_model.get_articulation("tray_tilt")
    yaw = object_model.get_articulation("turntable_yaw")

    # ── intentional overlap allowances ─────────────────────────────────

    # Inner link bars pass through turntable pivot brackets (pivot nesting)
    for i in range(2):
        ctx.allow_overlap(
            turntable, inner,
            elem_a=f"base_lug_{i}", elem_b=f"inner_link_{i}",
            reason="Inner link bar passes through the turntable bracket at the lower pivot.",
        )
        ctx.expect_contact(
            turntable, inner,
            elem_a=f"base_lug_{i}", elem_b=f"inner_link_{i}",
            contact_tol=0.005,
            name=f"base_lug_{i} contacts inner_link_{i}",
        )

    # Base shaft captured through turntable lugs
    for elem in ("base_lug_0", "base_lug_1"):
        ctx.allow_overlap(
            turntable, inner,
            elem_a=elem, elem_b="base_shaft",
            reason="Lower pivot shaft is captured through the turntable bracket.",
        )

    # Tray shaft captured through carriage lugs
    for elem in ("carriage_lug_0", "carriage_lug_1"):
        ctx.allow_overlap(
            carriage, outer,
            elem_a=elem, elem_b="tray_shaft",
            reason="Upper pivot shaft is captured through the carriage lug.",
        )
        ctx.expect_overlap(
            carriage, outer,
            axes="yz", elem_a=elem, elem_b="tray_shaft",
            min_overlap=0.004,
            name=f"{elem} captures tray shaft",
        )
    # Tray shaft nested inside carriage bushings
    for elem in ("carriage_bushing_0", "carriage_bushing_1"):
        ctx.allow_overlap(
            outer, carriage,
            elem_a="tray_shaft", elem_b=elem,
            reason="Tray pivot shaft passes through the carriage bushing at the upper pivot.",
        )

    # Center shaft passes through outer link bars and washers at the crossing
    for i in range(2):
        ctx.allow_overlap(
            inner, outer,
            elem_a="center_shaft", elem_b=f"outer_link_{i}",
            reason="The center pivot shaft passes through the outer link at the scissor crossing.",
        )
        ctx.allow_overlap(
            inner, outer,
            elem_a="center_shaft", elem_b=f"outer_center_washer_{i}",
            reason="The center pivot shaft is captured inside the outer center washer.",
        )
        ctx.expect_overlap(
            inner, outer,
            axes="xz", elem_a="center_shaft", elem_b=f"outer_center_washer_{i}",
            min_overlap=0.004,
            name=f"center_shaft engaged with outer_center_washer_{i}",
        )

    # Tray lugs seat against the carriage at the tilt joint
    for elem in ("tray_lug_0", "tray_lug_1"):
        ctx.allow_overlap(
            tray, carriage,
            elem_a=elem, elem_b="carriage_plate",
            reason="Tray pivot lug seats against the carriage plate.",
        )
    # Underside slide rail passes close to carriage at the pivot
    ctx.allow_overlap(
        tray, carriage,
        elem_a="upper_slide_rail", elem_b="carriage_plate",
        reason="The tray underside slide rail passes near the carriage plate at the tilt pivot.",
    )
    ctx.expect_contact(
        tray, carriage,
        elem_a="tray_lug_0", elem_b="carriage_plate",
        contact_tol=0.005,
        name="tray lug contacts carriage plate at tilt joint",
    )

    # ── structural checks ──────────────────────────────────────────────

    ctx.check(
        "broad rounded base plate present",
        base.get_visual("base_plate") is not None
        and len([v for v in base.visuals if "groove" in (v.name or "")]) >= 4,
        details="Expected rounded base plate and shallow groove visuals.",
    )
    ctx.check(
        "turntable disk and pedestal present",
        turntable.get_visual("upper_turntable_disk") is not None
        and turntable.get_visual("short_pedestal") is not None,
        details="Expected central rotating disk and pedestal.",
    )
    ctx.check(
        "scissor inner link pair present",
        inner.get_visual("inner_link_0") is not None
        and inner.get_visual("inner_link_1") is not None,
        details="Expected two inner crossed links loop-emitted with indexed names.",
    )
    ctx.check(
        "scissor outer link pair present",
        outer.get_visual("outer_link_0") is not None
        and outer.get_visual("outer_link_1") is not None,
        details="Expected two outer crossed links loop-emitted with indexed names.",
    )
    ctx.check(
        "pivot washers and shafts present",
        all(
            inner.get_visual(n) is not None
            for n in ("inner_center_washer_0", "inner_center_washer_1",
                       "base_shaft", "center_shaft")
        )
        and all(
            outer.get_visual(n) is not None
            for n in ("tray_pivot_washer_0", "tray_pivot_washer_1", "tray_shaft")
        ),
        details="Expected silver pivot hardware at crossing and endpoints.",
    )
    ctx.check(
        "front retaining lips present",
        tray.get_visual("front_lip_0") is not None
        and tray.get_visual("front_lip_1") is not None,
        details="Expected two front clamps/lips.",
    )

    # ── joint topology ─────────────────────────────────────────────────

    nonfixed = [
        j for j in object_model.articulations
        if str(j.articulation_type).split(".")[-1] != "FIXED"
    ]
    ctx.check(
        "scissor lift has multiple non-fixed joints",
        len(nonfixed) >= 4
        and spread is not None
        and cross is not None
        and tilt is not None
        and yaw is not None,
        details="Expected scissor_spread, scissor_cross, tray_tilt, and turntable_yaw.",
    )
    ctx.check(
        "scissor_cross mimics scissor_spread",
        cross.mimic is not None and cross.mimic.joint == "scissor_spread",
        details="Cross pivot must be coupled to the driving spread joint.",
    )

    # ── geometry relationships ─────────────────────────────────────────

    ctx.expect_gap(
        turntable, base, axis="z",
        max_gap=0.001, max_penetration=0.001,
        name="turntable sits on base",
    )

    # ── pose test: scissor_spread raises the tray vertically ───────────

    rest_aabb = ctx.part_world_aabb(tray)
    with ctx.pose({spread: 0.70}):
        lifted_aabb = ctx.part_world_aabb(tray)
    ctx.check(
        "scissor_spread raises tray vertically",
        rest_aabb is not None
        and lifted_aabb is not None
        and lifted_aabb[0][2] > rest_aabb[0][2] + 0.020,
        details=f"rest_min_z={rest_aabb}, lifted_min_z={lifted_aabb}",
    )

    # Tray should not drift far laterally (level-lift behaviour)
    rest_center = ctx.part_world_position(tray)
    with ctx.pose({spread: 0.70}):
        lifted_center = ctx.part_world_position(tray)
    ctx.check(
        "tray rises without large lateral drift",
        rest_center is not None
        and lifted_center is not None
        and abs(lifted_center[0] - rest_center[0]) < 0.080,
        details=f"rest_x={rest_center}, lifted_x={lifted_center}",
    )

    return ctx.report()


object_model = build_object_model()
