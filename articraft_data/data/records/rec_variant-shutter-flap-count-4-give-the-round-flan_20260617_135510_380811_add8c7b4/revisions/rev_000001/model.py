from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# World frame: +Y = front (out of the wall), +Z = up. Fan axis = world Y.
# CadQuery solids are built about local +Z and every such visual is rotated
# with rpy=(pi/2, 0, 0), which maps local +Z -> world -Y (rearward) and
# local +Y -> world +Z. A piece whose FRONT face should sit at world y = Y0
# is therefore extruded from local z=0 to z=depth and placed at xyz=(0, Y0, 0).

# ---- Housing (fixed flange + collar + drum) -------------------------------
FLANGE_R = 0.130  # 0.26 m outer diameter
OPENING_R = 0.080  # circular collar opening
FLANGE_T = 0.006
COLLAR_R = 0.094
COLLAR_T = 0.009  # front face at +0.008, embeds 1 mm into the flange
COLLAR_FRONT_Y = 0.008
DRUM_OUTER_R = 0.100
DRUM_INNER_R = 0.096
DRUM_LEN = 0.067  # world y in [-0.072, -0.005], embeds 1 mm into the flange
DRUM_FRONT_Y = -0.005
DRUM_REAR_Y = DRUM_FRONT_Y - DRUM_LEN  # -0.072 -> total depth 0.080 m

BOLT_COUNT = 8
BOLT_CIRCLE_R = 0.1165
BOLT_HEX_D = 0.012  # circumscribed hex diameter
BOLT_HEAD_H = 0.005  # 1 mm embedded, 4 mm proud of the flange face

# ---- Motor support spider --------------------------------------------------
BUSHING_OUTER_R = 0.017
BUSHING_BORE_R = 0.0035  # interference fit: the 4 mm shaft is pressed into this bore
BUSHING_LEN = 0.024
BUSHING_FRONT_Y = -0.020  # world y in [-0.044, -0.020]
STRUT_LEN = 0.083  # radial r in [0.015, 0.098]: embeds into bushing and drum
STRUT_MID_R = 0.0565
STRUT_Y = -0.032
STRUT_AXIAL_W = 0.009
STRUT_TANGENTIAL_T = 0.005

# ---- Impeller (continuous spin about world +Y) -----------------------------
SPIN_Y = -0.014  # hub mid-plane
HUB_R = 0.022
HUB_T = 0.010
BLADE_COUNT = 8
BLADE_ROOT_R = 0.020
BLADE_TIP_R = 0.074
BLADE_ROOT_HALF_CHORD = 0.013
BLADE_TIP_HALF_CHORD = 0.025
BLADE_T = 0.0028
BLADE_PITCH_DEG = 22.0
SHAFT_R = 0.004
SHAFT_LEN = 0.022  # part-local y in [-0.026, -0.004]; tip reaches world -0.040

# ---- Backdraft flaps (4x revolute, evenly spaced around the drum) ---------
FLAP_COUNT = 4
HINGE_Y_BASE = -0.060
HINGE_Y_SPACING = 0.004  # stagger flaps axially to prevent coplanar overlap
HINGE_R = 0.085  # radial distance of hinge from drum axis
PIN_R = 0.003
PIN_LEN = 0.064
BRACKET_X = 0.0285  # bracket offset along tangent from hinge center
FLAP_R = 0.080
FLAP_T = 0.0025
FLAP_DROP = 0.085  # disc center sits FLAP_DROP radially inward from the pin
KNUCKLE_X = 0.018
KNUCKLE_OUTER_R = 0.0055
KNUCKLE_BORE_R = 0.0025  # interference fit: knuckles pressed around the 3 mm pin
KNUCKLE_LEN = 0.010
FLAP_OPEN_LIMIT = 1.40  # ~80 degrees rearward


def _tube(outer_r: float, inner_r: float, depth: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(outer_r).circle(inner_r).extrude(depth)


def _rot_y(x: float, y: float, z: float, theta: float) -> tuple[float, float, float]:
    """Rotate point (x, y, z) about the Y axis by angle theta (radians)."""
    c, s = math.cos(theta), math.sin(theta)
    return (c * x + s * z, y, -s * x + c * z)


def _rotor_solid() -> cq.Workplane:
    """Hub disc plus 8 flat trapezoidal pinwheel blades, axis = local +Z."""
    rotor = (
        cq.Workplane("XY")
        .circle(HUB_R)
        .extrude(HUB_T)
        .translate((0.0, 0.0, -HUB_T / 2.0))
    )
    blade_pts = [
        (BLADE_ROOT_R, -BLADE_ROOT_HALF_CHORD),
        (BLADE_TIP_R, -BLADE_TIP_HALF_CHORD),
        (BLADE_TIP_R, BLADE_TIP_HALF_CHORD),
        (BLADE_ROOT_R, BLADE_ROOT_HALF_CHORD),
    ]
    blade = (
        cq.Workplane("XY")
        .polyline(blade_pts)
        .close()
        .extrude(BLADE_T)
        .translate((0.0, 0.0, -BLADE_T / 2.0))
        .rotate((0, 0, 0), (1, 0, 0), BLADE_PITCH_DEG)
    )
    for k in range(BLADE_COUNT):
        rotor = rotor.union(blade.rotate((0, 0, 0), (0, 0, 1), k * 360.0 / BLADE_COUNT))
    return rotor


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_vent_fan")

    paint_grey = model.material("paint_pale_grey", rgba=(0.78, 0.79, 0.81, 1.0))
    rotor_grey = model.material("rotor_metal_grey", rgba=(0.67, 0.68, 0.71, 1.0))
    bolt_steel = model.material("bolt_steel_dark", rgba=(0.40, 0.41, 0.44, 1.0))
    flap_zinc = model.material("flap_zinc_grey", rgba=(0.58, 0.60, 0.63, 1.0))

    # ---- housing -----------------------------------------------------------
    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_tube(FLANGE_R, OPENING_R, FLANGE_T), "flange_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=paint_grey,
        name="flange_plate",
    )
    housing.visual(
        mesh_from_cadquery(_tube(COLLAR_R, OPENING_R, COLLAR_T), "collar_ring"),
        origin=Origin(xyz=(0.0, COLLAR_FRONT_Y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=paint_grey,
        name="collar_ring",
    )
    housing.visual(
        mesh_from_cadquery(_tube(DRUM_OUTER_R, DRUM_INNER_R, DRUM_LEN), "drum_shell"),
        origin=Origin(xyz=(0.0, DRUM_FRONT_Y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=paint_grey,
        name="drum_shell",
    )

    bolt_mesh = mesh_from_cadquery(
        cq.Workplane("XY").polygon(6, BOLT_HEX_D).extrude(BOLT_HEAD_H),
        "hex_bolt_head",
    )
    for i in range(BOLT_COUNT):
        ang = 2.0 * math.pi * i / BOLT_COUNT
        housing.visual(
            bolt_mesh,
            origin=Origin(
                xyz=(
                    BOLT_CIRCLE_R * math.cos(ang),
                    BOLT_HEAD_H - 0.001,
                    BOLT_CIRCLE_R * math.sin(ang),
                ),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=bolt_steel,
            name=f"flange_bolt_{i}",
        )

    housing.visual(
        mesh_from_cadquery(
            _tube(BUSHING_OUTER_R, BUSHING_BORE_R, BUSHING_LEN), "motor_bushing"
        ),
        origin=Origin(xyz=(0.0, BUSHING_FRONT_Y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=paint_grey,
        name="motor_bushing",
    )
    for i in range(4):
        ang = math.radians(45.0 + 90.0 * i)
        housing.visual(
            Box((STRUT_LEN, STRUT_AXIAL_W, STRUT_TANGENTIAL_T)),
            origin=Origin(
                xyz=(STRUT_MID_R * math.cos(ang), STRUT_Y, STRUT_MID_R * math.sin(ang)),
                rpy=(0.0, -ang, 0.0),
            ),
            material=paint_grey,
            name=f"motor_strut_{i}",
        )

    # ---- Hinge hardware for 4 backdraft flaps (brackets + pins) -----------
    for i in range(FLAP_COUNT):
        theta = i * 2.0 * math.pi / FLAP_COUNT
        hinge_y = HINGE_Y_BASE + i * HINGE_Y_SPACING
        hx = HINGE_R * math.sin(theta)
        hz = HINGE_R * math.cos(theta)

        # Two mounting brackets per hinge, offset along tangent direction
        for j, sx in enumerate((-1.0, 1.0)):
            bx, _, bz = _rot_y(sx * BRACKET_X, 0.0, 0.002, theta)
            housing.visual(
                Box((0.007, 0.006, 0.013)),
                origin=Origin(
                    xyz=(hx + bx, hinge_y, hz + bz),
                    rpy=(0.0, theta, 0.0),
                ),
                material=paint_grey,
                name=f"hinge_bracket_{i}_{j}",
            )

        # Hinge pin oriented along the tangent at this angular position
        housing.visual(
            Cylinder(radius=PIN_R, length=PIN_LEN),
            origin=Origin(
                xyz=(hx, hinge_y, hz),
                rpy=(0.0, math.pi / 2.0 + theta, 0.0),
            ),
            material=bolt_steel,
            name=f"hinge_pin_{i}",
        )

    # ---- impeller ----------------------------------------------------------
    impeller = model.part("impeller")
    impeller.visual(
        mesh_from_cadquery(_rotor_solid(), "impeller_rotor"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rotor_grey,
        name="impeller_rotor",
    )
    impeller.visual(
        Cylinder(radius=SHAFT_R, length=SHAFT_LEN),
        origin=Origin(xyz=(0.0, -0.015, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bolt_steel,
        name="impeller_shaft",
    )

    model.articulation(
        "impeller_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=impeller,
        origin=Origin(xyz=(0.0, SPIN_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=30.0),
    )

    # ---- backdraft flaps (4x revolute, evenly spaced) ----------------------
    knuckle_mesh = mesh_from_cadquery(
        _tube(KNUCKLE_OUTER_R, KNUCKLE_BORE_R, KNUCKLE_LEN).translate(
            (0.0, 0.0, -KNUCKLE_LEN / 2.0)
        ),
        "flap_knuckle",
    )

    for i in range(FLAP_COUNT):
        theta = i * 2.0 * math.pi / FLAP_COUNT
        sin_t = math.sin(theta)
        cos_t = math.cos(theta)
        hinge_y = HINGE_Y_BASE + i * HINGE_Y_SPACING

        hx = HINGE_R * sin_t
        hz = HINGE_R * cos_t

        flap = model.part(f"flap_{i}")

        # Disc: canonical position (0, 0, -FLAP_DROP) rotated by theta about Y
        dx, dy, dz = _rot_y(0.0, 0.0, -FLAP_DROP, theta)
        flap.visual(
            Cylinder(radius=FLAP_R, length=FLAP_T),
            origin=Origin(xyz=(dx, dy, dz), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=flap_zinc,
            name="flap_disc",
        )

        # Knuckles and connecting tabs
        for j, sx in enumerate((-1.0, 1.0)):
            # Knuckle: canonical (sx*KNUCKLE_X, 0, 0) rotated about Y
            kx, ky, kz = _rot_y(sx * KNUCKLE_X, 0.0, 0.0, theta)
            flap.visual(
                knuckle_mesh,
                origin=Origin(
                    xyz=(kx, ky, kz),
                    rpy=(0.0, math.pi / 2.0 + theta, 0.0),
                ),
                material=flap_zinc,
                name=f"flap_knuckle_{j}",
            )

            # Tab connecting knuckle toward disc
            tx, ty, tz = _rot_y(sx * KNUCKLE_X, 0.0, -0.010, theta)
            flap.visual(
                Box((0.010, 0.004, 0.012)),
                origin=Origin(
                    xyz=(tx, ty, tz),
                    rpy=(0.0, theta, 0.0),
                ),
                material=flap_zinc,
                name=f"flap_tab_{j}",
            )

        # Hinge axis: tangent to drum circle, chosen so positive q opens
        # the flap rearward (-Y direction) via right-hand rule.
        axis = (-cos_t, 0.0, sin_t)

        model.articulation(
            f"flap_hinge_{i}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=flap,
            origin=Origin(xyz=(hx, hinge_y, hz)),
            axis=axis,
            motion_limits=MotionLimits(
                effort=2.0, velocity=4.0, lower=0.0, upper=FLAP_OPEN_LIMIT
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    impeller = object_model.get_part("impeller")
    spin = object_model.get_articulation("impeller_spin")

    flaps = [object_model.get_part(f"flap_{i}") for i in range(FLAP_COUNT)]
    hinges = [object_model.get_articulation(f"flap_hinge_{i}") for i in range(FLAP_COUNT)]

    # ---- intentional overlap allowances -----------------------------------
    # Captured-shaft interference fit for the impeller.
    ctx.allow_overlap(
        impeller,
        housing,
        elem_a="impeller_shaft",
        elem_b="motor_bushing",
        reason="The impeller shaft is pressed into the motor bushing bore so the rotor is physically carried by the spider.",
    )
    # Captured-pin interference fits for all 4 flap hinges.
    for i in range(FLAP_COUNT):
        for knuckle_name in ("flap_knuckle_0", "flap_knuckle_1"):
            ctx.allow_overlap(
                flaps[i],
                housing,
                elem_a=knuckle_name,
                elem_b=f"hinge_pin_{i}",
                reason=f"Flap {i} hinge knuckle is captured around housing hinge pin {i}; the pin carries the hanging flap.",
            )

    # ---- overall envelope: 0.26 m round flange, 0.08 m deep ---------------
    aabb = ctx.part_world_aabb(housing)
    ctx.check("housing_aabb_present", aabb is not None, "Expected a housing AABB.")
    if aabb is None:
        return ctx.report()
    mins, maxs = aabb
    dx = float(maxs[0] - mins[0])
    dy = float(maxs[1] - mins[1])
    dz = float(maxs[2] - mins[2])
    ctx.check("flange_outer_diameter_x", abs(dx - 2.0 * FLANGE_R) <= 0.005, f"dx={dx}")
    ctx.check("flange_outer_diameter_z", abs(dz - 2.0 * FLANGE_R) <= 0.005, f"dz={dz}")
    ctx.check("housing_depth", abs(dy - 0.080) <= 0.004, f"dy={dy}")

    # ---- raised collar and proud bolt heads on the flange face -------------
    collar_aabb = ctx.part_element_world_aabb(housing, elem="collar_ring")
    plate_aabb = ctx.part_element_world_aabb(housing, elem="flange_plate")
    ctx.check(
        "collar_raised_above_flange_face",
        collar_aabb is not None
        and plate_aabb is not None
        and float(collar_aabb[1][1]) >= float(plate_aabb[1][1]) + 0.005,
        f"collar={collar_aabb}, plate={plate_aabb}",
    )
    bolt_visuals = {v.name for v in housing.visuals}
    missing_bolts = [
        f"flange_bolt_{i}" for i in range(BOLT_COUNT) if f"flange_bolt_{i}" not in bolt_visuals
    ]
    ctx.check(
        "eight_hex_bolts_on_flange_rim",
        BOLT_COUNT == 8 and not missing_bolts,
        f"missing={missing_bolts}",
    )
    bolt_aabb = ctx.part_element_world_aabb(housing, elem="flange_bolt_0")
    ctx.check(
        "bolt_head_proud_of_flange_face",
        bolt_aabb is not None and float(bolt_aabb[1][1]) >= 0.002,
        f"bolt_aabb={bolt_aabb}",
    )

    # ---- impeller: 8-blade pinwheel visible inside the collar opening -----
    ctx.check("eight_pinwheel_blades", BLADE_COUNT == 8, f"blade_count={BLADE_COUNT}")
    imp_aabb = ctx.part_world_aabb(impeller)
    ctx.check("impeller_aabb_present", imp_aabb is not None, "Expected impeller AABB.")
    if imp_aabb is None:
        return ctx.report()
    imp_dx = float(imp_aabb[1][0] - imp_aabb[0][0])
    ctx.check(
        "impeller_fits_inside_collar_opening",
        0.145 <= imp_dx <= 2.0 * OPENING_R - 0.002,
        f"imp_dx={imp_dx}",
    )
    ctx.check(
        "impeller_recessed_behind_flange_front",
        -0.030 <= float(imp_aabb[1][1]) <= 0.0,
        f"imp_y_max={imp_aabb[1][1]}",
    )
    ctx.expect_within(impeller, housing, axes="xz", margin=0.0, name="impeller_centered_in_housing")
    ctx.expect_within(impeller, housing, axes="y", margin=0.0, name="impeller_inside_drum_depth")

    # Shaft retained inside motor bushing bore
    ctx.expect_within(
        impeller, housing, axes="xz",
        inner_elem="impeller_shaft", outer_elem="motor_bushing",
        margin=0.0005, name="shaft_centered_in_bushing",
    )
    ctx.expect_overlap(
        impeller, housing, axes="y",
        elem_a="impeller_shaft", elem_b="motor_bushing",
        min_overlap=0.015, name="shaft_inserted_in_bushing",
    )

    # Continuous spin about the horizontal axis normal to the flange
    spin_type = str(getattr(spin, "joint_type", spin.articulation_type)).lower()
    ctx.check("impeller_joint_is_continuous", "continuous" in spin_type, f"type={spin_type}")
    ax = tuple(float(a) for a in spin.axis)
    ctx.check(
        "impeller_axis_is_horizontal_flange_normal",
        abs(ax[0]) < 1e-6 and abs(ax[2]) < 1e-6 and abs(abs(ax[1]) - 1.0) < 1e-6,
        f"axis={ax}",
    )
    with ctx.pose({spin: math.pi / 3.0}):
        rot_aabb = ctx.part_world_aabb(impeller)
        ctx.check(
            "rotated_impeller_keeps_axial_position",
            rot_aabb is not None
            and abs(float(rot_aabb[0][1]) - float(imp_aabb[0][1])) <= 0.002
            and abs(float(rot_aabb[1][1]) - float(imp_aabb[1][1])) <= 0.002,
            f"rot_aabb={rot_aabb}, rest_aabb={imp_aabb}",
        )

    # ---- 4 backdraft flaps: evenly spaced revolute hinges -----------------
    ctx.check("four_backdraft_flaps", FLAP_COUNT == 4, f"flap_count={FLAP_COUNT}")

    # Verify even angular spacing by checking hinge radial positions and
    # angular distribution around the drum axis.
    hinge_angles = []
    for i in range(FLAP_COUNT):
        hinge = hinges[i]
        hpos = hinge.origin.xyz
        h_radial = math.sqrt(float(hpos[0]) ** 2 + float(hpos[2]) ** 2)
        ctx.check(
            f"flap_{i}_hinge_on_drum_wall",
            h_radial >= 0.06,
            f"hinge_radial={h_radial}",
        )
        angle = math.atan2(float(hpos[0]), float(hpos[2]))
        hinge_angles.append(angle)

    # Check angular spacing is approximately 90 degrees between consecutive hinges
    for i in range(FLAP_COUNT):
        expected = i * math.pi / 2.0
        actual = hinge_angles[i] if hinge_angles else 0.0
        # Normalize angle difference
        diff = abs(actual - expected)
        while diff > math.pi:
            diff = abs(diff - 2.0 * math.pi)
        ctx.check(
            f"flap_{i}_even_angular_spacing",
            diff < 0.1,
            f"expected_angle={math.degrees(expected)}, actual={math.degrees(actual)}",
        )

    for i in range(FLAP_COUNT):
        theta = i * 2.0 * math.pi / FLAP_COUNT
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        flap = flaps[i]
        hinge = hinges[i]

        # Motion limits: 0 to ~80 degrees
        lim = hinge.motion_limits
        ctx.check(
            f"flap_{i}_limits_0_to_80deg",
            lim is not None
            and lim.lower is not None
            and lim.upper is not None
            and abs(lim.lower) < 1e-6
            and 1.30 <= lim.upper <= 1.50,
            f"limits={lim}",
        )

        # Hinge axis is tangent to the drum circle (perpendicular to radial)
        h_axis = tuple(float(a) for a in hinge.axis)
        expected_axis = (-cos_t, 0.0, sin_t)
        axis_dot = sum(a * e for a, e in zip(h_axis, expected_axis))
        ctx.check(
            f"flap_{i}_axis_is_tangential",
            abs(axis_dot - 1.0) < 0.05,
            f"axis={h_axis}, expected={expected_axis}",
        )

        # Joint type is revolute
        jtype = str(getattr(hinge, "joint_type", hinge.articulation_type)).lower()
        ctx.check(
            f"flap_{i}_joint_is_revolute",
            "revolute" in jtype,
            f"type={jtype}",
        )

        # Knuckle-pin contact and capture (interference fit)
        ctx.expect_contact(
            flap, housing,
            elem_a="flap_knuckle_0", elem_b=f"hinge_pin_{i}",
            contact_tol=0.003,
            name=f"flap_{i}_knuckle_rides_on_pin",
        )

        # Choose perpendicular axes for pin containment check based on hinge angle
        if abs(cos_t) >= abs(sin_t):
            within_axes = "yz"
        else:
            within_axes = "xy"
        ctx.expect_within(
            housing, flap, axes=within_axes,
            inner_elem=f"hinge_pin_{i}", outer_elem="flap_knuckle_0",
            margin=0.003,
            name=f"flap_{i}_pin_captured_by_knuckle",
        )

        # Closed flap sits inside rear portion of drum
        flap_rest = ctx.part_world_aabb(flap)
        ctx.check(f"flap_{i}_aabb_present", flap_rest is not None, f"Expected flap {i} AABB.")
        if flap_rest is None:
            continue
        ctx.check(
            f"flap_{i}_closed_in_rear_drum_region",
            float(flap_rest[0][1]) >= DRUM_REAR_Y - 0.005
            and float(flap_rest[1][1]) <= -0.040,
            f"flap_y=({flap_rest[0][1]}, {flap_rest[1][1]})",
        )
        ctx.expect_within(
            flap, housing, axes="xz", margin=0.003,
            name=f"flap_{i}_closed_within_drum_bore",
        )

        # Open flap swings rearward (-Y)
        with ctx.pose({hinge: FLAP_OPEN_LIMIT}):
            flap_open = ctx.part_world_aabb(flap)
            ctx.check(
                f"flap_{i}_open_swings_rearward",
                flap_open is not None
                and float(flap_open[0][1]) <= float(flap_rest[0][1]) - 0.02,
                f"open_y_min={flap_open[0][1] if flap_open else None}, rest_y_min={flap_rest[0][1]}",
            )

    return ctx.report()


object_model = build_object_model()
