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

# ---- Backdraft flap (revolute on a top hinge pin) --------------------------
HINGE_Y = -0.060
HINGE_Z = 0.085
PIN_R = 0.003
PIN_LEN = 0.064
BRACKET_X = 0.0285  # bracket centers; boxes span x in [0.025, 0.032]
FLAP_R = 0.080
FLAP_T = 0.0025
FLAP_DROP = 0.085  # disc center sits 0.085 below the pin -> world z = 0
KNUCKLE_X = 0.018
KNUCKLE_OUTER_R = 0.0055
KNUCKLE_BORE_R = 0.0025  # interference fit: knuckles are pressed around the 3 mm pin
KNUCKLE_LEN = 0.010
FLAP_OPEN_LIMIT = 1.40  # ~80 degrees rearward

LAY_FLAT = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))


def _tube(outer_r: float, inner_r: float, depth: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(outer_r).circle(inner_r).extrude(depth)


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
        # Pitch each flat plate about its radial axis so the array reads as a
        # pinwheel of louver-like blades, not a flat disc.
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
                    BOLT_HEAD_H - 0.001,  # head proud of the flange, 1 mm embedded
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

    # Hinge hardware for the backdraft flap: a pin spanning two brackets that
    # drop from the inside of the drum wall near the top, toward the rear.
    for i, sx in enumerate((-1.0, 1.0)):
        housing.visual(
            Box((0.007, 0.006, 0.013)),
            origin=Origin(xyz=(sx * BRACKET_X, HINGE_Y, HINGE_Z + 0.002)),
            material=paint_grey,
            name=f"hinge_bracket_{i}",
        )
    housing.visual(
        Cylinder(radius=PIN_R, length=PIN_LEN),
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bolt_steel,
        name="hinge_pin",
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

    # ---- backdraft flap ----------------------------------------------------
    flap = model.part("backdraft_flap")
    flap.visual(
        Cylinder(radius=FLAP_R, length=FLAP_T),
        origin=Origin(xyz=(0.0, 0.0, -FLAP_DROP), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=flap_zinc,
        name="flap_disc",
    )
    knuckle_mesh = mesh_from_cadquery(
        _tube(KNUCKLE_OUTER_R, KNUCKLE_BORE_R, KNUCKLE_LEN).translate(
            (0.0, 0.0, -KNUCKLE_LEN / 2.0)
        ),
        "flap_knuckle",
    )
    for i, sx in enumerate((-1.0, 1.0)):
        flap.visual(
            knuckle_mesh,
            origin=Origin(xyz=(sx * KNUCKLE_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=flap_zinc,
            name=f"flap_knuckle_{i}",
        )
        flap.visual(
            Box((0.010, 0.004, 0.012)),
            origin=Origin(xyz=(sx * KNUCKLE_X, 0.0, -0.010)),
            material=flap_zinc,
            name=f"flap_tab_{i}",
        )

    model.articulation(
        "flap_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=flap,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        # Axis -X: positive q swings the hanging disc rearward (-Y), i.e. the
        # backdraft flap blows open away from the wall side of the duct.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=FLAP_OPEN_LIMIT),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    impeller = object_model.get_part("impeller")
    flap = object_model.get_part("backdraft_flap")
    spin = object_model.get_articulation("impeller_spin")
    hinge = object_model.get_articulation("flap_hinge")

    # Intentional captured-shaft / captured-pin interference fits. Each one is
    # local, hidden, and paired with exact retention proofs below.
    ctx.allow_overlap(
        impeller,
        housing,
        elem_a="impeller_shaft",
        elem_b="motor_bushing",
        reason="The impeller shaft is pressed into the motor bushing bore so the rotor is physically carried by the spider.",
    )
    for knuckle in ("flap_knuckle_0", "flap_knuckle_1"):
        ctx.allow_overlap(
            flap,
            housing,
            elem_a=knuckle,
            elem_b="hinge_pin",
            reason="The flap hinge knuckle is captured around the housing hinge pin; the pin carries the hanging flap.",
        )

    # --- overall envelope: 0.26 m round flange, 0.08 m deep -----------------
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

    # --- raised collar and proud bolt heads on the flange face --------------
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

    # --- impeller: 8-blade pinwheel visible inside the collar opening -------
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

    # Shaft is retained inside the motor bushing bore (clearance fit).
    ctx.expect_within(
        impeller,
        housing,
        axes="xz",
        inner_elem="impeller_shaft",
        outer_elem="motor_bushing",
        margin=0.0005,
        name="shaft_centered_in_bushing",
    )
    ctx.expect_overlap(
        impeller,
        housing,
        axes="y",
        elem_a="impeller_shaft",
        elem_b="motor_bushing",
        min_overlap=0.015,
        name="shaft_inserted_in_bushing",
    )

    # Continuous spin about the horizontal axis normal to the flange.
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
        # Spinning about world Y must not shift the rotor axially: its depth
        # band inside the drum stays where the hub and shaft put it.
        ctx.check(
            "rotated_impeller_keeps_axial_position",
            rot_aabb is not None
            and abs(float(rot_aabb[0][1]) - float(imp_aabb[0][1])) <= 0.002
            and abs(float(rot_aabb[1][1]) - float(imp_aabb[1][1])) <= 0.002,
            f"rot_aabb={rot_aabb}, rest_aabb={imp_aabb}",
        )

    # --- backdraft flap: hinged across its top edge, rear of the drum -------
    lim = hinge.motion_limits
    ctx.check(
        "flap_limits_0_to_80deg",
        lim is not None
        and lim.lower is not None
        and lim.upper is not None
        and abs(lim.lower) < 1e-6
        and 1.30 <= lim.upper <= 1.50,
        f"limits={lim}",
    )
    ctx.check(
        "flap_hinge_along_top_edge",
        float(hinge.origin.xyz[2]) >= 0.06,
        f"hinge_origin={hinge.origin.xyz}",
    )
    # Knuckles wrap the housing hinge pin with a running clearance.
    ctx.expect_contact(
        flap,
        housing,
        elem_a="flap_knuckle_0",
        elem_b="hinge_pin",
        contact_tol=0.002,
        name="knuckle_rides_on_hinge_pin",
    )
    ctx.expect_within(
        housing,
        flap,
        axes="yz",
        inner_elem="hinge_pin",
        outer_elem="flap_knuckle_0",
        margin=0.001,
        name="hinge_pin_captured_by_knuckle",
    )

    flap_rest = ctx.part_world_aabb(flap)
    ctx.check("flap_aabb_present", flap_rest is not None, "Expected flap AABB.")
    if flap_rest is None:
        return ctx.report()
    ctx.check(
        "closed_flap_sits_inside_rear_of_drum",
        float(flap_rest[0][1]) >= DRUM_REAR_Y and float(flap_rest[1][1]) <= -0.045,
        f"flap_y=({flap_rest[0][1]}, {flap_rest[1][1]})",
    )
    ctx.check(
        "closed_flap_hangs_to_drum_bottom",
        float(flap_rest[0][2]) <= -0.07,
        f"flap_z_min={flap_rest[0][2]}",
    )
    ctx.expect_within(flap, housing, axes="xz", margin=0.0, name="closed_flap_within_drum_bore")

    with ctx.pose({hinge: FLAP_OPEN_LIMIT}):
        flap_open = ctx.part_world_aabb(flap)
        ctx.check(
            "open_flap_swings_rearward_out_of_drum",
            flap_open is not None and float(flap_open[0][1]) <= -0.15,
            f"flap_open={flap_open}",
        )
        ctx.check(
            "open_flap_bottom_edge_lifts",
            flap_open is not None and float(flap_open[0][2]) >= -0.02,
            f"flap_open_z_min={flap_open[0][2] if flap_open else None}",
        )

    return ctx.report()


object_model = build_object_model()
