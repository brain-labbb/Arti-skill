from __future__ import annotations

from math import pi, sqrt

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Small round through-wall exhaust vent, cream plastic.
#
# World frame: the duct axis runs along +X (perpendicular to the wall face).
# x = 0 is the wall plane; the bezel sits in front of it (+X = room side) and
# the duct sleeve runs back through the wall (-X).
# ---------------------------------------------------------------------------

# Front bezel ring (donut-like rounded flange).
BEZEL_RING_RADIUS = 0.0675  # torus centerline radius
BEZEL_TUBE_RADIUS = 0.0125  # torus tube radius -> outer dia 0.16, opening dia 0.11
BEZEL_CENTER_X = 0.0125  # torus center plane; bezel spans x 0.0 .. 0.025

# Duct sleeve (hollow cylindrical tube through the wall).
SLEEVE_OUTER_R = 0.060
SLEEVE_INNER_R = 0.0505
SLEEVE_BACK_X = -0.045
SLEEVE_FRONT_X = 0.014
SLEEVE_LENGTH = SLEEVE_FRONT_X - SLEEVE_BACK_X  # 0.059

# Total depth = bezel front (0.025) - sleeve back (-0.045) = 0.070 m.

# Recessed square-mesh wire grille.
GRILLE_PLANE_X = 0.008  # bar centerline plane (recessed behind bezel front)
GRILLE_RIM_OUTER_R = 0.053
GRILLE_RIM_INNER_R = 0.0475
GRILLE_RIM_DEPTH = 0.004
GRILLE_BAR_SECTION = 0.0016
GRILLE_BAR_END_R = 0.0485  # bar ends embed into the rim, clear of the sleeve bore
GRILLE_BAR_PITCH = 0.0119  # 7 internal bars each way -> roughly 8x8 cells

# Axial fan impeller inside the duct.
FAN_CENTER_X = -0.012
FAN_OUTER_R = 0.046
FAN_HUB_R = 0.0135
FAN_THICKNESS = 0.013
FAN_BLADE_COUNT = 4
HUB_BORE_DIA = 0.0075

# Motor mount (fixed structure inside the sleeve).
STRUT_CENTER_X = -0.031
STRUT_SIZE = (0.005, 0.116, 0.006)  # spans the bore, ends embed in the sleeve wall
POD_RADIUS = 0.011
POD_BACK_X = -0.038
POD_FRONT_X = -0.024
SHAFT_RADIUS = 0.0042  # press-fit inside the hub bore (captured shaft)
SHAFT_BACK_X = -0.026
SHAFT_FRONT_X = -0.006  # tip hidden inside the hub bore

# Rotate a local +Z-axis solid so its axis runs along world +X.
AXIS_TO_X = (0.0, pi / 2.0, 0.0)


def _tube(outer_r: float, inner_r: float, length: float) -> cq.Workplane:
    """Hollow cylinder extruded along local Z from z=0."""
    return cq.Workplane("XY").circle(outer_r).circle(inner_r).extrude(length)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="round_wall_exhaust_vent")

    cream = model.material("cream_plastic", color=(0.91, 0.89, 0.83))
    cream_light = model.material("cream_plastic_light", color=(0.94, 0.92, 0.87))
    wire_metal = model.material("wire_metal", color=(0.80, 0.81, 0.82))
    blade_white = model.material("blade_white_plastic", color=(0.93, 0.93, 0.94))
    motor_dark = model.material("motor_dark_gray", color=(0.24, 0.24, 0.26))
    shaft_steel = model.material("shaft_steel", color=(0.66, 0.66, 0.69))

    # --- duct sleeve (root): hollow tube + motor mount strut/pod/shaft -----
    duct_sleeve = model.part("duct_sleeve")
    duct_sleeve.visual(
        mesh_from_cadquery(
            _tube(SLEEVE_OUTER_R, SLEEVE_INNER_R, SLEEVE_LENGTH), "sleeve_tube"
        ),
        origin=Origin(xyz=(SLEEVE_BACK_X, 0.0, 0.0), rpy=AXIS_TO_X),
        material=cream,
        name="sleeve_tube",
    )
    duct_sleeve.visual(
        Box(STRUT_SIZE),
        origin=Origin(xyz=(STRUT_CENTER_X, 0.0, 0.0)),
        material=cream,
        name="motor_strut",
    )
    duct_sleeve.visual(
        Cylinder(radius=POD_RADIUS, length=POD_FRONT_X - POD_BACK_X),
        origin=Origin(xyz=((POD_BACK_X + POD_FRONT_X) / 2.0, 0.0, 0.0), rpy=AXIS_TO_X),
        material=motor_dark,
        name="motor_pod",
    )
    duct_sleeve.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_FRONT_X - SHAFT_BACK_X),
        origin=Origin(xyz=((SHAFT_BACK_X + SHAFT_FRONT_X) / 2.0, 0.0, 0.0), rpy=AXIS_TO_X),
        material=shaft_steel,
        name="motor_shaft",
    )

    # --- front bezel ring (donut flange against the wall face) ------------
    bezel_ring = model.part("bezel_ring")
    bezel_ring.visual(
        mesh_from_geometry(
            TorusGeometry(
                BEZEL_RING_RADIUS,
                BEZEL_TUBE_RADIUS,
                radial_segments=24,
                tubular_segments=64,
            ),
            "bezel_torus",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=AXIS_TO_X),
        material=cream_light,
        name="bezel_torus",
    )
    model.articulation(
        "sleeve_to_bezel",
        ArticulationType.FIXED,
        parent=duct_sleeve,
        child=bezel_ring,
        origin=Origin(xyz=(BEZEL_CENTER_X, 0.0, 0.0)),
    )

    # --- recessed square-mesh wire grille over the duct mouth -------------
    mesh_grille = model.part("mesh_grille")
    mesh_grille.visual(
        mesh_from_cadquery(
            _tube(GRILLE_RIM_OUTER_R, GRILLE_RIM_INNER_R, GRILLE_RIM_DEPTH),
            "grille_rim",
        ),
        origin=Origin(xyz=(-GRILLE_RIM_DEPTH / 2.0, 0.0, 0.0), rpy=AXIS_TO_X),
        material=wire_metal,
        name="grille_rim",
    )
    for k in range(-3, 4):
        offset = k * GRILLE_BAR_PITCH
        half_len = sqrt(GRILLE_BAR_END_R**2 - offset**2)
        idx = k + 3
        # Horizontal bars run along Y at height z = offset.
        mesh_grille.visual(
            Box((GRILLE_BAR_SECTION, 2.0 * half_len, GRILLE_BAR_SECTION)),
            origin=Origin(xyz=(0.0, 0.0, offset)),
            material=wire_metal,
            name=f"grille_bar_h_{idx}",
        )
        # Vertical bars run along Z at y = offset.
        mesh_grille.visual(
            Box((GRILLE_BAR_SECTION, GRILLE_BAR_SECTION, 2.0 * half_len)),
            origin=Origin(xyz=(0.0, offset, 0.0)),
            material=wire_metal,
            name=f"grille_bar_v_{idx}",
        )
    model.articulation(
        "sleeve_to_grille",
        ArticulationType.FIXED,
        parent=duct_sleeve,
        child=mesh_grille,
        origin=Origin(xyz=(GRILLE_PLANE_X, 0.0, 0.0)),
    )

    # --- fan impeller: hub + 4 broad curved blades, spins about duct axis --
    fan_impeller = model.part("fan_impeller")
    fan_impeller.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                FAN_OUTER_R,
                FAN_HUB_R,
                FAN_BLADE_COUNT,
                thickness=FAN_THICKNESS,
                blade_pitch_deg=26.0,
                blade_sweep_deg=18.0,
                blade=FanRotorBlade(shape="broad", camber=0.12),
                hub=FanRotorHub(style="domed", bore_diameter=HUB_BORE_DIA),
            ),
            "fan_rotor",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=AXIS_TO_X),
        material=blade_white,
        name="fan_rotor",
    )
    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent=duct_sleeve,
        child=fan_impeller,
        origin=Origin(xyz=(FAN_CENTER_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    duct_sleeve = object_model.get_part("duct_sleeve")
    bezel_ring = object_model.get_part("bezel_ring")
    mesh_grille = object_model.get_part("mesh_grille")
    fan_impeller = object_model.get_part("fan_impeller")
    fan_spin = object_model.get_articulation("fan_spin")

    # Intentional seated fits.
    ctx.allow_overlap(
        bezel_ring,
        duct_sleeve,
        elem_a="bezel_torus",
        elem_b="sleeve_tube",
        reason="The donut bezel flange is seated over the duct sleeve front lip.",
    )
    ctx.allow_overlap(
        mesh_grille,
        duct_sleeve,
        elem_a="grille_rim",
        elem_b="sleeve_tube",
        reason="The grille rim is press-fit into the duct mouth wall.",
    )
    ctx.allow_overlap(
        fan_impeller,
        duct_sleeve,
        elem_a="fan_rotor",
        elem_b="motor_shaft",
        reason="The rotor hub bore is press-fit onto the fixed motor shaft (captured axle).",
    )

    # --- overall scale matches the prompt (0.16 m dia, 0.07 m deep) -------
    bezel_aabb = ctx.part_world_aabb(bezel_ring)
    sleeve_aabb = ctx.part_world_aabb(duct_sleeve)
    ctx.check(
        "aabbs_present",
        bezel_aabb is not None and sleeve_aabb is not None,
        "Expected world AABBs for bezel and sleeve.",
    )
    if bezel_aabb is None or sleeve_aabb is None:
        return ctx.report()
    bezel_dy = float(bezel_aabb[1][1] - bezel_aabb[0][1])
    bezel_dz = float(bezel_aabb[1][2] - bezel_aabb[0][2])
    ctx.check(
        "bezel_outer_diameter_0p16",
        abs(bezel_dy - 0.16) < 0.005 and abs(bezel_dz - 0.16) < 0.005,
        details=f"bezel dy={bezel_dy:.4f}, dz={bezel_dz:.4f}",
    )
    total_depth = float(bezel_aabb[1][0] - sleeve_aabb[0][0])
    ctx.check(
        "total_depth_0p07",
        abs(total_depth - 0.07) < 0.006,
        details=f"total depth={total_depth:.4f}",
    )

    # --- bezel is seated on the sleeve front, not floating ----------------
    ctx.expect_contact(
        bezel_ring,
        duct_sleeve,
        elem_a="bezel_torus",
        elem_b="sleeve_tube",
        name="bezel_seated_on_sleeve",
    )
    ctx.expect_overlap(
        bezel_ring,
        duct_sleeve,
        axes="x",
        elem_a="bezel_torus",
        elem_b="sleeve_tube",
        min_overlap=0.005,
        name="bezel_retains_sleeve_front",
    )

    # --- grille covers the duct mouth, recessed inside the bezel ----------
    ctx.expect_within(
        mesh_grille,
        bezel_ring,
        axes="yz",
        name="grille_inside_bezel_opening",
    )
    grille_aabb = ctx.part_world_aabb(mesh_grille)
    ctx.check("grille_aabb_present", grille_aabb is not None)
    if grille_aabb is not None:
        ctx.check(
            "grille_recessed_behind_bezel_front",
            float(grille_aabb[1][0]) < float(bezel_aabb[1][0]) - 0.005,
            details=f"grille max x={grille_aabb[1][0]:.4f}, bezel max x={bezel_aabb[1][0]:.4f}",
        )
    ctx.expect_contact(
        mesh_grille,
        duct_sleeve,
        elem_a="grille_rim",
        elem_b="sleeve_tube",
        name="grille_rim_seated_in_duct_mouth",
    )
    bar_names = [v.name for v in mesh_grille.visuals if v.name and "grille_bar" in v.name]
    ctx.check(
        "grille_has_8x8_square_mesh",
        len(bar_names) == 14,
        details=f"crossing bars={len(bar_names)} (7 horizontal + 7 vertical -> ~8x8 cells)",
    )

    # --- fan sits inside the hollow duct, visible behind the grille -------
    ctx.expect_within(
        fan_impeller,
        duct_sleeve,
        axes="yz",
        name="fan_centered_in_duct_bore",
    )
    fan_aabb = ctx.part_world_aabb(fan_impeller)
    ctx.check("fan_aabb_present", fan_aabb is not None)
    if fan_aabb is not None:
        fan_dy = float(fan_aabb[1][1] - fan_aabb[0][1])
        ctx.check(
            "fan_clears_duct_wall",
            fan_dy <= 2.0 * SLEEVE_INNER_R - 0.004,
            details=f"fan dy={fan_dy:.4f}, duct bore={2.0 * SLEEVE_INNER_R:.4f}",
        )
    ctx.expect_gap(
        mesh_grille,
        fan_impeller,
        axis="x",
        min_gap=0.003,
        max_gap=0.03,
        name="fan_behind_grille_with_clearance",
    )
    # The rotor hub stays threaded on the fixed motor shaft.
    ctx.expect_overlap(
        fan_impeller,
        duct_sleeve,
        axes="x",
        elem_a="fan_rotor",
        elem_b="motor_shaft",
        min_overlap=0.004,
        name="hub_retained_on_motor_shaft",
    )
    ctx.expect_gap(
        fan_impeller,
        duct_sleeve,
        axis="x",
        positive_elem="fan_rotor",
        negative_elem="motor_pod",
        min_gap=0.001,
        name="rotor_clear_of_motor_pod",
    )

    # --- the fan really spins about the duct's horizontal axis ------------
    ctx.check(
        "fan_spin_is_continuous_about_duct_axis",
        fan_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(fan_spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={fan_spin.articulation_type}, axis={fan_spin.axis}",
    )
    if fan_aabb is not None:
        rest_dy = float(fan_aabb[1][1] - fan_aabb[0][1])
        rest_dz = float(fan_aabb[1][2] - fan_aabb[0][2])
        with ctx.pose({fan_spin: pi / 4.0}):
            posed_aabb = ctx.part_world_aabb(fan_impeller)
            ctx.check("posed_fan_aabb_present", posed_aabb is not None)
            if posed_aabb is not None:
                posed_dy = float(posed_aabb[1][1] - posed_aabb[0][1])
                posed_dz = float(posed_aabb[1][2] - posed_aabb[0][2])
                ctx.check(
                    "fan_blades_sweep_when_spun",
                    abs(posed_dy - rest_dy) > 0.0015 or abs(posed_dz - rest_dz) > 0.0015,
                    details=(
                        f"rest dy/dz={rest_dy:.4f}/{rest_dz:.4f}, "
                        f"posed dy/dz={posed_dy:.4f}/{posed_dz:.4f}"
                    ),
                )
                # Blades must clear the duct wall in mid-spin poses too.
                ctx.expect_within(
                    fan_impeller,
                    duct_sleeve,
                    axes="yz",
                    name="spun_fan_stays_inside_duct",
                )

    return ctx.report()


object_model = build_object_model()
