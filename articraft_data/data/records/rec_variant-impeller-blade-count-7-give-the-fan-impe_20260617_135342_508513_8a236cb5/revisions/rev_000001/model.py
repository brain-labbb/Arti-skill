from __future__ import annotations

from math import pi

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Square wall-mounted exhaust ventilation fan, ~0.30 x 0.30 x 0.06 m.
# The wall plane is z=0; the fan axis is +Z (normal to the wall).
# Front face of the frame points toward +Z.
# ---------------------------------------------------------------------------

# Overall envelope
PANEL_SIZE = 0.30
TOTAL_DEPTH = 0.060

# Rear hollow square housing (closed back wall, open toward the front frame)
HOUSING_OUTER = 0.284
HOUSING_WALL = 0.005
BACK_WALL_T = 0.004
HOUSING_DEPTH = 0.046  # z 0.000 .. 0.046

# Front square frame plate with the large circular opening
PLATE_Z0 = HOUSING_DEPTH  # 0.046
PLATE_T = 0.011  # z 0.046 .. 0.057
OPENING_R = 0.132

# Slightly raised square border around the frame perimeter
RIM_Z0 = 0.056  # embeds 1 mm into the plate so the rim reads seated
RIM_Z1 = 0.060
RIM_INNER = 0.270

# Concentric-ring safety grille (in the plane of the opening)
GRILLE_Z0 = 0.050
GRILLE_T = 0.004
CAP_R = 0.040
CAP_T = 0.0055  # cap stands slightly proud of the rings
RING_W = 0.0028
RING_COUNT = 13
RING_R0 = 0.0465
RING_PITCH = 0.00655
SPOKE_COUNT = 6
SPOKE_W = 0.0058
SPOKE_R_IN = 0.030  # tucked under the center cap
SPOKE_R_OUT = 0.137  # embedded into the frame plate past the opening edge

# Motor boss and spindle shaft on the inner back wall
BOSS_R = 0.026
BOSS_Z0 = 0.003  # embeds 1 mm into the back wall
BOSS_Z1 = 0.013
SHAFT_R = 0.0048  # light interference fit inside the hub bore (r=0.0045)
SHAFT_Z0 = 0.012  # embeds 1 mm into the boss
SHAFT_Z1 = 0.0345

# Axial impeller (FanRotorGeometry spins about local Z)
ROTOR_R = 0.120
HUB_R = 0.034
BLADE_COUNT = 7  # variant: seven-blade impeller
ROTOR_T = 0.022
HUB_BORE_D = 0.009  # spindle shaft is pressed into this bore (captured fit)
FRONT_CLEARANCE = 0.0045  # blades stay recessed behind the grille rings


def _build_housing_shell() -> cq.Workplane:
    """Hollow rear box plus the front frame plate with the circular opening."""
    shell = cq.Workplane("XY").box(
        HOUSING_OUTER, HOUSING_OUTER, HOUSING_DEPTH, centered=(True, True, False)
    )
    cavity_side = HOUSING_OUTER - 2.0 * HOUSING_WALL
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=BACK_WALL_T)
        .box(cavity_side, cavity_side, HOUSING_DEPTH, centered=(True, True, False))
    )
    shell = shell.cut(cavity)

    plate = (
        cq.Workplane("XY")
        .workplane(offset=PLATE_Z0)
        .box(PANEL_SIZE, PANEL_SIZE, PLATE_T, centered=(True, True, False))
    )
    opening = (
        cq.Workplane("XY")
        .workplane(offset=PLATE_Z0 - 0.002)
        .circle(OPENING_R)
        .extrude(PLATE_T + 0.004)
    )
    plate = plate.cut(opening)
    return shell.union(plate)


def _build_border_rim() -> cq.Workplane:
    rim = (
        cq.Workplane("XY")
        .workplane(offset=RIM_Z0)
        .box(PANEL_SIZE, PANEL_SIZE, RIM_Z1 - RIM_Z0, centered=(True, True, False))
    )
    rim_cut = (
        cq.Workplane("XY")
        .workplane(offset=RIM_Z0 - 0.002)
        .box(RIM_INNER, RIM_INNER, (RIM_Z1 - RIM_Z0) + 0.004, centered=(True, True, False))
    )
    return rim.cut(rim_cut)


def _build_ring_grille() -> cq.Workplane:
    """Center cap disc + concentric rings + radial spokes, fused as one solid."""
    grille = cq.Workplane("XY").circle(CAP_R).extrude(CAP_T)
    for k in range(RING_COUNT):
        r_mid = RING_R0 + k * RING_PITCH
        ring = (
            cq.Workplane("XY")
            .circle(r_mid + RING_W / 2.0)
            .circle(r_mid - RING_W / 2.0)
            .extrude(GRILLE_T)
        )
        grille = grille.union(ring)
    spoke_len = SPOKE_R_OUT - SPOKE_R_IN
    spoke_cx = (SPOKE_R_OUT + SPOKE_R_IN) / 2.0
    for i in range(SPOKE_COUNT):
        angle_deg = 360.0 * i / SPOKE_COUNT + 30.0
        spoke = (
            cq.Workplane("XY")
            .box(spoke_len, SPOKE_W, GRILLE_T, centered=(True, True, False))
            .translate((spoke_cx, 0.0, 0.0))
            .rotate((0, 0, 0), (0, 0, 1), angle_deg)
        )
        grille = grille.union(spoke)
    return grille.translate((0.0, 0.0, GRILLE_Z0))


def _build_rotor_geometry() -> FanRotorGeometry:
    return FanRotorGeometry(
        ROTOR_R,
        HUB_R,
        BLADE_COUNT,
        thickness=ROTOR_T,
        blade_pitch_deg=32.0,
        blade_sweep_deg=26.0,
        blade=FanRotorBlade(shape="broad", camber=0.18),
        hub=FanRotorHub(
            style="domed",
            bore_diameter=HUB_BORE_D,
            rear_collar_height=0.0035,
            rear_collar_radius=0.024,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_exhaust_fan")

    plastic_white = model.material("plastic_grey_white", rgba=(0.88, 0.88, 0.86, 1.0))
    plastic_blade = model.material("plastic_blade_grey", rgba=(0.80, 0.81, 0.82, 1.0))
    motor_grey = model.material("motor_grey", rgba=(0.45, 0.45, 0.47, 1.0))
    steel = model.material("steel", rgba=(0.66, 0.67, 0.69, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_build_housing_shell(), "housing_shell", tolerance=0.0005),
        material=plastic_white,
        name="housing_shell",
    )
    housing.visual(
        mesh_from_cadquery(_build_border_rim(), "border_rim", tolerance=0.0005),
        material=plastic_white,
        name="border_rim",
    )
    housing.visual(
        mesh_from_cadquery(_build_ring_grille(), "ring_grille", tolerance=0.0004),
        material=plastic_white,
        name="ring_grille",
    )
    housing.visual(
        Cylinder(radius=BOSS_R, length=BOSS_Z1 - BOSS_Z0),
        origin=Origin(xyz=(0.0, 0.0, (BOSS_Z0 + BOSS_Z1) / 2.0)),
        material=motor_grey,
        name="motor_boss",
    )
    housing.visual(
        Cylinder(radius=SHAFT_R, length=SHAFT_Z1 - SHAFT_Z0),
        origin=Origin(xyz=(0.0, 0.0, (SHAFT_Z0 + SHAFT_Z1) / 2.0)),
        material=steel,
        name="motor_shaft",
    )
    housing.inertial = Inertial.from_geometry(
        Box((PANEL_SIZE, PANEL_SIZE, TOTAL_DEPTH)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, TOTAL_DEPTH / 2.0)),
    )

    rotor_geom = _build_rotor_geometry()
    rotor_zmax = max(v[2] for v in rotor_geom.vertices)
    # Lock the rotor front face a fixed clearance behind the grille rings.
    impeller_z = GRILLE_Z0 - FRONT_CLEARANCE - rotor_zmax

    impeller = model.part("impeller")
    impeller.visual(
        mesh_from_geometry(rotor_geom, "impeller_rotor"),
        material=plastic_blade,
        name="impeller_rotor",
    )
    impeller.inertial = Inertial.from_geometry(
        Cylinder(radius=ROTOR_R, length=ROTOR_T + 0.006),
        mass=0.18,
    )

    model.articulation(
        "impeller_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=impeller,
        origin=Origin(xyz=(0.0, 0.0, impeller_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=40.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    impeller = object_model.get_part("impeller")
    spin = object_model.get_articulation("impeller_spin")
    grille = housing.get_visual("ring_grille")
    rim = housing.get_visual("border_rim")
    boss = housing.get_visual("motor_boss")
    shaft = housing.get_visual("motor_shaft")

    # --- Static design claims -------------------------------------------------
    ctx.check(
        "blade_count_is_seven",
        BLADE_COUNT == 7,
        details=f"blade_count={BLADE_COUNT}",
    )
    ctx.check(
        "many_thin_grille_rings",
        RING_COUNT >= 10 and RING_W <= 0.004,
        details=f"ring_count={RING_COUNT}, ring_width={RING_W}",
    )

    # --- Overall envelope ~0.30 x 0.30 x 0.06 ---------------------------------
    aabb = ctx.part_world_aabb(housing)
    ctx.check("housing_aabb_present", aabb is not None, "Expected world AABB for housing.")
    if aabb is None:
        return ctx.report()
    mins, maxs = aabb
    dx = float(maxs[0] - mins[0])
    dy = float(maxs[1] - mins[1])
    dz = float(maxs[2] - mins[2])
    ctx.check("panel_width_0p30", abs(dx - PANEL_SIZE) <= 0.004, details=f"dx={dx}")
    ctx.check("panel_height_0p30", abs(dy - PANEL_SIZE) <= 0.004, details=f"dy={dy}")
    ctx.check("panel_depth_about_0p06", abs(dz - TOTAL_DEPTH) <= 0.005, details=f"dz={dz}")

    # --- Raised border rim sits proud of the frame face ------------------------
    rim_aabb = ctx.part_element_world_aabb(housing, elem="border_rim")
    shell_aabb = ctx.part_element_world_aabb(housing, elem="housing_shell")
    ctx.check(
        "rim_and_shell_aabbs_present",
        rim_aabb is not None and shell_aabb is not None,
        "Expected AABBs for border_rim and housing_shell.",
    )
    if rim_aabb is not None and shell_aabb is not None:
        ctx.check(
            "border_rim_raised_above_frame_face",
            float(rim_aabb[1][2]) >= float(shell_aabb[1][2]) + 0.002,
            details=f"rim_zmax={rim_aabb[1][2]}, shell_zmax={shell_aabb[1][2]}",
        )

    # --- Concentric ring grille spans the opening, cap proud of rings ----------
    g_aabb = ctx.part_element_world_aabb(housing, elem="ring_grille")
    ctx.check("grille_aabb_present", g_aabb is not None, "Expected AABB for ring_grille.")
    if g_aabb is not None:
        g_dx = float(g_aabb[1][0] - g_aabb[0][0])
        g_dy = float(g_aabb[1][1] - g_aabb[0][1])
        g_dz = float(g_aabb[1][2] - g_aabb[0][2])
        # The outermost ring sets the round envelope (spokes sit at 30 deg
        # offsets, so the AABB is bounded by ring or spoke radius per axis).
        outer_ring_d = 2.0 * (RING_R0 + (RING_COUNT - 1) * RING_PITCH + RING_W / 2.0)
        ctx.check(
            "grille_spans_circular_opening",
            outer_ring_d - 0.004 <= g_dx <= 2.0 * SPOKE_R_OUT + 0.004
            and outer_ring_d - 0.004 <= g_dy <= 2.0 * SPOKE_R_OUT + 0.004,
            details=f"grille_dx={g_dx}, grille_dy={g_dy}, outer_ring_d={outer_ring_d}",
        )
        ctx.check(
            "grille_is_thin_flat_lattice_with_proud_cap",
            0.0045 <= g_dz <= 0.008,
            details=f"grille_dz={g_dz}",
        )

    # --- Impeller size and placement -------------------------------------------
    i_aabb = ctx.part_world_aabb(impeller)
    ctx.check("impeller_aabb_present", i_aabb is not None, "Expected AABB for impeller.")
    if i_aabb is None:
        return ctx.report()
    i_dx = float(i_aabb[1][0] - i_aabb[0][0])
    ctx.check(
        "impeller_diameter_fills_opening",
        abs(i_dx - 2.0 * ROTOR_R) <= 0.012 and i_dx < 2.0 * OPENING_R,
        details=f"impeller_dx={i_dx}, opening_d={2.0 * OPENING_R}",
    )

    # Impeller is recessed: blades stay behind the grille rings, never poking out.
    ctx.expect_gap(
        housing,
        impeller,
        axis="z",
        positive_elem=grille,
        min_gap=0.002,
        max_gap=0.012,
        name="impeller_recessed_behind_grille",
    )
    # Impeller clears the motor boss behind it.
    ctx.expect_gap(
        impeller,
        housing,
        axis="z",
        negative_elem=boss,
        min_gap=0.0005,
        max_gap=0.025,
        name="impeller_clears_motor_boss",
    )
    # Impeller stays inside the housing footprint (and the circular opening zone).
    ctx.expect_within(impeller, housing, axes="xy", name="impeller_inside_housing_footprint")

    # Spindle shaft is intentionally pressed into the hub bore so the rotor is
    # physically carried by the motor spindle (captured shaft pattern).
    ctx.allow_overlap(
        impeller,
        housing,
        elem_a="impeller_rotor",
        elem_b="motor_shaft",
        reason=(
            "The motor spindle shaft is intentionally captured inside the impeller "
            "hub bore (light interference fit) so the rotor is supported by the "
            "motor instead of floating."
        ),
    )
    ctx.expect_contact(
        impeller,
        housing,
        elem_a="impeller_rotor",
        elem_b=shaft,
        name="hub_bore_rides_on_motor_shaft",
    )
    # Spindle shaft nests inside the hub bore: centered and axially inserted.
    ctx.expect_within(
        housing,
        impeller,
        axes="xy",
        inner_elem=shaft,
        name="shaft_centered_in_hub_bore",
    )
    ctx.expect_overlap(
        impeller,
        housing,
        axes="z",
        elem_b=shaft,
        min_overlap=0.008,
        name="shaft_inserted_into_hub",
    )

    # --- Articulation: one continuous spin about the wall-normal (Z) axis ------
    ctx.check(
        "spin_is_continuous",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={spin.articulation_type}",
    )
    ax = spin.axis
    ctx.check(
        "spin_axis_normal_to_wall",
        abs(ax[0]) < 1e-9 and abs(ax[1]) < 1e-9 and abs(ax[2] - 1.0) < 1e-9,
        details=f"axis={ax}",
    )
    limits = spin.motion_limits
    ctx.check(
        "spin_unlimited_rotation",
        limits is not None and limits.lower is None and limits.upper is None,
        details=f"limits={limits}",
    )

    # Posed mid-spin, the rotor still hides behind the grille and inside the frame.
    with ctx.pose({spin: pi / 2.0}):
        ctx.expect_gap(
            housing,
            impeller,
            axis="z",
            positive_elem=grille,
            min_gap=0.002,
            max_gap=0.012,
            name="rotated_impeller_still_recessed",
        )
        ctx.expect_within(
            impeller,
            housing,
            axes="xy",
            name="rotated_impeller_stays_in_opening",
        )

    return ctx.report()


object_model = build_object_model()
