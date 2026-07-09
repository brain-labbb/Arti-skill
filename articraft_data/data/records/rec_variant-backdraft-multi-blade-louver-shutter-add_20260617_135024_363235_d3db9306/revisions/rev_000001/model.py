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
# Variant: backdraft louver shutter at the rear.
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
BLADE_COUNT = 8  # prompt asks for roughly 7-9 wide swept blades
ROTOR_T = 0.022
HUB_BORE_D = 0.009  # spindle shaft is pressed into this bore (captured fit)
FRONT_CLEARANCE = 0.0045  # blades stay recessed behind the grille rings

# ---------------------------------------------------------------------------
# Backdraft louver shutter (rear of housing, behind the back wall)
# ---------------------------------------------------------------------------
LOUVER_COUNT = 5
LOUVER_FRAME_OUTER = 0.260
LOUVER_FRAME_BORDER = 0.008
LOUVER_OPENING_SIDE = LOUVER_FRAME_OUTER - 2.0 * LOUVER_FRAME_BORDER  # 0.244
LOUVER_FRAME_T = 0.005
LOUVER_FRAME_Z0 = -LOUVER_FRAME_T  # frame from z=-0.005 to z=0
LOUVER_BLADE_W = LOUVER_OPENING_SIDE - 0.006  # 0.238, side clearance
LOUVER_BLADE_SLOT_H = LOUVER_OPENING_SIDE / LOUVER_COUNT  # ~0.0488
LOUVER_BLADE_H = LOUVER_BLADE_SLOT_H  # fills slot exactly
LOUVER_BLADE_T = 0.002
LOUVER_PIVOT_Z = -0.002  # pivot line behind back wall face
LOUVER_OPEN_ANGLE = 1.22  # ~70 degrees open


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


def _build_louver_frame() -> cq.Workplane:
    """Rectangular border frame with square opening for louver blades."""
    frame = (
        cq.Workplane("XY")
        .box(LOUVER_FRAME_OUTER, LOUVER_FRAME_OUTER, LOUVER_FRAME_T, centered=(True, True, False))
        .translate((0.0, 0.0, LOUVER_FRAME_Z0))
    )
    inner_cut = (
        cq.Workplane("XY")
        .box(
            LOUVER_OPENING_SIDE,
            LOUVER_OPENING_SIDE,
            LOUVER_FRAME_T + 0.002,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, LOUVER_FRAME_Z0 - 0.001))
    )
    frame = frame.cut(inner_cut)

    # Thin horizontal pivot rails between blade slots (narrow bars at each
    # blade junction, acting as pivot bearing supports in the frame sides).
    rail_h = 0.0025
    rail_depth = LOUVER_FRAME_T * 0.55
    for i in range(1, LOUVER_COUNT):
        y_pos = LOUVER_OPENING_SIDE / 2.0 - i * LOUVER_BLADE_SLOT_H
        rail = (
            cq.Workplane("XY")
            .box(LOUVER_OPENING_SIDE, rail_h, rail_depth, centered=(True, True, False))
            .translate((0.0, y_pos, LOUVER_FRAME_Z0 + LOUVER_FRAME_T - rail_depth / 2.0))
        )
        frame = frame.union(rail)
    return frame


def _build_louver_blade() -> cq.Workplane:
    """Flat louver blade plate. Pivot at origin (top edge), blade hangs in -Y."""
    return (
        cq.Workplane("XY")
        .box(LOUVER_BLADE_W, LOUVER_BLADE_H, LOUVER_BLADE_T)
        .translate((0.0, -LOUVER_BLADE_H / 2.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_exhaust_fan")

    plastic_white = model.material("plastic_grey_white", rgba=(0.88, 0.88, 0.86, 1.0))
    plastic_blade = model.material("plastic_blade_grey", rgba=(0.80, 0.81, 0.82, 1.0))
    motor_grey = model.material("motor_grey", rgba=(0.45, 0.45, 0.47, 1.0))
    steel = model.material("steel", rgba=(0.66, 0.67, 0.69, 1.0))
    louver_white = model.material("louver_white", rgba=(0.90, 0.90, 0.88, 1.0))

    # ── Housing (root) ──────────────────────────────────────────────────────
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
    # Louver frame is a non-moving decoration inline on the housing.
    housing.visual(
        mesh_from_cadquery(_build_louver_frame(), "louver_frame", tolerance=0.0004),
        material=plastic_white,
        name="louver_frame",
    )
    housing.inertial = Inertial.from_geometry(
        Box((PANEL_SIZE, PANEL_SIZE, TOTAL_DEPTH)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, TOTAL_DEPTH / 2.0)),
    )

    # ── Impeller ────────────────────────────────────────────────────────────
    rotor_geom = _build_rotor_geometry()
    rotor_zmax = max(v[2] for v in rotor_geom.vertices)
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

    # ── Backdraft louver shutter blades ─────────────────────────────────────
    for i in range(LOUVER_COUNT):
        blade = model.part(f"shutter_blade_{i}")
        blade.visual(
            mesh_from_cadquery(
                _build_louver_blade(), f"shutter_blade_{i}", tolerance=0.0003
            ),
            material=louver_white,
            name=f"shutter_blade_{i}",
        )
        blade.inertial = Inertial.from_geometry(
            Box((LOUVER_BLADE_W, LOUVER_BLADE_T, LOUVER_BLADE_H)),
            mass=0.012,
            origin=Origin(xyz=(0.0, -LOUVER_BLADE_H / 2.0, 0.0)),
        )

        # Each blade pivots about a horizontal X-axis at its top edge.
        # At q=0 the blade hangs vertically (closed). Positive q swings
        # the bottom edge toward -Z (open, allowing exhaust airflow).
        pivot_y = LOUVER_OPENING_SIDE / 2.0 - i * LOUVER_BLADE_SLOT_H
        model.articulation(
            f"shutter_blade_{i}_hinge",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=blade,
            origin=Origin(xyz=(0.0, pivot_y, LOUVER_PIVOT_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.3, velocity=0.8, lower=0.0, upper=LOUVER_OPEN_ANGLE
            ),
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
    louver_frame_vis = housing.get_visual("louver_frame")

    # --- Static design claims -------------------------------------------------
    ctx.check(
        "blade_count_seven_to_nine",
        7 <= BLADE_COUNT <= 9,
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
    ctx.check("panel_depth_about_0p06", abs(dz - TOTAL_DEPTH) <= 0.012, details=f"dz={dz}")

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

    ctx.expect_gap(
        housing,
        impeller,
        axis="z",
        positive_elem=grille,
        min_gap=0.002,
        max_gap=0.012,
        name="impeller_recessed_behind_grille",
    )
    ctx.expect_gap(
        impeller,
        housing,
        axis="z",
        negative_elem=boss,
        min_gap=0.0005,
        max_gap=0.025,
        name="impeller_clears_motor_boss",
    )
    ctx.expect_within(impeller, housing, axes="xy", name="impeller_inside_housing_footprint")

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

    # --- Impeller spin articulation -------------------------------------------
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

    # --- Louver frame visual exists on housing --------------------------------
    ctx.check(
        "louver_frame_visual_exists",
        louver_frame_vis is not None,
        "Housing should have an inline louver_frame visual.",
    )

    # --- Backdraft louver shutter: all blades present and articulated ---------
    ctx.check(
        "louver_blade_count_is_five",
        LOUVER_COUNT == 5,
        details=f"LOUVER_COUNT={LOUVER_COUNT}",
    )

    shutter_hinges = []
    shutter_blades = []
    for i in range(LOUVER_COUNT):
        b = object_model.get_part(f"shutter_blade_{i}")
        h = object_model.get_articulation(f"shutter_blade_{i}_hinge")
        shutter_blades.append(b)
        shutter_hinges.append(h)

    # Each hinge is REVOLUTE about horizontal X-axis.
    for i, h in enumerate(shutter_hinges):
        ctx.check(
            f"shutter_blade_{i}_hinge_is_revolute",
            h.articulation_type == ArticulationType.REVOLUTE,
            details=f"type={h.articulation_type}",
        )
        hax = h.axis
        ctx.check(
            f"shutter_blade_{i}_hinge_axis_horizontal_x",
            abs(hax[0] - 1.0) < 1e-9 and abs(hax[1]) < 1e-9 and abs(hax[2]) < 1e-9,
            details=f"axis={hax}",
        )
        hlim = h.motion_limits
        ctx.check(
            f"shutter_blade_{i}_hinge_has_limits",
            hlim is not None and hlim.lower is not None and hlim.upper is not None,
            details=f"limits={hlim}",
        )
        if hlim is not None and hlim.lower is not None and hlim.upper is not None:
            ctx.check(
                f"shutter_blade_{i}_hinge_opens_outward",
                hlim.lower >= 0.0 and hlim.upper > 0.8,
                details=f"lower={hlim.lower}, upper={hlim.upper}",
            )

    # --- Even vertical spacing of shutter blades ------------------------------
    pivot_ys = []
    for h in shutter_hinges:
        pivot_ys.append(h.origin.xyz[1])
    if len(pivot_ys) >= 2:
        spacings = [pivot_ys[i] - pivot_ys[i + 1] for i in range(len(pivot_ys) - 1)]
        mean_spacing = sum(spacings) / len(spacings)
        max_deviation = max(abs(s - mean_spacing) for s in spacings)
        ctx.check(
            "shutter_blades_evenly_spaced_vertically",
            max_deviation < 0.001,
            details=f"spacings={spacings}, max_dev={max_deviation}",
        )

    # --- At rest (q=0), all blades hang closed inside the frame opening -------
    for i, b in enumerate(shutter_blades):
        ctx.expect_within(
            b,
            housing,
            axes="xy",
            name=f"shutter_blade_{i}_inside_frame_opening_at_rest",
        )

    # --- Louver frame is behind the housing (z < grille) ---------------------
    lf_aabb = ctx.part_element_world_aabb(housing, elem="louver_frame")
    ctx.check("louver_frame_aabb_present", lf_aabb is not None, "Expected AABB for louver_frame.")
    if lf_aabb is not None and g_aabb is not None:
        ctx.check(
            "louver_frame_behind_grille",
            float(lf_aabb[1][2]) < float(g_aabb[0][2]),
            details=f"frame_zmax={lf_aabb[1][2]}, grille_zmin={g_aabb[0][2]}",
        )

    # --- Posed: blades swing open (bottom edge moves toward -Z) --------------
    mid_open = LOUVER_OPEN_ANGLE * 0.6
    pose_dict = {h: mid_open for h in shutter_hinges}
    with ctx.pose(pose_dict):
        for i, b in enumerate(shutter_blades):
            b_aabb = ctx.part_world_aabb(b)
            if b_aabb is not None:
                # When open, the blade bottom should be at a lower Z than its
                # pivot (blade has swung outward toward -Z).
                pivot_z = shutter_hinges[i].origin.xyz[2]
                blade_zmin = float(b_aabb[0][2])
                ctx.check(
                    f"shutter_blade_{i}_swings_outward_when_open",
                    blade_zmin < pivot_z - 0.010,
                    details=f"pivot_z={pivot_z}, blade_zmin={blade_zmin}",
                )

    return ctx.report()


object_model = build_object_model()
