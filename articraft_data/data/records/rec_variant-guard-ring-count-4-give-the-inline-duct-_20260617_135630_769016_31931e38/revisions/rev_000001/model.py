from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoxGeometry,
    CylinderGeometry,
    DomeGeometry,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Industrial inline duct fan
#
# World layout: the duct tube lies with its axis horizontal along world +X,
# resting on the ground plane (z = 0). The visible "front" opening with the
# wire guard faces +X. The guard, motor mount, and impeller frames are all
# rotated so their local +Z is the duct axis (= world +X), which lets the
# axial helpers (FanRotorGeometry, TorusGeometry, lathes) be authored in
# their natural axial frame.
# ---------------------------------------------------------------------------

TUBE_LENGTH = 0.280          # m, along the duct axis
TUBE_OUTER_R = 0.160         # m, main shell outer radius (~0.32 m diameter)
TUBE_INNER_R = 0.1545        # m, bore radius (sheet-steel wall)
RIM_R = 0.1655               # m, slightly rolled rim bulge at both ends
AXIS_Z = RIM_R               # duct axis height so the rim rests on z = 0

HALF_LEN = TUBE_LENGTH / 2.0

# Front wire guard (spider / target pattern), authored in a local frame whose
# +Z is the duct axis. The guard dishes rearward into the tube.
GUARD_PLANE_X = 0.137        # world x of the guard's outer-ring plane
GUARD_DISH_DEPTH = 0.050     # axial depth from outer ring back to hub plate
GUARD_WIRE_R = 0.0022        # spoke wire radius
GUARD_RING_TUBE_R = 0.0028   # ring wire radius
GUARD_RING_RADII = (0.1535, 0.115, 0.075, 0.040)
GUARD_SPOKE_COUNT = 8
GUARD_HUB_PLATE_R = 0.020
GUARD_HUB_PLATE_T = 0.004
SPOKE_R0 = 0.012             # spoke inner radius (embedded in hub plate)
SPOKE_R1 = 0.152             # spoke outer radius (meets outer ring)

# Impeller / motor
IMPELLER_X = 0.050           # world x of the rotor midplane
IMPELLER_OUTER_R = 0.150     # blade tip radius (nearly fills the bore)
IMPELLER_HUB_R = 0.042
IMPELLER_THICKNESS = 0.042
MOTOR_X = -0.035             # world x of the motor can center
MOTOR_R = 0.045
MOTOR_LEN = 0.100
STRUT_COUNT = 3
STRUT_R_IN = 0.041           # strut inner end (embedded in motor can)
STRUT_R_OUT = 0.158          # strut outer end (welded into the duct wall)


def _guard_cone_z(r: float) -> float:
    """Axial position (guard-local z) of the dished spoke cone at radius r."""
    slope = (GUARD_DISH_DEPTH - 0.002) / (SPOKE_R1 - SPOKE_R0)
    return -GUARD_DISH_DEPTH + slope * (r - SPOKE_R0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_inline_duct_fan")

    steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    guard_steel = model.material("guard_steel", rgba=(0.60, 0.61, 0.63, 1.0))
    blade_dark = model.material("blade_steel_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    motor_dark = model.material("motor_graphite", rgba=(0.26, 0.27, 0.29, 1.0))

    # ------------------------------------------------------------------
    # Duct shell: hollow open-ended tube with slightly rolled rims.
    # Lathe is authored about local +Z, then the visual is pitched 90 deg
    # so the tube axis lies along world +X.
    # ------------------------------------------------------------------
    shell = model.part("duct_shell")
    outer_profile = [
        (RIM_R - 0.0005, -HALF_LEN),
        (RIM_R, -HALF_LEN + 0.006),
        (TUBE_OUTER_R, -HALF_LEN + 0.014),
        (TUBE_OUTER_R, HALF_LEN - 0.014),
        (RIM_R, HALF_LEN - 0.006),
        (RIM_R - 0.0005, HALF_LEN),
    ]
    inner_profile = [
        (TUBE_INNER_R, -HALF_LEN),
        (TUBE_INNER_R, -HALF_LEN + 0.006),
        (TUBE_INNER_R, -HALF_LEN + 0.014),
        (TUBE_INNER_R, HALF_LEN - 0.014),
        (TUBE_INNER_R, HALF_LEN - 0.006),
        (TUBE_INNER_R, HALF_LEN),
    ]
    shell_geo = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=96,
        start_cap="flat",
        end_cap="flat",
    )
    shell.visual(
        mesh_from_geometry(shell_geo, "duct_shell_tube"),
        origin=Origin(xyz=(0.0, 0.0, AXIS_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="shell_tube",
    )

    # ------------------------------------------------------------------
    # Front wire guard: 4 concentric rings + 8 wire spokes on a shallow
    # dished cone converging onto a small central hub plate.
    # Guard-local +Z = duct axis (world +X); local z=0 is the outer-ring
    # plane near the front rim, the dish extends to local z = -depth.
    # ------------------------------------------------------------------
    guard = model.part("front_guard")

    for i, ring_r in enumerate(GUARD_RING_RADII):
        ring = TorusGeometry(
            ring_r,
            GUARD_RING_TUBE_R,
            radial_segments=12,
            tubular_segments=72,
        )
        ring.translate(0.0, 0.0, _guard_cone_z(ring_r))
        guard.visual(
            mesh_from_geometry(ring, f"guard_ring_{i}"),
            material=guard_steel,
            name=f"guard_ring_{i}",
        )

    spoke_dr = SPOKE_R1 - SPOKE_R0
    spoke_dz = _guard_cone_z(SPOKE_R1) - _guard_cone_z(SPOKE_R0)
    spoke_len = math.hypot(spoke_dr, spoke_dz)
    spoke_tilt = math.atan2(spoke_dr, spoke_dz)  # rotate_y angle: +Z toward +X
    spoke_mid_r = (SPOKE_R0 + SPOKE_R1) / 2.0
    spoke_mid_z = (_guard_cone_z(SPOKE_R0) + _guard_cone_z(SPOKE_R1)) / 2.0

    spokes_geo = None
    for k in range(GUARD_SPOKE_COUNT):
        spoke = CylinderGeometry(GUARD_WIRE_R, spoke_len, radial_segments=12)
        spoke.rotate_y(spoke_tilt)
        spoke.translate(spoke_mid_r, 0.0, spoke_mid_z)
        spoke.rotate_z(2.0 * math.pi * k / GUARD_SPOKE_COUNT)
        spokes_geo = spoke if spokes_geo is None else spokes_geo.merge(spoke)
    guard.visual(
        mesh_from_geometry(spokes_geo, "guard_spokes"),
        material=guard_steel,
        name="guard_spokes",
    )

    hub_plate = CylinderGeometry(GUARD_HUB_PLATE_R, GUARD_HUB_PLATE_T, radial_segments=32)
    hub_plate.translate(0.0, 0.0, -GUARD_DISH_DEPTH)
    guard.visual(
        mesh_from_geometry(hub_plate, "guard_hub_plate"),
        material=guard_steel,
        name="guard_hub_plate",
    )

    model.articulation(
        "shell_to_guard",
        ArticulationType.FIXED,
        parent=shell,
        child=guard,
        origin=Origin(
            xyz=(GUARD_PLANE_X, 0.0, AXIS_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
    )

    # ------------------------------------------------------------------
    # Motor mount: dark motor can with a rear dome, three radial struts
    # welded into the duct wall, and a front shaft boss that carries the
    # impeller hub. Local +Z = duct axis (world +X).
    # ------------------------------------------------------------------
    mount = model.part("motor_mount")

    motor_geo = CylinderGeometry(MOTOR_R, MOTOR_LEN, radial_segments=48)
    rear_dome = DomeGeometry(MOTOR_R, radial_segments=48, height_segments=10)
    rear_dome.scale(1.0, 1.0, 0.35)
    rear_dome.rotate_x(math.pi)
    rear_dome.translate(0.0, 0.0, -MOTOR_LEN / 2.0)
    motor_geo.merge(rear_dome)
    mount.visual(
        mesh_from_geometry(motor_geo, "motor_body"),
        material=motor_dark,
        name="motor_body",
    )

    strut_len = STRUT_R_OUT - STRUT_R_IN
    for k in range(STRUT_COUNT):
        strut = BoxGeometry((strut_len, 0.007, 0.026))
        strut.translate(STRUT_R_IN + strut_len / 2.0, 0.0, 0.0)
        strut.rotate_z(2.0 * math.pi * k / STRUT_COUNT + math.pi / 6.0)
        mount.visual(
            mesh_from_geometry(strut, f"motor_strut_{k}"),
            material=motor_dark,
            name=f"motor_strut_{k}",
        )

    shaft_len = 0.030
    shaft = CylinderGeometry(0.010, shaft_len, radial_segments=24)
    shaft.translate(0.0, 0.0, MOTOR_LEN / 2.0 + shaft_len / 2.0 - 0.002)
    mount.visual(
        mesh_from_geometry(shaft, "motor_shaft"),
        material=motor_dark,
        name="motor_shaft",
    )

    model.articulation(
        "shell_to_motor_mount",
        ArticulationType.FIXED,
        parent=shell,
        child=mount,
        origin=Origin(
            xyz=(MOTOR_X, 0.0, AXIS_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
    )

    # ------------------------------------------------------------------
    # Impeller: 7 dark curved sheet-metal blades on a central hub, nearly
    # filling the bore, spinning continuously about the duct axis.
    # The joint frame inherits the mount's rotated orientation, so the
    # rotor's natural local +Z spin axis is already the duct axis.
    # ------------------------------------------------------------------
    impeller = model.part("impeller")
    rotor_geo = FanRotorGeometry(
        IMPELLER_OUTER_R,
        IMPELLER_HUB_R,
        7,
        thickness=IMPELLER_THICKNESS,
        blade_pitch_deg=32.0,
        blade_sweep_deg=24.0,
        # tip_clearance trims the swept/cambered tips back so the true blade
        # envelope (~0.1498 m) clears the 0.1545 m bore.
        blade=FanRotorBlade(
            shape="scimitar",
            camber=0.18,
            tip_pitch_deg=16.0,
            tip_clearance=0.006,
        ),
        hub=FanRotorHub(
            style="capped",
            rear_collar_height=0.012,
            rear_collar_radius=0.020,
        ),
    )
    impeller.visual(
        mesh_from_geometry(rotor_geo, "impeller_rotor"),
        material=blade_dark,
        name="impeller_rotor",
    )

    model.articulation(
        "impeller_spin",
        ArticulationType.CONTINUOUS,
        parent=mount,
        child=impeller,
        origin=Origin(xyz=(0.0, 0.0, IMPELLER_X - MOTOR_X)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=80.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shell = object_model.get_part("duct_shell")
    guard = object_model.get_part("front_guard")
    mount = object_model.get_part("motor_mount")
    impeller = object_model.get_part("impeller")
    spin = object_model.get_articulation("impeller_spin")

    # ------------------------------------------------------------------
    # Intentional, scoped overlaps that represent real construction.
    # ------------------------------------------------------------------
    ctx.allow_overlap(
        guard,
        shell,
        elem_a="guard_ring_0",
        elem_b="shell_tube",
        reason="The guard's outer ring clips into the duct bore at the rolled front rim.",
    )
    ctx.allow_overlap(
        guard,
        shell,
        elem_a="guard_spokes",
        elem_b="shell_tube",
        reason="Spoke wire tips seat against the duct bore beside the outer ring.",
    )
    for k in range(STRUT_COUNT):
        ctx.allow_overlap(
            mount,
            shell,
            elem_a=f"motor_strut_{k}",
            elem_b="shell_tube",
            reason="Motor mount struts are welded into the duct wall.",
        )
    ctx.allow_overlap(
        mount,
        impeller,
        elem_a="motor_shaft",
        elem_b="impeller_rotor",
        reason="The motor shaft is captured inside the impeller hub collar.",
    )

    # ------------------------------------------------------------------
    # Overall envelope: ~0.32 m diameter tube, ~0.28 m long, lying with
    # its axis horizontal (along x) and resting on the ground.
    # ------------------------------------------------------------------
    shell_aabb = ctx.part_world_aabb(shell)
    ctx.check("shell_aabb_present", shell_aabb is not None, "Expected duct shell AABB.")
    if shell_aabb is None:
        return ctx.report()
    smin, smax = shell_aabb
    ssize = tuple(float(smax[i] - smin[i]) for i in range(3))
    ctx.check(
        "tube_length_along_x",
        0.270 <= ssize[0] <= 0.292,
        f"Tube axial length should be ~0.28 m, got {ssize!r}",
    )
    ctx.check(
        "tube_diameter",
        0.315 <= ssize[1] <= 0.345 and 0.315 <= ssize[2] <= 0.345,
        f"Tube diameter should be ~0.32-0.33 m, got {ssize!r}",
    )
    ctx.check(
        "tube_rests_on_ground",
        abs(float(smin[2])) <= 0.004,
        f"Rolled rim should rest on z=0, got min z={float(smin[2]):.4f}",
    )

    # Hollow end-to-end: the bore stays open, so the impeller fits entirely
    # inside the tube's radial cross-section and is visible through the guard.
    ctx.expect_within(
        impeller,
        shell,
        axes="yz",
        margin=0.0,
        name="impeller_inside_bore",
    )
    ctx.expect_within(
        guard,
        shell,
        axes="yz",
        margin=0.0,
        name="guard_inside_front_rim",
    )

    # ------------------------------------------------------------------
    # Wire guard: rings + spokes + hub plate present, seated on the shell,
    # in front of the impeller.
    # ------------------------------------------------------------------
    for i in range(len(GUARD_RING_RADII)):
        ring = guard.get_visual(f"guard_ring_{i}")
        ctx.check(f"guard_ring_{i}_present", ring is not None, f"Missing guard ring {i}.")
    ctx.check(
        "guard_spokes_present",
        guard.get_visual("guard_spokes") is not None,
        "Missing guard spokes.",
    )
    hub_aabb = ctx.part_element_world_aabb(guard, elem="guard_hub_plate")
    ctx.check("guard_hub_plate_present", hub_aabb is not None, "Missing guard hub plate.")
    if hub_aabb is not None:
        hmin, hmax = hub_aabb
        hub_dia = float(hmax[1] - hmin[1])
        ctx.check(
            "guard_hub_plate_small_and_centered",
            hub_dia <= 0.05
            and abs(float(hmin[1] + hmax[1]) / 2.0) <= 0.003
            and abs(float(hmin[2] + hmax[2]) / 2.0 - AXIS_Z) <= 0.003,
            f"Hub plate should be a small centered disc, aabb={hub_aabb!r}",
        )

    ctx.expect_contact(
        guard,
        shell,
        elem_a="guard_ring_0",
        elem_b="shell_tube",
        name="guard_outer_ring_seats_in_rim",
    )

    # Guard covers the front opening: it sits at the +x end of the tube.
    guard_aabb = ctx.part_world_aabb(guard)
    ctx.check("guard_aabb_present", guard_aabb is not None, "Expected guard AABB.")
    if guard_aabb is not None:
        gmin, gmax = guard_aabb
        ctx.check(
            "guard_at_front_opening",
            float(gmax[0]) >= HALF_LEN - 0.012 and float(gmin[0]) >= 0.05,
            f"Guard should occupy the front (+x) opening, aabb={guard_aabb!r}",
        )
        guard_dia = float(gmax[1] - gmin[1])
        ctx.check(
            "guard_spans_opening",
            guard_dia >= 2.0 * TUBE_INNER_R - 0.012,
            f"Guard should span the bore, diameter={guard_dia:.4f}",
        )

    # Impeller sits just behind the guard (visible through it, not touching).
    ctx.expect_gap(
        guard,
        impeller,
        axis="x",
        min_gap=0.002,
        max_gap=0.06,
        name="impeller_just_behind_guard",
    )

    # ------------------------------------------------------------------
    # Motor mount: struts reach the duct wall; shaft engages the impeller hub.
    # ------------------------------------------------------------------
    ctx.expect_contact(
        mount,
        shell,
        elem_a="motor_strut_0",
        elem_b="shell_tube",
        name="strut_reaches_duct_wall",
    )
    ctx.expect_contact(
        mount,
        impeller,
        elem_a="motor_shaft",
        elem_b="impeller_rotor",
        name="shaft_engages_impeller_hub",
    )
    # Motor body is inside the tube, behind the impeller.
    ctx.expect_within(mount, shell, axes="yz", margin=0.0, name="motor_inside_bore")
    ctx.expect_gap(
        impeller,
        mount,
        axis="x",
        min_gap=None,
        max_penetration=0.035,
        positive_elem="impeller_rotor",
        negative_elem="motor_body",
        name="motor_behind_impeller",
    )

    # ------------------------------------------------------------------
    # Articulation: one continuous joint spinning the impeller about the
    # tube's central horizontal axis with unlimited rotation.
    # ------------------------------------------------------------------
    ctx.check(
        "impeller_joint_continuous",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        f"Impeller joint should be CONTINUOUS, got {spin.articulation_type!r}",
    )
    limits = spin.motion_limits
    ctx.check(
        "impeller_unlimited_rotation",
        limits is not None and limits.lower is None and limits.upper is None,
        f"Continuous spin must have no position limits, got {limits!r}",
    )

    # Impeller nearly fills the bore and stays a thin axial disc.
    imp_aabb = ctx.part_world_aabb(impeller)
    ctx.check("impeller_aabb_present", imp_aabb is not None, "Expected impeller AABB.")
    if imp_aabb is None:
        return ctx.report()
    imin, imax = imp_aabb
    imp_dia = max(float(imax[1] - imin[1]), float(imax[2] - imin[2]))
    ctx.check(
        "impeller_nearly_fills_bore",
        2.0 * TUBE_INNER_R - 0.025 <= imp_dia <= 2.0 * TUBE_INNER_R,
        f"Impeller diameter {imp_dia:.4f} should nearly fill bore {2.0 * TUBE_INNER_R:.4f}",
    )
    ctx.check(
        "impeller_axially_compact",
        float(imax[0] - imin[0]) <= 0.10,
        f"Impeller should be a compact axial rotor, x-extent={float(imax[0] - imin[0]):.4f}",
    )
    # Impeller spin axis is the horizontal tube axis: its center stays on it.
    ctx.check(
        "impeller_centered_on_axis",
        abs(float(imin[1] + imax[1]) / 2.0) <= 0.006
        and abs(float(imin[2] + imax[2]) / 2.0 - AXIS_Z) <= 0.006,
        f"Impeller should be centered on the duct axis, aabb={imp_aabb!r}",
    )

    # Decisive pose check: spinning a quarter turn keeps the rotor disc in
    # the same plane (axis = duct axis). A wrong spin axis would swing the
    # blade disc out of the bore / stretch its axial extent.
    with ctx.pose({spin: math.pi / 2.0}):
        posed_aabb = ctx.part_world_aabb(impeller)
        ctx.check(
            "spun_impeller_aabb_present",
            posed_aabb is not None,
            "Expected impeller AABB at posed rotation.",
        )
        if posed_aabb is not None:
            pmin, pmax = posed_aabb
            ctx.check(
                "spin_axis_is_duct_axis",
                float(pmax[0] - pmin[0]) <= 0.10
                and max(float(pmax[1] - pmin[1]), float(pmax[2] - pmin[2])) >= 0.27,
                f"Quarter-turn pose must keep the rotor disc radial, aabb={posed_aabb!r}",
            )
        ctx.expect_within(
            impeller,
            shell,
            axes="yz",
            margin=0.0,
            name="spun_impeller_stays_in_bore",
        )
        ctx.expect_gap(
            guard,
            impeller,
            axis="x",
            min_gap=0.002,
            name="spun_impeller_clears_guard",
        )

    return ctx.report()


object_model = build_object_model()
