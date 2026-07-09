from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


# ── Slim stick-lantern proportions ──────────────────────────────────────────
FOOT_RADIUS = 0.032
FOOT_HEIGHT = 0.050
TUBE_OUTER_R = 0.022
TUBE_INNER_R = 0.019
TUBE_HEIGHT = 0.220
COLLAR_TOP = 0.016
CAP_BASE_Z = TUBE_HEIGHT + COLLAR_TOP - 0.005  # small overlap into collar region
CAP_TOP_Z = CAP_BASE_Z + 0.030
HINGE_Z = CAP_TOP_Z - 0.012


def _ring_profile_mesh(
    name: str,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    *,
    segments: int = 96,
):
    """Thin-walled revolved shell with visible lips/ridges."""
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=segments,
            start_cap="flat",
            end_cap="flat",
            lip_samples=4,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="slim_tube_camping_lantern",
        meta={
            "run_notes": (
                "Slim stick/tube camping lantern variant: tall thin glowing column "
                "replacing the stout canister cylinder. Height-to-diameter ratio ~5:1. "
                "Static seat replaces telescoping slide; fold-up wire bail hinge retained."
            )
        },
    )

    black = model.material("satin_black_plastic", rgba=(0.005, 0.005, 0.004, 1.0))
    dark = model.material("dark_graphite_rubber", rgba=(0.020, 0.020, 0.018, 1.0))
    clear = model.material("slightly_smoked_clear_plastic", rgba=(0.70, 0.92, 1.0, 0.32))
    chrome = model.material("polished_wire_steel", rgba=(0.82, 0.86, 0.86, 1.0))
    white = model.material("white_led_carrier", rgba=(0.91, 0.93, 0.90, 1.0))
    led = model.material("warm_led_lenses", rgba=(1.0, 0.86, 0.42, 1.0))
    screw = model.material("small_silver_fasteners", rgba=(0.72, 0.74, 0.72, 1.0))

    # ── Lower base: slim ribbed foot ────────────────────────────────────────
    lower_base = model.part("lower_base")
    lower_base.visual(
        _ring_profile_mesh(
            "ribbed_lower_base_shell",
            [
                (0.024, 0.000),
                (0.032, 0.005),
                (0.032, 0.014),
                (0.029, 0.019),
                (0.029, 0.030),
                (0.031, 0.034),
                (0.031, 0.042),
                (0.028, 0.046),
                (0.026, 0.050),
            ],
            [
                (0.020, 0.005),
                (0.023, 0.012),
                (0.024, 0.044),
                (0.024, 0.050),
            ],
        ),
        material=black,
        name="base_shell",
    )
    # Vertical grip ribs on the slim foot.
    for i in range(12):
        angle = 2.0 * math.pi * i / 12
        r = 0.0305
        lower_base.visual(
            Box((0.005, 0.008, 0.028)),
            origin=Origin(
                xyz=(r * math.cos(angle), r * math.sin(angle), 0.028),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark,
            name=f"base_rib_{i}",
        )
    # Bottom castellated grip pads.
    for i in range(12):
        angle = 2.0 * math.pi * (i + 0.5) / 12
        r = 0.030
        lower_base.visual(
            Box((0.005, 0.010, 0.012)),
            origin=Origin(
                xyz=(r * math.cos(angle), r * math.sin(angle), 0.006),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark,
            name=f"foot_lug_{i}",
        )

    # ── Upper lantern: slim tall tube column ────────────────────────────────
    upper_lantern = model.part("upper_lantern")

    # Static seating lip where tube meets base.
    upper_lantern.visual(
        mesh_from_geometry(
            LatheGeometry(
                [(0.024, 0.000), (0.028, 0.000), (0.028, 0.003), (0.024, 0.003)],
                segments=96,
                closed=True,
            ),
            "base_rim_seating_lip",
        ),
        material=black,
        name="seat_lip",
    )

    # Lower collar: transitions from base width down to tube width.
    upper_lantern.visual(
        _ring_profile_mesh(
            "lower_light_chamber_collar",
            [(0.023, 0.000), (0.028, 0.004), (0.028, 0.012), (0.024, COLLAR_TOP)],
            [(0.019, 0.000), (0.020, COLLAR_TOP)],
            segments=96,
        ),
        material=black,
        name="lower_collar",
    )

    # Transparent light chamber tube: slim tall thin-walled cylinder.
    upper_lantern.visual(
        mesh_from_geometry(
            LatheGeometry(
                [
                    (TUBE_OUTER_R, COLLAR_TOP),
                    (TUBE_OUTER_R, COLLAR_TOP + TUBE_HEIGHT),
                    (TUBE_INNER_R, COLLAR_TOP + TUBE_HEIGHT),
                    (TUBE_INNER_R, COLLAR_TOP),
                ],
                segments=96,
                closed=True,
            ),
            "transparent_light_chamber_tube",
        ),
        material=clear,
        name="clear_chamber",
    )

    # Subtle vertical fluting on the tube exterior (host-conformal ridges).
    tube_mid_z = COLLAR_TOP + TUBE_HEIGHT / 2.0
    flute_height = TUBE_HEIGHT - 0.010
    for i in range(8):
        angle = 2.0 * math.pi * i / 8
        r = TUBE_OUTER_R + 0.001
        upper_lantern.visual(
            Box((0.003, 0.002, flute_height)),
            origin=Origin(
                xyz=(r * math.cos(angle), r * math.sin(angle), tube_mid_z),
                rpy=(0.0, 0.0, angle),
            ),
            material=clear,
            name=f"tube_flute_{i}",
        )

    # Carrier spine: thin structural plate spanning tube inner diameter,
    # provides the physical mounting path for the LED assembly inside the tube.
    upper_lantern.visual(
        Box((0.0385, 0.002, TUBE_HEIGHT - 0.004)),
        origin=Origin(xyz=(0.0, 0.0, tube_mid_z)),
        material=white,
        name="carrier_spine",
    )

    # Central LED strip mounted on the carrier spine (tall narrow board).
    upper_lantern.visual(
        Box((0.012, 0.008, TUBE_HEIGHT - 0.020)),
        origin=Origin(xyz=(0.0, -0.005, tube_mid_z)),
        material=white,
        name="led_board",
    )
    upper_lantern.visual(
        Box((0.008, 0.004, TUBE_HEIGHT - 0.020)),
        origin=Origin(xyz=(0.0, 0.003, tube_mid_z)),
        material=white,
        name="inner_reflector",
    )

    # LED lenses distributed along the tall column (8 rows × 2 columns).
    for row in range(8):
        z = COLLAR_TOP + 0.018 + row * (TUBE_HEIGHT - 0.036) / 7.0
        for col, x in enumerate((-0.004, 0.004)):
            upper_lantern.visual(
                Cylinder(radius=0.0030, length=0.002),
                origin=Origin(
                    xyz=(x, -0.010, z),
                    rpy=(math.pi / 2, 0.0, 0.0),
                ),
                material=led,
                name=f"led_lens_{row}_{col}",
            )
            upper_lantern.visual(
                Sphere(radius=0.0015),
                origin=Origin(xyz=(x, -0.0115, z)),
                material=screw,
                name=f"led_core_{row}_{col}",
            )

    # Stepped top cap for the slim tube.
    upper_lantern.visual(
        _ring_profile_mesh(
            "stepped_top_cap_shell",
            [
                (TUBE_OUTER_R, CAP_BASE_Z),
                (0.027, CAP_BASE_Z + 0.005),
                (0.027, CAP_BASE_Z + 0.020),
                (0.024, CAP_BASE_Z + 0.026),
                (0.018, CAP_TOP_Z),
            ],
            [(0.010, CAP_BASE_Z), (0.010, CAP_TOP_Z - 0.003), (0.000, CAP_TOP_Z)],
            segments=96,
        ),
        material=black,
        name="top_cap",
    )
    upper_lantern.visual(
        Cylinder(radius=0.016, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, CAP_TOP_Z + 0.002)),
        material=dark,
        name="top_disc",
    )
    # Small raised tabs around the top cap.
    for i in range(6):
        angle = 2.0 * math.pi * i / 6
        r = 0.025
        upper_lantern.visual(
            Box((0.014, 0.016, 0.014)),
            origin=Origin(
                xyz=(r * math.cos(angle), r * math.sin(angle), CAP_BASE_Z + 0.013),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark,
            name=f"cap_tab_{i}",
        )

    # Side hinge bosses for the wire bail.
    for side, x in enumerate((-0.029, 0.029)):
        upper_lantern.visual(
            Cylinder(radius=0.005, length=0.010),
            origin=Origin(xyz=(x, 0.0, HINGE_Z), rpy=(0.0, math.pi / 2, 0.0)),
            material=chrome,
            name=f"handle_boss_{side}",
        )

    # ── Wire handle: fold-up bail ───────────────────────────────────────────
    handle = model.part("wire_handle")
    handle.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (-0.024, 0.0, 0.000),
                    (-0.023, 0.0, 0.035),
                    (-0.018, 0.0, 0.070),
                    (-0.008, 0.0, 0.085),
                    (0.000, 0.0, 0.088),
                    (0.008, 0.0, 0.085),
                    (0.018, 0.0, 0.070),
                    (0.023, 0.0, 0.035),
                    (0.024, 0.0, 0.000),
                ],
                radius=0.0018,
                samples_per_segment=10,
                radial_segments=18,
                cap_ends=True,
            ),
            "fold_up_wire_bail",
        ),
        material=chrome,
        name="wire_bail",
    )
    handle.visual(
        Box((0.030, 0.005, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
        material=chrome,
        name="top_grip_flat",
    )

    # ── Articulations ───────────────────────────────────────────────────────
    # Static seat: upper lantern fixed to base top (no telescoping).
    model.articulation(
        "base_to_lantern_seat",
        ArticulationType.FIXED,
        parent=lower_base,
        child=upper_lantern,
        origin=Origin(xyz=(0.0, 0.0, FOOT_HEIGHT)),
    )

    # Fold-up wire bail hinge (preserved from parent).
    model.articulation(
        "lantern_to_handle_hinge",
        ArticulationType.REVOLUTE,
        parent=upper_lantern,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        # q=0 shows the carry handle upright; negative q folds it down.
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-1.45, upper=0.20),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower_base = object_model.get_part("lower_base")
    upper_lantern = object_model.get_part("upper_lantern")
    handle = object_model.get_part("wire_handle")
    hinge = object_model.get_articulation("lantern_to_handle_hinge")

    ctx.check(
        "slim tube lantern identity",
        "slim" in object_model.name and "lantern" in object_model.name,
        details="Variant must read as a slim tube camping lantern.",
    )

    # ── clear_chamber is a slim tall tube (height >> diameter) ──────────────
    chamber_aabb = ctx.part_element_world_aabb(upper_lantern, elem="clear_chamber")
    if chamber_aabb is not None:
        lo, hi = chamber_aabb
        tube_h = hi[2] - lo[2]
        tube_dx = hi[0] - lo[0]
        tube_dy = hi[1] - lo[1]
        tube_d = max(tube_dx, tube_dy)
        ratio = tube_h / tube_d if tube_d > 0 else 0
        ctx.check(
            "clear_chamber height-to-diameter ratio > 4",
            ratio > 4.0,
            details=f"ratio={ratio:.1f}, height={tube_h:.4f}, diameter={tube_d:.4f}",
        )
    else:
        ctx.fail("clear_chamber slim column check", "Could not resolve clear_chamber AABB.")

    # ── Upper lantern seats on lower base (static, no slide) ────────────────
    ctx.expect_gap(
        upper_lantern,
        lower_base,
        axis="z",
        positive_elem="seat_lip",
        negative_elem="base_shell",
        max_gap=0.002,
        max_penetration=0.002,
        name="upper lantern statically seated on base top",
    )

    # ── Clear chamber is coaxial with base ──────────────────────────────────
    ctx.expect_overlap(
        upper_lantern,
        lower_base,
        axes="xy",
        elem_a="clear_chamber",
        elem_b="base_shell",
        min_overlap=0.030,
        name="clear chamber is coaxial with slim base foot",
    )

    # ── Wire handle folds down on hinge ─────────────────────────────────────
    upright_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({hinge: -1.20}):
        folded_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "wire handle folds down on hinge",
        upright_aabb is not None
        and folded_aabb is not None
        and folded_aabb[1][2] < upright_aabb[1][2] - 0.025,
        details=f"upright={upright_aabb}, folded={folded_aabb}",
    )

    # ── Handle bosses exist on upper_lantern ────────────────────────────────
    boss_0 = upper_lantern.get_visual("handle_boss_0")
    boss_1 = upper_lantern.get_visual("handle_boss_1")
    ctx.check(
        "handle bosses present",
        boss_0 is not None and boss_1 is not None,
        details="Hinge bosses must mount the wire bail to the top cap.",
    )

    return ctx.report()


object_model = build_object_model()
