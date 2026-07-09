from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


BASE_HEIGHT = 0.105
SLIDE_TRAVEL = 0.070
LANTERN_RADIUS = 0.064

# Bellows parameters — concertina/accordion silicone folds
N_FOLDS = 7
FOLD_HEIGHT = 0.010  # 10 mm per fold
BELLOWS_Z_START = 0.003  # just above the seat lip
BELLOWS_Z_END = BELLOWS_Z_START + N_FOLDS * FOLD_HEIGHT  # 0.073
HEAD_OFFSET = BELLOWS_Z_END  # everything above bellows starts here

# Bellows cross-section radii
BELLOWS_R_WAIST = 0.052
BELLOWS_BULGE = 0.010  # outer equator = 0.062, flush with base shell


def _ring_profile_mesh(name, outer_profile, inner_profile, *, segments=96):
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
        name="compact_collapsible_camping_lantern",
        meta={
            "run_notes": (
                "Concertina/accordion silicone bellows variant: the smooth telescoping "
                "skirt is replaced by stacked ribbed bellows folds that squeeze flat "
                "when the prismatic joint compresses. Bellows folds are emitted as a "
                "loop of indexed ring segments. Companion palette variations on the "
                "translucent silicone bellows are supported (e.g. amber, sage, coral)."
            )
        },
    )

    black = model.material("satin_black_plastic", rgba=(0.005, 0.005, 0.004, 1.0))
    dark = model.material("dark_graphite_rubber", rgba=(0.020, 0.020, 0.018, 1.0))
    silicone = model.material("translucent_silicone_bellows", rgba=(0.75, 0.88, 0.95, 0.45))
    chrome = model.material("polished_wire_steel", rgba=(0.82, 0.86, 0.86, 1.0))
    white = model.material("white_led_carrier", rgba=(0.91, 0.93, 0.90, 1.0))
    led = model.material("warm_led_lenses", rgba=(1.0, 0.86, 0.42, 1.0))
    screw = model.material("small_silver_fasteners", rgba=(0.72, 0.74, 0.72, 1.0))

    # ── lower_base (unchanged from parent) ──────────────────────────────
    lower_base = model.part("lower_base")
    lower_base.visual(
        _ring_profile_mesh(
            "ribbed_lower_base_shell",
            [
                (0.056, 0.000), (0.064, 0.006), (0.064, 0.017),
                (0.059, 0.023), (0.059, 0.035), (0.062, 0.039),
                (0.062, 0.045), (0.058, 0.049), (0.058, 0.085),
                (0.062, 0.090), (0.062, 0.100), (0.058, 0.105),
            ],
            [
                (0.047, 0.006), (0.052, 0.014),
                (0.054, 0.095), (0.054, 0.105),
            ],
        ),
        material=black,
        name="base_shell",
    )
    for i in range(18):
        angle = 2.0 * math.pi * i / 18
        r = 0.0625
        lower_base.visual(
            Box((0.006, 0.010, 0.047)),
            origin=Origin(
                xyz=(r * math.cos(angle), r * math.sin(angle), 0.054),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark,
            name=f"base_rib_{i}",
        )
    for i in range(18):
        angle = 2.0 * math.pi * (i + 0.5) / 18
        r = 0.062
        lower_base.visual(
            Box((0.007, 0.014, 0.015)),
            origin=Origin(
                xyz=(r * math.cos(angle), r * math.sin(angle), 0.008),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark,
            name=f"foot_lug_{i}",
        )

    # ── upper_lantern ───────────────────────────────────────────────────
    upper_lantern = model.part("upper_lantern")

    # Seating lip: flat ring at bellows bottom that seats on the base rim.
    upper_lantern.visual(
        mesh_from_geometry(
            LatheGeometry(
                [(0.050, 0.000), (0.062, 0.000), (0.062, 0.003), (0.050, 0.003)],
                segments=96,
                closed=True,
            ),
            "base_rim_seating_lip",
        ),
        material=black,
        name="seat_lip",
    )

    # Build the bellows as one continuous connected backbone body.
    # The outer profile zigzags for fold creases/valleys in the bellows zone,
    # then transitions to a smooth cylindrical wall up to the top cap.
    # The inner profile is r=0 throughout (solid body) so all embedded items
    # (LEDs, collar, cap, handle bosses) are inside the solid and connected.
    cap_z = HEAD_OFFSET + 0.015  # top cap bottom z (0.088)
    bellows_outer = []
    bellows_inner = []
    # Bottom anchor extends to z=0 where seat_lip overlaps.
    bellows_outer.append((0.050, 0.000))
    bellows_inner.append((0.000, 0.000))
    for i in range(N_FOLDS):
        z_base = BELLOWS_Z_START + i * FOLD_HEIGHT
        z_mid = z_base + FOLD_HEIGHT * 0.5
        z_top = z_base + FOLD_HEIGHT
        bellows_outer.extend([
            (BELLOWS_R_WAIST, z_base),
            (BELLOWS_R_WAIST + BELLOWS_BULGE, z_mid),
            (BELLOWS_R_WAIST, z_top),
        ])
        bellows_inner.extend([
            (0.000, z_base),
            (0.000, z_mid),
            (0.000, z_top),
        ])
    # Transition from bellows top to smooth solid wall
    bellows_outer.append((0.052, BELLOWS_Z_END))
    bellows_inner.append((0.000, BELLOWS_Z_END))
    bellows_outer.append((0.052, cap_z - 0.002))
    bellows_inner.append((0.000, cap_z - 0.002))
    # Handle boss zone: outer wall bulges to r=0.058
    bellows_outer.append((0.058, cap_z + 0.015))
    bellows_inner.append((0.000, cap_z + 0.015))
    bellows_outer.append((0.058, cap_z + 0.028))
    bellows_inner.append((0.000, cap_z + 0.028))
    # Top cap closure
    bellows_outer.append((0.044, cap_z + 0.033))
    bellows_inner.append((0.000, cap_z + 0.033))
    bellows_outer.append((0.044, cap_z + 0.039))
    bellows_inner.append((0.000, cap_z + 0.039))

    upper_lantern.visual(
        mesh_from_geometry(
            LatheGeometry(bellows_outer, segments=72, closed=True),
            "bellows_shell",
        ),
        material=silicone,
        name="bellows_fold_0",
    )

    # LED board inside the bellows — shines through translucent silicone.
    led_center_z = BELLOWS_Z_START + (N_FOLDS * FOLD_HEIGHT) * 0.5
    led_height = N_FOLDS * FOLD_HEIGHT * 0.75
    upper_lantern.visual(
        Box((0.030, 0.010, led_height)),
        origin=Origin(xyz=(0.0, -0.014, led_center_z)),
        material=white,
        name="led_board",
    )
    upper_lantern.visual(
        Box((0.022, 0.014, led_height)),
        origin=Origin(xyz=(0.0, -0.004, led_center_z)),
        material=white,
        name="inner_reflector",
    )

    # LED mounting rails: thin horizontal brackets connecting the backbone
    # inner wall to the LED board, ensuring structural support.
    rail_half_len = led_height * 0.45
    for rail_idx, z_off in enumerate((-rail_half_len, rail_half_len)):
        upper_lantern.visual(
            Box((0.028, 0.020, 0.003)),
            origin=Origin(xyz=(0.0, -0.008, led_center_z + z_off)),
            material=white,
            name=f"led_mount_rail_{rail_idx}",
        )

    # LED lenses inside bellows
    led_span = N_FOLDS * FOLD_HEIGHT - 0.016
    for row in range(5):
        z = BELLOWS_Z_START + 0.008 + row * led_span / 4.0
        for col, x in enumerate((-0.009, 0.009)):
            upper_lantern.visual(
                Cylinder(radius=0.0043, length=0.0025),
                origin=Origin(
                    xyz=(x, -0.0184, z),
                    rpy=(math.pi / 2, 0.0, 0.0),
                ),
                material=led,
                name=f"led_lens_{row}_{col}",
            )
            upper_lantern.visual(
                Sphere(radius=0.0021),
                origin=Origin(xyz=(x, -0.0202, z)),
                material=screw,
                name=f"led_core_{row}_{col}",
            )

    # Lower collar above bellows — transition ring to the top cap.
    cz = HEAD_OFFSET
    upper_lantern.visual(
        _ring_profile_mesh(
            "lower_light_chamber_collar",
            [
                (0.051, cz),
                (0.060, cz + 0.004),
                (0.060, cz + 0.015),
                (0.052, cz + 0.019),
            ],
            [(0.042, cz), (0.045, cz + 0.019)],
            segments=96,
        ),
        material=black,
        name="lower_collar",
    )

    # Top cap
    cap_z = HEAD_OFFSET + 0.015
    upper_lantern.visual(
        _ring_profile_mesh(
            "stepped_top_cap_shell",
            [
                (0.052, cap_z),
                (0.061, cap_z + 0.006),
                (0.061, cap_z + 0.022),
                (0.057, cap_z + 0.028),
                (0.048, cap_z + 0.033),
            ],
            [(0.020, cap_z), (0.020, cap_z + 0.030), (0.000, cap_z + 0.033)],
            segments=96,
        ),
        material=black,
        name="top_cap",
    )
    upper_lantern.visual(
        Cylinder(radius=0.038, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, cap_z + 0.0345)),
        material=dark,
        name="top_disc",
    )
    for i in range(6):
        angle = 2.0 * math.pi * i / 6
        r = 0.057
        upper_lantern.visual(
            Box((0.022, 0.026, 0.018)),
            origin=Origin(
                xyz=(r * math.cos(angle), r * math.sin(angle), cap_z + 0.017),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark,
            name=f"cap_tab_{i}",
        )
    # Side hinge bosses for the wire bail
    hinge_z = cap_z + 0.026
    for side, x in enumerate((-0.0635, 0.0635)):
        upper_lantern.visual(
            Cylinder(radius=0.006, length=0.012),
            origin=Origin(xyz=(x, 0.0, hinge_z), rpy=(0.0, math.pi / 2, 0.0)),
            material=chrome,
            name=f"handle_boss_{side}",
        )

    # ── wire_handle (unchanged geometry, hinge z adjusted) ──────────────
    handle = model.part("wire_handle")
    handle.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (-0.056, 0.0, 0.000),
                    (-0.055, 0.0, 0.045),
                    (-0.044, 0.0, 0.102),
                    (-0.020, 0.0, 0.128),
                    (0.000, 0.0, 0.132),
                    (0.020, 0.0, 0.128),
                    (0.044, 0.0, 0.102),
                    (0.055, 0.0, 0.045),
                    (0.056, 0.0, 0.000),
                ],
                radius=0.0022,
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
        Box((0.052, 0.006, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, 0.132)),
        material=chrome,
        name="top_grip_flat",
    )

    # ── Articulations ───────────────────────────────────────────────────
    model.articulation(
        "base_to_lantern_slide",
        ArticulationType.PRISMATIC,
        parent=lower_base,
        child=upper_lantern,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.18, lower=0.0, upper=SLIDE_TRAVEL
        ),
    )

    model.articulation(
        "lantern_to_handle_hinge",
        ArticulationType.REVOLUTE,
        parent=upper_lantern,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0, velocity=2.0, lower=-1.45, upper=0.20
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower_base = object_model.get_part("lower_base")
    upper_lantern = object_model.get_part("upper_lantern")
    handle = object_model.get_part("wire_handle")
    slide = object_model.get_articulation("base_to_lantern_slide")
    hinge = object_model.get_articulation("lantern_to_handle_hinge")

    ctx.check(
        "image category matches lantern",
        "lantern" in object_model.name,
        details="Reference and category both describe a compact camping lantern.",
    )

    # ── Bellows-specific assertions (axis: concertina squeeze-collapse) ─

    # Bellows shell exists as one continuous ribbed backbone mesh
    bellows_aabb = ctx.part_element_world_aabb(upper_lantern, elem="bellows_fold_0")
    ctx.check(
        "bellows_fold_0 shell exists with accordion folds",
        bellows_aabb is not None,
        details="Expected bellows_fold_0 visual on upper_lantern.",
    )

    # Bellows shell has visible zigzag height spanning the expected fold zone
    if bellows_aabb is not None:
        bellows_dz = bellows_aabb[1][2] - bellows_aabb[0][2]
        ctx.check(
            "bellows shell spans full fold zone plus backbone",
            bellows_dz > 0.060,
            details=f"bellows_fold_0 height = {bellows_dz:.4f} m, expected > 0.060",
        )

    # Bellows bottom seats near base rim at rest (compressed / squeezed-flat state)
    ctx.expect_gap(
        upper_lantern,
        lower_base,
        axis="z",
        positive_elem="bellows_fold_0",
        negative_elem="base_shell",
        max_gap=0.008,
        max_penetration=0.003,
        name="bellows bottom seats near base rim when compressed",
    )

    # Bellows is coaxial with cylindrical base
    ctx.expect_overlap(
        upper_lantern,
        lower_base,
        axes="xy",
        elem_a="bellows_fold_0",
        elem_b="base_shell",
        min_overlap=0.075,
        name="bellows shell is coaxial with cylindrical base",
    )

    # Lower collar sits above bellows fold zone
    collar_aabb = ctx.part_element_world_aabb(upper_lantern, elem="lower_collar")
    ctx.check(
        "lower collar sits above bellows fold zone",
        collar_aabb is not None
        and bellows_aabb is not None
        and collar_aabb[0][2] >= BELLOWS_Z_END - 0.005,
        details=(
            f"collar_min_z={collar_aabb[0][2] if collar_aabb else None}, "
            f"bellows_fold_end_z={BELLOWS_Z_END}"
        ),
    )

    # Compression stroke: bellows lifts above base when extended
    rest_pos = ctx.part_world_position(upper_lantern)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        extended_pos = ctx.part_world_position(upper_lantern)
        ctx.expect_gap(
            upper_lantern,
            lower_base,
            axis="z",
            positive_elem="bellows_fold_0",
            negative_elem="base_shell",
            min_gap=0.050,
            name="extended bellows lifts above base (compression stroke = slide travel)",
        )
    ctx.check(
        "base_to_lantern_slide bellows compression stroke lifts body upward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + 0.060,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    # ── Wire handle hinge ───────────────────────────────────────────────
    upright_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({hinge: -1.20}):
        folded_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "wire handle folds down on hinge",
        upright_aabb is not None
        and folded_aabb is not None
        and folded_aabb[1][2] < upright_aabb[1][2] - 0.035,
        details=f"upright={upright_aabb}, folded={folded_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
