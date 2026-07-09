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


DISC_HEIGHT = 0.020
DISC_RADIUS = 0.054
LANTERN_RADIUS = 0.048


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
        name="multi_mount_camping_lantern",
        meta={
            "run_notes": (
                "Compact multi-mount camping lantern variant: flat magnetic disc base "
                "with fold-out spring clip replaces the tall canister base and telescoping "
                "slide. Wire bail handle hinge and clear chamber preserved. "
                "Dark gray body with bright orange accent clip."
            )
        },
    )

    # ── Materials ──────────────────────────────────────────────────────
    dark_gray = model.material("dark_gray_body", rgba=(0.12, 0.12, 0.13, 1.0))
    dark_rubber = model.material("dark_rubber_pad", rgba=(0.04, 0.04, 0.04, 1.0))
    accent_orange = model.material("accent_orange_clip", rgba=(0.95, 0.40, 0.08, 1.0))
    magnet_steel = model.material("magnet_steel_surface", rgba=(0.08, 0.08, 0.09, 1.0))
    clear = model.material("slightly_smoked_clear_plastic", rgba=(0.70, 0.92, 1.0, 0.32))
    chrome = model.material("polished_wire_steel", rgba=(0.82, 0.86, 0.86, 1.0))
    white = model.material("white_led_carrier", rgba=(0.91, 0.93, 0.90, 1.0))
    led = model.material("warm_led_lenses", rgba=(1.0, 0.86, 0.42, 1.0))
    screw = model.material("small_silver_fasteners", rgba=(0.72, 0.74, 0.72, 1.0))

    # ── lower_base: thin magnetic puck disc (root) ─────────────────────
    lower_base = model.part("lower_base")

    # Solid disc body with recessed pocket on top for upper lantern spigot.
    lower_base.visual(
        mesh_from_geometry(
            LatheGeometry(
                [
                    (0.000, 0.000),
                    (DISC_RADIUS, 0.000),
                    (DISC_RADIUS + 0.002, 0.003),
                    (DISC_RADIUS + 0.002, 0.017),
                    (DISC_RADIUS, DISC_HEIGHT),
                    (LANTERN_RADIUS + 0.002, DISC_HEIGHT),
                    (LANTERN_RADIUS + 0.002, 0.006),
                    (0.000, 0.006),
                ],
                segments=96,
                closed=True,
            ),
            "magnetic_puck_disc",
        ),
        material=dark_gray,
        name="base_shell",
    )

    # Magnetic contact ring on the bottom face.
    lower_base.visual(
        mesh_from_geometry(
            LatheGeometry(
                [
                    (0.012, -0.001),
                    (0.044, -0.001),
                    (0.044, 0.000),
                    (0.012, 0.000),
                ],
                segments=72,
                closed=True,
            ),
            "magnetic_contact_ring",
        ),
        material=magnet_steel,
        name="magnet_ring",
    )

    # Anti-slip rubber pad ring on the bottom.
    lower_base.visual(
        mesh_from_geometry(
            LatheGeometry(
                [
                    (0.008, -0.0015),
                    (0.050, -0.0015),
                    (0.050, -0.001),
                    (0.008, -0.001),
                ],
                segments=72,
                closed=True,
            ),
            "rubber_anti_slip_ring",
        ),
        material=dark_rubber,
        name="anti_slip_pad",
    )

    # Clip hinge boss: small cylindrical pivot pin at the +Y disc top rim.
    lower_base.visual(
        Cylinder(radius=0.004, length=0.014),
        origin=Origin(xyz=(0.0, DISC_RADIUS + 0.001, DISC_HEIGHT), rpy=(0.0, math.pi / 2, 0.0)),
        material=dark_gray,
        name="clip_hinge_boss",
    )

    # Subtle circumferential grip texture on the disc side.
    for i in range(24):
        angle = 2.0 * math.pi * i / 24
        r = DISC_RADIUS + 0.0025
        lower_base.visual(
            Box((0.004, 0.006, 0.010)),
            origin=Origin(
                xyz=(r * math.cos(angle), r * math.sin(angle), 0.010),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_rubber,
            name=f"disc_grip_{i}",
        )

    # ── upper_lantern: chamber + cap + LEDs (FIXED seat on disc) ──────
    upper_lantern = model.part("upper_lantern")

    # Locating spigot ring that seats into the disc pocket.
    upper_lantern.visual(
        _ring_profile_mesh(
            "locating_spigot_ring",
            [(0.046, -0.013), (0.046, 0.003)],
            [(0.040, -0.013), (0.040, 0.003)],
            segments=72,
        ),
        material=dark_gray,
        name="skirt_sleeve",
    )

    # Transition collar from spigot to clear chamber.
    upper_lantern.visual(
        _ring_profile_mesh(
            "lower_light_chamber_collar",
            [(0.040, 0.000), (0.050, 0.004), (0.050, 0.013), (0.044, 0.016)],
            [(0.034, 0.000), (0.037, 0.016)],
            segments=96,
        ),
        material=dark_gray,
        name="lower_collar",
    )

    # Seating lip at the base of the collar.
    upper_lantern.visual(
        mesh_from_geometry(
            LatheGeometry(
                [(0.044, 0.000), (0.050, 0.000), (0.050, 0.003), (0.044, 0.003)],
                segments=96,
                closed=True,
            ),
            "base_rim_seating_lip",
        ),
        material=dark_gray,
        name="seat_lip",
    )

    # Transparent cylindrical chamber (thin hollow tube).
    upper_lantern.visual(
        mesh_from_geometry(
            LatheGeometry(
                [(0.044, 0.013), (0.044, 0.098), (0.041, 0.098), (0.041, 0.013)],
                segments=96,
                closed=True,
            ),
            "transparent_light_chamber_tube",
        ),
        material=clear,
        name="clear_chamber",
    )

    # Vertical clear posts/guards on the transparent tube.
    for i in range(4):
        angle = 2.0 * math.pi * i / 4 + math.pi / 4
        r = 0.0445
        upper_lantern.visual(
            Box((0.005, 0.003, 0.082)),
            origin=Origin(
                xyz=(r * math.cos(angle), r * math.sin(angle), 0.056),
                rpy=(0.0, 0.0, angle),
            ),
            material=clear,
            name=f"clear_post_{i}",
        )

    # Central LED strip and reflector.
    upper_lantern.visual(
        Box((0.024, 0.008, 0.080)),
        origin=Origin(xyz=(0.0, -0.012, 0.056)),
        material=white,
        name="led_board",
    )
    upper_lantern.visual(
        Box((0.018, 0.010, 0.080)),
        origin=Origin(xyz=(0.0, -0.004, 0.056)),
        material=white,
        name="inner_reflector",
    )

    # LED lens dots inside the chamber.
    for row in range(4):
        z = 0.028 + row * 0.018
        for col, x in enumerate((-0.007, 0.007)):
            upper_lantern.visual(
                Cylinder(radius=0.0035, length=0.002),
                origin=Origin(
                    xyz=(x, -0.0155, z),
                    rpy=(math.pi / 2, 0.0, 0.0),
                ),
                material=led,
                name=f"led_lens_{row}_{col}",
            )
            upper_lantern.visual(
                Sphere(radius=0.0018),
                origin=Origin(xyz=(x, -0.0170, z)),
                material=screw,
                name=f"led_core_{row}_{col}",
            )

    # Stepped top cap shell.
    upper_lantern.visual(
        _ring_profile_mesh(
            "stepped_top_cap_shell",
            [
                (0.044, 0.094),
                (0.051, 0.100),
                (0.051, 0.114),
                (0.047, 0.120),
                (0.040, 0.125),
            ],
            [(0.018, 0.094), (0.018, 0.122), (0.000, 0.125)],
            segments=96,
        ),
        material=dark_gray,
        name="top_cap",
    )

    # Top disc cap.
    upper_lantern.visual(
        Cylinder(radius=0.032, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.126)),
        material=dark_rubber,
        name="top_disc",
    )

    # Raised tabs around the top cap.
    for i in range(6):
        angle = 2.0 * math.pi * i / 6
        r = 0.048
        upper_lantern.visual(
            Box((0.018, 0.020, 0.014)),
            origin=Origin(
                xyz=(r * math.cos(angle), r * math.sin(angle), 0.110),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_rubber,
            name=f"cap_tab_{i}",
        )

    # Side hinge bosses for the wire bail (KEEP: handle_boss_0, handle_boss_1).
    for side, x in enumerate((-0.052, 0.052)):
        upper_lantern.visual(
            Cylinder(radius=0.005, length=0.010),
            origin=Origin(xyz=(x, 0.0, 0.118), rpy=(0.0, math.pi / 2, 0.0)),
            material=chrome,
            name=f"handle_boss_{side}",
        )

    # ── wire_handle: fold-up bail (REVOLUTE child of upper_lantern) ───
    handle = model.part("wire_handle")
    handle.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (-0.044, 0.0, 0.000),
                    (-0.043, 0.0, 0.032),
                    (-0.035, 0.0, 0.074),
                    (-0.016, 0.0, 0.096),
                    (0.000, 0.0, 0.100),
                    (0.016, 0.0, 0.096),
                    (0.035, 0.0, 0.074),
                    (0.043, 0.0, 0.032),
                    (0.044, 0.0, 0.000),
                ],
                radius=0.0020,
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
        Box((0.042, 0.005, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=chrome,
        name="top_grip_flat",
    )

    # ── spring_clip: fold-out clip arm (REVOLUTE child of lower_base) ─
    clip = model.part("spring_clip")

    # Main clip arm: flat bar extending upward from pivot in local frame.
    # At q=0, the arm extends along +Z (flush against the lantern lower body).
    clip.visual(
        Box((0.010, 0.003, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=accent_orange,
        name="clip_arm",
    )

    # Pivot collar at the hinge: wraps around the hinge boss pin.
    clip.visual(
        Cylinder(radius=0.005, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.002), rpy=(0.0, math.pi / 2, 0.0)),
        material=accent_orange,
        name="clip_pivot_collar",
    )

    # Hook lip at the free end: small inward bend for clamping.
    clip.visual(
        Box((0.010, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, -0.004, 0.035)),
        material=accent_orange,
        name="clip_hook",
    )

    # Spring detent bump on the inner face of the arm.
    clip.visual(
        Sphere(radius=0.002),
        origin=Origin(xyz=(0.0, -0.003, 0.022)),
        material=accent_orange,
        name="clip_detent",
    )

    # ── Articulations ─────────────────────────────────────────────────

    # FIXED seat: upper_lantern sits rigidly on the magnetic disc.
    model.articulation(
        "base_to_lantern_seat",
        ArticulationType.FIXED,
        parent=lower_base,
        child=upper_lantern,
        origin=Origin(xyz=(0.0, 0.0, DISC_HEIGHT)),
    )

    # Wire handle hinge (KEEP: lantern_to_handle_hinge).
    model.articulation(
        "lantern_to_handle_hinge",
        ArticulationType.REVOLUTE,
        parent=upper_lantern,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, 0.118)),
        axis=(1.0, 0.0, 0.0),
        # q=0: handle upright; negative q folds it down against the lantern body.
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-1.45, upper=0.20),
    )

    # Fold-out spring clip: hinge at +Y disc top rim, axis tangent to rim.
    # q=0: arm folded flat against disc side (pointing up along +Z local).
    # axis=(-1,0,0): positive q swings arm outward in +Y (away from lantern center).
    model.articulation(
        "base_to_clip",
        ArticulationType.REVOLUTE,
        parent=lower_base,
        child=clip,
        origin=Origin(xyz=(0.0, DISC_RADIUS + 0.001, DISC_HEIGHT)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=0.0, upper=1.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower_base = object_model.get_part("lower_base")
    upper_lantern = object_model.get_part("upper_lantern")
    handle = object_model.get_part("wire_handle")
    clip = object_model.get_part("spring_clip")
    hinge = object_model.get_articulation("lantern_to_handle_hinge")
    clip_joint = object_model.get_articulation("base_to_clip")

    # ── Category and identity ─────────────────────────────────────────
    ctx.check(
        "object is a camping lantern",
        "lantern" in object_model.name,
        details="Multi-mount camping lantern with magnetic disc base.",
    )

    # ── Magnetic puck disc proportions ────────────────────────────────
    base_aabb = ctx.part_world_aabb(lower_base)
    ctx.check(
        "magnetic puck disc is thin (height < 30mm)",
        base_aabb is not None and (base_aabb[1][2] - base_aabb[0][2]) < 0.030,
        details=f"base_shell should read as a thin flat disc, aabb={base_aabb}",
    )

    # ── Intentional overlap: hinge pin inside pivot collar ────────────
    ctx.allow_overlap(
        lower_base,
        clip,
        elem_a="clip_hinge_boss",
        elem_b="clip_pivot_collar",
        reason=(
            "The clip pivot collar wraps around the hinge boss pin as a "
            "realistic hinge barrel/pin embedding."
        ),
    )

    # ── Fold-out spring clip mechanism ────────────────────────────────
    clip_arm_rest = ctx.part_element_world_aabb(clip, elem="clip_arm")
    with ctx.pose({clip_joint: 1.20}):
        clip_arm_deployed = ctx.part_element_world_aabb(clip, elem="clip_arm")

    ctx.check(
        "spring clip folds outward from magnetic disc rim",
        clip_arm_rest is not None
        and clip_arm_deployed is not None
        and clip_arm_deployed[1][1] > clip_arm_rest[1][1] + 0.008,
        details=(
            f"rest_max_y={clip_arm_rest[1][1] if clip_arm_rest else None}, "
            f"deployed_max_y={clip_arm_deployed[1][1] if clip_arm_deployed else None}"
        ),
    )

    # ── Upper lantern seats on disc ───────────────────────────────────
    ctx.expect_contact(
        upper_lantern,
        lower_base,
        elem_a="skirt_sleeve",
        elem_b="base_shell",
        contact_tol=0.005,
        name="upper lantern spigot seats in disc pocket",
    )

    # ── Clear chamber coaxial with base ───────────────────────────────
    ctx.expect_overlap(
        upper_lantern,
        lower_base,
        axes="xy",
        elem_a="clear_chamber",
        elem_b="base_shell",
        min_overlap=0.060,
        name="clear chamber is coaxial with magnetic disc",
    )

    # ── Wire handle folds on hinge ────────────────────────────────────
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

    return ctx.report()


object_model = build_object_model()
