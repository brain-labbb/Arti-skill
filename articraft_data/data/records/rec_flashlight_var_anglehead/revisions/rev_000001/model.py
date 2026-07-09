from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


# Rotation that maps local Z axis to world X axis (for barrel-aligned parts).
AXIS_TO_X = Origin(rpy=(0.0, math.pi / 2.0, 0.0))


def _axis_x_origin(x: float, y: float = 0.0, z: float = 0.0) -> Origin:
    """Place a local-Z axis primitive/lathe so its axis runs along world X."""
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


# ── angle-head layout constants ──────────────────────────────────────
# The grip barrel runs along world X; the head module points along +Z
# (perpendicular, MX-991 angle-head style).
X_HEAD = -0.010          # X position of the head/neck centre line
Z_BARREL_TOP = 0.028     # top of the barrel cylinder surface
Z_NECK_TOP = 0.055       # base of the head shell (top of neck)
Z_HEAD_BASE = Z_NECK_TOP
Z_HEAD_TOP = 0.140       # lens end of head
Z_HEAD_MID = 0.5 * (Z_HEAD_BASE + Z_HEAD_TOP)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="yellow_angle_head_flashlight")

    yellow = model.material("glossy_yellow_plastic", rgba=(1.0, 0.86, 0.0, 1.0))
    black = model.material("black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    dark_button = model.material("dark_button_insert", rgba=(0.02, 0.022, 0.02, 1.0))
    silver = model.material("mirror_silver_reflector", rgba=(0.86, 0.88, 0.86, 1.0))
    lens_mat = model.material("clear_polycarbonate", rgba=(0.82, 0.94, 1.0, 0.35))
    led_mat = model.material("warm_led", rgba=(1.0, 0.86, 0.45, 1.0))

    body = model.part("body")

    # ── Grip barrel (along X, same as parent) ────────────────────────
    body.visual(
        Cylinder(radius=0.028, length=0.180),
        origin=_axis_x_origin(0.055),
        material=yellow,
        name="barrel_shell",
    )

    # ── Neck / shoulder (tapers from barrel top to head base, along Z) ─
    shoulder = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.029, Z_BARREL_TOP),
            (0.038, 0.040),
            (0.048, Z_NECK_TOP),
        ],
        inner_profile=[
            (0.024, Z_BARREL_TOP),
            (0.032, 0.040),
            (0.039, Z_NECK_TOP),
        ],
        segments=96,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )
    body.visual(
        mesh_from_geometry(shoulder, "shoulder_shell"),
        origin=Origin(xyz=(X_HEAD, 0.0, 0.0)),
        material=yellow,
        name="shoulder_shell",
    )

    # ── Head shell (hollow, along Z, perpendicular to barrel) ────────
    head_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.048, Z_HEAD_BASE),
            (0.052, Z_HEAD_BASE + 0.009),
            (0.053, Z_HEAD_TOP - 0.013),
            (0.052, Z_HEAD_TOP),
        ],
        inner_profile=[
            (0.035, Z_HEAD_BASE),
            (0.040, Z_HEAD_BASE + 0.013),
            (0.041, Z_HEAD_TOP - 0.013),
            (0.041, Z_HEAD_TOP),
        ],
        segments=112,
        start_cap="flat",
        end_cap="flat",
        lip_samples=5,
    )
    body.visual(
        mesh_from_geometry(head_shell, "head_shell"),
        origin=Origin(xyz=(X_HEAD, 0.0, 0.0)),
        material=yellow,
        name="head_shell",
    )

    # ── Black bezel ring at the lens opening (top of head) ───────────
    front_ring = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.0535, Z_HEAD_TOP - 0.004), (0.0535, Z_HEAD_TOP + 0.003)],
        inner_profile=[(0.0415, Z_HEAD_TOP - 0.004), (0.0415, Z_HEAD_TOP + 0.003)],
        segments=112,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )
    body.visual(
        mesh_from_geometry(front_ring, "front_bezel_ring"),
        origin=Origin(xyz=(X_HEAD, 0.0, 0.0)),
        material=black,
        name="front_bezel_ring",
    )

    # ── Rear black band at the head base ─────────────────────────────
    rear_band = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.0525, Z_HEAD_BASE - 0.005), (0.0525, Z_HEAD_BASE + 0.009)],
        inner_profile=[(0.0435, Z_HEAD_BASE - 0.005), (0.0435, Z_HEAD_BASE + 0.009)],
        segments=96,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )
    body.visual(
        mesh_from_geometry(rear_band, "rear_bezel_band"),
        origin=Origin(xyz=(X_HEAD, 0.0, 0.0)),
        material=black,
        name="rear_bezel_band",
    )

    # ── Raised ribs on the bezel head (along Z, radial array) ────────
    head_length = Z_HEAD_TOP - Z_HEAD_BASE
    rib_length = 0.60 * head_length
    for i in range(16):
        theta = 2.0 * math.pi * i / 16.0
        dx = 0.055 * math.cos(theta)
        dy = 0.055 * math.sin(theta)
        body.visual(
            Cylinder(radius=0.0030, length=rib_length),
            origin=Origin(xyz=(X_HEAD + dx, dy, Z_HEAD_MID)),
            material=yellow,
            name=f"head_rib_{i}",
        )

    # ── Grip rails along the barrel (unchanged from parent) ──────────
    for i, theta in enumerate((math.pi / 2.0, 3.0 * math.pi / 2.0, 0.0, math.pi)):
        y = 0.0295 * math.cos(theta)
        z = 0.0295 * math.sin(theta)
        body.visual(
            Cylinder(radius=0.0018, length=0.128),
            origin=_axis_x_origin(0.055, y, z),
            material=yellow,
            name=f"barrel_grip_{i}",
        )

    # ── Tail cap and lanyard eyelet (unchanged) ──────────────────────
    body.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=_axis_x_origin(0.153),
        material=black,
        name="tail_cap",
    )
    body.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.020, tube=0.0026, radial_segments=18, tubular_segments=56),
            "tail_eyelet",
        ),
        origin=_axis_x_origin(0.162),
        material=black,
        name="tail_eyelet",
    )

    # ── Parabolic reflector (inside head, opening faces +Z) ──────────
    reflector_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.0415, Z_HEAD_TOP - 0.005),
            (0.030, Z_HEAD_TOP - 0.015),
            (0.019, Z_HEAD_TOP - 0.032),
            (0.009, Z_HEAD_TOP - 0.050),
        ],
        inner_profile=[
            (0.0385, Z_HEAD_TOP - 0.007),
            (0.027, Z_HEAD_TOP - 0.017),
            (0.016, Z_HEAD_TOP - 0.033),
            (0.006, Z_HEAD_TOP - 0.048),
        ],
        segments=112,
        start_cap="flat",
        end_cap="round",
        lip_samples=6,
    )
    body.visual(
        mesh_from_geometry(reflector_shell, "parabolic_reflector"),
        origin=Origin(xyz=(X_HEAD, 0.0, 0.0)),
        material=silver,
        name="parabolic_reflector",
    )

    # ── LED bulb at reflector focus ──────────────────────────────────
    body.visual(
        Sphere(radius=0.006),
        origin=Origin(xyz=(X_HEAD, 0.0, Z_HEAD_TOP - 0.050)),
        material=led_mat,
        name="led_bulb",
    )

    # ── Clear lens disc at head opening (along Z) ────────────────────
    body.visual(
        Cylinder(radius=0.0415, length=0.004),
        origin=Origin(xyz=(X_HEAD, 0.0, Z_HEAD_TOP)),
        material=lens_mat,
        name="lens_disc",
    )

    # ── Button (child part, radial press on barrel +Y side) ──────────
    button = model.part("button")
    button_base = ExtrudeGeometry(
        rounded_rect_profile(0.038, 0.024, radius=0.012, corner_segments=10),
        0.006,
        cap=True,
        center=True,
    )
    button.visual(
        mesh_from_geometry(button_base, "button_base"),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="button_base",
    )
    button_cap = ExtrudeGeometry(
        rounded_rect_profile(0.026, 0.016, radius=0.008, corner_segments=10),
        0.0022,
        cap=True,
        center=True,
    )
    button.visual(
        mesh_from_geometry(button_cap, "button_cap"),
        origin=Origin(xyz=(0.0, 0.0071, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_button,
        name="button_cap",
    )

    # ── Strap (child part, nylon loop at tail) ───────────────────────
    strap = model.part("strap")
    strap_loop = tube_from_spline_points(
        [
            (0.004, 0.018, 0.000),
            (0.032, 0.038, -0.002),
            (0.090, 0.044, -0.006),
            (0.124, 0.000, -0.008),
            (0.092, -0.044, -0.006),
            (0.030, -0.036, -0.002),
            (0.004, -0.018, 0.000),
        ],
        radius=0.0024,
        samples_per_segment=18,
        closed_spline=True,
        radial_segments=18,
        cap_ends=False,
    )
    strap.visual(
        mesh_from_geometry(strap_loop, "strap_loop"),
        origin=Origin(),
        material=black,
        name="strap_loop",
    )

    # ── Articulations ────────────────────────────────────────────────
    model.articulation(
        "body_to_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button,
        origin=Origin(xyz=(-0.008, 0.028, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.05, lower=0.0, upper=0.004),
    )
    model.articulation(
        "body_to_strap",
        ArticulationType.FIXED,
        parent=body,
        child=strap,
        origin=Origin(xyz=(0.162, 0.0, 0.0)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    button = object_model.get_part("button")
    strap = object_model.get_part("strap")
    switch = object_model.get_articulation("body_to_button")

    ctx.allow_overlap(
        strap,
        body,
        elem_a="strap_loop",
        elem_b="tail_eyelet",
        reason="The soft nylon strap is threaded through the tail eyelet opening.",
    )

    # ── Right-angle form: head perpendicular to barrel ───────────────
    barrel_box = ctx.part_element_world_aabb(body, elem="barrel_shell")
    head_box = ctx.part_element_world_aabb(body, elem="head_shell")
    ctx.check(
        "head oriented perpendicular to barrel (Z axis)",
        barrel_box is not None
        and head_box is not None
        and (head_box[1][2] - head_box[0][2]) > 0.060
        and head_box[0][2] > barrel_box[1][2] - 0.010
        and head_box[1][2] > barrel_box[1][2] + 0.040,
        details=f"barrel={barrel_box}, head={head_box}",
    )

    # ── Head wider than barrel in XY ─────────────────────────────────
    ctx.check(
        "wider head than barrel",
        barrel_box is not None
        and head_box is not None
        and (head_box[1][1] - head_box[0][1]) > (barrel_box[1][1] - barrel_box[0][1]) + 0.035,
        details=f"barrel={barrel_box}, head={head_box}",
    )

    # ── Optics: lens, reflector, bezel alignment along Z ─────────────
    lens_box = ctx.part_element_world_aabb(body, elem="lens_disc")
    front_ring_box = ctx.part_element_world_aabb(body, elem="front_bezel_ring")
    reflector_box = ctx.part_element_world_aabb(body, elem="parabolic_reflector")
    ctx.check(
        "clear lens seated inside black front bezel",
        lens_box is not None
        and front_ring_box is not None
        and lens_box[0][0] >= front_ring_box[0][0] - 0.0015
        and lens_box[1][0] <= front_ring_box[1][0] + 0.0015
        and lens_box[0][1] >= front_ring_box[0][1] - 0.0015
        and lens_box[1][1] <= front_ring_box[1][1] + 0.0015,
        details=f"lens={lens_box}, ring={front_ring_box}",
    )
    ctx.check(
        "reflector visible behind lens (XY overlap)",
        reflector_box is not None
        and lens_box is not None
        and min(reflector_box[1][0], lens_box[1][0]) - max(reflector_box[0][0], lens_box[0][0]) > 0.030
        and min(reflector_box[1][1], lens_box[1][1]) - max(reflector_box[0][1], lens_box[0][1]) > 0.030,
        details=f"reflector={reflector_box}, lens={lens_box}",
    )
    ctx.check(
        "lens sits just above reflector opening",
        reflector_box is not None
        and lens_box is not None
        and 0.0 <= lens_box[0][2] - reflector_box[1][2] <= 0.006,
        details=f"reflector={reflector_box}, lens={lens_box}",
    )

    # ── Button: radial press on barrel surface ───────────────────────
    ctx.expect_gap(
        button,
        body,
        axis="y",
        positive_elem="button_base",
        negative_elem="barrel_shell",
        min_gap=0.0,
        max_gap=0.0015,
        name="side button rests on barrel surface",
    )
    rest_pos = ctx.part_world_position(button)
    with ctx.pose({switch: 0.004}):
        pressed_pos = ctx.part_world_position(button)
    ctx.check(
        "button presses inward",
        rest_pos is not None and pressed_pos is not None and pressed_pos[1] < rest_pos[1] - 0.003,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )

    # ── Strap: emerges from tail eyelet ──────────────────────────────
    ctx.expect_gap(
        strap,
        body,
        axis="x",
        positive_elem="strap_loop",
        negative_elem="tail_eyelet",
        min_gap=-0.006,
        max_gap=0.010,
        name="strap loop emerges from tail eyelet",
    )
    ctx.expect_overlap(
        strap,
        body,
        axes="yz",
        elem_a="strap_loop",
        elem_b="tail_eyelet",
        min_overlap=0.010,
        name="strap loop is aligned with tail eyelet",
    )

    return ctx.report()


object_model = build_object_model()
