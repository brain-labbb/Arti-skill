from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)

# --- Dimensions (meters) -----------------------------------------------------
W = 0.36          # width (X)
D = 0.22          # depth (Y)
SHELL_Z0 = 0.05   # shell base above the wheels
SHELL_H = 0.52    # shell height
SHELL_TOP = SHELL_Z0 + SHELL_H  # 0.57
CORNER_R = 0.045

# Fixed top grab handle
GRAB_SPAN = 0.14        # half-distance between posts in X
GRAB_POST_H = 0.035     # post height above shell top
GRAB_PEAK_H = 0.065     # arch peak above shell top
GRAB_TUBE_R = 0.012     # grip tube radius

WHEEL_R = 0.035
WHEEL_T = 0.024
WHEEL_X = 0.13
WHEEL_Y = 0.072
WHEEL_CZ = WHEEL_R  # wheel touches ground at z=0


def _build_grab_handle_mesh() -> "MeshGeometry":
    """Build a curved arch grab handle via spline-swept tube."""
    z_top = SHELL_TOP
    pts = [
        (-GRAB_SPAN, 0.0, z_top + GRAB_POST_H),
        (-GRAB_SPAN * 0.6, 0.0, z_top + GRAB_PEAK_H * 0.85),
        (0.0, 0.0, z_top + GRAB_PEAK_H),
        (GRAB_SPAN * 0.6, 0.0, z_top + GRAB_PEAK_H * 0.85),
        (GRAB_SPAN, 0.0, z_top + GRAB_POST_H),
    ]
    return tube_from_spline_points(
        pts,
        radius=GRAB_TUBE_R,
        samples_per_segment=10,
        radial_segments=12,
        cap_ends=True,
        up_hint=(0.0, 1.0, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_luggage_bag")

    shell_mat = model.material("shell_green", rgba=(0.20, 0.40, 0.16, 1.0))
    trim_mat = model.material("trim_black", rgba=(0.08, 0.08, 0.09, 1.0))
    metal_mat = model.material("metal_chrome", rgba=(0.60, 0.60, 0.63, 1.0))
    wheel_mat = model.material("wheel_black", rgba=(0.12, 0.12, 0.13, 1.0))
    grip_mat = model.material("grip_rubber", rgba=(0.15, 0.15, 0.16, 1.0))

    # --- Body shell (root) ---------------------------------------------------
    body = model.part("luggage_body")
    shell_geo = ExtrudeGeometry.from_z0(
        rounded_rect_profile(W, D, CORNER_R, corner_segments=8),
        SHELL_H,
        cap=True,
    )
    shell_geo.translate(0.0, 0.0, SHELL_Z0)
    body.visual(mesh_from_geometry(shell_geo, "shell"), material=shell_mat, name="shell")

    # central frame seam (darker band around the middle split line)
    for i, sx in enumerate((-1.0, 1.0)):
        body.visual(
            Box((0.012, D + 0.006, 0.018)),
            origin=Origin(xyz=(sx * (W / 2.0 - 0.004), 0.0, SHELL_Z0 + SHELL_H * 0.5)),
            material=trim_mat,
            name=f"seam_side_{i}",
        )
    body.visual(
        Box((W + 0.006, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, -(D / 2.0 - 0.004), SHELL_Z0 + SHELL_H * 0.5)),
        material=trim_mat,
        name="seam_front",
    )

    # --- Fixed top grab handle (inlined as body visuals) ---------------------
    # Two mounting posts on the shell top
    for i, sx in enumerate((-GRAB_SPAN, GRAB_SPAN)):
        body.visual(
            Box((0.030, 0.030, GRAB_POST_H)),
            origin=Origin(xyz=(sx, 0.0, SHELL_TOP + GRAB_POST_H / 2.0)),
            material=trim_mat,
            name=f"grab_post_{i}",
        )

    # Curved arch grip connecting the posts
    grab_mesh = _build_grab_handle_mesh()
    body.visual(
        mesh_from_geometry(grab_mesh, "grab_grip"),
        material=grip_mat,
        name="grab_grip",
    )

    # --- Side grab handle on the +X face ------------------------------------
    for i, sy in enumerate((-0.05, 0.05)):
        body.visual(
            Box((0.035, 0.022, 0.022)),
            origin=Origin(xyz=(W / 2.0 + 0.0125, sy, SHELL_Z0 + SHELL_H * 0.78)),
            material=trim_mat,
            name=f"side_post_{i}",
        )
    body.visual(
        Cylinder(radius=0.011, length=0.12),
        origin=Origin(
            xyz=(W / 2.0 + 0.028, 0.0, SHELL_Z0 + SHELL_H * 0.78),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_mat,
        name="side_grip",
    )

    # --- Caster yokes that carry the four base wheels -----------------------
    wheel_positions = [
        (-WHEEL_X, -WHEEL_Y),
        (WHEEL_X, -WHEEL_Y),
        (-WHEEL_X, WHEEL_Y),
        (WHEEL_X, WHEEL_Y),
    ]
    for i, (sx, sy) in enumerate(wheel_positions):
        body.visual(
            Box((0.05, WHEEL_T + 0.018, 0.045)),
            origin=Origin(xyz=(sx, sy, SHELL_Z0 + 0.012)),
            material=trim_mat,
            name=f"yoke_{i}",
        )

    # --- Four spinner wheels (continuous spin about local Y) -----------------
    for i, (sx, sy) in enumerate(wheel_positions):
        wheel = model.part(f"wheel_{i}")
        wheel.visual(
            Cylinder(radius=WHEEL_R, length=WHEEL_T),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=wheel_mat,
            name="tire",
        )
        # axle stub captured by the yoke (grounds the wheel)
        wheel.visual(
            Cylinder(radius=0.006, length=WHEEL_T + 0.026),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal_mat,
            name="axle",
        )
        # tread lug to make rotation observable (placed at the front of the rim)
        wheel.visual(
            Box((0.010, WHEEL_T - 0.004, 0.010)),
            origin=Origin(xyz=(WHEEL_R - 0.005, 0.0, 0.0)),
            material=metal_mat,
            name="tread_lug",
        )
        model.articulation(
            f"wheel_{i}_spin",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(sx, sy, WHEEL_CZ)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=20.0),
        )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("luggage_body")
    wheel0 = object_model.get_part("wheel_0")
    wheel0_spin = object_model.get_articulation("wheel_0_spin")

    # --- Fixed top grab handle checks ----------------------------------------
    # The grab grip is above the shell top surface.
    ctx.expect_gap(
        body,
        body,
        axis="z",
        min_gap=-0.001,
        positive_elem="grab_grip",
        negative_elem="shell",
        name="grab grip sits above the shell top",
    )

    # The grab grip spans across the top between the posts (XY overlap).
    ctx.expect_overlap(
        body,
        body,
        axes="xy",
        elem_a="grab_grip",
        elem_b="grab_post_0",
        min_overlap=0.01,
        name="grab grip overlaps left post in XY",
    )
    ctx.expect_overlap(
        body,
        body,
        axes="xy",
        elem_a="grab_grip",
        elem_b="grab_post_1",
        min_overlap=0.01,
        name="grab grip overlaps right post in XY",
    )

    # Grab posts are mounted on the shell top surface.
    for i in range(2):
        ctx.expect_contact(
            body,
            body,
            elem_a=f"grab_post_{i}",
            elem_b="shell",
            contact_tol=0.002,
            name=f"grab post {i} contacts shell top",
        )

    # --- No telescoping handle part exists -----------------------------------
    part_names = {p.name for p in object_model.parts}
    ctx.check(
        "no telescoping pull handle part",
        "pull_handle" not in part_names,
        details="The pull_handle part should not exist in this variant.",
    )

    # --- Wheel checks --------------------------------------------------------
    # Each wheel is intentionally recessed into its caster yoke/housing well,
    # captured by the axle stub.
    for i in range(4):
        ctx.allow_overlap(
            object_model.get_part(f"wheel_{i}"),
            body,
            elem_a="axle",
            elem_b=f"yoke_{i}",
            reason="Wheel axle stub is captured inside the caster yoke.",
        )
        ctx.allow_overlap(
            object_model.get_part(f"wheel_{i}"),
            body,
            elem_a="tire",
            elem_b=f"yoke_{i}",
            reason="Spinner wheel is recessed up into its caster housing well.",
        )
        ctx.allow_overlap(
            object_model.get_part(f"wheel_{i}"),
            body,
            elem_a="tire",
            elem_b="shell",
            reason="Spinner wheel is recessed into the shell underside.",
        )

    # Wheels are mounted (axle contacts yoke), not floating.
    ctx.expect_contact(
        wheel0,
        body,
        elem_a="axle",
        elem_b="yoke_0",
        contact_tol=1e-4,
        name="front wheel axle is seated in its yoke",
    )

    # Wheel spins: the tread lug sweeps from top toward the rolling direction.
    with ctx.pose({wheel0_spin: 0.0}):
        lug_rest = ctx.part_element_world_aabb(wheel0, elem="tread_lug")
    with ctx.pose({wheel0_spin: math.pi / 2.0}):
        lug_spun = ctx.part_element_world_aabb(wheel0, elem="tread_lug")
    ctx.check(
        "wheel spins about its axle",
        lug_rest is not None
        and lug_spun is not None
        and lug_spun[1][2] < lug_rest[1][2] - 0.015,
        details=f"rest={lug_rest}, spun={lug_spun}",
    )

    # Four wheels exist.
    ctx.check(
        "four base wheels are present",
        all(object_model.get_part(f"wheel_{i}") is not None for i in range(4)),
        details="expected wheel_0..wheel_3",
    )

    return ctx.report()
