from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sample_catmull_rom_spline_2d,
)


PLATE_THICKNESS = 0.003
PLATE_Y = 0.010
SHEAVE_WIDTH = 0.014
SHEAVE_RADIUS = 0.0185
SHEAVE_BORE_RADIUS = 0.0055
AXLE_RADIUS = 0.0052

CAM_WIDTH = 0.008
CAM_PIVOT_Z = -0.025
CAM_BASE_RADIUS = 0.010
CAM_TOOTH_HEIGHT = 0.005
CAM_BORE_RADIUS = 0.0033
CAM_AXLE_RADIUS = 0.003
CAM_N_TEETH = 6


def _circle_profile(cx: float, cy: float, radius: float, segments: int = 36) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _translated_profile(
    profile: list[tuple[float, float]], dx: float, dy: float
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _side_plate_geometry() -> MeshGeometry:
    """One pear-shaped side cheek plate with prusik-minding lower skirts and cam bore."""
    control = [
        (0.000, 0.067),
        (0.012, 0.064),
        (0.018, 0.053),
        (0.016, 0.037),
        (0.022, 0.012),
        (0.020, -0.017),
        (0.014, -0.029),
        (0.010, -0.038),
        (0.007, -0.044),
        (0.000, -0.046),
        (-0.007, -0.044),
        (-0.010, -0.038),
        (-0.014, -0.029),
        (-0.020, -0.017),
        (-0.022, 0.012),
        (-0.016, 0.037),
        (-0.018, 0.053),
        (-0.012, 0.064),
    ]
    outer = sample_catmull_rom_spline_2d(control, samples_per_segment=8, closed=True)
    # Top oval is the rope/carabiner attachment eye; lower hole is the sheave axle bore.
    top_slot = _translated_profile(rounded_rect_profile(0.018, 0.009, 0.0045, corner_segments=8), 0.0, 0.044)
    axle_hole = _circle_profile(0.0, 0.0, 0.0063, segments=40)
    # Capture-cam pivot bore in the cheek plates, below the main axle.
    cam_axle_hole = _circle_profile(0.0, CAM_PIVOT_Z, 0.0035, segments=32)
    return ExtrudeWithHolesGeometry(
        outer,
        [top_slot, axle_hole, cam_axle_hole],
        PLATE_THICKNESS,
        center=True,
    )


def _grooved_sheave_geometry() -> MeshGeometry:
    """Lathe a small rope sheave around the local Y axis with a central bore."""
    # Closed radial profile in (radius, y).  The outer radius dips at the center
    # so the wheel reads as a rope-guiding V/U groove rather than a flat disk.
    profile = [
        (0.0160, -SHEAVE_WIDTH / 2.0),
        (0.0182, -0.0052),
        (0.0188, -0.0032),
        (0.0150, 0.0000),
        (0.0188, 0.0032),
        (0.0182, 0.0052),
        (0.0160, SHEAVE_WIDTH / 2.0),
        (SHEAVE_BORE_RADIUS, SHEAVE_WIDTH / 2.0),
        (SHEAVE_BORE_RADIUS, -SHEAVE_WIDTH / 2.0),
    ]
    segments = 72
    geom = MeshGeometry()
    for i in range(segments):
        theta = 2.0 * math.pi * i / segments
        c = math.cos(theta)
        s = math.sin(theta)
        for radius, y in profile:
            geom.add_vertex(radius * c, y, radius * s)

    n = len(profile)
    for i in range(segments):
        j = (i + 1) % segments
        for k in range(n):
            a = i * n + k
            b = j * n + k
            c = j * n + ((k + 1) % n)
            d = i * n + ((k + 1) % n)
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)
    return geom


def _capture_cam_profile() -> list[tuple[float, float]]:
    """Toothed progress-capture cam profile in the local XY plane.

    Teeth are on the +Y arc and face toward the sheave (+Z world) after
    the same pi/2 X-rotation applied to the side plates.
    """
    base_r = CAM_BASE_RADIUS
    tooth_h = CAM_TOOTH_HEIGHT
    n_teeth = CAM_N_TEETH

    points: list[tuple[float, float]] = []

    # Teeth cover the upper arc (~25° to ~155°), smooth arc covers the rest.
    tooth_start = math.radians(25.0)
    tooth_end = math.radians(155.0)
    tooth_span = tooth_end - tooth_start

    # Smooth lower arc (no teeth — this is the spring-loaded back of the cam)
    smooth_steps = 24
    smooth_start = tooth_end
    smooth_span = 2.0 * math.pi - tooth_span
    for i in range(smooth_steps):
        angle = smooth_start + smooth_span * i / smooth_steps
        points.append((base_r * math.cos(angle), base_r * math.sin(angle)))

    # Toothed upper arc — asymmetric ratchet teeth (gradual rise, sharp drop)
    for i in range(n_teeth):
        f0 = i / n_teeth
        f_tip = (i + 0.65) / n_teeth
        f1 = (i + 1.0) / n_teeth

        a0 = tooth_start + tooth_span * f0
        a_tip = tooth_start + tooth_span * f_tip
        a1 = tooth_start + tooth_span * f1

        points.append((base_r * math.cos(a0), base_r * math.sin(a0)))
        points.append(
            ((base_r + tooth_h) * math.cos(a_tip), (base_r + tooth_h) * math.sin(a_tip))
        )
        points.append((base_r * math.cos(a1), base_r * math.sin(a1)))

    return points


def _capture_cam_geometry() -> MeshGeometry:
    """Progress-capture cam body with asymmetric teeth and a pivot bore."""
    outer = _capture_cam_profile()
    bore = _circle_profile(0.0, 0.0, CAM_BORE_RADIUS, segments=32)
    return ExtrudeWithHolesGeometry(outer, [bore], CAM_WIDTH, center=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="progress_capture_pulley",
        meta={
            "run_notes": (
                "Progress-capture rope pulley (haul-system ratchet pulley). "
                "The grooved sheave spins freely on its axle while a spring-loaded "
                "toothed cam below the sheave grips the rope on reversal. "
                "Two-tone anodized finish: steel cam on colored plates."
            )
        },
    )

    orange = model.material("anodized_orange", rgba=(0.95, 0.30, 0.08, 1.0))
    steel = model.material("brushed_stainless", rgba=(0.72, 0.70, 0.66, 1.0))
    dark = model.material("dark_shadow", rgba=(0.03, 0.035, 0.04, 1.0))

    # ── Frame (root) ──────────────────────────────────────────────────
    frame = model.part("frame")
    frame.visual(
        mesh_from_geometry(_side_plate_geometry(), "front_plate_mesh"),
        origin=Origin(xyz=(0.0, PLATE_Y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=orange,
        name="front_plate",
    )
    frame.visual(
        mesh_from_geometry(_side_plate_geometry(), "rear_plate_mesh"),
        origin=Origin(xyz=(0.0, -PLATE_Y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=orange,
        name="rear_plate",
    )
    # Main sheave axle pin (fixed to the side plates; sheave rotates around it)
    frame.visual(
        Cylinder(radius=AXLE_RADIUS, length=0.029),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="axle_pin",
    )
    for name, y in (
        ("front_axle_cap", PLATE_Y + PLATE_THICKNESS / 2.0 + 0.0012),
        ("rear_axle_cap", -PLATE_Y - PLATE_THICKNESS / 2.0 - 0.0012),
    ):
        frame.visual(
            Cylinder(radius=0.0084, length=0.0024),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=name,
        )
    # Dark spacer pads visible along the edge between the cheek plates.
    for name, x in (("edge_spacer_0", 0.0186), ("edge_spacer_1", -0.0186)):
        frame.visual(
            Cylinder(radius=0.0022, length=0.019),
            origin=Origin(xyz=(x, 0.0, -0.015), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark,
            name=name,
        )
    # Capture-cam axle pin (smaller pivot pin below the main axle)
    frame.visual(
        Cylinder(radius=CAM_AXLE_RADIUS, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, CAM_PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="cam_axle_pin",
    )
    # Cam axle retaining caps (press-fit into the plate bores)
    for name, y in (
        ("front_cam_cap", PLATE_Y + PLATE_THICKNESS / 2.0 + 0.0008),
        ("rear_cam_cap", -PLATE_Y - PLATE_THICKNESS / 2.0 - 0.0008),
    ):
        frame.visual(
            Cylinder(radius=0.005, length=0.0016),
            origin=Origin(xyz=(0.0, y, CAM_PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=name,
        )

    # ── Sheave ────────────────────────────────────────────────────────
    sheave = model.part("sheave")
    sheave.visual(
        mesh_from_geometry(_grooved_sheave_geometry(), "grooved_sheave_mesh"),
        material=steel,
        name="grooved_sheave",
    )
    # A small stamped witness mark on the near flange makes the continuous sheave spin visible.
    sheave.visual(
        Box((0.010, 0.00045, 0.0014)),
        origin=Origin(xyz=(0.010, SHEAVE_WIDTH / 2.0 + 0.00022, 0.0)),
        material=dark,
        name="rotation_mark",
    )

    # ── Capture cam ───────────────────────────────────────────────────
    capture_cam = model.part("capture_cam")
    capture_cam.visual(
        mesh_from_geometry(_capture_cam_geometry(), "cam_body_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="cam_body",
    )

    # ── Articulations ─────────────────────────────────────────────────
    model.articulation(
        "frame_to_sheave",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=sheave,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=20.0),
    )

    # Progress-capture cam: REVOLUTE with limited travel.
    # Positive q swings the toothed face toward +X (opening the cam away from the rope).
    # At q=0 the teeth face the sheave groove (engaged/rest position).
    model.articulation(
        "frame_to_capture_cam",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=capture_cam,
        origin=Origin(xyz=(0.0, 0.0, CAM_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0, lower=-0.1, upper=0.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    sheave = object_model.get_part("sheave")
    capture_cam = object_model.get_part("capture_cam")
    spin = object_model.get_articulation("frame_to_sheave")
    cam_joint = object_model.get_articulation("frame_to_capture_cam")

    # ── Sheave checks (preserved from parent) ────────────────────────
    ctx.expect_within(
        sheave,
        frame,
        axes="y",
        inner_elem="grooved_sheave",
        outer_elem="axle_pin",
        margin=0.008,
        name="sheave is retained within the axle span",
    )
    ctx.expect_overlap(
        sheave,
        frame,
        axes="xz",
        elem_a="grooved_sheave",
        elem_b="axle_pin",
        min_overlap=0.009,
        name="sheave bore surrounds the axle location in projection",
    )
    ctx.expect_gap(
        frame,
        sheave,
        axis="y",
        positive_elem="front_plate",
        negative_elem="grooved_sheave",
        min_gap=0.001,
        max_gap=0.003,
        name="front plate clears the spinning sheave",
    )
    ctx.expect_gap(
        sheave,
        frame,
        axis="y",
        positive_elem="grooved_sheave",
        negative_elem="rear_plate",
        min_gap=0.001,
        max_gap=0.003,
        name="rear plate clears the spinning sheave",
    )

    rest_origin = ctx.part_world_position(sheave)
    rest_mark = ctx.part_element_world_aabb(sheave, elem="rotation_mark")
    with ctx.pose({spin: math.pi / 2.0}):
        turned_origin = ctx.part_world_position(sheave)
        turned_mark = ctx.part_element_world_aabb(sheave, elem="rotation_mark")

    def _center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_center = _center(rest_mark)
    turned_center = _center(turned_mark)
    ctx.check(
        "continuous sheave spin keeps axle center fixed",
        rest_origin is not None
        and turned_origin is not None
        and all(abs(rest_origin[i] - turned_origin[i]) < 1e-6 for i in range(3)),
        details=f"rest={rest_origin}, turned={turned_origin}",
    )
    ctx.check(
        "rotation mark moves around the sheave axis",
        rest_center is not None
        and turned_center is not None
        and abs(rest_center[0] - turned_center[0]) > 0.006
        and abs(rest_center[2] - turned_center[2]) > 0.006,
        details=f"rest_mark={rest_center}, turned_mark={turned_center}",
    )

    # ── Capture-cam checks (new for this progress-capture variant) ───

    # Cam body fits between the cheek plates (Y clearance on both sides)
    ctx.expect_gap(
        frame,
        capture_cam,
        axis="y",
        positive_elem="front_plate",
        negative_elem="cam_body",
        min_gap=0.0,
        max_gap=0.005,
        name="capture cam fits between the cheek plates (front)",
    )
    ctx.expect_gap(
        capture_cam,
        frame,
        axis="y",
        positive_elem="cam_body",
        negative_elem="rear_plate",
        min_gap=0.0,
        max_gap=0.005,
        name="capture cam clears the rear plate",
    )

    # Cam pivot test: positive q swings the toothed face from +Z toward +X
    # (opening the cam away from the rope path).  Compare AABB max extents
    # rather than centers, because the cam is nearly symmetric around its pivot.
    cam_rest = ctx.part_element_world_aabb(capture_cam, elem="cam_body")
    with ctx.pose({cam_joint: 0.4}):
        cam_open = ctx.part_element_world_aabb(capture_cam, elem="cam_body")

    if cam_rest is not None and cam_open is not None:
        rest_lo, rest_hi = cam_rest
        open_lo, open_hi = cam_open
        dx_max = open_hi[0] - rest_hi[0]   # max X should increase as teeth swing right
        dz_max = rest_hi[2] - open_hi[2]   # max Z should drop as teeth leave the top
        ctx.check(
            "frame_to_capture_cam rotation swings teeth away from rope path on positive q",
            dx_max > 0.001 or dz_max > 0.001,
            details=f"dx_max={dx_max:.5f}, dz_max={dz_max:.5f}",
        )
    else:
        ctx.fail("frame_to_capture_cam rotation test", "cam AABB unavailable")

    # Verify the cam has a toothed profile: it extends further toward the sheave
    # (+Z from pivot) than a plain circle of CAM_BASE_RADIUS would.
    cam_aabb = ctx.part_element_world_aabb(capture_cam, elem="cam_body")
    if cam_aabb is not None:
        cam_lo, cam_hi = cam_aabb
        pivot_z = CAM_PIVOT_Z
        extent_toward_sheave = cam_hi[2] - pivot_z
        extent_away = pivot_z - cam_lo[2]
        ctx.check(
            "capture cam teeth extend toward sheave beyond base radius",
            extent_toward_sheave > extent_away + 0.002,
            details=f"toward_sheave={extent_toward_sheave:.5f}, away={extent_away:.5f}",
        )

    # Intentional overlap: the cam teeth protrude into the rope channel of the
    # sheave groove so they can grip the rope on reversal.  This is the defining
    # mechanical relationship of a progress-capture pulley.
    ctx.allow_overlap(
        capture_cam,
        sheave,
        elem_a="cam_body",
        elem_b="grooved_sheave",
        reason=(
            "The capture cam teeth intentionally protrude into the rope channel "
            "of the sheave groove to grip the rope on reversal, as in a real "
            "progress-capture pulley mechanism."
        ),
    )
    # Cam axle pin passes through the cam bore as a pivot — small local overlap.
    ctx.allow_overlap(
        frame,
        capture_cam,
        elem_a="cam_axle_pin",
        elem_b="cam_body",
        reason=(
            "The cam axle pin passes through the cam bore as a pivot bearing; "
            "the pin diameter is smaller than the bore diameter."
        ),
    )

    return ctx.report()


object_model = build_object_model()
