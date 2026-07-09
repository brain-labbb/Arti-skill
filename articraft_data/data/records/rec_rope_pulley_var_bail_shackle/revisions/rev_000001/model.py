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
    sample_catmull_rom_spline_2d,
    tube_from_spline_points,
)


PLATE_THICKNESS = 0.003
PLATE_Y = 0.010
SHEAVE_WIDTH = 0.014
SHEAVE_RADIUS = 0.0185
SHEAVE_BORE_RADIUS = 0.0055
AXLE_RADIUS = 0.0052


def _circle_profile(cx: float, cy: float, radius: float, segments: int = 36) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _side_plate_geometry() -> MeshGeometry:
    """One pear-shaped side cheek plate, extruded before rotation into the XZ plane.
    Two small circular holes near the top accept the U-bail legs instead of the old oval eye."""
    control = [
        (0.000, 0.067),
        (0.012, 0.064),
        (0.018, 0.053),
        (0.016, 0.037),
        (0.022, 0.012),
        (0.020, -0.017),
        (0.010, -0.029),
        (0.000, -0.033),
        (-0.010, -0.029),
        (-0.020, -0.017),
        (-0.022, 0.012),
        (-0.016, 0.037),
        (-0.018, 0.053),
        (-0.012, 0.064),
    ]
    outer = sample_catmull_rom_spline_2d(control, samples_per_segment=8, closed=True)
    # Two small circular holes for the U-bail legs, positioned above the axle bore.
    bail_hole_left = _circle_profile(-0.008, 0.050, 0.0025, segments=24)
    bail_hole_right = _circle_profile(0.008, 0.050, 0.0025, segments=24)
    axle_hole = _circle_profile(0.0, 0.0, 0.0063, segments=40)
    return ExtrudeWithHolesGeometry(
        outer,
        [bail_hole_left, bail_hole_right, axle_hole],
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


def _u_bail_geometry() -> MeshGeometry:
    """Captive U-bail (shackle head) straddling the block as the top anchor attachment.
    The two legs pass through aligned holes in both cheek plates above the axle;
    the crown curves above the plate tops to form the load-bearing head."""
    # Path is in the frame's local space (XZ plane, Y=0).
    # Collinear leg points keep the Catmull-Rom spline straight through the plate region.
    path = [
        (-0.008, 0.0, 0.036),   # left leg bottom
        (-0.008, 0.0, 0.042),   # left leg lower
        (-0.008, 0.0, 0.050),   # left leg at plate-hole height
        (-0.008, 0.0, 0.058),   # left leg upper
        (-0.008, 0.0, 0.066),   # left leg above plates
        (-0.006, 0.0, 0.073),   # crown approach
        (-0.003, 0.0, 0.078),   # crown shoulder
        (0.000, 0.0, 0.080),    # crown top
        (0.003, 0.0, 0.078),    # crown shoulder
        (0.006, 0.0, 0.073),    # crown approach
        (0.008, 0.0, 0.066),    # right leg above plates
        (0.008, 0.0, 0.058),    # right leg upper
        (0.008, 0.0, 0.050),    # right leg at plate-hole height
        (0.008, 0.0, 0.042),    # right leg lower
        (0.008, 0.0, 0.036),    # right leg bottom
    ]
    return tube_from_spline_points(
        path,
        radius=0.002,
        samples_per_segment=14,
        radial_segments=16,
        cap_ends=True,
        up_hint=(0.0, 1.0, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="bail_mount_rope_pulley",
        meta={
            "run_notes": (
                "Bail-mount rope pulley variant: the parent's fixed top oval eye/becket slot "
                "is replaced by a captive U-bail (shackle head) whose two legs pin through both "
                "cheek plates above the axle. The sheave remains free-spinning on frame_to_sheave. "
                "Two-tone finish: brushed stainless bail on anodized orange plates."
            )
        },
    )

    orange = model.material("anodized_orange", rgba=(0.95, 0.30, 0.08, 1.0))
    steel = model.material("brushed_stainless", rgba=(0.72, 0.70, 0.66, 1.0))
    dark = model.material("dark_shadow", rgba=(0.03, 0.035, 0.04, 1.0))

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
    # The axle is fixed to the side plates; the sheave rotates on its bore around it.
    frame.visual(
        Cylinder(radius=AXLE_RADIUS, length=0.029),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="axle_pin",
    )
    for name, y in (("front_axle_cap", PLATE_Y + PLATE_THICKNESS / 2.0 + 0.0012), ("rear_axle_cap", -PLATE_Y - PLATE_THICKNESS / 2.0 - 0.0012)):
        frame.visual(
            Cylinder(radius=0.0084, length=0.0024),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=name,
        )
    # Dark spacer pads are visible along the edge between the cheek plates, like the black
    # separating layer in the reference hardware.  They also make the plate stack legible.
    for name, x in (("edge_spacer_0", 0.0186), ("edge_spacer_1", -0.0186)):
        frame.visual(
            Cylinder(radius=0.0022, length=0.019),
            origin=Origin(xyz=(x, 0.0, -0.015), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark,
            name=name,
        )
    # Captive U-bail straddling the block: steel rod pinned through both cheek plates.
    frame.visual(
        mesh_from_geometry(_u_bail_geometry(), "u_bail_mesh"),
        material=steel,
        name="u_bail",
    )
    # Connecting sleeves bridging the bail legs (at Y=0) to the plate inner faces,
    # showing the legs pass through and are captured by both plates.
    bail_tube_radius = 0.002
    plate_inner_y = PLATE_Y - PLATE_THICKNESS / 2.0  # 0.0085
    for name, x, y_center, length in (
        ("bail_sleeve_front_left", -0.008, (bail_tube_radius + plate_inner_y) / 2.0, plate_inner_y - bail_tube_radius),
        ("bail_sleeve_front_right", 0.008, (bail_tube_radius + plate_inner_y) / 2.0, plate_inner_y - bail_tube_radius),
        ("bail_sleeve_rear_left", -0.008, -(bail_tube_radius + plate_inner_y) / 2.0, plate_inner_y - bail_tube_radius),
        ("bail_sleeve_rear_right", 0.008, -(bail_tube_radius + plate_inner_y) / 2.0, plate_inner_y - bail_tube_radius),
    ):
        frame.visual(
            Cylinder(radius=0.0024, length=length + 0.0002),
            origin=Origin(xyz=(x, y_center, 0.050), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=name,
        )

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

    model.articulation(
        "frame_to_sheave",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=sheave,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=20.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    sheave = object_model.get_part("sheave")
    spin = object_model.get_articulation("frame_to_sheave")

    # --- Bail-mount attachment interface checks ---
    bail_aabb = ctx.part_element_world_aabb(frame, elem="u_bail")
    plate_aabb = ctx.part_element_world_aabb(frame, elem="front_plate")
    ctx.check(
        "u_bail crown extends above the cheek plates",
        bail_aabb is not None
        and plate_aabb is not None
        and bail_aabb[1][2] > plate_aabb[1][2] + 0.005,
        details=f"bail_top_z={bail_aabb[1][2] if bail_aabb else None}, "
        f"plate_top_z={plate_aabb[1][2] if plate_aabb else None}",
    )
    # Bail legs pass through the plate region (overlap plates in Z, confirming
    # the legs extend through the hole height rather than sitting entirely above).
    ctx.expect_overlap(
        frame,
        frame,
        axes="z",
        elem_a="u_bail",
        elem_b="front_plate",
        min_overlap=0.020,
        name="u_bail legs overlap the plate height range in Z, confirming through-plate mounting",
    )

    # --- Sheave retention and clearance (unchanged from parent) ---
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

    return ctx.report()


object_model = build_object_model()
