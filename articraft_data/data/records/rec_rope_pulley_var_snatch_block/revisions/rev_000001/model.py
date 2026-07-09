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
# Snatch-block pivot: hinge rivet near the top of the cheek plate, just below the
# attachment eye.  The swing plate part frame sits here so the cheek swings open
# around this point.
PIVOT_Z = 0.058


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
    """One pear-shaped side cheek plate, extruded before rotation into the XZ plane."""
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
    # Top oval is the rope/carabiner attachment eye; lower hole is the axle bore in the plates.
    top_slot = _translated_profile(rounded_rect_profile(0.018, 0.009, 0.0045, corner_segments=8), 0.0, 0.044)
    axle_hole = _circle_profile(0.0, 0.0, 0.0063, segments=40)
    return ExtrudeWithHolesGeometry(
        outer,
        [top_slot, axle_hole],
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="small_rope_pulley",
        meta={
            "run_notes": (
                "Single compact rope pulley modeled from the provided reference lineup; "
                "printed labels/catalog text were ignored and no category conflict was apparent."
            )
        },
    )

    orange = model.material("anodized_orange", rgba=(0.95, 0.30, 0.08, 1.0))
    steel = model.material("brushed_stainless", rgba=(0.72, 0.70, 0.66, 1.0))
    dark = model.material("dark_shadow", rgba=(0.03, 0.035, 0.04, 1.0))
    # Hi-vis accent for the swinging cheek — common on snatch blocks so the
    # movable side is instantly readable in the field.
    hi_vis = model.material("hi_vis_accent", rgba=(0.15, 0.85, 0.20, 1.0))

    # ── frame (fixed cheek + axle + spacers) ────────────────────────────────
    frame = model.part("frame")
    frame.visual(
        mesh_from_geometry(_side_plate_geometry(), "front_plate_mesh"),
        origin=Origin(xyz=(0.0, PLATE_Y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=orange,
        name="front_plate",
    )
    # The axle is fixed to the side plates; the sheave rotates on its bore around it.
    frame.visual(
        Cylinder(radius=AXLE_RADIUS, length=0.029),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="axle_pin",
    )
    # Front-side axle head cap (peened or flared onto the fixed plate).
    frame.visual(
        Cylinder(radius=0.0084, length=0.0024),
        origin=Origin(xyz=(0.0, PLATE_Y + PLATE_THICKNESS / 2.0 + 0.0012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="front_axle_cap",
    )
    # Dark spacer pads between the cheek plates, like the black separating
    # layer in the reference hardware.  They stay on the fixed frame.
    for name, x in (("edge_spacer_0", 0.0186), ("edge_spacer_1", -0.0186)):
        frame.visual(
            Cylinder(radius=0.0022, length=0.019),
            origin=Origin(xyz=(x, 0.0, -0.015), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark,
            name=name,
        )
    # Hinge boss on the frame side — a small cylindrical rivet head at the
    # pivot location that visually anchors the swinging cheek.
    frame.visual(
        Cylinder(radius=0.004, length=0.004),
        origin=Origin(xyz=(0.0, PLATE_Y + PLATE_THICKNESS / 2.0 + 0.002, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_boss_front",
    )

    # ── swing_plate (opening cheek — snatch-block movable side) ─────────────
    # Part frame sits at the pivot point so the revolute joint rotates the
    # entire cheek around the top hinge line.
    swing_plate = model.part("swing_plate")
    # Plate geometry offset: the plate center was at world (0, -PLATE_Y, 0);
    # relative to the part frame at (0, -PLATE_Y, PIVOT_Z) that is (0, 0, -PIVOT_Z).
    swing_plate.visual(
        mesh_from_geometry(_side_plate_geometry(), "rear_plate_mesh"),
        origin=Origin(xyz=(0.0, 0.0, -PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hi_vis,
        name="rear_plate",
    )
    # Rear axle head cap — rides with the swinging cheek.
    swing_plate.visual(
        Cylinder(radius=0.0084, length=0.0024),
        origin=Origin(xyz=(0.0, -PLATE_THICKNESS / 2.0 - 0.0012, -PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rear_axle_cap",
    )
    # Hinge boss on the swing side — mates with the frame boss at the pivot.
    swing_plate.visual(
        Cylinder(radius=0.004, length=0.004),
        origin=Origin(xyz=(0.0, -PLATE_THICKNESS / 2.0 - 0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_boss_rear",
    )

    # ── sheave (continuous-spin grooved rope wheel) ─────────────────────────
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

    # ── articulations ───────────────────────────────────────────────────────
    model.articulation(
        "frame_to_sheave",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=sheave,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=20.0),
    )
    # Snatch-block cheek hinge: the swing plate pivots open around the top
    # hinge line (axis along X at the pivot).  axis=(-1,0,0) so that positive q
    # swings the lower portion of the cheek outward (toward -Y), exposing the
    # sheave groove for mid-line rope insertion.
    model.articulation(
        "frame_to_swing_plate",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=swing_plate,
        origin=Origin(xyz=(0.0, -PLATE_Y, PIVOT_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=math.pi / 2.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    swing_plate = object_model.get_part("swing_plate")
    sheave = object_model.get_part("sheave")
    spin = object_model.get_articulation("frame_to_sheave")
    swing = object_model.get_articulation("frame_to_swing_plate")

    # Sheave containment on the fixed frame side
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
    # Fixed front plate clearance
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
    # Swinging cheek clearance at rest (closed)
    ctx.expect_gap(
        sheave,
        swing_plate,
        axis="y",
        positive_elem="grooved_sheave",
        negative_elem="rear_plate",
        min_gap=0.001,
        max_gap=0.003,
        name="swing_plate rear_plate clears the spinning sheave when closed",
    )

    # Continuous sheave spin proof
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

    # ── snatch-block cheek swing proof ──────────────────────────────────────
    # The swing_plate must pivot outward (toward -Y) when the hinge opens,
    # exposing the sheave groove for mid-line rope insertion.
    closed_rear_y = ctx.part_element_world_aabb(swing_plate, elem="rear_plate")
    with ctx.pose({swing: math.pi / 2.0}):
        open_rear_y = ctx.part_element_world_aabb(swing_plate, elem="rear_plate")

    def _min_y(aabb):
        return aabb[0][1] if aabb is not None else None

    closed_min_y = _min_y(closed_rear_y)
    open_min_y = _min_y(open_rear_y)
    ctx.check(
        "frame_to_swing_plate opens the cheek outward (toward -Y)",
        closed_min_y is not None
        and open_min_y is not None
        and open_min_y < closed_min_y - 0.005,
        details=f"closed_min_y={closed_min_y}, open_min_y={open_min_y}",
    )

    # At full open, the swinging cheek's main body is well clear of the sheave
    # in Y — the plate center has moved far toward -Y, exposing the groove.
    closed_plate_aabb = ctx.part_element_world_aabb(swing_plate, elem="rear_plate")
    with ctx.pose({swing: math.pi / 2.0}):
        open_plate_aabb = ctx.part_element_world_aabb(swing_plate, elem="rear_plate")

    def _center_y(aabb):
        if aabb is None:
            return None
        return (aabb[0][1] + aabb[1][1]) * 0.5

    closed_plate_cy = _center_y(closed_plate_aabb)
    open_plate_cy = _center_y(open_plate_aabb)
    ctx.check(
        "open swing_plate rear_plate exposes the sheave groove",
        closed_plate_cy is not None
        and open_plate_cy is not None
        and open_plate_cy < closed_plate_cy - 0.030,
        details=f"closed_center_y={closed_plate_cy}, open_center_y={open_plate_cy}",
    )

    return ctx.report()


object_model = build_object_model()
