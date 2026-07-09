from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


WHEEL_RADIUS = 0.145
WHEEL_WIDTH = 0.090
AXLE_HEIGHT = 0.180


def _make_wheel_body() -> object:
    """Translucent plastic running drum with a slotted rear disk."""

    outer_r = WHEEL_RADIUS
    inner_r = 0.132
    back_t = 0.006
    axle_clearance_r = 0.012
    hub_outer_r = 0.034

    # Main open running drum: a thin annular tube whose axis is local X.
    drum = (
        cq.Workplane("YZ")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(WHEEL_WIDTH)
        .translate((-WHEEL_WIDTH / 2.0, 0.0, 0.0))
    )

    # Rear panel with a central axle clearance and five rounded ventilation slots,
    # echoing the petal-like openings in the reference wheel.
    rear_panel = (
        cq.Workplane("YZ")
        .circle(inner_r)
        .circle(axle_clearance_r)
        .extrude(back_t)
        .translate((-WHEEL_WIDTH / 2.0, 0.0, 0.0))
    )

    for i in range(5):
        theta = 2.0 * math.pi * i / 5.0 + math.radians(12.0)
        slot_mid_r = 0.084
        slot_y = slot_mid_r * math.cos(theta)
        slot_z = slot_mid_r * math.sin(theta)
        tangent_deg = math.degrees(theta + math.pi / 2.0)
        cutter = (
            cq.Workplane("YZ")
            .center(slot_y, slot_z)
            .slot2D(0.060, 0.018, angle=tangent_deg)
            .extrude(back_t + 0.014)
            .translate((-WHEEL_WIDTH / 2.0 - 0.007, 0.0, 0.0))
        )
        rear_panel = rear_panel.cut(cutter)

    # Raised bearing boss around the axle clearance, still part of the rotating
    # plastic wheel.  It has a true central bore so the fixed axle does not
    # collide with the spinning wheel.
    hub_ring = (
        cq.Workplane("YZ")
        .circle(hub_outer_r)
        .circle(axle_clearance_r)
        .extrude(WHEEL_WIDTH)
        .translate((-WHEEL_WIDTH / 2.0, 0.0, 0.0))
    )

    return drum.union(rear_panel).union(hub_ring)


def _make_annular_ring(outer_r: float, inner_r: float, width: float, x0: float) -> object:
    """Thin cylindrical lip whose axis is local X and whose bore stays open."""
    return (
        cq.Workplane("YZ")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(width)
        .translate((x0, 0.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="hamster_exercise_wheel",
        meta={
            "reference_note": (
                "The reference image includes a hamster, but this asset models "
                "only the visible core exercise wheel as requested."
            )
        },
    )

    translucent_green = model.material("translucent_green_plastic", rgba=(0.45, 0.70, 0.38, 0.55))
    clear_edge = model.material("clear_rim_plastic", rgba=(0.82, 0.95, 0.82, 0.40))
    white_metal = model.material("white_powder_coated_metal", rgba=(0.93, 0.96, 0.95, 1.0))
    pale_bearing = model.material("white_plastic_bearing", rgba=(0.96, 0.96, 0.92, 1.0))

    stand = model.part("stand")

    base_loop = tube_from_spline_points(
        [
            (-0.082, -0.175, 0.006),
            (0.094, -0.175, 0.006),
            (0.104, 0.175, 0.006),
            (-0.082, 0.175, 0.006),
        ],
        radius=0.0045,
        samples_per_segment=12,
        closed_spline=True,
        radial_segments=18,
    )
    stand.visual(
        mesh_from_geometry(base_loop, "stand_base_loop"),
        material=white_metal,
        name="base_loop",
    )

    for idx, side in enumerate((-1.0, 1.0)):
        yoke = tube_from_spline_points(
            [
                (-0.082, side * 0.175, 0.006),
                (-0.080, side * 0.135, 0.050),
                (-0.072, side * 0.070, 0.125),
                (-0.058, side * 0.014, AXLE_HEIGHT),
                (-0.058, 0.000, AXLE_HEIGHT),
            ],
            radius=0.0042,
            samples_per_segment=10,
            closed_spline=False,
            radial_segments=18,
            cap_ends=True,
        )
        stand.visual(
            mesh_from_geometry(yoke, f"side_support_{idx}"),
            material=white_metal,
            name=f"side_support_{idx}",
        )

    stand.visual(
        Cylinder(radius=0.006, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, AXLE_HEIGHT), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white_metal,
        name="axle_shaft",
    )
    stand.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(WHEEL_WIDTH / 2.0 + 0.006, 0.0, AXLE_HEIGHT), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pale_bearing,
        name="front_axle_cap",
    )
    stand.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(xyz=(-WHEEL_WIDTH / 2.0 - 0.011, 0.0, AXLE_HEIGHT), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pale_bearing,
        name="rear_bearing",
    )

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_cadquery(
            _make_wheel_body(),
            "green_running_drum",
            tolerance=0.0008,
            angular_tolerance=0.05,
        ),
        material=translucent_green,
        name="running_drum",
    )

    # Clear raised front/back lips emphasize the thick transparent rim visible in
    # the reference and give the otherwise open drum a manufactured edge.
    wheel.visual(
        mesh_from_cadquery(
            _make_annular_ring(WHEEL_RADIUS + 0.002, WHEEL_RADIUS - 0.010, 0.006, WHEEL_WIDTH / 2.0 - 0.001),
            "front_rim_lip",
            tolerance=0.0008,
            angular_tolerance=0.05,
        ),
        material=clear_edge,
        name="front_rim_lip",
    )
    wheel.visual(
        mesh_from_cadquery(
            _make_annular_ring(WHEEL_RADIUS + 0.001, WHEEL_RADIUS - 0.011, 0.005, -WHEEL_WIDTH / 2.0 - 0.004),
            "rear_rim_lip",
            tolerance=0.0008,
            angular_tolerance=0.05,
        ),
        material=clear_edge,
        name="rear_rim_lip",
    )

    # Axial raised ribs around the outside of the drum, like the molded grip and
    # stiffening marks around the reference wheel's transparent rim.
    for i in range(18):
        theta = 2.0 * math.pi * i / 18.0
        y = (WHEEL_RADIUS + 0.002) * math.sin(theta)
        z = (WHEEL_RADIUS + 0.002) * math.cos(theta)
        wheel.visual(
            Box((0.080, 0.005, 0.014)),
            origin=Origin(xyz=(0.0, y, z), rpy=(theta, 0.0, 0.0)),
            material=clear_edge,
            name=f"rim_rib_{i}",
        )

    model.articulation(
        "stand_to_wheel",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=25.0,
            lower=-2.0 * math.pi,
            upper=2.0 * math.pi,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    wheel = object_model.get_part("wheel")
    spin = object_model.get_articulation("stand_to_wheel")

    ctx.check(
        "single spinning wheel joint",
        spin.articulation_type == ArticulationType.REVOLUTE and tuple(spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_origin_gap(
        wheel,
        stand,
        axis="z",
        min_gap=AXLE_HEIGHT - 0.001,
        max_gap=AXLE_HEIGHT + 0.001,
        name="wheel origin sits at axle height above stand base",
    )
    ctx.expect_overlap(
        wheel,
        stand,
        axes="x",
        elem_a="running_drum",
        elem_b="axle_shaft",
        min_overlap=0.050,
        name="fixed axle passes through wheel hub span",
    )
    ctx.expect_within(
        stand,
        wheel,
        axes="yz",
        inner_elem="axle_shaft",
        outer_elem="running_drum",
        margin=0.002,
        name="axle is centered in the wheel bore",
    )
    ctx.expect_contact(
        stand,
        wheel,
        elem_a="front_axle_cap",
        elem_b="running_drum",
        contact_tol=0.001,
        name="front cap seats against rotating hub collar",
    )

    closed_aabb = ctx.part_element_world_aabb(wheel, elem="rim_rib_0")
    with ctx.pose({spin: 1.0}):
        spun_aabb = ctx.part_element_world_aabb(wheel, elem="rim_rib_0")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return (
            (lo[0] + hi[0]) / 2.0,
            (lo[1] + hi[1]) / 2.0,
            (lo[2] + hi[2]) / 2.0,
        )

    c0 = _aabb_center(closed_aabb)
    c1 = _aabb_center(spun_aabb)
    ctx.check(
        "rim rib moves around axle when posed",
        c0 is not None and c1 is not None and abs(c1[1] - c0[1]) > 0.050 and c1[2] < c0[2] - 0.020,
        details=f"rest={c0}, spun={c1}",
    )

    return ctx.report()


object_model = build_object_model()
