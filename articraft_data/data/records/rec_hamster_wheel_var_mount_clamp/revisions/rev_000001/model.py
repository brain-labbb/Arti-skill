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
    chrome = model.material("chrome_clamp_finish", rgba=(0.78, 0.80, 0.83, 1.0))

    # ── Stand: cage-mount clamp bracket with cantilever arm ──
    stand = model.part("stand")

    # Vertical clamp plate — presses against the outside of a cage wall/bar.
    plate_cq = (
        cq.Workplane("XY")
        .box(0.006, 0.052, 0.082)
        .edges("|Z")
        .fillet(0.002)
        .translate((-0.100, 0.0, AXLE_HEIGHT - 0.005))
    )
    stand.visual(
        mesh_from_cadquery(plate_cq, "clamp_plate", tolerance=0.0008, angular_tolerance=0.05),
        material=chrome,
        name="clamp_plate",
    )

    # Upper jaw with a downward hook lip — hooks over a cage bar from above.
    upper_jaw_z = AXLE_HEIGHT + 0.032
    jaw_upper_body = (
        cq.Workplane("XY")
        .box(0.022, 0.042, 0.007)
        .edges("|Y")
        .fillet(0.002)
    )
    hook_lip = cq.Workplane("XY").box(0.005, 0.042, 0.014).translate(
        (-0.0085, 0.0, -0.0105)
    )
    jaw_upper = jaw_upper_body.union(hook_lip).translate((-0.113, 0.0, upper_jaw_z))
    stand.visual(
        mesh_from_cadquery(jaw_upper, "clamp_jaw_upper", tolerance=0.0008, angular_tolerance=0.05),
        material=chrome,
        name="clamp_jaw_upper",
    )

    # Lower jaw with an upward grip lip — presses under the cage bar from below.
    lower_jaw_z = AXLE_HEIGHT - 0.038
    jaw_lower_body = (
        cq.Workplane("XY")
        .box(0.022, 0.042, 0.007)
        .edges("|Y")
        .fillet(0.002)
    )
    grip_lip = cq.Workplane("XY").box(0.005, 0.042, 0.010).translate(
        (-0.0085, 0.0, 0.0085)
    )
    jaw_lower = jaw_lower_body.union(grip_lip).translate((-0.113, 0.0, lower_jaw_z))
    stand.visual(
        mesh_from_cadquery(jaw_lower, "clamp_jaw_lower", tolerance=0.0008, angular_tolerance=0.05),
        material=chrome,
        name="clamp_jaw_lower",
    )

    # Horizontal cantilever mount arm — extends from the clamp plate inward to
    # carry the fixed axle and rear bearing.  Includes a vertical brace at the
    # plate junction for structural rigidity.
    arm_start_x = -0.100 + 0.003  # plate +X face at -0.097
    arm_end_x = -0.050
    arm_length = arm_end_x - arm_start_x + 0.002  # includes plate overlap
    arm_center_x = (arm_start_x - 0.001 + arm_end_x) / 2.0

    arm_beam = (
        cq.Workplane("XY")
        .box(arm_length, 0.020, 0.016)
        .edges("|Z")
        .fillet(0.002)
    )
    brace_x_local = -(arm_length / 2.0) + 0.002
    brace = (
        cq.Workplane("XY")
        .box(0.006, 0.020, 0.028)
        .translate((brace_x_local, 0.0, 0.008 + 0.014))
    )
    arm_cq = arm_beam.union(brace).translate((arm_center_x, 0.0, AXLE_HEIGHT))
    stand.visual(
        mesh_from_cadquery(arm_cq, "mount_arm", tolerance=0.0008, angular_tolerance=0.05),
        material=white_metal,
        name="mount_arm",
    )

    # Fixed axle shaft — cantilevered from the mount arm through the wheel hub.
    stand.visual(
        Cylinder(radius=0.006, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, AXLE_HEIGHT), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white_metal,
        name="axle_shaft",
    )
    # Front axle cap — retainer on the open side of the wheel.
    stand.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(
            xyz=(WHEEL_WIDTH / 2.0 + 0.006, 0.0, AXLE_HEIGHT),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=pale_bearing,
        name="front_axle_cap",
    )
    # Rear bearing — sits between the mount arm end and the wheel hub bore.
    stand.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(
            xyz=(-WHEEL_WIDTH / 2.0 - 0.011, 0.0, AXLE_HEIGHT),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=pale_bearing,
        name="rear_bearing",
    )

    # ── Wheel (identical to parent) ──
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

    # Clear raised front/back lips emphasize the thick transparent rim visible
    # in the reference and give the otherwise open drum a manufactured edge.
    wheel.visual(
        mesh_from_cadquery(
            _make_annular_ring(
                WHEEL_RADIUS + 0.002, WHEEL_RADIUS - 0.010, 0.006, WHEEL_WIDTH / 2.0 - 0.001
            ),
            "front_rim_lip",
            tolerance=0.0008,
            angular_tolerance=0.05,
        ),
        material=clear_edge,
        name="front_rim_lip",
    )
    wheel.visual(
        mesh_from_cadquery(
            _make_annular_ring(
                WHEEL_RADIUS + 0.001, WHEEL_RADIUS - 0.011, 0.005, -WHEEL_WIDTH / 2.0 - 0.004
            ),
            "rear_rim_lip",
            tolerance=0.0008,
            angular_tolerance=0.05,
        ),
        material=clear_edge,
        name="rear_rim_lip",
    )

    # Axial raised ribs around the outside of the drum, like the molded grip
    # and stiffening marks around the reference wheel's transparent rim.
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

    # ── Articulation (preserved revolute X spin) ──
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

    # ── Spin joint preserved ──
    ctx.check(
        "single spinning wheel joint",
        spin.articulation_type == ArticulationType.REVOLUTE
        and tuple(spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )

    # ── Cage-clamp cantilever bracket structure (the changed axis) ──
    plate_aabb = ctx.part_element_world_aabb(stand, elem="clamp_plate")
    arm_aabb = ctx.part_element_world_aabb(stand, elem="mount_arm")
    ctx.check(
        "clamp_plate visual present on cage-mount stand",
        plate_aabb is not None,
        details="stand must have a clamp_plate for cage-bar mounting",
    )
    ctx.check(
        "mount_arm cantilevers from clamp bracket to axle area",
        arm_aabb is not None
        and arm_aabb[0][0] < -0.080
        and arm_aabb[1][0] > -0.060,
        details=f"arm AABB={arm_aabb}" if arm_aabb else "mount_arm missing",
    )

    # ── Axle–wheel interface preserved ──
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

    # ── Wheel spins correctly when posed ──
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
        c0 is not None
        and c1 is not None
        and abs(c1[1] - c0[1]) > 0.050
        and c1[2] < c0[2] - 0.020,
        details=f"rest={c0}, spun={c1}",
    )

    return ctx.report()


object_model = build_object_model()
