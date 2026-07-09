from __future__ import annotations

# Red portable fire extinguisher with a screw-down wheel valve.
#
# Body axis is +Z. Base ring rests at z=0. From the bottom up:
#   - recessed base ring / foot
#   - red cylindrical bottle with two rolled banding rings and a label band
#   - a domed red shoulder
#   - a brass valve neck and valve head block
#   - a short brass valve stem
#   - a round spoked hand-wheel (revolute, Z axis) that turns to open/close
#   - a fixed carry handle strap extending back from the valve head
#   - a round pressure gauge on the front of the valve
#   - a pull safety pin with ring through the valve head
#   - a short discharge hose with nozzle clipped to the side
#
# Primary articulation: the hand-wheel (revolute about the vertical Z axis);
# positive rotation turns the wheel to open the screw-down valve.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fire_extinguisher")

    red = model.material("body_red", rgba=(0.86, 0.20, 0.13, 1.0))
    brass = model.material("brass", rgba=(0.80, 0.60, 0.20, 1.0))
    steel = model.material("steel", rgba=(0.66, 0.67, 0.70, 1.0))
    label_white = model.material("label_white", rgba=(0.92, 0.92, 0.90, 1.0))
    black = model.material("rubber_black", rgba=(0.12, 0.12, 0.13, 1.0))
    gauge_face = model.material("gauge_face", rgba=(0.20, 0.55, 0.30, 1.0))

    # ---------- key dimensions ----------
    body_r = 0.056
    shoulder_z = 0.330       # where the cylinder ends and the dome begins
    dome_top_z = 0.400       # top of the dome
    neck_top_z = 0.440       # top of the brass valve neck

    # ===================================================================
    # ROOT: body (base ring + cylinder + banding + dome + valve assembly)
    # ===================================================================
    body = model.part("body")

    # Base ring / foot: a rolled ring the bottle stands on, recessed center.
    base_ring = LatheGeometry(
        [
            (0.0, 0.012),
            (body_r - 0.006, 0.012),
            (body_r, 0.004),
            (body_r, 0.0),
            (body_r - 0.004, 0.0),
            (body_r - 0.010, 0.020),
            (0.0, 0.022),
        ],
        segments=48,
    )
    body.visual(
        mesh_from_geometry(base_ring, "base_ring"),
        material=red,
        name="base_ring",
    )

    # Main cylinder bottle with two rolled banding rings (top and bottom).
    pts = [(body_r, 0.020)]
    for bz in (0.060, 0.300):  # rolled banding rings
        pts.append((body_r, bz - 0.012))
        pts.append((body_r + 0.004, bz - 0.004))
        pts.append((body_r + 0.004, bz + 0.004))
        pts.append((body_r, bz + 0.012))
    pts.append((body_r, shoulder_z))
    bottle_profile = [(0.0, 0.020)] + pts + [(0.0, shoulder_z)]
    bottle = LatheGeometry(bottle_profile, segments=48)
    body.visual(
        mesh_from_geometry(bottle, "bottle"),
        material=red,
        name="bottle",
    )

    # Domed red shoulder.
    dome = LatheGeometry(
        [
            (0.0, shoulder_z),
            (body_r, shoulder_z),
            (body_r - 0.004, shoulder_z + 0.018),
            (body_r * 0.78, shoulder_z + 0.045),
            (body_r * 0.42, dome_top_z - 0.004),
            (0.022, dome_top_z),
            (0.0, dome_top_z),
        ],
        segments=48,
    )
    body.visual(
        mesh_from_geometry(dome, "shoulder_dome"),
        material=red,
        name="shoulder_dome",
    )

    # Brass valve neck.
    neck = LatheGeometry(
        [
            (0.0, dome_top_z),
            (0.024, dome_top_z),
            (0.024, dome_top_z + 0.012),
            (0.020, dome_top_z + 0.014),
            (0.020, neck_top_z),
            (0.0, neck_top_z),
        ],
        segments=32,
    )
    body.visual(
        mesh_from_geometry(neck, "valve_neck"),
        material=brass,
        name="valve_neck",
    )

    # Label band wrapping the cylinder (a thin white sleeve over the body).
    label = LatheGeometry(
        [
            (body_r + 0.0008, 0.110),
            (body_r + 0.0012, 0.118),
            (body_r + 0.0012, 0.270),
            (body_r + 0.0008, 0.278),
        ],
        segments=48,
    )
    body.visual(
        mesh_from_geometry(label, "label_band"),
        material=label_white,
        name="label_band",
    )

    body.inertial = Inertial.from_geometry(
        Cylinder(radius=body_r, length=dome_top_z), mass=4.5
    )

    # --- valve head block: brass body that carries the stem and handle ---
    head_z = neck_top_z
    head = LatheGeometry(
        [
            (0.0, 0.0),
            (0.020, 0.0),
            (0.022, 0.008),
            (0.020, 0.022),
            (0.0, 0.022),
        ],
        segments=24,
    )
    body.visual(
        mesh_from_geometry(head, "valve_head"),
        origin=Origin(xyz=(0.0, 0.0, head_z)),
        material=brass,
        name="valve_head",
    )

    # --- valve stem (short brass cylinder the hand-wheel mounts on) ---
    stem_h = 0.010
    stem_r = 0.008
    body.visual(
        Cylinder(radius=stem_r, length=stem_h),
        origin=Origin(xyz=(0.0, 0.0, head_z + 0.022 + stem_h / 2.0)),
        material=brass,
        name="valve_stem",
    )

    # --- fixed carry handle (strap extending back over the dome) ---
    carry_profile = rounded_rect_profile(0.030, 0.010, 0.004)
    carry_pts = [
        (0.0, 0.0, head_z + 0.014),
        (-0.040, 0.0, head_z + 0.026),
        (-0.080, 0.0, head_z + 0.030),
        (-0.110, 0.0, head_z + 0.022),
    ]
    carry = sweep_profile_along_spline(
        carry_pts, profile=carry_profile, samples_per_segment=10, cap_profile=True
    )
    body.visual(
        mesh_from_geometry(carry, "carry_handle"),
        material=red,
        name="carry_handle",
    )

    # --- pressure gauge on the front-facing side of the valve ---
    gauge_z = head_z + 0.008
    body.visual(
        Cylinder(radius=0.005, length=0.034),
        origin=Origin(xyz=(0.0, -0.030, gauge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="gauge_stem",
    )
    gauge_case = LatheGeometry(
        [
            (0.0, 0.0),
            (0.018, 0.0),
            (0.018, 0.012),
            (0.016, 0.014),
            (0.0, 0.014),
        ],
        segments=28,
    )
    body.visual(
        mesh_from_geometry(gauge_case, "gauge_case"),
        origin=Origin(xyz=(0.0, -0.042, gauge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="gauge_case",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(0.0, -0.054, gauge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gauge_face,
        name="gauge_dial",
    )

    # --- safety pin + pull ring through the valve head ---
    # The pin runs along Y through the valve head body, preventing the
    # hand-wheel from turning when inserted.
    safety_z = head_z + 0.012
    pin_length = 0.080
    body.visual(
        Cylinder(radius=0.0022, length=pin_length),
        origin=Origin(xyz=(0.0, 0.0, safety_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="safety_pin",
    )
    ring_r = 0.014
    pin_end_y = pin_length / 2.0
    ring_geom = TorusGeometry(ring_r, 0.0028, radial_segments=12, tubular_segments=24)
    body.visual(
        mesh_from_geometry(ring_geom, "pull_ring"),
        origin=Origin(
            xyz=(0.0, pin_end_y - 0.003, safety_z - ring_r),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="pull_ring",
    )

    # --- discharge hose + nozzle clipped to the side ---
    hose_pts = [
        (0.018, 0.0, head_z + 0.004),
        (0.050, 0.020, head_z - 0.040),
        (0.062, 0.010, shoulder_z - 0.020),
        (body_r + 0.010, -0.010, 0.220),
        (body_r + 0.006, -0.004, 0.150),
    ]
    hose = tube_from_spline_points(
        hose_pts, radius=0.006, samples_per_segment=10, radial_segments=10
    )
    body.visual(
        mesh_from_geometry(hose, "discharge_hose"),
        material=black,
        name="discharge_hose",
    )
    nozzle = LatheGeometry(
        [
            (0.0, 0.0),
            (0.008, 0.0),
            (0.012, 0.024),
            (0.018, 0.040),
            (0.006, 0.040),
            (0.0, 0.024),
        ],
        segments=24,
    )
    body.visual(
        mesh_from_geometry(nozzle, "discharge_nozzle"),
        origin=Origin(
            xyz=(body_r + 0.006, -0.004, 0.150), rpy=(0.0, math.pi, 0.0)
        ),
        material=black,
        name="discharge_nozzle",
    )

    # ===================================================================
    # HAND-WHEEL (revolute about the vertical Z axis)
    # Round spoked wheel mounted on top of the valve stem; turning it
    # opens or closes the screw-down valve.
    # ===================================================================
    wheel_r = 0.035       # outer radius of the hand-wheel rim
    hub_r = 0.012         # hub radius
    hub_h = 0.016         # hub height
    rim_tube = 0.0045     # rim tube radius
    spoke_r = 0.003       # spoke cross-section radius
    n_spokes = 5          # number of spokes

    # Joint origin at the top of the valve stem (actual contact surface)
    joint_z = head_z + 0.022 + stem_h

    hand_wheel = model.part("hand_wheel")

    # Hub: central brass cylinder that fits over the valve stem
    hand_wheel.visual(
        Cylinder(radius=hub_r, length=hub_h),
        origin=Origin(xyz=(0.0, 0.0, hub_h / 2.0)),
        material=brass,
        name="hub",
    )

    # Rim: outer ring (torus) at the hub center height
    hand_wheel.visual(
        mesh_from_geometry(
            TorusGeometry(wheel_r, rim_tube, radial_segments=12, tubular_segments=32),
            "rim",
        ),
        origin=Origin(xyz=(0.0, 0.0, hub_h / 2.0)),
        material=red,
        name="rim",
    )

    # Spokes: radial bars connecting hub to rim.
    # Spokes extend slightly into the hub and rim for geometric connectivity.
    spoke_inner_r = hub_r - 0.002
    spoke_outer_r = wheel_r
    spoke_length = spoke_outer_r - spoke_inner_r
    spoke_mid_r = (spoke_inner_r + spoke_outer_r) / 2.0

    for i in range(n_spokes):
        theta = 2.0 * math.pi * i / n_spokes
        hand_wheel.visual(
            Cylinder(radius=spoke_r, length=spoke_length),
            origin=Origin(
                xyz=(
                    spoke_mid_r * math.cos(theta),
                    spoke_mid_r * math.sin(theta),
                    hub_h / 2.0,
                ),
                rpy=(0.0, math.pi / 2.0, theta),
            ),
            material=red,
            name=f"spoke_{i}",
        )

    hand_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_r, length=hub_h), mass=0.15
    )

    model.articulation(
        "body_to_wheel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hand_wheel,
        origin=Origin(xyz=(0.0, 0.0, joint_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0, lower=0.0, upper=4.0 * math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    wheel = object_model.get_part("hand_wheel")
    wheel_joint = object_model.get_articulation("body_to_wheel")

    # --- Base ring rests on the ground (z=0), nothing buried/floating ---
    bb = ctx.part_world_aabb(body)
    assert bb is not None
    ctx.check(
        "base_at_ground",
        abs(bb[0][2]) < 0.004,
        details=f"body min z={bb[0][2]:.4f}",
    )

    # --- Bottle is taller than wide (upright cylinder) ---
    width = bb[1][0] - bb[0][0]
    height = bb[1][2] - bb[0][2]
    ctx.check(
        "upright_bottle",
        height > width * 1.6,
        details=f"h={height:.3f}, w={width:.3f}",
    )

    # --- Pressure gauge present and on the visual front (-Y) ---
    gauge = ctx.part_element_world_aabb(body, elem="gauge_dial")
    assert gauge is not None
    gauge_y = (gauge[0][1] + gauge[1][1]) / 2
    gauge_z_mid = (gauge[0][2] + gauge[1][2]) / 2
    ctx.check(
        "gauge_on_front",
        gauge_y < -0.03,
        details=f"gauge y={gauge_y:.3f}",
    )
    ctx.check(
        "gauge_height_unchanged",
        0.440 < gauge_z_mid < 0.465,
        details=f"gauge z={gauge_z_mid:.3f}",
    )

    # --- Label band wraps the body at mid-height ---
    label = ctx.part_element_world_aabb(body, elem="label_band")
    assert label is not None
    ctx.check(
        "label_mid_body",
        0.08 < (label[0][2] + label[1][2]) / 2 < 0.30,
        details=f"label mid z={(label[0][2] + label[1][2]) / 2:.3f}",
    )

    # --- Hand-wheel axis is vertical Z (the changed mechanism) ---
    ctx.check(
        "wheel_axis_is_vertical_z",
        abs(wheel_joint.axis[2]) > 0.99,
        details=f"axis={wheel_joint.axis}",
    )

    # --- Hand-wheel sits above the valve head ---
    wheel_bb = ctx.part_world_aabb(wheel)
    assert wheel_bb is not None
    ctx.check(
        "wheel_above_valve_head",
        wheel_bb[0][2] > 0.457,
        details=f"wheel min z={wheel_bb[0][2]:.4f}",
    )

    # --- Hand-wheel is round (diameter >> height, reads as a wheel) ---
    wheel_dx = wheel_bb[1][0] - wheel_bb[0][0]
    wheel_dz = wheel_bb[1][2] - wheel_bb[0][2]
    ctx.check(
        "wheel_is_round",
        wheel_dx > wheel_dz * 2.0,
        details=f"dx={wheel_dx:.4f}, dz={wheel_dz:.4f}",
    )

    # --- Wheel has spoke visuals (spoked wheel, not a solid disk) ---
    spoke0 = ctx.part_element_world_aabb(wheel, elem="spoke_0")
    ctx.check(
        "wheel_has_spokes",
        spoke0 is not None,
        details="spoke_0 visual must exist",
    )

    # --- Wheel has multi-turn rotation range (screw-down valve) ---
    ctx.check(
        "wheel_multi_turn_range",
        wheel_joint.motion_limits.upper > 2.0 * math.pi,
        details=f"upper={wheel_joint.motion_limits.upper:.2f} rad",
    )

    # --- Wheel contacts body at the valve stem (mounted on top) ---
    ctx.expect_contact(
        wheel, body, contact_tol=0.015, name="wheel_on_stem",
    )

    # --- Wheel rim fits within the bottle footprint on XY ---
    ctx.expect_within(
        wheel, body,
        axes="xy",
        inner_elem="rim",
        outer_elem="bottle",
        margin=0.005,
        name="wheel within bottle footprint",
    )

    return ctx.report()


object_model = build_object_model()
