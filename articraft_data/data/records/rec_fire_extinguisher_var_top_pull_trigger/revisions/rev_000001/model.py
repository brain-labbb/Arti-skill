from __future__ import annotations

# Red portable fire extinguisher with push-down trigger actuator.
#
# Body axis is +Z. Base ring rests at z=0. From the bottom up:
#   - recessed base ring / foot
#   - red cylindrical bottle with two rolled banding rings and a label band
#   - a domed red shoulder
#   - a brass valve neck
#   - the operating head: a fixed carry handle (lower bar), a cylindrical
#     trigger guide boss, and a push-down trigger button on top that slides
#     straight down along the vertical body axis to actuate the valve
#   - a round pressure gauge on the front-facing side of the valve
#   - a pull safety pin with ring through the trigger guide boss
#   - a short discharge hose + nozzle clipped along the side
#
# Primary articulation: the push trigger (PRISMATIC along vertical Z axis);
# positive q pushes the button down toward the valve body to open the valve.

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
    # ROOT: body (base ring + cylinder + banding + dome + valve neck)
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
    bottle_profile = [(0.0, 0.020)]
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

    # --- valve head block: a small brass body that carries the trigger ---
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

    # --- trigger guide boss: a cylindrical collar on top of the valve head
    #     that the trigger stem slides through ---
    guide_z = head_z + 0.022  # top of valve head
    guide_height = 0.018
    guide = LatheGeometry(
        [
            (0.0, 0.0),
            (0.018, 0.0),
            (0.018, guide_height - 0.003),
            (0.016, guide_height),
            (0.0, guide_height),
        ],
        segments=24,
    )
    body.visual(
        mesh_from_geometry(guide, "trigger_guide"),
        origin=Origin(xyz=(0.0, 0.0, guide_z)),
        material=brass,
        name="trigger_guide",
    )

    # --- fixed carry handle (lower grab bar, extends back over the dome) ---
    carry_profile = rounded_rect_profile(0.030, 0.010, 0.004)
    carry_pts = [
        (-0.020, 0.0, head_z + 0.010),
        (-0.042, 0.0, head_z + 0.026),
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
    gauge_stem = Cylinder(radius=0.005, length=0.034)
    body.visual(
        gauge_stem,
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
    gauge_dial = Cylinder(radius=0.014, length=0.004)
    body.visual(
        gauge_dial,
        origin=Origin(xyz=(0.0, -0.054, gauge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gauge_face,
        name="gauge_dial",
    )

    # --- safety pin + pull ring through the trigger guide boss ---
    # The pin runs along Y through the guide boss (so it blocks the trigger
    # stem from moving down) and the pull ring hangs off the +Y end.
    safety_z = guide_z + guide_height * 0.5
    safety_pin_len = 0.056
    safety_pin = Cylinder(radius=0.0022, length=safety_pin_len)
    body.visual(
        safety_pin,
        origin=Origin(xyz=(0.0, 0.0, safety_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="safety_pin",
    )
    # Pull ring links onto the protruding +Y end of the pin.
    ring_r = 0.014
    pin_end_y = safety_pin_len / 2.0
    ring = TorusGeometry(ring_r, 0.0028, radial_segments=12, tubular_segments=24)
    body.visual(
        mesh_from_geometry(ring, "pull_ring"),
        origin=Origin(xyz=(0.0, pin_end_y - 0.003, safety_z - ring_r), rpy=(math.pi / 2.0, 0.0, 0.0)),
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
        [(0.0, 0.0), (0.008, 0.0), (0.012, 0.024), (0.018, 0.040), (0.006, 0.040), (0.0, 0.024)],
        segments=24,
    )
    body.visual(
        mesh_from_geometry(nozzle, "discharge_nozzle"),
        origin=Origin(xyz=(body_r + 0.006, -0.004, 0.150), rpy=(0.0, math.pi, 0.0)),
        material=black,
        name="discharge_nozzle",
    )

    # ===================================================================
    # PUSH-DOWN TRIGGER BUTTON (prismatic along vertical Z axis)
    # ===================================================================
    # The trigger part frame sits at the top of the trigger guide boss
    # (the contact surface). At q=0, the trigger is at rest (raised).
    # Positive q pushes the button down (-Z) to actuate the valve.
    trigger_top_z = guide_z + guide_height  # top of guide boss
    trigger = model.part("trigger")

    # Trigger button cap: a rounded cylindrical push-button.
    # Authored in the joint-local frame (origin at top of guide boss).
    # The cap extends upward (+Z) from origin; the stem goes down (-Z).
    cap_r = 0.016
    cap_h = 0.012
    stem_r = 0.007
    stem_len = 0.022  # stem reaches down into the guide boss

    # Button cap: a lathe profile with a slightly domed top.
    trigger_cap = LatheGeometry(
        [
            (0.0, 0.0),
            (stem_r, 0.0),
            (cap_r - 0.002, 0.001),
            (cap_r, 0.003),
            (cap_r, cap_h - 0.003),
            (cap_r - 0.002, cap_h - 0.001),
            (cap_r * 0.6, cap_h + 0.002),
            (0.0, cap_h + 0.003),
        ],
        segments=28,
    )
    trigger.visual(
        mesh_from_geometry(trigger_cap, "trigger_cap"),
        material=red,
        name="trigger_cap",
    )

    # Trigger stem: goes down into the guide boss bore.
    trigger_stem = Cylinder(radius=stem_r, length=stem_len)
    trigger.visual(
        trigger_stem,
        origin=Origin(xyz=(0.0, 0.0, -stem_len / 2.0)),
        material=steel,
        name="trigger_stem",
    )

    # Grip ribs on the button top (raised radial lines for thumb traction).
    for i in range(6):
        angle = i * math.pi / 3.0
        rib_x = 0.008 * math.cos(angle)
        rib_y = 0.008 * math.sin(angle)
        rib = Cylinder(radius=0.001, length=0.003)
        trigger.visual(
            rib,
            origin=Origin(
                xyz=(rib_x, rib_y, cap_h + 0.002),
                rpy=(0.0, 0.0, 0.0),
            ),
            material=red,
            name=f"grip_rib_{i}",
        )

    trigger.inertial = Inertial.from_geometry(
        Cylinder(radius=cap_r, length=cap_h + stem_len), mass=0.05
    )

    model.articulation(
        "body_to_trigger",
        ArticulationType.PRISMATIC,
        parent=body,
        child=trigger,
        # Origin at the top of the guide boss (contact surface).
        origin=Origin(xyz=(0.0, 0.0, trigger_top_z)),
        # Push down: positive q translates along -Z.
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=0.5, lower=0.0, upper=0.012),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    trigger = object_model.get_part("trigger")
    trigger_joint = object_model.get_articulation("body_to_trigger")

    # --- Base ring rests on the ground (z=0), nothing buried/floating ---
    bb = ctx.part_world_aabb(body)
    assert bb is not None
    ctx.check("base_at_ground", abs(bb[0][2]) < 0.004, details=f"body min z={bb[0][2]:.4f}")

    # --- Bottle is taller than wide (upright cylinder) ---
    width = bb[1][0] - bb[0][0]
    height = bb[1][2] - bb[0][2]
    ctx.check("upright_bottle", height > width * 1.6, details=f"h={height:.3f}, w={width:.3f}")

    # --- Pressure gauge present and on the visual front (-Y) ---
    gauge = ctx.part_element_world_aabb(body, elem="gauge_dial")
    assert gauge is not None
    gauge_y = (gauge[0][1] + gauge[1][1]) / 2
    gauge_z_mid = (gauge[0][2] + gauge[1][2]) / 2
    ctx.check("gauge_on_front", gauge_y < -0.03, details=f"gauge y={gauge_y:.3f}")
    ctx.check("gauge_height_unchanged", 0.440 < gauge_z_mid < 0.465, details=f"gauge z={gauge_z_mid:.3f}")

    # --- Label band wraps the body at mid-height ---
    label = ctx.part_element_world_aabb(body, elem="label_band")
    assert label is not None
    ctx.check(
        "label_mid_body",
        0.08 < (label[0][2] + label[1][2]) / 2 < 0.30,
        details=f"label mid z={(label[0][2]+label[1][2])/2:.3f}",
    )

    # --- Trigger is a PRISMATIC joint along the vertical axis ---
    assert trigger_joint.articulation_type == ArticulationType.PRISMATIC, (
        "trigger must be prismatic"
    )
    ctx.check(
        "trigger_axis_vertical",
        abs(trigger_joint.axis[2]) > 0.99,
        details=f"trigger axis={trigger_joint.axis}",
    )

    # --- Trigger sits above the valve body and pushes down when actuated ---
    rest_cap = ctx.part_element_world_aabb(trigger, elem="trigger_cap")
    assert rest_cap is not None
    rest_cap_z = (rest_cap[0][2] + rest_cap[1][2]) / 2.0

    with ctx.pose({trigger_joint: trigger_joint.motion_limits.upper}):
        pushed_cap = ctx.part_element_world_aabb(trigger, elem="trigger_cap")
        assert pushed_cap is not None
        pushed_cap_z = (pushed_cap[0][2] + pushed_cap[1][2]) / 2.0

    ctx.check(
        "trigger_pushes_down",
        pushed_cap_z < rest_cap_z - 0.005,
        details=f"rest cap z={rest_cap_z:.4f}, pushed cap z={pushed_cap_z:.4f}",
    )

    # --- Trigger cap sits above the guide boss top at rest ---
    guide = ctx.part_element_world_aabb(body, elem="trigger_guide")
    assert guide is not None
    guide_top_z = guide[1][2]
    ctx.check(
        "trigger_above_guide_at_rest",
        rest_cap_z > guide_top_z - 0.002,
        details=f"rest cap z={rest_cap_z:.4f}, guide top z={guide_top_z:.4f}",
    )

    # --- Trigger stem is captured inside the guide boss (intentional overlap) ---
    ctx.allow_overlap(
        trigger, body,
        elem_a="trigger_stem", elem_b="trigger_guide",
        reason="The trigger stem slides inside the guide boss bore (captured shaft).",
    )
    ctx.expect_within(
        trigger, body,
        axes="xy",
        inner_elem="trigger_stem",
        outer_elem="trigger_guide",
        margin=0.002,
        name="trigger stem stays centered in guide",
    )

    # --- Trigger contacts the body assembly at rest (seated on guide) ---
    ctx.expect_contact(
        trigger, body,
        contact_tol=0.005,
        name="trigger_seated_on_guide",
    )

    # --- Carry handle sits below the trigger (trigger is the topmost control) ---
    carry = ctx.part_element_world_aabb(body, elem="carry_handle")
    assert carry is not None
    carry_mid_z = (carry[0][2] + carry[1][2]) / 2
    ctx.check(
        "trigger_above_carry_handle",
        rest_cap_z > carry_mid_z,
        details=f"trigger cap z={rest_cap_z:.3f}, carry mid z={carry_mid_z:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
