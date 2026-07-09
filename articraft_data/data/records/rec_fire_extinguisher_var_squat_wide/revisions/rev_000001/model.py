from __future__ import annotations

# Red portable fire extinguisher — squat, wide-bodied (low-and-wide) variant.
#
# Body axis is +Z. Base ring rests at z=0. From the bottom up:
#   - wide recessed base ring / foot
#   - red short, fat cylindrical bottle with two rolled banding rings and a
#     label band (small length-to-diameter ratio, stout stored-pressure unit)
#   - a broad domed red shoulder
#   - a brass valve neck
#   - the operating head: a fixed carry handle (lower bar) and a squeeze
#     operating lever (upper, curved) that pivots down about a rear cross-pin
#   - a round pressure gauge on the front-facing side of the valve
#   - a pull safety pin with a ring through the handle slot
#   - a short discharge hose + nozzle clipped along the side
#
# Primary articulation: the squeeze operating lever (revolute about the rear
# valve cross-pin, horizontal Y axis); squeezing brings its front edge down
# toward the carry handle.

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

    # ---------- key dimensions (squat / low-and-wide) ----------
    body_r = 0.100          # wide 200 mm diameter bottle
    shoulder_z = 0.175      # cylinder ends here (short body)
    dome_top_z = 0.248      # broad dome apex
    neck_top_z = 0.288      # top of the brass valve neck

    # ===================================================================
    # ROOT: body (base ring + cylinder + banding + dome + valve neck)
    # ===================================================================
    body = model.part("body")

    # Wide base ring / foot: a rolled ring the bottle stands on, recessed center.
    base_ring = LatheGeometry(
        [
            (0.0, 0.014),
            (body_r - 0.008, 0.014),
            (body_r + 0.002, 0.005),
            (body_r + 0.002, 0.0),
            (body_r - 0.004, 0.0),
            (body_r - 0.012, 0.024),
            (0.0, 0.026),
        ],
        segments=48,
    )
    body.visual(
        mesh_from_geometry(base_ring, "base_ring"),
        material=red,
        name="base_ring",
    )

    # Main short-fat cylinder bottle with two rolled banding rings.
    bottle_profile_pts = [(body_r, 0.024)]
    for bz in (0.062, shoulder_z - 0.030):  # rolled banding rings
        bottle_profile_pts.append((body_r, bz - 0.012))
        bottle_profile_pts.append((body_r + 0.005, bz - 0.004))
        bottle_profile_pts.append((body_r + 0.005, bz + 0.004))
        bottle_profile_pts.append((body_r, bz + 0.012))
    bottle_profile_pts.append((body_r, shoulder_z))
    bottle_profile = [(0.0, 0.024)] + bottle_profile_pts + [(0.0, shoulder_z)]
    bottle = LatheGeometry(bottle_profile, segments=48)
    body.visual(
        mesh_from_geometry(bottle, "bottle"),
        material=red,
        name="bottle",
    )

    # Broad domed red shoulder — gentle curve over the wide body.
    dome = LatheGeometry(
        [
            (0.0, shoulder_z),
            (body_r, shoulder_z),
            (body_r - 0.006, shoulder_z + 0.020),
            (body_r * 0.78, shoulder_z + 0.046),
            (body_r * 0.44, dome_top_z - 0.005),
            (0.026, dome_top_z),
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
            (0.026, dome_top_z),
            (0.026, dome_top_z + 0.012),
            (0.022, dome_top_z + 0.014),
            (0.022, neck_top_z),
            (0.0, neck_top_z),
        ],
        segments=32,
    )
    body.visual(
        mesh_from_geometry(neck, "valve_neck"),
        material=brass,
        name="valve_neck",
    )

    # Label band wrapping the wide cylinder (thin white sleeve over the body).
    label_z_lo = 0.068
    label_z_hi = 0.160
    label = LatheGeometry(
        [
            (body_r + 0.001, label_z_lo),
            (body_r + 0.0014, label_z_lo + 0.008),
            (body_r + 0.0014, label_z_hi - 0.008),
            (body_r + 0.001, label_z_hi),
        ],
        segments=48,
    )
    body.visual(
        mesh_from_geometry(label, "label_band"),
        material=label_white,
        name="label_band",
    )

    body.inertial = Inertial.from_geometry(
        Cylinder(radius=body_r, length=dome_top_z), mass=6.8
    )

    # --- valve head block: a small brass body that carries the handle & lever ---
    head_z = neck_top_z
    head = LatheGeometry(
        [
            (0.0, 0.0),
            (0.022, 0.0),
            (0.024, 0.008),
            (0.022, 0.024),
            (0.0, 0.024),
        ],
        segments=24,
    )
    body.visual(
        mesh_from_geometry(head, "valve_head"),
        origin=Origin(xyz=(0.0, 0.0, head_z)),
        material=brass,
        name="valve_head",
    )

    # --- fixed carry handle (lower grab bar, extends back over the dome) ---
    carry_profile = rounded_rect_profile(0.032, 0.011, 0.004)
    carry_pts = [
        (0.0, 0.0, head_z + 0.014),
        (-0.042, 0.0, head_z + 0.026),
        (-0.082, 0.0, head_z + 0.030),
        (-0.112, 0.0, head_z + 0.022),
    ]
    carry = sweep_profile_along_spline(
        carry_pts, profile=carry_profile, samples_per_segment=10, cap_profile=True
    )
    body.visual(
        mesh_from_geometry(carry, "carry_handle"),
        material=red,
        name="carry_handle",
    )

    # --- rear pivot lugs + cross-pin for the operating lever ---
    pin_x = -0.014
    lug_base_z = head_z + 0.004
    lug_len = 0.036
    pin_z = lug_base_z + lug_len - 0.008
    pin_dy = 0.017
    for s, tag in ((1, "left"), (-1, "right")):
        lug = Cylinder(radius=0.006, length=lug_len)
        body.visual(
            lug,
            origin=Origin(xyz=(pin_x, s * pin_dy, lug_base_z + lug_len / 2.0)),
            material=brass,
            name=f"lever_lug_{tag}",
        )
    pin = Cylinder(radius=0.0035, length=2 * pin_dy + 0.014)
    body.visual(
        pin,
        origin=Origin(xyz=(pin_x, 0.0, pin_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="lever_pin",
    )

    # --- pressure gauge on the front-facing side of the valve ---
    gauge_z = head_z + 0.010
    gauge_stem = Cylinder(radius=0.005, length=0.036)
    body.visual(
        gauge_stem,
        origin=Origin(xyz=(0.0, -0.032, gauge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="gauge_stem",
    )
    gauge_case = LatheGeometry(
        [
            (0.0, 0.0),
            (0.019, 0.0),
            (0.019, 0.012),
            (0.017, 0.014),
            (0.0, 0.014),
        ],
        segments=28,
    )
    body.visual(
        mesh_from_geometry(gauge_case, "gauge_case"),
        origin=Origin(xyz=(0.0, -0.046, gauge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="gauge_case",
    )
    gauge_dial = Cylinder(radius=0.015, length=0.004)
    body.visual(
        gauge_dial,
        origin=Origin(xyz=(0.0, -0.058, gauge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gauge_face,
        name="gauge_dial",
    )

    # --- safety pin + pull ring threaded through the pivot lugs ---
    safety_z = lug_base_z + lug_len - 0.006
    safety_pin = Cylinder(radius=0.0022, length=2 * pin_dy + 0.040)
    body.visual(
        safety_pin,
        origin=Origin(xyz=(pin_x, 0.006, safety_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="safety_pin",
    )
    ring_r = 0.014
    pin_end_y = 0.006 + (2 * pin_dy + 0.040) / 2.0
    ring = TorusGeometry(ring_r, 0.0028, radial_segments=12, tubular_segments=24)
    body.visual(
        mesh_from_geometry(ring, "pull_ring"),
        origin=Origin(xyz=(pin_x, pin_end_y - 0.003, safety_z - ring_r), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pull_ring",
    )

    # --- discharge hose + nozzle clipped to the side ---
    hose_pts = [
        (0.020, 0.0, head_z + 0.004),
        (0.060, 0.024, head_z - 0.040),
        (0.080, 0.014, shoulder_z - 0.010),
        (body_r + 0.012, -0.012, 0.120),
        (body_r + 0.008, -0.006, 0.070),
    ]
    hose = tube_from_spline_points(
        hose_pts, radius=0.007, samples_per_segment=10, radial_segments=10
    )
    body.visual(
        mesh_from_geometry(hose, "discharge_hose"),
        material=black,
        name="discharge_hose",
    )
    nozzle = LatheGeometry(
        [(0.0, 0.0), (0.009, 0.0), (0.013, 0.026), (0.019, 0.042), (0.007, 0.042), (0.0, 0.026)],
        segments=24,
    )
    body.visual(
        mesh_from_geometry(nozzle, "discharge_nozzle"),
        origin=Origin(xyz=(body_r + 0.008, -0.006, 0.070), rpy=(0.0, math.pi, 0.0)),
        material=black,
        name="discharge_nozzle",
    )

    # ===================================================================
    # SQUEEZE OPERATING LEVER (revolute about the rear cross-pin, Y axis)
    # ===================================================================
    lever = model.part("operating_lever")
    # Authored in the joint-local frame (pin at origin). The lever extends
    # forward (+X) and curves up; at q=0 it sits raised (released). Positive q
    # brings its front edge down toward the carry handle (squeezed).
    lever_profile = rounded_rect_profile(0.030, 0.010, 0.003)
    lever_pts = [
        (0.0, 0.0, 0.0),
        (0.030, 0.0, 0.006),
        (0.062, 0.0, 0.012),
        (0.092, 0.0, 0.010),
        (0.114, 0.0, 0.002),   # curved front tip
    ]
    lever_geom = sweep_profile_along_spline(
        lever_pts, profile=lever_profile, samples_per_segment=12, cap_profile=True
    )
    lever.visual(
        mesh_from_geometry(lever_geom, "operating_lever"),
        material=red,
        name="operating_lever",
    )
    lever.inertial = Inertial.from_geometry(
        Cylinder(radius=0.008, length=0.114), mass=0.13
    )
    model.articulation(
        "body_to_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(pin_x, 0.0, pin_z)),
        # Lever extends along +X; +Y rotation drops the front edge (squeeze).
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=0.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lever = object_model.get_part("operating_lever")
    lever_joint = object_model.get_articulation("body_to_lever")

    # --- Base ring rests on the ground (z=0), nothing buried/floating ---
    bb = ctx.part_world_aabb(body)
    assert bb is not None
    ctx.check("base_at_ground", abs(bb[0][2]) < 0.005, details=f"body min z={bb[0][2]:.4f}")

    # --- Bottle is SQUAT: wider than tall (low-and-wide stored-pressure unit) ---
    width = bb[1][0] - bb[0][0]
    depth = bb[1][1] - bb[0][1]
    height = bb[1][2] - bb[0][2]
    ctx.check(
        "squat_wide_body",
        width > height * 0.55,
        details=f"w={width:.3f}, h={height:.3f}, ratio={width/max(height,1e-6):.2f}",
    )
    # The body diameter should clearly exceed 0.16 m (wide unit).
    ctx.check(
        "wide_diameter",
        width > 0.16,
        details=f"body width={width:.3f}",
    )
    # Body cylinder height is short relative to its diameter.
    ctx.check(
        "short_cylinder",
        height < width * 1.8,
        details=f"h={height:.3f} < {width * 1.8:.3f} = 1.8×w",
    )

    # --- Pressure gauge present and on the visual front (-Y) ---
    gauge = ctx.part_element_world_aabb(body, elem="gauge_dial")
    assert gauge is not None
    gauge_y = (gauge[0][1] + gauge[1][1]) / 2
    gauge_z_mid = (gauge[0][2] + gauge[1][2]) / 2
    ctx.check("gauge_on_front", gauge_y < -0.03, details=f"gauge y={gauge_y:.3f}")
    ctx.check(
        "gauge_near_valve_head",
        0.280 < gauge_z_mid < 0.340,
        details=f"gauge z={gauge_z_mid:.3f}",
    )

    # --- Label band wraps the wide body at mid-height ---
    label = ctx.part_element_world_aabb(body, elem="label_band")
    assert label is not None
    label_mid_z = (label[0][2] + label[1][2]) / 2
    ctx.check(
        "label_mid_body",
        0.04 < label_mid_z < 0.18,
        details=f"label mid z={label_mid_z:.3f}",
    )
    # Label radius should match the wide body.
    label_width = label[1][0] - label[0][0]
    ctx.check(
        "label_wide_wrap",
        label_width > 0.16,
        details=f"label width={label_width:.3f}",
    )

    # --- Operating lever pivots about horizontal Y and squeezes downward ---
    assert abs(lever_joint.axis[1]) > 0.99, "lever axis must be horizontal Y"
    rest_tip = ctx.part_element_world_aabb(lever, elem="operating_lever")
    assert rest_tip is not None
    rest_front_z = rest_tip[0][2]
    with ctx.pose({lever_joint: lever_joint.motion_limits.upper}):
        sq = ctx.part_element_world_aabb(lever, elem="operating_lever")
        assert sq is not None
        sq_front_z = sq[0][2]
    ctx.check(
        "lever_squeezes_down",
        sq_front_z < rest_front_z - 0.01,
        details=f"released front z={rest_front_z:.3f}, squeezed front z={sq_front_z:.3f}",
    )

    # --- Lever is carried at the rear pin (contacts the body) ---
    ctx.expect_contact(lever, body, contact_tol=0.010, name="lever_on_pin")

    # --- Carry handle sits below the operating lever (lever is the upper one) ---
    carry = ctx.part_element_world_aabb(body, elem="carry_handle")
    assert carry is not None
    lev_mid_z = (rest_tip[0][2] + rest_tip[1][2]) / 2
    carry_mid_z = (carry[0][2] + carry[1][2]) / 2
    ctx.check(
        "lever_above_carry_handle",
        lev_mid_z > carry_mid_z,
        details=f"lever mid z={lev_mid_z:.3f}, carry mid z={carry_mid_z:.3f}",
    )

    # The lever wraps around the cross-pin (intended pivot capture).
    ctx.allow_overlap(
        lever, body, elem_a="operating_lever", elem_b="lever_pin",
        reason="The operating lever pivots on and overlaps the cross-pin (captured hinge).",
    )

    return ctx.report()


object_model = build_object_model()
