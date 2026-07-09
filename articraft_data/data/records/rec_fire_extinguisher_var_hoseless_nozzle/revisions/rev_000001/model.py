from __future__ import annotations

# Red portable fire extinguisher.
#
# Body axis is +Z. Base ring rests at z=0. From the bottom up:
#   - recessed base ring / foot
#   - red cylindrical bottle with two rolled banding rings and a label band
#   - a domed red shoulder
#   - a brass valve neck
#   - the operating head: a fixed carry handle (lower bar) and a squeeze
#     operating lever (upper, curved) that pivots down about a rear cross-pin
#   - a round pressure gauge on the front-facing side of the valve
#   - a pull safety pin with a ring through the handle slot
#   - a short fixed discharge nozzle straight off the front of the valve head
#     (hose-less compact type, as on car/kitchen extinguishers)
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

    # --- valve head block: a small brass body that carries the handle & lever ---
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

    # --- fixed carry handle (lower grab bar, extends back over the dome) ---
    # A flat strap that sweeps from the rear of the valve head backward and up.
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

    # --- rear pivot lugs + cross-pin for the operating lever ---
    pin_x = -0.012
    lug_base_z = head_z + 0.004
    lug_len = 0.034
    pin_z = lug_base_z + lug_len - 0.008  # pin sits near the lug tops
    pin_dy = 0.016
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
    # The stem roots inside the valve head and reaches forward to the gauge
    # case; case and dial overlap solidly at the same height as before.
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

    # --- safety pin + pull ring threaded through the pivot lugs ---
    # The pin runs along Y through both brass lugs (so it stays connected) and
    # the pull ring hangs off the +Y end.
    safety_z = lug_base_z + lug_len - 0.006
    safety_pin = Cylinder(radius=0.0022, length=2 * pin_dy + 0.040)
    body.visual(
        safety_pin,
        origin=Origin(xyz=(pin_x, 0.006, safety_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="safety_pin",
    )
    # Pull ring links onto the protruding +Y end of the pin: the ring loop is
    # dropped so its upper tube overlaps the pin end (linked, connected).
    ring_r = 0.014
    pin_end_y = 0.006 + (2 * pin_dy + 0.040) / 2.0  # = +Y tip of the safety pin
    ring = TorusGeometry(ring_r, 0.0028, radial_segments=12, tubular_segments=24)
    body.visual(
        mesh_from_geometry(ring, "pull_ring"),
        origin=Origin(xyz=(pin_x, pin_end_y - 0.003, safety_z - ring_r), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pull_ring",
    )

    # --- fixed discharge nozzle (front of valve head, hose-less compact type) ---
    # A short flared horn that exits straight forward (-Y) from the valve
    # head, the way compact car/kitchen extinguishers discharge directly.
    nozzle_z = head_z + 0.006
    nozzle_geom = LatheGeometry(
        [
            (0.0, 0.0),
            (0.012, 0.0),       # mounting flange base
            (0.012, 0.004),     # flange step
            (0.009, 0.006),     # narrow throat
            (0.013, 0.028),     # flare body
            (0.017, 0.040),     # exit bell
            (0.015, 0.043),     # exit rim lip
            (0.0, 0.043),       # cap (closed visual end)
        ],
        segments=24,
    )
    body.visual(
        mesh_from_geometry(nozzle_geom, "discharge_nozzle"),
        origin=Origin(xyz=(0.0, -0.022, nozzle_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
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
    lever_profile = rounded_rect_profile(0.028, 0.009, 0.003)
    lever_pts = [
        (0.0, 0.0, 0.0),
        (0.028, 0.0, 0.006),
        (0.060, 0.0, 0.012),
        (0.090, 0.0, 0.010),
        (0.110, 0.0, 0.002),   # curved front tip
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
        Cylinder(radius=0.008, length=0.110), mass=0.12
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

    # --- Fixed discharge nozzle on front (-Y) of valve head, hose-less type ---
    nozzle_aabb = ctx.part_element_world_aabb(body, elem="discharge_nozzle")
    assert nozzle_aabb is not None
    nozzle_y_min = nozzle_aabb[0][1]
    nozzle_y_max = nozzle_aabb[1][1]
    nozzle_z_mid = (nozzle_aabb[0][2] + nozzle_aabb[1][2]) / 2.0
    # Nozzle must extend well forward (-Y) of the valve centerline.
    ctx.check(
        "nozzle_points_forward",
        nozzle_y_min < -0.04,
        details=f"nozzle min y={nozzle_y_min:.4f}",
    )
    # Nozzle must be at valve-head height (not down on the body).
    ctx.check(
        "nozzle_at_valve_height",
        0.440 < nozzle_z_mid < 0.465,
        details=f"nozzle mid z={nozzle_z_mid:.4f}",
    )
    # No hose present (hose-less compact type).
    hose_aabb = ctx.part_element_world_aabb(body, elem="discharge_hose")
    ctx.check(
        "no_discharge_hose",
        hose_aabb is None,
        details="discharge_hose visual should not exist on hose-less variant",
    )

    return ctx.report()


object_model = build_object_model()
