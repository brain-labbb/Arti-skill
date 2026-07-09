from __future__ import annotations

# Red portable fire extinguisher seated in a tubular floor stand.
#
# Body axis is +Z. The floor stand base plate sits at z=0. From the bottom up:
#   - flat steel base plate (floor stand root)
#   - two tubular posts rising to an encircling retainer ring
#   - the bottle seated on the base plate inside the ring:
#     - recessed base ring / foot
#     - red cylindrical bottle with two rolled banding rings and a label band
#     - a domed red shoulder
#     - a brass valve neck
#     - the operating head: a fixed carry handle (lower bar) and a squeeze
#       operating lever (upper, curved) that pivots down about a rear cross-pin
#     - a round pressure gauge on the front-facing side of the valve
#     - a pull safety pin with a ring through the handle slot
#     - a short discharge hose + nozzle clipped along the side
#
# Articulations:
#   - stand_to_body (FIXED): bottle seated on the base plate
#   - body_to_lever (REVOLUTE): squeeze operating lever about the rear
#     valve cross-pin, horizontal Y axis; squeezing brings the front edge
#     down toward the carry handle.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
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
    stand_paint = model.material("stand_paint", rgba=(0.10, 0.10, 0.11, 1.0))

    # ---------- key dimensions ----------
    body_r = 0.056
    shoulder_z = 0.330       # where the cylinder ends and the dome begins
    dome_top_z = 0.400       # top of the dome
    neck_top_z = 0.440       # top of the brass valve neck

    # ---------- floor stand dimensions ----------
    plate_size = 0.200          # base plate side length
    plate_thickness = 0.006     # base plate thickness
    ring_z = 0.100              # ring center height above ground
    ring_major_r = 0.068        # torus major radius (center to tube center)
    ring_tube_r = 0.008         # torus tube cross-section radius
    post_r = 0.008              # vertical post tube radius

    # ===================================================================
    # ROOT: floor_stand (base plate + tubular posts + encircling ring)
    # ===================================================================
    floor_stand = model.part("floor_stand")

    # Flat base plate sitting on the ground.
    floor_stand.visual(
        Box((plate_size, plate_size, plate_thickness)),
        origin=Origin(xyz=(0.0, 0.0, plate_thickness / 2.0)),
        material=stand_paint,
        name="base_plate",
    )

    # Two vertical tubular posts connecting the base plate to the ring.
    # Posts are at the ±Y sides of the ring, rising from the plate top.
    post_length = ring_z - plate_thickness
    for i in range(2):
        y_sign = 1 if i == 0 else -1
        post = Cylinder(radius=post_r, length=post_length)
        floor_stand.visual(
            post,
            origin=Origin(xyz=(0.0, y_sign * ring_major_r,
                               plate_thickness + post_length / 2.0)),
            material=stand_paint,
            name=f"post_{i}",
        )

    # Encircling retainer ring: a full torus that holds the bottle upright.
    ring_geom = TorusGeometry(
        ring_major_r, ring_tube_r,
        radial_segments=16, tubular_segments=48,
    )
    floor_stand.visual(
        mesh_from_geometry(ring_geom, "retainer_ring"),
        origin=Origin(xyz=(0.0, 0.0, ring_z)),
        material=stand_paint,
        name="retainer_ring",
    )

    # Small gusset plates where the posts meet the base plate (structural detail).
    for i in range(2):
        y_sign = 1 if i == 0 else -1
        gusset = Box((0.024, 0.006, 0.040))
        floor_stand.visual(
            gusset,
            origin=Origin(xyz=(0.0, y_sign * ring_major_r,
                               plate_thickness + 0.020)),
            material=stand_paint,
            name=f"gusset_{i}",
        )

    floor_stand.inertial = Inertial.from_geometry(
        Box((plate_size, plate_size, ring_z + ring_tube_r)), mass=2.0,
    )

    # ===================================================================
    # BODY: bottle (base ring + cylinder + banding + dome + valve neck)
    # Sits on the base plate, held by the encircling ring.
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

    # --- FIXED: body sits on the base plate inside the floor stand ring ---
    model.articulation(
        "stand_to_body",
        ArticulationType.FIXED,
        parent=floor_stand,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, plate_thickness)),
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
    floor_stand = object_model.get_part("floor_stand")
    body = object_model.get_part("body")
    lever = object_model.get_part("operating_lever")
    lever_joint = object_model.get_articulation("body_to_lever")

    # --- Floor stand base plate rests on the ground (z=0) ---
    stand_bb = ctx.part_world_aabb(floor_stand)
    assert stand_bb is not None
    ctx.check(
        "stand_at_ground",
        abs(stand_bb[0][2]) < 0.002,
        details=f"floor_stand min z={stand_bb[0][2]:.4f}",
    )

    # --- Floor stand is wider than tall in the lower region (flat plate + ring) ---
    stand_width = stand_bb[1][0] - stand_bb[0][0]
    ctx.check(
        "stand_plate_wide",
        stand_width > 0.15,
        details=f"stand width={stand_width:.3f}",
    )

    # --- Bottle body sits on the base plate (small Z gap from ground) ---
    body_bb = ctx.part_world_aabb(body)
    assert body_bb is not None
    ctx.check(
        "body_above_plate",
        body_bb[0][2] > 0.003,
        details=f"body min z={body_bb[0][2]:.4f}",
    )

    # --- Bottle is taller than wide (upright cylinder) ---
    width = body_bb[1][0] - body_bb[0][0]
    height = body_bb[1][2] - body_bb[0][2]
    ctx.check("upright_bottle", height > width * 1.6, details=f"h={height:.3f}, w={width:.3f}")

    # --- Retainer ring encircles the bottle on XY ---
    ring_aabb = ctx.part_element_world_aabb(floor_stand, elem="retainer_ring")
    assert ring_aabb is not None
    bottle_aabb = ctx.part_element_world_aabb(body, elem="bottle")
    assert bottle_aabb is not None
    bottle_cx = (bottle_aabb[0][0] + bottle_aabb[1][0]) / 2.0
    bottle_cy = (bottle_aabb[0][1] + bottle_aabb[1][1]) / 2.0
    ring_cx = (ring_aabb[0][0] + ring_aabb[1][0]) / 2.0
    ring_cy = (ring_aabb[0][1] + ring_aabb[1][1]) / 2.0
    ctx.check(
        "ring_centered_on_bottle",
        abs(ring_cx - bottle_cx) < 0.010
        and abs(ring_cy - bottle_cy) < 0.010,
        details=f"ring=({ring_cx:.3f},{ring_cy:.3f}), bottle=({bottle_cx:.3f},{bottle_cy:.3f})",
    )

    # --- Ring is at the lower quarter of the bottle height ---
    ring_z_mid = (ring_aabb[0][2] + ring_aabb[1][2]) / 2.0
    ctx.check(
        "ring_lower_quarter",
        0.06 < ring_z_mid < 0.18,
        details=f"ring z={ring_z_mid:.3f}",
    )

    # --- Pressure gauge present and on the visual front (-Y) ---
    gauge = ctx.part_element_world_aabb(body, elem="gauge_dial")
    assert gauge is not None
    gauge_y = (gauge[0][1] + gauge[1][1]) / 2
    gauge_z_mid = (gauge[0][2] + gauge[1][2]) / 2
    ctx.check("gauge_on_front", gauge_y < -0.03, details=f"gauge y={gauge_y:.3f}")
    ctx.check("gauge_height_valid", 0.440 < gauge_z_mid < 0.470, details=f"gauge z={gauge_z_mid:.3f}")

    # --- Label band wraps the body at mid-height ---
    label = ctx.part_element_world_aabb(body, elem="label_band")
    assert label is not None
    ctx.check(
        "label_mid_body",
        0.08 < (label[0][2] + label[1][2]) / 2 < 0.32,
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

    # --- Discharge hose present on the side ---
    hose = ctx.part_element_world_aabb(body, elem="discharge_hose")
    assert hose is not None
    ctx.check(
        "hose_on_side",
        hose[1][0] > 0.03,
        details=f"hose max x={hose[1][0]:.3f}",
    )

    # The lever wraps around the cross-pin (intended pivot capture).
    ctx.allow_overlap(
        lever, body, elem_a="operating_lever", elem_b="lever_pin",
        reason="The operating lever pivots on and overlaps the cross-pin (captured hinge).",
    )

    return ctx.report()


object_model = build_object_model()
