from __future__ import annotations

# Four-wheeled covered wooden wagon caravan (small house-cart / vardo).
#
# World frame: Z-up. A plank cargo box body sits on a four-wheeled wagon chassis
# (smaller steerable front pair, larger fixed rear pair). On top of the box is a
# peaked GABLED plank roof: two sloped roof panels meeting at a central ridge,
# with triangular gable end walls front and rear. A single long wooden draw pole
# runs forward from the steerable front bolster.
#
# +X = forward (toward the draw pole).  +Y = left.  -Y = right.
#
# Articulation:
#   PRIMARY = the four wheels rolling (CONTINUOUS spin about their axle, world Y).
#   SECONDARY = the front bolster STEERS (REVOLUTE yaw about a vertical kingpin,
#     world Z). The two front wheels and the draw pole are children of the
#     bolster, so they swing with it.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---- Wheel sizing (meters) -------------------------------------------------
REAR_WHEEL_R = 0.32
FRONT_WHEEL_R = 0.25
RIM_TUBE_R = 0.023
SPOKE_R = 0.008
HUB_R = 0.056
HUB_LEN = 0.12

REAR_AXLE_Z = REAR_WHEEL_R + RIM_TUBE_R     # rear wheels touch z=0
FRONT_AXLE_Z = FRONT_WHEEL_R + RIM_TUBE_R   # front wheels touch z=0

HALF_TRACK = 0.46
REAR_AXLE_X = -0.50
FRONT_AXLE_X = 0.50

# Box body
BOX_LEN = 1.30
BOX_WIDTH = 0.80
FLOOR_Z = REAR_AXLE_Z + 0.10
FLOOR_THK = 0.04
WALL_H = 0.46          # tall plank walls (it's a little cabin)
PLANK_THK = 0.028
AXLE_R = 0.030

# Gabled roof
WALL_TOP_Z = FLOOR_Z + FLOOR_THK / 2.0 + WALL_H
ROOF_RIDGE_H = 0.34    # ridge height above the wall top
ROOF_OVERHANG = 0.10


def _wheel_visuals(part, mesh_prefix: str, radius: float, wood, dark_wood, iron) -> None:
    seg = 48
    ring_pts = [
        (radius * math.cos(2.0 * math.pi * i / seg), 0.0, radius * math.sin(2.0 * math.pi * i / seg))
        for i in range(seg + 1)
    ]
    rim = tube_from_spline_points(
        ring_pts, radius=RIM_TUBE_R, samples_per_segment=2, radial_segments=12, closed_spline=True
    )
    part.visual(mesh_from_geometry(rim, f"{mesh_prefix}_rim"), material=wood, name="rim")

    inner = HUB_R - 0.015
    outer = radius + 0.004
    mid = 0.5 * (inner + outer)
    length = outer - inner
    spoke_count = 16
    for i in range(spoke_count):
        a = 2.0 * math.pi * i / spoke_count
        part.visual(
            Cylinder(radius=SPOKE_R, length=length),
            origin=Origin(xyz=(mid * math.cos(a), 0.0, mid * math.sin(a)), rpy=(0.0, math.pi / 2.0 - a, 0.0)),
            material=wood,
            name=f"spoke_{i}",
        )

    part.visual(
        Cylinder(radius=HUB_R, length=HUB_LEN),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_wood,
        name="hub",
    )
    part.visual(
        Cylinder(radius=HUB_R + 0.009, length=0.020),
        origin=Origin(xyz=(0.0, HUB_LEN * 0.34, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="hub_band",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="covered_wooden_wagon_caravan")

    wood = model.material("wood", rgba=(0.70, 0.55, 0.35, 1.0))
    dark_wood = model.material("dark_wood", rgba=(0.45, 0.33, 0.21, 1.0))
    plank = model.material("plank_wood", rgba=(0.78, 0.62, 0.40, 1.0))
    roof_wood = model.material("roof_wood", rgba=(0.64, 0.49, 0.30, 1.0))
    iron = model.material("wrought_iron", rgba=(0.17, 0.17, 0.18, 1.0))

    # ---- Body / chassis + box + roof (root) -------------------------------
    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((BOX_LEN, BOX_WIDTH, WALL_H + ROOF_RIDGE_H)),
        mass=160.0,
        origin=Origin(xyz=(0.0, 0.0, FLOOR_Z + 0.3)),
    )

    # Floor planks (butt together so they connect).
    n_floor = 6
    plank_w = BOX_WIDTH / n_floor
    for i in range(n_floor):
        y = -BOX_WIDTH / 2.0 + plank_w * (i + 0.5)
        body.visual(
            Box((BOX_LEN, plank_w + 0.002, FLOOR_THK)),
            origin=Origin(xyz=(0.0, y, FLOOR_Z)),
            material=plank,
            name=f"floor_plank_{i}",
        )

    # Side walls (+/-Y): vertical plank cabin walls (stacked horizontal boards).
    n_wall = 5
    board_h = WALL_H / n_wall
    for side_name, ysign in (("left", 1.0), ("right", -1.0)):
        y = ysign * (BOX_WIDTH / 2.0 - PLANK_THK / 2.0)
        for k in range(n_wall):
            z = FLOOR_Z + FLOOR_THK / 2.0 + board_h * (k + 0.5)
            body.visual(
                Box((BOX_LEN, PLANK_THK, board_h + 0.002)),
                origin=Origin(xyz=(0.0, y, z)),
                material=plank,
                name=f"{side_name}_wall_board_{k}",
            )

    # Rear end wall (-X), full height plank wall.
    rear_x = -BOX_LEN / 2.0 + PLANK_THK / 2.0
    body.visual(
        Box((PLANK_THK, BOX_WIDTH, WALL_H)),
        origin=Origin(xyz=(rear_x, 0.0, FLOOR_Z + FLOOR_THK / 2.0 + WALL_H / 2.0)),
        material=plank,
        name="rear_wall",
    )
    # Front end wall (+X), full height plank wall (with a low doorway implied).
    front_x = BOX_LEN / 2.0 - PLANK_THK / 2.0
    body.visual(
        Box((PLANK_THK, BOX_WIDTH, WALL_H)),
        origin=Origin(xyz=(front_x, 0.0, FLOOR_Z + FLOOR_THK / 2.0 + WALL_H / 2.0)),
        material=plank,
        name="front_wall",
    )

    # Corner posts.
    for xsign in (1.0, -1.0):
        for ysign in (1.0, -1.0):
            body.visual(
                Box((0.05, 0.05, WALL_H + 0.02)),
                origin=Origin(
                    xyz=(
                        xsign * (BOX_LEN / 2.0 - 0.025),
                        ysign * (BOX_WIDTH / 2.0 - 0.025),
                        FLOOR_Z + FLOOR_THK / 2.0 + WALL_H / 2.0,
                    )
                ),
                material=dark_wood,
                name=f"post_{'f' if xsign > 0 else 'r'}_{'l' if ysign > 0 else 'r'}",
            )

    # ---- Gabled roof on top of the box ------------------------------------
    # Ridge beam along X at the top center. Sized and dropped slightly so it
    # physically contacts both sloped roof panels at the apex.
    ridge_z = WALL_TOP_Z + ROOF_RIDGE_H
    body.visual(
        Box((BOX_LEN + 2 * ROOF_OVERHANG, 0.10, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, ridge_z - 0.04)),
        material=dark_wood,
        name="ridge_beam",
    )
    # Two sloped roof panels (left and right of the ridge). Each panel is a thin
    # box tilted so its top edge is at the ridge and its eave is just past the
    # wall, overhanging slightly.
    half_w = BOX_WIDTH / 2.0 + ROOF_OVERHANG
    slope_run = half_w
    slope_rise = ROOF_RIDGE_H
    slope_len = math.hypot(slope_run, slope_rise)
    slope_angle = math.atan2(slope_rise, slope_run)  # tilt about X
    roof_thk = 0.03
    for side_name, ysign in (("left", 1.0), ("right", -1.0)):
        # Panel center: midway between ridge (y=0, z=ridge_z) and eave
        # (y=ysign*half_w, z=WALL_TOP_Z).
        cy = ysign * half_w / 2.0
        cz = (ridge_z + WALL_TOP_Z) / 2.0
        # Roll so the panel's outer (eave) edge dips DOWN toward the wall: for the
        # +Y panel that means rotating +Y toward -Z, i.e. roll = -slope_angle.
        roll = -ysign * slope_angle
        body.visual(
            Box((BOX_LEN + 2 * ROOF_OVERHANG, slope_len, roof_thk)),
            origin=Origin(xyz=(0.0, cy, cz), rpy=(roll, 0.0, 0.0)),
            material=roof_wood,
            name=f"{side_name}_roof_panel",
        )

    # Triangular gable end fills (front and rear) under each roof slope. Two
    # thin boards per end, each tilted to lie flush just beneath a roof panel so
    # together they close the triangular gable opening.
    gable_board_h = 0.16
    for end_name, xsign in (("front", 1.0), ("rear", -1.0)):
        gx = xsign * (BOX_LEN / 2.0 - PLANK_THK / 2.0)
        for side_name, ysign in (("left", 1.0), ("right", -1.0)):
            cy = ysign * half_w / 2.0
            cz = (ridge_z + WALL_TOP_Z) / 2.0 - gable_board_h * 0.5
            roll = -ysign * slope_angle
            body.visual(
                Box((PLANK_THK, slope_len, gable_board_h)),
                origin=Origin(xyz=(gx, cy, cz), rpy=(roll, 0.0, 0.0)),
                material=plank,
                name=f"{end_name}_gable_{side_name}",
            )

    # Longitudinal chassis sills under the floor.
    for side_name, ysign in (("left", 1.0), ("right", -1.0)):
        body.visual(
            Box((BOX_LEN + 0.10, 0.07, 0.09)),
            origin=Origin(xyz=(0.0, ysign * 0.28, FLOOR_Z - FLOOR_THK / 2.0 - 0.045)),
            material=dark_wood,
            name=f"{side_name}_sill",
        )

    # Rear axle (fixed).
    body.visual(
        Cylinder(radius=AXLE_R, length=2.0 * HALF_TRACK + 0.12),
        origin=Origin(xyz=(REAR_AXLE_X, 0.0, REAR_AXLE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="rear_axle",
    )

    # Kingpin boss under the front of the bed (steering pivot).
    body.visual(
        Cylinder(radius=0.058, length=0.20),
        origin=Origin(xyz=(FRONT_AXLE_X, 0.0, FLOOR_Z - FLOOR_THK / 2.0 - 0.10)),
        material=iron,
        name="kingpin_boss",
    )

    # ---- Front steering bolster -------------------------------------------
    bolster = model.part("front_bolster")
    bolster.inertial = Inertial.from_geometry(
        Box((0.16, 2.0 * HALF_TRACK, 0.10)),
        mass=12.0,
        origin=Origin(),
    )
    bolster_z = FLOOR_Z - FLOOR_THK / 2.0 - 0.16  # below the sills
    axle_local_z = FRONT_AXLE_Z - bolster_z

    bolster.visual(
        Box((0.16, 2.0 * HALF_TRACK - 0.28, 0.10)),
        origin=Origin(),
        material=dark_wood,
        name="bolster_beam",
    )
    bolster.visual(
        Cylinder(radius=AXLE_R, length=2.0 * HALF_TRACK + 0.12),
        origin=Origin(xyz=(0.0, 0.0, axle_local_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="front_axle",
    )
    # Standards tying the beam to the axle.
    for ysign in (1.0, -1.0):
        bolster.visual(
            Box((0.07, 0.07, abs(axle_local_z) + 0.08)),
            origin=Origin(xyz=(0.0, ysign * 0.32, axle_local_z / 2.0)),
            material=dark_wood,
            name=f"standard_{'l' if ysign > 0 else 'r'}",
        )

    # Single long draw pole (central) running forward from the bolster.
    pole_len = 1.35
    pole = tube_from_spline_points(
        [
            (-0.05, 0.0, axle_local_z + 0.02),
            (0.25, 0.0, axle_local_z + 0.04),
            (0.70, 0.0, axle_local_z + 0.05),
            (pole_len, 0.0, axle_local_z + 0.07),
        ],
        radius=0.032,
        samples_per_segment=12,
        radial_segments=14,
    )
    bolster.visual(mesh_from_geometry(pole, "draw_pole"), material=wood, name="draw_pole")
    # A small swingletree / crossbar near the pole tip.
    bolster.visual(
        Box((0.05, 0.46, 0.05)),
        origin=Origin(xyz=(pole_len - 0.05, 0.0, axle_local_z + 0.07)),
        material=dark_wood,
        name="swingletree",
    )

    # ---- Wheels ------------------------------------------------------------
    def make_wheel(name: str, radius: float):
        w = model.part(name)
        w.inertial = Inertial.from_geometry(
            Cylinder(radius=radius, length=HUB_LEN),
            mass=10.0,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        )
        _wheel_visuals(w, name, radius, wood, dark_wood, iron)
        return w

    rear_left = make_wheel("rear_left_wheel", REAR_WHEEL_R)
    rear_right = make_wheel("rear_right_wheel", REAR_WHEEL_R)
    front_left = make_wheel("front_left_wheel", FRONT_WHEEL_R)
    front_right = make_wheel("front_right_wheel", FRONT_WHEEL_R)

    model.articulation(
        "front_steer",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bolster,
        origin=Origin(xyz=(FRONT_AXLE_X, 0.0, bolster_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=2.0, lower=-0.6, upper=0.6),
    )
    model.articulation(
        "rear_left_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rear_left,
        origin=Origin(xyz=(REAR_AXLE_X, HALF_TRACK, REAR_AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=20.0),
    )
    model.articulation(
        "rear_right_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rear_right,
        origin=Origin(xyz=(REAR_AXLE_X, -HALF_TRACK, REAR_AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=20.0),
    )
    model.articulation(
        "front_left_spin",
        ArticulationType.CONTINUOUS,
        parent=bolster,
        child=front_left,
        origin=Origin(xyz=(0.0, HALF_TRACK, axle_local_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=20.0),
    )
    model.articulation(
        "front_right_spin",
        ArticulationType.CONTINUOUS,
        parent=bolster,
        child=front_right,
        origin=Origin(xyz=(0.0, -HALF_TRACK, axle_local_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    bolster = object_model.get_part("front_bolster")
    rl = object_model.get_part("rear_left_wheel")
    rr = object_model.get_part("rear_right_wheel")
    fl = object_model.get_part("front_left_wheel")
    fr = object_model.get_part("front_right_wheel")

    steer = object_model.get_articulation("front_steer")
    rl_spin = object_model.get_articulation("rear_left_spin")
    rr_spin = object_model.get_articulation("rear_right_spin")
    fl_spin = object_model.get_articulation("front_left_spin")
    fr_spin = object_model.get_articulation("front_right_spin")

    # Intentional axle-through-hub mounts and steering-turntable interfaces.
    ctx.allow_overlap(body, bolster, elem_a="kingpin_boss", elem_b="bolster_beam",
                      reason="The kingpin boss seats into the bolster beam at the steering turntable.")
    ctx.allow_overlap(body, bolster, elem_a="kingpin_boss", elem_b="front_axle",
                      reason="The kingpin passes down through the bolster to the front axle.")
    ctx.allow_overlap(body, bolster, elem_a="kingpin_boss", elem_b="draw_pole",
                      reason="The draw pole roots at the turntable hub directly below the kingpin boss.")
    ctx.allow_overlap(body, rl, elem_a="rear_axle", elem_b="hub",
                      reason="Rear axle stub passes through the rear-left wheel hub.")
    ctx.allow_overlap(body, rl, elem_a="rear_axle", elem_b="hub_band",
                      reason="Rear axle also passes through the rear-left iron nave band.")
    ctx.allow_overlap(body, rr, elem_a="rear_axle", elem_b="hub",
                      reason="Rear axle stub passes through the rear-right wheel hub.")
    ctx.allow_overlap(body, rr, elem_a="rear_axle", elem_b="hub_band",
                      reason="Rear axle also passes through the rear-right iron nave band.")
    ctx.allow_overlap(bolster, fl, elem_a="front_axle", elem_b="hub",
                      reason="Front axle stub passes through the front-left wheel hub.")
    ctx.allow_overlap(bolster, fl, elem_a="front_axle", elem_b="hub_band",
                      reason="Front axle also passes through the front-left iron nave band.")
    ctx.allow_overlap(bolster, fr, elem_a="front_axle", elem_b="hub",
                      reason="Front axle stub passes through the front-right wheel hub.")
    ctx.allow_overlap(bolster, fr, elem_a="front_axle", elem_b="hub_band",
                      reason="Front axle also passes through the front-right iron nave band.")

    # --- Wheel spin joints: CONTINUOUS about Y ---
    for jname, joint in (
        ("rear_left_spin", rl_spin), ("rear_right_spin", rr_spin),
        ("front_left_spin", fl_spin), ("front_right_spin", fr_spin),
    ):
        ctx.check(f"{jname}_continuous", str(joint.joint_type).lower().endswith("continuous"),
                  f"type={joint.joint_type}")
        ax = joint.axis
        ctx.check(f"{jname}_axis_y", abs(abs(ax[1]) - 1.0) < 1e-6 and abs(ax[0]) < 1e-6 and abs(ax[2]) < 1e-6,
                  f"axis={ax}")

    # --- Steering joint: REVOLUTE about Z ---
    ctx.check("front_steer_revolute", str(steer.joint_type).lower().endswith("revolute"),
              f"type={steer.joint_type}")
    sax = steer.axis
    ctx.check("front_steer_axis_z", abs(abs(sax[2]) - 1.0) < 1e-6 and abs(sax[0]) < 1e-6 and abs(sax[1]) < 1e-6,
              f"axis={sax}")

    # --- All four wheels touch ground; front pair smaller than rear pair ---
    for name, wheel, exp_d in (
        ("rear_left_wheel", rl, 2 * (REAR_WHEEL_R + RIM_TUBE_R)),
        ("rear_right_wheel", rr, 2 * (REAR_WHEEL_R + RIM_TUBE_R)),
        ("front_left_wheel", fl, 2 * (FRONT_WHEEL_R + RIM_TUBE_R)),
        ("front_right_wheel", fr, 2 * (FRONT_WHEEL_R + RIM_TUBE_R)),
    ):
        aabb = ctx.part_world_aabb(wheel)
        assert aabb is not None
        mins, maxs = aabb
        ctx.check(f"{name}_touches_ground", abs(mins[2]) < 0.01, f"min_z={mins[2]:.4f}")
        d = maxs[2] - mins[2]
        ctx.check(f"{name}_diameter", abs(d - exp_d) < 0.02, f"d={d:.3f} exp={exp_d:.3f}")

    fl_aabb = ctx.part_world_aabb(fl)
    rl_aabb = ctx.part_world_aabb(rl)
    assert fl_aabb and rl_aabb
    ctx.check("front_smaller_than_rear",
              (fl_aabb[1][2] - fl_aabb[0][2]) < (rl_aabb[1][2] - rl_aabb[0][2]) - 0.05,
              "front pair must be visibly smaller than rear pair")

    # --- Symmetry ---
    for la, ra, label in ((rl, rr, "rear"), (fl, fr, "front")):
        lp = ctx.part_world_position(la)
        rp = ctx.part_world_position(ra)
        assert lp and rp
        ctx.check(f"{label}_wheels_symmetric_y", abs(lp[1] + rp[1]) < 1e-6, f"{lp[1]} vs {rp[1]}")

    # --- Gabled roof present: ridge is above both eaves and above the walls ---
    ridge = body.get_visual("ridge_beam")
    left_panel = body.get_visual("left_roof_panel")
    ridge_aabb = ctx.part_element_world_aabb(body, elem="ridge_beam")
    panel_aabb = ctx.part_element_world_aabb(body, elem="left_roof_panel")
    assert ridge is not None and left_panel is not None
    assert ridge_aabb is not None and panel_aabb is not None
    ridge_top = ridge_aabb[1][2]
    wall_aabb = ctx.part_element_world_aabb(body, elem="left_wall_board_4")
    assert wall_aabb is not None
    ctx.check("roof_peaks_above_walls", ridge_top > wall_aabb[1][2] + 0.10,
              f"ridge_top={ridge_top:.3f} wall_top={wall_aabb[1][2]:.3f}")
    # Roof panel spans laterally beyond the wall (overhang/peak shape).
    ctx.check("roof_panel_slopes_down_to_eave",
              panel_aabb[0][2] < ridge_top - 0.05,
              f"panel_min_z={panel_aabb[0][2]:.3f} ridge_top={ridge_top:.3f}")

    # --- Engagement: each wheel hub overlaps its axle (no detached gap) ---
    ctx.expect_overlap(rl, body, axes="xz", min_overlap=0.02, elem_b="rear_axle",
                       name="rear_left_engages_axle")
    ctx.expect_overlap(rr, body, axes="xz", min_overlap=0.02, elem_b="rear_axle",
                       name="rear_right_engages_axle")
    ctx.expect_overlap(fl, bolster, axes="xz", min_overlap=0.02, elem_b="front_axle",
                       name="front_left_engages_axle")
    ctx.expect_overlap(fr, bolster, axes="xz", min_overlap=0.02, elem_b="front_axle",
                       name="front_right_engages_axle")

    # --- Each wheel has exactly 16 spokes (dense fine spoke pattern) ---
    expected_spoke_count = 16
    for wname, wheel in (("rear_left_wheel", rl), ("rear_right_wheel", rr),
                         ("front_left_wheel", fl), ("front_right_wheel", fr)):
        spoke_names = [v.name for v in wheel.visuals if v.name.startswith("spoke_")]
        ctx.check(f"{wname}_has_16_spokes",
                  len(spoke_names) == expected_spoke_count,
                  f"found {len(spoke_names)} spokes: {sorted(spoke_names)}")
        # Confirm the highest spoke index is 15 (spoke_0..spoke_15)
        max_idx = max(int(n.split("_")[1]) for n in spoke_names) if spoke_names else -1
        ctx.check(f"{wname}_spoke_indices_0_to_15",
                  max_idx == expected_spoke_count - 1,
                  f"max spoke index={max_idx}")

    # --- Decisive spin pose ---
    spoke_rest = ctx.part_element_world_aabb(rl, elem="spoke_0")
    with ctx.pose({rl_spin: math.pi / 2.0}):
        spoke_turned = ctx.part_element_world_aabb(rl, elem="spoke_0")
    assert spoke_rest and spoke_turned
    ctx.check("rear_wheel_spins",
              abs(spoke_turned[0][0] - spoke_rest[0][0]) > 0.02 or abs(spoke_turned[0][2] - spoke_rest[0][2]) > 0.02,
              f"rest={spoke_rest[0]} turned={spoke_turned[0]}")

    # --- Decisive steer pose ---
    fl_rest = ctx.part_world_position(fl)
    with ctx.pose({steer: 0.5}):
        fl_steered = ctx.part_world_position(fl)
    assert fl_rest and fl_steered
    ctx.check("front_wheels_steer_with_bolster",
              abs(fl_steered[0] - fl_rest[0]) > 0.02 or abs(fl_steered[1] - fl_rest[1]) > 0.02,
              f"rest={fl_rest} steered={fl_steered}")

    return ctx.report()


object_model = build_object_model()
