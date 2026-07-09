from __future__ import annotations

# Four-wheeled wooden farm wagon (hay/dray wagon).
#
# World frame: Z-up. The wagon rolls on four large spoked wooden wheels: a
# SMALLER front pair on a steerable front bolster (classic wagon turntable
# under the front of the bed) and a LARGER rear pair on a fixed rear axle. A
# weathered plank box body with low side walls sits on the chassis. Two long
# wooden draw poles (shafts) run forward from the front axle, tied by rope.
#
# +X = forward (toward the draw poles).  +Y = left.  -Y = right.
#
# Articulation:
#   PRIMARY = the four wheels rolling (CONTINUOUS spin about their axle, world Y).
#   SECONDARY = the front axle bolster STEERS (REVOLUTE yaw about a vertical
#     kingpin, world Z). The two front wheels are children of the steering
#     bolster, so they swing with it; the draw poles are also children of the
#     bolster (the team turns the front axle, which steers the wagon).

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
REAR_WHEEL_R = 0.34
FRONT_WHEEL_R = 0.26
RIM_TUBE_R = 0.024
SPOKE_R = 0.015
HUB_R = 0.058
HUB_LEN = 0.13

REAR_AXLE_Z = REAR_WHEEL_R + RIM_TUBE_R     # rear wheels touch z=0
FRONT_AXLE_Z = FRONT_WHEEL_R + RIM_TUBE_R   # front wheels touch z=0

HALF_TRACK = 0.50          # wheel center distance from centerline
REAR_AXLE_X = -0.62
FRONT_AXLE_X = 0.62

# Body / bed (sits on the chassis above both axles)
BED_LEN = 1.70
BED_WIDTH = 0.86
BED_FLOOR_Z = REAR_AXLE_Z + 0.10
FLOOR_THK = 0.04
N_WALL_BOARDS = 6          # six stacked horizontal boards per side/end
PLANK_PITCH = 0.085        # vertical center-to-center board spacing
BOARD_H = 0.075            # individual board height (thickness when stacked)
SIDE_WALL_H = 0.05 + (N_WALL_BOARDS - 1) * PLANK_PITCH + BOARD_H  # ~0.55 m deep walls
PLANK_THK = 0.028
AXLE_R = 0.032


def _wall_board(length: float, thickness: float, height: float) -> Box:
    """Shared board geometry for stacked wall planks."""
    return Box((length, thickness, height))


def _wheel_visuals(part, mesh_prefix: str, radius: float, wood, dark_wood, iron) -> None:
    """Spoked wooden wheel in the local X-Z plane, spinning about local Y."""
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
    spoke_count = 12
    for s in range(spoke_count):
        a = 2.0 * math.pi * s / spoke_count
        part.visual(
            Cylinder(radius=SPOKE_R, length=length),
            origin=Origin(xyz=(mid * math.cos(a), 0.0, mid * math.sin(a)), rpy=(0.0, math.pi / 2.0 - a, 0.0)),
            material=wood,
            name=f"spoke_{s}",
        )

    part.visual(
        Cylinder(radius=HUB_R, length=HUB_LEN),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_wood,
        name="hub",
    )
    part.visual(
        Cylinder(radius=HUB_R + 0.009, length=0.022),
        origin=Origin(xyz=(0.0, HUB_LEN * 0.34, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="hub_band",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_wheeled_farm_wagon")

    wood = model.material("weathered_wood", rgba=(0.62, 0.55, 0.45, 1.0))
    dark_wood = model.material("dark_wood", rgba=(0.40, 0.34, 0.27, 1.0))
    plank = model.material("plank_wood", rgba=(0.68, 0.60, 0.49, 1.0))
    iron = model.material("wrought_iron", rgba=(0.17, 0.17, 0.18, 1.0))
    rope = model.material("rope", rgba=(0.72, 0.64, 0.42, 1.0))

    # ---- Body / chassis (root) --------------------------------------------
    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((BED_LEN, BED_WIDTH, 0.4)),
        mass=120.0,
        origin=Origin(xyz=(0.0, 0.0, BED_FLOOR_Z + 0.12)),
    )

    # Floor planks (run along X).
    n_floor = 6
    plank_w = BED_WIDTH / n_floor
    for i in range(n_floor):
        y = -BED_WIDTH / 2.0 + plank_w * (i + 0.5)
        body.visual(
            Box((BED_LEN, plank_w + 0.002, FLOOR_THK)),
            origin=Origin(xyz=(0.0, y, BED_FLOOR_Z)),
            material=plank,
            name=f"floor_plank_{i}",
        )

    # Deep side walls (+/-Y), N_WALL_BOARDS stacked horizontal boards each.
    for side_name, ysign in (("left", 1.0), ("right", -1.0)):
        y = ysign * (BED_WIDTH / 2.0 - PLANK_THK / 2.0)
        for i in range(N_WALL_BOARDS):
            z = BED_FLOOR_Z + FLOOR_THK / 2.0 + 0.05 + i * PLANK_PITCH
            body.visual(
                _wall_board(BED_LEN - 0.04, PLANK_THK, BOARD_H),
                origin=Origin(xyz=(0.0, y, z)),
                material=plank,
                name=f"{side_name}_side_board_{i}",
            )

    # End walls (front +X and rear -X), matching tall stacked boards + corner posts.
    for end_name, xsign in (("front", 1.0), ("rear", -1.0)):
        x = xsign * (BED_LEN / 2.0 - PLANK_THK / 2.0)
        for i in range(N_WALL_BOARDS):
            z = BED_FLOOR_Z + FLOOR_THK / 2.0 + 0.05 + i * PLANK_PITCH
            body.visual(
                _wall_board(PLANK_THK, BED_WIDTH - 0.03, BOARD_H),
                origin=Origin(xyz=(x, 0.0, z)),
                material=plank,
                name=f"{end_name}_end_board_{i}",
            )
    # Corner posts.
    for xsign in (1.0, -1.0):
        for ysign in (1.0, -1.0):
            body.visual(
                Box((0.05, 0.05, SIDE_WALL_H + 0.04)),
                origin=Origin(
                    xyz=(
                        xsign * (BED_LEN / 2.0 - 0.03),
                        ysign * (BED_WIDTH / 2.0 - 0.03),
                        BED_FLOOR_Z + SIDE_WALL_H / 2.0,
                    )
                ),
                material=dark_wood,
                name=f"post_{'f' if xsign > 0 else 'r'}_{'l' if ysign > 0 else 'r'}",
            )

    # Longitudinal chassis sills under the floor (carry both axle assemblies).
    for side_name, ysign in (("left", 1.0), ("right", -1.0)):
        body.visual(
            Box((BED_LEN + 0.20, 0.07, 0.09)),
            origin=Origin(xyz=(0.0, ysign * 0.30, BED_FLOOR_Z - FLOOR_THK / 2.0 - 0.045)),
            material=dark_wood,
            name=f"{side_name}_sill",
        )

    # Rear axle (fixed cross member carrying the rear wheels).
    body.visual(
        Cylinder(radius=AXLE_R, length=2.0 * HALF_TRACK + 0.12),
        origin=Origin(xyz=(REAR_AXLE_X, 0.0, REAR_AXLE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="rear_axle",
    )

    # Kingpin boss on the underside of the bed where the front bolster pivots.
    body.visual(
        Cylinder(radius=0.06, length=0.20),
        origin=Origin(xyz=(FRONT_AXLE_X, 0.0, BED_FLOOR_Z - FLOOR_THK / 2.0 - 0.10)),
        material=iron,
        name="kingpin_boss",
    )

    # ---- Front steering bolster (child of body; yaws about vertical kingpin) -
    bolster = model.part("front_bolster")
    bolster.inertial = Inertial.from_geometry(
        Box((0.16, 2.0 * HALF_TRACK, 0.10)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # The bolster frame sits just under the bed front; compute the front axle
    # height in the bolster's local frame.
    bolster_z = BED_FLOOR_Z - FLOOR_THK / 2.0 - 0.16  # world z of bolster origin (below the sills)
    axle_local_z = FRONT_AXLE_Z - bolster_z           # front axle z in bolster frame

    # Bolster beam (transverse) at the bolster origin plane. Kept short enough in
    # Y that it does not reach the front wheels.
    bolster.visual(
        Box((0.16, 2.0 * HALF_TRACK - 0.30, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_wood,
        name="bolster_beam",
    )
    # Front axle through the bolster (carries the front wheels).
    bolster.visual(
        Cylinder(radius=AXLE_R, length=2.0 * HALF_TRACK + 0.12),
        origin=Origin(xyz=(0.0, 0.0, axle_local_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="front_axle",
    )
    # Two short standards connecting the bolster beam down to the front axle.
    for ysign in (1.0, -1.0):
        bolster.visual(
            Box((0.07, 0.07, abs(axle_local_z) + 0.06)),
            origin=Origin(xyz=(0.0, ysign * 0.34, axle_local_z / 2.0)),
            material=dark_wood,
            name=f"standard_{'l' if ysign > 0 else 'r'}",
        )

    # Two long draw poles (shafts) extending forward from the bolster.
    pole_len = 1.30
    for side_name, ysign in (("left", 1.0), ("right", -1.0)):
        y = ysign * 0.20
        # Poles attach at the bolster axle and run forward at axle height (which
        # is below the body sills), staying clear of the bed underframe.
        pole = tube_from_spline_points(
            [
                (-0.06, ysign * 0.34, 0.0),
                (0.20, ysign * 0.30, -0.01),
                (0.55, y, -0.01),
                (1.00, y * 0.85, 0.0),
                (0.10 + pole_len, y * 0.70, 0.02),
            ],
            radius=0.028,
            samples_per_segment=10,
            radial_segments=12,
        )
        bolster.visual(mesh_from_geometry(pole, f"{side_name}_draw_pole"), material=wood)
    # Rope tie between the two poles near their forward end.
    rope_tie = tube_from_spline_points(
        [
            (1.00, 0.18, 0.04),
            (1.02, 0.0, -0.02),
            (1.00, -0.18, 0.04),
        ],
        radius=0.010,
        samples_per_segment=10,
        radial_segments=10,
    )
    bolster.visual(mesh_from_geometry(rope_tie, "rope_tie"), material=rope)

    # ---- Wheels ------------------------------------------------------------
    def make_wheel(name: str, radius: float):
        w = model.part(name)
        w.inertial = Inertial.from_geometry(
            Cylinder(radius=radius, length=HUB_LEN),
            mass=11.0,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        )
        _wheel_visuals(w, name, radius, wood, dark_wood, iron)
        return w

    rear_left = make_wheel("rear_left_wheel", REAR_WHEEL_R)
    rear_right = make_wheel("rear_right_wheel", REAR_WHEEL_R)
    front_left = make_wheel("front_left_wheel", FRONT_WHEEL_R)
    front_right = make_wheel("front_right_wheel", FRONT_WHEEL_R)

    # Steering bolster yaws about a vertical kingpin under the front of the bed.
    model.articulation(
        "front_steer",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bolster,
        origin=Origin(xyz=(FRONT_AXLE_X, 0.0, bolster_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=2.0, lower=-0.6, upper=0.6),
    )

    # Rear wheels spin on the fixed rear axle (children of body).
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

    # Front wheels spin AND swing with the bolster (children of the bolster).
    # Bolster frame origin is at (FRONT_AXLE_X, 0, bolster_z); express front-wheel
    # mounts in the bolster's local frame.
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

    # Intentional turntable pivot: the body kingpin boss seats into the front
    # bolster beam (this is the steering pivot interface, not a stray collision).
    ctx.allow_overlap(body, bolster, elem_a="kingpin_boss", elem_b="bolster_beam",
                      reason="The kingpin boss seats into the bolster beam at the steering turntable.")
    ctx.allow_overlap(body, bolster, elem_a="kingpin_boss", elem_b="front_axle",
                      reason="The kingpin passes down through the bolster to the front axle at the turntable hub.")

    # Intentional axle-through-hub mounts (rear axle on body; front axle on bolster).
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

    # --- All four wheels touch the ground; front pair smaller than rear pair ---
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

    # Front wheels strictly smaller than rear wheels.
    fl_aabb = ctx.part_world_aabb(fl)
    rl_aabb = ctx.part_world_aabb(rl)
    assert fl_aabb and rl_aabb
    ctx.check("front_smaller_than_rear",
              (fl_aabb[1][2] - fl_aabb[0][2]) < (rl_aabb[1][2] - rl_aabb[0][2]) - 0.05,
              "front pair must be visibly smaller than rear pair")

    # --- Left/right symmetry of each axle ---
    for la, ra, label in ((rl, rr, "rear"), (fl, fr, "front")):
        lp = ctx.part_world_position(la)
        rp = ctx.part_world_position(ra)
        assert lp and rp
        ctx.check(f"{label}_wheels_symmetric_y", abs(lp[1] + rp[1]) < 1e-6, f"{lp[1]} vs {rp[1]}")

    # --- Engagement: each wheel hub overlaps its axle (no detached gap) ---
    ctx.expect_overlap(rl, body, axes="xz", min_overlap=0.02, elem_b="rear_axle",
                       name="rear_left_engages_axle")
    ctx.expect_overlap(rr, body, axes="xz", min_overlap=0.02, elem_b="rear_axle",
                       name="rear_right_engages_axle")
    ctx.expect_overlap(fl, bolster, axes="xz", min_overlap=0.02, elem_b="front_axle",
                       name="front_left_engages_axle")
    ctx.expect_overlap(fr, bolster, axes="xz", min_overlap=0.02, elem_b="front_axle",
                       name="front_right_engages_axle")

    # --- Decisive spin pose: a rear spoke sweeps to a new place when rolled ---
    spoke_rest = ctx.part_element_world_aabb(rl, elem="spoke_0")
    with ctx.pose({rl_spin: math.pi / 2.0}):
        spoke_turned = ctx.part_element_world_aabb(rl, elem="spoke_0")
    assert spoke_rest and spoke_turned
    ctx.check("rear_wheel_spins",
              abs(spoke_turned[0][0] - spoke_rest[0][0]) > 0.02 or abs(spoke_turned[0][2] - spoke_rest[0][2]) > 0.02,
              f"rest={spoke_rest[0]} turned={spoke_turned[0]}")

    # --- Decisive steer pose: turning the bolster swings the front wheels in Y ---
    fl_rest = ctx.part_world_position(fl)
    with ctx.pose({steer: 0.5}):
        fl_steered = ctx.part_world_position(fl)
    assert fl_rest and fl_steered
    ctx.check("front_wheels_steer_with_bolster",
              abs(fl_steered[0] - fl_rest[0]) > 0.02 or abs(fl_steered[1] - fl_rest[1]) > 0.02,
              f"rest={fl_rest} steered={fl_steered}")

    # --- Deep grain-box walls: SIX stacked boards per side and per end ---
    body_visuals = [v.name for v in body.visuals]
    for side in ("left", "right"):
        for i in range(N_WALL_BOARDS):
            ctx.check(f"{side}_side_board_{i}_exists",
                      f"{side}_side_board_{i}" in body_visuals,
                      f"missing {side}_side_board_{i}")
    for end in ("front", "rear"):
        for i in range(N_WALL_BOARDS):
            ctx.check(f"{end}_end_board_{i}_exists",
                      f"{end}_end_board_{i}" in body_visuals,
                      f"missing {end}_end_board_{i}")

    # Top board is well above the floor (deep-walled grain hauler, not low rails).
    top_board = ctx.part_element_world_aabb(body, elem="left_side_board_5")
    floor_aabb = ctx.part_element_world_aabb(body, elem="floor_plank_0")
    assert top_board and floor_aabb
    wall_height_above_floor = top_board[1][2] - floor_aabb[1][2]
    ctx.check("deep_walls_at_least_0_40m",
              wall_height_above_floor > 0.40,
              f"wall height above floor top = {wall_height_above_floor:.3f}")

    return ctx.report()


object_model = build_object_model()
