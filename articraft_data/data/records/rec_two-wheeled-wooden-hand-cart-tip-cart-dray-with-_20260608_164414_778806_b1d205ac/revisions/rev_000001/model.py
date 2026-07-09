from __future__ import annotations

# Two-wheeled wooden hand cart (tip cart / dray).
#
# World frame: Z-up. The cart rolls on two large spoked wooden wheels on a single
# shared axle near the rear-bottom of the body. The box body has low plank side
# walls and a tall plank back wall. Two angled wooden legs at the front prop the
# cart level when parked (their feet rest on z~=0, same ground plane as the
# wheels). Two long wooden pull shafts run forward from under the body floor.
#
# +X = forward (toward the pull shafts / front legs).
# +Y = left.  -Y = right.  Wheels sit on the +/-Y sides on the shared axle.
#
# Articulation (PRIMARY = wheels rolling):
#   - left_wheel  : CONTINUOUS spin about the axle (world Y at rest).
#   - right_wheel : CONTINUOUS spin about the axle (world Y at rest).
# The wheels share the single rear axle and actually wrap around the axle stubs
# (no detached legs-beside-wheels gap): the axle stub passes through each wheel hub.

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

# ---- Principal dimensions (meters) -----------------------------------------
WHEEL_RADIUS = 0.30          # ~0.60 m tall wheels
WHEEL_HALF_TRACK = 0.46      # axle stub center distance from body centerline
RIM_TUBE_R = 0.022           # wooden felloe (rim) cross-section radius
SPOKE_R = 0.014              # spoke radius
SPOKE_COUNT = 12
HUB_R = 0.055
HUB_LEN = 0.12
WHEEL_PLANE_HALF = 0.05      # half thickness band the wheel occupies along Y

AXLE_Z = WHEEL_RADIUS + RIM_TUBE_R   # axle height so the rim outer touches z=0
AXLE_X = -0.18               # axle sits toward the rear of the body
AXLE_R = 0.030
AXLE_LEN = 2.0 * WHEEL_HALF_TRACK + 0.10

# Body box (cargo bed)
BODY_LEN = 0.92              # along X
BODY_WIDTH = 0.74           # along Y
FLOOR_Z = AXLE_Z + 0.04      # floor sits just above the axle
FLOOR_THK = 0.035
SIDE_WALL_H = 0.20           # low side walls
BACK_WALL_H = 0.40           # tall back wall (at -X end)
PLANK_THK = 0.025


def _wheel_visuals(part, mesh_prefix: str, wood, dark_wood, iron) -> None:
    """Build a spoked wooden wheel lying in the local X-Z plane, spinning about local Y.

    The wheel is authored centered on the local origin; its spin axis is local Y.
    Rim/spokes lie in the X-Z plane (so the wheel face normal is Y).
    """
    # Felloe (rim) ring: a closed tube swept around in the local X-Z plane.
    ring_pts = []
    seg = 48
    for i in range(seg + 1):
        a = 2.0 * math.pi * i / seg
        ring_pts.append((WHEEL_RADIUS * math.cos(a), 0.0, WHEEL_RADIUS * math.sin(a)))
    rim = tube_from_spline_points(
        ring_pts,
        radius=RIM_TUBE_R,
        samples_per_segment=2,
        radial_segments=12,
        closed_spline=True,
    )
    part.visual(mesh_from_geometry(rim, f"{mesh_prefix}_rim"), material=wood, name="rim")

    # Spokes: radial cylinders that root inside the hub and reach into the rim,
    # so each spoke physically connects hub -> felloe (no island gaps).
    inner = HUB_R - 0.015
    outer = WHEEL_RADIUS + 0.004
    mid = 0.5 * (inner + outer)
    length = outer - inner
    for s in range(SPOKE_COUNT):
        a = 2.0 * math.pi * s / SPOKE_COUNT
        cx = mid * math.cos(a)
        cz = mid * math.sin(a)
        # A cylinder's long axis is local Z; rotate it to point radially in X-Z.
        # Direction (cos a, 0, sin a); pitch about Y so Z->that direction.
        pitch = math.pi / 2.0 - a
        part.visual(
            Cylinder(radius=SPOKE_R, length=length),
            origin=Origin(xyz=(cx, 0.0, cz), rpy=(0.0, pitch, 0.0)),
            material=wood,
            name=f"spoke_{s}",
        )

    # Hub: a cylinder along local Y (the spin axis).
    part.visual(
        Cylinder(radius=HUB_R, length=HUB_LEN),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_wood,
        name="hub",
    )
    # Iron hub band / nave ring for realism.
    part.visual(
        Cylinder(radius=HUB_R + 0.008, length=0.020),
        origin=Origin(xyz=(0.0, HUB_LEN * 0.35, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="hub_band",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_wheeled_hand_cart")

    wood = model.material("oak_wood", rgba=(0.74, 0.58, 0.36, 1.0))
    dark_wood = model.material("dark_wood", rgba=(0.45, 0.33, 0.20, 1.0))
    plank = model.material("plank_wood", rgba=(0.80, 0.65, 0.43, 1.0))
    iron = model.material("wrought_iron", rgba=(0.16, 0.16, 0.17, 1.0))

    # ---- Body (root part) --------------------------------------------------
    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((BODY_LEN, BODY_WIDTH, 0.4)),
        mass=55.0,
        origin=Origin(xyz=(0.0, 0.0, FLOOR_Z + 0.1)),
    )

    # Floor planks (run along X). Lay a few boards across the width.
    n_floor = 5
    plank_w = BODY_WIDTH / n_floor
    for i in range(n_floor):
        y = -BODY_WIDTH / 2.0 + plank_w * (i + 0.5)
        body.visual(
            Box((BODY_LEN, plank_w * 0.94, FLOOR_THK)),
            origin=Origin(xyz=(0.0, y, FLOOR_Z)),
            material=plank,
            name=f"floor_plank_{i}",
        )

    # Side walls (low) on +/-Y, each made of 3 horizontal planks with gaps.
    for side_name, ysign in (("left", 1.0), ("right", -1.0)):
        y = ysign * (BODY_WIDTH / 2.0 - PLANK_THK / 2.0)
        for k in range(3):
            z = FLOOR_Z + FLOOR_THK / 2.0 + 0.035 + k * 0.07
            body.visual(
                Box((BODY_LEN - 0.06, PLANK_THK, 0.05)),
                origin=Origin(xyz=(0.0, y, z)),
                material=plank,
                name=f"{side_name}_side_plank_{k}",
            )

    # Tall back wall at -X end (3 stacked planks + 2 corner posts).
    back_x = -BODY_LEN / 2.0 + PLANK_THK / 2.0
    for k in range(4):
        z = FLOOR_Z + FLOOR_THK / 2.0 + 0.045 + k * 0.095
        body.visual(
            Box((PLANK_THK, BODY_WIDTH - 0.04, 0.085)),
            origin=Origin(xyz=(back_x, 0.0, z)),
            material=plank,
            name=f"back_plank_{k}",
        )
    for side_name, ysign in (("left", 1.0), ("right", -1.0)):
        body.visual(
            Box((0.06, 0.06, BACK_WALL_H + 0.05)),
            origin=Origin(
                xyz=(back_x, ysign * (BODY_WIDTH / 2.0 - 0.03), FLOOR_Z + BACK_WALL_H / 2.0)
            ),
            material=dark_wood,
            name=f"{side_name}_back_post",
        )

    # Front low rail (small board across the front edge, +X).
    front_x = BODY_LEN / 2.0 - PLANK_THK / 2.0
    body.visual(
        Box((PLANK_THK, BODY_WIDTH - 0.04, 0.09)),
        origin=Origin(xyz=(front_x, 0.0, FLOOR_Z + FLOOR_THK / 2.0 + 0.02)),
        material=plank,
        name="front_rail",
    )

    # Two longitudinal chassis beams under the floor (carry the axle).
    for side_name, ysign in (("left", 1.0), ("right", -1.0)):
        body.visual(
            Box((BODY_LEN + 0.10, 0.06, 0.07)),
            origin=Origin(xyz=(0.0, ysign * 0.22, FLOOR_Z - FLOOR_THK / 2.0 - 0.035)),
            material=dark_wood,
            name=f"{side_name}_chassis_beam",
        )

    # ---- Axle: a fixed cross member under the body that carries the wheels ---
    body.visual(
        Cylinder(radius=AXLE_R, length=AXLE_LEN),
        origin=Origin(xyz=(AXLE_X, 0.0, AXLE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="axle",
    )

    # ---- Front support legs (angled props down to the ground) --------------
    # Each leg is a slanted post from under the floor down to a foot at z~=0,
    # ahead of the axle, holding the parked cart level.
    leg_top_x = front_x - 0.12
    leg_foot_x = leg_top_x - 0.05
    leg_len = math.hypot(FLOOR_Z, 0.05) + 0.02
    leg_pitch = math.atan2(0.05, FLOOR_Z)  # small forward lean
    for side_name, ysign in (("left", 1.0), ("right", -1.0)):
        y = ysign * (BODY_WIDTH / 2.0 - 0.10)
        leg_cx = 0.5 * (leg_top_x + leg_foot_x)
        body.visual(
            Box((0.06, 0.06, leg_len)),
            origin=Origin(xyz=(leg_cx, y, FLOOR_Z / 2.0), rpy=(0.0, leg_pitch, 0.0)),
            material=dark_wood,
            name=f"{side_name}_leg",
        )
        # Small foot pad at the bottom (rests on the ground).
        body.visual(
            Box((0.10, 0.07, 0.03)),
            origin=Origin(xyz=(leg_foot_x, y, 0.015)),
            material=dark_wood,
            name=f"{side_name}_leg_foot",
        )

    # ---- Two long pull shafts (handles) running forward ---------------------
    shaft_len = 1.05
    shaft_x0 = front_x
    shaft_x1 = front_x + shaft_len
    for side_name, ysign in (("left", 1.0), ("right", -1.0)):
        y = ysign * 0.26
        z = FLOOR_Z - FLOOR_THK / 2.0 - 0.02
        shaft = tube_from_spline_points(
            [
                (shaft_x0 - 0.30, y, z + 0.01),
                (shaft_x0 + 0.10, y, z),
                (shaft_x1 - 0.30, y, z - 0.005),
                (shaft_x1, y * 0.92, z - 0.01),
            ],
            radius=0.026,
            samples_per_segment=10,
            radial_segments=12,
        )
        body.visual(mesh_from_geometry(shaft, f"{side_name}_shaft"), material=wood)
    # Cross brace tying the two shafts near their forward end.
    body.visual(
        Box((0.05, 0.60, 0.04)),
        origin=Origin(xyz=(shaft_x1 - 0.18, 0.0, FLOOR_Z - 0.07)),
        material=dark_wood,
        name="shaft_crossbrace",
    )

    # ---- Wheels ------------------------------------------------------------
    left_wheel = model.part("left_wheel")
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=HUB_LEN),
        mass=9.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    _wheel_visuals(left_wheel, "left_wheel", wood, dark_wood, iron)

    right_wheel = model.part("right_wheel")
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=HUB_LEN),
        mass=9.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    _wheel_visuals(right_wheel, "right_wheel", wood, dark_wood, iron)

    # Wheels mount on the axle stubs at +/-Y; spin about world Y at rest.
    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=left_wheel,
        origin=Origin(xyz=(AXLE_X, WHEEL_HALF_TRACK, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=20.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=right_wheel,
        origin=Origin(xyz=(AXLE_X, -WHEEL_HALF_TRACK, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    left_spin = object_model.get_articulation("left_wheel_spin")
    right_spin = object_model.get_articulation("right_wheel_spin")

    # The axle stub passes through each wheel hub (real wood-cart construction:
    # the wheel rides on the axle end). This local hub/axle interpenetration is
    # the mechanical mount, not a wrong joint origin.
    ctx.allow_overlap(
        body,
        left_wheel,
        elem_a="axle",
        elem_b="hub",
        reason="The single rear axle stub passes through the left wheel hub bore.",
    )
    ctx.allow_overlap(
        body,
        right_wheel,
        elem_a="axle",
        elem_b="hub",
        reason="The single rear axle stub passes through the right wheel hub bore.",
    )
    ctx.allow_overlap(
        body,
        left_wheel,
        elem_a="axle",
        elem_b="hub_band",
        reason="The axle stub also passes through the left wheel's iron nave band.",
    )
    ctx.allow_overlap(
        body,
        right_wheel,
        elem_a="axle",
        elem_b="hub_band",
        reason="The axle stub also passes through the right wheel's iron nave band.",
    )

    # --- Primary articulation: both wheels are CONTINUOUS spin about Y ---
    for jname, joint in (("left_wheel_spin", left_spin), ("right_wheel_spin", right_spin)):
        ctx.check(
            f"{jname}_is_continuous",
            str(joint.joint_type).lower().endswith("continuous"),
            f"{jname} type={joint.joint_type}",
        )
        ax = joint.axis
        ctx.check(
            f"{jname}_axis_is_y",
            abs(abs(ax[1]) - 1.0) < 1e-6 and abs(ax[0]) < 1e-6 and abs(ax[2]) < 1e-6,
            f"axis={ax}",
        )

    # --- Wheels touch the ground (z ~= 0) and body floor is above 0 ---
    for name, wheel in (("left_wheel", left_wheel), ("right_wheel", right_wheel)):
        aabb = ctx.part_world_aabb(wheel)
        assert aabb is not None
        mins, maxs = aabb
        ctx.check(f"{name}_touches_ground", abs(mins[2]) < 0.01, f"min_z={mins[2]:.4f}")
        diameter = maxs[2] - mins[2]
        ctx.check(
            f"{name}_diameter",
            0.60 <= diameter <= 0.68,
            f"diameter={diameter:.3f}",
        )

    body_aabb = ctx.part_world_aabb(body)
    assert body_aabb is not None
    bmins, bmaxs = body_aabb
    # The legs/feet should reach the ground so the parked cart sits level.
    ctx.check("body_reaches_ground", bmins[2] < 0.03, f"body_min_z={bmins[2]:.4f}")

    # --- Left/right symmetry of the wheel mounts ---
    lp = ctx.part_world_position(left_wheel)
    rp = ctx.part_world_position(right_wheel)
    assert lp is not None and rp is not None
    ctx.check("wheels_symmetric_y", abs(lp[1] + rp[1]) < 1e-6, f"ly={lp[1]}, ry={rp[1]}")
    ctx.check("wheels_same_x", abs(lp[0] - rp[0]) < 1e-6, f"lx={lp[0]}, rx={rp[0]}")
    ctx.check("wheels_same_z", abs(lp[2] - rp[2]) < 1e-6, f"lz={lp[2]}, rz={rp[2]}")

    # --- Wheel hub actually wraps the axle stub (no detached gap): the wheel
    #     part overlaps the body's axle region in the X-Z footprint. ---
    ctx.expect_overlap(
        left_wheel,
        body,
        axes="xz",
        min_overlap=0.02,
        elem_b="axle",
        name="left_wheel_engages_axle",
    )
    ctx.expect_overlap(
        right_wheel,
        body,
        axes="xz",
        min_overlap=0.02,
        elem_b="axle",
        name="right_wheel_engages_axle",
    )

    # --- Decisive spin pose: rolling the wheel sweeps a single spoke (a
    #     rotationally non-symmetric element) to a new X/Z location while the
    #     hub center stays fixed. ---
    spoke_rest = ctx.part_element_world_aabb(left_wheel, elem="spoke_0")
    with ctx.pose({left_spin: math.pi / 2.0}):
        spoke_turned = ctx.part_element_world_aabb(left_wheel, elem="spoke_0")
        hub_turned = ctx.part_world_position(left_wheel)
    assert spoke_rest is not None and spoke_turned is not None and hub_turned is not None
    ctx.check(
        "wheel_actually_spins",
        abs(spoke_turned[0][0] - spoke_rest[0][0]) > 0.02
        or abs(spoke_turned[0][2] - spoke_rest[0][2]) > 0.02,
        f"spoke AABB should shift when rolled: rest={spoke_rest[0]}, turned={spoke_turned[0]}",
    )
    ctx.check(
        "hub_center_fixed_on_spin",
        abs(hub_turned[1] - WHEEL_HALF_TRACK) < 1e-6,
        f"hub y={hub_turned[1]}",
    )

    return ctx.report()


object_model = build_object_model()
