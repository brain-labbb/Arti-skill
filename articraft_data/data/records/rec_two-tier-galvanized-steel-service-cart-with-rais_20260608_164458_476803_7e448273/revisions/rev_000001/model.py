from __future__ import annotations

# Two-tier galvanized steel service cart (utility / bus cart).
#
# World frame: Z up. Long axis of the trays is X, width is Y. The four caster
# wheels touch z ~ 0.
#
# Reading the reference (009.png):
#   - Two GALVANIZED steel trays (a lower and an upper shelf), each a shallow pan
#     with a RAISED LIP around all four edges.
#   - The trays are spaced apart by four corner tube legs.
#   - A tubular PUSH HANDLE rises above the top tray at BOTH short ends (the
#     corner legs continue up past the top tray and bend over into a handle).
#   - Four swivel casters at the bottom corners.
#
# Articulation (primary, user-facing):
#   - Each of the 4 swivel casters YAWS about a vertical kingpin (continuous) and
#     each wheel ROLLS about its horizontal axle (continuous). A service cart is
#     pushed and steers on its casters.
#
# Root part = lower_tray. Upper tray, corner legs, and handles are FIXED to it.
# Each caster: lower_tray --(continuous Z yaw)--> yoke --(continuous spin)--> wheel.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TireGeometry,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---- key dimensions (meters) ----
TRAY_LEN = 0.78  # X
TRAY_WID = 0.46  # Y
TRAY_THK = 0.014  # pan floor thickness
LIP_H = 0.040  # raised edge lip height
LIP_T = 0.012  # lip wall thickness

WHEEL_RADIUS = 0.052
WHEEL_WIDTH = 0.034
FORK_OFFSET = 0.032

LOWER_TRAY_Z = 0.175  # lower pan floor (top surface) height
UPPER_TRAY_Z = 0.86  # upper pan floor (top surface) height
LEG_R = 0.013  # corner leg tube radius

# Corner leg / caster positions
POST_X = TRAY_LEN / 2.0 - 0.045
POST_Y = TRAY_WID / 2.0 - 0.040
CASTER_X = TRAY_LEN / 2.0 - 0.070
CASTER_Y = TRAY_WID / 2.0 - 0.060

HANDLE_TOP_Z = UPPER_TRAY_Z + 0.16  # handle rises above the top tray
HANDLE_OUT = 0.035  # handle bows outward past the tray end


def _tube(points, name, *, radius=LEG_R):
    geom = tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=10,
        radial_segments=14,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, name)


def _tray_visuals(part, floor_z_center, material):
    """A shallow steel pan: floor slab plus four raised edge lips."""
    part.visual(
        Box((TRAY_LEN, TRAY_WID, TRAY_THK)),
        origin=Origin(xyz=(0.0, 0.0, floor_z_center)),
        material=material,
        name="pan_floor",
    )
    lip_z = floor_z_center + LIP_H / 2.0
    # long-side lips (run along X)
    for sy in (-1.0, 1.0):
        part.visual(
            Box((TRAY_LEN, LIP_T, LIP_H)),
            origin=Origin(xyz=(0.0, sy * (TRAY_WID / 2.0 - LIP_T / 2.0), lip_z)),
            material=material,
            name=f"lip_side_{'p' if sy > 0 else 'n'}",
        )
    # short-end lips (run along Y)
    for sx in (-1.0, 1.0):
        part.visual(
            Box((LIP_T, TRAY_WID, LIP_H)),
            origin=Origin(xyz=(sx * (TRAY_LEN / 2.0 - LIP_T / 2.0), 0.0, lip_z)),
            material=material,
            name=f"lip_end_{'p' if sx > 0 else 'n'}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_tier_service_cart")

    galv = model.material("galvanized", rgba=(0.66, 0.67, 0.69, 1.0))
    galv_leg = model.material("galvanized_leg", rgba=(0.60, 0.61, 0.63, 1.0))
    rubber = model.material("caster_tread", rgba=(0.14, 0.14, 0.16, 1.0))
    steel = model.material("caster_steel", rgba=(0.50, 0.51, 0.53, 1.0))
    rim_silver = model.material("rim_silver", rgba=(0.78, 0.79, 0.81, 1.0))

    # ---- lower tray (root) ----
    lower = model.part("lower_tray")
    _tray_visuals(lower, LOWER_TRAY_Z - TRAY_THK / 2.0, galv)
    lower.inertial = Inertial.from_geometry(
        Box((TRAY_LEN, TRAY_WID, TRAY_THK)), mass=10.0,
        origin=Origin(xyz=(0.0, 0.0, LOWER_TRAY_Z)),
    )

    # ---- upper tray (FIXED to lower via legs) ----
    upper = model.part("upper_tray")
    _tray_visuals(upper, UPPER_TRAY_Z - TRAY_THK / 2.0, galv)
    upper.inertial = Inertial.from_geometry(
        Box((TRAY_LEN, TRAY_WID, TRAY_THK)), mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, UPPER_TRAY_Z)),
    )
    model.articulation("lower_to_upper", ArticulationType.FIXED, parent=lower, child=upper)

    # ---- end frames: at each short end, two corner legs continue up past the top
    # tray and bow over into a push handle loop (one continuous tube run per end).
    # Each end frame is its own part (a single connected island), FIXED to the lower
    # tray.
    leg_base_z = LOWER_TRAY_Z - TRAY_THK  # legs start at the lower pan underside
    end_frames = {}
    for sx in (-1.0, 1.0):
        tag = "p" if sx > 0 else "n"
        ef = model.part(f"end_frame_{tag}")
        hx = sx * POST_X
        # one swept run: up the -Y leg, over the handle, down the +Y leg
        run = [
            (hx, -POST_Y, leg_base_z),
            (hx, -POST_Y, UPPER_TRAY_Z),
            (hx, -POST_Y, HANDLE_TOP_Z - 0.05),
            (hx + sx * HANDLE_OUT, -POST_Y * 0.9, HANDLE_TOP_Z),
            (hx + sx * HANDLE_OUT, 0.0, HANDLE_TOP_Z + 0.008),
            (hx + sx * HANDLE_OUT, POST_Y * 0.9, HANDLE_TOP_Z),
            (hx, POST_Y, HANDLE_TOP_Z - 0.05),
            (hx, POST_Y, UPPER_TRAY_Z),
            (hx, POST_Y, leg_base_z),
        ]
        ef.visual(_tube(run, f"end_frame_{tag}_run"), material=galv_leg,
                  name=f"end_frame_{tag}_run")
        ef.inertial = Inertial.from_geometry(
            Box((0.06, TRAY_WID, HANDLE_TOP_Z - leg_base_z)), mass=3.0,
            origin=Origin(xyz=(hx, 0.0, (HANDLE_TOP_Z + leg_base_z) / 2.0)),
        )
        model.articulation(f"lower_to_end_frame_{tag}", ArticulationType.FIXED,
                           parent=lower, child=ef)
        end_frames[tag] = ef

    # ---- four swivel casters under the lower tray ----
    caster_top_z = leg_base_z  # casters mount to the lower pan underside
    leg_bottom_z = -(caster_top_z - WHEEL_RADIUS)
    leg_top_z = leg_bottom_z + WHEEL_RADIUS + 0.012
    leg_half_y = WHEEL_WIDTH / 2.0 + 0.011

    def add_caster(idx, cx, cy):
        yoke = model.part(f"caster_yoke_{idx}")
        yoke.visual(
            Box((0.058, 0.058, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, -0.005)),
            material=steel,
            name="swivel_plate",
        )
        yoke.visual(
            Cylinder(radius=0.013, length=0.044),
            origin=Origin(xyz=(0.0, 0.0, -0.026)),
            material=steel,
            name="kingpin",
        )
        yoke.visual(
            Box((FORK_OFFSET + 0.028, 0.040, 0.014)),
            origin=Origin(xyz=(-FORK_OFFSET / 2.0, 0.0, -0.044)),
            material=steel,
            name="offset_bracket",
        )
        yoke.visual(
            Box((0.032, leg_half_y * 2.0 + 0.014, 0.014)),
            origin=Origin(xyz=(-FORK_OFFSET, 0.0, leg_top_z)),
            material=steel,
            name="fork_crown",
        )
        leg_h = abs(leg_bottom_z - leg_top_z) + 0.016
        for sy in (-1.0, 1.0):
            yoke.visual(
                Box((0.015, 0.013, leg_h)),
                origin=Origin(
                    xyz=(-FORK_OFFSET, sy * leg_half_y, (leg_top_z + leg_bottom_z) / 2.0)
                ),
                material=steel,
                name=f"fork_leg_{'p' if sy > 0 else 'n'}",
            )
        yoke.visual(
            Cylinder(radius=0.006, length=leg_half_y * 2.0 + 0.020),
            origin=Origin(xyz=(-FORK_OFFSET, 0.0, leg_bottom_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="axle",
        )
        yoke.inertial = Inertial.from_geometry(
            Box((0.06, 0.07, 0.10)), mass=0.5,
            origin=Origin(xyz=(-FORK_OFFSET, 0.0, -0.05)),
        )

        wheel = model.part(f"caster_wheel_{idx}")
        rim_geom = WheelGeometry(
            WHEEL_RADIUS * 0.66,
            WHEEL_WIDTH * 0.7,
            rim=WheelRim(inner_radius=WHEEL_RADIUS * 0.55, flange_height=0.004),
            hub=WheelHub(radius=WHEEL_RADIUS * 0.45, width=WHEEL_WIDTH * 0.8, cap_style="flat"),
            face=WheelFace(dish_depth=0.0),
            spokes=WheelSpokes(style="disc"),
            bore=WheelBore(style="round", diameter=0.008),
        )
        rim_geom.rotate_z(math.pi / 2.0)
        wheel.visual(mesh_from_geometry(rim_geom, f"caster_rim_{idx}"),
                     material=rim_silver, name="rim")
        tire_geom = TireGeometry(
            WHEEL_RADIUS,
            WHEEL_WIDTH,
            inner_radius=WHEEL_RADIUS * 0.64,
            tread=TireTread(style="circumferential", depth=0.002, count=1),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
        )
        tire_geom.rotate_z(math.pi / 2.0)
        wheel.visual(mesh_from_geometry(tire_geom, f"caster_tire_{idx}"),
                     material=rubber, name="tire")
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH), mass=0.4,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        )

        model.articulation(
            f"lower_to_caster_yoke_{idx}",
            ArticulationType.CONTINUOUS,
            parent=lower,
            child=yoke,
            origin=Origin(xyz=(cx, cy, caster_top_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=12.0),
        )
        model.articulation(
            f"caster_spin_{idx}",
            ArticulationType.CONTINUOUS,
            parent=yoke,
            child=wheel,
            origin=Origin(xyz=(-FORK_OFFSET, 0.0, leg_bottom_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=40.0),
        )

    add_caster(0, CASTER_X, CASTER_Y)
    add_caster(1, CASTER_X, -CASTER_Y)
    add_caster(2, -CASTER_X, CASTER_Y)
    add_caster(3, -CASTER_X, -CASTER_Y)

    return model


def run_tests():
    from sdk import TestContext

    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_tray")
    upper = object_model.get_part("upper_tray")
    end_p = object_model.get_part("end_frame_p")
    end_n = object_model.get_part("end_frame_n")
    ctx.check("lower_tray_present", lower is not None, "missing")

    # Captured axle in each wheel hub: scoped intentional overlap.
    for i in range(4):
        ctx.allow_overlap(
            object_model.get_part(f"caster_yoke_{i}"),
            object_model.get_part(f"caster_wheel_{i}"),
            elem_a="axle", elem_b="rim",
            reason="The fork axle is captured inside the wheel hub bore.",
        )

    # The end-frame legs pass through / weld to both tray pan floors: scoped overlap.
    for tag in ("p", "n"):
        ef = object_model.get_part(f"end_frame_{tag}")
        for tray in (lower, upper):
            ctx.allow_overlap(
                ef, tray,
                elem_a=f"end_frame_{tag}_run", elem_b="pan_floor",
                reason="The corner legs pass through and weld to the tray pan floors.",
            )

    # Two trays stacked: upper clearly above lower.
    la = ctx.part_world_aabb(lower)
    ua = ctx.part_world_aabb(upper)
    if la is not None and ua is not None:
        ctx.check("upper_above_lower",
                  ua[0][2] > la[1][2] + 0.20,
                  f"upper_bottom={ua[0][2]:.3f} lower_top={la[1][2]:.3f}")
    # Lower tray rides above the floor on the casters.
    if la is not None:
        ctx.check("lower_above_floor", 0.09 <= la[0][2] <= 0.18,
                  f"lower_bottom={la[0][2]:.3f}")

    # Wheels touch the floor.
    lows = []
    for i in range(4):
        wa = ctx.part_world_aabb(object_model.get_part(f"caster_wheel_{i}"))
        if wa is not None:
            lows.append(wa[0][2])
    ctx.check("four_wheels", len(lows) == 4, f"found {len(lows)}")
    if lows:
        ctx.check("wheels_touch_floor", all(abs(z) <= 0.012 for z in lows),
                  f"lows={['%.3f' % z for z in lows]}")

    # Handles rise above the top tray at both ends.
    hp = ctx.part_world_aabb(end_p)
    hn = ctx.part_world_aabb(end_n)
    if hp is not None and hn is not None and ua is not None:
        ctx.check("handle_p_above_top_tray", hp[1][2] > ua[1][2] + 0.08,
                  f"end_p_top={hp[1][2]:.3f} upper_top={ua[1][2]:.3f}")
        ctx.check("handle_n_above_top_tray", hn[1][2] > ua[1][2] + 0.08,
                  f"end_n_top={hn[1][2]:.3f} upper_top={ua[1][2]:.3f}")
        # the two end frames are at opposite short ends
        ctx.check("end_frames_opposite",
                  hp[1][0] > 0.2 and hn[0][0] < -0.2,
                  f"end_p_xmax={hp[1][0]:.3f} end_n_xmin={hn[0][0]:.3f}")

    # Raised lip on the upper tray makes the pan readable (lip taller than floor).
    floor_a = ctx.part_element_world_aabb(upper, elem="pan_floor")
    lip_a = ctx.part_element_world_aabb(upper, elem="lip_side_p")
    if floor_a is not None and lip_a is not None:
        ctx.check("upper_lip_raised", lip_a[1][2] > floor_a[1][2] + 0.02,
                  f"lip_top={lip_a[1][2]:.3f} floor_top={floor_a[1][2]:.3f}")

    # Joint inventory + spin/swivel correctness.
    for i in range(4):
        sw = object_model.get_articulation(f"lower_to_caster_yoke_{i}")
        sp = object_model.get_articulation(f"caster_spin_{i}")
        ctx.check(f"swivel_{i}_continuous_z",
                  sw.articulation_type == ArticulationType.CONTINUOUS
                  and tuple(sw.axis) == (0.0, 0.0, 1.0), f"axis={sw.axis}")
        ctx.check(f"spin_{i}_continuous_y",
                  sp.articulation_type == ArticulationType.CONTINUOUS
                  and tuple(sp.axis) == (0.0, 1.0, 0.0), f"axis={sp.axis}")

    # Decisive pose: wheel spins in place (center fixed).
    wheel0 = object_model.get_part("caster_wheel_0")
    spin0 = object_model.get_articulation("caster_spin_0")
    rest = ctx.part_world_position(wheel0)
    with ctx.pose({spin0: 0.6}):
        turned = ctx.part_world_position(wheel0)
    if rest is not None and turned is not None:
        moved = sum((turned[k] - rest[k]) ** 2 for k in range(3)) ** 0.5
        ctx.check("wheel_spins_in_place", moved < 1e-4, f"moved={moved:.5f}")

    return ctx.report()


object_model = build_object_model()
