from __future__ import annotations

# Two-tier service / bus cart (fork of flat platform utility cart).
#
# World frame: Z up, tray planes horizontal. The long axis is X, width is Y.
# Footprint (caster wheels) touches z ~ 0.
#
# Structure:
#   - Two shallow rectangular tray pans (lower + upper) with raised rim lips,
#     spaced apart by four corner tube legs.
#   - A tall tubular push handle at the +X end and a shorter end guard at -X,
#     both rising from the upper tray.
#   - Four swivel casters under the lower tray.
#
# Articulation (primary, user-facing):
#   - Each of 4 swivel casters YAWS about a vertical (Z) kingpin (continuous),
#     and each wheel ROLLS about its horizontal axle (continuous).
#
# Part tree:
#   lower_tray (root)
#     |-- caster_yoke_i  (CONTINUOUS Z yaw)
#     |     |-- caster_wheel_i  (CONTINUOUS Y roll)
#     |-- upper_tray  (FIXED)
#           |-- push_handle  (FIXED)
#           |-- end_guard    (FIXED)

import math

import cadquery as cq

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
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---- key dimensions (meters) ----
TRAY_LEN = 0.92  # along X (same footprint as parent deck)
TRAY_WID = 0.52  # along Y
TRAY_PLATE_THK = 0.004  # sheet-metal pan bottom
TRAY_RIM_HEIGHT = 0.028  # raised lip above the plate
TRAY_RIM_THK = 0.008  # rim wall thickness
TRAY_OUTER_H = TRAY_PLATE_THK + TRAY_RIM_HEIGHT  # total pan depth

# Lower tray sits at the same height as the parent deck underside.
LOWER_TRAY_BOTTOM_Z = 0.165
LOWER_TRAY_CENTER_Z = LOWER_TRAY_BOTTOM_Z + TRAY_OUTER_H / 2.0
LOWER_TRAY_TOP_Z = LOWER_TRAY_BOTTOM_Z + TRAY_OUTER_H

# Upper tray: spaced above the lower tray by corner legs.
TRAY_SPACING = 0.28  # bottom-to-bottom distance between trays
UPPER_TRAY_BOTTOM_Z = LOWER_TRAY_BOTTOM_Z + TRAY_SPACING
UPPER_TRAY_CENTER_Z = UPPER_TRAY_BOTTOM_Z + TRAY_OUTER_H / 2.0
UPPER_TRAY_TOP_Z = UPPER_TRAY_BOTTOM_Z + TRAY_OUTER_H

# Corner legs bridge the gap between the trays.
# Legs sit on the rim walls of the lower tray and support the upper tray bottom plate.
# Slight embed into both pans for a firm structural connection.
LEG_EMBED = 0.005  # how far the leg embeds into each pan surface
CORNER_LEG_BOTTOM_Z = LOWER_TRAY_TOP_Z - LEG_EMBED  # embeds into lower rim top
CORNER_LEG_TOP_Z = UPPER_TRAY_BOTTOM_Z + LEG_EMBED  # embeds into upper pan bottom
CORNER_LEG_LEN = CORNER_LEG_TOP_Z - CORNER_LEG_BOTTOM_Z
CORNER_LEG_CENTER_Z = (CORNER_LEG_BOTTOM_Z + CORNER_LEG_TOP_Z) / 2.0
# Position legs on the rim wall centerline at each corner.
LEG_X = TRAY_LEN / 2.0 - TRAY_RIM_THK / 2.0
LEG_Y = TRAY_WID / 2.0 - TRAY_RIM_THK / 2.0

# Caster geometry (unchanged from parent)
WHEEL_RADIUS = 0.055
WHEEL_WIDTH = 0.034
FORK_OFFSET = 0.034
CASTER_INSET_X = 0.130
CASTER_INSET_Y = 0.090
CASTER_X = TRAY_LEN / 2.0 - CASTER_INSET_X
CASTER_Y = TRAY_WID / 2.0 - CASTER_INSET_Y

# Tubular frame
TUBE_R = 0.011
HANDLE_X = TRAY_LEN / 2.0 - 0.055
HANDLE_BASE_Z = UPPER_TRAY_BOTTOM_Z + TRAY_PLATE_THK  # embed into the pan bottom plate
HANDLE_TOP_Z = HANDLE_BASE_Z + 0.58
HANDLE_HALF_W = TRAY_WID / 2.0 - 0.070
HANDLE_LEAN = 0.085

GUARD_X = -(TRAY_LEN / 2.0 - 0.055)
GUARD_BASE_Z = UPPER_TRAY_BOTTOM_Z + TRAY_PLATE_THK  # embed into the pan bottom plate
GUARD_TOP_Z = GUARD_BASE_Z + 0.28
GUARD_HALF_W = HANDLE_HALF_W


def _tube(points, *, radius=TUBE_R):
    return tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=10,
        radial_segments=14,
        cap_ends=True,
    )


def _u_frame_mesh(*, base_x, top_z, half_w, lean, base_z, name):
    """Inverted-U tubular frame: two uprights joined by a top bend."""
    pts = [
        (base_x, -half_w, base_z),
        (base_x - 0.004, -half_w, base_z + (top_z - base_z) * 0.45),
        (base_x + lean * 0.5, -half_w, top_z - 0.06),
        (base_x + lean, -half_w * 0.96, top_z),
        (base_x + lean, 0.0, top_z + 0.012),
        (base_x + lean, half_w * 0.96, top_z),
        (base_x + lean * 0.5, half_w, top_z - 0.06),
        (base_x - 0.004, half_w, base_z + (top_z - base_z) * 0.45),
        (base_x, half_w, base_z),
    ]
    return mesh_from_geometry(_tube(pts), name)


def _cross_rail_mesh(*, x, z, half_w, name):
    reach = half_w + 0.018
    pts = [(x, -reach, z), (x, 0.0, z), (x, reach, z)]
    return mesh_from_geometry(_tube(pts, radius=TUBE_R * 0.82), name)


def _upright_x_at_frac(base_x, top_z, base_z, lean, frac):
    """Approximate upright centerline x at a height fraction of the U-frame."""
    z = base_z + (top_z - base_z) * frac
    z_mid = base_z + (top_z - base_z) * 0.45
    if z <= z_mid:
        return base_x - 0.004 * (frac / 0.45)
    top_run_frac = (z - z_mid) / max(top_z - 0.06 - z_mid, 1e-6)
    top_run_frac = min(max(top_run_frac, 0.0), 1.0)
    return (base_x - 0.004) + (base_x + lean * 0.5 - (base_x - 0.004)) * top_run_frac


def _tray_pan_mesh(length, width, plate_thk, rim_height, rim_thk, name):
    """Shared geometry helper: shallow rectangular pan with raised rim.

    Builds a CadQuery box and cuts a blind pocket from the top face,
    leaving a thin bottom plate and four rim walls.
    """
    outer_h = plate_thk + rim_height
    pan = cq.Workplane("XY").box(length, width, outer_h)
    inner_l = length - 2.0 * rim_thk
    inner_w = width - 2.0 * rim_thk
    pan = (
        pan.faces(">Z").workplane()
        .rect(inner_l, inner_w)
        .cutBlind(-rim_height)
    )
    return mesh_from_cadquery(pan, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_tier_service_cart")

    tray_gray = model.material("tray_gray", rgba=(0.60, 0.62, 0.64, 1.0))
    tray_blue = model.material("tray_blue", rgba=(0.38, 0.48, 0.58, 1.0))
    frame_gray = model.material("frame_gray", rgba=(0.70, 0.71, 0.73, 1.0))
    leg_silver = model.material("leg_silver", rgba=(0.74, 0.75, 0.77, 1.0))
    rubber = model.material("caster_tread", rgba=(0.16, 0.16, 0.18, 1.0))
    steel = model.material("caster_steel", rgba=(0.55, 0.56, 0.58, 1.0))
    rim_silver = model.material("rim_silver", rgba=(0.80, 0.81, 0.83, 1.0))

    # ---- lower tray (root) ----
    lower_tray = model.part("lower_tray")
    lower_tray.visual(
        _tray_pan_mesh(TRAY_LEN, TRAY_WID, TRAY_PLATE_THK, TRAY_RIM_HEIGHT, TRAY_RIM_THK, "lower_pan"),
        origin=Origin(xyz=(0.0, 0.0, LOWER_TRAY_CENTER_Z)),
        material=tray_gray,
        name="lower_pan",
    )
    lower_tray.inertial = Inertial.from_geometry(
        Box((TRAY_LEN, TRAY_WID, TRAY_OUTER_H)), mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, LOWER_TRAY_CENTER_Z)),
    )

    # ---- upper tray (FIXED to lower tray, carries corner legs) ----
    upper_tray = model.part("upper_tray")
    upper_tray.visual(
        _tray_pan_mesh(TRAY_LEN, TRAY_WID, TRAY_PLATE_THK, TRAY_RIM_HEIGHT, TRAY_RIM_THK, "upper_pan"),
        origin=Origin(xyz=(0.0, 0.0, UPPER_TRAY_CENTER_Z)),
        material=tray_blue,
        name="upper_pan",
    )
    # Four corner tube legs: extend downward from the upper pan to the lower pan.
    for i, (sx, sy) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)]):
        upper_tray.visual(
            Cylinder(radius=TUBE_R, length=CORNER_LEG_LEN),
            origin=Origin(xyz=(sx * LEG_X, sy * LEG_Y, CORNER_LEG_CENTER_Z)),
            material=leg_silver,
            name=f"corner_leg_{i}",
        )
    upper_tray.inertial = Inertial.from_geometry(
        Box((TRAY_LEN, TRAY_WID, TRAY_OUTER_H + CORNER_LEG_LEN)), mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, (LOWER_TRAY_TOP_Z + UPPER_TRAY_TOP_Z) / 2.0)),
    )
    model.articulation(
        "lower_to_upper_tray", ArticulationType.FIXED,
        parent=lower_tray, child=upper_tray,
    )

    # ---- push handle (FIXED to upper tray, +X end) ----
    handle = model.part("push_handle")
    handle.visual(
        _u_frame_mesh(
            base_x=HANDLE_X, top_z=HANDLE_TOP_Z, half_w=HANDLE_HALF_W,
            lean=HANDLE_LEAN, base_z=HANDLE_BASE_Z, name="push_handle_loop",
        ),
        material=frame_gray,
        name="push_handle_loop",
    )
    for i, frac in enumerate((0.40, 0.70)):
        z = HANDLE_BASE_Z + (HANDLE_TOP_Z - HANDLE_BASE_Z) * frac
        x = _upright_x_at_frac(HANDLE_X, HANDLE_TOP_Z, HANDLE_BASE_Z, HANDLE_LEAN, frac)
        handle.visual(
            _cross_rail_mesh(x=x, z=z, half_w=HANDLE_HALF_W, name=f"handle_rail_{i}"),
            material=frame_gray,
            name=f"handle_rail_{i}",
        )
    model.articulation(
        "upper_tray_to_push_handle", ArticulationType.FIXED,
        parent=upper_tray, child=handle,
    )

    # ---- end guard (FIXED to upper tray, -X end) ----
    guard = model.part("end_guard")
    guard.visual(
        _u_frame_mesh(
            base_x=GUARD_X, top_z=GUARD_TOP_Z, half_w=GUARD_HALF_W,
            lean=0.0, base_z=GUARD_BASE_Z, name="end_guard_loop",
        ),
        material=frame_gray,
        name="end_guard_loop",
    )
    guard.visual(
        _cross_rail_mesh(
            x=GUARD_X, z=GUARD_BASE_Z + (GUARD_TOP_Z - GUARD_BASE_Z) * 0.55,
            half_w=GUARD_HALF_W, name="guard_rail",
        ),
        material=frame_gray,
        name="guard_rail",
    )
    model.articulation(
        "upper_tray_to_end_guard", ArticulationType.FIXED,
        parent=upper_tray, child=guard,
    )

    # ---- four swivel casters (for loop) ----
    leg_bottom_z = -(LOWER_TRAY_BOTTOM_Z - WHEEL_RADIUS)
    leg_top_z = leg_bottom_z + WHEEL_RADIUS + 0.012
    leg_half_y = WHEEL_WIDTH / 2.0 + 0.011

    caster_positions = [
        (CASTER_X, CASTER_Y),
        (CASTER_X, -CASTER_Y),
        (-CASTER_X, CASTER_Y),
        (-CASTER_X, -CASTER_Y),
    ]

    for i, (cx, cy) in enumerate(caster_positions):
        yoke = model.part(f"caster_yoke_{i}")
        # swivel mounting plate
        yoke.visual(
            Box((0.060, 0.060, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, -0.005)),
            material=steel,
            name="swivel_plate",
        )
        # kingpin neck
        yoke.visual(
            Cylinder(radius=0.013, length=0.046),
            origin=Origin(xyz=(0.0, 0.0, -0.027)),
            material=steel,
            name="kingpin",
        )
        # offset bracket
        yoke.visual(
            Box((FORK_OFFSET + 0.028, 0.040, 0.014)),
            origin=Origin(xyz=(-FORK_OFFSET / 2.0, 0.0, -0.046)),
            material=steel,
            name="offset_bracket",
        )
        # fork crown
        yoke.visual(
            Box((0.034, leg_half_y * 2.0 + 0.014, 0.014)),
            origin=Origin(xyz=(-FORK_OFFSET, 0.0, leg_top_z)),
            material=steel,
            name="fork_crown",
        )
        # fork legs
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
        # axle bolt
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

        wheel = model.part(f"caster_wheel_{i}")
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
        wheel.visual(
            mesh_from_geometry(rim_geom, f"caster_rim_{i}"),
            material=rim_silver,
            name="rim",
        )
        tire_geom = TireGeometry(
            WHEEL_RADIUS,
            WHEEL_WIDTH,
            inner_radius=WHEEL_RADIUS * 0.64,
            tread=TireTread(style="circumferential", depth=0.002, count=1),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
        )
        tire_geom.rotate_z(math.pi / 2.0)
        wheel.visual(
            mesh_from_geometry(tire_geom, f"caster_tire_{i}"),
            material=rubber,
            name="tire",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH), mass=0.4,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        )

        # swivel: lower_tray -> yoke, vertical kingpin at tray underside
        model.articulation(
            f"tray_to_caster_yoke_{i}",
            ArticulationType.CONTINUOUS,
            parent=lower_tray,
            child=yoke,
            origin=Origin(xyz=(cx, cy, LOWER_TRAY_BOTTOM_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=12.0),
        )
        # roll: yoke -> wheel, horizontal axle
        model.articulation(
            f"caster_spin_{i}",
            ArticulationType.CONTINUOUS,
            parent=yoke,
            child=wheel,
            origin=Origin(xyz=(-FORK_OFFSET, 0.0, -(LOWER_TRAY_BOTTOM_Z - WHEEL_RADIUS))),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=40.0),
        )

    return model


def run_tests():
    from sdk import TestContext

    ctx = TestContext(object_model)
    lower_tray = object_model.get_part("lower_tray")
    upper_tray = object_model.get_part("upper_tray")
    handle = object_model.get_part("push_handle")
    guard = object_model.get_part("end_guard")

    # --- both trays present ---
    ctx.check("lower_tray_present", lower_tray is not None, "lower_tray missing")
    ctx.check("upper_tray_present", upper_tray is not None, "upper_tray missing")

    # --- upper tray pan is above lower tray pan ---
    ctx.expect_gap(
        upper_tray, lower_tray,
        axis="z",
        positive_elem="upper_pan", negative_elem="lower_pan",
        min_gap=0.15, max_gap=0.35,
        name="tray_spacing_meaningful",
    )

    # --- corner legs embed into both trays for structural connection ---
    for i in range(4):
        ctx.allow_overlap(
            upper_tray, lower_tray,
            elem_a=f"corner_leg_{i}", elem_b="lower_pan",
            reason="Corner legs embed into the lower tray rim for structural support.",
        )
        # Prove the leg is near the rim corner and vertically spans the gap.
        ctx.expect_overlap(
            upper_tray, lower_tray,
            axes="z",
            elem_a=f"corner_leg_{i}", elem_b="lower_pan",
            min_overlap=0.001,
            name=f"corner_leg_{i}_contacts_lower_tray",
        )

    # --- axle captured in each wheel hub ---
    for i in range(4):
        ctx.allow_overlap(
            object_model.get_part(f"caster_yoke_{i}"),
            object_model.get_part(f"caster_wheel_{i}"),
            elem_a="axle",
            elem_b="rim",
            reason="The fork axle is captured inside the wheel hub bore.",
        )

    # --- handle and guard tubes embed into the upper pan bottom plate ---
    ctx.allow_overlap(
        handle, upper_tray,
        elem_a="push_handle_loop", elem_b="upper_pan",
        reason="Push handle uprights embed into the upper tray bottom plate for a welded-style mount.",
    )
    ctx.allow_overlap(
        guard, upper_tray,
        elem_a="end_guard_loop", elem_b="upper_pan",
        reason="End guard uprights embed into the upper tray bottom plate for a welded-style mount.",
    )
    # Prove the mounts are seated.
    ctx.expect_contact(
        handle, upper_tray,
        contact_tol=0.015,
        name="handle_contacts_upper_tray",
    )
    ctx.expect_contact(
        guard, upper_tray,
        contact_tol=0.015,
        name="guard_contacts_upper_tray",
    )

    # --- lower tray rides above the floor on casters ---
    la = ctx.part_world_aabb(lower_tray)
    if la is not None:
        ctx.check(
            "lower_tray_above_floor",
            0.12 <= la[0][2] <= 0.22,
            f"lower_tray bottom z={la[0][2]:.3f}",
        )

    # --- four wheels touch the floor ---
    lows = []
    for i in range(4):
        wa = ctx.part_world_aabb(object_model.get_part(f"caster_wheel_{i}"))
        if wa is not None:
            lows.append(wa[0][2])
    ctx.check("four_wheels", len(lows) == 4, f"found {len(lows)} wheels")
    if lows:
        ctx.check(
            "wheels_touch_floor",
            all(abs(z) <= 0.012 for z in lows),
            f"wheel bottoms={['%.3f' % z for z in lows]}",
        )

    # --- handle taller than end guard ---
    ha = ctx.part_world_aabb(handle)
    ga = ctx.part_world_aabb(guard)
    if ha is not None and ga is not None:
        ctx.check(
            "handle_taller_than_guard",
            ha[1][2] > ga[1][2] + 0.15,
            f"handle_top={ha[1][2]:.3f} guard_top={ga[1][2]:.3f}",
        )
        ctx.check(
            "handle_reachable_height",
            0.85 <= ha[1][2] <= 1.25,
            f"handle_top={ha[1][2]:.3f}",
        )
        ctx.check(
            "handle_and_guard_opposite_ends",
            ha[1][0] > 0.2 and ga[0][0] < -0.2,
            f"handle_xmax={ha[1][0]:.3f} guard_xmin={ga[0][0]:.3f}",
        )

    # --- joint inventory: 8 non-fixed caster joints ---
    for i in range(4):
        sw = object_model.get_articulation(f"tray_to_caster_yoke_{i}")
        sp = object_model.get_articulation(f"caster_spin_{i}")
        ctx.check(
            f"swivel_{i}_continuous_z",
            sw.articulation_type == ArticulationType.CONTINUOUS
            and tuple(sw.axis) == (0.0, 0.0, 1.0),
            f"axis={sw.axis} type={sw.articulation_type}",
        )
        ctx.check(
            f"spin_{i}_continuous_y",
            sp.articulation_type == ArticulationType.CONTINUOUS
            and tuple(sp.axis) == (0.0, 1.0, 0.0),
            f"axis={sp.axis} type={sp.articulation_type}",
        )

    # --- decisive pose: wheel spins in place ---
    wheel0 = object_model.get_part("caster_wheel_0")
    spin0 = object_model.get_articulation("caster_spin_0")
    rest = ctx.part_world_position(wheel0)
    with ctx.pose({spin0: 0.6}):
        turned = ctx.part_world_position(wheel0)
    if rest is not None and turned is not None:
        moved = sum((turned[k] - rest[k]) ** 2 for k in range(3)) ** 0.5
        ctx.check(
            "wheel_spins_in_place",
            moved < 1e-4,
            f"center moved {moved:.5f} m under spin",
        )

    # --- decisive pose: swivel yaws without lifting wheel ---
    yoke0 = object_model.get_part("caster_yoke_0")
    swivel0 = object_model.get_articulation("tray_to_caster_yoke_0")
    wheel_low_rest = ctx.part_world_aabb(wheel0)[0][2]
    with ctx.pose({swivel0: math.pi / 2.0}):
        wa = ctx.part_world_aabb(object_model.get_part("caster_wheel_0"))
    if wa is not None:
        ctx.check(
            "swivel_keeps_wheel_on_floor",
            abs(wa[0][2] - wheel_low_rest) < 0.02,
            f"rest_low={wheel_low_rest:.3f} yawed_low={wa[0][2]:.3f}",
        )

    # --- trays share the same XY footprint ---
    ctx.expect_overlap(
        lower_tray, upper_tray,
        axes="xy",
        elem_a="lower_pan", elem_b="upper_pan",
        min_overlap=0.40,
        name="trays_share_footprint_xy",
    )

    return ctx.report()


object_model = build_object_model()
