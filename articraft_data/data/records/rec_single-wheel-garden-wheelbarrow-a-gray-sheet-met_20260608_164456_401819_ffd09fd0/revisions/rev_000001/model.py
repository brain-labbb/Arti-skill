from __future__ import annotations

# Single-wheel garden wheelbarrow.
#
# Real object: a gray sheet-metal tub (bowl) carried on a tubular steel frame
# with two long rearward handles tipped with yellow plastic grips, two short
# rear stand legs, and a single pneumatic wheel held in a fork at the front. It
# rests on the wheel + the two legs; the wheel is the only moving part.
#
# Coordinate convention (Z-up world):
#   - +Z is up; the floor is z = 0. The wheel contact and both legs touch z ~ 0.
#   - +X is FORWARD (toward the wheel); the handles run back toward -X.
#   - +Y is lateral; centerline is y = 0. Left/right frame members mirror across.
#
# Root structure: the steel FRAME (handles + legs + wheel fork + cross braces)
# is the root and carries the sheet-metal TRAY (fixed) and the WHEEL.
#
# Articulations:
#   - frame_to_wheel : CONTINUOUS front-wheel roll about the lateral (Y) axis.
#
# The two handles, two legs, and two stays are mirror-identical across y = 0.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
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
    section_loft,
    tube_from_spline_points,
)

# ---- key dimensions (meters) -------------------------------------------------
WHEEL_R = 0.18           # pneumatic wheel radius (~0.36 m dia)
WHEEL_W = 0.085          # tire width
WHEEL_X = 0.62           # forward x of the wheel axle
AXLE_Z = WHEEL_R         # wheel center height (wheel touches z=0)

TRAY_LEN = 0.74          # fore-aft length of the tub
TRAY_WID = 0.62          # max width of the tub
TRAY_DEPTH = 0.27        # bowl depth
TRAY_FRONT_X = 0.42      # front lip x of the tray
TRAY_FLOOR_Z = 0.36      # the tray's lowest floor height (it sits up on the frame)

HANDLE_Y = 0.26          # |y| of each handle tube
GRIP_X = -0.78           # rear x of the handle grips
GRIP_Z = 0.55            # height of the grips above the floor
LEG_X = -0.34            # x of the rear legs
LEG_TOP_Z = 0.40         # where the legs meet the handle tubes


# ---------------------------------------------------------------------------
# Sheet-metal tub (bowl): an open rounded basin, deeper at the rear, that sits
# up on the frame. Built as a hollow shell (outer side loft + inset inner loft)
# using superellipse side sections so it reads as a smooth pressed-steel tub.
# Authored in world coordinates.
# ---------------------------------------------------------------------------
def _rrect_loop(cx, hx, hy, r, z, n_corner=6):
    r = min(r, hx - 1e-4, hy - 1e-4)
    pts = []
    corners = [(hx - r, hy - r), (-(hx - r), hy - r), (-(hx - r), -(hy - r)), (hx - r, -(hy - r))]
    a0s = [0.0, math.pi / 2.0, math.pi, 1.5 * math.pi]
    for (lx, ly), a0 in zip(corners, a0s):
        for i in range(n_corner + 1):
            a = a0 + (math.pi / 2.0) * (i / n_corner)
            pts.append((cx + lx + r * math.cos(a), ly + r * math.sin(a), z))
    return pts


# bowl cross-sections from floor (small) up to the wide mouth. (cx, hx, hy, r, z)
def _bowl_secs(inset=0.0, top_drop=0.0):
    cx = (TRAY_FRONT_X - TRAY_LEN / 2.0)  # tray center x
    fl = TRAY_FLOOR_Z
    top = fl + TRAY_DEPTH - top_drop
    return [
        (cx, TRAY_LEN * 0.30 - inset, TRAY_WID * 0.30 - inset, 0.06, fl),
        (cx, TRAY_LEN * 0.40 - inset, TRAY_WID * 0.40 - inset, 0.08, fl + 0.30 * TRAY_DEPTH),
        (cx, TRAY_LEN * 0.47 - inset, TRAY_WID * 0.47 - inset, 0.10, fl + 0.65 * TRAY_DEPTH),
        (cx, TRAY_LEN * 0.50 - inset, TRAY_WID * 0.50 - inset, 0.12, top),
    ]


def _tray_shell_mesh() -> MeshGeometry:
    secs = _bowl_secs()
    return section_loft([_rrect_loop(cx, hx, hy, r, z) for (cx, hx, hy, r, z) in secs])


def _tray_inner_mesh() -> MeshGeometry:
    t = 0.012
    secs = _bowl_secs(inset=t, top_drop=0.0)
    # raise the floor slightly so the inner shell sits inside the outer wall
    secs = [(cx, hx, hy, r, z + (t if i == 0 else 0.0)) for i, (cx, hx, hy, r, z) in enumerate(secs)]
    return section_loft([_rrect_loop(cx, hx, hy, r, z) for (cx, hx, hy, r, z) in secs])


def _tray_rim_mesh() -> MeshGeometry:
    cx, hx, hy, r, z = _bowl_secs()[-1]
    loop = _rrect_loop(cx, hx, hy, r, z, n_corner=9)
    return tube_from_spline_points(
        loop, radius=0.012, samples_per_segment=2, radial_segments=12,
        closed_spline=True, cap_ends=False,
    )


# ---------------------------------------------------------------------------
# Steel tubular frame (root): two long handle tubes that run from the rear grips
# forward under the tray and converge to the wheel fork; two rear legs; two
# cross stays; the wheel fork. Authored in world coordinates as one connected
# tube assembly.
# ---------------------------------------------------------------------------
def _handle_path(side: int):
    # one handle tube: rear grip (high, back) -> sweeps down/forward under the
    # tray -> forward to the wheel fork (low, front, converging toward center).
    return [
        (GRIP_X, side * HANDLE_Y, GRIP_Z),
        (GRIP_X + 0.18, side * HANDLE_Y, GRIP_Z - 0.05),
        (LEG_X, side * HANDLE_Y, LEG_TOP_Z),
        (0.10, side * HANDLE_Y, AXLE_Z + 0.10),
        (WHEEL_X - 0.06, side * (WHEEL_W / 2.0 + 0.03), AXLE_Z + 0.02),
        (WHEEL_X, side * (WHEEL_W / 2.0 + 0.02), AXLE_Z),
    ]


def _frame_mesh() -> MeshGeometry:
    geo = MeshGeometry()
    tube_r = 0.016
    # two handle tubes
    for side in (1, -1):
        h = tube_from_spline_points(
            _handle_path(side), radius=tube_r, samples_per_segment=10,
            radial_segments=12, cap_ends=True,
        )
        geo.merge(h)
    # two rear stand legs: from the handle tube down to the floor, angled out.
    for side in (1, -1):
        leg = tube_from_spline_points(
            [
                (LEG_X, side * HANDLE_Y, LEG_TOP_Z),
                (LEG_X - 0.02, side * (HANDLE_Y + 0.03), LEG_TOP_Z * 0.5),
                (LEG_X - 0.03, side * (HANDLE_Y + 0.06), 0.0),
            ],
            radius=0.013, samples_per_segment=8, radial_segments=10, cap_ends=True,
        )
        geo.merge(leg)
    # rear cross bar tying the two legs (a foot rail)
    foot = CylinderGeometry(0.012, 2 * (HANDLE_Y + 0.06), radial_segments=10)
    foot.rotate_x(math.pi / 2.0)
    foot.translate(LEG_X - 0.03, 0.0, 0.012)
    geo.merge(foot)
    # cross stays under the tray tying the two handle tubes (an X brace)
    for (x0, x1) in ((-0.05, 0.18),):
        for s in (1, -1):
            stay = tube_from_spline_points(
                [(x0, s * HANDLE_Y, AXLE_Z + 0.12), (x1, -s * HANDLE_Y, AXLE_Z + 0.14)],
                radius=0.009, samples_per_segment=6, radial_segments=8, cap_ends=True,
            )
            geo.merge(stay)
    # two transverse tray-mount cross tubes: they span between the handle rails
    # and rise to just under the tray floor so the sheet-metal tub is physically
    # bolted onto the frame (no floating tub). Their tops touch the tray bottom.
    for tx in (-0.12, 0.20):
        cross = tube_from_spline_points(
            [
                (tx, HANDLE_Y, AXLE_Z + 0.14),
                (tx, HANDLE_Y * 0.5, TRAY_FLOOR_Z - 0.005),
                (tx, -HANDLE_Y * 0.5, TRAY_FLOOR_Z - 0.005),
                (tx, -HANDLE_Y, AXLE_Z + 0.14),
            ],
            radius=0.011, samples_per_segment=8, radial_segments=10, cap_ends=True,
        )
        geo.merge(cross)
    # a front cross tube just behind the wheel tying the converging handle tubes
    # (sits at the handle height where it crosses so it actually touches them).
    front_cross = CylinderGeometry(0.012, 2 * (WHEEL_W / 2.0 + 0.05), radial_segments=10)
    front_cross.rotate_x(math.pi / 2.0)
    front_cross.translate(WHEEL_X - 0.07, 0.0, AXLE_Z + 0.02)
    geo.merge(front_cross)
    # two short fork stubs that carry the wheel axle (just inboard of the tire)
    for side in (1, -1):
        stub = CylinderGeometry(0.012, 0.05, radial_segments=10)
        stub.rotate_x(math.pi / 2.0)
        stub.translate(WHEEL_X, side * (WHEEL_W / 2.0 + 0.018), AXLE_Z)
        geo.merge(stub)
    return geo


def _grip_mesh(side: int) -> MeshGeometry:
    # yellow plastic grip slipped over the rear end of a handle tube.
    grip = CylinderGeometry(0.020, 0.13, radial_segments=14)
    grip.rotate_y(math.pi / 2.0)  # along X
    grip.translate(GRIP_X + 0.055, side * HANDLE_Y, GRIP_Z + 0.004)
    # rounded end cap
    cap = SphereGeometry(0.020, width_segments=14, height_segments=10)
    cap.translate(GRIP_X - 0.005, side * HANDLE_Y, GRIP_Z + 0.004)
    grip.merge(cap)
    return grip


# ---------------------------------------------------------------------------
# Wheel (WheelGeometry/TireGeometry spin about local X; rotate so the spin axis
# becomes local Y, matching the joint axis (0,1,0)).
# ---------------------------------------------------------------------------
def _wheel_disc_mesh() -> MeshGeometry:
    return WheelGeometry(
        WHEEL_R - 0.055,
        WHEEL_W - 0.02,
        rim=WheelRim(inner_radius=0.075, flange_height=0.010, flange_thickness=0.004),
        hub=WheelHub(
            radius=0.030,
            width=WHEEL_W - 0.01,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=5, circle_diameter=0.045, hole_diameter=0.005),
        ),
        face=WheelFace(dish_depth=0.012, front_inset=0.004),
        spokes=WheelSpokes(style="straight", count=6, thickness=0.005, window_radius=0.016),
        bore=WheelBore(style="round", diameter=0.016),
    )


def _wheel_tire_mesh() -> MeshGeometry:
    return TireGeometry(
        WHEEL_R,
        WHEEL_W,
        inner_radius=WHEEL_R - 0.06,
        tread=TireTread(style="block", depth=0.010, count=26, land_ratio=0.55),
        sidewall=TireSidewall(style="rounded", bulge=0.06),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelbarrow")

    tray_steel = model.material("tray_steel", rgba=(0.46, 0.47, 0.49, 1.0))
    tray_inner = model.material("tray_inner", rgba=(0.40, 0.41, 0.43, 1.0))
    frame_steel = model.material("frame_steel", rgba=(0.62, 0.63, 0.66, 1.0))
    grip_yellow = model.material("grip_yellow", rgba=(0.86, 0.78, 0.20, 1.0))
    rim_silver = model.material("rim_silver", rgba=(0.72, 0.74, 0.77, 1.0))
    rubber = model.material("rubber_dark", rgba=(0.13, 0.13, 0.14, 1.0))

    # ---- steel frame (root) -------------------------------------------------
    frame = model.part("frame")
    frame.visual(mesh_from_geometry(_frame_mesh(), "frame_tubes"),
                 material=frame_steel, name="frame_tubes")
    frame.visual(mesh_from_geometry(_grip_mesh(1), "grip_0"),
                 material=grip_yellow, name="grip_0")
    frame.visual(mesh_from_geometry(_grip_mesh(-1), "grip_1"),
                 material=grip_yellow, name="grip_1")
    # the sheet-metal tray is fixed to the frame, so it lives on the frame part.
    frame.visual(mesh_from_geometry(_tray_shell_mesh(), "tray_outer"),
                 material=tray_steel, name="tray_outer")
    frame.visual(mesh_from_geometry(_tray_inner_mesh(), "tray_inner"),
                 material=tray_inner, name="tray_inner")
    frame.visual(mesh_from_geometry(_tray_rim_mesh(), "tray_rim"),
                 material=tray_steel, name="tray_rim")
    frame.inertial = Inertial.from_geometry(
        Box((1.5, 2 * HANDLE_Y + 0.1, GRIP_Z)),
        mass=14.0,
        origin=Origin(xyz=(0.1, 0.0, TRAY_FLOOR_Z)),
    )

    # ---- single front wheel (continuous roll about Y) -----------------------
    wheel = model.part("wheel")
    disc = _wheel_disc_mesh()
    tire = _wheel_tire_mesh()
    disc.rotate_z(math.pi / 2.0)  # local X spin -> local Y
    tire.rotate_z(math.pi / 2.0)
    wheel.visual(mesh_from_geometry(disc, "wheel_disc"), material=rim_silver, name="wheel_disc")
    wheel.visual(mesh_from_geometry(tire, "wheel_tire"), material=rubber, name="wheel_tire")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_R, length=WHEEL_W), mass=2.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    model.articulation(
        "frame_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(WHEEL_X, 0.0, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=30.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    roll = object_model.get_articulation("frame_to_wheel")

    # --- the single wheel is a continuous roller about the lateral Y axis ----
    ctx.check(
        "front wheel rolls (continuous) about Y",
        roll.joint_type == ArticulationType.CONTINUOUS
        and tuple(roll.axis) == (0.0, 1.0, 0.0),
        details=f"type={roll.joint_type}, axis={roll.axis}",
    )

    # --- the wheel touches the floor (wheel contact at z~0) ------------------
    wa = ctx.part_world_aabb(wheel)
    ctx.check(
        "wheel touches the floor (z~0)",
        wa is not None and -0.01 <= wa[0][2] <= 0.02,
        details=f"wheel z_min={wa[0][2] if wa else None}",
    )
    we = _ext(wa)
    ctx.check(
        "wheel is a single round wheel (dia ~ 2R, width along Y)",
        abs(we[0] - 2 * WHEEL_R) < 0.05 and abs(we[2] - 2 * WHEEL_R) < 0.05
        and we[1] < we[0],
        details=f"wheel_ext={we}, 2R={2*WHEEL_R}",
    )
    # the wheel is centered on the centerline (single wheel, not a pair)
    wp = ctx.part_world_position(wheel)
    ctx.check(
        "wheel is centered on the y=0 centerline (single wheel)",
        wp is not None and abs(wp[1]) < 0.006,
        details=f"wheel_pos={wp}",
    )
    # the wheel is at the FRONT (+X) of the barrow
    ctx.check(
        "wheel is at the front (+X)",
        wp is not None and wp[0] > 0.4,
        details=f"wheel_x={wp[0] if wp else None}",
    )

    # --- the frame + legs rest on the floor (parked: legs touch z~0) ---------
    fa = ctx.part_world_aabb(frame)
    ctx.check(
        "frame legs reach the floor (rear stand legs touch z~0)",
        fa is not None and fa[0][2] <= 0.03,
        details=f"frame z_min={fa[0][2] if fa else None}",
    )
    # the grips are at the rear-high end
    ctx.check(
        "frame spans from the rear grips back behind the tray",
        fa is not None and fa[0][0] < GRIP_X + 0.1,
        details=f"frame x_min={fa[0][0] if fa else None}",
    )

    # --- the tray (tub) is the big bowl, up on the frame, above the floor ----
    tray_aabb = ctx.part_element_world_aabb(frame, elem="tray_outer")
    te = _ext(tray_aabb)
    ctx.check(
        "tray is a wide deep bowl (long & wide & deep)",
        te[0] > 0.6 and te[1] > 0.5 and te[2] > 0.2,
        details=f"tray_ext={te}",
    )
    ctx.check(
        "tray bowl sits up on the frame (floor of bowl above ground)",
        tray_aabb[0][2] > 0.25,
        details=f"tray z_min={tray_aabb[0][2]}",
    )

    # --- grips are yellow caps at the rear, on the handle tubes --------------
    g0 = ctx.part_element_world_aabb(frame, elem="grip_0")
    g1 = ctx.part_element_world_aabb(frame, elem="grip_1")
    ctx.check(
        "two grips at the rear, on opposite sides, mirror across centerline",
        g0 is not None and g1 is not None
        and (g0[0][0] < GRIP_X + 0.15) and (g1[0][0] < GRIP_X + 0.15)
        and (g0[1][1] > 0) != (g1[1][1] > 0),
        details=f"g0={g0}, g1={g1}",
    )

    # --- the wheel actually rolls: the (6-fold) spoke disc shifts when spun
    #     30 deg (half the 60-deg spoke pitch). ------------------------------
    e0 = ctx.part_element_world_aabb(wheel, elem="wheel_disc")
    with ctx.pose({roll: math.radians(30.0)}):
        e1 = ctx.part_element_world_aabb(wheel, elem="wheel_disc")
    ctx.check(
        "wheel disc spins (spoke extent shifts under rotation about Y)",
        e0 is not None and e1 is not None
        and (abs(e1[0][0] - e0[0][0]) > 0.001 or abs(e1[0][2] - e0[0][2]) > 0.001
             or abs(e1[1][0] - e0[1][0]) > 0.001 or abs(e1[1][2] - e0[1][2]) > 0.001),
        details=f"rest_disc_aabb={e0}, spun_disc_aabb={e1}",
    )

    # --- the wheel is held in the frame fork (axle stubs touch the wheel) ----
    ctx.allow_overlap(
        frame, wheel,
        reason="The frame fork axle stubs are seated into the wheel hub/axle.",
    )
    ctx.expect_contact(frame, wheel, name="wheel held in the frame fork")

    return ctx.report()


object_model = build_object_model()
