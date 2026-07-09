from __future__ import annotations

# Two-wheeled tipping barrow.
#
# A tapered open tub on a wheeled steel frame with rear handles, that tips
# forward about the axle line to dump. Two side-by-side roll wheels on a
# single rear axle provide a stable wide-track dump cart.
#
# Coordinate convention (Z-up world):
#   +Z is up; floor is z = 0. Wheel contacts and front legs touch z ~ 0.
#   +X is FORWARD (dump direction); handles extend toward -X.
#   +Y is lateral; centerline is y = 0. Wheels mirror across y = 0.
#
# Root structure: the steel FRAME (handles + front legs + axle tube + pivot
# brackets + cross braces) is the root and carries:
#   - the TUB (sheet-metal bowl) via a REVOLUTE tipping joint
#   - two WHEELS via CONTINUOUS roll joints on the shared rear axle
#
# Articulations:
#   frame_to_tub    : REVOLUTE, tips the tub forward about the axle line
#   frame_to_wheel_0: CONTINUOUS, right wheel roll about Y
#   frame_to_wheel_1: CONTINUOUS, left wheel roll about Y

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
AXLE_X = 0.05            # axle x position (under the tub, slightly rearward)
AXLE_Z = WHEEL_R         # wheel center height (wheels touch z=0)
TRACK = 0.56             # lateral distance between wheel centers

TRAY_LEN = 0.74          # fore-aft length of the tub
TRAY_WID = 0.62          # max width of the tub
TRAY_DEPTH = 0.27        # bowl depth
TRAY_FRONT_X = 0.42      # front lip x of the tray (world)
TRAY_FLOOR_Z = 0.36      # the tray floor height (world)
PIVOT_Z = TRAY_FLOOR_Z   # tub pivot height at the axle line

HANDLE_Y = 0.26          # |y| of each handle tube
GRIP_X = -0.78           # rear x of the handle grips
GRIP_Z = 0.55            # height of the grips above the floor
FRONT_LEG_X = 0.34       # x of the front stand legs


# ---------------------------------------------------------------------------
# Sheet-metal tub (bowl): an open rounded basin built as a loft of superellipse
# cross-sections. Authored in the tub part's local frame (origin at the pivot
# point on the axle line).
# ---------------------------------------------------------------------------
def _rrect_loop(cx, hx, hy, r, z, n_corner=6):
    """Rounded-rectangle loop in the XY plane at height z."""
    r = min(r, hx - 1e-4, hy - 1e-4)
    pts = []
    corners = [
        (hx - r, hy - r),
        (-(hx - r), hy - r),
        (-(hx - r), -(hy - r)),
        (hx - r, -(hy - r)),
    ]
    a0s = [0.0, math.pi / 2.0, math.pi, 1.5 * math.pi]
    for (lx, ly), a0 in zip(corners, a0s):
        for i in range(n_corner + 1):
            a = a0 + (math.pi / 2.0) * (i / n_corner)
            pts.append((cx + lx + r * math.cos(a), ly + r * math.sin(a), z))
    return pts


def _bowl_secs(inset=0.0, top_drop=0.0):
    """Bowl cross-sections in tub local frame (origin at pivot = axle line)."""
    # Tub center x in local frame: TRAY_FRONT_X - TRAY_LEN/2 - AXLE_X = 0.05 - 0.05 = 0
    cx = 0.0
    fl = 0.0  # floor at local z=0 (pivot height)
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
    # raise the floor slightly so the inner shell nests inside the outer wall
    secs = [
        (cx, hx, hy, r, z + (t if i == 0 else 0.0))
        for i, (cx, hx, hy, r, z) in enumerate(secs)
    ]
    return section_loft([_rrect_loop(cx, hx, hy, r, z) for (cx, hx, hy, r, z) in secs])


def _tray_rim_mesh() -> MeshGeometry:
    cx, hx, hy, r, z = _bowl_secs()[-1]
    loop = _rrect_loop(cx, hx, hy, r, z, n_corner=9)
    return tube_from_spline_points(
        loop, radius=0.012, samples_per_segment=2, radial_segments=12,
        closed_spline=True, cap_ends=False,
    )


# ---------------------------------------------------------------------------
# Steel tubular frame (root): two long handle tubes running from the rear grips
# forward past the axle; two front stand legs; a lateral axle tube; two pivot
# brackets; two tray-mount cross tubes; an X-brace; a front cross tube; a rear
# cross tube; a foot rail; and two axle stubs for the wheels.
# All authored in world coordinates.
# ---------------------------------------------------------------------------
def _handle_path(side: int):
    """One handle tube path: rear grip -> steep drop -> flat under tray -> front."""
    return [
        (GRIP_X, side * HANDLE_Y, GRIP_Z),                     # grip end (high, rear)
        (-0.58, side * HANDLE_Y, 0.46),                          # steep drop
        (-0.42, side * HANDLE_Y, TRAY_FLOOR_Z - 0.04),          # below tray rear edge
        (-0.10, side * HANDLE_Y, TRAY_FLOOR_Z - 0.06),          # flat under tray
        (AXLE_X + 0.10, side * HANDLE_Y, TRAY_FLOOR_Z - 0.06),  # under tray, past axle
        (FRONT_LEG_X, side * HANDLE_Y, TRAY_FLOOR_Z - 0.08),    # front end
    ]


def _frame_mesh() -> MeshGeometry:
    geo = MeshGeometry()
    tube_r = 0.016

    # --- two handle tubes ---
    for side in (1, -1):
        h = tube_from_spline_points(
            _handle_path(side), radius=tube_r,
            samples_per_segment=10, radial_segments=12, cap_ends=True,
        )
        geo.merge(h)

    # --- two front stand legs (from handle tubes down to the floor) ---
    for side in (1, -1):
        leg = tube_from_spline_points(
            [
                (FRONT_LEG_X, side * HANDLE_Y, TRAY_FLOOR_Z - 0.08),
                (FRONT_LEG_X + 0.01, side * (HANDLE_Y + 0.02), (TRAY_FLOOR_Z - 0.08) * 0.45),
                (FRONT_LEG_X + 0.02, side * (HANDLE_Y + 0.05), 0.0),
            ],
            radius=0.013, samples_per_segment=8, radial_segments=10, cap_ends=True,
        )
        geo.merge(leg)

    # --- front foot rail (connects the two front legs at the floor) ---
    foot = CylinderGeometry(0.012, 2 * (HANDLE_Y + 0.05), radial_segments=10)
    foot.rotate_x(math.pi / 2.0)
    foot.translate(FRONT_LEG_X + 0.02, 0.0, 0.012)
    geo.merge(foot)

    # --- rear cross tube (ties the two handles behind the tub) ---
    rear_x = -0.58
    rear_z = 0.46  # handle tube height at this x
    rear_cross = CylinderGeometry(0.012, 2 * HANDLE_Y, radial_segments=10)
    rear_cross.rotate_x(math.pi / 2.0)
    rear_cross.translate(rear_x, 0.0, rear_z)
    geo.merge(rear_cross)

    # --- lateral axle tube (spans between the two wheel positions) ---
    axle_len = TRACK + 0.04  # extends slightly past the wheel centers
    axle_tube = CylinderGeometry(0.014, axle_len, radial_segments=12)
    axle_tube.rotate_x(math.pi / 2.0)
    axle_tube.translate(AXLE_X, 0.0, AXLE_Z)
    geo.merge(axle_tube)

    # --- two pivot brackets (vertical tubes from axle up toward tray level) ---
    bracket_h = PIVOT_Z - AXLE_Z - 0.04  # stops just below the tray floor
    for side in (1, -1):
        bracket = CylinderGeometry(0.012, bracket_h, radial_segments=10)
        bracket.translate(
            AXLE_X, side * (HANDLE_Y - 0.02), AXLE_Z + bracket_h / 2.0,
        )
        geo.merge(bracket)

    # --- two tray-mount cross tubes (behind the axle, support the tub at rest) ---
    for tx in (-0.10, -0.02):
        # Handle tube height at these x positions
        h_z = TRAY_FLOOR_Z - 0.06  # handle tubes run at this height under the tray
        cross = tube_from_spline_points(
            [
                (tx, HANDLE_Y, h_z),
                (tx, HANDLE_Y * 0.4, TRAY_FLOOR_Z - 0.025),
                (tx, -HANDLE_Y * 0.4, TRAY_FLOOR_Z - 0.025),
                (tx, -HANDLE_Y, h_z),
            ],
            radius=0.011, samples_per_segment=8, radial_segments=10, cap_ends=True,
        )
        geo.merge(cross)

    # --- X-brace cross stays under the tray ---
    for s in (1, -1):
        stay = tube_from_spline_points(
            [
                (-0.10, s * HANDLE_Y, TRAY_FLOOR_Z - 0.06),
                (0.12, -s * HANDLE_Y, TRAY_FLOOR_Z - 0.06),
            ],
            radius=0.009, samples_per_segment=6, radial_segments=8, cap_ends=True,
        )
        geo.merge(stay)

    # --- front cross tube (ties handle tubes at the front) ---
    front_cross = CylinderGeometry(0.012, 2 * HANDLE_Y, radial_segments=10)
    front_cross.rotate_x(math.pi / 2.0)
    front_cross.translate(FRONT_LEG_X - 0.04, 0.0, TRAY_FLOOR_Z - 0.07)
    geo.merge(front_cross)

    # --- two axle stubs (extend outward from axle tube to wheel mounting) ---
    for side in (1, -1):
        stub = CylinderGeometry(0.012, 0.06, radial_segments=10)
        stub.rotate_x(math.pi / 2.0)
        stub.translate(AXLE_X, side * (TRACK / 2.0 - 0.01), AXLE_Z)
        geo.merge(stub)

    return geo


def _grip_mesh(side: int) -> MeshGeometry:
    """Yellow plastic grip slipped over the rear end of a handle tube."""
    grip = CylinderGeometry(0.020, 0.13, radial_segments=14)
    grip.rotate_y(math.pi / 2.0)  # along X
    grip.translate(GRIP_X + 0.055, side * HANDLE_Y, GRIP_Z + 0.004)
    # rounded end cap
    cap = SphereGeometry(0.020, width_segments=14, height_segments=10)
    cap.translate(GRIP_X - 0.005, side * HANDLE_Y, GRIP_Z + 0.004)
    grip.merge(cap)
    return grip


# ---------------------------------------------------------------------------
# Wheel geometry helpers (shared for both wheels).
# WheelGeometry / TireGeometry spin about local X; rotate so the spin axis
# becomes local Y, matching the joint axis (0,1,0).
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


def _make_wheel_part(model, frame, i: int, side: int, rim_mat, tire_mat):
    """Create one wheel part with disc+tire visuals and a CONTINUOUS roll joint."""
    wheel = model.part(f"wheel_{i}")
    disc = _wheel_disc_mesh()
    tire = _wheel_tire_mesh()
    # Rotate spin axis from local X to local Y
    disc.rotate_z(math.pi / 2.0)
    tire.rotate_z(math.pi / 2.0)
    wheel.visual(
        mesh_from_geometry(disc, f"wheel_disc_{i}"),
        material=rim_mat, name=f"wheel_disc_{i}",
    )
    wheel.visual(
        mesh_from_geometry(tire, f"wheel_tire_{i}"),
        material=tire_mat, name=f"wheel_tire_{i}",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_R, length=WHEEL_W), mass=2.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    y_pos = side * TRACK / 2.0
    model.articulation(
        f"frame_to_wheel_{i}",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(AXLE_X, y_pos, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=30.0),
    )
    return wheel


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tipping_barrow")

    tray_steel = model.material("tray_steel", rgba=(0.46, 0.47, 0.49, 1.0))
    tray_inner = model.material("tray_inner", rgba=(0.40, 0.41, 0.43, 1.0))
    frame_steel = model.material("frame_steel", rgba=(0.62, 0.63, 0.66, 1.0))
    grip_yellow = model.material("grip_yellow", rgba=(0.86, 0.78, 0.20, 1.0))
    rim_silver = model.material("rim_silver", rgba=(0.72, 0.74, 0.77, 1.0))
    rubber = model.material("rubber_dark", rgba=(0.13, 0.13, 0.14, 1.0))

    # ---- steel frame (root) -------------------------------------------------
    frame = model.part("frame")
    frame.visual(
        mesh_from_geometry(_frame_mesh(), "frame_tubes"),
        material=frame_steel, name="frame_tubes",
    )
    frame.visual(
        mesh_from_geometry(_grip_mesh(1), "grip_0"),
        material=grip_yellow, name="grip_0",
    )
    frame.visual(
        mesh_from_geometry(_grip_mesh(-1), "grip_1"),
        material=grip_yellow, name="grip_1",
    )
    frame.inertial = Inertial.from_geometry(
        Box((1.5, 2 * HANDLE_Y + 0.1, GRIP_Z)),
        mass=14.0,
        origin=Origin(xyz=(0.1, 0.0, TRAY_FLOOR_Z)),
    )

    # ---- tub (tips forward about axle line) ---------------------------------
    tub = model.part("tub")
    tub.visual(
        mesh_from_geometry(_tray_shell_mesh(), "tray_outer"),
        material=tray_steel, name="tray_outer",
    )
    tub.visual(
        mesh_from_geometry(_tray_inner_mesh(), "tray_inner"),
        material=tray_inner, name="tray_inner",
    )
    tub.visual(
        mesh_from_geometry(_tray_rim_mesh(), "tray_rim"),
        material=tray_steel, name="tray_rim",
    )
    tub.inertial = Inertial.from_geometry(
        Box((TRAY_LEN, TRAY_WID, TRAY_DEPTH)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, TRAY_DEPTH / 2.0)),
    )

    # Tipping joint: pivot at the axle line. axis=(0,-1,0) so positive q tips
    # the front of the tub (+X) downward for dumping.
    model.articulation(
        "frame_to_tub",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=tub,
        origin=Origin(xyz=(AXLE_X, 0.0, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=1.5, lower=0.0, upper=0.7,
        ),
    )

    # ---- two wheels (for-loop, mirrored, shared geometry helper) -------------
    for i, side in enumerate([1, -1]):
        _make_wheel_part(model, frame, i, side, rim_silver, rubber)

    return model


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    tub = object_model.get_part("tub")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    tip = object_model.get_articulation("frame_to_tub")
    roll_0 = object_model.get_articulation("frame_to_wheel_0")
    roll_1 = object_model.get_articulation("frame_to_wheel_1")

    # --- two wheels exist and are mirrored across the centerline ---
    w0_pos = ctx.part_world_position(wheel_0)
    w1_pos = ctx.part_world_position(wheel_1)
    ctx.check(
        "two wheels mirrored across y=0 centerline",
        w0_pos is not None and w1_pos is not None
        and w0_pos[1] * w1_pos[1] < 0
        and abs(abs(w0_pos[1]) - abs(w1_pos[1])) < 0.02,
        details=f"wheel_0_pos={w0_pos}, wheel_1_pos={w1_pos}",
    )

    # --- both wheels are continuous rollers about Y ---
    for name, roll in [("wheel_0", roll_0), ("wheel_1", roll_1)]:
        ctx.check(
            f"{name} rolls (continuous) about Y",
            roll.joint_type == ArticulationType.CONTINUOUS
            and tuple(roll.axis) == (0.0, 1.0, 0.0),
            details=f"type={roll.joint_type}, axis={roll.axis}",
        )

    # --- both wheels touch the floor ---
    for name, w in [("wheel_0", wheel_0), ("wheel_1", wheel_1)]:
        wa = ctx.part_world_aabb(w)
        ctx.check(
            f"{name} touches the floor (z~0)",
            wa is not None and -0.01 <= wa[0][2] <= 0.02,
            details=f"{name} z_min={wa[0][2] if wa else None}",
        )

    # --- wheels are at the rear (near or behind x=0) ---
    ctx.check(
        "both wheels are at the rear (x <= 0.15)",
        w0_pos is not None and w1_pos is not None
        and w0_pos[0] < 0.15 and w1_pos[0] < 0.15,
        details=f"wheel_0_x={w0_pos[0] if w0_pos else None}, wheel_1_x={w1_pos[0] if w1_pos else None}",
    )

    # --- wheels have correct size (diameter ~ 2R, width along Y) ---
    for name, w in [("wheel_0", wheel_0), ("wheel_1", wheel_1)]:
        wa = ctx.part_world_aabb(w)
        we = _ext(wa)
        ctx.check(
            f"{name} is round (dia ~ 2R, width along Y)",
            abs(we[0] - 2 * WHEEL_R) < 0.05
            and abs(we[2] - 2 * WHEEL_R) < 0.05
            and we[1] < we[0],
            details=f"{name}_ext={we}, 2R={2*WHEEL_R}",
        )

    # --- tub tipping joint is REVOLUTE about the lateral axis ---
    ctx.check(
        "tub tips forward (REVOLUTE about lateral axis)",
        tip.joint_type == ArticulationType.REVOLUTE
        and abs(tip.axis[1]) > 0.9,
        details=f"type={tip.joint_type}, axis={tip.axis}",
    )

    # --- tub is a wide deep bowl ---
    tray_aabb = ctx.part_element_world_aabb(tub, elem="tray_outer")
    te = _ext(tray_aabb)
    ctx.check(
        "tub is a wide deep bowl",
        te[0] > 0.6 and te[1] > 0.5 and te[2] > 0.2,
        details=f"tray_ext={te}",
    )
    ctx.check(
        "tub bowl sits up on the frame (floor above ground)",
        tray_aabb[0][2] > 0.25,
        details=f"tray z_min={tray_aabb[0][2]}",
    )

    # --- tub tipping: positive q drops the front of the tub ---
    tray_rest = ctx.part_element_world_aabb(tub, elem="tray_outer")
    with ctx.pose({tip: 0.5}):
        tray_tipped = ctx.part_element_world_aabb(tub, elem="tray_outer")
    # When tipping forward, the front of the tray (max x) drops and the rear
    # (min x) rises; the overall AABB min-z should drop (front dips below rest)
    ctx.check(
        "tub tips forward: tray front drops at positive q",
        tray_rest is not None and tray_tipped is not None
        and tray_tipped[0][2] < tray_rest[0][2] - 0.01,
        details=f"rest_zmin={tray_rest[0][2] if tray_rest else None}, "
                f"tipped_zmin={tray_tipped[0][2] if tray_tipped else None}",
    )

    # --- wheel spin check: disc extent shifts when spun 30 deg ---
    for i, (w, roll) in enumerate([(wheel_0, roll_0), (wheel_1, roll_1)]):
        ename = f"wheel_disc_{i}"
        e0 = ctx.part_element_world_aabb(w, elem=ename)
        with ctx.pose({roll: math.radians(30.0)}):
            e1 = ctx.part_element_world_aabb(w, elem=ename)
        ctx.check(
            f"wheel_{i} disc spins (spoke extent shifts under rotation)",
            e0 is not None and e1 is not None
            and (abs(e1[0][0] - e0[0][0]) > 0.001
                 or abs(e1[0][2] - e0[0][2]) > 0.001
                 or abs(e1[1][0] - e0[1][0]) > 0.001
                 or abs(e1[1][2] - e0[1][2]) > 0.001),
            details=f"rest={e0}, spun={e1}",
        )

    # --- frame has handles at the rear and legs reaching the floor ---
    fa = ctx.part_world_aabb(frame)
    ctx.check(
        "frame reaches the floor (front stand legs touch z~0)",
        fa is not None and fa[0][2] <= 0.03,
        details=f"frame z_min={fa[0][2] if fa else None}",
    )
    ctx.check(
        "frame spans to the rear grips",
        fa is not None and fa[0][0] < GRIP_X + 0.1,
        details=f"frame x_min={fa[0][0] if fa else None}",
    )

    # --- grips are yellow caps at the rear, on opposite sides ---
    g0 = ctx.part_element_world_aabb(frame, elem="grip_0")
    g1 = ctx.part_element_world_aabb(frame, elem="grip_1")
    ctx.check(
        "two grips at the rear, on opposite sides",
        g0 is not None and g1 is not None
        and g0[0][0] < GRIP_X + 0.15 and g1[0][0] < GRIP_X + 0.15
        and (g0[1][1] > 0) != (g1[1][1] > 0),
        details=f"g0={g0}, g1={g1}",
    )

    # --- wheel-to-frame overlap allowance (axle stubs seat into wheel hubs) ---
    ctx.allow_overlap(
        frame, wheel_0,
        reason="Frame axle stub is seated into the wheel hub bore.",
    )
    ctx.allow_overlap(
        frame, wheel_1,
        reason="Frame axle stub is seated into the wheel hub bore.",
    )
    ctx.expect_contact(frame, wheel_0, name="wheel_0 held in frame axle")
    ctx.expect_contact(frame, wheel_1, name="wheel_1 held in frame axle")

    # --- tub is carried on the frame via the tipping joint; small clearance
    #     gap at rest is realistic (tub rests on cross-tube supports). ---
    ctx.allow_isolated_part(
        tub,
        reason="The tub is carried on the frame via a REVOLUTE tipping joint "
               "and rests on cross-tube supports with a small clearance gap.",
    )
    ctx.expect_contact(
        tub, frame,
        elem_a="tray_outer", elem_b="frame_tubes",
        contact_tol=0.025,
        name="tray rests near frame cross-tube supports at q=0",
    )

    return ctx.report()


object_model = build_object_model()
