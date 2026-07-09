from __future__ import annotations

# Vintage GREEN kick scooter – THREE-WHEEL kids variant.
# Frame: world Z up, rolling direction along +Y (front wheel at +Y, rear pair at -Y),
# wheel axles along X (left/right). Ground at z=0; deck top is low (~z=0.10).
#
# Layout (side view, +Y to the right):
#   - low flat foot deck near the rear, with small branding on top.
#   - a tall, gracefully curved handlebar stem sweeping up from the FRONT of the
#     deck, ending in a T handlebar with two grips.
#   - front wheel under a big curved fender (turns with the steering).
#   - TWO rear wheels under matching curved fenders, one on each side of the
#     deck tail, spaced symmetrically on a shared rear axle (fixed to body).
#
# Articulations:
#   - steering: curved stem + T-bar + front fender + front wheel pivot together
#       as one column, REVOLUTE about the tilted steering (head) axis (~+/-40 deg).
#   - front wheel: CONTINUOUS roll about its axle (child of the steering column).
#   - rear_wheel_0, rear_wheel_1: each CONTINUOUS roll about its axle (child of body).

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
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
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---- key geometry constants (meters) ----
WHEEL_R = 0.090  # rolling radius of the small wheels
WHEEL_W = 0.040  # wheel/tire width along X
DECK_TOP_Z = 0.100  # top surface of the foot deck
REAR_WHEEL_Y = -0.235  # rear axle Y (both rear wheels share this Y)
FRONT_WHEEL_Y = 0.320  # front axle Y (at rest)
AXLE_Z = WHEEL_R  # axle height so wheels rest on the ground

# Rear track: lateral distance between the two rear wheel centers.
# Wheels sit just outside the deck edges (deck is 0.130 wide).
N_REAR_WHEELS = 2
REAR_TRACK = 0.180  # center-to-center distance between rear wheels along X

# Steering head: located just behind/above the front wheel where the curved
# stem meets the deck neck. The head axis is tilted back from vertical.
HEAD_BASE = (0.0, 0.180, 0.150)  # bottom of the head tube
HEAD_TILT = math.radians(18.0)  # lean back of the steering axis (about +X)
# Steering axis: tilted in the YZ plane (mostly +Z, leaning toward -Y at top).
STEER_AXIS = (0.0, math.sin(HEAD_TILT), math.cos(HEAD_TILT))


def _fender(radius: float, width: float, thickness: float, span_deg: float,
            start_deg: float, name: str):
    # A curved fender: a thin annular arc band lying in the YZ plane (the wheel
    # rolling plane), extruded along X. The arc is centered on the local origin
    # (the wheel axle). Angle 0 deg = +Y, 90 deg = +Z (up).
    r_out = radius + thickness
    r_in = radius
    a0 = math.radians(start_deg)
    a1 = math.radians(start_deg + span_deg)
    n = 40
    outer = []
    inner = []
    for i in range(n + 1):
        a = a0 + (a1 - a0) * (i / n)
        # profile in local (Y, Z) -> extrude helper uses (x, y) profile plane.
        outer.append((r_out * math.cos(a), r_out * math.sin(a)))
        inner.append((r_in * math.cos(a), r_in * math.sin(a)))
    profile = outer + list(reversed(inner))
    geom = ExtrudeGeometry.centered(profile, width, cap=True)
    # ExtrudeGeometry extrudes the XY profile along Z. We want the profile in the
    # YZ plane (so cos->Y, sin->Z) and the band thickness along X.
    # Built profile X=cos(->world Y), Y=sin(->world Z), extrude Z(->world X).
    # Rotate so profile-X -> world Y, profile-Y -> world Z, profile-Z -> world X.
    geom.rotate_y(math.pi / 2.0)  # profile Z(extrude) -> world X
    geom.rotate_x(math.pi / 2.0)  # profile Y -> world Z, profile X -> world Y
    return mesh_from_geometry(geom, name)


def _wheel_meshes(prefix: str):
    # Returns (wheel_mesh, tire_mesh, whitewall_mesh) all oriented so the wheel
    # spins about local X (axle along X). The white-wall is a thin disc ring.
    wheel = WheelGeometry(
        WHEEL_R * 0.62,
        WHEEL_W * 0.70,
        rim=WheelRim(inner_radius=WHEEL_R * 0.40, flange_height=0.006, flange_thickness=0.003),
        hub=WheelHub(radius=WHEEL_R * 0.20, width=WHEEL_W * 0.85, cap_style="domed"),
        face=WheelFace(dish_depth=0.004, front_inset=0.002),
        bore=WheelBore(style="round", diameter=0.010),
    )
    wheel_mesh = mesh_from_geometry(wheel, f"{prefix}_rim")

    tire = TireGeometry(
        WHEEL_R,
        WHEEL_W,
        inner_radius=WHEEL_R * 0.62,
        tread=TireTread(style="circumferential", depth=0.0025, count=2),
        sidewall=TireSidewall(style="rounded", bulge=0.04),
    )
    tire_mesh = mesh_from_geometry(tire, f"{prefix}_tire")

    # White-wall ring: a thin flat disc on the outer face of the tire sidewall.
    ww = CylinderGeometry(WHEEL_R * 0.80, 0.004, radial_segments=48).rotate_y(math.pi / 2.0)
    ww.translate(WHEEL_W * 0.52, 0.0, 0.0)
    whitewall_mesh = mesh_from_geometry(ww, f"{prefix}_whitewall")
    return wheel_mesh, tire_mesh, whitewall_mesh


def _rear_wheel_x(index: int) -> float:
    """Return the X position of rear wheel i, symmetrically about center."""
    return -REAR_TRACK / 2.0 + index * (REAR_TRACK / (N_REAR_WHEELS - 1))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kick_scooter")

    green = model.material("scooter_green", rgba=(0.36, 0.55, 0.40, 1.0))
    green_dark = model.material("scooter_green_dark", rgba=(0.28, 0.44, 0.32, 1.0))
    black_grip = model.material("grip_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber = model.material("tire_black", rgba=(0.08, 0.08, 0.09, 1.0))
    whitewall = model.material("whitewall", rgba=(0.90, 0.90, 0.88, 1.0))
    metal = model.material("chrome", rgba=(0.74, 0.76, 0.79, 1.0))
    silver = model.material("brand_silver", rgba=(0.82, 0.83, 0.85, 1.0))

    # =====================================================================
    # ROOT BODY: foot deck + rear neck/head tube + rear fenders + rear axle
    # =====================================================================
    body = model.part("body")

    # Flat foot deck (low, horizontal), running between the wheels.
    deck = BoxGeometry((0.130, 0.330, 0.020))
    deck.translate(0.0, 0.020, DECK_TOP_Z - 0.010)
    body.visual(mesh_from_geometry(deck, "foot_deck"), material=green, name="foot_deck")

    # Small raised branding plate on top of the deck.
    body.visual(
        Box((0.060, 0.150, 0.003)),
        origin=Origin(xyz=(0.0, 0.010, DECK_TOP_Z + 0.0015)),
        material=silver,
        name="branding",
    )

    # Front riser/neck: green body sweeping up from the front of the deck to the
    # steering head tube. Built as a bent tube so it reads as a cast neck.
    neck = tube_from_spline_points(
        [
            (0.0, 0.180, DECK_TOP_Z - 0.005),
            (0.0, 0.182, 0.118),
            (0.0, 0.183, 0.140),
            (0.0, 0.184, 0.158),
        ],
        radius=0.026,
        samples_per_segment=10,
        radial_segments=18,
    )
    body.visual(mesh_from_geometry(neck, "front_neck"), material=green, name="front_neck")

    # Steering head tube (the barrel the stem pivots in), aligned to the head axis.
    head_tube = CylinderGeometry(0.022, 0.090, radial_segments=24)
    # orient cylinder (local Z) along the steering axis by tilting about X.
    head_tube.rotate_x(HEAD_TILT)
    head_tube.translate(HEAD_BASE[0], HEAD_BASE[1] - 0.006, HEAD_BASE[2] + 0.012)
    body.visual(mesh_from_geometry(head_tube, "head_tube"), material=green_dark, name="head_tube")

    # Rear axle assembly: a visible metal bar spanning between the two rear wheels,
    # with two vertical mount plates connecting the axle to the deck underside.
    rear_axle = CylinderGeometry(0.010, REAR_TRACK + WHEEL_W, radial_segments=16)
    rear_axle.rotate_y(math.pi / 2.0)  # align along X
    rear_axle.translate(0.0, REAR_WHEEL_Y, AXLE_Z)
    body.visual(mesh_from_geometry(rear_axle, "rear_axle"), material=metal, name="rear_axle")

    # Central rear brace: a tube connecting the deck rear edge to the axle beam center.
    # This runs along Y at x=0 (between the two rear wheels, clear of both).
    deck_rear_y = 0.020 - 0.330 / 2.0  # rear edge of the deck box
    brace = tube_from_spline_points(
        [
            (0.0, deck_rear_y, DECK_TOP_Z - 0.010),
            (0.0, (deck_rear_y + REAR_WHEEL_Y) / 2.0, AXLE_Z + 0.005),
            (0.0, REAR_WHEEL_Y, AXLE_Z),
        ],
        radius=0.010,
        samples_per_segment=6,
        radial_segments=12,
    )
    body.visual(mesh_from_geometry(brace, "rear_brace"), material=green_dark, name="rear_brace")

    # Rear fenders and struts: one per rear wheel, via loop.
    for i in range(N_REAR_WHEELS):
        wx = _rear_wheel_x(i)
        # Fender arcing over this rear wheel, fixed to body.
        body.visual(
            _fender(WHEEL_R + 0.010, WHEEL_W + 0.022, 0.012, span_deg=150.0, start_deg=35.0,
                    name=f"rear_fender_{i}"),
            origin=Origin(xyz=(wx, REAR_WHEEL_Y, AXLE_Z)),
            material=green,
            name=f"rear_fender_{i}",
        )
        # Strut bridging the rear of the deck to the front-top edge of this fender.
        fender_front_top = (
            wx,
            REAR_WHEEL_Y + (WHEEL_R + 0.010) * math.cos(math.radians(48.0)),
            AXLE_Z + (WHEEL_R + 0.010) * math.sin(math.radians(48.0)),
        )
        strut = tube_from_spline_points(
            [
                (wx * 0.5, -0.118, DECK_TOP_Z - 0.002),
                (wx * 0.75, -0.125, 0.160),
                fender_front_top,
            ],
            radius=0.011,
            samples_per_segment=8,
            radial_segments=14,
        )
        body.visual(mesh_from_geometry(strut, f"rear_strut_{i}"), material=green_dark, name=f"rear_strut_{i}")

    body.inertial = Inertial.from_geometry(
        Box((0.20, 0.55, 0.22)), mass=6.5, origin=Origin(xyz=(0.0, -0.02, 0.10))
    )

    # =====================================================================
    # REAR WHEELS: continuous roll about each axle (children of body)
    #   Emitted via a for-i-in-range loop over the shared wheel geometry helper.
    # =====================================================================
    for i in range(N_REAR_WHEELS):
        wx = _rear_wheel_x(i)
        rw_part = model.part(f"rear_wheel_{i}")
        rw_rim, rw_tire, rw_ww = _wheel_meshes(f"rear_{i}")
        rw_part.visual(rw_tire, material=rubber, name=f"rear_tire_{i}")
        rw_part.visual(rw_ww, material=whitewall, name=f"rear_whitewall_{i}")
        rw_part.visual(rw_rim, material=metal, name=f"rear_rim_{i}")
        # Off-axis spin marker (valve stem) so the roll is detectable.
        rw_part.visual(
            Cylinder(radius=0.005, length=WHEEL_W + 0.020),
            origin=Origin(xyz=(0.0, WHEEL_R * 0.55, WHEEL_R * 0.55), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name=f"rear_valve_{i}",
        )
        rw_part.inertial = Inertial.from_geometry(
            Cylinder(radius=WHEEL_R, length=WHEEL_W),
            mass=0.6,
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        )
        model.articulation(
            f"rear_wheel_roll_{i}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=rw_part,
            origin=Origin(xyz=(wx, REAR_WHEEL_Y, AXLE_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=30.0),
        )

    # =====================================================================
    # STEERING COLUMN: curved stem + T-bar + grips + front fender
    #   pivots about the head axis (revolute).  The column's LOCAL frame origin
    #   is at HEAD_BASE (the steering joint origin), so all column geometry and
    #   the front-wheel axle are authored relative to HEAD_BASE.
    # =====================================================================
    column = model.part("steering_column")

    def _rel(pts):
        return [(x - HEAD_BASE[0], y - HEAD_BASE[1], z - HEAD_BASE[2]) for (x, y, z) in pts]

    # Front-wheel axle position expressed in the column-local frame.
    fw_axle_local = (
        0.0 - HEAD_BASE[0],
        FRONT_WHEEL_Y - HEAD_BASE[1],
        AXLE_Z - HEAD_BASE[2],
    )

    # The gracefully curved stem: sweeps up from the head, leaning back, then
    # forward to the T handlebar. Authored in body coords, then made local.
    stem = tube_from_spline_points(
        _rel(
            [
                (0.0, 0.182, 0.150),
                (0.0, 0.150, 0.320),
                (0.0, 0.120, 0.500),
                (0.0, 0.120, 0.660),
                (0.0, 0.150, 0.770),
                (0.0, 0.190, 0.815),
            ]
        ),
        radius=0.018,
        samples_per_segment=14,
        radial_segments=18,
    )
    column.visual(mesh_from_geometry(stem, "curved_stem"), material=green, name="curved_stem")

    # Lower fork legs from the head down to the front axle (carry the front wheel).
    for sx in (-1.0, 1.0):
        leg = tube_from_spline_points(
            _rel(
                [
                    (sx * 0.006, 0.184, 0.150),
                    (sx * 0.028, 0.250, 0.100),
                    (sx * 0.032, FRONT_WHEEL_Y, AXLE_Z + 0.004),
                ]
            ),
            radius=0.010,
            samples_per_segment=10,
            radial_segments=14,
        )
        column.visual(
            mesh_from_geometry(leg, f"fork_leg_{0 if sx < 0 else 1}"),
            material=green_dark,
            name=f"fork_leg_{0 if sx < 0 else 1}",
        )

    bar_z = 0.823 - HEAD_BASE[2]
    bar_y = 0.195 - HEAD_BASE[1]
    # T handlebar crossbar (along X) at the top of the stem.
    bar = Cylinder(radius=0.014, length=0.300)
    column.visual(
        bar,
        origin=Origin(xyz=(0.0, bar_y, bar_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="handlebar_crossbar",
    )
    # Stem-to-bar gooseneck cap.
    column.visual(
        Box((0.030, 0.040, 0.030)),
        origin=Origin(xyz=(0.0, 0.193 - HEAD_BASE[1], 0.818 - HEAD_BASE[2])),
        material=green_dark,
        name="gooseneck",
    )
    # Two black rubber grips at the ends of the crossbar.
    for sx in (-1.0, 1.0):
        column.visual(
            Cylinder(radius=0.018, length=0.090),
            origin=Origin(xyz=(sx * 0.110, bar_y, bar_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black_grip,
            name=f"grip_{0 if sx < 0 else 1}",
        )

    # Big curved front fender arcing over the front wheel (turns with steering).
    column.visual(
        _fender(WHEEL_R + 0.012, WHEEL_W + 0.024, 0.013, span_deg=170.0, start_deg=10.0,
                name="front_fender"),
        origin=Origin(xyz=fw_axle_local),
        material=green,
        name="front_fender",
    )

    column.inertial = Inertial.from_geometry(
        Box((0.30, 0.20, 0.70)),
        mass=2.5,
        origin=Origin(xyz=(0.0, 0.16 - HEAD_BASE[1], 0.45 - HEAD_BASE[2])),
    )
    model.articulation(
        "steering",
        ArticulationType.REVOLUTE,
        parent=body,
        child=column,
        origin=Origin(xyz=HEAD_BASE),
        axis=STEER_AXIS,
        motion_limits=MotionLimits(
            effort=8.0, velocity=3.0,
            lower=-math.radians(40.0), upper=math.radians(40.0),
        ),
    )

    # =====================================================================
    # FRONT WHEEL: continuous roll, child of the steering column
    # =====================================================================
    front_wheel = model.part("front_wheel")
    fw_rim, fw_tire, fw_ww = _wheel_meshes("front")
    front_wheel.visual(fw_tire, material=rubber, name="front_tire")
    front_wheel.visual(fw_ww, material=whitewall, name="front_whitewall")
    front_wheel.visual(fw_rim, material=metal, name="front_rim")
    front_wheel.visual(
        Cylinder(radius=0.005, length=WHEEL_W + 0.020),
        origin=Origin(xyz=(0.0, WHEEL_R * 0.55, WHEEL_R * 0.55), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="front_valve",
    )
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_R, length=WHEEL_W),
        mass=0.6,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    model.articulation(
        "front_wheel_roll",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=front_wheel,
        origin=Origin(xyz=fw_axle_local),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=30.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    column = object_model.get_part("steering_column")
    front_wheel = object_model.get_part("front_wheel")
    steering = object_model.get_articulation("steering")
    front_roll = object_model.get_articulation("front_wheel_roll")

    # Collect rear wheel parts and joints from the loop.
    rear_wheels = [object_model.get_part(f"rear_wheel_{i}") for i in range(N_REAR_WHEELS)]
    rear_rolls = [object_model.get_articulation(f"rear_wheel_roll_{i}") for i in range(N_REAR_WHEELS)]

    # ---- deck is low and horizontal ----
    deck_aabb = ctx.part_element_world_aabb(body, elem="foot_deck")
    dz = _ext(deck_aabb)
    ctx.check(
        "deck is flat/horizontal (thin in Z, long in Y)",
        dz[2] < 0.05 and dz[1] > 0.30,
        details=f"deck extents={dz}",
    )
    ctx.check(
        "deck sits low (top below 0.15 m)",
        deck_aabb[1][2] < 0.15,
        details=f"deck max z={deck_aabb[1][2]}",
    )

    # ---- scooter rests on ALL THREE wheels at the ground plane (z ~ 0) ----
    front_aabb = ctx.part_world_aabb(front_wheel)
    ctx.check(
        "front wheel reaches the ground",
        front_aabb[0][2] < 0.02,
        details=f"front min z={front_aabb[0][2]}",
    )
    for i, rw in enumerate(rear_wheels):
        rw_aabb = ctx.part_world_aabb(rw)
        ctx.check(
            f"rear_wheel_{i} reaches the ground",
            rw_aabb[0][2] < 0.02,
            details=f"rear_wheel_{i} min z={rw_aabb[0][2]}",
        )

    # ---- front wheel ahead of rear wheels ----
    front_y = ctx.part_world_position(front_wheel)[1]
    for i, rw in enumerate(rear_wheels):
        rw_y = ctx.part_world_position(rw)[1]
        ctx.check(
            f"front wheel ahead of rear_wheel_{i}",
            front_y > rw_y + 0.30,
            details=f"front_y={front_y}, rear_wheel_{i}_y={rw_y}",
        )

    # ---- two rear wheels are symmetrically spaced left/right ----
    rw0_x = ctx.part_world_position(rear_wheels[0])[0]
    rw1_x = ctx.part_world_position(rear_wheels[1])[0]
    ctx.check(
        "rear wheels are on opposite sides of center",
        rw0_x < -0.03 and rw1_x > 0.03,
        details=f"rw0_x={rw0_x}, rw1_x={rw1_x}",
    )
    ctx.check(
        "rear wheels are symmetrically spaced",
        abs(rw0_x + rw1_x) < 0.01,
        details=f"sum_x={rw0_x + rw1_x}",
    )
    ctx.check(
        "rear wheels share the same Y (rear axle line)",
        abs(ctx.part_world_position(rear_wheels[0])[1] - ctx.part_world_position(rear_wheels[1])[1]) < 0.005,
        details=f"rw0_y={ctx.part_world_position(rear_wheels[0])[1]}, rw1_y={ctx.part_world_position(rear_wheels[1])[1]}",
    )

    # ---- tall curved stem reaches handlebar height (~0.8 m) ----
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "handlebar/stem is tall (reaches ~0.8 m)",
        col_aabb[1][2] > 0.78,
        details=f"column max z={col_aabb[1][2]}",
    )

    # ---- front fender arcs OVER the front wheel ----
    fender_aabb = ctx.part_element_world_aabb(column, elem="front_fender")
    ctx.check(
        "front fender arcs above the front wheel",
        fender_aabb[1][2] > AXLE_Z + WHEEL_R * 0.5,
        details=f"front fender max z={fender_aabb[1][2]}",
    )
    # Fender is intentionally wider/over the wheel; allow the nested overlap.
    ctx.allow_overlap(
        column, front_wheel,
        elem_a="front_fender", elem_b="front_tire",
        reason="The curved fender intentionally arcs closely over the front wheel.",
    )
    # The fork legs reach the axle and intentionally meet the wheel rim/hub/tire.
    for leg in ("fork_leg_0", "fork_leg_1"):
        for elem in (
            "front_rim",
            "front_rim__component_001",
            "front_rim__component_002",
            "front_tire",
        ):
            ctx.allow_overlap(
                column, front_wheel,
                elem_a=leg, elem_b=elem,
                reason="Fork legs straddle and hold the front wheel at its axle.",
            )

    # The curved stem passes through the body head tube -- the steering bearing.
    ctx.allow_overlap(
        body, column,
        elem_a="head_tube", elem_b="curved_stem",
        reason="The curved steering stem turns inside the body head tube (the bearing).",
    )
    ctx.allow_overlap(
        body, column,
        elem_a="front_neck", elem_b="curved_stem",
        reason="The stem base enters the head where the body neck/head tube meet.",
    )
    # Fork legs bolt into the head right where the body neck/head tube sit.
    for leg in ("fork_leg_0", "fork_leg_1"):
        ctx.allow_overlap(
            body, column,
            elem_a="front_neck", elem_b=leg,
            reason="The fork legs join the steering head adjacent to the body neck.",
        )
        ctx.allow_overlap(
            body, column,
            elem_a="head_tube", elem_b=leg,
            reason="The fork legs emerge from the head tube where the steering bearing sits.",
        )
    ctx.expect_overlap(
        body, column, axes="z", min_overlap=0.02,
        elem_a="head_tube", elem_b="curved_stem",
        name="stem seated in the head tube",
    )

    # ---- steering rotates the front wheel + fender about the head axis ----
    rest_front = ctx.part_world_position(front_wheel)
    with ctx.pose({steering: math.radians(40.0)}):
        turned_front = ctx.part_world_position(front_wheel)
        turned_fender = ctx.part_element_world_aabb(column, elem="front_fender")
    ctx.check(
        "steering swings the front wheel sideways",
        abs(turned_front[0] - rest_front[0]) > 0.03,
        details=f"rest={rest_front}, turned={turned_front}",
    )
    fender_cx_rest = sum(c[0] for c in fender_aabb) / 2.0
    fender_cx_turn = sum(c[0] for c in turned_fender) / 2.0
    ctx.check(
        "steering turns the front fender with the wheel",
        abs(fender_cx_turn - fender_cx_rest) > 0.02,
        details=f"rest_cx={fender_cx_rest}, turn_cx={fender_cx_turn}",
    )
    # Steering must NOT move the rear wheels (they are on the fixed body).
    for i, rw in enumerate(rear_wheels):
        rw_rest = ctx.part_world_position(rw)
        with ctx.pose({steering: math.radians(40.0)}):
            rw_after = ctx.part_world_position(rw)
        ctx.check(
            f"steering leaves rear_wheel_{i} fixed",
            abs(rw_after[0] - rw_rest[0]) < 1e-6,
            details=f"rear_rest={rw_rest}, rear_after={rw_after}",
        )

    # ---- front wheel rolls about its axle: the off-axis valve marker moves ----
    valve_rest = ctx.part_element_world_aabb(front_wheel, elem="front_valve")
    with ctx.pose({front_roll: math.pi / 2.0}):
        valve_roll = ctx.part_element_world_aabb(front_wheel, elem="front_valve")
    vc_rest = ((valve_rest[0][1] + valve_rest[1][1]) / 2.0, (valve_rest[0][2] + valve_rest[1][2]) / 2.0)
    vc_roll = ((valve_roll[0][1] + valve_roll[1][1]) / 2.0, (valve_roll[0][2] + valve_roll[1][2]) / 2.0)
    ctx.check(
        "front wheel rolls (valve marker moves in YZ)",
        abs(vc_roll[0] - vc_rest[0]) > 0.02 or abs(vc_roll[1] - vc_rest[1]) > 0.02,
        details=f"rest={vc_rest}, rolled={vc_roll}",
    )

    # ---- each rear wheel rolls independently about its axle ----
    for i, (rw, rr) in enumerate(zip(rear_wheels, rear_rolls)):
        rvalve_rest = ctx.part_element_world_aabb(rw, elem=f"rear_valve_{i}")
        with ctx.pose({rr: math.pi / 2.0}):
            rvalve_roll = ctx.part_element_world_aabb(rw, elem=f"rear_valve_{i}")
        rvc_rest = ((rvalve_rest[0][1] + rvalve_rest[1][1]) / 2.0, (rvalve_rest[0][2] + rvalve_rest[1][2]) / 2.0)
        rvc_roll = ((rvalve_roll[0][1] + rvalve_roll[1][1]) / 2.0, (rvalve_roll[0][2] + rvalve_roll[1][2]) / 2.0)
        ctx.check(
            f"rear_wheel_{i} rolls (valve marker moves in YZ)",
            abs(rvc_roll[0] - rvc_rest[0]) > 0.02 or abs(rvc_roll[1] - rvc_rest[1]) > 0.02,
            details=f"rest={rvc_rest}, rolled={rvc_roll}",
        )

    # Rear fenders arc over the rear wheels; allow the close nested overlap.
    for i, rw in enumerate(rear_wheels):
        ctx.allow_overlap(
            body, rw,
            elem_a=f"rear_fender_{i}", elem_b=f"rear_tire_{i}",
            reason=f"The rear fender {i} intentionally arcs closely over rear wheel {i}.",
        )

    # The rear axle beam sits at the axle position and contacts both rear wheels.
    for i, rw in enumerate(rear_wheels):
        ctx.allow_overlap(
            body, rw,
            elem_a="rear_axle", elem_b=f"rear_rim_{i}",
            reason=f"The rear axle beam passes through the rear wheel {i} hub area.",
        )
        ctx.allow_overlap(
            body, rw,
            elem_a="rear_axle", elem_b=f"rear_tire_{i}",
            reason=f"The rear axle beam is near rear wheel {i} tire at the axle height.",
        )

    # ---- verify three total wheels exist (multiplicity check) ----
    all_wheel_parts = [p for p in object_model.parts if "wheel" in p.name]
    ctx.check(
        "exactly three wheel parts exist",
        len(all_wheel_parts) == 3,
        details=f"wheel parts: {[p.name for p in all_wheel_parts]}",
    )

    # ---- verify three roll joints exist ----
    all_roll_joints = [a for a in object_model.articulations if "roll" in a.name]
    ctx.check(
        "exactly three wheel roll joints exist",
        len(all_roll_joints) == 3,
        details=f"roll joints: {[a.name for a in all_roll_joints]}",
    )

    return ctx.report()


object_model = build_object_model()
