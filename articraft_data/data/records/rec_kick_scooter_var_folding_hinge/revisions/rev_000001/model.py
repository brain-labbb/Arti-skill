from __future__ import annotations

# Vintage GREEN kick scooter with FOLDING STEM ("Scooter Racer").
# Frame: world Z up, rolling direction along +Y (front wheel at +Y, rear at -Y),
# wheel axles along X (left/right). Ground at z=0; deck top is low (~z=0.10).
#
# Layout (side view, +Y to the right):
#   - low flat foot deck near the rear, with small branding on top.
#   - a tall, gracefully curved handlebar stem sweeping up from the FRONT of the
#     deck, split into a lower stem (fixed to steering column) and an upper stem
#     that folds forward/down via a visible knuckle hinge.
#   - front wheel under a big curved fender (turns with the steering).
#   - rear wheel under a matching curved fender (fixed to the deck/body).
#
# Articulations:
#   - steering: lower stem + fork + front fender + front wheel pivot together
#       as one column, REVOLUTE about the tilted steering (head) axis (~+/-40 deg).
#   - fold_hinge: upper stem section (carrying T-bar and grips) folds forward
#       and down toward the deck. REVOLUTE about X axis at the hinge knuckle.
#   - front wheel: CONTINUOUS roll about its axle (child of the steering column).
#   - rear wheel: CONTINUOUS roll about its axle (child of the deck/body).

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
REAR_WHEEL_Y = -0.235  # rear axle Y
FRONT_WHEEL_Y = 0.320  # front axle Y (at rest)
AXLE_Z = WHEEL_R  # axle height so wheels rest on the ground

# Steering head: located just behind/above the front wheel where the curved
# stem meets the deck neck. The head axis is tilted back from vertical.
HEAD_BASE = (0.0, 0.180, 0.150)  # bottom of the head tube
HEAD_TILT = math.radians(18.0)  # lean back of the steering axis (about +X)
# Steering axis: tilted in the YZ plane (mostly +Z, leaning toward -Y at top).
STEER_AXIS = (0.0, math.sin(HEAD_TILT), math.cos(HEAD_TILT))

# Folding hinge location partway up the curved stem (world coordinates).
HINGE_WORLD = (0.0, 0.120, 0.500)


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


def _hinge_barrel_mesh(name: str):
    """Hinge knuckle barrel: a thick cylinder along X (the pin housing)."""
    g = CylinderGeometry(0.028, 0.062, radial_segments=24)
    g.rotate_y(math.pi / 2.0)  # align along X
    return mesh_from_geometry(g, name)


def _hinge_pin_mesh(name: str):
    """Hinge pin: a thin rod along X that passes through the knuckle barrel."""
    g = CylinderGeometry(0.006, 0.082, radial_segments=16)
    g.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(g, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kick_scooter")

    green = model.material("scooter_green", rgba=(0.36, 0.55, 0.40, 1.0))
    green_dark = model.material("scooter_green_dark", rgba=(0.28, 0.44, 0.32, 1.0))
    black_grip = model.material("grip_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber = model.material("tire_black", rgba=(0.08, 0.08, 0.09, 1.0))
    whitewall = model.material("whitewall", rgba=(0.90, 0.90, 0.88, 1.0))
    metal = model.material("chrome", rgba=(0.74, 0.76, 0.79, 1.0))
    silver = model.material("brand_silver", rgba=(0.82, 0.83, 0.85, 1.0))
    dark_metal = model.material("dark_steel", rgba=(0.35, 0.36, 0.38, 1.0))

    # =====================================================================
    # ROOT BODY: foot deck + rear neck/head tube + rear fender
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
    head_tube.rotate_x(HEAD_TILT)
    head_tube.translate(HEAD_BASE[0], HEAD_BASE[1] - 0.006, HEAD_BASE[2] + 0.012)
    body.visual(mesh_from_geometry(head_tube, "head_tube"), material=green_dark, name="head_tube")

    # Rear fender arcing over the rear wheel (matches the front fender), fixed to body.
    body.visual(
        _fender(WHEEL_R + 0.010, WHEEL_W + 0.022, 0.012, span_deg=150.0, start_deg=35.0,
                name="rear_fender"),
        origin=Origin(xyz=(0.0, REAR_WHEEL_Y, AXLE_Z)),
        material=green,
        name="rear_fender",
    )
    # Strut bridging the rear of the deck to the front-top edge of the rear fender.
    fender_front_top = (
        0.0,
        REAR_WHEEL_Y + (WHEEL_R + 0.010) * math.cos(math.radians(48.0)),
        AXLE_Z + (WHEEL_R + 0.010) * math.sin(math.radians(48.0)),
    )
    rear_strut = tube_from_spline_points(
        [
            (0.0, -0.118, DECK_TOP_Z - 0.002),
            (0.0, -0.125, 0.160),
            fender_front_top,
        ],
        radius=0.011,
        samples_per_segment=8,
        radial_segments=14,
    )
    body.visual(mesh_from_geometry(rear_strut, "rear_strut"), material=green_dark, name="rear_strut")

    body.inertial = Inertial.from_geometry(
        Box((0.14, 0.55, 0.22)), mass=6.0, origin=Origin(xyz=(0.0, -0.02, 0.10))
    )

    # =====================================================================
    # REAR WHEEL: continuous roll about its axle (child of body)
    # =====================================================================
    rear_wheel = model.part("rear_wheel")
    rw_rim, rw_tire, rw_ww = _wheel_meshes("rear")
    rear_wheel.visual(rw_tire, material=rubber, name="rear_tire")
    rear_wheel.visual(rw_ww, material=whitewall, name="rear_whitewall")
    rear_wheel.visual(rw_rim, material=metal, name="rear_rim")
    rear_wheel.visual(
        Cylinder(radius=0.005, length=WHEEL_W + 0.020),
        origin=Origin(xyz=(0.0, WHEEL_R * 0.55, WHEEL_R * 0.55), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="rear_valve",
    )
    rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_R, length=WHEEL_W),
        mass=0.6,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    model.articulation(
        "rear_wheel_roll",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rear_wheel,
        origin=Origin(xyz=(0.0, REAR_WHEEL_Y, AXLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=30.0),
    )

    # =====================================================================
    # STEERING COLUMN: lower stem + fork legs + front fender + hinge barrel
    #   pivots about the head axis (revolute). The column's LOCAL frame origin
    #   is at HEAD_BASE (the steering joint origin).
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

    # Hinge point in column-local frame.
    hinge_local = (
        HINGE_WORLD[0] - HEAD_BASE[0],
        HINGE_WORLD[1] - HEAD_BASE[1],
        HINGE_WORLD[2] - HEAD_BASE[2],
    )

    # Lower stem section: from head tube up to the folding hinge.
    lower_stem = tube_from_spline_points(
        _rel(
            [
                (0.0, 0.182, 0.150),
                (0.0, 0.150, 0.320),
                (0.0, 0.120, 0.500),
            ]
        ),
        radius=0.018,
        samples_per_segment=14,
        radial_segments=18,
    )
    column.visual(mesh_from_geometry(lower_stem, "lower_stem"), material=green, name="lower_stem")

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

    # Big curved front fender arcing over the front wheel (turns with steering).
    column.visual(
        _fender(WHEEL_R + 0.012, WHEEL_W + 0.024, 0.013, span_deg=170.0, start_deg=10.0,
                name="front_fender"),
        origin=Origin(xyz=fw_axle_local),
        material=green,
        name="front_fender",
    )

    # Hinge knuckle barrel at the fold point (the fixed lower housing).
    column.visual(
        _hinge_barrel_mesh("hinge_barrel"),
        origin=Origin(xyz=hinge_local),
        material=dark_metal,
        name="hinge_barrel",
    )

    column.inertial = Inertial.from_geometry(
        Box((0.12, 0.20, 0.45)),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.08 - HEAD_BASE[1], 0.28 - HEAD_BASE[2])),
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
    # UPPER STEM (folding): carries T-bar, grips, and hinge clamp/pin.
    #   Folds forward and down about X axis at the hinge knuckle.
    #   Frame origin at the hinge point (HINGE_WORLD).
    # =====================================================================
    upper_stem = model.part("upper_stem")

    def _hrel(pts):
        """Convert world-coordinate spline points to upper_stem local frame."""
        return [(x - HINGE_WORLD[0], y - HINGE_WORLD[1], z - HINGE_WORLD[2]) for (x, y, z) in pts]

    # Upper stem tube: from hinge up to handlebar.
    upper_tube = tube_from_spline_points(
        _hrel(
            [
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
    upper_stem.visual(mesh_from_geometry(upper_tube, "upper_stem_tube"), material=green, name="upper_stem_tube")

    # Handlebar positions relative to hinge.
    bar_z = 0.823 - HINGE_WORLD[2]
    bar_y = 0.195 - HINGE_WORLD[1]

    # T handlebar crossbar (along X) at the top of the stem.
    upper_stem.visual(
        Cylinder(radius=0.014, length=0.300),
        origin=Origin(xyz=(0.0, bar_y, bar_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="handlebar_crossbar",
    )
    # Stem-to-bar gooseneck cap.
    upper_stem.visual(
        Box((0.030, 0.040, 0.030)),
        origin=Origin(xyz=(0.0, 0.193 - HINGE_WORLD[1], 0.818 - HINGE_WORLD[2])),
        material=green_dark,
        name="gooseneck",
    )
    # Two black rubber grips at the ends of the crossbar.
    for sx in (-1.0, 1.0):
        upper_stem.visual(
            Cylinder(radius=0.018, length=0.090),
            origin=Origin(xyz=(sx * 0.110, bar_y, bar_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black_grip,
            name=f"grip_{0 if sx < 0 else 1}",
        )

    # Hinge clamp plates: two side plates that straddle the barrel (clevis-style).
    for sx in (-1.0, 1.0):
        plate = BoxGeometry((0.008, 0.054, 0.054))
        plate.translate(sx * 0.035, 0.0, 0.0)
        upper_stem.visual(
            mesh_from_geometry(plate, f"hinge_plate_{0 if sx < 0 else 1}"),
            material=dark_metal,
            name=f"hinge_plate_{0 if sx < 0 else 1}",
        )

    # Hinge pin: thin rod through the knuckle barrel.
    upper_stem.visual(
        _hinge_pin_mesh("hinge_pin"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=metal,
        name="hinge_pin",
    )

    # Quick-release lever on the hinge (small tab sticking out to the side).
    upper_stem.visual(
        Box((0.030, 0.012, 0.018)),
        origin=Origin(xyz=(0.046, 0.0, 0.010)),
        material=dark_metal,
        name="hinge_lever",
    )

    upper_stem.inertial = Inertial.from_geometry(
        Box((0.30, 0.10, 0.40)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.04, 0.18)),
    )

    # Fold hinge articulation: axis along X (-1 so positive q folds forward).
    # At q=0 the stem is upright; at q=upper the handlebar folds toward the deck.
    model.articulation(
        "fold_hinge",
        ArticulationType.REVOLUTE,
        parent=column,
        child=upper_stem,
        # Origin in column-local frame (column world origin = HEAD_BASE at q=0).
        origin=Origin(xyz=hinge_local),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0, velocity=1.5,
            lower=0.0, upper=math.radians(150.0),
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
    rear_wheel = object_model.get_part("rear_wheel")
    front_wheel = object_model.get_part("front_wheel")
    column = object_model.get_part("steering_column")
    upper_stem = object_model.get_part("upper_stem")
    steering = object_model.get_articulation("steering")
    fold_hinge = object_model.get_articulation("fold_hinge")
    front_roll = object_model.get_articulation("front_wheel_roll")
    rear_roll = object_model.get_articulation("rear_wheel_roll")

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

    # ---- scooter rests on BOTH wheels at the ground plane (z ~ 0) ----
    rear_aabb = ctx.part_world_aabb(rear_wheel)
    front_aabb = ctx.part_world_aabb(front_wheel)
    ctx.check(
        "rear wheel reaches the ground",
        rear_aabb[0][2] < 0.02,
        details=f"rear min z={rear_aabb[0][2]}",
    )
    ctx.check(
        "front wheel reaches the ground",
        front_aabb[0][2] < 0.02,
        details=f"front min z={front_aabb[0][2]}",
    )
    # Wheels are at opposite ends of the deck (front +Y, rear -Y).
    rear_y = ctx.part_world_position(rear_wheel)[1]
    front_y = ctx.part_world_position(front_wheel)[1]
    ctx.check(
        "front wheel ahead of rear wheel",
        front_y > rear_y + 0.30,
        details=f"front_y={front_y}, rear_y={rear_y}",
    )

    # ---- tall stem reaches handlebar height (~0.8 m) at rest ----
    upper_aabb = ctx.part_world_aabb(upper_stem)
    ctx.check(
        "handlebar/stem is tall (reaches ~0.8 m)",
        upper_aabb[1][2] > 0.78,
        details=f"upper_stem max z={upper_aabb[1][2]}",
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

    # The lower stem passes through the body head tube -- the steering bearing.
    ctx.allow_overlap(
        body, column,
        elem_a="head_tube", elem_b="lower_stem",
        reason="The lower stem turns inside the body head tube (the steering bearing).",
    )
    ctx.allow_overlap(
        body, column,
        elem_a="front_neck", elem_b="lower_stem",
        reason="The lower stem base enters the head where the body neck/head tube meet.",
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
        elem_a="head_tube", elem_b="lower_stem",
        name="lower stem seated in the head tube",
    )

    # ---- folding hinge: visible knuckle at the junction of two stem sections ----
    barrel_aabb = ctx.part_element_world_aabb(column, elem="hinge_barrel")
    ctx.check(
        "hinge barrel exists and is visible on the column",
        barrel_aabb is not None and (barrel_aabb[1][2] - barrel_aabb[0][2]) > 0.04,
        details=f"hinge_barrel aabb={barrel_aabb}",
    )
    # The hinge barrel sits midway up the stem (between deck and handlebar).
    barrel_cz = (barrel_aabb[0][2] + barrel_aabb[1][2]) / 2.0
    ctx.check(
        "hinge barrel is partway up the stem (above deck, below handlebar)",
        0.30 < barrel_cz < 0.65,
        details=f"barrel center z={barrel_cz}",
    )

    # Hinge clamp plates and pin on the upper stem overlap with the barrel
    # (the clamp wraps around the barrel, the pin passes through it).
    for plate in ("hinge_plate_0", "hinge_plate_1"):
        ctx.allow_overlap(
            column, upper_stem,
            elem_a="hinge_barrel", elem_b=plate,
            reason="The hinge clamp plates straddle the hinge barrel in a clevis joint.",
        )
    ctx.allow_overlap(
        column, upper_stem,
        elem_a="hinge_barrel", elem_b="hinge_pin",
        reason="The hinge pin passes through the barrel as the pivot axle.",
    )
    # The lower stem and upper stem tube meet at the hinge junction.
    ctx.allow_overlap(
        column, upper_stem,
        elem_a="lower_stem", elem_b="upper_stem_tube",
        reason="The two stem sections meet at the fold hinge junction.",
    )
    ctx.allow_overlap(
        column, upper_stem,
        elem_a="hinge_barrel", elem_b="upper_stem_tube",
        reason="The upper stem tube enters the hinge barrel at the fold junction.",
    )
    ctx.allow_overlap(
        column, upper_stem,
        elem_a="lower_stem", elem_b="hinge_pin",
        reason="The hinge pin passes through the barrel seated at the lower stem top.",
    )

    # Prove the hinge clamp is at the barrel location (they're in contact).
    ctx.expect_contact(
        column, upper_stem,
        elem_a="hinge_barrel", elem_b="hinge_plate_0",
        contact_tol=0.005,
        name="hinge plate contacts the barrel at rest",
    )

    # ---- fold hinge articulation: upper stem folds forward and down ----
    handlebar_rest = ctx.part_element_world_aabb(upper_stem, elem="handlebar_crossbar")
    rest_cz = (handlebar_rest[0][2] + handlebar_rest[1][2]) / 2.0

    with ctx.pose({fold_hinge: math.radians(90.0)}):
        handlebar_folded = ctx.part_element_world_aabb(upper_stem, elem="handlebar_crossbar")
        folded_cz = (handlebar_folded[0][2] + handlebar_folded[1][2]) / 2.0
        folded_cy = (handlebar_folded[0][1] + handlebar_folded[1][1]) / 2.0

    rest_cy = (handlebar_rest[0][1] + handlebar_rest[1][1]) / 2.0
    ctx.check(
        "fold hinge lowers the handlebar",
        folded_cz < rest_cz - 0.10,
        details=f"rest_z={rest_cz}, folded_z={folded_cz}",
    )
    ctx.check(
        "fold hinge swings handlebar forward (+Y)",
        folded_cy > rest_cy + 0.05,
        details=f"rest_y={rest_cy}, folded_y={folded_cy}",
    )

    # At max fold (150 deg), handlebar should be well below its rest height.
    with ctx.pose({fold_hinge: math.radians(150.0)}):
        handlebar_max = ctx.part_element_world_aabb(upper_stem, elem="handlebar_crossbar")
        max_cz = (handlebar_max[0][2] + handlebar_max[1][2]) / 2.0
    ctx.check(
        "max fold brings handlebar near deck level",
        max_cz < rest_cz - 0.25,
        details=f"rest_z={rest_cz}, max_fold_z={max_cz}",
    )

    # Folding must NOT move the front wheel (it is on the steering column, not upper stem).
    fw_rest = ctx.part_world_position(front_wheel)
    with ctx.pose({fold_hinge: math.radians(90.0)}):
        fw_folded = ctx.part_world_position(front_wheel)
    ctx.check(
        "fold hinge does not move the front wheel",
        abs(fw_folded[2] - fw_rest[2]) < 1e-5 and abs(fw_folded[1] - fw_rest[1]) < 1e-5,
        details=f"fw_rest={fw_rest}, fw_folded={fw_folded}",
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
    # Steering must NOT move the rear wheel (it is on the fixed body).
    rear_rest = ctx.part_world_position(rear_wheel)
    with ctx.pose({steering: math.radians(40.0)}):
        rear_after = ctx.part_world_position(rear_wheel)
    ctx.check(
        "steering leaves the rear wheel fixed",
        abs(rear_after[0] - rear_rest[0]) < 1e-6,
        details=f"rear_rest={rear_rest}, rear_after={rear_after}",
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

    # ---- rear wheel rolls about its axle: the off-axis valve marker moves ----
    rvalve_rest = ctx.part_element_world_aabb(rear_wheel, elem="rear_valve")
    with ctx.pose({rear_roll: math.pi / 2.0}):
        rvalve_roll = ctx.part_element_world_aabb(rear_wheel, elem="rear_valve")
    rvc_rest = ((rvalve_rest[0][1] + rvalve_rest[1][1]) / 2.0, (rvalve_rest[0][2] + rvalve_rest[1][2]) / 2.0)
    rvc_roll = ((rvalve_roll[0][1] + rvalve_roll[1][1]) / 2.0, (rvalve_roll[0][2] + rvalve_roll[1][2]) / 2.0)
    ctx.check(
        "rear wheel rolls (valve marker moves in YZ)",
        abs(rvc_roll[0] - rvc_rest[0]) > 0.02 or abs(rvc_roll[1] - rvc_rest[1]) > 0.02,
        details=f"rest={rvc_rest}, rolled={rvc_roll}",
    )

    # Rear fender arcs over the rear wheel too; allow the close nested overlap.
    ctx.allow_overlap(
        body, rear_wheel,
        elem_a="rear_fender", elem_b="rear_tire",
        reason="The rear fender intentionally arcs closely over the rear wheel.",
    )

    return ctx.report()


object_model = build_object_model()
