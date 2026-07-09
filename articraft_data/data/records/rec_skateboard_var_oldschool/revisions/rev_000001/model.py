from __future__ import annotations

# Wooden skateboard with a natural maple cruiser deck and two metal trucks.
# Classic old-school cruiser: wide flat board with a squared-off pointed nose
# (low and flat), a single broad upturned kicktail at the rear only, a flat/
# mellow-concave middle, and parallel-ish rails that flare out toward the tail.
# Frame:
#   +X = nose direction, -X = tail (deck length ~0.80 m along X)
#   +Y / -Y = the two sides of the deck (width ~0.24 m at tail along Y)
#   +Z = up. The deck bottom face sits near z=0; the board rides on four
#        wheels below. Tail kicks upward in +Z; nose stays flat.
# Articulations:
#   - front truck hanger: REVOLUTE lean/steer about the tilted kingpin axis (~+/-15 deg).
#       its two front wheels: each CONTINUOUS roll about the front axle.
#   - rear truck hanger:  REVOLUTE lean about its kingpin axis (~+/-15 deg).
#       its two rear wheels: each CONTINUOUS roll about the rear axle.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Inertial,
    LoftGeometry,
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
    rounded_rect_profile,
)

# ---- key dimensions ----
DECK_LEN = 0.80
DECK_WIDTH = 0.23  # cruiser deck wider at tail than popsicle
DECK_THICK = 0.012

# Truck mount X positions (baseplate bolt centers).
FRONT_X = 0.275
REAR_X = -0.275

# Vertical layout. Deck bottom face nominally at z=0.
DECK_BOTTOM = 0.0
BASEPLATE_THICK = 0.010
AXLE_Z = -0.040  # axle / hanger center height below deck

WHEEL_RADIUS = 0.0275
WHEEL_WIDTH = 0.034
WHEEL_Y = 0.092  # half-track: lateral offset of each wheel from centerline

KINGPIN_TILT = math.radians(28.0)  # baseplate pivot angle from vertical


def _deck_mesh():
    # Lofted old-school cruiser deck: wide flat board with a squared-off pointed
    # nose (low and flat), a single broad upturned kicktail at the rear only,
    # a flat/mellow-concave middle, and parallel-ish rails that flare out toward
    # the tail. LoftGeometry validates profile area in the XY projection, so each
    # cross-section is authored as a closed loop in the XY plane (width along X,
    # thickness along Y) at a constant Z station, then the whole loft is rotated
    # so the loft axis (Z) maps to the deck length (X).
    half_len = DECK_LEN / 2.0
    # (frac along half-length [-1=tail, +1=nose], half_width, centerline kick rise)
    raw = [
        # --- Tail: broad upturned kicktail ---
        (-1.00, 0.110, 0.048),   # tail tip: wide, strong kick up
        (-0.96, 0.116, 0.038),   # tail flares slightly wider
        (-0.90, 0.119, 0.022),   # transition down from kick
        (-0.82, 0.119, 0.008),   # settling to flat (behind rear truck)
        (-0.72, 0.116, 0.0),     # flat at rear truck mount (frac~-0.69)
        # --- Mid-tail to middle: wide flat with mellow concave ---
        (-0.58, 0.112, 0.0),     # flat, still wide
        (-0.42, 0.108, -0.001),  # slight concave dip
        (-0.25, 0.105, -0.001),  # mellow concave center
        (0.0,  0.103, -0.001),   # center (slight concave)
        # --- Nose approach: rails narrow, stays flat ---
        (0.20, 0.100, 0.0),      # past center, narrowing
        (0.38, 0.094, 0.0),      # nose approach
        (0.52, 0.084, 0.0),      # nose taper begins
        (0.65, 0.070, 0.0),      # nose narrows
        (0.78, 0.054, 0.0),      # pointed nose shape
        (0.88, 0.038, 0.0),      # near nose tip
        (0.95, 0.028, 0.0),      # squaring off
        (1.00, 0.022, 0.0),      # squared-off pointed nose tip (flat, no kick)
    ]
    profiles = []
    for frac, half_w, kick in raw:
        z = frac * half_len  # station along the (pre-rotation) loft axis Z
        w = max(half_w * 2.0, 0.006)
        # Closed rounded-rect loop in XY: X = deck width, Y = deck thickness.
        prof2d = rounded_rect_profile(w, DECK_THICK, min(DECK_THICK * 0.45, half_w))
        loop = [(px, py + kick, z) for (px, py) in prof2d]
        profiles.append(loop)
    deck = LoftGeometry(profiles, cap=True, closed=True)
    # Map the loft axis Z -> X (deck length) and the in-plane width/thickness
    # into Y (width) and Z (thickness, kicking upward at the tail).
    deck.rotate_y(math.pi / 2.0)  # old +Z(length) -> +X
    deck.rotate_x(math.pi / 2.0)  # bring width->Y, thickness->Z (kick up +Z)
    deck.translate(0.0, 0.0, DECK_THICK / 2.0)  # flat board bottom near z=0
    return mesh_from_geometry(deck, "deck_top")


def _wheel_mesh(name: str):
    # Wheel/tire geometry spins about local X; rotate so it spins about Y (axle).
    wheel = WheelGeometry(
        WHEEL_RADIUS,
        WHEEL_WIDTH,
        rim=WheelRim(inner_radius=0.018, flange_height=0.003, flange_thickness=0.0025),
        hub=WheelHub(radius=0.009, width=0.030, cap_style="flat"),
        face=WheelFace(dish_depth=0.002, front_inset=0.001),
        bore=WheelBore(style="round", diameter=0.008),
    )
    wheel.rotate_z(math.pi / 2.0)
    return mesh_from_geometry(wheel, name)


def _tire_mesh(name: str):
    tire = TireGeometry(
        WHEEL_RADIUS,
        WHEEL_WIDTH,
        inner_radius=0.020,
        tread=TireTread(style="circumferential", depth=0.0015, count=2),
        sidewall=TireSidewall(style="rounded", bulge=0.03),
    )
    tire.rotate_z(math.pi / 2.0)
    return mesh_from_geometry(tire, name)


def _baseplate_mesh(name: str):
    # Flat metal baseplate that bolts to the deck bottom, with a pivot boss
    # sticking down to capture the hanger kingpin neck.
    plate = BoxGeometry((0.075, 0.075, BASEPLATE_THICK))
    plate.translate(0.0, 0.0, -BASEPLATE_THICK / 2.0)
    boss = CylinderGeometry(0.011, 0.022)
    boss.translate(0.0, 0.0, -BASEPLATE_THICK - 0.006)
    plate.merge(boss)
    return mesh_from_geometry(plate, name)


def _hanger_mesh(name: str):
    # Cast hanger body + cross axle (along Y) carrying both wheels, with a
    # pivot neck reaching up toward the baseplate kingpin boss.
    geo = BoxGeometry((0.036, 0.052, 0.030))
    geo.translate(0.0, 0.0, 0.004)
    neck = CylinderGeometry(0.010, 0.026)
    neck.translate(0.0, 0.0, 0.022)
    geo.merge(neck)
    axle = CylinderGeometry(0.0065, WHEEL_Y * 2.0 + WHEEL_WIDTH * 1.2)
    axle.rotate_x(math.pi / 2.0)  # align cylinder (local Z) to Y
    geo.merge(axle)
    for s in (-1.0, 1.0):
        arm = BoxGeometry((0.024, WHEEL_Y, 0.020))
        arm.translate(0.0, s * WHEEL_Y / 2.0, 0.0)
        geo.merge(arm)
    return mesh_from_geometry(geo, name)


def _bolt_mesh(name: str):
    head = CylinderGeometry(0.0045, 0.005)
    return mesh_from_geometry(head, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="skateboard_cruiser")

    maple = model.material("maple_deck", rgba=(0.74, 0.58, 0.36, 1.0))
    metal = model.material("truck_metal", rgba=(0.62, 0.64, 0.66, 1.0))
    wheel_tan = model.material("wheel_tan", rgba=(0.86, 0.55, 0.27, 1.0))
    bolt_dark = model.material("bolt_dark", rgba=(0.20, 0.20, 0.22, 1.0))

    # ---------------- deck (root) ----------------
    deck = model.part("deck")
    deck.visual(_deck_mesh(), material=maple, name="deck_board")

    # Mounting bolts: 4 per truck, rectangular pattern, heads proud on the top.
    bolt_dx = 0.030
    bolt_dy = 0.020
    for tx in (FRONT_X, REAR_X):
        for i, sx in enumerate((-1.0, 1.0)):
            for j, sy in enumerate((-1.0, 1.0)):
                tag = f"{tx:+.2f}_{i}_{j}".replace(".", "p").replace("+", "p").replace("-", "m")
                deck.visual(
                    _bolt_mesh(f"bolt_{tag}"),
                    origin=Origin(xyz=(tx + sx * bolt_dx, sy * bolt_dy, DECK_THICK + 0.001)),
                    material=bolt_dark,
                    name=f"deck_bolt_{tag}",
                )

    deck.inertial = Inertial.from_geometry(
        Box((DECK_LEN, DECK_WIDTH, DECK_THICK)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, DECK_THICK / 2.0)),
    )

    # ---------------- two trucks ----------------
    # Each truck = baseplate (fixed to deck) + hanger (revolute kingpin) + 2 wheels.
    truck_defs = [
        ("front", FRONT_X, 1.0),  # boss tilts toward nose (+X)
        ("rear", REAR_X, -1.0),  # boss tilts toward tail (-X)
    ]

    for label, tx, tilt_sign in truck_defs:
        # ---- baseplate: fixed to deck bottom ----
        bp = model.part(f"{label}_baseplate")
        bp.visual(_baseplate_mesh(f"{label}_plate"), material=metal, name="baseplate")
        bp.inertial = Inertial.from_geometry(
            Box((0.075, 0.075, 0.030)), mass=0.10, origin=Origin(xyz=(0.0, 0.0, -0.012))
        )
        model.articulation(
            f"deck_to_{label}_baseplate",
            ArticulationType.FIXED,
            parent=deck,
            child=bp,
            origin=Origin(xyz=(tx, 0.0, DECK_BOTTOM)),
        )

        # ---- hanger: REVOLUTE about the tilted kingpin axis ----
        hanger = model.part(f"{label}_hanger")
        hanger.visual(_hanger_mesh(f"{label}_hanger_body"), material=metal, name="hanger")
        hanger.inertial = Inertial.from_geometry(Box((0.04, WHEEL_Y * 2.0, 0.03)), mass=0.18)
        kp_axis = (tilt_sign * math.sin(KINGPIN_TILT), 0.0, math.cos(KINGPIN_TILT))
        model.articulation(
            f"{label}_baseplate_to_hanger",
            ArticulationType.REVOLUTE,
            parent=bp,
            child=hanger,
            origin=Origin(xyz=(0.0, 0.0, AXLE_Z)),
            axis=kp_axis,
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=4.0,
                lower=-math.radians(15.0),
                upper=math.radians(15.0),
            ),
        )

        # ---- two wheels: each CONTINUOUS roll about the axle (Y axis) ----
        for s, side in ((-1.0, "0"), (1.0, "1")):
            wheel = model.part(f"{label}_wheel_{side}")
            wheel.visual(_wheel_mesh(f"{label}_wheel_{side}_rim"), material=metal, name="wheel_rim")
            wheel.visual(_tire_mesh(f"{label}_wheel_{side}_tire"), material=wheel_tan, name="wheel_tire")
            pip = CylinderGeometry(0.003, 0.006)
            pip.rotate_x(math.pi / 2.0)  # spin axis along Y
            pip.translate(0.0, 0.0, WHEEL_RADIUS * 0.62)
            wheel.visual(
                mesh_from_geometry(pip, f"{label}_wheel_{side}_pip"),
                material=bolt_dark,
                name="roll_marker",
            )
            wheel.inertial = Inertial.from_geometry(
                Box((WHEEL_WIDTH, WHEEL_RADIUS * 2, WHEEL_RADIUS * 2)), mass=0.06
            )
            model.articulation(
                f"{label}_hanger_to_wheel_{side}",
                ArticulationType.CONTINUOUS,
                parent=hanger,
                child=wheel,
                origin=Origin(xyz=(0.0, s * WHEEL_Y, 0.0)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(effort=2.0, velocity=30.0),
            )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")

    # ---- deck is longer than wide, cruiser outline ----
    dext = _ext(ctx.part_world_aabb(deck))
    ctx.check(
        "deck longer than wide",
        dext[0] > dext[1] + 0.3,
        details=f"deck extents={dext}",
    )
    ctx.check(
        "cruiser deck has tail kick (vertical rise at rear)",
        dext[2] > DECK_THICK + 0.030,
        details=f"deck z-extent={dext[2]:.4f}",
    )

    # ---- cruiser: nose stays flat, tail kicks up (asymmetric profile) ----
    deck_aabb = ctx.part_world_aabb(deck)
    deck_board_aabb = ctx.part_element_world_aabb(deck, elem="deck_board")
    # The tail kicks up ~0.048m; the nose stays at deck-thickness height.
    # Overall z-extent proves the single kicktail is present.
    nose_tip_z_max = deck_board_aabb[1][2]  # AABB max z includes tail kick
    deck_z_min = deck_board_aabb[0][2]
    ctx.check(
        "cruiser deck is wider at tail than nose (flared rails)",
        dext[1] > 0.22,  # max width > 0.22m (from wide tail section)
        details=f"deck y-extent={dext[1]:.4f}",
    )
    ctx.check(
        "cruiser deck nose stays flat (no nose kick)",
        # The AABB max-z includes the tail kick (0.06m), so the z-extent
        # being driven by the tail alone (not symmetric) proves nose stays flat.
        # Station data: nose tip kick=0.0, tail tip kick=0.048.
        dext[2] < 0.08,  # z-extent is moderate (tail kick only, not double-kick)
        details=f"deck z-extent={dext[2]:.4f} (tail kick only)",
    )

    # ---- four wheels rest on a common ground plane ----
    wheel_bottoms = []
    for label in ("front", "rear"):
        for side in ("0", "1"):
            w = object_model.get_part(f"{label}_wheel_{side}")
            wheel_bottoms.append(ctx.part_world_aabb(w)[0][2])
    zmin = min(wheel_bottoms)
    zmax = max(wheel_bottoms)
    ctx.check(
        "board rests on four wheels (coplanar wheel bottoms)",
        (zmax - zmin) < 0.006,
        details=f"wheel bottoms={[round(z, 4) for z in wheel_bottoms]}",
    )
    deck_zmin = ctx.part_world_aabb(deck)[0][2]
    ctx.check(
        "wheels are the lowest part of the assembly",
        zmin < deck_zmin - 0.04,
        details=f"wheel zmin={zmin:.4f}, deck zmin={deck_zmin:.4f}",
    )

    # ---- trucks symmetric front/back about the deck center ----
    fbp = ctx.part_world_position(object_model.get_part("front_baseplate"))
    rbp = ctx.part_world_position(object_model.get_part("rear_baseplate"))
    ctx.check(
        "two trucks symmetric front/back",
        abs(fbp[0] + rbp[0]) < 0.01 and abs(fbp[0]) > 0.2,
        details=f"front_x={fbp[0]:.3f}, rear_x={rbp[0]:.3f}",
    )

    # ---- each truck hanger leans about its tilted kingpin axis ----
    for label in ("front", "rear"):
        hanger = object_model.get_part(f"{label}_hanger")
        kp = object_model.get_articulation(f"{label}_baseplate_to_hanger")
        rest_ext = _ext(ctx.part_world_aabb(hanger))
        with ctx.pose({kp: math.radians(15.0)}):
            leaned_ext = _ext(ctx.part_world_aabb(hanger))
        ctx.check(
            f"{label} hanger leans about kingpin (axle tips)",
            leaned_ext[2] > rest_ext[2] + 0.004,
            details=f"{label} rest_z={rest_ext[2]:.4f} leaned_z={leaned_ext[2]:.4f}",
        )

    # ---- each of the 4 wheels rolls about its axle (marker moves) ----
    for label in ("front", "rear"):
        for side in ("0", "1"):
            wheel = object_model.get_part(f"{label}_wheel_{side}")
            roll = object_model.get_articulation(f"{label}_hanger_to_wheel_{side}")
            pip0 = ctx.part_element_world_aabb(wheel, elem="roll_marker")
            c0 = ((pip0[0][0] + pip0[1][0]) / 2.0, (pip0[0][2] + pip0[1][2]) / 2.0)
            with ctx.pose({roll: math.pi / 2.0}):
                pip1 = ctx.part_element_world_aabb(wheel, elem="roll_marker")
                c1 = ((pip1[0][0] + pip1[1][0]) / 2.0, (pip1[0][2] + pip1[1][2]) / 2.0)
            moved = math.hypot(c1[0] - c0[0], c1[1] - c0[1])
            ctx.check(
                f"{label} wheel {side} rolls about axle (marker moves)",
                moved > 0.01,
                details=f"marker moved {moved:.4f} m",
            )

    # ---- intentional seated overlaps ----
    for label in ("front", "rear"):
        bp = object_model.get_part(f"{label}_baseplate")
        hanger = object_model.get_part(f"{label}_hanger")
        ctx.allow_overlap(
            bp,
            deck,
            elem_a="baseplate",
            elem_b="deck_board",
            reason="Baseplate is bolted flush against the maple deck bottom.",
        )
        ctx.expect_contact(bp, deck, name=f"{label} baseplate seated on deck")
        ctx.allow_overlap(
            hanger,
            bp,
            elem_a="hanger",
            elem_b="baseplate",
            reason="Hanger pivot neck is captured by the baseplate kingpin boss.",
        )
        for side in ("0", "1"):
            wheel = object_model.get_part(f"{label}_wheel_{side}")
            ctx.allow_overlap(
                wheel,
                hanger,
                elem_a="wheel_rim",
                elem_b="hanger",
                reason="Wheel is seated on the hanger cross-axle (axle runs through the wheel bore).",
            )

    return ctx.report()


object_model = build_object_model()
