from __future__ import annotations

# Symmetric flat platform utility cart (warehouse "platform truck").
#
# World frame: Z up, deck plane horizontal. The long axis of the deck is X,
# the width is Y. The footprint (caster wheels) touches z ~ 0.
#
# Structure:
#   - A flat gray steel deck (rectangular platform) rides on four swivel casters.
#   - BOTH short ends carry a TALL tubular push handle: an inverted-U upright
#     with two horizontal cross rails (ladder-like guard), each leaning outward.
#   - The cart is symmetric and can be pushed from either end.
#   - Frames are round steel tube; the deck is a solid sheet-steel slab.
#
# Articulation (primary, user-facing):
#   - Each of the 4 swivel casters YAWS about a vertical (Z) kingpin (continuous),
#     and each wheel ROLLS about its horizontal axle (continuous).
#
# Root part = deck. Both handles are FIXED to the deck. Each caster:
#   deck --(continuous Z yaw)--> caster_yoke --(continuous axle spin)--> caster_wheel.

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
DECK_LEN = 0.92  # along X
DECK_WID = 0.52  # along Y
DECK_THK = 0.045  # sheet-steel deck slab thickness

# Caster geometry
WHEEL_RADIUS = 0.055  # caster wheel rolling radius (~110 mm dia)
WHEEL_WIDTH = 0.034
FORK_OFFSET = 0.034  # caster trail: wheel axle sits behind the kingpin axis (-X local)
# Deck underside rides above the wheel top + fork crown with clearance.
DECK_BOTTOM_Z = 0.165  # wheel top ~0.110, crown ~0.130, deck above that
DECK_TOP_Z = DECK_BOTTOM_Z + DECK_THK
DECK_CENTER_Z = DECK_BOTTOM_Z + DECK_THK / 2.0

# Caster mount positions (under deck corners, inset from edges)
CASTER_INSET_X = 0.130
CASTER_INSET_Y = 0.090
CASTER_X = DECK_LEN / 2.0 - CASTER_INSET_X
CASTER_Y = DECK_WID / 2.0 - CASTER_INSET_Y

# Tubular frame
TUBE_R = 0.011  # round steel tube radius (~22 mm dia)
HANDLE_BASE_X = DECK_LEN / 2.0 - 0.055  # handle uprights sit near each end
HANDLE_TOP_Z = DECK_TOP_Z + 0.93  # tall handle ~1.0 m above floor
HANDLE_HALF_W = DECK_WID / 2.0 - 0.070  # upright spacing (Y)
HANDLE_LEAN = 0.085  # top leans outward (away from deck center) for push angle


def _tube(points, *, radius=TUBE_R):
    return tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=10,
        radial_segments=14,
        cap_ends=True,
    )


def _upright_x_at_frac(base_x, top_z, base_z, lean, frac):
    """Approximate the upright centerline x at a given height fraction of the U-frame.

    Mirrors the straight-ish lower run of `_u_frame_mesh`: near-vertical at base_x
    for the lower span, then leaning toward base_x+lean near the top bend."""
    z = base_z + (top_z - base_z) * frac
    z_mid = base_z + (top_z - base_z) * 0.45  # control knee
    if z <= z_mid:
        return base_x - 0.004 * (frac / 0.45) * (1.0 if lean >= 0 else -1.0)
    # between knee and the top bend lean
    top_run_frac = (z - z_mid) / max(top_z - 0.06 - z_mid, 1e-6)
    top_run_frac = min(max(top_run_frac, 0.0), 1.0)
    knee_x = base_x - 0.004 * (1.0 if lean >= 0 else -1.0)
    return knee_x + (base_x + lean * 0.5 - knee_x) * top_run_frac


def _u_frame_mesh(*, base_x, top_z, half_w, lean, base_z, name):
    """Inverted-U tubular frame: two uprights joined by a top bend, rooted at base_z."""
    # The lean direction is encoded in the sign of lean: positive leans +X, negative -X.
    lean_sign = 1.0 if lean >= 0 else -1.0
    knee_offset = 0.004 * lean_sign
    pts = [
        (base_x, -half_w, base_z),
        (base_x - knee_offset, -half_w, base_z + (top_z - base_z) * 0.45),
        (base_x + lean * 0.5, -half_w, top_z - 0.06),
        (base_x + lean, -half_w * 0.96, top_z),
        (base_x + lean, 0.0, top_z + 0.012),  # rounded top crown
        (base_x + lean, half_w * 0.96, top_z),
        (base_x + lean * 0.5, half_w, top_z - 0.06),
        (base_x - knee_offset, half_w, base_z + (top_z - base_z) * 0.45),
        (base_x, half_w, base_z),
    ]
    return mesh_from_geometry(_tube(pts), name)


def _cross_rail_mesh(*, x, z, half_w, name):
    # Extend past the uprights so the straight rail tube intersects them.
    reach = half_w + 0.018
    pts = [(x, -reach, z), (x, 0.0, z), (x, reach, z)]
    return mesh_from_geometry(_tube(pts, radius=TUBE_R * 0.82), name)


def _add_push_handle(model, deck, idx, *, base_x, lean, frame_gray):
    """Shared helper: emit one tall tubular push-handle assembly FIXED to the deck.

    Each handle is an inverted-U loop with two ladder cross rails.
    idx: 0 for +X end, 1 for -X end.
    base_x: X position of the upright bases.
    lean: signed outward lean of the top (positive = +X, negative = -X).
    """
    handle = model.part(f"handle_{idx}")
    handle.visual(
        _u_frame_mesh(
            base_x=base_x,
            top_z=HANDLE_TOP_Z,
            half_w=HANDLE_HALF_W,
            lean=lean,
            base_z=DECK_TOP_Z,
            name=f"handle_{idx}_loop",
        ),
        material=frame_gray,
        name=f"handle_{idx}_loop",
    )
    # Two ladder cross rails on the handle
    for i, frac in enumerate((0.40, 0.70)):
        z = DECK_TOP_Z + (HANDLE_TOP_Z - DECK_TOP_Z) * frac
        x = _upright_x_at_frac(base_x, HANDLE_TOP_Z, DECK_TOP_Z, lean, frac)
        handle.visual(
            _cross_rail_mesh(
                x=x, z=z, half_w=HANDLE_HALF_W, name=f"handle_{idx}_rail_{i}"
            ),
            material=frame_gray,
            name=f"handle_{idx}_rail_{i}",
        )
    model.articulation(
        f"deck_to_handle_{idx}",
        ArticulationType.FIXED,
        parent=deck,
        child=handle,
    )
    return handle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flat_platform_cart")

    deck_gray = model.material("deck_gray", rgba=(0.62, 0.63, 0.65, 1.0))
    frame_gray = model.material("frame_gray", rgba=(0.70, 0.71, 0.73, 1.0))
    rubber = model.material("caster_tread", rgba=(0.16, 0.16, 0.18, 1.0))
    steel = model.material("caster_steel", rgba=(0.55, 0.56, 0.58, 1.0))
    rim_silver = model.material("rim_silver", rgba=(0.80, 0.81, 0.83, 1.0))

    # ---- deck (root) ----
    deck = model.part("deck")
    deck.visual(
        Box((DECK_LEN, DECK_WID, DECK_THK)),
        origin=Origin(xyz=(0.0, 0.0, DECK_CENTER_Z)),
        material=deck_gray,
        name="deck_slab",
    )
    # rolled-edge lip around the deck top perimeter (long sides) for realism
    for sy in (-1.0, 1.0):
        deck.visual(
            Box((DECK_LEN, 0.018, DECK_THK + 0.020)),
            origin=Origin(xyz=(0.0, sy * (DECK_WID / 2.0 - 0.009), DECK_CENTER_Z + 0.010)),
            material=deck_gray,
            name=f"deck_lip_{'p' if sy > 0 else 'n'}",
        )
    deck.inertial = Inertial.from_geometry(
        Box((DECK_LEN, DECK_WID, DECK_THK)), mass=26.0,
        origin=Origin(xyz=(0.0, 0.0, DECK_CENTER_Z)),
    )

    # ---- two symmetric tall push handles (one at each short end) ----
    # Handle 0 at +X end, leans outward (+X)
    _add_push_handle(
        model, deck, 0,
        base_x=HANDLE_BASE_X,
        lean=HANDLE_LEAN,
        frame_gray=frame_gray,
    )
    # Handle 1 at -X end, leans outward (-X)
    _add_push_handle(
        model, deck, 1,
        base_x=-HANDLE_BASE_X,
        lean=-HANDLE_LEAN,
        frame_gray=frame_gray,
    )

    # ---- four swivel casters (single for loop) ----
    # Yoke local frame: origin at the kingpin TOP (deck underside). +Z up.
    leg_bottom_z = -(DECK_BOTTOM_Z - WHEEL_RADIUS)  # wheel axle height (negative)
    leg_top_z = leg_bottom_z + WHEEL_RADIUS + 0.012  # crown just above the wheel top
    leg_half_y = WHEEL_WIDTH / 2.0 + 0.011

    # Caster corner positions: [(cx, cy), ...]
    caster_corners = [
        (CASTER_X, CASTER_Y),
        (CASTER_X, -CASTER_Y),
        (-CASTER_X, CASTER_Y),
        (-CASTER_X, -CASTER_Y),
    ]

    for idx in range(4):
        cx, cy = caster_corners[idx]

        yoke = model.part(f"caster_yoke_{idx}")
        # swivel top mounting plate flush under the deck
        yoke.visual(
            Box((0.060, 0.060, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, -0.005)),
            material=steel,
            name="swivel_plate",
        )
        # kingpin neck dropping from the plate
        yoke.visual(
            Cylinder(radius=0.013, length=0.046),
            origin=Origin(xyz=(0.0, 0.0, -0.027)),
            material=steel,
            name="kingpin",
        )
        # offset bracket linking the kingpin axis to the trailing fork crown
        yoke.visual(
            Box((FORK_OFFSET + 0.028, 0.040, 0.014)),
            origin=Origin(xyz=(-FORK_OFFSET / 2.0, 0.0, -0.046)),
            material=steel,
            name="offset_bracket",
        )
        # fork crown bridging the two legs above the wheel
        yoke.visual(
            Box((0.034, leg_half_y * 2.0 + 0.014, 0.014)),
            origin=Origin(xyz=(-FORK_OFFSET, 0.0, leg_top_z)),
            material=steel,
            name="fork_crown",
        )
        # offset fork legs straddling the wheel; axle trails behind kingpin (-X local)
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
        # axle bolt spanning the two legs, passing through the wheel hub bore
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
        # WheelGeometry/TireGeometry spin about local X; wheel lies in YZ -> rotate
        # so the axle is along world Y. Build at local origin; the joint places it.
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
            mesh_from_geometry(rim_geom, f"caster_rim_{idx}"),
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
            mesh_from_geometry(tire_geom, f"caster_tire_{idx}"),
            material=rubber,
            name="tire",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH), mass=0.4,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        )

        # swivel: deck -> yoke, vertical kingpin at corner, top plate under deck.
        model.articulation(
            f"deck_to_caster_yoke_{idx}",
            ArticulationType.CONTINUOUS,
            parent=deck,
            child=yoke,
            origin=Origin(xyz=(cx, cy, DECK_BOTTOM_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=12.0),
        )
        # roll: yoke -> wheel, axle along Y, trailing behind the kingpin, at wheel center.
        model.articulation(
            f"caster_spin_{idx}",
            ArticulationType.CONTINUOUS,
            parent=yoke,
            child=wheel,
            origin=Origin(xyz=(-FORK_OFFSET, 0.0, -(DECK_BOTTOM_Z - WHEEL_RADIUS))),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=40.0),
        )

    return model


def run_tests():
    from sdk import TestContext

    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    handle_0 = object_model.get_part("handle_0")
    handle_1 = object_model.get_part("handle_1")

    ctx.check("deck_present", deck is not None, "deck missing")

    # Both handles present
    ctx.check("handle_0_present", handle_0 is not None, "handle_0 missing")
    ctx.check("handle_1_present", handle_1 is not None, "handle_1 missing")

    # The axle is captured in each wheel hub: scoped, intentional overlap.
    for i in range(4):
        ctx.allow_overlap(
            object_model.get_part(f"caster_yoke_{i}"),
            object_model.get_part(f"caster_wheel_{i}"),
            elem_a="axle",
            elem_b="rim",
            reason="The fork axle is captured inside the wheel hub bore.",
        )

    # Deck sits above the floor on the casters (not buried, not floating absurdly).
    deck_aabb = ctx.part_world_aabb(deck)
    if deck_aabb is not None:
        mins, maxs = deck_aabb
        ctx.check(
            "deck_above_floor",
            0.12 <= mins[2] <= 0.22,
            f"deck bottom z={mins[2]:.3f}",
        )

    # Lowest wheel point touches z ~ 0 (footprint on floor).
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

    # Both handles are tall (symmetric dual-handle platform truck).
    h0_aabb = ctx.part_world_aabb(handle_0)
    h1_aabb = ctx.part_world_aabb(handle_1)
    if h0_aabb is not None and h1_aabb is not None:
        # Both reach approximately the same tall height
        ctx.check(
            "handle_0_tall_enough",
            0.85 <= h0_aabb[1][2] <= 1.25,
            f"handle_0_top={h0_aabb[1][2]:.3f}",
        )
        ctx.check(
            "handle_1_tall_enough",
            0.85 <= h1_aabb[1][2] <= 1.25,
            f"handle_1_top={h1_aabb[1][2]:.3f}",
        )
        # Both handles are approximately equal height (symmetric)
        ctx.check(
            "handles_equal_height",
            abs(h0_aabb[1][2] - h1_aabb[1][2]) < 0.02,
            f"handle_0_top={h0_aabb[1][2]:.3f} handle_1_top={h1_aabb[1][2]:.3f}",
        )
        # Handles are at opposite ends of the deck (along X axis)
        ctx.check(
            "handles_at_opposite_ends",
            h0_aabb[1][0] > 0.2 and h1_aabb[0][0] < -0.2,
            f"handle_0_xmax={h0_aabb[1][0]:.3f} handle_1_xmin={h1_aabb[0][0]:.3f}",
        )

    # Joint inventory: 8 caster joints (4 swivel continuous Z + 4 spin continuous Y).
    for i in range(4):
        sw = object_model.get_articulation(f"deck_to_caster_yoke_{i}")
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

    # Decisive pose: the wheel spins in place -- its center must not translate.
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

    # Decisive pose: a swivel caster yaws (yoke turns about Z) without lifting wheel.
    yoke0 = object_model.get_part("caster_yoke_0")
    swivel0 = object_model.get_articulation("deck_to_caster_yoke_0")
    wheel_low_rest = ctx.part_world_aabb(wheel0)[0][2]
    with ctx.pose({swivel0: math.pi / 2.0}):
        wa = ctx.part_world_aabb(object_model.get_part("caster_wheel_0"))
    if wa is not None:
        ctx.check(
            "swivel_keeps_wheel_on_floor",
            abs(wa[0][2] - wheel_low_rest) < 0.02,
            f"rest_low={wheel_low_rest:.3f} yawed_low={wa[0][2]:.3f}",
        )

    # At least one non-fixed joint exists (caster joints are CONTINUOUS)
    non_fixed = [
        a for a in object_model.articulations
        if a.articulation_type != ArticulationType.FIXED
    ]
    ctx.check(
        "has_non_fixed_joints",
        len(non_fixed) >= 1,
        f"found {len(non_fixed)} non-fixed joints",
    )

    return ctx.report()


object_model = build_object_model()
