from __future__ import annotations

# Flat platform utility cart (warehouse "platform truck") with a FOLDING push handle.
#
# World frame: Z up, deck plane horizontal. The long axis of the deck is X,
# the width is Y. The footprint (caster wheels) touches z ~ 0.
#
# Reading the reference (002.png):
#   - A flat gray steel deck (rectangular platform) rides on four swivel casters.
#   - One short end carries a TALL tubular push handle that FOLDS DOWN: an inverted-U
#     upright with two horizontal cross rails, hinged at its base on the deck top.
#   - The other short end carries a SHORTER tubular end guard rail (an inverted-U
#     with a single cross rail) -- a stack stop.
#   - Frames are round steel tube; the deck is a solid sheet-steel slab.
#
# Articulation (primary, user-facing):
#   - The push handle FOLDS via a REVOLUTE hinge at its base (transverse Y axis).
#     q=0 is upright, positive q folds the handle down over the deck.
#   - Each of the 4 swivel casters YAWS about a vertical (Z) kingpin (continuous),
#     and each wheel ROLLS about its horizontal axle (continuous).
#
# Root part = deck. End guard is FIXED to the deck. Handle is REVOLUTE-hinged.
# Each caster:
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
HANDLE_X = DECK_LEN / 2.0 - 0.055  # push-handle hinge sits near the +X end
HANDLE_HEIGHT = 0.93  # handle height above hinge (total ~1.0 m above floor)
HANDLE_HALF_W = DECK_WID / 2.0 - 0.070  # upright spacing (Y)
HANDLE_LEAN = 0.085  # top leans back (+X) for a comfortable push angle

GUARD_X = -(DECK_LEN / 2.0 - 0.055)  # short end guard near the -X end
GUARD_TOP_Z = DECK_TOP_Z + 0.34  # shorter stack guard
GUARD_HALF_W = HANDLE_HALF_W

# Hinge bracket dimensions
HINGE_BRACKET_LEN = 0.050  # along Y
HINGE_BRACKET_WID = 0.030  # along X
HINGE_BRACKET_THK = 0.006  # plate thickness
HINGE_PIN_R = 0.008  # hinge pin radius
HINGE_BARREL_R = 0.014  # hinge barrel outer radius


def _tube(points, *, radius=TUBE_R):
    return tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=10,
        radial_segments=14,
        cap_ends=True,
    )


def _upright_x_at_frac_local(height, lean, frac):
    """X position of upright centerline at height fraction (local frame, hinge origin)."""
    z = height * frac
    z_mid = height * 0.45
    if z <= z_mid:
        return -0.004 * (frac / 0.45)
    top_run_frac = (z - z_mid) / max(height - 0.06 - z_mid, 1e-6)
    top_run_frac = min(max(top_run_frac, 0.0), 1.0)
    return -0.004 + (lean * 0.5 - (-0.004)) * top_run_frac


def _u_frame_mesh_local(*, height, half_w, lean, name):
    """Inverted-U tubular frame in local coords (origin at base/hinge)."""
    pts = [
        (0.0, -half_w, 0.0),
        (-0.004, -half_w, height * 0.45),
        (lean * 0.5, -half_w, height - 0.06),
        (lean, -half_w * 0.96, height),
        (lean, 0.0, height + 0.012),  # rounded top crown
        (lean, half_w * 0.96, height),
        (lean * 0.5, half_w, height - 0.06),
        (-0.004, half_w, height * 0.45),
        (0.0, half_w, 0.0),
    ]
    return mesh_from_geometry(_tube(pts), name)


def _u_frame_mesh(*, base_x, top_z, half_w, lean, base_z, name):
    """Inverted-U tubular frame: two uprights joined by a top bend, rooted at base_z."""
    pts = [
        (base_x, -half_w, base_z),
        (base_x - 0.004, -half_w, base_z + (top_z - base_z) * 0.45),
        (base_x + lean * 0.5, -half_w, top_z - 0.06),
        (base_x + lean, -half_w * 0.96, top_z),
        (base_x + lean, 0.0, top_z + 0.012),  # rounded top crown
        (base_x + lean, half_w * 0.96, top_z),
        (base_x + lean * 0.5, half_w, top_z - 0.06),
        (base_x - 0.004, half_w, base_z + (top_z - base_z) * 0.45),
        (base_x, half_w, base_z),
    ]
    return mesh_from_geometry(_tube(pts), name)


def _cross_rail_mesh(*, x, z, half_w, name):
    # Extend past the uprights so the straight rail tube intersects them.
    reach = half_w + 0.018
    pts = [(x, -reach, z), (x, 0.0, z), (x, reach, z)]
    return mesh_from_geometry(_tube(pts, radius=TUBE_R * 0.82), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flat_platform_cart")

    deck_gray = model.material("deck_gray", rgba=(0.62, 0.63, 0.65, 1.0))
    frame_gray = model.material("frame_gray", rgba=(0.70, 0.71, 0.73, 1.0))
    rubber = model.material("caster_tread", rgba=(0.16, 0.16, 0.18, 1.0))
    steel = model.material("caster_steel", rgba=(0.55, 0.56, 0.58, 1.0))
    rim_silver = model.material("rim_silver", rgba=(0.80, 0.81, 0.83, 1.0))
    hinge_dark = model.material("hinge_dark", rgba=(0.40, 0.41, 0.43, 1.0))

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

    # Hinge brackets on deck at handle end (two plates where handle pivots)
    for sy in (-1.0, 1.0):
        deck.visual(
            Box((HINGE_BRACKET_WID, HINGE_BRACKET_LEN, HINGE_BRACKET_THK)),
            origin=Origin(xyz=(
                HANDLE_X,
                sy * HANDLE_HALF_W,
                DECK_TOP_Z + HINGE_BRACKET_THK / 2.0,
            )),
            material=hinge_dark,
            name=f"hinge_bracket_{'p' if sy > 0 else 'n'}",
        )
        # Bracket upright tab rising from the plate (carries the pin hole)
        deck.visual(
            Box((HINGE_BRACKET_THK, HINGE_BRACKET_LEN, 0.030)),
            origin=Origin(xyz=(
                HANDLE_X - HINGE_BRACKET_WID / 2.0 + HINGE_BRACKET_THK / 2.0,
                sy * HANDLE_HALF_W,
                DECK_TOP_Z + 0.015,
            )),
            material=hinge_dark,
            name=f"hinge_tab_{'p' if sy > 0 else 'n'}",
        )

    deck.inertial = Inertial.from_geometry(
        Box((DECK_LEN, DECK_WID, DECK_THK)), mass=26.0,
        origin=Origin(xyz=(0.0, 0.0, DECK_CENTER_Z)),
    )

    # ---- folding push handle (REVOLUTE hinge at deck +X end) ----
    # Handle part frame origin = hinge pivot point.
    # At q=0 the handle is upright (extends in local +Z from the hinge).
    # Positive q about -Y folds the handle toward -X (down over the deck).
    handle = model.part("push_handle")

    # Main U-frame loop in local coordinates (origin at hinge base)
    handle.visual(
        _u_frame_mesh_local(
            height=HANDLE_HEIGHT, half_w=HANDLE_HALF_W,
            lean=HANDLE_LEAN, name="push_handle_loop",
        ),
        material=frame_gray,
        name="push_handle_loop",
    )

    # Two ladder cross rails on the handle (local coordinates)
    for i, frac in enumerate((0.40, 0.70)):
        z = HANDLE_HEIGHT * frac
        x = _upright_x_at_frac_local(HANDLE_HEIGHT, HANDLE_LEAN, frac)
        handle.visual(
            _cross_rail_mesh(x=x, z=z, half_w=HANDLE_HALF_W, name=f"handle_rail_{i}"),
            material=frame_gray,
            name=f"handle_rail_{i}",
        )

    # Hinge pin (transverse cylinder through the hinge barrel at each upright base)
    handle.visual(
        Cylinder(radius=HINGE_PIN_R, length=HANDLE_HALF_W * 2.0 + 0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.015), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_dark,
        name="hinge_pin",
    )
    # Hinge barrel sleeves at each upright base (wrap around the pin)
    for sy in (-1.0, 1.0):
        handle.visual(
            Cylinder(radius=HINGE_BARREL_R, length=0.028),
            origin=Origin(xyz=(0.0, sy * HANDLE_HALF_W, 0.015), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hinge_dark,
            name=f"hinge_barrel_{'p' if sy > 0 else 'n'}",
        )

    handle.inertial = Inertial.from_geometry(
        Box((0.10, HANDLE_HALF_W * 2.0, HANDLE_HEIGHT)), mass=4.5,
        origin=Origin(xyz=(HANDLE_LEAN / 2.0, 0.0, HANDLE_HEIGHT / 2.0)),
    )

    # REVOLUTE hinge: handle folds from upright (q=0) down toward flat (q~pi/2).
    # Origin at hinge pivot on deck top surface, axis = -Y so positive q folds
    # the handle toward -X (over the deck).
    model.articulation(
        "deck_to_push_handle",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=handle,
        origin=Origin(xyz=(HANDLE_X, 0.0, DECK_TOP_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.50),
    )

    # ---- short end guard (FIXED to deck, -X end) ----
    guard = model.part("end_guard")
    guard.visual(
        _u_frame_mesh(
            base_x=GUARD_X, top_z=GUARD_TOP_Z, half_w=GUARD_HALF_W,
            lean=0.0, base_z=DECK_TOP_Z, name="end_guard_loop",
        ),
        material=frame_gray,
        name="end_guard_loop",
    )
    guard.visual(
        _cross_rail_mesh(
            x=GUARD_X, z=DECK_TOP_Z + (GUARD_TOP_Z - DECK_TOP_Z) * 0.55,
            half_w=GUARD_HALF_W, name="guard_rail",
        ),
        material=frame_gray,
        name="guard_rail",
    )
    model.articulation(
        "deck_to_end_guard", ArticulationType.FIXED, parent=deck, child=guard,
    )

    # ---- four swivel casters ----
    # Yoke local frame: origin at the kingpin TOP (deck underside). +Z up.
    leg_bottom_z = -(DECK_BOTTOM_Z - WHEEL_RADIUS)  # wheel axle height (negative)
    leg_top_z = leg_bottom_z + WHEEL_RADIUS + 0.012  # crown just above the wheel top
    leg_half_y = WHEEL_WIDTH / 2.0 + 0.011

    def add_caster(idx, cx, cy):
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

    for i, (cx, cy) in enumerate([
        (CASTER_X, CASTER_Y),
        (CASTER_X, -CASTER_Y),
        (-CASTER_X, CASTER_Y),
        (-CASTER_X, -CASTER_Y),
    ]):
        add_caster(i, cx, cy)

    return model


def run_tests():
    from sdk import TestContext

    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    handle = object_model.get_part("push_handle")
    guard = object_model.get_part("end_guard")
    hinge = object_model.get_articulation("deck_to_push_handle")

    ctx.check("deck_present", deck is not None, "deck missing")
    ctx.check("handle_present", handle is not None, "handle missing")

    # ---- Handle hinge: REVOLUTE with transverse Y axis ----
    ctx.check(
        "handle_hinge_is_revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        f"type={hinge.articulation_type}",
    )
    ctx.check(
        "handle_hinge_axis_transverse_y",
        abs(hinge.axis[1]) > 0.9 and abs(hinge.axis[0]) < 0.1 and abs(hinge.axis[2]) < 0.1,
        f"axis={hinge.axis}",
    )

    # Hinge origin sits at deck top surface near the +X end
    hinge_origin = hinge.origin
    ctx.check(
        "hinge_origin_at_deck_top",
        abs(hinge_origin.xyz[2] - DECK_TOP_Z) < 0.010,
        f"hinge_z={hinge_origin.xyz[2]:.4f} deck_top={DECK_TOP_Z:.4f}",
    )
    ctx.check(
        "hinge_origin_at_handle_end",
        hinge_origin.xyz[0] > DECK_LEN / 2.0 - 0.10,
        f"hinge_x={hinge_origin.xyz[0]:.4f}",
    )

    # Motion limits: 0 = upright, upper ~ 1.5 rad (nearly flat)
    limits = hinge.motion_limits
    ctx.check(
        "handle_has_motion_limits",
        limits is not None and limits.lower is not None and limits.upper is not None,
        f"limits={limits}",
    )
    if limits and limits.lower is not None and limits.upper is not None:
        ctx.check(
            "handle_upright_at_zero",
            abs(limits.lower) < 0.05,
            f"lower={limits.lower}",
        )
        ctx.check(
            "handle_folds_near_flat",
            limits.upper > 1.2,
            f"upper={limits.upper}",
        )

    # The axle is captured in each wheel hub: scoped, intentional overlap.
    for i in range(4):
        ctx.allow_overlap(
            object_model.get_part(f"caster_yoke_{i}"),
            object_model.get_part(f"caster_wheel_{i}"),
            elem_a="axle",
            elem_b="rim",
            reason="The fork axle is captured inside the wheel hub bore.",
        )

    # Hinge pivot: brackets, tabs, barrels, and tube base all share the pivot region.
    # Deck slab contacts handle tube base at the pivot
    ctx.allow_overlap(
        deck, handle,
        elem_a="deck_slab",
        elem_b="push_handle_loop",
        reason="Handle tube base sits on the deck top surface at the pivot.",
    )
    # Bracket plates and tabs embed into handle tube and barrels at the pivot
    for side in ("p", "n"):
        ctx.allow_overlap(
            deck, handle,
            elem_a=f"hinge_bracket_{side}",
            elem_b="push_handle_loop",
            reason="Hinge bracket plate cradles the handle tube at the pivot.",
        )
        ctx.allow_overlap(
            deck, handle,
            elem_a=f"hinge_bracket_{side}",
            elem_b=f"hinge_barrel_{side}",
            reason="Hinge bracket plate embeds into the barrel sleeve at the pivot.",
        )
        ctx.allow_overlap(
            deck, handle,
            elem_a=f"hinge_tab_{side}",
            elem_b="push_handle_loop",
            reason="Hinge tab rises alongside the handle tube at the pivot.",
        )
        ctx.allow_overlap(
            deck, handle,
            elem_a=f"hinge_tab_{side}",
            elem_b=f"hinge_barrel_{side}",
            reason="Hinge tab embeds into the barrel sleeve at the pivot.",
        )
    # Paired proof: the handle pivot overlaps the bracket at the hinge (revolute proof)
    ctx.expect_overlap(
        deck, handle,
        axes="z",
        min_overlap=0.003,
        name="hinge bracket and handle overlap at pivot Z",
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

    # Push handle is clearly taller than the end guard at rest (upright).
    ha = ctx.part_world_aabb(handle)
    ga = ctx.part_world_aabb(guard)
    if ha is not None and ga is not None:
        ctx.check(
            "handle_taller_than_guard",
            ha[1][2] > ga[1][2] + 0.30,
            f"handle_top={ha[1][2]:.3f} guard_top={ga[1][2]:.3f}",
        )
        ctx.check(
            "handle_tall_enough",
            0.85 <= ha[1][2] <= 1.25,
            f"handle_top={ha[1][2]:.3f}",
        )
        # handle at +X end, guard at -X end
        ctx.check(
            "handle_and_guard_opposite_ends",
            ha[1][0] > 0.2 and ga[0][0] < -0.2,
            f"handle_xmax={ha[1][0]:.3f} guard_xmin={ga[0][0]:.3f}",
        )

    # ---- Decisive pose: handle folds DOWN when hinge rotates positive ----
    handle_top_rest = ha[1][2] if ha is not None else None
    # At q=1.5 (upper limit, fully folded), handle should be nearly flat over the deck
    with ctx.pose({hinge: 1.5}):
        ha_folded = ctx.part_world_aabb(handle)
    if handle_top_rest is not None and ha_folded is not None:
        folded_top = ha_folded[1][2]
        ctx.check(
            "handle_folds_down",
            folded_top < handle_top_rest - 0.20,
            f"rest_top={handle_top_rest:.3f} folded_top={folded_top:.3f}",
        )
        # When fully folded, handle top should be near deck level (over the deck)
        ctx.check(
            "handle_folded_near_deck",
            folded_top < DECK_TOP_Z + 0.25,
            f"folded_top={folded_top:.3f} deck_top={DECK_TOP_Z:.3f}",
        )

    # Joint inventory: 8 caster joints (4 swivel continuous Z + 4 spin continuous).
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

    return ctx.report()


object_model = build_object_model()
