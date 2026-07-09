from __future__ import annotations

# Flat platform stock cart (single-tier galvanized steel utility cart).
#
# World frame: Z up. Long axis of the deck is X, width is Y. The four caster
# wheels touch z ~ 0.
#
# Structure:
#   - One FLAT solid rectangular sheet-steel deck slab (no raised lips) sitting
#     just above the caster crowns. This is the load surface.
#   - Tubular PUSH HANDLES at both short ends: two vertical posts rise from the
#     deck and bow over into a horizontal handle grip bar.
#   - Four swivel casters at the bottom corners.
#
# Articulation (primary, user-facing):
#   - Each of the 4 swivel casters YAWS about a vertical kingpin (continuous) and
#     each wheel ROLLS about its horizontal axle (continuous).
#
# Root part = platform_deck. End frames and casters attach to it.
# Each caster: platform_deck --(continuous Z yaw)--> yoke --(continuous spin)--> wheel.

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
DECK_LEN = 0.78  # X
DECK_WID = 0.46  # Y
DECK_THK = 0.025  # solid flat slab thickness

WHEEL_RADIUS = 0.052
WHEEL_WIDTH = 0.034
FORK_OFFSET = 0.032

# Deck top surface height – sits just above caster crowns
DECK_TOP_Z = 0.195
DECK_CENTER_Z = DECK_TOP_Z - DECK_THK / 2.0
DECK_BOTTOM_Z = DECK_TOP_Z - DECK_THK

LEG_R = 0.013  # handle frame tube radius

# Handle frame / caster corner positions
POST_X = DECK_LEN / 2.0 - 0.045
POST_Y = DECK_WID / 2.0 - 0.040
CASTER_X = DECK_LEN / 2.0 - 0.070
CASTER_Y = DECK_WID / 2.0 - 0.060

HANDLE_TOP_Z = 0.88  # handle grip height above floor (comfortable push height)
HANDLE_OUT = 0.035  # handle bows outward past the deck end


def _tube(points, name, *, radius=LEG_R):
    geom = tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=10,
        radial_segments=14,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, name)


def _build_caster(model, deck, idx, cx, cy, *, steel, rubber, rim_silver,
                  caster_top_z, deck_bottom_z):
    """Build one swivel caster: yoke part + wheel part + two articulations."""
    leg_bottom_z = -(caster_top_z - WHEEL_RADIUS)
    leg_top_z = leg_bottom_z + WHEEL_RADIUS + 0.012
    leg_half_y = WHEEL_WIDTH / 2.0 + 0.011

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
        origin=Origin(xyz=(-FORK_OFFSET, 0.0, leg_bottom_z),
                      rpy=(math.pi / 2.0, 0.0, 0.0)),
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
        hub=WheelHub(radius=WHEEL_RADIUS * 0.45, width=WHEEL_WIDTH * 0.8,
                     cap_style="flat"),
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
        f"deck_to_caster_yoke_{idx}",
        ArticulationType.CONTINUOUS,
        parent=deck,
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flat_platform_cart")

    galv = model.material("galvanized", rgba=(0.66, 0.67, 0.69, 1.0))
    galv_dark = model.material("galvanized_dark", rgba=(0.58, 0.59, 0.61, 1.0))
    galv_leg = model.material("galvanized_leg", rgba=(0.60, 0.61, 0.63, 1.0))
    rubber = model.material("caster_tread", rgba=(0.14, 0.14, 0.16, 1.0))
    steel = model.material("caster_steel", rgba=(0.50, 0.51, 0.53, 1.0))
    rim_silver = model.material("rim_silver", rgba=(0.78, 0.79, 0.81, 1.0))

    # ---- platform deck (root) ----
    # A single flat solid rectangular sheet-steel slab. No raised lips.
    deck = model.part("platform_deck")
    deck.visual(
        Box((DECK_LEN, DECK_WID, DECK_THK)),
        origin=Origin(xyz=(0.0, 0.0, DECK_CENTER_Z)),
        material=galv,
        name="deck_slab",
    )
    # Subtle underside reinforcement rib pattern (two longitudinal ribs)
    for sy in (-1.0, 1.0):
        deck.visual(
            Box((DECK_LEN - 0.12, 0.020, 0.018),
            ),
            origin=Origin(xyz=(0.0, sy * 0.10, DECK_BOTTOM_Z - 0.009)),
            material=galv_dark,
            name=f"deck_rib_{'p' if sy > 0 else 'n'}",
        )
    # Cross rib at center
    deck.visual(
        Box((0.020, DECK_WID - 0.10, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, DECK_BOTTOM_Z - 0.009)),
        material=galv_dark,
        name="deck_rib_center",
    )
    deck.inertial = Inertial.from_geometry(
        Box((DECK_LEN, DECK_WID, DECK_THK)), mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, DECK_CENTER_Z)),
    )

    # ---- end frames: tubular push handles at both short ends ----
    # Each end frame is one continuous tube: two vertical posts rising from the
    # deck underside, connected by a handle grip bar that bows outward at the top.
    for i, sx in enumerate((-1.0, 1.0)):
        tag = "p" if sx > 0 else "n"
        ef = model.part(f"end_frame_{tag}")
        hx = sx * POST_X
        leg_base_z = DECK_BOTTOM_Z  # posts start at deck underside
        # One swept tube run: up -Y post, over handle, down +Y post
        run = [
            (hx, -POST_Y, leg_base_z),
            (hx, -POST_Y, DECK_TOP_Z + 0.02),
            (hx, -POST_Y, HANDLE_TOP_Z - 0.05),
            (hx + sx * HANDLE_OUT, -POST_Y * 0.9, HANDLE_TOP_Z),
            (hx + sx * HANDLE_OUT, 0.0, HANDLE_TOP_Z + 0.008),
            (hx + sx * HANDLE_OUT, POST_Y * 0.9, HANDLE_TOP_Z),
            (hx, POST_Y, HANDLE_TOP_Z - 0.05),
            (hx, POST_Y, DECK_TOP_Z + 0.02),
            (hx, POST_Y, leg_base_z),
        ]
        ef.visual(_tube(run, f"end_frame_{tag}_run"),
                  material=galv_leg, name=f"end_frame_{tag}_run")
        ef.inertial = Inertial.from_geometry(
            Box((0.06, DECK_WID, HANDLE_TOP_Z - leg_base_z)), mass=2.5,
            origin=Origin(xyz=(hx, 0.0, (HANDLE_TOP_Z + leg_base_z) / 2.0)),
        )
        model.articulation(
            f"deck_to_end_frame_{tag}",
            ArticulationType.FIXED,
            parent=deck,
            child=ef,
        )

    # ---- four swivel casters under the deck ----
    caster_top_z = DECK_BOTTOM_Z  # casters mount to deck underside

    # Corner positions for casters
    caster_positions = [
        (CASTER_X, CASTER_Y),
        (CASTER_X, -CASTER_Y),
        (-CASTER_X, CASTER_Y),
        (-CASTER_X, -CASTER_Y),
    ]
    for i in range(4):
        cx, cy = caster_positions[i]
        _build_caster(
            model, deck, i, cx, cy,
            steel=steel, rubber=rubber, rim_silver=rim_silver,
            caster_top_z=caster_top_z, deck_bottom_z=DECK_BOTTOM_Z,
        )

    return model


def run_tests():
    from sdk import TestContext, ArticulationType as AT

    ctx = TestContext(object_model)
    deck = object_model.get_part("platform_deck")
    end_p = object_model.get_part("end_frame_p")
    end_n = object_model.get_part("end_frame_n")

    ctx.check("platform_deck_present", deck is not None, "missing")

    # No upper tray should exist
    upper_names = [p.name for p in object_model.parts]
    ctx.check("no_upper_tray", "upper_tray" not in upper_names,
              "upper_tray should be removed")

    # Deck slab is a flat surface (no raised lips). Verify deck_slab exists and
    # there are no lip_* visuals on the deck.
    deck_slab = None
    for v in deck.visuals:
        if v.name == "deck_slab":
            deck_slab = v
        ctx.check(f"no_lip_{v.name}", not v.name.startswith("lip_"),
                  f"deck should not have lip visuals, found {v.name}")
    ctx.check("deck_slab_exists", deck_slab is not None, "missing deck_slab visual")

    # Captured axle in each wheel hub: scoped intentional overlap.
    for i in range(4):
        ctx.allow_overlap(
            object_model.get_part(f"caster_yoke_{i}"),
            object_model.get_part(f"caster_wheel_{i}"),
            elem_a="axle", elem_b="rim",
            reason="The fork axle is captured inside the wheel hub bore.",
        )

    # End-frame handle posts pass through and weld to the deck slab.
    for tag in ("p", "n"):
        ef = object_model.get_part(f"end_frame_{tag}")
        ctx.allow_overlap(
            ef, deck,
            elem_a=f"end_frame_{tag}_run", elem_b="deck_slab",
            reason="The handle frame posts pass through and weld to the deck slab.",
        )
        ctx.expect_contact(ef, deck,
                           elem_a=f"end_frame_{tag}_run", elem_b="deck_slab",
                           name=f"end_frame_{tag}_seated_on_deck")

    # Deck rides above the floor on the casters.
    deck_aabb = ctx.part_world_aabb(deck)
    if deck_aabb is not None:
        ctx.check("deck_above_floor",
                  0.09 <= deck_aabb[0][2] <= 0.22,
                  f"deck_bottom={deck_aabb[0][2]:.3f}")

    # Deck top surface is low and flat (not a high tray).
    if deck_aabb is not None:
        ctx.check("deck_is_low_platform",
                  deck_aabb[1][2] < 0.25,
                  f"deck_top={deck_aabb[1][2]:.3f} should be low")

    # Wheels touch the floor.
    lows = []
    for i in range(4):
        wa = ctx.part_world_aabb(object_model.get_part(f"caster_wheel_{i}"))
        if wa is not None:
            lows.append(wa[0][2])
    ctx.check("four_wheels", len(lows) == 4, f"found {len(lows)}")
    if lows:
        ctx.check("wheels_touch_floor",
                  all(abs(z) <= 0.012 for z in lows),
                  f"lows={['%.3f' % z for z in lows]}")

    # Handles rise well above the deck at both ends.
    hp = ctx.part_world_aabb(end_p)
    hn = ctx.part_world_aabb(end_n)
    if hp is not None and hn is not None and deck_aabb is not None:
        ctx.check("handle_p_above_deck",
                  hp[1][2] > deck_aabb[1][2] + 0.40,
                  f"end_p_top={hp[1][2]:.3f} deck_top={deck_aabb[1][2]:.3f}")
        ctx.check("handle_n_above_deck",
                  hn[1][2] > deck_aabb[1][2] + 0.40,
                  f"end_n_top={hn[1][2]:.3f} deck_top={deck_aabb[1][2]:.3f}")
        # End frames at opposite short ends of the deck
        ctx.check("end_frames_opposite",
                  hp[1][0] > 0.2 and hn[0][0] < -0.2,
                  f"end_p_xmax={hp[1][0]:.3f} end_n_xmin={hn[0][0]:.3f}")

    # Joint inventory: 4 swivel (continuous Z) + 4 spin (continuous Y) = 8 moving
    # joints, plus 2 FIXED end frames = 10 total articulations.
    for i in range(4):
        sw = object_model.get_articulation(f"deck_to_caster_yoke_{i}")
        sp = object_model.get_articulation(f"caster_spin_{i}")
        ctx.check(f"swivel_{i}_continuous_z",
                  sw.articulation_type == AT.CONTINUOUS
                  and tuple(sw.axis) == (0.0, 0.0, 1.0),
                  f"axis={sw.axis}")
        ctx.check(f"spin_{i}_continuous_y",
                  sp.articulation_type == AT.CONTINUOUS
                  and tuple(sp.axis) == (0.0, 1.0, 0.0),
                  f"axis={sp.axis}")

    # At least one real non-fixed joint exists (casters provide this).
    non_fixed = [a for a in object_model.articulations
                 if a.articulation_type != AT.FIXED]
    ctx.check("has_non_fixed_joints", len(non_fixed) >= 8,
              f"non_fixed_count={len(non_fixed)}")

    # Decisive pose: wheel spins in place (center fixed).
    wheel0 = object_model.get_part("caster_wheel_0")
    spin0 = object_model.get_articulation("caster_spin_0")
    rest = ctx.part_world_position(wheel0)
    with ctx.pose({spin0: 0.6}):
        turned = ctx.part_world_position(wheel0)
    if rest is not None and turned is not None:
        moved = sum((turned[k] - rest[k]) ** 2 for k in range(3)) ** 0.5
        ctx.check("wheel_spins_in_place", moved < 1e-4, f"moved={moved:.5f}")

    # Decisive pose: caster yoke swivels (wheel center shifts in XY, not Z).
    yoke0 = object_model.get_part("caster_yoke_0")
    swivel0 = object_model.get_articulation("deck_to_caster_yoke_0")
    rest_w = ctx.part_world_position(wheel0)
    with ctx.pose({swivel0: 1.0}):
        swiveled_w = ctx.part_world_position(wheel0)
    if rest_w is not None and swiveled_w is not None:
        dz = abs(swiveled_w[2] - rest_w[2])
        ctx.check("swivel_keeps_wheel_height", dz < 0.005,
                  f"dz={dz:.5f}")

    return ctx.report()


object_model = build_object_model()
