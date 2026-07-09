from __future__ import annotations

# Industrial platform / stock cart (warehouse trolley).
#
# Real object: a flat wooden deck on a steel chassis, with a tall tubular steel
# push handle at one end, a wire-mesh end panel (backstop) under the handle, and
# four swivel casters underneath. The casters both roll and steer.
#
# Coordinate convention (Z-up world):
#   - +Z is up; the floor is z = 0 (all four casters touch z ~ 0).
#   - +X points toward the PUSH-HANDLE end (the handle/mesh are at +X).
#   - +Y is lateral; centerline is y = 0. Left/right members mirror across.
#
# Root structure: the DECK (wood top + steel chassis) is the root. The push
# handle and the wire-mesh backstop are fixed to the deck. Each of the four
# corners carries a swivel caster: a yoke that yaws about Z, holding a wheel
# that rolls about its axle.
#
# Articulations (x4 corners, i = 0..3):
#   - deck_to_caster_yoke_i : REVOLUTE swivel (yaw about Z)
#   - caster_yoke_i_to_wheel: CONTINUOUS wheel roll (about the yoke's Y axis)

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireSidewall,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---- key dimensions (meters) -------------------------------------------------
DECK_LEN = 0.92          # fore-aft length of the deck
DECK_WID = 0.61          # width of the deck
DECK_T = 0.035           # wood deck thickness
CHASSIS_T = 0.05         # steel chassis frame height under the deck

CASTER_WHEEL_R = 0.062   # caster wheel radius
CASTER_WHEEL_W = 0.045   # caster wheel width
CASTER_OFFSET = 0.034    # trailing offset of the wheel behind the swivel axis

# the deck top height = caster wheel diameter + chassis + a little clearance
DECK_BOTTOM_Z = 2 * CASTER_WHEEL_R + 0.012   # underside of the chassis
DECK_TOP_Z = DECK_BOTTOM_Z + CHASSIS_T + DECK_T

CASTER_INSET_X = 0.13    # caster king-pin inset from the deck ends
CASTER_INSET_Y = 0.10    # caster king-pin inset from the deck sides
CASTER_X = DECK_LEN / 2.0 - CASTER_INSET_X
CASTER_Y = DECK_WID / 2.0 - CASTER_INSET_Y
KINGPIN_Z = DECK_BOTTOM_Z + 0.012   # swivel mount height (under the chassis)

HANDLE_X = DECK_LEN / 2.0 - 0.05     # handle rises near the +X end
HANDLE_W = DECK_WID - 0.10           # handle loop width
HANDLE_H = 1.0                       # handle top height above the deck top
MESH_H = 0.42                        # wire-mesh backstop panel height


# ---------------------------------------------------------------------------
# Deck: a worn wooden plank top sitting on a welded steel perimeter chassis with
# two cross members. Authored in world coordinates; root part.
# ---------------------------------------------------------------------------
def _deck_wood_mesh() -> MeshGeometry:
    geo = MeshGeometry()
    n = 6
    gap = 0.004
    plank_w = (DECK_WID - (n - 1) * gap) / n
    for i in range(n):
        y = -DECK_WID / 2.0 + plank_w / 2.0 + i * (plank_w + gap)
        plank = BoxGeometry((DECK_LEN, plank_w, DECK_T))
        plank.translate(0.0, y, DECK_TOP_Z - DECK_T / 2.0)
        geo.merge(plank)
    return geo


def _chassis_mesh() -> MeshGeometry:
    geo = MeshGeometry()
    z = DECK_BOTTOM_Z + CHASSIS_T / 2.0
    for sy in (1, -1):
        rail = BoxGeometry((DECK_LEN, 0.045, CHASSIS_T))
        rail.translate(0.0, sy * (DECK_WID / 2.0 - 0.0225), z)
        geo.merge(rail)
    for sx in (1, -1):
        end = BoxGeometry((0.045, DECK_WID - 0.09, CHASSIS_T))
        end.translate(sx * (DECK_LEN / 2.0 - 0.0225), 0.0, z)
        geo.merge(end)
    for cx in (-DECK_LEN * 0.18, DECK_LEN * 0.18):
        cm = BoxGeometry((0.04, DECK_WID - 0.09, CHASSIS_T))
        cm.translate(cx, 0.0, z)
        geo.merge(cm)
    # four corner caster mounting plates under the chassis. Each plate is sized
    # so it reaches the perimeter end rail and side rail above it, welding it to
    # the chassis (no floating plate).
    for sx in (1, -1):
        for sy in (1, -1):
            plate = BoxGeometry((0.20, 0.18, 0.016))
            plate.translate(sx * CASTER_X, sy * CASTER_Y, DECK_BOTTOM_Z + 0.008)
            geo.merge(plate)
    return geo


# ---------------------------------------------------------------------------
# Push handle: an inverted-U tubular steel loop rising from the +X end.
# ---------------------------------------------------------------------------
def _handle_mesh() -> MeshGeometry:
    top_z = DECK_TOP_Z + HANDLE_H
    pts = [
        (HANDLE_X, HANDLE_W / 2.0, DECK_TOP_Z - 0.01),
        (HANDLE_X, HANDLE_W / 2.0, DECK_TOP_Z + HANDLE_H * 0.6),
        (HANDLE_X - 0.02, HANDLE_W / 2.0, top_z),
        (HANDLE_X - 0.06, HANDLE_W / 2.0 - 0.04, top_z + 0.005),
        (HANDLE_X - 0.06, -(HANDLE_W / 2.0 - 0.04), top_z + 0.005),
        (HANDLE_X - 0.02, -HANDLE_W / 2.0, top_z),
        (HANDLE_X, -HANDLE_W / 2.0, DECK_TOP_Z + HANDLE_H * 0.6),
        (HANDLE_X, -HANDLE_W / 2.0, DECK_TOP_Z - 0.01),
    ]
    return tube_from_spline_points(
        pts, radius=0.016, samples_per_segment=12, radial_segments=14, cap_ends=True,
    )


# ---------------------------------------------------------------------------
# Wire-mesh backstop panel filling the loop of the push handle: a grid of thin
# horizontal + vertical wires plus a perimeter frame tying it to the handle legs.
# ---------------------------------------------------------------------------
def _mesh_panel_mesh() -> MeshGeometry:
    geo = MeshGeometry()
    x = HANDLE_X - 0.02
    z0 = DECK_TOP_Z + 0.02
    z1 = z0 + MESH_H
    hw = HANDLE_W / 2.0 - 0.02
    wire_r = 0.0035
    nv = 9
    for i in range(nv):
        y = -hw + 2 * hw * (i / (nv - 1))
        w = CylinderGeometry(wire_r, z1 - z0, radial_segments=8)
        w.translate(x, y, (z0 + z1) / 2.0)
        geo.merge(w)
    nh = 6
    for i in range(nh):
        z = z0 + (z1 - z0) * (i / (nh - 1))
        w = CylinderGeometry(wire_r, 2 * hw, radial_segments=8)
        w.rotate_x(math.pi / 2.0)  # Z -> Y
        w.translate(x, 0.0, z)
        geo.merge(w)
    frame = tube_from_spline_points(
        [(x, hw, z0), (x, hw, z1), (x, -hw, z1), (x, -hw, z0), (x, hw, z0)],
        radius=0.006, samples_per_segment=2, radial_segments=8,
        closed_spline=True, cap_ends=False,
    )
    geo.merge(frame)
    return geo


# ---------------------------------------------------------------------------
# Swivel caster: a yoke (king-pin boss + offset fork) that yaws about Z, holding
# a wheel that rolls about the lateral axis. Authored in the yoke local frame:
# the king-pin swivel axis is at the local origin (= the mount plate underside).
# ---------------------------------------------------------------------------
def _caster_yoke_mesh() -> MeshGeometry:
    geo = MeshGeometry()
    boss = CylinderGeometry(0.020, 0.03, radial_segments=16)
    boss.translate(0.0, 0.0, -0.004)
    geo.merge(boss)
    wheel_ctr_z = CASTER_WHEEL_R - KINGPIN_Z  # local z of wheel center (negative)
    for sy in (1, -1):
        cheek = BoxGeometry((0.045, 0.012, abs(wheel_ctr_z) + 0.04))
        cheek.translate(
            CASTER_OFFSET, sy * (CASTER_WHEEL_W / 2.0 + 0.008),
            wheel_ctr_z / 2.0,
        )
        geo.merge(cheek)
    leg = BoxGeometry((CASTER_OFFSET + 0.02, 0.05, 0.018))
    leg.translate(CASTER_OFFSET / 2.0, 0.0, -0.006)
    geo.merge(leg)
    return geo


def _caster_wheel_mesh() -> MeshGeometry:
    return WheelGeometry(
        CASTER_WHEEL_R - 0.022,
        CASTER_WHEEL_W - 0.014,
        rim=WheelRim(inner_radius=0.026, flange_height=0.006, flange_thickness=0.003),
        hub=WheelHub(radius=0.014, width=CASTER_WHEEL_W - 0.008, cap_style="domed"),
        face=WheelFace(dish_depth=0.004, front_inset=0.002),
        spokes=WheelSpokes(style="solid"),
        bore=WheelBore(style="round", diameter=0.010),
    )


def _caster_tire_mesh() -> MeshGeometry:
    return TireGeometry(
        CASTER_WHEEL_R,
        CASTER_WHEEL_W,
        inner_radius=CASTER_WHEEL_R - 0.022,
        sidewall=TireSidewall(style="rounded", bulge=0.05),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="platform_stock_cart")

    wood = model.material("deck_wood", rgba=(0.55, 0.45, 0.32, 1.0))
    steel = model.material("chassis_steel", rgba=(0.30, 0.32, 0.36, 1.0))
    handle_steel = model.material("handle_steel", rgba=(0.28, 0.36, 0.52, 1.0))
    mesh_steel = model.material("mesh_steel", rgba=(0.40, 0.42, 0.46, 1.0))
    red_hub = model.material("caster_red", rgba=(0.62, 0.14, 0.13, 1.0))
    rubber = model.material("caster_rubber", rgba=(0.12, 0.12, 0.13, 1.0))
    yoke_steel = model.material("yoke_steel", rgba=(0.34, 0.35, 0.37, 1.0))

    # ---- deck (root): wood top + steel chassis + push handle + mesh panel ----
    deck = model.part("deck")
    deck.visual(mesh_from_geometry(_chassis_mesh(), "chassis"), material=steel, name="chassis")
    deck.visual(mesh_from_geometry(_deck_wood_mesh(), "deck_top"), material=wood, name="deck_top")
    deck.visual(mesh_from_geometry(_handle_mesh(), "push_handle"),
                material=handle_steel, name="push_handle")
    deck.visual(mesh_from_geometry(_mesh_panel_mesh(), "mesh_panel"),
                material=mesh_steel, name="mesh_panel")
    deck.inertial = Inertial.from_geometry(
        Box((DECK_LEN, DECK_WID, DECK_TOP_Z)),
        mass=20.0,
        origin=Origin(xyz=(0.0, 0.0, DECK_BOTTOM_Z + CHASSIS_T)),
    )

    # ---- four swivel casters (yaw about Z) + roll (about Y) ------------------
    corners = [(1, 1), (1, -1), (-1, 1), (-1, -1)]
    for i, (sx, sy) in enumerate(corners):
        yoke = model.part(f"caster_yoke_{i}")
        yoke.visual(mesh_from_geometry(_caster_yoke_mesh(), f"caster_yoke_{i}_body"),
                    material=yoke_steel, name=f"caster_yoke_{i}_body")
        yoke.inertial = Inertial.from_geometry(
            Box((0.12, 0.07, 0.10)), mass=0.4,
            origin=Origin(xyz=(CASTER_OFFSET / 2.0, 0.0, (CASTER_WHEEL_R - KINGPIN_Z) / 2.0)),
        )
        model.articulation(
            f"deck_to_caster_yoke_{i}",
            ArticulationType.REVOLUTE,
            parent=deck,
            child=yoke,
            origin=Origin(xyz=(sx * CASTER_X, sy * CASTER_Y, KINGPIN_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=20.0, velocity=12.0, lower=-math.pi, upper=math.pi),
        )

        wheel = model.part(f"caster_wheel_{i}")
        disc = _caster_wheel_mesh()
        tire = _caster_tire_mesh()
        disc.rotate_z(math.pi / 2.0)  # local X spin -> local Y
        tire.rotate_z(math.pi / 2.0)
        wheel.visual(mesh_from_geometry(disc, f"caster_wheel_{i}_disc"),
                     material=red_hub, name=f"caster_wheel_{i}_disc")
        wheel.visual(mesh_from_geometry(tire, f"caster_wheel_{i}_tire"),
                     material=rubber, name=f"caster_wheel_{i}_tire")
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=CASTER_WHEEL_R, length=CASTER_WHEEL_W), mass=0.4,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        )
        model.articulation(
            f"caster_yoke_{i}_to_wheel",
            ArticulationType.CONTINUOUS,
            parent=yoke,
            child=wheel,
            origin=Origin(xyz=(CASTER_OFFSET, 0.0, CASTER_WHEEL_R - KINGPIN_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=20.0, velocity=30.0),
        )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    wheels = [object_model.get_part(f"caster_wheel_{i}") for i in range(4)]
    yokes = [object_model.get_part(f"caster_yoke_{i}") for i in range(4)]
    swivels = [object_model.get_articulation(f"deck_to_caster_yoke_{i}") for i in range(4)]
    rolls = [object_model.get_articulation(f"caster_yoke_{i}_to_wheel") for i in range(4)]

    # --- caster joints: each is a swivel (Z) + roller (Y) -------------------
    ctx.check(
        "all four casters swivel (revolute) about Z",
        all(s.joint_type == ArticulationType.REVOLUTE and tuple(s.axis) == (0.0, 0.0, 1.0)
            for s in swivels),
        details=f"types={[s.joint_type for s in swivels]}",
    )
    ctx.check(
        "all four caster wheels roll (continuous) about Y",
        all(r.joint_type == ArticulationType.CONTINUOUS and tuple(r.axis) == (0.0, 1.0, 0.0)
            for r in rolls),
        details=f"types={[r.joint_type for r in rolls]}",
    )

    # --- all four casters touch the floor (z~0) -----------------------------
    for i, w in enumerate(wheels):
        wa = ctx.part_world_aabb(w)
        ctx.check(
            f"caster wheel {i} touches the floor (z~0)",
            wa is not None and -0.01 <= wa[0][2] <= 0.02,
            details=f"wheel {i} z_min={wa[0][2] if wa else None}",
        )

    # --- casters at the four corners (one per quadrant) ---------------------
    quad = set()
    for w in wheels:
        wp = ctx.part_world_position(w)
        quad.add((wp[0] > 0, wp[1] > 0))
    ctx.check(
        "casters are spread across all four deck corners",
        len(quad) == 4,
        details=f"quadrants={quad}",
    )

    # --- the wood deck is the big flat top, well above the floor -------------
    da = ctx.part_element_world_aabb(deck, elem="deck_top")
    de = _ext(da)
    ctx.check(
        "deck is a wide flat wooden platform",
        de[0] > 0.8 and de[1] > 0.5 and de[2] < 0.08,
        details=f"deck_ext={de}",
    )
    ctx.check(
        "deck sits up on the casters (top well above the floor)",
        da[0][2] > 0.10,
        details=f"deck z_min={da[0][2]}",
    )

    # --- the push handle rises tall above the deck, at the +X end -----------
    ha = ctx.part_element_world_aabb(deck, elem="push_handle")
    ctx.check(
        "push handle rises tall above the deck (+X end)",
        ha is not None and ha[1][2] > DECK_TOP_Z + 0.8 and ha[1][0] > 0.3,
        details=f"handle_top_z={ha[1][2] if ha else None}, handle_x_max={ha[1][0] if ha else None}",
    )

    # --- the wire-mesh backstop fills the handle area -----------------------
    ma = ctx.part_element_world_aabb(deck, elem="mesh_panel")
    me = _ext(ma)
    ctx.check(
        "wire-mesh panel is a wide vertical backstop under the handle",
        ma is not None and me[1] > 0.4 and me[2] > 0.3 and me[0] < 0.06,
        details=f"mesh_ext={me}",
    )
    ctx.check(
        "wire-mesh panel is at the +X handle end",
        ma is not None and ma[0][0] > 0.25,
        details=f"mesh_x_min={ma[0][0] if ma else None}",
    )

    # --- caster swivel actually yaws the wheel about Z ----------------------
    w0_rest = ctx.part_world_position(wheels[0])
    with ctx.pose({swivels[0]: math.pi / 2.0}):
        w0_yaw = ctx.part_world_position(wheels[0])
    ctx.check(
        "caster 0 yaws about its king-pin (wheel swings in XY)",
        w0_rest is not None and w0_yaw is not None
        and (abs(w0_yaw[0] - w0_rest[0]) > 0.01 or abs(w0_yaw[1] - w0_rest[1]) > 0.01),
        details=f"rest={w0_rest}, yawed={w0_yaw}",
    )

    # --- caster wheel actually rolls about Y --------------------------------
    e0 = ctx.part_world_aabb(wheels[0])
    with ctx.pose({rolls[0]: math.radians(40.0)}):
        e1 = ctx.part_world_aabb(wheels[0])
    ctx.check(
        "caster 0 wheel rolls (extent shifts under rotation about Y)",
        e0 is not None and e1 is not None
        and (abs(e1[1][0] - e0[1][0]) > 0.0005 or abs(e1[0][2] - e0[0][2]) > 0.0005
             or abs(e1[1][2] - e0[1][2]) > 0.0005),
        details=f"rest_aabb={e0}, rolled_aabb={e1}",
    )

    # --- each caster wheel held in its yoke fork (captured) -----------------
    for i in range(4):
        ctx.allow_overlap(
            wheels[i], yokes[i],
            reason="The caster wheel is captured between the swivel-fork cheeks.",
        )
    ctx.expect_contact(wheels[0], yokes[0], name="caster wheel 0 held in its fork")

    # --- each yoke king-pin seated up into its deck mount plate --------------
    for i in range(4):
        ctx.allow_overlap(
            deck, yokes[i],
            reason="The caster king-pin boss is captured up into the chassis "
                   "mount plate it swivels on.",
        )
    ctx.expect_contact(deck, yokes[0], name="caster 0 king-pin seated in chassis plate")

    return ctx.report()


object_model = build_object_model()
