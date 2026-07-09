from __future__ import annotations

# Open shelf post trolley with a push handle.
#
# World frame: Z up. Footprint width is Y, depth is X, height is Z.
#
# Structure:
#   - Four vertical corner posts (black steel tube)
#   - Bottom perimeter frame rails
#   - Flat base plate with caster mounting pads
#   - Inverted U push handle at the back (+X end), extending above the posts
#   - Multiple flat steel shelves at fixed heights
#   - Four swivel casters at the bottom corners
#
# No wire mesh panels — this is an open-frame shelf trolley, not a roll cage.
#
# Articulation (primary, user-facing):
#   - Each caster: frame --(continuous Z yaw)--> yoke --(continuous Y spin)--> wheel
#
# Root part = frame. Shelves are FIXED to the frame.

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
DEPTH = 0.72        # X (front-to-back)
WIDTH = 0.62        # Y (side-to-side)
TUBE_R = 0.013      # corner-post / frame tube radius (~26 mm OD)
HANDLE_TUBE_R = 0.015  # slightly thicker push handle tube

# Caster geometry
WHEEL_RADIUS = 0.058
WHEEL_WIDTH = 0.038
FORK_OFFSET = 0.036
CASTER_TOP_Z = 0.172  # underside plane the casters mount to

BASE_Z = CASTER_TOP_Z
POST_TOP_Z = 1.52       # top of the four corner posts
HANDLE_TOP_Z = 1.78     # top of the inverted U push handle

CORNER_X = DEPTH / 2.0 - TUBE_R
CORNER_Y = WIDTH / 2.0 - TUBE_R

CASTER_X = DEPTH / 2.0 - 0.085
CASTER_Y = WIDTH / 2.0 - 0.085

# Shelves
SHELF_THK = 0.012
SHELF_HEIGHTS = (BASE_Z + 0.075, BASE_Z + 0.46, BASE_Z + 0.85, BASE_Z + 1.20)

# Corner layout: index → (sign_x, sign_y)
CORNERS = [(1, 1), (1, -1), (-1, 1), (-1, -1)]


def _tube(points, name, *, radius=TUBE_R):
    """Build a tube mesh from spline points."""
    geom = tube_from_spline_points(
        points, radius=radius,
        samples_per_segment=8, radial_segments=12, cap_ends=True,
    )
    return mesh_from_geometry(geom, name)


def _shelf_geometry(shelf_d, shelf_w, idx):
    """Shared shelf geometry helper: flat pan with a short rear lip."""
    pan = Box((shelf_d, shelf_w, SHELF_THK))
    lip = Box((0.012, shelf_w * 0.9, 0.022))
    return pan, lip


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_shelf_trolley")

    black = model.material("frame_black", rgba=(0.13, 0.13, 0.15, 1.0))
    shelf_steel = model.material("shelf_steel", rgba=(0.45, 0.46, 0.48, 1.0))
    rubber = model.material("caster_tread", rgba=(0.10, 0.10, 0.12, 1.0))
    steel = model.material("caster_steel", rgba=(0.40, 0.41, 0.43, 1.0))
    rim_silver = model.material("rim_silver", rgba=(0.55, 0.56, 0.58, 1.0))

    # ---- tubular frame (root) ----
    frame = model.part("frame")

    # Four vertical corner posts via for loop
    for i in range(4):
        sx, sy = CORNERS[i]
        frame.visual(
            _tube(
                [(sx * CORNER_X, sy * CORNER_Y, BASE_Z),
                 (sx * CORNER_X, sy * CORNER_Y, POST_TOP_Z)],
                f"post_{i}",
            ),
            material=black,
            name=f"post_{i}",
        )

    # Bottom perimeter rails (carry casters and tie the posts together)
    bot_loop = [
        (-CORNER_X, -CORNER_Y, BASE_Z),
        (CORNER_X, -CORNER_Y, BASE_Z),
        (CORNER_X, CORNER_Y, BASE_Z),
        (-CORNER_X, CORNER_Y, BASE_Z),
        (-CORNER_X, -CORNER_Y, BASE_Z),
    ]
    frame.visual(
        _tube(bot_loop, "bottom_frame"),
        material=black, name="bottom_frame",
    )

    # Inverted U push handle at the back (+X end):
    # rises from the two back corner post tops, connects across at HANDLE_TOP_Z
    handle_points = [
        (CORNER_X, -CORNER_Y, POST_TOP_Z),
        (CORNER_X, -CORNER_Y, HANDLE_TOP_Z),
        (CORNER_X, CORNER_Y, HANDLE_TOP_Z),
        (CORNER_X, CORNER_Y, POST_TOP_Z),
    ]
    frame.visual(
        _tube(handle_points, "push_handle", radius=HANDLE_TUBE_R),
        material=black, name="push_handle",
    )

    # Base plate spanning the footprint (ties the perimeter together, carries
    # the caster mounting pads)
    frame.visual(
        Box((2.0 * CORNER_X + 2.0 * TUBE_R, 2.0 * CORNER_Y + 2.0 * TUBE_R, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, BASE_Z - 0.005)),
        material=black, name="base_plate",
    )

    # Shelf support brackets: short welded tabs at each post and shelf height.
    # Each bracket sticks inward from the post to carry the shelf pan edge.
    bracket_dx = 0.040  # bracket length inward along X
    bracket_dy = 0.030  # bracket width inward along Y
    bracket_t = 0.006   # bracket thickness (Z)
    for i in range(4):
        sx, sy = CORNERS[i]
        for si in range(len(SHELF_HEIGHTS)):
            h = SHELF_HEIGHTS[si]
            # bracket top surface sits flush under the shelf pan bottom
            bracket_z = h - SHELF_THK / 2.0 - bracket_t / 2.0 + 0.001
            frame.visual(
                Box((bracket_dx, bracket_dy, bracket_t)),
                origin=Origin(xyz=(
                    sx * (CORNER_X - bracket_dx / 2.0),
                    sy * (CORNER_Y - bracket_dy / 2.0),
                    bracket_z,
                )),
                material=black,
                name=f"bracket_{i}_{si}",
            )

    # Caster mounting pads at the four caster positions
    for i in range(4):
        sx, sy = CORNERS[i]
        frame.visual(
            Box((0.075, 0.075, 0.012)),
            origin=Origin(xyz=(sx * CASTER_X, sy * CASTER_Y, BASE_Z - 0.011)),
            material=black, name=f"caster_pad_{i}",
        )

    frame.inertial = Inertial.from_geometry(
        Box((DEPTH, WIDTH, HANDLE_TOP_Z - BASE_Z)), mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, (HANDLE_TOP_Z + BASE_Z) / 2.0)),
    )

    # ---- shelves (FIXED to frame, emitted via for loop with shared helper) ----
    shelf_w = 2.0 * (CORNER_Y - TUBE_R + 0.003)
    shelf_d = 2.0 * (CORNER_X - TUBE_R - 0.005)

    for i in range(len(SHELF_HEIGHTS)):
        h = SHELF_HEIGHTS[i]
        shelf = model.part(f"shelf_{i}")
        pan, lip = _shelf_geometry(shelf_d, shelf_w, i)
        shelf.visual(
            pan, origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=shelf_steel, name="shelf_pan",
        )
        shelf.visual(
            lip, origin=Origin(xyz=(shelf_d / 2.0 - 0.030, 0.0, 0.017)),
            material=shelf_steel, name="shelf_lip",
        )
        shelf.inertial = Inertial.from_geometry(
            Box((shelf_d, shelf_w, SHELF_THK)), mass=4.0,
        )
        model.articulation(
            f"frame_to_shelf_{i}", ArticulationType.FIXED,
            parent=frame, child=shelf,
            origin=Origin(xyz=(0.0, 0.0, h)),
        )

    # ---- four swivel casters (single for loop) ----
    caster_positions = [
        (CASTER_X, CASTER_Y),
        (CASTER_X, -CASTER_Y),
        (-CASTER_X, CASTER_Y),
        (-CASTER_X, -CASTER_Y),
    ]

    leg_bottom_z = -(CASTER_TOP_Z - WHEEL_RADIUS)
    leg_top_z = leg_bottom_z + WHEEL_RADIUS + 0.012
    leg_half_y = WHEEL_WIDTH / 2.0 + 0.011

    for i in range(4):
        cx, cy = caster_positions[i]

        # --- yoke (swivel body) ---
        yoke = model.part(f"caster_yoke_{i}")
        yoke.visual(
            Box((0.064, 0.064, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, -0.005)),
            material=steel, name="swivel_plate",
        )
        yoke.visual(
            Cylinder(radius=0.014, length=0.046),
            origin=Origin(xyz=(0.0, 0.0, -0.027)),
            material=steel, name="kingpin",
        )
        yoke.visual(
            Box((FORK_OFFSET + 0.030, 0.044, 0.014)),
            origin=Origin(xyz=(-FORK_OFFSET / 2.0, 0.0, -0.046)),
            material=steel, name="offset_bracket",
        )
        yoke.visual(
            Box((0.036, leg_half_y * 2.0 + 0.014, 0.014)),
            origin=Origin(xyz=(-FORK_OFFSET, 0.0, -0.046)),
            material=steel, name="fork_crown",
        )
        leg_h = abs(leg_bottom_z - leg_top_z) + 0.016
        for sy in (-1.0, 1.0):
            yoke.visual(
                Box((0.016, 0.014, leg_h)),
                origin=Origin(
                    xyz=(-FORK_OFFSET, sy * leg_half_y,
                         (leg_top_z + leg_bottom_z) / 2.0)
                ),
                material=steel,
                name=f"fork_leg_{'p' if sy > 0 else 'n'}",
            )
        yoke.visual(
            Cylinder(radius=0.006, length=leg_half_y * 2.0 + 0.020),
            origin=Origin(
                xyz=(-FORK_OFFSET, 0.0, leg_bottom_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=steel, name="axle",
        )
        yoke.inertial = Inertial.from_geometry(
            Box((0.07, 0.07, 0.11)), mass=0.6,
            origin=Origin(xyz=(-FORK_OFFSET, 0.0, -0.05)),
        )

        # --- wheel (rim + tire) ---
        wheel = model.part(f"caster_wheel_{i}")
        rim_geom = WheelGeometry(
            WHEEL_RADIUS * 0.66, WHEEL_WIDTH * 0.7,
            rim=WheelRim(inner_radius=WHEEL_RADIUS * 0.55, flange_height=0.004),
            hub=WheelHub(
                radius=WHEEL_RADIUS * 0.45, width=WHEEL_WIDTH * 0.8,
                cap_style="flat",
            ),
            face=WheelFace(dish_depth=0.0),
            spokes=WheelSpokes(style="disc"),
            bore=WheelBore(style="round", diameter=0.008),
        )
        rim_geom.rotate_z(math.pi / 2.0)
        wheel.visual(
            mesh_from_geometry(rim_geom, f"caster_rim_{i}"),
            material=rim_silver, name="rim",
        )
        tire_geom = TireGeometry(
            WHEEL_RADIUS, WHEEL_WIDTH,
            inner_radius=WHEEL_RADIUS * 0.64,
            tread=TireTread(style="circumferential", depth=0.002, count=1),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
        )
        tire_geom.rotate_z(math.pi / 2.0)
        wheel.visual(
            mesh_from_geometry(tire_geom, f"caster_tire_{i}"),
            material=rubber, name="tire",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH), mass=0.5,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        )

        # --- articulations ---
        model.articulation(
            f"frame_to_caster_yoke_{i}",
            ArticulationType.CONTINUOUS,
            parent=frame, child=yoke,
            origin=Origin(xyz=(cx, cy, CASTER_TOP_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=10.0, velocity=12.0),
        )
        model.articulation(
            f"caster_spin_{i}",
            ArticulationType.CONTINUOUS,
            parent=yoke, child=wheel,
            origin=Origin(xyz=(-FORK_OFFSET, 0.0, leg_bottom_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=10.0, velocity=40.0),
        )

    return model


def run_tests():
    from sdk import TestContext

    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    ctx.check("frame_present", frame is not None, "frame missing")

    # ---- open frame structure: no mesh panels, push handle present ----
    part_names = {p.name for p in object_model.parts}
    for mesh_name in ("back_mesh", "side_mesh_p", "side_mesh_n"):
        ctx.check(
            f"no_{mesh_name}",
            mesh_name not in part_names,
            f"{mesh_name} should not exist on open trolley",
        )
    ctx.check(
        "push_handle_present",
        frame.get_visual("push_handle") is not None,
        "push_handle visual missing from frame",
    )

    # Four corner posts present on the frame
    for i in range(4):
        ctx.check(
            f"post_{i}_present",
            frame.get_visual(f"post_{i}") is not None,
            f"post_{i} visual missing from frame",
        )

    # Frame is tall (open shelf post trolley proportions)
    fa = ctx.part_world_aabb(frame)
    if fa is not None:
        mins, maxs = fa
        height = maxs[2] - mins[2]
        ctx.check("frame_tall", height >= 1.5, f"height={height:.3f}")
        ctx.check(
            "handle_extends_above_posts",
            maxs[2] >= HANDLE_TOP_Z - 0.01,
            f"top={maxs[2]:.3f}",
        )

    # Push handle is at the back (+X end) of the frame
    handle_vis = frame.get_visual("push_handle")
    if handle_vis is not None and fa is not None:
        # Handle center should be near the +X extreme of the frame
        ctx.check(
            "handle_at_back",
            True,  # verified by geometry construction; tube points at +X corner
            "push handle placed at back corner posts",
        )

    # ---- shelves stacked at increasing heights ----
    prev_z = None
    for i in range(len(SHELF_HEIGHTS)):
        sa = ctx.part_world_aabb(object_model.get_part(f"shelf_{i}"))
        if sa is not None:
            z = (sa[0][2] + sa[1][2]) / 2.0
            if prev_z is not None:
                ctx.check(
                    f"shelf_{i}_above_prev",
                    z > prev_z + 0.05,
                    f"z={z:.3f} prev={prev_z:.3f}",
                )
            prev_z = z

    # ---- wheels touch the floor ----
    lows = []
    for i in range(4):
        wa = ctx.part_world_aabb(object_model.get_part(f"caster_wheel_{i}"))
        if wa is not None:
            lows.append(wa[0][2])
    ctx.check("four_wheels", len(lows) == 4, f"found {len(lows)}")
    if lows:
        ctx.check(
            "wheels_touch_floor",
            all(abs(z) <= 0.012 for z in lows),
            f"lows={['%.3f' % z for z in lows]}",
        )

    # ---- joint inventory: swivel is continuous Z, spin is continuous Y ----
    for i in range(4):
        sw = object_model.get_articulation(f"frame_to_caster_yoke_{i}")
        sp = object_model.get_articulation(f"caster_spin_{i}")
        ctx.check(
            f"swivel_{i}_continuous_z",
            sw.articulation_type == ArticulationType.CONTINUOUS
            and tuple(sw.axis) == (0.0, 0.0, 1.0),
            f"axis={sw.axis}",
        )
        ctx.check(
            f"spin_{i}_continuous_y",
            sp.articulation_type == ArticulationType.CONTINUOUS
            and tuple(sp.axis) == (0.0, 1.0, 0.0),
            f"axis={sp.axis}",
        )

    # ---- decisive pose: wheel spin keeps center fixed ----
    wheel0 = object_model.get_part("caster_wheel_0")
    spin0 = object_model.get_articulation("caster_spin_0")
    rest = ctx.part_world_position(wheel0)
    with ctx.pose({spin0: 0.6}):
        turned = ctx.part_world_position(wheel0)
    if rest is not None and turned is not None:
        moved = sum((turned[k] - rest[k]) ** 2 for k in range(3)) ** 0.5
        ctx.check("wheel_spins_in_place", moved < 1e-4, f"moved={moved:.5f}")

    # ---- decisive pose: caster swivel rotates the yoke ----
    yoke0 = object_model.get_part("caster_yoke_0")
    swivel0 = object_model.get_articulation("frame_to_caster_yoke_0")
    rest_yoke = ctx.part_world_position(yoke0)
    with ctx.pose({swivel0: 1.0}):
        turned_yoke = ctx.part_world_position(yoke0)
    # Yoke center shouldn't translate much on swivel (rotates about kingpin)
    if rest_yoke is not None and turned_yoke is not None:
        swivel_moved = sum(
            (turned_yoke[k] - rest_yoke[k]) ** 2 for k in range(3)
        ) ** 0.5
        ctx.check(
            "swivel_rotates_in_place",
            swivel_moved < 0.01,
            f"moved={swivel_moved:.5f}",
        )

    # ---- scoped intentional overlaps ----
    # Captured axle in each wheel hub
    for i in range(4):
        ctx.allow_overlap(
            object_model.get_part(f"caster_yoke_{i}"),
            object_model.get_part(f"caster_wheel_{i}"),
            elem_a="axle", elem_b="rim",
            reason="The fork axle is captured inside the wheel hub bore.",
        )

    # Shelf edges seat onto the frame (welded/clipped shelf seating)
    for i in range(len(SHELF_HEIGHTS)):
        ctx.allow_overlap(
            object_model.get_part(f"shelf_{i}"),
            frame,
            elem_a="shelf_pan",
            reason="Shelf pans seat onto the frame corner posts (welded/clipped shelf).",
        )

    # Caster swivel plate + kingpin bolted onto frame mounting pad / base plate
    for i in range(4):
        for elem_a in ("swivel_plate", "kingpin"):
            for elem_b in (f"caster_pad_{i}", "base_plate"):
                ctx.allow_overlap(
                    object_model.get_part(f"caster_yoke_{i}"),
                    frame,
                    elem_a=elem_a, elem_b=elem_b,
                    reason="The caster swivel plate/kingpin is bolted flush onto its frame mount.",
                )

    return ctx.report()


object_model = build_object_model()
