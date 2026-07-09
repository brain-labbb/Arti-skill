from __future__ import annotations

# Tall wire-mesh shelf roll-cage trolley (warehouse stock/order trolley).
#
# World frame: Z up. Footprint width is Y, depth is X, height is Z. The four
# caster wheels touch z ~ 0.
#
# Reading the reference (008.png):
#   - A tall, narrow rectangular trolley built from BLACK round steel tube:
#     four vertical corner posts, a top rectangular tube frame (the push frame),
#     and bottom perimeter rails carrying the casters.
#   - WIRE MESH panels close the back and the two long sides; the FRONT is OPEN
#     for loading. (Mesh modeled with a real perforated/grid panel, not a slab.)
#   - Several flat steel SHELVES span the interior at different heights, each
#     holding cardboard boxes.
#   - Four black swivel casters at the bottom corners.
#
# Articulation (primary, user-facing):
#   - Each of the 4 swivel casters YAWS about a vertical kingpin (continuous) and
#     each wheel ROLLS about its horizontal axle (continuous). That is how a roll
#     cage actually moves: it is pushed and steers on swivel casters.
#
# Root part = frame. Mesh panels, shelves, and boxes are FIXED to the frame.
# Each caster: frame --(continuous Z yaw)--> yoke --(continuous spin)--> wheel.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
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
DEPTH = 0.72  # X (front-to-back)
WIDTH = 0.62  # Y (side-to-side)
TUBE_R = 0.013  # corner-post / frame tube radius (~26 mm)

# Caster geometry (bottom)
WHEEL_RADIUS = 0.058
WHEEL_WIDTH = 0.038
FORK_OFFSET = 0.036
CASTER_TOP_Z = 0.172  # underside plane the casters mount to (frame base height)

BASE_Z = CASTER_TOP_Z  # bottom perimeter rail height
TOP_Z = 1.52  # overall top of the push frame
CORNER_X = DEPTH / 2.0 - TUBE_R
CORNER_Y = WIDTH / 2.0 - TUBE_R

# Caster mount positions
CASTER_X = DEPTH / 2.0 - 0.085
CASTER_Y = WIDTH / 2.0 - 0.085

# Shelves
SHELF_THK = 0.012
SHELF_HEIGHTS = (BASE_Z + 0.075, BASE_Z + 0.637, BASE_Z + 1.20)
SHELF_INSET = 0.040  # shelves sit just inside the posts (clear of the corner tubes)


def _tube(points, name, *, radius=TUBE_R):
    geom = tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=8,
        radial_segments=12,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wire_mesh_roll_cage")

    black = model.material("frame_black", rgba=(0.13, 0.13, 0.15, 1.0))
    mesh_gray = model.material("wire_mesh", rgba=(0.30, 0.31, 0.33, 1.0))
    shelf_steel = model.material("shelf_steel", rgba=(0.45, 0.46, 0.48, 1.0))
    cardboard = model.material("cardboard", rgba=(0.74, 0.58, 0.36, 1.0))
    rubber = model.material("caster_tread", rgba=(0.10, 0.10, 0.12, 1.0))
    steel = model.material("caster_steel", rgba=(0.40, 0.41, 0.43, 1.0))
    rim_silver = model.material("rim_silver", rgba=(0.55, 0.56, 0.58, 1.0))

    # ---- tubular frame (root) ----
    frame = model.part("frame")

    # four vertical corner posts
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            pname = f"post_{'p' if sx > 0 else 'n'}{'p' if sy > 0 else 'n'}"
            frame.visual(
                _tube([(sx * CORNER_X, sy * CORNER_Y, BASE_Z),
                       (sx * CORNER_X, sy * CORNER_Y, TOP_Z)], pname),
                material=black,
                name=pname,
            )

    # top rectangular frame loop (the push frame)
    top_loop = [
        (-CORNER_X, -CORNER_Y, TOP_Z),
        (CORNER_X, -CORNER_Y, TOP_Z),
        (CORNER_X, CORNER_Y, TOP_Z),
        (-CORNER_X, CORNER_Y, TOP_Z),
        (-CORNER_X, -CORNER_Y, TOP_Z),
    ]
    frame.visual(_tube(top_loop, "top_frame", radius=TUBE_R), material=black, name="top_frame")

    # bottom perimeter rails (carry the casters)
    bot_loop = [
        (-CORNER_X, -CORNER_Y, BASE_Z),
        (CORNER_X, -CORNER_Y, BASE_Z),
        (CORNER_X, CORNER_Y, BASE_Z),
        (-CORNER_X, CORNER_Y, BASE_Z),
        (-CORNER_X, -CORNER_Y, BASE_Z),
    ]
    frame.visual(_tube(bot_loop, "bottom_frame", radius=TUBE_R), material=black, name="bottom_frame")

    # solid bottom base plate spanning the footprint: ties the perimeter together
    # and carries the four caster mounting pads (no floating casters).
    frame.visual(
        Box((2.0 * CORNER_X + 2.0 * TUBE_R, 2.0 * CORNER_Y + 2.0 * TUBE_R, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, BASE_Z - 0.005)),
        material=black,
        name="base_plate",
    )
    # mounting pads at the four caster positions, flush under the base plate
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            frame.visual(
                Box((0.075, 0.075, 0.012)),
                origin=Origin(xyz=(sx * CASTER_X, sy * CASTER_Y, BASE_Z - 0.011)),
                material=black,
                name=f"caster_pad_{'p' if sx > 0 else 'n'}{'p' if sy > 0 else 'n'}",
            )

    # mid horizontal rails at each shelf height (back + sides) for rigidity
    for h in SHELF_HEIGHTS:
        # back rail (along Y at +X back)
        brn = f"back_rail_{int(h*1000)}"
        frame.visual(
            _tube([(CORNER_X, -CORNER_Y, h), (CORNER_X, CORNER_Y, h)], brn, radius=TUBE_R * 0.7),
            material=black,
            name=brn,
        )
        # side rails (along X)
        for sy in (-1.0, 1.0):
            srn = f"side_rail_{'p' if sy > 0 else 'n'}_{int(h*1000)}"
            frame.visual(
                _tube([(-CORNER_X, sy * CORNER_Y, h), (CORNER_X, sy * CORNER_Y, h)],
                      srn, radius=TUBE_R * 0.7),
                material=black,
                name=srn,
            )

    frame.inertial = Inertial.from_geometry(
        Box((DEPTH, WIDTH, TOP_Z - BASE_Z)), mass=38.0,
        origin=Origin(xyz=(0.0, 0.0, (TOP_Z + BASE_Z) / 2.0)),
    )

    # ---- wire-mesh panels (FIXED to frame): back (+X) and two sides ----
    panel_h = TOP_Z - BASE_Z - 0.04
    panel_z = (TOP_Z + BASE_Z) / 2.0

    def add_mesh_panel(name, size, origin, rpy):
        part = model.part(name)
        geom = PerforatedPanelGeometry(
            size,
            0.004,
            hole_diameter=0.030,
            pitch=0.044,
            frame=0.012,
        )
        # panel lies in local XY (thickness Z); rotate into place.
        if rpy is not None:
            r, p, y = rpy
            if r:
                geom.rotate_x(r)
            if p:
                geom.rotate_y(p)
            if y:
                geom.rotate_z(y)
        part.visual(mesh_from_geometry(geom, name), material=mesh_gray, name=name)
        model.articulation(
            f"frame_to_{name}", ArticulationType.FIXED, parent=frame, child=part,
            origin=origin,
        )
        return part

    # back panel: spans Y x Z, on the +X back plane. Base panel is X=width, Y=height.
    # rotate_x(90) stands it vertical (height -> Z), then rotate_z(90) turns the
    # width to run along Y, leaving the thin axis along X (the back-plane normal).
    add_mesh_panel(
        "back_mesh",
        (WIDTH - 0.05, panel_h),
        Origin(xyz=(CORNER_X - 0.004, 0.0, panel_z)),
        rpy=(math.pi / 2.0, 0.0, math.pi / 2.0),
    )
    # side panels: span X x Z, on the +/-Y planes. Roll the XY panel up about X.
    for sy in (-1.0, 1.0):
        add_mesh_panel(
            f"side_mesh_{'p' if sy > 0 else 'n'}",
            (DEPTH - 0.05, panel_h),
            Origin(xyz=(0.0, sy * (CORNER_Y - 0.004), panel_z)),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        )

    # ---- shelves (FIXED to frame) ----
    # Shelves span almost the full interior so their edges rest on the side/back
    # mid rails (a small seating overlap, like a welded/clipped shelf).
    rail_r = TUBE_R * 0.7
    # Width reaches the side mid rails (seating overlap); depth clears the corner
    # posts so the shelf only seats on the rails, not the posts.
    shelf_w = 2.0 * (CORNER_Y - rail_r + 0.003)
    shelf_d = 2.0 * (CORNER_X - TUBE_R - 0.005)

    def _add_shelf(si: int, h: float):
        """Shared shelf geometry helper: one flat steel shelf pan with rear lip."""
        shelf = model.part(f"shelf_{si}")
        shelf.visual(
            Box((shelf_d, shelf_w, SHELF_THK)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=shelf_steel,
            name="shelf_pan",
        )
        # short upturned rear lip (kept inboard of the rear rail)
        shelf.visual(
            Box((0.012, shelf_w * 0.9, 0.022)),
            origin=Origin(xyz=(shelf_d / 2.0 - 0.030, 0.0, 0.017)),
            material=shelf_steel,
            name="shelf_lip",
        )
        shelf.inertial = Inertial.from_geometry(
            Box((shelf_d, shelf_w, SHELF_THK)), mass=4.0,
        )
        model.articulation(
            f"frame_to_shelf_{si}", ArticulationType.FIXED, parent=frame, child=shelf,
            origin=Origin(xyz=(0.0, 0.0, h)),
        )

    for si, h in enumerate(SHELF_HEIGHTS):
        _add_shelf(si, h)

    # ---- cardboard boxes resting on shelves (FIXED cargo) ----
    box_specs = [
        (0, (0.26, 0.20, 0.16), (-0.05, 0.06)),
        (1, (0.30, 0.24, 0.20), (0.04, -0.05)),
        (2, (0.24, 0.22, 0.17), (-0.04, -0.04)),
    ]
    for bi, (si, dims, (ox, oy)) in enumerate(box_specs):
        box = model.part(f"box_{bi}")
        box.visual(
            Box(dims),
            origin=Origin(xyz=(0.0, 0.0, dims[2] / 2.0)),
            material=cardboard,
            name="carton",
        )
        box.inertial = Inertial.from_geometry(Box(dims), mass=2.5)
        # box bottom sits on the shelf top (shelf top = +SHELF_THK/2 in shelf frame)
        model.articulation(
            f"shelf_{si}_to_box_{bi}", ArticulationType.FIXED,
            parent=model.get_part(f"shelf_{si}"), child=box,
            origin=Origin(xyz=(ox, oy, SHELF_THK / 2.0)),
        )

    # ---- four swivel casters ----
    leg_bottom_z = -(CASTER_TOP_Z - WHEEL_RADIUS)
    leg_top_z = leg_bottom_z + WHEEL_RADIUS + 0.012
    leg_half_y = WHEEL_WIDTH / 2.0 + 0.011

    def add_caster(idx, cx, cy):
        yoke = model.part(f"caster_yoke_{idx}")
        yoke.visual(
            Box((0.064, 0.064, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, -0.005)),
            material=steel,
            name="swivel_plate",
        )
        yoke.visual(
            Cylinder(radius=0.014, length=0.046),
            origin=Origin(xyz=(0.0, 0.0, -0.027)),
            material=steel,
            name="kingpin",
        )
        yoke.visual(
            Box((FORK_OFFSET + 0.030, 0.044, 0.014)),
            origin=Origin(xyz=(-FORK_OFFSET / 2.0, 0.0, -0.046)),
            material=steel,
            name="offset_bracket",
        )
        yoke.visual(
            Box((0.036, leg_half_y * 2.0 + 0.014, 0.014)),
            origin=Origin(xyz=(-FORK_OFFSET, 0.0, leg_top_z)),
            material=steel,
            name="fork_crown",
        )
        leg_h = abs(leg_bottom_z - leg_top_z) + 0.016
        for sy in (-1.0, 1.0):
            yoke.visual(
                Box((0.016, 0.014, leg_h)),
                origin=Origin(
                    xyz=(-FORK_OFFSET, sy * leg_half_y, (leg_top_z + leg_bottom_z) / 2.0)
                ),
                material=steel,
                name=f"fork_leg_{'p' if sy > 0 else 'n'}",
            )
        yoke.visual(
            Cylinder(radius=0.006, length=leg_half_y * 2.0 + 0.020),
            origin=Origin(xyz=(-FORK_OFFSET, 0.0, leg_bottom_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="axle",
        )
        yoke.inertial = Inertial.from_geometry(
            Box((0.07, 0.07, 0.11)), mass=0.6,
            origin=Origin(xyz=(-FORK_OFFSET, 0.0, -0.05)),
        )

        wheel = model.part(f"caster_wheel_{idx}")
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
            Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH), mass=0.5,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        )

        model.articulation(
            f"frame_to_caster_yoke_{idx}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=yoke,
            origin=Origin(xyz=(cx, cy, CASTER_TOP_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=10.0, velocity=12.0),
        )
        model.articulation(
            f"caster_spin_{idx}",
            ArticulationType.CONTINUOUS,
            parent=yoke,
            child=wheel,
            origin=Origin(xyz=(-FORK_OFFSET, 0.0, leg_bottom_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=10.0, velocity=40.0),
        )

    caster_positions = (
        (CASTER_X, CASTER_Y),
        (CASTER_X, -CASTER_Y),
        (-CASTER_X, CASTER_Y),
        (-CASTER_X, -CASTER_Y),
    )
    for i, (cx, cy) in enumerate(caster_positions):
        add_caster(i, cx, cy)

    return model


def run_tests():
    from sdk import TestContext

    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    ctx.check("frame_present", frame is not None, "frame missing")

    # Captured axle in each wheel hub: scoped intentional overlap.
    for i in range(4):
        ctx.allow_overlap(
            object_model.get_part(f"caster_yoke_{i}"),
            object_model.get_part(f"caster_wheel_{i}"),
            elem_a="axle",
            elem_b="rim",
            reason="The fork axle is captured inside the wheel hub bore.",
        )

    # Shelf count is driven by SHELF_HEIGHTS length.
    shelf_count = len(SHELF_HEIGHTS)
    ctx.check("shelf_count_is_three", shelf_count == 3,
              f"expected 3 shelves, got {shelf_count}")

    # Shelf edges seat onto the side mid rails: small intentional overlap.
    for si in range(shelf_count):
        ctx.allow_overlap(
            object_model.get_part(f"shelf_{si}"),
            frame,
            elem_a="shelf_pan",
            reason="Shelf pans seat onto the frame side rails (welded/clipped shelf).",
        )

    # Each caster's swivel plate + kingpin bolts onto its frame mounting pad /
    # base plate: scoped intentional overlap at the mount interface.
    for i, sxsy in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        sx, sy = sxsy
        pad = f"caster_pad_{'p' if sx > 0 else 'n'}{'p' if sy > 0 else 'n'}"
        for elem_a in ("swivel_plate", "kingpin"):
            for elem_b in (pad, "base_plate"):
                ctx.allow_overlap(
                    object_model.get_part(f"caster_yoke_{i}"),
                    frame,
                    elem_a=elem_a,
                    elem_b=elem_b,
                    reason="The caster swivel plate/kingpin is bolted flush onto its frame mount.",
                )

    # Overall is tall and narrow (roll cage proportions).
    fa = ctx.part_world_aabb(frame)
    if fa is not None:
        mins, maxs = fa
        height = maxs[2] - mins[2]
        ctx.check("frame_tall", height >= 1.2, f"height={height:.3f}")
        ctx.check("frame_top_high", maxs[2] >= 1.45, f"top={maxs[2]:.3f}")

    # Wheels touch the floor.
    lows = []
    for i in range(4):
        wa = ctx.part_world_aabb(object_model.get_part(f"caster_wheel_{i}"))
        if wa is not None:
            lows.append(wa[0][2])
    ctx.check("four_wheels", len(lows) == 4, f"found {len(lows)}")
    if lows:
        ctx.check("wheels_touch_floor", all(abs(z) <= 0.012 for z in lows),
                  f"lows={['%.3f' % z for z in lows]}")

    # Shelves stacked at increasing heights with even spacing.
    shelf_centers_z = []
    prev = None
    for si in range(len(SHELF_HEIGHTS)):
        sa = ctx.part_world_aabb(object_model.get_part(f"shelf_{si}"))
        if sa is not None:
            z = (sa[0][2] + sa[1][2]) / 2.0
            shelf_centers_z.append(z)
            if prev is not None:
                ctx.check(f"shelf_{si}_above_prev", z > prev, f"z={z:.3f} prev={prev:.3f}")
            prev = z

    # Verify shelves are evenly spaced (gaps between consecutive shelves are similar).
    if len(shelf_centers_z) == 3:
        gap_01 = shelf_centers_z[1] - shelf_centers_z[0]
        gap_12 = shelf_centers_z[2] - shelf_centers_z[1]
        ctx.check("shelves_evenly_spaced",
                  abs(gap_01 - gap_12) < 0.02,
                  f"gap_01={gap_01:.4f} gap_12={gap_12:.4f}")

    # Each box sits ON its shelf pan (box bottom ~ shelf top, contact, not floating).
    for bi, si in ((0, 0), (1, 1), (2, 2)):
        box = object_model.get_part(f"box_{bi}")
        shelf = object_model.get_part(f"shelf_{si}")
        ctx.expect_gap(box, shelf, axis="z", min_gap=-0.004, max_gap=0.004,
                       positive_elem="carton", negative_elem="shelf_pan",
                       name=f"box_{bi}_rests_on_shelf_{si}")

    # Mesh panels present on back and both sides; front open (no front panel part).
    for name in ("back_mesh", "side_mesh_p", "side_mesh_n"):
        ctx.check(f"{name}_present", object_model.get_part(name) is not None,
                  f"{name} missing")

    # Joint inventory + spin/swivel correctness.
    for i in range(4):
        sw = object_model.get_articulation(f"frame_to_caster_yoke_{i}")
        sp = object_model.get_articulation(f"caster_spin_{i}")
        ctx.check(f"swivel_{i}_continuous_z",
                  sw.articulation_type == ArticulationType.CONTINUOUS
                  and tuple(sw.axis) == (0.0, 0.0, 1.0),
                  f"axis={sw.axis}")
        ctx.check(f"spin_{i}_continuous_y",
                  sp.articulation_type == ArticulationType.CONTINUOUS
                  and tuple(sp.axis) == (0.0, 1.0, 0.0),
                  f"axis={sp.axis}")

    # Decisive pose: wheel spins in place (center fixed).
    wheel0 = object_model.get_part("caster_wheel_0")
    spin0 = object_model.get_articulation("caster_spin_0")
    rest = ctx.part_world_position(wheel0)
    with ctx.pose({spin0: 0.6}):
        turned = ctx.part_world_position(wheel0)
    if rest is not None and turned is not None:
        moved = sum((turned[k] - rest[k]) ** 2 for k in range(3)) ** 0.5
        ctx.check("wheel_spins_in_place", moved < 1e-4, f"moved={moved:.5f}")

    return ctx.report()


object_model = build_object_model()
