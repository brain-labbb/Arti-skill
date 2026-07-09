from __future__ import annotations

# Wire basket stock trolley (warehouse "basket trolley").
#
# World frame: Z up, basket floor horizontal. Long axis is X, width is Y.
# Footprint (caster wheels) touches z ~ 0.
#
# A wire mesh open-topped rectangular basket sits on a tubular chassis frame.
# Tall tubular push handle at +X end, short end guard rail at -X end.
# Four swivel casters under the chassis corners.
#
# Articulation:
#   - Each caster YAWS about vertical Z kingpin (continuous)
#   - Each wheel ROLLS about horizontal Y axle (continuous)
#
# Root = chassis. Handle, guard, and basket mesh are inline on chassis.
# Caster chain: chassis --(continuous Z)--> yoke --(continuous Y)--> wheel.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MeshGeometry,
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
DECK_THK = 0.040  # chassis frame depth

# Caster geometry
WHEEL_RADIUS = 0.055
WHEEL_WIDTH = 0.034
FORK_OFFSET = 0.034
DECK_BOTTOM_Z = 0.165
DECK_TOP_Z = DECK_BOTTOM_Z + DECK_THK
DECK_CENTER_Z = DECK_BOTTOM_Z + DECK_THK / 2.0

# Caster mount positions
CASTER_INSET_X = 0.130
CASTER_INSET_Y = 0.090
CASTER_X = DECK_LEN / 2.0 - CASTER_INSET_X
CASTER_Y = DECK_WID / 2.0 - CASTER_INSET_Y

# Tubular frame
TUBE_R = 0.011
HANDLE_X = DECK_LEN / 2.0 - 0.055
HANDLE_TOP_Z = DECK_TOP_Z + 0.93
HANDLE_HALF_W = DECK_WID / 2.0 - 0.070
HANDLE_LEAN = 0.085

GUARD_X = -(DECK_LEN / 2.0 - 0.055)
GUARD_TOP_Z = DECK_TOP_Z + 0.34
GUARD_HALF_W = HANDLE_HALF_W

# Basket dimensions
BASKET_HEIGHT = 0.32
WIRE_R = 0.002  # 2mm wire radius
WIRE_SPACING = 0.048  # ~48mm grid spacing
BASKET_INSET = 0.012  # walls inset from frame edges

# Basket extents
BX_MIN = -(DECK_LEN / 2.0 - BASKET_INSET)
BX_MAX = DECK_LEN / 2.0 - BASKET_INSET
BY_MIN = -(DECK_WID / 2.0 - BASKET_INSET)
BY_MAX = DECK_WID / 2.0 - BASKET_INSET
BZ_FLOOR = DECK_TOP_Z
BZ_TOP = DECK_TOP_Z + BASKET_HEIGHT

# Frame rail
FRAME_R = 0.020  # 40mm frame tube (structural rectangular tube)
FRAME_TUBE_Z = DECK_BOTTOM_Z + FRAME_R  # tube center so bottom = DECK_BOTTOM_Z

# Rim rail
RIM_R = 0.005  # 10mm rim tube


def _tube(points, *, radius=TUBE_R):
    return tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=10,
        radial_segments=14,
        cap_ends=True,
    )


def _straight_wire(p0, p1, *, radius=WIRE_R, radial_segments=8):
    """Build a straight wire tube segment between two 3D points."""
    return tube_from_spline_points(
        [p0, p1],
        radius=radius,
        samples_per_segment=1,
        radial_segments=radial_segments,
        cap_ends=True,
    )


def _grid_positions(start, end, spacing):
    """Generate evenly spaced positions from start to end (inclusive)."""
    n = max(int(round((end - start) / spacing)) + 1, 2)
    actual_spacing = (end - start) / (n - 1)
    return [start + i * actual_spacing for i in range(n)]


def _inner_grid_positions(start, end, spacing):
    """Generate evenly spaced interior positions, leaving the boundary for rims."""
    return _grid_positions(start, end, spacing)[1:-1]


def _build_wall_mesh(*, span_axis, normal_pos, span_min, span_max,
                     z_min, z_max, spacing, wire_r):
    """Build a wire mesh wall panel as a single merged MeshGeometry."""
    wall = MeshGeometry()
    span_positions = _grid_positions(span_min, span_max, spacing)
    z_positions = _grid_positions(z_min, z_max, spacing)

    # Horizontal wires
    for z in z_positions:
        if span_axis == "x":
            seg = _straight_wire(
                (span_min, normal_pos, z),
                (span_max, normal_pos, z),
                radius=wire_r,
            )
        else:
            seg = _straight_wire(
                (normal_pos, span_min, z),
                (normal_pos, span_max, z),
                radius=wire_r,
            )
        wall.merge(seg)

    # Vertical wires
    for p in span_positions:
        if span_axis == "x":
            seg = _straight_wire(
                (p, normal_pos, z_min),
                (p, normal_pos, z_max),
                radius=wire_r,
            )
        else:
            seg = _straight_wire(
                (normal_pos, p, z_min),
                (normal_pos, p, z_max),
                radius=wire_r,
            )
        wall.merge(seg)

    return wall


def _build_floor_mesh(*, x_min, x_max, y_min, y_max, z, spacing, wire_r):
    """Build a straight, closed-edge wire mesh floor panel."""
    floor = MeshGeometry()
    x_positions = _inner_grid_positions(x_min, x_max, spacing)
    y_positions = _inner_grid_positions(y_min, y_max, spacing)
    edge_r = wire_r * 1.35

    # Continuous perimeter first: this guarantees no visual missing edge.
    for y in (y_min, y_max):
        floor.merge(
            _straight_wire(
                (x_min, y, z),
                (x_max, y, z),
                radius=edge_r,
                radial_segments=10,
            )
        )
    for x in (x_min, x_max):
        floor.merge(
            _straight_wire(
                (x, y_min, z),
                (x, y_max, z),
                radius=edge_r,
                radial_segments=10,
            )
        )

    # X-direction wires (along X, at each interior Y)
    for y in y_positions:
        seg = _straight_wire(
            (x_min, y, z),
            (x_max, y, z),
            radius=wire_r,
        )
        floor.merge(seg)

    # Y-direction wires (along Y, at each interior X)
    for x in x_positions:
        seg = _straight_wire(
            (x, y_min, z),
            (x, y_max, z),
            radius=wire_r,
        )
        floor.merge(seg)

    return floor


def _build_rim_mesh(*, x_min, x_max, y_min, y_max, z, radius):
    """Build a rectangular rim rail (4 edge tubes) as a merged mesh."""
    rim = MeshGeometry()
    # Long edges (along X)
    for y in (y_min, y_max):
        seg = _straight_wire(
            (x_min, y, z), (x_max, y, z), radius=radius, radial_segments=10,
        )
        rim.merge(seg)
    # Short edges (along Y)
    for x in (x_min, x_max):
        seg = _straight_wire(
            (x, y_min, z), (x, y_max, z), radius=radius, radial_segments=10,
        )
        rim.merge(seg)
    return rim


def _upright_x_at_frac(base_x, top_z, base_z, lean, frac):
    """Approximate the upright centerline x at a given height fraction."""
    z = base_z + (top_z - base_z) * frac
    z_mid = base_z + (top_z - base_z) * 0.45
    if z <= z_mid:
        return base_x - 0.004 * (frac / 0.45)
    top_run_frac = (z - z_mid) / max(top_z - 0.06 - z_mid, 1e-6)
    top_run_frac = min(max(top_run_frac, 0.0), 1.0)
    return (base_x - 0.004) + (base_x + lean * 0.5 - (base_x - 0.004)) * top_run_frac


def _u_frame_mesh(*, base_x, top_z, half_w, lean, base_z, name):
    """Inverted-U tubular frame: two uprights joined by a top bend."""
    pts = [
        (base_x, -half_w, base_z),
        (base_x - 0.004, -half_w, base_z + (top_z - base_z) * 0.45),
        (base_x + lean * 0.5, -half_w, top_z - 0.06),
        (base_x + lean, -half_w * 0.96, top_z),
        (base_x + lean, 0.0, top_z + 0.012),
        (base_x + lean, half_w * 0.96, top_z),
        (base_x + lean * 0.5, half_w, top_z - 0.06),
        (base_x - 0.004, half_w, base_z + (top_z - base_z) * 0.45),
        (base_x, half_w, base_z),
    ]
    return mesh_from_geometry(_tube(pts), name)


def _cross_rail_mesh(*, x, z, half_w, name):
    reach = half_w + 0.018
    pts = [(x, -reach, z), (x, 0.0, z), (x, reach, z)]
    return mesh_from_geometry(_tube(pts, radius=TUBE_R * 0.82), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wire_basket_cart")

    frame_gray = model.material("frame_gray", rgba=(0.70, 0.71, 0.73, 1.0))
    wire_zinc = model.material("wire_zinc", rgba=(0.76, 0.77, 0.78, 1.0))
    rim_silver = model.material("rim_silver", rgba=(0.80, 0.81, 0.83, 1.0))
    rubber = model.material("caster_tread", rgba=(0.16, 0.16, 0.18, 1.0))
    steel = model.material("caster_steel", rgba=(0.55, 0.56, 0.58, 1.0))

    # ---- chassis (root) ----
    chassis = model.part("chassis")

    # Frame rails: rectangular tube chassis frame
    # Long rails (along X, at ±Y edges)
    for i, sy in enumerate((-1.0, 1.0)):
        y = sy * (DECK_WID / 2.0 - FRAME_R)
        rail_pts = [
            (-(DECK_LEN / 2.0 - FRAME_R), y, FRAME_TUBE_Z),
            (0.0, y, FRAME_TUBE_Z),
            (DECK_LEN / 2.0 - FRAME_R, y, FRAME_TUBE_Z),
        ]
        chassis.visual(
            mesh_from_geometry(_tube(rail_pts, radius=FRAME_R), f"frame_long_{i}"),
            material=frame_gray,
            name=f"frame_long_{i}",
        )
    # Short rails (along Y, at ±X edges)
    for i, sx in enumerate((-1.0, 1.0)):
        x = sx * (DECK_LEN / 2.0 - FRAME_R)
        rail_pts = [
            (x, -(DECK_WID / 2.0 - FRAME_R), FRAME_TUBE_Z),
            (x, 0.0, FRAME_TUBE_Z),
            (x, DECK_WID / 2.0 - FRAME_R, FRAME_TUBE_Z),
        ]
        chassis.visual(
            mesh_from_geometry(_tube(rail_pts, radius=FRAME_R), f"frame_short_{i}"),
            material=frame_gray,
            name=f"frame_short_{i}",
        )
    # Cross members at caster X positions (provide caster mounting support)
    for i, sx in enumerate((-1.0, 1.0)):
        cx = sx * CASTER_X
        rail_pts = [
            (cx, -(DECK_WID / 2.0 - FRAME_R), FRAME_TUBE_Z),
            (cx, 0.0, FRAME_TUBE_Z),
            (cx, DECK_WID / 2.0 - FRAME_R, FRAME_TUBE_Z),
        ]
        chassis.visual(
            mesh_from_geometry(_tube(rail_pts, radius=FRAME_R), f"frame_cross_{i}"),
            material=frame_gray,
            name=f"frame_cross_{i}",
        )

    # ---- basket wire mesh (inline on chassis) ----
    # Floor mesh
    floor_mesh = _build_floor_mesh(
        x_min=BX_MIN, x_max=BX_MAX,
        y_min=BY_MIN, y_max=BY_MAX,
        z=BZ_FLOOR, spacing=WIRE_SPACING, wire_r=WIRE_R,
    )
    chassis.visual(
        mesh_from_geometry(floor_mesh, "basket_floor"),
        material=wire_zinc,
        name="basket_floor",
    )

    # Wall meshes: 4 walls (long sides at ±Y, short sides at ±X)
    wall_configs = [
        # (span_axis, normal_pos, span_min, span_max, name)
        ("x", BY_MAX, BX_MIN, BX_MAX, "basket_wall_front"),   # +Y
        ("x", BY_MIN, BX_MIN, BX_MAX, "basket_wall_rear"),    # -Y
        ("y", BX_MAX, BY_MIN, BY_MAX, "basket_wall_right"),   # +X
        ("y", BX_MIN, BY_MIN, BY_MAX, "basket_wall_left"),    # -X
    ]
    for i, (span_axis, normal_pos, span_min, span_max, wall_name) in enumerate(wall_configs):
        wall_mesh = _build_wall_mesh(
            span_axis=span_axis,
            normal_pos=normal_pos,
            span_min=span_min,
            span_max=span_max,
            z_min=BZ_FLOOR,
            z_max=BZ_TOP,
            spacing=WIRE_SPACING,
            wire_r=WIRE_R,
        )
        chassis.visual(
            mesh_from_geometry(wall_mesh, wall_name),
            material=wire_zinc,
            name=wall_name,
        )

    # Rim rail around basket top edge
    rim_mesh = _build_rim_mesh(
        x_min=BX_MIN, x_max=BX_MAX,
        y_min=BY_MIN, y_max=BY_MAX,
        z=BZ_TOP, radius=RIM_R,
    )
    chassis.visual(
        mesh_from_geometry(rim_mesh, "basket_rim"),
        material=frame_gray,
        name="basket_rim",
    )

    chassis.inertial = Inertial.from_geometry(
        Box((DECK_LEN, DECK_WID, DECK_THK)), mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, DECK_CENTER_Z)),
    )

    # ---- tall push handle (FIXED to chassis, +X end) ----
    handle = model.part("push_handle")
    handle.visual(
        _u_frame_mesh(
            base_x=HANDLE_X, top_z=HANDLE_TOP_Z, half_w=HANDLE_HALF_W,
            lean=HANDLE_LEAN, base_z=DECK_TOP_Z, name="push_handle_loop",
        ),
        material=frame_gray,
        name="push_handle_loop",
    )
    for i, frac in enumerate((0.40, 0.70)):
        z = DECK_TOP_Z + (HANDLE_TOP_Z - DECK_TOP_Z) * frac
        x = _upright_x_at_frac(HANDLE_X, HANDLE_TOP_Z, DECK_TOP_Z, HANDLE_LEAN, frac)
        handle.visual(
            _cross_rail_mesh(x=x, z=z, half_w=HANDLE_HALF_W, name=f"handle_rail_{i}"),
            material=frame_gray,
            name=f"handle_rail_{i}",
        )
    model.articulation(
        "chassis_to_push_handle", ArticulationType.FIXED, parent=chassis, child=handle,
    )

    # ---- short end guard (FIXED to chassis, -X end) ----
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
        "chassis_to_end_guard", ArticulationType.FIXED, parent=chassis, child=guard,
    )

    # ---- four swivel casters (single for loop) ----
    leg_bottom_z = -(DECK_BOTTOM_Z - WHEEL_RADIUS)
    leg_top_z = leg_bottom_z + WHEEL_RADIUS + 0.012
    leg_half_y = WHEEL_WIDTH / 2.0 + 0.011

    caster_positions = [
        (CASTER_X, CASTER_Y),
        (CASTER_X, -CASTER_Y),
        (-CASTER_X, CASTER_Y),
        (-CASTER_X, -CASTER_Y),
    ]

    for i in range(4):
        cx, cy = caster_positions[i]

        yoke = model.part(f"caster_yoke_{i}")
        # swivel plate (slightly overlaps chassis frame tube for contact)
        yoke.visual(
            Box((0.060, 0.060, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, -0.003)),
            material=steel,
            name="swivel_plate",
        )
        # kingpin neck
        yoke.visual(
            Cylinder(radius=0.013, length=0.046),
            origin=Origin(xyz=(0.0, 0.0, -0.027)),
            material=steel,
            name="kingpin",
        )
        # offset bracket
        yoke.visual(
            Box((FORK_OFFSET + 0.028, 0.040, 0.014)),
            origin=Origin(xyz=(-FORK_OFFSET / 2.0, 0.0, -0.046)),
            material=steel,
            name="offset_bracket",
        )
        # fork crown
        yoke.visual(
            Box((0.034, leg_half_y * 2.0 + 0.014, 0.014)),
            origin=Origin(xyz=(-FORK_OFFSET, 0.0, leg_top_z)),
            material=steel,
            name="fork_crown",
        )
        # fork legs
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
        # axle bolt
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

        wheel = model.part(f"caster_wheel_{i}")
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
            mesh_from_geometry(rim_geom, f"caster_rim_{i}"),
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
            mesh_from_geometry(tire_geom, f"caster_tire_{i}"),
            material=rubber,
            name="tire",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH), mass=0.4,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        )

        # swivel: chassis -> yoke
        model.articulation(
            f"chassis_to_caster_yoke_{i}",
            ArticulationType.CONTINUOUS,
            parent=chassis,
            child=yoke,
            origin=Origin(xyz=(cx, cy, DECK_BOTTOM_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=12.0),
        )
        # roll: yoke -> wheel
        model.articulation(
            f"caster_spin_{i}",
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
    chassis = object_model.get_part("chassis")
    handle = object_model.get_part("push_handle")
    guard = object_model.get_part("end_guard")

    ctx.check("chassis_present", chassis is not None, "chassis missing")

    # Basket walls exist as named visuals on chassis
    for wall_name in ("basket_wall_front", "basket_wall_rear",
                      "basket_wall_right", "basket_wall_left"):
        v = chassis.get_visual(wall_name)
        ctx.check(
            f"basket_{wall_name}_present",
            v is not None,
            f"missing visual {wall_name}",
        )

    # Basket floor present
    floor_v = chassis.get_visual("basket_floor")
    ctx.check("basket_floor_present", floor_v is not None, "missing basket_floor visual")

    # Basket walls extend above the floor (open-topped container)
    floor_aabb = ctx.part_element_world_aabb(chassis, elem="basket_floor")
    wall_aabb = ctx.part_element_world_aabb(chassis, elem="basket_wall_front")
    if floor_aabb is not None and wall_aabb is not None:
        ctx.check(
            "basket_walls_above_floor",
            wall_aabb[1][2] > floor_aabb[1][2] + 0.20,
            f"wall_top={wall_aabb[1][2]:.3f} floor_top={floor_aabb[1][2]:.3f}",
        )

    # Axle captured in wheel hub
    for i in range(4):
        ctx.allow_overlap(
            object_model.get_part(f"caster_yoke_{i}"),
            object_model.get_part(f"caster_wheel_{i}"),
            elem_a="axle",
            elem_b="rim",
            reason="The fork axle is captured inside the wheel hub bore.",
        )

    # Chassis sits above the floor on the casters
    chassis_aabb = ctx.part_world_aabb(chassis)
    if chassis_aabb is not None:
        mins, maxs = chassis_aabb
        ctx.check(
            "chassis_above_floor",
            0.12 <= mins[2] <= 0.25,
            f"chassis bottom z={mins[2]:.3f}",
        )

    # Wheels touch floor
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

    # Handle taller than guard
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
        ctx.check(
            "handle_and_guard_opposite_ends",
            ha[1][0] > 0.2 and ga[0][0] < -0.2,
            f"handle_xmax={ha[1][0]:.3f} guard_xmin={ga[0][0]:.3f}",
        )

    # Joint inventory: 8 caster joints
    for i in range(4):
        sw = object_model.get_articulation(f"chassis_to_caster_yoke_{i}")
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

    # Wheel spins in place
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

    # Swivel keeps wheel on floor
    swivel0 = object_model.get_articulation("chassis_to_caster_yoke_0")
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
