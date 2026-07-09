from __future__ import annotations

# Stainless-steel wire-mesh shopping basket with hinged drop-front gate.
# Open mesh walls and floor made from a grid of thin metal wires (vertical
# and horizontal rods), reinforced thick-wire top rim and corner posts,
# a hinged front gate panel that drops open on a bottom-edge revolute joint,
# and a single central swing bail handle.
#
# Coordinate convention: +Z up, basket rests on the ground at z=0, long axis
# along X (wider in X than in Y). Front gate on +Y side.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Dimensions (meters)
# ---------------------------------------------------------------------------
BODY_H = 0.300  # tub height (z) — noticeably deeper basket

# Outer footprint, bottom (slightly narrower -> stackable taper).
BOT_X = 0.410
BOT_Y = 0.260
# Outer footprint, top mouth (slightly wider).
TOP_X = 0.450
TOP_Y = 0.300

# Wire radii.
WIRE_R_FRAME = 0.0030   # thick frame wires (rim, corners, bottom)
WIRE_R_MESH = 0.0013    # thin mesh grid wires
WIRE_R_FLOOR = 0.0018   # floor mesh wires (slightly thicker for support)

# Mesh spacing.
WALL_H_SPACING = 0.028   # horizontal wire spacing on walls (vertical direction)
WALL_V_SPACING = 0.030   # vertical wire spacing on walls (along wall run)
FLOOR_SPACING = 0.028    # floor grid spacing

# Grip ears on the two short ends (wire loop extensions at +X / -X rim).
EAR_X = 0.022  # how far the ear sticks out past the rim in X
EAR_Y = 0.100  # ear width in Y
EAR_Z = 0.028  # ear height in Z

# Single central swing bail handle.
BAIL_HALF_SPAN = TOP_X / 2.0  # legs attach at short-end rim wires
BAIL_HEIGHT = 0.220            # arch height above rim
BAIL_WIRE_R = 0.0040      # handle leg wire radius
BAIL_GRIP_R = 0.0060      # grip bar radius (thicker for comfort)
KNUCKLE_R = 0.010         # pivot knuckle radius at leg bases

STEEL = (0.76, 0.76, 0.79, 1.0)


# ---------------------------------------------------------------------------
# Wire segment helper
# ---------------------------------------------------------------------------
def _wire_segment(p0, p1, radius, radial_segments=8):
    """Build a cylinder mesh from point p0 to point p1."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length < 1e-6:
        return None

    cyl = CylinderGeometry(radius, length, radial_segments=radial_segments, closed=False)

    # Cylinder is along local Z. Rotate to align with direction (dx, dy, dz).
    dir_x, dir_y, dir_z = dx / length, dy / length, dz / length

    # Cross product of (0,0,1) with direction = (-dir_y, dir_x, 0)
    cross_x = -dir_y
    cross_y = dir_x
    cross_len = math.sqrt(cross_x ** 2 + cross_y ** 2)

    if cross_len < 1e-6:
        # Direction is along +Z or -Z
        if dir_z < 0:
            cyl.rotate((1.0, 0.0, 0.0), math.pi)
    else:
        angle = math.acos(max(-1.0, min(1.0, dir_z)))
        cyl.rotate((cross_x / cross_len, cross_y / cross_len, 0.0), angle)

    # Translate to midpoint
    mx = (p0[0] + p1[0]) / 2.0
    my = (p0[1] + p1[1]) / 2.0
    mz = (p0[2] + p1[2]) / 2.0
    cyl.translate(mx, my, mz)

    return cyl


def _wall_extent_x(z):
    """Half-width in X at height z (interpolated for taper)."""
    t = z / BODY_H
    return (BOT_X + (TOP_X - BOT_X) * t) / 2.0


def _wall_extent_y(z):
    """Half-width in Y at height z (interpolated for taper)."""
    t = z / BODY_H
    return (BOT_Y + (TOP_Y - BOT_Y) * t) / 2.0


# ---------------------------------------------------------------------------
# Wire mesh basket body (3 walls + floor; front +Y wall is the gate)
# ---------------------------------------------------------------------------
def _build_wire_basket():
    """Build the wire-mesh basket body as a merged procedural mesh.

    Structure:
    - Thick wire frame: top rim (3 sides, no +Y front), bottom rectangle (all 4),
      4 corner posts, intermediate vertical stiffeners on 3 walls.
    - Thin wire mesh: horizontal and vertical grid wires on -Y and ±X walls.
    - Floor grid: X and Y direction wires at the bottom.
    - Grip ear wire loops on the two short ends.
    """
    mesh = MeshGeometry()

    top_hx = TOP_X / 2.0
    top_hy = TOP_Y / 2.0
    bot_hx = BOT_X / 2.0
    bot_hy = BOT_Y / 2.0
    z_top = BODY_H
    z_floor = WIRE_R_FRAME * 2.0

    # --- Top rim: 3 sides (skip +Y front — that rim wire belongs to gate) ---
    for p0, p1 in [
        ((-top_hx, -top_hy, z_top), (top_hx, -top_hy, z_top)),   # -Y back
        ((top_hx, -top_hy, z_top), (top_hx, top_hy, z_top)),     # +X right
        ((-top_hx, top_hy, z_top), (-top_hx, -top_hy, z_top)),   # -X left
    ]:
        w = _wire_segment(p0, p1, WIRE_R_FRAME, radial_segments=10)
        if w:
            mesh.merge(w)

    # Bottom frame segments (all 4 sides — +Y bottom is the hinge anchor)
    for p0, p1 in [
        ((-bot_hx, -bot_hy, z_floor), (bot_hx, -bot_hy, z_floor)),
        ((bot_hx, -bot_hy, z_floor), (bot_hx, bot_hy, z_floor)),
        ((bot_hx, bot_hy, z_floor), (-bot_hx, bot_hy, z_floor)),
        ((-bot_hx, bot_hy, z_floor), (-bot_hx, -bot_hy, z_floor)),
    ]:
        w = _wire_segment(p0, p1, WIRE_R_FRAME, radial_segments=10)
        if w:
            mesh.merge(w)

    # Corner posts (thick wire, with taper)
    for sx in (1.0, -1.0):
        for sy in (1.0, -1.0):
            p0 = (sx * bot_hx, sy * bot_hy, z_floor)
            p1 = (sx * top_hx, sy * top_hy, z_top)
            w = _wire_segment(p0, p1, WIRE_R_FRAME, radial_segments=10)
            if w:
                mesh.merge(w)

    # --- Intermediate vertical stiffeners ---
    # Back wall (-Y): 3 intermediate verticals
    n_vert_long = 3
    for i in range(1, n_vert_long + 1):
        frac = i / (n_vert_long + 1)
        x_bot = -bot_hx + frac * BOT_X
        x_top = -top_hx + frac * TOP_X
        # Only -Y wall (back)
        p0 = (x_bot, -bot_hy, z_floor)
        p1 = (x_top, -top_hy, z_top)
        w = _wire_segment(p0, p1, WIRE_R_FRAME * 0.7, radial_segments=8)
        if w:
            mesh.merge(w)

    # Short walls (±X): 1 intermediate vertical per wall
    frac = 0.5
    y_bot_mid = -bot_hy + frac * BOT_Y
    y_top_mid = -top_hy + frac * TOP_Y
    for sx in (1.0, -1.0):
        p0 = (sx * bot_hx, y_bot_mid, z_floor)
        p1 = (sx * top_hx, y_top_mid, z_top)
        w = _wire_segment(p0, p1, WIRE_R_FRAME * 0.7, radial_segments=8)
        if w:
            mesh.merge(w)

    # --- Wall mesh: horizontal wires (only on -Y and ±X walls) ---
    z_positions = []
    z = WALL_H_SPACING
    while z < BODY_H - WALL_H_SPACING * 0.5:
        z_positions.append(z)
        z += WALL_H_SPACING

    for zz in z_positions:
        hx = _wall_extent_x(zz)
        hy = _wall_extent_y(zz)
        # Back wall (-Y): wire runs along X
        p0 = (-hx, -hy, zz)
        p1 = (hx, -hy, zz)
        w = _wire_segment(p0, p1, WIRE_R_MESH, radial_segments=6)
        if w:
            mesh.merge(w)
        # Short walls (±X): wire runs along Y
        for sx in (1.0, -1.0):
            p0 = (sx * hx, -hy, zz)
            p1 = (sx * hx, hy, zz)
            w = _wire_segment(p0, p1, WIRE_R_MESH, radial_segments=6)
            if w:
                mesh.merge(w)

    # --- Wall mesh: vertical wires (only on -Y and ±X walls) ---
    # Back wall (-Y): vertical wires spaced along X
    n_v_long = max(2, int(BOT_X / WALL_V_SPACING) - 1)
    for i in range(n_v_long + 1):
        frac = i / n_v_long
        x_bot = -bot_hx + frac * BOT_X
        x_top = -top_hx + frac * TOP_X
        p0 = (x_bot, -bot_hy, z_floor)
        p1 = (x_top, -top_hy, z_top)
        w = _wire_segment(p0, p1, WIRE_R_MESH, radial_segments=6)
        if w:
            mesh.merge(w)

    # Short walls (±X): vertical wires spaced along Y
    n_v_short = max(2, int(BOT_Y / WALL_V_SPACING) - 1)
    for j in range(n_v_short + 1):
        frac = j / n_v_short
        y_bot = -bot_hy + frac * BOT_Y
        y_top = -top_hy + frac * TOP_Y
        for sx in (1.0, -1.0):
            p0 = (sx * bot_hx, y_bot, z_floor)
            p1 = (sx * top_hx, y_top, z_top)
            w = _wire_segment(p0, p1, WIRE_R_MESH, radial_segments=6)
            if w:
                mesh.merge(w)

    # --- Floor mesh: grid of wires at z_floor ---
    # X-direction wires
    n_floor_y = max(2, int(BOT_Y / FLOOR_SPACING) - 1)
    for j in range(n_floor_y + 1):
        frac = j / n_floor_y
        y = -bot_hy + frac * BOT_Y
        p0 = (-bot_hx, y, z_floor)
        p1 = (bot_hx, y, z_floor)
        w = _wire_segment(p0, p1, WIRE_R_FLOOR, radial_segments=6)
        if w:
            mesh.merge(w)

    # Y-direction wires
    n_floor_x = max(2, int(BOT_X / FLOOR_SPACING) - 1)
    for i in range(n_floor_x + 1):
        frac = i / n_floor_x
        x = -bot_hx + frac * BOT_X
        p0 = (x, -bot_hy, z_floor)
        p1 = (x, bot_hy, z_floor)
        w = _wire_segment(p0, p1, WIRE_R_FLOOR, radial_segments=6)
        if w:
            mesh.merge(w)

    # --- Grip ears: wire loops on short ends (±X) ---
    for sx in (1.0, -1.0):
        ear_cx = sx * (top_hx + EAR_X * 0.5)
        ear_cz = z_top - EAR_Z / 2.0
        ex = EAR_X
        ey = EAR_Y / 2.0
        ez_top = z_top
        ez_bot = z_top - EAR_Z
        rim_x = sx * top_hx

        # Top wire of ear
        pts = [
            (rim_x, -ey, ez_top),
            (rim_x + sx * ex, -ey, ez_top),
            (rim_x + sx * ex, ey, ez_top),
            (rim_x, ey, ez_top),
        ]
        for k in range(len(pts) - 1):
            w = _wire_segment(pts[k], pts[k + 1], WIRE_R_FRAME, radial_segments=8)
            if w:
                mesh.merge(w)

        # Bottom wire of ear
        pts_bot = [
            (rim_x, -ey, ez_bot),
            (rim_x + sx * ex, -ey, ez_bot),
            (rim_x + sx * ex, ey, ez_bot),
            (rim_x, ey, ez_bot),
        ]
        for k in range(len(pts_bot) - 1):
            w = _wire_segment(pts_bot[k], pts_bot[k + 1], WIRE_R_FRAME, radial_segments=8)
            if w:
                mesh.merge(w)

        # Vertical connecting wires at the outer corners
        for sy in (1.0, -1.0):
            p0 = (rim_x + sx * ex, sy * ey, ez_bot)
            p1 = (rim_x + sx * ex, sy * ey, ez_top)
            w = _wire_segment(p0, p1, WIRE_R_FRAME, radial_segments=8)
            if w:
                mesh.merge(w)

    return mesh


# ---------------------------------------------------------------------------
# Gate panel (+Y front wall, hinged at bottom edge)
# ---------------------------------------------------------------------------
def _build_gate_panel():
    """Build the front gate panel (+Y wall) as a wire mesh panel.

    Authored in gate-local frame whose origin sits at the hinge line center
    (0, 0, 0). The gate extends in local +Z (upward when closed).
    """
    mesh = MeshGeometry()

    z_floor = WIRE_R_FRAME * 2.0
    gate_height = BODY_H - z_floor
    bot_hy = BOT_Y / 2.0
    bot_hx = BOT_X / 2.0
    top_hx = TOP_X / 2.0
    top_y_local = TOP_Y / 2.0 - bot_hy  # Y offset of top edge due to taper

    # Top rim wire (front gate has its own top rim)
    w = _wire_segment(
        (-top_hx, top_y_local, gate_height),
        (top_hx, top_y_local, gate_height),
        WIRE_R_FRAME, radial_segments=10,
    )
    if w:
        mesh.merge(w)

    # Bottom edge wire (hinge rod on gate side)
    w = _wire_segment(
        (-bot_hx, 0.0, 0.0),
        (bot_hx, 0.0, 0.0),
        WIRE_R_FRAME, radial_segments=10,
    )
    if w:
        mesh.merge(w)

    # Horizontal mesh wires
    z_positions = []
    z = WALL_H_SPACING
    while z < BODY_H - WALL_H_SPACING * 0.5:
        z_positions.append(z)
        z += WALL_H_SPACING

    for zz in z_positions:
        hx = _wall_extent_x(zz)
        hy = _wall_extent_y(zz)
        z_local = zz - z_floor
        y_local = hy - bot_hy
        p0 = (-hx, y_local, z_local)
        p1 = (hx, y_local, z_local)
        w = _wire_segment(p0, p1, WIRE_R_MESH, radial_segments=6)
        if w:
            mesh.merge(w)

    # Vertical mesh wires
    n_v_long = max(2, int(BOT_X / WALL_V_SPACING) - 1)
    for i in range(n_v_long + 1):
        frac = i / n_v_long
        x_bot = -bot_hx + frac * BOT_X
        x_top = -top_hx + frac * TOP_X
        p0 = (x_bot, 0.0, 0.0)
        p1 = (x_top, top_y_local, gate_height)
        w = _wire_segment(p0, p1, WIRE_R_MESH, radial_segments=6)
        if w:
            mesh.merge(w)

    # Intermediate vertical stiffeners (3 for long wall)
    n_vert_long = 3
    for i in range(1, n_vert_long + 1):
        frac = i / (n_vert_long + 1)
        x_bot = -bot_hx + frac * BOT_X
        x_top = -top_hx + frac * TOP_X
        p0 = (x_bot, 0.0, 0.0)
        p1 = (x_top, top_y_local, gate_height)
        w = _wire_segment(p0, p1, WIRE_R_FRAME * 0.7, radial_segments=8)
        if w:
            mesh.merge(w)

    return mesh


# ---------------------------------------------------------------------------
# Single central swing bail handle
# ---------------------------------------------------------------------------
def _build_bail_handle():
    """Build a single central swing bail handle as wire segments.

    Authored in handle-local frame whose origin sits at the pivot line center
    (0, 0, 0). The handle extends in local +Z (upward when standing).
    Two legs at ±BAIL_HALF_SPAN, connected by a thick grip bar at the top.
    """
    mesh = MeshGeometry()

    half_span = BAIL_HALF_SPAN

    # Left leg
    w = _wire_segment(
        (-half_span, 0.0, 0.0),
        (-half_span, 0.0, BAIL_HEIGHT),
        BAIL_WIRE_R, radial_segments=8,
    )
    if w:
        mesh.merge(w)

    # Right leg
    w = _wire_segment(
        (half_span, 0.0, 0.0),
        (half_span, 0.0, BAIL_HEIGHT),
        BAIL_WIRE_R, radial_segments=8,
    )
    if w:
        mesh.merge(w)

    # Top grip bar (thicker for comfort)
    w = _wire_segment(
        (-half_span, 0.0, BAIL_HEIGHT),
        (half_span, 0.0, BAIL_HEIGHT),
        BAIL_GRIP_R, radial_segments=10,
    )
    if w:
        mesh.merge(w)

    # Curved shoulder wires connecting legs to grip (smooth transition)
    for sx in (-1.0, 1.0):
        # Small diagonal brace from top of leg to grip bar
        w = _wire_segment(
            (sx * half_span, 0.0, BAIL_HEIGHT - 0.020),
            (sx * half_span, 0.0, BAIL_HEIGHT),
            BAIL_WIRE_R * 1.2, radial_segments=8,
        )
        if w:
            mesh.merge(w)

    # Pivot knuckles at the leg bases (cylinders along X axis)
    for sx in (-1.0, 1.0):
        knuckle = CylinderGeometry(KNUCKLE_R, 0.022, radial_segments=10, closed=False)
        # Knuckle is along local Z; rotate to align with X axis
        knuckle.rotate((0.0, 1.0, 0.0), math.pi / 2.0)
        knuckle.translate(sx * half_span, 0.0, 0.0)
        mesh.merge(knuckle)

    return mesh


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wire_mesh_shopping_basket")

    steel = model.material("brushed_steel", rgba=STEEL)

    # Root: the wire-mesh basket body (3 fixed walls + floor; no front wall).
    basket = model.part("basket_tub")
    basket_wire_mesh = _build_wire_basket()
    basket.visual(
        mesh_from_geometry(basket_wire_mesh, "basket_wire_mesh"),
        material=steel,
    )
    basket.inertial = Inertial.from_geometry(
        Box((TOP_X, TOP_Y, BODY_H)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # Front gate panel (+Y wall, hinged at bottom edge).
    gate = model.part("gate_panel")
    gate_mesh = _build_gate_panel()
    gate.visual(
        mesh_from_geometry(gate_mesh, "gate_wire_mesh"),
        material=steel,
    )
    gate.inertial = Inertial.from_geometry(
        Box((TOP_X, 0.02, BODY_H)),
        mass=0.15,
        origin=Origin(xyz=(0.0, 0.01, BODY_H / 2.0)),
    )

    z_floor = WIRE_R_FRAME * 2.0
    bot_hy = BOT_Y / 2.0

    model.articulation(
        "tub_to_gate",
        ArticulationType.REVOLUTE,
        parent=basket,
        child=gate,
        # Hinge line at bottom edge of front wall, center X.
        origin=Origin(xyz=(0.0, bot_hy, z_floor)),
        # axis=(-1,0,0): right-hand rule makes positive q rotate local +Z
        # toward world +Y, dropping the gate top outward (down and away).
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(85.0),
        ),
    )

    # Single central swing bail handle.
    handle = model.part("bail_handle")
    handle_mesh = _build_bail_handle()
    handle.visual(
        mesh_from_geometry(handle_mesh, "bail_wire_mesh"),
        material=steel,
    )
    handle.inertial = Inertial.from_geometry(
        Box((BAIL_HALF_SPAN * 2, 0.02, BAIL_HEIGHT)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, BAIL_HEIGHT / 2.0)),
    )

    model.articulation(
        "tub_to_handle",
        ArticulationType.REVOLUTE,
        parent=basket,
        child=handle,
        # Pivot line at rim height, center of basket, along X.
        origin=Origin(xyz=(0.0, 0.0, BODY_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=-math.radians(10.0),
            upper=math.radians(170.0),
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    basket = object_model.get_part("basket_tub")
    gate = object_model.get_part("gate_panel")
    handle = object_model.get_part("bail_handle")
    gate_joint = object_model.get_articulation("tub_to_gate")
    handle_joint = object_model.get_articulation("tub_to_handle")

    # --- Footprint: wider in X than Y, rests at z ~ 0. -----------------------
    lo, hi = ctx.part_world_aabb(basket)
    width_x = hi[0] - lo[0]
    depth_y = hi[1] - lo[1]
    height_z = hi[2] - lo[2]
    ctx.check(
        "footprint wider in X than Y",
        width_x > depth_y + 0.05,
        details=f"x={width_x:.3f}, y={depth_y:.3f}",
    )
    ctx.check(
        "basket rests at z~0",
        abs(lo[2]) < 0.01,
        details=f"min_z={lo[2]:.4f}",
    )
    ctx.check(
        "basket is noticeably deep (height > 0.28m)",
        height_z > 0.28,
        details=f"height={height_z:.3f}",
    )

    # --- Wire mesh: the basket is an open wire frame. -------------------------
    ctx.check(
        "tub spans full outer mouth in XY",
        width_x > TOP_X - 0.01 and depth_y > TOP_Y - 0.01,
        details=f"x={width_x:.3f}, y={depth_y:.3f}",
    )

    # --- Gate panel exists and is on the +Y (front) side. --------------------
    gate_lo, gate_hi = ctx.part_world_aabb(gate)
    ctx.check(
        "gate panel on +Y front side",
        gate_lo[1] > -0.02,
        details=f"gate min_y={gate_lo[1]:.3f}",
    )

    # --- Gate drops open with positive q. ------------------------------------
    with ctx.pose({gate_joint: 0.0}):
        closed_lo, closed_hi = ctx.part_world_aabb(gate)
        closed_top = closed_hi[2]
        closed_front_y = closed_hi[1]
    with ctx.pose({gate_joint: math.radians(60.0)}):
        open_lo, open_hi = ctx.part_world_aabb(gate)
        open_top = open_hi[2]
        open_front_y = open_hi[1]
    ctx.check(
        "gate top drops when opened",
        open_top < closed_top - 0.05,
        details=f"closed_top={closed_top:.3f}, open_top={open_top:.3f}",
    )
    ctx.check(
        "gate swings outward (+Y) when opened",
        open_front_y > closed_front_y - 0.01,
        details=f"closed_y={closed_front_y:.3f}, open_y={open_front_y:.3f}",
    )

    # --- Bail handle stands above basket rim at rest. ------------------------
    handle_lo, handle_hi = ctx.part_world_aabb(handle)
    ctx.check(
        "bail handle rises above basket rim",
        handle_hi[2] > BODY_H + 0.10,
        details=f"handle_top={handle_hi[2]:.3f}, rim_z={BODY_H:.3f}",
    )

    # --- Handle swings between poses. ----------------------------------------
    with ctx.pose({handle_joint: 0.0}):
        up_lo, up_hi = ctx.part_world_aabb(handle)
        up_top = up_hi[2]
    with ctx.pose({handle_joint: math.radians(90.0)}):
        swing_lo, swing_hi = ctx.part_world_aabb(handle)
        swing_top = swing_hi[2]
    ctx.check(
        "handle arch drops when swung to 90°",
        swing_top < up_top - 0.08,
        details=f"up_top={up_top:.3f}, swing_top={swing_top:.3f}",
    )

    # --- Handle is centered (single handle, not two). ------------------------
    handle_center_y = 0.5 * (handle_lo[1] + handle_hi[1])
    ctx.check(
        "bail handle centered in Y",
        abs(handle_center_y) < 0.05,
        details=f"handle_center_y={handle_center_y:.3f}",
    )

    # --- Intentional overlaps. -----------------------------------------------
    # Handle pivot knuckles are captured at the rim pivot points.
    ctx.allow_overlap(
        basket,
        handle,
        reason="Bail handle pivot knuckles are intentionally captured at the rim pivot points.",
    )

    # Gate hinge connection: gate bottom mesh wires meet basket bottom frame at hinge line.
    ctx.allow_overlap(
        basket,
        gate,
        reason="Gate bottom edge wires and hinge rod contact the basket bottom frame wire at the hinge line; "
               "gate vertical wires also meet the corner posts when closed.",
    )

    return ctx.report()


object_model = build_object_model()
