from __future__ import annotations

# Stainless-steel wire-mesh hand-held shopping basket.
# Open mesh walls and floor made from a grid of thin metal wires (vertical
# and horizontal rods), a reinforced thick-wire top rim, thick-wire bottom
# frame and corner posts, a slightly tapered stackable body, molded grip ears
# on the short ends (wire loops), and two black folding carry handles that
# each pivot up from the long rims and fold down flat into the basket.
#
# Coordinate convention: +Z up, basket rests on the ground at z=0, long axis
# along X (wider in X than in Y).

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
BODY_H = 0.230  # tub height (z)

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

# Handles. Each handle is a tall inverted-U arch whose two feet both attach to
# ONE long rim. The two feet are spaced apart ALONG the rim (along X); the arch
# rises in Z. The handle pivots about the rim line (X axis) to fold flat.
HANDLE_ARCH_H = 0.215
HANDLE_FOOT_SPAN_X = 0.190
HANDLE_STRAP_W = 0.024
HANDLE_STRAP_T = 0.007
KNUCKLE_R = 0.012
PIVOT_Y = TOP_Y / 2.0 - 0.006
PIVOT_Z = BODY_H - 0.012

STEEL = (0.76, 0.76, 0.79, 1.0)
BLACK = (0.07, 0.07, 0.08, 1.0)


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
# Wire mesh basket body
# ---------------------------------------------------------------------------
def _build_wire_basket():
    """Build the wire-mesh basket body as a merged procedural mesh.

    Structure:
    - Thick wire frame: top rim rectangle, bottom rectangle, 4 corner posts,
      intermediate vertical stiffeners on each wall.
    - Thin wire mesh: horizontal and vertical grid wires on all 4 walls.
    - Floor grid: X and Y direction wires at the bottom.
    - Grip ear wire loops on the two short ends.
    """
    mesh = MeshGeometry()

    # --- Frame: top rim (thick wire rectangle at z = BODY_H) ---
    top_hx = TOP_X / 2.0
    top_hy = TOP_Y / 2.0
    bot_hx = BOT_X / 2.0
    bot_hy = BOT_Y / 2.0
    z_top = BODY_H
    z_bot = 0.0

    # Top rim segments
    for p0, p1 in [
        ((-top_hx, -top_hy, z_top), (top_hx, -top_hy, z_top)),
        ((top_hx, -top_hy, z_top), (top_hx, top_hy, z_top)),
        ((top_hx, top_hy, z_top), (-top_hx, top_hy, z_top)),
        ((-top_hx, top_hy, z_top), (-top_hx, -top_hy, z_top)),
    ]:
        w = _wire_segment(p0, p1, WIRE_R_FRAME, radial_segments=10)
        if w:
            mesh.merge(w)

    # Bottom frame segments
    z_floor = WIRE_R_FRAME * 2.0  # slightly above ground for floor wires
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

    # --- Intermediate vertical stiffeners on each wall (frame gauge) ---
    # Long walls: 3 intermediate verticals per wall
    n_vert_long = 3
    for i in range(1, n_vert_long + 1):
        frac = i / (n_vert_long + 1)
        x_bot = -bot_hx + frac * BOT_X
        x_top = -top_hx + frac * TOP_X
        for sy in (1.0, -1.0):
            p0 = (x_bot, sy * bot_hy, z_floor)
            p1 = (x_top, sy * top_hy, z_top)
            w = _wire_segment(p0, p1, WIRE_R_FRAME * 0.7, radial_segments=8)
            if w:
                mesh.merge(w)

    # Short walls: 1 intermediate vertical per wall
    frac = 0.5
    y_bot_mid = -bot_hy + frac * BOT_Y
    y_top_mid = -top_hy + frac * TOP_Y
    for sx in (1.0, -1.0):
        p0 = (sx * bot_hx, y_bot_mid, z_floor)
        p1 = (sx * top_hx, y_top_mid, z_top)
        w = _wire_segment(p0, p1, WIRE_R_FRAME * 0.7, radial_segments=8)
        if w:
            mesh.merge(w)

    # --- Wall mesh: horizontal wires ---
    # Horizontal wires at various Z heights on all 4 walls.
    z_positions = []
    z = WALL_H_SPACING
    while z < BODY_H - WALL_H_SPACING * 0.5:
        z_positions.append(z)
        z += WALL_H_SPACING

    for zz in z_positions:
        hx = _wall_extent_x(zz)
        hy = _wall_extent_y(zz)
        # Long walls (front/back, at ±Y): wire runs along X
        for sy in (1.0, -1.0):
            p0 = (-hx, sy * hy, zz)
            p1 = (hx, sy * hy, zz)
            w = _wire_segment(p0, p1, WIRE_R_MESH, radial_segments=6)
            if w:
                mesh.merge(w)
        # Short walls (left/right, at ±X): wire runs along Y
        for sx in (1.0, -1.0):
            p0 = (sx * hx, -hy, zz)
            p1 = (sx * hx, hy, zz)
            w = _wire_segment(p0, p1, WIRE_R_MESH, radial_segments=6)
            if w:
                mesh.merge(w)

    # --- Wall mesh: vertical wires ---
    # Long walls: vertical wires spaced along X
    n_v_long = max(2, int(BOT_X / WALL_V_SPACING) - 1)
    for i in range(n_v_long + 1):
        frac = i / n_v_long
        x_bot = -bot_hx + frac * BOT_X
        x_top = -top_hx + frac * TOP_X
        for sy in (1.0, -1.0):
            p0 = (x_bot, sy * bot_hy, z_floor)
            p1 = (x_top, sy * top_hy, z_top)
            w = _wire_segment(p0, p1, WIRE_R_MESH, radial_segments=6)
            if w:
                mesh.merge(w)

    # Short walls: vertical wires spaced along Y
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
    # X-direction wires (running along X at various Y positions)
    n_floor_y = max(2, int(BOT_Y / FLOOR_SPACING) - 1)
    for j in range(n_floor_y + 1):
        frac = j / n_floor_y
        y = -bot_hy + frac * BOT_Y
        p0 = (-bot_hx, y, z_floor)
        p1 = (bot_hx, y, z_floor)
        w = _wire_segment(p0, p1, WIRE_R_FLOOR, radial_segments=6)
        if w:
            mesh.merge(w)

    # Y-direction wires (running along Y at various X positions)
    n_floor_x = max(2, int(BOT_X / FLOOR_SPACING) - 1)
    for i in range(n_floor_x + 1):
        frac = i / n_floor_x
        x = -bot_hx + frac * BOT_X
        p0 = (x, -bot_hy, z_floor)
        p1 = (x, bot_hy, z_floor)
        w = _wire_segment(p0, p1, WIRE_R_FLOOR, radial_segments=6)
        if w:
            mesh.merge(w)

    # --- Grip ears: wire loops on short ends (+X and -X) ---
    for sx in (1.0, -1.0):
        ear_cx = sx * (top_hx + EAR_X * 0.5)
        ear_cz = z_top - EAR_Z / 2.0
        # Build a rounded rectangular loop from 4 wire segments
        # The ear is a wire loop protruding outward from the rim
        ex = EAR_X
        ey = EAR_Y / 2.0
        ez_top = z_top
        ez_bot = z_top - EAR_Z
        rim_x = sx * top_hx

        # Top wire of ear (at rim height, extending outward)
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
# Handle (kept from original — flat black strap bent into inverted-U arch)
# ---------------------------------------------------------------------------
def _build_handle():
    """A flat black strap bent into a tall inverted-U arch.

    Authored in the handle-local frame whose origin sits on the rim pivot line.
    Both feet attach to ONE long rim, spaced apart along the rim (local X). The
    arch rises in +Z. At q=0 the arch stands vertical (carry pose); the joint
    pivots it about local X to fold flat.
    """
    foot_half = HANDLE_FOOT_SPAN_X / 2.0
    half_w = HANDLE_STRAP_W / 2.0

    n = 48
    centerline = []
    for i in range(n + 1):
        t = i / n
        x = foot_half - 2.0 * foot_half * t
        z = HANDLE_ARCH_H * math.sin(math.pi * t) ** 0.40
        centerline.append((x, z))

    def normal_at(i: int) -> tuple[float, float]:
        i0 = max(0, i - 1)
        i1 = min(n, i + 1)
        dx = centerline[i1][0] - centerline[i0][0]
        dz = centerline[i1][1] - centerline[i0][1]
        L = math.hypot(dx, dz) or 1.0
        return (-dz / L, dx / L)

    outer = []
    inner = []
    for i, (x, z) in enumerate(centerline):
        nx, nz = normal_at(i)
        outer.append((x + nx * half_w, z + nz * half_w))
        inner.append((x - nx * half_w, z - nz * half_w))

    profile_pts = outer + list(reversed(inner))
    band = (
        cq.Workplane("XZ")
        .polyline(profile_pts)
        .close()
        .extrude(HANDLE_STRAP_T / 2.0, both=True)
    )

    handle = band
    for sx in (1.0, -1.0):
        knuckle = (
            cq.Workplane("YZ")
            .circle(KNUCKLE_R)
            .extrude(0.012, both=True)
            .translate((sx * foot_half, 0.0, 0.0))
        )
        handle = handle.union(knuckle)

    return handle


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wire_mesh_shopping_basket")

    steel = model.material("brushed_steel", rgba=STEEL)
    black = model.material("handle_black", rgba=BLACK)

    # Root: the wire-mesh basket body.
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

    # Two folding handles.
    handle_mesh = _build_handle()
    for idx, sy in enumerate((1.0, -1.0)):
        name = f"handle_{idx}"
        handle = model.part(name)
        handle.visual(mesh_from_cadquery(handle_mesh, name), material=black)
        handle.inertial = Inertial.from_geometry(
            Box((HANDLE_FOOT_SPAN_X, HANDLE_STRAP_T, HANDLE_ARCH_H)),
            mass=0.05,
            origin=Origin(xyz=(0.0, 0.0, HANDLE_ARCH_H / 2.0)),
        )

        mount_origin = Origin(xyz=(0.0, sy * PIVOT_Y, PIVOT_Z))

        model.articulation(
            f"tub_to_handle_{idx}",
            ArticulationType.REVOLUTE,
            parent=basket,
            child=handle,
            origin=mount_origin,
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=2.0,
                lower=-math.radians(100.0),
                upper=math.radians(100.0),
            ),
        )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    basket = object_model.get_part("basket_tub")
    handle_0 = object_model.get_part("handle_0")
    handle_1 = object_model.get_part("handle_1")
    j0 = object_model.get_articulation("tub_to_handle_0")
    j1 = object_model.get_articulation("tub_to_handle_1")

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
        "basket realistic height",
        0.18 < height_z < 0.30,
        details=f"height={height_z:.3f}",
    )

    # --- Wire mesh: the basket is an open wire frame, not a solid block. -----
    ctx.check(
        "tub spans full outer mouth in XY",
        width_x > TOP_X - 0.01 and depth_y > TOP_Y - 0.01,
        details=f"x={width_x:.3f}, y={depth_y:.3f}",
    )

    # --- Two distinct handles on the two long rims (offset in +/-Y). ---------
    p0 = ctx.part_world_position(handle_0)
    p1 = ctx.part_world_position(handle_1)
    ctx.check(
        "handles mounted on opposite long rims (+/-Y)",
        p0 is not None and p1 is not None and (p0[1] > 0.0 > p1[1]),
        details=f"handle_0 y={p0[1]:.3f}, handle_1 y={p1[1]:.3f}",
    )
    ctx.expect_origin_distance(
        handle_0,
        handle_1,
        axes="y",
        min_dist=0.10,
        name="handles separated across Y",
    )

    # --- Handle pivots: swings the top arch in Z/Y between poses. ------------
    for handle, joint, name in ((handle_0, j0, "handle_0"), (handle_1, j1, "handle_1")):
        with ctx.pose({joint: 0.0}):
            up_lo, up_hi = ctx.part_world_aabb(handle)
            up_top = up_hi[2]
        with ctx.pose({joint: math.radians(95.0)}):
            fold_lo, fold_hi = ctx.part_world_aabb(handle)
            fold_top = fold_hi[2]
        ctx.check(
            f"{name} arch drops when folded",
            fold_top < up_top - 0.08,
            details=f"up_top={up_top:.3f}, fold_top={fold_top:.3f}",
        )
        up_cy = 0.5 * (up_lo[1] + up_hi[1])
        fold_cy = 0.5 * (fold_lo[1] + fold_hi[1])
        ctx.check(
            f"{name} swings in Y when folded",
            abs(fold_cy - up_cy) > 0.03,
            details=f"up_cy={up_cy:.3f}, fold_cy={fold_cy:.3f}",
        )

    # --- Handle knuckles are captured at the rim (intentional local overlap).
    ctx.allow_overlap(
        basket,
        handle_0,
        reason="handle_0 pivot knuckles are intentionally captured inside the long rim at the pivot line.",
    )
    ctx.allow_overlap(
        basket,
        handle_1,
        reason="handle_1 pivot knuckles are intentionally captured inside the long rim at the pivot line.",
    )

    return ctx.report()


object_model = build_object_model()
