from __future__ import annotations

# Hexagonal blue plastic shopping basket with open diagonal cross-lattice
# (X-pattern) walls, a reinforced rolled top rim, molded grip ears at opposite
# vertices, a slightly tapered stackable body, and two black folding carry
# handles that each pivot up from opposite flat rims and fold down flat.
#
# Coordinate convention: +Z up, basket rests on z=0, elongated along X
# (wider across corners in X than across flats in Y).

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Dimensions (meters)
# ---------------------------------------------------------------------------
BODY_H = 0.230  # body height

# Hexagonal footprint (flat-top: flats at +/-Y, points at +/-X)
# half_x = center to pointed vertex; half_y = center to flat midpoint.
BOT_HX = 0.190  # bottom (across corners 0.38 m)
BOT_HY = 0.145  # bottom (across flats  0.29 m)
TOP_HX = 0.215  # top    (across corners 0.43 m)
TOP_HY = 0.165  # top    (across flats  0.33 m)

WALL_T = 0.004   # wall / lattice bar thickness
FLOOR_T = 0.006  # floor plate thickness

# Rolled top rim
RIM_H = 0.018
RIM_LIP = 0.012
RIM_Z = BODY_H - RIM_H / 2.0

# Grip ears at +/-X pointed vertices
EAR_X = 0.022
EAR_Y = 0.080
EAR_Z = 0.030

# Lattice bars (X-pattern on each wall face)
BAR_WIDTH = 0.008        # width of each diagonal bar
LATTICE_PITCH = 0.030    # center-to-center spacing of bars
LATTICE_GAP = LATTICE_PITCH - BAR_WIDTH  # open gap between bars (0.022 m)

# Corner structural posts at hex vertices
POST_SIZE = 0.009

# Handles
HANDLE_ARCH_H = 0.210
HANDLE_FOOT_SPAN_X = 0.170
HANDLE_STRAP_W = 0.024
HANDLE_STRAP_T = 0.007
KNUCKLE_R = 0.012
PIVOT_Y = TOP_HY - 0.006
PIVOT_Z = BODY_H - 0.012

BLUE = (0.10, 0.32, 0.92, 1.0)
BLACK = (0.07, 0.07, 0.08, 1.0)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def _hex_verts(half_x: float, half_y: float):
    """6 vertices of a flat-top hexagon (flats at +/-Y, points at +/-X), CCW."""
    return [
        (half_x, 0.0),
        (half_x / 2.0, half_y),
        (-half_x / 2.0, half_y),
        (-half_x, 0.0),
        (-half_x / 2.0, -half_y),
        (half_x / 2.0, -half_y),
    ]


def _hex_prism(half_x: float, half_y: float, h: float):
    """Straight hexagonal prism sitting on z=0."""
    pts = _hex_verts(half_x, half_y)
    return (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .polyline(pts).close()
        .workplane(offset=h)
        .polyline(pts).close()
        .loft()
    )


def _tapered_hex(bot_hx: float, bot_hy: float,
                 top_hx: float, top_hy: float, h: float):
    """Tapered hexagonal prism sitting on z=0."""
    bot_pts = _hex_verts(bot_hx, bot_hy)
    top_pts = _hex_verts(top_hx, top_hy)
    return (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .polyline(bot_pts).close()
        .workplane(offset=h)
        .polyline(top_pts).close()
        .loft()
    )


def _build_lattice_wall(wall_len: float, wall_height: float):
    """Build an X-pattern cross-lattice wall panel from positive bars.

    Local frame: X = along wall, Y = wall normal (thickness), Z = vertical.
    Centered at origin.  Built by creating two sets of parallel diagonal bars
    that physically cross each other (unioned at crossing points), then
    clipping to the wall rectangle and adding a structural frame.
    """
    bar_len = math.sqrt(wall_len ** 2 + wall_height ** 2) + 0.04
    n = int(bar_len / LATTICE_PITCH) + 2

    # --- Diagonal bars (+45° and -45°) as positive geometry -----------------
    # Each bar is a thin box at origin, offset in Z, then tilted around Y.
    # Bars from different directions cross at 90° in XZ and physically
    # overlap at each crossing, so boolean union merges them.

    # Build all bars interleaved so each new bar crosses existing bars.
    bars = None
    for i in range(-n, n + 1):
        offset = i * LATTICE_PITCH

        # +45° bar
        bar_plus = (
            cq.Workplane("XY")
            .box(bar_len, WALL_T, BAR_WIDTH)
            .translate((0.0, 0.0, offset))
            .rotate((0, 0, 0), (0, 1, 0), -45.0)
        )
        bars = bar_plus if bars is None else bars.union(bar_plus)

        # -45° bar
        bar_minus = (
            cq.Workplane("XY")
            .box(bar_len, WALL_T, BAR_WIDTH)
            .translate((0.0, 0.0, offset))
            .rotate((0, 0, 0), (0, 1, 0), 45.0)
        )
        bars = bars.union(bar_minus)

    # --- Clip to wall rectangle ---------------------------------------------
    clip = cq.Workplane("XY").box(
        wall_len + 0.001, WALL_T + 0.001, wall_height
    )
    lattice = bars.intersect(clip)

    # --- Structural frame (horizontal rails + vertical stiles) ---------------
    # Ensures the lattice is one connected piece and provides solid edges
    # that overlap with the floor, rim, and corner posts.
    rail_h = BAR_WIDTH * 2.5  # ~20 mm
    stile_w = BAR_WIDTH * 2.5

    bottom_rail = (
        cq.Workplane("XY")
        .box(wall_len, WALL_T, rail_h)
        .translate((0.0, 0.0, -wall_height / 2.0 + rail_h / 2.0))
    )
    top_rail = (
        cq.Workplane("XY")
        .box(wall_len, WALL_T, rail_h)
        .translate((0.0, 0.0, wall_height / 2.0 - rail_h / 2.0))
    )
    left_stile = (
        cq.Workplane("XY")
        .box(stile_w, WALL_T, wall_height)
        .translate((-wall_len / 2.0 + stile_w / 2.0, 0.0, 0.0))
    )
    right_stile = (
        cq.Workplane("XY")
        .box(stile_w, WALL_T, wall_height)
        .translate((wall_len / 2.0 - stile_w / 2.0, 0.0, 0.0))
    )

    lattice = (
        lattice.union(bottom_rail)
        .union(top_rail)
        .union(left_stile)
        .union(right_stile)
    )

    return lattice


def _build_rim():
    """Hexagonal rolled rim band at the top of the basket."""
    rim_z_bot = BODY_H - RIM_H

    # Outer profile with outward lip
    outer_hx = TOP_HX + RIM_LIP
    outer_hy = TOP_HY + RIM_LIP
    outer = _hex_prism(outer_hx, outer_hy, RIM_H).translate(
        (0.0, 0.0, rim_z_bot)
    )

    # Inner cutout (slightly smaller than body top opening)
    inner_hx = TOP_HX - WALL_T
    inner_hy = TOP_HY - WALL_T
    inner = _hex_prism(inner_hx, inner_hy, RIM_H + 0.02).translate(
        (0.0, 0.0, rim_z_bot - 0.01)
    )

    return outer.cut(inner)


def _build_body():
    """Hexagonal basket body: floor + lattice walls + corner posts + rim
    + grip ears + floor ribs."""

    # --- Floor plate (hex base that bridges to lattice walls) ---------------
    # The floor extends to the top hex perimeter (+margin) so the vertical
    # lattice walls and corner posts overlap with it.  A small lip at the
    # base reads as a reinforced base plate on the real basket.
    floor_hx = TOP_HX + 0.004
    floor_hy = TOP_HY + 0.004
    floor = _hex_prism(floor_hx, floor_hy, FLOOR_T + 0.004)

    body = floor

    # --- Six lattice walls --------------------------------------------------
    verts_top = _hex_verts(TOP_HX, TOP_HY)

    for i in range(6):
        a = verts_top[i]
        b = verts_top[(i + 1) % 6]
        mx = (a[0] + b[0]) / 2.0
        my = (a[1] + b[1]) / 2.0
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        wall_len = math.sqrt(dx * dx + dy * dy)
        theta_deg = math.degrees(math.atan2(dy, dx))

        # Slightly taller than body so bars overlap floor and rim
        wall_h = BODY_H + 0.006

        lattice = _build_lattice_wall(wall_len, wall_h)
        lattice = lattice.rotate((0, 0, 0), (0, 0, 1), theta_deg)
        lattice = lattice.translate((mx, my, BODY_H / 2.0))

        body = body.union(lattice)

    # --- Corner posts at each hex vertex -----------------------------------
    for vx, vy in _hex_verts(TOP_HX, TOP_HY):
        post = (
            cq.Workplane("XY")
            .box(POST_SIZE, POST_SIZE, BODY_H)
            .translate((vx, vy, BODY_H / 2.0))
        )
        body = body.union(post)

    # --- Rolled rim ---------------------------------------------------------
    rim = _build_rim()
    body = body.union(rim)

    # --- Grip ears at +/-X pointed vertices --------------------------------
    for sx in (1.0, -1.0):
        ear = (
            cq.Workplane("XY")
            .box(EAR_X * 2.0, EAR_Y, EAR_Z)
            .edges("|Y").fillet(0.008)
            .translate((sx * (TOP_HX + RIM_LIP), 0.0, RIM_Z))
        )
        body = body.union(ear)

    # --- Floor ribs ---------------------------------------------------------
    for rx in (-0.08, 0.0, 0.08):
        rib = (
            cq.Workplane("XY")
            .box(0.006, BOT_HY * 1.6, 0.004)
            .translate((rx, 0.0, FLOOR_T))
        )
        body = body.union(rib)

    return body


def _build_handle():
    """Flat black strap bent into a tall inverted-U arch.

    Local frame: origin on pivot line midway between feet, arch rises in +Z,
    pivots about local X.  Same design as the original rectangular basket.
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

    def normal_at(idx: int):
        i0 = max(0, idx - 1)
        i1 = min(n, idx + 1)
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
        .polyline(profile_pts).close()
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
    model = ArticulatedObject(name="hex_shopping_basket")

    blue = model.material("basket_blue", rgba=BLUE)
    black = model.material("handle_black", rgba=BLACK)

    # Root: basket body
    basket = model.part("basket_tub")
    basket.visual(
        mesh_from_cadquery(_build_body(), "basket_tub"), material=blue
    )
    basket.inertial = Inertial.from_geometry(
        Box((TOP_HX * 2, TOP_HY * 2, BODY_H)),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # Two folding handles on opposite flat rims (+/-Y)
    handle_mesh = _build_handle()
    for idx, sy in enumerate((1.0, -1.0)):
        name = f"handle_{idx}"
        handle = model.part(name)
        handle.visual(
            mesh_from_cadquery(handle_mesh, name), material=black
        )
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

    # --- Footprint: hexagonal, wider across corners (X) than flats (Y) -----
    lo, hi = ctx.part_world_aabb(basket)
    width_x = hi[0] - lo[0]
    depth_y = hi[1] - lo[1]
    height_z = hi[2] - lo[2]

    ctx.check(
        "hexagonal footprint wider across corners (X) than flats (Y)",
        width_x > depth_y + 0.02,
        details=f"x={width_x:.3f}, y={depth_y:.3f}",
    )
    ctx.check(
        "basket rests at z~0",
        abs(lo[2]) < 0.01,
        details=f"min_z={lo[2]:.4f}",
    )
    ctx.check(
        "basket realistic height",
        0.18 < height_z < 0.32,
        details=f"height={height_z:.3f}",
    )

    # --- Two handles on opposite flat rims (+/-Y) --------------------------
    p0 = ctx.part_world_position(handle_0)
    p1 = ctx.part_world_position(handle_1)
    ctx.check(
        "handles mounted on opposite flat rims (+/-Y)",
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

    # --- Handle pivot: arch drops when folded flat -------------------------
    for handle, joint, name in (
        (handle_0, j0, "handle_0"),
        (handle_1, j1, "handle_1"),
    ):
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

    # --- Handle knuckle overlap (intentional, captured in rim) -------------
    ctx.allow_overlap(
        basket,
        handle_0,
        reason=(
            "handle_0 pivot knuckles are intentionally captured inside "
            "the rim at the pivot line."
        ),
    )
    ctx.allow_overlap(
        basket,
        handle_1,
        reason=(
            "handle_1 pivot knuckles are intentionally captured inside "
            "the rim at the pivot line."
        ),
    )

    return ctx.report()


object_model = build_object_model()
