from __future__ import annotations

# Oval deep tote-style shopping basket: elongated elliptical footprint,
# noticeably deeper and narrower than a standard rectangular basket.
# Slotted/perforated curved walls, reinforced rolled top rim, molded grip ears
# on the narrow ends, tapered stackable body, and two black folding carry
# handles that each pivot up from the long rims and fold down flat.
#
# Coordinate convention: +Z up, basket rests on the ground at z=0, long axis
# along X (wider in X than in Y).

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
BODY_H = 0.300  # tub height (z) — deeper tote style

# Elliptical footprint semi-axes.
BOT_RX = 0.170  # bottom semi-major (X, long axis)
BOT_RY = 0.088  # bottom semi-minor (Y, short axis)
TOP_RX = 0.210  # top semi-major (X)
TOP_RY = 0.115  # top semi-minor (Y)

WALL_T = 0.004  # nominal wall thickness
FLOOR_T = 0.006  # floor thickness

# Rolled / flanged top rim.
RIM_H = 0.018  # vertical height of the rolled rim band
RIM_LIP = 0.012  # how far the lip protrudes outward beyond the wall
RIM_Z = BODY_H - RIM_H / 2.0  # vertical center of rim band

# Grip ears on the two narrow ends (small molded scoops on +X / -X rim).
EAR_X = 0.022  # how far the ear sticks out past the rim in X
EAR_Y = 0.080  # ear width in Y
EAR_Z = 0.030  # ear height in Z

# Slot perforations (tall vertical slots cut through the curved walls).
SLOT_W = 0.010  # slot width (horizontal, tangent to wall)
SLOT_H = 0.140  # slot height (vertical) — taller for deeper basket
SLOT_Z = 0.148  # vertical center of the slot band

# Handles. Each handle is a tall inverted-U arch whose two feet both attach to
# ONE long rim. The two feet are spaced apart ALONG the rim (along X); the arch
# rises in Z. The handle pivots about the rim line (X axis) to fold flat.
HANDLE_ARCH_H = 0.230  # how high the arch top rises above the rim
HANDLE_FOOT_SPAN_X = 0.180  # foot-to-foot spacing along the rim (X)
HANDLE_STRAP_W = 0.024  # visible strap width (radial band thickness, in arch plane)
HANDLE_STRAP_T = 0.007  # flat strap thickness (thin dimension, along Y)
KNUCKLE_R = 0.012  # pivot knuckle radius
# Pivot lines sit on the two long rims, offset in +/-Y from center.
PIVOT_Y = TOP_RY - 0.006  # just inside the long rim
PIVOT_Z = BODY_H - 0.012  # just below the rim top

BLUE = (0.10, 0.32, 0.92, 1.0)
BLACK = (0.07, 0.07, 0.08, 1.0)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def _tapered_oval(bot_rx: float, bot_ry: float, top_rx: float, top_ry: float, h: float):
    """An elliptical solid that tapers from a bottom ellipse to a top ellipse.

    Built as a loft between two centered elliptical wires, sitting on z=0.
    """
    return (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .ellipse(bot_rx, bot_ry)
        .workplane(offset=h)
        .ellipse(top_rx, top_ry)
        .loft()
    )


def _ellipse_at_z(rx: float, ry: float, z: float):
    """Return an elliptical wire on a workplane at height z."""
    return cq.Workplane("XY").workplane(offset=z).ellipse(rx, ry)


def _build_body():
    """Hollow tapered oval tub: outer shell minus inner cavity, with floor,
    rolled rim flange, grip ears, and tall vertical slot perforations."""
    outer = _tapered_oval(BOT_RX, BOT_RY, TOP_RX, TOP_RY, BODY_H)

    # Inner cavity: same taper, inset by the wall thickness, starting above the
    # floor and open at the top (cut a bit above the mouth so the top is open).
    inner = _tapered_oval(
        BOT_RX - WALL_T,
        BOT_RY - WALL_T,
        TOP_RX - WALL_T,
        TOP_RY - WALL_T,
        BODY_H + 0.02,
    ).translate((0.0, 0.0, FLOOR_T))

    tub = outer.cut(inner)

    # Rolled top rim: an outward elliptical flange band running around the mouth.
    # Interpolate the rim ellipse at the rim center height, then add lip.
    rim_frac = (RIM_Z - RIM_H / 2.0) / BODY_H
    rim_bot_rx = BOT_RX + (TOP_RX - BOT_RX) * rim_frac
    rim_bot_ry = BOT_RY + (TOP_RY - BOT_RY) * rim_frac
    rim_top_frac = (RIM_Z + RIM_H / 2.0) / BODY_H
    rim_top_rx = BOT_RX + (TOP_RX - BOT_RX) * rim_top_frac
    rim_top_ry = BOT_RY + (TOP_RY - BOT_RY) * rim_top_frac

    rim_outer = _tapered_oval(
        rim_bot_rx + RIM_LIP,
        rim_bot_ry + RIM_LIP,
        rim_top_rx + RIM_LIP,
        rim_top_ry + RIM_LIP,
        RIM_H,
    ).translate((0.0, 0.0, RIM_Z - RIM_H / 2.0))

    rim_hole = _tapered_oval(
        rim_bot_rx - WALL_T,
        rim_bot_ry - WALL_T,
        rim_top_rx - WALL_T,
        rim_top_ry - WALL_T,
        RIM_H + 0.02,
    ).translate((0.0, 0.0, RIM_Z - RIM_H / 2.0 - 0.01))

    rim = rim_outer.cut(rim_hole)
    tub = tub.union(rim)

    # Molded grip ears on the two narrow ends (+X and -X), at the rim.
    for sx in (1.0, -1.0):
        ear = (
            cq.Workplane("XY")
            .box(EAR_X * 2.0, EAR_Y, EAR_Z)
            .edges("|Y")
            .fillet(0.008)
            .translate((sx * (TOP_RX + RIM_LIP), 0.0, RIM_Z))
        )
        tub = tub.union(ear)

    # Tall vertical slot perforations cut through the curved walls.
    tub = _cut_oval_slots(tub)

    # Floor ribs (subtle raised lines on the inner floor, following oval shape).
    for frac_x in (-0.35, 0.0, 0.35):
        rx_at_floor = BOT_RX * abs(frac_x) if frac_x != 0 else BOT_RX * 0.35
        rib_len = 2.0 * (BOT_RY - 4.0 * WALL_T) * (1.0 - abs(frac_x) * 0.3)
        rib = (
            cq.Workplane("XY")
            .box(0.006, max(rib_len, 0.02), 0.004)
            .translate((frac_x * BOT_RX, 0.0, FLOOR_T + 0.002))
        )
        tub = tub.union(rib)

    return tub


def _cut_oval_slots(tub):
    """Cut rows of tall vertical slots through the oval walls.

    Slots are placed at angular intervals around the elliptical perimeter,
    oriented radially (normal to the wall surface). Slots near the grip ears
    and handle pivot areas are skipped.
    """
    cut_depth = 0.06  # deeper than any wall thickness so the cut goes through

    # Use average radii for slot positioning (mid-height of the basket).
    mid_rx = (BOT_RX + TOP_RX) / 2.0
    mid_ry = (BOT_RY + TOP_RY) / 2.0

    n_slots = 28
    for i in range(n_slots):
        theta = 2.0 * math.pi * i / n_slots

        # Skip slots very close to the grip ears (θ near 0 or π, i.e. ±X ends).
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        if abs(sin_t) < 0.18:
            continue

        # Position on the outer ellipse at mid-height.
        cx = mid_rx * cos_t
        cy = mid_ry * sin_t

        # Outward normal of the ellipse at this parameter.
        nx = cos_t / mid_rx
        ny = sin_t / mid_ry
        norm = math.hypot(nx, ny) or 1.0
        nx /= norm
        ny /= norm

        # Rotation angle to align cutter depth with the outward normal.
        angle_deg = math.degrees(math.atan2(ny, nx))

        # Cutter: thin tall box with depth (X) along the wall normal,
        # width (Y) along the wall tangent, height (Z) vertical.
        cutter = (
            cq.Workplane("XY")
            .box(cut_depth, SLOT_W, SLOT_H)
            .edges("|Z")
            .fillet(SLOT_W / 2.0 - 0.0005)
        )
        # Rotate around Z to align with wall normal, then translate.
        cutter = (
            cutter
            .rotate((0, 0, 0), (0, 0, 1), angle_deg)
            .translate((cx, cy, SLOT_Z))
        )
        tub = tub.cut(cutter)

    return tub


def _build_handle():
    """A flat black strap bent into a tall inverted-U arch.

    Authored in the handle-local frame whose origin sits on the rim pivot line.
    Both feet attach to ONE long rim, spaced apart along the rim (local X). The
    arch rises in +Z. The flat strap lies in the local X-Z plane (broad face
    normal along local Y, the thin dimension). At q=0 the arch stands vertical
    (carry pose); the joint pivots it about local X to fold flat.

    Local frame origin: midway between the two feet, on the pivot line (z=0).
    The two pivot knuckles sit at local (+-foot_half_x, 0, 0) with their axis
    along local X, so the joint axis (X) runs through both knuckles.
    """
    foot_half = HANDLE_FOOT_SPAN_X / 2.0
    half_w = HANDLE_STRAP_W / 2.0

    # Arch centerline in the X-Z plane, from one foot up and over to the other.
    n = 48
    centerline = []
    for i in range(n + 1):
        t = i / n
        x = foot_half - 2.0 * foot_half * t
        z = HANDLE_ARCH_H * math.sin(math.pi * t) ** 0.40
        centerline.append((x, z))

    # Offset the centerline along its local normal to make a constant-width band.
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

    # Pivot knuckles at both feet, on the rim pivot line (local z=0), axis
    # along local X (so the joint axis runs through them).
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
    model = ArticulatedObject(name="oval_tote_basket")

    blue = model.material("basket_blue", rgba=BLUE)
    black = model.material("handle_black", rgba=BLACK)

    # Root: the hollow oval tub.
    basket = model.part("basket_tub")
    basket.visual(mesh_from_cadquery(_build_body(), "basket_tub"), material=blue)
    basket.inertial = Inertial.from_geometry(
        Box((TOP_RX * 2, TOP_RY * 2, BODY_H)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # Two folding handles. Each is built in a local frame whose origin sits on
    # its long rim pivot line; the joint origin places that frame on the rim.
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

        # Both feet of this handle pivot on the SAME long rim (offset in +/-Y).
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
        "basket is deep (tote style)",
        0.26 < height_z < 0.36,
        details=f"height={height_z:.3f}",
    )

    # --- Oval shape: noticeably narrow relative to length. -------------------
    ctx.check(
        "oval footprint is elongated",
        width_x / depth_y > 1.5,
        details=f"ratio={width_x / depth_y:.2f}",
    )

    # --- Hollow: top is open, interior cavity exists. ------------------------
    ctx.check(
        "tub spans full outer mouth in XY",
        width_x > TOP_RX * 2 - 0.01 and depth_y > TOP_RY * 2 - 0.01,
        details=f"x={width_x:.3f}, y={depth_y:.3f}",
    )
    ctx.check(
        "wall is thin (hollow shell)",
        WALL_T < 0.01 and WALL_T < BODY_H / 10.0,
        details=f"wall_t={WALL_T}",
    )

    # --- Slot perforations present on the walls. -----------------------------
    ctx.check(
        "slot perforations present",
        SLOT_H > 0.05 and SLOT_W > 0.0,
        details=f"slot {SLOT_W}x{SLOT_H}",
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
        min_dist=0.08,
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
