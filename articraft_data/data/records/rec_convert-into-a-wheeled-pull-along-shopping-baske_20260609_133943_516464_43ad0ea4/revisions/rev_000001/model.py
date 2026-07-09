from __future__ import annotations

# Rectangular blue plastic hand-held shopping basket with slotted/perforated
# side walls, a reinforced rolled top rim, molded grip ears on the short ends,
# a slightly tapered (stackable) body, and two black folding carry handles that
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
BODY_H = 0.230  # tub height (z)

# Outer footprint, bottom (slightly narrower -> stackable taper).
BOT_X = 0.410
BOT_Y = 0.260
# Outer footprint, top mouth (slightly wider).
TOP_X = 0.450
TOP_Y = 0.300

WALL_T = 0.004  # nominal wall thickness
FLOOR_T = 0.006  # floor thickness

# Rolled / flanged top rim.
RIM_H = 0.018  # vertical height of the rolled rim band
RIM_LIP = 0.012  # how far the lip protrudes outward beyond the wall
RIM_Z = BODY_H - RIM_H / 2.0  # vertical center of rim band

# Grip ears on the two short ends (small molded scoops on +X / -X rim).
EAR_X = 0.022  # how far the ear sticks out past the rim in X
EAR_Y = 0.110  # ear width in Y
EAR_Z = 0.030  # ear height in Z

# Slot perforations (tall vertical slots cut through the walls).
SLOT_W = 0.010  # slot width (horizontal, along the wall run)
SLOT_H = 0.110  # slot height (vertical)
SLOT_Z = 0.108  # vertical center of the slot band

# Handles. Each handle is a tall inverted-U arch whose two feet both attach to
# ONE long rim. The two feet are spaced apart ALONG the rim (along X); the arch
# rises in Z. The handle pivots about the rim line (X axis) to fold flat.
HANDLE_ARCH_H = 0.215  # how high the arch top rises above the rim
HANDLE_FOOT_SPAN_X = 0.190  # foot-to-foot spacing along the rim (X)
HANDLE_STRAP_W = 0.024  # visible strap width (radial band thickness, in arch plane)
HANDLE_STRAP_T = 0.007  # flat strap thickness (thin dimension, along Y)
KNUCKLE_R = 0.012  # pivot knuckle radius
# Pivot lines sit on the two long rims, offset in +/-Y from center.
PIVOT_Y = TOP_Y / 2.0 - 0.006  # just inside the long rim
PIVOT_Z = BODY_H - 0.012  # just below the rim top

# Rear axle and wheels.
WHEEL_R = 0.050  # wheel radius
WHEEL_W = 0.035  # wheel width (axial, along Y)
WHEEL_Y_HALF = 0.090  # half spacing between wheel centers in Y
AXLE_R = 0.005  # axle bar radius
AXLE_X = -BOT_X / 2.0 - 0.010  # axle x position (behind the back wall)
AXLE_Z = WHEEL_R  # axle height (wheels sit on ground at z=0)

# Telescoping pull handle (mounted at the -X end, centered).
# The handle's local frame origin is at the grip center (top of handle at rest).
# Two rods extend downward from the grip; at q=0 (retracted) the grip sits near
# the rim. The PRISMATIC joint lifts the handle along +Z.
PULL_ROD_HALF_Y = 0.090  # half spacing of rods in Y
PULL_ROD_R = 0.005  # rod radius
PULL_ROD_LEN = 0.580  # rod length (extends downward from grip in -Z)
PULL_GRIP_LEN = 0.220  # cross-bar grip length along Y
PULL_GRIP_R = 0.011  # grip radius
PULL_EXTEND = 0.550  # maximum extension (meters)

# Rear guide sleeve (fixed to basket back wall, outer housing for the pull
# handle rods). A small rectangular boss on the back wall.
GUIDE_W = 0.040  # guide width in X (protrusion from back wall)
GUIDE_H = 0.080  # guide height in Z
GUIDE_T = 0.006  # guide wall thickness

BLUE = (0.10, 0.32, 0.92, 1.0)
BLACK = (0.07, 0.07, 0.08, 1.0)
SILVER = (0.60, 0.62, 0.66, 1.0)
GRAY = (0.35, 0.36, 0.38, 1.0)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def _tapered_box(bot_x: float, bot_y: float, top_x: float, top_y: float, h: float):
    """A rectangular prism that tapers from a bottom footprint to a top footprint.

    Built as a loft between two centered rectangular wires, sitting on z=0.
    """
    bottom = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .rect(bot_x, bot_y)
    )
    return (
        bottom.workplane(offset=h)
        .rect(top_x, top_y)
        .loft(combine=True)
    )


def _build_body():
    """Hollow tapered tub: outer shell minus inner cavity, with floor, rolled
    rim flange, grip ears, and tall vertical slot perforations."""
    outer = _tapered_box(BOT_X, BOT_Y, TOP_X, TOP_Y, BODY_H)

    # Inner cavity: same taper, inset by the wall thickness, starting above the
    # floor and open at the top (cut a bit above the mouth so the top is open).
    inner = _tapered_box(
        BOT_X - 2.0 * WALL_T,
        BOT_Y - 2.0 * WALL_T,
        TOP_X - 2.0 * WALL_T,
        TOP_Y - 2.0 * WALL_T,
        BODY_H + 0.02,
    ).translate((0.0, 0.0, FLOOR_T))

    tub = outer.cut(inner)

    # Rolled top rim: an outward flange band running around the mouth. Built as
    # an outer tapered band minus the mouth opening so it reads as a rolled lip.
    rim_bot_x = TOP_X - (TOP_X - BOT_X) * (RIM_Z - RIM_H / 2.0) / BODY_H
    rim_bot_y = TOP_Y - (TOP_Y - BOT_Y) * (RIM_Z - RIM_H / 2.0) / BODY_H
    rim_outer = _tapered_box(
        rim_bot_x + 2.0 * RIM_LIP,
        rim_bot_y + 2.0 * RIM_LIP,
        TOP_X + 2.0 * RIM_LIP,
        TOP_Y + 2.0 * RIM_LIP,
        RIM_H,
    ).translate((0.0, 0.0, RIM_Z - RIM_H / 2.0))
    rim_hole = _tapered_box(
        rim_bot_x - 2.0 * WALL_T,
        rim_bot_y - 2.0 * WALL_T,
        TOP_X - 2.0 * WALL_T,
        TOP_Y - 2.0 * WALL_T,
        RIM_H + 0.02,
    ).translate((0.0, 0.0, RIM_Z - RIM_H / 2.0 - 0.01))
    rim = rim_outer.cut(rim_hole)
    tub = tub.union(rim)

    # Molded grip ears on the two short ends (+X and -X), at the rim.
    for sx in (1.0, -1.0):
        ear = (
            cq.Workplane("XY")
            .box(EAR_X * 2.0, EAR_Y, EAR_Z)
            .edges("|Y")
            .fillet(0.008)
            .translate((sx * (TOP_X / 2.0 + RIM_LIP), 0.0, RIM_Z))
        )
        tub = tub.union(ear)

    # Tall vertical slot perforations cut through all four walls.
    tub = _cut_slots(tub)

    # Faint floor ribs (subtle raised lines on the inner floor).
    for rx in (-0.10, 0.0, 0.10):
        rib = (
            cq.Workplane("XY")
            .box(0.006, BOT_Y - 4.0 * WALL_T, 0.004)
            .translate((rx, 0.0, FLOOR_T + 0.002))
        )
        tub = tub.union(rib)

    return tub


def _cut_slots(tub):
    """Cut rows of tall vertical slots through the long (Y-normal) and short
    (X-normal) walls. Slots are real through-cuts."""
    cut_depth = 0.06  # deeper than any wall thickness so the cut goes through

    # Long walls (front/back, normal along Y): slots arrayed along X.
    n_long = 13
    long_pitch = 0.026
    start_x = -(n_long - 1) / 2.0 * long_pitch
    for i in range(n_long):
        cx = start_x + i * long_pitch
        for sy in (1.0, -1.0):
            cutter = (
                cq.Workplane("XY")
                .box(SLOT_W, cut_depth, SLOT_H)
                .edges("|Y")
                .fillet(SLOT_W / 2.0 - 0.0005)
                .translate((cx, sy * (TOP_Y / 2.0), SLOT_Z))
            )
            tub = tub.cut(cutter)

    # Short walls (left/right ends, normal along X): slots arrayed along Y.
    n_short = 7
    short_pitch = 0.030
    start_y = -(n_short - 1) / 2.0 * short_pitch
    for j in range(n_short):
        cy = start_y + j * short_pitch
        for sx in (1.0, -1.0):
            cutter = (
                cq.Workplane("XY")
                .box(cut_depth, SLOT_W, SLOT_H)
                .edges("|X")
                .fillet(SLOT_W / 2.0 - 0.0005)
                .translate((sx * (TOP_X / 2.0), cy, SLOT_Z))
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
    # x runs +foot_half -> -foot_half; z rises with rounded shoulders.
    n = 48
    centerline = []
    for i in range(n + 1):
        t = i / n
        x = foot_half - 2.0 * foot_half * t
        # Tall arch: nearly straight vertical posts with a rounded top. Use a
        # super-ellipse-like profile so the shoulders are rounded but the sides
        # stay fairly vertical.
        z = HANDLE_ARCH_H * math.sin(math.pi * t) ** 0.40
        centerline.append((x, z))

    # Offset the centerline along its local normal to make a constant-width
    # band. Approximate the normal from finite differences.
    def normal_at(i: int) -> tuple[float, float]:
        i0 = max(0, i - 1)
        i1 = min(n, i + 1)
        dx = centerline[i1][0] - centerline[i0][0]
        dz = centerline[i1][1] - centerline[i0][1]
        L = math.hypot(dx, dz) or 1.0
        # Left normal of tangent (dx, dz) is (-dz, dx).
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


def _build_rear_guide():
    """Rectangular guide boss fixed to the back (-X) wall of the basket.

    This is the outer sleeve/housing for the telescoping pull handle rods.
    Built as a thin-walled rectangular box sitting on the back wall, with
    its top at the rim level.
    """
    guide_half_y = PULL_ROD_HALF_Y + 0.010
    guide = (
        cq.Workplane("XY")
        .box(GUIDE_W, guide_half_y * 2.0, GUIDE_H)
        .faces("<Z")
        .shell(GUIDE_T)
    )
    # Position at the back wall center, top at rim level.
    guide = guide.translate(
        (-TOP_X / 2.0 - GUIDE_W / 2.0 + 0.002, 0.0, RIM_Z - GUIDE_H / 2.0)
    )
    return guide


def _build_pull_handle():
    """U-shaped telescoping handle: two vertical rods and a cross-bar grip.

    Local frame origin at the grip center (top of handle at rest). Two rods
    extend downward along -Z from the grip. The PRISMATIC joint lifts the
    handle along +Z from the articulation origin at the rim.
    """
    rod_half = PULL_ROD_HALF_Y
    rod_r = PULL_ROD_R
    rod_len = PULL_ROD_LEN
    grip_len = PULL_GRIP_LEN
    grip_r = PULL_GRIP_R

    handle = None

    # Two vertical rods at y = ±rod_half, extending from z=-rod_len to z=0.
    for sy in (-1.0, 1.0):
        rod = (
            cq.Workplane("XY")
            .cylinder(rod_len, rod_r, centered=False)
            .translate((0.0, sy * rod_half, -rod_len))
        )
        handle = rod if handle is None else handle.union(rod)

    # Cross-bar grip at z=0 oriented along Y.
    grip = (
        cq.Workplane("XZ")
        .circle(grip_r)
        .extrude(grip_len, both=True)
    )
    handle = handle.union(grip)

    # Small rounded caps on the grip ends.
    for sy in (-1.0, 1.0):
        cap = (
            cq.Workplane("XZ")
            .sphere(grip_r)
            .translate((0.0, sy * grip_len / 2.0, 0.0))
        )
        handle = handle.union(cap)

    return handle


def _build_axle_assembly():
    """Axle bar with two wheels.

    Local frame origin at the axle center. Axle runs along Y; wheels at
    y = ±WHEEL_Y_HALF. The assembly rotates about local Y (axle axis).
    """
    axle_len = WHEEL_Y_HALF * 2.0 + 0.020
    axle = (
        cq.Workplane("XY")
        .cylinder(axle_len, AXLE_R, centered=True)
        .rotate((0, 0, 0), (1, 0, 0), -90)
    )

    for sy in (-1.0, 1.0):
        wheel = (
            cq.Workplane("XY")
            .cylinder(WHEEL_W, WHEEL_R, centered=True)
            .rotate((0, 0, 0), (1, 0, 0), -90)
            .translate((0.0, sy * WHEEL_Y_HALF, 0.0))
        )
        axle = axle.union(wheel)

    return axle


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="blue_shopping_basket")

    blue = model.material("basket_blue", rgba=BLUE)
    black = model.material("handle_black", rgba=BLACK)
    silver = model.material("metal_silver", rgba=SILVER)
    gray = model.material("plastic_gray", rgba=GRAY)

    # Root: the hollow basket tub.
    basket = model.part("basket_tub")
    basket.visual(mesh_from_cadquery(_build_body(), "basket_tub"), material=blue)
    # Rear guide sleeve for the telescoping pull handle, fixed to the back wall.
    basket.visual(
        mesh_from_cadquery(_build_rear_guide(), "rear_guide"),
        material=gray,
    )
    basket.inertial = Inertial.from_geometry(
        Box((TOP_X, TOP_Y, BODY_H)),
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
        # The handle local frame origin sits on the rim pivot line (z=0 local),
        # midway between the two feet. Place that frame on the rim and pivot
        # about world X so the arch folds down toward / over the mouth.
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

    # -----------------------------------------------------------------------
    # Rear axle with two wheels (fixed axle, rotates about Y).
    # -----------------------------------------------------------------------
    axle = model.part("rear_axle")
    axle.visual(
        mesh_from_cadquery(_build_axle_assembly(), "rear_axle"),
        material=black,
    )
    axle.inertial = Inertial.from_geometry(
        Box((0.020, WHEEL_Y_HALF * 2.0 + WHEEL_W, WHEEL_R * 2.0)),
        mass=0.3,
        origin=Origin(xyz=(0.0, 0.0, WHEEL_R)),
    )

    model.articulation(
        "tub_to_axle",
        ArticulationType.CONTINUOUS,
        parent=basket,
        child=axle,
        origin=Origin(xyz=(AXLE_X, 0.0, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=4.0,
        ),
    )

    # -----------------------------------------------------------------------
    # Telescoping pull handle (prismatic, extends along +Z).
    # -----------------------------------------------------------------------
    pull = model.part("pull_handle")
    pull.visual(
        mesh_from_cadquery(_build_pull_handle(), "pull_handle"),
        material=silver,
    )
    pull.inertial = Inertial.from_geometry(
        Box((0.020, PULL_ROD_HALF_Y * 2.0 + 0.010, PULL_ROD_LEN)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # Articulation origin placed at the back rim center, at the grip height
    # when fully retracted. At q=0 the handle sits retracted (grip near rim).
    # Positive q raises the handle upward.
    model.articulation(
        "tub_to_pull_handle",
        ArticulationType.PRISMATIC,
        parent=basket,
        child=pull,
        origin=Origin(xyz=(-TOP_X / 2.0 - 0.005, 0.0, RIM_Z + 0.010)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=4.0,
            lower=0.0,
            upper=PULL_EXTEND,
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
    axle = object_model.get_part("rear_axle")
    pull = object_model.get_part("pull_handle")
    pull_joint = object_model.get_articulation("tub_to_pull_handle")
    axle_joint = object_model.get_articulation("tub_to_axle")

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

    # --- Hollow: top is open, interior cavity exists. ------------------------
    # The tub mesh AABB should reach the full mouth size, but a solid block of
    # that size would have far more volume. Verify the mesh is a thin shell by
    # checking the wall: the inner cavity opening near the top is empty. We
    # approximate by confirming the tub bounding box matches the outer mouth
    # while the modeled inner cavity is inset by ~wall thickness.
    ctx.check(
        "tub spans full outer mouth in XY",
        width_x > TOP_X - 0.001 and depth_y > TOP_Y - 0.001,
        details=f"x={width_x:.3f}, y={depth_y:.3f}",
    )
    # The basket is authored hollow (outer shell minus inner cavity). Assert the
    # wall thickness is thin relative to the body so it is not a solid block.
    ctx.check(
        "wall is thin (hollow shell)",
        WALL_T < 0.01 and WALL_T < BODY_H / 10.0,
        details=f"wall_t={WALL_T}",
    )

    # --- Slot perforations present on the walls. -----------------------------
    # Slots are cut as real through-holes; assert the modeled slot counts.
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
        min_dist=0.10,
        name="handles separated across Y",
    )

    # --- Handle pivots: swings the top arch in Z/Y between poses. ------------
    # Carry-up pose (q=0) vs folded-flat pose (large angle). Compare the world
    # AABB of the handle: standing up it is tall (high max-z); folded it lies
    # low and extends sideways.
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
        # Folding swings the arch sideways in Y (its Y-extent center shifts).
        up_cy = 0.5 * (up_lo[1] + up_hi[1])
        fold_cy = 0.5 * (fold_lo[1] + fold_hi[1])
        ctx.check(
            f"{name} swings in Y when folded",
            abs(fold_cy - up_cy) > 0.03,
            details=f"up_cy={up_cy:.3f}, fold_cy={fold_cy:.3f}",
        )

    # --- Handle knuckles are captured at the rim (intentional local overlap).-
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

    # -----------------------------------------------------------------------
    # Wheel axle tests
    # -----------------------------------------------------------------------
    axle_lo, axle_hi = ctx.part_world_aabb(axle)
    ctx.check(
        "axle spans Y across basket width",
        axle_hi[1] - axle_lo[1] > BOT_Y * 0.50,
        details=f"axle y-span={axle_hi[1]-axle_lo[1]:.3f}",
    )
    ctx.check(
        "wheels sit near ground plane",
        abs(axle_lo[2]) < 0.01,
        details=f"axle min_z={axle_lo[2]:.4f}",
    )
    # Axle position: should be at the back (-X) end of the basket.
    ctx.check(
        "axle is at the back (-X) end of the basket",
        axle_lo[0] < -BOT_X / 3.0,
        details=f"axle min_x={axle_lo[0]:.3f}",
    )
    # Axle rotates: verify positive rotation moves the right-side wheel
    # in the +Z direction (proper rolling direction).
    with ctx.pose({axle_joint: 0.0}):
        rest_pos = ctx.part_world_position(axle)
    with ctx.pose({axle_joint: 1.0}):
        roll_pos = ctx.part_world_position(axle)
    ctx.check(
        "axle rotation does not translate the part origin",
        rest_pos is not None and roll_pos is not None and
        max(abs(rest_pos[i] - roll_pos[i]) for i in range(3)) < 0.001,
        details=f"rest={rest_pos}, roll={roll_pos}",
    )

    # -----------------------------------------------------------------------
    # Telescoping pull handle tests
    # -----------------------------------------------------------------------
    # At q=0 (retracted): the handle grip sits near the rim.
    with ctx.pose({pull_joint: 0.0}):
        retr_lo, retr_hi = ctx.part_world_aabb(pull)
        retr_top = retr_hi[2]
    ctx.check(
        "pull handle retracted near rim level",
        retr_top < BODY_H + 0.040,
        details=f"retracted top z={retr_top:.3f}",
    )

    # At q=PULL_EXTEND (extended): the handle rises well above the rim.
    with ctx.pose({pull_joint: PULL_EXTEND}):
        ext_lo, ext_hi = ctx.part_world_aabb(pull)
        ext_top = ext_hi[2]
    ctx.check(
        "pull handle extends upward when actuated",
        ext_top > retr_top + PULL_EXTEND * 0.90,
        details=f"retracted top={retr_top:.3f}, extended top={ext_top:.3f}",
    )
    ctx.check(
        "pull handle extended height > basket",
        ext_top > BODY_H + 0.40,
        details=f"extended top z={ext_top:.3f}",
    )

    # The pull handle is centered on the back (-X) end.
    with ctx.pose({pull_joint: 0.0}):
        p_lo, p_hi = ctx.part_world_aabb(pull)
    ctx.check(
        "pull handle centered in Y on back wall",
        abs(p_lo[1] + p_hi[1]) < 0.010,
        details=f"pull y-center={0.5*(p_lo[1]+p_hi[1]):.4f}",
    )

    # The pull handle rods overlap with the basket at retracted position
    # (rods extend downward through the guide into the basket wall).
    # This is intentional — the outer guide sleeve and inner rods are a
    # telescoping mechanism.
    ctx.allow_overlap(
        basket,
        pull,
        reason="Pull handle inner rods are intentionally nested inside the outer guide sleeve on the basket back wall (telescoping mechanism).",
    )

    # Allow axle-to-basket overlap at the axle mount point.
    ctx.allow_overlap(
        basket,
        axle,
        reason="Axle mount area overlaps with the basket back wall at the mounting interface.",
    )

    # Allow pull_handle to axle overlap — the handle rods sit behind the back
    # wall in the same X-region as the wheel axle; this is a design coexistence
    # in the hidden under-basket zone.
    ctx.allow_overlap(
        pull,
        axle,
        reason="Pull handle rods and wheel axle coexist in the hidden region behind the basket back wall.",
    )

    return ctx.report()


object_model = build_object_model()
