from __future__ import annotations

# Rectangular translucent frosted green plastic hand-held shopping basket with
# slotted/perforated side walls, a reinforced rolled top rim, molded grip ears
# on the short ends, a slightly tapered (stackable) body, and a single central
# telescoping pull-up handle: a U-shaped grip bar mounted on two vertical posts
# that slide vertically up out of the basket rim on a prismatic joint.
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

# Telescoping pull-up handle.
POST_SPAN = 0.120  # center-to-center spacing between the two posts (along X)
POST_R = 0.006  # post radius (12mm diameter posts)
POST_LENGTH = 0.230  # total post length (extends below rim when retracted)
BAR_R = 0.007  # grip bar cross-section radius
BAR_RISE = 0.030  # how far the U-bar rises above the post tops
HANDLE_EXTEND = 0.180  # prismatic travel (how far handle extends above rim)
GUIDE_R = 0.010  # guide boss outer radius on rim
GUIDE_H = 0.016  # guide boss height above rim top

# The articulation origin sits at the center between the two posts, at the
# top of the rim. At q=0 the handle is retracted (U-bar flush with rim top).
HANDLE_ORIGIN_Z = BODY_H + GUIDE_H  # top of guide bosses on the bridge

FROSTED_GREEN = (0.35, 0.68, 0.38, 0.65)
BLACK = (0.07, 0.07, 0.08, 1.0)


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
    rim flange, grip ears, tall vertical slot perforations, and two guide
    bosses on the rim for the telescoping handle posts."""
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

    # Rolled top rim: an outward flange band running around the mouth.
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

    # Reinforced bridge/platform across the mouth at center, spanning from the
    # front rim to the back rim. This provides a mounting surface for the
    # telescoping handle guide bosses. The bridge connects to the rim on both
    # long sides (+Y and -Y).
    bridge_x_half = POST_SPAN / 2.0 + GUIDE_R + 0.004  # bridge width in X
    bridge_y_half = TOP_Y / 2.0 - WALL_T  # extends to inner rim edge
    bridge_t = 0.008  # bridge thickness in Z
    bridge_z = BODY_H - bridge_t  # bridge sits just below rim top

    bridge = (
        cq.Workplane("XY")
        .box(bridge_x_half * 2.0, bridge_y_half * 2.0, bridge_t)
        .edges("|Z")
        .fillet(0.005)
        .translate((0.0, 0.0, bridge_z + bridge_t / 2.0))
    )
    # Cut post holes through the bridge
    for sx in (1.0, -1.0):
        hole = (
            cq.Workplane("XY")
            .circle(POST_R + 0.001)
            .extrude(bridge_t + 0.004)
            .translate((sx * POST_SPAN / 2.0, 0.0, bridge_z - 0.002))
        )
        bridge = bridge.cut(hole)
    tub = tub.union(bridge)

    # Guide bosses on the bridge: two short cylindrical sleeves where the
    # telescoping handle posts pass through. Embedded into the bridge for
    # connectivity.
    boss_embed = 0.006
    for sx in (1.0, -1.0):
        boss_outer = (
            cq.Workplane("XY")
            .circle(GUIDE_R)
            .extrude(GUIDE_H + boss_embed)
            .translate((sx * POST_SPAN / 2.0, 0.0, bridge_z + bridge_t - boss_embed))
        )
        boss_inner = (
            cq.Workplane("XY")
            .circle(POST_R + 0.001)
            .extrude(GUIDE_H + boss_embed + 0.004)
            .translate((sx * POST_SPAN / 2.0, 0.0, bridge_z + bridge_t - boss_embed - 0.002))
        )
        boss = boss_outer.cut(boss_inner)
        tub = tub.union(boss)

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


def _build_telescoping_handle():
    """A U-shaped pull handle on two vertical posts.

    Authored in the handle-local frame whose origin sits at the articulation
    point: midway between the two posts at the rim top level (z=0 in local).
    At q=0 (retracted), the U-bar is at rim level and posts extend downward
    into the basket body. At q=HANDLE_EXTEND, the whole assembly rises.

    The two posts are at (+-POST_SPAN/2, 0, *) in local X. They extend from
    z=0 downward to z=-POST_LENGTH. The U-bar connects the post tops and
    rises slightly above z=0.
    """
    half_span = POST_SPAN / 2.0

    # Two vertical cylindrical posts, extending from z=0 down to z=-POST_LENGTH.
    handle = cq.Workplane("XY")  # start empty via a dummy
    # Build posts
    left_post = (
        cq.Workplane("XY")
        .circle(POST_R)
        .extrude(POST_LENGTH)
        .translate((-half_span, 0.0, -POST_LENGTH))
    )
    right_post = (
        cq.Workplane("XY")
        .circle(POST_R)
        .extrude(POST_LENGTH)
        .translate((half_span, 0.0, -POST_LENGTH))
    )

    # U-shaped grip bar connecting the two post tops.
    # The bar is a horizontal tube that goes from one post up and over to the
    # other, forming a U-shape in the X-Z plane. The bar centerline goes:
    # from (-half_span, 0, 0) up to (-half_span, 0, BAR_RISE), across to
    # (+half_span, 0, BAR_RISE), then down to (+half_span, 0, 0).
    # We'll sweep a circle along this path.
    bar_path = (
        cq.Workplane("XZ")
        .moveTo(-half_span, 0.0)
        .lineTo(-half_span, BAR_RISE - BAR_R)
        .radiusArc((half_span, BAR_RISE - BAR_R), half_span)
        .lineTo(half_span, 0.0)
    )
    # Use a simpler approach: build the U-bar from cylinders and a curved section.
    # Vertical stubs at each end
    left_stub = (
        cq.Workplane("XY")
        .circle(BAR_R)
        .extrude(BAR_RISE)
        .translate((-half_span, 0.0, 0.0))
    )
    right_stub = (
        cq.Workplane("XY")
        .circle(BAR_R)
        .extrude(BAR_RISE)
        .translate((half_span, 0.0, 0.0))
    )
    # Horizontal bar connecting the stub tops
    horiz_bar = (
        cq.Workplane("YZ")
        .circle(BAR_R)
        .extrude(POST_SPAN, both=True)
        .translate((0.0, 0.0, BAR_RISE))
    )

    # Small fillet spheres at the corners for smooth transitions
    corner_r = BAR_R * 1.1
    left_corner = (
        cq.Workplane("XY")
        .sphere(corner_r)
        .translate((-half_span, 0.0, BAR_RISE))
    )
    right_corner = (
        cq.Workplane("XY")
        .sphere(corner_r)
        .translate((half_span, 0.0, BAR_RISE))
    )

    # Combine everything
    handle = left_post.union(right_post)
    handle = handle.union(left_stub).union(right_stub).union(horiz_bar)
    handle = handle.union(left_corner).union(right_corner)

    # Small end caps at the bottom of posts (rounded ends)
    for sx in (1.0, -1.0):
        cap = (
            cq.Workplane("XY")
            .sphere(POST_R)
            .translate((sx * half_span, 0.0, -POST_LENGTH))
        )
        handle = handle.union(cap)

    return handle


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="green_shopping_basket")

    green = model.material("frosted_green", rgba=FROSTED_GREEN)
    black = model.material("handle_black", rgba=BLACK)

    # Root: the hollow basket tub.
    basket = model.part("basket_tub")
    basket.visual(mesh_from_cadquery(_build_body(), "basket_tub"), material=green)
    basket.inertial = Inertial.from_geometry(
        Box((TOP_X, TOP_Y, BODY_H)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # Single telescoping pull-up handle.
    handle = model.part("pull_handle")
    handle.visual(
        mesh_from_cadquery(_build_telescoping_handle(), "pull_handle"),
        material=black,
    )
    handle.inertial = Inertial.from_geometry(
        Box((POST_SPAN, 2.0 * BAR_R, POST_LENGTH + BAR_RISE)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, -POST_LENGTH / 2.0 + BAR_RISE / 2.0)),
    )

    # Prismatic joint: handle slides vertically (+Z) out of the basket rim.
    # At q=0 the handle is retracted (U-bar at rim level).
    # At q=HANDLE_EXTEND the handle is fully extended for carrying.
    model.articulation(
        "tub_to_handle",
        ArticulationType.PRISMATIC,
        parent=basket,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, HANDLE_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.5,
            lower=0.0,
            upper=HANDLE_EXTEND,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    basket = object_model.get_part("basket_tub")
    handle = object_model.get_part("pull_handle")
    slide = object_model.get_articulation("tub_to_handle")

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
    ctx.check(
        "tub spans full outer mouth in XY",
        width_x > TOP_X - 0.001 and depth_y > TOP_Y - 0.001,
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

    # --- Single central handle (not two side handles). -----------------------
    handle_pos = ctx.part_world_position(handle)
    ctx.check(
        "handle centered on basket (X)",
        handle_pos is not None and abs(handle_pos[0]) < 0.02,
        details=f"handle_x={handle_pos[0]:.4f}" if handle_pos else "no position",
    )
    ctx.check(
        "handle centered on basket (Y)",
        handle_pos is not None and abs(handle_pos[1]) < 0.02,
        details=f"handle_y={handle_pos[1]:.4f}" if handle_pos else "no position",
    )

    # --- Prismatic slide: handle extends upward when actuated. ---------------
    with ctx.pose({slide: 0.0}):
        retracted_lo, retracted_hi = ctx.part_world_aabb(handle)
        retracted_top = retracted_hi[2]

    with ctx.pose({slide: HANDLE_EXTEND}):
        extended_lo, extended_hi = ctx.part_world_aabb(handle)
        extended_top = extended_hi[2]

    ctx.check(
        "handle extends upward when pulled",
        extended_top > retracted_top + HANDLE_EXTEND - 0.01,
        details=f"retracted_top={retracted_top:.3f}, extended_top={extended_top:.3f}",
    )

    # At retracted pose, the U-bar should be near the rim top level.
    ctx.check(
        "handle retracted near rim level",
        abs(retracted_top - HANDLE_ORIGIN_Z - BAR_RISE) < 0.02,
        details=f"retracted_top={retracted_top:.3f}, expected~{HANDLE_ORIGIN_Z + BAR_RISE:.3f}",
    )

    # --- Posts remain partially inserted when fully extended. ----------------
    # At full extension, the post bottoms should still be above the basket floor.
    with ctx.pose({slide: HANDLE_EXTEND}):
        ext_lo, ext_hi = ctx.part_world_aabb(handle)
        post_bottom = ext_lo[2]
    ctx.check(
        "posts still partially inserted at full extension",
        post_bottom > 0.0 and post_bottom < BODY_H,
        details=f"post_bottom={post_bottom:.3f}",
    )

    # --- Handle posts pass through guide bosses (intentional overlap). -------
    ctx.allow_overlap(
        basket,
        handle,
        reason="Handle posts slide through guide bosses on the rim; the posts are intentionally nested inside the guide sleeves.",
    )

    # Prove the handle is retained: posts overlap the basket vertically at
    # both rest and extended poses.
    with ctx.pose({slide: 0.0}):
        ctx.expect_overlap(
            handle,
            basket,
            axes="z",
            min_overlap=0.01,
            name="retracted handle posts overlap basket in Z",
        )
    with ctx.pose({slide: HANDLE_EXTEND}):
        ctx.expect_overlap(
            handle,
            basket,
            axes="z",
            min_overlap=0.01,
            name="extended handle posts still overlap basket in Z",
        )

    return ctx.report()


object_model = build_object_model()
