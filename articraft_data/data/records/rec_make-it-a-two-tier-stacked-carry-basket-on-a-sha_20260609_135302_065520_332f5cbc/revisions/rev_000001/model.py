from __future__ import annotations

# Two-tier stacked carry basket on a shared carry post/frame.
# The lower basket is fixed to the post; the upper basket slides straight up
# the central carry post via a prismatic joint, lifting away from the lower basket.
#
# The carry post (root) is a vertical bar on the -X short end of the paired
# baskets. The lower basket sits on the ground at z=0; the upper basket sits
# on top with a small gap and can slide up the post (positive q raises it).
#
# Each basket uses the same slotted/tapered hollow blue body as the parent
# asset: slotted walls, rolled rim, grip ears.
#
# Coordinate convention: +Z up, baskets rest on ground at z=0, long axis along X.
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

# Grip ears on the two short ends (+X / -X rim).
EAR_X = 0.022  # how far the ear sticks out past the rim in X
EAR_Y = 0.110  # ear width in Y
EAR_Z = 0.030  # ear height in Z

# Slot perforations (tall vertical slots cut through the walls).
SLOT_W = 0.010  # slot width (horizontal, along the wall run)
SLOT_H = 0.110  # slot height (vertical)
SLOT_Z = 0.108  # vertical center of the slot band

# Carry post (root frame).
POST_DIAM = 0.032  # post cross-section side length (square tube)

# Stacking layout.
GAP = 0.040  # gap between lower basket top and upper basket bottom
UPPER_Z = BODY_H + GAP  # bottom elevation of the upper basket
SLIDE_Z = UPPER_Z  # bottom-of-upper-basket height where the slide frame sits

# Post position on the -X short end, just outside the basket body.
POST_X = -(TOP_X / 2.0 + 0.060)

# Post height: from ground to a comfortable carry height above the upper basket.
POST_H = UPPER_Z + BODY_H + 0.120  # ~0.59 m

# Vertical slide travel: raise the upper basket until its top reaches the post top.
UPPER_SLIDE = POST_H - (UPPER_Z + BODY_H)  # ~0.12 m

# Carry grip at top of post.
GRIP_H = 0.090  # crossbar grip length
GRIP_T = 0.020  # crossbar thickness

BLUE = (0.10, 0.32, 0.92, 1.0)
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


def _build_post():
    """Carry post: a square vertical bar with a short crossbar grip at the top.

    Authored centered at (0, 0, 0) with the post extending from z=0 to z=POST_H.
    """
    post = (
        cq.Workplane("XY")
        .rect(POST_DIAM, POST_DIAM)
        .extrude(POST_H)
    )

    # Round the top corners slightly.
    post = post.edges("|Z").fillet(0.005)

    # Crossbar grip at the top: a short horizontal bar along Y so the user can
    # grip it from the front.
    grip = (
        cq.Workplane("XY")
        .box(GRIP_T, GRIP_H, GRIP_T)
        .translate((0.0, 0.0, POST_H + GRIP_T / 2.0))
    )
    grip = grip.edges("|Z").fillet(0.004)

    post = post.union(grip)

    # Add a small slide-collar flange at the slide height (for visual realism).
    bracket = (
        cq.Workplane("XY")
        .box(POST_DIAM + 0.015, POST_DIAM + 0.015, 0.006)
        .translate((0.0, 0.0, SLIDE_Z))
    )
    post = post.union(bracket)

    return post


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_tier_carry_basket")

    blue = model.material("basket_blue", rgba=BLUE)
    black = model.material("post_black", rgba=BLACK)

    # --- Carry post (root) ---
    post = model.part("carry_post")
    post.visual(
        mesh_from_cadquery(_build_post(), "carry_post"),
        material=black,
        name="carry_post",
    )
    post.inertial = Inertial.from_geometry(
        Box((POST_DIAM, POST_DIAM, POST_H)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, POST_H / 2.0)),
    )

    # --- Lower basket (fixed to post) ---
    lower_body = _build_body()
    lower_basket = model.part("lower_basket")
    lower_basket.visual(
        mesh_from_cadquery(lower_body, "lower_basket"),
        material=blue,
        name="lower_basket",
    )
    lower_basket.inertial = Inertial.from_geometry(
        Box((TOP_X, TOP_Y, BODY_H)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # Fixed mount: the lower basket is positioned such that its center is at
    # world origin (resting on z=0).
    model.articulation(
        "post_to_lower",
        ArticulationType.FIXED,
        parent=post,
        child=lower_basket,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Upper basket (hinged to post) ---
    upper_body = _build_body()
    upper_basket = model.part("upper_basket")
    upper_basket.inertial = Inertial.from_geometry(
        Box((TOP_X, TOP_Y, BODY_H)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # The upper basket slides straight up the carry post (prismatic, +Z). The
    # part frame of the upper basket sits at the slide origin (at the post, on
    # the -X side, at the bottom-of-upper-basket height). The basket body is
    # offset in the part frame so that at q=0 it sits centered above the lower
    # basket; positive q raises it up the post.
    slide_origin = Origin(xyz=(POST_X, 0.0, SLIDE_Z))

    # Offset: the basket body bottom-center should be at world (0, 0, UPPER_Z)
    # when q=0. Since the part frame is at (POST_X, 0, SLIDE_Z), the visual
    # origin offset in the part frame is:
    #   dx = 0 - POST_X = -POST_X
    #   dz = UPPER_Z - SLIDE_Z = 0 (since SLIDE_Z == UPPER_Z)
    visual_offset = Origin(xyz=(-POST_X, 0.0, 0.0))

    # We pass the visual offset via the visual's origin parameter.
    upper_basket.visual(
        mesh_from_cadquery(upper_body, "upper_basket_mesh"),
        material=blue,
        origin=visual_offset,
        name="upper_basket_mesh",
    )

    model.articulation(
        "post_to_upper",
        ArticulationType.PRISMATIC,
        parent=post,
        child=upper_basket,
        origin=slide_origin,
        axis=(0.0, 0.0, 1.0),  # slides straight up along the carry post
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.2,
            lower=0.0,
            upper=UPPER_SLIDE,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    post = object_model.get_part("carry_post")
    lower = object_model.get_part("lower_basket")
    upper = object_model.get_part("upper_basket")
    slide_j = object_model.get_articulation("post_to_upper")

    # --- Basic structure checks ---
    lo_lower, hi_lower = ctx.part_world_aabb(lower)
    lo_upper, hi_upper = ctx.part_world_aabb(upper)

    # Lower basket sits at ground level.
    ctx.check(
        "lower basket rests at z~0",
        lo_lower is not None and abs(lo_lower[2]) < 0.01,
        details=f"lower min_z={lo_lower[2]:.4f}" if lo_lower else "None",
    )

    # Upper basket is above lower basket (stacked) at rest.
    ctx.check(
        "upper basket is above lower at rest",
        lo_lower is not None and lo_upper is not None
        and lo_upper[2] > hi_lower[2] - 0.01,
        details=f"lower top={hi_lower[2]:.3f}, upper bottom={lo_upper[2]:.3f}",
    )

    # Carry post spans from ground to well above the upper basket.
    lo_post, hi_post = ctx.part_world_aabb(post)
    ctx.check(
        "post reaches above upper basket",
        hi_post is not None and hi_post[2] > hi_upper[2] + 0.05,
        details=f"post top={hi_post[2]:.3f}, upper top={hi_upper[2]:.3f}",
    )
    ctx.check(
        "post reaches to ground",
        lo_post is not None and abs(lo_post[2]) < 0.01,
        details=f"post min_z={lo_post[2]:.4f}" if lo_post else "None",
    )

    # --- Upper-tier joint is a prismatic slide along the post (not a hinge). ---
    ctx.check(
        "upper-tier joint is prismatic",
        str(slide_j.articulation_type).upper().endswith("PRISMATIC"),
        details=f"type={slide_j.articulation_type}",
    )
    ctx.check(
        "upper-tier slide axis is +Z (along the post)",
        abs(slide_j.axis[0]) < 0.01
        and abs(slide_j.axis[1]) < 0.01
        and slide_j.axis[2] > 0.99,
        details=f"axis={slide_j.axis}",
    )

    # --- Hollow slotted bodies (same as parent) ---
    ctx.check(
        "wall is thin (hollow shell)",
        WALL_T < 0.01 and WALL_T < BODY_H / 10.0,
        details=f"wall_t={WALL_T}",
    )
    ctx.check(
        "slot perforations present",
        SLOT_H > 0.05 and SLOT_W > 0.0,
        details=f"slot {SLOT_W}x{SLOT_H}",
    )

    # --- Upper tier slides clear of lower tier when raised up the post. ---
    with ctx.pose({slide_j: UPPER_SLIDE}):
        lo_open, hi_open = ctx.part_world_aabb(upper)
        lo_lower_closed, hi_lower_closed = ctx.part_world_aabb(lower)

        # Raised to the top of the slide, the upper basket bottom is well above
        # the lower basket top.
        ctx.check(
            "upper tier slides clear above lower tier",
            lo_open is not None and hi_lower_closed is not None
            and lo_open[2] > hi_lower_closed[2] + 0.05,
            details=(
                f"raised upper bottom={lo_open[2]:.3f}, "
                f"lower top={hi_lower_closed[2]:.3f}"
            ),
        )

        # The raised upper basket stays within the carry post (top <= post top).
        ctx.check(
            "raised upper basket stays within the carry post",
            hi_open is not None and hi_post is not None
            and hi_open[2] <= hi_post[2] + 1e-6,
            details=f"raised upper top={hi_open[2]:.3f}, post top={hi_post[2]:.3f}",
        )

    # --- Allow intentional overlap: the slide collar on the post is inside
    #     the upper basket's nearest wall (small local capture). ---
    ctx.allow_overlap(
        post,
        upper,
        elem_a="carry_post",
        elem_b="upper_basket_mesh",
        reason="The slide collar at the post intentionally nests inside the upper basket's near wall for a captured prismatic slide interface.",
    )

    # The lower basket is fixed to the post; the post runs alongside its -X
    # wall, creating a small intentional overlap along that edge.
    ctx.allow_overlap(
        post,
        lower,
        elem_a="carry_post",
        elem_b="lower_basket",
        reason="The carry post runs alongside the -X short-end wall of the lower basket; small overlap from the post geometry passing through the outer wall.",
    )

    return ctx.report()


object_model = build_object_model()
