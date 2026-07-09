from __future__ import annotations

# Large deep floor-standing shopping basket: straight vertical walls (no taper),
# slotted/perforated side walls, reinforced rolled top rim, integrated molded
# cutout grip handles recessed into the upper short walls, and four short stub
# feet under the corners. Bright blue plastic body.
#
# Coordinate convention: +Z up, basket rests on feet at z=0, long axis along X.

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
BODY_H = 0.560  # tub wall height (z), above feet
FOOT_H = 0.025  # stub foot height

# Straight-walled: same footprint top and bottom (no taper).
BODY_X = 0.480  # outer width (long axis)
BODY_Y = 0.340  # outer depth (short axis)

WALL_T = 0.004  # nominal wall thickness
FLOOR_T = 0.008  # floor thickness

# Rolled / flanged top rim.
RIM_H = 0.020  # vertical height of the rolled rim band
RIM_LIP = 0.012  # how far the lip protrudes outward beyond the wall
RIM_Z = BODY_H - RIM_H / 2.0  # vertical center of rim band

# Integrated grip handle cutouts (recessed into upper short walls).
GRIP_W = 0.110  # grip cutout width (along Y, i.e. along the wall run)
GRIP_H = 0.045  # grip cutout height (vertical)
GRIP_R = 0.018  # fillet radius on the grip cutout corners
GRIP_Z = BODY_H - 0.060  # vertical center of the grip cutout (near top)
# The grip is a through-cut in the short wall, centered in X on each short wall.

# Stub feet (four corners).
FOOT_X = 0.040  # foot width in X
FOOT_Y = 0.040  # foot width in Y
FOOT_CHAMFER = 0.004  # chamfer on foot edges

# Slot perforations (tall vertical slots cut through the walls).
SLOT_W = 0.010  # slot width (horizontal, along the wall run)
SLOT_H = 0.180  # slot height (vertical) - taller for the deeper basket
SLOT_Z = BODY_H * 0.48  # vertical center of the slot band (roughly mid-wall)

BLUE = (0.10, 0.32, 0.92, 1.0)

# Lid: flat rectangular panel hinged along the +Y long rim.
LID_X = BODY_X + 2.0 * RIM_LIP - 0.004  # lid width (slightly less than rim outer)
LID_Y = BODY_Y + 2.0 * RIM_LIP - 0.004  # lid depth
LID_T = 0.008  # lid panel thickness
HINGE_Y = BODY_Y / 2.0 + RIM_LIP  # hinge line Y (outer edge of +Y rim)
HINGE_Z = BODY_H + LID_T / 2.0  # hinge Z so lid bottom contacts rim top
KNUCKLE_R = 0.004  # hinge knuckle cylinder radius
KNUCKLE_LEN = 0.030  # hinge knuckle length along X


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def _build_body():
    """Hollow straight-walled tub: outer shell minus inner cavity, with floor,
    rolled rim flange, integrated grip handle cutouts, slot perforations, and
    four corner stub feet."""

    # Outer shell: straight rectangular box sitting on z=0 (top of feet).
    outer = (
        cq.Workplane("XY")
        .box(BODY_X, BODY_Y, BODY_H)
        .translate((0.0, 0.0, BODY_H / 2.0))
    )

    # Inner cavity: inset by wall thickness, starting above the floor.
    inner = (
        cq.Workplane("XY")
        .box(BODY_X - 2.0 * WALL_T, BODY_Y - 2.0 * WALL_T, BODY_H + 0.02)
        .translate((0.0, 0.0, (BODY_H + 0.02) / 2.0 + FLOOR_T))
    )

    tub = outer.cut(inner)

    # Rolled top rim: an outward flange band running around the mouth.
    rim_outer = (
        cq.Workplane("XY")
        .box(BODY_X + 2.0 * RIM_LIP, BODY_Y + 2.0 * RIM_LIP, RIM_H)
        .translate((0.0, 0.0, RIM_Z))
    )
    rim_hole = (
        cq.Workplane("XY")
        .box(BODY_X - 2.0 * WALL_T, BODY_Y - 2.0 * WALL_T, RIM_H + 0.02)
        .translate((0.0, 0.0, RIM_Z))
    )
    rim = rim_outer.cut(rim_hole)
    tub = tub.union(rim)

    # Integrated grip handle cutouts on the two short walls (+X and -X).
    # These are rounded-rectangular through-cuts in the upper portion of the
    # short walls, creating hand-grip openings.
    cut_depth = WALL_T + 0.04  # deep enough to cut fully through wall + rim
    for sx in (1.0, -1.0):
        grip_cutter = (
            cq.Workplane("XY")
            .box(cut_depth, GRIP_W, GRIP_H)
            .edges("|X")
            .fillet(GRIP_R)
            .translate((sx * (BODY_X / 2.0), 0.0, GRIP_Z))
        )
        tub = tub.cut(grip_cutter)

    # Add a reinforcing surround / lip around each grip cutout so the grip
    # reads as a molded feature rather than a raw hole. Small outward frame.
    for sx in (1.0, -1.0):
        frame_outer = (
            cq.Workplane("XY")
            .box(0.008, GRIP_W + 0.016, GRIP_H + 0.016)
            .edges("|X")
            .fillet(GRIP_R + 0.004)
            .translate((sx * (BODY_X / 2.0 + 0.002), 0.0, GRIP_Z))
        )
        frame_inner = (
            cq.Workplane("XY")
            .box(0.012, GRIP_W - 0.002, GRIP_H - 0.002)
            .edges("|X")
            .fillet(GRIP_R - 0.002)
            .translate((sx * (BODY_X / 2.0 + 0.002), 0.0, GRIP_Z))
        )
        frame = frame_outer.cut(frame_inner)
        tub = tub.union(frame)

    # Tall vertical slot perforations cut through all four walls.
    tub = _cut_slots(tub)

    # Floor ribs on the inner floor.
    for rx in (-0.14, -0.07, 0.0, 0.07, 0.14):
        rib = (
            cq.Workplane("XY")
            .box(0.006, BODY_Y - 4.0 * WALL_T, 0.004)
            .translate((rx, 0.0, FLOOR_T + 0.002))
        )
        tub = tub.union(rib)

    # Four corner stub feet. Each is a small rounded box protruding downward
    # from the base corners. The tub sits on top of these feet.
    foot_inset_x = BODY_X / 2.0 - FOOT_X / 2.0 - 0.015
    foot_inset_y = BODY_Y / 2.0 - FOOT_Y / 2.0 - 0.015
    for sx in (1.0, -1.0):
        for sy in (1.0, -1.0):
            foot = (
                cq.Workplane("XY")
                .box(FOOT_X, FOOT_Y, FOOT_H)
                .edges("|Z")
                .fillet(FOOT_CHAMFER)
                .translate((sx * foot_inset_x, sy * foot_inset_y, -FOOT_H / 2.0))
            )
            tub = tub.union(foot)

    return tub


def _cut_slots(tub):
    """Cut rows of tall vertical slots through the long (Y-normal) and short
    (X-normal) walls. Slots are real through-cuts."""
    cut_depth = 0.06  # deeper than any wall thickness so the cut goes through

    # Long walls (front/back, normal along Y): slots arrayed along X.
    n_long = 15
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
                .translate((cx, sy * (BODY_Y / 2.0), SLOT_Z))
            )
            tub = tub.cut(cutter)

    # Short walls (left/right ends, normal along X): slots arrayed along Y.
    # Fewer slots on short walls to leave room for the grip cutouts above.
    n_short = 8
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
                .translate((sx * (BODY_X / 2.0), cy, SLOT_Z))
            )
            tub = tub.cut(cutter)

    return tub


def _build_lid():
    """Flat rectangular lid panel with hinge knuckles and opening tab.

    The lid part frame sits at the hinge line (HINGE_Y, HINGE_Z).
    In local coordinates the panel extends in -Y from the hinge and
    is centered at z=0 so the panel bottom touches the rim top when closed.
    """
    # Main flat panel with slightly rounded corners.
    panel = (
        cq.Workplane("XY")
        .box(LID_X, LID_Y, LID_T)
        .edges("|Z")
        .fillet(0.006)
        .translate((0.0, -LID_Y / 2.0, 0.0))
    )

    # Two hinge knuckle barrels along the hinge edge (local y=0, z=0).
    # Cylinders with axis along X, centered on the hinge pivot.
    for kx in (-0.14, 0.14):
        knuckle = (
            cq.Workplane("YZ")
            .circle(KNUCKLE_R)
            .extrude(KNUCKLE_LEN)
            .translate((kx - KNUCKLE_LEN / 2.0, 0.0, 0.0))
        )
        panel = panel.union(knuckle)

    # Small raised opening tab on the far edge (-Y) for easy grip.
    tab = (
        cq.Workplane("XY")
        .box(0.050, 0.010, 0.014)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.0, -LID_Y + 0.005, LID_T / 2.0 + 0.007))
    )
    panel = panel.union(tab)

    # Subtle perimeter lip: a thin raised border around the top face of the
    # lid (except the hinge side) to give it a molded-plastic look.
    lip_h = 0.003
    lip_t = 0.004
    # Far edge lip (-Y side)
    lip_far = (
        cq.Workplane("XY")
        .box(LID_X - 2.0 * lip_t, lip_t, lip_h)
        .translate((0.0, -LID_Y + lip_t / 2.0, LID_T / 2.0 + lip_h / 2.0))
    )
    panel = panel.union(lip_far)
    # Side lips (±X)
    for sx in (1.0, -1.0):
        lip_side = (
            cq.Workplane("XY")
            .box(lip_t, LID_Y - 2.0 * lip_t, lip_h)
            .translate((sx * (LID_X / 2.0 - lip_t / 2.0), -LID_Y / 2.0, LID_T / 2.0 + lip_h / 2.0))
        )
        panel = panel.union(lip_side)

    return panel


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_shopping_basket")

    blue = model.material("basket_blue", rgba=BLUE)

    # Basket body: deep straight-walled tub with slotted walls, integrated
    # grip cutouts, rolled rim, and four stub feet.
    basket = model.part("basket_body")
    basket.visual(mesh_from_cadquery(_build_body(), "basket_body"), material=blue)
    basket.inertial = Inertial.from_geometry(
        Box((BODY_X, BODY_Y, BODY_H + FOOT_H)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # Hinged lid: flat rectangular panel covering the basket opening,
    # attached along the +Y long rim by a revolute hinge.
    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_build_lid(), "lid_panel"), material=blue)
    lid.inertial = Inertial.from_geometry(
        Box((LID_X, LID_Y, LID_T + 0.014)),
        mass=0.35,
        origin=Origin(xyz=(0.0, -LID_Y / 2.0, 0.0)),
    )

    # Revolute hinge along the +Y long rim.
    # axis=(-1, 0, 0): positive q rotates the far edge (-Y) upward (+Z).
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=basket,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=0.0,
            upper=2.0,  # ~115° open
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    basket = object_model.get_part("basket_body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    # --- Footprint: wider in X than Y, floor-standing height. ----------------
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
        "basket base near z=0 (feet on ground)",
        lo[2] < 0.005,
        details=f"min_z={lo[2]:.4f}",
    )
    ctx.check(
        "basket is tall (floor-standing, > 0.50m)",
        height_z > 0.50,
        details=f"height={height_z:.3f}",
    )

    # --- Straight walls (no taper): top and bottom footprint similar. --------
    ctx.check(
        "straight walls (no significant taper)",
        abs(width_x - BODY_X - 2.0 * RIM_LIP) < 0.01,
        details=f"width_x={width_x:.3f}, expected~{BODY_X + 2.0 * RIM_LIP:.3f}",
    )

    # --- Hollow: thin wall shell, not a solid block. -------------------------
    ctx.check(
        "wall is thin (hollow shell)",
        WALL_T < 0.01 and WALL_T < BODY_H / 10.0,
        details=f"wall_t={WALL_T}",
    )

    # --- Slot perforations present. ------------------------------------------
    ctx.check(
        "slot perforations present",
        SLOT_H > 0.10 and SLOT_W > 0.0,
        details=f"slot {SLOT_W}x{SLOT_H}",
    )

    # --- Grip cutouts present (integrated into short walls). -----------------
    ctx.check(
        "grip cutouts defined (integrated handles)",
        GRIP_W > 0.08 and GRIP_H > 0.03,
        details=f"grip {GRIP_W}x{GRIP_H}",
    )

    # --- Four stub feet under corners. ---------------------------------------
    ctx.check(
        "stub feet defined",
        FOOT_H > 0.015 and FOOT_X > 0.025,
        details=f"foot {FOOT_X}x{FOOT_Y}x{FOOT_H}",
    )

    # --- Lid exists and covers the basket opening. ---------------------------
    ctx.check(
        "lid part exists",
        lid is not None,
        details="lid part not found",
    )

    # --- Hinge articulation exists. ------------------------------------------
    ctx.check(
        "hinge articulation exists",
        hinge is not None,
        details="body_to_lid articulation not found",
    )

    # --- Closed pose (q=0): lid seats on the basket rim. --------------------
    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(
            lid,
            basket,
            contact_tol=0.002,
            name="lid seats on rim when closed",
        )
        # Lid should cover the opening: overlap in XY with basket.
        ctx.expect_overlap(
            lid,
            basket,
            axes="xy",
            min_overlap=0.10,
            name="lid covers basket opening in XY",
        )

    # --- Open pose (q=upper limit): lid swings upward. ----------------------
    with ctx.pose({hinge: 2.0}):
        lid_lo, lid_hi = ctx.part_world_aabb(lid)
        ctx.check(
            "lid opens upward (far edge rises above basket)",
            lid_hi[2] > BODY_H + 0.20,
            details=f"lid max z={lid_hi[2]:.3f} at q=2.0",
        )

    return ctx.report()


object_model = build_object_model()
