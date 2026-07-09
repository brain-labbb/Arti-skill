from __future__ import annotations

# Rectangular (oblong) cast-iron utility access cover seated in a matching
# rectangular recessed frame. The cover, frame opening, and shaft throat are all
# elongated rectangles (trench-style, clearly longer than wide). The heavy cast
# cover lifts straight up out of the rectangular recessed frame (prismatic +Z) to
# reveal the rectangular shaft void below. A raised perimeter rim and a center
# pry slot detail the top face.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Real-world dimensions (meters). A trench-style rectangular cast-iron cover.
# ---------------------------------------------------------------------------
COVER_LENGTH = 0.90  # long plan dimension (X axis)
COVER_WIDTH = 0.45   # short plan dimension (Y axis); 2:1 aspect
COVER_THICK = 0.065  # heavy cast-iron slab thickness
COVER_TOP_BEVEL = 0.012  # chamfer on the top perimeter edge

# Raised perimeter rim on the cover top face.
RIM_HEIGHT = 0.010   # how far the rim stands above the main top face
RIM_WIDTH = 0.025    # rim band width measured inward from the edge

SLOT_LEN = 0.120     # center pry slot length (along long axis)
SLOT_WID = 0.024     # center pry slot width
SLOT_DEPTH = 0.028   # pry slot recess depth (does not pierce the slab)

# Frame ring: a chunky recessed seating frame around the rectangular cover.
FRAME_SEAT_GAP = 0.012  # perimeter gap between cover edge and frame inner wall
FRAME_WALL = 0.100      # width of the visible frame band
FRAME_INNER_L = COVER_LENGTH + 2.0 * FRAME_SEAT_GAP  # frame opening length
FRAME_INNER_W = COVER_WIDTH + 2.0 * FRAME_SEAT_GAP   # frame opening width
FRAME_OUTER_L = FRAME_INNER_L + 2.0 * FRAME_WALL     # frame outer length
FRAME_OUTER_W = FRAME_INNER_W + 2.0 * FRAME_WALL     # frame outer width
FRAME_HEIGHT = 0.140   # total frame depth (top band + downstand)
FRAME_TOP_BAND = 0.050  # height of the visible top frame band above seat ledge

# Seat ledge: the cover rests on a recessed ledge inside the frame.
SEAT_LEDGE = 0.028   # how far the ledge projects inward under the cover
SEAT_DROP = COVER_THICK + RIM_HEIGHT - 0.008  # cover+rim top sits ~8mm proud
SEAT_LEDGE_TOP_Z = FRAME_HEIGHT - SEAT_DROP
SEAT_EMBED = 0.003   # cover seats a few mm into the ledge so it reads as seated
COVER_REST_BOTTOM_Z = SEAT_LEDGE_TOP_Z - SEAT_EMBED
COVER_REST_TOP_Z = COVER_REST_BOTTOM_Z + COVER_THICK + RIM_HEIGHT

# Shaft void revealed when the cover is lifted.
SHAFT_LENGTH = FRAME_INNER_L - 2.0 * SEAT_LEDGE
SHAFT_WIDTH = FRAME_INNER_W - 2.0 * SEAT_LEDGE
SHAFT_DEPTH = 0.30

# Prismatic lift travel.
LIFT_CLEARANCE = FRAME_TOP_BAND + 0.06
LIFT_TRAVEL = SEAT_DROP + LIFT_CLEARANCE


def _frame_solid() -> cq.Workplane:
    """Rectangular recessed seating frame: outer block, hollow rectangular opening,
    inner seat ledge, and a downstand throat that becomes the visible shaft void."""
    half_h = FRAME_HEIGHT / 2.0
    # Outer block, base at z=0.
    frame = (
        cq.Workplane("XY")
        .box(FRAME_OUTER_L, FRAME_OUTER_W, FRAME_HEIGHT)
        .translate((0.0, 0.0, half_h))
    )
    # Upper bore: the cover-receiving rectangular recess cut from above the frame
    # top down exactly to the seat ledge top face at SEAT_LEDGE_TOP_Z.
    recess_top = FRAME_HEIGHT + 0.02
    recess_bot = SEAT_LEDGE_TOP_Z
    recess_h = recess_top - recess_bot
    recess = (
        cq.Workplane("XY")
        .box(FRAME_INNER_L, FRAME_INNER_W, recess_h)
        .translate((0.0, 0.0, (recess_top + recess_bot) / 2.0))
    )
    frame = frame.cut(recess)
    # Lower bore: the rectangular shaft throat, cut all the way through.
    throat = (
        cq.Workplane("XY")
        .box(SHAFT_LENGTH, SHAFT_WIDTH, FRAME_HEIGHT + 0.04)
        .translate((0.0, 0.0, FRAME_HEIGHT / 2.0))
    )
    frame = frame.cut(throat)
    # Soften visible top outer perimeter edges (weathered/worn cast frame).
    frame = frame.faces(">Z").edges().chamfer(0.008)
    return frame


def _shaft_solid() -> cq.Workplane:
    """Hollow rectangular chamber shaft below the frame, revealed when the cover
    lifts. Thin walls; sleeve sits below the frame underside at z=0."""
    wall = 0.028
    outer_l = SHAFT_LENGTH + 2.0 * wall
    outer_w = SHAFT_WIDTH + 2.0 * wall
    top_z = 0.0
    bottom_z = top_z - SHAFT_DEPTH
    h = top_z - bottom_z
    outer_box = (
        cq.Workplane("XY")
        .box(outer_l, outer_w, h)
        .translate((0.0, 0.0, (top_z + bottom_z) / 2.0))
    )
    bore = (
        cq.Workplane("XY")
        .box(SHAFT_LENGTH, SHAFT_WIDTH, h + 0.04)
        .translate((0.0, 0.0, (top_z + bottom_z) / 2.0))
    )
    return outer_box.cut(bore)


def _cover_solid() -> cq.Workplane:
    """Heavy rectangular cast-iron slab with chamfered top edges, a raised
    perimeter rim, and a center pry slot. Authored in local frame: centered in
    plan, bottom at local z=0."""
    # Main slab body.
    cover = (
        cq.Workplane("XY")
        .box(COVER_LENGTH, COVER_WIDTH, COVER_THICK)
        .translate((0.0, 0.0, COVER_THICK / 2.0))
    )
    # Weathered chamfer on the top perimeter edges.
    cover = cover.faces(">Z").edges().chamfer(COVER_TOP_BEVEL)

    # Raised perimeter rim: a rectangular band standing above the top face.
    # Build as four rectangular bars forming a border on the top face.
    rim_top_z = COVER_THICK
    rim_cz = rim_top_z + RIM_HEIGHT / 2.0
    half_l = COVER_LENGTH / 2.0
    half_w = COVER_WIDTH / 2.0
    rw = RIM_WIDTH
    # Long rim bars (along X, at +/- Y edges).
    rim_long_l = COVER_LENGTH
    rim_long_w = rw
    rim_y_offset = half_w - rw / 2.0
    rim_bar_pos_y = (
        cq.Workplane("XY")
        .box(rim_long_l, rim_long_w, RIM_HEIGHT)
        .translate((0.0, rim_y_offset, rim_cz))
    )
    rim_bar_neg_y = (
        cq.Workplane("XY")
        .box(rim_long_l, rim_long_w, RIM_HEIGHT)
        .translate((0.0, -rim_y_offset, rim_cz))
    )
    # Short rim bars (along Y, at +/- X edges), fitting between the long bars.
    rim_short_l = rw
    rim_short_w = COVER_WIDTH - 2.0 * rw
    rim_x_offset = half_l - rw / 2.0
    rim_bar_pos_x = (
        cq.Workplane("XY")
        .box(rim_short_l, rim_short_w, RIM_HEIGHT)
        .translate((rim_x_offset, 0.0, rim_cz))
    )
    rim_bar_neg_x = (
        cq.Workplane("XY")
        .box(rim_short_l, rim_short_w, RIM_HEIGHT)
        .translate((-rim_x_offset, 0.0, rim_cz))
    )
    cover = cover.union(rim_bar_pos_y).union(rim_bar_neg_y)
    cover = cover.union(rim_bar_pos_x).union(rim_bar_neg_x)

    # Center pry slot: a shallow recess cut into the top face (through the rim
    # into the slab). Oriented along the long axis.
    slot = (
        cq.Workplane("XY")
        .slot2D(SLOT_LEN, SLOT_WID, 0.0)
        .extrude(SLOT_DEPTH + 0.01)
        .translate((0.0, 0.0, COVER_THICK - SLOT_DEPTH))
    )
    cover = cover.cut(slot)
    return cover


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rectangular_cast_iron_access_cover")
    model.material("cast_iron_cover", rgba=(0.22, 0.22, 0.24, 1.0))
    model.material("stone_frame", rgba=(0.48, 0.47, 0.44, 1.0))
    model.material("shaft_void", rgba=(0.06, 0.06, 0.07, 1.0))

    # Frame ring (root, fixed to ground).
    frame = model.part("frame_ring")
    frame.visual(mesh_from_cadquery(_frame_solid(), "frame_ring"), material="stone_frame")

    # Hollow shaft below, fixed to the frame so the void reads when cover lifts.
    shaft = model.part("shaft")
    shaft.visual(mesh_from_cadquery(_shaft_solid(), "shaft"), material="shaft_void")
    model.articulation(
        "frame_to_shaft",
        ArticulationType.FIXED,
        parent=frame,
        child=shaft,
        origin=Origin(),
    )

    # Cover. Local frame: slab centered in plan, bottom at local z=0.
    cover = model.part("access_cover")
    cover.visual(
        mesh_from_cadquery(_cover_solid(), "access_cover"),
        material="cast_iron_cover",
        name="cover_slab",
    )

    # Prismatic lift: at q=0 the cover bottom rests on the seat ledge.
    model.articulation(
        "frame_to_cover",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=cover,
        origin=Origin(xyz=(0.0, 0.0, COVER_REST_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2500.0, velocity=0.15, lower=0.0, upper=LIFT_TRAVEL
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame_ring")
    shaft = object_model.get_part("shaft")
    cover = object_model.get_part("access_cover")
    lift = object_model.get_articulation("frame_to_cover")

    # The cover seats a few mm into the recessed frame ledge at rest. Allow that
    # small, local, mechanically-real seating embed and prove it with contact.
    ctx.allow_overlap(
        cover,
        frame,
        reason="Cover seats a few mm onto the recessed frame ledge (seated insertion).",
    )

    # --- Joint contract: primary articulation is a +Z prismatic lift. ---
    ctx.check(
        "primary joint is prismatic",
        lift.articulation_type == ArticulationType.PRISMATIC,
        details=str(lift.articulation_type),
    )
    ctx.check(
        "lift axis is +Z",
        tuple(round(a, 6) for a in lift.axis) == (0.0, 0.0, 1.0),
        details=str(lift.axis),
    )
    lim = lift.motion_limits
    ctx.check(
        "lift travel clears the frame band",
        lim is not None and lim.lower == 0.0 and lim.upper >= FRAME_TOP_BAND,
        details=f"lower={lim.lower}, upper={lim.upper}, band={FRAME_TOP_BAND}",
    )

    # --- Rectangular plan: cover is clearly oblong (longer than wide). ---
    c_aabb_rest = ctx.part_world_aabb(cover)
    if c_aabb_rest is not None:
        dx = c_aabb_rest[1][0] - c_aabb_rest[0][0]
        dy = c_aabb_rest[1][1] - c_aabb_rest[0][1]
        ctx.check(
            "cover plan is oblong (length > 1.5x width)",
            dx > 1.5 * dy,
            details=f"dx={dx:.4f}, dy={dy:.4f}, ratio={dx/max(dy,1e-9):.2f}",
        )

    # --- Frame opening is also oblong and matches cover aspect. ---
    f_aabb = ctx.part_world_aabb(frame)
    if f_aabb is not None:
        fdx = f_aabb[1][0] - f_aabb[0][0]
        fdy = f_aabb[1][1] - f_aabb[0][1]
        ctx.check(
            "frame plan is oblong (length > 1.3x width)",
            fdx > 1.3 * fdy,
            details=f"fdx={fdx:.4f}, fdy={fdy:.4f}",
        )

    # --- Base at z ~ 0: frame footprint sits on the ground. ---
    ctx.check(
        "frame footprint at z~0",
        f_aabb is not None and abs(f_aabb[0][2]) <= 0.002,
        details=str(f_aabb),
    )

    # --- Rest pose: cover seated in the frame opening, slightly proud. ---
    with ctx.pose({lift: 0.0}):
        c_aabb = ctx.part_world_aabb(cover)
        ctx.check(
            "cover seated on ledge at rest",
            c_aabb is not None
            and abs(c_aabb[0][2] - COVER_REST_BOTTOM_Z) <= 0.004,
            details=f"cover_bottom_z={c_aabb[0][2] if c_aabb else None}, expected={COVER_REST_BOTTOM_Z}",
        )
        ctx.check(
            "cover top proud of frame top band",
            c_aabb is not None and f_aabb is not None
            and c_aabb[1][2] >= f_aabb[1][2] - 0.001,
            details=f"cover_top={c_aabb[1][2] if c_aabb else None} frame_top={f_aabb[1][2] if f_aabb else None}",
        )
        # Cover spans the frame opening in plan on both axes.
        ctx.expect_overlap(
            cover, frame, axes="xy", min_overlap=0.30,
            name="cover spans the frame opening in plan",
        )
        # Cover sits within the frame opening.
        ctx.expect_within(
            cover, frame, axes="xy", margin=0.0,
            name="cover sits within the frame opening",
        )
        # Cover is genuinely seated (in contact) on the frame ledge.
        ctx.expect_contact(
            cover, frame, contact_tol=0.001,
            name="cover seated in contact with frame ledge",
        )

    # --- Lifted pose: cover rises clear of the frame. ---
    rest_pos = ctx.part_world_position(cover)
    with ctx.pose({lift: LIFT_TRAVEL}):
        up_pos = ctx.part_world_position(cover)
        up_aabb = ctx.part_world_aabb(cover)
        ctx.check(
            "cover lifts upward along +Z",
            rest_pos is not None and up_pos is not None
            and up_pos[2] > rest_pos[2] + 0.05,
            details=f"rest={rest_pos}, up={up_pos}",
        )
        ctx.check(
            "lifted cover clears the frame",
            up_aabb is not None and f_aabb is not None
            and up_aabb[0][2] >= f_aabb[1][2] - 0.001,
            details=f"cover_bottom={up_aabb[0][2] if up_aabb else None} frame_top={f_aabb[1][2] if f_aabb else None}",
        )

    # --- Shaft is a hollow void under the opening, fixed to the frame. ---
    s_aabb = ctx.part_world_aabb(shaft)
    ctx.check(
        "shaft void sits below the seat ledge",
        s_aabb is not None and s_aabb[1][2] <= SEAT_LEDGE_TOP_Z + 0.001,
        details=str(s_aabb),
    )
    ctx.expect_overlap(
        shaft, frame, axes="xy", min_overlap=0.25,
        name="shaft is centered under the frame opening",
    )

    return ctx.report()


object_model = build_object_model()
