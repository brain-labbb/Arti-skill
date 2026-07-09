from __future__ import annotations

# Square weathered concrete utility access cover seated in a recessed stone frame
# ring. The heavy cast/cast-concrete cover lifts straight up out of the frame
# (prismatic +Z) to reveal the chamber shaft void below. A small center pry slot
# lets a tool lever the cover up.

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
# Real-world dimensions (meters). A 600 mm class concrete access cover.
# ---------------------------------------------------------------------------
COVER_SIDE = 0.62  # square cover plan side
COVER_THICK = 0.075  # heavy cast-concrete slab thickness
COVER_TOP_BEVEL = 0.018  # chamfer on the top perimeter edge (weathered look)

SLOT_LEN = 0.110  # center pry slot length
SLOT_WID = 0.026  # center pry slot width
SLOT_DEPTH = 0.030  # pry slot recess depth (does not pierce the slab)

# Frame ring: a chunky recessed stone seating frame around the cover.
FRAME_SEAT_GAP = 0.012  # perimeter gap between cover and frame inner wall
FRAME_WALL = 0.115  # radial width of the visible stone frame band
FRAME_INNER = COVER_SIDE + 2.0 * FRAME_SEAT_GAP  # frame opening side
FRAME_OUTER = FRAME_INNER + 2.0 * FRAME_WALL  # frame outer side
FRAME_HEIGHT = 0.140  # total frame depth (top band + downstand into ground)
FRAME_TOP_BAND = 0.055  # height of the visible top frame band above the seat ledge

# Seat ledge: the cover rests on a recessed ledge inside the frame.
SEAT_LEDGE = 0.030  # how far the ledge projects inward under the cover
SEAT_DROP = COVER_THICK - 0.006  # cover top sits ~6 mm proud of the frame top band

# Shaft void revealed when the cover is lifted.
SHAFT_SIDE = FRAME_INNER - 2.0 * SEAT_LEDGE
SHAFT_DEPTH = 0.30

# Prismatic lift travel: enough to clear the frame top band + seat depth.
LIFT_CLEARANCE = FRAME_TOP_BAND + 0.06
LIFT_TRAVEL = SEAT_DROP + LIFT_CLEARANCE  # full clearance of the frame

# At rest, the cover bottom sits on the seat ledge. The seat ledge top is at
# z = FRAME_TOP_BAND - SEAT_DROP relative to the frame's top band reference.
# We place the frame top band so its top is at z = 0 reference is messy; instead
# anchor everything so the ground/frame top sits near z = 0..FRAME_TOP_BAND and
# the footprint base of the frame is at z = 0.

# Frame body spans z in [0, FRAME_HEIGHT]; its top band top face is at FRAME_HEIGHT.
# Cover rest: bottom on the seat ledge.
SEAT_LEDGE_TOP_Z = FRAME_HEIGHT - SEAT_DROP  # ledge top face the cover rests on
SEAT_EMBED = 0.003  # cover seats a few mm into the ledge so it reads as seated
COVER_REST_BOTTOM_Z = SEAT_LEDGE_TOP_Z - SEAT_EMBED  # cover underside seats on ledge
COVER_REST_TOP_Z = COVER_REST_BOTTOM_Z + COVER_THICK  # still proud of frame top band


def _frame_solid() -> cq.Workplane:
    """Recessed stone seating frame: outer block, hollow opening, inner seat ledge,
    and a downstand throat that becomes the visible shaft void when the cover lifts."""
    half_h = FRAME_HEIGHT / 2.0
    # Outer block, base at z=0.
    frame = (
        cq.Workplane("XY")
        .box(FRAME_OUTER, FRAME_OUTER, FRAME_HEIGHT)
        .translate((0.0, 0.0, half_h))
    )
    # Cut the full opening (cover seat + shaft) from the top down.
    # Upper bore: the cover-receiving recess (inner = FRAME_INNER) cut from above
    # the frame top down exactly to the seat ledge top face at SEAT_LEDGE_TOP_Z.
    recess_top = FRAME_HEIGHT + 0.02
    recess_bot = SEAT_LEDGE_TOP_Z
    recess_h = recess_top - recess_bot
    recess = (
        cq.Workplane("XY")
        .box(FRAME_INNER, FRAME_INNER, recess_h)
        .translate((0.0, 0.0, (recess_top + recess_bot) / 2.0))
    )
    frame = frame.cut(recess)
    # Lower bore: the shaft throat (narrower, sits inside the seat ledge),
    # cut all the way through the remaining frame thickness.
    throat = (
        cq.Workplane("XY")
        .box(SHAFT_SIDE, SHAFT_SIDE, FRAME_HEIGHT + 0.04)
        .translate((0.0, 0.0, FRAME_HEIGHT / 2.0))
    )
    frame = frame.cut(throat)
    # Soften the visible top outer perimeter edges a touch (weathered).
    frame = frame.faces(">Z").edges().chamfer(0.010)
    return frame


def _shaft_solid() -> cq.Workplane:
    """A hollow chamber shaft below the frame, fixed to the frame, revealed when
    the cover lifts. Thin walls so it reads as a hollow throat. The sleeve sits
    entirely below the frame (top face abutting the frame underside at z=0) so it
    continues the frame's central void downward without colliding with the frame."""
    wall = 0.030
    outer = SHAFT_SIDE + 2.0 * wall
    # Shaft spans downward from the frame underside (z=0).
    top_z = 0.0
    bottom_z = top_z - SHAFT_DEPTH
    h = top_z - bottom_z
    outer_box = (
        cq.Workplane("XY")
        .box(outer, outer, h)
        .translate((0.0, 0.0, (top_z + bottom_z) / 2.0))
    )
    bore = (
        cq.Workplane("XY")
        .box(SHAFT_SIDE, SHAFT_SIDE, h + 0.04)
        .translate((0.0, 0.0, (top_z + bottom_z) / 2.0))
    )
    return outer_box.cut(bore)


def _cover_solid() -> cq.Workplane:
    """Heavy square slab with chamfered weathered top edges and a center pry slot
    recessed into the top face. Authored in its own local frame with the slab
    centered at the origin in plan and its bottom at local z=0."""
    cover = (
        cq.Workplane("XY")
        .box(COVER_SIDE, COVER_SIDE, COVER_THICK)
        .translate((0.0, 0.0, COVER_THICK / 2.0))
    )
    # Weathered, rounded-over top perimeter (a chamfer reads as a worn slab).
    # Chamfer the top face boundary edges (all four top perimeter edges).
    cover = cover.faces(">Z").edges().chamfer(COVER_TOP_BEVEL)
    # Center pry slot: a shallow rounded recess cut into the top face.
    slot = (
        cq.Workplane("XY")
        .slot2D(SLOT_LEN, SLOT_WID, 0.0)
        .extrude(SLOT_DEPTH + 0.01)
        .translate((0.0, 0.0, COVER_THICK - SLOT_DEPTH))
    )
    cover = cover.cut(slot)
    return cover


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="concrete_utility_access_cover")
    model.material("concrete_cover", rgba=(0.74, 0.73, 0.69, 1.0))
    model.material("stone_frame", rgba=(0.55, 0.55, 0.52, 1.0))
    model.material("shaft_void", rgba=(0.10, 0.10, 0.11, 1.0))

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

    # Cover. Its local frame: slab centered in plan, bottom at local z=0.
    cover = model.part("access_cover")
    cover.visual(
        mesh_from_cadquery(_cover_solid(), "access_cover"),
        material="concrete_cover",
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

    # --- Base at z ~ 0: frame footprint sits on the ground. ---
    f_aabb = ctx.part_world_aabb(frame)
    ctx.check(
        "frame footprint at z~0",
        f_aabb is not None and abs(f_aabb[0][2]) <= 0.002,
        details=str(f_aabb),
    )

    # --- Rest pose: cover seated in the frame opening, slightly proud, no float. ---
    with ctx.pose({lift: 0.0}):
        c_aabb = ctx.part_world_aabb(cover)
        # Cover bottom rests on the seat ledge (~ SEAT_LEDGE_TOP_Z).
        ctx.check(
            "cover seated on ledge at rest",
            c_aabb is not None
            and abs(c_aabb[0][2] - COVER_REST_BOTTOM_Z) <= 0.003,
            details=f"cover_bottom={c_aabb[1] if c_aabb else None} z0={c_aabb[0][2] if c_aabb else None}",
        )
        # Cover top is proud of the frame opening (reads as a seated lid).
        ctx.check(
            "cover top proud of frame top band",
            c_aabb is not None and f_aabb is not None
            and c_aabb[1][2] >= f_aabb[1][2] - 0.001,
            details=f"cover_top={c_aabb[1][2] if c_aabb else None} frame_top={f_aabb[1][2] if f_aabb else None}",
        )
        # Cover footprint overlaps the frame opening in plan (it fills the hole).
        ctx.expect_overlap(
            cover, frame, axes="xy", min_overlap=0.40,
            name="cover spans the frame opening in plan",
        )
        # Perimeter seating gap exists (cover slightly smaller than opening).
        ctx.expect_within(
            cover, frame, axes="xy", margin=0.0,
            name="cover sits within the frame opening",
        )
        # Cover is genuinely seated (in contact) on the frame ledge, not floating.
        ctx.expect_contact(
            cover, frame, contact_tol=0.001,
            name="cover seated in contact with frame ledge",
        )

    # --- Lifted pose: cover rises clear of the frame and the void is exposed. ---
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
        # When fully lifted, the cover bottom clears the frame top band.
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
        shaft, frame, axes="xy", min_overlap=0.30,
        name="shaft is centered under the frame opening",
    )

    return ctx.report()


object_model = build_object_model()
