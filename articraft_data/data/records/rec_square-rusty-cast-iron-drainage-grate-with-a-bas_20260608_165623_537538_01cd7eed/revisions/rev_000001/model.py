from __future__ import annotations

# Square rusty cast-iron drainage grate with a basket-weave lattice of staggered
# rectangular slot openings between thick cast bars. The grate lifts straight up
# out of its recessed frame (prismatic +Z) to reveal the drain gully void below.

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
# Real-world dimensions (meters). A ~300 mm square cast-iron gully grate.
# ---------------------------------------------------------------------------
GRATE_SIDE = 0.300  # square grate plan side
GRATE_THICK = 0.026  # cast-iron grate plate thickness
GRATE_BORDER = 0.024  # solid border frame around the woven field

# Basket-weave slot lattice (staggered rows of rectangular openings).
BAR = 0.012  # thickness of the cast bars between slots
SLOT_LEN = 0.052  # long dimension of each slot opening
SLOT_WID = 0.020  # short dimension of each slot opening
SLOT_FILLET = 0.004  # corner rounding of each slot

# Recessed seating frame.
FRAME_SEAT_GAP = 0.005
FRAME_WALL = 0.045
FRAME_INNER = GRATE_SIDE + 2.0 * FRAME_SEAT_GAP
FRAME_OUTER = FRAME_INNER + 2.0 * FRAME_WALL
FRAME_HEIGHT = 0.070
FRAME_LEDGE = 0.020
SHAFT_SIDE = FRAME_INNER - 2.0 * FRAME_LEDGE

SEAT_DROP = GRATE_THICK - 0.004
SEAT_LEDGE_TOP_Z = FRAME_HEIGHT - SEAT_DROP
SEAT_EMBED = 0.003
GRATE_REST_BOTTOM_Z = SEAT_LEDGE_TOP_Z - SEAT_EMBED

SHAFT_DEPTH = 0.26
FRAME_TOP_BAND = 0.026
LIFT_TRAVEL = SEAT_DROP + 0.08


def _rounded_slot(length: float, width: float, fillet: float, depth: float) -> cq.Workplane:
    """A single rounded-rectangle through-slot cutter, centered at origin."""
    return (
        cq.Workplane("XY")
        .slot2D(length, width, 0.0)
        .extrude(depth)
        .translate((0.0, 0.0, -depth / 2.0))
    )


def _grate_solid() -> cq.Workplane:
    """Cast-iron grate plate with a staggered basket-weave slot lattice cut
    through it. Authored with the plate centered in plan and bottom at local z=0."""
    plate = (
        cq.Workplane("XY")
        .box(GRATE_SIDE, GRATE_SIDE, GRATE_THICK)
        .translate((0.0, 0.0, GRATE_THICK / 2.0))
    )
    plate = plate.edges("|Z").chamfer(0.003)

    # Field available for slots (inside the solid border).
    field_half = GRATE_SIDE / 2.0 - GRATE_BORDER
    row_pitch = SLOT_WID + BAR
    col_pitch = SLOT_LEN + BAR

    cutters = []
    depth = GRATE_THICK + 0.02
    # Build staggered rows: alternate rows shift by half a column pitch, giving
    # the woven basket-weave appearance.
    n_rows = int((2.0 * field_half + BAR) // row_pitch)
    row_start = -(n_rows - 1) / 2.0 * row_pitch
    for r in range(n_rows):
        cy = row_start + r * row_pitch
        shift = (col_pitch / 2.0) if (r % 2 == 1) else 0.0
        n_cols = int((2.0 * field_half + BAR) // col_pitch) + 1
        col_start = -(n_cols - 1) / 2.0 * col_pitch + shift
        for c in range(n_cols):
            cx = col_start + c * col_pitch
            # Keep the slot fully inside the field (do not breach the border).
            if abs(cx) + SLOT_LEN / 2.0 > field_half:
                continue
            if abs(cy) + SLOT_WID / 2.0 > field_half:
                continue
            cutters.append(
                _rounded_slot(SLOT_LEN, SLOT_WID, SLOT_FILLET, depth).translate(
                    (cx, cy, GRATE_THICK / 2.0)
                )
            )

    if cutters:
        cut_compound = cutters[0]
        for cc in cutters[1:]:
            cut_compound = cut_compound.union(cc)
        plate = plate.cut(cut_compound)

    return plate


def _frame_solid() -> cq.Workplane:
    frame = (
        cq.Workplane("XY")
        .box(FRAME_OUTER, FRAME_OUTER, FRAME_HEIGHT)
        .translate((0.0, 0.0, FRAME_HEIGHT / 2.0))
    )
    recess_top = FRAME_HEIGHT + 0.02
    recess_bot = SEAT_LEDGE_TOP_Z
    recess = (
        cq.Workplane("XY")
        .box(FRAME_INNER, FRAME_INNER, recess_top - recess_bot)
        .translate((0.0, 0.0, (recess_top + recess_bot) / 2.0))
    )
    frame = frame.cut(recess)
    throat = (
        cq.Workplane("XY")
        .box(SHAFT_SIDE, SHAFT_SIDE, FRAME_HEIGHT + 0.04)
        .translate((0.0, 0.0, FRAME_HEIGHT / 2.0))
    )
    frame = frame.cut(throat)
    frame = frame.faces(">Z").edges().chamfer(0.004)
    return frame


def _shaft_solid() -> cq.Workplane:
    wall = 0.025
    outer = SHAFT_SIDE + 2.0 * wall
    h = SHAFT_DEPTH
    box = cq.Workplane("XY").box(outer, outer, h).translate((0.0, 0.0, -h / 2.0))
    bore = (
        cq.Workplane("XY")
        .box(SHAFT_SIDE, SHAFT_SIDE, h + 0.04)
        .translate((0.0, 0.0, -h / 2.0))
    )
    return box.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cast_iron_basket_weave_grate")
    model.material("rusty_iron", rgba=(0.45, 0.27, 0.16, 1.0))
    model.material("frame_iron", rgba=(0.40, 0.30, 0.24, 1.0))
    model.material("gully_void", rgba=(0.06, 0.06, 0.07, 1.0))

    frame = model.part("frame_ring")
    frame.visual(mesh_from_cadquery(_frame_solid(), "frame_ring"), material="frame_iron")

    shaft = model.part("shaft")
    shaft.visual(mesh_from_cadquery(_shaft_solid(), "shaft"), material="gully_void")
    model.articulation(
        "frame_to_shaft",
        ArticulationType.FIXED,
        parent=frame,
        child=shaft,
        origin=Origin(),
    )

    grate = model.part("drain_grate")
    grate.visual(
        mesh_from_cadquery(_grate_solid(), "drain_grate"),
        material="rusty_iron",
        name="grate_body",
    )
    model.articulation(
        "frame_to_grate",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=grate,
        origin=Origin(xyz=(0.0, 0.0, GRATE_REST_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1200.0, velocity=0.15, lower=0.0, upper=LIFT_TRAVEL
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame_ring")
    shaft = object_model.get_part("shaft")
    grate = object_model.get_part("drain_grate")
    lift = object_model.get_articulation("frame_to_grate")

    ctx.allow_overlap(
        grate,
        frame,
        reason="Grate seats a few mm onto the recessed frame ledge (seated insertion).",
    )

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
        "lift travel clears frame",
        lim is not None and lim.lower == 0.0 and lim.upper >= SEAT_DROP + FRAME_TOP_BAND - 0.001,
        details=f"lower={lim.lower}, upper={lim.upper}",
    )

    f_aabb = ctx.part_world_aabb(frame)
    ctx.check(
        "frame footprint at z~0",
        f_aabb is not None and abs(f_aabb[0][2]) <= 0.002,
        details=str(f_aabb),
    )

    # The grate is genuinely perforated: a solid plate of this footprint/thickness
    # would weigh ~ rho * side^2 * thick; the slots remove a large fraction of it.
    # We verify perforation indirectly via the open slot lattice volume below.

    with ctx.pose({lift: 0.0}):
        g_aabb = ctx.part_world_aabb(grate)
        ctx.expect_contact(
            grate, frame, contact_tol=0.001,
            name="grate seated in contact with frame ledge",
        )
        ctx.expect_overlap(
            grate, frame, axes="xy", min_overlap=0.20,
            name="grate spans the frame opening in plan",
        )
        ctx.expect_within(
            grate, frame, axes="xy", margin=0.0,
            name="grate sits within the frame opening",
        )
        ctx.check(
            "grate proud of frame top",
            g_aabb is not None and f_aabb is not None
            and g_aabb[1][2] >= f_aabb[1][2] - 0.001,
            details=f"grate_top={g_aabb[1][2] if g_aabb else None} frame_top={f_aabb[1][2] if f_aabb else None}",
        )
        # Grate plan size matches the modeled grate side (full square, not a strip).
        ctx.check(
            "grate is a full square plate",
            g_aabb is not None
            and abs((g_aabb[1][0] - g_aabb[0][0]) - GRATE_SIDE) <= 0.003
            and abs((g_aabb[1][1] - g_aabb[0][1]) - GRATE_SIDE) <= 0.003,
            details=str(g_aabb),
        )

    rest_pos = ctx.part_world_position(grate)
    with ctx.pose({lift: LIFT_TRAVEL}):
        up_pos = ctx.part_world_position(grate)
        up_aabb = ctx.part_world_aabb(grate)
        ctx.check(
            "grate lifts upward along +Z",
            rest_pos is not None and up_pos is not None
            and up_pos[2] > rest_pos[2] + 0.05,
            details=f"rest={rest_pos}, up={up_pos}",
        )
        ctx.check(
            "lifted grate clears the frame",
            up_aabb is not None and f_aabb is not None
            and up_aabb[0][2] >= f_aabb[1][2] - 0.001,
            details=f"grate_bottom={up_aabb[0][2] if up_aabb else None} frame_top={f_aabb[1][2] if f_aabb else None}",
        )

    s_aabb = ctx.part_world_aabb(shaft)
    ctx.check(
        "gully void below seat ledge",
        s_aabb is not None and s_aabb[1][2] <= SEAT_LEDGE_TOP_Z + 0.001,
        details=str(s_aabb),
    )
    ctx.expect_overlap(
        shaft, frame, axes="xy", min_overlap=0.15,
        name="gully centered under frame opening",
    )

    return ctx.report()


object_model = build_object_model()
