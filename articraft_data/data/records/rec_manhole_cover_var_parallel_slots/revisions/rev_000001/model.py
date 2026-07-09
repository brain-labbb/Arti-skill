from __future__ import annotations

# Square rusty cast-iron drainage grate with straight parallel through-slots
# running edge to edge across the field: long straight bars (grate_0, grate_1, …)
# with evenly spaced slot openings between them.  The grate seats in its recessed
# frame and lifts straight up (PRISMATIC +Z) to reveal the gully void below.

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
# Real-world dimensions (meters).  A ~300 mm square cast-iron gully grate.
# ---------------------------------------------------------------------------
GRATE_SIDE = 0.300          # square grate plan side
GRATE_THICK = 0.026         # cast-iron grate plate thickness
GRATE_BORDER = 0.024        # solid border frame around the slotted field

# Parallel slot pattern: long straight bars running in X, spaced in Y.
BAR = 0.012                 # thickness of each cast bar (in Y)
SLOT_WID = 0.020            # width of each through-slot opening (in Y)
BAR_EMBED = 0.003           # how far each bar embeds into the border on each X end

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


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _bar_solid(length: float, width: float, thickness: float) -> cq.Workplane:
    """A single straight cast-iron bar, bottom at z=0, centered in XY."""
    return (
        cq.Workplane("XY")
        .box(length, width, thickness)
        .translate((0.0, 0.0, thickness / 2.0))
    )


def _border_solid() -> cq.Workplane:
    """Square border frame: plate with rectangular field opening, bottom at z=0."""
    plate = (
        cq.Workplane("XY")
        .box(GRATE_SIDE, GRATE_SIDE, GRATE_THICK)
        .translate((0.0, 0.0, GRATE_THICK / 2.0))
    )
    plate = plate.edges("|Z").chamfer(0.003)
    field_side = GRATE_SIDE - 2.0 * GRATE_BORDER
    hole = (
        cq.Workplane("XY")
        .box(field_side, field_side, GRATE_THICK + 0.02)
        .translate((0.0, 0.0, GRATE_THICK / 2.0))
    )
    return plate.cut(hole)


def _compute_bar_layout() -> tuple[int, float, float]:
    """Return (n_bars, y_first_center, pitch) for evenly spaced parallel bars."""
    field_half = GRATE_SIDE / 2.0 - GRATE_BORDER
    field_span = 2.0 * field_half
    pitch = BAR + SLOT_WID
    n_bars = int((field_span + SLOT_WID) / pitch)
    total = n_bars * BAR + (n_bars - 1) * SLOT_WID
    padding = (field_span - total) / 2.0
    y_first = -field_half + padding + BAR / 2.0
    return n_bars, y_first, pitch


BAR_LENGTH = 2.0 * (GRATE_SIDE / 2.0 - GRATE_BORDER) + 2.0 * BAR_EMBED


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


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cast_iron_parallel_slot_grate")
    model.material("rusty_iron", rgba=(0.45, 0.27, 0.16, 1.0))
    model.material("frame_iron", rgba=(0.40, 0.30, 0.24, 1.0))
    model.material("gully_void", rgba=(0.06, 0.06, 0.07, 1.0))

    # --- Frame ring (root) ---------------------------------------------------
    frame = model.part("frame_ring")
    frame.visual(mesh_from_cadquery(_frame_solid(), "frame_ring"), material="frame_iron")

    # --- Shaft / gully void (fixed to frame) ---------------------------------
    shaft = model.part("shaft")
    shaft.visual(mesh_from_cadquery(_shaft_solid(), "shaft"), material="gully_void")
    model.articulation(
        "frame_to_shaft",
        ArticulationType.FIXED,
        parent=frame,
        child=shaft,
        origin=Origin(),
    )

    # --- Drain grate with parallel slot bars ---------------------------------
    grate = model.part("drain_grate")

    # Border frame (solid perimeter)
    grate.visual(
        mesh_from_cadquery(_border_solid(), "border"),
        material="rusty_iron",
        name="border",
    )

    # Parallel bars running in X, evenly spaced in Y, emitted via loop
    n_bars, y_first, pitch = _compute_bar_layout()
    for i in range(n_bars):
        y = y_first + i * pitch
        bar = _bar_solid(BAR_LENGTH, BAR, GRATE_THICK).translate((0.0, y, 0.0))
        grate.visual(
            mesh_from_cadquery(bar, f"grate_{i}"),
            material="rusty_iron",
            name=f"grate_{i}",
        )

    # Prismatic lift joint
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


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame_ring")
    shaft = object_model.get_part("shaft")
    grate = object_model.get_part("drain_grate")
    lift = object_model.get_articulation("frame_to_grate")

    # Seated insertion: grate sits a few mm onto the recessed frame ledge.
    ctx.allow_overlap(
        grate,
        frame,
        reason="Grate seats a few mm onto the recessed frame ledge (seated insertion).",
    )

    # --- Joint type and axis -------------------------------------------------
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

    # --- Frame position ------------------------------------------------------
    f_aabb = ctx.part_world_aabb(frame)
    ctx.check(
        "frame footprint at z~0",
        f_aabb is not None and abs(f_aabb[0][2]) <= 0.002,
        details=str(f_aabb),
    )

    # --- Parallel slot pattern -----------------------------------------------
    n_bars, y_first, pitch = _compute_bar_layout()
    bar_names = [f"grate_{i}" for i in range(n_bars)]
    ctx.check(
        "parallel bars emitted",
        all(grate.get_visual(n) is not None for n in bar_names),
        details=f"expected {n_bars} bars: {bar_names}",
    )

    # Border exists
    ctx.check(
        "border frame present",
        grate.get_visual("border") is not None,
        details="border visual missing",
    )

    # Bars are long straight members: verify via known construction constants
    ctx.check(
        "bars are long straight members",
        BAR_LENGTH > 5.0 * BAR,
        details=f"bar_length={BAR_LENGTH:.4f}, bar_width={BAR:.4f}",
    )
    # Slots are through-openings: slot width > 0 and bars don't fill the field
    total_bar_y = n_bars * BAR
    ctx.check(
        "parallel slots exist between bars",
        total_bar_y < (GRATE_SIDE - 2.0 * GRATE_BORDER),
        details=f"total_bar_y={total_bar_y:.4f}, field_span={GRATE_SIDE - 2.0 * GRATE_BORDER:.4f}",
    )

    # --- Rest-pose seating ---------------------------------------------------
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
        ctx.check(
            "grate is a full square plate",
            g_aabb is not None
            and abs((g_aabb[1][0] - g_aabb[0][0]) - GRATE_SIDE) <= 0.003
            and abs((g_aabb[1][1] - g_aabb[0][1]) - GRATE_SIDE) <= 0.003,
            details=str(g_aabb),
        )

    # --- Lifted pose ---------------------------------------------------------
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

    # --- Gully void ----------------------------------------------------------
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
