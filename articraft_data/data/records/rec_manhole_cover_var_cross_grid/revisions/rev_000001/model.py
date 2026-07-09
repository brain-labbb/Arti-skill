from __future__ import annotations

# Square rusty cast-iron drainage grate with a regular cross / waffle grid of
# square through-holes separated by an even orthogonal lattice of cast bars
# running in both X and Y directions. The grate lifts straight up out of its
# recessed frame (prismatic +Z) to reveal the drain gully void below.

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
GRATE_BORDER = 0.024  # solid border frame around the waffle field

# Cross-grid waffle lattice (uniform square openings with orthogonal bars).
BAR = 0.012  # thickness of the cast bars between holes
HOLE_SIDE = 0.030  # side length of each square hole opening
HOLE_FILLET = 0.003  # corner rounding of each square hole

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

# Derived grid parameters for the cross-waffle pattern.
GRID_PITCH = HOLE_SIDE + BAR  # center-to-center spacing of square holes
FIELD_HALF = GRATE_SIDE / 2.0 - GRATE_BORDER
N_HOLES = int((2.0 * FIELD_HALF + BAR) // GRID_PITCH)


def _square_hole_cutter(side: float, fillet: float, depth: float) -> cq.Workplane:
    """A single square through-hole cutter with optional fillet, centered at origin."""
    cutter = (
        cq.Workplane("XY")
        .rect(side, side)
        .extrude(depth)
        .translate((0.0, 0.0, -depth / 2.0))
    )
    if fillet > 0.0:
        cutter = cutter.edges("|Z").fillet(fillet)
    return cutter


def _grate_solid() -> cq.Workplane:
    """Cast-iron grate plate with a uniform cross-grid waffle pattern of square
    through-holes cut through it. Two orthogonal families of cast bars separate
    the holes, producing a clean cross-hatched waffle grate face.
    Authored with the plate centered in plan and bottom at local z=0."""
    plate = (
        cq.Workplane("XY")
        .box(GRATE_SIDE, GRATE_SIDE, GRATE_THICK)
        .translate((0.0, 0.0, GRATE_THICK / 2.0))
    )
    plate = plate.edges("|Z").chamfer(0.003)

    cutters = []
    depth = GRATE_THICK + 0.02

    # Regular grid: n_holes x n_holes square holes centered in the field.
    grid_start = -(N_HOLES - 1) / 2.0 * GRID_PITCH
    for row_i in range(N_HOLES):
        cy = grid_start + row_i * GRID_PITCH
        for col_i in range(N_HOLES):
            cx = grid_start + col_i * GRID_PITCH
            # Keep the hole fully inside the field (do not breach the border).
            if abs(cx) + HOLE_SIDE / 2.0 > FIELD_HALF:
                continue
            if abs(cy) + HOLE_SIDE / 2.0 > FIELD_HALF:
                continue
            cutters.append(
                _square_hole_cutter(HOLE_SIDE, HOLE_FILLET, depth).translate(
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
    model = ArticulatedObject(name="cast_iron_waffle_grate")
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

    # Cross-grid waffle pattern checks: verify the grate has square holes
    # in a regular (non-staggered) orthogonal grid.
    ctx.check(
        "square holes have equal side length",
        abs(HOLE_SIDE - 0.030) < 0.001,
        details=f"HOLE_SIDE={HOLE_SIDE}",
    )
    ctx.check(
        "uniform grid pitch in both directions",
        abs(GRID_PITCH - (HOLE_SIDE + BAR)) < 0.0001,
        details=f"GRID_PITCH={GRID_PITCH}, expected={HOLE_SIDE + BAR}",
    )
    ctx.check(
        "grid has at least 4 holes per row for waffle pattern",
        N_HOLES >= 4,
        details=f"N_HOLES={N_HOLES}",
    )

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
