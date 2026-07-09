from __future__ import annotations

# Small dark cast-iron square floor drain grate set flush into a concrete
# surround. The grate has a raised beveled border and a set of curved teardrop
# slots arranged in a radial fan pattern. The grate lifts straight up out of the
# concrete recess (prismatic +Z) to reveal the drain void below.

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
# Real-world dimensions (meters). A ~150 mm square cast-iron floor drain grate.
# ---------------------------------------------------------------------------
GRATE_SIDE = 0.150  # square grate plan side
GRATE_THICK = 0.014  # cast-iron grate plate thickness
BORDER_W = 0.018  # raised beveled border band width
BORDER_RAISE = 0.0  # border sits flush with the field top (single plate top)

# Curved teardrop fan slots.
N_SLOTS = 5  # number of curved fan slots
SLOT_WID = 0.011  # nominal width of each curved slot
ARC_R = 0.085  # radius of the arc the curved slots follow (center below grate)
ARC_SPAN = math.radians(34.0)  # angular extent of each curved slot
SLOT_PITCH = 0.026  # spacing between fan slots across the grate

# Concrete surround (a flush curb block with a recessed cover seat).
SUR_SEAT_GAP = 0.004
SUR_WALL = 0.060  # concrete surround band width
SUR_INNER = GRATE_SIDE + 2.0 * SUR_SEAT_GAP
SUR_OUTER = SUR_INNER + 2.0 * SUR_WALL
SUR_HEIGHT = 0.085  # concrete block height (footprint base at z=0, top = ground)
SUR_LEDGE = 0.016  # inward seat ledge for the grate
SHAFT_SIDE = SUR_INNER - 2.0 * SUR_LEDGE

# The grate sits flush: its top is level with the concrete top (ground plane).
# Grate top at z = SUR_HEIGHT; grate bottom rests on the seat ledge.
SEAT_LEDGE_TOP_Z = SUR_HEIGHT - GRATE_THICK
SEAT_EMBED = 0.002
GRATE_REST_BOTTOM_Z = SEAT_LEDGE_TOP_Z - SEAT_EMBED

SHAFT_DEPTH = 0.20
LIFT_TRAVEL = GRATE_THICK + 0.06  # lift the grate fully out of the recess


def _curved_slot_cutter(center_y: float, depth: float) -> cq.Workplane:
    """One curved teardrop fan slot. Built as an annular sector swept band that
    tapers toward its trailing end (teardrop), extruded through the plate.

    The slot follows an arc of radius ARC_R centered well below the grate so the
    visible curve fans across the plate. center_y shifts the slot across the fan.
    """
    cx, cy = 0.0, center_y - ARC_R  # arc center sits below this slot's row
    a0 = math.radians(90.0) - ARC_SPAN / 2.0
    a1 = math.radians(90.0) + ARC_SPAN / 2.0
    steps = 18
    outer = []
    inner = []
    for i in range(steps + 1):
        t = i / steps
        a = a0 + (a1 - a0) * t
        # Teardrop taper: full width at the start, narrowing toward the end.
        w = SLOT_WID * (1.0 - 0.55 * t) + 0.0015
        ro = ARC_R + w / 2.0
        ri = ARC_R - w / 2.0
        outer.append((cx + ro * math.cos(a), cy + ro * math.sin(a)))
        inner.append((cx + ri * math.cos(a), cy + ri * math.sin(a)))
    pts = outer + list(reversed(inner))
    slot = (
        cq.Workplane("XY")
        .polyline(pts)
        .close()
        .extrude(depth)
        .translate((0.0, 0.0, -depth / 2.0))
    )
    return slot


def _grate_solid() -> cq.Workplane:
    """Cast-iron grate plate with a beveled raised border and curved fan slots.
    Plate centered in plan, bottom at local z=0."""
    plate = (
        cq.Workplane("XY")
        .box(GRATE_SIDE, GRATE_SIDE, GRATE_THICK)
        .translate((0.0, 0.0, GRATE_THICK / 2.0))
    )
    # Beveled border: chamfer the top outer edges so the rim reads as a raised,
    # chamfered cast border.
    plate = plate.faces(">Z").edges().chamfer(0.004)
    plate = plate.edges("|Z").chamfer(0.002)

    # Recess the inner field slightly below the border (so the border reads raised).
    field = GRATE_SIDE - 2.0 * BORDER_W
    field_recess = 0.003
    field_cut = (
        cq.Workplane("XY")
        .box(field, field, field_recess + 0.02)
        .translate((0.0, 0.0, GRATE_THICK - (field_recess + 0.02) / 2.0 + field_recess))
    )
    plate = plate.cut(field_cut)

    # Curved teardrop fan slots, cut through the plate across the field.
    depth = GRATE_THICK + 0.02
    start = -(N_SLOTS - 1) / 2.0 * SLOT_PITCH
    cutters = []
    for s in range(N_SLOTS):
        cy = start + s * SLOT_PITCH
        cutters.append(_curved_slot_cutter(cy, depth).translate((0.0, 0.0, GRATE_THICK / 2.0)))
    cut_compound = cutters[0]
    for cc in cutters[1:]:
        cut_compound = cut_compound.union(cc)
    plate = plate.cut(cut_compound)

    return plate


def _surround_solid() -> cq.Workplane:
    """Concrete surround block with a recessed grate seat and a through drain
    throat. Footprint base at z=0; top face at SUR_HEIGHT is the flush ground."""
    block = (
        cq.Workplane("XY")
        .box(SUR_OUTER, SUR_OUTER, SUR_HEIGHT)
        .translate((0.0, 0.0, SUR_HEIGHT / 2.0))
    )
    # Cover-seat recess down to the seat ledge.
    recess_top = SUR_HEIGHT + 0.02
    recess_bot = SEAT_LEDGE_TOP_Z
    recess = (
        cq.Workplane("XY")
        .box(SUR_INNER, SUR_INNER, recess_top - recess_bot)
        .translate((0.0, 0.0, (recess_top + recess_bot) / 2.0))
    )
    block = block.cut(recess)
    # Through drain throat.
    throat = (
        cq.Workplane("XY")
        .box(SHAFT_SIDE, SHAFT_SIDE, SUR_HEIGHT + 0.04)
        .translate((0.0, 0.0, SUR_HEIGHT / 2.0))
    )
    block = block.cut(throat)
    return block


def _shaft_solid() -> cq.Workplane:
    wall = 0.022
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
    model = ArticulatedObject(name="cast_iron_fan_slot_floor_drain")
    model.material("dark_iron", rgba=(0.16, 0.16, 0.17, 1.0))
    model.material("concrete", rgba=(0.66, 0.65, 0.62, 1.0))
    model.material("drain_void", rgba=(0.05, 0.05, 0.06, 1.0))

    surround = model.part("concrete_surround")
    surround.visual(
        mesh_from_cadquery(_surround_solid(), "concrete_surround"), material="concrete"
    )

    shaft = model.part("shaft")
    shaft.visual(mesh_from_cadquery(_shaft_solid(), "shaft"), material="drain_void")
    model.articulation(
        "surround_to_shaft",
        ArticulationType.FIXED,
        parent=surround,
        child=shaft,
        origin=Origin(),
    )

    grate = model.part("drain_grate")
    grate.visual(
        mesh_from_cadquery(_grate_solid(), "drain_grate"),
        material="dark_iron",
        name="grate_body",
    )
    model.articulation(
        "surround_to_grate",
        ArticulationType.PRISMATIC,
        parent=surround,
        child=grate,
        origin=Origin(xyz=(0.0, 0.0, GRATE_REST_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=600.0, velocity=0.15, lower=0.0, upper=LIFT_TRAVEL
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    surround = object_model.get_part("concrete_surround")
    shaft = object_model.get_part("shaft")
    grate = object_model.get_part("drain_grate")
    lift = object_model.get_articulation("surround_to_grate")

    ctx.allow_overlap(
        grate,
        surround,
        reason="Grate seats a couple mm onto the recessed concrete ledge (seated insertion).",
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
        "lift travel clears the recess",
        lim is not None and lim.lower == 0.0 and lim.upper >= GRATE_THICK,
        details=f"lower={lim.lower}, upper={lim.upper}",
    )

    sur_aabb = ctx.part_world_aabb(surround)
    ctx.check(
        "concrete footprint at z~0",
        sur_aabb is not None and abs(sur_aabb[0][2]) <= 0.002,
        details=str(sur_aabb),
    )
    # Concrete top is the flush ground plane.
    ctx.check(
        "concrete top at ground plane",
        sur_aabb is not None and abs(sur_aabb[1][2] - SUR_HEIGHT) <= 0.002,
        details=str(sur_aabb),
    )

    with ctx.pose({lift: 0.0}):
        g_aabb = ctx.part_world_aabb(grate)
        # Grate seated in contact on the concrete ledge.
        ctx.expect_contact(
            grate, surround, contact_tol=0.0015,
            name="grate seated in contact with concrete ledge",
        )
        # Grate fills the surround opening in plan.
        ctx.expect_overlap(
            grate, surround, axes="xy", min_overlap=0.10,
            name="grate spans the surround opening in plan",
        )
        ctx.expect_within(
            grate, surround, axes="xy", margin=0.0,
            name="grate sits within the surround opening",
        )
        # Grate is flush: its top is level with the concrete top (ground).
        ctx.check(
            "grate top flush with ground",
            g_aabb is not None and sur_aabb is not None
            and abs(g_aabb[1][2] - sur_aabb[1][2]) <= 0.003,
            details=f"grate_top={g_aabb[1][2] if g_aabb else None} ground={sur_aabb[1][2] if sur_aabb else None}",
        )

    rest_pos = ctx.part_world_position(grate)
    with ctx.pose({lift: LIFT_TRAVEL}):
        up_pos = ctx.part_world_position(grate)
        up_aabb = ctx.part_world_aabb(grate)
        ctx.check(
            "grate lifts upward along +Z",
            rest_pos is not None and up_pos is not None
            and up_pos[2] > rest_pos[2] + 0.04,
            details=f"rest={rest_pos}, up={up_pos}",
        )
        ctx.check(
            "lifted grate clears the recess",
            up_aabb is not None and sur_aabb is not None
            and up_aabb[0][2] >= sur_aabb[1][2] - 0.001,
            details=f"grate_bottom={up_aabb[0][2] if up_aabb else None} ground={sur_aabb[1][2] if sur_aabb else None}",
        )

    s_aabb = ctx.part_world_aabb(shaft)
    ctx.check(
        "drain void below seat ledge",
        s_aabb is not None and s_aabb[1][2] <= SEAT_LEDGE_TOP_Z + 0.001,
        details=str(s_aabb),
    )
    ctx.expect_overlap(
        shaft, surround, axes="xy", min_overlap=0.08,
        name="drain void centered under opening",
    )

    return ctx.report()


object_model = build_object_model()
