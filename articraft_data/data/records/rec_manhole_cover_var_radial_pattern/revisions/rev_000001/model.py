from __future__ import annotations

# Round cast-iron drain grate with a radial fan of spoke-like slot openings
# arranged around a solid central hub. Each slim slot points outward from the
# hub toward the rim, giving the classic radial-burst storm-drain look. The
# grate disc seats in its concrete surround frame and lifts straight up
# (prismatic +Z) to reveal the drain void below.

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
# Real-world dimensions (meters). ~150 mm diameter round cast-iron drain grate.
# ---------------------------------------------------------------------------
GRATE_RADIUS = 0.075  # grate disc radius (150 mm diameter)
GRATE_THICK = 0.014   # cast-iron grate plate thickness
RIM_WIDTH = 0.012     # solid outer rim band width
HUB_RADIUS = 0.022    # solid central hub radius

# Radial spoke slots
N_SLOTS = 10          # number of radial slots evenly distributed around the circle
SLOT_WIDTH = 0.009    # width of each radial slot
SLOT_INNER_R = HUB_RADIUS + 0.003   # slot starts just outside hub
SLOT_OUTER_R = GRATE_RADIUS - RIM_WIDTH - 0.002  # slot ends just inside rim

# Concrete surround (square block with round recess)
SUR_SEAT_GAP = 0.004
SUR_WALL = 0.060
SUR_INNER_R = GRATE_RADIUS + SUR_SEAT_GAP  # round recess radius
SUR_OUTER = 2.0 * SUR_INNER_R + 2.0 * SUR_WALL  # square block side
SUR_HEIGHT = 0.085
SUR_LEDGE = 0.016
SHAFT_RADIUS = SUR_INNER_R - SUR_LEDGE

# Grate seating: top flush with concrete top (ground plane)
SEAT_LEDGE_TOP_Z = SUR_HEIGHT - GRATE_THICK
SEAT_EMBED = 0.002
GRATE_REST_BOTTOM_Z = SEAT_LEDGE_TOP_Z - SEAT_EMBED

SHAFT_DEPTH = 0.20
LIFT_TRAVEL = GRATE_THICK + 0.06


def _radial_slot_box(
    angle_deg: float,
    inner_r: float,
    outer_r: float,
    width: float,
    depth: float,
    z_center: float,
) -> cq.Workplane:
    """Shared radial slot geometry helper.

    Builds a thin box oriented radially at the given angle from the origin.
    Used both as a boolean cutter (for slot openings) and as a visual mesh
    (for dark void fills in each slot).
    """
    length = outer_r - inner_r
    mid_r = (inner_r + outer_r) / 2.0
    slot = (
        cq.Workplane("XY")
        .box(width, length, depth)
        .translate((0.0, mid_r, z_center))
    )
    return slot.rotate((0, 0, 0), (0, 0, 1), angle_deg)


def _grate_solid() -> cq.Workplane:
    """Round cast-iron grate disc with radial spoke slots.

    Centered in plan, bottom at local z=0. The disc has a raised solid
    central hub and a set of slim through-slots radiating outward toward
    the rim.
    """
    # Main disc plate
    plate = (
        cq.Workplane("XY")
        .circle(GRATE_RADIUS)
        .extrude(GRATE_THICK)
    )
    # Raised solid central hub (stands slightly proud of the field)
    hub = (
        cq.Workplane("XY")
        .circle(HUB_RADIUS)
        .extrude(GRATE_THICK + 0.003)
    )
    plate = plate.union(hub)

    # Cut radial spoke slots through the plate using a shared helper
    cut_depth = GRATE_THICK + 0.04
    z_center = GRATE_THICK / 2.0
    cutters = []
    for i in range(N_SLOTS):
        angle = i * (360.0 / N_SLOTS)
        cutters.append(
            _radial_slot_box(
                angle, SLOT_INNER_R, SLOT_OUTER_R, SLOT_WIDTH, cut_depth, z_center
            )
        )
    cut_compound = cutters[0]
    for cc in cutters[1:]:
        cut_compound = cut_compound.union(cc)
    plate = plate.cut(cut_compound)

    return plate


def _surround_solid() -> cq.Workplane:
    """Square concrete surround block with a round recessed grate seat and
    a round through drain throat. Base at z=0; top at SUR_HEIGHT is the
    flush ground plane."""
    block = (
        cq.Workplane("XY")
        .box(SUR_OUTER, SUR_OUTER, SUR_HEIGHT)
        .translate((0.0, 0.0, SUR_HEIGHT / 2.0))
    )
    # Round cover-seat recess down to the seat ledge
    recess_top = SUR_HEIGHT + 0.02
    recess_bot = SEAT_LEDGE_TOP_Z
    recess = (
        cq.Workplane("XY")
        .circle(SUR_INNER_R)
        .extrude(recess_top - recess_bot)
        .translate((0.0, 0.0, recess_bot))
    )
    block = block.cut(recess)
    # Round through drain throat
    throat = (
        cq.Workplane("XY")
        .circle(SHAFT_RADIUS)
        .extrude(SUR_HEIGHT + 0.04)
        .translate((0.0, 0.0, -0.02))
    )
    block = block.cut(throat)
    return block


def _shaft_solid() -> cq.Workplane:
    """Round drain shaft tube below the seat ledge."""
    wall = 0.022
    outer_r = SHAFT_RADIUS + wall
    h = SHAFT_DEPTH
    tube = (
        cq.Workplane("XY")
        .circle(outer_r)
        .extrude(h)
        .translate((0.0, 0.0, -h))
    )
    bore = (
        cq.Workplane("XY")
        .circle(SHAFT_RADIUS)
        .extrude(h + 0.04)
        .translate((0.0, 0.0, -h - 0.02))
    )
    return tube.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cast_iron_radial_slot_drain_grate")
    model.material("dark_iron", rgba=(0.14, 0.14, 0.15, 1.0))
    model.material("concrete", rgba=(0.66, 0.65, 0.62, 1.0))
    model.material("drain_void", rgba=(0.04, 0.04, 0.05, 1.0))

    # --- Concrete surround (root) ---
    surround = model.part("concrete_surround")
    surround.visual(
        mesh_from_cadquery(_surround_solid(), "concrete_surround"),
        material="concrete",
    )

    # --- Drain shaft (fixed child of surround) ---
    shaft = model.part("shaft")
    shaft.visual(
        mesh_from_cadquery(_shaft_solid(), "shaft"),
        material="drain_void",
    )
    model.articulation(
        "surround_to_shaft",
        ArticulationType.FIXED,
        parent=surround,
        child=shaft,
        origin=Origin(),
    )

    # --- Round drain grate with radial spoke slots ---
    grate = model.part("drain_grate")
    grate.visual(
        mesh_from_cadquery(_grate_solid(), "drain_grate"),
        material="dark_iron",
        name="grate_body",
    )

    # Radial slot void visuals: dark fills seated in each slot opening.
    # Each slot visual extends slightly into the hub (inner_r < SLOT_INNER_R)
    # so the dark void fill physically connects to the grate body hub.
    slot_vis_inner_r = SLOT_INNER_R - 0.005
    slot_vis_depth = GRATE_THICK - 0.004
    slot_vis_z = GRATE_THICK / 2.0
    for i in range(N_SLOTS):
        angle = i * (360.0 / N_SLOTS)
        slot_vis = _radial_slot_box(
            angle,
            slot_vis_inner_r,
            SLOT_OUTER_R,
            SLOT_WIDTH,
            slot_vis_depth,
            slot_vis_z,
        )
        grate.visual(
            mesh_from_cadquery(slot_vis, f"grate_{i}"),
            material="drain_void",
            name=f"grate_{i}",
        )

    # Prismatic lift: grate lifts straight up out of the recess
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

    # --- Mechanism checks ---
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

    # --- Surround geometry checks ---
    sur_aabb = ctx.part_world_aabb(surround)
    ctx.check(
        "concrete footprint at z~0",
        sur_aabb is not None and abs(sur_aabb[0][2]) <= 0.002,
        details=str(sur_aabb),
    )
    ctx.check(
        "concrete top at ground plane",
        sur_aabb is not None and abs(sur_aabb[1][2] - SUR_HEIGHT) <= 0.002,
        details=str(sur_aabb),
    )

    # --- Radial slot visuals exist ---
    grate_vis_names = [v.name for v in grate.visuals]
    for i in range(N_SLOTS):
        sn = f"grate_{i}"
        ctx.check(
            f"radial slot visual {sn} exists",
            sn in grate_vis_names,
            details=f"visuals={grate_vis_names}",
        )

    # --- Grate is round: plan dims roughly equal (not square) ---
    with ctx.pose({lift: 0.0}):
        g_aabb = ctx.part_world_aabb(grate)
        if g_aabb is not None:
            dx = g_aabb[1][0] - g_aabb[0][0]
            dy = g_aabb[1][1] - g_aabb[0][1]
            ctx.check(
                "grate is round: plan extents roughly equal",
                abs(dx - dy) <= 0.005,
                details=f"dx={dx:.4f} dy={dy:.4f}",
            )

        # Grate seated and flush
        ctx.expect_contact(
            grate, surround, contact_tol=0.0015,
            name="grate seated in contact with concrete ledge",
        )
        ctx.expect_overlap(
            grate, surround, axes="xy", min_overlap=0.10,
            name="grate spans the surround opening in plan",
        )
        ctx.check(
            "grate top flush with ground",
            g_aabb is not None and sur_aabb is not None
            and abs(g_aabb[1][2] - sur_aabb[1][2]) <= 0.003,
            details=f"grate_top={g_aabb[1][2] if g_aabb else None} ground={sur_aabb[1][2] if sur_aabb else None}",
        )

    # --- Lifted pose: grate clears the recess ---
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

    # --- Drain void ---
    s_aabb = ctx.part_world_aabb(shaft)
    ctx.check(
        "drain void below seat ledge",
        s_aabb is not None and s_aabb[1][2] <= SEAT_LEDGE_TOP_Z + 0.001,
        details=str(s_aabb),
    )
    ctx.expect_overlap(
        shaft, surround, axes="xy", min_overlap=0.06,
        name="drain void centered under opening",
    )

    return ctx.report()


object_model = build_object_model()
