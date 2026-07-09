from __future__ import annotations

# Square stainless-steel bathroom floor drain, ~0.12 x 0.12 m and 0.03 m tall.
# Variant: concentric_rings grate pattern.
# A flat polished square flange (0.12 x 0.12 x 0.005 m, beveled top edges) sits
# on a shallow open-bottomed tapered drain cup. A circular opening (~0.095 m)
# in the flange holds a removable round grate: a thin seat ring carrying a
# perforated disc with a pattern of concentric circular ring slots (annular
# gaps) radiating outward from the center.
# Articulations: the grate twists about +Z (revolute, -90..+90 deg, twist-to-
# lock) and lifts straight up out of its seat (prismatic +Z, 0..0.03 m).
# Reference image: picture/Other/Metal drain/001.png

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
# Real-world dimensions (meters).
# ---------------------------------------------------------------------------
TOTAL_H = 0.030  # overall height: cup bottom (z=0) to flange top
FLANGE_SIDE = 0.120  # square flange plan side
FLANGE_T = 0.005  # flange plate thickness
FLANGE_BOT_Z = TOTAL_H - FLANGE_T  # 0.025
FLANGE_CHAMFER = 0.0015  # slight bevel on the flange top edges
HOLE_R = 0.0476  # circular flange opening (~0.095 m diameter)

# Tapered, open-bottomed drain cup under the flange (z = 0 .. FLANGE_BOT_Z).
CUP_H = FLANGE_BOT_Z
CUP_TOP_HALF = 0.055  # outer half-side at the top (under the flange)
CUP_BOT_HALF = 0.0425  # outer half-side at the open bottom
CUP_WALL = 0.005
SEAT_PLATE_T = 0.0035  # inward seat plate at the cup top
SEAT_PLATE_HALF = 0.053
SEAT_HOLE_R = 0.034  # round drain throat through the seat plate
SEAT_TOP_Z = FLANGE_BOT_Z  # the grate-rim seat surface

# Dark cavity liner inside the cup (same part as the body; reads as the dark
# recessed drain cavity through the grate slots).
LINER_WALL = 0.0015
LINER_Z0 = 0.002
LINER_Z1 = FLANGE_BOT_Z - SEAT_PLATE_T  # 0.0215
LINER_EMBED = 0.0005  # embed into the cup wall so the liner is connected

# Removable grate assembly: outer seat ring (lifts) + slotted disc (twists).
RIM_OUT_R = 0.0472  # ring outer radius (0.4 mm radial clearance in the hole)
RIM_IN_R = 0.0420  # ring inner radius
RIM_H = 0.0053  # ring height (seat surface to flange-flush top)
LIP_IN_R = 0.0360  # inner support lip the disc rests on
LIP_H = 0.0015
RIM_SEAT_EMBED = 0.0003  # seated insertion into the cup seat plate

GRATE_R = 0.0415  # slotted disc radius (0.5 mm radial clearance in the ring)
GRATE_T = 0.004  # disc thickness
DISC_LOCAL_Z = RIM_H - GRATE_T  # 0.0013: disc bottom in rim frame (0.2 mm
#                                 embedded into the 0.0015 m lip = seated)

# Concentric ring slot field: a series of annular ring gaps cut through the
# disc, with solid web between adjacent rings.
N_RINGS = 5  # number of concentric ring slots
RING_WIDTH = 0.003  # radial width of each annular slot
RING_GAP = 0.003  # solid web between adjacent ring slots
RING_PITCH = RING_WIDTH + RING_GAP  # 0.006 center-to-center spacing
RING_R0 = 0.007  # inner radius of the first (innermost) ring slot

# Radial bridge bars that reconnect the concentric rings across the annular
# slots (common in real drain grates for structural rigidity).
N_BRIDGES = 4  # number of radial spokes
BRIDGE_W = 0.002  # bar width (tangential)
BRIDGE_INNER_R = 0.004  # bars start inside the first ring slot
BRIDGE_OUTER_R = GRATE_R - 0.001  # bars end just inside the disc edge

# Articulation ranges.
LIFT_TRAVEL = 0.030  # prismatic +Z travel
TWIST_LIMIT = math.pi / 2.0  # revolute +/- 90 degrees

RIM_REST_Z = SEAT_TOP_Z - RIM_SEAT_EMBED  # 0.0247: rim frame at rest


def _ring_specs() -> list[tuple[float, float]]:
    """(inner_r, outer_r) for each concentric ring slot, emitted in a loop."""
    specs: list[tuple[float, float]] = []
    for i in range(N_RINGS):
        inner_r = RING_R0 + i * RING_PITCH
        outer_r = inner_r + RING_WIDTH
        specs.append((inner_r, outer_r))
    return specs


def _ring_slot(inner_r: float, outer_r: float, thickness: float) -> cq.Workplane:
    """Annular cutter for one concentric ring slot.

    Returns a hollow cylinder (outer minus inner) centered at z = thickness/2.
    """
    outer_cyl = (
        cq.Workplane("XY")
        .cylinder(thickness, outer_r)
        .translate((0.0, 0.0, thickness / 2.0))
    )
    inner_cyl = (
        cq.Workplane("XY")
        .cylinder(thickness + 0.002, inner_r)
        .translate((0.0, 0.0, thickness / 2.0))
    )
    return outer_cyl.cut(inner_cyl)


def _tapered_square_tube(
    half_bot_out: float,
    half_top_out: float,
    wall: float,
    z0: float,
    z1: float,
) -> cq.Workplane:
    """Hollow tapered square tube (open top and bottom), lofted between two
    square sections. Walls have `wall` thickness; the inner cut is extended
    slightly past both ends for a clean boolean."""
    h = z1 - z0
    outer = (
        cq.Workplane("XY")
        .workplane(offset=z0)
        .rect(2.0 * half_bot_out, 2.0 * half_bot_out)
        .workplane(offset=h)
        .rect(2.0 * half_top_out, 2.0 * half_top_out)
        .loft(combine=True)
    )
    slope = (half_top_out - half_bot_out) / h
    ext = 0.002
    hb = half_bot_out - wall - slope * ext
    ht = half_top_out - wall + slope * ext
    inner = (
        cq.Workplane("XY")
        .workplane(offset=z0 - ext)
        .rect(2.0 * hb, 2.0 * hb)
        .workplane(offset=h + 2.0 * ext)
        .rect(2.0 * ht, 2.0 * ht)
        .loft(combine=True)
    )
    return outer.cut(inner)


def _flange_solid() -> cq.Workplane:
    """Polished square flange plate with beveled top edges and the round grate
    opening. World frame: z = FLANGE_BOT_Z .. TOTAL_H."""
    plate = (
        cq.Workplane("XY")
        .box(FLANGE_SIDE, FLANGE_SIDE, FLANGE_T)
        .translate((0.0, 0.0, FLANGE_BOT_Z + FLANGE_T / 2.0))
    )
    hole = (
        cq.Workplane("XY")
        .cylinder(FLANGE_T + 0.01, HOLE_R)
        .translate((0.0, 0.0, FLANGE_BOT_Z + FLANGE_T / 2.0))
    )
    plate = plate.cut(hole)
    # Slight bevel on the top edges (outer square rim and the opening edge).
    plate = plate.faces(">Z").edges().chamfer(FLANGE_CHAMFER)
    return plate


def _cup_solid() -> cq.Workplane:
    """Open-bottomed tapered drain cup with an inward seat plate at the top.
    World frame: z = 0 .. FLANGE_BOT_Z."""
    shell = _tapered_square_tube(CUP_BOT_HALF, CUP_TOP_HALF, CUP_WALL, 0.0, CUP_H)
    plate = (
        cq.Workplane("XY")
        .box(2.0 * SEAT_PLATE_HALF, 2.0 * SEAT_PLATE_HALF, SEAT_PLATE_T)
        .translate((0.0, 0.0, CUP_H - SEAT_PLATE_T / 2.0))
    )
    throat = (
        cq.Workplane("XY")
        .cylinder(SEAT_PLATE_T + 0.01, SEAT_HOLE_R)
        .translate((0.0, 0.0, CUP_H - SEAT_PLATE_T / 2.0))
    )
    plate = plate.cut(throat)
    return shell.union(plate)


def _liner_solid() -> cq.Workplane:
    """Dark cavity liner: a tapered square sleeve embedded against the cup's
    inner walls so the recess reads dark through the grate slots."""

    def cup_inner_half(z: float) -> float:
        return (CUP_BOT_HALF - CUP_WALL) + (CUP_TOP_HALF - CUP_BOT_HALF) * (z / CUP_H)

    half_bot = cup_inner_half(LINER_Z0) + LINER_EMBED
    half_top = cup_inner_half(LINER_Z1) + LINER_EMBED
    return _tapered_square_tube(half_bot, half_top, LINER_WALL, LINER_Z0, LINER_Z1)


def _rim_solid() -> cq.Workplane:
    """Grate seat ring: outer wall ring with an inward bottom lip the slotted
    disc rests on (L-shaped section). Local frame: bottom at z = 0."""
    wall = (
        cq.Workplane("XY")
        .cylinder(RIM_H, RIM_OUT_R)
        .translate((0.0, 0.0, RIM_H / 2.0))
        .cut(
            cq.Workplane("XY")
            .cylinder(RIM_H + 0.01, RIM_IN_R)
            .translate((0.0, 0.0, RIM_H / 2.0))
        )
    )
    lip = (
        cq.Workplane("XY")
        .cylinder(LIP_H, RIM_IN_R + 0.0002)
        .translate((0.0, 0.0, LIP_H / 2.0))
        .cut(
            cq.Workplane("XY")
            .cylinder(LIP_H + 0.01, LIP_IN_R)
            .translate((0.0, 0.0, LIP_H / 2.0))
        )
    )
    return wall.union(lip)


def _bridge_bar(index: int) -> cq.Workplane:
    """One radial bridge bar connecting the concentric rings across the annular
    slots. Bar spans from BRIDGE_INNER_R to BRIDGE_OUTER_R at the given angle."""
    angle = 360.0 * index / N_BRIDGES
    length = BRIDGE_OUTER_R - BRIDGE_INNER_R
    cx = (BRIDGE_INNER_R + BRIDGE_OUTER_R) / 2.0
    bar = (
        cq.Workplane("XY")
        .box(length, BRIDGE_W, GRATE_T)
        .translate((cx, 0.0, GRATE_T / 2.0))
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
    )
    return bar


def _grate_solid() -> cq.Workplane:
    """Round grate disc perforated with concentric ring slots: a series of
    annular gaps cut through the full disc thickness, with radial bridge bars
    reconnecting the rings across the slots.
    Local frame: disc bottom at z = 0."""
    disc = (
        cq.Workplane("XY")
        .cylinder(GRATE_T, GRATE_R)
        .translate((0.0, 0.0, GRATE_T / 2.0))
    )
    # Cut concentric ring slots.
    cutters: cq.Workplane | None = None
    for i, (inner_r, outer_r) in enumerate(_ring_specs()):
        ring = _ring_slot(inner_r, outer_r, GRATE_T + 0.006)
        cutters = ring if cutters is None else cutters.union(ring)
    assert cutters is not None
    disc_cut = disc.cut(cutters)

    # Add radial bridge bars to reconnect the annular bands.
    bridges: cq.Workplane | None = None
    for i in range(N_BRIDGES):
        bar = _bridge_bar(i)
        bridges = bar if bridges is None else bridges.union(bar)
    assert bridges is not None
    return disc_cut.union(bridges)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_stainless_floor_drain")
    model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("polished_steel", rgba=(0.81, 0.82, 0.84, 1.0))
    model.material("dark_cavity", rgba=(0.05, 0.05, 0.06, 1.0))

    body = model.part("drain_body")
    body.visual(
        mesh_from_cadquery(_flange_solid(), "flange_plate"),
        material="polished_steel",
        name="flange_plate",
    )
    body.visual(
        mesh_from_cadquery(_cup_solid(), "drain_cup"),
        material="brushed_steel",
        name="drain_cup",
    )
    body.visual(
        mesh_from_cadquery(_liner_solid(), "cavity_liner"),
        material="dark_cavity",
        name="cavity_liner",
    )

    rim = model.part("grate_rim")
    rim.visual(
        mesh_from_cadquery(_rim_solid(), "grate_rim"),
        material="brushed_steel",
        name="rim_ring",
    )

    grate = model.part("grate")
    grate.visual(
        mesh_from_cadquery(_grate_solid(), "grate_disc"),
        material="polished_steel",
        name="grate_disc",
    )

    # Lift: the whole removable grate (ring + disc) slides straight up +Z out
    # of its seat in the flange opening.
    model.articulation(
        "body_to_grate_rim",
        ArticulationType.PRISMATIC,
        parent=body,
        child=rim,
        origin=Origin(xyz=(0.0, 0.0, RIM_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.2, lower=0.0, upper=LIFT_TRAVEL),
    )

    # Twist-to-lock: the slotted disc rotates about the vertical axis through
    # its center, riding on the seat ring's inner lip.
    model.articulation(
        "rim_to_grate",
        ArticulationType.REVOLUTE,
        parent=rim,
        child=grate,
        origin=Origin(xyz=(0.0, 0.0, DISC_LOCAL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0, lower=-TWIST_LIMIT, upper=TWIST_LIMIT
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("drain_body")
    rim = object_model.get_part("grate_rim")
    grate = object_model.get_part("grate")
    lift = object_model.get_articulation("body_to_grate_rim")
    twist = object_model.get_articulation("rim_to_grate")

    # Intentional seated insertions (twist-to-lock grate dropped into its seat).
    ctx.allow_overlap(
        rim,
        body,
        reason="Seat ring bottoms 0.3 mm into the cup's seat plate (seated insertion).",
    )
    ctx.allow_overlap(
        grate,
        rim,
        reason="Grate disc rests 0.2 mm embedded on the seat ring's inner lip (seated).",
    )

    # --- Joint contract -----------------------------------------------------
    ctx.check(
        "lift joint is prismatic along +Z",
        lift.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(a, 6) for a in lift.axis) == (0.0, 0.0, 1.0),
        details=f"type={lift.articulation_type} axis={lift.axis}",
    )
    lift_lim = lift.motion_limits
    ctx.check(
        "lift travel is 0 .. 0.03 m",
        lift_lim is not None
        and abs(lift_lim.lower) < 1e-9
        and abs(lift_lim.upper - LIFT_TRAVEL) < 1e-9,
        details=f"lower={lift_lim.lower} upper={lift_lim.upper}",
    )
    ctx.check(
        "twist joint is revolute about +Z",
        twist.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(a, 6) for a in twist.axis) == (0.0, 0.0, 1.0),
        details=f"type={twist.articulation_type} axis={twist.axis}",
    )
    twist_lim = twist.motion_limits
    ctx.check(
        "twist range is about -90 .. +90 degrees",
        twist_lim is not None
        and abs(twist_lim.lower + TWIST_LIMIT) < 1e-6
        and abs(twist_lim.upper - TWIST_LIMIT) < 1e-6,
        details=f"lower={twist_lim.lower} upper={twist_lim.upper}",
    )

    # --- Base body geometry ---------------------------------------------------
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "flange footprint is ~0.12 m square",
        body_aabb is not None
        and abs((body_aabb[1][0] - body_aabb[0][0]) - FLANGE_SIDE) <= 0.002
        and abs((body_aabb[1][1] - body_aabb[0][1]) - FLANGE_SIDE) <= 0.002,
        details=str(body_aabb),
    )
    ctx.check(
        "overall height ~0.03 m with cup mouth at z=0",
        body_aabb is not None
        and abs(body_aabb[0][2]) <= 0.001
        and abs(body_aabb[1][2] - TOTAL_H) <= 0.001,
        details=str(body_aabb),
    )

    # Hero perforation: the concentric ring slots actually cut through the disc.
    disc_vol = float(_grate_solid().val().Volume())
    solid_vol = math.pi * GRATE_R**2 * GRATE_T
    ctx.check(
        "concentric ring slots perforate the grate disc",
        0.4 * solid_vol < disc_vol < 0.85 * solid_vol,
        details=f"disc_vol={disc_vol:.3e} solid_vol={solid_vol:.3e}",
    )
    ring_specs = _ring_specs()
    ctx.check(
        "five concentric ring slots",
        N_RINGS == 5 and len(ring_specs) == N_RINGS,
        details=f"n_rings={N_RINGS} specs={len(ring_specs)}",
    )
    # Each ring slot is an annular gap with the designed radial width.
    ctx.check(
        "ring slots have uniform radial width",
        all(abs((outer_r - inner_r) - RING_WIDTH) < 1e-9 for inner_r, outer_r in ring_specs),
        details=f"specs={ring_specs} width={RING_WIDTH}",
    )
    # All ring slots fit inside the disc radius.
    ctx.check(
        "all ring slots inside the grate disc",
        all(outer_r < GRATE_R for _, outer_r in ring_specs),
        details=f"max_outer_r={ring_specs[-1][1]} grate_r={GRATE_R}",
    )
    # Radial bridge bars reconnect the rings across the annular slots.
    ctx.check(
        "radial bridge bars authored for structural connectivity",
        N_BRIDGES == 4 and BRIDGE_W > 0.0,
        details=f"n_bridges={N_BRIDGES} bridge_w={BRIDGE_W}",
    )

    # Dark recessed cavity sits below the grate seat.
    liner = body.get_visual("cavity_liner")
    ctx.check("dark cavity liner authored", liner is not None)

    # --- Rest pose: grate seated flush in the flange opening -----------------
    with ctx.pose({lift: 0.0, twist: 0.0}):
        grate_aabb = ctx.part_world_aabb(grate)
        rim_aabb = ctx.part_world_aabb(rim)
        ctx.expect_contact(
            rim, body, contact_tol=0.0015, name="seat ring seated on the cup seat plate"
        )
        ctx.expect_contact(
            grate, rim, contact_tol=0.0015, name="grate disc seated on the ring lip"
        )
        ctx.expect_within(
            grate, body, axes="xy", margin=0.0, name="grate inside the flange opening (plan)"
        )
        ctx.expect_within(
            rim, body, axes="xy", margin=0.0, name="seat ring inside the flange opening (plan)"
        )
        ctx.expect_overlap(
            grate, rim, axes="xy", min_overlap=0.06, name="disc nested in the seat ring"
        )
        ctx.check(
            "grate top flush with the flange top",
            grate_aabb is not None
            and body_aabb is not None
            and abs(grate_aabb[1][2] - body_aabb[1][2]) <= 0.0015,
            details=f"grate_top={grate_aabb[1][2] if grate_aabb else None} "
            f"flange_top={body_aabb[1][2] if body_aabb else None}",
        )
        ctx.check(
            "seat ring top flush with the flange top",
            rim_aabb is not None
            and body_aabb is not None
            and abs(rim_aabb[1][2] - body_aabb[1][2]) <= 0.0015,
            details=f"rim_top={rim_aabb[1][2] if rim_aabb else None} "
            f"flange_top={body_aabb[1][2] if body_aabb else None}",
        )
        ctx.check(
            "grate disc is ~0.083 m diameter and ~0.004 m thick",
            grate_aabb is not None
            and abs((grate_aabb[1][0] - grate_aabb[0][0]) - 2.0 * GRATE_R) <= 0.002
            and abs((grate_aabb[1][2] - grate_aabb[0][2]) - GRATE_T) <= 0.001,
            details=str(grate_aabb),
        )

    rest_pos = ctx.part_world_position(grate)

    # --- Lift pose: grate rises straight out of the seat ---------------------
    with ctx.pose({lift: LIFT_TRAVEL, twist: 0.0}):
        up_pos = ctx.part_world_position(grate)
        up_grate = ctx.part_world_aabb(grate)
        up_rim = ctx.part_world_aabb(rim)
        ctx.check(
            "grate lifts straight up by the full travel",
            rest_pos is not None
            and up_pos is not None
            and abs((up_pos[2] - rest_pos[2]) - LIFT_TRAVEL) <= 0.001
            and abs(up_pos[0] - rest_pos[0]) <= 1e-6
            and abs(up_pos[1] - rest_pos[1]) <= 1e-6,
            details=f"rest={rest_pos} up={up_pos}",
        )
        ctx.check(
            "lifted grate assembly clears the flange seat",
            up_grate is not None
            and up_rim is not None
            and body_aabb is not None
            and up_rim[0][2] >= body_aabb[1][2] - 0.001
            and up_grate[0][2] >= body_aabb[1][2] - 0.001,
            details=f"rim_bottom={up_rim[0][2] if up_rim else None} "
            f"flange_top={body_aabb[1][2] if body_aabb else None}",
        )

    # --- Twist pose: grate spins in place about its vertical center axis ------
    with ctx.pose({lift: 0.0, twist: TWIST_LIMIT}):
        tw_pos = ctx.part_world_position(grate)
        tw_aabb = ctx.part_world_aabb(grate)
        ctx.check(
            "twisted grate stays centered in its seat",
            rest_pos is not None
            and tw_pos is not None
            and abs(tw_pos[0] - rest_pos[0]) <= 1e-6
            and abs(tw_pos[1] - rest_pos[1]) <= 1e-6
            and abs(tw_pos[2] - rest_pos[2]) <= 1e-6,
            details=f"rest={rest_pos} twisted={tw_pos}",
        )
        ctx.check(
            "twisted grate stays flush and inside the opening",
            tw_aabb is not None
            and body_aabb is not None
            and abs(tw_aabb[1][2] - body_aabb[1][2]) <= 0.0015,
            details=str(tw_aabb),
        )

    return ctx.report()


object_model = build_object_model()
