from __future__ import annotations

# Square stainless-steel bathroom floor drain, ~0.12 x 0.12 m and 0.03 m tall.
# A flat polished square flange (0.12 x 0.12 x 0.005 m, beveled top edges) sits
# on a shallow open-bottomed tapered drain cup. A circular opening (~0.095 m)
# in the flange holds a removable round grate: a thin seat ring carrying a
# perforated disc with a pinwheel (windmill) pattern of straight parallel slots
# in four quadrant groups, each group rotated 90 degrees from its neighbor.
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

# Pinwheel slot field: four quadrant groups of parallel slots, each group
# rotated 90 degrees from its neighbour.
FIELD_R = 0.0335  # slots stay inside this radius (solid outer margin)
N_GROUPS = 4
SLOTS_PER_GROUP = 5
SLOT_W = 0.004  # slot width
SLOT_PITCH = 0.0062  # spacing between parallel slots in one group
SLOT_Y0 = 0.0048  # first slot centerline offset from the group axis
CENTER_GAP = 0.0022  # solid web half-width kept at the disc center

# Pop-up center plug/stopper (variant: popup_center_plug).
PLUG_BORE_R = 0.003  # 6 mm bore through grate center for the plug stem
PLUG_CAP_R = 0.007  # 14 mm cap diameter (covers bore, sits on grate surface)
PLUG_CAP_T = 0.003  # 3 mm cap thickness
PLUG_STEM_R = 0.0025  # 5 mm stem (fits through bore with 0.5 mm clearance)
PLUG_STEM_H = 0.013  # stem extends 13 mm below cap bottom (stays engaged at max travel)
PLUG_TRAVEL = 0.008  # 8 mm pop-up travel

# Articulation ranges.
LIFT_TRAVEL = 0.030  # prismatic +Z travel
TWIST_LIMIT = math.pi / 2.0  # revolute +/- 90 degrees

RIM_REST_Z = SEAT_TOP_Z - RIM_SEAT_EMBED  # 0.0247: rim frame at rest


def _slot_specs() -> list[tuple[float, float]]:
    """(y_center, outer_x_extent) for one quadrant group of parallel slots.

    Slots run parallel to local X in the quadrant x < 0, y > 0, from the field
    circle inward to the solid center web. Rotated copies at 90-degree steps
    produce the windmill motif.
    """
    specs: list[tuple[float, float]] = []
    for i in range(SLOTS_PER_GROUP):
        y = SLOT_Y0 + i * SLOT_PITCH
        y_out = y + SLOT_W / 2.0
        x_outer = math.sqrt(FIELD_R**2 - y_out**2)
        specs.append((y, x_outer))
    return specs


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


def _grate_solid() -> cq.Workplane:
    """Round grate disc perforated with the pinwheel pattern: four quadrant
    groups of parallel rectangular slots, each group rotated 90 degrees.
    Local frame: disc bottom at z = 0."""
    disc = (
        cq.Workplane("XY")
        .cylinder(GRATE_T, GRATE_R)
        .translate((0.0, 0.0, GRATE_T / 2.0))
    )
    cutters: cq.Workplane | None = None
    for group in range(N_GROUPS):
        angle = 90.0 * group
        for y, x_outer in _slot_specs():
            length = x_outer - CENTER_GAP
            cx = -(x_outer + CENTER_GAP) / 2.0
            slot = (
                cq.Workplane("XY")
                .box(length, SLOT_W, GRATE_T + 0.006)
                .translate((cx, y, GRATE_T / 2.0))
                .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
            )
            cutters = slot if cutters is None else cutters.union(slot)
    assert cutters is not None
    slotted = disc.cut(cutters)
    # Central bore for the pop-up plug stem (variant: popup_center_plug).
    bore = (
        cq.Workplane("XY")
        .cylinder(GRATE_T + 0.006, PLUG_BORE_R)
        .translate((0.0, 0.0, GRATE_T / 2.0))
    )
    return slotted.cut(bore)


def _plug_solid() -> cq.Workplane:
    """Pop-up center plug: flat cap with chamfered top edge and a guide stem.
    Local frame: cap bottom at z = 0 (sits on the grate top surface at rest)."""
    cap = (
        cq.Workplane("XY")
        .cylinder(PLUG_CAP_T, PLUG_CAP_R)
        .translate((0.0, 0.0, PLUG_CAP_T / 2.0))
    )
    cap = cap.faces(">Z").edges().chamfer(0.0008)
    stem = (
        cq.Workplane("XY")
        .cylinder(PLUG_STEM_H, PLUG_STEM_R)
        .translate((0.0, 0.0, -PLUG_STEM_H / 2.0))
    )
    return cap.union(stem)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_stainless_floor_drain")
    model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("polished_steel", rgba=(0.81, 0.82, 0.84, 1.0))
    model.material("dark_cavity", rgba=(0.05, 0.05, 0.06, 1.0))
    model.material("satin_steel", rgba=(0.66, 0.67, 0.70, 1.0))

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

    # Pop-up center plug (variant: popup_center_plug).
    plug = model.part("center_plug")
    plug.visual(
        mesh_from_cadquery(_plug_solid(), "center_plug"),
        material="satin_steel",
        name="plug_cap",
    )

    # The plug lifts straight up out of its seat in the grate center.
    model.articulation(
        "grate_to_plug",
        ArticulationType.PRISMATIC,
        parent=grate,
        child=plug,
        origin=Origin(xyz=(0.0, 0.0, GRATE_T)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=0.1, lower=0.0, upper=PLUG_TRAVEL
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("drain_body")
    rim = object_model.get_part("grate_rim")
    grate = object_model.get_part("grate")
    plug = object_model.get_part("center_plug")
    lift = object_model.get_articulation("body_to_grate_rim")
    twist = object_model.get_articulation("rim_to_grate")
    popup = object_model.get_articulation("grate_to_plug")

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
    ctx.allow_overlap(
        plug,
        grate,
        elem_a="plug_cap",
        elem_b="grate_disc",
        reason="Plug cap bottom seats on the grate top surface (contact/seated insertion).",
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

    # Hero perforation: the pinwheel slots actually cut through the disc.
    disc_vol = float(_grate_solid().val().Volume())
    solid_vol = math.pi * GRATE_R**2 * GRATE_T
    ctx.check(
        "pinwheel slots perforate the grate disc",
        0.5 * solid_vol < disc_vol < 0.82 * solid_vol,
        details=f"disc_vol={disc_vol:.3e} solid_vol={solid_vol:.3e}",
    )
    ctx.check(
        "four quadrant groups of 5 slots each",
        N_GROUPS == 4 and len(_slot_specs()) == SLOTS_PER_GROUP == 5,
        details=f"groups={N_GROUPS} slots_per_group={len(_slot_specs())}",
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

    # --- Pop-up center plug (variant: popup_center_plug) --------------------
    ctx.check(
        "popup joint is prismatic along +Z",
        popup.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(a, 6) for a in popup.axis) == (0.0, 0.0, 1.0),
        details=f"type={popup.articulation_type} axis={popup.axis}",
    )
    popup_lim = popup.motion_limits
    ctx.check(
        "popup travel is 0 .. PLUG_TRAVEL",
        popup_lim is not None
        and abs(popup_lim.lower) < 1e-9
        and abs(popup_lim.upper - PLUG_TRAVEL) < 1e-9,
        details=f"lower={popup_lim.lower} upper={popup_lim.upper}",
    )

    plug_aabb = ctx.part_world_aabb(plug)
    ctx.check(
        "plug cap is ~14 mm diameter",
        plug_aabb is not None
        and abs((plug_aabb[1][0] - plug_aabb[0][0]) - 2.0 * PLUG_CAP_R) <= 0.003,
        details=str(plug_aabb),
    )

    # Rest pose: plug cap seated on grate surface.
    plug_rest_pos = ctx.part_world_position(plug)
    with ctx.pose({lift: 0.0, twist: 0.0, popup: 0.0}):
        plug_rest_pos = ctx.part_world_position(plug)
        grate_rest = ctx.part_world_aabb(grate)
        # Plug frame origin = cap bottom; at rest it should sit at grate top.
        ctx.check(
            "plug cap bottom sits at grate top when closed",
            plug_rest_pos is not None
            and grate_rest is not None
            and abs(plug_rest_pos[2] - grate_rest[1][2]) <= 0.002,
            details=f"plug_origin_z={plug_rest_pos[2] if plug_rest_pos else None} "
            f"grate_top={grate_rest[1][2] if grate_rest else None}",
        )
        ctx.expect_contact(
            plug, grate, elem_a="plug_cap", elem_b="grate_disc",
            contact_tol=0.002,
            name="plug cap in contact with grate disc at rest",
        )

    # Popup pose: plug rises above the grate.
    with ctx.pose({lift: 0.0, twist: 0.0, popup: PLUG_TRAVEL}):
        plug_up_pos = ctx.part_world_position(plug)
        grate_at = ctx.part_world_aabb(grate)
        ctx.check(
            "plug rises by the full popup travel",
            plug_rest_pos is not None
            and plug_up_pos is not None
            and abs((plug_up_pos[2] - plug_rest_pos[2]) - PLUG_TRAVEL) <= 0.001
            and abs(plug_up_pos[0] - plug_rest_pos[0]) <= 1e-6
            and abs(plug_up_pos[1] - plug_rest_pos[1]) <= 1e-6,
            details=f"rest={plug_rest_pos} up={plug_up_pos}",
        )
        ctx.check(
            "popped plug cap bottom clears grate top",
            plug_up_pos is not None
            and grate_at is not None
            and plug_up_pos[2] >= grate_at[1][2] + PLUG_TRAVEL - 0.002,
            details=f"plug_origin_z={plug_up_pos[2] if plug_up_pos else None} "
            f"grate_top={grate_at[1][2] if grate_at else None}",
        )

    # Plug stays centered on the grate when the grate twists.
    with ctx.pose({lift: 0.0, twist: TWIST_LIMIT, popup: 0.0}):
        plug_tw = ctx.part_world_position(plug)
        ctx.check(
            "plug stays centered when grate twists",
            plug_rest_pos is not None
            and plug_tw is not None
            and abs(plug_tw[0] - plug_rest_pos[0]) <= 1e-6
            and abs(plug_tw[1] - plug_rest_pos[1]) <= 1e-6,
            details=f"rest={plug_rest_pos} twisted={plug_tw}",
        )

    return ctx.report()


object_model = build_object_model()
