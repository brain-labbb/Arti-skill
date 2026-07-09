from __future__ import annotations

# Square stainless-steel bathroom floor drain, ~0.12 x 0.12 m and 0.03 m tall.
# Variant: hinged flip grate (replaces twist-lock + lift-out).
# A flat polished square flange (0.12 x 0.12 x 0.005 m, beveled top edges) sits
# on a shallow open-bottomed tapered drain cup. A circular opening (~0.095 m)
# in the flange holds a round grate disc with a pinwheel (windmill) pattern of
# straight parallel slots in four quadrant groups, each rotated 90 degrees.
# The grate is hinged on one edge and flips up to open for cleaning.
# Articulation: REVOLUTE about horizontal X axis at the -Y seat edge, 0..+2.0 rad.
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

# Dark cavity liner inside the cup.
LINER_WALL = 0.0015
LINER_Z0 = 0.002
LINER_Z1 = FLANGE_BOT_Z - SEAT_PLATE_T  # 0.0215
LINER_EMBED = 0.0005

# Seat ring (fixed, part of the body in the hinged variant).
RIM_OUT_R = 0.0472
RIM_IN_R = 0.0420
RIM_H = 0.0053
LIP_IN_R = 0.0360
LIP_H = 0.0015
RIM_SEAT_EMBED = 0.0003

# Grate disc dimensions (identical to parent pinwheel pattern).
GRATE_R = 0.0415
GRATE_T = 0.004

# Pinwheel slot field (identical to parent).
FIELD_R = 0.0335
N_GROUPS = 4
SLOTS_PER_GROUP = 5
SLOT_W = 0.004
SLOT_PITCH = 0.0062
SLOT_Y0 = 0.0048
CENTER_GAP = 0.0022

# Hinge geometry: barrel pin along X at the -Y edge of the seat opening.
HINGE_R = 0.002  # hinge pin/barrel radius
HINGE_LEN = 0.030  # barrel length along X
HINGE_Y = -GRATE_R  # hinge at the -Y edge of the grate disc
HINGE_Z = SEAT_TOP_Z  # pin center at seat surface level

# Grate disc offset in its local frame (origin at hinge pin center, q=0 closed).
DISC_CY = GRATE_R  # disc center Y offset from hinge
DISC_CZ = GRATE_T / 2.0 + 0.001  # disc center Z above pin = 0.003

# Hinge knuckle on the grate side.
KNUCKLE_R = HINGE_R + 0.001  # wraps around the barrel
KNUCKLE_LEN = 0.018

# Hinge bracket ears on the body side (extend downward from pin into the cup).
BRACKET_W = 0.006  # bracket width in Y
BRACKET_DROP = 0.005  # ear height below pin center
BRACKET_T = 0.004  # bracket thickness (along X, each ear)

# Flip range.
FLIP_UPPER = 2.0  # ~115 degrees, realistic flip-up travel

RIM_REST_Z = SEAT_TOP_Z - RIM_SEAT_EMBED


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
    square sections."""
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
    """Polished square flange plate with beveled top edges and round opening."""
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
    plate = plate.faces(">Z").edges().chamfer(FLANGE_CHAMFER)
    return plate


def _cup_solid() -> cq.Workplane:
    """Open-bottomed tapered drain cup with an inward seat plate at the top."""
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
    """Dark cavity liner embedded against the cup's inner walls."""

    def cup_inner_half(z: float) -> float:
        return (CUP_BOT_HALF - CUP_WALL) + (CUP_TOP_HALF - CUP_BOT_HALF) * (z / CUP_H)

    half_bot = cup_inner_half(LINER_Z0) + LINER_EMBED
    half_top = cup_inner_half(LINER_Z1) + LINER_EMBED
    return _tapered_square_tube(half_bot, half_top, LINER_WALL, LINER_Z0, LINER_Z1)


def _rim_solid() -> cq.Workplane:
    """Fixed seat ring: outer wall ring with inward bottom lip.
    World frame: bottom at RIM_REST_Z."""
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
    ring = wall.union(lip)
    return ring.translate((0.0, 0.0, RIM_REST_Z))


def _hinge_bracket_solid() -> cq.Workplane:
    """Two bracket ears on the body that carry the hinge pin, mounted at the
    -Y edge of the seat opening. Ears extend downward from the pin into the
    cup region. Pin center at HINGE_Z. World frame."""
    ear_half_gap = HINGE_LEN / 2.0 - BRACKET_T / 2.0
    ears = None
    for sign in (-1.0, 1.0):
        # Ear body: extends from pin center downward.
        ear = (
            cq.Workplane("XY")
            .box(BRACKET_T, BRACKET_W, BRACKET_DROP)
            .translate((
                sign * (ear_half_gap + BRACKET_T / 2.0),
                HINGE_Y - BRACKET_W / 2.0 + BRACKET_W / 2.0,
                HINGE_Z - BRACKET_DROP / 2.0,
            ))
        )
        # Round the top of each ear around the pin axis.
        cap = (
            cq.Workplane("YZ")
            .cylinder(BRACKET_T, BRACKET_W / 2.0)
            .translate((
                sign * (ear_half_gap + BRACKET_T / 2.0),
                HINGE_Y,
                HINGE_Z,
            ))
        )
        ear = ear.union(cap)
        # Bore a hole for the hinge pin.
        hole = (
            cq.Workplane("YZ")
            .cylinder(BRACKET_T + 0.01, HINGE_R * 1.1)
            .translate((
                sign * (ear_half_gap + BRACKET_T / 2.0),
                HINGE_Y,
                HINGE_Z,
            ))
        )
        ear = ear.cut(hole)
        ears = ear if ears is None else ears.union(ear)
    # Hinge pin through both ears.
    pin = (
        cq.Workplane("YZ")
        .cylinder(HINGE_LEN, HINGE_R)
        .translate((0.0, HINGE_Y, HINGE_Z))
    )
    return ears.union(pin)


def _grate_solid() -> cq.Workplane:
    """Round grate disc with pinwheel slots, offset from hinge origin at local
    (0, 0, 0). Local frame: hinge pin center, closed pose (q=0).
    The disc center is at local (0, DISC_CY, DISC_CZ)."""
    # Disc centered at (0, DISC_CY, DISC_CZ) in local frame.
    disc = (
        cq.Workplane("XY")
        .cylinder(GRATE_T, GRATE_R)
        .translate((0.0, DISC_CY, DISC_CZ))
    )

    # Cut pinwheel slots (same pattern, offset to disc center).
    cutters: cq.Workplane | None = None
    for group in range(N_GROUPS):
        angle = 90.0 * group
        for y, x_outer in _slot_specs():
            length = x_outer - CENTER_GAP
            cx = -(x_outer + CENTER_GAP) / 2.0
            slot = (
                cq.Workplane("XY")
                .box(length, SLOT_W, GRATE_T + 0.006)
                .translate((cx, y, 0.0))
                .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
                .translate((0.0, DISC_CY, DISC_CZ))
            )
            cutters = slot if cutters is None else cutters.union(slot)
    assert cutters is not None
    disc = disc.cut(cutters)

    # Hinge knuckle at local origin wrapping around the pin.
    knuckle = cq.Workplane("YZ").cylinder(KNUCKLE_LEN, KNUCKLE_R)
    pin_hole = cq.Workplane("YZ").cylinder(KNUCKLE_LEN + 0.01, HINGE_R * 1.1)
    knuckle = knuckle.cut(pin_hole)

    # Connecting tab from knuckle to the disc near edge.
    tab_dy = DISC_CY - GRATE_R  # gap from pin center to disc near edge
    tab_height = DISC_CZ + GRATE_T / 2.0  # top of disc above pin
    tab = (
        cq.Workplane("XY")
        .box(KNUCKLE_LEN * 0.6, abs(tab_dy) + KNUCKLE_R, tab_height)
        .translate((0.0, tab_dy / 2.0, tab_height / 2.0))
    )

    return disc.union(knuckle).union(tab)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_stainless_floor_drain_hinged")
    model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("polished_steel", rgba=(0.81, 0.82, 0.84, 1.0))
    model.material("dark_cavity", rgba=(0.05, 0.05, 0.06, 1.0))
    model.material("hinge_steel", rgba=(0.68, 0.69, 0.71, 1.0))

    # Root body: flange, cup, liner, fixed seat ring, hinge bracket.
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
    body.visual(
        mesh_from_cadquery(_rim_solid(), "seat_ring"),
        material="brushed_steel",
        name="seat_ring",
    )
    body.visual(
        mesh_from_cadquery(_hinge_bracket_solid(), "hinge_bracket"),
        material="hinge_steel",
        name="hinge_bracket",
    )

    # Hinged grate: slotted disc + hinge knuckle, single moving part.
    grate = model.part("grate")
    grate.visual(
        mesh_from_cadquery(_grate_solid(), "grate_disc"),
        material="polished_steel",
        name="grate_disc",
    )

    # Flip hinge: REVOLUTE about horizontal X axis at the -Y seat edge.
    # Positive q flips the far (+Y) edge of the grate upward.
    model.articulation(
        "body_to_grate",
        ArticulationType.REVOLUTE,
        parent=body,
        child=grate,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0, velocity=1.5, lower=0.0, upper=FLIP_UPPER
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("drain_body")
    grate = object_model.get_part("grate")
    hinge = object_model.get_articulation("body_to_grate")

    # Intentional seated insertion: grate disc sits inside the seat ring
    # opening, resting on the ring's inner lip.
    ctx.allow_overlap(
        grate,
        body,
        elem_a="grate_disc",
        elem_b="seat_ring",
        reason="Grate disc is seated inside the seat ring opening, resting on the inner lip.",
    )
    # Intentional captured pin: grate knuckle wraps around the hinge pin barrel.
    ctx.allow_overlap(
        grate,
        body,
        elem_a="grate_disc",
        elem_b="hinge_bracket",
        reason="Grate knuckle wraps around the hinge pin barrel (captured pin fit).",
    )

    # --- Joint contract -----------------------------------------------------
    ctx.check(
        "hinge joint is revolute about +X",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(a, 6) for a in hinge.axis) == (1.0, 0.0, 0.0),
        details=f"type={hinge.articulation_type} axis={hinge.axis}",
    )
    hinge_lim = hinge.motion_limits
    ctx.check(
        "hinge range is 0 .. ~2.0 rad (flip-up)",
        hinge_lim is not None
        and abs(hinge_lim.lower) < 1e-9
        and abs(hinge_lim.upper - FLIP_UPPER) < 1e-6,
        details=f"lower={hinge_lim.lower} upper={hinge_lim.upper}",
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
        and abs(body_aabb[1][2] - TOTAL_H) <= 0.002,
        details=str(body_aabb),
    )

    # Pinwheel perforation.
    solid_vol = math.pi * GRATE_R**2 * GRATE_T
    # Build a standalone centered disc to measure slot removal fraction.
    ref_disc = (
        cq.Workplane("XY")
        .cylinder(GRATE_T, GRATE_R)
        .translate((0.0, 0.0, GRATE_T / 2.0))
    )
    ref_cutters: cq.Workplane | None = None
    for group in range(N_GROUPS):
        angle = 90.0 * group
        for y, x_outer in _slot_specs():
            length = x_outer - CENTER_GAP
            cx = -(x_outer + CENTER_GAP) / 2.0
            slot = (
                cq.Workplane("XY")
                .box(length, SLOT_W, GRATE_T + 0.006)
                .translate((cx, y, 0.0))
                .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
                .translate((0.0, 0.0, GRATE_T / 2.0))
            )
            ref_cutters = slot if ref_cutters is None else ref_cutters.union(slot)
    assert ref_cutters is not None
    ref_slotted = ref_disc.cut(ref_cutters)
    disc_vol = float(ref_slotted.val().Volume())
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

    # Dark recessed cavity.
    liner = body.get_visual("cavity_liner")
    ctx.check("dark cavity liner authored", liner is not None)

    # Hinge bracket authored on body.
    bracket = body.get_visual("hinge_bracket")
    ctx.check("hinge bracket authored on body", bracket is not None)

    # --- Rest pose (q=0): grate closed, seated flush -------------------------
    with ctx.pose({hinge: 0.0}):
        grate_aabb = ctx.part_world_aabb(grate)
        ctx.check(
            "grate top near flange top when closed",
            grate_aabb is not None
            and body_aabb is not None
            and abs(grate_aabb[1][2] - body_aabb[1][2]) <= 0.005,
            details=f"grate_top={grate_aabb[1][2] if grate_aabb else None} "
            f"flange_top={body_aabb[1][2] if body_aabb else None}",
        )
        ctx.check(
            "grate disc is ~0.083 m diameter",
            grate_aabb is not None
            and abs((grate_aabb[1][0] - grate_aabb[0][0]) - 2.0 * GRATE_R) <= 0.004,
            details=str(grate_aabb),
        )
        ctx.expect_within(
            grate, body, axes="xy", margin=0.002,
            name="grate fits inside the flange opening (plan)",
        )
        ctx.expect_overlap(
            grate, body, axes="xy", min_overlap=0.06,
            name="grate covers the drain opening in plan",
        )

    # --- Open pose: grate flipped up -----------------------------------------
    with ctx.pose({hinge: 0.0}):
        rest_aabb = ctx.part_world_aabb(grate)

    with ctx.pose({hinge: FLIP_UPPER}):
        open_aabb = ctx.part_world_aabb(grate)
        ctx.check(
            "flipped grate rises well above the flange",
            open_aabb is not None
            and body_aabb is not None
            and open_aabb[1][2] > body_aabb[1][2] + 0.03,
            details=f"grate_top={open_aabb[1][2] if open_aabb else None} "
            f"flange_top={body_aabb[1][2] if body_aabb else None}",
        )
        ctx.check(
            "flipped grate top is higher than rest top",
            rest_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] > rest_aabb[1][2] + 0.02,
            details=f"rest_top={rest_aabb[1][2] if rest_aabb else None} "
            f"open_top={open_aabb[1][2] if open_aabb else None}",
        )
        ctx.check(
            "flipped grate Z span proves disc swung upward",
            rest_aabb is not None
            and open_aabb is not None
            and (open_aabb[1][2] - open_aabb[0][2])
            > (rest_aabb[1][2] - rest_aabb[0][2]) + 0.03,
            details=f"rest_span={rest_aabb[1][2] - rest_aabb[0][2] if rest_aabb else None} "
            f"open_span={open_aabb[1][2] - open_aabb[0][2] if open_aabb else None}",
        )

    # --- Mid-open pose: partial flip -----------------------------------------
    with ctx.pose({hinge: math.pi / 4.0}):
        mid_aabb = ctx.part_world_aabb(grate)
        ctx.check(
            "mid-flip grate is partially raised",
            mid_aabb is not None
            and body_aabb is not None
            and mid_aabb[1][2] > body_aabb[1][2] + 0.005,
            details=str(mid_aabb),
        )

    return ctx.report()


object_model = build_object_model()
