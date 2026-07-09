from __future__ import annotations

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
# Surgical / gynecological exam stool.
#
# Reference (picture/Science/Surgical chair/001.png): a low padded stool used
# in surgery / exam rooms. It has:
#   - a round blue upholstered seat with a curved backrest behind it
#   - two padded "wing" armrests that swing out on vertical chrome posts, each
#     with a small adjustment lever/knob
#   - a telescoping height pillar wrapped in a black ribbed bellows boot
#   - a yellow conical hub above a 5-star chrome caster base
#   - twin foot pedals on the base
#
# Primary mechanisms modeled:
#   1. seat_lift   PRISMATIC  - the seat raises/lowers on the gas pillar (+Z)
#   2. left_arm_swing  REVOLUTE - left armrest swings out about its vertical post
#   3. right_arm_swing REVOLUTE - right armrest swings out about its vertical post
# ---------------------------------------------------------------------------

# Real-world scale (meters).
SEAT_RADIUS = 0.205
SEAT_THICK = 0.085
SEAT_TOP_Z = 0.560  # seat top height at the collapsed (lowest) pose

COLUMN_RADIUS = 0.040  # outer telescoping sleeve radius
HUB_TOP_Z = 0.250  # top of yellow conical hub
BASE_SPAN = 0.560  # caster-to-caster diameter of the 5-star base

ARM_POST_HEIGHT = 0.135
ARM_PAD_LEN = 0.230
ARM_PAD_W = 0.062
ARM_PAD_H = 0.058

LIFT_TRAVEL = 0.080  # height adjustment range (gas-pillar telescopes inside)

# --- Base assembly z-budget (everything in world meters, ground at z=0) ---
WHEEL_R = 0.030  # caster wheel radius; wheel centers at z=WHEEL_R, bottoms at 0
WHEEL_W = 0.014  # one wheel's width along its (horizontal, tangential) axis
WHEEL_GAP = 0.009  # half the clear gap between the twin wheels (yoke half-width)
CASTER_R = 0.240  # radial distance of each caster stem from the column axis
LEG_ROOT_X = 0.040  # leg root cross-section x (embedded in the hub cylinder)
LEG_TIP_X = 0.250  # leg tip cross-section x
LEG_ROOT_Z = (0.060, 0.125)  # leg root bottom/top z
LEG_TIP_Z = (0.080, 0.112)  # leg tip bottom/top z
HUB_CYL_R = 0.062  # central hub cylinder radius
HUB_CYL_Z = (0.060, 0.135)  # central hub cylinder bottom/top z
CONE_Z = (0.120, HUB_TOP_Z)  # yellow cone bottom (sunk into hub) / top z
CONE_R = (0.088, 0.052)  # yellow cone bottom/top radius
COLLAR_Z = (HUB_TOP_Z, HUB_TOP_Z + 0.022)  # collar atop the cone
COLLAR_R = 0.064  # collar radius: wider than the bellows bore so they mate
BELLOWS_Z0 = 0.258  # bellows bottom: overlaps the collar (no gap)
PEDAL_Z = 0.105  # pedal stem axis height (inside the hub cylinder z-range)

TOL = dict(tolerance=0.0009, angular_tolerance=0.09)


# ---------------------------------------------------------------------------
# Geometry builders (CadQuery, authored directly in meters).
# ---------------------------------------------------------------------------
def _star_base_mesh(name: str):
    """Five-spoke chrome caster base with a central column socket.

    Each spoke is a loft between two vertical (YZ) cross-sections: a tall wide
    root sunk into the hub cylinder and a shorter narrower tip that the caster
    stem plugs into from below."""
    arms = 5
    root_hw, tip_hw = 0.038, 0.020  # half-widths (y) at root / tip
    root = [
        (-root_hw, LEG_ROOT_Z[0]),
        (root_hw, LEG_ROOT_Z[0]),
        (root_hw, LEG_ROOT_Z[1]),
        (-root_hw, LEG_ROOT_Z[1]),
    ]
    tip = [
        (-tip_hw, LEG_TIP_Z[0]),
        (tip_hw, LEG_TIP_Z[0]),
        (tip_hw, LEG_TIP_Z[1]),
        (-tip_hw, LEG_TIP_Z[1]),
    ]
    base = None
    for i in range(arms):
        ang = 2.0 * math.pi * i / arms + math.radians(90.0)
        leg = (
            cq.Workplane("YZ", origin=(LEG_ROOT_X, 0.0, 0.0))
            .polyline(root)
            .close()
            .workplane(offset=LEG_TIP_X - LEG_ROOT_X)
            .polyline(tip)
            .close()
            .loft(combine=True)
        )
        leg = leg.rotate((0, 0, 0), (0, 0, 1), math.degrees(ang))
        base = leg if base is None else base.union(leg)

    # Central hub cylinder that ties all five spokes together and sockets the
    # yellow cone / column above it.
    hub = (
        cq.Workplane("XY", origin=(0.0, 0.0, HUB_CYL_Z[0]))
        .circle(HUB_CYL_R)
        .extrude(HUB_CYL_Z[1] - HUB_CYL_Z[0])
    )
    base = base.union(hub)
    return mesh_from_cadquery(base, name, **TOL)


def _caster_mesh(name: str):
    """One twin-wheel caster, authored so the wheel bottoms sit exactly at z=0.

    Local frame: +X points radially outward (set by the placement rpy), the
    twin wheels spin about a horizontal axis along local +Y (tangential), and
    the vertical stem along +Z plugs up into the spoke tip."""
    wheels = None
    for s in (-1.0, 1.0):
        w = (
            cq.Workplane("XZ", origin=(0.0, s * (WHEEL_GAP + WHEEL_W / 2.0), WHEEL_R))
            .circle(WHEEL_R)
            .extrude(WHEEL_W / 2.0, both=True)
        )
        wheels = w if wheels is None else wheels.union(w)
    # Axle through both wheels and the central yoke.
    axle = (
        cq.Workplane("XZ", origin=(0.0, 0.0, WHEEL_R))
        .circle(0.007)
        .extrude(WHEEL_GAP + WHEEL_W, both=True)
    )
    # Central yoke block between the twin wheels, hanging off the stem.
    yoke = (
        cq.Workplane("XY", origin=(0.0, 0.0, 0.052))
        .box(0.034, 2.0 * WHEEL_GAP, 0.045)
    )
    # Vertical stem rising from the yoke into the spoke tip (no skew).
    stem = (
        cq.Workplane("XY", origin=(0.0, 0.0, 0.070))
        .circle(0.009)
        .extrude(0.040)
    )
    cas = wheels.union(axle).union(yoke).union(stem)
    return mesh_from_cadquery(cas, name, **TOL)


def _pedal_mesh(name: str):
    """Black foot pedal: a horizontal stem rooted in the hub cylinder carrying
    a slightly tilted rounded paddle at its outboard end (nothing floats).

    Local frame: stem axis along +X at z=0; placement puts z=0 at PEDAL_Z."""
    stem = (
        cq.Workplane("YZ", origin=(0.030, 0.0, 0.0))
        .circle(0.008)
        .extrude(0.175)
    )
    paddle = (
        cq.Workplane("XY")
        .box(0.075, 0.085, 0.014)
        .edges("|Z")
        .fillet(0.010)
    )
    # Tilt the paddle toe-up and seat its inboard edge onto the stem end.
    paddle = paddle.rotate((0, 0, 0), (0, 1, 0), -12.0).translate((0.210, 0.0, 0.004))
    ped = stem.union(paddle)
    return mesh_from_cadquery(ped, name, **TOL)


def _hub_cone_mesh(name: str):
    """Yellow conical hub: sunk into the star-base hub cylinder at the bottom,
    capped by a collar that the bellows boot overlaps at the top (no gaps).
    Authored directly at world z (visual placed with identity origin)."""
    cone = (
        cq.Workplane("XY", origin=(0.0, 0.0, CONE_Z[0]))
        .circle(CONE_R[0])
        .workplane(offset=CONE_Z[1] - CONE_Z[0])
        .circle(CONE_R[1])
        .loft(combine=True)
    )
    collar = (
        cq.Workplane("XY", origin=(0.0, 0.0, COLLAR_Z[0]))
        .circle(COLLAR_R)
        .extrude(COLLAR_Z[1] - COLLAR_Z[0])
    )
    return mesh_from_cadquery(cone.union(collar), name, **TOL)


def _bellows_mesh(name: str):
    """Black ribbed accordion boot around the telescoping pillar.

    Built as a closed half-section (outer accordion wall + straight inner bore
    wall) revolved about the Z axis, so it reads as a hollow ribbed sleeve."""
    z0 = 0.0
    z1 = 0.205
    ribs = 8
    # The boot wraps OUTSIDE the seat collar (collar outer radius ~0.054), so
    # its bore clears the collar and there is no interpenetration.
    outer = 0.080
    inner = 0.067
    bore = 0.060
    wall = 0.006  # radial wall thickness so the boot is a true shell

    # Outer accordion wall, sampled bottom -> top (radius, z).
    outer_wall: list[tuple[float, float]] = []
    n = ribs * 2
    for k in range(n + 1):
        z = z0 + (z1 - z0) * k / n
        r = outer if (k % 2 == 1) else inner
        outer_wall.append((r, z))

    # Inner bore wall, sampled top -> bottom so the loop is continuous.
    inner_wall = [(bore, z1), (bore, z0)]

    poly = outer_wall + inner_wall
    boot = (
        cq.Workplane("XZ")
        .polyline(poly)
        .close()
        .revolve(360.0, (0, 0, 0), (0, 1, 0))
    )
    # `wall` retained for documentation of intent; bore/inner already enforce it.
    _ = wall
    return mesh_from_cadquery(boot, name, **TOL)


def _column_sleeve_mesh(name: str):
    """Chrome telescoping sleeve riser that the seat collar slides on. Its top
    stops just below the seat cushion so only the hidden collar grips it."""
    sleeve = (
        cq.Workplane("XY", origin=(0.0, 0.0, 0.0))
        .circle(COLUMN_RADIUS)
        .circle(COLUMN_RADIUS - 0.008)
        .extrude(0.210)
    )
    return mesh_from_cadquery(sleeve, name, **TOL)


def _seat_mesh(name: str):
    """Round upholstered seat cushion with a soft domed top and rounded rim."""
    # Lower disc + bulged cushion top.
    base_disc = (
        cq.Workplane("XY", origin=(0.0, 0.0, 0.0))
        .circle(SEAT_RADIUS)
        .extrude(0.030)
    )
    cushion = (
        cq.Workplane("XY", origin=(0.0, 0.0, 0.030))
        .circle(SEAT_RADIUS)
        .workplane(offset=SEAT_THICK - 0.030)
        .circle(SEAT_RADIUS - 0.045)
        .loft(combine=True)
    )
    seat = base_disc.union(cushion)
    seat = seat.edges(">Z").fillet(0.018)
    return mesh_from_cadquery(seat, name, **TOL)


def _seat_mount_mesh(name: str):
    """Hidden underside collar that grips the column sleeve. Its bore sits in
    light interference with the sleeve wall so the telescoping fit is a real
    captured contact (declared via allow_overlap), not a floating clearance."""
    tube = (
        cq.Workplane("XY", origin=(0.0, 0.0, -0.145))
        .circle(COLUMN_RADIUS + 0.014)
        .circle(COLUMN_RADIUS - 0.002)
        .extrude(0.148)
    )
    # A short top plate ties the collar to the cushion underside (real contact).
    plate = (
        cq.Workplane("XY", origin=(0.0, 0.0, -0.004))
        .circle(COLUMN_RADIUS + 0.014)
        .extrude(0.006)
    )
    return mesh_from_cadquery(tube.union(plate), name, **TOL)


def _backrest_mesh(name: str):
    """Curved padded backrest behind the seat."""
    pad = (
        cq.Workplane("XZ", origin=(0.0, 0.0, 0.0))
        .moveTo(-0.150, 0.0)
        .threePointArc((0.0, 0.030), (0.150, 0.0))
        .lineTo(0.150, 0.140)
        .threePointArc((0.0, 0.175), (-0.150, 0.140))
        .close()
        .extrude(0.060, both=True)
    )
    pad = pad.edges("|Y").fillet(0.020)
    return mesh_from_cadquery(pad, name, **TOL)


def _backrest_arm_mesh(name: str):
    """Chrome stalk that carries the backrest up off the seat rim.

    It is authored long on purpose: the bottom end sinks into the cushion base
    disc and the top end embeds into the backrest pad so the chain
    cushion -> stalk -> pad is continuous solid contact (no floating)."""
    stalk = (
        cq.Workplane("XY", origin=(0.0, 0.0, -0.060))
        .circle(0.012)
        .extrude(0.205)
    )
    return mesh_from_cadquery(stalk, name, **TOL)


# Geometry constants shared by the arm frame and its pad so the two visuals
# (chrome frame, upholstered pad) line up exactly.
_ARM_STALK_LEN = ARM_PAD_LEN * 0.55
_ARM_PAD_CX = ARM_PAD_LEN * 0.45
_ARM_PAD_CZ = ARM_POST_HEIGHT + ARM_PAD_H / 2.0 - 0.020


def _arm_frame_mesh(name: str, *, sign: float):
    """Chrome frame of one armrest: vertical swing post, base clamp collar,
    horizontal carrier stalk, and a small adjustment lever + knob. The post
    axis is local +Z through (0,0,*); the pad extends along +X from it."""
    # Vertical chrome post (the swing axis is local +Z through this post).
    post = (
        cq.Workplane("XY", origin=(0.0, 0.0, 0.0))
        .circle(0.013)
        .extrude(ARM_POST_HEIGHT)
    )
    # A clamp collar at the post base wraps the seat rim so the arm is mounted,
    # not floating. It reaches inward (-X, toward the seat center) to contact
    # the cushion side wall.
    collar = (
        cq.Workplane("XY", origin=(-0.016, 0.0, 0.010))
        .box(0.052, 0.030, 0.034)
        .edges("|Z")
        .fillet(0.008)
    )
    post = post.union(collar)
    # Horizontal carrier stalk reaching out to where the pad sits.
    stalk = (
        cq.Workplane("XY", origin=(0.0, 0.0, ARM_POST_HEIGHT - 0.020))
        .transformed(rotate=(0.0, 90.0, 0.0))
        .circle(0.010)
        .extrude(_ARM_STALK_LEN)
    )
    # Small adjustment lever/knob low on the post. The lever shaft starts on
    # the post axis and extrudes outward to the knob center, so post, lever,
    # and knob form one connected solid (XZ workplane normal is -Y, hence the
    # -sign to extrude toward +Y for the left arm and -Y for the right arm).
    lever = (
        cq.Workplane("XZ", origin=(0.0, 0.0, 0.040))
        .circle(0.006)
        .extrude(-sign * 0.043)
    )
    knob = cq.Workplane("XY", origin=(0.0, sign * 0.043, 0.040)).sphere(0.011)
    frame = post.union(stalk).union(lever).union(knob)
    return mesh_from_cadquery(frame, name, **TOL)


def _arm_pad_mesh(name: str):
    """Upholstered armrest cushion, long axis along X, resting on the stalk."""
    pad = (
        cq.Workplane("XY", origin=(_ARM_PAD_CX, 0.0, _ARM_PAD_CZ))
        .box(ARM_PAD_LEN, ARM_PAD_W, ARM_PAD_H)
        .edges("|Z")
        .fillet(0.020)
    )
    pad = pad.edges(">Z").fillet(0.014)
    return mesh_from_cadquery(pad, name, **TOL)


# ---------------------------------------------------------------------------
# Model assembly.
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="surgical_exam_stool")

    chrome = model.material("chrome_steel", color=(0.66, 0.68, 0.72, 1.0))
    upholstery = model.material("blue_upholstery", color=(0.30, 0.32, 0.50, 1.0))
    yellow = model.material("hub_yellow", color=(0.92, 0.82, 0.34, 1.0))
    black_rubber = model.material("black_rubber", color=(0.07, 0.07, 0.08, 1.0))

    # --- Root: caster base + casters + pedals + yellow hub + lower sleeve ---
    base = model.part("base")
    base.visual(_star_base_mesh("star_base"), material=chrome, name="star_base")
    base.visual(
        _hub_cone_mesh("hub_cone"),
        material=yellow,
        name="hub_cone",
    )

    # Five casters, one under each spoke tip: vertical stem up into the spoke,
    # twin wheels below with horizontal tangential axes, bottoms at z=0.
    for i in range(5):
        ang = 2.0 * math.pi * i / 5.0 + math.radians(90.0)
        cx = CASTER_R * math.cos(ang)
        cy = CASTER_R * math.sin(ang)
        base.visual(
            _caster_mesh(f"caster_{i}"),
            origin=Origin(xyz=(cx, cy, 0.0), rpy=(0.0, 0.0, ang)),
            material=black_rubber,
            name=f"caster_{i}",
        )

    # Two foot pedals flanking the front, stems rooted in the hub cylinder
    # (pedal local z=0 is the stem axis; spokes sit at 90/162/234/306/18 deg,
    # so +/-45 deg keeps the paddles clear of the spokes and casters).
    for j, pang in enumerate((math.radians(45.0), math.radians(-45.0))):
        base.visual(
            _pedal_mesh(f"pedal_{j}"),
            origin=Origin(xyz=(0.0, 0.0, PEDAL_Z), rpy=(0.0, 0.0, pang)),
            material=black_rubber,
            name=f"pedal_{j}",
        )

    # Lower telescoping sleeve rises from the hub; the bellows wraps it.
    base.visual(
        _column_sleeve_mesh("column_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, HUB_TOP_Z)),
        material=chrome,
        name="column_sleeve",
    )
    base.visual(
        _bellows_mesh("bellows_boot"),
        origin=Origin(xyz=(0.0, 0.0, BELLOWS_Z0)),
        material=black_rubber,
        name="bellows_boot",
    )

    # --- Seat (carries backrest); slides up/down on the sleeve ---
    seat = model.part("seat")
    seat.visual(_seat_mesh("seat_cushion"), material=upholstery, name="seat_cushion")
    seat.visual(
        _seat_mount_mesh("seat_mount"),
        material=chrome,
        name="seat_mount",
    )
    # Backrest carried on a chrome stalk off the rear seat rim.
    seat.visual(
        _backrest_arm_mesh("backrest_arm"),
        origin=Origin(xyz=(-SEAT_RADIUS + 0.020, 0.0, SEAT_THICK - 0.010)),
        material=chrome,
        name="backrest_arm",
    )
    seat.visual(
        _backrest_mesh("backrest_pad"),
        origin=Origin(xyz=(-SEAT_RADIUS - 0.010, 0.0, SEAT_THICK + 0.090)),
        material=upholstery,
        name="backrest_pad",
    )

    # Seat top sits at SEAT_TOP_Z at the collapsed pose. The seat part frame is
    # its cushion bottom, so place that at SEAT_TOP_Z - SEAT_THICK.
    seat_frame_z = SEAT_TOP_Z - SEAT_THICK
    model.articulation(
        "seat_lift",
        ArticulationType.PRISMATIC,
        parent=base,
        child=seat,
        # Joint frame at the seat's collapsed mount height, on the column axis.
        origin=Origin(xyz=(0.0, 0.0, seat_frame_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=600.0, velocity=0.10, lower=0.0, upper=LIFT_TRAVEL
        ),
    )

    # --- Left and right swing-out armrests ---
    # Posts mount on the seat rim, just below seat top. Pads sit above seat top.
    post_z = SEAT_THICK - 0.030
    # Post sits just inside the cushion rim so its base clamp collar embeds into
    # the cushion side wall (real seated mount, not a floating post).
    post_radius_xy = SEAT_RADIUS - 0.028

    left_arm = model.part("left_arm")
    left_arm.visual(
        _arm_frame_mesh("left_arm_frame", sign=1.0),
        material=chrome,
        name="left_arm_frame",
    )
    left_arm.visual(
        _arm_pad_mesh("left_arm_pad"),
        material=upholstery,
        name="left_arm_pad",
    )
    right_arm = model.part("right_arm")
    right_arm.visual(
        _arm_frame_mesh("right_arm_frame", sign=-1.0),
        material=chrome,
        name="right_arm_frame",
    )
    right_arm.visual(
        _arm_pad_mesh("right_arm_pad"),
        material=upholstery,
        name="right_arm_pad",
    )

    # Left post sits on the +Y side of the seat, right post on the -Y side.
    model.articulation(
        "left_arm_swing",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=left_arm,
        origin=Origin(xyz=(0.060, post_radius_xy, post_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=2.0, lower=-1.4, upper=0.2
        ),
    )
    model.articulation(
        "right_arm_swing",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=right_arm,
        origin=Origin(xyz=(0.060, -post_radius_xy, post_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=2.0, lower=-0.2, upper=1.4
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests: prompt-specific exact claims.
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    seat = object_model.get_part("seat")
    left_arm = object_model.get_part("left_arm")
    right_arm = object_model.get_part("right_arm")

    seat_lift = object_model.get_articulation("seat_lift")
    left_swing = object_model.get_articulation("left_arm_swing")
    right_swing = object_model.get_articulation("right_arm_swing")

    # --- Intentional nested/captured fits ---
    # The seat collar telescopes over the column sleeve: this is the real
    # prismatic capture fit, so the collar bore intentionally surrounds the
    # sleeve. The bellows boot also wraps that same sleeve.
    ctx.allow_overlap(
        seat,
        base,
        elem_a="seat_mount",
        elem_b="column_sleeve",
        reason="Seat collar telescopes over the gas-pillar sleeve (prismatic capture fit).",
    )
    # Each armrest post-base clamp collar embeds into the seat cushion side wall
    # so the arm is mounted to the seat, not floating.
    ctx.allow_overlap(
        left_arm,
        seat,
        elem_a="left_arm_frame",
        elem_b="seat_cushion",
        reason="Left armrest clamp collar seats into the cushion rim (mounted, not floating).",
    )
    ctx.allow_overlap(
        right_arm,
        seat,
        elem_a="right_arm_frame",
        elem_b="seat_cushion",
        reason="Right armrest clamp collar seats into the cushion rim (mounted, not floating).",
    )
    # Exact proofs that the declared fits really engage (no floating clearance).
    ctx.expect_overlap(
        seat,
        base,
        elem_a="seat_mount",
        elem_b="column_sleeve",
        axes="xyz",
        min_overlap=0.002,
        name="seat collar really grips the column sleeve (telescoping capture)",
    )
    ctx.expect_overlap(
        left_arm,
        seat,
        elem_a="left_arm_frame",
        elem_b="seat_cushion",
        axes="xyz",
        min_overlap=0.002,
        name="left arm clamp collar engages the cushion rim",
    )
    ctx.expect_overlap(
        right_arm,
        seat,
        elem_a="right_arm_frame",
        elem_b="seat_cushion",
        axes="xyz",
        min_overlap=0.002,
        name="right arm clamp collar engages the cushion rim",
    )

    # --- Joint types / axes are what we claim ---
    ctx.check(
        "seat_lift is prismatic about +Z",
        seat_lift.articulation_type == ArticulationType.PRISMATIC
        and tuple(seat_lift.axis) == (0.0, 0.0, 1.0),
        details=f"type={seat_lift.articulation_type}, axis={seat_lift.axis}",
    )
    ctx.check(
        "left arm swings (revolute) about +Z",
        left_swing.articulation_type == ArticulationType.REVOLUTE
        and tuple(left_swing.axis) == (0.0, 0.0, 1.0),
        details=f"type={left_swing.articulation_type}, axis={left_swing.axis}",
    )
    ctx.check(
        "right arm swings (revolute) about +Z",
        right_swing.articulation_type == ArticulationType.REVOLUTE
        and tuple(right_swing.axis) == (0.0, 0.0, 1.0),
        details=f"type={right_swing.articulation_type}, axis={right_swing.axis}",
    )

    # --- Hero parts present ---
    seat_cushion = seat.get_visual("seat_cushion")
    sc_aabb = ctx.part_element_world_aabb(seat, elem=seat_cushion)
    star = base.get_visual("star_base")
    star_aabb = ctx.part_element_world_aabb(base, elem=star)
    ctx.check(
        "seat cushion sits well above the caster base",
        sc_aabb is not None and star_aabb is not None
        and sc_aabb[0][2] > star_aabb[1][2] + 0.25,
        details=f"cushion_min_z={sc_aabb[0][2] if sc_aabb else None}, "
        f"base_top_z={star_aabb[1][2] if star_aabb else None}",
    )

    # Round seat: roughly equal X/Y extent and the expected diameter.
    if sc_aabb is not None:
        sx = sc_aabb[1][0] - sc_aabb[0][0]
        sy = sc_aabb[1][1] - sc_aabb[0][1]
        ctx.check(
            "seat reads as a wide round cushion",
            sx > 2.0 * SEAT_RADIUS - 0.02 and sy > 2.0 * SEAT_RADIUS - 0.02,
            details=f"seat extent x={sx:.3f}, y={sy:.3f}",
        )

    # Backrest pad rises above the seat top and sits behind it (-X).
    backrest = seat.get_visual("backrest_pad")
    br_aabb = ctx.part_element_world_aabb(seat, elem=backrest)
    ctx.check(
        "backrest rises above the seat cushion",
        br_aabb is not None and sc_aabb is not None
        and br_aabb[1][2] > sc_aabb[1][2] + 0.05,
        details=f"backrest_top={br_aabb[1][2] if br_aabb else None}",
    )
    ctx.check(
        "backrest sits behind the seat (-X)",
        br_aabb is not None and sc_aabb is not None
        and br_aabb[0][0] < sc_aabb[0][0] + 0.02,
        details=f"backrest_min_x={br_aabb[0][0] if br_aabb else None}",
    )

    # Armrest pads are above seat top on opposite Y sides.
    la_aabb = ctx.part_world_aabb(left_arm)
    ra_aabb = ctx.part_world_aabb(right_arm)
    ctx.check(
        "left armrest is on +Y side, right on -Y side",
        la_aabb is not None and ra_aabb is not None
        and la_aabb[1][1] > 0.0 and ra_aabb[0][1] < 0.0,
        details=f"left_max_y={la_aabb[1][1] if la_aabb else None}, "
        f"right_min_y={ra_aabb[0][1] if ra_aabb else None}",
    )
    if sc_aabb is not None and la_aabb is not None:
        ctx.check(
            "armrest pads rise above seat top",
            la_aabb[1][2] > sc_aabb[1][2],
            details=f"arm_top={la_aabb[1][2]:.3f}, seat_top={sc_aabb[1][2]:.3f}",
        )

    # --- Prismatic lift actually raises the seat ---
    rest_seat = ctx.part_world_position(seat)
    with ctx.pose({seat_lift: LIFT_TRAVEL}):
        lifted_seat = ctx.part_world_position(seat)
    ctx.check(
        "raising seat_lift moves the seat upward",
        rest_seat is not None and lifted_seat is not None
        and lifted_seat[2] > rest_seat[2] + LIFT_TRAVEL - 0.005,
        details=f"rest_z={rest_seat[2] if rest_seat else None}, "
        f"lifted_z={lifted_seat[2] if lifted_seat else None}",
    )

    # --- Swinging an armrest rotates its pad out in Y (footprint moves) ---
    la_pad_rest = ctx.part_world_aabb(left_arm)
    with ctx.pose({left_swing: -1.2}):
        la_pad_swung = ctx.part_world_aabb(left_arm)
    ctx.check(
        "swinging left arm moves its pad outward (footprint shifts)",
        la_pad_rest is not None and la_pad_swung is not None
        and abs(la_pad_swung[1][0] - la_pad_rest[1][0])
        + abs(la_pad_swung[1][1] - la_pad_rest[1][1])
        > 0.08,
        details=f"rest={la_pad_rest[1]}, swung={la_pad_swung[1]}",
    )

    ra_pad_rest = ctx.part_world_aabb(right_arm)
    with ctx.pose({right_swing: 1.2}):
        ra_pad_swung = ctx.part_world_aabb(right_arm)
    ctx.check(
        "swinging right arm moves its pad outward (footprint shifts)",
        ra_pad_rest is not None and ra_pad_swung is not None
        and abs(ra_pad_swung[1][0] - ra_pad_rest[1][0])
        + abs(ra_pad_swung[0][1] - ra_pad_rest[0][1])
        > 0.08,
        details=f"rest={ra_pad_rest[1]}, swung={ra_pad_swung[1]}",
    )

    return ctx.report()


object_model = build_object_model()
