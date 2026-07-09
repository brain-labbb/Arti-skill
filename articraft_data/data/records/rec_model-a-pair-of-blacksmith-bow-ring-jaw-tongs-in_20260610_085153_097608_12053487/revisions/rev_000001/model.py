from __future__ import annotations

"""Blacksmith bow (ring-jaw) tongs in weathered dark gray forged steel.

Two identical forged halves cross at a riveted boss. Above the boss the two
flat-bar half-rings form a large open oval ring whose small flat tips lap at
the very top; below the boss two long tapered reins splay apart. One revolute
joint at the rivet (axis perpendicular to the tool plane) swings one
handle-plus-half-ring arm relative to the other, opening the ring jaws like
scissors.

Coordinate convention: the tool lies in the world XZ plane (Z up along the
tool, X across it). Y is the thickness direction and the rivet/pivot axis.
The pivot is at the world origin. The rear arm (-Y lap layer) is the root and
carries the rivet; the front arm (+Y lap layer) is bored at the boss and
rotates about the rivet.
"""

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

# ---------------------------------------------------------------- dimensions
# Ring jaw (open oval): ~0.12 m tall x ~0.09 m wide outside.
RING_OUT_X = 0.045  # outer semi-width of the oval ring
RING_OUT_Z = 0.060  # outer semi-height of the oval ring
BAR_W = 0.013  # in-plane width of the flat half-ring bar
BAR_T = 0.008  # thickness (Y) of the flat half-ring bar
RING_CZ = 0.069  # ring center height above the pivot

TIP_LAP = 0.004  # how far each flat tip laps past the ring center plane
TIP_REACH = 0.006  # how far each flat tip extends onto its own half

# Boss / rivet: flattened circular hinge plates, ~0.04 m diameter,
# ~0.04 m total stack thickness including the rivet heads.
BOSS_R = 0.020
BOSS_T = 0.016  # each arm's boss plate thickness
# The rivet clamps the lap joint, so the two boss faces ride in true sliding
# contact on the y=0 lap plane (rotation about the rivet axis preserves Y, so
# this contact never becomes penetration at any joint angle).
LAYER_GAP = 0.0
Y0 = LAYER_GAP / 2.0  # inner face of each arm's lap layer

BORE_R = 0.0060  # rivet bore through the front (moving) boss
RIVET_SHANK_R = 0.0055
RIVET_HEAD_R = 0.0090
RIVET_HEAD_T = 0.0040
HEAD_GAP = 0.0  # the front rivet head bears directly on the front boss face

# Reins: long slender tapered handles, ~0.38 m below the boss.
REIN_TOP_W = 0.018  # near-boss cross-section (roughly square)
REIN_TOP_T = 0.016
REIN_TIP_R = 0.004  # rounded end, ~0.008 m across
REIN_TOP_Z = -0.008  # rein root starts inside the boss disc, clear of the rivet
REIN_DROP = 0.388  # vertical run of the rein
REIN_TIP_Z = REIN_TOP_Z - REIN_DROP
REIN_SPLAY_X = 0.047  # lateral offset of each rein end (narrow splay angle)

PIVOT_UPPER = 0.4  # joint range 0 .. 0.4 rad


def _half_space_x(direction: float) -> cq.Workplane:
    """Large box covering x >= 0 (direction=+1) or x <= 0 (direction=-1)."""
    return (
        cq.Workplane("XY")
        .box(0.4, 0.4, 1.2)
        .translate((direction * 0.2, 0.0, 0.0))
    )


def _ring_half(direction: float, y_inner: float, y_sign: float) -> cq.Workplane:
    """Flat-bar half of the oval ring on one side of the tool centerline.

    direction: +1 keeps the +X half, -1 keeps the -X half.
    The bar occupies the arm's own lap layer in Y.
    """
    y_top = y_inner + y_sign * BAR_T
    y_hi = max(y_inner, y_top)
    outer = cq.Workplane("XZ").ellipse(RING_OUT_X, RING_OUT_Z).extrude(BAR_T)
    inner = (
        cq.Workplane("XZ")
        .ellipse(RING_OUT_X - BAR_W, RING_OUT_Z - BAR_W)
        .extrude(BAR_T * 3.0)
        .translate((0.0, BAR_T, 0.0))
    )
    band = outer.cut(inner)
    # XZ-plane extrude goes along -Y from y=0; move band into the lap layer.
    band = band.translate((0.0, y_hi, RING_CZ))
    return band.intersect(_half_space_x(direction))


def _jaw_tip(direction: float, y_inner: float, y_sign: float) -> cq.Workplane:
    """Small flat tip at the very top of one half-ring, lapping past center."""
    x_lo = -TIP_LAP if direction > 0 else -TIP_REACH
    x_hi = TIP_REACH if direction > 0 else TIP_LAP
    z_hi = RING_CZ + RING_OUT_Z
    z_lo = z_hi - BAR_W
    y_c = y_inner + y_sign * BAR_T / 2.0
    return (
        cq.Workplane("XY")
        .transformed(offset=((x_lo + x_hi) / 2.0, y_c, (z_lo + z_hi) / 2.0))
        .box(x_hi - x_lo, BAR_T, z_hi - z_lo)
    )


def _boss_plate(y_inner: float, y_sign: float, bored: bool) -> cq.Workplane:
    """Flattened circular hinge plate centered on the pivot, in one lap layer."""
    y_hi = max(y_inner, y_inner + y_sign * BOSS_T)
    plate = (
        cq.Workplane("XZ")
        .circle(BOSS_R)
        .extrude(BOSS_T)
        .translate((0.0, y_hi, 0.0))
    )
    if bored:
        bore = (
            cq.Workplane("XZ")
            .circle(BORE_R)
            .extrude(BOSS_T * 3.0)
            .translate((0.0, y_hi + BOSS_T, 0.0))
        )
        plate = plate.cut(bore)
    return plate


def _rein(tip_x: float, y_inner: float, y_sign: float) -> cq.Workplane:
    """Long slender rein: lofts from a near-square root to a small round end."""
    y_c = y_inner + y_sign * BOSS_T / 2.0
    shaft = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, y_c, REIN_TOP_Z))
        .rect(REIN_TOP_W, REIN_TOP_T)
        .workplane(offset=-REIN_DROP)
        .center(tip_x, 0.0)
        .circle(REIN_TIP_R)
        .loft()
    )
    tip_ball = (
        cq.Workplane("XY")
        .transformed(offset=(tip_x, y_c, REIN_TIP_Z))
        .sphere(REIN_TIP_R)
    )
    return shaft.union(tip_ball)


def _rivet_shank() -> cq.Workplane:
    """Rivet shank: embedded in the rear boss, passes through the front bore."""
    y_lo = -BOSS_T - Y0 + 0.002  # buried inside the rear boss plate
    y_hi = Y0 + BOSS_T + HEAD_GAP + RIVET_HEAD_T / 2.0  # reaches into front head
    return (
        cq.Workplane("XZ")
        .circle(RIVET_SHANK_R)
        .extrude(y_hi - y_lo)
        .translate((0.0, y_hi, 0.0))
    )


def _rivet_head(y_lo: float) -> cq.Workplane:
    head = (
        cq.Workplane("XZ")
        .circle(RIVET_HEAD_R)
        .extrude(RIVET_HEAD_T)
        .translate((0.0, y_lo + RIVET_HEAD_T, 0.0))
    )
    return head.edges(">Y").fillet(0.0015)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="blacksmith_bow_ring_jaw_tongs")

    forged_steel = model.material("forged_steel", rgba=(0.30, 0.31, 0.33, 1.0))
    boss_steel = model.material("boss_steel", rgba=(0.26, 0.27, 0.29, 1.0))
    worn_steel = model.material("worn_steel", rgba=(0.44, 0.45, 0.47, 1.0))
    rivet_steel = model.material("rivet_steel", rgba=(0.20, 0.21, 0.23, 1.0))

    # ---------------------------------------------------------- rear arm
    # Root half: left half-ring (-X), boss plate in the -Y lap layer,
    # rein splaying to +X, and the rivet (shank + both heads).
    rear = model.part("rear_arm")
    rear.visual(
        mesh_from_cadquery(_ring_half(-1.0, -Y0, -1.0), "rear_ring_jaw"),
        name="ring_jaw",
        material=forged_steel,
    )
    rear.visual(
        mesh_from_cadquery(_jaw_tip(-1.0, -Y0, -1.0), "rear_jaw_tip"),
        name="jaw_tip",
        material=worn_steel,
    )
    rear.visual(
        mesh_from_cadquery(_boss_plate(-Y0, -1.0, bored=False), "rear_boss_plate"),
        name="boss_plate",
        material=boss_steel,
    )
    rear.visual(
        mesh_from_cadquery(_rein(REIN_SPLAY_X, -Y0, -1.0), "rear_rein"),
        name="rein",
        material=forged_steel,
    )
    rear.visual(
        mesh_from_cadquery(_rivet_shank(), "rivet_shank"),
        name="rivet_shank",
        material=rivet_steel,
    )
    rear.visual(
        mesh_from_cadquery(_rivet_head(Y0 + BOSS_T + HEAD_GAP), "rivet_head_front"),
        name="rivet_head_front",
        material=rivet_steel,
    )
    rear.visual(
        mesh_from_cadquery(
            # Built at +Y with the domed fillet outward, then mirrored to -Y
            # so it overlaps the rear boss face by 0.4 mm and domes outward.
            _rivet_head(Y0 + BOSS_T - 0.0004).mirror("XZ"),
            "rivet_head_rear",
        ),
        name="rivet_head_rear",
        material=rivet_steel,
    )

    # ---------------------------------------------------------- front arm
    # Moving half: right half-ring (+X), bored boss plate in the +Y lap
    # layer, rein splaying to -X. The arms cross at the boss so squeezing
    # the reins drives the ring jaws like scissors.
    front = model.part("front_arm")
    front.visual(
        mesh_from_cadquery(_ring_half(1.0, Y0, 1.0), "front_ring_jaw"),
        name="ring_jaw",
        material=forged_steel,
    )
    front.visual(
        mesh_from_cadquery(_jaw_tip(1.0, Y0, 1.0), "front_jaw_tip"),
        name="jaw_tip",
        material=worn_steel,
    )
    front.visual(
        mesh_from_cadquery(_boss_plate(Y0, 1.0, bored=True), "front_boss_plate"),
        name="boss_plate",
        material=boss_steel,
    )
    front.visual(
        mesh_from_cadquery(_rein(-REIN_SPLAY_X, Y0, 1.0), "front_rein"),
        name="rein",
        material=forged_steel,
    )

    # ---------------------------------------------------------- articulation
    # Single revolute joint at the rivet, axis perpendicular to the tool
    # plane (+Y). Positive q swings the front arm's half-ring tip toward +X
    # (opening the ring at the top) while its rein splays further to -X.
    model.articulation(
        "boss_rivet_pivot",
        ArticulationType.REVOLUTE,
        parent=rear,
        child=front,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=PIVOT_UPPER, effort=40.0, velocity=2.0
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_arm")
    rear = object_model.get_part("rear_arm")
    pivot = object_model.get_articulation("boss_rivet_pivot")

    # --- closed rest pose: the two flat tips lap to close the oval ring ---
    ctx.expect_overlap(
        front,
        rear,
        axes="xz",
        elem_a="jaw_tip",
        elem_b="jaw_tip",
        min_overlap=0.005,
        name="flat jaw tips lap at the very top of the ring",
    )
    ctx.expect_gap(
        front,
        rear,
        axis="y",
        positive_elem="jaw_tip",
        negative_elem="jaw_tip",
        min_gap=0.0,
        max_gap=0.002,
        name="lapped jaw tips sit in adjacent layers without interpenetrating",
    )
    ctx.expect_gap(
        front,
        rear,
        axis="y",
        positive_elem="boss_plate",
        negative_elem="boss_plate",
        min_gap=0.0,
        max_gap=0.002,
        name="boss plates stack face to face at the rivet lap joint",
    )

    # --- ring proportions: large open oval ~0.12 m tall x ~0.09 m wide ---
    f_ring = ctx.part_element_world_aabb(front, elem="ring_jaw")
    r_ring = ctx.part_element_world_aabb(rear, elem="ring_jaw")
    ring_ok = f_ring is not None and r_ring is not None
    if ring_ok:
        width = f_ring[1][0] - r_ring[0][0]
        height = max(f_ring[1][2], r_ring[1][2]) - min(f_ring[0][2], r_ring[0][2])
        ctx.check(
            "ring jaw forms a ~0.09 m wide, ~0.12 m tall oval above the boss",
            abs(width - 0.090) < 0.01
            and abs(height - 0.120) < 0.01
            and min(f_ring[0][2], r_ring[0][2]) > 0.0,
            details=f"width={width:.4f}, height={height:.4f}",
        )
    else:
        ctx.fail("ring jaw aabb available", "missing ring_jaw element aabb")

    # --- rivet: shank passes through the bored moving boss, heads cap it ---
    ctx.expect_within(
        rear,
        front,
        axes="xz",
        inner_elem="rivet_shank",
        outer_elem="boss_plate",
        margin=0.0,
        name="rivet shank stays centered inside the front boss bore",
    )
    ctx.expect_overlap(
        rear,
        front,
        axes="y",
        elem_a="rivet_shank",
        elem_b="boss_plate",
        min_overlap=0.014,
        name="rivet shank passes through the full front boss thickness",
    )
    ctx.expect_gap(
        rear,
        front,
        axis="y",
        positive_elem="rivet_head_front",
        negative_elem="boss_plate",
        min_gap=0.0,
        max_gap=0.001,
        name="front rivet head caps the bored boss face",
    )

    # --- overall stack thickness at the boss is roughly 0.04 m ---
    head_f = ctx.part_element_world_aabb(rear, elem="rivet_head_front")
    head_r = ctx.part_element_world_aabb(rear, elem="rivet_head_rear")
    if head_f is not None and head_r is not None:
        stack = head_f[1][1] - head_r[0][1]
        ctx.check(
            "boss stack with rivet heads is ~0.04 m thick",
            0.030 <= stack <= 0.050,
            details=f"stack={stack:.4f}",
        )
    else:
        ctx.fail("rivet head aabbs available", "missing rivet head element aabb")

    # --- reins: long slender splayed handles below the boss ---
    f_rein = ctx.part_element_world_aabb(front, elem="rein")
    r_rein = ctx.part_element_world_aabb(rear, elem="rein")
    rein_ok = f_rein is not None and r_rein is not None
    if rein_ok:
        drop = -min(f_rein[0][2], r_rein[0][2])
        splay = r_rein[1][0] - f_rein[0][0]
        ctx.check(
            "reins extend ~0.38 m below the boss and splay apart",
            0.35 <= drop <= 0.43 and 0.06 <= splay <= 0.16,
            details=f"drop={drop:.4f}, splay={splay:.4f}",
        )
        ctx.check(
            "front rein splays toward -X, rear rein toward +X",
            f_rein[0][0] < -0.03 and r_rein[1][0] > 0.03,
            details=f"front_min_x={f_rein[0][0]:.4f}, rear_max_x={r_rein[1][0]:.4f}",
        )
    else:
        ctx.fail("rein aabbs available", "missing rein element aabb")

    # --- open pose: positive q opens the ring tips like scissors ---
    with ctx.pose({pivot: PIVOT_UPPER}):
        ctx.expect_gap(
            front,
            rear,
            axis="x",
            positive_elem="jaw_tip",
            negative_elem="jaw_tip",
            min_gap=0.02,
            name="open pose separates the ring tips at the top",
        )
        f_rein_open = ctx.part_element_world_aabb(front, elem="rein")
        if rein_ok and f_rein_open is not None:
            ctx.check(
                "open pose swings the front rein further outward",
                f_rein_open[0][0] < f_rein[0][0] - 0.05,
                details=(
                    f"closed_min_x={f_rein[0][0]:.4f}, "
                    f"open_min_x={f_rein_open[0][0]:.4f}"
                ),
            )
        else:
            ctx.fail("open rein aabb available", "missing open-pose rein aabb")

    return ctx.report()


object_model = build_object_model()
