from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Racing-style gaming chair in matte-black PU leather with orange accents.
# Chair faces +X, Z up. About 0.7 m wide x 0.75 m deep x 1.3 m tall.
#
# Kinematic chain:
#   base (chrome 5-spoke star + gas-lift outer column, root)
#     -> caster_wheel_0..4   (continuous rolling joints)
#     -> gas_piston          (prismatic gas lift along Z, 0..0.1 m)
#          -> seat           (continuous swivel about the column Z axis)
#               -> backrest          (revolute recline, 0..-1.2 rad)
#               -> left/right armrest (prismatic height stems, 0..0.06 m)
# ---------------------------------------------------------------------------

LIFT_BASE_Z = 0.14          # world z of the gas-lift joint (piston frame origin)
PISTON_LEN = 0.20
SWIVEL_Z_IN_PISTON = 0.195  # swivel joint height inside the piston frame
SEAT_FRAME_Z = LIFT_BASE_Z + SWIVEL_Z_IN_PISTON  # 0.335 world at rest

HINGE_X = -0.24             # recline hinge in the seat frame
HINGE_Z = 0.17
BACK_TILT = math.radians(8.0)  # rest rake of the backrest, baked into visuals

STAR_R = 0.30               # radial distance of the caster axes
WHEEL_R = 0.030
ARM_Y = 0.31                # armrest stem centerline offset


def _rbox(dx: float, dy: float, dz: float, r: float) -> cq.Workplane:
    return cq.Workplane("XY").box(dx, dy, dz).edges().fillet(r)


def _star_spoke() -> cq.Workplane:
    """One chrome star-base spoke lofted along +X from a tall root near the hub
    to a thin raised tip that carries a caster stem underneath."""
    return (
        cq.Workplane("YZ")
        .workplane(offset=0.03)
        .center(0.0, 0.065)
        .rect(0.075, 0.080)
        .workplane(offset=0.28)
        .center(0.0, 0.026)
        .rect(0.050, 0.018)
        .loft()
    )


def _caster_fork() -> cq.Workplane:
    """Caster fork in its local frame: two side plates flanking the wheel, a
    top cap, and a stem that sockets up into the spoke tip."""
    plate_a = cq.Workplane("XY").box(0.05, 0.006, 0.054).translate((0.0, 0.017, 0.039))
    plate_b = cq.Workplane("XY").box(0.05, 0.006, 0.054).translate((0.0, -0.017, 0.039))
    cap = cq.Workplane("XY").box(0.055, 0.046, 0.012).translate((0.0, 0.0, 0.068))
    stem = cq.Workplane("XY").cylinder(0.024, 0.009).translate((0.0, 0.0, 0.080))
    return plate_a.union(plate_b).union(cap).union(stem)


def _seat_pan() -> cq.Workplane:
    return _rbox(0.52, 0.54, 0.08, 0.02).translate((0.04, 0.0, 0.09))


def _seat_bolster(side: float) -> cq.Workplane:
    return _rbox(0.46, 0.10, 0.12, 0.03).translate((0.02, 0.24 * side, 0.13))


def _back_panel() -> cq.Workplane:
    return _rbox(0.09, 0.50, 0.84, 0.025).translate((-0.055, 0.0, 0.38))


def _back_bolster(side: float) -> cq.Workplane:
    return (
        _rbox(0.06, 0.10, 0.52, 0.02)
        .rotate((0, 0, 0), (0, 0, 1), -25.0 * side)
        .translate((0.01, 0.26 * side, 0.32))
    )


def _shoulder_wing(side: float) -> cq.Workplane:
    return (
        _rbox(0.05, 0.16, 0.24, 0.015)
        .rotate((0, 0, 0), (0, 0, 1), -35.0 * side)
        .translate((0.0, 0.22 * side, 0.70))
    )


def _stripe_band() -> cq.Workplane:
    """Orange diagonal stripe band across the mid backrest: a rotated band
    intersected with a thin slab proud of the panel front face."""
    slab = cq.Workplane("XY").box(0.008, 0.44, 0.24).translate((-0.010, 0.0, 0.38))
    band = (
        cq.Workplane("XY")
        .box(0.03, 1.4, 0.11)
        .rotate((0, 0, 0), (1, 0, 0), 25.0)
        .translate((-0.010, 0.0, 0.38))
    )
    return slab.intersect(band)


def _headrest_pillow() -> cq.Workplane:
    return _rbox(0.10, 0.26, 0.11, 0.030).translate((0.03, 0.0, 0.655))


def _head_strap(side: float) -> cq.Workplane:
    vertical = (
        cq.Workplane("XY").box(0.012, 0.035, 0.165).translate((-0.010, 0.09 * side, 0.7225))
    )
    over_top = (
        cq.Workplane("XY").box(0.115, 0.035, 0.012).translate((-0.0595, 0.09 * side, 0.80))
    )
    return vertical.union(over_top)


def _lumbar_cushion() -> cq.Workplane:
    return _rbox(0.09, 0.30, 0.18, 0.030).translate((0.025, 0.0, 0.14))


def _armrest_pad() -> cq.Workplane:
    return _rbox(0.26, 0.09, 0.035, 0.012).translate((0.02, 0.0, 0.2125))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="racing_gaming_chair")

    pu_black = model.material("pu_black", rgba=(0.07, 0.07, 0.08, 1.0))
    orange = model.material("accent_orange", rgba=(0.93, 0.47, 0.07, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.79, 0.82, 1.0))
    charcoal = model.material("charcoal_plastic", rgba=(0.13, 0.13, 0.14, 1.0))
    rubber = model.material("rubber_black", rgba=(0.05, 0.05, 0.06, 1.0))

    caster_angles = [math.radians(72.0 * i) for i in range(5)]

    # ------------------------------------------------------------------ base
    base = model.part("base")
    spoke_mesh = mesh_from_cadquery(_star_spoke(), "base_spoke")
    fork_mesh = mesh_from_cadquery(_caster_fork(), "caster_fork")
    for i, th in enumerate(caster_angles):
        base.visual(
            spoke_mesh,
            origin=Origin(rpy=(0.0, 0.0, th)),
            material=chrome,
            name=f"base_spoke_{i}",
        )
        base.visual(
            fork_mesh,
            origin=Origin(
                xyz=(STAR_R * math.cos(th), STAR_R * math.sin(th), 0.0),
                rpy=(0.0, 0.0, th),
            ),
            material=charcoal,
            name=f"caster_fork_{i}",
        )
    base.visual(
        Cylinder(0.050, 0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=chrome,
        name="base_hub",
    )
    base.visual(
        Cylinder(0.040, 0.11),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=chrome,
        name="column_lower",
    )
    base.visual(
        Cylinder(0.034, 0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
        material=chrome,
        name="column_upper",
    )

    # ------------------------------------------------------------ caster wheels
    for i, th in enumerate(caster_angles):
        wheel = model.part(f"caster_wheel_{i}")
        wheel.visual(
            Cylinder(WHEEL_R, 0.018),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(0.006, 0.046),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=charcoal,
            name="axle",
        )
        model.articulation(
            f"caster_roll_{i}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=wheel,
            origin=Origin(
                xyz=(STAR_R * math.cos(th), STAR_R * math.sin(th), WHEEL_R),
                rpy=(0.0, 0.0, th),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=20.0, velocity=20.0),
        )

    # ------------------------------------------------------------- gas piston
    piston = model.part("gas_piston")
    piston.visual(
        Cylinder(0.024, PISTON_LEN),
        origin=Origin(xyz=(0.0, 0.0, PISTON_LEN / 2.0)),
        material=chrome,
        name="piston_rod",
    )
    model.articulation(
        "gas_lift",
        ArticulationType.PRISMATIC,
        parent=base,
        child=piston,
        origin=Origin(xyz=(0.0, 0.0, LIFT_BASE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=800.0, velocity=0.2, lower=0.0, upper=0.1),
    )

    # ------------------------------------------------------------------- seat
    seat = model.part("seat")
    seat.visual(
        Cylinder(0.032, 0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=charcoal,
        name="swivel_boss",
    )
    seat.visual(
        Box((0.22, 0.18, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=charcoal,
        name="tilt_mechanism",
    )
    seat.visual(mesh_from_cadquery(_seat_pan(), "seat_pan"), material=pu_black, name="seat_pan")
    for i, sy in enumerate((1.0, -1.0)):
        seat.visual(
            mesh_from_cadquery(_seat_bolster(sy), f"seat_bolster_{i}"),
            material=pu_black,
            name=f"seat_bolster_{i}",
        )
    seat.visual(
        Box((0.015, 0.40, 0.012)),
        origin=Origin(xyz=(0.2975, 0.0, 0.106)),
        material=orange,
        name="seat_front_piping",
    )
    for i, sy in enumerate((1.0, -1.0)):
        seat.visual(
            Box((0.08, 0.02, 0.15)),
            origin=Origin(xyz=(HINGE_X, 0.255 * sy, 0.135)),
            material=charcoal,
            name=f"hinge_plate_{i}",
        )
    for label, sy in (("left", 1.0), ("right", -1.0)):
        seat.visual(
            Box((0.10, 0.09, 0.045)),
            origin=Origin(xyz=(0.02, 0.285 * sy, 0.0725)),
            material=charcoal,
            name=f"{label}_armrest_bracket",
        )
        seat.visual(
            Box((0.06, 0.06, 0.16)),
            origin=Origin(xyz=(0.02, ARM_Y * sy, 0.12)),
            material=charcoal,
            name=f"{label}_armrest_sleeve",
        )
    model.articulation(
        "swivel",
        ArticulationType.CONTINUOUS,
        parent=piston,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z_IN_PISTON)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=2.0),
    )

    # --------------------------------------------------------------- backrest
    # Backrest frame origin sits on the recline hinge line; the 8 degree rest
    # rake is baked into the visuals via a pitch rotation.
    backrest = model.part("backrest")
    tilt = (0.0, -BACK_TILT, 0.0)
    backrest.visual(
        mesh_from_cadquery(_back_panel(), "back_panel"),
        origin=Origin(rpy=tilt),
        material=pu_black,
        name="backrest_shell",
    )
    for i, sy in enumerate((1.0, -1.0)):
        backrest.visual(
            mesh_from_cadquery(_back_bolster(sy), f"back_bolster_{i}"),
            origin=Origin(rpy=tilt),
            material=pu_black,
            name=f"back_bolster_{i}",
        )
        backrest.visual(
            mesh_from_cadquery(_shoulder_wing(sy), f"shoulder_wing_{i}"),
            origin=Origin(rpy=tilt),
            material=pu_black,
            name=f"shoulder_wing_{i}",
        )
        backrest.visual(
            mesh_from_cadquery(
                cq.Workplane("XY").box(0.010, 0.014, 0.66).translate((-0.008, 0.20 * sy, 0.38)),
                f"edge_piping_{i}",
            ),
            origin=Origin(rpy=tilt),
            material=orange,
            name=f"edge_piping_{i}",
        )
        backrest.visual(
            mesh_from_cadquery(_head_strap(sy), f"head_strap_{i}"),
            origin=Origin(rpy=tilt),
            material=pu_black,
            name=f"head_strap_{i}",
        )
    backrest.visual(
        mesh_from_cadquery(_stripe_band(), "stripe_band"),
        origin=Origin(rpy=tilt),
        material=orange,
        name="stripe_band",
    )
    backrest.visual(
        mesh_from_cadquery(
            cq.Workplane("XY").box(0.005, 0.16, 0.045).translate((-0.0085, 0.0, 0.545)),
            "backrest_logo",
        ),
        origin=Origin(rpy=tilt),
        material=orange,
        name="backrest_logo",
    )
    backrest.visual(
        mesh_from_cadquery(_headrest_pillow(), "headrest_pillow"),
        origin=Origin(rpy=tilt),
        material=pu_black,
        name="headrest_pillow",
    )
    backrest.visual(
        mesh_from_cadquery(
            cq.Workplane("XY").box(0.004, 0.10, 0.05).translate((0.079, 0.0, 0.655)),
            "pillow_emblem",
        ),
        origin=Origin(rpy=tilt),
        material=orange,
        name="pillow_emblem",
    )
    backrest.visual(
        mesh_from_cadquery(_lumbar_cushion(), "lumbar_cushion"),
        origin=Origin(rpy=tilt),
        material=pu_black,
        name="lumbar_cushion",
    )
    backrest.visual(
        mesh_from_cadquery(
            cq.Workplane("XY").box(0.004, 0.16, 0.05).translate((0.0705, 0.0, 0.13)),
            "lumbar_print",
        ),
        origin=Origin(rpy=tilt),
        material=orange,
        name="lumbar_print",
    )
    model.articulation(
        "recline",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=backrest,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        # Negative q about +Y leans the backrest top backward (-X): deep recline.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=200.0, velocity=1.0, lower=-1.2, upper=0.0),
    )

    # --------------------------------------------------------------- armrests
    pad_mesh = mesh_from_cadquery(_armrest_pad(), "armrest_pad")
    for label, sy, joint_name in (
        ("left", 1.0, "left_armrest_lift"),
        ("right", -1.0, "right_armrest_lift"),
    ):
        arm = model.part(f"{label}_armrest")
        arm.visual(
            Box((0.04, 0.04, 0.20)),
            origin=Origin(xyz=(0.0, 0.0, 0.10)),
            material=charcoal,
            name="arm_stem",
        )
        arm.visual(pad_mesh, material=pu_black, name="arm_pad")
        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=seat,
            child=arm,
            origin=Origin(xyz=(0.02, ARM_Y * sy, 0.10)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=100.0, velocity=0.1, lower=0.0, upper=0.06),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    piston = object_model.get_part("gas_piston")
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    left_arm = object_model.get_part("left_armrest")
    right_arm = object_model.get_part("right_armrest")
    wheels = [object_model.get_part(f"caster_wheel_{i}") for i in range(5)]
    lift = object_model.get_articulation("gas_lift")
    swivel = object_model.get_articulation("swivel")
    recline = object_model.get_articulation("recline")
    left_lift = object_model.get_articulation("left_armrest_lift")
    roll_0 = object_model.get_articulation("caster_roll_0")

    # ---------------------------------------------------- intentional overlaps
    ctx.allow_overlap(
        piston,
        base,
        elem_a="piston_rod",
        elem_b="column_upper",
        reason="The gas-lift piston rod telescopes inside the outer chrome column.",
    )
    ctx.allow_overlap(
        piston,
        base,
        elem_a="piston_rod",
        elem_b="column_lower",
        reason="The gas-lift piston rod telescopes inside the outer chrome column.",
    )
    ctx.allow_overlap(
        seat,
        piston,
        elem_a="swivel_boss",
        elem_b="piston_rod",
        reason="The piston tip is intentionally captured inside the seat swivel boss.",
    )
    for plate in ("hinge_plate_0", "hinge_plate_1"):
        ctx.allow_overlap(
            backrest,
            seat,
            elem_a="backrest_shell",
            elem_b=plate,
            reason="The side hinge plates intentionally capture the backrest shell at the recline pivot.",
        )
    for arm, sleeve in ((left_arm, "left_armrest_sleeve"), (right_arm, "right_armrest_sleeve")):
        ctx.allow_overlap(
            arm,
            seat,
            elem_a="arm_stem",
            elem_b=sleeve,
            reason="The armrest stem telescopes inside its seat-side sleeve for height adjustment.",
        )
    for i, wheel in enumerate(wheels):
        ctx.allow_overlap(
            wheel,
            base,
            elem_a="axle",
            elem_b=f"caster_fork_{i}",
            reason="The caster axle pin is intentionally journaled through both fork plates.",
        )

    # ----------------------------------------------------------- star base
    spokes = sum(1 for v in base.visuals if v.name and v.name.startswith("base_spoke"))
    ctx.check("chrome star base has 5 spokes", spokes == 5, details=f"spokes={spokes}")
    forks = sum(1 for v in base.visuals if v.name and v.name.startswith("caster_fork"))
    ctx.check("each spoke tip carries a caster fork", forks == 5, details=f"forks={forks}")
    for i, wheel in enumerate(wheels):
        w_aabb = ctx.part_world_aabb(wheel)
        ctx.check(
            f"caster wheel {i} rests on the floor",
            w_aabb is not None and -0.003 <= w_aabb[0][2] <= 0.003,
            details=f"wheel_{i}_aabb={w_aabb}",
        )
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "star base clears the floor and spans about 0.6 m",
        base_aabb is not None
        and 0.004 <= base_aabb[0][2] <= 0.03
        and 0.55 <= base_aabb[1][0] - base_aabb[0][0] <= 0.72,
        details=f"base_aabb={base_aabb}",
    )

    # ------------------------------------------------ gas lift column stack-up
    ctx.expect_gap(
        seat,
        piston,
        axis="z",
        positive_elem="swivel_boss",
        negative_elem="piston_rod",
        max_gap=0.0,
        max_penetration=0.05,
        name="seat swivel boss seats over the gas piston tip",
    )
    ctx.expect_overlap(
        piston,
        base,
        axes="z",
        elem_a="piston_rod",
        elem_b="column_upper",
        min_overlap=0.02,
        name="piston rod stays engaged in the outer column at rest",
    )

    # ----------------------------------------------- recline hinge engagement
    for plate in ("hinge_plate_0", "hinge_plate_1"):
        ctx.expect_overlap(
            backrest,
            seat,
            axes="xz",
            elem_a="backrest_shell",
            elem_b=plate,
            min_overlap=0.004,
            name=f"{plate} captures the backrest shell",
        )

    # -------------------------------------------------------- overall stature
    rest_back = ctx.part_world_aabb(backrest)
    ctx.check(
        "backrest tops out around 1.3 m",
        rest_back is not None and 1.24 <= rest_back[1][2] <= 1.38,
        details=f"backrest_aabb={rest_back}",
    )
    lpad = ctx.part_element_world_aabb(left_arm, elem="arm_pad")
    rpad = ctx.part_element_world_aabb(right_arm, elem="arm_pad")
    ctx.check(
        "chair is about 0.7 m wide across the armrest pads",
        lpad is not None and rpad is not None and 0.60 <= lpad[1][1] - rpad[0][1] <= 0.80,
        details=f"left_pad={lpad}, right_pad={rpad}",
    )

    # ------------------------------------------------------- orange accents
    stripe = ctx.part_element_world_aabb(backrest, elem="stripe_band")
    ctx.check(
        "orange stripe band crosses the mid backrest diagonally",
        stripe is not None
        and 0.68 <= stripe[0][2]
        and stripe[1][2] <= 1.08
        and stripe[1][1] - stripe[0][1] >= 0.40
        and 0.18 <= stripe[1][2] - stripe[0][2] <= 0.32,
        details=f"stripe_aabb={stripe}",
    )
    logo = ctx.part_element_world_aabb(backrest, elem="backrest_logo")
    ctx.check(
        "orange logo print sits on the upper backrest face",
        logo is not None and 0.98 <= 0.5 * (logo[0][2] + logo[1][2]) <= 1.10,
        details=f"logo_aabb={logo}",
    )
    piping = ctx.part_element_world_aabb(seat, elem="seat_front_piping")
    ctx.check(
        "orange piping runs along the seat front edge",
        piping is not None and piping[1][0] >= 0.295 and 0.42 <= piping[0][2] <= 0.47,
        details=f"piping_aabb={piping}",
    )
    edge0 = ctx.part_element_world_aabb(backrest, elem="edge_piping_0")
    edge1 = ctx.part_element_world_aabb(backrest, elem="edge_piping_1")
    ctx.check(
        "orange edge piping flanks both sides of the backrest face",
        edge0 is not None
        and edge1 is not None
        and edge0[0][1] > 0.12
        and edge1[1][1] < -0.12,
        details=f"edge_piping_0={edge0}, edge_piping_1={edge1}",
    )

    # ------------------------------------------------- headrest and lumbar
    pillow = ctx.part_element_world_aabb(backrest, elem="headrest_pillow")
    ctx.check(
        "headrest pillow is strapped near the top of the backrest",
        pillow is not None and 1.05 <= pillow[0][2] and 1.15 <= pillow[1][2] <= 1.27,
        details=f"pillow_aabb={pillow}",
    )
    emblem = ctx.part_element_world_aabb(backrest, elem="pillow_emblem")
    ctx.check(
        "orange emblem sits on the headrest pillow",
        emblem is not None
        and pillow is not None
        and 1.10 <= emblem[0][2]
        and emblem[1][2] <= 1.22
        and abs(0.5 * (emblem[0][1] + emblem[1][1])) < 0.06,
        details=f"emblem_aabb={emblem}",
    )
    strap0 = ctx.part_element_world_aabb(backrest, elem="head_strap_0")
    strap1 = ctx.part_element_world_aabb(backrest, elem="head_strap_1")
    ctx.check(
        "two straps wrap over the backrest top to hold the pillow",
        strap0 is not None
        and strap1 is not None
        and strap0[1][2] > 1.25
        and strap1[1][2] > 1.25
        and strap0[0][1] > 0.0 > strap1[1][1],
        details=f"strap_0={strap0}, strap_1={strap1}",
    )
    lumbar = ctx.part_element_world_aabb(backrest, elem="lumbar_cushion")
    pan = ctx.part_element_world_aabb(seat, elem="seat_pan")
    ctx.check(
        "lumbar cushion rests low on the seat back, just above the seat pan",
        lumbar is not None
        and pan is not None
        and pan[1][2] <= lumbar[0][2] <= pan[1][2] + 0.12
        and lumbar[1][0] > -0.20,
        details=f"lumbar_aabb={lumbar}, pan_aabb={pan}",
    )
    lprint = ctx.part_element_world_aabb(backrest, elem="lumbar_print")
    ctx.check(
        "orange print sits on the lumbar cushion face",
        lprint is not None
        and lumbar is not None
        # Tolerance covers the AABB skew introduced by the baked backrest rake.
        and lprint[1][0] >= lumbar[1][0] - 0.012
        and lumbar[0][2] < 0.5 * (lprint[0][2] + lprint[1][2]) < lumbar[1][2]
        and abs(0.5 * (lprint[0][1] + lprint[1][1])) < 0.04,
        details=f"lumbar_print={lprint}, lumbar={lumbar}",
    )

    # --------------------------------------------------- T-shaped 4D armrests
    stem = ctx.part_element_world_aabb(left_arm, elem="arm_stem")
    ctx.check(
        "armrest pad forms a T over its vertical stem",
        lpad is not None
        and stem is not None
        and lpad[1][0] - lpad[0][0] >= 0.22
        and stem[1][0] - stem[0][0] <= 0.06
        and lpad[0][2] >= stem[1][2] - 0.02,
        details=f"pad={lpad}, stem={stem}",
    )
    ctx.check(
        "armrest pads sit at desk height (~0.67 m)",
        lpad is not None and rpad is not None and 0.62 <= lpad[1][2] <= 0.71 and 0.62 <= rpad[1][2] <= 0.71,
        details=f"left_pad={lpad}, right_pad={rpad}",
    )
    with ctx.pose({left_lift: 0.06}):
        raised_pad = ctx.part_element_world_aabb(left_arm, elem="arm_pad")
        raised_stem = ctx.part_element_world_aabb(left_arm, elem="arm_stem")
    sleeve = ctx.part_element_world_aabb(seat, elem="left_armrest_sleeve")
    ctx.check(
        "armrest height adjustment raises the pad by 0.06 m",
        lpad is not None
        and raised_pad is not None
        and abs((raised_pad[1][2] - lpad[1][2]) - 0.06) < 0.005,
        details=f"rest_top={lpad[1][2] if lpad else None}, raised_top={raised_pad[1][2] if raised_pad else None}",
    )
    ctx.check(
        "raised armrest stem stays engaged in its sleeve",
        raised_stem is not None and sleeve is not None and raised_stem[0][2] < sleeve[1][2] - 0.02,
        details=f"raised_stem={raised_stem}, sleeve={sleeve}",
    )

    # ------------------------------------------------------------ recline pose
    with ctx.pose({recline: -1.2}):
        leaned = ctx.part_world_aabb(backrest)
    ctx.check(
        "recline to -1.2 rad lays the backrest deep and rearward",
        rest_back is not None
        and leaned is not None
        and leaned[1][2] < rest_back[1][2] - 0.40
        and leaned[0][0] < rest_back[0][0] - 0.35
        and leaned[0][2] > 0.0,
        details=f"rest={rest_back}, leaned={leaned}",
    )

    # ----------------------------------------------------------- gas lift pose
    with ctx.pose({lift: 0.1}):
        lifted_pan = ctx.part_element_world_aabb(seat, elem="seat_pan")
        lifted_piston = ctx.part_world_aabb(piston)
        column = ctx.part_element_world_aabb(base, elem="column_upper")
    ctx.check(
        "gas lift raises the whole upper chair by 0.1 m",
        pan is not None
        and lifted_pan is not None
        and abs((lifted_pan[0][2] - pan[0][2]) - 0.1) < 0.005,
        details=f"rest_pan={pan}, lifted_pan={lifted_pan}",
    )
    ctx.check(
        "fully lifted piston still engages the outer column",
        lifted_piston is not None
        and column is not None
        and lifted_piston[0][2] <= column[1][2] + 0.001,
        details=f"lifted_piston={lifted_piston}, column={column}",
    )

    # -------------------------------------------------------------- swivel pose
    with ctx.pose({swivel: math.pi / 2.0}):
        back_pos = ctx.part_world_position(backrest)
        base_aabb_turned = ctx.part_world_aabb(base)
    ctx.check(
        "quarter-turn swivel carries the backrest around the column Z axis",
        back_pos is not None and abs(back_pos[0]) < 0.03 and abs(back_pos[1] + 0.24) < 0.03,
        details=f"backrest_pos_at_quarter_turn={back_pos}",
    )
    ctx.check(
        "star base stays put while the upper chair swivels",
        base_aabb is not None
        and base_aabb_turned is not None
        and abs(base_aabb[0][0] - base_aabb_turned[0][0]) < 1e-9
        and abs(base_aabb[1][1] - base_aabb_turned[1][1]) < 1e-9,
        details=f"before={base_aabb}, after={base_aabb_turned}",
    )

    # ------------------------------------------------------------ caster roll
    # The tire is rotationally symmetric, so rolling must keep the wheel hub
    # centered at axle height over the same floor point.
    rest_wheel = ctx.part_world_aabb(wheels[0])
    with ctx.pose({roll_0: 1.0}):
        rolled = ctx.part_world_aabb(wheels[0])
    ctx.check(
        "rolling caster wheel spins in place about its axle",
        rest_wheel is not None
        and rolled is not None
        and abs(0.5 * (rolled[0][2] + rolled[1][2]) - WHEEL_R) < 0.003
        and abs(0.5 * (rolled[0][0] + rolled[1][0]) - 0.5 * (rest_wheel[0][0] + rest_wheel[1][0])) < 0.003,
        details=f"rest_wheel_aabb={rest_wheel}, rolled_wheel_aabb={rolled}",
    )

    return ctx.report()


object_model = build_object_model()
