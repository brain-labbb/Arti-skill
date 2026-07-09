from __future__ import annotations

"""Vintage hurricane (kerosene) lantern, ~0.28 m tall, 0.14 m base diameter.

Articraft brief:
- Object: classic dark-green hurricane lantern. Stepped fount base, barrel
  glass globe with flame on a wick, collar rings, two curved side tubes,
  tall conical chimney funnel with slanted louver vent slots and a flared rim.
- Root/support: lantern_body (grounded at z=0) carries everything fixed.
- Articulations:
  * body_to_bail_handle: REVOLUTE, brass wire bail pivoting about the
    horizontal X axis through the two ear bosses near the tube tops,
    range -100..+100 deg, q=0 = bail upright over the top.
  * fount_to_wick_knob: CONTINUOUS, knurled wick knob spinning about its
    horizontal radially protruding +Y axis on the fount.
- Intentional overlaps: bail pivot pins seat inside the ear bosses; knob
  stem passes through the fount wall. Both scoped allowances.
"""

import math
from functools import reduce

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------- dimensions
FOOT_R = 0.070          # base radius -> 0.14 m diameter
BODY_TOP_Z = 0.278      # dome apex -> ~0.28 m tall
PIVOT_Z = 0.212         # bail pivot height
PIVOT_X = 0.058         # bail wire end / pin centre
BAIL_RISE = 0.085       # bail apex height above pivot
BAIL_LIMIT = math.radians(100.0)
KNOB_BASE_Y = 0.0565    # fount surface radius at knob height
KNOB_Z = 0.055


# ---------------------------------------------------------------- cq solids
def _fount_solid() -> cq.Workplane:
    """Stepped foot + bulged kerosene fount, revolved about Z."""
    return (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.070, 0.0)
        .lineTo(0.070, 0.010)
        .lineTo(0.060, 0.014)
        .lineTo(0.060, 0.024)
        .spline(
            [(0.0545, 0.034), (0.0565, 0.055), (0.049, 0.080), (0.036, 0.092)],
            includeCurrent=True,
        )
        .lineTo(0.036, 0.095)
        .lineTo(0.0, 0.095)
        .close()
        .revolve(360.0, (0, 0), (0, 1))
    )


def _lower_collar_solid() -> cq.Workplane:
    """Green collar ring / globe seat plate under the glass globe."""
    return (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.095)
        .lineTo(0.050, 0.095)
        .lineTo(0.050, 0.103)
        .lineTo(0.040, 0.108)
        .lineTo(0.0, 0.108)
        .close()
        .revolve(360.0, (0, 0), (0, 1))
    )


def _top_assembly_solid() -> cq.Workplane:
    """Upper collar and tall hollow conical chimney funnel with a flared rim."""
    return (
        cq.Workplane("XZ")
        # Annular, open-centre sheet-metal section.  The previous parent dome
        # is replaced by a narrow, taller conical funnel and rolled top lip.
        .moveTo(0.025, 0.205)        # inner lower collar
        .lineTo(0.047, 0.205)        # outer globe-top collar
        .lineTo(0.047, 0.210)
        .lineTo(0.039, 0.218)
        .lineTo(0.038, 0.222)        # lower edge of conical funnel
        .lineTo(0.031, 0.270)        # tall tapering chimney wall
        .lineTo(0.039, 0.274)        # rolled/flared top rim
        .lineTo(0.039, BODY_TOP_Z)
        .lineTo(0.023, BODY_TOP_Z)   # open chimney throat
        .lineTo(0.023, 0.274)
        .lineTo(0.027, 0.270)
        .lineTo(0.033, 0.222)        # inner conical wall
        .lineTo(0.030, 0.218)
        .lineTo(0.025, 0.210)
        .close()
        .revolve(360.0, (0, 0), (0, 1))
    )


def _side_tube_solid() -> cq.Workplane:
    """+X curved air tube swept from the fount shoulder to the upper collar."""
    path = (
        cq.Workplane("XZ")
        .moveTo(0.050, 0.055)
        .spline(
            [(0.061, 0.095), (0.0655, 0.150), (0.058, 0.198), (0.042, 0.218)],
            includeCurrent=True,
        )
    )
    tangent = cq.Vector(0.011, 0.0, 0.040).normalized()
    profile = cq.Workplane(
        cq.Plane(origin=(0.050, 0.0, 0.055), xDir=(0, 1, 0), normal=tangent)
    ).circle(0.0075)
    return profile.sweep(path, isFrenet=True)


def _ear_solid(side: float) -> cq.Workplane:
    """Bail pivot ear boss on one side tube (side = +1 or -1)."""
    return (
        cq.Workplane("YZ", origin=(side * 0.038, 0.0, PIVOT_Z))
        .circle(0.007)
        .extrude(side * 0.018)
    )


def _green_body_solid() -> cq.Workplane:
    tube_r = _side_tube_solid()
    tube_l = tube_r.mirror("YZ")
    pieces = [
        _fount_solid(),
        _lower_collar_solid(),
        _top_assembly_solid(),
        tube_r,
        tube_l,
        _ear_solid(+1.0),
        _ear_solid(-1.0),
    ]
    return reduce(lambda a, b: a.union(b), pieces)


def _globe_solid() -> cq.Workplane:
    """Hollow barrel-shaped glass globe shell (2 mm wall, open top/bottom rims)."""
    return (
        cq.Workplane("XZ")
        .moveTo(0.036, 0.108)
        .spline(
            [(0.0465, 0.130), (0.048, 0.1565), (0.0465, 0.182), (0.036, 0.205)],
            includeCurrent=True,
        )
        .lineTo(0.034, 0.205)
        .spline(
            [(0.0445, 0.182), (0.046, 0.1565), (0.0445, 0.130), (0.034, 0.108)],
            includeCurrent=True,
        )
        .close()
        .revolve(360.0, (0, 0), (0, 1))
    )


def _burner_wick_solid() -> cq.Workplane:
    burner = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.104)
        .lineTo(0.018, 0.104)
        .lineTo(0.016, 0.112)
        .lineTo(0.0075, 0.117)
        .lineTo(0.0, 0.117)
        .close()
        .revolve(360.0, (0, 0), (0, 1))
    )
    wick = cq.Workplane("XY", origin=(0.0, 0.0, 0.104)).circle(0.0055).extrude(0.022)
    return burner.union(wick)


def _flame_solid() -> cq.Workplane:
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, 0.124))
        .circle(0.0045)
        .workplane(offset=0.011)
        .circle(0.0085)
        .workplane(offset=0.015)
        .circle(0.004)
        .workplane(offset=0.008)
        .circle(0.0012)
        .loft(ruled=False)
    )


def _bail_solid() -> cq.Workplane:
    """Thin brass wire arc over the top + pivot pins, in the bail joint frame.

    Joint frame sits at the pivot (world (0,0,PIVOT_Z)); at q=0 the bail is
    upright with its apex BAIL_RISE above the pivot.
    """
    path = (
        cq.Workplane("XZ")
        .moveTo(-PIVOT_X, 0.0)
        .threePointArc((0.0, BAIL_RISE), (PIVOT_X, 0.0))
    )
    # circle through (+-0.058, 0) and (0, 0.085): centre (0, 0.0227), R 0.0623
    arc_cz = (BAIL_RISE**2 - PIVOT_X**2) / (2.0 * BAIL_RISE)
    tangent = cq.Vector(-(BAIL_RISE - arc_cz), 0.0, PIVOT_X).normalized()
    profile = cq.Workplane(
        cq.Plane(origin=(-PIVOT_X, 0.0, 0.0), xDir=(0, 1, 0), normal=tangent)
    ).circle(0.002)
    wire_arc = profile.sweep(path, isFrenet=True)
    pin_r = cq.Workplane("YZ", origin=(0.048, 0.0, 0.0)).circle(0.0025).extrude(0.016)
    pin_l = cq.Workplane("YZ", origin=(-0.048, 0.0, 0.0)).circle(0.0025).extrude(-0.016)
    return wire_arc.union(pin_r).union(pin_l)


def _knob_solid() -> cq.Workplane:
    """Knurled wick knob in its joint frame; spin axis = local +Y."""
    stem = cq.Workplane("ZX", origin=(0.0, -0.012, 0.0)).circle(0.0035).extrude(0.024)
    disk = cq.Workplane("ZX", origin=(0.0, 0.009, 0.0)).circle(0.012).extrude(0.007)
    knob = stem.union(disk)
    for i in range(12):
        ang = 2.0 * math.pi * i / 12.0
        rib = (
            cq.Workplane("XY")
            .box(0.0024, 0.007, 0.0024)
            .translate((0.012 * math.cos(ang), 0.0125, 0.012 * math.sin(ang)))
        )
        knob = knob.union(rib)
    return knob


# ---------------------------------------------------------------- model
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_hurricane_lantern")

    green = Material(name="green_enamel", rgba=(0.05, 0.28, 0.12, 1.0))
    brass = Material(name="brass_wire", rgba=(0.80, 0.64, 0.30, 1.0))
    glass = Material(name="clear_glass", rgba=(0.82, 0.88, 0.90, 0.28))
    steel = Material(name="burner_steel", rgba=(0.30, 0.30, 0.32, 1.0))
    flame_mat = Material(name="flame_orange", rgba=(1.0, 0.58, 0.12, 1.0))
    slot_dark = Material(name="vent_slot_dark", rgba=(0.05, 0.06, 0.05, 1.0))

    # ---- root: lantern body -------------------------------------------------
    body = model.part("lantern_body")
    body.visual(
        mesh_from_cadquery(_green_body_solid(), "green_body_shell"),
        material=green,
        name="green_body_shell",
    )
    body.visual(
        mesh_from_cadquery(_globe_solid(), "globe_glass"),
        material=glass,
        name="globe_glass",
    )
    body.visual(
        mesh_from_cadquery(_burner_wick_solid(), "burner_wick"),
        material=steel,
        name="burner_wick",
    )
    body.visual(
        mesh_from_cadquery(_flame_solid(), "flame"),
        material=flame_mat,
        name="flame",
    )
    # conical chimney louvers: 12 dark slanted slits plus raised green lips
    for i in range(12):
        ang = 2.0 * math.pi * i / 12.0
        z = 0.235 + 0.010 * (i % 3)
        # Match the taper of the conical funnel so each louver lands on the
        # sheet-metal surface instead of floating off the chimney.
        cone_r = 0.038 - (0.038 - 0.031) * ((z - 0.222) / (0.270 - 0.222))
        slot_r = cone_r + 0.0014
        body.visual(
            Box((0.0030, 0.0130, 0.0035)),
            origin=Origin(
                xyz=(slot_r * math.cos(ang), slot_r * math.sin(ang), z),
                rpy=(math.radians(25.0), 0.0, ang + math.pi / 2.0),
            ),
            material=slot_dark,
            name=f"louver_slot_{i}",
        )
        lip_r = cone_r + 0.0026
        body.visual(
            Box((0.0022, 0.0145, 0.0020)),
            origin=Origin(
                xyz=(lip_r * math.cos(ang), lip_r * math.sin(ang), z + 0.004),
                rpy=(math.radians(25.0), 0.0, ang + math.pi / 2.0),
            ),
            material=green,
            name=f"louver_lip_{i}",
        )

    # ---- bail handle --------------------------------------------------------
    bail = model.part("bail_handle")
    bail.visual(
        mesh_from_cadquery(_bail_solid(), "bail_wire", tolerance=0.0003),
        material=brass,
        name="bail_wire",
    )
    model.articulation(
        "body_to_bail_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bail,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=3.0, lower=-BAIL_LIMIT, upper=BAIL_LIMIT
        ),
    )

    # ---- wick adjustment knob ------------------------------------------------
    knob = model.part("wick_knob")
    knob.visual(
        mesh_from_cadquery(_knob_solid(), "knob_knurled_disk", tolerance=0.0003),
        material=brass,
        name="knob_knurled_disk",
    )
    # off-axis index nub on the knob face proves continuous rotation
    knob.visual(
        Cylinder(radius=0.0018, length=0.0028),
        origin=Origin(xyz=(0.006, 0.0172, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="knob_index_nub",
    )
    model.articulation(
        "fount_to_wick_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.0, KNOB_BASE_Y, KNOB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )

    return model


# ---------------------------------------------------------------- tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("lantern_body")
    bail = object_model.get_part("bail_handle")
    knob = object_model.get_part("wick_knob")
    bail_joint = object_model.get_articulation("body_to_bail_handle")
    knob_joint = object_model.get_articulation("fount_to_wick_knob")

    # intentional, local embeddings
    ctx.allow_overlap(
        bail,
        body,
        elem_a="bail_wire",
        elem_b="green_body_shell",
        reason="Bail pivot pins are intentionally seated through the ear bosses on the side tubes.",
    )
    ctx.allow_overlap(
        knob,
        body,
        elem_a="knob_knurled_disk",
        elem_b="green_body_shell",
        reason="Wick knob stem intentionally passes through the fount wall to the wick mechanism.",
    )

    # ---- overall form ----
    aabb = ctx.part_world_aabb(body)
    ctx.check(
        "lantern grounded at z=0",
        aabb is not None and abs(aabb[0][2]) <= 0.002,
        details=f"body aabb={aabb}",
    )
    ctx.check(
        "lantern body about 0.28 m tall",
        aabb is not None and 0.27 <= aabb[1][2] <= 0.29,
        details=f"body aabb={aabb}",
    )
    ctx.check(
        "base/footprint about 0.14 m across",
        aabb is not None and 0.138 <= (aabb[1][0] - aabb[0][0]) <= 0.152,
        details=f"body aabb={aabb}",
    )

    # ---- glass globe with flame inside ----
    glass_visual = body.get_visual("globe_glass")
    glass_rgba = glass_visual.material.rgba if glass_visual.material else None
    ctx.check(
        "globe glass is translucent",
        glass_rgba is not None and glass_rgba[3] < 0.5,
        details=f"globe rgba={glass_rgba}",
    )
    flame_bb = ctx.part_element_world_aabb(body, elem="flame")
    globe_bb = ctx.part_element_world_aabb(body, elem="globe_glass")
    ctx.check(
        "flame sits inside the glass globe",
        flame_bb is not None
        and globe_bb is not None
        and flame_bb[0][0] > globe_bb[0][0]
        and flame_bb[1][0] < globe_bb[1][0]
        and flame_bb[0][1] > globe_bb[0][1]
        and flame_bb[1][1] < globe_bb[1][1]
        and flame_bb[0][2] > globe_bb[0][2]
        and flame_bb[1][2] < globe_bb[1][2],
        details=f"flame={flame_bb}, globe={globe_bb}",
    )

    # ---- conical chimney funnel with slanted louvers ----
    n_slots = sum(1 for v in body.visuals if (v.name or "").startswith("louver_slot_"))
    n_lips = sum(1 for v in body.visuals if (v.name or "").startswith("louver_lip_"))
    ctx.check("12 slanted louver vent slots on the chimney funnel", n_slots == 12, details=f"slots={n_slots}")
    ctx.check("each louver has a raised green rain lip", n_lips == 12, details=f"lips={n_lips}")
    lower_slot_bb = ctx.part_element_world_aabb(body, elem="louver_slot_0")
    upper_slot_bb = ctx.part_element_world_aabb(body, elem="louver_slot_2")
    if lower_slot_bb is not None and upper_slot_bb is not None:
        lower_center = (
            (lower_slot_bb[0][0] + lower_slot_bb[1][0]) / 2.0,
            (lower_slot_bb[0][1] + lower_slot_bb[1][1]) / 2.0,
            (lower_slot_bb[0][2] + lower_slot_bb[1][2]) / 2.0,
        )
        upper_center = (
            (upper_slot_bb[0][0] + upper_slot_bb[1][0]) / 2.0,
            (upper_slot_bb[0][1] + upper_slot_bb[1][1]) / 2.0,
            (upper_slot_bb[0][2] + upper_slot_bb[1][2]) / 2.0,
        )
        lower_r = math.hypot(lower_center[0], lower_center[1])
        upper_r = math.hypot(upper_center[0], upper_center[1])
    else:
        lower_center = upper_center = None
        lower_r = upper_r = None
    ctx.check(
        "louver slots ride on a tapering conical chimney",
        lower_r is not None
        and upper_r is not None
        and lower_center is not None
        and upper_center is not None
        and upper_center[2] > lower_center[2] + 0.010
        and upper_r < lower_r - 0.001,
        details=f"lower={lower_center}, upper={upper_center}, radii=({lower_r}, {upper_r})",
    )
    slot_height = (lower_slot_bb[1][2] - lower_slot_bb[0][2]) if lower_slot_bb else None
    ctx.check(
        "louver slots are visibly slanted rather than vertical pinholes",
        slot_height is not None and slot_height > 0.006,
        details=f"slot_0 aabb={lower_slot_bb}",
    )

    # ---- bail handle joint ----
    limits = bail_joint.motion_limits
    ctx.check(
        "bail is revolute with about +/-100 deg range",
        bail_joint.articulation_type == ArticulationType.REVOLUTE
        and limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower + BAIL_LIMIT) < 1e-6
        and abs(limits.upper - BAIL_LIMIT) < 1e-6,
        details=f"type={bail_joint.articulation_type}, limits={limits}",
    )
    ctx.expect_contact(
        bail,
        body,
        contact_tol=1e-6,
        name="bail pins engage the pivot ears",
    )
    bail_bb0 = ctx.part_world_aabb(bail)
    ctx.check(
        "upright bail arcs above the chimney cap",
        bail_bb0 is not None and bail_bb0[1][2] > BODY_TOP_Z + 0.01,
        details=f"bail aabb={bail_bb0}",
    )
    with ctx.pose({bail_joint: 1.5}):
        bail_bb1 = ctx.part_world_aabb(bail)
        ctx.check(
            "positive bail swing drops the loop down one side",
            bail_bb1 is not None
            and bail_bb1[0][1] < -0.06
            and bail_bb1[1][2] < BODY_TOP_Z - 0.01,
            details=f"bail swung aabb={bail_bb1}",
        )

    # ---- wick knob joint ----
    ctx.check(
        "wick knob is a continuous horizontal spinner",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_joint.motion_limits is not None
        and knob_joint.motion_limits.lower is None
        and abs(knob_joint.axis[1]) > 0.99,
        details=f"type={knob_joint.articulation_type}, axis={knob_joint.axis}",
    )
    knob_bb = ctx.part_world_aabb(knob)
    ctx.check(
        "knob protrudes radially from the fount",
        knob_bb is not None and knob_bb[1][1] > KNOB_BASE_Y + 0.010,
        details=f"knob aabb={knob_bb}",
    )
    nub0 = ctx.part_element_world_aabb(knob, elem="knob_index_nub")
    with ctx.pose({knob_joint: math.pi}):
        nub1 = ctx.part_element_world_aabb(knob, elem="knob_index_nub")
    ctx.check(
        "off-axis index nub sweeps across the spin axis after half a turn",
        nub0 is not None
        and nub1 is not None
        and (nub0[0][0] + nub0[1][0]) / 2.0 > 0.004
        and (nub1[0][0] + nub1[1][0]) / 2.0 < -0.004,
        details=f"nub rest={nub0}, half-turn={nub1}",
    )

    return ctx.report()


object_model = build_object_model()
