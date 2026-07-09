from __future__ import annotations

import glob
import math
import os

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ----------------------------------------------------------------------------
# Zippo-style armor-case windproof flip lighter (beveled armor variant).
#
# World frame: Z up, X = case width (0.038 m), Y = case depth (0.013 m).
# Front (logo) face looks toward -Y; the lid hinge sits on the rear (+Y) edge.
# Closed height 0.057 m: black lower case (0 .. 0.040), brushed-steel insert
# deck crown (0.040 .. 0.0425) and a black hollow lid (0.0427 .. 0.057).
#
# This variant uses a heavier armor case with chamfered/beveled long edges
# instead of rounded fillets, creating a faceted thicker profile. All other
# functional layers (insert, chimney, flint wheel, lid, hinge) are identical
# to the parent model.
# ----------------------------------------------------------------------------

CASE_W = 0.038
CASE_D = 0.013
CASE_H = 0.0400
CASE_WALL = 0.0012
CASE_FLOOR = 0.0015
EDGE_R = 0.0025

# Armor-case variant: chamfered vertical edges with thicker walls.
ARMOR_BEVEL = 0.0020
ARMOR_WALL = 0.0016
ARMOR_FLOOR = 0.0018

DECK_TOP = 0.0425
DECK_T = 0.0025

LID_Z0 = 0.0427
LID_TOP = 0.0570
LID_H = LID_TOP - LID_Z0  # 0.0143
LID_WALL = 0.0011
LID_TOP_WALL = 0.0012

# Hinge pivot on the rear top edge of the case (five-knuckle hinge line).
PIVOT_Y = 0.0074
PIVOT_Z = 0.0415
KNUCKLE_R = 0.0013
KNUCKLE_L = 0.0020
CASE_KNUCKLE_X = (-0.0044, 0.0, 0.0044)
LID_KNUCKLE_X = (-0.0022, 0.0022)
LID_OPEN = math.radians(110.0)

# Insert details.
CHIMNEY_CENTER = (-0.006, 0.0, 0.04825)
CHIMNEY_SIZE = (0.014, 0.0095, 0.0125)
WHEEL_CENTER = (0.006, 0.0, 0.0475)
WHEEL_D = 0.0095
WHEEL_T = 0.0035

MESH_TOL = 5e-5
MESH_ANG_TOL = 0.15


def _case_shell_shape() -> cq.Workplane:
    """Armor-case variant: chamfered vertical edges, thicker walls, open top, closed floor.

    The bevel creates flat 45-degree facets along all four long edges,
    giving the heavier armor profile its characteristic faceted silhouette.
    """
    shell = cq.Workplane("XY").box(CASE_W, CASE_D, CASE_H).edges("|Z").chamfer(ARMOR_BEVEL)
    cavity = (
        cq.Workplane("XY")
        .box(CASE_W - 2 * ARMOR_WALL, CASE_D - 2 * ARMOR_WALL, CASE_H)
        .edges("|Z")
        .chamfer(ARMOR_BEVEL - 0.0012)
        .translate((0.0, 0.0, ARMOR_FLOOR))
    )
    return shell.cut(cavity).translate((0.0, 0.0, CASE_H / 2))


def _lid_shell_shape() -> cq.Workplane:
    """Hollow open-bottom lid box, authored in the lid (hinge) local frame."""
    shell = cq.Workplane("XY").box(CASE_W, CASE_D, LID_H).edges("|Z").fillet(EDGE_R)
    cavity = (
        cq.Workplane("XY")
        .box(CASE_W - 2 * LID_WALL, CASE_D - 2 * LID_WALL, LID_H)
        .edges("|Z")
        .fillet(EDGE_R - 0.001)
        .translate((0.0, 0.0, -LID_TOP_WALL))
    )
    shell = shell.cut(cavity)
    # Lid local frame sits on the hinge pivot.
    return shell.translate((0.0, -PIVOT_Y, (LID_Z0 + LID_TOP) / 2 - PIVOT_Z))


def _deck_shape() -> cq.Workplane:
    """Brushed-steel insert deck crown resting on the case rim."""
    deck = (
        cq.Workplane("XY")
        .box(0.0368, 0.0116, DECK_T)
        .edges("|Z")
        .fillet(0.0018)
        .edges(">Z")
        .chamfer(0.0010)
    )
    return deck.translate((0.0, 0.0, DECK_TOP - DECK_T / 2))


def _chimney_shape() -> cq.Workplane:
    """Perforated rectangular wind-screen chimney, hollow with an open top."""
    cw, cd, ch = CHIMNEY_SIZE
    wall = 0.0009
    bottom = 0.0012
    body = cq.Workplane("XY").box(cw, cd, ch)
    cavity = (
        cq.Workplane("XY")
        .box(cw - 2 * wall, cd - 2 * wall, ch)
        .translate((0.0, 0.0, bottom))
    )
    body = body.cut(cavity)
    hole_r = 0.0011
    rows = (-0.0025, 0.0025)
    # Round holes on the two wide faces (normal +-Y).
    for dx in (-0.0045, -0.0015, 0.0015, 0.0045):
        for dz in rows:
            punch = (
                cq.Workplane("XY")
                .cylinder(cd + 0.004, hole_r)
                .rotate((0, 0, 0), (1, 0, 0), 90)
                .translate((dx, 0.0, dz))
            )
            body = body.cut(punch)
    # Round holes on the two narrow faces (normal +-X).
    for dz in rows:
        punch = (
            cq.Workplane("XY")
            .cylinder(cw + 0.004, hole_r)
            .rotate((0, 0, 0), (0, 1, 0), 90)
            .translate((0.0, 0.0, dz))
        )
        body = body.cut(punch)
    return body.translate(CHIMNEY_CENTER)


def _logo_shape() -> cq.Workplane:
    """Subtle raised "zippo" logo on the lid front face (lid local frame).

    Falls back to a slim rounded plaque when no usable TTF font exists.
    """
    font_candidates: list[str | None] = []
    for pattern in (
        "/usr/share/fonts/**/*.ttf",
        os.path.expanduser("~/.fonts/**/*.ttf"),
        "/mnt/zsn/miniconda3/fonts/Ubuntu-B.ttf",
        "/mnt/zsn/miniconda3/fonts/Ubuntu-R.ttf",
    ):
        font_candidates.extend(sorted(glob.glob(pattern, recursive=True))[:2])
    font_candidates.append(None)  # default font lookup

    text = None
    for font_path in font_candidates:
        try:
            candidate = cq.Workplane("XZ").text(
                "zippo",
                0.0055,
                0.0004,
                combine=False,
                halign="center",
                valign="center",
                fontPath=font_path,
            )
            if candidate.solids().size() > 0:
                text = candidate
                break
        except Exception:
            continue
    if text is None:
        text = (
            cq.Workplane("XY")
            .box(0.014, 0.0004, 0.004)
            .edges("|Y")
            .fillet(0.0012)
            .translate((0.0, -0.0002, 0.0))
        )
    # Base plane embedded 0.2 mm inside the lid front wall, proud 0.2 mm.
    return text.translate((0.0, -PIVOT_Y - CASE_D / 2 + 0.0002, 0.0095))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="zippo_armor_case_flip_lighter")

    matte_black = Material(name="matte_black_paint", rgba=(0.08, 0.08, 0.085, 1.0))
    steel = Material(name="brushed_steel", rgba=(0.70, 0.71, 0.73, 1.0))
    hinge_steel = Material(name="hinge_steel", rgba=(0.60, 0.61, 0.63, 1.0))
    wheel_gray = Material(name="flint_wheel_gray", rgba=(0.24, 0.24, 0.25, 1.0))
    brass = Material(name="brass", rgba=(0.72, 0.57, 0.22, 1.0))
    logo_gray = Material(name="logo_gray", rgba=(0.52, 0.53, 0.55, 1.0))

    # ------------------------------------------------------------------ case
    case = model.part("case")
    case.visual(
        mesh_from_cadquery(
            _case_shell_shape(), "case_shell",
            tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL,
        ),
        material=matte_black,
        name="case_shell",
    )
    # Three case knuckles of the five-knuckle hinge plus their rear-face tabs.
    for i, kx in enumerate(CASE_KNUCKLE_X):
        case.visual(
            Cylinder(radius=KNUCKLE_R, length=KNUCKLE_L),
            origin=Origin(xyz=(kx, PIVOT_Y, PIVOT_Z), rpy=(0.0, math.pi / 2, 0.0)),
            material=hinge_steel,
            name=f"case_knuckle_{i}",
        )
        case.visual(
            Box((0.0018, 0.0008, 0.0061)),
            origin=Origin(xyz=(kx, 0.0065, 0.03905)),
            material=hinge_steel,
            name=f"case_hinge_tab_{i}",
        )
    case.visual(
        Cylinder(radius=0.0006, length=0.0118),
        origin=Origin(xyz=(0.0, PIVOT_Y, PIVOT_Z), rpy=(0.0, math.pi / 2, 0.0)),
        material=hinge_steel,
        name="hinge_pin",
    )

    # ---------------------------------------------------------------- insert
    insert = model.part("insert")
    insert.visual(
        Box((0.034, 0.009, 0.0365)),
        origin=Origin(xyz=(0.0, 0.0, 0.02225)),
        material=steel,
        name="insert_tube",
    )
    insert.visual(
        mesh_from_cadquery(
            _deck_shape(), "deck",
            tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL,
        ),
        material=steel,
        name="deck",
    )
    insert.visual(
        mesh_from_cadquery(
            _chimney_shape(), "chimney",
            tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL,
        ),
        material=steel,
        name="chimney",
    )
    # Flint-wheel support bracket arms (front = -Y, rear = +Y).
    insert.visual(
        Box((0.005, 0.0008, 0.0068)),
        origin=Origin(xyz=(WHEEL_CENTER[0], -0.0024, 0.0454)),
        material=steel,
        name="bracket_arm_front",
    )
    insert.visual(
        Box((0.005, 0.0008, 0.0068)),
        origin=Origin(xyz=(WHEEL_CENTER[0], 0.0024, 0.0454)),
        material=steel,
        name="bracket_arm_rear",
    )
    insert.visual(
        Cylinder(radius=0.0014, length=0.0014),
        origin=Origin(xyz=(0.0105, -0.0035, 0.0430)),
        material=brass,
        name="flint_screw",
    )

    # ------------------------------------------------------------------- lid
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(
            _lid_shell_shape(), "lid_shell",
            tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL,
        ),
        material=matte_black,
        name="lid_shell",
    )
    lid.visual(
        mesh_from_cadquery(
            _logo_shape(), "zippo_logo",
            tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL,
        ),
        material=logo_gray,
        name="zippo_logo",
    )
    # Two lid knuckles of the five-knuckle hinge plus their rear-face tabs.
    for i, kx in enumerate(LID_KNUCKLE_X):
        lid.visual(
            Cylinder(radius=KNUCKLE_R, length=KNUCKLE_L),
            origin=Origin(xyz=(kx, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
            material=hinge_steel,
            name=f"lid_knuckle_{i}",
        )
        lid.visual(
            Box((0.0018, 0.0008, 0.0038)),
            origin=Origin(xyz=(kx, -0.0010, 0.0011)),
            material=hinge_steel,
            name=f"lid_hinge_tab_{i}",
        )

    # ----------------------------------------------------------- flint wheel
    wheel = model.part("flint_wheel")
    wheel_geo = KnobGeometry(
        WHEEL_D,
        WHEEL_T,
        body_style="cylindrical",
        grip=KnobGrip(style="knurled", count=30, depth=0.0005),
    )
    wheel.visual(
        mesh_from_geometry(wheel_geo, "wheel_knurl"),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=wheel_gray,
        name="wheel_knurl",
    )
    wheel.visual(
        Cylinder(radius=0.0009, length=0.0066),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=hinge_steel,
        name="wheel_axle",
    )

    # ----------------------------------------------------------------- joints
    model.articulation(
        "case_to_insert",
        ArticulationType.FIXED,
        parent=case,
        child=insert,
    )
    # Lid flips backward about the rear top edge: positive q opens toward +Y.
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=case,
        child=lid,
        origin=Origin(xyz=(0.0, PIVOT_Y, PIVOT_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0, lower=0.0, upper=LID_OPEN),
    )
    # Flint wheel free-spins about a horizontal axis perpendicular to the
    # case's wide faces (the Y axis).
    model.articulation(
        "flint_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=insert,
        child=wheel,
        origin=Origin(xyz=WHEEL_CENTER),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    case = object_model.get_part("case")
    insert = object_model.get_part("insert")
    lid = object_model.get_part("lid")
    wheel = object_model.get_part("flint_wheel")
    hinge = object_model.get_articulation("lid_hinge")
    spin = object_model.get_articulation("flint_wheel_spin")

    # Intentional captured fits.
    ctx.allow_overlap(
        case,
        insert,
        elem_a="case_shell",
        elem_b="insert_tube",
        reason="The insert tube is intentionally captured inside the hollow case shell, representing the seated insert mechanism.",
    )
    ctx.expect_within(
        insert,
        case,
        axes="xy",
        inner_elem="insert_tube",
        outer_elem="case_shell",
        margin=0.0,
        name="insert tube stays centered inside the case shell",
    )
    for i in range(len(LID_KNUCKLE_X)):
        ctx.allow_overlap(
            lid,
            case,
            elem_a=f"lid_knuckle_{i}",
            elem_b="hinge_pin",
            reason="The case hinge pin is intentionally captured inside the lid knuckles.",
        )
    for arm in ("bracket_arm_front", "bracket_arm_rear"):
        ctx.allow_overlap(
            wheel,
            insert,
            elem_a="wheel_axle",
            elem_b=arm,
            reason="The flint-wheel axle pin is intentionally seated through the bracket arm.",
        )

    # ----- closed rest pose (q = 0) -----
    ctx.expect_gap(
        lid,
        insert,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="deck",
        min_gap=0.0,
        max_gap=0.001,
        name="closed lid rim seats just above the insert deck",
    )
    ctx.expect_within(
        insert,
        lid,
        axes="xy",
        inner_elem="chimney",
        outer_elem="lid_shell",
        margin=0.0,
        name="closed lid hollow encloses the chimney footprint",
    )
    ctx.expect_overlap(
        lid,
        case,
        axes="xy",
        min_overlap=0.012,
        name="lid matches the case footprint when closed",
    )
    ctx.expect_contact(
        insert,
        case,
        elem_a="deck",
        elem_b="case_shell",
        contact_tol=1e-4,
        name="insert deck rests on the case rim",
    )
    ctx.expect_contact(
        lid,
        case,
        elem_a="lid_knuckle_0",
        elem_b="hinge_pin",
        name="lid knuckles are threaded on the hinge pin",
    )
    ctx.expect_gap(
        wheel,
        insert,
        axis="z",
        positive_elem="wheel_knurl",
        negative_elem="deck",
        min_gap=0.0,
        max_gap=0.001,
        name="flint wheel clears the deck by a hairline gap",
    )
    ctx.expect_gap(
        insert,
        wheel,
        axis="y",
        positive_elem="bracket_arm_rear",
        negative_elem="wheel_knurl",
        min_gap=0.0,
        max_gap=0.001,
        name="flint wheel runs between the bracket arms",
    )

    chimney_aabb = ctx.part_element_world_aabb(insert, elem="chimney")
    ctx.check(
        "perforated chimney rises above the deck",
        chimney_aabb is not None
        and 0.052 <= chimney_aabb[1][2] <= 0.057
        and chimney_aabb[0][2] < DECK_TOP,
        details=f"chimney aabb={chimney_aabb}",
    )
    screw_aabb = ctx.part_element_world_aabb(insert, elem="flint_screw")
    ctx.check(
        "brass flint screw is seated proud of the deck",
        screw_aabb is not None
        and screw_aabb[0][2] < DECK_TOP < screw_aabb[1][2],
        details=f"flint screw aabb={screw_aabb}",
    )
    logo_aabb = ctx.part_element_world_aabb(lid, elem="zippo_logo")
    ctx.check(
        "gray logo sits proud on the lid front face",
        logo_aabb is not None
        and logo_aabb[0][1] <= -0.0066
        and logo_aabb[0][1] > -0.008
        and LID_Z0 < logo_aabb[0][2]
        and logo_aabb[1][2] < LID_TOP,
        details=f"logo aabb={logo_aabb}",
    )

    case_knuckles = sum(1 for v in case.visuals if "knuckle" in (v.name or ""))
    lid_knuckles = sum(1 for v in lid.visuals if "knuckle" in (v.name or ""))
    ctx.check(
        "hinge has five knuckles (3 on case, 2 on lid)",
        case_knuckles == 3 and lid_knuckles == 2,
        details=f"case={case_knuckles}, lid={lid_knuckles}",
    )

    # ----- joint conventions -----
    limits = hinge.motion_limits
    ctx.check(
        "lid hinge axis is horizontal along the rear top edge (X)",
        abs(abs(hinge.axis[0]) - 1.0) < 1e-9
        and abs(hinge.axis[1]) < 1e-9
        and abs(hinge.axis[2]) < 1e-9,
        details=f"axis={hinge.axis}",
    )
    ctx.check(
        "lid hinge range is 0 to about 110 degrees",
        limits is not None
        and limits.lower == 0.0
        and 1.85 <= float(limits.upper) <= 2.0,
        details=f"limits=({limits.lower}, {limits.upper})" if limits else "no limits",
    )
    spin_limits = spin.motion_limits
    ctx.check(
        "flint wheel is a continuous joint about Y",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and abs(abs(spin.axis[1]) - 1.0) < 1e-9
        and abs(spin.axis[0]) < 1e-9
        and abs(spin.axis[2]) < 1e-9
        and (spin_limits is None or spin_limits.lower is None),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )

    # ----- fully open pose (as in the reference image) -----
    with ctx.pose({hinge: LID_OPEN}):
        lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
        ctx.check(
            "open lid flips backward past the rear face",
            lid_aabb is not None and lid_aabb[1][1] > 0.012,
            details=f"open lid shell aabb={lid_aabb}",
        )
        ctx.expect_gap(
            lid,
            insert,
            axis="y",
            positive_elem="lid_shell",
            negative_elem="chimney",
            min_gap=0.001,
            name="open lid fully exposes the chimney",
        )
        ctx.expect_contact(
            lid,
            case,
            elem_a="lid_knuckle_1",
            elem_b="hinge_pin",
            name="hinge stays engaged at full open",
        )

    # ----- flint wheel spin pose -----
    rest_pos = ctx.part_world_position(wheel)
    with ctx.pose({spin: 1.2}):
        spun_pos = ctx.part_world_position(wheel)
        ctx.expect_gap(
            insert,
            wheel,
            axis="y",
            positive_elem="bracket_arm_rear",
            negative_elem="wheel_knurl",
            min_gap=0.0,
            max_gap=0.001,
            name="spinning wheel stays between the bracket arms",
        )
    ctx.check(
        "flint wheel spins in place on its axle",
        rest_pos is not None
        and spun_pos is not None
        and all(abs(a - b) < 1e-6 for a, b in zip(rest_pos, spun_pos)),
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    # ----- armor case geometry verification -----
    case_shell_aabb = ctx.part_element_world_aabb(case, elem="case_shell")
    ctx.check(
        "armor case shell has correct dimensions",
        case_shell_aabb is not None
        and abs((case_shell_aabb[1][0] - case_shell_aabb[0][0]) - CASE_W) < 0.001
        and abs((case_shell_aabb[1][1] - case_shell_aabb[0][1]) - CASE_D) < 0.001
        and abs((case_shell_aabb[1][2] - case_shell_aabb[0][2]) - CASE_H) < 0.001,
        details=f"case shell aabb={case_shell_aabb}",
    )
    ctx.check(
        "armor case uses thicker walls than parent",
        ARMOR_WALL > CASE_WALL and ARMOR_FLOOR > CASE_FLOOR,
        details=f"armor_wall={ARMOR_WALL}, parent_wall={CASE_WALL}",
    )

    return ctx.report()


object_model = build_object_model()
