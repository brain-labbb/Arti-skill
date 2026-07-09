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
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ----------------------------------------------------------------------------
# Classic Zippo-style windproof flip lighter — PIEZO PUSH-BUTTON VARIANT.
#
# World frame: Z up, X = case width (0.038 m), Y = case depth (0.013 m).
# Front (logo) face looks toward -Y; the lid hinge sits on the rear (+Y) edge.
# Closed height 0.057 m: black lower case (0 .. 0.040), brushed-steel insert
# deck crown (0.040 .. 0.0425) and a black hollow lid (0.0427 .. 0.057).
#
# Variant change: the knurled flint wheel + bracket arms are replaced by an
# electronic piezo push-button on the insert deck that presses down (PRISMATIC
# -Z) to spark.
# ----------------------------------------------------------------------------

CASE_W = 0.038
CASE_D = 0.013
CASE_H = 0.0400
CASE_WALL = 0.0012
CASE_FLOOR = 0.0015
EDGE_R = 0.0025

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

# Insert details — chimney unchanged from parent.
CHIMNEY_CENTER = (-0.006, 0.0, 0.04825)
CHIMNEY_SIZE = (0.014, 0.0095, 0.0125)

# Piezo push-button constants (replaces flint wheel).
BUTTON_X = 0.006
BUTTON_Y = 0.0
BUTTON_CAP_R = 0.0042
BUTTON_CAP_H = 0.0035
BUTTON_STEM_R = 0.0025
BUTTON_STEM_H = 0.0025
BUTTON_PRESS = 0.003  # 3 mm press travel

MESH_TOL = 5e-5
MESH_ANG_TOL = 0.15


def _case_shell_shape() -> cq.Workplane:
    """Hollow lower case: rounded vertical edges, open top, closed floor."""
    shell = cq.Workplane("XY").box(CASE_W, CASE_D, CASE_H).edges("|Z").fillet(EDGE_R)
    cavity = (
        cq.Workplane("XY")
        .box(CASE_W - 2 * CASE_WALL, CASE_D - 2 * CASE_WALL, CASE_H)
        .edges("|Z")
        .fillet(EDGE_R - 0.001)
        .translate((0.0, 0.0, CASE_FLOOR))
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
    # Round holes on the two wide faces (normal +-Y), emitted via loop.
    wide_dx = (-0.0045, -0.0015, 0.0015, 0.0045)
    for i in range(len(wide_dx)):
        for j in range(len(rows)):
            punch = (
                cq.Workplane("XY")
                .cylinder(cd + 0.004, hole_r)
                .rotate((0, 0, 0), (1, 0, 0), 90)
                .translate((wide_dx[i], 0.0, rows[j]))
            )
            body = body.cut(punch)
    # Round holes on the two narrow faces (normal +-X), emitted via loop.
    for i in range(len(rows)):
        punch = (
            cq.Workplane("XY")
            .cylinder(cw + 0.004, hole_r)
            .rotate((0, 0, 0), (0, 1, 0), 90)
            .translate((0.0, 0.0, rows[i]))
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


def _button_body_shape() -> cq.Workplane:
    """Piezo push-button body: narrow stem neck + wider chamfered cap.

    Authored in the button part local frame (origin at the deck surface).
    """
    stem = (
        cq.Workplane("XY")
        .cylinder(BUTTON_STEM_H, BUTTON_STEM_R)
        .translate((0.0, 0.0, BUTTON_STEM_H / 2))
    )
    cap = (
        cq.Workplane("XY")
        .cylinder(BUTTON_CAP_H, BUTTON_CAP_R)
    )
    cap = cap.edges(">Z").chamfer(0.0005)
    cap = cap.translate((0.0, 0.0, BUTTON_STEM_H + BUTTON_CAP_H / 2))
    return stem.union(cap)


def _button_collar_shape() -> cq.Workplane:
    """Mounting collar/bezel ring on the deck around the button stem."""
    outer_r = 0.0052
    inner_r = 0.0028
    h = 0.0008
    collar = cq.Workplane("XY").cylinder(h, outer_r)
    hole = cq.Workplane("XY").cylinder(h + 0.002, inner_r)
    collar = collar.cut(hole)
    collar = collar.edges(">Z").chamfer(0.0002)
    return collar.translate((BUTTON_X, BUTTON_Y, DECK_TOP + h / 2))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="zippo_windproof_flip_lighter_piezo")

    matte_black = Material(name="matte_black_paint", rgba=(0.08, 0.08, 0.085, 1.0))
    steel = Material(name="brushed_steel", rgba=(0.70, 0.71, 0.73, 1.0))
    hinge_steel = Material(name="hinge_steel", rgba=(0.60, 0.61, 0.63, 1.0))
    brass = Material(name="brass", rgba=(0.72, 0.57, 0.22, 1.0))
    logo_gray = Material(name="logo_gray", rgba=(0.52, 0.53, 0.55, 1.0))
    button_red = Material(name="piezo_button_cap", rgba=(0.45, 0.06, 0.05, 1.0))
    button_stem_mat = Material(name="piezo_button_stem", rgba=(0.22, 0.22, 0.23, 1.0))

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
    for i in range(len(CASE_KNUCKLE_X)):
        kx = CASE_KNUCKLE_X[i]
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
    # Button mounting collar/bezel on the deck surface.
    insert.visual(
        mesh_from_cadquery(
            _button_collar_shape(), "button_collar",
            tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL,
        ),
        material=steel,
        name="button_collar",
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
    for i in range(len(LID_KNUCKLE_X)):
        kx = LID_KNUCKLE_X[i]
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

    # --------------------------------------------------------- piezo button
    piezo_button = model.part("piezo_button")
    piezo_button.visual(
        mesh_from_cadquery(
            _button_body_shape(), "button_body",
            tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL,
        ),
        material=button_red,
        name="button_cap",
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
    # Piezo push-button: prismatic along -Z, pressing down into the deck.
    model.articulation(
        "piezo_push",
        ArticulationType.PRISMATIC,
        parent=insert,
        child=piezo_button,
        origin=Origin(xyz=(BUTTON_X, BUTTON_Y, DECK_TOP)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=0.5, lower=0.0, upper=BUTTON_PRESS,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    case = object_model.get_part("case")
    insert = object_model.get_part("insert")
    lid = object_model.get_part("lid")
    piezo_button = object_model.get_part("piezo_button")
    hinge = object_model.get_articulation("lid_hinge")
    push = object_model.get_articulation("piezo_push")

    # Intentional captured fits: lid knuckles on hinge pin.
    for i in range(len(LID_KNUCKLE_X)):
        ctx.allow_overlap(
            lid,
            case,
            elem_a=f"lid_knuckle_{i}",
            elem_b="hinge_pin",
            reason="The case hinge pin is intentionally captured inside the lid knuckles.",
        )

    # The button stem passes through the deck surface (seated insertion at press).
    ctx.allow_overlap(
        piezo_button,
        insert,
        elem_a="button_cap",
        elem_b="deck",
        reason=(
            "The piezo button stem is intentionally represented passing through "
            "the deck surface as seated insertion into the insert housing."
        ),
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

    # Button at rest sits above the deck.
    ctx.expect_gap(
        piezo_button,
        insert,
        axis="z",
        positive_elem="button_cap",
        negative_elem="deck",
        min_gap=0.0,
        max_gap=0.001,
        name="piezo button at rest contacts the deck surface",
    )
    # Button XY footprint is within the insert deck.
    ctx.expect_within(
        piezo_button,
        insert,
        axes="xy",
        inner_elem="button_cap",
        outer_elem="deck",
        margin=0.0,
        name="piezo button stays within the deck footprint",
    )
    # Button does not overlap the chimney in XY.
    chimney_aabb = ctx.part_element_world_aabb(insert, elem="chimney")
    button_aabb = ctx.part_element_world_aabb(piezo_button, elem="button_cap")
    ctx.check(
        "piezo button clears the chimney in XY",
        chimney_aabb is not None
        and button_aabb is not None
        and button_aabb[0][0] > chimney_aabb[1][0] - 0.0002,
        details=f"chimney_x_max={chimney_aabb[1][0] if chimney_aabb else None}, "
                f"button_x_min={button_aabb[0][0] if button_aabb else None}",
    )

    chimney_aabb = ctx.part_element_world_aabb(insert, elem="chimney")
    ctx.check(
        "perforated chimney rises above the deck",
        chimney_aabb is not None
        and 0.052 <= chimney_aabb[1][2] <= 0.057
        and chimney_aabb[0][2] < DECK_TOP,
        details=f"chimney aabb={chimney_aabb}",
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

    # Collar exists on the deck.
    collar_aabb = ctx.part_element_world_aabb(insert, elem="button_collar")
    ctx.check(
        "button mounting collar sits on the deck surface",
        collar_aabb is not None
        and collar_aabb[0][2] >= DECK_TOP - 0.0001
        and collar_aabb[1][2] <= DECK_TOP + 0.002,
        details=f"collar aabb={collar_aabb}",
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

    push_limits = push.motion_limits
    ctx.check(
        "piezo push is a PRISMATIC joint along -Z",
        push.articulation_type == ArticulationType.PRISMATIC
        and abs(push.axis[0]) < 1e-9
        and abs(push.axis[1]) < 1e-9
        and abs(push.axis[2] + 1.0) < 1e-9,
        details=f"type={push.articulation_type}, axis={push.axis}",
    )
    ctx.check(
        "piezo push travel is 0 to about 3 mm",
        push_limits is not None
        and push_limits.lower == 0.0
        and 0.002 <= push_limits.upper <= 0.005,
        details=f"limits=({push_limits.lower}, {push_limits.upper})" if push_limits else "no limits",
    )

    # No flint wheel parts exist in this variant.
    flint_wheel_found = any(
        p.name == "flint_wheel" for p in object_model.parts
    )
    ctx.check(
        "no flint_wheel part exists (piezo variant)",
        not flint_wheel_found,
        details="flint_wheel part should not exist in piezo variant",
    )

    # ----- fully open lid pose -----
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

    # ----- piezo button press pose -----
    rest_pos = ctx.part_world_position(piezo_button)
    with ctx.pose({push: BUTTON_PRESS}):
        pressed_pos = ctx.part_world_position(piezo_button)
        # Button moves down when pressed.
        ctx.check(
            "piezo button presses downward when actuated",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[2] < rest_pos[2] - 0.001,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )
        # Button XY stays in place (prismatic only in Z).
        ctx.check(
            "piezo button does not drift laterally when pressed",
            rest_pos is not None
            and pressed_pos is not None
            and abs(pressed_pos[0] - rest_pos[0]) < 1e-6
            and abs(pressed_pos[1] - rest_pos[1]) < 1e-6,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )
        # Button still within deck footprint when pressed.
        ctx.expect_within(
            piezo_button,
            insert,
            axes="xy",
            inner_elem="button_cap",
            outer_elem="deck",
            margin=0.0,
            name="pressed button stays within the deck footprint",
        )

    return ctx.report()


object_model = build_object_model()
