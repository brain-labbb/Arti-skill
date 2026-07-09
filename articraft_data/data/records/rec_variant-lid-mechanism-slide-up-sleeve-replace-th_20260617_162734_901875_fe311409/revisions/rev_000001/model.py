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
# Zippo-style windproof lighter — slide-up sleeve variant.
#
# World frame: Z up, X = case width (0.038 m), Y = case depth (0.013 m).
# Front (logo) face looks toward -Y.
# Closed height 0.057 m: black lower case (0 .. 0.040), brushed-steel insert
# deck crown (0.040 .. 0.0425), chimney rising to ~0.0545, and a black hollow
# sleeve cap (0.043 .. 0.057) that slides straight up (+Z) to expose the flame.
# ----------------------------------------------------------------------------

CASE_W = 0.038
CASE_D = 0.013
CASE_H = 0.0400
CASE_WALL = 0.0012
CASE_FLOOR = 0.0015
EDGE_R = 0.0025

DECK_TOP = 0.0425
DECK_T = 0.0025

# Sleeve (sliding cap) dimensions.
# The sleeve fits over the case exterior, so inner cavity > case outer.
SLEEVE_Z0 = CASE_H  # 0.0400 — sleeve bottom sits on case top rim
SLEEVE_TOP = 0.0570
SLEEVE_H = SLEEVE_TOP - SLEEVE_Z0  # 0.0170
SLEEVE_WALL = 0.0011
SLEEVE_TOP_WALL = 0.0012
SLEEVE_CLEARANCE = 0.0002  # sliding clearance per side
SLEEVE_OUTER_W = CASE_W + 2 * SLEEVE_WALL + 2 * SLEEVE_CLEARANCE
SLEEVE_OUTER_D = CASE_D + 2 * SLEEVE_WALL + 2 * SLEEVE_CLEARANCE
SLEEVE_INNER_W = CASE_W + 2 * SLEEVE_CLEARANCE
SLEEVE_INNER_D = CASE_D + 2 * SLEEVE_CLEARANCE
SLEEVE_TRAVEL = 0.018  # prismatic travel to fully expose chimney

# Case hinge hardware (kept identical to parent — part of the case stamping).
PIVOT_Y = 0.0074
PIVOT_Z = 0.0415
KNUCKLE_R = 0.0013
KNUCKLE_L = 0.0020
CASE_KNUCKLE_X = (-0.0044, 0.0, 0.0044)

# Insert details.
CHIMNEY_CENTER = (-0.006, 0.0, 0.04825)
CHIMNEY_SIZE = (0.014, 0.0095, 0.0125)
WHEEL_CENTER = (0.006, 0.0, 0.0475)
WHEEL_D = 0.0095
WHEEL_T = 0.0035

MESH_TOL = 5e-5
MESH_ANG_TOL = 0.15


# ---------------------------------------------------------------------------
# Shared geometry helpers
# ---------------------------------------------------------------------------

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


def _sleeve_shell_shape() -> cq.Workplane:
    """Hollow open-bottom sleeve cap, centered at local origin.

    The sleeve slides straight up (+Z) over the case body.  The outer
    shell is slightly larger than the case so the sleeve wraps around it.
    The cavity is taller than the shell so the bottom face is cut through,
    leaving an open-bottom cap with a solid top wall and thin side walls.
    """
    shell = (
        cq.Workplane("XY")
        .box(SLEEVE_OUTER_W, SLEEVE_OUTER_D, SLEEVE_H)
        .edges("|Z")
        .fillet(EDGE_R + SLEEVE_WALL)
    )
    # Cavity: open at bottom (extends below shell), closed at top.
    cavity = (
        cq.Workplane("XY")
        .box(SLEEVE_INNER_W, SLEEVE_INNER_D, SLEEVE_H)
        .edges("|Z")
        .fillet(EDGE_R)
        .translate((0.0, 0.0, -SLEEVE_TOP_WALL))
    )
    return shell.cut(cavity)


def _sleeve_thumb_tab_shape() -> cq.Workplane:
    """Small thumb-grip tab protruding from the sleeve bottom-front edge."""
    tab_w = 0.010
    tab_d = 0.003
    tab_t = 0.0012
    tab = (
        cq.Workplane("XY")
        .box(tab_w, tab_d, tab_t)
        .edges("|Z")
        .fillet(0.0004)
    )
    # Positioned at the bottom of the sleeve, front face, protruding outward (-Y).
    return tab.translate((0.0, -SLEEVE_OUTER_D / 2 - tab_d / 2 + 0.0008, -SLEEVE_H / 2 + tab_t / 2))


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
    # Round holes on the two wide faces (normal ±Y) via indexed loop.
    wide_x_offsets = (-0.0045, -0.0015, 0.0015, 0.0045)
    wide_z_offsets = (-0.0025, 0.0025)
    for i in range(len(wide_x_offsets)):
        for j in range(len(wide_z_offsets)):
            dx = wide_x_offsets[i]
            dz = wide_z_offsets[j]
            punch = (
                cq.Workplane("XY")
                .cylinder(cd + 0.004, hole_r)
                .rotate((0, 0, 0), (1, 0, 0), 90)
                .translate((dx, 0.0, dz))
            )
            body = body.cut(punch)
    # Round holes on the two narrow faces (normal ±X).
    narrow_z_offsets = (-0.0025, 0.0025)
    for i in range(len(narrow_z_offsets)):
        dz = narrow_z_offsets[i]
        punch = (
            cq.Workplane("XY")
            .cylinder(cw + 0.004, hole_r)
            .rotate((0, 0, 0), (0, 1, 0), 90)
            .translate((0.0, 0.0, dz))
        )
        body = body.cut(punch)
    return body.translate(CHIMNEY_CENTER)


def _logo_shape() -> cq.Workplane:
    """Subtle raised "zippo" logo on the sleeve front face (sleeve local frame).

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
    # Front face of sleeve is at local Y = -SLEEVE_OUTER_D/2; text protrudes slightly.
    return text.translate((0.0, -SLEEVE_OUTER_D / 2 + 0.0002, 0.001))


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="zippo_slide_sleeve_lighter")

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
    # Three case knuckles of the five-knuckle hinge pattern (case stamping).
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
    # Flint-wheel support bracket arms (front = -Y, rear = +Y).
    for i in range(2):
        arm_y = -0.0024 if i == 0 else 0.0024
        arm_name = "bracket_arm_front" if i == 0 else "bracket_arm_rear"
        insert.visual(
            Box((0.005, 0.0008, 0.0068)),
            origin=Origin(xyz=(WHEEL_CENTER[0], arm_y, 0.0454)),
            material=steel,
            name=arm_name,
        )
    insert.visual(
        Cylinder(radius=0.0014, length=0.0014),
        origin=Origin(xyz=(0.0105, -0.0035, 0.0430)),
        material=brass,
        name="flint_screw",
    )

    # --------------------------------------------------------------- sleeve
    sleeve = model.part("sleeve")
    sleeve.visual(
        mesh_from_cadquery(
            _sleeve_shell_shape(), "sleeve_shell",
            tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL,
        ),
        material=matte_black,
        name="sleeve_shell",
    )
    sleeve.visual(
        mesh_from_cadquery(
            _sleeve_thumb_tab_shape(), "sleeve_thumb_tab",
            tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL,
        ),
        material=matte_black,
        name="thumb_tab",
    )
    sleeve.visual(
        mesh_from_cadquery(
            _logo_shape(), "zippo_logo",
            tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL,
        ),
        material=logo_gray,
        name="zippo_logo",
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
    # Sleeve slides straight up (+Z) to expose the chimney and flame.
    # At q=0 the sleeve is closed (covering chimney); positive q lifts it.
    # The articulation origin is at the sleeve center so the part frame matches.
    model.articulation(
        "sleeve_slide",
        ArticulationType.PRISMATIC,
        parent=case,
        child=sleeve,
        origin=Origin(xyz=(0.0, 0.0, (SLEEVE_Z0 + SLEEVE_TOP) / 2.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=0.3, lower=0.0, upper=SLEEVE_TRAVEL,
        ),
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


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    case = object_model.get_part("case")
    insert = object_model.get_part("insert")
    sleeve = object_model.get_part("sleeve")
    wheel = object_model.get_part("flint_wheel")
    slide = object_model.get_articulation("sleeve_slide")
    spin = object_model.get_articulation("flint_wheel_spin")

    # Intentional captured fits (wheel axle through bracket arms).
    for i in range(2):
        arm = "bracket_arm_front" if i == 0 else "bracket_arm_rear"
        ctx.allow_overlap(
            wheel,
            insert,
            elem_a="wheel_axle",
            elem_b=arm,
            reason="The flint-wheel axle pin is intentionally seated through the bracket arm.",
        )

    # ----- closed rest pose (q = 0) -----
    # Sleeve covers the chimney at rest — chimney is fully enclosed in XY and Z.
    ctx.expect_within(
        insert,
        sleeve,
        axes="xy",
        inner_elem="chimney",
        outer_elem="sleeve_shell",
        margin=0.0,
        name="closed sleeve encloses the chimney footprint in XY",
    )
    ctx.expect_within(
        insert,
        sleeve,
        axes="z",
        inner_elem="chimney",
        outer_elem="sleeve_shell",
        margin=0.0,
        name="closed sleeve fully covers the chimney in Z",
    )
    ctx.expect_overlap(
        sleeve,
        case,
        axes="xy",
        min_overlap=0.012,
        name="sleeve matches the case footprint when closed",
    )
    # Sleeve bottom sits on the case top rim with a small sliding clearance.
    # The sleeve walls slide over the case exterior with ~0.2 mm clearance per side.
    ctx.expect_gap(
        sleeve,
        case,
        axis="z",
        positive_elem="sleeve_shell",
        negative_elem="case_shell",
        min_gap=-0.001,
        max_gap=0.001,
        name="closed sleeve bottom rests near the case top rim",
    )
    # Insert deck rests on case rim.
    ctx.expect_contact(
        insert,
        case,
        elem_a="deck",
        elem_b="case_shell",
        contact_tol=1e-4,
        name="insert deck rests on the case rim",
    )
    # Flint wheel clearance.
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

    # Chimney and flint screw visible geometry.
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

    # Logo on sleeve front face.
    logo_aabb = ctx.part_element_world_aabb(sleeve, elem="zippo_logo")
    ctx.check(
        "gray logo sits on the sleeve front face",
        logo_aabb is not None
        and logo_aabb[0][1] < -0.006
        and logo_aabb[1][1] < -0.006
        and SLEEVE_Z0 < logo_aabb[0][2]
        and logo_aabb[1][2] < SLEEVE_TOP,
        details=f"logo aabb={logo_aabb}",
    )

    # Thumb tab on sleeve bottom front.
    tab_aabb = ctx.part_element_world_aabb(sleeve, elem="thumb_tab")
    ctx.check(
        "thumb tab protrudes from the sleeve bottom front",
        tab_aabb is not None
        and tab_aabb[0][1] < -CASE_D / 2 - 0.0005
        and SLEEVE_Z0 - 0.001 < tab_aabb[0][2] < SLEEVE_Z0 + 0.005,
        details=f"thumb tab aabb={tab_aabb}",
    )

    # Case knuckles present (kept from parent case stamping).
    case_knuckles = sum(1 for v in case.visuals if "knuckle" in (v.name or ""))
    ctx.check(
        "case has three hinge knuckles from parent stamping",
        case_knuckles == 3,
        details=f"case knuckle count={case_knuckles}",
    )

    # ----- joint conventions -----
    ctx.check(
        "sleeve slide is a prismatic joint along +Z",
        slide.articulation_type == ArticulationType.PRISMATIC
        and abs(slide.axis[2] - 1.0) < 1e-9
        and abs(slide.axis[0]) < 1e-9
        and abs(slide.axis[1]) < 1e-9,
        details=f"type={slide.articulation_type}, axis={slide.axis}",
    )
    slide_limits = slide.motion_limits
    ctx.check(
        "sleeve slide range is 0 to about 0.020 m",
        slide_limits is not None
        and slide_limits.lower == 0.0
        and 0.015 <= float(slide_limits.upper) <= 0.025,
        details=f"limits=({slide_limits.lower}, {slide_limits.upper})" if slide_limits else "no limits",
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

    # ----- fully open pose (sleeve slid up to expose chimney) -----
    with ctx.pose({slide: SLEEVE_TRAVEL}):
        sleeve_aabb = ctx.part_element_world_aabb(sleeve, elem="sleeve_shell")
        chimney_top = chimney_aabb[1][2] if chimney_aabb else 0.0545
        ctx.check(
            "open sleeve bottom is above chimney top (chimney exposed)",
            sleeve_aabb is not None
            and sleeve_aabb[0][2] > chimney_top - 0.001,
            details=f"open sleeve aabb={sleeve_aabb}, chimney top={chimney_top}",
        )
        ctx.expect_overlap(
            sleeve,
            case,
            axes="xy",
            min_overlap=0.012,
            name="sleeve still aligned with case footprint when open",
        )

    # ----- sleeve actually moves upward -----
    rest_pos = ctx.part_world_position(sleeve)
    with ctx.pose({slide: SLEEVE_TRAVEL}):
        open_pos = ctx.part_world_position(sleeve)
    ctx.check(
        "sleeve moves upward along Z when opened",
        rest_pos is not None
        and open_pos is not None
        and open_pos[2] > rest_pos[2] + 0.015
        and abs(open_pos[0] - rest_pos[0]) < 1e-6
        and abs(open_pos[1] - rest_pos[1]) < 1e-6,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    # ----- flint wheel spin pose -----
    wheel_rest = ctx.part_world_position(wheel)
    with ctx.pose({spin: 1.2}):
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
    wheel_spun = ctx.part_world_position(wheel)
    ctx.check(
        "flint wheel spins in place on its axle",
        wheel_rest is not None
        and wheel_spun is not None
        and all(abs(a - b) < 1e-6 for a, b in zip(wheel_rest, wheel_spun)),
        details=f"rest={wheel_rest}, spun={wheel_spun}",
    )

    return ctx.report()


object_model = build_object_model()
