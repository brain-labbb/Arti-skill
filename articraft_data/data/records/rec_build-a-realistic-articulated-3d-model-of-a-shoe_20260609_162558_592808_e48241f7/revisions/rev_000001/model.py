from __future__ import annotations

# adidas shoe box (Footwear / Shoe box / variant 3).
#
# The reference image shows a rigid two-piece cardboard shoe box:
#   - a deep base tray that holds the shoe (matte black, white spec label on
#     a front corner, three-bar adidas logo on the long front face),
#   - a separate telescoping lift-off lid that caps over the top of the base.
#     The lid carries the white adidas wordmark + three-bar logo on its top,
#     a small adidas badge on its long face, and finger-pull notches on the
#     two short ends used to lift it off.
#
# Real mechanism: the lid is NOT hinged. It is a lift-off telescoping lid whose
# skirt walls drop down OVER the outside of the base's upper rim. The faithful
# articulation is therefore a PRISMATIC vertical lift: positive travel raises
# the lid straight up off the base along +Z.
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Real-world dimensions (meters)
# ---------------------------------------------------------------------------
BOX_LEN = 0.345  # long axis (X)
BOX_WID = 0.205  # short axis (Y)

WALL = 0.0028  # cardboard wall thickness

BASE_H = 0.110  # base tray exterior height (floor + walls)
LID_SKIRT = 0.058  # lid skirt (sidewall) drop height
LID_TOP_T = WALL  # lid top panel thickness

# The lid telescopes OUTSIDE the base. A real cardboard shoe-box lid grips the
# tray snugly: the lid skirt inner wall presses against the base outer wall with
# a tiny interference (seated friction fit), so the seated lid is supported by
# the base, not floating. That interference is an intentional seated overlap.
LID_GRIP = 0.0006  # interference of lid skirt onto base outer wall (seated grip)
LID_OVERLAP = 0.030  # how far the lid skirt drops down over the base rim

# Outer footprint of the lid: skirt inner wall pressed onto the base outer wall.
# inner_lid = base_outer - 2*LID_GRIP  ->  lid_outer = base_outer + 2*WALL - 2*LID_GRIP
LID_LEN = BOX_LEN + 2.0 * WALL - 2.0 * LID_GRIP
LID_WID = BOX_WID + 2.0 * WALL - 2.0 * LID_GRIP

# Decal / label sizes
LABEL_W = 0.090
LABEL_H = 0.058
DECAL_T = 0.0016  # printed decal slab thickness
DECAL_SINK = 0.0008  # how far the decal back face sinks into the host shell

# Materials --------------------------------------------------------------------
MAT_BLACK = Material(name="box_black", rgba=(0.07, 0.07, 0.08, 1.0))
MAT_BLACK_LID = Material(name="lid_black", rgba=(0.10, 0.10, 0.11, 1.0))
MAT_WHITE = Material(name="print_white", rgba=(0.92, 0.92, 0.92, 1.0))
MAT_GREY = Material(name="print_grey", rgba=(0.55, 0.55, 0.56, 1.0))
MAT_LABEL = Material(name="spec_label", rgba=(0.95, 0.95, 0.95, 1.0))


# ---------------------------------------------------------------------------
# CadQuery geometry builders
# ---------------------------------------------------------------------------
def _base_tray() -> cq.Workplane:
    """Open-top hollow tray. Local origin at the tray geometric center; the
    floor sits at z=0 and the open rim at z=BASE_H."""
    outer = (
        cq.Workplane("XY")
        .box(BOX_LEN, BOX_WID, BASE_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
    )
    # Hollow the tray, leaving the floor and four walls. Cut a pocket from the
    # top face down to just above the floor.
    inner_len = BOX_LEN - 2.0 * WALL
    inner_wid = BOX_WID - 2.0 * WALL
    pocket = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .box(inner_len, inner_wid, BASE_H, centered=(True, True, False))
    )
    return outer.cut(pocket)


def _lid() -> cq.Workplane:
    """Inverted hollow lid (top panel + four skirt walls) that drops over the
    base. Local origin at the lid center; the underside skirt rim sits at z=0
    and the top panel spans z=[LID_SKIRT, LID_SKIRT+LID_TOP_T]."""
    skirt = (
        cq.Workplane("XY")
        .box(LID_LEN, LID_WID, LID_SKIRT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
    )
    # Hollow out the underside so the lid is a real cap (open bottom).
    inner_len = LID_LEN - 2.0 * WALL
    inner_wid = LID_WID - 2.0 * WALL
    cavity = cq.Workplane("XY").box(inner_len, inner_wid, LID_SKIRT, centered=(True, True, False))
    lid = skirt.cut(cavity)
    # Add the solid top panel covering the cavity.
    top = (
        cq.Workplane("XY")
        .workplane(offset=LID_SKIRT)
        .box(LID_LEN, LID_WID, LID_TOP_T, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
    )
    lid = lid.union(top)

    # Finger-pull notches: a semicircular cut on each short end of the skirt,
    # at the bottom rim, used to lift the lid off.
    notch_r = 0.013
    for sx in (-1.0, 1.0):
        notch = (
            cq.Workplane("YZ")
            .workplane(offset=sx * (LID_LEN / 2.0))
            .center(0.0, 0.006)  # center the cut just above the bottom rim
            .circle(notch_r)
            .extrude(-sx * (WALL + 0.002), both=True)
        )
        lid = lid.cut(notch)
    return lid


def _rect_plate(
    center: tuple[float, float],
    width: float,
    height: float,
    thickness: float = DECAL_T,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(width, height, thickness, centered=(True, True, False))
        .translate((center[0], center[1], 0.0))
    )


def _union_plates(plates: list[cq.Workplane]) -> cq.Workplane:
    result = plates[0]
    for plate in plates[1:]:
        result = result.union(plate)
    return result


def _three_bar_logo(scale: float) -> cq.Workplane:
    """The adidas three-bar 'mountain' logo as three rising slanted bars,
    authored as a flat plate in the XY plane (thin in Z)."""
    bar_w = 0.018 * scale
    gap = 0.010 * scale
    heights = [0.030 * scale, 0.046 * scale, 0.062 * scale]
    slant = 0.012 * scale  # parallelogram lean
    t = DECAL_T
    bars = None
    x0 = -(1.5 * bar_w + gap)
    for i, h in enumerate(heights):
        cx = x0 + i * (bar_w + gap)
        # Parallelogram bar: leans to the right as it rises.
        pts = [
            (cx - bar_w / 2.0, 0.0),
            (cx + bar_w / 2.0, 0.0),
            (cx + bar_w / 2.0 + slant, h),
            (cx - bar_w / 2.0 + slant, h),
        ]
        bar = cq.Workplane("XY").polyline(pts).close().extrude(t)
        bars = bar if bars is None else bars.union(bar)
    return bars


def _wordmark(scale: float) -> cq.Workplane:
    """Small block-built lowercase adidas wordmark."""
    h = 0.028 * scale
    asc_h = 0.041 * scale
    w = 0.020 * scale
    i_w = 0.008 * scale
    gap = 0.0075 * scale
    s = 0.0046 * scale
    total = 5.0 * w + i_w + 5.0 * gap
    x = -total / 2.0
    plates: list[cq.Workplane] = []

    def add_a(cx: float) -> None:
        plates.extend(
            [
                _rect_plate((cx, h / 2.0 - s / 2.0), w * 0.78, s),
                _rect_plate((cx, 0.0), w * 0.82, s),
                _rect_plate((cx, -h / 2.0 + s / 2.0), w * 0.76, s),
                _rect_plate((cx + w / 2.0 - s / 2.0, 0.0), s, h),
                _rect_plate((cx - w / 2.0 + s / 2.0, -h * 0.24), s, h * 0.52),
            ]
        )

    def add_d(cx: float) -> None:
        stem_x = cx + w / 2.0 - s / 2.0
        plates.extend(
            [
                _rect_plate((cx, h * 0.03), w * 0.84, s),
                _rect_plate((cx, -h / 2.0 + s / 2.0), w * 0.76, s),
                _rect_plate((stem_x, (asc_h - h) / 2.0), s, asc_h),
                _rect_plate((cx - w / 2.0 + s / 2.0, -h * 0.24), s, h * 0.52),
            ]
        )

    def add_i(cx: float) -> None:
        plates.extend(
            [
                _rect_plate((cx, -h * 0.17), s, h * 0.64),
                _rect_plate((cx, h * 0.43), i_w * 0.72, s),
            ]
        )

    def add_s(cx: float) -> None:
        plates.extend(
            [
                _rect_plate((cx, h / 2.0 - s / 2.0), w * 0.80, s),
                _rect_plate((cx, 0.0), w * 0.78, s),
                _rect_plate((cx, -h / 2.0 + s / 2.0), w * 0.80, s),
                _rect_plate((cx - w / 2.0 + s / 2.0, h * 0.22), s, h * 0.42),
                _rect_plate((cx + w / 2.0 - s / 2.0, -h * 0.22), s, h * 0.42),
            ]
        )

    centers = [
        x + w / 2.0,
        x + w + gap + w / 2.0,
        x + 2.0 * (w + gap) + i_w / 2.0,
        x + 2.0 * (w + gap) + i_w + gap + w / 2.0,
        x + 3.0 * (w + gap) + i_w + gap + w / 2.0,
        x + 4.0 * (w + gap) + i_w + gap + w / 2.0,
    ]
    add_a(centers[0])
    add_d(centers[1])
    add_i(centers[2])
    add_d(centers[3])
    add_a(centers[4])
    add_s(centers[5])
    return _union_plates(plates)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="adidas_shoe_box")

    # ---- Base tray (root) -------------------------------------------------
    base = model.part("base_tray")
    base.visual(
        mesh_from_cadquery(_base_tray(), "base_tray"),
        material=MAT_BLACK,
        name="base_shell",
    )

    # White spec/size label on the front-left corner of the base long face.
    label_y = -BOX_WID / 2.0 + DECAL_SINK
    base.visual(
        cq_mesh_label(),
        origin=Origin(
            xyz=(-BOX_LEN / 2.0 + LABEL_W / 2.0 + 0.012, label_y, BASE_H * 0.5),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=MAT_LABEL,
        name="spec_label",
    )

    # Three-bar adidas logo on the base front long face (right of label).
    # Decal local z=0 plane sinks DECAL_SINK into the front wall (y=-BOX_WID/2)
    # so the printed graphic reads as flush-on, sharing geometry with the shell.
    front_y_in = -BOX_WID / 2.0 + DECAL_SINK
    base.visual(
        mesh_from_cadquery(_three_bar_logo(1.2), "base_logo"),
        origin=Origin(
            xyz=(0.045, front_y_in, BASE_H * 0.30),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=MAT_GREY,
        name="base_front_logo",
    )
    # 'adidas' wordmark beneath the front logo.
    base.visual(
        mesh_from_cadquery(_wordmark(1.0), "base_wordmark"),
        origin=Origin(
            xyz=(0.045, front_y_in, BASE_H * 0.30 - 0.012),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=MAT_WHITE,
        name="base_front_wordmark",
    )

    # ---- Lid (lift-off telescoping cap) -----------------------------------
    lid = model.part("lid")
    # Lid part frame: place it so the lid sits seated over the base at q=0.
    # The lid skirt rim should drop LID_OVERLAP below the base rim. We model the
    # lid in its own frame (rim at z=0, top at z=LID_SKIRT) and position the
    # whole part via the articulation origin.
    lid.visual(
        mesh_from_cadquery(_lid(), "lid"),
        material=MAT_BLACK_LID,
        name="lid_shell",
    )

    # White three-bar logo on the lid top (sinks DECAL_SINK into the top panel).
    lid.visual(
        mesh_from_cadquery(_three_bar_logo(1.4), "lid_top_logo"),
        origin=Origin(
            xyz=(0.0, 0.0, LID_SKIRT + LID_TOP_T - DECAL_SINK),
            rpy=(0.0, 0.0, 0.0),
        ),
        material=MAT_WHITE,
        name="lid_top_logo",
    )
    lid.visual(
        mesh_from_cadquery(_wordmark(0.82), "lid_top_wordmark"),
        origin=Origin(
            xyz=(0.0, -0.024, LID_SKIRT + LID_TOP_T - DECAL_SINK),
            rpy=(0.0, 0.0, 0.0),
        ),
        material=MAT_WHITE,
        name="lid_top_wordmark",
    )
    # Small adidas badge on the lid long face (front skirt).
    lid.visual(
        mesh_from_cadquery(_three_bar_logo(0.6), "lid_badge"),
        origin=Origin(
            xyz=(-BOX_LEN * 0.28, -LID_WID / 2.0 + DECAL_SINK, LID_SKIRT * 0.42),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=MAT_WHITE,
        name="lid_front_badge",
    )
    lid.visual(
        mesh_from_cadquery(_wordmark(0.34), "lid_badge_wordmark"),
        origin=Origin(
            xyz=(-BOX_LEN * 0.28, -LID_WID / 2.0 + DECAL_SINK, LID_SKIRT * 0.42 - 0.012),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=MAT_WHITE,
        name="lid_front_badge_wordmark",
    )

    # ---- Articulation: lid lifts straight up off the base -----------------
    # At q=0 the lid rim drops LID_OVERLAP below the base rim (seated). Joint
    # origin is the lid-seated frame in the base part frame: shift up so the
    # lid top clears the base, with the skirt overlapping the base rim.
    seated_z = BASE_H - LID_OVERLAP
    model.articulation(
        "base_to_lid",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, seated_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.3, lower=0.0, upper=0.110),
    )

    return model


def cq_mesh_label() -> object:
    """Flat white spec-label sticker as a thin plate in XY (thin in Z)."""
    return mesh_from_cadquery(
        cq.Workplane("XY").box(LABEL_W, LABEL_H, DECAL_T, centered=(True, True, False)),
        "spec_label",
    )


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_tray")
    lid = object_model.get_part("lid")
    joint = object_model.get_articulation("base_to_lid")

    # The seated lid grips the tray: its skirt walls press onto the base outer
    # walls with a small interference (a real friction/telescoping fit), so the
    # closed pose has an intentional local overlap between the two shells.
    ctx.allow_overlap(
        lid,
        base,
        elem_a="lid_shell",
        elem_b="base_shell",
        reason="Seated lift-off lid skirt telescopes over and grips the tray's outer walls (intended nesting fit).",
    )

    # --- Joint type / axis contract --------------------------------------
    ctx.check(
        "lid joint is prismatic",
        joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={joint.articulation_type}",
    )
    ax = tuple(round(c, 6) for c in joint.axis)
    ctx.check(
        "lid lifts along +Z",
        ax == (0.0, 0.0, 1.0),
        details=f"axis={ax}",
    )
    lim = joint.motion_limits
    ctx.check(
        "lid travel is positive lift",
        lim is not None and lim.lower == 0.0 and lim.upper is not None and lim.upper > 0.05,
        details=f"limits=({lim.lower}, {lim.upper})" if lim else "no limits",
    )

    # --- Hero geometry present & placed ----------------------------------
    # Base tray is hollow (open top): its top face should be open, i.e. the
    # base part top is below the lid top in the seated pose. Check sizes.
    base_aabb = ctx.part_world_aabb(base)
    assert base_aabb is not None
    (bx0, by0, bz0), (bx1, by1, bz1) = base_aabb
    ctx.check(
        "base footprint matches a shoe box",
        abs((bx1 - bx0) - BOX_LEN) < 0.02 and abs((by1 - by0) - BOX_WID) < 0.02,
        details=f"len={bx1 - bx0:.3f}, wid={by1 - by0:.3f}",
    )

    # Lid sits over the base at rest: lid skirt extends outside the base rim
    # in the seated pose (telescoping fit), and the base fits within the lid
    # footprint on XY.
    with ctx.pose({joint: 0.0}):
        ctx.expect_within(
            base,
            lid,
            axes="xy",
            margin=0.001,
            name="base rim fits inside lid skirt",
        )
        # Lid top is above the base rim (lid caps the box).
        lid_aabb = ctx.part_world_aabb(lid)
        assert lid_aabb is not None
        (_, _, _), (_, _, lz1) = lid_aabb
        ctx.check(
            "lid caps over base top",
            lz1 > bz1,
            details=f"lid_top={lz1:.3f}, base_top={bz1:.3f}",
        )
        # Telescoping engagement: lid skirt overlaps the base rim along Z at
        # rest (retained insertion of the cap over the tray) and shares the
        # full footprint on XY (the skirt grips the base outer walls).
        ctx.expect_overlap(
            lid,
            base,
            axes="z",
            min_overlap=0.015,
            name="lid skirt engages base rim at rest",
        )
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            min_overlap=BOX_WID * 0.5,
            name="lid grips base footprint at rest",
        )
        # Seated lid is supported by the base (skirt contacts the tray walls).
        ctx.expect_contact(
            lid,
            base,
            contact_tol=1e-4,
            name="seated lid contacts base",
        )
        seated_lid_z = ctx.part_world_position(lid)

    # --- Actuating the joint lifts the lid off the base ------------------
    with ctx.pose({joint: 0.105}):
        lifted_lid_z = ctx.part_world_position(lid)
        # Once fully lifted the lid clears the base entirely along Z.
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            min_gap=0.0,
            name="lifted lid clears base",
        )

    assert seated_lid_z is not None and lifted_lid_z is not None
    ctx.check(
        "actuating joint raises the lid",
        lifted_lid_z[2] > seated_lid_z[2] + 0.08,
        details=f"seated_z={seated_lid_z[2]:.3f}, lifted_z={lifted_lid_z[2]:.3f}",
    )

    # --- Decals present on the right faces -------------------------------
    lid_logo_aabb = ctx.part_element_world_aabb(lid, elem="lid_top_logo")
    ctx.check(
        "adidas logo printed on lid top",
        lid_logo_aabb is not None,
        details="lid_top_logo element missing",
    )
    base_label_aabb = ctx.part_element_world_aabb(base, elem="spec_label")
    ctx.check(
        "spec label printed on base",
        base_label_aabb is not None,
        details="spec_label element missing",
    )

    return ctx.report()


object_model = build_object_model()
