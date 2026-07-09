from __future__ import annotations

"""Compact countertop air fryer with a top-opening clamshell lid.

Layout convention (body frame):
- +X is the front of the appliance.
- +Y is the left side; the object is symmetric in Y.
- +Z is up; the body sits on the floor plane z = 0.

Overall envelope ~0.28 m wide (Y) x 0.32 m deep (X) x 0.33 m tall (Z).
The single articulation is a revolute lid hinge at the rear upper rim,
opening upward/backward to reveal the fixed cooking basket in the lower bowl.
"""

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

# ---------------------------------------------------------------- dimensions
BODY_H = 0.33
BODY_DEPTH_TOP = 0.32  # X extent at the top rim
BODY_WIDTH_TOP = 0.28  # Y extent at the top rim
BODY_DEPTH_BOT = 0.30  # slight inward taper toward the base
BODY_WIDTH_BOT = 0.26
CORNER_R = 0.040

# Split line between lower body (bowl) and upper lid
SPLIT_Z = 0.195
LID_HEIGHT = BODY_H - SPLIT_Z  # 0.135

# Interpolated cross-section at the split line
_T = SPLIT_Z / BODY_H
DEPTH_AT_SPLIT = BODY_DEPTH_BOT + _T * (BODY_DEPTH_TOP - BODY_DEPTH_BOT)
WIDTH_AT_SPLIT = BODY_WIDTH_BOT + _T * (BODY_WIDTH_TOP - BODY_WIDTH_BOT)

# Shell wall thickness
WALL_T = 0.008

# Hinge at the rear edge of the body at the split line
HINGE_X = -DEPTH_AT_SPLIT / 2.0
HINGE_Z = SPLIT_Z

# Cooking basket (body/world coordinates, centered in the bowl)
BASKET_LEN = 0.240
BASKET_W = 0.210
BASKET_H = 0.105
BASKET_CX = 0.0
BASKET_FLOOR_Z = WALL_T - 0.002  # embed 2 mm into the shell floor for connectivity
BASKET_CZ = BASKET_FLOOR_Z + BASKET_H / 2.0

# Hinge barrel dimensions
BARREL_R = 0.007
BARREL_LEN = 0.038
BARREL_Y_OFFSET = 0.058

# Half-depth at split, used as the lid geometry center offset in lid-local X
_HALF_DS = DEPTH_AT_SPLIT / 2.0


# ---------------------------------------------------------- geometry helpers


def _rounded_rect_sketch(wp: cq.Workplane, dx: float, dy: float, r: float) -> cq.Workplane:
    return wp.sketch().rect(dx, dy).vertices().fillet(r).finalize()


def _build_body_shell() -> cq.Workplane:
    """Lower body shell: tapered rounded loft from z=0 to SPLIT_Z, hollowed from top."""
    bottom = cq.Sketch().rect(BODY_DEPTH_BOT, BODY_WIDTH_BOT).vertices().fillet(CORNER_R)
    top = cq.Sketch().rect(DEPTH_AT_SPLIT, WIDTH_AT_SPLIT).vertices().fillet(CORNER_R)

    shell = (
        cq.Workplane("XY")
        .placeSketch(
            bottom,
            top.moved(cq.Location(cq.Vector(0.0, 0.0, SPLIT_Z))),
        )
        .loft()
    )

    # Cooking chamber: open at top, solid floor
    inner_r = max(CORNER_R - WALL_T, 0.005)
    cav_bot = (
        cq.Sketch()
        .rect(BODY_DEPTH_BOT - 2 * WALL_T, BODY_WIDTH_BOT - 2 * WALL_T)
        .vertices()
        .fillet(inner_r)
    )
    cav_top = (
        cq.Sketch()
        .rect(DEPTH_AT_SPLIT - 2 * WALL_T, WIDTH_AT_SPLIT - 2 * WALL_T)
        .vertices()
        .fillet(inner_r)
    )

    cavity = (
        cq.Workplane("XY")
        .placeSketch(
            cav_bot.moved(cq.Location(cq.Vector(0.0, 0.0, WALL_T))),
            cav_top.moved(cq.Location(cq.Vector(0.0, 0.0, SPLIT_Z + 0.002))),
        )
        .loft()
    )
    return shell.cut(cavity)


def _build_trim_band() -> cq.Workplane:
    """Copper trim ring wrapping the body at the split-line seam."""
    outer = _rounded_rect_sketch(
        cq.Workplane("XY", origin=(0.0, 0.0, SPLIT_Z - 0.007)),
        DEPTH_AT_SPLIT + 0.012,
        WIDTH_AT_SPLIT + 0.012,
        CORNER_R + 0.006,
    ).extrude(0.016)
    inner = _rounded_rect_sketch(
        cq.Workplane("XY", origin=(0.0, 0.0, SPLIT_Z - 0.014)),
        DEPTH_AT_SPLIT - 0.002,
        WIDTH_AT_SPLIT - 0.002,
        max(CORNER_R - 0.001, 0.005),
    ).extrude(0.034)
    return outer.cut(inner)


def _build_basket() -> cq.Workplane:
    """Open-top cooking basket fixed inside the lower bowl."""
    outer = (
        cq.Workplane("XY")
        .box(BASKET_LEN, BASKET_W, BASKET_H)
        .edges("|Z")
        .fillet(0.015)
    )
    cavity = (
        cq.Workplane("XY")
        .box(BASKET_LEN - 0.010, BASKET_W - 0.010, 0.120)
        .edges("|Z")
        .fillet(0.012)
        .translate((0.0, 0.0, 0.0125))
    )
    return outer.cut(cavity).translate((BASKET_CX, 0.0, BASKET_CZ))


# Fries: (dx, dy, yaw_deg, length) relative to basket center in body frame.
_FRIES = [
    # lower layer
    (-0.040, -0.040, 80.0, 0.090),
    (0.035, 0.035, 10.0, 0.100),
    (-0.030, -0.030, -15.0, 0.095),
    (0.045, 0.045, 60.0, 0.080),
    (-0.050, 0.050, -30.0, 0.070),
    (0.045, -0.045, 15.0, 0.090),
    (0.000, 0.000, 75.0, 0.080),
    # upper layer
    (-0.035, -0.035, 170.0, 0.085),
    (0.030, 0.030, 100.0, 0.090),
    (-0.025, -0.025, 70.0, 0.080),
    (-0.040, -0.040, 105.0, 0.080),
]


def _build_fries_heap() -> cq.Workplane:
    """Heap of golden fries inside the basket, in body/world coordinates."""
    bed_z = BASKET_FLOOR_Z + 0.009
    heap = (
        cq.Workplane("XY")
        .box(0.180, 0.160, 0.018)
        .translate((BASKET_CX, 0.0, bed_z))
    )
    lower_z = bed_z + 0.014
    upper_z = bed_z + 0.023
    for i, (fx, fy, yaw, length) in enumerate(_FRIES):
        fz = lower_z if i < 7 else upper_z
        fry = (
            cq.Workplane("XY")
            .box(length, 0.011, 0.011)
            .rotate((0, 0, 0), (0, 0, 1), yaw)
            .translate((BASKET_CX + fx, fy, fz))
        )
        heap = heap.union(fry)
    return heap


def _build_hinge_barrel() -> cq.Workplane:
    """Small cylindrical hinge barrel along Y, centered at origin."""
    cyl = (
        cq.Workplane("XY")
        .circle(BARREL_R)
        .extrude(BARREL_LEN)
        .translate((0.0, 0.0, -BARREL_LEN / 2.0))
    )
    # Rotate from Z axis to Y axis: 90° around X maps +Z to -Y
    return cyl.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)


def _build_lid_shell() -> cq.Workplane:
    """Upper lid dome in lid-local coords (origin at hinge, +X forward, +Z up).

    Bottom face at z_local = 0 (the split line), top at z_local = LID_HEIGHT.
    Both cross-sections are centered at x_local = _HALF_DS so the lid aligns
    with the body when the joint is at q = 0.
    """
    bottom = (
        cq.Sketch()
        .rect(DEPTH_AT_SPLIT, WIDTH_AT_SPLIT)
        .vertices()
        .fillet(CORNER_R)
    )
    top = (
        cq.Sketch()
        .rect(BODY_DEPTH_TOP, BODY_WIDTH_TOP)
        .vertices()
        .fillet(CORNER_R)
    )

    shell = (
        cq.Workplane("XY")
        .placeSketch(
            bottom.moved(cq.Location(cq.Vector(_HALF_DS, 0.0, 0.0))),
            top.moved(cq.Location(cq.Vector(_HALF_DS, 0.0, LID_HEIGHT))),
        )
        .loft()
    )

    # Hollow from the bottom: leaves top wall solid for the touch panel
    inner_r = max(CORNER_R - WALL_T, 0.005)
    cav_bot = (
        cq.Sketch()
        .rect(DEPTH_AT_SPLIT - 2 * WALL_T, WIDTH_AT_SPLIT - 2 * WALL_T)
        .vertices()
        .fillet(inner_r)
    )
    cav_top = (
        cq.Sketch()
        .rect(BODY_DEPTH_TOP - 2 * WALL_T, BODY_WIDTH_TOP - 2 * WALL_T)
        .vertices()
        .fillet(inner_r)
    )

    cavity = (
        cq.Workplane("XY")
        .placeSketch(
            cav_bot.moved(cq.Location(cq.Vector(_HALF_DS, 0.0, -0.002))),
            cav_top.moved(cq.Location(cq.Vector(_HALF_DS, 0.0, LID_HEIGHT - WALL_T))),
        )
        .loft()
    )
    return shell.cut(cavity)


def _build_lid_handle() -> cq.Workplane:
    """Horizontal copper bar handle on the front face of the lid (lid-local coords)."""
    front_x = DEPTH_AT_SPLIT - 0.006
    handle_z = LID_HEIGHT * 0.60
    boss_sep = 0.052

    boss = cq.Workplane("XY").box(0.018, 0.026, 0.018)
    b0 = boss.translate((front_x + 0.009, -boss_sep, handle_z))
    b1 = boss.translate((front_x + 0.009, boss_sep, handle_z))

    bar = (
        cq.Workplane("XY")
        .box(0.010, 2.0 * boss_sep + 0.026, 0.012)
        .edges("|Y")
        .fillet(0.004)
        .translate((front_x + 0.023, 0.0, handle_z))
    )
    return b0.union(b1).union(bar)


# ----------------------------------------------------------- object model


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_air_fryer")

    gloss_black = model.material("gloss_black", rgba=(0.06, 0.06, 0.065, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.03, 0.03, 0.035, 1.0))
    copper = model.material("copper", rgba=(0.76, 0.44, 0.30, 1.0))
    basket_metal = model.material("basket_metal", rgba=(0.16, 0.16, 0.16, 1.0))
    fries_gold = model.material("fries_gold", rgba=(0.88, 0.62, 0.24, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.22, 0.22, 0.22, 1.0))

    # -------------------------------------------------------------- body (root)
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "body_shell"),
        material=gloss_black,
        name="shell",
    )
    body.visual(
        mesh_from_cadquery(_build_trim_band(), "rim_trim_band"),
        material=copper,
        name="rim_trim_band",
    )
    body.visual(
        Box((0.005, 0.070, 0.016)),
        origin=Origin(xyz=(BODY_DEPTH_BOT / 2.0 + 0.003, 0.0, 0.155)),
        material=copper,
        name="brand_logo",
    )
    body.visual(
        mesh_from_cadquery(_build_basket(), "cooking_basket"),
        material=basket_metal,
        name="basket",
    )
    body.visual(
        mesh_from_cadquery(_build_fries_heap(), "fries_heap"),
        material=fries_gold,
        name="fries_heap",
    )

    # Hinge barrels at the rear upper edge (provide visible pivot hardware)
    barrel_mesh = mesh_from_cadquery(_build_hinge_barrel(), "hinge_barrel")
    for i, y_sign in enumerate((1, -1)):
        body.visual(
            barrel_mesh,
            origin=Origin(xyz=(HINGE_X + 0.004, y_sign * BARREL_Y_OFFSET, HINGE_Z)),
            material=hinge_metal,
            name=f"hinge_barrel_{i}",
        )

    # -------------------------------------------------------------- lid (child)
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_shell(), "lid_shell"),
        material=gloss_black,
        name="lid_shell",
    )
    # Glass touch-control panel on the lid top
    lid.visual(
        Box((0.220, 0.180, 0.006)),
        origin=Origin(xyz=(_HALF_DS, 0.0, LID_HEIGHT + 0.003)),
        material=smoked_glass,
        name="top_glass_panel",
    )
    # Copper handle on the lid front face
    lid.visual(
        mesh_from_cadquery(_build_lid_handle(), "lid_handle"),
        material=copper,
        name="lid_handle",
    )

    # --------------------------------------------------------- revolute hinge
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=0.0, upper=1.8,
        ),
    )

    return model


# ----------------------------------------------------------- tests


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("lid_hinge")

    # ---- joint plan: revolute rear hinge, lateral axis, upward opening
    ctx.check(
        "lid hinge is revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={hinge.articulation_type}",
    )
    ax = tuple(hinge.axis)
    ctx.check(
        "hinge axis is lateral (-y) for rear-hinge upward opening",
        abs(ax[0]) < 1e-9 and abs(ax[1] + 1.0) < 1e-9 and abs(ax[2]) < 1e-9,
        details=f"axis={ax}",
    )
    lim = hinge.motion_limits
    ctx.check(
        "hinge limits allow 0 to ~103 deg opening",
        lim is not None and abs(lim.lower) < 1e-9 and 1.5 < lim.upper < 2.2,
        details=f"limits={lim}",
    )

    # ---- true scale and grounding (closed pose)
    body_aabb = ctx.part_world_aabb(body)
    ctx.check("body aabb available", body_aabb is not None)
    if body_aabb is not None:
        (xmin, ymin, zmin), (xmax, ymax, zmax) = body_aabb
        ctx.check(
            "body depth ~0.32 m",
            0.28 <= (xmax - xmin) <= 0.36,
            details=f"depth={xmax - xmin:.3f}",
        )
        ctx.check(
            "body width ~0.28 m",
            0.24 <= (ymax - ymin) <= 0.31,
            details=f"width={ymax - ymin:.3f}",
        )
        ctx.check(
            "lower bowl height matches split line",
            0.18 <= (zmax - zmin) <= 0.23,
            details=f"bowl_height={zmax - zmin:.3f}",
        )
        ctx.check(
            "body grounded at z=0", abs(zmin) < 0.002, details=f"zmin={zmin:.4f}"
        )

    # ---- closed pose: lid seated on body at the split seam
    lid_aabb = ctx.part_world_aabb(lid)
    ctx.check("lid aabb available", lid_aabb is not None)
    if lid_aabb is not None:
        ctx.check(
            "lid bottom near the split line",
            abs(lid_aabb[0][2] - SPLIT_Z) < 0.015,
            details=f"lid_zmin={lid_aabb[0][2]:.4f}",
        )
        ctx.check(
            "lid top reaches near body height",
            lid_aabb[1][2] > BODY_H - 0.020,
            details=f"lid_zmax={lid_aabb[1][2]:.4f}",
        )

    # Combined appliance height: body bowl + lid reaches ~0.33 m
    if body_aabb is not None and lid_aabb is not None:
        total_zmax = max(body_aabb[1][2], lid_aabb[1][2])
        total_zmin = min(body_aabb[0][2], lid_aabb[0][2])
        ctx.check(
            "appliance total height ~0.33 m",
            0.30 <= (total_zmax - total_zmin) <= 0.36,
            details=f"total_height={total_zmax - total_zmin:.3f}",
        )

    # ---- basket is fixed inside the body bowl
    basket_aabb = ctx.part_element_world_aabb(body, elem="basket")
    ctx.check("basket aabb available", basket_aabb is not None)
    if basket_aabb is not None:
        ctx.check(
            "basket sits inside the lower bowl",
            basket_aabb[0][2] > 0.0 and basket_aabb[1][2] < SPLIT_Z,
            details=f"basket_z=({basket_aabb[0][2]:.3f},{basket_aabb[1][2]:.3f})",
        )
    fries_aabb = ctx.part_element_world_aabb(body, elem="fries_heap")
    if basket_aabb is not None and fries_aabb is not None:
        ctx.expect_within(
            body,
            body,
            axes="xy",
            inner_elem="fries_heap",
            outer_elem="basket",
            name="fries contained in basket",
        )
        ctx.check(
            "basket open-topped above food",
            fries_aabb[1][2] < basket_aabb[1][2],
            details=f"fries_zmax={fries_aabb[1][2]:.4f}, basket_zmax={basket_aabb[1][2]:.4f}",
        )

    # ---- hinge barrels provide visible pivot hardware
    for i in range(2):
        ba = ctx.part_element_world_aabb(body, elem=f"hinge_barrel_{i}")
        if ba is not None:
            ctx.check(
                f"hinge_barrel_{i} at the rear hinge area",
                ba[0][0] < -0.10 and abs(ba[0][2] - SPLIT_Z) < 0.020,
                details=f"barrel={ba}",
            )

    # ---- intentional overlaps: hinge barrels captured in lid rear wall
    for i in range(2):
        ctx.allow_overlap(
            body,
            lid,
            elem_a=f"hinge_barrel_{i}",
            elem_b="lid_shell",
            reason=(
                f"Hinge barrel {i} is captured inside the lid rear wall "
                "for the revolute pivot, matching a real barrel hinge."
            ),
        )
        ctx.expect_overlap(
            body,
            lid,
            axes="z",
            elem_a=f"hinge_barrel_{i}",
            elem_b="lid_shell",
            min_overlap=0.001,
            name=f"hinge_barrel_{i} nests inside lid rear wall",
        )

    # ---- intentional overlap: trim band wraps around the lid seam
    ctx.allow_overlap(
        body,
        lid,
        elem_a="rim_trim_band",
        elem_b="lid_shell",
        reason=(
            "Copper trim band wraps around the split seam and embeds ~2 mm "
            "into the lid shell outer wall for a flush decorative fit."
        ),
    )
    ctx.expect_overlap(
        body,
        lid,
        axes="z",
        elem_a="rim_trim_band",
        elem_b="lid_shell",
        min_overlap=0.001,
        name="trim band overlaps the lid seam region",
    )

    # ---- copper trim band at the split seam
    band = ctx.part_element_world_aabb(body, elem="rim_trim_band")
    if band is not None:
        ctx.check(
            "copper trim band wraps the split seam",
            abs((band[0][2] + band[1][2]) / 2.0 - SPLIT_Z) < 0.015,
            details=f"band_z=({band[0][2]:.3f},{band[1][2]:.3f})",
        )

    # ---- glass touch panel on lid top
    glass = ctx.part_element_world_aabb(lid, elem="top_glass_panel")
    if glass is not None:
        ctx.check(
            "glass touch panel on lid top",
            glass[1][2] > BODY_H - 0.010,
            details=f"glass_zmax={glass[1][2]:.4f}",
        )

    # ---- lid handle on front of lid
    handle = ctx.part_element_world_aabb(lid, elem="lid_handle")
    if handle is not None:
        ctx.check(
            "lid handle on the front of the lid",
            handle[1][0] > 0.10,
            details=f"handle_xmax={handle[1][0]:.4f}",
        )

    # ---- brand logo on front face below the seam
    logo = ctx.part_element_world_aabb(body, elem="brand_logo")
    if logo is not None:
        ctx.check(
            "brand logo on the front face below the split",
            logo[1][0] > 0.14 and 0.10 < logo[0][2] < SPLIT_Z,
            details=f"logo={logo}",
        )

    # ---- open pose: lid rotates upward, basket exposed from above
    closed_handle = ctx.part_element_world_aabb(lid, elem="lid_handle")
    with ctx.pose({hinge: 1.5}):
        open_handle = ctx.part_element_world_aabb(lid, elem="lid_handle")
        lid_open_aabb = ctx.part_world_aabb(lid)
        if lid_open_aabb is not None:
            ctx.check(
                "open lid rises well above the body",
                lid_open_aabb[1][2] > BODY_H + 0.05,
                details=f"lid_zmax_open={lid_open_aabb[1][2]:.4f}",
            )
        if closed_handle is not None and open_handle is not None:
            ctx.check(
                "handle rises significantly on opening",
                open_handle[1][2] > closed_handle[1][2] + 0.10,
                details=(
                    f"closed_handle_zmax={closed_handle[1][2]:.4f}, "
                    f"open_handle_zmax={open_handle[1][2]:.4f}"
                ),
            )
            ctx.check(
                "open handle above the body top",
                open_handle[0][2] > BODY_H,
                details=f"open_handle_zmin={open_handle[0][2]:.4f}",
            )

    return ctx.report()


object_model = build_object_model()
