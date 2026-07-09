from __future__ import annotations

"""Saddle brown leather knitting needle case with zip-around closure.

A bifold leather organizer that lies open on the table. Each interior leather
panel holds a row of colorful interchangeable knitting needle tips: the
needles rest on the panel and thread under a stitched pocket band with divider
slots, and a hinged leather flap folds down over the exposed needle tips. The
two panels fold together at a central spine seam. A zip-around closure with
zipper tapes along the outer edges and a sliding brass zipper pull secures
the case when folded shut.

Layout (rest pose, world frame): the case is fully open and lying flat, z up.
The base panel spans x in [-0.085, 0.085]; the fold panel continues on the +x
side of the spine seam; zipper tapes run along the three free edges of each
panel (top, outer, bottom) and a brass slider/pull travels along the fold
panel's outer tape on a prismatic joint.
"""

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    CylinderGeometry,
    ExtrudeGeometry,
    Mesh,
    MotionLimits,
    Origin,
    Part,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

# ---------------------------------------------------------------- dimensions
PANEL_W = 0.17  # leather panel width along x
PANEL_H = 0.24  # leather panel height along y
PANEL_T = 0.004  # leather panel thickness
CORNER_R = 0.012

SEAM_OVER = 0.0004  # leather is continuous across fold seams: tiny hidden embed
HINGE_Z = 0.0045  # spine hinge line height (just above leather top)

# Needle bed
NEEDLE_R = 0.0025
NEEDLE_SHAFT_L = 0.115
NEEDLE_TIP_L = 0.018
NEEDLE_BASE_Y = -0.098  # needle butt end (tucked inside the pocket band)
NEEDLE_XS = (-0.060, -0.036, -0.012, 0.012, 0.036, 0.060)
NEEDLE_SINK = 0.0003  # slight settle into the leather so the row reads seated

# Pocket band (leather strap the needles slide under, with stitched dividers)
BAND_Y = -0.085
BAND_H = 0.060
DIVIDER_XS = (-0.072, -0.048, -0.024, 0.0, 0.024, 0.048, 0.072)
DIVIDER_W = 0.0022
DIVIDER_RISE = 0.0062  # divider top above the panel surface
BAND_TOP_T = 0.0025
BAND_W = 0.1466

# Needle flap (folds down over the needle tips)
FLAP_HINGE_Y = 0.048
FLAP_LEN = 0.062
FLAP_W = 0.150
FLAP_T = 0.003
FLAP_CLEAR = 0.0003  # closed flap floats this far above the needle tips
RIDGE_D = 0.006
# Ridge front face tucks 0.3 mm under the flap hinge edge (stitched seam).
RIDGE_Y = FLAP_HINGE_Y - 0.0003 + RIDGE_D / 2
RIDGE_RISE = 0.007

FOLD_HINGE_X = PANEL_W / 2  # spine fold line sits at the base panel edge

# Zip tape (fabric tape sewn along three free panel edges)
ZIP_TAPE_W = 0.008  # tape width
ZIP_TAPE_T = 0.0015  # tape thickness (sits on panel surface)
ZIP_TAPE_INSET = 0.012  # outer tape y-inset from panel corners
ZIP_TEETH_PITCH = 0.016  # spacing between interlocking teeth
ZIP_TEETH_D = 0.002  # tooth protrusion from tape inner edge
ZIP_TEETH_W = 0.003  # tooth width along travel axis
ZIP_TEETH_H = 0.0015  # tooth height above tape surface

# Zip pull slider (travels along the fold panel outer tape)
PULL_BODY_W = 0.012  # slider width across tape (x)
PULL_BODY_L = 0.014  # slider length along travel (y)
PULL_BODY_H = 0.005  # slider height (z)
PULL_TAB_L = 0.018  # pull tab length (x)
PULL_TAB_W = 0.007  # pull tab width (y)
PULL_TAB_T = 0.002  # pull tab thickness (z)

NEEDLE_RGBAS = (
    (0.16, 0.55, 0.55, 1.0),  # teal
    (0.48, 0.30, 0.62, 1.0),  # purple
    (0.75, 0.22, 0.24, 1.0),  # red
    (0.23, 0.38, 0.72, 1.0),  # blue
    (0.33, 0.58, 0.30, 1.0),  # green
    (0.85, 0.52, 0.18, 1.0),  # orange
)


def _panel_mesh(width: float, height: float, thickness: float):
    """Rounded-corner leather panel spanning z in [0, thickness]."""
    return ExtrudeGeometry.from_z0(
        rounded_rect_profile(width, height, CORNER_R, corner_segments=8),
        thickness,
    )


def _needle_geometry():
    """One interchangeable needle tip lying along +y, butt end at y=0.

    Cylindrical shaft with a conical point; the cone is embedded 2 mm into the
    shaft end so the mesh is one connected solid.
    """
    shaft = CylinderGeometry(NEEDLE_R, NEEDLE_SHAFT_L, radial_segments=20)
    shaft.translate(0.0, 0.0, NEEDLE_SHAFT_L / 2)
    tip = ConeGeometry(NEEDLE_R, NEEDLE_TIP_L, radial_segments=20)
    tip.translate(0.0, 0.0, NEEDLE_SHAFT_L - 0.002 + NEEDLE_TIP_L / 2)
    needle = shaft.merge(tip)
    needle.rotate_x(-math.pi / 2)  # +z -> +y
    return needle


def _add_needle_bed(part: Part, needle_mesh: Mesh, cx: float, top_z: float) -> None:
    """Needle row, stitched pocket band, and flap seam ridge on one panel.

    ``cx`` is the panel center x and ``top_z`` the panel top-surface height,
    both in the part's local frame.
    """
    needle_cz = top_z + NEEDLE_R - NEEDLE_SINK
    for i, dx in enumerate(NEEDLE_XS):
        part.visual(
            needle_mesh,
            origin=Origin(xyz=(cx + dx, NEEDLE_BASE_Y, needle_cz)),
            material=f"needle_{i}",
            name=f"needle_{i}",
        )

    # Stitched dividers between the needle slots (sunk 0.5 mm into the panel).
    div_h = DIVIDER_RISE + 0.0005
    for i, dx in enumerate(DIVIDER_XS):
        part.visual(
            Box((DIVIDER_W, BAND_H, div_h)),
            origin=Origin(xyz=(cx + dx, BAND_Y, top_z - 0.0005 + div_h / 2)),
            material="leather_dark",
            name=f"band_divider_{i}",
        )

    # Band top strap resting on the dividers (0.3 mm overlap so it reads sewn).
    band_cz = top_z + DIVIDER_RISE - 0.0003 + BAND_TOP_T / 2
    part.visual(
        Box((BAND_W, BAND_H, BAND_TOP_T)),
        origin=Origin(xyz=(cx, BAND_Y, band_cz)),
        material="leather_band",
        name="pocket_band",
    )

    # Cream stitch lines on the band above each divider.
    stitch_cz = band_cz + BAND_TOP_T / 2 + 0.0001
    for i, dx in enumerate(DIVIDER_XS):
        part.visual(
            Box((0.0012, BAND_H - 0.004, 0.0006)),
            origin=Origin(xyz=(cx + dx, BAND_Y, stitch_cz)),
            material="stitch_cream",
            name=f"band_stitch_{i}",
        )

    # Stitched seam ridge that carries the needle-flap hinge (sunk into panel).
    ridge_h = RIDGE_RISE + 0.0005
    part.visual(
        Box((FLAP_W, RIDGE_D, ridge_h)),
        origin=Origin(xyz=(cx, RIDGE_Y, top_z - 0.0005 + ridge_h / 2)),
        material="leather_dark",
        name="flap_seam_ridge",
    )


def _build_needle_flap(model: ArticulatedObject, name: str) -> Part:
    """Leather flap hinged at its +y edge, extending along local -y."""
    flap = model.part(name)
    flap.visual(
        Box((FLAP_W, FLAP_LEN, FLAP_T)),
        origin=Origin(xyz=(0.0, -FLAP_LEN / 2, 0.0)),
        material="leather_flap",
        name="flap_leather",
    )
    # Leather pull tab overlapping the free edge (for lifting the flap open).
    flap.visual(
        Box((0.030, 0.016, 0.0025)),
        origin=Origin(xyz=(0.0, -0.066, 0.0)),
        material="leather_band",
        name="pull_tab",
    )
    return flap


def _add_zip_tape(
    part: Part,
    outer_x_sign: int,
    panel_cx: float,
    top_z: float,
) -> tuple[float, float]:
    """Add zip-tape visuals along the three free edges of one panel.

    Returns (outer_tape_center_x, outer_tape_length) for the caller to place
    teeth and the slider joint.

    ``outer_x_sign`` is +1 when the free edge is at local +x (fold panel) and
    -1 when it is at local -x (base panel).
    """
    tape_hw = ZIP_TAPE_W / 2
    panel_hw = PANEL_W / 2
    panel_hh = PANEL_H / 2
    tape_z = top_z + ZIP_TAPE_T / 2

    # Outer-edge tape center x (inset half-width from the panel edge).
    outer_tape_x = panel_cx + outer_x_sign * (panel_hw - tape_hw)

    # Outer tape length along y (inset from corners).
    outer_len = 2.0 * (panel_hh - ZIP_TAPE_INSET)
    part.visual(
        Box((ZIP_TAPE_W, outer_len, ZIP_TAPE_T)),
        origin=Origin(xyz=(outer_tape_x, 0.0, tape_z)),
        material="zip_fabric",
        name="zip_tape_outer",
    )

    # Top-edge tape (runs along x from spine to outer edge).
    spine_x = panel_cx - outer_x_sign * panel_hw
    outer_edge_x = outer_tape_x + outer_x_sign * tape_hw
    side_len = abs(outer_edge_x - spine_x)
    side_cx = (outer_edge_x + spine_x) / 2.0
    top_y = panel_hh - tape_hw
    part.visual(
        Box((side_len, ZIP_TAPE_W, ZIP_TAPE_T)),
        origin=Origin(xyz=(side_cx, top_y, tape_z)),
        material="zip_fabric",
        name="zip_tape_top",
    )

    # Bottom-edge tape (mirror of top).
    bot_y = -(panel_hh - tape_hw)
    part.visual(
        Box((side_len, ZIP_TAPE_W, ZIP_TAPE_T)),
        origin=Origin(xyz=(side_cx, bot_y, tape_z)),
        material="zip_fabric",
        name="zip_tape_bottom",
    )

    # Interlocking teeth along the inner edge of the outer tape.
    tooth_z = top_z + ZIP_TAPE_T + ZIP_TEETH_H / 2
    tooth_x = outer_tape_x - outer_x_sign * (tape_hw + ZIP_TEETH_D / 2)
    n_teeth = max(1, int(outer_len / ZIP_TEETH_PITCH))
    for i in range(n_teeth):
        ty = -outer_len / 2 + ZIP_TEETH_PITCH / 2 + i * ZIP_TEETH_PITCH
        part.visual(
            Box((ZIP_TEETH_D, ZIP_TEETH_W, ZIP_TEETH_H)),
            origin=Origin(xyz=(tooth_x, ty, tooth_z)),
            material="brass",
            name=f"zip_tooth_{i}",
        )

    return outer_tape_x, outer_len


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="leather_knitting_needle_case")

    model.material("leather_saddle", rgba=(0.47, 0.26, 0.14, 1.0))
    model.material("leather_band", rgba=(0.54, 0.31, 0.17, 1.0))
    model.material("leather_flap", rgba=(0.50, 0.28, 0.15, 1.0))
    model.material("leather_dark", rgba=(0.40, 0.21, 0.11, 1.0))
    model.material("stitch_cream", rgba=(0.91, 0.86, 0.72, 1.0))
    model.material("brass", rgba=(0.76, 0.62, 0.30, 1.0))
    model.material("zip_fabric", rgba=(0.18, 0.12, 0.07, 1.0))
    model.material("zip_slider", rgba=(0.72, 0.58, 0.28, 1.0))
    for i, rgba in enumerate(NEEDLE_RGBAS):
        model.material(f"needle_{i}", rgba=rgba)

    panel_mesh = mesh_from_geometry(_panel_mesh(PANEL_W, PANEL_H, PANEL_T), "leather_panel")
    needle_mesh = mesh_from_geometry(_needle_geometry(), "needle_tip")

    # ------------------------------------------------------------ base panel
    base = model.part("base_panel")
    base.visual(
        panel_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="leather_saddle",
        name="panel_shell",
    )
    _add_needle_bed(base, needle_mesh, cx=0.0, top_z=PANEL_T)
    # Zip tape along three free edges (outer -x, top, bottom).
    _add_zip_tape(base, outer_x_sign=-1, panel_cx=0.0, top_z=PANEL_T)

    # ------------------------------------------------------------ fold panel
    # Hinged at the spine seam: the local frame origin sits on the fold line
    # (world z = HINGE_Z) and the panel extends along local +x with its top
    # surface at local z = -0.0005, i.e. coplanar with the base panel at rest.
    fold = model.part("fold_panel")
    fold_top = -0.0005
    fold_cx = PANEL_W / 2 - SEAM_OVER
    fold.visual(
        panel_mesh,
        origin=Origin(xyz=(fold_cx, 0.0, fold_top - PANEL_T)),
        material="leather_saddle",
        name="panel_shell",
    )
    _add_needle_bed(fold, needle_mesh, cx=fold_cx, top_z=fold_top)
    # Zip tape along three free edges (outer +x, top, bottom).
    fold_outer_tape_x, fold_outer_tape_len = _add_zip_tape(
        fold, outer_x_sign=+1, panel_cx=fold_cx, top_z=fold_top
    )

    model.articulation(
        "base_panel_to_fold_panel",
        ArticulationType.REVOLUTE,
        parent=base,
        child=fold,
        origin=Origin(xyz=(FOLD_HINGE_X, 0.0, HINGE_Z)),
        # The fold panel extends along local +x from the spine; -y lifts its
        # free edge upward so positive q folds the case shut like a book.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=math.pi),
    )

    # ------------------------------------------------------------- zip pull
    # Brass slider body sits atop the fold panel's outer zip tape; a pull tab
    # extends inward (-x) for finger grip. The prismatic joint drives the
    # pull along +y (from the bottom of the tape toward the top).
    zip_pull = model.part("zip_pull")
    # Slider body: centered above the joint origin (which sits at teeth top).
    zip_pull.visual(
        Box((PULL_BODY_W, PULL_BODY_L, PULL_BODY_H)),
        origin=Origin(xyz=(0.0, 0.0, PULL_BODY_H / 2)),
        material="zip_slider",
        name="slider_body",
    )
    # Pull tab extending inward (-x) from the slider body.
    tab_cx = -(PULL_BODY_W / 2 + PULL_TAB_L / 2 - 0.003)
    zip_pull.visual(
        Box((PULL_TAB_L, PULL_TAB_W, PULL_TAB_T)),
        origin=Origin(xyz=(tab_cx, 0.0, PULL_TAB_T / 2)),
        material="zip_slider",
        name="pull_tab",
    )
    # Small ring connector between body and tab (ensures visual connectivity).
    zip_pull.visual(
        Box((0.004, 0.005, PULL_BODY_H - PULL_TAB_T)),
        origin=Origin(xyz=(-PULL_BODY_W / 2 + 0.001, 0.0, PULL_TAB_T + (PULL_BODY_H - PULL_TAB_T) / 2)),
        material="zip_slider",
        name="tab_ring",
    )

    # Prismatic joint: joint origin at the bottom of the fold panel's outer
    # tape, so q=0 parks the pull at the bottom and q=upper slides it up.
    teeth_top_z = fold_top + ZIP_TAPE_T + ZIP_TEETH_H
    pull_start_y = -fold_outer_tape_len / 2 + PULL_BODY_L / 2
    pull_travel = fold_outer_tape_len - PULL_BODY_L  # max usable travel

    model.articulation(
        "fold_panel_to_zip_pull",
        ArticulationType.PRISMATIC,
        parent=fold,
        child=zip_pull,
        origin=Origin(xyz=(fold_outer_tape_x, pull_start_y, teeth_top_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0, velocity=0.5, lower=0.0, upper=pull_travel
        ),
    )

    # ---------------------------------------------------------- needle flaps
    # Each flap hinges on the stitched seam ridge above the needle tips and at
    # rest lies flat with 0.3 mm slot clearance above the tips it protects.
    for panel, cx, top_z, flap_name, joint_name in (
        (base, 0.0, PANEL_T, "base_needle_flap", "base_panel_to_needle_flap"),
        (fold, fold_cx, fold_top, "fold_needle_flap", "fold_panel_to_needle_flap"),
    ):
        flap = _build_needle_flap(model, flap_name)
        hinge_z = top_z + 2 * NEEDLE_R - NEEDLE_SINK + FLAP_CLEAR + FLAP_T / 2
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=panel,
            child=flap,
            origin=Origin(xyz=(cx, FLAP_HINGE_Y, hinge_z)),
            # The flap extends along local -y from its hinge; -x lifts the
            # free edge upward so positive q opens it away from the needles.
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=2.7),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_panel")
    fold = object_model.get_part("fold_panel")
    zip_pull = object_model.get_part("zip_pull")
    base_flap = object_model.get_part("base_needle_flap")
    fold_flap = object_model.get_part("fold_needle_flap")
    spine = object_model.get_articulation("base_panel_to_fold_panel")
    zip_joint = object_model.get_articulation("fold_panel_to_zip_pull")
    base_flap_hinge = object_model.get_articulation("base_panel_to_needle_flap")

    # --- fold seam: continuous leather, tiny hidden embed -------------------
    ctx.allow_overlap(
        base,
        fold,
        elem_a="panel_shell",
        elem_b="panel_shell",
        reason="The leather is one continuous piece across the spine fold; the panel edges share a 0.4 mm hidden embed at the fold line.",
    )
    ctx.expect_contact(
        base,
        fold,
        elem_a="panel_shell",
        elem_b="panel_shell",
        name="fold panel leather is continuous with the base panel at the spine",
    )

    # --- open-book layout at rest -------------------------------------------
    ctx.expect_origin_gap(
        fold,
        base,
        axis="x",
        min_gap=0.06,
        max_gap=0.12,
        name="fold panel hinges outboard of the base panel at the spine",
    )

    base_aabb = ctx.part_world_aabb(base)
    fold_aabb = ctx.part_world_aabb(fold)
    ctx.check(
        "open case lies flat: both panels rest on the same table plane",
        base_aabb is not None
        and fold_aabb is not None
        and abs(fold_aabb[0][2] - base_aabb[0][2]) < 0.0015,
        details=f"base={base_aabb}, fold={fold_aabb}",
    )

    # --- zip tape present on both panels ------------------------------------
    for panel in (base, fold):
        tape = ctx.part_element_world_aabb(panel, elem="zip_tape_outer")
        shell = ctx.part_element_world_aabb(panel, elem="panel_shell")
        ctx.check(
            f"{panel.name} has a zip_tape_outer rail along its free edge",
            tape is not None and shell is not None,
            details=f"tape={tape}",
        )

    # --- zip pull sits on the fold panel outer tape -------------------------
    ctx.expect_contact(
        fold,
        zip_pull,
        elem_a="zip_tape_outer",
        elem_b="slider_body",
        contact_tol=0.002,
        name="zip_pull slider_body is seated on the fold panel zip tape",
    )

    # --- zip pull prismatic travel along the tape rail ----------------------
    rest_pos = ctx.part_world_position(zip_pull)
    travel_upper = zip_joint.motion_limits.upper or 0.0
    with ctx.pose({zip_joint: travel_upper * 0.8}):
        moved_pos = ctx.part_world_position(zip_pull)
        ctx.check(
            "fold_panel_to_zip_pull prismatic joint slides the zip_pull along the tape rail in +y",
            rest_pos is not None
            and moved_pos is not None
            and moved_pos[1] > rest_pos[1] + 0.05,
            details=f"rest={rest_pos}, moved={moved_pos}, upper={travel_upper}",
        )

    # Verify the pull remains on the fold panel tape at max travel.
    with ctx.pose({zip_joint: travel_upper}):
        ctx.expect_overlap(
            zip_pull,
            fold,
            axes="y",
            elem_a="slider_body",
            elem_b="zip_tape_outer",
            min_overlap=0.005,
            name="zip_pull slider_body remains on the zip tape at full travel",
        )

    # --- needle rows threaded under the stitched pocket bands ---------------
    for panel in (base, fold):
        band = ctx.part_element_world_aabb(panel, elem="pocket_band")
        shell = ctx.part_element_world_aabb(panel, elem="panel_shell")
        for i in range(len(NEEDLE_XS)):
            needle = ctx.part_element_world_aabb(panel, elem=f"needle_{i}")
            ok = (
                band is not None
                and shell is not None
                and needle is not None
                # the shaft passes under the band strap
                and needle[1][2] <= band[0][2] + 1e-6
                # and rests on the panel leather
                and needle[0][2] <= shell[1][2] + 1e-6
                # butt end tucked inside the band footprint
                and band[0][1] - 1e-6 <= needle[0][1] <= band[1][1] + 1e-6
                and band[0][0] <= needle[0][0]
                and needle[1][0] <= band[1][0]
            )
            ctx.check(
                f"{panel.name} needle_{i} threads under the stitched pocket band",
                ok,
                details=f"needle={needle}, band={band}, shell={shell}",
            )

    # --- needle flaps cover the tips with slot clearance --------------------
    for panel, flap in ((base, base_flap), (fold, fold_flap)):
        ctx.expect_gap(
            flap,
            panel,
            axis="z",
            min_gap=0.0,
            max_gap=0.002,
            positive_elem="flap_leather",
            negative_elem="needle_2",
            name=f"{flap.name} rests just above the needle tips",
        )
        ctx.expect_overlap(
            flap,
            panel,
            axes="y",
            elem_a="flap_leather",
            elem_b="needle_2",
            min_overlap=0.04,
            name=f"{flap.name} covers the needle tip zone",
        )
        ctx.expect_overlap(
            flap,
            panel,
            axes="x",
            elem_a="flap_leather",
            elem_b="pocket_band",
            min_overlap=0.14,
            name=f"{flap.name} spans the full needle row",
        )
        ctx.allow_overlap(
            panel,
            flap,
            elem_a="flap_seam_ridge",
            elem_b="flap_leather",
            reason="The flap is stitched onto the seam ridge; the hinge edge shares a 0.3 mm hidden embed with the ridge front face.",
        )
        ctx.expect_contact(
            panel,
            flap,
            elem_a="flap_seam_ridge",
            elem_b="flap_leather",
            name=f"{flap.name} hinge edge is stitched to the seam ridge",
        )

    # --- decisive articulated poses ------------------------------------------
    with ctx.pose({spine: 1.3}):
        posed = ctx.part_world_aabb(fold)
        ctx.check(
            "folding the spine lifts the fold panel free edge upward and inward",
            fold_aabb is not None
            and posed is not None
            and posed[1][2] > 0.05
            and posed[1][0] < fold_aabb[1][0] - 0.02,
            details=f"rest={fold_aabb}, posed={posed}",
        )

    rest_flap_aabb = ctx.part_world_aabb(base_flap)
    with ctx.pose({base_flap_hinge: 1.8}):
        posed = ctx.part_world_aabb(base_flap)
        ctx.check(
            "needle flap swings up and away from the needle tips",
            rest_flap_aabb is not None
            and posed is not None
            and posed[1][2] > rest_flap_aabb[1][2] + 0.03
            and posed[0][1] > rest_flap_aabb[0][1] + 0.02,
            details=f"rest={rest_flap_aabb}, posed={posed}",
        )

    return ctx.report()


object_model = build_object_model()
