from __future__ import annotations

"""Saddle brown leather knitting needle case.

A bifold leather organizer that lies open on the table. Each interior leather
panel holds a row of colorful interchangeable knitting needle tips: each needle
rests on the panel and threads under its own individual bent-leather retaining
loop (one loop per needle, like a tool-loop panel), and a hinged leather flap
folds down over the exposed needle tips. The two panels fold together at a
central spine seam, and a fold-over closure flap with brass snap studs wraps
the case shut.

Layout (rest pose, world frame): the case is fully open and lying flat, z up.
The base panel spans x in [-0.085, 0.085]; the fold panel continues on the +x
side of the spine seam; the closure flap continues outboard of the fold
panel's free edge, exactly as in the reference photo of the case laid open.
"""

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
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
    sweep_profile_along_spline,
)

# ---------------------------------------------------------------- dimensions
PANEL_W = 0.17  # leather panel width along x
PANEL_H = 0.24  # leather panel height along y
PANEL_T = 0.004  # leather panel thickness
CORNER_R = 0.012

SEAM_OVER = 0.0004  # leather is continuous across fold seams: tiny hidden embed
HINGE_Z = 0.0045  # spine / closure hinge line height (just above leather top)

# Needle bed
NEEDLE_R = 0.0025
NEEDLE_SHAFT_L = 0.115
NEEDLE_TIP_L = 0.018
NEEDLE_BASE_Y = -0.098  # needle butt end (tucked inside the pocket band)
NEEDLE_XS = (-0.060, -0.036, -0.012, 0.012, 0.036, 0.060)
NEEDLE_SINK = 0.0003  # slight settle into the leather so the row reads seated

# Leather retaining loops (one per needle, bent arch over each needle shaft)
LOOP_Y = -0.085  # loop anchor center y on the panel
LOOP_W = 0.012  # loop strip width (along x, the flat leather breadth)
LOOP_T = 0.0018  # loop strip thickness
LOOP_RISE = 0.007  # arch apex above the panel top surface
LOOP_SPAN = 0.018  # distance between the two feet along y

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

# Closure flap
COVER_W = 0.150
COVER_H = 0.220
COVER_T = 0.0035

FOLD_HINGE_X = PANEL_W / 2  # spine fold line sits at the base panel edge
COVER_HINGE_X = PANEL_W - SEAM_OVER  # in fold-panel local frame

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


def _loop_mesh(cx: float, base_z: float) -> Mesh:
    """One bent-leather retaining arch at panel x=``cx``, base at z=``base_z``.

    The loop is a thin flat leather strip swept along a smooth arch path that
    rises from the panel surface, peaks over the needle, and returns to the
    panel. The strip cross-section is a small rounded rectangle whose wide
    face runs along x (the loop width) and whose thin face runs along z.
    """
    half_span = LOOP_SPAN / 2
    # Path points: front foot -> rise -> apex -> descent -> rear foot
    # Arch runs along local y; the strip cross-section is in XZ.
    apex_z = base_z + LOOP_RISE
    path_points = [
        (cx, LOOP_Y - half_span, base_z),
        (cx, LOOP_Y - half_span * 0.45, base_z + LOOP_RISE * 0.55),
        (cx, LOOP_Y, apex_z),
        (cx, LOOP_Y + half_span * 0.45, base_z + LOOP_RISE * 0.55),
        (cx, LOOP_Y + half_span, base_z),
    ]
    profile = rounded_rect_profile(LOOP_W, LOOP_T, radius=0.0005, corner_segments=3)
    geom = sweep_profile_along_spline(
        path_points,
        profile=profile,
        samples_per_segment=14,
        cap_profile=True,
        up_hint=(0.0, 0.0, 1.0),
    )
    return mesh_from_geometry(geom, f"loop_{cx:.4f}")


def _add_needle_bed(part: Part, needle_mesh: Mesh, cx: float, top_z: float) -> None:
    """Needle row, leather retaining loops, and flap seam ridge on one panel.

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

    # Individual leather retaining loops — one arch per needle.
    # Each loop sits on the panel surface and arches over the needle shaft so
    # the needle threads under it. The loop base embeds 0.3 mm into the panel
    # leather so the feet read as stitched/glued onto the panel bed.
    loop_base_z = top_z - 0.0003
    for i, dx in enumerate(NEEDLE_XS):
        part.visual(
            _loop_mesh(cx + dx, loop_base_z),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material="leather_band",
            name=f"loop_{i}",
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
    # Brass snap cap near the free edge (embedded 0.5 mm into the leather).
    flap.visual(
        Cylinder(radius=0.005, length=0.0025),
        origin=Origin(xyz=(0.0, -0.050, FLAP_T / 2 + 0.00125 - 0.0005)),
        material="brass",
        name="snap_stud",
    )
    # Leather pull tab overlapping the free edge.
    flap.visual(
        Box((0.030, 0.016, 0.0025)),
        origin=Origin(xyz=(0.0, -0.066, 0.0)),
        material="leather_band",
        name="pull_tab",
    )
    return flap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="leather_knitting_needle_case")

    model.material("leather_saddle", rgba=(0.47, 0.26, 0.14, 1.0))
    model.material("leather_band", rgba=(0.54, 0.31, 0.17, 1.0))
    model.material("leather_flap", rgba=(0.50, 0.28, 0.15, 1.0))
    model.material("leather_dark", rgba=(0.40, 0.21, 0.11, 1.0))
    model.material("stitch_cream", rgba=(0.91, 0.86, 0.72, 1.0))
    model.material("brass", rgba=(0.76, 0.62, 0.30, 1.0))
    for i, rgba in enumerate(NEEDLE_RGBAS):
        model.material(f"needle_{i}", rgba=rgba)

    panel_mesh = mesh_from_geometry(_panel_mesh(PANEL_W, PANEL_H, PANEL_T), "leather_panel")
    cover_mesh = mesh_from_geometry(_panel_mesh(COVER_W, COVER_H, COVER_T), "closure_panel")
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

    # ------------------------------------------------------------ fold panel
    # Hinged at the spine seam: the local frame origin sits on the fold line
    # (world z = HINGE_Z) and the panel extends along local +x with its top
    # surface at local z = -0.0005, i.e. coplanar with the base panel at rest.
    # The leather is continuous across the fold, so the panel edge embeds
    # SEAM_OVER into the base panel edge (scoped allowance in run_tests).
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

    # ---------------------------------------------------------- closure flap
    # Continuous leather across the outer fold seam as well: the cover edge
    # embeds SEAM_OVER into the fold panel free edge.
    cover = model.part("closure_flap")
    cover.visual(
        cover_mesh,
        origin=Origin(xyz=(COVER_W / 2 - SEAM_OVER, 0.0, -HINGE_Z)),
        material="leather_saddle",
        name="cover_leather",
    )
    for i, dy in enumerate((0.080, -0.080)):
        cover.visual(
            Cylinder(radius=0.005, length=0.003),
            origin=Origin(xyz=(0.125, dy, -HINGE_Z + COVER_T + 0.0008)),
            material="brass",
            name=f"snap_stud_{i}",
        )

    model.articulation(
        "fold_panel_to_closure_flap",
        ArticulationType.REVOLUTE,
        parent=fold,
        child=cover,
        origin=Origin(xyz=(COVER_HINGE_X, 0.0, 0.0)),
        # The cover extends along local +x from its hinge; positive q folds it
        # up and over the fold panel to wrap the case shut.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=math.pi),
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
    cover = object_model.get_part("closure_flap")
    base_flap = object_model.get_part("base_needle_flap")
    fold_flap = object_model.get_part("fold_needle_flap")
    spine = object_model.get_articulation("base_panel_to_fold_panel")
    cover_hinge = object_model.get_articulation("fold_panel_to_closure_flap")
    base_flap_hinge = object_model.get_articulation("base_panel_to_needle_flap")

    # --- fold seams: continuous leather, tiny hidden embeds -----------------
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
    ctx.allow_overlap(
        fold,
        cover,
        elem_a="panel_shell",
        elem_b="cover_leather",
        reason="The closure flap is cut from the same leather as the fold panel; the edges share a 0.4 mm hidden embed at the fold line.",
    )
    ctx.expect_contact(
        fold,
        cover,
        elem_a="panel_shell",
        elem_b="cover_leather",
        name="closure flap leather is continuous with the fold panel edge",
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
    ctx.expect_origin_gap(
        cover,
        fold,
        axis="x",
        min_gap=0.15,
        max_gap=0.20,
        name="closure flap continues outboard of the fold panel free edge",
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

    # --- needle rows threaded under individual leather retaining loops ------
    for panel in (base, fold):
        shell = ctx.part_element_world_aabb(panel, elem="panel_shell")
        for i in range(len(NEEDLE_XS)):
            needle = ctx.part_element_world_aabb(panel, elem=f"needle_{i}")
            loop = ctx.part_element_world_aabb(panel, elem=f"loop_{i}")
            ok = (
                loop is not None
                and shell is not None
                and needle is not None
                # the needle shaft passes under the loop arch apex
                and needle[1][2] <= loop[1][2] + 1e-6
                # and rests on the panel leather
                and needle[0][2] <= shell[1][2] + 1e-6
                # the loop straddles the needle along y (loop y range overlaps
                # the needle shaft; needle extends beyond the loop on both ends)
                and needle[0][1] <= loop[0][1] + 1e-6
                and loop[1][1] - 1e-6 <= needle[1][1]
                # the loop is centered over the needle along x
                and abs((loop[0][0] + loop[1][0]) / 2 - (needle[0][0] + needle[1][0]) / 2) < 0.002
            )
            ctx.check(
                f"{panel.name} needle_{i} threads under its leather retaining loop",
                ok,
                details=f"needle={needle}, loop={loop}, shell={shell}",
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
            elem_b="loop_2",
            min_overlap=0.01,
            name=f"{flap.name} spans the needle loop zone",
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

    with ctx.pose({cover_hinge: 1.5}):
        posed = ctx.part_world_aabb(cover)
        ctx.check(
            "closure flap folds up over the case",
            posed is not None and posed[1][2] > 0.05,
            details=f"posed={posed}",
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
