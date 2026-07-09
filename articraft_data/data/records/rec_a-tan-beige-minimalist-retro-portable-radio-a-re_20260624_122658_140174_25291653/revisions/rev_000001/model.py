from __future__ import annotations

from math import hypot, pi, radians

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


BODY_W = 0.320
BODY_D = 0.086
BODY_H = 0.170


def _rounded_box(width: float, depth: float, height: float, radius: float) -> cq.Workplane:
    """CadQuery rounded rectangular solid, centered on its local origin."""
    return cq.Workplane("XY").box(width, depth, height).edges().fillet(radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="minimalist_retro_portable_radio")

    tan = model.material("matte_tan", rgba=(0.72, 0.52, 0.34, 1.0))
    tan_shadow = model.material("recessed_tan", rgba=(0.61, 0.43, 0.28, 1.0))
    cream = model.material("warm_button_ivory", rgba=(0.86, 0.79, 0.66, 1.0))
    black = model.material("black_speaker_mesh", rgba=(0.01, 0.011, 0.010, 1.0))
    dark = model.material("dark_backing", rgba=(0.0, 0.0, 0.0, 1.0))
    brass = model.material("brushed_brass", rgba=(0.86, 0.62, 0.25, 1.0))

    body = model.part("body")
    body_shell = _rounded_box(BODY_W, BODY_D, BODY_H, 0.018)
    body.visual(
        mesh_from_cadquery(body_shell, "rounded_body", tolerance=0.0007, angular_tolerance=0.06),
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
        material=tan,
        name="body_shell",
    )

    # A shallow front inset and a true perforated black speaker mesh fill most of
    # the radio's front face.  The black backing makes the perforations read as
    # an open mesh instead of tan holes.
    body.visual(
        Box((0.266, 0.002, 0.116)),
        origin=Origin(xyz=(0.0, -BODY_D / 2.0 + 0.0008, 0.088)),
        material=dark,
        name="speaker_backing",
    )
    speaker_bezel = BezelGeometry(
        (0.258, 0.108),
        (0.284, 0.134),
        0.006,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.008,
        outer_corner_radius=0.014,
    )
    body.visual(
        mesh_from_geometry(speaker_bezel, "speaker_bezel"),
        origin=Origin(xyz=(0.0, -BODY_D / 2.0 - 0.0030, 0.088), rpy=(pi / 2.0, 0.0, 0.0)),
        material=tan,
        name="speaker_bezel",
    )
    speaker_grille = PerforatedPanelGeometry(
        (0.252, 0.102),
        0.004,
        hole_diameter=0.0048,
        pitch=(0.010, 0.0086),
        frame=0.006,
        corner_radius=0.007,
        stagger=True,
    )
    body.visual(
        mesh_from_geometry(speaker_grille, "speaker_grille"),
        origin=Origin(xyz=(0.0, -BODY_D / 2.0 - 0.0022, 0.088), rpy=(pi / 2.0, 0.0, 0.0)),
        material=black,
        name="speaker_grille",
    )

    # Subtle top control strip, same family color as the case but slightly
    # darker, under a row of separate push buttons.
    body.visual(
        Box((0.188, 0.026, 0.003)),
        origin=Origin(xyz=(-0.010, -0.018, BODY_H + 0.0015)),
        material=tan_shadow,
        name="top_control_strip",
    )

    # Fixed brass mounting boss for the telescoping antenna (top-right of the case).
    # Placed at the depth centre so it clears the right handle saddle behind it.
    antenna_x = 0.126
    antenna_y = 0.000
    boss_h = 0.012
    body.visual(
        Cylinder(radius=0.0075, length=boss_h),
        origin=Origin(xyz=(antenna_x, antenna_y, BODY_H + boss_h / 2.0)),
        material=brass,
        name="antenna_boss",
    )
    boss_top_z = BODY_H + boss_h

    # Alternating hinge knuckles fixed to the body support the folding brass clip.
    clip_hinge_xyz = (BODY_W / 2.0 + 0.006, 0.025, 0.132)
    for idx, y in enumerate((0.017, 0.033)):
        body.visual(
            Box((0.014, 0.0065, 0.054)),
            origin=Origin(xyz=(BODY_W / 2.0, y, clip_hinge_xyz[2] - 0.020)),
            material=brass,
            name=f"clip_mount_{idx}",
        )
        body.visual(
            Cylinder(radius=0.005, length=0.007),
            origin=Origin(xyz=(clip_hinge_xyz[0], y, clip_hinge_xyz[2]), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=brass,
            name=f"clip_hinge_{idx}",
        )

    # Same-color arched carry handle: separate fixed link, with two saddles
    # seated on the top of the case and a continuous rounded strap between them.
    handle = model.part("handle")
    for x in (-0.136, 0.136):
        handle.visual(
            Box((0.028, 0.026, 0.040)),
            origin=Origin(xyz=(x, 0.027, BODY_H + 0.020)),
            material=tan,
            name=f"handle_saddle_{0 if x < 0 else 1}",
        )
    handle_path = [
        (-0.136, 0.027, BODY_H + 0.037),
        (-0.095, 0.029, BODY_H + 0.106),
        (0.000, 0.030, BODY_H + 0.134),
        (0.095, 0.029, BODY_H + 0.106),
        (0.136, 0.027, BODY_H + 0.037),
    ]
    handle_geom = sweep_profile_along_spline(
        handle_path,
        profile=rounded_rect_profile(0.012, 0.007, radius=0.002),
        samples_per_segment=14,
        spline="catmull_rom",
        cap_profile=True,
    )
    handle.visual(
        mesh_from_geometry(handle_geom, "arched_handle"),
        material=tan,
        name="handle_arch",
    )
    model.articulation(
        "body_to_handle",
        ArticulationType.FIXED,
        parent=body,
        child=handle,
        origin=Origin(),
    )

    # Five small top push buttons, each with real down-travel.
    button_shape = _rounded_box(0.026, 0.018, 0.008, 0.0025)
    button_mesh = mesh_from_cadquery(button_shape, "button_cap", tolerance=0.0004, angular_tolerance=0.05)
    button_xs = (-0.074, -0.039, -0.004, 0.031, 0.066)
    for idx, x in enumerate(button_xs):
        button = model.part(f"button_{idx}")
        button.visual(
            button_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
            material=cream,
            name="button_cap",
        )
        model.articulation(
            f"body_to_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, -0.018, BODY_H)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.08, lower=0.0, upper=0.005),
        )

    # Telescoping antenna: a brass swivel knuckle carries a visible outer sleeve.
    # The thinner inner rod always remains captured inside the sleeve mouth, so
    # collapsing the slider cannot push the rod out through the back of the hinge.
    ant_base = model.part("antenna_base")
    ant_base.visual(
        Cylinder(radius=0.0066, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=brass,
        name="antenna_knuckle",
    )
    antenna_sleeve_len = 0.090
    antenna_sleeve_bottom = 0.012
    antenna_sleeve_top = antenna_sleeve_bottom + antenna_sleeve_len
    ant_base.visual(
        Cylinder(radius=0.0047, length=antenna_sleeve_len),
        origin=Origin(xyz=(0.0, 0.0, antenna_sleeve_bottom + antenna_sleeve_len / 2.0)),
        material=brass,
        name="antenna_outer_sleeve",
    )
    ant_base.visual(
        Cylinder(radius=0.0054, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, antenna_sleeve_top - 0.0035)),
        material=brass,
        name="antenna_sleeve_lip",
    )
    model.articulation(
        "antenna_swivel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=ant_base,
        origin=Origin(xyz=(antenna_x, antenna_y, boss_top_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=radians(85.0)),
    )

    ant_rod = model.part("antenna_rod")
    antenna_rod_len = 0.105
    antenna_slide = 0.070
    ant_rod.visual(
        Cylinder(radius=0.0023, length=antenna_rod_len),
        origin=Origin(xyz=(0.0, 0.0, antenna_rod_len / 2.0)),
        material=brass,
        name="antenna_inner_rod",
    )
    ant_rod.visual(
        Sphere(radius=0.0044),
        origin=Origin(xyz=(0.0, 0.0, antenna_rod_len)),
        material=brass,
        name="antenna_tip",
    )
    # Rest pose is fully extended.  The negative lower limit retracts the rod
    # into the outer sleeve, leaving its back end still inside the sleeve body.
    model.articulation(
        "antenna_extend",
        ArticulationType.PRISMATIC,
        parent=ant_base,
        child=ant_rod,
        origin=Origin(xyz=(0.0, 0.0, antenna_sleeve_top - 0.016)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.2, lower=-antenna_slide, upper=0.0),
    )

    # Small brass folding clip, hinged beside the antenna.  The moving link has
    # a center knuckle between the two fixed body knuckles and a flat leaf that
    # folds out from the side.
    clip = model.part("folding_clip")
    clip.visual(
        Cylinder(radius=0.0048, length=0.009),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="clip_knuckle",
    )
    clip.visual(
        Box((0.004, 0.012, 0.055)),
        origin=Origin(xyz=(0.004, 0.0, -0.027)),
        material=brass,
        name="clip_leaf",
    )
    clip.visual(
        Sphere(radius=0.0045),
        origin=Origin(xyz=(0.0065, 0.0, -0.0575)),
        material=brass,
        name="clip_pull",
    )
    model.articulation(
        "body_to_clip",
        ArticulationType.REVOLUTE,
        parent=body,
        child=clip,
        origin=Origin(xyz=clip_hinge_xyz),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=1.2, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    ant_base = object_model.get_part("antenna_base")
    ant_rod = object_model.get_part("antenna_rod")
    clip = object_model.get_part("folding_clip")
    ant_swivel = object_model.get_articulation("antenna_swivel")
    ant_extend = object_model.get_articulation("antenna_extend")
    clip_joint = object_model.get_articulation("body_to_clip")

    # The front grille should dominate the front face like a minimalist retro
    # radio, while remaining mounted on the body.
    grille_aabb = ctx.part_element_world_aabb(body, elem="speaker_grille")
    body_aabb = ctx.part_element_world_aabb(body, elem="body_shell")
    ctx.check(
        "speaker grille covers most front face",
        grille_aabb is not None
        and body_aabb is not None
        and (grille_aabb[1][0] - grille_aabb[0][0]) > 0.74 * (body_aabb[1][0] - body_aabb[0][0])
        and (grille_aabb[1][2] - grille_aabb[0][2]) > 0.55 * (body_aabb[1][2] - body_aabb[0][2]),
        details=f"grille={grille_aabb}, body={body_aabb}",
    )

    ctx.expect_contact(
        handle,
        body,
        elem_a="handle_saddle_0",
        elem_b="body_shell",
        contact_tol=0.001,
        name="handle saddle sits on case",
    )
    # The swivel knuckle seats on the body boss, and the inner antenna rod stays
    # nested inside the outer sleeve as it telescopes.
    ctx.allow_overlap(
        ant_base,
        body,
        elem_a="antenna_knuckle",
        elem_b="antenna_boss",
        reason="The swivel knuckle seats on the brass mounting boss.",
    )
    ctx.allow_overlap(
        ant_rod,
        ant_base,
        elem_a="antenna_inner_rod",
        elem_b="antenna_outer_sleeve",
        reason="The thin inner rod telescopes inside the outer sleeve.",
    )
    ctx.allow_overlap(
        ant_rod,
        ant_base,
        elem_a="antenna_inner_rod",
        elem_b="antenna_sleeve_lip",
        reason="The sleeve lip wraps around the captured inner rod.",
    )
    base_pos = ctx.part_world_position(ant_base)
    ctx.check(
        "antenna mounted at the upper right",
        base_pos is not None and base_pos[0] > 0.08 and base_pos[2] > BODY_H,
        details=f"antenna base={base_pos}",
    )

    # Every top button is a distinct movable control with a tiny press stroke.
    for idx in range(5):
        button = object_model.get_part(f"button_{idx}")
        joint = object_model.get_articulation(f"body_to_button_{idx}")
        ctx.expect_gap(
            button,
            body,
            axis="z",
            positive_elem="button_cap",
            negative_elem="body_shell",
            max_gap=0.0008,
            max_penetration=0.0,
            name=f"button_{idx} rests on top",
        )
        rest = ctx.part_world_position(button)
        with ctx.pose({joint: 0.005}):
            pressed = ctx.part_world_position(button)
        ctx.check(
            f"button_{idx} presses downward",
            rest is not None and pressed is not None and pressed[2] < rest[2] - 0.004,
            details=f"rest={rest}, pressed={pressed}",
        )

    # Telescope: rest pose is extended; driving the joint to its negative lower
    # limit retracts the rod into the sleeve, lowering the tip.
    rest_tip = ctx.part_world_aabb(ant_rod)[1][2]
    with ctx.pose({ant_extend: -0.070}):
        collapsed_tip = ctx.part_world_aabb(ant_rod)[1][2]
    ctx.check(
        "antenna telescopes (tip lowers as it collapses)",
        rest_tip - collapsed_tip > 0.055,
        details=f"extended_tip={rest_tip:.3f} collapsed_tip={collapsed_tip:.3f}",
    )

    # With the antenna folded, the collapsed inner rod should still start in
    # front of the hinge instead of sliding backward through the joint.
    with ctx.pose({ant_swivel: radians(70.0), ant_extend: -0.070}):
        folded_base_pos = ctx.part_world_position(ant_base)
        folded_rod_aabb = ctx.part_element_world_aabb(ant_rod, elem="antenna_inner_rod")
    ctx.check(
        "collapsed antenna rod remains captured in sleeve when folded",
        folded_base_pos is not None
        and folded_rod_aabb is not None
        and folded_rod_aabb[0][0] > folded_base_pos[0] - 0.002,
        details=f"base={folded_base_pos}, rod={folded_rod_aabb}",
    )

    # Swivel: folding the base joint swings the rod sideways off vertical.
    rest_pos = ctx.part_world_position(ant_rod)
    with ctx.pose({ant_swivel: radians(70.0)}):
        swiveled_pos = ctx.part_world_position(ant_rod)
    ctx.check(
        "antenna swivels (folds) at the base",
        rest_pos is not None
        and swiveled_pos is not None
        and hypot(swiveled_pos[0] - rest_pos[0], swiveled_pos[1] - rest_pos[1]) > 0.03,
        details=f"rest={tuple(round(v, 3) for v in rest_pos)} swiveled={tuple(round(v, 3) for v in swiveled_pos)}",
    )

    clip_rest = ctx.part_world_aabb(clip)
    with ctx.pose({clip_joint: 1.20}):
        clip_folded = ctx.part_world_aabb(clip)
    ctx.check(
        "folding clip swings outward",
        clip_rest is not None and clip_folded is not None and clip_folded[1][0] > clip_rest[1][0] + 0.030,
        details=f"rest={clip_rest}, folded={clip_folded}",
    )

    return ctx.report()


object_model = build_object_model()
