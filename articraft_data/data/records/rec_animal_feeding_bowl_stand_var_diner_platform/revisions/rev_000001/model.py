from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(cx: float, cy: float, r: float, segments: int = 48) -> list[tuple[float, float]]:
    """Return a CCW circular profile centered at (cx, cy)."""
    return [
        (cx + r * math.cos(2.0 * math.pi * i / segments),
         cy + r * math.sin(2.0 * math.pi * i / segments))
        for i in range(segments)
    ]


def _bowl_cup_mesh(name: str):
    """Thin open stainless bowl shell with rolled-lip top edge."""
    outer = [
        (0.028, -0.090),
        (0.046, -0.084),
        (0.066, -0.065),
        (0.080, -0.034),
        (0.087, -0.006),
        (0.088, 0.004),
        (0.090, 0.0052),
        (0.095, 0.00942),
    ]
    inner = [
        (0.020, -0.078),
        (0.040, -0.073),
        (0.060, -0.056),
        (0.074, -0.028),
        (0.081, -0.004),
        (0.083, 0.004),
        (0.089, 0.00942),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer,
            inner,
            segments=72,
            lip_samples=5,
        ),
        name,
    )


# ── Frame dimensions ────────────────────────────────────────────────
_PANEL_W = 0.500          # top panel width  (X)
_PANEL_D = 0.260          # top panel depth  (Y)
_PANEL_T = 0.022          # top panel thickness (Z)
_PANEL_Z = 0.300          # top panel center Z above floor
_LEG_W = 0.036            # leg cross-section
_LEG_H = _PANEL_Z - _PANEL_T / 2.0   # leg height = floor to panel underside
_CUTOUT_R = 0.093         # cutout radius (smaller than bowl rim 0.099)
_BOWL_X = (-0.130, 0.130) # bowl cutout centers along X
_CORNER_R = 0.012         # panel corner rounding


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="raised_pet_bowl_stand")

    # ── Materials ────────────────────────────────────────────────────
    walnut = model.material("walnut", rgba=(0.36, 0.22, 0.13, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.82, 0.82, 0.78, 1.0))

    # ── Root: rigid four-leg raised frame ────────────────────────────
    base = model.part("base_frame")

    # Top panel with two circular bowl cutouts (extruded with holes)
    outer = rounded_rect_profile(_PANEL_W, _PANEL_D, _CORNER_R, corner_segments=6)
    hole_0 = _circle_profile(_BOWL_X[0], 0.0, _CUTOUT_R, segments=48)
    hole_1 = _circle_profile(_BOWL_X[1], 0.0, _CUTOUT_R, segments=48)

    panel_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer,
            [hole_0, hole_1],
            _PANEL_T,
            cap=True,
            center=True,
        ),
        "top_panel",
    )
    base.visual(
        panel_mesh,
        origin=Origin(xyz=(0.0, 0.0, _PANEL_Z)),
        material=walnut,
        name="top_panel",
    )

    # Four legs at the corners
    leg_positions = [
        (-_PANEL_W / 2.0 + _LEG_W / 2.0 + _CORNER_R,
         -_PANEL_D / 2.0 + _LEG_W / 2.0 + _CORNER_R),
        (-_PANEL_W / 2.0 + _LEG_W / 2.0 + _CORNER_R,
          _PANEL_D / 2.0 - _LEG_W / 2.0 - _CORNER_R),
        ( _PANEL_W / 2.0 - _LEG_W / 2.0 - _CORNER_R,
         -_PANEL_D / 2.0 + _LEG_W / 2.0 + _CORNER_R),
        ( _PANEL_W / 2.0 - _LEG_W / 2.0 - _CORNER_R,
          _PANEL_D / 2.0 - _LEG_W / 2.0 - _CORNER_R),
    ]
    for i, (lx, ly) in enumerate(leg_positions):
        base.visual(
            Box((_LEG_W, _LEG_W, _LEG_H)),
            origin=Origin(xyz=(lx, ly, _LEG_H / 2.0)),
            material=walnut,
            name=f"leg_{i}",
        )

    # Rubber pads under each leg
    pad_t = 0.006
    for i, (lx, ly) in enumerate(leg_positions):
        base.visual(
            Box((_LEG_W + 0.004, _LEG_W + 0.004, pad_t)),
            origin=Origin(xyz=(lx, ly, pad_t / 2.0)),
            material=dark_rubber,
            name=f"rubber_pad_{i}",
        )

    # ── Bowls: lift-out prismatic into each cutout ───────────────────
    panel_top_z = _PANEL_Z + _PANEL_T / 2.0  # top surface of panel

    for idx, bx in enumerate(_BOWL_X):
        bowl = model.part(f"bowl_{idx}")
        bowl.visual(
            _bowl_cup_mesh(f"bowl_{idx}_cup"),
            material=stainless,
            name="cup",
        )
        bowl.visual(
            mesh_from_geometry(
                TorusGeometry(0.099, 0.0050, radial_segments=18, tubular_segments=72),
                f"bowl_{idx}_rolled_rim",
            ),
            origin=Origin(xyz=(0.0, 0.0, 0.00942)),
            material=stainless,
            name="rim",
        )
        bowl.visual(
            Cylinder(radius=0.026, length=0.005),
            origin=Origin(xyz=(0.0, 0.0, -0.0875)),
            material=stainless,
            name="flat_bottom",
        )

        # Prismatic lift-out joint: bowl seats in cutout, can be lifted up.
        # At q=0 the bowl origin is offset so the rim bottom lands on the
        # panel top surface.  Positive q lifts the bowl out of the cutout.
        rim_bottom_offset = 0.00942 - 0.005  # rim center z - tube radius
        model.articulation(
            f"base_to_bowl_{idx}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=bowl,
            origin=Origin(xyz=(bx, 0.0, panel_top_z - rim_bottom_offset)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=10.0,
                velocity=0.15,
                lower=0.0,
                upper=0.120,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    bowl_0 = object_model.get_part("bowl_0")
    bowl_1 = object_model.get_part("bowl_1")
    joint_0 = object_model.get_articulation("base_to_bowl_0")
    joint_1 = object_model.get_articulation("base_to_bowl_1")

    # ── Structural check: the rigid frame replaced the height column ─
    part_names = {p.name for p in object_model.parts}
    ctx.check(
        "no height_carriage part exists",
        "height_carriage" not in part_names,
        details="The adjustable height column should be removed in this variant.",
    )

    # ── Top panel is elevated well above the floor ───────────────────
    ctx.expect_gap(
        base, base,
        axis="z",
        min_gap=0.25,
        positive_elem="top_panel",
        negative_elem="rubber_pad_0",
        name="top panel stands well above the floor (elevated feeder)",
    )

    # ── Bowl cups intentionally pass through the panel cutouts ───────
    for idx, bowl_name in enumerate(("bowl_0", "bowl_1")):
        ctx.allow_overlap(
            "base_frame",
            bowl_name,
            elem_a="top_panel",
            elem_b="cup",
            reason=(
                f"The {bowl_name} cup shell drops through the circular panel cutout "
                "and is retained by the rim sitting on the panel surface."
            ),
        )

    # ── Bowls seated in cutouts at rest ──────────────────────────────
    for idx, bowl, joint, cutout_elem in [
        (0, bowl_0, joint_0, "top_panel"),
        (1, bowl_1, joint_1, "top_panel"),
    ]:
        # Bowl rim overlaps the panel in XY (rim sits on panel surface around cutout)
        ctx.expect_overlap(
            bowl, base,
            axes="xy",
            min_overlap=0.008,
            elem_a="rim",
            elem_b=cutout_elem,
            name=f"bowl_{idx} rim overlaps panel around cutout (seated)",
        )
        # Bowl rim sits on the panel top surface (contact or tiny gap)
        ctx.expect_gap(
            bowl, base,
            axis="z",
            max_gap=0.002,
            max_penetration=0.002,
            positive_elem="rim",
            negative_elem=cutout_elem,
            name=f"bowl_{idx} rim rests on panel top surface",
        )

    # ── Two bowls have realistic side-by-side spacing ────────────────
    ctx.expect_origin_distance(
        bowl_0, bowl_1,
        axes="x",
        min_dist=0.22,
        max_dist=0.30,
        name="two bowls keep a realistic side-by-side spacing",
    )

    # ── Prismatic lift-out: positive q raises the bowl ───────────────
    rest_z = ctx.part_world_position(bowl_0)[2]
    with ctx.pose({joint_0: 0.120}):
        lifted_z = ctx.part_world_position(bowl_0)[2]
    ctx.check(
        "prismatic lift-out raises bowl_0 upward",
        lifted_z > rest_z + 0.100,
        details=f"rest_z={rest_z:.4f}, lifted_z={lifted_z:.4f}",
    )

    # At max lift, the bowl cup bottom clears above the panel top
    with ctx.pose({joint_0: 0.120}):
        ctx.expect_gap(
            bowl_0, base,
            axis="z",
            min_gap=0.010,
            positive_elem="flat_bottom",
            negative_elem="top_panel",
            name="lifted bowl_0 flat_bottom clears above the panel",
        )

    return ctx.report()


object_model = build_object_model()
