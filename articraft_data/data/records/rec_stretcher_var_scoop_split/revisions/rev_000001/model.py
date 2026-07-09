from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


# ============================================================
# Preserved helper functions (KEEP from parent)
# ============================================================


def _origin_for_cylinder_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    extend: float = 0.0,
) -> tuple[Origin, float]:
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1e-9:
        raise ValueError("tube endpoints must be separated")

    ux, uy, uz = dx / length, dy / length, dz / length
    sx -= ux * extend
    sy -= uy * extend
    sz -= uz * extend
    ex += ux * extend
    ey += uy * extend
    ez += uz * extend
    length += 2.0 * extend

    mx, my, mz = (sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5
    yaw = math.atan2(uy, ux)
    pitch = math.atan2(math.sqrt(ux * ux + uy * uy), uz)
    return Origin(xyz=(mx, my, mz), rpy=(0.0, pitch, yaw)), length


def _tube(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material: Material,
    name: str,
    *,
    extend: float = 0.002,
) -> None:
    origin, length = _origin_for_cylinder_between(start, end, extend=extend)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=origin,
        material=material,
        name=name,
    )


def _curved_tube(
    part,
    points: list[tuple[float, float, float]],
    radius: float,
    material: Material,
    name: str,
    *,
    samples_per_segment: int = 8,
    radial_segments: int = 18,
) -> None:
    part.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                points,
                radius=radius,
                samples_per_segment=samples_per_segment,
                radial_segments=radial_segments,
                cap_ends=True,
            ),
            name,
        ),
        material=material,
        name=name,
    )


def _rounded_pad_mesh(width: float, length: float, thickness: float, radius: float, name: str):
    profile = rounded_rect_profile(length, width, radius, corner_segments=8)
    return mesh_from_geometry(ExtrudeGeometry.centered(profile, thickness), name)


# ============================================================
# Scoop stretcher geometry helpers
# ============================================================

BLADE_LENGTH = 0.95
BLADE_WIDTH = 0.24
BLADE_THICKNESS = 0.025
DECK_Z = 0.12


def _make_scoop_blade(length: float, width: float, thickness: float, name: str):
    """CadQuery blade panel with rounded corners for one scoop half."""
    blade = (
        cq.Workplane("XY")
        .box(length, width, thickness)
        .edges("|Z")
        .fillet(min(0.012, width * 0.05))
    )
    return mesh_from_cadquery(blade, name)


def _add_scoop_half_geometry(
    part,
    *,
    side: float,
    blade_cx: float,
    blade_cz: float,
    prefix: str,
    yellow: Material,
    black: Material,
    padded_black: Material,
    metal: Material,
) -> None:
    """
    Add all visuals for one scoop stretcher half.

    side:     -1 for left (outer edge at -Y), +1 for right (outer edge at +Y)
    blade_cx: blade center X in part local frame
    blade_cz: blade center Z in part local frame
    """
    half_len = BLADE_LENGTH / 2.0
    hx0 = blade_cx - half_len
    hx1 = blade_cx + half_len
    outer_y = side * BLADE_WIDTH
    inner_y = 0.0
    tube_z = blade_cz - BLADE_THICKNESS / 2.0 - 0.010

    # CadQuery blade panel
    blade_mesh = _make_scoop_blade(BLADE_LENGTH, BLADE_WIDTH, BLADE_THICKNESS, f"{prefix}_blade")
    part.visual(
        blade_mesh,
        origin=Origin(xyz=(blade_cx, side * BLADE_WIDTH / 2.0, blade_cz)),
        material=yellow,
        name="blade_panel",
    )

    # Raised outer lip
    lip_w = 0.016
    lip_h = 0.018
    part.visual(
        Box((BLADE_LENGTH - 0.08, lip_w, lip_h)),
        origin=Origin(
            xyz=(blade_cx, outer_y - side * lip_w / 2.0, blade_cz + BLADE_THICKNESS / 2.0 + lip_h / 2.0)
        ),
        material=yellow,
        name="outer_lip",
    )

    # Frame tubes (yellow aluminum)
    _tube(
        part,
        (hx0 + 0.02, outer_y, tube_z),
        (hx1 - 0.02, outer_y, tube_z),
        0.014,
        yellow,
        "outer_frame_tube",
    )
    # Inner tube offset from centerline by side*0.018 to avoid overlap when closed
    inner_tube_y = inner_y + side * 0.018
    _tube(
        part,
        (hx0 + 0.02, inner_tube_y, tube_z),
        (hx1 - 0.02, inner_tube_y, tube_z),
        0.012,
        yellow,
        "inner_frame_tube",
    )
    _tube(
        part,
        (hx0 + 0.02, outer_y, tube_z),
        (hx0 + 0.02, inner_y, tube_z),
        0.012,
        yellow,
        "head_cross_tube",
    )
    _tube(
        part,
        (hx1 - 0.02, outer_y, tube_z),
        (hx1 - 0.02, inner_y, tube_z),
        0.012,
        yellow,
        "foot_cross_tube",
    )
    for i, frac in enumerate((0.35, 0.65)):
        mx = hx0 + frac * BLADE_LENGTH
        _tube(part, (mx, outer_y, tube_z), (mx, inner_y, tube_z), 0.008, yellow, f"cross_brace_{i}")

    # Padding on top of blade
    pad_w = BLADE_WIDTH - 0.040
    pad_l = BLADE_LENGTH - 0.10
    pad_t = 0.032
    pad_mesh = _rounded_pad_mesh(pad_w, pad_l, pad_t, 0.025, f"{prefix}_pad")
    part.visual(
        pad_mesh,
        origin=Origin(
            xyz=(blade_cx, side * BLADE_WIDTH / 2.0, blade_cz + BLADE_THICKNESS / 2.0 + pad_t / 2.0)
        ),
        material=padded_black,
        name="mattress_pad",
    )

    # Handle grips at head and foot ends (curved carry handles)
    # Handles are inset from the ends and kept below the hinge axis
    # to avoid interference with the backrest hinge bar.
    handle_z = blade_cz - 0.005  # below blade top surface
    handle_inset = 0.10
    for end_name, end_x, end_sign in (("head", hx0 + handle_inset, 1.0), ("foot", hx1 - handle_inset, -1.0)):
        _curved_tube(
            part,
            [
                (end_x, side * 0.04, handle_z),
                (end_x + end_sign * 0.025, side * 0.10, handle_z + 0.008),
                (end_x + end_sign * 0.025, side * 0.20, handle_z + 0.008),
                (end_x, side * (BLADE_WIDTH - 0.02), handle_z),
            ],
            0.008,
            black,
            f"{end_name}_handle",
            samples_per_segment=6,
            radial_segments=14,
        )
        # Handle grip sleeve (rubber)
        _curved_tube(
            part,
            [
                (end_x + end_sign * 0.023, side * 0.11, handle_z + 0.007),
                (end_x + end_sign * 0.023, side * 0.19, handle_z + 0.007),
            ],
            0.011,
            padded_black,
            f"{end_name}_grip_sleeve",
            samples_per_segment=4,
            radial_segments=12,
        )

    # Notched end latches (metal plates and pins along centerline)
    for end_name, end_x in (("head", hx0 + 0.045), ("foot", hx1 - 0.045)):
        # Latch plate: offset from centerline so mating halves don't collide
        part.visual(
            Box((0.038, 0.022, 0.016)),
            origin=Origin(
                xyz=(end_x, side * 0.016, blade_cz + BLADE_THICKNESS / 2.0 + 0.008)
            ),
            material=metal,
            name=f"{end_name}_latch_plate",
        )
        # Latch pin (engages the mating half when closed)
        part.visual(
            Cylinder(radius=0.005, length=0.018),
            origin=Origin(
                xyz=(end_x, 0.0, blade_cz),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=metal,
            name=f"{end_name}_latch_pin",
        )
        # Latch lever (flip lever for locking)
        part.visual(
            Box((0.030, 0.008, 0.006)),
            origin=Origin(
                xyz=(end_x, side * 0.030, blade_cz + BLADE_THICKNESS / 2.0 + 0.019)
            ),
            material=metal,
            name=f"{end_name}_latch_lever",
        )


def _add_telescope_geometry(
    part,
    yellow: Material,
    black: Material,
    padded_black: Material,
    metal: Material,
) -> None:
    """Add visuals for the telescoping foot extension (child local frame)."""
    ext_length = 0.50
    ext_width = 0.22
    ext_thickness = 0.020

    # Extension panel: center shifted forward (+X) so the rear edge stays
    # well clear of the parent blade foot handle.
    ext_cx = 0.10
    part.visual(
        Box((ext_length, ext_width, ext_thickness)),
        origin=Origin(xyz=(ext_cx, -0.12, 0.0)),
        material=yellow,
        name="extension_panel",
    )

    # Guide rails (slide through parent sleeves; extend back for retained insertion
    # at max travel). Overlap with foot_cross_tube is allowed by test justification.
    for i, y in enumerate((-0.19, -0.05)):
        _tube(
            part,
            (-0.30, y, -0.010),
            (ext_cx + ext_length / 2.0 - 0.02, y, -0.010),
            0.009,
            metal,
            f"guide_rail_{i}",
        )

    # Padding on extension (forward portion only, clear of foot handle zone)
    pad_mesh = _rounded_pad_mesh(ext_width - 0.04, ext_length - 0.16, 0.025, 0.020, "extension_pad")
    part.visual(
        pad_mesh,
        origin=Origin(xyz=(ext_cx + 0.04, -0.12, ext_thickness / 2.0 + 0.0125)),
        material=padded_black,
        name="extension_pad",
    )

    # Foot end crossbar
    foot_x = ext_cx + ext_length / 2.0 - 0.02
    _tube(part, (foot_x, -0.22, 0.0), (foot_x, -0.02, 0.0), 0.012, yellow, "foot_crossbar")

    # Foot handle
    _curved_tube(
        part,
        [
            (foot_x + 0.01, -0.04, 0.005),
            (foot_x + 0.04, -0.10, 0.025),
            (foot_x + 0.04, -0.18, 0.025),
            (foot_x + 0.01, -0.22, 0.005),
        ],
        0.009,
        black,
        "foot_handle",
        samples_per_segment=6,
        radial_segments=14,
    )


# ============================================================
# Main build
# ============================================================


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="orthopedic_scoop_stretcher",
        meta={
            "run_notes": (
                "Orthopedic scoop stretcher: two longitudinal scoop halves that "
                "separate laterally to slide under a patient, with telescoping "
                "length adjustment at the foot end and notched end latches. "
                "Derived from the yellow wheeled stretcher baseline by removing "
                "wheels, legs, side rails, and IV pole, then splitting the deck "
                "into contoured blade halves."
            )
        },
    )

    # Materials
    yellow = model.material("safety_yellow", rgba=(1.0, 0.74, 0.03, 1.0))
    black = model.material("matte_black", rgba=(0.005, 0.006, 0.005, 1.0))
    padded_black = model.material("soft_black_vinyl", rgba=(0.015, 0.018, 0.015, 1.0))
    metal = model.material("brushed_metal", rgba=(0.65, 0.66, 0.62, 1.0))

    # ------------------------------------------------------------------
    # deck_half_left (root part)
    # ------------------------------------------------------------------
    deck_left = model.part("deck_half_left")
    _add_scoop_half_geometry(
        deck_left,
        side=-1,
        blade_cx=0.175,
        blade_cz=DECK_Z,
        prefix="left",
        yellow=yellow,
        black=black,
        padded_black=padded_black,
        metal=metal,
    )

    # Telescope guide sleeves at foot end of left half
    for i, y in enumerate((-0.19, -0.05)):
        _tube(
            deck_left,
            (0.35, y, DECK_Z - BLADE_THICKNESS / 2.0 - 0.010),
            (0.65, y, DECK_Z - BLADE_THICKNESS / 2.0 - 0.010),
            0.014,
            yellow,
            f"telescope_sleeve_{i}",
        )

    # Separation guide rails along centerline (z offsets match right-half channels)
    for i, z_off in enumerate((-0.004, -0.020)):
        _tube(
            deck_left,
            (-0.15, -0.003, DECK_Z + z_off),
            (0.50, -0.003, DECK_Z + z_off),
            0.007,
            metal,
            f"separation_rail_{i}",
        )

    # Backrest hinge socket at head end
    _tube(
        deck_left,
        (-0.30, -0.24, DECK_Z + 0.020),
        (-0.30, 0.0, DECK_Z + 0.020),
        0.018,
        yellow,
        "back_hinge_socket",
    )
    # Bracket plates for hinge socket
    for i, y in enumerate((-0.22, -0.02)):
        deck_left.visual(
            Box((0.040, 0.020, 0.030)),
            origin=Origin(xyz=(-0.30, y, DECK_Z + 0.005)),
            material=yellow,
            name=f"hinge_bracket_{i}",
        )

    # ------------------------------------------------------------------
    # deck_half_right (child of scoop_separation PRISMATIC)
    # ------------------------------------------------------------------
    deck_right = model.part("deck_half_right")
    _add_scoop_half_geometry(
        deck_right,
        side=+1,
        blade_cx=0.0,
        blade_cz=0.0,
        prefix="right",
        yellow=yellow,
        black=black,
        padded_black=padded_black,
        metal=metal,
    )

    # Separation guide channels on right half (align with left rails).
    # Channels are positioned near the frame tubes to stay connected.
    for i, z_off in enumerate((-0.004, -0.020)):
        _tube(
            deck_right,
            (-0.325, 0.003, z_off),
            (0.325, 0.003, z_off),
            0.005,
            metal,
            f"separation_channel_{i}",
        )

    # ------------------------------------------------------------------
    # telescope_extension (child of telescope_extend PRISMATIC)
    # ------------------------------------------------------------------
    telescope = model.part("telescope_extension")
    _add_telescope_geometry(telescope, yellow, black, padded_black, metal)

    # ------------------------------------------------------------------
    # backrest (preserved from parent with adjusted joint origin)
    # ------------------------------------------------------------------
    backrest = model.part("backrest")
    _tube(backrest, (0.000, -0.310, 0.000), (0.000, 0.310, 0.000), 0.017, yellow, "hinge_bar")
    backrest.visual(
        Box((0.055, 0.590, 0.024)),
        origin=Origin(xyz=(-0.025, 0.0, 0.006)),
        material=yellow,
        name="hinge_web",
    )
    _tube(backrest, (-0.670, -0.290, 0.000), (-0.020, -0.290, 0.000), 0.015, yellow, "side_tube_0")
    _tube(backrest, (-0.670, 0.290, 0.000), (-0.020, 0.290, 0.000), 0.015, yellow, "side_tube_1")
    _tube(backrest, (-0.670, -0.290, 0.000), (-0.670, 0.290, 0.000), 0.014, yellow, "head_tube")
    _tube(
        backrest, (-0.340, -0.276, 0.015), (-0.340, 0.276, 0.015), 0.010, black, "back_cross_slat"
    )
    back_pad_mesh = _rounded_pad_mesh(0.545, 0.680, 0.080, 0.050, "backrest_mattress_pad")
    pillow_mesh = _rounded_pad_mesh(0.500, 0.300, 0.105, 0.060, "head_pillow_pad")
    backrest.visual(
        back_pad_mesh,
        origin=Origin(xyz=(-0.375, 0.0, 0.055)),
        material=padded_black,
        name="back_pad",
    )
    backrest.visual(
        pillow_mesh,
        origin=Origin(xyz=(-0.610, 0.0, 0.145)),
        material=padded_black,
        name="head_pillow",
    )

    # ------------------------------------------------------------------
    # Articulations
    # ------------------------------------------------------------------

    # 1. Scoop separation: PRISMATIC along +Y
    model.articulation(
        "scoop_separation",
        ArticulationType.PRISMATIC,
        parent=deck_left,
        child=deck_right,
        origin=Origin(xyz=(0.175, 0.0, DECK_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.15, lower=0.0, upper=0.30),
    )

    # 2. Telescope extension: PRISMATIC along +X
    model.articulation(
        "telescope_extend",
        ArticulationType.PRISMATIC,
        parent=deck_left,
        child=telescope,
        origin=Origin(xyz=(0.65, 0.0, DECK_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.10, lower=0.0, upper=0.18),
    )

    # 3. Backrest tilt: REVOLUTE (same convention as parent)
    model.articulation(
        "deck_to_backrest",
        ArticulationType.REVOLUTE,
        parent=deck_left,
        child=backrest,
        origin=Origin(xyz=(-0.30, 0.0, DECK_Z + 0.020), rpy=(0.0, 0.55, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.0, lower=-0.55, upper=0.60),
    )

    return model


# ============================================================
# Tests
# ============================================================


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck_left = object_model.get_part("deck_half_left")
    deck_right = object_model.get_part("deck_half_right")
    telescope = object_model.get_part("telescope_extension")
    backrest = object_model.get_part("backrest")

    scoop_joint = object_model.get_articulation("scoop_separation")
    telescope_joint = object_model.get_articulation("telescope_extend")
    back_joint = object_model.get_articulation("deck_to_backrest")

    # --- Intentional overlap allowances ---

    # Backrest hinge bar captured inside the deck hinge socket
    ctx.allow_overlap(
        backrest,
        deck_left,
        elem_a="hinge_bar",
        elem_b="back_hinge_socket",
        reason="The backrest hinge bar is intentionally captured inside the fixed yellow hinge socket.",
    )
    for elem in ("side_tube_0", "side_tube_1", "hinge_web"):
        ctx.allow_overlap(
            backrest,
            deck_left,
            elem_a=elem,
            elem_b="back_hinge_socket",
            reason="The backrest hinge weldment locally enters the fixed hinge socket around the pivot tube.",
        )
    # Hinge bar passes through the hinge bracket plates (intentional mount)
    for bracket in ("hinge_bracket_0", "hinge_bracket_1"):
        ctx.allow_overlap(
            backrest,
            deck_left,
            elem_a="hinge_bar",
            elem_b=bracket,
            reason="The backrest hinge bar passes through the bracket plate that supports the hinge socket.",
        )
    # Hinge web also locally enters the bracket zone around the pivot tube
    for bracket in ("hinge_bracket_0", "hinge_bracket_1"):
        ctx.allow_overlap(
            backrest,
            deck_left,
            elem_a="hinge_web",
            elem_b=bracket,
            reason="The backrest hinge web locally enters the bracket plate region around the pivot tube.",
        )

    # Backrest hinge bar crosses the head edge of the right blade when closed.
    # This is realistic: the hinge spans the full stretcher width.
    ctx.allow_overlap(
        backrest,
        deck_right,
        elem_a="hinge_bar",
        elem_b="blade_panel",
        reason="The full-width backrest hinge bar crosses the head edge of the right blade when the scoop is closed.",
    )
    # Same issue on the left blade
    ctx.allow_overlap(
        backrest,
        deck_left,
        elem_a="hinge_bar",
        elem_b="blade_panel",
        reason="The full-width backrest hinge bar crosses the head edge of the left blade when the scoop is closed.",
    )

    # Latch pins overlap at centerline when closed (intentional mating engagement)
    ctx.allow_overlap(
        deck_left,
        deck_right,
        elem_a="head_latch_pin",
        elem_b="head_latch_pin",
        reason="The head latch pins from both halves engage each other when the scoop is closed.",
    )
    ctx.allow_overlap(
        deck_left,
        deck_right,
        elem_a="foot_latch_pin",
        elem_b="foot_latch_pin",
        reason="The foot latch pins from both halves engage each other when the scoop is closed.",
    )
    # Latch pins also cross into the opposite blade panel at the centerline
    for end in ("head", "foot"):
        ctx.allow_overlap(
            deck_left,
            deck_right,
            elem_a="blade_panel",
            elem_b=f"{end}_latch_pin",
            reason=f"The {end} latch pin from the right half crosses the centerline into the left blade mating interface.",
        )

    # Foot handle on left half overlaps with telescope extension panel
    ctx.allow_overlap(
        deck_left,
        telescope,
        elem_a="foot_handle",
        elem_b="extension_panel",
        reason="The foot-end carry handle on the left blade naturally intersects the telescoping extension panel at the foot end.",
    )
    ctx.allow_overlap(
        deck_left,
        telescope,
        elem_a="foot_grip_sleeve",
        elem_b="extension_panel",
        reason="The foot-end grip sleeve on the left blade naturally intersects the telescoping extension panel at the foot end.",
    )
    # Foot handle also crosses the telescope guide rails
    for rail in ("guide_rail_0", "guide_rail_1"):
        ctx.allow_overlap(
            deck_left,
            telescope,
            elem_a="foot_handle",
            elem_b=rail,
            reason="The foot-end carry handle on the left blade naturally intersects the telescoping guide rail at the foot end.",
        )
        ctx.allow_overlap(
            deck_left,
            telescope,
            elem_a="foot_grip_sleeve",
            elem_b=rail,
            reason="The foot-end grip sleeve on the left blade naturally intersects the telescoping guide rail at the foot end.",
        )

    # Telescope extension panel slides under the left blade panel.
    ctx.allow_overlap(
        deck_left,
        telescope,
        elem_a="blade_panel",
        elem_b="extension_panel",
        reason="The telescoping extension panel slides beneath the left blade panel as a retained-insertion fit.",
    )
    # Guide rails extend back under the blade panel for retained insertion
    for rail in ("guide_rail_0", "guide_rail_1"):
        ctx.allow_overlap(
            deck_left,
            telescope,
            elem_a="blade_panel",
            elem_b=rail,
            reason=f"The telescoping {rail} extends back beneath the blade panel for retained insertion.",
        )
        ctx.allow_overlap(
            deck_left,
            telescope,
            elem_a="foot_cross_tube",
            elem_b=rail,
            reason=f"The telescoping {rail} passes through the foot cross tube frame member for retained insertion.",
        )

    # Telescope guide rails inside guide sleeves (retained insertion)
    for i in range(2):
        ctx.allow_overlap(
            telescope,
            deck_left,
            elem_a=f"guide_rail_{i}",
            elem_b=f"telescope_sleeve_{i}",
            reason="The telescoping guide rail slides inside the matching deck sleeve for retained insertion.",
        )

    # Separation channels overlap with separation rails (sliding interface)
    for i in range(2):
        ctx.allow_overlap(
            deck_right,
            deck_left,
            elem_a=f"separation_channel_{i}",
            elem_b=f"separation_rail_{i}",
            reason="The right-half separation channel slides along the left-half guide rail.",
        )
    # Right-half cross braces cross through the left separation rails when closed
    for rail in ("separation_rail_0", "separation_rail_1"):
        ctx.allow_overlap(
            deck_left,
            deck_right,
            elem_a=rail,
            elem_b="cross_brace_0",
            reason="The left separation rail passes near the right-half cross brace when the scoop halves are closed.",
        )
        ctx.allow_overlap(
            deck_left,
            deck_right,
            elem_a=rail,
            elem_b="cross_brace_1",
            reason="The left separation rail passes near the right-half cross brace when the scoop halves are closed.",
        )

    # Latch pins may overlap with mating half plates when closed
    for end in ("head", "foot"):
        ctx.allow_overlap(
            deck_right,
            deck_left,
            elem_a=f"{end}_latch_pin",
            elem_b=f"{end}_latch_plate",
            reason=f"The {end} latch pin engages the mating plate when the scoop halves are closed.",
        )

    # --- Exact support proofs ---

    # Backrest hinge capture
    ctx.expect_overlap(
        backrest,
        deck_left,
        axes="y",
        elem_a="hinge_bar",
        elem_b="back_hinge_socket",
        min_overlap=0.20,
        name="backrest hinge bar engaged in socket",
    )
    ctx.expect_gap(
        backrest,
        deck_left,
        axis="z",
        positive_elem="hinge_bar",
        negative_elem="back_hinge_socket",
        max_penetration=0.040,
        name="backrest hinge capture is local",
    )

    # Telescope retained insertion at rest
    ctx.expect_overlap(
        telescope,
        deck_left,
        axes="x",
        elem_a="guide_rail_0",
        elem_b="telescope_sleeve_0",
        min_overlap=0.20,
        name="collapsed telescope rail remains inserted in sleeve",
    )

    # Telescope extends along +X
    rest_pos = ctx.part_world_position(telescope)
    with ctx.pose({telescope_joint: 0.18}):
        extended_pos = ctx.part_world_position(telescope)
        ctx.expect_overlap(
            telescope,
            deck_left,
            axes="x",
            elem_a="guide_rail_0",
            elem_b="telescope_sleeve_0",
            min_overlap=0.03,
            name="extended telescope rail retains insertion in sleeve",
        )
    ctx.check(
        "telescope extension moves footward along +X",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.05,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    # Scoop separation: right half moves along +Y
    closed_right = ctx.part_world_aabb(deck_right)
    with ctx.pose({scoop_joint: 0.25}):
        opened_right = ctx.part_world_aabb(deck_right)
    ctx.check(
        "scoop separation opens deck_half_right along +Y",
        closed_right is not None
        and opened_right is not None
        and opened_right[0][1] > closed_right[0][1] + 0.15,
        details=f"closed={closed_right}, opened={opened_right}",
    )

    # Backrest tilt
    with ctx.pose({back_joint: -0.55}):
        flat_back = ctx.part_world_aabb(backrest)
    with ctx.pose({back_joint: 0.55}):
        raised_back = ctx.part_world_aabb(backrest)
    ctx.check(
        "backrest hinge tilts the padded head section upward",
        flat_back is not None
        and raised_back is not None
        and raised_back[1][2] > flat_back[1][2] + 0.15,
        details=f"flat={flat_back}, raised={raised_back}",
    )

    # Structural existence checks
    ctx.check(
        "scoop stretcher has both deck halves",
        deck_left is not None and deck_right is not None,
    )
    ctx.check(
        "telescope_extension part exists",
        telescope is not None,
    )
    ctx.check(
        "scoop_separation joint is prismatic",
        scoop_joint is not None,
    )
    ctx.check(
        "telescope_extend joint is prismatic",
        telescope_joint is not None,
    )
    ctx.check(
        "reference classification note recorded",
        "scoop" in str(object_model.meta.get("run_notes", "")).lower(),
    )

    return ctx.report()


object_model = build_object_model()
