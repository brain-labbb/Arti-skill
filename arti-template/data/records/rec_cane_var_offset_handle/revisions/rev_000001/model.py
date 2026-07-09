from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_side_loft,
    tube_from_spline_points,
)


LOWER_TUBE_TOP = 0.590
COLLAR_BOTTOM = 0.575
COLLAR_TOP = 0.640
SLIDE_HOME_Z = 0.540
SLIDE_TRAVEL = 0.100
INNER_TUBE_BOTTOM_LOCAL = -0.220
INNER_TUBE_TOP_LOCAL = 0.300


def _hollow_tube_mesh(
    outer_radius: float,
    inner_radius: float,
    z_min: float,
    z_max: float,
    *,
    name: str,
):
    """Revolved thin-wall tube with real central clearance."""
    shell = LatheGeometry.from_shell_profiles(
        [(outer_radius, z_min), (outer_radius, z_max)],
        [(inner_radius, z_min), (inner_radius, z_max)],
        segments=48,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )
    return mesh_from_geometry(shell, name)


def _ferrule_mesh():
    # A compact, flared rubber ferrule: narrow stem, beveled waist, and a
    # slightly broader contact foot without forming a multi-leg base.
    ferrule = LatheGeometry(
        [
            (0.000, 0.000),
            (0.022, 0.000),
            (0.026, 0.006),
            (0.024, 0.014),
            (0.018, 0.026),
            (0.016, 0.050),
            (0.010, 0.056),
            (0.000, 0.056),
        ],
        segments=56,
        closed=True,
    )
    return mesh_from_geometry(ferrule, "rubber_ferrule")


def _swan_neck_mesh():
    """S-curved offset neck tube: rises from shaft then curves forward (+X)
    so the user's weight loads over the ferrule tip."""
    neck = tube_from_spline_points(
        [
            (0.000, 0.000, 0.012),
            (0.000, 0.000, 0.032),
            (0.008, 0.000, 0.048),
            (0.022, 0.000, 0.058),
            (0.036, 0.000, 0.063),
            (0.044, 0.000, 0.064),
        ],
        radius=0.0110,
        samples_per_segment=18,
        radial_segments=20,
        cap_ends=True,
    )
    return mesh_from_geometry(neck, "swan_neck")


def _offset_grip_mesh():
    """Contoured foam grip for the offset handle.
    The loft axis is Y so the grip spans side-to-side; profiles in XZ
    give a palm-swelled cross section with a dipped underside."""
    grip = superellipse_side_loft(
        [
            (-0.054, 0.000, 0.030, 0.034),
            (-0.034, -0.001, 0.033, 0.040),
            (-0.012, -0.003, 0.035, 0.044),
            (0.000, -0.004, 0.036, 0.046),
            (0.012, -0.003, 0.035, 0.044),
            (0.034, -0.001, 0.033, 0.040),
            (0.054, 0.000, 0.030, 0.034),
        ],
        exponents=3.0,
        segments=56,
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(grip, "foam_grip")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_point_telescoping_cane")

    chrome = model.material("brushed_chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_chrome = model.material("shadow_chrome", rgba=(0.42, 0.44, 0.46, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.012, 0.012, 0.014, 1.0))
    molded_black = model.material("molded_black", rgba=(0.020, 0.020, 0.024, 1.0))
    hole_black = model.material("black_hole", rgba=(0.0, 0.0, 0.0, 1.0))

    lower_shaft = model.part("lower_shaft")
    lower_shaft.visual(
        _ferrule_mesh(),
        material=black_rubber,
        name="ferrule",
    )
    lower_shaft.visual(
        _hollow_tube_mesh(0.0120, 0.0097, 0.050, LOWER_TUBE_TOP, name="lower_tube"),
        material=chrome,
        name="lower_tube",
    )
    lower_shaft.visual(
        _hollow_tube_mesh(0.0170, 0.0100, COLLAR_BOTTOM, COLLAR_TOP, name="collar_shell"),
        material=chrome,
        name="collar_shell",
    )
    lower_shaft.visual(
        _hollow_tube_mesh(
            0.0182,
            0.0101,
            COLLAR_BOTTOM + 0.001,
            COLLAR_BOTTOM + 0.007,
            name="lower_collar_lip",
        ),
        material=dark_chrome,
        name="lower_collar_lip",
    )
    lower_shaft.visual(
        _hollow_tube_mesh(
            0.0182,
            0.0101,
            COLLAR_TOP - 0.007,
            COLLAR_TOP - 0.001,
            name="upper_collar_lip",
        ),
        material=dark_chrome,
        name="upper_collar_lip",
    )

    # The row of dark, flush circular marks reads as real push-button indexing
    # holes along the lower telescoping sleeve.
    for index, z in enumerate((0.392, 0.432, 0.472, 0.512, 0.552, 0.612)):
        radius = 0.0038 if index < 5 else 0.0045
        x = 0.01235 if index < 5 else 0.01735
        lower_shaft.visual(
            Cylinder(radius=radius, length=0.0016),
            origin=Origin(
                xyz=(x, 0.0, z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=hole_black,
            name=f"button_hole_{index}",
        )

    lower_shaft.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, COLLAR_TOP)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_TOP / 2.0)),
    )

    upper_shaft = model.part("upper_shaft")
    upper_shaft.visual(
        Cylinder(radius=0.0085, length=INNER_TUBE_TOP_LOCAL - INNER_TUBE_BOTTOM_LOCAL),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                (INNER_TUBE_TOP_LOCAL + INNER_TUBE_BOTTOM_LOCAL) / 2.0,
            )
        ),
        material=chrome,
        name="inner_tube",
    )
    upper_shaft.visual(
        Cylinder(radius=0.0100, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, INNER_TUBE_TOP_LOCAL - 0.006)),
        material=dark_chrome,
        name="handle_socket",
    )
    upper_shaft.visual(
        Sphere(radius=0.0052),
        origin=Origin(xyz=(0.0136, 0.0, 0.072)),
        material=chrome,
        name="spring_button",
    )
    upper_shaft.inertial = Inertial.from_geometry(
        Box((0.030, 0.030, 0.540)),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
    )

    model.articulation(
        "lower_to_upper",
        ArticulationType.PRISMATIC,
        parent=lower_shaft,
        child=upper_shaft,
        origin=Origin(xyz=(0.0, 0.0, SLIDE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.12,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    foam_charcoal = model.material("foam_charcoal", rgba=(0.07, 0.07, 0.08, 1.0))

    handle = model.part("handle")
    # Socket flare: visible transition ring where handle seats into the shaft
    handle.visual(
        Cylinder(radius=0.0190, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=molded_black,
        name="socket_flare",
    )
    # S-curved swan neck: offsets the grip forward from the shaft axis
    handle.visual(
        _swan_neck_mesh(),
        material=molded_black,
        name="swan_neck",
    )
    # Contoured foam grip at the end of the offset neck
    handle.visual(
        _offset_grip_mesh(),
        origin=Origin(xyz=(0.044, 0.0, 0.064)),
        material=foam_charcoal,
        name="foam_grip",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.120, 0.110, 0.100)),
        mass=0.19,
        origin=Origin(xyz=(0.022, 0.0, 0.050)),
    )

    model.articulation(
        "upper_to_handle",
        ArticulationType.FIXED,
        parent=upper_shaft,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, INNER_TUBE_TOP_LOCAL)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower_shaft = object_model.get_part("lower_shaft")
    upper_shaft = object_model.get_part("upper_shaft")
    handle = object_model.get_part("handle")
    slide = object_model.get_articulation("lower_to_upper")

    # The spring-loaded button is intentionally shown protruding through the
    # collar indexing hole; this small local embed stands in for the wall hole.
    ctx.allow_overlap(
        lower_shaft,
        upper_shaft,
        elem_a="collar_shell",
        elem_b="spring_button",
        reason="The push button is intentionally seated through the collar's height-indexing hole.",
    )
    ctx.expect_overlap(
        lower_shaft,
        upper_shaft,
        axes="z",
        elem_a="collar_shell",
        elem_b="spring_button",
        min_overlap=0.006,
        name="button sits within collar height band",
    )

    ctx.expect_within(
        upper_shaft,
        lower_shaft,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="lower_tube",
        margin=0.001,
        name="inner tube centered in lower sleeve",
    )
    ctx.expect_overlap(
        upper_shaft,
        lower_shaft,
        axes="z",
        elem_a="inner_tube",
        elem_b="lower_tube",
        min_overlap=0.24,
        name="collapsed cane retains tube insertion",
    )
    ctx.expect_contact(
        handle,
        upper_shaft,
        elem_a="socket_flare",
        elem_b="handle_socket",
        contact_tol=0.001,
        name="handle socket seats on upper shaft",
    )

    # Swan-neck offset: the foam grip must sit forward of the shaft axis
    grip_aabb = ctx.part_element_world_aabb(handle, elem="foam_grip")
    shaft_pos = ctx.part_world_position(upper_shaft)
    ctx.check(
        "swan-neck grip offset forward from shaft",
        grip_aabb is not None
        and shaft_pos is not None
        and (grip_aabb[0][0] + grip_aabb[1][0]) / 2.0 > shaft_pos[0] + 0.020,
        details="foam_grip center X should be at least 20 mm forward of shaft axis",
    )

    rest_pos = ctx.part_world_position(handle)
    rest_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_within(
            upper_shaft,
            lower_shaft,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="lower_tube",
            margin=0.001,
            name="extended tube remains centered",
        )
        ctx.expect_overlap(
            upper_shaft,
            lower_shaft,
            axes="z",
            elem_a="inner_tube",
            elem_b="lower_tube",
            min_overlap=0.14,
            name="extended cane retains tube insertion",
        )
        extended_pos = ctx.part_world_position(handle)
        extended_aabb = ctx.part_world_aabb(handle)

    ctx.check(
        "prismatic slide raises handle",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + 0.090,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )
    rest_top = rest_aabb[1][2] if rest_aabb is not None else None
    extended_top = extended_aabb[1][2] if extended_aabb is not None else None
    ctx.check(
        "cane height is walking scale",
        rest_top is not None
        and extended_top is not None
        and 0.88 <= rest_top <= 0.95
        and 0.98 <= extended_top <= 1.06,
        details=f"rest_top={rest_top}, extended_top={extended_top}",
    )

    return ctx.report()


object_model = build_object_model()
