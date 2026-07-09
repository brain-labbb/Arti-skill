from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------
BLACK_METAL = "slightly_gloss_black_powder_coated_steel"
CHROME = "brushed_chrome_rail"
RUBBER = "matte_black_rubber"
DARK_HUB = "dark_grey_hub_hardware"

# ---------------------------------------------------------------------------
# Rounder dimensions (real-world meters)
# ---------------------------------------------------------------------------
RAIL_RADIUS = 0.35        # circular hanging-rail radius
RAIL_TUBE_R = 0.021       # tube wall radius of the hoop (matches hook design)
BASE_ARM_LENGTH = 0.40    # spider-base arm length from center
MAST_TUBE_R = 0.018       # central mast outer tube radius
NUM_HANGERS = 10          # hangers evenly around the hoop
NUM_ARMS = 4              # spider-base arms / spokes / casters
HANGER_ANGLE_OFFSET = math.pi / (2.0 * NUM_HANGERS)  # offset to avoid spokes


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def _hollow_z_tube(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    """Open-ended vertical annular tube (meters)."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
    )


def _hollow_y_tube(outer_radius: float, inner_radius: float, width: float) -> cq.Workplane:
    """Open-ended wheel/tire blank with spin axis along local Y."""
    return (
        cq.Workplane("XZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(width)
        .translate((0.0, -width / 2.0, 0.0))
    )


def _add_cylinder_z(part, name: str, length: float, radius: float,
                    xyz, material: Material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _add_horizontal_radial(part, name: str, length: float, radius: float,
                           start_xyz, angle: float, material: Material) -> None:
    """Horizontal cylinder radiating outward from *start_xyz* at *angle* in XY."""
    sx, sy, sz = start_xyz
    mid_x = sx + (length / 2.0) * math.cos(angle)
    mid_y = sy + (length / 2.0) * math.sin(angle)
    # Rz(angle) * Ry(pi/2) maps local Z to (cos(angle), sin(angle), 0)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(mid_x, mid_y, sz), rpy=(0.0, math.pi / 2.0, angle)),
        material=material,
        name=name,
    )


def _add_wheel_visuals(part, tire_mat: Material, hub_mat: Material,
                       tire_name: str) -> None:
    # Tire is offset along local Y (spin axis) to sit between the fork plates,
    # matching the original caster geometry from the parent baseline.
    part.visual(
        mesh_from_cadquery(
            _hollow_y_tube(0.043, 0.025, 0.032),
            tire_name,
            tolerance=0.0008,
            angular_tolerance=0.08,
        ),
        origin=Origin(xyz=(0.0, 0.032, 0.0)),
        material=tire_mat,
        name="rubber_tire",
    )
    part.visual(
        Cylinder(radius=0.026, length=0.040),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_mat,
        name="hub_disc",
    )


def _create_hanger(
    model: ArticulatedObject,
    parent,
    prefix: str,
    x: float,
    y: float,
    rail_z: float,
    hook_material: Material,
    body_material: Material,
    *,
    width: float = 0.17,
    swing_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    """Build a single garment hanger and attach it with a revolute swing joint.

    *swing_rpy* rotates the articulation frame so the hook faces radially on
    the rounder hoop and the swing axis maps to the local tangent direction.
    """
    hanger = model.part(prefix)

    # --- wire hook (local YZ plane) ----------------------------------
    hook_path = [
        (0.0, -0.018, 0.018),
        (0.0, -0.010, 0.040),
        (0.0, 0.016, 0.036),
        (0.0, 0.028, 0.014),
        (0.0, 0.018, -0.012),
        (0.0, 0.003, -0.032),
    ]
    hook = tube_from_spline_points(
        hook_path,
        radius=0.0032,
        samples_per_segment=8,
        radial_segments=12,
        cap_ends=True,
    )
    hanger.visual(
        mesh_from_geometry(hook, f"{prefix}_hook_mesh"),
        material=hook_material,
        name="hook_loop",
    )

    # --- vertical neck connecting hook to triangle body ---------------
    _add_cylinder_z(hanger, "vertical_neck", 0.064, 0.0032,
                    (0.0, 0.002, -0.054), hook_material)

    # --- triangular hanger body (local YZ plane) ----------------------
    body_path = [
        (0.0, 0.0, -0.074),
        (0.0, -width / 2.0, -0.225),
        (0.0, width / 2.0, -0.225),
        (0.0, 0.0, -0.074),
    ]
    body = tube_from_spline_points(
        body_path,
        radius=0.0055,
        samples_per_segment=6,
        radial_segments=12,
        cap_ends=True,
    )
    hanger.visual(
        mesh_from_geometry(body, f"{prefix}_body_mesh"),
        material=body_material,
        name="hanger_body",
    )

    # --- revolute swing joint ----------------------------------------
    model.articulation(
        f"{prefix}_swing",
        ArticulationType.REVOLUTE,
        parent=parent,
        child=hanger,
        origin=Origin(xyz=(x, y, rail_z), rpy=swing_rpy),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0, velocity=1.2, lower=-0.35, upper=0.35,
        ),
    )


# ---------------------------------------------------------------------------
# Object model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="retail_clothing_rounder",
        meta={
            "category": "Retail_Shop Fixtures / Clothing rack",
            "reference_note": (
                "Rounder-style garment display: closed circular hanging rail "
                "on a telescoping central mast over a four-arm wheeled spider base."
            ),
        },
    )

    black = model.material(BLACK_METAL, rgba=(0.005, 0.006, 0.006, 1.0))
    chrome = model.material(CHROME, rgba=(0.72, 0.72, 0.70, 1.0))
    rubber = model.material(RUBBER, rgba=(0.006, 0.006, 0.006, 1.0))
    hub = model.material(DARK_HUB, rgba=(0.09, 0.09, 0.085, 1.0))
    hanger_hook = model.material(
        "brushed_silver_hanger_hooks", rgba=(0.78, 0.74, 0.66, 1.0),
    )
    pale_wood = model.material(
        "pale_wood_hangers", rgba=(0.74, 0.54, 0.34, 1.0),
    )

    # ==================================================================
    # BASE FRAME – spider base with 4 radial arms and casters
    # ==================================================================
    base_frame = model.part("base_frame")
    base_z = 0.105
    tube_r = 0.022
    arm_angles = [i * math.pi / 2.0 for i in range(NUM_ARMS)]

    # Central welded hub
    _add_cylinder_z(base_frame, "central_hub", 0.060, 0.035,
                    (0.0, 0.0, base_z), black)

    # Radial arms
    for i, angle in enumerate(arm_angles):
        _add_horizontal_radial(
            base_frame, f"base_arm_{i}", BASE_ARM_LENGTH, tube_r,
            (0.0, 0.0, base_z), angle, black,
        )
        # Welded tip sphere
        tip_x = BASE_ARM_LENGTH * math.cos(angle)
        tip_y = BASE_ARM_LENGTH * math.sin(angle)
        base_frame.visual(
            Sphere(radius=tube_r),
            origin=Origin(xyz=(tip_x, tip_y, base_z)),
            material=black,
            name=f"arm_tip_{i}",
        )

    # Open telescoping sleeve (single central)
    lower_sleeve_mesh = mesh_from_cadquery(
        _hollow_z_tube(0.0235, 0.0175, 0.94),
        "lower_telescoping_sleeve",
        tolerance=0.0008,
        angular_tolerance=0.08,
    )
    clamp_collar_mesh = mesh_from_cadquery(
        _hollow_z_tube(0.027, 0.018, 0.036),
        "split_clamp_collar",
        tolerance=0.0008,
        angular_tolerance=0.08,
    )
    base_frame.visual(
        lower_sleeve_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=black,
        name="lower_sleeve_0",
    )
    base_frame.visual(
        clamp_collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, 1.004)),
        material=black,
        name="clamp_collar_0",
    )
    # Spring-button detent for height adjustment
    base_frame.visual(
        Cylinder(radius=0.005, length=0.014),
        origin=Origin(xyz=(0.0, -0.027, 0.72), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub,
        name="spring_button_0",
    )

    # Caster assemblies at arm tips
    caster_positions: list[tuple[float, float, float, float]] = []
    for i, angle in enumerate(arm_angles):
        cx = (BASE_ARM_LENGTH - 0.04) * math.cos(angle)
        cy = (BASE_ARM_LENGTH - 0.04) * math.sin(angle)
        cz = 0.043
        caster_positions.append((cx, cy, cz, angle))

        # Vertical stem into the arm
        _add_cylinder_z(base_frame, f"caster_stem_{i}", 0.060, 0.007,
                        (cx, cy, 0.122), black)
        # Horizontal bridge plate (rotated with arm)
        base_frame.visual(
            Box((0.030, 0.064, 0.008)),
            origin=Origin(xyz=(cx, cy, 0.094), rpy=(0.0, 0.0, angle)),
            material=black,
            name=f"caster_bridge_{i}",
        )
        # Axle pin along tangent direction
        base_frame.visual(
            Cylinder(radius=0.006, length=0.070),
            origin=Origin(
                xyz=(cx, cy, cz),
                rpy=(0.0, math.pi / 2.0, angle + math.pi / 2.0),
            ),
            material=hub,
            name=f"caster_cross_pin_{i}",
        )
        # Fork side plates straddling the wheel along tangent
        tan_x = -math.sin(angle)
        tan_y = math.cos(angle)
        for side, offset in enumerate((-0.026, 0.026)):
            fx = cx + offset * tan_x
            fy = cy + offset * tan_y
            base_frame.visual(
                Box((0.030, 0.006, 0.078)),
                origin=Origin(xyz=(fx, fy, 0.059), rpy=(0.0, 0.0, angle)),
                material=black,
                name=f"caster_fork_{i}_{side}",
            )

    # ==================================================================
    # UPPER FRAME – central mast + circular round rail
    # ==================================================================
    upper_frame = model.part("upper_frame")
    rail_z = 0.66  # rail height relative to upper-frame origin

    # Central upper post (inserted into lower sleeve)
    _add_cylinder_z(upper_frame, "upper_post_0", 0.84, 0.014,
                    (0.0, 0.0, 0.24), chrome)
    _add_cylinder_z(upper_frame, "sliding_bushing_0", 0.080, 0.01755,
                    (0.0, 0.0, -0.10), hub)

    # Mast cap / spoke junction
    upper_frame.visual(
        Cylinder(radius=0.025, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, rail_z)),
        material=black,
        name="mast_cap",
    )

    # Radial spoke arms from mast to hoop (at rail height)
    spoke_length = RAIL_RADIUS  # reaches rail centre-line
    for i, angle in enumerate(arm_angles):
        _add_horizontal_radial(
            upper_frame, f"spoke_arm_{i}", spoke_length, 0.010,
            (0.0, 0.0, rail_z), angle, black,
        )

    # Circular round-rail hoop
    num_rail_pts = 48
    rail_points = []
    for k in range(num_rail_pts):
        a = 2.0 * math.pi * k / num_rail_pts
        rail_points.append((
            RAIL_RADIUS * math.cos(a),
            RAIL_RADIUS * math.sin(a),
            0.0,
        ))
    round_rail_mesh = mesh_from_geometry(
        tube_from_spline_points(
            rail_points,
            radius=RAIL_TUBE_R,
            samples_per_segment=3,
            closed_spline=True,
            radial_segments=16,
            cap_ends=False,
        ),
        "round_rail_hoop",
    )
    upper_frame.visual(
        round_rail_mesh,
        origin=Origin(xyz=(0.0, 0.0, rail_z)),
        material=black,
        name="round_rail",
    )

    # ==================================================================
    # HANGERS – radially distributed around the round rail
    # ==================================================================
    for idx in range(NUM_HANGERS):
        angle = idx * 2.0 * math.pi / NUM_HANGERS + HANGER_ANGLE_OFFSET
        hx = RAIL_RADIUS * math.cos(angle)
        hy = RAIL_RADIUS * math.sin(angle)
        # Rotate the articulation frame so local X maps to the tangent
        # direction (-sin(angle), cos(angle), 0); the hook (in the local
        # YZ plane) then faces radially and the swing axis is tangent.
        swing_rpy = (0.0, 0.0, angle + math.pi / 2.0)
        _create_hanger(
            model, upper_frame, f"hanger_{idx}",
            hx, hy, rail_z,
            hanger_hook, pale_wood,
            width=0.16,
            swing_rpy=swing_rpy,
        )

    # ==================================================================
    # ARTICULATIONS
    # ==================================================================
    # Telescoping height slide
    model.articulation(
        "height_slide",
        ArticulationType.PRISMATIC,
        parent=base_frame,
        child=upper_frame,
        origin=Origin(xyz=(0.0, 0.0, 1.04)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0, velocity=0.18, lower=0.0, upper=0.14,
        ),
    )

    # Caster wheels
    for i, (cx, cy, cz, angle) in enumerate(caster_positions):
        wheel = model.part(f"wheel_{i}")
        _add_wheel_visuals(wheel, rubber, hub, f"caster_tire_{i}")
        model.articulation(
            f"wheel_spin_{i}",
            ArticulationType.CONTINUOUS,
            parent=base_frame,
            child=wheel,
            # Rotate wheel frame so local Y → tangent = wheel spin axis
            origin=Origin(xyz=(cx, cy, cz), rpy=(0.0, 0.0, angle)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=20.0),
        )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_frame")
    upper = object_model.get_part("upper_frame")
    height = object_model.get_articulation("height_slide")

    # --- wheels -------------------------------------------------------
    ctx.check(
        "rack has four caster wheels",
        all(object_model.get_part(f"wheel_{i}") is not None for i in range(4)),
    )
    ctx.check(
        "rack has wheel spin joints",
        all(
            object_model.get_articulation(f"wheel_spin_{i}") is not None
            for i in range(4)
        ),
    )
    for i in range(4):
        wheel = object_model.get_part(f"wheel_{i}")
        ctx.allow_overlap(
            base, wheel,
            elem_a=f"caster_cross_pin_{i}",
            elem_b="hub_disc",
            reason="The fixed caster axle is intentionally captured through the rotating wheel hub.",
        )
        ctx.expect_contact(
            base, wheel,
            elem_a=f"caster_cross_pin_{i}",
            elem_b="hub_disc",
            contact_tol=0.012,
            name=f"caster axle {i} passes through wheel hub",
        )

    # --- telescoping mast insertion -----------------------------------
    ctx.allow_overlap(
        base, upper,
        elem_a="lower_sleeve_0",
        elem_b="sliding_bushing_0",
        reason="The hidden plastic guide bushing is intentionally captured in the telescoping sleeve for a snug sliding fit.",
    )
    ctx.expect_within(
        upper, base, axes="xy",
        inner_elem="sliding_bushing_0",
        outer_elem="lower_sleeve_0",
        margin=0.002,
        name="sliding bushing 0 is captured inside sleeve",
    )
    ctx.expect_overlap(
        upper, base, axes="z",
        elem_a="sliding_bushing_0",
        elem_b="lower_sleeve_0",
        min_overlap=0.070,
        name="sliding bushing 0 remains inserted",
    )
    ctx.expect_within(
        upper, base, axes="xy",
        inner_elem="upper_post_0",
        outer_elem="lower_sleeve_0",
        margin=0.001,
        name="upper post 0 centered in lower sleeve",
    )
    ctx.expect_overlap(
        upper, base, axes="z",
        elem_a="upper_post_0",
        elem_b="lower_sleeve_0",
        min_overlap=0.16,
        name="upper post 0 has collapsed insertion",
    )

    # --- height slide raises round rail --------------------------------
    rest_aabb = ctx.part_world_aabb(upper)
    with ctx.pose({height: 0.14}):
        raised_aabb = ctx.part_world_aabb(upper)
        ctx.expect_within(
            upper, base, axes="xy",
            inner_elem="upper_post_0",
            outer_elem="lower_sleeve_0",
            margin=0.001,
            name="raised upper post 0 remains centered",
        )
        ctx.expect_overlap(
            upper, base, axes="z",
            elem_a="upper_post_0",
            elem_b="lower_sleeve_0",
            min_overlap=0.025,
            name="raised upper post 0 retains insertion",
        )

    ctx.check(
        "height slide raises round rail",
        rest_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > rest_aabb[1][2] + 0.10,
        details=f"rest_aabb={rest_aabb}, raised_aabb={raised_aabb}",
    )

    # --- round rail geometry -------------------------------------------
    ctx.expect_gap(
        upper, base,
        axis="z",
        positive_elem="round_rail",
        negative_elem="central_hub",
        min_gap=1.40,
        name="round rail is well above base",
    )

    # --- radial hangers ------------------------------------------------
    ctx.check(
        "round rail has radial hangers",
        all(
            object_model.get_part(f"hanger_{idx}") is not None
            and object_model.get_articulation(f"hanger_{idx}_swing") is not None
            for idx in range(NUM_HANGERS)
        ),
    )

    # Each hanger hook wraps around the round rail (intentional overlap)
    for idx in range(NUM_HANGERS):
        hanger_part = object_model.get_part(f"hanger_{idx}")
        ctx.allow_overlap(
            upper, hanger_part,
            elem_a="round_rail",
            elem_b="hook_loop",
            reason="The hanger hook intentionally wraps around the circular hanging rail.",
        )
        ctx.expect_overlap(
            upper, hanger_part,
            axes="z",
            elem_a="round_rail",
            elem_b="hook_loop",
            min_overlap=0.010,
            name=f"hanger_{idx} hook wraps the round rail",
        )

    hanger_0 = object_model.get_part("hanger_0")
    swing_0 = object_model.get_articulation("hanger_0_swing")
    ctx.check(
        "hanger_0 swing is revolute",
        swing_0 is not None
        and swing_0.articulation_type == ArticulationType.REVOLUTE
        and swing_0.motion_limits is not None
        and swing_0.motion_limits.lower == -0.35
        and swing_0.motion_limits.upper == 0.35,
    )
    ctx.expect_gap(
        upper, hanger_0,
        axis="z",
        positive_elem="round_rail",
        negative_elem="hanger_body",
        min_gap=0.040,
        name="hanger body hangs below round rail",
    )

    return ctx.report()


object_model = build_object_model()
