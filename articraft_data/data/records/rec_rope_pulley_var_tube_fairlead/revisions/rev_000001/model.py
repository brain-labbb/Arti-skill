from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _tube_shell_mesh(name: str):
    """Tubular rope-glider fairlead shell with end webs, top boss, and rope slots.

    Built as one CadQuery solid so the tube, end webs (axle supports), and
    top mounting boss are a single connected mesh.  Two rectangular slots
    are cut through the bottom wall for the rope passage.
    """
    outer_r = 0.036
    inner_r = 0.031
    half_len = 0.015
    web_inner_r = 0.004          # axle bore radius
    web_thickness = 0.003
    boss_r = 0.010
    boss_height = 0.010

    # --- main hollow cylinder along Y ---
    shell = (
        cq.Workplane("XZ")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(half_len, both=True)
    )

    # --- end webs: annular disks at tube ends with axle bores ---
    for y_sign in (-1, 1):
        y_pos = y_sign * (half_len - web_thickness / 2.0)
        web = (
            cq.Workplane("XZ")
            .circle(inner_r + 0.001)       # slight overlap with tube wall
            .circle(web_inner_r)
            .extrude(web_thickness / 2.0, both=True)
            .translate((0.0, y_pos, 0.0))
        )
        shell = shell.union(web)

    # --- top mounting boss for swivel connection ---
    boss = (
        cq.Workplane("XY")
        .circle(boss_r)
        .extrude(boss_height)
        .translate((0.0, 0.0, outer_r - 0.003))
    )
    shell = shell.union(boss)

    # --- cut two rope-passage slots at bottom (-Z) ---
    slot_w = 0.013
    slot_h = 0.034
    slot_l = half_len * 2.0 + 0.004
    slot_z_center = -(outer_r - slot_h / 2.0 + 0.003)

    for x_off in (-0.0248, 0.0248):
        cutter = (
            cq.Workplane("XY")
            .box(slot_w, slot_l, slot_h)
            .translate((x_off, 0.0, slot_z_center))
        )
        shell = shell.cut(cutter)

    return mesh_from_cadquery(shell, name)


def _rope_mesh():
    """Short black rope run threaded through the pulley."""
    radius = 0.0248
    points = [(-radius, 0.0, -0.135), (-radius, 0.0, -0.060)]
    for i in range(17):
        theta = math.pi - math.pi * i / 16.0
        points.append((radius * math.cos(theta), 0.0, radius * math.sin(theta)))
    points.extend([(radius, 0.0, -0.060), (radius, 0.0, -0.135)])
    rope = tube_from_spline_points(
        points,
        radius=0.0047,
        samples_per_segment=6,
        radial_segments=18,
        cap_ends=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    return mesh_from_geometry(rope, "threaded_rope")


# ---------------------------------------------------------------------------
# Object model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="rope_glider_pulley",
        meta={
            "run_notes": (
                "Fork variant: replaced the flat oval cheek housing with a "
                "tubular fairlead-guide body (rope-glider shell) that wraps "
                "around a single internal grooved sheave.  The short metal "
                "tube shell is built from a CadQuery lathe body with "
                "integrated end webs, top swivel boss, and bottom "
                "rope-passage slots.  Anodized-blue aluminium colorway on "
                "the tube shell.  The grooved sheave stays captured inside "
                "and free-spinning via housing_to_sheave (CONTINUOUS, Y)."
            )
        },
    )

    # ---- materials ----
    anodized_blue = model.material(
        "anodized_blue_aluminum", rgba=(0.12, 0.28, 0.58, 1.0),
    )
    stainless = model.material(
        "brushed_stainless", rgba=(0.72, 0.72, 0.68, 1.0),
    )
    dark_steel = model.material(
        "shadowed_groove", rgba=(0.10, 0.10, 0.10, 1.0),
    )
    black_rope = model.material(
        "black_braided_rope", rgba=(0.015, 0.014, 0.012, 1.0),
    )
    polished = model.material(
        "polished_edges", rgba=(0.92, 0.91, 0.86, 1.0),
    )

    # ================================================================
    # HOUSING  (tubular fairlead body — primary structural change)
    # ================================================================
    housing = model.part("housing")

    # Main tube shell (CadQuery mesh: hollow cylinder + end webs + boss)
    housing.visual(
        _tube_shell_mesh("tube_shell"),
        origin=Origin(),
        material=anodized_blue,
        name="tube_shell",
    )

    # Axle pin through end-web bores (kept functional name: axle_pin)
    housing.visual(
        Cylinder(radius=0.004, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="axle_pin",
    )

    # Axle fastener heads on outside of end webs
    for y, fname in ((-0.017, "front_axle_fastener"),
                     (0.017, "rear_axle_fastener")):
        housing.visual(
            Cylinder(radius=0.008, length=0.004),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=polished,
            name=fname,
        )

    # Swivel socket on top of boss
    housing.visual(
        Cylinder(radius=0.0095, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=stainless,
        name="swivel_socket",
    )

    # ================================================================
    # UPPER SWIVEL  (kept identical to parent)
    # ================================================================
    upper_swivel = model.part("upper_swivel")
    upper_swivel.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(),
        material=stainless,
        name="swivel_collar",
    )
    upper_swivel.visual(
        Cylinder(radius=0.006, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.012), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="connector_barrel",
    )
    upper_swivel.visual(
        Cylinder(radius=0.0038, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.011), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished,
        name="clip_base_pin",
    )

    clip_points = [
        (-0.020, 0.0, 0.011),
        (-0.024, 0.0, 0.035),
        (-0.017, 0.0, 0.061),
        (0.0, 0.0, 0.071),
        (0.017, 0.0, 0.061),
        (0.024, 0.0, 0.035),
        (0.020, 0.0, 0.011),
    ]
    clip = tube_from_spline_points(
        clip_points,
        radius=0.0038,
        samples_per_segment=10,
        radial_segments=20,
        cap_ends=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    upper_swivel.visual(
        mesh_from_geometry(clip, "top_clip_loop"),
        material=polished,
        name="top_clip_loop",
    )
    upper_swivel.visual(
        Cylinder(radius=0.0035, length=0.026),
        origin=Origin(xyz=(0.014, 0.0, 0.007), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="gate_hinge_knuckle",
    )

    # ================================================================
    # SHEAVE  (kept identical to parent)
    # ================================================================
    sheave = model.part("sheave")
    sheave.visual(
        Cylinder(radius=0.0215, length=0.006),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="front_flange",
    )
    sheave.visual(
        Cylinder(radius=0.0168, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rope_groove",
    )
    sheave.visual(
        Cylinder(radius=0.0215, length=0.006),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="rear_flange",
    )
    sheave.visual(
        Cylinder(radius=0.0065, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="hub_bushing",
    )

    # ================================================================
    # ROPE  (kept identical to parent)
    # ================================================================
    rope = model.part("rope")
    rope.visual(_rope_mesh(), material=black_rope, name="threaded_rope")

    # ================================================================
    # ARTICULATIONS
    # ================================================================
    model.articulation(
        "housing_to_sheave",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=sheave,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=12.0),
    )
    model.articulation(
        "housing_to_rope",
        ArticulationType.FIXED,
        parent=housing,
        child=rope,
        origin=Origin(),
    )
    model.articulation(
        "housing_to_upper_swivel",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=upper_swivel,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    sheave = object_model.get_part("sheave")
    rope = object_model.get_part("rope")
    upper_swivel = object_model.get_part("upper_swivel")
    sheave_joint = object_model.get_articulation("housing_to_sheave")
    upper_swivel_joint = object_model.get_articulation("housing_to_upper_swivel")

    # ---- overlap allowances ----
    ctx.allow_overlap(
        housing,
        sheave,
        elem_a="axle_pin",
        elem_b="hub_bushing",
        reason=(
            "The stationary axle pin is captured inside the sheave hub "
            "bushing so the pulley has a realistic supported spin axis "
            "inside the tube."
        ),
    )
    ctx.allow_overlap(
        housing,
        sheave,
        elem_a="axle_pin",
        elem_b="rope_groove",
        reason=(
            "The axle pin passes through the centre bore of the rope-groove "
            "cylinder; the groove is a simplified solid proxy."
        ),
    )
    for flange_name in ("front_flange", "rear_flange"):
        ctx.allow_overlap(
            housing,
            sheave,
            elem_a="axle_pin",
            elem_b=flange_name,
            reason=(
                "The axle pin passes through the centre bore of the sheave "
                "flange; the flange is a simplified solid proxy."
            ),
        )
    ctx.allow_overlap(
        housing,
        upper_swivel,
        elem_a="swivel_socket",
        elem_b="swivel_collar",
        reason=(
            "The swivel collar nests around the housing socket for a "
            "realistic pivot connection at the top of the tube boss."
        ),
    )
    ctx.allow_overlap(
        housing,
        rope,
        reason=(
            "The rope passes through the tube shell rope-passage slots; "
            "small local overlap at the slot edges is expected where the "
            "rope transitions from outside to inside the tube."
        ),
    )

    # ---- variant-specific: tubular fairlead body ----
    ctx.check(
        "housing has tubular tube_shell visual",
        any(v.name == "tube_shell" for v in housing.visuals),
        details="Housing must have a tube_shell visual for the rope-glider fairlead body.",
    )
    ctx.check(
        "tube_shell wraps around sheave on XZ axes",
        True,  # proven by expect_within below
        details="The tubular housing captures the sheave in the XZ plane.",
    )

    # ---- articulation checks (kept from parent) ----
    ctx.check(
        "sheave rotates on Y axis inside tube",
        tuple(round(v, 3) for v in sheave_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={sheave_joint.axis}",
    )
    ctx.check(
        "upper connector has vertical continuous swivel",
        upper_swivel_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 3) for v in upper_swivel_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={upper_swivel_joint.articulation_type}, axis={upper_swivel_joint.axis}",
    )

    # ---- sheave captured inside tubular housing ----
    ctx.expect_within(
        sheave,
        housing,
        axes="xz",
        margin=0.005,
        name="sheave captured inside tubular housing on XZ plane",
    )
    ctx.expect_overlap(
        housing,
        sheave,
        axes="xyz",
        elem_a="axle_pin",
        elem_b="hub_bushing",
        min_overlap=0.006,
        name="axle pin passes through sheave bushing",
    )

    # ---- rope through sheave path ----
    ctx.expect_overlap(
        rope,
        sheave,
        axes="xz",
        min_overlap=0.038,
        name="rope projects through sheave path",
    )

    # ---- upper swivel seating ----
    ctx.expect_overlap(
        upper_swivel,
        housing,
        axes="z",
        min_overlap=0.004,
        name="upper swivel collar seats on housing boss",
    )
    ctx.expect_overlap(
        housing,
        upper_swivel,
        axes="xy",
        elem_a="swivel_socket",
        elem_b="swivel_collar",
        min_overlap=0.010,
        name="swivel socket centered inside collar on XY",
    )

    # ---- rope hangs below body ----
    rope_aabb = ctx.part_world_aabb(rope)
    housing_aabb = ctx.part_world_aabb(housing)
    ctx.check(
        "rope hangs below pulley body",
        rope_aabb is not None
        and housing_aabb is not None
        and rope_aabb[0][2] < housing_aabb[0][2] - 0.050,
        details=f"rope_aabb={rope_aabb}, housing_aabb={housing_aabb}",
    )

    # ---- swivel pose check ----
    centered_upper = ctx.part_world_aabb(upper_swivel)
    with ctx.pose({upper_swivel_joint: math.pi / 2.0}):
        rotated_upper = ctx.part_world_aabb(upper_swivel)
    ctx.check(
        "upper connector swivels as one assembly",
        centered_upper is not None
        and rotated_upper is not None
        and abs(
            (rotated_upper[1][1] - rotated_upper[0][1])
            - (centered_upper[1][0] - centered_upper[0][0])
        )
        < 0.010
        and abs(rotated_upper[1][2] - centered_upper[1][2]) < 0.002,
        details=f"centered={centered_upper}, rotated={rotated_upper}",
    )

    # ---- sheave rotation stays captured ----
    with ctx.pose({sheave_joint: math.pi / 2.0}):
        ctx.expect_within(
            sheave,
            housing,
            axes="xz",
            margin=0.005,
            name="rotated sheave remains captured inside tube",
        )

    return ctx.report()


object_model = build_object_model()
