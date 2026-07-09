from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _tube_shell(
    *,
    outer_radius: float,
    inner_radius: float,
    z_min: float,
    z_max: float,
    name: str,
) -> object:
    wall = LatheGeometry.from_shell_profiles(
        [
            (outer_radius, z_min),
            (outer_radius, z_max),
        ],
        [
            (inner_radius, z_min + 0.002),
            (inner_radius, z_max - 0.002),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(wall, name)


def _conical_band_mesh(name: str) -> object:
    """Thin blue enamel sleeve on the outside of the steel bowl."""
    band = LatheGeometry.from_shell_profiles(
        [
            (0.085, 0.020),
            (0.118, 0.056),
            (0.145, 0.092),
        ],
        [
            (0.082, 0.022),
            (0.115, 0.058),
            (0.141, 0.090),
        ],
        segments=96,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(band, name)


def _bowl_shell_mesh(name: str) -> object:
    outer_profile = [
        (0.048, 0.000),
        (0.070, 0.006),
        (0.094, 0.022),
        (0.124, 0.066),
        (0.146, 0.104),
        (0.152, 0.112),
    ]
    inner_profile = [
        (0.030, 0.010),
        (0.064, 0.017),
        (0.087, 0.031),
        (0.115, 0.071),
        (0.139, 0.101),
    ]
    bowl = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=96,
        start_cap="round",
        end_cap="round",
        lip_samples=8,
    )
    bowl.merge(TorusGeometry(radius=0.148, tube=0.008, radial_segments=16, tubular_segments=96).translate(0, 0, 0.106))
    return mesh_from_geometry(bowl, name)


def _ribbed_ring_mesh(name: str) -> object:
    """Circular support ring with small grip ribs like the reference holder."""
    base = TorusGeometry(radius=0.150, tube=0.012, radial_segments=16, tubular_segments=96)
    base.merge(CylinderGeometry(radius=0.140, height=0.010, radial_segments=72).translate(0.0, 0.0, -0.010))
    rib = Box((0.010, 0.014, 0.026))
    rib_geom = MeshGeometry()
    for index in range(36):
        angle = index * math.tau / 36.0
        r = 0.142
        geom = BoxGeometryShim(rib).rotate_z(angle).translate(r * math.cos(angle), r * math.sin(angle), -0.014)
        rib_geom.merge(geom)
    base.merge(rib_geom)
    return mesh_from_geometry(base, name)


def BoxGeometryShim(box: Box) -> MeshGeometry:
    # Local helper keeps the main authoring path primitive-independent while
    # still allowing radial mesh ribs to be merged into a single connected ring.
    sx, sy, sz = box.size
    x, y, z = sx / 2.0, sy / 2.0, sz / 2.0
    vertices = [
        (-x, -y, -z),
        (x, -y, -z),
        (x, y, -z),
        (-x, y, -z),
        (-x, -y, z),
        (x, -y, z),
        (x, y, z),
        (-x, y, z),
    ]
    faces = [
        (0, 1, 2), (0, 2, 3),
        (4, 6, 5), (4, 7, 6),
        (0, 4, 5), (0, 5, 1),
        (1, 5, 6), (1, 6, 2),
        (2, 6, 7), (2, 7, 3),
        (3, 7, 4), (3, 4, 0),
    ]
    return MeshGeometry(vertices=list(vertices), faces=list(faces))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="raised_pet_feeding_bowl_stand")

    black_plastic = model.material("black_textured_plastic", rgba=(0.015, 0.017, 0.018, 1.0))
    soft_black = model.material("soft_black_feet", rgba=(0.025, 0.028, 0.030, 1.0))
    blue_enamel = model.material("blue_enamel", rgba=(0.02, 0.30, 0.72, 1.0))
    stainless = model.material("stainless_steel", rgba=(0.78, 0.80, 0.83, 1.0))
    white_print = model.material("white_bowl_print", rgba=(0.95, 0.97, 0.98, 1.0))

    leg_member_mesh = mesh_from_geometry(
        CapsuleGeometry(radius=0.018, length=0.220, radial_segments=16),
        "folding_leg_member",
    )
    hinge_barrel_mesh = mesh_from_geometry(
        CylinderGeometry(radius=0.010, height=0.038, radial_segments=24),
        "hinge_barrel",
    )
    base_sleeve = _tube_shell(
        outer_radius=0.047,
        inner_radius=0.036,
        z_min=0.058,
        z_max=0.420,
        name="hollow_pedestal_sleeve",
    )
    collar_mesh = mesh_from_geometry(TorusGeometry(radius=0.047, tube=0.007, radial_segments=14, tubular_segments=72), "pedestal_collar")

    base = model.part("base")
    # Hinge barrels on the hub perimeter for each folding leg.
    # Cylinder axis (local Z) is rotated to the tangent direction at each leg angle.
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        base.visual(
            hinge_barrel_mesh,
            origin=Origin(
                xyz=(0.090 * math.cos(angle), 0.090 * math.sin(angle), 0.030),
                rpy=(0.0, math.pi / 2.0, angle + math.pi / 2.0),
            ),
            material=black_plastic,
            name=f"hinge_barrel_{index}",
        )

    base.visual(
        Cylinder(radius=0.090, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=black_plastic,
        name="central_hub",
    )
    base.visual(
        base_sleeve,
        material=black_plastic,
        name="hollow_pedestal_sleeve",
    )
    for z, visual_name in ((0.062, "lower_lock_collar"), (0.250, "height_lock_collar"), (0.418, "upper_socket_lip")):
        base.visual(
            collar_mesh,
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=black_plastic,
            name=visual_name,
        )
    base.inertial = Inertial.from_geometry(Cylinder(radius=0.10, length=0.42), mass=1.2, origin=Origin(xyz=(0, 0, 0.21)))

    holder = model.part("bowl_holder")
    holder.visual(
        Cylinder(radius=0.036, length=0.340),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=black_plastic,
        name="sliding_inner_post",
    )
    holder.visual(
        Cylinder(radius=0.056, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.228)),
        material=black_plastic,
        name="top_socket",
    )
    holder.visual(
        mesh_from_geometry(TorusGeometry(radius=0.064, tube=0.010, radial_segments=14, tubular_segments=72), "top_socket_lip"),
        origin=Origin(xyz=(0.0, 0.0, 0.256)),
        material=black_plastic,
        name="top_socket_lip",
    )
    holder.visual(
        _ribbed_ring_mesh("ribbed_bowl_support_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.268)),
        material=black_plastic,
        name="ribbed_support_ring",
    )
    for index, angle in enumerate((math.radians(18), math.radians(162), math.radians(198), math.radians(342))):
        holder.visual(
            Box((0.084, 0.022, 0.018)),
            origin=Origin(
                xyz=(0.160 * math.cos(angle), 0.160 * math.sin(angle), 0.274),
                rpy=(0.0, 0.0, angle),
            ),
            material=black_plastic,
            name=f"radial_bracket_{index}",
        )
        holder.visual(
            Box((0.048, 0.026, 0.105)),
            origin=Origin(
                xyz=(0.176 * math.cos(angle), 0.176 * math.sin(angle), 0.335),
                rpy=(0.0, 0.0, angle),
            ),
            material=black_plastic,
            name=f"upright_clip_{index}",
        )
        holder.visual(
            Box((0.052, 0.036, 0.014)),
            origin=Origin(
                xyz=(0.180 * math.cos(angle), 0.180 * math.sin(angle), 0.392),
                rpy=(0.0, 0.0, angle),
            ),
            material=black_plastic,
            name=f"clip_cap_{index}",
        )
    holder.inertial = Inertial.from_geometry(Cylinder(radius=0.21, length=0.46), mass=0.9, origin=Origin(xyz=(0, 0, 0.13)))

    bowl = model.part("bowl")
    bowl.visual(_bowl_shell_mesh("stainless_bowl_shell"), material=stainless, name="stainless_bowl_shell")
    bowl.visual(_conical_band_mesh("blue_outer_bowl_band"), material=blue_enamel, name="blue_outer_band")
    # Small surface-mounted white paw marks evoke the printed pet-bowl decoration
    # without adding labels or unrelated scene objects.
    paw_specs = [
        (-0.020, -0.119, 0.061, 0.013),
        (0.000, -0.128, 0.078, 0.007),
        (-0.016, -0.130, 0.082, 0.005),
        (0.016, -0.130, 0.082, 0.005),
        (-0.030, -0.124, 0.073, 0.0045),
        (0.030, -0.124, 0.073, 0.0045),
    ]
    for index, (x, y, z, radius) in enumerate(paw_specs):
        bowl.visual(
            Cylinder(radius=radius, length=0.010),
            origin=Origin(xyz=(x, y, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=white_print,
            name=f"paw_mark_{index}",
        )
    bowl.inertial = Inertial.from_geometry(Cylinder(radius=0.15, length=0.11), mass=0.45, origin=Origin(xyz=(0, 0, 0.055)))

    model.articulation(
        "height_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=holder,
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.12, lower=0.0, upper=0.090),
    )
    model.articulation(
        "holder_to_bowl",
        ArticulationType.FIXED,
        parent=holder,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, 0.270)),
    )

    # Folding legs: each leg is a separate part hinged at the hub perimeter.
    # Positive q rotates the leg upward (fold-flat) around the tangent axis.
    _leg_angles = (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)
    for index, angle in enumerate(_leg_angles):
        leg = model.part(f"leg_{index}")
        # Leg member: rounded rod extending radially from the hinge.
        # Capsule axis (local Z) is rotated to the radial direction via rpy.
        leg.visual(
            leg_member_mesh,
            origin=Origin(
                xyz=(0.110 * math.cos(angle), 0.110 * math.sin(angle), -0.012),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=soft_black,
            name=f"leg_member_{index}",
        )
        # Toe pad near the outer end of the leg, overlapping the capsule
        # so the part reads as one connected assembly.
        leg.visual(
            Box((0.016, 0.044, 0.060)),
            origin=Origin(
                xyz=(0.190 * math.cos(angle), 0.190 * math.sin(angle), -0.030),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=soft_black,
            name=f"toe_pad_{index}",
        )

        model.articulation(
            f"hub_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=leg,
            origin=Origin(xyz=(0.090 * math.cos(angle), 0.090 * math.sin(angle), 0.030)),
            axis=(math.sin(angle), -math.cos(angle), 0.0),
            motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=1.55),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    holder = object_model.get_part("bowl_holder")
    bowl = object_model.get_part("bowl")
    slide = object_model.get_articulation("height_slide")

    # Folding leg hinge allowances: each leg member nests at the hub hinge point.
    for index in range(4):
        leg = object_model.get_part(f"leg_{index}")
        ctx.allow_overlap(
            base,
            leg,
            elem_a="central_hub",
            elem_b=f"leg_member_{index}",
            reason=f"Leg {index} member end wraps the hub hinge barrel at the folding pivot.",
        )
        ctx.allow_overlap(
            base,
            leg,
            elem_a=f"hinge_barrel_{index}",
            elem_b=f"leg_member_{index}",
            reason=f"Hinge barrel {index} is captured inside the leg member end at the pivot.",
        )

    ctx.allow_overlap(
        base,
        holder,
        elem_a="hollow_pedestal_sleeve",
        elem_b="sliding_inner_post",
        reason="The inner height-adjustment post is intentionally retained inside the simplified pedestal sleeve.",
    )
    ctx.allow_overlap(
        holder,
        bowl,
        elem_a="ribbed_support_ring",
        elem_b="stainless_bowl_shell",
        reason="The removable bowl is intentionally seated a few millimeters into the molded support ring.",
    )
    for index in range(4):
        ctx.allow_overlap(
            bowl,
            holder,
            elem_a="stainless_bowl_shell",
            elem_b=f"upright_clip_{index}",
            reason="The molded upright clip lightly captures the removable bowl wall so the bowl is not loose in the raised stand.",
        )
        ctx.expect_overlap(
            bowl,
            holder,
            axes="z",
            min_overlap=0.006,
            elem_a="stainless_bowl_shell",
            elem_b=f"upright_clip_{index}",
            name=f"upright clip {index} captures bowl height",
        )
    ctx.expect_overlap(
        holder,
        bowl,
        axes="xy",
        min_overlap=0.20,
        elem_a="ribbed_support_ring",
        elem_b="stainless_bowl_shell",
        name="bowl is seated in circular holder ring",
    )
    ctx.expect_gap(
        bowl,
        holder,
        axis="z",
        max_gap=0.030,
        max_penetration=0.012,
        positive_elem="stainless_bowl_shell",
        negative_elem="ribbed_support_ring",
        name="bowl depth sits just above support ring",
    )
    ctx.expect_within(
        holder,
        base,
        axes="xy",
        margin=0.004,
        inner_elem="sliding_inner_post",
        outer_elem="hollow_pedestal_sleeve",
        name="telescoping post centered in pedestal sleeve",
    )
    ctx.expect_overlap(
        holder,
        base,
        axes="z",
        min_overlap=0.060,
        elem_a="sliding_inner_post",
        elem_b="hollow_pedestal_sleeve",
        name="collapsed height adjustment retains insertion",
    )

    rest_pos = ctx.part_world_position(holder)
    with ctx.pose({slide: 0.090}):
        ctx.expect_within(
            holder,
            base,
            axes="xy",
            margin=0.004,
            inner_elem="sliding_inner_post",
            outer_elem="hollow_pedestal_sleeve",
            name="extended post remains centered",
        )
        ctx.expect_overlap(
            holder,
            base,
            axes="z",
            min_overlap=0.055,
            elem_a="sliding_inner_post",
            elem_b="hollow_pedestal_sleeve",
            name="extended height retains post in sleeve",
        )
        extended_pos = ctx.part_world_position(holder)

    ctx.check(
        "height slide raises the bowl holder",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.080,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    # Verify each hub_to_leg hinge folds the leg upward for compact storage.
    for index in range(4):
        leg = object_model.get_part(f"leg_{index}")
        hinge = object_model.get_articulation(f"hub_to_leg_{index}")
        deployed_aabb = ctx.part_world_aabb(leg)
        with ctx.pose({hinge: 1.50}):
            folded_aabb = ctx.part_world_aabb(leg)
        ctx.check(
            f"hub_to_leg_{index} folds leg_{index} upward for storage",
            deployed_aabb is not None
            and folded_aabb is not None
            and folded_aabb[1][2] > deployed_aabb[1][2] + 0.04,
            details=f"deployed_max_z={deployed_aabb[1][2] if deployed_aabb else None}, "
            f"folded_max_z={folded_aabb[1][2] if folded_aabb else None}",
        )

    return ctx.report()


object_model = build_object_model()
