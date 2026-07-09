from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    scale_geometry_to_size,
)


BODY_RADIUS = 0.030
BODY_TOP_Z = 0.104
LABEL_Z = 0.052
LABEL_HEIGHT = 0.045
CAP_RADIUS = 0.0265
CAP_HEIGHT = 0.020
CAP_BOTTOM_Z = BODY_TOP_Z
CAP_LIFT = 0.055
CAPSULE_COUNT = 126


def _revolved_profile(points: list[tuple[float, float]]) -> cq.Workplane:
    """Revolve an X/Z cross-section around the world Z axis."""
    return (
        cq.Workplane("XZ")
        .polyline(points)
        .close()
        .revolve(360.0, axisStart=(0.0, 0.0, 0.0), axisEnd=(0.0, 1.0, 0.0))
    )


def _bottle_shell() -> cq.Workplane:
    # A single hollow plastic shell: thick bottom, straight cylindrical body,
    # gently rounded/tapered shoulder, threaded neck, and open mouth.
    return _revolved_profile(
        [
            (0.000, 0.003),
            (BODY_RADIUS, 0.003),
            (BODY_RADIUS, 0.074),
            (0.029, 0.080),
            (0.026, 0.088),
            (0.020, 0.096),
            (0.020, BODY_TOP_Z),
            (0.016, BODY_TOP_Z),
            (0.016, 0.098),
            (0.020, 0.091),
            (0.023, 0.083),
            (0.026, 0.074),
            (0.026, 0.008),
            (0.000, 0.008),
        ]
    )


def _label_sleeve(radius_outer: float, z_min: float, z_max: float) -> cq.Workplane:
    # Thin annular sleeve instead of a solid label cylinder, so the label reads
    # as paper wrapped around the outside of the bottle.
    return _revolved_profile(
        [
            (BODY_RADIUS - 0.00015, z_min),
            (radius_outer, z_min),
            (radius_outer, z_max),
            (BODY_RADIUS - 0.00015, z_max),
        ]
    )


def _packed_softgel_mass() -> cq.Workplane:
    # A translucent connected amber volume that follows the usable inner bottle
    # cavity.  It keeps the many individual capsule visuals supported while
    # reading as a bottle completely filled with softgels from floor to mouth.
    return _revolved_profile(
        [
            (0.000, 0.0078),
            (0.0251, 0.0078),
            (0.0251, 0.0730),
            (0.0224, 0.0830),
            (0.0190, 0.0915),
            (0.0132, 0.0975),
            (0.000, 0.0975),
        ]
    )


def _inner_capsule_radius(z: float) -> float:
    """Conservative centerline radius for a tangential softgel at height z."""
    if z <= 0.072:
        return 0.0214
    if z <= 0.083:
        return 0.0214 - (z - 0.072) / 0.011 * 0.0034
    if z <= 0.093:
        return 0.0180 - (z - 0.083) / 0.010 * 0.0040
    return 0.0135


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="omega3_fish_oil_bottle")

    glossy_white = model.material("glossy_white_plastic", rgba=(0.96, 0.96, 0.93, 1.0))
    label_paper = model.material("printed_white_paper", rgba=(1.0, 1.0, 0.96, 1.0))
    label_teal = model.material("label_teal_ink", rgba=(0.17, 0.78, 0.75, 1.0))
    label_navy = model.material("label_navy_ink", rgba=(0.04, 0.08, 0.22, 1.0))
    label_gold = model.material("label_gold_ink", rgba=(0.95, 0.74, 0.20, 1.0))
    black_plastic = model.material("ribbed_black_plastic", rgba=(0.01, 0.01, 0.01, 1.0))
    amber_gel = model.material("translucent_amber_gel", rgba=(1.0, 0.70, 0.08, 0.58))

    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_cadquery(_bottle_shell(), "bottle_hollow_shell", tolerance=0.0008),
        origin=Origin(),
        material=glossy_white,
        name="hollow_shell",
    )
    bottle.visual(
        mesh_from_cadquery(
            _label_sleeve(BODY_RADIUS + 0.0009, LABEL_Z - LABEL_HEIGHT / 2.0, LABEL_Z + LABEL_HEIGHT / 2.0),
            "wraparound_label_band",
            tolerance=0.0008,
        ),
        origin=Origin(),
        material=label_paper,
        name="label_band",
    )
    bottle.visual(
        mesh_from_cadquery(
            _label_sleeve(BODY_RADIUS + 0.0012, 0.0210, 0.0232),
            "label_bottom_teal_ring",
            tolerance=0.0008,
        ),
        origin=Origin(),
        material=label_teal,
        name="teal_ring",
    )
    bottle.visual(
        mesh_from_cadquery(
            _label_sleeve(BODY_RADIUS + 0.00135, 0.0245, 0.0254),
            "label_gold_ring",
            tolerance=0.0008,
        ),
        origin=Origin(),
        material=label_gold,
        name="gold_ring",
    )
    bottle.visual(
        mesh_from_cadquery(
            _label_sleeve(BODY_RADIUS + 0.00145, 0.0260, 0.0266),
            "label_navy_pinstripe",
            tolerance=0.0008,
        ),
        origin=Origin(),
        material=label_navy,
        name="navy_pinstripe",
    )

    softgels = model.part("softgels")
    softgels.visual(
        Cylinder(radius=0.0251, length=0.0660),
        origin=Origin(xyz=(0.0, 0.0, 0.0408)),
        material=amber_gel,
        name="packed_core",
    )
    softgels.visual(
        Cylinder(radius=0.0224, length=0.0120),
        origin=Origin(xyz=(0.0, 0.0, 0.0785)),
        material=amber_gel,
        name="shoulder_core",
    )
    softgels.visual(
        Cylinder(radius=0.0185, length=0.0100),
        origin=Origin(xyz=(0.0, 0.0, 0.0880)),
        material=amber_gel,
        name="neck_core",
    )
    softgels.visual(
        Cylinder(radius=0.0132, length=0.0075),
        origin=Origin(xyz=(0.0, 0.0, 0.0940)),
        material=amber_gel,
        name="mouth_core",
    )
    softgel_geom = scale_geometry_to_size(
        Sphere(0.005),
        (0.014, 0.007, 0.006),
        filename="softgel_ovoid",
    )
    for i in range(CAPSULE_COUNT):
        layer = i // 9
        slot = i % 9
        z = 0.013 + (layer % 14) * 0.0058 + (0.0012 if slot % 2 else 0.0)
        angle = slot * (2.0 * math.pi / 9.0) + layer * 0.47
        ring = slot % 3
        radius_fraction = (0.22, 0.58, 0.93)[ring]
        radius = _inner_capsule_radius(z) * radius_fraction
        # Make the top neck visibly full as well, with smaller-radius rings that
        # sit just below the open mouth.
        if i >= CAPSULE_COUNT - 18:
            top_index = i - (CAPSULE_COUNT - 18)
            z = 0.0865 + (top_index // 6) * 0.0030 + (0.0006 if top_index % 2 else 0.0)
            angle = (top_index % 6) * (2.0 * math.pi / 6.0) + top_index * 0.21
            radius = _inner_capsule_radius(z) * (0.30 + 0.30 * (top_index % 3))
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        softgels.visual(
            softgel_geom,
            origin=Origin(
                xyz=(x, y, z),
                rpy=(0.10 * ((i % 5) - 2), 0.22 * ((i % 4) - 1.5), angle + math.pi / 2.0),
            ),
            material=amber_gel,
            name=f"capsule_{i}",
        )

    cap = model.part("cap")
    cap.visual(
        Cylinder(radius=CAP_RADIUS, length=CAP_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, CAP_HEIGHT / 2.0)),
        material=black_plastic,
        name="cap_shell",
    )
    cap.visual(
        Cylinder(radius=CAP_RADIUS * 0.92, length=0.0016),
        origin=Origin(xyz=(0.0, 0.0, CAP_HEIGHT + 0.0008)),
        material=black_plastic,
        name="slight_top_dome",
    )
    rib_count = 48
    for i in range(rib_count):
        theta = 2.0 * math.pi * i / rib_count
        rib_radius = CAP_RADIUS + 0.00025
        cap.visual(
            Box((0.0030, 0.0010, 0.0155)),
            origin=Origin(
                xyz=(rib_radius * math.cos(theta), rib_radius * math.sin(theta), 0.0095),
                rpy=(0.0, 0.0, theta),
            ),
            material=black_plastic,
            name=f"cap_rib_{i}",
        )

    model.articulation(
        "bottle_to_softgels",
        ArticulationType.FIXED,
        parent=bottle,
        child=softgels,
        origin=Origin(),
    )
    model.articulation(
        "bottle_to_cap",
        ArticulationType.PRISMATIC,
        parent=bottle,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, CAP_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.25, lower=0.0, upper=CAP_LIFT),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    cap = object_model.get_part("cap")
    softgels = object_model.get_part("softgels")
    cap_slide = object_model.get_articulation("bottle_to_cap")

    ctx.check("bottle_part_present", bottle is not None, "Expected bottle part.")
    ctx.check("cap_part_present", cap is not None, "Expected cap part.")
    ctx.check("softgels_part_present", softgels is not None, "Expected softgels part.")
    if bottle is None or cap is None or softgels is None or cap_slide is None:
        return ctx.report()

    bottle_aabb = ctx.part_world_aabb(bottle)
    cap_aabb = ctx.part_world_aabb(cap)
    ctx.check("bottle_aabb_present", bottle_aabb is not None, "Expected bottle AABB.")
    ctx.check("cap_aabb_present", cap_aabb is not None, "Expected cap AABB.")
    if bottle_aabb is not None and cap_aabb is not None:
        bmin, bmax = bottle_aabb
        cmin, cmax = cap_aabb
        diameter = float(bmax[0] - bmin[0])
        total_height = float(cmax[2] - bmin[2])
        ctx.check("diameter_about_6cm", 0.058 <= diameter <= 0.065, details=f"diameter={diameter}")
        ctx.check("height_about_12cm", 0.118 <= total_height <= 0.130, details=f"height={total_height}")

    capsule_visuals = [v for v in softgels.visuals if (v.name or "").startswith("capsule_")]
    rib_visuals = [v for v in cap.visuals if (v.name or "").startswith("cap_rib_")]
    ctx.check("many_capsules_emitted", len(capsule_visuals) == CAPSULE_COUNT, details=f"count={len(capsule_visuals)}")
    ctx.check("ribbed_cap_emitted", len(rib_visuals) >= 40, details=f"ribs={len(rib_visuals)}")
    ctx.check("wraparound_label_present", bottle.get_visual("label_band") is not None, "Expected printed wraparound label band.")
    removed_ink_names = (
        "fish_oil_title",
        "omega3_title",
        "strength_line",
        "softgel_count_badge",
    )
    bottle_visual_names = tuple(v.name or "" for v in bottle.visuals)
    ctx.check(
        "blocky_label_text_removed",
        all(name not in bottle_visual_names for name in removed_ink_names)
        and not any(name.startswith(("benefit_dot_", "benefit_line_")) for name in bottle_visual_names),
        details="Expected only clean wraparound sleeve and subtle accent stripes, not blocky fake text patches.",
    )
    ctx.check(
        "subtle_label_stripes_present",
        bottle.get_visual("teal_ring") is not None
        and bottle.get_visual("gold_ring") is not None
        and bottle.get_visual("navy_pinstripe") is not None,
        details="Expected minimal teal, gold, and navy wraparound accent stripes.",
    )

    ctx.allow_overlap(
        bottle,
        softgels,
        elem_a="hollow_shell",
        elem_b="packed_core",
        reason=(
            "The packed amber softgel mass is intentionally contained inside the hollow bottle; "
            "the visual shell is used as the container wall and exact overlap reports the contained volume."
        ),
    )

    ctx.expect_gap(
        cap,
        bottle,
        axis="z",
        max_gap=0.004,
        max_penetration=0.0,
        positive_elem="cap_shell",
        negative_elem="hollow_shell",
        name="closed_cap_sits_on_mouth",
    )
    ctx.expect_within(
        softgels,
        bottle,
        axes="xy",
        inner_elem="packed_core",
        outer_elem="hollow_shell",
        margin=0.001,
        name="softgel_pile_inside_bottle_radius",
    )
    packed_aabb = ctx.part_element_world_aabb(softgels, elem="packed_core")
    ctx.check("packed_core_aabb_present", packed_aabb is not None, "Expected packed softgel mass AABB.")
    if packed_aabb is not None:
        pmin, pmax = packed_aabb
        packed_diameter = float(pmax[0] - pmin[0])
        ctx.check(
            "softgels_fill_inner_diameter",
            packed_diameter >= 0.049,
            details=f"packed_diameter={packed_diameter}",
        )
    softgels_aabb = ctx.part_world_aabb(softgels)
    ctx.check("softgels_aabb_present", softgels_aabb is not None, "Expected full softgel pile AABB.")
    if softgels_aabb is not None:
        smin, smax = softgels_aabb
        softgel_height = float(smax[2] - smin[2])
        ctx.check(
            "softgels_fill_floor_to_mouth",
            softgel_height >= 0.088 and float(smin[2]) <= 0.0085 and float(smax[2]) >= 0.096,
            details=f"z_min={float(smin[2])}, z_max={float(smax[2])}, height={softgel_height}",
        )

    rest_cap_pos = ctx.part_world_position(cap)
    with ctx.pose({cap_slide: CAP_LIFT}):
        lifted_cap_pos = ctx.part_world_position(cap)
        ctx.expect_gap(
            cap,
            bottle,
            axis="z",
            min_gap=0.045,
            positive_elem="cap_shell",
            negative_elem="hollow_shell",
            name="lifted_cap_clears_open_mouth",
        )
    ctx.check(
        "cap_lifts_upward",
        rest_cap_pos is not None and lifted_cap_pos is not None and lifted_cap_pos[2] > rest_cap_pos[2] + 0.045,
        details=f"rest={rest_cap_pos}, lifted={lifted_cap_pos}",
    )

    return ctx.report()


object_model = build_object_model()
