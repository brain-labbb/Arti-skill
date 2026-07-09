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
# Flip-top snap cap geometry (collar + hinged lid)
COLLAR_INNER_RADIUS = 0.0198  # snug on neck (neck outer=0.020)
COLLAR_OUTER_RADIUS = 0.0260
COLLAR_HEIGHT = 0.010
COLLAR_BOTTOM_Z = 0.096  # starts at shoulder/neck junction
COLLAR_TOP_Z = COLLAR_BOTTOM_Z + COLLAR_HEIGHT  # 0.106
LID_RADIUS = 0.023  # larger than collar inner → rests on collar top
LID_THICKNESS = 0.003
HINGE_X = COLLAR_OUTER_RADIUS  # hinge pivot at outer rim of collar
FLIP_OPEN_ANGLE = 2.3  # radians (~132°)
BARREL_LENGTH = 0.012
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


def _cap_collar_ring() -> cq.Workplane:
    """Annular snap collar that sits on the bottle neck rim."""
    return _revolved_profile(
        [
            (COLLAR_INNER_RADIUS, COLLAR_BOTTOM_Z),
            (COLLAR_OUTER_RADIUS, COLLAR_BOTTOM_Z),
            (COLLAR_OUTER_RADIUS, COLLAR_TOP_Z),
            (COLLAR_INNER_RADIUS, COLLAR_TOP_Z),
        ]
    )


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

    # ── Flip-top snap cap: fixed collar + hinged lid ──────────────────
    cap_collar = model.part("cap_collar")
    cap_collar.visual(
        mesh_from_cadquery(_cap_collar_ring(), "cap_collar_ring", tolerance=0.0008),
        origin=Origin(),
        material=black_plastic,
        name="collar_ring",
    )
    # Two hinge lugs flanking the pivot pin (sandwich the lid barrel)
    for sign, suffix in ((-1, "neg"), (1, "pos")):
        cap_collar.visual(
            Box((0.004, 0.003, 0.008)),
            origin=Origin(xyz=(HINGE_X, sign * 0.005, COLLAR_TOP_Z + 0.002)),
            material=black_plastic,
            name=f"hinge_lug_{suffix}",
        )

    flip_lid = model.part("flip_lid")
    # Main lid disk – centered over the opening when closed
    flip_lid.visual(
        Cylinder(radius=LID_RADIUS, length=LID_THICKNESS),
        origin=Origin(xyz=(-HINGE_X, 0.0, LID_THICKNESS / 2.0)),
        material=black_plastic,
        name="lid_disk",
    )
    # Thumb tab on the far edge (opposite hinge) for opening
    flip_lid.visual(
        Box((0.008, 0.012, 0.003)),
        origin=Origin(xyz=(-2.0 * HINGE_X, 0.0, LID_THICKNESS + 0.0005)),
        material=black_plastic,
        name="lid_tab",
    )
    # Hinge ear bridging lid disk to the pivot barrel
    flip_lid.visual(
        Box((0.006, 0.008, LID_THICKNESS)),
        origin=Origin(xyz=(-0.002, 0.0, LID_THICKNESS)),
        material=black_plastic,
        name="hinge_ear",
    )
    # Hinge barrel at the pivot – raised so it sits between the collar lugs
    flip_lid.visual(
        Cylinder(radius=0.002, length=BARREL_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, LID_THICKNESS), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="hinge_barrel",
    )

    model.articulation(
        "bottle_to_softgels",
        ArticulationType.FIXED,
        parent=bottle,
        child=softgels,
        origin=Origin(),
    )
    model.articulation(
        "bottle_to_collar",
        ArticulationType.FIXED,
        parent=bottle,
        child=cap_collar,
        origin=Origin(),
    )
    model.articulation(
        "collar_to_flip_lid",
        ArticulationType.REVOLUTE,
        parent=cap_collar,
        child=flip_lid,
        origin=Origin(xyz=(HINGE_X, 0.0, COLLAR_TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=3.0, lower=0.0, upper=FLIP_OPEN_ANGLE
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    cap_collar = object_model.get_part("cap_collar")
    flip_lid = object_model.get_part("flip_lid")
    softgels = object_model.get_part("softgels")
    lid_hinge = object_model.get_articulation("collar_to_flip_lid")

    ctx.check("bottle_part_present", bottle is not None, "Expected bottle part.")
    ctx.check("cap_collar_part_present", cap_collar is not None, "Expected cap_collar part.")
    ctx.check("flip_lid_part_present", flip_lid is not None, "Expected flip_lid part.")
    ctx.check("softgels_part_present", softgels is not None, "Expected softgels part.")
    ctx.check("lid_hinge_present", lid_hinge is not None, "Expected collar_to_flip_lid articulation.")
    if bottle is None or cap_collar is None or flip_lid is None or softgels is None or lid_hinge is None:
        return ctx.report()

    # ── Variant axis: collar_to_flip_lid is REVOLUTE and lid_disk is visible ──
    ctx.check(
        "collar_to_flip_lid_is_revolute",
        str(lid_hinge.articulation_type).split(".")[-1].lower() == "revolute",
        details=f"type={lid_hinge.articulation_type}",
    )
    ctx.check(
        "flip_lid_has_lid_disk",
        flip_lid.get_visual("lid_disk") is not None,
        details="Expected flip_lid part to expose the visible lid disk.",
    )

    bottle_aabb = ctx.part_world_aabb(bottle)
    ctx.check("bottle_aabb_present", bottle_aabb is not None, "Expected bottle AABB.")
    if bottle_aabb is not None:
        bmin, bmax = bottle_aabb
        diameter = float(bmax[0] - bmin[0])
        total_height = float(bmax[2] - bmin[2])
        ctx.check("diameter_about_6cm", 0.058 <= diameter <= 0.065, details=f"diameter={diameter}")
        ctx.check("height_about_12cm", 0.100 <= total_height <= 0.125, details=f"height={total_height}")

    capsule_visuals = [v for v in softgels.visuals if (v.name or "").startswith("capsule_")]
    ctx.check("many_capsules_emitted", len(capsule_visuals) == CAPSULE_COUNT, details=f"count={len(capsule_visuals)}")
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

    # The collar wraps around the bottle neck as a snap-fit: intentional small
    # overlap between collar inner wall and neck outer wall.
    ctx.allow_overlap(
        bottle,
        cap_collar,
        elem_a="hollow_shell",
        elem_b="collar_ring",
        reason=(
            "The snap-fit collar is intentionally represented as wrapping around "
            "the bottle neck; the inner collar wall contacts the outer neck wall."
        ),
    )
    # Prove the collar is retained on the neck (overlap on Z + within on XY)
    ctx.expect_overlap(
        cap_collar,
        bottle,
        axes="z",
        elem_a="collar_ring",
        elem_b="hollow_shell",
        min_overlap=0.004,
        name="collar_wraps_neck_retained_z",
    )
    ctx.expect_within(
        cap_collar,
        bottle,
        axes="xy",
        inner_elem="collar_ring",
        outer_elem="hollow_shell",
        margin=0.008,
        name="collar_centered_on_bottle_neck",
    )
    # Lid disk covers the collar opening when closed
    ctx.expect_overlap(
        flip_lid,
        cap_collar,
        axes="xy",
        elem_a="lid_disk",
        elem_b="collar_ring",
        min_overlap=0.010,
        name="closed_lid_covers_collar_opening",
    )
    ctx.expect_gap(
        flip_lid,
        cap_collar,
        axis="z",
        max_gap=0.002,
        max_penetration=0.002,
        positive_elem="lid_disk",
        negative_elem="collar_ring",
        name="closed_lid_seated_on_collar",
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

    # Prove the flip lid opens: the lid disk swings away from the mouth
    rest_disk_aabb = ctx.part_element_world_aabb(flip_lid, elem="lid_disk")
    with ctx.pose({lid_hinge: FLIP_OPEN_ANGLE}):
        open_disk_aabb = ctx.part_element_world_aabb(flip_lid, elem="lid_disk")
        ctx.expect_gap(
            flip_lid,
            bottle,
            axis="z",
            min_gap=0.001,
            positive_elem="lid_disk",
            negative_elem="hollow_shell",
            name="open_lid_clears_mouth",
        )
    ctx.check(
        "collar_to_flip_lid_lid_swings_away",
        rest_disk_aabb is not None
        and open_disk_aabb is not None
        and (
            abs(open_disk_aabb[1][2] - rest_disk_aabb[1][2]) > 0.005
            or abs(open_disk_aabb[0][0] - rest_disk_aabb[0][0]) > 0.005
        ),
        details=f"rest_max_z={rest_disk_aabb[1][2] if rest_disk_aabb else None}, "
                f"open_max_z={open_disk_aabb[1][2] if open_disk_aabb else None}",
    )

    return ctx.report()


object_model = build_object_model()
