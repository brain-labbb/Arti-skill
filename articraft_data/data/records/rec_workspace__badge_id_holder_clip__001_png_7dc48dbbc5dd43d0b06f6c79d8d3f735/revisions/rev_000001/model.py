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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


METAL = Material("brushed_spring_steel", rgba=(0.72, 0.82, 0.82, 1.0))
METAL_DARK = Material("shadowed_cut_metal", rgba=(0.10, 0.12, 0.11, 1.0))
CLEAR_PLASTIC = Material("clear_frosted_vinyl", rgba=(0.82, 0.97, 1.00, 0.34))


def _rounded_plate(
    *,
    width: float,
    length: float,
    thickness: float,
    corner_radius: float,
    holes: tuple[tuple[float, float, float], ...] = (),
) -> cq.Workplane:
    """Thin stamped plate centered on XY, with through-punched round holes."""
    plate = cq.Workplane("XY").box(width, length, thickness)
    if corner_radius > 0.0:
        plate = plate.edges("|Z").fillet(corner_radius)
    for x, y, diameter in holes:
        plate = plate.faces(">Z").workplane().pushPoints([(x, y)]).hole(diameter)
    return plate


def _slotted_clear_strap() -> cq.Workplane:
    """Transparent badge strap with a punched oblong slot at the badge end."""
    strap = _rounded_plate(
        width=0.020,
        length=0.064,
        thickness=0.0016,
        corner_radius=0.004,
        holes=((0.0, -0.027, 0.0065),),
    )
    # The visible badge/key-ring opening is an oblong slot: two end holes joined
    # by a rectangular through-cut, like the reference's transparent tab.
    strap = strap.faces(">Z").workplane().pushPoints([(0.0, 0.014), (0.0, 0.026)]).hole(0.006)
    strap = strap.faces(">Z").workplane().center(0.0, 0.020).rect(0.006, 0.012).cutThruAll()
    return strap


def _annular_button() -> cq.Workplane:
    """Raised snap ring/cup around the swivel button."""
    return cq.Workplane("XY").circle(0.0072).circle(0.0040).extrude(0.0013)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="badge_id_holder_clip",
        meta={
            "classification_note": "Reference and category both indicate a badge/ID holder clip.",
            "scale": "Approximately 75 mm long, 20 mm wide, thin stamped metal with clear vinyl strap.",
        },
    )

    clip_base = model.part("clip_base")

    # Lower stamped jaw: thin steel strip with a real punched hole.
    clip_base.visual(
        mesh_from_cadquery(
            _rounded_plate(
                width=0.018,
                length=0.052,
                thickness=0.0012,
                corner_radius=0.002,
                holes=((0.0, -0.004, 0.0062),),
            ),
            "lower_jaw_plate",
            tolerance=0.00035,
            angular_tolerance=0.08,
        ),
        origin=Origin(xyz=(0.0, -0.018, -0.0068)),
        material=METAL,
        name="lower_plate",
    )
    # Thin folded side channels stiffen the stamped lower jaw and make it read
    # as bent sheet metal rather than a solid block.
    for idx, x in enumerate((-0.0089, 0.0089)):
        clip_base.visual(
            Box((0.0014, 0.036, 0.0038)),
            origin=Origin(xyz=(x, -0.014, -0.0045)),
            material=METAL,
            name=f"side_rail_{idx}",
        )
    # Front curled lower lip and opposing lower serrations.
    clip_base.visual(
        Box((0.018, 0.0014, 0.0030)),
        origin=Origin(xyz=(0.0, -0.0440, -0.0051), rpy=(0.18, 0.0, 0.0)),
        material=METAL,
        name="lower_front_lip",
    )
    for idx, x in enumerate((-0.0054, -0.0027, 0.0, 0.0027, 0.0054)):
        clip_base.visual(
            Box((0.0015, 0.0030, 0.0008)),
            origin=Origin(xyz=(x, -0.0360, -0.0058)),
            material=METAL_DARK,
            name=f"lower_tooth_{idx}",
        )

    # Upright hinge ears and pin; the moving jaw's barrel wraps this pin.
    for idx, x in enumerate((-0.0097, 0.0097)):
        clip_base.visual(
            Box((0.0018, 0.0080, 0.0100)),
            origin=Origin(xyz=(x, -0.0006, -0.0020)),
            material=METAL,
            name=f"hinge_ear_{idx}",
        )
        clip_base.visual(
            Cylinder(radius=0.0020, length=0.0007),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=METAL_DARK,
            name=f"ear_rivet_{idx}",
        )
    clip_base.visual(
        Cylinder(radius=0.0011, length=0.0230),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=METAL,
        name="hinge_pin",
    )
    # Small torsion-spring coils drawn around the hinge pin at either side of
    # the center jaw barrel.
    for idx, x in enumerate((-0.0062, 0.0062)):
        clip_base.visual(
            Cylinder(radius=0.0027, length=0.0020),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=METAL,
            name=f"spring_coil_{idx}",
        )

    # Rear swivel receiver for the clear strap's snap button.
    clip_base.visual(
        mesh_from_cadquery(
            _rounded_plate(width=0.0155, length=0.021, thickness=0.0010, corner_radius=0.0018),
            "swivel_mount_plate",
            tolerance=0.00035,
            angular_tolerance=0.08,
        ),
        origin=Origin(xyz=(0.0, 0.0150, 0.0040)),
        material=METAL,
        name="swivel_mount",
    )
    clip_base.visual(
        Box((0.0200, 0.0140, 0.0018)),
        origin=Origin(xyz=(0.0, 0.0060, 0.0031)),
        material=METAL,
        name="rear_bridge",
    )
    clip_base.visual(
        Cylinder(radius=0.0050, length=0.0015),
        origin=Origin(xyz=(0.0, 0.0180, 0.00525)),
        material=METAL,
        name="swivel_receiver",
    )

    jaw = model.part("jaw")
    jaw.visual(
        mesh_from_cadquery(
            _rounded_plate(width=0.0160, length=0.043, thickness=0.0012, corner_radius=0.0018),
            "upper_jaw_plate",
            tolerance=0.00035,
            angular_tolerance=0.08,
        ),
        origin=Origin(xyz=(0.0, -0.0200, -0.0028)),
        material=METAL,
        name="upper_plate",
    )
    jaw.visual(
        Box((0.0140, 0.0150, 0.0010)),
        origin=Origin(xyz=(0.0, -0.0120, -0.0023)),
        material=METAL,
        name="thumb_pad",
    )
    jaw.visual(
        Cylinder(radius=0.0043, length=0.0010),
        origin=Origin(xyz=(0.0, -0.0120, -0.0014)),
        material=METAL,
        name="thumb_rivet",
    )
    jaw.visual(
        Cylinder(radius=0.0022, length=0.0125),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=METAL,
        name="hinge_barrel",
    )
    jaw.visual(
        Box((0.0120, 0.0060, 0.0010)),
        origin=Origin(xyz=(0.0, -0.0015, -0.0021)),
        material=METAL,
        name="barrel_web",
    )
    jaw.visual(
        Box((0.0160, 0.0014, 0.0030)),
        origin=Origin(xyz=(0.0, -0.0415, -0.0045), rpy=(-0.12, 0.0, 0.0)),
        material=METAL,
        name="upper_front_lip",
    )
    for idx, x in enumerate((-0.0054, -0.0027, 0.0, 0.0027, 0.0054)):
        jaw.visual(
            Box((0.0015, 0.0030, 0.0008)),
            origin=Origin(xyz=(x, -0.0396, -0.0052)),
            material=METAL_DARK,
            name=f"upper_tooth_{idx}",
        )

    swivel_tab = model.part("swivel_tab")
    # Clear strip extends behind the metal clip; its local origin is the swivel
    # rivet center, so the whole connector can rotate around the snap.
    swivel_tab.visual(
        mesh_from_cadquery(
            _slotted_clear_strap(),
            "clear_slotted_strap",
            tolerance=0.00035,
            angular_tolerance=0.08,
        ),
        origin=Origin(xyz=(0.0, 0.0270, 0.0009)),
        material=CLEAR_PLASTIC,
        name="strap",
    )
    for idx, x in enumerate((-0.0092, 0.0092)):
        swivel_tab.visual(
            Box((0.0010, 0.0580, 0.0007)),
            origin=Origin(xyz=(x, 0.0270, 0.0020)),
            material=CLEAR_PLASTIC,
            name=f"strap_edge_{idx}",
        )
    swivel_tab.visual(
        mesh_from_cadquery(
            _rounded_plate(width=0.0140, length=0.018, thickness=0.0010, corner_radius=0.0016),
            "snap_reinforcement_plate",
            tolerance=0.00035,
            angular_tolerance=0.08,
        ),
        origin=Origin(xyz=(0.0, 0.0000, 0.0020)),
        material=METAL,
        name="snap_plate",
    )
    swivel_tab.visual(
        Cylinder(radius=0.0026, length=0.0020),
        origin=Origin(xyz=(0.0, 0.0, 0.0010)),
        material=METAL,
        name="pivot_stem",
    )
    swivel_tab.visual(
        Cylinder(radius=0.0060, length=0.0015),
        origin=Origin(xyz=(0.0, 0.0, 0.0032)),
        material=METAL,
        name="button_cap",
    )
    swivel_tab.visual(
        mesh_from_cadquery(
            _annular_button(),
            "raised_snap_ring",
            tolerance=0.00025,
            angular_tolerance=0.06,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0039)),
        material=METAL,
        name="button_ring",
    )
    swivel_tab.visual(
        Cylinder(radius=0.0028, length=0.00045),
        origin=Origin(xyz=(0.0, 0.0, 0.00410)),
        material=METAL_DARK,
        name="button_dimple",
    )

    model.articulation(
        "clip_base_to_jaw",
        ArticulationType.REVOLUTE,
        parent=clip_base,
        child=jaw,
        origin=Origin(),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=0.45),
    )
    model.articulation(
        "clip_base_to_swivel_tab",
        ArticulationType.CONTINUOUS,
        parent=clip_base,
        child=swivel_tab,
        origin=Origin(xyz=(0.0, 0.0180, 0.0060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    clip_base = object_model.get_part("clip_base")
    jaw = object_model.get_part("jaw")
    swivel_tab = object_model.get_part("swivel_tab")
    jaw_joint = object_model.get_articulation("clip_base_to_jaw")
    swivel_joint = object_model.get_articulation("clip_base_to_swivel_tab")

    ctx.allow_overlap(
        clip_base,
        jaw,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason="Captured hinge pin intentionally passes through the moving jaw barrel.",
    )
    ctx.expect_overlap(
        jaw,
        clip_base,
        axes="xyz",
        elem_a="hinge_barrel",
        elem_b="hinge_pin",
        min_overlap=0.001,
        name="jaw barrel is coaxial with captured pin",
    )

    ctx.expect_gap(
        jaw,
        clip_base,
        axis="z",
        positive_elem="upper_plate",
        negative_elem="lower_plate",
        min_gap=0.001,
        max_gap=0.006,
        name="spring jaw has realistic closed clearance above lower jaw",
    )
    ctx.expect_overlap(
        jaw,
        clip_base,
        axes="xy",
        elem_a="upper_plate",
        elem_b="lower_plate",
        min_overlap=0.010,
        name="upper and lower jaw plates oppose each other",
    )
    ctx.expect_gap(
        swivel_tab,
        clip_base,
        axis="z",
        positive_elem="strap",
        negative_elem="swivel_receiver",
        min_gap=0.0,
        max_gap=0.001,
        name="clear swivel strap seats just above receiver button",
    )
    ctx.expect_overlap(
        swivel_tab,
        clip_base,
        axes="xy",
        elem_a="button_cap",
        elem_b="swivel_receiver",
        min_overlap=0.004,
        name="swivel button remains centered on receiver",
    )

    rest_lip = ctx.part_element_world_aabb(jaw, elem="upper_front_lip")
    with ctx.pose({jaw_joint: 0.45}):
        open_lip = ctx.part_element_world_aabb(jaw, elem="upper_front_lip")
    ctx.check(
        "jaw hinge opens front teeth upward",
        rest_lip is not None
        and open_lip is not None
        and open_lip[1][2] > rest_lip[1][2] + 0.010,
        details=f"closed_lip={rest_lip}, open_lip={open_lip}",
    )

    rest_strap = ctx.part_element_world_aabb(swivel_tab, elem="strap")
    with ctx.pose({swivel_joint: math.pi / 2.0}):
        turned_strap = ctx.part_element_world_aabb(swivel_tab, elem="strap")
    if rest_strap is not None and turned_strap is not None:
        rest_x = rest_strap[1][0] - rest_strap[0][0]
        rest_y = rest_strap[1][1] - rest_strap[0][1]
        turned_x = turned_strap[1][0] - turned_strap[0][0]
        turned_y = turned_strap[1][1] - turned_strap[0][1]
        swivel_ok = rest_y > rest_x * 2.0 and turned_x > turned_y * 2.0
    else:
        swivel_ok = False
    ctx.check(
        "swivel tab rotates ninety degrees around snap",
        swivel_ok,
        details=f"rest={rest_strap}, turned={turned_strap}",
    )

    return ctx.report()


object_model = build_object_model()
