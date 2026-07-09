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
REEL_PLASTIC = Material("abs_reel_housing", rgba=(0.14, 0.15, 0.18, 1.0))
CORD_NYLON = Material("nylon_retractor_cord", rgba=(0.82, 0.80, 0.74, 1.0))
REEL_DOME = Material("translucent_reel_cap", rgba=(0.80, 0.86, 0.92, 0.55))


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


def _annular_button() -> cq.Workplane:
    """Raised snap ring/cup around the swivel button."""
    return cq.Workplane("XY").circle(0.0072).circle(0.0040).extrude(0.0013)


def _reel_drum_housing() -> cq.Workplane:
    """Round badge-reel drum: solid disc with center bore and cord nozzle."""
    body = (
        cq.Workplane("XY")
        .workplane(offset=-0.006)
        .circle(0.016)
        .extrude(0.012)
    )
    # Axle bore through center
    bore = (
        cq.Workplane("XY")
        .workplane(offset=-0.008)
        .circle(0.0018)
        .extrude(0.016)
    )
    body = body.cut(bore)
    # Cord exit nozzle at +Y edge (solid bump for cord anchor contact)
    # XZ workplane normal is -Y; use negative offset to place at +Y
    nozzle = (
        cq.Workplane("XZ")
        .workplane(offset=-0.021)
        .circle(0.0026)
        .extrude(0.007)
    )
    body = body.union(nozzle)
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="badge_id_holder_clip",
        meta={
            "classification_note": "Retractable badge-reel variant of the ID holder clip.",
            "scale": "Clip ~75 mm, reel drum 32 mm dia, cord travel ~150 mm.",
        },
    )

    # ── clip_base (KEEP identical to parent) ──────────────────────────────
    clip_base = model.part("clip_base")

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
    for idx, x in enumerate((-0.0089, 0.0089)):
        clip_base.visual(
            Box((0.0014, 0.036, 0.0038)),
            origin=Origin(xyz=(x, -0.014, -0.0045)),
            material=METAL,
            name=f"side_rail_{idx}",
        )
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
    for idx, x in enumerate((-0.0062, 0.0062)):
        clip_base.visual(
            Cylinder(radius=0.0027, length=0.0020),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=METAL,
            name=f"spring_coil_{idx}",
        )

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

    # ── jaw (KEEP identical to parent) ────────────────────────────────────
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

    # ── swivel_tab (MODIFIED: drum mount bracket replaces clear strap) ────
    swivel_tab = model.part("swivel_tab")
    # Retained swivel button / snap mechanism
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
    # NEW: drum mount bracket (L-shaped arm + axle)
    swivel_tab.visual(
        Box((0.006, 0.006, 0.005)),
        origin=Origin(xyz=(0.0, 0.003, 0.0045)),
        material=METAL,
        name="mount_riser",
    )
    swivel_tab.visual(
        Box((0.006, 0.026, 0.0015)),
        origin=Origin(xyz=(0.0, 0.015, 0.00725)),
        material=METAL,
        name="drum_mount_arm",
    )
    swivel_tab.visual(
        Cylinder(radius=0.002, length=0.014),
        origin=Origin(xyz=(0.0, 0.028, 0.0)),
        material=METAL,
        name="drum_axle",
    )

    # ── reel_drum (NEW: round spring-reel housing) ────────────────────────
    reel_drum = model.part("reel_drum")
    reel_drum.visual(
        mesh_from_cadquery(
            _reel_drum_housing(),
            "drum_housing",
            tolerance=0.0004,
            angular_tolerance=0.10,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=REEL_PLASTIC,
        name="drum_body",
    )
    # Front face dome/cap (translucent logo area)
    reel_drum.visual(
        Cylinder(radius=0.010, length=0.0008),
        origin=Origin(xyz=(0.0, 0.0, 0.0064)),
        material=REEL_DOME,
        name="front_dome",
    )
    # Back metal plate
    reel_drum.visual(
        Cylinder(radius=0.014, length=0.0005),
        origin=Origin(xyz=(0.0, 0.0, -0.00625)),
        material=METAL,
        name="back_plate",
    )

    # ── card_clip (NEW: badge card clip at cord end) ──────────────────────
    card_clip = model.part("card_clip")
    # Nylon cord stub (along Y axis; enters drum nozzle at rest)
    card_clip.visual(
        Cylinder(radius=0.0008, length=0.020),
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=CORD_NYLON,
        name="cord_stub",
    )
    # Pivot/spring cylinder
    card_clip.visual(
        Cylinder(radius=0.002, length=0.014),
        origin=Origin(xyz=(0.0, 0.019, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=METAL_DARK,
        name="clip_spring",
    )
    # Upper jaw
    card_clip.visual(
        Box((0.012, 0.024, 0.001)),
        origin=Origin(xyz=(0.0, 0.032, 0.0015)),
        material=METAL,
        name="clip_upper_jaw",
    )
    # Lower jaw
    card_clip.visual(
        Box((0.012, 0.024, 0.001)),
        origin=Origin(xyz=(0.0, 0.032, -0.0015)),
        material=METAL,
        name="clip_lower_jaw",
    )
    # Grip teeth on jaw tips
    for i, y_off in enumerate((0.040, 0.042, 0.044)):
        card_clip.visual(
            Box((0.010, 0.0008, 0.0006)),
            origin=Origin(xyz=(0.0, y_off, 0.002)),
            material=METAL_DARK,
            name=f"tooth_upper_{i}",
        )
        card_clip.visual(
            Box((0.010, 0.0008, 0.0006)),
            origin=Origin(xyz=(0.0, y_off, -0.002)),
            material=METAL_DARK,
            name=f"tooth_lower_{i}",
        )

    # ── Articulations ─────────────────────────────────────────────────────
    # KEEP: spring jaw hinge (identical to parent)
    model.articulation(
        "clip_base_to_jaw",
        ArticulationType.REVOLUTE,
        parent=clip_base,
        child=jaw,
        origin=Origin(),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=0.45),
    )
    # KEEP: swivel rotation (identical to parent)
    model.articulation(
        "clip_base_to_swivel_tab",
        ArticulationType.CONTINUOUS,
        parent=clip_base,
        child=swivel_tab,
        origin=Origin(xyz=(0.0, 0.0180, 0.0060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )
    # NEW: reel spool continuous rotation (cord wind/unwind)
    model.articulation(
        "swivel_tab_to_reel_drum",
        ArticulationType.CONTINUOUS,
        parent=swivel_tab,
        child=reel_drum,
        origin=Origin(xyz=(0.0, 0.028, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=8.0),
    )
    # NEW: cord extension prismatic
    model.articulation(
        "reel_drum_to_card_clip",
        ArticulationType.PRISMATIC,
        parent=reel_drum,
        child=card_clip,
        origin=Origin(xyz=(0.0, 0.021, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.5, lower=0.0, upper=0.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    clip_base = object_model.get_part("clip_base")
    jaw = object_model.get_part("jaw")
    swivel_tab = object_model.get_part("swivel_tab")
    reel_drum = object_model.get_part("reel_drum")
    card_clip = object_model.get_part("card_clip")

    jaw_joint = object_model.get_articulation("clip_base_to_jaw")
    swivel_joint = object_model.get_articulation("clip_base_to_swivel_tab")
    reel_spool = object_model.get_articulation("swivel_tab_to_reel_drum")
    cord_extend = object_model.get_articulation("reel_drum_to_card_clip")

    # ── Jaw hinge allowances and checks (KEEP) ──────────────────────────
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

    # Jaw opens upward
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

    # ── Swivel button/receiver (KEEP) ────────────────────────────────────
    ctx.expect_overlap(
        swivel_tab,
        clip_base,
        axes="xy",
        elem_a="button_cap",
        elem_b="swivel_receiver",
        min_overlap=0.004,
        name="swivel button remains centered on receiver",
    )

    # ── NEW: reel_drum housing round geometry check ──────────────────────
    drum_aabb = ctx.part_element_world_aabb(reel_drum, elem="drum_body")
    if drum_aabb is not None:
        drum_dx = drum_aabb[1][0] - drum_aabb[0][0]
        drum_dy = drum_aabb[1][1] - drum_aabb[0][1]
        drum_dz = drum_aabb[1][2] - drum_aabb[0][2]
        # Drum is a disc: XY extents should be ~32mm, Z should be ~12mm
        round_ok = drum_dx > 0.025 and drum_dy > 0.025 and drum_dz < 0.025
    else:
        round_ok = False
    ctx.check(
        "reel_drum housing is round disc with thin profile",
        round_ok,
        details=f"drum_aabb={drum_aabb}",
    )

    # ── NEW: reel_spool continuous rotation proof ─────────────────────────
    rest_nozzle = ctx.part_element_world_aabb(reel_drum, elem="drum_body")
    with ctx.pose({reel_spool: math.pi / 2.0}):
        turned_nozzle = ctx.part_element_world_aabb(reel_drum, elem="drum_body")
    if rest_nozzle is not None and turned_nozzle is not None:
        # After 90° Z rotation, the nozzle (at +Y edge) should shift to +X edge
        rest_cy = (rest_nozzle[0][1] + rest_nozzle[1][1]) / 2.0
        rot_cy = (turned_nozzle[0][1] + turned_nozzle[1][1]) / 2.0
        rest_cx = (rest_nozzle[0][0] + rest_nozzle[1][0]) / 2.0
        rot_cx = (turned_nozzle[0][0] + turned_nozzle[1][0]) / 2.0
        # The drum body center should stay near the axle (only nozzle asymmetry shifts centroid slightly)
        spool_ok = abs(rest_cx - rot_cx) < 0.005 and abs(rest_cy - rot_cy) < 0.005
    else:
        spool_ok = False
    ctx.check(
        "reel_spool rotates drum for cord wind/unwind",
        spool_ok,
        details=f"rest={rest_nozzle}, turned={turned_nozzle}",
    )

    # ── NEW: cord_extend prismatic travel proof ───────────────────────────
    rest_clip_pos = ctx.part_world_position(card_clip)
    with ctx.pose({cord_extend: 0.10}):
        extended_clip_pos = ctx.part_world_position(card_clip)
    ctx.check(
        "cord_extend prismatic moves card_clip outward from reel_drum",
        rest_clip_pos is not None
        and extended_clip_pos is not None
        and extended_clip_pos[1] > rest_clip_pos[1] + 0.05,
        details=f"rest={rest_clip_pos}, extended={extended_clip_pos}",
    )

    # ── NEW: axle-through-bore intentional overlap ────────────────────────
    ctx.allow_overlap(
        swivel_tab,
        reel_drum,
        elem_a="drum_axle",
        elem_b="drum_body",
        reason="Drum axle is press-fit through the drum center bore for rotation support.",
    )
    ctx.expect_overlap(
        reel_drum,
        swivel_tab,
        axes="z",
        elem_a="drum_body",
        elem_b="drum_axle",
        min_overlap=0.008,
        name="drum body spans across the captured axle",
    )

    # ── NEW: cord stub enters drum nozzle at rest ─────────────────────────
    ctx.allow_overlap(
        reel_drum,
        card_clip,
        elem_a="drum_body",
        elem_b="cord_stub",
        reason="Cord stub enters the solid drum nozzle at the retracted rest position.",
    )
    ctx.expect_overlap(
        card_clip,
        reel_drum,
        axes="y",
        elem_a="cord_stub",
        elem_b="drum_body",
        min_overlap=0.001,
        name="cord stub remains inserted in drum nozzle at rest",
    )

    # ── Swivel rotation proof (modified for drum) ────────────────────────
    rest_arm = ctx.part_element_world_aabb(swivel_tab, elem="drum_mount_arm")
    with ctx.pose({swivel_joint: math.pi / 2.0}):
        turned_arm = ctx.part_element_world_aabb(swivel_tab, elem="drum_mount_arm")
    if rest_arm is not None and turned_arm is not None:
        rest_dx = rest_arm[1][0] - rest_arm[0][0]
        rest_dy = rest_arm[1][1] - rest_arm[0][1]
        turned_dx = turned_arm[1][0] - turned_arm[0][0]
        turned_dy = turned_arm[1][1] - turned_arm[0][1]
        swivel_ok = rest_dy > rest_dx * 1.5 and turned_dx > turned_dy * 1.5
    else:
        swivel_ok = False
    ctx.check(
        "swivel tab rotates ninety degrees around snap",
        swivel_ok,
        details=f"rest={rest_arm}, turned={turned_arm}",
    )

    return ctx.report()


object_model = build_object_model()
