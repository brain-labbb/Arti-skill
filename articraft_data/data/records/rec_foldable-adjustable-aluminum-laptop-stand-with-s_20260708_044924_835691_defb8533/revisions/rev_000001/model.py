from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

SILVER_RGBA = (0.80, 0.81, 0.79, 1.0)
HARDWARE_RGBA = (0.32, 0.33, 0.34, 1.0)
RUBBER_RGBA = (0.03, 0.03, 0.03, 1.0)

RAIL_LEN = 0.290
RAIL_WID = 0.030
RAIL_THK = 0.006
RAIL_Y = 0.095

# Local +X is "rear" (toward the laptop hinge/keyboard back), local -X is
# "front" (toward the screen / laptop opening edge) on every part below.


def _lightening_slot_bar(
    *,
    length: float,
    width: float,
    thickness: float,
    cuts: tuple[tuple[float, float, float, bool], ...],
    name: str,
) -> object:
    """Flat rounded-end bar stock with a row of through cuts.

    `cuts` entries are `(center_x, cut_len, cut_wid, rounded)`.
    """
    body = cq.Workplane("XY").slot2D(length - width, width).extrude(thickness)
    body = body.translate((0.0, 0.0, -thickness / 2.0))
    for center_x, cut_len, cut_wid, rounded in cuts:
        wp = body.faces(">Z").workplane(centerOption="CenterOfMass").center(center_x, 0.0)
        if rounded:
            body = wp.slot2D(max(cut_len - cut_wid, 0.001), cut_wid).cutThruAll()
        else:
            body = wp.rect(cut_len, cut_wid).cutThruAll()
    try:
        body = body.edges("|Z").chamfer(0.0010).edges(">Z").chamfer(0.0006)
    except Exception:
        pass
    return mesh_from_cadquery(body, name, tolerance=0.0006, angular_tolerance=0.08)


def _rubber_pad_mesh(length: float, width: float, thickness: float, name: str) -> object:
    pad = cq.Workplane("XY").slot2D(max(length - width, 0.001), width).extrude(thickness)
    pad = pad.translate((0.0, 0.0, -thickness / 2.0))
    return mesh_from_cadquery(pad, name, tolerance=0.0005, angular_tolerance=0.08)


def _hook_bracket_mesh(width: float, name: str) -> object:
    """Upturned front hook: a clean L-shaped lip at the rail's front tip.

    Side profile in the local XZ plane. Local +X runs back over the rail body
    (so the floor is carried by the rail), local -X cantilevers past the front
    tip where the upturned lip rises. The laptop's front edge rests in the
    inner corner of the L.
    """
    profile = (
        cq.Workplane("XZ")
        .moveTo(0.030, 0.0)
        .lineTo(-0.036, 0.0)      # floor underside, rail side -> front tip
        .lineTo(-0.036, 0.030)    # front lip outer face rises
        .lineTo(-0.030, 0.030)    # lip top
        .lineTo(-0.030, 0.006)    # lip inner face back down to floor top
        .lineTo(0.030, 0.006)     # floor top, front -> rail side
        .close()
        .extrude(width)
    )
    profile = profile.translate((0.0, width / 2.0, 0.0))
    try:
        profile = profile.edges("|Y").fillet(0.0022)
    except Exception:
        pass
    return mesh_from_cadquery(profile, name, tolerance=0.0005, angular_tolerance=0.08)


def _add_rear_upright(part, side: int, silver, hardware, rubber) -> None:
    # Local frame: hinge barrel at local origin, bar runs along +Z once the
    # slotted bar (authored long-axis = X) is rotated -90 deg about Y.
    upright_len = 0.235
    bar_mesh = _lightening_slot_bar(
        length=upright_len,
        width=0.026,
        thickness=0.005,
        cuts=(
            (0.048, 0.036, 0.008, False),
            (0.120, 0.100, 0.009, True),
        ),
        name=f"rear_upright_slots_{side}",
    )
    part.visual(
        bar_mesh,
        origin=Origin(xyz=(0.0, 0.0, upright_len / 2.0 - 0.014), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=silver,
        name="upright_bar",
    )
    pad_mesh = _rubber_pad_mesh(0.058, 0.009, 0.003, f"upright_pad_{side}")
    for index, z in enumerate((0.070, 0.180)):
        part.visual(
            pad_mesh,
            origin=Origin(xyz=(-0.004, 0.0, z), rpy=(0.0, -math.pi / 2.0, 0.0)),
            material=rubber,
            name=f"rubber_pad_{index}",
        )
    # Hinge boss + captured pin at the base, visibly carried by the rail lug.
    # The boss is sized generously so it demonstrably overlaps the bar's own
    # bottom slice (a real knuckle-to-bar connection), not just the pin.
    part.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="hinge_boss",
    )
    part.visual(
        Cylinder(radius=0.0028, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="hinge_pin",
    )
    # Round adjustment pivot buttons embedded in the outer face, within the
    # bar's own width so they read as mounted, not floating past its edge.
    outer_y = -0.010 if side == 0 else 0.010
    for index, z in enumerate((0.030, 0.130)):
        part.visual(
            Cylinder(radius=0.0075, length=0.004),
            origin=Origin(xyz=(-0.0035, outer_y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hardware,
            name=f"adjust_button_{index}",
        )
    part.inertial = Inertial.from_geometry(
        Box((0.006, 0.026, upright_len)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, upright_len / 2.0)),
    )


def _add_brace_strut(part, side: int, silver, hardware) -> None:
    strut_len = 0.145
    bar_mesh = _lightening_slot_bar(
        length=strut_len,
        width=0.013,
        thickness=0.004,
        cuts=((0.0, 0.078, 0.006, True),),
        name=f"brace_strut_slot_{side}",
    )
    part.visual(
        bar_mesh,
        origin=Origin(xyz=(0.0, 0.0, strut_len / 2.0), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=silver,
        name="strut_bar",
    )
    part.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="hinge_boss",
    )
    part.visual(
        Cylinder(radius=0.0022, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="hinge_pin",
    )
    for index, z in enumerate((0.004, strut_len - 0.004)):
        part.visual(
            Cylinder(radius=0.0075, length=0.0045),
            origin=Origin(xyz=(0.0, 0.0, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hardware,
            name=f"pivot_button_{index}",
        )
    part.inertial = Inertial.from_geometry(
        Box((0.004, 0.013, strut_len)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, strut_len / 2.0)),
    )


def _add_front_hook(part, silver, rubber) -> None:
    part.visual(
        _hook_bracket_mesh(0.030, "hook_bracket"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=silver,
        name="hook_bracket",
    )
    # Rubber saddle seated in the L-corner: its base is embedded into the floor
    # top (z=0.006) so it reads as one bonded pad, and it nestles against the
    # upturned lip where the laptop edge rests.
    part.visual(
        Box((0.040, 0.024, 0.008)),
        origin=Origin(xyz=(-0.012, 0.0, 0.007)),
        material=rubber,
        name="rubber_saddle",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.066, 0.030, 0.030)),
        mass=0.02,
        origin=Origin(xyz=(-0.006, 0.0, 0.012)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="foldable_x_frame_laptop_stand")

    silver = model.material("brushed_silver_aluminum", rgba=SILVER_RGBA)
    hardware = model.material("satin_pivot_hardware", rgba=HARDWARE_RGBA)
    rubber = model.material("matte_black_rubber", rgba=RUBBER_RGBA)

    base_cuts = (
        (-0.098, 0.020, 0.007, False),
        (-0.049, 0.020, 0.007, False),
        (0.000, 0.020, 0.007, False),
        (0.049, 0.020, 0.007, False),
        (0.098, 0.020, 0.007, False),
    )
    rail_mesh_0 = _lightening_slot_bar(
        length=RAIL_LEN, width=RAIL_WID, thickness=RAIL_THK, cuts=base_cuts, name="base_rail_slots_0"
    )
    rail_mesh_1 = _lightening_slot_bar(
        length=RAIL_LEN, width=RAIL_WID, thickness=RAIL_THK, cuts=base_cuts, name="base_rail_slots_1"
    )

    base_rail_0 = model.part("base_rail_0")
    base_rail_0.visual(
        rail_mesh_0,
        origin=Origin(xyz=(0.0, -RAIL_Y, RAIL_THK / 2.0)),
        material=silver,
        name="rail_bar",
    )
    base_rail_1 = model.part("base_rail_1")
    base_rail_1.visual(
        rail_mesh_1,
        origin=Origin(xyz=(0.0, RAIL_Y, RAIL_THK / 2.0)),
        material=silver,
        name="rail_bar",
    )
    # Cross ties bind the two rails into one real tray-like frame.
    for index, x in enumerate((-0.120, 0.112)):
        base_rail_0.visual(
            Box((0.012, RAIL_Y * 2.0 + RAIL_WID, 0.005)),
            origin=Origin(xyz=(x, 0.0, RAIL_THK / 2.0)),
            material=silver,
            name=f"cross_tie_{index}",
        )
    for index, x in enumerate((-0.115, 0.115)):
        base_rail_0.visual(
            Cylinder(radius=0.0075, length=0.0035),
            origin=Origin(xyz=(x, -RAIL_Y, -0.0015)),
            material=rubber,
            name=f"rail_foot_0_{index}",
        )
        base_rail_1.visual(
            Cylinder(radius=0.0075, length=0.0035),
            origin=Origin(xyz=(x, RAIL_Y, -0.0015)),
            material=rubber,
            name=f"rail_foot_1_{index}",
        )
    # Hinge lugs that visibly carry the upright and strut pivot pins.
    base_rail_0.visual(
        Box((0.014, 0.012, 0.017)),
        origin=Origin(xyz=(0.108, -RAIL_Y, RAIL_THK - 0.0005)),
        material=hardware,
        name="upright_hinge_lug",
    )
    base_rail_0.visual(
        Box((0.012, 0.010, 0.014)),
        origin=Origin(xyz=(-0.028, -RAIL_Y, RAIL_THK - 0.0005)),
        material=hardware,
        name="strut_hinge_lug",
    )
    base_rail_1.visual(
        Box((0.014, 0.012, 0.017)),
        origin=Origin(xyz=(0.108, RAIL_Y, RAIL_THK - 0.0005)),
        material=hardware,
        name="upright_hinge_lug",
    )
    base_rail_1.visual(
        Box((0.012, 0.010, 0.014)),
        origin=Origin(xyz=(-0.028, RAIL_Y, RAIL_THK - 0.0005)),
        material=hardware,
        name="strut_hinge_lug",
    )
    base_rail_0.inertial = Inertial.from_geometry(
        Box((RAIL_LEN, RAIL_WID, RAIL_THK)), mass=0.15, origin=Origin(xyz=(0.0, 0.0, RAIL_THK / 2.0))
    )
    base_rail_1.inertial = Inertial.from_geometry(
        Box((RAIL_LEN, RAIL_WID, RAIL_THK)), mass=0.15, origin=Origin(xyz=(0.0, 0.0, RAIL_THK / 2.0))
    )

    model.articulation(
        "base_rail_pair",
        ArticulationType.FIXED,
        parent=base_rail_0,
        child=base_rail_1,
        origin=Origin(),
    )

    # Front laptop-retaining hooks, fixed at the front tip of each rail.
    front_hook_0 = model.part("front_hook_0")
    _add_front_hook(front_hook_0, silver, rubber)
    front_hook_1 = model.part("front_hook_1")
    _add_front_hook(front_hook_1, silver, rubber)
    model.articulation(
        "rail_to_hook_0",
        ArticulationType.FIXED,
        parent=base_rail_0,
        child=front_hook_0,
        origin=Origin(xyz=(-0.145, -RAIL_Y, RAIL_THK - 0.002)),
    )
    model.articulation(
        "rail_to_hook_1",
        ArticulationType.FIXED,
        parent=base_rail_1,
        child=front_hook_1,
        origin=Origin(xyz=(-0.145, RAIL_Y, RAIL_THK - 0.002)),
    )

    # Rear uprights: hinge from the rail rear, default pose leans back and up;
    # positive rotation folds them flat toward the rail.
    upright_0 = model.part("upright_0")
    upright_1 = model.part("upright_1")
    _add_rear_upright(upright_0, 0, silver, hardware, rubber)
    _add_rear_upright(upright_1, 1, silver, hardware, rubber)
    for index, (upright, rail, y) in enumerate(
        ((upright_0, base_rail_0, -RAIL_Y), (upright_1, base_rail_1, RAIL_Y))
    ):
        model.articulation(
            f"upright_pitch_{index}",
            ArticulationType.REVOLUTE,
            parent=rail,
            child=upright,
            origin=Origin(xyz=(0.108, y, RAIL_THK), rpy=(0.0, 0.62, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=1.5, lower=-0.10, upper=1.55),
        )

    # Diagonal braces: hinge from a lug forward of center, cross up under the
    # uprights to form the X silhouette. Positive rotation swings them flat.
    strut_0 = model.part("brace_strut_0")
    strut_1 = model.part("brace_strut_1")
    _add_brace_strut(strut_0, 0, silver, hardware)
    _add_brace_strut(strut_1, 1, silver, hardware)
    for index, (strut, rail, y) in enumerate(
        ((strut_0, base_rail_0, -RAIL_Y), (strut_1, base_rail_1, RAIL_Y))
    ):
        model.articulation(
            f"strut_pitch_{index}",
            ArticulationType.REVOLUTE,
            parent=rail,
            child=strut,
            origin=Origin(xyz=(-0.028, y, RAIL_THK), rpy=(0.0, 0.55, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=1.2, lower=-0.10, upper=1.05),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_rail_0 = object_model.get_part("base_rail_0")
    base_rail_1 = object_model.get_part("base_rail_1")
    upright_0 = object_model.get_part("upright_0")
    upright_1 = object_model.get_part("upright_1")
    strut_0 = object_model.get_part("brace_strut_0")
    strut_1 = object_model.get_part("brace_strut_1")
    hook_0 = object_model.get_part("front_hook_0")
    hook_1 = object_model.get_part("front_hook_1")

    def visual_names(part) -> set[str]:
        return {visual.name for visual in part.visuals}

    ctx.check(
        "paired long base rails exist",
        "rail_bar" in visual_names(base_rail_0) and "rail_bar" in visual_names(base_rail_1),
        details=f"{visual_names(base_rail_0)=}, {visual_names(base_rail_1)=}",
    )
    ctx.expect_origin_gap(
        base_rail_1, base_rail_0, axis="y", min_gap=0.0, max_gap=0.001,
        name="base rails are fixed in one shared frame",
    )
    ctx.expect_overlap(
        base_rail_0, base_rail_1, axes="x", min_overlap=0.22,
        elem_a="rail_bar", elem_b="rail_bar",
        name="base rails run parallel along their length",
    )
    ctx.check(
        "base rails include lightening slots",
        all(
            "base_rail_slots" in rail.get_visual("rail_bar").geometry.filename
            for rail in (base_rail_0, base_rail_1)
        ),
        details="base rail meshes are generated with repeated through-cut slots",
    )
    ctx.check(
        "rear uprights include adjustment slots and rubber pads",
        "upright_bar" in visual_names(upright_0)
        and "upright_bar" in visual_names(upright_1)
        and "rubber_pad_0" in visual_names(upright_0)
        and "rubber_pad_1" in visual_names(upright_1),
        details=f"{visual_names(upright_0)=}, {visual_names(upright_1)=}",
    )
    ctx.check(
        "diagonal braces carry a lightening slot and pivot buttons",
        "strut_bar" in visual_names(strut_0)
        and any(name.startswith("pivot_button") for name in visual_names(strut_1)),
        details=f"{visual_names(strut_0)=}, {visual_names(strut_1)=}",
    )
    ctx.check(
        "front retaining hooks with rubber saddles exist",
        "hook_bracket" in visual_names(hook_0)
        and "rubber_saddle" in visual_names(hook_0)
        and "hook_bracket" in visual_names(hook_1),
        details=f"{visual_names(hook_0)=}, {visual_names(hook_1)=}",
    )

    # Captured hinge knuckles: upright/strut bosses and pins nest inside the
    # rail lugs, not floating above them.
    for index, (upright, rail_name) in enumerate(((upright_0, "base_rail_0"), (upright_1, "base_rail_1"))):
        rail = base_rail_0 if rail_name == "base_rail_0" else base_rail_1
        ctx.allow_overlap(
            rail, upright, elem_a="upright_hinge_lug", elem_b="hinge_boss",
            reason="Upright hinge knuckle nests inside the rail's hinge lug.",
        )
        ctx.allow_overlap(
            rail, upright, elem_a="upright_hinge_lug", elem_b="upright_bar",
            reason="Upright bar's lower end seats into the rail lug at the pivot.",
        )
        ctx.allow_overlap(
            rail, upright, elem_a="rail_bar", elem_b="hinge_boss",
            reason="Upright hinge knuckle seats down onto the rail end plate at the pivot.",
        )
        ctx.allow_overlap(
            rail, upright, elem_a="upright_hinge_lug", elem_b="hinge_pin",
            reason="Upright hinge pin is captured through the rail's hinge lug.",
        )
        ctx.expect_overlap(
            upright, rail, axes="y", elem_a="hinge_boss", elem_b="upright_hinge_lug",
            min_overlap=0.004, name=f"upright {index} knuckle engages its lug",
        )
    for index, (strut, rail) in enumerate(((strut_0, base_rail_0), (strut_1, base_rail_1))):
        ctx.allow_overlap(
            rail, strut, elem_a="strut_hinge_lug", elem_b="hinge_boss",
            reason="Brace strut hinge knuckle nests inside the rail's hinge lug.",
        )
        ctx.allow_overlap(
            rail, strut, elem_a="rail_bar", elem_b="hinge_boss",
            reason="Brace strut hinge knuckle seats down onto the rail at the pivot.",
        )
        ctx.allow_overlap(
            rail, strut, elem_a="strut_hinge_lug", elem_b="hinge_pin",
            reason="Brace strut hinge pin is captured through the rail's hinge lug.",
        )
        ctx.expect_overlap(
            strut, rail, axes="y", elem_a="hinge_boss", elem_b="strut_hinge_lug",
            min_overlap=0.004, name=f"strut {index} knuckle engages its lug",
        )

    non_fixed = [
        joint for joint in object_model.articulations
        if joint.articulation_type != ArticulationType.FIXED
    ]
    ctx.check(
        "folding stand has four independent tilt joints",
        len(non_fixed) == 4
        and all(
            joint.motion_limits is not None and joint.motion_limits.lower < joint.motion_limits.upper
            for joint in non_fixed
        ),
        details=f"non_fixed={[joint.name for joint in non_fixed]}",
    )

    upright_joint = object_model.get_articulation("upright_pitch_0")
    strut_joint = object_model.get_articulation("strut_pitch_0")
    open_upright_aabb = ctx.part_world_aabb(upright_0)
    open_strut_aabb = ctx.part_world_aabb(strut_0)
    ctx.check(
        "open uprights stand tall behind the rail",
        open_upright_aabb is not None and open_upright_aabb[1][2] > 0.15,
        details=f"open_upright={open_upright_aabb}",
    )
    ctx.check(
        "open braces cross upward from the front lug",
        open_strut_aabb is not None and open_strut_aabb[1][2] > 0.08,
        details=f"open_strut={open_strut_aabb}",
    )
    with ctx.pose({upright_joint: 1.5, strut_joint: 1.0}):
        folded_upright_aabb = ctx.part_world_aabb(upright_0)
        folded_strut_aabb = ctx.part_world_aabb(strut_0)
    ctx.check(
        "upright folds down flat toward the rail",
        open_upright_aabb is not None
        and folded_upright_aabb is not None
        and folded_upright_aabb[1][2] < open_upright_aabb[1][2] - 0.12,
        details=f"open={open_upright_aabb}, folded={folded_upright_aabb}",
    )
    ctx.check(
        "brace strut swings flat during fold",
        open_strut_aabb is not None
        and folded_strut_aabb is not None
        and folded_strut_aabb[1][2] < open_strut_aabb[1][2] - 0.03,
        details=f"open={open_strut_aabb}, folded={folded_strut_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
