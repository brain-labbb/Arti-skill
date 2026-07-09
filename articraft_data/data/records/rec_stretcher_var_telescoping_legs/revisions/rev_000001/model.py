from __future__ import annotations

import math

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
)


def _tube_origin(p0: tuple[float, float, float], p1: tuple[float, float, float]) -> tuple[Origin, float]:
    """Return an Origin that aligns a URDF +Z cylinder between two local points."""
    x0, y0, z0 = p0
    x1, y1, z1 = p1
    dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("tube endpoints must be distinct")
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    return Origin(xyz=((x0 + x1) * 0.5, (y0 + y1) * 0.5, (z0 + z1) * 0.5), rpy=(0.0, pitch, yaw)), length


def _tube(part, name: str, p0, p1, radius: float, material: Material) -> None:
    origin, length = _tube_origin(p0, p1)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _box(part, name: str, size, xyz, material: Material, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="ambulance_stretcher_gurney",
        meta={
            "run_notes": (
                "Telescoping-leg variant: four tube-in-tube vertical height legs replace "
                "the X-brace scissor linkage. Each leg has a prismatic joint along +Z for "
                "height adjustment. Companion colorway: navy/EMS-blue vinyl pads instead of "
                "safety orange. All other functional layers preserved from parent baseline."
            )
        },
    )

    # ── Materials ─────────────────────────────────────────────────────────
    navy = model.material("navy_ems_blue_vinyl", rgba=(0.08, 0.14, 0.38, 1.0))
    black = model.material("black_webbing_rubber", rgba=(0.015, 0.015, 0.012, 1.0))
    metal = model.material("brushed_stainless_tube", rgba=(0.72, 0.74, 0.74, 1.0))
    dark_metal = model.material("dark_grey_hubs", rgba=(0.16, 0.16, 0.16, 1.0))
    grey = model.material("pale_grey_plastic", rgba=(0.50, 0.52, 0.52, 1.0))
    red = model.material("red_release_handle", rgba=(0.90, 0.02, 0.01, 1.0))

    # Companion variation: navy blue pads instead of safety orange
    pad = navy

    # ── Root: lower wheeled carriage ──────────────────────────────────────
    lower = model.part("lower_carriage")
    rail_z = 0.30
    for y in (-0.25, 0.25):
        _tube(lower, f"lower_side_rail_{y:+.2f}", (-0.86, y, rail_z), (0.86, y, rail_z), 0.014, metal)
    for x in (-0.86, 0.0, 0.86):
        _tube(lower, f"lower_cross_rail_{x:+.2f}", (x, -0.25, rail_z), (x, 0.25, rail_z), 0.012, metal)

    # Caster stems and forks (fixed to carriage)
    caster_locations = [(-0.78, -0.24), (-0.78, 0.24), (0.78, -0.24), (0.78, 0.24)]
    for i, (x, y) in enumerate(caster_locations):
        _tube(lower, f"caster_stem_{i}", (x, y, 0.14), (x, y, rail_z), 0.012, metal)
        _box(lower, f"fork_plate_{i}_0", (0.030, 0.006, 0.115), (x, y - 0.032, 0.095), metal)
        _box(lower, f"fork_plate_{i}_1", (0.030, 0.006, 0.115), (x, y + 0.032, 0.095), metal)
        _tube(lower, f"fork_crown_{i}", (x - 0.028, y, 0.150), (x + 0.028, y, 0.150), 0.008, metal)
        _tube(lower, f"fork_bridge_{i}", (x, y - 0.032, 0.150), (x, y + 0.032, 0.150), 0.008, metal)

    # Telescoping leg outer sleeves (fixed to carriage at four corners)
    leg_locations = [(-0.50, -0.25), (-0.50, 0.25), (0.50, -0.25), (0.50, 0.25)]
    sleeve_top = 0.60
    for i, (x, y) in enumerate(leg_locations):
        _tube(lower, f"leg_sleeve_{i}", (x, y, 0.30), (x, y, sleeve_top), 0.022, metal)
        _box(lower, f"leg_base_plate_{i}", (0.058, 0.058, 0.010), (x, y, 0.305), dark_metal)
        _tube(lower, f"leg_collar_{i}", (x, y, sleeve_top - 0.020), (x, y, sleeve_top), 0.026, dark_metal)
    # Lateral braces connecting left/right sleeves at front and rear
    for x in (-0.50, 0.50):
        _tube(lower, f"lat_brace_{x:+.2f}", (x, -0.25, 0.30), (x, 0.25, 0.30), 0.010, metal)

    # ── Litter frame (height-adjusting deck) ──────────────────────────────
    litter = model.part("litter_frame")
    frame_z = 0.035
    for y in (-0.315, 0.315):
        _tube(litter, f"upper_side_tube_{y:+.2f}", (-1.03, y, frame_z), (1.03, y, frame_z), 0.015, metal)
        _tube(litter, f"inner_mattress_support_{y:+.2f}", (-0.98, y * 0.72, 0.065), (0.29, y * 0.72, 0.065), 0.009, metal)
    _box(litter, "deck_pan", (1.48, 0.545, 0.014), (-0.355, 0.0, 0.078), dark_metal)
    for y in (-0.335, 0.335):
        _box(litter, f"side_rail_saddle_{y:+.2f}", (1.230, 0.040, 0.050), (-0.05, y, 0.069), metal)
    for x in (-1.03, -0.45, 0.24, 1.03):
        _tube(litter, f"upper_cross_tube_{x:+.2f}", (x, -0.315, frame_z), (x, 0.315, frame_z), 0.013, metal)
    # Vertical support brackets from litter frame down to inner tube tops
    # Each bracket seats onto the inner tube top to bridge the leg-to-litter connection.
    # A horizontal rib connects each bracket laterally to the nearest side tube.
    for i, (x, y) in enumerate(leg_locations):
        _box(litter, f"leg_support_{i}", (0.044, 0.044, 0.022), (x, y, 0.029), metal)
        y_side = -0.315 if y < 0 else 0.315
        _tube(litter, f"leg_strut_{i}", (x, y, 0.035), (x, y_side, 0.035), 0.005, metal)

    # Foot-end push hoop and release lever
    _tube(litter, "foot_push_crossbar", (-1.16, -0.235, 0.120), (-1.16, 0.235, 0.120), 0.014, metal)
    _tube(litter, "foot_push_side_0", (-1.03, -0.235, 0.055), (-1.16, -0.235, 0.120), 0.012, metal)
    _tube(litter, "foot_push_side_1", (-1.03, 0.235, 0.055), (-1.16, 0.235, 0.120), 0.012, metal)
    _tube(litter, "red_release_lever", (-1.04, -0.220, 0.045), (-1.15, -0.260, -0.055), 0.009, red)
    _tube(litter, "black_release_grip", (-1.15, -0.260, -0.055), (-1.20, -0.285, -0.085), 0.010, black)

    # Navy blue segmented pads with black restraint straps
    _box(litter, "foot_pad", (0.55, 0.525, 0.070), (-0.72, 0.0, 0.120), pad)
    _box(litter, "seat_pad", (0.60, 0.525, 0.070), (-0.100, 0.0, 0.120), pad)
    _box(litter, "foot_strap", (0.045, 0.555, 0.010), (-0.52, 0.0, 0.160), black)
    _box(litter, "seat_strap", (0.045, 0.555, 0.010), (0.090, 0.0, 0.160), black)
    _box(litter, "hinge_block", (0.060, 0.590, 0.030), (0.270, 0.0, 0.070), dark_metal)
    _box(litter, "back_hinge_lug_0", (0.080, 0.050, 0.036), (0.290, -0.295, 0.103), dark_metal)
    _box(litter, "back_hinge_lug_1", (0.080, 0.050, 0.036), (0.290, 0.295, 0.103), dark_metal)

    # Height slide: main prismatic between carriage and litter (preserved from parent)
    model.articulation(
        "height_slide",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=litter,
        origin=Origin(xyz=(0.0, 0.0, 0.670)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.18, lower=0.0, upper=0.240),
    )

    # ── Hinged backrest ───────────────────────────────────────────────────
    backrest = model.part("backrest")
    back_angle = math.radians(34.0)
    back_len = 0.78
    cx = 0.5 * back_len * math.cos(back_angle)
    cz = 0.5 * back_len * math.sin(back_angle) + 0.045
    _box(backrest, "back_pad", (back_len, 0.525, 0.075), (cx, 0.0, cz), pad, rpy=(0.0, -back_angle, 0.0))
    _box(
        backrest,
        "back_strap",
        (0.045, 0.555, 0.010),
        (0.42 * math.cos(back_angle), 0.0, 0.42 * math.sin(back_angle) + 0.092),
        black,
        rpy=(0.0, -back_angle, 0.0),
    )
    for y in (-0.305, 0.305):
        _tube(backrest, f"back_side_tube_{y:+.2f}",
              (0.02, y, 0.030),
              (back_len * math.cos(back_angle), y, back_len * math.sin(back_angle) + 0.030),
              0.012, metal)
        _tube(backrest, f"back_pad_standoff_{y:+.2f}",
              (0.04, y * 0.86, 0.060), (0.04, y, 0.035), 0.007, metal)
        _tube(
            backrest,
            f"head_standoff_{y:+.2f}",
            (back_len * math.cos(back_angle) - 0.03, y * 0.86,
             back_len * math.sin(back_angle) + 0.060),
            (back_len * math.cos(back_angle) - 0.03, y,
             back_len * math.sin(back_angle) + 0.035),
            0.007,
            metal,
        )
    _tube(
        backrest,
        "head_end_tube",
        (back_len * math.cos(back_angle), -0.305,
         back_len * math.sin(back_angle) + 0.030),
        (back_len * math.cos(back_angle), 0.305,
         back_len * math.sin(back_angle) + 0.030),
        0.012,
        metal,
    )
    _tube(backrest, "back_hinge_pin",
          (0.020, -0.305, 0.030), (0.020, 0.305, 0.030), 0.011, dark_metal)

    model.articulation(
        "backrest_hinge",
        ArticulationType.REVOLUTE,
        parent=litter,
        child=backrest,
        origin=Origin(xyz=(0.270, 0.0, 0.102)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.8, lower=-0.58, upper=0.55),
    )

    # ── Folding tubular side guard rails ──────────────────────────────────
    for idx, side in enumerate((1.0, -1.0)):
        rail = model.part(f"side_rail_{idx}")
        _tube(rail, "hinge_tube", (-0.58, 0.0, 0.0), (0.58, 0.0, 0.0), 0.011, metal)
        _tube(rail, "top_guard_tube", (-0.52, 0.0, 0.185), (0.52, 0.0, 0.185), 0.012, metal)
        for x in (-0.52, 0.0, 0.52):
            _tube(rail, f"guard_upright_{x:+.2f}", (x, 0.0, 0.0), (x, 0.0, 0.185), 0.010, metal)
        axis = (-1.0, 0.0, 0.0) if side > 0.0 else (1.0, 0.0, 0.0)
        model.articulation(
            f"side_rail_hinge_{idx}",
            ArticulationType.REVOLUTE,
            parent=litter,
            child=rail,
            origin=Origin(xyz=(-0.05, side * 0.335, 0.105)),
            axis=axis,
            motion_limits=MotionLimits(effort=25.0, velocity=1.2, lower=0.0, upper=1.55),
        )

    # ── Telescoping leg inner sliders (four vertical tube-in-tube legs) ───
    # Inner tube extends from inside the sleeve upward to contact the litter frame.
    # At rest: tube inside sleeve (z=0.32-0.60) plus visible extension (z=0.60-0.71).
    # Tube starts above the carriage side rails (z=0.30) to avoid intersection.
    inner_tube_z_start = 0.020
    inner_tube_z_end = 0.410
    for i, (x, y) in enumerate(leg_locations):
        leg = model.part(f"telescoping_leg_{i}")
        # Inner tube: slides inside outer sleeve and extends up to litter frame
        _tube(leg, "inner_tube",
              (0.0, 0.0, inner_tube_z_start), (0.0, 0.0, inner_tube_z_end),
              0.016, metal)
        model.articulation(
            f"leg_prismatic_{i}",
            ArticulationType.PRISMATIC,
            parent=lower,
            child=leg,
            origin=Origin(xyz=(x, y, 0.30)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=250.0, velocity=0.18, lower=0.0, upper=0.240),
        )

    # ── Caster wheels (four independent spin joints) ──────────────────────
    for i, (x, y) in enumerate(caster_locations):
        wheel = model.part(f"caster_{i}")
        wheel.visual(
            Cylinder(radius=0.062, length=0.058),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.030, length=0.064),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=grey,
            name="hub",
        )
        _box(wheel, "spin_marker", (0.012, 0.046, 0.006), (0.037, 0.0, 0.0), grey)
        model.articulation(
            f"caster_spin_{i}",
            ArticulationType.CONTINUOUS,
            parent=lower,
            child=wheel,
            origin=Origin(xyz=(x, y, 0.065)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=12.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_carriage")
    litter = object_model.get_part("litter_frame")
    backrest = object_model.get_part("backrest")
    side_rail = object_model.get_part("side_rail_0")
    height = object_model.get_articulation("height_slide")
    back_hinge = object_model.get_articulation("backrest_hinge")
    rail_hinge = object_model.get_articulation("side_rail_hinge_0")

    # ── Structural delta: scissor removed, telescoping legs present ───────
    scissor_removed = True
    for scissor_name in ("scissor_arm_0", "scissor_arm_1"):
        try:
            object_model.get_part(scissor_name)
            scissor_removed = False
        except Exception:
            pass
    ctx.check(
        "scissor linkage removed in telescoping variant",
        scissor_removed,
    )

    # Required subassemblies (telescoping legs replace scissor arms)
    required = [
        "lower_carriage",
        "litter_frame",
        "backrest",
        "side_rail_0",
        "side_rail_1",
        "telescoping_leg_0",
        "telescoping_leg_1",
        "telescoping_leg_2",
        "telescoping_leg_3",
        "caster_0",
        "caster_1",
        "caster_2",
        "caster_3",
    ]
    ctx.check(
        "stretcher subassemblies present",
        all(object_model.get_part(name) for name in required),
    )

    # ── Telescoping leg prismatic joints ──────────────────────────────────
    leg_joints = [object_model.get_articulation(f"leg_prismatic_{i}") for i in range(4)]
    ctx.check(
        "four telescoping leg prismatic joints with +Z axis",
        all(
            j.articulation_type == ArticulationType.PRISMATIC
            and tuple(j.axis) == (0.0, 0.0, 1.0)
            for j in leg_joints
        ),
    )

    # Allow intentional telescoping overlap (inner tube inside outer sleeve and collar)
    legs = [object_model.get_part(f"telescoping_leg_{i}") for i in range(4)]
    for i, leg in enumerate(legs):
        ctx.allow_overlap(
            lower,
            leg,
            elem_a=f"leg_sleeve_{i}",
            elem_b="inner_tube",
            reason=f"Inner tube telescopes inside outer sleeve for height leg {i}.",
        )
        ctx.allow_overlap(
            lower,
            leg,
            elem_a=f"leg_collar_{i}",
            elem_b="inner_tube",
            reason=f"Collar bushing wraps the inner tube at sleeve top for leg {i}.",
        )
        ctx.allow_overlap(
            litter,
            leg,
            elem_a=f"leg_support_{i}",
            elem_b="inner_tube",
            reason=f"Leg support bracket seats onto the inner tube top for leg {i}.",
        )
        ctx.allow_overlap(
            litter,
            leg,
            elem_a=f"leg_strut_{i}",
            elem_b="inner_tube",
            reason=f"Connecting strut passes alongside the inner tube at the leg-to-litter junction for leg {i}.",
        )

    # Prove inner tubes stay centered in outer sleeves (XY containment)
    # This also proves the collar/tube and sleeve/tube containment.
    for i, leg in enumerate(legs):
        ctx.expect_within(
            leg,
            lower,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem=f"leg_sleeve_{i}",
            margin=0.008,
            name=f"leg {i} inner tube stays centered in outer sleeve",
        )
        # Prove leg support bracket overlaps inner tube on Z (seated connection)
        ctx.expect_overlap(
            litter,
            leg,
            axes="z",
            elem_a=f"leg_support_{i}",
            elem_b="inner_tube",
            min_overlap=0.005,
            name=f"leg {i} support bracket seats onto inner tube",
        )

    # Leg prismatic extends the leg upward (decisive pose check for leg_prismatic_0)
    leg_prismatic_0 = object_model.get_articulation("leg_prismatic_0")
    rest_leg_pos = ctx.part_world_position(legs[0])
    with ctx.pose({leg_prismatic_0: 0.24}):
        extended_leg_pos = ctx.part_world_position(legs[0])
    ctx.check(
        "telescoping_leg_0 extends upward via leg_prismatic_0",
        rest_leg_pos is not None
        and extended_leg_pos is not None
        and extended_leg_pos[2] > rest_leg_pos[2] + 0.20,
        details=f"rest={rest_leg_pos}, extended={extended_leg_pos}",
    )

    # ── Litter and carriage relationship ──────────────────────────────────
    ctx.expect_overlap(
        litter, lower, axes="xy", min_overlap=0.45,
        name="litter frame stays above wheeled carriage footprint",
    )
    ctx.expect_overlap(
        backrest, litter, axes="y", min_overlap=0.40,
        name="backrest spans the mattress width",
    )

    # Height adjustment raises the litter deck
    rest_litter_pos = ctx.part_world_position(litter)
    with ctx.pose({height: 0.24}):
        raised_litter_pos = ctx.part_world_position(litter)
    ctx.check(
        "height_slide raises the litter deck",
        rest_litter_pos is not None
        and raised_litter_pos is not None
        and raised_litter_pos[2] > rest_litter_pos[2] + 0.20,
        details=f"rest={rest_litter_pos}, raised={raised_litter_pos}",
    )

    # ── Backrest hinge ────────────────────────────────────────────────────
    raised_back_aabb = ctx.part_world_aabb(backrest)
    with ctx.pose({back_hinge: -0.55}):
        lowered_back_aabb = ctx.part_world_aabb(backrest)
    ctx.check(
        "hinged backrest lowers from the reference raised pose",
        raised_back_aabb is not None
        and lowered_back_aabb is not None
        and ((lowered_back_aabb[0][2] + lowered_back_aabb[1][2]) * 0.5)
        < ((raised_back_aabb[0][2] + raised_back_aabb[1][2]) * 0.5) - 0.18,
        details=f"raised={raised_back_aabb}, lowered={lowered_back_aabb}",
    )

    # ── Side rail folding ─────────────────────────────────────────────────
    upright_rail_aabb = ctx.part_world_aabb(side_rail)
    with ctx.pose({rail_hinge: 1.45}):
        folded_rail_aabb = ctx.part_world_aabb(side_rail)
    ctx.check(
        "folding side rail drops outward",
        upright_rail_aabb is not None
        and folded_rail_aabb is not None
        and folded_rail_aabb[1][2] < upright_rail_aabb[1][2] - 0.10
        and folded_rail_aabb[1][1] > upright_rail_aabb[1][1] + 0.10,
        details=f"upright={upright_rail_aabb}, folded={folded_rail_aabb}",
    )

    # ── Caster spin joints ────────────────────────────────────────────────
    caster_joints = [object_model.get_articulation(f"caster_spin_{i}") for i in range(4)]
    ctx.check(
        "all four caster wheels have spin joints",
        all(
            j.articulation_type == ArticulationType.CONTINUOUS
            and tuple(j.axis) == (0.0, 1.0, 0.0)
            for j in caster_joints
        ),
    )

    return ctx.report()


object_model = build_object_model()
