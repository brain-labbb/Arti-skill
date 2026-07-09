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


BLACK = Material("black_powder_coated_metal", rgba=(0.005, 0.005, 0.004, 1.0))
DARK_TRAY = Material("satin_black_tray", rgba=(0.02, 0.018, 0.015, 1.0))
WOOD = Material("light_wood_perch", rgba=(0.72, 0.54, 0.34, 1.0))
RUBBER = Material("dark_rubber_wheel", rgba=(0.015, 0.014, 0.013, 1.0))
STEEL = Material("caster_steel", rgba=(0.45, 0.45, 0.42, 1.0))


def _rod_rpy_for_delta(dx: float, dy: float, dz: float) -> tuple[float, float, float]:
    """Return an Origin.rpy that turns a local-Z cylinder into the segment direction."""
    if abs(dy) < 1e-9:
        return (0.0, math.atan2(dx, dz), 0.0)
    if abs(dx) < 1e-9 and abs(dz) < 1e-9:
        return (math.pi / 2.0, 0.0, 0.0)

    # General fallback: yaw toward XY direction, then pitch local Z toward the
    # full 3-D vector. The model mostly uses axis-aligned and XZ arch segments.
    yaw = math.atan2(dy, dx)
    horiz = math.hypot(dx, dy)
    pitch = math.atan2(horiz, dz)
    return (0.0, pitch, yaw)


def _add_rod(
    part,
    name: str,
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    *,
    radius: float = 0.0045,
    material: Material = BLACK,
    end_pad: float = 0.0015,
) -> None:
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1e-6:
        return
    part.visual(
        Cylinder(radius=radius, length=length + 2.0 * end_pad),
        origin=Origin(
            xyz=((p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0, (p0[2] + p1[2]) / 2.0),
            rpy=_rod_rpy_for_delta(dx, dy, dz),
        ),
        material=material,
        name=name,
    )


def _add_arch(
    part,
    prefix: str,
    *,
    center_x: float,
    y: float,
    spring_z: float,
    radius: float,
    segments: int = 18,
    rod_radius: float = 0.0055,
    material: Material = BLACK,
    local: bool = False,
) -> None:
    """Half-round arch in the XZ plane from left spring to right spring."""
    last = None
    for i in range(segments + 1):
        theta = math.pi - (math.pi * i / segments)
        p = (center_x + radius * math.cos(theta), y, spring_z + radius * math.sin(theta))
        if last is not None:
            _add_rod(part, f"{prefix}_{i:02d}", last, p, radius=rod_radius, material=material)
        last = p


def _door_arch_z(local_x: float, door_width: float, spring_rel_z: float) -> float:
    r = door_width / 2.0
    return spring_rel_z + math.sqrt(max(0.0, r * r - (local_x - r) * (local_x - r)))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="tall_black_bird_cage",
        materials=[BLACK, DARK_TRAY, WOOD, RUBBER, STEEL],
        meta={
            "description": "Tall black bird cage with barrel-vault roof, guillotine sliding front access door, perches, tray, and casters.",
        },
    )

    width = 0.76
    depth = 0.52
    half_w = width / 2.0
    half_d = depth / 2.0
    front_y = -half_d
    back_y = half_d
    bottom_z = 0.23
    spring_z = 1.43
    roof_r = half_w
    top_z = spring_z + roof_r
    rod_r = 0.0045
    rail_r = 0.006

    door_w = 0.36
    door_bottom = 0.39
    door_x_min = -door_w / 2.0
    door_x_max = door_w / 2.0
    door_spring_rel = 0.59
    door_spring_z = door_bottom + door_spring_rel
    door_top_z = door_spring_z + door_w / 2.0
    door_frame_clearance = 0.022
    surround_w = door_w + 2.0 * door_frame_clearance
    surround_x_min = door_x_min - door_frame_clearance
    surround_x_max = door_x_max + door_frame_clearance
    surround_spring_z = door_spring_z + 0.014
    surround_top_z = surround_spring_z + surround_w / 2.0

    # Guide rail dimensions for the guillotine sliding door.
    guide_rail_width = 0.018
    guide_rail_depth = 0.022
    guide_z_bottom = door_bottom - 0.04
    guide_z_top = door_top_z + 0.48
    guide_h = guide_z_top - guide_z_bottom
    guide_z_mid = (guide_z_bottom + guide_z_top) / 2.0

    # Prismatic travel limit: door slides up this many meters from closed.
    slide_upper = 0.45

    cage = model.part("cage")

    # Rectangular slide-out tray and raised lip.
    cage.visual(Box((0.88, 0.64, 0.075)), origin=Origin(xyz=(0.0, 0.0, 0.105)), material=DARK_TRAY, name="base_tray")
    cage.visual(Box((0.92, 0.045, 0.075)), origin=Origin(xyz=(0.0, -0.342, 0.165)), material=DARK_TRAY, name="front_tray_lip")
    cage.visual(Box((0.92, 0.045, 0.075)), origin=Origin(xyz=(0.0, 0.342, 0.165)), material=DARK_TRAY, name="rear_tray_lip")
    cage.visual(Box((0.045, 0.64, 0.075)), origin=Origin(xyz=(-0.462, 0.0, 0.165)), material=DARK_TRAY, name="side_tray_lip_0")
    cage.visual(Box((0.045, 0.64, 0.075)), origin=Origin(xyz=(0.462, 0.0, 0.165)), material=DARK_TRAY, name="side_tray_lip_1")

    # Four small casters under the tray, each with a stem, fork, and wheel.
    for ix, x in enumerate((-0.36, 0.36)):
        for iy, y in enumerate((-0.255, 0.255)):
            suffix = f"{ix}_{iy}"
            _add_rod(cage, f"caster_stem_{suffix}", (x, y, 0.034), (x, y, 0.074), radius=0.006, material=STEEL)
            cage.visual(Box((0.040, 0.014, 0.050)), origin=Origin(xyz=(x - 0.020, y, 0.043)), material=STEEL, name=f"caster_fork_a_{suffix}")
            cage.visual(Box((0.040, 0.014, 0.050)), origin=Origin(xyz=(x + 0.020, y, 0.043)), material=STEEL, name=f"caster_fork_b_{suffix}")
            cage.visual(
                Cylinder(radius=0.030, length=0.020),
                origin=Origin(xyz=(x, y, 0.030), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=RUBBER,
                name=f"caster_wheel_{suffix}",
            )

    # Bottom and top rectangular rails.
    for z, label, rr in ((bottom_z, "bottom", rail_r), (spring_z, "spring", rail_r)):
        _add_rod(cage, f"{label}_front_rail", (-half_w, front_y, z), (half_w, front_y, z), radius=rr)
        _add_rod(cage, f"{label}_rear_rail", (-half_w, back_y, z), (half_w, back_y, z), radius=rr)
        _add_rod(cage, f"{label}_side_rail_0", (-half_w, front_y, z), (-half_w, back_y, z), radius=rr)
        _add_rod(cage, f"{label}_side_rail_1", (half_w, front_y, z), (half_w, back_y, z), radius=rr)

    # Corner posts.
    for ix, x in enumerate((-half_w, half_w)):
        for iy, y in enumerate((front_y, back_y)):
            _add_rod(cage, f"corner_post_{ix}_{iy}", (x, y, bottom_z), (x, y, spring_z), radius=rail_r)

    # Side wall straight vertical wires and horizontal rails.
    side_y_values = [front_y + i * (depth / 8.0) for i in range(9)]
    side_z_values = [0.39, 0.68, 0.97, 1.22, spring_z]
    for side_i, x in enumerate((-half_w, half_w)):
        for i, y in enumerate(side_y_values):
            _add_rod(cage, f"side_vertical_{side_i}_{i}", (x, y, bottom_z), (x, y, spring_z), radius=rod_r)
        for i, z in enumerate(side_z_values):
            _add_rod(cage, f"side_horizontal_{side_i}_{i}", (x, front_y, z), (x, back_y, z), radius=rod_r)

    # Back wall grid.
    back_x_values = [-0.33 + i * 0.055 for i in range(13)]
    for i, x in enumerate(back_x_values):
        _add_rod(cage, f"rear_vertical_{i}", (x, back_y, bottom_z), (x, back_y, spring_z), radius=rod_r)
    for i, z in enumerate([0.36, 0.62, 0.88, 1.14, spring_z]):
        _add_rod(cage, f"rear_horizontal_{i}", (-half_w, back_y, z), (half_w, back_y, z), radius=rod_r)

    # Front face grid. The arched door is a true cut-out: central front wires stop
    # at the door sill, restart only above the arched opening, and never continue
    # behind the closed door panel.
    front_x_values = [-0.33 + i * 0.055 for i in range(13)]
    for i, x in enumerate(front_x_values):
        if door_x_min - 0.006 < x < door_x_max + 0.006:
            _add_rod(cage, f"front_lower_short_{i}", (x, front_y, bottom_z), (x, front_y, door_bottom - 0.018), radius=rod_r)
            arch_start = door_bottom + _door_arch_z(x - door_x_min, door_w, door_spring_rel) - 0.002
            _add_rod(cage, f"front_upper_short_{i}", (x, front_y, arch_start + 0.050), (x, front_y, spring_z), radius=rod_r)
        else:
            _add_rod(cage, f"front_vertical_{i}", (x, front_y, bottom_z), (x, front_y, spring_z), radius=rod_r)

    _add_rod(cage, "front_sill_left_rail", (-half_w, front_y, door_bottom), (door_x_min - 0.020, front_y, door_bottom), radius=rail_r)
    _add_rod(cage, "front_sill_right_rail", (door_x_max + 0.020, front_y, door_bottom), (half_w, front_y, door_bottom), radius=rail_r)
    for i, z in enumerate([0.68, 0.97]):
        _add_rod(cage, f"front_left_split_rail_{i}", (-half_w, front_y, z), (door_x_min - 0.020, front_y, z), radius=rod_r)
        _add_rod(cage, f"front_right_split_rail_{i}", (door_x_max + 0.020, front_y, z), (half_w, front_y, z), radius=rod_r)
    for i, z in enumerate([1.24, spring_z]):
        _add_rod(cage, f"front_upper_full_rail_{i}", (-half_w, front_y, z), (half_w, front_y, z), radius=rod_r)

    # Fixed arched door surround on the cage face. It is slightly larger than
    # the sliding door so the closed door reads as seated tightly into the cage
    # rather than floating over the front grid.
    surround_y = front_y + 0.004
    surround_r = rail_r * 1.2
    _add_rod(
        cage,
        "fixed_door_frame_left",
        (surround_x_min, surround_y, door_bottom),
        (surround_x_min, surround_y, surround_spring_z),
        radius=surround_r,
    )
    _add_rod(
        cage,
        "fixed_door_frame_right",
        (surround_x_max, surround_y, door_bottom),
        (surround_x_max, surround_y, surround_spring_z),
        radius=surround_r,
    )
    _add_rod(
        cage,
        "fixed_door_frame_bottom",
        (surround_x_min, surround_y, door_bottom),
        (surround_x_max, surround_y, door_bottom),
        radius=surround_r,
    )
    _add_arch(
        cage,
        "fixed_door_frame_arch",
        center_x=(surround_x_min + surround_x_max) / 2.0,
        y=surround_y,
        spring_z=surround_spring_z,
        radius=surround_w / 2.0,
        segments=14,
        rod_radius=surround_r,
    )

    # Guide rail channels for the guillotine sliding door. Two tall box-section
    # rails flanking the aperture capture the door edges and lugs as the panel
    # slides vertically.
    cage.visual(
        Box((guide_rail_width, guide_rail_depth, guide_h)),
        origin=Origin(xyz=(door_x_min, front_y - 0.004, guide_z_mid)),
        material=BLACK,
        name="left_guide_rail",
    )
    cage.visual(
        Box((guide_rail_width, guide_rail_depth, guide_h)),
        origin=Origin(xyz=(door_x_max, front_y - 0.004, guide_z_mid)),
        material=BLACK,
        name="right_guide_rail",
    )

    # Small stop pin on the right guide rail prevents the door from sliding
    # below the closed position. The door bottom frame rests on this pin at q=0.
    # The pin starts from the guide rail surface so it reads as mechanically attached.
    _add_rod(
        cage,
        "door_stop_pin",
        (door_x_max, front_y - 0.015, door_bottom - 0.006),
        (door_x_max + 0.024, front_y - 0.015, door_bottom - 0.006),
        radius=0.004,
    )

    # Barrel-vault roof: transverse arch ribs and longitudinal roof rods.
    roof_y_values = [front_y, -0.13, 0.0, 0.13, back_y]
    for i, y in enumerate(roof_y_values):
        _add_arch(cage, f"roof_arch_rib_{i}", center_x=0.0, y=y, spring_z=spring_z, radius=roof_r, rod_radius=rail_r)
    for i, deg in enumerate([18, 36, 54, 72, 90, 108, 126, 144, 162]):
        theta = math.radians(deg)
        x = roof_r * math.cos(theta)
        z = spring_z + roof_r * math.sin(theta)
        _add_rod(cage, f"roof_longitudinal_{i}", (x, front_y, z), (x, back_y, z), radius=rod_r)
    for face_i, y in enumerate((front_y, back_y)):
        for i, x in enumerate([-0.275, -0.165, -0.055, 0.055, 0.165, 0.275]):
            z_to = spring_z + math.sqrt(max(0.0, roof_r * roof_r - x * x))
            _add_rod(cage, f"gable_vertical_{face_i}_{i}", (x, y, spring_z), (x, y, z_to), radius=rod_r)
        for i, z in enumerate([1.54, 1.65]):
            x_span = math.sqrt(max(0.0, roof_r * roof_r - (z - spring_z) * (z - spring_z)))
            _add_rod(cage, f"gable_horizontal_{face_i}_{i}", (-x_span, y, z), (x_span, y, z), radius=rod_r)

    # Two pale wood perches spanning the cage interior and seated into the side wires.
    _add_rod(cage, "lower_wood_perch", (-half_w - 0.006, -0.070, 0.735), (half_w + 0.006, -0.070, 0.735), radius=0.014, material=WOOD)
    _add_rod(cage, "upper_wood_perch", (-half_w - 0.006, 0.065, 1.095), (half_w + 0.006, 0.065, 1.095), radius=0.014, material=WOOD)

    # Tray-to-cage metal risers make the wire body visibly bolted to the base tray.
    for ix, x in enumerate((-half_w, half_w)):
        for iy, y in enumerate((front_y, back_y)):
            _add_rod(cage, f"tray_riser_{ix}_{iy}", (x, y, 0.135), (x, y, bottom_z + 0.008), radius=rail_r)

    # Guillotine sliding door. The door part frame sits at the bottom-left corner
    # of the panel; its closed q=0 geometry fills the front-face cut-out exactly,
    # with no duplicate cage wires behind the closed panel.
    door = model.part("door")
    door_h = door_spring_rel
    # Arched frame, bottom sill, and internal vertical bars.
    _add_rod(door, "door_left_frame", (0.0, 0.0, 0.0), (0.0, 0.0, door_h), radius=rail_r)
    _add_rod(door, "door_right_frame", (door_w, 0.0, 0.0), (door_w, 0.0, door_h), radius=rail_r)
    _add_rod(door, "door_bottom_frame", (0.0, 0.0, 0.0), (door_w, 0.0, 0.0), radius=rail_r)
    _add_arch(door, "door_arch_frame", center_x=door_w / 2.0, y=0.0, spring_z=door_h, radius=door_w / 2.0, segments=14, rod_radius=rail_r)
    for i, lx in enumerate([0.060, 0.120, 0.180, 0.240, 0.300]):
        _add_rod(door, f"door_vertical_bar_{i}", (lx, 0.0, 0.0), (lx, 0.0, _door_arch_z(lx, door_w, door_h)), radius=rod_r)
    for i, lz in enumerate([0.285, 0.520]):
        _add_rod(door, f"door_horizontal_bar_{i}", (0.020, 0.0, lz), (door_w - 0.020, 0.0, lz), radius=rod_r)

    # Side lugs on the door edges ride inside the guide rail channels. Two lugs
    # per side (bottom and top) keep the panel captured and prevent twisting.
    lug_x = 0.010
    lug_y = 0.018
    lug_z = 0.030
    lug_positions = [
        (-lug_x / 2.0, 0.060),            # bottom-left
        (door_w + lug_x / 2.0, 0.060),    # bottom-right
        (-lug_x / 2.0, door_h - 0.060),   # top-left
        (door_w + lug_x / 2.0, door_h - 0.060),  # top-right
    ]
    for i, (lx, lz) in enumerate(lug_positions):
        door.visual(
            Box((lug_x, lug_y, lug_z)),
            origin=Origin(xyz=(lx, -0.003, lz)),
            material=BLACK,
            name=f"door_lug_{i}",
        )

    # Prismatic joint: door slides vertically upward along +Z within the guide rails.
    door_slide = model.articulation(
        "door_slide",
        ArticulationType.PRISMATIC,
        parent=cage,
        child=door,
        origin=Origin(xyz=(door_x_min, front_y, door_bottom)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.5, lower=0.0, upper=slide_upper),
    )

    # Store key dimensions for semantic tests without coupling to every wire name.
    model.meta["door_slide_name"] = door_slide.name
    model.meta["front_y"] = front_y
    model.meta["door_bottom"] = door_bottom
    model.meta["door_x_min"] = door_x_min
    model.meta["door_x_max"] = door_x_max
    model.meta["door_top_z"] = door_top_z
    model.meta["surround_x_min"] = surround_x_min
    model.meta["surround_x_max"] = surround_x_max
    model.meta["surround_top_z"] = surround_top_z
    model.meta["slide_upper"] = slide_upper
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cage = object_model.get_part("cage")
    door = object_model.get_part("door")
    door_slide = object_model.get_articulation("door_slide")

    # The door's frame rods, arch segments, and side lugs are intentionally
    # captured inside the box-section guide rails so the panel can slide
    # vertically. Each allowance is scoped to the exact element pair.
    left_rail_elems = ["door_left_frame", "door_bottom_frame", "door_arch_frame_01", "door_arch_frame_02", "door_lug_0", "door_lug_2"]
    right_rail_elems = ["door_right_frame", "door_bottom_frame", "door_arch_frame_13", "door_arch_frame_14", "door_lug_1", "door_lug_3"]
    for elem_b in left_rail_elems:
        ctx.allow_overlap(
            cage, door,
            elem_a="left_guide_rail", elem_b=elem_b,
            reason=f"Door element {elem_b} is intentionally captured within the left guide rail channel for vertical sliding.",
        )
    for elem_b in right_rail_elems:
        ctx.allow_overlap(
            cage, door,
            elem_a="right_guide_rail", elem_b=elem_b,
            reason=f"Door element {elem_b} is intentionally captured within the right guide rail channel for vertical sliding.",
        )
    # The fixed door frame bottom sill contacts the door bottom frame and
    # vertical bars when the door is closed at q=0. This is the seated contact
    # where the guillotine door rests on its sill.
    sill_elems = ["door_left_frame", "door_right_frame", "door_bottom_frame",
                  "door_vertical_bar_0", "door_vertical_bar_1", "door_vertical_bar_2",
                  "door_vertical_bar_3", "door_vertical_bar_4"]
    for elem_b in sill_elems:
        ctx.allow_overlap(
            cage, door,
            elem_a="fixed_door_frame_bottom", elem_b=elem_b,
            reason=f"Door element {elem_b} is intentionally seated on the fixed door frame bottom sill at closed position.",
        )

    # Prompt-specific structure: prismatic guillotine door replacing the parent
    # revolute hinge, with guide rails and no swing latch.
    ctx.check(
        "door_slide is prismatic",
        door_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"got {door_slide.articulation_type}",
    )

    # The closed door panel occupies the cut-out footprint, proving it is a
    # replacement panel rather than a decorative overlay.
    ctx.expect_overlap(door, cage, axes="xz", min_overlap=0.25, name="closed door fills cage front footprint")

    # The fixed cage door surround is slightly larger than the sliding door.
    door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "fixed cage door surround is slightly larger than sliding door",
        door_aabb is not None
        and object_model.meta["surround_x_min"] < door_aabb[0][0]
        and object_model.meta["surround_x_max"] > door_aabb[1][0]
        and object_model.meta["surround_top_z"] > door_aabb[1][2],
        details=(
            f"door_aabb={door_aabb}, "
            f"surround_x=({object_model.meta['surround_x_min']}, {object_model.meta['surround_x_max']}), "
            f"surround_top_z={object_model.meta['surround_top_z']}"
        ),
    )

    # Prove the door edges are captured within the guide rails on the X axis.
    ctx.expect_within(
        door,
        cage,
        axes="x",
        inner_elem="door_left_frame",
        outer_elem="left_guide_rail",
        margin=0.005,
        name="door left edge is captured within left guide rail",
    )
    ctx.expect_within(
        door,
        cage,
        axes="x",
        inner_elem="door_right_frame",
        outer_elem="right_guide_rail",
        margin=0.005,
        name="door right edge is captured within right guide rail",
    )

    # Prove the door actually slides upward along +Z when articulated.
    rest_pos = ctx.part_world_position(door)
    slide_upper = object_model.meta["slide_upper"]
    with ctx.pose({door_slide: slide_upper}):
        open_pos = ctx.part_world_position(door)
    ctx.check(
        "door_slide moves door upward along +Z",
        rest_pos is not None
        and open_pos is not None
        and open_pos[2] > rest_pos[2] + 0.10,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    # Prove the door stays in the front plane when sliding (no lateral drift).
    ctx.check(
        "door stays in front plane when sliding",
        rest_pos is not None
        and open_pos is not None
        and abs(open_pos[1] - rest_pos[1]) < 0.002
        and abs(open_pos[0] - rest_pos[0]) < 0.002,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    # At maximum extension, the door bottom is above the aperture sill, proving
    # the opening is exposed.
    with ctx.pose({door_slide: slide_upper}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "fully open door exposes aperture below door bottom",
        open_aabb is not None and open_aabb[0][2] > object_model.meta["door_bottom"] + 0.10,
        details=f"open_aabb={open_aabb}, door_bottom={object_model.meta['door_bottom']}",
    )

    return ctx.report()


object_model = build_object_model()
