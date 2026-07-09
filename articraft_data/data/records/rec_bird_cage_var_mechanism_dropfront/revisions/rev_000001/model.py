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
            "description": "Tall black bird cage with barrel-vault roof, cut-out front access door, latch, perches, tray, and casters.",
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
    # the hinged door so the closed door reads as seated tightly into the cage
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

    # A small strike plate for the latch is mounted at the top center of the
    # door cut-out, just above the closed door's top edge so the drop-front
    # latch tab can engage when the door is raised into the closed position.
    cage.visual(
        Box((0.040, 0.006, 0.030)),
        origin=Origin(xyz=(0.0, front_y - 0.010, door_top_z + 0.012)),
        material=BLACK,
        name="latch_strike",
    )
    _add_rod(
        cage,
        "strike_standoff",
        (0.0, front_y - 0.010, door_top_z + 0.012),
        (0.0, front_y, door_top_z + 0.012),
        radius=0.0035,
    )
    # Short bracket ties the strike standoff to the nearest front-face vertical
    # wire so the strike plate reads as mounted to the cage grid.
    _add_rod(
        cage,
        "strike_bracket",
        (0.0, front_y, door_top_z + 0.012),
        (0.0, front_y, door_top_z + 0.048),
        radius=0.0035,
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

    # Door child frame sits at the center of the bottom sill (the hinge line).
    # Its closed q=0 geometry lies exactly in the front face, filling the
    # cut-out rather than sitting over duplicate cage wires. The door extends
    # upward from the sill in local +Z and is symmetric about local X=0.
    door = model.part("door")
    door_h = door_spring_rel
    half_dw = door_w / 2.0

    # Arched frame: bottom sill, two vertical sides, and arched top.
    _add_rod(door, "door_left_frame", (-half_dw, 0.0, 0.0), (-half_dw, 0.0, door_h), radius=rail_r)
    _add_rod(door, "door_right_frame", (half_dw, 0.0, 0.0), (half_dw, 0.0, door_h), radius=rail_r)
    _add_rod(door, "door_bottom_frame", (-half_dw, 0.0, 0.0), (half_dw, 0.0, 0.0), radius=rail_r)
    _add_arch(door, "door_arch_frame", center_x=0.0, y=0.0, spring_z=door_h, radius=half_dw, segments=14, rod_radius=rail_r)

    # Internal vertical bars (centered x positions). Start slightly above the
    # bottom frame so they clear the thicker fixed surround rail at the sill.
    for i, cx in enumerate([-0.120, -0.060, 0.0, 0.060, 0.120]):
        local_x = cx + half_dw
        _add_rod(door, f"door_vertical_bar_{i}", (cx, 0.0, 0.010), (cx, 0.0, _door_arch_z(local_x, door_w, door_h)), radius=rod_r)

    # Internal horizontal bars.
    for i, lz in enumerate([0.285, 0.520]):
        _add_rod(door, f"door_horizontal_bar_{i}", (-half_dw + 0.020, 0.0, lz), (half_dw - 0.020, 0.0, lz), radius=rod_r)

    # Bottom hinge barrels run horizontally along the sill, proud of the front
    # plane, with small mounting leaves tying them back to the door bottom frame.
    for i, xc in enumerate([-0.090, 0.090]):
        _add_rod(door, f"door_hinge_barrel_{i}", (xc - 0.060, -0.018, 0.0), (xc + 0.060, -0.018, 0.0), radius=0.011)
        door.visual(
            Box((0.018, 0.020, 0.020)),
            origin=Origin(xyz=(xc, -0.009, 0.004)),
            material=BLACK,
            name=f"door_hinge_leaf_{i}",
        )

    # Latch boss at the top center of the door (the free edge that drops down).
    latch_z = door_h + half_dw - 0.020
    door.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.0, -0.004, latch_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=BLACK,
        name="door_latch_boss",
    )

    # Bottom-hinged drop-front: the revolute axis runs along X at the sill
    # center. Positive q rotates the top edge outward (-Y) and downward,
    # dropping the door into a horizontal landing platform.
    door_hinge = model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cage,
        child=door,
        origin=Origin(xyz=(0.0, front_y, door_bottom)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.57),
    )

    # Latch at the top center of the door (free edge). The pivot disc sits on
    # the door outer face; the tab swings about Y to engage the cage strike.
    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.017, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=BLACK,
        name="latch_pivot_disc",
    )
    latch.visual(Box((0.082, 0.008, 0.017)), origin=Origin(xyz=(0.046, 0.0, 0.0)), material=BLACK, name="latch_tab")
    latch.visual(
        Cylinder(radius=0.004, length=0.032),
        origin=Origin(xyz=(0.0, 0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=BLACK,
        name="latch_spindle",
    )
    model.articulation(
        "latch_pivot",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch,
        origin=Origin(xyz=(0.0, -0.030, latch_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=3.0, lower=0.0, upper=1.57),
    )

    # Store key dimensions for semantic tests without coupling to every wire name.
    model.meta["door_hinge_name"] = door_hinge.name
    model.meta["front_y"] = front_y
    model.meta["door_bottom"] = door_bottom
    model.meta["door_x_min"] = door_x_min
    model.meta["door_x_max"] = door_x_max
    model.meta["door_top_z"] = door_top_z
    model.meta["surround_x_min"] = surround_x_min
    model.meta["surround_x_max"] = surround_x_max
    model.meta["surround_top_z"] = surround_top_z
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cage = object_model.get_part("cage")
    door = object_model.get_part("door")
    latch = object_model.get_part("latch")
    door_hinge = object_model.get_articulation("door_hinge")
    latch_pivot = object_model.get_articulation("latch_pivot")

    ctx.allow_overlap(
        door,
        latch,
        elem_a="door_latch_boss",
        elem_b="latch_spindle",
        reason="The rotating latch spindle is intentionally captured through the small boss on the door frame.",
    )
    ctx.expect_overlap(
        door,
        latch,
        axes="y",
        elem_a="door_latch_boss",
        elem_b="latch_spindle",
        min_overlap=0.003,
        name="latch spindle is retained in door boss",
    )

    # The closed door's frame rails sit flush against the fixed cage surround
    # frame at the sill and corners — these small seated contact overlaps are
    # intentional for a drop-front door that folds flush into the aperture.
    ctx.allow_overlap(
        cage,
        door,
        elem_a="fixed_door_frame_bottom",
        elem_b="door_bottom_frame",
        reason="The closed door bottom rail is seated against the fixed cage sill rail for flush closure.",
    )
    ctx.allow_overlap(
        cage,
        door,
        elem_a="fixed_door_frame_bottom",
        elem_b="door_left_frame",
        reason="The door left frame corner meets the cage sill surround at the aperture edge.",
    )
    ctx.allow_overlap(
        cage,
        door,
        elem_a="fixed_door_frame_bottom",
        elem_b="door_right_frame",
        reason="The door right frame corner meets the cage sill surround at the aperture edge.",
    )
    ctx.expect_contact(
        cage,
        door,
        elem_a="fixed_door_frame_bottom",
        elem_b="door_bottom_frame",
        contact_tol=0.012,
        name="closed door bottom frame contacts cage sill frame",
    )

    # Prompt-specific structure: one root cage, bottom-hinged drop-front door,
    # and a separately rotating latch at the door top edge.
    ctx.check("door_hinge is revolute", door_hinge.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("latch_pivot is revolute", latch_pivot.articulation_type == ArticulationType.REVOLUTE)

    # Drop-front axis claim: the door_hinge axis must be horizontal (along X),
    # not vertical — this is the primary mechanism change from side-swing to
    # bottom-hinged drop-front.
    hinge_axis = door_hinge.axis
    ctx.check(
        "door_hinge axis is horizontal along X (drop-front)",
        hinge_axis is not None and abs(hinge_axis[0]) > 0.9 and abs(hinge_axis[2]) < 0.1,
        details=f"hinge_axis={hinge_axis}",
    )

    # The door sits flush with the front plane when closed.
    front_y = object_model.meta["front_y"]
    door_closed_pos = ctx.part_world_position(door)
    ctx.check(
        "closed door lies on front face",
        door_closed_pos is not None and abs(door_closed_pos[1] - front_y) < 0.05,
        details=f"door_pos={door_closed_pos}, front_y={front_y}",
    )

    # The closed door panel occupies the cut-out footprint.
    ctx.expect_overlap(door, cage, axes="xz", min_overlap=0.25, name="closed door fills cage front footprint")
    door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "fixed cage door surround is slightly larger than hinged door",
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

    # Drop-front visible geometry claim: opening the door drops its top edge
    # downward and outward from the cage front, creating a horizontal platform.
    rest_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.40}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door_hinge drops top edge downward when opened (drop-front platform)",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] < rest_aabb[1][2] - 0.10,
        details=f"rest_top_z={rest_aabb[1][2] if rest_aabb else None}, open_top_z={open_aabb[1][2] if open_aabb else None}",
    )
    ctx.check(
        "door extends outward from cage front when dropped",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < rest_aabb[0][1] - 0.08,
        details=f"rest_min_y={rest_aabb[0][1] if rest_aabb else None}, open_min_y={open_aabb[0][1] if open_aabb else None}",
    )

    # Latch tab rotates about its pivot.
    latch_rest = ctx.part_element_world_aabb(latch, elem="latch_tab")
    with ctx.pose({latch_pivot: 1.20}):
        latch_turned = ctx.part_element_world_aabb(latch, elem="latch_tab")
    ctx.check(
        "latch tab rotates about pivot",
        latch_rest is not None and latch_turned is not None and abs(latch_turned[0][2] - latch_rest[0][2]) > 0.015,
        details=f"rest={latch_rest}, turned={latch_turned}",
    )

    return ctx.report()


object_model = build_object_model()
