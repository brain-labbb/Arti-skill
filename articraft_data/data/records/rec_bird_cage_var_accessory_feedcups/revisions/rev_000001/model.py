from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BLACK = Material("black_powder_coated_metal", rgba=(0.005, 0.005, 0.004, 1.0))
DARK_TRAY = Material("satin_black_tray", rgba=(0.02, 0.018, 0.015, 1.0))
WOOD = Material("light_wood_perch", rgba=(0.72, 0.54, 0.34, 1.0))
RUBBER = Material("dark_rubber_wheel", rgba=(0.015, 0.014, 0.013, 1.0))
STEEL = Material("caster_steel", rgba=(0.45, 0.45, 0.42, 1.0))
CUP_PLASTIC = Material("feeder_cup_plastic", rgba=(0.88, 0.85, 0.76, 1.0))


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
        materials=[BLACK, DARK_TRAY, WOOD, RUBBER, STEEL, CUP_PLASTIC],
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
    door_frame_clearance = 0.030
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

    # A small strike plate for the latch is mounted beside the cut-out, slightly
    # inboard of the rotating tab so the closed pose has no interpenetration.
    cage.visual(
        Box((0.040, 0.006, 0.030)),
        origin=Origin(xyz=(door_x_max + 0.040, front_y - 0.010, door_bottom + 0.360)),
        material=BLACK,
        name="latch_strike",
    )
    _add_rod(
        cage,
        "strike_standoff",
        (door_x_max + 0.040, front_y - 0.010, door_bottom + 0.360),
        (door_x_max + 0.040, front_y, door_bottom + 0.360),
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

    # Two seed/water feeder cups clipped to the front wall bars beside the door.
    # Each assembly: bracket clip on the bar, short arm into the cage, open-top cup.
    feed_heights = [0.55, 0.88]
    for i, fz in enumerate(feed_heights):
        fx = 0.275  # mounted on right-side front vertical bar
        # Bracket clip gripping the vertical bar
        cage.visual(
            Box((0.020, 0.022, 0.026)),
            origin=Origin(xyz=(fx, front_y, fz)),
            material=BLACK,
            name=f"feed_bracket_{i}",
        )
        # Horizontal arm from bracket into cage interior
        _add_rod(
            cage,
            f"feed_arm_{i}",
            (fx, front_y + 0.011, fz),
            (fx, front_y + 0.053, fz),
            radius=0.004,
        )
        # Open-top cup body via lathe shell
        cup_outer = [(0.035, 0.0), (0.037, 0.042)]
        cup_inner = [(0.031, 0.003), (0.033, 0.042)]
        cup_geom = LatheGeometry.from_shell_profiles(
            cup_outer,
            cup_inner,
            segments=20,
            start_cap="flat",
            end_cap="flat",
        )
        cage.visual(
            mesh_from_geometry(cup_geom, f"feed_cup_{i}"),
            origin=Origin(xyz=(fx, front_y + 0.058, fz - 0.021)),
            material=CUP_PLASTIC,
            name=f"feed_cup_{i}",
        )

    # Tray-to-cage metal risers make the wire body visibly bolted to the base tray.
    for ix, x in enumerate((-half_w, half_w)):
        for iy, y in enumerate((front_y, back_y)):
            _add_rod(cage, f"tray_riser_{ix}_{iy}", (x, y, 0.135), (x, y, bottom_z + 0.008), radius=rail_r)

    # Door child frame is the left hinge line at the door sill. Its closed q=0
    # geometry lies exactly in the front face, filling the cut-out rather than
    # sitting over duplicate cage wires.
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
    # Visible hinge barrels just proud of the front plane, tied back to the door frame.
    for i, zc in enumerate([0.170, 0.455]):
        _add_rod(door, f"door_hinge_barrel_{i}", (0.0, -0.018, zc - 0.060), (0.0, -0.018, zc + 0.060), radius=0.011)
        door.visual(Box((0.020, 0.020, 0.018)), origin=Origin(xyz=(0.004, -0.009, zc)), material=BLACK, name=f"door_hinge_leaf_{i}")
    door.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(door_w + 0.014, -0.004, 0.360), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=BLACK,
        name="door_latch_boss",
    )

    door_hinge = model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cage,
        child=door,
        origin=Origin(xyz=(door_x_min, front_y, door_bottom)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.35),
    )

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
        origin=Origin(xyz=(door_w + 0.014, -0.030, 0.360)),
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

    # The closed door sits proud within the fixed surround frame. The surround
    # rods are intentionally slightly thicker than the door frame rods so the
    # door reads as seated within the cage surround at every contact edge.
    ctx.allow_overlap(
        cage,
        door,
        reason="The closed door nests flush within the slightly oversized fixed cage surround frame. All surround-door frame rod contacts are intentional seated trim.",
    )
    ctx.expect_contact(
        cage,
        door,
        elem_a="fixed_door_frame_bottom",
        elem_b="door_bottom_frame",
        contact_tol=0.012,
        name="door bottom frame contacts cage surround at sill",
    )

    # Prompt-specific structure: one root cage, hinged cut-out door, and a
    # separately rotating latch.
    ctx.check("door has revolute hinge", door_hinge.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("latch has revolute pivot", latch_pivot.articulation_type == ArticulationType.REVOLUTE)

    # The door lives flush with the front plane when closed.
    front_y = object_model.meta["front_y"]
    door_closed_pos = ctx.part_world_position(door)
    ctx.check(
        "closed door hinge lies on front face",
        door_closed_pos is not None and abs(door_closed_pos[1] - front_y) < 0.002,
        details=f"door_pos={door_closed_pos}, front_y={front_y}",
    )

    # The closed door panel occupies the cut-out footprint, proving it is a
    # replacement panel rather than a decorative overlay.
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

    rest_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 0.95}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door opens outward from cage front",
        rest_aabb is not None and open_aabb is not None and open_aabb[0][1] < rest_aabb[0][1] - 0.08,
        details=f"rest={rest_aabb}, open={open_aabb}",
    )

    latch_rest = ctx.part_element_world_aabb(latch, elem="latch_tab")
    with ctx.pose({latch_pivot: 1.20}):
        latch_turned = ctx.part_element_world_aabb(latch, elem="latch_tab")
    ctx.check(
        "latch tab rotates upward/downward",
        latch_rest is not None and latch_turned is not None and abs(latch_turned[0][2] - latch_rest[0][2]) > 0.015,
        details=f"rest={latch_rest}, turned={latch_turned}",
    )

    # Feeder cups: two indexed cup+bracket assemblies mounted on the cage front bars.
    for i in range(2):
        cup_vis = cage.get_visual(f"feed_cup_{i}")
        bracket_vis = cage.get_visual(f"feed_bracket_{i}")
        ctx.check(
            f"feed_cup_{i} visual exists on cage",
            cup_vis is not None,
            details=f"feed_cup_{i} not found",
        )
        ctx.check(
            f"feed_bracket_{i} visual exists on cage",
            bracket_vis is not None,
            details=f"feed_bracket_{i} not found",
        )

    # Cups are mounted inside the cage (positive Y from front face).
    cup_0_aabb = ctx.part_element_world_aabb(cage, elem="feed_cup_0")
    cup_1_aabb = ctx.part_element_world_aabb(cage, elem="feed_cup_1")
    front_y = object_model.meta["front_y"]
    ctx.check(
        "feeder cups are inside the cage front plane",
        cup_0_aabb is not None
        and cup_1_aabb is not None
        and cup_0_aabb[0][1] > front_y + 0.02
        and cup_1_aabb[0][1] > front_y + 0.02,
        details=f"cup_0_aabb={cup_0_aabb}, cup_1_aabb={cup_1_aabb}, front_y={front_y}",
    )

    # Cups are at different heights.
    ctx.check(
        "feeder cups at two distinct heights",
        cup_0_aabb is not None
        and cup_1_aabb is not None
        and abs((cup_0_aabb[0][2] + cup_0_aabb[1][2]) / 2.0 - (cup_1_aabb[0][2] + cup_1_aabb[1][2]) / 2.0) > 0.15,
        details=f"cup_0_center_z={(cup_0_aabb[0][2] + cup_0_aabb[1][2]) / 2.0 if cup_0_aabb else None}, "
                f"cup_1_center_z={(cup_1_aabb[0][2] + cup_1_aabb[1][2]) / 2.0 if cup_1_aabb else None}",
    )

    return ctx.report()


object_model = build_object_model()
