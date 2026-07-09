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
        name="tall_black_bird_cage_on_stand",
        materials=[BLACK, DARK_TRAY, WOOD],
        meta={
            "description": "Tall black bird cage on four-leg floor stand with barrel-vault roof, cut-out front access door, latch, perches, and lower shelf.",
        },
    )

    # ── Stand dimensions ──────────────────────────────────────────────
    stand_top_z = 0.60
    stand_half_w = 0.41
    stand_half_d = 0.29
    leg_r = 0.014
    apron_r = 0.008
    shelf_z = 0.14
    foot_r = 0.018
    foot_h = 0.008

    # ── Cage body dimensions (unchanged from parent) ──────────────────
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

    # ═══════════════════════════════════════════════════════════════════
    # STAND PART (root)
    # ═══════════════════════════════════════════════════════════════════
    stand = model.part("stand")

    # Four corner legs with foot pads (loop-emitted)
    for i, (lx, ly) in enumerate([
        (-stand_half_w, -stand_half_d),
        (-stand_half_w, stand_half_d),
        (stand_half_w, -stand_half_d),
        (stand_half_w, stand_half_d),
    ]):
        _add_rod(stand, f"stand_leg_{i}", (lx, ly, foot_h), (lx, ly, stand_top_z), radius=leg_r)
        stand.visual(
            Cylinder(radius=foot_r, length=foot_h),
            origin=Origin(xyz=(lx, ly, foot_h / 2.0)),
            material=BLACK,
            name=f"foot_pad_{i}",
        )

    # Top apron rails (rectangular frame supporting the cage tray)
    _add_rod(stand, "apron_front", (-stand_half_w, -stand_half_d, stand_top_z), (stand_half_w, -stand_half_d, stand_top_z), radius=apron_r)
    _add_rod(stand, "apron_rear", (-stand_half_w, stand_half_d, stand_top_z), (stand_half_w, stand_half_d, stand_top_z), radius=apron_r)
    _add_rod(stand, "apron_left", (-stand_half_w, -stand_half_d, stand_top_z), (-stand_half_w, stand_half_d, stand_top_z), radius=apron_r)
    _add_rod(stand, "apron_right", (stand_half_w, -stand_half_d, stand_top_z), (stand_half_w, stand_half_d, stand_top_z), radius=apron_r)

    # Lower storage shelf: frame + cross rods
    _add_rod(stand, "shelf_front", (-stand_half_w, -stand_half_d, shelf_z), (stand_half_w, -stand_half_d, shelf_z), radius=apron_r)
    _add_rod(stand, "shelf_rear", (-stand_half_w, stand_half_d, shelf_z), (stand_half_w, stand_half_d, shelf_z), radius=apron_r)
    _add_rod(stand, "shelf_left", (-stand_half_w, -stand_half_d, shelf_z), (-stand_half_w, stand_half_d, shelf_z), radius=apron_r)
    _add_rod(stand, "shelf_right", (stand_half_w, -stand_half_d, shelf_z), (stand_half_w, stand_half_d, shelf_z), radius=apron_r)
    for i in range(9):
        sx = -stand_half_w + 0.04 + i * ((2.0 * stand_half_w - 0.08) / 8.0)
        _add_rod(stand, f"shelf_cross_{i}", (sx, -stand_half_d, shelf_z), (sx, stand_half_d, shelf_z), radius=rod_r)

    # Mid-height stretcher rails for lateral rigidity
    mid_z = (shelf_z + stand_top_z) / 2.0
    _add_rod(stand, "stretcher_front", (-stand_half_w, -stand_half_d, mid_z), (stand_half_w, -stand_half_d, mid_z), radius=rod_r)
    _add_rod(stand, "stretcher_rear", (-stand_half_w, stand_half_d, mid_z), (stand_half_w, stand_half_d, mid_z), radius=rod_r)
    _add_rod(stand, "stretcher_left", (-stand_half_w, -stand_half_d, mid_z), (-stand_half_w, stand_half_d, mid_z), radius=rod_r)
    _add_rod(stand, "stretcher_right", (stand_half_w, -stand_half_d, mid_z), (stand_half_w, stand_half_d, mid_z), radius=rod_r)

    # ═══════════════════════════════════════════════════════════════════
    # CAGE PART (fixed to stand apron)
    # ═══════════════════════════════════════════════════════════════════
    cage = model.part("cage")

    # Thin slide-out tray sitting on the stand apron
    cage.visual(Box((0.82, 0.56, 0.022)), origin=Origin(xyz=(0.0, 0.0, 0.011)), material=DARK_TRAY, name="cage_tray")
    # Tray containment lips
    cage.visual(Box((0.82, 0.022, 0.038)), origin=Origin(xyz=(0.0, -0.29, 0.030)), material=DARK_TRAY, name="tray_lip_front")
    cage.visual(Box((0.82, 0.022, 0.038)), origin=Origin(xyz=(0.0, 0.29, 0.030)), material=DARK_TRAY, name="tray_lip_rear")
    cage.visual(Box((0.022, 0.56, 0.038)), origin=Origin(xyz=(-0.42, 0.0, 0.030)), material=DARK_TRAY, name="tray_lip_left")
    cage.visual(Box((0.022, 0.56, 0.038)), origin=Origin(xyz=(0.42, 0.0, 0.030)), material=DARK_TRAY, name="tray_lip_right")

    # Tray-to-cage metal risers (connect tray to wire body corners)
    for ix, x in enumerate((-half_w, half_w)):
        for iy, y in enumerate((front_y, back_y)):
            _add_rod(cage, f"tray_riser_{ix}_{iy}", (x, y, 0.022), (x, y, bottom_z - 0.005), radius=rail_r)

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

    # Fixed arched door surround on the cage face.
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

    # Strike plate for the latch.
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

    # Two pale wood perches spanning the cage interior.
    _add_rod(cage, "lower_wood_perch", (-half_w - 0.006, -0.070, 0.735), (half_w + 0.006, -0.070, 0.735), radius=0.014, material=WOOD)
    _add_rod(cage, "upper_wood_perch", (-half_w - 0.006, 0.065, 1.095), (half_w + 0.006, 0.065, 1.095), radius=0.014, material=WOOD)

    # ── FIXED: stand → cage ──────────────────────────────────────────
    model.articulation(
        "stand_to_cage",
        ArticulationType.FIXED,
        parent=stand,
        child=cage,
        origin=Origin(xyz=(0.0, 0.0, stand_top_z)),
    )

    # ═══════════════════════════════════════════════════════════════════
    # DOOR (child of cage, unchanged from parent)
    # ═══════════════════════════════════════════════════════════════════
    door = model.part("door")
    door_h = door_spring_rel
    _add_rod(door, "door_left_frame", (0.0, 0.0, 0.0), (0.0, 0.0, door_h), radius=rail_r)
    _add_rod(door, "door_right_frame", (door_w, 0.0, 0.0), (door_w, 0.0, door_h), radius=rail_r)
    _add_rod(door, "door_bottom_frame", (0.0, 0.0, 0.0), (door_w, 0.0, 0.0), radius=rail_r)
    _add_arch(door, "door_arch_frame", center_x=door_w / 2.0, y=0.0, spring_z=door_h, radius=door_w / 2.0, segments=14, rod_radius=rail_r)
    for i, lx in enumerate([0.060, 0.120, 0.180, 0.240, 0.300]):
        _add_rod(door, f"door_vertical_bar_{i}", (lx, 0.0, 0.0), (lx, 0.0, _door_arch_z(lx, door_w, door_h)), radius=rod_r)
    for i, lz in enumerate([0.285, 0.520]):
        _add_rod(door, f"door_horizontal_bar_{i}", (0.020, 0.0, lz), (door_w - 0.020, 0.0, lz), radius=rod_r)
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

    # ═══════════════════════════════════════════════════════════════════
    # LATCH (child of door, unchanged from parent)
    # ═══════════════════════════════════════════════════════════════════
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

    # Store key dimensions for semantic tests.
    # Z values need the stand offset since tests compare against world AABBs.
    model.meta["door_hinge_name"] = door_hinge.name
    model.meta["front_y"] = front_y
    model.meta["door_bottom"] = door_bottom
    model.meta["door_x_min"] = door_x_min
    model.meta["door_x_max"] = door_x_max
    model.meta["door_top_z"] = door_top_z + stand_top_z
    model.meta["surround_x_min"] = surround_x_min
    model.meta["surround_x_max"] = surround_x_max
    model.meta["surround_top_z"] = surround_top_z + stand_top_z
    model.meta["stand_top_z"] = stand_top_z
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    cage = object_model.get_part("cage")
    door = object_model.get_part("door")
    latch = object_model.get_part("latch")
    door_hinge = object_model.get_articulation("door_hinge")
    latch_pivot = object_model.get_articulation("latch_pivot")
    stand_to_cage = object_model.get_articulation("stand_to_cage")

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

    # The closed door is a cut-out replacement panel that seats flush within
    # the fixed surround frame. The surround rod radii produce small local
    # overlap with the door frame elements at the seating interface.
    ctx.allow_overlap(
        cage,
        door,
        elem_a="fixed_door_frame_bottom",
        elem_b="door_bottom_frame",
        reason="The closed door bottom sill seats flush within the fixed surround frame bottom rail.",
    )
    ctx.allow_overlap(
        cage,
        door,
        elem_a="fixed_door_frame_bottom",
        elem_b="door_left_frame",
        reason="The closed door left jamb seats within the surround frame at the hinge corner.",
    )
    ctx.allow_overlap(
        cage,
        door,
        elem_a="fixed_door_frame_bottom",
        elem_b="door_right_frame",
        reason="The closed door right jamb seats within the surround frame at the latch corner.",
    )
    ctx.allow_overlap(
        cage,
        door,
        elem_a="fixed_door_frame_bottom",
        elem_b="door_vertical_bar_0",
        reason="Door vertical bars near the left jamb contact the surround bottom rail at seating.",
    )
    ctx.allow_overlap(
        cage,
        door,
        elem_a="fixed_door_frame_bottom",
        elem_b="door_vertical_bar_1",
        reason="Door vertical bar contacts the surround bottom rail at seating.",
    )
    ctx.allow_overlap(
        cage,
        door,
        elem_a="fixed_door_frame_bottom",
        elem_b="door_vertical_bar_2",
        reason="Door vertical bar contacts the surround bottom rail at seating.",
    )
    ctx.allow_overlap(
        cage,
        door,
        elem_a="fixed_door_frame_bottom",
        elem_b="door_vertical_bar_3",
        reason="Door vertical bar contacts the surround bottom rail at seating.",
    )
    ctx.allow_overlap(
        cage,
        door,
        elem_a="fixed_door_frame_bottom",
        elem_b="door_vertical_bar_4",
        reason="Door vertical bar contacts the surround bottom rail at seating.",
    )
    ctx.expect_overlap(
        cage,
        door,
        axes="xz",
        elem_a="fixed_door_frame_bottom",
        elem_b="door_bottom_frame",
        min_overlap=0.01,
        name="closed door bottom frame overlaps surround sill in XZ",
    )

    # The cage tray sits on the stand apron rails.
    ctx.allow_overlap(
        cage,
        stand,
        elem_a="cage_tray",
        elem_b="apron_left",
        reason="The cage tray rests on the stand apron; the tray underside is seated against the apron rod.",
    )
    ctx.allow_overlap(
        cage,
        stand,
        elem_a="cage_tray",
        elem_b="apron_right",
        reason="The cage tray rests on the stand apron; the tray underside is seated against the apron rod.",
    )
    ctx.allow_overlap(
        cage,
        stand,
        elem_a="cage_tray",
        elem_b="apron_front",
        reason="The cage tray rests on the stand apron; the tray underside is seated against the apron rod.",
    )
    ctx.allow_overlap(
        cage,
        stand,
        elem_a="cage_tray",
        elem_b="apron_rear",
        reason="The cage tray rests on the stand apron; the tray underside is seated against the apron rod.",
    )
    ctx.expect_contact(
        cage,
        stand,
        elem_a="cage_tray",
        elem_b="apron_front",
        contact_tol=0.015,
        name="cage tray contacts stand apron",
    )

    # ── Stand structure tests (variant-specific) ─────────────────────
    ctx.check(
        "stand_to_cage is FIXED articulation",
        stand_to_cage.articulation_type == ArticulationType.FIXED,
    )
    ctx.check("door has revolute hinge", door_hinge.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("latch has revolute pivot", latch_pivot.articulation_type == ArticulationType.REVOLUTE)

    # The stand has four legs emitted via loop
    stand_visual_names = [v.name for v in stand.visuals]
    leg_names = [n for n in stand_visual_names if n.startswith("stand_leg_")]
    foot_names = [n for n in stand_visual_names if n.startswith("foot_pad_")]
    ctx.check(
        "stand has four corner legs",
        len(leg_names) == 4,
        details=f"found legs: {leg_names}",
    )
    ctx.check(
        "stand has four foot pads",
        len(foot_names) == 4,
        details=f"found pads: {foot_names}",
    )

    # The cage body is elevated above the floor by the stand
    stand_top_z = object_model.meta["stand_top_z"]
    cage_aabb = ctx.part_world_aabb(cage)
    ctx.check(
        "cage body is elevated above floor by stand",
        cage_aabb is not None and cage_aabb[0][2] > stand_top_z - 0.05,
        details=f"cage_aabb_min_z={cage_aabb[0][2] if cage_aabb else None}, stand_top_z={stand_top_z}",
    )

    # The stand top apron overlaps the cage footprint in XY
    ctx.expect_overlap(stand, cage, axes="xy", min_overlap=0.30, name="stand apron supports cage footprint")

    # ── Door and latch tests (unchanged from parent) ─────────────────
    front_y = object_model.meta["front_y"]
    door_closed_pos = ctx.part_world_position(door)
    ctx.check(
        "closed door hinge lies on front face",
        door_closed_pos is not None and abs(door_closed_pos[1] - front_y) < 0.002,
        details=f"door_pos={door_closed_pos}, front_y={front_y}",
    )

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

    return ctx.report()


object_model = build_object_model()
