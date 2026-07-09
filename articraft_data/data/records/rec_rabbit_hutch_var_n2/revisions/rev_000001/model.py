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

FRONT_Y = -0.430
LATCH_FACE_Y = -0.046


def _box(part, name: str, size, xyz, material: Material, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _cylinder(
    part, name: str, radius: float, length: float, xyz, material: Material, rpy=(0.0, 0.0, 0.0)
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_wood_grain(
    part, prefix: str, width: float, height: float, y: float, material: Material
) -> None:
    """Fine shallow plank seams and short grain marks on the exterior face."""
    for i, z in enumerate((height * 0.30, height * 0.45, height * 0.60, height * 0.75)):
        _box(
            part,
            f"{prefix}_plank_seam_{i}",
            (width * 0.74, 0.003, 0.004),
            (width * 0.50, y, z),
            material,
        )
    grain_marks = (
        (0.25, 0.37, 0.20),
        (0.52, 0.51, 0.16),
        (0.38, 0.69, 0.12),
        (0.68, 0.82, 0.18),
    )
    for i, (x_frac, z_frac, length_frac) in enumerate(grain_marks):
        _box(
            part,
            f"{prefix}_fine_grain_{i}",
            (width * length_frac, 0.0025, 0.003),
            (width * x_frac, y - 0.001, height * z_frac),
            material,
        )


def _add_face_screws(
    part, prefix: str, width: float, height: float, y: float, material: Material
) -> None:
    for i, (x, z) in enumerate(
        (
            (0.075, 0.075),
            (width - 0.075, 0.075),
            (0.075, height - 0.075),
            (width - 0.075, height - 0.075),
        )
    ):
        _cylinder(
            part,
            f"{prefix}_screw_{i}",
            0.006,
            0.004,
            (x, y, z),
            material,
            rpy=(math.pi / 2, 0.0, 0.0),
        )


def _add_hinge_knuckles(part, height: float, material: Material) -> None:
    """Visible black barrel-hinge knuckles carried by a hinged door part."""
    for i, z in enumerate((height * 0.25, height * 0.74)):
        _cylinder(part, f"hinge_barrel_{i}", 0.012, 0.105, (0.0, -0.002, z), material)
        _box(part, f"hinge_leaf_{i}", (0.055, 0.006, 0.075), (0.028, -0.017, z), material)
        # A return leaf reaching back to the fixed front stile so the door reads as
        # physically mounted rather than hovering in front of the hutch.
        _box(part, f"frame_leaf_{i}", (0.080, 0.035, 0.065), (-0.035, 0.010, z), material)


def _add_latch_mount(
    part, width: float, height: float, material: Material, prefix: str = "latch"
) -> tuple[float, float, float, float]:
    """Fixed exterior latch plates on the door face; the hasp bar is a child joint."""
    z = height * 0.55
    bar_len = min(0.180, max(0.085, width * 0.22))
    pivot_x = width - bar_len - 0.045
    keeper_x = width - 0.036

    _box(part, f"{prefix}_pivot_plate", (0.052, 0.016, 0.038), (pivot_x, LATCH_FACE_Y, z), material)
    _box(
        part,
        f"{prefix}_keeper_plate",
        (0.048, 0.018, 0.034),
        (keeper_x, LATCH_FACE_Y - 0.002, z),
        material,
    )
    _box(
        part,
        f"{prefix}_keeper_slot",
        (0.030, 0.022, 0.011),
        (keeper_x - 0.002, LATCH_FACE_Y - 0.010, z),
        material,
    )
    for i, x in enumerate((pivot_x - 0.016, pivot_x + 0.016, keeper_x - 0.014, keeper_x + 0.014)):
        _cylinder(
            part,
            f"{prefix}_plate_screw_{i}",
            0.0045,
            0.004,
            (x, LATCH_FACE_Y - 0.010, z + (0.011 if i % 2 == 0 else -0.011)),
            material,
            rpy=(math.pi / 2, 0.0, 0.0),
        )
    return pivot_x, LATCH_FACE_Y - 0.006, z, bar_len


def _add_latch_bar(part, length: float, material: Material, prefix: str = "latch") -> None:
    """Rotating hasp bar in a child part, with its pivot at local origin."""
    _cylinder(
        part,
        f"{prefix}_pivot_pin",
        0.014,
        0.026,
        (0.0, 0.0, 0.0),
        material,
        rpy=(math.pi / 2, 0.0, 0.0),
    )
    _box(part, f"{prefix}_hasp_bar", (length, 0.014, 0.016), (length / 2.0, -0.002, 0.0), material)
    _box(part, f"{prefix}_hook_end", (0.026, 0.018, 0.024), (length + 0.006, -0.003, 0.0), material)
    _cylinder(
        part,
        f"{prefix}_small_pull",
        0.008,
        0.030,
        (length * 0.58, -0.016, 0.018),
        material,
        rpy=(math.pi / 2, 0.0, 0.0),
    )


def _solid_door_geometry(
    part, width: float, height: float, wood: Material, dark_wood: Material, metal: Material
) -> tuple[float, float, float, float]:
    panel_w = width - 0.055
    panel_h = height - 0.050
    _box(part, "solid_panel", (panel_w, 0.024, panel_h), (width / 2.0, 0.0, height / 2.0), wood)

    rail = 0.050
    y_front = -0.018
    y_detail = -0.034
    _box(part, "left_stile", (rail, 0.014, height), (rail / 2.0, y_front, height / 2.0), wood)
    _box(
        part, "free_stile", (rail, 0.014, height), (width - rail / 2.0, y_front, height / 2.0), wood
    )
    _box(part, "top_rail", (width, 0.014, rail), (width / 2.0, y_front, height - rail / 2.0), wood)
    _box(part, "bottom_rail", (width, 0.014, rail), (width / 2.0, y_front, rail / 2.0), wood)
    _box(
        part,
        "inset_panel_shadow",
        (width - 0.150, 0.006, height - 0.165),
        (width / 2.0, y_detail, height / 2.0),
        dark_wood,
    )
    _box(
        part,
        "inset_panel_face",
        (width - 0.170, 0.006, height - 0.185),
        (width / 2.0, y_detail - 0.004, height / 2.0),
        wood,
    )
    _box(
        part,
        "inner_left_molding",
        (0.018, 0.010, height - 0.135),
        (0.090, y_detail - 0.010, height / 2.0),
        wood,
    )
    _box(
        part,
        "inner_right_molding",
        (0.018, 0.010, height - 0.135),
        (width - 0.090, y_detail - 0.010, height / 2.0),
        wood,
    )
    _box(
        part,
        "inner_top_molding",
        (width - 0.145, 0.010, 0.018),
        (width / 2.0, y_detail - 0.010, height - 0.090),
        wood,
    )
    _box(
        part,
        "inner_bottom_molding",
        (width - 0.145, 0.010, 0.018),
        (width / 2.0, y_detail - 0.010, 0.090),
        wood,
    )
    _add_wood_grain(part, "solid", width, height, y_detail - 0.014, dark_wood)
    _add_face_screws(part, "solid", width, height, y_front - 0.014, dark_wood)
    _add_hinge_knuckles(part, height, metal)
    return _add_latch_mount(part, width, height, metal)


def _mesh_door_geometry(
    part, width: float, height: float, wood: Material, mesh: Material, metal: Material, prefix: str
) -> tuple[float, float, float, float]:
    frame = 0.045
    y_front = -0.018
    _box(
        part,
        f"{prefix}_hinge_stile",
        (frame, 0.030, height),
        (frame / 2.0, 0.0, height / 2.0),
        wood,
    )
    _box(
        part,
        f"{prefix}_free_stile",
        (frame, 0.030, height),
        (width - frame / 2.0, 0.0, height / 2.0),
        wood,
    )
    _box(
        part,
        f"{prefix}_top_rail",
        (width, 0.030, frame),
        (width / 2.0, 0.0, height - frame / 2.0),
        wood,
    )
    _box(
        part, f"{prefix}_bottom_rail", (width, 0.030, frame), (width / 2.0, 0.0, frame / 2.0), wood
    )
    _box(
        part,
        f"{prefix}_mid_rail",
        (width, 0.025, 0.030),
        (width / 2.0, y_front, height * 0.47),
        wood,
    )

    inner_w = width - 2.0 * frame + 0.020
    inner_h = height - 2.0 * frame + 0.020
    x0 = frame - 0.010
    z0 = frame - 0.010
    # Closely spaced square mesh: thin wires crossing and captured by the wood frame.
    v_count = max(4, int(inner_w / 0.055))
    for i in range(v_count + 1):
        x = x0 + inner_w * i / v_count
        _box(
            part,
            f"{prefix}_mesh_v_{i}",
            (0.005, 0.006, inner_h),
            (x, -0.026, z0 + inner_h / 2.0),
            mesh,
        )
    h_count = max(5, int(inner_h / 0.055))
    for i in range(h_count + 1):
        z = z0 + inner_h * i / h_count
        _box(
            part,
            f"{prefix}_mesh_h_{i}",
            (inner_w, 0.006, 0.005),
            (x0 + inner_w / 2.0, -0.030, z),
            mesh,
        )

    _add_hinge_knuckles(part, height, metal)
    return _add_latch_mount(part, width, height, metal, prefix=f"{prefix}_latch")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="wooden_rabbit_hutch",
        meta={
            "category_note": "Reference and category both indicate a rabbit hutch; no classification mismatch suspected."
        },
    )

    pine = model.material("warm_pine", rgba=(0.72, 0.43, 0.18, 1.0))
    dark_pine = model.material("dark_endgrain", rgba=(0.42, 0.24, 0.11, 1.0))
    mesh_mat = model.material("dark_wire_mesh", rgba=(0.06, 0.055, 0.050, 1.0))
    black_metal = model.material("black_latch_metal", rgba=(0.01, 0.010, 0.008, 1.0))
    tray_mat = model.material("galvanized_tray", rgba=(0.54, 0.56, 0.53, 1.0))

    # ── Tier parameterization ──────────────────────────────────────────
    n_tiers = 2
    door_h = 0.465
    tier_pitch = 0.580  # vertical distance between successive row bottoms
    row_bottoms = tuple(0.205 + i * tier_pitch for i in range(n_tiers))
    # row_bottoms = (0.205, 0.785)

    frame = model.part("hutch_frame")

    width = 2.40
    depth = 0.72
    body_bottom = 0.12
    # Body top sits just above the uppermost rail.
    top_rail_z = row_bottoms[-1] + door_h + 0.050  # 1.300
    body_top = top_rail_z + 0.020  # 1.320
    front_rail_y = -0.370
    rear_y = 0.360
    post = 0.060

    # Four load-bearing corner posts/legs, continuing below the cabinet as in the reference.
    for ix, x in enumerate((-width / 2.0 + post / 2.0, width / 2.0 - post / 2.0)):
        for iy, y in enumerate((front_rail_y, rear_y)):
            _box(frame, f"leg_{ix}_{iy}", (post, post, body_top), (x, y, body_top / 2.0), pine)

    # Front compartment grid: vertical bay stiles.
    x_lines = (-1.20, -0.52, -0.10, 1.20)
    for i, x in enumerate(x_lines):
        _box(
            frame,
            f"front_stile_{i}",
            (0.060, 0.065, body_top - body_bottom),
            (x, front_rail_y, (body_top + body_bottom) / 2.0),
            pine,
        )

    # Horizontal front rails: one below the first row + one above each row.
    rail_zs = [0.15]
    for z0 in row_bottoms:
        rail_zs.append(z0 + door_h + 0.050)
    for i, z in enumerate(rail_zs):
        _box(frame, f"front_rail_{i}", (width, 0.065, 0.060), (0.0, front_rail_y, z), pine)

    # Internal shelf/floor boards tie the rows together and read as real hutch compartments.
    for i, z0 in enumerate(row_bottoms):
        floor_offset = 0.035 if i == 0 else 0.045
        _box(
            frame,
            f"compartment_floor_{i}",
            (width - 0.08, depth + 0.02, 0.030),
            (0.0, 0.010, z0 - floor_offset),
            pine,
        )

    # Back and side plank walls, with shallow darker seams to suggest stacked boards.
    wall_height = body_top - 0.20
    wall_bottom_z = 0.17
    wall_center_z = wall_bottom_z + wall_height / 2.0

    _box(frame, "back_wall", (width - 0.08, 0.040, wall_height), (0.0, 0.390, wall_center_z), pine)
    _box(frame, "side_wall_0", (0.040, depth + 0.08, wall_height), (-1.230, 0.020, wall_center_z), pine)
    _box(frame, "side_wall_1", (0.040, depth + 0.08, wall_height), (1.230, 0.020, wall_center_z), pine)

    # Plank seams distributed across the shorter wall height.
    seam_zs = tuple(
        wall_bottom_z + wall_height * frac
        for frac in (0.22, 0.42, 0.62, 0.82)
    )
    for i, z in enumerate(seam_zs):
        _box(
            frame,
            f"side_plank_seam_0_{i}",
            (0.006, depth - 0.10, 0.010),
            (-1.253, 0.020, z),
            dark_pine,
        )
        _box(
            frame,
            f"side_plank_seam_1_{i}",
            (0.006, depth - 0.10, 0.010),
            (1.253, 0.020, z),
            dark_pine,
        )
        _box(
            frame, f"back_plank_seam_{i}", (width - 0.18, 0.006, 0.010), (0.0, 0.413, z), dark_pine
        )

    # Slightly sloped overhanging roof/cap with a full perimeter skirt.
    # The lower bearing frame closes the side-view gap so the overhang reads as supported.
    roof_center_y = 0.010
    roof_center_z = body_top + 0.032
    roof_pitch = 0.035
    roof_panel_thickness = 0.075

    def roof_surface_xyz(
        x: float, local_y: float, local_z_offset: float
    ) -> tuple[float, float, float]:
        local_z = roof_panel_thickness / 2.0 + local_z_offset
        roof_cos = math.cos(roof_pitch)
        roof_sin = math.sin(roof_pitch)
        return (
            x,
            roof_center_y + local_y * roof_cos - local_z * roof_sin,
            roof_center_z + local_y * roof_sin + local_z * roof_cos,
        )

    _box(
        frame,
        "roof_bearing_frame",
        (width + 0.08, depth + 0.04, 0.050),
        (0.0, roof_center_y, body_top - 0.002),
        pine,
    )
    _box(frame, "front_top_ledger", (width + 0.12, 0.050, 0.045), (0.0, -0.415, body_top - 0.020), dark_pine)
    _box(frame, "rear_top_ledger", (width + 0.12, 0.050, 0.045), (0.0, 0.425, body_top - 0.020), dark_pine)
    _box(frame, "left_top_ledger", (0.055, depth + 0.04, 0.045), (-1.230, 0.010, body_top - 0.020), dark_pine)
    _box(frame, "right_top_ledger", (0.055, depth + 0.04, 0.045), (1.230, 0.010, body_top - 0.020), dark_pine)
    _box(
        frame,
        "sloped_roof_panel",
        (width + 0.22, depth + 0.18, 0.075),
        (0.0, roof_center_y, roof_center_z),
        pine,
        rpy=(roof_pitch, 0.0, 0.0),
    )
    _box(
        frame,
        "front_roof_fascia",
        (width + 0.28, 0.060, 0.105),
        (0.0, -0.455, roof_center_z - 0.027),
        dark_pine,
        rpy=(roof_pitch, 0.0, 0.0),
    )
    _box(
        frame,
        "rear_roof_fascia",
        (width + 0.22, 0.055, 0.095),
        (0.0, 0.455, roof_center_z - 0.014),
        dark_pine,
        rpy=(roof_pitch, 0.0, 0.0),
    )
    _box(
        frame,
        "left_roof_side_fascia",
        (0.070, depth + 0.18, 0.105),
        (-1.285, 0.010, roof_center_z - 0.020),
        dark_pine,
        rpy=(roof_pitch, 0.0, 0.0),
    )
    _box(
        frame,
        "right_roof_side_fascia",
        (0.070, depth + 0.18, 0.105),
        (1.285, 0.010, roof_center_z - 0.020),
        dark_pine,
        rpy=(roof_pitch, 0.0, 0.0),
    )
    seam_thickness = 0.006
    for i, local_y in enumerate((-0.310, -0.130, 0.050, 0.230)):
        _box(
            frame,
            f"roof_plank_seam_{i}",
            (width + 0.10, 0.006, seam_thickness),
            roof_surface_xyz(0.0, local_y, seam_thickness / 2.0 - 0.0005),
            dark_pine,
            rpy=(roof_pitch, 0.0, 0.0),
        )
    for i, local_y in enumerate((-0.430, 0.430)):
        _box(
            frame,
            f"roof_edge_cap_{i}",
            (width + 0.18, 0.014, 0.014),
            roof_surface_xyz(0.0, local_y, 0.014 / 2.0 - 0.0005),
            dark_pine,
            rpy=(roof_pitch, 0.0, 0.0),
        )
    _box(
        frame, "front_roof_thin_lip", (width + 0.30, 0.020, 0.030), (0.0, -0.488, body_top - 0.058), dark_pine
    )
    _box(frame, "left_roof_return_block", (0.070, 0.070, 0.090), (-1.285, -0.455, body_top - 0.025), dark_pine)
    _box(frame, "right_roof_return_block", (0.070, 0.070, 0.090), (1.285, -0.455, body_top - 0.025), dark_pine)

    # Side runners for a removable galvanized cleaning tray below the lower compartments.
    _box(frame, "tray_runner_0", (0.045, 0.610, 0.035), (-1.105, -0.0325, 0.1025), dark_pine)
    _box(frame, "tray_runner_1", (0.045, 0.610, 0.035), (1.105, -0.0325, 0.1025), dark_pine)

    # ── Door grid: n_tiers rows × 3 columns ───────────────────────────
    columns = (
        ("solid", -1.130, 0.555),
        ("mesh_narrow", -0.455, 0.300),
        ("mesh_wide", -0.030, 1.125),
    )

    for row, z0 in enumerate(row_bottoms):
        for kind, hinge_x, door_w in columns:
            name = f"{kind}_door_{row}"
            door = model.part(name)
            if kind == "solid":
                latch_origin_x, latch_origin_y, latch_origin_z, latch_len = _solid_door_geometry(
                    door, door_w, door_h, pine, dark_pine, black_metal
                )
            elif kind == "mesh_narrow":
                latch_origin_x, latch_origin_y, latch_origin_z, latch_len = _mesh_door_geometry(
                    door, door_w, door_h, pine, mesh_mat, black_metal, "narrow"
                )
            else:
                latch_origin_x, latch_origin_y, latch_origin_z, latch_len = _mesh_door_geometry(
                    door, door_w, door_h, pine, mesh_mat, black_metal, "wide"
                )

            model.articulation(
                f"frame_to_{name}",
                ArticulationType.REVOLUTE,
                parent=frame,
                child=door,
                origin=Origin(xyz=(hinge_x, FRONT_Y, z0)),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=0.0, upper=1.35),
            )

            latch = model.part(f"{name}_latch")
            _add_latch_bar(latch, latch_len, black_metal, prefix=f"{name}_latch")
            model.articulation(
                f"{name}_to_latch",
                ArticulationType.REVOLUTE,
                parent=door,
                child=latch,
                origin=Origin(xyz=(latch_origin_x, latch_origin_y, latch_origin_z)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=0.0, upper=1.15),
            )

    tray = model.part("cleaning_tray")
    _box(tray, "tray_pan", (2.115, 0.515, 0.018), (0.0, 0.235, 0.0), tray_mat)
    _box(tray, "front_lip", (2.100, 0.030, 0.065), (0.0, -0.030, 0.000), tray_mat)
    _box(tray, "tray_handle", (0.280, 0.020, 0.030), (0.0, -0.055, 0.025), black_metal)
    _box(tray, "side_lip_0", (0.025, 0.500, 0.045), (-1.070, 0.220, 0.020), tray_mat)
    _box(tray, "side_lip_1", (0.025, 0.500, 0.045), (1.070, 0.220, 0.020), tray_mat)
    model.articulation(
        "frame_to_cleaning_tray",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=tray,
        origin=Origin(xyz=(0.0, -0.330, 0.065)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.25, lower=0.0, upper=0.320),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("hutch_frame")
    solid = object_model.get_part("solid_door_1")
    mesh = object_model.get_part("mesh_wide_door_1")
    solid_latch = object_model.get_part("solid_door_1_latch")
    mesh_latch = object_model.get_part("mesh_wide_door_1_latch")
    tray = object_model.get_part("cleaning_tray")
    solid_hinge = object_model.get_articulation("frame_to_solid_door_1")
    mesh_hinge = object_model.get_articulation("frame_to_mesh_wide_door_1")
    solid_latch_joint = object_model.get_articulation("solid_door_1_to_latch")
    mesh_latch_joint = object_model.get_articulation("mesh_wide_door_1_to_latch")
    tray_slide = object_model.get_articulation("frame_to_cleaning_tray")

    # ── Two-tier compartment grid: exactly 2 rows × 3 columns = 6 doors ──
    def _part_exists(part_name: str) -> bool:
        try:
            object_model.get_part(part_name)
            return True
        except Exception:
            return False

    column_kinds = ("solid", "mesh_narrow", "mesh_wide")
    n_tiers = 2
    for row in range(n_tiers):
        for kind in column_kinds:
            door_name = f"{kind}_door_{row}"
            ctx.check(
                f"two-tier grid: {door_name} exists",
                _part_exists(door_name),
                details=f"expected door part {door_name} in 2-row × 3-column grid",
            )
            latch_name = f"{door_name}_latch"
            ctx.check(
                f"two-tier grid: {latch_name} exists",
                _part_exists(latch_name),
                details=f"expected latch part {latch_name} for tier {row}",
            )

    # Verify no third-tier doors exist (proves reduction from 3 to 2 tiers).
    for kind in column_kinds:
        ctx.check(
            f"no tier-2 {kind} door in two-tier hutch",
            not _part_exists(f"{kind}_door_2"),
            details=f"{kind}_door_2 should not exist in a 2-tier hutch",
        )

    # Upper row doors sit above lower row doors in world Z.
    solid_0 = object_model.get_part("solid_door_0")
    solid_1 = object_model.get_part("solid_door_1")
    ctx.expect_gap(
        solid_1, solid_0, axis="z", min_gap=0.010,
        name="upper-tier solid door sits above lower-tier solid door",
    )

    # ── Latch mounting allowances: each latch hasp is intentionally mounted on
    #    its door — the pivot pin seats in the pivot plate and the hook end
    #    engages the keeper slot. These are standard hasp-latch interfaces.
    for row in range(n_tiers):
        for kind in column_kinds:
            door_name = f"{kind}_door_{row}"
            latch_name = f"{door_name}_latch"
            door_part = object_model.get_part(door_name)
            latch_part = object_model.get_part(latch_name)
            ctx.allow_overlap(
                door_part,
                latch_part,
                reason=(
                    f"Hasp latch mounting: {latch_name} pivot pin seats in {door_name} pivot plate "
                    f"and hook end engages keeper slot — standard latch hardware interface."
                ),
            )
            # Proof: the latch part remains mounted on (in contact with) its door.
            ctx.expect_contact(
                door_part,
                latch_part,
                name=f"{latch_name} is mounted on {door_name}",
            )

    # The repeated compartment layout should be visible: closed access doors sit on the front grid.
    ctx.expect_overlap(
        solid,
        frame,
        axes="xz",
        min_overlap=0.20,
        name="solid wooden door occupies a front compartment",
    )
    ctx.expect_overlap(
        mesh,
        frame,
        axes="xz",
        min_overlap=0.35,
        name="large wire-mesh door occupies a front compartment",
    )
    ctx.expect_gap(
        frame,
        solid,
        axis="y",
        min_gap=0.004,
        max_gap=0.040,
        positive_elem="front_stile_0",
        negative_elem="solid_panel",
        name="closed doors stand just proud of the wooden front frame",
    )

    # Positive hinge motion opens outward from the hutch front (toward negative Y).
    rest_solid = ctx.part_world_aabb(solid)
    rest_mesh = ctx.part_world_aabb(mesh)
    with ctx.pose({solid_hinge: 0.90, mesh_hinge: 0.90}):
        open_solid = ctx.part_world_aabb(solid)
        open_mesh = ctx.part_world_aabb(mesh)
    ctx.check(
        "solid door opens outward on hinge",
        rest_solid is not None
        and open_solid is not None
        and open_solid[0][1] < rest_solid[0][1] - 0.12,
        details=f"rest={rest_solid}, open={open_solid}",
    )
    ctx.check(
        "wire mesh door opens outward on hinge",
        rest_mesh is not None
        and open_mesh is not None
        and open_mesh[0][1] < rest_mesh[0][1] - 0.20,
        details=f"rest={rest_mesh}, open={open_mesh}",
    )

    # Each black hasp sits proud of the exterior face and has its own small pivot.
    rest_solid_latch = ctx.part_world_aabb(solid_latch)
    rest_mesh_latch = ctx.part_world_aabb(mesh_latch)
    ctx.check(
        "solid door latch is mounted on the exterior face",
        rest_solid_latch is not None and rest_solid_latch[0][1] < FRONT_Y - 0.045,
        details=f"latch={rest_solid_latch}",
    )
    ctx.check(
        "wire door latch is mounted on the exterior face",
        rest_mesh_latch is not None and rest_mesh_latch[0][1] < FRONT_Y - 0.045,
        details=f"latch={rest_mesh_latch}",
    )
    with ctx.pose({solid_latch_joint: 0.80, mesh_latch_joint: 0.80}):
        lifted_solid_latch = ctx.part_world_aabb(solid_latch)
        lifted_mesh_latch = ctx.part_world_aabb(mesh_latch)
    ctx.check(
        "solid door hasp pivots on its latch joint",
        rest_solid_latch is not None
        and lifted_solid_latch is not None
        and lifted_solid_latch[0][2] < rest_solid_latch[0][2] - 0.050,
        details=f"rest={rest_solid_latch}, moved={lifted_solid_latch}",
    )
    ctx.check(
        "wire door hasp pivots on its latch joint",
        rest_mesh_latch is not None
        and lifted_mesh_latch is not None
        and lifted_mesh_latch[0][2] < rest_mesh_latch[0][2] - 0.070,
        details=f"rest={rest_mesh_latch}, moved={lifted_mesh_latch}",
    )

    # The cleaning tray is retained between wooden runners and slides forward for removal.
    ctx.expect_within(tray, frame, axes="x", margin=0.070, name="tray width fits between side legs")
    rest_tray = ctx.part_world_aabb(tray)
    with ctx.pose({tray_slide: 0.280}):
        extended_tray = ctx.part_world_aabb(tray)
        ctx.expect_overlap(
            tray,
            frame,
            axes="x",
            min_overlap=1.80,
            name="extended tray remains aligned with hutch width",
        )
    ctx.check(
        "cleaning tray slides out toward the viewer",
        rest_tray is not None
        and extended_tray is not None
        and extended_tray[0][1] < rest_tray[0][1] - 0.20,
        details=f"rest={rest_tray}, extended={extended_tray}",
    )

    return ctx.report()


object_model = build_object_model()
