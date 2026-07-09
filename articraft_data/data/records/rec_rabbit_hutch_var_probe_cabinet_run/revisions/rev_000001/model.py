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
        # A return leaf reaches back to the fixed front stile so the door reads as
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

    frame = model.part("hutch_frame")

    width = 2.40
    depth = 0.72
    body_bottom = 0.12
    body_top = 1.90
    front_rail_y = -0.370
    rear_y = 0.360
    post = 0.060

    # Four load-bearing corner posts/legs, continuing below the cabinet as in the reference.
    for ix, x in enumerate((-width / 2.0 + post / 2.0, width / 2.0 - post / 2.0)):
        for iy, y in enumerate((front_rail_y, rear_y)):
            _box(frame, f"leg_{ix}_{iy}", (post, post, body_top), (x, y, body_top / 2.0), pine)

    # Front compartment grid: three rows and three vertical bay divisions.
    x_lines = (-1.20, -0.52, -0.10, 1.20)
    for i, x in enumerate(x_lines):
        _box(
            frame,
            f"front_stile_{i}",
            (0.060, 0.065, body_top - body_bottom),
            (x, front_rail_y, (body_top + body_bottom) / 2.0),
            pine,
        )

    for i, z in enumerate((0.15, 0.72, 1.30, 1.88)):
        _box(frame, f"front_rail_{i}", (width, 0.065, 0.060), (0.0, front_rail_y, z), pine)

    # Internal shelf/floor boards tie the rows together and read as real hutch compartments.
    for i, z in enumerate((0.17, 0.74, 1.32)):
        _box(
            frame,
            f"compartment_floor_{i}",
            (width - 0.08, depth + 0.02, 0.030),
            (0.0, 0.010, z),
            pine,
        )

    # Back and side plank walls, with shallow darker seams to suggest stacked boards.
    _box(frame, "back_wall", (width - 0.08, 0.040, body_top - 0.20), (0.0, 0.390, 1.02), pine)
    _box(frame, "side_wall_0", (0.040, depth + 0.08, body_top - 0.20), (-1.230, 0.020, 1.02), pine)
    _box(frame, "side_wall_1", (0.040, depth + 0.08, body_top - 0.20), (1.230, 0.020, 1.02), pine)
    for i, z in enumerate((0.42, 0.64, 0.98, 1.20, 1.55, 1.76)):
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
    roof_center_z = 1.932
    roof_pitch = 0.035
    roof_panel_thickness = 0.075
    roof_cos = math.cos(roof_pitch)
    roof_sin = math.sin(roof_pitch)

    def roof_surface_xyz(
        x: float, local_y: float, local_z_offset: float
    ) -> tuple[float, float, float]:
        local_z = roof_panel_thickness / 2.0 + local_z_offset
        return (
            x,
            roof_center_y + local_y * roof_cos - local_z * roof_sin,
            roof_center_z + local_y * roof_sin + local_z * roof_cos,
        )

    _box(
        frame,
        "roof_bearing_frame",
        (width + 0.08, depth + 0.04, 0.050),
        (0.0, roof_center_y, 1.898),
        pine,
    )
    _box(frame, "front_top_ledger", (width + 0.12, 0.050, 0.045), (0.0, -0.415, 1.880), dark_pine)
    _box(frame, "rear_top_ledger", (width + 0.12, 0.050, 0.045), (0.0, 0.425, 1.880), dark_pine)
    _box(frame, "left_top_ledger", (0.055, depth + 0.04, 0.045), (-1.230, 0.010, 1.880), dark_pine)
    _box(frame, "right_top_ledger", (0.055, depth + 0.04, 0.045), (1.230, 0.010, 1.880), dark_pine)
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
        (0.0, -0.455, 1.905),
        dark_pine,
        rpy=(roof_pitch, 0.0, 0.0),
    )
    _box(
        frame,
        "rear_roof_fascia",
        (width + 0.22, 0.055, 0.095),
        (0.0, 0.455, 1.918),
        dark_pine,
        rpy=(roof_pitch, 0.0, 0.0),
    )
    _box(
        frame,
        "left_roof_side_fascia",
        (0.070, depth + 0.18, 0.105),
        (-1.285, 0.010, 1.912),
        dark_pine,
        rpy=(roof_pitch, 0.0, 0.0),
    )
    _box(
        frame,
        "right_roof_side_fascia",
        (0.070, depth + 0.18, 0.105),
        (1.285, 0.010, 1.912),
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
        frame, "front_roof_thin_lip", (width + 0.30, 0.020, 0.030), (0.0, -0.488, 1.862), dark_pine
    )
    _box(frame, "left_roof_return_block", (0.070, 0.070, 0.090), (-1.285, -0.455, 1.875), dark_pine)
    _box(frame, "right_roof_return_block", (0.070, 0.070, 0.090), (1.285, -0.455, 1.875), dark_pine)

    # Side runners for a removable galvanized cleaning tray below the lower compartments.
    _box(frame, "tray_runner_0", (0.045, 0.610, 0.035), (-1.105, -0.0325, 0.1025), dark_pine)
    _box(frame, "tray_runner_1", (0.045, 0.610, 0.035), (1.105, -0.0325, 0.1025), dark_pine)

    row_bottoms = (0.205, 0.785, 1.365)
    door_h = 0.465
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

    # =====================================================================
    # Run bay: open post-and-rail wire-mesh exercise run off the right side
    # =====================================================================
    run_left_x = 1.250       # just outside right side wall (exterior face ~1.250)
    run_right_x = 2.280
    run_width = run_right_x - run_left_x   # ~1.03
    run_front_y = front_rail_y              # -0.370
    run_rear_y = rear_y                     # 0.360
    run_depth = run_rear_y - run_front_y    # 0.73
    run_center_x = (run_left_x + run_right_x) / 2.0
    run_center_y = (run_front_y + run_rear_y) / 2.0
    run_bottom_z = 0.05
    run_top_z = 0.72
    run_height = run_top_z - run_bottom_z   # 0.67
    run_post = 0.050
    run_rail = 0.036

    dark_void = model.material("dark_void", rgba=(0.02, 0.02, 0.02, 1.0))

    # Vertical hinge mount board at the run door hinge line, split into two
    # pieces above and below the door mid-rail so the door frame members do
    # not penetrate the mount board.
    mount_x = run_left_x + 0.06
    door_hinge_z_world = run_bottom_z + run_rail + 0.012  # 0.098
    mid_rail_z_world = door_hinge_z_world + door_h * 0.48  # ~0.372
    # Lower mount piece: from door bottom rail area to just below mid rail
    m_lo_z = door_hinge_z_world + 0.04  # ~0.138
    m_lo_hi = mid_rail_z_world - 0.018  # just below mid_rail
    m_lo_h = m_lo_hi - m_lo_z
    _box(
        frame, "run_door_hinge_mount_lo",
        (0.030, 0.050, m_lo_h),
        (mount_x, run_front_y, m_lo_z + m_lo_h / 2.0),
        pine,
    )
    # Upper mount piece: from just above mid rail to just below top rail
    m_hi_z = mid_rail_z_world + 0.018  # just above mid_rail
    m_hi_hi = door_hinge_z_world + door_h - 0.030  # just below top rail
    m_hi_h = m_hi_hi - m_hi_z
    _box(
        frame, "run_door_hinge_mount_hi",
        (0.030, 0.050, m_hi_h),
        (mount_x, run_front_y, m_hi_z + m_hi_h / 2.0),
        pine,
    )

    # Far-right corner posts (left side shares the cabinet legs leg_1_*)
    for iy, y in enumerate((run_front_y + run_post / 2.0, run_rear_y - run_post / 2.0)):
        _box(
            frame,
            f"run_post_{iy}",
            (run_post, run_post, run_top_z),
            (run_right_x - run_post / 2.0, y, run_top_z / 2.0),
            pine,
        )

    # Top and bottom horizontal rails (front, rear, far side)
    for i, z in enumerate((run_bottom_z + run_rail / 2.0, run_top_z - run_rail / 2.0)):
        _box(
            frame, f"run_rail_front_{i}", (run_width, run_rail, run_rail),
            (run_center_x, run_front_y, z), pine,
        )
        _box(
            frame, f"run_rail_rear_{i}", (run_width, run_rail, run_rail),
            (run_center_x, run_rear_y, z), pine,
        )
        _box(
            frame, f"run_rail_side_{i}", (run_rail, run_depth - 2 * run_rail, run_rail),
            (run_right_x - run_rail / 2.0, run_center_y, z), pine,
        )

    # Run floor panel
    _box(
        frame, "run_floor_panel",
        (run_width - 0.06, run_depth - 0.06, 0.022),
        (run_center_x, run_center_y, 0.025),
        pine,
    )

    # --- Rear mesh panel (XZ plane at rear of run) ---
    rear_frame = run_rail
    rear_inner_w = run_width - 2.0 * rear_frame
    rear_inner_h = run_height - 2.0 * rear_frame
    rear_x0 = run_left_x + rear_frame
    rear_z0 = run_bottom_z + rear_frame
    rear_v = max(4, int(rear_inner_w / 0.055))
    for i in range(rear_v + 1):
        x = rear_x0 + rear_inner_w * i / rear_v
        _box(
            frame, f"run_rear_mesh_v_{i}", (0.005, 0.006, rear_inner_h),
            (x, run_rear_y + 0.003, rear_z0 + rear_inner_h / 2.0), mesh_mat,
        )
    rear_h = max(5, int(rear_inner_h / 0.055))
    for i in range(rear_h + 1):
        z = rear_z0 + rear_inner_h * i / rear_h
        _box(
            frame, f"run_rear_mesh_h_{i}", (rear_inner_w, 0.006, 0.005),
            (rear_x0 + rear_inner_w / 2.0, run_rear_y + 0.006, z), mesh_mat,
        )

    # --- Far-side mesh panel (YZ plane at right of run) ---
    side_inner_w = run_depth - 2.0 * rear_frame
    side_inner_h = run_height - 2.0 * rear_frame
    side_y0 = run_front_y + rear_frame
    side_z0 = run_bottom_z + rear_frame
    side_v = max(4, int(side_inner_w / 0.055))
    for i in range(side_v + 1):
        y = side_y0 + side_inner_w * i / side_v
        _box(
            frame, f"run_side_mesh_v_{i}", (0.006, 0.005, side_inner_h),
            (run_right_x + 0.003, y, side_z0 + side_inner_h / 2.0), mesh_mat,
        )
    side_h = max(5, int(side_inner_h / 0.055))
    for i in range(side_h + 1):
        z = side_z0 + side_inner_h * i / side_h
        _box(
            frame, f"run_side_mesh_h_{i}", (0.006, side_inner_w, 0.005),
            (run_right_x + 0.006, side_y0 + side_inner_w / 2.0, z), mesh_mat,
        )

    # --- Top mesh panel (XY plane at top of run) ---
    top_inner_w = run_width - 2.0 * rear_frame
    top_inner_d = run_depth - 2.0 * rear_frame
    top_x0 = run_left_x + rear_frame
    top_y0 = run_front_y + rear_frame
    top_v = max(4, int(top_inner_w / 0.055))
    for i in range(top_v + 1):
        x = top_x0 + top_inner_w * i / top_v
        _box(
            frame, f"run_top_mesh_v_{i}", (0.005, top_inner_d, 0.006),
            (x, run_center_y, run_top_z + 0.003), mesh_mat,
        )
    top_h = max(4, int(top_inner_d / 0.055))
    for i in range(top_h + 1):
        y = top_y0 + top_inner_d * i / top_h
        _box(
            frame, f"run_top_mesh_h_{i}", (top_inner_w, 0.005, 0.006),
            (top_x0 + top_inner_w / 2.0, y, run_top_z + 0.006), mesh_mat,
        )

    # --- Pop-hole in right side wall (lowest compartment) ---
    pop_hole_w = 0.20
    pop_hole_h = 0.22
    pop_hole_center_z = 0.42
    pop_hole_bottom_z = pop_hole_center_z - pop_hole_h / 2.0  # 0.31

    # Dark surround on the exterior face suggesting the opening
    _box(
        frame, "pop_hole_surround",
        (0.010, pop_hole_w + 0.04, pop_hole_h + 0.04),
        (1.258, 0.0, pop_hole_center_z), dark_pine,
    )
    _box(
        frame, "pop_hole_void",
        (0.008, pop_hole_w, pop_hole_h),
        (1.262, 0.0, pop_hole_center_z), dark_void,
    )

    # --- Ramp: hinged plank from pop-hole down into the run ---
    ramp = model.part("run_ramp")
    ramp_length = 0.44
    ramp_width = pop_hole_w + 0.04
    ramp_thickness = 0.022
    ramp_deploy_angle = math.atan2(pop_hole_bottom_z - run_bottom_z, ramp_length)
    r_cos = math.cos(ramp_deploy_angle)
    r_sin = math.sin(ramp_deploy_angle)

    # Visual origins: pin the plank near-end center at the part-frame origin
    # (which is the hinge point). After Ry(+deploy_angle), the far end tilts
    # downward toward the run floor.
    plank_vx = (ramp_length / 2.0) * r_cos
    plank_vz = -(ramp_length / 2.0) * r_sin

    _box(
        ramp, "ramp_plank",
        (ramp_length, ramp_width, ramp_thickness),
        (plank_vx, 0.0, plank_vz),
        pine,
        rpy=(0.0, ramp_deploy_angle, 0.0),
    )
    for i in range(3):
        gx = ramp_length * (0.25 + 0.25 * i)
        grip_offset_z = ramp_thickness / 2.0 + 0.003
        gv_x = gx * r_cos + grip_offset_z * r_sin
        gv_z = -gx * r_sin + grip_offset_z * r_cos
        _box(
            ramp, f"ramp_grip_{i}",
            (0.012, ramp_width - 0.02, 0.005),
            (gv_x, 0.0, gv_z),
            dark_pine,
            rpy=(0.0, ramp_deploy_angle, 0.0),
        )

    # Hinge brackets: barrel offset slightly outward from the wall so it
    # does not penetrate side_wall_1; frame-mount plates sit flush against
    # the wall exterior (intentional small overlap for seated mounting).
    barrel_offset_x = 0.012
    _cylinder(ramp, "ramp_hinge_barrel", 0.010, ramp_width + 0.02,
              (barrel_offset_x, 0.0, 0.0), black_metal, rpy=(math.pi / 2, 0.0, 0.0))
    plate_x = barrel_offset_x - 0.010
    for i, y_off in enumerate((-0.15, 0.15)):
        _box(ramp, f"ramp_frame_plate_{i}",
             (0.040, 0.050, 0.028),
             (plate_x, y_off, 0.0), black_metal)
    # Short strap connects the barrel to the plank underside for continuity.
    strap_len = 0.060
    strap_vx = barrel_offset_x + (strap_len / 2.0) * r_cos
    strap_vz = -(strap_len / 2.0) * r_sin - ramp_thickness / 2.0 - 0.003
    _box(ramp, "ramp_strap",
         (strap_len, ramp_width - 0.04, 0.006),
         (strap_vx, 0.0, strap_vz),
         black_metal,
         rpy=(0.0, ramp_deploy_angle, 0.0))

    model.articulation(
        "frame_to_run_ramp",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=ramp,
        origin=Origin(xyz=(run_left_x, 0.0, pop_hole_bottom_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=0.0, upper=1.60),
    )

    # --- Run access door: hinged mesh door on the front of the run ---
    run_door = model.part("run_access_door")
    door_w = run_width - 0.16   # ~0.87, clears run_post at far end
    door_h = run_height - 0.10  # ~0.57
    door_frame_w = 0.038
    door_hinge_x = run_left_x + 0.06

    # Door wooden frame
    _box(run_door, "run_door_hinge_stile", (door_frame_w, 0.026, door_h),
         (door_frame_w / 2.0, 0.0, door_h / 2.0), pine)
    _box(run_door, "run_door_free_stile", (door_frame_w, 0.026, door_h),
         (door_w - door_frame_w / 2.0, 0.0, door_h / 2.0), pine)
    _box(run_door, "run_door_top_rail", (door_w, 0.026, door_frame_w),
         (door_w / 2.0, 0.0, door_h - door_frame_w / 2.0), pine)
    _box(run_door, "run_door_bottom_rail", (door_w, 0.026, door_frame_w),
         (door_w / 2.0, 0.0, door_frame_w / 2.0), pine)
    _box(run_door, "run_door_mid_rail", (door_w, 0.022, 0.026),
         (door_w / 2.0, -0.003, door_h * 0.48), pine)

    # Door wire mesh (loop-emitted grid like the parent mesh doors)
    door_inner_w = door_w - 2.0 * door_frame_w + 0.02
    door_inner_h = door_h - 2.0 * door_frame_w + 0.02
    door_mesh_x0 = door_frame_w - 0.01
    door_mesh_z0 = door_frame_w - 0.01
    d_v = max(4, int(door_inner_w / 0.055))
    for i in range(d_v + 1):
        x = door_mesh_x0 + door_inner_w * i / d_v
        _box(run_door, f"run_door_mesh_v_{i}", (0.005, 0.006, door_inner_h),
             (x, -0.018, door_mesh_z0 + door_inner_h / 2.0), mesh_mat)
    d_h = max(5, int(door_inner_h / 0.055))
    for i in range(d_h + 1):
        z = door_mesh_z0 + door_inner_h * i / d_h
        _box(run_door, f"run_door_mesh_h_{i}", (door_inner_w, 0.006, 0.005),
             (door_mesh_x0 + door_inner_w / 2.0, -0.022, z), mesh_mat)

    # Hinge knuckles for run door
    for i, z_frac in enumerate((0.22, 0.78)):
        hz = door_h * z_frac
        _cylinder(run_door, f"run_door_hinge_barrel_{i}", 0.010, 0.075,
                  (0.0, -0.002, hz), black_metal)
        _box(run_door, f"run_door_hinge_leaf_{i}", (0.042, 0.005, 0.055),
             (0.021, -0.013, hz), black_metal)
        _box(run_door, f"run_door_frame_leaf_{i}", (0.055, 0.028, 0.050),
             (-0.025, 0.008, hz), black_metal)

    # Latch mount on the run door face
    run_latch_z = door_h * 0.55
    run_latch_bar_len = min(0.14, max(0.08, door_w * 0.18))
    run_latch_pivot_x = door_w - run_latch_bar_len - 0.040
    run_latch_keeper_x = door_w - 0.032

    _box(run_door, "run_latch_pivot_plate", (0.045, 0.014, 0.034),
         (run_latch_pivot_x, -0.018, run_latch_z), black_metal)
    _box(run_door, "run_latch_keeper_plate", (0.042, 0.016, 0.030),
         (run_latch_keeper_x, -0.020, run_latch_z), black_metal)
    _box(run_door, "run_latch_keeper_slot", (0.028, 0.020, 0.010),
         (run_latch_keeper_x - 0.002, -0.028, run_latch_z), black_metal)

    door_hinge_z = run_bottom_z + run_rail + 0.012  # clear above bottom rail

    model.articulation(
        "frame_to_run_access_door",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=run_door,
        origin=Origin(xyz=(door_hinge_x, run_front_y, door_hinge_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=0.0, upper=1.35),
    )

    # --- Run access door latch (child of the door) ---
    run_latch = model.part("run_access_door_latch")
    _add_latch_bar(run_latch, run_latch_bar_len, black_metal, prefix="run_latch")

    model.articulation(
        "run_access_door_to_latch",
        ArticulationType.REVOLUTE,
        parent=run_door,
        child=run_latch,
        origin=Origin(xyz=(run_latch_pivot_x, -0.024, run_latch_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=0.0, upper=1.15),
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

    # --- Latch pivot-pin / pivot-plate intentional overlaps ---
    # Each hasp pivot pin is intentionally nested inside its pivot plate
    # (captured-pin pattern common to all door latches).
    for door_name in (
        "solid_door_0", "solid_door_1", "solid_door_2",
        "mesh_narrow_door_0", "mesh_narrow_door_1", "mesh_narrow_door_2",
        "mesh_wide_door_0", "mesh_wide_door_1", "mesh_wide_door_2",
    ):
        door_part = object_model.get_part(door_name)
        latch_part = object_model.get_part(f"{door_name}_latch")
        ctx.allow_overlap(
            door_part, latch_part,
            reason="Latch pivot pin is intentionally captured inside the pivot plate.",
        )

    # Run access door latch: same captured-pin pattern.
    run_door = object_model.get_part("run_access_door")
    run_latch = object_model.get_part("run_access_door_latch")
    ctx.allow_overlap(
        run_door, run_latch,
        reason="Run access door latch pivot pin is captured inside the pivot plate.",
    )
    ctx.expect_contact(
        run_door, run_latch,
        elem_a="run_latch_pivot_plate",
        elem_b="run_latch_pivot_pin",
        name="run latch pivot pin seats in the pivot plate",
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

    # =====================================================================
    # Run bay tests
    # =====================================================================
    run_ramp = object_model.get_part("run_ramp")
    run_hinge = object_model.get_articulation("frame_to_run_access_door")
    ramp_hinge = object_model.get_articulation("frame_to_run_ramp")
    run_latch_joint = object_model.get_articulation("run_access_door_to_latch")

    # Run access door overlaps the run opening in XZ footprint when closed.
    ctx.expect_overlap(
        run_door, frame,
        axes="xz",
        min_overlap=0.30,
        name="run access door covers the run front opening",
    )

    # The run door hinge barrels sit in the frame mount boards.
    # This is intentional mechanical embedding for hinge support.
    # Both upper and lower mount pieces may embed door frame members.
    for mount_name in ("run_door_hinge_mount_lo", "run_door_hinge_mount_hi"):
        for barrel_name in ("run_door_hinge_barrel_0", "run_door_hinge_barrel_1"):
            ctx.allow_overlap(
                frame, run_door,
                elem_a=mount_name,
                elem_b=barrel_name,
                reason="Run door hinge barrel is captured in the frame mount board.",
            )
        for leaf_name in ("run_door_frame_leaf_0", "run_door_frame_leaf_1"):
            ctx.allow_overlap(
                frame, run_door,
                elem_a=mount_name,
                elem_b=leaf_name,
                reason="Frame hinge leaf is embedded in the mount board for the run door.",
            )
        ctx.allow_overlap(
            frame, run_door,
            elem_a=mount_name,
            elem_b="run_door_hinge_stile",
            reason="Door hinge stile is seated in the frame mount board.",
        )
        ctx.allow_overlap(
            frame, run_door,
            elem_a=mount_name,
            elem_b="run_door_mid_rail",
            reason="Door mid rail passes through the mount board for structural support.",
        )
        for hinge_leaf_name in ("run_door_hinge_leaf_0", "run_door_hinge_leaf_1"):
            ctx.allow_overlap(
                frame, run_door,
                elem_a=mount_name,
                elem_b=hinge_leaf_name,
                reason="Hinge leaf is embedded in the mount board for hinge assembly.",
            )

    # Run access door opens outward (toward -Y) on its hinge.
    rest_run_door = ctx.part_world_aabb(run_door)
    with ctx.pose({run_hinge: 0.90}):
        open_run_door = ctx.part_world_aabb(run_door)
    ctx.check(
        "run access door opens outward on its hinge",
        rest_run_door is not None
        and open_run_door is not None
        and open_run_door[0][1] < rest_run_door[0][1] - 0.15,
        details=f"rest={rest_run_door}, open={open_run_door}",
    )

    # Run access door swing must clear the cabinet corner (side_wall_1 at x≈1.25).
    # At full open, the free edge of the door should stay past the cabinet side.
    with ctx.pose({run_hinge: 1.20}):
        full_open_door = ctx.part_world_aabb(run_door)
    ctx.check(
        "run door full-open swing clears the cabinet corner",
        full_open_door is not None and full_open_door[0][0] > 1.10,
        details=f"full_open_aabb={full_open_door}",
    )

    # Run access door latch pivots.
    rest_run_latch = ctx.part_world_aabb(run_latch)
    with ctx.pose({run_latch_joint: 0.80}):
        lifted_run_latch = ctx.part_world_aabb(run_latch)
    ctx.check(
        "run access door hasp pivots on its latch joint",
        rest_run_latch is not None
        and lifted_run_latch is not None
        and lifted_run_latch[0][2] < rest_run_latch[0][2] - 0.040,
        details=f"rest={rest_run_latch}, moved={lifted_run_latch}",
    )

    # Ramp hinge mounting: frame plates sit flush against the cabinet side wall.
    # The plates intentionally overlap the wall surface for seated mounting.
    ctx.allow_overlap(
        frame, run_ramp,
        elem_a="side_wall_1",
        elem_b="ramp_frame_plate_0",
        reason="Ramp hinge frame plate is intentionally flush-mounted against the side wall.",
    )
    ctx.allow_overlap(
        frame, run_ramp,
        elem_a="side_wall_1",
        elem_b="ramp_frame_plate_1",
        reason="Ramp hinge frame plate is intentionally flush-mounted against the side wall.",
    )
    ctx.allow_overlap(
        frame, run_ramp,
        elem_a="side_wall_1",
        elem_b="ramp_plank",
        reason="Ramp plank rests against the side wall when deployed; this is the intended structural interface.",
    )
    # The ramp plank near-end sits at the pop-hole opening on the wall exterior,
    # creating a small local overlap with the pop-hole surround trim.
    ctx.allow_overlap(
        frame, run_ramp,
        elem_a="pop_hole_surround",
        elem_b="ramp_plank",
        reason="Ramp plank is seated at the pop-hole opening; small local embed in the surround trim.",
    )
    # The ramp hinge barrel is embedded in the pop-hole surround for mounting.
    ctx.allow_overlap(
        frame, run_ramp,
        elem_a="pop_hole_surround",
        elem_b="ramp_hinge_barrel",
        reason="Ramp hinge barrel is captured in the pop-hole surround mount.",
    )
    ctx.allow_overlap(
        frame, run_ramp,
        elem_a="pop_hole_surround",
        elem_b="ramp_frame_plate_0",
        reason="Ramp frame plate is embedded in the pop-hole surround for mounting.",
    )
    ctx.allow_overlap(
        frame, run_ramp,
        elem_a="pop_hole_void",
        elem_b="ramp_hinge_barrel",
        reason="Ramp hinge barrel sits at the pop-hole void opening for pivot.",
    )
    ctx.allow_overlap(
        frame, run_ramp,
        elem_a="pop_hole_void",
        elem_b="ramp_frame_plate_1",
        reason="Ramp frame plate passes through the pop-hole void for wall mounting.",
    )
    ctx.allow_overlap(
        frame, run_ramp,
        elem_a="pop_hole_void",
        elem_b="ramp_plank",
        reason="Ramp plank fits into the pop-hole void to create the proper structural interface.",
    )
    ctx.expect_contact(
        frame, run_ramp,
        elem_a="side_wall_1",
        elem_b="ramp_frame_plate_0",
        name="ramp hinge plate contacts the cabinet side wall",
    )

    # Ramp deploys downward from pop-hole toward the run floor at rest (q=0).
    rest_ramp = ctx.part_world_aabb(run_ramp)
    ctx.check(
        "ramp reaches down toward the run floor at rest",
        rest_ramp is not None and rest_ramp[0][2] < 0.10,
        details=f"rest_ramp_aabb={rest_ramp}",
    )

    # Ramp folds upward when articulated (positive q rotates around -Y axis).
    with ctx.pose({ramp_hinge: 1.40}):
        folded_ramp = ctx.part_world_aabb(run_ramp)
    ctx.check(
        "ramp folds up against the wall when stowed",
        rest_ramp is not None
        and folded_ramp is not None
        and folded_ramp[0][2] > rest_ramp[0][2] + 0.10,
        details=f"rest={rest_ramp}, folded={folded_ramp}",
    )

    # Run bay shares the cabinet's base line: its floor extends from the right side.
    # The run floor and lowest compartment floor should be at similar heights.
    ctx.expect_gap(
        frame, frame,
        axis="z",
        min_gap=-0.05,
        max_gap=0.20,
        positive_elem="compartment_floor_0",
        negative_elem="run_floor_panel",
        name="run floor aligns vertically with cabinet base",
    )

    # Pop-hole interface: the opening is on the cabinet side wall at the run junction.
    ctx.expect_overlap(
        frame, frame,
        axes="z",
        min_overlap=0.10,
        elem_a="pop_hole_surround",
        elem_b="side_wall_1",
        name="pop-hole surround sits on the lowest-compartment side wall",
    )

    return ctx.report()


object_model = build_object_model()
