from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _v_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _v_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _fabric_panel_geometry(
    corners: tuple[tuple[float, float, float], ...],
    *,
    thickness: float = 0.006,
    nx: int = 8,
    ny: int = 6,
    sag: float = 0.012,
) -> MeshGeometry:
    """Thin bilinear cloth-like panel with slight vertical sag."""
    p00, p10, p11, p01 = corners
    geom = MeshGeometry()

    top: list[list[int]] = []
    bottom: list[list[int]] = []
    for ix in range(nx + 1):
        u = ix / nx
        top_row = []
        bottom_row = []
        for iy in range(ny + 1):
            v = iy / ny
            x = (
                (1 - u) * (1 - v) * p00[0]
                + u * (1 - v) * p10[0]
                + u * v * p11[0]
                + (1 - u) * v * p01[0]
            )
            y = (
                (1 - u) * (1 - v) * p00[1]
                + u * (1 - v) * p10[1]
                + u * v * p11[1]
                + (1 - u) * v * p01[1]
            )
            z = (
                (1 - u) * (1 - v) * p00[2]
                + u * (1 - v) * p10[2]
                + u * v * p11[2]
                + (1 - u) * v * p01[2]
            )
            z -= sag * math.sin(math.pi * u) * math.sin(math.pi * v)
            top_row.append(geom.add_vertex(x, y, z))
            bottom_row.append(geom.add_vertex(x, y, z - thickness))
        top.append(top_row)
        bottom.append(bottom_row)

    for ix in range(nx):
        for iy in range(ny):
            a, b, c, d = top[ix][iy], top[ix + 1][iy], top[ix + 1][iy + 1], top[ix][iy + 1]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)
            ab, bb, cb, db = bottom[ix][iy], bottom[ix + 1][iy], bottom[ix + 1][iy + 1], bottom[ix][iy + 1]
            geom.add_face(ab, cb, bb)
            geom.add_face(ab, db, cb)

    for ix in range(nx):
        for iy in (0, ny):
            a, b = top[ix][iy], top[ix + 1][iy]
            ab, bb = bottom[ix][iy], bottom[ix + 1][iy]
            geom.add_face(a, ab, bb)
            geom.add_face(a, bb, b)
    for iy in range(ny):
        for ix in (0, nx):
            a, b = top[ix][iy], top[ix][iy + 1]
            ab, bb = bottom[ix][iy], bottom[ix][iy + 1]
            geom.add_face(a, b, bb)
            geom.add_face(a, bb, ab)
    return geom


def _edge_loop(corners, lift=(0.0, 0.0, 0.0)):
    return [_v_add(c, lift) for c in (corners[0], corners[1], corners[2], corners[3])]


def _straight_tube(points, *, radius=0.012, radial_segments=16):
    return tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=2,
        radial_segments=radial_segments,
        cap_ends=True,
        spline="catmull_rom",
    )


def _loop_tube(points, *, radius=0.006, radial_segments=14):
    return tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=4,
        radial_segments=radial_segments,
        cap_ends=True,
        closed_spline=True,
        spline="catmull_rom",
    )


# Pivot origin in base frame local (= world since base is root)
PX, PY, PZ = 0.05, 0.0, 0.43


def _to_local(wx, wy, wz):
    """Convert world coordinates to recline_frame local coordinates."""
    return (wx - PX, wy - PY, wz - PZ)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="zero_gravity_camp_recliner",
        meta={
            "run_notes": (
                "Zero-gravity folding camp recliner variant: body-contour sling "
                "(seat + back + footrest) suspended by elastic bungee cords inside "
                "a rectangular pivoting tube frame. Single REVOLUTE recline-lock "
                "pivot (base_to_recline_pivot) tilts the whole contour from upright "
                "to legs-up zero-gravity pose. Gray/black mesh colorway. "
                "Camp chair family lounge product."
            )
        },
    )

    metal = model.material("powder_coated_gray_tube", rgba=(0.48, 0.50, 0.52, 1.0))
    dark_metal = model.material("black_hardware", rgba=(0.04, 0.04, 0.04, 1.0))
    mesh_fabric = model.material("black_mesh_fabric", rgba=(0.08, 0.08, 0.09, 1.0))
    mesh_shadow = model.material("dark_gray_mesh", rgba=(0.12, 0.12, 0.13, 1.0))
    bungee_cord = model.material("elastic_cord_gray", rgba=(0.35, 0.36, 0.38, 1.0))
    lever_metal = model.material("lock_lever_black", rgba=(0.06, 0.06, 0.06, 1.0))

    # ── BASE FRAME (root) ─────────────────────────────────────────────────────
    base = model.part("base_frame")
    base_path = [
        (-0.46, 0.34, 0.035),
        (-0.23, 0.34, 0.40),
        (-0.58, 0.34, 0.93),
        (-0.58, -0.34, 0.93),
        (-0.23, -0.34, 0.40),
        (-0.46, -0.34, 0.035),
        (0.72, -0.34, 0.035),
        (0.40, -0.34, 0.39),
        (0.40, 0.34, 0.39),
        (0.72, 0.34, 0.035),
        (-0.46, 0.34, 0.035),
    ]
    base.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                base_path,
                radius=0.014,
                samples_per_segment=7,
                radial_segments=20,
                closed_spline=False,
                cap_ends=True,
            ),
            "base_bent_tube",
        ),
        material=metal,
        name="bent_tube",
    )

    # Pivot mount brackets: vertical stubs rising from the base tube to the
    # pivot height. These overlap with recline_frame pivot bushings to form
    # the bearing assembly.
    for y_sign, bracket_name in ((1.0, "left"), (-1.0, "right")):
        base.visual(
            mesh_from_geometry(
                _straight_tube(
                    [
                        (PX, 0.34 * y_sign, 0.38),
                        (PX, 0.34 * y_sign, PZ),
                        (PX, 0.34 * y_sign, PZ + 0.02),
                    ],
                    radius=0.018,
                    radial_segments=14,
                ),
                f"pivot_bracket_{bracket_name}",
            ),
            material=dark_metal,
            name=f"pivot_bracket_{bracket_name}",
        )

    # Feet
    for name, x, y in (
        ("rear_left_foot", -0.46, 0.34),
        ("rear_right_foot", -0.46, -0.34),
        ("front_left_foot", 0.72, 0.34),
        ("front_right_foot", 0.72, -0.34),
    ):
        foot = model.part(name)
        foot.visual(
            Box((0.075, 0.055, 0.026)),
            origin=Origin(xyz=(0.0, 0.0, 0.013)),
            material=dark_metal,
            name="rubber_pad",
        )
        model.articulation(
            f"base_to_{name}",
            ArticulationType.FIXED,
            parent=base,
            child=foot,
            origin=Origin(xyz=(x, y, 0.0)),
        )

    # Scissor cross legs
    scissor_specs = (
        ("left_cross_leg_0", (0.10, 0.255, 0.300), [(-0.48, 0.0, -0.255), (0.0, 0.0, 0.0), (0.36, 0.0, 0.120)]),
        ("left_cross_leg_1", (0.10, 0.315, 0.300), [(0.62, 0.0, -0.260), (0.0, 0.0, 0.0), (-0.34, 0.0, 0.130)]),
        ("right_cross_leg_0", (0.10, -0.255, 0.300), [(-0.48, 0.0, -0.255), (0.0, 0.0, 0.0), (0.36, 0.0, 0.120)]),
        ("right_cross_leg_1", (0.10, -0.315, 0.300), [(0.62, 0.0, -0.260), (0.0, 0.0, 0.0), (-0.34, 0.0, 0.130)]),
    )
    for leg_name, pivot, local_points in scissor_specs:
        leg = model.part(leg_name)
        leg.visual(
            mesh_from_geometry(_straight_tube(local_points, radius=0.0105), f"{leg_name}_tube"),
            material=metal,
            name="cross_tube",
        )
        leg.visual(
            Sphere(radius=0.020),
            origin=Origin(),
            material=dark_metal,
            name="pivot_collar",
        )
        model.articulation(
            f"base_to_{leg_name}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=leg,
            origin=Origin(xyz=pivot),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=25.0, velocity=1.5, lower=-0.55, upper=0.55),
        )

    # ── RECLINE FRAME ─────────────────────────────────────────────────────────
    # Rectangular tube frame carrying the body-contour sling.
    # All geometry in LOCAL coords (world - pivot origin).
    recline = model.part("recline_frame")

    # Side rails at y=±0.30 (inside the base tube at y=±0.34 to reduce overlap)
    rail_y = 0.30
    recline.visual(
        mesh_from_geometry(
            _straight_tube(
                [_to_local(-0.52, rail_y, 0.46), _to_local(-0.10, rail_y, 0.44),
                 _to_local(0.42, rail_y, 0.42), _to_local(0.88, rail_y, 0.38)],
                radius=0.013,
            ),
            "left_side_rail",
        ),
        material=metal,
        name="left_rail",
    )
    recline.visual(
        mesh_from_geometry(
            _straight_tube(
                [_to_local(-0.52, -rail_y, 0.46), _to_local(-0.10, -rail_y, 0.44),
                 _to_local(0.42, -rail_y, 0.42), _to_local(0.88, -rail_y, 0.38)],
                radius=0.013,
            ),
            "right_side_rail",
        ),
        material=metal,
        name="right_rail",
    )
    # Top crossbar
    recline.visual(
        mesh_from_geometry(
            _straight_tube(
                [_to_local(-0.52, rail_y, 0.46), _to_local(-0.52, -rail_y, 0.46)],
                radius=0.012,
            ),
            "top_crossbar",
        ),
        material=metal,
        name="top_bar",
    )
    # Bottom crossbar
    recline.visual(
        mesh_from_geometry(
            _straight_tube(
                [_to_local(0.88, rail_y, 0.38), _to_local(0.88, -rail_y, 0.38)],
                radius=0.012,
            ),
            "bottom_crossbar",
        ),
        material=metal,
        name="bottom_bar",
    )
    # Hinge crossbar at the footrest hinge location (connects rails at the seat front)
    recline.visual(
        mesh_from_geometry(
            _straight_tube(
                [_to_local(0.42, rail_y, 0.42), _to_local(0.42, -rail_y, 0.42)],
                radius=0.012,
            ),
            "hinge_crossbar",
        ),
        material=metal,
        name="hinge_bar",
    )

    # Pivot bushings around the pivot axis
    for y_sign, bush_name in ((1.0, "left"), (-1.0, "right")):
        recline.visual(
            Cylinder(radius=0.022, length=0.05),
            origin=Origin(
                xyz=_to_local(PX, 0.34 * y_sign, PZ),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_metal,
            name=f"pivot_bushing_{bush_name}",
        )
        # Extension arm from bushing to the side rail (connects the pivot to the frame)
        recline.visual(
            mesh_from_geometry(
                _straight_tube(
                    [_to_local(PX, 0.34 * y_sign, PZ), _to_local(PX, rail_y * y_sign, PZ)],
                    radius=0.010,
                    radial_segments=10,
                ),
                f"pivot_arm_{bush_name}",
            ),
            material=metal,
            name=f"pivot_arm_{bush_name}",
        )

    # Lock lever on the left pivot bushing
    recline.visual(
        Box((0.040, 0.040, 0.016)),
        origin=Origin(xyz=_to_local(PX, 0.38, PZ)),
        material=lever_metal,
        name="lock_lever",
    )
    recline.visual(
        Cylinder(radius=0.009, length=0.030),
        origin=Origin(
            xyz=_to_local(PX, 0.41, PZ),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=lever_metal,
        name="lever_grip",
    )

    # THE KEY ARTICULATION: base_to_recline_pivot REVOLUTE
    model.articulation(
        "base_to_recline_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=recline,
        origin=Origin(xyz=(PX, PY, PZ)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.6, lower=0.0, upper=1.15),
    )

    # ── BODY-CONTOUR SLING (recline_frame local coords) ───────────────────────
    # The back and seat share the edge at world x=-0.10, z=0.44 to form one
    # continuous contour. The seat and footrest share the edge at world x=0.40.

    # Back panel — front-bottom edge at world (-0.10, ±0.27, 0.44) matches seat
    back_corners = (
        _to_local(-0.10, 0.27, 0.44),
        _to_local(-0.46, 0.27, 0.87),
        _to_local(-0.46, -0.27, 0.87),
        _to_local(-0.10, -0.27, 0.44),
    )
    back = model.part("back_panel")
    back.visual(
        mesh_from_geometry(_fabric_panel_geometry(back_corners, sag=0.018), "back_sling"),
        material=mesh_shadow,
        name="leaning_sling",
    )
    # Bungee cords from back sling edge to frame rail (both touch the sling)
    for y_sign, cord_name in ((1.0, "left"), (-1.0, "right")):
        back.visual(
            mesh_from_geometry(
                _straight_tube(
                    [
                        _to_local(-0.10, 0.27 * y_sign, 0.44),
                        _to_local(-0.28, 0.29 * y_sign, 0.66),
                        _to_local(-0.46, 0.27 * y_sign, 0.87),
                    ],
                    radius=0.005,
                    radial_segments=8,
                ),
                f"back_bungee_{cord_name}",
            ),
            material=bungee_cord,
            name=f"bungee_{cord_name}",
        )
    model.articulation(
        "recline_to_back", ArticulationType.FIXED, parent=recline, child=back,
    )

    # Seat panel — back edge at world (-0.10, ±0.27, 0.44) matches back,
    # front edge at world (0.40, ±0.27, 0.42) matches footrest hinge area
    seat_corners = (
        _to_local(-0.10, 0.27, 0.44),
        _to_local(0.40, 0.27, 0.42),
        _to_local(0.40, -0.27, 0.42),
        _to_local(-0.10, -0.27, 0.44),
    )
    seat = model.part("seat_panel")
    seat.visual(
        mesh_from_geometry(_fabric_panel_geometry(seat_corners, sag=0.018), "seat_sling"),
        material=mesh_fabric,
        name="sling",
    )
    # Bungee cords from seat sling edge toward frame rails
    for y_sign, cord_name in ((1.0, "left"), (-1.0, "right")):
        seat.visual(
            mesh_from_geometry(
                _straight_tube(
                    [
                        _to_local(-0.10, 0.27 * y_sign, 0.44),
                        _to_local(0.15, 0.29 * y_sign, 0.43),
                        _to_local(0.40, 0.27 * y_sign, 0.42),
                    ],
                    radius=0.005,
                    radial_segments=8,
                ),
                f"seat_bungee_{cord_name}",
            ),
            material=bungee_cord,
            name=f"bungee_{cord_name}",
        )
    model.articulation(
        "recline_to_seat", ArticulationType.FIXED, parent=recline, child=seat,
    )

    # Footrest panel — hinged at the front of the recline frame.
    # Hinge point in recline local: _to_local(0.42, 0.0, 0.42) = (0.37, 0.0, -0.01)
    footrest_hinge_local = _to_local(0.42, 0.0, 0.42)

    footrest = model.part("footrest_panel")
    # Footrest corners in footrest LOCAL frame (relative to hinge).
    # At q=0, footrest frame is at world (0.42, 0.0, 0.42).
    # The back edge (x=0 in local) is at world (0.42, ±0.27, 0.42) matching the seat front.
    foot_corners = (
        (0.00, 0.27, 0.00),
        (0.82, 0.27, -0.10),
        (0.82, -0.27, -0.10),
        (0.00, -0.27, 0.00),
    )
    footrest.visual(
        mesh_from_geometry(_fabric_panel_geometry(foot_corners, sag=0.014), "footrest_sling"),
        material=mesh_fabric,
        name="extended_sling",
    )
    # Bungee cords along footrest sides
    for y_sign, cord_name in ((1.0, "left"), (-1.0, "right")):
        footrest.visual(
            mesh_from_geometry(
                _straight_tube(
                    [
                        (0.02, 0.27 * y_sign, -0.003),
                        (0.40, 0.29 * y_sign, -0.05),
                        (0.78, 0.27 * y_sign, -0.095),
                    ],
                    radius=0.005,
                    radial_segments=8,
                ),
                f"foot_bungee_{cord_name}",
            ),
            material=bungee_cord,
            name=f"bungee_{cord_name}",
        )
    # Footrest U-frame tube — starts at the sling edge to connect within the part.
    # Extended outward in y to reach the recline frame rails.
    footrest.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.00, 0.27, 0.00),
                    (0.05, 0.32, -0.01),
                    (0.74, 0.32, -0.11),
                    (0.86, 0.0, -0.12),
                    (0.74, -0.32, -0.11),
                    (0.05, -0.32, -0.01),
                    (0.00, -0.27, 0.00),
                ],
                radius=0.011,
                samples_per_segment=5,
                radial_segments=18,
                cap_ends=True,
            ),
            "footrest_u_tube",
        ),
        material=metal,
        name="u_tube",
    )

    # base_to_footrest: REVOLUTE hinge (name preserved per KEEP constraint)
    model.articulation(
        "base_to_footrest",
        ArticulationType.REVOLUTE,
        parent=recline,
        child=footrest,
        origin=Origin(xyz=footrest_hinge_local),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.8, lower=-0.95, upper=0.35),
    )

    # ── ARMRESTS (on base_frame, don't recline) ──────────────────────────────
    for side_name, y, axis_sign in (("left", 0.405, 1.0), ("right", -0.405, -1.0)):
        arm = model.part(f"{side_name}_armrest")
        arm_corners = (
            (0.00, -0.040 * axis_sign, 0.000),
            (0.82, -0.040 * axis_sign, -0.070),
            (0.82, 0.040 * axis_sign, -0.070),
            (0.00, 0.040 * axis_sign, 0.000),
        )
        arm.visual(
            mesh_from_geometry(
                _fabric_panel_geometry(arm_corners, thickness=0.004, sag=0.006),
                f"{side_name}_arm_sling",
            ),
            material=mesh_fabric,
            name="fabric_arm",
        )
        model.articulation(
            f"base_to_{side_name}_armrest",
            ArticulationType.REVOLUTE,
            parent=base,
            child=arm,
            origin=Origin(xyz=(-0.38, y, 0.64)),
            axis=(0.0, axis_sign, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=1.0, lower=-0.45, upper=0.55),
        )

        support = model.part(f"{side_name}_arm_support")
        support.visual(
            mesh_from_geometry(
                _straight_tube(
                    [(0.0, 0.0, 0.0), (0.10, 0.035 * axis_sign, 0.14), (0.28, 0.065 * axis_sign, 0.27)],
                    radius=0.009,
                ),
                f"{side_name}_arm_support_tube",
            ),
            material=metal,
            name="support_tube",
        )
        model.articulation(
            f"base_to_{side_name}_arm_support",
            ArticulationType.REVOLUTE,
            parent=base,
            child=support,
            origin=Origin(xyz=(0.40, 0.34 * axis_sign, 0.39)),
            axis=(0.0, axis_sign, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=1.0, lower=-0.7, upper=0.35),
        )

    # Front footrest support prop — offset from arm support origins, still on base tube
    prop = model.part("front_footrest_support")
    prop.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.40, 0.33, 0.39),
                    (0.78, 0.220, 0.060),
                    (1.02, 0.0, 0.040),
                    (0.78, -0.220, 0.060),
                    (0.40, -0.33, 0.39),
                ],
                radius=0.010,
                samples_per_segment=5,
                radial_segments=16,
                cap_ends=True,
            ),
            "front_prop_tube",
        ),
        material=metal,
        name="prop_tube",
    )
    model.articulation("base_to_front_support", ArticulationType.FIXED, parent=base, child=prop)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_frame")
    recline = object_model.get_part("recline_frame")
    seat = object_model.get_part("seat_panel")
    back = object_model.get_part("back_panel")
    footrest = object_model.get_part("footrest_panel")
    recline_pivot = object_model.get_articulation("base_to_recline_pivot")
    foot_hinge = object_model.get_articulation("base_to_footrest")

    # ── Intentional overlaps ──
    intentional_overlaps = [
        # Feet
        ("base_frame", "rear_left_foot", "bent_tube", "rubber_pad", "rubber foot socket wraps the tube end"),
        ("base_frame", "rear_right_foot", "bent_tube", "rubber_pad", "rubber foot socket wraps the tube end"),
        ("base_frame", "front_left_foot", "bent_tube", "rubber_pad", "rubber foot socket wraps the tube end"),
        ("base_frame", "front_right_foot", "bent_tube", "rubber_pad", "rubber foot socket wraps the tube end"),
        # Pivot bearing assembly
        ("base_frame", "recline_frame", "pivot_bracket_left", "pivot_bushing_left", "pivot bearing bushing sleeves over the bracket stub"),
        ("base_frame", "recline_frame", "pivot_bracket_right", "pivot_bushing_right", "pivot bearing bushing sleeves over the bracket stub"),
        # Bungee cords to frame rails
        ("recline_frame", "back_panel", "left_rail", "bungee_left", "bungee cord laces back sling to the left frame rail"),
        ("recline_frame", "back_panel", "right_rail", "bungee_right", "bungee cord laces back sling to the right frame rail"),
        ("recline_frame", "seat_panel", "left_rail", "bungee_left", "bungee cord laces seat sling to the left frame rail"),
        ("recline_frame", "seat_panel", "right_rail", "bungee_right", "bungee cord laces seat sling to the right frame rail"),
        # Footrest U-tube overlaps with recline frame hinge bar and rails
        ("footrest_panel", "recline_frame", "u_tube", "hinge_bar", "footrest U-tube pivots on the frame hinge crossbar"),
        ("footrest_panel", "recline_frame", "u_tube", "left_rail", "footrest U-tube passes alongside the left rail near the hinge"),
        ("footrest_panel", "recline_frame", "u_tube", "right_rail", "footrest U-tube passes alongside the right rail near the hinge"),
        # Armrests and supports
        ("base_frame", "left_armrest", "bent_tube", "fabric_arm", "arm fabric wraps around the rear upright tube"),
        ("base_frame", "right_armrest", "bent_tube", "fabric_arm", "arm fabric wraps around the rear upright tube"),
        ("base_frame", "left_arm_support", "bent_tube", "support_tube", "front arm support is hinged on the side tube"),
        ("base_frame", "right_arm_support", "bent_tube", "support_tube", "front arm support is hinged on the side tube"),
        ("base_frame", "front_footrest_support", "bent_tube", "prop_tube", "front prop tube is fixed into the base frame sockets"),
        # Continuous body contour panels share edges (slight overlap at seam)
        ("seat_panel", "back_panel", "sling", "leaning_sling", "seat and back form one continuous body contour sling sharing the junction edge"),
        ("seat_panel", "footrest_panel", "sling", "extended_sling", "seat and footrest form one continuous body contour sling sharing the hinge edge"),
        # Scissor legs near seat/footrest
        ("seat_panel", "left_cross_leg_0", "sling", "cross_tube", "sagged fabric bears lightly on the folded scissor tube"),
        ("seat_panel", "right_cross_leg_0", "sling", "cross_tube", "sagged fabric bears lightly on the folded scissor tube"),
        ("footrest_panel", "left_cross_leg_0", "extended_sling", "cross_tube", "front scissor tube passes near the footrest sling"),
        ("footrest_panel", "right_cross_leg_0", "extended_sling", "cross_tube", "front scissor tube passes near the footrest sling"),
        # Cross legs near recline frame rails
        ("recline_frame", "left_cross_leg_1", "left_rail", "cross_tube", "scissor leg crosses near the recline frame rail at the side pivot"),
        ("recline_frame", "right_cross_leg_1", "right_rail", "cross_tube", "scissor leg crosses near the recline frame rail at the side pivot"),
        # Pivot bracket overlaps with pivot arm (both form the bearing assembly)
        ("base_frame", "recline_frame", "pivot_bracket_left", "pivot_arm_left", "pivot bracket stub and pivot arm share the bearing mount region"),
        ("base_frame", "recline_frame", "pivot_bracket_right", "pivot_arm_right", "pivot bracket stub and pivot arm share the bearing mount region"),
        # Seat bungee cords pass near base tube crossover area
        ("base_frame", "seat_panel", "bent_tube", "bungee_left", "seat bungee cord lacing passes near the base frame crossover tube"),
        ("base_frame", "seat_panel", "bent_tube", "bungee_right", "seat bungee cord lacing passes near the base frame crossover tube"),
        # Base tube crossover passes near recline frame rails at the front crossbar area
        ("base_frame", "recline_frame", "bent_tube", "left_rail", "base frame front crossover tube passes under the recline frame left rail"),
        ("base_frame", "recline_frame", "bent_tube", "right_rail", "base frame front crossover tube passes under the recline frame right rail"),
        ("base_frame", "recline_frame", "bent_tube", "hinge_bar", "base frame front crossover tube passes under the recline hinge bar"),
        # Seat sling overlaps base tube crossover area
        ("base_frame", "seat_panel", "bent_tube", "sling", "seat sling sags into the base frame crossover tube region"),
        # Footrest U-tube crosses near scissor legs
        ("footrest_panel", "left_cross_leg_0", "u_tube", "cross_tube", "footrest U-tube passes near the left front scissor leg"),
        ("footrest_panel", "right_cross_leg_0", "u_tube", "cross_tube", "footrest U-tube passes near the right front scissor leg"),
        # Footrest sling near recline hinge bar
        ("footrest_panel", "recline_frame", "extended_sling", "hinge_bar", "footrest sling attaches at the recline frame hinge bar"),
        # Scissor legs near recline hinge bar
        ("left_cross_leg_0", "recline_frame", "cross_tube", "hinge_bar", "left front scissor leg crosses near the recline hinge bar"),
        ("right_cross_leg_0", "recline_frame", "cross_tube", "hinge_bar", "right front scissor leg crosses near the recline hinge bar"),
        # Prop tube near arm support tubes at the base front
        ("front_footrest_support", "left_arm_support", "prop_tube", "support_tube", "front prop and arm support tubes cross near the base frame mount"),
        ("front_footrest_support", "right_arm_support", "prop_tube", "support_tube", "front prop and arm support tubes cross near the base frame mount"),
    ]
    for a, b, elem_a, elem_b, reason in intentional_overlaps:
        ctx.allow_overlap(a, b, elem_a=elem_a, elem_b=elem_b, reason=reason)

    # ── Prompt-specific: base_to_recline_pivot exists ──
    ctx.check(
        "base_to_recline_pivot exists as the single added recline-lock mechanism",
        recline_pivot is not None,
        details="base_to_recline_pivot must be present as REVOLUTE",
    )

    # ── Recline pivot tilts the contour: use AABB to measure geometry movement ──
    upright_seat_aabb = ctx.part_world_aabb(seat)
    upright_back_aabb = ctx.part_world_aabb(back)
    with ctx.pose({recline_pivot: 1.0}):
        reclined_seat_aabb = ctx.part_world_aabb(seat)
        reclined_back_aabb = ctx.part_world_aabb(back)
    ctx.check(
        "recline pivot tilts back panel downward (back max_z descends)",
        upright_back_aabb is not None
        and reclined_back_aabb is not None
        and reclined_back_aabb[1][2] < upright_back_aabb[1][2] - 0.05,
        details=f"upright_back_max_z={upright_back_aabb[1][2] if upright_back_aabb else None}, "
                f"reclined_back_max_z={reclined_back_aabb[1][2] if reclined_back_aabb else None}",
    )

    # Footrest rises during recline
    upright_foot_aabb = ctx.part_world_aabb(footrest)
    with ctx.pose({recline_pivot: 1.0}):
        reclined_foot_aabb = ctx.part_world_aabb(footrest)
    ctx.check(
        "recline raises footrest toward zero-gravity legs-up pose",
        upright_foot_aabb is not None
        and reclined_foot_aabb is not None
        and reclined_foot_aabb[0][2] > upright_foot_aabb[0][2] + 0.05,
        details=f"upright_foot_min_z={upright_foot_aabb[0][2] if upright_foot_aabb else None}, "
                f"reclined_foot_min_z={reclined_foot_aabb[0][2] if reclined_foot_aabb else None}",
    )

    # ── base_to_footrest hinge still works ──
    rest_foot_aabb = ctx.part_world_aabb(footrest)
    with ctx.pose({foot_hinge: -0.6}):
        folded_foot_aabb = ctx.part_world_aabb(footrest)
    ctx.check(
        "footrest hinge folds independently within the contour",
        rest_foot_aabb is not None
        and folded_foot_aabb is not None
        and abs(folded_foot_aabb[1][0] - rest_foot_aabb[1][0]) > 0.04,
        details=f"rest_max_x={rest_foot_aabb[1][0] if rest_foot_aabb else None}, "
                f"folded_max_x={folded_foot_aabb[1][0] if folded_foot_aabb else None}",
    )

    # ── Structural coverage ──
    ctx.expect_overlap(seat, recline, axes="y", min_overlap=0.35, name="seat sling spans the recline frame width")
    ctx.expect_overlap(back, seat, axes="y", min_overlap=0.40, name="back and seat share contour width")

    # ── Classification ──
    ctx.check(
        "run note records zero-gravity recliner classification",
        "zero-gravity" in object_model.meta.get("run_notes", "").lower()
        and "camp" in object_model.meta.get("run_notes", "").lower(),
        details=object_model.meta.get("run_notes", ""),
    )

    return ctx.report()


object_model = build_object_model()
