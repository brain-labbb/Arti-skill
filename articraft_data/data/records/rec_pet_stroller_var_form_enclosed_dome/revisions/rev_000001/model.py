from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    Mimic,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mat(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name, rgba=rgba)


def _add_tube(part, points, radius: float, name: str, material) -> None:
    """Add a smooth structural tube visual in object units (meters)."""
    geom = tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=10,
        radial_segments=16,
        cap_ends=True,
    )
    part.visual(mesh_from_geometry(geom, name), material=material, name=name)


def _curved_fabric_panel(path, width: float, thickness: float = 0.006) -> MeshGeometry:
    """A thin barrel-vault fabric sheet extruded across stroller width."""
    geom = MeshGeometry()
    rows = []
    half = width / 2.0
    for x, z in path:
        # Four vertices per station: outer left/right, inner left/right.
        rows.append(
            (
                geom.add_vertex(x, -half, z),
                geom.add_vertex(x, half, z),
                geom.add_vertex(x, -half, z - thickness),
                geom.add_vertex(x, half, z - thickness),
            )
        )
    for i in range(len(rows) - 1):
        a0, a1, a2, a3 = rows[i]
        b0, b1, b2, b3 = rows[i + 1]
        # Outer skin.
        geom.add_face(a0, b0, b1)
        geom.add_face(a0, b1, a1)
        # Inner skin.
        geom.add_face(a2, a3, b3)
        geom.add_face(a2, b3, b2)
        # Side hems.
        geom.add_face(a0, a2, b2)
        geom.add_face(a0, b2, b0)
        geom.add_face(a1, b1, b3)
        geom.add_face(a1, b3, a3)
    # Front and rear stitched edges.
    for row in (rows[0], rows[-1]):
        a0, a1, a2, a3 = row
        geom.add_face(a0, a1, a3)
        geom.add_face(a0, a3, a2)
    return geom


def _bezier(p0, p1, p2, p3, steps: int) -> list[tuple[float, float]]:
    pts = []
    for i in range(steps + 1):
        t = i / steps
        u = 1.0 - t
        x = (
            u * u * u * p0[0]
            + 3 * u * u * t * p1[0]
            + 3 * u * t * t * p2[0]
            + t * t * t * p3[0]
        )
        z = (
            u * u * u * p0[1]
            + 3 * u * u * t * p1[1]
            + 3 * u * t * t * p2[1]
            + t * t * t * p3[1]
        )
        pts.append((x, z))
    return pts


def _dome_side_wall(arch_points, y, bottom_z: float, thickness: float = 0.003) -> MeshGeometry:
    """Thin vertical mesh wall at fixed Y, following a curved arch top edge down to a flat bottom.
    Thickness extends toward the dome center (y=0)."""
    sign = -1.0 if y > 0 else 1.0
    y2 = y + sign * thickness
    geom = MeshGeometry()
    n = len(arch_points)
    outer_top, inner_top, outer_bot, inner_bot = [], [], [], []
    for x, z in arch_points:
        outer_top.append(geom.add_vertex(x, y, z))
        inner_top.append(geom.add_vertex(x, y2, z))
        outer_bot.append(geom.add_vertex(x, y, bottom_z))
        inner_bot.append(geom.add_vertex(x, y2, bottom_z))
    for i in range(n - 1):
        ot0, ot1 = outer_top[i], outer_top[i + 1]
        it0, it1 = inner_top[i], inner_top[i + 1]
        ob0, ob1 = outer_bot[i], outer_bot[i + 1]
        ib0, ib1 = inner_bot[i], inner_bot[i + 1]
        # Outer face
        geom.add_face(ot0, ot1, ob1)
        geom.add_face(ot0, ob1, ob0)
        # Inner face
        geom.add_face(it0, ib1, it1)
        geom.add_face(it0, ib0, ib1)
        # Top edge strip
        geom.add_face(ot0, it0, it1)
        geom.add_face(ot0, it1, ot1)
        # Bottom edge strip
        geom.add_face(ob0, ob1, ib1)
        geom.add_face(ob0, ib1, ib0)
    # End caps at rear and front
    geom.add_face(outer_top[0], outer_bot[0], inner_bot[0])
    geom.add_face(outer_top[0], inner_bot[0], inner_top[0])
    geom.add_face(outer_top[-1], inner_top[-1], inner_bot[-1])
    geom.add_face(outer_top[-1], inner_bot[-1], outer_bot[-1])
    return geom


def _dome_front_wall(x, z_top, z_bottom, half_width: float, thickness: float = 0.003) -> MeshGeometry:
    """Flat rectangular mesh panel for the dome front wall at fixed X.
    Thickness extends toward the dome rear (-X)."""
    x2 = x - thickness
    geom = MeshGeometry()
    otl = geom.add_vertex(x, -half_width, z_top)
    otr = geom.add_vertex(x, half_width, z_top)
    obl = geom.add_vertex(x, -half_width, z_bottom)
    obr = geom.add_vertex(x, half_width, z_bottom)
    itl = geom.add_vertex(x2, -half_width, z_top)
    itr = geom.add_vertex(x2, half_width, z_top)
    ibl = geom.add_vertex(x2, -half_width, z_bottom)
    ibr = geom.add_vertex(x2, half_width, z_bottom)
    # Outer face
    geom.add_face(otl, otr, obr)
    geom.add_face(otl, obr, obl)
    # Inner face
    geom.add_face(itl, ibr, itr)
    geom.add_face(itl, ibl, ibr)
    # Top edge
    geom.add_face(otl, itl, itr)
    geom.add_face(otl, itr, otr)
    # Bottom edge
    geom.add_face(obl, obr, ibr)
    geom.add_face(obl, ibr, ibl)
    # Left edge
    geom.add_face(otl, obl, ibl)
    geom.add_face(otl, ibl, itl)
    # Right edge
    geom.add_face(otr, itr, ibr)
    geom.add_face(otr, ibr, obr)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="three_wheel_pet_stroller",
        meta={
            "category_context": "Pet_Animal related / Pet stroller",
            "reference_note": "No classification mismatch suspected: the reference shows a pet stroller.",
        },
    )

    navy = model.material("navy_fabric", rgba=(0.02, 0.06, 0.30, 1.0))
    black = model.material("matte_black", rgba=(0.005, 0.005, 0.006, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    silver = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    mesh_black = model.material("translucent_black_mesh", rgba=(0.0, 0.0, 0.0, 0.46))
    rim_white = model.material("white_rim", rgba=(0.86, 0.88, 0.86, 1.0))

    # Root part: the fixed chassis, fabric lower cabin, storage basket, brackets,
    # and hinge collars.  X is forward, Y is width, Z is up.
    base = model.part("base")

    # Blue fabric lower cabin/basket: open above, with front mesh window and side panels.
    base.visual(
        Box((0.62, 0.012, 0.32)),
        origin=Origin(xyz=(0.03, -0.266, 0.56)),
        material=navy,
        name="side_panel_0",
    )
    base.visual(
        Box((0.62, 0.012, 0.32)),
        origin=Origin(xyz=(0.03, 0.266, 0.56)),
        material=navy,
        name="side_panel_1",
    )
    base.visual(
        Box((0.012, 0.52, 0.39)),
        origin=Origin(xyz=(-0.285, 0.0, 0.585)),
        material=navy,
        name="rear_fabric_panel",
    )
    base.visual(
        Box((0.012, 0.55, 0.25)),
        origin=Origin(xyz=(0.34, 0.0, 0.50)),
        material=navy,
        name="front_fabric_panel",
    )
    base.visual(
        Box((0.52, 0.48, 0.030)),
        origin=Origin(xyz=(0.03, 0.0, 0.405)),
        material=navy,
        name="seat_floor",
    )
    base.visual(
        Box((0.008, 0.29, 0.17)),
        origin=Origin(xyz=(0.347, 0.0, 0.54)),
        material=mesh_black,
        name="front_mesh_window",
    )
    base.visual(
        Box((0.33, 0.006, 0.16)),
        origin=Origin(xyz=(0.11, -0.273, 0.53)),
        material=mesh_black,
        name="side_mesh_window_0",
    )
    base.visual(
        Box((0.33, 0.006, 0.16)),
        origin=Origin(xyz=(0.11, 0.273, 0.53)),
        material=mesh_black,
        name="side_mesh_window_1",
    )
    # White/gray stitched trim around front mesh and cabin lip.
    base.visual(
        Box((0.010, 0.32, 0.012)),
        origin=Origin(xyz=(0.352, 0.0, 0.63)),
        material=silver,
        name="front_window_top_trim",
    )
    base.visual(
        Box((0.010, 0.018, 0.19)),
        origin=Origin(xyz=(0.352, -0.145, 0.54)),
        material=silver,
        name="front_window_trim_0",
    )
    base.visual(
        Box((0.010, 0.018, 0.19)),
        origin=Origin(xyz=(0.352, 0.145, 0.54)),
        material=silver,
        name="front_window_trim_1",
    )

    # Lower mesh storage basket under the cabin.
    storage_side = PerforatedPanelGeometry(
        (0.50, 0.18),
        0.004,
        hole_diameter=0.030,
        pitch=(0.060, 0.050),
        frame=0.010,
        corner_radius=0.012,
    )
    for idx, y in enumerate((-0.235, 0.235)):
        base.visual(
            mesh_from_geometry(storage_side.copy(), f"storage_mesh_{idx}"),
            origin=Origin(xyz=(0.03, y, 0.285), rpy=(-math.pi / 2, 0.0, 0.0)),
            material=mesh_black,
            name=f"storage_mesh_{idx}",
        )
    storage_floor = PerforatedPanelGeometry(
        (0.50, 0.49),
        0.004,
        hole_diameter=0.028,
        pitch=(0.060, 0.060),
        frame=0.012,
        corner_radius=0.014,
    )
    base.visual(
        mesh_from_geometry(storage_floor, "storage_floor_mesh"),
        origin=Origin(xyz=(0.03, 0.0, 0.215)),
        material=mesh_black,
        name="storage_floor_mesh",
    )

    # Tubular chassis rails and folding-looking bracing members.
    for y in (-0.285, 0.285):
        suffix = "0" if y < 0 else "1"
        _add_tube(
            base,
            [(-0.38, y, 0.20), (-0.22, y, 0.47), (-0.05, y, 0.69), (0.25, y, 0.69)],
            0.014,
            f"upper_side_rail_{suffix}",
            black,
        )
        _add_tube(
            base,
            [(-0.38, y, 0.20), (-0.16, y, 0.34), (0.42, y * 0.72, 0.34)],
            0.014,
            f"lower_side_rail_{suffix}",
            black,
        )
        _add_tube(
            base,
            [(-0.38, y, 0.20), (-0.43, y * 0.94, 0.46), (-0.45, y * 0.90, 0.74)],
            0.015,
            f"rear_upright_{suffix}",
            black,
        )
        _add_tube(
            base,
            [(-0.30, y, 0.24), (0.04, y * 0.88, 0.38), (0.36, y * 0.72, 0.34)],
            0.011,
            f"folding_strut_{suffix}",
            silver,
        )
        base.visual(
            Cylinder(radius=0.035, length=0.028),
            origin=Origin(xyz=(-0.05, y * 0.99, 0.64), rpy=(math.pi / 2, 0.0, 0.0)),
            material=black,
            name=f"side_hinge_disc_{suffix}",
        )

    base.visual(
        Cylinder(radius=0.014, length=0.60),
        origin=Origin(xyz=(-0.38, 0.0, 0.20), rpy=(math.pi / 2, 0.0, 0.0)),
        material=black,
        name="rear_axle",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.48),
        origin=Origin(xyz=(0.05, 0.0, 0.36), rpy=(math.pi / 2, 0.0, 0.0)),
        material=black,
        name="basket_crossbar",
    )
    base.visual(
        Cylinder(radius=0.017, length=0.45),
        origin=Origin(xyz=(0.39, 0.0, 0.36), rpy=(math.pi / 2, 0.0, 0.0)),
        material=black,
        name="front_bumper_bar",
    )

    # Rear axle brackets and front fork for the single front wheel.
    for y in (-0.302, 0.302):
        suffix = "0" if y < 0 else "1"
        base.visual(
            Cylinder(radius=0.018, length=0.060),
            origin=Origin(xyz=(-0.38, y, 0.20), rpy=(math.pi / 2, 0.0, 0.0)),
            material=black,
            name=f"rear_axle_stub_{suffix}",
        )
    _add_tube(
        base,
        [(0.39, -0.07, 0.34), (0.45, -0.06, 0.23), (0.46, -0.055, 0.145)],
        0.012,
        "front_fork_leg_0",
        black,
    )
    _add_tube(
        base,
        [(0.39, 0.07, 0.34), (0.45, 0.06, 0.23), (0.46, 0.055, 0.145)],
        0.012,
        "front_fork_leg_1",
        black,
    )
    base.visual(
        Cylinder(radius=0.010, length=0.120),
        origin=Origin(xyz=(0.46, 0.0, 0.145), rpy=(math.pi / 2, 0.0, 0.0)),
        material=silver,
        name="front_axle",
    )

    # Visible collars for the canopy and folding handle hinges.
    base.visual(
        Cylinder(radius=0.022, length=0.60),
        origin=Origin(xyz=(-0.245, 0.0, 0.715), rpy=(math.pi / 2, 0.0, 0.0)),
        material=black,
        name="canopy_hinge_bar",
    )
    for y in (-0.255, 0.255):
        suffix = "0" if y < 0 else "1"
        base.visual(
            Cylinder(radius=0.026, length=0.030),
            origin=Origin(xyz=(-0.455, y, 0.745), rpy=(math.pi / 2, 0.0, 0.0)),
            material=black,
            name=f"handle_hinge_collar_{suffix}",
        )

    # Rear handlebar: a separate folding part with two side arms and a rounded grip.
    handle = model.part("handle")
    for y in (-0.245, 0.245):
        suffix = "0" if y < 0 else "1"
        _add_tube(
            handle,
            [(0.0, y, 0.0), (-0.10, y, 0.12), (-0.25, y * 0.92, 0.27)],
            0.014,
            f"handle_arm_{suffix}",
            black,
        )
    _add_tube(
        handle,
        [(-0.25, -0.225, 0.27), (-0.30, 0.0, 0.295), (-0.25, 0.225, 0.27)],
        0.024,
        "foam_grip",
        black,
    )
    handle.visual(
        Box((0.050, 0.050, 0.070)),
        origin=Origin(xyz=(-0.11, -0.270, 0.09)),
        material=black,
        name="fold_latch",
    )
    model.articulation(
        "base_to_handle",
        ArticulationType.REVOLUTE,
        parent=base,
        child=handle,
        origin=Origin(xyz=(-0.47, 0.0, 0.745)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=-0.65, upper=0.35),
    )

    # Folding canopy: fully enclosed mesh dome that zips around the entire cabin.
    # The dome wraps front, top, and sides down to the cabin rim for an
    # escape-proof enclosure; the whole dome lifts open on the rear hinge.
    canopy = model.part("canopy")
    canopy_path = _bezier((0.00, 0.00), (0.10, 0.35), (0.44, 0.38), (0.58, 0.03), 22)

    # Dome roof: continuous arched navy fabric top surface spanning the cabin width.
    canopy.visual(
        mesh_from_geometry(_curved_fabric_panel(canopy_path, 0.57, 0.008), "dome_roof"),
        material=navy,
        name="dome_roof",
    )

    # Dome side walls: translucent mesh panels dropping from the arch side bows
    # down to the cabin rim (side panel tops).  In local canopy frame, z=0 is
    # the hinge height (world z=0.720).  Side panels top is at world z=0.72.
    side_wall_bottom_z = -0.005
    for idx, y in enumerate((-0.285, 0.285)):
        wall_geom = _dome_side_wall(canopy_path, y, side_wall_bottom_z, thickness=0.004)
        canopy.visual(
            mesh_from_geometry(wall_geom, f"dome_side_wall_{idx}"),
            material=mesh_black,
            name=f"dome_side_wall_{idx}",
        )

    # Dome front wall: translucent mesh panel from the arch front edge down to
    # the cabin front panel rim (world z~0.625 = local z~-0.095).
    front_x, front_z = canopy_path[-1]
    front_wall_bottom_z = -0.09
    front_wall_geom = _dome_front_wall(
        front_x, front_z, front_wall_bottom_z, half_width=0.285, thickness=0.004,
    )
    canopy.visual(
        mesh_from_geometry(front_wall_geom, "dome_front_wall"),
        material=mesh_black,
        name="dome_front_wall",
    )

    # Structural bow ribs defining the dome silhouette.
    _add_tube(canopy, [(x, -0.285, z) for x, z in canopy_path], 0.010, "canopy_side_bow_0", silver)
    _add_tube(canopy, [(x, 0.285, z) for x, z in canopy_path], 0.010, "canopy_side_bow_1", silver)

    # Front cross bow connecting side bows at the upper front of the dome.
    seam_x, seam_z = canopy_path[12]
    canopy.visual(
        Cylinder(radius=0.008, length=0.58),
        origin=Origin(xyz=(seam_x, 0.0, seam_z), rpy=(math.pi / 2, 0.0, 0.0)),
        material=silver,
        name="canopy_front_bow",
    )

    # Front lip bar at the bottom of the dome front wall — acts as a rigid zip
    # track anchor and frame member for the enclosure.
    canopy.visual(
        Cylinder(radius=0.010, length=0.57),
        origin=Origin(xyz=(front_x, 0.0, front_wall_bottom_z), rpy=(math.pi / 2, 0.0, 0.0)),
        material=black,
        name="canopy_front_lip",
    )

    model.articulation(
        "base_to_canopy",
        ArticulationType.REVOLUTE,
        parent=base,
        child=canopy,
        origin=Origin(xyz=(-0.245, 0.0, 0.720)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.2, lower=0.0, upper=1.20),
    )

    # Wheel helper geometry: separate tire and spoked rim visuals inside each spinning wheel part.
    rear_rim = WheelGeometry(
        0.145,
        0.042,
        rim=WheelRim(inner_radius=0.105, flange_height=0.008, flange_thickness=0.004, bead_seat_depth=0.004),
        hub=WheelHub(
            radius=0.032,
            width=0.035,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=6, circle_diameter=0.045, hole_diameter=0.004),
        ),
        face=WheelFace(dish_depth=0.006, front_inset=0.004, rear_inset=0.003),
        spokes=WheelSpokes(style="radial", count=12, thickness=0.003, window_radius=0.012),
        bore=WheelBore(style="round", diameter=0.014),
    )
    rear_tire = TireGeometry(
        0.190,
        0.052,
        inner_radius=0.139,
        carcass=TireCarcass(belt_width_ratio=0.64, sidewall_bulge=0.06),
        tread=TireTread(style="block", depth=0.006, count=26, land_ratio=0.52),
        grooves=(TireGroove(center_offset=0.0, width=0.005, depth=0.003),),
        sidewall=TireSidewall(style="rounded", bulge=0.05),
        shoulder=TireShoulder(width=0.006, radius=0.003),
    )
    front_rim = WheelGeometry(
        0.096,
        0.036,
        rim=WheelRim(inner_radius=0.070, flange_height=0.006, flange_thickness=0.003, bead_seat_depth=0.003),
        hub=WheelHub(radius=0.026, width=0.030, cap_style="domed"),
        face=WheelFace(dish_depth=0.004, front_inset=0.003, rear_inset=0.003),
        spokes=WheelSpokes(style="radial", count=10, thickness=0.0025, window_radius=0.009),
        bore=WheelBore(style="round", diameter=0.012),
    )
    front_tire = TireGeometry(
        0.130,
        0.044,
        inner_radius=0.092,
        carcass=TireCarcass(belt_width_ratio=0.64, sidewall_bulge=0.06),
        tread=TireTread(style="block", depth=0.004, count=22, land_ratio=0.55),
        sidewall=TireSidewall(style="rounded", bulge=0.04),
        shoulder=TireShoulder(width=0.005, radius=0.002),
    )

    rear_spin_source = None
    for idx, y in enumerate((-0.342, 0.342)):
        wheel = model.part(f"rear_wheel_{idx}")
        wheel.visual(
            mesh_from_geometry(rear_tire.copy(), f"rear_tire_{idx}"),
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2)),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            mesh_from_geometry(rear_rim.copy(), f"rear_rim_{idx}"),
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2)),
            material=rim_white,
            name="spoked_rim",
        )
        joint = model.articulation(
            f"base_to_rear_wheel_{idx}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=wheel,
            origin=Origin(xyz=(-0.38, y, 0.20)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=20.0),
            mimic=Mimic(rear_spin_source) if rear_spin_source else None,
        )
        if rear_spin_source is None:
            rear_spin_source = joint.name

    front_wheel = model.part("front_wheel")
    front_wheel.visual(
        mesh_from_geometry(front_tire, "front_tire"),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2)),
        material=rubber,
        name="tire",
    )
    front_wheel.visual(
        mesh_from_geometry(front_rim, "front_rim"),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2)),
        material=rim_white,
        name="spoked_rim",
    )
    model.articulation(
        "base_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=front_wheel,
        origin=Origin(xyz=(0.46, 0.0, 0.145)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=18.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    canopy = object_model.get_part("canopy")
    handle = object_model.get_part("handle")
    front_wheel = object_model.get_part("front_wheel")
    rear_0 = object_model.get_part("rear_wheel_0")
    rear_1 = object_model.get_part("rear_wheel_1")
    # Local hidden embeddings: axles/collars pass into hubs and fabric hinge sleeves.
    # These are intentional small mechanical captures, not broad body intersections.
    ctx.allow_overlap(
        base,
        canopy,
        elem_a="canopy_hinge_bar",
        elem_b="dome_roof",
        reason="The dome roof fabric wraps around the fixed hinge bar.",
    )
    # The dome roof fabric extends slightly below the hinge line where it wraps
    # around the hinge barrel, creating a small local overlap with the side
    # panel tops at the cabin rim.  This is intentional fabric seating.
    for idx in (0, 1):
        ctx.allow_overlap(
            base,
            canopy,
            elem_a=f"side_panel_{idx}",
            elem_b="dome_roof",
            reason="The dome roof fabric seats into the cabin side panel rim at the hinge line.",
        )
        ctx.expect_contact(
            base,
            canopy,
            elem_a=f"side_panel_{idx}",
            elem_b="dome_roof",
            contact_tol=0.010,
            name=f"dome roof contacts side panel {idx} at cabin rim",
        )
    for idx in (0, 1):
        ctx.allow_overlap(
            base,
            canopy,
            elem_a="canopy_hinge_bar",
            elem_b=f"canopy_side_bow_{idx}",
            reason="The canopy bow terminates in the same hinge barrel as the fabric sleeve.",
        )
        ctx.expect_overlap(
            canopy,
            base,
            axes="xz",
            elem_a=f"canopy_side_bow_{idx}",
            elem_b="canopy_hinge_bar",
            min_overlap=0.004,
            name=f"canopy bow {idx} is captured at hinge",
        )
        # The dome front lip seats into the cabin rim at the front corners where
        # it meets the side panels.  This is intentional local seating, not a
        # broad body intersection.
        ctx.allow_overlap(
            base,
            canopy,
            elem_a=f"side_panel_{idx}",
            elem_b="canopy_front_lip",
            reason="The dome front lip seats into the cabin side panel rim at the front corners.",
        )
    # The dome front lip seats onto the front fabric panel rim.  This is
    # intentional seating contact, not a broad body intersection.
    ctx.allow_overlap(
        base,
        canopy,
        elem_a="front_fabric_panel",
        elem_b="canopy_front_lip",
        reason="The dome front lip seats onto the cabin front fabric panel rim.",
    )
    ctx.expect_contact(
        base,
        canopy,
        elem_a="front_fabric_panel",
        elem_b="canopy_front_lip",
        contact_tol=0.010,
        name="dome front lip contacts front fabric panel at rim",
    )
    ctx.allow_overlap(
        base,
        front_wheel,
        elem_a="front_axle",
        elem_b="spoked_rim",
        reason="The front axle is intentionally captured inside the wheel hub bore.",
    )
    for idx in (0, 1):
        ctx.allow_overlap(
            base,
            f"rear_wheel_{idx}",
            elem_a=f"rear_axle_stub_{idx}",
            elem_b="spoked_rim",
            reason="The rear axle stub is intentionally captured inside the spoked wheel hub.",
        )
        ctx.expect_overlap(
            f"rear_wheel_{idx}",
            base,
            axes="yz",
            elem_a="spoked_rim",
            elem_b=f"rear_axle_stub_{idx}",
            min_overlap=0.010,
            name=f"rear wheel {idx} is retained on axle stub",
        )
    for idx in (0, 1):
        ctx.allow_overlap(
            base,
            handle,
            elem_a=f"handle_hinge_collar_{idx}",
            elem_b=f"handle_arm_{idx}",
            reason="The folding handle arm nests locally in the hinge collar.",
        )

    ctx.check(
        "pet stroller has three rolling wheels",
        all(object_model.get_part(name) is not None for name in ("front_wheel", "rear_wheel_0", "rear_wheel_1")),
    )
    ctx.expect_overlap(
        front_wheel,
        base,
        axes="yz",
        elem_a="spoked_rim",
        elem_b="front_axle",
        min_overlap=0.010,
        name="front wheel is retained on fork axle",
    )
    ctx.expect_overlap(
        canopy,
        base,
        axes="y",
        elem_a="dome_roof",
        elem_b="canopy_hinge_bar",
        min_overlap=0.45,
        name="dome roof spans hinge bar width",
    )
    ctx.expect_origin_gap(front_wheel, rear_0, axis="x", min_gap=0.70, name="front wheel leads rear axle")
    ctx.expect_origin_gap(rear_1, rear_0, axis="y", min_gap=0.60, name="rear wheels span stroller width")
    ctx.expect_overlap(canopy, base, axes="xy", min_overlap=0.18, name="dome covers cabin footprint")

    # Verify the dome enclosure fully wraps the cabin: side walls and front wall
    # extend down to the cabin rim for a secure/escape-proof enclosure.
    # Side walls should reach the side panel tops (cabin rim at z~0.72).
    for idx in (0, 1):
        ctx.expect_gap(
            canopy,
            base,
            axis="z",
            positive_elem=f"dome_side_wall_{idx}",
            negative_elem=f"side_panel_{idx}",
            min_gap=-0.01,
            max_gap=0.02,
            max_penetration=0.01,
            name=f"dome side wall {idx} reaches cabin side panel rim",
        )
    # Front wall should reach the front panel top (z~0.625).
    ctx.expect_gap(
        canopy,
        base,
        axis="z",
        positive_elem="dome_front_wall",
        negative_elem="front_fabric_panel",
        min_gap=-0.01,
        max_gap=0.02,
        max_penetration=0.01,
        name="dome front wall reaches cabin front panel rim",
    )
    # Front lip seats at the bottom of the front wall, near the front panel rim.
    ctx.expect_gap(
        canopy,
        base,
        axis="z",
        positive_elem="canopy_front_lip",
        negative_elem="front_fabric_panel",
        min_gap=-0.01,
        max_gap=0.02,
        max_penetration=0.01,
        name="dome front lip seats at cabin front panel rim",
    )

    canopy_joint = object_model.get_articulation("base_to_canopy")
    handle_joint = object_model.get_articulation("base_to_handle")
    rest_canopy_aabb = ctx.part_world_aabb(canopy)
    rest_handle_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({canopy_joint: 0.90, handle_joint: -0.45}):
        folded_canopy_aabb = ctx.part_world_aabb(canopy)
        folded_handle_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "canopy revolute joint folds the arched cover upward",
        rest_canopy_aabb is not None
        and folded_canopy_aabb is not None
        and folded_canopy_aabb[1][2] > rest_canopy_aabb[1][2] + 0.03,
        details=f"rest={rest_canopy_aabb}, folded={folded_canopy_aabb}",
    )
    ctx.check(
        "handle hinge changes rear grip pose",
        rest_handle_aabb is not None
        and folded_handle_aabb is not None
        and abs(folded_handle_aabb[0][0] - rest_handle_aabb[0][0]) > 0.05,
        details=f"rest={rest_handle_aabb}, folded={folded_handle_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
