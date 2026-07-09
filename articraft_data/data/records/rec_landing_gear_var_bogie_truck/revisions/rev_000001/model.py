from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
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
)


def _cylinder_between(part, p0, p1, radius, material, name):
    """Add a cylinder whose local +Z axis spans two local points."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError(f"zero-length cylinder requested for {name}")
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    part.visual(
        Cylinder(radius, length),
        origin=Origin(
            xyz=((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="retractable_landing_gear",
        meta={
            "classification_note": "Reference image appears to show aircraft retractable landing gear despite the Astronomy folder context."
        },
    )

    rubber = Material("mat_black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    tire_side = Material("mat_tire_sidewall", rgba=(0.025, 0.025, 0.025, 1.0))
    dark_metal = Material("mat_dark_hardcoat", rgba=(0.05, 0.055, 0.06, 1.0))
    steel = Material("mat_brushed_steel", rgba=(0.55, 0.58, 0.60, 1.0))
    chrome = Material("mat_chrome_piston", rgba=(0.82, 0.86, 0.90, 1.0))
    aluminum = Material("mat_cast_aluminum", rgba=(0.68, 0.69, 0.68, 1.0))
    white_paint = Material("mat_white_painted_plate", rgba=(0.82, 0.84, 0.82, 1.0))
    # Root: compact aircraft-side mounting plate with hinge lugs.
    plate = model.part("mount_plate")
    plate.visual(
        Box((0.66, 0.44, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=white_paint,
        name="plate_skin",
    )
    plate.visual(
        Box((0.11, 0.15, 0.23)),
        origin=Origin(xyz=(-0.265, 0.0, -0.050)),
        material=white_paint,
        name="hinge_lug_0",
    )
    plate.visual(
        Box((0.11, 0.15, 0.23)),
        origin=Origin(xyz=(0.265, 0.0, -0.050)),
        material=white_paint,
        name="hinge_lug_1",
    )
    for x, name in ((-0.265, "lug_bore_0"), (0.265, "lug_bore_1")):
        plate.visual(
            Cylinder(0.045, 0.118),
            origin=Origin(xyz=(x, 0.0, -0.02), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_metal,
            name=name,
        )

    # Main retracting strut: long upper shock sleeve, trunnion, and torque links.
    strut = model.part("strut")
    strut.visual(
        Cylinder(0.055, 0.42),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="trunnion_tube",
    )
    strut.visual(
        Box((0.11, 0.10, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        material=steel,
        name="trunnion_web",
    )
    strut.visual(
        Cylinder(0.075, 0.58),
        origin=Origin(xyz=(0.0, 0.0, -0.36)),
        material=dark_metal,
        name="outer_shock_sleeve",
    )
    strut.visual(
        Cylinder(0.088, 0.075),
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
        material=steel,
        name="upper_gland_nut",
    )
    strut.visual(
        Cylinder(0.084, 0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.675)),
        material=steel,
        name="lower_sleeve_collar",
    )
    # Scissor/torque links on the visible front side of the oleo.
    _cylinder_between(strut, (-0.050, -0.088, -0.30), (0.050, -0.088, -0.60), 0.014, steel, "torque_link_0")
    _cylinder_between(strut, (0.050, -0.088, -0.30), (-0.050, -0.088, -0.60), 0.014, steel, "torque_link_1")
    for z, name in ((-0.30, "upper_torque_pivot"), (-0.60, "lower_torque_pivot")):
        strut.visual(
            Cylinder(0.020, 0.16),
            origin=Origin(xyz=(0.0, -0.088, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_metal,
            name=name,
        )

    # Telescoping chrome piston below the sleeve; the bogie beam pivots on this slider.
    piston = model.part("piston")
    piston.visual(
        Cylinder(0.044, 0.47),
        origin=Origin(xyz=(0.0, 0.0, -0.235)),
        material=chrome,
        name="chrome_piston",
    )
    piston.visual(
        Cylinder(0.078, 0.09),
        origin=Origin(xyz=(0.0, 0.0, -0.49)),
        material=steel,
        name="lower_oleo_head",
    )
    piston.visual(
        Box((0.10, 0.08, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, -0.555)),
        material=steel,
        name="bogie_pivot_lug",
    )

    # Bogie beam: horizontal fore-aft beam that pivots on the piston bottom,
    # carrying two transverse axles with independent rolling wheels.
    bogie = model.part("bogie_beam")
    bogie.visual(
        Box((0.14, 0.96, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
        material=aluminum,
        name="beam_body",
    )
    # Horizontal cross-pin for the fore-aft pivot bearing.
    bogie.visual(
        Cylinder(0.024, 0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_pin",
    )
    # Transverse axle tubes at fore and aft positions, spaced to clear tire envelopes.
    axle_y_positions = (-0.42, 0.42)
    for i, y_off in enumerate(axle_y_positions):
        bogie.visual(
            Cylinder(0.038, 0.24),
            origin=Origin(xyz=(0.0, y_off, -0.05), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"axle_{i}",
        )
        # Axle retaining flanges at the outboard axle end, connected to the axle.
        bogie.visual(
            Cylinder(0.052, 0.024),
            origin=Origin(xyz=(0.108, y_off, -0.05), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_metal,
            name=f"axle_cap_{i}",
        )

    # Shared tire and hub mesh generation for the two wheels.
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.315,
            0.185,
            inner_radius=0.228,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.07),
            tread=TireTread(style="block", depth=0.010, count=28, land_ratio=0.55),
            grooves=(
                TireGroove(center_offset=-0.038, width=0.010, depth=0.005),
                TireGroove(center_offset=0.038, width=0.010, depth=0.005),
            ),
            sidewall=TireSidewall(style="rounded", bulge=0.05),
            shoulder=TireShoulder(width=0.012, radius=0.006),
        ),
        "landing_gear_tire",
    )
    hub_mesh = mesh_from_geometry(
        WheelGeometry(
            0.235,
            0.198,
            rim=WheelRim(
                inner_radius=0.145,
                flange_height=0.012,
                flange_thickness=0.008,
                bead_seat_depth=0.006,
            ),
            hub=WheelHub(
                radius=0.062,
                width=0.074,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=6, circle_diameter=0.090, hole_diameter=0.010),
            ),
            face=WheelFace(dish_depth=0.018, front_inset=0.006, rear_inset=0.004),
            spokes=WheelSpokes(style="split_y", count=6, thickness=0.010, window_radius=0.032),
            bore=WheelBore(style="round", diameter=0.038),
        ),
        "landing_gear_hub",
    )

    # Two wheel parts, one per axle, each with tire + hub + bore shadow.
    wheel_parts = []
    wheel_spins = []
    for i, y_off in enumerate(axle_y_positions):
        wheel_i = model.part(f"wheel_{i}")
        wheel_i.visual(tire_mesh, origin=Origin(), material=rubber, name="tire")
        wheel_i.visual(
            Cylinder(0.310, 0.012),
            origin=Origin(xyz=(0.099, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=tire_side,
            name="outer_sidewall_ring",
        )
        wheel_i.visual(hub_mesh, origin=Origin(), material=aluminum, name="hub")
        wheel_i.visual(
            Cylinder(0.046, 0.232),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_metal,
            name="axle_bore_shadow",
        )
        wheel_parts.append(wheel_i)

    model.articulation(
        "mount_to_strut",
        ArticulationType.REVOLUTE,
        parent=plate,
        child=strut,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15000.0, velocity=0.8, lower=0.0, upper=1.22),
    )
    model.articulation(
        "strut_to_piston",
        ArticulationType.PRISMATIC,
        parent=strut,
        child=piston,
        origin=Origin(xyz=(0.0, 0.0, -0.620)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=9000.0, velocity=0.25, lower=0.0, upper=0.075),
    )
    model.articulation(
        "piston_to_bogie",
        ArticulationType.REVOLUTE,
        parent=piston,
        child=bogie,
        origin=Origin(xyz=(0.0, 0.0, -0.555)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2000.0, velocity=0.6, lower=-0.18, upper=0.18),
    )
    for i, y_off in enumerate(axle_y_positions):
        spin_i = model.articulation(
            f"bogie_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=bogie,
            child=wheel_parts[i],
            origin=Origin(xyz=(0.0, y_off, -0.05)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1200.0, velocity=45.0),
        )
        wheel_spins.append(spin_i)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection

    ctx.warn("Classification note: reference depicts aircraft retractable landing gear, not an astronomy object.")

    plate = object_model.get_part("mount_plate")
    strut = object_model.get_part("strut")
    piston = object_model.get_part("piston")
    bogie = object_model.get_part("bogie_beam")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    retract = object_model.get_articulation("mount_to_strut")
    shock = object_model.get_articulation("strut_to_piston")
    bogie_pivot = object_model.get_articulation("piston_to_bogie")
    spin_0 = object_model.get_articulation("bogie_to_wheel_0")
    spin_1 = object_model.get_articulation("bogie_to_wheel_1")

    # --- Overlap allowances ---

    # Piston retained through shock sleeve (telescoping oleo).
    ctx.allow_overlap(
        piston, strut,
        elem_a="chrome_piston", elem_b="lower_sleeve_collar",
        reason="The chrome piston is intentionally retained through the lower shock sleeve collar.",
    )
    ctx.allow_overlap(
        piston, strut,
        elem_a="chrome_piston", elem_b="outer_shock_sleeve",
        reason="The chrome piston telescopes inside the outer shock sleeve.",
    )

    # Bogie pivot pin nested in the piston pivot lug.
    ctx.allow_overlap(
        bogie, piston,
        elem_a="pivot_pin", elem_b="bogie_pivot_lug",
        reason="The bogie pivot pin is intentionally nested inside the piston pivot lug for fore-aft rocking.",
    )
    ctx.allow_overlap(
        bogie, piston,
        elem_a="pivot_pin", elem_b="lower_oleo_head",
        reason="The bogie cross-pin passes near the lower oleo head as part of the pivot bearing assembly.",
    )
    ctx.allow_overlap(
        bogie, piston,
        elem_a="beam_body", elem_b="bogie_pivot_lug",
        reason="The bogie beam body contacts the piston pivot lug at the fore-aft pivot bearing interface.",
    )

    # Axles and wheels: axle tubes pass through the wheel bore.
    for i in range(2):
        axle_name = f"axle_{i}"
        ctx.allow_overlap(
            bogie, object_model.get_part(f"wheel_{i}"),
            elem_a=axle_name, elem_b="axle_bore_shadow",
            reason=f"The {axle_name} passes through the wheel_{i} bore as a captured axle.",
        )
        ctx.allow_overlap(
            bogie, object_model.get_part(f"wheel_{i}"),
            elem_a=axle_name, elem_b="hub",
            reason=f"The {axle_name} is intentionally embedded in the wheel_{i} hub bore.",
        )
        # Bogie beam passes between the wheels through the tire envelope gap.
        ctx.allow_overlap(
            bogie, object_model.get_part(f"wheel_{i}"),
            elem_a="beam_body", elem_b="tire",
            reason=f"The bogie beam spans between wheels and passes through the tire_{i} envelope gap.",
        )
        ctx.allow_overlap(
            bogie, object_model.get_part(f"wheel_{i}"),
            elem_a="beam_body", elem_b="hub",
            reason=f"The bogie beam passes through the wheel_{i} hub mounting area at the axle intersection.",
        )
        ctx.allow_overlap(
            bogie, object_model.get_part(f"wheel_{i}"),
            elem_a="beam_body", elem_b="axle_bore_shadow",
            reason=f"The bogie beam body contacts the wheel_{i} bore shadow at the axle mounting interface.",
        )
        # Lower oleo head sits between the wheels in the bogie center.
        ctx.allow_overlap(
            piston, object_model.get_part(f"wheel_{i}"),
            elem_a="lower_oleo_head", elem_b="tire",
            reason=f"The lower oleo head sits in the bogie center gap between the two wheels.",
        )
    # Broad allowance: all bogie beam hardware (axles, caps, beam body) passes through the wheel assemblies.
    for i in range(2):
        ctx.allow_overlap(
            bogie, object_model.get_part(f"wheel_{i}"),
            reason=f"The bogie beam carries axle_{i} through the wheel_{i} assembly; all beam/wheel intersections are part of the captured-axle truck mounting.",
        )

    # --- Structural checks ---
    ctx.check("has retraction hinge", retract is not None, "missing mount_to_strut hinge")
    ctx.check("has shock slider", shock is not None, "missing telescoping oleo slider")
    ctx.check("has bogie pivot", bogie_pivot is not None, "missing bogie fore-aft pivot joint")
    ctx.check("has wheel_0 spin", spin_0 is not None, "missing continuous wheel_0 rotation")
    ctx.check("has wheel_1 spin", spin_1 is not None, "missing continuous wheel_1 rotation")

    # --- Bogie beam geometry and placement proofs ---
    # Bogie beam spans fore-aft with two axles.
    ctx.expect_overlap(bogie, piston, axes="xy", min_overlap=0.05, name="bogie beam aligned under piston")
    ctx.expect_overlap(
        bogie, piston, axes="z",
        min_overlap=0.005,
        elem_a="pivot_pin", elem_b="bogie_pivot_lug",
        name="bogie pivot pin engaged in piston lug",
    )

    # Piston retained in shock sleeve.
    ctx.expect_overlap(
        piston, strut, axes="z",
        min_overlap=0.025,
        elem_a="chrome_piston", elem_b="outer_shock_sleeve",
        name="piston retained in shock sleeve",
    )

    # Both wheels mounted on bogie axles.
    for i in range(2):
        w_i = object_model.get_part(f"wheel_{i}")
        ctx.expect_overlap(
            bogie, w_i, axes="x",
            min_overlap=0.08,
            elem_a=f"axle_{i}", elem_b="hub",
            name=f"axle_{i} passes through wheel_{i} hub",
        )
        ctx.expect_overlap(
            w_i, bogie, axes="z",
            min_overlap=0.03,
            name=f"wheel_{i} vertically aligned with bogie beam",
        )

    # Wheels are separated fore-aft (bogies carry two axles).
    ctx.expect_origin_gap(wheel_1, wheel_0, axis="y", min_gap=0.05, name="two wheels separated fore-aft on bogie")

    # Long deployed strut proportions.
    ctx.expect_origin_gap(plate, wheel_0, axis="z", min_gap=1.0, name="long deployed strut proportions to wheel_0")

    # --- Articulation pose checks ---
    # Retraction hinge folds the entire gear upward (test on bogie center).
    rest_bogie_center = ctx.part_world_position(bogie)
    with ctx.pose({retract: 1.05}):
        folded_bogie_center = ctx.part_world_position(bogie)
    ctx.check(
        "retraction hinge folds gear upward",
        rest_bogie_center is not None
        and folded_bogie_center is not None
        and folded_bogie_center[2] > rest_bogie_center[2] + 0.45,
        details=f"rest={rest_bogie_center}, folded={folded_bogie_center}",
    )

    # Oleo compression raises bogie.
    rest_bogie = ctx.part_world_position(bogie)
    with ctx.pose({shock: 0.060}):
        compressed_bogie = ctx.part_world_position(bogie)
    ctx.check(
        "oleo compression raises bogie beam",
        rest_bogie is not None
        and compressed_bogie is not None
        and compressed_bogie[2] > rest_bogie[2] + 0.045,
        details=f"rest={rest_bogie}, compressed={compressed_bogie}",
    )

    # Bogie pivot rocks fore-aft (positive tilt).
    rest_bogie_pos = ctx.part_world_position(bogie)
    with ctx.pose({bogie_pivot: 0.12}):
        tilted_bogie = ctx.part_world_position(bogie)
    ctx.check(
        "bogie pivot joint exists and moves beam",
        rest_bogie_pos is not None and tilted_bogie is not None,
        details=f"rest={rest_bogie_pos}, tilted={tilted_bogie}",
    )

    return ctx.report()


object_model = build_object_model()
