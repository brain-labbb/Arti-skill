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
    TestContext,
    TestReport,
    mesh_from_geometry,
)


# ---------------------------------------------------------------------------
# Layout constants
# ---------------------------------------------------------------------------
BEND_Z = 1.05
RAM_ORIGIN_X = -0.22
RAM_UPPER = 0.12
ROLLER_X = 0.125
ROLLER_Y = 0.120
FORMER_LOCAL_X = 0.12
CONDUIT_X = 0.088
PUMP_UPPER = math.radians(40.0)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def _cylinder_between(part, name, p0, p1, radius, material, *, segments=24):
    x0, y0, z0 = p0
    x1, y1, z1 = p1
    dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0:
        raise ValueError(f"zero-length cylinder {name}")
    ux, uy, uz = dx / length, dy / length, dz / length
    pitch = math.acos(max(-1.0, min(1.0, uz)))
    yaw = math.atan2(uy, ux)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((x0 + x1) * 0.5, (y0 + y1) * 0.5, (z0 + z1) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _annular_sector(
    *,
    inner_radius: float,
    outer_radius: float,
    thickness_y: float,
    theta0: float,
    theta1: float,
    segments: int = 48,
    y_offset: float = 0.0,
) -> MeshGeometry:
    geom = MeshGeometry()
    front = y_offset + thickness_y * 0.5
    back = y_offset - thickness_y * 0.5
    angles = [theta0 + (theta1 - theta0) * i / segments for i in range(segments + 1)]
    for theta in angles:
        c = math.cos(theta)
        s = math.sin(theta)
        geom.add_vertex(outer_radius * c, front, outer_radius * s)
        geom.add_vertex(inner_radius * c, front, inner_radius * s)
        geom.add_vertex(outer_radius * c, back, outer_radius * s)
        geom.add_vertex(inner_radius * c, back, inner_radius * s)
    for i in range(segments):
        a = 4 * i
        b = 4 * (i + 1)
        geom.add_face(a, b, b + 1)
        geom.add_face(a, b + 1, a + 1)
        geom.add_face(a + 2, b + 3, b + 2)
        geom.add_face(a + 2, a + 3, b + 3)
        geom.add_face(a, a + 2, b + 2)
        geom.add_face(a, b + 2, b)
        geom.add_face(a + 1, b + 1, b + 3)
        geom.add_face(a + 1, b + 3, a + 3)
    a = 0
    geom.add_face(a, a + 1, a + 3)
    geom.add_face(a, a + 3, a + 2)
    a = 4 * segments
    geom.add_face(a, a + 3, a + 1)
    geom.add_face(a, a + 2, a + 3)
    return geom


def _add_radial_box(part, name, radius, theta, size, material, *, y=0.0):
    part.visual(
        Box(size),
        origin=Origin(
            xyz=(radius * math.cos(theta), y, radius * math.sin(theta)),
            rpy=(0.0, -theta, 0.0),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="hydraulic_conduit_bender",
        meta={"small_class": "Conduit bender", "domain": "Electrical_Wiring"},
    )

    yellow = Material("painted_yellow_steel", rgba=(1.0, 0.72, 0.02, 1.0))
    black = Material("black_cast_iron", rgba=(0.015, 0.015, 0.018, 1.0))
    dark = Material("dark_hardware", rgba=(0.08, 0.085, 0.09, 1.0))
    rubber = Material("black_rubber_grip", rgba=(0.005, 0.006, 0.007, 1.0))
    galvanized = Material("galvanized_conduit", rgba=(0.70, 0.74, 0.75, 1.0))
    brass = Material("brass_terminal_screw", rgba=(0.86, 0.64, 0.25, 1.0))
    white = Material("white_marking_paint", rgba=(0.96, 0.95, 0.88, 1.0))
    label = Material("warning_label", rgba=(1.0, 0.92, 0.10, 1.0))
    red = Material("hydraulic_fitting", rgba=(0.55, 0.12, 0.08, 1.0))

    # ==================================================================
    # FRAME (root – tubular stand + platform + roller mounts + cylinder)
    # ==================================================================
    frame = model.part("frame")

    # Floor rails.
    _cylinder_between(frame, "base_rail_0", (-0.55, -0.44, 0.035), (-0.55, 0.44, 0.035), 0.020, yellow)
    _cylinder_between(frame, "base_rail_1", (0.38, -0.44, 0.035), (0.38, 0.44, 0.035), 0.020, yellow)

    # Splayed legs.
    _cylinder_between(frame, "front_leg_0", (-0.55, -0.36, 0.055), (-0.28, -0.10, 0.975), 0.018, yellow)
    _cylinder_between(frame, "front_leg_1", (-0.55, 0.36, 0.055), (-0.28, 0.10, 0.975), 0.018, yellow)
    _cylinder_between(frame, "rear_leg_0", (0.38, -0.36, 0.055), (0.08, -0.10, 0.975), 0.018, yellow)
    _cylinder_between(frame, "rear_leg_1", (0.38, 0.36, 0.055), (0.08, 0.10, 0.975), 0.018, yellow)

    # Cross braces – endpoints match exact leg positions at z=0.22.
    # front_leg_0 at z=0.22 ≈ (-0.502, -0.313, 0.22)
    # rear_leg_1 at z=0.22 ≈ (0.326, 0.313, 0.22)
    _cylinder_between(frame, "cross_brace_0", (-0.502, -0.313, 0.22), (0.326, 0.313, 0.22), 0.010, dark)
    _cylinder_between(frame, "cross_brace_1", (-0.502, 0.313, 0.22), (0.326, -0.313, 0.22), 0.010, dark)

    # Platform beam at working height.
    frame.visual(
        Box((0.56, 0.26, 0.032)),
        origin=Origin(xyz=(-0.08, 0.0, BEND_Z - 0.058)),
        material=yellow,
        name="platform_beam",
    )

    # Guide rails for ram carriage.
    for i, y_s in enumerate((-1.0, 1.0)):
        frame.visual(
            Box((0.42, 0.018, 0.018)),
            origin=Origin(xyz=(-0.04, y_s * 0.058, BEND_Z - 0.033)),
            material=dark,
            name=f"guide_rail_{i}",
        )

    # Hydraulic cylinder body (fixed to frame).
    _cylinder_between(frame, "ram_cylinder_body", (-0.42, 0.0, BEND_Z), (-0.22, 0.0, BEND_Z), 0.038, dark)
    _cylinder_between(frame, "ram_cylinder_rear_cap", (-0.44, 0.0, BEND_Z), (-0.42, 0.0, BEND_Z), 0.044, black)
    _cylinder_between(frame, "ram_cylinder_front_gland", (-0.23, 0.0, BEND_Z), (-0.22, 0.0, BEND_Z), 0.044, black)

    # Hydraulic fittings and gauge.
    frame.visual(
        Cylinder(radius=0.009, length=0.028),
        origin=Origin(xyz=(-0.36, 0.042, BEND_Z + 0.025), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=red, name="hydraulic_fitting_0",
    )
    frame.visual(
        Cylinder(radius=0.009, length=0.028),
        origin=Origin(xyz=(-0.30, 0.042, BEND_Z + 0.025), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=red, name="hydraulic_fitting_1",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(-0.33, 0.040, BEND_Z - 0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass, name="pressure_gauge_boss",
    )
    frame.visual(
        Cylinder(radius=0.015, length=0.004),
        origin=Origin(xyz=(-0.33, 0.046, BEND_Z - 0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=white, name="pressure_gauge_face",
    )

    # Cylinder mount bracket.
    frame.visual(
        Box((0.06, 0.10, 0.055)),
        origin=Origin(xyz=(-0.40, 0.0, BEND_Z - 0.048)),
        material=yellow, name="cylinder_mount_bracket",
    )

    # Roller mount uprights and gussets.
    for i, y_s in enumerate((-1.0, 1.0)):
        yp = y_s * ROLLER_Y
        frame.visual(
            Box((0.045, 0.030, 0.110)),
            origin=Origin(xyz=(ROLLER_X, yp, BEND_Z - 0.005)),
            material=yellow, name=f"roller_upright_{i}",
        )
        frame.visual(
            Box((0.060, 0.025, 0.030)),
            origin=Origin(xyz=(ROLLER_X - 0.035, yp, BEND_Z - 0.055)),
            material=yellow, name=f"roller_gusset_{i}",
        )

    # Fixed reaction rollers (grooved wheels with pins).
    for i, y_s in enumerate((-1.0, 1.0)):
        yc = y_s * ROLLER_Y
        _cylinder_between(
            frame, f"roller_body_{i}",
            (ROLLER_X, yc - 0.018, BEND_Z), (ROLLER_X, yc + 0.018, BEND_Z),
            0.026, dark,
        )
        # Flanges: reduced radius (0.025) so they don't overlap conduit.
        _cylinder_between(
            frame, f"roller_flange_{i}_0",
            (ROLLER_X, yc - 0.024, BEND_Z), (ROLLER_X, yc - 0.018, BEND_Z),
            0.025, dark,
        )
        _cylinder_between(
            frame, f"roller_flange_{i}_1",
            (ROLLER_X, yc + 0.018, BEND_Z), (ROLLER_X, yc + 0.024, BEND_Z),
            0.025, dark,
        )
        _cylinder_between(
            frame, f"roller_pin_{i}",
            (ROLLER_X, yc - 0.032, BEND_Z), (ROLLER_X, yc + 0.032, BEND_Z),
            0.008, brass,
        )

    # Degree-marking plate – overlaps the roller_upright_1 for connectivity.
    frame.visual(
        Box((0.040, 0.005, 0.110)),
        origin=Origin(xyz=(ROLLER_X + 0.025, ROLLER_Y + 0.015, BEND_Z + 0.010)),
        material=white, name="degree_plate",
    )
    # Degree ticks on the plate surface.
    for idx, deg in enumerate((0, 15, 30, 45, 60, 75, 90)):
        z_off = (deg / 90.0) * 0.080 - 0.040
        tick_len = 0.014 if deg % 30 == 0 else 0.009
        frame.visual(
            Box((0.009, 0.006, tick_len)),
            origin=Origin(xyz=(ROLLER_X + 0.038, ROLLER_Y + 0.015, BEND_Z + 0.010 + z_off)),
            material=black, name=f"degree_tick_{idx}",
        )
    # Degree number pads on the plate surface.
    for idx, deg in enumerate((0, 45, 90)):
        z_off = (deg / 90.0) * 0.080 - 0.040
        frame.visual(
            Box((0.012, 0.006, 0.016)),
            origin=Origin(xyz=(ROLLER_X + 0.042, ROLLER_Y + 0.015, BEND_Z + 0.010 + z_off)),
            material=black, name=f"degree_pad_{idx}",
        )

    # Warning label plate – embedded slightly into beam surface for connectivity.
    frame.visual(
        Box((0.060, 0.005, 0.100)),
        origin=Origin(xyz=(-0.08, 0.130, BEND_Z - 0.058)),
        material=label, name="warning_label",
    )
    # Label stripes on the label surface.
    for i in range(3):
        frame.visual(
            Box((0.045, 0.006, 0.006)),
            origin=Origin(xyz=(-0.08, 0.134, BEND_Z - 0.080 + i * 0.025)),
            material=black, name=f"label_stripe_{i}",
        )

    # Terminal screws on uprights.
    for i, y_s in enumerate((-1.0, 1.0)):
        frame.visual(
            Cylinder(radius=0.009, length=0.025),
            origin=Origin(
                xyz=(ROLLER_X + 0.024, y_s * ROLLER_Y, BEND_Z + 0.040),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=brass, name=f"terminal_screw_{i}",
        )

    # Pump pivot boss.
    frame.visual(
        Cylinder(radius=0.022, length=0.030),
        origin=Origin(xyz=(-0.35, 0.14, BEND_Z - 0.050), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black, name="pump_pivot_boss",
    )

    # Conduit support clamps on the platform (connect conduit to frame).
    # Clamps bridge from beam interior (z=0.998) up through conduit bottom (z=1.04),
    # overlapping both for connectivity. Placed at y=±0.09 (within beam Y span [-0.13, 0.13]).
    _clamp_z_half = 0.026
    _clamp_z_center = 1.024  # [0.998, 1.050] overlaps beam (top=1.008) and conduit (bot=1.04)
    for i, y_pos in enumerate((-0.09, 0.09)):
        frame.visual(
            Box((0.028, 0.024, _clamp_z_half * 2)),
            origin=Origin(xyz=(CONDUIT_X, y_pos, _clamp_z_center)),
            material=dark, name=f"conduit_clamp_{i}",
        )

    # ==================================================================
    # RAM (prismatic – piston rod + carriage + former die)
    # ==================================================================
    ram = model.part("ram")

    # Piston rod.
    _cylinder_between(
        ram, "piston_rod",
        (-RAM_UPPER, 0.0, 0.0), (FORMER_LOCAL_X - 0.030, 0.0, 0.0),
        0.014, dark,
    )

    # Carriage plate – moved forward to clear cylinder body at rest.
    ram.visual(
        Box((0.10, 0.100, 0.016)),
        origin=Origin(xyz=(0.06, 0.0, -0.025)),
        material=dark, name="carriage_plate",
    )
    # Carriage bearing blocks (engage the guide rails).
    for i, y_s in enumerate((-1.0, 1.0)):
        ram.visual(
            Box((0.080, 0.018, 0.022)),
            origin=Origin(xyz=(0.06, y_s * 0.044, -0.028)),
            material=black, name=f"carriage_bearing_{i}",
        )

    # Die mount block – extended forward to overlap the die meshes.
    ram.visual(
        Box((0.086, 0.055, 0.050)),
        origin=Origin(xyz=(FORMER_LOCAL_X - 0.008, 0.0, 0.0)),
        material=black, name="die_mount_block",
    )

    # Former die – curved shoe (annular sector rotated from XZ → XY plane).
    die_theta0 = math.radians(-42.0)
    die_theta1 = math.radians(42.0)

    die_web = _annular_sector(
        inner_radius=0.030, outer_radius=0.055,
        thickness_y=0.040,
        theta0=die_theta0, theta1=die_theta1, segments=36,
    )
    ram.visual(
        mesh_from_geometry(die_web, "former_die_web"),
        origin=Origin(xyz=(FORMER_LOCAL_X, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black, name="former_die_web",
    )

    groove_bottom = _annular_sector(
        inner_radius=0.040, outer_radius=0.048,
        thickness_y=0.022,
        theta0=die_theta0, theta1=die_theta1, segments=36,
    )
    ram.visual(
        mesh_from_geometry(groove_bottom, "former_groove_floor"),
        origin=Origin(xyz=(FORMER_LOCAL_X, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark, name="former_groove_floor",
    )

    for i, y_off in enumerate((-0.016, 0.016)):
        lip = _annular_sector(
            inner_radius=0.046, outer_radius=0.058,
            thickness_y=0.008,
            theta0=die_theta0, theta1=die_theta1, segments=36,
            y_offset=y_off,
        )
        ram.visual(
            mesh_from_geometry(lip, f"former_groove_lip_{i}"),
            origin=Origin(xyz=(FORMER_LOCAL_X, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black, name=f"former_groove_lip_{i}",
        )

    # Cast reinforcing ribs on the die (kept short and low-angled to clear cylinder gland).
    for idx, ang in enumerate((math.radians(10), math.radians(30), math.radians(50))):
        _add_radial_box(
            ram, f"die_rib_{idx}", 0.040, ang,
            (0.008, 0.038, 0.035), black, y=0.0,
        )

    # ==================================================================
    # PUMP HANDLE (revolute – lever + grip)
    # ==================================================================
    pump = model.part("pump_handle")

    pump.visual(
        Cylinder(radius=0.018, length=0.028),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black, name="pump_boss",
    )
    _cylinder_between(pump, "pump_lever", (0.0, 0.0, 0.0), (-0.28, 0.0, 0.10), 0.010, yellow)
    pump.visual(
        Box((0.060, 0.022, 0.030)),
        origin=Origin(xyz=(-0.06, 0.0, 0.020), rpy=(0.0, -0.35, 0.0)),
        material=yellow, name="pump_web",
    )
    _cylinder_between(pump, "pump_grip", (-0.22, 0.0, 0.078), (-0.32, 0.0, 0.114), 0.017, rubber)

    # ==================================================================
    # CONDUIT (fixed to frame – sample EMT segment)
    # ==================================================================
    conduit = model.part("conduit")

    _cylinder_between(
        conduit, "conduit_tube",
        (CONDUIT_X, -0.22, BEND_Z), (CONDUIT_X, 0.22, BEND_Z),
        0.010, galvanized,
    )
    _cylinder_between(
        conduit, "conduit_stub_0",
        (CONDUIT_X, -0.22, BEND_Z), (CONDUIT_X, -0.36, BEND_Z),
        0.010, galvanized,
    )
    _cylinder_between(
        conduit, "conduit_stub_1",
        (CONDUIT_X, 0.22, BEND_Z), (CONDUIT_X, 0.36, BEND_Z),
        0.010, galvanized,
    )

    # ==================================================================
    # ARTICULATIONS
    # ==================================================================
    model.articulation(
        "frame_to_ram",
        ArticulationType.PRISMATIC,
        parent=frame, child=ram,
        origin=Origin(xyz=(RAM_ORIGIN_X, 0.0, BEND_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0, velocity=0.04, lower=0.0, upper=RAM_UPPER,
        ),
        meta={"mechanism": "hydraulic ram piston advances former die toward fixed rollers"},
    )

    model.articulation(
        "frame_to_pump",
        ArticulationType.REVOLUTE,
        parent=frame, child=pump,
        origin=Origin(xyz=(-0.35, 0.14, BEND_Z - 0.050)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0, velocity=2.5, lower=0.0, upper=PUMP_UPPER,
        ),
        meta={"mechanism": "manual pump handle actuates hydraulic ram"},
    )

    model.articulation(
        "frame_to_conduit",
        ArticulationType.FIXED,
        parent=frame, child=conduit,
        origin=Origin(),
        meta={"fit": "sample EMT conduit seated between former die and reaction rollers"},
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    ram = object_model.get_part("ram")
    pump = object_model.get_part("pump_handle")
    conduit = object_model.get_part("conduit")
    ram_joint = object_model.get_articulation("frame_to_ram")
    pump_joint = object_model.get_articulation("frame_to_pump")
    conduit_joint = object_model.get_articulation("frame_to_conduit")

    # -- identity -----------------------------------------------------------
    ctx.check(
        "small class is conduit bender",
        object_model.name == "hydraulic_conduit_bender"
        and object_model.meta.get("small_class") == "Conduit bender",
        details=f"name={object_model.name}, meta={object_model.meta}",
    )

    # -- joint types and limits --------------------------------------------
    ctx.check(
        "ram prismatic joint has correct type and stroke",
        ram_joint.articulation_type == ArticulationType.PRISMATIC
        and ram_joint.motion_limits is not None
        and math.isclose(ram_joint.motion_limits.lower, 0.0, abs_tol=1e-6)
        and math.isclose(ram_joint.motion_limits.upper, RAM_UPPER, abs_tol=1e-6),
        details=f"type={ram_joint.articulation_type}, limits={ram_joint.motion_limits}",
    )
    ctx.check(
        "pump handle revolute joint has correct type and limits",
        pump_joint.articulation_type == ArticulationType.REVOLUTE
        and pump_joint.motion_limits is not None
        and math.isclose(pump_joint.motion_limits.lower, 0.0, abs_tol=1e-6)
        and math.isclose(pump_joint.motion_limits.upper, PUMP_UPPER, abs_tol=1e-6),
        details=f"type={pump_joint.articulation_type}, limits={pump_joint.motion_limits}",
    )
    ctx.check(
        "conduit is fixed to frame",
        conduit_joint.articulation_type == ArticulationType.FIXED,
        details=f"conduit joint type={conduit_joint.articulation_type}",
    )

    # -- visual presence checks --------------------------------------------
    frame_names = {v.name for v in frame.visuals}
    ram_names = {v.name for v in ram.visuals}
    pump_names = {v.name for v in pump.visuals}
    conduit_names = {v.name for v in conduit.visuals}

    ctx.check(
        "frame has platform rails legs cylinder body rollers and degree markings",
        {
            "base_rail_0", "base_rail_1", "front_leg_0", "rear_leg_0",
            "platform_beam", "guide_rail_0", "guide_rail_1",
            "ram_cylinder_body", "ram_cylinder_rear_cap",
            "roller_body_0", "roller_body_1", "roller_pin_0", "roller_pin_1",
            "roller_upright_0", "roller_flange_0_0",
            "degree_plate", "degree_tick_0", "degree_pad_1",
            "warning_label", "terminal_screw_0",
            "hydraulic_fitting_0", "pump_pivot_boss",
        }.issubset(frame_names),
        details=str(sorted(frame_names)),
    )
    ctx.check(
        "ram has piston rod carriage former die with groove and ribs",
        {
            "piston_rod", "carriage_plate", "die_mount_block",
            "former_die_web", "former_groove_floor",
            "former_groove_lip_0", "former_groove_lip_1",
            "die_rib_0",
        }.issubset(ram_names),
        details=str(sorted(ram_names)),
    )
    ctx.check(
        "pump handle has lever grip and boss",
        {"pump_boss", "pump_lever", "pump_grip", "pump_web"}.issubset(pump_names),
        details=str(sorted(pump_names)),
    )
    ctx.check(
        "galvanised sample conduit has tube and stubs",
        {"conduit_tube", "conduit_stub_0", "conduit_stub_1"}.issubset(conduit_names),
        details=str(sorted(conduit_names)),
    )

    # -- intentional overlaps -----------------------------------------------
    ctx.allow_overlap(
        ram, frame,
        elem_a="piston_rod", elem_b="ram_cylinder_body",
        reason="The piston rod is intentionally represented as sliding inside the hydraulic cylinder body.",
    )
    ctx.allow_overlap(
        ram, frame,
        elem_a="piston_rod", elem_b="ram_cylinder_front_gland",
        reason="The piston rod passes through the cylinder front gland seal as expected in a hydraulic ram.",
    )
    ctx.allow_overlap(
        frame, pump,
        elem_a="pump_pivot_boss", elem_b="pump_boss",
        reason="The pump handle pivot boss is intentionally mounted on the frame pivot boss as a bearing fit.",
    )
    ctx.allow_overlap(
        frame, pump,
        elem_a="pump_pivot_boss", elem_b="pump_lever",
        reason="The pump lever tube starts at the pivot center inside the boss as a hinge connection.",
    )
    ctx.allow_overlap(
        frame, conduit,
        elem_a="conduit_clamp_0", elem_b="conduit_tube",
        reason="The conduit is intentionally seated on the frame clamp as a support mount.",
    )
    ctx.allow_overlap(
        frame, conduit,
        elem_a="conduit_clamp_1", elem_b="conduit_tube",
        reason="The conduit is intentionally seated on the second frame clamp as a support mount.",
    )

    # Proof that conduit is seated on clamps.
    ctx.expect_contact(
        conduit, frame,
        elem_a="conduit_tube", elem_b="conduit_clamp_0",
        name="conduit seated on clamp 0",
    )
    ctx.expect_contact(
        conduit, frame,
        elem_a="conduit_tube", elem_b="conduit_clamp_1",
        name="conduit seated on clamp 1",
    )

    # -- conduit positioning -----------------------------------------------
    ctx.expect_gap(
        conduit, ram, axis="x",
        positive_elem="conduit_tube", negative_elem="former_die_web",
        min_gap=0.001,
        name="conduit ahead of former die at rest",
    )

    # -- ram advance proof --------------------------------------------------
    rest_die = ctx.part_element_world_aabb(ram, elem="former_die_web")
    with ctx.pose({ram_joint: RAM_UPPER}):
        ext_die = ctx.part_element_world_aabb(ram, elem="former_die_web")
    ctx.check(
        "ram prismatic stroke advances former die toward conduit",
        rest_die is not None
        and ext_die is not None
        and ext_die[1][0] > rest_die[1][0] + 0.08,
        details=f"rest_max_x={rest_die[1][0] if rest_die else None}, ext_max_x={ext_die[1][0] if ext_die else None}",
    )

    # -- pose sweep --------------------------------------------------------
    poses = {
        "rest": 0.0,
        "mid_stroke": RAM_UPPER * 0.5,
        "near_upper": RAM_UPPER - 0.005,
    }
    for pose_name, q in poses.items():
        with ctx.pose({ram_joint: q}):
            ctx.expect_gap(
                conduit, ram, axis="x",
                positive_elem="conduit_tube", negative_elem="former_groove_floor",
                min_gap=-0.004,
                name=f"{pose_name} conduit clears former groove floor",
            )
            # Carriage stays within platform beam Y span.
            ctx.expect_within(
                ram, frame, axes="y",
                inner_elem="carriage_plate", outer_elem="platform_beam",
                margin=0.0,
                name=f"{pose_name} carriage stays within platform beam",
            )
            # Former clears reaction rollers.
            ctx.expect_gap(
                frame, ram, axis="x",
                positive_elem="roller_body_0", negative_elem="former_die_web",
                min_gap=-0.005,
                name=f"{pose_name} former clears roller 0",
            )
            ctx.expect_gap(
                frame, ram, axis="x",
                positive_elem="roller_body_1", negative_elem="former_die_web",
                min_gap=-0.005,
                name=f"{pose_name} former clears roller 1",
            )

    # -- pump handle pose check --------------------------------------------
    rest_grip = ctx.part_element_world_aabb(pump, elem="pump_grip")
    with ctx.pose({pump_joint: PUMP_UPPER}):
        pumped_grip = ctx.part_element_world_aabb(pump, elem="pump_grip")
    ctx.check(
        "pump handle stroke visibly lowers grip",
        rest_grip is not None
        and pumped_grip is not None
        and pumped_grip[0][2] < rest_grip[0][2] - 0.08,
        details=f"rest_min_z={rest_grip[0][2] if rest_grip else None}, pumped_min_z={pumped_grip[0][2] if pumped_grip else None}",
    )

    # -- floor clearance ----------------------------------------------------
    for pose_name, q in poses.items():
        with ctx.pose({ram_joint: q}):
            carriage_aabb = ctx.part_element_world_aabb(ram, elem="carriage_plate")
            ctx.check(
                f"{pose_name} carriage clears floor",
                carriage_aabb is not None and carriage_aabb[0][2] > 0.50,
                details=f"carriage_min_z={carriage_aabb[0][2] if carriage_aabb else None}",
            )

    return ctx.report()


object_model = build_object_model()
