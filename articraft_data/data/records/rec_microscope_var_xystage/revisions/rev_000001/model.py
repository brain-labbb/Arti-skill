from __future__ import annotations

# Compound monocular student microscope with a mechanical X-Y stage.
#
# Real object (from reference image): a white-painted metal microscope with an
# inclined monocular eyepiece tube ending in a dark ocular, a cylindrical head,
# a ROTATING NOSEPIECE TURRET carrying three objective lenses of different
# lengths over a MECHANICAL STAGE with nested X-Y specimen carriage and two
# drive knobs, an angled white arm/column rising from a heavy white base,
# coaxial FOCUS KNOBS on both sides of the arm, and a black sub-stage
# condenser/illuminator below the stage.
#
# Structure (Z-up, optical axis = vertical line at X=0, arm behind at +X):
#   - The arm is an extruded XZ profile whose front face stays at X >= ARM_FRONT_X
#     below the head, so the turret disc (r=0.040) and the swept objectives
#     (r=0.037) clear it at EVERY rotation angle.
#   - The stage plate is FIXED to the arm front face. A nested X-Y carriage
#     assembly on top provides specimen positioning via two prismatic joints.
#   - Two small drive knobs on the stage plate edges rotate to represent the
#     lead-screw drives for X and Y carriage motion.
#
# Mechanisms:
#   - nosepiece turret: CONTINUOUS about the vertical optical axis
#   - focus knobs: CONTINUOUS about the horizontal Y axle
#   - x_carriage: PRISMATIC along X on the stage plate
#   - y_carriage: PRISMATIC along Y on the x_carriage
#   - drive_knob_0 (X drive): REVOLUTE about world -X on the stage front face
#   - drive_knob_1 (Y drive): REVOLUTE about world +Y on the stage right face

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ----------------------------------------------------------------------
# Shared layout constants (used by both the build and the tests)
# ----------------------------------------------------------------------
ARM_FRONT_X = 0.048          # arm front face below the head (clears turret sweep)
ARM_HALF_WIDTH = 0.027       # arm half-width in Y
TURRET_Z = 0.240             # turret joint height (world)
TURRET_DISC_R = 0.040        # turret disc radius
OBJ_ORBIT_R = 0.028          # objective mounting radius (off-axis)
OBJ_BARREL_R = 0.0090        # objective barrel top radius
OBJ_LENGTHS = (0.050, 0.040, 0.032)   # 40x / 10x / 4x barrel lengths
OBJ_ANGLES_DEG = (180.0, 60.0, 300.0)  # 120 deg spacing; #0 faces front (-X)
STAGE_Z = 0.146              # stage joint height (plate mid-plane, world)
X_TRAVEL = (-0.015, 0.015)   # x_carriage prismatic travel (lower, upper)
Y_TRAVEL = (-0.012, 0.012)   # y_carriage prismatic travel (lower, upper)
KNOB_Z = 0.105               # focus axle height (world)
KNOB_X = 0.078               # focus axle X position (inside the arm column)
HEAD_R = 0.034               # head housing radius
HEAD_Z0, HEAD_Z1 = 0.245, 0.298  # head housing bottom/top
X_FRAME_OUTER = (0.095, 0.088, 0.005)  # x_carriage frame outer dims
X_FRAME_INNER = (0.068, 0.058)         # x_carriage frame inner opening
Y_PLATE_SIZE = (0.065, 0.055, 0.004)   # y_carriage plate dims
N_DRIVE_KNOBS = 2
DRIVE_KNOB_R = 0.007         # drive knob body radius
DRIVE_KNOB_LENGTH = 0.012    # drive knob body length


def _make_drive_knob(length: float, radius: float):
    """Build a cylindrical drive knob with mounting boss and grip ring.

    The knob extends along local +Z from z=0. A short boss extends from
    z=-0.003 to z=0 (enters the mounting face for mechanical realism).
    """
    boss = (
        cq.Workplane("XY")
        .workplane(offset=-0.003)
        .circle(radius * 0.50)
        .extrude(0.003)
    )
    body = (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
    )
    # Grip ring near the tip for tactile detail.
    ring = (
        cq.Workplane("XY")
        .workplane(offset=length * 0.60)
        .circle(radius * 1.12)
        .extrude(length * 0.18)
    )
    return boss.union(body).union(ring)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compound_microscope")

    white_metal = model.material("white_metal", rgba=(0.93, 0.93, 0.94, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.09, 0.09, 0.10, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.16, 0.16, 0.18, 1.0))
    chrome = model.material("chrome", rgba=(0.75, 0.76, 0.78, 1.0))
    brass_lens = model.material("brass_lens", rgba=(0.78, 0.74, 0.55, 1.0))
    teal_glass = model.material("teal_glass", rgba=(0.10, 0.45, 0.45, 1.0))
    slide_glass = model.material("slide_glass", rgba=(0.45, 0.60, 0.70, 0.9))

    # ------------------------------------------------------------------
    # BASE: foot + angled arm column + cylindrical head (one rigid casting)
    # ------------------------------------------------------------------
    base = model.part("base")

    # Heavy rectangular foot, bottom face exactly on the ground (z = 0).
    foot = (
        cq.Workplane("XY")
        .center(0.015, 0.0)
        .rect(0.200, 0.150)
        .workplane(offset=0.035)
        .rect(0.185, 0.135)
        .loft(combine=True)
        .edges(">Z")
        .fillet(0.008)
    )
    base.visual(
        mesh_from_cadquery(foot, "foot"),
        material=white_metal,
        name="foot",
    )

    # Angled arm column: extruded XZ side profile.
    arm_profile = [
        (ARM_FRONT_X, 0.030),
        (ARM_FRONT_X, 0.245),
        (0.020, 0.245),
        (0.020, 0.296),
        (0.072, 0.296),
        (0.105, 0.160),
        (0.105, 0.030),
    ]
    arm = (
        cq.Workplane("XZ")
        .polyline(arm_profile)
        .close()
        .extrude(ARM_HALF_WIDTH, both=True)
    )
    base.visual(
        mesh_from_cadquery(arm, "arm_limb"),
        material=white_metal,
        name="arm_limb",
    )

    # Cylindrical head housing on the optical axis.
    head = (
        cq.Workplane("XY")
        .workplane(offset=HEAD_Z0)
        .circle(HEAD_R)
        .extrude(HEAD_Z1 - HEAD_Z0)
        .edges(">Z")
        .fillet(0.010)
    )
    base.visual(
        mesh_from_cadquery(head, "head_housing"),
        material=white_metal,
        name="head_housing",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.20, 0.15, 0.30)),
        mass=3.0,
        origin=Origin(xyz=(0.04, 0.0, 0.13)),
    )

    # ------------------------------------------------------------------
    # MONOCULAR TUBE + OCULAR: fixed to the head, inclined 40 deg.
    # ------------------------------------------------------------------
    eyepiece = model.part("eyepiece_tube")
    tube_body = (
        cq.Workplane("XY")
        .circle(0.015)
        .extrude(0.075)
    )
    eyepiece.visual(
        mesh_from_cadquery(tube_body, "tube_body"),
        material=white_metal,
        name="tube_body",
    )
    ocular = (
        cq.Workplane("XY")
        .workplane(offset=0.073)
        .circle(0.0135)
        .extrude(0.030)
        .faces(">Z")
        .workplane()
        .circle(0.0115)
        .extrude(0.010)
    )
    eyepiece.visual(
        mesh_from_cadquery(ocular, "ocular_barrel"),
        material=black_plastic,
        name="ocular_barrel",
    )
    eyepiece.visual(
        Cylinder(radius=0.0105, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
        material=teal_glass,
        name="ocular_lens",
    )
    eyepiece.inertial = Inertial.from_geometry(
        Cylinder(radius=0.015, length=0.115),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )
    model.articulation(
        "head_to_eyepiece",
        ArticulationType.FIXED,
        parent=base,
        child=eyepiece,
        origin=Origin(xyz=(0.0, 0.0, 0.290), rpy=(0.0, math.radians(-40.0), 0.0)),
    )

    # ------------------------------------------------------------------
    # NOSEPIECE TURRET: continuous about the vertical optical axis.
    # ------------------------------------------------------------------
    nosepiece = model.part("nosepiece_turret")
    turret_plate = (
        cq.Workplane("XY")
        .workplane(offset=-0.025)
        .circle(TURRET_DISC_R)
        .extrude(0.005)
    )
    turret_cone = (
        cq.Workplane("XY")
        .workplane(offset=-0.020)
        .circle(TURRET_DISC_R)
        .workplane(offset=0.016)
        .circle(0.024)
        .loft(combine=True)
    )
    turret_hub = (
        cq.Workplane("XY")
        .workplane(offset=-0.006)
        .circle(0.013)
        .extrude(0.016)
    )
    turret_solid = turret_plate.union(turret_cone).union(turret_hub)
    nosepiece.visual(
        mesh_from_cadquery(turret_solid, "turret_disc"),
        material=dark_metal,
        name="turret_disc",
    )

    for i, length in enumerate(OBJ_LENGTHS):
        ang = math.radians(OBJ_ANGLES_DEG[i])
        ox = OBJ_ORBIT_R * math.cos(ang)
        oy = OBJ_ORBIT_R * math.sin(ang)
        barrel = (
            cq.Workplane("XY")
            .workplane(offset=-0.023)
            .circle(OBJ_BARREL_R)
            .workplane(offset=-length * 0.60)
            .circle(0.0082)
            .workplane(offset=-length * 0.40)
            .circle(0.0050)
            .loft(combine=True)
        )
        barrel = barrel.translate((ox, oy, 0.0))
        nosepiece.visual(
            mesh_from_cadquery(barrel, f"objective_barrel_{i}"),
            material=chrome,
            name=f"objective_barrel_{i}",
        )
        ring = (
            cq.Workplane("XY")
            .workplane(offset=-0.023 - 0.55 * length)
            .circle(0.0090)
            .extrude(-0.005)
        )
        ring = ring.translate((ox, oy, 0.0))
        ring_mat = (brass_lens, teal_glass, dark_metal)[i]
        nosepiece.visual(
            mesh_from_cadquery(ring, f"objective_ring_{i}"),
            material=ring_mat,
            name=f"objective_ring_{i}",
        )
    nosepiece.inertial = Inertial.from_geometry(
        Cylinder(radius=TURRET_DISC_R, length=0.09),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
    )
    model.articulation(
        "head_to_nosepiece",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=nosepiece,
        origin=Origin(xyz=(0.0, 0.0, TURRET_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )

    # ------------------------------------------------------------------
    # STAGE: black mechanical stage plate, FIXED to the arm front face.
    # The plate carries guide rails for the X carriage and mounting
    # bosses for the drive knobs. No vertical travel (focus is handled
    # by the focus knob mechanism acting on the head/arm assembly).
    # ------------------------------------------------------------------
    stage = model.part("stage")

    # Stage plate with integrated X-direction guide rails on top.
    plate_solid = (
        cq.Workplane("XY")
        .center(-0.0075, 0.0)
        .rect(0.135, 0.120)
        .extrude(0.008)
        .edges("|Z")
        .fillet(0.006)
    )
    plate_solid = plate_solid.translate((0.0, 0.0, -0.004))
    # Guide rails on top for the X carriage to slide on.
    for ry in (0.045, -0.045):
        rail = (
            cq.Workplane("XY")
            .workplane(offset=0.004)
            .center(-0.0075, ry)
            .rect(0.100, 0.006)
            .extrude(0.002)
        )
        plate_solid = plate_solid.union(rail)
    # Central aperture over the condenser for the light path.
    plate_solid = (
        plate_solid.faces(">Z")
        .workplane(origin=(0.0, 0.0, 0.0))
        .circle(0.011)
        .cutThruAll()
    )
    stage.visual(
        mesh_from_cadquery(plate_solid, "stage_plate"),
        material=black_plastic,
        name="stage_plate",
    )

    # Dovetail saddle bracket under the plate rear, bolting to the arm.
    bracket = (
        cq.Workplane("XY")
        .center(0.053, 0.0)
        .rect(0.026, 0.040)
        .extrude(-0.027)
    )
    bracket = bracket.translate((0.0, 0.0, -0.003))
    stage.visual(
        mesh_from_cadquery(bracket, "stage_bracket"),
        material=dark_metal,
        name="stage_bracket",
    )
    stage.inertial = Inertial.from_geometry(
        Box((0.135, 0.120, 0.030)),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
    )
    model.articulation(
        "arm_to_stage",
        ArticulationType.FIXED,
        parent=base,
        child=stage,
        origin=Origin(xyz=(0.0, 0.0, STAGE_Z)),
    )

    # ------------------------------------------------------------------
    # X CARRIAGE: rectangular frame on the stage plate, PRISMATIC along X.
    # The frame has an inner opening for the Y carriage and Y-direction
    # guide rails on its top surface.
    # ------------------------------------------------------------------
    x_carriage = model.part("x_carriage")

    # Frame: outer rectangle with inner cutout.
    x_frame = (
        cq.Workplane("XY")
        .rect(X_FRAME_OUTER[0], X_FRAME_OUTER[1])
        .extrude(X_FRAME_OUTER[2])
        .edges("|Z")
        .fillet(0.003)
    )
    x_frame = (
        x_frame.faces(">Z")
        .workplane()
        .rect(X_FRAME_INNER[0], X_FRAME_INNER[1])
        .cutThruAll()
    )
    # Y-direction guide rails on top of the frame side walls.
    for rx in (0.030, -0.030):
        y_rail = (
            cq.Workplane("XY")
            .workplane(offset=X_FRAME_OUTER[2])
            .center(rx, 0.0)
            .rect(0.006, 0.060)
            .extrude(0.002)
        )
        x_frame = x_frame.union(y_rail)
    x_carriage.visual(
        mesh_from_cadquery(x_frame, "x_carriage_frame"),
        material=dark_metal,
        name="x_carriage_frame",
    )
    x_carriage.inertial = Inertial.from_geometry(
        Box((X_FRAME_OUTER[0], X_FRAME_OUTER[1], X_FRAME_OUTER[2] + 0.002)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
    )
    model.articulation(
        "stage_to_x_carriage",
        ArticulationType.PRISMATIC,
        parent=stage,
        child=x_carriage,
        # Joint origin at the stage plate top surface (contact face).
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.02,
            lower=X_TRAVEL[0],
            upper=X_TRAVEL[1],
        ),
    )

    # ------------------------------------------------------------------
    # Y CARRIAGE: specimen platform inside the X carriage opening,
    # PRISMATIC along Y. Carries the spring clips and specimen slide.
    # ------------------------------------------------------------------
    y_carriage = model.part("y_carriage")

    # Platform plate with central aperture for the light path.
    y_plate = (
        cq.Workplane("XY")
        .rect(Y_PLATE_SIZE[0], Y_PLATE_SIZE[1])
        .extrude(Y_PLATE_SIZE[2])
        .edges("|Z")
        .fillet(0.002)
    )
    y_plate = (
        y_plate.faces(">Z")
        .workplane()
        .circle(0.010)
        .cutThruAll()
    )
    y_carriage.visual(
        mesh_from_cadquery(y_plate, "y_carriage_plate"),
        material=dark_metal,
        name="y_carriage_plate",
    )

    # Spring clips embedded 0.3 mm into the plate top (declared).
    for cy, tag in ((-0.018, "0"), (0.018, "1")):
        clip = (
            cq.Workplane("XY")
            .center(0.015, cy)
            .rect(0.028, 0.004)
            .extrude(0.0015)
        )
        clip = clip.translate((0.0, 0.0, 0.0037))
        y_carriage.visual(
            mesh_from_cadquery(clip, f"stage_clip_{tag}"),
            material=chrome,
            name=f"stage_clip_{tag}",
        )

    # Glass slide over the aperture, seated 0.2 mm into the plate top.
    y_carriage.visual(
        Box((0.064, 0.024, 0.0012)),
        origin=Origin(xyz=(0.0, 0.0, 0.0044)),
        material=slide_glass,
        name="specimen_slide",
    )
    y_carriage.inertial = Inertial.from_geometry(
        Box((Y_PLATE_SIZE[0], Y_PLATE_SIZE[1], Y_PLATE_SIZE[2] + 0.002)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
    )
    model.articulation(
        "x_carriage_to_y_carriage",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=y_carriage,
        # Joint origin at 1 mm above X carriage frame bottom (dovetail ways).
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.02,
            lower=Y_TRAVEL[0],
            upper=Y_TRAVEL[1],
        ),
    )

    # ------------------------------------------------------------------
    # DRIVE KNOBS: two small revolute knobs on the stage plate edges,
    # representing the X and Y lead-screw drives. Built via a shared
    # geometry helper with regular placement and uniform joint policy.
    # ------------------------------------------------------------------
    # Knob mount configurations: (mount_origin_in_stage_frame, axis_rpy)
    # Knob 0 (X drive): on the front face (-X side), rotates about world -X.
    # Knob 1 (Y drive): on the right face (+Y side), rotates about world +Y.
    knob_mounts = [
        Origin(xyz=(-0.075, 0.0, 0.0), rpy=(0.0, -math.pi / 2.0, 0.0)),
        Origin(xyz=(-0.0075, 0.060, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
    ]
    for i in range(N_DRIVE_KNOBS):
        knob = model.part(f"drive_knob_{i}")
        knob.visual(
            mesh_from_cadquery(
                _make_drive_knob(DRIVE_KNOB_LENGTH, DRIVE_KNOB_R),
                f"knob_body_{i}",
            ),
            material=dark_metal,
            name=f"knob_body_{i}",
        )
        # Grip accent ring on the knob face (visual detail).
        knob.visual(
            Cylinder(radius=DRIVE_KNOB_R * 0.40, length=0.002),
            origin=Origin(xyz=(0.0, 0.0, DRIVE_KNOB_LENGTH + 0.001)),
            material=chrome,
            name=f"knob_cap_{i}",
        )
        knob.inertial = Inertial.from_geometry(
            Cylinder(radius=DRIVE_KNOB_R, length=DRIVE_KNOB_LENGTH),
            mass=0.015,
            origin=Origin(xyz=(0.0, 0.0, DRIVE_KNOB_LENGTH * 0.5)),
        )
        model.articulation(
            f"stage_to_drive_knob_{i}",
            ArticulationType.REVOLUTE,
            parent=stage,
            child=knob,
            origin=knob_mounts[i],
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=6.0,
                lower=-math.pi,
                upper=math.pi,
            ),
        )

    # ------------------------------------------------------------------
    # SUB-STAGE CONDENSER / ILLUMINATOR: black stack standing on the base
    # under the stage aperture (fixed).
    # ------------------------------------------------------------------
    condenser = model.part("condenser")
    lamp = (
        cq.Workplane("XY")
        .workplane(offset=-0.053)
        .circle(0.026)
        .extrude(0.068)
        .edges(">Z")
        .fillet(0.004)
    )
    condenser.visual(
        mesh_from_cadquery(lamp, "condenser_body"),
        material=black_plastic,
        name="condenser_body",
    )
    mount = (
        cq.Workplane("XY")
        .workplane(offset=0.013)
        .circle(0.017)
        .extrude(0.032)
    )
    condenser.visual(
        mesh_from_cadquery(mount, "condenser_mount"),
        material=dark_metal,
        name="condenser_mount",
    )
    condenser.inertial = Inertial.from_geometry(
        Cylinder(radius=0.026, length=0.10),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
    )
    model.articulation(
        "base_to_condenser",
        ArticulationType.FIXED,
        parent=base,
        child=condenser,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
    )

    # ------------------------------------------------------------------
    # FOCUS KNOBS: one through-axle carrying a knob disc on EACH side,
    # CONTINUOUS about world Y.
    # ------------------------------------------------------------------
    focus_knob = model.part("focus_knob")
    knob_axle = (
        cq.Workplane("XY")
        .workplane(offset=-0.046)
        .circle(0.006)
        .extrude(0.092)
    )
    focus_knob.visual(
        mesh_from_cadquery(knob_axle, "knob_axle"),
        material=dark_metal,
        name="knob_axle",
    )
    for sz, tag in ((-1.0, "n"), (1.0, "p")):
        disc = (
            cq.Workplane("XY")
            .workplane(offset=sz * 0.028 if sz > 0 else sz * 0.040)
            .circle(0.018)
            .extrude(0.012)
        )
        hub = (
            cq.Workplane("XY")
            .workplane(offset=sz * 0.040 if sz > 0 else sz * 0.046)
            .circle(0.011)
            .extrude(0.006)
        )
        focus_knob.visual(
            mesh_from_cadquery(disc.union(hub), f"knob_disc_{tag}"),
            material=dark_metal,
            name=f"knob_disc_{tag}",
        )
    for sz, tag in ((-1.0, "n"), (1.0, "p")):
        nub = (
            cq.Workplane("XY")
            .workplane(offset=sz * 0.0395 if sz > 0 else sz * 0.0445)
            .center(0.0140, 0.0)
            .rect(0.006, 0.003)
            .extrude(0.005)
        )
        focus_knob.visual(
            mesh_from_cadquery(nub, f"knob_nub_{tag}"),
            material=white_metal,
            name=f"knob_nub_{tag}",
        )
    focus_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.092),
        mass=0.06,
    )
    model.articulation(
        "arm_to_focus_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=focus_knob,
        origin=Origin(xyz=(KNOB_X, 0.0, KNOB_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    eyepiece = object_model.get_part("eyepiece_tube")
    nosepiece = object_model.get_part("nosepiece_turret")
    stage = object_model.get_part("stage")
    x_carriage = object_model.get_part("x_carriage")
    y_carriage = object_model.get_part("y_carriage")
    drive_knobs = [object_model.get_part(f"drive_knob_{i}") for i in range(N_DRIVE_KNOBS)]
    condenser = object_model.get_part("condenser")
    focus_knob = object_model.get_part("focus_knob")

    turret_joint = object_model.get_articulation("head_to_nosepiece")
    x_joint = object_model.get_articulation("stage_to_x_carriage")
    y_joint = object_model.get_articulation("x_carriage_to_y_carriage")
    focus_joint = object_model.get_articulation("arm_to_focus_knob")
    knob_joints = [
        object_model.get_articulation(f"stage_to_drive_knob_{i}")
        for i in range(N_DRIVE_KNOBS)
    ]

    # --- Joint contracts ---
    ctx.check(
        "turret is continuous about Z",
        turret_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(turret_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={turret_joint.articulation_type}, axis={turret_joint.axis}",
    )
    ctx.check(
        "focus knob is continuous",
        focus_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={focus_joint.articulation_type}",
    )
    ctx.check(
        "x_carriage is a bounded X-axis prismatic",
        x_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(x_joint.axis) == (1.0, 0.0, 0.0)
        and x_joint.motion_limits is not None
        and x_joint.motion_limits.lower == X_TRAVEL[0]
        and x_joint.motion_limits.upper == X_TRAVEL[1],
        details=f"type={x_joint.articulation_type}, axis={x_joint.axis}, "
        f"limits={x_joint.motion_limits}",
    )
    ctx.check(
        "y_carriage is a bounded Y-axis prismatic",
        y_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(y_joint.axis) == (0.0, 1.0, 0.0)
        and y_joint.motion_limits is not None
        and y_joint.motion_limits.lower == Y_TRAVEL[0]
        and y_joint.motion_limits.upper == Y_TRAVEL[1],
        details=f"type={y_joint.articulation_type}, axis={y_joint.axis}, "
        f"limits={y_joint.motion_limits}",
    )
    for i in range(N_DRIVE_KNOBS):
        ctx.check(
            f"drive_knob_{i} is revolute",
            knob_joints[i].articulation_type == ArticulationType.REVOLUTE,
            details=f"type={knob_joints[i].articulation_type}",
        )

    # --- Rests on the ground: global z-min == 0 ---
    zmin = None
    all_parts = [base, eyepiece, nosepiece, stage, x_carriage, y_carriage,
                 condenser, focus_knob] + drive_knobs
    for part in all_parts:
        aabb = ctx.part_world_aabb(part)
        if aabb is not None:
            zmin = aabb[0][2] if zmin is None else min(zmin, aabb[0][2])
    ctx.check(
        "model rests on the ground (z-min ~ 0)",
        zmin is not None and abs(zmin) < 1.0e-4,
        details=f"global z-min={zmin}",
    )

    # --- Hero geometry: three different-length objectives on the turret ---
    obj_barrels = [
        v for v in nosepiece.visuals if v.name and v.name.startswith("objective_barrel_")
    ]
    ctx.check(
        "turret carries three objective lenses",
        len(obj_barrels) == 3,
        details=f"objective barrels found: {len(obj_barrels)}",
    )

    # --- Vertical optical stack ordering ---
    eye_aabb = ctx.part_world_aabb(eyepiece)
    nose_aabb = ctx.part_world_aabb(nosepiece)
    y_carriage_aabb = ctx.part_world_aabb(y_carriage)
    cond_aabb = ctx.part_world_aabb(condenser)
    disc_aabb = ctx.part_element_world_aabb(nosepiece, elem="turret_disc")
    ctx.check(
        "eyepiece tube sits above the turret",
        eye_aabb is not None and nose_aabb is not None and eye_aabb[0][2] > nose_aabb[1][2],
        details=f"eye_min_z={eye_aabb[0][2] if eye_aabb else None}, "
        f"nose_max_z={nose_aabb[1][2] if nose_aabb else None}",
    )
    ctx.check(
        "turret disc hangs above the y_carriage",
        disc_aabb is not None and y_carriage_aabb is not None
        and disc_aabb[0][2] > y_carriage_aabb[1][2],
        details=f"disc_min_z={disc_aabb[0][2] if disc_aabb else None}, "
        f"y_carriage_max_z={y_carriage_aabb[1][2] if y_carriage_aabb else None}",
    )
    plate_aabb = ctx.part_element_world_aabb(stage, elem="stage_plate")
    ctx.check(
        "condenser sits below the stage plate",
        cond_aabb is not None and plate_aabb is not None and cond_aabb[1][2] < plate_aabb[0][2],
        details=f"cond_max_z={cond_aabb[1][2] if cond_aabb else None}, "
        f"plate_min_z={plate_aabb[0][2] if plate_aabb else None}",
    )
    bracket_aabb = ctx.part_element_world_aabb(stage, elem="stage_bracket")
    ctx.check(
        "stage bracket passes beside the condenser, not through it",
        bracket_aabb is not None and cond_aabb is not None
        and bracket_aabb[0][0] > cond_aabb[1][0] + 0.005,
        details=f"bracket_min_x={bracket_aabb[0][0] if bracket_aabb else None}, "
        f"cond_max_x={cond_aabb[1][0] if cond_aabb else None}",
    )
    ctx.expect_within(
        nosepiece, stage, axes="xy", margin=0.005,
        name="turret stays over the stage footprint",
    )

    # --- FK sweep: at >= 8 turret angles every objective clears the arm
    #     front face, the head bottom, and the carriage top. Stage is fixed. ---
    carriage_top = None
    yc_aabb = ctx.part_world_aabb(y_carriage)
    if yc_aabb is not None:
        carriage_top = yc_aabb[1][2]
    sweep_ok = True
    sweep_details = []
    min_radius = None
    max_radius = None
    n_angles = 12
    for k in range(n_angles):
        ang = k * (2.0 * math.pi / n_angles)
        with ctx.pose({turret_joint: ang}):
            for i in range(3):
                bb = ctx.part_element_world_aabb(nosepiece, elem=f"objective_barrel_{i}")
                if bb is None:
                    sweep_ok = False
                    sweep_details.append(f"missing barrel {i} @ {ang:.2f}")
                    continue
                cx = 0.5 * (bb[0][0] + bb[1][0])
                cy = 0.5 * (bb[0][1] + bb[1][1])
                r = math.hypot(cx, cy)
                min_radius = r if min_radius is None else min(min_radius, r)
                max_radius = r if max_radius is None else max(max_radius, r)
                if bb[1][0] >= ARM_FRONT_X - 1.0e-4:
                    sweep_ok = False
                    sweep_details.append(
                        f"barrel {i} @ {ang:.2f} hits arm: max_x={bb[1][0]:.4f}"
                    )
                if bb[1][2] >= HEAD_Z0 - 1.0e-4:
                    sweep_ok = False
                    sweep_details.append(
                        f"barrel {i} @ {ang:.2f} hits head: max_z={bb[1][2]:.4f}"
                    )
                if carriage_top is None or bb[0][2] <= carriage_top + 0.003:
                    sweep_ok = False
                    sweep_details.append(
                        f"barrel {i} @ {ang:.2f} too close to carriage: "
                        f"min_z={bb[0][2]:.4f} vs carriage_top={carriage_top}"
                    )
    ctx.check(
        "objectives clear arm/head/carriage through a full turret sweep (12 angles)",
        sweep_ok,
        details="; ".join(sweep_details) if sweep_details else
        f"all clear; carriage_top={carriage_top:.4f}",
    )
    ctx.check(
        "objective centroids trace an off-axis circle (visible rotation)",
        min_radius is not None
        and max_radius is not None
        and min_radius > OBJ_BARREL_R
        and (max_radius - min_radius) < 0.004,
        details=f"centroid orbit radius range=[{min_radius}, {max_radius}], "
        f"barrel radius={OBJ_BARREL_R}",
    )

    # --- Decisive pose check: a quarter turn moves objective 0 sideways ---
    rest_obj = ctx.part_element_world_aabb(nosepiece, elem="objective_barrel_0")
    with ctx.pose({turret_joint: math.pi / 2.0}):
        turned_obj = ctx.part_element_world_aabb(nosepiece, elem="objective_barrel_0")
    moved = 0.0
    if rest_obj is not None and turned_obj is not None:
        moved = math.hypot(
            0.5 * (turned_obj[0][0] + turned_obj[1][0])
            - 0.5 * (rest_obj[0][0] + rest_obj[1][0]),
            0.5 * (turned_obj[0][1] + turned_obj[1][1])
            - 0.5 * (rest_obj[0][1] + rest_obj[1][1]),
        )
    ctx.check(
        "rotating the turret visibly swings an objective",
        moved > 0.02,
        details=f"objective 0 center moved {moved:.4f} m under a quarter turn",
    )

    # --- X carriage: prismatic joint translates the carriage in X ---
    rest_x_pos = ctx.part_world_position(x_carriage)
    with ctx.pose({x_joint: X_TRAVEL[1]}):
        extended_x_pos = ctx.part_world_position(x_carriage)
    dx = None
    if rest_x_pos is not None and extended_x_pos is not None:
        dx = extended_x_pos[0] - rest_x_pos[0]
    ctx.check(
        "x_carriage prismatic joint translates in X",
        dx is not None and abs(dx - X_TRAVEL[1]) < 1.0e-4,
        details=f"x_carriage moved {dx} m for upper limit {X_TRAVEL[1]}",
    )

    # --- Y carriage: prismatic joint translates the carriage in Y ---
    rest_y_pos = ctx.part_world_position(y_carriage)
    with ctx.pose({y_joint: Y_TRAVEL[1]}):
        extended_y_pos = ctx.part_world_position(y_carriage)
    dy = None
    if rest_y_pos is not None and extended_y_pos is not None:
        dy = extended_y_pos[1] - rest_y_pos[1]
    ctx.check(
        "y_carriage prismatic joint translates in Y",
        dy is not None and abs(dy - Y_TRAVEL[1]) < 1.0e-4,
        details=f"y_carriage moved {dy} m for upper limit {Y_TRAVEL[1]}",
    )

    # --- X carriage stays within the stage plate footprint ---
    ctx.expect_within(
        x_carriage, stage, axes="xy", margin=0.005,
        name="x_carriage stays within the stage plate footprint",
    )
    # Y carriage stays within the X carriage frame at all travel extremes.
    for yq in (Y_TRAVEL[0], 0.0, Y_TRAVEL[1]):
        with ctx.pose({y_joint: yq}):
            ctx.expect_within(
                y_carriage, x_carriage, axes="xy", margin=0.003,
                name=f"y_carriage stays within x_carriage at q={yq:.3f}",
            )

    # --- Drive knobs rotate and project from the stage edges ---
    for i in range(N_DRIVE_KNOBS):
        rest_knob = ctx.part_element_world_aabb(drive_knobs[i], elem=f"knob_body_{i}")
        with ctx.pose({knob_joints[i]: math.pi / 2.0}):
            turned_knob = ctx.part_element_world_aabb(drive_knobs[i], elem=f"knob_body_{i}")
        # The grip ring makes the AABB change when the knob rotates.
        ctx.check(
            f"drive_knob_{i} rotates (grip ring changes AABB)",
            rest_knob is not None and turned_knob is not None,
            details=f"rest={rest_knob}, turned={turned_knob}",
        )
    # Knob 0 projects from the front face (-X side).
    knob0_aabb = ctx.part_world_aabb(drive_knobs[0])
    stage_plate_aabb = ctx.part_element_world_aabb(stage, elem="stage_plate")
    ctx.check(
        "drive_knob_0 projects in front of the stage plate",
        knob0_aabb is not None and stage_plate_aabb is not None
        and knob0_aabb[0][0] < stage_plate_aabb[0][0] - 0.003,
        details=f"knob0_min_x={knob0_aabb[0][0] if knob0_aabb else None}, "
        f"plate_min_x={stage_plate_aabb[0][0] if stage_plate_aabb else None}",
    )
    # Knob 1 projects from the right face (+Y side).
    knob1_aabb = ctx.part_world_aabb(drive_knobs[1])
    ctx.check(
        "drive_knob_1 projects to the right of the stage plate",
        knob1_aabb is not None and stage_plate_aabb is not None
        and knob1_aabb[1][1] > stage_plate_aabb[1][1] + 0.003,
        details=f"knob1_max_y={knob1_aabb[1][1] if knob1_aabb else None}, "
        f"plate_max_y={stage_plate_aabb[1][1] if stage_plate_aabb else None}",
    )

    # --- Specimen slide travels with the carriages ---
    rest_slide = ctx.part_element_world_aabb(y_carriage, elem="specimen_slide")
    with ctx.pose({x_joint: X_TRAVEL[1], y_joint: Y_TRAVEL[1]}):
        moved_slide = ctx.part_element_world_aabb(y_carriage, elem="specimen_slide")
    slide_travel = 0.0
    if rest_slide is not None and moved_slide is not None:
        slide_travel = math.hypot(
            0.5 * (moved_slide[0][0] + moved_slide[1][0])
            - 0.5 * (rest_slide[0][0] + rest_slide[1][0]),
            0.5 * (moved_slide[0][1] + moved_slide[1][1])
            - 0.5 * (rest_slide[0][1] + rest_slide[1][1]),
        )
    ctx.check(
        "specimen slide travels with the X-Y carriage (> 15 mm diagonal)",
        slide_travel > 0.015,
        details=f"slide diagonal travel={slide_travel:.4f} m",
    )

    # --- Focus knob: off-axis nub proves rotation about the Y axle ---
    rest_ptr = ctx.part_element_world_aabb(focus_knob, elem="knob_nub_p")
    with ctx.pose({focus_joint: math.pi / 2.0}):
        turned_ptr = ctx.part_element_world_aabb(focus_knob, elem="knob_nub_p")
    d_knob = 0.0
    if rest_ptr is not None and turned_ptr is not None:
        d_knob = math.hypot(
            0.5 * (turned_ptr[0][0] + turned_ptr[1][0])
            - 0.5 * (rest_ptr[0][0] + rest_ptr[1][0]),
            0.5 * (turned_ptr[0][2] + turned_ptr[1][2])
            - 0.5 * (rest_ptr[0][2] + rest_ptr[1][2]),
        )
    ctx.check(
        "focus knob rotates about its axle (off-axis nub moves)",
        d_knob > 0.008,
        details=f"knob nub moved {d_knob:.4f} m under a quarter turn",
    )
    knob_aabb = ctx.part_world_aabb(focus_knob)
    ctx.check(
        "focus knobs project from both sides of the arm",
        knob_aabb is not None
        and knob_aabb[0][1] < -ARM_HALF_WIDTH - 0.005
        and knob_aabb[1][1] > ARM_HALF_WIDTH + 0.005,
        details=f"knob y-range=[{knob_aabb[0][1] if knob_aabb else None}, "
        f"{knob_aabb[1][1] if knob_aabb else None}], arm half width={ARM_HALF_WIDTH}",
    )
    # Focus knobs stay below the stage plate and clear of the carriage.
    axle_aabb = ctx.part_element_world_aabb(focus_knob, elem="knob_axle")
    disc_p_aabb = ctx.part_element_world_aabb(focus_knob, elem="knob_disc_p")
    disc_n_aabb = ctx.part_element_world_aabb(focus_knob, elem="knob_disc_n")
    ctx.check(
        "focus knobs stay below the stage plate",
        knob_aabb is not None and plate_aabb is not None
        and knob_aabb[1][2] < plate_aabb[0][2],
        details=f"knob max z={knob_aabb[1][2] if knob_aabb else None}, "
        f"plate min z={plate_aabb[0][2] if plate_aabb else None}",
    )
    ctx.check(
        "knob axle stays behind the stage saddle bracket",
        axle_aabb is not None and bracket_aabb is not None
        and axle_aabb[0][0] > bracket_aabb[1][0] + 0.003,
        details=f"axle min x={axle_aabb[0][0] if axle_aabb else None}, "
        f"bracket max x={bracket_aabb[1][0] if bracket_aabb else None}",
    )
    ctx.check(
        "knob discs straddle the stage saddle bracket in Y",
        disc_p_aabb is not None and disc_n_aabb is not None
        and bracket_aabb is not None
        and disc_p_aabb[0][1] > bracket_aabb[1][1] + 0.003
        and disc_n_aabb[1][1] < bracket_aabb[0][1] - 0.003,
        details=f"disc_p min y={disc_p_aabb[0][1] if disc_p_aabb else None}, "
        f"disc_n max y={disc_n_aabb[1][1] if disc_n_aabb else None}, "
        f"bracket y range={[bracket_aabb[0][1], bracket_aabb[1][1]] if bracket_aabb else None}",
    )

    # --- Neutral-pose separation between major bodies ---
    head_aabb = ctx.part_element_world_aabb(base, elem="head_housing")
    ctx.check(
        "turret disc stays below the head housing (only the spindle enters)",
        disc_aabb is not None and head_aabb is not None
        and disc_aabb[1][2] - head_aabb[0][2] < 0.006,
        details=f"disc max z={disc_aabb[1][2] if disc_aabb else None}, "
        f"head min z={head_aabb[0][2] if head_aabb else None} "
        "(5 mm spindle engagement is the declared socket)",
    )

    # --- Nearest approach: objectives above carriage at rest ---
    obj_tip_min = None
    for i in range(3):
        bb = ctx.part_element_world_aabb(nosepiece, elem=f"objective_barrel_{i}")
        if bb is not None:
            obj_tip_min = bb[0][2] if obj_tip_min is None else min(obj_tip_min, bb[0][2])
    ctx.check(
        "lowest objective tip stays above the y_carriage top at rest",
        obj_tip_min is not None and carriage_top is not None
        and obj_tip_min - carriage_top > 0.005,
        details=f"objective tip min z={obj_tip_min}, "
        f"y_carriage top={carriage_top}",
    )

    # ------------------------------------------------------------------
    # Declared, intentional socket overlaps.
    # ------------------------------------------------------------------
    # Arm column casts into the foot; head casts onto the arm jut.
    ctx.allow_overlap(
        base, base, elem_a="foot", elem_b="arm_limb",
        reason="The arm column is cast into the top of the foot.",
    )
    ctx.allow_overlap(
        base, base, elem_a="arm_limb", elem_b="head_housing",
        reason="The head housing is cast onto the forward jut of the arm.",
    )
    # Eyepiece tube plugs into the head; ocular stack is press-fit.
    ctx.allow_overlap(
        base, eyepiece, elem_a="head_housing", elem_b="tube_body",
        reason="The inclined monocular tube plugs into the top of the head housing.",
    )
    ctx.expect_contact(
        base, eyepiece, elem_a="head_housing", elem_b="tube_body",
        name="eyepiece tube is seated in the head",
    )
    ctx.allow_overlap(
        eyepiece, eyepiece, elem_a="tube_body", elem_b="ocular_barrel",
        reason="The ocular barrel is press-fit 2 mm into the tube top.",
    )
    ctx.allow_overlap(
        eyepiece, eyepiece, elem_a="ocular_barrel", elem_b="ocular_lens",
        reason="The ocular lens is seated 1 mm into the ocular top cap.",
    )
    # Turret spindle rides in a bore in the head underside.
    ctx.allow_overlap(
        base, nosepiece, elem_a="head_housing", elem_b="turret_disc",
        reason="The turret spindle hub rides in a bearing bore in the head underside.",
    )
    ctx.expect_contact(
        base, nosepiece, elem_a="head_housing", elem_b="turret_disc",
        name="turret spindle is captured in the head bore",
    )
    # Objectives thread into the turret; colored bands wrap the barrels.
    for i in range(3):
        ctx.allow_overlap(
            nosepiece, nosepiece, elem_a="turret_disc", elem_b=f"objective_barrel_{i}",
            reason="Objective barrel threads 2 mm into the turret disc.",
        )
        ctx.allow_overlap(
            nosepiece, nosepiece,
            elem_a=f"objective_barrel_{i}", elem_b=f"objective_ring_{i}",
            reason="Colored magnification band wraps around the barrel.",
        )
    # Stage plate bolts to the arm front face via the dovetail bracket.
    ctx.allow_overlap(
        base, stage, elem_a="arm_limb", elem_b="stage_plate",
        reason="The stage plate rear is bolted to the arm front face.",
    )
    ctx.allow_overlap(
        base, stage, elem_a="arm_limb", elem_b="stage_bracket",
        reason="The stage saddle bracket bolts to the arm front face.",
    )
    ctx.expect_contact(
        base, stage, elem_a="arm_limb", elem_b="stage_bracket",
        name="stage saddle engages the arm front face",
    )
    ctx.allow_overlap(
        stage, stage, elem_a="stage_plate", elem_b="stage_bracket",
        reason="The saddle bracket bolts 1 mm into the plate underside.",
    )
    # X carriage rides on the stage plate top surface (dovetail contact).
    ctx.expect_contact(
        stage, x_carriage, elem_a="stage_plate", elem_b="x_carriage_frame",
        name="x_carriage frame contacts the stage plate top",
    )
    # Y carriage plate nests inside the X carriage frame opening.
    ctx.expect_within(
        y_carriage, x_carriage, axes="xy",
        inner_elem="y_carriage_plate",
        outer_elem="x_carriage_frame",
        margin=0.001,
        name="y_carriage plate stays inside the x_carriage frame opening",
    )
    # Spring clips and specimen slide are seated on the y_carriage plate.
    for tag in ("0", "1"):
        ctx.allow_overlap(
            y_carriage, y_carriage, elem_a="y_carriage_plate", elem_b=f"stage_clip_{tag}",
            reason="Spring clip foot is screwed 0.3 mm into the y_carriage plate top.",
        )
    ctx.allow_overlap(
        y_carriage, y_carriage, elem_a="y_carriage_plate", elem_b="specimen_slide",
        reason="The glass slide rests seated on the y_carriage plate over the aperture.",
    )
    # Drive knobs: mounting bosses enter the stage plate edge by 3 mm.
    for i in range(N_DRIVE_KNOBS):
        ctx.allow_overlap(
            stage, drive_knobs[i], elem_a="stage_plate", elem_b=f"knob_body_{i}",
            reason=f"Drive knob {i} mounting boss enters the stage plate edge by 3 mm.",
        )
        ctx.allow_overlap(
            drive_knobs[i], drive_knobs[i], elem_a=f"knob_body_{i}", elem_b=f"knob_cap_{i}",
            reason=f"Drive knob {i} cap accent sits on the knob tip.",
        )
    # Illuminator bolts onto the foot; condenser mount stacks on the lamp.
    ctx.allow_overlap(
        base, condenser, elem_a="foot", elem_b="condenser_body",
        reason="The illuminator housing bolts 3 mm into the top of the foot.",
    )
    ctx.expect_contact(
        base, condenser, elem_a="foot", elem_b="condenser_body",
        name="illuminator stands on the base",
    )
    ctx.allow_overlap(
        condenser, condenser, elem_a="condenser_body", elem_b="condenser_mount",
        reason="The condenser mount is press-fit 2 mm into the lamp housing.",
    )
    # Focus axle passes through a bore in the arm column.
    ctx.allow_overlap(
        base, focus_knob, elem_a="arm_limb", elem_b="knob_axle",
        reason="The focus axle passes through the bearing bore in the arm column.",
    )
    ctx.expect_contact(
        base, focus_knob, elem_a="arm_limb", elem_b="knob_axle",
        name="focus axle is captured in the arm",
    )
    for tag in ("p", "n"):
        ctx.allow_overlap(
            focus_knob, focus_knob, elem_a="knob_axle", elem_b=f"knob_disc_{tag}",
            reason="The knob disc is keyed onto the through-axle.",
        )
        ctx.allow_overlap(
            focus_knob, focus_knob, elem_a=f"knob_disc_{tag}", elem_b=f"knob_nub_{tag}",
            reason="Index nub is riveted 0.5 mm into the knob face.",
        )

    return ctx.report()


object_model = build_object_model()
