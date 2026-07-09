from __future__ import annotations

# Compound monocular student microscope (mirror-illumination variant).
#
# Real object: a white-painted metal microscope with an inclined monocular
# eyepiece tube, a cylindrical head, a ROTATING NOSEPIECE TURRET carrying
# three objective lenses, a flat black mechanical stage, an angled arm/column,
# coaxial FOCUS KNOBS, and a round TILTING REFLECTOR MIRROR on a swinging
# trunnion fork below the stage (instead of a sub-stage condenser lamp).
#
# Structure (Z-up, optical axis = vertical line at X=0, arm behind at +X):
#   - The arm is an extruded XZ profile whose front face stays at X >= ARM_FRONT_X
#     below the head, so the turret disc (r=0.040) and the swept objectives
#     (r=0.037) clear it at EVERY rotation angle.
#   - Every attachment is a declared positive-overlap socket (arm->foot,
#     head->arm, tube->head, turret spindle->head bore, stage dovetail->arm,
#     fork stem->foot, knob axle->arm bore). No accidental slab crossings.
#
# Mechanisms:
#   - nosepiece turret: CONTINUOUS about the vertical optical axis.
#   - focus knobs: one through-axle with a knob disc + off-axis nub on EACH
#     side of the arm, CONTINUOUS about the horizontal Y axle.
#   - stage focus travel: small PRISMATIC vertical travel of the stage.
#   - mirror tilt: REVOLUTE about horizontal Y axis on the trunnion fork.

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
STAGE_TRAVEL = (-0.010, 0.006)  # prismatic focus travel (lower, upper)
KNOB_Z = 0.105               # focus axle height (world)
KNOB_X = 0.078               # focus axle X position (inside the arm column)
HEAD_R = 0.034               # head housing radius
HEAD_Z0, HEAD_Z1 = 0.245, 0.298  # head housing bottom/top

# Mirror fork + tilting mirror layout
FORK_MOUNT_Z = 0.035         # fork part origin height (world), at foot top
FORK_GAP = 0.046             # clear gap between fork cheeks (along Y)
CHEEK_T = 0.004              # cheek thickness (along Y)
FORK_WIDTH_Y = FORK_GAP + 2 * CHEEK_T  # total fork width
FORK_DEPTH_X = 0.012         # fork depth (front-to-back, along X)
FORK_BASE_T = 0.008          # base plate thickness
FORK_CHEEK_H = 0.052         # cheek height above base plate
FORK_TRUNNION_Z_LOCAL = 0.042  # trunnion center height from fork base (local)
FORK_TRUNNION_D = 0.006      # trunnion bore diameter
MIRROR_R = 0.020             # mirror disc radius
MIRROR_THICK = 0.003         # mirror disc thickness
MIRROR_TILT = (-0.5, 0.5)    # mirror revolute limits (radians)
STUB_R = 0.0025              # trunnion stub radius
STUB_EMBED = 0.002           # stub embed depth into disc edge
WORLD_TRUNNION_Z = FORK_MOUNT_Z + FORK_TRUNNION_Z_LOCAL  # world trunnion height


def _trunnion_stub(sign: int, disc_r: float, stub_r: float, stub_len: float):
    """Shared helper: horizontal trunnion stub cylinder along Y.

    The stub is embedded 2 mm into the disc edge and extends outward
    toward the fork cheek bore.
    """
    embed = 0.002
    y_inner = disc_r - embed
    if sign > 0:
        y_start = y_inner
    else:
        y_start = -(y_inner + stub_len)
    return (
        cq.Workplane("XZ")
        .workplane(offset=y_start)
        .center(0, 0)
        .circle(stub_r)
        .extrude(stub_len)
    )


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

    # Cylindrical head housing on the optical axis, carried by the arm jut.
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
    # MONOCULAR TUBE + OCULAR: fixed to the head, inclined 40 deg toward
    # the front (-X, away from the arm), as in the photo.
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
    # STAGE: black mechanical stage on a small PRISMATIC vertical focus
    # travel. The plate rear and dovetail bracket slide inside the arm
    # front face (declared sliding socket).
    # ------------------------------------------------------------------
    stage = model.part("stage")
    stage_plate = (
        cq.Workplane("XY")
        .center(-0.0075, 0.0)
        .rect(0.135, 0.120)
        .extrude(0.008)
        .edges("|Z")
        .fillet(0.006)
    )
    stage_plate = stage_plate.translate((0.0, 0.0, -0.004))
    stage_plate = (
        stage_plate.faces(">Z")
        .workplane(origin=(0.0, 0.0, 0.0))
        .circle(0.011)
        .cutThruAll()
    )
    stage.visual(
        mesh_from_cadquery(stage_plate, "stage_plate"),
        material=black_plastic,
        name="stage_plate",
    )
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
    for sy, tag in ((-0.020, "n"), (0.020, "p")):
        clip = (
            cq.Workplane("XY")
            .center(0.022, sy)
            .rect(0.030, 0.004)
            .extrude(0.0015)
        )
        clip = clip.translate((0.0, 0.0, 0.0037))
        stage.visual(
            mesh_from_cadquery(clip, f"stage_clip_{tag}"),
            material=chrome,
            name=f"stage_clip_{tag}",
        )
    stage.visual(
        Box((0.064, 0.024, 0.0012)),
        origin=Origin(xyz=(0.0, 0.0, 0.0044)),
        material=slide_glass,
        name="specimen_slide",
    )
    stage.inertial = Inertial.from_geometry(
        Box((0.135, 0.120, 0.030)),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
    )
    model.articulation(
        "arm_to_stage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stage,
        origin=Origin(xyz=(0.0, 0.0, STAGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=0.05,
            lower=STAGE_TRAVEL[0],
            upper=STAGE_TRAVEL[1],
        ),
    )

    # ------------------------------------------------------------------
    # MIRROR FORK: trunnion yoke bracket standing on the base foot below
    # the stage, carrying the tilting reflector mirror (fixed mount).
    # Built with manual CadQuery for correct Y-axis cheek orientation.
    # ------------------------------------------------------------------
    mirror_fork = model.part("mirror_fork")

    # Base plate: thin rectangular plate at the fork bottom.
    base_plate = (
        cq.Workplane("XY")
        .center(0.0, 0.0)
        .rect(FORK_DEPTH_X, FORK_WIDTH_Y)
        .extrude(FORK_BASE_T)
    )
    mirror_fork.visual(
        mesh_from_cadquery(base_plate, "fork_base"),
        material=dark_metal,
        name="fork_base",
    )

    # Two cheek plates: vertical plates on the Y sides with trunnion bores.
    cheek_y = FORK_GAP / 2.0 + CHEEK_T / 2.0
    for sign, tag in ((-1, "n"), (1, "p")):
        cheek = (
            cq.Workplane("XY")
            .workplane(offset=FORK_BASE_T)
            .center(0.0, sign * cheek_y)
            .rect(FORK_DEPTH_X, CHEEK_T)
            .extrude(FORK_CHEEK_H)
        )
        # Cut trunnion bore through the cheek (cylinder along Y).
        bore = (
            cq.Workplane("XZ")
            .workplane(offset=sign * (cheek_y - CHEEK_T / 2.0))
            .center(0.0, FORK_TRUNNION_Z_LOCAL)
            .circle(FORK_TRUNNION_D / 2.0)
            .extrude(CHEEK_T)
        )
        cheek = cheek.cut(bore)
        mirror_fork.visual(
            mesh_from_cadquery(cheek, f"fork_cheek_{tag}"),
            material=dark_metal,
            name=f"fork_cheek_{tag}",
        )

    # Cylindrical stem: embedded 3 mm into the foot top, extends into the
    # fork base plate for connectivity.
    fork_stem = (
        cq.Workplane("XY")
        .workplane(offset=-0.003)
        .circle(0.006)
        .extrude(FORK_BASE_T + 0.003)
    )
    mirror_fork.visual(
        mesh_from_cadquery(fork_stem, "fork_stem"),
        material=dark_metal,
        name="fork_stem",
    )
    mirror_fork.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.068),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )
    model.articulation(
        "base_to_mirror_fork",
        ArticulationType.FIXED,
        parent=base,
        child=mirror_fork,
        origin=Origin(xyz=(0.0, 0.0, FORK_MOUNT_Z)),
    )

    # ------------------------------------------------------------------
    # TILTING MIRROR: round reflector disc on a REVOLUTE joint about the
    # horizontal Y axis. Trunnion stubs extend from the disc edge into
    # the fork cheek bores.
    # ------------------------------------------------------------------
    mirror = model.part("mirror")

    # Mirror face: thin disc centered at origin, axis along Z.
    # At q=0 the face is horizontal (reflecting upward toward the stage).
    mirror_face = (
        cq.Workplane("XY")
        .workplane(offset=-MIRROR_THICK / 2)
        .circle(MIRROR_R)
        .extrude(MIRROR_THICK)
    )
    mirror.visual(
        mesh_from_cadquery(mirror_face, "mirror_face"),
        material=chrome,
        name="mirror_face",
    )

    # Bezel rim: annular ring encircling the disc edge.
    rim_outer = (
        cq.Workplane("XY")
        .workplane(offset=-MIRROR_THICK / 2 - 0.0005)
        .circle(MIRROR_R + 0.001)
        .extrude(MIRROR_THICK + 0.001)
    )
    rim_hole = (
        cq.Workplane("XY")
        .workplane(offset=-MIRROR_THICK / 2 - 0.001)
        .circle(MIRROR_R - 0.001)
        .extrude(MIRROR_THICK + 0.002)
    )
    rim = rim_outer.cut(rim_hole)
    mirror.visual(
        mesh_from_cadquery(rim, "mirror_rim"),
        material=dark_metal,
        name="mirror_rim",
    )

    # Trunnion stubs: horizontal pins extending from disc edge into fork
    # cheek bores (repeated sub-parts via for-i-in-range with helper).
    stub_len = FORK_GAP / 2.0 - MIRROR_R + STUB_EMBED + 0.002
    for i in range(2):
        sign = -1 if i == 0 else 1
        stub = _trunnion_stub(sign, MIRROR_R, STUB_R, stub_len)
        mirror.visual(
            mesh_from_cadquery(stub, f"mirror_stub_{i}"),
            material=dark_metal,
            name=f"mirror_stub_{i}",
        )
    mirror.inertial = Inertial.from_geometry(
        Cylinder(radius=MIRROR_R, length=MIRROR_THICK),
        mass=0.04,
    )
    model.articulation(
        "fork_to_mirror",
        ArticulationType.REVOLUTE,
        parent=mirror_fork,
        child=mirror,
        origin=Origin(xyz=(0.0, 0.0, FORK_TRUNNION_Z_LOCAL)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=MIRROR_TILT[0],
            upper=MIRROR_TILT[1],
        ),
    )

    # ------------------------------------------------------------------
    # FOCUS KNOBS: one through-axle (cylinder through the arm) carrying a
    # coaxial knob disc on EACH side of the arm, CONTINUOUS about world Y.
    # Local +Z maps onto world +Y via the joint rpy.
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
    mirror_fork = object_model.get_part("mirror_fork")
    mirror = object_model.get_part("mirror")
    focus_knob = object_model.get_part("focus_knob")

    turret_joint = object_model.get_articulation("head_to_nosepiece")
    stage_joint = object_model.get_articulation("arm_to_stage")
    focus_joint = object_model.get_articulation("arm_to_focus_knob")
    mirror_joint = object_model.get_articulation("fork_to_mirror")

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
        "stage focus travel is a bounded vertical prismatic",
        stage_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(stage_joint.axis) == (0.0, 0.0, 1.0)
        and stage_joint.motion_limits is not None
        and stage_joint.motion_limits.lower == STAGE_TRAVEL[0]
        and stage_joint.motion_limits.upper == STAGE_TRAVEL[1],
        details=f"type={stage_joint.articulation_type}, limits={stage_joint.motion_limits}",
    )
    ctx.check(
        "mirror tilt is a bounded revolute about horizontal Y",
        mirror_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(mirror_joint.axis) == (0.0, 1.0, 0.0)
        and mirror_joint.motion_limits is not None
        and mirror_joint.motion_limits.lower == MIRROR_TILT[0]
        and mirror_joint.motion_limits.upper == MIRROR_TILT[1],
        details=f"type={mirror_joint.articulation_type}, axis={mirror_joint.axis}, "
        f"limits={mirror_joint.motion_limits}",
    )

    # --- Rests on the ground: global z-min == 0 ---
    zmin = None
    for p in (base, eyepiece, nosepiece, stage, mirror_fork, mirror, focus_knob):
        aabb = ctx.part_world_aabb(p)
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

    # --- Mirror is a round disc ---
    mirror_face_vis = [
        v for v in mirror.visuals if v.name and v.name == "mirror_face"
    ]
    ctx.check(
        "mirror has a round reflector face",
        len(mirror_face_vis) == 1,
        details=f"mirror_face visuals: {len(mirror_face_vis)}",
    )

    # --- Vertical optical stack ordering ---
    eye_aabb = ctx.part_world_aabb(eyepiece)
    nose_aabb = ctx.part_world_aabb(nosepiece)
    stage_aabb = ctx.part_world_aabb(stage)
    fork_aabb = ctx.part_world_aabb(mirror_fork)
    mirror_aabb = ctx.part_world_aabb(mirror)
    disc_aabb = ctx.part_element_world_aabb(nosepiece, elem="turret_disc")
    ctx.check(
        "eyepiece tube sits above the turret",
        eye_aabb is not None and nose_aabb is not None and eye_aabb[0][2] > nose_aabb[1][2],
        details=f"eye_min_z={eye_aabb[0][2] if eye_aabb else None}, "
        f"nose_max_z={nose_aabb[1][2] if nose_aabb else None}",
    )
    ctx.check(
        "turret disc hangs above the stage",
        disc_aabb is not None and stage_aabb is not None and disc_aabb[0][2] > stage_aabb[1][2],
        details=f"disc_min_z={disc_aabb[0][2] if disc_aabb else None}, "
        f"stage_max_z={stage_aabb[1][2] if stage_aabb else None}",
    )
    plate_aabb = ctx.part_element_world_aabb(stage, elem="stage_plate")
    ctx.check(
        "mirror sits below the stage plate",
        mirror_aabb is not None and plate_aabb is not None
        and mirror_aabb[1][2] < plate_aabb[0][2],
        details=f"mirror_max_z={mirror_aabb[1][2] if mirror_aabb else None}, "
        f"plate_min_z={plate_aabb[0][2] if plate_aabb else None}",
    )
    ctx.check(
        "mirror fork stands below the mirror disc",
        fork_aabb is not None and mirror_aabb is not None
        and fork_aabb[0][2] < mirror_aabb[0][2],
        details=f"fork_min_z={fork_aabb[0][2] if fork_aabb else None}, "
        f"mirror_min_z={mirror_aabb[0][2] if mirror_aabb else None}",
    )
    ctx.expect_within(
        nosepiece, stage, axes="xy", margin=0.005,
        name="turret stays over the stage footprint",
    )
    ctx.expect_within(
        mirror, stage, axes="xy", margin=0.010,
        name="mirror stays within the stage footprint on XY",
    )

    # --- FK sweep: turret objectives clear arm/head/stage ---
    stage_top_near = None
    with ctx.pose({stage_joint: STAGE_TRAVEL[1]}):
        near_aabb = ctx.part_world_aabb(stage)
        if near_aabb is not None:
            stage_top_near = near_aabb[1][2]
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
                if stage_top_near is None or bb[0][2] <= stage_top_near + 0.003:
                    sweep_ok = False
                    sweep_details.append(
                        f"barrel {i} @ {ang:.2f} too close to stage: "
                        f"min_z={bb[0][2]:.4f} vs stage_top={stage_top_near}"
                    )
    ctx.check(
        "objectives clear arm/head/stage through a full turret sweep (12 angles)",
        sweep_ok,
        details="; ".join(sweep_details) if sweep_details else
        f"all clear; stage_top_near={stage_top_near:.4f}",
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

    # --- Decisive pose: turret quarter turn swings objective ---
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

    # --- Focus travel: stage moves vertically ---
    rest_stage = ctx.part_world_aabb(stage)
    with ctx.pose({stage_joint: STAGE_TRAVEL[0]}):
        low_stage = ctx.part_world_aabb(stage)
    dz = None
    if rest_stage is not None and low_stage is not None:
        dz = rest_stage[1][2] - low_stage[1][2]
    ctx.check(
        "prismatic focus joint translates the stage",
        dz is not None and abs(dz - (-STAGE_TRAVEL[0])) < 1.0e-4,
        details=f"stage top moved {dz} m for lower limit {STAGE_TRAVEL[0]}",
    )
    obj_tip_min = None
    for i in range(3):
        bb = ctx.part_element_world_aabb(nosepiece, elem=f"objective_barrel_{i}")
        if bb is not None:
            obj_tip_min = bb[0][2] if obj_tip_min is None else min(obj_tip_min, bb[0][2])
    ctx.check(
        "stage at top of travel stays below the lowest objective tip",
        stage_top_near is not None
        and obj_tip_min is not None
        and obj_tip_min - stage_top_near > 0.005,
        details=f"objective tip min z={obj_tip_min}, stage top (near limit)={stage_top_near}",
    )
    with ctx.pose({stage_joint: STAGE_TRAVEL[0]}):
        low_plate = ctx.part_element_world_aabb(stage, elem="stage_plate")
        low_bracket = ctx.part_element_world_aabb(stage, elem="stage_bracket")
    ctx.check(
        "stage plate at bottom of travel stays above the mirror",
        low_plate is not None
        and mirror_aabb is not None
        and low_plate[0][2] > mirror_aabb[1][2],
        details=f"plate min z (low limit)={low_plate[0][2] if low_plate else None}, "
        f"mirror max z={mirror_aabb[1][2] if mirror_aabb else None}",
    )

    # --- Mirror tilt: decisive pose check ---
    rest_mirror_face = ctx.part_element_world_aabb(mirror, elem="mirror_face")
    with ctx.pose({mirror_joint: MIRROR_TILT[1]}):
        tilted_mirror_face = ctx.part_element_world_aabb(mirror, elem="mirror_face")
    rest_z_ext = 0.0
    tilt_z_ext = 0.0
    if rest_mirror_face is not None:
        rest_z_ext = rest_mirror_face[1][2] - rest_mirror_face[0][2]
    if tilted_mirror_face is not None:
        tilt_z_ext = tilted_mirror_face[1][2] - tilted_mirror_face[0][2]
    ctx.check(
        "tilting the mirror increases its vertical extent (disc swings)",
        tilt_z_ext - rest_z_ext > 0.005,
        details=f"rest z-extent={rest_z_ext:.4f}, tilted z-extent={tilt_z_ext:.4f}",
    )
    rest_x_ext = 0.0
    tilt_x_ext = 0.0
    if rest_mirror_face is not None:
        rest_x_ext = rest_mirror_face[1][0] - rest_mirror_face[0][0]
    if tilted_mirror_face is not None:
        tilt_x_ext = tilted_mirror_face[1][0] - tilted_mirror_face[0][0]
    ctx.check(
        "tilting the mirror shrinks its X footprint (edge tilts away)",
        rest_x_ext - tilt_x_ext > 0.001,
        details=f"rest x-extent={rest_x_ext:.4f}, tilted x-extent={tilt_x_ext:.4f}",
    )

    # --- Focus knob checks ---
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
    axle_aabb = ctx.part_element_world_aabb(focus_knob, elem="knob_axle")
    disc_p_aabb = ctx.part_element_world_aabb(focus_knob, elem="knob_disc_p")
    disc_n_aabb = ctx.part_element_world_aabb(focus_knob, elem="knob_disc_n")
    ctx.check(
        "focus knobs stay below the stage plate at its lowest travel",
        knob_aabb is not None
        and low_plate is not None
        and knob_aabb[1][2] < low_plate[0][2],
        details=f"knob max z={knob_aabb[1][2] if knob_aabb else None}, "
        f"plate min z (low)={low_plate[0][2] if low_plate else None}",
    )
    ctx.check(
        "knob axle stays behind the stage saddle bracket",
        axle_aabb is not None
        and low_bracket is not None
        and axle_aabb[0][0] > low_bracket[1][0] + 0.003,
        details=f"axle min x={axle_aabb[0][0] if axle_aabb else None}, "
        f"bracket max x={low_bracket[1][0] if low_bracket else None}",
    )
    ctx.check(
        "knob discs straddle the stage saddle bracket in Y",
        disc_p_aabb is not None
        and disc_n_aabb is not None
        and low_bracket is not None
        and disc_p_aabb[0][1] > low_bracket[1][1] + 0.003
        and disc_n_aabb[1][1] < low_bracket[0][1] - 0.003,
        details=f"disc_p min y={disc_p_aabb[0][1] if disc_p_aabb else None}, "
        f"disc_n max y={disc_n_aabb[1][1] if disc_n_aabb else None}, "
        f"bracket y range={[low_bracket[0][1], low_bracket[1][1]] if low_bracket else None}",
    )

    # --- Neutral-pose separation ---
    head_aabb = ctx.part_element_world_aabb(base, elem="head_housing")
    ctx.check(
        "turret disc stays below the head housing (only the spindle enters)",
        disc_aabb is not None and head_aabb is not None
        and disc_aabb[1][2] - head_aabb[0][2] < 0.006,
        details=f"disc max z={disc_aabb[1][2] if disc_aabb else None}, "
        f"head min z={head_aabb[0][2] if head_aabb else None} "
        "(5 mm spindle engagement is the declared socket)",
    )

    # ------------------------------------------------------------------
    # Declared intentional overlaps
    # ------------------------------------------------------------------
    # Arm/base/head casting
    ctx.allow_overlap(
        base, base, elem_a="foot", elem_b="arm_limb",
        reason="The arm column is cast into the top of the foot.",
    )
    ctx.allow_overlap(
        base, base, elem_a="arm_limb", elem_b="head_housing",
        reason="The head housing is cast onto the forward jut of the arm.",
    )
    # Eyepiece
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
    # Turret
    ctx.allow_overlap(
        base, nosepiece, elem_a="head_housing", elem_b="turret_disc",
        reason="The turret spindle hub rides in a bearing bore in the head underside.",
    )
    ctx.expect_contact(
        base, nosepiece, elem_a="head_housing", elem_b="turret_disc",
        name="turret spindle is captured in the head bore",
    )
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
    # Stage
    ctx.allow_overlap(
        base, stage, elem_a="arm_limb", elem_b="stage_plate",
        reason="The stage plate rear slides in the dovetail ways on the arm front.",
    )
    ctx.allow_overlap(
        base, stage, elem_a="arm_limb", elem_b="stage_bracket",
        reason="The stage saddle bracket rides in the dovetail ways on the arm front.",
    )
    ctx.expect_contact(
        base, stage, elem_a="arm_limb", elem_b="stage_bracket",
        name="stage saddle engages the arm dovetail",
    )
    ctx.allow_overlap(
        stage, stage, elem_a="stage_plate", elem_b="stage_bracket",
        reason="The saddle bracket bolts 1 mm into the plate underside.",
    )
    ctx.allow_overlap(
        stage, stage, elem_a="stage_plate", elem_b="stage_clip_p",
        reason="Spring clip foot is screwed into the stage top plate.",
    )
    ctx.allow_overlap(
        stage, stage, elem_a="stage_plate", elem_b="stage_clip_n",
        reason="Spring clip foot is screwed into the stage top plate.",
    )
    ctx.allow_overlap(
        stage, stage, elem_a="stage_plate", elem_b="specimen_slide",
        reason="The glass slide rests seated on the stage plate over the aperture.",
    )
    # Mirror fork: stem embeds into foot; stem overlaps with fork base.
    ctx.allow_overlap(
        base, mirror_fork, elem_a="foot", elem_b="fork_stem",
        reason="The mirror fork stem plugs 3 mm into the top of the foot.",
    )
    ctx.expect_contact(
        base, mirror_fork, elem_a="foot", elem_b="fork_stem",
        name="mirror fork stem stands on the base foot",
    )
    ctx.allow_overlap(
        mirror_fork, mirror_fork, elem_a="fork_stem", elem_b="fork_base",
        reason="The fork stem extends into the fork base plate for structural continuity.",
    )
    ctx.allow_overlap(
        mirror_fork, mirror_fork, elem_a="fork_base", elem_b="fork_cheek_p",
        reason="The fork cheek is welded to the fork base plate.",
    )
    ctx.allow_overlap(
        mirror_fork, mirror_fork, elem_a="fork_base", elem_b="fork_cheek_n",
        reason="The fork cheek is welded to the fork base plate.",
    )
    # Mirror: rim encircles the disc edge; stubs embed into the disc.
    ctx.allow_overlap(
        mirror, mirror, elem_a="mirror_face", elem_b="mirror_rim",
        reason="The bezel rim encircles the mirror disc edge.",
    )
    for i in range(2):
        ctx.allow_overlap(
            mirror, mirror, elem_a="mirror_face", elem_b=f"mirror_stub_{i}",
            reason=f"Trunnion stub {i} is embedded 2 mm into the mirror disc edge.",
        )
    # Mirror stubs enter the fork cheek trunnion bores (pivot bearing).
    ctx.allow_overlap(
        mirror_fork, mirror, elem_a="fork_cheek_n", elem_b="mirror_stub_0",
        reason="Mirror trunnion stub 0 enters the fork cheek bore as a pivot bearing.",
    )
    ctx.allow_overlap(
        mirror_fork, mirror, elem_a="fork_cheek_p", elem_b="mirror_stub_1",
        reason="Mirror trunnion stub 1 enters the fork cheek bore as a pivot bearing.",
    )

    # Focus knob
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
