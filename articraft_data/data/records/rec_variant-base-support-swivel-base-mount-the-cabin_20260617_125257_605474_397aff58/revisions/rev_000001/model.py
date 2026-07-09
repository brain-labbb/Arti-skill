from __future__ import annotations

# Vintage 1980s Samsung CRT television on a round swivel base/pedestal.
#
# Articraft brief:
# - Object: tabletop CRT TV on a turntable pedestal, 0.62 m wide (Y), 0.45 m
#   tall cabinet (Z), 0.32 m deep (X), front face toward +X.
# - Root/support: round swivel base (dark plastic pedestal) sitting at z=0.
#   Cabinet rides on top via a CONTINUOUS vertical-axis turntable joint.
# - Cabinet: hollow wooden shell with recessed beige front plate, CRT bezel
#   and glass, control column with vent, dial escutcheon, grille, nameplate.
# - Articulations:
#   * base_to_cabinet: CONTINUOUS about +Z (vertical turntable).
#   * cabinet_to_channel_dial: CONTINUOUS about +X (normal to front face).
#   * cabinet_to_volume/brightness/tuning_knob: REVOLUTE about +X, +/-135 deg.
# - Intentional overlaps: knob/dial stems captured inside the front plate /
#   escutcheon (scoped allowances + retained-insertion checks).

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    BezelRecess,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---------------------------------------------------------------- dimensions
CAB_W = 0.62  # full width along Y
CAB_H = 0.45  # full height along Z
CAB_D = 0.32  # full depth along X
WOOD_T = 0.020

FRONT_X = CAB_D / 2.0  # +0.16, wood front edge plane
PLATE_X0 = 0.132  # rear face of the recessed beige front plate
PLATE_X1 = 0.144  # front face of the recessed beige front plate

SCREEN_CY = 0.0875  # bezel / CRT centre (Y)
SCREEN_CZ = 0.225  # bezel / CRT centre (Z)

COL_Y = -0.2025  # control-column centreline (Y)

DIAL_DIAMETER = 0.070
DIAL_CZ = 0.300
KNOB_DIAMETER = 0.024
KNOB_ROW_Z = 0.225
KNOB_YS = (-0.1525, -0.2025, -0.2525)
KNOB_RANGE = 0.75 * math.pi  # +/-135 degrees

DIAL_JOINT_X = 0.150  # escutcheon front face
KNOB_JOINT_X = PLATE_X1  # plate front face

# Swivel base dimensions
BASE_RADIUS = 0.150  # outer base disk radius
BASE_DISK_H = 0.012  # base disk thickness
COLUMN_RADIUS = 0.058  # central pedestal column radius
COLUMN_TOP_Z = 0.050  # top of the column before the flare
PLATE_RADIUS = 0.110  # top turntable plate radius
PLATE_H = 0.008  # top plate thickness
PEDESTAL_TOP_Z = COLUMN_TOP_Z + PLATE_H  # 0.058, where cabinet bottom sits


def _rounded_rect_outline(
    height: float,
    width: float,
    radius: float,
    *,
    segments: int = 5,
) -> list[tuple[float, float]]:
    """CCW rounded-rectangle outline in a local (x=height, y=width) plane."""
    hx = height / 2.0
    hy = width / 2.0
    r = min(radius, hx * 0.95, hy * 0.95)
    arcs = (
        ((hx - r, hy - r), 0.0, math.pi / 2.0),
        ((-(hx - r), hy - r), math.pi / 2.0, math.pi),
        ((-(hx - r), -(hy - r)), math.pi, 1.5 * math.pi),
        ((hx - r, -(hy - r)), 1.5 * math.pi, 2.0 * math.pi),
    )
    points: list[tuple[float, float]] = []
    for (cx, cy), a0, a1 in arcs:
        for index in range(segments + 1):
            t = index / segments
            angle = a0 + (a1 - a0) * t
            points.append((cx + r * math.cos(angle), cy + r * math.sin(angle)))
    return points


def _build_crt_glass_mesh():
    """Bulged rounded-rectangle CRT face lofted along local +Z (-> world +X)."""
    sections_spec = (
        (0.000, 0.300, 0.370, 0.050),  # rear flange behind the bezel opening
        (0.006, 0.290, 0.355, 0.050),
        (0.012, 0.255, 0.325, 0.050),
        (0.018, 0.210, 0.270, 0.060),  # convex front cap, recessed behind bezel lip
    )
    sections = []
    for z, h, w, r in sections_spec:
        outline = _rounded_rect_outline(h, w, r, segments=5)
        sections.append([(x, y, z) for x, y in outline])
    geom = LoftGeometry(sections, cap=True, closed=True)
    return mesh_from_geometry(geom, "crt_glass_tube")


def _build_swivel_base_mesh():
    """Lathed round pedestal: base disk, tapered column, top turntable plate."""
    profile = [
        (0.000, 0.000),       # centre bottom
        (BASE_RADIUS, 0.000), # outer base bottom
        (BASE_RADIUS, BASE_DISK_H),  # outer base top edge
        (COLUMN_RADIUS + 0.010, BASE_DISK_H + 0.004),  # fillet start
        (COLUMN_RADIUS, BASE_DISK_H + 0.010),  # column wall start
        (COLUMN_RADIUS, COLUMN_TOP_Z - 0.006),  # column wall end (taper)
        (COLUMN_RADIUS - 0.004, COLUMN_TOP_Z),  # slight inward taper at top
        (PLATE_RADIUS - 0.006, COLUMN_TOP_Z + 0.002),  # flare to plate
        (PLATE_RADIUS, COLUMN_TOP_Z + 0.004),  # plate outer edge
        (PLATE_RADIUS, PEDESTAL_TOP_Z),  # plate top outer
        (0.000, PEDESTAL_TOP_Z),  # centre top
    ]
    geom = LatheGeometry(profile, segments=48, closed=True)
    return mesh_from_geometry(geom, "swivel_pedestal")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_samsung_crt_television_swivel")

    wood = model.material("wood_veneer_tan", rgba=(0.72, 0.62, 0.47, 1.0))
    plastic = model.material("plastic_beige", rgba=(0.79, 0.75, 0.65, 1.0))
    bezel_plastic = model.material("bezel_beige_dark", rgba=(0.70, 0.66, 0.55, 1.0))
    glass = model.material("crt_glass_green_grey", rgba=(0.72, 0.77, 0.72, 1.0))
    dark_face = model.material("dial_face_dark", rgba=(0.15, 0.15, 0.16, 1.0))
    metal = model.material("metal_ring", rgba=(0.63, 0.62, 0.58, 1.0))
    white_mark = model.material("marking_white", rgba=(0.93, 0.93, 0.90, 1.0))
    grille_dark = model.material("grille_recess_dark", rgba=(0.34, 0.31, 0.26, 1.0))
    silver = model.material("nameplate_silver", rgba=(0.76, 0.76, 0.74, 1.0))
    base_plastic = model.material("base_dark_plastic", rgba=(0.18, 0.17, 0.16, 1.0))

    # ============================================================= swivel base
    swivel_base = model.part("swivel_base")
    swivel_base.visual(
        _build_swivel_base_mesh(),
        origin=Origin(),
        material=base_plastic,
        name="pedestal",
    )
    # Thin turntable ring on top showing the rotation bearing seam.
    swivel_base.visual(
        Cylinder(radius=PLATE_RADIUS - 0.003, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_TOP_Z + 0.001)),
        material=metal,
        name="turntable_ring",
    )

    # ============================================================= cabinet
    cabinet = model.part("cabinet")

    inner_h = CAB_H - 2.0 * WOOD_T  # 0.41
    inner_w = CAB_W - 2.0 * WOOD_T  # 0.58

    cabinet.visual(
        Box((CAB_D, CAB_W, WOOD_T)),
        origin=Origin(xyz=(0.0, 0.0, WOOD_T / 2.0)),
        material=wood,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((CAB_D, CAB_W, WOOD_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_H - WOOD_T / 2.0)),
        material=wood,
        name="top_panel",
    )
    for label, sign in (("left", 1.0), ("right", -1.0)):
        cabinet.visual(
            Box((CAB_D, WOOD_T, inner_h)),
            origin=Origin(xyz=(0.0, sign * (CAB_W / 2.0 - WOOD_T / 2.0), CAB_H / 2.0)),
            material=wood,
            name=f"{label}_side_panel",
        )
    cabinet.visual(
        Box((0.014, inner_w, inner_h)),
        origin=Origin(xyz=(-CAB_D / 2.0 + 0.007, 0.0, CAB_H / 2.0)),
        material=wood,
        name="back_panel",
    )

    # Recessed beige plastic front plate filling the cabinet opening.
    cabinet.visual(
        Box((PLATE_X1 - PLATE_X0, inner_w, inner_h)),
        origin=Origin(xyz=((PLATE_X0 + PLATE_X1) / 2.0, 0.0, CAB_H / 2.0)),
        material=plastic,
        name="front_plate",
    )

    # ------------------------------------------------- CRT bezel and glass
    bezel_geom = BezelGeometry(
        (0.26, 0.33),  # opening (height, width) in the rotated local frame
        (0.40, 0.40),
        0.014,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.045,
        outer_corner_radius=0.012,
        recess=BezelRecess(depth=0.004, inset=0.005),
        center=False,
    )
    cabinet.visual(
        mesh_from_geometry(bezel_geom, "crt_bezel"),
        origin=Origin(xyz=(PLATE_X1, SCREEN_CY, SCREEN_CZ), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bezel_plastic,
        name="screen_bezel",
    )
    cabinet.visual(
        _build_crt_glass_mesh(),
        origin=Origin(xyz=(0.138, SCREEN_CY, SCREEN_CZ), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="crt_screen",
    )

    # Vertical seam trim between screen area and control column.
    cabinet.visual(
        Box((0.005, 0.006, inner_h)),
        origin=Origin(xyz=(0.1445, -0.115, CAB_H / 2.0)),
        material=bezel_plastic,
        name="column_seam_trim",
    )

    # ------------------------------------------------- louvered vent (top)
    cabinet.visual(
        Box((0.005, 0.120, 0.054)),
        origin=Origin(xyz=(0.1415, COL_Y, 0.390)),
        material=grille_dark,
        name="vent_recess",
    )
    for index, slat_z in enumerate((0.373, 0.390, 0.407)):
        cabinet.visual(
            Box((0.010, 0.118, 0.011)),
            origin=Origin(xyz=(0.146, COL_Y, slat_z)),
            material=plastic,
            name=f"vent_louver_{index}",
        )

    # ------------------------------------------------- dial escutcheon
    cabinet.visual(
        Box((0.006, 0.115, 0.115)),
        origin=Origin(xyz=(0.147, COL_Y, DIAL_CZ)),
        material=plastic,
        name="dial_escutcheon",
    )

    # ------------------------------------------------- speaker grille
    cabinet.visual(
        Box((0.005, 0.150, 0.150)),
        origin=Origin(xyz=(0.1415, COL_Y, 0.1175)),
        material=grille_dark,
        name="grille_recess",
    )
    for index in range(12):
        slat_z = 0.052 + index * 0.0115
        cabinet.visual(
            Box((0.008, 0.146, 0.0055)),
            origin=Origin(xyz=(0.146, COL_Y, slat_z)),
            material=plastic,
            name=f"grille_slat_{index}",
        )

    cabinet.visual(
        Box((0.004, 0.075, 0.013)),
        origin=Origin(xyz=(0.1455, COL_Y, 0.031)),
        material=silver,
        name="samsung_nameplate",
    )

    # ------------------------------------------------- channel selector dial
    dial = model.part("channel_dial")
    dial.visual(
        Cylinder(radius=0.009, length=0.022),
        origin=Origin(xyz=(-0.0105, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_face,
        name="dial_stem",
    )
    dial_ring = KnobGeometry(
        DIAL_DIAMETER,
        0.018,
        body_style="cylindrical",
        grip=KnobGrip(style="ribbed", count=24, depth=0.0012, width=0.002),
        center=False,
    )
    dial.visual(
        mesh_from_geometry(dial_ring, "dial_ribbed_ring"),
        origin=Origin(xyz=(0.0005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="dial_ribbed_ring",
    )
    dial.visual(
        Cylinder(radius=0.027, length=0.004),
        origin=Origin(xyz=(0.0205, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_face,
        name="dial_face",
    )
    # White tick markings around the dark face.
    tick_radius = 0.0215
    for index in range(8):
        angle = index * math.pi / 4.0
        dial.visual(
            Box((0.0015, 0.002, 0.006)),
            origin=Origin(
                xyz=(0.023, -tick_radius * math.sin(angle), tick_radius * math.cos(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=white_mark,
            name=f"dial_tick_{index}",
        )
    # Raised off-axis pointer proving continuous rotation.
    dial.visual(
        Box((0.004, 0.005, 0.013)),
        origin=Origin(xyz=(0.0235, 0.0, 0.0185)),
        material=white_mark,
        name="pointer_wedge",
    )

    model.articulation(
        "cabinet_to_channel_dial",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=dial,
        origin=Origin(xyz=(DIAL_JOINT_X, COL_Y, DIAL_CZ)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=8.0),
    )

    # ------------------------------------------------- three small knobs
    knob_geom = KnobGeometry(
        KNOB_DIAMETER,
        0.015,
        body_style="cylindrical",
        grip=KnobGrip(style="fluted", count=16, depth=0.001),
        indicator=KnobIndicator(style="line", mode="raised"),
        center=False,
    )
    knob_mesh = mesh_from_geometry(knob_geom, "small_control_knob")
    for knob_name, knob_y in zip(("volume_knob", "brightness_knob", "tuning_knob"), KNOB_YS):
        knob = model.part(knob_name)
        knob.visual(
            Cylinder(radius=0.005, length=0.018),
            origin=Origin(xyz=(-0.0085, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_face,
            name=f"{knob_name}_stem",
        )
        knob.visual(
            knob_mesh,
            origin=Origin(xyz=(0.0005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=plastic,
            name=f"{knob_name}_cap",
        )
        model.articulation(
            f"cabinet_to_{knob_name}",
            ArticulationType.REVOLUTE,
            parent=cabinet,
            child=knob,
            origin=Origin(xyz=(KNOB_JOINT_X, knob_y, KNOB_ROW_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.3,
                velocity=8.0,
                lower=-KNOB_RANGE,
                upper=KNOB_RANGE,
            ),
        )

    # ============================================================= swivel joint
    # The cabinet sits on top of the pedestal. Joint origin is at the top of
    # the pedestal in the swivel_base frame. Axis +Z gives left-right rotation.
    model.articulation(
        "base_to_cabinet",
        ArticulationType.CONTINUOUS,
        parent=swivel_base,
        child=cabinet,
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    swivel_base = object_model.get_part("swivel_base")
    cabinet = object_model.get_part("cabinet")
    dial = object_model.get_part("channel_dial")
    knob_names = ("volume_knob", "brightness_knob", "tuning_knob")
    knobs = [object_model.get_part(name) for name in knob_names]
    swivel_joint = object_model.get_articulation("base_to_cabinet")
    dial_joint = object_model.get_articulation("cabinet_to_channel_dial")
    knob_joints = [object_model.get_articulation(f"cabinet_to_{name}") for name in knob_names]

    # ---- swivel base structure
    ctx.check(
        "swivel_base is the root part (only root)",
        len(object_model.root_parts()) == 1
        and object_model.root_parts()[0].name == "swivel_base",
        f"root parts: {[p.name for p in object_model.root_parts()]}",
    )
    pedestal = swivel_base.get_visual("pedestal")
    ctx.check(
        "swivel base has a round lathed pedestal",
        pedestal is not None and hasattr(pedestal.geometry, "filename"),
        "expected mesh-backed lathe pedestal",
    )
    ctx.check(
        "cabinet has true 1980s CRT TV dimensions (0.62 x 0.32 x 0.45 m)",
        abs(CAB_W - 0.62) < 1e-9 and abs(CAB_D - 0.32) < 1e-9 and abs(CAB_H - 0.45) < 1e-9,
        f"got W={CAB_W} D={CAB_D} H={CAB_H}",
    )

    # ---- swivel joint
    ctx.check(
        "base_to_cabinet joint is CONTINUOUS about +Z (vertical turntable)",
        swivel_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(swivel_joint.axis) == (0.0, 0.0, 1.0),
        f"got type={swivel_joint.articulation_type} axis={swivel_joint.axis}",
    )
    ctx.check(
        "swivel joint origin is at the pedestal top surface",
        abs(swivel_joint.origin.xyz[2] - PEDESTAL_TOP_Z) < 1e-9
        and abs(swivel_joint.origin.xyz[0]) < 1e-9
        and abs(swivel_joint.origin.xyz[1]) < 1e-9,
        f"got origin {swivel_joint.origin.xyz}",
    )
    limits = swivel_joint.motion_limits
    ctx.check(
        "swivel rotation is unlimited (CONTINUOUS)",
        limits is not None and limits.lower is None and limits.upper is None,
        f"got limits {limits}",
    )

    # ---- swivel pose: cabinet rotates left-right
    # The cabinet origin sits on the Z rotation axis, so origin position
    # does not change. Instead, verify the screen visual (offset from centre)
    # moves laterally when the turntable rotates.
    rest_aabb = ctx.part_world_aabb(cabinet)
    with ctx.pose({swivel_joint: math.pi / 2.0}):
        rotated_aabb = ctx.part_world_aabb(cabinet)
    if rest_aabb is not None and rotated_aabb is not None:
        rest_y_span = rest_aabb[1][1] - rest_aabb[0][1]
        rotated_y_span = rotated_aabb[1][1] - rotated_aabb[0][1]
        ctx.check(
            "cabinet rotates on the swivel (Y span changes at pi/2)",
            abs(rest_y_span - rotated_y_span) > 0.02,
            f"rest Y span={rest_y_span:.4f}, rotated Y span={rotated_y_span:.4f}",
        )
    else:
        ctx.fail("swivel rotation check", "cabinet AABB unavailable")

    # ---- cabinet sits on pedestal (z gap check)
    ctx.expect_gap(
        cabinet,
        swivel_base,
        axis="z",
        max_penetration=0.005,
        max_gap=0.005,
        name="cabinet bottom seats on the pedestal top",
    )

    # ---- hero front features exist and are placed correctly
    screen = cabinet.get_visual("crt_screen")
    bezel = cabinet.get_visual("screen_bezel")
    ctx.check(
        "CRT screen sits on the left two-thirds of the front",
        screen.origin.xyz[1] > 0.05 and bezel.origin.xyz[1] > 0.05,
        f"screen y={screen.origin.xyz[1]}, bezel y={bezel.origin.xyz[1]}",
    )
    ctx.check(
        "CRT screen is recessed behind the wood front edge",
        screen.origin.xyz[0] + 0.018 < FRONT_X,
        f"glass front x={screen.origin.xyz[0] + 0.018}, wood front x={FRONT_X}",
    )
    grille = cabinet.get_visual("grille_recess")
    vent = cabinet.get_visual("vent_recess")
    nameplate = cabinet.get_visual("samsung_nameplate")
    ctx.check(
        "vent, grille, and nameplate stack on the right control column",
        vent.origin.xyz[1] == COL_Y
        and grille.origin.xyz[1] == COL_Y
        and nameplate.origin.xyz[1] == COL_Y
        and vent.origin.xyz[2] > DIAL_CZ > KNOB_ROW_Z > grille.origin.xyz[2] > nameplate.origin.xyz[2],
        "expected top-to-bottom order: vent, dial, knobs, grille, nameplate",
    )
    ctx.check(
        "speaker grille has fine horizontal slats",
        sum(1 for v in cabinet.visuals if v.name and v.name.startswith("grille_slat_")) == 12,
        "expected 12 grille slats",
    )

    # ---- channel dial joint
    ctx.check(
        "channel dial joint is CONTINUOUS",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        f"got {dial_joint.articulation_type}",
    )
    ctx.check(
        "channel dial spins about the axis normal to the front face (+X)",
        tuple(dial_joint.axis) == (1.0, 0.0, 0.0),
        f"got axis {dial_joint.axis}",
    )
    dial_limits = dial_joint.motion_limits
    ctx.check(
        "channel dial rotation is unlimited",
        dial_limits is not None and dial_limits.lower is None and dial_limits.upper is None,
        f"got limits {dial_limits}",
    )
    ctx.check(
        "channel dial is ~0.07 m diameter on the upper control column",
        abs(DIAL_DIAMETER - 0.070) < 1e-9
        and dial_joint.origin.xyz[1] == COL_Y
        and dial_joint.origin.xyz[2] == DIAL_CZ,
        f"dial joint origin {dial_joint.origin.xyz}",
    )
    pointer = dial.get_visual("pointer_wedge")
    pointer_off_axis = math.hypot(pointer.origin.xyz[1], pointer.origin.xyz[2])
    ctx.check(
        "dial carries an off-axis pointer proving continuous rotation",
        pointer_off_axis > 0.012,
        f"pointer off-axis distance {pointer_off_axis}",
    )

    # ---- three small knobs
    for name, joint, knob_y in zip(knob_names, knob_joints, KNOB_YS):
        ctx.check(
            f"{name} joint is REVOLUTE about +X with -135..+135 deg travel",
            joint.articulation_type == ArticulationType.REVOLUTE
            and tuple(joint.axis) == (1.0, 0.0, 0.0)
            and joint.motion_limits is not None
            and abs(joint.motion_limits.lower + KNOB_RANGE) < 1e-9
            and abs(joint.motion_limits.upper - KNOB_RANGE) < 1e-9,
            f"got type={joint.articulation_type} axis={joint.axis} limits={joint.motion_limits}",
        )
        ctx.check(
            f"{name} sits in the row below the dial",
            joint.origin.xyz[1] == knob_y and joint.origin.xyz[2] == KNOB_ROW_Z,
            f"got origin {joint.origin.xyz}",
        )

    # ---- captured stems: scoped allowances + retained-insertion proof
    ctx.allow_overlap(
        dial,
        cabinet,
        elem_a="dial_stem",
        elem_b="front_plate",
        reason="Dial shaft is intentionally captured inside the front plate bore.",
    )
    ctx.allow_overlap(
        dial,
        cabinet,
        elem_a="dial_stem",
        elem_b="dial_escutcheon",
        reason="Dial shaft passes through the escutcheon boss.",
    )
    ctx.expect_overlap(
        dial,
        cabinet,
        axes="x",
        elem_a="dial_stem",
        elem_b="front_plate",
        min_overlap=0.008,
        name="dial shaft stays inserted in the front plate",
    )
    ctx.expect_within(
        dial,
        cabinet,
        axes="yz",
        name="dial stays within the cabinet front footprint",
    )
    for name, knob in zip(knob_names, knobs):
        ctx.allow_overlap(
            knob,
            cabinet,
            elem_a=f"{name}_stem",
            elem_b="front_plate",
            reason="Knob shaft is intentionally captured inside the front plate bore.",
        )
        ctx.expect_overlap(
            knob,
            cabinet,
            axes="x",
            elem_a=f"{name}_stem",
            elem_b="front_plate",
            min_overlap=0.008,
            name=f"{name} shaft stays inserted in the front plate",
        )

    # ---- decisive articulated poses: rotation about +X keeps shafts captured
    volume_joint = object_model.get_articulation("cabinet_to_volume_knob")
    with ctx.pose({dial_joint: math.pi, volume_joint: KNOB_RANGE}):
        ctx.expect_overlap(
            dial,
            cabinet,
            axes="x",
            elem_a="dial_stem",
            elem_b="front_plate",
            min_overlap=0.008,
            name="spun dial shaft remains captured",
        )
        ctx.expect_overlap(
            knobs[0],
            cabinet,
            axes="x",
            elem_a="volume_knob_stem",
            elem_b="front_plate",
            min_overlap=0.008,
            name="turned volume knob shaft remains captured",
        )

    return ctx.report()


object_model = build_object_model()
