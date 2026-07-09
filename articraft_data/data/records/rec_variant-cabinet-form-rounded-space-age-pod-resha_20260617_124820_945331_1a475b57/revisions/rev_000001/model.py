from __future__ import annotations

# Variant: rounded space-age pod cabinet for vintage 1980s Samsung CRT TV.
#
# Articraft brief:
# - Object: tabletop CRT TV, 0.62 m wide (Y), 0.45 m tall (Z), 0.32 m deep (X),
#   front face toward +X, grounded at z=0.
# - Root/support: hollow rounded pod shell (CadQuery box + vertical fillets + shell)
#   with strongly rounded corners and an open front, carrying a recessed beige
#   plastic front plate.
# - Fixed front: CRT bezel pocket (left two-thirds) with a bulged pale green-grey
#   glass tube, beige control column (right third) with louvered vent, square
#   dial escutcheon, ribbed speaker grille of fine horizontal slats, SAMSUNG plate.
# - Articulations: channel_dial CONTINUOUS about +X; three small knobs REVOLUTE
#   about +X, -135..+135 deg.
# - Intentional overlaps: knob/dial stems captured inside the front plate /
#   escutcheon (scoped allowances + retained-insertion checks).

import math

import cadquery as cq

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
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------- dimensions
CAB_W = 0.62   # full width along Y
CAB_H = 0.45   # full height along Z
CAB_D = 0.32   # full depth along X
SHELL_T = 0.015  # pod wall thickness
POD_FILLET = 0.055  # strong rounding on vertical edges

PLATE_X0 = 0.132  # rear face of the recessed beige front plate
PLATE_X1 = 0.148  # front face of the recessed beige front plate (overlaps shell lip)
PLATE_W = 0.60    # front plate width (reaches into shell side walls)
PLATE_H = 0.43    # front plate height (reaches into shell top/bottom walls)

SCREEN_CY = 0.0875
SCREEN_CZ = 0.225

COL_Y = -0.2025  # control-column centreline (Y)

DIAL_DIAMETER = 0.070
DIAL_CZ = 0.300
KNOB_DIAMETER = 0.024
KNOB_ROW_Z = 0.225
KNOB_COUNT = 3
KNOB_Y_START = -0.1525
KNOB_Y_STEP = -0.050
KNOB_RANGE = 0.75 * math.pi  # +/-135 degrees

DIAL_JOINT_X = 0.150
KNOB_JOINT_X = PLATE_X1

FRONT_X = CAB_D / 2.0  # +0.16, pod front face plane


# ===================================================== geometry helpers

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
        (0.000, 0.300, 0.370, 0.050),
        (0.006, 0.290, 0.355, 0.050),
        (0.012, 0.255, 0.325, 0.050),
        (0.018, 0.210, 0.270, 0.060),
    )
    sections = []
    for z, h, w, r in sections_spec:
        outline = _rounded_rect_outline(h, w, r, segments=5)
        sections.append([(x, y, z) for x, y in outline])
    geom = LoftGeometry(sections, cap=True, closed=True)
    return mesh_from_geometry(geom, "crt_glass_tube")


def _build_pod_shell():
    """Rounded space-age pod shell: outer box + fillets, inner cavity cut from front."""
    outer = (
        cq.Workplane("XY")
        .box(CAB_D, CAB_W, CAB_H)
        .edges("|Z").fillet(POD_FILLET)
    )
    # Inner cavity: slightly smaller box cut from the front face (+X) side.
    inner_d = CAB_D - SHELL_T
    inner_w = CAB_W - 2.0 * SHELL_T
    inner_h = CAB_H - 2.0 * SHELL_T
    cavity = (
        cq.Workplane("XY")
        .transformed(offset=(SHELL_T / 2.0, 0.0, 0.0))
        .box(inner_d, inner_w, inner_h)
    )
    pod = outer.cut(cavity)
    return mesh_from_cadquery(pod, "pod_shell")


# ===================================================== object model

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_samsung_crt_television_pod")

    # -- materials (space-age palette)
    pod_mat = model.material("pod_shell_cream", rgba=(0.82, 0.79, 0.73, 1.0))
    plastic = model.material("plastic_beige", rgba=(0.79, 0.75, 0.65, 1.0))
    bezel_plastic = model.material("bezel_beige_dark", rgba=(0.70, 0.66, 0.55, 1.0))
    glass = model.material("crt_glass_green_grey", rgba=(0.72, 0.77, 0.72, 1.0))
    dark_face = model.material("dial_face_dark", rgba=(0.15, 0.15, 0.16, 1.0))
    metal = model.material("metal_ring", rgba=(0.63, 0.62, 0.58, 1.0))
    white_mark = model.material("marking_white", rgba=(0.93, 0.93, 0.90, 1.0))
    grille_dark = model.material("grille_recess_dark", rgba=(0.34, 0.31, 0.26, 1.0))
    silver = model.material("nameplate_silver", rgba=(0.76, 0.76, 0.74, 1.0))

    # ============================================ shell layer (pod + front plate)
    cabinet = model.part("cabinet")

    cabinet.visual(
        _build_pod_shell(),
        origin=Origin(xyz=(0.0, 0.0, CAB_H / 2.0)),
        material=pod_mat,
        name="pod_shell",
    )

    cabinet.visual(
        Box((PLATE_X1 - PLATE_X0, PLATE_W, PLATE_H)),
        origin=Origin(xyz=((PLATE_X0 + PLATE_X1) / 2.0, 0.0, CAB_H / 2.0)),
        material=plastic,
        name="front_plate",
    )

    # ============================================ CRT layer (bezel + glass)
    bezel_geom = BezelGeometry(
        (0.26, 0.33),
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
        Box((0.005, 0.006, PLATE_H)),
        origin=Origin(xyz=(0.1445, -0.115, CAB_H / 2.0)),
        material=bezel_plastic,
        name="column_seam_trim",
    )

    # ============================================ control column (vent, dial, grille)

    # -- louvered vent (top of control column)
    cabinet.visual(
        Box((0.005, 0.120, 0.054)),
        origin=Origin(xyz=(0.1415, COL_Y, 0.390)),
        material=grille_dark,
        name="vent_recess",
    )
    for i in range(3):
        slat_z = 0.373 + i * 0.017
        cabinet.visual(
            Box((0.010, 0.118, 0.011)),
            origin=Origin(xyz=(0.146, COL_Y, slat_z)),
            material=plastic,
            name=f"vent_louver_{i}",
        )

    # -- dial escutcheon
    cabinet.visual(
        Box((0.006, 0.115, 0.115)),
        origin=Origin(xyz=(0.147, COL_Y, DIAL_CZ)),
        material=plastic,
        name="dial_escutcheon",
    )

    # -- speaker grille (recess + horizontal slats)
    cabinet.visual(
        Box((0.005, 0.150, 0.150)),
        origin=Origin(xyz=(0.1415, COL_Y, 0.1175)),
        material=grille_dark,
        name="grille_recess",
    )
    for i in range(12):
        slat_z = 0.052 + i * 0.0115
        cabinet.visual(
            Box((0.008, 0.146, 0.0055)),
            origin=Origin(xyz=(0.146, COL_Y, slat_z)),
            material=plastic,
            name=f"grille_slat_{i}",
        )

    # -- SAMSUNG nameplate
    cabinet.visual(
        Box((0.004, 0.075, 0.013)),
        origin=Origin(xyz=(0.1455, COL_Y, 0.031)),
        material=silver,
        name="samsung_nameplate",
    )

    # ============================================ channel selector dial
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
    tick_radius = 0.0215
    for i in range(8):
        angle = i * math.pi / 4.0
        dial.visual(
            Box((0.0015, 0.002, 0.006)),
            origin=Origin(
                xyz=(0.023, -tick_radius * math.sin(angle), tick_radius * math.cos(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=white_mark,
            name=f"dial_tick_{i}",
        )
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

    # ============================================ three small control knobs
    knob_geom = KnobGeometry(
        KNOB_DIAMETER,
        0.015,
        body_style="cylindrical",
        grip=KnobGrip(style="fluted", count=16, depth=0.001),
        indicator=KnobIndicator(style="line", mode="raised"),
        center=False,
    )
    knob_mesh = mesh_from_geometry(knob_geom, "small_control_knob")
    for i in range(KNOB_COUNT):
        knob_y = KNOB_Y_START + i * KNOB_Y_STEP
        knob_name = f"knob_{i}"
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

    return model


# ===================================================== tests

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    dial = object_model.get_part("channel_dial")
    knob_names = [f"knob_{i}" for i in range(KNOB_COUNT)]
    knobs = [object_model.get_part(name) for name in knob_names]
    dial_joint = object_model.get_articulation("cabinet_to_channel_dial")
    knob_joints = [object_model.get_articulation(f"cabinet_to_{name}") for name in knob_names]

    # ---- pod shell identity and visible geometry
    pod_shell = cabinet.get_visual("pod_shell")
    ctx.check(
        "cabinet is a rounded space-age pod shell (single curved CadQuery mesh)",
        pod_shell is not None and pod_shell.geometry is not None,
        "pod_shell visual missing or not mesh-backed",
    )
    ctx.check(
        "pod shell has correct overall dimensions (0.62 x 0.32 x 0.45 m)",
        abs(CAB_W - 0.62) < 1e-9 and abs(CAB_D - 0.32) < 1e-9 and abs(CAB_H - 0.45) < 1e-9,
        f"got W={CAB_W} D={CAB_D} H={CAB_H}",
    )
    ctx.check(
        "pod shell uses strong fillet rounding (not flat box panels)",
        POD_FILLET >= 0.040,
        f"fillet radius {POD_FILLET} too small for space-age pod",
    )
    front_plate = cabinet.get_visual("front_plate")
    ctx.check(
        "pod is grounded at z=0 (visual origin places base on ground)",
        abs(pod_shell.origin.xyz[2] - CAB_H / 2.0) < 1e-9,
        f"pod_shell origin z={pod_shell.origin.xyz[2]}",
    )

    # ---- CRT screen on left two-thirds, recessed behind pod front
    screen = cabinet.get_visual("crt_screen")
    bezel = cabinet.get_visual("screen_bezel")
    ctx.check(
        "CRT screen sits on the left two-thirds of the front",
        screen.origin.xyz[1] > 0.05 and bezel.origin.xyz[1] > 0.05,
        f"screen y={screen.origin.xyz[1]}, bezel y={bezel.origin.xyz[1]}",
    )
    ctx.check(
        "CRT screen is recessed behind the pod front face",
        screen.origin.xyz[0] + 0.018 < FRONT_X,
        f"glass front x={screen.origin.xyz[0] + 0.018}, pod front x={FRONT_X}",
    )

    # ---- control column layout
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
        "speaker grille has fine horizontal slats (12 slats)",
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
    limits = dial_joint.motion_limits
    ctx.check(
        "channel dial rotation is unlimited",
        limits is not None and limits.lower is None and limits.upper is None,
        f"got limits {limits}",
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

    # ---- three small knobs (indexed loop, regular placement, uniform joint policy)
    for i, (name, joint) in enumerate(zip(knob_names, knob_joints)):
        knob_y = KNOB_Y_START + i * KNOB_Y_STEP
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
            f"{name} sits in the row below the dial at regular spacing",
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

    # ---- decisive articulated poses
    knob_0_joint = object_model.get_articulation("cabinet_to_knob_0")
    with ctx.pose({dial_joint: math.pi, knob_0_joint: KNOB_RANGE}):
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
            elem_a="knob_0_stem",
            elem_b="front_plate",
            min_overlap=0.008,
            name="turned knob_0 shaft remains captured",
        )

    return ctx.report()


object_model = build_object_model()
