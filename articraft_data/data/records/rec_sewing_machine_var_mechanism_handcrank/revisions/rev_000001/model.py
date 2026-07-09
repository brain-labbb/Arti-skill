from __future__ import annotations

"""White compact household sewing machine with hand-crank drive.

Articraft brief:
- Object: portable hand-crank domestic lockstitch sewing machine, ~0.40 m wide,
  ~0.30 m tall, ~0.17 m deep. White body, blue paisley art panel on the front
  pillar, silver needle plate, chrome needle/presser bars. Manual power input
  via a folding hand-crank mounted on the balance wheel.
- Root/support: flat bed base (root) with rubber feet; the C-shaped body casting
  (pillar + arm + head) is fixed onto the bed.
- Articulations:
  - handwheel_spin: CONTINUOUS, side handwheel on the right pillar face, axis +X.
  - crank_arm_mount: FIXED, radial crank arm bolted to the wheel outer face.
  - crank_grip_spin: CONTINUOUS, grip rotating freely on the crank pin, axis +X.
  - stitch_select: REVOLUTE, large fluted stitch-selector dial on the pillar front.
  - reverse_press: PRISMATIC, reverse-stitch lever pressed downward.
  - tension_adjust: REVOLUTE, upper-thread tension dial on the head front.
  - needle_stroke: PRISMATIC, chrome needle bar with clamp and needle, vertical.
  - presser_lift: PRISMATIC, presser bar with slotted presser foot, vertical lift.
- Visible geometry: open harp space between head and bed, needle plate with feed
  dog, bobbin cover, spool pin / bobbin winder on top, paisley accent panel with
  cutouts where the dial and lever pass through, steel crank arm with dark grip.
- Intentional overlaps: control shafts and the needle/presser bars are captured
  in bores of the body casting; the body casting seats 2 mm into the bed; the
  crank hub embeds into the wheel face; the grip barrel encloses the crank pin.
- Frame: +X right (handwheel side), +Y back, +Z up. Front face at -Y.
"""

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobRelief,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------- dimensions
BED_TOP = 0.062
HEAD_BOTTOM = 0.115
PLATE_TOP = 0.0645
NEEDLE_X = -0.145
NEEDLE_Y = -0.018
PRESSER_Y = 0.004

WHITE = Material(name="body_white", rgba=(0.955, 0.955, 0.945, 1.0))
WHITE_SOFT = Material(name="trim_white", rgba=(0.92, 0.92, 0.91, 1.0))
PALE_BLUE = Material(name="paisley_blue", rgba=(0.72, 0.80, 0.92, 1.0))
STEEL = Material(name="steel_silver", rgba=(0.72, 0.73, 0.75, 1.0))
CHROME = Material(name="chrome", rgba=(0.83, 0.85, 0.88, 1.0))
DARK = Material(name="dark_gray", rgba=(0.20, 0.20, 0.22, 1.0))
LIGHT_GRAY = Material(name="light_gray", rgba=(0.82, 0.82, 0.84, 1.0))
RUBBER = Material(name="rubber_gray", rgba=(0.35, 0.35, 0.36, 1.0))


def _filleted(shape: cq.Workplane, selector: str, radii: tuple[float, ...]) -> cq.Workplane:
    """Try progressively smaller fillets; fall back to the unfilleted shape."""
    for radius in radii:
        try:
            return shape.edges(selector).fillet(radius)
        except Exception:
            continue
    return shape


def _bed_shape() -> cq.Workplane:
    bed = cq.Workplane("XY").box(0.400, 0.170, 0.052)
    bed = _filleted(bed, "|Z", (0.012, 0.008))
    bed = _filleted(bed, ">Z", (0.004, 0.002))
    return bed.translate((0.0, 0.0, 0.036))


def _body_shape() -> cq.Workplane:
    pillar = cq.Workplane("XY").box(0.100, 0.150, 0.240).translate((0.150, 0.0, 0.180))
    arm = cq.Workplane("XY").box(0.330, 0.120, 0.088).translate((0.035, 0.0, 0.256))
    head = cq.Workplane("XY").box(0.105, 0.140, 0.185).translate((-0.1475, 0.0, 0.2075))
    body = pillar.union(arm).union(head)
    # Round the front/back silhouette edges like the molded housing in the photo.
    body = _filleted(body, "|Y", (0.012, 0.008, 0.005))
    return body


def _art_panel_shape() -> cq.Workplane:
    """Front paisley art panel with pass-through cutouts for dial and lever.

    Local frame: x = machine x, y = machine z, thickness along machine -y
    (built on the XZ workplane). Panel center sits at (0.152, *, 0.175).
    """
    panel = cq.Workplane("XZ").box(0.088, 0.205, 0.0015)
    dial_hole = (
        cq.Workplane("XZ")
        .center(-0.002, 0.030)
        .circle(0.0305)
        .extrude(0.02, both=True)
    )
    lever_hole = (
        cq.Workplane("XZ")
        .center(-0.002, -0.045)
        .rect(0.018, 0.040)
        .extrude(0.02, both=True)
    )
    return panel.cut(dial_hole).cut(lever_hole)


def _foot_sole_shape() -> cq.Workplane:
    """Presser foot sole with a through needle slot.

    Local frame center at foot center; needle slot offset -0.008 in y.
    """
    sole = cq.Workplane("XY").box(0.016, 0.034, 0.0042)
    sole = _filleted(sole, "|Z", (0.004, 0.002))
    slot = cq.Workplane("XY").center(0.0, -0.008).rect(0.0045, 0.012).extrude(0.02, both=True)
    return sole.cut(slot)


def _paddle_shape() -> cq.Workplane:
    paddle = cq.Workplane("XY").box(0.020, 0.012, 0.034)
    return _filleted(paddle, "|Y", (0.004, 0.002))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_sewing_machine")

    # ------------------------------------------------------------------ bed
    bed = model.part("bed")
    bed.visual(mesh_from_cadquery(_bed_shape(), "bed_shell"), material=WHITE, name="bed_shell")
    # Rubber feet under the four corners.
    for i, (fx, fy) in enumerate([(-0.175, -0.060), (0.175, -0.060), (-0.175, 0.060), (0.175, 0.060)]):
        bed.visual(
            Cylinder(radius=0.011, length=0.012),
            origin=Origin(xyz=(fx, fy, 0.006)),
            material=RUBBER,
            name=f"foot_pad_{i}",
        )
    # Silver needle plate with dark feed dog inset.
    bed.visual(
        Box((0.095, 0.060, 0.0025)),
        origin=Origin(xyz=(NEEDLE_X, -0.010, 0.06325)),
        material=STEEL,
        name="needle_plate",
    )
    bed.visual(
        Box((0.028, 0.014, 0.0008)),
        origin=Origin(xyz=(NEEDLE_X, NEEDLE_Y, 0.0644)),
        material=DARK,
        name="feed_dog",
    )
    # Removable bobbin cover panel on the front of the bed.
    bed.visual(
        Box((0.070, 0.026, 0.0016)),
        origin=Origin(xyz=(-0.100, -0.058, 0.0626)),
        material=LIGHT_GRAY,
        name="bobbin_cover",
    )

    # ----------------------------------------------------------------- body
    body = model.part("body")
    body.visual(mesh_from_cadquery(_body_shape(), "body_shell"), material=WHITE, name="body_shell")
    # Blue paisley art panel on the pillar front (dial/lever pass through cutouts).
    body.visual(
        mesh_from_cadquery(_art_panel_shape(), "front_art_panel"),
        origin=Origin(xyz=(0.152, -0.0752, 0.175)),
        material=PALE_BLUE,
        name="front_art_panel",
    )
    # Thread take-up slot on the head front.
    body.visual(
        Box((0.006, 0.0014, 0.100)),
        origin=Origin(xyz=(-0.170, -0.0702, 0.235)),
        material=DARK,
        name="takeup_slot",
    )
    # Top deck details: spool pin, bobbin winder spindle and stopper, thread guide.
    body.visual(
        Cylinder(radius=0.0035, length=0.040),
        origin=Origin(xyz=(0.155, 0.010, 0.316)),
        material=WHITE_SOFT,
        name="spool_pin",
    )
    body.visual(
        Cylinder(radius=0.0025, length=0.026),
        origin=Origin(xyz=(0.105, 0.010, 0.309)),
        material=CHROME,
        name="bobbin_winder_spindle",
    )
    body.visual(
        Cylinder(radius=0.0045, length=0.010),
        origin=Origin(xyz=(0.128, 0.010, 0.301)),
        material=WHITE_SOFT,
        name="winder_stopper",
    )
    body.visual(
        Box((0.016, 0.012, 0.010)),
        origin=Origin(xyz=(-0.085, 0.0, 0.303)),
        material=LIGHT_GRAY,
        name="thread_guide",
    )

    model.articulation(
        "bed_to_body",
        ArticulationType.FIXED,
        parent=bed,
        child=body,
    )

    # ------------------------------------------------------------ handwheel
    handwheel = model.part("handwheel")
    wheel_geo = KnobGeometry(
        0.080,
        0.020,
        body_style="cylindrical",
        edge_radius=0.0015,
        grip=KnobGrip(style="ribbed", count=36, depth=0.0006, width=0.0018),
        body_reliefs=(KnobRelief(style="top_recess", width=0.046, depth=0.004),),
        center=False,
    )
    handwheel.visual(
        mesh_from_geometry(wheel_geo, "handwheel_cap"),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=WHITE,
        name="handwheel_cap",
    )
    handwheel.visual(
        Cylinder(radius=0.008, length=0.028),
        origin=Origin(xyz=(-0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=CHROME,
        name="handwheel_shaft",
    )
    model.articulation(
        "handwheel_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=handwheel,
        origin=Origin(xyz=(0.200, 0.0, 0.2565)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )

    # --------------------------------------------------- hand-crank assembly
    # A radial crank arm bolted to the outer face of the handwheel, with a
    # freely spinning grip at its tip. The operator turns the grip to drive
    # wheel rotation, replacing the internal electric motor.
    crank_arm = model.part("crank_arm")
    # Hub boss that clamps to the wheel face.
    crank_arm.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(0.022, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=STEEL,
        name="crank_hub",
    )
    # Radial bar extending outward from the hub past the wheel rim.
    # Inner face at x=0.020 contacts the wheel outer face (no gap).
    crank_arm.visual(
        Box((0.004, 0.010, 0.050)),
        origin=Origin(xyz=(0.022, 0.0, 0.028)),
        material=STEEL,
        name="crank_bar",
    )
    # Pin on which the grip rotates (bearing journal).
    crank_arm.visual(
        Cylinder(radius=0.003, length=0.016),
        origin=Origin(xyz=(0.022, 0.0, 0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=CHROME,
        name="grip_pin",
    )
    model.articulation(
        "crank_arm_mount",
        ArticulationType.FIXED,
        parent=handwheel,
        child=crank_arm,
    )

    crank_grip = model.part("crank_grip")
    # Cylindrical grip handle (dark bakelite/ebony style).
    crank_grip.visual(
        Cylinder(radius=0.009, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=DARK,
        name="grip_barrel",
    )
    # Raised finger ridge along the grip — non-axisymmetric so the spin is
    # visually detectable and the grip reads as a turned handle.
    crank_grip.visual(
        Box((0.024, 0.006, 0.003)),
        origin=Origin(xyz=(0.0, 0.011, 0.0)),
        material=DARK,
        name="grip_ridge",
    )
    model.articulation(
        "crank_grip_spin",
        ArticulationType.CONTINUOUS,
        parent=crank_arm,
        child=crank_grip,
        origin=Origin(xyz=(0.022, 0.0, 0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0),
    )

    # ------------------------------------------------- stitch selector dial
    stitch_dial = model.part("stitch_dial")
    dial_geo = KnobGeometry(
        0.056,
        0.017,
        body_style="cylindrical",
        edge_radius=0.0012,
        grip=KnobGrip(style="fluted", count=22, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
        center=False,
    )
    stitch_dial.visual(
        mesh_from_geometry(dial_geo, "stitch_dial_cap"),
        origin=Origin(xyz=(0.0, -0.0016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=WHITE,
        name="stitch_dial_cap",
    )
    stitch_dial.visual(
        Cylinder(radius=0.006, length=0.022),
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=CHROME,
        name="stitch_dial_shaft",
    )
    model.articulation(
        "stitch_select",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stitch_dial,
        origin=Origin(xyz=(0.150, -0.075, 0.205)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-math.pi, upper=math.pi),
    )

    # ------------------------------------------------- reverse-stitch lever
    reverse_lever = model.part("reverse_lever")
    reverse_lever.visual(
        mesh_from_cadquery(_paddle_shape(), "reverse_lever_paddle"),
        origin=Origin(xyz=(0.0, -0.011, 0.0)),
        material=WHITE_SOFT,
        name="reverse_lever_paddle",
    )
    reverse_lever.visual(
        Box((0.012, 0.016, 0.020)),
        origin=Origin(xyz=(0.0, 0.002, 0.0)),
        material=WHITE_SOFT,
        name="reverse_lever_stem",
    )
    model.articulation(
        "reverse_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=reverse_lever,
        origin=Origin(xyz=(0.150, -0.075, 0.130)),
        # Positive q presses the lever downward, as on the real machine.
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.1, lower=0.0, upper=0.008),
    )

    # -------------------------------------------------- upper tension dial
    tension_dial = model.part("tension_dial")
    tension_geo = KnobGeometry(
        0.030,
        0.009,
        body_style="cylindrical",
        edge_radius=0.0008,
        grip=KnobGrip(style="fluted", count=18, depth=0.0007),
        indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
        center=False,
    )
    tension_dial.visual(
        mesh_from_geometry(tension_geo, "tension_dial_cap"),
        origin=Origin(xyz=(0.0, -0.0004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=WHITE,
        name="tension_dial_cap",
    )
    tension_dial.visual(
        Cylinder(radius=0.004, length=0.016),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=CHROME,
        name="tension_dial_shaft",
    )
    model.articulation(
        "tension_adjust",
        ArticulationType.REVOLUTE,
        parent=body,
        child=tension_dial,
        origin=Origin(xyz=(-0.130, -0.070, 0.215)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=4.0, lower=-math.pi, upper=math.pi),
    )

    # ------------------------------------------------------------ needle bar
    needle_bar = model.part("needle_bar")
    needle_bar.visual(
        Cylinder(radius=0.0038, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=CHROME,
        name="needle_bar_shaft",
    )
    needle_bar.visual(
        Box((0.011, 0.011, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=STEEL,
        name="needle_clamp",
    )
    needle_bar.visual(
        Cylinder(radius=0.0007, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=CHROME,
        name="needle",
    )
    model.articulation(
        "needle_stroke",
        ArticulationType.PRISMATIC,
        parent=body,
        child=needle_bar,
        origin=Origin(xyz=(NEEDLE_X, NEEDLE_Y, HEAD_BOTTOM)),
        axis=(0.0, 0.0, 1.0),
        # Negative q drives the needle toward the plate; positive raises it.
        motion_limits=MotionLimits(effort=10.0, velocity=0.5, lower=-0.0045, upper=0.010),
    )

    # --------------------------------------------------------- presser foot
    presser_foot = model.part("presser_foot")
    presser_foot.visual(
        Cylinder(radius=0.0032, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=CHROME,
        name="presser_bar_shaft",
    )
    presser_foot.visual(
        Box((0.008, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.038)),
        material=STEEL,
        name="foot_shank",
    )
    presser_foot.visual(
        mesh_from_cadquery(_foot_sole_shape(), "foot_sole"),
        origin=Origin(xyz=(0.0, -0.014, -0.0479)),
        material=STEEL,
        name="foot_sole",
    )
    model.articulation(
        "presser_lift",
        ArticulationType.PRISMATIC,
        parent=body,
        child=presser_foot,
        origin=Origin(xyz=(NEEDLE_X, PRESSER_Y, HEAD_BOTTOM)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.2, lower=0.0, upper=0.007),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bed = object_model.get_part("bed")
    body = object_model.get_part("body")
    handwheel = object_model.get_part("handwheel")
    crank_arm = object_model.get_part("crank_arm")
    crank_grip = object_model.get_part("crank_grip")
    stitch_dial = object_model.get_part("stitch_dial")
    reverse_lever = object_model.get_part("reverse_lever")
    tension_dial = object_model.get_part("tension_dial")
    needle_bar = object_model.get_part("needle_bar")
    presser_foot = object_model.get_part("presser_foot")

    needle_joint = object_model.get_articulation("needle_stroke")
    presser_joint = object_model.get_articulation("presser_lift")
    reverse_joint = object_model.get_articulation("reverse_press")
    crank_grip_joint = object_model.get_articulation("crank_grip_spin")

    # ---------------------------------------------------------- allowances
    ctx.allow_overlap(
        body,
        bed,
        elem_a="body_shell",
        elem_b="bed_shell",
        reason="The pillar casting intentionally seats 2 mm into the bed base.",
    )
    ctx.allow_overlap(
        body,
        handwheel,
        elem_a="body_shell",
        elem_b="handwheel_shaft",
        reason="Handwheel shaft is captured in the pillar bushing bore.",
    )
    ctx.allow_overlap(
        handwheel,
        crank_arm,
        elem_a="handwheel_cap",
        elem_b="crank_hub",
        reason="Crank hub is bolted to the handwheel outer face.",
    )
    ctx.allow_overlap(
        crank_arm,
        crank_grip,
        elem_a="grip_pin",
        elem_b="grip_barrel",
        reason="Grip barrel encloses the crank pin as a rotational bearing.",
    )
    ctx.allow_overlap(
        crank_arm,
        crank_grip,
        elem_a="crank_bar",
        elem_b="grip_barrel",
        reason="Grip barrel surrounds the arm end where it encloses the pin joint.",
    )
    ctx.allow_overlap(
        body,
        stitch_dial,
        elem_a="body_shell",
        elem_b="stitch_dial_shaft",
        reason="Stitch-selector shaft is captured in the pillar bore behind the panel.",
    )
    ctx.allow_overlap(
        body,
        reverse_lever,
        elem_a="body_shell",
        elem_b="reverse_lever_stem",
        reason="Reverse lever stem slides in the pillar slot behind the art panel.",
    )
    ctx.allow_overlap(
        body,
        tension_dial,
        elem_a="body_shell",
        elem_b="tension_dial_shaft",
        reason="Tension dial shaft is captured in the head bore.",
    )
    ctx.allow_overlap(
        body,
        needle_bar,
        elem_a="body_shell",
        elem_b="needle_bar_shaft",
        reason="Needle bar slides vertically inside the head bushing.",
    )
    ctx.allow_overlap(
        body,
        presser_foot,
        elem_a="body_shell",
        elem_b="presser_bar_shaft",
        reason="Presser bar slides vertically inside the head bushing.",
    )

    # ------------------------------------------------ rest-pose visual claims
    ctx.expect_gap(
        needle_bar,
        bed,
        axis="z",
        positive_elem="needle",
        negative_elem="needle_plate",
        min_gap=0.002,
        max_gap=0.012,
        name="needle tip hovers above the needle plate at rest",
    )
    ctx.expect_gap(
        presser_foot,
        bed,
        axis="z",
        positive_elem="foot_sole",
        negative_elem="needle_plate",
        min_gap=0.0,
        max_gap=0.002,
        name="presser foot sole sits just on the needle plate",
    )
    ctx.expect_overlap(
        needle_bar,
        presser_foot,
        axes="xy",
        elem_a="needle",
        elem_b="foot_sole",
        min_overlap=0.0009,
        name="needle is aligned over the presser foot needle slot",
    )
    ctx.expect_within(
        presser_foot,
        bed,
        axes="xy",
        inner_elem="foot_sole",
        outer_elem="needle_plate",
        margin=0.001,
        name="presser foot works on the needle plate",
    )

    # Handwheel protrudes from the right pillar face.
    hw_aabb = ctx.part_world_aabb(handwheel)
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "handwheel protrudes from the right side of the pillar",
        hw_aabb is not None and body_aabb is not None and hw_aabb[1][0] > body_aabb[1][0] + 0.010,
        details=f"handwheel_aabb={hw_aabb}, body_aabb={body_aabb}",
    )
    ctx.expect_overlap(
        handwheel,
        body,
        axes="x",
        elem_a="handwheel_shaft",
        min_overlap=0.012,
        name="handwheel shaft stays inserted in the pillar",
    )

    # Front controls protrude from the front faces.
    dial_aabb = ctx.part_world_aabb(stitch_dial)
    ctx.check(
        "stitch selector dial protrudes from the pillar front",
        dial_aabb is not None and dial_aabb[0][1] < -0.076,
        details=f"stitch_dial_aabb={dial_aabb}",
    )
    tension_aabb = ctx.part_world_aabb(tension_dial)
    ctx.check(
        "tension dial protrudes from the head front",
        tension_aabb is not None and tension_aabb[0][1] < -0.071,
        details=f"tension_dial_aabb={tension_aabb}",
    )

    # ---------------------------------------------------- decisive pose checks
    rest_needle = ctx.part_element_world_aabb(needle_bar, elem="needle")
    with ctx.pose({needle_joint: -0.0045}):
        low_needle = ctx.part_element_world_aabb(needle_bar, elem="needle")
        ctx.check(
            "needle descends toward the plate without striking it",
            rest_needle is not None
            and low_needle is not None
            and low_needle[0][2] < rest_needle[0][2] - 0.003
            and low_needle[0][2] > 0.0646,
            details=f"rest={rest_needle}, low={low_needle}",
        )

    rest_sole = ctx.part_element_world_aabb(presser_foot, elem="foot_sole")
    with ctx.pose({presser_joint: 0.007}):
        lifted_sole = ctx.part_element_world_aabb(presser_foot, elem="foot_sole")
        ctx.check(
            "presser foot lifts off the needle plate",
            rest_sole is not None
            and lifted_sole is not None
            and lifted_sole[0][2] > rest_sole[0][2] + 0.005,
            details=f"rest={rest_sole}, lifted={lifted_sole}",
        )

    rest_lever = ctx.part_world_position(reverse_lever)
    with ctx.pose({reverse_joint: 0.008}):
        pressed_lever = ctx.part_world_position(reverse_lever)
        ctx.check(
            "reverse-stitch lever presses downward",
            rest_lever is not None
            and pressed_lever is not None
            and pressed_lever[2] < rest_lever[2] - 0.006,
            details=f"rest={rest_lever}, pressed={pressed_lever}",
        )

    # --------------------------------------------------- hand-crank checks
    # The crank hub is bolted to the handwheel outer face.
    ctx.expect_overlap(
        crank_arm,
        handwheel,
        axes="x",
        elem_a="crank_hub",
        elem_b="handwheel_cap",
        min_overlap=0.001,
        name="crank hub embeds into the handwheel face",
    )

    # The crank arm extends radially beyond the wheel rim (visible at rest).
    crank_arm_aabb = ctx.part_world_aabb(crank_arm)
    hw_aabb_for_crank = ctx.part_world_aabb(handwheel)
    ctx.check(
        "crank arm extends radially beyond the handwheel rim",
        crank_arm_aabb is not None
        and hw_aabb_for_crank is not None
        and crank_arm_aabb[1][2] > hw_aabb_for_crank[1][2] + 0.008,
        details=f"crank_arm_aabb={crank_arm_aabb}, handwheel_aabb={hw_aabb_for_crank}",
    )

    # The crank pin stays within the grip barrel bore (bearing fit).
    ctx.expect_within(
        crank_arm,
        crank_grip,
        axes="yz",
        inner_elem="grip_pin",
        outer_elem="grip_barrel",
        margin=0.0,
        name="crank pin is captured inside the grip barrel",
    )

    # The grip spins freely on its pin — a π/2 rotation moves the ridge from
    # +Y toward +Z, proving the CONTINUOUS joint drives visible motion.
    rest_ridge = ctx.part_element_world_aabb(crank_grip, elem="grip_ridge")
    with ctx.pose({crank_grip_joint: math.pi / 2.0}):
        spun_ridge = ctx.part_element_world_aabb(crank_grip, elem="grip_ridge")
        ctx.check(
            "crank_grip_spin drives the grip handle around the pin axis",
            rest_ridge is not None
            and spun_ridge is not None
            and abs(spun_ridge[1][1] - rest_ridge[1][1]) > 0.005,
            details=f"rest_ridge={rest_ridge}, spun_ridge={spun_ridge}",
        )

    return ctx.report()


object_model = build_object_model()
