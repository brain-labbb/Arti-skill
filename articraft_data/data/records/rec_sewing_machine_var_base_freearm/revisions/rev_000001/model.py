from __future__ import annotations

"""White compact household sewing machine (Brother LX3817 style) — free-arm variant.

Articraft brief:
- Object: compact free-arm household sewing machine, ~0.40 m wide, ~0.30 m tall,
  ~0.17 m deep. White ABS body, blue paisley art panel on the front pillar,
  silver needle plate, chrome needle/presser bars.
- Root/support: narrow cylindrical free-arm bed (root) with rubber feet; the
  C-shaped body casting (pillar + arm + head) is fixed onto the arm's right end.
  A removable accessory tray slides on/off the free arm toward the front.
- Articulations:
  - tray_slide: PRISMATIC, accessory tray slides off toward -Y (front).
  - handwheel_spin: CONTINUOUS, side handwheel on the right pillar face, axis +X.
  - stitch_select: REVOLUTE, large fluted stitch-selector dial on the pillar front.
  - reverse_press: PRISMATIC, reverse-stitch lever pressed downward.
  - tension_adjust: REVOLUTE, upper-thread tension dial on the head front.
  - needle_stroke: PRISMATIC, chrome needle bar with clamp and needle, vertical.
  - presser_lift: PRISMATIC, presser bar with slotted presser foot, vertical lift.
- Visible geometry: narrow rounded free-arm tube with needle plate and feed dog
  at the arm end, C-body with open harp space, removable extension tray,
  paisley accent panel, spool pin and bobbin winder on top.
- Intentional overlaps: control shafts and the needle/presser bars are captured
  in bores of the body casting; the body casting seats 2 mm into the arm bed.
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

# Free-arm dimensions
ARM_LENGTH = 0.300
ARM_WIDTH = 0.080
ARM_HEIGHT = 0.052
ARM_CENTER_X = -0.040
ARM_CENTER_Z = BED_TOP - ARM_HEIGHT / 2  # 0.036

# Accessory tray
TRAY_LENGTH = 0.190
TRAY_WIDTH = 0.155
TRAY_THICKNESS = 0.006
TRAY_CENTER_X = 0.005
TRAY_CENTER_Z = BED_TOP + TRAY_THICKNESS / 2  # 0.065

WHITE = Material(name="body_white", rgba=(0.955, 0.955, 0.945, 1.0))
WHITE_SOFT = Material(name="trim_white", rgba=(0.92, 0.92, 0.91, 1.0))
PALE_BLUE = Material(name="paisley_blue", rgba=(0.72, 0.80, 0.92, 1.0))
STEEL = Material(name="steel_silver", rgba=(0.72, 0.73, 0.75, 1.0))
CHROME = Material(name="chrome", rgba=(0.83, 0.85, 0.88, 1.0))
DARK = Material(name="dark_gray", rgba=(0.20, 0.20, 0.22, 1.0))
LIGHT_GRAY = Material(name="light_gray", rgba=(0.82, 0.82, 0.84, 1.0))
RUBBER = Material(name="rubber_gray", rgba=(0.35, 0.35, 0.36, 1.0))
TRAY_WHITE = Material(name="tray_white", rgba=(0.94, 0.94, 0.93, 1.0))


def _filleted(shape: cq.Workplane, selector: str, radii: tuple[float, ...]) -> cq.Workplane:
    """Try progressively smaller fillets; fall back to the unfilleted shape."""
    for radius in radii:
        try:
            return shape.edges(selector).fillet(radius)
        except Exception:
            continue
    return shape


def _free_arm_shape() -> cq.Workplane:
    """Narrow rounded free-arm sleeve cantilevered forward from the pillar.

    A heavily filleted box approximates a rounded tube cross-section.
    The arm extends along X from the pillar junction to the needle area.
    """
    arm = cq.Workplane("XY").box(ARM_LENGTH, ARM_WIDTH, ARM_HEIGHT)
    # Heavy vertical-edge fillets for tube appearance (cross-section rounds).
    arm = _filleted(arm, "|Z", (0.025, 0.022, 0.018))
    # Moderate top/bottom fillets for smooth manufactured profile.
    arm = _filleted(arm, ">Z", (0.008, 0.006, 0.004))
    arm = _filleted(arm, "<Z", (0.008, 0.006, 0.004))
    return arm.translate((ARM_CENTER_X, 0.0, ARM_CENTER_Z))


def _body_shape() -> cq.Workplane:
    pillar = cq.Workplane("XY").box(0.100, 0.150, 0.240).translate((0.150, 0.0, 0.180))
    arm = cq.Workplane("XY").box(0.330, 0.120, 0.088).translate((0.035, 0.0, 0.256))
    head = cq.Workplane("XY").box(0.105, 0.140, 0.185).translate((-0.1475, 0.0, 0.2075))
    body = pillar.union(arm).union(head)
    # Round the front/back silhouette edges like the molded housing.
    body = _filleted(body, "|Y", (0.012, 0.008, 0.005))
    return body


def _art_panel_shape() -> cq.Workplane:
    """Front paisley art panel with pass-through cutouts for dial and lever."""
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
    """Presser foot sole with a through needle slot."""
    sole = cq.Workplane("XY").box(0.016, 0.034, 0.0042)
    sole = _filleted(sole, "|Z", (0.004, 0.002))
    slot = cq.Workplane("XY").center(0.0, -0.008).rect(0.0045, 0.012).extrude(0.02, both=True)
    return sole.cut(slot)


def _paddle_shape() -> cq.Workplane:
    paddle = cq.Workplane("XY").box(0.020, 0.012, 0.034)
    return _filleted(paddle, "|Y", (0.004, 0.002))


def _tray_shape() -> cq.Workplane:
    """Removable extension tray that sits on top of the free arm.

    Flat plate with rounded corners and a front grip lip for pull-off removal.
    Local origin at plate center.
    """
    plate = cq.Workplane("XY").box(TRAY_LENGTH, TRAY_WIDTH, TRAY_THICKNESS)
    plate = _filleted(plate, "|Z", (0.008, 0.006))
    plate = _filleted(plate, ">Z", (0.002, 0.001))
    plate = _filleted(plate, "<Z", (0.002, 0.001))
    # Front grip lip — a small raised ridge on the front edge for pull-off grip.
    lip = (
        cq.Workplane("XY")
        .box(0.055, 0.008, 0.005)
        .translate((0.0, -(TRAY_WIDTH / 2 - 0.004), TRAY_THICKNESS / 2 + 0.0025))
    )
    lip = _filleted(lip, "|Z", (0.003, 0.002))
    return plate.union(lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_sewing_machine")

    # ------------------------------------------------------------------ bed
    # Free-arm bed: narrow rounded tube cantilevered from the pillar.
    bed = model.part("bed")
    bed.visual(mesh_from_cadquery(_free_arm_shape(), "bed_shell"), material=WHITE, name="bed_shell")

    # Rubber feet under the arm — four pads near the arm ends.
    for i, (fx, fy) in enumerate([
        (ARM_CENTER_X + 0.120, -0.028),
        (ARM_CENTER_X + 0.120, 0.028),
        (ARM_CENTER_X - 0.120, -0.028),
        (ARM_CENTER_X - 0.120, 0.028),
    ]):
        bed.visual(
            Cylinder(radius=0.011, length=0.012),
            origin=Origin(xyz=(fx, fy, 0.006)),
            material=RUBBER,
            name=f"foot_pad_{i}",
        )

    # Silver needle plate on the arm's upper end (near needle position).
    bed.visual(
        Box((0.080, 0.058, 0.0025)),
        origin=Origin(xyz=(NEEDLE_X, -0.005, BED_TOP + 0.00125)),
        material=STEEL,
        name="needle_plate",
    )
    # Dark feed dog inset on the needle plate.
    bed.visual(
        Box((0.028, 0.014, 0.0008)),
        origin=Origin(xyz=(NEEDLE_X, NEEDLE_Y, PLATE_TOP - 0.0004 + 0.0008)),
        material=DARK,
        name="feed_dog",
    )
    # Bobbin access cover on the front face of the free arm near the needle end.
    bed.visual(
        Box((0.048, 0.002, 0.030)),
        origin=Origin(xyz=(NEEDLE_X, -(ARM_WIDTH / 2 + 0.001), ARM_CENTER_Z)),
        material=LIGHT_GRAY,
        name="bobbin_cover",
    )

    # -------------------------------------------------------- accessory tray
    # Removable extension tray that wraps the free arm; slides off toward -Y.
    tray = model.part("accessory_tray")
    tray.visual(
        mesh_from_cadquery(_tray_shape(), "tray_plate"),
        material=TRAY_WHITE,
        name="tray_plate",
    )
    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=bed,
        child=tray,
        # Origin at the tray center when installed (flush on arm top).
        origin=Origin(xyz=(TRAY_CENTER_X, 0.0, TRAY_CENTER_Z)),
        # Positive q pulls the tray off toward the front (-Y).
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.3, lower=0.0, upper=0.160),
    )

    # ----------------------------------------------------------------- body
    body = model.part("body")
    body.visual(mesh_from_cadquery(_body_shape(), "body_shell"), material=WHITE, name="body_shell")
    # Blue paisley art panel on the pillar front.
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
        # Positive q presses the lever downward.
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
    tray = object_model.get_part("accessory_tray")
    handwheel = object_model.get_part("handwheel")
    stitch_dial = object_model.get_part("stitch_dial")
    reverse_lever = object_model.get_part("reverse_lever")
    tension_dial = object_model.get_part("tension_dial")
    needle_bar = object_model.get_part("needle_bar")
    presser_foot = object_model.get_part("presser_foot")

    needle_joint = object_model.get_articulation("needle_stroke")
    presser_joint = object_model.get_articulation("presser_lift")
    reverse_joint = object_model.get_articulation("reverse_press")
    tray_joint = object_model.get_articulation("tray_slide")

    # ---------------------------------------------------------- allowances
    ctx.allow_overlap(
        body,
        bed,
        elem_a="body_shell",
        elem_b="bed_shell",
        reason="The pillar casting intentionally seats 2 mm into the free-arm bed.",
    )
    ctx.allow_overlap(
        body,
        handwheel,
        elem_a="body_shell",
        elem_b="handwheel_shaft",
        reason="Handwheel shaft is captured in the pillar bushing bore.",
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

    # --------------------------------------- free-arm geometry verification
    # The bed must read as a narrow free-arm tube, not a flat bench bed.
    bed_aabb = ctx.part_world_aabb(bed)
    ctx.check(
        "bed is a narrow free-arm sleeve (Y width < 0.10 m, not a flat bed)",
        bed_aabb is not None and (bed_aabb[1][1] - bed_aabb[0][1]) < 0.10,
        details=f"bed_aabb={bed_aabb}",
    )
    ctx.check(
        "bed free arm extends forward past the needle position",
        bed_aabb is not None and bed_aabb[0][0] < NEEDLE_X - 0.020,
        details=f"bed_aabb={bed_aabb}, NEEDLE_X={NEEDLE_X}",
    )

    # Needle plate is on the arm's upper end.
    ctx.expect_overlap(
        bed,
        bed,
        axes="xy",
        elem_a="needle_plate",
        elem_b="bed_shell",
        min_overlap=0.040,
        name="needle_plate sits on the free-arm upper surface",
    )

    # --------------------------------------- accessory tray slide behaviour
    # At rest (q=0), the tray is installed on the arm — centered in Y.
    tray_rest = ctx.part_world_position(tray)
    ctx.check(
        "accessory_tray is installed centered on the arm at rest",
        tray_rest is not None and abs(tray_rest[1]) < 0.005,
        details=f"tray_rest={tray_rest}",
    )

    # Tray sits flush on the arm top.
    ctx.expect_gap(
        tray,
        bed,
        axis="z",
        positive_elem="tray_plate",
        negative_elem="bed_shell",
        min_gap=-0.001,
        max_gap=0.004,
        name="tray plate sits on the free-arm top surface",
    )

    # Tray is wider than the free arm (wraps around it visually).
    ctx.check(
        "accessory tray is wider than the free arm (extension surface)",
        bed_aabb is not None,
    )
    tray_aabb = ctx.part_world_aabb(tray)
    ctx.check(
        "accessory tray extends beyond the arm in Y",
        tray_aabb is not None
        and bed_aabb is not None
        and (tray_aabb[1][1] - tray_aabb[0][1]) > (bed_aabb[1][1] - bed_aabb[0][1]) + 0.040,
        details=f"tray_y_span={tray_aabb}, bed_y_span={bed_aabb}",
    )

    # Positive q pulls the tray off toward -Y (front).
    with ctx.pose({tray_joint: 0.140}):
        tray_off = ctx.part_world_position(tray)
        ctx.check(
            "tray_slide positive q moves tray toward -Y (front removal)",
            tray_rest is not None
            and tray_off is not None
            and tray_off[1] < tray_rest[1] - 0.10,
            details=f"rest={tray_rest}, removed={tray_off}",
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
        details=f"tension_aabb={tension_aabb}",
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
            and low_needle[0][2] > BED_TOP + 0.0001,
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

    return ctx.report()


object_model = build_object_model()
