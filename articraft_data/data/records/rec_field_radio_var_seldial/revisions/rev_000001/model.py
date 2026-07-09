from __future__ import annotations

"""Rugged military-style handheld two-way radio (walkie-talkie).

Upright at z=0. Body slab 0.065 x 0.038 x 0.22 m: matte-black upper half with a
recessed green LCD and red/green LEDs, desert-tan lower half with a large rotary
channel selector dial recessed into the front face. Top face carries a knurled
rotary volume/channel knob (continuous) and a fold-over tapered whip antenna
(revolute 0-90 deg). Left side has two oblong rubber PTT buttons (prismatic,
3 mm press). Back carries a spring belt clip hinged at its top screw boss
(revolute 0-25 deg).

Frame conventions: X = width (left side at -X), Y = depth (front face at -Y,
back at +Y), Z = up.
"""

from math import pi

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------- dimensions
BODY_W = 0.065  # X
BODY_D = 0.038  # Y
BODY_H = 0.220  # Z
HALF_W = BODY_W / 2.0  # 0.0325
HALF_D = BODY_D / 2.0  # 0.019
SPLIT_Z = 0.128  # tan lower / black upper color split

FRONT_Y = -HALF_D
BACK_Y = HALF_D
LEFT_X = -HALF_W

# top-face hardware
KNOB_X = 0.018
KNOB_DIA = 0.018
KNOB_H = 0.015
KNOB_BASE_Z = 0.2245  # knob skirt bottom (1 mm above its boss)

ANT_X = -0.018
ANT_PIVOT_Z = 0.229  # top of antenna boss = fold pivot
ANT_COLLAR_H = 0.036
ANT_WHIP_LEN = 0.350

# LCD
LCD_CZ = 0.185
LCD_W = 0.046
LCD_H = 0.042

# channel selector dial (replaces keypad)
DIAL_CZ = 0.076  # center Z on the front face
DIAL_DIA = 0.048  # dial diameter
DIAL_R = DIAL_DIA / 2.0  # 0.024
DIAL_H = 0.012  # dial disc thickness along Y
DIAL_BEZEL_OUTER_R = DIAL_R + 0.004  # 0.028
DIAL_BEZEL_INNER_R = DIAL_R + 0.001  # 0.025
DIAL_BEZEL_DEPTH = 0.003

# PTT buttons (left side)
PTT_Z = (0.165, 0.142)  # upper, lower
PTT_TRAVEL = 0.003

# belt clip
CLIP_PIVOT_Y = 0.0225  # outer face of the clip boss
CLIP_PIVOT_Z = 0.168
CLIP_OPEN = 25.0 * pi / 180.0


# ---------------------------------------------------------------- cq shapes
def _rounded_slab(w: float, d: float, z0: float, z1: float, rad: float) -> cq.Workplane:
    slab = (
        cq.Workplane("XY")
        .box(w, d, z1 - z0, centered=(True, True, False))
        .edges("|Z")
        .fillet(rad)
    )
    return slab.translate((0.0, 0.0, z0))


def _lower_shell_shape() -> cq.Workplane:
    return _rounded_slab(BODY_W, BODY_D, 0.0, SPLIT_Z + 0.001, 0.006)


def _upper_shell_shape() -> cq.Workplane:
    shell = _rounded_slab(BODY_W, BODY_D, SPLIT_Z, BODY_H, 0.006)
    try:
        shell = shell.edges(">Z").fillet(0.0025)
    except Exception:
        pass
    return shell


def _lcd_bezel_shape() -> cq.Workplane:
    frame = cq.Workplane("XY").box(LCD_W, 0.004, LCD_H)
    window = cq.Workplane("XY").box(0.034, 0.012, 0.030)
    return frame.cut(window)


def _ptt_button_shape() -> cq.Workplane:
    # Oblong rubber cap (pill along Y) plus a hidden stem reaching into the body.
    cap = (
        cq.Workplane("XY")
        .box(0.007, 0.024, 0.013)
        .edges("|X")
        .fillet(0.0045)
        .translate((-0.0025, 0.0, 0.0))
    )
    stem = cq.Workplane("XY").box(0.008, 0.014, 0.008).translate((0.003, 0.0, 0.0))
    return cap.union(stem)


def _belt_clip_shape() -> cq.Workplane:
    # Child frame: origin at the top screw pivot on the boss outer face,
    # +Y away from the body, clip blade hanging toward -Z.
    blade = (
        cq.Workplane("XY")
        .box(0.020, 0.0025, 0.108)
        .translate((0.0, 0.00175, -0.043))  # y: 0.0005..0.003, z: 0.011..-0.097
    )
    tip = (
        cq.Workplane("XY")
        .box(0.020, 0.0025, 0.016)
        .rotate((0, 0, 0), (1, 0, 0), 20.0)
        .translate((0.0, 0.0044, -0.1015))
    )
    screw = (
        cq.Workplane("XY")
        .cylinder(0.0035, 0.004, centered=(True, True, False))
        .rotate((0, 0, 0), (1, 0, 0), -90.0)
        .translate((0.0, -0.0008, 0.0))  # y: -0.0008..0.0027, embeds into boss
    )
    return blade.union(tip).union(screw)


def _antenna_shape() -> cq.Workplane:
    # Child frame: origin at the fold pivot, antenna up along +Z when q=0.
    collar = cq.Workplane("XY").cylinder(
        ANT_COLLAR_H + 0.001, 0.0065, centered=(True, True, False)
    ).translate((0.0, 0.0, -0.001))
    for rib_z in (0.004, 0.012, 0.020, 0.028):
        rib = (
            cq.Workplane("XY")
            .cylinder(0.004, 0.008, centered=(True, True, False))
            .translate((0.0, 0.0, rib_z))
        )
        collar = collar.union(rib)
    whip = cq.Workplane("XY").add(
        cq.Solid.makeCone(0.0045, 0.0016, ANT_WHIP_LEN)
    ).translate((0.0, 0.0, ANT_COLLAR_H - 0.002))
    tip = cq.Workplane("XY").sphere(0.0024).translate(
        (0.0, 0.0, ANT_COLLAR_H - 0.002 + ANT_WHIP_LEN)
    )
    return collar.union(whip).union(tip)


def _dial_bezel_shape() -> cq.Workplane:
    """Annular bezel ring on the front face framing the dial pocket.

    Built on the XZ workplane (normal = -Y), so positive extrude goes in -Y
    (outward from the front face).
    """
    outer = cq.Workplane("XZ").circle(DIAL_BEZEL_OUTER_R).extrude(DIAL_BEZEL_DEPTH)
    inner = cq.Workplane("XZ").circle(DIAL_BEZEL_INNER_R).extrude(DIAL_BEZEL_DEPTH + 0.001)
    ring = outer.cut(inner)
    return ring.translate((0.0, FRONT_Y, DIAL_CZ))


# ---------------------------------------------------------------- model
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="military_handheld_radio")

    model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("graphite", rgba=(0.14, 0.14, 0.15, 1.0))
    model.material("desert_tan", rgba=(0.74, 0.63, 0.43, 1.0))
    model.material("dial_tan", rgba=(0.80, 0.70, 0.48, 1.0))
    model.material("panel_tan", rgba=(0.68, 0.57, 0.38, 1.0))
    model.material("rubber_black", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("lcd_green", rgba=(0.62, 0.78, 0.25, 1.0))
    model.material("led_red", rgba=(0.85, 0.10, 0.08, 1.0))
    model.material("led_green", rgba=(0.15, 0.80, 0.20, 1.0))
    model.material("clip_steel", rgba=(0.17, 0.18, 0.20, 1.0))

    # ---- body (root) ----
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_lower_shell_shape(), "lower_shell", tolerance=0.0004),
        material="desert_tan",
        name="lower_shell",
    )
    body.visual(
        mesh_from_cadquery(_upper_shell_shape(), "upper_shell", tolerance=0.0004),
        material="matte_black",
        name="upper_shell",
    )

    # LCD: raised black bezel frame with a slightly recessed green screen.
    body.visual(
        mesh_from_cadquery(_lcd_bezel_shape(), "lcd_bezel", tolerance=0.0003),
        origin=Origin(xyz=(0.0, FRONT_Y, LCD_CZ)),  # frame y: -0.021..-0.017
        material="graphite",
        name="lcd_bezel",
    )
    body.visual(
        Box((0.034, 0.002, 0.030)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.0005, LCD_CZ)),  # face at -0.0205
        material="lcd_green",
        name="lcd_screen",
    )

    # Indicator LEDs beside the LCD (red above green), proud of the front face.
    body.visual(
        Cylinder(radius=0.0016, length=0.003),
        origin=Origin(xyz=(0.027, FRONT_Y - 0.0005, 0.198), rpy=(pi / 2.0, 0.0, 0.0)),
        material="led_red",
        name="led_red",
    )
    body.visual(
        Cylinder(radius=0.0016, length=0.003),
        origin=Origin(xyz=(0.027, FRONT_Y - 0.0005, 0.188), rpy=(pi / 2.0, 0.0, 0.0)),
        material="led_green",
        name="led_green",
    )

    # Channel selector dial recess: bezel ring + dark well on the front face.
    body.visual(
        mesh_from_cadquery(_dial_bezel_shape(), "dial_bezel", tolerance=0.0003),
        material="panel_tan",
        name="dial_bezel",
    )
    body.visual(
        Cylinder(radius=DIAL_BEZEL_INNER_R, length=0.008),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.004, DIAL_CZ), rpy=(pi / 2.0, 0.0, 0.0)),
        material="graphite",
        name="dial_well",
    )

    # Top-face mounting bosses.
    body.visual(
        Cylinder(radius=0.0095, length=0.005),
        origin=Origin(xyz=(KNOB_X, 0.0, BODY_H + 0.0015)),  # z: 0.219..0.224
        material="graphite",
        name="knob_boss",
    )
    body.visual(
        Cylinder(radius=0.0085, length=0.010),
        origin=Origin(xyz=(ANT_X, 0.0, BODY_H + 0.004)),  # z: 0.219..0.229
        material="graphite",
        name="antenna_boss",
    )

    # Back boss carrying the belt-clip screw.
    body.visual(
        Box((0.022, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, 0.0205, CLIP_PIVOT_Z)),  # y: 0.0185..0.0225
        material="matte_black",
        name="clip_boss",
    )

    # ---- volume/channel knob (continuous) ----
    knob = model.part("volume_knob")
    knob_geom = KnobGeometry(
        KNOB_DIA,
        KNOB_H,
        body_style="cylindrical",
        grip=KnobGrip(style="knurled", count=32, depth=0.0008, helix_angle_deg=18.0),
        indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
        center=False,
    )
    knob.visual(
        mesh_from_geometry(knob_geom, "volume_knob_cap"),
        material="rubber_black",
        name="knob_cap",
    )
    # Hidden shaft bridging knob to the boss/body (intentional embedding).
    knob.visual(
        Cylinder(radius=0.0035, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),  # child z: -0.009..0.001
        material="graphite",
        name="knob_shaft",
    )
    # Off-axis pointer nub on the knob top proving continuous rotation.
    knob.visual(
        Box((0.0024, 0.0024, 0.0014)),
        origin=Origin(xyz=(0.0, -0.0062, KNOB_H + 0.0005)),
        material="keypad_tan" if False else "dial_tan",
        name="knob_pointer",
    )
    model.articulation(
        "knob_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(KNOB_X, 0.0, KNOB_BASE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=6.0),
    )

    # ---- channel selector dial (continuous, front face, Y-axis) ----
    dial = model.part("channel_dial")
    # Wide detented disc: KnobGeometry builds along Z; rotate +90° about X
    # so the cylinder axis becomes Y and the indicator face points outward (-Y).
    dial_geom = KnobGeometry(
        DIAL_DIA,
        DIAL_H,
        body_style="faceted",
        base_diameter=DIAL_DIA + 0.002,
        grip=KnobGrip(style="fluted", count=16, depth=0.003),
        indicator=KnobIndicator(style="wedge", mode="raised", angle_deg=0.0),
        center=False,
    )
    dial.visual(
        mesh_from_geometry(dial_geom, "channel_dial_disc"),
        origin=Origin(xyz=(0.0, DIAL_H / 2.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="dial_tan",
        name="dial_body",
    )
    # Shaft reaching from the dial back face into the body housing.
    dial.visual(
        Cylinder(radius=0.005, length=0.018),
        origin=Origin(xyz=(0.0, 0.015, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="graphite",
        name="dial_shaft",
    )
    # Off-axis pointer nub on the front face near the top edge of the dial,
    # proving continuous rotation sweeps the marker around the dial face.
    dial.visual(
        Box((0.004, 0.002, 0.004)),
        origin=Origin(xyz=(0.0, -DIAL_H / 2.0 - 0.001, DIAL_R - 0.006)),
        material="led_red",
        name="dial_pointer",
    )
    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, FRONT_Y, DIAL_CZ)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0),
    )

    # ---- whip antenna (fold-over revolute) ----
    antenna = model.part("whip_antenna")
    antenna.visual(
        mesh_from_cadquery(_antenna_shape(), "whip_antenna", tolerance=0.0002),
        material="rubber_black",
        name="antenna_body",
    )
    model.articulation(
        "antenna_fold",
        ArticulationType.REVOLUTE,
        parent=body,
        child=antenna,
        origin=Origin(xyz=(ANT_X, 0.0, ANT_PIVOT_Z)),
        # -Y axis: positive q folds the whip outboard toward -X, away from the knob.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=pi / 2.0),
    )

    # ---- PTT buttons (prismatic, press inward) ----
    ptt_mesh = mesh_from_cadquery(_ptt_button_shape(), "ptt_button", tolerance=0.0003)
    ptt_specs = (("ptt_button_upper", PTT_Z[0]), ("ptt_button_lower", PTT_Z[1]))
    for ptt_name, ptt_z in ptt_specs:
        ptt = model.part(ptt_name)
        ptt.visual(ptt_mesh, material="rubber_black", name="ptt_cap")
        model.articulation(
            ptt_name.replace("button", "press"),
            ArticulationType.PRISMATIC,
            parent=body,
            child=ptt,
            origin=Origin(xyz=(LEFT_X, 0.0, ptt_z)),
            axis=(1.0, 0.0, 0.0),  # positive q presses inward
            motion_limits=MotionLimits(effort=3.0, velocity=0.1, lower=0.0, upper=PTT_TRAVEL),
        )

    # ---- belt clip (revolute at top screw) ----
    clip = model.part("belt_clip")
    clip.visual(
        mesh_from_cadquery(_belt_clip_shape(), "belt_clip", tolerance=0.0003),
        material="clip_steel",
        name="clip_blade",
    )
    model.articulation(
        "clip_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=clip,
        origin=Origin(xyz=(0.0, CLIP_PIVOT_Y, CLIP_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),  # positive q swings the lower tip away from the back
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=CLIP_OPEN),
    )

    return model


# ---------------------------------------------------------------- tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    knob = object_model.get_part("volume_knob")
    dial = object_model.get_part("channel_dial")
    antenna = object_model.get_part("whip_antenna")
    clip = object_model.get_part("belt_clip")
    ptt_upper = object_model.get_part("ptt_button_upper")
    ptt_lower = object_model.get_part("ptt_button_lower")

    knob_spin = object_model.get_articulation("knob_spin")
    dial_spin = object_model.get_articulation("dial_spin")
    antenna_fold = object_model.get_articulation("antenna_fold")
    clip_hinge = object_model.get_articulation("clip_hinge")
    ptt_press_upper = object_model.get_articulation("ptt_press_upper")
    ptt_press_lower = object_model.get_articulation("ptt_press_lower")

    # ---- intentional embeddings ----
    ctx.allow_overlap(
        knob, body, reason="knob shaft seats through the top boss into the housing",
        elem_a="knob_shaft", elem_b="knob_boss",
    )
    ctx.allow_overlap(
        knob, body, reason="knob shaft passes into the upper housing shell",
        elem_a="knob_shaft", elem_b="upper_shell",
    )
    ctx.allow_overlap(
        antenna, body, reason="antenna collar seats 0.5 mm into its mounting boss",
        elem_a="antenna_body", elem_b="antenna_boss",
    )
    ctx.allow_overlap(
        clip, body, reason="clip screw head embeds into the back screw boss",
        elem_a="clip_blade", elem_b="clip_boss",
    )
    for ptt in (ptt_upper, ptt_lower):
        ctx.allow_overlap(
            ptt, body, reason="PTT cap rim and stem pass through the housing wall",
            elem_a="ptt_cap", elem_b="upper_shell",
        )
        ctx.allow_overlap(
            ptt, body, reason="PTT stem may cross the housing color split line",
            elem_a="ptt_cap", elem_b="lower_shell",
        )

    # Dial: the disc rear half and shaft are recessed into the lower shell pocket.
    ctx.allow_overlap(
        dial, body, reason="dial disc rear half is recessed into the front-face pocket",
        elem_a="dial_body", elem_b="lower_shell",
    )
    ctx.allow_overlap(
        dial, body, reason="dial shaft passes through the front-face pocket into the housing",
        elem_a="dial_shaft", elem_b="lower_shell",
    )
    ctx.allow_overlap(
        dial, body, reason="dial well sits inside the body pocket behind the dial disc",
        elem_a="dial_well", elem_b="lower_shell",
    )
    ctx.allow_overlap(
        body, dial, reason="dial well cylinder nests behind the dial disc inside the pocket",
        elem_a="dial_well", elem_b="dial_body",
    )

    # ---- joint plan: types, axes, limits ----
    ctx.check(
        "knob_is_continuous_vertical",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(knob_spin.axis) == (0.0, 0.0, 1.0),
        f"type={knob_spin.articulation_type}, axis={knob_spin.axis}",
    )
    ctx.check(
        "dial_is_continuous_front_axis",
        dial_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(dial_spin.axis) == (0.0, 1.0, 0.0),
        f"type={dial_spin.articulation_type}, axis={dial_spin.axis}",
    )
    ctx.check(
        "antenna_fold_revolute_0_to_90deg",
        antenna_fold.articulation_type == ArticulationType.REVOLUTE
        and abs(antenna_fold.motion_limits.lower - 0.0) < 1e-9
        and abs(antenna_fold.motion_limits.upper - pi / 2.0) < 1e-6,
        f"limits=({antenna_fold.motion_limits.lower}, {antenna_fold.motion_limits.upper})",
    )
    ctx.check(
        "clip_hinge_revolute_0_to_25deg",
        clip_hinge.articulation_type == ArticulationType.REVOLUTE
        and abs(clip_hinge.motion_limits.upper - CLIP_OPEN) < 1e-6,
        f"upper={clip_hinge.motion_limits.upper}",
    )
    for joint in (ptt_press_upper, ptt_press_lower):
        ctx.check(
            f"{joint.name}_prismatic_3mm_inward",
            joint.articulation_type == ArticulationType.PRISMATIC
            and abs(joint.motion_limits.upper - PTT_TRAVEL) < 1e-9
            and tuple(joint.axis) == (1.0, 0.0, 0.0),
            f"axis={joint.axis}, upper={joint.motion_limits.upper}",
        )

    # ---- body scale and grounding ----
    body_aabb = ctx.part_world_aabb(body)
    (bx0, by0, bz0), (bx1, by1, bz1) = body_aabb
    ctx.check(
        "body_true_scale_and_grounded",
        abs((bx1 - bx0) - BODY_W) < 0.004
        and abs(bz1 - BODY_H) < 0.012  # bosses extend slightly above 0.22
        and abs(bz0) < 1e-4,
        f"body aabb={body_aabb}",
    )

    # ---- LCD recessed in its bezel, both proud of the housing front ----
    bezel_aabb = ctx.part_element_world_aabb(body, elem="lcd_bezel")
    screen_aabb = ctx.part_element_world_aabb(body, elem="lcd_screen")
    ctx.check(
        "lcd_screen_recessed_in_bezel",
        screen_aabb[0][1] > bezel_aabb[0][1] and screen_aabb[0][1] < FRONT_Y,
        f"screen front y={screen_aabb[0][1]}, bezel front y={bezel_aabb[0][1]}",
    )

    # ---- channel dial: on front face, within body width, pointer sweeps ----
    dial_aabb = ctx.part_world_aabb(dial)
    ctx.check(
        "dial_on_front_face_lower_half",
        dial_aabb[0][1] < FRONT_Y  # protrudes in front of body
        and dial_aabb[0][2] > 0.040  # in the lower half
        and dial_aabb[1][2] < SPLIT_Z,  # below the color split
        f"dial aabb={dial_aabb}",
    )
    ctx.check(
        "dial_wide_enough",
        (dial_aabb[1][0] - dial_aabb[0][0]) > 0.040,
        f"dial x span={dial_aabb[1][0] - dial_aabb[0][0]}",
    )
    ctx.expect_within(dial, body, axes="x", margin=0.002, name="dial_fits_within_body_width")

    # Bezel ring frames the dial on the front face.
    bezel_dial_aabb = ctx.part_element_world_aabb(body, elem="dial_bezel")
    ctx.check(
        "dial_bezel_on_front_face",
        bezel_dial_aabb[0][1] < FRONT_Y,
        f"bezel front y={bezel_dial_aabb[0][1]}",
    )

    # Off-axis pointer proves continuous rotation about the Y axis.
    p0 = ctx.part_element_world_aabb(dial, elem="dial_pointer")
    z0 = 0.5 * (p0[0][2] + p0[1][2])
    x0 = 0.5 * (p0[0][0] + p0[1][0])
    with ctx.pose({dial_spin: pi / 2.0}):
        p1 = ctx.part_element_world_aabb(dial, elem="dial_pointer")
        z1 = 0.5 * (p1[0][2] + p1[1][2])
        x1 = 0.5 * (p1[0][0] + p1[1][0])
    ctx.check(
        "dial_pointer_sweeps_off_axis",
        abs(z1 - z0) > 0.010 or abs(x1 - x0) > 0.010,
        f"pointer at q=0: x={x0}, z={z0}; at q=pi/2: x={x1}, z={z1}",
    )

    # ---- LEDs sit beside the LCD on the front face ----
    led_red_aabb = ctx.part_element_world_aabb(body, elem="led_red")
    ctx.check(
        "red_led_beside_lcd",
        led_red_aabb[0][0] > 0.023 and led_red_aabb[0][2] > 0.18,
        f"led_red aabb={led_red_aabb}",
    )

    # ---- knob on the top face, beside the antenna ----
    knob_aabb = ctx.part_world_aabb(knob)
    ctx.check(
        "knob_on_top_face",
        knob_aabb[0][2] > 0.210 and knob_aabb[1][2] < 0.245,
        f"knob aabb={knob_aabb}",
    )
    ctx.expect_overlap(knob, body, axes="xy", min_overlap=0.012)

    # Off-axis pointer proves continuous rotation about the vertical axis.
    kp0 = ctx.part_element_world_aabb(knob, elem="knob_pointer")
    ky0 = 0.5 * (kp0[0][1] + kp0[1][1])
    with ctx.pose({knob_spin: pi}):
        kp1 = ctx.part_element_world_aabb(knob, elem="knob_pointer")
        ky1 = 0.5 * (kp1[0][1] + kp1[1][1])
    ctx.check(
        "knob_pointer_sweeps_off_axis",
        abs(ky1 - ky0) > 0.010,
        f"pointer y at q=0: {ky0}, at q=pi: {ky1}",
    )

    # ---- antenna: upright at rest, folds flat outboard ----
    ant_aabb = ctx.part_world_aabb(antenna)
    ctx.check(
        "antenna_upright_total_height",
        0.55 < ant_aabb[1][2] < 0.66,
        f"antenna aabb={ant_aabb}",
    )
    with ctx.pose({antenna_fold: pi / 2.0}):
        folded = ctx.part_world_aabb(antenna)
        ctx.check(
            "antenna_folds_horizontal_outboard",
            folded[1][2] < 0.27 and folded[0][0] < -0.33,
            f"folded antenna aabb={folded}",
        )
        ctx.check(
            "folded_antenna_clears_housing_top",
            folded[0][2] > BODY_H - 0.0005,
            f"folded antenna zmin={folded[0][2]}, housing top z={BODY_H}",
        )

    # ---- PTT: proud of left face at rest, presses 3 mm inward ----
    cap0 = ctx.part_world_aabb(ptt_upper)
    ctx.check(
        "ptt_proud_of_left_face",
        cap0[0][0] < LEFT_X - 0.004,
        f"ptt aabb={cap0}, left face x={LEFT_X}",
    )
    with ctx.pose({ptt_press_upper: PTT_TRAVEL}):
        cap1 = ctx.part_world_aabb(ptt_upper)
        ctx.check(
            "ptt_presses_inward_3mm",
            abs((cap1[0][0] - cap0[0][0]) - PTT_TRAVEL) < 1e-6
            and cap1[0][0] < LEFT_X,  # still proud when fully pressed
            f"rest xmin={cap0[0][0]}, pressed xmin={cap1[0][0]}",
        )

    # ---- belt clip: hugs the back at rest, swings open by its top screw ----
    clip0 = ctx.part_world_aabb(clip)
    ctx.check(
        "clip_hugs_back_panel",
        BACK_Y < clip0[1][1] < BACK_Y + 0.012 and clip0[0][2] < 0.075,
        f"clip aabb={clip0}",
    )
    with ctx.pose({clip_hinge: CLIP_OPEN}):
        clip1 = ctx.part_world_aabb(clip)
        ctx.check(
            "clip_tip_swings_away_from_back",
            clip1[1][1] > clip0[1][1] + 0.030,
            f"rest ymax={clip0[1][1]}, open ymax={clip1[1][1]}",
        )

    return ctx.report()


object_model = build_object_model()
