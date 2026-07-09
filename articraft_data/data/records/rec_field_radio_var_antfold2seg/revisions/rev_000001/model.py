from __future__ import annotations

"""Rugged military-style handheld two-way radio (walkie-talkie).

Upright at z=0. Body slab 0.065 x 0.038 x 0.22 m: matte-black upper half with a
recessed green LCD and red/green LEDs, desert-tan lower half with a 4x4 rubber
keypad. Top face carries a knurled rotary volume/channel knob (continuous) and
a fold-over tapered whip antenna (revolute 0-90 deg). Left side has two oblong
rubber PTT buttons (prismatic, 3 mm press). Back carries a spring belt clip
hinged at its top screw boss (revolute 0-25 deg).

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
ANT_PIVOT_Z = 0.229  # top of antenna boss = body fold pivot
ANT_COLLAR_H = 0.036
ANT_BASE_LEN = 0.080  # base segment total height above body pivot
ANT_ELBOW_Z = ANT_BASE_LEN  # elbow pivot at top of base segment (local frame)
ANT_WHIP_LEN = 0.280  # whip segment length above elbow

# LCD
LCD_CZ = 0.185
LCD_W = 0.046
LCD_H = 0.042

# keypad
KEY_COLS_X = (-0.018, -0.006, 0.006, 0.018)
KEY_ROWS_Z = (0.0535, 0.0685, 0.0835, 0.0985)

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


def _antenna_base_shape() -> cq.Workplane:
    """Short rigid base segment: ribbed collar, tube stub, and elbow pivot housing.

    Child frame: origin at the body-to-base fold pivot, base extends +Z.
    """
    # Ribbed collar (same as original lower portion).
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
    # Rigid tube stub running from collar top through the pivot housing.
    # Height spans from collar top to pivot top, ensuring one connected solid.
    stub_h = ANT_BASE_LEN - ANT_COLLAR_H + 0.008
    stub = cq.Workplane("XY").cylinder(
        stub_h, 0.0045, centered=(True, True, False)
    ).translate((0.0, 0.0, ANT_COLLAR_H))
    # Elbow pivot housing: a barrel at the top of the base segment.
    pivot_h = 0.016
    pivot = (
        cq.Workplane("XY")
        .cylinder(pivot_h, 0.007, centered=(True, True, False))
        .translate((0.0, 0.0, ANT_BASE_LEN - pivot_h / 2.0))
    )
    return collar.union(stub).union(pivot)


def _antenna_whip_shape() -> cq.Workplane:
    """Longer tapered whip segment with pivot collar at its base.

    Child frame: origin at the elbow pivot, whip extends +Z when q=0.
    """
    # Pivot collar wrapping around the base segment elbow housing.
    collar = cq.Workplane("XY").cylinder(
        0.014, 0.0075, centered=(True, True, False)
    ).translate((0.0, 0.0, -0.007))
    # Tapered whip above the collar.
    whip_start_z = 0.004
    whip = cq.Workplane("XY").add(
        cq.Solid.makeCone(0.0042, 0.0015, ANT_WHIP_LEN)
    ).translate((0.0, 0.0, whip_start_z))
    tip = cq.Workplane("XY").sphere(0.0022).translate(
        (0.0, 0.0, whip_start_z + ANT_WHIP_LEN)
    )
    return collar.union(whip).union(tip)


# ---------------------------------------------------------------- model
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="military_handheld_radio")

    model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("graphite", rgba=(0.14, 0.14, 0.15, 1.0))
    model.material("desert_tan", rgba=(0.74, 0.63, 0.43, 1.0))
    model.material("keypad_tan", rgba=(0.83, 0.75, 0.55, 1.0))
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

    # Keypad: tan inset panel with 16 raised rubber keys (4x4).
    body.visual(
        Box((0.054, 0.003, 0.062)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.0005, 0.076)),  # face at -0.021
        material="panel_tan",
        name="keypad_panel",
    )
    for row, key_z in enumerate(KEY_ROWS_Z):
        for col, key_x in enumerate(KEY_COLS_X):
            body.visual(
                Box((0.0095, 0.003, 0.0075)),
                origin=Origin(xyz=(key_x, FRONT_Y - 0.0025, key_z)),  # proud of panel
                material="keypad_tan",
                name=f"key_{row}_{col}",
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
        material="keypad_tan",
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

    # ---- two-segment folding antenna ----
    # Base segment: short rigid stub rising from the top boss, with a ribbed
    # collar and an elbow pivot housing at the top.
    antenna_base = model.part("antenna_base")
    antenna_base.visual(
        mesh_from_cadquery(_antenna_base_shape(), "antenna_base", tolerance=0.0002),
        material="rubber_black",
        name="base_segment",
    )
    model.articulation(
        "antenna_fold",
        ArticulationType.REVOLUTE,
        parent=body,
        child=antenna_base,
        origin=Origin(xyz=(ANT_X, 0.0, ANT_PIVOT_Z)),
        # -Y axis: positive q folds the base outboard toward -X, away from the knob.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=pi / 2.0),
    )

    # Whip segment: longer tapered whip joined to the base by an elbow pivot.
    antenna_whip = model.part("antenna_whip")
    antenna_whip.visual(
        mesh_from_cadquery(_antenna_whip_shape(), "antenna_whip", tolerance=0.0002),
        material="rubber_black",
        name="whip_segment",
    )
    model.articulation(
        "elbow_fold",
        ArticulationType.REVOLUTE,
        parent=antenna_base,
        child=antenna_whip,
        # Elbow pivot at top of the base segment (in base part local frame).
        origin=Origin(xyz=(0.0, 0.0, ANT_ELBOW_Z)),
        # -Y axis: positive q folds the whip down against the base toward -X.
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
    antenna_base = object_model.get_part("antenna_base")
    antenna_whip = object_model.get_part("antenna_whip")
    clip = object_model.get_part("belt_clip")
    ptt_upper = object_model.get_part("ptt_button_upper")
    ptt_lower = object_model.get_part("ptt_button_lower")

    knob_spin = object_model.get_articulation("knob_spin")
    antenna_fold = object_model.get_articulation("antenna_fold")
    elbow_fold = object_model.get_articulation("elbow_fold")
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
        antenna_base, body, reason="antenna base collar seats 0.5 mm into its mounting boss",
        elem_a="base_segment", elem_b="antenna_boss",
    )
    ctx.allow_overlap(
        antenna_whip, antenna_base, reason="whip pivot collar wraps around the base elbow housing",
        elem_a="whip_segment", elem_b="base_segment",
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

    # ---- joint plan: types, axes, limits ----
    ctx.check(
        "knob_is_continuous_vertical",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(knob_spin.axis) == (0.0, 0.0, 1.0),
        f"type={knob_spin.articulation_type}, axis={knob_spin.axis}",
    )
    ctx.check(
        "antenna_fold_revolute_0_to_90deg",
        antenna_fold.articulation_type == ArticulationType.REVOLUTE
        and abs(antenna_fold.motion_limits.lower - 0.0) < 1e-9
        and abs(antenna_fold.motion_limits.upper - pi / 2.0) < 1e-6,
        f"limits=({antenna_fold.motion_limits.lower}, {antenna_fold.motion_limits.upper})",
    )
    ctx.check(
        "elbow_fold_revolute_0_to_90deg",
        elbow_fold.articulation_type == ArticulationType.REVOLUTE
        and abs(elbow_fold.motion_limits.lower - 0.0) < 1e-9
        and abs(elbow_fold.motion_limits.upper - pi / 2.0) < 1e-6,
        f"limits=({elbow_fold.motion_limits.lower}, {elbow_fold.motion_limits.upper})",
    )
    # Both antenna joints must be non-fixed revolute.
    for joint in (antenna_fold, elbow_fold):
        ctx.check(
            f"{joint.name}_is_nonfixed_revolute",
            joint.articulation_type == ArticulationType.REVOLUTE
            and joint.motion_limits.lower != joint.motion_limits.upper,
            f"type={joint.articulation_type}, limits=({joint.motion_limits.lower}, {joint.motion_limits.upper})",
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

    # ---- keypad: 16 raised keys proud of the panel ----
    missing = [
        f"key_{r}_{c}"
        for r in range(4)
        for c in range(4)
        if body.get_visual(f"key_{r}_{c}") is None
    ]
    ctx.check("keypad_has_16_keys", not missing, f"missing={missing}")
    key_aabb = ctx.part_element_world_aabb(body, elem="key_0_0")
    panel_aabb = ctx.part_element_world_aabb(body, elem="keypad_panel")
    ctx.check(
        "keys_proud_of_keypad_panel",
        key_aabb[0][1] < panel_aabb[0][1],
        f"key front y={key_aabb[0][1]}, panel front y={panel_aabb[0][1]}",
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
    p0 = ctx.part_element_world_aabb(knob, elem="knob_pointer")
    y0 = 0.5 * (p0[0][1] + p0[1][1])
    with ctx.pose({knob_spin: pi}):
        p1 = ctx.part_element_world_aabb(knob, elem="knob_pointer")
        y1 = 0.5 * (p1[0][1] + p1[1][1])
    ctx.check(
        "knob_pointer_sweeps_off_axis",
        abs(y1 - y0) > 0.010,
        f"pointer y at q=0: {y0}, at q=pi: {y1}",
    )

    # ---- antenna: two-segment folding mechanism ----
    # Both segments upright at rest (q=0 for both joints).
    base_aabb = ctx.part_world_aabb(antenna_base)
    whip_aabb = ctx.part_world_aabb(antenna_whip)
    ctx.check(
        "antenna_base_upright_on_top_face",
        base_aabb[0][2] > 0.22 and base_aabb[1][2] < 0.35,
        f"base aabb={base_aabb}",
    )
    ctx.check(
        "antenna_whip_extends_above_base",
        whip_aabb[0][2] > base_aabb[0][2] + 0.04 and whip_aabb[1][2] < 0.62,
        f"whip aabb={whip_aabb}, base aabb={base_aabb}",
    )
    # Whip is centered on the base segment in XY and its tip extends well above.
    ctx.expect_overlap(
        antenna_whip, antenna_base, axes="xy",
        min_overlap=0.006,
        name="whip_centered_on_base_in_xy",
    )

    # Fold the base segment (whip follows as child).
    with ctx.pose({antenna_fold: pi / 2.0}):
        folded_base = ctx.part_world_aabb(antenna_base)
        folded_whip = ctx.part_world_aabb(antenna_whip)
        ctx.check(
            "antenna_base_folds_outboard",
            folded_base[0][0] < ANT_X - 0.02,
            f"folded base aabb={folded_base}",
        )
        ctx.check(
            "whip_follows_base_fold",
            folded_whip[0][0] < ANT_X - 0.02,
            f"folded whip aabb={folded_whip}",
        )

    # Fold just the elbow (base stays upright, whip folds down against base).
    with ctx.pose({elbow_fold: pi / 2.0}):
        elbow_folded_whip = ctx.part_world_aabb(antenna_whip)
        elbow_folded_base = ctx.part_world_aabb(antenna_base)
        ctx.check(
            "elbow_folds_whip_down_against_base",
            elbow_folded_whip[0][0] < elbow_folded_base[0][0] - 0.02,
            f"whip aabb={elbow_folded_whip}, base aabb={elbow_folded_base}",
        )
        ctx.check(
            "elbow_folded_whip_stays_near_base_top",
            elbow_folded_whip[1][2] < elbow_folded_base[1][2] + 0.02,
            f"whip top z={elbow_folded_whip[1][2]}, base top z={elbow_folded_base[1][2]}",
        )

    # Both segments folded (base at 90°, elbow at 90°).
    with ctx.pose({antenna_fold: pi / 2.0, elbow_fold: pi / 2.0}):
        full_fold_base = ctx.part_world_aabb(antenna_base)
        full_fold_whip = ctx.part_world_aabb(antenna_whip)
        ctx.check(
            "full_fold_both_segments_compact",
            full_fold_base[1][2] < 0.30 and full_fold_whip[1][2] < 0.30,
            f"base aabb={full_fold_base}, whip aabb={full_fold_whip}",
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
