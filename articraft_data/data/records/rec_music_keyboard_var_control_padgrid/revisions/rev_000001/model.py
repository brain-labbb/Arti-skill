from __future__ import annotations

# Drum-pad synthesizer keyboard variant:
# A flat panel with a 4x4 grid of 16 square backlit drum pads (prismatic
# press-down joints into a raised bezel), a 2-octave (15 white + 10 black)
# hinged keybed at the front, four envelope sliders on the right, and a
# pitch/mod touch-strip block at the front-left.

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Global layout (meters). X = width (+right), Y = depth (+back), Z = up.
# ---------------------------------------------------------------------------
BODY_HALF_W = 0.25
BODY_FRONT_Y = -0.155
BODY_BACK_Y = 0.155

BASE_TOP_Z = 0.040
PANEL_FRONT_Y = -0.035
PANEL_TOP_Z = 0.085

KEY_PITCH = 0.024
WHITE_KEY_COUNT = 15  # two octaves C..C
WHITE_KEY_WIDTH = 0.0225
WHITE_KEY_DEPTH = 0.123  # hinge line to front edge
WHITE_KEY_THICK = 0.012
WHITE_TOP_Z = 0.062
KEY_HINGE_Y = -0.030  # hinge line tucked under the panel lip
RELIEF_SHOULDER = -0.085  # local y of the black-key notch shoulder
RELIEF_HALF_X = 0.0057  # white-key tail half width on a notched side

FIRST_WHITE_X = -0.118
BLACK_AFTER_WHITE = (0, 1, 3, 4, 5, 7, 8, 10, 11, 12)

BLACK_KEY_WIDTH = 0.011
BLACK_KEY_THICK = 0.0155
BLACK_BOTTOM_Z = 0.058

KEY_PRESS_RAD = 0.06

# ---------------------------------------------------------------------------
# Pad grid layout: 4 rows x 4 columns of square backlit drum pads.
# ---------------------------------------------------------------------------
PAD_SIZE = 0.032       # 32mm square pad face
PAD_GAP = 0.006        # 6mm gap between pads
PAD_PITCH = PAD_SIZE + PAD_GAP  # 38mm center-to-center
PAD_ROWS = 4
PAD_COLS = 4
PAD_HEIGHT = 0.007     # 7mm pad body thickness
PAD_PRESS = 0.004      # 4mm press travel
BEZEL_MARGIN = 0.008   # bezel platform margin around grid
BEZEL_HEIGHT = 0.002   # 2mm raised bezel platform above panel

GRID_CENTER_X = -0.015
GRID_CENTER_Y = 0.040

# Envelope sliders (kept from parent)
SLIDER_XS = (0.143, 0.168, 0.193, 0.218)
SLIDER_Y = 0.0455
SLIDER_TRAVEL = 0.030
SLOT_TOP_Z = 0.0859


def _white_key_outline(left_relief: bool, right_relief: bool) -> list[tuple[float, float]]:
    """Top-view outline of a white key in its hinge frame (back edge at y=0)."""
    half = WHITE_KEY_WIDTH / 2.0
    xr = RELIEF_HALF_X if right_relief else half
    xl = -RELIEF_HALF_X if left_relief else -half
    pts: list[tuple[float, float]] = [(xr, 0.0)]
    if right_relief:
        pts.append((xr, RELIEF_SHOULDER))
        pts.append((half, RELIEF_SHOULDER))
    pts.append((half, -WHITE_KEY_DEPTH))
    pts.append((-half, -WHITE_KEY_DEPTH))
    if left_relief:
        pts.append((-half, RELIEF_SHOULDER))
        pts.append((xl, RELIEF_SHOULDER))
    pts.append((xl, 0.0))
    return pts


def _white_key_mesh(name: str, left_relief: bool, right_relief: bool):
    outline = _white_key_outline(left_relief, right_relief)
    geom = LoftGeometry(
        [
            [(x, y, 0.0) for x, y in outline],
            [(x, y, WHITE_KEY_THICK) for x, y in outline],
        ],
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geom, name)


def _black_key_mesh(name: str):
    half = BLACK_KEY_WIDTH / 2.0
    bottom = [(half, -0.001), (half, -0.083), (-half, -0.083), (-half, -0.001)]
    top_half = 0.00425
    top = [(top_half, -0.004), (top_half, -0.077), (-top_half, -0.077), (-top_half, -0.004)]
    geom = LoftGeometry(
        [
            [(x, y, 0.0) for x, y in bottom],
            [(x, y, BLACK_KEY_THICK) for x, y in top],
        ],
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geom, name)


def _pad_mesh(name: str):
    """Shared geometry helper: square drum pad with a slight inward taper at
    the top face, typical of injection-molded rubber performance pads."""
    half_b = PAD_SIZE / 2.0
    half_t = half_b - 0.001  # 1mm draft on each side
    bottom = [
        (-half_b, -half_b, 0.0),
        (half_b, -half_b, 0.0),
        (half_b, half_b, 0.0),
        (-half_b, half_b, 0.0),
    ]
    top = [
        (-half_t, -half_t, PAD_HEIGHT),
        (half_t, -half_t, PAD_HEIGHT),
        (half_t, half_t, PAD_HEIGHT),
        (-half_t, half_t, PAD_HEIGHT),
    ]
    geom = LoftGeometry([bottom, top], cap=True, closed=True)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drum_pad_synth_keyboard")

    body_gray = model.material("body_dark_gray", rgba=(0.16, 0.165, 0.175, 1.0))
    trim_graphite = model.material("trim_graphite", rgba=(0.115, 0.12, 0.13, 1.0))
    key_white = model.material("key_white", rgba=(0.93, 0.93, 0.91, 1.0))
    key_black = model.material("key_black", rgba=(0.07, 0.07, 0.075, 1.0))
    pad_rubber = model.material("pad_rubber", rgba=(0.11, 0.11, 0.12, 1.0))
    strip_rubber = model.material("strip_rubber", rgba=(0.22, 0.23, 0.245, 1.0))
    accent_teal = model.material("accent_teal", rgba=(0.55, 0.86, 0.80, 1.0))
    knob_charcoal = model.material("knob_charcoal", rgba=(0.10, 0.105, 0.115, 1.0))
    pointer_white = model.material("pointer_white", rgba=(0.92, 0.94, 0.93, 1.0))

    # Backlight materials — one warm tone per row for visual variety.
    backlight_colors = [
        model.material("backlight_amber", rgba=(1.0, 0.55, 0.15, 0.9)),
        model.material("backlight_crimson", rgba=(0.95, 0.18, 0.22, 0.9)),
        model.material("backlight_cobalt", rgba=(0.22, 0.50, 1.0, 0.9)),
        model.material("backlight_lime", rgba=(0.35, 0.95, 0.40, 0.9)),
    ]

    chassis = model.part("chassis")

    # Main enclosure: full-footprint base slab plus the taller rear panel housing.
    chassis.visual(
        Box((2 * BODY_HALF_W, BODY_BACK_Y - BODY_FRONT_Y, BASE_TOP_Z)),
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z / 2.0)),
        material=body_gray,
        name="base_shell",
    )
    panel_depth = BODY_BACK_Y - PANEL_FRONT_Y
    chassis.visual(
        Box((2 * BODY_HALF_W, panel_depth, PANEL_TOP_Z - BASE_TOP_Z)),
        origin=Origin(
            xyz=(0.0, (PANEL_FRONT_Y + BODY_BACK_Y) / 2.0, (BASE_TOP_Z + PANEL_TOP_Z) / 2.0)
        ),
        material=body_gray,
        name="panel_housing",
    )

    # Front-left pitch/mod touch-strip block.
    chassis.visual(
        Box((0.118, 0.120, 0.018)),
        origin=Origin(xyz=(-0.191, -0.095, 0.049)),
        material=trim_graphite,
        name="bender_block",
    )
    strip_names = ("pitch_strip", "mod_strip", "bend_strip_0", "bend_strip_1")
    for strip_name, strip_y in zip(strip_names, (-0.135, -0.110, -0.085, -0.060)):
        chassis.visual(
            Box((0.095, 0.016, 0.0045)),
            origin=Origin(xyz=(-0.191, strip_y, 0.0598)),
            material=strip_rubber,
            name=strip_name,
        )

    # Right keybed end cheek.
    chassis.visual(
        Box((0.018, 0.120, 0.024)),
        origin=Origin(xyz=(0.241, -0.095, 0.052)),
        material=trim_graphite,
        name="keybed_cheek",
    )

    # ------------------------------------------------------------------
    # Pad bezel: raised dark platform on the panel top framing the pad grid,
    # plus a teal accent outline around the pad section.
    # ------------------------------------------------------------------
    grid_span_x = (PAD_COLS - 1) * PAD_PITCH + PAD_SIZE
    grid_span_y = (PAD_ROWS - 1) * PAD_PITCH + PAD_SIZE
    bezel_x0 = GRID_CENTER_X - grid_span_x / 2.0 - BEZEL_MARGIN
    bezel_x1 = GRID_CENTER_X + grid_span_x / 2.0 + BEZEL_MARGIN
    bezel_y0 = GRID_CENTER_Y - grid_span_y / 2.0 - BEZEL_MARGIN
    bezel_y1 = GRID_CENTER_Y + grid_span_y / 2.0 + BEZEL_MARGIN
    bezel_w = bezel_x1 - bezel_x0
    bezel_d = bezel_y1 - bezel_y0

    chassis.visual(
        Box((bezel_w, bezel_d, BEZEL_HEIGHT)),
        origin=Origin(
            xyz=((bezel_x0 + bezel_x1) / 2.0, (bezel_y0 + bezel_y1) / 2.0,
                 PANEL_TOP_Z + BEZEL_HEIGHT / 2.0)
        ),
        material=trim_graphite,
        name="pad_bezel_platform",
    )

    # Teal accent outline around the pad area (proud of bezel).
    ft = 0.0025  # frame thickness
    fh = 0.0014  # frame height
    pad_frame_zc = PANEL_TOP_Z + BEZEL_HEIGHT + fh / 2.0
    chassis.visual(
        Box((bezel_w + 2 * ft, ft, fh)),
        origin=Origin(xyz=((bezel_x0 + bezel_x1) / 2.0, bezel_y0 - ft / 2.0, pad_frame_zc)),
        material=accent_teal,
        name="pad_frame_front",
    )
    chassis.visual(
        Box((bezel_w + 2 * ft, ft, fh)),
        origin=Origin(xyz=((bezel_x0 + bezel_x1) / 2.0, bezel_y1 + ft / 2.0, pad_frame_zc)),
        material=accent_teal,
        name="pad_frame_back",
    )
    chassis.visual(
        Box((ft, bezel_d + 2 * ft, fh)),
        origin=Origin(xyz=(bezel_x0 - ft / 2.0, (bezel_y0 + bezel_y1) / 2.0, pad_frame_zc)),
        material=accent_teal,
        name="pad_frame_left",
    )
    chassis.visual(
        Box((ft, bezel_d + 2 * ft, fh)),
        origin=Origin(xyz=(bezel_x1 + ft / 2.0, (bezel_y0 + bezel_y1) / 2.0, pad_frame_zc)),
        material=accent_teal,
        name="pad_frame_right",
    )

    # Env section teal frame around the slider area (kept from parent layout).
    env_frame_zc = PANEL_TOP_Z + fh / 2.0
    chassis.visual(
        Box((0.110, ft, fh)),
        origin=Origin(xyz=(0.180, -0.018 + ft / 2.0, env_frame_zc)),
        material=accent_teal,
        name="env_frame_front",
    )
    chassis.visual(
        Box((0.110, ft, fh)),
        origin=Origin(xyz=(0.180, 0.105 - ft / 2.0, env_frame_zc)),
        material=accent_teal,
        name="env_frame_back",
    )
    chassis.visual(
        Box((ft, 0.123, fh)),
        origin=Origin(xyz=(0.125 + ft / 2.0, 0.0435, env_frame_zc)),
        material=accent_teal,
        name="env_frame_left",
    )
    chassis.visual(
        Box((ft, 0.123, fh)),
        origin=Origin(xyz=(0.235 - ft / 2.0, 0.0435, env_frame_zc)),
        material=accent_teal,
        name="env_frame_right",
    )

    # ------------------------------------------------------------------
    # 4x4 drum pad grid: 16 square backlit pads, each a prismatic press-down
    # joint seated on the bezel platform.  Loop-emitted pad_i chain.
    # ------------------------------------------------------------------
    shared_pad_mesh = _pad_mesh("synth_drum_pad")
    first_pad_x = GRID_CENTER_X - (PAD_COLS - 1) / 2.0 * PAD_PITCH
    first_pad_y = GRID_CENTER_Y - (PAD_ROWS - 1) / 2.0 * PAD_PITCH
    pad_origin_z = PANEL_TOP_Z + BEZEL_HEIGHT  # contact surface: bezel top

    for i in range(PAD_ROWS * PAD_COLS):
        row = i // PAD_COLS
        col = i % PAD_COLS
        px = first_pad_x + col * PAD_PITCH
        py = first_pad_y + row * PAD_PITCH

        pad = model.part(f"pad_{i}")
        # Pad body sits on the bezel (mesh bottom at part frame = bezel top).
        pad.visual(
            shared_pad_mesh,
            material=pad_rubber,
            name="pad_body",
        )
        # Backlit inset glow on the pad top face.
        pad.visual(
            Box((PAD_SIZE - 0.008, PAD_SIZE - 0.008, 0.001)),
            origin=Origin(xyz=(0.0, 0.0, PAD_HEIGHT - 0.0003)),
            material=backlight_colors[row],
            name="backlight",
        )

        model.articulation(
            f"chassis_to_pad_{i}",
            ArticulationType.PRISMATIC,
            parent=chassis,
            child=pad,
            origin=Origin(xyz=(px, py, pad_origin_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.5,
                lower=0.0,
                upper=PAD_PRESS,
            ),
        )

    # ------------------------------------------------------------------
    # Two octaves of hinged keys (15 white + 10 black), pivot at the back.
    # ------------------------------------------------------------------
    white_centers = [FIRST_WHITE_X + i * KEY_PITCH for i in range(WHITE_KEY_COUNT)]
    white_mesh_cache: dict[tuple[bool, bool], object] = {}
    white_key_z = WHITE_TOP_Z - WHITE_KEY_THICK / 2.0
    for i, cx in enumerate(white_centers):
        left_relief = (i - 1) in BLACK_AFTER_WHITE
        right_relief = i in BLACK_AFTER_WHITE
        cache_key = (left_relief, right_relief)
        mesh = white_mesh_cache.get(cache_key)
        if mesh is None:
            mesh = _white_key_mesh(
                f"synth_white_key_{int(left_relief)}{int(right_relief)}",
                left_relief,
                right_relief,
            )
            white_mesh_cache[cache_key] = mesh
        key = model.part(f"white_key_{i}")
        key.visual(
            mesh,
            origin=Origin(xyz=(0.0, 0.0, -WHITE_KEY_THICK / 2.0)),
            material=key_white,
            name="key_body",
        )
        model.articulation(
            f"chassis_to_white_key_{i}",
            ArticulationType.REVOLUTE,
            parent=chassis,
            child=key,
            origin=Origin(xyz=(cx, KEY_HINGE_Y, white_key_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=KEY_PRESS_RAD),
        )

    black_mesh = _black_key_mesh("synth_black_key")
    black_key_z = BLACK_BOTTOM_Z + BLACK_KEY_THICK / 2.0
    for j, after in enumerate(BLACK_AFTER_WHITE):
        bx = white_centers[after] + KEY_PITCH / 2.0
        key = model.part(f"black_key_{j}")
        key.visual(
            black_mesh,
            origin=Origin(xyz=(0.0, 0.0, -BLACK_KEY_THICK / 2.0)),
            material=key_black,
            name="key_body",
        )
        model.articulation(
            f"chassis_to_black_key_{j}",
            ArticulationType.REVOLUTE,
            parent=chassis,
            child=key,
            origin=Origin(xyz=(bx, KEY_HINGE_Y, black_key_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.8, velocity=3.0, lower=0.0, upper=KEY_PRESS_RAD),
        )

    # ------------------------------------------------------------------
    # Envelope sliders: raised slot rails on the chassis, prismatic caps.
    # ------------------------------------------------------------------
    for s, sx in enumerate(SLIDER_XS):
        chassis.visual(
            Box((0.005, 0.078, 0.0014)),
            origin=Origin(xyz=(sx, SLIDER_Y, PANEL_TOP_Z + 0.0002)),
            material=trim_graphite,
            name=f"slider_slot_{s}",
        )
        cap = model.part(f"env_slider_{s}")
        cap.visual(
            Box((0.016, 0.009, 0.007)),
            origin=Origin(xyz=(0.0, 0.0, 0.0035)),
            material=knob_charcoal,
            name="cap",
        )
        cap.visual(
            Box((0.016, 0.0016, 0.0008)),
            origin=Origin(xyz=(0.0, 0.0, 0.0068)),
            material=pointer_white,
            name="cap_line",
        )
        model.articulation(
            f"chassis_to_env_slider_{s}",
            ArticulationType.PRISMATIC,
            parent=chassis,
            child=cap,
            origin=Origin(xyz=(sx, SLIDER_Y, SLOT_TOP_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.2,
                lower=-SLIDER_TRAVEL,
                upper=SLIDER_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")

    # ------------------------------------------------------------------
    # Keys (kept from parent): two-octave hinged keybed.
    # ------------------------------------------------------------------
    white_keys = [object_model.get_part(f"white_key_{i}") for i in range(WHITE_KEY_COUNT)]
    black_keys = [object_model.get_part(f"black_key_{j}") for j in range(len(BLACK_AFTER_WHITE))]
    ctx.check(
        "two octaves of keys (15 white + 10 black = 25)",
        len(white_keys) == 15 and len(black_keys) == 10,
        details=f"white={len(white_keys)}, black={len(black_keys)}",
    )
    ctx.expect_origin_distance(
        white_keys[0],
        white_keys[-1],
        axes="x",
        min_dist=0.330,
        max_dist=0.342,
        name="keybed spans two octaves of white keys",
    )

    # Key tails pass under the panel lip, as on a real hinged keybed.
    for key in white_keys + black_keys:
        ctx.allow_overlap(
            key,
            chassis,
            elem_a="key_body",
            elem_b="panel_housing",
            reason="key tail and hidden hinge line pass under the panel lip into the keybed cavity",
        )

    ctx.expect_gap(
        white_keys[1],
        white_keys[0],
        axis="x",
        min_gap=0.0005,
        name="adjacent white keys keep a visible gap",
    )
    black0_aabb = ctx.part_world_aabb(black_keys[0])
    tail_left_edge = FIRST_WHITE_X + RELIEF_HALF_X
    tail_right_edge = FIRST_WHITE_X + KEY_PITCH - RELIEF_HALF_X
    ctx.check(
        "black key clears the notched white-key tails on both sides",
        black0_aabb is not None
        and black0_aabb[0][0] > tail_left_edge + 0.0003
        and black0_aabb[1][0] < tail_right_edge - 0.0003,
        details=f"black_aabb={black0_aabb}, channel=({tail_left_edge}, {tail_right_edge})",
    )
    ctx.expect_gap(
        black_keys[0],
        white_keys[0],
        axis="z",
        min_gap=-0.005,
        name="black key rides above the white key level",
    )

    # Key press: hinged at the back, the front edge dips.
    white_joint = object_model.get_articulation("chassis_to_white_key_7")
    limits_ok = all(
        abs(object_model.get_articulation(f"chassis_to_white_key_{i}").motion_limits.upper - KEY_PRESS_RAD) < 1e-9
        and object_model.get_articulation(f"chassis_to_white_key_{i}").motion_limits.lower == 0.0
        for i in range(WHITE_KEY_COUNT)
    )
    ctx.check("all white keys share the same downward press travel", limits_ok)
    rest_aabb = ctx.part_world_aabb(white_keys[7])
    with ctx.pose({white_joint: KEY_PRESS_RAD}):
        pressed_aabb = ctx.part_world_aabb(white_keys[7])
        ctx.expect_gap(
            white_keys[7],
            chassis,
            axis="z",
            min_gap=0.0,
            negative_elem="base_shell",
            name="pressed white key stays clear of the base slab",
        )
    ctx.check(
        "pressing a white key dips its front edge by several millimeters",
        rest_aabb is not None
        and pressed_aabb is not None
        and pressed_aabb[0][2] < rest_aabb[0][2] - 0.004,
        details=f"rest_min_z={rest_aabb}, pressed_min_z={pressed_aabb}",
    )
    black_joint = object_model.get_articulation("chassis_to_black_key_0")
    rest_black = ctx.part_world_aabb(black_keys[0])
    with ctx.pose({black_joint: KEY_PRESS_RAD}):
        pressed_black = ctx.part_world_aabb(black_keys[0])
    ctx.check(
        "pressing a black key dips its front edge",
        rest_black is not None
        and pressed_black is not None
        and pressed_black[0][2] < rest_black[0][2] - 0.003,
        details=f"rest={rest_black}, pressed={pressed_black}",
    )

    # ------------------------------------------------------------------
    # Drum pads: 16 pads in a 4x4 grid, prismatic press-down, backlit.
    # ------------------------------------------------------------------
    total_pads = PAD_ROWS * PAD_COLS
    pads = [object_model.get_part(f"pad_{i}") for i in range(total_pads)]
    ctx.check(
        "4x4 drum pad grid has 16 pads",
        len(pads) == total_pads,
        details=f"count={len(pads)}",
    )

    # Every pad has a backlight visual and a pad_body visual.
    for i in range(total_pads):
        pad = object_model.get_part(f"pad_{i}")
        ctx.check(
            f"pad_{i} has a backlight inset",
            pad.get_visual("backlight") is not None,
        )
        ctx.check(
            f"pad_{i} has a pad_body",
            pad.get_visual("pad_body") is not None,
        )

    # Uniform press policy: all pad joints are prismatic, vertical downward,
    # lower=0, upper=PAD_PRESS.
    for i in range(total_pads):
        joint = object_model.get_articulation(f"chassis_to_pad_{i}")
        ctx.check(
            f"pad_{i} joint is downward prismatic with uniform press",
            joint.articulation_type == ArticulationType.PRISMATIC
            and tuple(joint.axis) == (0.0, 0.0, -1.0)
            and joint.motion_limits.lower == 0.0
            and abs(joint.motion_limits.upper - PAD_PRESS) < 1e-9,
        )

    # Grid regularity: adjacent column pads spaced by PAD_PITCH in X,
    # adjacent row pads spaced by PAD_PITCH in Y.
    pad0 = object_model.get_part("pad_0")
    pad1 = object_model.get_part("pad_1")
    pad4 = object_model.get_part("pad_4")
    ctx.expect_origin_distance(
        pad0, pad1, axes="x",
        min_dist=PAD_PITCH - 0.001,
        max_dist=PAD_PITCH + 0.001,
        name="column-adjacent pads are evenly spaced in X",
    )
    ctx.expect_origin_distance(
        pad0, pad4, axes="y",
        min_dist=PAD_PITCH - 0.001,
        max_dist=PAD_PITCH + 0.001,
        name="row-adjacent pads are evenly spaced in Y",
    )

    # Press check: pressing pad_0 moves it downward along Z.
    pad0_joint = object_model.get_articulation("chassis_to_pad_0")
    pad0_rest = ctx.part_world_position(pad0)
    with ctx.pose({pad0_joint: PAD_PRESS}):
        pad0_pressed = ctx.part_world_position(pad0)
    ctx.check(
        "pressing pad_0 moves it downward",
        pad0_rest is not None
        and pad0_pressed is not None
        and pad0_pressed[2] < pad0_rest[2] - 0.002,
        details=f"rest_z={pad0_rest}, pressed_z={pad0_pressed}",
    )

    # Pads sit seated on the bezel platform (contact, not floating).
    for i in range(total_pads):
        pad = object_model.get_part(f"pad_{i}")
        ctx.expect_gap(
            pad,
            chassis,
            axis="z",
            min_gap=-0.001,
            max_gap=0.001,
            negative_elem="pad_bezel_platform",
            name=f"pad_{i} sits on the bezel platform",
        )

    # Pad grid does not overlap with the slider section on the right.
    pad_last = object_model.get_part(f"pad_{total_pads - 1}")
    slider0 = object_model.get_part("env_slider_0")
    ctx.expect_gap(
        slider0,
        pad_last,
        axis="x",
        min_gap=0.01,
        name="pad grid clears the slider section",
    )

    # ------------------------------------------------------------------
    # Sliders (kept from parent): prismatic caps ride their slot rails.
    # ------------------------------------------------------------------
    for s in range(len(SLIDER_XS)):
        cap = object_model.get_part(f"env_slider_{s}")
        slot = chassis.get_visual(f"slider_slot_{s}")
        ctx.expect_contact(
            cap,
            chassis,
            elem_a="cap",
            elem_b=slot,
            contact_tol=1e-5,
            name=f"env_slider_{s} cap rests on its slot rail",
        )
        ctx.expect_within(
            cap,
            chassis,
            axes="y",
            inner_elem="cap",
            outer_elem=slot,
            margin=0.001,
            name=f"env_slider_{s} cap starts inside the slot run",
        )
    slider_joint = object_model.get_articulation("chassis_to_env_slider_0")
    cap0 = object_model.get_part("env_slider_0")
    rest_pos = ctx.part_world_position(cap0)
    with ctx.pose({slider_joint: SLIDER_TRAVEL}):
        moved_pos = ctx.part_world_position(cap0)
        ctx.expect_within(
            cap0,
            chassis,
            axes="y",
            inner_elem="cap",
            outer_elem="slider_slot_0",
            margin=0.001,
            name="slider cap stays on the slot rail at full travel",
        )
    ctx.check(
        "slider cap travels along the slot",
        rest_pos is not None and moved_pos is not None and moved_pos[1] > rest_pos[1] + 0.02,
        details=f"rest={rest_pos}, moved={moved_pos}",
    )

    # ------------------------------------------------------------------
    # Pitch/mod strips (kept from parent).
    # ------------------------------------------------------------------
    pitch_aabb = ctx.part_element_world_aabb(chassis, elem="pitch_strip")
    mod_aabb = ctx.part_element_world_aabb(chassis, elem="mod_strip")
    ctx.check(
        "pitch/mod strips sit at the front-left of the body",
        pitch_aabb is not None
        and mod_aabb is not None
        and pitch_aabb[1][0] < -0.13
        and pitch_aabb[1][1] < -0.03
        and mod_aabb[1][0] < -0.13,
        details=f"pitch={pitch_aabb}, mod={mod_aabb}",
    )

    # Pad section teal frame sits on the bezel platform.
    frame_aabb = ctx.part_element_world_aabb(chassis, elem="pad_frame_front")
    ctx.check(
        "pad section frame sits on the bezel top",
        frame_aabb is not None and abs(frame_aabb[0][2] - (PANEL_TOP_Z + BEZEL_HEIGHT)) < 0.001,
        details=f"frame_z_min={frame_aabb[0][2] if frame_aabb else None}, bezel_top={PANEL_TOP_Z + BEZEL_HEIGHT}",
    )

    return ctx.report()


object_model = build_object_model()
