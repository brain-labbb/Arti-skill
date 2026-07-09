from __future__ import annotations

# Analog synthesizer keyboard, dark gray, modeled after picture/Music/keyboard/002.png:
# a flat top panel densely populated with rotary knobs grouped in teal-outlined
# sections, a row of small master knobs along the back, a slider section on the
# right, a 2-octave (15 white + 10 black) hinged keybed at the front, and a
# pitch/mod touch-strip block at the front-left.

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    KnobGeometry,
    KnobGrip,
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
KNOB_LIMIT_RAD = 2.4

# Large-section knob grid (two rows of three knobs per section).
SECTION_KNOB_COLS = {
    "osc": (-0.195, -0.140, -0.085),
    "filter": (-0.005, 0.045, 0.095),
}
SECTION_KNOB_ROWS = (0.014, 0.072)
MASTER_KNOB_XS = tuple(-0.20 + 0.06 * i for i in range(8))
MASTER_KNOB_Y = 0.131
SLIDER_XS = (0.143, 0.168, 0.193, 0.218)
SLIDER_Y = 0.0455
SLIDER_TRAVEL = 0.030
SLOT_TOP_Z = 0.0859  # top of the raised slider slot rail


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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="analog_synth_keyboard")

    body_gray = model.material("body_dark_gray", rgba=(0.16, 0.165, 0.175, 1.0))
    trim_graphite = model.material("trim_graphite", rgba=(0.115, 0.12, 0.13, 1.0))
    key_white = model.material("key_white", rgba=(0.93, 0.93, 0.91, 1.0))
    key_black = model.material("key_black", rgba=(0.07, 0.07, 0.075, 1.0))
    knob_charcoal = model.material("knob_charcoal", rgba=(0.10, 0.105, 0.115, 1.0))
    accent_teal = model.material("accent_teal", rgba=(0.55, 0.86, 0.80, 1.0))
    pointer_white = model.material("pointer_white", rgba=(0.92, 0.94, 0.93, 1.0))
    strip_rubber = model.material("strip_rubber", rgba=(0.22, 0.23, 0.245, 1.0))

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

    # Teal section outline decals, raised slightly above the panel surface so
    # they do not z-fight (embedded 0.5 mm into the panel, proud 0.9 mm).
    def _section_frame(label: str, x0: float, x1: float, y0: float, y1: float) -> None:
        t = 0.0025
        h = 0.0014
        zc = PANEL_TOP_Z + 0.0002
        chassis.visual(
            Box((x1 - x0, t, h)),
            origin=Origin(xyz=((x0 + x1) / 2.0, y0 + t / 2.0, zc)),
            material=accent_teal,
            name=f"{label}_frame_front",
        )
        chassis.visual(
            Box((x1 - x0, t, h)),
            origin=Origin(xyz=((x0 + x1) / 2.0, y1 - t / 2.0, zc)),
            material=accent_teal,
            name=f"{label}_frame_back",
        )
        chassis.visual(
            Box((t, y1 - y0, h)),
            origin=Origin(xyz=(x0 + t / 2.0, (y0 + y1) / 2.0, zc)),
            material=accent_teal,
            name=f"{label}_frame_left",
        )
        chassis.visual(
            Box((t, y1 - y0, h)),
            origin=Origin(xyz=(x1 - t / 2.0, (y0 + y1) / 2.0, zc)),
            material=accent_teal,
            name=f"{label}_frame_right",
        )

    _section_frame("osc_section", -0.235, -0.045, -0.018, 0.105)
    _section_frame("filter_section", -0.035, 0.115, -0.018, 0.105)
    _section_frame("env_section", 0.125, 0.235, -0.018, 0.105)
    _section_frame("master_section", -0.235, 0.235, 0.112, 0.150)

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
    # Rotary knobs: large section knobs and a back row of small master knobs.
    # Each knob carries a raised off-axis pointer mark.
    # ------------------------------------------------------------------
    big_knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.024,
            0.018,
            body_style="cylindrical",
            edge_radius=0.0008,
            grip=KnobGrip(style="ribbed", count=14, depth=0.0007, width=0.0016),
            center=False,
        ),
        "synth_big_knob",
    )
    small_knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.017,
            0.013,
            body_style="cylindrical",
            edge_radius=0.0006,
            grip=KnobGrip(style="ribbed", count=12, depth=0.0006, width=0.0013),
            center=False,
        ),
        "synth_small_knob",
    )

    def _add_knob(name: str, x: float, y: float, *, small: bool) -> None:
        knob = model.part(name)
        knob.visual(
            small_knob_mesh if small else big_knob_mesh,
            material=knob_charcoal,
            name="knob_body",
        )
        if small:
            pointer_dims = (0.0015, 0.006, 0.0012)
            pointer_xyz = (0.0, 0.004, 0.0126)
        else:
            pointer_dims = (0.0018, 0.0085, 0.0014)
            pointer_xyz = (0.0, 0.0055, 0.0176)
        knob.visual(
            Box(pointer_dims),
            origin=Origin(xyz=pointer_xyz),
            material=pointer_white,
            name="pointer",
        )
        model.articulation(
            f"chassis_to_{name}",
            ArticulationType.REVOLUTE,
            parent=chassis,
            child=knob,
            origin=Origin(xyz=(x, y, PANEL_TOP_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=0.1,
                velocity=4.0,
                lower=-KNOB_LIMIT_RAD,
                upper=KNOB_LIMIT_RAD,
            ),
        )

    for section, cols in SECTION_KNOB_COLS.items():
        for r, ky in enumerate(SECTION_KNOB_ROWS):
            for c, kx in enumerate(cols):
                _add_knob(f"{section}_knob_{r}_{c}", kx, ky, small=False)
    for c, kx in enumerate(MASTER_KNOB_XS):
        _add_knob(f"master_knob_{c}", kx, MASTER_KNOB_Y, small=True)

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

    # Key tails intentionally pass 5 mm under the panel lip, as on a real
    # hinged keybed where the pivot sits behind the panel edge.
    for key in white_keys + black_keys:
        ctx.allow_overlap(
            key,
            chassis,
            elem_a="key_body",
            elem_b="panel_housing",
            reason="key tail and hidden hinge line pass under the panel lip into the keybed cavity",
        )

    # Key fit: neighboring white keys stay separated; black keys clear the
    # notched white-key tails on both sides.
    ctx.expect_gap(
        white_keys[1],
        white_keys[0],
        axis="x",
        min_gap=0.0005,
        name="adjacent white keys keep a visible gap",
    )
    # Black keys ride in the notched channel between white-key tails; the
    # whole-part AABB overlaps the wide white-key fronts, so check the black
    # key x-bounds against the notch edges directly.
    black0_aabb = ctx.part_world_aabb(black_keys[0])
    tail_left_edge = FIRST_WHITE_X + RELIEF_HALF_X  # white_key_0 notched right edge
    tail_right_edge = FIRST_WHITE_X + KEY_PITCH - RELIEF_HALF_X  # white_key_1 notched left edge
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

    # Key press travel: hinged at the back, the front edge dips but stays
    # clear of the base slab.
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

    # Rotary knobs: 20 knobs, vertical-axis revolute with symmetric limits,
    # each seated on the panel top and carrying an off-axis pointer mark.
    knob_names = [
        f"{section}_knob_{r}_{c}"
        for section, cols in SECTION_KNOB_COLS.items()
        for r in range(len(SECTION_KNOB_ROWS))
        for c in range(len(cols))
    ] + [f"master_knob_{c}" for c in range(len(MASTER_KNOB_XS))]
    ctx.check("dense knob field has 20 rotary knobs", len(knob_names) == 20)
    for name in knob_names:
        knob = object_model.get_part(name)
        joint = object_model.get_articulation(f"chassis_to_{name}")
        ctx.check(
            f"{name} rotates about a vertical axis with symmetric limits",
            tuple(joint.axis) == (0.0, 0.0, 1.0)
            and joint.motion_limits.lower == -KNOB_LIMIT_RAD
            and joint.motion_limits.upper == KNOB_LIMIT_RAD,
        )
        ctx.check(
            f"{name} has an off-axis pointer mark",
            knob.get_visual("pointer") is not None,
        )
        ctx.expect_gap(
            knob,
            chassis,
            axis="z",
            min_gap=0.0,
            max_gap=0.0005,
            negative_elem="panel_housing",
            name=f"{name} sits seated on the panel top",
        )

    # Sliders: prismatic caps ride their raised slot rails.
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

    # Front-left pitch/mod touch strips exist on the bender block.
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

    # Teal section outlines sit proud of the panel surface (no z-fighting).
    frame_aabb = ctx.part_element_world_aabb(chassis, elem="osc_section_frame_front")
    ctx.check(
        "section outline decals are raised above the panel surface",
        frame_aabb is not None and frame_aabb[1][2] > PANEL_TOP_Z + 0.0004,
        details=f"frame={frame_aabb}, panel_top={PANEL_TOP_Z}",
    )

    return ctx.report()


object_model = build_object_model()
