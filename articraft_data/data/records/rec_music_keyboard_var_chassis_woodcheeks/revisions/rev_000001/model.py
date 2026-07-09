from __future__ import annotations

# Analog synthesizer keyboard with upright angled body and wooden end cheeks.
# The flat slab chassis is reshaped into a wedge: a low base slab at the front
# carries the keybed and pitch/mod strips, while the control panel rises at an
# angle toward the back. Tall solid walnut end cheeks frame the assembly on
# both sides, extending from the table to well above the tilted panel.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    KnobGeometry,
    KnobGrip,
    LoftGeometry,
    MeshGeometry,
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

# Tilted panel: surface rises from front to back.
PANEL_FRONT_Y = -0.035
PANEL_FRONT_Z = 0.065   # panel surface height at front edge
PANEL_BACK_Z = 0.155     # panel surface height at back edge
PANEL_TILT_RUN = BODY_BACK_Y - PANEL_FRONT_Y
PANEL_TILT_RISE = PANEL_BACK_Z - PANEL_FRONT_Z
PANEL_TILT_ANGLE = math.atan2(PANEL_TILT_RISE, PANEL_TILT_RUN)

# Panel surface normal (points up and toward player) and along-surface direction.
PANEL_NORMAL = (0.0, -math.sin(PANEL_TILT_ANGLE), math.cos(PANEL_TILT_ANGLE))
PANEL_ALONG_Y = (0.0, math.cos(PANEL_TILT_ANGLE), math.sin(PANEL_TILT_ANGLE))


def panel_surface_z(y: float) -> float:
    """Return the Z height of the tilted panel surface at a given Y coordinate."""
    t = (y - PANEL_FRONT_Y) / PANEL_TILT_RUN
    return PANEL_FRONT_Z + t * PANEL_TILT_RISE


# End cheeks (tall solid wooden boards).
CHEEK_THICK = 0.022
CHEEK_FRONT_H = 0.13
CHEEK_BACK_H = 0.21

# Key parameters (unchanged from parent).
KEY_PITCH = 0.024
WHITE_KEY_COUNT = 15  # two octaves C..C
WHITE_KEY_WIDTH = 0.0225
WHITE_KEY_DEPTH = 0.123
WHITE_KEY_THICK = 0.012
WHITE_TOP_Z = 0.062
KEY_HINGE_Y = -0.030
RELIEF_SHOULDER = -0.085
RELIEF_HALF_X = 0.0057

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


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _add_box_faces(g: MeshGeometry, v: list[int]) -> None:
    """Add 12 triangles for an 8-vertex box (v0..v3 = front, v4..v7 = back)."""
    # Front (-Y facing)
    g.add_face(v[0], v[1], v[2]); g.add_face(v[0], v[2], v[3])
    # Back (+Y facing)
    g.add_face(v[5], v[4], v[7]); g.add_face(v[5], v[7], v[6])
    # Bottom (-Z facing)
    g.add_face(v[0], v[4], v[5]); g.add_face(v[0], v[5], v[1])
    # Top (+Z facing)
    g.add_face(v[3], v[2], v[6]); g.add_face(v[3], v[6], v[7])
    # Left (-X facing)
    g.add_face(v[0], v[3], v[7]); g.add_face(v[0], v[7], v[4])
    # Right (+X facing)
    g.add_face(v[1], v[5], v[6]); g.add_face(v[1], v[6], v[2])


def _cheek_mesh(name: str):
    """Shared geometry helper: solid wooden end cheek, centered at X=0.

    Trapezoidal profile in YZ: low at front, tall at back.
    Extruded symmetrically in X by CHEEK_THICK.
    """
    g = MeshGeometry()
    ht = CHEEK_THICK / 2.0
    # Front face (at BODY_FRONT_Y, height = CHEEK_FRONT_H)
    v = [
        g.add_vertex(-ht, BODY_FRONT_Y, 0.0),           # 0
        g.add_vertex(ht, BODY_FRONT_Y, 0.0),             # 1
        g.add_vertex(ht, BODY_FRONT_Y, CHEEK_FRONT_H),   # 2
        g.add_vertex(-ht, BODY_FRONT_Y, CHEEK_FRONT_H),  # 3
        # Back face (at BODY_BACK_Y, height = CHEEK_BACK_H)
        g.add_vertex(-ht, BODY_BACK_Y, 0.0),             # 4
        g.add_vertex(ht, BODY_BACK_Y, 0.0),              # 5
        g.add_vertex(ht, BODY_BACK_Y, CHEEK_BACK_H),     # 6
        g.add_vertex(-ht, BODY_BACK_Y, CHEEK_BACK_H),    # 7
    ]
    _add_box_faces(g, v)
    return mesh_from_geometry(g, name)


def _angled_panel_mesh(name: str):
    """Wedge-shaped panel body: flat bottom, angled top rising from front to back."""
    g = MeshGeometry()
    hw = BODY_HALF_W
    v = [
        g.add_vertex(-hw, PANEL_FRONT_Y, BASE_TOP_Z),     # 0
        g.add_vertex(hw, PANEL_FRONT_Y, BASE_TOP_Z),      # 1
        g.add_vertex(hw, PANEL_FRONT_Y, PANEL_FRONT_Z),   # 2
        g.add_vertex(-hw, PANEL_FRONT_Y, PANEL_FRONT_Z),  # 3
        g.add_vertex(-hw, BODY_BACK_Y, BASE_TOP_Z),       # 4
        g.add_vertex(hw, BODY_BACK_Y, BASE_TOP_Z),        # 5
        g.add_vertex(hw, BODY_BACK_Y, PANEL_BACK_Z),      # 6
        g.add_vertex(-hw, BODY_BACK_Y, PANEL_BACK_Z),     # 7
    ]
    _add_box_faces(g, v)
    return mesh_from_geometry(g, name)


def _tilted_bar_mesh(name: str, x_center: float, y_start: float, y_end: float,
                     width_x: float, height_z: float, *, proud: float = 0.0002):
    """A thin bar that follows the panel tilt from y_start to y_end."""
    g = MeshGeometry()
    hw = width_x / 2.0
    z0 = panel_surface_z(y_start) + proud - height_z / 2.0
    z1 = panel_surface_z(y_end) + proud - height_z / 2.0
    v = [
        g.add_vertex(x_center - hw, y_start, z0),              # 0
        g.add_vertex(x_center + hw, y_start, z0),              # 1
        g.add_vertex(x_center + hw, y_start, z0 + height_z),   # 2
        g.add_vertex(x_center - hw, y_start, z0 + height_z),   # 3
        g.add_vertex(x_center - hw, y_end, z1),                 # 4
        g.add_vertex(x_center + hw, y_end, z1),                 # 5
        g.add_vertex(x_center + hw, y_end, z1 + height_z),     # 6
        g.add_vertex(x_center - hw, y_end, z1 + height_z),     # 7
    ]
    _add_box_faces(g, v)
    return mesh_from_geometry(g, name)


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


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="analog_synth_keyboard")

    # -- Materials --
    body_gray = model.material("body_dark_gray", rgba=(0.16, 0.165, 0.175, 1.0))
    trim_graphite = model.material("trim_graphite", rgba=(0.115, 0.12, 0.13, 1.0))
    key_white = model.material("key_white", rgba=(0.93, 0.93, 0.91, 1.0))
    key_black = model.material("key_black", rgba=(0.07, 0.07, 0.075, 1.0))
    knob_charcoal = model.material("knob_charcoal", rgba=(0.10, 0.105, 0.115, 1.0))
    accent_teal = model.material("accent_teal", rgba=(0.55, 0.86, 0.80, 1.0))
    pointer_white = model.material("pointer_white", rgba=(0.92, 0.94, 0.93, 1.0))
    strip_rubber = model.material("strip_rubber", rgba=(0.22, 0.23, 0.245, 1.0))
    wood_walnut = model.material("wood_walnut", rgba=(0.42, 0.26, 0.15, 1.0))

    # ==================================================================
    # LAYER 1: Chassis body (root part)
    # ==================================================================
    chassis = model.part("chassis")

    # Base slab: flat platform under the full footprint.
    chassis.visual(
        Box((2 * BODY_HALF_W, BODY_BACK_Y - BODY_FRONT_Y, BASE_TOP_Z)),
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z / 2.0)),
        material=body_gray,
        name="base_shell",
    )

    # Angled panel: wedge rising from front to back, carries all controls.
    chassis.visual(
        _angled_panel_mesh("angled_panel"),
        material=body_gray,
        name="angled_panel",
    )

    # Front-left pitch/mod touch-strip block (on the flat base, not tilted).
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

    # ==================================================================
    # LAYER 2: End cheeks (loop-emitted, mirrored, inline parent visuals)
    # ==================================================================
    cheek_mesh = _cheek_mesh("synth_cheek")
    for i in range(2):
        sign = 1.0 if i == 0 else -1.0
        cheek_x = sign * (BODY_HALF_W - CHEEK_THICK / 2.0)
        chassis.visual(
            cheek_mesh,
            origin=Origin(xyz=(cheek_x, 0.0, 0.0)),
            material=wood_walnut,
            name=f"cheek_{i}",
        )

    # ==================================================================
    # LAYER 3: Section frame decals (on tilted panel surface)
    # ==================================================================
    def _section_frame(label: str, x0: float, x1: float, y0: float, y1: float) -> None:
        t = 0.0025
        h = 0.0014
        # Front strip (horizontal, at fixed Y = y0)
        z_f = panel_surface_z(y0) + 0.0002 + h / 2.0
        chassis.visual(
            Box((x1 - x0, t, h)),
            origin=Origin(xyz=((x0 + x1) / 2.0, y0 + t / 2.0, z_f)),
            material=accent_teal,
            name=f"{label}_frame_front",
        )
        # Back strip (horizontal, at fixed Y = y1)
        z_b = panel_surface_z(y1) + 0.0002 + h / 2.0
        chassis.visual(
            Box((x1 - x0, t, h)),
            origin=Origin(xyz=((x0 + x1) / 2.0, y1 - t / 2.0, z_b)),
            material=accent_teal,
            name=f"{label}_frame_back",
        )
        # Left strip (follows panel tilt)
        chassis.visual(
            _tilted_bar_mesh(f"{label}_frame_left", x0 + t / 2.0, y0, y1, t, h),
            material=accent_teal,
            name=f"{label}_frame_left",
        )
        # Right strip (follows panel tilt)
        chassis.visual(
            _tilted_bar_mesh(f"{label}_frame_right", x1 - t / 2.0, y0, y1, t, h),
            material=accent_teal,
            name=f"{label}_frame_right",
        )

    _section_frame("osc_section", -0.235, -0.045, -0.018, 0.105)
    _section_frame("filter_section", -0.035, 0.115, -0.018, 0.105)
    _section_frame("env_section", 0.125, 0.235, -0.018, 0.105)
    _section_frame("master_section", -0.235, 0.235, 0.112, 0.150)

    # ==================================================================
    # LAYER 4: Slider slots (raised rails on tilted panel)
    # ==================================================================
    slot_half_d = 0.039
    for s, sx in enumerate(SLIDER_XS):
        chassis.visual(
            _tilted_bar_mesh(f"slider_slot_{s}", sx,
                             SLIDER_Y - slot_half_d, SLIDER_Y + slot_half_d,
                             0.005, 0.0014),
            material=trim_graphite,
            name=f"slider_slot_{s}",
        )

    # ==================================================================
    # LAYER 5: Two octaves of hinged keys (15 white + 10 black)
    # ==================================================================
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

    # ==================================================================
    # LAYER 6: Rotary knobs on tilted panel
    # ==================================================================
    big_knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.024, 0.018,
            body_style="cylindrical",
            edge_radius=0.0008,
            grip=KnobGrip(style="ribbed", count=14, depth=0.0007, width=0.0016),
            center=False,
        ),
        "synth_big_knob",
    )
    small_knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.017, 0.013,
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
        # Tilt the child frame so the knob base is coplanar with the panel.
        # Local Z after tilt = panel normal; axis (0,0,1) rotates around it.
        model.articulation(
            f"chassis_to_{name}",
            ArticulationType.REVOLUTE,
            parent=chassis,
            child=knob,
            origin=Origin(
                xyz=(x, y, panel_surface_z(y)),
                rpy=(PANEL_TILT_ANGLE, 0.0, 0.0),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=0.1, velocity=4.0,
                lower=-KNOB_LIMIT_RAD, upper=KNOB_LIMIT_RAD,
            ),
        )

    for section, cols in SECTION_KNOB_COLS.items():
        for r, ky in enumerate(SECTION_KNOB_ROWS):
            for c, kx in enumerate(cols):
                _add_knob(f"{section}_knob_{r}_{c}", kx, ky, small=False)
    for c, kx in enumerate(MASTER_KNOB_XS):
        _add_knob(f"master_knob_{c}", kx, MASTER_KNOB_Y, small=True)

    # ==================================================================
    # LAYER 7: Envelope sliders on tilted panel
    # ==================================================================
    slot_top_z_at_slider = panel_surface_z(SLIDER_Y) + 0.0002 + 0.0007
    for s, sx in enumerate(SLIDER_XS):
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
        # Tilt child frame so cap sits flush on the tilted panel.
        # Local Y after tilt = along panel surface; axis (0,1,0) slides along it.
        model.articulation(
            f"chassis_to_env_slider_{s}",
            ArticulationType.PRISMATIC,
            parent=chassis,
            child=cap,
            origin=Origin(
                xyz=(sx, SLIDER_Y, slot_top_z_at_slider),
                rpy=(PANEL_TILT_ANGLE, 0.0, 0.0),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0, velocity=0.2,
                lower=-SLIDER_TRAVEL, upper=SLIDER_TRAVEL,
            ),
        )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")

    # -- End cheeks exist, are tall, and rise above the panel --
    cheek_0_vis = chassis.get_visual("cheek_0")
    cheek_1_vis = chassis.get_visual("cheek_1")
    ctx.check(
        "two wooden end cheeks (cheek_0 and cheek_1) are inline parent visuals",
        cheek_0_vis is not None and cheek_1_vis is not None,
    )
    cheek_0_aabb = ctx.part_element_world_aabb(chassis, elem="cheek_0")
    cheek_1_aabb = ctx.part_element_world_aabb(chassis, elem="cheek_1")
    ctx.check(
        "end cheeks are mirrored left and right",
        cheek_0_aabb is not None
        and cheek_1_aabb is not None
        and cheek_0_aabb[0][0] > 0.1
        and cheek_1_aabb[1][0] < -0.1,
        details=f"cheek_0={cheek_0_aabb}, cheek_1={cheek_1_aabb}",
    )
    ctx.check(
        "end cheeks rise above the tilted panel back",
        cheek_0_aabb is not None
        and cheek_0_aabb[1][2] > PANEL_BACK_Z + 0.03
        and cheek_1_aabb is not None
        and cheek_1_aabb[1][2] > PANEL_BACK_Z + 0.03,
        details=f"cheek_0_top={cheek_0_aabb[1][2] if cheek_0_aabb else None}, "
                f"cheek_1_top={cheek_1_aabb[1][2] if cheek_1_aabb else None}, "
                f"panel_back={PANEL_BACK_Z}",
    )
    ctx.check(
        "cheek profile is taller at the back than at the front",
        cheek_0_aabb is not None
        and abs(cheek_0_aabb[1][2] - CHEEK_BACK_H) < 0.005
        and abs(cheek_0_aabb[0][2]) < 0.005,
        details=f"cheek_0_aabb={cheek_0_aabb}",
    )

    # -- Angled panel rises from front to back --
    panel_aabb = ctx.part_element_world_aabb(chassis, elem="angled_panel")
    ctx.check(
        "angled panel rises from front to back (tilted body)",
        panel_aabb is not None
        and abs(panel_aabb[0][2] - BASE_TOP_Z) < 0.005
        and abs(panel_aabb[1][2] - PANEL_BACK_Z) < 0.005,
        details=f"panel_aabb={panel_aabb}",
    )

    # -- Keys: two octaves, hinged, pressable --
    white_keys = [object_model.get_part(f"white_key_{i}") for i in range(WHITE_KEY_COUNT)]
    black_keys = [object_model.get_part(f"black_key_{j}") for j in range(len(BLACK_AFTER_WHITE))]
    ctx.check(
        "two octaves of keys (15 white + 10 black = 25)",
        len(white_keys) == 15 and len(black_keys) == 10,
        details=f"white={len(white_keys)}, black={len(black_keys)}",
    )
    ctx.expect_origin_distance(
        white_keys[0], white_keys[-1],
        axes="x", min_dist=0.330, max_dist=0.342,
        name="keybed spans two octaves of white keys",
    )

    # Key tails pass under the tilted panel lip.
    for key in white_keys + black_keys:
        ctx.allow_overlap(
            key, chassis,
            elem_a="key_body", elem_b="angled_panel",
            reason="key tail and hidden hinge line pass under the panel lip into the keybed cavity",
        )

    ctx.expect_gap(
        white_keys[1], white_keys[0], axis="x", min_gap=0.0005,
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
        black_keys[0], white_keys[0], axis="z", min_gap=-0.005,
        name="black key rides above the white key level",
    )

    # Key press travel.
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
            white_keys[7], chassis, axis="z", min_gap=0.0,
            negative_elem="base_shell",
            name="pressed white key stays clear of the base slab",
        )
    ctx.check(
        "pressing a white key dips its front edge by several millimeters",
        rest_aabb is not None and pressed_aabb is not None
        and pressed_aabb[0][2] < rest_aabb[0][2] - 0.004,
        details=f"rest_min_z={rest_aabb}, pressed_min_z={pressed_aabb}",
    )
    black_joint = object_model.get_articulation("chassis_to_black_key_0")
    rest_black = ctx.part_world_aabb(black_keys[0])
    with ctx.pose({black_joint: KEY_PRESS_RAD}):
        pressed_black = ctx.part_world_aabb(black_keys[0])
    ctx.check(
        "pressing a black key dips its front edge",
        rest_black is not None and pressed_black is not None
        and pressed_black[0][2] < rest_black[0][2] - 0.003,
        details=f"rest={rest_black}, pressed={pressed_black}",
    )

    # -- Rotary knobs: 20 knobs on tilted panel, panel-normal axis --
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
        # Knob base sits flush on the tilted panel; the AABB overlap is
        # surface-contact embedding, not penetration.
        ctx.allow_overlap(
            chassis, knob,
            elem_a="angled_panel", elem_b="knob_body",
            reason="knob base is seated flush on the tilted panel surface",
        )
        ctx.check(
            f"{name} rotates about the panel normal with symmetric limits",
            (all(abs(a - b) < 1e-6 for a, b in zip(joint.axis, PANEL_NORMAL))
             or all(abs(a - b) < 1e-6 for a, b in zip(joint.axis, (0.0, 0.0, 1.0))))
            and joint.motion_limits.lower == -KNOB_LIMIT_RAD
            and joint.motion_limits.upper == KNOB_LIMIT_RAD,
        )
        ctx.check(
            f"{name} has an off-axis pointer mark",
            knob.get_visual("pointer") is not None,
        )
        ctx.expect_contact(
            knob, chassis,
            elem_a="knob_body", elem_b="angled_panel",
            contact_tol=0.003,
            name=f"{name} sits seated on the tilted panel",
        )

    # -- Sliders: prismatic caps on tilted panel --
    for s in range(len(SLIDER_XS)):
        cap = object_model.get_part(f"env_slider_{s}")
        slot = chassis.get_visual(f"slider_slot_{s}")
        ctx.expect_contact(
            cap, chassis,
            elem_a="cap", elem_b=slot,
            contact_tol=0.005,
            name=f"env_slider_{s} cap rests near its slot rail",
        )
    slider_joint = object_model.get_articulation("chassis_to_env_slider_0")
    cap0 = object_model.get_part("env_slider_0")
    rest_pos = ctx.part_world_position(cap0)
    with ctx.pose({slider_joint: SLIDER_TRAVEL}):
        moved_pos = ctx.part_world_position(cap0)
    ctx.check(
        "slider cap travels along the tilted panel",
        rest_pos is not None and moved_pos is not None
        and moved_pos[1] > rest_pos[1] + 0.02,
        details=f"rest={rest_pos}, moved={moved_pos}",
    )

    # -- Pitch/mod strips at front-left --
    pitch_aabb = ctx.part_element_world_aabb(chassis, elem="pitch_strip")
    mod_aabb = ctx.part_element_world_aabb(chassis, elem="mod_strip")
    ctx.check(
        "pitch/mod strips sit at the front-left of the body",
        pitch_aabb is not None and mod_aabb is not None
        and pitch_aabb[1][0] < -0.13
        and pitch_aabb[1][1] < -0.03
        and mod_aabb[1][0] < -0.13,
        details=f"pitch={pitch_aabb}, mod={mod_aabb}",
    )

    # -- Section frame decals sit above the panel surface --
    frame_aabb = ctx.part_element_world_aabb(chassis, elem="osc_section_frame_front")
    expected_frame_z = panel_surface_z(-0.018) + 0.0004
    ctx.check(
        "section outline decals are raised above the tilted panel surface",
        frame_aabb is not None and frame_aabb[1][2] > expected_frame_z,
        details=f"frame={frame_aabb}, expected_min_z={expected_frame_z}",
    )

    return ctx.report()


object_model = build_object_model()
