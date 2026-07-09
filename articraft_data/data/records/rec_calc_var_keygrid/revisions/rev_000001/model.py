from __future__ import annotations

# Realistic articulated pocket calculator — compact basic-calculator variant.
#
# Reference: picture/Stationary/Calculater/001.png
#   - Portrait slate-blue plastic body with rounded corners.
#   - A dark recessed LCD display occupies the upper-third of the face.
#   - A small dark solar-cell window sits at the very top.
#   - Cream rounded keys arranged in a compact 4-column grid for basic
#     arithmetic (AC, C, %, ÷, 7-9, ×, 4-6, −, 1-3, +, 0, ., +/−, =).
#
# Variant change: reduced from the parent 5-column scientific layout to a
# compact 4-column basic-calculator keypad (20 keys). Body, display, and
# solar geometry are identical; only the key grid is smaller.
#
# Primary mechanism: each key travels a short linear distance DOWN into its
# well when pressed → PRISMATIC joints (one per key) with a small stroke.
# The display and solar cell are inlined as body visuals (no FIXED joints).

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Real-world dimensions (meters). A typical pocket calculator.
# ---------------------------------------------------------------------------
BODY_W = 0.075          # width  (X)
BODY_L = 0.135          # length (Y, portrait)
BODY_T = 0.012          # thickness (Z)
CORNER_R = 0.006        # body corner radius

FACE_Z = BODY_T         # top face of body (keys/display live here)

# Display recess (the dark LCD window)
DISPLAY_W = 0.058
DISPLAY_L = 0.030
DISPLAY_Y = 0.040       # center Y of display (toward the top)
DISPLAY_DEPTH = 0.0022
DISPLAY_LCD_INSET = 0.004

# Solar-cell window (small dark bar at very top)
SOLAR_W = 0.030
SOLAR_L = 0.011
SOLAR_Y = 0.058
SOLAR_DEPTH = 0.0018

# Key geometry
KEY_TRAVEL = 0.0015     # downward press stroke
KEY_PROUD = 0.0026      # how far a key rises above the body face at rest
WELL_DEPTH = 0.0020     # recessed key-well depth cut into the face
WELL_CLEAR = 0.0010     # clearance between key edge and well wall (per side)


def _rounded_box(width: float, length: float, height: float, radius: float):
    """A box centered in XY, sitting on z=0, with rounded vertical edges."""
    return (
        cq.Workplane("XY")
        .box(width, length, height, centered=(True, True, False))
        .edges("|Z")
        .fillet(radius)
    )


def _build_body_mesh():
    """Slate-blue calculator body: rounded shell with display + solar recesses
    and recessed wells for every key, all cut from a single solid so the face
    reads as one continuous molded piece."""
    body = _rounded_box(BODY_W, BODY_L, BODY_T, CORNER_R)

    # Slight top-face inset frame so the face reads as a molded bezel.
    groove = (
        cq.Workplane("XY")
        .workplane(offset=FACE_Z)
        .rect(BODY_W - 0.005, BODY_L - 0.005)
        .rect(BODY_W - 0.008, BODY_L - 0.008)
        .extrude(-0.0006)
    )
    body = body.cut(groove)

    # Display recess
    disp_cut = (
        cq.Workplane("XY")
        .workplane(offset=FACE_Z)
        .center(0.0, DISPLAY_Y)
        .rect(DISPLAY_W, DISPLAY_L)
        .extrude(-DISPLAY_DEPTH)
    )
    body = body.cut(disp_cut)

    # Solar-cell window recess
    solar_cut = (
        cq.Workplane("XY")
        .workplane(offset=FACE_Z)
        .center(0.0, SOLAR_Y)
        .rect(SOLAR_W, SOLAR_L)
        .extrude(-SOLAR_DEPTH)
    )
    body = body.cut(solar_cut)

    # Key wells: cut a shallow recess for each key footprint.
    for kx, ky, kw, kl in _key_well_rects():
        well = (
            cq.Workplane("XY")
            .workplane(offset=FACE_Z)
            .center(kx, ky)
            .rect(kw + 2.0 * WELL_CLEAR, kl + 2.0 * WELL_CLEAR)
            .extrude(-WELL_DEPTH)
        )
        body = body.cut(well)

    return mesh_from_cadquery(body, "calculator_body")


def _build_display_panel_mesh():
    """Dark LCD glass panel that seats inside the display recess."""
    panel = _rounded_box(
        DISPLAY_W - DISPLAY_LCD_INSET,
        DISPLAY_L - DISPLAY_LCD_INSET,
        0.0016,
        0.002,
    )
    return mesh_from_cadquery(panel, "calculator_display")


def _build_solar_panel_mesh():
    """Dark solar-cell strip seated in the top window."""
    panel = _rounded_box(SOLAR_W - 0.003, SOLAR_L - 0.003, 0.0012, 0.0015)
    return mesh_from_cadquery(panel, "calculator_solar")


def _build_key_cap_mesh(name: str, width: float, length: float):
    """Shared geometry helper: a single cream key cap with domed edges."""
    height = KEY_PROUD + WELL_DEPTH
    key = (
        cq.Workplane("XY")
        .box(width, length, height, centered=(True, True, False))
        .edges("|Z")
        .fillet(min(width, length) * 0.28)
        .edges(">Z")
        .fillet(0.0006)
    )
    return mesh_from_cadquery(key, name)


# ---------------------------------------------------------------------------
# Compact basic-calculator keypad: 5 rows × 4 columns = 20 keys
# ---------------------------------------------------------------------------
COL_X = [-0.0207, -0.0069, 0.0069, 0.0207]
COL_PITCH = 0.0138
KEY_W = 0.0108
KEY_L = 0.0108

# Row Y centers (portrait, below the display)
ROW_Y = [0.0080, -0.0060, -0.0200, -0.0340, -0.0480]

# Flat key definitions: (label, column_index, row_index)
_KEY_DEFS: list[tuple[str, int, int]] = [
    # Row 0 — function row
    ("ac",    0, 0),  ("c",     1, 0),  ("pct",   2, 0),  ("div",   3, 0),
    # Row 1 — top digit row
    ("seven", 0, 1),  ("eight", 1, 1),  ("nine",  2, 1),  ("mul",   3, 1),
    # Row 2 — middle digit row
    ("four",  0, 2),  ("five",  1, 2),  ("six",   2, 2),  ("sub",   3, 2),
    # Row 3 — lower digit row
    ("one",   0, 3),  ("two",   1, 3),  ("three", 2, 3),  ("add",   3, 3),
    # Row 4 — bottom row
    ("zero",  0, 4),  ("dot",   1, 4),  ("neg",   2, 4),  ("eq",    3, 4),
]

# Labels for keys that get the accent (darker cream) material
_ACCENT_LABELS = {"div", "mul", "sub", "add", "eq", "ac", "c", "neg"}


def _key_well_rects():
    """Footprint rects (cx, cy, w, l) used to cut key wells in the body."""
    return [
        (COL_X[col], ROW_Y[row], KEY_W, KEY_L)
        for _label, col, row in _KEY_DEFS
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pocket_calculator")

    body_mat = model.material("body_slate", rgba=(0.42, 0.47, 0.58, 1.0))
    display_mat = model.material("display_dark", rgba=(0.08, 0.07, 0.06, 1.0))
    solar_mat = model.material("solar_dark", rgba=(0.10, 0.09, 0.08, 1.0))
    key_mat = model.material("key_cream", rgba=(0.92, 0.86, 0.74, 1.0))
    key_accent = model.material("key_accent", rgba=(0.86, 0.80, 0.68, 1.0))

    # ---- Root body shell with inlined display and solar visuals ----
    body = model.part("body")
    body.visual(_build_body_mesh(), material=body_mat, name="body_shell")
    body.visual(
        _build_display_panel_mesh(),
        material=display_mat,
        name="lcd",
        origin=Origin(xyz=(0.0, DISPLAY_Y, FACE_Z - DISPLAY_DEPTH)),
    )
    body.visual(
        _build_solar_panel_mesh(),
        material=solar_mat,
        name="solar_cell",
        origin=Origin(xyz=(0.0, SOLAR_Y, FACE_Z - SOLAR_DEPTH)),
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_L, BODY_T)),
        mass=0.09,
        origin=Origin(xyz=(0.0, 0.0, BODY_T / 2.0)),
    )

    # ---- Keys: each is a prismatic press key, emitted by index ----
    key_mesh_cache: dict[tuple[float, float], object] = {}
    n_keys = len(_KEY_DEFS)

    for i in range(n_keys):
        label, col, row = _KEY_DEFS[i]
        cx = COL_X[col]
        cy = ROW_Y[row]
        w, l = KEY_W, KEY_L

        cache_key = (round(w, 5), round(l, 5))
        mesh = key_mesh_cache.get(cache_key)
        if mesh is None:
            mesh = _build_key_cap_mesh(f"key_cap_{i}", w, l)
            key_mesh_cache[cache_key] = mesh

        key_part = model.part(f"key_{i}")
        mat = key_accent if label in _ACCENT_LABELS else key_mat
        key_part.visual(mesh, material=mat, name="key_cap")
        key_part.inertial = Inertial.from_geometry(
            Box((w, l, KEY_PROUD + WELL_DEPTH)),
            mass=0.0015,
            origin=Origin(xyz=(0.0, 0.0, (KEY_PROUD + WELL_DEPTH) / 2.0)),
        )

        # Prismatic press: joint frame at the well floor, axis -Z so positive
        # q presses the key DOWN into the well.
        model.articulation(
            f"body_to_key_{i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=key_part,
            origin=Origin(xyz=(cx, cy, FACE_Z - WELL_DEPTH)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.05,
                lower=0.0,
                upper=KEY_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")

    # --- Body silhouette: portrait, real pocket-calculator scale ---
    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is not None:
        (bx0, by0, bz0), (bx1, by1, bz1) = body_aabb
        width = bx1 - bx0
        length = by1 - by0
        thick = bz1 - bz0
        ctx.check(
            "body is portrait pocket-calculator scale",
            0.07 <= width <= 0.085
            and 0.13 <= length <= 0.145
            and length > width,
            details=f"w={width:.4f} l={length:.4f}",
        )
        ctx.check(
            "body is a thin slab",
            0.008 <= thick <= 0.016,
            details=f"thick={thick:.4f}",
        )

    # --- Display and solar are inlined body visuals, not separate parts ---
    body_visuals = [v.name for v in body.visuals]
    ctx.check(
        "display LCD is a body visual",
        "lcd" in body_visuals,
        details=f"visuals={body_visuals}",
    )
    ctx.check(
        "solar cell is a body visual",
        "solar_cell" in body_visuals,
        details=f"visuals={body_visuals}",
    )

    # --- Keypad has exactly 20 keys (5 rows × 4 columns) ---
    n_keys = len(_KEY_DEFS)
    key_parts = [object_model.get_part(f"key_{i}") for i in range(n_keys)]
    ctx.check(
        "keypad has 20 keys",
        n_keys == 20,
        details=f"n_keys={n_keys}",
    )

    # --- All keys are prismatic press joints along -Z ---
    for i in range(n_keys):
        joint = object_model.get_articulation(f"body_to_key_{i}")
        jtype = str(joint.articulation_type).lower()
        ctx.check(
            f"key_{i} is prismatic",
            "prismatic" in jtype,
            details=f"type={jtype}",
        )
        ax = tuple(round(c, 3) for c in joint.axis)
        ctx.check(
            f"key_{i} presses along -Z",
            ax == (0.0, 0.0, -1.0),
            details=f"axis={ax}",
        )

    # --- 4-column grid: keys in column 0 are left of column 3 ---
    k_left = object_model.get_part("key_0")   # AC at col 0, row 0
    k_right = object_model.get_part("key_3")  # ÷ at col 3, row 0
    left_pos = ctx.part_world_position(k_left)
    right_pos = ctx.part_world_position(k_right)
    if left_pos is not None and right_pos is not None:
        ctx.check(
            "grid spans 4 columns left-to-right",
            right_pos[0] > left_pos[0] + 0.025,
            details=f"left_x={left_pos[0]:.4f} right_x={right_pos[0]:.4f}",
        )

    # --- Keys are seated in the body face (proud at rest), not floating ---
    body_top = body_aabb[1][2] if body_aabb is not None else FACE_Z
    # Check key_4 (digit '7', col 0, row 1) as representative
    k7 = object_model.get_part("key_4")
    k7_aabb = ctx.part_world_aabb(k7)
    if k7_aabb is not None:
        key_top = k7_aabb[1][2]
        key_bottom = k7_aabb[0][2]
        ctx.check(
            "key '7' rises above the body face at rest",
            key_top > body_top - 0.0005,
            details=f"key_top={key_top:.4f} body_top={body_top:.4f}",
        )
        ctx.check(
            "key '7' is captured down inside its well",
            key_bottom < body_top - 0.0005,
            details=f"key_bottom={key_bottom:.4f} body_top={body_top:.4f}",
        )
    ctx.expect_overlap(
        k7,
        body,
        axes="xy",
        min_overlap=0.005,
        name="key '7' overlaps body footprint",
    )

    # --- Pressing the '=' key (key_19) moves it DOWN by the stroke ---
    eq_joint = object_model.get_articulation("body_to_key_19")
    eq_part = object_model.get_part("key_19")
    rest_pos = ctx.part_world_position(eq_part)
    with ctx.pose({eq_joint: KEY_TRAVEL}):
        pressed_pos = ctx.part_world_position(eq_part)
    if rest_pos is not None and pressed_pos is not None:
        drop = rest_pos[2] - pressed_pos[2]
        ctx.check(
            "pressing '=' moves the key down by the stroke",
            abs(drop - KEY_TRAVEL) < 1e-4 and drop > 0.0,
            details=f"drop={drop:.5f} expected={KEY_TRAVEL:.5f}",
        )

    return ctx.report()


object_model = build_object_model()
