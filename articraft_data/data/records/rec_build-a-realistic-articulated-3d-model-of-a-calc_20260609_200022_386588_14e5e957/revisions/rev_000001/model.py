from __future__ import annotations

# Realistic articulated pocket calculator, modeled from the reference image.
#
# Reference: picture/Stationary/Calculater/001.png
#   - Portrait slate-blue plastic body with rounded corners.
#   - A dark recessed LCD display occupies the upper-third of the face.
#   - A small dark solar-cell window sits at the very top.
#   - Cream rounded keys are arranged in a grid below the display,
#     including a tall "+" key spanning two rows at the lower right.
#
# Primary user-facing mechanism: the keys are press keys. Each key travels
# a short linear distance DOWN into its well when pressed -> PRISMATIC joints
# (one per key) with a small downward stroke. The body shell is the fixed root.

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
    # (cosmetic shallow groove around the perimeter)
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


def _build_key_mesh(name: str, width: float, length: float):
    """A single cream key: a low rounded cap with a domed-ish top."""
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
# Keypad layout
# ---------------------------------------------------------------------------
# Standard keys are square-ish; some are wider. Coordinates are face-local
# (X right, Y up). The keypad lives below the display.
#
# Grid: 5 columns. Column X centers:
COL_X = [-0.0276, -0.0138, 0.0, 0.0138, 0.0276]
COL_PITCH = 0.0138
KEY_W = 0.0108
KEY_L = 0.0108

# Top control pair row (just under display): +/- and OFF on the right.
# Main grid rows from top to bottom.
ROW_Y = {
    "ctrl": 0.0150,    # +/- , OFF
    "r0": 0.0010,      # MU MMC M- M+ +
    "r1": -0.0130,     # % 7 8 9 ×
    "r2": -0.0270,     # √ 4 5 6 -
    "r3": -0.0410,     # C/CE 1 2 3   (+ tall spans r3..r4 col4)
    "r4": -0.0550,     # ON 0 . =
}


def _key_specs():
    """Return list of (key_id, label, cx, cy, w, l) for every key.

    Also yields the special tall '+' key which spans rows r3..r4 in column 4.
    """
    specs: list[tuple[str, str, float, float, float, float]] = []

    # Control pair: +/- and OFF, sitting at upper-right under the display.
    specs.append(("plus_minus", "+/-", COL_X[2], ROW_Y["ctrl"], 0.0124, KEY_L))
    specs.append(("off", "OFF", COL_X[3] + 0.0010, ROW_Y["ctrl"], 0.0124, KEY_L))

    # Row r0: MU MMC M- M+ +
    r0_ids = ["mu", "mmc", "m_minus", "m_plus", "plus_op"]
    for col, kid in enumerate(r0_ids):
        specs.append((kid, kid, COL_X[col], ROW_Y["r0"], KEY_W, KEY_L))

    # Row r1: % 7 8 9 ×
    r1_ids = ["percent", "seven", "eight", "nine", "multiply"]
    for col, kid in enumerate(r1_ids):
        specs.append((kid, kid, COL_X[col], ROW_Y["r1"], KEY_W, KEY_L))

    # Row r2: √ 4 5 6 -
    r2_ids = ["sqrt", "four", "five", "six", "minus_op"]
    for col, kid in enumerate(r2_ids):
        specs.append((kid, kid, COL_X[col], ROW_Y["r2"], KEY_W, KEY_L))

    # Row r3: C/CE 1 2 3  (column 4 reserved for tall '+')
    r3_ids = ["c_ce", "one", "two", "three"]
    for col, kid in enumerate(r3_ids):
        specs.append((kid, kid, COL_X[col], ROW_Y["r3"], KEY_W, KEY_L))

    # Row r4: ON 0 . =  (column 4 reserved for tall '+')
    r4_ids = ["on", "zero", "decimal", "equals"]
    for col, kid in enumerate(r4_ids):
        specs.append((kid, kid, COL_X[col], ROW_Y["r4"], KEY_W, KEY_L))

    # Tall '+' (the big plus) spanning r3..r4 in column 4.
    tall_cy = (ROW_Y["r3"] + ROW_Y["r4"]) / 2.0
    tall_l = (ROW_Y["r3"] - ROW_Y["r4"]) + KEY_L
    specs.append(("plus_big", "+", COL_X[4], tall_cy, KEY_W, tall_l))

    return specs


def _key_well_rects():
    """Footprint rects (cx, cy, w, l) used to cut key wells in the body."""
    return [(cx, cy, w, l) for (_kid, _lbl, cx, cy, w, l) in _key_specs()]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pocket_calculator")

    body_mat = model.material("body_slate", rgba=(0.42, 0.47, 0.58, 1.0))
    display_mat = model.material("display_dark", rgba=(0.08, 0.07, 0.06, 1.0))
    solar_mat = model.material("solar_dark", rgba=(0.10, 0.09, 0.08, 1.0))
    key_mat = model.material("key_cream", rgba=(0.92, 0.86, 0.74, 1.0))
    key_accent = model.material("key_accent", rgba=(0.86, 0.80, 0.68, 1.0))

    # ---- Root body shell ----
    body = model.part("body")
    body.visual(_build_body_mesh(), material=body_mat, name="body_shell")
    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_L, BODY_T)),
        mass=0.09,
        origin=Origin(xyz=(0.0, 0.0, BODY_T / 2.0)),
    )

    # ---- Display panel (fixed, seated in its recess) ----
    display = model.part("display")
    display.visual(_build_display_panel_mesh(), material=display_mat, name="lcd")
    model.articulation(
        "body_to_display",
        ArticulationType.FIXED,
        parent=body,
        child=display,
        # Seat the LCD bottom flush with the recess floor.
        origin=Origin(xyz=(0.0, DISPLAY_Y, FACE_Z - DISPLAY_DEPTH)),
    )

    # ---- Solar window (fixed) ----
    solar = model.part("solar_window")
    solar.visual(_build_solar_panel_mesh(), material=solar_mat, name="solar_cell")
    model.articulation(
        "body_to_solar",
        ArticulationType.FIXED,
        parent=body,
        child=solar,
        origin=Origin(xyz=(0.0, SOLAR_Y, FACE_Z - SOLAR_DEPTH)),
    )

    # ---- Keys: each is a prismatic press key ----
    # Cache key meshes by footprint so identical keys reuse geometry.
    key_mesh_cache: dict[tuple[float, float], object] = {}

    accent_ids = {
        "plus_op",
        "multiply",
        "minus_op",
        "plus_big",
        "equals",
        "off",
        "on",
        "plus_minus",
    }

    for kid, _label, cx, cy, w, l in _key_specs():
        cache_key = (round(w, 5), round(l, 5))
        mesh = key_mesh_cache.get(cache_key)
        if mesh is None:
            mesh = _build_key_mesh(f"key_{kid}", w, l)
            key_mesh_cache[cache_key] = mesh

        key_part = model.part(f"key_{kid}")
        mat = key_accent if kid in accent_ids else key_mat
        # Key mesh sits on its part frame z=0; the part frame is at the
        # well floor, so the key bottom is captured inside the well.
        key_part.visual(mesh, material=mat, name="key_cap")
        key_part.inertial = Inertial.from_geometry(
            Box((w, l, KEY_PROUD + WELL_DEPTH)),
            mass=0.0015,
            origin=Origin(xyz=(0.0, 0.0, (KEY_PROUD + WELL_DEPTH) / 2.0)),
        )

        # Prismatic press: joint frame at the well floor, axis -Z so positive
        # q presses the key DOWN into the well.
        model.articulation(
            f"body_to_key_{kid}",
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
    display = object_model.get_part("display")
    solar = object_model.get_part("solar_window")

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

    # --- Display present, seated near the top, inside the body footprint ---
    ctx.expect_within(
        display,
        body,
        axes="xy",
        margin=0.001,
        name="display sits within body footprint",
    )
    ctx.expect_overlap(
        display,
        body,
        axes="xy",
        min_overlap=0.020,
        name="display covers the upper face area",
    )
    disp_aabb = ctx.part_world_aabb(display)
    if disp_aabb is not None:
        disp_cy = (disp_aabb[0][1] + disp_aabb[1][1]) / 2.0
        ctx.check(
            "display sits in the upper third",
            disp_cy > 0.02,
            details=f"disp_cy={disp_cy:.4f}",
        )

    # --- Solar window present, above the display ---
    ctx.expect_within(
        solar,
        body,
        axes="xy",
        margin=0.001,
        name="solar window within body footprint",
    )
    solar_aabb = ctx.part_world_aabb(solar)
    if solar_aabb is not None and disp_aabb is not None:
        solar_cy = (solar_aabb[0][1] + solar_aabb[1][1]) / 2.0
        disp_top = disp_aabb[1][1]
        ctx.check(
            "solar window is above the display",
            solar_cy > disp_top,
            details=f"solar_cy={solar_cy:.4f} disp_top={disp_top:.4f}",
        )

    # --- Hero keys exist with correct joint type and axis ---
    hero_keys = [
        "seven",
        "eight",
        "nine",
        "zero",
        "equals",
        "plus_big",
        "on",
        "off",
        "c_ce",
        "decimal",
    ]
    for kid in hero_keys:
        key_part = object_model.get_part(f"key_{kid}")
        joint = object_model.get_articulation(f"body_to_key_{kid}")
        jtype = str(joint.articulation_type).lower()
        ctx.check(
            f"key_{kid} is prismatic",
            "prismatic" in jtype,
            details=f"type={jtype}",
        )
        ax = tuple(round(c, 3) for c in joint.axis)
        ctx.check(
            f"key_{kid} presses along -Z",
            ax == (0.0, 0.0, -1.0),
            details=f"axis={ax}",
        )

    # The big '+' key is taller than a standard digit key (spans two rows).
    plus_big = object_model.get_part("key_plus_big")
    seven = object_model.get_part("key_seven")
    pb_aabb = ctx.part_world_aabb(plus_big)
    sv_aabb = ctx.part_world_aabb(seven)
    if pb_aabb is not None and sv_aabb is not None:
        pb_len = pb_aabb[1][1] - pb_aabb[0][1]
        sv_len = sv_aabb[1][1] - sv_aabb[0][1]
        ctx.check(
            "big plus key spans two rows",
            pb_len > sv_len * 1.6,
            details=f"plus_len={pb_len:.4f} digit_len={sv_len:.4f}",
        )

    # --- Keys are seated in the body face (proud at rest), not floating ---
    # The '7' key should sit at/above the body top face at rest and overlap
    # the body footprint (captured in its well).
    body_top = body_aabb[1][2] if body_aabb is not None else FACE_Z
    if sv_aabb is not None:
        key_top = sv_aabb[1][2]
        key_bottom = sv_aabb[0][2]
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
        seven,
        body,
        axes="xy",
        min_overlap=0.005,
        name="key '7' overlaps body footprint",
    )

    # --- Pressing a key moves it DOWN by the stroke ---
    eq_joint = object_model.get_articulation("body_to_key_equals")
    equals = object_model.get_part("key_equals")
    rest_pos = ctx.part_world_position(equals)
    with ctx.pose({eq_joint: KEY_TRAVEL}):
        pressed_pos = ctx.part_world_position(equals)
    if rest_pos is not None and pressed_pos is not None:
        drop = rest_pos[2] - pressed_pos[2]
        ctx.check(
            "pressing '=' moves the key down by the stroke",
            abs(drop - KEY_TRAVEL) < 1e-4 and drop > 0.0,
            details=f"drop={drop:.5f} expected={KEY_TRAVEL:.5f}",
        )

    return ctx.report()


object_model = build_object_model()
