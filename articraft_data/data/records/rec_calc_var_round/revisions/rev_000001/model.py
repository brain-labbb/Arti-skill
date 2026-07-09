from __future__ import annotations

# Round / pill variant of the pocket calculator.
#
# Variant change: body shell is an elliptical (oval / pill) slab instead of a
# rectangular slab.  Every key cap is a circular disc instead of a rounded
# square.  Same key grid arrangement, same prismatic press mechanism, same
# display panel and solar window, same colours and materials.
#
# Reference: picture/Stationary/Calculater/001.png
#   - Portrait slate-blue plastic body (now elliptical footprint).
#   - Dark recessed LCD display occupies the upper-third of the face.
#   - Small dark solar-cell window sits at the very top.
#   - Cream circular keys in a grid below the display, including a larger
#     "+" key disc at the lower right.
#
# Primary mechanism: press keys travel a short linear distance DOWN into
# their circular wells -> PRISMATIC joints with a small downward stroke.

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
# Real-world dimensions (metres).  Elliptical pocket / desktop calculator.
# ---------------------------------------------------------------------------
BODY_RX = 0.050          # semi-axis X  (half-width)
BODY_RY = 0.078          # semi-axis Y  (half-length, portrait)
BODY_T  = 0.012          # thickness (Z)

FACE_Z = BODY_T          # top face of body

# Display recess (dark LCD window – still rectangular)
DISPLAY_W = 0.058
DISPLAY_L = 0.030
DISPLAY_Y = 0.040        # centre Y
DISPLAY_DEPTH = 0.0022
DISPLAY_LCD_INSET = 0.004

# Solar-cell window (small dark bar at very top – still rectangular)
SOLAR_W = 0.030
SOLAR_L = 0.011
SOLAR_Y = 0.058
SOLAR_DEPTH = 0.0018

# Key geometry
KEY_TRAVEL = 0.0015      # downward press stroke
KEY_PROUD  = 0.0026      # how far a key rises above the body face at rest
WELL_DEPTH = 0.0020      # recessed key-well depth cut into the face
WELL_CLEAR = 0.0010      # radial clearance between key edge and well wall

# Circular key radii
KEY_RADIUS_STD  = 0.0054   # standard digit / operator key
KEY_RADIUS_CTRL = 0.0062   # wider control-row keys (±, OFF)
KEY_RADIUS_PLUS = 0.0078   # big "+" key (larger disc)


# ---------------------------------------------------------------------------
# Keypad layout (same grid as parent; keys are now circular)
# ---------------------------------------------------------------------------
COL_X = [-0.0276, -0.0138, 0.0, 0.0138, 0.0276]

ROW_Y = {
    "ctrl": 0.0150,
    "r0":   0.0010,
    "r1":  -0.0130,
    "r2":  -0.0270,
    "r3":  -0.0410,
    "r4":  -0.0550,
}


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _rounded_rect_panel(width: float, length: float, height: float, radius: float):
    """Rectangular panel with filleted vertical edges (for display / solar)."""
    r = min(radius, min(width, length) * 0.3)
    return (
        cq.Workplane("XY")
        .box(width, length, height, centered=(True, True, False))
        .edges("|Z")
        .fillet(r)
    )


def _build_body_mesh():
    """Elliptical slab with display + solar recesses and circular key wells,
    all cut from a single solid so the face reads as one continuous molded
    piece."""
    body = (
        cq.Workplane("XY")
        .ellipse(BODY_RX, BODY_RY)
        .extrude(BODY_T)
    )

    # Subtle top-edge rounding for a manufactured look.
    body = body.edges(">Z").fillet(0.001)

    # Display recess (rectangular)
    disp_cut = (
        cq.Workplane("XY")
        .workplane(offset=FACE_Z)
        .center(0.0, DISPLAY_Y)
        .rect(DISPLAY_W, DISPLAY_L)
        .extrude(-DISPLAY_DEPTH)
    )
    body = body.cut(disp_cut)

    # Solar-cell window recess (rectangular)
    solar_cut = (
        cq.Workplane("XY")
        .workplane(offset=FACE_Z)
        .center(0.0, SOLAR_Y)
        .rect(SOLAR_W, SOLAR_L)
        .extrude(-SOLAR_DEPTH)
    )
    body = body.cut(solar_cut)

    # Circular key wells
    for kx, ky, kr in _key_well_circles():
        well = (
            cq.Workplane("XY")
            .workplane(offset=FACE_Z)
            .center(kx, ky)
            .circle(kr + WELL_CLEAR)
            .extrude(-WELL_DEPTH)
        )
        body = body.cut(well)

    return mesh_from_cadquery(body, "calculator_body")


def _build_display_panel_mesh():
    """Dark LCD glass panel seated inside the display recess."""
    panel = _rounded_rect_panel(
        DISPLAY_W - DISPLAY_LCD_INSET,
        DISPLAY_L - DISPLAY_LCD_INSET,
        0.0016,
        0.002,
    )
    return mesh_from_cadquery(panel, "calculator_display")


def _build_solar_panel_mesh():
    """Dark solar-cell strip seated in the top window."""
    panel = _rounded_rect_panel(SOLAR_W - 0.003, SOLAR_L - 0.003, 0.0012, 0.0015)
    return mesh_from_cadquery(panel, "calculator_solar")


def _build_key_mesh(name: str, radius: float):
    """A single circular disc key cap with a domed top edge."""
    height = KEY_PROUD + WELL_DEPTH
    key = (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(height)
        .edges(">Z")
        .fillet(min(0.0006, radius * 0.25))
    )
    return mesh_from_cadquery(key, name)


# ---------------------------------------------------------------------------
# Key specs & well layout
# ---------------------------------------------------------------------------

def _key_specs():
    """Return list of (key_id, label, cx, cy, radius) for every key."""
    specs: list[tuple[str, str, float, float, float]] = []

    # Control pair: +/- and OFF at upper-right under the display.
    specs.append(("plus_minus", "+/-", COL_X[2], ROW_Y["ctrl"], KEY_RADIUS_CTRL))
    specs.append(("off", "OFF", COL_X[3] + 0.001, ROW_Y["ctrl"], KEY_RADIUS_CTRL))

    # Row r0: MU MMC M- M+ +
    for i, kid in enumerate(["mu", "mmc", "m_minus", "m_plus", "plus_op"]):
        specs.append((kid, kid, COL_X[i], ROW_Y["r0"], KEY_RADIUS_STD))

    # Row r1: % 7 8 9 ×
    for i, kid in enumerate(["percent", "seven", "eight", "nine", "multiply"]):
        specs.append((kid, kid, COL_X[i], ROW_Y["r1"], KEY_RADIUS_STD))

    # Row r2: √ 4 5 6 -
    for i, kid in enumerate(["sqrt", "four", "five", "six", "minus_op"]):
        specs.append((kid, kid, COL_X[i], ROW_Y["r2"], KEY_RADIUS_STD))

    # Row r3: C/CE 1 2 3  (col 4 reserved for big +)
    for i, kid in enumerate(["c_ce", "one", "two", "three"]):
        specs.append((kid, kid, COL_X[i], ROW_Y["r3"], KEY_RADIUS_STD))

    # Row r4: ON 0 . =  (col 4 reserved for big +)
    for i, kid in enumerate(["on", "zero", "decimal", "equals"]):
        specs.append((kid, kid, COL_X[i], ROW_Y["r4"], KEY_RADIUS_STD))

    # Big "+" – larger circular disc at the midpoint of rows r3..r4, col 4
    tall_cy = (ROW_Y["r3"] + ROW_Y["r4"]) / 2.0
    specs.append(("plus_big", "+", COL_X[4], tall_cy, KEY_RADIUS_PLUS))

    return specs


def _key_well_circles():
    """(cx, cy, radius) tuples for circular key wells cut into the body."""
    return [(cx, cy, kr) for (_kid, _lbl, cx, cy, kr) in _key_specs()]


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="round_calculator")

    body_mat    = model.material("body_slate",   rgba=(0.42, 0.47, 0.58, 1.0))
    display_mat = model.material("display_dark", rgba=(0.08, 0.07, 0.06, 1.0))
    solar_mat   = model.material("solar_dark",   rgba=(0.10, 0.09, 0.08, 1.0))
    key_mat     = model.material("key_cream",    rgba=(0.92, 0.86, 0.74, 1.0))
    key_accent  = model.material("key_accent",   rgba=(0.86, 0.80, 0.68, 1.0))

    # ---- Root body shell (elliptical slab) ----
    body = model.part("body")
    body.visual(_build_body_mesh(), material=body_mat, name="body_shell")
    body.inertial = Inertial.from_geometry(
        Box((2.0 * BODY_RX, 2.0 * BODY_RY, BODY_T)),
        mass=0.09,
        origin=Origin(xyz=(0.0, 0.0, BODY_T / 2.0)),
    )

    # ---- Display panel (fixed, seated in recess) ----
    display = model.part("display")
    display.visual(_build_display_panel_mesh(), material=display_mat, name="lcd")
    model.articulation(
        "body_to_display",
        ArticulationType.FIXED,
        parent=body,
        child=display,
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

    # ---- Keys: each is a prismatic press key (circular disc) ----
    key_mesh_cache: dict[float, object] = {}

    accent_ids = {
        "plus_op", "multiply", "minus_op", "plus_big",
        "equals", "off", "on", "plus_minus",
    }

    for kid, _label, cx, cy, radius in _key_specs():
        cache_key = round(radius, 5)
        mesh = key_mesh_cache.get(cache_key)
        if mesh is None:
            mesh = _build_key_mesh(f"key_{kid}", radius)
            key_mesh_cache[cache_key] = mesh

        key_part = model.part(f"key_{kid}")
        mat = key_accent if kid in accent_ids else key_mat
        key_part.visual(mesh, material=mat, name="key_cap")
        key_part.inertial = Inertial.from_geometry(
            Box((2.0 * radius, 2.0 * radius, KEY_PROUD + WELL_DEPTH)),
            mass=0.0015,
            origin=Origin(xyz=(0.0, 0.0, (KEY_PROUD + WELL_DEPTH) / 2.0)),
        )

        # Prismatic press: joint frame at well floor, axis -Z so positive q
        # presses the key DOWN into the well.
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


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body    = object_model.get_part("body")
    display = object_model.get_part("display")
    solar   = object_model.get_part("solar_window")

    # --- Body silhouette: elliptical slab at pocket-calculator scale ---
    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is not None:
        (bx0, by0, bz0), (bx1, by1, bz1) = body_aabb
        width  = bx1 - bx0
        length = by1 - by0
        thick  = bz1 - bz0
        ctx.check(
            "body is elliptical pocket-calculator scale",
            0.08 <= width <= 0.12 and 0.13 <= length <= 0.18 and length > width,
            details=f"w={width:.4f} l={length:.4f}",
        )
        ctx.check(
            "body is a thin slab",
            0.008 <= thick <= 0.016,
            details=f"thick={thick:.4f}",
        )
        # Aspect ratio should match the ellipse semi-axis ratio.
        expected_ratio = BODY_RX / BODY_RY
        actual_ratio = width / length if length > 0 else 0.0
        ctx.check(
            "body has elliptical aspect ratio",
            abs(actual_ratio - expected_ratio) < 0.05,
            details=f"expected={expected_ratio:.3f} actual={actual_ratio:.3f}",
        )

    # --- Display within body footprint, upper third ---
    ctx.expect_within(
        display, body, axes="xy", margin=0.002,
        name="display sits within body footprint",
    )
    disp_aabb = ctx.part_world_aabb(display)
    if disp_aabb is not None:
        disp_cy = (disp_aabb[0][1] + disp_aabb[1][1]) / 2.0
        ctx.check(
            "display sits in the upper third",
            disp_cy > 0.02,
            details=f"disp_cy={disp_cy:.4f}",
        )

    # --- Solar window above the display ---
    ctx.expect_within(
        solar, body, axes="xy", margin=0.002,
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

    # --- Hero keys: prismatic, -Z axis ---
    hero_keys = [
        "seven", "eight", "nine", "zero", "equals",
        "plus_big", "on", "off", "c_ce", "decimal",
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

    # --- Keys are circular discs: X extent ≈ Y extent ---
    seven = object_model.get_part("key_seven")
    sv_aabb = ctx.part_world_aabb(seven)
    if sv_aabb is not None:
        x_ext = sv_aabb[1][0] - sv_aabb[0][0]
        y_ext = sv_aabb[1][1] - sv_aabb[0][1]
        ctx.check(
            "key '7' is a circular disc (X extent ≈ Y extent)",
            abs(x_ext - y_ext) < 0.001,
            details=f"x={x_ext:.4f} y={y_ext:.4f}",
        )

    # --- Big plus key is a larger disc than standard digit keys ---
    plus_big = object_model.get_part("key_plus_big")
    pb_aabb = ctx.part_world_aabb(plus_big)
    if pb_aabb is not None and sv_aabb is not None:
        pb_ext = pb_aabb[1][0] - pb_aabb[0][0]
        sv_ext = sv_aabb[1][0] - sv_aabb[0][0]
        ctx.check(
            "big plus key disc is larger than standard digit key disc",
            pb_ext > sv_ext * 1.2,
            details=f"plus={pb_ext:.4f} digit={sv_ext:.4f}",
        )

    # --- Keys seated in the body face (proud at rest) ---
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
        seven, body, axes="xy", min_overlap=0.005,
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
