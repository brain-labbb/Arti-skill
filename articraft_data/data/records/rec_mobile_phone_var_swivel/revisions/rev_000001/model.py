from __future__ import annotations

# Nokia 3310 swivel / rotator phone variant.
# Frame: face points up (+Z). Phone is a tall candybar at rest (q=0):
#   - Y = long axis (tall), top at +Y near the earpiece/screen, keypad toward -Y
#   - X = width (narrow)
#   - Z = thickness; the front face is the +Z surface, keys press DOWN (-Z)
#
# The monoblock is split into two slabs at Y = SPLIT_Y:
#   - body (root): lower keypad slab with all 15 pressable keys
#   - screen_slab: upper screen slab carrying screen, earpiece, logo
# They share a corner pivot post at (PIVOT_X, SPLIT_Y) with a REVOLUTE
# joint about +Z so the screen slab swings ~180° to reveal the keypad.
#
# Two-tone colorway: dark navy lower body + bright steel-blue upper slab.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    ExtrudeGeometry,
    boolean_difference,
    boolean_union,
    mesh_from_geometry,
    rounded_rect_profile,
)

# ---- overall dimensions ----
BODY_W = 0.048   # X width
BODY_H = 0.110   # Y height (tall candybar)
BODY_T = 0.022   # Z thickness
RIM_Z = 0.0105   # top of the front shell (the navy rim plane around the fascia)
FASCIA_TOP_Z = RIM_Z + 0.0003

# ---- swivel split line ----
SPLIT_Y = 0.013
LOWER_H = BODY_H / 2.0 + SPLIT_Y        # 0.068
UPPER_H = BODY_H / 2.0 - SPLIT_Y        # 0.042
LOWER_CY = (SPLIT_Y - BODY_H / 2.0) / 2.0  # -0.021
UPPER_CY_BODY = (BODY_H / 2.0 + SPLIT_Y) / 2.0  # 0.034

# ---- pivot at the left corner of the split line ----
PIVOT_X = -0.019
PIVOT_Y = SPLIT_Y
PIVOT_R_POST = 0.004
PIVOT_R_HUB = 0.005
PIVOT_LENGTH = 0.024  # full length through both slabs

# ---- screen slab local coords (body coord - pivot coord) ----
SLAB_DX = -PIVOT_X   # 0.019
SLAB_DY = -PIVOT_Y   # -0.013
SLAB_SHELL_CX = SLAB_DX                        # 0.019
SLAB_SHELL_CY = UPPER_CY_BODY + SLAB_DY        # 0.021

# ---- keypad constants (unchanged from parent) ----
POCKET_DEPTH = 0.0030
POCKET_FLOOR_Z = RIM_Z - POCKET_DEPTH
FUNC_POCKET_W = 0.0420
FUNC_POCKET_H = 0.0128
FUNC_POCKET_CY = 0.0062
NUM_POCKET_W = 0.0395
NUM_POCKET_H = 0.0500
NUM_POCKET_CY = -0.0246

KEY_SEAT_Z = POCKET_FLOOR_Z
KEY_RISE = 0.0025
PRESS_TRAVEL = 0.0010

# ---- screen slab feature Y positions (in local coords) ----
_display_shift_y = 0.0045
SLAB_SCREEN_CX = SLAB_DX + 0.0
SLAB_SCREEN_CY = (0.0215 + _display_shift_y) + SLAB_DY   # 0.013
SLAB_EAR_CY = 0.047475 + SLAB_DY                          # 0.034475
SLAB_LOGO_CY = (0.0372 + _display_shift_y) + SLAB_DY      # 0.0287


# ---- helpers ----

def _rot_pts(pts: list[tuple[float, float]], ang: float) -> list[tuple[float, float]]:
    c, s = math.cos(ang), math.sin(ang)
    return [(x * c - y * s, x * s + y * c) for x, y in pts]


def _translated_rounded_slab(
    w: float, h: float, z0: float, t: float, r: float, name: str,
    *, x: float = 0.0, y: float = 0.0,
):
    geom = ExtrudeGeometry.from_z0(rounded_rect_profile(w, h, r, corner_segments=10), t, cap=True)
    geom.translate(x, y, z0)
    return mesh_from_geometry(geom, name)


def _shell_sections():
    """Z-stacking sections for the smoothly rounded phone shell edge."""
    return [
        (-0.0110, 0.0016, 0.0030),
        (-0.0099, 0.0017, 0.0018),
        (-0.0088, 0.0019, 0.0010),
        (-0.0075, 0.0021, 0.0005),
        (-0.0060, 0.0021, 0.0002),
        (-0.0045, 0.0084, 0.0000),
        (0.0034, 0.0022, 0.0002),
        (0.0051, 0.0020, 0.0006),
        (0.0066, 0.0018, 0.0012),
        (0.0079, 0.0017, 0.0018),
        (0.0091, 0.0014, 0.0026),
    ]


def _build_shell_geom(half_w: float, half_h: float, cx: float, cy: float):
    """Build Z-stacked rounded-rect shell geometry for one slab."""
    geom = None
    for z0, t, inset in _shell_sections():
        w = half_w - 2.0 * inset
        h = half_h - 2.0 * inset
        r = max(0.016 - inset * 0.5, 0.002)
        if w < 0.004 or h < 0.004:
            continue
        prof = rounded_rect_profile(w, h, r, corner_segments=10)
        slab = ExtrudeGeometry.from_z0(prof, t, cap=True)
        slab.translate(cx, cy, z0)
        geom = slab if geom is None else boolean_union(geom, slab)
    return geom


def _lower_shell_mesh():
    """Lower keypad slab shell with recessed keypad pockets."""
    geom = _build_shell_geom(BODY_W, LOWER_H, 0.0, LOWER_CY)
    for w, h, cy, r in (
        (FUNC_POCKET_W, FUNC_POCKET_H, FUNC_POCKET_CY, 0.0045),
        (NUM_POCKET_W, NUM_POCKET_H, NUM_POCKET_CY, 0.0055),
    ):
        pocket = ExtrudeGeometry.from_z0(
            rounded_rect_profile(w, h, r, corner_segments=8),
            POCKET_DEPTH + 0.003, cap=True,
        )
        pocket.translate(0.0, cy, POCKET_FLOOR_Z)
        geom = boolean_difference(geom, pocket)
    return mesh_from_geometry(geom, "body_shell")


def _upper_shell_mesh():
    """Upper screen slab shell (in screen_slab local coords)."""
    geom = _build_shell_geom(BODY_W, UPPER_H, SLAB_SHELL_CX, SLAB_SHELL_CY)
    return mesh_from_geometry(geom, "screen_slab_shell")


def _lower_fascia_mesh():
    """Simplified fascia covering the function key cluster on the lower body."""
    w, h, cy, r = 0.042, 0.022, 0.002, 0.005
    geom = ExtrudeGeometry.from_z0(
        rounded_rect_profile(w, h, r, corner_segments=8), 0.0009, cap=True,
    )
    geom.translate(0.0, cy, RIM_Z - 0.0006)
    holes = [
        (0.0, 0.0040, 0.0190, 0.0095, 0.0038, 0.0),
        (-0.0150, 0.0058, 0.0085, 0.0110, 0.0035, -0.18),
        (0.0150, 0.0058, 0.0085, 0.0110, 0.0035, 0.18),
    ]
    for hx, hy, hw, hh, hr, yaw in holes:
        pts = _rot_pts(rounded_rect_profile(hw, hh, hr, corner_segments=8), yaw)
        cut = ExtrudeGeometry.from_z0(pts, 0.004, cap=True)
        cut.translate(hx, hy, RIM_Z - 0.0016)
        geom = boolean_difference(geom, cut)
    return mesh_from_geometry(geom, "front_fascia")


def _arc_bar_mesh(name: str, w: float, h: float, curve: float, thickness: float = 0.00018):
    half_w = w / 2.0
    n = 10
    top: list[tuple[float, float]] = []
    bot: list[tuple[float, float]] = []
    for i in range(n + 1):
        t = i / n
        x = -half_w + w * t
        bow = curve * (1.0 - (2.0 * t - 1.0) ** 2)
        top.append((x, h / 2.0 + bow))
        bot.append((x, -h / 2.0 + bow))
    geom = ExtrudeGeometry.from_z0(top + list(reversed(bot)), thickness, cap=True)
    return mesh_from_geometry(geom, name)


def _slot_stack_mesh(name: str, count: int, slot_w: float, slot_h: float, pitch: float):
    """Earpiece grille: solid block with slot cutouts for robust connectivity."""
    total_h = pitch * (count - 1) + slot_h + 0.0004
    block = ExtrudeGeometry.from_z0(
        rounded_rect_profile(slot_w + 0.0004, total_h, slot_w * 0.25, corner_segments=4),
        0.00055, cap=True,
    )
    for i in range(count):
        y = (i - (count - 1) / 2.0) * pitch
        x = 0.0004 * math.sin((i - (count - 1) / 2.0) * 0.7)
        cut = ExtrudeGeometry.from_z0(
            rounded_rect_profile(slot_w - 0.0002, slot_h - 0.0001,
                                 min(slot_w, slot_h) * 0.35, corner_segments=4),
            0.0008, cap=True,
        )
        cut.translate(x, y, -0.0001)
        block = boolean_difference(block, cut)
    return mesh_from_geometry(block, name)


def _stroke_box(part, name: str, x: float, y: float, z: float,
                sx: float, sy: float, sz: float, material, yaw: float = 0.0):
    part.visual(
        Box((sx, sy, sz)),
        origin=Origin(xyz=(x, y, z), rpy=(0.0, 0.0, yaw)),
        material=material, name=name,
    )


def _add_nokia_wordmark(part, material, *, prefix: str, cx: float, cy: float,
                        z: float, scale: float, thickness: float = 0.00028):
    s = scale
    w = 4.0 * s
    h = 6.0 * s
    gap = 1.6 * s
    total = 5 * w + 4 * gap
    x0 = cx - total / 2.0

    def seg(letter_idx: int, seg_name: str, ox: float, oy: float,
            sx: float, sy: float, yaw: float = 0.0):
        lx = x0 + letter_idx * (w + gap) + ox
        _stroke_box(part, f"{prefix}_{letter_idx}_{seg_name}",
                    lx, cy + oy, z, sx, sy, thickness, material, yaw)

    seg(0, "left", 0.0, 0.0, s, h)
    seg(0, "right", w, 0.0, s, h)
    seg(0, "diag", w / 2.0, 0.0, s * 0.9, h * 1.15, math.radians(28))
    seg(1, "left", 0.0, 0.0, s, h)
    seg(1, "right", w, 0.0, s, h)
    seg(1, "top", w / 2.0, h / 2.0, w, s)
    seg(1, "bottom", w / 2.0, -h / 2.0, w, s)
    seg(2, "left", 0.0, 0.0, s, h)
    seg(2, "updiag", w * 0.55, h * 0.18, s * 0.9, h * 0.65, math.radians(-35))
    seg(2, "downdiag", w * 0.55, -h * 0.18, s * 0.9, h * 0.65, math.radians(35))
    seg(3, "stem", w / 2.0, 0.0, s, h)
    seg(3, "top", w / 2.0, h / 2.0, w * 0.85, s)
    seg(3, "bottom", w / 2.0, -h / 2.0, w * 0.85, s)
    seg(4, "left", w * 0.15, 0.0, s, h, math.radians(-8))
    seg(4, "right", w * 0.85, 0.0, s, h, math.radians(8))
    seg(4, "top", w / 2.0, h / 2.0, w * 0.75, s)
    seg(4, "mid", w / 2.0, 0.0, w * 0.65, s)


SEGMENTS = {
    "0": "abcfed", "1": "bc", "2": "abged", "3": "abgcd",
    "4": "fgbc", "5": "afgcd", "6": "afgecd", "7": "abc",
    "8": "abcdefg", "9": "abfgcd",
}


def _add_seven_segment_digit(part, material, digit: str, *, prefix: str,
                             z: float, scale: float = 1.0,
                             x: float = -0.0020, y: float = 0.00015):
    lw = 0.00042 * scale
    length = 0.0024 * scale
    vlen = 0.0020 * scale
    segs = SEGMENTS.get(digit, "")
    coords = {
        "a": (x, y + vlen, length, lw, 0.0),
        "b": (x + length / 2.0, y + vlen / 2.0, lw, vlen, 0.0),
        "c": (x + length / 2.0, y - vlen / 2.0, lw, vlen, 0.0),
        "d": (x, y - vlen, length, lw, 0.0),
        "e": (x - length / 2.0, y - vlen / 2.0, lw, vlen, 0.0),
        "f": (x - length / 2.0, y + vlen / 2.0, lw, vlen, 0.0),
        "g": (x, y, length, lw, 0.0),
    }
    for seg in segs:
        sx, sy, bx, by, yaw = coords[seg]
        _stroke_box(part, f"{prefix}_{seg}", sx, sy, z, bx, by, 0.00014, material, yaw)


def _add_key_label(part, material, label: str, *, prefix: str, kind: str):
    z = KEY_RISE - 0.00002
    if label.isdigit():
        _add_seven_segment_digit(part, material, label, prefix=prefix, z=z, scale=0.82)
        _stroke_box(part, f"{prefix}_letters", 0.0027, -0.0010, z,
                    0.0023, 0.00024, 0.00013, material)
    elif label == "*":
        for i, yaw in enumerate((0.0, math.radians(60), math.radians(-60))):
            _stroke_box(part, f"{prefix}_star_{i}", -0.0014, 0.0, z,
                        0.0034, 0.00034, 0.00013, material, yaw)
        _stroke_box(part, f"{prefix}_plus", 0.0026, -0.0010, z,
                    0.0011, 0.00025, 0.00013, material)
        _stroke_box(part, f"{prefix}_plus_v", 0.0026, -0.0010, z,
                    0.00025, 0.0011, 0.00013, material)
    elif label == "#":
        for i, xx in enumerate((-0.0008, 0.0008)):
            _stroke_box(part, f"{prefix}_hash_v{i}", xx, 0.0, z,
                        0.00028, 0.0030, 0.00013, material)
        for i, yy in enumerate((-0.0007, 0.0007)):
            _stroke_box(part, f"{prefix}_hash_h{i}", 0.0, yy, z,
                        0.0030, 0.00028, 0.00013, material)
    elif kind == "nav":
        part.visual(
            _arc_bar_mesh(f"{prefix}_cyan_smile", 0.0080, 0.0013, -0.0018),
            origin=Origin(xyz=(0.0, -0.0006, z)), material=material, name="menu_smile",
        )
    elif label == "C":
        _stroke_box(part, f"{prefix}_c_top", 0.0, 0.0011, z,
                    0.0025, 0.00038, 0.00013, material)
        _stroke_box(part, f"{prefix}_c_bottom", 0.0, -0.0011, z,
                    0.0025, 0.00038, 0.00013, material)
        _stroke_box(part, f"{prefix}_c_side", -0.00125, 0.0, z,
                    0.00038, 0.0022, 0.00013, material)
    else:
        _stroke_box(part, f"{prefix}_mark_a", -0.0004, 0.0, z,
                    0.0028, 0.00035, 0.00013, material, math.radians(35))
        _stroke_box(part, f"{prefix}_mark_b", 0.0004, 0.0, z,
                    0.0028, 0.00035, 0.00013, material, math.radians(-35))


def _keycap_mesh(name: str, w: float, h: float, r: float):
    base = ExtrudeGeometry.from_z0(
        rounded_rect_profile(w, h, r, corner_segments=6), KEY_RISE * 0.62, cap=True,
    )
    crown = ExtrudeGeometry.from_z0(
        rounded_rect_profile(w - 0.0016, h - 0.0012, max(r - 0.0006, 0.0006), corner_segments=6),
        KEY_RISE, cap=True,
    )
    base.merge(crown)
    return mesh_from_geometry(base, name)


def _smile_keycap_mesh(name: str, w: float, h: float, curve: float):
    half_w = w / 2.0
    half_h = h / 2.0
    n = 9
    top: list[tuple[float, float]] = []
    bot: list[tuple[float, float]] = []
    for i in range(n + 1):
        t = i / n
        x = -half_w + w * t
        bow = curve * (1.0 - (2.0 * t - 1.0) ** 2)
        top.append((x, half_h - bow * 0.35))
        bot.append((x, -half_h - bow))
    outline = top + list(reversed(bot))
    base = ExtrudeGeometry.from_z0(outline, KEY_RISE * 0.62, cap=True)
    crown_top = [(x, y - 0.0006) for (x, y) in top]
    crown_bot = [(x, y + 0.0006) for (x, y) in bot]
    crown_outline = [(x * 0.92, y) for (x, y) in (crown_top + list(reversed(crown_bot)))]
    crown = ExtrudeGeometry.from_z0(crown_outline, KEY_RISE, cap=True)
    base.merge(crown)
    return mesh_from_geometry(base, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nokia_3310_swivel")

    # ---- materials ----
    body_dark = model.material("glossy_dark_navy", rgba=(0.050, 0.070, 0.155, 1.0))
    slab_bright = model.material("bright_steel_blue", rgba=(0.58, 0.66, 0.76, 1.0))
    slate = model.material("slate_grey_fascia", rgba=(0.355, 0.370, 0.400, 1.0))
    silver = model.material("warm_silver_grey", rgba=(0.62, 0.64, 0.63, 1.0))
    well_dark = model.material("navy_key_well", rgba=(0.032, 0.045, 0.105, 1.0))
    key_light = model.material("pearl_silver_keys", rgba=(0.705, 0.715, 0.700, 1.0))
    screen_green = model.material("green_lcd", rgba=(0.585, 0.705, 0.485, 1.0))
    screen_frame_mat = model.material("black_display_bezel", rgba=(0.020, 0.022, 0.028, 1.0))
    print_dark = model.material("printed_black", rgba=(0.015, 0.018, 0.015, 1.0))
    plaque_light = model.material("logo_plaque_silver", rgba=(0.87, 0.89, 0.86, 1.0))
    cyan = model.material("navi_cyan", rgba=(0.00, 0.62, 0.76, 1.0))
    earpiece_mat = model.material("earpiece_black", rgba=(0.006, 0.006, 0.010, 1.0))
    gold = model.material("charge_contact_gold", rgba=(0.78, 0.64, 0.28, 1.0))

    # ================ BODY (lower keypad slab, root) ================
    body = model.part("body")
    body.visual(_lower_shell_mesh(), material=body_dark, name="body_shell")
    body.visual(_lower_fascia_mesh(), material=slate, name="front_fascia")

    # Pivot post: visible cylinder at the corner pivot
    body.visual(
        Cylinder(radius=PIVOT_R_POST, length=PIVOT_LENGTH),
        origin=Origin(xyz=(PIVOT_X, PIVOT_Y, 0.0)),
        material=silver, name="pivot_post",
    )

    # Side button (left edge of lower slab)
    body.visual(
        Box((0.0024, 0.0200, 0.0052)),
        origin=Origin(xyz=(-0.0235, -0.0170, 0.0012)),
        material=silver, name="side_button",
    )
    # Gold charging contact at the bottom
    body.visual(
        Box((0.0100, 0.0030, 0.0035)),
        origin=Origin(xyz=(0.0, -0.0538, 0.0010)),
        material=gold, name="bottom_contact",
    )

    # Well floors
    body.visual(
        _translated_rounded_slab(0.0408, 0.0120, POCKET_FLOOR_Z - 0.0008, 0.0009, 0.0040,
                                 "function_floor_mesh", y=FUNC_POCKET_CY),
        material=slate, name="function_floor",
    )
    body.visual(
        _translated_rounded_slab(0.0383, 0.0488, POCKET_FLOOR_Z - 0.0008, 0.0009, 0.0050,
                                 "keypad_fascia_panel_mesh", y=NUM_POCKET_CY),
        material=well_dark, name="keypad_fascia_panel",
    )

    body.inertial = Inertial.from_geometry(
        Box((BODY_W, LOWER_H, BODY_T)), mass=0.085,
        origin=Origin(xyz=(0.0, LOWER_CY, 0.0)),
    )

    # ================ SCREEN_SLAB (upper screen slab) ================
    screen_slab = model.part("screen_slab")
    screen_slab.visual(_upper_shell_mesh(), material=slab_bright, name="screen_slab_shell")

    # Pivot hub: wider cylinder on the screen_slab at the pivot corner
    screen_slab.visual(
        Cylinder(radius=PIVOT_R_HUB, length=PIVOT_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=silver, name="pivot_hub",
    )

    # Screen frame (dark bezel) - in slab local coords
    sw, sh = 0.0270, 0.0210
    screen_slab.visual(
        _translated_rounded_slab(sw + 0.0050, sh + 0.0050, RIM_Z - 0.0004, 0.0008, 0.0030,
                                 "screen_frame_mesh",
                                 x=SLAB_SCREEN_CX, y=SLAB_SCREEN_CY),
        material=screen_frame_mat, name="screen_frame",
    )
    # Screen glass (green LCD) proud of frame
    screen_slab.visual(
        Box((sw, sh, 0.0006)),
        origin=Origin(xyz=(SLAB_SCREEN_CX, SLAB_SCREEN_CY, RIM_Z + 0.0006)),
        material=screen_green, name="screen_glass",
    )

    # LCD art on the display (in slab local coords)
    lcd_z = RIM_Z + 0.00092
    scy = SLAB_SCREEN_CY
    for i in range(4):
        _stroke_box(screen_slab, f"screen_signal_{i}",
                    SLAB_SCREEN_CX - 0.0115 + i * 0.0012,
                    scy + 0.0082 + i * 0.00020, lcd_z,
                    0.0007, 0.00045 + i * 0.00035, 0.00008, print_dark)
    _stroke_box(screen_slab, "screen_battery",
                SLAB_SCREEN_CX + 0.0105, scy + 0.0086, lcd_z,
                0.0035, 0.0010, 0.00008, print_dark)
    _stroke_box(screen_slab, "screen_battery_nub",
                SLAB_SCREEN_CX + 0.0126, scy + 0.0086, lcd_z,
                0.0005, 0.00055, 0.00008, print_dark)
    for i, (px, py, sx, sy) in enumerate([
        (-0.0060, 0.0036, 0.0016, 0.0048),
        (-0.0039, 0.0016, 0.0017, 0.0043),
        (-0.0018, 0.0001, 0.0015, 0.0034),
        (0.0042, 0.0035, 0.0012, 0.0036),
        (0.0060, 0.0018, 0.0014, 0.0031),
    ]):
        _stroke_box(screen_slab, f"screen_pixel_art_{i}",
                    SLAB_SCREEN_CX + px, scy + py, lcd_z,
                    sx, sy, 0.00008, print_dark,
                    math.radians(-24 if i < 3 else 18))
    _add_nokia_wordmark(screen_slab, print_dark, prefix="screen_nokia",
                        cx=SLAB_SCREEN_CX + 0.0010, cy=scy - 0.0062,
                        z=lcd_z, scale=0.00060, thickness=0.00008)

    # Earpiece grille at top of screen slab (embedded into shell surface)
    screen_slab.visual(
        _slot_stack_mesh("earpiece_slot_mesh", 4, 0.0030, 0.0009, 0.00135),
        origin=Origin(xyz=(SLAB_DX, SLAB_EAR_CY, RIM_Z - 0.0002)),
        material=earpiece_mat, name="earpiece_slot",
    )

    # Silver NOKIA plaque between earpiece and screen (embedded into shell)
    screen_slab.visual(
        _translated_rounded_slab(0.0150, 0.0048, RIM_Z - 0.0002, 0.00042, 0.0010,
                                 "upper_logo_plaque",
                                 x=SLAB_DX, y=SLAB_LOGO_CY),
        material=plaque_light, name="upper_logo_plaque",
    )
    _add_nokia_wordmark(screen_slab, print_dark, prefix="upper_nokia",
                        cx=SLAB_DX, cy=SLAB_LOGO_CY,
                        z=RIM_Z + 0.00028, scale=0.00042, thickness=0.00015)

    screen_slab.inertial = Inertial.from_geometry(
        Box((BODY_W, UPPER_H, BODY_T)), mass=0.045,
        origin=Origin(xyz=(SLAB_SHELL_CX, SLAB_SHELL_CY, 0.0)),
    )

    # ================ SWIVEL ARTICULATION ================
    model.articulation(
        "body_to_screen_slab",
        ArticulationType.REVOLUTE,
        parent=body,
        child=screen_slab,
        origin=Origin(xyz=(PIVOT_X, PIVOT_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=0.0, upper=math.pi,
        ),
    )

    # ================ KEYS (all on lower body slab) ================
    key_specs: list[tuple[str, str, float, float, float, float, float]] = []

    # Function cluster
    key_specs.append(("key_nav", "nav", 0.0, 0.0044, 0.0190, 0.0070, 0.0))
    key_specs.append(("key_c", "soft", -0.0150, 0.0058, 0.0075, 0.0100, -0.18))
    key_specs.append(("key_arrow", "soft", 0.0150, 0.0058, 0.0075, 0.0100, 0.18))

    # Numeric keypad: 4 rows x 3 cols
    num_labels = [
        ["key_1", "key_2", "key_3"],
        ["key_4", "key_5", "key_6"],
        ["key_7", "key_8", "key_9"],
        ["key_star", "key_0", "key_hash"],
    ]
    col_x = (-0.0131, 0.0, 0.0131)
    col_yaw = (-0.10, 0.0, 0.10)
    row_y = (-0.0135, -0.0237, -0.0339, -0.0441)
    num_w, num_h = 0.0115, 0.0075
    for r, row in enumerate(num_labels):
        for c, label in enumerate(row):
            key_specs.append((label, "num", col_x[c], row_y[r], num_w, num_h, col_yaw[c]))

    key_mass = 0.0010
    for name, kind, kx, ky, kw, kh, yaw in key_specs:
        part = model.part(name)
        if kind == "nav":
            cap = _smile_keycap_mesh(f"{name}_cap", kw, kh, 0.0014)
            inertia_box = Box((kw, kh + 0.0014, KEY_RISE))
        elif kind == "soft":
            cap = _keycap_mesh(f"{name}_cap", kw, kh, 0.0030)
            inertia_box = Box((kw, kh, KEY_RISE))
        else:
            cap = _keycap_mesh(f"{name}_cap", kw, kh, 0.0030)
            inertia_box = Box((kw, kh, KEY_RISE))

        part.visual(cap, origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, yaw)),
                    material=key_light, name="keycap")
        if kind == "num":
            label = "*" if name == "key_star" else ("#" if name == "key_hash" else name.split("_")[-1])
            _add_key_label(part, print_dark, label, prefix=f"{name}_label", kind=kind)
        elif kind == "nav":
            _add_key_label(part, cyan, "menu", prefix=f"{name}_label", kind=kind)
        elif name == "key_c":
            _add_key_label(part, print_dark, "C", prefix=f"{name}_label", kind=kind)
        else:
            _add_key_label(part, print_dark, "arrow", prefix=f"{name}_label", kind=kind)
        part.inertial = Inertial.from_geometry(
            inertia_box, mass=key_mass, origin=Origin(xyz=(0.0, 0.0, KEY_RISE / 2.0))
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=part,
            origin=Origin(xyz=(kx, ky, KEY_SEAT_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=3.0, velocity=0.05, lower=0.0, upper=PRESS_TRAVEL,
            ),
        )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    screen_slab = object_model.get_part("screen_slab")
    swivel = object_model.get_articulation("body_to_screen_slab")

    # ---- combined phone at rest is still a tall candybar ----
    body_aabb = ctx.part_world_aabb(body)
    slab_aabb = ctx.part_world_aabb(screen_slab)
    combined_min = [min(body_aabb[0][i], slab_aabb[0][i]) for i in range(3)]
    combined_max = [max(body_aabb[1][i], slab_aabb[1][i]) for i in range(3)]
    combined_ext = (combined_max[0] - combined_min[0],
                    combined_max[1] - combined_min[1],
                    combined_max[2] - combined_min[2])
    ctx.check(
        "phone at rest is a tall candybar (Y >> X)",
        combined_ext[1] > 2.0 * combined_ext[0],
        details=f"combined extents={combined_ext}",
    )

    # ---- swivel joint: REVOLUTE about +Z with ~180° range ----
    ctx.check(
        "body_to_screen_slab is REVOLUTE about +Z with ~180° range",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in swivel.axis) == (0.0, 0.0, 1.0)
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower) < 1e-9
        and swivel.motion_limits.upper > 2.8,
        details=f"type={swivel.articulation_type}, axis={swivel.axis}, "
                f"limits={swivel.motion_limits}",
    )

    # ---- pivot post is a visible cylinder on the body ----
    pivot_aabb = ctx.part_element_world_aabb(body, elem="pivot_post")
    ctx.check(
        "pivot post is a visible cylinder (diameter > 6mm)",
        pivot_aabb is not None and _ext(pivot_aabb)[0] > 0.006,
        details=f"pivot_post aabb ext={_ext(pivot_aabb) if pivot_aabb else None}",
    )

    # ---- pivot hub on screen_slab at local origin ----
    hub_aabb = ctx.part_element_world_aabb(screen_slab, elem="pivot_hub")
    ctx.check(
        "pivot hub is visible on screen_slab",
        hub_aabb is not None and _ext(hub_aabb)[0] > 0.006,
        details=f"pivot_hub ext={_ext(hub_aabb) if hub_aabb else None}",
    )

    # ---- pivot post/hub intentional overlap (post inside hub bushing) ----
    ctx.allow_overlap(
        "body", "screen_slab",
        elem_a="pivot_post", elem_b="pivot_hub",
        reason="The pivot post passes through the hub bushing at the corner swivel.",
    )
    ctx.expect_within(
        body, screen_slab, axes="xy",
        inner_elem="pivot_post", outer_elem="pivot_hub",
        name="pivot post is inside hub bushing in XY",
    )
    ctx.expect_overlap(
        body, screen_slab, axes="z",
        elem_a="pivot_post", elem_b="pivot_hub",
        min_overlap=0.020,
        name="pivot post and hub share Z extent",
    )

    # ---- screen_glass proud of screen_frame (on screen_slab) ----
    screen_aabb = ctx.part_element_world_aabb(screen_slab, elem="screen_glass")
    frame_aabb = ctx.part_element_world_aabb(screen_slab, elem="screen_frame")
    ctx.check(
        "green LCD glass tops the dark bezel on screen_slab",
        screen_aabb[1][2] > frame_aabb[1][2] + 0.0002,
        details=f"glass top={screen_aabb[1][2]}, bezel top={frame_aabb[1][2]}",
    )

    # ---- earpiece on screen_slab, above screen ----
    ear_aabb = ctx.part_element_world_aabb(screen_slab, elem="earpiece_slot")
    screen_cy = (screen_aabb[0][1] + screen_aabb[1][1]) / 2.0
    ear_cy = (ear_aabb[0][1] + ear_aabb[1][1]) / 2.0
    ctx.check(
        "earpiece slot is above the screen on screen_slab",
        ear_cy > screen_cy,
        details=f"earpiece y={ear_cy}, screen y={screen_cy}",
    )

    # ---- upper_logo_plaque on screen_slab ----
    logo_aabb = ctx.part_element_world_aabb(screen_slab, elem="upper_logo_plaque")
    logo_cy = (logo_aabb[0][1] + logo_aabb[1][1]) / 2.0
    ctx.check(
        "upper logo plaque sits between earpiece and screen",
        ear_cy > logo_cy > screen_cy,
        details=f"ear={ear_cy}, logo={logo_cy}, screen={screen_cy}",
    )

    # ---- swivel proof: rotating screen_slab swings it away ----
    rest_slab_cx = (slab_aabb[0][0] + slab_aabb[1][0]) / 2.0
    rest_slab_cy = (slab_aabb[0][1] + slab_aabb[1][1]) / 2.0
    with ctx.pose({swivel: math.pi}):
        rotated_aabb = ctx.part_world_aabb(screen_slab)
        rot_cx = (rotated_aabb[0][0] + rotated_aabb[1][0]) / 2.0
        rot_cy = (rotated_aabb[0][1] + rotated_aabb[1][1]) / 2.0
    ctx.check(
        "swivel at pi rotates screen slab away from rest position",
        abs(rot_cx - rest_slab_cx) > 0.020 or abs(rot_cy - rest_slab_cy) > 0.020,
        details=f"rest=({rest_slab_cx:.4f},{rest_slab_cy:.4f}), "
                f"rotated=({rot_cx:.4f},{rot_cy:.4f})",
    )

    # ---- all 15 keys press straight down on the body ----
    all_key_names = [
        "key_nav", "key_c", "key_arrow",
        "key_1", "key_2", "key_3", "key_4", "key_5", "key_6",
        "key_7", "key_8", "key_9", "key_star", "key_0", "key_hash",
    ]
    for kname in all_key_names:
        kpart = object_model.get_part(kname)
        joint = object_model.get_articulation(f"body_to_{kname}")
        ctx.check(
            f"{kname} is a straight -Z prismatic key on body",
            joint.articulation_type == ArticulationType.PRISMATIC
            and tuple(round(v, 6) for v in joint.axis) == (0.0, 0.0, -1.0)
            and joint.motion_limits is not None
            and abs(joint.motion_limits.lower - 0.0) < 1e-9
            and abs(joint.motion_limits.upper - PRESS_TRAVEL) < 1e-9,
            details=f"type={joint.articulation_type}, axis={joint.axis}, "
                    f"limits={joint.motion_limits}",
        )
        crown_top = ctx.part_world_aabb(kpart)[1][2]
        ctx.check(
            f"{kname} crown is recessed below the face rim",
            crown_top <= RIM_Z + 0.0002,
            details=f"{kname} crown_top={crown_top}, rim={RIM_Z}",
        )

    # ---- key press proof on a sample ----
    for kname in ["key_5", "key_nav", "key_star"]:
        kpart = object_model.get_part(kname)
        joint = object_model.get_articulation(f"body_to_{kname}")
        rest_z = ctx.part_world_position(kpart)[2]
        with ctx.pose({joint: PRESS_TRAVEL}):
            pressed_z = ctx.part_world_position(kpart)[2]
        ctx.check(
            f"{kname} presses down into the face",
            pressed_z < rest_z - 0.0005,
            details=f"rest_z={rest_z}, pressed_z={pressed_z}",
        )

    # ---- number row ordering ----
    y1 = (ctx.part_world_aabb(object_model.get_part("key_1"))[0][1]
          + ctx.part_world_aabb(object_model.get_part("key_1"))[1][1]) / 2.0
    y7 = (ctx.part_world_aabb(object_model.get_part("key_7"))[0][1]
          + ctx.part_world_aabb(object_model.get_part("key_7"))[1][1]) / 2.0
    ystar = (ctx.part_world_aabb(object_model.get_part("key_star"))[0][1]
             + ctx.part_world_aabb(object_model.get_part("key_star"))[1][1]) / 2.0
    ctx.check(
        "number rows descend 1 -> 7 -> *",
        y1 > y7 > ystar,
        details=f"y1={y1}, y7={y7}, ystar={ystar}",
    )

    # ---- full keypad count ----
    press_joints = [
        a for a in object_model.articulations
        if a.articulation_type == ArticulationType.PRISMATIC
    ]
    ctx.check(
        "keypad has 15 pressable keys",
        len(press_joints) == 15,
        details=f"prismatic key joints={len(press_joints)}",
    )

    # ---- two-tone: body and screen_slab use different materials ----
    body_shell_mat = None
    slab_shell_mat = None
    for v in body.visuals:
        if v.name == "body_shell":
            body_shell_mat = v.material
    for v in screen_slab.visuals:
        if v.name == "screen_slab_shell":
            slab_shell_mat = v.material
    ctx.check(
        "two-tone colorway: body and screen_slab shells differ",
        body_shell_mat is not None and slab_shell_mat is not None
        and body_shell_mat.name != slab_shell_mat.name,
        details=f"body_mat={body_shell_mat.name if body_shell_mat else None}, "
                f"slab_mat={slab_shell_mat.name if slab_shell_mat else None}",
    )

    return ctx.report()


object_model = build_object_model()
