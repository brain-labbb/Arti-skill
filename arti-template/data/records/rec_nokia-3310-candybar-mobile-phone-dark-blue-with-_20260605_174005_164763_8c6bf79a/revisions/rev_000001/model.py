from __future__ import annotations

# Nokia 3310 candybar mobile phone, dark blue.
# Frame: face points up (+Z). Phone is a tall candybar:
#   - Y = long axis (tall), top at +Y near the earpiece/screen, keypad toward -Y
#   - X = width (narrow)
#   - Z = thickness; the front face is the +Z surface, keys press DOWN into it (-Z)
# Static: dark-navy rounded monoblock shell, slate-grey hourglass fascia,
#         green LCD proud of a thin dark bezel, earpiece slit column,
#         NOKIA plaque, embedded side button, gold bottom contact.
# Articulations (15): every key is a PRISMATIC press straight down (-Z, ~1mm):
#   - the wide curved center menu/navi bar (cyan smile arc),
#   - 2 kidney soft keys flanking it (C left, arrow right),
#   - the 12 number keys 1-9, *, 0, # (tilted oval caps in a 4x3 grid).

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
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
FASCIA_TOP_Z = RIM_Z + 0.0003  # slate fascia inlay sits 0.3mm proud of the rim

# Keypad is set INTO the front face: a real recessed pocket is carved into the
# body shell, and the keycaps sit in that pocket with their crowns just BELOW
# the surrounding rim (so the keys read as inside the phone, not floating on top).
POCKET_DEPTH = 0.0030                    # how deep the keypad well is cut into the face
POCKET_FLOOR_Z = RIM_Z - POCKET_DEPTH    # well floor z (0.0075)
# The cavity is an hourglass union of two rounded pockets: a wider upper well
# for the function cluster and a slimmer lower well for the number grid.
FUNC_POCKET_W = 0.0420
FUNC_POCKET_H = 0.0128
FUNC_POCKET_CY = 0.0062
NUM_POCKET_W = 0.0395
NUM_POCKET_H = 0.0500
NUM_POCKET_CY = -0.0246

KEY_SEAT_Z = POCKET_FLOOR_Z       # caps seat on the well floor / joint plane
KEY_RISE = 0.0025                 # crown reaches ~0.0100, i.e. ~0.5mm below the rim
PRESS_TRAVEL = 0.0010             # 1 mm prismatic press into the face


def _rot_pts(pts: list[tuple[float, float]], ang: float) -> list[tuple[float, float]]:
    c, s = math.cos(ang), math.sin(ang)
    return [(x * c - y * s, x * s + y * c) for x, y in pts]


def _translated_rounded_slab(
    w: float,
    h: float,
    z0: float,
    t: float,
    r: float,
    name: str,
    *,
    x: float = 0.0,
    y: float = 0.0,
):
    """Rounded rectangle slab with its XY profile already in body coordinates."""
    geom = ExtrudeGeometry.from_z0(rounded_rect_profile(w, h, r, corner_segments=10), t, cap=True)
    geom.translate(x, y, z0)
    return mesh_from_geometry(geom, name)


def _front_fascia_mesh():
    """3310 front insert: one clean slate-grey hourglass fascia. Wide around the
    screen and the function cluster, pinching to a V-tail above the number
    grid. Three openings are cut for the pressable function keys so they sit
    recessed in their own wells."""
    y_halfwidth = [
        (0.0532, 0.0140),
        (0.0505, 0.0172),
        (0.0450, 0.0192),
        (0.0330, 0.0206),
        (0.0180, 0.0212),
        (0.0060, 0.0216),
        (-0.0030, 0.0198),
        (-0.0058, 0.0152),
        (-0.0080, 0.0085),
    ]
    right = [(hw, y) for y, hw in y_halfwidth]
    left = [(-hw, y) for y, hw in reversed(y_halfwidth)]
    geom = ExtrudeGeometry.from_z0(right + left, 0.0009, cap=True)
    geom.translate(0.0, 0.0, RIM_Z - 0.0006)
    # Openings over the three function keys (nav bar + two tilted kidneys).
    holes = [
        (0.0, 0.0040, 0.0205, 0.0105, 0.0040, 0.0),
        (-0.0150, 0.0058, 0.0095, 0.0120, 0.0038, -0.18),
        (0.0150, 0.0058, 0.0095, 0.0120, 0.0038, 0.18),
    ]
    for hx, hy, hw, hh, hr, yaw in holes:
        pts = _rot_pts(rounded_rect_profile(hw, hh, hr, corner_segments=8), yaw)
        cut = ExtrudeGeometry.from_z0(pts, 0.004, cap=True)
        cut.translate(hx, hy, RIM_Z - 0.0016)
        geom = boolean_difference(geom, cut)
    return mesh_from_geometry(geom, "front_fascia")


def _arc_bar_mesh(name: str, w: float, h: float, curve: float, thickness: float = 0.00018):
    """A very thin smiling arc/chevron mark used on the menu key and screen art."""
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
    """Small separated grille slits joined by an invisible shallow spine so the
    earpiece reads as individual slots without creating floating mesh islands."""
    spine = ExtrudeGeometry.from_z0(
        rounded_rect_profile(slot_w * 0.40, pitch * (count - 1) + slot_h, slot_w * 0.18, corner_segments=4),
        0.00035,
        cap=True,
    )
    for i in range(count):
        y = (i - (count - 1) / 2.0) * pitch
        slit = ExtrudeGeometry.from_z0(
            rounded_rect_profile(slot_w, slot_h, min(slot_w, slot_h) * 0.45, corner_segments=5),
            0.00055,
            cap=True,
        )
        # Slight banana curve as in the reference top grille.
        x = 0.0004 * math.sin((i - (count - 1) / 2.0) * 0.7)
        slit.translate(x, y, 0.0)
        spine.merge(slit)
    return mesh_from_geometry(spine, name)


def _stroke_box(part, name: str, x: float, y: float, z: float, sx: float, sy: float, sz: float, material, yaw: float = 0.0):
    part.visual(
        Box((sx, sy, sz)),
        origin=Origin(xyz=(x, y, z), rpy=(0.0, 0.0, yaw)),
        material=material,
        name=name,
    )


def _add_nokia_wordmark(part, material, *, prefix: str, cx: float, cy: float, z: float, scale: float, thickness: float = 0.00028):
    """Block-stroke NOKIA wordmark.  It is intentionally geometric so it remains
    mesh-light while still reading as the logo on both body and display."""
    # Letter cell: width 4*scale, height 6*scale, stroke scale.
    s = scale
    w = 4.0 * s
    h = 6.0 * s
    gap = 1.6 * s
    total = 5 * w + 4 * gap
    x0 = cx - total / 2.0

    def seg(letter_idx: int, seg_name: str, ox: float, oy: float, sx: float, sy: float, yaw: float = 0.0):
        lx = x0 + letter_idx * (w + gap) + ox
        _stroke_box(part, f"{prefix}_{letter_idx}_{seg_name}", lx, cy + oy, z, sx, sy, thickness, material, yaw)

    # N
    seg(0, "left", 0.0, 0.0, s, h)
    seg(0, "right", w, 0.0, s, h)
    seg(0, "diag", w / 2.0, 0.0, s * 0.9, h * 1.15, math.radians(28))
    # O
    seg(1, "left", 0.0, 0.0, s, h)
    seg(1, "right", w, 0.0, s, h)
    seg(1, "top", w / 2.0, h / 2.0, w, s)
    seg(1, "bottom", w / 2.0, -h / 2.0, w, s)
    # K
    seg(2, "left", 0.0, 0.0, s, h)
    seg(2, "updiag", w * 0.55, h * 0.18, s * 0.9, h * 0.65, math.radians(-35))
    seg(2, "downdiag", w * 0.55, -h * 0.18, s * 0.9, h * 0.65, math.radians(35))
    # I
    seg(3, "stem", w / 2.0, 0.0, s, h)
    seg(3, "top", w / 2.0, h / 2.0, w * 0.85, s)
    seg(3, "bottom", w / 2.0, -h / 2.0, w * 0.85, s)
    # A
    seg(4, "left", w * 0.15, 0.0, s, h, math.radians(-8))
    seg(4, "right", w * 0.85, 0.0, s, h, math.radians(8))
    seg(4, "top", w / 2.0, h / 2.0, w * 0.75, s)
    seg(4, "mid", w / 2.0, 0.0, w * 0.65, s)


SEGMENTS = {
    "0": "abcfed",
    "1": "bc",
    "2": "abged",
    "3": "abgcd",
    "4": "fgbc",
    "5": "afgcd",
    "6": "afgecd",
    "7": "abc",
    "8": "abcdefg",
    "9": "abfgcd",
}


def _add_seven_segment_digit(part, material, digit: str, *, prefix: str, z: float, scale: float = 1.0, x: float = -0.0020, y: float = 0.00015):
    """Small black seven-segment digit printed on a keycap."""
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
    # Slightly embedded ink prevents label strokes from reading as floating
    # islands while leaving them visible on the key crown.
    z = KEY_RISE - 0.00002
    if label.isdigit():
        _add_seven_segment_digit(part, material, label, prefix=prefix, z=z, scale=0.82)
        # tiny T9 letters as dark micro bars to the right of the digit
        _stroke_box(part, f"{prefix}_letters", 0.0027, -0.0010, z, 0.0023, 0.00024, 0.00013, material)
    elif label == "*":
        for i, yaw in enumerate((0.0, math.radians(60), math.radians(-60))):
            _stroke_box(part, f"{prefix}_star_{i}", -0.0014, 0.0, z, 0.0034, 0.00034, 0.00013, material, yaw)
        _stroke_box(part, f"{prefix}_plus", 0.0026, -0.0010, z, 0.0011, 0.00025, 0.00013, material)
        _stroke_box(part, f"{prefix}_plus_v", 0.0026, -0.0010, z, 0.00025, 0.0011, 0.00013, material)
    elif label == "#":
        for i, xx in enumerate((-0.0008, 0.0008)):
            _stroke_box(part, f"{prefix}_hash_v{i}", xx, 0.0, z, 0.00028, 0.0030, 0.00013, material)
        for i, yy in enumerate((-0.0007, 0.0007)):
            _stroke_box(part, f"{prefix}_hash_h{i}", 0.0, yy, z, 0.0030, 0.00028, 0.00013, material)
    elif kind == "nav":
        part.visual(_arc_bar_mesh(f"{prefix}_cyan_smile", 0.0080, 0.0013, -0.0018), origin=Origin(xyz=(0.0, -0.0006, z)), material=material, name="menu_smile")
    elif label == "C":
        _stroke_box(part, f"{prefix}_c_top", 0.0, 0.0011, z, 0.0025, 0.00038, 0.00013, material)
        _stroke_box(part, f"{prefix}_c_bottom", 0.0, -0.0011, z, 0.0025, 0.00038, 0.00013, material)
        _stroke_box(part, f"{prefix}_c_side", -0.00125, 0.0, z, 0.00038, 0.0022, 0.00013, material)
    else:
        # Soft/scroll arrow-like marks.
        _stroke_box(part, f"{prefix}_mark_a", -0.0004, 0.0, z, 0.0028, 0.00035, 0.00013, material, math.radians(35))
        _stroke_box(part, f"{prefix}_mark_b", 0.0004, 0.0, z, 0.0028, 0.00035, 0.00013, material, math.radians(-35))


def _body_shell_mesh():
    """Dark-navy candybar monoblock: seven stacked rounded slabs whose insets
    follow a smooth edge-rounding curve (small steps -> reads as one smoothly
    curved plastic shell instead of stacked pancakes)."""
    geom = BoxGeometry((0.0, 0.0, 0.0))  # empty seed
    sections = [
        # (z0, thickness, inset) - small per-step insets so the edge rounding
        # reads smooth instead of stacked pancakes
        (-0.0110, 0.0016, 0.0030),   # back face, most rounded-in
        (-0.0099, 0.0017, 0.0018),
        (-0.0088, 0.0019, 0.0010),
        (-0.0075, 0.0021, 0.0005),
        (-0.0060, 0.0021, 0.0002),
        (-0.0045, 0.0084, 0.0000),   # main barrel (widest)
        (0.0034, 0.0022, 0.0002),
        (0.0051, 0.0020, 0.0006),
        (0.0066, 0.0018, 0.0012),
        (0.0079, 0.0017, 0.0018),
        (0.0091, 0.0014, 0.0026),    # front rim plane (top = RIM_Z)
    ]
    for i, (z0, t, inset) in enumerate(sections):
        prof = rounded_rect_profile(
            BODY_W - 2.0 * inset,
            BODY_H - 2.0 * inset,
            0.016 - inset * 0.5,
            corner_segments=10,
        )
        # true boolean union (not merge) so the shell exports as ONE watertight
        # solid -> no tolerance-thin seams between stacked slabs
        slab = ExtrudeGeometry.from_z0(prof, t, cap=True)
        slab.translate(0.0, 0.0, z0)
        if i == 0:
            geom = slab
        else:
            geom = boolean_union(geom, slab)

    # Carve the recessed keypad cavity (hourglass union of two rounded wells)
    # into the front face so the keys sit INSIDE the phone.
    for w, h, cy, r in (
        (FUNC_POCKET_W, FUNC_POCKET_H, FUNC_POCKET_CY, 0.0045),
        (NUM_POCKET_W, NUM_POCKET_H, NUM_POCKET_CY, 0.0055),
    ):
        pocket = ExtrudeGeometry.from_z0(
            rounded_rect_profile(w, h, r, corner_segments=8),
            POCKET_DEPTH + 0.003,
            cap=True,
        )
        pocket.translate(0.0, cy, POCKET_FLOOR_Z)
        geom = boolean_difference(geom, pocket)
    return mesh_from_geometry(geom, "body_shell")


def _keycap_mesh(name: str, w: float, h: float, r: float):
    """A curved, slightly domed keycap: a low rounded-rect slab whose top is a
    smaller slab (gives a beveled/curved crown like the 3310 keys)."""
    base = ExtrudeGeometry.from_z0(rounded_rect_profile(w, h, r, corner_segments=6), KEY_RISE * 0.62, cap=True)
    crown = ExtrudeGeometry.from_z0(
        rounded_rect_profile(w - 0.0016, h - 0.0012, max(r - 0.0006, 0.0006), corner_segments=6),
        KEY_RISE,
        cap=True,
    )
    base.merge(crown)
    return mesh_from_geometry(base, name)


def _smile_keycap_mesh(name: str, w: float, h: float, curve: float):
    """The wide 3310 menu/navi bar: a rounded bar curved into a shallow arc
    across X, drooping toward -Y at its center (the classic 'smile')."""
    half_w = w / 2.0
    half_h = h / 2.0
    n = 9
    top: list[tuple[float, float]] = []
    bot: list[tuple[float, float]] = []
    for i in range(n + 1):
        t = i / n
        x = -half_w + w * t
        bow = curve * (1.0 - (2.0 * t - 1.0) ** 2)  # 0 at ends, max in middle
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
    model = ArticulatedObject(name="nokia_3310")

    body_blue = model.material("glossy_dark_navy", rgba=(0.050, 0.070, 0.155, 1.0))
    slate = model.material("slate_grey_fascia", rgba=(0.355, 0.370, 0.400, 1.0))
    silver = model.material("warm_silver_grey", rgba=(0.62, 0.64, 0.63, 1.0))
    well_dark = model.material("navy_key_well", rgba=(0.032, 0.045, 0.105, 1.0))
    key_light = model.material("pearl_silver_keys", rgba=(0.705, 0.715, 0.700, 1.0))
    screen_green = model.material("green_lcd", rgba=(0.585, 0.705, 0.485, 1.0))
    screen_frame = model.material("black_display_bezel", rgba=(0.020, 0.022, 0.028, 1.0))
    print_dark = model.material("printed_black", rgba=(0.015, 0.018, 0.015, 1.0))
    plaque_light = model.material("logo_plaque_silver", rgba=(0.87, 0.89, 0.86, 1.0))
    cyan = model.material("navi_cyan", rgba=(0.00, 0.62, 0.76, 1.0))
    earpiece = model.material("earpiece_black", rgba=(0.006, 0.006, 0.010, 1.0))
    gold = model.material("charge_contact_gold", rgba=(0.78, 0.64, 0.28, 1.0))

    # ================= BODY (root) =================
    body = model.part("body")
    body.visual(_body_shell_mesh(), material=body_blue, name="body_shell")

    # Embedded side button on the left edge (protrudes ~0.7mm from the wall).
    body.visual(
        Box((0.0024, 0.0200, 0.0052)),
        origin=Origin(xyz=(-0.0235, -0.0170, 0.0012)),
        material=silver,
        name="side_button",
    )
    # Gold charging contact peeking from the bottom edge (as in the photo).
    body.visual(
        Box((0.0100, 0.0030, 0.0035)),
        origin=Origin(xyz=(0.0, -0.0538, 0.0010)),
        material=gold,
        name="bottom_contact",
    )

    # ---- slate-grey hourglass fascia inlay with function-key openings ----
    body.visual(_front_fascia_mesh(), material=slate, name="front_fascia")

    # ---- earpiece: curved column of grille slits at the very top ----
    body.visual(
        _slot_stack_mesh("earpiece_slot_mesh", 4, 0.0030, 0.0009, 0.00135),
        origin=Origin(xyz=(0.0, 0.047475, RIM_Z + 0.00025)),
        material=earpiece,
        name="earpiece_slot",
    )

    display_shift_y = 0.0045
    upper_logo_cy = 0.0372 + display_shift_y

    # Silver NOKIA plaque with dark letters, between grille and screen.
    body.visual(
        _translated_rounded_slab(0.0150, 0.0048, RIM_Z + 0.00022, 0.00042, 0.0010, "upper_logo_plaque", y=upper_logo_cy),
        material=plaque_light,
        name="upper_logo_plaque",
    )
    _add_nokia_wordmark(body, print_dark, prefix="upper_nokia", cx=0.0, cy=upper_logo_cy, z=RIM_Z + 0.00069, scale=0.00042, thickness=0.00012)

    # ---- screen: thin dark bezel with the green LCD proud of it ----
    screen_cx, screen_cy = 0.0, 0.0215 + display_shift_y
    sw, sh = 0.0270, 0.0210
    body.visual(
        _translated_rounded_slab(sw + 0.0050, sh + 0.0050, RIM_Z - 0.0004, 0.0008, 0.0030, "screen_frame_mesh", x=screen_cx, y=screen_cy),
        material=screen_frame,
        name="screen_frame",
    )
    body.visual(
        Box((sw, sh, 0.0006)),
        origin=Origin(xyz=(screen_cx, screen_cy, RIM_Z + 0.0006)),
        material=screen_green,
        name="screen_glass",
    )
    # LCD icon row, pixel art silhouette, and NOKIA wordmark on the green display.
    lcd_z = RIM_Z + 0.00092
    for i in range(4):
        _stroke_box(body, f"screen_signal_{i}", -0.0115 + i * 0.0012, screen_cy + 0.0082 + i * 0.00020, lcd_z, 0.0007, 0.00045 + i * 0.00035, 0.00008, print_dark)
    _stroke_box(body, "screen_battery", 0.0105, screen_cy + 0.0086, lcd_z, 0.0035, 0.0010, 0.00008, print_dark)
    _stroke_box(body, "screen_battery_nub", 0.0126, screen_cy + 0.0086, lcd_z, 0.0005, 0.00055, 0.00008, print_dark)
    for i, (px, py, sx, sy) in enumerate([
        (-0.0060, 0.0036, 0.0016, 0.0048),
        (-0.0039, 0.0016, 0.0017, 0.0043),
        (-0.0018, 0.0001, 0.0015, 0.0034),
        (0.0042, 0.0035, 0.0012, 0.0036),
        (0.0060, 0.0018, 0.0014, 0.0031),
    ]):
        _stroke_box(body, f"screen_pixel_art_{i}", px, screen_cy + py, lcd_z, sx, sy, 0.00008, print_dark, math.radians(-24 if i < 3 else 18))
    _add_nokia_wordmark(body, print_dark, prefix="screen_nokia", cx=0.0010, cy=screen_cy - 0.0062, z=lcd_z, scale=0.00060, thickness=0.00008)

    # ---- well floors: slate under the function cluster, navy under numbers ----
    body.visual(
        _translated_rounded_slab(0.0408, 0.0120, POCKET_FLOOR_Z - 0.0008, 0.0009, 0.0040, "function_floor_mesh", y=FUNC_POCKET_CY),
        material=slate,
        name="function_floor",
    )
    body.visual(
        _translated_rounded_slab(0.0383, 0.0488, POCKET_FLOOR_Z - 0.0008, 0.0009, 0.0050, "keypad_fascia_panel_mesh", y=NUM_POCKET_CY),
        material=well_dark,
        name="keypad_fascia_panel",
    )

    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_H, BODY_T)), mass=0.130, origin=Origin(xyz=(0.0, 0.0, 0.0))
    )

    # ================= KEYS =================
    # Each key: a part with a keycap visual, mounted by a PRISMATIC joint whose
    # origin is at the well floor (KEY_SEAT_Z); axis -Z so positive q presses in.
    # (name, kind, x, y, w, h, yaw)  kind in {"nav","soft","num"}
    key_specs: list[tuple[str, str, float, float, float, float, float]] = []

    # --- function cluster directly below the screen ---
    # Wide curved center menu/navi bar with the cyan smile arc.
    key_specs.append(("key_nav", "nav", 0.0, 0.0044, 0.0190, 0.0070, 0.0))
    # Two kidney soft keys hugging the nav bar (C left, arrow right).
    key_specs.append(("key_c", "soft", -0.0150, 0.0058, 0.0075, 0.0100, -0.18))
    key_specs.append(("key_arrow", "soft", 0.0150, 0.0058, 0.0075, 0.0100, 0.18))

    # --- numeric keypad: 4 rows x 3 cols of tilted oval caps ---
    num_labels = [
        ["key_1", "key_2", "key_3"],
        ["key_4", "key_5", "key_6"],
        ["key_7", "key_8", "key_9"],
        ["key_star", "key_0", "key_hash"],
    ]
    col_x = (-0.0131, 0.0, 0.0131)
    col_yaw = (-0.10, 0.0, 0.10)   # outer ends tip up -> gently arced rows
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
        else:  # num
            cap = _keycap_mesh(f"{name}_cap", kw, kh, 0.0030)
            inertia_box = Box((kw, kh, KEY_RISE))

        # keycap mesh is built from z=0 upward; seat it so its base sits on the
        # well floor (KEY_SEAT_Z) when child frame is at the joint origin.
        part.visual(cap, origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, yaw)), material=key_light, name="keycap")
        if kind == "num":
            if name == "key_star":
                label = "*"
            elif name == "key_hash":
                label = "#"
            else:
                label = name.split("_")[-1]
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
                effort=3.0, velocity=0.05, lower=0.0, upper=PRESS_TRAVEL
            ),
        )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")

    # ---- body is a tall candybar (taller than wide and thicker check) ----
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "body is a tall candybar (Y much greater than X)",
        bext[1] > bext[0] + 0.04,
        details=f"body extents={bext}",
    )
    ctx.check(
        "body is narrow in width vs height",
        bext[1] > 2.0 * bext[0],
        details=f"body extents={bext}",
    )

    # ---- screen is near the top, keypad is below ----
    screen_aabb = ctx.part_element_world_aabb(body, elem="screen_glass")
    screen_cy = (screen_aabb[0][1] + screen_aabb[1][1]) / 2.0
    ctx.check(
        "screen sits in the upper half of the body",
        screen_cy > 0.012,
        details=f"screen center y={screen_cy}",
    )
    # regression: the green LCD must sit PROUD of the dark bezel, never inside
    # it (a buried glass renders as an all-black screen).
    frame_aabb = ctx.part_element_world_aabb(body, elem="screen_frame")
    ctx.check(
        "green LCD glass tops the dark bezel",
        screen_aabb[1][2] > frame_aabb[1][2] + 0.0002,
        details=f"glass top={screen_aabb[1][2]}, bezel top={frame_aabb[1][2]}",
    )
    ear_aabb = ctx.part_element_world_aabb(body, elem="earpiece_slot")
    ear_cy = (ear_aabb[0][1] + ear_aabb[1][1]) / 2.0
    ctx.check(
        "earpiece slot is above the screen",
        ear_cy > screen_cy,
        details=f"earpiece y={ear_cy}, screen y={screen_cy}",
    )
    fascia_aabb = ctx.part_element_world_aabb(body, elem="front_fascia")
    keypad_panel_aabb = ctx.part_element_world_aabb(body, elem="keypad_fascia_panel")
    fascia_cy = (fascia_aabb[0][1] + fascia_aabb[1][1]) / 2.0
    keypad_panel_cy = (keypad_panel_aabb[0][1] + keypad_panel_aabb[1][1]) / 2.0
    ctx.check(
        "slate fascia spans the screen and function-key region",
        fascia_aabb[1][1] > screen_aabb[1][1] and fascia_aabb[0][1] < 0.000,
        details=f"fascia={fascia_aabb}, screen={screen_aabb}, fascia_cy={fascia_cy}",
    )
    ctx.check(
        "number keypad panel is below the screen",
        keypad_panel_cy < screen_cy - 0.020,
        details=f"keypad_panel_y={keypad_panel_cy}, screen_y={screen_cy}",
    )

    # ---- all keys press straight down (-Z) and stay seated ----
    all_key_names = [
        "key_nav", "key_c", "key_arrow",
        "key_1", "key_2", "key_3", "key_4", "key_5", "key_6",
        "key_7", "key_8", "key_9", "key_star", "key_0", "key_hash",
    ]
    for kname in all_key_names:
        part = object_model.get_part(kname)
        joint = object_model.get_articulation(f"body_to_{kname}")
        ctx.check(
            f"{kname} is a straight -Z prismatic key",
            joint.articulation_type == ArticulationType.PRISMATIC
            and tuple(round(v, 6) for v in joint.axis) == (0.0, 0.0, -1.0)
            and joint.motion_limits is not None
            and abs(joint.motion_limits.lower - 0.0) < 1e-9
            and abs(joint.motion_limits.upper - PRESS_TRAVEL) < 1e-9,
            details=f"type={joint.articulation_type}, axis={joint.axis}, limits={joint.motion_limits}",
        )
        crown_top = ctx.part_world_aabb(part)[1][2]
        ctx.check(
            f"{kname} crown is below the raised body/fascia rim",
            crown_top <= RIM_Z + 0.0002,
            details=f"{kname} crown_top={crown_top}, rim={RIM_Z}",
        )

    sample_keys = ["key_5", "key_star", "key_hash", "key_nav", "key_c", "key_0"]
    for kname in sample_keys:
        part = object_model.get_part(kname)
        joint = object_model.get_articulation(f"body_to_{kname}")

        # key sits below the screen
        k_aabb = ctx.part_world_aabb(part)
        k_cy = (k_aabb[0][1] + k_aabb[1][1]) / 2.0
        ctx.check(
            f"{kname} sits below the screen",
            k_cy < screen_cy,
            details=f"{kname} y={k_cy}, screen y={screen_cy}",
        )

        # key sits seated in the recessed keypad well, connected to the body
        rest_z = ctx.part_world_position(part)[2]
        ctx.expect_contact(part, body, name=f"{kname} seated in its well")

        # key is RECESSED: its crown sits at/below the surrounding face rim
        # (the keys are inside the phone, not floating proud of the body).
        crown_top = ctx.part_world_aabb(part)[1][2]
        ctx.check(
            f"{kname} crown is recessed below the face rim",
            crown_top <= RIM_Z + 0.0002,
            details=f"{kname} crown_top={crown_top}, rim={RIM_Z}",
        )

        # pressing the key moves it straight down into the face (-Z)
        with ctx.pose({joint: PRESS_TRAVEL}):
            pressed_z = ctx.part_world_position(part)[2]
            pressed_pos = ctx.part_world_position(part)
        ctx.check(
            f"{kname} presses straight down into the face",
            pressed_z < rest_z - 0.0005,
            details=f"{kname} rest_z={rest_z}, pressed_z={pressed_z}",
        )
        # pressing only moves Z, not X/Y (straight-down travel)
        rest_pos = ctx.part_world_position(part)
        ctx.check(
            f"{kname} travel is vertical only",
            abs(pressed_pos[0] - rest_pos[0]) < 1e-5
            and abs(pressed_pos[1] - rest_pos[1]) < 1e-5,
            details=f"{kname} rest={rest_pos}, pressed={pressed_pos}",
        )

    # ---- number keypad ordering: 1-row above 7-row; * row at the bottom ----
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
    # the bottom key row must stay INSIDE the body outline (regression for the
    # keys that used to hang off the bottom edge of the shell).
    body_aabb = ctx.part_world_aabb(body)
    star_aabb = ctx.part_world_aabb(object_model.get_part("key_star"))
    ctx.check(
        "bottom key row stays inside the body outline",
        star_aabb[0][1] > body_aabb[0][1] + 0.004,
        details=f"star bottom={star_aabb[0][1]}, body bottom={body_aabb[0][1]}",
    )
    key5_ext = _ext(ctx.part_world_aabb(object_model.get_part("key_5")))
    ctx.check(
        "number keys are wide oval caps",
        key5_ext[0] > key5_ext[1] * 1.15,
        details=f"key_5 extents={key5_ext}",
    )
    nav_ext = _ext(ctx.part_world_aabb(object_model.get_part("key_nav")))
    ctx.check(
        "menu bar is a wide curved key",
        nav_ext[0] > nav_ext[1] * 1.8,
        details=f"key_nav extents={nav_ext}",
    )

    # ---- full keypad count present (15 pressable keys) ----
    press_joints = [
        a for a in object_model.articulations
        if a.articulation_type == ArticulationType.PRISMATIC
    ]
    ctx.check(
        "keypad has 15 pressable keys",
        len(press_joints) == 15,
        details=f"prismatic key joints={len(press_joints)}",
    )

    return ctx.report()


object_model = build_object_model()
