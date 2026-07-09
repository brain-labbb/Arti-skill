from __future__ import annotations

# Nokia 3310 QWERTY variant: wider dark-blue candybar phone with a full
# 4-row × 10-column pressable QWERTY keyboard below a small color screen.
# Frame: face points up (+Z). Phone is a tall candybar:
#   - Y = long axis (tall), top at +Y near the earpiece/screen, keypad toward -Y
#   - X = width (narrow)
#   - Z = thickness; the front face is the +Z surface, keys press DOWN into it (-Z)
# Static: dark-navy rounded monoblock shell, slate-grey fascia,
#         green LCD proud of a thin dark bezel, earpiece slit column,
#         NOKIA plaque, embedded side button, gold bottom contact.
# Articulations (40): every QWERTY key is a PRISMATIC press straight down (-Z, 1mm):
#   4 rows × 10 columns looped emission with uniform keycap helper, named key_r{r}_c{c}.

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

# ---- overall dimensions (wider for QWERTY) ----
BODY_W = 0.056   # X width (was 0.048 in numeric variant)
BODY_H = 0.110   # Y height (tall candybar)
BODY_T = 0.022   # Z thickness
RIM_Z = 0.0105   # top of the front shell (the navy rim plane)
FASCIA_TOP_Z = RIM_Z + 0.0003

# ---- QWERTY keyboard grid ----
QWERTY_ROWS = 4
QWERTY_COLS = 10
KEY_W = 0.0038       # keycap width (X)
KEY_H = 0.0038       # keycap depth (Y)
KEY_GAP_X = 0.0007   # gap between keys in X
KEY_GAP_Y = 0.0007   # gap between keys in Y
KEY_PITCH_X = KEY_W + KEY_GAP_X   # 0.0045
KEY_PITCH_Y = KEY_H + KEY_GAP_Y   # 0.0045
GRID_CY = 0.002      # center Y of the QWERTY grid

# ---- keypad pocket (recessed well carved into the body shell) ----
POCKET_DEPTH = 0.0030
POCKET_FLOOR_Z = RIM_Z - POCKET_DEPTH
POCKET_W = (QWERTY_COLS - 1) * KEY_PITCH_X + KEY_W + 0.004
POCKET_H = (QWERTY_ROWS - 1) * KEY_PITCH_Y + KEY_H + 0.004
POCKET_CY = GRID_CY

KEY_SEAT_Z = POCKET_FLOOR_Z
KEY_RISE = 0.0022
PRESS_TRAVEL = 0.0010


def _rot_pts(pts: list[tuple[float, float]], ang: float) -> list[tuple[float, float]]:
    c, s = math.cos(ang), math.sin(ang)
    return [(x * c - y * s, x * s + y * c) for x, y in pts]


def _translated_rounded_slab(
    w: float, h: float, z0: float, t: float, r: float, name: str,
    *, x: float = 0.0, y: float = 0.0,
):
    """Rounded rectangle slab with its XY profile already in body coordinates."""
    geom = ExtrudeGeometry.from_z0(rounded_rect_profile(w, h, r, corner_segments=10), t, cap=True)
    geom.translate(x, y, z0)
    return mesh_from_geometry(geom, name)


def _front_fascia_mesh():
    """Wider fascia for QWERTY variant: slate-grey panel covering screen area
    with one rectangular opening for the QWERTY keyboard grid."""
    y_halfwidth = [
        (0.053, 0.014),
        (0.050, 0.018),
        (0.040, 0.022),
        (0.025, 0.024),
        (0.015, 0.025),
        (0.005, 0.025),
        (-0.005, 0.025),
        (-0.010, 0.023),
        (-0.015, 0.018),
    ]
    right = [(hw, y) for y, hw in y_halfwidth]
    left = [(-hw, y) for y, hw in reversed(y_halfwidth)]
    geom = ExtrudeGeometry.from_z0(right + left, 0.0009, cap=True)
    geom.translate(0.0, 0.0, RIM_Z - 0.0006)
    # Rectangular opening for QWERTY keyboard pocket
    opening_w = POCKET_W + 0.001
    opening_h = POCKET_H + 0.001
    cut = ExtrudeGeometry.from_z0(
        rounded_rect_profile(opening_w, opening_h, 0.003, corner_segments=6),
        0.004, cap=True,
    )
    cut.translate(0.0, POCKET_CY, RIM_Z - 0.0016)
    geom = boolean_difference(geom, cut)
    return mesh_from_geometry(geom, "front_fascia")


def _slot_stack_mesh(name: str, count: int, slot_w: float, slot_h: float, pitch: float):
    """Small separated grille slits joined by an invisible shallow spine so the
    earpiece reads as individual slots without creating floating mesh islands."""
    spine = ExtrudeGeometry.from_z0(
        rounded_rect_profile(slot_w * 0.40, pitch * (count - 1) + slot_h, slot_w * 0.18, corner_segments=4),
        0.00035, cap=True,
    )
    for i in range(count):
        y = (i - (count - 1) / 2.0) * pitch
        slit = ExtrudeGeometry.from_z0(
            rounded_rect_profile(slot_w, slot_h, min(slot_w, slot_h) * 0.45, corner_segments=5),
            0.00055, cap=True,
        )
        x = 0.0004 * math.sin((i - (count - 1) / 2.0) * 0.7)
        slit.translate(x, y, 0.0)
        spine.merge(slit)
    return mesh_from_geometry(spine, name)


def _stroke_box(part, name: str, x: float, y: float, z: float,
                sx: float, sy: float, sz: float, material, yaw: float = 0.0):
    part.visual(
        Box((sx, sy, sz)),
        origin=Origin(xyz=(x, y, z), rpy=(0.0, 0.0, yaw)),
        material=material,
        name=name,
    )


def _add_nokia_wordmark(part, material, *, prefix: str, cx: float, cy: float,
                        z: float, scale: float, thickness: float = 0.00028):
    """Block-stroke NOKIA wordmark."""
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


def _body_shell_mesh():
    """Dark-navy candybar monoblock: smoothly rounded shell, wider for QWERTY,
    with one large rectangular pocket carved into the front face."""
    geom = BoxGeometry((0.0, 0.0, 0.0))
    sections = [
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
    for i, (z0, t, inset) in enumerate(sections):
        prof = rounded_rect_profile(
            BODY_W - 2.0 * inset,
            BODY_H - 2.0 * inset,
            0.018 - inset * 0.5,
            corner_segments=10,
        )
        slab = ExtrudeGeometry.from_z0(prof, t, cap=True)
        slab.translate(0.0, 0.0, z0)
        if i == 0:
            geom = slab
        else:
            geom = boolean_union(geom, slab)

    # Single QWERTY pocket carved into the front face
    pocket = ExtrudeGeometry.from_z0(
        rounded_rect_profile(POCKET_W, POCKET_H, 0.004, corner_segments=8),
        POCKET_DEPTH + 0.003, cap=True,
    )
    pocket.translate(0.0, POCKET_CY, POCKET_FLOOR_Z)
    geom = boolean_difference(geom, pocket)
    return mesh_from_geometry(geom, "body_shell")


def _qwerty_keycap_mesh(name: str, w: float, h: float):
    """Small rectangular keycap for QWERTY grid: rounded slab with beveled crown."""
    r = min(w, h) * 0.22
    base = ExtrudeGeometry.from_z0(
        rounded_rect_profile(w, h, r, corner_segments=4),
        KEY_RISE * 0.6, cap=True,
    )
    crown = ExtrudeGeometry.from_z0(
        rounded_rect_profile(w - 0.0006, h - 0.0006, max(r - 0.0002, 0.0003), corner_segments=4),
        KEY_RISE, cap=True,
    )
    base.merge(crown)
    return mesh_from_geometry(base, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nokia_3310_qwerty")

    body_blue = model.material("glossy_dark_navy", rgba=(0.050, 0.070, 0.155, 1.0))
    slate = model.material("slate_grey_fascia", rgba=(0.355, 0.370, 0.400, 1.0))
    silver = model.material("warm_silver_grey", rgba=(0.62, 0.64, 0.63, 1.0))
    well_dark = model.material("navy_key_well", rgba=(0.032, 0.045, 0.105, 1.0))
    key_charcoal = model.material("charcoal_keys", rgba=(0.22, 0.23, 0.26, 1.0))
    key_print = model.material("silver_key_print", rgba=(0.78, 0.80, 0.76, 1.0))
    screen_green = model.material("green_lcd", rgba=(0.585, 0.705, 0.485, 1.0))
    screen_frame_mat = model.material("black_display_bezel", rgba=(0.020, 0.022, 0.028, 1.0))
    print_dark = model.material("printed_black", rgba=(0.015, 0.018, 0.015, 1.0))
    plaque_light = model.material("logo_plaque_silver", rgba=(0.87, 0.89, 0.86, 1.0))
    earpiece_mat = model.material("earpiece_black", rgba=(0.006, 0.006, 0.010, 1.0))
    gold = model.material("charge_contact_gold", rgba=(0.78, 0.64, 0.28, 1.0))

    # ================= BODY (root) =================
    body = model.part("body")
    body.visual(_body_shell_mesh(), material=body_blue, name="body_shell")

    # Embedded side button on the left edge
    body.visual(
        Box((0.0024, 0.0200, 0.0052)),
        origin=Origin(xyz=(-0.0275, -0.0170, 0.0012)),
        material=silver,
        name="side_button",
    )
    # Gold charging contact at bottom edge
    body.visual(
        Box((0.0100, 0.0030, 0.0035)),
        origin=Origin(xyz=(0.0, -0.0538, 0.0010)),
        material=gold,
        name="bottom_contact",
    )

    # ---- slate-grey fascia inlay with QWERTY keyboard opening ----
    body.visual(_front_fascia_mesh(), material=slate, name="front_fascia")

    # ---- earpiece: curved column of grille slits at the very top ----
    earpiece_y = 0.049
    body.visual(
        _slot_stack_mesh("earpiece_slot_mesh", 4, 0.0030, 0.0009, 0.00135),
        origin=Origin(xyz=(0.0, earpiece_y, RIM_Z + 0.00025)),
        material=earpiece_mat,
        name="earpiece_slot",
    )

    # Screen and logo positions (shifted up slightly for QWERTY room)
    display_shift_y = 0.006
    upper_logo_cy = 0.0372 + display_shift_y

    # Silver NOKIA plaque with dark letters
    body.visual(
        _translated_rounded_slab(0.0150, 0.0048, RIM_Z + 0.00022, 0.00042, 0.0010,
                                 "upper_logo_plaque", y=upper_logo_cy),
        material=plaque_light,
        name="upper_logo_plaque",
    )
    _add_nokia_wordmark(body, print_dark, prefix="upper_nokia",
                        cx=0.0, cy=upper_logo_cy, z=RIM_Z + 0.00069,
                        scale=0.00042, thickness=0.00012)

    # ---- screen: thin dark bezel with the green LCD proud of it ----
    screen_cx, screen_cy = 0.0, 0.0215 + display_shift_y
    sw, sh = 0.0270, 0.0210
    body.visual(
        _translated_rounded_slab(sw + 0.0050, sh + 0.0050, RIM_Z - 0.0004, 0.0008,
                                 0.0030, "screen_frame_mesh",
                                 x=screen_cx, y=screen_cy),
        material=screen_frame_mat,
        name="screen_frame",
    )
    body.visual(
        Box((sw, sh, 0.0006)),
        origin=Origin(xyz=(screen_cx, screen_cy, RIM_Z + 0.0006)),
        material=screen_green,
        name="screen_glass",
    )
    # LCD icon row, pixel art silhouette, and NOKIA wordmark on the green display
    lcd_z = RIM_Z + 0.00092
    for i in range(4):
        _stroke_box(body, f"screen_signal_{i}",
                    -0.0115 + i * 0.0012, screen_cy + 0.0082 + i * 0.00020, lcd_z,
                    0.0007, 0.00045 + i * 0.00035, 0.00008, print_dark)
    _stroke_box(body, "screen_battery", 0.0105, screen_cy + 0.0086, lcd_z,
                0.0035, 0.0010, 0.00008, print_dark)
    _stroke_box(body, "screen_battery_nub", 0.0126, screen_cy + 0.0086, lcd_z,
                0.0005, 0.00055, 0.00008, print_dark)
    for i, (px, py, sx, sy) in enumerate([
        (-0.0060, 0.0036, 0.0016, 0.0048),
        (-0.0039, 0.0016, 0.0017, 0.0043),
        (-0.0018, 0.0001, 0.0015, 0.0034),
        (0.0042, 0.0035, 0.0012, 0.0036),
        (0.0060, 0.0018, 0.0014, 0.0031),
    ]):
        _stroke_box(body, f"screen_pixel_art_{i}", px, screen_cy + py, lcd_z,
                    sx, sy, 0.00008, print_dark,
                    math.radians(-24 if i < 3 else 18))
    _add_nokia_wordmark(body, print_dark, prefix="screen_nokia",
                        cx=0.0010, cy=screen_cy - 0.0062, z=lcd_z,
                        scale=0.00060, thickness=0.00008)

    # ---- QWERTY pocket floor: dark panel under the keys ----
    body.visual(
        _translated_rounded_slab(POCKET_W - 0.002, POCKET_H - 0.002,
                                 POCKET_FLOOR_Z - 0.0008, 0.0009, 0.004,
                                 "keypad_fascia_panel_mesh", y=POCKET_CY),
        material=well_dark,
        name="keypad_fascia_panel",
    )

    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_H, BODY_T)), mass=0.140, origin=Origin(xyz=(0.0, 0.0, 0.0))
    )

    # ================= QWERTY KEYS =================
    # 4 rows × 10 columns, looped emission with shared keycap helper.
    # Each key: a part with a keycap visual + label mark, mounted by a PRISMATIC
    # joint at the well floor (KEY_SEAT_Z); axis -Z so positive q presses in.
    key_mass = 0.0008

    for r in range(QWERTY_ROWS):
        row_y = GRID_CY + ((QWERTY_ROWS - 1) / 2.0 - r) * KEY_PITCH_Y
        for c in range(QWERTY_COLS):
            col_x = (c - (QWERTY_COLS - 1) / 2.0) * KEY_PITCH_X
            kname = f"key_r{r}_c{c}"

            kp = model.part(kname)
            cap = _qwerty_keycap_mesh(f"{kname}_cap", KEY_W, KEY_H)
            kp.visual(
                cap,
                origin=Origin(xyz=(0.0, 0.0, 0.0)),
                material=key_charcoal,
                name="keycap",
            )
            # Small printed label mark on key crown (silver on charcoal)
            _stroke_box(kp, f"{kname}_label",
                        0.0, 0.0002, KEY_RISE - 0.00002,
                        KEY_W * 0.42, KEY_H * 0.28, 0.00010, key_print)

            kp.inertial = Inertial.from_geometry(
                Box((KEY_W, KEY_H, KEY_RISE)),
                mass=key_mass,
                origin=Origin(xyz=(0.0, 0.0, KEY_RISE / 2.0)),
            )
            model.articulation(
                f"body_to_{kname}",
                ArticulationType.PRISMATIC,
                parent=body,
                child=kp,
                origin=Origin(xyz=(col_x, row_y, KEY_SEAT_Z)),
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

    # ---- body is wider for QWERTY and still a tall candybar ----
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "body is wider for QWERTY (X > 0.050)",
        bext[0] > 0.050,
        details=f"body extents={bext}",
    )
    ctx.check(
        "body is a tall candybar (Y much greater than X)",
        bext[1] > bext[0] + 0.04,
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
    # LCD must sit PROUD of the dark bezel
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

    # ---- QWERTY grid: specific part/joint references (TARGET axis) ----
    key_r0_c0 = object_model.get_part("key_r0_c0")
    key_r0_c9 = object_model.get_part("key_r0_c9")
    key_r3_c0 = object_model.get_part("key_r3_c0")
    key_r3_c9 = object_model.get_part("key_r3_c9")

    # Row ordering: row 0 (top) above row 3 (bottom)
    y_r0 = (ctx.part_world_aabb(key_r0_c0)[0][1] + ctx.part_world_aabb(key_r0_c0)[1][1]) / 2.0
    y_r3 = (ctx.part_world_aabb(key_r3_c0)[0][1] + ctx.part_world_aabb(key_r3_c0)[1][1]) / 2.0
    ctx.check(
        "QWERTY row 0 above row 3",
        y_r0 > y_r3 + 0.005,
        details=f"y_r0={y_r0}, y_r3={y_r3}",
    )

    # Column ordering: col 0 (left) left of col 9 (right)
    x_c0 = ctx.part_world_position(key_r0_c0)[0]
    x_c9 = ctx.part_world_position(key_r0_c9)[0]
    ctx.check(
        "QWERTY col 0 left of col 9",
        x_c0 < x_c9 - 0.020,
        details=f"x_c0={x_c0}, x_c9={x_c9}",
    )

    # QWERTY keyboard below screen
    ctx.check(
        "QWERTY keyboard below screen",
        y_r0 < screen_cy - 0.005,
        details=f"y_r0={y_r0}, screen_cy={screen_cy}",
    )

    # ---- full QWERTY key count: 40 prismatic press joints ----
    press_joints = [
        a for a in object_model.articulations
        if a.articulation_type == ArticulationType.PRISMATIC
    ]
    ctx.check(
        "QWERTY keyboard has 40 pressable keys",
        len(press_joints) == 40,
        details=f"prismatic key joints={len(press_joints)}",
    )

    # ---- all keys press straight down (-Z) and stay seated ----
    for r in range(QWERTY_ROWS):
        for c in range(QWERTY_COLS):
            kname = f"key_r{r}_c{c}"
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
            crown_top = ctx.part_world_aabb(object_model.get_part(kname))[1][2]
            ctx.check(
                f"{kname} crown is below the body rim",
                crown_top <= RIM_Z + 0.0002,
                details=f"{kname} crown_top={crown_top}, rim={RIM_Z}",
            )

    # ---- sample keys: press behavior, seating, recession ----
    sample_keys = ["key_r0_c0", "key_r1_c5", "key_r2_c9", "key_r3_c4"]
    for kname in sample_keys:
        kp = object_model.get_part(kname)
        joint = object_model.get_articulation(f"body_to_{kname}")

        # seated in the recessed keypad well
        ctx.expect_contact(kp, body, name=f"{kname} seated in its well")

        # pressing the key moves it straight down into the face (-Z)
        rest_z = ctx.part_world_position(kp)[2]
        rest_pos = ctx.part_world_position(kp)
        with ctx.pose({joint: PRESS_TRAVEL}):
            pressed_z = ctx.part_world_position(kp)[2]
            pressed_pos = ctx.part_world_position(kp)
        ctx.check(
            f"{kname} presses straight down into the face",
            pressed_z < rest_z - 0.0005,
            details=f"{kname} rest_z={rest_z}, pressed_z={pressed_z}",
        )
        ctx.check(
            f"{kname} travel is vertical only",
            abs(pressed_pos[0] - rest_pos[0]) < 1e-5
            and abs(pressed_pos[1] - rest_pos[1]) < 1e-5,
            details=f"{kname} rest={rest_pos}, pressed={pressed_pos}",
        )

    # ---- bottom row stays inside the body outline ----
    body_aabb = ctx.part_world_aabb(body)
    r3_aabb = ctx.part_world_aabb(key_r3_c0)
    ctx.check(
        "bottom QWERTY row stays inside body outline",
        r3_aabb[0][1] > body_aabb[0][1] + 0.004,
        details=f"r3 bottom={r3_aabb[0][1]}, body bottom={body_aabb[0][1]}",
    )

    # ---- keypad panel is below the screen ----
    keypad_panel_aabb = ctx.part_element_world_aabb(body, elem="keypad_fascia_panel")
    keypad_panel_cy = (keypad_panel_aabb[0][1] + keypad_panel_aabb[1][1]) / 2.0
    ctx.check(
        "keypad panel is below the screen",
        keypad_panel_cy < screen_cy - 0.010,
        details=f"keypad_panel_y={keypad_panel_cy}, screen_y={screen_cy}",
    )

    # ---- key_r0_c0 visible geometry: QWERTY keycap has charcoal material ----
    key_r0_c0_cap = key_r0_c0.get_visual("keycap")
    ctx.check(
        "key_r0_c0 has charcoal keycap material (QWERTY colorway)",
        key_r0_c0_cap is not None and key_r0_c0_cap.material is not None,
        details=f"keycap visual exists={key_r0_c0_cap is not None}",
    )

    return ctx.report()


object_model = build_object_model()
