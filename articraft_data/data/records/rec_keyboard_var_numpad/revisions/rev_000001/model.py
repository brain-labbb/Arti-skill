from __future__ import annotations

# Full-size black wireless chiclet computer keyboard.
# Frame: width along +X, depth along +Y, thickness along +Z.
#   - chassis is a slim black filleted slab with a slightly raised back edge
#     and a recessed key well; key centerline at z near the well floor.
#   - every keycap is a small rounded-rect chiclet that PRESSES straight down
#     (prismatic along -Z, ~1.5mm travel). Caps are laid out in a standard
#     compact layout (function row, number row, QWERTY block, ASDF/ZXCV rows,
#     bottom modifier+space row) plus a right-side arrow cluster.
#   - wider caps for Space/Shift/Enter/Tab/Backspace/Caps.
# Key links use indexed names key_r{row}_c{col} (2D repeated layout).

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    LoftGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    mesh_from_geometry,
    rounded_rect_profile,
)

# ---------------------------------------------------------------------------
# Overall dimensions (meters).
# ---------------------------------------------------------------------------
KEYBOARD_WIDTH = 0.450
KEYBOARD_DEPTH = 0.130
CHASSIS_THICK = 0.020  # nominal slab thickness (front edge)
BACK_RAISE = 0.006  # extra height at the raised back edge

# Key-well recess geometry.
WELL_MARGIN_X = 0.010  # chassis border on each side of the key field
WELL_MARGIN_Y_FRONT = 0.009
WELL_MARGIN_Y_BACK = 0.009
WELL_DEPTH = 0.005  # how far the key well is recessed below the top deck
WELL_FLOOR_Z = CHASSIS_THICK - WELL_DEPTH  # z of the recessed well floor (front)

# Chiclet key unit geometry.
KEY_UNIT = 0.0188  # 1u pitch in X (cell size, center-to-center)
ROW_PITCH = 0.0188  # row pitch in Y
KEY_GAP = 0.0028  # gap between adjacent keycaps (clean per-key collision)
KEYCAP_HEIGHT = 0.0075  # keycap body height above the well floor
KEYCAP_CORNER = 0.0014
KEY_TRAVEL = 0.0015  # prismatic press distance (~1.5mm)

# ---------------------------------------------------------------------------
# Layout: each row is a list of width units (in "u"). Standard full-size
# layout with numeric keypad, read back(top of image, +Y) to front(-Y).
# Row 0 = function row (back), Row 5 = spacebar row (front).
# Wider caps: Backspace, Tab, Caps, Enter, Shift, Space.
# Numpad block is appended right of the nav cluster with a gap.
# ---------------------------------------------------------------------------
NUMPAD_GAP_U = 0.7  # gap in key units between nav cluster and numpad block

# Number of main+nav columns per row (before numpad keys begin).
_MAIN_COLS = [16, 16, 16, 15, 14, 11]

LAYOUT = [
    # Row 0: Esc, F1..F12, then a right nav trio (Del/Home/End) = 16 keys, 1u
    [1.0] * 16,
    # Row 1: `~ 1 2 3 4 5 6 7 8 9 0 - = Backspace(2u) + nav (PgUp/PgDn)
    #        + numpad: NumLk / * - = 16 + 4 = 20 keys
    [1.0] * 13 + [2.0] + [1.0, 1.0] + [1.0, 1.0, 1.0, 1.0],
    # Row 2: Tab(1.5u) Q W E R T Y U I O P [ ] \(1.5u) + 2 nav
    #        + numpad: 7 8 9 + = 16 + 4 = 20 keys
    [1.5] + [1.0] * 12 + [1.5] + [1.0, 1.0] + [1.0, 1.0, 1.0, 1.0],
    # Row 3: Caps(1.75u) A S D F G H J K L ; ' Enter(2.25u) + 2 nav
    #        + numpad: 4 5 6 = 15 + 3 = 18 keys
    [1.75] + [1.0] * 11 + [2.25] + [1.0, 1.0] + [1.0, 1.0, 1.0],
    # Row 4: Shift(2.25u) Z X C V B N M , . / Shift(1.75u) Up + 1 nav
    #        + numpad: 1 2 3 Enter = 14 + 4 = 18 keys
    [2.25] + [1.0] * 10 + [1.75, 1.0, 1.0] + [1.0, 1.0, 1.0, 1.0],
    # Row 5: Ctrl Fn Win Alt Space(6.0u) Alt Ctrl Fn2 Left Down Right
    #        + numpad: 0(2u) . = 11 + 2 = 13 keys
    [1.25, 1.0, 1.0, 1.25, 6.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0] + [2.0, 1.0],
]

# Row spans include the numpad gap for rows that have numpad keys.
_ROW_SPANS = [
    sum(row) + (NUMPAD_GAP_U if len(row) > _MAIN_COLS[i] else 0.0)
    for i, row in enumerate(LAYOUT)
]
FIELD_UNITS = max(_ROW_SPANS)
FIELD_WIDTH = FIELD_UNITS * KEY_UNIT
NUM_ROWS = len(LAYOUT)


def _keycap_mesh(name: str, width_u: float):
    """Rounded-rect chiclet keycap, slightly tapered toward the top."""
    w = width_u * KEY_UNIT - KEY_GAP
    d = ROW_PITCH - KEY_GAP
    top_w = w - 0.0016
    top_d = d - 0.0016
    lower = rounded_rect_profile(w, d, min(KEYCAP_CORNER, w * 0.4, d * 0.4))
    upper = rounded_rect_profile(top_w, top_d, min(KEYCAP_CORNER, top_w * 0.4, top_d * 0.4))
    geom = LoftGeometry(
        [
            [(x, y, 0.0) for x, y in lower],
            [(x, y, KEYCAP_HEIGHT) for x, y in upper],
        ],
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geom, name)


def _chassis_slab_mesh() -> object:
    """Slim slab top deck with a slight back rake (raised back edge)."""
    half_w = KEYBOARD_WIDTH * 0.5
    front_y = -KEYBOARD_DEPTH * 0.5
    back_y = KEYBOARD_DEPTH * 0.5
    geom = MeshGeometry()
    v = [
        (-half_w, front_y, 0.0),
        (half_w, front_y, 0.0),
        (half_w, back_y, 0.0),
        (-half_w, back_y, 0.0),
        (-half_w, front_y, CHASSIS_THICK),
        (half_w, front_y, CHASSIS_THICK),
        (half_w, back_y, CHASSIS_THICK + BACK_RAISE),
        (-half_w, back_y, CHASSIS_THICK + BACK_RAISE),
    ]
    ids = [geom.add_vertex(*p) for p in v]

    def quad(a, b, c, d):
        geom.add_face(ids[a], ids[b], ids[c])
        geom.add_face(ids[a], ids[c], ids[d])

    quad(0, 3, 2, 1)  # bottom
    quad(4, 5, 6, 7)  # top
    quad(0, 1, 5, 4)  # front
    quad(1, 2, 6, 5)  # right
    quad(2, 3, 7, 6)  # back
    quad(3, 0, 4, 7)  # left
    return mesh_from_geometry(geom, "chassis_slab")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="computer_keyboard")

    chassis_black = model.material("chassis_black", rgba=(0.09, 0.09, 0.10, 1.0))
    keycap_gray = model.material("keycap_dark_gray", rgba=(0.16, 0.16, 0.17, 1.0))

    half_d = KEYBOARD_DEPTH * 0.5

    # ---- chassis (root) ----
    # The slab top deck is the key-well floor. A raised rim frame around the key
    # field makes the keys read as recessed in a well. Keycaps seat onto the
    # deck with a tiny seating embed (intentional, allowed in run_tests).
    chassis = model.part("chassis")
    chassis.visual(_chassis_slab_mesh(), material=chassis_black, name="chassis_slab")

    half_w = KEYBOARD_WIDTH * 0.5

    # Key-field footprint (centered in X; spans the deck in Y between margins).
    field_w = FIELD_WIDTH
    field_d = KEYBOARD_DEPTH - WELL_MARGIN_Y_FRONT - WELL_MARGIN_Y_BACK
    field_cy = -(WELL_MARGIN_Y_FRONT - WELL_MARGIN_Y_BACK) * 0.5
    rim_h = WELL_DEPTH + KEYCAP_HEIGHT * 0.45  # rim taller than seated key base
    rim_t = WELL_MARGIN_X  # rim wall thickness
    deck_top = CHASSIS_THICK  # nominal (front) deck top; rim rides on it

    # Raised rim walls (left/right/front/back) forming the recessed key well.
    chassis.visual(
        Box((rim_t, KEYBOARD_DEPTH, rim_h)),
        origin=Origin(xyz=(-half_w + rim_t * 0.5, 0.0, deck_top + rim_h * 0.5)),
        material=chassis_black,
        name="rim_left",
    )
    chassis.visual(
        Box((rim_t, KEYBOARD_DEPTH, rim_h)),
        origin=Origin(xyz=(half_w - rim_t * 0.5, 0.0, deck_top + rim_h * 0.5)),
        material=chassis_black,
        name="rim_right",
    )
    chassis.visual(
        Box((KEYBOARD_WIDTH, WELL_MARGIN_Y_FRONT, rim_h)),
        origin=Origin(
            xyz=(0.0, -half_d + WELL_MARGIN_Y_FRONT * 0.5, deck_top + rim_h * 0.5)
        ),
        material=chassis_black,
        name="rim_front",
    )
    chassis.visual(
        Box((KEYBOARD_WIDTH, WELL_MARGIN_Y_BACK, rim_h)),
        origin=Origin(
            xyz=(0.0, half_d - WELL_MARGIN_Y_BACK * 0.5, deck_top + BACK_RAISE + rim_h * 0.5)
        ),
        material=chassis_black,
        name="rim_back",
    )
    _ = (field_w, field_d)  # documented for layout reference

    chassis.inertial = Inertial.from_geometry(
        Box((KEYBOARD_WIDTH, KEYBOARD_DEPTH, CHASSIS_THICK + BACK_RAISE)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, (CHASSIS_THICK + BACK_RAISE) * 0.5)),
    )

    # ---- keys ----
    # Key field left-aligned in X (all rows share a common left edge); row 0 at
    # the back (+Y), last row at front (-Y). The numpad block sits to the right
    # of the nav cluster with a visible gap.
    field_back_y = field_cy + field_d * 0.5 - ROW_PITCH * 0.5 - 0.001
    field_left_x = -FIELD_WIDTH * 0.5
    keycap_mesh_cache: dict[float, object] = {}

    for row_index, row in enumerate(LAYOUT):
        x_cursor = field_left_x
        row_cy = field_back_y - row_index * ROW_PITCH
        for col_index, width_u in enumerate(row):
            # Insert numpad gap when crossing from nav cluster to numpad block.
            if col_index == _MAIN_COLS[row_index]:
                x_cursor += NUMPAD_GAP_U * KEY_UNIT
            key_cx = x_cursor + (width_u * KEY_UNIT) * 0.5

            # local deck height for this key (linear back-rake along Y); keycap
            # seats onto the deck with a small intentional seating embed.
            t = (row_cy + half_d) / KEYBOARD_DEPTH  # 0 front .. 1 back
            local_top_z = CHASSIS_THICK + BACK_RAISE * t
            seat_z = local_top_z - 0.0008  # keycap base embeds ~0.8mm into deck

            cache_key = round(width_u, 4)
            mesh = keycap_mesh_cache.get(cache_key)
            if mesh is None:
                mesh = _keycap_mesh(f"keycap_w{int(round(width_u * 100))}", width_u)
                keycap_mesh_cache[cache_key] = mesh

            part_name = f"key_r{row_index}_c{col_index:02d}"
            key = model.part(part_name)
            key.visual(mesh, material=keycap_gray, name="keycap")
            cap_w = width_u * KEY_UNIT - KEY_GAP
            cap_d = ROW_PITCH - KEY_GAP
            key.inertial = Inertial.from_geometry(
                Box((cap_w, cap_d, KEYCAP_HEIGHT)),
                mass=0.0012 * max(1.0, width_u),
                origin=Origin(xyz=(0.0, 0.0, KEYCAP_HEIGHT * 0.5)),
            )
            model.articulation(
                f"chassis_to_{part_name}",
                ArticulationType.PRISMATIC,
                parent=chassis,
                child=key,
                origin=Origin(xyz=(key_cx, row_cy, seat_z)),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(
                    effort=3.0,
                    velocity=0.05,
                    lower=0.0,
                    upper=KEY_TRAVEL,
                ),
            )
            x_cursor += width_u * KEY_UNIT

    return model


def run_tests():
    ctx = TestContext(object_model)

    chassis = object_model.get_part("chassis")

    # Chassis is wider (X) than deep (Y).
    cb = ctx.part_world_aabb(chassis)
    cw = cb[1][0] - cb[0][0]
    cd = cb[1][1] - cb[0][1]
    ctx.check(
        "chassis wider than deep",
        cw > cd + 0.05,
        details=f"width={cw:.3f}, depth={cd:.3f}",
    )

    # Keys sit in a recessed well: the rim frame rises above the chassis deck
    # and surrounds the lower part of the seated keycaps (rim top is above the
    # keycap base so the keys are nested in the well, while their tops protrude
    # slightly as on a real chiclet board).
    rim = ctx.part_element_world_aabb(chassis, elem="rim_left")
    slab = ctx.part_element_world_aabb(chassis, elem="chassis_slab")
    a_key = ctx.part_world_aabb(object_model.get_part("key_r2_c05"))
    ctx.check(
        "key well rim rises above the chassis deck",
        rim[1][2] > slab[1][2] + 0.002,
        details=f"rim_top={rim[1][2]:.4f}, deck_top={slab[1][2]:.4f}",
    )
    ctx.check(
        "keycap base nests inside the well rim",
        a_key[0][2] < rim[1][2] - 0.001,
        details=f"key_base={a_key[0][2]:.4f}, rim_top={rim[1][2]:.4f}",
    )

    # Every keycap seats with a small intentional embed into the solid deck
    # (chassis_slab) so it reads as seated in the well rather than floating on
    # top. Scope the seating allowance to that exact element pair for all keys.
    for r_index, r_row in enumerate(LAYOUT):
        for c_index in range(len(r_row)):
            kp = object_model.get_part(f"key_r{r_index}_c{c_index:02d}")
            ctx.allow_overlap(
                kp,
                chassis,
                elem_a="keycap",
                elem_b="chassis_slab",
                reason="Keycap base intentionally seats a fraction of a mm into the solid deck (well floor).",
            )

    # Representative key sample spanning every row of the layout, including
    # the numpad block (cols beyond _MAIN_COLS in each row).
    sample = [
        "key_r0_c00",  # Esc / func row
        "key_r0_c08",  # mid func row
        "key_r1_c00",  # tilde / number row
        "key_r1_c13",  # Backspace (wide)
        "key_r2_c00",  # Tab (wide)
        "key_r2_c05",  # QWERTY mid
        "key_r3_c00",  # Caps (wide)
        "key_r3_c12",  # Enter (wide)
        "key_r4_c00",  # Left Shift (wide)
        "key_r4_c06",  # ZXCV mid
        "key_r5_c04",  # Spacebar (wide)
        "key_r5_c01",  # Fn modifier
        "key_r5_c09",  # Right arrow
        # Numpad keys (right of nav cluster):
        "key_r1_c16",  # NumLk
        "key_r2_c18",  # 9
        "key_r3_c16",  # 5
        "key_r4_c15",  # 2
        "key_r5_c11",  # 0 (wide)
    ]

    for part_name in sample:
        key = object_model.get_part(part_name)
        joint = object_model.get_articulation(f"chassis_to_{part_name}")

        # Key footprint sits over the chassis (seated in the well).
        ctx.expect_overlap(
            key,
            chassis,
            axes="xy",
            min_overlap=0.002,
            name=f"{part_name} seated over chassis well",
        )

        # Key presses straight down with no lateral drift.
        rest = ctx.part_world_aabb(key)
        rest_top = rest[1][2]
        rest_cx = (rest[0][0] + rest[1][0]) * 0.5
        rest_cy = (rest[0][1] + rest[1][1]) * 0.5
        with ctx.pose({joint: KEY_TRAVEL}):
            pressed = ctx.part_world_aabb(key)
        pressed_top = pressed[1][2]
        pressed_cx = (pressed[0][0] + pressed[1][0]) * 0.5
        pressed_cy = (pressed[0][1] + pressed[1][1]) * 0.5
        ctx.check(
            f"{part_name} presses straight down",
            pressed_top < rest_top - KEY_TRAVEL * 0.6
            and abs(pressed_cx - rest_cx) < 1e-4
            and abs(pressed_cy - rest_cy) < 1e-4,
            details=(
                f"rest_top={rest_top:.5f}, pressed_top={pressed_top:.5f}, "
                f"dx={pressed_cx - rest_cx:.6f}, dy={pressed_cy - rest_cy:.6f}"
            ),
        )

    # Spacebar is wider than a regular letter key.
    space = object_model.get_part("key_r5_c04")
    letter = object_model.get_part("key_r2_c05")
    sb = ctx.part_world_aabb(space)
    lb = ctx.part_world_aabb(letter)
    space_w = sb[1][0] - sb[0][0]
    letter_w = lb[1][0] - lb[0][0]
    ctx.check(
        "spacebar wider than a letter key",
        space_w > letter_w + 0.02,
        details=f"space_w={space_w:.3f}, letter_w={letter_w:.3f}",
    )

    # Numpad block exists and is positioned to the right of the main key field.
    # Verify a representative numpad key (NumLk at r1_c16) is right of the
    # rightmost nav key in the same row, with a visible gap between them.
    numlk = object_model.get_part("key_r1_c16")
    nav_right = object_model.get_part("key_r1_c15")  # rightmost nav key in row 1
    numlk_box = ctx.part_world_aabb(numlk)
    nav_box = ctx.part_world_aabb(nav_right)
    numlk_left = numlk_box[0][0]
    nav_right_edge = nav_box[1][0]
    gap_x = numlk_left - nav_right_edge
    ctx.check(
        "numpad block right of nav cluster with visible gap",
        gap_x > 0.005,
        details=f"numlk_left={numlk_left:.4f}, nav_right={nav_right_edge:.4f}, gap={gap_x:.4f}",
    )

    # Numpad zero key is double-wide (2u).
    zero_key = object_model.get_part("key_r5_c11")
    period_key = object_model.get_part("key_r5_c12")
    zero_box = ctx.part_world_aabb(zero_key)
    period_box = ctx.part_world_aabb(period_key)
    zero_w = zero_box[1][0] - zero_box[0][0]
    period_w = period_box[1][0] - period_box[0][0]
    ctx.check(
        "numpad zero key wider than period key",
        zero_w > period_w + 0.010,
        details=f"zero_w={zero_w:.4f}, period_w={period_w:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
