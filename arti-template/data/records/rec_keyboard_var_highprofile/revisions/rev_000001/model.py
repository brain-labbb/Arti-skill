from __future__ import annotations

# Full-size wireless computer keyboard with stepped high-profile aluminium case.
# Frame: width along +X, depth along +Y, thickness along +Z.
#   - chassis is a tall stepped aluminium case body (25mm) with a visible
#     exterior step groove and deep key well; thick rim walls rise well above
#     the keycap tops so keys sit deep in a tall tray (mechanical-keyboard
#     high-profile case look).
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
KEYBOARD_WIDTH = 0.360
KEYBOARD_DEPTH = 0.130
CHASSIS_THICK = 0.025  # high-profile case body height (front edge)
BACK_RAISE = 0.003  # extra height at the raised back edge

# Key-well recess geometry.
WELL_MARGIN_X = 0.010  # chassis border on each side of the key field
WELL_MARGIN_Y_FRONT = 0.009
WELL_MARGIN_Y_BACK = 0.009
WELL_DEPTH = 0.003  # how far the key well is recessed below the top deck
WELL_FLOOR_Z = CHASSIS_THICK - WELL_DEPTH  # z of the recessed well floor (front)

# High-profile case walls rise well above keycap tops for deep-tray look.
WALL_ABOVE_KEYCAPS = 0.006  # rim walls extend 6mm above keycap tops

# Chiclet key unit geometry.
KEY_UNIT = 0.0188  # 1u pitch in X (cell size, center-to-center)
ROW_PITCH = 0.0188  # row pitch in Y
KEY_GAP = 0.0028  # gap between adjacent keycaps (clean per-key collision)
KEYCAP_HEIGHT = 0.0075  # keycap body height above the well floor
KEYCAP_CORNER = 0.0014
KEY_TRAVEL = 0.0015  # prismatic press distance (~1.5mm)

# ---------------------------------------------------------------------------
# Layout: each row is a list of width units (in "u"). Standard compact
# full-size layout, read back(top of image, +Y) to front(-Y).
# Row 0 = function row (back), Row 5 = spacebar row (front).
# Wider caps: Backspace, Tab, Caps, Enter, Shift, Space.
# ---------------------------------------------------------------------------
LAYOUT = [
    # Row 0: Esc, F1..F12, then a right nav trio (Del/Home/End) = 16 keys, 1u
    [1.0] * 16,
    # Row 1: `~ 1 2 3 4 5 6 7 8 9 0 - = Backspace(2u) + nav (PgUp/PgDn) = 16
    [1.0] * 13 + [2.0] + [1.0, 1.0],
    # Row 2: Tab(1.5u) Q W E R T Y U I O P [ ] \(1.5u) + 2 nav = 16
    [1.5] + [1.0] * 12 + [1.5] + [1.0, 1.0],
    # Row 3: Caps(1.75u) A S D F G H J K L ; ' Enter(2.25u) + 2 nav = 15
    [1.75] + [1.0] * 11 + [2.25] + [1.0, 1.0],
    # Row 4: Shift(2.25u) Z X C V B N M , . / Shift(1.75u) Up + 1 nav = 15
    [2.25] + [1.0] * 10 + [1.75, 1.0, 1.0],
    # Row 5: Ctrl Fn Win Alt Space(6.0u) Alt Ctrl Fn2 Left Down Right = 11
    [1.25, 1.0, 1.0, 1.25, 6.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
]

_ROW_SPANS = [sum(row) for row in LAYOUT]
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
    """Tall high-profile case body with visible exterior step groove.

    The case body has a horizontal channel/groove around the exterior at
    ~15mm height, giving the characteristic stepped mechanical-keyboard
    aluminium case profile.  The groove is 2mm tall and 1mm deep.
    """
    half_w = KEYBOARD_WIDTH * 0.5
    front_y = -KEYBOARD_DEPTH * 0.5
    back_y = KEYBOARD_DEPTH * 0.5

    body_h = CHASSIS_THICK  # 25mm
    back_h = body_h + BACK_RAISE  # 28mm at back

    # Exterior step groove geometry.
    groove_z = 0.015  # groove bottom at 15mm from base
    groove_h = 0.002  # groove is 2mm tall
    groove_d = 0.001  # groove is 1mm deep (inset)

    def _rake(z: float) -> float:
        """Linear back-rake offset at height z."""
        return BACK_RAISE * (z / body_h) if body_h > 0 else 0.0

    geom = MeshGeometry()
    ids: list[int] = []

    def add_v(x: float, y: float, z: float) -> None:
        ids.append(geom.add_vertex(x, y, z))

    def tri(a: int, b: int, c: int) -> None:
        geom.add_face(ids[a], ids[b], ids[c])

    def quad(a: int, b: int, c: int, d: int) -> None:
        tri(a, b, c)
        tri(a, c, d)

    gz0 = groove_z
    gz0_b = groove_z + _rake(groove_z)
    gz1 = groove_z + groove_h
    gz1_b = gz1 + _rake(gz1)

    # Groove inset boundary.
    gx0, gx1 = -half_w + groove_d, half_w - groove_d
    gy0, gy1 = front_y + groove_d, back_y - groove_d

    # --- 24 vertices for the stepped profile ---
    # Bottom (0-3): full footprint at z=0
    add_v(-half_w, front_y, 0.0)  # 0
    add_v(half_w, front_y, 0.0)  # 1
    add_v(half_w, back_y, 0.0)  # 2
    add_v(-half_w, back_y, 0.0)  # 3

    # Groove bottom outer (4-7): full footprint at z=gz0
    add_v(-half_w, front_y, gz0)  # 4
    add_v(half_w, front_y, gz0)  # 5
    add_v(half_w, back_y, gz0_b)  # 6
    add_v(-half_w, back_y, gz0_b)  # 7

    # Groove bottom inner (8-11): inset footprint at z=gz0
    add_v(gx0, gy0, gz0)  # 8
    add_v(gx1, gy0, gz0)  # 9
    add_v(gx1, gy1, gz0_b)  # 10
    add_v(gx0, gy1, gz0_b)  # 11

    # Groove top inner (12-15): inset footprint at z=gz1
    add_v(gx0, gy0, gz1)  # 12
    add_v(gx1, gy0, gz1)  # 13
    add_v(gx1, gy1, gz1_b)  # 14
    add_v(gx0, gy1, gz1_b)  # 15

    # Groove top outer (16-19): full footprint at z=gz1
    add_v(-half_w, front_y, gz1)  # 16
    add_v(half_w, front_y, gz1)  # 17
    add_v(half_w, back_y, gz1_b)  # 18
    add_v(-half_w, back_y, gz1_b)  # 19

    # Top (20-23): full footprint at z=body_h
    add_v(-half_w, front_y, body_h)  # 20
    add_v(half_w, front_y, body_h)  # 21
    add_v(half_w, back_y, back_h)  # 22
    add_v(-half_w, back_y, back_h)  # 23

    # --- Faces ---
    # Bottom
    quad(0, 3, 2, 1)

    # Lower outer walls (z=0 to groove_z, full footprint)
    quad(0, 1, 5, 4)  # front
    quad(1, 2, 6, 5)  # right
    quad(2, 3, 7, 6)  # back
    quad(3, 0, 4, 7)  # left

    # Groove bottom shelf (horizontal, facing up, at z=gz0)
    quad(4, 5, 9, 8)  # front shelf
    quad(5, 6, 10, 9)  # right shelf
    quad(6, 7, 11, 10)  # back shelf
    quad(7, 4, 8, 11)  # left shelf

    # Groove inner walls (vertical, from gz0 to gz1, at inset position)
    quad(8, 9, 13, 12)  # front inner
    quad(9, 10, 14, 13)  # right inner
    quad(10, 11, 15, 14)  # back inner
    quad(11, 8, 12, 15)  # left inner

    # Groove top shelf (horizontal, facing down, at z=gz1)
    quad(16, 17, 13, 12)  # front shelf (normal down)
    quad(17, 18, 14, 13)  # right shelf
    quad(18, 19, 15, 14)  # back shelf
    quad(19, 16, 12, 15)  # left shelf

    # Upper outer walls (gz1 to body_h, full footprint)
    quad(16, 17, 21, 20)  # front
    quad(17, 18, 22, 21)  # right
    quad(18, 19, 23, 22)  # back
    quad(19, 16, 20, 23)  # left

    # Top
    quad(20, 21, 22, 23)

    return mesh_from_geometry(geom, "chassis_slab")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="computer_keyboard")

    # Anodized-aluminium-look case material (dark gunmetal with slight blue tint).
    chassis_aluminium = model.material(
        "anodized_gunmetal", rgba=(0.13, 0.13, 0.16, 1.0)
    )
    keycap_gray = model.material("keycap_dark_gray", rgba=(0.16, 0.16, 0.17, 1.0))

    half_d = KEYBOARD_DEPTH * 0.5

    # ---- chassis (root) ----
    # Stepped high-profile case: tall aluminium body with an exterior step
    # groove and deep key well. Rim walls rise well above keycap tops so keys
    # sit deep in a tall tray (mechanical-keyboard high-profile case look).
    chassis = model.part("chassis")
    chassis.visual(
        _chassis_slab_mesh(), material=chassis_aluminium, name="chassis_slab"
    )

    half_w = KEYBOARD_WIDTH * 0.5

    # Key-field footprint (centered in X; spans the deck in Y between margins).
    field_w = FIELD_WIDTH
    field_d = KEYBOARD_DEPTH - WELL_MARGIN_Y_FRONT - WELL_MARGIN_Y_BACK
    field_cy = -(WELL_MARGIN_Y_FRONT - WELL_MARGIN_Y_BACK) * 0.5

    # Rim walls rise well above keycap tops for the deep-tray look.
    # seat_z ~ deck_top - 0.0008; keycap top ~ seat_z + KEYCAP_HEIGHT
    # rim_h chosen so rim tops sit WALL_ABOVE_KEYCAPS above keycap tops.
    rim_h = KEYCAP_HEIGHT - 0.0008 + WALL_ABOVE_KEYCAPS
    rim_t = WELL_MARGIN_X  # rim wall thickness
    deck_top = CHASSIS_THICK  # nominal (front) deck top; rim rides on it

    # Raised rim walls (left/right/front/back) forming the deep key well.
    chassis.visual(
        Box((rim_t, KEYBOARD_DEPTH, rim_h)),
        origin=Origin(xyz=(-half_w + rim_t * 0.5, 0.0, deck_top + rim_h * 0.5)),
        material=chassis_aluminium,
        name="rim_left",
    )
    chassis.visual(
        Box((rim_t, KEYBOARD_DEPTH, rim_h)),
        origin=Origin(xyz=(half_w - rim_t * 0.5, 0.0, deck_top + rim_h * 0.5)),
        material=chassis_aluminium,
        name="rim_right",
    )
    chassis.visual(
        Box((KEYBOARD_WIDTH, WELL_MARGIN_Y_FRONT, rim_h)),
        origin=Origin(
            xyz=(0.0, -half_d + WELL_MARGIN_Y_FRONT * 0.5, deck_top + rim_h * 0.5)
        ),
        material=chassis_aluminium,
        name="rim_front",
    )
    chassis.visual(
        Box((KEYBOARD_WIDTH, WELL_MARGIN_Y_BACK, rim_h)),
        origin=Origin(
            xyz=(
                0.0,
                half_d - WELL_MARGIN_Y_BACK * 0.5,
                deck_top + BACK_RAISE + rim_h * 0.5,
            )
        ),
        material=chassis_aluminium,
        name="rim_back",
    )
    _ = (field_w, field_d)  # documented for layout reference

    chassis.inertial = Inertial.from_geometry(
        Box((KEYBOARD_WIDTH, KEYBOARD_DEPTH, CHASSIS_THICK + BACK_RAISE)),
        mass=0.80,  # heavier aluminium high-profile case
        origin=Origin(xyz=(0.0, 0.0, (CHASSIS_THICK + BACK_RAISE) * 0.5)),
    )

    # ---- keys ----
    # Key field centered in X; row 0 at the back (+Y), last row at front (-Y).
    field_back_y = field_cy + field_d * 0.5 - ROW_PITCH * 0.5 - 0.001
    keycap_mesh_cache: dict[float, object] = {}

    for row_index, row in enumerate(LAYOUT):
        row_width = _ROW_SPANS[row_index] * KEY_UNIT
        x_cursor = -row_width * 0.5
        row_cy = field_back_y - row_index * ROW_PITCH
        for col_index, width_u in enumerate(row):
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

    # Stepped high-profile case: the chassis_slab body is significantly taller
    # than the original low-profile slab (>22mm), and the rim walls rise well
    # above keycap tops creating the deep-tray mechanical keyboard look.
    slab = ctx.part_element_world_aabb(chassis, elem="chassis_slab")
    slab_height = slab[1][2] - slab[0][2]
    ctx.check(
        "stepped high-profile case body is tall (chassis_slab > 22mm)",
        slab_height > 0.022,
        details=f"case_height={slab_height:.3f}",
    )

    rim = ctx.part_element_world_aabb(chassis, elem="rim_left")
    a_key = ctx.part_world_aabb(object_model.get_part("key_r2_c05"))
    key_top = a_key[1][2]
    ctx.check(
        "rim walls rise well above keycap tops (deep-tray high-profile)",
        rim[1][2] > key_top + 0.003,
        details=f"rim_top={rim[1][2]:.4f}, keycap_top={key_top:.4f}",
    )
    ctx.check(
        "keycap base nests deep inside the well rim",
        a_key[0][2] < rim[1][2] - 0.003,
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

    # Representative key sample spanning every row of the layout.
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

    return ctx.report()


object_model = build_object_model()
