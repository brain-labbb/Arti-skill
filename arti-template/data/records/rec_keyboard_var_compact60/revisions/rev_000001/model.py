from __future__ import annotations

from collections import defaultdict

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    LoftGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


U = 0.0185
TOTAL_UNITS_X = 15.0
ROW_CENTER_OFFSET = 2.65
CHASSIS_WIDTH = 0.296
CHASSIS_DEPTH = 0.115
CHASSIS_TOP_INSET = 0.006
CHASSIS_FRONT_TOP_Z = 0.0065
CHASSIS_REAR_TOP_Z = 0.0145
KEY_GAP = 0.0020
KEY_DEPTH = 0.0162
KEY_HEIGHT = 0.0044
KEY_BASE_CLEARANCE = 0.0015
CHASSIS_SLOPE = (CHASSIS_REAR_TOP_Z - CHASSIS_FRONT_TOP_Z) / (
    CHASSIS_DEPTH - 2.0 * CHASSIS_TOP_INSET
)


FONT_3X5: dict[str, tuple[str, ...]] = {
    "A": ("111", "101", "111", "101", "101"),
    "B": ("110", "101", "110", "101", "110"),
    "C": ("111", "100", "100", "100", "111"),
    "D": ("110", "101", "101", "101", "110"),
    "E": ("111", "100", "110", "100", "111"),
    "F": ("111", "100", "110", "100", "100"),
    "G": ("111", "100", "101", "101", "111"),
    "H": ("101", "101", "111", "101", "101"),
    "I": ("111", "010", "010", "010", "111"),
    "J": ("001", "001", "001", "101", "111"),
    "K": ("101", "101", "110", "101", "101"),
    "L": ("100", "100", "100", "100", "111"),
    "M": ("101", "111", "111", "101", "101"),
    "N": ("101", "111", "111", "111", "101"),
    "O": ("111", "101", "101", "101", "111"),
    "P": ("111", "101", "111", "100", "100"),
    "Q": ("111", "101", "101", "111", "001"),
    "R": ("110", "101", "110", "101", "101"),
    "S": ("111", "100", "111", "001", "111"),
    "T": ("111", "010", "010", "010", "010"),
    "U": ("101", "101", "101", "101", "111"),
    "V": ("101", "101", "101", "101", "010"),
    "W": ("101", "101", "111", "111", "101"),
    "X": ("101", "101", "010", "101", "101"),
    "Y": ("101", "101", "010", "010", "010"),
    "Z": ("111", "001", "010", "100", "111"),
    "0": ("111", "101", "101", "101", "111"),
    "1": ("010", "110", "010", "010", "111"),
    "2": ("111", "001", "111", "100", "111"),
    "3": ("111", "001", "111", "001", "111"),
    "4": ("101", "101", "111", "001", "001"),
    "5": ("111", "100", "111", "001", "111"),
    "6": ("111", "100", "111", "101", "111"),
    "7": ("111", "001", "010", "010", "010"),
    "8": ("111", "101", "111", "101", "111"),
    "9": ("111", "101", "111", "001", "111"),
    "-": ("000", "000", "111", "000", "000"),
    "=": ("000", "111", "000", "111", "000"),
    "`": ("100", "010", "000", "000", "000"),
    "[": ("110", "100", "100", "100", "110"),
    "]": ("011", "001", "001", "001", "011"),
    "\\": ("100", "100", "010", "001", "001"),
    ";": ("000", "010", "000", "010", "100"),
    "'": ("010", "010", "000", "000", "000"),
    ",": ("000", "000", "000", "010", "100"),
    ".": ("000", "000", "000", "000", "010"),
    "/": ("001", "001", "010", "100", "100"),
    " ": ("000", "000", "000", "000", "000"),
}


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _chassis_top_z(y: float) -> float:
    front_y = -CHASSIS_DEPTH * 0.5 + CHASSIS_TOP_INSET
    rear_y = CHASSIS_DEPTH * 0.5 - CHASSIS_TOP_INSET
    t = (y - front_y) / (rear_y - front_y)
    t = max(0.0, min(1.0, t))
    return CHASSIS_FRONT_TOP_Z + (CHASSIS_REAR_TOP_Z - CHASSIS_FRONT_TOP_Z) * t


def _build_chassis_mesh() -> MeshGeometry:
    """Thin tapered base: shallow front lip and a slightly higher rear deck."""
    geom = MeshGeometry()
    inset = CHASSIS_TOP_INSET
    half_w = CHASSIS_WIDTH * 0.5
    half_d = CHASSIS_DEPTH * 0.5

    points = [
        (-half_w, -half_d, 0.0),
        (half_w, -half_d, 0.0),
        (half_w, half_d, 0.0),
        (-half_w, half_d, 0.0),
        (-half_w + inset, -half_d + inset, CHASSIS_FRONT_TOP_Z),
        (half_w - inset, -half_d + inset, CHASSIS_FRONT_TOP_Z),
        (half_w - inset, half_d - inset, CHASSIS_REAR_TOP_Z),
        (-half_w + inset, half_d - inset, CHASSIS_REAR_TOP_Z),
    ]
    ids = [geom.add_vertex(*p) for p in points]
    _add_quad(geom, ids[0], ids[1], ids[2], ids[3])
    _add_quad(geom, ids[4], ids[7], ids[6], ids[5])
    _add_quad(geom, ids[0], ids[4], ids[5], ids[1])
    _add_quad(geom, ids[1], ids[5], ids[6], ids[2])
    _add_quad(geom, ids[2], ids[6], ids[7], ids[3])
    _add_quad(geom, ids[3], ids[7], ids[4], ids[0])
    return geom


def _keycap_mesh(width_units: float, name: str):
    width = width_units * U - KEY_GAP
    lower = rounded_rect_profile(width, KEY_DEPTH, radius=0.0024, corner_segments=5)
    upper = rounded_rect_profile(width - 0.0016, KEY_DEPTH - 0.0014, radius=0.0020, corner_segments=5)
    geom = LoftGeometry(
        [
            [(x, y, 0.0) for x, y in lower],
            [(x, y, KEY_HEIGHT) for x, y in upper],
        ],
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geom, name)


def _safe_label(label: str) -> str:
    names = {
        "`": "grave",
        "-": "minus",
        "=": "equal",
        "[": "lbracket",
        "]": "rbracket",
        "\\": "backslash",
        ";": "semicolon",
        "'": "quote",
        ",": "comma",
        ".": "period",
        "/": "slash",
    }
    if label in names:
        return names[label]
    return label.lower().replace(" ", "_")


def _legend_text(label: str) -> str:
    aliases = {
        "BACK": "BACK",
        "CAPS": "CAPS",
        "ENTER": "ENT",
        "SHIFT": "SHFT",
        "CTRL": "CTRL",
        "SPACE": "SPACE",
        "MENU": "MENU",
        "PRINT": "PRT",
        "SCROLL": "SCR",
        "PAUSE": "PAU",
        "INSERT": "INS",
        "DELETE": "DEL",
        "PGUP": "PGU",
        "PGDN": "PGD",
        "LEFT": "LT",
        "DOWN": "DN",
        "RIGHT": "RT",
    }
    return aliases.get(label, label)


def _add_legend(part, text: str, key_width: float, material) -> None:
    chars = [ch for ch in text.upper() if ch in FONT_3X5]
    if not chars:
        return

    columns = len(chars) * 4 - 1
    pixel = min(0.00145, key_width * 0.66 / max(columns, 1), KEY_DEPTH * 0.30 / 5.0)
    cell = pixel * 1.22
    total_w = (columns - 1) * cell + pixel
    total_h = 4 * cell + pixel
    start_x = -total_w * 0.5 + pixel * 0.5
    start_y = total_h * 0.5 - pixel * 0.5
    z = KEY_HEIGHT + 0.00008
    count = 0

    for char_i, ch in enumerate(chars):
        pattern = FONT_3X5[ch]
        char_x = start_x + char_i * 4 * cell
        for row_i, row in enumerate(pattern):
            for col_i, on in enumerate(row):
                if on != "1":
                    continue
                part.visual(
                    Box((pixel, pixel, 0.00020)),
                    origin=Origin(
                        xyz=(
                            char_x + col_i * cell,
                            0.0009 + start_y - row_i * cell,
                            z,
                        )
                    ),
                    material=material,
                    name=f"legend_{count}",
                )
                count += 1


def _add_key(
    model: ArticulatedObject,
    chassis,
    *,
    label: str,
    width_units: float,
    left_units: float,
    row_y_units: float,
    mesh_cache: dict[float, object],
    label_counts: defaultdict[str, int],
    key_material,
    legend_material,
):
    base_name = _safe_label(label)
    occurrence = label_counts[base_name]
    label_counts[base_name] += 1
    part_name = f"key_{base_name}" if occurrence == 0 else f"key_{base_name}_{occurrence}"

    x = (left_units + width_units * 0.5 - TOTAL_UNITS_X * 0.5) * U
    y = (row_y_units - ROW_CENTER_OFFSET) * U
    key_width = width_units * U - KEY_GAP
    z = _chassis_top_z(y) + KEY_BASE_CLEARANCE

    if width_units not in mesh_cache:
        mesh_cache[width_units] = _keycap_mesh(width_units, f"low_profile_keycap_{width_units:g}u")

    key = model.part(part_name)
    key.visual(
        mesh_cache[width_units],
        origin=Origin(),
        material=key_material,
        name="keycap",
    )
    # A small center plunger just under the cap makes each separate key read as
    # mechanically mounted without intersecting the deck in the neutral pose.
    key.visual(
        Box((min(0.006, key_width * 0.45), 0.0050, 0.0012)),
        origin=Origin(xyz=(0.0, 0.0, -0.0006)),
        material=key_material,
        name="plunger",
    )
    foot_radius = 0.00075
    key.visual(
        Sphere(foot_radius),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                -KEY_BASE_CLEARANCE + foot_radius * (1.0 + CHASSIS_SLOPE * CHASSIS_SLOPE) ** 0.5,
            )
        ),
        material=key_material,
        name="switch_foot",
    )
    _add_legend(key, _legend_text(label), key_width, legend_material)

    model.articulation(
        f"chassis_to_{part_name}",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=key,
        origin=Origin(xyz=(x, y, z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=0.05, lower=0.0, upper=0.0015),
    )
    return key


def _keyboard_layout() -> list[tuple[str, float, float, float]]:
    """60% compact layout: number row + QWERTY + ASDF + ZXCV + bottom modifiers.

    Dropped vs tenkeyless: function row (ESC/F1–F12/PRINT/SCROLL/PAUSE),
    navigation cluster (INSERT/HOME/PGUP/DELETE/END/PGDN), and arrow keys.
    """
    keys: list[tuple[str, float, float, float]] = []

    # Number row (row_y = 4.65)
    for label, width, left in [
        ("`", 1.0, 0.0),
        ("1", 1.0, 1.0),
        ("2", 1.0, 2.0),
        ("3", 1.0, 3.0),
        ("4", 1.0, 4.0),
        ("5", 1.0, 5.0),
        ("6", 1.0, 6.0),
        ("7", 1.0, 7.0),
        ("8", 1.0, 8.0),
        ("9", 1.0, 9.0),
        ("0", 1.0, 10.0),
        ("-", 1.0, 11.0),
        ("=", 1.0, 12.0),
        ("BACK", 2.0, 13.0),
    ]:
        keys.append((label, width, left, 4.65))

    # QWERTY row (row_y = 3.65)
    for label, width, left in [
        ("TAB", 1.5, 0.0),
        ("Q", 1.0, 1.5),
        ("W", 1.0, 2.5),
        ("E", 1.0, 3.5),
        ("R", 1.0, 4.5),
        ("T", 1.0, 5.5),
        ("Y", 1.0, 6.5),
        ("U", 1.0, 7.5),
        ("I", 1.0, 8.5),
        ("O", 1.0, 9.5),
        ("P", 1.0, 10.5),
        ("[", 1.0, 11.5),
        ("]", 1.0, 12.5),
        ("\\", 1.5, 13.5),
    ]:
        keys.append((label, width, left, 3.65))

    # Home/ASDF row (row_y = 2.65)
    for label, width, left in [
        ("CAPS", 1.75, 0.0),
        ("A", 1.0, 1.75),
        ("S", 1.0, 2.75),
        ("D", 1.0, 3.75),
        ("F", 1.0, 4.75),
        ("G", 1.0, 5.75),
        ("H", 1.0, 6.75),
        ("J", 1.0, 7.75),
        ("K", 1.0, 8.75),
        ("L", 1.0, 9.75),
        (";", 1.0, 10.75),
        ("'", 1.0, 11.75),
        ("ENTER", 2.25, 12.75),
    ]:
        keys.append((label, width, left, 2.65))

    # ZXCV row (row_y = 1.65)
    for label, width, left in [
        ("SHIFT", 2.25, 0.0),
        ("Z", 1.0, 2.25),
        ("X", 1.0, 3.25),
        ("C", 1.0, 4.25),
        ("V", 1.0, 5.25),
        ("B", 1.0, 6.25),
        ("N", 1.0, 7.25),
        ("M", 1.0, 8.25),
        (",", 1.0, 9.25),
        (".", 1.0, 10.25),
        ("/", 1.0, 11.25),
        ("SHIFT", 2.75, 12.25),
    ]:
        keys.append((label, width, left, 1.65))

    # Bottom modifier row (row_y = 0.65)
    for label, width, left in [
        ("CTRL", 1.25, 0.0),
        ("OS", 1.25, 1.25),
        ("ALT", 1.25, 2.50),
        ("SPACE", 6.25, 3.75),
        ("ALT", 1.25, 10.00),
        ("FN", 1.25, 11.25),
        ("MENU", 1.25, 12.50),
        ("CTRL", 1.25, 13.75),
    ]:
        keys.append((label, width, left, 0.65))

    return keys


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="white_60percent_wireless_keyboard")

    chassis_white = model.material("warm_white_plastic", rgba=(0.96, 0.97, 0.965, 1.0))
    key_white = model.material("white_keycap_plastic", rgba=(0.985, 0.987, 0.982, 1.0))
    legend_gray = model.material("light_gray_legends", rgba=(0.52, 0.54, 0.55, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        mesh_from_geometry(_build_chassis_mesh(), "slightly_raised_keyboard_chassis"),
        origin=Origin(),
        material=chassis_white,
        name="tapered_shell",
    )

    rear_y = CHASSIS_DEPTH * 0.5 - 0.005
    rear_z = _chassis_top_z(rear_y) + 0.0032
    chassis.visual(
        Box((CHASSIS_WIDTH - 0.030, 0.006, 0.0064)),
        origin=Origin(xyz=(0.0, rear_y, rear_z)),
        material=chassis_white,
        name="raised_back_edge",
    )

    mesh_cache: dict[float, object] = {}
    label_counts: defaultdict[str, int] = defaultdict(int)
    for label, width, left, row_y in _keyboard_layout():
        _add_key(
            model,
            chassis,
            label=label,
            width_units=width,
            left_units=left,
            row_y_units=row_y,
            mesh_cache=mesh_cache,
            label_counts=label_counts,
            key_material=key_white,
            legend_material=legend_gray,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    chassis = object_model.get_part("chassis")
    key_q = object_model.get_part("key_q")
    key_w = object_model.get_part("key_w")
    key_a = object_model.get_part("key_a")
    key_space = object_model.get_part("key_space")

    # 60% compact: chassis + 61 keys (no function row, no nav cluster, no arrows)
    ctx.check(
        "60_percent compact key count",
        len(object_model.parts) == 62,
        details=f"expected chassis plus 61 keys, got {len(object_model.parts)} parts",
    )

    # Verify the function row and navigation cluster are absent
    part_names = {p.name for p in object_model.parts}
    dropped_keys = {"key_esc", "key_f1", "key_f12", "key_print", "key_insert", "key_delete", "key_left", "key_right", "key_up", "key_down"}
    ctx.check(
        "function row and nav cluster dropped for 60_percent layout",
        not dropped_keys.intersection(part_names),
        details=f"found dropped keys still present: {dropped_keys.intersection(part_names)}",
    )

    chassis_aabb = ctx.part_world_aabb(chassis)
    if chassis_aabb is not None:
        (mn, mx) = chassis_aabb
        ctx.check(
            "60_percent keyboard footprint",
            (mx[0] - mn[0]) < 0.31 and (mx[1] - mn[1]) < 0.13,
            details=f"footprint={(mx[0] - mn[0], mx[1] - mn[1])}",
        )

    q_pos = ctx.part_world_position(key_q)
    w_pos = ctx.part_world_position(key_w)
    a_pos = ctx.part_world_position(key_a)
    ctx.check(
        "QWERTY row ordering",
        q_pos is not None
        and w_pos is not None
        and a_pos is not None
        and q_pos[0] < w_pos[0]
        and a_pos[1] < q_pos[1],
        details=f"q={q_pos}, w={w_pos}, a={a_pos}",
    )

    space_cap_aabb = ctx.part_element_world_aabb(key_space, elem="keycap")
    if space_cap_aabb is not None:
        mn, mx = space_cap_aabb
        ctx.check(
            "low profile keycap height",
            (mx[2] - mn[2]) < 0.006 and mn[2] > 0.006,
            details=f"space keycap z interval={(mn[2], mx[2])}",
        )

    a_joint = object_model.get_articulation("chassis_to_key_a")
    rest_a = ctx.part_world_position(key_a)
    with ctx.pose({a_joint: 0.0015}):
        pressed_a = ctx.part_world_position(key_a)
    ctx.check(
        "individual keys depress downward",
        rest_a is not None and pressed_a is not None and pressed_a[2] < rest_a[2] - 0.0010,
        details=f"rest={rest_a}, pressed={pressed_a}",
    )

    return ctx.report()


object_model = build_object_model()
