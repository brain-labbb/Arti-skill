from __future__ import annotations

import math
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
TOTAL_UNITS_X = 18.75
ROW_CENTER_OFFSET = 3.25
CHASSIS_WIDTH = 0.365
CHASSIS_DEPTH = 0.132
CHASSIS_TOP_INSET = 0.006
CHASSIS_FRONT_TOP_Z = 0.0065
CHASSIS_REAR_TOP_Z = 0.0145
KEY_GAP = 0.0020
KEY_DEPTH = 0.0162
KEY_HEIGHT = 0.0044
KEY_BASE_CLEARANCE = 0.0015

# Tented split chassis parameters
TENT_ANGLE_DEG = 6.0
TENT_ANGLE = math.radians(TENT_ANGLE_DEG)

# Each half uses the full half-width of the chassis; rotation creates a
# narrow natural gap (~2 mm) at the bottom between the two tented platforms.
HALF_WIDTH = CHASSIS_WIDTH / 2.0  # 0.1825

# The split for key assignment is at x = 0 (physical center of keyboard).
SPLIT_WORLD_X = 0.0

# Derived constants
LEFT_OUTER_X = -HALF_WIDTH
RIGHT_OUTER_X = HALF_WIDTH
TENT_SLOPE = math.tan(TENT_ANGLE)
COS_T = math.cos(TENT_ANGLE)
SIN_T = math.sin(TENT_ANGLE)
TAN_T = math.tan(TENT_ANGLE)
INV_COS_T = 1.0 / COS_T


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


def _tented_surface_z(x: float, y: float) -> float:
    """Compute the exact surface Z of the tented chassis at world position (x, y).

    Uses the exact rotation formula so the key placement matches the mesh surface:
        z = dist_from_outer * tan(TENT) + slope_z / cos(TENT)
    """
    front_y = -CHASSIS_DEPTH * 0.5 + CHASSIS_TOP_INSET
    rear_y = CHASSIS_DEPTH * 0.5 - CHASSIS_TOP_INSET
    t_y = (y - front_y) / (rear_y - front_y)
    t_y = max(0.0, min(1.0, t_y))
    slope_z = CHASSIS_FRONT_TOP_Z + (CHASSIS_REAR_TOP_Z - CHASSIS_FRONT_TOP_Z) * t_y

    if x <= 0.0:
        dist = max(0.0, x - LEFT_OUTER_X)
    else:
        dist = max(0.0, RIGHT_OUTER_X - x)
    return dist * TAN_T + slope_z * INV_COS_T


def _build_chassis_mesh() -> MeshGeometry:
    """Tented split chassis: two symmetric halves tilted outward with central bridge."""
    geom = MeshGeometry()

    hd = CHASSIS_DEPTH * 0.5
    inset = CHASSIS_TOP_INSET
    fZ = CHASSIS_FRONT_TOP_Z
    rZ = CHASSIS_REAR_TOP_Z

    # Build one half in local frame (pivot at outer edge x=0, extending +x by HALF_WIDTH).
    # Inner edge has NO x-inset so the top surface extends to the same x as the bottom.
    local_verts = [
        (0.0, -hd, 0.0),                       # 0: outer front bottom
        (HALF_WIDTH, -hd, 0.0),                # 1: inner front bottom
        (HALF_WIDTH, hd, 0.0),                 # 2: inner rear bottom
        (0.0, hd, 0.0),                        # 3: outer rear bottom
        (inset, -hd + inset, fZ),              # 4: outer front top (outer inset)
        (HALF_WIDTH, -hd + inset, fZ),         # 5: inner front top (no x inset)
        (HALF_WIDTH, hd - inset, rZ),          # 6: inner rear top (no x inset)
        (inset, hd - inset, rZ),               # 7: outer rear top (outer inset)
    ]

    # Rotate left half (inner edge raised): rotation by -TENT_ANGLE around outer edge.
    # x_world = x_local * cos(T) - z_local * sin(T) + LEFT_OUTER_X
    # z_world = x_local * sin(T) + z_local * cos(T)
    left_world = []
    for x, y, z in local_verts:
        xw = x * COS_T - z * SIN_T + LEFT_OUTER_X
        zw = x * SIN_T + z * COS_T
        left_world.append((xw, y, zw))

    # Right half: mirror by negating x
    right_world = [(-x, y, z) for x, y, z in left_world]

    lv = [geom.add_vertex(*v) for v in left_world]
    rv = [geom.add_vertex(*v) for v in right_world]

    # Left half faces
    _add_quad(geom, lv[0], lv[1], lv[2], lv[3])  # bottom
    _add_quad(geom, lv[4], lv[7], lv[6], lv[5])  # top
    _add_quad(geom, lv[0], lv[4], lv[5], lv[1])  # front
    _add_quad(geom, lv[1], lv[5], lv[6], lv[2])  # inner wall
    _add_quad(geom, lv[2], lv[6], lv[7], lv[3])  # rear
    _add_quad(geom, lv[3], lv[7], lv[4], lv[0])  # outer wall

    # Right half faces (mirrored winding)
    _add_quad(geom, rv[3], rv[2], rv[1], rv[0])  # bottom
    _add_quad(geom, rv[5], rv[6], rv[7], rv[4])  # top
    _add_quad(geom, rv[1], rv[5], rv[4], rv[0])  # front
    _add_quad(geom, rv[2], rv[6], rv[5], rv[1])  # inner wall
    _add_quad(geom, rv[3], rv[7], rv[6], rv[2])  # rear
    _add_quad(geom, rv[0], rv[4], rv[7], rv[3])  # outer wall

    # Connecting bridge at the bottom of the central gap.
    # After rotation the inner bottom edges are raised; connect them to z=0.
    x_li = left_world[1][0]   # left inner bottom x
    x_ri = right_world[1][0]  # right inner bottom x

    sk0 = geom.add_vertex(x_li, -hd, 0.0)
    sk1 = geom.add_vertex(x_ri, -hd, 0.0)
    sk2 = geom.add_vertex(x_ri, hd, 0.0)
    sk3 = geom.add_vertex(x_li, hd, 0.0)

    _add_quad(geom, sk0, sk1, sk2, sk3)  # bridge floor
    _add_quad(geom, sk0, sk1, rv[1], lv[1])  # front bridge wall
    _add_quad(geom, lv[2], rv[2], sk2, sk3)  # rear bridge wall
    _add_quad(geom, sk0, lv[1], lv[2], sk3)  # left bridge wall
    _add_quad(geom, rv[1], sk1, sk2, rv[2])  # right bridge wall

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


def _compute_key_axis(x: float) -> tuple[float, float, float]:
    """Compute the PRISMATIC axis for a key (perpendicular to tented surface, pressing inward+down)."""
    if x <= 0.0:
        axis_x = SIN_T
    else:
        axis_x = -SIN_T
    axis_z = -COS_T
    length = math.sqrt(axis_x * axis_x + axis_z * axis_z)
    return (axis_x / length, 0.0, axis_z / length)


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

    z = _tented_surface_z(x, y) + KEY_BASE_CLEARANCE
    axis = _compute_key_axis(x)

    if width_units not in mesh_cache:
        mesh_cache[width_units] = _keycap_mesh(width_units, f"low_profile_keycap_{width_units:g}u")

    key = model.part(part_name)
    key.visual(
        mesh_cache[width_units],
        origin=Origin(),
        material=key_material,
        name="keycap",
    )
    key.visual(
        Box((min(0.006, key_width * 0.45), 0.0050, 0.0012)),
        origin=Origin(xyz=(0.0, 0.0, -0.0006)),
        material=key_material,
        name="plunger",
    )
    # Stem that extends from key bottom through the clearance to contact the surface.
    stem_height = KEY_BASE_CLEARANCE + 0.0002
    key.visual(
        Box((0.003, 0.003, stem_height)),
        origin=Origin(xyz=(0.0, 0.0, -stem_height * 0.5)),
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
        axis=axis,
        motion_limits=MotionLimits(effort=1.2, velocity=0.05, lower=0.0, upper=0.0015),
    )
    return key


def _keyboard_layout() -> list[tuple[str, float, float, float]]:
    keys: list[tuple[str, float, float, float]] = []

    for label, left in [
        ("ESC", 0.0),
        ("F1", 2.0),
        ("F2", 3.0),
        ("F3", 4.0),
        ("F4", 5.0),
        ("F5", 6.25),
        ("F6", 7.25),
        ("F7", 8.25),
        ("F8", 9.25),
        ("F9", 10.50),
        ("F10", 11.50),
        ("F11", 12.50),
        ("F12", 13.50),
        ("PRINT", 15.75),
        ("SCROLL", 16.75),
        ("PAUSE", 17.75),
    ]:
        keys.append((label, 1.0, left, 5.85))

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
        ("INSERT", 1.0, 15.75),
        ("HOME", 1.0, 16.75),
        ("PGUP", 1.0, 17.75),
    ]:
        keys.append((label, width, left, 4.65))

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
        ("DELETE", 1.0, 15.75),
        ("END", 1.0, 16.75),
        ("PGDN", 1.0, 17.75),
    ]:
        keys.append((label, width, left, 3.65))

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
        ("UP", 1.0, 16.75),
    ]:
        keys.append((label, width, left, 1.65))

    for label, width, left in [
        ("CTRL", 1.25, 0.0),
        ("OS", 1.25, 1.25),
        ("ALT", 1.25, 2.50),
        ("SPACE", 6.25, 3.75),
        ("ALT", 1.25, 10.00),
        ("FN", 1.25, 11.25),
        ("MENU", 1.25, 12.50),
        ("CTRL", 1.25, 13.75),
        ("LEFT", 1.0, 15.75),
        ("DOWN", 1.0, 16.75),
        ("RIGHT", 1.0, 17.75),
    ]:
        keys.append((label, width, left, 0.65))

    return keys


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="white_tenkeyless_wireless_keyboard_tented")

    chassis_white = model.material("warm_white_plastic", rgba=(0.96, 0.97, 0.965, 1.0))
    key_white = model.material("white_keycap_plastic", rgba=(0.985, 0.987, 0.982, 1.0))
    legend_gray = model.material("light_gray_legends", rgba=(0.52, 0.54, 0.55, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        mesh_from_geometry(_build_chassis_mesh(), "tented_split_keyboard_chassis"),
        origin=Origin(),
        material=chassis_white,
        name="tapered_shell",
    )

    # Raised back edges for each half, positioned on the tented surfaces
    rear_y = CHASSIS_DEPTH * 0.5 - 0.005
    half_center_x = HALF_WIDTH * 0.5

    left_rear_z = _tented_surface_z(-half_center_x, rear_y) + 0.0032
    chassis.visual(
        Box((HALF_WIDTH - 0.030, 0.006, 0.0064)),
        origin=Origin(xyz=(-half_center_x, rear_y, left_rear_z), rpy=(0.0, TENT_ANGLE, 0.0)),
        material=chassis_white,
        name="raised_back_edge_left",
    )

    right_rear_z = _tented_surface_z(half_center_x, rear_y) + 0.0032
    chassis.visual(
        Box((HALF_WIDTH - 0.030, 0.006, 0.0064)),
        origin=Origin(xyz=(half_center_x, rear_y, right_rear_z), rpy=(0.0, -TENT_ANGLE, 0.0)),
        material=chassis_white,
        name="raised_back_edge_right",
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

    ctx.check(
        "tenkeyless compact scale",
        len(object_model.parts) == 88,
        details=f"expected chassis plus 87 keys, got {len(object_model.parts)} parts",
    )
    chassis_aabb = ctx.part_world_aabb(chassis)
    if chassis_aabb is not None:
        (mn, mx) = chassis_aabb
        ctx.check(
            "compact keyboard footprint without numeric pad",
            (mx[0] - mn[0]) < 0.39 and (mx[1] - mn[1]) < 0.15,
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

    # Tented split chassis verification
    key_g = object_model.get_part("key_g")
    key_p = object_model.get_part("key_p")
    key_9 = object_model.get_part("key_9")  # key 9 is on right half near center
    g_pos = ctx.part_world_position(key_g)
    p_pos = ctx.part_world_position(key_p)
    nine_pos = ctx.part_world_position(key_9)

    ctx.check(
        "tented split: left inner edge higher than outer",
        g_pos is not None and q_pos is not None and g_pos[2] > q_pos[2] + 0.005,
        details=f"G(inner left) z={g_pos[2] if g_pos else None}, Q(outer left) z={q_pos[2] if q_pos else None}",
    )
    ctx.check(
        "tented split: right inner edge higher than outer",
        nine_pos is not None and p_pos is not None and nine_pos[2] > p_pos[2] + 0.003,
        details=f"9(inner right) z={nine_pos[2] if nine_pos else None}, P(outer right) z={p_pos[2] if p_pos else None}",
    )

    return ctx.report()


object_model = build_object_model()
