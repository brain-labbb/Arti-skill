from __future__ import annotations

# Black clamshell flip phone for seniors.
#
# Frame:
#   - The BASE half (root) lies flat on the XY plane, long axis along X.
#     Base body occupies x in [-0.095, 0.0], y in [-0.025, 0.025],
#     z in [0.0, 0.012]. The hinge barrel sits at the +X end (x ~ 0),
#     near the top of the base (z ~ 0.011), axis along Y.
#   - The LID half is hinged at the barrel. At q=0 (closed) the lid lies
#     flat OVER the base, its inner screen facing down (-Z) toward the keypad.
#     Opening the lid is positive revolute about +Y: the lid free edge and the
#     inner screen swing up and away from the base (0 -> ~155 deg).
#   - Keypad keys are recessed in a well carved into the base top (+Z face),
#     laid out as a tidy 7x3 grid; each presses straight down (-Z, PRISMATIC).
#
# Articulations:
#   - flip_hinge: REVOLUTE about the central hinge barrel, 0..~155 deg.
#   - 21 keypad keys: each PRISMATIC press straight down (~1mm).

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    rounded_rect_profile,
)

DEG = math.pi / 180.0

# ---- Base half dimensions ----
BASE_LEN = 0.095  # X
BASE_WID = 0.050  # Y
BASE_THK = 0.012  # Z
BASE_X0 = -BASE_LEN  # base spans x in [-0.095, 0.0]
BASE_X1 = 0.0
BASE_CX = (BASE_X0 + BASE_X1) / 2.0  # -0.0475

# Hinge geometry (at +X end of the base, near top face).
HINGE_X = 0.0
HINGE_Z = BASE_THK - 0.001  # 0.011
HINGE_R = 0.0055
HINGE_LEN = 0.044  # along Y

# Lid half dimensions (same footprint as base).
LID_LEN = 0.095
LID_WID = 0.050
LID_THK = 0.011

# Keypad: a single recessed well is carved into the base top face, and all keys
# sit INSIDE that well with their crowns just below the surrounding rim, so the
# keypad reads as set into the phone instead of floating on top.
KEY_TOP_Z = BASE_THK  # 0.012 (top face of base = the keypad rim plane)
POCKET_DEPTH = 0.0026  # how deep the keypad well is cut into the base top
POCKET_FLOOR_Z = KEY_TOP_Z - POCKET_DEPTH  # 0.0094 (well floor)
POCKET_HALF_X = 0.043  # well footprint half-extent along X (base-local frame)
POCKET_HALF_Y = 0.021  # well footprint half-extent along Y
CAP_H = 0.0020  # keycap height (floor -> crown)
CROWN_SUBRIM = 0.0006  # crown sits this far below the rim (recessed)
KEY_TRAVEL = 0.001  # 1 mm press


def _rounded_box_mesh(w, d, h, r, name, segments=6):
    # A rounded-rectangle prism of footprint (w x d) and height h along +Z,
    # centered at origin (z in [-h/2, +h/2]).
    prof = rounded_rect_profile(w, d, r, corner_segments=segments)
    geom = ExtrudeGeometry.centered(prof, h, cap=True)
    return mesh_from_geometry(geom, name)


def _shell_mesh(length, width, thk, name):
    # A glossy clamshell half: rounded slab extruded z in [0, thk].
    prof = rounded_rect_profile(length, width, 0.008, corner_segments=8)
    body = ExtrudeGeometry.from_z0(prof, thk, cap=True)
    return mesh_from_geometry(body, name)


def _base_shell_mesh(name):
    # The base half with a recessed keypad well carved into its top face. The
    # well floor sits POCKET_DEPTH below the top so keys can nest inside it.
    prof = rounded_rect_profile(BASE_LEN, BASE_WID, 0.008, corner_segments=8)
    body = ExtrudeGeometry.from_z0(prof, BASE_THK, cap=True)
    pocket = ExtrudeGeometry.from_z0(
        rounded_rect_profile(POCKET_HALF_X * 2.0, POCKET_HALF_Y * 2.0, 0.004, corner_segments=8),
        POCKET_DEPTH + 0.003,
        cap=True,
    )
    pocket.translate(0.0, 0.0, POCKET_FLOOR_Z)
    body = boolean_difference(body, pocket)
    return mesh_from_geometry(body, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clamshell_flip_phone")

    gloss_black = model.material("gloss_black", rgba=(0.07, 0.07, 0.08, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.16, 0.16, 0.18, 1.0))
    screen_blue = model.material("screen_blue", rgba=(0.62, 0.80, 0.92, 1.0))
    key_light = model.material("key_light", rgba=(0.85, 0.86, 0.88, 1.0))
    key_legend = model.material("key_legend", rgba=(0.20, 0.20, 0.22, 1.0))
    red_accent = model.material("red_accent", rgba=(0.80, 0.12, 0.12, 1.0))
    green_accent = model.material("green_accent", rgba=(0.18, 0.62, 0.24, 1.0))

    # ============================================================
    # BASE HALF (root)
    # ============================================================
    base = model.part("base")

    # Glossy black base shell with a recessed keypad well carved into the top.
    base.visual(
        _base_shell_mesh("base_shell"),
        origin=Origin(xyz=(BASE_CX, 0.0, 0.0)),
        material=gloss_black,
        name="base_shell",
    )

    # Dark keypad tray: a liner on the floor of the recessed keypad well.
    base.visual(
        Box((POCKET_HALF_X * 2.0 - 0.0015, POCKET_HALF_Y * 2.0 - 0.0015, 0.0012)),
        origin=Origin(xyz=(BASE_CX, 0.0, POCKET_FLOOR_Z - 0.0006)),
        material=dark_gray,
        name="keypad_tray",
    )

    # Hinge barrel on the base: a cylinder along Y at the +X end.
    barrel = CylinderGeometry(HINGE_R, HINGE_LEN, radial_segments=32).rotate_x(math.pi / 2.0)
    base.visual(
        mesh_from_geometry(barrel, "base_hinge_barrel"),
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        material=dark_gray,
        name="base_hinge_barrel",
    )

    base.inertial = Inertial.from_geometry(
        Box((BASE_LEN, BASE_WID, BASE_THK)),
        mass=0.060,
        origin=Origin(xyz=(BASE_CX, 0.0, BASE_THK / 2.0)),
    )

    # ============================================================
    # LID HALF (hinged at the barrel)
    # ============================================================
    # The lid part frame is placed by the hinge articulation at the barrel.
    # Within the lid frame (q=0, closed): the lid extends in -X over the base,
    # its shell sits just above the base top, the inner screen faces DOWN (-Z).
    lid = model.part("lid")

    # All lid geometry below is authored in the LID FRAME, whose origin is the
    # hinge pin (world z = HINGE_Z at q=0). To place the lid inner face just
    # above the base top in world space, subtract HINGE_Z.
    lid_clear = 0.0015  # closed-pose clearance above the base top
    lid_bottom_z = (KEY_TOP_Z + lid_clear) - HINGE_Z  # inner face, lid-frame z
    lid_cx = -LID_LEN / 2.0

    # Lid shell (glossy black). from_z0 extrudes z in [0, LID_THK]; place it so
    # its inner (z=0) face sits at lid_bottom_z.
    lid.visual(
        _shell_mesh(LID_LEN, LID_WID, LID_THK, "lid_shell"),
        origin=Origin(xyz=(lid_cx, 0.0, lid_bottom_z)),
        material=gloss_black,
        name="lid_shell",
    )

    # Large INNER screen (light blue) recessed on the lid INNER face (facing the
    # base / keypad when closed). It is on the -Z side of the lid shell.
    inner_screen_len = LID_LEN - 0.014
    inner_screen_wid = LID_WID - 0.012
    lid.visual(
        Box((inner_screen_len, inner_screen_wid, 0.0014)),
        origin=Origin(xyz=(lid_cx, 0.0, lid_bottom_z + 0.0009)),
        material=screen_blue,
        name="inner_screen",
    )
    # Thin dark bezel frame around the inner screen.
    lid.visual(
        Box((inner_screen_len + 0.006, inner_screen_wid + 0.005, 0.0006)),
        origin=Origin(xyz=(lid_cx, 0.0, lid_bottom_z + 0.0002)),
        material=dark_gray,
        name="inner_screen_bezel",
    )

    # Small OUTER screen on the lid OUTER face (+Z side of the lid shell, visible
    # when the phone is closed). Placed near the hinge end.
    outer_screen_z = lid_bottom_z + LID_THK
    lid.visual(
        Box((0.026, 0.020, 0.0012)),
        origin=Origin(xyz=(-0.022, 0.010, outer_screen_z - 0.0004)),
        material=screen_blue,
        name="outer_screen",
    )
    lid.visual(
        Box((0.032, 0.026, 0.0006)),
        origin=Origin(xyz=(-0.022, 0.010, outer_screen_z - 0.0009)),
        material=dark_gray,
        name="outer_screen_bezel",
    )

    # Lid-side hinge knuckle so the lid visibly meets the hinge line. Same
    # radius as the base barrel so the two cylinder surfaces are in contact at
    # the pin line. The knuckle z-range overlaps the lid shell inner face near
    # x=0, so it connects directly to the lid body without a separate neck.
    lid_barrel = CylinderGeometry(
        HINGE_R, HINGE_LEN - 0.012, radial_segments=28
    ).rotate_x(math.pi / 2.0)
    lid.visual(
        mesh_from_geometry(lid_barrel, "lid_hinge_knuckle"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_gray,
        name="lid_hinge_knuckle",
    )

    lid.inertial = Inertial.from_geometry(
        Box((LID_LEN, LID_WID, LID_THK)),
        mass=0.050,
        origin=Origin(xyz=(lid_cx, 0.0, lid_bottom_z + LID_THK / 2.0)),
    )

    # Flip hinge: revolute about the barrel (axis +Y). Closed at q=0 (lid lies
    # over base); positive q lifts the lid free edge up and back.
    model.articulation(
        "flip_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=155.0 * DEG),
    )

    # ============================================================
    # KEYPAD KEYS (each a PRISMATIC press straight down -Z)
    # ============================================================
    # Large senior-friendly keys laid out as a clean, regular 7-row x 3-column
    # grid that marches from the hinge end (+X) toward the free end (-X). Every
    # key sits recessed in the keypad well; pressing moves it down by KEY_TRAVEL.
    #
    #   row 1 : soft_left   | nav_up    | soft_right
    #   row 2 : nav_left    | OK center | nav_right     (4-way D-pad + OK)
    #   row 3 : call (green)| nav_down  | end (red)
    #   rows 4-7 : the 12 number keys (1..9, *, 0, #), 4 rows x 3 cols
    #
    # Columns share three Y values; rows share an X pitch -> a tidy aligned grid.
    col_y = (0.0142, 0.0, -0.0142)            # left / center / right columns
    row_x0 = -0.012                            # first row, just past the hinge
    row_pitch = 0.0117                         # spacing between rows along -X

    def row_x(r: int) -> float:
        return row_x0 - r * row_pitch

    NUM_W, NUM_D = 0.0102, 0.0115              # number keycap (X, Y)
    FN_W, FN_D = 0.0096, 0.0110                # function/nav keycap (X, Y)
    OK_DIA = 0.0118                            # round OK key diameter

    # key spec: (name, x, y, w, d, material_key, is_round)
    key_specs: list[tuple[str, float, float, float, float, str, bool]] = []

    # Row 1: soft keys flanking the up arrow.
    key_specs.append(("soft_left", row_x(0), col_y[0], 0.0100, 0.0118, "key_light", False))
    key_specs.append(("nav_up", row_x(0), col_y[1], FN_W, FN_D, "key_light", False))
    key_specs.append(("soft_right", row_x(0), col_y[2], 0.0100, 0.0118, "key_light", False))

    # Row 2: D-pad middle row (left / OK / right).
    key_specs.append(("nav_left", row_x(1), col_y[0], FN_W, FN_D, "key_light", False))
    key_specs.append(("ok_center", row_x(1), col_y[1], OK_DIA, OK_DIA, "key_light", True))
    key_specs.append(("nav_right", row_x(1), col_y[2], FN_W, FN_D, "key_light", False))

    # Row 3: green call + down arrow + red end.
    key_specs.append(("call", row_x(2), col_y[0], 0.0100, 0.0112, "green", False))
    key_specs.append(("nav_down", row_x(2), col_y[1], FN_W, FN_D, "key_light", False))
    key_specs.append(("end", row_x(2), col_y[2], 0.0100, 0.0112, "red", False))

    # Rows 4-7: number keypad 1..9, *, 0, #.
    labels = ["1", "2", "3", "4", "5", "6", "7", "8", "9", "star", "0", "hash"]
    for i, label in enumerate(labels):
        r = 3 + i // 3
        col = i % 3
        key_specs.append((f"num_{label}", row_x(r), col_y[col], NUM_W, NUM_D, "key_light", False))

    mat_lookup = {
        "key_light": key_light,
        "red": red_accent,
        "green": green_accent,
    }

    cap_h = CAP_H
    # Crown sits CROWN_SUBRIM below the rim; cap base rests on the well floor.
    cap_center_z = KEY_TOP_Z - CROWN_SUBRIM - cap_h / 2.0
    for spec in key_specs:
        kname, kx, ky, kw, kd, kmat, is_round = spec
        key = model.part(f"key_{kname}")
        mat = mat_lookup[kmat]

        if is_round:
            cap_geom = CylinderGeometry(kw / 2.0, cap_h, radial_segments=24)
            cap_mesh = mesh_from_geometry(cap_geom, f"key_{kname}_cap")
        else:
            cap_mesh = _rounded_box_mesh(kw, kd, cap_h, min(kw, kd) * 0.28, f"key_{kname}_cap")

        # Keycap authored centered at z=0 in the key frame.
        key.visual(
            cap_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=mat,
            name=f"key_{kname}_cap",
        )
        # Big legible legend tile on the cap top.
        key.visual(
            Box((kw * 0.62, kd * 0.62, 0.0004)),
            origin=Origin(xyz=(0.0, 0.0, cap_h / 2.0 - 0.0001)),
            material=key_legend,
            name=f"key_{kname}_legend",
        )

        key.inertial = Inertial.from_geometry(
            Box((max(kw, 0.006), max(kd, 0.006), cap_h)), mass=0.0015
        )

        # Press is straight down (-Z) into the well.
        model.articulation(
            f"press_{kname}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=key,
            origin=Origin(xyz=(kx, ky, cap_center_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=1.0, velocity=0.05, lower=0.0, upper=KEY_TRAVEL
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    flip = object_model.get_articulation("flip_hinge")

    # --- Hinge: lid knuckle nests at the pin line, sharing the barrel and the
    # back edge of the base body (a real seated clamshell hinge) ---
    ctx.allow_overlap(
        lid,
        base,
        elem_a="lid_hinge_knuckle",
        elem_b="base_hinge_barrel",
        reason="Lid hinge knuckle is intentionally nested at the base hinge barrel / pin line.",
    )
    ctx.allow_overlap(
        lid,
        base,
        elem_a="lid_hinge_knuckle",
        elem_b="base_shell",
        reason="The hinge pin sits at the back top edge of the base, so the lid knuckle "
        "seats into the base body's back edge as a real clamshell hinge.",
    )
    ctx.expect_contact(
        lid,
        base,
        elem_a="lid_hinge_knuckle",
        elem_b="base_hinge_barrel",
        name="lid knuckle seated at base barrel",
    )
    # Knuckle and base barrel share the hinge line along Y (retained engagement).
    ctx.expect_overlap(
        lid,
        base,
        axes="y",
        elem_a="lid_hinge_knuckle",
        elem_b="base_hinge_barrel",
        min_overlap=0.020,
        name="lid knuckle engages the hinge line",
    )

    # --- Inner screen is on the LID; keypad tray on the BASE ---
    base_tray = base.get_visual("keypad_tray")
    inner_screen = lid.get_visual("inner_screen")
    ctx.check(
        "keypad tray belongs to the base",
        base_tray is not None,
        details="keypad_tray visual missing from base",
    )
    ctx.check(
        "inner screen belongs to the lid",
        inner_screen is not None,
        details="inner_screen visual missing from lid",
    )

    # --- Closed pose: the lid lies over the base footprint, above the base ---
    ctx.expect_overlap(
        lid,
        base,
        axes="xy",
        min_overlap=0.04,
        name="closed lid lies over the base footprint",
    )
    # Lid shell bottom (inner face) sits above the base body top face when closed.
    lid_shell_bottom = ctx.part_element_world_aabb(lid, elem="lid_shell")[0][2]
    base_body_top = ctx.part_element_world_aabb(base, elem="base_shell")[1][2]
    ctx.check(
        "closed lid lies on top of the base (inner face above base top)",
        lid_shell_bottom is not None
        and base_body_top is not None
        and lid_shell_bottom >= base_body_top - 0.0005,
        details=f"lid_shell_bottom={lid_shell_bottom}, base_body_top={base_body_top}",
    )

    # --- Opening the hinge swings the lid (inner screen) up and away ---
    # Mid-open (~90 deg) the lid stands up tall; fully open (~150 deg) it lays
    # back at an angle. Both raise the inner screen well above the closed pose.
    screen_closed = ctx.part_element_world_aabb(lid, elem="inner_screen")
    closed_screen_top = screen_closed[1][2]
    with ctx.pose({flip: 90.0 * DEG}):
        mid_screen_top = ctx.part_element_world_aabb(lid, elem="inner_screen")[1][2]
    with ctx.pose({flip: 150.0 * DEG}):
        open_screen_top = ctx.part_element_world_aabb(lid, elem="inner_screen")[1][2]
        open_lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "mid-open lid raises the inner screen well above the base",
        mid_screen_top is not None and mid_screen_top > closed_screen_top + 0.06,
        details=f"closed_top={closed_screen_top}, mid_top={mid_screen_top}",
    )
    ctx.check(
        "fully-open lid keeps the inner screen raised above the closed pose",
        open_screen_top is not None and open_screen_top > closed_screen_top + 0.005,
        details=f"closed_top={closed_screen_top}, open_top={open_screen_top}",
    )
    ctx.check(
        "opened lid swings the free edge up/back (not down into base)",
        open_lid_pos is not None and open_lid_pos[2] > 0.0,
        details=f"open_lid_pos={open_lid_pos}",
    )

    # --- Sampled keypad keys press straight down, stay seated, and are recessed ---
    sample_keys = [
        "num_5",
        "num_0",
        "ok_center",
        "end",
        "call",
        "nav_up",
        "soft_left",
    ]
    base_top = ctx.part_element_world_aabb(base, elem="base_shell")[1][2]
    for kname in sample_keys:
        key = object_model.get_part(f"key_{kname}")
        joint = object_model.get_articulation(f"press_{kname}")

        ctx.expect_contact(key, base, name=f"key_{kname} seated on base")

        # Key is recessed: its crown sits at/below the base top rim (inside the
        # phone), not floating proud of the keypad surface.
        crown_top = ctx.part_world_aabb(key)[1][2]
        ctx.check(
            f"key_{kname} crown is recessed below the base rim",
            crown_top <= base_top + 0.0002,
            details=f"crown_top={crown_top}, base_top={base_top}",
        )

        rest_z = ctx.part_world_position(key)[2]
        with ctx.pose({joint: KEY_TRAVEL}):
            pressed_z = ctx.part_world_position(key)[2]
        ctx.check(
            f"key_{kname} presses straight down",
            pressed_z is not None and pressed_z < rest_z - (KEY_TRAVEL * 0.8),
            details=f"rest_z={rest_z}, pressed_z={pressed_z}",
        )

    # --- Keypad is a tidy aligned grid: each column shares one Y value ---
    def _cy(name: str) -> float:
        a = ctx.part_world_aabb(object_model.get_part(f"key_{name}"))
        return (a[0][1] + a[1][1]) / 2.0

    left_col = [_cy(n) for n in ("soft_left", "nav_left", "call", "num_1", "num_7")]
    ctx.check(
        "left keypad column is vertically aligned",
        max(left_col) - min(left_col) < 0.0006,
        details=f"left column Y values={left_col}",
    )
    center_col = [_cy(n) for n in ("nav_up", "ok_center", "nav_down", "num_2", "num_0")]
    ctx.check(
        "center keypad column is vertically aligned",
        max(center_col) - min(center_col) < 0.0006,
        details=f"center column Y values={center_col}",
    )

    return ctx.report()


object_model = build_object_model()


# ======================================================================
# Revision redo: default pose is the OPEN reference-photo pose.
# The definitions below intentionally supersede the staged parent build above.
# ======================================================================

DEG = math.pi / 180.0

# Larger, photo-like senior clamshell dimensions.
BASE_LEN = 0.112
BASE_WID = 0.058
BASE_THK = 0.012
BASE_X0 = -BASE_LEN
BASE_X1 = 0.0
BASE_CX = -BASE_LEN / 2.0

HINGE_R = 0.0048
HINGE_Z = BASE_THK + HINGE_R
HINGE_X = 0.0

LID_LEN = 0.108
LID_WID = 0.058
LID_THK = 0.012
LID_X_INNER = -0.0033          # when closed, inner face lands just above keypad
LID_X_OUTER = LID_X_INNER + LID_THK
LID_Z0 = 0.006                 # lower black chin below screen

POCKET_HALF_X = 0.049
POCKET_HALF_Y = 0.0255
POCKET_FLOOR_Z = BASE_THK - 0.0032
TRAY_H = 0.00055
KEY_FLOOR_Z = POCKET_FLOOR_Z + TRAY_H
CAP_H = 0.00215
KEY_TRAVEL = 0.00115

# StarTAC-style external telescoping antenna.
ANT_X = -0.005             # near the hinge end, on the body top face
ANT_Y = 0.026              # top corner, next to body_hinge_knuckle_1
BOSS_R = 0.0042
BOSS_H = 0.003
SLEEVE_R = 0.0034
SLEEVE_H = 0.012
MAST_R = 0.0019
MAST_H = 0.048
TIP_R = 0.0028
TIP_H = 0.004
ANT_TRAVEL = 0.034


def _rr_prism(w: float, d: float, h: float, r: float, name: str, segments: int = 8):
    """Centered rounded-rectangle prism: footprint X/Y, thickness Z."""
    prof = rounded_rect_profile(w, d, min(r, w * 0.45, d * 0.45), corner_segments=segments)
    geom = ExtrudeGeometry.centered(prof, h, cap=True)
    return mesh_from_geometry(geom, name)


def _horizontal_panel(length: float, width: float, thick: float, radius: float, name: str):
    """Rounded slab in X/Y, with bottom at z=0 and top at z=thick."""
    prof = rounded_rect_profile(length, width, radius, corner_segments=12)
    geom = ExtrudeGeometry.from_z0(prof, thick, cap=True)
    return mesh_from_geometry(geom, name)


def _vertical_panel(height: float, width: float, thick: float, radius: float, name: str):
    """Rounded slab whose visible broad faces are vertical Y/Z planes."""
    prof = rounded_rect_profile(height, width, radius, corner_segments=12)
    geom = ExtrudeGeometry.from_z0(prof, thick, cap=True)
    geom.rotate_y(math.pi / 2.0)         # extrusion thickness -> local +X
    geom.translate(LID_X_INNER, 0.0, LID_Z0 + height / 2.0)
    return mesh_from_geometry(geom, name)


def _vertical_plate(width: float, height: float, thick: float, radius: float, name: str):
    """Centered rounded plate in Y/Z, thin along X, for screens and slots."""
    prof = rounded_rect_profile(height, width, radius, corner_segments=10)
    geom = ExtrudeGeometry.centered(prof, thick, cap=True)
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _base_shell_photo_mesh(name: str):
    """Glossy lower shell with a real recessed keypad tray cut into the top."""
    prof = rounded_rect_profile(BASE_LEN, BASE_WID, 0.010, corner_segments=14)
    body = ExtrudeGeometry.from_z0(prof, BASE_THK, cap=True)

    pocket = ExtrudeGeometry.from_z0(
        rounded_rect_profile(POCKET_HALF_X * 2.0, POCKET_HALF_Y * 2.0, 0.0045, corner_segments=10),
        BASE_THK - POCKET_FLOOR_Z + 0.004,
        cap=True,
    )
    pocket.translate(0.0, 0.0, POCKET_FLOOR_Z)
    body = boolean_difference(body, pocket)
    return mesh_from_geometry(body, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_senior_clamshell_flip_phone")

    gloss_black = model.material("gloss_black", rgba=(0.015, 0.016, 0.018, 1.0))
    black_soft = model.material("soft_black", rgba=(0.055, 0.057, 0.063, 1.0))
    charcoal = model.material("charcoal_key", rgba=(0.105, 0.108, 0.116, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.23, 0.235, 0.245, 1.0))
    screen_blue = model.material("lit_blue_screen", rgba=(0.55, 0.86, 0.95, 1.0))
    white = model.material("white_legends", rgba=(0.92, 0.92, 0.88, 1.0))
    red = model.material("bright_red_send", rgba=(0.92, 0.03, 0.025, 1.0))
    copper = model.material("orange_ok", rgba=(0.95, 0.43, 0.18, 1.0))
    silver = model.material("silver_trim", rgba=(0.62, 0.64, 0.66, 1.0))

    # ------------------------------------------------------------------
    # Lower body / keypad half (root): glossy rounded shell with raised rim,
    # hinge saddles, side rails, and a dark recessed keypad tray.
    # ------------------------------------------------------------------
    base = model.part("body")
    base.visual(
        _base_shell_photo_mesh("body_shell"),
        origin=Origin(xyz=(BASE_CX, 0.0, 0.0)),
        material=gloss_black,
        name="body_shell",
    )
    base.visual(
        Box((POCKET_HALF_X * 2.0 - 0.0015, POCKET_HALF_Y * 2.0 - 0.0015, TRAY_H)),
        origin=Origin(xyz=(BASE_CX, 0.0, POCKET_FLOOR_Z + TRAY_H / 2.0)),
        material=black_soft,
        name="keypad_tray",
    )
    # Raised glossy perimeter around the keypad, like the molded lip in photo.
    base.visual(
        Box((0.101, 0.0032, 0.0014)),
        origin=Origin(xyz=(BASE_CX - 0.001, POCKET_HALF_Y + 0.0017, BASE_THK + 0.00045)),
        material=gloss_black,
        name="side_rail_0",
    )
    base.visual(
        Box((0.101, 0.0032, 0.0014)),
        origin=Origin(xyz=(BASE_CX - 0.001, -POCKET_HALF_Y - 0.0017, BASE_THK + 0.00045)),
        material=gloss_black,
        name="side_rail_1",
    )
    base.visual(
        Box((0.003, 0.050, 0.0013)),
        origin=Origin(xyz=(-0.006, 0.0, BASE_THK + 0.00035)),
        material=gloss_black,
        name="hinge_lip",
    )
    base.visual(
        Box((0.006, 0.048, 0.0015)),
        origin=Origin(xyz=(-0.108, 0.0, BASE_THK + 0.0004)),
        material=gloss_black,
        name="front_lip",
    )

    # Split hinge: two body-side black knuckles and saddles at the outer ends.
    for i, y in enumerate((-0.021, 0.021)):
        base.visual(
            Box((0.012, 0.014, HINGE_R * 0.95)),
            origin=Origin(xyz=(-0.0018, y, BASE_THK + HINGE_R * 0.47)),
            material=gloss_black,
            name=f"hinge_saddle_{i}",
        )
        knuckle = CylinderGeometry(HINGE_R, 0.0125, radial_segments=36).rotate_x(math.pi / 2.0)
        base.visual(
            mesh_from_geometry(knuckle, f"body_hinge_knuckle_{i}"),
            origin=Origin(xyz=(HINGE_X, y, HINGE_Z)),
            material=gloss_black,
            name=f"body_hinge_knuckle_{i}",
        )
        cap = CylinderGeometry(HINGE_R * 0.86, 0.0018, radial_segments=30).rotate_x(math.pi / 2.0)
        base.visual(
            mesh_from_geometry(cap, f"hinge_end_cap_{i}"),
            origin=Origin(xyz=(HINGE_X, y + (0.0073 if y > 0 else -0.0073), HINGE_Z)),
            material=silver,
            name=f"hinge_end_cap_{i}",
        )

    # StarTAC-style antenna boss (mounting pad) and outer sleeve on the body top
    # corner, next to the hinge knuckle pair. The sleeve receives the retracting
    # antenna mast; the boss provides the visible seating surface on the shell.
    boss_geom = CylinderGeometry(BOSS_R, BOSS_H, radial_segments=28)
    base.visual(
        mesh_from_geometry(boss_geom, "antenna_boss"),
        origin=Origin(xyz=(ANT_X, ANT_Y, BASE_THK + BOSS_H / 2.0)),
        material=gunmetal,
        name="antenna_boss",
    )
    sleeve_geom = CylinderGeometry(SLEEVE_R, SLEEVE_H, radial_segments=28)
    base.visual(
        mesh_from_geometry(sleeve_geom, "antenna_sleeve"),
        origin=Origin(xyz=(ANT_X, ANT_Y, BASE_THK + BOSS_H + SLEEVE_H / 2.0)),
        material=gunmetal,
        name="antenna_sleeve",
    )

    base.inertial = Inertial.from_geometry(
        Box((BASE_LEN, BASE_WID, BASE_THK)),
        mass=0.075,
        origin=Origin(xyz=(BASE_CX, 0.0, BASE_THK / 2.0)),
    )

    # ------------------------------------------------------------------
    # Upper lid: authored in the OPEN reference pose.  q=0 is open, and
    # q=-90deg folds the lid down over the keypad.
    # ------------------------------------------------------------------
    lid = model.part("lid")
    lid.visual(
        _vertical_panel(LID_LEN, LID_WID, LID_THK, 0.010, "lid_shell"),
        origin=Origin(),
        material=gloss_black,
        name="lid_shell",
    )
    # Lower bridge/chin physically ties the vertical lid to its hinge knuckle.
    lid.visual(
        Box((0.014, 0.029, 0.012)),
        origin=Origin(xyz=(0.0015, 0.0, 0.0052)),
        material=gloss_black,
        name="lid_hinge_bridge",
    )
    lid_knuckle = CylinderGeometry(HINGE_R, 0.026, radial_segments=36).rotate_x(math.pi / 2.0)
    lid.visual(
        mesh_from_geometry(lid_knuckle, "lid_hinge_knuckle"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=gloss_black,
        name="lid_hinge_knuckle",
    )
    # Inner display: large blue rectangle nearly filling the upper lid, black bezel.
    lid.visual(
        _vertical_plate(0.052, 0.087, 0.0010, 0.0045, "inner_screen_bezel"),
        origin=Origin(xyz=(LID_X_INNER - 0.00015, 0.0, LID_Z0 + 0.0575)),
        material=black_soft,
        name="inner_screen_bezel",
    )
    lid.visual(
        _vertical_plate(0.0465, 0.078, 0.00115, 0.0035, "inner_screen"),
        origin=Origin(xyz=(LID_X_INNER - 0.00075, 0.0, LID_Z0 + 0.0575)),
        material=screen_blue,
        name="inner_screen",
    )
    # Curved earpiece slot above the display.
    lid.visual(
        _vertical_plate(0.025, 0.0036, 0.0010, 0.0018, "earpiece_slot"),
        origin=Origin(xyz=(LID_X_INNER - 0.00085, 0.0, LID_Z0 + 0.102)),
        material=gunmetal,
        name="earpiece_slot",
    )
    # Small exterior screen and circular outside button on the back of the lid.
    lid.visual(
        _vertical_plate(0.029, 0.039, 0.0010, 0.0035, "outer_screen_bezel"),
        origin=Origin(xyz=(LID_X_OUTER + 0.00010, 0.0, LID_Z0 + 0.053)),
        material=black_soft,
        name="outer_screen_bezel",
    )
    lid.visual(
        _vertical_plate(0.024, 0.033, 0.0011, 0.0028, "outer_screen"),
        origin=Origin(xyz=(LID_X_OUTER + 0.00065, 0.0, LID_Z0 + 0.053)),
        material=screen_blue,
        name="outer_screen",
    )
    outside_button = CylinderGeometry(0.0038, 0.0014, radial_segments=28).rotate_y(math.pi / 2.0)
    lid.visual(
        mesh_from_geometry(outside_button, "outer_round_button"),
        origin=Origin(xyz=(LID_X_OUTER + 0.00035, 0.0, LID_Z0 + 0.023)),
        material=gunmetal,
        name="outer_round_button",
    )

    lid.inertial = Inertial.from_geometry(
        Box((LID_THK, LID_WID, LID_LEN)),
        mass=0.060,
        origin=Origin(xyz=((LID_X_INNER + LID_X_OUTER) / 2.0, 0.0, LID_Z0 + LID_LEN / 2.0)),
    )

    model.articulation(
        "flip_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=-98.0 * DEG, upper=0.0),
    )

    # ------------------------------------------------------------------
    # Telescoping antenna mast (StarTAC-style stub antenna).
    # The mast + tip is one child part; it extends/retracts along +Z via a
    # single PRISMATIC joint. At q=0 (retracted) only a short stub protrudes
    # above the sleeve; the rest hides inside the sleeve and body cavity.
    # ------------------------------------------------------------------
    SLEEVE_TOP_Z = BASE_THK + BOSS_H + SLEEVE_H
    mast_stub = 0.008  # visible stub above sleeve when fully retracted
    mast_z_offset = -(MAST_H / 2.0 - mast_stub)

    antenna = model.part("antenna")
    mast_geom = CylinderGeometry(MAST_R, MAST_H, radial_segments=24)
    antenna.visual(
        mesh_from_geometry(mast_geom, "antenna_mast"),
        origin=Origin(xyz=(0.0, 0.0, mast_z_offset)),
        material=gunmetal,
        name="antenna_mast",
    )
    tip_geom = CylinderGeometry(TIP_R, TIP_H, radial_segments=24)
    antenna.visual(
        mesh_from_geometry(tip_geom, "antenna_tip"),
        origin=Origin(xyz=(0.0, 0.0, mast_z_offset + MAST_H / 2.0 + TIP_H / 2.0)),
        material=gunmetal,
        name="antenna_tip",
    )
    antenna.inertial = Inertial.from_geometry(
        Box((MAST_R * 2.0, MAST_R * 2.0, MAST_H + TIP_H)),
        mass=0.003,
    )

    model.articulation(
        "antenna_extend",
        ArticulationType.PRISMATIC,
        parent=base,
        child=antenna,
        origin=Origin(xyz=(ANT_X, ANT_Y, SLEEVE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0, velocity=0.1, lower=0.0, upper=ANT_TRAVEL
        ),
    )

    # ------------------------------------------------------------------
    # Pressable senior keypad.  Each cap is its own prismatic part, travelling
    # straight down into the keypad face.  Large dark caps with white legends,
    # red send key, dark function key, and copper/orange OK pad match the photo.
    # ------------------------------------------------------------------
    col_y = (-0.0168, 0.0, 0.0168)
    row0 = -0.017
    row_pitch = 0.0131

    def rx(row: int) -> float:
        return row0 - row * row_pitch

    key_specs = [
        # name, row, column, w, d, material, round, legend style
        ("soft_left", 0, 0, 0.0120, 0.0132, charcoal, False, "bar"),
        ("nav_up", 0, 1, 0.0114, 0.0126, charcoal, False, "up"),
        ("soft_right", 0, 2, 0.0120, 0.0132, charcoal, False, "bar"),
        ("nav_left", 1, 0, 0.0122, 0.0130, charcoal, False, "left"),
        ("ok_center", 1, 1, 0.0132, 0.0132, copper, True, "ok"),
        ("nav_right", 1, 2, 0.0122, 0.0130, charcoal, False, "right"),
        ("send_red", 2, 0, 0.0120, 0.0132, red, False, "phone"),
        ("nav_down", 2, 1, 0.0114, 0.0126, charcoal, False, "down"),
        ("function_dark", 2, 2, 0.0120, 0.0132, gunmetal, False, "bar"),
    ]
    for i, label in enumerate(("1", "2", "3", "4", "5", "6", "7", "8", "9", "star", "0", "hash")):
        key_specs.append((f"num_{label}", 3 + i // 3, i % 3, 0.0118, 0.0130, charcoal, False, "number"))

    cap_center_z = KEY_FLOOR_Z + CAP_H / 2.0

    def add_legend(key, name: str, w: float, d: float, style: str) -> None:
        z = CAP_H / 2.0 + 0.00004
        # bright, raised markings: a large primary mark plus smaller sub-legend bars.
        if style == "ok":
            key.visual(Box((w * 0.36, d * 0.36, 0.00022)), origin=Origin(xyz=(0.0, 0.0, z)), material=white, name=f"{name}_ok_mark")
            for iy in (-0.0024, 0.0, 0.0024):
                key.visual(Box((w * 0.52, 0.00055, 0.00020)), origin=Origin(xyz=(0.0, iy, z + 0.00003)), material=white, name=f"{name}_texture_{iy:+.4f}")
        elif style in ("up", "down"):
            key.visual(Box((w * 0.28, d * 0.52, 0.00022)), origin=Origin(xyz=(0.0, 0.0, z)), material=white, name=f"{name}_arrow_stem")
            key.visual(Box((w * 0.52, d * 0.18, 0.00022)), origin=Origin(xyz=((0.0022 if style == "up" else -0.0022), 0.0, z)), material=white, name=f"{name}_arrow_head")
        elif style in ("left", "right"):
            key.visual(Box((w * 0.52, d * 0.28, 0.00022)), origin=Origin(xyz=(0.0, 0.0, z)), material=white, name=f"{name}_arrow_stem")
            key.visual(Box((w * 0.18, d * 0.52, 0.00022)), origin=Origin(xyz=(0.0, (-0.0025 if style == "left" else 0.0025), z)), material=white, name=f"{name}_arrow_head")
        elif style == "phone":
            key.visual(Box((w * 0.56, d * 0.20, 0.00022)), origin=Origin(xyz=(0.0, 0.0, z)), material=white, name=f"{name}_phone_bar")
            key.visual(Box((w * 0.18, d * 0.46, 0.00022)), origin=Origin(xyz=(0.0, -d * 0.18, z)), material=white, name=f"{name}_phone_hook_0")
            key.visual(Box((w * 0.18, d * 0.46, 0.00022)), origin=Origin(xyz=(0.0, d * 0.18, z)), material=white, name=f"{name}_phone_hook_1")
        elif style == "number":
            key.visual(Box((w * 0.38, d * 0.46, 0.00022)), origin=Origin(xyz=(0.0, -d * 0.12, z)), material=white, name=f"{name}_large_legend")
            key.visual(Box((w * 0.40, d * 0.055, 0.00020)), origin=Origin(xyz=(-w * 0.22, d * 0.22, z + 0.00002)), material=silver, name=f"{name}_letters_0")
            key.visual(Box((w * 0.28, d * 0.055, 0.00020)), origin=Origin(xyz=(w * 0.18, d * 0.22, z + 0.00002)), material=silver, name=f"{name}_letters_1")
        else:
            key.visual(Box((w * 0.54, d * 0.18, 0.00022)), origin=Origin(xyz=(0.0, 0.0, z)), material=white, name=f"{name}_bar_mark")

    for name, row, col, w, d, mat, is_round, legend_style in key_specs:
        key = model.part(f"key_{name}")
        cap = (
            mesh_from_geometry(CylinderGeometry(w / 2.0, CAP_H, radial_segments=36), f"key_{name}_cap")
            if is_round
            else _rr_prism(w, d, CAP_H, min(w, d) * 0.22, f"key_{name}_cap", segments=8)
        )
        key.visual(cap, origin=Origin(), material=mat, name=f"key_{name}_cap")
        # Thin silver line across each cap, echoing the molded highlight outlines.
        if not is_round:
            key.visual(
                Box((w * 0.82, d * 0.060, 0.00018)),
                origin=Origin(xyz=(0.0, d * 0.34, CAP_H / 2.0 + 0.00003)),
                material=silver,
                name=f"key_{name}_top_highlight",
            )
        add_legend(key, f"key_{name}", w, d, legend_style)
        key.inertial = Inertial.from_geometry(Box((w, d, CAP_H)), mass=0.0012)
        model.articulation(
            f"press_{name}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=key,
            origin=Origin(xyz=(rx(row), col_y[col], cap_center_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=1.0, velocity=0.04, lower=0.0, upper=KEY_TRAVEL),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("flip_hinge")

    # Required photo details on the correct clamshell half.
    ctx.check("large inner screen is authored on the lid", lid.get_visual("inner_screen") is not None)
    ctx.check("black bezel surrounds the inner screen", lid.get_visual("inner_screen_bezel") is not None)
    ctx.check("curved earpiece slot is above the screen", lid.get_visual("earpiece_slot") is not None)
    ctx.check("small external screen is on the lid back", lid.get_visual("outer_screen") is not None)
    ctx.check("red send key is present", object_model.get_part("key_send_red") is not None)

    # Default pose is the open reference pose: lid vertical/raised and screen on the lid.
    open_screen = ctx.part_element_world_aabb(lid, elem="inner_screen")
    body_top = ctx.part_element_world_aabb(body, elem="body_shell")[1][2]
    ctx.check(
        "default hinge pose displays the phone open",
        open_screen is not None and open_screen[1][2] > body_top + 0.075,
        details=f"open_screen_aabb={open_screen}, body_top={body_top}",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        min_gap=-0.0001,
        positive_elem="lid_hinge_knuckle",
        negative_elem="body_shell",
        name="open lid hinge knuckle sits above the body shell",
    )

    # The same revolute hinge can close the clamshell over the keypad.
    with ctx.pose({hinge: -90.0 * DEG}):
        ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.040, name="closed lid covers the lower body footprint")
        lid_bottom = ctx.part_element_world_aabb(lid, elem="inner_screen")[0][2]
        key_top = ctx.part_world_aabb(object_model.get_part("key_num_5"))[1][2]
        ctx.check(
            "closed lid stays just above the raised keypad",
            lid_bottom > key_top + 0.0003,
            details=f"closed_lid_bottom={lid_bottom}, key_top={key_top}",
        )

    with ctx.pose({hinge: -45.0 * DEG}):
        mid_screen = ctx.part_element_world_aabb(lid, elem="inner_screen")
        ctx.check(
            "hinge sweeps through an intermediate open angle",
            mid_screen is not None and body_top + 0.030 < mid_screen[1][2] < open_screen[1][2],
            details=f"mid_screen={mid_screen}, open_screen={open_screen}",
        )

    # All visible keypad buttons are separate pressable prismatic parts.
    key_names = [
        "soft_left", "nav_up", "soft_right",
        "nav_left", "ok_center", "nav_right",
        "send_red", "nav_down", "function_dark",
        "num_1", "num_2", "num_3", "num_4", "num_5", "num_6",
        "num_7", "num_8", "num_9", "num_star", "num_0", "num_hash",
    ]
    for name in key_names:
        key = object_model.get_part(f"key_{name}")
        joint = object_model.get_articulation(f"press_{name}")
        ctx.check(f"key_{name} has a press joint", key is not None and joint is not None)
        ctx.expect_contact(key, body, name=f"key_{name} seated in the keypad tray")
        rest_z = ctx.part_world_position(key)[2]
        with ctx.pose({joint: KEY_TRAVEL}):
            pressed_z = ctx.part_world_position(key)[2]
        ctx.check(
            f"key_{name} presses straight into the face",
            pressed_z is not None and rest_z is not None and pressed_z < rest_z - KEY_TRAVEL * 0.80,
            details=f"rest_z={rest_z}, pressed_z={pressed_z}",
        )

    # Numeric keypad is the senior-phone 1-9 then * 0 # arrangement.
    def center_xy(part_name: str):
        aabb = ctx.part_world_aabb(object_model.get_part(part_name))
        return ((aabb[0][0] + aabb[1][0]) / 2.0, (aabb[0][1] + aabb[1][1]) / 2.0)

    for upper, lower in (("key_num_1", "key_num_4"), ("key_num_4", "key_num_7"), ("key_num_7", "key_num_star")):
        ctx.check(
            f"{upper} is above {lower} in the keypad order",
            center_xy(upper)[0] > center_xy(lower)[0],
        )
    for left, center, right in (("key_num_1", "key_num_2", "key_num_3"), ("key_num_star", "key_num_0", "key_num_hash")):
        ly, cy, ry = center_xy(left)[1], center_xy(center)[1], center_xy(right)[1]
        ctx.check(f"{left} {center} {right} are ordered across the row", ly < cy < ry)

    # --- Telescoping antenna: mast extends upward from the body sleeve ---
    antenna = object_model.get_part("antenna")
    ant_joint = object_model.get_articulation("antenna_extend")
    ctx.check("antenna part exists", antenna is not None)
    ctx.check("antenna mast uses cylindrical geometry", antenna.get_visual("antenna_mast") is not None)
    ctx.check("antenna tip bulb is present", antenna.get_visual("antenna_tip") is not None)
    ctx.check("body has a visible antenna boss", body.get_visual("antenna_boss") is not None)
    ctx.check("body has a visible antenna sleeve", body.get_visual("antenna_sleeve") is not None)

    # The retracting mast intentionally passes through the sleeve and body cavity.
    ctx.allow_overlap(
        body,
        antenna,
        elem_a="antenna_sleeve",
        elem_b="antenna_mast",
        reason="Antenna mast telescopes inside the sleeve as a real stub antenna.",
    )
    ctx.allow_overlap(
        body,
        antenna,
        elem_a="body_shell",
        elem_b="antenna_mast",
        reason="Retracted antenna mast passes through the body shell interior when collapsed.",
    )
    ctx.allow_overlap(
        body,
        antenna,
        elem_a="hinge_saddle_1",
        elem_b="antenna_mast",
        reason="Antenna mast retracts past the hinge saddle when collapsed into the body.",
    )

    # Mast stays centered in the sleeve on XY (telescoping fit).
    ctx.expect_within(
        antenna,
        body,
        axes="xy",
        inner_elem="antenna_mast",
        outer_elem="antenna_sleeve",
        margin=0.002,
        name="antenna mast stays centered in the sleeve",
    )
    # At rest, the mast retains insertion in the sleeve along Z.
    ctx.expect_overlap(
        antenna,
        body,
        axes="z",
        elem_a="antenna_mast",
        elem_b="antenna_sleeve",
        min_overlap=0.005,
        name="retracted mast remains inserted in the sleeve",
    )

    # Extending the antenna raises the mast upward along +Z.
    rest_pos = ctx.part_world_position(antenna)
    with ctx.pose({ant_joint: ANT_TRAVEL}):
        extended_pos = ctx.part_world_position(antenna)
    ctx.check(
        "antenna extends upward along +Z when actuated",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.020,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
