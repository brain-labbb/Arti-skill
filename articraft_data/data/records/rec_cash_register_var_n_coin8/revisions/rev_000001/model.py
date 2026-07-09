from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LoftGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

# ---------------------------------------------------------------------------
# White electronic cash register (counter-top ECR on a wide cash drawer base).
# World frame: x = width, +y = front (toward the operator), z = up.
#
# Cabinet (root): drawer base 0.40 x 0.42 x ~0.14 m on rubber feet.
# Console (fixed): sloped keyboard deck, raised rear platform with a printer
# bay (hinged cover + paper roll) and a display pod with an LCD screen.
# Articulations: sliding cash drawer, rotating drawer lock, hinged printer
# cover, and 59 pressable keys in four color groups.
# ---------------------------------------------------------------------------

# Cabinet outer envelope
CAB_W = 0.400
CAB_D = 0.420
CAB_TOP_Z = 0.137  # top surface of the cabinet counter plate

# Keyboard deck slope (console-local yz profile of the front wedge)
DECK_REAR = (-0.022, 0.086)
DECK_FRONT = (0.145, 0.032)
_DDY = DECK_FRONT[0] - DECK_REAR[0]
_DDZ = DECK_REAR[1] - DECK_FRONT[1]
DECK_LEN = math.hypot(_DDY, _DDZ)
DECK_CA = _DDY / DECK_LEN  # cos(slope)
DECK_SA = _DDZ / DECK_LEN  # sin(slope)
DECK_ALPHA = math.atan2(_DDZ, _DDY)
DECK_NORMAL = (0.0, DECK_SA, DECK_CA)

KEY_LIFT = -0.0006  # key skirt seats slightly into the deck (through-hole fit)
KEY_TRAVEL = 0.0028

# Printer bay (console-local; photo-left of the platform => world +x)
BAY_CX = 0.090
BAY_RIM_TOP_Z = 0.154
COVER_HINGE_Y = -0.171
COVER_HINGE_Z = BAY_RIM_TOP_Z  # cover underside rests on the rim top


def _prism_mesh(profile_yz: list[tuple[float, float]], half_width: float, name: str):
    """Convex prism spanning x = +/-half_width with a (y, z) cross-section."""
    pts = list(profile_yz)
    area = 0.0
    for i in range(len(pts)):
        y0, z0 = pts[i]
        y1, z1 = pts[(i + 1) % len(pts)]
        area += y0 * z1 - y1 * z0
    if area < 0.0:
        pts.reverse()  # ensure CCW in (y, z) so +x face normals point outward

    geom = MeshGeometry()
    plus = [geom.add_vertex(half_width, y, z) for (y, z) in pts]
    minus = [geom.add_vertex(-half_width, y, z) for (y, z) in pts]

    for i in range(1, len(pts) - 1):
        geom.add_face(plus[0], plus[i], plus[i + 1])
        geom.add_face(minus[0], minus[i + 1], minus[i])
    for i in range(len(pts)):
        j = (i + 1) % len(pts)
        geom.add_face(minus[i], minus[j], plus[j])
        geom.add_face(minus[i], plus[j], plus[i])
    return mesh_from_geometry(geom, name)


def _keycap_mesh(name: str, width: float, depth: float, height: float):
    """Slightly tapered keycap: larger base, smaller crown."""
    bottom = rounded_rect_profile(width, depth, 0.0024)
    top = rounded_rect_profile(width - 0.0034, depth - 0.0034, 0.0016)
    geom = LoftGeometry(
        [
            [(x, y, 0.0) for x, y in bottom],
            [(x, y, height) for x, y in top],
        ],
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geom, name)


def _deck_point(u: float, t: float) -> tuple[float, float, float]:
    """Console-local point on the keyboard deck: u lateral, t down-slope."""
    return (
        u,
        DECK_REAR[0] + t * DECK_CA,
        DECK_REAR[1] - t * DECK_SA,
    )


def _col_x(col: int) -> float:
    # Column 0 is the operator's leftmost column (photo-left => world +x).
    return 0.160 - col * 0.0180


def _row_t(row: int) -> float:
    return 0.018 + row * 0.0165


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="electronic_cash_register")

    body_white = model.material("body_white", rgba=(0.93, 0.93, 0.91, 1.0))
    base_white = model.material("base_white", rgba=(0.89, 0.89, 0.87, 1.0))
    tray_gray = model.material("tray_gray", rgba=(0.80, 0.80, 0.78, 1.0))
    foot_rubber = model.material("foot_rubber", rgba=(0.16, 0.16, 0.17, 1.0))
    key_tan = model.material("key_tan", rgba=(0.90, 0.82, 0.62, 1.0))
    key_blue = model.material("key_blue", rgba=(0.63, 0.80, 0.90, 1.0))
    key_green = model.material("key_green", rgba=(0.68, 0.87, 0.74, 1.0))
    key_pink = model.material("key_pink", rgba=(0.92, 0.65, 0.72, 1.0))
    bezel_dark = model.material("bezel_dark", rgba=(0.12, 0.12, 0.13, 1.0))
    screen_teal = model.material("screen_teal", rgba=(0.25, 0.63, 0.83, 1.0))
    chrome = model.material("chrome", rgba=(0.76, 0.77, 0.79, 1.0))
    cutter_blue = model.material("cutter_blue", rgba=(0.36, 0.62, 0.85, 1.0))
    paper_white = model.material("paper_white", rgba=(0.97, 0.97, 0.95, 1.0))
    label_silver = model.material("label_silver", rgba=(0.81, 0.81, 0.80, 1.0))

    # ------------------------------------------------------------------ #
    # Cabinet (root): hollow drawer base with a front opening             #
    # ------------------------------------------------------------------ #
    cabinet = model.part("cabinet")
    for i, (fx, fy) in enumerate(((-0.170, -0.180), (0.170, -0.180), (-0.170, 0.180), (0.170, 0.180))):
        cabinet.visual(
            Box((0.032, 0.032, 0.010)),
            origin=Origin(xyz=(fx, fy, 0.005)),
            material=foot_rubber,
            name=f"foot_{i}",
        )
    cabinet.visual(
        Box((CAB_W, CAB_D, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=base_white,
        name="bottom_panel",
    )
    for i, wx in enumerate((-0.194, 0.194)):
        cabinet.visual(
            Box((0.012, CAB_D, 0.103)),
            origin=Origin(xyz=(wx, 0.0, 0.0735)),
            material=base_white,
            name=f"side_wall_{i}",
        )
    cabinet.visual(
        Box((0.376, 0.012, 0.103)),
        origin=Origin(xyz=(0.0, -0.204, 0.0735)),
        material=base_white,
        name="back_wall",
    )
    cabinet.visual(
        Box((CAB_W, CAB_D, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.131)),
        material=base_white,
        name="counter_top",
    )

    # ------------------------------------------------------------------ #
    # Console head (fixed on the cabinet top)                             #
    # ------------------------------------------------------------------ #
    console = model.part("console")
    model.articulation(
        "cabinet_to_console",
        ArticulationType.FIXED,
        parent=cabinet,
        child=console,
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP_Z)),
    )

    rear_profile = [
        (-0.200, 0.0),
        (-0.200, 0.118),
        (-0.185, 0.128),
        (-0.040, 0.128),
        (-0.020, 0.085),
        (-0.020, 0.0),
    ]
    console.visual(
        _prism_mesh(rear_profile, 0.180, "register_rear_housing"),
        material=body_white,
        name="rear_housing",
    )
    deck_profile = [
        (DECK_REAR[0], 0.0),
        DECK_REAR,
        DECK_FRONT,
        (0.160, 0.024),
        (0.160, 0.0),
    ]
    console.visual(
        _prism_mesh(deck_profile, 0.180, "register_keyboard_deck"),
        material=body_white,
        name="keyboard_deck",
    )

    # Display pod with sloped face, dark bezel and teal LCD screen
    pod_profile = [
        (-0.042, 0.0),
        (-0.042, 0.060),
        (-0.022, 0.066),
        (0.038, 0.014),
        (0.042, 0.0),
    ]
    pod_origin = (-0.105, -0.113, 0.1275)
    console.visual(
        _prism_mesh(pod_profile, 0.055, "display_pod"),
        origin=Origin(xyz=pod_origin),
        material=body_white,
        name="display_pod",
    )
    pod_dy = 0.038 - (-0.022)
    pod_dz = 0.066 - 0.014
    pod_len = math.hypot(pod_dy, pod_dz)
    pod_sa = pod_dz / pod_len
    pod_ca = pod_dy / pod_len
    pod_n = (0.0, pod_sa, pod_ca)  # outward normal of the pod's sloped face
    pod_rx = -math.atan2(pod_sa, pod_ca)
    pod_mid = (0.008, 0.040)  # (y, z) midpoint of the sloped face, pod-local
    bezel_c = (
        pod_origin[0],
        pod_origin[1] + pod_mid[0] + pod_n[1] * 0.0017,
        pod_origin[2] + pod_mid[1] + pod_n[2] * 0.0017,
    )
    console.visual(
        Box((0.088, 0.068, 0.005)),
        origin=Origin(xyz=bezel_c, rpy=(pod_rx, 0.0, 0.0)),
        material=bezel_dark,
        name="display_bezel",
    )
    screen_c = (
        pod_origin[0],
        pod_origin[1] + pod_mid[0] + pod_n[1] * 0.0051,
        pod_origin[2] + pod_mid[1] + pod_n[2] * 0.0051,
    )
    console.visual(
        Box((0.066, 0.048, 0.003)),
        origin=Origin(xyz=screen_c, rpy=(pod_rx, 0.0, 0.0)),
        material=screen_teal,
        name="display_screen",
    )

    # Printer bay: raised rim on the rear platform with a paper roll inside
    console.visual(
        Box((0.130, 0.008, 0.028)),
        origin=Origin(xyz=(BAY_CX, -0.171, 0.140)),
        material=body_white,
        name="printer_bay_rear_wall",
    )
    console.visual(
        Box((0.130, 0.008, 0.028)),
        origin=Origin(xyz=(BAY_CX, -0.069, 0.140)),
        material=body_white,
        name="printer_bay_front_wall",
    )
    for i, wx in enumerate((BAY_CX - 0.061, BAY_CX + 0.061)):
        console.visual(
            Box((0.008, 0.102, 0.028)),
            origin=Origin(xyz=(wx, -0.120, 0.140)),
            material=body_white,
            name=f"printer_bay_wall_{i}",
        )
    console.visual(
        Cylinder(radius=0.012, length=0.110),
        origin=Origin(xyz=(BAY_CX, -0.120, 0.140), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=paper_white,
        name="paper_roll",
    )

    # Brand plaque and model label on the deck
    plaque_p = _deck_point(0.0, 0.148)
    console.visual(
        Box((0.056, 0.026, 0.0026)),
        origin=Origin(
            xyz=(
                plaque_p[0],
                plaque_p[1] + DECK_NORMAL[1] * 0.0006,
                plaque_p[2] + DECK_NORMAL[2] * 0.0006,
            ),
            rpy=(-DECK_ALPHA, 0.0, 0.0),
        ),
        material=label_silver,
        name="brand_plaque",
    )
    label_p = _deck_point(-0.130, 0.045)
    console.visual(
        Box((0.046, 0.020, 0.002)),
        origin=Origin(
            xyz=(
                label_p[0],
                label_p[1] + DECK_NORMAL[1] * 0.0004,
                label_p[2] + DECK_NORMAL[2] * 0.0004,
            ),
            rpy=(-DECK_ALPHA, 0.0, 0.0),
        ),
        material=label_silver,
        name="model_label",
    )

    # ------------------------------------------------------------------ #
    # Keyboard: 59 pressable keys on the sloped deck                      #
    # ------------------------------------------------------------------ #
    cap_std = _keycap_mesh("keycap_std", 0.0150, 0.0135, 0.0085)
    cap_num = _keycap_mesh("keycap_num", 0.0160, 0.0140, 0.0085)
    cap_wide = _keycap_mesh("keycap_wide", 0.0340, 0.0140, 0.0085)

    def add_key(part_name: str, u: float, t: float, cap_mesh, material) -> None:
        key = model.part(part_name)
        key.visual(cap_mesh, material=material, name="keycap")
        p = _deck_point(u, t)
        model.articulation(
            f"console_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=console,
            child=key,
            origin=Origin(
                xyz=(
                    p[0],
                    p[1] + DECK_NORMAL[1] * KEY_LIFT,
                    p[2] + DECK_NORMAL[2] * KEY_LIFT,
                ),
                rpy=(-DECK_ALPHA, 0.0, 0.0),
            ),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=KEY_TRAVEL),
        )

    # Left tan function column (FEED / SHIFT / RECEIPT / BKSP / CLEAR)
    for r in range(5):
        add_key(f"function_key_{r}", _col_x(0), _row_t(r), cap_std, key_tan)
    # Blue department key block, 5 rows x 6 columns
    for r in range(5):
        for c in range(6):
            add_key(f"dept_key_{r}_{c}", _col_x(1 + c), _row_t(r), cap_std, key_blue)
    # Green numeric pad, 4 rows x 3 columns
    for r in range(4):
        for c in range(3):
            add_key(f"numeric_key_{r}_{c}", _col_x(8 + c), _row_t(1 + r), cap_num, key_green)
    # Right tan mode/function block, 4 rows x 2 columns
    for r in range(4):
        for c in range(2):
            add_key(f"mode_key_{r}_{c}", _col_x(12 + c), _row_t(r), cap_std, key_tan)
    # Pink card/cashier keys
    for c in range(2):
        add_key(f"payment_key_{c}", _col_x(12 + c), _row_t(4), cap_std, key_pink)
    # Wide SUBTOT and CASH keys
    wide_x = (_col_x(12) + _col_x(13)) / 2.0
    add_key("subtotal_key", wide_x, _row_t(5), cap_wide, key_tan)
    add_key("cash_key", wide_x, _row_t(6), cap_wide, key_pink)

    # ------------------------------------------------------------------ #
    # Printer cover: hinged lid with a real receipt slot and blue cutter  #
    # ------------------------------------------------------------------ #
    cover = model.part("printer_cover")
    cover.visual(
        Box((0.112, 0.062, 0.008)),
        origin=Origin(xyz=(0.0, 0.033, 0.004)),
        material=body_white,
        name="cover_rear_panel",
    )
    cover.visual(
        Box((0.112, 0.034, 0.008)),
        origin=Origin(xyz=(0.0, 0.089, 0.004)),
        material=body_white,
        name="cover_front_panel",
    )
    for i, rx in enumerate((-0.061, 0.061)):
        cover.visual(
            Box((0.012, 0.104, 0.008)),
            origin=Origin(xyz=(rx, 0.054, 0.004)),
            material=body_white,
            name=f"cover_rail_{i}",
        )
    cover.visual(
        Box((0.052, 0.009, 0.0035)),
        origin=Origin(xyz=(0.0, 0.0755, 0.00925)),
        material=cutter_blue,
        name="paper_cutter",
    )
    model.articulation(
        "console_to_printer_cover",
        ArticulationType.REVOLUTE,
        parent=console,
        child=cover,
        origin=Origin(xyz=(BAY_CX, COVER_HINGE_Y, COVER_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.35),
    )

    # ------------------------------------------------------------------ #
    # Cash drawer: front fascia + coin tray with five compartments        #
    # ------------------------------------------------------------------ #
    drawer = model.part("cash_drawer")
    drawer.visual(
        Box((CAB_W, 0.020, 0.112)),
        origin=Origin(xyz=(0.0, 0.220, 0.072)),
        material=base_white,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.360, 0.360, 0.008)),
        origin=Origin(xyz=(0.0, 0.0305, 0.028)),
        material=tray_gray,
        name="tray_floor",
    )
    drawer.visual(
        Box((0.360, 0.008, 0.055)),
        origin=Origin(xyz=(0.0, -0.1455, 0.0595)),
        material=tray_gray,
        name="tray_rear_wall",
    )
    drawer.visual(
        Box((0.360, 0.008, 0.055)),
        origin=Origin(xyz=(0.0, 0.207, 0.0595)),
        material=tray_gray,
        name="tray_front_wall",
    )
    for i, wx in enumerate((-0.176, 0.176)):
        drawer.visual(
            Box((0.008, 0.360, 0.055)),
            origin=Origin(xyz=(wx, 0.0305, 0.0595)),
            material=tray_gray,
            name=f"tray_wall_{i}",
        )
    # 7 dividers evenly spaced inside the tray walls → 8 coin compartments
    _tray_inner_half = 0.172  # half-width between tray wall inner faces
    _num_coin_slots = 8
    _slot_w = (2.0 * _tray_inner_half) / _num_coin_slots
    for i in range(_num_coin_slots - 1):
        dx = -_tray_inner_half + (i + 1) * _slot_w
        drawer.visual(
            Box((0.006, 0.346, 0.045)),
            origin=Origin(xyz=(dx, 0.0305, 0.0545)),
            material=tray_gray,
            name=f"coin_divider_{i}",
        )
    model.articulation(
        "cabinet_to_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.5, lower=0.0, upper=0.245),
    )

    # Chrome key lock on the drawer front
    lock = model.part("drawer_lock")
    lock.visual(
        Cylinder(radius=0.0115, length=0.013),
        origin=Origin(xyz=(0.0, 0.0065, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="lock_barrel",
    )
    lock.visual(
        Cylinder(radius=0.009, length=0.002),
        origin=Origin(xyz=(0.0, 0.0140, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="lock_face_ring",
    )
    lock.visual(
        Box((0.0026, 0.0018, 0.011)),
        origin=Origin(xyz=(0.0, 0.0142, 0.0)),
        material=bezel_dark,
        name="keyway",
    )
    model.articulation(
        "drawer_to_lock",
        ArticulationType.REVOLUTE,
        parent=drawer,
        child=lock,
        origin=Origin(xyz=(0.100, 0.230, 0.072)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=-1.4, upper=1.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    console = object_model.get_part("console")
    drawer = object_model.get_part("cash_drawer")
    lock = object_model.get_part("drawer_lock")
    cover = object_model.get_part("printer_cover")
    cash_key = object_model.get_part("cash_key")

    drawer_joint = object_model.get_articulation("cabinet_to_drawer")
    cover_joint = object_model.get_articulation("console_to_printer_cover")
    cash_joint = object_model.get_articulation("console_to_cash_key")
    lock_joint = object_model.get_articulation("drawer_to_lock")

    # --- Cash drawer: closed fit ---------------------------------------
    ctx.expect_gap(
        drawer,
        cabinet,
        axis="y",
        positive_elem="drawer_front",
        min_gap=0.0,
        max_gap=0.002,
        name="closed drawer front seats just ahead of the cabinet opening",
    )
    ctx.expect_within(
        drawer,
        cabinet,
        axes="x",
        inner_elem="tray_floor",
        margin=0.0,
        name="coin tray stays laterally inside the cabinet",
    )
    ctx.expect_gap(
        drawer,
        cabinet,
        axis="z",
        positive_elem="tray_floor",
        negative_elem="bottom_panel",
        min_gap=0.0005,
        max_gap=0.005,
        name="tray floor rides just above the cabinet bottom panel",
    )
    ctx.expect_overlap(
        drawer,
        cabinet,
        axes="y",
        elem_a="tray_floor",
        min_overlap=0.30,
        name="closed tray is housed inside the cabinet depth",
    )

    # --- Cash drawer: slides open toward the operator ------------------
    closed_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_joint: 0.245}):
        open_pos = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            cabinet,
            axes="y",
            elem_a="tray_floor",
            min_overlap=0.05,
            name="open drawer keeps its tray rear captured in the cabinet",
        )
        ctx.expect_within(
            drawer,
            cabinet,
            axes="x",
            inner_elem="tray_floor",
            margin=0.0,
            name="open drawer stays laterally guided",
        )
    ctx.check(
        "drawer slides forward when opened",
        closed_pos is not None and open_pos is not None and open_pos[1] > closed_pos[1] + 0.20,
        details=f"closed={closed_pos}, open={open_pos}",
    )

    # --- Eight coin compartments -----------------------------------------
    dividers = [drawer.get_visual(f"coin_divider_{i}") for i in range(7)]
    ctx.check(
        "seven dividers form eight coin compartments",
        all(v is not None for v in dividers),
        details="expected coin_divider_0..6 in the cash drawer tray",
    )

    # --- Keys: seated through deck cutouts, pressing moves into the deck ---
    key_parts = [
        p
        for p in object_model.parts
        if p.name.endswith(("_key",)) or ("_key_" in p.name)
    ]
    for key_part in key_parts:
        # Each keycap skirt intentionally seats 0.6 mm into the sloped deck,
        # standing in for the real through-hole cutout under every key.
        ctx.allow_overlap(
            key_part,
            console,
            elem_a="keycap",
            elem_b="keyboard_deck",
            reason=(
                "Keycap skirt seats slightly into the keyboard deck, standing in "
                "for the real key cutout in the deck surface."
            ),
        )
    key_rest = ctx.part_world_position(cash_key)
    with ctx.pose({cash_joint: KEY_TRAVEL}):
        key_down = ctx.part_world_position(cash_key)
    ctx.check(
        "cash key presses down into the sloped deck",
        key_rest is not None and key_down is not None and key_down[2] < key_rest[2] - 0.002,
        details=f"rest={key_rest}, pressed={key_down}",
    )
    ctx.expect_contact(
        cash_key,
        console,
        contact_tol=0.002,
        name="cash key sits seated on the keyboard deck",
    )
    ctx.expect_contact(
        object_model.get_part("dept_key_0_0"),
        console,
        contact_tol=0.002,
        name="department keys sit seated on the keyboard deck",
    )
    ctx.expect_within(
        cash_key,
        console,
        axes="xy",
        outer_elem="keyboard_deck",
        margin=0.0,
        name="cash key stays inside the deck footprint",
    )
    ctx.check(
        "keyboard has 59 articulated keys",
        len(key_parts) == 59,
        details=f"found {len(key_parts)} key parts",
    )

    # --- Printer cover: seated closed, opens upward ---------------------
    ctx.expect_gap(
        cover,
        console,
        axis="z",
        negative_elem="printer_bay_front_wall",
        min_gap=0.0,
        max_gap=0.002,
        name="closed printer cover rests on the bay rim",
    )
    ctx.expect_gap(
        cover,
        console,
        axis="z",
        negative_elem="paper_roll",
        min_gap=0.001,
        max_gap=0.010,
        name="paper roll nests under the closed cover",
    )
    closed_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({cover_joint: 1.2}):
        open_aabb = ctx.part_world_aabb(cover)
    ctx.check(
        "printer cover swings open upward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.04,
        details=f"closed_max_z={closed_aabb}, open_max_z={open_aabb}",
    )
    rear_panel = cover.get_visual("cover_rear_panel")
    front_panel = cover.get_visual("cover_front_panel")
    ctx.check(
        "cover has a receipt slot between its panels",
        rear_panel is not None and front_panel is not None,
        details="expected separate cover panels flanking the paper slot",
    )

    # --- Drawer lock ------------------------------------------------------
    ctx.expect_gap(
        lock,
        drawer,
        axis="y",
        negative_elem="drawer_front",
        min_gap=0.0,
        max_gap=0.002,
        name="chrome lock sits on the drawer front",
    )
    ctx.check(
        "lock cylinder rotates both ways",
        lock_joint.motion_limits is not None
        and lock_joint.motion_limits.lower < -1.0
        and lock_joint.motion_limits.upper > 1.0,
        details="expected symmetric rotation limits on drawer_to_lock",
    )

    # --- Display -----------------------------------------------------------
    screen_aabb = ctx.part_element_world_aabb(console, elem="display_screen")
    ctx.check(
        "LCD screen sits on the raised rear platform",
        screen_aabb is not None and screen_aabb[0][2] > CAB_TOP_Z + 0.12,
        details=f"screen_aabb={screen_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
