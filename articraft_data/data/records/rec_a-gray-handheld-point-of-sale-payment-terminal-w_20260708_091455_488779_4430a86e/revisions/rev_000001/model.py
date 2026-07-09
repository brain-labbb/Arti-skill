from __future__ import annotations

# Gray handheld point-of-sale payment terminal.
#
# Canonical frame: the terminal rests flat on the table, +Y is the rear
# (receipt printer end), -Y is the front (user / chip card slot end), +X is
# the right side carrying the magnetic stripe swipe groove, +Z is up.
# The top face is a single sloped plane rising from the front edge toward the
# rear printer deck, exactly as cut into the CadQuery housing solid, so every
# key, the display, and the hinged paper cover are placed with exact math on
# that same plane.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
)

# ---------------------------------------------------------------- dimensions
BODY_W = 0.082
BODY_L = 0.180
CORNER_R = 0.012
Y_FRONT = -0.090
Y_REAR = 0.090
Z_FRONT = 0.034
Z_REAR = 0.058

SLOPE = (Z_REAR - Z_FRONT) / (Y_REAR - Y_FRONT)
ANG = math.atan(SLOPE)
COS_A = math.cos(ANG)
SIN_A = math.sin(ANG)

# Paper bay (receipt roll pocket) cut into the rear top deck.
BAY_X_HALF = 0.031
BAY_Y0 = 0.030
BAY_Y1 = 0.077
BAY_FLOOR = 0.026

# Magnetic stripe swipe groove along the right side.
GROOVE_X0 = 0.0345
GROOVE_X1 = 0.0380
GROOVE_Z0 = 0.022

# Chip card slot in the front face.
SLOT_W = 0.056
SLOT_Z0 = 0.009
SLOT_Z1 = 0.0125
SLOT_Y_BACK = -0.0575

# Paper exit slit just in front of the cover front edge.
SLIT_Y0 = 0.021
SLIT_Y1 = 0.0245

# Hinge of the paper cover.
HINGE_Y = 0.0855
HINGE_LIFT = 0.0035  # hinge axis height above the sloped top plane

KEY_TRAVEL = 0.0022

# Keycaps are intentionally seated slightly into the keypad plate so each key
# reads as mounted in its aperture (scoped allowances + contact proofs in
# run_tests()).
KEY_SEAT_EMBED = 0.0002


def top_z(y: float) -> float:
    """Exact z of the sloped top plane at a given y."""
    return Z_FRONT + (y - Y_FRONT) * SLOPE


def face_point(x: float, y: float, lift: float = 0.0) -> tuple[float, float, float]:
    """World point on the sloped top plane, offset `lift` along its normal."""
    return (x, y - lift * SIN_A, top_z(y) + lift * COS_A)


def slope_frame_xyz(u_dist: float, n_dist: float) -> tuple[float, float, float]:
    """Offset expressed in the hinge child frame: u = down-slope toward the
    front along the top plane, n = outward plane normal."""
    return (
        0.0,
        -u_dist * COS_A - n_dist * SIN_A,
        -u_dist * SIN_A + n_dist * COS_A,
    )


# ------------------------------------------------------------- housing solid
def _build_housing_solid() -> cq.Workplane:
    # Side profile (wedge) extruded across the width, intersected with the
    # rounded-rectangle footprint, then detailed with boolean cuts.
    profile = (
        cq.Workplane("YZ")
        .polyline(
            [
                (Y_FRONT, 0.0),
                (Y_REAR, 0.0),
                (Y_REAR, Z_REAR),
                (Y_FRONT, Z_FRONT),
            ]
        )
        .close()
        .extrude(BODY_W / 2.0, both=True)
    )
    footprint = (
        cq.Workplane("XY")
        .rect(BODY_W, BODY_L)
        .extrude(0.075)
        .edges("|Z")
        .fillet(CORNER_R)
    )
    body = footprint.intersect(profile)

    # Receipt paper bay pocket in the rear deck.
    bay = (
        cq.Workplane("XY")
        .box(BAY_X_HALF * 2.0, BAY_Y1 - BAY_Y0, 0.050)
        .translate((0.0, (BAY_Y0 + BAY_Y1) / 2.0, BAY_FLOOR + 0.025))
    )
    body = body.cut(bay)

    # Magnetic stripe swipe groove along the right side, open at both ends.
    groove = (
        cq.Workplane("XY")
        .box(GROOVE_X1 - GROOVE_X0, 0.300, 0.060)
        .translate(((GROOVE_X0 + GROOVE_X1) / 2.0, 0.0, GROOVE_Z0 + 0.030))
    )
    body = body.cut(groove)

    # Chip card slot in the front face.
    slot = (
        cq.Workplane("XY")
        .box(SLOT_W, 0.034, SLOT_Z1 - SLOT_Z0)
        .translate((0.0, -0.0745, (SLOT_Z0 + SLOT_Z1) / 2.0))
    )
    body = body.cut(slot)

    # Paper exit slit across the top, just in front of the cover edge.
    slit_mid_y = (SLIT_Y0 + SLIT_Y1) / 2.0
    slit = (
        cq.Workplane("XY")
        .box(0.058, SLIT_Y1 - SLIT_Y0, 0.020)
        .translate((0.0, slit_mid_y, top_z(slit_mid_y) - 0.0025 + 0.010))
    )
    body = body.cut(slit)
    return body


# ------------------------------------------------------------------ key caps
_KEYCAP_CACHE: dict[str, object] = {}


def _keycap_mesh(name: str, w: float, d: float, h: float):
    mesh = _KEYCAP_CACHE.get(name)
    if mesh is None:
        bottom = rounded_rect_profile(w, d, 0.0016)
        top = rounded_rect_profile(w - 0.0024, d - 0.0020, 0.0012)
        geom = LoftGeometry(
            [
                [(x, y, 0.0) for x, y in bottom],
                [(x, y, h) for x, y in top],
            ],
            cap=True,
            closed=True,
        )
        mesh = mesh_from_geometry(geom, name)
        _KEYCAP_CACHE[name] = mesh
    return mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pos_payment_terminal")

    shell_gray = model.material("shell_gray", rgba=(0.74, 0.74, 0.76, 1.0))
    deck_charcoal = model.material("deck_charcoal", rgba=(0.24, 0.24, 0.26, 1.0))
    bezel_dark = model.material("bezel_dark", rgba=(0.17, 0.17, 0.19, 1.0))
    screen_pale = model.material("screen_pale", rgba=(0.86, 0.90, 0.86, 1.0))
    key_black = model.material("key_black", rgba=(0.12, 0.12, 0.13, 1.0))
    key_gray = model.material("key_gray", rgba=(0.58, 0.58, 0.60, 1.0))
    key_red = model.material("key_red", rgba=(0.76, 0.10, 0.10, 1.0))
    key_yellow = model.material("key_yellow", rgba=(0.94, 0.74, 0.08, 1.0))
    key_green = model.material("key_green", rgba=(0.10, 0.55, 0.18, 1.0))
    cover_gray = model.material("cover_gray", rgba=(0.62, 0.62, 0.65, 1.0))
    paper_white = model.material("paper_white", rgba=(0.96, 0.96, 0.95, 1.0))
    card_blue = model.material("card_blue", rgba=(0.10, 0.34, 0.58, 1.0))
    card_white = model.material("card_white", rgba=(0.92, 0.93, 0.94, 1.0))
    rubber_dark = model.material("rubber_dark", rgba=(0.10, 0.10, 0.11, 1.0))

    # ------------------------------------------------------------- housing
    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_build_housing_solid(), "pos_housing"),
        material=shell_gray,
        name="housing_shell",
    )

    # Dark keypad surround plate, slightly proud of the sloped face.
    plate_y_center = -0.0475
    housing.visual(
        Box((0.060, 0.0757, 0.0024)),
        origin=Origin(xyz=face_point(0.0, plate_y_center), rpy=(ANG, 0.0, 0.0)),
        material=deck_charcoal,
        name="keypad_plate",
    )

    # Display bezel and pale LCD screen above the keypad.
    housing.visual(
        Box((0.050, 0.0242, 0.005)),
        origin=Origin(xyz=face_point(0.0, 0.004, 0.001), rpy=(ANG, 0.0, 0.0)),
        material=bezel_dark,
        name="display_bezel",
    )
    housing.visual(
        Box((0.042, 0.018, 0.0016)),
        origin=Origin(xyz=face_point(0.0, 0.004, 0.0037), rpy=(ANG, 0.0, 0.0)),
        material=screen_pale,
        name="display_screen",
    )

    # White receipt paper roll resting in the bay (lightly seated in floor).
    roll_y = (BAY_Y0 + BAY_Y1) / 2.0
    housing.visual(
        Cylinder(radius=0.0105, length=0.058),
        origin=Origin(xyz=(0.0, roll_y, BAY_FLOOR + 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=paper_white,
        name="paper_roll",
    )

    # Hinge bosses flanking the cover knuckles on the rear rim.
    for index, boss_x in enumerate((-0.012, 0.012)):
        housing.visual(
            Box((0.008, 0.008, 0.0078)),
            origin=Origin(
                xyz=face_point(boss_x, HINGE_Y, HINGE_LIFT - 0.0009),
                rpy=(ANG, 0.0, 0.0),
            ),
            material=shell_gray,
            name=f"hinge_boss_{index}",
        )

    # Rubber feet under the base, lightly embedded into the bottom shell.
    for index, (fx, fy) in enumerate(
        ((-0.028, -0.070), (0.028, -0.070), (-0.028, 0.070), (0.028, 0.070))
    ):
        housing.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(fx, fy, -0.0005)),
            material=rubber_dark,
            name=f"foot_{index}",
        )

    # ---------------------------------------------------------- paper cover
    # Hinged receipt printer cover: child frame sits on the hinge axis,
    # HINGE_LIFT above the sloped plane at HINGE_Y; the plate extends
    # down-slope toward the front and rests on the bay rim when closed.
    cover = model.part("paper_cover")
    # Seated 0.2 mm into the bay rim (scoped allowance + contact proof).
    plate_center = slope_frame_xyz(0.0325, -0.0012)
    cover.visual(
        Box((0.066, 0.056, 0.005)),
        origin=Origin(xyz=plate_center, rpy=(ANG, 0.0, 0.0)),
        material=cover_gray,
        name="cover_plate",
    )
    # Raised grip ridge near the cover front edge.
    cover.visual(
        Box((0.036, 0.005, 0.0022)),
        origin=Origin(xyz=slope_frame_xyz(0.052, 0.0024), rpy=(ANG, 0.0, 0.0)),
        material=cover_gray,
        name="grip_ridge",
    )
    # Hinge knuckle barrels on the axis plus tabs joining them to the plate.
    for index, kx in enumerate((-0.024, 0.024)):
        cover.visual(
            Cylinder(radius=0.0034, length=0.012),
            origin=Origin(xyz=(kx, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=cover_gray,
            name=f"hinge_knuckle_{index}",
        )
        tab_xyz = slope_frame_xyz(0.0035, -0.0008)
        cover.visual(
            Box((0.010, 0.007, 0.005)),
            origin=Origin(xyz=(kx, tab_xyz[1], tab_xyz[2]), rpy=(ANG, 0.0, 0.0)),
            material=cover_gray,
            name=f"knuckle_tab_{index}",
        )

    model.articulation(
        "housing_to_paper_cover",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=cover,
        origin=Origin(xyz=face_point(0.0, HINGE_Y, HINGE_LIFT)),
        # Cover extends toward -Y from the hinge; -X axis lifts the free edge.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=2.0, lower=0.0, upper=1.85),
    )

    # ------------------------------------------------------------ key field
    def add_key(
        part_name: str,
        mesh,
        size: tuple[float, float, float],
        material,
        kx: float,
        ky: float,
    ) -> None:
        key = model.part(part_name)
        key.visual(mesh, material=material, name="keycap")
        model.articulation(
            f"housing_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=key,
            origin=Origin(
                xyz=face_point(kx, ky, 0.0012 - KEY_SEAT_EMBED),
                rpy=(ANG, 0.0, 0.0),
            ),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=3.0, velocity=0.05, lower=0.0, upper=KEY_TRAVEL
            ),
        )

    func_mesh = _keycap_mesh("keycap_function", 0.0115, 0.0075, 0.0032)
    num_mesh = _keycap_mesh("keycap_numeric", 0.0135, 0.0095, 0.0042)
    wide_mesh = _keycap_mesh("keycap_wide", 0.0160, 0.0095, 0.0042)

    # Row of four gray function keys under the display.
    for index, kx in enumerate((-0.0225, -0.0075, 0.0075, 0.0225)):
        add_key(
            f"function_key_{index}",
            func_mesh,
            (0.0115, 0.0075, 0.0032),
            key_gray,
            kx,
            -0.016,
        )

    # 3 x 4 black numeric keypad: 1-9, then F / 0 / dot.
    numeric_names = [
        "key_1", "key_2", "key_3",
        "key_4", "key_5", "key_6",
        "key_7", "key_8", "key_9",
        "key_f", "key_0", "key_dot",
    ]
    row_ys = (-0.0285, -0.041, -0.0535, -0.066)
    col_xs = (-0.019, 0.0, 0.019)
    for row, ky in enumerate(row_ys):
        for col, kx in enumerate(col_xs):
            add_key(
                numeric_names[row * 3 + col],
                num_mesh,
                (0.0135, 0.0095, 0.0042),
                key_black,
                kx,
                ky,
            )

    # Front command row: red cancel, yellow clear, green enter.
    add_key("cancel_key", wide_mesh, (0.016, 0.0095, 0.0042), key_red, -0.019, -0.0785)
    add_key("clear_key", wide_mesh, (0.016, 0.0095, 0.0042), key_yellow, 0.0, -0.0785)
    add_key("enter_key", wide_mesh, (0.016, 0.0095, 0.0042), key_green, 0.019, -0.0785)

    # ------------------------------------------------------------ bank card
    # Inserted chip card resting on the slot floor and protruding forward.
    card = model.part("bank_card")
    card_solid = (
        cq.Workplane("XY").box(0.054, 0.086, 0.0022).edges("|Z").fillet(0.004)
    )
    card.visual(
        mesh_from_cadquery(card_solid, "bank_card_body"),
        material=card_blue,
        name="card_body",
    )
    card.visual(
        Box((0.011, 0.008, 0.0006)),
        origin=Origin(xyz=(0.016, -0.032, 0.0012)),
        material=card_white,
        name="card_logo",
    )
    card.visual(
        Box((0.044, 0.0045, 0.0005)),
        origin=Origin(xyz=(0.0, -0.020, 0.0011)),
        material=card_white,
        name="card_band",
    )

    # Card center: rear end 0.0025 in front of the slot back wall, bottom
    # resting exactly on the slot floor.
    card_center_y = SLOT_Y_BACK - 0.0025 - 0.043
    model.articulation(
        "housing_to_bank_card",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=card,
        origin=Origin(xyz=(0.0, card_center_y, SLOT_Z0 + 0.0011)),
        # Positive travel withdraws the card toward the user (front).
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.1, lower=0.0, upper=0.018),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    cover = object_model.get_part("paper_cover")
    card = object_model.get_part("bank_card")
    enter = object_model.get_part("enter_key")
    cancel = object_model.get_part("cancel_key")
    clear = object_model.get_part("clear_key")
    hinge = object_model.get_articulation("housing_to_paper_cover")
    card_slide = object_model.get_articulation("housing_to_bank_card")
    enter_press = object_model.get_articulation("housing_to_enter_key")

    # --- receipt printer paper cover -------------------------------------
    ctx.allow_overlap(
        cover,
        housing,
        elem_a="cover_plate",
        elem_b="housing_shell",
        reason=(
            "The closed cover lip is intentionally seated 0.2 mm into the "
            "bay rim so the printer lid reads as latched shut."
        ),
    )
    ctx.expect_contact(
        cover,
        housing,
        contact_tol=5e-4,
        name="closed paper cover rests on the rear deck rim",
    )
    ctx.expect_overlap(
        cover,
        housing,
        axes="xy",
        min_overlap=0.030,
        name="paper cover spans the paper bay opening",
    )
    ctx.expect_within(
        housing,
        cover,
        axes="xy",
        inner_elem="paper_roll",
        outer_elem="cover_plate",
        margin=0.001,
        name="paper roll sits under the closed cover",
    )
    closed_aabb = ctx.part_element_world_aabb(cover, elem="cover_plate")
    with ctx.pose({hinge: 1.5}):
        open_aabb = ctx.part_element_world_aabb(cover, elem="cover_plate")
        ctx.check(
            "paper cover opens upward about its rear hinge",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] > closed_aabb[1][2] + 0.020,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    # --- chip card in the front slot --------------------------------------
    ctx.expect_contact(
        card,
        housing,
        contact_tol=5e-4,
        name="bank card seats on the chip slot floor",
    )
    ctx.expect_within(
        card,
        housing,
        axes="x",
        margin=0.0,
        name="bank card stays centered in the slot width",
    )
    ctx.expect_overlap(
        card,
        housing,
        axes="y",
        min_overlap=0.020,
        name="bank card is inserted into the housing slot",
    )
    card_rest = ctx.part_world_position(card)
    ctx.check(
        "bank card protrudes from the front edge",
        card_rest is not None and card_rest[1] < Y_FRONT,
        details=f"card position={card_rest}",
    )
    with ctx.pose({card_slide: 0.018}):
        ctx.expect_overlap(
            card,
            housing,
            axes="y",
            min_overlap=0.008,
            name="withdrawn card remains retained in the slot",
        )
        card_pulled = ctx.part_world_position(card)
        ctx.check(
            "card slides out toward the front",
            card_rest is not None
            and card_pulled is not None
            and card_pulled[1] < card_rest[1] - 0.010,
            details=f"rest={card_rest}, pulled={card_pulled}",
        )

    # --- keypad ------------------------------------------------------------
    all_key_names = (
        "function_key_0", "function_key_1", "function_key_2", "function_key_3",
        "key_1", "key_2", "key_3", "key_4", "key_5", "key_6",
        "key_7", "key_8", "key_9", "key_f", "key_0", "key_dot",
        "cancel_key", "clear_key", "enter_key",
    )
    for key_name in all_key_names:
        key_part = object_model.get_part(key_name)
        ctx.allow_overlap(
            key_part,
            housing,
            elem_a="keycap",
            elem_b="keypad_plate",
            reason=(
                "Keycap skirt is intentionally seated 0.2 mm into the keypad "
                "plate so the key reads as mounted in its aperture; the "
                "prismatic travel presses it further into the plate."
            ),
        )
        ctx.expect_contact(
            key_part,
            housing,
            elem_a="keycap",
            elem_b="keypad_plate",
            contact_tol=5e-4,
            name=f"{key_name} seats on the keypad plate",
        )
    enter_pos = ctx.part_world_position(enter)
    cancel_pos = ctx.part_world_position(cancel)
    clear_pos = ctx.part_world_position(clear)
    ctx.check(
        "green enter key sits front-right, red cancel front-left",
        enter_pos is not None
        and cancel_pos is not None
        and enter_pos[0] > 0.010
        and cancel_pos[0] < -0.010
        and enter_pos[1] < -0.060
        and cancel_pos[1] < -0.060,
        details=f"enter={enter_pos}, cancel={cancel_pos}",
    )
    ctx.check(
        "yellow clear key sits between cancel and enter",
        clear_pos is not None and abs(clear_pos[0]) < 0.005,
        details=f"clear={clear_pos}",
    )
    numeric_count = sum(
        1
        for name in (
            "key_1", "key_2", "key_3", "key_4", "key_5", "key_6",
            "key_7", "key_8", "key_9", "key_f", "key_0", "key_dot",
        )
        if object_model.get_part(name) is not None
    )
    ctx.check(
        "keypad has 12 numeric-block keys",
        numeric_count == 12,
        details=f"numeric_count={numeric_count}",
    )
    with ctx.pose({enter_press: KEY_TRAVEL}):
        enter_pressed = ctx.part_world_position(enter)
        ctx.check(
            "enter key presses down into the keypad face",
            enter_pos is not None
            and enter_pressed is not None
            and enter_pressed[2] < enter_pos[2] - 0.001,
            details=f"rest={enter_pos}, pressed={enter_pressed}",
        )

    # --- display -----------------------------------------------------------
    screen_aabb = ctx.part_element_world_aabb(housing, elem="display_screen")
    bezel_aabb = ctx.part_element_world_aabb(housing, elem="display_bezel")
    ctx.check(
        "display screen is framed inside its bezel",
        screen_aabb is not None
        and bezel_aabb is not None
        and screen_aabb[0][0] > bezel_aabb[0][0]
        and screen_aabb[1][0] < bezel_aabb[1][0]
        and screen_aabb[0][1] > bezel_aabb[0][1]
        and screen_aabb[1][1] < bezel_aabb[1][1],
        details=f"screen={screen_aabb}, bezel={bezel_aabb}",
    )
    ctx.check(
        "display sits behind the keypad, in front of the printer bay",
        screen_aabb is not None
        and screen_aabb[0][1] > -0.010
        and screen_aabb[1][1] < BAY_Y0,
        details=f"screen={screen_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
