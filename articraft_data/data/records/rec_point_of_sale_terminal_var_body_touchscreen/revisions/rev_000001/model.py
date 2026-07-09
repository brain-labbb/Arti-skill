from __future__ import annotations

# Android smart POS touchscreen terminal (Clover Flex / PAX A920 class).
#
# Variant of the handheld POS terminal: the numeric keypad and function keys
# have been replaced by a single large capacitive touchscreen covering most
# of the sloped top face. Two physical side hard-buttons remain on the right
# edge. The rear flip-cover receipt printer, chip-card slot with inserted
# bank card, and magnetic stripe swipe groove are preserved.
#
# Canonical frame: the terminal rests flat on the table, +Y is the rear
# (receipt printer end), -Y is the front (user / chip card slot end), +X is
# the right side carrying the magnetic stripe swipe groove, +Z is up.
# The top face is a single sloped plane rising from the front edge toward the
# rear printer deck.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
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
HINGE_LIFT = 0.0035

# Side button travel.
BUTTON_TRAVEL = 0.0015


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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pos_smart_terminal")

    shell_gray = model.material("shell_gray", rgba=(0.74, 0.74, 0.76, 1.0))
    bezel_dark = model.material("bezel_dark", rgba=(0.08, 0.08, 0.10, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.04, 0.05, 0.07, 1.0))
    cover_gray = model.material("cover_gray", rgba=(0.62, 0.62, 0.65, 1.0))
    paper_white = model.material("paper_white", rgba=(0.96, 0.96, 0.95, 1.0))
    card_blue = model.material("card_blue", rgba=(0.10, 0.34, 0.58, 1.0))
    card_white = model.material("card_white", rgba=(0.92, 0.93, 0.94, 1.0))
    rubber_dark = model.material("rubber_dark", rgba=(0.10, 0.10, 0.11, 1.0))
    button_dark = model.material("button_dark", rgba=(0.18, 0.18, 0.20, 1.0))
    nfc_light = model.material("nfc_light", rgba=(0.85, 0.86, 0.88, 1.0))
    status_dim = model.material("status_dim", rgba=(0.20, 0.22, 0.25, 1.0))

    # ------------------------------------------------------------- housing
    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_build_housing_solid(), "pos_housing"),
        material=shell_gray,
        name="housing_shell",
    )

    # Large touchscreen bezel covering most of the sloped top face.
    # Screen spans from y≈-0.082 to y≈0.018, just before the paper exit slit.
    screen_y_center = -0.032
    bezel_len = 0.107  # along slope
    bezel_w = 0.068
    housing.visual(
        Box((bezel_w, bezel_len, 0.003)),
        origin=Origin(xyz=face_point(0.0, screen_y_center, 0.0015), rpy=(ANG, 0.0, 0.0)),
        material=bezel_dark,
        name="display_bezel",
    )

    # Dark glass touchscreen surface, framed by the bezel.
    glass_len = 0.101
    glass_w = 0.062
    housing.visual(
        Box((glass_w, glass_len, 0.0012)),
        origin=Origin(xyz=face_point(0.0, screen_y_center, 0.0032), rpy=(ANG, 0.0, 0.0)),
        material=screen_glass,
        name="display_screen",
    )

    # Status bar decoration at the top of the screen area (thin strip).
    bar_y = screen_y_center + 0.044
    housing.visual(
        Box((0.058, 0.005, 0.0004)),
        origin=Origin(xyz=face_point(0.0, bar_y, 0.0040), rpy=(ANG, 0.0, 0.0)),
        material=status_dim,
        name="status_bar",
    )

    # NFC contactless payment glyph on the front face of the housing.
    # Embedded slightly into the front face so it reads as printed/embossed
    # on the housing surface (connects to housing_shell geometry).
    housing.visual(
        Box((0.018, 0.002, 0.010)),
        origin=Origin(xyz=(0.0, Y_FRONT - 0.0002, 0.020)),
        material=nfc_light,
        name="nfc_mark",
    )

    # Camera / barcode scanner lens near the top of the screen.
    cam_y = screen_y_center + 0.048
    housing.visual(
        Cylinder(radius=0.002, length=0.001),
        origin=Origin(xyz=face_point(0.0, cam_y, 0.004), rpy=(ANG + math.pi / 2.0, 0.0, 0.0)),
        material=rubber_dark,
        name="camera_lens",
    )

    # White receipt paper roll resting in the bay.
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

    # Rubber feet under the base.
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
    cover = model.part("paper_cover")
    plate_center = slope_frame_xyz(0.0325, -0.0012)
    cover.visual(
        Box((0.066, 0.056, 0.005)),
        origin=Origin(xyz=plate_center, rpy=(ANG, 0.0, 0.0)),
        material=cover_gray,
        name="cover_plate",
    )
    cover.visual(
        Box((0.036, 0.005, 0.0022)),
        origin=Origin(xyz=slope_frame_xyz(0.052, 0.0024), rpy=(ANG, 0.0, 0.0)),
        material=cover_gray,
        name="grip_ridge",
    )
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
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=2.0, lower=0.0, upper=1.85),
    )

    # -------------------------------------------------------- side buttons
    # Two physical hard-buttons on the right side edge below the magstripe
    # groove: power button (front) and function/scan button (rear).
    button_defs = [
        ("power_button", -0.045, 0.014),
        ("function_button", -0.018, 0.014),
    ]
    for btn_name, btn_y, btn_z in button_defs:
        btn = model.part(btn_name)
        # Button cap protrudes 2mm from the housing right face at rest.
        btn.visual(
            Box((0.002, 0.008, 0.005)),
            origin=Origin(xyz=(0.001, 0.0, 0.0)),
            material=button_dark,
            name="button_cap",
        )
        model.articulation(
            f"housing_to_{btn_name}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=btn,
            origin=Origin(xyz=(0.041, btn_y, btn_z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0, velocity=0.05, lower=0.0, upper=BUTTON_TRAVEL
            ),
        )

    # ------------------------------------------------------------ bank card
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

    card_center_y = SLOT_Y_BACK - 0.0025 - 0.043
    model.articulation(
        "housing_to_bank_card",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=card,
        origin=Origin(xyz=(0.0, card_center_y, SLOT_Z0 + 0.0011)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.1, lower=0.0, upper=0.018),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    cover = object_model.get_part("paper_cover")
    card = object_model.get_part("bank_card")
    power_btn = object_model.get_part("power_button")
    func_btn = object_model.get_part("function_button")
    hinge = object_model.get_articulation("housing_to_paper_cover")
    card_slide = object_model.get_articulation("housing_to_bank_card")
    power_joint = object_model.get_articulation("housing_to_power_button")

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

    # --- touchscreen display (variant-specific) ---------------------------
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
    # The touchscreen is screen-dominant: it must cover most of the top face.
    ctx.check(
        "touchscreen spans most of the sloped top face (length >= 0.080)",
        screen_aabb is not None
        and (screen_aabb[1][1] - screen_aabb[0][1]) >= 0.080,
        details=f"screen_aabb={screen_aabb}",
    )
    ctx.check(
        "touchscreen width covers at least 60% of body width",
        screen_aabb is not None
        and (screen_aabb[1][0] - screen_aabb[0][0]) >= 0.60 * BODY_W,
        details=f"screen_aabb={screen_aabb}",
    )
    ctx.check(
        "touchscreen sits in front of the printer bay",
        screen_aabb is not None
        and screen_aabb[1][1] < BAY_Y0,
        details=f"screen rear y={screen_aabb[1][1] if screen_aabb else None}",
    )

    # --- side buttons (variant-specific) ----------------------------------
    ctx.check(
        "power button exists as a separate articulated part",
        power_btn is not None,
        details="power_button part not found",
    )
    ctx.check(
        "function button exists as a separate articulated part",
        func_btn is not None,
        details="function_button part not found",
    )
    # Verify the power button is on the right side of the housing.
    power_pos = ctx.part_world_position(power_btn)
    ctx.check(
        "power button is on the right side of the housing (x > 0.030)",
        power_pos is not None and power_pos[0] > 0.030,
        details=f"power_button position={power_pos}",
    )
    # Verify button press moves inward.
    with ctx.pose({power_joint: BUTTON_TRAVEL}):
        power_pressed = ctx.part_world_position(power_btn)
        ctx.check(
            "power button presses inward (-X direction)",
            power_pos is not None
            and power_pressed is not None
            and power_pressed[0] < power_pos[0] - 0.0008,
            details=f"rest={power_pos}, pressed={power_pressed}",
        )

    # --- no numeric keypad present (variant-specific) ---------------------
    removed_key_names = (
        "key_1", "key_5", "key_9", "cancel_key", "enter_key", "function_key_0",
    )
    keypad_gone = True
    for kname in removed_key_names:
        try:
            object_model.get_part(kname)
            keypad_gone = False
            break
        except Exception:
            pass
    ctx.check(
        "numeric keypad and command keys removed (touchscreen variant)",
        keypad_gone,
        details="one or more keypad parts still present",
    )

    return ctx.report()


object_model = build_object_model()
