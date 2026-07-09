from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_TOP_Z = 0.036
DECK_TOP_Z = 0.040
BUTTON_TRAVEL = 0.003
KEY_COLS = 4
KEY_ROWS = 4
BODY_SIDE = 0.280
BODY_CORNER_RADIUS = 0.070
GRILLE_RADIUS = 0.082
LED_RADIUS = 0.115
FOOT_RADIUS = 0.105
SPEAKER_ANGLES = (90.0, 210.0, 330.0)


def _rounded_box(length: float, width: float, height: float, radius: float) -> cq.Workplane:
    """CadQuery box with rounded plan corners and its bottom face on z=0."""
    box = cq.Workplane("XY").box(length, width, height, centered=(True, True, False))
    if radius > 0:
        box = box.edges("|Z").fillet(radius)
    return box


def _radial_xy(radius: float, angle_deg: float) -> tuple[float, float]:
    angle = math.radians(angle_deg)
    return radius * math.cos(angle), radius * math.sin(angle)


def _rounded_square_body() -> cq.Workplane:
    """Low rounded-square conference speakerphone body with large corner radii."""
    body = _rounded_box(BODY_SIDE, BODY_SIDE, BODY_TOP_Z, BODY_CORNER_RADIUS)
    # Subtle top-edge chamfer and bottom-edge fillet for manufactured realism.
    return body.edges(">Z").fillet(0.004).edges("<Z").fillet(0.003)


def _control_deck() -> cq.Workplane:
    """Slightly raised rectangular controls island on the central front."""
    return (
        _rounded_box(0.150, 0.138, 0.006, 0.018)
        .translate((0.0, -0.060, BODY_TOP_Z - 0.002))
        .edges(">Z")
        .fillet(0.0015)
    )


def _rubber_key() -> cq.Workplane:
    """One low rounded rectangular press key; bottom face is at local z=0."""
    return _rounded_box(0.019, 0.012, 0.0055, 0.004).edges(">Z").fillet(0.001)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="white_rounded_square_conference_speakerphone")

    # Cisco-style white body with silver deck accent.
    body_white = Material("cisco_white_body", rgba=(0.92, 0.92, 0.90, 1.0))
    silver_deck = Material("silver_deck_accent", rgba=(0.72, 0.73, 0.74, 1.0))
    dark_graphite = Material("dark_graphite", rgba=(0.055, 0.060, 0.058, 1.0))
    perforated_metal = Material("black_perforated_metal", rgba=(0.012, 0.014, 0.014, 1.0))
    rubber_black = Material("soft_rubber_black", rgba=(0.0, 0.0, 0.0, 1.0))
    lcd_green = Material("green_lcd_glass", rgba=(0.10, 0.78, 0.56, 0.92))
    led_green = Material("lit_green_led", rgba=(0.0, 1.0, 0.18, 1.0))
    label_white = Material("white_button_marks", rgba=(0.86, 0.88, 0.84, 1.0))
    shadow_black = Material("deep_black_recess", rgba=(0.0, 0.0, 0.0, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_rounded_square_body(), "rounded_square_body_shell", tolerance=0.0008),
        material=body_white,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_control_deck(), "raised_control_deck", tolerance=0.0008),
        material=silver_deck,
        name="control_deck",
    )

    # Three perforated speaker grilles in a 120-degree triangular pattern on the square top deck.
    grille_mesh = mesh_from_geometry(
        PerforatedPanelGeometry(
            (0.108, 0.070),
            0.003,
            hole_diameter=0.0032,
            pitch=(0.0065, 0.0060),
            frame=0.006,
            corner_radius=0.016,
            stagger=True,
        ),
        "perforated_speaker_grille",
    )
    for idx, angle in enumerate(SPEAKER_ANGLES):
        x, y = _radial_xy(GRILLE_RADIUS, angle)
        body.visual(
            grille_mesh,
            origin=Origin(xyz=(x, y, BODY_TOP_Z + 0.0015), rpy=(0.0, 0.0, math.radians(angle))),
            material=perforated_metal,
            name=f"speaker_grille_{idx}",
        )
        lx, ly = _radial_xy(LED_RADIUS, angle)
        body.visual(
            Cylinder(radius=0.0064, length=0.003),
            origin=Origin(xyz=(lx, ly, BODY_TOP_Z + 0.0035)),
            material=led_green,
            name=f"status_led_{idx}",
        )

    # Central LCD and its black surround, with a few dark "digits" on the glass.
    body.visual(
        Box((0.104, 0.047, 0.0018)),
        origin=Origin(xyz=(0.0, -0.023, DECK_TOP_Z + 0.0009)),
        material=shadow_black,
        name="lcd_bezel",
    )
    body.visual(
        Box((0.086, 0.032, 0.0022)),
        origin=Origin(xyz=(0.0, -0.023, DECK_TOP_Z + 0.0021)),
        material=lcd_green,
        name="lcd_display",
    )
    for idx, (tx, ty, sx) in enumerate(((-0.026, -0.023, 0.020), (0.000, -0.018, 0.015), (0.027, -0.029, 0.018))):
        body.visual(
            Box((sx, 0.0020, 0.0006)),
            origin=Origin(xyz=(tx, ty, DECK_TOP_Z + 0.0035)),
            material=dark_graphite,
            name=f"lcd_text_{idx}",
        )

    # Three low rubber feet under the rounded-square body.
    for idx, angle in enumerate(SPEAKER_ANGLES):
        x, y = _radial_xy(FOOT_RADIUS, angle)
        body.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(x, y, -0.003)),
            material=rubber_black,
            name=f"rubber_foot_{idx}",
        )

    key_mesh = mesh_from_cadquery(_rubber_key(), "rounded_rubber_key", tolerance=0.0004)
    x_positions = (-0.036, -0.012, 0.012, 0.036)
    y_positions = (-0.058, -0.077, -0.096, -0.115)
    for row, y in enumerate(y_positions):
        for col, x in enumerate(x_positions):
            key = model.part(f"key_{row}_{col}")
            key.visual(key_mesh, material=rubber_black, name="key_cap")
            key.visual(
                Box((0.008, 0.0015, 0.0005)),
                origin=Origin(xyz=(0.0, 0.0, 0.00565)),
                material=label_white,
                name="key_label",
            )
            model.articulation(
                f"body_to_key_{row}_{col}",
                ArticulationType.PRISMATIC,
                parent=body,
                child=key,
                origin=Origin(xyz=(x, y, DECK_TOP_Z)),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(effort=1.0, velocity=0.10, lower=0.0, upper=BUTTON_TRAVEL),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    keys = [object_model.get_part(f"key_{r}_{c}") for r in range(KEY_ROWS) for c in range(KEY_COLS)]
    joints = [
        object_model.get_articulation(f"body_to_key_{r}_{c}")
        for r in range(KEY_ROWS)
        for c in range(KEY_COLS)
    ]

    # Verify the rounded-square body shell replaced the tri-star shell.
    ctx.check(
        "rounded square body_shell exists on body",
        any(v.name == "body_shell" for v in body.visuals),
    )

    ctx.check("three speaker grilles in 120-degree pattern", len([v for v in body.visuals if v.name.startswith("speaker_grille_")]) == 3)
    ctx.check("three green status LEDs", len([v for v in body.visuals if v.name.startswith("status_led_")]) == 3)
    ctx.check("full four by four keypad", len(keys) == 16 and len(joints) == 16)

    # Verify the body has a roughly square plan footprint (width ≈ depth).
    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is not None:
        lo, hi = body_aabb
        dx = hi[0] - lo[0]
        dy = hi[1] - lo[1]
        ctx.check(
            "body shell has near-square plan footprint",
            abs(dx - dy) < 0.010,
            details=f"dx={dx:.4f}, dy={dy:.4f}",
        )

    for key in keys:
        ctx.expect_gap(
            key,
            body,
            axis="z",
            min_gap=0.0,
            max_gap=0.0006,
            positive_elem="key_cap",
            negative_elem="control_deck",
            name=f"{key.name} sits on control deck",
        )

    sample_key = object_model.get_part("key_0_0")
    sample_joint = object_model.get_articulation("body_to_key_0_0")
    rest_position = ctx.part_world_position(sample_key)
    with ctx.pose({sample_joint: BUTTON_TRAVEL}):
        pressed_position = ctx.part_world_position(sample_key)
    ctx.check(
        "key travel presses downward",
        rest_position is not None
        and pressed_position is not None
        and pressed_position[2] < rest_position[2] - 0.0025,
        details=f"rest={rest_position}, pressed={pressed_position}",
    )

    return ctx.report()


object_model = build_object_model()
