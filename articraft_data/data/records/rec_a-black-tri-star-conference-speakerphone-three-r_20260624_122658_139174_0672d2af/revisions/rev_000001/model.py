from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LoftGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


BODY_HEIGHT = 0.018
DECK_TOP_Z = 0.024
BUTTON_STEM_H = 0.0012
BUTTON_CAP_H = 0.0026
BUTTON_TRAVEL = 0.0011


def _shift_profile(profile: list[tuple[float, float]], dx: float, dy: float) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _radial_profile(samples: int = 120) -> list[tuple[float, float]]:
    """Rounded three-lobed planform with lobe centers at 90, 210, and 330 degrees."""
    pts: list[tuple[float, float]] = []
    for i in range(samples):
        theta = 2.0 * math.pi * i / samples
        # Smooth tri-star: broad lobes and shallow valleys, like a molded conference phone.
        r = 0.111 + 0.030 * math.cos(3.0 * (theta - math.pi / 2.0))
        r += 0.004 * math.cos(6.0 * (theta - math.pi / 2.0))
        pts.append((r * math.cos(theta), r * math.sin(theta)))
    return pts


def _scaled_loop(profile: list[tuple[float, float]], scale: float, z: float) -> list[tuple[float, float, float]]:
    return [(x * scale, y * scale, z) for x, y in profile]


def _woven_threads(
    angle: float,
    *,
    center_radius: float,
    width: float,
    length: float,
    exponent: float = 2.9,
    pitch: float = 0.0048,
    edge_inset: float = 0.0040,
) -> list[tuple[float, float, float, float]]:
    """Fine cross-hatch weave clipped to the wing's superellipse cloth pad.

    Builds two families of diagonal threads (a woven look) and trims each one
    to the inside chord of the rounded pad, so the texture stays strictly within
    the wing instead of overshooting its edges. Returns one tuple per thread:
    (center_x, center_y, length, yaw).
    """
    a = width * 0.5 - edge_inset
    b = length * 0.5 - edge_inset
    u = (math.cos(angle), math.sin(angle))
    v = (-math.sin(angle), math.cos(angle))

    def to_world(s: float, t: float) -> tuple[float, float]:
        return (
            u[0] * (center_radius + t) + v[0] * s,
            u[1] * (center_radius + t) + v[1] * s,
        )

    def inside(s: float, t: float) -> bool:
        return (abs(s) / a) ** exponent + (abs(t) / b) ** exponent <= 1.0

    threads: list[tuple[float, float, float, float]] = []
    n_lines = int((a + b) / pitch) + 2
    lam_max = math.hypot(a, b) + pitch
    n_steps = 200
    for dx, dy in (
        (math.cos(math.pi / 4.0), math.sin(math.pi / 4.0)),
        (math.cos(math.pi / 4.0), -math.sin(math.pi / 4.0)),
    ):
        px, py = -dy, dx  # perpendicular: offset direction between parallel threads
        for k in range(-n_lines, n_lines + 1):
            c = k * pitch
            lam_in: list[float] = []
            for j in range(n_steps + 1):
                lam = -lam_max + 2.0 * lam_max * j / n_steps
                if inside(c * px + lam * dx, c * py + lam * dy):
                    lam_in.append(lam)
            if len(lam_in) < 2 or (lam_in[-1] - lam_in[0]) < pitch * 0.5:
                continue
            s1, t1 = c * px + lam_in[0] * dx, c * py + lam_in[0] * dy
            s2, t2 = c * px + lam_in[-1] * dx, c * py + lam_in[-1] * dy
            w1 = to_world(s1, t1)
            w2 = to_world(s2, t2)
            threads.append(
                (
                    (w1[0] + w2[0]) * 0.5,
                    (w1[1] + w2[1]) * 0.5,
                    math.hypot(w2[0] - w1[0], w2[1] - w1[1]),
                    math.atan2(w2[1] - w1[1], w2[0] - w1[0]),
                )
            )
    return threads


def _wing_profile(
    angle: float,
    *,
    center_radius: float,
    width: float,
    length: float,
    exponent: float = 2.9,
    segments: int = 64,
) -> list[tuple[float, float]]:
    """Rounded fabric wing profile oriented radially from the hub."""
    base = superellipse_profile(width, length, exponent=exponent, segments=segments)
    u = (math.cos(angle), math.sin(angle))
    v = (-math.sin(angle), math.cos(angle))
    out: list[tuple[float, float]] = []
    for tangent, radial in base:
        x = u[0] * (center_radius + radial) + v[0] * tangent
        y = u[1] * (center_radius + radial) + v[1] * tangent
        out.append((x, y))
    return out


def _add_button(
    model: ArticulatedObject,
    base,
    *,
    name: str,
    x: float,
    y: float,
    sx: float,
    sy: float,
    cap_material: Material,
    label_material: Material,
    label_size: tuple[float, float] = (0.0040, 0.0011),
) -> None:
    """Add a low-profile pressable elastomer key with a hidden support stem."""
    key = model.part(name)
    key.visual(
        Cylinder(radius=min(sx, sy) * 0.12, length=BUTTON_STEM_H),
        origin=Origin(xyz=(0.0, 0.0, BUTTON_STEM_H / 2.0)),
        material=cap_material,
        name="stem",
    )
    key.visual(
        Box((sx, sy, BUTTON_CAP_H)),
        origin=Origin(xyz=(0.0, 0.0, BUTTON_STEM_H + BUTTON_CAP_H / 2.0)),
        material=cap_material,
        name="cap",
    )
    key.visual(
        Box((label_size[0], label_size[1], 0.00022)),
        origin=Origin(xyz=(0.0, 0.0, BUTTON_STEM_H + BUTTON_CAP_H + 0.00011)),
        material=label_material,
        name="legend",
    )
    model.articulation(
        f"base_to_{name}",
        ArticulationType.PRISMATIC,
        parent=base,
        child=key,
        origin=Origin(xyz=(x, y, DECK_TOP_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=0.04, lower=0.0, upper=BUTTON_TRAVEL),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="black_tri_star_conference_speakerphone")

    matte_black = model.material("matte_black", rgba=(0.005, 0.005, 0.006, 1.0))
    satin_black = model.material("satin_black", rgba=(0.018, 0.018, 0.020, 1.0))
    fabric = model.material("black_woven_fabric", rgba=(0.055, 0.055, 0.058, 1.0))
    thread = model.material("subtle_fabric_thread", rgba=(0.115, 0.115, 0.118, 1.0))
    rubber = model.material("dark_rubber_key", rgba=(0.075, 0.075, 0.078, 1.0))
    key_gray = model.material("graphite_key", rgba=(0.115, 0.115, 0.120, 1.0))
    white = model.material("pale_key_legend", rgba=(0.82, 0.84, 0.82, 1.0))
    lcd = model.material("monochrome_lcd", rgba=(0.66, 0.75, 0.70, 1.0))
    lcd_ink = model.material("lcd_dark_segments", rgba=(0.025, 0.040, 0.035, 1.0))
    green = model.material("call_key_green", rgba=(0.02, 0.45, 0.18, 1.0))
    red = model.material("end_key_red", rgba=(0.52, 0.04, 0.035, 1.0))

    base = model.part("base")

    # One continuous low molded tri-star desk body.
    plan = _radial_profile()
    body_geom = LoftGeometry(
        [
            _scaled_loop(plan, 1.00, 0.000),
            _scaled_loop(plan, 0.985, 0.010),
            _scaled_loop(plan, 0.955, BODY_HEIGHT),
        ],
        cap=True,
        closed=True,
    )
    base.visual(
        mesh_from_geometry(body_geom, "tri_star_low_body"),
        material=matte_black,
        name="tri_star_body",
    )

    # Low central hub and the slightly raised control island.
    hub = ExtrudeGeometry.from_z0(superellipse_profile(0.116, 0.104, exponent=2.4, segments=72), 0.004)
    hub.translate(0.0, -0.004, BODY_HEIGHT - 0.0004)
    base.visual(mesh_from_geometry(hub, "central_hub"), material=satin_black, name="central_hub")

    deck_profile = rounded_rect_profile(0.086, 0.119, 0.010, corner_segments=8)
    deck = ExtrudeGeometry.from_z0(_shift_profile(deck_profile, 0.0, -0.018), 0.0044)
    deck.translate(0.0, 0.0, DECK_TOP_Z - 0.0044)
    base.visual(mesh_from_geometry(deck, "control_deck"), material=satin_black, name="control_deck")

    # Three fabric speaker wings with black trim, radiating 120 degrees apart.
    wing_angles = [math.pi / 2.0, math.pi / 2.0 + 2.0 * math.pi / 3.0, math.pi / 2.0 + 4.0 * math.pi / 3.0]
    for i, angle in enumerate(wing_angles):
        trim_profile = _wing_profile(angle, center_radius=0.088, width=0.076, length=0.092)
        trim = ExtrudeGeometry.from_z0(trim_profile, 0.0022)
        trim.translate(0.0, 0.0, BODY_HEIGHT + 0.0001)
        base.visual(mesh_from_geometry(trim, f"fabric_trim_{i}"), material=satin_black, name=f"fabric_trim_{i}")

        cloth_profile = _wing_profile(angle, center_radius=0.089, width=0.064, length=0.080)
        cloth = ExtrudeGeometry.from_z0(cloth_profile, 0.0016)
        cloth.translate(0.0, 0.0, BODY_HEIGHT + 0.0021)
        base.visual(mesh_from_geometry(cloth, f"fabric_wing_{i}"), material=fabric, name=f"fabric_wing_{i}")

        # Fine cross-hatch weave, clipped to the pad so it reads as woven cloth
        # without spilling past the rounded wing edges.
        for j, (cx, cy, seg_len, yaw) in enumerate(
            _woven_threads(angle, center_radius=0.089, width=0.064, length=0.080)
        ):
            base.visual(
                Box((seg_len, 0.0004, 0.00022)),
                origin=Origin(xyz=(cx, cy, BODY_HEIGHT + 0.00374), rpy=(0.0, 0.0, yaw)),
                material=thread,
                name=f"fabric_thread_{i}_{j}",
            )

    # Small rubber feet under the three lobes keep the body visibly desk-low.
    for i, angle in enumerate(wing_angles):
        base.visual(
            Cylinder(radius=0.010, length=0.0025),
            origin=Origin(xyz=(0.100 * math.cos(angle), 0.100 * math.sin(angle), -0.00125)),
            material=satin_black,
            name=f"foot_{i}",
        )

    # LCD with raised bezel and simple monochrome segment marks.
    screen_bezel = ExtrudeGeometry.from_z0(_shift_profile(rounded_rect_profile(0.062, 0.037, 0.004, corner_segments=6), 0.0, 0.026), 0.0020)
    screen_bezel.translate(0.0, 0.0, DECK_TOP_Z)
    base.visual(mesh_from_geometry(screen_bezel, "lcd_bezel"), material=matte_black, name="lcd_bezel")
    base.visual(
        Box((0.052, 0.026, 0.0008)),
        origin=Origin(xyz=(0.0, 0.026, DECK_TOP_Z + 0.0024)),
        material=lcd,
        name="lcd_glass",
    )
    for j, (dy, length) in enumerate(((0.006, 0.033), (0.000, 0.026), (-0.006, 0.019))):
        base.visual(
            Box((length, 0.00115, 0.00025)),
            origin=Origin(xyz=(-0.003, 0.026 + dy, DECK_TOP_Z + 0.002925)),
            material=lcd_ink,
            name=f"lcd_segment_{j}",
        )

    # Numeric keypad: 3 columns by 4 rows, each key individually pressable.
    x_cols = (-0.020, 0.0, 0.020)
    y_rows = (-0.026, -0.039, -0.052, -0.065)
    for r, y in enumerate(y_rows):
        for c, x in enumerate(x_cols):
            _add_button(
                model,
                base,
                name=f"key_{r}_{c}",
                x=x,
                y=y,
                sx=0.014,
                sy=0.0095,
                cap_material=rubber,
                label_material=white,
            )

    # Navigation cluster between screen and numeric pad.
    nav_specs = [
        ("nav_up", 0.000, -0.004, 0.012, 0.008),
        ("nav_down", 0.000, -0.017, 0.012, 0.008),
        ("nav_left", -0.014, -0.0105, 0.010, 0.010),
        ("nav_right", 0.014, -0.0105, 0.010, 0.010),
        ("select_key", 0.000, -0.0105, 0.010, 0.010),
    ]
    for name, x, y, sx, sy in nav_specs:
        _add_button(model, base, name=name, x=x, y=y, sx=sx, sy=sy, cap_material=key_gray, label_material=white)

    # Phone call controls, colored as on real conference phones.
    _add_button(
        model,
        base,
        name="call_key",
        x=-0.032,
        y=-0.065,
        sx=0.011,
        sy=0.0095,
        cap_material=green,
        label_material=white,
        label_size=(0.0045, 0.0012),
    )
    _add_button(
        model,
        base,
        name="end_key",
        x=0.032,
        y=-0.065,
        sx=0.011,
        sy=0.0095,
        cap_material=red,
        label_material=white,
        label_size=(0.0045, 0.0012),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")

    # The three speaker wings should be radially spaced and all lie outside the central hub.
    wing_centers = []
    for i in range(3):
        aabb = ctx.part_element_world_aabb(base, elem=f"fabric_wing_{i}")
        if aabb is not None:
            mn, mx = aabb
            wing_centers.append(((mn[0] + mx[0]) / 2.0, (mn[1] + mx[1]) / 2.0))
    ctx.check(
        "three fabric wings present",
        len(wing_centers) == 3 and all(math.hypot(x, y) > 0.060 for x, y in wing_centers),
        details=f"wing_centers={wing_centers}",
    )
    if len(wing_centers) == 3:
        angles = sorted((math.atan2(y, x) + 2.0 * math.pi) % (2.0 * math.pi) for x, y in wing_centers)
        gaps = [angles[(i + 1) % 3] - angles[i] for i in range(2)] + [angles[0] + 2.0 * math.pi - angles[-1]]
        ctx.check(
            "speaker wings spaced about 120 degrees",
            all(abs(g - 2.0 * math.pi / 3.0) < 0.18 for g in gaps),
            details=f"angles={angles}, gaps={gaps}",
        )

    # LCD must sit above the keypad in the central control area.
    lcd_aabb = ctx.part_element_world_aabb(base, elem="lcd_glass")
    key_aabb = ctx.part_world_aabb(object_model.get_part("key_0_1"))
    ok = False
    if lcd_aabb is not None and key_aabb is not None:
        lcd_cy = (lcd_aabb[0][1] + lcd_aabb[1][1]) / 2.0
        key_cy = (key_aabb[0][1] + key_aabb[1][1]) / 2.0
        ok = lcd_cy > key_cy + 0.040
    ctx.check("lcd is above numeric keypad", ok, details=f"lcd_aabb={lcd_aabb}, key_aabb={key_aabb}")

    # Button stems make the separate movable keys physically seated on the deck at rest.
    ctx.expect_contact(
        object_model.get_part("key_1_1"),
        base,
        elem_a="stem",
        elem_b="control_deck",
        contact_tol=0.00005,
        name="key stem seats on control deck",
    )

    key = object_model.get_part("key_1_1")
    joint = object_model.get_articulation("base_to_key_1_1")
    rest_pos = ctx.part_world_position(key)
    with ctx.pose({joint: BUTTON_TRAVEL}):
        pressed_pos = ctx.part_world_position(key)
    ctx.check(
        "numeric key presses downward",
        rest_pos is not None and pressed_pos is not None and pressed_pos[2] < rest_pos[2] - 0.0008,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )

    return ctx.report()


object_model = build_object_model()
