from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LoftGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


# Molded hexagonal desk body: a corner-truncated triangle, as in the reference.
# Main edges face 30/150/270 degrees (keypad on the front edge); the three
# corner-cut edges face 90/210/330 degrees, straight at the grille corners.
BODY_H1 = 0.100  # apothem of the three main edges
BODY_H2 = 0.125  # apothem of the three corner-cut edges
BODY_TOP = 0.026

# Domed perforated speaker grille, pushed slightly toward the rear lobe.
# Its planform is a CONVEX rounded triangle (flat edge facing the keypad, one
# corner rearward) that blends to a circle at the crown, as in the reference.
GRILLE_CENTER_Y = 0.010
GRILLE_RV = 0.135  # triangle vertex radius (corners at 90/210/330 degrees)
GRILLE_RC = 0.030  # corner rounding radius
DOME_BASE = 0.024
DOME_H = 0.046

# Angled control console on the front valley.
PANEL_TILT = 0.35
PANEL_CENTER = (0.0, -0.069, 0.024)
PANEL_SIZE = (0.126, 0.052, 0.026)
PANEL_NORMAL = (0.0, -math.sin(PANEL_TILT), math.cos(PANEL_TILT))
PANEL_UP = (0.0, math.cos(PANEL_TILT), math.sin(PANEL_TILT))
PANEL_TOP_CENTER = (
    PANEL_CENTER[0] + PANEL_NORMAL[0] * PANEL_SIZE[2] / 2.0,
    PANEL_CENTER[1] + PANEL_NORMAL[1] * PANEL_SIZE[2] / 2.0,
    PANEL_CENTER[2] + PANEL_NORMAL[2] * PANEL_SIZE[2] / 2.0,
)

KEY_STEM_H = 0.0012
KEY_CAP_H = 0.0030
KEY_TRAVEL = 0.0011

PERF_DOT_R = 0.0011


def _body_radius_sharp(theta: float) -> float:
    """Support radius of the sharp hexagonal (corner-truncated triangle) body planform."""
    candidates: list[float] = []
    for k in range(3):
        beta = math.pi / 6.0 + 2.0 * math.pi * k / 3.0  # main edges at 30/150/270
        c = math.cos(theta - beta)
        if c > 1e-6:
            candidates.append(BODY_H1 / c)
        alpha = math.pi / 2.0 + 2.0 * math.pi * k / 3.0  # corner cuts at 90/210/330
        c = math.cos(theta - alpha)
        if c > 1e-6:
            candidates.append(BODY_H2 / c)
    return min(candidates)


def _body_profile(samples: int = 180, smooth: int = 4) -> list[tuple[float, float]]:
    """Hexagonal body planform with softly rounded corners (radial moving average)."""
    radii = [_body_radius_sharp(2.0 * math.pi * i / samples) for i in range(samples)]
    pts: list[tuple[float, float]] = []
    for i in range(samples):
        r = sum(radii[(i + j) % samples] for j in range(-smooth, smooth + 1)) / (2 * smooth + 1)
        theta = 2.0 * math.pi * i / samples
        pts.append((r * math.cos(theta), r * math.sin(theta)))
    return pts


def _scaled_loop(
    profile: list[tuple[float, float]],
    scale: float,
    z: float,
    cx: float = 0.0,
    cy: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(cx + x * scale, cy + y * scale, z) for x, y in profile]


def _grille_tri_radius(theta: float) -> float:
    """Boundary radius of the convex rounded-triangle grille planform at angle theta."""
    corner_d = GRILLE_RV - 2.0 * GRILLE_RC
    candidates: list[float] = []
    for k in range(3):
        alpha = math.pi / 2.0 + 2.0 * math.pi * k / 3.0
        # Straight edge opposite each corner pair (edge normals at 30/150/270 degrees).
        beta = alpha + math.pi / 3.0
        c = math.cos(theta - beta)
        if c > 1e-6:
            candidates.append((GRILLE_RV / 2.0) / c)
        # Rounded corner arc (only the near side of each corner circle).
        s = corner_d * math.sin(theta - alpha)
        if abs(s) <= GRILLE_RC and math.cos(theta - alpha) > 0.0:
            val = corner_d * math.cos(theta - alpha) + math.sqrt(GRILLE_RC**2 - s**2)
            if val > 0.0:
                candidates.append(val)
    return min(candidates)


_GRILLE_MEAN_R = sum(_grille_tri_radius(2.0 * math.pi * i / 90.0) for i in range(90)) / 90.0


def _grille_radius(theta: float, f: float) -> float:
    """Grille boundary radius at height fraction f: triangular skirt, circular crown."""
    w = f**1.2
    return w * _grille_tri_radius(theta) + (1.0 - w) * _GRILLE_MEAN_R


def _dome_z(f: float) -> float:
    """Height of the grille dome skin at planform fraction f (0=center, 1=skirt).

    Superellipse height profile: flat crown, steep skirt, like the molded grille.
    """
    f = min(max(f, 0.0), 1.0)
    return DOME_BASE + DOME_H * (1.0 - f**2.8) ** (1.0 / 1.9)


def _panel_point(u: float, v: float) -> tuple[float, float, float]:
    """World position on the tilted console surface (u across, v up-slope)."""
    return (
        PANEL_TOP_CENTER[0] + u,
        PANEL_TOP_CENTER[1] + v * PANEL_UP[1],
        PANEL_TOP_CENTER[2] + v * PANEL_UP[2],
    )


DOME_SAMPLES = 120
DOME_FS = [1.0, 0.98, 0.95, 0.90, 0.84, 0.76, 0.66, 0.54, 0.40, 0.26, 0.14, 0.06]
DOT_RING_FS = [0.95, 0.90, 0.84, 0.76, 0.66, 0.54, 0.40, 0.26, 0.14]


def _dome_vertex(i: int, f: float) -> tuple[float, float, float]:
    """Exact loft-ring vertex i of the dome at height fraction f."""
    theta = 2.0 * math.pi * i / DOME_SAMPLES
    rho = f * _grille_radius(theta, f)
    return (rho * math.cos(theta), GRILLE_CENTER_Y + rho * math.sin(theta), _dome_z(f))


def _perforation_dots() -> list[tuple[float, float, float]]:
    """Perforation holes seated exactly on loft-ring vertices of the dome mesh."""
    dots: list[tuple[float, float, float]] = []
    for k, f in enumerate(DOT_RING_FS):
        mean_r = f * _GRILLE_MEAN_R
        n_target = max(8, int(2.0 * math.pi * mean_r / 0.0075))
        step = max(1, round(DOME_SAMPLES / n_target))
        start = (k % 2) * (step // 2)
        for i in range(start, DOME_SAMPLES, step):
            x, y, z = _dome_vertex(i, f)
            dots.append((x, y, z - 0.0005))
    return dots


PERF_DOTS = _perforation_dots()


def _add_key(
    model: ArticulatedObject,
    base,
    *,
    name: str,
    u: float,
    v: float,
    sx: float,
    sy: float,
    cap_material: Material,
    label_material: Material,
    label_size: tuple[float, float] = (0.0042, 0.0011),
) -> None:
    """Pressable elastomer key seated on the tilted console, pressing along its normal."""
    key = model.part(name)
    key.visual(
        Cylinder(radius=min(sx, sy) * 0.12, length=KEY_STEM_H),
        origin=Origin(xyz=(0.0, 0.0, KEY_STEM_H / 2.0)),
        material=cap_material,
        name="stem",
    )
    key.visual(
        Box((sx, sy, KEY_CAP_H)),
        origin=Origin(xyz=(0.0, 0.0, KEY_STEM_H + KEY_CAP_H / 2.0)),
        material=cap_material,
        name="cap",
    )
    key.visual(
        Box((label_size[0], label_size[1], 0.00022)),
        origin=Origin(xyz=(0.0, 0.0, KEY_STEM_H + KEY_CAP_H + 0.00011)),
        material=label_material,
        name="legend",
    )
    model.articulation(
        f"base_to_{name}",
        ArticulationType.PRISMATIC,
        parent=base,
        child=key,
        origin=Origin(xyz=_panel_point(u, v), rpy=(PANEL_TILT, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=0.04, lower=0.0, upper=KEY_TRAVEL),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="black_hexagonal_conference_speakerphone")

    matte_black = model.material("matte_black_shell", rgba=(0.010, 0.010, 0.012, 1.0))
    satin_black = model.material("satin_black_trim", rgba=(0.030, 0.030, 0.033, 1.0))
    grille_gray = model.material("grille_charcoal", rgba=(0.060, 0.060, 0.064, 1.0))
    hole_black = model.material("perforation_hole", rgba=(0.006, 0.006, 0.007, 1.0))
    led_green = model.material("status_led_green", rgba=(0.16, 0.86, 0.30, 1.0))
    rubber = model.material("dark_rubber_key", rgba=(0.105, 0.105, 0.110, 1.0))
    key_gray = model.material("graphite_key", rgba=(0.145, 0.145, 0.150, 1.0))
    legend_white = model.material("pale_key_legend", rgba=(0.82, 0.84, 0.82, 1.0))
    legend_red = model.material("mute_legend_red", rgba=(0.62, 0.06, 0.05, 1.0))
    lcd_green = model.material("backlit_lcd_green", rgba=(0.52, 0.76, 0.44, 1.0))
    lcd_ink = model.material("lcd_dark_segments", rgba=(0.030, 0.060, 0.035, 1.0))
    badge_silver = model.material("logo_badge_silver", rgba=(0.70, 0.71, 0.72, 1.0))

    base = model.part("base")

    # Low molded hexagonal desk body (corner-truncated triangle).
    plan = _body_profile()
    body_geom = LoftGeometry(
        [
            _scaled_loop(plan, 0.97, 0.000),
            _scaled_loop(plan, 1.00, 0.008),
            _scaled_loop(plan, 0.94, BODY_TOP),
        ],
        cap=True,
        closed=True,
    )
    base.visual(mesh_from_geometry(body_geom, "hex_body"), material=matte_black, name="hex_body")

    # Domed speaker grille lofted over the convex rounded-triangle planform,
    # blending to a circular crown, set toward the rear.
    dome_loops = [[_dome_vertex(i, f) for i in range(DOME_SAMPLES)] for f in DOME_FS]
    dome_geom = LoftGeometry(dome_loops, cap=True, closed=True)
    base.visual(mesh_from_geometry(dome_geom, "speaker_dome"), material=grille_gray, name="speaker_dome")

    # Perforation pattern: fine dark holes half-embedded in the dome skin.
    for i, (x, y, z) in enumerate(PERF_DOTS):
        base.visual(
            Sphere(radius=PERF_DOT_R),
            origin=Origin(xyz=(x, y, z)),
            material=hole_black,
            name=f"perf_dot_{i}",
        )

    # Three green status LEDs at the grille corners.
    for i, lobe in enumerate((math.pi / 2.0, math.pi / 2.0 + 2.0 * math.pi / 3.0, math.pi / 2.0 + 4.0 * math.pi / 3.0)):
        rho = 0.90 * _grille_radius(lobe, 0.90)
        base.visual(
            Sphere(radius=0.0032),
            origin=Origin(xyz=(rho * math.cos(lobe), GRILLE_CENTER_Y + rho * math.sin(lobe), _dome_z(0.90))),
            material=led_green,
            name=f"status_led_{i}",
        )

    # Small silver maker badge low on the front face of the dome, above the console.
    badge_f = 0.84  # exact loft ring, so the plate stays seated on the mesh
    badge_r = _grille_radius(3.0 * math.pi / 2.0, badge_f)
    badge_rho = badge_f * badge_r
    eps = 0.01
    badge_slope = math.atan2(_dome_z(badge_f - eps) - _dome_z(badge_f + eps), 2.0 * eps * badge_r)
    badge_n = (0.0, -math.sin(badge_slope), math.cos(badge_slope))
    base.visual(
        Box((0.024, 0.006, 0.0008)),
        origin=Origin(
            xyz=(
                0.0 + badge_n[0] * -0.0003,
                GRILLE_CENTER_Y - badge_rho + badge_n[1] * -0.0003,
                _dome_z(badge_f) + badge_n[2] * -0.0003,
            ),
            rpy=(badge_slope, 0.0, 0.0),
        ),
        material=badge_silver,
        name="logo_badge",
    )

    # Angled control console spanning the front valley, buried into the body slab.
    base.visual(
        Box(PANEL_SIZE),
        origin=Origin(xyz=PANEL_CENTER, rpy=(PANEL_TILT, 0.0, 0.0)),
        material=satin_black,
        name="keypad_console",
    )

    # Green-backlit LCD with raised bezel on the upper console band.
    bezel_pt = _panel_point(0.0, 0.017)
    base.visual(
        Box((0.056, 0.018, 0.0020)),
        origin=Origin(
            xyz=(bezel_pt[0], bezel_pt[1] + PANEL_NORMAL[1] * 0.0010, bezel_pt[2] + PANEL_NORMAL[2] * 0.0010),
            rpy=(PANEL_TILT, 0.0, 0.0),
        ),
        material=matte_black,
        name="lcd_bezel",
    )
    base.visual(
        Box((0.048, 0.012, 0.0008)),
        origin=Origin(
            xyz=(bezel_pt[0], bezel_pt[1] + PANEL_NORMAL[1] * 0.0022, bezel_pt[2] + PANEL_NORMAL[2] * 0.0022),
            rpy=(PANEL_TILT, 0.0, 0.0),
        ),
        material=lcd_green,
        name="lcd_glass",
    )
    for j, (du, length) in enumerate(((-0.010, 0.020), (0.012, 0.014))):
        base.visual(
            Box((length, 0.0016, 0.00025)),
            origin=Origin(
                xyz=(
                    bezel_pt[0] + du,
                    bezel_pt[1] + PANEL_NORMAL[1] * 0.00255,
                    bezel_pt[2] + PANEL_NORMAL[2] * 0.00255,
                ),
                rpy=(PANEL_TILT, 0.0, 0.0),
            ),
            material=lcd_ink,
            name=f"lcd_segment_{j}",
        )

    # Rubber feet under the three lobes.
    for i, lobe in enumerate((math.pi / 2.0, math.pi / 2.0 + 2.0 * math.pi / 3.0, math.pi / 2.0 + 4.0 * math.pi / 3.0)):
        base.visual(
            Cylinder(radius=0.011, length=0.003),
            origin=Origin(xyz=(0.105 * math.cos(lobe), 0.105 * math.sin(lobe), -0.0015)),
            material=satin_black,
            name=f"foot_{i}",
        )

    # Numeric dial pad: 3 columns by 4 rows, each key individually pressable.
    u_cols = (-0.016, 0.0, 0.016)
    v_rows = (0.005, -0.003, -0.011, -0.019)
    for r, v in enumerate(v_rows):
        for c, u in enumerate(u_cols):
            _add_key(
                model,
                base,
                name=f"key_{r}_{c}",
                u=u,
                v=v,
                sx=0.012,
                sy=0.0064,
                cap_material=rubber,
                label_material=legend_white,
            )

    # Function keys left of the dial pad.
    _add_key(model, base, name="phone_key", u=-0.042, v=0.004, sx=0.013, sy=0.0062,
             cap_material=key_gray, label_material=legend_white)
    _add_key(model, base, name="redial_key", u=-0.042, v=-0.008, sx=0.013, sy=0.0062,
             cap_material=key_gray, label_material=legend_white)

    # Volume rocker pair and mute key on the right console band.
    _add_key(model, base, name="volume_up_key", u=0.037, v=0.004, sx=0.013, sy=0.0062,
             cap_material=key_gray, label_material=legend_white)
    _add_key(model, base, name="volume_down_key", u=0.037, v=-0.008, sx=0.013, sy=0.0062,
             cap_material=key_gray, label_material=legend_white)
    _add_key(model, base, name="mute_key", u=0.052, v=-0.002, sx=0.012, sy=0.0090,
             cap_material=rubber, label_material=legend_red, label_size=(0.0030, 0.0030))

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")

    # The perforated dome must rise well above the console and sit toward the rear.
    dome_aabb = ctx.part_element_world_aabb(base, elem="speaker_dome")
    ok = False
    if dome_aabb is not None:
        mn, mx = dome_aabb
        cy = (mn[1] + mx[1]) / 2.0
        ok = mx[2] > 0.060 and cy > 0.012
    ctx.check("speaker dome tall and rear-set", ok, details=f"dome_aabb={dome_aabb}")

    # Three green LEDs spaced about 120 degrees around the grille corners, on the dome skin.
    led_centers = []
    for i in range(3):
        aabb = ctx.part_element_world_aabb(base, elem=f"status_led_{i}")
        if aabb is not None:
            mn, mx = aabb
            led_centers.append(
                ((mn[0] + mx[0]) / 2.0, (mn[1] + mx[1]) / 2.0 - GRILLE_CENTER_Y, (mn[2] + mx[2]) / 2.0)
            )
    ctx.check(
        "three corner status leds on dome",
        len(led_centers) == 3
        and all(math.hypot(x, y) > 0.050 and 0.030 < z < 0.055 for x, y, z in led_centers),
        details=f"led_centers={led_centers}",
    )
    if len(led_centers) == 3:
        angles = sorted((math.atan2(y, x) + 2.0 * math.pi) % (2.0 * math.pi) for x, y, _ in led_centers)
        gaps = [angles[1] - angles[0], angles[2] - angles[1], angles[0] + 2.0 * math.pi - angles[2]]
        ctx.check(
            "status leds spaced about 120 degrees",
            all(abs(g - 2.0 * math.pi / 3.0) < 0.20 for g in gaps),
            details=f"gaps={gaps}",
        )

    # Perforation pattern covers the dome densely and stays on its skin.
    sample = ctx.part_element_world_aabb(base, elem=f"perf_dot_{len(PERF_DOTS) // 2}")
    ok = len(PERF_DOTS) >= 240 and sample is not None
    if ok and sample is not None:
        zc = (sample[0][2] + sample[1][2]) / 2.0
        ok = DOME_BASE - 0.002 < zc < DOME_BASE + DOME_H
    ctx.check(
        "dense perforation on dome skin",
        ok,
        details=f"dot_count={len(PERF_DOTS)}, sample_aabb={sample}",
    )

    # The backlit LCD sits on the console band above (behind) the dial pad.
    lcd_aabb = ctx.part_element_world_aabb(base, elem="lcd_glass")
    key_aabb = ctx.part_world_aabb(object_model.get_part("key_0_1"))
    ok = False
    if lcd_aabb is not None and key_aabb is not None:
        lcd_c = ((lcd_aabb[0][1] + lcd_aabb[1][1]) / 2.0, (lcd_aabb[0][2] + lcd_aabb[1][2]) / 2.0)
        key_c = ((key_aabb[0][1] + key_aabb[1][1]) / 2.0, (key_aabb[0][2] + key_aabb[1][2]) / 2.0)
        ok = lcd_c[0] > key_c[0] + 0.008 and lcd_c[1] > key_c[1] + 0.003
    ctx.check("lcd above dial pad on tilted console", ok, details=f"lcd={lcd_aabb}, key={key_aabb}")

    # Keys are seated on the console through their stems at rest.
    ctx.expect_contact(
        object_model.get_part("key_1_1"),
        base,
        elem_a="stem",
        elem_b="keypad_console",
        contact_tol=0.00005,
        name="key stem seats on console",
    )

    # Pressing a key moves it along the tilted console normal: down and slightly rearward.
    key = object_model.get_part("key_1_1")
    joint = object_model.get_articulation("base_to_key_1_1")
    rest = ctx.part_world_position(key)
    with ctx.pose({joint: KEY_TRAVEL}):
        pressed = ctx.part_world_position(key)
    ok = (
        rest is not None
        and pressed is not None
        and pressed[2] < rest[2] - 0.0008
        and pressed[1] > rest[1] + 0.0002
    )
    ctx.check("dial key presses into tilted console", ok, details=f"rest={rest}, pressed={pressed}")

    return ctx.report()


object_model = build_object_model()
