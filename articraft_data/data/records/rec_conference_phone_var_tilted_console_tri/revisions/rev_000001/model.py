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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


BODY_HEIGHT = 0.018
BUTTON_STEM_H = 0.0012
BUTTON_CAP_H = 0.0026
BUTTON_TRAVEL = 0.0011

# Tilted wedge console parameters
RAKE_ANGLE = math.radians(15)
WEDGE_WIDTH = 0.086
WEDGE_DEPTH = 0.119
WEDGE_Y_CENTER = -0.018
WEDGE_H_FRONT = 0.004
WEDGE_H_REAR = WEDGE_H_FRONT + WEDGE_DEPTH * math.tan(RAKE_ANGLE)
EMBED = 0.001  # stem embed depth into tilted surface (>= 1mm)


def _shift_profile(profile: list[tuple[float, float]], dx: float, dy: float) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _radial_profile(samples: int = 120) -> list[tuple[float, float]]:
    """Rounded three-lobed planform with lobe centers at 90, 210, and 330 degrees."""
    pts: list[tuple[float, float]] = []
    for i in range(samples):
        theta = 2.0 * math.pi * i / samples
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
    """Fine cross-hatch weave clipped to the wing's superellipse cloth pad."""
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
        px, py = -dy, dx
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


def _surface_z(y: float) -> float:
    """Z of the tilted console top surface at world Y position."""
    y_front = WEDGE_Y_CENTER - WEDGE_DEPTH / 2.0
    return BODY_HEIGHT + WEDGE_H_FRONT + (y - y_front) * math.tan(RAKE_ANGLE)


def _on_tilted_surface(x: float, y: float, z_local: float = 0.0) -> Origin:
    """Origin on the tilted surface at (x, y) with z_local offset along the surface normal."""
    z_surf = _surface_z(y)
    return Origin(
        xyz=(
            x,
            y - z_local * math.sin(RAKE_ANGLE),
            z_surf + z_local * math.cos(RAKE_ANGLE),
        ),
        rpy=(RAKE_ANGLE, 0.0, 0.0),
    )


def _build_rounded_wedge(
    profile_2d: list[tuple[float, float]],
    y_offset: float,
    z_base: float,
    h_front: float,
    h_rear: float,
    depth: float,
) -> MeshGeometry:
    """Wedge with rounded planform: flat bottom, tilted top surface (front low, rear high)."""
    n = len(profile_2d)
    y_front = y_offset - depth / 2.0
    g = MeshGeometry()

    # Bottom ring (flat at z_base)
    for x, y in profile_2d:
        g.add_vertex(x, y + y_offset, z_base)

    # Top ring (z depends on y — tilted surface)
    for x, y in profile_2d:
        yw = y + y_offset
        t = (yw - y_front) / depth
        z_top = z_base + h_front + t * (h_rear - h_front)
        g.add_vertex(x, yw, z_top)

    # Side walls (outward normals)
    for i in range(n):
        j = (i + 1) % n
        g.add_face(i, j, j + n)
        g.add_face(i, j + n, i + n)

    # Bottom cap (normal -Z)
    for i in range(1, n - 1):
        g.add_face(0, i + 1, i)

    # Top cap (normal ~+Z tilted)
    for i in range(1, n - 1):
        g.add_face(n, n + i, n + i + 1)

    return g


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
    """Pressable key seated on the tilted console with stem embedded >=1mm into the face."""
    key = model.part(name)
    z_surf = _surface_z(y)

    # Stem embedded EMBED below the tilted surface along the local normal
    key.visual(
        Cylinder(radius=min(sx, sy) * 0.12, length=BUTTON_STEM_H),
        origin=Origin(xyz=(0.0, 0.0, -EMBED + BUTTON_STEM_H / 2.0)),
        material=cap_material,
        name="stem",
    )
    # Cap sits proud of the surface
    key.visual(
        Box((sx, sy, BUTTON_CAP_H)),
        origin=Origin(xyz=(0.0, 0.0, -EMBED + BUTTON_STEM_H + BUTTON_CAP_H / 2.0)),
        material=cap_material,
        name="cap",
    )
    key.visual(
        Box((label_size[0], label_size[1], 0.00022)),
        origin=Origin(xyz=(0.0, 0.0, -EMBED + BUTTON_STEM_H + BUTTON_CAP_H + 0.00011)),
        material=label_material,
        name="legend",
    )

    # Prismatic joint: origin on tilted surface, frame tilted so local Z = surface normal.
    # axis=(0,0,-1) in the tilted frame presses along the inward normal in world.
    model.articulation(
        f"base_to_{name}",
        ArticulationType.PRISMATIC,
        parent=base,
        child=key,
        origin=Origin(xyz=(x, y, z_surf), rpy=(RAKE_ANGLE, 0.0, 0.0)),
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
    lcd_mat = model.material("monochrome_lcd", rgba=(0.66, 0.75, 0.70, 1.0))
    lcd_ink = model.material("lcd_dark_segments", rgba=(0.025, 0.040, 0.035, 1.0))
    green = model.material("call_key_green", rgba=(0.02, 0.45, 0.18, 1.0))
    red = model.material("end_key_red", rgba=(0.52, 0.04, 0.035, 1.0))

    base = model.part("base")

    # --- Tri-star body (KEEP: unchanged) ---
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

    # Central hub (KEEP: unchanged)
    hub = ExtrudeGeometry.from_z0(superellipse_profile(0.116, 0.104, exponent=2.4, segments=72), 0.004)
    hub.translate(0.0, -0.004, BODY_HEIGHT - 0.0004)
    base.visual(mesh_from_geometry(hub, "central_hub"), material=satin_black, name="central_hub")

    # --- Tilted wedge control console (CHANGED: was flat control_deck) ---
    console_profile = rounded_rect_profile(WEDGE_WIDTH, WEDGE_DEPTH, 0.010, corner_segments=8)
    wedge_geom = _build_rounded_wedge(
        console_profile, WEDGE_Y_CENTER, BODY_HEIGHT, WEDGE_H_FRONT, WEDGE_H_REAR, WEDGE_DEPTH,
    )
    base.visual(
        mesh_from_geometry(wedge_geom, "control_console"),
        material=satin_black,
        name="control_console",
    )

    # --- Fabric speaker wings (KEEP: unchanged) ---
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

        for j, (cx, cy, seg_len, yaw) in enumerate(
            _woven_threads(angle, center_radius=0.089, width=0.064, length=0.080)
        ):
            base.visual(
                Box((seg_len, 0.0004, 0.00022)),
                origin=Origin(xyz=(cx, cy, BODY_HEIGHT + 0.00374), rpy=(0.0, 0.0, yaw)),
                material=thread,
                name=f"fabric_thread_{i}_{j}",
            )

    # Feet (KEEP: unchanged)
    for i, angle in enumerate(wing_angles):
        base.visual(
            Cylinder(radius=0.010, length=0.0025),
            origin=Origin(xyz=(0.100 * math.cos(angle), 0.100 * math.sin(angle), -0.00125)),
            material=satin_black,
            name=f"foot_{i}",
        )

    # --- LCD on tilted surface (re-seated with tilt) ---
    y_lcd = 0.026
    bezel_profile = rounded_rect_profile(0.062, 0.037, 0.004, corner_segments=6)
    screen_bezel = ExtrudeGeometry.from_z0(bezel_profile, 0.0020)
    base.visual(
        mesh_from_geometry(screen_bezel, "lcd_bezel"),
        origin=_on_tilted_surface(0.0, y_lcd, 0.0),
        material=matte_black,
        name="lcd_bezel",
    )
    base.visual(
        Box((0.052, 0.026, 0.0008)),
        origin=_on_tilted_surface(0.0, y_lcd, 0.0024),
        material=lcd_mat,
        name="lcd_glass",
    )
    for j, (dy, length) in enumerate(((0.006, 0.033), (0.000, 0.026), (-0.006, 0.019))):
        base.visual(
            Box((length, 0.00115, 0.00025)),
            origin=_on_tilted_surface(-0.003, y_lcd + dy, 0.002925),
            material=lcd_ink,
            name=f"lcd_segment_{j}",
        )

    # --- Numeric keypad on tilted surface (re-seated with tilt) ---
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

    # --- Navigation cluster on tilted surface ---
    nav_specs = [
        ("nav_up", 0.000, -0.004, 0.012, 0.008),
        ("nav_down", 0.000, -0.017, 0.012, 0.008),
        ("nav_left", -0.014, -0.0105, 0.010, 0.010),
        ("nav_right", 0.014, -0.0105, 0.010, 0.010),
        ("select_key", 0.000, -0.0105, 0.010, 0.010),
    ]
    for name, x, y, sx, sy in nav_specs:
        _add_button(model, base, name=name, x=x, y=y, sx=sx, sy=sy, cap_material=key_gray, label_material=white)

    # --- Call/end keys on tilted surface ---
    _add_button(
        model, base, name="call_key", x=-0.032, y=-0.065,
        sx=0.011, sy=0.0095, cap_material=green, label_material=white, label_size=(0.0045, 0.0012),
    )
    _add_button(
        model, base, name="end_key", x=0.032, y=-0.065,
        sx=0.011, sy=0.0095, cap_material=red, label_material=white, label_size=(0.0045, 0.0012),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")

    # --- Wing tests (unchanged) ---
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

    # --- Console tilt verification (new: proves the wedge is tilted, not flat) ---
    console_aabb = ctx.part_element_world_aabb(base, elem="control_console")
    if console_aabb is not None:
        z_range = console_aabb[1][2] - console_aabb[0][2]
        ctx.check(
            "control_console is a tilted wedge (significant Z span from front to rear)",
            z_range > 0.020,
            details=f"z_range={z_range:.4f}m (expected >0.020)",
        )

    # --- Console bottom contacts body deck (new) ---
    body_aabb = ctx.part_element_world_aabb(base, elem="tri_star_body")
    if console_aabb is not None and body_aabb is not None:
        console_bottom_z = console_aabb[0][2]
        body_top_z = body_aabb[1][2]
        ctx.check(
            "control_console bottom sits flush on tri_star_body deck",
            abs(console_bottom_z - body_top_z) < 0.002,
            details=f"console_bottom_z={console_bottom_z:.5f}, body_top_z={body_top_z:.5f}",
        )

    # --- LCD above keypad on tilted console ---
    lcd_aabb = ctx.part_element_world_aabb(base, elem="lcd_glass")
    key_aabb = ctx.part_world_aabb(object_model.get_part("key_0_1"))
    ok = False
    if lcd_aabb is not None and key_aabb is not None:
        lcd_cy = (lcd_aabb[0][1] + lcd_aabb[1][1]) / 2.0
        key_cy = (key_aabb[0][1] + key_aabb[1][1]) / 2.0
        ok = lcd_cy > key_cy + 0.025
    ctx.check("lcd is above numeric keypad on tilted console", ok, details=f"lcd_aabb={lcd_aabb}, key_aabb={key_aabb}")

    # --- Key stem embedding: allow intentional overlap for all keys ---
    all_key_names = [f"key_{r}_{c}" for r in range(4) for c in range(3)]
    all_key_names += ["nav_up", "nav_down", "nav_left", "nav_right", "select_key", "call_key", "end_key"]
    for kn in all_key_names:
        ctx.allow_overlap(
            object_model.get_part(kn),
            base,
            elem_a="stem",
            elem_b="control_console",
            reason=f"Key '{kn}' stem is embedded >=1mm into the tilted console surface for a seated press feel.",
        )

    # --- Keycap sits above the tilted face along its normal ---
    key_test = object_model.get_part("key_1_1")
    key_joint = object_model.get_articulation("base_to_key_1_1")
    key_pos = ctx.part_world_position(key_test)
    y_key = key_pos[1]
    z_surf_at_key = _surface_z(y_key)
    ctx.check(
        "key_1_1 cap sits above tilted console face along normal",
        key_pos[2] > z_surf_at_key - 0.0005,
        details=f"key_z={key_pos[2]:.5f}, surface_z_at_key_y={z_surf_at_key:.5f}",
    )

    # --- Key travel along the tilted surface normal ---
    rest_pos = ctx.part_world_position(key_test)
    with ctx.pose({key_joint: BUTTON_TRAVEL}):
        pressed_pos = ctx.part_world_position(key_test)

    dy = pressed_pos[1] - rest_pos[1]
    dz = pressed_pos[2] - rest_pos[2]
    expected_dy = BUTTON_TRAVEL * math.sin(RAKE_ANGLE)
    expected_dz = -BUTTON_TRAVEL * math.cos(RAKE_ANGLE)
    ctx.check(
        "key_1_1 presses along tilted surface normal (not world -Z)",
        abs(dy - expected_dy) < 0.0002 and abs(dz - expected_dz) < 0.0002,
        details=f"dy={dy:.6f} (exp {expected_dy:.6f}), dz={dz:.6f} (exp {expected_dz:.6f})",
    )

    return ctx.report()


object_model = build_object_model()
