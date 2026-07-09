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
)


# ---------------------------------------------------------------------------
# Molded hexagonal desk body: a corner-truncated triangle.
# Main edges face 30/150/270 degrees (apothem H1); corner-cut edges face
# 90/210/330 degrees (apothem H2).
# ---------------------------------------------------------------------------
BODY_H1 = 0.100
BODY_H2 = 0.125
BODY_TOP = 0.026

# Perimeter speaker grille band dimensions
BAND_THICKNESS = 0.006
BAND_HEIGHT = 0.025          # total height including embedded portion
BAND_BOTTOM_OFFSET = 0.005   # how far below BODY_TOP the band starts (for embed)
BAND_EMBED = 0.0015          # radial embed into hex side face (>=1mm)

# Body top scale (matches the body loft's top ring)
BODY_TOP_SCALE = 0.94

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


# ---------------------------------------------------------------------------
# Hex geometry helpers
# ---------------------------------------------------------------------------

def _hex_edge_params() -> list[tuple[float, float]]:
    """Return (normal_angle, apothem) for the 6 hex edges in CCW order."""
    params: list[tuple[float, float]] = []
    for k in range(6):
        if k % 2 == 0:
            beta = math.pi / 6.0 + (k // 2) * 2.0 * math.pi / 3.0
            h = BODY_H1
        else:
            beta = math.pi / 2.0 + (k // 2) * 2.0 * math.pi / 3.0
            h = BODY_H2
        params.append((beta, h))
    return params


def _hex_vertices() -> list[tuple[float, float]]:
    """Compute the 6 vertices of the hexagonal body planform."""
    edges = _hex_edge_params()
    verts: list[tuple[float, float]] = []
    for i in range(6):
        j = (i + 1) % 6
        bi, hi = edges[i]
        bj, hj = edges[j]
        det = math.sin(bj - bi)
        x = (hi * math.sin(bj) - hj * math.sin(bi)) / det
        y = (hj * math.cos(bi) - hi * math.cos(bj)) / det
        verts.append((x, y))
    return verts


HEX_EDGES = _hex_edge_params()
HEX_VERTICES = _hex_vertices()


def _body_radius_sharp(theta: float) -> float:
    """Support radius of the sharp hexagonal body planform."""
    candidates: list[float] = []
    for k in range(3):
        beta = math.pi / 6.0 + 2.0 * math.pi * k / 3.0
        c = math.cos(theta - beta)
        if c > 1e-6:
            candidates.append(BODY_H1 / c)
        alpha = math.pi / 2.0 + 2.0 * math.pi * k / 3.0
        c = math.cos(theta - alpha)
        if c > 1e-6:
            candidates.append(BODY_H2 / c)
    return min(candidates)


def _body_profile(samples: int = 180, smooth: int = 4) -> list[tuple[float, float]]:
    """Hexagonal body planform with softly rounded corners."""
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


def _panel_point(u: float, v: float) -> tuple[float, float, float]:
    """World position on the tilted console surface (u across, v up-slope)."""
    return (
        PANEL_TOP_CENTER[0] + u,
        PANEL_TOP_CENTER[1] + v * PANEL_UP[1],
        PANEL_TOP_CENTER[2] + v * PANEL_UP[2],
    )


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

    # --- Materials (two-tone: light top plate + dark graphite band) ---
    matte_black = model.material("matte_black_shell", rgba=(0.010, 0.010, 0.012, 1.0))
    satin_black = model.material("satin_black_trim", rgba=(0.030, 0.030, 0.033, 1.0))
    top_plate_mat = model.material("light_top_plate", rgba=(0.82, 0.82, 0.80, 1.0))
    band_graphite = model.material("dark_graphite_band", rgba=(0.10, 0.10, 0.11, 1.0))
    grille_slot = model.material("grille_slot_dark", rgba=(0.020, 0.020, 0.022, 1.0))
    led_green = model.material("status_led_green", rgba=(0.16, 0.86, 0.30, 1.0))
    rubber = model.material("dark_rubber_key", rgba=(0.105, 0.105, 0.110, 1.0))
    key_gray = model.material("graphite_key", rgba=(0.145, 0.145, 0.150, 1.0))
    legend_white = model.material("pale_key_legend", rgba=(0.82, 0.84, 0.82, 1.0))
    legend_red = model.material("mute_legend_red", rgba=(0.62, 0.06, 0.05, 1.0))
    lcd_green = model.material("backlit_lcd_green", rgba=(0.52, 0.76, 0.44, 1.0))
    lcd_ink = model.material("lcd_dark_segments", rgba=(0.030, 0.060, 0.035, 1.0))
    badge_silver = model.material("logo_badge_silver", rgba=(0.70, 0.71, 0.72, 1.0))

    base = model.part("base")

    # --- Low molded hexagonal desk body (corner-truncated triangle) ---
    plan = _body_profile()
    body_geom = LoftGeometry(
        [
            _scaled_loop(plan, 0.97, 0.000),
            _scaled_loop(plan, 1.00, 0.008),
            _scaled_loop(plan, BODY_TOP_SCALE, BODY_TOP),
        ],
        cap=True,
        closed=True,
    )
    base.visual(mesh_from_geometry(body_geom, "hex_body"), material=matte_black, name="hex_body")

    # --- Flat top plate capping where dome was (two-tone: light color) ---
    top_ring_bottom = [(v[0] * 0.92, v[1] * 0.92, BODY_TOP - 0.0005) for v in HEX_VERTICES]
    top_ring_top = [(v[0] * 0.92, v[1] * 0.92, BODY_TOP + 0.002) for v in HEX_VERTICES]
    top_plate_geom = LoftGeometry([top_ring_bottom, top_ring_top], cap=True, closed=True)
    base.visual(
        mesh_from_geometry(top_plate_geom, "top_plate"),
        material=top_plate_mat,
        name="top_plate",
    )

    # --- Perimeter speaker grille band: 6 straight segments along hex edges ---
    band_z_bottom = BODY_TOP - BAND_BOTTOM_OFFSET
    band_z_center = band_z_bottom + BAND_HEIGHT / 2.0

    for i in range(6):
        beta_i, h_i = HEX_EDGES[i]
        v_start = HEX_VERTICES[(i - 1) % 6]
        v_end = HEX_VERTICES[i]

        # Edge geometry at body-top scale
        edge_len = BODY_TOP_SCALE * math.hypot(v_end[0] - v_start[0], v_end[1] - v_start[1])

        # Band inner face at body-top perimeter minus embed depth
        inner_d = BODY_TOP_SCALE * h_i - BAND_EMBED
        center_d = inner_d + BAND_THICKNESS / 2.0
        cx = center_d * math.cos(beta_i)
        cy = center_d * math.sin(beta_i)

        # Tangent angle rotates box X along the edge direction
        tangent_angle = beta_i + math.pi / 2.0

        base.visual(
            Box((edge_len, BAND_THICKNESS, BAND_HEIGHT)),
            origin=Origin(xyz=(cx, cy, band_z_center), rpy=(0.0, 0.0, tangent_angle)),
            material=band_graphite,
            name=f"band_segment_{i}",
        )

        # Horizontal grille slot marks on the outer face (2 per segment)
        outer_d = center_d + BAND_THICKNESS / 2.0 + 0.0003
        slot_len = edge_len * 0.55
        slot_thick = 0.0008
        slot_h = 0.002
        for si, z_frac in enumerate((0.35, 0.65)):
            sz = band_z_bottom + z_frac * BAND_HEIGHT
            base.visual(
                Box((slot_len, slot_thick, slot_h)),
                origin=Origin(
                    xyz=(outer_d * math.cos(beta_i), outer_d * math.sin(beta_i), sz),
                    rpy=(0.0, 0.0, tangent_angle),
                ),
                material=grille_slot,
                name=f"band_slot_{i}_{si}",
            )

    # --- Three green status LEDs atop alternating band segments (edges 1, 3, 5) ---
    for i, edge_idx in enumerate((1, 3, 5)):
        beta_led, h_led = HEX_EDGES[edge_idx]
        led_d = BODY_TOP_SCALE * h_led + 0.002
        led_z = band_z_bottom + BAND_HEIGHT + 0.002
        base.visual(
            Sphere(radius=0.0032),
            origin=Origin(xyz=(led_d * math.cos(beta_led), led_d * math.sin(beta_led), led_z)),
            material=led_green,
            name=f"status_led_{i}",
        )

    # --- Silver maker badge on the front band segment (edge 4, beta=270) ---
    front_beta, front_h = HEX_EDGES[4]
    # Place badge center just proud of the band outer face (thin axis = 0.0008 along normal)
    front_inner_d = BODY_TOP_SCALE * front_h - BAND_EMBED
    badge_d = front_inner_d + BAND_THICKNESS + 0.0002
    badge_z = band_z_bottom + BAND_HEIGHT * 0.45
    base.visual(
        Box((0.024, 0.006, 0.0008)),
        origin=Origin(
            xyz=(badge_d * math.cos(front_beta), badge_d * math.sin(front_beta), badge_z),
            rpy=(math.pi / 2.0, 0.0, front_beta + math.pi / 2.0),
        ),
        material=badge_silver,
        name="logo_badge",
    )

    # --- Angled control console spanning the front valley ---
    base.visual(
        Box(PANEL_SIZE),
        origin=Origin(xyz=PANEL_CENTER, rpy=(PANEL_TILT, 0.0, 0.0)),
        material=satin_black,
        name="keypad_console",
    )

    # --- Green-backlit LCD with raised bezel on the upper console band ---
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

    # --- Rubber feet under the three lobes ---
    for i, lobe in enumerate(
        (math.pi / 2.0, math.pi / 2.0 + 2.0 * math.pi / 3.0, math.pi / 2.0 + 4.0 * math.pi / 3.0)
    ):
        base.visual(
            Cylinder(radius=0.011, length=0.003),
            origin=Origin(xyz=(0.105 * math.cos(lobe), 0.105 * math.sin(lobe), -0.0015)),
            material=satin_black,
            name=f"foot_{i}",
        )

    # --- Numeric dial pad: 3 columns by 4 rows ---
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

    # --- Function keys left of the dial pad ---
    _add_key(model, base, name="phone_key", u=-0.042, v=0.004, sx=0.013, sy=0.0062,
             cap_material=key_gray, label_material=legend_white)
    _add_key(model, base, name="redial_key", u=-0.042, v=-0.008, sx=0.013, sy=0.0062,
             cap_material=key_gray, label_material=legend_white)

    # --- Volume rocker pair and mute key on the right console band ---
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

    # --- Six band segments exist ---
    band_count = 0
    for i in range(6):
        aabb = ctx.part_element_world_aabb(base, elem=f"band_segment_{i}")
        if aabb is not None:
            band_count += 1
    ctx.check(
        "six band segments exist",
        band_count == 6,
        details=f"found {band_count} of 6 band_segment_* visuals",
    )

    # --- Each band segment contacts its hex side (Z overlap with body) ---
    body_aabb = ctx.part_element_world_aabb(base, elem="hex_body")
    for i in range(6):
        band_aabb = ctx.part_element_world_aabb(base, elem=f"band_segment_{i}")
        ok = False
        if band_aabb is not None and body_aabb is not None:
            bmn, bmx = band_aabb
            omn, omx = body_aabb
            # Band Z range must overlap body Z range (band wraps below body top)
            z_overlap = min(bmx[2], omx[2]) - max(bmn[2], omn[2])
            # Band XY must overlap body XY (band is at body perimeter)
            xy_overlap_x = min(bmx[0], omx[0]) - max(bmn[0], omn[0])
            xy_overlap_y = min(bmx[1], omx[1]) - max(bmn[1], omn[1])
            ok = z_overlap > 0.002 and xy_overlap_x > 0.0 and xy_overlap_y > 0.0
        ctx.check(
            f"band_segment_{i} contacts hex side",
            ok,
            details=f"band_aabb={band_aabb}, body_aabb={body_aabb}",
        )

    # --- No circular ring primitive: each band segment is a tall box (not a flat ring) ---
    for i in range(6):
        aabb = ctx.part_element_world_aabb(base, elem=f"band_segment_{i}")
        ok = False
        if aabb is not None:
            mn, mx = aabb
            dx = mx[0] - mn[0]
            dy = mx[1] - mn[1]
            dz = mx[2] - mn[2]
            # A rotated box segment: dz matches BAND_HEIGHT and max XY extent
            # is significantly larger than dz (elongated edge, not a flat ring).
            # A torus ring would be a single visual with dx ≈ dy >> dz.
            ok = abs(dz - BAND_HEIGHT) < 0.002 and max(dx, dy) > 2.0 * dz
        ctx.check(
            f"band_segment_{i} is box not ring",
            ok,
            details=f"dims=({dx:.4f}, {dy:.4f}, {dz:.4f})" if aabb else "no aabb",
        )

    # --- Band segments are distributed around hex perimeter (not clustered) ---
    band_centers = []
    for i in range(6):
        aabb = ctx.part_element_world_aabb(base, elem=f"band_segment_{i}")
        if aabb is not None:
            mn, mx = aabb
            band_centers.append(((mn[0] + mx[0]) / 2.0, (mn[1] + mx[1]) / 2.0))
    if len(band_centers) == 6:
        angles = sorted(math.atan2(y, x) for x, y in band_centers)
        # Should span most of 360 degrees
        span = max(angles) - min(angles)
        ctx.check(
            "band segments span hex perimeter",
            span > math.pi,
            details=f"angle_span={math.degrees(span):.1f}°",
        )

    # --- Top plate exists and is above body ---
    plate_aabb = ctx.part_element_world_aabb(base, elem="top_plate")
    ok = False
    if plate_aabb is not None:
        mn, mx = plate_aabb
        ok = mn[2] > BODY_TOP - 0.002 and mx[2] < BODY_TOP + 0.005
    ctx.check("flat top plate exists at body top", ok, details=f"plate_aabb={plate_aabb}")

    # --- Three green LEDs on band ---
    led_centers = []
    for i in range(3):
        aabb = ctx.part_element_world_aabb(base, elem=f"status_led_{i}")
        if aabb is not None:
            mn, mx = aabb
            led_centers.append(
                ((mn[0] + mx[0]) / 2.0, (mn[1] + mx[1]) / 2.0, (mn[2] + mx[2]) / 2.0)
            )
    ctx.check(
        "three status leds on band",
        len(led_centers) == 3 and all(z > BODY_TOP + 0.010 for _, _, z in led_centers),
        details=f"led_centers={led_centers}",
    )

    # --- The backlit LCD sits on the console band above (behind) the dial pad ---
    lcd_aabb = ctx.part_element_world_aabb(base, elem="lcd_glass")
    key_aabb = ctx.part_world_aabb(object_model.get_part("key_0_1"))
    ok = False
    if lcd_aabb is not None and key_aabb is not None:
        lcd_c = ((lcd_aabb[0][1] + lcd_aabb[1][1]) / 2.0, (lcd_aabb[0][2] + lcd_aabb[1][2]) / 2.0)
        key_c = ((key_aabb[0][1] + key_aabb[1][1]) / 2.0, (key_aabb[0][2] + key_aabb[1][2]) / 2.0)
        ok = lcd_c[0] > key_c[0] + 0.008 and lcd_c[1] > key_c[1] + 0.003
    ctx.check("lcd above dial pad on tilted console", ok, details=f"lcd={lcd_aabb}, key={key_aabb}")

    # --- Keys are seated on the console through their stems at rest ---
    ctx.expect_contact(
        object_model.get_part("key_1_1"),
        base,
        elem_a="stem",
        elem_b="keypad_console",
        contact_tol=0.00005,
        name="key stem seats on console",
    )

    # --- Pressing a key moves it along the tilted console normal ---
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
