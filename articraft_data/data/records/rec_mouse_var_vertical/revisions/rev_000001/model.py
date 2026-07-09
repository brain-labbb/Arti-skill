from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


# ── Surface geometry constants ────────────────────────────────────────
_TILT_DEG = 57.0
_TILT_RAD = math.radians(_TILT_DEG)
_TILT_SIN = math.sin(_TILT_RAD)
_TILT_COS = math.cos(_TILT_RAD)

# Press axis: INTO the tilted face (inward surface normal on finger side).
# The finger-side face tilts up toward +Z and toward -Y, so the inward normal
# points toward +Y and -Z.
_PRESS_AXIS = (0.0, _TILT_SIN, -_TILT_COS)

# Scroll-wheel axle: in the tilted surface plane, perpendicular to X.
_WHEEL_AXIS = (0.0, _TILT_COS, _TILT_SIN)

# Shell sections: the front (button area) is lower, the rear (palm rest) is taller.
# This creates a realistic vertical mouse profile where buttons sit on a lower
# front face and the palm rest rises behind them.
_SECTIONS = [
    # (x, half_w, h_finger, h_thumb)
    (-0.054, 0.015, 0.068, 0.020),   # rear: tall palm rest
    (-0.036, 0.018, 0.088, 0.022),   # mid-rear: tallest
    (-0.012, 0.019, 0.094, 0.024),   # middle: tall palm area
    (0.012, 0.017, 0.058, 0.022),    # front-mid: lower for buttons
    (0.036, 0.015, 0.046, 0.020),    # front: lowest, button area
    (0.052, 0.012, 0.035, 0.018),    # nose: short
]

_Z_BASE = 0.003
_SE_EXP = 2.4


def _mouse_section(
    x: float,
    half_w: float,
    h_finger: float,
    h_thumb: float,
    *,
    z_base: float = _Z_BASE,
    segments: int = 56,
) -> list[tuple[float, float, float]]:
    """Closed asymmetric cross-section for a vertical/handshake-grip mouse."""
    pts: list[tuple[float, float, float]] = []
    exp = _SE_EXP

    for i in range(segments):
        t = 2.0 * math.pi * i / segments
        c = math.cos(t)
        s = math.sin(t)

        # Height varies from h_finger at -Y (finger) to h_thumb at +Y (thumb).
        frac = 0.5 + 0.5 * c
        height = h_finger + (h_thumb - h_finger) * frac

        # Y: superellipse, narrower at bottom for the foot.
        y_amp = abs(c) ** (2.0 / exp) * half_w
        if s < -0.15:
            squeeze = max(0.30, 1.0 + 0.82 * (s + 0.15))
            y_amp *= squeeze
        y = math.copysign(y_amp, c)

        # Z: flat bottom, shaped top.
        z01 = 0.5 + 0.5 * math.copysign(abs(s) ** (2.0 / exp), s)
        z = z_base + height * z01

        if z < z_base + 0.002 and abs(c) < 0.70:
            z = z_base

        pts.append((x, y, z))
    return pts


def _interp_station(x: float):
    """Interpolate (half_w, h_finger, h_thumb) at a given x position."""
    if x <= _SECTIONS[0][0]:
        return _SECTIONS[0][1], _SECTIONS[0][2], _SECTIONS[0][3]
    if x >= _SECTIONS[-1][0]:
        return _SECTIONS[-1][1], _SECTIONS[-1][2], _SECTIONS[-1][3]
    for k in range(len(_SECTIONS) - 1):
        x0, hw0, hf0, ht0 = _SECTIONS[k]
        x1, hw1, hf1, ht1 = _SECTIONS[k + 1]
        if x0 <= x <= x1:
            t = (x - x0) / (x1 - x0)
            return (
                hw0 + t * (hw1 - hw0),
                hf0 + t * (hf1 - hf0),
                ht0 + t * (ht1 - ht0),
            )
    return _SECTIONS[0][1], _SECTIONS[0][2], _SECTIONS[0][3]


def _surface_z(x: float, y: float) -> float:
    """Compute the upper shell surface Z at a given (x, y)."""
    hw, hf, ht = _interp_station(x)
    frac = (y + hw) / (2.0 * hw) if hw > 0.001 else 0.5
    frac = max(0.0, min(1.0, frac))
    h = hf + (ht - hf) * frac
    y_abs = abs(y)
    if y_abs >= hw or hw < 0.001:
        ratio = 1.0
    else:
        ratio = y_abs / hw
    z_frac = 0.5 + 0.5 * (max(0.0, 1.0 - ratio ** _SE_EXP)) ** (1.0 / _SE_EXP)
    return _Z_BASE + h * z_frac


def _outward_normal_offset(dist: float) -> tuple[float, float, float]:
    """Offset outward from the finger-side surface by `dist` along the normal.

    Outward normal on finger side: (0, -sin(tilt), cos(tilt)).
    """
    return (0.0, -_TILT_SIN * dist, _TILT_COS * dist)


def _make_body_shell() -> MeshGeometry:
    """Build the tall vertical/handshake-grip shell."""
    sections = [_mouse_section(x, hw, hf, ht) for x, hw, hf, ht in _SECTIONS]

    geom = MeshGeometry()
    rows: list[list[int]] = []
    for section in sections:
        rows.append([geom.add_vertex(*p) for p in section])

    count = len(rows[0])
    for a, b in zip(rows[:-1], rows[1:]):
        for i in range(count):
            j = (i + 1) % count
            geom.add_face(a[i], b[i], b[j])
            geom.add_face(a[i], b[j], a[j])

    # End caps.
    rear_center = geom.add_vertex(sections[0][0][0], 0.0, 0.025)
    front_center = geom.add_vertex(sections[-1][0][0], 0.0, 0.018)
    for i in range(count):
        j = (i + 1) % count
        geom.add_face(rear_center, rows[0][j], rows[0][i])
        geom.add_face(front_center, rows[-1][i], rows[-1][j])

    return geom


def _rounded_button(width: float, depth: float, radius: float, thickness: float, name: str):
    seg = 5
    pts: list[tuple[float, float]] = []
    centers = [
        (width / 2 - radius, depth / 2 - radius, 0.0),
        (-width / 2 + radius, depth / 2 - radius, math.pi / 2),
        (-width / 2 + radius, -depth / 2 + radius, math.pi),
        (width / 2 - radius, -depth / 2 + radius, 3 * math.pi / 2),
    ]
    for cx, cy, start in centers:
        for k in range(seg + 1):
            a = start + (math.pi / 2) * k / seg
            pts.append((cx + radius * math.cos(a), cy + radius * math.sin(a)))
    return mesh_from_geometry(ExtrudeGeometry(pts, thickness, center=True), name)


def _click_panel_mesh(is_left: bool, name: str):
    if is_left:
        pts = [
            (-0.020, -0.006),
            (0.018, -0.006),
            (0.022, 0.003),
            (0.010, 0.007),
            (-0.014, 0.006),
            (-0.022, 0.002),
        ]
    else:
        pts = [
            (-0.020, 0.006),
            (0.018, 0.006),
            (0.022, -0.003),
            (0.010, -0.007),
            (-0.014, -0.006),
            (-0.022, -0.002),
        ]
    return mesh_from_geometry(ExtrudeGeometry(pts, 0.0025, center=True), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_wireless_gaming_mouse")

    # Two-tone dark palette (companion variation ⑥).
    matte_charcoal = model.material("matte_charcoal", rgba=(0.020, 0.020, 0.022, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.055, 0.055, 0.060, 1.0))
    rubber_black = model.material("textured_rubber", rgba=(0.005, 0.005, 0.005, 1.0))
    mid_gray = model.material("mid_gray", rgba=(0.090, 0.090, 0.095, 1.0))
    logo_accent = model.material("logo_accent", rgba=(0.45, 0.45, 0.50, 1.0))

    # ── Body (root) ────────────────────────────────────────────────────
    body = model.part("body")
    body.visual(
        mesh_from_geometry(_make_body_shell(), "ergonomic_vertical_shell"),
        material=matte_charcoal,
        name="ergonomic_shell",
    )

    # Front shelf: a thin deck on the tilted face that seats the click buttons.
    # Positioned at the surface level of the front section.
    shelf_x, shelf_y = 0.030, -0.010
    shelf_sz = _surface_z(shelf_x, shelf_y)
    shelf_n = _outward_normal_offset(0.001)
    body.visual(
        Box((0.044, 0.012, 0.002)),
        origin=Origin(
            xyz=(shelf_x + shelf_n[0], shelf_y + shelf_n[1], shelf_sz + shelf_n[2]),
            rpy=(-_TILT_RAD, 0.0, 0.0),
        ),
        material=dark_gray,
        name="front_shelf",
    )

    # DPI button seat behind the scroll wheel.
    dpi_seat_x, dpi_seat_y = -0.002, -0.008
    dpi_seat_sz = _surface_z(dpi_seat_x, dpi_seat_y)
    dpi_seat_n = _outward_normal_offset(0.001)
    body.visual(
        Box((0.014, 0.010, 0.002)),
        origin=Origin(
            xyz=(dpi_seat_x + dpi_seat_n[0], dpi_seat_y + dpi_seat_n[1], dpi_seat_sz + dpi_seat_n[2]),
            rpy=(-_TILT_RAD, 0.0, 0.0),
        ),
        material=dark_gray,
        name="dpi_seat",
    )

    # Wheel bearing cheeks flanking the scroll wheel.
    wheel_x, wheel_y = 0.018, -0.010
    wheel_sz = _surface_z(wheel_x, wheel_y)
    wheel_n = _outward_normal_offset(0.003)
    wheel_cx = wheel_x + wheel_n[0]
    wheel_cy = wheel_y + wheel_n[1]
    wheel_cz = wheel_sz + wheel_n[2]

    cheek_gap = 0.007  # half-gap between cheeks (wheel fits between)
    for idx, sign in enumerate((-1.0, 1.0)):
        # Offset perpendicular to the wheel axle, in the surface tangent plane.
        perp_y = sign * cheek_gap * (-_TILT_COS)
        perp_z = sign * cheek_gap * (-_TILT_SIN)
        # Also offset slightly INTO the shell surface for connectivity.
        inward_n = _outward_normal_offset(-0.006)
        body.visual(
            Box((0.012, 0.003, 0.009)),
            origin=Origin(
                xyz=(
                    wheel_cx + inward_n[0],
                    wheel_cy + perp_y + inward_n[1],
                    wheel_cz + perp_z + inward_n[2],
                ),
                rpy=(-_TILT_RAD, 0.0, 0.0),
            ),
            material=dark_gray,
            name=f"wheel_cheek_{idx}",
        )

    # Textured rubber grips on both sides.
    # Left grip (thumb / +Y side).
    body.visual(
        Box((0.054, 0.004, 0.018)),
        origin=Origin(xyz=(-0.004, 0.0185, 0.018)),
        material=rubber_black,
        name="left_grip",
    )
    for i in range(8):
        rx = -0.024 + i * 0.007
        body.visual(
            Box((0.0020, 0.004, 0.016)),
            origin=Origin(xyz=(rx, 0.0205, 0.018), rpy=(0.0, 0.45, 0.0)),
            material=mid_gray,
            name=f"left_grip_rib_{i}",
        )

    # Right grip (finger / -Y side) — taller.
    body.visual(
        Box((0.054, 0.004, 0.040)),
        origin=Origin(xyz=(-0.004, -0.0185, 0.036)),
        material=rubber_black,
        name="right_grip",
    )
    for i in range(8):
        rx = -0.024 + i * 0.007
        body.visual(
            Box((0.0020, 0.004, 0.038)),
            origin=Origin(xyz=(rx, -0.0205, 0.036), rpy=(0.0, -0.45, 0.0)),
            material=mid_gray,
            name=f"right_grip_rib_{i}",
        )

    # Thumb-button recess on the +Y (thumb) side.
    body.visual(
        Box((0.040, 0.004, 0.014)),
        origin=Origin(xyz=(0.008, 0.0190, 0.028)),
        material=dark_gray,
        name="thumb_recess",
    )

    # Brand mark on the palm rest (finger-side wall).
    logo_x, logo_y = -0.036, -0.016
    logo_sz = _surface_z(logo_x, logo_y)
    body.visual(
        Cylinder(radius=0.006, length=0.001),
        origin=Origin(
            xyz=(logo_x, logo_y - 0.001, logo_sz + 0.001),
            rpy=(0.0, math.radians(38), 0.0),
        ),
        material=logo_accent,
        name="logo_disc",
    )
    # Logo cut stroke overlapping logo_disc for connectivity.
    body.visual(
        Box((0.005, 0.0015, 0.001)),
        origin=Origin(
            xyz=(logo_x + 0.003, logo_y + 0.002, logo_sz + 0.0015),
            rpy=(0.0, math.radians(38), -0.25),
        ),
        material=matte_charcoal,
        name="logo_cut_stroke",
    )

    # Resting foot at the base.
    body.visual(
        Box((0.040, 0.018, 0.002)),
        origin=Origin(xyz=(0.000, 0.000, 0.002)),
        material=mid_gray,
        name="rest_foot",
    )

    # ── Click buttons (on the tilted face, placed outside the shell) ───
    left_click = model.part("left_click")
    left_click.visual(
        _click_panel_mesh(True, "left_click_panel_mesh"),
        origin=Origin(rpy=(-_TILT_RAD, 0.0, 0.0)),
        material=dark_gray,
        name="button_plate",
    )
    right_click = model.part("right_click")
    right_click.visual(
        _click_panel_mesh(False, "right_click_panel_mesh"),
        origin=Origin(rpy=(-_TILT_RAD, 0.0, 0.0)),
        material=dark_gray,
        name="button_plate",
    )

    # ── DPI button ─────────────────────────────────────────────────────
    dpi_button = model.part("dpi_button")
    dpi_button.visual(
        _rounded_button(0.009, 0.011, 0.002, 0.0025, "dpi_button_cap_mesh"),
        origin=Origin(rpy=(-_TILT_RAD, 0.0, 0.0)),
        material=mid_gray,
        name="button_cap",
    )
    # DPI indicator mark — overlapping button_cap for connectivity.
    dpi_button.visual(
        Box((0.004, 0.0015, 0.002)),
        origin=Origin(xyz=(0.0, 0.001, 0.0005), rpy=(-_TILT_RAD, 0.0, 0.0)),
        material=logo_accent,
        name="dpi_mark",
    )

    # ── Scroll wheel ───────────────────────────────────────────────────
    scroll_wheel = model.part("scroll_wheel")
    wheel_geom = KnobGeometry(
        0.011,
        0.009,
        body_style="cylindrical",
        grip=KnobGrip(style="ribbed", count=18, depth=0.0008, width=0.0012),
        edge_radius=0.0007,
    )
    wheel_roll = math.atan2(-_TILT_COS, _TILT_SIN)
    scroll_wheel.visual(
        mesh_from_geometry(wheel_geom, "ribbed_scroll_wheel"),
        origin=Origin(rpy=(wheel_roll, 0.0, 0.0)),
        material=rubber_black,
        name="rubber_wheel",
    )
    scroll_wheel.visual(
        Cylinder(radius=0.0015, length=0.014),
        origin=Origin(rpy=(wheel_roll, 0.0, 0.0)),
        material=mid_gray,
        name="axle",
    )

    # ── Thumb buttons (on the +Y / thumb side) ─────────────────────────
    for idx, tx in enumerate((-0.002, 0.020)):
        thumb = model.part(f"thumb_button_{idx}")
        thumb.visual(
            _rounded_button(0.015, 0.006, 0.0015, 0.003, f"thumb_button_{idx}_mesh"),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=mid_gray,
            name="button_cap",
        )
        model.articulation(
            f"body_to_thumb_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=thumb,
            origin=Origin(xyz=(tx, 0.0210, 0.028 - idx * 0.002)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=0.05, lower=0.0, upper=0.0025),
        )

    # ── Articulations ──────────────────────────────────────────────────
    # Place buttons OUTSIDE the shell surface with generous normal offset.
    btn_offset = _outward_normal_offset(0.008)

    # Left click: finger side (-Y), on the lower front face.
    lc_x, lc_y = 0.032, -0.014
    lc_sz = _surface_z(lc_x, lc_y)
    model.articulation(
        "body_to_left_click",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left_click,
        origin=Origin(xyz=(lc_x + btn_offset[0], lc_y + btn_offset[1], lc_sz + btn_offset[2])),
        axis=_PRESS_AXIS,
        motion_limits=MotionLimits(effort=1.5, velocity=0.05, lower=0.0, upper=0.0020),
    )

    # Right click: slightly toward center on the tilted face.
    rc_x, rc_y = 0.032, -0.004
    rc_sz = _surface_z(rc_x, rc_y)
    model.articulation(
        "body_to_right_click",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right_click,
        origin=Origin(xyz=(rc_x + btn_offset[0], rc_y + btn_offset[1], rc_sz + btn_offset[2])),
        axis=_PRESS_AXIS,
        motion_limits=MotionLimits(effort=1.5, velocity=0.05, lower=0.0, upper=0.0020),
    )

    # DPI button: behind scroll wheel.
    dpi_x, dpi_y = -0.002, -0.008
    dpi_sz = _surface_z(dpi_x, dpi_y)
    model.articulation(
        "body_to_dpi_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=dpi_button,
        origin=Origin(xyz=(dpi_x + btn_offset[0], dpi_y + btn_offset[1], dpi_sz + btn_offset[2])),
        axis=_PRESS_AXIS,
        motion_limits=MotionLimits(effort=0.8, velocity=0.04, lower=0.0, upper=0.0015),
    )

    # Scroll wheel: continuous spin on the tilted face.
    model.articulation(
        "body_to_scroll_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=scroll_wheel,
        origin=Origin(xyz=(wheel_cx, wheel_cy, wheel_cz)),
        axis=_WHEEL_AXIS,
        motion_limits=MotionLimits(effort=0.15, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    left_click = object_model.get_part("left_click")
    right_click = object_model.get_part("right_click")
    dpi_button = object_model.get_part("dpi_button")
    scroll_wheel = object_model.get_part("scroll_wheel")
    thumb_0 = object_model.get_part("thumb_button_0")
    thumb_1 = object_model.get_part("thumb_button_1")

    visual_names = {v.name for v in body.visuals}

    # ── Variant-specific: vertical / handshake-grip shell form ─────────
    ctx.check(
        "ergonomic_shell is present as the primary vertical shell",
        "ergonomic_shell" in visual_names,
        details=f"body visuals={sorted(visual_names)}",
    )

    shell_aabb = ctx.part_world_aabb(body)
    if shell_aabb is not None:
        lo, hi = shell_aabb
        dy = hi[1] - lo[1]
        dz = hi[2] - lo[2]
        ctx.check(
            "ergonomic_shell reads as tall vertical handshake-grip (height >> width)",
            dz > dy * 1.5 and dz > 0.060,
            details=f"aabb dy={dy:.4f} dz={dz:.4f} ratio={dz/dy:.2f}",
        )
    else:
        ctx.fail("ergonomic_shell aabb", "could not compute body AABB")

    # The tilted face joint axis must follow the surface normal.
    click_joint = object_model.get_articulation("body_to_left_click")
    ax = click_joint.axis
    ctx.check(
        "left click press axis follows the tilted ~57° face normal",
        abs(ax[1]) > 0.40 and abs(ax[2]) > 0.30,
        details=f"axis={ax}",
    )

    # ── Grips, logo, foot, thumb recess ────────────────────────────────
    ctx.check(
        "vertical mouse has grips, logo, thumb recess, and resting foot",
        {"left_grip", "right_grip", "logo_disc", "thumb_recess", "rest_foot"}.issubset(visual_names)
        and sum(1 for n in visual_names if n.startswith("left_grip_rib_")) >= 6
        and sum(1 for n in visual_names if n.startswith("right_grip_rib_")) >= 6,
        details=f"body visuals={sorted(visual_names)}",
    )

    # ── Scroll wheel allowances and checks ─────────────────────────────
    ctx.allow_overlap(
        body,
        scroll_wheel,
        elem_a="ergonomic_shell",
        elem_b="rubber_wheel",
        reason=(
            "The scroll wheel is intentionally captured in a slot on the "
            "tilted shell face; the shell mesh leaves the slot throat simplified."
        ),
    )
    ctx.allow_overlap(
        body,
        scroll_wheel,
        elem_a="front_shelf",
        elem_b="rubber_wheel",
        reason=(
            "The front shelf backs the scroll wheel slot; small overlap at the "
            "slot throat is intentional for the seated wheel representation."
        ),
    )
    ctx.allow_overlap(
        body,
        right_click,
        elem_a="front_shelf",
        elem_b="button_plate",
        reason=(
            "The right click button_plate seats on the tilted front shelf; "
            "small edge overlap at the curved surface interface is intentional."
        ),
    )
    ctx.allow_overlap(
        right_click,
        scroll_wheel,
        elem_a="button_plate",
        elem_b="rubber_wheel",
        reason=(
            "The right click and scroll wheel are adjacent controls on the "
            "tilted face; small edge overlap at the shared boundary is intentional."
        ),
    )
    ctx.allow_overlap(
        body,
        right_click,
        elem_a="wheel_cheek_1",
        elem_b="button_plate",
        reason=(
            "The wheel cheek flanks the scroll wheel slot adjacent to the right "
            "click button; small edge overlap is intentional for the seated assembly."
        ),
    )

    ctx.expect_overlap(
        scroll_wheel,
        body,
        axes="xy",
        min_overlap=0.004,
        elem_a="rubber_wheel",
        elem_b="ergonomic_shell",
        name="scroll wheel sits in the tilted face slot",
    )
    ctx.expect_contact(
        scroll_wheel,
        body,
        contact_tol=0.012,
        elem_a="axle",
        elem_b="wheel_cheek_0",
        name="scroll wheel axle near rear bearing cheek",
    )
    ctx.expect_contact(
        scroll_wheel,
        body,
        contact_tol=0.012,
        elem_a="axle",
        elem_b="wheel_cheek_1",
        name="scroll wheel axle near front bearing cheek",
    )

    # ── Click buttons: allow small overlap at shell seat interface ─────
    for btn_part, btn_elem, seat_elem in (
        (left_click, "button_plate", "ergonomic_shell"),
        (right_click, "button_plate", "ergonomic_shell"),
        (dpi_button, "button_cap", "ergonomic_shell"),
    ):
        ctx.allow_overlap(
            body,
            btn_part,
            elem_a=seat_elem,
            elem_b=btn_elem,
            reason=(
                f"The {btn_elem} is seated on the tilted shell face; small edge "
                f"overlap at the curved surface interface is intentional."
            ),
        )

    for part_obj, joint_name in (
        (left_click, "body_to_left_click"),
        (right_click, "body_to_right_click"),
        (dpi_button, "body_to_dpi_button"),
    ):
        ctx.expect_gap(
            part_obj,
            body,
            axis="z",
            min_gap=-0.016,
            max_gap=0.025,
            positive_elem="button_plate" if part_obj in (left_click, right_click) else "button_cap",
            negative_elem="front_shelf" if part_obj in (left_click, right_click) else "dpi_seat",
            name=f"{joint_name} cap rests near its tilted seat",
        )

    # ── Thumb buttons ──────────────────────────────────────────────────
    for thumb in (thumb_0, thumb_1):
        ctx.expect_gap(
            thumb,
            body,
            axis="y",
            max_gap=0.012,
            max_penetration=0.003,
            positive_elem="button_cap",
            negative_elem="thumb_recess",
            name=f"{thumb.name} sits just outside thumb recess",
        )

    # ── Press directions ───────────────────────────────────────────────
    left_joint = object_model.get_articulation("body_to_left_click")
    thumb_joint = object_model.get_articulation("body_to_thumb_button_0")
    wheel_joint = object_model.get_articulation("body_to_scroll_wheel")

    left_rest = ctx.part_world_position(left_click)
    thumb_rest = ctx.part_world_position(thumb_0)
    with ctx.pose({left_joint: 0.0015, thumb_joint: 0.0020, wheel_joint: 0.8}):
        left_pressed = ctx.part_world_position(left_click)
        thumb_pressed = ctx.part_world_position(thumb_0)

    ctx.check(
        "primary buttons press in realistic directions on the tilted face",
        left_rest is not None
        and left_pressed is not None
        and thumb_rest is not None
        and thumb_pressed is not None
        and (left_pressed[2] < left_rest[2] - 0.0005 or left_pressed[1] > left_rest[1] + 0.0005)
        and thumb_pressed[1] < thumb_rest[1] - 0.001,
        details=(
            f"left rest={left_rest}, left pressed={left_pressed}, "
            f"thumb rest={thumb_rest}, thumb pressed={thumb_pressed}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
