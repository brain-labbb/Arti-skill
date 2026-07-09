from __future__ import annotations

# Modern full-touch SLAB smartphone, dark glossy shell.
# Forked from Nokia 3310 candybar envelope into a full-touch slab form.
# Frame: face points up (+Z). Phone is a tall candybar:
#   - Y = long axis (tall), top at +Y near the earpiece, bottom toward -Y
#   - X = width (narrow)
#   - Z = thickness; the front face is the +Z surface
# Static: dark-navy rounded monoblock shell, edge-to-edge touchscreen,
#         earpiece slit, thin dark bezel frame.
# Articulations (4): all PRISMATIC straight-in press buttons:
#   - button_0 (home): front bottom center, press -Z
#   - button_1 (power): right side upper, press -X (inward)
#   - button_2 (volume_up): left side upper, press +X (inward)
#   - button_3 (volume_down): left side lower, press +X (inward)
#
# Companion variation ⑥: set LIGHT_COLORWAY=True for white/silver glass-front
# colorway. Material swap only; no part-tree, joint, or geometry changes.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    ExtrudeGeometry,
    boolean_union,
    mesh_from_geometry,
    rounded_rect_profile,
)

# ---- overall dimensions (same candybar envelope as parent) ----
BODY_W = 0.048   # X width
BODY_H = 0.110   # Y height (tall candybar)
BODY_T = 0.022   # Z thickness
RIM_Z = 0.0105   # top of the front shell (the rim plane)

# Screen: edge-to-edge touchscreen filling nearly the whole front face
SCREEN_W = 0.043   # X: ~90% of body width
SCREEN_H = 0.086   # Y: ~78% of body height
SCREEN_CY = 0.004  # slightly above center (forehead for earpiece, chin for home)
BEZEL_BORDER = 0.0012  # thin dark bezel frame around glass

# Button press travel
PRESS_TRAVEL = 0.0008  # 0.8mm prismatic press

# Companion ⑥: material-only colorway toggle
LIGHT_COLORWAY = False


def _translated_rounded_slab(
    w: float, h: float, z0: float, t: float, r: float, name: str,
    *, x: float = 0.0, y: float = 0.0,
):
    """Rounded rectangle slab with its XY profile already in body coordinates."""
    geom = ExtrudeGeometry.from_z0(
        rounded_rect_profile(w, h, r, corner_segments=10), t, cap=True,
    )
    geom.translate(x, y, z0)
    return mesh_from_geometry(geom, name)


def _slot_stack_mesh(name: str, count: int, slot_w: float, slot_h: float, pitch: float):
    """Small separated grille slits joined by an invisible shallow spine so the
    earpiece reads as individual slots without creating floating mesh islands."""
    spine = ExtrudeGeometry.from_z0(
        rounded_rect_profile(
            slot_w * 0.40, pitch * (count - 1) + slot_h, slot_w * 0.18,
            corner_segments=4,
        ),
        0.00035, cap=True,
    )
    for i in range(count):
        y = (i - (count - 1) / 2.0) * pitch
        slit = ExtrudeGeometry.from_z0(
            rounded_rect_profile(
                slot_w, slot_h, min(slot_w, slot_h) * 0.45, corner_segments=5,
            ),
            0.00055, cap=True,
        )
        x = 0.0004 * math.sin((i - (count - 1) / 2.0) * 0.7)
        slit.translate(x, y, 0.0)
        spine.merge(slit)
    return mesh_from_geometry(spine, name)


def _body_shell_mesh():
    """Dark-navy candybar monoblock shell — smooth front face, no keypad pocket.
    Eleven stacked rounded slabs with small per-step insets give a smooth
    edge-rounding profile. Unlike the parent 3310, there is no recessed keypad
    well carved into the front face."""
    geom = None
    sections = [
        # (z0, thickness, inset)
        (-0.0110, 0.0016, 0.0030),
        (-0.0099, 0.0017, 0.0018),
        (-0.0088, 0.0019, 0.0010),
        (-0.0075, 0.0021, 0.0005),
        (-0.0060, 0.0021, 0.0002),
        (-0.0045, 0.0084, 0.0000),   # main barrel (widest)
        (0.0034, 0.0022, 0.0002),
        (0.0051, 0.0020, 0.0006),
        (0.0066, 0.0018, 0.0012),
        (0.0079, 0.0017, 0.0018),
        (0.0091, 0.0014, 0.0026),    # front rim plane (top = RIM_Z)
    ]
    for i, (z0, t, inset) in enumerate(sections):
        prof = rounded_rect_profile(
            BODY_W - 2.0 * inset,
            BODY_H - 2.0 * inset,
            0.016 - inset * 0.5,
            corner_segments=10,
        )
        slab = ExtrudeGeometry.from_z0(prof, t, cap=True)
        slab.translate(0.0, 0.0, z0)
        if geom is None:
            geom = slab
        else:
            geom = boolean_union(geom, slab)
    return mesh_from_geometry(geom, "body_shell")


def _button_cap_mesh(name: str, w: float, h: float, t: float):
    """Shared button keycap: a thin rounded-rect slab from z=0 upward (the
    outward direction from the phone face).  Two stacked layers give a beveled
    crown like a real button cap."""
    base = ExtrudeGeometry.from_z0(
        rounded_rect_profile(w, h, min(w, h) * 0.35, corner_segments=8),
        t * 0.55, cap=True,
    )
    crown = ExtrudeGeometry.from_z0(
        rounded_rect_profile(w * 0.86, h * 0.86, min(w, h) * 0.25, corner_segments=8),
        t, cap=True,
    )
    base.merge(crown)
    return mesh_from_geometry(base, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slab_phone")

    # ---- materials (companion ⑥: LIGHT_COLORWAY swaps to white/silver) ----
    if LIGHT_COLORWAY:
        shell = model.material("glossy_white_silver", rgba=(0.90, 0.91, 0.93, 1.0))
        bezel = model.material("light_bezel", rgba=(0.12, 0.12, 0.14, 1.0))
        glass = model.material("white_glass", rgba=(0.85, 0.87, 0.90, 1.0))
        earpiece_mat = model.material("earpiece_grey", rgba=(0.20, 0.20, 0.22, 1.0))
        btn_mat = model.material("silver_button", rgba=(0.78, 0.80, 0.82, 1.0))
    else:
        shell = model.material("glossy_dark_navy", rgba=(0.050, 0.070, 0.155, 1.0))
        bezel = model.material("black_display_bezel", rgba=(0.020, 0.022, 0.028, 1.0))
        glass = model.material("dark_touch_glass", rgba=(0.030, 0.035, 0.050, 1.0))
        earpiece_mat = model.material("earpiece_black", rgba=(0.006, 0.006, 0.010, 1.0))
        btn_mat = model.material("brushed_dark_button", rgba=(0.080, 0.090, 0.130, 1.0))

    # ================= BODY (root monoblock) =================
    body = model.part("body")
    body.visual(_body_shell_mesh(), material=shell, name="body_shell")

    # ---- earpiece: thin slit grille at the top of the front face ----
    # Base seats 0.4mm into the shell so the grille reads as embedded in the face.
    body.visual(
        _slot_stack_mesh("earpiece_slot_mesh", 4, 0.0030, 0.0009, 0.00135),
        origin=Origin(xyz=(0.0, 0.0510, RIM_Z - 0.0004)),
        material=earpiece_mat,
        name="earpiece_slot",
    )

    # ---- screen: dark bezel frame + edge-to-edge touchscreen glass ----
    # Bezel frame: slightly larger than glass, thin slab seated at the rim.
    body.visual(
        _translated_rounded_slab(
            SCREEN_W + 2.0 * BEZEL_BORDER,
            SCREEN_H + 2.0 * BEZEL_BORDER,
            RIM_Z - 0.0004, 0.0008, 0.004,
            "screen_frame_mesh",
            y=SCREEN_CY,
        ),
        material=bezel,
        name="screen_frame",
    )
    # Touchscreen glass: seated into the bezel frame, crown slightly proud.
    body.visual(
        Box((SCREEN_W, SCREEN_H, 0.0006)),
        origin=Origin(xyz=(0.0, SCREEN_CY, RIM_Z + 0.0004)),
        material=glass,
        name="screen_glass",
    )

    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_H, BODY_T)), mass=0.170, origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # ================= BUTTONS (for-loop, shared keycap helper) =================
    # Each button: a separate part with one keycap visual, mounted by a
    # PRISMATIC articulation whose origin is at the body contact surface.
    # Axis points inward so positive q presses the button into the phone.
    HALF_W = BODY_W / 2.0
    button_specs: list[tuple[str, float, float, float, tuple, float, float, float, tuple]] = [
        # (name,  jx,       jy,      jz,          axis,              bw,     bh,     bt,      visual_rpy)
        # Home: front bottom center, press -Z into the front face.
        # Joint origin is 0.2mm below the rim so the cap base seats into the shell.
        ("button_0", 0.0, -0.0470, RIM_Z - 0.0002, (0.0, 0.0, -1.0),  0.008,  0.008,  0.0010,  (0.0, 0.0, 0.0)),
        # Power: right side (+X), upper area, press -X inward.
        # Joint origin is 0.3mm inside the body edge so the cap seats into the shell.
        ("button_1", HALF_W - 0.0003, 0.0180, 0.0015, (-1.0, 0.0, 0.0), 0.003, 0.014, 0.0008, (0.0, math.pi / 2.0, 0.0)),
        # Volume up: left side (-X), upper, press +X inward.
        ("button_2", -(HALF_W - 0.0003), 0.0250, 0.0015, (1.0, 0.0, 0.0), 0.003, 0.010, 0.0008, (0.0, -math.pi / 2.0, 0.0)),
        # Volume down: left side (-X), below volume up, press +X inward.
        ("button_3", -(HALF_W - 0.0003), 0.0130, 0.0015, (1.0, 0.0, 0.0), 0.003, 0.010, 0.0008, (0.0, -math.pi / 2.0, 0.0)),
    ]

    for i, (name, jx, jy, jz, axis, bw, bh, bt, rpy) in enumerate(button_specs):
        btn = model.part(name)
        cap = _button_cap_mesh(f"{name}_cap", bw, bh, bt)
        # Visual is rotated so local +Z (cap outward direction) aligns with the
        # button's outward face direction on the phone body.
        btn.visual(
            cap, origin=Origin(rpy=rpy), material=btn_mat, name="keycap",
        )
        btn.inertial = Inertial.from_geometry(
            Box((bw, bh, bt)), mass=0.0005, origin=Origin(xyz=(0.0, 0.0, bt / 2.0)),
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=btn,
            origin=Origin(xyz=(jx, jy, jz)),
            axis=axis,
            motion_limits=MotionLimits(
                effort=3.0, velocity=0.05, lower=0.0, upper=PRESS_TRAVEL,
            ),
        )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")

    # ---- body retains tall candybar envelope ----
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "body is a tall candybar (Y much greater than X)",
        bext[1] > bext[0] + 0.04,
        details=f"body extents={bext}",
    )
    ctx.check(
        "body is narrow in width vs height",
        bext[1] > 2.0 * bext[0],
        details=f"body extents={bext}",
    )

    # ---- screen_glass fills nearly the whole front face (TARGET axis) ----
    screen_aabb = ctx.part_element_world_aabb(body, elem="screen_glass")
    scr_ext = _ext(screen_aabb)
    ctx.check(
        "screen_glass spans >= 85% of body width (edge-to-edge touchscreen)",
        scr_ext[0] >= BODY_W * 0.85,
        details=f"screen_w={scr_ext[0]:.4f}, body_w={BODY_W}",
    )
    ctx.check(
        "screen_glass spans >= 75% of body height (full-touch slab)",
        scr_ext[1] >= BODY_H * 0.75,
        details=f"screen_h={scr_ext[1]:.4f}, body_h={BODY_H}",
    )

    # ---- screen glass is proud of bezel frame ----
    frame_aabb = ctx.part_element_world_aabb(body, elem="screen_frame")
    ctx.check(
        "touchscreen glass tops the dark bezel",
        screen_aabb[1][2] > frame_aabb[1][2] + 0.0002,
        details=f"glass top={screen_aabb[1][2]}, bezel top={frame_aabb[1][2]}",
    )

    # ---- earpiece above screen ----
    ear_aabb = ctx.part_element_world_aabb(body, elem="earpiece_slot")
    ear_cy = (ear_aabb[0][1] + ear_aabb[1][1]) / 2.0
    ctx.check(
        "earpiece slot is above the screen top",
        ear_aabb[0][1] > screen_aabb[1][1] - 0.001,
        details=f"earpiece bottom={ear_aabb[0][1]}, screen top={screen_aabb[1][1]}",
    )

    # ---- exactly 4 PRISMATIC press buttons (no legacy keypad) ----
    press_joints = [
        a for a in object_model.articulations
        if a.articulation_type == ArticulationType.PRISMATIC
    ]
    ctx.check(
        "exactly 4 pressable buttons (home, power, vol_up, vol_down)",
        len(press_joints) == 4,
        details=f"prismatic joints={len(press_joints)}",
    )

    # ---- verify each button via for-loop checks ----
    # Semantic map: 0=home(-Z), 1=power(-X), 2=vol_up(+X), 3=vol_down(+X)
    expected_axes = [
        (0.0, 0.0, -1.0),
        (-1.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
    ]
    for i in range(4):
        name = f"button_{i}"
        btn = object_model.get_part(name)
        joint = object_model.get_articulation(f"body_to_{name}")
        ctx.check(
            f"{name} is PRISMATIC with correct travel",
            joint.articulation_type == ArticulationType.PRISMATIC
            and joint.motion_limits is not None
            and abs(joint.motion_limits.lower) < 1e-9
            and abs(joint.motion_limits.upper - PRESS_TRAVEL) < 1e-9,
            details=f"type={joint.articulation_type}, limits={joint.motion_limits}",
        )
        ctx.check(
            f"{name} press axis is straight-in",
            tuple(round(v, 6) for v in joint.axis) == expected_axes[i],
            details=f"axis={joint.axis}, expected={expected_axes[i]}",
        )

    # ---- home button (button_0) presses into the front face ----
    home = object_model.get_part("button_0")
    home_joint = object_model.get_articulation("body_to_button_0")
    rest_z = ctx.part_world_position(home)[2]
    with ctx.pose({home_joint: PRESS_TRAVEL}):
        pressed_z = ctx.part_world_position(home)[2]
    ctx.check(
        "button_0 (home) presses inward (-Z)",
        pressed_z < rest_z - 0.0003,
        details=f"rest_z={rest_z}, pressed_z={pressed_z}",
    )

    # ---- power button (button_1) presses inward from right side ----
    power = object_model.get_part("button_1")
    power_joint = object_model.get_articulation("body_to_button_1")
    rest_x = ctx.part_world_position(power)[0]
    with ctx.pose({power_joint: PRESS_TRAVEL}):
        pressed_x = ctx.part_world_position(power)[0]
    ctx.check(
        "button_1 (power) presses inward (-X from right side)",
        pressed_x < rest_x - 0.0003,
        details=f"rest_x={rest_x}, pressed_x={pressed_x}",
    )

    # ---- no legacy keypad parts remain ----
    all_part_names = [p.name for p in object_model.parts]
    ctx.check(
        "no legacy keypad keys remain (full-touch slab)",
        not any(n.startswith("key_") for n in all_part_names),
        details=f"parts={all_part_names}",
    )

    return ctx.report()


object_model = build_object_model()
