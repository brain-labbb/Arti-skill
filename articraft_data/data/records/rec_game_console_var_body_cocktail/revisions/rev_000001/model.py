from __future__ import annotations

# Retro cocktail-table arcade game console.
#
# The reference image shows a weathered blue sheet-metal cabinet with:
#   - weathered blue sheet-metal body panels, black metal edge trim, and a
#     bolted front access panel.
#   - in this cocktail-table fork, a low wide cabinet with a horizontal +Z top
#     deck; the dark "GAME OVER" screen is set flush into that deck.
#   - two gold keypad button grids and a central red joystick plate mounted on
#     the same top deck in front of the screen.
#   - a blue ball-top joystick standing up out of the red control plate.
#   - a low base pedestal block under the cabinet.
#
# The clear, physical moving control is the JOYSTICK: a ball-top stick that
# pivots about a gimbal at the red control plate. It is articulated as a
# REVOLUTE joint that tilts forward/back about the plate.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Overall scale (meters). Low wide cocktail-table arcade cabinet.
# ---------------------------------------------------------------------------
CAB_W = 0.780  # left-right width
CAB_D = 0.560  # front-back depth
CAB_H = 0.320  # low body height (excluding base pedestal)

WALL = 0.014  # sheet-metal shell wall thickness

# Front of cabinet is at +Y, back at -Y. Center the body on X and put the body
# floor at z = base_h so the pedestal sits beneath it.
BASE_H = 0.055
BASE_INSET = 0.006  # plinth is just inside the cabinet footprint on each side

# Y of the front face plane and back face plane.
FRONT_Y = CAB_D / 2.0
BACK_Y = -CAB_D / 2.0

# Z levels (world, body floor at BASE_H).
Z_FLOOR = BASE_H
Z_TOP = Z_FLOOR + CAB_H  # horizontal cocktail-table top deck height


# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------
def _materials(model: ArticulatedObject) -> dict[str, object]:
    return {
        "blue_metal": model.material("cabinet_blue_metal", rgba=(0.16, 0.24, 0.62, 1.0)),
        "dark_trim": model.material("cabinet_dark_trim", rgba=(0.10, 0.10, 0.12, 1.0)),
        "screen": model.material("marquee_screen", rgba=(0.06, 0.06, 0.07, 1.0)),
        "gold_text": model.material("marquee_gold_text", rgba=(0.80, 0.66, 0.12, 1.0)),
        "keypad": model.material("control_keypad_gold", rgba=(0.72, 0.60, 0.20, 1.0)),
        "red_plate": model.material("control_red_plate", rgba=(0.62, 0.10, 0.10, 1.0)),
        "access_panel": model.material("front_access_panel", rgba=(0.18, 0.16, 0.15, 1.0)),
        "screw": model.material("panel_screw", rgba=(0.55, 0.55, 0.58, 1.0)),
        "stick_blue": model.material("joystick_ball_blue", rgba=(0.12, 0.32, 0.85, 1.0)),
        "stick_shaft": model.material("joystick_shaft", rgba=(0.20, 0.20, 0.22, 1.0)),
        "stick_collar": model.material("joystick_collar", rgba=(0.45, 0.45, 0.48, 1.0)),
    }


# ---------------------------------------------------------------------------
# Cabinet shell, built as a hollow low cocktail-table body in CadQuery.
# ---------------------------------------------------------------------------
def _build_cabinet_shell() -> cq.Workplane:
    """Hollow rectangular cocktail-table shell.

    The local frame origin sits at the body floor center; z=0 is the underside
    of the blue body and z=CAB_H is the +Z top deck.  The bottom is open like a
    sheet-metal cabinet shell, leaving visible wall thickness while the top deck
    stays continuous for the flush screen and controls.
    """
    outer = (
        cq.Workplane("XY")
        .box(CAB_W, CAB_D, CAB_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.012)
    )
    inner = (
        cq.Workplane("XY")
        .box(
            CAB_W - 2 * WALL,
            CAB_D - 2 * WALL,
            CAB_H - WALL + 0.006,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, -0.003))
    )
    shell = outer.cut(inner)
    return shell


def _rect_frame(outer_w: float, outer_d: float, inner_w: float, inner_d: float, thickness: float) -> cq.Workplane:
    """Thin rectangular frame with a true open center."""
    return cq.Workplane("XY").rect(outer_w, outer_d).rect(inner_w, inner_d).extrude(thickness)


def _keypad_geometry(width: float, depth: float, thickness: float) -> cq.Workplane:
    """Gold keypad slab with a connected raised 7-by-3 button grid."""
    return (
        cq.Workplane("XY")
        .box(width, depth, thickness, centered=(True, True, False))
        .faces(">Z")
        .workplane()
        .rarray(width / 7.0, depth / 3.0, 7, 3)
        .box(width / 9.0, depth / 4.0, 0.004, centered=(True, True, False))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cocktail_table_arcade_console")
    mats = _materials(model)

    # ---- Root cabinet body --------------------------------------------------
    cabinet = model.part("cabinet_body")

    shell = _build_cabinet_shell()
    cabinet.visual(
        mesh_from_cadquery(shell, "cabinet_shell"),
        origin=Origin(xyz=(0.0, 0.0, Z_FLOOR)),
        material=mats["blue_metal"],
        name="cabinet_shell",
    )

    # Base pedestal under the body.
    cabinet.visual(
        Box((CAB_W - 2 * BASE_INSET, CAB_D - 2 * BASE_INSET, BASE_H + 0.006)),
        origin=Origin(xyz=(0.0, 0.0, (BASE_H + 0.006) / 2.0)),
        material=mats["dark_trim"],
        name="base_pedestal",
    )

    # ---- Horizontal +Z top deck: screen, text, and top-mounted controls -----
    # The cocktail-table fork keeps the same layers as the parent console, but
    # moves them from a sloped cabinet face onto the horizontal top deck.
    deck_z = Z_TOP

    # Black metal edge rails emphasize the low table/cabinet construction.
    rail_h = 0.020
    rail_t = 0.016
    rail_specs = (
        ("top_rail_0", (CAB_W, rail_t, rail_h), (0.0, FRONT_Y - rail_t / 2.0, deck_z - rail_h / 2.0)),
        ("top_rail_1", (CAB_W, rail_t, rail_h), (0.0, BACK_Y + rail_t / 2.0, deck_z - rail_h / 2.0)),
        ("top_rail_2", (rail_t, CAB_D, rail_h), (-CAB_W / 2.0 + rail_t / 2.0, 0.0, deck_z - rail_h / 2.0)),
        ("top_rail_3", (rail_t, CAB_D, rail_h), (CAB_W / 2.0 - rail_t / 2.0, 0.0, deck_z - rail_h / 2.0)),
    )
    for rail_name, rail_size, rail_xyz in rail_specs:
        cabinet.visual(
            Box(rail_size),
            origin=Origin(xyz=rail_xyz),
            material=mats["dark_trim"],
            name=rail_name,
        )

    # Flush horizontal screen set into the +Z deck, with a real frame opening.
    screen_w = 0.455
    screen_d = 0.285
    screen_center = (0.0, -0.065, deck_z - 0.006)
    bezel = _rect_frame(screen_w + 0.040, screen_d + 0.040, screen_w, screen_d, 0.006)
    cabinet.visual(
        mesh_from_cadquery(bezel, "screen_bezel"),
        origin=Origin(xyz=screen_center),
        material=mats["dark_trim"],
        name="screen_bezel",
    )
    screen = cq.Workplane("XY").box(screen_w, screen_d, 0.006, centered=(True, True, False))
    cabinet.visual(
        mesh_from_cadquery(screen, "screen_glass"),
        origin=Origin(xyz=screen_center),
        material=mats["screen"],
        name="screen_glass",
    )
    # "GAME OVER" as a thin golden applique seated onto the glass.
    text_bar = cq.Workplane("XY").box(screen_w * 0.58, screen_d * 0.13, 0.0015, centered=(True, True, False))
    cabinet.visual(
        mesh_from_cadquery(text_bar, "game_over_text"),
        origin=Origin(xyz=(screen_w * 0.04, screen_center[1] - screen_d * 0.12, deck_z)),
        material=mats["gold_text"],
        name="game_over_text",
    )

    # Top control cluster: two keypad grids flanking a central red plate.
    control_y = 0.190
    keypad_w = 0.185
    keypad_d = 0.080
    keypad_t = 0.010
    keypad_xs = (-0.245, 0.245)
    for i in range(2):
        keypad = _keypad_geometry(keypad_w, keypad_d, keypad_t)
        cabinet.visual(
            mesh_from_cadquery(keypad, f"keypad_{i}"),
            origin=Origin(xyz=(keypad_xs[i], control_y, deck_z)),
            material=mats["keypad"],
            name=f"keypad_{i}",
        )

    # Central red control plate holding the joystick gimbal.
    red_w = 0.092
    red_d = 0.086
    red_t = 0.012
    red_plate = (
        cq.Workplane("XY")
        .box(red_w, red_d, red_t, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
    )
    plate_xyz = (0.0, control_y, deck_z)
    cabinet.visual(
        mesh_from_cadquery(red_plate, "control_red_plate"),
        origin=Origin(xyz=plate_xyz),
        material=mats["red_plate"],
        name="control_red_plate",
    )
    # Raised gimbal collar the stick passes through (mount + hides the pivot).
    collar = cq.Workplane("XY").circle(0.016).extrude(0.010)
    collar_origin = (0.0, control_y, deck_z + red_t)
    cabinet.visual(
        mesh_from_cadquery(collar, "joystick_collar"),
        origin=Origin(xyz=collar_origin),
        material=mats["stick_collar"],
        name="joystick_collar",
    )

    # ---- Lower front access panel + corner screws --------------------------
    panel_w = CAB_W - 2 * 0.170
    panel_h = CAB_H - 2 * 0.080
    panel_cz = Z_FLOOR + CAB_H / 2.0
    # The panel is built lying in XY (thickness along Z) then stood upright on
    # the vertical front face by rotating 90 deg about X, so its thickness ends
    # up along Y (front-back). panel_w stays the X width, panel_h becomes the Z
    # height.
    panel = (
        cq.Workplane("XY")
        .box(panel_w, panel_h, 0.010, centered=(True, True, False))
    )
    cabinet.visual(
        mesh_from_cadquery(panel, "access_panel"),
        origin=Origin(xyz=(0.0, FRONT_Y, panel_cz), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mats["access_panel"],
        name="access_panel",
    )
    # Four corner screws, heads proud of the panel front, shanks sunk into it.
    panel_front_y = FRONT_Y + 0.005  # outer face of the standing panel
    screw_offsets = (
        (-panel_w / 2.0 + 0.022, panel_h / 2.0 - 0.022),
        (panel_w / 2.0 - 0.022, panel_h / 2.0 - 0.022),
        (-panel_w / 2.0 + 0.022, -panel_h / 2.0 + 0.022),
        (panel_w / 2.0 - 0.022, -panel_h / 2.0 + 0.022),
    )
    for i in range(4):
        scx = screw_offsets[i][0]
        scz = panel_cz + screw_offsets[i][1]
        # Cylinder axis along Y (points out of the front face).
        screw = cq.Workplane("XY").circle(0.0045).extrude(0.010)
        cabinet.visual(
            mesh_from_cadquery(screw, f"screw_{i}"),
            origin=Origin(xyz=(scx, panel_front_y - 0.006, scz), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=mats["screw"],
            name=f"access_screw_{i}",
        )

    # ---- Joystick (the articulated control) --------------------------------
    # Authored in its own local frame so the pivot is at local z=0 (the gimbal),
    # the shaft rises along +Z from the horizontal top deck, topped by the blue
    # ball.
    joystick = model.part("joystick")
    shaft_len = 0.055
    shaft = (
        cq.Workplane("XY")
        .circle(0.006)
        .extrude(shaft_len)
    )
    joystick.visual(
        mesh_from_cadquery(shaft, "joystick_shaft"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=mats["stick_shaft"],
        name="joystick_shaft",
    )
    # Tapered dust-boot base flare near the gimbal.
    boot = (
        cq.Workplane("XY")
        .circle(0.013)
        .workplane(offset=0.012)
        .circle(0.007)
        .loft()
    )
    joystick.visual(
        mesh_from_cadquery(boot, "joystick_boot"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=mats["stick_collar"],
        name="joystick_boot",
    )
    # Blue ball top.
    joystick.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(0.0, 0.0, shaft_len + 0.012)),
        material=mats["stick_blue"],
        name="joystick_ball",
    )

    # Joint: pivot at the visible top red plate/collar contact surface. The
    # child local +Z points upward from the horizontal deck; positive q tips
    # the ball toward +Y and downward.
    pivot_world = (0.0, control_y, deck_z + red_t)
    model.articulation(
        "panel_to_joystick",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=joystick,
        origin=Origin(xyz=pivot_world),
        # Rotation about -X tips the top of the stick toward +Y (forward).
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=-0.45, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet_body")
    joystick = object_model.get_part("joystick")
    joint = object_model.get_articulation("panel_to_joystick")

    # The joystick boot sits down over the mounting collar and the joystick
    # shaft passes up through that same collar bore (a captured gimbal seat),
    # so a small local interpenetration of those element pairs is by design.
    ctx.allow_overlap(
        joystick,
        cabinet,
        elem_a="joystick_boot",
        elem_b="joystick_collar",
        reason="The joystick boot is seated down over the fixed gimbal collar; this captured fit overlaps locally by design.",
    )
    ctx.allow_overlap(
        joystick,
        cabinet,
        elem_a="joystick_shaft",
        elem_b="joystick_collar",
        reason="The joystick shaft passes up through the fixed gimbal collar bore (modeled as a solid collar proxy); the captured shaft-in-collar fit overlaps locally by design.",
    )

    # ---- Joint type / axis contract ----------------------------------------
    ctx.check(
        "joystick joint is revolute",
        str(getattr(joint.articulation_type, "value", joint.articulation_type)).lower().endswith("revolute"),
        f"type={joint.articulation_type!r}",
    )
    ctx.check(
        "joystick joint tilts about X",
        abs(abs(joint.axis[0]) - 1.0) < 1e-6 and abs(joint.axis[1]) < 1e-6 and abs(joint.axis[2]) < 1e-6,
        f"axis={joint.axis!r}",
    )
    lim = joint.motion_limits
    ctx.check(
        "joystick has symmetric deflection limits",
        lim is not None and lim.lower is not None and lim.upper is not None and lim.lower < 0.0 < lim.upper,
        f"limits=({None if lim is None else lim.lower}, {None if lim is None else lim.upper})",
    )

    # ---- Hero parts present & placed ---------------------------------------
    cab_aabb = ctx.part_world_aabb(cabinet)
    assert cab_aabb is not None
    cmins, cmaxs = cab_aabb
    csize = tuple(cmaxs[i] - cmins[i] for i in range(3))
    ctx.check(
        "cabinet is a low wide cocktail table",
        0.74 <= csize[0] <= 0.82 and 0.53 <= csize[1] <= 0.60 and 0.36 <= csize[2] <= 0.43,
        f"size={csize!r}",
    )

    # Screen is horizontal and flush with the +Z top deck.
    screen_aabb = ctx.part_element_world_aabb(cabinet, elem="screen_glass")
    assert screen_aabb is not None
    smin, smax = screen_aabb
    screen_cz = (smin[2] + smax[2]) / 2.0
    ctx.check(
        "screen glass is flush in the top deck",
        abs(smax[2] - Z_TOP) <= 0.002 and (smax[2] - smin[2]) <= 0.008,
        f"screen z=[{smin[2]:.3f},{smax[2]:.3f}], top={Z_TOP:.3f}",
    )
    ctx.check(
        "screen lies flat on the horizontal deck",
        (smax[2] - smin[2]) < 0.02 and (smax[0] - smin[0]) > 0.40 and (smax[1] - smin[1]) > 0.25,
        f"screen dims={(smax[0]-smin[0], smax[1]-smin[1], smax[2]-smin[2])!r}",
    )
    # GAME OVER text overlaps the screen plane (legible marquee).
    ctx.expect_overlap(
        cabinet,
        cabinet,
        axes="x",
        elem_a="game_over_text",
        elem_b="screen_glass",
        min_overlap=0.04,
        name="game over text overlaps screen",
    )

    # Two keypad grids flank the center on opposite sides of X, on the same top deck.
    kl = ctx.part_element_world_aabb(cabinet, elem="keypad_0")
    kr = ctx.part_element_world_aabb(cabinet, elem="keypad_1")
    assert kl is not None and kr is not None
    kl_cx = (kl[0][0] + kl[1][0]) / 2.0
    kr_cx = (kr[0][0] + kr[1][0]) / 2.0
    kl_min_z = kl[0][2]
    kr_min_z = kr[0][2]
    ctx.check(
        "keypads flank center on top deck",
        kl_cx < -0.08 and kr_cx > 0.08 and abs(kl_min_z - Z_TOP) <= 0.004 and abs(kr_min_z - Z_TOP) <= 0.004,
        f"k0_cx={kl_cx:.3f}, k1_cx={kr_cx:.3f}, zmins=({kl_min_z:.3f},{kr_min_z:.3f})",
    )
    rp = ctx.part_element_world_aabb(cabinet, elem="control_red_plate")
    assert rp is not None
    rp_cy = (rp[0][1] + rp[1][1]) / 2.0
    screen_cy = (smin[1] + smax[1]) / 2.0
    ctx.check(
        "controls are mounted forward of the flush screen",
        rp_cy > screen_cy + 0.16 and abs(rp[0][2] - Z_TOP) <= 0.004,
        f"red_plate_cy={rp_cy:.3f}, screen_cy={screen_cy:.3f}, red_zmin={rp[0][2]:.3f}",
    )

    # Access panel remains on the front vertical face, with four screws.
    ap = ctx.part_element_world_aabb(cabinet, elem="access_panel")
    assert ap is not None
    ap_cz = (ap[0][2] + ap[1][2]) / 2.0
    ctx.check(
        "access panel is on the front face",
        ap[1][1] > FRONT_Y - 0.003 and cmins[2] + 0.20 * csize[2] < ap_cz < cmins[2] + 0.70 * csize[2],
        f"ap_cz={ap_cz:.3f}, ap_ymax={ap[1][1]:.3f}",
    )
    for i in range(4):
        nm = f"access_screw_{i}"
        sa = ctx.part_element_world_aabb(cabinet, elem=nm)
        ctx.check(f"{nm} present", sa is not None, f"missing {nm}")

    # ---- Joystick mounting & geometry --------------------------------------
    ball = ctx.part_element_world_aabb(joystick, elem="joystick_ball")
    assert ball is not None
    ball_cz = (ball[0][2] + ball[1][2]) / 2.0
    # Ball is above its shaft base.
    shaft = ctx.part_element_world_aabb(joystick, elem="joystick_shaft")
    assert shaft is not None
    shaft_base_z = shaft[0][2]
    ctx.check(
        "joystick ball sits above the shaft",
        ball_cz > shaft_base_z + 0.04,
        f"ball_cz={ball_cz:.3f}, shaft_base_z={shaft_base_z:.3f}",
    )
    # Joystick base is seated near the collar on the control plate (connected).
    ctx.expect_contact(
        joystick,
        cabinet,
        elem_a="joystick_boot",
        elem_b="joystick_collar",
        contact_tol=0.012,
        name="joystick boot seats at the collar",
    )
    # The captured shaft stays centered within the collar footprint and remains
    # engaged with it along the stick axis (retained insertion proof).
    ctx.expect_within(
        joystick,
        cabinet,
        axes="xy",
        inner_elem="joystick_shaft",
        outer_elem="joystick_collar",
        margin=0.004,
        name="joystick shaft stays inside the collar bore",
    )
    ctx.expect_overlap(
        joystick,
        cabinet,
        axes="z",
        elem_a="joystick_shaft",
        elem_b="joystick_collar",
        min_overlap=0.006,
        name="joystick shaft stays engaged in the collar",
    )

    # ---- Decisive articulated pose: deflecting tilts the ball forward ------
    rest_ball = ctx.part_element_world_aabb(joystick, elem="joystick_ball")
    assert rest_ball is not None
    rest_y = (rest_ball[0][1] + rest_ball[1][1]) / 2.0
    rest_z = (rest_ball[0][2] + rest_ball[1][2]) / 2.0
    with ctx.pose({joint: 0.40}):
        fwd_ball = ctx.part_element_world_aabb(joystick, elem="joystick_ball")
        assert fwd_ball is not None
        fwd_y = (fwd_ball[0][1] + fwd_ball[1][1]) / 2.0
        fwd_z = (fwd_ball[0][2] + fwd_ball[1][2]) / 2.0
    ctx.check(
        "positive deflection tilts the ball forward and down",
        fwd_y > rest_y + 0.01 and fwd_z < rest_z - 0.002,
        f"rest=(y={rest_y:.3f},z={rest_z:.3f}) fwd=(y={fwd_y:.3f},z={fwd_z:.3f})",
    )

    return ctx.report()


object_model = build_object_model()
