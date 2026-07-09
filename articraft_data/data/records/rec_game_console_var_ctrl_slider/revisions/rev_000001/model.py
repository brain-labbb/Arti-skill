from __future__ import annotations

# Retro tabletop / countertop arcade game console.
#
# The reference image shows a weathered blue sheet-metal cabinet with:
#   - a wedge body: tall vertical back, a sloped upper-front face carrying a
#     dark "GAME OVER" marquee/screen behind a metal bezel, and a vertical lower
#     front carrying a bolted access panel.
#   - a recessed control band between the screen and the lower panel holding two
#     gold keypad button grids that flank a central red control plate.
#   - this fork replaces the original blue ball-top joystick with a low thumb
#     slider riding left-right in a recessed rail slot across the red plate.
#   - a base pedestal block under the cabinet.
#
# The clear, physical moving control is the SLIDER: a thumb cap and captured
# runner translating left-right in the rail slot. It is articulated as a
# PRISMATIC joint along cabinet X.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Overall scale (meters). Compact countertop arcade cabinet.
# ---------------------------------------------------------------------------
CAB_W = 0.420  # left-right width
CAB_D = 0.460  # front-back depth (back face is the deepest extent)
CAB_H = 0.560  # body height (excluding base pedestal)

WALL = 0.014  # sheet-metal shell wall thickness

# Vertical front face (lower, carrying the access panel) goes from the floor of
# the body up to the control band. The upper front face slopes back toward the
# top of the cabinet, carrying the screen/marquee.
LOWER_FRONT_H = 0.250  # height of the vertical lower front face
CONTROL_BAND_H = 0.085  # height of the recessed control band
SLOPE_TOP_BACK_Y = -0.060  # how far back (toward -Y) the slope top edge sits

# Front of cabinet is at +Y, back at -Y. Center the body on X and put the body
# floor at z = base_h so the pedestal sits beneath it.
BASE_H = 0.045
BASE_INSET = 0.030  # pedestal is inset from the cabinet footprint on each side

# Y of the front face plane and back face plane.
FRONT_Y = CAB_D / 2.0
BACK_Y = -CAB_D / 2.0

# Z levels (world, body floor at BASE_H).
Z_FLOOR = BASE_H
Z_LOWER_TOP = Z_FLOOR + LOWER_FRONT_H  # top of the vertical lower front
Z_CONTROL_TOP = Z_LOWER_TOP + CONTROL_BAND_H  # top of the control band
Z_TOP = Z_FLOOR + CAB_H  # very top of cabinet (back edge height)


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
        "slider_blue": model.material("slider_thumb_blue", rgba=(0.12, 0.32, 0.85, 1.0)),
        "slider_runner": model.material("slider_runner_dark", rgba=(0.08, 0.08, 0.09, 1.0)),
        "slot_floor": model.material("slider_slot_shadow", rgba=(0.025, 0.025, 0.030, 1.0)),
    }


# ---------------------------------------------------------------------------
# Cabinet shell, built as a hollow wedge in CadQuery.
# ---------------------------------------------------------------------------
def _build_cabinet_shell() -> cq.Workplane:
    """Hollow wedge cabinet shell.

    Built in a frame whose origin sits at the body floor center
    (x=0, y=0, z=Z_FLOOR maps to local z=0). The side profile (in the Y-Z
    plane) is a polygon: vertical back, sloped top, then a kinked front going
    down through the control-band setback and the vertical lower front.
    """
    # Side profile points (y, z) in local coords with z measured from floor.
    lower_top = LOWER_FRONT_H
    control_top = lower_top + CONTROL_BAND_H
    top_h = CAB_H
    control_setback = 0.055  # control band is recessed back from the front plane

    side = [
        (BACK_Y, 0.0),  # back-bottom
        (BACK_Y, top_h),  # back-top
        (SLOPE_TOP_BACK_Y, top_h),  # top edge (front of the flat roof)
        (FRONT_Y - control_setback, control_top),  # bottom of sloped screen face
        (FRONT_Y - control_setback, lower_top),  # back wall of control band
        (FRONT_Y, lower_top),  # front lip of control band -> lower front top
        (FRONT_Y, 0.0),  # front-bottom
    ]

    outer = (
        cq.Workplane("YZ")
        .polyline(side)
        .close()
        .extrude(CAB_W, both=True)  # extrude along X, total width = 2*CAB_W? no
    )
    # `both=True` extrudes CAB_W in each direction -> total 2*CAB_W. We want
    # total width CAB_W centered, so extrude CAB_W/2 both sides instead.
    outer = (
        cq.Workplane("YZ")
        .polyline(side)
        .close()
        .extrude(CAB_W / 2.0, both=True)
    )

    # Hollow it out: a smaller inset wedge subtracted, leaving the shell walls,
    # but keep the bottom closed and the top closed.
    inner_side = [
        (BACK_Y + WALL, WALL),
        (BACK_Y + WALL, top_h - WALL),
        (SLOPE_TOP_BACK_Y, top_h - WALL),
        (FRONT_Y - control_setback - WALL, control_top - WALL),
        (FRONT_Y - control_setback - WALL, lower_top + WALL),
        (FRONT_Y - WALL, lower_top + WALL),
        (FRONT_Y - WALL, WALL),
    ]
    inner = (
        cq.Workplane("YZ")
        .polyline(inner_side)
        .close()
        .extrude((CAB_W / 2.0) - WALL, both=True)
    )
    shell = outer.cut(inner)

    # Cut the screen opening in the sloped upper-front face and the access-panel
    # recess in the lower front face are handled as separate applied visuals
    # (dark inset plates), so the shell stays a clean hollow wedge.
    shell = shell.edges("|X").fillet(0.006)
    return shell


def _sloped_face_geometry() -> tuple[tuple[float, float, float], float, float]:
    """Return (center_xyz, pitch, face_length) of the sloped screen face.

    The sloped face runs from the top edge at
    (y=SLOPE_TOP_BACK_Y, z=Z_TOP) down to the control-band back wall at
    (y=FRONT_Y-control_setback, z=Z_CONTROL_TOP).
    """
    control_setback = 0.055
    y_top = SLOPE_TOP_BACK_Y
    z_top = Z_TOP
    y_bot = FRONT_Y - control_setback
    z_bot = Z_CONTROL_TOP
    cy = (y_top + y_bot) / 2.0
    cz = (z_top + z_bot) / 2.0
    dy = y_bot - y_top
    dz = z_bot - z_top
    length = math.hypot(dy, dz)
    # Pitch about X so that a panel lying in XY (normal +Z) rotates to lie
    # against this face. Rx(pitch) maps +Z to (0, -sin(pitch), cos(pitch));
    # the slope's outward (up-and-forward) normal is (0, -dz, dy) / length,
    # so pitch = atan2(dz, dy).
    pitch = math.atan2(dz, dy)  # rotation about +X (roll), see usage below
    return (0.0, cy, cz), pitch, length


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tabletop_arcade_console")
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
        Box((CAB_W - 2 * BASE_INSET, CAB_D - 2 * BASE_INSET, BASE_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_H / 2.0)),
        material=mats["dark_trim"],
        name="base_pedestal",
    )

    # ---- Sloped screen face: bezel + dark screen + GAME OVER text ----------
    (fx, fy, fz), pitch, face_len = _sloped_face_geometry()
    face_w = CAB_W - 2 * 0.030  # leave side margins

    # Dark metal bezel frame around the screen (sits slightly proud of face).
    bezel = (
        cq.Workplane("XY")
        .box(face_w + 0.024, face_len - 0.006, 0.010)
    )
    cabinet.visual(
        mesh_from_cadquery(bezel, "screen_bezel"),
        origin=Origin(xyz=(fx, fy, fz), rpy=(pitch, 0.0, 0.0)),
        material=mats["dark_trim"],
        name="screen_bezel",
    )
    # Dark recessed screen.
    screen = (
        cq.Workplane("XY")
        .box(face_w, face_len - 0.020, 0.008)
    )
    cabinet.visual(
        mesh_from_cadquery(screen, "screen_glass"),
        origin=Origin(xyz=(fx, fy, fz + 0.0015), rpy=(pitch, 0.0, 0.0)),
        material=mats["screen"],
        name="screen_glass",
    )
    # "GAME OVER" gold text bar (a slim raised plate reading as the lit text).
    text_bar = (
        cq.Workplane("XY")
        .box(face_w * 0.62, face_len * 0.16, 0.004)
    )
    # Offset the text toward the upper part of the screen face like the image.
    # Local +Y of the panel maps to the downslope direction, so a negative
    # local-Y offset moves the text upslope (toward the top edge).
    text_local = (face_w * 0.06, -face_len * 0.12)
    # Rotate the local in-plane offset into the tilted face frame.
    tx = text_local[0]
    ty = text_local[1] * math.cos(pitch)
    tz = text_local[1] * math.sin(pitch)
    # Seat the raised text into the screen glass like applied ink/paint so it is
    # visibly supported rather than a floating plate.
    n_y = -math.sin(pitch)
    n_z = math.cos(pitch)
    text_embed_offset = 0.0065
    cabinet.visual(
        mesh_from_cadquery(text_bar, "game_over_text"),
        origin=Origin(
            xyz=(fx + tx, fy + ty + text_embed_offset * n_y, fz + tz + text_embed_offset * n_z),
            rpy=(pitch, 0.0, 0.0),
        ),
        material=mats["gold_text"],
        name="game_over_text",
    )

    # ---- Control band: two keypad grids + central red plate ----------------
    band_y = FRONT_Y - 0.055 + 0.010  # just in front of the band back wall
    band_z = Z_LOWER_TOP + CONTROL_BAND_H / 2.0
    band_face_pitch = -0.18  # control band tilts up slightly toward the player

    keypad_w = 0.140
    keypad_h = 0.052
    keypad_t = 0.010
    for side_sign, label in ((-1.0, "left"), (1.0, "right")):
        kx = side_sign * (keypad_w / 2.0 + 0.040)
        keypad = (
            cq.Workplane("XY")
            .box(keypad_w, keypad_h, keypad_t)
            .faces(">Z")
            .workplane()
            .rarray(keypad_w / 7.0, keypad_h / 3.0, 7, 3)
            .box(keypad_w / 9.0, keypad_h / 4.0, 0.004, centered=(True, True, False))
        )
        cabinet.visual(
            mesh_from_cadquery(keypad, f"keypad_{label}"),
            origin=Origin(xyz=(kx, band_y, band_z), rpy=(band_face_pitch, 0.0, 0.0)),
            material=mats["keypad"],
            name=f"keypad_{label}",
        )

    # Central red control plate (recessed pocket holding the sliding control).
    red_w = 0.072
    red_h = 0.060
    slot_len = 0.060
    slot_w = 0.018
    red_plate = (
        cq.Workplane("XY")
        .box(red_w, red_h, 0.012)
        # Through-slot across X: the cut edges of the red plate act as the
        # rails, with a dark sub-floor below making the recess readable.
        .cut(
            cq.Workplane("XY")
            .box(slot_len, slot_w, 0.030)
        )
        .edges("|Z")
        .fillet(0.004)
    )
    # Plate front face center, in world. The slider rides in the slot cut here.
    plate_xyz = (0.0, band_y + 0.004, band_z)
    cabinet.visual(
        mesh_from_cadquery(red_plate, "control_red_plate"),
        origin=Origin(xyz=plate_xyz, rpy=(band_face_pitch, 0.0, 0.0)),
        material=mats["red_plate"],
        name="control_red_plate",
    )
    # Dark recessed floor visible at the bottom of the through-slot.
    slot_floor = (
        cq.Workplane("XY")
        .box(slot_len + 0.006, slot_w - 0.002, 0.002)
        .edges("|Z")
        .fillet(0.002)
    )
    slot_floor_offset = -0.005
    slot_floor_xyz = (
        plate_xyz[0],
        plate_xyz[1] - math.sin(band_face_pitch) * slot_floor_offset,
        plate_xyz[2] + math.cos(band_face_pitch) * slot_floor_offset,
    )
    cabinet.visual(
        mesh_from_cadquery(slot_floor, "slider_slot_floor"),
        origin=Origin(xyz=slot_floor_xyz, rpy=(band_face_pitch, 0.0, 0.0)),
        material=mats["slot_floor"],
        name="slider_slot_floor",
    )

    # ---- Lower front access panel + corner screws --------------------------
    panel_w = CAB_W - 2 * 0.055
    panel_h = LOWER_FRONT_H - 2 * 0.045
    panel_cz = Z_FLOOR + LOWER_FRONT_H / 2.0
    # The panel is built lying in XY (thickness along Z) then stood upright on
    # the vertical front face by rotating 90 deg about X, so its thickness ends
    # up along Y (front-back). panel_w stays the X width, panel_h becomes the Z
    # height.
    panel = (
        cq.Workplane("XY")
        .box(panel_w, panel_h, 0.010)
    )
    cabinet.visual(
        mesh_from_cadquery(panel, "access_panel"),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.004, panel_cz), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mats["access_panel"],
        name="access_panel",
    )
    # Four corner screws, heads proud of the panel front, shanks sunk into it.
    panel_front_y = FRONT_Y - 0.004 + 0.005  # outer face of the standing panel
    for sx in (-1, 1):
        for sz in (-1, 1):
            scx = sx * (panel_w / 2.0 - 0.018)
            scz = panel_cz + sz * (panel_h / 2.0 - 0.018)
            # Cylinder axis along Y (points out of the front face).
            screw = cq.Workplane("XY").circle(0.0045).extrude(0.010)
            cabinet.visual(
                mesh_from_cadquery(screw, f"screw_{sx}_{sz}"),
                origin=Origin(xyz=(scx, panel_front_y - 0.006, scz), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=mats["screw"],
                name=f"access_screw_{'r' if sx > 0 else 'l'}_{'t' if sz > 0 else 'b'}",
            )

    # ---- Linear slider (the articulated control) ----------------------------
    # Authored in the same tilted frame as the red plate. The part origin is on
    # the actual top contact surface at the center of the slot; local X is the
    # cabinet's left-right rail direction and local +Z is the plate normal.
    slider = model.part("slider")
    slider_runner = (
        cq.Workplane("XY")
        .box(0.028, slot_w * 0.62, 0.006)
        .edges("|Z")
        .fillet(0.002)
    )
    slider.visual(
        mesh_from_cadquery(slider_runner, "slider_runner"),
        # Runner hangs down into the slot but stays clear of the dark floor.
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=mats["slider_runner"],
        name="slider_runner",
    )
    slider_thumb = (
        cq.Workplane("XY")
        .box(0.034, 0.026, 0.016, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.006)
        .faces(">Z")
        .edges()
        .fillet(0.003)
    )
    slider.visual(
        mesh_from_cadquery(slider_thumb, "slider_thumb"),
        # Bottom of the cap is flush with the rail plate surface, overlapping
        # the red rails in footprint while its runner passes through the slot.
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=mats["slider_blue"],
        name="slider_thumb",
    )

    # Joint: translate along local X. Because the band tilt is a rotation about
    # X, this is also cabinet/world X, matching the visible left-right slot.
    plate_top_offset = 0.006
    slider_origin = (
        plate_xyz[0],
        plate_xyz[1] - math.sin(band_face_pitch) * plate_top_offset,
        plate_xyz[2] + math.cos(band_face_pitch) * plate_top_offset,
    )
    model.articulation(
        "panel_to_slider",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=slider,
        origin=Origin(xyz=slider_origin, rpy=(band_face_pitch, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.18, lower=-0.018, upper=0.018),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet_body")
    slider = object_model.get_part("slider")
    joint = object_model.get_articulation("panel_to_slider")

    # ---- Joint type / axis contract ----------------------------------------
    ctx.check(
        "slider joint is prismatic",
        str(getattr(joint.articulation_type, "value", joint.articulation_type)).lower().endswith("prismatic"),
        f"type={joint.articulation_type!r}",
    )
    ctx.check(
        "slider joint translates along X",
        abs(joint.axis[0] - 1.0) < 1e-6 and abs(joint.axis[1]) < 1e-6 and abs(joint.axis[2]) < 1e-6,
        f"axis={joint.axis!r}",
    )
    lim = joint.motion_limits
    ctx.check(
        "slider has short symmetric travel limits",
        lim is not None
        and lim.lower is not None
        and lim.upper is not None
        and -0.025 <= lim.lower < 0.0 < lim.upper <= 0.025,
        f"limits=({None if lim is None else lim.lower}, {None if lim is None else lim.upper})",
    )

    # ---- Hero parts present & placed ---------------------------------------
    cab_aabb = ctx.part_world_aabb(cabinet)
    assert cab_aabb is not None
    cmins, cmaxs = cab_aabb
    csize = tuple(cmaxs[i] - cmins[i] for i in range(3))
    ctx.check(
        "cabinet is a tabletop-scale wedge",
        0.40 <= csize[0] <= 0.46 and 0.40 <= csize[1] <= 0.52 and 0.55 <= csize[2] <= 0.65,
        f"size={csize!r}",
    )

    # Screen sits high on the cabinet (upper sloped face).
    screen_aabb = ctx.part_element_world_aabb(cabinet, elem="screen_glass")
    assert screen_aabb is not None
    smin, smax = screen_aabb
    screen_cz = (smin[2] + smax[2]) / 2.0
    ctx.check(
        "screen is on the upper half of the cabinet",
        screen_cz > cmins[2] + 0.62 * csize[2],
        f"screen_cz={screen_cz:.3f}, cab z=[{cmins[2]:.3f},{cmaxs[2]:.3f}]",
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

    # Two keypad grids flank the center on opposite sides of X.
    kl = ctx.part_element_world_aabb(cabinet, elem="keypad_left")
    kr = ctx.part_element_world_aabb(cabinet, elem="keypad_right")
    assert kl is not None and kr is not None
    kl_cx = (kl[0][0] + kl[1][0]) / 2.0
    kr_cx = (kr[0][0] + kr[1][0]) / 2.0
    ctx.check(
        "keypads flank center on opposite sides",
        kl_cx < -0.03 and kr_cx > 0.03,
        f"left_cx={kl_cx:.3f}, right_cx={kr_cx:.3f}",
    )

    # Access panel low on the front, with four screws.
    ap = ctx.part_element_world_aabb(cabinet, elem="access_panel")
    assert ap is not None
    ap_cz = (ap[0][2] + ap[1][2]) / 2.0
    ctx.check(
        "access panel is on the lower front",
        ap_cz < cmins[2] + 0.45 * csize[2],
        f"ap_cz={ap_cz:.3f}",
    )
    for nm in ("access_screw_l_t", "access_screw_r_t", "access_screw_l_b", "access_screw_r_b"):
        sa = ctx.part_element_world_aabb(cabinet, elem=nm)
        ctx.check(f"{nm} present", sa is not None, f"missing {nm}")

    # ---- Linear slider mounting & geometry ---------------------------------
    slot = ctx.part_element_world_aabb(cabinet, elem="slider_slot_floor")
    runner = ctx.part_element_world_aabb(slider, elem="slider_runner")
    thumb = ctx.part_element_world_aabb(slider, elem="slider_thumb")
    plate = ctx.part_element_world_aabb(cabinet, elem="control_red_plate")
    assert slot is not None and runner is not None and thumb is not None and plate is not None
    slot_len_x = slot[1][0] - slot[0][0]
    slot_width_y = slot[1][1] - slot[0][1]
    runner_len_x = runner[1][0] - runner[0][0]
    runner_width_y = runner[1][1] - runner[0][1]
    ctx.check(
        "rail slot is a long horizontal recess",
        slot_len_x > 2.2 * slot_width_y and slot_len_x > runner_len_x + 0.025,
        f"slot_len_x={slot_len_x:.3f}, slot_width_y={slot_width_y:.3f}, runner_len_x={runner_len_x:.3f}",
    )
    ctx.check(
        "runner is narrower than the recessed slot",
        runner_width_y < slot_width_y,
        f"runner_width_y={runner_width_y:.3f}, slot_width_y={slot_width_y:.3f}",
    )
    # The blue thumb cap sits visibly proud of the red control plate while its
    # dark runner is retained inside the slot footprint.
    thumb_top_z = thumb[1][2]
    plate_top_z = plate[1][2]
    ctx.check(
        "slider thumb cap stands proud of the control deck",
        thumb_top_z > plate_top_z + 0.010,
        f"thumb_top_z={thumb_top_z:.3f}, plate_top_z={plate_top_z:.3f}",
    )
    ctx.expect_within(
        slider,
        cabinet,
        axes="xy",
        inner_elem="slider_runner",
        outer_elem="slider_slot_floor",
        margin=0.002,
        name="slider runner stays captured in the rail slot",
    )
    ctx.expect_overlap(
        slider,
        cabinet,
        axes="xy",
        elem_a="slider_thumb",
        elem_b="control_red_plate",
        min_overlap=0.018,
        name="slider thumb overlaps the red control deck",
    )

    # ---- Decisive articulated pose: the control slides right in the rail ----
    rest_thumb = ctx.part_element_world_aabb(slider, elem="slider_thumb")
    assert rest_thumb is not None
    rest_x = (rest_thumb[0][0] + rest_thumb[1][0]) / 2.0
    rest_y = (rest_thumb[0][1] + rest_thumb[1][1]) / 2.0
    rest_z = (rest_thumb[0][2] + rest_thumb[1][2]) / 2.0
    with ctx.pose({joint: 0.018}):
        moved_thumb = ctx.part_element_world_aabb(slider, elem="slider_thumb")
        moved_runner = ctx.part_element_world_aabb(slider, elem="slider_runner")
        assert moved_thumb is not None and moved_runner is not None
        moved_x = (moved_thumb[0][0] + moved_thumb[1][0]) / 2.0
        moved_y = (moved_thumb[0][1] + moved_thumb[1][1]) / 2.0
        moved_z = (moved_thumb[0][2] + moved_thumb[1][2]) / 2.0
        ctx.expect_within(
            slider,
            cabinet,
            axes="xy",
            inner_elem="slider_runner",
            outer_elem="slider_slot_floor",
            margin=0.002,
            name="extended slider remains retained by the rail slot",
        )
    ctx.check(
        "positive travel slides the thumb right without lifting",
        moved_x > rest_x + 0.015 and abs(moved_y - rest_y) < 0.002 and abs(moved_z - rest_z) < 0.002,
        f"rest=(x={rest_x:.3f},y={rest_y:.3f},z={rest_z:.3f}) "
        f"moved=(x={moved_x:.3f},y={moved_y:.3f},z={moved_z:.3f})",
    )

    return ctx.report()


object_model = build_object_model()
