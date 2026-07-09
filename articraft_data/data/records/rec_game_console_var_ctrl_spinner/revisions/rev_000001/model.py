from __future__ import annotations

# Retro tabletop / countertop arcade game console.
#
# The reference image shows a weathered blue sheet-metal cabinet with:
#   - a wedge body: tall vertical back, a sloped upper-front face carrying a
#     dark "GAME OVER" marquee/screen behind a metal bezel, and a vertical lower
#     front carrying a bolted access panel.
#   - a recessed control band between the screen and the lower panel holding two
#     gold keypad button grids that flank a central red control plate.
#   - a blue knurled spinner/paddle knob standing up out of the red control
#     plate in this fork variant.
#   - a base pedestal block under the cabinet.
#
# The clear, physical moving control in this variant is the SPINNER: a knurled
# paddle knob captured in the red control plate. It is articulated as a
# CONTINUOUS revolute joint about the vertical Z axis.

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
        "spinner_blue": model.material("spinner_blue_cap", rgba=(0.12, 0.32, 0.85, 1.0)),
        "spinner_metal": model.material("spinner_dark_metal", rgba=(0.20, 0.20, 0.22, 1.0)),
        "spinner_bearing": model.material("spinner_bearing_ring", rgba=(0.45, 0.45, 0.48, 1.0)),
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


def _build_knurled_spinner_body() -> cq.Workplane:
    """One-piece spinner knob with vertical knurl ribs and a raised paddle grip.

    The child part frame is at the lower bearing contact plane. The main knob
    body starts at local z=0 and rises along +Z, so a vertical continuous joint
    can be placed exactly at the visible bearing contact surface.
    """
    radius = 0.025
    height = 0.026
    rib_count = 28
    rib_radial = 0.005
    rib_tangent = 0.0032
    rib_height = 0.021

    knob = cq.Workplane("XY").circle(radius).extrude(height)

    # Vertical knurl flutes/ribs around the side wall. Each rib overlaps the
    # cylinder slightly so the exported mesh remains one connected solid.
    for i in range(rib_count):
        angle = 360.0 * i / rib_count
        rib = (
            cq.Workplane("XY")
            .box(rib_radial, rib_tangent, rib_height)
            .translate((radius + rib_radial / 2.0 - 0.0012, 0.0, 0.004 + rib_height / 2.0))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        )
        knob = knob.union(rib)

    # Raised oval-ish top boss and a slim across-diameter paddle ridge so the
    # control reads as a spinner/paddle knob rather than a joystick ball.
    top_boss = cq.Workplane("XY").circle(0.018).extrude(0.006).translate((0.0, 0.0, height - 0.001))
    paddle = (
        cq.Workplane("XY")
        .box(0.078, 0.008, 0.008)
        .edges("|Z")
        .fillet(0.002)
        .translate((0.0, 0.0, height + 0.002))
    )
    knob = knob.union(top_boss).union(paddle)
    return knob


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
    # Sit the text as a shallow raised/decal-like plate directly on the screen
    # glass, not as a floating island above it.
    n_y = -math.sin(pitch)
    n_z = math.cos(pitch)
    cabinet.visual(
        mesh_from_cadquery(text_bar, "game_over_text"),
        origin=Origin(
            xyz=(fx + tx, fy + ty + 0.0068 * n_y, fz + tz + 0.0068 * n_z),
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

    # Central red control plate (recessed pocket holding the spinner).
    red_w = 0.072
    red_h = 0.060
    red_plate = (
        cq.Workplane("XY")
        .box(red_w, red_h, 0.012)
        .edges("|Z")
        .fillet(0.004)
    )
    # Plate front face center, in world. Spinner bearing mounts here.
    plate_xyz = (0.0, band_y + 0.004, band_z)
    cabinet.visual(
        mesh_from_cadquery(red_plate, "control_red_plate"),
        origin=Origin(xyz=plate_xyz, rpy=(band_face_pitch, 0.0, 0.0)),
        material=mats["red_plate"],
        name="control_red_plate",
    )
    # Raised vertical bearing/bushing for the spinner shaft. It is a visible
    # mechanical mount on the red plate; the rotating child is anchored at this
    # top contact surface rather than on an invisible pad.
    plate_normal_y = -math.sin(band_face_pitch)
    plate_normal_z = math.cos(band_face_pitch)
    plate_surface_y = plate_xyz[1] + 0.006 * plate_normal_y
    plate_surface_z = plate_xyz[2] + 0.006 * plate_normal_z
    bearing_h = 0.052
    bearing_bottom_z = plate_surface_z - 0.003
    bearing_top_z = bearing_bottom_z + bearing_h
    bearing_y = plate_surface_y + 0.012
    bearing = (
        cq.Workplane("XY")
        .circle(0.017)
        .extrude(bearing_h)
    )
    cabinet.visual(
        mesh_from_cadquery(bearing, "spinner_bearing"),
        origin=Origin(xyz=(0.0, bearing_y, bearing_bottom_z)),
        material=mats["spinner_bearing"],
        name="spinner_bearing",
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

    # ---- Spinner (the articulated control) ---------------------------------
    # Authored in its own local frame with local z=0 at the visible top face of
    # the fixed bearing. The stem extends downward into that bearing, while the
    # knurled knob and raised paddle grip sit above it and spin about world Z.
    spinner = model.part("spinner")
    stem = cq.Workplane("XY").circle(0.0065).extrude(0.014)
    spinner.visual(
        mesh_from_cadquery(stem, "spinner_stem"),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=mats["spinner_metal"],
        name="spinner_stem",
    )
    spinner.visual(
        mesh_from_cadquery(_build_knurled_spinner_body(), "spinner_body"),
        origin=Origin(),
        material=mats["spinner_blue"],
        name="spinner_body",
    )

    # Continuous joint: mounted on the fixed bearing top surface, with a true
    # vertical Z-axis spin like a paddle/spinner controller.
    pivot_world = (0.0, bearing_y, bearing_top_z)
    model.articulation(
        "panel_to_spinner",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=spinner,
        origin=Origin(xyz=pivot_world),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet_body")
    spinner = object_model.get_part("spinner")
    joint = object_model.get_articulation("panel_to_spinner")

    # The spinner's metal stem is intentionally captured inside the visible
    # bearing ring on the red control plate; the bearing is a simplified solid
    # bushing proxy, so this local shaft-in-bushing interpenetration is intended.
    ctx.allow_overlap(
        spinner,
        cabinet,
        elem_a="spinner_stem",
        elem_b="spinner_bearing",
        reason="The spinner stem is captured inside the fixed bearing/bushing on the control plate; the solid bushing proxy overlaps locally by design.",
    )

    # ---- Joint type / axis contract ----------------------------------------
    ctx.check(
        "spinner joint is continuous",
        str(getattr(joint.articulation_type, "value", joint.articulation_type)).lower().endswith("continuous"),
        f"type={joint.articulation_type!r}",
    )
    ctx.check(
        "spinner joint rotates about vertical Z",
        abs(joint.axis[0]) < 1e-6 and abs(joint.axis[1]) < 1e-6 and abs(abs(joint.axis[2]) - 1.0) < 1e-6,
        f"axis={joint.axis!r}",
    )
    lim = joint.motion_limits
    ctx.check(
        "spinner has no angular stops",
        lim is not None and lim.lower is None and lim.upper is None,
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

    # ---- Spinner mounting & geometry ---------------------------------------
    body = ctx.part_element_world_aabb(spinner, elem="spinner_body")
    stem = ctx.part_element_world_aabb(spinner, elem="spinner_stem")
    bearing = ctx.part_element_world_aabb(cabinet, elem="spinner_bearing")
    assert body is not None and stem is not None and bearing is not None
    body_w = body[1][0] - body[0][0]
    body_d = body[1][1] - body[0][1]
    ctx.check(
        "spinner is a broad low paddle knob, not a joystick stick",
        0.070 <= body_w <= 0.090 and 0.045 <= body_d <= 0.070 and (body[1][2] - body[0][2]) < 0.040,
        f"body_size={(body_w, body_d, body[1][2] - body[0][2])!r}",
    )
    ctx.expect_gap(
        spinner,
        cabinet,
        axis="z",
        positive_elem="spinner_body",
        negative_elem="spinner_bearing",
        max_gap=0.002,
        max_penetration=0.003,
        name="spinner knob seats on bearing top",
    )
    ctx.expect_within(
        spinner,
        cabinet,
        axes="xy",
        inner_elem="spinner_stem",
        outer_elem="spinner_bearing",
        margin=0.004,
        name="spinner stem stays centered in bearing",
    )
    ctx.expect_overlap(
        spinner,
        cabinet,
        axes="z",
        elem_a="spinner_stem",
        elem_b="spinner_bearing",
        min_overlap=0.008,
        name="spinner stem remains captured in bearing",
    )

    # ---- Decisive articulated pose: spinning changes paddle orientation -----
    rest_body = ctx.part_element_world_aabb(spinner, elem="spinner_body")
    assert rest_body is not None
    rest_w = rest_body[1][0] - rest_body[0][0]
    rest_d = rest_body[1][1] - rest_body[0][1]
    with ctx.pose({joint: math.pi / 2.0}):
        turned_body = ctx.part_element_world_aabb(spinner, elem="spinner_body")
        assert turned_body is not None
        turned_w = turned_body[1][0] - turned_body[0][0]
        turned_d = turned_body[1][1] - turned_body[0][1]
    ctx.check(
        "quarter turn spins the paddle footprint",
        turned_w < rest_w - 0.010 and turned_d > rest_d + 0.010,
        f"rest=(w={rest_w:.3f},d={rest_d:.3f}) turned=(w={turned_w:.3f},d={turned_d:.3f})",
    )

    return ctx.report()


object_model = build_object_model()
