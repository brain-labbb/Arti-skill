from __future__ import annotations

# Retro tabletop / countertop arcade game console.
#
# The reference image shows a weathered blue sheet-metal cabinet with the same
# arcade-console layers as the parent asset, but this fork uses the requested
# upright cabinet structure:
#   - a straight rectilinear box body with a flat vertical front carrying the
#     dark "GAME OVER" marquee/screen behind a metal bezel and the lower bolted
#     access panel.
#   - a flat horizontal control shelf projecting from that vertical front,
#     holding two gold keypad button grids that flank a central red control
#     plate.
#   - a blue ball-top joystick standing up out of the red control plate.
#   - a base pedestal block under the cabinet.
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
# Overall scale (meters). Compact countertop arcade cabinet.
# ---------------------------------------------------------------------------
CAB_W = 0.420  # left-right width
CAB_D = 0.460  # front-back depth (back face is the deepest extent)
CAB_H = 0.560  # body height (excluding base pedestal)

WALL = 0.014  # sheet-metal shell wall thickness

# The front is now a single flat vertical plane; the lower portion carries the
# access panel, the upper portion carries the screen, and a horizontal shelf
# projects from the seam where the old sloped control band used to be.
LOWER_FRONT_H = 0.250  # height of the vertical lower front face
CONTROL_BAND_H = 0.085  # height of the recessed control band

# Front of cabinet is at +Y, back at -Y. Center the body on X and put the body
# floor at z = base_h so the pedestal sits beneath it.
BASE_H = 0.045
BASE_INSET = 0.000  # flush plinth reaches the upright shell walls

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
        "stick_blue": model.material("joystick_ball_blue", rgba=(0.12, 0.32, 0.85, 1.0)),
        "stick_shaft": model.material("joystick_shaft", rgba=(0.20, 0.20, 0.22, 1.0)),
        "stick_collar": model.material("joystick_collar", rgba=(0.45, 0.45, 0.48, 1.0)),
    }


# ---------------------------------------------------------------------------
# Cabinet shell, built as a hollow upright box in CadQuery.
# ---------------------------------------------------------------------------
def _build_cabinet_shell() -> cq.Workplane:
    """Hollow rectilinear cabinet shell.

    Built in a frame whose origin sits at the body floor center
    (x=0, y=0, z=Z_FLOOR maps to local z=0). The side profile (in the Y-Z
    plane) is a rectangle: vertical back, flat roof, vertical front, and floor.
    """
    # Side profile points (y, z) in local coords with z measured from floor.
    top_h = CAB_H
    side = [
        (BACK_Y, 0.0),  # back-bottom
        (BACK_Y, top_h),  # back-top
        (FRONT_Y, top_h),  # front-top, making the front face vertical
        (FRONT_Y, 0.0),  # front-bottom
    ]

    outer = (
        cq.Workplane("YZ")
        .polyline(side)
        .close()
        .extrude(CAB_W / 2.0, both=True)
    )

    # Hollow it out: a smaller inset box subtracted, leaving the shell walls.
    # The inset cuts through the underside so the inner and outer sheet-metal
    # surfaces share a visible bottom rim instead of becoming separate mesh
    # surface islands; the dark pedestal below hides the open underside.
    inner_side = [
        (BACK_Y + WALL, -WALL),
        (BACK_Y + WALL, top_h - WALL),
        (FRONT_Y - WALL, top_h - WALL),
        (FRONT_Y - WALL, -WALL),
    ]
    inner = (
        cq.Workplane("YZ")
        .polyline(inner_side)
        .close()
        .extrude((CAB_W / 2.0) - WALL, both=True)
    )
    shell = outer.cut(inner)

    # Cut the screen opening in the vertical front face and the access-panel
    # recess in the lower front face are handled as separate applied visuals
    # (dark inset plates), so the shell stays a clean hollow box.
    shell = shell.edges("|X").fillet(0.006)
    return shell


def _front_screen_geometry() -> tuple[tuple[float, float, float], float, float]:
    """Return (center_xyz, roll, height) of the flat vertical screen face."""
    screen_h = min(0.205, Z_TOP - Z_CONTROL_TOP - 0.030)
    cy = FRONT_Y + 0.001
    cz = Z_CONTROL_TOP + 0.020 + screen_h / 2.0
    # A panel authored in XY is stood on the front by a +90 degree roll: local
    # X remains world X, local Y becomes world Z, and thickness becomes Y.
    return (0.0, cy, cz), math.pi / 2.0, screen_h


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

    # Base pedestal under the body; it reaches the cabinet footprint so the
    # open-bottom sheet-metal shell visibly rests on the dark plinth.
    cabinet.visual(
        Box((CAB_W - 2 * BASE_INSET, CAB_D - 2 * BASE_INSET, BASE_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_H / 2.0)),
        material=mats["dark_trim"],
        name="base_pedestal",
    )

    # ---- Vertical front screen: bezel + dark screen + GAME OVER text -------
    (fx, fy, fz), front_roll, screen_h = _front_screen_geometry()
    face_w = CAB_W - 2 * 0.030  # leave side margins

    # Dark metal bezel frame around the screen, with a true central opening
    # rather than a solid slab over the glass.
    bezel_outer = cq.Workplane("XY").box(face_w + 0.030, screen_h + 0.030, 0.010)
    bezel_inner = cq.Workplane("XY").box(face_w - 0.012, screen_h - 0.012, 0.014)
    bezel = bezel_outer.cut(bezel_inner)
    cabinet.visual(
        mesh_from_cadquery(bezel, "screen_bezel"),
        origin=Origin(xyz=(fx, fy, fz), rpy=(front_roll, 0.0, 0.0)),
        material=mats["dark_trim"],
        name="screen_bezel",
    )
    # Dark recessed screen.
    screen = (
        cq.Workplane("XY")
        .box(face_w - 0.020, screen_h - 0.020, 0.008)
    )
    cabinet.visual(
        mesh_from_cadquery(screen, "screen_glass"),
        origin=Origin(xyz=(fx, fy + 0.003, fz), rpy=(front_roll, 0.0, 0.0)),
        material=mats["screen"],
        name="screen_glass",
    )
    # "GAME OVER" gold text bar (a slim raised plate reading as the lit text).
    text_bar = (
        cq.Workplane("XY")
        .box(face_w * 0.62, screen_h * 0.15, 0.004)
    )
    # Offset the text toward the upper part of the now-vertical screen face.
    tx = face_w * 0.06
    tz = screen_h * 0.12
    cabinet.visual(
        mesh_from_cadquery(text_bar, "game_over_text"),
        origin=Origin(
            xyz=(fx + tx, fy + 0.007, fz + tz),
            rpy=(front_roll, 0.0, 0.0),
        ),
        material=mats["gold_text"],
        name="game_over_text",
    )

    # ---- Horizontal control shelf: two keypad grids + central red plate ----
    shelf_w = CAB_W - 0.030
    shelf_d = 0.145
    shelf_t = 0.018
    shelf_top_z = Z_CONTROL_TOP
    shelf_y = FRONT_Y + shelf_d / 2.0 - 0.010  # back edge embeds in front wall
    control_y = FRONT_Y + shelf_d * 0.52
    control_shelf = (
        cq.Workplane("XY")
        .box(shelf_w, shelf_d, shelf_t)
        .edges("|Z")
        .fillet(0.006)
    )
    cabinet.visual(
        mesh_from_cadquery(control_shelf, "control_shelf"),
        origin=Origin(xyz=(0.0, shelf_y, shelf_top_z - shelf_t / 2.0)),
        material=mats["blue_metal"],
        name="control_shelf",
    )
    # Dark front lip under the shelf, matching the worn black edge trim.
    cabinet.visual(
        Box((shelf_w, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, FRONT_Y + shelf_d - 0.010, shelf_top_z - 0.030)),
        material=mats["dark_trim"],
        name="shelf_front_lip",
    )

    keypad_w = 0.140
    keypad_h = 0.052
    keypad_t = 0.010
    for i in range(2):
        side_sign = -1.0 if i == 0 else 1.0
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
            mesh_from_cadquery(keypad, f"keypad_{i}"),
            origin=Origin(xyz=(kx, control_y, shelf_top_z + keypad_t / 2.0)),
            material=mats["keypad"],
            name=f"keypad_{i}",
        )

    # Central red control plate (recessed pocket holding the joystick).
    red_w = 0.072
    red_h = 0.060
    red_plate = (
        cq.Workplane("XY")
        .box(red_w, red_h, 0.012)
        .edges("|Z")
        .fillet(0.004)
    )
    # Plate top sits on the flat horizontal shelf. Joystick base mounts here.
    plate_t = 0.012
    plate_xyz = (0.0, control_y, shelf_top_z + plate_t / 2.0)
    cabinet.visual(
        mesh_from_cadquery(red_plate, "control_red_plate"),
        origin=Origin(xyz=plate_xyz),
        material=mats["red_plate"],
        name="control_red_plate",
    )
    # Raised gimbal collar the stick passes through (mount + hides the pivot).
    collar = (
        cq.Workplane("XY")
        .circle(0.014)
        .extrude(0.010)
    )
    pivot_world = (0.0, control_y, shelf_top_z + plate_t)
    cabinet.visual(
        mesh_from_cadquery(collar, "joystick_collar"),
        origin=Origin(xyz=pivot_world),
        material=mats["stick_collar"],
        name="joystick_collar",
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
    screw_offsets = [(-1, 1), (1, 1), (-1, -1), (1, -1)]
    for i in range(4):
        sx, sz = screw_offsets[i]
        scx = sx * (panel_w / 2.0 - 0.018)
        scz = panel_cz + sz * (panel_h / 2.0 - 0.018)
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
    # the shaft rises along +Z, topped by the blue ball. The joint frame places
    # this local +Z vertical on the horizontal control shelf.
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

    # Joint: pivot at the top of the control plate, with local +Z vertical.
    # Positive q tilts the stick forward (toward +Y) about the X axis.
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
        "cabinet is a tabletop upright box with shelf",
        0.40 <= csize[0] <= 0.46 and 0.52 <= csize[1] <= 0.65 and 0.55 <= csize[2] <= 0.65,
        f"size={csize!r}",
    )
    shell_aabb = ctx.part_element_world_aabb(cabinet, elem="cabinet_shell")
    assert shell_aabb is not None
    sh_min, sh_max = shell_aabb
    shell_size = tuple(sh_max[i] - sh_min[i] for i in range(3))
    ctx.check(
        "cabinet shell has a rectilinear box footprint",
        abs(shell_size[0] - CAB_W) < 0.020
        and abs(shell_size[1] - CAB_D) < 0.020
        and abs(shell_size[2] - CAB_H) < 0.020,
        f"shell_size={shell_size!r}",
    )

    # Screen sits high on the flat vertical front face.
    screen_aabb = ctx.part_element_world_aabb(cabinet, elem="screen_glass")
    assert screen_aabb is not None
    smin, smax = screen_aabb
    screen_cz = (smin[2] + smax[2]) / 2.0
    screen_size = tuple(smax[i] - smin[i] for i in range(3))
    ctx.check(
        "screen is on the upper half of the cabinet",
        screen_cz > cmins[2] + 0.62 * csize[2],
        f"screen_cz={screen_cz:.3f}, cab z=[{cmins[2]:.3f},{cmaxs[2]:.3f}]",
    )
    ctx.check(
        "screen is mounted on the flat vertical front",
        screen_size[2] > 0.16 and screen_size[1] < 0.020 and abs(smax[1] - FRONT_Y) < 0.025,
        f"screen_size={screen_size!r}, screen_y=({smin[1]:.3f},{smax[1]:.3f}), front_y={FRONT_Y:.3f}",
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

    # Two keypad grids flank the center on opposite sides of X on a horizontal shelf.
    shelf = ctx.part_element_world_aabb(cabinet, elem="control_shelf")
    assert shelf is not None
    shelf_size = tuple(shelf[1][i] - shelf[0][i] for i in range(3))
    ctx.check(
        "controls sit on a flat horizontal shelf",
        shelf_size[1] > 0.12 and shelf_size[2] < 0.030,
        f"shelf_size={shelf_size!r}",
    )
    plate = ctx.part_element_world_aabb(cabinet, elem="control_red_plate")
    assert plate is not None
    ctx.check(
        "red control plate rests on the horizontal shelf top",
        abs(plate[0][2] - shelf[1][2]) < 0.003,
        f"plate_bottom_z={plate[0][2]:.3f}, shelf_top_z={shelf[1][2]:.3f}",
    )
    kl = ctx.part_element_world_aabb(cabinet, elem="keypad_0")
    kr = ctx.part_element_world_aabb(cabinet, elem="keypad_1")
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
    for nm in ("access_screw_0", "access_screw_1", "access_screw_2", "access_screw_3"):
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
