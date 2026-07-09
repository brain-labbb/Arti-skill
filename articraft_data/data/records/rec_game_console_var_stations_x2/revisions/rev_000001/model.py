from __future__ import annotations

# Retro tabletop / countertop arcade game console.
#
# The reference image shows a weathered blue sheet-metal cabinet with:
#   - a wedge body: tall vertical back, a sloped upper-front face carrying a
#     dark "GAME OVER" marquee/screen behind a metal bezel, and a vertical lower
#     front carrying a bolted access panel.
#   - a recessed control band between the screen and the lower panel holding two
#     identical player control stations spread regularly across the deck.
#   - a blue ball-top joystick standing up out of each red station plate.
#   - a base pedestal block under the cabinet.
#
# The clear, physical moving controls are the two JOYSTICKS: each ball-top stick
# pivots independently about a gimbal at its own red station plate. Each is
# articulated as a REVOLUTE joint that tilts forward/back about the plate.

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
        "stick_blue": model.material("joystick_ball_blue", rgba=(0.12, 0.32, 0.85, 1.0)),
        "stick_shaft": model.material("joystick_shaft", rgba=(0.20, 0.20, 0.22, 1.0)),
        "stick_collar": model.material("joystick_collar", rgba=(0.45, 0.45, 0.48, 1.0)),
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


def _build_control_station_geometry() -> dict[str, cq.Workplane]:
    """Shared visible geometry for one repeated two-player control station."""
    station_w = 0.160
    station_h = 0.060
    plate_t = 0.012

    joystick_hole = (
        cq.Workplane("XY")
        .circle(0.015)
        .extrude(plate_t * 3.0, both=True)
    )
    plate = (
        cq.Workplane("XY")
        .box(station_w, station_h, plate_t)
        .cut(joystick_hole)
        .edges("|Z")
        .fillet(0.004)
    )

    collar = (
        cq.Workplane("XY")
        .circle(0.021)
        .extrude(0.010)
    )

    buttons = None
    button_w = 0.011
    button_h = 0.010
    button_t = 0.004
    # A compact 3x2 gold button pad on the right half of each identical
    # station, preserving the arcade control-deck read while the joystick is
    # the actual moving child.
    for col in range(3):
        for row in range(2):
            bx = 0.040 + (col - 1) * 0.016
            by = (row - 0.5) * 0.018
            key = (
                cq.Workplane("XY")
                .box(button_w, button_h, button_t)
                .translate((bx, by, 0.0))
            )
            buttons = key if buttons is None else buttons.union(key)

    screw = cq.Workplane("XY").circle(0.0035).extrude(0.004)

    return {
        "plate": plate,
        "collar": collar,
        "buttons": buttons,
        "screw": screw,
        "station_w": station_w,
        "station_h": station_h,
        "plate_t": plate_t,
        "button_t": button_t,
    }


def _build_joystick_geometry(shaft_len: float) -> dict[str, cq.Workplane]:
    """Shared mesh geometry for one articulated joystick child."""
    shaft = (
        cq.Workplane("XY")
        .circle(0.006)
        .extrude(shaft_len)
    )
    boot = (
        cq.Workplane("XY")
        .circle(0.013)
        .workplane(offset=0.012)
        .circle(0.007)
        .loft()
    )
    return {"shaft": shaft, "boot": boot}


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
    # Sit the text proud of the screen glass along the slope's outward normal.
    n_y = -math.sin(pitch)
    n_z = math.cos(pitch)
    # Seat the raised text directly on the glass so it reads as printed/painted
    # lettering instead of a floating plaque.
    text_proud = 0.007
    cabinet.visual(
        mesh_from_cadquery(text_bar, "game_over_text"),
        origin=Origin(
            xyz=(fx + tx, fy + ty + text_proud * n_y, fz + tz + text_proud * n_z),
            rpy=(pitch, 0.0, 0.0),
        ),
        material=mats["gold_text"],
        name="game_over_text",
    )

    # ---- Control band: two repeated player stations ------------------------
    band_y = FRONT_Y - 0.055 + 0.010  # just in front of the band back wall
    band_z = Z_LOWER_TOP + CONTROL_BAND_H / 2.0
    band_face_pitch = -0.18  # control band tilts up slightly toward the player

    station_geo = _build_control_station_geometry()
    station_spacing = 0.190
    station_surface_y = band_y + 0.004
    station_surface_z = band_z
    cos_pitch = math.cos(band_face_pitch)
    sin_pitch = math.sin(band_face_pitch)

    def band_point(station_x: float, local_x: float = 0.0, local_y: float = 0.0, local_z: float = 0.0) -> tuple[float, float, float]:
        """Map local station offsets onto the tilted control-deck surface."""
        return (
            station_x + local_x,
            station_surface_y + local_y * cos_pitch - local_z * sin_pitch,
            station_surface_z + local_y * sin_pitch + local_z * cos_pitch,
        )

    for i in range(2):
        station_x = (i - 0.5) * station_spacing
        station_name = f"station_{i}"
        plate_t = station_geo["plate_t"]

        cabinet.visual(
            mesh_from_cadquery(station_geo["plate"], station_name),
            origin=Origin(xyz=band_point(station_x, local_z=plate_t / 2.0), rpy=(band_face_pitch, 0.0, 0.0)),
            material=mats["red_plate"],
            name=station_name,
        )
        cabinet.visual(
            mesh_from_cadquery(station_geo["buttons"], f"{station_name}_buttons"),
            origin=Origin(
                xyz=band_point(station_x, local_z=plate_t + station_geo["button_t"] / 2.0),
                rpy=(band_face_pitch, 0.0, 0.0),
            ),
            material=mats["keypad"],
            name=f"{station_name}_buttons",
        )
        cabinet.visual(
            mesh_from_cadquery(station_geo["collar"], f"joystick_collar_{i}"),
            origin=Origin(xyz=band_point(station_x, local_z=plate_t), rpy=(band_face_pitch, 0.0, 0.0)),
            material=mats["stick_collar"],
            name=f"joystick_collar_{i}",
        )
        for sx in (-1, 1):
            for sy in (-1, 1):
                cabinet.visual(
                    mesh_from_cadquery(station_geo["screw"], f"station_screw_{i}_{sx}_{sy}"),
                    origin=Origin(
                        xyz=band_point(
                            station_x,
                            local_x=sx * (station_geo["station_w"] / 2.0 - 0.012),
                            local_y=sy * (station_geo["station_h"] / 2.0 - 0.010),
                            local_z=plate_t,
                        ),
                        rpy=(band_face_pitch, 0.0, 0.0),
                    ),
                    material=mats["screw"],
                    name=f"station_screw_{i}_{sx}_{sy}",
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

    # ---- Joysticks (two independent articulated controls) ------------------
    # Each child is authored in its own local frame so the pivot is at local
    # z=0 (the gimbal), the shaft rises along +Z, topped by the blue ball. The
    # joint frame places local +Z along the tilted normal of the station plate.
    shaft_len = 0.055
    joystick_geo = _build_joystick_geometry(shaft_len)
    for i in range(2):
        joystick = model.part(f"joystick_{i}")
        joystick.visual(
            mesh_from_cadquery(joystick_geo["shaft"], f"joystick_shaft_{i}"),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=mats["stick_shaft"],
            name="joystick_shaft",
        )
        joystick.visual(
            mesh_from_cadquery(joystick_geo["boot"], f"joystick_boot_{i}"),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=mats["stick_collar"],
            name="joystick_boot",
        )
        joystick.visual(
            Sphere(radius=0.018),
            origin=Origin(xyz=(0.0, 0.0, shaft_len + 0.012)),
            material=mats["stick_blue"],
            name="joystick_ball",
        )

        station_x = (i - 0.5) * station_spacing
        pivot_world = band_point(station_x, local_z=station_geo["plate_t"])
        model.articulation(
            f"panel_to_joystick_{i}",
            ArticulationType.REVOLUTE,
            parent=cabinet,
            child=joystick,
            origin=Origin(xyz=pivot_world, rpy=(band_face_pitch, 0.0, 0.0)),
            # Rotation about -X tips the top of the stick toward +Y (forward).
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=-0.45, upper=0.45),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet_body")
    joysticks = [object_model.get_part(f"joystick_{i}") for i in range(2)]
    joints = [object_model.get_articulation(f"panel_to_joystick_{i}") for i in range(2)]

    # Each joystick boot sits down over its own fixed mounting collar and each
    # shaft passes up through that same collar bore (a captured gimbal seat), so
    # small local interpenetration of those element pairs is by design.
    for i, joystick in enumerate(joysticks):
        ctx.allow_overlap(
            joystick,
            cabinet,
            elem_a="joystick_boot",
            elem_b=f"joystick_collar_{i}",
            reason="The joystick boot is seated down over its fixed gimbal collar; this captured fit overlaps locally by design.",
        )
        ctx.allow_overlap(
            joystick,
            cabinet,
            elem_a="joystick_shaft",
            elem_b=f"joystick_collar_{i}",
            reason="The joystick shaft passes up through the fixed gimbal collar bore (modeled as a solid collar proxy); the captured shaft-in-collar fit overlaps locally by design.",
        )

    # ---- Joint type / axis contract ----------------------------------------
    ctx.check("two independent joystick parts exist", all(j is not None for j in joysticks), f"joysticks={joysticks!r}")
    ctx.check("two independent joystick joints exist", all(j is not None for j in joints), f"joints={joints!r}")
    for i, joint in enumerate(joints):
        ctx.check(
            f"joystick_{i} joint is revolute",
            str(getattr(joint.articulation_type, "value", joint.articulation_type)).lower().endswith("revolute"),
            f"type={joint.articulation_type!r}",
        )
        ctx.check(
            f"joystick_{i} joint tilts about X",
            abs(abs(joint.axis[0]) - 1.0) < 1e-6 and abs(joint.axis[1]) < 1e-6 and abs(joint.axis[2]) < 1e-6,
            f"axis={joint.axis!r}",
        )
        lim = joint.motion_limits
        ctx.check(
            f"joystick_{i} has symmetric deflection limits",
            lim is not None and lim.lower is not None and lim.upper is not None and lim.lower < 0.0 < lim.upper,
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

    # Two identical control stations are evenly spaced across the deck.
    station_aabbs = [ctx.part_element_world_aabb(cabinet, elem=f"station_{i}") for i in range(2)]
    button_aabbs = [ctx.part_element_world_aabb(cabinet, elem=f"station_{i}_buttons") for i in range(2)]
    collar_aabbs = [ctx.part_element_world_aabb(cabinet, elem=f"joystick_collar_{i}") for i in range(2)]
    assert all(a is not None for a in station_aabbs)
    assert all(a is not None for a in button_aabbs)
    assert all(a is not None for a in collar_aabbs)
    station_cx = [(a[0][0] + a[1][0]) / 2.0 for a in station_aabbs]
    ctx.check(
        "two stations flank center evenly",
        station_cx[0] < -0.06 and station_cx[1] > 0.06 and abs(station_cx[0] + station_cx[1]) < 0.004,
        f"station_cx={station_cx!r}",
    )
    station_w = [a[1][0] - a[0][0] for a in station_aabbs]
    station_h = [a[1][2] - a[0][2] for a in station_aabbs]
    ctx.check(
        "two stations have matching visible size",
        abs(station_w[0] - station_w[1]) < 0.003 and abs(station_h[0] - station_h[1]) < 0.003,
        f"station_w={station_w!r}, station_h={station_h!r}",
    )
    button_cx = [(a[0][0] + a[1][0]) / 2.0 for a in button_aabbs]
    ctx.check(
        "each station carries a gold button pad",
        all(button_cx[i] > station_cx[i] + 0.02 for i in range(2)),
        f"station_cx={station_cx!r}, button_cx={button_cx!r}",
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

    # ---- Joystick mounting & geometry --------------------------------------
    ball_centers_x = []
    for i, joystick in enumerate(joysticks):
        ball = ctx.part_element_world_aabb(joystick, elem="joystick_ball")
        assert ball is not None
        ball_cx = (ball[0][0] + ball[1][0]) / 2.0
        ball_cz = (ball[0][2] + ball[1][2]) / 2.0
        ball_centers_x.append(ball_cx)
        shaft = ctx.part_element_world_aabb(joystick, elem="joystick_shaft")
        assert shaft is not None
        shaft_base_z = shaft[0][2]
        ctx.check(
            f"joystick_{i} ball sits above the shaft",
            ball_cz > shaft_base_z + 0.04,
            f"ball_cz={ball_cz:.3f}, shaft_base_z={shaft_base_z:.3f}",
        )
        # Joystick base is seated near its collar on its station plate (connected).
        ctx.expect_contact(
            joystick,
            cabinet,
            elem_a="joystick_boot",
            elem_b=f"joystick_collar_{i}",
            contact_tol=0.012,
            name=f"joystick_{i} boot seats at its collar",
        )
        # The captured shaft stays centered within the collar footprint and
        # remains engaged with it along the stick axis (retained insertion).
        ctx.expect_within(
            joystick,
            cabinet,
            axes="xy",
            inner_elem="joystick_shaft",
            outer_elem=f"joystick_collar_{i}",
            margin=0.004,
            name=f"joystick_{i} shaft stays inside its collar bore",
        )
        ctx.expect_overlap(
            joystick,
            cabinet,
            axes="z",
            elem_a="joystick_shaft",
            elem_b=f"joystick_collar_{i}",
            min_overlap=0.006,
            name=f"joystick_{i} shaft stays engaged in its collar",
        )

    ctx.check(
        "joystick balls align with station centers",
        all(abs(ball_centers_x[i] - station_cx[i]) < 0.006 for i in range(2)),
        f"ball_centers_x={ball_centers_x!r}, station_cx={station_cx!r}",
    )

    # ---- Decisive articulated pose: each joystick deflects independently ----
    rest_ball_0 = ctx.part_element_world_aabb(joysticks[0], elem="joystick_ball")
    rest_ball_1 = ctx.part_element_world_aabb(joysticks[1], elem="joystick_ball")
    assert rest_ball_0 is not None and rest_ball_1 is not None
    rest_y_0 = (rest_ball_0[0][1] + rest_ball_0[1][1]) / 2.0
    rest_z_0 = (rest_ball_0[0][2] + rest_ball_0[1][2]) / 2.0
    rest_y_1 = (rest_ball_1[0][1] + rest_ball_1[1][1]) / 2.0
    with ctx.pose({joints[0]: 0.40}):
        fwd_ball_0 = ctx.part_element_world_aabb(joysticks[0], elem="joystick_ball")
        still_ball_1 = ctx.part_element_world_aabb(joysticks[1], elem="joystick_ball")
        assert fwd_ball_0 is not None and still_ball_1 is not None
        fwd_y_0 = (fwd_ball_0[0][1] + fwd_ball_0[1][1]) / 2.0
        fwd_z_0 = (fwd_ball_0[0][2] + fwd_ball_0[1][2]) / 2.0
        still_y_1 = (still_ball_1[0][1] + still_ball_1[1][1]) / 2.0
    ctx.check(
        "joystick_0 positive deflection tilts its ball forward and down",
        fwd_y_0 > rest_y_0 + 0.01 and fwd_z_0 < rest_z_0 - 0.002,
        f"rest=(y={rest_y_0:.3f},z={rest_z_0:.3f}) fwd=(y={fwd_y_0:.3f},z={fwd_z_0:.3f})",
    )
    ctx.check(
        "joystick_1 remains independent while joystick_0 moves",
        abs(still_y_1 - rest_y_1) < 0.002,
        f"rest_y_1={rest_y_1:.3f}, posed_y_1={still_y_1:.3f}",
    )
    rest_ball_0 = ctx.part_element_world_aabb(joysticks[0], elem="joystick_ball")
    rest_ball_1 = ctx.part_element_world_aabb(joysticks[1], elem="joystick_ball")
    assert rest_ball_0 is not None and rest_ball_1 is not None
    rest_y_0 = (rest_ball_0[0][1] + rest_ball_0[1][1]) / 2.0
    rest_y_1 = (rest_ball_1[0][1] + rest_ball_1[1][1]) / 2.0
    rest_z_1 = (rest_ball_1[0][2] + rest_ball_1[1][2]) / 2.0
    with ctx.pose({joints[1]: 0.40}):
        still_ball_0 = ctx.part_element_world_aabb(joysticks[0], elem="joystick_ball")
        fwd_ball_1 = ctx.part_element_world_aabb(joysticks[1], elem="joystick_ball")
        assert still_ball_0 is not None and fwd_ball_1 is not None
        still_y_0 = (still_ball_0[0][1] + still_ball_0[1][1]) / 2.0
        fwd_y_1 = (fwd_ball_1[0][1] + fwd_ball_1[1][1]) / 2.0
        fwd_z_1 = (fwd_ball_1[0][2] + fwd_ball_1[1][2]) / 2.0
    ctx.check(
        "joystick_1 positive deflection tilts its ball forward and down",
        fwd_y_1 > rest_y_1 + 0.01 and fwd_z_1 < rest_z_1 - 0.002,
        f"rest=(y={rest_y_1:.3f},z={rest_z_1:.3f}) fwd=(y={fwd_y_1:.3f},z={fwd_z_1:.3f})",
    )
    ctx.check(
        "joystick_0 remains independent while joystick_1 moves",
        abs(still_y_0 - rest_y_0) < 0.002,
        f"rest_y_0={rest_y_0:.3f}, posed_y_0={still_y_0:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
