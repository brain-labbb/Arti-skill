from __future__ import annotations

# Black Sony soundbar remote control.
# Slim rounded-rectangle matte-black wand.
# Frame:
#   X = long axis (length ~0.16 m). SONY logo end at -X, button cluster at +X.
#   Y = width (~0.042 m), front face up at +Z, thickness ~0.012 m.
# Front (top) face is the +Z broad face; all controls live on it, in the upper
# half (positive X). The big circular volume control is the cluster center.
# Articulations:
#   - ~9 round rubber buttons: each PRISMATIC, pressing straight into the body
#     (along -Z, ~1.2 mm travel).
#   - the large central circular volume control: REVOLUTE rocker tilting about a
#     center axis (its -/triangle/+ rocks about the Y axis).
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    DomeGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- body envelope ----
BODY_LEN = 0.160
BODY_WID = 0.042
BODY_THK = 0.012
FRONT_Z = BODY_THK / 2.0  # +0.006, the front face plane

# Button press travel (straight down into the body).
PRESS_TRAVEL = 0.0012

# Front-face geometry constants for buttons.
BTN_PROUD = 0.0016  # how far a button cap sits proud of the front face
WELL_DEPTH = 0.0018  # recessed well depth into the front face
WELL_FLOOR_Z = FRONT_Z - WELL_DEPTH  # z of the recessed well floor


def _body_solid() -> cq.Workplane:
    # Slim rounded-rectangle wand: a filleted box. Round the long edges that run
    # along X (the rounded-rectangle profile) and soften the short end caps.
    body = cq.Workplane("XY").box(BODY_LEN, BODY_WID, BODY_THK)
    # Round the short-end vertical (|Z) corners first (gives the wand its
    # rounded-rectangle ends), then soften the long edges running along X.
    body = body.edges("|Z").fillet(0.0090)
    body = body.edges("|X").fillet(0.0040)
    return body


def _carve_wells(body: cq.Workplane, wells) -> cq.Workplane:
    # Cut shallow circular wells into the front face so each button sits in a
    # depression rather than floating on a flat face.
    for cx, cy, well_r in wells:
        cutter = (
            cq.Workplane("XY")
            .workplane(offset=FRONT_Z + 0.001)
            .center(cx, cy)
            .circle(well_r)
            .extrude(-WELL_DEPTH - 0.001)
        )
        body = body.cut(cutter)
    return body


def _sony_logo_relief(body: cq.Workplane) -> cq.Workplane:
    # Deboss block "SONY" near the -X end on the front face by cutting thin
    # rectangular strokes into the surface.
    logo_x = -0.058
    logo_y = 0.0
    h = 0.006  # letter height (across Y)
    w = 0.005  # letter width (along X)
    gap = 0.0072
    depth = 0.0006
    t = 0.0011  # stroke thickness

    strokes = []

    def add(cx, cy, sx, sy):
        strokes.append((cx, cy, sx, sy))

    xs = [logo_x - 1.5 * gap, logo_x - 0.5 * gap, logo_x + 0.5 * gap, logo_x + 1.5 * gap]

    # S
    sx = xs[0]
    add(sx, logo_y + h / 2, w, t)
    add(sx, logo_y, w, t)
    add(sx, logo_y - h / 2, w, t)
    add(sx - w / 2, logo_y + h / 4, t, h / 2)
    add(sx + w / 2, logo_y - h / 4, t, h / 2)
    # O
    ox = xs[1]
    add(ox, logo_y + h / 2, w, t)
    add(ox, logo_y - h / 2, w, t)
    add(ox - w / 2, logo_y, t, h)
    add(ox + w / 2, logo_y, t, h)
    # N
    nx = xs[2]
    add(nx - w / 2, logo_y, t, h)
    add(nx + w / 2, logo_y, t, h)
    add(nx, logo_y, t, h * 0.95)
    # Y
    yx = xs[3]
    add(yx, logo_y - h / 4, t, h / 2)
    add(yx - w / 4, logo_y + h / 4, t, h / 2)
    add(yx + w / 4, logo_y + h / 4, t, h / 2)

    for cx, cy, sx2, sy2 in strokes:
        cutter = (
            cq.Workplane("XY")
            .workplane(offset=FRONT_Z + 0.0005)
            .center(cx, cy)
            .rect(sx2, sy2)
            .extrude(-depth - 0.0005)
        )
        body = body.cut(cutter)
    return body


def _round_button_mesh(name: str, radius: float, height: float):
    # Short cylinder shaft with a soft flattened dome cap (rubber button proud).
    shaft = CylinderGeometry(radius, height, radial_segments=32)
    shaft.translate(0.0, 0.0, height / 2.0)
    cap = DomeGeometry(radius, radial_segments=32, height_segments=10)
    cap.scale(1.0, 1.0, 0.35)
    cap.translate(0.0, 0.0, height)
    shaft.merge(cap)
    return mesh_from_geometry(shaft, name)


def _oval_button_mesh(name: str, rx: float, ry: float, height: float):
    # Oval rubber pad: unit cylinder scaled in X/Y plus a flattened dome cap.
    shaft = CylinderGeometry(1.0, height, radial_segments=40)
    shaft.scale(rx, ry, 1.0)
    shaft.translate(0.0, 0.0, height / 2.0)
    cap = DomeGeometry(1.0, radial_segments=40, height_segments=10)
    cap.scale(rx, ry, height * 0.5)
    cap.translate(0.0, 0.0, height)
    shaft.merge(cap)
    return mesh_from_geometry(shaft, name)


_PIXEL_FONT = {
    "A": ("01110", "10001", "10001", "11111", "10001", "10001", "10001"),
    "B": ("11110", "10001", "10001", "11110", "10001", "10001", "11110"),
    "C": ("01111", "10000", "10000", "10000", "10000", "10000", "01111"),
    "D": ("11110", "10001", "10001", "10001", "10001", "10001", "11110"),
    "E": ("11111", "10000", "10000", "11110", "10000", "10000", "11111"),
    "F": ("11111", "10000", "10000", "11110", "10000", "10000", "10000"),
    "G": ("01111", "10000", "10000", "10011", "10001", "10001", "01111"),
    "H": ("10001", "10001", "10001", "11111", "10001", "10001", "10001"),
    "I": ("11111", "00100", "00100", "00100", "00100", "00100", "11111"),
    "L": ("10000", "10000", "10000", "10000", "10000", "10000", "11111"),
    "M": ("10001", "11011", "10101", "10101", "10001", "10001", "10001"),
    "N": ("10001", "11001", "10101", "10011", "10001", "10001", "10001"),
    "O": ("01110", "10001", "10001", "10001", "10001", "10001", "01110"),
    "P": ("11110", "10001", "10001", "11110", "10000", "10000", "10000"),
    "S": ("01111", "10000", "10000", "01110", "00001", "00001", "11110"),
    "T": ("11111", "00100", "00100", "00100", "00100", "00100", "00100"),
    "U": ("10001", "10001", "10001", "10001", "10001", "10001", "01110"),
    "V": ("10001", "10001", "10001", "10001", "10001", "01010", "00100"),
    "Y": ("10001", "10001", "01010", "00100", "00100", "00100", "00100"),
    " ": ("00000", "00000", "00000", "00000", "00000", "00000", "00000"),
}


def _merge_meshes(meshes: list[MeshGeometry]) -> MeshGeometry:
    if not meshes:
        return BoxGeometry((0.0001, 0.0001, 0.0001))
    merged = meshes[0]
    for mesh in meshes[1:]:
        merged.merge(mesh)
    return merged


def _stroke_box(
    sx: float,
    sy: float,
    sz: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    angle: float = 0.0,
) -> MeshGeometry:
    mesh = BoxGeometry((sx, sy, sz))
    if angle:
        mesh.rotate_z(angle)
    mesh.translate(x, y, z)
    return mesh


def _pixel_label_geometry(
    text: str,
    *,
    pixel: float,
    thickness: float,
    z: float,
    line_gap: float = 0.0010,
) -> MeshGeometry:
    # Tiny raised pixel lettering avoids font dependencies in the compile host.
    lines = text.upper().split("\n")
    char_gap = pixel
    box_size = pixel * 0.82
    line_height = 7 * pixel + line_gap
    meshes: list[MeshGeometry] = []

    widths = []
    for line in lines:
        if line:
            widths.append(len(line) * 5 * pixel + max(0, len(line) - 1) * char_gap)
        else:
            widths.append(0.0)
    total_h = len(lines) * 7 * pixel + max(0, len(lines) - 1) * line_gap

    for line_i, line in enumerate(lines):
        width = widths[line_i]
        base_x = -width / 2.0 + pixel / 2.0
        base_y = total_h / 2.0 - line_i * line_height - pixel / 2.0
        cursor_x = base_x
        for ch in line:
            glyph = _PIXEL_FONT.get(ch, _PIXEL_FONT[" "])
            for row_i, row in enumerate(glyph):
                for col_i, bit in enumerate(row):
                    if bit == "1":
                        meshes.append(
                            _stroke_box(
                                box_size,
                                box_size,
                                thickness,
                                x=cursor_x + col_i * pixel,
                                y=base_y - row_i * pixel,
                                z=z,
                            )
                        )
            cursor_x += 5 * pixel + char_gap
    return _merge_meshes(meshes)


def _pixel_label_mesh(name: str, text: str, **kwargs):
    return mesh_from_geometry(_pixel_label_geometry(text, **kwargs), name)


def _plus_icon_geometry(*, size: float, thickness: float, z: float) -> MeshGeometry:
    bar = size * 0.34
    meshes = [
        _stroke_box(size, bar, thickness, z=z),
        _stroke_box(bar, size, thickness, z=z),
    ]
    return _merge_meshes(meshes)


def _plus_icon_mesh(name: str, **kwargs):
    return mesh_from_geometry(_plus_icon_geometry(**kwargs), name)


def _minus_icon_geometry(*, size: float, thickness: float, z: float) -> MeshGeometry:
    return _stroke_box(size, size * 0.30, thickness, z=z)


def _minus_icon_mesh(name: str, **kwargs):
    return mesh_from_geometry(_minus_icon_geometry(**kwargs), name)


def _triangle_icon_geometry(*, size: float, thickness: float, z: float) -> MeshGeometry:
    # Outlined right-facing triangle, as on the original volume rocker.
    half = size / 2.0
    meshes = [
        _stroke_box(
            size * 0.82, size * 0.18, thickness, x=-size * 0.05, y=half * 0.48, z=z, angle=-0.64
        ),
        _stroke_box(
            size * 0.82, size * 0.18, thickness, x=-size * 0.05, y=-half * 0.48, z=z, angle=0.64
        ),
        _stroke_box(size * 0.80, size * 0.18, thickness, x=-half * 0.45, y=0.0, z=z, angle=1.5708),
    ]
    return _merge_meshes(meshes)


def _triangle_icon_mesh(name: str, **kwargs):
    return mesh_from_geometry(_triangle_icon_geometry(**kwargs), name)


def _speaker_icon_geometry(*, size: float, thickness: float, z: float) -> MeshGeometry:
    meshes = [
        _stroke_box(size * 0.26, size * 0.48, thickness, x=-size * 0.30, y=0.0, z=z),
        _stroke_box(
            size * 0.36, size * 0.18, thickness, x=-size * 0.05, y=size * 0.14, z=z, angle=-0.55
        ),
        _stroke_box(
            size * 0.36, size * 0.18, thickness, x=-size * 0.05, y=-size * 0.14, z=z, angle=0.55
        ),
        _stroke_box(size * 1.00, size * 0.18, thickness, x=size * 0.08, y=0.0, z=z, angle=0.78),
    ]
    return _merge_meshes(meshes)


def _speaker_icon_mesh(name: str, **kwargs):
    return mesh_from_geometry(_speaker_icon_geometry(**kwargs), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sony_soundbar_remote")

    # White/light-grey streaming-remote palette (companion colorway variant).
    body_white = model.material("body_white", rgba=(0.92, 0.92, 0.90, 1.0))
    btn_gray = model.material("button_gray", rgba=(0.72, 0.72, 0.74, 1.0))
    vol_gray = model.material("volume_gray", rgba=(0.78, 0.78, 0.80, 1.0))
    green = model.material("power_green", rgba=(0.20, 0.72, 0.30, 1.0))
    print_dark = model.material("print_dark", rgba=(0.15, 0.15, 0.16, 1.0))
    logo_gray = model.material("logo_gray", rgba=(0.40, 0.40, 0.42, 1.0))

    # ---- central volume control ----
    VOL_X = 0.040
    VOL_Y = 0.0
    VOL_R = 0.0150  # large central circular control radius

    # Buttons are laid out in neat two-button rows running down the length of the
    # wand (a column on +Y and a column on -Y), with the large volume control as
    # the top-middle anchor. This mirrors a real Sony soundbar remote and keeps
    # every cap clear of its neighbours and of the volume control.
    COL_Y = 0.0096  # left/right column offset from the wand centerline

    # ---- round rubber buttons (key, x, y, radius) ----
    # Compact streaming/media cluster: 4 buttons only.
    round_buttons = [
        # top row, above the volume control
        ("play", 0.064, COL_Y, 0.0060),
        ("power", 0.064, -COL_Y, 0.0060),
        # row below the volume control
        ("home", 0.016, COL_Y, 0.0060),
        ("voice", 0.016, -COL_Y, 0.0060),
    ]
    # ---- BASS +/- oval pads (key, x, y, rx, ry) ----
    # Dropped for the minimal streaming variant; loop emits nothing.
    oval_buttons = []

    round_wells = [(x, y, r + 0.0010) for (_, x, y, r) in round_buttons]
    oval_wells = [(x, y, max(rx, ry) + 0.0012) for (_, x, y, rx, ry) in oval_buttons]
    vol_well = [(VOL_X, VOL_Y, VOL_R + 0.0014)]

    # ---- body (root) ----
    body = model.part("body")
    body_wp = _body_solid()
    body_wp = _carve_wells(body_wp, round_wells + oval_wells + vol_well)
    body_wp = _sony_logo_relief(body_wp)
    body.visual(mesh_from_cadquery(body_wp, "body_shell"), material=body_white, name="body_shell")
    logo = _pixel_label_geometry("SONY", pixel=0.00075, thickness=0.00018, z=FRONT_Z + 0.00015)
    logo.translate(-0.058, 0.0, 0.0)
    body.visual(mesh_from_geometry(logo, "sony_print"), material=logo_gray, name="sony_print")
    body.inertial = Inertial.from_geometry(
        Box((BODY_LEN, BODY_WID, BODY_THK)), mass=0.085, origin=Origin(xyz=(0.0, 0.0, 0.0))
    )

    btn_height = BTN_PROUD + WELL_DEPTH  # cap proud of face + seated into the well

    # ---- round rubber buttons: each presses straight down (-Z) ----
    for key, x, y, r in round_buttons:
        part = model.part(f"btn_{key}")
        part.visual(
            _round_button_mesh(f"btn_{key}", r, btn_height),
            material=btn_gray,
            name=f"btn_{key}_cap",
        )
        if key == "power":
            ring_z = btn_height + r * 0.35 + 0.00020
            ring = TorusGeometry(0.0028, 0.00028, radial_segments=8, tubular_segments=36)
            ring.translate(0.0, -0.00025, ring_z)
            part.visual(
                mesh_from_geometry(ring, f"btn_{key}_ring"), material=green, name=f"btn_{key}_ring"
            )
            bar = _stroke_box(0.00055, 0.0024, 0.00020, x=0.0, y=0.00155, z=ring_z + 0.00002)
            part.visual(
                mesh_from_geometry(bar, f"btn_{key}_bar"), material=green, name=f"btn_{key}_bar"
            )
        elif key in {"play", "home", "voice"}:
            label = {
                "play": "PLAY",
                "home": "HOME",
                "voice": "VOICE",
            }[key]
            part.visual(
                _pixel_label_mesh(
                    f"btn_{key}_label",
                    label,
                    pixel=0.00034,
                    thickness=0.00016,
                    z=btn_height + r * 0.35 + 0.00018,
                    line_gap=0.00045,
                ),
                material=print_dark,
                name=f"btn_{key}_label",
            )
        part.inertial = Inertial.from_geometry(
            Cylinder(r, btn_height),
            mass=0.0015,
            origin=Origin(xyz=(0.0, 0.0, btn_height / 2.0)),
        )
        model.articulation(
            f"body_to_btn_{key}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=part,
            origin=Origin(xyz=(x, y, WELL_FLOOR_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.05, lower=-PRESS_TRAVEL, upper=0.0),
        )

    # ---- oval BASS +/- pads: each presses straight down (-Z) ----
    for key, x, y, rx, ry in oval_buttons:
        part = model.part(f"btn_{key}")
        part.visual(
            _oval_button_mesh(f"btn_{key}", rx, ry, btn_height),
            material=btn_gray,
            name=f"btn_{key}_cap",
        )
        symbol_z = btn_height * 1.55 + 0.00015
        symbol = _plus_icon_mesh if key == "bass_up" else _minus_icon_mesh
        part.visual(
            symbol(f"btn_{key}_symbol", size=0.0032, thickness=0.00016, z=symbol_z),
            material=print_white,
            name=f"btn_{key}_symbol",
        )
        if key == "bass_up":
            bass_label = _pixel_label_geometry("BASS", pixel=0.00028, thickness=0.00016, z=symbol_z)
            bass_label.translate(-0.0008, -0.0024, 0.0)
            part.visual(
                mesh_from_geometry(bass_label, "btn_bass_label"),
                material=print_white,
                name="btn_bass_label",
            )
        part.inertial = Inertial.from_geometry(
            Box((rx * 2, ry * 2, btn_height)),
            mass=0.0010,
            origin=Origin(xyz=(0.0, 0.0, btn_height / 2.0)),
        )
        model.articulation(
            f"body_to_btn_{key}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=part,
            origin=Origin(xyz=(x, y, WELL_FLOOR_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.05, lower=-PRESS_TRAVEL, upper=0.0),
        )

    # ---- large central circular volume control: REVOLUTE rocker (rocks ~Y) ----
    vol = model.part("volume_rocker")
    disc = CylinderGeometry(VOL_R, 0.0040, radial_segments=48)
    disc.translate(0.0, 0.0, 0.0020)
    vol.visual(mesh_from_geometry(disc, "volume_disc"), material=vol_gray, name="volume_disc")
    cap = DomeGeometry(VOL_R, radial_segments=48, height_segments=12)
    cap.scale(1.0, 1.0, 0.18)
    cap.translate(0.0, 0.0, 0.0040)
    vol.visual(mesh_from_geometry(cap, "volume_dome"), material=vol_gray, name="volume_dome")
    vol_label_z = 0.0040 + VOL_R * 0.18 + 0.00018
    plus = _plus_icon_geometry(size=0.0052, thickness=0.00018, z=vol_label_z)
    plus.translate(0.0058, 0.0043, 0.0)
    vol.visual(mesh_from_geometry(plus, "volume_plus"), material=print_dark, name="volume_plus")
    minus = _minus_icon_geometry(size=0.0050, thickness=0.00018, z=vol_label_z)
    minus.translate(-0.0060, -0.0047, 0.0)
    vol.visual(mesh_from_geometry(minus, "volume_minus"), material=print_dark, name="volume_minus")
    tri = _triangle_icon_geometry(size=0.0060, thickness=0.00018, z=vol_label_z)
    tri.translate(-0.0005, 0.0000, 0.0)
    vol.visual(
        mesh_from_geometry(tri, "volume_triangle"), material=print_dark, name="volume_triangle"
    )
    vol.inertial = Inertial.from_geometry(
        Cylinder(VOL_R, 0.0060), mass=0.004, origin=Origin(xyz=(0.0, 0.0, 0.0030))
    )
    model.articulation(
        "body_to_volume_rocker",
        ArticulationType.REVOLUTE,
        parent=body,
        child=vol,
        origin=Origin(xyz=(VOL_X, VOL_Y, WELL_FLOOR_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.0, lower=-0.18, upper=0.18),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")

    # ---- body is a slim wand: long in X, narrow in Y, thin in Z ----
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "body is a long slim wand",
        bext[0] > 0.14 and bext[1] < 0.05 and bext[2] < 0.02 and bext[0] > 3.0 * bext[1],
        details=f"body extents={bext}",
    )

    # ---- compact media cluster: exactly 4 round buttons, no BASS pads ----
    media_keys = ["power", "play", "home", "voice"]
    all_part_names = [p.name for p in object_model.parts]
    btn_parts = [n for n in all_part_names if n.startswith("btn_")]
    ctx.check(
        "exactly 4 media buttons (no BASS pads)",
        sorted(btn_parts) == sorted(f"btn_{k}" for k in media_keys),
        details=f"found button parts={sorted(btn_parts)}",
    )

    # ---- representative round buttons: front face, upper half, press down ----
    for key in media_keys:
        part = object_model.get_part(f"btn_{key}")
        joint = object_model.get_articulation(f"body_to_btn_{key}")
        pos = ctx.part_world_position(part)
        ctx.check(
            f"btn_{key} on front face upper half",
            pos is not None and pos[2] > 0.0 and pos[0] > -0.02,
            details=f"btn_{key} pos={pos}",
        )
        ctx.expect_contact(part, body, name=f"btn_{key} seated on body")
        top0 = ctx.part_world_aabb(part)[1][2]
        with ctx.pose({joint: -PRESS_TRAVEL}):
            top1 = ctx.part_world_aabb(part)[1][2]
        ctx.check(
            f"btn_{key} presses straight down",
            top1 < top0 - 0.0008,
            details=f"top rest={top0}, pressed={top1}",
        )

    # Buttons seat into their recessed wells (cap skirt enters the well lip).
    for key in media_keys:
        part = object_model.get_part(f"btn_{key}")
        ctx.allow_overlap(
            part,
            body,
            elem_a=f"btn_{key}_cap",
            elem_b="body_shell",
            reason="Rubber button cap is intentionally seated into its recessed front-face well.",
        )

    # ---- central volume control articulates (rocks about Y) ----
    vol = object_model.get_part("volume_rocker")
    rocker = object_model.get_articulation("body_to_volume_rocker")
    ctx.allow_overlap(
        vol,
        body,
        elem_a="volume_disc",
        elem_b="body_shell",
        reason="Volume rocker disc is seated into its recessed central well and pivots there.",
    )
    vext = _ext(ctx.part_world_aabb(vol))
    ctx.check(
        "volume control is large and central",
        vext[0] > 0.024 and vext[1] > 0.024,
        details=f"volume extents={vext}",
    )
    zmax0 = ctx.part_world_aabb(vol)[1][2]
    with ctx.pose({rocker: 0.18}):
        zmax_pos = ctx.part_world_aabb(vol)[1][2]
    with ctx.pose({rocker: -0.18}):
        zmax_neg = ctx.part_world_aabb(vol)[1][2]
    ctx.check(
        "volume rocker tilts (z extent changes when rocked)",
        abs(zmax_pos - zmax0) > 0.0004 or abs(zmax_neg - zmax0) > 0.0004,
        details=f"rest={zmax0}, +tilt={zmax_pos}, -tilt={zmax_neg}",
    )

    return ctx.report()


object_model = build_object_model()
