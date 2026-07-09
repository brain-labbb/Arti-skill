from __future__ import annotations

# Silver thin-and-light laptop computer (open).
# Frame: base lies flat in the XY plane, thickness along +Z.
#   +X = width (right), -X = left; +Y = rear (hinge side), -Y = front (user);
#   +Z = up. Base centerline at z=0..base_thickness.
# Articulations:
#   - screen lid: REVOLUTE hinge at the rear of the base, swings the screen up
#     about +X (open ~0..130 deg). The lid geometry is authored flat, extending
#     +Y from the hinge, with the screen on its top (+Z) face; positive q lifts
#     the screen up and forward toward the user.
#   - keyboard keys: a full island-style grid; EACH key is a PRISMATIC press
#     straight down (-Z, ~1 mm), with indexed names key_r{r}_c{c}.
#   - trackpad: PRISMATIC click press straight down (-Z, ~0.5 mm).

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Dimensions (meters)
# ---------------------------------------------------------------------------
BASE_W = 0.310   # X
BASE_D = 0.220   # Y
BASE_T = 0.015   # Z thickness
BASE_FILLET = 0.006

HINGE_Y = BASE_D / 2.0          # rear edge of the base (hinge line)
HINGE_Z = BASE_T                # hinge sits at the top-rear edge

LID_W = 0.302
LID_D = 0.208       # lid length along its local +Y
LID_T = 0.006       # lid slab thickness
LID_FILLET = 0.004

# Keyboard well + key grid
WELL_W = 0.250
WELL_D = 0.105
WELL_DEPTH = 0.004              # recess depth into the top face
KEY_AREA_FRONT_Y = -0.010      # front edge of key grid (in base frame)
KB_ROWS = 6
KB_COLS = 14
KEY_PITCH_X = 0.0172
KEY_PITCH_Y = 0.0162
KEY_W = 0.0150
KEY_D = 0.0140
KEY_H = 0.0045
KEY_TRAVEL = 0.0010

# Trackpad
TRACKPAD_W = 0.110
TRACKPAD_D = 0.072
TRACKPAD_T = 0.0025
TRACKPAD_TRAVEL = 0.0005


def _filleted_slab(w: float, d: float, t: float, fillet: float) -> cq.Workplane:
    """A rounded-corner slab centered at origin in XY, base at z=0."""
    slab = cq.Workplane("XY").box(w, d, t, centered=(True, True, False))
    slab = slab.edges("|Z").fillet(fillet)
    return slab


def _base_solid() -> cq.Workplane:
    """Slim aluminum base: filleted slab with a recessed keyboard well and a
    recessed trackpad pocket on the top face."""
    base = _filleted_slab(BASE_W, BASE_D, BASE_T, BASE_FILLET)

    # Keyboard well: rectangular recess cut into the top face.
    well_cy = KEY_AREA_FRONT_Y + WELL_D / 2.0 + 0.004
    well = (
        cq.Workplane("XY")
        .center(0.0, well_cy)
        .box(WELL_W, WELL_D, WELL_DEPTH * 2.0, centered=(True, True, True))
        .edges("|Z")
        .fillet(0.004)
    )
    well = well.translate((0.0, 0.0, BASE_T))  # center plane at the top surface
    base = base.cut(well)

    # Trackpad pocket: shallow recess in front of the keyboard.
    tp_cy = KEY_AREA_FRONT_Y - 0.006 - TRACKPAD_D / 2.0
    pocket = (
        cq.Workplane("XY")
        .center(0.0, tp_cy)
        .box(TRACKPAD_W + 0.004, TRACKPAD_D + 0.004, 0.004, centered=(True, True, True))
        .edges("|Z")
        .fillet(0.003)
    )
    pocket = pocket.translate((0.0, 0.0, BASE_T))
    base = base.cut(pocket)

    # Two small rubber-foot pads under the rear (kept subtle, structural).
    return base


def _hinge_barrel() -> cq.Workplane:
    """A short cylindrical hinge barrel spanning the rear edge, giving the lid a
    real mount instead of a floating pivot."""
    r = 0.005
    barrel = (
        cq.Workplane("YZ")
        .workplane(offset=-LID_W / 2.0 + 0.012)
        .circle(r)
        .workplane(offset=LID_W - 0.024)
        .circle(r)
        .loft(ruled=True)
    )
    barrel = barrel.translate((0.0, HINGE_Y - 0.002, HINGE_Z - 0.001))
    return barrel


def _lid_solid() -> cq.Workplane:
    """Thin display lid slab, authored flat extending +Y from the hinge."""
    lid = (
        cq.Workplane("XY")
        .center(0.0, LID_D / 2.0)
        .box(LID_W, LID_D, LID_T, centered=(True, True, False))
        .edges("|Z")
        .fillet(LID_FILLET)
    )
    return lid


def _trackpad_solid() -> cq.Workplane:
    pad = (
        cq.Workplane("XY")
        .box(TRACKPAD_W, TRACKPAD_D, TRACKPAD_T, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
    )
    return pad


def _key_solid() -> cq.Workplane:
    """A small rounded-rect island keycap, base at z=0."""
    key = (
        cq.Workplane("XY")
        .box(KEY_W, KEY_D, KEY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0012)
    )
    return key


def _key_centers() -> list[tuple[int, int, float, float]]:
    """Indexed key grid centers in the base frame.
    Returns (row, col, x, y). Row 0 is the front row, increasing toward rear."""
    grid_w = (KB_COLS - 1) * KEY_PITCH_X
    grid_d = (KB_ROWS - 1) * KEY_PITCH_Y
    x0 = -grid_w / 2.0
    y0 = KEY_AREA_FRONT_Y + 0.010
    centers: list[tuple[int, int, float, float]] = []
    for r in range(KB_ROWS):
        for c in range(KB_COLS):
            x = x0 + c * KEY_PITCH_X
            y = y0 + r * KEY_PITCH_Y
            centers.append((r, c, x, y))
    return centers


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laptop_computer")

    aluminum = model.material("silver_aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    key_dark = model.material("key_dark", rgba=(0.16, 0.17, 0.19, 1.0))
    screen_dark = model.material("screen_dark", rgba=(0.05, 0.06, 0.09, 1.0))
    screen_blue = model.material("screen_wallpaper_blue", rgba=(0.22, 0.46, 0.82, 1.0))
    glass = model.material("trackpad_glass", rgba=(0.70, 0.74, 0.78, 0.85))
    webcam = model.material("webcam_dark", rgba=(0.02, 0.02, 0.03, 1.0))
    logo = model.material("logo_leaf", rgba=(0.62, 0.66, 0.70, 1.0))

    # ---- base (root) ----
    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_solid(), "base_shell"),
        material=aluminum,
        name="base_shell",
    )
    # Hinge barrel mounted at the rear top edge.
    base.visual(
        mesh_from_cadquery(_hinge_barrel(), "hinge_barrel"),
        material=key_dark,
        name="hinge_barrel",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_W, BASE_D, BASE_T)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
    )

    # ---- screen lid: revolute hinge at the rear, screen on the top (+Z) face ----
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_solid(), "lid_shell"),
        material=aluminum,
        name="lid_shell",
    )
    # Thin-bezel dark screen on the inner (+Z) face of the lid.
    screen_inset = 0.008
    lid.visual(
        Box((LID_W - 2 * screen_inset, LID_D - 2 * screen_inset, 0.0012)),
        origin=Origin(xyz=(0.0, LID_D / 2.0, LID_T + 0.0006)),
        material=screen_dark,
        name="screen_panel",
    )
    # Blue abstract wallpaper, slightly inset on top of the screen.
    lid.visual(
        Box((LID_W - 2 * screen_inset - 0.004, LID_D - 2 * screen_inset - 0.004, 0.0006)),
        origin=Origin(xyz=(0.0, LID_D / 2.0, LID_T + 0.0014)),
        material=screen_blue,
        name="screen_wallpaper",
    )
    # Small webcam in the top bezel (rear end of the lid, on the inner face).
    # Sits proud of the bezel but is embedded into the lid shell so it reads as
    # one body with the lid (no floating island).
    lid.visual(
        Box((0.005, 0.004, 0.0030)),
        origin=Origin(xyz=(0.0, LID_D - 0.004, LID_T - 0.0010)),
        material=webcam,
        name="webcam",
    )
    # Subtle leaf/logo on the lid back (outer, -Z face).
    lid.visual(
        Box((0.024, 0.024, 0.0008)),
        origin=Origin(xyz=(0.0, LID_D / 2.0, -0.0004)),
        material=logo,
        name="back_logo",
    )
    lid.inertial = Inertial.from_geometry(
        Box((LID_W, LID_D, LID_T)),
        mass=0.5,
        origin=Origin(xyz=(0.0, LID_D / 2.0, LID_T / 2.0)),
    )
    lid_hinge = model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y - 0.002, HINGE_Z - 0.001)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(130.0),
        ),
    )

    # ---- trackpad: prismatic click press straight down ----
    trackpad = model.part("trackpad")
    trackpad.visual(
        mesh_from_cadquery(_trackpad_solid(), "trackpad_glass"),
        material=glass,
        name="trackpad_glass",
    )
    trackpad.inertial = Inertial.from_geometry(
        Box((TRACKPAD_W, TRACKPAD_D, TRACKPAD_T)),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, TRACKPAD_T / 2.0)),
    )
    tp_cy = KEY_AREA_FRONT_Y - 0.006 - TRACKPAD_D / 2.0
    # Seat the trackpad on its pocket floor (base contact), glass top flush with
    # the deck surface. Pocket floor is at BASE_T - 0.004.
    tp_z = BASE_T - 0.004
    model.articulation(
        "base_to_trackpad",
        ArticulationType.PRISMATIC,
        parent=base,
        child=trackpad,
        origin=Origin(xyz=(0.0, tp_cy, tp_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.05,
            lower=0.0,
            upper=TRACKPAD_TRAVEL,
        ),
    )

    # ---- keyboard keys: full island grid, each a prismatic press down ----
    key_mesh = mesh_from_cadquery(_key_solid(), "keycap")
    # Keys seat in the well: their base sits at the recessed well floor, caps
    # rise just above the base top surface.
    well_floor_z = BASE_T - WELL_DEPTH
    # Seat each keycap so its base embeds slightly into the well floor: this
    # gives real contact with the base (no floating part) and a small, allowed
    # seating overlap, while the cap still rises just above the deck surface.
    key_seat_z = well_floor_z - 0.0006
    for r, c, x, y in _key_centers():
        key = model.part(f"key_r{r}_c{c}")
        key.visual(key_mesh, material=key_dark, name="keycap")
        key.inertial = Inertial.from_geometry(
            Box((KEY_W, KEY_D, KEY_H)),
            mass=0.0015,
            origin=Origin(xyz=(0.0, 0.0, KEY_H / 2.0)),
        )
        model.articulation(
            f"base_to_key_r{r}_c{c}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=key,
            origin=Origin(xyz=(x, y, key_seat_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.05,
                lower=0.0,
                upper=KEY_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    trackpad = object_model.get_part("trackpad")
    lid_hinge = object_model.get_articulation("base_to_lid")
    tp_joint = object_model.get_articulation("base_to_trackpad")

    # --- Lid opens about the rear hinge: screen swings up and forward. ---
    lid_rest = ctx.part_world_aabb(lid)
    rest_top_z = lid_rest[1][2]
    open_q = math.radians(110.0)
    with ctx.pose({lid_hinge: open_q}):
        lid_open = ctx.part_world_aabb(lid)
        open_top_z = lid_open[1][2]
        open_rear_y = lid_open[1][1]
    ctx.check(
        "lid swings up about the rear hinge",
        open_top_z > rest_top_z + 0.08,
        details=f"rest_top_z={rest_top_z:.4f}, open_top_z={open_top_z:.4f}",
    )
    # When opened, the lid stands tall (Z extent dominates its flat-rest Y span).
    with ctx.pose({lid_hinge: open_q}):
        ext = ctx.part_world_aabb(lid)
        z_span = ext[1][2] - ext[0][2]
        y_span = ext[1][1] - ext[0][1]
    ctx.check(
        "opened lid stands roughly upright",
        z_span > 0.12,
        details=f"z_span={z_span:.4f}, y_span={y_span:.4f}",
    )

    # Hinge is at the rear of the base (positive Y side).
    base_aabb = ctx.part_world_aabb(base)
    hinge_y = lid_hinge.origin.xyz[1]
    ctx.check(
        "hinge mounted at the rear of the base",
        hinge_y > 0.5 * base_aabb[1][1],
        details=f"hinge_y={hinge_y:.4f}, base_rear_y={base_aabb[1][1]:.4f}",
    )

    # Screen panel sits on the inner (+Z) face of the lid (faces user when open).
    screen = lid.get_visual("screen_panel")
    lid_shell = lid.get_visual("lid_shell")
    ctx.expect_contact(
        lid, lid, elem_a="screen_panel", elem_b="lid_shell",
        name="screen mounted on the lid inner face",
    )
    ctx.check(
        "screen present and named",
        screen is not None and lid_shell is not None,
    )

    # --- Trackpad clicks straight down. ---
    tp_rest = ctx.part_world_position(trackpad)
    with ctx.pose({tp_joint: TRACKPAD_TRAVEL}):
        tp_down = ctx.part_world_position(trackpad)
    ctx.check(
        "trackpad clicks downward",
        tp_down[2] < tp_rest[2] - 0.0003,
        details=f"rest_z={tp_rest[2]:.5f}, down_z={tp_down[2]:.5f}",
    )
    # Trackpad sits in front of the keyboard (negative Y of base center).
    ctx.check(
        "trackpad in front of keyboard",
        tp_rest[1] < 0.0,
        details=f"trackpad_y={tp_rest[1]:.4f}",
    )

    # --- A representative sample of keys: each presses straight down and is
    #     seated in the base well below the (closed) lid footprint. ---
    sample = [(0, 0), (0, KB_COLS - 1), (2, 6), (4, 3), (KB_ROWS - 1, KB_COLS - 1)]
    for r, c in sample:
        key = object_model.get_part(f"key_r{r}_c{c}")
        joint = object_model.get_articulation(f"base_to_key_r{r}_c{c}")
        rest = ctx.part_world_position(key)
        with ctx.pose({joint: KEY_TRAVEL}):
            pressed = ctx.part_world_position(key)
        ctx.check(
            f"key_r{r}_c{c} presses straight down",
            pressed[2] < rest[2] - 0.0005
            and abs(pressed[0] - rest[0]) < 1e-4
            and abs(pressed[1] - rest[1]) < 1e-4,
            details=f"rest={rest}, pressed={pressed}",
        )
        # Key cap top stays at/below the base top surface +small (seated in well).
        kbb = ctx.part_element_world_aabb(key, elem="keycap")
        ctx.check(
            f"key_r{r}_c{c} seated in the base well",
            kbb[0][2] < BASE_T + 1e-4,
            details=f"key_base_z={kbb[0][2]:.5f}, base_top={BASE_T:.5f}",
        )

    # Keys sit within the base footprint (below the closed-lid region / on the deck).
    k_mid = object_model.get_part("key_r2_c6")
    ctx.expect_within(
        k_mid, base, axes="xy", margin=0.002,
        name="middle key within base footprint",
    )

    # Keys are recessed in the well: allow the keycap-in-well seating overlap.
    for r, c in sample:
        key = object_model.get_part(f"key_r{r}_c{c}")
        ctx.allow_overlap(
            key, base,
            elem_a="keycap", elem_b="base_shell",
            reason="Island keycap is seated inside the recessed keyboard well.",
        )

    # Trackpad seated in its pocket recess.
    ctx.allow_overlap(
        trackpad, base,
        elem_a="trackpad_glass", elem_b="base_shell",
        reason="Glass trackpad is seated inside its recessed pocket in the base.",
    )

    return ctx.report()


object_model = build_object_model()
