from __future__ import annotations

# Sliding-door variant of the two-locker bank.
# Frame:
#   - X: bank width (the two lockers are arranged along +X), bank centered on x=0.
#   - Y: depth; the doors face the front at +Y.
#   - Z: height; floor at z=0, top of the bank at z~0.90.
# Each locker unit is ~0.30 (W) x 0.45 (D) x 0.90 (H).
# Movers (built by _build_locker, called twice via a for loop):
#   - door_{idx}: PRISMATIC, slides sideways along the bank width axis (X) on
#     top/bottom track rails. Bay 0 slides left (-X), bay 1 slides right (+X).
#   - lockbtn_{idx}_{n}: PRISMATIC round push-buttons on each door's keypad,
#     children of the door, press straight in ~1.5 mm.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    VentGrilleGeometry,
    VentGrilleSleeve,
    VentGrilleSlats,
    mesh_from_geometry,
)

# ---- Bank / locker dimensions (identical to parent) ----
LOCKER_W = 0.30
LOCKER_D = 0.45
LOCKER_H = 0.90
DOOR_TH = 0.018          # door panel thickness
WALL = 0.012             # carcass sheet-metal thickness (visual)
CARCASS_FRONT_Y = LOCKER_D / 2.0   # front plane of carcass

# ---- Track rail dimensions (NEW for sliding variant) ----
TRACK_H = 0.010          # track rail height (visible channel lip)
TRACK_D = 0.004          # track rail depth (protrudes from carcass front)
TRACK_GAP = 0.0          # door back face contacts track front face (seated in track)

# ---- Door dimensions ----
HINGE_GAP = 0.004        # reveal gap around door edges
DOOR_W = LOCKER_W - 2 * HINGE_GAP
DOOR_H = LOCKER_H - 2 * HINGE_GAP
# Door panel center Y: track depth + clearance + half door thickness in front of
# the carcass front face.
DOOR_FRONT_Y = CARCASS_FRONT_Y + TRACK_D + TRACK_GAP + DOOR_TH / 2.0

# Slide distance: enough to fully clear the bay opening.
SLIDE_DIST = LOCKER_W

# ---- Keypad button layout (identical to parent) ----
BTN_R = 0.011
BTN_PRESS = 0.0015       # 1.5 mm press-in travel
BTN_LEN = 0.010          # button cylinder length
BTN_COLS = 5
BTN_ROWS = 2
BTN_N = BTN_COLS * BTN_ROWS  # 10 buttons per door

NUM_BAYS = 2


def _locker_center_x(idx: int) -> float:
    """Two lockers arranged along X, the whole bank centered on x=0."""
    return (idx - 0.5) * LOCKER_W


def _slide_direction(idx: int) -> int:
    """Bay 0 slides left (-X), bay 1 slides right (+X)."""
    return -1 if idx == 0 else 1


def _build_track_rails(carcass, cx: float, idx: int, slide_dir: int,
                       grey) -> None:
    """Add top and bottom sliding-track rails for one bay onto the carcass."""
    track_len = SLIDE_DIST + DOOR_W
    # Track center X is shifted in the slide direction so it supports the door
    # through its full travel range.
    track_cx = cx + slide_dir * (SLIDE_DIST / 2.0)
    track_y = CARCASS_FRONT_Y + TRACK_D / 2.0

    # Top track rail: sits at the top inner edge of the bay opening.
    carcass.visual(
        Box((track_len, TRACK_D, TRACK_H)),
        origin=Origin(xyz=(track_cx, track_y, LOCKER_H - WALL - TRACK_H / 2.0)),
        material=grey,
        name=f"track_top_{idx}",
    )
    # Bottom track rail: sits at the bottom inner edge of the bay opening.
    carcass.visual(
        Box((track_len, TRACK_D, TRACK_H)),
        origin=Origin(xyz=(track_cx, track_y, WALL + TRACK_H / 2.0)),
        material=grey,
        name=f"track_bottom_{idx}",
    )


def _build_locker(model: ArticulatedObject, x_offset: float, idx: int, *,
                  carcass, white, grey, dark, btn_dark) -> None:
    """Build one sliding-door locker bay at world x = x_offset.

    Adds the door (prismatic slider), its keypad push-buttons (prismatic),
    and the track rails on the carcass.
    """
    cx = x_offset
    slide_dir = _slide_direction(idx)

    # ---- carcass body geometry for this bay (sides / back / top / bottom) ----
    # back panel
    carcass.visual(
        Box((LOCKER_W, WALL, LOCKER_H)),
        origin=Origin(xyz=(cx, -LOCKER_D / 2.0 + WALL / 2.0, LOCKER_H / 2.0)),
        material=grey,
        name=f"back_{idx}",
    )
    # left side panel
    carcass.visual(
        Box((WALL, LOCKER_D, LOCKER_H)),
        origin=Origin(xyz=(cx - LOCKER_W / 2.0 + WALL / 2.0, 0.0, LOCKER_H / 2.0)),
        material=white,
        name=f"side_l_{idx}",
    )
    # right side panel
    carcass.visual(
        Box((WALL, LOCKER_D, LOCKER_H)),
        origin=Origin(xyz=(cx + LOCKER_W / 2.0 - WALL / 2.0, 0.0, LOCKER_H / 2.0)),
        material=white,
        name=f"side_r_{idx}",
    )
    # top panel
    carcass.visual(
        Box((LOCKER_W, LOCKER_D, WALL)),
        origin=Origin(xyz=(cx, 0.0, LOCKER_H - WALL / 2.0)),
        material=white,
        name=f"top_{idx}",
    )
    # bottom panel (raised slightly off the floor)
    carcass.visual(
        Box((LOCKER_W, LOCKER_D, WALL)),
        origin=Origin(xyz=(cx, 0.0, WALL / 2.0)),
        material=white,
        name=f"bottom_{idx}",
    )
    # a shelf inside, mid height
    carcass.visual(
        Box((LOCKER_W - 2 * WALL, LOCKER_D - WALL, WALL * 0.7)),
        origin=Origin(xyz=(cx, WALL / 2.0, LOCKER_H * 0.55)),
        material=grey,
        name=f"shelf_{idx}",
    )

    # ---- Track rails (mounted on carcass front face for this bay) ----
    _build_track_rails(carcass, cx, idx, slide_dir, grey)

    # ---- Sliding door (PRISMATIC, slides along X) ----
    # The door part frame is centered on the door panel, at the bay center in X.
    door = model.part(f"door_{idx}")

    # Main door panel: centered on its own frame origin.
    door.visual(
        Box((DOOR_W, DOOR_TH, DOOR_H)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=white,
        name=f"door_panel_{idx}",
    )

    # Recessed louver air vents near the TOP of the door (front face, +Y).
    vent_w = DOOR_W * 0.55
    vent_h = 0.075
    vent_z = DOOR_H / 2.0 - 0.085  # near the top of the door (local z)
    vent = VentGrilleGeometry(
        (vent_w, vent_h),
        frame=0.008,
        face_thickness=0.0035,
        duct_depth=0.012,
        duct_wall=0.0025,
        slat_pitch=0.012,
        slat_width=0.008,
        slat_angle_deg=35.0,
        corner_radius=0.003,
        slats=VentGrilleSlats(profile="flat", direction="down"),
        sleeve=VentGrilleSleeve(style="short"),
        center=True,
    )
    vent_mesh = mesh_from_geometry(vent, f"vent_{idx}")
    door.visual(
        vent_mesh,
        origin=Origin(xyz=(0.0, DOOR_TH / 2.0 - 0.006, vent_z),
                      rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name=f"door_vent_{idx}",
    )

    # Small barcode / number plate below the vents.
    plate = PerforatedPanelGeometry(
        (0.045, 0.030),
        0.0025,
        hole_diameter=0.0016,
        pitch=(0.0055, 0.0055),
        frame=0.005,
        corner_radius=0.002,
    )
    plate_mesh = mesh_from_geometry(plate, f"plate_{idx}")
    door.visual(
        plate_mesh,
        origin=Origin(xyz=(0.0, DOOR_TH / 2.0, vent_z - 0.075),
                      rpy=(0.0, 0.0, 0.0)),
        material=dark,
        name=f"door_plate_{idx}",
    )

    # Lock keypad mounting plate near the BOTTOM of the door (front face).
    pad_w = BTN_COLS * (2 * BTN_R + 0.004) + 0.010
    pad_h = BTN_ROWS * (2 * BTN_R + 0.004) + 0.010
    pad_cx = 0.0  # centered on door (center-relative frame)
    pad_cz = -DOOR_H / 2.0 + 0.110  # near the bottom of the door
    door.visual(
        Box((pad_w, 0.006, pad_h)),
        origin=Origin(xyz=(pad_cx, DOOR_TH / 2.0 + 0.003, pad_cz)),
        material=grey,
        name=f"lockpad_{idx}",
    )

    # Handle bar on the door (vertical, near the grab edge).
    # The grab edge is opposite the slide direction: if the door slides left,
    # you grab the right edge to pull it.
    handle_x = -slide_dir * (DOOR_W / 2.0 - 0.030)
    door.visual(
        Box((0.016, 0.022, 0.16)),
        origin=Origin(xyz=(handle_x, DOOR_TH / 2.0 + 0.011, 0.06)),
        material=grey,
        name=f"door_handle_{idx}",
    )

    door.inertial = Inertial.from_geometry(
        Box((DOOR_W, DOOR_TH, DOOR_H)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # PRISMATIC joint: door slides along X on the track rails.
    # Joint origin is at the bay center, at the door panel center Y, mid-height.
    model.articulation(
        f"slide_{idx}",
        ArticulationType.PRISMATIC,
        parent=carcass,
        child=door,
        origin=Origin(xyz=(cx, DOOR_FRONT_Y, LOCKER_H / 2.0)),
        axis=(float(slide_dir), 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=0.5, lower=0.0, upper=SLIDE_DIST
        ),
    )

    # ---- Keypad push-buttons: children of the door, press straight in ----
    col_pitch = 2 * BTN_R + 0.004
    row_pitch = 2 * BTN_R + 0.004
    x0 = pad_cx - (BTN_COLS - 1) * col_pitch / 2.0
    z0 = pad_cz - (BTN_ROWS - 1) * row_pitch / 2.0
    btn_front_y = DOOR_TH / 2.0 + 0.006  # button sits on the lock plate face
    n = 0
    for r in range(BTN_ROWS):
        for c in range(BTN_COLS):
            bx = x0 + c * col_pitch
            bz = z0 + r * row_pitch
            btn = model.part(f"lockbtn_{idx}_{n}")
            cyl = CylinderGeometry(BTN_R, BTN_LEN, radial_segments=20).rotate_x(
                math.pi / 2.0
            )
            btn.visual(
                mesh_from_geometry(cyl, f"btn_{idx}_{n}"),
                origin=Origin(xyz=(0.0, 0.0, 0.0)),
                material=btn_dark,
                name=f"btn_cap_{idx}_{n}",
            )
            btn.inertial = Inertial.from_geometry(
                Box((2 * BTN_R, BTN_LEN, 2 * BTN_R)), mass=0.004
            )
            model.articulation(
                f"btnjoint_{idx}_{n}",
                ArticulationType.PRISMATIC,
                parent=door,
                child=btn,
                origin=Origin(xyz=(bx, btn_front_y + BTN_LEN / 2.0, bz)),
                axis=(0.0, -1.0, 0.0),
                motion_limits=MotionLimits(
                    effort=5.0, velocity=0.05, lower=0.0, upper=BTN_PRESS
                ),
            )
            n += 1


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="locker_bank_sliding")

    white = model.material("locker_white", rgba=(0.90, 0.90, 0.88, 1.0))
    grey = model.material("locker_grey", rgba=(0.62, 0.64, 0.66, 1.0))
    dark = model.material("vent_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    btn_dark = model.material("button_dark", rgba=(0.12, 0.12, 0.14, 1.0))

    # Shared carcass (root) carries both locker bays + the floor plinth.
    carcass = model.part("carcass")

    # Base plinth running under the whole bank.
    carcass.visual(
        Box((2 * LOCKER_W + 0.004, LOCKER_D, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=grey,
        name="plinth",
    )

    # Central divider between the two bays (shared sheet).
    carcass.visual(
        Box((WALL, LOCKER_D, LOCKER_H)),
        origin=Origin(xyz=(0.0, 0.0, LOCKER_H / 2.0)),
        material=white,
        name="divider",
    )

    carcass.inertial = Inertial.from_geometry(
        Box((2 * LOCKER_W, LOCKER_D, LOCKER_H)),
        mass=30.0,
        origin=Origin(xyz=(0.0, 0.0, LOCKER_H / 2.0)),
    )

    for idx in range(NUM_BAYS):
        _build_locker(
            model,
            _locker_center_x(idx),
            idx,
            carcass=carcass,
            white=white,
            grey=grey,
            dark=dark,
            btn_dark=btn_dark,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    carcass = object_model.get_part("carcass")

    # ---- The two doors are side by side (offset along X) ----
    door0 = object_model.get_part("door_0")
    door1 = object_model.get_part("door_1")
    p0 = ctx.part_world_position(door0)
    p1 = ctx.part_world_position(door1)
    ctx.check(
        "two doors offset side by side along X",
        p0 is not None and p1 is not None and abs((p1[0] - p0[0]) - LOCKER_W) < 0.02,
        details=f"door0={p0}, door1={p1}",
    )

    # ---- Each door slides along X on its track and clears the bay opening ----
    for idx in range(NUM_BAYS):
        door = object_model.get_part(f"door_{idx}")
        slide = object_model.get_articulation(f"slide_{idx}")
        slide_dir = _slide_direction(idx)
        bay_cx = _locker_center_x(idx)

        # Verify the joint is prismatic (not revolute like the parent).
        ctx.check(
            f"slide_{idx} is PRISMATIC",
            slide.articulation_type == ArticulationType.PRISMATIC,
            details=f"type={slide.articulation_type}",
        )

        # Closed pose: door covers the bay opening (X overlap with bay center).
        closed_pos = ctx.part_world_position(door)
        ctx.check(
            f"door_{idx} covers bay at rest",
            closed_pos is not None and abs(closed_pos[0] - bay_cx) < 0.02,
            details=f"closed_pos={closed_pos}, bay_cx={bay_cx}",
        )

        # Fully open pose: door has slid SLIDE_DIST along the slide axis.
        with ctx.pose({slide: SLIDE_DIST}):
            open_pos = ctx.part_world_position(door)

        # The door should have moved substantially along X in the slide direction.
        ctx.check(
            f"door_{idx} slides clear of bay opening",
            closed_pos is not None and open_pos is not None
            and slide_dir * (open_pos[0] - closed_pos[0]) > SLIDE_DIST * 0.9,
            details=f"closed={closed_pos}, open={open_pos}, dir={slide_dir}",
        )

        # At full slide, the door X extent should no longer overlap the bay X extent.
        # Bay X extent: [bay_cx - LOCKER_W/2, bay_cx + LOCKER_W/2]
        if open_pos is not None:
            door_min_x = open_pos[0] - DOOR_W / 2.0
            door_max_x = open_pos[0] + DOOR_W / 2.0
            bay_min_x = bay_cx - LOCKER_W / 2.0
            bay_max_x = bay_cx + LOCKER_W / 2.0
            if slide_dir < 0:
                # Door slid left: door max X should be left of bay min X.
                cleared = door_max_x < bay_min_x + 0.002
            else:
                # Door slid right: door min X should be right of bay max X.
                cleared = door_min_x > bay_max_x - 0.002
            ctx.check(
                f"door_{idx} fully clears bay opening at max slide",
                cleared,
                details=f"door_x=[{door_min_x:.4f},{door_max_x:.4f}], "
                        f"bay_x=[{bay_min_x:.4f},{bay_max_x:.4f}]",
            )

    # ---- Track rails exist on the carcass for each bay ----
    for idx in range(NUM_BAYS):
        for rail in ("top", "bottom"):
            track_name = f"track_{rail}_{idx}"
            track_vis = carcass.get_visual(track_name)
            ctx.check(
                f"carcass has {track_name} rail",
                track_vis is not None,
                details=f"visual={track_vis}",
            )

    # ---- A sampled lock button on each door presses straight in ----
    for idx in range(NUM_BAYS):
        door = object_model.get_part(f"door_{idx}")
        n = 3  # representative button
        btn = object_model.get_part(f"lockbtn_{idx}_{n}")
        btnjoint = object_model.get_articulation(f"btnjoint_{idx}_{n}")
        # Button base intentionally embeds into the lock plate / door front.
        ctx.allow_overlap(
            btn,
            door,
            reason="Push-button base is seated into the lock keypad plate on the door.",
        )
        ctx.expect_contact(
            btn, door, name=f"lockbtn_{idx}_{n} mounted on door"
        )
        rest = ctx.part_world_position(btn)
        with ctx.pose({btnjoint: BTN_PRESS}):
            pressed = ctx.part_world_position(btn)
        # Pressing moves the button into the door (toward -Y in world at rest).
        ctx.check(
            f"lockbtn_{idx}_{n} presses straight in",
            rest is not None and pressed is not None
            and (rest[1] - pressed[1]) > BTN_PRESS * 0.7,
            details=f"rest={rest}, pressed={pressed}",
        )

    # ---- Vents are present on the doors (hero feature) ----
    for idx in range(NUM_BAYS):
        door = object_model.get_part(f"door_{idx}")
        vent = door.get_visual(f"door_vent_{idx}")
        ctx.check(
            f"door_{idx} has a vent visual",
            vent is not None,
            details=f"vent={vent}",
        )

    return ctx.report()


object_model = build_object_model()
