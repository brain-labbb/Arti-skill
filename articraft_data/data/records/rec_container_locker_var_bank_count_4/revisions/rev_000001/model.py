from __future__ import annotations

# A bank of FOUR metal storage lockers standing side by side.
# Frame:
#   - X: bank width (the four lockers are arranged along +X), bank centered on x=0.
#   - Y: depth; the doors face the front at +Y.
#   - Z: height; floor at z=0, top of the bank at z~0.90.
# Each locker unit is ~0.30 (W) x 0.45 (D) x 0.90 (H).
# Movers (built by _build_bay, called N_BAYS times via a single loop):
#   - door_{i}: REVOLUTE, hinged on one vertical side, swings outward 0..~100 deg.
#   - lockbtn_{i}_{n}: PRISMATIC round push-buttons on each door's keypad,
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

# ---- Bank / locker dimensions ----
N_BAYS = 4
LOCKER_W = 0.30
LOCKER_D = 0.45
LOCKER_H = 0.90
DOOR_TH = 0.018          # door panel thickness
WALL = 0.012             # carcass sheet-metal thickness (visual)
CARCASS_FRONT_Y = LOCKER_D / 2.0   # front plane of carcass (door sits just in front)

# Door hinge geometry
HINGE_GAP = 0.004        # small reveal between adjacent doors / frame
DOOR_W = LOCKER_W - 2 * HINGE_GAP
DOOR_H = LOCKER_H - 2 * HINGE_GAP
DOOR_FRONT_Y = CARCASS_FRONT_Y + DOOR_TH / 2.0  # door panel centerline in Y

# Keypad button layout (a small keypad of round buttons near the bottom)
BTN_R = 0.011
BTN_PRESS = 0.0015       # 1.5 mm press-in travel
BTN_LEN = 0.010          # button cylinder length (sticks out of the lock plate)
BTN_COLS = 5
BTN_ROWS = 2
BTN_N = BTN_COLS * BTN_ROWS  # 10 buttons per door


def _bay_center_x(i: int) -> float:
    """Return the world X center of bay i, with the bank centered on x=0."""
    return (i - (N_BAYS - 1) / 2.0) * LOCKER_W


def _build_bay(model: ArticulatedObject, bay_i: int, *,
               carcass: object, white: object, grey: object,
               dark: object, btn_dark: object) -> None:
    """Build one locker bay on the shared carcass.

    Adds carcass plates for the bay, a revolute door, vent grille, keypad plate,
    handle, and prismatic push-buttons. Called once per bay in a single loop.
    """
    cx = _bay_center_x(bay_i)

    # ---- carcass body geometry for this bay (sides / back / top / bottom) ----
    # back panel
    carcass.visual(
        Box((LOCKER_W, WALL, LOCKER_H)),
        origin=Origin(xyz=(cx, -LOCKER_D / 2.0 + WALL / 2.0, LOCKER_H / 2.0)),
        material=grey,
        name=f"back_{bay_i}",
    )
    # left side panel
    carcass.visual(
        Box((WALL, LOCKER_D, LOCKER_H)),
        origin=Origin(xyz=(cx - LOCKER_W / 2.0 + WALL / 2.0, 0.0, LOCKER_H / 2.0)),
        material=white,
        name=f"side_l_{bay_i}",
    )
    # right side panel
    carcass.visual(
        Box((WALL, LOCKER_D, LOCKER_H)),
        origin=Origin(xyz=(cx + LOCKER_W / 2.0 - WALL / 2.0, 0.0, LOCKER_H / 2.0)),
        material=white,
        name=f"side_r_{bay_i}",
    )
    # top panel
    carcass.visual(
        Box((LOCKER_W, LOCKER_D, WALL)),
        origin=Origin(xyz=(cx, 0.0, LOCKER_H - WALL / 2.0)),
        material=white,
        name=f"top_{bay_i}",
    )
    # bottom panel (raised slightly off the floor)
    carcass.visual(
        Box((LOCKER_W, LOCKER_D, WALL)),
        origin=Origin(xyz=(cx, 0.0, WALL / 2.0)),
        material=white,
        name=f"bottom_{bay_i}",
    )
    # a shelf inside, mid height
    carcass.visual(
        Box((LOCKER_W - 2 * WALL, LOCKER_D - WALL, WALL * 0.7)),
        origin=Origin(xyz=(cx, WALL / 2.0, LOCKER_H * 0.55)),
        material=grey,
        name=f"shelf_{bay_i}",
    )

    # ---- door (revolute, hinged on the left vertical edge of this bay) ----
    door = model.part(f"door_{bay_i}")

    # main door panel
    door.visual(
        Box((DOOR_W, DOOR_TH, DOOR_H)),
        origin=Origin(xyz=(DOOR_W / 2.0, 0.0, 0.0)),
        material=white,
        name=f"door_panel_{bay_i}",
    )

    # recessed louver air vents near the TOP of the door
    vent_w = DOOR_W * 0.55
    vent_h = 0.075
    vent_z = DOOR_H / 2.0 - 0.085
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
    vent_mesh = mesh_from_geometry(vent, f"vent_{bay_i}")
    door.visual(
        vent_mesh,
        origin=Origin(xyz=(DOOR_W / 2.0, DOOR_TH / 2.0 - 0.006, vent_z),
                      rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name=f"door_vent_{bay_i}",
    )

    # small barcode / number plate below the vents
    plate = PerforatedPanelGeometry(
        (0.045, 0.030),
        0.0025,
        hole_diameter=0.0016,
        pitch=(0.0055, 0.0055),
        frame=0.005,
        corner_radius=0.002,
    )
    plate_mesh = mesh_from_geometry(plate, f"plate_{bay_i}")
    door.visual(
        plate_mesh,
        origin=Origin(xyz=(DOOR_W / 2.0, DOOR_TH / 2.0, vent_z - 0.075),
                      rpy=(0.0, 0.0, 0.0)),
        material=dark,
        name=f"door_plate_{bay_i}",
    )

    # lock keypad mounting plate near the BOTTOM of the door
    pad_w = BTN_COLS * (2 * BTN_R + 0.004) + 0.010
    pad_h = BTN_ROWS * (2 * BTN_R + 0.004) + 0.010
    pad_cx = DOOR_W / 2.0
    pad_cz = -DOOR_H / 2.0 + 0.110
    door.visual(
        Box((pad_w, 0.006, pad_h)),
        origin=Origin(xyz=(pad_cx, DOOR_TH / 2.0 + 0.003, pad_cz)),
        material=grey,
        name=f"lockpad_{bay_i}",
    )

    # handle bar on the door (vertical, near the free edge)
    door.visual(
        Box((0.016, 0.022, 0.16)),
        origin=Origin(xyz=(DOOR_W - 0.030, DOOR_TH / 2.0 + 0.011, 0.06)),
        material=grey,
        name=f"door_handle_{bay_i}",
    )

    door.inertial = Inertial.from_geometry(
        Box((DOOR_W, DOOR_TH, DOOR_H)),
        mass=4.0,
        origin=Origin(xyz=(DOOR_W / 2.0, 0.0, 0.0)),
    )

    # Hinge line: left vertical edge of this bay, at the door front plane.
    hinge_x = cx - DOOR_W / 2.0
    hinge_y = DOOR_FRONT_Y
    model.articulation(
        f"hinge_{bay_i}",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, LOCKER_H / 2.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=2.0, lower=0.0, upper=math.radians(100.0)
        ),
    )

    # ---- keypad push-buttons: children of the door, press straight in ----
    col_pitch = 2 * BTN_R + 0.004
    row_pitch = 2 * BTN_R + 0.004
    x0 = pad_cx - (BTN_COLS - 1) * col_pitch / 2.0
    z0 = pad_cz - (BTN_ROWS - 1) * row_pitch / 2.0
    btn_front_y = DOOR_TH / 2.0 + 0.006
    n = 0
    for r in range(BTN_ROWS):
        for c in range(BTN_COLS):
            bx = x0 + c * col_pitch
            bz = z0 + r * row_pitch
            btn = model.part(f"lockbtn_{bay_i}_{n}")
            cyl = CylinderGeometry(BTN_R, BTN_LEN, radial_segments=20).rotate_x(
                math.pi / 2.0
            )
            btn.visual(
                mesh_from_geometry(cyl, f"btn_{bay_i}_{n}"),
                origin=Origin(xyz=(0.0, 0.0, 0.0)),
                material=btn_dark,
                name=f"btn_cap_{bay_i}_{n}",
            )
            btn.inertial = Inertial.from_geometry(
                Box((2 * BTN_R, BTN_LEN, 2 * BTN_R)), mass=0.004
            )
            model.articulation(
                f"btnjoint_{bay_i}_{n}",
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
    model = ArticulatedObject(name="locker_bank")

    white = model.material("locker_white", rgba=(0.90, 0.90, 0.88, 1.0))
    grey = model.material("locker_grey", rgba=(0.62, 0.64, 0.66, 1.0))
    dark = model.material("vent_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    btn_dark = model.material("button_dark", rgba=(0.12, 0.12, 0.14, 1.0))

    # Shared carcass (root) carries all locker bays + the floor plinth.
    carcass = model.part("carcass")

    bank_w = N_BAYS * LOCKER_W + 0.004

    # base plinth running under the whole bank
    carcass.visual(
        Box((bank_w, LOCKER_D, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=grey,
        name="plinth",
    )

    # dividers between adjacent bays (N_BAYS - 1 dividers)
    for i in range(N_BAYS - 1):
        div_x = _bay_center_x(i) + LOCKER_W / 2.0
        carcass.visual(
            Box((WALL, LOCKER_D, LOCKER_H)),
            origin=Origin(xyz=(div_x, 0.0, LOCKER_H / 2.0)),
            material=white,
            name=f"divider_{i}",
        )

    carcass.inertial = Inertial.from_geometry(
        Box((N_BAYS * LOCKER_W, LOCKER_D, LOCKER_H)),
        mass=15.0 * N_BAYS,
        origin=Origin(xyz=(0.0, 0.0, LOCKER_H / 2.0)),
    )

    # Build each bay via a single loop over range(N_BAYS)
    for i in range(N_BAYS):
        _build_bay(
            model,
            i,
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

    # ---- All four bays exist and are equally spaced along X ----
    doors = [object_model.get_part(f"door_{i}") for i in range(N_BAYS)]
    positions = [ctx.part_world_position(d) for d in doors]
    ctx.check(
        "all four bay doors exist",
        all(p is not None for p in positions),
        details=f"positions={positions}",
    )
    # adjacent doors should be LOCKER_W apart in X
    spacings_ok = True
    for i in range(N_BAYS - 1):
        if positions[i] is None or positions[i + 1] is None:
            spacings_ok = False
            break
        dx = positions[i + 1][0] - positions[i][0]
        if abs(dx - LOCKER_W) > 0.02:
            spacings_ok = False
            break
    ctx.check(
        "four bays equally spaced along X",
        spacings_ok,
        details=f"positions={positions}",
    )

    # ---- The bank is centered on x=0 (measure via carcass AABB) ----
    carcass_aabb = ctx.part_world_aabb(carcass)
    if carcass_aabb is not None:
        bank_center = (carcass_aabb[0][0] + carcass_aabb[1][0]) / 2.0
        ctx.check(
            "bank centered on x=0",
            abs(bank_center) < 0.02,
            details=f"bank_center_x={bank_center}",
        )

    # ---- Each door is hinged on its side and the free edge swings out ----
    for i in range(N_BAYS):
        door = object_model.get_part(f"door_{i}")
        hinge = object_model.get_articulation(f"hinge_{i}")
        # door rides in front of the carcass; seated overlap is acceptable
        ctx.allow_overlap(
            door,
            carcass,
            reason=(
                "Door panel overlaps the carcass front face/frame at the hinge "
                "reveal; this is the intended seated/closed fit."
            ),
        )
        # closed-pose free-edge position
        closed_aabb = ctx.part_world_aabb(door)
        closed_y = closed_aabb[1][1]
        with ctx.pose({hinge: math.radians(90.0)}):
            open_aabb = ctx.part_world_aabb(door)
            open_y = open_aabb[1][1]
        ctx.check(
            f"door_{i} free edge swings out when opened",
            open_y > closed_y + 0.10,
            details=f"closed_max_y={closed_y}, open_max_y={open_y}",
        )

    # ---- A sampled lock button on each door presses straight in ----
    for i in range(N_BAYS):
        door = object_model.get_part(f"door_{i}")
        n = 3  # representative button
        btn = object_model.get_part(f"lockbtn_{i}_{n}")
        btnjoint = object_model.get_articulation(f"btnjoint_{i}_{n}")
        ctx.allow_overlap(
            btn,
            door,
            reason="Push-button base is seated into the lock keypad plate on the door.",
        )
        ctx.expect_contact(
            btn, door, name=f"lockbtn_{i}_{n} mounted on door"
        )
        rest = ctx.part_world_position(btn)
        with ctx.pose({btnjoint: BTN_PRESS}):
            pressed = ctx.part_world_position(btn)
        ctx.check(
            f"lockbtn_{i}_{n} presses straight in",
            rest is not None and pressed is not None
            and (rest[1] - pressed[1]) > BTN_PRESS * 0.7,
            details=f"rest={rest}, pressed={pressed}",
        )

    # ---- Vents are present on every door ----
    for i in range(N_BAYS):
        door = object_model.get_part(f"door_{i}")
        vent = door.get_visual(f"door_vent_{i}")
        ctx.check(
            f"door_{i} has a vent visual",
            vent is not None,
            details=f"vent={vent}",
        )

    # ---- N_BAYS-1 dividers exist between adjacent bays ----
    for i in range(N_BAYS - 1):
        div_name = f"divider_{i}"
        div = carcass.get_visual(div_name)
        ctx.check(
            f"divider {i} exists between bays",
            div is not None,
            details=f"visual={div}",
        )

    return ctx.report()


object_model = build_object_model()
