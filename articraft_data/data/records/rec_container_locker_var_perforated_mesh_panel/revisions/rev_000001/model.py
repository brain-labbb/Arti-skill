from __future__ import annotations

# A bank of TWO metal storage lockers standing side by side, with full-height
# perforated mesh ventilation panels on each door face.
# Frame:
#   - X: bank width (the two lockers are arranged along +X), bank centered on x=0.
#   - Y: depth; the doors face the front at +Y.
#   - Z: height; floor at z=0, top of the bank at z~0.90.
# Each locker unit is ~0.30 (W) x 0.45 (D) x 0.90 (H).
# Movers (built by _build_locker, called twice):
#   - door_{idx}: REVOLUTE, hinged on one vertical side, swings outward 0..~100 deg.
#   - lockbtn_{idx}_{n}: PRISMATIC round push-buttons on each door's keypad,
#     children of the door, press straight in ~1.5 mm.
# Ventilation: each door carries a large PerforatedPanelGeometry mesh panel
# covering ~75% of the door face (round-hole staggered grid), replacing a
# simpler louver vent.

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
    mesh_from_geometry,
)

# ---- Bank / locker dimensions ----
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


def _locker_center_x(idx: int) -> float:
    # Two lockers arranged along X, the whole bank centered on x=0.
    return (idx - 0.5) * LOCKER_W


def _build_locker(model: ArticulatedObject, x_offset: float, idx: int, *,
                  carcass: object, white: object, grey: object,
                  dark: object, btn_dark: object) -> None:
    """Build one locker unit at world x = x_offset on the shared carcass.

    Adds the door (revolute) and its keypad push-buttons (prismatic).
    The carcass body is shared across the bank and is passed in as `carcass`.
    """
    # ---- carcass body geometry for this bay (sides / back / top / bottom) ----
    # Modeled hollow: outer shell minus an interior cavity is approximated by
    # plate visuals so the interior reads as an open compartment.
    cx = x_offset
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

    # ---- door (revolute, hinged on the left vertical edge of this bay) ----
    # The door part frame origin is at the hinge line so it swings cleanly.
    # Build the door geometry relative to its own local frame: the hinge edge at
    # local x=0, the door panel extending toward +X (the free edge swings out).
    door = model.part(f"door_{idx}")

    # main door panel: spans the bay width, centered on the bay in its local
    # closed pose. Local frame: hinge at x=0, so panel center is at +DOOR_W/2.
    door.visual(
        Box((DOOR_W, DOOR_TH, DOOR_H)),
        origin=Origin(xyz=(DOOR_W / 2.0, 0.0, 0.0)),
        material=white,
        name=f"door_panel_{idx}",
    )

    # ---- Full-height perforated mesh panel covering most of the door face ----
    # Large perforated sheet-metal panel mounted on the door front (+Y face).
    # Leaves room for the keypad at the bottom and margins around the edges.
    mesh_panel_w = DOOR_W * 0.78
    mesh_panel_h = DOOR_H * 0.74
    mesh_panel_th = 0.003            # thin sheet-metal thickness
    # Position: centered horizontally, biased upward to leave room below for
    # the barcode plate and keypad.
    mesh_cx = DOOR_W / 2.0
    mesh_cz = 0.065                  # slightly above door center
    perf = PerforatedPanelGeometry(
        (mesh_panel_w, mesh_panel_h),
        mesh_panel_th,
        hole_diameter=0.005,
        pitch=(0.013, 0.013),
        frame=0.012,
        corner_radius=0.004,
        stagger=False,
        center=True,
    )
    # Panel lies in local XY, thickness along Z. Rotate -90° around X so the
    # face points along door-local +Y (front) with thickness extending outward.
    perf_mesh = mesh_from_geometry(perf, f"door_mesh_panel_{idx}")
    door.visual(
        perf_mesh,
        origin=Origin(
            xyz=(mesh_cx, DOOR_TH / 2.0 + mesh_panel_th / 2.0 - 0.001, mesh_cz),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark,
        name=f"door_mesh_{idx}",
    )

    # small barcode / number plate between the mesh panel bottom and keypad
    plate_z = mesh_cz - mesh_panel_h / 2.0 - 0.028
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
        origin=Origin(xyz=(DOOR_W / 2.0, DOOR_TH / 2.0, plate_z),
                      rpy=(0.0, 0.0, 0.0)),
        material=dark,
        name=f"door_plate_{idx}",
    )

    # lock keypad mounting plate near the BOTTOM of the door (front face)
    pad_w = BTN_COLS * (2 * BTN_R + 0.004) + 0.010
    pad_h = BTN_ROWS * (2 * BTN_R + 0.004) + 0.010
    pad_cx = DOOR_W / 2.0
    pad_cz = -DOOR_H / 2.0 + 0.110  # near the bottom of the door
    door.visual(
        Box((pad_w, 0.006, pad_h)),
        origin=Origin(xyz=(pad_cx, DOOR_TH / 2.0 + 0.003, pad_cz)),
        material=grey,
        name=f"lockpad_{idx}",
    )

    # handle bar on the door (vertical, near the free edge)
    door.visual(
        Box((0.016, 0.022, 0.16)),
        origin=Origin(xyz=(DOOR_W - 0.030, DOOR_TH / 2.0 + 0.011, 0.06)),
        material=grey,
        name=f"door_handle_{idx}",
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
        f"hinge_{idx}",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, LOCKER_H / 2.0)),
        # Door panel extends along local +X from the hinge; +Z axis swings the
        # free edge toward +Y... we want it to swing OUT (toward +Y/front then
        # around). Rotating about +Z by positive q moves +X toward +Y, i.e. the
        # free edge swings forward and out. Use +Z.
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=2.0, lower=0.0, upper=math.radians(100.0)
        ),
    )

    # ---- keypad push-buttons: children of the door, press straight in ----
    # Buttons protrude from the lock plate front (+Y in door-local frame at rest)
    # and press in along -Y (into the door).
    col_pitch = 2 * BTN_R + 0.004
    row_pitch = 2 * BTN_R + 0.004
    x0 = pad_cx - (BTN_COLS - 1) * col_pitch / 2.0
    z0 = pad_cz - (BTN_ROWS - 1) * row_pitch / 2.0
    btn_front_y = DOOR_TH / 2.0 + 0.006  # button base sits on the lock plate face
    n = 0
    for r in range(BTN_ROWS):
        for c in range(BTN_COLS):
            bx = x0 + c * col_pitch
            bz = z0 + r * row_pitch
            btn = model.part(f"lockbtn_{idx}_{n}")
            # round button: short cylinder, axis along Y (cap faces +Y / front)
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
                # button sits proud of the lock plate, on the door front
                origin=Origin(xyz=(bx, btn_front_y + BTN_LEN / 2.0, bz)),
                # press straight in = toward -Y in door-local frame
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

    # Shared carcass (root) carries both locker bays + the floor plinth.
    carcass = model.part("carcass")

    # base plinth running under the whole bank
    carcass.visual(
        Box((2 * LOCKER_W + 0.004, LOCKER_D, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=grey,
        name="plinth",
    )

    # central divider between the two bays (shared sheet)
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

    for idx in (0, 1):
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


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    carcass = object_model.get_part("carcass")

    # ---- The two lockers are side by side (mirror/translate along X) ----
    door0 = object_model.get_part("door_0")
    door1 = object_model.get_part("door_1")
    p0 = ctx.part_world_position(door0)
    p1 = ctx.part_world_position(door1)
    ctx.check(
        "two doors offset side by side along X",
        p0 is not None and p1 is not None and abs((p1[0] - p0[0]) - LOCKER_W) < 0.02,
        details=f"door0={p0}, door1={p1}",
    )

    # ---- Each door is hinged on its side and the free edge swings out ----
    for idx in (0, 1):
        door = object_model.get_part(f"door_{idx}")
        hinge = object_model.get_articulation(f"hinge_{idx}")
        # door rides in front of the carcass; tiny seated overlap of the panel
        # against the carcass front is acceptable.
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
        closed_y = closed_aabb[1][1]  # max Y (front face)
        with ctx.pose({hinge: math.radians(90.0)}):
            open_aabb = ctx.part_world_aabb(door)
            open_y = open_aabb[1][1]
        # opening the door swings the free edge forward (front of the bank moves
        # out in +Y substantially as the panel rotates to face sideways).
        ctx.check(
            f"door_{idx} free edge swings out when opened",
            open_y > closed_y + 0.10,
            details=f"closed_max_y={closed_y}, open_max_y={open_y}",
        )

    # ---- A sampled lock button on each door presses straight in and seats ----
    for idx in (0, 1):
        door = object_model.get_part(f"door_{idx}")
        n = 3  # representative button
        btn = object_model.get_part(f"lockbtn_{idx}_{n}")
        btnjoint = object_model.get_articulation(f"btnjoint_{idx}_{n}")
        # button base intentionally embeds into the lock plate / door front.
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
        # pressing moves the button into the door (toward -Y in world at rest).
        ctx.check(
            f"lockbtn_{idx}_{n} presses straight in",
            rest is not None and pressed is not None
            and (rest[1] - pressed[1]) > BTN_PRESS * 0.7,
            details=f"rest={rest}, pressed={pressed}",
        )

    # ---- Full-height perforated mesh panels on the doors (hero feature) ----
    for idx in (0, 1):
        door = object_model.get_part(f"door_{idx}")
        mesh_panel = door.get_visual(f"door_mesh_{idx}")
        ctx.check(
            f"door_{idx} has a perforated mesh panel",
            mesh_panel is not None,
            details=f"mesh_panel={mesh_panel}",
        )
        # The mesh panel should be large — covering most of the door face.
        # Check that the panel height is at least 60% of the door height.
        if mesh_panel is not None:
            panel_aabb = ctx.part_element_world_aabb(door, elem=f"door_mesh_{idx}")
            if panel_aabb is not None:
                panel_h = panel_aabb[1][2] - panel_aabb[0][2]
                ctx.check(
                    f"door_{idx} mesh panel is full-height (>60% of door)",
                    panel_h > DOOR_H * 0.60,
                    details=f"panel_h={panel_h:.4f}, door_h={DOOR_H:.4f}",
                )

    return ctx.report()


object_model = build_object_model()
