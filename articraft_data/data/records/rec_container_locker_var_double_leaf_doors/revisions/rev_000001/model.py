from __future__ import annotations

# A bank of TWO metal storage lockers standing side by side.
# Frame:
#   - X: bank width (the two lockers are arranged along +X), bank centered on x=0.
#   - Y: depth; the doors face the front at +Y.
#   - Z: height; floor at z=0, top of the bank at z~0.90.
# Each locker unit is ~0.30 (W) x 0.45 (D) x 0.90 (H).
# Each bay has a pair of narrow double-leaf doors meeting at the bay centerline:
#   - door_{idx}_{0}: left leaf, REVOLUTE on the left vertical edge, swings outward.
#   - door_{idx}_{1}: right leaf, REVOLUTE on the right vertical edge, swings outward.
#   - lockbtn_{idx}_{n}: PRISMATIC round push-buttons on the left leaf keypad,
#     children of the left leaf, press straight in ~1.5 mm.

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
LOCKER_W = 0.30
LOCKER_D = 0.45
LOCKER_H = 0.90
DOOR_TH = 0.018          # door panel thickness
WALL = 0.012             # carcass sheet-metal thickness (visual)
CARCASS_FRONT_Y = LOCKER_D / 2.0   # front plane of carcass (door sits just in front)

# Door hinge geometry
HINGE_GAP = 0.004        # small reveal between doors / frame
CENTER_GAP = 0.003       # gap where the two leaves meet at bay centerline
DOOR_W = LOCKER_W - 2 * HINGE_GAP  # total usable door width inside the bay
LEAF_W = (DOOR_W - CENTER_GAP) / 2.0  # each leaf is half the door width
DOOR_H = LOCKER_H - 2 * HINGE_GAP
DOOR_FRONT_Y = CARCASS_FRONT_Y + DOOR_TH / 2.0  # door panel centerline in Y

# Keypad button layout (a small keypad of round buttons near the bottom)
BTN_R = 0.011
BTN_PRESS = 0.0015       # 1.5 mm press-in travel
BTN_LEN = 0.010          # button cylinder length (sticks out of the lock plate)
BTN_COLS = 5
BTN_ROWS = 2
BTN_N = BTN_COLS * BTN_ROWS  # 10 buttons per bay (on left leaf)


def _locker_center_x(idx: int) -> float:
    # Two lockers arranged along X, the whole bank centered on x=0.
    return (idx - 0.5) * LOCKER_W


def _build_leaf(model: ArticulatedObject, x_offset: float, bay_idx: int,
                leaf_idx: int, *, carcass: object,
                white: object, grey: object, dark: object,
                btn_dark: object) -> object:
    """Build one door leaf for the given bay and leaf index.

    leaf_idx=0: left leaf, hinged on the left vertical edge of the bay.
    leaf_idx=1: right leaf, hinged on the right vertical edge of the bay.

    Returns the leaf part so callers can attach child parts (buttons).
    """
    cx = x_offset
    leaf = model.part(f"door_{bay_idx}_{leaf_idx}")

    # ---- Hinge position and axis convention ----
    if leaf_idx == 0:
        # Left leaf: hinge on the left edge, panel extends +X from hinge
        hinge_x = cx - DOOR_W / 2.0
        panel_local_cx = LEAF_W / 2.0
        axis = (0.0, 0.0, 1.0)   # +Z: positive q swings +X toward +Y (outward)
    else:
        # Right leaf: hinge on the right edge, panel extends -X from hinge
        hinge_x = cx + DOOR_W / 2.0
        panel_local_cx = -LEAF_W / 2.0
        axis = (0.0, 0.0, -1.0)  # -Z: positive q swings -X toward +Y (outward)

    # ---- Main door panel ----
    leaf.visual(
        Box((LEAF_W, DOOR_TH, DOOR_H)),
        origin=Origin(xyz=(panel_local_cx, 0.0, 0.0)),
        material=white,
        name=f"door_panel_{bay_idx}_{leaf_idx}",
    )

    # ---- Recessed louver vent near the top of the leaf ----
    vent_w = LEAF_W * 0.70
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
    vent_mesh = mesh_from_geometry(vent, f"vent_{bay_idx}_{leaf_idx}")
    leaf.visual(
        vent_mesh,
        origin=Origin(xyz=(panel_local_cx, DOOR_TH / 2.0 - 0.006, vent_z),
                      rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name=f"door_vent_{bay_idx}_{leaf_idx}",
    )

    # ---- Handle near the free (meeting) edge ----
    if leaf_idx == 0:
        handle_local_x = LEAF_W - 0.022
    else:
        handle_local_x = -(LEAF_W - 0.022)
    leaf.visual(
        Box((0.016, 0.022, 0.16)),
        origin=Origin(xyz=(handle_local_x, DOOR_TH / 2.0 + 0.011, 0.06)),
        material=grey,
        name=f"door_handle_{bay_idx}_{leaf_idx}",
    )

    # ---- Leaf-specific features ----
    if leaf_idx == 0:
        # Barcode / number plate below the vent (left leaf)
        plate = PerforatedPanelGeometry(
            (0.045, 0.030),
            0.0025,
            hole_diameter=0.0016,
            pitch=(0.0055, 0.0055),
            frame=0.005,
            corner_radius=0.002,
        )
        plate_mesh = mesh_from_geometry(plate, f"plate_{bay_idx}")
        leaf.visual(
            plate_mesh,
            origin=Origin(xyz=(panel_local_cx, DOOR_TH / 2.0, vent_z - 0.075),
                          rpy=(0.0, 0.0, 0.0)),
            material=dark,
            name=f"door_plate_{bay_idx}",
        )

        # ---- Lock keypad mounting plate near the bottom of the leaf ----
        pad_w = BTN_COLS * (2 * BTN_R + 0.004) + 0.010
        pad_h = BTN_ROWS * (2 * BTN_R + 0.004) + 0.010
        pad_cx = panel_local_cx
        pad_cz = -DOOR_H / 2.0 + 0.110
        leaf.visual(
            Box((pad_w, 0.006, pad_h)),
            origin=Origin(xyz=(pad_cx, DOOR_TH / 2.0 + 0.003, pad_cz)),
            material=grey,
            name=f"lockpad_{bay_idx}",
        )

        leaf.inertial = Inertial.from_geometry(
            Box((LEAF_W, DOOR_TH, DOOR_H)),
            mass=2.0,
            origin=Origin(xyz=(panel_local_cx, 0.0, 0.0)),
        )

        # ---- Keypad push-buttons: children of the left leaf ----
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
                btn = model.part(f"lockbtn_{bay_idx}_{n}")
                cyl = CylinderGeometry(BTN_R, BTN_LEN, radial_segments=20).rotate_x(
                    math.pi / 2.0
                )
                btn.visual(
                    mesh_from_geometry(cyl, f"btn_{bay_idx}_{n}"),
                    origin=Origin(xyz=(0.0, 0.0, 0.0)),
                    material=btn_dark,
                    name=f"btn_cap_{bay_idx}_{n}",
                )
                btn.inertial = Inertial.from_geometry(
                    Box((2 * BTN_R, BTN_LEN, 2 * BTN_R)), mass=0.004
                )
                model.articulation(
                    f"btnjoint_{bay_idx}_{n}",
                    ArticulationType.PRISMATIC,
                    parent=leaf,
                    child=btn,
                    origin=Origin(xyz=(bx, btn_front_y + BTN_LEN / 2.0, bz)),
                    axis=(0.0, -1.0, 0.0),
                    motion_limits=MotionLimits(
                        effort=5.0, velocity=0.05, lower=0.0, upper=BTN_PRESS
                    ),
                )
                n += 1
    else:
        # Right leaf: barcode / number plate below the vent
        plate = PerforatedPanelGeometry(
            (0.045, 0.030),
            0.0025,
            hole_diameter=0.0016,
            pitch=(0.0055, 0.0055),
            frame=0.005,
            corner_radius=0.002,
        )
        plate_mesh = mesh_from_geometry(plate, f"plate_{bay_idx}_{leaf_idx}")
        leaf.visual(
            plate_mesh,
            origin=Origin(xyz=(panel_local_cx, DOOR_TH / 2.0, vent_z - 0.075),
                          rpy=(0.0, 0.0, 0.0)),
            material=dark,
            name=f"door_plate_{bay_idx}_{leaf_idx}",
        )

        leaf.inertial = Inertial.from_geometry(
            Box((LEAF_W, DOOR_TH, DOOR_H)),
            mass=2.0,
            origin=Origin(xyz=(panel_local_cx, 0.0, 0.0)),
        )

    # ---- Hinge articulation ----
    model.articulation(
        f"hinge_{bay_idx}_{leaf_idx}",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=leaf,
        origin=Origin(xyz=(hinge_x, DOOR_FRONT_Y, LOCKER_H / 2.0)),
        axis=axis,
        motion_limits=MotionLimits(
            effort=20.0, velocity=2.0, lower=0.0, upper=math.radians(100.0)
        ),
    )

    return leaf


def _build_locker(model: ArticulatedObject, x_offset: float, idx: int, *,
                  carcass: object, white: object, grey: object,
                  dark: object, btn_dark: object) -> None:
    """Build one locker bay at world x = x_offset on the shared carcass.

    Adds the double-leaf doors (revolute each) and keypad push-buttons.
    The carcass body is shared across the bank and is passed in as `carcass`.
    """
    cx = x_offset

    # ---- carcass body geometry for this bay (sides / back / top / bottom) ----
    carcass.visual(
        Box((LOCKER_W, WALL, LOCKER_H)),
        origin=Origin(xyz=(cx, -LOCKER_D / 2.0 + WALL / 2.0, LOCKER_H / 2.0)),
        material=grey,
        name=f"back_{idx}",
    )
    carcass.visual(
        Box((WALL, LOCKER_D, LOCKER_H)),
        origin=Origin(xyz=(cx - LOCKER_W / 2.0 + WALL / 2.0, 0.0, LOCKER_H / 2.0)),
        material=white,
        name=f"side_l_{idx}",
    )
    carcass.visual(
        Box((WALL, LOCKER_D, LOCKER_H)),
        origin=Origin(xyz=(cx + LOCKER_W / 2.0 - WALL / 2.0, 0.0, LOCKER_H / 2.0)),
        material=white,
        name=f"side_r_{idx}",
    )
    carcass.visual(
        Box((LOCKER_W, LOCKER_D, WALL)),
        origin=Origin(xyz=(cx, 0.0, LOCKER_H - WALL / 2.0)),
        material=white,
        name=f"top_{idx}",
    )
    carcass.visual(
        Box((LOCKER_W, LOCKER_D, WALL)),
        origin=Origin(xyz=(cx, 0.0, WALL / 2.0)),
        material=white,
        name=f"bottom_{idx}",
    )
    # interior shelf
    carcass.visual(
        Box((LOCKER_W - 2 * WALL, LOCKER_D - WALL, WALL * 0.7)),
        origin=Origin(xyz=(cx, WALL / 2.0, LOCKER_H * 0.55)),
        material=grey,
        name=f"shelf_{idx}",
    )

    # ---- Build the two door leaves via shared helper ----
    for leaf_idx in range(2):
        _build_leaf(
            model, x_offset, idx, leaf_idx,
            carcass=carcass, white=white, grey=grey,
            dark=dark, btn_dark=btn_dark,
        )


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

    for idx in range(2):
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

    # ---- Each bay has exactly two door leaves ----
    for idx in range(2):
        leaf0 = object_model.get_part(f"door_{idx}_0")
        leaf1 = object_model.get_part(f"door_{idx}_1")
        ctx.check(
            f"bay {idx} has two door leaves",
            leaf0 is not None and leaf1 is not None,
            details=f"leaf0={leaf0}, leaf1={leaf1}",
        )

    # ---- The two bays are side by side along X ----
    l0_bay0 = object_model.get_part("door_0_0")
    l0_bay1 = object_model.get_part("door_1_0")
    p0 = ctx.part_world_position(l0_bay0)
    p1 = ctx.part_world_position(l0_bay1)
    ctx.check(
        "two bays offset side by side along X",
        p0 is not None and p1 is not None and abs((p1[0] - p0[0]) - LOCKER_W) < 0.02,
        details=f"bay0_left={p0}, bay1_left={p1}",
    )

    # ---- Each leaf swings outward and the two leaves of a bay open in opposite directions ----
    for idx in range(2):
        for leaf_idx in range(2):
            leaf = object_model.get_part(f"door_{idx}_{leaf_idx}")
            hinge = object_model.get_articulation(f"hinge_{idx}_{leaf_idx}")

            # Door leaf overlaps the carcass front face at the hinge reveal
            ctx.allow_overlap(
                leaf,
                carcass,
                reason=(
                    f"Door leaf {idx}_{leaf_idx} overlaps the carcass front "
                    "face/frame at the hinge reveal; this is the intended "
                    "seated/closed fit."
                ),
            )

            closed_aabb = ctx.part_world_aabb(leaf)
            closed_max_y = closed_aabb[1][1]

            with ctx.pose({hinge: math.radians(90.0)}):
                open_aabb = ctx.part_world_aabb(leaf)
                open_max_y = open_aabb[1][1]

            ctx.check(
                f"door_{idx}_{leaf_idx} free edge swings outward when opened",
                open_max_y > closed_max_y + 0.05,
                details=f"closed_max_y={closed_max_y}, open_max_y={open_max_y}",
            )

        # The two leaves of the same bay should have their hinge lines on
        # opposite sides (one at -X edge, one at +X edge of the bay).
        hinge0 = object_model.get_articulation(f"hinge_{idx}_0")
        hinge1 = object_model.get_articulation(f"hinge_{idx}_1")
        h0_pos = ctx.part_world_position(object_model.get_part(f"door_{idx}_0"))
        h1_pos = ctx.part_world_position(object_model.get_part(f"door_{idx}_1"))
        ctx.check(
            f"bay {idx} leaves have hinges on opposite edges",
            h0_pos is not None and h1_pos is not None
            and abs(h1_pos[0] - h0_pos[0]) > LEAF_W * 0.5,
            details=f"leaf0_pos={h0_pos}, leaf1_pos={h1_pos}",
        )

    # ---- A sampled lock button on each bay presses straight in and seats ----
    for idx in range(2):
        leaf0 = object_model.get_part(f"door_{idx}_0")
        n = 3  # representative button
        btn = object_model.get_part(f"lockbtn_{idx}_{n}")
        btnjoint = object_model.get_articulation(f"btnjoint_{idx}_{n}")

        ctx.allow_overlap(
            btn,
            leaf0,
            reason="Push-button base is seated into the lock keypad plate on the leaf.",
        )
        ctx.expect_contact(
            btn, leaf0, name=f"lockbtn_{idx}_{n} mounted on left leaf"
        )
        rest = ctx.part_world_position(btn)
        with ctx.pose({btnjoint: BTN_PRESS}):
            pressed = ctx.part_world_position(btn)
        ctx.check(
            f"lockbtn_{idx}_{n} presses straight in",
            rest is not None and pressed is not None
            and (rest[1] - pressed[1]) > BTN_PRESS * 0.7,
            details=f"rest={rest}, pressed={pressed}",
        )

    # ---- Vents are present on each leaf (hero feature) ----
    for idx in range(2):
        for leaf_idx in range(2):
            leaf = object_model.get_part(f"door_{idx}_{leaf_idx}")
            vent = leaf.get_visual(f"door_vent_{idx}_{leaf_idx}")
            ctx.check(
                f"door_{idx}_{leaf_idx} has a vent visual",
                vent is not None,
                details=f"vent={vent}",
            )

    # ---- Each leaf has a handle near the meeting edge ----
    for idx in range(2):
        for leaf_idx in range(2):
            leaf = object_model.get_part(f"door_{idx}_{leaf_idx}")
            handle = leaf.get_visual(f"door_handle_{idx}_{leaf_idx}")
            ctx.check(
                f"door_{idx}_{leaf_idx} has a handle",
                handle is not None,
                details=f"handle={handle}",
            )

    # ---- Left leaf has the keypad ----
    for idx in range(2):
        leaf0 = object_model.get_part(f"door_{idx}_0")
        lockpad = leaf0.get_visual(f"lockpad_{idx}")
        ctx.check(
            f"bay {idx} left leaf has the keypad",
            lockpad is not None,
            details=f"lockpad={lockpad}",
        )

    return ctx.report()


object_model = build_object_model()
