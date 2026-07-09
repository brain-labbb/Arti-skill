from __future__ import annotations

# Roller-shutter locker bank: fork of the two-locker hinged-door parent.
# Only the door closure mechanism changes: solid hinged doors are replaced by
# roll-down corrugated shutters (stack of horizontal slats in a for-i-in-range
# loop) that slide vertically along side guide channels via a PRISMATIC joint
# up into a head box at the top of each bay.
#
# Frame:
#   X: bank width (two bays side by side, centered on x=0).
#   Y: depth; the shutters face the front at +Y.
#   Z: height; floor at z=0, top at z~0.90.
# Each bay: ~0.30 (W) x 0.45 (D) x 0.90 (H).
#
# Movers per bay:
#   shutter_{idx}: PRISMATIC, slides up along +Z into the head box.
#   lockbtn_{idx}_{n}: PRISMATIC push-buttons on the fixed keypad below the
#     opening, pressing straight in along -Y.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- Bank / bay dimensions (same carcass footprint as parent) ----
LOCKER_W = 0.30
LOCKER_D = 0.45
LOCKER_H = 0.90
WALL = 0.012              # carcass sheet-metal thickness
CARCASS_FRONT_Y = LOCKER_D / 2.0

# ---- Shutter mechanism ----
GUIDE_W = 0.014           # guide-channel visible width
GUIDE_DEPTH = 0.018       # guide-channel depth (front to back)
SHUTTER_W = LOCKER_W - 2 * WALL - 2 * GUIDE_W  # visible slat span

SLAT_H = 0.022            # slat visible height
SLAT_D = 0.010            # slat depth (front to back)
SLAT_BULGE = 0.003        # convex bulge on the front face (corrugation)
SLAT_GAP = 0.002          # air gap between consecutive slats
SLAT_PITCH = SLAT_H + SLAT_GAP

HEAD_BOX_H = 0.065        # head-box housing height at top
OPENING_BOTTOM_Z = 0.160  # bottom of the shutter opening
OPENING_TOP_Z = LOCKER_H - WALL - HEAD_BOX_H  # 0.823
OPENING_H = OPENING_TOP_Z - OPENING_BOTTOM_Z   # 0.663

N_SLATS = int(OPENING_H / SLAT_PITCH)           # 27
SLAT_STACK_H = N_SLATS * SLAT_PITCH             # total stack height
SHUTTER_TRAVEL = (N_SLATS - 4) * SLAT_PITCH     # leave ~4 slats visible open

# ---- Keypad (same button count and sizes as parent) ----
BTN_R = 0.011
BTN_PRESS = 0.0015
BTN_LEN = 0.010
BTN_COLS = 5
BTN_ROWS = 2
BTN_N = BTN_COLS * BTN_ROWS

# ---- Handle ----
HANDLE_W = 0.100
HANDLE_D = 0.020
HANDLE_H = 0.016


def _locker_center_x(idx: int) -> float:
    """World X center for bay idx in a two-bay bank centered on x=0."""
    return (idx - 0.5) * LOCKER_W


def _build_slat_cq(slat_w: float, slat_d: float, slat_h: float,
                   bulge: float):
    """Build one corrugated shutter slat with CadQuery.

    Width along X (from x=0 to x=slat_w), depth along Y, height along Z.
    The front face (+Y) has a convex bulge for the corrugated look.
    The visual origin offsets X by -slat_w/2 to center the slat on the bay.
    """
    hd = slat_d / 2.0
    hh = slat_h / 2.0
    # Workplane("YZ"): workplane-x -> world-Y, workplane-y -> world-Z,
    # extrude normal -> world +X.
    shape = (
        cq.Workplane("YZ")
        .moveTo(-hd, -hh)
        .lineTo(hd, -hh)
        .lineTo(hd + bulge, 0.0)
        .lineTo(hd, hh)
        .lineTo(-hd, hh)
        .close()
        .extrude(slat_w)
    )
    return shape


def _build_locker_bay(model, cx, idx, *, carcass,
                      white, grey, dark, btn_dark, shutter_mat, channel_mat):
    """Build one locker bay: carcass panels, guide channels, head box,
    front panel, keypad, shutter with slats, handle, and buttons."""

    # ---- Carcass shell panels (same as parent) ----
    carcass.visual(
        Box((LOCKER_W, WALL, LOCKER_H)),
        origin=Origin(xyz=(cx, -LOCKER_D / 2.0 + WALL / 2.0, LOCKER_H / 2.0)),
        material=grey, name=f"back_{idx}",
    )
    carcass.visual(
        Box((WALL, LOCKER_D, LOCKER_H)),
        origin=Origin(xyz=(cx - LOCKER_W / 2.0 + WALL / 2.0, 0.0, LOCKER_H / 2.0)),
        material=white, name=f"side_l_{idx}",
    )
    carcass.visual(
        Box((WALL, LOCKER_D, LOCKER_H)),
        origin=Origin(xyz=(cx + LOCKER_W / 2.0 - WALL / 2.0, 0.0, LOCKER_H / 2.0)),
        material=white, name=f"side_r_{idx}",
    )
    carcass.visual(
        Box((LOCKER_W, LOCKER_D, WALL)),
        origin=Origin(xyz=(cx, 0.0, LOCKER_H - WALL / 2.0)),
        material=white, name=f"top_{idx}",
    )
    carcass.visual(
        Box((LOCKER_W, LOCKER_D, WALL)),
        origin=Origin(xyz=(cx, 0.0, WALL / 2.0)),
        material=white, name=f"bottom_{idx}",
    )
    carcass.visual(
        Box((LOCKER_W - 2 * WALL, LOCKER_D - WALL, WALL * 0.7)),
        origin=Origin(xyz=(cx, WALL / 2.0, LOCKER_H * 0.55)),
        material=grey, name=f"shelf_{idx}",
    )

    # ---- Front panel below the shutter opening ----
    fp_bottom = 0.030  # top of plinth
    fp_h = OPENING_BOTTOM_Z - fp_bottom
    carcass.visual(
        Box((LOCKER_W - 2 * WALL, WALL, fp_h)),
        origin=Origin(xyz=(cx, CARCASS_FRONT_Y - WALL / 2.0,
                           fp_bottom + fp_h / 2.0)),
        material=white, name=f"front_panel_{idx}",
    )

    # ---- Head-box front panel (above the opening) ----
    hb_cz = OPENING_TOP_Z + HEAD_BOX_H / 2.0
    carcass.visual(
        Box((LOCKER_W - 2 * WALL, WALL, HEAD_BOX_H)),
        origin=Origin(xyz=(cx, CARCASS_FRONT_Y - WALL / 2.0, hb_cz)),
        material=white, name=f"head_box_{idx}",
    )

    # ---- Guide channels (vertical strips flanking the shutter opening) ----
    guide_cz = OPENING_BOTTOM_Z + OPENING_H / 2.0
    for side_sign, side_name in [(-1, "l"), (1, "r")]:
        gx = cx + side_sign * (LOCKER_W / 2.0 - WALL - GUIDE_W / 2.0)
        carcass.visual(
            Box((GUIDE_W, GUIDE_DEPTH, OPENING_H + HEAD_BOX_H)),
            origin=Origin(xyz=(gx, CARCASS_FRONT_Y, guide_cz + HEAD_BOX_H / 2.0)),
            material=channel_mat, name=f"guide_{side_name}_{idx}",
        )

    # ---- Keypad plate on the front panel below the opening ----
    col_pitch = 2 * BTN_R + 0.004
    row_pitch = 2 * BTN_R + 0.004
    pad_w = BTN_COLS * col_pitch + 0.010
    pad_h = BTN_ROWS * row_pitch + 0.010
    pad_cz = 0.100
    pad_front_y = CARCASS_FRONT_Y + 0.006  # proud of the front panel face
    carcass.visual(
        Box((pad_w, 0.006, pad_h)),
        origin=Origin(xyz=(cx, pad_front_y - 0.003, pad_cz)),
        material=grey, name=f"lockpad_{idx}",
    )

    # ---- Shutter part: corrugated slats + handle + connecting strip ----
    shutter = model.part(f"shutter_{idx}")

    # Shared slat mesh (created once, reused for all N_SLATS visuals)
    slat_shape = _build_slat_cq(SHUTTER_W, SLAT_D, SLAT_H, SLAT_BULGE)
    slat_mesh = mesh_from_cadquery(slat_shape, f"slat_{idx}")

    # Emit slats in a for-i-in-range loop
    for i in range(N_SLATS):
        slat_z = i * SLAT_PITCH + SLAT_H / 2.0
        shutter.visual(
            slat_mesh,
            origin=Origin(xyz=(-SHUTTER_W / 2.0, 0.0, slat_z)),
            material=shutter_mat,
            name=f"slat_{idx}_{i}",
        )

    # Connecting strip behind the slats (ensures geometry connectivity;
    # represents the interlocking curtain backing of a real roller shutter).
    strip_th = 0.003
    strip_y_center = -SLAT_D / 2.0 + strip_th / 2.0  # overlaps slat backs
    shutter.visual(
        Box((SHUTTER_W * 0.92, strip_th, SLAT_STACK_H)),
        origin=Origin(xyz=(0.0, strip_y_center, SLAT_STACK_H / 2.0)),
        material=dark,
        name=f"curtain_strip_{idx}",
    )

    # Pull handle at the bottom of the shutter (front face of the first slat)
    handle_y = SLAT_D / 2.0 + SLAT_BULGE + HANDLE_D / 2.0
    handle_z = SLAT_H * 0.40
    shutter.visual(
        Box((HANDLE_W, HANDLE_D, HANDLE_H)),
        origin=Origin(xyz=(0.0, handle_y, handle_z)),
        material=grey,
        name=f"handle_{idx}",
    )

    shutter.inertial = Inertial.from_geometry(
        Box((SHUTTER_W, SLAT_D, SLAT_STACK_H)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, SLAT_STACK_H / 2.0)),
    )

    # ---- PRISMATIC joint: shutter slides up along +Z ----
    # Joint origin on the carcass at the bottom-center of the opening.
    # At q=0 the shutter is fully closed; positive q raises it.
    model.articulation(
        f"slide_{idx}",
        ArticulationType.PRISMATIC,
        parent=carcass,
        child=shutter,
        origin=Origin(xyz=(cx, CARCASS_FRONT_Y, OPENING_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0, velocity=0.30,
            lower=0.0, upper=SHUTTER_TRAVEL,
        ),
    )

    # ---- Keypad push-buttons: children of the carcass ----
    x0 = cx - (BTN_COLS - 1) * col_pitch / 2.0
    z0 = pad_cz - (BTN_ROWS - 1) * row_pitch / 2.0
    btn_front_y = pad_front_y + BTN_LEN / 2.0
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
                Box((2 * BTN_R, BTN_LEN, 2 * BTN_R)), mass=0.004,
            )
            model.articulation(
                f"btnjoint_{idx}_{n}",
                ArticulationType.PRISMATIC,
                parent=carcass,
                child=btn,
                origin=Origin(xyz=(bx, btn_front_y, bz)),
                axis=(0.0, -1.0, 0.0),
                motion_limits=MotionLimits(
                    effort=5.0, velocity=0.05,
                    lower=0.0, upper=BTN_PRESS,
                ),
            )
            n += 1


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="locker_bank_shutter")

    # Materials
    white = model.material("locker_white", rgba=(0.90, 0.90, 0.88, 1.0))
    grey = model.material("locker_grey", rgba=(0.62, 0.64, 0.66, 1.0))
    dark = model.material("vent_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    btn_dark = model.material("button_dark", rgba=(0.12, 0.12, 0.14, 1.0))
    shutter_mat = model.material("shutter_silver", rgba=(0.78, 0.80, 0.82, 1.0))
    channel_mat = model.material("channel_grey", rgba=(0.48, 0.50, 0.52, 1.0))

    # Shared carcass (root)
    carcass = model.part("carcass")

    # Base plinth under the whole bank
    carcass.visual(
        Box((2 * LOCKER_W + 0.004, LOCKER_D, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=grey, name="plinth",
    )
    # Central divider
    carcass.visual(
        Box((WALL, LOCKER_D, LOCKER_H)),
        origin=Origin(xyz=(0.0, 0.0, LOCKER_H / 2.0)),
        material=white, name="divider",
    )
    carcass.inertial = Inertial.from_geometry(
        Box((2 * LOCKER_W, LOCKER_D, LOCKER_H)),
        mass=30.0,
        origin=Origin(xyz=(0.0, 0.0, LOCKER_H / 2.0)),
    )

    for idx in (0, 1):
        _build_locker_bay(
            model, _locker_center_x(idx), idx,
            carcass=carcass,
            white=white, grey=grey, dark=dark, btn_dark=btn_dark,
            shutter_mat=shutter_mat, channel_mat=channel_mat,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")

    # ---- Two shutters side by side ----
    sh0 = object_model.get_part("shutter_0")
    sh1 = object_model.get_part("shutter_1")
    p0 = ctx.part_world_position(sh0)
    p1 = ctx.part_world_position(sh1)
    ctx.check(
        "two shutters offset side by side along X",
        p0 is not None and p1 is not None
        and abs((p1[0] - p0[0]) - LOCKER_W) < 0.02,
        details=f"shutter0={p0}, shutter1={p1}",
    )

    # ---- Per-bay checks ----
    for idx in (0, 1):
        shutter = object_model.get_part(f"shutter_{idx}")
        slide = object_model.get_articulation(f"slide_{idx}")

        # The shutter overlaps the carcass guide channels and head-box region
        # by design (slats ride inside the channels, and the curtain stacks
        # into the head box). Allow this scoped overlap.
        ctx.allow_overlap(
            shutter, carcass,
            reason=(
                "Shutter slats ride inside the carcass guide channels and "
                "stack into the head-box housing; this is the intended "
                "prismatic fit."
            ),
        )

        # Each shutter has exactly N_SLATS slat visuals
        slat_names = [
            v.name for v in shutter.visuals
            if v.name is not None and v.name.startswith(f"slat_{idx}_")
        ]
        ctx.check(
            f"shutter_{idx} has {N_SLATS} corrugated slats",
            len(slat_names) == N_SLATS,
            details=f"found {len(slat_names)} slat visuals",
        )

        # Slats are corrugated: Y depth includes the bulge (> SLAT_D)
        slat0 = shutter.get_visual(f"slat_{idx}_0")
        if slat0 is not None:
            slat_aabb = ctx.part_element_world_aabb(
                shutter, elem=f"slat_{idx}_0"
            )
            if slat_aabb is not None:
                dy = slat_aabb[1][1] - slat_aabb[0][1]
                ctx.check(
                    f"slat_{idx}_0 has corrugated bulge (depth > {SLAT_D})",
                    dy > SLAT_D + 0.0005,
                    details=f"slat depth={dy:.4f}, expected > {SLAT_D}",
                )

        # Positive pose raises the shutter along +Z
        rest_pos = ctx.part_world_position(shutter)
        with ctx.pose({slide: SHUTTER_TRAVEL}):
            open_pos = ctx.part_world_position(shutter)
        ctx.check(
            f"shutter_{idx} slides up when opened",
            rest_pos is not None and open_pos is not None
            and open_pos[2] > rest_pos[2] + SHUTTER_TRAVEL * 0.9,
            details=f"rest_z={rest_pos}, open_z={open_pos}",
        )

        # Guide channels exist on the carcass for this bay
        for side in ("l", "r"):
            gv = carcass.get_visual(f"guide_{side}_{idx}")
            ctx.check(
                f"guide channel guide_{side}_{idx} exists",
                gv is not None,
            )

        # Head box exists on the carcass
        hb = carcass.get_visual(f"head_box_{idx}")
        ctx.check(
            f"head_box_{idx} exists",
            hb is not None,
        )

    # ---- Button press checks ----
    for idx in (0, 1):
        n = 3  # representative button
        btn = object_model.get_part(f"lockbtn_{idx}_{n}")
        btnjoint = object_model.get_articulation(f"btnjoint_{idx}_{n}")

        ctx.allow_overlap(
            btn, carcass,
            reason="Push-button base is seated into the lock keypad plate on the carcass front panel.",
        )
        ctx.expect_contact(
            btn, carcass,
            name=f"lockbtn_{idx}_{n} mounted on carcass",
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

    return ctx.report()


object_model = build_object_model()
