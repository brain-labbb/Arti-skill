from __future__ import annotations

# Free-standing top-freezer refrigerator, glossy black.
# Frame:
#   - X: width (0.65 m), cabinet centered on x=0; the hinge edge is at +X (right).
#   - Y: depth (~0.68 m incl. doors); the doors face the front at +Y.
#   - Z: height; floor at z=0, top of the cabinet at z=1.75.
# Structure:
#   - cabinet (root): hollow insulated carcass (side/back/top/bottom walls),
#     a horizontal mullion divider splitting a top freezer compartment
#     (~30% of height) from the fresh-food compartment below, interior
#     shelves, and a recessed dark-gray plinth/kick panel at the bottom front.
#   - freezer_door / fridge_door: full-width slabs with a slightly rounded
#     front face (CadQuery fillet), each carrying a tall full-height tubular
#     pro-style vertical handle on standoffs near the free edge; a small
#     silver brand badge sits on the upper freezer door.
# Articulation:
#   - freezer_door_hinge / fridge_door_hinge: independent REVOLUTE joints on
#     the same vertical axis line at the right front edge of the cabinet,
#     0 .. ~120 deg. Door panels extend along local -X from the hinge, so
#     axis (0, 0, -1) makes positive q swing the free edge outward (+Y).

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- Overall dimensions (meters) ----
W = 0.65                  # cabinet width
H = 1.75                  # overall height
BODY_BACK_Y = -0.34       # back face of the carcass
BODY_FRONT_Y = 0.28       # front face of the carcass (doors sit just in front)
WALL = 0.04               # insulated wall thickness
BODY_BOTTOM_Z = 0.05      # carcass bottom (plinth below)

# ---- Doors ----
DOOR_TH = 0.055           # door slab thickness
DOOR_W = 0.64             # full-width door slab (0.005 reveal each side)
DOOR_GAP_Y = 0.004        # thin gap between carcass front and door back
DOOR_CY = BODY_FRONT_Y + DOOR_GAP_Y + DOOR_TH / 2.0  # door slab centerline in Y

MULLION_CZ = 1.21         # compartment split height
MULLION_TH = 0.04

FRIDGE_DOOR_Z0 = 0.06
FRIDGE_DOOR_Z1 = 1.195
FREEZER_DOOR_Z0 = 1.225
FREEZER_DOOR_Z1 = 1.73
FRIDGE_DOOR_H = FRIDGE_DOOR_Z1 - FRIDGE_DOOR_Z0      # 1.135
FREEZER_DOOR_H = FREEZER_DOOR_Z1 - FREEZER_DOOR_Z0   # 0.505 (~29% of H)
FRIDGE_DOOR_CZ = (FRIDGE_DOOR_Z0 + FRIDGE_DOOR_Z1) / 2.0
FREEZER_DOOR_CZ = (FREEZER_DOOR_Z0 + FREEZER_DOOR_Z1) / 2.0

HINGE_X = W / 2.0 - 0.005  # vertical hinge line at the right front edge

# ---- Handles (tall tubular pro-style, vertical, on standoffs) ----
HANDLE_TUBE_R = 0.012          # main tube radius
HANDLE_TUBE_FRAC = 0.80        # tube length as fraction of door height
HANDLE_TUBE_CX = -0.52         # door-local X; near the free (left) edge
HANDLE_STANDOFF_R = 0.008      # standoff post radius
HANDLE_STANDOFF_LEN = 0.035    # standoff length (door face to tube center)
HANDLE_STANDOFF_ZS = (-0.35, 0.0, 0.35)  # standoff Z positions as fraction of tube half-length

DOOR_OPEN_MAX = math.radians(120.0)

# ---- Hinge hardware (brackets on the cabinet, pins on the hinge axis) ----
HINGE_PLATE_W = 0.04      # bracket plate extent in X (kept inside cabinet width)
HINGE_PLATE_CX = W / 2.0 - HINGE_PLATE_W / 2.0
HINGE_PLATE_TH = 0.012    # bracket plate thickness in Z
HINGE_PLATE_Y0 = BODY_FRONT_Y - 0.02            # embeds into the carcass front
HINGE_PLATE_Y1 = DOOR_CY + DOOR_TH / 2.0 + 0.006  # reaches past the door front
HINGE_PIN_R = 0.008


def _door_slab_mesh(width: float, height: float, name: str):
    """Door slab with a slightly rounded front face (filleted front border)."""
    slab = (
        cq.Workplane("XY")
        .box(width, DOOR_TH, height)
        .edges(">Y")
        .fillet(0.015)
    )
    return mesh_from_cadquery(slab, name)


def _build_door(
    model: ArticulatedObject,
    *,
    name: str,
    height: float,
    hinge_z: float,
    black_gloss,
    aluminum,
    cabinet,
) -> None:
    """One full-width door. Local frame: hinge line at x=0, panel along -X,
    slab centered on local z=0 and y=0 (closed pose)."""
    door = model.part(name)

    door.visual(
        _door_slab_mesh(DOOR_W, height, f"{name}_slab"),
        origin=Origin(xyz=(-DOOR_W / 2.0, 0.0, 0.0)),
        material=black_gloss,
        name=f"{name}_slab",
    )

    # Tall tubular pro-style handle: vertical tube on horizontal standoffs.
    tube_len = height * HANDLE_TUBE_FRAC
    tube_y = DOOR_TH / 2.0 + HANDLE_STANDOFF_LEN  # tube axis in front of door face
    door.visual(
        Cylinder(radius=HANDLE_TUBE_R, length=tube_len),
        origin=Origin(xyz=(HANDLE_TUBE_CX, tube_y, 0.0)),
        material=aluminum,
        name=f"{name}_handle_tube",
    )
    # Three standoffs projecting horizontally from the door face to the tube.
    for i, frac in enumerate(HANDLE_STANDOFF_ZS):
        standoff_z = frac * tube_len / 2.0
        standoff_cy = DOOR_TH / 2.0 + HANDLE_STANDOFF_LEN / 2.0
        door.visual(
            Cylinder(radius=HANDLE_STANDOFF_R, length=HANDLE_STANDOFF_LEN),
            origin=Origin(
                xyz=(HANDLE_TUBE_CX, standoff_cy, standoff_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=aluminum,
            name=f"{name}_handle_standoff_{i}",
        )

    model.articulation(
        f"{name}_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(HINGE_X, DOOR_CY, hinge_z)),
        # Panel extends along local -X from the hinge; -Z makes positive q
        # swing the free edge outward toward +Y (away from the cabinet).
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=40.0, velocity=1.5, lower=0.0, upper=DOOR_OPEN_MAX
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="top_freezer_refrigerator")

    black_gloss = model.material("gloss_black", rgba=(0.06, 0.06, 0.07, 1.0))
    body_black = model.material("body_black", rgba=(0.09, 0.09, 0.10, 1.0))
    kick_gray = model.material("kick_dark_gray", rgba=(0.24, 0.24, 0.26, 1.0))
    liner_white = model.material("liner_white", rgba=(0.93, 0.93, 0.94, 1.0))
    shelf_glass = model.material("shelf_glass", rgba=(0.82, 0.88, 0.92, 0.55))
    aluminum = model.material("brushed_aluminum", rgba=(0.78, 0.79, 0.81, 1.0))
    badge_silver = model.material("badge_silver", rgba=(0.86, 0.87, 0.88, 1.0))

    body_d = BODY_FRONT_Y - BODY_BACK_Y      # 0.62
    body_h = H - BODY_BOTTOM_Z               # 1.70
    body_cy = (BODY_FRONT_Y + BODY_BACK_Y) / 2.0
    body_cz = BODY_BOTTOM_Z + body_h / 2.0

    # ---- Hollow carcass (root) ----
    cabinet = model.part("cabinet")
    # side walls
    for tag, sx in (("a", -1.0), ("b", 1.0)):
        cabinet.visual(
            Box((WALL, body_d, body_h)),
            origin=Origin(xyz=(sx * (W / 2.0 - WALL / 2.0), body_cy, body_cz)),
            material=body_black,
            name=f"side_wall_{tag}",
        )
    # back wall
    cabinet.visual(
        Box((W, WALL, body_h)),
        origin=Origin(xyz=(0.0, BODY_BACK_Y + WALL / 2.0, body_cz)),
        material=body_black,
        name="back_wall",
    )
    # top wall
    cabinet.visual(
        Box((W, body_d, WALL)),
        origin=Origin(xyz=(0.0, body_cy, H - WALL / 2.0)),
        material=body_black,
        name="top_wall",
    )
    # bottom wall
    cabinet.visual(
        Box((W, body_d, WALL)),
        origin=Origin(xyz=(0.0, body_cy, BODY_BOTTOM_Z + WALL / 2.0)),
        material=body_black,
        name="bottom_wall",
    )
    # horizontal mullion dividing freezer (top) from fresh-food compartment
    cabinet.visual(
        Box((W, body_d, MULLION_TH)),
        origin=Origin(xyz=(0.0, body_cy, MULLION_CZ)),
        material=body_black,
        name="mullion",
    )

    # interior back liner (visible when doors open)
    inner_w = W - 2 * WALL
    cabinet.visual(
        Box((inner_w, 0.006, body_h - 2 * WALL)),
        origin=Origin(xyz=(0.0, BODY_BACK_Y + WALL + 0.003, body_cz)),
        material=liner_white,
        name="back_liner",
    )

    # glass shelves in the fresh-food compartment (embed 2 mm into side walls)
    shelf_w = inner_w + 0.004
    shelf_d = body_d - WALL - 0.05
    shelf_cy = BODY_BACK_Y + WALL + shelf_d / 2.0
    for tag, sz in (("lower", 0.45), ("upper", 0.82)):
        cabinet.visual(
            Box((shelf_w, shelf_d, 0.008)),
            origin=Origin(xyz=(0.0, shelf_cy, sz)),
            material=shelf_glass,
            name=f"fridge_shelf_{tag}",
        )
    # wire shelf in the freezer compartment
    cabinet.visual(
        Box((shelf_w, shelf_d, 0.008)),
        origin=Origin(xyz=(0.0, shelf_cy, 1.46)),
        material=liner_white,
        name="freezer_shelf",
    )

    # ---- Hinge brackets + pins at the right front edge ----
    plate_d = HINGE_PLATE_Y1 - HINGE_PLATE_Y0
    plate_cy = (HINGE_PLATE_Y0 + HINGE_PLATE_Y1) / 2.0

    def _hinge_bracket(tag: str, plate_cz: float, pin_z0: float, pin_z1: float):
        # Plate overhangs the door edge; it embeds into the carcass front so it
        # reads as bolted to the cabinet (within-part overlap is intentional).
        cabinet.visual(
            Box((HINGE_PLATE_W, plate_d, HINGE_PLATE_TH)),
            origin=Origin(xyz=(HINGE_PLATE_CX, plate_cy, plate_cz)),
            material=body_black,
            name=f"hinge_bracket_{tag}",
        )
        # Steel pin on the hinge axis; it enters the door slab's edge bore.
        cabinet.visual(
            Cylinder(radius=HINGE_PIN_R, length=pin_z1 - pin_z0),
            origin=Origin(xyz=(HINGE_X, DOOR_CY, (pin_z0 + pin_z1) / 2.0)),
            material=aluminum,
            name=f"hinge_pin_{tag}",
        )

    # top bracket: above the freezer door, pin points down into the door top
    top_plate_cz = FREEZER_DOOR_Z1 + 0.002 + HINGE_PLATE_TH / 2.0
    _hinge_bracket(
        "top",
        top_plate_cz,
        FREEZER_DOOR_Z1 - 0.015,
        top_plate_cz + HINGE_PLATE_TH / 2.0,
    )
    # middle bracket: in the mullion gap; one through-pin engages the fridge
    # door top bore and the freezer door bottom bore
    mid_plate_cz = (FRIDGE_DOOR_Z1 + FREEZER_DOOR_Z0) / 2.0
    _hinge_bracket(
        "mid",
        mid_plate_cz,
        FRIDGE_DOOR_Z1 - 0.012,
        FREEZER_DOOR_Z0 + 0.015,
    )
    # bottom bracket: under the fridge door, pin points up into the door bottom
    bot_plate_cz = FRIDGE_DOOR_Z0 - 0.002 - HINGE_PLATE_TH / 2.0
    _hinge_bracket(
        "bottom",
        bot_plate_cz,
        bot_plate_cz - HINGE_PLATE_TH / 2.0 + 0.002,
        FRIDGE_DOOR_Z0 + 0.025,
    )

    # recessed dark-gray plinth / kick panel under the carcass
    plinth_front_y = BODY_FRONT_Y - 0.04
    plinth_d = plinth_front_y - (BODY_BACK_Y + 0.02)
    cabinet.visual(
        Box((W - 0.05, plinth_d, BODY_BOTTOM_Z)),
        origin=Origin(
            xyz=(0.0, plinth_front_y - plinth_d / 2.0, BODY_BOTTOM_Z / 2.0)
        ),
        material=kick_gray,
        name="kick_panel",
    )

    # ---- Doors ----
    _build_door(
        model,
        name="freezer_door",
        height=FREEZER_DOOR_H,
        hinge_z=FREEZER_DOOR_CZ,
        black_gloss=black_gloss,
        aluminum=aluminum,
        cabinet=cabinet,
    )
    _build_door(
        model,
        name="fridge_door",
        height=FRIDGE_DOOR_H,
        hinge_z=FRIDGE_DOOR_CZ,
        black_gloss=black_gloss,
        aluminum=aluminum,
        cabinet=cabinet,
    )

    # small silver brand badge on the upper freezer door (half-embedded seat)
    freezer_door = model.get_part("freezer_door")
    freezer_door.visual(
        Box((0.05, 0.003, 0.018)),
        origin=Origin(
            xyz=(-0.36, DOOR_TH / 2.0, FREEZER_DOOR_H / 2.0 - 0.10)
        ),
        material=badge_silver,
        name="brand_badge",
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    freezer_door = object_model.get_part("freezer_door")
    fridge_door = object_model.get_part("fridge_door")
    freezer_hinge = object_model.get_articulation("freezer_door_hinge")
    fridge_hinge = object_model.get_articulation("fridge_door_hinge")

    # ---- Hinge pins are intentionally captured in the door edge bores ----
    for door, slab_elem, pin_elems in (
        (freezer_door, "freezer_door_slab", ("hinge_pin_top", "hinge_pin_mid")),
        (fridge_door, "fridge_door_slab", ("hinge_pin_mid", "hinge_pin_bottom")),
    ):
        for pin_elem in pin_elems:
            ctx.allow_overlap(
                cabinet,
                door,
                elem_a=pin_elem,
                elem_b=slab_elem,
                reason=(
                    "Hinge pin on the door pivot axis is intentionally seated "
                    "inside the door slab's hinge bore (captured pin)."
                ),
            )
            ctx.expect_contact(
                cabinet,
                door,
                elem_a=pin_elem,
                elem_b=slab_elem,
                name=f"{pin_elem} engages {slab_elem}",
            )

    # ---- Doors sit slightly proud of the carcass with a thin gap ----
    for door, label in ((freezer_door, "freezer"), (fridge_door, "fridge")):
        ctx.expect_gap(
            door,
            cabinet,
            axis="y",
            positive_elem=f"{label}_door_slab",
            negative_elem="side_wall_b",
            min_gap=0.001,
            max_gap=0.02,
            name=f"{label} door slab rides just in front of the carcass face",
        )
        ctx.expect_within(
            door,
            cabinet,
            axes="x",
            margin=0.002,
            name=f"{label} door stays within the cabinet width",
        )

    # ---- Freezer door above the fridge door, thin mullion gap between ----
    ctx.expect_gap(
        freezer_door,
        fridge_door,
        axis="z",
        min_gap=0.01,
        max_gap=0.06,
        name="freezer door sits above the fridge door with a thin split gap",
    )
    fz_aabb = ctx.part_world_aabb(freezer_door)
    fr_aabb = ctx.part_world_aabb(fridge_door)
    ctx.check(
        "freezer compartment occupies roughly the top 30% of the height",
        fz_aabb is not None
        and fr_aabb is not None
        and 0.22 * H < (fz_aabb[1][2] - fz_aabb[0][2]) < 0.36 * H
        and fz_aabb[0][2] > 0.6 * H,
        details=f"freezer_aabb={fz_aabb}",
    )

    # mullion divider is present behind the door split gap
    mull = ctx.part_element_world_aabb(cabinet, elem="mullion")
    ctx.check(
        "mullion divider spans the gap between the two doors",
        mull is not None
        and fr_aabb is not None
        and fz_aabb is not None
        and mull[0][2] < fz_aabb[0][2]
        and mull[1][2] > fr_aabb[1][2] - 0.005,
        details=f"mullion={mull}",
    )

    # ---- Handles: tall vertical tubular bars on standoffs, proud of door faces ----
    for door, label in (
        (freezer_door, "freezer"),
        (fridge_door, "fridge"),
    ):
        slab = ctx.part_element_world_aabb(door, elem=f"{label}_door_slab")
        tube = ctx.part_element_world_aabb(door, elem=f"{label}_door_handle_tube")
        ctx.check(
            f"{label} door handle tube is vertical and runs most of the door height",
            slab is not None
            and tube is not None
            and tube[0][1] > slab[1][1] - HANDLE_TUBE_R - 0.002
            and (tube[1][2] - tube[0][2]) > 0.6 * (slab[1][2] - slab[0][2])
            and (tube[1][0] - tube[0][0]) < 0.05,
            details=f"slab={slab}, tube={tube}",
        )
        # Standoffs project horizontally from the door face to the tube.
        standoff = ctx.part_element_world_aabb(door, elem=f"{label}_door_handle_standoff_1")
        ctx.check(
            f"{label} door handle standoff connects the tube to the door face",
            slab is not None
            and standoff is not None
            and tube is not None
            and standoff[0][1] >= slab[1][1] - 0.005
            and standoff[1][1] <= tube[1][1] + HANDLE_TUBE_R + 0.005,
            details=f"slab={slab}, standoff={standoff}, tube={tube}",
        )

    # ---- Brand badge on the upper freezer door ----
    badge = ctx.part_element_world_aabb(freezer_door, elem="brand_badge")
    fz_slab = ctx.part_element_world_aabb(freezer_door, elem="freezer_door_slab")
    ctx.check(
        "silver badge sits on the upper half of the freezer door front",
        badge is not None
        and fz_slab is not None
        and (badge[0][2] + badge[1][2]) / 2.0
        > (fz_slab[0][2] + fz_slab[1][2]) / 2.0
        and badge[1][1] > fz_slab[1][1] - 0.002,
        details=f"badge={badge}",
    )

    # ---- Recessed kick panel at the bottom front ----
    kick = ctx.part_element_world_aabb(cabinet, elem="kick_panel")
    ctx.check(
        "kick panel is short, at the floor, and recessed behind the door plane",
        kick is not None
        and fr_aabb is not None
        and kick[1][2] <= FRIDGE_DOOR_Z0 + 0.001
        and kick[0][2] < 0.005
        and kick[1][1] < fr_aabb[0][1] - 0.02,
        details=f"kick={kick}",
    )

    # ---- Hinges: vertical axis line at the right front edge ----
    for hinge, label in ((freezer_hinge, "freezer"), (fridge_hinge, "fridge")):
        ox, oy, _ = hinge.origin.xyz
        limits = hinge.motion_limits
        ctx.check(
            f"{label} hinge is on the right front edge with a ~120 deg range",
            ox > W / 2.0 - 0.02
            and oy > BODY_FRONT_Y
            and limits is not None
            and limits.lower == 0.0
            and math.radians(110.0) < limits.upper < math.radians(130.0),
            details=f"origin=({ox}, {oy}), limits={limits}",
        )

    # ---- Each door opens outward independently ----
    closed_fz = ctx.part_world_aabb(freezer_door)
    closed_fr = ctx.part_world_aabb(fridge_door)
    fridge_rest_pos = ctx.part_world_position(fridge_door)
    with ctx.pose({freezer_hinge: DOOR_OPEN_MAX}):
        open_fz = ctx.part_world_aabb(freezer_door)
        fridge_pos_during = ctx.part_world_position(fridge_door)
    ctx.check(
        "freezer door free edge swings outward when opened",
        closed_fz is not None
        and open_fz is not None
        and open_fz[1][1] > closed_fz[1][1] + 0.15,
        details=f"closed_max_y={closed_fz[1][1]}, open_max_y={open_fz[1][1]}",
    )
    ctx.check(
        "fridge door is independent of the freezer door",
        fridge_rest_pos is not None
        and fridge_pos_during is not None
        and all(
            abs(a - b) < 1e-9
            for a, b in zip(fridge_rest_pos, fridge_pos_during)
        ),
        details=f"rest={fridge_rest_pos}, during={fridge_pos_during}",
    )
    with ctx.pose({fridge_hinge: DOOR_OPEN_MAX}):
        open_fr = ctx.part_world_aabb(fridge_door)
    ctx.check(
        "fridge door free edge swings outward when opened",
        closed_fr is not None
        and open_fr is not None
        and open_fr[1][1] > closed_fr[1][1] + 0.15,
        details=f"closed_max_y={closed_fr[1][1]}, open_max_y={open_fr[1][1]}",
    )

    # ---- Hollow interior: shelves sit inside the wall-to-wall cavity ----
    shelf = ctx.part_element_world_aabb(cabinet, elem="fridge_shelf_lower")
    ctx.check(
        "fresh-food shelf spans the hollow interior between the side walls",
        shelf is not None
        and shelf[0][0] > -W / 2.0 + WALL - 0.005
        and shelf[1][0] < W / 2.0 - WALL + 0.005
        and shelf[0][2] > BODY_BOTTOM_Z + WALL
        and shelf[1][2] < MULLION_CZ - MULLION_TH / 2.0,
        details=f"shelf={shelf}",
    )

    return ctx.report()


object_model = build_object_model()
