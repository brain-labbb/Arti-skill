from __future__ import annotations

# Free-standing single-door refrigerator, glossy black.
# Variant: single full-height door covering the whole front (right-hinged).
# Frame:
#   - X: width (0.65 m), cabinet centered on x=0; the hinge edge is at +X (right).
#   - Y: depth (~0.68 m incl. doors); the doors face the front at +Y.
#   - Z: height; floor at z=0, top of the cabinet at z=1.75.
# Structure:
#   - cabinet (root): hollow insulated carcass (side/back/top/bottom walls),
#     a horizontal mullion divider (internal, visible when door opens),
#     interior shelves, and a recessed dark-gray plinth/kick panel at the bottom front.
#   - door: single full-width slab with a slightly rounded front face
#     (CadQuery fillet), carrying a horizontal brushed aluminum bar handle
#     near the vertical center; a small silver brand badge sits on the upper
#     portion of the door.
# Articulation:
#   - door_hinge: REVOLUTE joint on the vertical axis line at the right front
#     edge of the cabinet, 0 .. ~120 deg. Door panel extends along local -X
#     from the hinge, so axis (0, 0, -1) makes positive q swing the free edge
#     outward (+Y).

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

MULLION_CZ = 1.21         # compartment split height (internal structure)
MULLION_TH = 0.04

# Single full-height door spans both compartments
SINGLE_DOOR_Z0 = 0.06
SINGLE_DOOR_Z1 = 1.73
SINGLE_DOOR_H = SINGLE_DOOR_Z1 - SINGLE_DOOR_Z0      # 1.67
SINGLE_DOOR_CZ = (SINGLE_DOOR_Z0 + SINGLE_DOOR_Z1) / 2.0  # 0.895

HINGE_X = W / 2.0 - 0.005  # vertical hinge line at the right front edge

# ---- Handles ----
HANDLE_BAR_R = 0.011
HANDLE_BAR_LEN = 0.46
HANDLE_BAR_CX = -0.36      # door-local; biased toward the free (left) edge
HANDLE_STANDOFF_DX = 0.19  # standoff offset from bar center
HANDLE_STANDOFF_R = 0.009
HANDLE_PROUD = 0.030       # bar axis distance in front of the door face

# Handle placed near the vertical center, slightly above for ergonomics (~1.0m)
HANDLE_LOCAL_Z = 0.10

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
    handle_local_z: float,
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

    # Horizontal brushed-aluminum bar handle on two standoffs.
    bar_y = DOOR_TH / 2.0 + HANDLE_PROUD
    door.visual(
        Cylinder(radius=HANDLE_BAR_R, length=HANDLE_BAR_LEN),
        origin=Origin(xyz=(HANDLE_BAR_CX, bar_y, handle_local_z),
                      rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name=f"{name}_handle_bar",
    )
    for tag, sx in (("a", -HANDLE_STANDOFF_DX), ("b", HANDLE_STANDOFF_DX)):
        # Standoff post along Y: embeds 4 mm into the slab and reaches the bar.
        door.visual(
            Cylinder(radius=HANDLE_STANDOFF_R, length=0.034),
            origin=Origin(
                xyz=(HANDLE_BAR_CX + sx, DOOR_TH / 2.0 + 0.013, handle_local_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=aluminum,
            name=f"{name}_handle_standoff_{tag}",
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
    model = ArticulatedObject(name="free_standing_refrigerator")

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
    # horizontal mullion (internal divider, visible when door opens)
    cabinet.visual(
        Box((W, body_d, MULLION_TH)),
        origin=Origin(xyz=(0.0, body_cy, MULLION_CZ)),
        material=body_black,
        name="mullion",
    )

    # interior back liner (visible when door opens)
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
    for i, sz in enumerate((0.45, 0.82)):
        cabinet.visual(
            Box((shelf_w, shelf_d, 0.008)),
            origin=Origin(xyz=(0.0, shelf_cy, sz)),
            material=shelf_glass,
            name=f"fridge_shelf_{i}",
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

    # top bracket: above the door, pin points down into the door top edge
    top_plate_cz = SINGLE_DOOR_Z1 + 0.002 + HINGE_PLATE_TH / 2.0
    _hinge_bracket(
        "top",
        top_plate_cz,
        SINGLE_DOOR_Z1 - 0.015,
        top_plate_cz + HINGE_PLATE_TH / 2.0,
    )
    # middle bracket: at the mullion height, pin engages the door mid-edge
    mid_plate_cz = MULLION_CZ
    _hinge_bracket(
        "mid",
        mid_plate_cz,
        mid_plate_cz - 0.015,
        mid_plate_cz + 0.015,
    )
    # bottom bracket: under the door, pin points up into the door bottom edge
    bot_plate_cz = SINGLE_DOOR_Z0 - 0.002 - HINGE_PLATE_TH / 2.0
    _hinge_bracket(
        "bottom",
        bot_plate_cz,
        bot_plate_cz - HINGE_PLATE_TH / 2.0 + 0.002,
        SINGLE_DOOR_Z0 + 0.025,
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

    # ---- Single full-height door ----
    _build_door(
        model,
        name="door",
        height=SINGLE_DOOR_H,
        hinge_z=SINGLE_DOOR_CZ,
        handle_local_z=HANDLE_LOCAL_Z,
        black_gloss=black_gloss,
        aluminum=aluminum,
        cabinet=cabinet,
    )

    # small silver brand badge on the upper portion of the door (half-embedded seat)
    door_part = model.get_part("door")
    door_part.visual(
        Box((0.05, 0.003, 0.018)),
        origin=Origin(
            xyz=(-0.36, DOOR_TH / 2.0, SINGLE_DOOR_H / 2.0 - 0.10)
        ),
        material=badge_silver,
        name="brand_badge",
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("door_hinge")

    # ---- Hinge pins are intentionally captured in the door edge bores ----
    for pin_elem in ("hinge_pin_top", "hinge_pin_mid", "hinge_pin_bottom"):
        ctx.allow_overlap(
            cabinet,
            door,
            elem_a=pin_elem,
            elem_b="door_slab",
            reason=(
                "Hinge pin on the door pivot axis is intentionally seated "
                "inside the door slab's hinge bore (captured pin)."
            ),
        )
        ctx.expect_contact(
            cabinet,
            door,
            elem_a=pin_elem,
            elem_b="door_slab",
            name=f"{pin_elem} engages door_slab",
        )

    # ---- Middle hinge bracket embeds into the door edge at the mullion height ----
    ctx.allow_overlap(
        cabinet,
        door,
        elem_a="hinge_bracket_mid",
        elem_b="door_slab",
        reason=(
            "The middle hinge bracket plate at the mullion height intentionally "
            "embeds into the single door slab's hinge edge to mount the hinge pin."
        ),
    )
    ctx.expect_contact(
        cabinet,
        door,
        elem_a="hinge_bracket_mid",
        elem_b="door_slab",
        name="hinge_bracket_mid seats against door_slab hinge edge",
    )

    # ---- Door sits slightly proud of the carcass with a thin gap ----
    ctx.expect_gap(
        door,
        cabinet,
        axis="y",
        positive_elem="door_slab",
        negative_elem="side_wall_b",
        min_gap=0.001,
        max_gap=0.02,
        name="door slab rides just in front of the carcass face",
    )
    ctx.expect_within(
        door,
        cabinet,
        axes="x",
        margin=0.002,
        name="door stays within the cabinet width",
    )

    # ---- Single door covers the full front height ----
    door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "single door covers the full cabinet front from bottom to top",
        door_aabb is not None
        and door_aabb[0][2] < 0.10
        and door_aabb[1][2] > H - 0.10
        and (door_aabb[1][2] - door_aabb[0][2]) > 0.85 * H,
        details=f"door_aabb={door_aabb}",
    )

    # ---- Internal mullion still present (visible when door opens) ----
    mull = ctx.part_element_world_aabb(cabinet, elem="mullion")
    ctx.check(
        "mullion divider remains inside the cabinet carcass",
        mull is not None
        and mull[0][2] > 0.5 * H
        and mull[1][2] < 0.8 * H,
        details=f"mullion={mull}",
    )

    # ---- Handle: horizontal aluminum bar proud of the door face ----
    slab = ctx.part_element_world_aabb(door, elem="door_slab")
    bar = ctx.part_element_world_aabb(door, elem="door_handle_bar")
    ctx.check(
        "door handle bar is horizontal and proud of the door face",
        slab is not None
        and bar is not None
        and bar[0][1] > slab[1][1] - HANDLE_BAR_R - 0.002
        and (bar[1][0] - bar[0][0]) > 0.4
        and (bar[1][2] - bar[0][2]) < 0.03,
        details=f"slab={slab}, bar={bar}",
    )
    # Handle near the vertical center for ergonomics
    door_cz = (slab[0][2] + slab[1][2]) / 2.0 if slab is not None else 0.0
    bar_cz = (bar[0][2] + bar[1][2]) / 2.0 if bar is not None else 0.0
    ctx.check(
        "door handle sits near the vertical center of the door",
        bar is not None
        and abs(bar_cz - door_cz) < 0.25 * SINGLE_DOOR_H,
        details=f"bar_cz={bar_cz}, door_cz={door_cz}",
    )

    # ---- Brand badge on the upper portion of the door ----
    badge = ctx.part_element_world_aabb(door, elem="brand_badge")
    door_slab_aabb = ctx.part_element_world_aabb(door, elem="door_slab")
    ctx.check(
        "silver badge sits on the upper half of the door front",
        badge is not None
        and door_slab_aabb is not None
        and (badge[0][2] + badge[1][2]) / 2.0
        > (door_slab_aabb[0][2] + door_slab_aabb[1][2]) / 2.0
        and badge[1][1] > door_slab_aabb[1][1] - 0.002,
        details=f"badge={badge}",
    )

    # ---- Recessed kick panel at the bottom front ----
    kick = ctx.part_element_world_aabb(cabinet, elem="kick_panel")
    ctx.check(
        "kick panel is short, at the floor, and recessed behind the door plane",
        kick is not None
        and door_aabb is not None
        and kick[1][2] <= SINGLE_DOOR_Z0 + 0.001
        and kick[0][2] < 0.005
        and kick[1][1] < door_aabb[0][1] - 0.02,
        details=f"kick={kick}",
    )

    # ---- Hinge: vertical axis line at the right front edge ----
    ox, oy, _ = door_hinge.origin.xyz
    limits = door_hinge.motion_limits
    ctx.check(
        "hinge is on the right front edge with a ~120 deg range",
        ox > W / 2.0 - 0.02
        and oy > BODY_FRONT_Y
        and limits is not None
        and limits.lower == 0.0
        and math.radians(110.0) < limits.upper < math.radians(130.0),
        details=f"origin=({ox}, {oy}), limits={limits}",
    )

    # ---- Door opens outward ----
    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: DOOR_OPEN_MAX}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door free edge swings outward when opened",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.15,
        details=f"closed_max_y={closed_aabb[1][1]}, open_max_y={open_aabb[1][1]}",
    )

    # ---- Hollow interior: shelves sit inside the wall-to-wall cavity ----
    shelf = ctx.part_element_world_aabb(cabinet, elem="fridge_shelf_0")
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
