from __future__ import annotations

# Free-standing top-freezer refrigerator, glossy black.
# Variant: short_legs base support (4 adjustable legs instead of kick panel).
#
# Frame:
#   - X: width (0.65 m), cabinet centered on x=0; the hinge edge is at +X (right).
#   - Y: depth (~0.68 m incl. doors); the doors face the front at +Y.
#   - Z: height; floor at z=0, top of the cabinet at z=1.75.
# Structure:
#   - cabinet (root): hollow insulated carcass (side/back/top/bottom walls),
#     a horizontal mullion divider splitting a top freezer compartment
#     (~30% of height) from the fresh-food compartment below, interior
#     shelves, and four short adjustable legs at the corners.
#   - door_0 (freezer) / door_1 (fridge): full-width slabs with a slightly
#     rounded front face (CadQuery fillet), each carrying a horizontal brushed
#     aluminum bar handle near the compartment split; a small silver brand
#     badge sits on the upper freezer door.
# Articulation:
#   - door_0_hinge / door_1_hinge: independent REVOLUTE joints on
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

# ---- Adjustable legs ----
LEG_HEIGHT = 0.050        # total leg height (foot + rod visible above floor)
LEG_FOOT_R = 0.018        # foot pad radius
LEG_FOOT_H = 0.012        # foot pad height
LEG_ROD_R = 0.010         # threaded rod radius
LEG_ROD_H = LEG_HEIGHT - LEG_FOOT_H  # visible rod height (0.038)
LEG_EMBED = 0.005         # rod extends into cabinet bottom wall for connectivity
LEG_INSET_X = 0.040       # inset from side edges
LEG_INSET_Y = 0.040       # inset from front/back edges

BODY_BOTTOM_Z = LEG_HEIGHT  # carcass bottom sits on top of legs

# ---- Doors ----
DOOR_TH = 0.055           # door slab thickness
DOOR_W = 0.64             # full-width door slab (0.005 reveal each side)
DOOR_GAP_Y = 0.004        # thin gap between carcass front and door back
DOOR_CY = BODY_FRONT_Y + DOOR_GAP_Y + DOOR_TH / 2.0

MULLION_CZ = 1.21         # compartment split height
MULLION_TH = 0.04

FRIDGE_DOOR_Z0 = 0.06
FRIDGE_DOOR_Z1 = 1.195
FREEZER_DOOR_Z0 = 1.225
FREEZER_DOOR_Z1 = 1.73
FRIDGE_DOOR_H = FRIDGE_DOOR_Z1 - FRIDGE_DOOR_Z0
FREEZER_DOOR_H = FREEZER_DOOR_Z1 - FREEZER_DOOR_Z0
FRIDGE_DOOR_CZ = (FRIDGE_DOOR_Z0 + FRIDGE_DOOR_Z1) / 2.0
FREEZER_DOOR_CZ = (FREEZER_DOOR_Z0 + FREEZER_DOOR_Z1) / 2.0

HINGE_X = W / 2.0 - 0.005

# ---- Handles ----
HANDLE_BAR_R = 0.011
HANDLE_BAR_LEN = 0.46
HANDLE_BAR_CX = -0.36      # door-local; biased toward the free (left) edge
HANDLE_STANDOFF_DX = 0.19
HANDLE_STANDOFF_R = 0.009
HANDLE_PROUD = 0.030       # bar axis distance in front of the door face

DOOR_OPEN_MAX = math.radians(120.0)

# ---- Hinge hardware ----
HINGE_PLATE_W = 0.04
HINGE_PLATE_CX = W / 2.0 - HINGE_PLATE_W / 2.0
HINGE_PLATE_TH = 0.012
HINGE_PLATE_Y0 = BODY_FRONT_Y - 0.02
HINGE_PLATE_Y1 = DOOR_CY + DOOR_TH / 2.0 + 0.006
HINGE_PIN_R = 0.008

NUM_LEGS = 4
NUM_SHELVES = 3
NUM_DOORS = 2


# ---- Shared geometry helpers ----

def _leg_mesh(name: str):
    """Adjustable refrigerator leg: cylindrical rod with a wider foot pad.
    Built in local frame with foot at z=0 and rod extending upward."""
    base = (
        cq.Workplane("XY")
        .circle(LEG_FOOT_R)
        .extrude(LEG_FOOT_H)
    )
    rod = (
        cq.Workplane("XY")
        .workplane(offset=LEG_FOOT_H)
        .circle(LEG_ROD_R)
        .extrude(LEG_ROD_H + LEG_EMBED)
    )
    leg = base.union(rod)
    return mesh_from_cadquery(leg, name)


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
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=40.0, velocity=1.5, lower=0.0, upper=DOOR_OPEN_MAX
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="top_freezer_refrigerator")

    black_gloss = model.material("gloss_black", rgba=(0.06, 0.06, 0.07, 1.0))
    body_black = model.material("body_black", rgba=(0.09, 0.09, 0.10, 1.0))
    leg_gray = model.material("leg_gray", rgba=(0.32, 0.32, 0.34, 1.0))
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

    # ---- Four adjustable legs at the corners (for i in range loop) ----
    leg_positions = [
        (-W / 2.0 + LEG_INSET_X, BODY_FRONT_Y - LEG_INSET_Y),  # leg_0: front-left
        (W / 2.0 - LEG_INSET_X, BODY_FRONT_Y - LEG_INSET_Y),   # leg_1: front-right
        (-W / 2.0 + LEG_INSET_X, BODY_BACK_Y + LEG_INSET_Y),   # leg_2: back-left
        (W / 2.0 - LEG_INSET_X, BODY_BACK_Y + LEG_INSET_Y),    # leg_3: back-right
    ]
    for i in range(NUM_LEGS):
        lx, ly = leg_positions[i]
        cabinet.visual(
            _leg_mesh(f"leg_{i}"),
            origin=Origin(xyz=(lx, ly, 0.0)),
            material=leg_gray,
            name=f"leg_{i}",
        )

    # ---- Interior shelves (for i in range loop with name_{i}) ----
    shelf_w = inner_w + 0.004
    shelf_d = body_d - WALL - 0.05
    shelf_cy = BODY_BACK_Y + WALL + shelf_d / 2.0
    shelf_configs = [
        (0.45, shelf_glass),    # shelf_0: lower fridge shelf
        (0.82, shelf_glass),    # shelf_1: upper fridge shelf
        (1.46, liner_white),    # shelf_2: freezer wire shelf
    ]
    for i in range(NUM_SHELVES):
        sz, mat = shelf_configs[i]
        cabinet.visual(
            Box((shelf_w, shelf_d, 0.008)),
            origin=Origin(xyz=(0.0, shelf_cy, sz)),
            material=mat,
            name=f"shelf_{i}",
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
    # middle bracket: in the mullion gap
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

    # ---- Doors (for i in range loop with name_{i}) ----
    door_configs = [
        {
            "name": "door_0",
            "height": FREEZER_DOOR_H,
            "hinge_z": FREEZER_DOOR_CZ,
            "handle_local_z": -FREEZER_DOOR_H / 2.0 + 0.05,
        },
        {
            "name": "door_1",
            "height": FRIDGE_DOOR_H,
            "hinge_z": FRIDGE_DOOR_CZ,
            "handle_local_z": FRIDGE_DOOR_H / 2.0 - 0.05,
        },
    ]
    for i in range(NUM_DOORS):
        cfg = door_configs[i]
        _build_door(
            model,
            name=cfg["name"],
            height=cfg["height"],
            hinge_z=cfg["hinge_z"],
            handle_local_z=cfg["handle_local_z"],
            black_gloss=black_gloss,
            aluminum=aluminum,
            cabinet=cabinet,
        )

    # small silver brand badge on the upper freezer door (half-embedded seat)
    door_0 = model.get_part("door_0")
    door_0.visual(
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
    door_0 = object_model.get_part("door_0")    # freezer door
    door_1 = object_model.get_part("door_1")    # fridge door
    hinge_0 = object_model.get_articulation("door_0_hinge")
    hinge_1 = object_model.get_articulation("door_1_hinge")

    # ---- Hinge pins are intentionally captured in the door edge bores ----
    for door, slab_elem, pin_elems in (
        (door_0, "door_0_slab", ("hinge_pin_top", "hinge_pin_mid")),
        (door_1, "door_1_slab", ("hinge_pin_mid", "hinge_pin_bottom")),
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
    for door, label in ((door_0, "door_0"), (door_1, "door_1")):
        ctx.expect_gap(
            door,
            cabinet,
            axis="y",
            positive_elem=f"{label}_slab",
            negative_elem="side_wall_b",
            min_gap=0.001,
            max_gap=0.02,
            name=f"{label} slab rides just in front of the carcass face",
        )
        ctx.expect_within(
            door,
            cabinet,
            axes="x",
            margin=0.002,
            name=f"{label} stays within the cabinet width",
        )

    # ---- door_0 (freezer) above door_1 (fridge), thin mullion gap ----
    ctx.expect_gap(
        door_0,
        door_1,
        axis="z",
        min_gap=0.01,
        max_gap=0.06,
        name="freezer door sits above the fridge door with a thin split gap",
    )
    fz_aabb = ctx.part_world_aabb(door_0)
    fr_aabb = ctx.part_world_aabb(door_1)
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

    # ---- Handles: horizontal aluminum bars proud of the door faces ----
    for door, label, near_split in (
        (door_0, "door_0", "lower"),
        (door_1, "door_1", "upper"),
    ):
        slab = ctx.part_element_world_aabb(door, elem=f"{label}_slab")
        bar = ctx.part_element_world_aabb(door, elem=f"{label}_handle_bar")
        ctx.check(
            f"{label} handle bar is horizontal and proud of the door face",
            slab is not None
            and bar is not None
            and bar[0][1] > slab[1][1] - HANDLE_BAR_R - 0.002
            and (bar[1][0] - bar[0][0]) > 0.4
            and (bar[1][2] - bar[0][2]) < 0.03,
            details=f"slab={slab}, bar={bar}",
        )
        door_cz = (slab[0][2] + slab[1][2]) / 2.0 if slab is not None else 0.0
        bar_cz = (bar[0][2] + bar[1][2]) / 2.0 if bar is not None else 0.0
        if near_split == "lower":
            ok = bar is not None and bar_cz < door_cz - 0.1
        else:
            ok = bar is not None and bar_cz > door_cz + 0.1
        ctx.check(
            f"{label} handle sits near the compartment split "
            f"({near_split} edge of the door)",
            ok,
            details=f"bar_cz={bar_cz}, door_cz={door_cz}",
        )

    # ---- Brand badge on the upper freezer door ----
    badge = ctx.part_element_world_aabb(door_0, elem="brand_badge")
    fz_slab = ctx.part_element_world_aabb(door_0, elem="door_0_slab")
    ctx.check(
        "silver badge sits on the upper half of the freezer door front",
        badge is not None
        and fz_slab is not None
        and (badge[0][2] + badge[1][2]) / 2.0
        > (fz_slab[0][2] + fz_slab[1][2]) / 2.0
        and badge[1][1] > fz_slab[1][1] - 0.002,
        details=f"badge={badge}",
    )

    # ---- Four adjustable legs at the corners, lifting cabinet off floor ----
    for i in range(NUM_LEGS):
        leg = ctx.part_element_world_aabb(cabinet, elem=f"leg_{i}")
        ctx.check(
            f"leg_{i} is present at the cabinet base near the floor",
            leg is not None
            and leg[0][2] < 0.005       # foot near floor
            and leg[1][2] > LEG_HEIGHT - 0.010  # rod reaches toward cabinet bottom
            and leg[0][0] > -W / 2.0 - 0.01    # within cabinet footprint + margin
            and leg[1][0] < W / 2.0 + 0.01,
            details=f"leg_{i}_aabb={leg}",
        )

    # Cabinet bottom wall is elevated above the floor by the legs
    bottom_wall = ctx.part_element_world_aabb(cabinet, elem="bottom_wall")
    ctx.check(
        "cabinet bottom wall is elevated above the floor by the legs",
        bottom_wall is not None
        and bottom_wall[0][2] >= LEG_HEIGHT - 0.002,
        details=f"bottom_wall={bottom_wall}",
    )

    # ---- Hinges: vertical axis line at the right front edge ----
    for hinge, label in ((hinge_0, "door_0"), (hinge_1, "door_1")):
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
    closed_fz = ctx.part_world_aabb(door_0)
    closed_fr = ctx.part_world_aabb(door_1)
    fridge_rest_pos = ctx.part_world_position(door_1)
    with ctx.pose({hinge_0: DOOR_OPEN_MAX}):
        open_fz = ctx.part_world_aabb(door_0)
        fridge_pos_during = ctx.part_world_position(door_1)
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
    with ctx.pose({hinge_1: DOOR_OPEN_MAX}):
        open_fr = ctx.part_world_aabb(door_1)
    ctx.check(
        "fridge door free edge swings outward when opened",
        closed_fr is not None
        and open_fr is not None
        and open_fr[1][1] > closed_fr[1][1] + 0.15,
        details=f"closed_max_y={closed_fr[1][1]}, open_max_y={open_fr[1][1]}",
    )

    # ---- Hollow interior: shelves sit inside the wall-to-wall cavity ----
    shelf = ctx.part_element_world_aabb(cabinet, elem="shelf_0")
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
