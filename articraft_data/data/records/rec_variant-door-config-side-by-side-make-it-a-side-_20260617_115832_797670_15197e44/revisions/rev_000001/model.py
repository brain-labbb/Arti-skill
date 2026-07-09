from __future__ import annotations

# Side-by-side refrigerator variant (door_config = side_by_side).
# Two full-height doors split vertically down the middle:
#   door_0 (left):  hinged on the left  cabinet edge, axis (0,0,+1)
#   door_1 (right): hinged on the right cabinet edge, axis (0,0,-1)
# Both swing outward for positive q.
# Cabinet carcass, interior shelves, plinth, and materials are identical to
# the top-freezer parent; only the door layer changes.
#
# Frame:
#   X: width  (0.65 m), cabinet centered on x=0.
#   Y: depth  (~0.68 m incl. doors); doors face +Y.
#   Z: height; floor at z=0, top at z=1.75.

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
W = 0.65
H = 1.75
BODY_BACK_Y = -0.34
BODY_FRONT_Y = 0.28
WALL = 0.04
BODY_BOTTOM_Z = 0.05

# ---- Doors (side-by-side: two half-width full-height slabs) ----
DOOR_TH = 0.055
DOOR_CENTER_GAP = 0.008          # visible gap at the vertical center split
HINGE_SPAN = W - 0.010           # distance between the two hinge axis lines
SIDE_DOOR_W = (HINGE_SPAN - DOOR_CENTER_GAP) / 2.0   # ≈ 0.316 each
DOOR_GAP_Y = 0.004               # gap between carcass front and door back
DOOR_CY = BODY_FRONT_Y + DOOR_GAP_Y + DOOR_TH / 2.0

DOOR_Z0 = 0.06
DOOR_Z1 = 1.73
DOOR_H = DOOR_Z1 - DOOR_Z0       # 1.67
DOOR_CZ = (DOOR_Z0 + DOOR_Z1) / 2.0

# Hinge axis X positions at the outer cabinet edges
LEFT_HINGE_X = -(W / 2.0 - 0.005)
RIGHT_HINGE_X = W / 2.0 - 0.005

# ---- Handles (shorter bars for half-width doors) ----
HANDLE_BAR_R = 0.011
HANDLE_BAR_LEN = 0.22
HANDLE_STANDOFF_DX = 0.08
HANDLE_STANDOFF_R = 0.009
HANDLE_PROUD = 0.030

DOOR_OPEN_MAX = math.radians(120.0)

# ---- Hinge hardware ----
HINGE_PLATE_W = 0.04
HINGE_PLATE_TH = 0.012
HINGE_PLATE_Y0 = BODY_FRONT_Y - 0.02
HINGE_PLATE_Y1 = DOOR_CY + DOOR_TH / 2.0 + 0.006
HINGE_PIN_R = 0.008

# Parent carcass constants (kept identical)
MULLION_CZ = 1.21
MULLION_TH = 0.04


# ---------------------------------------------------------------------------
# Shared geometry helpers
# ---------------------------------------------------------------------------

def _door_slab_mesh(width: float, height: float, name: str):
    """Door slab with a slightly rounded front face (filleted front border)."""
    slab = (
        cq.Workplane("XY")
        .box(width, DOOR_TH, height)
        .edges(">Y")
        .fillet(0.015)
    )
    return mesh_from_cadquery(slab, name)


def _build_side_door(
    model: ArticulatedObject,
    *,
    name: str,
    hinge_x: float,
    extends_positive_x: bool,
    handle_local_x: float,
    black_gloss,
    aluminum,
    cabinet,
) -> None:
    """One full-height half-width door.

    Part frame sits at the hinge line (hinge_x, DOOR_CY, DOOR_CZ).
    The slab extends along local ±X from the hinge.
    """
    door = model.part(name)

    # Slab offset from the hinge-line frame
    slab_x = SIDE_DOOR_W / 2.0 if extends_positive_x else -SIDE_DOOR_W / 2.0
    door.visual(
        _door_slab_mesh(SIDE_DOOR_W, DOOR_H, f"{name}_slab"),
        origin=Origin(xyz=(slab_x, 0.0, 0.0)),
        material=black_gloss,
        name=f"{name}_slab",
    )

    # Horizontal brushed-aluminum bar handle on two standoffs
    bar_y = DOOR_TH / 2.0 + HANDLE_PROUD
    door.visual(
        Cylinder(radius=HANDLE_BAR_R, length=HANDLE_BAR_LEN),
        origin=Origin(
            xyz=(handle_local_x, bar_y, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=aluminum,
        name=f"{name}_handle_bar",
    )
    for tag, sx in (("a", -HANDLE_STANDOFF_DX), ("b", HANDLE_STANDOFF_DX)):
        door.visual(
            Cylinder(radius=HANDLE_STANDOFF_R, length=0.034),
            origin=Origin(
                xyz=(handle_local_x + sx, DOOR_TH / 2.0 + 0.013, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=aluminum,
            name=f"{name}_handle_standoff_{tag}",
        )

    # Joint axis: chosen so positive q swings the free edge toward +Y (outward)
    axis = (0.0, 0.0, 1.0) if extends_positive_x else (0.0, 0.0, -1.0)

    model.articulation(
        f"{name}_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(hinge_x, DOOR_CY, DOOR_CZ)),
        axis=axis,
        motion_limits=MotionLimits(
            effort=40.0, velocity=1.5, lower=0.0, upper=DOOR_OPEN_MAX
        ),
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_by_side_refrigerator")

    # ---- Materials ----
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

    # ---- Cabinet carcass (root, identical to parent) ----
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
    # horizontal mullion (internal carcass structure, kept identical)
    cabinet.visual(
        Box((W, body_d, MULLION_TH)),
        origin=Origin(xyz=(0.0, body_cy, MULLION_CZ)),
        material=body_black,
        name="mullion",
    )

    # interior back liner
    inner_w = W - 2 * WALL
    cabinet.visual(
        Box((inner_w, 0.006, body_h - 2 * WALL)),
        origin=Origin(xyz=(0.0, BODY_BACK_Y + WALL + 0.003, body_cz)),
        material=liner_white,
        name="back_liner",
    )

    # glass shelves (identical positions to parent)
    shelf_w = inner_w + 0.004
    shelf_d = body_d - WALL - 0.05
    shelf_cy = BODY_BACK_Y + WALL + shelf_d / 2.0
    for i, sz in enumerate([0.45, 0.82, 1.46]):
        mat = shelf_glass if i < 2 else liner_white
        cabinet.visual(
            Box((shelf_w, shelf_d, 0.008)),
            origin=Origin(xyz=(0.0, shelf_cy, sz)),
            material=mat,
            name=f"shelf_{i}",
        )

    # ---- Hinge brackets + pins at the outer edges ----
    plate_d = HINGE_PLATE_Y1 - HINGE_PLATE_Y0
    plate_cy = (HINGE_PLATE_Y0 + HINGE_PLATE_Y1) / 2.0

    for i, hinge_x in enumerate([LEFT_HINGE_X, RIGHT_HINGE_X]):
        # Plate X center: inset from the cabinet edge, mirroring the parent
        sign = -1.0 if hinge_x < 0 else 1.0
        plate_cx = sign * (W / 2.0 - HINGE_PLATE_W / 2.0)

        # Top bracket + pin
        top_plate_cz = DOOR_Z1 + 0.002 + HINGE_PLATE_TH / 2.0
        cabinet.visual(
            Box((HINGE_PLATE_W, plate_d, HINGE_PLATE_TH)),
            origin=Origin(xyz=(plate_cx, plate_cy, top_plate_cz)),
            material=body_black,
            name=f"hinge_bracket_{i}_top",
        )
        cabinet.visual(
            Cylinder(radius=HINGE_PIN_R, length=0.030),
            origin=Origin(xyz=(hinge_x, DOOR_CY, DOOR_Z1 - 0.005)),
            material=aluminum,
            name=f"hinge_pin_{i}_top",
        )

        # Bottom bracket + pin
        bot_plate_cz = DOOR_Z0 - 0.002 - HINGE_PLATE_TH / 2.0
        cabinet.visual(
            Box((HINGE_PLATE_W, plate_d, HINGE_PLATE_TH)),
            origin=Origin(xyz=(plate_cx, plate_cy, bot_plate_cz)),
            material=body_black,
            name=f"hinge_bracket_{i}_bottom",
        )
        cabinet.visual(
            Cylinder(radius=HINGE_PIN_R, length=0.030),
            origin=Origin(xyz=(hinge_x, DOOR_CY, DOOR_Z0 + 0.005)),
            material=aluminum,
            name=f"hinge_pin_{i}_bottom",
        )

    # recessed dark-gray plinth / kick panel (identical to parent)
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

    # ---- Doors (loop with name_{i} convention) ----
    # door_0: left  – hinge on left edge,  slab extends +X, handle near free right edge
    # door_1: right – hinge on right edge, slab extends -X, handle near free left edge
    door_configs = [
        {
            "name": "door_0",
            "hinge_x": LEFT_HINGE_X,
            "extends_positive_x": True,
            "handle_local_x": SIDE_DOOR_W / 2.0 - 0.04,
        },
        {
            "name": "door_1",
            "hinge_x": RIGHT_HINGE_X,
            "extends_positive_x": False,
            "handle_local_x": -(SIDE_DOOR_W / 2.0 - 0.04),
        },
    ]
    for cfg in door_configs:
        _build_side_door(
            model,
            name=cfg["name"],
            hinge_x=cfg["hinge_x"],
            extends_positive_x=cfg["extends_positive_x"],
            handle_local_x=cfg["handle_local_x"],
            black_gloss=black_gloss,
            aluminum=aluminum,
            cabinet=cabinet,
        )

    # small silver brand badge on door_0 upper area
    door_0 = model.get_part("door_0")
    door_0.visual(
        Box((0.05, 0.003, 0.018)),
        origin=Origin(
            xyz=(SIDE_DOOR_W / 2.0 - 0.06, DOOR_TH / 2.0, DOOR_H / 2.0 - 0.10)
        ),
        material=badge_silver,
        name="brand_badge",
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    hinge_0 = object_model.get_articulation("door_0_hinge")
    hinge_1 = object_model.get_articulation("door_1_hinge")

    # ---- Hinge pins are captured in the door edge bores ----
    for door, slab_elem, pin_elems in (
        (door_0, "door_0_slab", ("hinge_pin_0_top", "hinge_pin_0_bottom")),
        (door_1, "door_1_slab", ("hinge_pin_1_top", "hinge_pin_1_bottom")),
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

    # ---- Doors are split vertically: door_0 on the left, door_1 on the right ----
    aabb_0 = ctx.part_world_aabb(door_0)
    aabb_1 = ctx.part_world_aabb(door_1)
    ctx.check(
        "door_0 is on the left half and door_1 is on the right half",
        aabb_0 is not None
        and aabb_1 is not None
        and aabb_0[1][0] < 0.01          # door_0 max-X near center
        and aabb_1[0][0] > -0.01         # door_1 min-X near center
        and aabb_0[0][0] < -W / 2.0 + 0.02  # door_0 reaches the left edge
        and aabb_1[1][0] > W / 2.0 - 0.02,  # door_1 reaches the right edge
        details=f"door_0_aabb={aabb_0}, door_1_aabb={aabb_1}",
    )

    # ---- Both doors are full height ----
    for door, label in ((door_0, "door_0"), (door_1, "door_1")):
        da = ctx.part_world_aabb(door)
        ctx.check(
            f"{label} is full height (spans most of the cabinet)",
            da is not None
            and (da[1][2] - da[0][2]) > 0.85 * H
            and da[0][2] < DOOR_Z0 + 0.01
            and da[1][2] > DOOR_Z1 - 0.01,
            details=f"aabb={da}",
        )

    # ---- Thin vertical gap between the two doors at center ----
    ctx.expect_gap(
        door_1,
        door_0,
        axis="x",
        min_gap=0.001,
        max_gap=0.03,
        name="thin vertical gap between door_0 and door_1 at center split",
    )

    # ---- Doors sit slightly proud of the carcass ----
    for door, label in ((door_0, "door_0"), (door_1, "door_1")):
        ctx.expect_gap(
            door,
            cabinet,
            axis="y",
            positive_elem=f"{label}_slab",
            negative_elem="back_wall",
            min_gap=0.02,
            name=f"{label} slab sits in front of the carcass",
        )

    # ---- Hinge positions: door_0 on left edge, door_1 on right edge ----
    for hinge, expected_sign, label in (
        (hinge_0, -1.0, "door_0"),
        (hinge_1, +1.0, "door_1"),
    ):
        ox, oy, _ = hinge.origin.xyz
        limits = hinge.motion_limits
        ctx.check(
            f"{label} hinge is on the {'left' if expected_sign < 0 else 'right'} "
            f"edge with ~120 deg range",
            ox * expected_sign > W / 2.0 - 0.02
            and oy > BODY_FRONT_Y - 0.01
            and limits is not None
            and limits.lower == 0.0
            and math.radians(110.0) < limits.upper < math.radians(130.0),
            details=f"origin=({ox}, {oy}), limits={limits}",
        )

    # ---- Each door opens outward independently ----
    closed_0 = ctx.part_world_aabb(door_0)
    closed_1 = ctx.part_world_aabb(door_1)

    # Open door_0 only → door_1 must stay put
    door_1_rest = ctx.part_world_position(door_1)
    with ctx.pose({hinge_0: DOOR_OPEN_MAX}):
        open_0 = ctx.part_world_aabb(door_0)
        door_1_during = ctx.part_world_position(door_1)
    ctx.check(
        "door_0 free edge swings outward when opened",
        closed_0 is not None
        and open_0 is not None
        and open_0[1][1] > closed_0[1][1] + 0.10,
        details=f"closed_max_y={closed_0[1][1] if closed_0 else None}, "
                f"open_max_y={open_0[1][1] if open_0 else None}",
    )
    ctx.check(
        "door_1 is independent of door_0",
        door_1_rest is not None
        and door_1_during is not None
        and all(abs(a - b) < 1e-9 for a, b in zip(door_1_rest, door_1_during)),
        details=f"rest={door_1_rest}, during={door_1_during}",
    )

    # Open door_1 only → confirm outward swing
    door_0_rest = ctx.part_world_position(door_0)
    with ctx.pose({hinge_1: DOOR_OPEN_MAX}):
        open_1 = ctx.part_world_aabb(door_1)
        door_0_during = ctx.part_world_position(door_0)
    ctx.check(
        "door_1 free edge swings outward when opened",
        closed_1 is not None
        and open_1 is not None
        and open_1[1][1] > closed_1[1][1] + 0.10,
        details=f"closed_max_y={closed_1[1][1] if closed_1 else None}, "
                f"open_max_y={open_1[1][1] if open_1 else None}",
    )
    ctx.check(
        "door_0 is independent of door_1",
        door_0_rest is not None
        and door_0_during is not None
        and all(abs(a - b) < 1e-9 for a, b in zip(door_0_rest, door_0_during)),
        details=f"rest={door_0_rest}, during={door_0_during}",
    )

    # ---- Handles: horizontal aluminum bars proud of each door face ----
    for door, label in ((door_0, "door_0"), (door_1, "door_1")):
        slab = ctx.part_element_world_aabb(door, elem=f"{label}_slab")
        bar = ctx.part_element_world_aabb(door, elem=f"{label}_handle_bar")
        ctx.check(
            f"{label} handle bar is horizontal and proud of the door face",
            slab is not None
            and bar is not None
            and bar[0][1] > slab[1][1] - HANDLE_BAR_R - 0.002
            and (bar[1][0] - bar[0][0]) > 0.15
            and (bar[1][2] - bar[0][2]) < 0.03,
            details=f"slab={slab}, bar={bar}",
        )

    # ---- Brand badge on door_0 upper area ----
    badge = ctx.part_element_world_aabb(door_0, elem="brand_badge")
    slab_0 = ctx.part_element_world_aabb(door_0, elem="door_0_slab")
    ctx.check(
        "silver badge sits on the upper half of door_0 front",
        badge is not None
        and slab_0 is not None
        and (badge[0][2] + badge[1][2]) / 2.0
        > (slab_0[0][2] + slab_0[1][2]) / 2.0
        and badge[1][1] > slab_0[1][1] - 0.002,
        details=f"badge={badge}",
    )

    # ---- Kick panel at bottom front (identical to parent) ----
    kick = ctx.part_element_world_aabb(cabinet, elem="kick_panel")
    ctx.check(
        "kick panel is short, at the floor, and recessed behind the door plane",
        kick is not None
        and aabb_0 is not None
        and kick[1][2] <= DOOR_Z0 + 0.001
        and kick[0][2] < 0.005
        and kick[1][1] < aabb_0[0][1] - 0.02,
        details=f"kick={kick}",
    )

    # ---- Interior shelves still present inside the carcass ----
    shelf = ctx.part_element_world_aabb(cabinet, elem="shelf_0")
    ctx.check(
        "interior shelf spans the hollow interior between the side walls",
        shelf is not None
        and shelf[0][0] > -W / 2.0 + WALL - 0.005
        and shelf[1][0] < W / 2.0 - WALL + 0.005
        and shelf[0][2] > BODY_BOTTOM_Z + WALL
        and shelf[1][2] < H - WALL,
        details=f"shelf={shelf}",
    )

    return ctx.report()


object_model = build_object_model()
