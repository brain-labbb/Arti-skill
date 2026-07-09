from __future__ import annotations

# French-door bottom-freezer refrigerator variant, glossy black.
# Frame:
#   - X: width (0.65 m), cabinet centered on x=0
#   - Y: depth (~0.68 m incl. doors); doors face front at +Y
#   - Z: height; floor at z=0, top at z=1.75
# Structure:
#   - cabinet (root): hollow insulated carcass (side/back/top/bottom walls),
#     horizontal mullion dividing a bottom freezer compartment (~30%) from the
#     upper fresh-food compartment (~70%), interior shelves, recessed plinth.
#   - french_door_0 (left): half-width slab, REVOLUTE about left outer edge.
#   - french_door_1 (right): half-width slab, REVOLUTE about right outer edge.
#   - freezer_drawer: full-width drawer front + tray, PRISMATIC along +Y.
# Articulation:
#   - french_door_0_hinge / french_door_1_hinge: vertical axis, 0..120 deg.
#   - drawer_slide: PRISMATIC along +Y, 0..0.35 m travel.

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
DOOR_W = 0.64             # full door span (0.005 reveal each side)
DOOR_GAP_Y = 0.004        # thin gap between carcass front and door back
DOOR_CY = BODY_FRONT_Y + DOOR_GAP_Y + DOOR_TH / 2.0  # door slab centerline in Y

MULLION_CZ = 0.55         # compartment split height (freezer below, fresh-food above)
MULLION_TH = 0.04

# French doors (upper fresh-food compartment)
FRENCH_DOOR_Z0 = MULLION_CZ + MULLION_TH / 2.0 + 0.005   # ~0.575
FRENCH_DOOR_Z1 = 1.73
FRENCH_DOOR_H = FRENCH_DOOR_Z1 - FRENCH_DOOR_Z0          # ~1.155
FRENCH_DOOR_CZ = (FRENCH_DOOR_Z0 + FRENCH_DOOR_Z1) / 2.0
HALF_DOOR_W = (DOOR_W - 0.004) / 2.0                     # ~0.318 each (0.004 center gap)

# Freezer drawer (lower compartment)
DRAWER_Z0 = 0.06
DRAWER_Z1 = MULLION_CZ - MULLION_TH / 2.0 - 0.005        # ~0.525
DRAWER_H = DRAWER_Z1 - DRAWER_Z0                          # ~0.465
DRAWER_CZ = (DRAWER_Z0 + DRAWER_Z1) / 2.0

# ---- Handles ----
HANDLE_BAR_R = 0.011
HANDLE_BAR_LEN = 0.50       # vertical bar length for French doors
HANDLE_STANDOFF_SPAN = 0.20 # standoff spacing along bar axis
HANDLE_STANDOFF_R = 0.009
HANDLE_PROUD = 0.030        # bar axis distance in front of the door face

DOOR_OPEN_MAX = math.radians(120.0)
DRAWER_TRAVEL = 0.35        # max drawer extension (meters)

# ---- Hinge hardware (brackets on the cabinet, pins on the hinge axis) ----
HINGE_PLATE_W = 0.04
HINGE_PLATE_TH = 0.012
HINGE_PLATE_Y0 = BODY_FRONT_Y - 0.02
HINGE_PLATE_Y1 = DOOR_CY + DOOR_TH / 2.0 + 0.006
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


def _drawer_front_mesh(width: float, height: float, name: str):
    """Drawer front panel with rounded front face edges."""
    panel = (
        cq.Workplane("XY")
        .box(width, DOOR_TH, height)
        .edges(">Y")
        .fillet(0.012)
    )
    return mesh_from_cadquery(panel, name)


def _build_french_door(
    model: ArticulatedObject,
    *,
    index: int,
    height: float,
    hinge_z: float,
    black_gloss,
    aluminum,
    cabinet,
) -> None:
    """One French door half-panel. index 0=left, 1=right.
    Local frame: hinge line at origin, panel extends toward center (inward)."""
    name = f"french_door_{index}"
    door = model.part(name)

    # Side conventions:
    #   left (i=0):  hinge at -X edge, panel extends along local +X, axis_z=+1
    #   right (i=1): hinge at +X edge, panel extends along local -X, axis_z=-1
    hinge_sign = -1 + 2 * index   # left: -1, right: +1
    panel_sign = 1 - 2 * index    # left: +1, right: -1

    hinge_x = hinge_sign * (W / 2.0 - 0.005)

    # Door slab (panel center offset from hinge along panel_sign * X)
    door.visual(
        _door_slab_mesh(HALF_DOOR_W, height, f"{name}_slab"),
        origin=Origin(xyz=(panel_sign * HALF_DOOR_W / 2.0, 0.0, 0.0)),
        material=black_gloss,
        name=f"{name}_slab",
    )

    # Vertical brushed-aluminum handle bar near the inner (center) edge.
    bar_x = panel_sign * (HALF_DOOR_W - 0.045)
    bar_y = DOOR_TH / 2.0 + HANDLE_PROUD
    door.visual(
        Cylinder(radius=HANDLE_BAR_R, length=HANDLE_BAR_LEN),
        origin=Origin(xyz=(bar_x, bar_y, 0.0)),
        material=aluminum,
        name=f"{name}_handle_bar",
    )
    # Two standoffs connecting the bar to the door face
    for tag, dz in (("a", -HANDLE_STANDOFF_SPAN), ("b", HANDLE_STANDOFF_SPAN)):
        door.visual(
            Cylinder(radius=HANDLE_STANDOFF_R, length=0.034),
            origin=Origin(
                xyz=(bar_x, DOOR_TH / 2.0 + 0.013, dz),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=aluminum,
            name=f"{name}_handle_standoff_{tag}",
        )

    # Revolute hinge on the outer vertical edge
    axis_z = float(panel_sign)  # left: +1, right: -1
    model.articulation(
        f"{name}_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(hinge_x, DOOR_CY, hinge_z)),
        axis=(0.0, 0.0, axis_z),
        motion_limits=MotionLimits(
            effort=40.0, velocity=1.5, lower=0.0, upper=DOOR_OPEN_MAX
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="french_door_refrigerator")

    black_gloss = model.material("gloss_black", rgba=(0.06, 0.06, 0.07, 1.0))
    body_black = model.material("body_black", rgba=(0.09, 0.09, 0.10, 1.0))
    kick_gray = model.material("kick_dark_gray", rgba=(0.24, 0.24, 0.26, 1.0))
    liner_white = model.material("liner_white", rgba=(0.93, 0.93, 0.94, 1.0))
    shelf_glass = model.material("shelf_glass", rgba=(0.82, 0.88, 0.92, 0.55))
    aluminum = model.material("brushed_aluminum", rgba=(0.78, 0.79, 0.81, 1.0))
    badge_silver = model.material("badge_silver", rgba=(0.86, 0.87, 0.88, 1.0))
    drawer_liner = model.material("drawer_liner", rgba=(0.88, 0.88, 0.90, 1.0))

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
    # horizontal mullion dividing bottom freezer from upper fresh-food compartment
    cabinet.visual(
        Box((W, body_d, MULLION_TH)),
        origin=Origin(xyz=(0.0, body_cy, MULLION_CZ)),
        material=body_black,
        name="mullion",
    )

    # interior back liner (visible when doors/drawer open)
    inner_w = W - 2 * WALL
    cabinet.visual(
        Box((inner_w, 0.006, body_h - 2 * WALL)),
        origin=Origin(xyz=(0.0, BODY_BACK_Y + WALL + 0.003, body_cz)),
        material=liner_white,
        name="back_liner",
    )

    # glass shelves in the fresh-food compartment (above mullion)
    shelf_w = inner_w + 0.004
    shelf_d = body_d - WALL - 0.05
    shelf_cy = BODY_BACK_Y + WALL + shelf_d / 2.0
    fridge_bot = MULLION_CZ + MULLION_TH / 2.0
    fridge_top = H - WALL
    for i in range(3):
        sz = fridge_bot + (i + 1) * (fridge_top - fridge_bot) / 4.0
        cabinet.visual(
            Box((shelf_w, shelf_d, 0.008)),
            origin=Origin(xyz=(0.0, shelf_cy, sz)),
            material=shelf_glass,
            name=f"fridge_shelf_{i}",
        )
    # The bottom freezer compartment is a pull-out drawer; no internal shelf
    # is needed since the drawer tray itself organizes the frozen items.

    # ---- Hinge brackets + pins at the outer front edges (one set per door) ----
    plate_d = HINGE_PLATE_Y1 - HINGE_PLATE_Y0
    plate_cy = (HINGE_PLATE_Y0 + HINGE_PLATE_Y1) / 2.0

    for i in range(2):
        hinge_sign = -1 + 2 * i  # left: -1, right: +1
        hinge_x = hinge_sign * (W / 2.0 - 0.005)
        plate_cx = hinge_sign * (W / 2.0 - HINGE_PLATE_W / 2.0)

        # Top bracket: above the door, pin points down into door top edge
        top_plate_cz = FRENCH_DOOR_Z1 + 0.002 + HINGE_PLATE_TH / 2.0
        top_pin_z0 = FRENCH_DOOR_Z1 - 0.015
        top_pin_z1 = top_plate_cz + HINGE_PLATE_TH / 2.0
        cabinet.visual(
            Box((HINGE_PLATE_W, plate_d, HINGE_PLATE_TH)),
            origin=Origin(xyz=(plate_cx, plate_cy, top_plate_cz)),
            material=body_black,
            name=f"hinge_bracket_{i}_top",
        )
        cabinet.visual(
            Cylinder(radius=HINGE_PIN_R, length=top_pin_z1 - top_pin_z0),
            origin=Origin(xyz=(hinge_x, DOOR_CY, (top_pin_z0 + top_pin_z1) / 2.0)),
            material=aluminum,
            name=f"hinge_pin_{i}_top",
        )

        # Bottom bracket: below the door, pin points up into door bottom edge
        bot_plate_cz = FRENCH_DOOR_Z0 - 0.002 - HINGE_PLATE_TH / 2.0
        bot_pin_z0 = bot_plate_cz - HINGE_PLATE_TH / 2.0 + 0.002
        bot_pin_z1 = FRENCH_DOOR_Z0 + 0.015
        cabinet.visual(
            Box((HINGE_PLATE_W, plate_d, HINGE_PLATE_TH)),
            origin=Origin(xyz=(plate_cx, plate_cy, bot_plate_cz)),
            material=body_black,
            name=f"hinge_bracket_{i}_bottom",
        )
        cabinet.visual(
            Cylinder(radius=HINGE_PIN_R, length=bot_pin_z1 - bot_pin_z0),
            origin=Origin(xyz=(hinge_x, DOOR_CY, (bot_pin_z0 + bot_pin_z1) / 2.0)),
            material=aluminum,
            name=f"hinge_pin_{i}_bottom",
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

    # ---- French doors (loop for i in range(2)) ----
    for i in range(2):
        _build_french_door(
            model,
            index=i,
            height=FRENCH_DOOR_H,
            hinge_z=FRENCH_DOOR_CZ,
            black_gloss=black_gloss,
            aluminum=aluminum,
            cabinet=cabinet,
        )

    # small silver brand badge on the right French door upper area
    right_door = model.get_part("french_door_1")
    right_door.visual(
        Box((0.05, 0.003, 0.018)),
        origin=Origin(
            xyz=(-(HALF_DOOR_W * 0.45), DOOR_TH / 2.0, FRENCH_DOOR_H / 2.0 - 0.10)
        ),
        material=badge_silver,
        name="brand_badge",
    )

    # ---- Freezer drawer (PRISMATIC along +Y) ----
    drawer = model.part("freezer_drawer")

    # Drawer front panel (same plane as the French doors)
    drawer.visual(
        _drawer_front_mesh(DOOR_W, DRAWER_H, "drawer_front"),
        origin=Origin(xyz=(0.0, DOOR_GAP_Y + DOOR_TH / 2.0, 0.0)),
        material=black_gloss,
        name="drawer_front",
    )

    # Horizontal brushed-aluminum handle bar near the top of the drawer front
    drawer_handle_y = DOOR_GAP_Y + DOOR_TH + HANDLE_PROUD
    drawer_handle_z = DRAWER_H / 2.0 - 0.05
    drawer.visual(
        Cylinder(radius=HANDLE_BAR_R, length=HANDLE_BAR_LEN),
        origin=Origin(
            xyz=(0.0, drawer_handle_y, drawer_handle_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=aluminum,
        name="drawer_handle_bar",
    )
    for tag, sx in (("a", -HANDLE_BAR_LEN * 0.38), ("b", HANDLE_BAR_LEN * 0.38)):
        drawer.visual(
            Cylinder(radius=HANDLE_STANDOFF_R, length=0.034),
            origin=Origin(
                xyz=(sx, DOOR_GAP_Y + DOOR_TH + 0.013, drawer_handle_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=aluminum,
            name=f"drawer_handle_standoff_{tag}",
        )

    # Drawer tray body (extends back into the cabinet for retained insertion).
    # Width matches the inner cavity so tray sides contact the side walls
    # (representing the slide-rail interface between drawer and cabinet).
    tray_depth = 0.40
    tray_h = DRAWER_H - 0.08  # small clearance above/below compartment walls
    tray_w = inner_w          # full inner width; slide-rail contact
    drawer.visual(
        Box((tray_w, tray_depth, tray_h)),
        origin=Origin(
            xyz=(0.0, DOOR_GAP_Y - tray_depth / 2.0, 0.0)
        ),
        material=drawer_liner,
        name="drawer_tray",
    )

    # Prismatic articulation: drawer pulls out along +Y
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        origin=Origin(xyz=(0.0, BODY_FRONT_Y, DRAWER_CZ)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.25, lower=0.0, upper=DRAWER_TRAVEL
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    french_door_0 = object_model.get_part("french_door_0")
    french_door_1 = object_model.get_part("french_door_1")
    freezer_drawer = object_model.get_part("freezer_drawer")
    hinge_0 = object_model.get_articulation("french_door_0_hinge")
    hinge_1 = object_model.get_articulation("french_door_1_hinge")
    drawer_slide = object_model.get_articulation("drawer_slide")

    # ---- Hinge pins captured in door edge bores (intentional overlap) ----
    for door, slab_elem, pin_elems in (
        (french_door_0, "french_door_0_slab", ("hinge_pin_0_top", "hinge_pin_0_bottom")),
        (french_door_1, "french_door_1_slab", ("hinge_pin_1_top", "hinge_pin_1_bottom")),
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

    # ---- French doors sit slightly proud of the carcass with a thin gap ----
    for door, label in ((french_door_0, "french_door_0"), (french_door_1, "french_door_1")):
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

    # ---- French doors are on opposite sides of the cabinet ----
    d0_aabb = ctx.part_world_aabb(french_door_0)
    d1_aabb = ctx.part_world_aabb(french_door_1)
    ctx.check(
        "left French door is on the left side and right on the right side",
        d0_aabb is not None
        and d1_aabb is not None
        and d0_aabb[0][0] < d1_aabb[0][0]
        and (d0_aabb[0][0] + d0_aabb[1][0]) / 2.0 < 0.0
        and (d1_aabb[0][0] + d1_aabb[1][0]) / 2.0 > 0.0,
        details=f"d0_aabb={d0_aabb}, d1_aabb={d1_aabb}",
    )

    # ---- Thin center gap between the two French doors ----
    ctx.expect_gap(
        french_door_1,
        french_door_0,
        axis="x",
        min_gap=0.001,
        max_gap=0.02,
        name="thin center gap between the two French doors",
    )

    # ---- French door hinges are on the outer vertical edges ----
    for hinge, label, expected_sign in (
        (hinge_0, "left", -1.0),
        (hinge_1, "right", 1.0),
    ):
        ox, oy, _ = hinge.origin.xyz
        limits = hinge.motion_limits
        ctx.check(
            f"{label} French door hinge is on the outer front edge",
            ox * expected_sign > W / 2.0 - 0.02
            and oy > BODY_FRONT_Y - 0.01
            and limits is not None
            and limits.lower == 0.0
            and math.radians(110.0) < limits.upper < math.radians(130.0),
            details=f"origin=({ox}, {oy}), limits={limits}",
        )

    # ---- Each French door opens outward independently ----
    closed_d0 = ctx.part_world_aabb(french_door_0)
    closed_d1 = ctx.part_world_aabb(french_door_1)

    with ctx.pose({hinge_0: DOOR_OPEN_MAX}):
        open_d0 = ctx.part_world_aabb(french_door_0)
        d1_pos_during_d0 = ctx.part_world_position(french_door_1)
    d1_rest_pos = ctx.part_world_position(french_door_1)

    ctx.check(
        "left French door free edge swings outward when opened",
        closed_d0 is not None
        and open_d0 is not None
        and open_d0[1][1] > closed_d0[1][1] + 0.10,
        details=f"closed_max_y={closed_d0[1][1] if closed_d0 else None}, "
                f"open_max_y={open_d0[1][1] if open_d0 else None}",
    )
    ctx.check(
        "right French door is independent of the left door",
        d1_rest_pos is not None
        and d1_pos_during_d0 is not None
        and all(abs(a - b) < 1e-9 for a, b in zip(d1_rest_pos, d1_pos_during_d0)),
        details=f"rest={d1_rest_pos}, during={d1_pos_during_d0}",
    )

    with ctx.pose({hinge_1: DOOR_OPEN_MAX}):
        open_d1 = ctx.part_world_aabb(french_door_1)
    ctx.check(
        "right French door free edge swings outward when opened",
        closed_d1 is not None
        and open_d1 is not None
        and open_d1[1][1] > closed_d1[1][1] + 0.10,
        details=f"closed_max_y={closed_d1[1][1] if closed_d1 else None}, "
                f"open_max_y={open_d1[1][1] if open_d1 else None}",
    )

    # ---- Freezer drawer: PRISMATIC along +Y, pulls out toward user ----
    ctx.check(
        "drawer slide is prismatic along +Y with sensible travel",
        drawer_slide.articulation_type == ArticulationType.PRISMATIC
        and drawer_slide.axis == (0.0, 1.0, 0.0)
        and drawer_slide.motion_limits is not None
        and drawer_slide.motion_limits.lower == 0.0
        and 0.25 < drawer_slide.motion_limits.upper < 0.45,
        details=f"axis={drawer_slide.axis}, limits={drawer_slide.motion_limits}",
    )

    drawer_rest_y = ctx.part_world_position(freezer_drawer)
    with ctx.pose({drawer_slide: DRAWER_TRAVEL}):
        drawer_ext_y = ctx.part_world_position(freezer_drawer)
    ctx.check(
        "freezer drawer pulls out toward the user (+Y)",
        drawer_rest_y is not None
        and drawer_ext_y is not None
        and drawer_ext_y[1] > drawer_rest_y[1] + 0.20,
        details=f"rest={drawer_rest_y}, extended={drawer_ext_y}",
    )

    # Drawer tray stays inside the cabinet footprint at rest (retained insertion)
    tray_aabb = ctx.part_element_world_aabb(freezer_drawer, elem="drawer_tray")
    ctx.check(
        "drawer tray stays within cabinet width at rest",
        tray_aabb is not None
        and tray_aabb[0][0] >= -W / 2.0 + WALL - 0.001
        and tray_aabb[1][0] <= W / 2.0 - WALL + 0.001,
        details=f"tray_aabb={tray_aabb}",
    )

    # ---- Drawer sits below the French doors, separated by the mullion ----
    ctx.expect_gap(
        french_door_0,
        freezer_drawer,
        axis="z",
        positive_elem="french_door_0_slab",
        negative_elem="drawer_front",
        min_gap=0.01,
        max_gap=0.08,
        name="French doors sit above the freezer drawer with a mullion gap",
    )

    # Mullion divider spans the gap between doors and drawer
    mull = ctx.part_element_world_aabb(cabinet, elem="mullion")
    dr_aabb = ctx.part_world_aabb(freezer_drawer)
    ctx.check(
        "mullion divider spans the gap between the French doors and the drawer",
        mull is not None
        and d0_aabb is not None
        and dr_aabb is not None
        and mull[0][2] < d0_aabb[0][2]
        and mull[1][2] > dr_aabb[1][2] - 0.005,
        details=f"mullion={mull}",
    )

    # ---- Handles: vertical bars on French doors, horizontal bar on drawer ----
    for door, label in ((french_door_0, "french_door_0"), (french_door_1, "french_door_1")):
        slab = ctx.part_element_world_aabb(door, elem=f"{label}_slab")
        bar = ctx.part_element_world_aabb(door, elem=f"{label}_handle_bar")
        ctx.check(
            f"{label} handle bar is vertical and proud of the door face",
            slab is not None
            and bar is not None
            and bar[0][1] > slab[1][1] - HANDLE_BAR_R - 0.002
            and (bar[1][2] - bar[0][2]) > 0.40   # vertical span
            and (bar[1][0] - bar[0][0]) < 0.04,   # narrow in X
            details=f"slab={slab}, bar={bar}",
        )

    # Drawer handle is horizontal and proud of the drawer front
    dr_front = ctx.part_element_world_aabb(freezer_drawer, elem="drawer_front")
    dr_bar = ctx.part_element_world_aabb(freezer_drawer, elem="drawer_handle_bar")
    ctx.check(
        "drawer handle bar is horizontal and proud of the drawer front",
        dr_front is not None
        and dr_bar is not None
        and dr_bar[0][1] > dr_front[1][1] - HANDLE_BAR_R - 0.002
        and (dr_bar[1][0] - dr_bar[0][0]) > 0.35   # horizontal span in X
        and (dr_bar[1][2] - dr_bar[0][2]) < 0.04,   # narrow in Z
        details=f"front={dr_front}, bar={dr_bar}",
    )

    # ---- Brand badge on the right French door upper area ----
    badge = ctx.part_element_world_aabb(french_door_1, elem="brand_badge")
    d1_slab = ctx.part_element_world_aabb(french_door_1, elem="french_door_1_slab")
    ctx.check(
        "silver badge sits on the upper half of the right French door front",
        badge is not None
        and d1_slab is not None
        and (badge[0][2] + badge[1][2]) / 2.0
        > (d1_slab[0][2] + d1_slab[1][2]) / 2.0
        and badge[1][1] > d1_slab[1][1] - 0.002,
        details=f"badge={badge}",
    )

    # ---- Recessed kick panel at the bottom front ----
    kick = ctx.part_element_world_aabb(cabinet, elem="kick_panel")
    dr_front_aabb = ctx.part_element_world_aabb(freezer_drawer, elem="drawer_front")
    ctx.check(
        "kick panel is short, at the floor, and recessed behind the drawer front",
        kick is not None
        and dr_front_aabb is not None
        and kick[1][2] <= DRAWER_Z0 + 0.001
        and kick[0][2] < 0.005
        and kick[1][1] < dr_front_aabb[0][1] - 0.02,
        details=f"kick={kick}, drawer_front={dr_front_aabb}",
    )

    # ---- Hollow interior: shelves inside the wall-to-wall cavity ----
    shelf = ctx.part_element_world_aabb(cabinet, elem="fridge_shelf_0")
    ctx.check(
        "fresh-food shelf spans the hollow interior between the side walls",
        shelf is not None
        and shelf[0][0] > -W / 2.0 + WALL - 0.005
        and shelf[1][0] < W / 2.0 - WALL + 0.005
        and shelf[0][2] > MULLION_CZ + MULLION_TH / 2.0
        and shelf[1][2] < H - WALL,
        details=f"shelf={shelf}",
    )

    # ---- Freezer compartment occupies roughly the bottom 30% ----
    ctx.check(
        "freezer compartment occupies roughly the bottom 30% of the height",
        dr_aabb is not None
        and 0.22 * H < (dr_aabb[1][2] - dr_aabb[0][2]) < 0.36 * H
        and dr_aabb[1][2] < 0.40 * H,
        details=f"drawer_aabb={dr_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
