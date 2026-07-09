from __future__ import annotations

"""Dish-drawer style stainless-steel dishwasher.

Articraft brief:
- Object: single-drawer dishwasher, ~0.60 m wide x 0.62 m deep x 0.85 m tall.
- Root/support: hollow cabinet shell (outer housing) with recessed kick plinth,
  brushed countertop-style top slab, and a fixed lower front panel below the
  drawer opening.
- Parts: cabinet (root), drawer (prismatic slide carrying front panel + tub
  liner + chrome wire rack + slide runners).
- Articulation: drawer_slide — PRISMATIC along +Y (depth axis, front = +Y),
  0..0.42 m of forward travel. The entire tub assembly pulls out as one drawer.
- Visible geometry: brushed silver shell, polished stainless tub inside the
  drawer, chrome wire basket rack, front panel with dark control strip,
  recessed pocket handle, round buttons, blue LCD display.
- Frame: X = width, Y = depth (front = +Y), Z = up; grounded at z = 0.
"""

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

# ---------------------------------------------------------------- dimensions
TOP_W = 0.60          # top slab width
TOP_D = 0.62          # top slab depth
TOTAL_H = 0.85        # overall height

BODY_W = 0.58         # cabinet body width
BODY_FRONT_Y = 0.27   # cabinet front face y
BODY_BACK_Y = -0.29   # cabinet back face y
WALL_T = 0.02         # shell wall thickness

PLINTH_H = 0.07       # kick plinth height
BODY_TOP_Z = 0.82     # cabinet top (underside of slab)
SLAB_T = 0.03         # top slab thickness

# Drawer opening & front panel
DRAWER_BOTTOM_Z = 0.40
DRAWER_PANEL_H = BODY_TOP_Z - DRAWER_BOTTOM_Z   # 0.42 m
DRAWER_PANEL_T = 0.022                           # front panel thickness

# Prismatic travel
DRAWER_TRAVEL = 0.42

# Tub inside the drawer (carried by the drawer part)
TUB_DEPTH = 0.46
TUB_WALL_T = 0.008
TUB_INTERIOR_H = DRAWER_PANEL_H - 0.04           # 0.38 m

# Chrome wire rack inside the tub
RACK_W = 0.48
RACK_D = 0.40
RACK_H = 0.14
WIRE = 0.006
RACK_BASE_Z = 0.02 + TUB_WALL_T                  # sits on tub floor

# ---------------------------------------------------------------- materials
BRUSHED_STEEL = Material(name="brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
STEEL_TOP = Material(name="steel_top", rgba=(0.66, 0.67, 0.69, 1.0))
POLISHED_TUB = Material(name="polished_tub", rgba=(0.82, 0.83, 0.85, 1.0))
PLINTH_GRAY = Material(name="plinth_gray", rgba=(0.28, 0.29, 0.30, 1.0))
STRIP_GRAY = Material(name="strip_gray", rgba=(0.20, 0.21, 0.23, 1.0))
HANDLE_DARK = Material(name="handle_dark", rgba=(0.10, 0.10, 0.11, 1.0))
LCD_BLUE = Material(name="lcd_blue", rgba=(0.25, 0.50, 0.92, 1.0))
BUTTON_GRAY = Material(name="button_gray", rgba=(0.55, 0.56, 0.58, 1.0))
CHROME_WIRE = Material(name="chrome_wire", rgba=(0.85, 0.86, 0.88, 1.0))
RAIL_STEEL = Material(name="rail_steel", rgba=(0.60, 0.61, 0.63, 1.0))


# ---------------------------------------------------------------- helpers
def _linspace(lo: float, hi: float, n: int) -> list[float]:
    if n == 1:
        return [(lo + hi) * 0.5]
    step = (hi - lo) / (n - 1)
    return [lo + i * step for i in range(n)]


def _add_rack_wires(
    part,
    *,
    prefix: str,
    hw: float,
    hd: float,
    rack_yc: float,
    base_z: float,
    height: float,
) -> None:
    """Shared helper: open wire-frame basket at a given position in *part*."""
    # Longitudinal floor rods (along Y)
    for i, x in enumerate(_linspace(-hw + 0.01, hw - 0.01, 9)):
        part.visual(
            Box((WIRE, 2 * hd - 0.004, WIRE)),
            origin=Origin(xyz=(x, rack_yc, base_z + 0.003)),
            material=CHROME_WIRE,
            name=f"{prefix}_floor_rod_y_{i}",
        )
    cross_ys = _linspace(rack_yc - hd + 0.003, rack_yc + hd - 0.003, 5)
    for i, y in enumerate(cross_ys):
        part.visual(
            Box((2 * hw, WIRE, WIRE)),
            origin=Origin(xyz=(0.0, y, base_z + 0.007)),
            material=CHROME_WIRE,
            name=f"{prefix}_floor_rod_x_{i}",
        )
    # Top perimeter rail
    rail_z = base_z + height - WIRE / 2
    for i, y in enumerate((rack_yc - hd + 0.003, rack_yc + hd - 0.003)):
        part.visual(
            Box((2 * hw, WIRE, WIRE)),
            origin=Origin(xyz=(0.0, y, rail_z)),
            material=CHROME_WIRE,
            name=f"{prefix}_top_rail_xy_{i}",
        )
    for i, x in enumerate((-hw + 0.003, hw - 0.003)):
        part.visual(
            Box((WIRE, 2 * hd, WIRE)),
            origin=Origin(xyz=(x, rack_yc, rail_z)),
            material=CHROME_WIRE,
            name=f"{prefix}_top_rail_side_{i}",
        )
    # Vertical side wires (aligned with cross rods)
    for j, y in enumerate(cross_ys):
        for k, x in enumerate((-hw + 0.003, hw - 0.003)):
            part.visual(
                Box((WIRE, WIRE, height)),
                origin=Origin(xyz=(x, y, base_z + height / 2)),
                material=CHROME_WIRE,
                name=f"{prefix}_side_wire_{k}_{j}",
            )
    # Front/back vertical wires (aligned with longitudinal rods)
    face_xs = _linspace(-hw + 0.01, hw - 0.01, 7)
    for j, x in enumerate(face_xs):
        for k, y in enumerate((rack_yc - hd + 0.003, rack_yc + hd - 0.003)):
            part.visual(
                Box((WIRE, WIRE, height)),
                origin=Origin(xyz=(x, y, base_z + height / 2)),
                material=CHROME_WIRE,
                name=f"{prefix}_face_wire_{k}_{j}",
            )


# ---------------------------------------------------------------- cabinet
def _build_cabinet(model: ArticulatedObject):
    """Fixed outer shell: plinth, walls, back, floor, ceiling, top slab,
    lower front panel, and stationary drawer slide rails."""
    cab = model.part("cabinet")

    # Recessed dark kick plinth (slightly embedded into the floor for
    # connectivity within the same part).
    cab.visual(
        Box((0.54, 0.56, PLINTH_H + 0.005)),
        origin=Origin(xyz=(0.0, -0.01, (PLINTH_H + 0.005) / 2)),
        material=PLINTH_GRAY,
        name="kick_plinth",
    )

    # Shell walls
    wall_h = BODY_TOP_Z - PLINTH_H
    wall_zc = PLINTH_H + wall_h / 2
    body_d = BODY_FRONT_Y - BODY_BACK_Y
    body_yc = (BODY_FRONT_Y + BODY_BACK_Y) / 2

    for i, (sx, tag) in enumerate(((-1.0, "side_wall_0"), (1.0, "side_wall_1"))):
        cab.visual(
            Box((WALL_T, body_d, wall_h)),
            origin=Origin(xyz=(sx * (BODY_W / 2 - WALL_T / 2), body_yc, wall_zc)),
            material=BRUSHED_STEEL,
            name=tag,
        )
    cab.visual(
        Box((BODY_W - 2 * WALL_T, WALL_T, wall_h)),
        origin=Origin(xyz=(0.0, BODY_BACK_Y + WALL_T / 2, wall_zc)),
        material=BRUSHED_STEEL,
        name="back_wall",
    )
    cab.visual(
        Box((BODY_W - 2 * WALL_T, body_d, WALL_T)),
        origin=Origin(xyz=(0.0, body_yc, PLINTH_H + WALL_T / 2)),
        material=BRUSHED_STEEL,
        name="cabinet_floor",
    )
    cab.visual(
        Box((BODY_W - 2 * WALL_T, body_d, WALL_T)),
        origin=Origin(xyz=(0.0, body_yc, BODY_TOP_Z - WALL_T / 2)),
        material=BRUSHED_STEEL,
        name="cabinet_ceiling",
    )

    # Fixed lower front panel (below the drawer opening)
    lower_panel_h = DRAWER_BOTTOM_Z - PLINTH_H
    cab.visual(
        Box((BODY_W - 2 * WALL_T, WALL_T, lower_panel_h)),
        origin=Origin(
            xyz=(0.0, BODY_FRONT_Y - WALL_T / 2, PLINTH_H + lower_panel_h / 2)
        ),
        material=BRUSHED_STEEL,
        name="lower_front_panel",
    )

    # Overhanging brushed countertop-style top slab
    # Bottom face sits flush at BODY_TOP_Z so it contacts the drawer panel top.
    cab.visual(
        Box((TOP_W, TOP_D, SLAB_T)),
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP_Z + SLAB_T / 2)),
        material=STEEL_TOP,
        name="top_slab",
    )

    # Stationary drawer slide channels on the inner side walls
    rail_h = 0.032
    rail_d = 0.52
    rail_zc = DRAWER_BOTTOM_Z + DRAWER_PANEL_H / 2
    for i, sx in enumerate((-1.0, 1.0)):
        cab.visual(
            Box((0.014, rail_d, rail_h)),
            origin=Origin(
                xyz=(sx * (BODY_W / 2 - WALL_T - 0.005), body_yc + 0.02, rail_zc)
            ),
            material=RAIL_STEEL,
            name=f"cabinet_rail_{i}",
        )

    return cab


# ---------------------------------------------------------------- drawer
def _build_drawer(model: ArticulatedObject):
    """Pull-out drawer: front panel + tub liner + rack + slide runners.

    Local frame: origin at the bottom center of the drawer front face.
    At q=0, this coincides with the joint origin at
    (0, BODY_FRONT_Y, DRAWER_BOTTOM_Z).
    - Front panel extends +Y (thickness) and +Z (height).
    - Tub extends -Y (backward into cabinet) and +Z (upward).
    """
    drawer = model.part("drawer")

    # --- front panel ---------------------------------------------------
    drawer.visual(
        Box((BODY_W, DRAWER_PANEL_T, DRAWER_PANEL_H)),
        origin=Origin(xyz=(0.0, DRAWER_PANEL_T / 2, DRAWER_PANEL_H / 2)),
        material=BRUSHED_STEEL,
        name="drawer_panel",
    )

    # Dark control strip across the top edge
    strip_h = 0.065
    strip_zc = DRAWER_PANEL_H - 0.005 - strip_h / 2
    drawer.visual(
        Box((BODY_W, 0.010, strip_h)),
        origin=Origin(xyz=(0.0, DRAWER_PANEL_T + 0.003, strip_zc)),
        material=STRIP_GRAY,
        name="control_strip",
    )
    strip_face = DRAWER_PANEL_T + 0.008

    # Recessed pocket handle
    drawer.visual(
        Box((0.20, 0.005, 0.026)),
        origin=Origin(xyz=(0.0, strip_face + 0.0015, strip_zc - 0.006)),
        material=HANDLE_DARK,
        name="pocket_handle",
    )

    # Blue LCD display
    drawer.visual(
        Box((0.085, 0.004, 0.024)),
        origin=Origin(xyz=(0.155, strip_face + 0.001, strip_zc + 0.004)),
        material=LCD_BLUE,
        name="display_lcd",
    )

    # Round buttons (cylinders along +Y)
    for i, bx in enumerate((-0.215, -0.180, -0.145, -0.110)):
        drawer.visual(
            Cylinder(radius=0.0075, length=0.008),
            origin=Origin(
                xyz=(bx, strip_face, strip_zc + 0.004),
                rpy=(math.pi / 2, 0.0, 0.0),
            ),
            material=BUTTON_GRAY,
            name=f"button_{i}",
        )

    # --- tub liner (wash chamber) --------------------------------------
    tub_wall_xc = BODY_W / 2 - WALL_T - TUB_WALL_T / 2 - 0.018
    tub_zc = TUB_INTERIOR_H / 2 + 0.02
    for i, sx in enumerate((-1.0, 1.0)):
        drawer.visual(
            Box((TUB_WALL_T, TUB_DEPTH, TUB_INTERIOR_H)),
            origin=Origin(xyz=(sx * tub_wall_xc, -TUB_DEPTH / 2, tub_zc)),
            material=POLISHED_TUB,
            name=f"tub_wall_{i}",
        )
    drawer.visual(
        Box((BODY_W - 2 * WALL_T - 0.02, TUB_WALL_T, TUB_INTERIOR_H)),
        origin=Origin(xyz=(0.0, -TUB_DEPTH + TUB_WALL_T / 2, tub_zc)),
        material=POLISHED_TUB,
        name="tub_back_panel",
    )
    drawer.visual(
        Box((BODY_W - 2 * WALL_T - 0.02, TUB_DEPTH, TUB_WALL_T)),
        origin=Origin(xyz=(0.0, -TUB_DEPTH / 2, 0.02 + TUB_WALL_T / 2)),
        material=POLISHED_TUB,
        name="tub_floor",
    )

    # --- slide runners (moving members on the drawer sides) ------------
    runner_w = 0.012
    runner_h = 0.026
    runner_d = TUB_DEPTH + 0.02
    # Runner outer face stays inside the cabinet side wall inner face
    # (BODY_W/2 - WALL_T = 0.27); runner inner face embeds into the tub wall
    # outer face for within-part connectivity.
    runner_xc = BODY_W / 2 - WALL_T - runner_w / 2 - 0.006
    for i, sx in enumerate((-1.0, 1.0)):
        drawer.visual(
            Box((runner_w, runner_d, runner_h)),
            origin=Origin(
                xyz=(
                    sx * runner_xc,
                    -TUB_DEPTH / 2 + 0.01,
                    DRAWER_PANEL_H / 2,
                )
            ),
            material=RAIL_STEEL,
            name=f"drawer_runner_{i}",
        )

    # --- chrome wire rack inside the tub --------------------------------
    _add_rack_wires(
        drawer,
        prefix="rack",
        hw=RACK_W / 2,
        hd=RACK_D / 2,
        rack_yc=-TUB_DEPTH / 2,
        base_z=RACK_BASE_Z,
        height=RACK_H,
    )

    return drawer


# ---------------------------------------------------------------- build
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dish_drawer_dishwasher")

    cabinet = _build_cabinet(model)
    drawer = _build_drawer(model)

    # Drawer slide: prismatic along +Y (forward out of the cabinet).
    # Origin at the bottom edge of the drawer opening on the cabinet front.
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        origin=Origin(xyz=(0.0, BODY_FRONT_Y, DRAWER_BOTTOM_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0, velocity=0.5, lower=0.0, upper=DRAWER_TRAVEL
        ),
    )

    return model


# ---------------------------------------------------------------- tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    drawer = object_model.get_part("drawer")
    slide = object_model.get_articulation("drawer_slide")

    # --- overall envelope: ~0.60 x 0.62 x 0.85, grounded at z = 0
    aabb = ctx.part_world_aabb(cabinet)
    ok = aabb is not None
    if ok:
        (x0, y0, z0), (x1, y1, z1) = aabb
        ok = (
            abs((x1 - x0) - TOP_W) < 0.02
            and abs((y1 - y0) - TOP_D) < 0.02
            and abs(z1 - TOTAL_H) < 0.02
            and abs(z0) < 1e-6
        )
    ctx.check(
        "cabinet has ~0.60 x 0.62 x 0.85 envelope grounded at z=0",
        ok,
        details=f"cabinet aabb = {aabb}",
    )

    # --- joint: prismatic along +Y with correct travel
    ctx.check(
        "drawer slide is prismatic along +Y with 0..0.42 m travel",
        slide.articulation_type == ArticulationType.PRISMATIC
        and abs(slide.axis[1] - 1.0) < 1e-9
        and abs(slide.axis[0]) < 1e-9
        and abs(slide.axis[2]) < 1e-9
        and slide.motion_limits is not None
        and abs(slide.motion_limits.lower - 0.0) < 1e-9
        and abs(slide.motion_limits.upper - DRAWER_TRAVEL) < 1e-9,
        details=f"axis={slide.axis}, limits={slide.motion_limits}",
    )
    ctx.check(
        "slide origin sits at the cabinet front at the drawer opening bottom",
        abs(slide.origin.xyz[1] - BODY_FRONT_Y) < 1e-9
        and abs(slide.origin.xyz[2] - DRAWER_BOTTOM_Z) < 1e-9,
        details=f"slide origin = {slide.origin.xyz}",
    )

    # --- closed pose: drawer front panel flush at cabinet front opening
    drawer_aabb = ctx.part_world_aabb(drawer)
    ok = drawer_aabb is not None
    if ok:
        (dx0, dy0, dz0), (dx1, dy1, dz1) = drawer_aabb
        ok = (
            (dz1 - dz0) > 0.35               # tall enough (panel + tub)
            and dy1 >= BODY_FRONT_Y - 0.005   # front face near cabinet front
            and abs(dz0 - DRAWER_BOTTOM_Z) < 0.01
        )
    ctx.check(
        "closed drawer sits at the opening with its front panel at the cabinet front",
        ok,
        details=f"drawer aabb = {drawer_aabb}",
    )

    # Drawer tub is inside the cabinet footprint when closed
    ctx.expect_within(
        drawer,
        cabinet,
        axes="xy",
        margin=0.03,
        name="closed drawer tub stays inside the cabinet footprint",
    )

    # --- control strip features on the drawer front panel
    lcd_aabb = ctx.part_element_world_aabb(drawer, elem="display_lcd")
    handle_aabb = ctx.part_element_world_aabb(drawer, elem="pocket_handle")
    ok = lcd_aabb is not None and handle_aabb is not None
    if ok:
        ok = (
            lcd_aabb[1][2] > 0.70
            and handle_aabb[1][2] > 0.70
            and lcd_aabb[0][1] > BODY_FRONT_Y + DRAWER_PANEL_T - 0.005
        )
    ctx.check(
        "LCD and pocket handle sit on the drawer front control strip, proud of the panel",
        ok,
        details=f"lcd={lcd_aabb}, handle={handle_aabb}",
    )
    btn_aabb = ctx.part_element_world_aabb(drawer, elem="button_0")
    ctx.check(
        "round buttons sit off-axis on the left of the control strip",
        btn_aabb is not None and btn_aabb[1][0] < -0.10,
        details=f"button_0 aabb = {btn_aabb}",
    )

    # --- tub & rack inside the drawer
    tub_floor_aabb = ctx.part_element_world_aabb(drawer, elem="tub_floor")
    ctx.check(
        "drawer contains a polished tub floor above the plinth",
        tub_floor_aabb is not None and tub_floor_aabb[0][2] > PLINTH_H + 0.30,
        details=f"tub_floor aabb = {tub_floor_aabb}",
    )

    # --- intentional overlap: drawer runners nested inside cabinet rails
    for i in range(2):
        ctx.allow_overlap(
            drawer,
            cabinet,
            elem_a=f"drawer_runner_{i}",
            elem_b=f"cabinet_rail_{i}",
            reason=(
                f"Drawer runner {i} slides inside the stationary cabinet "
                f"rail {i} channel — a telescoping slide mechanism."
            ),
        )
    # Proof: runners stay centered in rails on the non-slide axes
    for i in range(2):
        ctx.expect_contact(
            drawer,
            cabinet,
            elem_a=f"drawer_runner_{i}",
            elem_b=f"cabinet_rail_{i}",
            contact_tol=0.006,
            name=f"drawer_runner_{i} stays in contact with cabinet_rail_{i}",
        )

    # --- open pose: drawer slides forward, tub and rack come out
    rest_pos = ctx.part_world_position(drawer)
    with ctx.pose({slide: DRAWER_TRAVEL}):
        ext_pos = ctx.part_world_position(drawer)
        ctx.check(
            "drawer slides forward by the full travel distance",
            rest_pos is not None
            and ext_pos is not None
            and abs((ext_pos[1] - rest_pos[1]) - DRAWER_TRAVEL) < 1e-6,
            details=f"rest={rest_pos}, extended={ext_pos}",
        )
        # Drawer is still partly retained inside the cabinet
        ctx.expect_overlap(
            drawer,
            cabinet,
            axes="y",
            min_overlap=0.02,
            name="extended drawer stays partly retained in the cabinet",
        )
        # Front panel projects well past the cabinet front
        ext_aabb = ctx.part_world_aabb(drawer)
        ctx.check(
            "extended drawer front projects well past the cabinet front",
            ext_aabb is not None and ext_aabb[1][1] > BODY_FRONT_Y + 0.30,
            details=f"extended drawer aabb = {ext_aabb}",
        )
        # Tub side walls are now visible outside the cabinet
        tub_wall_aabb = ctx.part_element_world_aabb(drawer, elem="tub_wall_0")
        ctx.check(
            "tub side walls extend past the cabinet front when drawer is open",
            tub_wall_aabb is not None and tub_wall_aabb[1][1] > BODY_FRONT_Y + 0.10,
            details=f"tub_wall_0 aabb = {tub_wall_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
