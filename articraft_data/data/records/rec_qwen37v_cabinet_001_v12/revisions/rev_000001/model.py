from __future__ import annotations

"""Vintage industrial steel dresser cabinet with three horizontal drawers.

Variant of the vintage steel locker cabinet family, reconfigured as a low wide
dresser.  Overall envelope ~1.2 m wide x 0.5 m deep x ~0.85 m tall.  Brushed /
tarnished raw steel finish.  A hollow thin-wall (~0.02 m) carcass sits on four
short splayed legs and carries a riveted top cap strip.  The front is divided
into three full-width horizontal drawers that slide out on prismatic joints
along +Y.  Each drawer has a flat steel front panel with a narrow horizontal
vent slot (dark recessed, rounded ends) near the bottom and a bail-pull handle
at centre height.  A small rotating turn-latch at the centre of the middle
drawer locks it against the frame when turned.  Interior shelf boards are
visible through the drawer openings when drawers are pulled out.
"""

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Global dimensions (meters). Dresser centred on X, front face at +Y.
# ---------------------------------------------------------------------------
DRS_W = 1.20
DRS_D = 0.50
DRS_TOP = 0.85
LEG_H = 0.12
WALL_T = 0.02

FRONT_Y = DRS_D / 2.0   # +0.25
BACK_Y = -DRS_D / 2.0   # -0.25

CARCASS_H = DRS_TOP - LEG_H  # 0.73
CARCASS_ZC = LEG_H + CARCASS_H / 2.0

# Front frame rails
BOTTOM_RAIL_H = 0.04
TOP_RAIL_H = 0.04
BOTTOM_RAIL_TOP = LEG_H + BOTTOM_RAIL_H  # 0.16
TOP_RAIL_BOT = DRS_TOP - TOP_RAIL_H       # 0.81

# Drawer opening zone
OPENING_Z0 = BOTTOM_RAIL_TOP   # 0.16
OPENING_Z1 = TOP_RAIL_BOT      # 0.81
OPENING_H = OPENING_Z1 - OPENING_Z0  # 0.65

# Three drawers stacked
N_DRAWERS = 3
DRAWER_GAP = 0.006
DRAWER_H = (OPENING_H - (N_DRAWERS - 1) * DRAWER_GAP) / N_DRAWERS
DRAWER_W = DRS_W - 2 * WALL_T - 0.004   # front panel width
DRAWER_BOX_W = DRS_W - 2 * WALL_T - 0.032  # box width (clears guide rails)
DRAWER_D = DRS_D - 2 * WALL_T - 0.04
DRAWER_FRONT_T = WALL_T
DRAWER_SIDE_T = 0.012
DRAWER_BOTTOM_T = 0.010

DRAWER_Z_BOTS = [OPENING_Z0 + i * (DRAWER_H + DRAWER_GAP) for i in range(N_DRAWERS)]
DRAWER_Z_CENTRES = [zb + DRAWER_H / 2.0 for zb in DRAWER_Z_BOTS]

DRAWER_PULL = 0.35

# Vent slot
SLOT_LEN = 0.60
SLOT_W = 0.022
# In drawer-local coords (z=0 at drawer centre), slot near bottom
SLOT_Z_LOCAL = -DRAWER_H / 2.0 + 0.04

# Handle
HANDLE_W = 0.12
HANDLE_H = 0.025
HANDLE_D = 0.018

# Latch
LATCH_BASE_R = 0.014
LATCH_BASE_H = 0.005
LATCH_BOSS_R = 0.005
LATCH_BOSS_H = 0.012
LATCH_BAR_W = 0.008
LATCH_BAR_D = 0.006
LATCH_BAR_H = 0.030
LATCH_TURN = math.radians(90.0)

# Top cap
CAP_T = 0.018
CAP_OVERHANG = 0.015

# Shelf
SHELF_T = 0.012

# Legs
LEG_FOOT = 0.030
LEG_TOP_SIZE = 0.050


def _drawer_front_mesh(mesh_name: str):
    """Drawer front panel: flat rectangle with horizontal rounded-end slot.
    Centred at (0, -DRAWER_FRONT_T/2, 0) so the outer face is at local y=0."""
    panel = (
        cq.Workplane("XY")
        .box(DRAWER_W, DRAWER_FRONT_T, DRAWER_H)
        .translate((0.0, -DRAWER_FRONT_T / 2.0, 0.0))
    )
    cutter = (
        cq.Workplane("XZ")
        .slot2D(SLOT_LEN, SLOT_W, 0)
        .extrude(0.05, both=True)
        .translate((0.0, -DRAWER_FRONT_T / 2.0, SLOT_Z_LOCAL))
    )
    leaf = panel.cut(cutter)
    return mesh_from_cadquery(leaf, mesh_name)


def _drawer_box_mesh(mesh_name: str):
    """Open-top drawer box tray. Front face at local y=0, extends in -Y.
    Centred vertically at z=0."""
    outer = (
        cq.Workplane("XY")
        .box(DRAWER_BOX_W, DRAWER_D, DRAWER_H)
        .translate((0.0, -DRAWER_D / 2.0, 0.0))
    )
    inner_w = DRAWER_BOX_W - 2 * DRAWER_SIDE_T
    inner_d = DRAWER_D - DRAWER_SIDE_T
    inner_h = DRAWER_H - DRAWER_BOTTOM_T
    cavity = (
        cq.Workplane("XY")
        .box(inner_w, inner_d, inner_h)
        .translate((0.0, -inner_d / 2.0 - DRAWER_SIDE_T / 2.0, DRAWER_BOTTOM_T / 2.0 - DRAWER_H / 2.0 + inner_h / 2.0))
    )
    # Recalculate cavity position: bottom of cavity should be at z = -DRAWER_H/2 + DRAWER_BOTTOM_T
    # bottom of cavity: z_bot_cav = -DRAWER_H/2 + DRAWER_BOTTOM_T
    # top of cavity: z_top_cav = DRAWER_H/2 (open top)
    # centre of cavity: z_c = (z_bot_cav + z_top_cav) / 2 = DRAWER_BOTTOM_T / 2
    z_cav = DRAWER_BOTTOM_T / 2.0
    cavity = (
        cq.Workplane("XY")
        .box(inner_w, inner_d, inner_h)
        .translate((0.0, -(DRAWER_SIDE_T + inner_d) / 2.0, z_cav))
    )
    box = outer.cut(cavity)
    return mesh_from_cadquery(box, mesh_name)


def _leg_mesh(mesh_name: str):
    """Splayed tapered leg."""
    leg = (
        cq.Workplane("XY")
        .center(0.02, 0.02)
        .rect(LEG_FOOT, LEG_FOOT)
        .workplane(offset=LEG_H + 0.008)
        .center(-0.02, -0.02)
        .rect(LEG_TOP_SIZE, LEG_TOP_SIZE)
        .loft()
    )
    return mesh_from_cadquery(leg, mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_steel_dresser_cabinet")

    steel_body = model.material("steel_body", rgba=(0.58, 0.59, 0.61, 1.0))
    steel_drawer = model.material("steel_drawer", rgba=(0.52, 0.53, 0.56, 1.0))
    steel_drawer_alt = model.material("steel_drawer_alt", rgba=(0.48, 0.49, 0.52, 1.0))
    steel_trim = model.material("steel_trim", rgba=(0.44, 0.45, 0.47, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.06, 0.06, 0.07, 1.0))
    steel_leg = model.material("steel_leg", rgba=(0.36, 0.37, 0.39, 1.0))
    steel_handle = model.material("steel_handle", rgba=(0.25, 0.26, 0.28, 1.0))
    steel_rivet = model.material("steel_rivet", rgba=(0.50, 0.51, 0.53, 1.0))
    steel_shelf = model.material("steel_shelf", rgba=(0.54, 0.55, 0.57, 1.0))
    steel_latch = model.material("steel_latch", rgba=(0.16, 0.16, 0.18, 1.0))

    # ------------------------------------------------------------------
    # Dresser body: hollow carcass + legs + rails + cap + shelves
    # ------------------------------------------------------------------
    body = model.part("dresser_body")

    # Side walls
    for sx, vname in ((-1.0, "side_wall_0"), (1.0, "side_wall_1")):
        body.visual(
            Box((WALL_T, DRS_D, CARCASS_H)),
            origin=Origin(xyz=(sx * (DRS_W / 2.0 - WALL_T / 2.0), 0.0, CARCASS_ZC)),
            material=steel_body,
            name=vname,
        )
    # Back wall
    body.visual(
        Box((DRS_W - WALL_T, WALL_T, CARCASS_H - 0.02)),
        origin=Origin(xyz=(0.0, BACK_Y + WALL_T / 2.0, CARCASS_ZC)),
        material=steel_body,
        name="back_wall",
    )
    # Bottom panel
    body.visual(
        Box((DRS_W, DRS_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, LEG_H + WALL_T / 2.0)),
        material=steel_body,
        name="bottom_panel",
    )
    # Top panel
    body.visual(
        Box((DRS_W, DRS_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, DRS_TOP - WALL_T / 2.0)),
        material=steel_body,
        name="top_panel",
    )

    # Interior shelf boards (between drawer bays, visible through openings)
    shelf_w = DRS_W - 2 * WALL_T + 0.005
    shelf_d = DRS_D - 2 * WALL_T - 0.01
    for i in range(N_DRAWERS - 1):
        shelf_z = DRAWER_Z_BOTS[i + 1] - DRAWER_GAP / 2.0
        body.visual(
            Box((shelf_w, shelf_d, SHELF_T)),
            origin=Origin(xyz=(0.0, -0.01, shelf_z)),
            material=steel_shelf,
            name=f"shelf_{i}",
        )

    # Front frame rails
    body.visual(
        Box((DRS_W - 2 * WALL_T, WALL_T, BOTTOM_RAIL_H)),
        origin=Origin(xyz=(0.0, FRONT_Y - WALL_T / 2.0, LEG_H + BOTTOM_RAIL_H / 2.0)),
        material=steel_body,
        name="front_bottom_rail",
    )
    body.visual(
        Box((DRS_W - 2 * WALL_T, WALL_T, TOP_RAIL_H)),
        origin=Origin(xyz=(0.0, FRONT_Y - WALL_T / 2.0, DRS_TOP - TOP_RAIL_H / 2.0)),
        material=steel_body,
        name="front_top_rail",
    )

    # Drawer guide rails (thin strips on inner side walls for each drawer)
    for i in range(N_DRAWERS):
        z_bot = DRAWER_Z_BOTS[i]
        for sx, side in ((-1.0, "L"), (1.0, "R")):
            rail_x = sx * (DRS_W / 2.0 - WALL_T - 0.006)
            body.visual(
                Box((0.010, DRS_D - 0.08, 0.010)),
                origin=Origin(xyz=(rail_x, -0.02, z_bot + 0.005)),
                material=steel_trim,
                name=f"guide_{i}_{side}",
            )

    # Top cap
    body.visual(
        Box((DRS_W + 2 * CAP_OVERHANG, DRS_D + 2 * CAP_OVERHANG, CAP_T)),
        origin=Origin(xyz=(0.0, 0.0, DRS_TOP - 0.001 + CAP_T / 2.0)),
        material=steel_trim,
        name="top_cap",
    )

    # Rivet dots along top rail
    n_riv = 11
    for i in range(n_riv):
        rx = -0.50 + i * (1.00 / (n_riv - 1))
        body.visual(
            Sphere(radius=0.004),
            origin=Origin(xyz=(rx, FRONT_Y + 0.002, DRS_TOP - TOP_RAIL_H / 2.0)),
            material=steel_rivet,
            name=f"rivet_{i}",
        )

    # Legs
    leg_mesh = _leg_mesh("dresser_leg")
    lx = DRS_W / 2.0 - 0.06
    ly = DRS_D / 2.0 - 0.06
    leg_corners = [
        (lx, ly, 0.0),
        (-lx, ly, math.pi / 2.0),
        (-lx, -ly, math.pi),
        (lx, -ly, 3.0 * math.pi / 2.0),
    ]
    for i, (px, py, yaw) in enumerate(leg_corners):
        body.visual(
            leg_mesh,
            origin=Origin(xyz=(px, py, 0.0), rpy=(0.0, 0.0, yaw)),
            material=steel_leg,
            name=f"leg_{i}",
        )

    # ------------------------------------------------------------------
    # Three drawers. Each slides out on a PRISMATIC joint along +Y.
    # Part frame: at the articulation origin (0, FRONT_Y, zc_world_i).
    # In part-local coords at q=0: y=0 is the front face, z=0 is the
    # drawer vertical centre, x=0 is horizontal centre.
    # ------------------------------------------------------------------
    drawers = []
    drawer_mats = [steel_drawer, steel_drawer_alt, steel_drawer]

    for i in range(N_DRAWERS):
        drawer = model.part(f"drawer_{i}")
        zc_w = DRAWER_Z_CENTRES[i]

        # Front panel (outer face at local y=0)
        drawer.visual(
            _drawer_front_mesh(f"drawer_front_{i}"),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=drawer_mats[i],
            name="front_panel",
        )

        # Dark backing behind vent slot
        drawer.visual(
            Box((SLOT_LEN + 0.03, 0.004, SLOT_W + 0.012)),
            origin=Origin(xyz=(0.0, -DRAWER_FRONT_T - 0.002, SLOT_Z_LOCAL)),
            material=steel_dark,
            name="vent_backing",
        )

        # Drawer box (open-top tray behind front panel)
        drawer.visual(
            _drawer_box_mesh(f"drawer_box_{i}"),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=drawer_mats[i],
            name="box",
        )

        # Bail-pull handle protruding from front face (above centre to clear latch)
        handle_z = 0.055
        post_x_off = HANDLE_W / 2.0 - 0.015
        for hx, sfx in ((-post_x_off, "L"), (post_x_off, "R")):
            drawer.visual(
                Cylinder(radius=0.004, length=HANDLE_D),
                origin=Origin(
                    xyz=(hx, HANDLE_D / 2.0, handle_z),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=steel_handle,
                name=f"handle_post_{sfx}",
            )
        drawer.visual(
            Box((HANDLE_W - 0.02, 0.008, HANDLE_H)),
            origin=Origin(xyz=(0.0, HANDLE_D, handle_z)),
            material=steel_handle,
            name="handle_bar",
        )

        # Prismatic articulation: slides along +Y
        model.articulation(
            f"drawer_{i}_slide",
            ArticulationType.PRISMATIC,
            parent=body,
            child=drawer,
            origin=Origin(xyz=(0.0, FRONT_Y, zc_w)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=30.0, velocity=0.5, lower=0.0, upper=DRAWER_PULL
            ),
        )
        drawers.append(drawer)

    # ------------------------------------------------------------------
    # Centre latch on the middle drawer - rotating turn-latch
    # Part frame at articulation origin (0, FRONT_Y, zc_world_1).
    # In latch-local: y=0 at front face, z=0 at mid height.
    # ------------------------------------------------------------------
    latch = model.part("center_latch")

    # Base plate
    latch.visual(
        Cylinder(radius=LATCH_BASE_R, length=LATCH_BASE_H),
        origin=Origin(
            xyz=(0.0, LATCH_BASE_H / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel_latch,
        name="base_plate",
    )
    # Boss
    latch.visual(
        Cylinder(radius=LATCH_BOSS_R, length=LATCH_BOSS_H),
        origin=Origin(
            xyz=(0.0, LATCH_BASE_H + LATCH_BOSS_H / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel_latch,
        name="boss",
    )
    # Turn bar
    latch.visual(
        Box((LATCH_BAR_W, LATCH_BAR_D, LATCH_BAR_H)),
        origin=Origin(
            xyz=(0.0, LATCH_BASE_H + LATCH_BOSS_H + LATCH_BAR_D / 2.0, 0.0),
        ),
        material=steel_latch,
        name="turn_bar",
    )
    # Tip sphere at the end of the bar
    latch.visual(
        Sphere(radius=0.005),
        origin=Origin(
            xyz=(0.0, LATCH_BASE_H + LATCH_BOSS_H + LATCH_BAR_D / 2.0, LATCH_BAR_H / 2.0),
        ),
        material=steel_latch,
        name="bar_tip",
    )

    model.articulation(
        "latch_turn",
        ArticulationType.REVOLUTE,
        parent=drawers[1],
        child=latch,
        # Origin in parent (drawer_1) frame; drawer_1 frame is already at
        # (0, FRONT_Y, DRAWER_Z_CENTRES[1]) in world at q=0.
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=4.0, lower=0.0, upper=LATCH_TURN
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("dresser_body")
    drawers = [object_model.get_part(f"drawer_{i}") for i in range(N_DRAWERS)]
    slides = [object_model.get_articulation(f"drawer_{i}_slide") for i in range(N_DRAWERS)]
    latch_part = object_model.get_part("center_latch")
    latch_turn = object_model.get_articulation("latch_turn")

    # --- Dresser proportions: low and wide --------------------------------
    aabb = ctx.part_world_aabb(body)
    ok = aabb is not None
    ctx.check("dresser body has bounds", ok, details=str(aabb))
    if aabb is not None:
        (x0, y0, z0), (x1, y1, z1) = aabb
        ctx.check(
            "dresser width ~1.2 m",
            1.15 <= (x1 - x0) <= 1.30,
            details=f"width={x1 - x0:.3f}",
        )
        ctx.check(
            "dresser depth ~0.5 m",
            0.46 <= (y1 - y0) <= 0.58,
            details=f"depth={y1 - y0:.3f}",
        )
        ctx.check(
            "dresser height ~0.85 m (low profile)",
            0.80 <= z1 <= 0.92,
            details=f"top={z1:.3f}",
        )
        ctx.check("legs rest on the floor", abs(z0) <= 1e-6, details=f"zmin={z0:.5f}")
        ctx.check(
            "dresser is wider than tall (low wide profile)",
            (x1 - x0) > (z1 - z0),
            details=f"w={x1-x0:.3f} h={z1-z0:.3f}",
        )

    # --- Three drawers with prismatic joints -----------------------------
    ctx.check(
        "exactly three drawers exist",
        len(drawers) == 3,
    )

    for i, (drawer, slide) in enumerate(zip(drawers, slides)):
        ctx.check(
            f"drawer_{i} slide is prismatic",
            slide.articulation_type == ArticulationType.PRISMATIC,
        )
        ax = slide.axis
        ctx.check(
            f"drawer_{i} slide axis along Y",
            abs(ax[0]) < 1e-9 and abs(abs(ax[1]) - 1.0) < 1e-9 and abs(ax[2]) < 1e-9,
            details=str(ax),
        )
        lim = slide.motion_limits
        ctx.check(
            f"drawer_{i} slide range 0..0.35 m",
            lim is not None and lim.lower == 0.0 and abs(lim.upper - DRAWER_PULL) < 1e-6,
        )

    # Drawer stacking order
    for i in range(N_DRAWERS - 1):
        aabb_lo = ctx.part_world_aabb(drawers[i])
        aabb_hi = ctx.part_world_aabb(drawers[i + 1])
        ctx.check(
            f"drawer_{i+1} is above drawer_{i}",
            aabb_lo is not None
            and aabb_hi is not None
            and aabb_hi[0][2] > aabb_lo[0][2] + 0.05,
        )

    # Drawer actually extends outward when pulled
    closed_aabb = ctx.part_world_aabb(drawers[0])
    with ctx.pose({slides[0]: DRAWER_PULL}):
        open_aabb = ctx.part_world_aabb(drawers[0])
    ctx.check(
        "drawer_0 extends outward (+Y) when pulled",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.20,
        details=f"closed_max_y={closed_aabb[1][1]:.3f}, open_max_y={open_aabb[1][1]:.3f}",
    )

    # --- Shelf boards visible through opening ------------------------------
    shelf0_aabb = ctx.part_element_world_aabb(body, elem="shelf_0")
    shelf1_aabb = ctx.part_element_world_aabb(body, elem="shelf_1")
    ctx.check(
        "interior shelf boards exist",
        shelf0_aabb is not None and shelf1_aabb is not None,
    )

    # When middle drawer is open, shelf_0 z is within the gap left by drawer_1
    with ctx.pose({slides[1]: DRAWER_PULL}):
        d1_aabb = ctx.part_world_aabb(drawers[1])
    ctx.check(
        "shelf_0 is in the gap exposed when drawer_1 is pulled",
        shelf0_aabb is not None
        and d1_aabb is not None
        # shelf should be near where drawer_1 used to be
        and shelf0_aabb[0][2] > d1_aabb[0][2] - 0.25
        and shelf0_aabb[1][2] < d1_aabb[0][2] + 0.05,
        details=f"shelf0_z=[{shelf0_aabb[0][2]:.3f},{shelf0_aabb[1][2]:.3f}], "
                f"d1_open_zmin={d1_aabb[0][2]:.3f}",
    )

    # --- Centre latch: rotating turn-latch --------------------------------
    ctx.check(
        "latch_turn is revolute",
        latch_turn.articulation_type == ArticulationType.REVOLUTE,
    )
    ctx.check(
        "latch_turn axis along Y",
        latch_turn.axis == (0.0, 1.0, 0.0),
        details=str(latch_turn.axis),
    )
    lim = latch_turn.motion_limits
    ctx.check(
        "latch_turn range 0..90 deg",
        lim is not None
        and lim.lower == 0.0
        and abs(lim.upper - math.pi / 2.0) < 1e-6,
    )

    # Latch parent is the middle drawer
    mid_name = drawers[1].name if hasattr(drawers[1], 'name') else "drawer_1"
    latch_parent = latch_turn.parent
    parent_name = latch_parent.name if hasattr(latch_parent, 'name') else str(latch_parent)
    ctx.check(
        "latch is mounted on drawer_1 (middle drawer)",
        parent_name == mid_name or parent_name == "drawer_1",
        details=f"parent={parent_name}",
    )

    # Turning the latch sweeps the bar tip
    tip_rest = ctx.part_element_world_aabb(latch_part, elem="bar_tip")
    with ctx.pose({latch_turn: LATCH_TURN}):
        tip_turned = ctx.part_element_world_aabb(latch_part, elem="bar_tip")
    ctx.check(
        "turning latch sweeps bar tip to a new position",
        tip_rest is not None
        and tip_turned is not None
        and (
            abs(tip_turned[0][0] - tip_rest[0][0]) > 0.010
            or abs(tip_turned[0][2] - tip_rest[0][2]) > 0.010
        ),
        details=f"rest={tip_rest}, turned={tip_turned}",
    )

    # --- Vent slots on drawer fronts ------------------------------------
    for i in range(N_DRAWERS):
        vb = ctx.part_element_world_aabb(drawers[i], elem="vent_backing")
        ctx.check(
            f"drawer_{i} vent backing in the lower portion",
            vb is not None
            and vb[1][2] < DRAWER_Z_CENTRES[i]
            and vb[0][2] > DRAWER_Z_BOTS[i] - 0.01,
            details=str(vb),
        )

    # Riveted top cap
    rivet_aabb = ctx.part_element_world_aabb(body, elem="rivet_0")
    ctx.check(
        "rivet dots present on top rail",
        rivet_aabb is not None
        and rivet_aabb[1][1] > FRONT_Y + 0.002
        and rivet_aabb[0][2] > DRS_TOP - TOP_RAIL_H - 0.01,
    )

    return ctx.report()


object_model = build_object_model()
