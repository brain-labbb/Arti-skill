from __future__ import annotations

"""Low wide vintage industrial steel dresser cabinet with three drawers and a
tambour sliding front.

Variant 02 of the vintage industrial steel locker family. This is a low wide
dresser cabinet (~1.50 m wide x 0.50 m deep x ~0.85 m tall) in brushed/tarnished
raw steel. The body is a hollow rectangular carcass on four short splayed legs.

The front is divided into two zones:
- Upper zone: an open compartment with interior shelf boards, fronted by a
  tambour panel that slides sideways (prismatic along X) to reveal the shelves.
- Lower zone: three horizontal drawers stacked vertically, each a flat steel
  panel with recessed panel borders and a small bar pull handle, sliding on
  prismatic joints along Y (pull-out).

Wall thickness ~0.02 m. Riveted top cap strip along the top rail.
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
# Global dimensions (meters). Cabinet centred on X, front face at +Y.
# ---------------------------------------------------------------------------
CAB_W = 1.50  # overall carcass width (X)
CAB_D = 0.50  # overall carcass depth (Y)
CAB_TOP = 0.85  # carcass top height (Z)
LEG_H = 0.12  # leg height; carcass starts here
WALL_T = 0.02  # thin steel wall thickness

FRONT_Y = CAB_D / 2.0  # +0.25
BACK_Y = -CAB_D / 2.0

# Body interior extents
BODY_BOT = LEG_H  # 0.12
BODY_H = CAB_TOP - LEG_H  # 0.73

# Horizontal divider between upper (tambour) and lower (drawer) zones
DIVIDER_Z = 0.55
DIVIDER_T = 0.015

# Drawer zone: three drawers below the divider
DRAWER_ZONE_BOT = BODY_BOT + WALL_T  # 0.14
DRAWER_ZONE_TOP = DIVIDER_Z - DIVIDER_T / 2.0  # ~0.5425
DRAWER_ZONE_H = DRAWER_ZONE_TOP - DRAWER_ZONE_BOT  # ~0.40

N_DRAWERS = 3
DRAWER_GAP = 0.005  # vertical gap between drawer fronts
DRAWER_FRONT_H = (DRAWER_ZONE_H - (N_DRAWERS - 1) * DRAWER_GAP) / N_DRAWERS  # ~0.13
DRAWER_FRONT_T = WALL_T  # 0.02
DRAWER_BOX_DEPTH = 0.38  # drawer box extends back from front face
DRAWER_BOX_WALL_T = 0.010  # side/back wall thickness of drawer box
DRAWER_BOX_BOTTOM_T = 0.008

# Tambour zone: above the divider
TAMBOUR_ZONE_BOT = DIVIDER_Z + DIVIDER_T / 2.0  # ~0.5575
TAMBOUR_ZONE_TOP = CAB_TOP - WALL_T  # 0.83
TAMBOUR_ZONE_H = TAMBOUR_ZONE_TOP - TAMBOUR_ZONE_BOT  # ~0.27
TAMBOUR_T = 0.012  # tambour panel thickness

# Tambour design: a sliding panel that covers the left portion of the upper
# opening and slides right behind a fixed right panel section.
# Fixed right panel (part of body): ~0.45m wide on the right of the opening.
FIXED_PANEL_W = 0.45
TAMBOUR_OPEN_W = CAB_W - 2.0 * WALL_T - FIXED_PANEL_W  # sliding panel width ~1.01m
TAMBOUR_PANEL_W = TAMBOUR_OPEN_W  # matches the open portion exactly
TAMBOUR_SLIDE = FIXED_PANEL_W  # slides right behind the fixed panel
# Tambour center X at rest: centered on the open portion (left side)
TAMBOUR_REST_X = -(CAB_W / 2.0 - WALL_T) + TAMBOUR_PANEL_W / 2.0  # left-aligned

# Recessed border on drawer fronts
RECESS_BORDER = 0.018  # width of the recessed border frame
RECESS_DEPTH = 0.004  # depth of the recess

# Interior shelves in upper compartment
SHELF_T = 0.012

# Top cap
CAP_T = 0.020
CAP_OVERHANG = 0.015

# Leg dimensions
LEG_FOOT_SIZE = 0.035
LEG_TOP_SIZE = 0.055


def _drawer_front_solid(mesh_name: str):
    """Drawer front panel with recessed rectangular border (stamped inset)."""
    w = CAB_W - 2.0 * WALL_T - 0.010  # slightly narrower than interior
    h = DRAWER_FRONT_H
    t = DRAWER_FRONT_T

    # Main panel
    panel = cq.Workplane("XY").box(w, t, h)

    # Cut recessed border: a frame-shaped cut on the front face
    outer_w = w - 2.0 * 0.012
    outer_h = h - 2.0 * 0.012
    inner_w = outer_w - 2.0 * RECESS_BORDER
    inner_h = outer_h - 2.0 * RECESS_BORDER

    # Outer recess rectangle
    recess_outer = (
        cq.Workplane("XY")
        .box(outer_w, RECESS_DEPTH, outer_h)
        .translate((0.0, t / 2.0 - RECESS_DEPTH / 2.0 + 0.001, 0.0))
    )
    # Inner raised area (we subtract outer, then add back inner to get a frame)
    recess_inner = (
        cq.Workplane("XY")
        .box(inner_w, RECESS_DEPTH, inner_h)
        .translate((0.0, t / 2.0 - RECESS_DEPTH / 2.0 + 0.001, 0.0))
    )
    # The frame is outer minus inner: cut outer, union inner back
    result = panel.cut(recess_outer).union(recess_inner)

    return mesh_from_cadquery(result, mesh_name)


def _drawer_box_solid(mesh_name: str):
    """Drawer box: open-top tray with thin walls and bottom."""
    w = CAB_W - 2.0 * WALL_T - 0.020  # slightly narrower than interior
    d = DRAWER_BOX_DEPTH
    h = DRAWER_FRONT_H - 0.010  # slightly shorter than front

    # Outer box
    outer = cq.Workplane("XY").box(w, d, h)
    # Inner cavity (open top)
    inner = (
        cq.Workplane("XY")
        .box(w - 2.0 * DRAWER_BOX_WALL_T, d - DRAWER_BOX_WALL_T, h - DRAWER_BOX_BOTTOM_T)
        .translate((0.0, -DRAWER_BOX_WALL_T / 2.0, DRAWER_BOX_BOTTOM_T / 2.0 + 0.001))
    )
    result = outer.cut(inner)
    return mesh_from_cadquery(result, mesh_name)


def _handle_solid(mesh_name: str):
    """Drawer pull handle: horizontal bar with two mounting posts as one solid."""
    bar_r = 0.005
    bar_len = 0.10
    post_r = 0.004
    post_len = 0.016
    post_spacing = 0.04
    # Horizontal bar along X
    bar = (
        cq.Workplane("YZ")
        .circle(bar_r)
        .extrude(bar_len / 2.0, both=True)
    )
    # Two mounting posts along Y (below the bar)
    for px in (-post_spacing, post_spacing):
        post = (
            cq.Workplane("XZ")
            .workplane(offset=0.0)
            .center(px, 0.0)
            .circle(post_r)
            .extrude(post_len)
            .translate((0.0, -post_len / 2.0, 0.0))
        )
        bar = bar.union(post)
    return mesh_from_cadquery(bar, mesh_name)


def _tambour_panel(mesh_name: str):
    """Tambour sliding panel: flat steel sheet with a grip groove."""
    w = TAMBOUR_PANEL_W
    h = TAMBOUR_ZONE_H - 0.004
    t = TAMBOUR_T
    panel = cq.Workplane("XY").box(w, t, h)
    # Add a small horizontal grip groove near center-bottom
    groove = (
        cq.Workplane("XY")
        .box(0.12, t + 0.01, 0.012)
        .translate((0.0, 0.0, -h / 2.0 + 0.025))
    )
    result = panel.cut(groove)
    return mesh_from_cadquery(result, mesh_name)


def _leg_solid(mesh_name: str):
    """Splayed tapered leg."""
    leg = (
        cq.Workplane("XY")
        .center(0.025, 0.025)
        .rect(LEG_FOOT_SIZE, LEG_FOOT_SIZE)
        .workplane(offset=LEG_H + 0.01)
        .center(-0.025, -0.025)
        .rect(LEG_TOP_SIZE, LEG_TOP_SIZE)
        .loft()
    )
    return mesh_from_cadquery(leg, mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_steel_dresser_cabinet")

    # Materials
    steel_body = model.material("steel_body", rgba=(0.58, 0.59, 0.61, 1.0))
    steel_drawer = model.material("steel_drawer", rgba=(0.52, 0.53, 0.56, 1.0))
    steel_tambour = model.material("steel_tambour", rgba=(0.50, 0.51, 0.53, 1.0))
    steel_trim = model.material("steel_trim", rgba=(0.44, 0.45, 0.47, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    steel_leg = model.material("steel_leg", rgba=(0.36, 0.37, 0.39, 1.0))
    steel_handle = model.material("steel_handle", rgba=(0.22, 0.22, 0.24, 1.0))
    steel_rivet = model.material("steel_rivet", rgba=(0.50, 0.51, 0.53, 1.0))
    steel_shelf = model.material("steel_shelf", rgba=(0.48, 0.49, 0.51, 1.0))
    steel_recess = model.material("steel_recess", rgba=(0.35, 0.36, 0.38, 1.0))

    # ==================================================================
    # Cabinet body (root)
    # ==================================================================
    body = model.part("cabinet_body")

    # Side walls
    for sx, vname in ((-1.0, "side_wall_0"), (1.0, "side_wall_1")):
        body.visual(
            Box((WALL_T, CAB_D, BODY_H)),
            origin=Origin(xyz=(sx * (CAB_W / 2.0 - WALL_T / 2.0), 0.0, BODY_BOT + BODY_H / 2.0)),
            material=steel_body,
            name=vname,
        )

    # Back wall
    body.visual(
        Box((CAB_W - WALL_T, WALL_T, BODY_H - 0.02)),
        origin=Origin(xyz=(0.0, BACK_Y + WALL_T / 2.0, BODY_BOT + BODY_H / 2.0)),
        material=steel_body,
        name="back_wall",
    )

    # Bottom panel
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, BODY_BOT + WALL_T / 2.0)),
        material=steel_body,
        name="bottom_panel",
    )

    # Top panel
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - WALL_T / 2.0)),
        material=steel_body,
        name="top_panel",
    )

    # Horizontal divider between upper and lower zones
    body.visual(
        Box((CAB_W - 2.0 * WALL_T + 0.01, CAB_D - WALL_T, DIVIDER_T)),
        origin=Origin(xyz=(0.0, -WALL_T / 2.0, DIVIDER_Z)),
        material=steel_body,
        name="zone_divider",
    )

    # Front frame: bottom rail (below drawers)
    bottom_rail_h = DRAWER_ZONE_BOT - BODY_BOT  # 0.02
    body.visual(
        Box((CAB_W - 2.0 * WALL_T, WALL_T, bottom_rail_h)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, BODY_BOT + bottom_rail_h / 2.0)
        ),
        material=steel_body,
        name="front_bottom_rail",
    )

    # Front frame: top rail (above tambour opening)
    top_rail_h = CAB_TOP - TAMBOUR_ZONE_TOP  # ~0.02
    body.visual(
        Box((CAB_W - 2.0 * WALL_T, WALL_T, top_rail_h + 0.005)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, TAMBOUR_ZONE_TOP + (top_rail_h + 0.005) / 2.0 - 0.002)
        ),
        material=steel_body,
        name="front_top_rail",
    )

    # Fixed front panel on the right side of the upper opening (tambour stacks behind this)
    fixed_panel_x = CAB_W / 2.0 - WALL_T - FIXED_PANEL_W / 2.0
    body.visual(
        Box((FIXED_PANEL_W, WALL_T, TAMBOUR_ZONE_H + 0.01)),
        origin=Origin(
            xyz=(fixed_panel_x, FRONT_Y - WALL_T / 2.0, TAMBOUR_ZONE_BOT + TAMBOUR_ZONE_H / 2.0)
        ),
        material=steel_body,
        name="fixed_right_panel",
    )

    # Tambour guide rail (thin strip along top and bottom of opening)
    rail_w = CAB_W - 2.0 * WALL_T + 0.01
    for dz, name in (
        (TAMBOUR_ZONE_BOT + 0.003, "tambour_rail_bottom"),
        (TAMBOUR_ZONE_TOP - 0.003, "tambour_rail_top"),
    ):
        body.visual(
            Box((rail_w, 0.015, 0.005)),
            origin=Origin(xyz=(0.0, FRONT_Y - 0.010, dz)),
            material=steel_trim,
            name=name,
        )

    # Interior shelves in upper compartment (visible through open tambour)
    shelf_w = CAB_W - 2.0 * WALL_T - 0.01
    shelf_d = CAB_D - 2.0 * WALL_T - 0.02
    shelf_z = TAMBOUR_ZONE_BOT + TAMBOUR_ZONE_H * 0.50
    body.visual(
        Box((shelf_w, shelf_d, SHELF_T)),
        origin=Origin(xyz=(0.0, -WALL_T, shelf_z)),
        material=steel_shelf,
        name="interior_shelf_0",
    )
    # Second shelf higher up
    shelf_z2 = TAMBOUR_ZONE_BOT + TAMBOUR_ZONE_H * 0.82
    body.visual(
        Box((shelf_w, shelf_d, SHELF_T)),
        origin=Origin(xyz=(0.0, -WALL_T, shelf_z2)),
        material=steel_shelf,
        name="interior_shelf_1",
    )

    # Drawer guide rails (thin strips on inner side walls for drawers to slide on)
    # Each rail touches the inner side wall face for connectivity and sits at the
    # bottom of the drawer box to support sliding.
    for i in range(N_DRAWERS):
        dz = DRAWER_ZONE_BOT + i * (DRAWER_FRONT_H + DRAWER_GAP) + DRAWER_FRONT_H / 2.0
        rail_z = dz - DRAWER_FRONT_H / 2.0 + 0.010  # near box bottom
        for sx in (-1.0, 1.0):
            body.visual(
                Box((0.010, DRAWER_BOX_DEPTH + 0.02, 0.008)),
                origin=Origin(
                    xyz=(
                        sx * (CAB_W / 2.0 - WALL_T - 0.002),
                        FRONT_Y - DRAWER_BOX_DEPTH / 2.0 - 0.01,
                        rail_z,
                    )
                ),
                material=steel_trim,
                name=f"drawer_rail_{i}_{'L' if sx < 0 else 'R'}",
            )

    # Top cap strip with overhang
    body.visual(
        Box((CAB_W + 2 * CAP_OVERHANG, CAB_D + 2 * CAP_OVERHANG, CAP_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - 0.001 + CAP_T / 2.0)),
        material=steel_trim,
        name="top_cap",
    )

    # Rivet dots along the top cap front edge
    n_riv = 11
    for i in range(n_riv):
        rx = -0.65 + i * (1.30 / (n_riv - 1))
        body.visual(
            Sphere(radius=0.004),
            origin=Origin(xyz=(rx, FRONT_Y + 0.002, CAB_TOP + CAP_T - 0.003)),
            material=steel_rivet,
            name=f"rivet_{i}",
        )

    # Splayed legs
    leg_mesh = _leg_solid("splayed_leg")
    leg_corners = [
        (CAB_W / 2.0 - 0.06, CAB_D / 2.0 - 0.06, 0.0),
        (-(CAB_W / 2.0 - 0.06), CAB_D / 2.0 - 0.06, math.pi / 2.0),
        (-(CAB_W / 2.0 - 0.06), -(CAB_D / 2.0 - 0.06), math.pi),
        (CAB_W / 2.0 - 0.06, -(CAB_D / 2.0 - 0.06), 3.0 * math.pi / 2.0),
    ]
    for i, (lx, ly, yaw) in enumerate(leg_corners):
        body.visual(
            leg_mesh,
            origin=Origin(xyz=(lx, ly, 0.0), rpy=(0.0, 0.0, yaw)),
            material=steel_leg,
            name=f"leg_{i}",
        )

    # ==================================================================
    # Three drawers
    # ==================================================================
    drawer_front_mesh_name = "drawer_front_panel"
    drawer_box_mesh_name = "drawer_box_tray"
    drawer_front_mesh = _drawer_front_solid(drawer_front_mesh_name)
    drawer_box_mesh = _drawer_box_solid(drawer_box_mesh_name)

    drawers = []
    for i in range(N_DRAWERS):
        drawer = model.part(f"drawer_{i}")

        # Drawer front panel - visuals in part-local frame where z=0 is the
        # drawer center (the joint origin already places the part at the
        # correct world z).
        drawer_zc = DRAWER_ZONE_BOT + i * (DRAWER_FRONT_H + DRAWER_GAP) + DRAWER_FRONT_H / 2.0
        front_y = FRONT_Y - DRAWER_FRONT_T / 2.0

        drawer.visual(
            drawer_front_mesh,
            origin=Origin(xyz=(0.0, front_y, 0.0)),
            material=steel_drawer,
            name="front_panel",
        )

        # Recessed border overlay (darker inset to show the recess)
        drawer.visual(
            Box((CAB_W - 2.0 * WALL_T - 0.010 - 2.0 * 0.012, 0.002, DRAWER_FRONT_H - 2.0 * 0.012)),
            origin=Origin(xyz=(0.0, front_y + DRAWER_FRONT_T / 2.0 + 0.001, 0.0)),
            material=steel_recess,
            name="recess_border",
        )

        # Drawer box (tray behind the front panel)
        box_y = FRONT_Y - DRAWER_FRONT_T - DRAWER_BOX_DEPTH / 2.0
        drawer.visual(
            drawer_box_mesh,
            origin=Origin(xyz=(0.0, box_y, 0.0)),
            material=steel_body,
            name="box_tray",
        )

        # Pull handle: combined bar + mounting posts as one CadQuery solid
        # The handle posts penetrate into the front panel for mesh connectivity
        handle_mesh = _handle_solid(f"handle_{i}")
        drawer.visual(
            handle_mesh,
            origin=Origin(
                xyz=(0.0, front_y + DRAWER_FRONT_T / 2.0 + 0.003, 0.0),
            ),
            material=steel_handle,
            name="pull_handle",
        )

        # Prismatic articulation: slides along +Y (pulling out toward viewer)
        model.articulation(
            f"drawer_{i}_slide",
            ArticulationType.PRISMATIC,
            parent=body,
            child=drawer,
            origin=Origin(xyz=(0.0, 0.0, drawer_zc)),
            axis=(0.0, 1.0, 0.0),  # slides out toward +Y
            motion_limits=MotionLimits(
                effort=30.0, velocity=0.5, lower=0.0, upper=DRAWER_BOX_DEPTH - 0.05
            ),
        )
        drawers.append(drawer)

    # ==================================================================
    # Tambour sliding panel
    # ==================================================================
    tambour = model.part("tambour")
    tambour_zc = TAMBOUR_ZONE_BOT + TAMBOUR_ZONE_H / 2.0

    # All visuals in part-local frame (joint origin handles world placement)
    # Joint origin is at TAMBOUR_REST_X, so panel center is at local x=0
    tambour.visual(
        _tambour_panel("tambour_panel"),
        origin=Origin(xyz=(0.0, FRONT_Y - TAMBOUR_T / 2.0, 0.0)),
        material=steel_tambour,
        name="panel",
    )

    # Small grip tab on the left edge of the tambour (inset from the panel edge)
    tambour.visual(
        Box((0.025, 0.008, 0.05)),
        origin=Origin(
            xyz=(-TAMBOUR_PANEL_W / 2.0 + 0.025, FRONT_Y - 0.004, 0.0),
        ),
        material=steel_handle,
        name="grip_tab",
    )

    model.articulation(
        "tambour_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tambour,
        origin=Origin(xyz=(TAMBOUR_REST_X, 0.0, tambour_zc)),
        axis=(1.0, 0.0, 0.0),  # slides right (+X) to open
        motion_limits=MotionLimits(
            effort=20.0, velocity=0.4, lower=0.0, upper=TAMBOUR_SLIDE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
    drawers = [object_model.get_part(f"drawer_{i}") for i in range(N_DRAWERS)]
    drawer_joints = [object_model.get_articulation(f"drawer_{i}_slide") for i in range(N_DRAWERS)]
    tambour = object_model.get_part("tambour")
    tambour_joint = object_model.get_articulation("tambour_slide")

    # --- Overall envelope and scale ----------------------------------------
    aabb = ctx.part_world_aabb(body)
    ok = aabb is not None
    ctx.check("cabinet body has bounds", ok, details=str(aabb))
    if aabb is not None:
        (x0, y0, z0), (x1, y1, z1) = aabb
        ctx.check(
            "overall width ~1.5 m",
            1.48 <= (x1 - x0) <= 1.58,
            details=f"width={x1 - x0:.3f}",
        )
        ctx.check(
            "overall depth ~0.5 m",
            0.48 <= (y1 - y0) <= 0.58,
            details=f"depth={y1 - y0:.3f}",
        )
        ctx.check(
            "overall height ~0.85 m (low dresser)",
            0.83 <= z1 <= 0.92,
            details=f"top={z1:.3f}",
        )
        ctx.check(
            "legs rest on the floor",
            abs(z0) <= 1e-6,
            details=f"zmin={z0:.5f}",
        )

    # --- Drawer joints: prismatic, axis, limits ----------------------------
    for i, (drawer, joint) in enumerate(zip(drawers, drawer_joints)):
        ctx.check(
            f"drawer_{i}_slide is prismatic",
            joint.articulation_type == ArticulationType.PRISMATIC,
        )
        ax = joint.axis
        ctx.check(
            f"drawer_{i}_slide axis is along Y (pull-out direction)",
            abs(ax[0]) < 1e-9 and abs(abs(ax[1]) - 1.0) < 1e-9 and abs(ax[2]) < 1e-9,
            details=str(ax),
        )
        lim = joint.motion_limits
        ctx.check(
            f"drawer_{i}_slide range 0..~{DRAWER_BOX_DEPTH - 0.05:.2f} m",
            lim is not None and lim.lower == 0.0 and lim.upper > 0.25,
            details=f"upper={lim.upper:.3f}" if lim else "no limits",
        )

    # Drawer rest pose: fronts sit flush at front face
    for i, drawer in enumerate(drawers):
        daabb = ctx.part_element_world_aabb(drawer, elem="front_panel")
        ctx.check(
            f"drawer_{i} front panel sits at the cabinet front face when closed",
            daabb is not None and abs(daabb[1][1] - FRONT_Y) < 0.01,
            details=str(daabb),
        )

    # Drawer recessed borders exist
    for i, drawer in enumerate(drawers):
        raabb = ctx.part_element_world_aabb(drawer, elem="recess_border")
        ctx.check(
            f"drawer_{i} has recessed panel border visible on front",
            raabb is not None and raabb[1][1] > FRONT_Y - 0.005,
            details=str(raabb),
        )

    # Drawer pull-out test: drawer_0 moves forward (toward +Y)
    closed_aabb = ctx.part_world_aabb(drawers[0])
    with ctx.pose({drawer_joints[0]: 0.20}):
        open_aabb = ctx.part_world_aabb(drawers[0])
    ctx.check(
        "drawer_0 slides outward (toward +Y) when opened",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.15,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    # Drawer stays within cabinet width
    for i, drawer in enumerate(drawers):
        ctx.expect_within(
            drawer,
            body,
            axes="x",
            margin=0.02,
            name=f"drawer_{i} stays within cabinet width",
        )

    # --- Tambour joint: prismatic, axis, limits ---------------------------
    ctx.check(
        "tambour_slide is prismatic",
        tambour_joint.articulation_type == ArticulationType.PRISMATIC,
    )
    tax = tambour_joint.axis
    ctx.check(
        "tambour_slide axis is along X (sideways slide)",
        abs(abs(tax[0]) - 1.0) < 1e-9 and abs(tax[1]) < 1e-9 and abs(tax[2]) < 1e-9,
        details=str(tax),
    )
    tlim = tambour_joint.motion_limits
    ctx.check(
        "tambour_slide has positive travel range",
        tlim is not None and tlim.lower == 0.0 and tlim.upper > 0.30,
        details=f"upper={tlim.upper:.3f}" if tlim else "no limits",
    )

    # Tambour closed: panel covers the upper opening
    tp_closed = ctx.part_element_world_aabb(tambour, elem="panel")
    ctx.check(
        "tambour panel covers the upper opening when closed",
        tp_closed is not None
        and tp_closed[0][2] < TAMBOUR_ZONE_BOT + 0.03
        and tp_closed[1][2] > TAMBOUR_ZONE_TOP - 0.03,
        details=str(tp_closed),
    )

    # Tambour open: slides right, exposing the opening
    with ctx.pose({tambour_joint: TAMBOUR_SLIDE}):
        tp_open = ctx.part_element_world_aabb(tambour, elem="panel")
    ctx.check(
        "tambour slides right (positive X) when opened",
        tp_open is not None
        and tp_closed is not None
        and tp_open[0][0] > tp_closed[0][0] + 0.4,
        details=f"closed={tp_closed}, open={tp_open}",
    )

    # --- Interior shelves visible through the opening ----------------------
    shelf0_aabb = ctx.part_element_world_aabb(body, elem="interior_shelf_0")
    shelf1_aabb = ctx.part_element_world_aabb(body, elem="interior_shelf_1")
    ctx.check(
        "interior shelf 0 exists in the upper compartment",
        shelf0_aabb is not None
        and shelf0_aabb[0][2] > TAMBOUR_ZONE_BOT
        and shelf0_aabb[1][2] < TAMBOUR_ZONE_TOP,
        details=str(shelf0_aabb),
    )
    ctx.check(
        "interior shelf 1 exists above shelf 0",
        shelf1_aabb is not None
        and shelf0_aabb is not None
        and shelf1_aabb[0][2] > shelf0_aabb[1][2] - 0.01,
        details=f"shelf0={shelf0_aabb}, shelf1={shelf1_aabb}",
    )

    # --- Drawers stacked vertically ----------------------------------------
    drawer_centers = []
    for i in range(N_DRAWERS):
        daabb = ctx.part_element_world_aabb(drawers[i], elem="front_panel")
        if daabb is not None:
            drawer_centers.append(0.5 * (daabb[0][2] + daabb[1][2]))
    if len(drawer_centers) == N_DRAWERS:
        ctx.check(
            "drawers are stacked vertically (ascending Z order)",
            drawer_centers[0] < drawer_centers[1] < drawer_centers[2],
            details=str(drawer_centers),
        )
        ctx.check(
            "drawers span the lower zone of the cabinet",
            drawer_centers[0] < 0.30 and drawer_centers[2] > 0.40,
            details=str(drawer_centers),
        )

    # --- Non-fixed joint count ---------------------------------------------
    non_fixed = [
        a for a in object_model.articulations
        if a.articulation_type != ArticulationType.FIXED
    ]
    ctx.check(
        "at least 4 non-fixed joints (3 drawers + tambour)",
        len(non_fixed) >= 4,
        details=f"count={len(non_fixed)}",
    )

    return ctx.report()


object_model = build_object_model()
