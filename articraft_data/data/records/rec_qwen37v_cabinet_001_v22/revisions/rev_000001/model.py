from __future__ import annotations

"""Low wide vintage industrial steel dresser cabinet with three drawers and a hinged top lid.

Variant of the vintage steel locker cabinet family, reconfigured as a low wide
dresser. Overall envelope ~1.60 m wide x 0.50 m deep x ~0.90 m tall, brushed /
tarnished raw steel finish. The hollow thin-wall (~0.02 m) carcass sits on four
short splayed legs and carries three full-width horizontal drawers stacked
vertically with visible gap seams around each drawer front. A flat top lid
hinges upward on a rear revolute joint to reveal shelf boards inside the upper
compartment. Each drawer slides out on a prismatic joint along the front (+Y)
axis.
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
# Global dimensions (meters). Cabinet is centred on X, front face at +Y.
# ---------------------------------------------------------------------------
CAB_W = 1.60  # overall carcass width  (X)
CAB_D = 0.50  # overall carcass depth  (Y)
CAB_TOP = 0.90  # carcass top height   (Z)
LEG_H = 0.12  # short splayed legs
WALL_T = 0.02  # thin steel wall

FRONT_Y = CAB_D / 2.0  # +0.25
BACK_Y = -CAB_D / 2.0

CARCASS_H = CAB_TOP - LEG_H  # 0.78
CARCASS_ZC = LEG_H + CARCASS_H / 2.0

# Front frame rails
BOTTOM_RAIL_H = 0.04
TOP_RAIL_H = 0.04
BOTTOM_RAIL_TOP = LEG_H + BOTTOM_RAIL_H  # 0.16
TOP_RAIL_BOT = CAB_TOP - TOP_RAIL_H  # 0.86

# Drawer layout: 3 drawers in the space between rails
DRAWER_GAP = 0.004  # visible seam gap around each drawer front
DRAWER_SPACE = TOP_RAIL_BOT - BOTTOM_RAIL_TOP  # 0.70
DRAWER_H = (DRAWER_SPACE - 4.0 * DRAWER_GAP) / 3.0  # ~0.228
DRAWER_FRONT_W = CAB_W - 2.0 * WALL_T - 2.0 * DRAWER_GAP  # ~1.552
DRAWER_FRONT_T = WALL_T  # 0.02

# Drawer box dimensions (behind the front panel)
DRAWER_BOX_D = 0.40  # depth of the drawer box
DRAWER_BOX_WALL = 0.012  # thin sides
DRAWER_BOX_H = DRAWER_H - 0.02  # slightly shorter than front

# Drawer Z positions (bottom edge of each drawer front)
DRAWER_Z_BOT = []
for i in range(3):
    z_bot = BOTTOM_RAIL_TOP + DRAWER_GAP + i * (DRAWER_H + DRAWER_GAP)
    DRAWER_Z_BOT.append(z_bot)

# Drawer slide travel
DRAWER_TRAVEL = 0.35  # how far drawers can pull out

# Lid
LID_T = 0.022  # lid thickness
LID_OVERHANG = 0.015  # slight overhang on sides and front
LID_OPEN = math.radians(85.0)  # lid opens ~85 degrees

# Cap and trim
CAP_T = 0.005  # thin rear hinge rail on top of carcass
CAP_OVERHANG = 0.02


def _leg_solid(mesh_name: str):
    """Splayed tapered leg."""
    leg = (
        cq.Workplane("XY")
        .center(0.025, 0.025)
        .rect(0.030, 0.030)
        .workplane(offset=LEG_H + 0.01)
        .center(-0.025, -0.025)
        .rect(0.050, 0.050)
        .loft()
    )
    return mesh_from_cadquery(leg, mesh_name)


def _drawer_box_solid(mesh_name: str):
    """Open-top drawer box: thin-walled tray with a bottom and four sides."""
    outer_w = DRAWER_FRONT_W - 0.01  # slightly narrower than front
    outer_d = DRAWER_BOX_D
    outer_h = DRAWER_BOX_H
    t = DRAWER_BOX_WALL

    # Bottom plate
    bottom = cq.Workplane("XY").box(outer_w, outer_d, t)
    # Back wall
    back = (
        cq.Workplane("XY")
        .box(outer_w, t, outer_h)
        .translate((0.0, -outer_d / 2.0 + t / 2.0, outer_h / 2.0 + t / 2.0))
    )
    # Front wall (shorter, behind the drawer front panel)
    front = (
        cq.Workplane("XY")
        .box(outer_w, t, outer_h * 0.7)
        .translate((0.0, outer_d / 2.0 - t / 2.0, outer_h * 0.35 + t / 2.0))
    )
    # Side walls
    side_l = (
        cq.Workplane("XY")
        .box(t, outer_d - 2.0 * t, outer_h)
        .translate((-outer_w / 2.0 + t / 2.0, 0.0, outer_h / 2.0 + t / 2.0))
    )
    side_r = (
        cq.Workplane("XY")
        .box(t, outer_d - 2.0 * t, outer_h)
        .translate((outer_w / 2.0 - t / 2.0, 0.0, outer_h / 2.0 + t / 2.0))
    )
    box = bottom.union(back).union(front).union(side_l).union(side_r)
    return mesh_from_cadquery(box, mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_steel_dresser_cabinet")

    steel_body = model.material("steel_body", rgba=(0.60, 0.61, 0.63, 1.0))
    steel_drawer = model.material("steel_drawer", rgba=(0.53, 0.54, 0.57, 1.0))
    steel_trim = model.material("steel_trim", rgba=(0.46, 0.47, 0.49, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.07, 0.07, 0.08, 1.0))
    steel_leg = model.material("steel_leg", rgba=(0.38, 0.39, 0.41, 1.0))
    steel_knob = model.material("steel_knob", rgba=(0.18, 0.18, 0.20, 1.0))
    steel_rivet = model.material("steel_rivet", rgba=(0.52, 0.53, 0.55, 1.0))
    steel_lid = model.material("steel_lid", rgba=(0.57, 0.58, 0.60, 1.0))
    steel_shelf = model.material("steel_shelf", rgba=(0.50, 0.51, 0.53, 1.0))

    # ------------------------------------------------------------------
    # Cabinet body: hollow carcass + legs + front frame + shelves + rivets
    # ------------------------------------------------------------------
    body = model.part("cabinet_body")

    # Side walls
    for sx, vname in ((-1.0, "side_wall_0"), (1.0, "side_wall_1")):
        body.visual(
            Box((WALL_T, CAB_D, CARCASS_H)),
            origin=Origin(xyz=(sx * (CAB_W / 2.0 - WALL_T / 2.0), 0.0, CARCASS_ZC)),
            material=steel_body,
            name=vname,
        )
    # Back wall
    body.visual(
        Box((CAB_W - WALL_T, WALL_T, CARCASS_H - 0.02)),
        origin=Origin(xyz=(0.0, BACK_Y + WALL_T / 2.0, CARCASS_ZC)),
        material=steel_body,
        name="back_wall",
    )
    # Bottom panel
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, LEG_H + WALL_T / 2.0)),
        material=steel_body,
        name="bottom_panel",
    )
    # Top panel (under the lid - the lid rests on this)
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - WALL_T / 2.0)),
        material=steel_body,
        name="top_panel",
    )

    # Front frame: bottom rail and top rail
    body.visual(
        Box((CAB_W - 2.0 * WALL_T, WALL_T, BOTTOM_RAIL_H)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, LEG_H + BOTTOM_RAIL_H / 2.0)
        ),
        material=steel_body,
        name="front_bottom_rail",
    )
    body.visual(
        Box((CAB_W - 2.0 * WALL_T, WALL_T, TOP_RAIL_H)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, CAB_TOP - TOP_RAIL_H / 2.0)
        ),
        material=steel_body,
        name="front_top_rail",
    )

    # Horizontal divider rails between drawers (thin structural strips filling
    # the gap seam between adjacent drawer fronts)
    for i in range(2):
        z_div = DRAWER_Z_BOT[i] + DRAWER_H + DRAWER_GAP / 2.0
        body.visual(
            Box((CAB_W - 2.0 * WALL_T, WALL_T, DRAWER_GAP)),
            origin=Origin(xyz=(0.0, FRONT_Y - WALL_T / 2.0, z_div)),
            material=steel_trim,
            name=f"divider_rail_{i}",
        )

    # Interior shelf boards (visible when drawers are pulled out)
    # Two shelves: one between drawer 0 and 1, one between drawer 1 and 2
    shelf_w = CAB_W - 2.0 * WALL_T - 0.01
    shelf_d = CAB_D - 2.0 * WALL_T - 0.02
    for i in range(2):
        z_shelf = DRAWER_Z_BOT[i] + DRAWER_H + DRAWER_GAP / 2.0
        body.visual(
            Box((shelf_w, shelf_d, 0.015)),
            origin=Origin(xyz=(0.0, -0.01, z_shelf)),
            material=steel_shelf,
            name=f"shelf_board_{i}",
        )

    # Rear hinge rail strip on top (where the lid hinges)
    body.visual(
        Box((CAB_W + 2.0 * CAP_OVERHANG, 0.03, CAP_T)),
        origin=Origin(xyz=(0.0, BACK_Y + 0.015, CAB_TOP + CAP_T / 2.0)),
        material=steel_trim,
        name="rear_hinge_rail",
    )

    # Raised rivet dots along the top front rail
    n_riv = 13
    for i in range(n_riv):
        rx = -0.72 + i * (1.44 / (n_riv - 1))
        body.visual(
            Sphere(radius=0.0045),
            origin=Origin(xyz=(rx, FRONT_Y + 0.002, CAB_TOP - TOP_RAIL_H / 2.0)),
            material=steel_rivet,
            name=f"rivet_{i}",
        )

    # Splayed legs
    leg_mesh = _leg_solid("splayed_leg")
    leg_corners = [
        (0.72, 0.19, 0.0),
        (-0.72, 0.19, math.pi / 2.0),
        (-0.72, -0.19, math.pi),
        (0.72, -0.19, 3.0 * math.pi / 2.0),
    ]
    for i, (lx, ly, yaw) in enumerate(leg_corners):
        body.visual(
            leg_mesh,
            origin=Origin(xyz=(lx, ly, 0.0), rpy=(0.0, 0.0, yaw)),
            material=steel_leg,
            name=f"leg_{i}",
        )

    # Internal drawer slide rails (thin strips on each side wall interior)
    for i in range(3):
        z_rail = DRAWER_Z_BOT[i] + DRAWER_BOX_H / 2.0 + WALL_T
        for sx in (-1.0, 1.0):
            body.visual(
                Box((0.008, DRAWER_BOX_D + 0.02, 0.008)),
                origin=Origin(
                    xyz=(sx * (CAB_W / 2.0 - WALL_T - 0.004), 0.03, z_rail)
                ),
                material=steel_trim,
                name=f"slide_rail_{i}_{'L' if sx < 0 else 'R'}",
            )

    # ------------------------------------------------------------------
    # Top lid: flat panel hinged at rear edge
    # ------------------------------------------------------------------
    lid = model.part("top_lid")
    lid_w = CAB_W + 2.0 * LID_OVERHANG
    lid_d = CAB_D + LID_OVERHANG  # overhangs front only, rear is at hinge

    # Lid panel - part frame at the hinge line (rear top edge), panel extends +Y
    lid.visual(
        Box((lid_w, lid_d, LID_T)),
        origin=Origin(xyz=(0.0, lid_d / 2.0, LID_T / 2.0)),
        material=steel_lid,
        name="lid_panel",
    )
    # Small lid handle on the front edge
    lid.visual(
        Box((0.10, 0.015, 0.020)),
        origin=Origin(xyz=(0.0, lid_d - 0.03, LID_T + 0.010)),
        material=steel_knob,
        name="lid_handle",
    )
    # Hinge barrel detail on the rear edge
    hinge_barrel_len = CAB_W - 0.10
    lid.visual(
        Cylinder(radius=0.008, length=hinge_barrel_len),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel_trim,
        name="lid_hinge_barrel",
    )

    # Articulation: lid hinges at rear top edge
    # Part frame at hinge line. Panel extends along +Y from the hinge.
    # Axis: -X so positive rotation lifts the front edge upward.
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, BACK_Y, CAB_TOP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0, velocity=2.0, lower=0.0, upper=LID_OPEN
        ),
    )

    # ------------------------------------------------------------------
    # Three drawers: prismatic joints sliding out along +Y
    # ------------------------------------------------------------------
    drawer_box_mesh = _drawer_box_solid("drawer_box")

    drawers = []
    for i in range(3):
        drawer = model.part(f"drawer_{i}")
        z_center = DRAWER_Z_BOT[i] + DRAWER_H / 2.0

        # Drawer front panel (the visible face with gap seams)
        drawer.visual(
            Box((DRAWER_FRONT_W, DRAWER_FRONT_T, DRAWER_H)),
            origin=Origin(xyz=(0.0, -DRAWER_FRONT_T / 2.0, 0.0)),
            material=steel_drawer,
            name="front_panel",
        )

        # Drawer pull handle (small horizontal bar centered on the front)
        drawer.visual(
            Box((0.12, 0.010, 0.020)),
            origin=Origin(xyz=(0.0, 0.005, 0.0)),
            material=steel_knob,
            name="pull_handle",
        )
        # Handle mounting bosses
        for hx in (-0.045, 0.045):
            drawer.visual(
                Cylinder(radius=0.006, length=0.008),
                origin=Origin(
                    xyz=(hx, 0.004, 0.0),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=steel_knob,
                name=f"handle_boss_{'L' if hx < 0 else 'R'}",
            )

        # Vent slot near the bottom of the drawer front (dark recessed)
        slot_len = 0.30
        slot_w = 0.018
        slot_zc = -DRAWER_H / 2.0 + 0.04  # near bottom
        drawer.visual(
            Box((slot_w + 0.012, 0.004, slot_len + 0.024)),
            origin=Origin(xyz=(0.0, -DRAWER_FRONT_T - 0.001, slot_zc)),
            material=steel_dark,
            name="vent_slot",
        )

        # Drawer box (tray behind the front, slides with the drawer)
        # Origin of the drawer part is at the front face center
        drawer.visual(
            drawer_box_mesh,
            origin=Origin(
                xyz=(0.0, -DRAWER_FRONT_T - DRAWER_BOX_D / 2.0, -DRAWER_H / 2.0 + DRAWER_BOX_WALL)
            ),
            material=steel_body,
            name="box",
        )

        # Prismatic articulation: drawer slides out along +Y
        model.articulation(
            f"drawer_{i}_slide",
            ArticulationType.PRISMATIC,
            parent=body,
            child=drawer,
            origin=Origin(xyz=(0.0, FRONT_Y, z_center)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=30.0, velocity=0.5, lower=0.0, upper=DRAWER_TRAVEL
            ),
        )
        drawers.append(drawer)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
    lid = object_model.get_part("top_lid")
    lid_hinge = object_model.get_articulation("lid_hinge")
    drawers = [object_model.get_part(f"drawer_{i}") for i in range(3)]
    slides = [object_model.get_articulation(f"drawer_{i}_slide") for i in range(3)]

    # Lid hinge barrel intentionally overlaps the carcass top panel at the
    # rear pivot line (captured pivot pin in a top-panel bore).
    ctx.allow_overlap(
        body,
        lid,
        elem_a="top_panel",
        elem_b="lid_hinge_barrel",
        reason="Lid hinge barrel is a captured pivot element seated at the rear edge of the carcass top panel.",
    )
    ctx.expect_contact(
        lid,
        body,
        elem_a="lid_hinge_barrel",
        elem_b="rear_hinge_rail",
        contact_tol=0.005,
        name="lid hinge barrel contacts the rear hinge rail",
    )

    # --- Overall envelope, true scale, grounded on the floor ---------------
    aabb = ctx.part_world_aabb(body)
    ok = aabb is not None
    ctx.check("cabinet body has bounds", ok, details=str(aabb))
    if aabb is not None:
        (x0, y0, z0), (x1, y1, z1) = aabb
        ctx.check(
            "overall width ~1.6 m",
            1.55 <= (x1 - x0) <= 1.70,
            details=f"width={x1 - x0:.3f}",
        )
        ctx.check(
            "overall depth ~0.5 m",
            0.48 <= (y1 - y0) <= 0.58,
            details=f"depth={y1 - y0:.3f}",
        )
        ctx.check(
            "overall height ~0.9 m (low dresser)",
            0.85 <= z1 <= 1.00,
            details=f"top={z1:.3f}",
        )
        ctx.check("legs rest on the floor", abs(z0) <= 1e-6, details=f"zmin={z0:.5f}")

    # --- Lid: hinge type, axis, closed position, opens upward --------------
    ctx.check(
        "lid hinge is revolute",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE,
    )
    ax = lid_hinge.axis
    ctx.check(
        "lid hinge axis is horizontal (along X)",
        abs(abs(ax[0]) - 1.0) < 1e-9 and abs(ax[1]) < 1e-9 and abs(ax[2]) < 1e-9,
        details=str(ax),
    )
    lim = lid_hinge.motion_limits
    ctx.check(
        "lid hinge opens 0 to ~85 deg",
        lim is not None and lim.lower == 0.0 and lim.upper > math.radians(60.0),
        details=f"upper={lim.upper:.3f}" if lim else "no limits",
    )

    # Closed lid sits on top of the carcass
    lid_closed = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "closed lid sits at carcass top height",
        lid_closed is not None and abs(lid_closed[0][2] - CAB_TOP) < 0.005,
        details=str(lid_closed),
    )

    # Open lid: front edge rises above the carcass
    with ctx.pose({lid_hinge: LID_OPEN}):
        lid_open = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "open lid front edge rises above the carcass top",
        lid_open is not None and lid_open[1][2] > CAB_TOP + 0.15,
        details=str(lid_open),
    )

    # --- Drawers: prismatic joints, slide out, gap seams --------------------
    for i, (drawer, slide) in enumerate(zip(drawers, slides)):
        ctx.check(
            f"drawer_{i} slide is prismatic",
            slide.articulation_type == ArticulationType.PRISMATIC,
        )
        slide_ax = slide.axis
        ctx.check(
            f"drawer_{i} slides along +Y (front)",
            abs(slide_ax[0]) < 1e-9 and abs(slide_ax[1] - 1.0) < 1e-9 and abs(slide_ax[2]) < 1e-9,
            details=str(slide_ax),
        )
        slide_lim = slide.motion_limits
        ctx.check(
            f"drawer_{i} slide range 0..{DRAWER_TRAVEL:.2f} m",
            slide_lim is not None
            and slide_lim.lower == 0.0
            and abs(slide_lim.upper - DRAWER_TRAVEL) < 1e-6,
        )

        # Closed drawer front is flush with the cabinet front face
        front_closed = ctx.part_element_world_aabb(drawer, elem="front_panel")
        ctx.check(
            f"drawer_{i} closed front is flush with cabinet front",
            front_closed is not None and abs(front_closed[1][1] - FRONT_Y) < 0.005,
            details=str(front_closed),
        )

        # Gap seams: drawer front does NOT fully touch the frame on all sides
        # (there should be a visible gap around it)
        ctx.expect_gap(
            drawer,
            body,
            axis="z",
            positive_elem="front_panel",
            negative_elem="front_bottom_rail" if i == 0 else f"divider_rail_{i - 1}",
            max_penetration=0.002,
            name=f"drawer_{i} has gap seam at bottom edge",
        )

        # Opened drawer: front panel moves forward
        with ctx.pose({slide: DRAWER_TRAVEL}):
            front_open = ctx.part_element_world_aabb(drawer, elem="front_panel")
        ctx.check(
            f"drawer_{i} front moves forward when opened",
            front_open is not None
            and front_open[0][1] >= FRONT_Y + DRAWER_TRAVEL - 0.03,
            details=str(front_open),
        )

        # Drawer stays within cabinet width
        ctx.expect_within(
            drawer,
            body,
            axes="x",
            margin=0.02,
            name=f"drawer_{i} stays inside cabinet width",
        )

    # --- Shelf boards are present inside the cabinet -----------------------
    for i in range(2):
        shelf_aabb = ctx.part_element_world_aabb(body, elem=f"shelf_board_{i}")
        ctx.check(
            f"shelf_board_{i} exists inside the cabinet",
            shelf_aabb is not None
            and shelf_aabb[0][2] > LEG_H
            and shelf_aabb[1][2] < CAB_TOP,
            details=str(shelf_aabb),
        )

    # Shelf boards become exposed when drawers are pulled out
    with ctx.pose({slides[1]: DRAWER_TRAVEL}):
        shelf_visible = ctx.part_element_world_aabb(body, elem="shelf_board_0")
    ctx.check(
        "shelf board is present behind the drawer opening",
        shelf_visible is not None
        and shelf_visible[1][1] > BACK_Y + WALL_T,
        details=str(shelf_visible),
    )

    # --- Rivet detail on top rail -----------------------------------------
    rivet_aabb = ctx.part_element_world_aabb(body, elem="rivet_0")
    ctx.check(
        "rivet dots stand proud of the top rail face",
        rivet_aabb is not None
        and rivet_aabb[1][1] > FRONT_Y + 0.003,
        details=str(rivet_aabb),
    )

    # --- Hinge barrel on lid -----------------------------------------------
    barrel_aabb = ctx.part_element_world_aabb(lid, elem="lid_hinge_barrel")
    ctx.check(
        "lid hinge barrel is at the rear of the cabinet",
        barrel_aabb is not None
        and barrel_aabb[0][1] < BACK_Y + 0.03,
        details=str(barrel_aabb),
    )

    return ctx.report()


object_model = build_object_model()
