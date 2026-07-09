from __future__ import annotations

# Realistic articulated dishwasher modeled from picture/Equipment/Dishwisher/001.png.
#
# Articraft brief:
# - Object: freestanding stainless-steel dishwasher, ~0.598 m wide x 0.585 m deep
#   x 0.845 m tall. Brushed-steel cabinet with a top-front control panel (recessed
#   display + buttons), a large stainless front door, and a dark recessed kickplate.
# - Root/support: `cabinet` is the fixed hollow tub housing (sides, top, back,
#   bottom, kickplate) plus the non-moving top-front control panel.
# - Parts: cabinet (housing + tub interior + kickplate + control panel + display
#   bezel + buttons), door (front skin + inner liner + handle + interior tub wall),
#   lower_rack (pull-out wire dish rack inside the tub).
# - Articulations:
#   * cabinet_to_door  REVOLUTE, hinge line at the front-bottom edge of the
#     cabinet, axis +Y so positive q drops the door forward/down from vertical
#     (closed=0) to horizontal (open ~1.50 rad).
#   * cabinet_to_rack  PRISMATIC, +X (forward), the lower rack slides out of the
#     tub on its rails (closed=0, extended ~0.42 m).
# - Visible geometry: hollow tub cavity, brushed-steel skins, recessed dark
#   control-panel display with blue digits and round buttons, a horizontal bar
#   handle on the door, a wire-frame pull-out rack with tines.
# - Support/fit: door hinges off the cabinet front-bottom edge; rack rails captured
#   inside the tub cavity. Door inner liner seats against the tub opening when shut.
# - Intentional overlaps: rack rails nested in the tub side walls (proxy capture);
#   handle bar ends embedded in their stand-offs.
# - Tests: door present + bottom-hinged drop-down (positive q lowers free edge),
#   closed door seats against the cabinet opening, control display present on the
#   panel, handle on the door, rack present and slides forward out of the tub.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Inertial,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Overall real-world dimensions (meters).
# ---------------------------------------------------------------------------
W = 0.598          # cabinet width  (X)
D = 0.585          # cabinet depth  (Y)
H = 0.845          # cabinet height (Z)

WALL = 0.018       # housing skin thickness
TUB_WALL = 0.010   # interior tub wall thickness

KICK_H = 0.085     # kickplate height at the bottom front
PANEL_H = 0.075    # top-front control-panel band height

DOOR_GAP = 0.006   # cosmetic reveal between door and cabinet opening
DOOR_T = 0.045     # door assembly thickness (front skin + liner)

# The structural door opening in the cabinet front face.
OPEN_W = W - 2.0 * WALL
OPEN_BOTTOM = KICK_H
OPEN_TOP = H - PANEL_H
OPEN_H = OPEN_TOP - OPEN_BOTTOM


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dishwasher")

    steel = model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    steel_dark = model.material("dark_steel", rgba=(0.36, 0.37, 0.39, 1.0))
    tub_steel = model.material("tub_steel", rgba=(0.55, 0.56, 0.58, 1.0))
    kick_mat = model.material("kickplate", rgba=(0.18, 0.18, 0.20, 1.0))
    panel_mat = model.material("panel_face", rgba=(0.62, 0.63, 0.65, 1.0))
    display_mat = model.material("display_glass", rgba=(0.06, 0.07, 0.09, 1.0))
    digit_mat = model.material("display_digits", rgba=(0.20, 0.55, 0.95, 1.0))
    button_mat = model.material("button", rgba=(0.12, 0.12, 0.14, 1.0))
    handle_mat = model.material("handle_steel", rgba=(0.80, 0.81, 0.83, 1.0))
    rack_mat = model.material("rack_wire", rgba=(0.70, 0.71, 0.73, 1.0))

    # =======================================================================
    # CABINET (root): hollow tub housing, top, back, bottom, kickplate.
    # Built in world frame; cabinet front face is +Y, opening centered.
    # =======================================================================
    cabinet = model.part("cabinet")

    # --- Outer housing as a hollow shell. Author it as a single CadQuery body:
    # a box shelled open on the front (+Y) and with the lower kickplate carved
    # so the dark recessed kickplate reads correctly.
    outer = (
        cq.Workplane("XY")
        .box(W, D, H, centered=(True, True, False))
    )
    # Hollow the tub cavity: subtract an inner box, leaving walls/top/back/bottom.
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .box(
            W - 2.0 * WALL,
            D - WALL,  # open the front (+Y) face by extending past front
            H - WALL - PANEL_H,
            centered=(True, True, False),
        )
        .translate((0.0, (WALL) / 2.0 + 0.001, 0.0))
    )
    housing = outer.cut(cavity)
    # Open the front of the cavity fully (cut the front wall over the opening).
    front_cut = (
        cq.Workplane("XY")
        .workplane(offset=OPEN_BOTTOM)
        .box(OPEN_W, 0.06, OPEN_H, centered=(True, False, False))
        .translate((0.0, D / 2.0 - 0.03 + 0.0001, 0.0))
    )
    housing = housing.cut(front_cut)
    housing_mesh = mesh_from_cadquery(housing, "cabinet_housing")
    cabinet.visual(housing_mesh, material=steel, name="housing")

    # --- Dark recessed kickplate across the bottom front.
    kick = (
        cq.Workplane("XY")
        .box(OPEN_W, 0.012, KICK_H - 0.006, centered=(True, False, False))
        .translate((0.0, D / 2.0 - 0.020, 0.003))
    )
    kick_mesh = mesh_from_cadquery(kick, "kickplate")
    cabinet.visual(kick_mesh, material=kick_mat, name="kickplate")

    # --- Interior tub back/side liner so the open door reveals a real cavity,
    # not a hollow void edge. A thin liner box inset from the housing.
    liner = (
        cq.Workplane("XY")
        .workplane(offset=OPEN_BOTTOM + 0.004)
        .box(
            OPEN_W - 0.012,
            D - WALL - 0.05,
            OPEN_H - 0.010,
            centered=(True, True, False),
        )
        .translate((0.0, -0.018, 0.0))
        .faces(">Y")
        .shell(-TUB_WALL)
    )
    liner_mesh = mesh_from_cadquery(liner, "tub_liner")
    cabinet.visual(liner_mesh, material=tub_steel, name="tub_liner")

    # --- Top-front control panel band (non-moving), brushed steel face.
    panel = (
        cq.Workplane("XY")
        .workplane(offset=H - PANEL_H)
        .box(W, 0.020, PANEL_H, centered=(True, False, False))
        .translate((0.0, D / 2.0 - 0.020, 0.0))
    )
    panel_mesh = mesh_from_cadquery(panel, "control_panel")
    cabinet.visual(panel_mesh, material=panel_mat, name="control_panel")

    # --- Recessed display window on the right of the panel (dark glass).
    disp_z = H - PANEL_H + PANEL_H * 0.45
    display = (
        cq.Workplane("XY")
        .box(0.090, 0.006, 0.030)
    )
    display_mesh = mesh_from_cadquery(display, "display_glass")
    cabinet.visual(
        display_mesh,
        origin=Origin(xyz=(0.150, D / 2.0 + 0.002, disp_z)),
        material=display_mat,
        name="display_glass",
    )
    # Blue digit strip on the display.
    digits = (
        cq.Workplane("XY")
        .box(0.060, 0.004, 0.014)
    )
    digits_mesh = mesh_from_cadquery(digits, "display_digits")
    cabinet.visual(
        digits_mesh,
        origin=Origin(xyz=(0.150, D / 2.0 + 0.004, disp_z)),
        material=digit_mat,
        name="display_digits",
    )

    # --- Round control buttons to the left of the display.
    # cq cylinder signature is cylinder(height, radius); build a short Z disk
    # then tilt it so it faces forward (+Y) out of the panel.
    for i, bx in enumerate((-0.150, -0.110, -0.070, -0.030)):
        btn = (
            cq.Workplane("XY")
            .cylinder(0.010, 0.008, centered=(True, True, False))
            .rotate((0, 0, 0), (1, 0, 0), -90.0)
        )
        btn_mesh = mesh_from_cadquery(btn, f"button_{i}")
        cabinet.visual(
            btn_mesh,
            origin=Origin(xyz=(bx, D / 2.0, disp_z)),
            material=button_mat,
            name=f"button_{i}",
        )

    cabinet.inertial = Inertial.from_geometry(Box((W, D, H)), mass=45.0)

    # =======================================================================
    # DOOR: large stainless front panel, bottom-hinged drop-down.
    #
    # The door part frame is placed AT the hinge line (front-bottom edge of the
    # opening) so the articulation pivot and the door frame coincide at q=0.
    # In the door-local frame: +Z is up (door extends upward when closed),
    # +Y is outward (front). The inner liner faces -Y (into the tub).
    # =======================================================================
    door = model.part("door")

    # The outer door panel overlaps the cabinet front frame around the opening so
    # the closed door seats against the cabinet face (a real gasket-style seat).
    door_w = OPEN_W + 0.020
    door_h = OPEN_H + 0.018

    # Door-local Y layout (closed door faces +Y / forward):
    #   liner: y in [0.000, 0.018]  (plugs into the tub opening when closed)
    #   skin:  y in [0.000, 0.024]  (back face seats on the cabinet front frame)
    LINER_BACK = 0.000
    LINER_FRONT = 0.018
    SKIN_BACK = 0.000
    SKIN_FRONT = 0.024

    # Outer stainless skin (front face). Local +Z runs up the closed door.
    skin = (
        cq.Workplane("XZ")
        .box(door_w, door_h, SKIN_FRONT - SKIN_BACK, centered=(True, False, True))
        .edges("|Y").fillet(0.004)
        .translate((0.0, (SKIN_BACK + SKIN_FRONT) / 2.0, 0.0))
    )
    skin_mesh = mesh_from_cadquery(skin, "door_skin")
    door.visual(skin_mesh, material=steel, name="door_skin")

    # Inner liner (faces the tub when closed); sized smaller than the opening so
    # it plugs into the tub cavity instead of jamming on the opening frame.
    liner_in = (
        cq.Workplane("XZ")
        .box(OPEN_W - 0.040, OPEN_H - 0.040, LINER_FRONT - LINER_BACK,
             centered=(True, False, True))
        .translate((0.0, (LINER_BACK + LINER_FRONT) / 2.0, 0.0))
    )
    liner_in_mesh = mesh_from_cadquery(liner_in, "door_liner")
    door.visual(liner_in_mesh, material=tub_steel, name="door_liner")

    # Horizontal bar handle near the top of the door, standing proud of the skin.
    handle_y = SKIN_FRONT + 0.028  # front skin face + stand-off length
    handle_z = door_h - 0.045
    # Two end stand-offs. cylinder(height, radius): a Z disk of height 0.028,
    # tilted to point forward (+Y) so it stands the bar proud of the skin.
    boss_x = door_w * 0.5 - 0.060
    for sx in (-boss_x, boss_x):
        boss = (
            cq.Workplane("XY")
            .cylinder(0.028, 0.010, centered=(True, True, False))
            .rotate((0, 0, 0), (1, 0, 0), -90.0)
        )
        boss_mesh = mesh_from_cadquery(boss, f"handle_boss_{1 if sx > 0 else 0}")
        door.visual(
            boss_mesh,
            origin=Origin(xyz=(sx, SKIN_FRONT - 0.004, handle_z)),
            material=handle_mat,
            name=f"handle_boss_{1 if sx > 0 else 0}",
        )
    # The handle bar itself, spanning between the stand-offs along X.
    # cylinder(height, radius) builds along +Z; rotate 90 about Y -> along X.
    bar = (
        cq.Workplane("XY")
        .cylinder(door_w - 0.080, 0.010, centered=(True, True, True))
        .rotate((0, 0, 0), (0, 1, 0), 90.0)
    )
    bar_mesh = mesh_from_cadquery(bar, "door_handle")
    door.visual(
        bar_mesh,
        origin=Origin(xyz=(0.0, handle_y, handle_z)),
        material=handle_mat,
        name="door_handle",
    )

    door.inertial = Inertial.from_geometry(
        Box((door_w, 0.045, door_h)),
        mass=9.0,
        origin=Origin(xyz=(0.0, DOOR_T / 2.0, door_h / 2.0)),
    )

    # Hinge line: front-bottom edge of the opening, on the cabinet front plane.
    # Skin back (local y=0) is coplanar with the cabinet front face so the closed
    # door perimeter seats against the cabinet front frame around the opening.
    hinge_y = D / 2.0
    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(0.0, hinge_y, OPEN_BOTTOM)),
        # Hinge runs left-right along X at the front-bottom edge. The closed door
        # extends along door-local +Z. axis=(-1,0,0): positive q rotates the top
        # (free) edge forward (+Y) and down -> classic dishwasher drop-down door.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.5, lower=0.0, upper=1.50),
    )

    # =======================================================================
    # LOWER RACK: pull-out wire dish rack inside the tub. PRISMATIC along +Y
    # (forward, out of the tub opening).
    # =======================================================================
    rack = model.part("rack")

    rack_w = OPEN_W - 0.060
    rack_d = 0.400  # shorter than the cavity so it clears the tub back wall
    rack_floor_z = 0.004  # local; rack base near tub floor

    # Perimeter rails (a wire-frame basket): straight wires built along +Z
    # (cylinder(height, radius)) then rotated to run along X or Y.
    def wire_box(length, axis_dir, pos):
        c = (
            cq.Workplane("XY")
            .cylinder(length, 0.0035, centered=(True, True, True))
        )
        if axis_dir == "x":
            c = c.rotate((0, 0, 0), (0, 1, 0), 90.0)
        elif axis_dir == "y":
            c = c.rotate((0, 0, 0), (1, 0, 0), 90.0)
        return c.translate(pos)

    parts = []
    # Bottom perimeter frame.
    parts.append(wire_box(rack_w, "x", (0.0, rack_d / 2.0, rack_floor_z)))
    parts.append(wire_box(rack_w, "x", (0.0, -rack_d / 2.0, rack_floor_z)))
    parts.append(wire_box(rack_d, "y", (rack_w / 2.0, 0.0, rack_floor_z)))
    parts.append(wire_box(rack_d, "y", (-rack_w / 2.0, 0.0, rack_floor_z)))
    # Top perimeter frame (basket height).
    rack_top = rack_floor_z + 0.090
    parts.append(wire_box(rack_w, "x", (0.0, rack_d / 2.0, rack_top)))
    parts.append(wire_box(rack_w, "x", (0.0, -rack_d / 2.0, rack_top)))
    parts.append(wire_box(rack_d, "y", (rack_w / 2.0, 0.0, rack_top)))
    parts.append(wire_box(rack_d, "y", (-rack_w / 2.0, 0.0, rack_top)))
    # Corner verticals (Z bars: cylinder(height, radius)).
    for sx in (-rack_w / 2.0, rack_w / 2.0):
        for sy in (-rack_d / 2.0, rack_d / 2.0):
            v = (
                cq.Workplane("XY")
                .cylinder(0.090, 0.0035, centered=(True, True, False))
                .translate((sx, sy, rack_floor_z))
            )
            parts.append(v)
    # Tine grid along the floor (dish prongs).
    n_tines = 9
    for i in range(n_tines):
        tx = -rack_w / 2.0 + (i + 0.5) * (rack_w / n_tines)
        parts.append(wire_box(rack_d, "y", (tx, 0.0, rack_floor_z)))

    # The basket frame and tines union into one connected wire-rack mesh.
    rack_geom = parts[0]
    for p in parts[1:]:
        rack_geom = rack_geom.union(p)
    rack_mesh = mesh_from_cadquery(rack_geom, "rack_basket")
    rack.visual(rack_mesh, material=rack_mat, name="rack_basket")

    # Side rails that ride in the tub on the liner side walls (captured fit).
    # Each rail spans from inside the basket top side wire (so it touches the
    # basket and the rack reads as one piece) outward to ~1 mm inside the tub
    # liner inner side wall, keeping the rack rail-supported instead of floating.
    liner_inner_x = 0.275 - TUB_WALL  # ~0.265, tub liner inner side-wall face
    rail_outer = liner_inner_x + 0.001  # ~1 mm into the wall -> captured contact
    rail_inner = rack_w / 2.0 - 0.006  # overlaps the basket top side wire
    rail_len = rail_outer - rail_inner
    rail_cx = (rail_outer + rail_inner) / 2.0
    for sx in (-rail_cx, rail_cx):
        rail = (
            cq.Workplane("XY")
            .box(rail_len, rack_d, 0.010, centered=(True, True, False))
            .translate((sx, 0.0, rack_top - 0.004))
        )
        rail_mesh = mesh_from_cadquery(rail, f"rack_rail_{1 if sx > 0 else 0}")
        rack.visual(
            rail_mesh,
            material=rack_mat,
            name=f"rack_rail_{1 if sx > 0 else 0}",
        )

    rack.inertial = Inertial.from_geometry(
        Box((rack_w, rack_d, 0.090)), mass=2.5
    )

    # Rack rest position: suspended on its rails inside the tub, clear of the
    # tub floor and back wall, set back so it can travel forward (+Y) out.
    rack_rest_z = OPEN_BOTTOM + 0.030
    rack_rest_y = -0.010  # rest centered slightly back inside the tub
    model.articulation(
        "cabinet_to_rack",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=rack,
        origin=Origin(xyz=(0.0, rack_rest_y, rack_rest_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.3, lower=0.0, upper=0.42),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    rack = object_model.get_part("rack")
    door_hinge = object_model.get_articulation("cabinet_to_door")
    rack_slide = object_model.get_articulation("cabinet_to_rack")

    # The rack rails ride captured in the tub liner side walls (a real
    # pull-out slide fit), so their ~1 mm rail-in-wall embed is intentional.
    ctx.allow_overlap(
        rack,
        cabinet,
        elem_a="rack_rail_0",
        elem_b="tub_liner",
        reason="Left rack rail is captured in the tub liner side-wall slide channel.",
    )
    ctx.allow_overlap(
        rack,
        cabinet,
        elem_a="rack_rail_1",
        elem_b="tub_liner",
        reason="Right rack rail is captured in the tub liner side-wall slide channel.",
    )

    # --- Joint type / axis contracts -------------------------------------
    ctx.check(
        "door is a revolute drop-down hinge",
        door_hinge.joint_type == "revolute"
        and tuple(round(a, 3) for a in door_hinge.axis) == (-1.0, 0.0, 0.0),
        details=f"type={door_hinge.joint_type}, axis={door_hinge.axis}",
    )
    ctx.check(
        "rack is a prismatic pull-out",
        rack_slide.joint_type == "prismatic"
        and tuple(round(a, 3) for a in rack_slide.axis) == (0.0, 1.0, 0.0),
        details=f"type={rack_slide.joint_type}, axis={rack_slide.axis}",
    )

    # --- Hero parts present ----------------------------------------------
    ctx.check(
        "door has a stainless skin, liner, and handle",
        {"door_skin", "door_liner", "door_handle"}.issubset(
            {v.name for v in door.visuals}
        ),
        details=f"door visuals={[v.name for v in door.visuals]}",
    )
    ctx.check(
        "control panel carries display + digits + buttons",
        {"control_panel", "display_glass", "display_digits", "button_0"}.issubset(
            {v.name for v in cabinet.visuals}
        ),
        details=f"cabinet visuals={[v.name for v in cabinet.visuals]}",
    )

    # --- Rest-pose seating: closed door fronts the cabinet opening --------
    with ctx.pose({door_hinge: 0.0, rack_slide: 0.0}):
        # The closed door reads as tall and vertical: its Z extent should be
        # much larger than its Y (thickness) extent.
        d_aabb = ctx.part_world_aabb(door)
        assert d_aabb is not None
        (dx0, dy0, dz0), (dx1, dy1, dz1) = d_aabb
        ctx.check(
            "closed door stands vertically tall",
            (dz1 - dz0) > 0.55 and (dy1 - dy0) < 0.12,
            details=f"z_extent={dz1 - dz0:.3f}, y_extent={dy1 - dy0:.3f}",
        )
        # Door front sits ahead of (or flush with) the cabinet front opening.
        ctx.expect_overlap(door, cabinet, axes="xz", min_overlap=0.30)
        # Closed door seats against the cabinet front frame (no big gap).
        ctx.expect_contact(door, cabinet, contact_tol=0.002, name="closed door seats on cabinet")
        # Rack lives inside the tub cavity at rest (within cabinet footprint).
        ctx.expect_within(rack, cabinet, axes="x", margin=0.02)
        # Rack is rail-supported: each rail contacts the tub liner side wall.
        ctx.expect_contact(
            rack, cabinet, elem_a="rack_rail_0", elem_b="tub_liner",
            name="left rack rail captured in tub wall",
        )
        ctx.expect_contact(
            rack, cabinet, elem_a="rack_rail_1", elem_b="tub_liner",
            name="right rack rail captured in tub wall",
        )

    # --- Door drops down: positive q lowers the free (top) edge forward ----
    closed_top = None
    with ctx.pose({door_hinge: 0.0}):
        c_aabb = ctx.part_world_aabb(door)
        assert c_aabb is not None
        closed_top = c_aabb[1][2]  # max Z
        closed_front = c_aabb[1][1]  # max Y
    with ctx.pose({door_hinge: 1.50}):
        o_aabb = ctx.part_world_aabb(door)
        assert o_aabb is not None
        open_top = o_aabb[1][2]
        open_front = o_aabb[1][1]
    ctx.check(
        "door free edge drops when opened",
        open_top < closed_top - 0.40,
        details=f"closed_top_z={closed_top:.3f}, open_top_z={open_top:.3f}",
    )
    ctx.check(
        "door swings forward when opened",
        open_front > closed_front + 0.20,
        details=f"closed_front_y={closed_front:.3f}, open_front_y={open_front:.3f}",
    )

    # --- Rack pulls forward out of the tub --------------------------------
    with ctx.pose({rack_slide: 0.0}):
        r0 = ctx.part_world_position(rack)
    with ctx.pose({rack_slide: 0.42}):
        r1 = ctx.part_world_position(rack)
    assert r0 is not None and r1 is not None
    ctx.check(
        "rack slides forward out of the tub",
        r1[1] > r0[1] + 0.35,
        details=f"rest_y={r0[1]:.3f}, extended_y={r1[1]:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
