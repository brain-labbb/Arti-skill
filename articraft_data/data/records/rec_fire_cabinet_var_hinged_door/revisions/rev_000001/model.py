from __future__ import annotations

# Tall single-door utility / fire cabinet (dark charcoal steel).
#
# World layout: the cabinet front faces +X; width is along Y; height along +Z.
# The base plinth rests at z=0. The carcass is a hollow shell (back wall, two
# side walls, top, bottom, and a perimeter front face frame) enclosing a cavity
# with two fixed internal shelves. A single tall door hangs on the left edge
# (positive Y) and swings open on a vertical-axis REVOLUTE hinge. The door
# carries a recessed pull handle on its free (right) edge.
#
# Primary articulation: cabinet_to_door (REVOLUTE, +Z vertical axis at the left
# hinge line), positive q swings the free edge outward (+X).

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BarrelHingeGeometry,
    Box,
    HingeHolePattern,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_door_utility_cabinet")

    charcoal = model.material("charcoal_steel", rgba=(0.15, 0.16, 0.18, 1.0))
    charcoal_dark = model.material("charcoal_dark", rgba=(0.075, 0.080, 0.090, 1.0))
    interior_dark = model.material("interior_dark_steel", rgba=(0.105, 0.110, 0.125, 1.0))
    handle_dark = model.material("handle_dark", rgba=(0.09, 0.09, 0.10, 1.0))
    hinge_grey = model.material("hinge_grey", rgba=(0.22, 0.23, 0.25, 1.0))
    badge_red = model.material("badge_red", rgba=(0.78, 0.20, 0.13, 1.0))
    shelf_steel = model.material("shelf_steel", rgba=(0.18, 0.19, 0.21, 1.0))

    # ---------- key dimensions (meters) ----------
    W = 0.380          # width (Y)
    D = 0.600          # depth (X), front at x=D, back at x=0
    H = 1.300          # total height (Z)
    wall = 0.014       # carcass wall thickness
    plinth_h = 0.060   # recessed base plinth height
    plinth_inset = 0.020

    # ===================================================================
    # ROOT: cabinet carcass (shell)
    # ===================================================================
    cab = model.part("cabinet")

    # Recessed base plinth (the cabinet stands on this).
    cab.visual(
        Box((D - 2 * plinth_inset, W - 2 * plinth_inset, plinth_h)),
        origin=Origin(xyz=(D / 2.0, 0.0, plinth_h / 2.0)),
        material=charcoal_dark,
        name="base_plinth",
    )

    body_bottom_z = plinth_h
    body_h = H - plinth_h

    # Bottom panel of the carcass.
    cab.visual(
        Box((D, W, wall)),
        origin=Origin(xyz=(D / 2.0, 0.0, body_bottom_z + wall / 2.0)),
        material=charcoal,
        name="bottom_panel",
    )
    # Top panel.
    cab.visual(
        Box((D, W, wall)),
        origin=Origin(xyz=(D / 2.0, 0.0, H - wall / 2.0)),
        material=charcoal,
        name="top_panel",
    )
    # Back wall.
    cab.visual(
        Box((wall, W, body_h)),
        origin=Origin(xyz=(wall / 2.0, 0.0, body_bottom_z + body_h / 2.0)),
        material=charcoal,
        name="back_wall",
    )
    # Two side walls.
    for s, tag in ((1, "left"), (-1, "right")):
        cab.visual(
            Box((D, wall, body_h)),
            origin=Origin(xyz=(D / 2.0, s * (W / 2.0 - wall / 2.0),
                               body_bottom_z + body_h / 2.0)),
            material=charcoal,
            name=f"side_wall_{tag}",
        )

    # ---------- front face frame (perimeter around the single door opening) ----------
    open_top_z = H - wall          # under the top panel
    open_bot_z = body_bottom_z + wall
    stack_h = open_top_z - open_bot_z
    frame_w = 0.018                # frame member width
    face_x = D - 0.004             # slightly proud of the body
    inner_W = W - 2 * wall

    # Top horizontal rail.
    cab.visual(
        Box((0.010, W, frame_w)),
        origin=Origin(xyz=(face_x, 0.0, open_top_z - frame_w / 2.0)),
        material=charcoal,
        name="face_rail_top",
    )
    # Bottom horizontal rail.
    cab.visual(
        Box((0.010, W, frame_w)),
        origin=Origin(xyz=(face_x, 0.0, open_bot_z + frame_w / 2.0)),
        material=charcoal,
        name="face_rail_bottom",
    )
    # Left and right vertical stiles.
    for s, tag in ((1, "left"), (-1, "right")):
        cab.visual(
            Box((0.010, frame_w, stack_h)),
            origin=Origin(xyz=(face_x, s * (W / 2.0 - frame_w / 2.0),
                               open_bot_z + stack_h / 2.0)),
            material=charcoal,
            name=f"face_stile_{tag}",
        )

    # ---------- two fixed internal shelves ----------
    shelf_thk = 0.008
    shelf_depth = D - wall - 0.020   # nearly full depth, stopping short of back
    shelf_width = inner_W - 0.004    # slightly narrower than the opening
    n_shelves = 2
    for i in range(n_shelves):
        frac = (i + 1) / (n_shelves + 1)
        sz = open_bot_z + frac * stack_h
        cab.visual(
            Box((shelf_depth, shelf_width, shelf_thk)),
            origin=Origin(xyz=(wall + shelf_depth / 2.0, 0.0, sz)),
            material=shelf_steel,
            name=f"shelf_{i}",
        )
        # Shelf support lips (small angle brackets at each side).
        for s, side_tag in ((1, "left"), (-1, "right")):
            cab.visual(
                Box((0.040, 0.006, 0.020)),
                origin=Origin(xyz=(wall + 0.060, s * (shelf_width / 2.0 + 0.003),
                                   sz - 0.014)),
                material=charcoal_dark,
                name=f"shelf_bracket_{i}_{side_tag}",
            )

    # Small maker badge near the top front edge.
    cab.visual(
        Box((0.006, 0.055, 0.022)),
        origin=Origin(xyz=(D + 0.001, -0.105, H - 0.030)),
        material=badge_red,
        name="top_badge",
    )

    cab.inertial = Inertial.from_geometry(Box((D, W, H)), mass=35.0)

    # ===================================================================
    # DOOR (REVOLUTE, vertical axis at the left hinge line)
    # ===================================================================
    door = model.part("door")

    face_thk = 0.018              # door panel thickness
    door_W = inner_W + 0.004      # slightly wider than opening to overlap frame
    door_H = stack_h - 2 * frame_w + 0.004  # fill the opening height
    # Door part frame is at the hinge line. The door panel extends along -Y
    # (toward the right) and along -X (slightly proud of the front plane).
    door_center_y = -door_W / 2.0
    door_center_x = -face_thk / 2.0

    # Main door panel.
    door.visual(
        Box((face_thk, door_W, door_H)),
        origin=Origin(xyz=(door_center_x, door_center_y, 0.0)),
        material=charcoal,
        name="door_panel",
    )

    # Reinforcement emboss (a raised rectangular stiffener on the outer face,
    # common on sheet-metal utility cabinets).
    emboss_thk = 0.003
    emboss_W = door_W - 0.060
    emboss_H = door_H - 0.100
    door.visual(
        Box((emboss_thk, emboss_W, emboss_H)),
        origin=Origin(xyz=(door_center_x + face_thk / 2.0 + emboss_thk / 2.0,
                           door_center_y, 0.0)),
        material=charcoal_dark,
        name="door_emboss",
    )

    # Recessed pull handle on the free (right) edge of the door.
    handle_x = door_center_x + face_thk / 2.0
    handle_y = door_center_y - door_W / 2.0 + 0.045   # near the free edge
    door.visual(
        Box((0.012, 0.030, 0.100)),
        origin=Origin(xyz=(handle_x + 0.004, handle_y, 0.0)),
        material=charcoal_dark,
        name="handle_surround",
    )
    door.visual(
        Box((0.014, 0.016, 0.080)),
        origin=Origin(xyz=(handle_x + 0.006, handle_y, 0.0)),
        material=handle_dark,
        name="pull_handle",
    )

    # ---------- barrel hinges (two, top and bottom) ----------
    hinge_length = 0.080
    hinge_geom = BarrelHingeGeometry(
        hinge_length,
        leaf_width_a=0.022,
        leaf_width_b=0.018,
        leaf_thickness=0.0024,
        pin_diameter=0.004,
        knuckle_count=5,
        holes_a=HingeHolePattern(style="round", count=3, diameter=0.0032, edge_margin=0.010),
        holes_b=HingeHolePattern(style="round", count=3, diameter=0.0032, edge_margin=0.010),
    )
    hinge_mesh = mesh_from_geometry(hinge_geom, "barrel_hinge")

    # Hinge positions: near top and bottom of the door, at the hinge line.
    # The BarrelHingeGeometry is built around local Z (pin axis) which matches
    # our vertical hinge axis. Place on the door part frame (hinge line).
    hinge_z_offsets = [
        -door_H / 2.0 + hinge_length / 2.0 + 0.040,   # bottom hinge
        door_H / 2.0 - hinge_length / 2.0 - 0.040,    # top hinge
    ]
    for i, dz in enumerate(hinge_z_offsets):
        door.visual(
            hinge_mesh,
            origin=Origin(xyz=(0.0, 0.0, dz)),
            material=hinge_grey,
            name=f"hinge_{i}",
        )

    door.inertial = Inertial.from_geometry(
        Box((face_thk, door_W, door_H)), mass=6.0,
        origin=Origin(xyz=(door_center_x, door_center_y, 0.0)),
    )

    # ---------- articulation ----------
    # The joint origin is at the left hinge line on the cabinet front face.
    # Left edge of opening (inside the left side wall): y = W/2 - wall.
    # The mid-height of the opening: z = open_bot_z + stack_h / 2.
    hinge_y = W / 2.0 - wall
    hinge_z = open_bot_z + stack_h / 2.0

    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cab,
        child=door,
        origin=Origin(xyz=(D, hinge_y, hinge_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.5, lower=0.0, upper=2.3),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cab = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("cabinet_to_door")

    # --- Cabinet base rests on the floor (z=0), upright and tall ---
    cb = ctx.part_world_aabb(cab)
    assert cb is not None
    ctx.check("base_at_floor", abs(cb[0][2]) < 0.004, details=f"cabinet min z={cb[0][2]:.4f}")
    ctx.check("tall_cabinet", cb[1][2] > 1.2, details=f"cabinet top z={cb[1][2]:.3f}")
    width_y = cb[1][1] - cb[0][1]
    height_z = cb[1][2] - cb[0][2]
    ctx.check("upright", height_z > width_y * 2.5, details=f"h={height_z:.3f}, w={width_y:.3f}")

    # --- Single door exists and is revolute on a vertical axis ---
    ctx.check("door_exists", door is not None, details="door part missing")
    ctx.check(
        "hinge_is_revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={hinge.articulation_type}",
    )
    ctx.check(
        "hinge_axis_is_vertical",
        abs(hinge.axis[2]) > 0.99,
        details=f"axis={hinge.axis}",
    )

    # --- Cabinet has two internal shelves ---
    cab_visual_names = {v.name for v in cab.visuals}
    ctx.check(
        "two_internal_shelves",
        "shelf_0" in cab_visual_names and "shelf_1" in cab_visual_names,
        details=f"cabinet visuals={sorted(cab_visual_names)}",
    )

    # --- Door has a pull handle on its free edge ---
    door_visual_names = {v.name for v in door.visuals}
    ctx.check(
        "door_has_pull_handle",
        "pull_handle" in door_visual_names,
        details=f"door visuals={sorted(door_visual_names)}",
    )

    # --- Door is flush at the cabinet front when closed ---
    front_x = cb[1][0]
    door_panel_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    assert door_panel_aabb is not None
    ctx.check(
        "door_flush_closed",
        abs(door_panel_aabb[1][0] - front_x) < 0.025,
        details=f"door front x={door_panel_aabb[1][0]:.3f}, cabinet front={front_x:.3f}",
    )

    # --- Door swings open: free edge moves outward (+X) at positive angle ---
    rest_pos = ctx.part_world_position(door)
    with ctx.pose({hinge: hinge.motion_limits.upper}):
        open_pos = ctx.part_world_position(door)
        # The handle (free edge) should move significantly in +X.
        handle_aabb = ctx.part_element_world_aabb(door, elem="pull_handle")
    assert rest_pos is not None and open_pos is not None and handle_aabb is not None
    ctx.check(
        "door_swallows_outward",
        handle_aabb[1][0] > front_x + 0.10,
        details=f"handle front x at open={handle_aabb[1][0]:.3f}, closed front={front_x:.3f}",
    )

    # --- Door retains connection at the hinge when open (doesn't fly off) ---
    with ctx.pose({hinge: hinge.motion_limits.upper}):
        door_open_aabb = ctx.part_world_aabb(door)
    assert door_open_aabb is not None
    # The hinge edge should stay near the cabinet front-left corner.
    ctx.check(
        "door_hinge_retained",
        door_open_aabb[0][1] > cb[0][1] - 0.10,
        details=f"door min y open={door_open_aabb[0][1]:.3f}, cab min y={cb[0][1]:.3f}",
    )

    # --- Shelves span the interior width and sit between top/bottom panels ---
    for i in range(2):
        shelf_aabb = ctx.part_element_world_aabb(cab, elem=f"shelf_{i}")
        assert shelf_aabb is not None
        ctx.check(
            f"shelf_{i}_within_cabinet_height",
            shelf_aabb[0][2] > cb[0][2] + 0.05 and shelf_aabb[1][2] < cb[1][2] - 0.05,
            details=f"shelf z=[{shelf_aabb[0][2]:.3f}, {shelf_aabb[1][2]:.3f}]",
        )

    # --- Door panel laps over the face frame when closed (intentional) ---
    ctx.allow_overlap(
        cab, door,
        elem_a="face_stile_right",
        elem_b="door_panel",
        reason="The closed door panel intentionally laps over the right face stile to seal the opening, as on a real sheet-metal utility cabinet.",
    )
    ctx.allow_overlap(
        cab, door,
        elem_a="face_stile_left",
        elem_b="hinge_1",
        reason="The top barrel hinge barrel sits at the left stile edge where the door mounts to the frame, causing small local overlap with the stile.",
    )
    ctx.allow_overlap(
        cab, door,
        elem_a="face_stile_left",
        elem_b="hinge_0",
        reason="The bottom barrel hinge barrel sits at the left stile edge where the door mounts to the frame, causing small local overlap with the stile.",
    )

    ctx.expect_overlap(
        door, cab,
        axes="z",
        elem_a="door_panel",
        elem_b="face_stile_left",
        min_overlap=1.0,
        name="door_covers_opening_height",
    )
    ctx.expect_contact(
        door, cab,
        elem_a="door_panel",
        elem_b="face_stile_right",
        contact_tol=0.012,
        name="door_seats_against_right_stile",
    )

    return ctx.report()


object_model = build_object_model()
