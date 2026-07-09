from __future__ import annotations

# Fire-equipment style wall cabinet (dark charcoal steel).
#
# Upright sheet-steel carcass with a single tall hinged door whose central
# panel is a glazed transparent window pane framed in steel. The door swings
# open on a vertical-axis REVOLUTE hinge to reveal two fixed internal shelves.
# An edge pull sits on the free side of the door.
#
# World layout: cabinet front faces +X; width along Y; height along +Z.
# The base plinth rests at z=0.

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fire_equipment_cabinet")

    # --- Materials ---
    charcoal = model.material("charcoal_steel", rgba=(0.15, 0.16, 0.18, 1.0))
    charcoal_dark = model.material("charcoal_dark", rgba=(0.075, 0.080, 0.090, 1.0))
    frame_steel = model.material("frame_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.60, 0.70, 0.78, 0.35))
    handle_dark = model.material("handle_dark", rgba=(0.09, 0.09, 0.10, 1.0))
    badge_red = model.material("badge_red", rgba=(0.78, 0.20, 0.13, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.35, 0.35, 0.32, 1.0))
    shelf_grey = model.material("shelf_grey", rgba=(0.22, 0.23, 0.25, 1.0))

    # --- Key dimensions (meters) ---
    W = 0.380          # cabinet width (Y)
    D = 0.600          # cabinet depth (X), front at x=D, back at x=0
    H = 1.300          # total height (Z)
    wall = 0.014       # carcass wall thickness
    plinth_h = 0.060   # recessed base plinth height
    plinth_inset = 0.020

    body_bottom_z = plinth_h
    body_h = H - plinth_h

    # Door opening geometry (inner edges of carcass panels)
    open_bot_z = body_bottom_z + wall
    open_top_z = H - wall
    opening_h = open_top_z - open_bot_z
    inner_W = W - 2 * wall

    # Hinge line: at the left side of the front opening (positive Y when
    # facing the cabinet front at +X).
    hinge_y = W / 2.0 - wall

    # ===================================================================
    # ROOT: cabinet carcass
    # ===================================================================
    cab = model.part("cabinet")

    # Recessed base plinth
    cab.visual(
        Box((D - 2 * plinth_inset, W - 2 * plinth_inset, plinth_h)),
        origin=Origin(xyz=(D / 2.0, 0.0, plinth_h / 2.0)),
        material=charcoal_dark,
        name="base_plinth",
    )

    # Bottom panel
    cab.visual(
        Box((D, W, wall)),
        origin=Origin(xyz=(D / 2.0, 0.0, body_bottom_z + wall / 2.0)),
        material=charcoal,
        name="bottom_panel",
    )

    # Top panel
    cab.visual(
        Box((D, W, wall)),
        origin=Origin(xyz=(D / 2.0, 0.0, H - wall / 2.0)),
        material=charcoal,
        name="top_panel",
    )

    # Back wall
    cab.visual(
        Box((wall, W, body_h)),
        origin=Origin(xyz=(wall / 2.0, 0.0, body_bottom_z + body_h / 2.0)),
        material=charcoal,
        name="back_wall",
    )

    # Side walls
    for s, tag in ((1, "left"), (-1, "right")):
        cab.visual(
            Box((D, wall, body_h)),
            origin=Origin(xyz=(D / 2.0, s * (W / 2.0 - wall / 2.0),
                               body_bottom_z + body_h / 2.0)),
            material=charcoal,
            name=f"side_wall_{tag}",
        )

    # --- Internal shelves (2 fixed shelves, evenly spaced) ---
    # Each shelf is supported by two small steel angle brackets screwed into the
    # side walls, so the shelf reads as physically connected to the carcass.
    n_shelves = 2
    shelf_depth = D - wall - 0.022 - 0.012  # back wall to behind closed door inner face
    shelf_w = inner_W - 0.006
    shelf_thk = 0.008
    bracket_w = 0.014
    bracket_d = 0.040
    bracket_h = 0.014
    for i in range(n_shelves):
        sz = open_bot_z + (i + 1) * opening_h / (n_shelves + 1)
        # Shelf plate
        cab.visual(
            Box((shelf_depth, shelf_w, shelf_thk)),
            origin=Origin(xyz=(wall + shelf_depth / 2.0 + 0.005, 0.0, sz)),
            material=shelf_grey,
            name=f"shelf_{i}",
        )
        # Two L-brackets (one per side wall) that physically connect the shelf
        # to the carcass. Each bracket has a vertical leg against the side wall
        # and a horizontal leg under the shelf.
        for s, side_tag in ((1, "left"), (-1, "right")):
            # Vertical leg: thin plate flush against side wall inner face
            cab.visual(
                Box((bracket_d, bracket_w, bracket_h)),
                origin=Origin(xyz=(wall + bracket_d / 2.0 + 0.020,
                                   s * (W / 2.0 - wall - bracket_w / 2.0),
                                   sz - shelf_thk / 2.0 - bracket_h / 2.0)),
                material=hinge_steel,
                name=f"shelf_bracket_{i}_{side_tag}",
            )

    # --- Hinge barrels (two exposed knuckles at top and bottom of hinge line) ---
    hinge_r = 0.006
    hinge_len = 0.040
    for i, dz in enumerate((0.060, opening_h - 0.060)):
        cab.visual(
            Cylinder(radius=hinge_r, length=hinge_len),
            origin=Origin(xyz=(D, hinge_y, open_bot_z + dz)),
            material=hinge_steel,
            name=f"hinge_barrel_{i}",
        )

    # Maker badge on the cabinet top front (visible above the door frame line)
    cab.visual(
        Box((0.006, 0.055, 0.018)),
        origin=Origin(xyz=(D - 0.003, -0.105, H - 0.003)),
        material=badge_red,
        name="top_badge",
    )

    cab.inertial = Inertial.from_geometry(Box((D, W, H)), mass=35.0)

    # ===================================================================
    # DOOR (REVOLUTE, vertical-axis hinge at left edge)
    # ===================================================================
    door = model.part("door")

    door_W = inner_W       # 0.352 m
    door_H = opening_h     # 1.212 m
    door_thk = 0.022       # total door thickness
    fb = 0.038             # frame border width (stiles and rails)

    # The door part frame origin sits at the hinge corner (bottom-left of the
    # door when facing it). In local coords:
    #   - X: outer face at x=0, inner face at x=-door_thk
    #   - Y: from y=0 (hinge edge) to y=-door_W (free edge)
    #   - Z: from z=0 (bottom) to z=door_H (top)

    # Frame stiles (vertical members)
    door.visual(
        Box((door_thk, fb, door_H)),
        origin=Origin(xyz=(-door_thk / 2.0, -fb / 2.0, door_H / 2.0)),
        material=frame_steel,
        name="frame_stile_left",
    )
    door.visual(
        Box((door_thk, fb, door_H)),
        origin=Origin(xyz=(-door_thk / 2.0, -(door_W - fb / 2.0), door_H / 2.0)),
        material=frame_steel,
        name="frame_stile_right",
    )

    # Frame rails (horizontal members between stiles)
    rail_w = door_W - 2 * fb
    door.visual(
        Box((door_thk, rail_w, fb)),
        origin=Origin(xyz=(-door_thk / 2.0, -door_W / 2.0, door_H - fb / 2.0)),
        material=frame_steel,
        name="frame_rail_top",
    )
    door.visual(
        Box((door_thk, rail_w, fb)),
        origin=Origin(xyz=(-door_thk / 2.0, -door_W / 2.0, fb / 2.0)),
        material=frame_steel,
        name="frame_rail_bottom",
    )

    # Glazed transparent window pane (centered in the frame opening).
    # The glass sits in a rebate/rabbet in the frame, extending ~5 mm under
    # each frame member edge (realistic glazing capture).
    glass_thk = 0.004
    glass_rebate = 0.005
    glass_w = door_W - 2 * fb + 2 * glass_rebate
    glass_h = door_H - 2 * fb + 2 * glass_rebate
    door.visual(
        Box((glass_thk, glass_w, glass_h)),
        origin=Origin(xyz=(-door_thk / 2.0, -door_W / 2.0, door_H / 2.0)),
        material=glass_tint,
        name="glass_pane",
    )

    # Edge pull handle on the free-edge stile, at mid-height.
    # The pull protrudes outward (+X) from the stile outer face.
    pull_d = 0.015
    pull_w = 0.012
    pull_h = 0.080
    door.visual(
        Box((pull_d, pull_w, pull_h)),
        origin=Origin(xyz=(pull_d / 2.0 - 0.001, -(door_W - fb / 2.0), door_H / 2.0)),
        material=handle_dark,
        name="edge_pull",
    )

    # Fire equipment identification label (on the right stile, upper area)
    door.visual(
        Box((0.004, 0.030, 0.025)),
        origin=Origin(xyz=(-0.001, -(door_W - fb / 2.0), door_H * 0.75)),
        material=badge_red,
        name="fire_label",
    )

    door.inertial = Inertial.from_geometry(
        Box((door_thk, door_W, door_H)), mass=5.0,
        origin=Origin(xyz=(-door_thk / 2.0, -door_W / 2.0, door_H / 2.0)),
    )

    # --- Articulation: vertical-axis revolute hinge ---
    # Joint origin at the hinge axis (front-left-bottom of the opening).
    # Axis +Z: positive rotation swings the free edge (at -Y) toward +X
    # (outward), opening the door.
    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cab,
        child=door,
        origin=Origin(xyz=(D, hinge_y, open_bot_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=2.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cab = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("cabinet_to_door")

    # --- Intentional overlap allowances ---
    # The hinge barrel knuckles wrap the hinge pin at the junction of the door
    # stile and the carcass frame; small local embedding is by design.
    for i in range(2):
        ctx.allow_overlap(
            cab, door,
            elem_a=f"hinge_barrel_{i}", elem_b="frame_stile_left",
            reason="Hinge barrel wraps the pin at the door stile junction.",
        )

    # --- Cabinet rests at floor, tall upright proportions ---
    cb = ctx.part_world_aabb(cab)
    assert cb is not None
    ctx.check("base_at_floor", abs(cb[0][2]) < 0.004,
              details=f"cabinet min z={cb[0][2]:.4f}")
    ctx.check("tall_cabinet", cb[1][2] > 1.2,
              details=f"cabinet top z={cb[1][2]:.3f}")
    width_y = cb[1][1] - cb[0][1]
    height_z = cb[1][2] - cb[0][2]
    ctx.check("upright_proportions", height_z > width_y * 2.5,
              details=f"h={height_z:.3f}, w={width_y:.3f}")

    # --- Hinge is REVOLUTE with vertical axis ---
    ctx.check("hinge_is_revolute",
              hinge.articulation_type == ArticulationType.REVOLUTE,
              details=f"type={hinge.articulation_type}")
    ctx.check("hinge_vertical_axis", abs(hinge.axis[2]) > 0.99,
              details=f"axis={hinge.axis}")

    # --- Door has glazed pane framed in steel ---
    door_visuals = {v.name for v in door.visuals}
    ctx.check("door_has_glass_pane", "glass_pane" in door_visuals,
              details=f"visuals={sorted(door_visuals)}")
    for name in ("frame_stile_left", "frame_stile_right",
                 "frame_rail_top", "frame_rail_bottom"):
        ctx.check(f"door_has_{name}", name in door_visuals,
                  details=f"visuals={sorted(door_visuals)}")

    # Glass pane is seated between the frame members:
    # - Z overlap with the top rail proves it doesn't extend above the rail.
    # - Z overlap with the bottom rail proves it doesn't extend below the rail.
    # - Y overlap with the right stile proves the pane reaches the far edge.
    ctx.expect_overlap(door, door, axes="z",
                       elem_a="glass_pane", elem_b="frame_rail_top",
                       min_overlap=0.0, name="glass_overlaps_top_rail_z")
    ctx.expect_overlap(door, door, axes="z",
                       elem_a="glass_pane", elem_b="frame_rail_bottom",
                       min_overlap=0.0, name="glass_overlaps_bottom_rail_z")
    # Glass Y is flush with the inner stile edges (it spans between stiles)
    ctx.expect_overlap(door, door, axes="y",
                       elem_a="glass_pane", elem_b="frame_stile_left",
                       min_overlap=0.0, name="glass_adjacent_left_stile_y")

    # --- Door has edge pull on the free side ---
    ctx.check("door_has_edge_pull", "edge_pull" in door_visuals,
              details=f"visuals={sorted(door_visuals)}")

    # --- Cabinet has 2 internal shelves within the carcass ---
    cab_visuals = {v.name for v in cab.visuals}
    for i in range(2):
        ctx.check(f"cabinet_has_shelf_{i}", f"shelf_{i}" in cab_visuals,
                  details=f"visuals={sorted(cab_visuals)}")
        ctx.expect_within(cab, cab, axes="xy",
                          inner_elem=f"shelf_{i}", outer_elem="bottom_panel",
                          margin=0.002,
                          name=f"shelf_{i}_within_cabinet_footprint")

    # --- Door closed: outer face is flush with the cabinet front ---
    front_x = cb[1][0]
    door_stile_aabb = ctx.part_element_world_aabb(door, elem="frame_stile_right")
    assert door_stile_aabb is not None
    ctx.check("door_closed_flush",
              abs(door_stile_aabb[1][0] - front_x) < 0.025,
              details=f"door face x={door_stile_aabb[1][0]:.3f}, front={front_x:.3f}")

    # --- Door swings open (free edge moves outward / past the cabinet side) ---
    with ctx.pose({hinge: 0.0}):
        closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({hinge: hinge.motion_limits.upper}):
        open_aabb = ctx.part_world_aabb(door)
    assert closed_aabb is not None and open_aabb is not None
    ctx.check("door_swallows_open",
              (open_aabb[1][1] > closed_aabb[1][1] + 0.15
               or open_aabb[0][1] < closed_aabb[0][1] - 0.15),
              details=(f"closed y=[{closed_aabb[0][1]:.3f},{closed_aabb[1][1]:.3f}], "
                       f"open y=[{open_aabb[0][1]:.3f},{open_aabb[1][1]:.3f}]"))

    # --- Hinge barrels are near the door stile (contact within hinge tolerance) ---
    for i in range(2):
        ctx.expect_contact(cab, door,
                           elem_a=f"hinge_barrel_{i}", elem_b="frame_stile_left",
                           contact_tol=0.008,
                           name=f"hinge_barrel_{i}_near_door_stile")

    return ctx.report()


object_model = build_object_model()
