from __future__ import annotations

# Tall double-door utility/fire cabinet (dark charcoal steel).
#
# Fork variant of the four-drawer filing cabinet: same carcass proportions,
# plinth, sides, top and back, but the drawer stack is replaced by a pair of
# tall double doors meeting at a center seam. The left door is hinged on the
# left edge and the right door on the right edge, each swinging outward on its
# own vertical-axis REVOLUTE hinge. Three fixed internal shelves sit behind
# the doors. Each door carries a recessed edge pull near the meeting edge.
#
# World layout: cabinet front faces +X; width along Y; height along +Z.
# Base plinth rests at z=0.

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_door_fire_cabinet")

    charcoal = model.material("charcoal_steel", rgba=(0.15, 0.16, 0.18, 1.0))
    charcoal_dark = model.material("charcoal_dark", rgba=(0.075, 0.080, 0.090, 1.0))
    interior_dark = model.material("interior_dark_steel", rgba=(0.105, 0.110, 0.125, 1.0))
    handle_dark = model.material("handle_dark", rgba=(0.09, 0.09, 0.10, 1.0))
    badge_red = model.material("badge_red", rgba=(0.78, 0.20, 0.13, 1.0))
    shelf_grey = model.material("shelf_grey", rgba=(0.22, 0.23, 0.25, 1.0))

    # ---------- key dimensions (meters) ----------
    W = 0.380          # width (Y)
    D = 0.600          # depth (X), front at x=D, back at x=0
    H = 1.300          # total height (Z)
    wall = 0.014       # carcass wall thickness
    plinth_h = 0.060   # recessed base plinth height
    plinth_inset = 0.020

    n_shelves = 3
    door_thickness = 0.016

    # ===================================================================
    # ROOT: cabinet carcass
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

    # Small maker badge on the top panel front edge.
    cab.visual(
        Box((0.006, 0.055, wall)),
        origin=Origin(xyz=(D + 0.001, -0.105, H - wall / 2.0)),
        material=badge_red,
        name="top_badge",
    )

    cab.inertial = Inertial.from_geometry(Box((D, W, H)), mass=42.0)

    # ---------- internal shelves ----------
    open_top_z = H - wall
    open_bot_z = body_bottom_z + wall
    cavity_h = open_top_z - open_bot_z
    inner_W = W - 2 * wall

    # Shelves span from near the back wall to just behind the closed door back
    # face, and overlap slightly into the side walls for structural contact.
    shelf_x_min = wall - 0.004       # small embed into back wall
    shelf_x_max = D - door_thickness - 0.005  # clears door back face
    shelf_dx = shelf_x_max - shelf_x_min
    shelf_dy = inner_W + 2 * 0.004   # slight embed into each side wall
    shelf_thick = 0.008

    for i in range(n_shelves):
        sz = open_bot_z + cavity_h * (i + 1) / (n_shelves + 1)
        cab.visual(
            Box((shelf_dx, shelf_dy, shelf_thick)),
            origin=Origin(xyz=((shelf_x_min + shelf_x_max) / 2.0, 0.0, sz)),
            material=shelf_grey,
            name=f"shelf_{i}",
        )

    # ===================================================================
    # DOORS (REVOLUTE, vertical axis, swing outward)
    # ===================================================================
    center_gap = 0.003               # gap between doors at center seam
    door_w = (inner_W - center_gap) / 2.0
    door_h = cavity_h - 0.004        # small clearance top and bottom
    hinge_y = W / 2.0 - wall         # hinge at inner face of side wall

    # Each door: part frame origin at the hinge line (front face, side edge,
    # bottom of opening). Panel extends toward center (-s*Y), inward (-X),
    # and upward (+Z) in the local frame.
    for i, (s, tag, axis_z) in enumerate(
        ((1, "left", 1.0), (-1, "right", -1.0))
    ):
        door = model.part(f"door_{tag}")

        # Main door panel (sheet steel).
        door.visual(
            Box((door_thickness, door_w, door_h)),
            origin=Origin(xyz=(-door_thickness / 2.0,
                               -s * door_w / 2.0,
                               door_h / 2.0)),
            material=charcoal,
            name="door_panel",
        )

        # Recessed edge pull near the free edge (center seam side).
        # A proud surround bar with a dark inset grip slot.
        pull_y = -s * (door_w - 0.025)
        pull_z = door_h * 0.50

        door.visual(
            Box((0.012, 0.030, 0.120)),
            origin=Origin(xyz=(0.004, pull_y, pull_z)),
            material=charcoal_dark,
            name="handle_surround",
        )
        door.visual(
            Box((0.014, 0.014, 0.100)),
            origin=Origin(xyz=(0.006, pull_y, pull_z)),
            material=handle_dark,
            name="pull_handle",
        )

        door.inertial = Inertial.from_geometry(
            Box((door_thickness, door_w, door_h)),
            mass=5.0,
            origin=Origin(xyz=(-door_thickness / 2.0,
                               -s * door_w / 2.0,
                               door_h / 2.0)),
        )

        # Vertical-axis revolute hinge at the side edge of the front opening.
        # axis=(0,0,+1) for left door, (0,0,-1) for right door so that
        # positive q swings the free edge outward (+X) in both cases.
        model.articulation(
            f"cabinet_to_door_{tag}",
            ArticulationType.REVOLUTE,
            parent=cab,
            child=door,
            origin=Origin(xyz=(D, s * hinge_y, open_bot_z)),
            axis=(0.0, 0.0, axis_z),
            motion_limits=MotionLimits(
                effort=10.0, velocity=1.5,
                lower=0.0, upper=1.5,  # ~86 degrees open
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cab = object_model.get_part("cabinet")

    # --- Cabinet base rests on the floor, upright and tall ---
    cb = ctx.part_world_aabb(cab)
    assert cb is not None
    ctx.check("base_at_floor", abs(cb[0][2]) < 0.004,
              details=f"cabinet min z={cb[0][2]:.4f}")
    ctx.check("tall_cabinet", cb[1][2] > 1.2,
              details=f"cabinet top z={cb[1][2]:.3f}")
    width_y = cb[1][1] - cb[0][1]
    height_z = cb[1][2] - cb[0][2]
    ctx.check("upright", height_z > width_y * 2.5,
              details=f"h={height_z:.3f}, w={width_y:.3f}")

    door_left = object_model.get_part("door_left")
    door_right = object_model.get_part("door_right")
    hinge_left = object_model.get_articulation("cabinet_to_door_left")
    hinge_right = object_model.get_articulation("cabinet_to_door_right")

    # --- Both hinges are revolute on a vertical (Z) axis ---
    for hinge, tag in ((hinge_left, "left"), (hinge_right, "right")):
        ctx.check(
            f"{tag}_hinge_vertical_axis",
            abs(hinge.axis[2]) > 0.99,
            details=f"axis={hinge.axis}",
        )

    # --- Hinges are on opposite sides of the cabinet ---
    left_hinge_y = hinge_left.origin.xyz[1]
    right_hinge_y = hinge_right.origin.xyz[1]
    ctx.check(
        "hinges_on_opposite_sides",
        left_hinge_y > 0.05 and right_hinge_y < -0.05,
        details=f"left_hinge_y={left_hinge_y:.3f}, right_hinge_y={right_hinge_y:.3f}",
    )

    front_x = cb[1][0]

    # --- Doors swing outward: panel max-X moves forward when opened ---
    for door, hinge, tag in (
        (door_left, hinge_left, "left"),
        (door_right, hinge_right, "right"),
    ):
        rest_panel = ctx.part_element_world_aabb(door, elem="door_panel")
        with ctx.pose({hinge: hinge.motion_limits.upper}):
            open_panel = ctx.part_element_world_aabb(door, elem="door_panel")
        assert rest_panel is not None and open_panel is not None
        ctx.check(
            f"door_{tag}_swings_outward",
            open_panel[1][0] > rest_panel[1][0] + 0.05,
            details=f"rest_max_x={rest_panel[1][0]:.3f}, open_max_x={open_panel[1][0]:.3f}",
        )

    # --- When closed, door panels are flush with the cabinet front ---
    for door, tag in ((door_left, "left"), (door_right, "right")):
        panel = ctx.part_element_world_aabb(door, elem="door_panel")
        assert panel is not None
        ctx.check(
            f"door_{tag}_flush_closed",
            abs(panel[1][0] - front_x) < 0.020,
            details=f"panel front x={panel[1][0]:.3f}, cabinet front={front_x:.3f}",
        )

    # --- Doors meet near the center with a small gap ---
    left_panel = ctx.part_element_world_aabb(door_left, elem="door_panel")
    right_panel = ctx.part_element_world_aabb(door_right, elem="door_panel")
    assert left_panel is not None and right_panel is not None
    # Left door min-Y is its free edge; right door max-Y is its free edge.
    left_inner_y = left_panel[0][1]
    right_inner_y = right_panel[1][1]
    ctx.check(
        "doors_center_gap_small",
        0.0 <= left_inner_y - right_inner_y <= 0.010,
        details=f"left_inner_y={left_inner_y:.4f}, right_inner_y={right_inner_y:.4f}",
    )

    # --- Internal shelves exist and are in the cavity ---
    for i in range(3):
        name = f"shelf_{i}"
        v = cab.get_visual(name)
        ctx.check(f"{name}_exists", v is not None,
                  details=f"visual {name} not found on cabinet")

    # --- Shelves are vertically stacked ---
    shelf_zs = []
    for i in range(3):
        v = cab.get_visual(f"shelf_{i}")
        if v is not None:
            shelf_zs.append(v.origin.xyz[2])
    ctx.check("shelves_stacked", len(shelf_zs) == 3 and all(
        shelf_zs[k] < shelf_zs[k + 1] for k in range(2)),
        details=f"shelf zs={shelf_zs}")

    # --- Each door has a recessed pull handle ---
    for door, tag in ((door_left, "door_left"), (door_right, "door_right")):
        vnames = {v.name for v in door.visuals}
        ctx.check(
            f"{tag}_has_pull",
            "pull_handle" in vnames and "handle_surround" in vnames,
            details=f"visuals={sorted(vnames)}",
        )

    return ctx.report()


object_model = build_object_model()
