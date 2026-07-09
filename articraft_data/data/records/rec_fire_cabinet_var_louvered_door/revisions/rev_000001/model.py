from __future__ import annotations

# Tall louvered utility cabinet (dark charcoal steel).
#
# Variant of the four-drawer filing cabinet: same upright carcass with recessed
# plinth, sides, top, back, and bottom, but fronted by a single tall hinged door
# whose face is filled with horizontal louvered ventilation slats (a regular
# stack of angled blades).  The door swings open on a vertical-axis REVOLUTE
# hinge at the left edge to reveal two fixed internal shelves.  An edge pull
# sits on the free (right) side of the door.
#
# World layout: the cabinet front faces +X; width is along Y; height along +Z.
# The base plinth rests at z=0.

import math

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


# ---------------------------------------------------------------------------
# Shared geometry helper: one louver slat
# ---------------------------------------------------------------------------

def _louver_slat(depth: float, width: float, thickness: float) -> Box:
    """Return a thin box representing one angled louver blade."""
    return Box((depth, width, thickness))


# ---------------------------------------------------------------------------
# Model construction
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="louvered_utility_cabinet")

    # ---- materials ----
    charcoal = model.material("charcoal_steel", rgba=(0.15, 0.16, 0.18, 1.0))
    charcoal_dark = model.material("charcoal_dark", rgba=(0.075, 0.080, 0.090, 1.0))
    interior_dark = model.material("interior_dark_steel", rgba=(0.105, 0.110, 0.125, 1.0))
    handle_dark = model.material("handle_dark", rgba=(0.09, 0.09, 0.10, 1.0))
    shelf_steel = model.material("shelf_steel", rgba=(0.20, 0.21, 0.23, 1.0))
    badge_red = model.material("badge_red", rgba=(0.78, 0.20, 0.13, 1.0))

    # ---- key dimensions (meters) ----
    W = 0.380            # cabinet width  (Y)
    D = 0.600            # cabinet depth  (X), front at x=D, back at x=0
    H = 1.300            # cabinet height (Z)
    wall = 0.014         # carcass wall thickness
    plinth_h = 0.060     # recessed base plinth height
    plinth_inset = 0.020 # plinth inset from cabinet edges

    body_bottom_z = plinth_h
    body_h = H - plinth_h       # carcass body height above plinth

    open_bot_z = body_bottom_z + wall   # bottom of the front opening
    open_top_z = H - wall               # top of the front opening
    inner_W = W - 2 * wall              # inner width between side walls
    open_H = open_top_z - open_bot_z    # height of the front opening

    # ===================================================================
    # ROOT: cabinet carcass
    # ===================================================================
    cab = model.part("cabinet")

    # -- Recessed base plinth --
    cab.visual(
        Box((D - 2 * plinth_inset, W - 2 * plinth_inset, plinth_h)),
        origin=Origin(xyz=(D / 2.0, 0.0, plinth_h / 2.0)),
        material=charcoal_dark,
        name="base_plinth",
    )

    # -- Bottom panel --
    cab.visual(
        Box((D, W, wall)),
        origin=Origin(xyz=(D / 2.0, 0.0, body_bottom_z + wall / 2.0)),
        material=charcoal,
        name="bottom_panel",
    )

    # -- Top panel --
    cab.visual(
        Box((D, W, wall)),
        origin=Origin(xyz=(D / 2.0, 0.0, H - wall / 2.0)),
        material=charcoal,
        name="top_panel",
    )

    # -- Back wall --
    cab.visual(
        Box((wall, W, body_h)),
        origin=Origin(xyz=(wall / 2.0, 0.0, body_bottom_z + body_h / 2.0)),
        material=charcoal,
        name="back_wall",
    )

    # -- Side walls --
    for s, tag in ((1, "left"), (-1, "right")):
        cab.visual(
            Box((D, wall, body_h)),
            origin=Origin(xyz=(D / 2.0, s * (W / 2.0 - wall / 2.0),
                               body_bottom_z + body_h / 2.0)),
            material=charcoal,
            name=f"side_wall_{tag}",
        )

    # -- Two fixed internal shelves (inline decorations) --
    shelf_W = inner_W - 0.006
    shelf_D = D - wall - 0.020
    shelf_thk = 0.012
    shelf_x = wall + shelf_D / 2.0        # centered depth-wise inside cabinet
    n_shelves = 2
    for i in range(n_shelves):
        shelf_z = open_bot_z + open_H * (i + 1) / (n_shelves + 1)
        cab.visual(
            Box((shelf_D, shelf_W, shelf_thk)),
            origin=Origin(xyz=(shelf_x, 0.0, shelf_z)),
            material=shelf_steel,
            name=f"shelf_{i}",
        )

    # -- Piano hinge barrel (continuous cylinder along the left front edge) --
    hinge_y = -(W / 2.0 - wall)          # at the inner face of left side wall
    hinge_barrel_r = 0.006
    hinge_barrel_h = open_H - 0.020      # slightly shorter than the opening
    cab.visual(
        Cylinder(radius=hinge_barrel_r, length=hinge_barrel_h),
        origin=Origin(xyz=(D, hinge_y,
                           open_bot_z + 0.010 + hinge_barrel_h / 2.0)),
        material=charcoal_dark,
        name="hinge_barrel",
    )

    # -- Small maker badge on the left side wall near the top front corner --
    badge_thk = 0.003
    badge_l = 0.055
    badge_h = 0.022
    cab.visual(
        Box((badge_l, badge_thk, badge_h)),
        origin=Origin(xyz=(D - 0.060,
                           W / 2.0 - badge_thk * 0.25,
                           H - 0.040)),
        material=badge_red,
        name="top_badge",
    )

    cab.inertial = Inertial.from_geometry(Box((D, W, H)), mass=35.0)

    # ===================================================================
    # DOOR (single tall hinged door with louver slats)
    # ===================================================================
    door_W = inner_W                      # door width matches the opening
    door_H = open_H                       # door height matches the opening
    door_thk = 0.008                      # thin sheet-steel back plate

    door = model.part("door")

    # -- Door back plate (structural skin, inner face visible when open) --
    door.visual(
        Box((door_thk, door_W, door_H)),
        origin=Origin(xyz=(-door_thk / 2.0, door_W / 2.0, door_H / 2.0)),
        material=charcoal,
        name="door_panel",
    )

    # -- Louver slats: regular stack of angled blades across the door face --
    slat_depth = 0.018                    # blade front-to-back depth
    slat_thk = 0.002                      # blade thickness
    slat_W = door_W - 0.040              # width with margin from door edges
    slat_angle_deg = 35.0
    slat_angle_rad = math.radians(slat_angle_deg)

    slat_margin_z = 0.035                 # vertical margin at top and bottom
    usable_h = door_H - 2.0 * slat_margin_z
    n_slats = 35
    actual_pitch = usable_h / n_slats

    for i in range(n_slats):
        z_i = slat_margin_z + actual_pitch * (i + 0.5)
        door.visual(
            _louver_slat(slat_depth, slat_W, slat_thk),
            origin=Origin(
                xyz=(0.0, door_W / 2.0, z_i),
                rpy=(0.0, slat_angle_rad, 0.0),
            ),
            material=charcoal,
            name=f"louver_slat_{i}",
        )

    # -- Edge pull handle on the free (right) side of the door --
    pull_h = 0.100
    pull_w = 0.022
    pull_d = 0.018
    door.visual(
        Box((pull_d, pull_w, pull_h)),
        origin=Origin(xyz=(pull_d / 2.0, door_W - pull_w / 2.0 - 0.004,
                           door_H / 2.0)),
        material=handle_dark,
        name="edge_pull",
    )

    door.inertial = Inertial.from_geometry(
        Box((door_thk + slat_depth, door_W, door_H)),
        mass=6.5,
        origin=Origin(xyz=(-door_thk / 2.0, door_W / 2.0, door_H / 2.0)),
    )

    # ===================================================================
    # ARTICULATION: vertical-axis revolute hinge
    # ===================================================================
    # Hinge at the left edge of the front opening.  The door part frame
    # origin sits at the hinge point; door geometry extends in +Y (width)
    # and +Z (height) from there.
    #
    # axis = (0, 0, -1): right-hand rule around -Z rotates the free edge
    # (at +Y from hinge) toward +X, so positive q opens the door outward.
    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cab,
        child=door,
        origin=Origin(xyz=(D, hinge_y, open_bot_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0, velocity=1.5, lower=0.0, upper=1.5,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cab = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("cabinet_to_door")

    # ---- Cabinet is tall, upright, and rests on the floor ----
    cb = ctx.part_world_aabb(cab)
    assert cb is not None
    ctx.check(
        "base_at_floor",
        abs(cb[0][2]) < 0.004,
        details=f"cabinet min z={cb[0][2]:.4f}",
    )
    ctx.check(
        "tall_cabinet",
        cb[1][2] > 1.2,
        details=f"cabinet top z={cb[1][2]:.3f}",
    )
    width_y = cb[1][1] - cb[0][1]
    height_z = cb[1][2] - cb[0][2]
    ctx.check(
        "upright_proportions",
        height_z > width_y * 2.5,
        details=f"h={height_z:.3f}, w={width_y:.3f}",
    )

    # ---- Door has louver slats (angled blades) ----
    door_names = {v.name for v in door.visuals}
    slat_names = sorted(n for n in door_names if n.startswith("louver_slat_"))
    ctx.check(
        "door_has_louver_slats",
        len(slat_names) >= 20,
        details=f"found {len(slat_names)} louver slats",
    )
    ctx.check(
        "door_has_panel",
        "door_panel" in door_names,
        details=f"door visuals={sorted(door_names)}",
    )

    # ---- Door has edge pull on the free side ----
    ctx.check(
        "door_has_edge_pull",
        "edge_pull" in door_names,
        details=f"door visuals={sorted(door_names)}",
    )

    # ---- Internal shelves exist on the cabinet ----
    cab_names = {v.name for v in cab.visuals}
    for i in range(2):
        ctx.check(
            f"has_shelf_{i}",
            f"shelf_{i}" in cab_names,
            details=f"cabinet visuals missing shelf_{i}",
        )

    # ---- Shelves are at distinct heights inside the cabinet ----
    shelf_aabbs = []
    for i in range(2):
        sa = ctx.part_element_world_aabb(cab, elem=f"shelf_{i}")
        assert sa is not None
        shelf_aabbs.append(sa)
    ctx.check(
        "shelves_at_distinct_heights",
        abs(shelf_aabbs[1][0][2] - shelf_aabbs[0][0][2]) > 0.20,
        details=f"shelf_0 z={shelf_aabbs[0][0][2]:.3f}, shelf_1 z={shelf_aabbs[1][0][2]:.3f}",
    )

    # ---- Shelves are contained within the cabinet footprint (XY) ----
    for i in range(2):
        sa = ctx.part_element_world_aabb(cab, elem=f"shelf_{i}")
        assert sa is not None
        ctx.check(
            f"shelf_{i}_within_cabinet_y",
            sa[0][1] >= cb[0][1] and sa[1][1] <= cb[1][1],
            details=f"shelf y=({sa[0][1]:.3f},{sa[1][1]:.3f}), cab y=({cb[0][1]:.3f},{cb[1][1]:.3f})",
        )
        ctx.check(
            f"shelf_{i}_within_cabinet_x",
            sa[0][0] >= cb[0][0] and sa[1][0] <= cb[1][0],
            details=f"shelf x=({sa[0][0]:.3f},{sa[1][0]:.3f}), cab x=({cb[0][0]:.3f},{cb[1][0]:.3f})",
        )

    # ---- Hinge is revolute with a vertical axis ----
    ctx.check(
        "hinge_is_revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={hinge.articulation_type}",
    )
    ctx.check(
        "hinge_vertical_axis",
        abs(hinge.axis[2]) > 0.99,
        details=f"axis={hinge.axis}",
    )

    # ---- Door is flush with cabinet front when closed (q=0) ----
    front_x = cb[1][0]
    door_aabb_closed = ctx.part_world_aabb(door)
    assert door_aabb_closed is not None
    ctx.check(
        "door_flush_when_closed",
        abs(door_aabb_closed[1][0] - front_x) < 0.025,
        details=f"door max x={door_aabb_closed[1][0]:.3f}, cabinet front={front_x:.3f}",
    )

    # ---- Door swings open: positive q moves free edge outward (+X) ----
    with ctx.pose({hinge: hinge.motion_limits.upper}):
        door_aabb_open = ctx.part_world_aabb(door)
    assert door_aabb_open is not None
    ctx.check(
        "door_opens_outward",
        door_aabb_open[1][0] > front_x + 0.05,
        details=f"open max x={door_aabb_open[1][0]:.3f}, front={front_x:.3f}",
    )

    # ---- At open pose, door clears the front opening so shelves are exposed ----
    with ctx.pose({hinge: hinge.motion_limits.upper}):
        door_open_aabb = ctx.part_world_aabb(door)
    assert door_open_aabb is not None
    ctx.check(
        "door_clears_opening_when_open",
        door_open_aabb[0][0] > front_x - 0.05,
        details=f"open min x={door_open_aabb[0][0]:.3f}, front={front_x:.3f}",
    )

    # ---- Hinge barrel / door panel intentional overlap allowance ----
    # The piano hinge barrel straddles the junction of the door edge and the
    # cabinet side wall, as in a real piano-hinged utility cabinet door.
    ctx.allow_overlap(
        cab, door,
        elem_a="hinge_barrel",
        elem_b="door_panel",
        reason=(
            "The hinge barrel sits at the junction of the door hinge edge and "
            "the cabinet side wall; half the barrel is embedded in the door "
            "panel as in a real piano-hinged cabinet door."
        ),
    )
    ctx.expect_contact(
        cab, door,
        elem_a="hinge_barrel",
        elem_b="door_panel",
        name="hinge_barrel_contacts_door_edge",
    )

    # ---- Door width fits within the cabinet opening ----
    ctx.expect_gap(
        door, cab,
        axis="y",
        max_penetration=0.004,
        positive_elem="door_panel",
        negative_elem="side_wall_right",
        name="door_clears_right_side_wall",
    )

    return ctx.report()


object_model = build_object_model()
