from __future__ import annotations

# Tall three-drawer metal filing cabinet (dark charcoal steel).
#
# World layout: the cabinet front faces +X; width is along Y; height along +Z.
# The base plinth rests at z=0. The carcass is a hollow shell (back wall, two
# side walls, top, bottom, and a narrow front face frame) enclosing a cavity in
# which three equal stacked drawers nest. Each drawer is its own part that slides
# out along +X on runners (PRISMATIC). Each drawer face carries a recessed pull
# handle and a brass card-label holder.
#
# Primary articulation: the three sliding drawers (PRISMATIC, +X), retaining
# rear insertion at full extension.

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
    model = ArticulatedObject(name="three_drawer_filing_cabinet")

    charcoal = model.material("charcoal_steel", rgba=(0.15, 0.16, 0.18, 1.0))
    charcoal_dark = model.material("charcoal_dark", rgba=(0.075, 0.080, 0.090, 1.0))
    interior_dark = model.material("interior_dark_steel", rgba=(0.105, 0.110, 0.125, 1.0))
    handle_dark = model.material("handle_dark", rgba=(0.09, 0.09, 0.10, 1.0))
    label_brass = model.material("label_brass", rgba=(0.74, 0.68, 0.45, 1.0))
    badge_red = model.material("badge_red", rgba=(0.78, 0.20, 0.13, 1.0))

    # ---------- key dimensions (meters) ----------
    W = 0.380          # width (Y)
    D = 0.600          # depth (X), front at x=D, back at x=0
    H = 1.300          # total height (Z)
    wall = 0.014       # carcass wall thickness
    plinth_h = 0.060   # recessed base plinth height
    plinth_inset = 0.020

    cavity_front_x = D - wall      # inner face of the front frame plane
    n_drawers = 3

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

    # Bottom panel of the carcass (on the plinth).
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

    # Small maker badge near the top front edge, like the red badge in the
    # reference photo.
    cab.visual(
        Box((0.006, 0.055, 0.022)),
        origin=Origin(xyz=(D + 0.001, -0.105, H - 0.030)),
        material=badge_red,
        name="top_badge",
    )

    cab.inertial = Inertial.from_geometry(Box((D, W, H)), mass=40.0)

    # ---------- drawer opening geometry ----------
    # Stack three equal drawer openings in the body region, separated by thin
    # face rails. Compute the front frame rails (between drawers) on the carcass.
    open_top_z = H - wall          # under the top panel
    open_bot_z = body_bottom_z + wall
    stack_h = open_top_z - open_bot_z
    rail = 0.012                   # gap between drawer faces (face frame rail)
    drawer_face_h = (stack_h - rail * (n_drawers + 1)) / n_drawers
    inner_W = W - 2 * wall

    # Front face rails (horizontal bars between/around drawer faces) so the
    # carcass front reads as a framed face, not an open hole.
    face_x = D - 0.004
    for i in range(n_drawers + 1):
        rz = open_bot_z + rail / 2.0 + i * (drawer_face_h + rail)
        cab.visual(
            Box((0.010, W, rail)),
            origin=Origin(xyz=(face_x, 0.0, rz)),
            material=charcoal,
            name=f"face_rail_{i}",
        )
    # Vertical front frame stiles (left/right edges of the face).
    for s, tag in ((1, "left"), (-1, "right")):
        cab.visual(
            Box((0.010, rail, stack_h)),
            origin=Origin(xyz=(face_x, s * (W / 2.0 - rail / 2.0),
                               open_bot_z + stack_h / 2.0)),
            material=charcoal,
            name=f"face_stile_{tag}",
        )

    # Dark slide runners visible in the side shadows when a drawer is pulled
    # out.  They are fixed to the cabinet side walls and sit just below each
    # drawer side wall.
    runner_len = D - 0.090
    runner_x = wall + runner_len / 2.0
    runner_y = W / 2.0 - wall - 0.004
    runner_h = 0.010

    # ===================================================================
    # DRAWERS (PRISMATIC, slide out along +X)
    # ===================================================================
    drawer_W = inner_W - 0.010
    drawer_depth = D - wall - 0.030      # drawer box depth (X)
    drawer_box_h = drawer_face_h - 0.004
    travel = drawer_depth * 0.72         # usable pull-out travel
    face_thk = 0.016
    sheet = 0.0035                       # 3.5 mm sheet-metal drawer panels

    for i in range(n_drawers):
        tag = f"{i}"
        drawer = model.part(f"drawer_{tag}")
        # vertical center of this drawer face
        cz = open_bot_z + rail + drawer_face_h / 2.0 + i * (drawer_face_h + rail)

        # Fixed cabinet runners at the lower outside edges of this bay.
        runner_z = cz - drawer_box_h / 2.0 + 0.018
        for s, side_tag in ((1, "left"), (-1, "right")):
            cab.visual(
                Box((runner_len, 0.008, runner_h)),
                origin=Origin(xyz=(runner_x, s * runner_y, runner_z)),
                material=charcoal_dark,
                name=f"runner_{tag}_{side_tag}",
            )

        # Hollow open-top sheet-metal drawer box.  The drawer-local frame is at
        # the front face plane (local x=0), and the tray extends rearward along
        # -X.  It is deliberately built from thin bottom, side, and back panels:
        # there is no solid fill and no top panel.
        tray_front_x = -face_thk
        tray_rear_x = tray_front_x - drawer_depth
        tray_mid_x = (tray_front_x + tray_rear_x) / 2.0
        floor_z = -drawer_box_h / 2.0 + sheet / 2.0
        wall_z = -drawer_box_h / 2.0 + drawer_box_h / 2.0
        side_h = drawer_box_h
        # Thin bottom floor panel.
        drawer.visual(
            Box((drawer_depth, drawer_W, sheet)),
            origin=Origin(xyz=(tray_mid_x, 0.0, floor_z)),
            material=interior_dark,
            name="bottom_floor",
        )
        # Left and right side walls of the open tray.
        for s, side_tag in ((1, "left"), (-1, "right")):
            drawer.visual(
                Box((drawer_depth + sheet, sheet, side_h)),
                origin=Origin(xyz=(tray_mid_x, s * (drawer_W / 2.0 - sheet / 2.0), wall_z)),
                material=interior_dark,
                name=f"side_wall_{side_tag}",
            )
        # Back wall closes only the rear end; the top remains open.
        drawer.visual(
            Box((sheet, drawer_W, side_h)),
            origin=Origin(xyz=(tray_rear_x + sheet / 2.0, 0.0, wall_z)),
            material=interior_dark,
            name="back_wall",
        )
        # Drawer front face (slightly proud, full width).
        drawer.visual(
            Box((face_thk, drawer_W + 0.006, drawer_face_h)),
            origin=Origin(xyz=(-face_thk / 2.0, 0.0, 0.0)),
            material=charcoal,
            name="drawer_face",
        )
        # Recessed pull handle: a proud surround bar with a dark inset grip slot.
        drawer.visual(
            Box((0.012, 0.140, 0.030)),
            origin=Origin(xyz=(face_thk * 0.0 + 0.004, 0.0, -drawer_face_h * 0.22)),
            material=charcoal_dark,
            name="handle_surround",
        )
        drawer.visual(
            Box((0.014, 0.120, 0.014)),
            origin=Origin(xyz=(0.006, 0.0, -drawer_face_h * 0.22)),
            material=handle_dark,
            name="pull_handle",
        )
        # Metal card-label holder (a brass plate that stands proud on the face).
        drawer.visual(
            Box((0.010, 0.078, 0.030)),
            origin=Origin(xyz=(0.005, 0.0, drawer_face_h * 0.18)),
            material=label_brass,
            name="label_holder",
        )

        drawer.inertial = Inertial.from_geometry(
            Box((drawer_depth, drawer_W, drawer_box_h)), mass=4.5,
            origin=Origin(xyz=(tray_mid_x, 0.0, 0.0)),
        )

        # Prismatic joint: the closed drawer face sits flush at the cabinet
        # front (x = D). The drawer part frame origin is at the face front, so
        # place the joint origin at (D, 0, cz). Slide along +X to open.
        model.articulation(
            f"cabinet_to_drawer_{tag}",
            ArticulationType.PRISMATIC,
            parent=cab,
            child=drawer,
            origin=Origin(xyz=(D, 0.0, cz)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=60.0, velocity=0.4, lower=0.0, upper=travel),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cab = object_model.get_part("cabinet")

    # --- Cabinet base rests on the floor (z=0), upright and tall ---
    cb = ctx.part_world_aabb(cab)
    assert cb is not None
    ctx.check("base_at_floor", abs(cb[0][2]) < 0.004, details=f"cabinet min z={cb[0][2]:.4f}")
    ctx.check("tall_cabinet", cb[1][2] > 1.2, details=f"cabinet top z={cb[1][2]:.3f}")
    # taller than wide and deeper than wide (upright file cabinet proportions)
    width_y = cb[1][1] - cb[0][1]
    height_z = cb[1][2] - cb[0][2]
    ctx.check("upright", height_z > width_y * 2.5, details=f"h={height_z:.3f}, w={width_y:.3f}")

    drawers = [object_model.get_part(f"drawer_{i}") for i in range(3)]
    joints = [object_model.get_articulation(f"cabinet_to_drawer_{i}") for i in range(3)]

    # --- Exactly 3 drawers exist (not 4) ---
    all_part_names = {p.name for p in object_model.parts}
    drawer_names = {n for n in all_part_names if n.startswith("drawer_")}
    ctx.check("exactly_three_drawers", len(drawer_names) == 3,
              details=f"drawer parts={sorted(drawer_names)}")

    # --- Drawers are stacked in increasing Z and all slide along +X ---
    zs = []
    face_heights = []
    for d, j in zip(drawers, joints):
        assert abs(j.axis[0]) > 0.99, "drawer must slide along X"
        dpos = ctx.part_world_position(d)
        assert dpos is not None
        zs.append(dpos[2])
        face_aabb = ctx.part_element_world_aabb(d, elem="drawer_face")
        assert face_aabb is not None
        face_heights.append(face_aabb[1][2] - face_aabb[0][2])
    ctx.check("drawers_stacked", all(zs[k] < zs[k + 1] for k in range(2)),
              details=f"drawer z order={['%.3f' % z for z in zs]}")

    # --- All 3 drawers are equal height (equal stacked drawers) ---
    if face_heights:
        h_min, h_max = min(face_heights), max(face_heights)
        ctx.check("equal_drawer_heights", h_max - h_min < 0.005,
                  details=f"face heights={['%.4f' % h for h in face_heights]}")

    # --- Each drawer face is flush at the cabinet front when closed ---
    front_x = cb[1][0]
    for i, d in enumerate(drawers):
        face = ctx.part_element_world_aabb(d, elem="drawer_face")
        assert face is not None
        ctx.check(
            f"drawer_{i}_flush_closed",
            abs(face[1][0] - front_x) < 0.020,
            details=f"face front x={face[1][0]:.3f}, cabinet front={front_x:.3f}",
        )

    # --- Every drawer is a hollow open-top sheet-metal tray, not a solid block ---
    required_tray_panels = ("bottom_floor", "side_wall_left", "side_wall_right", "back_wall")
    for i, d in enumerate(drawers):
        visual_names = {v.name for v in d.visuals}
        ctx.check(
            f"drawer_{i}_has_distinct_tray_panels",
            all(name in visual_names for name in required_tray_panels),
            details=f"visuals={sorted(visual_names)}",
        )
        ctx.check(
            f"drawer_{i}_has_no_solid_drawer_box",
            "drawer_box" not in visual_names,
            details=f"visuals={sorted(visual_names)}",
        )

        panel_boxes = []
        panel_volume = 0.0
        for panel_name in required_tray_panels:
            panel = ctx.part_element_world_aabb(d, elem=panel_name)
            assert panel is not None
            panel_boxes.append(panel)
            sx = panel[1][0] - panel[0][0]
            sy = panel[1][1] - panel[0][1]
            sz = panel[1][2] - panel[0][2]
            panel_volume += sx * sy * sz

        tray_min = [min(box[0][axis] for box in panel_boxes) for axis in range(3)]
        tray_max = [max(box[1][axis] for box in panel_boxes) for axis in range(3)]
        tray_volume = (
            (tray_max[0] - tray_min[0])
            * (tray_max[1] - tray_min[1])
            * (tray_max[2] - tray_min[2])
        )
        solid_fraction = panel_volume / tray_volume if tray_volume > 0 else 1.0
        ctx.check(
            f"drawer_{i}_tray_volume_mostly_empty",
            solid_fraction < 0.12,
            details=f"panel_volume={panel_volume:.6f}, envelope={tray_volume:.6f}, fraction={solid_fraction:.3f}",
        )

        floor = ctx.part_element_world_aabb(d, elem="bottom_floor")
        side = ctx.part_element_world_aabb(d, elem="side_wall_left")
        back = ctx.part_element_world_aabb(d, elem="back_wall")
        assert floor is not None and side is not None and back is not None
        ctx.check(
            f"drawer_{i}_open_top_cavity",
            side[1][2] > floor[1][2] + 0.18 and back[1][2] > floor[1][2] + 0.18,
            details=f"floor_top={floor[1][2]:.3f}, side_top={side[1][2]:.3f}, back_top={back[1][2]:.3f}",
        )

    # --- Each drawer slides out (+X) and retains rear insertion ---
    for i, (d, j) in enumerate(zip(drawers, joints)):
        rest = ctx.part_world_position(d)
        with ctx.pose({j: j.motion_limits.upper}):
            out = ctx.part_world_position(d)
            # rear tray wall should still be inside the cabinet front
            back = ctx.part_element_world_aabb(d, elem="back_wall")
            assert back is not None
            rear_x = back[0][0]
        assert rest is not None and out is not None
        ctx.check(f"drawer_{i}_opens_forward", out[0] > rest[0] + 0.10,
                  details=f"closed x={rest[0]:.3f}, open x={out[0]:.3f}")
        ctx.check(f"drawer_{i}_retains_insertion", rear_x < front_x - 0.05,
                  details=f"open rear_x={rear_x:.3f}, front={front_x:.3f}")

    # --- A representative drawer fits within the cabinet footprint when closed ---
    ctx.expect_within(drawers[1], cab, axes="y", margin=0.004, name="drawer_within_width")

    return ctx.report()


object_model = build_object_model()
