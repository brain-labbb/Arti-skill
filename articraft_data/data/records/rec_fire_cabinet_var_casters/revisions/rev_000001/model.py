from __future__ import annotations

# Tall four-drawer metal filing cabinet (dark charcoal steel) on caster wheels.
#
# World layout: the cabinet front faces +X; width is along Y; height along +Z.
# Four swiveling caster wheels at the bottom corners carry the carcass; the
# wheel bottoms rest at z=0. The carcass is a hollow shell (back wall, two
# side walls, top, bottom, and a narrow front face frame) enclosing a cavity
# in which four stacked drawers nest. Each drawer is its own part that slides
# out along +X on runners (PRISMATIC). Each drawer face carries a recessed
# pull handle and a brass card-label holder.
#
# Primary articulations:
#   - four sliding drawers (PRISMATIC, +X), retaining rear insertion at full extension
#   - four swiveling casters (CONTINUOUS, Z axis) at the cabinet bottom corners

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


def _add_caster_visuals(caster_part, caster_steel_mat, wheel_mat, caster_h, wheel_r, wheel_w):
    """Build the visible geometry of one swivel caster.

    The caster part frame sits at the swivel axis on the cabinet bottom surface.
    All visuals extend downward (negative local Z) from there.
    """
    # --- mounting plate (bolted to cabinet bottom) ---
    plate_sz = (0.050, 0.050, 0.005)
    caster_part.visual(
        Box(plate_sz),
        origin=Origin(xyz=(0.0, 0.0, -plate_sz[2] / 2.0)),
        material=caster_steel_mat,
        name="mounting_plate",
    )

    # --- swivel raceway ring ---
    ring_z_top = -plate_sz[2]
    ring_h = 0.008
    caster_part.visual(
        Cylinder(radius=0.016, length=ring_h),
        origin=Origin(xyz=(0.0, 0.0, ring_z_top - ring_h / 2.0)),
        material=caster_steel_mat,
        name="swivel_ring",
    )

    # --- fork stem (connects raceway to fork bridge) ---
    stem_z_top = ring_z_top - ring_h
    stem_h = 0.014
    caster_part.visual(
        Cylinder(radius=0.009, length=stem_h),
        origin=Origin(xyz=(0.0, 0.0, stem_z_top - stem_h / 2.0)),
        material=caster_steel_mat,
        name="fork_stem",
    )

    # --- fork bridge (connects the two fork legs) ---
    bridge_z_top = stem_z_top - stem_h
    bridge_sz = (0.012, 0.044, 0.006)
    caster_part.visual(
        Box(bridge_sz),
        origin=Origin(xyz=(0.0, 0.0, bridge_z_top - bridge_sz[2] / 2.0)),
        material=caster_steel_mat,
        name="fork_bridge",
    )

    # --- fork legs (two stamped steel cheeks straddling the wheel) ---
    leg_z_top = bridge_z_top - bridge_sz[2]
    leg_bottom = -(caster_h - wheel_r) + 0.003  # extend slightly past the axle
    leg_h = leg_z_top - leg_bottom
    leg_z_center = (leg_z_top + leg_bottom) / 2.0
    leg_y_offset = wheel_w / 2.0 + 0.005  # gap between wheel face and leg
    leg_thickness = 0.005

    for idx, s in enumerate((1, -1)):
        caster_part.visual(
            Box((0.010, leg_thickness, leg_h)),
            origin=Origin(xyz=(0.0, s * leg_y_offset, leg_z_center)),
            material=caster_steel_mat,
            name=f"fork_leg_{idx}",
        )

    # --- wheel (cylinder rotated to lie along Y axis like a rolling caster) ---
    axle_z = -(caster_h - wheel_r)
    caster_part.visual(
        Cylinder(radius=wheel_r, length=wheel_w),
        origin=Origin(
            xyz=(0.0, 0.0, axle_z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=wheel_mat,
        name="wheel",
    )

    # --- hub caps (small disks on each side of the wheel center) ---
    hub_r = 0.008
    hub_h = 0.004
    for idx, s in enumerate((1, -1)):
        caster_part.visual(
            Cylinder(radius=hub_r, length=hub_h),
            origin=Origin(
                xyz=(0.0, s * (wheel_w / 2.0 + hub_h / 2.0), axle_z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=caster_steel_mat,
            name=f"hub_cap_{idx}",
        )


# ---------- key dimensions (meters) ----------
W = 0.380          # width (Y)
D = 0.600          # depth (X), front at x=D, back at x=0
H = 1.320          # carcass top height (Z)
WALL = 0.014       # carcass wall thickness
CASTER_H = 0.080   # floor (z=0) to cabinet bottom surface
WHEEL_R = 0.025    # caster wheel radius
WHEEL_W = 0.020    # caster wheel width


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_drawer_filing_cabinet")

    charcoal = model.material("charcoal_steel", rgba=(0.15, 0.16, 0.18, 1.0))
    charcoal_dark = model.material("charcoal_dark", rgba=(0.075, 0.080, 0.090, 1.0))
    interior_dark = model.material("interior_dark_steel", rgba=(0.105, 0.110, 0.125, 1.0))
    handle_dark = model.material("handle_dark", rgba=(0.09, 0.09, 0.10, 1.0))
    label_brass = model.material("label_brass", rgba=(0.74, 0.68, 0.45, 1.0))
    badge_red = model.material("badge_red", rgba=(0.78, 0.20, 0.13, 1.0))
    caster_steel = model.material("caster_steel", rgba=(0.25, 0.26, 0.28, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.06, 0.06, 0.07, 1.0))

    wall = WALL
    caster_h = CASTER_H
    wheel_r = WHEEL_R
    wheel_w = WHEEL_W

    cavity_front_x = D - wall      # inner face of the front frame plane
    n_drawers = 4

    # ===================================================================
    # ROOT: cabinet carcass (shell)
    # ===================================================================
    cab = model.part("cabinet")

    body_bottom_z = caster_h
    body_h = H - caster_h

    # Bottom panel of the carcass (sits on the casters).
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

    # Small maker badge near the top front edge.
    cab.visual(
        Box((0.006, 0.055, 0.022)),
        origin=Origin(xyz=(D + 0.001, -0.105, H - 0.030)),
        material=badge_red,
        name="top_badge",
    )

    cab.inertial = Inertial.from_geometry(Box((D, W, H)), mass=40.0)

    # ---------- drawer opening geometry ----------
    open_top_z = H - wall          # under the top panel
    open_bot_z = body_bottom_z + wall
    stack_h = open_top_z - open_bot_z
    rail = 0.012                   # gap between drawer faces (face frame rail)
    drawer_face_h = (stack_h - rail * (n_drawers + 1)) / n_drawers
    inner_W = W - 2 * wall

    # Front face rails (horizontal bars between/around drawer faces).
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
    # out.  They are fixed to the cabinet side walls.
    runner_len = D - 0.090
    runner_x = wall + runner_len / 2.0
    runner_y = W / 2.0 - wall - 0.004
    runner_h = 0.010

    # ===================================================================
    # CASTERS (CONTINUOUS, swivel around Z at each bottom corner)
    # ===================================================================
    corner_inset = 0.035
    corner_positions = [
        (D - corner_inset,  W / 2.0 - corner_inset),   # front-left
        (D - corner_inset, -(W / 2.0 - corner_inset)),  # front-right
        (corner_inset,      W / 2.0 - corner_inset),    # rear-left
        (corner_inset,     -(W / 2.0 - corner_inset)),  # rear-right
    ]

    for i in range(4):
        cx, cy = corner_positions[i]
        caster = model.part(f"caster_{i}")

        _add_caster_visuals(
            caster, caster_steel, wheel_rubber,
            caster_h, wheel_r, wheel_w,
        )

        caster.inertial = Inertial.from_geometry(
            Cylinder(radius=0.030, length=caster_h), mass=0.8,
            origin=Origin(xyz=(0.0, 0.0, -caster_h / 2.0)),
        )

        # Swivel joint: CONTINUOUS around Z. Origin at the cabinet bottom
        # surface where the mounting plate contacts the carcass floor.
        model.articulation(
            f"cabinet_to_caster_{i}",
            ArticulationType.CONTINUOUS,
            parent=cab,
            child=caster,
            origin=Origin(xyz=(cx, cy, body_bottom_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=4.0),
        )

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

        # Hollow open-top sheet-metal drawer box.
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
        # Metal card-label holder (brass plate proud on the face).
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
        # front (x = D). Slide along +X to open.
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

    # --- Cabinet is elevated above floor by the casters (no plinth) ---
    cab_aabb = ctx.part_world_aabb(cab)
    assert cab_aabb is not None
    ctx.check(
        "cabinet_elevated_above_floor",
        cab_aabb[0][2] > 0.040,
        details=f"cabinet min z={cab_aabb[0][2]:.4f}",
    )
    ctx.check(
        "no_base_plinth_visual",
        "base_plinth" not in {v.name for v in cab.visuals},
        details=f"visuals={sorted(v.name for v in cab.visuals)}",
    )

    # --- Cabinet is tall and upright ---
    ctx.check("tall_cabinet", cab_aabb[1][2] > 1.2, details=f"top z={cab_aabb[1][2]:.3f}")
    width_y = cab_aabb[1][1] - cab_aabb[0][1]
    height_z = cab_aabb[1][2] - cab_aabb[0][2]
    ctx.check("upright", height_z > width_y * 2.5, details=f"h={height_z:.3f}, w={width_y:.3f}")

    # --- Four casters exist and their wheels reach the floor ---
    casters = [object_model.get_part(f"caster_{i}") for i in range(4)]
    caster_joints = [object_model.get_articulation(f"cabinet_to_caster_{i}") for i in range(4)]

    for i, (c, j) in enumerate(zip(casters, caster_joints)):
        c_aabb = ctx.part_world_aabb(c)
        assert c_aabb is not None
        ctx.check(
            f"caster_{i}_wheel_near_floor",
            c_aabb[0][2] < 0.005,
            details=f"caster_{i} min z={c_aabb[0][2]:.4f}",
        )
        # Caster has a wheel visual
        visual_names = {v.name for v in c.visuals}
        ctx.check(
            f"caster_{i}_has_wheel",
            "wheel" in visual_names,
            details=f"visuals={sorted(visual_names)}",
        )
        # Caster has fork structure
        ctx.check(
            f"caster_{i}_has_fork",
            "fork_bridge" in visual_names and "fork_leg_0" in visual_names,
            details=f"visuals={sorted(visual_names)}",
        )
        # Joint is continuous (free swivel) around Z
        ctx.check(
            f"caster_{i}_swivel_axis_z",
            abs(j.axis[2]) > 0.99,
            details=f"axis={j.axis}",
        )

    # --- Casters are positioned at the four corners of the cabinet ---
    caster_xs = []
    caster_ys = []
    for c in casters:
        pos = ctx.part_world_position(c)
        assert pos is not None
        caster_xs.append(pos[0])
        caster_ys.append(pos[1])
    ctx.check(
        "casters_span_depth",
        max(caster_xs) - min(caster_xs) > 0.4 * D,
        details=f"caster x range=[{min(caster_xs):.3f}, {max(caster_xs):.3f}]",
    )
    ctx.check(
        "casters_span_width",
        max(caster_ys) - min(caster_ys) > 0.4 * W,
        details=f"caster y range=[{min(caster_ys):.3f}, {max(caster_ys):.3f}]",
    )

    # --- Drawers are stacked and slide correctly ---
    drawers = [object_model.get_part(f"drawer_{i}") for i in range(4)]
    drawer_joints = [object_model.get_articulation(f"cabinet_to_drawer_{i}") for i in range(4)]

    zs = []
    for d, j in zip(drawers, drawer_joints):
        assert abs(j.axis[0]) > 0.99, "drawer must slide along X"
        dpos = ctx.part_world_position(d)
        assert dpos is not None
        zs.append(dpos[2])
    ctx.check("drawers_stacked", all(zs[k] < zs[k + 1] for k in range(3)),
              details=f"drawer z order={['%.3f' % z for z in zs]}")

    # --- Each drawer face is flush at the cabinet front when closed ---
    front_x = cab_aabb[1][0]
    for i, d in enumerate(drawers):
        face = ctx.part_element_world_aabb(d, elem="drawer_face")
        assert face is not None
        ctx.check(
            f"drawer_{i}_flush_closed",
            abs(face[1][0] - front_x) < 0.020,
            details=f"face front x={face[1][0]:.3f}, cabinet front={front_x:.3f}",
        )

    # --- Every drawer is a hollow open-top sheet-metal tray ---
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
    for i, (d, j) in enumerate(zip(drawers, drawer_joints)):
        rest = ctx.part_world_position(d)
        with ctx.pose({j: j.motion_limits.upper}):
            out = ctx.part_world_position(d)
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
