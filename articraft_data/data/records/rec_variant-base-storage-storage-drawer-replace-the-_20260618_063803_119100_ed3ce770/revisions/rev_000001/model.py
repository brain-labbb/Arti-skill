from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Overall envelope (meters). Front of the cooker faces +X.
#   width  (Y): 0.50
#   depth  (X): 0.60
#   height (Z): ~0.85 including feet, burners, and pan grates
# ---------------------------------------------------------------------------
HALF_W = 0.25   # half width  (Y)
HALF_D = 0.30   # half depth  (X)

FOOT_H = 0.05
DRAWER_BOTTOM = FOOT_H          # 0.05 — top of feet
DRAWER_H = 0.140                # drawer front panel height
DRAWER_CZ = DRAWER_BOTTOM + DRAWER_H / 2.0  # 0.120

OVEN_FLOOR = DRAWER_BOTTOM + DRAWER_H + 0.010  # 0.200 — raised to make drawer space
DOOR_TOP = 0.62
COOKTOP_FLOOR = 0.785
RIM_TOP = 0.805
GRATE_Z0 = RIM_TOP
GRATE_BAR = 0.008

DOOR_H = DOOR_TOP - OVEN_FLOOR  # 0.42
DOOR_T = 0.04

KNOB_Z = 0.71
KNOB_YS = [-0.1875 + 0.075 * i for i in range(6)]

# (x, y) burner centers in the cooktop well; index order: 0=front-right view
BURNER_XY = [(0.13, 0.115), (0.13, -0.115), (-0.13, 0.115), (-0.13, -0.115)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_gas_range_cooker_storage_drawer")

    enamel_white = model.material("enamel_white", rgba=(0.93, 0.93, 0.91, 1.0))
    knob_white = model.material("knob_white", rgba=(0.96, 0.96, 0.95, 1.0))
    iron_black = model.material("iron_black", rgba=(0.07, 0.07, 0.08, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.05, 0.06, 0.07, 0.93))
    steel_silver = model.material("steel_silver", rgba=(0.76, 0.77, 0.78, 1.0))
    cap_silver = model.material("cap_silver", rgba=(0.82, 0.83, 0.84, 1.0))
    drip_silver = model.material("drip_silver", rgba=(0.60, 0.61, 0.62, 1.0))
    cavity_grey = model.material("cavity_grey", rgba=(0.20, 0.19, 0.18, 1.0))
    rack_chrome = model.material("rack_chrome", rgba=(0.68, 0.69, 0.70, 1.0))
    foot_black = model.material("foot_black", rgba=(0.12, 0.12, 0.12, 1.0))
    drawer_liner = model.material("drawer_liner", rgba=(0.88, 0.88, 0.87, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("body")

    # Four short adjustable feet (embedded 1 mm into the underside).
    for i, (fx, fy) in enumerate([(0.26, 0.21), (0.26, -0.21), (-0.26, 0.21), (-0.26, -0.21)]):
        body.visual(
            Cylinder(radius=0.022, length=0.051),
            origin=Origin(xyz=(fx, fy, 0.0255)),
            material=foot_black,
            name=f"foot_{i}",
        )

    # Bottom plate connecting feet to lower panels.
    body.visual(
        Box((0.58, 0.49, 0.005)),
        origin=Origin(xyz=(-0.01, 0.0, 0.048)),
        material=enamel_white,
        name="bottom_plate",
    )

    # --- lower outer panels (drawer compartment enclosure) ---
    # Thin sheet-metal panels from near the feet up to the oven floor.
    lower_h = OVEN_FLOOR - 0.048  # 0.152
    lower_zc = 0.048 + lower_h / 2.0  # 0.124
    body.visual(
        Box((0.60, 0.015, lower_h)),
        origin=Origin(xyz=(0.0, -0.2425, lower_zc)),
        material=enamel_white,
        name="lower_panel_left",
    )
    body.visual(
        Box((0.60, 0.015, lower_h)),
        origin=Origin(xyz=(0.0, 0.2425, lower_zc)),
        material=enamel_white,
        name="lower_panel_right",
    )
    body.visual(
        Box((0.015, 0.47, lower_h)),
        origin=Origin(xyz=(-0.2925, 0.0, lower_zc)),
        material=enamel_white,
        name="lower_panel_rear",
    )

    # --- oven floor slab (ceiling of drawer zone, floor of oven cavity) ---
    body.visual(
        Box((0.56, 0.47, 0.012)),
        origin=Origin(xyz=(-0.01, 0.0, OVEN_FLOOR - 0.006)),
        material=enamel_white,
        name="oven_floor_slab",
    )

    # --- upper cavity walls (oven zone, thick insulated panels) ---
    upper_h = DOOR_TOP - (OVEN_FLOOR - 0.005)  # 0.425
    upper_zc = (OVEN_FLOOR - 0.005) + upper_h / 2.0  # 0.4075
    body.visual(
        Box((0.60, 0.05, upper_h)),
        origin=Origin(xyz=(0.0, -0.225, upper_zc)),
        material=enamel_white,
        name="wall_left",
    )
    body.visual(
        Box((0.60, 0.05, upper_h)),
        origin=Origin(xyz=(0.0, 0.225, upper_zc)),
        material=enamel_white,
        name="wall_right",
    )
    body.visual(
        Box((0.05, 0.40, upper_h)),
        origin=Origin(xyz=(-0.275, 0.0, upper_zc)),
        material=enamel_white,
        name="wall_rear",
    )

    # Upper housing: control fascia front band + burner box under the cooktop.
    body.visual(
        Box((0.60, 0.50, 0.166)),
        origin=Origin(xyz=(0.0, 0.0, 0.702)),
        material=enamel_white,
        name="top_housing",
    )

    # Raised rim around the recessed cooktop well (z 0.784 .. 0.805).
    body.visual(Box((0.03, 0.50, 0.021)), origin=Origin(xyz=(0.285, 0.0, 0.7945)), material=enamel_white, name="rim_front")
    body.visual(Box((0.03, 0.50, 0.021)), origin=Origin(xyz=(-0.285, 0.0, 0.7945)), material=enamel_white, name="rim_rear")
    body.visual(Box((0.54, 0.03, 0.021)), origin=Origin(xyz=(0.0, -0.235, 0.7945)), material=enamel_white, name="rim_left")
    body.visual(Box((0.54, 0.03, 0.021)), origin=Origin(xyz=(0.0, 0.235, 0.7945)), material=enamel_white, name="rim_right")

    # Dark oven cavity liners (thin panels inside the upper walls).
    liner_h = DOOR_TOP - OVEN_FLOOR - 0.004  # 0.416
    liner_zc = OVEN_FLOOR + 0.002 + liner_h / 2.0  # ~0.41
    body.visual(Box((0.55, 0.005, liner_h)), origin=Origin(xyz=(0.025, -0.1985, liner_zc)), material=cavity_grey, name="liner_left")
    body.visual(Box((0.55, 0.005, liner_h)), origin=Origin(xyz=(0.025, 0.1985, liner_zc)), material=cavity_grey, name="liner_right")
    body.visual(Box((0.005, 0.40, liner_h)), origin=Origin(xyz=(-0.2485, 0.0, liner_zc)), material=cavity_grey, name="liner_rear")
    body.visual(Box((0.55, 0.40, 0.005)), origin=Origin(xyz=(0.025, 0.0, DOOR_TOP - 0.002)), material=cavity_grey, name="liner_top")
    body.visual(Box((0.55, 0.40, 0.005)), origin=Origin(xyz=(0.025, 0.0, OVEN_FLOOR + 0.002)), material=cavity_grey, name="liner_bottom")

    # Chrome oven rack shelf spanning between the side liners.
    body.visual(
        Box((0.48, 0.398, 0.008)),
        origin=Origin(xyz=(0.02, 0.0, OVEN_FLOOR + 0.10)),
        material=rack_chrome,
        name="oven_rack",
    )

    # Drawer slide rails (two steel channels inside the lower compartment,
    # extending from the bottom plate up to the slide height).
    rail_top = DRAWER_BOTTOM + 0.045
    rail_bot = 0.050
    rail_zc = (rail_top + rail_bot) / 2.0
    rail_h = rail_top - rail_bot
    for side_name, sy in [("left", -0.195), ("right", 0.195)]:
        body.visual(
            Box((0.48, 0.012, rail_h)),
            origin=Origin(xyz=(-0.04, sy, rail_zc)),
            material=steel_silver,
            name=f"rail_{side_name}",
        )

    # Four gas burners: recessed drip cup, burner body, silver cap.
    for i, (bx, by) in enumerate(BURNER_XY):
        body.visual(
            Cylinder(radius=0.055, length=0.006),
            origin=Origin(xyz=(bx, by, 0.7875)),
            material=drip_silver,
            name=f"drip_cup_{i}",
        )
        body.visual(
            Cylinder(radius=0.030, length=0.010),
            origin=Origin(xyz=(bx, by, 0.795)),
            material=steel_silver,
            name=f"burner_body_{i}",
        )
        body.visual(
            Cylinder(radius=0.022, length=0.004),
            origin=Origin(xyz=(bx, by, 0.8015)),
            material=cap_silver,
            name=f"burner_cap_{i}",
        )

    # ------------------------------------------------------- cast-iron grates
    bar_zc = GRATE_Z0 + GRATE_BAR / 2.0
    for gi, side in enumerate((-1.0, 1.0)):
        grate = model.part(f"grate_{gi}")
        grate.visual(Box((GRATE_BAR, 0.230, GRATE_BAR)), origin=Origin(xyz=(0.281, side * 0.125, bar_zc)), material=iron_black, name="frame_bar_front")
        grate.visual(Box((GRATE_BAR, 0.230, GRATE_BAR)), origin=Origin(xyz=(-0.281, side * 0.125, bar_zc)), material=iron_black, name="frame_bar_rear")
        grate.visual(Box((0.570, GRATE_BAR, GRATE_BAR)), origin=Origin(xyz=(0.0, side * 0.236, bar_zc)), material=iron_black, name="frame_bar_outer")
        grate.visual(Box((0.570, GRATE_BAR, GRATE_BAR)), origin=Origin(xyz=(0.0, side * 0.014, bar_zc)), material=iron_black, name="frame_bar_inner")
        for fi, fy in enumerate((0.0695, 0.125, 0.1805)):
            grate.visual(Box((0.570, GRATE_BAR, GRATE_BAR)), origin=Origin(xyz=(0.0, side * fy, bar_zc)), material=iron_black, name=f"finger_{fi}")
        model.articulation(
            f"grate_{gi}_mount",
            ArticulationType.FIXED,
            parent=body,
            child=grate,
            origin=Origin(),
        )

    # --------------------------------------------------------------- oven door
    door = model.part("oven_door")
    door_frame = BezelGeometry(
        (0.36, 0.28),
        (0.50, DOOR_H),
        DOOR_T,
        opening_shape="rounded_rect",
        outer_shape="rect",
        opening_corner_radius=0.012,
    )
    door.visual(
        mesh_from_geometry(door_frame, "oven_door_frame"),
        origin=Origin(xyz=(DOOR_T / 2.0, 0.0, DOOR_H / 2.0), rpy=(math.pi / 2, 0.0, math.pi / 2)),
        material=enamel_white,
        name="door_frame",
    )
    door.visual(
        Box((0.012, 0.38, 0.26)),
        origin=Origin(xyz=(0.018, 0.0, DOOR_H / 2.0)),
        material=glass_tint,
        name="window_glass",
    )
    handle_z = DOOR_H - 0.05
    for si, hy in enumerate((-0.18, 0.18)):
        door.visual(
            Cylinder(radius=0.009, length=0.030),
            origin=Origin(xyz=(0.053, hy, handle_z), rpy=(0.0, math.pi / 2, 0.0)),
            material=steel_silver,
            name=f"handle_standoff_{si}",
        )
    door.visual(
        Cylinder(radius=0.011, length=0.44),
        origin=Origin(xyz=(0.072, 0.0, handle_z), rpy=(math.pi / 2, 0.0, 0.0)),
        material=steel_silver,
        name="handle_bar",
    )
    model.articulation(
        "oven_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(HALF_D, 0.0, OVEN_FLOOR)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.5, lower=0.0, upper=math.pi / 2),
    )

    # -------------------------------------------------------- storage drawer
    drawer = model.part("storage_drawer")

    # Front panel — visible white enamel face, flush with the body front.
    drawer.visual(
        Box((0.020, 0.45, DRAWER_H)),
        origin=Origin(xyz=(-0.010, 0.0, 0.0)),
        material=enamel_white,
        name="drawer_front",
    )

    # Internal drawer box (open-top tray behind the front panel).
    # Width is kept inside the slide rails (rails at y=±0.195 ± 0.006).
    box_depth = 0.44
    box_width = 0.36
    box_height = 0.10
    box_cx = -0.020 - box_depth / 2.0   # -0.240
    box_cz = -DRAWER_H / 2.0 + 0.005 + box_height / 2.0  # bottom-aligned

    # Bottom panel
    drawer.visual(
        Box((box_depth, box_width, 0.005)),
        origin=Origin(xyz=(box_cx, 0.0, -DRAWER_H / 2.0 + 0.005 + 0.0025)),
        material=drawer_liner,
        name="drawer_bottom",
    )
    # Side panels
    for ds_name, dsy in [("left", -box_width / 2.0), ("right", box_width / 2.0)]:
        drawer.visual(
            Box((box_depth, 0.005, box_height)),
            origin=Origin(xyz=(box_cx, dsy, box_cz)),
            material=drawer_liner,
            name=f"drawer_side_{ds_name}",
        )
    # Back panel
    drawer.visual(
        Box((0.005, box_width, box_height)),
        origin=Origin(xyz=(box_cx - box_depth / 2.0 + 0.0025, 0.0, box_cz)),
        material=drawer_liner,
        name="drawer_back",
    )

    # Drawer handle: slim bar near the top of the front panel on two standoffs.
    dh_z = DRAWER_H / 2.0 - 0.025  # near top of drawer front
    for si, hy in enumerate((-0.10, 0.10)):
        drawer.visual(
            Cylinder(radius=0.007, length=0.020),
            origin=Origin(xyz=(0.010, hy, dh_z), rpy=(0.0, math.pi / 2, 0.0)),
            material=steel_silver,
            name=f"drawer_handle_standoff_{si}",
        )
    drawer.visual(
        Cylinder(radius=0.008, length=0.24),
        origin=Origin(xyz=(0.022, 0.0, dh_z), rpy=(math.pi / 2, 0.0, 0.0)),
        material=steel_silver,
        name="drawer_handle_bar",
    )

    # Drawer PRISMATIC slide — pulls out along +X (toward the user).
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(HALF_D, 0.0, DRAWER_CZ)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.5, lower=0.0, upper=0.30),
    )

    # ------------------------------------------------------------- control knobs
    for i, ky in enumerate(KNOB_YS):
        knob = model.part(f"knob_{i}")
        knob_geo = KnobGeometry(
            0.038,
            0.024,
            body_style="skirted",
            top_diameter=0.030,
            skirt=KnobSkirt(0.048, 0.006, flare=0.08, chamfer=0.001),
            grip=KnobGrip(style="fluted", count=16, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
            center=False,
        )
        knob.visual(
            mesh_from_geometry(knob_geo, f"range_knob_{i}"),
            origin=Origin(rpy=(0.0, math.pi / 2, 0.0)),
            material=knob_white,
            name="knob_cap",
        )
        model.articulation(
            f"knob_{i}_dial",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(HALF_D, ky, KNOB_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0, velocity=6.0, lower=0.0, upper=math.radians(270.0)
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("oven_door")
    hinge = object_model.get_articulation("oven_door_hinge")
    drawer = object_model.get_part("storage_drawer")
    slide = object_model.get_articulation("drawer_slide")
    knobs = [object_model.get_part(f"knob_{i}") for i in range(6)]
    grates = [object_model.get_part(f"grate_{i}") for i in range(2)]

    # --- storage drawer mechanism ------------------------------------------

    # Drawer front outer face is flush with the body front (x ≈ HALF_D).
    front_aabb = ctx.part_element_world_aabb(drawer, elem="drawer_front")
    ok_flush = front_aabb is not None
    if front_aabb is not None:
        fmin, fmax = front_aabb
        ok_flush = abs(fmax[0] - HALF_D) < 0.003
    ctx.check(
        "closed drawer front flush with body front",
        ok_flush,
        f"front_aabb={front_aabb!r}",
    )

    # Drawer zone is below the oven floor.
    drawer_aabb = ctx.part_world_aabb(drawer)
    ok_drawer_z = drawer_aabb is not None
    if drawer_aabb is not None:
        dmin, dmax = drawer_aabb
        ok_drawer_z = dmin[2] >= 0.03 and dmax[2] <= OVEN_FLOOR + 0.005
    ctx.check("drawer zone is below oven floor", ok_drawer_z, f"drawer_aabb={drawer_aabb!r}")

    # Oven floor slab separates the drawer compartment from the oven cavity.
    slab_aabb = ctx.part_element_world_aabb(body, elem="oven_floor_slab")
    ok_slab = slab_aabb is not None
    if slab_aabb is not None:
        smin, smax = slab_aabb
        slab_z = 0.5 * (smin[2] + smax[2])
        ok_slab = (
            abs(slab_z - OVEN_FLOOR) < 0.012
            and (smax[1] - smin[1]) > 0.40
            and (smax[0] - smin[0]) > 0.50
        )
    ctx.check("oven floor slab separates drawer from oven cavity", ok_slab, f"slab_aabb={slab_aabb!r}")

    # Drawer has a handle bar protruding from the front face.
    handle_aabb = ctx.part_element_world_aabb(drawer, elem="drawer_handle_bar")
    ok_dh = handle_aabb is not None
    if handle_aabb is not None:
        hmin, hmax = handle_aabb
        ok_dh = hmax[0] > HALF_D and (hmax[1] - hmin[1]) > 0.15
    ctx.check("drawer has visible handle bar", ok_dh, f"handle_aabb={handle_aabb!r}")

    # Drawer slide rails are present on the body.
    for side in ("left", "right"):
        rail_aabb = ctx.part_element_world_aabb(body, elem=f"rail_{side}")
        ok_rail = rail_aabb is not None
        if rail_aabb is not None:
            rmin, rmax = rail_aabb
            ok_rail = rmin[2] > 0.03 and rmax[2] < OVEN_FLOOR and (rmax[0] - rmin[0]) > 0.30
        ctx.check(f"drawer slide rail ({side})", ok_rail, f"rail_aabb={rail_aabb!r}")

    # Prismatic slide: drawer extends forward (+X) when opened.
    closed_pos = ctx.part_world_position(drawer)
    slide_upper = slide.motion_limits.upper
    with ctx.pose({slide: slide_upper}):
        open_pos = ctx.part_world_position(drawer)
    ok_slide = closed_pos is not None and open_pos is not None
    if closed_pos is not None and open_pos is not None:
        ok_slide = open_pos[0] > closed_pos[0] + 0.20
    ctx.check(
        "drawer slides out forward on prismatic joint",
        ok_slide,
        f"closed={closed_pos!r} open={open_pos!r}",
    )

    # Drawer slide limits are sensible.
    slim = slide.motion_limits
    ctx.check(
        "drawer slide limits 0..~0.30m",
        slim is not None and abs(slim.lower) < 1e-6 and 0.20 <= slim.upper <= 0.40,
        f"limits={slim!r}",
    )

    # --- oven door: closed pose seats flush on the body front -------------
    ctx.expect_gap(
        door, body,
        axis="x",
        max_gap=0.002,
        max_penetration=0.0005,
        name="closed door seats against body front",
    )
    ctx.expect_within(door, body, axes="y", margin=0.002, name="door spans within body width")

    # Dark tinted glass window (adjusted for shorter door).
    glass_aabb = ctx.part_element_world_aabb(door, elem="window_glass")
    ok_glass = glass_aabb is not None
    if glass_aabb is not None:
        gmin, gmax = glass_aabb
        ok_glass = (
            (gmax[1] - gmin[1]) >= 0.36
            and (gmax[2] - gmin[2]) >= 0.24
            and OVEN_FLOOR < 0.5 * (gmin[2] + gmax[2]) < DOOR_TOP
        )
    ctx.check("door has large tinted glass window", ok_glass, f"glass_aabb={glass_aabb!r}")

    # Slim handle bar near the door top edge.
    door_handle_aabb = ctx.part_element_world_aabb(door, elem="handle_bar")
    ok_dh_door = door_handle_aabb is not None
    if door_handle_aabb is not None:
        hmin, hmax = door_handle_aabb
        ok_dh_door = hmin[2] > 0.40 and hmax[0] > 0.35 and (hmax[1] - hmin[1]) > 0.40
    ctx.check("handle bar near door top edge", ok_dh_door, f"handle_aabb={door_handle_aabb!r}")

    # Drop-down motion: at 90 deg the door lies horizontal in front.
    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({hinge: math.pi / 2}):
        open_aabb = ctx.part_world_aabb(door)
    ok_open = closed_aabb is not None and open_aabb is not None
    if closed_aabb is not None and open_aabb is not None:
        ok_open = (
            open_aabb[1][0] > 0.60
            and open_aabb[1][2] < 0.28
            and open_aabb[0][2] > 0.05
            and open_aabb[1][2] < closed_aabb[1][2] - 0.25
        )
    ctx.check(
        "door drops down 90 deg in front of the oven",
        ok_open,
        f"closed={closed_aabb!r} open={open_aabb!r}",
    )
    lim = hinge.motion_limits
    ctx.check(
        "door hinge limits 0..~90deg",
        lim is not None and abs(lim.lower) < 1e-6 and abs(lim.upper - math.pi / 2) < 0.1,
        f"limits={lim!r}",
    )

    # --- control knobs: six dials seated on the fascia above the door -----
    knob_ys = []
    for i, knob in enumerate(knobs):
        ctx.expect_contact(knob, body, contact_tol=5e-4, name=f"knob_{i} seated on fascia")
        pos = ctx.part_world_position(knob)
        ok_pos = pos is not None and pos[2] > DOOR_TOP + 0.04 and pos[0] > 0.29
        ctx.check(f"knob_{i} on fascia above oven door", ok_pos, f"pos={pos!r}")
        if pos is not None:
            knob_ys.append(pos[1])
        klim = object_model.get_articulation(f"knob_{i}_dial").motion_limits
        ctx.check(
            f"knob_{i} dial range ~270deg about front axis",
            klim is not None and abs(klim.lower) < 1e-6 and abs(klim.upper - math.radians(270.0)) < 0.05,
            f"limits={klim!r}",
        )
    ctx.check(
        "six knobs form a horizontal row",
        len(knob_ys) == 6
        and all(knob_ys[i] < knob_ys[i + 1] - 0.05 for i in range(5))
        and (knob_ys[-1] - knob_ys[0]) > 0.30,
        f"knob_ys={knob_ys!r}",
    )

    # --- cooktop: four silver burner caps in a 2x2 layout -----------------
    quadrants = set()
    caps_ok = True
    for i in range(4):
        cap_aabb = ctx.part_element_world_aabb(body, elem=f"burner_cap_{i}")
        if cap_aabb is None:
            caps_ok = False
            continue
        cmin, cmax = cap_aabb
        cx = 0.5 * (cmin[0] + cmax[0])
        cy = 0.5 * (cmin[1] + cmax[1])
        if not (0.79 <= cmax[2] <= 0.81 and abs(cx) > 0.05 and abs(cy) > 0.05):
            caps_ok = False
        quadrants.add((cx > 0.0, cy > 0.0))
    ctx.check(
        "four burner caps in a 2x2 layout on the recessed cooktop",
        caps_ok and len(quadrants) == 4,
        f"quadrants={quadrants!r}",
    )

    # --- grates: rest on the cooktop rim and clear the burner caps --------
    for gi, grate in enumerate(grates):
        ctx.expect_gap(
            grate, body,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0005,
            name=f"grate_{gi} rests on cooktop rim",
        )
        ctx.expect_overlap(
            grate, body,
            axes="xy",
            min_overlap=0.02,
            name=f"grate_{gi} bars seated over the cooktop",
        )
    ctx.expect_gap(
        grates[0], body,
        axis="z",
        negative_elem="burner_cap_1",
        min_gap=0.0005,
        max_gap=0.01,
        name="grate bars pass just above the burner caps (left)",
    )
    ctx.expect_gap(
        grates[1], body,
        axis="z",
        negative_elem="burner_cap_0",
        min_gap=0.0005,
        max_gap=0.01,
        name="grate bars pass just above the burner caps (right)",
    )

    # --- oven cavity stays hollow behind the door -------------------------
    probe = (0.05, 0.0, OVEN_FLOOR + 0.20)
    blockers = []
    for vis in body.visuals:
        aabb = ctx.part_element_world_aabb(body, elem=vis.name)
        if aabb is None:
            continue
        amin, amax = aabb
        if all(amin[k] <= probe[k] <= amax[k] for k in range(3)):
            blockers.append(vis.name)
    ctx.check(
        "oven cavity hollow behind the door",
        not blockers,
        f"solid geometry at cavity probe point: {blockers!r}",
    )

    # Oven rack shelf sits inside the cavity.
    rack_aabb = ctx.part_element_world_aabb(body, elem="oven_rack")
    ok_rack = rack_aabb is not None
    if rack_aabb is not None:
        rmin, rmax = rack_aabb
        rz = 0.5 * (rmin[2] + rmax[2])
        ok_rack = OVEN_FLOOR < rz < DOOR_TOP and rmin[1] > -0.21 and rmax[1] < 0.21
    ctx.check("oven rack shelf inside the cavity", ok_rack, f"rack_aabb={rack_aabb!r}")

    return ctx.report()


object_model = build_object_model()
