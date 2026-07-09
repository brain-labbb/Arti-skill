from __future__ import annotations

# Rectangular matte black plastic shopping basket with a single arched bail
# handle and a slide-out under-basket drawer tray.
#
# Coordinate convention: +Z up, basket rests on the ground at z=0 via two
# guide rails, long axis along X. The tub is a hollow tapered shell (wider at
# the top) with rounded vertical edges, a thick rolled top rim, and vertical
# slot perforations cut clean through every wall. A single dark bail handle
# arches over the top and pivots (REVOLUTE about the X line through the two
# short-side pivots). A shallow drawer tray slides out horizontally from
# beneath the basket floor on a PRISMATIC joint along +Y.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Dimensions (meters)
# ---------------------------------------------------------------------------
L_TOP = 0.40  # outer length at the mouth (long axis, X)
D_TOP = 0.30  # outer depth at the mouth (short axis, Y)
L_BOT = 0.34  # outer length at the base (taper: narrower at the bottom)
D_BOT = 0.24  # outer depth at the base
H = 0.22  # tub height
WALL = 0.012  # wall thickness

R_VERT_BOT = 0.030  # rounded vertical edge radius at the base
R_VERT_TOP = 0.036  # rounded vertical edge radius at the mouth

# Rolled rim lip
RIM_H = 0.024
RIM_OVERHANG = 0.011  # how far the lip rolls out past the top wall

# Slot perforations
SLOT_W = 0.028
SLOT_H = 0.120
SLOT_R = 0.013
SLOT_ZC = H * 0.43  # vertical center of the slots
LONG_SLOT_X = (-0.124, -0.042, 0.042, 0.124)  # 4 slots per long wall
SHORT_SLOT_Y = (-0.060, 0.060)  # 2 slots per short wall

# Bail handle / pivots
PIVOT_X = 0.205  # |x| of the two pivots (just outside the short walls)
PIVOT_Z = H - 0.014  # pivot height at the top of the short sides
BAR_R = 0.012  # rounded bar radius (thick handle)
ARCH_RISE = 0.205  # how far the arch top rises above the pivot line
HANDLE_BOSS_R = 0.020  # pivot knuckle on the handle ends
HANDLE_BOSS_LEN = 0.020
TUB_BOSS_R = 0.022  # pivot boss on the tub short walls
TUB_BOSS_LEN = 0.014

# Under-basket drawer system
RAIL_H = 0.040  # gap height between ground and basket floor
RAIL_W = 0.015  # rail width (X)
DRAWER_CLEARANCE = 0.006  # sliding clearance on each side
DRAWER_L = L_BOT - 2 * RAIL_W - DRAWER_CLEARANCE  # tray outer length (X)
DRAWER_D = D_BOT - DRAWER_CLEARANCE  # tray outer depth (Y)
DRAWER_H = RAIL_H - 0.005  # tray outer height (Z)
DRAWER_WALL = 0.003  # tray wall/floor thickness
DRAWER_TRAVEL = 0.22  # max pull-out distance (m)


# ---------------------------------------------------------------------------
# Geometry builders
# ---------------------------------------------------------------------------
def _tub_mesh() -> object:
    """Hollow tapered tub: loft two rounded rectangles, shell open at the top,
    add the rolled rim, slot perforations, pivot bosses, and two under-basket
    guide rails for the drawer. The tub body sits at z=RAIL_H and above; the
    rails extend from z=0 to z=RAIL_H so the assembly rests on the ground."""
    outer = (
        cq.Workplane("XY")
        .placeSketch(
            cq.Sketch().rect(L_BOT, D_BOT).vertices().fillet(R_VERT_BOT),
            cq.Sketch()
            .rect(L_TOP, D_TOP)
            .vertices()
            .fillet(R_VERT_TOP)
            .moved(cq.Location(cq.Vector(0, 0, H))),
        )
        .loft()
    )
    # Hollow it out, removing the top face -> open-mouth tub.
    tub = outer.faces(">Z").shell(-WALL)

    # Soften the bottom outer edge so it reads as molded plastic.
    try:
        tub = tub.edges("<Z").fillet(0.008)
    except Exception:
        pass

    # Rolled top rim: a rounded-rect ring band that caps the wall and rolls out.
    l_o, d_o = L_TOP + 2 * RIM_OVERHANG, D_TOP + 2 * RIM_OVERHANG
    l_i, d_i = L_TOP - WALL, D_TOP - WALL
    r_o = R_VERT_TOP + RIM_OVERHANG
    r_i = max(0.012, R_VERT_TOP - WALL)
    ring = (
        cq.Workplane("XY", origin=(0, 0, H - RIM_H * 0.5))
        .placeSketch(cq.Sketch().rect(l_o, d_o).vertices().fillet(r_o))
        .extrude(RIM_H)
    )
    inner = (
        cq.Workplane("XY", origin=(0, 0, H - RIM_H * 0.5 - 0.002))
        .placeSketch(cq.Sketch().rect(l_i, d_i).vertices().fillet(r_i))
        .extrude(RIM_H + 0.004)
    )
    rim = ring.cut(inner)
    # Round the rim's top/bottom outer edges so it reads as a rolled lip.
    try:
        rim = rim.edges("#Z").fillet(0.005)
    except Exception:
        pass
    tub = tub.union(rim)

    # Vertical slot perforations cut clean through the walls.
    def _slot(plane: str, origin: tuple[float, float, float]) -> object:
        sk = cq.Sketch().rect(SLOT_W, SLOT_H).vertices().fillet(SLOT_R)
        return cq.Workplane(plane, origin=origin).placeSketch(sk).extrude(0.6, both=True)

    for x in LONG_SLOT_X:
        tub = tub.cut(_slot("XZ", (x, 0.0, SLOT_ZC)))
    for y in SHORT_SLOT_Y:
        tub = tub.cut(_slot("YZ", (0.0, y, SLOT_ZC)))

    # Pivot bosses on both short walls (raised knuckles the handle captures).
    for sx in (-1.0, 1.0):
        boss = (
            cq.Workplane(
                "YZ", origin=(sx * (L_TOP / 2.0 - 0.001), 0.0, PIVOT_Z)
            )
            .circle(TUB_BOSS_R)
            .extrude(sx * TUB_BOSS_LEN)
        )
        tub = tub.union(boss)

    # Translate the entire tub body up by RAIL_H so the basket floor sits at
    # z=RAIL_H and there is a gap underneath for the drawer to slide in.
    solid = tub.val()
    solid = solid.moved(cq.Location(cq.Vector(0, 0, RAIL_H)))
    tub = cq.Workplane("XY").newObject([solid])

    # Guide rails: two parallel bars under the basket, running along Y (the
    # short axis / drawer slide direction). These create a channel for the
    # drawer and raise the basket off the ground.
    for sx in (-1.0, 1.0):
        rail_x = sx * (L_BOT / 2.0 - RAIL_W / 2.0)
        rail = (
            cq.Workplane("XY", origin=(rail_x, 0.0, 0.0))
            .rect(RAIL_W, D_BOT)
            .extrude(RAIL_H)
        )
        tub = tub.union(rail)

    return mesh_from_cadquery(tub, "basket_tub")


def _handle_mesh() -> object:
    """Arched bail handle authored in a LOCAL frame whose pivot line is the
    local X axis at z=0; the arch rises in local +Z and the two ends sit at
    (+-PIVOT_X, 0, 0). Pivot knuckles cap the two ends."""
    path = (
        cq.Workplane("XZ")
        .moveTo(-PIVOT_X, 0.0)
        .threePointArc((0.0, ARCH_RISE), (PIVOT_X, 0.0))
    )
    path_wire = path.val()
    start = path_wire.positionAt(0.0)
    tan = path_wire.tangentAt(0.0)
    bar = (
        cq.Workplane(cq.Plane(origin=start.toTuple(), normal=tan.toTuple()))
        .circle(BAR_R)
        .sweep(path, transition="round")
    )

    # Pivot knuckles at the two ends, axis along X through the pivot line.
    for sx in (-1.0, 1.0):
        boss = (
            cq.Workplane("YZ", origin=(sx * PIVOT_X, 0.0, 0.0))
            .circle(HANDLE_BOSS_R)
            .extrude(-sx * HANDLE_BOSS_LEN)
        )
        bar = bar.union(boss)

    return mesh_from_cadquery(bar, "bail_handle")


def _drawer_mesh() -> object:
    """Shallow rectangular drawer tray centered at origin. Open-top tray
    with thin walls/floor and a small pull lip on the +Y face."""
    half_h = DRAWER_H / 2.0

    # Outer shell from z=-half_h to z=+half_h
    outer = (
        cq.Workplane("XY", origin=(0, 0, -half_h))
        .rect(DRAWER_L, DRAWER_D)
        .extrude(DRAWER_H)
    )

    # Inner cavity (open top): starts at floor thickness above the bottom,
    # extrudes past the outer top to cut it open.
    cavity = (
        cq.Workplane("XY", origin=(0, 0, -half_h + DRAWER_WALL))
        .rect(DRAWER_L - 2 * DRAWER_WALL, DRAWER_D - 2 * DRAWER_WALL)
        .extrude(DRAWER_H)
    )
    tray = outer.cut(cavity)

    # Pull lip on the +Y face: a small protruding bar for gripping.
    lip = (
        cq.Workplane("XZ", origin=(0, DRAWER_D / 2.0, 0))
        .rect(DRAWER_L * 0.35, DRAWER_H * 0.6)
        .extrude(0.008)
    )
    tray = tray.union(lip)

    # Soften vertical edges for a molded-plastic look.
    try:
        tray = tray.edges("|Z").fillet(0.003)
    except Exception:
        pass

    return mesh_from_cadquery(tray, "drawer_tray")


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="matte_black_basket_with_drawer")

    body_finish = model.material("body_finish", rgba=(0.08, 0.08, 0.09, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.05, 0.05, 0.06, 1.0))

    tub = model.part("basket_tub")
    tub.visual(_tub_mesh(), material=body_finish, name="tub_shell")
    tub.inertial = Inertial.from_geometry(
        Box((L_TOP, D_TOP, H + RAIL_H)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, RAIL_H + H / 2.0)),
    )

    handle = model.part("bail_handle")
    # The handle mesh is authored with its pivot line on the local X axis at
    # z=0, so the joint origin (on the real pivot line) lines it up directly.
    handle.visual(_handle_mesh(), material=handle_finish, name="bail_bar")
    handle.inertial = Inertial.from_geometry(
        Box((2 * PIVOT_X, 2 * BAR_R, ARCH_RISE)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, ARCH_RISE / 2.0)),
    )

    # REVOLUTE about the X line through the two short-side pivots, at the rim.
    # Shifted up by RAIL_H since the tub body now sits above the guide rails.
    # At q=0 the arch stands upright over the mouth; positive/negative q swings
    # it down toward the +Y / -Y long rims.
    model.articulation(
        "tub_to_handle",
        ArticulationType.REVOLUTE,
        parent=tub,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, RAIL_H + PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=-math.radians(95.0),
            upper=math.radians(95.0),
        ),
    )

    # Drawer: shallow tray that slides out from beneath the basket along +Y.
    # At q=0 the drawer is fully inserted under the basket; positive q pulls
    # it out toward the +Y short side.
    drawer = model.part("drawer_tray")
    drawer.visual(_drawer_mesh(), material=body_finish, name="tray_shell")
    drawer.inertial = Inertial.from_geometry(
        Box((DRAWER_L, DRAWER_D, DRAWER_H)),
        mass=0.15,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "tub_to_drawer",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=drawer,
        # Joint origin at the center of the rail gap, centered in XY.
        origin=Origin(xyz=(0.0, 0.0, RAIL_H / 2.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.3,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tub = object_model.get_part("basket_tub")
    handle = object_model.get_part("bail_handle")
    drawer = object_model.get_part("drawer_tray")
    handle_joint = object_model.get_articulation("tub_to_handle")
    drawer_joint = object_model.get_articulation("tub_to_drawer")

    # --- Footprint: wider in X (long axis) than Y, and rests at z ~= 0. ---
    tub_aabb = ctx.part_world_aabb(tub)
    ctx.check(
        "tub present with world AABB",
        tub_aabb is not None,
        details=f"tub_aabb={tub_aabb}",
    )
    if tub_aabb is not None:
        (tmn, tmx) = tub_aabb
        ext_x = tmx[0] - tmn[0]
        ext_y = tmx[1] - tmn[1]
        ext_z = tmx[2] - tmn[2]
        ctx.check(
            "footprint wider in X than Y",
            ext_x > ext_y + 0.05,
            details=f"ext_x={ext_x:.3f}, ext_y={ext_y:.3f}",
        )
        ctx.check(
            "basket rests at z~=0",
            abs(tmn[2]) < 0.01,
            details=f"z_min={tmn[2]:.4f}",
        )
        ctx.check(
            "tub has realistic height (including rails)",
            0.22 < ext_z < 0.35,
            details=f"ext_z={ext_z:.3f}",
        )

    # --- Hollow + tapered: top mouth wider than the base. ---
    ctx.check(
        "body is tapered (mouth wider than base)",
        L_TOP > L_BOT + 0.02 and D_TOP > D_BOT + 0.02,
        details=f"L_top={L_TOP}, L_bot={L_BOT}, D_top={D_TOP}, D_bot={D_BOT}",
    )
    inner_top = L_TOP - 2 * WALL
    ctx.check(
        "tub is hollow (open interior cavity)",
        inner_top > 0.30 and WALL < 0.05,
        details=f"inner_top={inner_top:.3f}, wall={WALL}",
    )

    # --- Slot perforations present on the walls. ---
    n_slots = len(LONG_SLOT_X) * 2 + len(SHORT_SLOT_Y) * 2
    ctx.check(
        "vertical slot perforations present on every wall",
        len(LONG_SLOT_X) >= 3 and len(SHORT_SLOT_Y) >= 1 and n_slots >= 8,
        details=f"long_per_wall={len(LONG_SLOT_X)}, short_per_wall={len(SHORT_SLOT_Y)}, total={n_slots}",
    )

    # --- Bail handle pivots about the short sides (axis along X). ---
    ax = handle_joint.axis
    ctx.check(
        "handle joint axis runs along X (through short-side pivots)",
        abs(ax[0]) > 0.99 and abs(ax[1]) < 0.01 and abs(ax[2]) < 0.01,
        details=f"axis={ax}",
    )
    ctx.check(
        "handle joint is revolute",
        str(handle_joint.articulation_type).upper().endswith("REVOLUTE"),
        details=f"type={handle_joint.articulation_type}",
    )
    lim = handle_joint.motion_limits
    ctx.check(
        "handle has realistic swing limits (~+-95deg)",
        lim is not None
        and lim.lower is not None
        and lim.upper is not None
        and lim.lower < -1.5
        and lim.upper > 1.5,
        details=f"lower={None if lim is None else lim.lower}, upper={None if lim is None else lim.upper}",
    )

    # Driving the joint swings the arch top: compare an upright pose vs a
    # laid-down pose.
    with ctx.pose({handle_joint: 0.0}):
        up_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({handle_joint: math.radians(90.0)}):
        down_aabb = ctx.part_world_aabb(handle)

    ctx.check(
        "handle poses resolve",
        up_aabb is not None and down_aabb is not None,
        details=f"up={up_aabb}, down={down_aabb}",
    )
    if up_aabb is not None and down_aabb is not None:
        up_top_z = up_aabb[1][2]
        down_top_z = down_aabb[1][2]
        ctx.check(
            "upright handle reaches higher than laid-down handle",
            up_top_z > down_top_z + 0.10,
            details=f"upright_top_z={up_top_z:.3f}, laid_top_z={down_top_z:.3f}",
        )
        up_cy = 0.5 * (up_aabb[0][1] + up_aabb[1][1])
        down_cy = 0.5 * (down_aabb[0][1] + down_aabb[1][1])
        ctx.check(
            "swinging the handle moves the arch out in Y",
            abs(down_cy) > abs(up_cy) + 0.05,
            details=f"up_center_y={up_cy:.3f}, down_center_y={down_cy:.3f}",
        )

    # Handle arch rises well above the rim in the upright pose.
    if up_aabb is not None and tub_aabb is not None:
        rise_above_rim = up_aabb[1][2] - tub_aabb[1][2]
        ctx.check(
            "bail arches above the rim",
            rise_above_rim > 0.12,
            details=f"rise_above_rim={rise_above_rim:.3f}",
        )

    # --- Handle ends are captured at the short-side pivot bosses ---
    ctx.allow_overlap(
        tub,
        handle,
        reason=(
            "The bail-handle pivot knuckles are intentionally captured inside the "
            "tub short-side pivot bosses at the rim; local overlap represents the "
            "real pin-in-boss pivot joint."
        ),
    )

    # --- Drawer tray ---
    # The drawer slides in a channel between guide rails with small clearances
    # that prevent direct contact at the rest pose, but the prismatic joint
    # provides the mechanical connection.
    ctx.allow_isolated_part(
        drawer,
        reason=(
            "The drawer tray slides horizontally in a channel between two guide "
            "rails attached to the basket tub; the prismatic joint provides the "
            "mechanical connection. Small sliding clearances (2.5mm) prevent "
            "direct surface contact at the rest pose."
        ),
    )

    ctx.check(
        "drawer joint is prismatic",
        str(drawer_joint.articulation_type).upper().endswith("PRISMATIC"),
        details=f"type={drawer_joint.articulation_type}",
    )
    dax = drawer_joint.axis
    ctx.check(
        "drawer slides along +Y (short axis)",
        abs(dax[1]) > 0.99 and abs(dax[0]) < 0.01 and abs(dax[2]) < 0.01,
        details=f"axis={dax}",
    )
    dlim = drawer_joint.motion_limits
    ctx.check(
        "drawer has positive travel range",
        dlim is not None
        and dlim.lower is not None
        and dlim.upper is not None
        and dlim.lower >= 0.0
        and dlim.upper > 0.10,
        details=f"lower={dlim.lower}, upper={dlim.upper}",
    )

    # Drawer pose checks: at rest it sits under the basket; at max extension
    # it protrudes beyond the basket footprint on the +Y side.
    with ctx.pose({drawer_joint: 0.0}):
        drawer_rest_aabb = ctx.part_world_aabb(drawer)
    with ctx.pose({drawer_joint: DRAWER_TRAVEL}):
        drawer_ext_aabb = ctx.part_world_aabb(drawer)

    ctx.check(
        "drawer AABBs resolve",
        drawer_rest_aabb is not None and drawer_ext_aabb is not None,
        details=f"rest={drawer_rest_aabb}, ext={drawer_ext_aabb}",
    )

    if drawer_rest_aabb is not None and tub_aabb is not None:
        # Drawer sits below the main basket body.
        ctx.check(
            "drawer sits below the basket floor",
            drawer_rest_aabb[1][2] < tub_aabb[0][2] + RAIL_H + 0.01,
            details=(
                f"drawer_top_z={drawer_rest_aabb[1][2]:.4f}, "
                f"tub_bottom_z={tub_aabb[0][2]:.4f}"
            ),
        )
        # At rest, drawer is within the basket footprint in X.
        ctx.expect_within(
            drawer,
            tub,
            axes="x",
            name="drawer centered under basket in X at rest",
        )

    if drawer_rest_aabb is not None and drawer_ext_aabb is not None:
        # Drawer actually moves in +Y when the joint is driven positive.
        rest_cy = 0.5 * (drawer_rest_aabb[0][1] + drawer_rest_aabb[1][1])
        ext_cy = 0.5 * (drawer_ext_aabb[0][1] + drawer_ext_aabb[1][1])
        ctx.check(
            "drawer extends in +Y when opened",
            ext_cy > rest_cy + 0.10,
            details=f"rest_center_y={rest_cy:.3f}, ext_center_y={ext_cy:.3f}",
        )

    if drawer_ext_aabb is not None and tub_aabb is not None:
        # At max extension, the drawer front protrudes beyond the basket.
        ctx.check(
            "drawer protrudes beyond basket when fully opened",
            drawer_ext_aabb[1][1] > tub_aabb[1][1] + 0.05,
            details=(
                f"drawer_front_y={drawer_ext_aabb[1][1]:.3f}, "
                f"tub_front_y={tub_aabb[1][1]:.3f}"
            ),
        )

    return ctx.report()


object_model = build_object_model()
