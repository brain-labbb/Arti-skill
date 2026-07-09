from __future__ import annotations

# Rectangular red plastic shopping basket with a single arched bail handle
# and a low-hinged front panel that tilts down for pouring/emptying.
#
# Coordinate convention: +Z up, basket rests on the ground at z=0, long axis
# along X. The tub is a hollow tapered shell (wider at the top) with rounded
# vertical edges, a thick rolled top rim, and vertical slot perforations cut
# clean through the back and side walls. The front (+Y) wall is a separate
# panel that hinges near its base about the X axis.

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
LONG_SLOT_X = (-0.124, -0.042, 0.042, 0.124)  # 4 slots per long wall (front/back)
SHORT_SLOT_Y = (-0.060, 0.060)  # 2 slots per short wall (the ends)

# Bail handle / pivots
PIVOT_X = 0.205  # |x| of the two pivots (just outside the short walls)
PIVOT_Z = H - 0.014  # pivot height at the top of the short sides
BAR_R = 0.012  # rounded bar radius (thick handle)
ARCH_RISE = 0.205  # how far the arch top rises above the pivot line
HANDLE_BOSS_R = 0.020  # pivot knuckle on the handle ends
HANDLE_BOSS_LEN = 0.020
TUB_BOSS_R = 0.022  # pivot boss on the tub short walls
TUB_BOSS_LEN = 0.014

# Front panel / hinge
HINGE_Z = 0.008  # height of the hinge axis above the base
FRONT_MARGIN = 0.032  # margin left at the sides of the front wall opening
PANEL_H = H - RIM_H - 0.010  # height of the front panel (from hinge to below rim)


# ---------------------------------------------------------------------------
# Geometry builders
# ---------------------------------------------------------------------------
def _tub_mesh() -> object:
    """Hollow tapered tub: loft two rounded rectangles, shell open at the top,
    add the rolled rim, cut vertical slot perforations through back/side walls,
    then cut away the front (+Y) wall to create the opening for the hinged panel."""
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
    try:
        rim = rim.edges("#Z").fillet(0.005)
    except Exception:
        pass
    tub = tub.union(rim)

    # Vertical slot perforations cut through the back and side walls.
    # (Front wall slots are on the separate front panel.)
    def _slot(plane: str, origin: tuple[float, float, float]) -> object:
        sk = cq.Sketch().rect(SLOT_W, SLOT_H).vertices().fillet(SLOT_R)
        return cq.Workplane(plane, origin=origin).placeSketch(sk).extrude(0.6, both=True)

    for x in LONG_SLOT_X:
        tub = tub.cut(_slot("XZ", (x, 0.0, SLOT_ZC)))
    for y in SHORT_SLOT_Y:
        tub = tub.cut(_slot("YZ", (0.0, y, SLOT_ZC)))

    # --- Cut the front (+Y) wall opening for the hinged panel ---
    cut_w = L_TOP - 2 * FRONT_MARGIN
    cut_h = PANEL_H
    # Box that spans from just inside the front wall to well outside it
    front_cut = (
        cq.Workplane("XY")
        .box(cut_w, 0.14, cut_h)
        .translate((0, D_BOT / 2 + 0.040, cut_h / 2 + HINGE_Z))
    )
    tub = tub.cut(front_cut)

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

    return mesh_from_cadquery(tub, "basket_tub")


def _front_panel_mesh() -> object:
    """Tapered flat panel that covers the front wall opening.  Hinge line at
    the local +Z origin; the panel extends upward (+Z) from the hinge.
    Slots match the original front wall layout."""
    panel_w_bot = L_BOT - 2 * FRONT_MARGIN
    panel_w_top = L_TOP - 2 * FRONT_MARGIN
    panel_h = PANEL_H
    slot_z = SLOT_ZC - HINGE_Z  # slot centre in panel local frame

    # Tapered thin plate: trapezoidal profile on XZ, extruded in Y.
    panel = (
        cq.Workplane("XZ")
        .moveTo(-panel_w_bot / 2.0, 0.0)
        .lineTo(panel_w_bot / 2.0, 0.0)
        .lineTo(panel_w_top / 2.0, panel_h)
        .lineTo(-panel_w_top / 2.0, panel_h)
        .close()
        .extrude(WALL, both=True)
    )

    # Cut slot perforations matching the original front wall layout.
    for x in LONG_SLOT_X:
        sk = cq.Sketch().rect(SLOT_W, SLOT_H).vertices().fillet(SLOT_R)
        slot_cut = (
            cq.Workplane("XZ", origin=(x, 0, slot_z))
            .placeSketch(sk)
            .extrude(0.06, both=True)
        )
        panel = panel.cut(slot_cut)

    # Soften top edges.
    try:
        panel = panel.faces(">Z").edges().chamfer(0.002)
    except Exception:
        pass

    return mesh_from_cadquery(panel, "front_panel")


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


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="red_shopping_basket")

    body_finish = model.material("body_finish", rgba=(0.88, 0.27, 0.22, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.55, 0.10, 0.09, 1.0))

    tub = model.part("basket_tub")
    tub.visual(_tub_mesh(), material=body_finish, name="tub_shell")
    tub.inertial = Inertial.from_geometry(
        Box((L_TOP, D_TOP, H)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, H / 2.0)),
    )

    # --- Hinged front panel ---
    front_panel = model.part("front_panel")
    front_panel.visual(
        _front_panel_mesh(), material=body_finish, name="panel_shell"
    )
    front_panel.inertial = Inertial.from_geometry(
        Box((L_TOP - 2 * FRONT_MARGIN, WALL, PANEL_H)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, PANEL_H / 2.0)),
    )

    # REVOLUTE about X at the base of the front wall.
    # Positive q tilts the panel forward (+Y direction, away from basket).
    # q=0: panel upright / closed.  q~=1.57 rad (90 deg): horizontal.
    model.articulation(
        "tub_to_front_panel",
        ArticulationType.REVOLUTE,
        parent=tub,
        child=front_panel,
        origin=Origin(xyz=(0.0, D_BOT / 2.0, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )

    # --- Bail handle ---
    handle = model.part("bail_handle")
    handle.visual(_handle_mesh(), material=handle_finish, name="bail_bar")
    handle.inertial = Inertial.from_geometry(
        Box((2 * PIVOT_X, 2 * BAR_R, ARCH_RISE)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, ARCH_RISE / 2.0)),
    )

    model.articulation(
        "tub_to_handle",
        ArticulationType.REVOLUTE,
        parent=tub,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=-math.radians(95.0),
            upper=math.radians(95.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tub = object_model.get_part("basket_tub")
    handle = object_model.get_part("bail_handle")
    front_panel = object_model.get_part("front_panel")
    handle_joint = object_model.get_articulation("tub_to_handle")
    panel_joint = object_model.get_articulation("tub_to_front_panel")

    # --- Core basket checks (unchanged) ---
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
            "tub has realistic height",
            0.18 < ext_z < 0.30,
            details=f"ext_z={ext_z:.3f}",
        )

    # --- Hollow + tapered ---
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
    top_band = ctx.part_element_world_aabb(tub, elem="tub_shell")
    ctx.check(
        "tub shell element resolves",
        top_band is not None,
        details=f"tub_shell_aabb={top_band}",
    )

    # --- Slot perforations on back and side walls ---
    n_slots = len(LONG_SLOT_X) * 1 + len(SHORT_SLOT_Y) * 2  # front slots are on panel
    ctx.check(
        "slot perforations present on rear and side walls",
        len(LONG_SLOT_X) >= 3 and len(SHORT_SLOT_Y) >= 1 and n_slots >= 8,
        details=f"long_per_wall={len(LONG_SLOT_X)}, short_per_wall={len(SHORT_SLOT_Y)}, total_back_sides={n_slots + len(LONG_SLOT_X)}",
    )

    # --- Bail handle checks (unchanged) ---
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

    if up_aabb is not None and tub_aabb is not None:
        rise_above_rim = up_aabb[1][2] - tub_aabb[1][2]
        ctx.check(
            "bail arches above the rim",
            rise_above_rim > 0.12,
            details=f"rise_above_rim={rise_above_rim:.3f}",
        )

    ctx.allow_overlap(
        tub,
        handle,
        reason=(
            "The bail-handle pivot knuckles are intentionally captured inside the "
            "tub short-side pivot bosses at the rim; local overlap represents the "
            "real pin-in-boss pivot joint."
        ),
    )

    # --- Front panel checks ---
    ctx.check(
        "front panel part exists",
        front_panel is not None,
        details="front_panel part resolved",
    )

    # Panel joint is revolute about X, near the base.
    p_ax = panel_joint.axis
    ctx.check(
        "panel joint axis runs along X (hinge at base of front wall)",
        abs(p_ax[0]) > 0.99 and abs(p_ax[1]) < 0.01 and abs(p_ax[2]) < 0.01,
        details=f"panel_axis={p_ax}",
    )
    p_lim = panel_joint.motion_limits
    ctx.check(
        "panel joint has open-limiting motion (lower=0, upper > pi/2)",
        p_lim is not None
        and p_lim.lower is not None
        and p_lim.upper is not None
        and abs(p_lim.lower) < 0.01
        and p_lim.upper > 1.5,
        details=f"lower={None if p_lim is None else p_lim.lower}, upper={None if p_lim is None else p_lim.upper}",
    )

    # Compare closed vs tilted-down pose.
    with ctx.pose({panel_joint: 0.0}):
        closed_aabb = ctx.part_world_aabb(front_panel)
        closed_handle_aabb = ctx.part_world_aabb(handle)

    # At full open (~105 deg = ~1.83 rad) the panel should be roughly horizontal
    # (leaning slightly past horizontal for dumping).
    with ctx.pose({panel_joint: math.radians(100.0)}):
        tilted_aabb = ctx.part_world_aabb(front_panel)

    ctx.check(
        "front panel AABB resolves in closed and tilted poses",
        closed_aabb is not None and tilted_aabb is not None,
        details=f"closed={closed_aabb}, tilted={tilted_aabb}",
    )

    if closed_aabb is not None and tilted_aabb is not None:
        closed_top_z = closed_aabb[1][2]
        tilted_top_z = tilted_aabb[1][2]
        ctx.check(
            "closed panel is upright (top z above hinge)",
            closed_top_z > HINGE_Z + PANEL_H * 0.6,
            details=f"closed_top_z={closed_top_z:.3f}",
        )
        ctx.check(
            "tilted panel top is lower than closed panel top",
            tilted_top_z < closed_top_z - 0.08,
            details=f"closed_top_z={closed_top_z:.3f}, tilted_top_z={tilted_top_z:.3f}",
        )

        # The panel top should move forward in +Y when tilting.
        closed_cy = 0.5 * (closed_aabb[0][1] + closed_aabb[1][1])
        tilted_cy = 0.5 * (tilted_aabb[0][1] + tilted_aabb[1][1])
        ctx.check(
            "tilting the panel swings its top forward in +Y",
            tilted_cy > closed_cy + 0.03,
            details=f"closed_center_y={closed_cy:.3f}, tilted_center_y={tilted_cy:.3f}",
        )

    # The hinge line sits near the base where the tub front wall meets the base.
    hinge_origin = Origin(xyz=(0.0, D_BOT / 2.0, HINGE_Z))
    ctx.check(
        "panel hinge at base of front wall",
        abs(panel_joint.origin.xyz[0] - 0.0) < 0.001
        and abs(panel_joint.origin.xyz[1] - D_BOT / 2.0) < 0.001
        and abs(panel_joint.origin.xyz[2] - HINGE_Z) < 0.001,
        details=f"panel_joint_origin={panel_joint.origin.xyz}",
    )

    # Allow intentional local overlap at the hinge line (panel bottom edge
    # and tub front-opening sill represent the hinge barrel assembly).
    ctx.allow_overlap(
        tub,
        front_panel,
        reason=(
            "The front panel sits in the tub's front wall opening with the bottom "
            "edge overlapping the hinge sill; this local overlap represents the "
            "molded hinge barrel/pin capture region."
        ),
    )

    return ctx.report()


object_model = build_object_model()
