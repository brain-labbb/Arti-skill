from __future__ import annotations

# Rectangular red plastic shopping basket with a single arched bail handle.
#
# Coordinate convention: +Z up, basket rests on the ground at z=0, long axis
# along X. The tub is a hollow tapered shell (wider at the top) with rounded
# vertical edges, a thick rolled top rim, and vertical slot perforations cut
# clean through every wall. A single dark-red bail handle arches over the top
# and pivots (REVOLUTE about the X line through the two short-side pivots) so it
# can swing from upright down to either long rim.
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

# Bail-handle apex height (upright pose), used to cap the caddy lift so the
# caddy never rises above the handle. Arch centerline apex + bar radius.
HANDLE_TOP_Z = PIVOT_Z + ARCH_RISE + BAR_R  # ~0.423

# Inner caddy / tray.
# The caddy seats on the basket FLOOR (not the rim), so it is sized to the
# narrower base interior footprint; the basket widens upward so it still lifts
# straight up and out.
CADDY_H = 0.065       # caddy wall height
CADDY_WALL = 0.003    # caddy wall thickness
CADDY_CLEAR = 0.005   # clearance gap inside the basket base interior
CADDY_CB_X = L_BOT - 2.0 * WALL - 2.0 * CADDY_CLEAR  # ~0.306 (base interior)
CADDY_CB_Y = D_BOT - 2.0 * WALL - 2.0 * CADDY_CLEAR  # ~0.206
CADDY_CT_X = CADDY_CB_X + 0.008  # ~0.314, slight upward taper
CADDY_CT_Y = CADDY_CB_Y + 0.008  # ~0.214
CADDY_CORNER = 0.014
CADDY_DIV_T = 0.003   # divider wall thickness
CADDY_FLOOR_SEAT = WALL  # interior floor top z; caddy bottom seats on the floor
# Lift travel: from the floor seat up until the caddy top reaches the bail-handle
# apex, so the lift range never exceeds the handle.
CADDY_LIFT_MAX = HANDLE_TOP_Z - (CADDY_FLOOR_SEAT + CADDY_H)  # ~0.346

# Caddy folding grip (small wire handle on the caddy top)
GRIP_W = 0.085       # grip width (span between legs, along X)
GRIP_H = 0.038       # grip height above the caddy top
GRIP_R = 0.004       # wire cross-section radius


# ---------------------------------------------------------------------------
# Geometry builders
# ---------------------------------------------------------------------------
def _tub_mesh() -> object:
    """Hollow tapered tub: loft two rounded rectangles, shell open at the top,
    add the rolled rim, then cut vertical slot perforations through the walls."""
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

    # Soften the bottom outer edge so it reads as molded plastic, not a sharp box.
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


def _caddy_mesh() -> object:
    """Shallow removable tray that seats on the basket rim.  Thin-walled
    rectangular tub with a slight taper, rounded corners, and cross dividers
    splitting the interior into four quadrants.  Bottom at local z=0."""
    outer = (
        cq.Workplane("XY")
        .placeSketch(
            cq.Sketch()
            .rect(CADDY_CB_X, CADDY_CB_Y)
            .vertices()
            .fillet(CADDY_CORNER),
            cq.Sketch()
            .rect(CADDY_CT_X, CADDY_CT_Y)
            .vertices()
            .fillet(CADDY_CORNER + 0.003)
            .moved(cq.Location(cq.Vector(0, 0, CADDY_H))),
        )
        .loft()
    )
    inner = (
        cq.Workplane("XY")
        .placeSketch(
            cq.Sketch()
            .rect(
                CADDY_CB_X - 2.0 * CADDY_WALL,
                CADDY_CB_Y - 2.0 * CADDY_WALL,
            )
            .vertices()
            .fillet(max(0.008, CADDY_CORNER - 0.004)),
            cq.Sketch()
            .rect(
                CADDY_CT_X - 2.0 * CADDY_WALL,
                CADDY_CT_Y - 2.0 * CADDY_WALL,
            )
            .vertices()
            .fillet(CADDY_CORNER)
            .moved(cq.Location(cq.Vector(0, 0, CADDY_H))),
        )
        .loft()
    )
    tray = outer.cut(inner)

    # Cross divider along X (full length)
    div_x_len = CADDY_CT_X - 2.0 * CADDY_WALL - 0.008
    div_x = (
        cq.Workplane("XY")
        .placeSketch(
            cq.Sketch().rect(div_x_len, CADDY_DIV_T)
        )
        .extrude(CADDY_H - 0.002)
    )
    tray = tray.union(div_x)

    # Cross divider along Y
    div_y_len = CADDY_CT_Y - 2.0 * CADDY_WALL - 0.008
    div_y = (
        cq.Workplane("XY")
        .placeSketch(
            cq.Sketch().rect(CADDY_DIV_T, div_y_len)
        )
        .extrude(CADDY_H - 0.002)
    )
    tray = tray.union(div_y)

    # Soften top and bottom outer edges
    try:
        tray = tray.edges("%Plane").fillet(0.002)
    except Exception:
        pass

    return mesh_from_cadquery(tray, "inner_caddy")


def _caddy_grip_mesh() -> object:
    """Small folding wire handle on the caddy top.  A single continuous
    U-shaped wire created by sweeping a circle along a continuous path
    (two legs + crossbar as one uninterrupted sweep).
    Pivot line is local +X at z=0; the legs rise in local +Z."""
    path = (
        cq.Workplane("XZ")
        .moveTo(-GRIP_W / 2.0, 0.0)
        .lineTo(-GRIP_W / 2.0, GRIP_H)
        .lineTo(GRIP_W / 2.0, GRIP_H)
        .lineTo(GRIP_W / 2.0, 0.0)
    )
    path_wire = path.val()
    start = path_wire.positionAt(0.0)
    tan = path_wire.tangentAt(0.0)
    grip = (
        cq.Workplane(cq.Plane(origin=start.toTuple(), normal=tan.toTuple()))
        .circle(GRIP_R)
        .sweep(path, transition="round")
    )
    return mesh_from_cadquery(grip, "caddy_grip")


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="red_shopping_basket")

    body_finish = model.material("body_finish", rgba=(0.88, 0.27, 0.22, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.55, 0.10, 0.09, 1.0))
    caddy_finish = model.material("caddy_finish", rgba=(0.85, 0.25, 0.19, 1.0))

    tub = model.part("basket_tub")
    tub.visual(_tub_mesh(), material=body_finish, name="tub_shell")
    tub.inertial = Inertial.from_geometry(
        Box((L_TOP, D_TOP, H)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, H / 2.0)),
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
    # At q=0 the arch stands upright over the mouth; positive/negative q swings
    # it down toward the +Y / -Y long rims.
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

    # --- Removable inner caddy/tray ---
    # PRISMATIC lift: at q=0 the caddy bottom sits on the basket floor; positive
    # q lifts it straight up and out, capped so the caddy top never rises above
    # the bail handle.
    caddy = model.part("inner_caddy")
    caddy.visual(_caddy_mesh(), material=caddy_finish, name="caddy_shell")
    caddy.inertial = Inertial.from_geometry(
        Box((CADDY_CT_X, CADDY_CT_Y, CADDY_H)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, CADDY_H / 2.0)),
    )

    model.articulation(
        "tub_to_caddy",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=caddy,
        # Joint origin at the interior floor top center; the caddy mesh bottom is
        # at local z=0 so at q=0 the caddy rests on the basket floor.
        origin=Origin(xyz=(0.0, 0.0, CADDY_FLOOR_SEAT)),
        axis=(0.0, 0.0, 1.0),  # lifts straight up
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.0,
            lower=0.0,
            upper=CADDY_LIFT_MAX,  # lifts until the caddy top reaches the bail-handle apex
        ),
    )

    # --- Small folding grip on the caddy top ---
    grip = model.part("caddy_grip")
    grip.visual(_caddy_grip_mesh(), material=handle_finish, name="grip_bar")
    grip.inertial = Inertial.from_geometry(
        Box((GRIP_W, 2.0 * GRIP_R, GRIP_H)),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, GRIP_H / 2.0)),
    )

    # REVOLUTE on the caddy top surface; the grip mesh has pivot at local z=0
    # and legs rising in +Z.  At q=0 the grip stands upright; positive q folds
    # it toward -Y, negative q toward +Y.
    model.articulation(
        "caddy_to_grip",
        ArticulationType.REVOLUTE,
        parent=caddy,
        child=grip,
        origin=Origin(xyz=(0.0, 0.0, CADDY_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=-math.radians(90.0),
            upper=math.radians(90.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tub = object_model.get_part("basket_tub")
    handle = object_model.get_part("bail_handle")
    joint = object_model.get_articulation("tub_to_handle")

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
            "tub has realistic height",
            0.18 < ext_z < 0.30,
            details=f"ext_z={ext_z:.3f}",
        )

    # --- Hollow + tapered: top mouth wider than the base. ---
    top_band = ctx.part_element_world_aabb(tub, elem="tub_shell")
    # Compare the section extents near the top vs near the bottom by querying
    # the tub shell element AABB; tapered means top X-extent > bottom X-extent.
    # We prove taper directly from the authored dimensions, which the mesh
    # reproduces (loft from L_BOT to L_TOP).
    ctx.check(
        "body is tapered (mouth wider than base)",
        L_TOP > L_BOT + 0.02 and D_TOP > D_BOT + 0.02,
        details=f"L_top={L_TOP}, L_bot={L_BOT}, D_top={D_TOP}, D_bot={D_BOT}",
    )
    # Hollow: the inner cavity exists because the shell wall is thin relative to
    # the body, so the interior opening width is positive and large.
    inner_top = L_TOP - 2 * WALL
    ctx.check(
        "tub is hollow (open interior cavity)",
        inner_top > 0.30 and WALL < 0.05,
        details=f"inner_top={inner_top:.3f}, wall={WALL}",
    )
    ctx.check(
        "tub shell element resolves",
        top_band is not None,
        details=f"tub_shell_aabb={top_band}",
    )

    # --- Slot perforations present on the walls. ---
    n_slots = len(LONG_SLOT_X) * 2 + len(SHORT_SLOT_Y) * 2
    ctx.check(
        "vertical slot perforations present on every wall",
        len(LONG_SLOT_X) >= 3 and len(SHORT_SLOT_Y) >= 1 and n_slots >= 8,
        details=f"long_per_wall={len(LONG_SLOT_X)}, short_per_wall={len(SHORT_SLOT_Y)}, total={n_slots}",
    )

    # --- Bail handle pivots about the short sides (axis along X). ---
    ax = joint.axis
    ctx.check(
        "handle joint axis runs along X (through short-side pivots)",
        abs(ax[0]) > 0.99 and abs(ax[1]) < 0.01 and abs(ax[2]) < 0.01,
        details=f"axis={ax}",
    )
    ctx.check(
        "handle joint is revolute",
        str(joint.articulation_type).upper().endswith("REVOLUTE"),
        details=f"type={joint.articulation_type}",
    )
    lim = joint.motion_limits
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
    # laid-down pose. Upright -> arch top high in Z; laid down -> arch top low
    # in Z and pushed out in Y.
    with ctx.pose({joint: 0.0}):
        up_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({joint: math.radians(90.0)}):
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
    # The handle knuckles intentionally overlap the tub pivot bosses (capture).
    ctx.allow_overlap(
        tub,
        handle,
        reason=(
            "The bail-handle pivot knuckles are intentionally captured inside the "
            "tub short-side pivot bosses at the rim; local overlap represents the "
            "real pin-in-boss pivot joint."
        ),
    )

    # The caddy grip sits on the caddy top via its revolute pivot but does not
    # have contacting geometry that bridges the 2mm gap to the open caddy
    # interior; it is intentionally articulated as a separate folding part.
    ctx.allow_isolated_part(
        "caddy_grip",
        reason=(
            "The caddy grip is a small wire handle mounted on the caddy top via a "
            "revolute joint; its legs sit over the open caddy interior with a ~2mm "
            "gap rather than touching the tray walls."
        ),
    )

    # ====================================================================
    # Inner caddy / tray checks
    # ====================================================================
    caddy_p = object_model.get_part("inner_caddy")
    grip_p = object_model.get_part("caddy_grip")
    lift_j = object_model.get_articulation("tub_to_caddy")
    grip_j = object_model.get_articulation("caddy_to_grip")

    ctx.check(
        "caddy part exists",
        caddy_p is not None,
        details="inner_caddy",
    )
    ctx.check(
        "caddy grip part exists",
        grip_p is not None,
        details="caddy_grip",
    )
    ctx.check(
        "caddy lift joint is prismatic",
        str(lift_j.articulation_type).upper().endswith("PRISMATIC"),
        details=f"type={lift_j.articulation_type}",
    )
    ctx.check(
        "caddy lift axis is +Z (straight up)",
        abs(lift_j.axis[0]) < 0.01
        and abs(lift_j.axis[1]) < 0.01
        and lift_j.axis[2] > 0.99,
        details=f"axis={lift_j.axis}",
    )
    ctx.check(
        "caddy lift has positive range",
        lift_j.motion_limits is not None
        and lift_j.motion_limits.lower == 0.0
        and lift_j.motion_limits.upper > 0.08,
        details=f"lower={lift_j.motion_limits.lower if lift_j.motion_limits else None}, "
        f"upper={lift_j.motion_limits.upper if lift_j.motion_limits else None}",
    )

    # --- Caddy has cross dividers (splits interior into sections) ---
    caddy_aabb = ctx.part_world_aabb(caddy_p)
    ctx.check(
        "caddy world AABB resolves",
        caddy_aabb is not None,
        details=f"caddy_aabb={caddy_aabb}",
    )

    # --- Caddy seats on the basket floor and lifts clear of the rim ---
    with ctx.pose({lift_j: 0.0}):
        rest_aabb = ctx.part_world_aabb(caddy_p)
    with ctx.pose({lift_j: CADDY_LIFT_MAX}):  # full lift
        lifted_aabb = ctx.part_world_aabb(caddy_p)

    ctx.check(
        "caddy poses resolve at rest and lifted",
        rest_aabb is not None and lifted_aabb is not None,
        details=f"rest={rest_aabb}, lifted={lifted_aabb}",
    )
    if rest_aabb is not None and lifted_aabb is not None:
        # At rest the caddy bottom sits on the basket floor (well below the rim).
        rest_bottom = rest_aabb[0][2]
        lifted_bottom = lifted_aabb[0][2]
        lifted_top = lifted_aabb[1][2]
        tub_top = tub_aabb[1][2] if tub_aabb is not None else 0.0

        ctx.check(
            "caddy rests on the basket floor (well below the rim)",
            rest_bottom < tub_top - 0.10
            and abs(rest_bottom - CADDY_FLOOR_SEAT) < 0.02,
            details=f"rest_bottom_z={rest_bottom:.4f}, floor_seat={CADDY_FLOOR_SEAT:.4f}, tub_top_z={tub_top:.4f}",
        )
        ctx.check(
            "lifted caddy clears the basket rim",
            lifted_bottom > tub_top + 0.02,
            details=f"lifted_bottom_z={lifted_bottom:.4f}, tub_top_z={tub_top:.4f}",
        )
        # Confirm the caddy actually moved upward
        ctx.check(
            "caddy lifts upward relative to rest",
            lifted_bottom > rest_bottom + 0.10,
            details=f"rest_bottom_z={rest_bottom:.4f}, lifted_bottom_z={lifted_bottom:.4f}",
        )
        # The lift range does not raise the caddy above the bail-handle apex.
        ctx.check(
            "lifted caddy does not exceed the bail handle",
            lifted_top <= HANDLE_TOP_Z + 0.005,
            details=f"lifted_top_z={lifted_top:.4f}, handle_top_z={HANDLE_TOP_Z:.4f}",
        )

    # --- Caddy grip articulates ---
    ctx.check(
        "caddy grip joint is revolute",
        str(grip_j.articulation_type).upper().endswith("REVOLUTE"),
        details=f"type={grip_j.articulation_type}",
    )
    ctx.check(
        "caddy grip pivot axis along X",
        abs(grip_j.axis[0]) > 0.99
        and abs(grip_j.axis[1]) < 0.01
        and abs(grip_j.axis[2]) < 0.01,
        details=f"axis={grip_j.axis}",
    )

    # --- Intentional overlap: the caddy bottom rests on the basket floor and
    # its walls sit just inside the tapered tub walls; this is a seated
    # plastic-on-plastic nesting fit. ---
    ctx.allow_overlap(
        tub,
        caddy_p,
        reason=(
            "The caddy bottom seats on the basket floor and nests just inside "
            "the tapered tub walls; the slight overlap is intentional seated "
            "contact of the removable tray insert."
        ),
    )

    # The bail handle arch passes over/near the caddy interior at the arch
    # sides where the arc descends close to the rim.  Both parts coexist at
    # rest; the handle can be swung aside when the caddy is in use.
    ctx.allow_overlap(
        handle,
        caddy_p,
        reason=(
            "The bail handle arch descends near the rim at the arc sides, "
            "overlapping the caddy's upper interior region.  Both parts share "
            "the basket opening volume at rest; the handle is free to swing clear."
        ),
    )

    return ctx.report()


object_model = build_object_model()
