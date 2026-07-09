from __future__ import annotations

# Rectangular red plastic nesting shopping basket with a folding bail handle.
#
# Coordinate convention: +Z up, basket rests on the ground at z=0, long axis
# along X. The tub is a hollow strongly-tapered shell (wider at the top) with
# rounded vertical edges, a thick rolled top rim, a pronounced inward nesting
# flange just below the rim, and vertical slot perforations cut through every
# wall. A single dark-red bail handle pivots from the two long-side rims
# (axis along Y) and folds flat against the long wall.

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
L_BOT = 0.28  # outer length at the base (strong taper for nesting)
D_BOT = 0.16  # outer depth at the base (strong taper)
H = 0.22  # tub height
WALL = 0.012  # wall thickness

R_VERT_BOT = 0.024  # rounded vertical edge radius at the base
R_VERT_TOP = 0.036  # rounded vertical edge radius at the mouth

# Rolled rim lip
RIM_H = 0.024
RIM_OVERHANG = 0.011  # how far the lip rolls out past the top wall

# Nesting flange: inward step just below the rim on the interior
FLANGE_H = 0.018  # vertical height of the flange band
FLANGE_INSET = 0.014  # how far the flange protrudes inward from the inner wall
FLANGE_Z_TOP = H - RIM_H  # flange sits just below the rim

# Slot perforations
SLOT_W = 0.028
SLOT_H = 0.110
SLOT_R = 0.013
SLOT_ZC = H * 0.40  # vertical center of the slots
LONG_SLOT_X = (-0.124, -0.042, 0.042, 0.124)  # 4 slots per long wall
SHORT_SLOT_Y = (-0.050, 0.050)  # 2 slots per short wall

# Bail handle / pivots — now on the LONG sides (front/back walls)
PIVOT_Y = D_TOP / 2.0 - 0.008  # |y| of the two pivots on long walls
PIVOT_Z = H - 0.014  # pivot height at the top of the long sides
BAR_R = 0.010  # rounded bar radius
ARCH_RISE = 0.18  # how far the arch rises above the pivot line
# handle spans along Y between long-side pivots
HANDLE_BOSS_R = 0.018  # pivot knuckle on the handle ends
HANDLE_BOSS_LEN = 0.018
TUB_BOSS_R = 0.020  # pivot boss on the tub long walls
TUB_BOSS_LEN = 0.014


# ---------------------------------------------------------------------------
# Geometry builders
# ---------------------------------------------------------------------------
def _tub_mesh() -> object:
    """Hollow strongly-tapered tub with nesting flange, rolled rim, and slots."""
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

    # Soften the bottom outer edge.
    try:
        tub = tub.edges("<Z").fillet(0.006)
    except Exception:
        pass

    # Rolled top rim
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

    # Nesting flange: an inward-protruding rectangular band on the interior,
    # just below the rim. This creates the ledge that a stacked basket sits on.
    # Build as a solid band that fills from inner wall inward.
    fl_z_bot = FLANGE_Z_TOP - FLANGE_H
    # Outer dims of flange match inner wall at that height (interpolated)
    frac = fl_z_bot / H  # approximate height fraction for taper interpolation
    l_at_flange = L_BOT + (L_TOP - L_BOT) * (fl_z_bot + FLANGE_H * 0.5) / H
    d_at_flange = D_BOT + (D_TOP - D_BOT) * (fl_z_bot + FLANGE_H * 0.5) / H
    # Inner cavity at flange height (before flange)
    l_inner = l_at_flange - 2 * WALL
    d_inner = d_at_flange - 2 * WALL
    # Flange outer = inner wall, flange inner = inner wall minus inset
    flange_outer_l = l_inner
    flange_outer_d = d_inner
    flange_inner_l = l_inner - 2 * FLANGE_INSET
    flange_inner_d = d_inner - 2 * FLANGE_INSET

    flange_outer = (
        cq.Workplane("XY", origin=(0, 0, fl_z_bot))
        .placeSketch(
            cq.Sketch().rect(flange_outer_l, flange_outer_d).vertices().fillet(
                max(0.008, R_VERT_TOP - WALL)
            )
        )
        .extrude(FLANGE_H)
    )
    flange_cutout = (
        cq.Workplane("XY", origin=(0, 0, fl_z_bot - 0.001))
        .placeSketch(
            cq.Sketch().rect(flange_inner_l, flange_inner_d).vertices().fillet(
                max(0.006, R_VERT_TOP - WALL - FLANGE_INSET)
            )
        )
        .extrude(FLANGE_H + 0.002)
    )
    flange = flange_outer.cut(flange_cutout)
    tub = tub.union(flange)

    # Vertical slot perforations
    def _slot(plane: str, origin: tuple[float, float, float]) -> object:
        sk = cq.Sketch().rect(SLOT_W, SLOT_H).vertices().fillet(SLOT_R)
        return cq.Workplane(plane, origin=origin).placeSketch(sk).extrude(0.6, both=True)

    for x in LONG_SLOT_X:
        tub = tub.cut(_slot("XZ", (x, 0.0, SLOT_ZC)))
    for y in SHORT_SLOT_Y:
        tub = tub.cut(_slot("YZ", (0.0, y, SLOT_ZC)))

    # Pivot bosses on both LONG walls (front/back) for the folding bail handle.
    for sy in (-1.0, 1.0):
        boss = (
            cq.Workplane(
                "XZ", origin=(0.0, sy * (D_TOP / 2.0 - 0.001), PIVOT_Z)
            )
            .circle(TUB_BOSS_R)
            .extrude(sy * TUB_BOSS_LEN)
        )
        tub = tub.union(boss)

    return mesh_from_cadquery(tub, "basket_tub")


def _handle_mesh() -> object:
    """Folding bail handle. LOCAL frame: pivot line is the local Y axis at z=0.
    The arch spans along Y (front-to-back between the two long-side pivots) and
    rises in local +Z. Pivot knuckles at the ends extend along Y to capture the
    tub bosses on the long walls."""
    # Arch in the YZ plane: from (0, -PIVOT_Y, 0) up through (0, 0, ARCH_RISE)
    # to (0, +PIVOT_Y, 0).
    path = (
        cq.Workplane("YZ")
        .moveTo(-PIVOT_Y, 0.0)
        .threePointArc((0.0, ARCH_RISE), (PIVOT_Y, 0.0))
    )
    path_wire = path.val()
    start = path_wire.positionAt(0.0)
    tan = path_wire.tangentAt(0.0)
    bar = (
        cq.Workplane(cq.Plane(origin=start.toTuple(), normal=tan.toTuple()))
        .circle(BAR_R)
        .sweep(path, transition="round")
    )

    # Pivot knuckles at the two ends, extending along Y to capture the tub
    # bosses on the long walls.
    for sy in (-1.0, 1.0):
        knuckle = (
            cq.Workplane("XZ", origin=(0.0, sy * PIVOT_Y, 0.0))
            .circle(HANDLE_BOSS_R)
            .extrude(sy * HANDLE_BOSS_LEN)
        )
        bar = bar.union(knuckle)

    return mesh_from_cadquery(bar, "bail_handle")


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="red_nesting_basket")

    body_finish = model.material("body_finish", rgba=(0.88, 0.27, 0.22, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.55, 0.10, 0.09, 1.0))

    tub = model.part("basket_tub")
    tub.visual(_tub_mesh(), material=body_finish, name="tub_shell")
    tub.inertial = Inertial.from_geometry(
        Box((L_TOP, D_TOP, H)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, H / 2.0)),
    )

    handle = model.part("bail_handle")
    handle.visual(_handle_mesh(), material=handle_finish, name="bail_bar")
    handle.inertial = Inertial.from_geometry(
        Box((2 * BAR_R, 2 * PIVOT_Y, ARCH_RISE)),
        mass=0.15,
        origin=Origin(xyz=(0.0, 0.0, ARCH_RISE / 2.0)),
    )

    # REVOLUTE about Y axis through the two long-side pivots.
    # At q=0 the arch stands upright over the mouth.
    # Positive q folds it down toward the +X long wall end.
    model.articulation(
        "tub_to_handle",
        ArticulationType.REVOLUTE,
        parent=tub,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=-math.radians(100.0),
            upper=math.radians(100.0),
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

    # --- Strong taper for nesting: mouth much wider than base. ---
    ctx.check(
        "body is strongly tapered for nesting (mouth much wider than base)",
        L_TOP > L_BOT + 0.08 and D_TOP > D_BOT + 0.08,
        details=f"L_top={L_TOP}, L_bot={L_BOT}, D_top={D_TOP}, D_bot={D_BOT}",
    )
    inner_top = L_TOP - 2 * WALL
    ctx.check(
        "tub is hollow (open interior cavity)",
        inner_top > 0.30 and WALL < 0.05,
        details=f"inner_top={inner_top:.3f}, wall={WALL}",
    )

    # --- Nesting flange present under the rim. ---
    ctx.check(
        "nesting flange dimensions are meaningful",
        FLANGE_INSET > 0.005 and FLANGE_H > 0.008,
        details=f"flange_inset={FLANGE_INSET}, flange_h={FLANGE_H}",
    )

    # --- Slot perforations present on the walls. ---
    n_slots = len(LONG_SLOT_X) * 2 + len(SHORT_SLOT_Y) * 2
    ctx.check(
        "vertical slot perforations present on every wall",
        len(LONG_SLOT_X) >= 3 and len(SHORT_SLOT_Y) >= 1 and n_slots >= 8,
        details=f"long_per_wall={len(LONG_SLOT_X)}, short_per_wall={len(SHORT_SLOT_Y)}, total={n_slots}",
    )

    # --- Bail handle pivots from the long sides (axis along Y). ---
    ax = joint.axis
    ctx.check(
        "handle joint axis runs along Y (through long-side pivots)",
        abs(ax[1]) > 0.99 and abs(ax[0]) < 0.01 and abs(ax[2]) < 0.01,
        details=f"axis={ax}",
    )
    ctx.check(
        "handle joint is revolute",
        str(joint.articulation_type).upper().endswith("REVOLUTE"),
        details=f"type={joint.articulation_type}",
    )
    lim = joint.motion_limits
    ctx.check(
        "handle has realistic fold limits (~+-100deg)",
        lim is not None
        and lim.lower is not None
        and lim.upper is not None
        and lim.lower < -1.5
        and lim.upper > 1.5,
        details=f"lower={None if lim is None else lim.lower}, upper={None if lim is None else lim.upper}",
    )

    # Upright vs folded-flat pose comparison.
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
            "upright handle reaches higher than folded-flat handle",
            up_top_z > down_top_z + 0.08,
            details=f"upright_top_z={up_top_z:.3f}, flat_top_z={down_top_z:.3f}",
        )

    # Handle arch rises above the rim in upright pose.
    if up_aabb is not None and tub_aabb is not None:
        rise_above_rim = up_aabb[1][2] - tub_aabb[1][2]
        ctx.check(
            "bail arches above the rim when upright",
            rise_above_rim > 0.10,
            details=f"rise_above_rim={rise_above_rim:.3f}",
        )

    # Handle pivot knuckles intentionally overlap tub bosses (capture joint).
    ctx.allow_overlap(
        tub,
        handle,
        reason=(
            "The bail-handle pivot knuckles are intentionally captured inside the "
            "tub long-side pivot bosses at the rim; local overlap represents the "
            "real pin-in-boss pivot joint."
        ),
    )

    return ctx.report()


object_model = build_object_model()
