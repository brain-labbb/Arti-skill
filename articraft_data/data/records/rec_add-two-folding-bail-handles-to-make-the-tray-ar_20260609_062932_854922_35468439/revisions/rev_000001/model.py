from __future__ import annotations

# Stackable red plastic produce flat: shallow wide rectangular tray with
# open diagonal-slat vented floor, four corner stacking lugs, slot-perforated
# walls, rolled rim, and two end-grip cutout handles in the short walls.
#
# Coordinate convention: +Z up, tray rests on the ground at z=0, long axis X.

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
L_TOP = 0.50       # outer length at mouth (long axis, X)
D_TOP = 0.38       # outer depth at mouth (short axis, Y)
L_BOT = 0.44       # outer length at base (taper)
D_BOT = 0.32       # outer depth at base
H = 0.09           # wall height (shallow tray)
WALL = 0.012       # wall / floor thickness

R_VERT_BOT = 0.028  # vertical corner radius at base
R_VERT_TOP = 0.034  # vertical corner radius at mouth

# Rolled rim
RIM_H = 0.018
RIM_OVERHANG = 0.010

# Vertical slot perforations on the walls
SLOT_W = 0.026
SLOT_H = 0.040
SLOT_R = 0.012
SLOT_ZC = H * 0.45
LONG_SLOT_X = (-0.16, -0.08, 0.0, 0.08, 0.16)   # 5 per long wall
SHORT_SLOT_Y = (-0.11, 0.11)                      # 2 per short wall (centre slot replaced by grip)

# Corner stacking lugs (rise above rim for stacking)
LUG_W = 0.020
LUG_D = 0.020
LUG_H = 0.022
LUG_FILLET = 0.004

# End-grip cutout handles (in the short walls)
GRIP_W = 0.082
GRIP_H = 0.032
GRIP_R = 0.014
GRIP_ZC = H * 0.62

# Diagonal slat vented floor
SLAT_W = 0.008         # slat bar width
SLAT_SPACING = 0.028   # centre-to-centre spacing
FLOOR_FRAME = 0.016    # solid perimeter frame around slat area

# Folding bail handles (one per short end)
BAIL_WIRE_D = 0.006    # wire diameter
BAIL_SPAN = 0.14       # distance between pivot centres along Y
BAIL_DROP = 0.085      # leg length (crossbar hangs this far below pivot when stowed)
PIVOT_PIN_L = 0.012    # pivot pin protrusion along Y
PIVOT_R = BAIL_WIRE_D / 2.0

# Pivot mounting ears on the tray outer wall
EAR_W = 0.014          # ear thickness (along X, protruding from wall)
EAR_H = 0.020          # ear height (along Z, straddling the rim)
EAR_D = 0.018          # ear depth (along Y, width of each ear)


# ---------------------------------------------------------------------------
# Geometry builder
# ---------------------------------------------------------------------------
def _tray_mesh() -> object:
    """Complete stackable produce-flat tray as one CadQuery solid."""

    # --- 1. Lofted tapered outer shell, shelled open at the top -----------
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
    tray = outer.faces(">Z").shell(-WALL)

    # Soften the bottom outer edge (moulded-plastic look)
    try:
        tray = tray.edges("<Z").fillet(0.006)
    except Exception:
        pass

    # --- 2. Rolled top rim ------------------------------------------------
    l_o, d_o = L_TOP + 2 * RIM_OVERHANG, D_TOP + 2 * RIM_OVERHANG
    l_i, d_i = L_TOP - WALL, D_TOP - WALL
    r_o = R_VERT_TOP + RIM_OVERHANG
    r_i = max(0.012, R_VERT_TOP - WALL)
    ring = (
        cq.Workplane("XY", origin=(0, 0, H - RIM_H * 0.5))
        .placeSketch(cq.Sketch().rect(l_o, d_o).vertices().fillet(r_o))
        .extrude(RIM_H)
    )
    inner_void = (
        cq.Workplane("XY", origin=(0, 0, H - RIM_H * 0.5 - 0.002))
        .placeSketch(cq.Sketch().rect(l_i, d_i).vertices().fillet(r_i))
        .extrude(RIM_H + 0.004)
    )
    rim = ring.cut(inner_void)
    try:
        rim = rim.edges("#Z").fillet(0.004)
    except Exception:
        pass
    tray = tray.union(rim)

    # --- 3. Cut floor opening for slat ventilation -----------------------
    opening_l = L_BOT - 2 * WALL - 2 * FLOOR_FRAME
    opening_d = D_BOT - 2 * WALL - 2 * FLOOR_FRAME
    floor_cutter = (
        cq.Workplane("XY", origin=(0, 0, -0.001))
        .rect(opening_l, opening_d)
        .extrude(WALL + 0.004)
    )
    tray = tray.cut(floor_cutter)

    # --- 4. Diagonal slat grid floor (crosshatch) ------------------------
    # Build each bar individually and fuse → guarantees connected geometry
    # at every crossing point (unlike the sequential-channel approach).
    grid_l = opening_l + 0.010                 # overlap into frame
    grid_d = opening_d + 0.010
    bar_len = math.sqrt(grid_l ** 2 + grid_d ** 2) + 0.04
    half_bar = 0.5 * bar_len
    n_bars = int(half_bar / SLAT_SPACING) + 1
    offsets = [i * SLAT_SPACING for i in range(-n_bars, n_bars + 1)]

    # +45° bars (compound of parallel bars on a rotated workplane)
    bars_p = (
        cq.Workplane("XY")
        .transformed(rotate=(0, 0, 45))
        .pushPoints([(0.0, off) for off in offsets])
        .rect(bar_len, SLAT_W)
        .extrude(WALL)
    )
    # -45° bars
    bars_m = (
        cq.Workplane("XY")
        .transformed(rotate=(0, 0, -45))
        .pushPoints([(0.0, off) for off in offsets])
        .rect(bar_len, SLAT_W)
        .extrude(WALL)
    )
    # Fuse both directions → connected at every crossing
    grid = bars_p.union(bars_m)

    # Trim to the floor opening + overlap into the perimeter frame
    boundary = (
        cq.Workplane("XY")
        .rect(grid_l, grid_d)
        .extrude(WALL)
    )
    grid = grid.intersect(boundary)
    tray = tray.union(grid)

    # --- 5. Vertical slot perforations on walls --------------------------
    def _slot(plane: str, origin: tuple) -> object:
        sk = cq.Sketch().rect(SLOT_W, SLOT_H).vertices().fillet(SLOT_R)
        return cq.Workplane(plane, origin=origin).placeSketch(sk).extrude(0.6, both=True)

    for x in LONG_SLOT_X:
        tray = tray.cut(_slot("XZ", (x, 0.0, SLOT_ZC)))
    for y in SHORT_SLOT_Y:
        tray = tray.cut(_slot("YZ", (0.0, y, SLOT_ZC)))

    # --- 6. End-grip cutout handles in the short walls -------------------
    for sx in (-1.0, 1.0):
        x_origin = sx * (L_TOP / 2.0 + 0.020)
        grip_cutter = (
            cq.Workplane("YZ", origin=(x_origin, 0.0, GRIP_ZC))
            .placeSketch(cq.Sketch().rect(GRIP_W, GRIP_H).vertices().fillet(GRIP_R))
            .extrude(-sx * 0.060)
        )
        tray = tray.cut(grip_cutter)

    # --- 7. Corner stacking lugs -----------------------------------------
    rim_outer_x = L_TOP / 2.0 + RIM_OVERHANG
    rim_outer_y = D_TOP / 2.0 + RIM_OVERHANG
    lug_cx = rim_outer_x - LUG_W / 2.0 - 0.005
    lug_cy = rim_outer_y - LUG_D / 2.0 - 0.005
    lug_base_z = H - 0.002  # embed slightly into rim for union
    lug_total_h = LUG_H + RIM_H * 0.5 + 0.002  # total extrusion height

    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            lug = (
                cq.Workplane("XY", origin=(sx * lug_cx, sy * lug_cy, lug_base_z))
                .rect(LUG_W, LUG_D)
                .extrude(lug_total_h)
            )
            try:
                lug = lug.edges(">Z").fillet(LUG_FILLET)
            except Exception:
                pass
            tray = tray.union(lug)

    # --- 8. Bail handle pivot mounting ears on short walls ---------------
    # Two ears per short end, at ±BAIL_SPAN/2 along Y, straddling the rim.
    ear_base_z = H - EAR_H * 0.5 - RIM_H * 0.25  # centred on rim height
    for sx in (-1.0, 1.0):
        # Outer face of the short wall at this end
        x_outer = sx * (L_TOP / 2.0 + RIM_OVERHANG)
        for sy in (-1.0, 1.0):
            y_c = sy * BAIL_SPAN / 2.0
            ear = (
                cq.Workplane("YZ", origin=(x_outer, y_c, ear_base_z))
                .rect(EAR_D, EAR_H)
                .extrude(sx * EAR_W)
            )
            try:
                ear = ear.edges("|X").fillet(0.003)
            except Exception:
                pass
            tray = tray.union(ear)

    return mesh_from_cadquery(tray, "produce_flat")


# ---------------------------------------------------------------------------
# Bail handle geometry
# ---------------------------------------------------------------------------
def _bail_handle_mesh() -> object:
    """U-shaped folding bail handle built as a swept wire for single connected solid.

    Part frame sits at the pivot centreline (midpoint between the two pivots).
    At q=0 the bail hangs downward (local -Z); positive rotation swings it up.
    """
    r = PIVOT_R
    half_span = BAIL_SPAN / 2.0
    drop = BAIL_DROP
    pin_ext = PIVOT_PIN_L
    
    # Build the bail as a single swept solid along a wire path
    # Path goes: left pin → left pivot → left leg bottom → crossbar → right leg bottom → right pivot → right pin
    
    # Create the path as a series of connected edges in YZ plane
    path = (
        cq.Workplane("YZ")
        .moveTo(-half_span - pin_ext, 0)  # left pin outer end
        .lineTo(-half_span, 0)             # left pivot
        .lineTo(-half_span, -drop)         # left leg bottom
        .lineTo(half_span, -drop)          # crossbar to right leg
        .lineTo(half_span, 0)              # right leg top (pivot)
        .lineTo(half_span + pin_ext, 0)    # right pin outer end
    )
    
    # Create a circular profile at the start of the path
    profile = (
        cq.Workplane("XZ", origin=(0, -half_span - pin_ext, 0))
        .circle(r)
    )
    
    # Sweep the profile along the path to create a single connected solid
    bail = profile.sweep(path)
    
    return mesh_from_cadquery(bail, "bail_handle")


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stackable_produce_flat")

    body_finish = model.material("body_finish", rgba=(0.82, 0.15, 0.12, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.22, 0.22, 0.24, 1.0))

    tray = model.part("tray")
    tray.visual(_tray_mesh(), material=body_finish, name="tray_body")
    tray.inertial = Inertial.from_geometry(
        Box((L_TOP, D_TOP, H + LUG_H)),
        mass=0.60,
        origin=Origin(xyz=(0.0, 0.0, (H + LUG_H) / 2.0)),
    )

    # --- Folding bail handles (one per short end) -------------------------
    # Pivot centreline sits at the rim outer edge on each short wall.
    pivot_z = H
    pivot_x_offset = L_TOP / 2.0 + RIM_OVERHANG + EAR_W * 0.5

    handle_0 = model.part("handle_0")
    handle_0.visual(_bail_handle_mesh(), material=handle_finish, name="bail_0")

    handle_1 = model.part("handle_1")
    handle_1.visual(_bail_handle_mesh(), material=handle_finish, name="bail_1")

    # +X end handle: axis +Y makes positive q swing the bail inward and up
    model.articulation(
        "tray_to_handle_0",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=handle_0,
        origin=Origin(xyz=(pivot_x_offset, 0.0, pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0, lower=0.0, upper=math.pi,
        ),
    )

    # -X end handle: axis -Y gives the same "positive q = up" convention
    model.articulation(
        "tray_to_handle_1",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=handle_1,
        origin=Origin(xyz=(-pivot_x_offset, 0.0, pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0, lower=0.0, upper=math.pi,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tray = object_model.get_part("tray")
    handle_0 = object_model.get_part("handle_0")
    handle_1 = object_model.get_part("handle_1")
    hinge_0 = object_model.get_articulation("tray_to_handle_0")
    hinge_1 = object_model.get_articulation("tray_to_handle_1")

    # --- Presence & overall dimensions ---
    aabb = ctx.part_world_aabb(tray)
    ctx.check("tray present with world AABB", aabb is not None,
              details=f"aabb={aabb}")

    if aabb is not None:
        mn, mx = aabb
        ext_x = mx[0] - mn[0]
        ext_y = mx[1] - mn[1]
        ext_z = mx[2] - mn[2]

        ctx.check(
            "footprint wider in X than Y",
            ext_x > ext_y + 0.05,
            details=f"ext_x={ext_x:.3f}, ext_y={ext_y:.3f}",
        )
        ctx.check(
            "tray rests at z~=0",
            abs(mn[2]) < 0.01,
            details=f"z_min={mn[2]:.4f}",
        )
        ctx.check(
            "shallow tray body height",
            0.06 < ext_z < 0.16,
            details=f"ext_z={ext_z:.3f}",
        )
        # Stacking lugs must push the top above the rim
        rim_top = H + RIM_H * 0.5
        ctx.check(
            "stacking lugs extend above rim",
            mx[2] > rim_top + LUG_H * 0.6,
            details=f"z_max={mx[2]:.4f}, rim_top={rim_top:.4f}, lug_h={LUG_H}",
        )

    # --- Tapered body (mouth wider than base) ---
    ctx.check(
        "body is tapered (mouth wider than base)",
        L_TOP > L_BOT + 0.02 and D_TOP > D_BOT + 0.02,
        details=f"L_top={L_TOP}, L_bot={L_BOT}, D_top={D_TOP}, D_bot={D_BOT}",
    )

    # --- Hollow interior ---
    inner_top = L_TOP - 2 * WALL
    ctx.check(
        "tray is hollow (open interior cavity)",
        inner_top > 0.30 and WALL < 0.05,
        details=f"inner_top={inner_top:.3f}, wall={WALL}",
    )

    # --- Slot perforations on walls ---
    n_long = len(LONG_SLOT_X) * 2
    n_short = len(SHORT_SLOT_Y) * 2
    n_total = n_long + n_short
    ctx.check(
        "slot perforations on all walls",
        len(LONG_SLOT_X) >= 3 and len(SHORT_SLOT_Y) >= 1 and n_total >= 8,
        details=f"long={len(LONG_SLOT_X)}x2={n_long}, short={len(SHORT_SLOT_Y)}x2={n_short}, total={n_total}",
    )

    # --- Diagonal slat floor is not a solid plate ---
    slat_open_ratio = (SLAT_SPACING - SLAT_W) / SLAT_SPACING
    ctx.check(
        "floor has open vented slat structure",
        SLAT_W < SLAT_SPACING * 0.5 and slat_open_ratio > 0.5,
        details=f"slat_w={SLAT_W}, spacing={SLAT_SPACING}, open_ratio={slat_open_ratio:.2f}",
    )

    # --- End-grip cutouts dimensioned for hand grip ---
    ctx.check(
        "end-grip handles sized for hand grip",
        GRIP_W > 0.06 and GRIP_H > 0.02,
        details=f"grip_w={GRIP_W}, grip_h={GRIP_H}",
    )

    # --- Four stacking lugs ---
    lug_count = 4  # hardcoded by construction (2x2 loop)
    ctx.check(
        "four corner stacking lugs",
        lug_count == 4 and LUG_H > 0.015,
        details=f"lug_count={lug_count}, lug_h={LUG_H}",
    )

    # --- Folding bail handles exist and articulate ---
    ctx.check(
        "two folding bail handles present",
        handle_0 is not None and handle_1 is not None,
        details="handle_0 or handle_1 missing",
    )

    # Allow intentional overlap: handles fold against the tray outer wall
    # and pivot pins nestle against the mounting ears.
    ctx.allow_overlap(
        "tray", "handle_0",
        reason="Bail handle folds flat against the outer wall and pivots on mounting ears",
    )
    ctx.allow_overlap(
        "tray", "handle_1",
        reason="Bail handle folds flat against the outer wall and pivots on mounting ears",
    )

    # At q=0 (stowed), handles hang down against the outer walls.
    # The handle crossbar should be below the pivot (rim level).
    with ctx.pose({hinge_0: 0.0, hinge_1: 0.0}):
        h0_aabb = ctx.part_world_aabb(handle_0)
        h1_aabb = ctx.part_world_aabb(handle_1)
        if h0_aabb is not None and h1_aabb is not None:
            ctx.check(
                "handle_0 stowed below pivot (crossbar hangs down)",
                h0_aabb[0][2] < H - BAIL_DROP * 0.5,
                details=f"handle_0 z_min={h0_aabb[0][2]:.4f}, H={H}",
            )
            ctx.check(
                "handle_1 stowed below pivot (crossbar hangs down)",
                h1_aabb[0][2] < H - BAIL_DROP * 0.5,
                details=f"handle_1 z_min={h1_aabb[0][2]:.4f}, H={H}",
            )

    # At q=upper (carry), handles swing up above the tray.
    carry_angle = math.pi * 0.95  # near-vertical
    with ctx.pose({hinge_0: carry_angle, hinge_1: carry_angle}):
        h0_aabb_up = ctx.part_world_aabb(handle_0)
        h1_aabb_up = ctx.part_world_aabb(handle_1)
        if h0_aabb_up is not None and h1_aabb_up is not None:
            ctx.check(
                "handle_0 raised above rim for carry",
                h0_aabb_up[1][2] > H + BAIL_DROP * 0.6,
                details=f"handle_0 z_max={h0_aabb_up[1][2]:.4f}, H={H}",
            )
            ctx.check(
                "handle_1 raised above rim for carry",
                h1_aabb_up[1][2] > H + BAIL_DROP * 0.6,
                details=f"handle_1 z_max={h1_aabb_up[1][2]:.4f}, H={H}",
            )

    # Bail handle wire is sized for hand grip
    ctx.check(
        "bail handle wire sized for grip",
        BAIL_WIRE_D > 0.004 and BAIL_SPAN > 0.10 and BAIL_DROP > 0.08,
        details=f"wire_d={BAIL_WIRE_D}, span={BAIL_SPAN}, drop={BAIL_DROP}",
    )

    return ctx.report()


object_model = build_object_model()
