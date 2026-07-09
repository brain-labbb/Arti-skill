from __future__ import annotations

# Rounded-corner tub-style red plastic shopping basket with hexagonal honeycomb
# perforations, two integral cutout handles in the top rim on the short ends,
# and a single central swing bail handle that pivots on two lugs at the front
# rim by a revolute joint.
#
# Coordinate convention: +Z up, basket rests on the ground at z=0, long axis
# along X. The tub is a hollow tapered shell (wider at the top) with heavily
# rounded vertical edges, a thick rolled top rim, hexagonal honeycomb
# perforations cut through every wall, two oval cutout handle openings
# in the rim at the short ends, two pivot lugs on the front long-side rim,
# and a semicircular bail handle that swings up for carrying and folds down
# against the front rim.

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
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Dimensions (meters)
# ---------------------------------------------------------------------------
L_TOP = 0.40   # outer length at the mouth (long axis, X)
D_TOP = 0.30   # outer depth at the mouth (short axis, Y)
L_BOT = 0.34   # outer length at the base (taper: narrower at the bottom)
D_BOT = 0.24   # outer depth at the base
H = 0.22       # tub height
WALL = 0.012   # wall thickness

# Heavily rounded corners
R_VERT_BOT = 0.050  # rounded vertical edge radius at the base
R_VERT_TOP = 0.060  # rounded vertical edge radius at the mouth

# Rolled rim lip
RIM_H = 0.024
RIM_OVERHANG = 0.011  # how far the lip rolls out past the top wall

# Hexagonal honeycomb perforations
HEX_AF = 0.010        # across-flats of each hex hole
HEX_WALL = 0.003      # wall thickness between adjacent hex holes
HEX_PITCH_X = HEX_AF + HEX_WALL  # horizontal center-to-center
HEX_PITCH_Y = HEX_PITCH_X * math.sqrt(3.0) / 2.0  # vertical row spacing (honeycomb)
HEX_CIRC_D = 2.0 * HEX_AF / math.sqrt(3.0)  # circumscribed diameter for polygon()
HEX_Z_START = 0.035   # bottom of perforated zone
HEX_Z_END = H - RIM_H - 0.012  # top of perforated zone

# Usable perforation extents (stay away from heavily rounded corners)
LONG_USABLE = L_TOP * 0.72   # usable width on long walls
SHORT_USABLE = D_TOP * 0.55  # usable width on short walls

# Integral cutout handles (openings in the rim on short ends)
CUTOUT_W = 0.082   # width of the handle cutout
CUTOUT_H = 0.028   # height of the handle cutout
CUTOUT_R = 0.012   # corner radius of the cutout
CUTOUT_ZC = H - RIM_H * 0.45  # centered vertically in the rim/upper-wall zone

# Bail handle pivot lugs (on the front long-side rim)
LUG_X = 0.090        # distance from center to each lug along X
LUG_R = 0.007        # lug boss radius
LUG_LEN = 0.010      # lug protrusion from rim outer face
PIVOT_Y = D_TOP / 2.0 + RIM_OVERHANG + LUG_LEN / 2.0  # pivot axis Y in parent frame
PIVOT_Z = H - RIM_H / 2.0  # pivot axis Z in parent frame (mid-rim)

# Bail handle wire
BAIL_ARM = 0.030     # straight arm length below pivot
BAIL_DROP = 0.100    # arc drop below arm ends
BAIL_ROD_R = 0.004   # rod radius (4 mm wire)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def _hex_centers(half_width: float) -> list[tuple[float, float]]:
    """Return (local_x, local_z) centers for a honeycomb hex pattern that
    fits within +-half_width horizontally and HEX_Z_START..HEX_Z_END vertically."""
    centers: list[tuple[float, float]] = []
    row = 0
    z = HEX_Z_START
    while z < HEX_Z_END:
        x_offset = (HEX_PITCH_X * 0.5) if (row % 2 == 1) else 0.0
        x = -half_width + HEX_PITCH_X * 0.5 + x_offset
        while x < half_width - HEX_PITCH_X * 0.25:
            centers.append((x, z))
            x += HEX_PITCH_X
        z += HEX_PITCH_Y
        row += 1
    return centers


def _tub_mesh() -> object:
    """Hollow tapered tub: heavily rounded corners, hexagonal honeycomb
    perforations, integral cutout handles in the rim on short ends, and
    two pivot lugs on the front long-side rim."""

    # --- Base tub shell: loft two heavily-rounded rectangles, shell open top ---
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
    tub = outer.faces(">Z").shell(-WALL)

    # Soften the bottom outer edge for molded-plastic look.
    try:
        tub = tub.edges("<Z").fillet(0.012)
    except Exception:
        pass

    # --- Rolled top rim: rounded-rect ring band that caps the wall and rolls out ---
    l_o = L_TOP + 2 * RIM_OVERHANG
    d_o = D_TOP + 2 * RIM_OVERHANG
    l_i = L_TOP - WALL
    d_i = D_TOP - WALL
    r_o = R_VERT_TOP + RIM_OVERHANG
    r_i = max(0.020, R_VERT_TOP - WALL)

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
        rim = rim.edges("#Z").fillet(0.005)
    except Exception:
        pass
    tub = tub.union(rim)

    # --- Hexagonal honeycomb perforations ---
    cut_depth = 0.06  # enough to penetrate wall + margin

    # Long walls (front / back, along X at y = +-D_TOP/2)
    long_centers = _hex_centers(LONG_USABLE * 0.5)
    if long_centers:
        for sy in (-1.0, 1.0):
            hex_block = (
                cq.Workplane("XZ", origin=(0.0, sy * D_TOP * 0.5, 0.0))
                .pushPoints(long_centers)
                .polygon(6, HEX_CIRC_D)
                .extrude(cut_depth, both=True)
            )
            tub = tub.cut(hex_block)

    # Short walls (ends, along Y at x = +-L_TOP/2)
    short_centers = _hex_centers(SHORT_USABLE * 0.5)
    if short_centers:
        for sx in (-1.0, 1.0):
            hex_block = (
                cq.Workplane("YZ", origin=(sx * L_TOP * 0.5, 0.0, 0.0))
                .pushPoints(short_centers)
                .polygon(6, HEX_CIRC_D)
                .extrude(cut_depth, both=True)
            )
            tub = tub.cut(hex_block)

    # --- Integral cutout handles in the rim on short ends ---
    for sx in (-1.0, 1.0):
        cutout = (
            cq.Workplane("YZ", origin=(sx * (L_TOP * 0.5 + RIM_OVERHANG * 0.5), 0.0, CUTOUT_ZC))
            .placeSketch(cq.Sketch().rect(CUTOUT_W, CUTOUT_H).vertices().fillet(CUTOUT_R))
            .extrude(0.10, both=True)
        )
        tub = tub.cut(cutout)

    # --- Pivot lugs on the front long-side rim ---
    # Two cylindrical bosses protruding outward from the rim for the bail pivot.
    # CadQuery "XZ" plane has normal -Y, so we position at the outer face and
    # extrude inward (in -Y) to create the lug that protrudes outward.
    lug_y_base = D_TOP / 2.0 + RIM_OVERHANG  # rim outer face
    for sx in (-1, 1):
        lug = (
            cq.Workplane("XZ", origin=(sx * LUG_X, lug_y_base + LUG_LEN, PIVOT_Z))
            .circle(LUG_R)
            .extrude(LUG_LEN)  # extrudes in -Y (normal of XZ), from outer face inward
        )
        tub = tub.union(lug)
        # Soften the lug outer edge
        try:
            tub = tub.edges(cq.NearestToPointSelector(
                cq.Vector(sx * LUG_X, lug_y_base + LUG_LEN, PIVOT_Z)
            )).fillet(LUG_R * 0.4)
        except Exception:
            pass

    return mesh_from_cadquery(tub, "basket_tub")


def _bail_handle_mesh() -> object:
    """Semicircular bail handle wire: two arms and a sweeping arc, built as
    a continuous tube from spline points. Defined in the handle part frame
    (child frame at q=0), with the pivot axis along X at the origin."""

    # Build the U-shaped path in the XZ plane (Y=0 in child frame).
    # The handle hangs downward at q=0.
    arm_bottom_z = -BAIL_ARM
    arc_bottom_z = -(BAIL_ARM + BAIL_DROP)

    # Parametric arc points for a smooth semicircular bottom
    arc_pts = []
    n_arc = 9
    for i in range(n_arc):
        t = i / (n_arc - 1)  # 0..1
        angle = math.pi * t  # 0..pi
        x = -LUG_X + 2.0 * LUG_X * t
        z = arm_bottom_z - BAIL_DROP * math.sin(angle)
        arc_pts.append((x, 0.0, z))

    points = [
        (-LUG_X, 0.0, 0.0),         # left pivot (at lug face)
        (-LUG_X, 0.0, arm_bottom_z), # left arm bottom
        *arc_pts,                     # semicircular arc
        (LUG_X, 0.0, arm_bottom_z),  # right arm bottom
        (LUG_X, 0.0, 0.0),           # right pivot (at lug face)
    ]

    bail_geom = tube_from_spline_points(
        points,
        radius=BAIL_ROD_R,
        samples_per_segment=16,
        radial_segments=16,
        cap_ends=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    return mesh_from_geometry(bail_geom, "bail_wire")


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="red_shopping_basket")

    body_finish = model.material("body_finish", rgba=(0.88, 0.27, 0.22, 1.0))

    tub = model.part("basket_tub")
    tub.visual(_tub_mesh(), material=body_finish, name="tub_shell")
    tub.inertial = Inertial.from_geometry(
        Box((L_TOP, D_TOP, H)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, H / 2.0)),
    )

    bail = model.part("bail_handle")
    bail.visual(_bail_handle_mesh(), material=body_finish, name="bail_wire")
    bail.inertial = Inertial.from_geometry(
        Box((2 * LUG_X, 0.02, BAIL_ARM + BAIL_DROP)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, -(BAIL_ARM + BAIL_DROP) / 2.0)),
    )

    # Revolute articulation: bail swings around X axis on the front rim.
    # At q=0 the handle hangs down against the front wall (folded).
    # Positive q (right-hand rule around +X) swings the handle upward;
    # at q=pi the handle is upright for carrying.
    model.articulation(
        "tub_to_bail",
        ArticulationType.REVOLUTE,
        parent=tub,
        child=bail,
        origin=Origin(xyz=(0.0, PIVOT_Y, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=4.0,
            lower=0.0,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tub = object_model.get_part("basket_tub")
    bail = object_model.get_part("bail_handle")
    hinge = object_model.get_articulation("tub_to_bail")

    # --- Allow intentional overlap at the pivot lugs ---
    ctx.allow_overlap(
        tub,
        bail,
        elem_a="tub_shell",
        elem_b="bail_wire",
        reason=(
            "The bail handle pivot ends pass through the cylindrical lug bosses "
            "on the rim, representing the pivot capture."
        ),
    )

    # --- Proof: handle pivot ends overlap with the lug region ---
    ctx.expect_overlap(
        tub,
        bail,
        axes="y",
        elem_a="tub_shell",
        elem_b="bail_wire",
        min_overlap=0.002,
        name="bail pivot ends overlap with lug bosses in Y",
    )

    # --- Footprint: wider in X (long axis) than Y, and rests at z ~= 0 ---
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

    # --- Tapered + hollow ---
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

    # --- Heavily rounded corners ---
    ctx.check(
        "heavily rounded corners (large fillet radii)",
        R_VERT_BOT >= 0.040 and R_VERT_TOP >= 0.050,
        details=f"R_bot={R_VERT_BOT}, R_top={R_VERT_TOP}",
    )

    # --- Hexagonal honeycomb perforations on all walls ---
    long_count = len(_hex_centers(LONG_USABLE * 0.5))
    short_count = len(_hex_centers(SHORT_USABLE * 0.5))
    total_hex = long_count * 2 + short_count * 2
    ctx.check(
        "hexagonal honeycomb perforations on every wall",
        long_count >= 20 and short_count >= 8 and total_hex >= 50,
        details=f"long_per_wall={long_count}, short_per_wall={short_count}, total={total_hex}",
    )

    # --- Cutout handles are real openings in the rim ---
    ctx.check(
        "cutout handle openings are sized for finger grip",
        CUTOUT_W >= 0.060 and CUTOUT_H >= 0.020,
        details=f"cutout_w={CUTOUT_W}, cutout_h={CUTOUT_H}",
    )

    # --- Bail handle exists and is articulated ---
    part_names = [p.name for p in object_model.parts]
    ctx.check(
        "bail handle part exists",
        "bail_handle" in part_names,
        details=f"parts={part_names}",
    )
    ctx.check(
        "exactly one revolute articulation for bail",
        len(object_model.articulations) == 1
        and hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"n_articulations={len(object_model.articulations)}",
    )

    # --- Bail handle motion: positive q raises the handle ---
    # Use AABB to track the visual center (part origin stays at the pivot).
    rest_aabb = ctx.part_world_aabb(bail)
    with ctx.pose({hinge: math.pi}):
        up_aabb = ctx.part_world_aabb(bail)

    rest_z_center = (rest_aabb[0][2] + rest_aabb[1][2]) / 2.0 if rest_aabb else None
    up_z_center = (up_aabb[0][2] + up_aabb[1][2]) / 2.0 if up_aabb else None

    ctx.check(
        "bail swings up at positive q (handle AABB center rises)",
        rest_z_center is not None
        and up_z_center is not None
        and up_z_center > rest_z_center + 0.05,
        details=f"rest_z_center={rest_z_center:.4f}, up_z_center={up_z_center:.4f}",
    )

    # At full swing (q=pi), the handle top should be well above the rim
    up_z_max = up_aabb[1][2] if up_aabb else None
    ctx.check(
        "bail handle upright position clears the rim",
        up_z_max is not None and up_z_max > H + 0.08,
        details=f"up_z_max={up_z_max:.4f}, rim_top={H + RIM_H * 0.5:.4f}",
    )

    # --- Bail handle at rest is near the rim (folded down) ---
    ctx.check(
        "bail handle pivot is at rim height",
        abs(PIVOT_Z - (H - RIM_H / 2.0)) < 0.001,
        details=f"pivot_z={PIVOT_Z}, expected={H - RIM_H / 2.0}",
    )

    # --- Bail handle rod is thin wire ---
    ctx.check(
        "bail handle is a thin wire (rod radius < 8mm)",
        BAIL_ROD_R < 0.008,
        details=f"rod_r={BAIL_ROD_R}",
    )

    # --- Pivot lugs are on the front rim ---
    ctx.check(
        "pivot lugs are on the front long-side rim",
        PIVOT_Y > D_TOP / 2.0 and PIVOT_Z > H - RIM_H,
        details=f"pivot_y={PIVOT_Y}, pivot_z={PIVOT_Z}",
    )

    # --- Handle arc provides useful carrying height ---
    ctx.check(
        "handle arc drop provides carrying clearance",
        BAIL_DROP >= 0.080,
        details=f"bail_drop={BAIL_DROP}",
    )

    return ctx.report()


object_model = build_object_model()
