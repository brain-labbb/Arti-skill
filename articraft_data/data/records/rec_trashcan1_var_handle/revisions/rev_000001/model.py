from __future__ import annotations

# Galvanized steel trash can (reference: single bright galvanized can, body
# slightly TAPERED - wider at the top, narrower at the foot - with fine vertical
# ribbing, a rolled top rim, a tall domed round lid carrying a top loop handle,
# and a single overhead BAIL handle that swings over the lid).
#
# Coordinate convention (Z-up world):
#   - vertical axis of the can : +Z
#   - footprint sits at z = 0; the body rises in +Z.
#   - the lid is hinged along the REAR (+Y) top rim and flips up/back.
#   - the bail pivots about the X axis through two lug pivots on opposite sides.
#
# Parts / articulations:
#   - can_body (ROOT): hollow tapered corrugated steel cylinder, rolled rim,
#       solid floor, two riveted bail pivot lugs.
#   - lid_assembly : tall domed round lid with a top loop handle. REVOLUTE about
#       the rear top-rim hinge line, opens 0..~100 deg.
#   - bail_handle : semicircular wire bail with two arms and a crossbar.
#       REVOLUTE about the X axis through the two lug pivots, swings from
#       hanging down (q=0) to arching over the lid (q~pi).

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
    DomeGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---- overall dimensions (metres) ----
R_TOP = 0.215           # outer radius at the rim
R_BOT = 0.170           # outer radius at the foot (tapered, narrower)
BODY_H = 0.660          # body height (rim plane), footprint at z=0
WALL = 0.009            # nominal sheet-metal wall thickness
FLOOR_T = 0.014         # floor thickness
RIB_AMP = 0.006         # corrugation amplitude (fine ribbing)
RIB_COUNT = 40          # number of vertical corrugations
RIM_TUBE = 0.012        # rolled-rim tube radius
LID_R = R_TOP + 0.016   # lid overhangs the rim
LID_RISE = 0.085        # tall domed lid height
HINGE_Z = BODY_H        # hinge at the top rim plane
HINGE_Y = R_TOP + 0.004

# ---- bail handle dimensions ----
BAIL_LUG_Z = BODY_H - RIM_TUBE - 0.010   # pivot lugs just below the rolled rim
BAIL_WIRE_R = 0.005                       # bail wire tube radius
BAIL_ARM_LEN = 0.30                       # arm length from pivot to crossbar
BAIL_BOW = 0.04                           # crossbar outward bow


def _radius_at(z: float) -> float:
    """Linear taper of the mean wall radius from foot to rim."""
    t = (z - 0.0) / BODY_H
    return R_BOT + (R_TOP - R_BOT) * t


# Pivot radius: at the lug boss centre so the bail arm pivot end sits inside
# the lug (small intentional overlap for the captured-pin pivot).
_BAIL_R_PIVOT = _radius_at(BAIL_LUG_Z) + 0.010


def _corrugated_tapered_shell(
    *,
    n_ribs: int,
    z0: float,
    z1: float,
    amp: float,
    wall: float,
    z_segments: int = 6,
) -> MeshGeometry:
    """Hollow tapered corrugated cylinder wall between z0 and z1.

    Outer radius = taper(z) + amp*cos(n*theta); inner radius is a smooth taper
    set in by (amp + wall) so the can is genuinely hollow.
    """
    geom = MeshGeometry()
    ang_segs = n_ribs * 4
    zs = [z0 + (z1 - z0) * k / z_segments for k in range(z_segments + 1)]

    outer_rows = []
    inner_rows = []
    for z in zs:
        rbase = _radius_at(z)
        r_in = rbase - amp - wall
        orow = []
        irow = []
        for i in range(ang_segs):
            theta = 2.0 * math.pi * i / ang_segs
            ro = rbase + amp * math.cos(n_ribs * theta)
            orow.append(geom.add_vertex(ro * math.cos(theta), ro * math.sin(theta), z))
            irow.append(geom.add_vertex(r_in * math.cos(theta), r_in * math.sin(theta), z))
        outer_rows.append(orow)
        inner_rows.append(irow)

    def quad(a, b, c, d):
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    for k in range(z_segments):
        lo, hi = outer_rows[k], outer_rows[k + 1]
        for i in range(ang_segs):
            j = (i + 1) % ang_segs
            quad(lo[i], lo[j], hi[j], hi[i])
    for k in range(z_segments):
        lo, hi = inner_rows[k], inner_rows[k + 1]
        for i in range(ang_segs):
            j = (i + 1) % ang_segs
            quad(lo[j], lo[i], hi[i], hi[j])
    lo_o, lo_i = outer_rows[0], inner_rows[0]
    for i in range(ang_segs):
        j = (i + 1) % ang_segs
        quad(lo_o[j], lo_o[i], lo_i[i], lo_i[j])
    hi_o, hi_i = outer_rows[-1], inner_rows[-1]
    for i in range(ang_segs):
        j = (i + 1) % ang_segs
        quad(hi_o[i], hi_o[j], hi_i[j], hi_i[i])
    return geom


def _floor_disc(*, radius: float, z_bottom: float, thickness: float) -> MeshGeometry:
    g = CylinderGeometry(radius, thickness, radial_segments=48)
    g.translate(0.0, 0.0, z_bottom + thickness / 2.0)
    return g


def _rolled_rim(*, radius: float, z: float, tube: float) -> MeshGeometry:
    g = TorusGeometry(radius, tube, radial_segments=20, tubular_segments=72)
    g.translate(0.0, 0.0, z)
    return g


def _bail_side_element(
    i: int,
    *,
    kind: str,
    radius: float,
    z: float,
    arm_len: float,
    wire_r: float,
) -> MeshGeometry:
    """Shared helper for one side of the bail assembly.

    Parameters
    ----------
    i : int
        Side index (0 = +X side, 1 = -X side).
    kind : str
        "lug" for the riveted pivot lug on the body wall,
        "arm" for one straight bail arm tube in the bail local frame.
    radius : float
        Pivot radius from the can axis.
    z : float
        Lug height (used for lugs only; arms are in the bail local frame).
    arm_len : float
        Arm length from pivot to crossbar junction.
    wire_r : float
        Wire tube radius.
    """
    sign = 1 - 2 * i          # i=0 → +1, i=1 → -1
    angle = i * math.pi       # i=0 → 0 rad, i=1 → pi rad

    if kind == "lug":
        # Riveted pivot lug: a single stub cylinder protruding from the body wall
        # with a pivot hole boss. The two cylinders overlap to form one connected piece.
        # Outer bracket plate (wide, thin, against the wall).
        plate = CylinderGeometry(0.018, 0.006, radial_segments=16)
        plate.rotate_y(math.pi / 2.0)
        plate.translate(radius + 0.003, 0.0, 0.0)
        # Pivot boss (narrower, protruding outward) - overlaps with plate.
        boss = CylinderGeometry(0.009, 0.014, radial_segments=12)
        boss.rotate_y(math.pi / 2.0)
        boss.translate(radius + 0.010, 0.0, 0.0)
        plate.merge(boss)
        plate.rotate_z(angle)
        plate.translate(0.0, 0.0, z)
        return plate

    # kind == "arm": straight arm tube in the bail LOCAL frame (origin at pivot
    # axis centre, q=0 → bail hangs down in -Z).
    x = sign * radius
    pts = [
        (x, 0.0, 0.008),          # slightly above pivot (captured pin region)
        (x, 0.0, -arm_len * 0.3),
        (x, 0.0, -arm_len * 0.7),
        (x, 0.0, -arm_len),       # bottom junction with crossbar
    ]
    return tube_from_spline_points(
        pts,
        radius=wire_r,
        samples_per_segment=10,
        radial_segments=12,
        cap_ends=False,
    )


def _bail_crossbar(
    *,
    radius: float,
    arm_len: float,
    bow: float,
    wire_r: float,
) -> MeshGeometry:
    """Curved crossbar connecting the two bail arm bottoms.

    In the bail local frame (q=0 → bail hangs down), the crossbar curves
    outward in -Y so it wraps around the front of the can body. The path is
    designed to stay at distance >= radius from the body axis at all points,
    ensuring it clears the tapered body wall.
    """
    # Parametrize the crossbar as an arc from angle=0 (+X side) to angle=pi
    # (-X side). The radial distance from the axis increases toward the midpoint
    # by `bow`, keeping the crossbar outside the body wall.
    n_pts = 11
    pts = []
    for k in range(n_pts):
        t = k / (n_pts - 1)            # 0..1
        angle = math.pi * t            # 0..pi
        # Radial distance from axis: peaks at midpoint
        r_arc = radius + bow * math.sin(angle)
        x = r_arc * math.cos(angle)
        y = -r_arc * math.sin(angle)   # bows in -Y (away from body)
        # Slight downward dip at midpoint for a natural hanging shape
        z = -arm_len - bow * 0.3 * math.sin(angle)
        pts.append((x, y, z))
    return tube_from_spline_points(
        pts,
        radius=wire_r,
        samples_per_segment=14,
        radial_segments=12,
        cap_ends=False,
    )


def _lid_geometry() -> MeshGeometry:
    """Tall domed round lid in a LOCAL frame whose origin is the rear hinge line
    at the rim plane. Body extends toward -Y (over the can). A real DomeGeometry
    forms the dome; a flat brim seats over the rim; a top wire loop handle."""
    g = MeshGeometry()
    cy = -LID_R + 0.002  # lid centre offset along -Y from the hinge
    # flat brim disc at the rim plane
    brim = CylinderGeometry(LID_R, 0.010, radial_segments=64)
    brim.translate(0.0, cy, 0.005)
    g.merge(brim)
    # tall dome rising above the brim
    dome = DomeGeometry(LID_R * 0.86, radial_segments=48, height_segments=16)
    # DomeGeometry base on z=0; scale its height to LID_RISE and lift onto brim
    dome.scale(1.0, 1.0, LID_RISE / (LID_R * 0.86))
    dome.translate(0.0, cy, 0.009)
    g.merge(dome)
    # downturned rim skirt overlapping the body rim
    skirt = TorusGeometry(LID_R - RIM_TUBE, RIM_TUBE, radial_segments=16, tubular_segments=72)
    skirt.translate(0.0, cy, 0.003)
    g.merge(skirt)
    # central arched wire loop handle on top of the dome
    loop_z = LID_RISE + 0.009
    loop = tube_from_spline_points(
        [
            (-0.050, cy, loop_z - 0.005),
            (-0.034, cy, loop_z + 0.040),
            (0.0, cy, loop_z + 0.052),
            (0.034, cy, loop_z + 0.040),
            (0.050, cy, loop_z - 0.005),
        ],
        radius=0.006,
        samples_per_segment=14,
        radial_segments=12,
        cap_ends=True,
    )
    g.merge(loop)
    return g


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="galvanized_trash_can_bail")

    galv = model.material("galvanized_steel", rgba=(0.62, 0.62, 0.64, 1.0))
    galv_dark = model.material("galvanized_dark", rgba=(0.48, 0.47, 0.46, 1.0))
    wire = model.material("handle_steel", rgba=(0.34, 0.33, 0.32, 1.0))

    # ---- body (root) ----
    body = model.part("can_body")
    body.visual(
        mesh_from_geometry(
            _corrugated_tapered_shell(
                n_ribs=RIB_COUNT, z0=FLOOR_T, z1=BODY_H - RIM_TUBE,
                amp=RIB_AMP, wall=WALL, z_segments=7,
            ),
            "corrugated_wall",
        ),
        material=galv,
        name="corrugated_wall",
    )
    body.visual(
        mesh_from_geometry(_floor_disc(radius=R_BOT - 0.003, z_bottom=0.0, thickness=FLOOR_T), "floor"),
        material=galv_dark,
        name="floor",
    )
    body.visual(
        mesh_from_geometry(_rolled_rim(radius=R_TOP - RIM_TUBE + 0.004, z=BODY_H - RIM_TUBE, tube=RIM_TUBE), "rolled_rim"),
        material=galv,
        name="rolled_rim",
    )

    # Two riveted bail pivot lugs on opposite sides, high on the body.
    # Emitted via a for-i-in-range loop through the shared helper.
    for i in range(2):
        body.visual(
            mesh_from_geometry(
                _bail_side_element(
                    i,
                    kind="lug",
                    radius=_radius_at(BAIL_LUG_Z),
                    z=BAIL_LUG_Z,
                    arm_len=BAIL_ARM_LEN,
                    wire_r=BAIL_WIRE_R,
                ),
                f"bail_lug_{i}",
            ),
            material=galv_dark,
            name=f"bail_lug_{i}",
        )

    body.inertial = Inertial.from_geometry(
        Box((2 * R_TOP, 2 * R_TOP, BODY_H)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # ---- bail handle (revolute, swings from down to over-the-lid) ----
    bail = model.part("bail_handle")

    # Two bail arms emitted via a for-i-in-range loop through the shared helper.
    for i in range(2):
        bail.visual(
            mesh_from_geometry(
                _bail_side_element(
                    i,
                    kind="arm",
                    radius=_BAIL_R_PIVOT,
                    z=BAIL_LUG_Z,
                    arm_len=BAIL_ARM_LEN,
                    wire_r=BAIL_WIRE_R,
                ),
                f"bail_arm_{i}",
            ),
            material=wire,
            name=f"bail_arm_{i}",
        )

    # Crossbar connecting the two arm bottoms, curving outward.
    bail.visual(
        mesh_from_geometry(
            _bail_crossbar(
                radius=_BAIL_R_PIVOT,
                arm_len=BAIL_ARM_LEN,
                bow=BAIL_BOW,
                wire_r=BAIL_WIRE_R,
            ),
            "bail_crossbar",
        ),
        material=wire,
        name="bail_crossbar",
    )

    bail.inertial = Inertial.from_geometry(
        Box((2 * _BAIL_R_PIVOT, 2 * _BAIL_R_PIVOT, BAIL_ARM_LEN + BAIL_BOW)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, -(BAIL_ARM_LEN + BAIL_BOW) / 2.0)),
    )

    model.articulation(
        "body_to_bail",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bail,
        # Joint origin at the centre of the pivot axis (midpoint between lugs).
        origin=Origin(xyz=(0.0, 0.0, BAIL_LUG_Z)),
        # Axis along X (line through both lug pivots).
        # Positive q swings the bail from hanging down (-Z) toward up (+Z).
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=math.radians(170.0),
            effort=4.0,
            velocity=2.5,
        ),
    )

    # ---- domed lid (revolute, rear-hinged flip lid) ----
    lid = model.part("lid_assembly")
    lid.visual(mesh_from_geometry(_lid_geometry(), "lid_dome"), material=galv, name="lid_dome")
    lid.inertial = Inertial.from_geometry(
        Box((2 * LID_R, 2 * LID_R, LID_RISE)),
        mass=0.8,
        origin=Origin(xyz=(0.0, -LID_R, LID_RISE / 2.0)),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=math.radians(100.0), effort=6.0, velocity=2.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("can_body")
    lid = object_model.get_part("lid_assembly")
    bail = object_model.get_part("bail_handle")
    lid_joint = object_model.get_articulation("body_to_lid")
    bail_joint = object_model.get_articulation("body_to_bail")

    # ---- footprint at z ~ 0 ----
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "can base sits at z~0",
        abs(body_aabb[0][2]) < 0.005,
        details=f"body z-min={body_aabb[0][2]:.4f}",
    )
    ext = _ext(body_aabb)
    ctx.check(
        "body roughly cylindrical (x~y) and tall",
        abs(ext[0] - ext[1]) < 0.05 and ext[2] > ext[0] + 0.10,
        details=f"body extents (x,y,z)={ext}",
    )

    # ---- body is tapered: wider at top than at the foot ----
    ctx.check(
        "body is tapered (rim wider than foot)",
        R_TOP > R_BOT + 0.02,
        details=f"R_TOP={R_TOP}, R_BOT={R_BOT}",
    )

    # ---- hollow can: deep interior cavity ----
    ctx.check(
        "deep interior cavity (hollow can)",
        (BODY_H - RIM_TUBE) - FLOOR_T > 0.45,
        details=f"cavity depth ~{(BODY_H - RIM_TUBE) - FLOOR_T:.3f} m",
    )

    # ---- lid closed: seated over the rim ----
    ctx.allow_overlap(
        lid, body,
        reason="Closed lid skirt nests down over the body rolled rim.",
    )
    closed_aabb = ctx.part_world_aabb(lid)
    closed_top = closed_aabb[1][2]
    closed_free_y = closed_aabb[0][1]
    ctx.check(
        "closed lid sits at/above the body rim",
        closed_aabb[0][2] > BODY_H - 0.05,
        details=f"lid z-min closed={closed_aabb[0][2]:.3f}, rim={BODY_H:.3f}",
    )
    ctx.check(
        "closed lid covers the can opening",
        _ext(closed_aabb)[0] > ext[0] - 0.04,
        details=f"lid x-extent={_ext(closed_aabb)[0]:.3f}, body x={ext[0]:.3f}",
    )
    # ---- domed lid is genuinely domed (tall) ----
    ctx.check(
        "lid is domed (significant rise)",
        _ext(closed_aabb)[2] > 0.07,
        details=f"lid z-extent={_ext(closed_aabb)[2]:.3f}",
    )

    # ---- lid flips open about the rear hinge ----
    with ctx.pose({lid_joint: math.radians(100.0)}):
        open_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid free edge lifts when opened",
        open_aabb[1][2] > closed_top + 0.10,
        details=f"closed_top={closed_top:.3f}, open_top={open_aabb[1][2]:.3f}",
    )
    ctx.check(
        "lid swings back over the rear hinge when opened",
        open_aabb[1][1] > closed_free_y + 0.10,
        details=f"closed_free_y={closed_free_y:.3f}, open_free_y={open_aabb[1][1]:.3f}",
    )
    ax = lid_joint.axis
    ctx.check(
        "lid hinge axis is horizontal along the rear rim (X)",
        abs(ax[0]) > 0.99 and abs(ax[1]) < 0.01 and abs(ax[2]) < 0.01,
        details=f"axis={ax}",
    )

    # ---- bail handle: two arms + crossbar, swings over the lid ----
    bail_aabb_rest = ctx.part_world_aabb(bail)
    bail_rest_top = bail_aabb_rest[1][2]
    bail_rest_bottom = bail_aabb_rest[0][2]

    # Bail joint axis is along X (through both lug pivots).
    bax = bail_joint.axis
    ctx.check(
        "bail pivot axis is horizontal along X",
        abs(bax[0]) > 0.99 and abs(bax[1]) < 0.01 and abs(bax[2]) < 0.01,
        details=f"bail axis={bax}",
    )

    # At rest (q=0), bail hangs down: bottom is well below the lug height.
    ctx.check(
        "bail hangs down at rest (bottom below lug height)",
        bail_rest_bottom < BAIL_LUG_Z - BAIL_ARM_LEN * 0.8,
        details=f"bail z-min={bail_rest_bottom:.3f}, lug_z={BAIL_LUG_Z:.3f}",
    )

    # At rest, bail top is at or below the lug height (not arching up).
    ctx.check(
        "bail does not arch above lugs at rest",
        bail_rest_top < BAIL_LUG_Z + 0.05,
        details=f"bail z-max rest={bail_rest_top:.3f}, lug_z={BAIL_LUG_Z:.3f}",
    )

    # Bail arms extend to both sides of the body.
    ctx.check(
        "bail spans both sides of the can (x-extent)",
        bail_aabb_rest[0][0] < -(_radius_at(BAIL_LUG_Z) + 0.01)
        and bail_aabb_rest[1][0] > (_radius_at(BAIL_LUG_Z) + 0.01),
        details=f"bail x=[{bail_aabb_rest[0][0]:.3f}, {bail_aabb_rest[1][0]:.3f}]",
    )

    # Bail has two arm visuals and a crossbar visual.
    bail_visual_names = [v.name for v in bail.visuals]
    ctx.check(
        "bail has two arm visuals",
        "bail_arm_0" in bail_visual_names and "bail_arm_1" in bail_visual_names,
        details=f"bail visuals={bail_visual_names}",
    )
    ctx.check(
        "bail has crossbar visual",
        "bail_crossbar" in bail_visual_names,
        details=f"bail visuals={bail_visual_names}",
    )

    # Body has two bail pivot lugs.
    body_visual_names = [v.name for v in body.visuals]
    ctx.check(
        "body has two bail pivot lugs",
        "bail_lug_0" in body_visual_names and "bail_lug_1" in body_visual_names,
        details=f"body visuals={body_visual_names}",
    )

    # When bail swings up (q ~ pi), the crossbar arches over the lid dome.
    bail_up_angle = math.radians(170.0)
    with ctx.pose({bail_joint: bail_up_angle}):
        bail_aabb_up = ctx.part_world_aabb(bail)
    bail_up_top = bail_aabb_up[1][2]

    ctx.check(
        "bail arches above the lid when swung up",
        bail_up_top > closed_top + 0.05,
        details=f"bail z-max up={bail_up_top:.3f}, lid closed top={closed_top:.3f}",
    )

    # Bail pivot lugs are on opposite sides of the body.
    lug_0_aabb = ctx.part_element_world_aabb(body, elem="bail_lug_0")
    lug_1_aabb = ctx.part_element_world_aabb(body, elem="bail_lug_1")
    ctx.check(
        "bail lug 0 on +X side",
        lug_0_aabb[1][0] > _radius_at(BAIL_LUG_Z) - 0.01,
        details=f"lug_0 x-max={lug_0_aabb[1][0]:.3f}",
    )
    ctx.check(
        "bail lug 1 on -X side",
        lug_1_aabb[0][0] < -(_radius_at(BAIL_LUG_Z) - 0.01),
        details=f"lug_1 x-min={lug_1_aabb[0][0]:.3f}",
    )

    # Allow small overlap between bail arm pivot ends and the body lugs
    # (the arm pivots inside the lug).
    ctx.allow_overlap(
        bail, body,
        elem_a="bail_arm_0",
        elem_b="bail_lug_0",
        reason="Bail arm pivot end is captured inside the pivot lug.",
    )
    ctx.allow_overlap(
        bail, body,
        elem_a="bail_arm_1",
        elem_b="bail_lug_1",
        reason="Bail arm pivot end is captured inside the pivot lug.",
    )

    return ctx.report()


object_model = build_object_model()
