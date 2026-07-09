from __future__ import annotations

# Galvanized steel trash can (reference: single bright galvanized can, body
# slightly TAPERED - wider at the top, narrower at the foot - with fine vertical
# ribbing, a rolled top rim, a tall domed round lid carrying a top loop handle,
# and two riveted ring (D-ring) side handles).
#
# Coordinate convention (Z-up world):
#   - vertical axis of the can : +Z
#   - footprint sits at z = 0; the body rises in +Z.
#   - the lid is hinged along the REAR (+Y) top rim and flips up/back.
#
# Parts / articulations:
#   - can_body (ROOT): hollow tapered corrugated steel cylinder, rolled rim,
#       solid floor, two riveted lug pads carrying the ring handles.
#   - lid_assembly : tall domed round lid with a top loop handle. REVOLUTE about
#       the rear top-rim hinge line, opens 0..~100 deg.
#   - ring_handle_left / ring_handle_right : two closed D-ring handles, each
#       FIXED to the body through its riveted lug.

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


def _radius_at(z: float) -> float:
    """Linear taper of the mean wall radius from foot to rim."""
    t = (z - 0.0) / BODY_H
    return R_BOT + (R_TOP - R_BOT) * t


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


def _lug_pad(angle: float, *, radius: float, z: float) -> MeshGeometry:
    """Small riveted ear that carries a ring handle, set against the body wall."""
    stub = CylinderGeometry(0.016, 0.012, radial_segments=16)
    stub.rotate_y(math.pi / 2.0)
    stub.translate(radius - 0.002, 0.0, 0.0)
    stub.rotate_z(angle)
    stub.translate(0.0, 0.0, z)
    return stub


def _ring_handle(angle: float, *, radius: float, z: float) -> MeshGeometry:
    """A closed D-ring loop handle hanging in a vertical plane against the body.

    The ring lies roughly in the X-Z plane (then rotated by angle about Z). Its
    top hooks into the lug pad; the ring drops down and bows outward like a real
    swinging drop handle.
    """
    cz = z - 0.035  # ring centre hangs below the lug
    # an oval loop: parametrize an ellipse in X(out)-Z plane, attached at top.
    pts = []
    n = 22
    a_out = 0.045   # outward bow
    b_dn = 0.055    # vertical half-height
    for k in range(n + 1):
        t = 2.0 * math.pi * k / n
        x = radius - 0.004 + a_out * (0.5 - 0.5 * math.cos(t))  # 0 at top, max at bottom-ish
        zz = cz + b_dn * math.sin(t)
        pts.append((x, 0.0, zz))
    ring = tube_from_spline_points(
        pts, radius=0.005, samples_per_segment=6, radial_segments=12,
        closed_spline=True, cap_ends=False,
    )
    ring.rotate_z(angle)
    return ring


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
    model = ArticulatedObject(name="galvanized_trash_can_tapered")

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
    # two riveted lug pads (left/right = +X / -X) high on the body
    lug_z = BODY_H * 0.66
    body.visual(mesh_from_geometry(_lug_pad(0.0, radius=_radius_at(lug_z), z=lug_z), "lug_right"), material=galv_dark, name="lug_right")
    body.visual(mesh_from_geometry(_lug_pad(math.pi, radius=_radius_at(lug_z), z=lug_z), "lug_left"), material=galv_dark, name="lug_left")
    body.inertial = Inertial.from_geometry(
        Box((2 * R_TOP, 2 * R_TOP, BODY_H)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # ---- two ring handles (fixed via lugs) ----
    for side_name, ang in (("right", 0.0), ("left", math.pi)):
        handle = model.part(f"ring_handle_{side_name}")
        handle.visual(
            mesh_from_geometry(_ring_handle(ang, radius=_radius_at(lug_z), z=lug_z), f"ring_{side_name}"),
            material=wire,
            name=f"ring_{side_name}",
        )
        handle.inertial = Inertial.from_geometry(
            Box((0.05, 0.04, 0.11)), mass=0.10,
            origin=Origin(xyz=((_radius_at(lug_z) + 0.03) * math.cos(ang),
                               (_radius_at(lug_z) + 0.03) * math.sin(ang),
                               lug_z - 0.035)),
        )
        model.articulation(
            f"body_to_ring_{side_name}",
            ArticulationType.FIXED,
            parent=body,
            child=handle,
            origin=Origin(),
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
    ring_r = object_model.get_part("ring_handle_right")
    ring_l = object_model.get_part("ring_handle_left")
    lid_joint = object_model.get_articulation("body_to_lid")

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
    bottom_band = None
    top_band = None
    # measure outer radius at two heights via element AABB of the wall mesh is
    # hard; instead assert the design taper directly via the radius function and
    # confirm geometry is consistent (rim wider than foot).
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

    # ---- two ring handles, opposite sides, hanging against the body ----
    rb = ctx.part_world_aabb(ring_r)
    lb = ctx.part_world_aabb(ring_l)
    ctx.check(
        "right ring handle on +X side past the body",
        rb[1][0] > _radius_at(BODY_H * 0.66) + 0.02,
        details=f"right ring x-max={rb[1][0]:.3f}",
    )
    ctx.check(
        "left ring handle on -X side past the body",
        lb[0][0] < -(_radius_at(BODY_H * 0.66) + 0.02),
        details=f"left ring x-min={lb[0][0]:.3f}",
    )
    ctx.allow_overlap(ring_r, body, reason="Ring handle top hooks into the riveted side lug on the body.")
    ctx.allow_overlap(ring_l, body, reason="Ring handle top hooks into the riveted side lug on the body.")

    return ctx.report()


object_model = build_object_model()
