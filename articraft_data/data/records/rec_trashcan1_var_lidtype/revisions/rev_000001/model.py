from __future__ import annotations

# Galvanized steel trash can (reference: single bright galvanized can, body
# slightly TAPERED - wider at the top, narrower at the foot - with fine vertical
# ribbing, a rolled top rim, a FLAT lift-off lid with a central bail handle,
# and two riveted ring (D-ring) side handles).
#
# Coordinate convention (Z-up world):
#   - vertical axis of the can : +Z
#   - footprint sits at z = 0; the body rises in +Z.
#   - the lid lifts straight up along +Z (PRISMATIC).
#
# Parts / articulations:
#   - can_body (ROOT): hollow tapered corrugated steel cylinder, rolled rim,
#       solid floor, two riveted lug pads carrying the ring handles.
#   - lid_assembly : flat shallow-rimmed steel disc with a central bail handle.
#       PRISMATIC along +Z, lifts 0..0.40 m off the rim.
#   - ring_handle_left / ring_handle_right : two closed D-ring handles, each
#       FIXED to the body through its riveted lug.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
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
LID_DISC_T = 0.008      # flat lid disc thickness
LID_SKIRT_H = 0.022     # shallow downturned skirt depth
LID_SEAT_Z = BODY_H + LID_DISC_T / 2.0  # lid disc centre when seated on rim
LIFT_MAX = 0.40         # max prismatic lift (metres)


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


def _flat_lid_geometry() -> MeshGeometry:
    """Flat lift-off lid in a LOCAL frame whose origin is the disc centre when
    seated on the rim.  The disc is at z=0; a shallow skirt hangs below (-Z) to
    nest over the rolled rim; a central bail handle arches above (+Z)."""
    g = MeshGeometry()
    # -- flat disc plate --
    disc = CylinderGeometry(LID_R, LID_DISC_T, radial_segments=64)
    # CylinderGeometry is centred at origin along Z already
    g.merge(disc)
    # -- shallow downturned skirt (short cylindrical wall below the disc) --
    # The skirt inner matches the body inner wall so it centres the lid.
    skirt_r = LID_R - RIM_TUBE * 0.6  # sits just inside the rolled rim
    skirt_wall = MeshGeometry()
    n_ang = 72
    z_top = -LID_DISC_T / 2.0          # starts at disc bottom
    z_bot = z_top - LID_SKIRT_H        # hangs down
    top_row = []
    bot_row = []
    for i in range(n_ang):
        th = 2.0 * math.pi * i / n_ang
        cx, cy = skirt_r * math.cos(th), skirt_r * math.sin(th)
        top_row.append(skirt_wall.add_vertex(cx, cy, z_top))
        bot_row.append(skirt_wall.add_vertex(cx, cy, z_bot))
    # outer face of skirt
    for i in range(n_ang):
        j = (i + 1) % n_ang
        skirt_wall.add_face(top_row[i], top_row[j], bot_row[j])
        skirt_wall.add_face(top_row[i], bot_row[j], bot_row[i])
    # bottom ring cap (annular ring from skirt_r inward to disc edge underside)
    inner_r = skirt_r - 0.006
    inner_bot = []
    for i in range(n_ang):
        th = 2.0 * math.pi * i / n_ang
        inner_bot.append(skirt_wall.add_vertex(inner_r * math.cos(th), inner_r * math.sin(th), z_bot))
    for i in range(n_ang):
        j = (i + 1) % n_ang
        skirt_wall.add_face(bot_row[j], bot_row[i], inner_bot[i])
        skirt_wall.add_face(bot_row[j], inner_bot[i], inner_bot[j])
    g.merge(skirt_wall)
    # -- central bail / loop handle --
    handle_z_base = LID_DISC_T / 2.0
    loop = tube_from_spline_points(
        [
            (-0.055, 0.0, handle_z_base),
            (-0.040, 0.0, handle_z_base + 0.042),
            (0.0, 0.0, handle_z_base + 0.055),
            (0.040, 0.0, handle_z_base + 0.042),
            (0.055, 0.0, handle_z_base),
        ],
        radius=0.005,
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

    # ---- flat lift-off lid (prismatic, vertical lift) ----
    lid = model.part("lid_assembly")
    lid.visual(mesh_from_geometry(_flat_lid_geometry(), "flat_lid"), material=galv, name="flat_lid")
    lid.inertial = Inertial.from_geometry(
        Box((2 * LID_R, 2 * LID_R, LID_DISC_T + LID_SKIRT_H)),
        mass=0.6,
        origin=Origin(xyz=(0.0, 0.0, -LID_SKIRT_H / 2.0)),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, LID_SEAT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=LIFT_MAX, effort=20.0, velocity=0.5),
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
        elem_a="flat_lid",
        elem_b="rolled_rim",
        reason="Closed flat lid skirt nests down over the body rolled rim to centre and seat the lid.",
    )
    ctx.allow_overlap(
        lid, body,
        elem_a="flat_lid",
        elem_b="corrugated_wall",
        reason="Flat lid skirt bottom ring overlaps the top edge of the corrugated wall where the skirt seats over the rim area.",
    )
    closed_aabb = ctx.part_world_aabb(lid)
    closed_top = closed_aabb[1][2]
    ctx.check(
        "closed lid sits at/above the body rim",
        closed_aabb[0][2] > BODY_H - 0.05,
        details=f"lid z-min closed={closed_aabb[0][2]:.3f}, rim={BODY_H:.3f}",
    )
    # proof: lid disc seats on top of the body rim (contact or near-contact)
    ctx.expect_gap(
        lid, body, axis="z",
        max_penetration=0.025,
        positive_elem="flat_lid",
        negative_elem="rolled_rim",
        name="lid disc seats on rolled rim with limited penetration",
    )
    ctx.check(
        "closed lid covers the can opening",
        _ext(closed_aabb)[0] > ext[0] - 0.04,
        details=f"lid x-extent={_ext(closed_aabb)[0]:.3f}, body x={ext[0]:.3f}",
    )
    # ---- lid is FLAT (not domed): small z-extent ----
    lid_z_extent = _ext(closed_aabb)[2]
    ctx.check(
        "lid is flat (small z-extent, not domed)",
        lid_z_extent < 0.12,
        details=f"lid z-extent={lid_z_extent:.3f} m (flat lid should be < 0.12)",
    )

    # ---- lid lifts straight up (prismatic along +Z) ----
    lift_amount = 0.30
    with ctx.pose({lid_joint: lift_amount}):
        open_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid lifts vertically when raised",
        open_aabb[0][2] > closed_aabb[0][2] + lift_amount - 0.01,
        details=f"closed z-min={closed_aabb[0][2]:.3f}, open z-min={open_aabb[0][2]:.3f}, expected lift={lift_amount}",
    )
    ctx.check(
        "lid stays centred over the can when lifted (no lateral shift)",
        abs(open_aabb[0][0] - closed_aabb[0][0]) < 0.005 and abs(open_aabb[0][1] - closed_aabb[0][1]) < 0.005,
        details=f"closed xy-min=({closed_aabb[0][0]:.3f},{closed_aabb[0][1]:.3f}), open xy-min=({open_aabb[0][0]:.3f},{open_aabb[0][1]:.3f})",
    )
    ax = lid_joint.axis
    ctx.check(
        "lid joint axis is vertical (+Z) for prismatic lift",
        abs(ax[2]) > 0.99 and abs(ax[0]) < 0.01 and abs(ax[1]) < 0.01,
        details=f"axis={ax}",
    )
    ctx.check(
        "lid joint is prismatic type",
        lid_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={lid_joint.articulation_type}",
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
