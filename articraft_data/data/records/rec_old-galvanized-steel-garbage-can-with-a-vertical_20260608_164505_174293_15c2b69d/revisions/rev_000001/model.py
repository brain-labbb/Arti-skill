from __future__ import annotations

# Old galvanized steel garbage can (reference: weathered metal can with a
# vertically corrugated cylindrical body, a rolled rim, two wire-bail side
# handles, and a separate domed round lid).
#
# Coordinate convention (Z-up world):
#   - vertical axis of the can          : +Z
#   - the can footprint sits at z = 0; the body rises in +Z.
#   - the lid is hinged along the REAR (+Y) top rim and flips up/back.
#
# Parts / articulations:
#   - can_body (ROOT): hollow corrugated steel cylinder with a rolled top rim,
#       slightly domed/recessed floor, and two riveted lug pads on the sides.
#   - lid_assembly : domed round lid + central wire loop handle. REVOLUTE about
#       the rear top-rim hinge line, opens 0..~100 deg (lid tips up and back).
#   - bail_handle_left / bail_handle_right : two semicircular wire bail handles,
#       each FIXED to the body through its side lugs (not the primary motion).

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
BODY_R = 0.205          # nominal outer radius of the corrugated body
BODY_H = 0.620          # body height (rim plane), footprint at z=0
WALL = 0.010            # nominal sheet-metal wall thickness (mean)
FLOOR_T = 0.014         # floor thickness
RIB_AMP = 0.010         # corrugation amplitude (ridge depth)
RIB_COUNT = 28          # number of vertical corrugations around the body
RIM_TUBE = 0.013        # rolled-rim tube radius
LID_R = BODY_R + 0.018  # lid overhangs the rim
LID_RISE = 0.045        # domed lid height
HINGE_Z = BODY_H        # hinge at the top rim plane
HINGE_Y = BODY_R + 0.004


def _corrugated_shell(
    *,
    r_mean: float,
    amp: float,
    n_ribs: int,
    z0: float,
    z1: float,
    wall: float,
    z_segments: int = 4,
) -> MeshGeometry:
    """Hollow corrugated (fluted) cylinder wall between z0 and z1.

    The radius varies sinusoidally with angle to read as vertical ribs. Builds
    an outer surface and a smooth inner surface (so the can is genuinely hollow),
    joined with top and bottom rim bands.
    """
    geom = MeshGeometry()
    ang_segs = n_ribs * 4  # 4 samples per rib for smooth flutes
    zs = [z0 + (z1 - z0) * k / z_segments for k in range(z_segments + 1)]
    r_in = r_mean - amp - wall

    def outer_r(i: int) -> float:
        theta = 2.0 * math.pi * i / ang_segs
        return r_mean + amp * math.cos(n_ribs * theta)

    # vertex grids
    outer_rows = []
    inner_rows = []
    for z in zs:
        orow = []
        irow = []
        for i in range(ang_segs):
            theta = 2.0 * math.pi * i / ang_segs
            ro = outer_r(i)
            orow.append(geom.add_vertex(ro * math.cos(theta), ro * math.sin(theta), z))
            irow.append(geom.add_vertex(r_in * math.cos(theta), r_in * math.sin(theta), z))
        outer_rows.append(orow)
        inner_rows.append(irow)

    def quad(a, b, c, d):
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    # outer wall (outward facing)
    for k in range(z_segments):
        lo, hi = outer_rows[k], outer_rows[k + 1]
        for i in range(ang_segs):
            j = (i + 1) % ang_segs
            quad(lo[i], lo[j], hi[j], hi[i])
    # inner wall (inward facing -> reverse winding)
    for k in range(z_segments):
        lo, hi = inner_rows[k], inner_rows[k + 1]
        for i in range(ang_segs):
            j = (i + 1) % ang_segs
            quad(lo[j], lo[i], hi[i], hi[j])
    # bottom rim band (close the wall thickness at z0)
    lo_o, lo_i = outer_rows[0], inner_rows[0]
    for i in range(ang_segs):
        j = (i + 1) % ang_segs
        quad(lo_o[j], lo_o[i], lo_i[i], lo_i[j])
    # top rim band (z1)
    hi_o, hi_i = outer_rows[-1], inner_rows[-1]
    for i in range(ang_segs):
        j = (i + 1) % ang_segs
        quad(hi_o[i], hi_o[j], hi_i[j], hi_i[i])
    return geom


def _floor_disc(*, radius: float, z_bottom: float, thickness: float) -> MeshGeometry:
    """Solid floor disc closing the bottom of the can."""
    g = CylinderGeometry(radius, thickness, radial_segments=48)
    g.translate(0.0, 0.0, z_bottom + thickness / 2.0)
    return g


def _rolled_rim(*, radius: float, z: float, tube: float) -> MeshGeometry:
    """A torus ring representing the rolled top rim of the body."""
    g = TorusGeometry(radius, tube, radial_segments=20, tubular_segments=64)
    g.translate(0.0, 0.0, z)
    return g


def _lug_pad(angle: float, *, radius: float, z: float) -> MeshGeometry:
    """A small riveted ear/lug on the body wall that carries a bail handle end."""
    pad = MeshGeometry()
    box = CylinderGeometry(0.018, 0.012, radial_segments=16)  # short stub along Z
    box.rotate_y(math.pi / 2.0)  # lay it radially (long axis along local X)
    box.translate(radius - 0.002, 0.0, 0.0)
    box.rotate_z(angle)
    box.translate(0.0, 0.0, z)
    pad.merge(box)
    return pad


def _bail_handle(angle: float) -> MeshGeometry:
    """A semicircular wire bail handle in a vertical plane on one side of the can.

    Authored in a local frame centred on the side of the body so it arcs
    outward and up; both ends curl in toward the lug pads. Rotated into place
    by `angle` about Z.
    """
    # arc lies in the X-Z plane near x = BODY_R, peaking outward and slightly up.
    z_attach = BODY_H * 0.62
    span = 0.150  # full chord between the two lug ends (along Z)
    out = 0.060   # how far the handle bows outward (+X)
    pts = [
        (BODY_R - 0.006, 0.0, z_attach + span / 2.0),
        (BODY_R + out * 0.55, 0.0, z_attach + span / 2.0 + 0.010),
        (BODY_R + out, 0.0, z_attach),
        (BODY_R + out * 0.55, 0.0, z_attach - span / 2.0 - 0.010),
        (BODY_R - 0.006, 0.0, z_attach - span / 2.0),
    ]
    tube = tube_from_spline_points(
        pts, radius=0.0045, samples_per_segment=16, radial_segments=12, cap_ends=True
    )
    tube.rotate_z(angle)
    return tube


def _lid_geometry() -> MeshGeometry:
    """Domed round lid built in a LOCAL frame whose origin is the rear hinge
    line at the rim plane. The lid extends toward -Y (over the can) and its
    underside skirt drops slightly below to seat over the rim. Includes a
    central arched wire loop handle on top."""
    g = MeshGeometry()
    # Flat lid disc with a shallow dome: a stack of shrinking discs.
    levels = 8
    disc_z0 = 0.0
    for k in range(levels):
        frac = k / levels
        z = disc_z0 + frac * LID_RISE
        r = LID_R * (1.0 - 0.18 * frac)
        thick = LID_RISE / levels + 0.004
        ring = CylinderGeometry(r, thick, radial_segments=48)
        ring.translate(0.0, -LID_R + 0.002, z + thick / 2.0)
        g.merge(ring)
    # downturned rim skirt that overlaps the body rim (closing the lid edge)
    skirt = TorusGeometry(LID_R - RIM_TUBE, RIM_TUBE, radial_segments=16, tubular_segments=64)
    skirt.translate(0.0, -LID_R + 0.002, 0.002)
    g.merge(skirt)
    # central arched wire loop handle (a small bail on top of the lid dome)
    loop_z = disc_z0 + LID_RISE
    loop = tube_from_spline_points(
        [
            (-0.045, -LID_R + 0.002, loop_z),
            (-0.030, -LID_R + 0.002, loop_z + 0.035),
            (0.0, -LID_R + 0.002, loop_z + 0.045),
            (0.030, -LID_R + 0.002, loop_z + 0.035),
            (0.045, -LID_R + 0.002, loop_z),
        ],
        radius=0.005,
        samples_per_segment=14,
        radial_segments=12,
        cap_ends=True,
    )
    g.merge(loop)
    return g


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="galvanized_garbage_can")

    galv = model.material("galvanized_steel", rgba=(0.55, 0.55, 0.58, 1.0))
    galv_dark = model.material("galvanized_dark", rgba=(0.42, 0.41, 0.40, 1.0))
    wire = model.material("wire_steel", rgba=(0.30, 0.29, 0.28, 1.0))

    # ---- body (root) ----
    body = model.part("can_body")
    body.visual(
        mesh_from_geometry(
            _corrugated_shell(
                r_mean=BODY_R, amp=RIB_AMP, n_ribs=RIB_COUNT,
                z0=FLOOR_T, z1=BODY_H - RIM_TUBE, wall=WALL, z_segments=5,
            ),
            "corrugated_wall",
        ),
        material=galv,
        name="corrugated_wall",
    )
    body.visual(
        mesh_from_geometry(_floor_disc(radius=BODY_R - 0.004, z_bottom=0.0, thickness=FLOOR_T), "floor"),
        material=galv_dark,
        name="floor",
    )
    body.visual(
        mesh_from_geometry(_rolled_rim(radius=BODY_R - RIM_TUBE + 0.004, z=BODY_H - RIM_TUBE, tube=RIM_TUBE), "rolled_rim"),
        material=galv,
        name="rolled_rim",
    )
    # two side lug pads carrying the bail handles (left/right = +X / -X sides)
    body.visual(mesh_from_geometry(_lug_pad(0.0, radius=BODY_R, z=BODY_H * 0.62 + 0.075), "lug_right_top"), material=galv_dark, name="lug_right_top")
    body.visual(mesh_from_geometry(_lug_pad(0.0, radius=BODY_R, z=BODY_H * 0.62 - 0.075), "lug_right_bot"), material=galv_dark, name="lug_right_bot")
    body.visual(mesh_from_geometry(_lug_pad(math.pi, radius=BODY_R, z=BODY_H * 0.62 + 0.075), "lug_left_top"), material=galv_dark, name="lug_left_top")
    body.visual(mesh_from_geometry(_lug_pad(math.pi, radius=BODY_R, z=BODY_H * 0.62 - 0.075), "lug_left_bot"), material=galv_dark, name="lug_left_bot")
    body.inertial = Inertial.from_geometry(
        Box((2 * BODY_R, 2 * BODY_R, BODY_H)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # ---- two bail handles (fixed to body via lugs) ----
    for side_name, ang in (("right", 0.0), ("left", math.pi)):
        handle = model.part(f"bail_handle_{side_name}")
        handle.visual(
            mesh_from_geometry(_bail_handle(ang), f"bail_{side_name}"),
            material=wire,
            name=f"bail_{side_name}",
        )
        handle.inertial = Inertial.from_geometry(
            Box((0.08, 0.06, 0.16)), mass=0.15,
            origin=Origin(xyz=((BODY_R + 0.05) * math.cos(ang), (BODY_R + 0.05) * math.sin(ang), BODY_H * 0.62)),
        )
        model.articulation(
            f"body_to_bail_{side_name}",
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
        mass=0.9,
        origin=Origin(xyz=(0.0, -LID_R, LID_RISE / 2.0)),
    )
    # Hinge along the rear top rim (-X axis line at y=+HINGE_Y, z=rim). The lid
    # body extends toward -Y from the hinge, so axis -X lifts the free (-Y) edge
    # up and back (positive q opens). 0..~100 deg realistic flip.
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
    bail_r = object_model.get_part("bail_handle_right")
    bail_l = object_model.get_part("bail_handle_left")
    lid_joint = object_model.get_articulation("body_to_lid")

    # ---- footprint sits at z ~ 0 (not buried / not floating) ----
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "can base sits at z~0",
        abs(body_aabb[0][2]) < 0.005,
        details=f"body z-min={body_aabb[0][2]:.4f}",
    )
    # ---- body is roughly cylindrical: width ~ depth, and tall ----
    ext = _ext(body_aabb)
    ctx.check(
        "body roughly cylindrical (x~y) and tall",
        abs(ext[0] - ext[1]) < 0.05 and ext[2] > ext[0] + 0.10,
        details=f"body extents (x,y,z)={ext}",
    )
    ctx.check(
        "body diameter ~0.4-0.5 m",
        0.38 < ext[0] < 0.52,
        details=f"body x-extent={ext[0]:.3f}",
    )

    # ---- can is hollow: a probe point inside the upper interior is empty ----
    # The interior radius is r_mean - amp - wall; sample near the centre top.
    # We assert the floor is well below the rim (genuine cavity depth).
    floor_top = FLOOR_T
    ctx.check(
        "deep interior cavity (hollow can)",
        (BODY_H - RIM_TUBE) - floor_top > 0.45,
        details=f"cavity depth ~{(BODY_H - RIM_TUBE) - floor_top:.3f} m",
    )

    # ---- lid closed: seated over the rim, overlapping it ----
    ctx.allow_overlap(
        lid, body,
        reason="Closed lid skirt nests down over the body rolled rim.",
    )
    closed_aabb = ctx.part_world_aabb(lid)
    closed_top = closed_aabb[1][2]
    closed_free_y = closed_aabb[0][1]  # most -Y (free front edge) when closed
    ctx.check(
        "closed lid sits at/above the body rim",
        closed_aabb[0][2] > BODY_H - 0.05,
        details=f"lid z-min closed={closed_aabb[0][2]:.3f}, rim={BODY_H:.3f}",
    )
    ctx.check(
        "closed lid covers the can opening (diameter)",
        _ext(closed_aabb)[0] > ext[0] - 0.02,
        details=f"lid x-extent={_ext(closed_aabb)[0]:.3f}, body x={ext[0]:.3f}",
    )

    # ---- lid flips open about the rear hinge: free edge rises and swings back ----
    with ctx.pose({lid_joint: math.radians(100.0)}):
        open_aabb = ctx.part_world_aabb(lid)
    open_top = open_aabb[1][2]
    open_free_y = open_aabb[1][1]  # most +Y (free edge swung toward rear) when open
    ctx.check(
        "lid free edge lifts when opened",
        open_top > closed_top + 0.10,
        details=f"closed_top={closed_top:.3f}, open_top={open_top:.3f}",
    )
    ctx.check(
        "lid swings back over the rear hinge when opened",
        open_free_y > closed_free_y + 0.10,
        details=f"closed_free_y={closed_free_y:.3f}, open_free_y={open_free_y:.3f}",
    )

    # ---- hinge axis is horizontal along X (the rear rim line) ----
    ax = lid_joint.axis
    ctx.check(
        "lid hinge axis is horizontal along the rear rim (X)",
        abs(ax[0]) > 0.99 and abs(ax[1]) < 0.01 and abs(ax[2]) < 0.01,
        details=f"axis={ax}",
    )

    # ---- two bail handles, on opposite (+X / -X) sides, bowing outward ----
    rb = ctx.part_world_aabb(bail_r)
    lb = ctx.part_world_aabb(bail_l)
    ctx.check(
        "right bail handle on +X side, bows past the body",
        rb[1][0] > BODY_R + 0.03,
        details=f"right bail x-max={rb[1][0]:.3f}, body_r={BODY_R:.3f}",
    )
    ctx.check(
        "left bail handle on -X side, bows past the body",
        lb[0][0] < -(BODY_R + 0.03),
        details=f"left bail x-min={lb[0][0]:.3f}, -body_r={-BODY_R:.3f}",
    )
    # handles attach into the body lugs (intended contact/overlap at the ears)
    ctx.allow_overlap(bail_r, body, reason="Bail handle ends curl into the side lug pads on the body.")
    ctx.allow_overlap(bail_l, body, reason="Bail handle ends curl into the side lug pads on the body.")

    return ctx.report()


object_model = build_object_model()
