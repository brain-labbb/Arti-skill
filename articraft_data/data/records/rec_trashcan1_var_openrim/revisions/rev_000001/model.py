from __future__ import annotations

# Galvanized steel trash can — OPEN TOP (no lid).
# Tapered (wider at top) vertically ribbed cylindrical body, rolled rim exposing
# an open mouth, deep hollow interior visible from above, and two riveted
# swinging bail / drop-ring side handles that pivot on their lugs (REVOLUTE).
#
# Coordinate convention (Z-up world):
#   - vertical axis of the can : +Z
#   - footprint sits at z = 0; the body rises in +Z.
#   - handle 0 on the +X side, handle 1 on the -X side.
#
# Parts / articulations:
#   - can_body (ROOT): hollow tapered corrugated steel cylinder, rolled rim,
#       solid floor, two riveted lug pads (inline visuals).
#   - bail_handle_0 / bail_handle_1 : swinging D-ring bail handles, each
#       REVOLUTE about a horizontal tangent axis through its lug pivot.

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
LUG_Z_FRAC = 0.66       # lug height as fraction of body height
BAIL_UPPER = math.radians(90.0)  # max bail swing (horizontal carry position)


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
    """Small riveted ear that carries a bail handle, set against the body wall."""
    stub = CylinderGeometry(0.016, 0.012, radial_segments=16)
    stub.rotate_y(math.pi / 2.0)
    stub.translate(radius - 0.002, 0.0, 0.0)
    stub.rotate_z(angle)
    stub.translate(0.0, 0.0, z)
    return stub


def _bail_handle_local(side: float = 1.0) -> MeshGeometry:
    """Swinging bail / drop-ring handle in local pivot frame.

    Pivot at origin. For side=+1 the outward direction from the can body is
    local +X; for side=-1 it is local -X. +Z is always up.
    At q=0 the ring hangs down from the pivot against the can body; positive
    joint angle swings it outward and upward.
    """
    pts = []
    n = 22
    a_out = 0.045   # outward bow amplitude
    b_dn = 0.055    # vertical half-height of the oval ring
    for k in range(n + 1):
        t = 2.0 * math.pi * k / n
        x = side * (-0.004 + a_out * (0.5 - 0.5 * math.cos(t)))
        z = -0.035 + b_dn * math.sin(t)
        pts.append((x, 0.0, z))
    return tube_from_spline_points(
        pts,
        radius=0.005,
        samples_per_segment=6,
        radial_segments=12,
        closed_spline=True,
        cap_ends=False,
    )


# Handle configuration: angle around the can, joint axis, and outward side.
# For handle i at angle a:
#   pivot = (lug_r * cos(a), lug_r * sin(a), lug_z)
#   axis chosen so that positive q swings the ring outward from the can body.
_HANDLE_CONFIGS = [
    {"angle": 0.0, "axis": (0.0, -1.0, 0.0), "side": 1.0},      # +X side
    {"angle": math.pi, "axis": (0.0, 1.0, 0.0), "side": -1.0},  # -X side
]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="galvanized_trash_can_open_top")

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
        mesh_from_geometry(
            _floor_disc(radius=R_BOT - 0.003, z_bottom=0.0, thickness=FLOOR_T),
            "floor",
        ),
        material=galv_dark,
        name="floor",
    )
    body.visual(
        mesh_from_geometry(
            _rolled_rim(
                radius=R_TOP - RIM_TUBE + 0.004,
                z=BODY_H - RIM_TUBE,
                tube=RIM_TUBE,
            ),
            "rolled_rim",
        ),
        material=galv,
        name="rolled_rim",
    )

    # Lug pads — inline non-moving decorations on the body
    lug_z = BODY_H * LUG_Z_FRAC
    lug_r = _radius_at(lug_z)
    for i in range(len(_HANDLE_CONFIGS)):
        cfg = _HANDLE_CONFIGS[i]
        body.visual(
            mesh_from_geometry(
                _lug_pad(cfg["angle"], radius=lug_r, z=lug_z),
                f"lug_{i}",
            ),
            material=galv_dark,
            name=f"lug_{i}",
        )

    body.inertial = Inertial.from_geometry(
        Box((2 * R_TOP, 2 * R_TOP, BODY_H)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # ---- swinging bail handles (REVOLUTE on both sides) ----
    for i in range(len(_HANDLE_CONFIGS)):
        cfg = _HANDLE_CONFIGS[i]
        handle = model.part(f"bail_handle_{i}")
        handle.visual(
            mesh_from_geometry(
                _bail_handle_local(side=cfg["side"]),
                f"ring_{i}",
            ),
            material=wire,
            name=f"ring_{i}",
        )
        handle.inertial = Inertial.from_geometry(
            Box((0.05, 0.04, 0.11)),
            mass=0.10,
            origin=Origin(xyz=(cfg["side"] * 0.020, 0.0, -0.035)),
        )

        pivot_x = lug_r * math.cos(cfg["angle"])
        pivot_y = lug_r * math.sin(cfg["angle"])

        model.articulation(
            f"body_to_bail_{i}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=handle,
            origin=Origin(xyz=(pivot_x, pivot_y, lug_z)),
            axis=cfg["axis"],
            motion_limits=MotionLimits(
                lower=0.0,
                upper=BAIL_UPPER,
                effort=5.0,
                velocity=2.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("can_body")
    bail_0 = object_model.get_part("bail_handle_0")
    bail_1 = object_model.get_part("bail_handle_1")
    joint_0 = object_model.get_articulation("body_to_bail_0")
    joint_1 = object_model.get_articulation("body_to_bail_1")

    lug_z = BODY_H * LUG_Z_FRAC
    lug_r = _radius_at(lug_z)

    # ---- footprint at z ~ 0 ----
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "can base sits at z~0",
        abs(body_aabb[0][2]) < 0.005,
        details=f"body z-min={body_aabb[0][2]:.4f}",
    )
    ext = (
        body_aabb[1][0] - body_aabb[0][0],
        body_aabb[1][1] - body_aabb[0][1],
        body_aabb[1][2] - body_aabb[0][2],
    )
    ctx.check(
        "body roughly cylindrical (x~y) and tall",
        abs(ext[0] - ext[1]) < 0.05 and ext[2] > ext[0] + 0.10,
        details=f"body extents (x,y,z)={ext}",
    )

    # ---- tapered body ----
    ctx.check(
        "body is tapered (rim wider than foot)",
        R_TOP > R_BOT + 0.02,
        details=f"R_TOP={R_TOP}, R_BOT={R_BOT}",
    )

    # ---- deep hollow interior ----
    ctx.check(
        "deep interior cavity (hollow can)",
        (BODY_H - RIM_TUBE) - FLOOR_T > 0.45,
        details=f"cavity depth ~{(BODY_H - RIM_TUBE) - FLOOR_T:.3f} m",
    )

    # ---- open top: no lid, body top is approximately the rim height ----
    ctx.check(
        "open top — body top near rim height (no lid above)",
        body_aabb[1][2] < BODY_H + RIM_TUBE + 0.010,
        details=f"body z-max={body_aabb[1][2]:.3f}, rim+tol={BODY_H + RIM_TUBE + 0.010:.3f}",
    )

    # ---- bail handles on opposite sides, extending past the body ----
    b0_rest = ctx.part_world_aabb(bail_0)
    b1_rest = ctx.part_world_aabb(bail_1)

    ctx.check(
        "bail handle 0 on +X side past the body wall",
        b0_rest[1][0] > lug_r + 0.02,
        details=f"bail_0 x-max={b0_rest[1][0]:.3f}, lug_r={lug_r:.3f}",
    )
    ctx.check(
        "bail handle 1 on -X side past the body wall",
        b1_rest[0][0] < -(lug_r + 0.02),
        details=f"bail_1 x-min={b1_rest[0][0]:.3f}",
    )

    # ---- bail handles hang below the lug at rest ----
    ctx.check(
        "bail 0 hangs below lug at rest",
        b0_rest[0][2] < lug_z - 0.05,
        details=f"bail_0 z-min={b0_rest[0][2]:.3f}, lug_z={lug_z:.3f}",
    )
    ctx.check(
        "bail 1 hangs below lug at rest",
        b1_rest[0][2] < lug_z - 0.05,
        details=f"bail_1 z-min={b1_rest[0][2]:.3f}, lug_z={lug_z:.3f}",
    )

    # ---- bail handles swing up when opened ----
    rest_z_max_0 = b0_rest[1][2]
    with ctx.pose({joint_0: BAIL_UPPER}):
        swung_0 = ctx.part_world_aabb(bail_0)
    ctx.check(
        "bail 0 swings upward when opened",
        swung_0[1][2] > rest_z_max_0 + 0.02,
        details=f"rest z-max={rest_z_max_0:.3f}, swung z-max={swung_0[1][2]:.3f}",
    )

    rest_z_max_1 = b1_rest[1][2]
    with ctx.pose({joint_1: BAIL_UPPER}):
        swung_1 = ctx.part_world_aabb(bail_1)
    ctx.check(
        "bail 1 swings upward when opened",
        swung_1[1][2] > rest_z_max_1 + 0.02,
        details=f"rest z-max={rest_z_max_1:.3f}, swung z-max={swung_1[1][2]:.3f}",
    )

    # ---- joint axes are horizontal (along Y, tangent to can at each side) ----
    for jname in ("body_to_bail_0", "body_to_bail_1"):
        j = object_model.get_articulation(jname)
        ax = j.axis
        ctx.check(
            f"{jname} axis is horizontal along Y",
            abs(ax[1]) > 0.99 and abs(ax[0]) < 0.01 and abs(ax[2]) < 0.01,
            details=f"axis={ax}",
        )

    # ---- handle ring hooks into the lug (small intentional overlap) ----
    ctx.allow_overlap(
        bail_0, body,
        elem_a="ring_0",
        elem_b="corrugated_wall",
        reason="Bail ring rests against the outer corrugated wall surface at the lug mount.",
    )
    ctx.allow_overlap(
        bail_1, body,
        elem_a="ring_1",
        elem_b="corrugated_wall",
        reason="Bail ring rests against the outer corrugated wall surface at the lug mount.",
    )
    ctx.allow_overlap(
        bail_0, body,
        elem_a="ring_0",
        elem_b="lug_0",
        reason="Bail ring top hooks into the riveted lug pad and pivots on it.",
    )
    ctx.allow_overlap(
        bail_1, body,
        elem_a="ring_1",
        elem_b="lug_1",
        reason="Bail ring top hooks into the riveted lug pad and pivots on it.",
    )

    # ---- prove the handles are actually supported by the body at the lugs ----
    ctx.expect_overlap(
        bail_0, body, axes="z",
        elem_a="ring_0", elem_b="corrugated_wall",
        min_overlap=0.02,
        name="bail 0 ring overlaps the body wall zone vertically (supported at lug)",
    )
    ctx.expect_overlap(
        bail_1, body, axes="z",
        elem_a="ring_1", elem_b="corrugated_wall",
        min_overlap=0.02,
        name="bail 1 ring overlaps the body wall zone vertically (supported at lug)",
    )

    return ctx.report()


object_model = build_object_model()
