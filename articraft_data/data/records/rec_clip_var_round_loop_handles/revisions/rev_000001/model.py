from __future__ import annotations

# Articraft model: a binder clip (foldback clip).
#
# Real object (from picture/Stationary/Clip/001.png):
#   - A single piece of folded spring steel forms a roughly triangular-prism
#     body, painted glossy orange with a speckled finish. The wide flat bottom
#     is the clamping mouth; the two sloped faces meet at a folded ridge on top.
#     The two bottom front/back edges are rolled into small lips (barrels) that
#     run across the full width of the clip.
#   - Two stiff steel wire handles (lever loops) are threaded through those two
#     rolled lips. Each handle pivots about its lip axis: you squeeze the
#     handles upward/together to spring the mouth open, and they fold back down
#     flat against the body at rest. The handles are the moving parts.
#
# Articulation:
#   - Root: clip_body (the folded triangular spring-steel body).
#   - front_handle, rear_handle: each a REVOLUTE lever pivoting about its rolled
#     lip axis (the width / Y axis of the clip). Positive q lifts the free end
#     of the handle up and away from the body (the squeeze/open gesture).
#
# Frame convention:
#   - +Y is the clip width and the lip/pivot axis.
#   - +X is depth (front-to-back); the front lip is at -X, the rear lip at +X.
#   - +Z is up; the clamping mouth (bottom of the triangle) sits on z = 0 and the
#     folded apex is the highest point.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ----------------------------------------------------------------------------
# Dimensions (meters) -- a standard ~32 mm "medium" binder clip.
# ----------------------------------------------------------------------------
WIDTH = 0.032          # clip width along Y (the lip / pivot axis span)
HALF_W = WIDTH / 2.0

DEPTH = 0.026          # front-to-back span of the clamping mouth (X)
HALF_D = DEPTH / 2.0
APEX_Z = 0.018         # height of the folded ridge above the mouth (Z)
APEX_X = 0.0           # ridge sits centered front-to-back

SHEET_T = 0.0010       # spring-steel sheet thickness
LIP_R = 0.0018         # outer radius of each rolled lip (barrel)
LIP_INNER_R = 0.0011   # hollow inner radius of each rolled lip

WIRE_R = 0.00085       # handle wire radius
HANDLE_LEN = 0.030     # how far a handle reaches from its lip when laid flat
HANDLE_HALF_W = 0.0085  # half the span between a handle's two legs (narrow near barrel)
TEARDROP_MAX_HW = 0.013  # max half-width of the teardrop bulge
TEARDROP_TIP_HW = 0.0045  # half-width at the rounded teardrop tip

# Lip centers: the two bottom corners of the triangle.
FRONT_LIP = (-HALF_D, 0.0)   # (x, z)
REAR_LIP = (HALF_D, 0.0)


def _band_profile() -> list[tuple[float, float]]:
    """2D (x, z) center-line of the folded triangular spring-steel band.

    Traced front-bottom -> up the front face -> over the apex -> down the rear
    face -> rear-bottom. The clamping mouth is the open bottom between the two
    lips; we model the band as the visible folded sheet, so the bottom stays
    open (hollow mouth).
    """
    return [
        (FRONT_LIP[0], LIP_R),        # just above the front lip
        (-HALF_D * 0.55, APEX_Z * 0.62),
        (APEX_X, APEX_Z),             # folded ridge / apex
        (HALF_D * 0.55, APEX_Z * 0.62),
        (REAR_LIP[0], LIP_R),         # just above the rear lip
    ]


def _build_body_mesh():
    """Folded sheet-steel triangular body with two rolled lips, in CadQuery."""
    centerline = _band_profile()

    # Build a thin-walled folded band by sweeping a short across-thickness
    # rectangle along the (x,z) center-line, extruded across the full width Y.
    # We construct it as a swept solid: offset the polyline center-line to both
    # faces of the sheet, then extrude the closed loop along Y.
    pts = [(x, z) for (x, z) in centerline]

    # Outward normals (in XZ) at each vertex to give the band its thickness.
    n = len(pts)
    outer: list[tuple[float, float]] = []
    inner: list[tuple[float, float]] = []
    for i in range(n):
        px, pz = pts[i]
        # tangent from neighbours
        x0, z0 = pts[max(i - 1, 0)]
        x1, z1 = pts[min(i + 1, n - 1)]
        tx, tz = (x1 - x0), (z1 - z0)
        tl = math.hypot(tx, tz) or 1.0
        tx, tz = tx / tl, tz / tl
        # normal (rotate tangent +90deg): points generally "outward/up"
        nx, nz = -tz, tx
        # ensure the normal points away from interior (upward-ish)
        if nz < 0:
            nx, nz = -nx, -nz
        h = SHEET_T / 2.0
        outer.append((px + nx * h, pz + nz * h))
        inner.append((px - nx * h, pz - nz * h))

    loop = outer + list(reversed(inner))
    band = (
        cq.Workplane("XZ")
        .polyline([(x, z) for (x, z) in loop])
        .close()
        .extrude(WIDTH)
    )
    # Extrude along XZ workplane pushes into -Y by default; recenter on Y=0.
    band = band.translate((0, HALF_W, 0))

    # Rolled lips: hollow tubes (barrels) running along Y at the two bottom
    # corners. The wire handles thread through these.
    def lip(cx: float, cz: float):
        outer_tube = (
            cq.Workplane("XY")
            .workplane(offset=0)
            .center(cx, 0)
            .circle(LIP_R)
            .extrude(WIDTH)
        )
        # The XY circle extrudes along +Z; rotate so the barrel runs along Y.
        outer_tube = outer_tube.rotate((0, 0, 0), (1, 0, 0), -90)
        inner_tube = (
            cq.Workplane("XY")
            .center(cx, 0)
            .circle(LIP_INNER_R)
            .extrude(WIDTH)
        )
        inner_tube = inner_tube.rotate((0, 0, 0), (1, 0, 0), -90)
        barrel = outer_tube.cut(inner_tube)
        # The -90deg rotation about X maps (x, y, z) -> (x, z, -y), so the tube
        # extruded over z in [0, WIDTH] now spans y in [0, WIDTH]; shift to
        # [-HALF_W, HALF_W] then lift to the lip height cz.
        barrel = barrel.translate((0, -HALF_W, cz))
        return barrel

    body = band.union(lip(*FRONT_LIP)).union(lip(*REAR_LIP))
    return mesh_from_cadquery(body, "clip_body", tolerance=0.0004, angular_tolerance=0.2)


def _handle_points(lip_x: float, lip_z: float, reach_dir: float) -> list[tuple[float, float, float]]:
    """Center-line points for one teardrop-shaped wire handle laid flat.

    The handle is authored in the joint frame whose origin is at the lip center,
    so points are relative to the lip. ``reach_dir`` is +1 (reaches toward +X /
    outward-rear) or -1 (reaches toward -X / outward-front) depending on which
    lip. We author the *closed* (folded flat) pose: the loop lies low, hugging
    the body, with its free end out near the mouth plane.

    The teardrop shape: narrow near the barrel hooks, bowing out to a smooth
    wide bulge in the middle-to-free region, then narrowing to a rounded tip.
    The center-line is a smooth spline through ~15 control points so the
    curvature reads clearly as a teardrop ring, not a squared U.
    """
    ex = reach_dir * HANDLE_LEN          # free-end x offset from lip
    z_flat = LIP_R + WIRE_R + 0.0003     # leg height when laid against the body

    # Wrap radius: the bent wire centerline rides just on the barrel surface,
    # with a hair of nest so it reads as captured (allowed via allow_overlap).
    wrap = LIP_R + WIRE_R * 0.2          # centerline distance from lip center
    hook_x = -reach_dir * wrap * 0.85
    hook_z = -wrap * 0.45

    # Teardrop width profile: narrow at barrel, wide in middle, rounded tip.
    yw_narrow = HANDLE_HALF_W * 0.62     # near the barrel hooks
    yw_mid = TEARDROP_MAX_HW * 0.78      # transitioning outward
    yw_wide = TEARDROP_MAX_HW            # max bulge width
    yw_tip = TEARDROP_TIP_HW             # rounded tip width

    # X-stations along the reach (fractions of full HANDLE_LEN).
    x0 = reach_dir * wrap                # emerging from barrel
    x1 = reach_dir * HANDLE_LEN * 0.18   # leg rising
    x2 = reach_dir * HANDLE_LEN * 0.35   # beginning to bow out
    x3 = reach_dir * HANDLE_LEN * 0.52   # approaching max width
    x4 = reach_dir * HANDLE_LEN * 0.70   # at max width (bulge)
    x5 = reach_dir * HANDLE_LEN * 0.85   # past bulge, curving to tip
    x6 = reach_dir * HANDLE_LEN * 0.95   # approaching tip
    x7 = reach_dir * HANDLE_LEN * 1.03   # rounded tip apex

    # Z profile: legs lie flat against the body, slight dip at the tip.
    z0 = z_flat * 0.92
    z1 = z_flat
    z2 = z_flat
    z3 = z_flat * 0.97
    z4 = z_flat * 0.92
    z5 = z_flat * 0.82
    z6 = z_flat * 0.68
    z7 = z_flat * 0.52

    # Trace the continuous teardrop loop:
    # +y hook -> +y leg -> +y bulge -> +y approach -> tip -> -y approach ->
    # -y bulge -> -y leg -> -y hook
    return [
        (hook_x, +yw_narrow, hook_z),    # +y hook under/behind the barrel
        (x0, +yw_narrow, z0),            # +y leg emerging from barrel
        (x1, +yw_mid, z1),               # +y leg bowing outward
        (x2, +yw_wide * 0.92, z2),       # +y widening toward bulge
        (x3, +yw_wide, z3),              # +y at max bulge
        (x4, +yw_wide * 0.95, z4),       # +y past bulge
        (x5, +yw_tip * 1.4, z5),         # +y curving toward tip
        (x6, +yw_tip, z6),               # +y approaching rounded tip
        (x7, 0.0, z7),                   # rounded tip apex (center)
        (x6, -yw_tip, z6),               # -y leaving rounded tip
        (x5, -yw_tip * 1.4, z5),         # -y curving from tip
        (x4, -yw_wide * 0.95, z4),       # -y past bulge
        (x3, -yw_wide, z3),              # -y at max bulge
        (x2, -yw_wide * 0.92, z2),       # -y narrowing from bulge
        (x1, -yw_mid, z1),               # -y leg coming back
        (x0, -yw_narrow, z0),            # -y leg approaching barrel
        (hook_x, -yw_narrow, hook_z),    # -y hook under/behind the barrel
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="binder_clip")

    steel_orange = Material(name="clip_orange", rgba=(0.86, 0.34, 0.13, 1.0))
    wire_steel = Material(name="wire_steel", rgba=(0.30, 0.30, 0.33, 1.0))

    # --- Root: folded triangular spring-steel body --------------------------
    body = model.part("clip_body")
    body.visual(_build_body_mesh(), material=steel_orange, name="body_shell")

    # --- Two wire lever handles --------------------------------------------
    # Front handle: pivots in the front lip, reaches outward toward -X.
    # --- Two teardrop wire lever handles ------------------------------------
    # Each handle is a smooth teardrop-shaped closed wire loop: narrow near the
    # barrel hooks, bowing out to a wide smooth bulge, then narrowing to a
    # rounded tip. Built via shared geometry helper with regular placement.
    def _build_handle(name_tag: str, lip: tuple[float, float], reach_dir: float):
        pts = _handle_points(*lip, reach_dir=reach_dir)
        wire = tube_from_spline_points(
            pts,
            radius=WIRE_R,
            samples_per_segment=18,
            radial_segments=14,
            cap_ends=True,
        )
        return mesh_from_geometry(wire, f"{name_tag}_handle_loop")

    front_handle = model.part("front_handle")
    front_handle.visual(
        _build_handle("front", FRONT_LIP, reach_dir=-1.0),
        material=wire_steel,
        name="front_handle_loop",
    )

    rear_handle = model.part("rear_handle")
    rear_handle.visual(
        _build_handle("rear", REAR_LIP, reach_dir=1.0),
        material=wire_steel,
        name="rear_handle_loop",
    )

    # --- Articulations: each handle pivots about its rolled-lip (Y) axis -----
    # Front handle laid flat reaches toward -X. To lift the free end up (+Z),
    # positive rotation about +Y rotates -X toward +Z, so axis = (0,1,0).
    model.articulation(
        "front_handle_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=front_handle,
        origin=Origin(xyz=(FRONT_LIP[0], 0.0, FRONT_LIP[1])),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0, lower=0.0, upper=2.0),
    )
    # Rear handle reaches toward +X; positive rotation about -Y lifts +X to +Z.
    model.articulation(
        "rear_handle_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rear_handle,
        origin=Origin(xyz=(REAR_LIP[0], 0.0, REAR_LIP[1])),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0, lower=0.0, upper=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("clip_body")
    front_handle = object_model.get_part("front_handle")
    rear_handle = object_model.get_part("rear_handle")
    front_pivot = object_model.get_articulation("front_handle_pivot")
    rear_pivot = object_model.get_articulation("rear_handle_pivot")

    # --- Mechanism type and axis claims ------------------------------------
    ctx.check(
        "front handle is a revolute lever",
        front_pivot.joint_type == "revolute",
        details=f"got {front_pivot.joint_type}",
    )
    ctx.check(
        "rear handle is a revolute lever",
        rear_pivot.joint_type == "revolute",
        details=f"got {rear_pivot.joint_type}",
    )
    ctx.check(
        "front pivot axis is the lip (Y) axis",
        abs(front_pivot.axis[1]) > 0.99 and abs(front_pivot.axis[0]) < 1e-6,
        details=f"axis={front_pivot.axis}",
    )
    ctx.check(
        "rear pivot axis is the lip (Y) axis",
        abs(rear_pivot.axis[1]) > 0.99 and abs(rear_pivot.axis[0]) < 1e-6,
        details=f"axis={rear_pivot.axis}",
    )

    # Body is the single root; handles hang off it.
    roots = [p.name for p in object_model.root_parts()]
    ctx.check(
        "clip body is the sole root",
        roots == ["clip_body"],
        details=f"roots={roots}",
    )

    # --- Geometry / proportion claims --------------------------------------
    body_aabb = ctx.part_world_aabb(body)
    assert body_aabb is not None
    (bx0, by0, bz0), (bx1, by1, bz1) = body_aabb
    # Triangular body: rises clearly in +Z (folded apex) and spans the width Y.
    ctx.check(
        "body has the folded apex height",
        (bz1 - bz0) > 0.012,
        details=f"height={bz1 - bz0:.4f}",
    )
    ctx.check(
        "body spans the full clip width",
        (by1 - by0) > 0.028,
        details=f"width={by1 - by0:.4f}",
    )
    ctx.check(
        "body mouth sits near z=0",
        abs(bz0) < 0.002,
        details=f"z_min={bz0:.4f}",
    )

    # Handles sit at the bottom corners (near the lips), not floating mid-air.
    fh_aabb = ctx.part_world_aabb(front_handle)
    rh_aabb = ctx.part_world_aabb(rear_handle)
    assert fh_aabb is not None and rh_aabb is not None
    # Front handle reaches toward -X beyond the front lip.
    ctx.check(
        "front handle reaches outward past the front lip",
        fh_aabb[0][0] < FRONT_LIP[0] - 0.005,
        details=f"front_handle x_min={fh_aabb[0][0]:.4f}, lip_x={FRONT_LIP[0]:.4f}",
    )
    ctx.check(
        "rear handle reaches outward past the rear lip",
        rh_aabb[1][0] > REAR_LIP[0] + 0.005,
        details=f"rear_handle x_max={rh_aabb[1][0]:.4f}, lip_x={REAR_LIP[0]:.4f}",
    )

    # Teardrop shape: each handle's Y-span must exceed the narrow parent U-shape
    # width (2 * HANDLE_HALF_W = 0.017) and approach the teardrop max bulge
    # (2 * TEARDROP_MAX_HW = 0.026).
    fh_dy = fh_aabb[1][1] - fh_aabb[0][1]
    rh_dy = rh_aabb[1][1] - rh_aabb[0][1]
    ctx.check(
        "front handle teardrop bulge is wider than a U-shape",
        fh_dy > 2.0 * HANDLE_HALF_W + 0.003,
        details=f"front handle Y span={fh_dy:.4f}, threshold={2.0 * HANDLE_HALF_W + 0.003:.4f}",
    )
    ctx.check(
        "rear handle teardrop bulge is wider than a U-shape",
        rh_dy > 2.0 * HANDLE_HALF_W + 0.003,
        details=f"rear handle Y span={rh_dy:.4f}, threshold={2.0 * HANDLE_HALF_W + 0.003:.4f}",
    )

    # Each handle is captured at its lip: the wire loop contacts the body barrel.
    ctx.expect_contact(
        front_handle, body,
        elem_a="front_handle_loop", elem_b="body_shell",
        contact_tol=0.0015,
        name="front handle threaded through front lip",
    )
    ctx.expect_contact(
        rear_handle, body,
        elem_a="rear_handle_loop", elem_b="body_shell",
        contact_tol=0.0015,
        name="rear handle threaded through rear lip",
    )

    # The wire threading through the rolled lip is an intentional capture nest.
    ctx.allow_overlap(
        front_handle, body,
        elem_a="front_handle_loop", elem_b="body_shell",
        reason="The wire handle is threaded through the rolled lip barrel; a small wire/lip nest is the real capture fit.",
    )
    ctx.allow_overlap(
        rear_handle, body,
        elem_a="rear_handle_loop", elem_b="body_shell",
        reason="The wire handle is threaded through the rolled lip barrel; a small wire/lip nest is the real capture fit.",
    )

    # --- Decisive motion check: squeezing the handle lifts its free end -----
    fh_rest = ctx.part_world_aabb(front_handle)
    assert fh_rest is not None
    rest_top = fh_rest[1][2]
    with ctx.pose({front_pivot: 1.6}):
        fh_open = ctx.part_world_aabb(front_handle)
        assert fh_open is not None
        open_top = fh_open[1][2]
    ctx.check(
        "raising the front handle lifts its free end upward",
        open_top > rest_top + 0.008,
        details=f"rest_top={rest_top:.4f}, open_top={open_top:.4f}",
    )

    # Same for the rear handle (mirrored axis).
    rh_rest = ctx.part_world_aabb(rear_handle)
    assert rh_rest is not None
    rest_top_r = rh_rest[1][2]
    with ctx.pose({rear_pivot: 1.6}):
        rh_open = ctx.part_world_aabb(rear_handle)
        assert rh_open is not None
        open_top_r = rh_open[1][2]
    ctx.check(
        "raising the rear handle lifts its free end upward",
        open_top_r > rest_top_r + 0.008,
        details=f"rest_top={rest_top_r:.4f}, open_top={open_top_r:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
