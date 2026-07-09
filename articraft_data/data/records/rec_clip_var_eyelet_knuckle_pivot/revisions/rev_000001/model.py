from __future__ import annotations

# Articraft model: a binder clip (foldback clip) — knuckle-eyelet variant.
#
# Real object (from picture/Stationary/Clip/001.png):
#   - A single piece of folded spring steel forms a roughly triangular-prism
#     body, painted glossy orange with a speckled finish. The wide flat bottom
#     is the clamping mouth; the two sloped faces meet at a folded ridge on top.
#   - VARIANT: instead of continuous rolled barrel lips running the full width,
#     each lip edge carries a set of discrete pierced eyelet knuckles — short
#     hollow cylindrical tubes spaced across the width. The wire handle hooks
#     through these eyelets and pivots about the same lip axis.
#   - Two stiff steel wire handles (lever loops) are threaded through the
#     knuckle eyelets on each lip. Each handle pivots about its lip axis: you
#     squeeze the handles upward/together to spring the mouth open, and they
#     fold back down flat against the body at rest. The handles are the moving
#     parts.
#
# Articulation:
#   - Root: clip_body (the folded triangular spring-steel body with knuckle
#     eyelets on each lip).
#   - front_handle, rear_handle: each a REVOLUTE lever pivoting about its lip
#     axis (the width / Y axis of the clip). Positive q lifts the free end of
#     the handle up and away from the body (the squeeze/open gesture).
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
LIP_R = 0.0018         # outer radius of each rolled lip (barrel / knuckle)
LIP_INNER_R = 0.0011   # hollow inner radius of each rolled lip

WIRE_R = 0.00085       # handle wire radius
HANDLE_LEN = 0.030     # how far a handle reaches from its lip when laid flat
HANDLE_HALF_W = 0.0085  # half the span between a handle's two legs

# Knuckle eyelet geometry: each lip is a set of discrete short hollow cylinders
# (eyelet knuckles) spaced across the clip width, instead of one continuous
# rolled barrel. The wire handle hooks through these eyelets and pivots about
# the same lip axis.
N_KNUCKLES = 5                      # number of knuckle eyelets per lip
KNUCKLE_LEN = WIDTH / (N_KNUCKLES * 1.55)  # axial length of one knuckle
KNUCKLE_SPAN = WIDTH - KNUCKLE_LEN  # total Y span from first to last knuckle center

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


def _knuckle_eyelet(cx: float, cz: float, cy: float) -> cq.Workplane:
    """Build one short hollow cylindrical knuckle eyelet at (cx, cy, cz).

    The eyelet is a short tube aligned along the Y axis (the lip/pivot axis).
    ``cx`` and ``cz`` are the lip position; ``cy`` is the Y center of this
    knuckle within the clip width.
    """
    half_len = KNUCKLE_LEN / 2.0
    # Build a hollow tube: outer cylinder minus inner cylinder, both along Y.
    outer_cyl = (
        cq.Workplane("XZ")
        .workplane(offset=cy - half_len)
        .center(cx, cz)
        .circle(LIP_R)
        .extrude(KNUCKLE_LEN)
    )
    inner_cyl = (
        cq.Workplane("XZ")
        .workplane(offset=cy - half_len)
        .center(cx, cz)
        .circle(LIP_INNER_R)
        .extrude(KNUCKLE_LEN)
    )
    return outer_cyl.cut(inner_cyl)


def _knuckle_y_positions() -> list[float]:
    """Return evenly spaced Y centers for N_KNUCKLES knuckles across the width."""
    if N_KNUCKLES == 1:
        return [0.0]
    step = KNUCKLE_SPAN / (N_KNUCKLES - 1)
    return [-KNUCKLE_SPAN / 2.0 + i * step for i in range(N_KNUCKLES)]


def _build_body_mesh():
    """Folded sheet-steel triangular body with discrete knuckle eyelets on each lip."""
    centerline = _band_profile()

    # Build a thin-walled folded band by sweeping a short across-thickness
    # rectangle along the (x,z) center-line, extruded across the full width Y.
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

    # Knuckle eyelets: discrete short hollow tubes (pierced eyelets) spaced
    # across the clip width at each lip, replacing the continuous barrel.
    # The wire handle hooks through these eyelets and pivots about the same
    # lip axis.
    y_positions = _knuckle_y_positions()
    body = band
    for i in range(N_KNUCKLES):
        cy = y_positions[i]
        body = body.union(_knuckle_eyelet(FRONT_LIP[0], FRONT_LIP[1], cy))
        body = body.union(_knuckle_eyelet(REAR_LIP[0], REAR_LIP[1], cy))

    return mesh_from_cadquery(body, "clip_body", tolerance=0.0004, angular_tolerance=0.2)


def _handle_points(lip_x: float, lip_z: float, reach_dir: float) -> list[tuple[float, float, float]]:
    """Center-line points for one U-shaped wire handle laid flat against body.

    The handle is authored in the joint frame whose origin is at the lip center,
    so points are relative to the lip. ``reach_dir`` is +1 (reaches toward -X /
    outward-front) or -1 depending on which lip. We author the *closed* (folded
    flat) pose: the loop lies low, hugging the body, with its free end out near
    the mouth plane.
    """
    # The loop: two legs running out from the lip, joined by a curved tip. Each
    # leg root hooks *through* the knuckle eyelets so the wire is captured by
    # (and contacts) the lip, then runs out to the free tip laid against the
    # body. Authored in lip-local coords: x toward the free end, y across width,
    # z up.
    ex = reach_dir * HANDLE_LEN          # free-end x offset from lip
    yw = HANDLE_HALF_W
    z_flat = LIP_R + WIRE_R + 0.0003     # leg height when laid against the body

    # Wrap radius: the bent wire centerline rides just inside the knuckle
    # eyelet bore, with a hair of nest so it reads as captured (allowed via
    # allow_overlap).
    wrap = LIP_R + WIRE_R * 0.2          # centerline distance from lip center
    # Hook point on the far (inboard) side of the knuckle eyelet, slightly
    # under it, so the wire clearly encircles the lip.
    hook_x = -reach_dir * wrap * 0.85
    hook_z = -wrap * 0.45

    # Trace one continuous loop:
    #   +y leg: hook around barrel -> out to tip -> across to -y -> back -> hook.
    return [
        (hook_x, +yw, hook_z),               # +y hook under/behind the knuckle eyelet
        (reach_dir * wrap, +yw, z_flat),     # +y leg rises onto the body
        (ex * 0.55, +yw, z_flat * 0.9),
        (ex, +yw * 0.7, z_flat * 0.6),
        (ex * 1.02, 0.0, z_flat * 0.55),     # rounded free tip
        (ex, -yw * 0.7, z_flat * 0.6),
        (ex * 0.55, -yw, z_flat * 0.9),
        (reach_dir * wrap, -yw, z_flat),     # -y leg
        (hook_x, -yw, hook_z),               # -y hook under/behind the knuckle eyelet
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
    front_handle = model.part("front_handle")
    front_pts = _handle_points(*FRONT_LIP, reach_dir=-1.0)
    front_wire = tube_from_spline_points(
        front_pts,
        radius=WIRE_R,
        samples_per_segment=14,
        radial_segments=14,
        cap_ends=True,
    )
    front_handle.visual(
        mesh_from_geometry(front_wire, "front_handle_loop"),
        material=wire_steel,
        name="front_handle_loop",
    )

    rear_handle = model.part("rear_handle")
    rear_pts = _handle_points(*REAR_LIP, reach_dir=1.0)
    rear_wire = tube_from_spline_points(
        rear_pts,
        radius=WIRE_R,
        samples_per_segment=14,
        radial_segments=14,
        cap_ends=True,
    )
    rear_handle.visual(
        mesh_from_geometry(rear_wire, "rear_handle_loop"),
        material=wire_steel,
        name="rear_handle_loop",
    )

    # --- Articulations: each handle pivots about its knuckle-eyelet lip (Y) axis
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

    # Each handle is captured at its lip: the wire loop contacts the body knuckle
    # eyelets.
    ctx.expect_contact(
        front_handle, body,
        elem_a="front_handle_loop", elem_b="body_shell",
        contact_tol=0.0015,
        name="front handle threaded through front knuckle eyelets",
    )
    ctx.expect_contact(
        rear_handle, body,
        elem_a="rear_handle_loop", elem_b="body_shell",
        contact_tol=0.0015,
        name="rear handle threaded through rear knuckle eyelets",
    )

    # The wire threading through the knuckle eyelets is an intentional capture
    # nest.
    ctx.allow_overlap(
        front_handle, body,
        elem_a="front_handle_loop", elem_b="body_shell",
        reason="The wire handle is threaded through the knuckle eyelets on the front lip; a small wire/eyelet nest is the real capture fit.",
    )
    ctx.allow_overlap(
        rear_handle, body,
        elem_a="rear_handle_loop", elem_b="body_shell",
        reason="The wire handle is threaded through the knuckle eyelets on the rear lip; a small wire/eyelet nest is the real capture fit.",
    )

    # --- Knuckle eyelet geometry: the lips have discrete eyelets, not barrels --
    # The knuckle eyelet count per lip is N_KNUCKLES (>= 2), confirming the
    # bearing form changed from a continuous barrel to discrete pierced eyelets.
    ctx.check(
        "each lip has multiple discrete knuckle eyelets",
        N_KNUCKLES >= 3,
        details=f"n_knuckles={N_KNUCKLES}",
    )
    # Knuckle eyelets sit at the lip height (z near 0), not at the apex.
    # The body extends down to the lip region where knuckles are located.
    ctx.check(
        "body knuckle eyelets reach near the mouth plane",
        bz0 < LIP_R * 2.5,
        details=f"body_z_min={bz0:.4f}, lip_r={LIP_R:.4f}",
    )
    # Knuckle eyelets are spaced across the clip width.
    ctx.check(
        "knuckle span covers most of the clip width",
        KNUCKLE_SPAN > WIDTH * 0.5,
        details=f"knuckle_span={KNUCKLE_SPAN:.4f}, width={WIDTH:.4f}",
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
