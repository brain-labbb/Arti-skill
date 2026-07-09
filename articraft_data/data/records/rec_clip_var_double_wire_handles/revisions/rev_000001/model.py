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
HANDLE_HALF_W = 0.0085  # half the span between a handle's two legs

# Doubled-wire handle: two parallel strands per handle, offset side-by-side.
STRAND_COUNT = 2
STRAND_SPACING = 0.0022  # center-to-center Y gap between the two strands

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


def _handle_strand_points(
    lip_x: float, lip_z: float, reach_dir: float, y_offset: float = 0.0,
) -> list[tuple[float, float, float]]:
    """Center-line points for one strand of a doubled U-shaped wire handle.

    The handle is authored in the joint frame whose origin is at the lip center,
    so points are relative to the lip. ``reach_dir`` is +1 (reaches toward -X /
    outward-front) or -1 depending on which lip. ``y_offset`` shifts the leg
    midpoints in Y so two strands run side-by-side; hooks and tip are shared
    (offset tapers to zero at those anchor points).
    """
    ex = reach_dir * HANDLE_LEN          # free-end x offset from lip
    yw = HANDLE_HALF_W
    z_flat = LIP_R + WIRE_R + 0.0003     # leg height when laid against the body

    # Wrap radius: the bent wire centerline rides just on the barrel surface,
    # with a hair of nest so it reads as captured (allowed via allow_overlap).
    wrap = LIP_R + WIRE_R * 0.2          # centerline distance from lip center
    # Hook point on the far (inboard) side of the barrel, slightly under it,
    # so the wire clearly encircles the lip.
    hook_x = -reach_dir * wrap * 0.85
    hook_z = -wrap * 0.45

    # Tapered offsets: full y_offset at the straight-leg midpoints, partial
    # approaching the tip, zero at the shared hook roots and shared free tip.
    yo = y_offset
    yo_tip = y_offset * 0.35             # taper toward the shared tip

    # Trace one continuous strand loop:
    #   +y leg: hook around barrel -> out to tip -> across to -y -> back -> hook.
    return [
        (hook_x, +yw, hook_z),                        # +y hook (shared root)
        (reach_dir * wrap, +yw + yo, z_flat),         # +y leg rises
        (ex * 0.55, +yw + yo, z_flat * 0.9),          # +y leg mid
        (ex, +yw * 0.7 + yo_tip, z_flat * 0.6),       # +y tapering toward tip
        (ex * 1.02, 0.0, z_flat * 0.55),               # rounded free tip (shared)
        (ex, -yw * 0.7 + yo_tip, z_flat * 0.6),       # -y tapering toward tip
        (ex * 0.55, -yw + yo, z_flat * 0.9),          # -y leg mid
        (reach_dir * wrap, -yw + yo, z_flat),         # -y leg rises
        (hook_x, -yw, hook_z),                        # -y hook (shared root)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="binder_clip")

    steel_orange = Material(name="clip_orange", rgba=(0.86, 0.34, 0.13, 1.0))
    wire_steel = Material(name="wire_steel", rgba=(0.30, 0.30, 0.33, 1.0))

    # --- Root: folded triangular spring-steel body --------------------------
    body = model.part("clip_body")
    body.visual(_build_body_mesh(), material=steel_orange, name="body_shell")

    # --- Two doubled-wire lever handles ------------------------------------
    # Each handle is built from STRAND_COUNT parallel wire strands that share
    # the same hook root (around the lip barrel) and the same free tip, running
    # side-by-side for a heavier reinforced grip. Both strands are rigidly part
    # of the one pivoting handle link.
    def _build_strand_mesh(pts: list[tuple[float, float, float]]):
        """Shared geometry helper: one wire strand from center-line points."""
        return tube_from_spline_points(
            pts,
            radius=WIRE_R,
            samples_per_segment=14,
            radial_segments=14,
            cap_ends=True,
        )

    # Strand Y offsets: evenly spaced around zero so the pair is symmetric.
    strand_offsets = [
        (i - (STRAND_COUNT - 1) / 2.0) * STRAND_SPACING
        for i in range(STRAND_COUNT)
    ]

    # Front handle: pivots in the front lip, reaches outward toward -X.
    front_handle = model.part("front_handle")
    for i in range(STRAND_COUNT):
        pts = _handle_strand_points(*FRONT_LIP, reach_dir=-1.0, y_offset=strand_offsets[i])
        front_handle.visual(
            mesh_from_geometry(_build_strand_mesh(pts), f"front_handle_strand_{i}"),
            material=wire_steel,
            name=f"front_handle_strand_{i}",
        )

    # Rear handle: pivots in the rear lip, reaches outward toward +X.
    rear_handle = model.part("rear_handle")
    for i in range(STRAND_COUNT):
        pts = _handle_strand_points(*REAR_LIP, reach_dir=1.0, y_offset=strand_offsets[i])
        rear_handle.visual(
            mesh_from_geometry(_build_strand_mesh(pts), f"rear_handle_strand_{i}"),
            material=wire_steel,
            name=f"rear_handle_strand_{i}",
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

    # Each handle strand is captured at its lip: the wire contacts the body barrel.
    # Check the inner strand (strand_0) of each handle against the body.
    for i in range(STRAND_COUNT):
        ctx.expect_contact(
            front_handle, body,
            elem_a=f"front_handle_strand_{i}", elem_b="body_shell",
            contact_tol=0.0015,
            name=f"front handle strand {i} threaded through front lip",
        )
        ctx.expect_contact(
            rear_handle, body,
            elem_a=f"rear_handle_strand_{i}", elem_b="body_shell",
            contact_tol=0.0015,
            name=f"rear handle strand {i} threaded through rear lip",
        )

    # The wire threading through the rolled lip is an intentional capture nest.
    for i in range(STRAND_COUNT):
        ctx.allow_overlap(
            front_handle, body,
            elem_a=f"front_handle_strand_{i}", elem_b="body_shell",
            reason="The doubled wire handle strand is threaded through the rolled lip barrel; a small wire/lip nest is the real capture fit.",
        )
        ctx.allow_overlap(
            rear_handle, body,
            elem_a=f"rear_handle_strand_{i}", elem_b="body_shell",
            reason="The doubled wire handle strand is threaded through the rolled lip barrel; a small wire/lip nest is the real capture fit.",
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

    # --- Doubled-wire structure claims -------------------------------------
    # Each handle must have exactly STRAND_COUNT visuals (the parallel strands).
    front_visuals = [v.name for v in front_handle.visuals]
    rear_visuals = [v.name for v in rear_handle.visuals]
    ctx.check(
        "front handle has doubled wire strands",
        len(front_visuals) == STRAND_COUNT
        and all(f"front_handle_strand_{i}" in front_visuals for i in range(STRAND_COUNT)),
        details=f"visuals={front_visuals}",
    )
    ctx.check(
        "rear handle has doubled wire strands",
        len(rear_visuals) == STRAND_COUNT
        and all(f"rear_handle_strand_{i}" in rear_visuals for i in range(STRAND_COUNT)),
        details=f"visuals={rear_visuals}",
    )

    # The two strands of each handle are separated side-by-side in Y.
    fh_s0_aabb = ctx.part_element_world_aabb(front_handle, elem="front_handle_strand_0")
    fh_s1_aabb = ctx.part_element_world_aabb(front_handle, elem="front_handle_strand_1")
    assert fh_s0_aabb is not None and fh_s1_aabb is not None
    fh_s0_y_center = (fh_s0_aabb[0][1] + fh_s0_aabb[1][1]) / 2.0
    fh_s1_y_center = (fh_s1_aabb[0][1] + fh_s1_aabb[1][1]) / 2.0
    ctx.check(
        "front handle strands are side-by-side separated in Y",
        abs(fh_s1_y_center - fh_s0_y_center) > 0.001,
        details=f"strand_0 y_center={fh_s0_y_center:.4f}, strand_1 y_center={fh_s1_y_center:.4f}",
    )

    rh_s0_aabb = ctx.part_element_world_aabb(rear_handle, elem="rear_handle_strand_0")
    rh_s1_aabb = ctx.part_element_world_aabb(rear_handle, elem="rear_handle_strand_1")
    assert rh_s0_aabb is not None and rh_s1_aabb is not None
    rh_s0_y_center = (rh_s0_aabb[0][1] + rh_s0_aabb[1][1]) / 2.0
    rh_s1_y_center = (rh_s1_aabb[0][1] + rh_s1_aabb[1][1]) / 2.0
    ctx.check(
        "rear handle strands are side-by-side separated in Y",
        abs(rh_s1_y_center - rh_s0_y_center) > 0.001,
        details=f"strand_0 y_center={rh_s0_y_center:.4f}, strand_1 y_center={rh_s1_y_center:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
