from __future__ import annotations

# Articraft model: a binder clip (foldback clip) — rounded-dome body variant.
#
# Real object (from picture/Stationary/Clip/001.png):
#   - A single piece of spring steel forms a rounded-dome body, painted glossy
#     orange with a speckled finish. The wide flat bottom is the clamping mouth;
#     the two side faces sweep up into a single smooth rounded dome ridge
#     instead of meeting at a sharp folded apex. The two bottom front/back edges
#     are rolled into small lips (barrels) that run across the full width.
#   - Two stiff steel wire handles (lever loops) are threaded through those two
#     rolled lips. Each handle pivots about its lip axis: you squeeze the
#     handles upward/together to spring the mouth open, and they fold back down
#     flat against the body at rest. The handles are the moving parts.
#
# Variant change (body cross-section family):
#   - Replaced the folded triangular-prism cross-section with a rounded-back
#     body family. The body center-line sweeps up from each lip into a smooth
#     sinusoidal arch dome, authored with CadQuery spline interpolation for the
#     outer and inner sheet surfaces so the rounded fold reads clearly.
#
# Articulation:
#   - Root: clip_body (the rounded-dome spring-steel body).
#   - front_handle, rear_handle: each a REVOLUTE lever pivoting about its rolled
#     lip axis (the width / Y axis of the clip). Positive q lifts the free end
#     of the handle up and away from the body (the squeeze/open gesture).
#
# Frame convention:
#   - +Y is the clip width and the lip/pivot axis.
#   - +X is depth (front-to-back); the front lip is at -X, the rear lip at +X.
#   - +Z is up; the clamping mouth (bottom) sits on z = 0 and the dome apex is
#     the highest point.

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

# Lip centers: the two bottom corners of the triangle.
FRONT_LIP = (-HALF_D, 0.0)   # (x, z)
REAR_LIP = (HALF_D, 0.0)


def _dome_centerline(n_control: int = 9) -> list[tuple[float, float]]:
    """Smooth dome arch center-line from front lip to rear lip.

    Uses sin(pi * t) so the arch rises smoothly from each lip with a horizontal
    tangent at the rounded apex — no sharp fold.  Returns *n_control* (x, z)
    sample points suitable for CadQuery spline interpolation.
    """
    pts: list[tuple[float, float]] = []
    for i in range(n_control):
        t = i / max(n_control - 1, 1)
        x = FRONT_LIP[0] * (1.0 - t) + REAR_LIP[0] * t
        z = LIP_R + (APEX_Z - LIP_R) * math.sin(math.pi * t)
        pts.append((x, z))
    return pts


def _offset_curve(pts: list[tuple[float, float]], offset: float) -> list[tuple[float, float]]:
    """Offset a 2-D (x, z) polyline along outward normals in the XZ plane."""
    n = len(pts)
    result: list[tuple[float, float]] = []
    for i in range(n):
        px, pz = pts[i]
        x0, z0 = pts[max(i - 1, 0)]
        x1, z1 = pts[min(i + 1, n - 1)]
        tx, tz = x1 - x0, z1 - z0
        tl = math.hypot(tx, tz) or 1.0
        tx /= tl
        tz /= tl
        # Normal (rotate tangent +90°): points generally "outward/up"
        nx, nz = -tz, tx
        if nz < 0:
            nx, nz = -nx, -nz
        result.append((px + nx * offset, pz + nz * offset))
    return result


def _build_body_mesh():
    """Rounded-dome sheet-steel body with two rolled lips, in CadQuery.

    The body cross-section is a smooth sinusoidal arch authored with spline
    interpolation for the outer and inner sheet surfaces.  The two side faces
    sweep up into a single smooth rounded dome ridge instead of meeting at a
    sharp folded apex.
    """
    centerline = _dome_centerline(n_control=9)

    h = SHEET_T / 2.0
    outer_pts = _offset_curve(centerline, +h)
    inner_pts = _offset_curve(centerline, -h)
    inner_rev = list(reversed(inner_pts))

    # Build the closed sheet cross-section on the XZ workplane using CadQuery
    # spline() for the outer dome curve and the inner return curve, joined by
    # short end-cap lines.  Extrude along Y (the clip width).
    wp = cq.Workplane("XZ")
    wp = wp.moveTo(*outer_pts[0])
    wp = wp.spline(outer_pts[1:], includeCurrent=True)
    # Rear end cap: outer last → inner-reversed first
    wp = wp.lineTo(*inner_rev[0])
    # Inner curve back (reversed order)
    wp = wp.spline(inner_rev[1:], includeCurrent=True)
    # Front end cap: close back to outer start
    wp = wp.close()
    band = wp.extrude(WIDTH)
    # XZ workplane extrudes into -Y by default; recenter on Y = 0.
    band = band.translate((0, HALF_W, 0))

    # Rolled lips: hollow tubes (barrels) running along Y at the two bottom
    # corners.  The wire handles thread through these.
    def lip(cx: float, cz: float):
        outer_tube = (
            cq.Workplane("XY")
            .workplane(offset=0)
            .center(cx, 0)
            .circle(LIP_R)
            .extrude(WIDTH)
        )
        outer_tube = outer_tube.rotate((0, 0, 0), (1, 0, 0), -90)
        inner_tube = (
            cq.Workplane("XY")
            .center(cx, 0)
            .circle(LIP_INNER_R)
            .extrude(WIDTH)
        )
        inner_tube = inner_tube.rotate((0, 0, 0), (1, 0, 0), -90)
        barrel = outer_tube.cut(inner_tube)
        barrel = barrel.translate((0, -HALF_W, cz))
        return barrel

    body = band.union(lip(*FRONT_LIP)).union(lip(*REAR_LIP))
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
    # leg root hooks *around* the rolled lip barrel so the wire is captured by
    # (and contacts) the lip, then runs out to the free tip laid against the
    # body. Authored in lip-local coords: x toward the free end, y across width,
    # z up.
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

    # Trace one continuous loop:
    #   +y leg: hook around barrel -> out to tip -> across to -y -> back -> hook.
    return [
        (hook_x, +yw, hook_z),               # +y hook under/behind the barrel
        (reach_dir * wrap, +yw, z_flat),     # +y leg rises onto the body
        (ex * 0.55, +yw, z_flat * 0.9),
        (ex, +yw * 0.7, z_flat * 0.6),
        (ex * 1.02, 0.0, z_flat * 0.55),     # rounded free tip
        (ex, -yw * 0.7, z_flat * 0.6),
        (ex * 0.55, -yw, z_flat * 0.9),
        (reach_dir * wrap, -yw, z_flat),     # -y leg
        (hook_x, -yw, hook_z),               # -y hook under/behind the barrel
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="binder_clip")

    steel_orange = Material(name="clip_orange", rgba=(0.86, 0.34, 0.13, 1.0))
    wire_steel = Material(name="wire_steel", rgba=(0.30, 0.30, 0.33, 1.0))

    # --- Root: rounded-dome spring-steel body --------------------------------
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
    # Rounded-dome body: rises clearly in +Z (smooth dome ridge) and spans
    # the full width Y.
    body_height = bz1 - bz0
    ctx.check(
        "body has the dome apex height",
        body_height > 0.012,
        details=f"height={body_height:.4f}",
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
    # Rounded dome: the body peak must reach at least the parametric dome apex
    # (APEX_Z ~ 0.018 m above the mouth plane, minus a small sheet-offset
    # tolerance).  A sharp triangular fold at this depth span would be lower.
    ctx.check(
        "body dome reaches the rounded apex height",
        bz1 > APEX_Z - 0.002,
        details=f"z_max={bz1:.4f}, APEX_Z={APEX_Z:.4f}",
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
