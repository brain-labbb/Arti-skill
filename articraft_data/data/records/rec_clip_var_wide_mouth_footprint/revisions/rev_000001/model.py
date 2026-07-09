from __future__ import annotations

# Articraft model: a wide-mouth binder clip (foldback clamp).
#
# Variant of the standard medium binder clip with a wide-mouth clamp footprint.
# Same folded triangular spring-steel body, same two rolled lip barrels, same
# two wire lever handles, same two revolute pivots — but the front-to-back
# mouth span and lip-to-lip reach are substantially widened so the clip reads
# as a broad wide-mouth clamp that can swallow a thick stack.
#
# Real object identity:
#   - Single piece of folded spring steel forms a triangular-prism body.
#   - Wide flat bottom is the clamping mouth; two sloped faces meet at a
#     folded ridge on top.
#   - Two bottom edges rolled into small lips (barrels) across the full width.
#   - Two stiff steel wire handles threaded through those lips; each pivots
#     about its lip axis. Squeeze upward/together to spring the mouth open;
#     fold flat at rest.
#
# Frame convention:
#   +Y = clip width (lip / pivot axis)
#   +X = depth (front-to-back); front lip at -X, rear lip at +X
#   +Z = up; mouth sits on z=0, folded apex is highest point.

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

# ---------------------------------------------------------------------------
# Dimensions (meters) — wide-mouth clamp variant.
# ---------------------------------------------------------------------------
WIDTH = 0.032           # clip width along Y (lip / pivot axis span)
HALF_W = WIDTH / 2.0

DEPTH = 0.044           # WIDE mouth: front-to-back span (parent was 0.026)
HALF_D = DEPTH / 2.0
APEX_Z = 0.026          # proportionally taller ridge for wide triangle
APEX_X = 0.0            # ridge centered front-to-back

SHEET_T = 0.0010        # spring-steel sheet thickness
LIP_R = 0.0018          # outer radius of each rolled lip barrel
LIP_INNER_R = 0.0011    # hollow inner radius of each rolled lip

WIRE_R = 0.00085        # handle wire radius
HANDLE_LEN = 0.044      # handle reach from lip (scales with wider depth)
HANDLE_HALF_W = 0.009   # half-span between handle legs

# Lip centers: the two bottom corners of the triangle.
FRONT_LIP = (-HALF_D, 0.0)   # (x, z)
REAR_LIP = (HALF_D, 0.0)

# Handle configuration: (lip_position, reach_direction, pivot_axis_y_sign)
# Front handle reaches toward -X (reach_dir=-1), rear toward +X (reach_dir=+1).
HANDLE_SPECS = [
    (FRONT_LIP, -1.0),
    (REAR_LIP, +1.0),
]


def _band_profile() -> list[tuple[float, float]]:
    """2D (x, z) center-line of the folded triangular spring-steel band."""
    return [
        (FRONT_LIP[0], LIP_R),
        (-HALF_D * 0.55, APEX_Z * 0.62),
        (APEX_X, APEX_Z),
        (HALF_D * 0.55, APEX_Z * 0.62),
        (REAR_LIP[0], LIP_R),
    ]


def _build_body_mesh():
    """Folded sheet-steel triangular body with two rolled lips, in CadQuery."""
    centerline = _band_profile()
    pts = [(x, z) for (x, z) in centerline]

    n = len(pts)
    outer: list[tuple[float, float]] = []
    inner: list[tuple[float, float]] = []
    for i in range(n):
        px, pz = pts[i]
        x0, z0 = pts[max(i - 1, 0)]
        x1, z1 = pts[min(i + 1, n - 1)]
        tx, tz = (x1 - x0), (z1 - z0)
        tl = math.hypot(tx, tz) or 1.0
        tx, tz = tx / tl, tz / tl
        nx, nz = -tz, tx
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
    band = band.translate((0, HALF_W, 0))

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
    """Center-line points for one U-shaped wire handle (shared geometry helper).

    Authored in the joint frame at the lip center. ``reach_dir`` is +1 for the
    rear handle (reaches toward +X) or -1 for the front handle (reaches -X).
    The closed (folded flat) pose has the loop low against the body.
    """
    ex = reach_dir * HANDLE_LEN
    yw = HANDLE_HALF_W
    z_flat = LIP_R + WIRE_R + 0.0003

    wrap = LIP_R + WIRE_R * 0.2
    hook_x = -reach_dir * wrap * 0.85
    hook_z = -wrap * 0.45

    return [
        (hook_x, +yw, hook_z),
        (reach_dir * wrap, +yw, z_flat),
        (ex * 0.55, +yw, z_flat * 0.9),
        (ex, +yw * 0.7, z_flat * 0.6),
        (ex * 1.02, 0.0, z_flat * 0.55),
        (ex, -yw * 0.7, z_flat * 0.6),
        (ex * 0.55, -yw, z_flat * 0.9),
        (reach_dir * wrap, -yw, z_flat),
        (hook_x, -yw, hook_z),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_mouth_clip")

    steel_orange = Material(name="clip_orange", rgba=(0.86, 0.34, 0.13, 1.0))
    wire_steel = Material(name="wire_steel", rgba=(0.30, 0.30, 0.33, 1.0))

    # --- Root: folded triangular spring-steel body --------------------------
    body = model.part("clip_body")
    body.visual(_build_body_mesh(), material=steel_orange, name="body_shell")

    # --- Two wire lever handles (repeated sub-parts via loop) ---------------
    handles = []
    for i in range(len(HANDLE_SPECS)):
        lip_pos, reach_dir = HANDLE_SPECS[i]
        handle_name = f"handle_{i}"
        loop_name = f"handle_{i}_loop"

        h = model.part(handle_name)
        pts = _handle_points(lip_pos[0], lip_pos[1], reach_dir)
        wire = tube_from_spline_points(
            pts,
            radius=WIRE_R,
            samples_per_segment=14,
            radial_segments=14,
            cap_ends=True,
        )
        h.visual(
            mesh_from_geometry(wire, loop_name),
            material=wire_steel,
            name=loop_name,
        )
        handles.append(h)

        # Uniform joint policy: revolute pivot about the lip Y-axis.
        # Front (reach_dir=-1): axis (0,+1,0) so positive q lifts -X toward +Z.
        # Rear  (reach_dir=+1): axis (0,-1,0) so positive q lifts +X toward +Z.
        axis_y = 1.0 if reach_dir < 0 else -1.0
        model.articulation(
            f"handle_{i}_pivot",
            ArticulationType.REVOLUTE,
            parent=body,
            child=h,
            origin=Origin(xyz=(lip_pos[0], 0.0, lip_pos[1])),
            axis=(0.0, axis_y, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=4.0, lower=0.0, upper=2.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("clip_body")
    handle_0 = object_model.get_part("handle_0")
    handle_1 = object_model.get_part("handle_1")
    pivot_0 = object_model.get_articulation("handle_0_pivot")
    pivot_1 = object_model.get_articulation("handle_1_pivot")

    # --- Mechanism type and axis claims ------------------------------------
    for i, piv in enumerate([pivot_0, pivot_1]):
        ctx.check(
            f"handle_{i} is a revolute lever",
            piv.joint_type == "revolute",
            details=f"got {piv.joint_type}",
        )
        ctx.check(
            f"handle_{i} pivot axis is the lip (Y) axis",
            abs(piv.axis[1]) > 0.99 and abs(piv.axis[0]) < 1e-6,
            details=f"axis={piv.axis}",
        )

    # Body is the sole root; handles hang off it.
    roots = [p.name for p in object_model.root_parts()]
    ctx.check(
        "clip body is the sole root",
        roots == ["clip_body"],
        details=f"roots={roots}",
    )

    # --- Wide-mouth proportion claims --------------------------------------
    body_aabb = ctx.part_world_aabb(body)
    assert body_aabb is not None
    (bx0, by0, bz0), (bx1, by1, bz1) = body_aabb
    body_depth = bx1 - bx0
    body_height = bz1 - bz0

    # The wide-mouth variant must be clearly wider than the parent (0.026 m).
    ctx.check(
        "body has wide-mouth depth (>= 0.038 m)",
        body_depth >= 0.038,
        details=f"depth={body_depth:.4f}",
    )
    ctx.check(
        "body has the folded apex height",
        body_height > 0.018,
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

    # Wide-mouth ratio: depth should be notably larger than height.
    ctx.check(
        "wide-mouth depth-to-height ratio >= 1.4",
        body_height > 0 and body_depth / body_height >= 1.4,
        details=f"ratio={body_depth / body_height:.2f}" if body_height > 0 else "height=0",
    )

    # --- Handle reach claims -----------------------------------------------
    h0_aabb = ctx.part_world_aabb(handle_0)
    h1_aabb = ctx.part_world_aabb(handle_1)
    assert h0_aabb is not None and h1_aabb is not None

    # Front handle (handle_0) reaches past front lip toward -X.
    ctx.check(
        "handle_0 reaches outward past the front lip",
        h0_aabb[0][0] < FRONT_LIP[0] - 0.008,
        details=f"handle_0 x_min={h0_aabb[0][0]:.4f}, lip_x={FRONT_LIP[0]:.4f}",
    )
    # Rear handle (handle_1) reaches past rear lip toward +X.
    ctx.check(
        "handle_1 reaches outward past the rear lip",
        h1_aabb[1][0] > REAR_LIP[0] + 0.008,
        details=f"handle_1 x_max={h1_aabb[1][0]:.4f}, lip_x={REAR_LIP[0]:.4f}",
    )

    # Each handle is captured at its lip barrel.
    ctx.expect_contact(
        handle_0, body,
        elem_a="handle_0_loop", elem_b="body_shell",
        contact_tol=0.0015,
        name="handle_0 threaded through front lip",
    )
    ctx.expect_contact(
        handle_1, body,
        elem_a="handle_1_loop", elem_b="body_shell",
        contact_tol=0.0015,
        name="handle_1 threaded through rear lip",
    )

    # Wire/lip capture overlap allowances.
    ctx.allow_overlap(
        handle_0, body,
        elem_a="handle_0_loop", elem_b="body_shell",
        reason="Wire handle threaded through the rolled lip barrel; small wire/lip nest is the real capture fit.",
    )
    ctx.allow_overlap(
        handle_1, body,
        elem_a="handle_1_loop", elem_b="body_shell",
        reason="Wire handle threaded through the rolled lip barrel; small wire/lip nest is the real capture fit.",
    )

    # --- Decisive motion check: squeezing handle lifts its free end --------
    for i, (h, piv) in enumerate([(handle_0, pivot_0), (handle_1, pivot_1)]):
        rest_aabb = ctx.part_world_aabb(h)
        assert rest_aabb is not None
        rest_top = rest_aabb[1][2]
        with ctx.pose({piv: 1.6}):
            open_aabb = ctx.part_world_aabb(h)
            assert open_aabb is not None
            open_top = open_aabb[1][2]
        ctx.check(
            f"raising handle_{i} lifts its free end upward",
            open_top > rest_top + 0.008,
            details=f"rest_top={rest_top:.4f}, open_top={open_top:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
