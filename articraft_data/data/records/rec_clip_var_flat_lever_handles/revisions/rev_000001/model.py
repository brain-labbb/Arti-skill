from __future__ import annotations

# Articraft model: a binder clip (foldback clip) with paddle lever handles.
#
# Variant of the parent binder clip: flat stamped sheet-metal paddle levers
# replace the thin U-shaped wire handles.  Each paddle is a thin flat plate
# with a rolled / hooked root that captures the lip barrel, pivoting about
# the lip axis exactly like the wire handle did.
#
# Real object:
#   - A single piece of folded spring steel forms a roughly triangular-prism
#     body, painted glossy orange.  The wide flat bottom is the clamping
#     mouth; the two sloped faces meet at a folded ridge on top.  The two
#     bottom front/back edges are rolled into small lips (barrels).
#   - Two stamped steel paddle levers hook onto those two rolled lips.  Each
#     paddle pivots about its lip axis: squeeze the paddles upward / together
#     to spring the mouth open; they fold back flat against the body at rest.
#
# Articulation:
#   - Root: clip_body (the folded triangular spring-steel body).
#   - handle_0, handle_1: each a REVOLUTE paddle lever pivoting about its
#     rolled lip axis (the clip Y / width axis).  Positive q lifts the free
#     end of the paddle up and away from the body (the squeeze gesture).
#
# Frame convention:
#   - +Y is the clip width and the lip / pivot axis.
#   - +X is depth (front-to-back); the front lip is at -X, the rear lip at +X.
#   - +Z is up; the clamping mouth (bottom of the triangle) sits on z = 0
#     and the folded apex is the highest point.

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
)

# ---------------------------------------------------------------------------
# Dimensions (meters) -- a standard ~32 mm "medium" binder clip.
# ---------------------------------------------------------------------------
WIDTH = 0.032          # clip width along Y (the lip / pivot axis span)
HALF_W = WIDTH / 2.0

DEPTH = 0.026          # front-to-back span of the clamping mouth (X)
HALF_D = DEPTH / 2.0
APEX_Z = 0.018         # height of the folded ridge above the mouth (Z)
APEX_X = 0.0           # ridge sits centered front-to-back

SHEET_T = 0.0010       # spring-steel sheet thickness
LIP_R = 0.0018         # outer radius of each rolled lip (barrel)
LIP_INNER_R = 0.0011   # hollow inner radius of each rolled lip

# Paddle lever dimensions (stamped sheet metal)
PLATE_T = 0.0008        # paddle plate thickness
PADDLE_W = 0.022        # paddle blade width (along Y / lip axis)
PADDLE_L = 0.024        # paddle blade length from hook to tip
HOOK_INNER_R = LIP_R + 0.0002   # hook bore radius (clearance fit on barrel)
HOOK_OUTER_R = HOOK_INNER_R + PLATE_T
HOOK_GAP_HALF_ANGLE = 50.0  # degrees: half the opening gap of the C-hook

# Lip centers: the two bottom corners of the triangle.
FRONT_LIP = (-HALF_D, 0.0)   # (x, z)
REAR_LIP  = (HALF_D, 0.0)

# Handle configurations: two paddle levers, one per lip.
# Each entry: lip position, reach direction, and joint axis for positive squeeze.
HANDLE_CONFIGS = [
    {"lip": FRONT_LIP, "reach_dir": -1.0, "axis": (0.0,  1.0, 0.0)},
    {"lip": REAR_LIP,  "reach_dir":  1.0, "axis": (0.0, -1.0, 0.0)},
]


# ---------------------------------------------------------------------------
# Body geometry (identical to parent)
# ---------------------------------------------------------------------------

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
    return mesh_from_cadquery(body, "clip_body",
                              tolerance=0.0004, angular_tolerance=0.2)


# ---------------------------------------------------------------------------
# Paddle lever geometry (variant)
# ---------------------------------------------------------------------------

def _hook_profile(reach_dir: float,
                  outer_r: float = HOOK_OUTER_R,
                  inner_r: float = HOOK_INNER_R,
                  gap_half_deg: float = HOOK_GAP_HALF_ANGLE,
                  n: int = 28) -> list[tuple[float, float]]:
    """C-shaped annular sector in (x, z) for the rolled hook root.

    The gap faces away from the blade so the hook captures the barrel:
    - reach_dir < 0 (front): opening at +X, blade toward -X
    - reach_dir > 0 (rear):  opening at -X, blade toward +X
    """
    opening = 0.0 if reach_dir < 0 else 180.0
    a_start = math.radians(opening + gap_half_deg)
    a_end = math.radians(opening + 360.0 - gap_half_deg)

    pts: list[tuple[float, float]] = []
    # Outer arc (counter-clockwise through the solid sector)
    for i in range(n + 1):
        a = a_start + (a_end - a_start) * i / n
        pts.append((outer_r * math.cos(a), outer_r * math.sin(a)))
    # Inner arc (clockwise back to start)
    for i in range(n + 1):
        a = a_end - (a_end - a_start) * i / n
        pts.append((inner_r * math.cos(a), inner_r * math.sin(a)))
    return pts


def _build_paddle_cq(reach_dir: float):
    """Flat sheet-metal paddle lever with rolled hook root (CadQuery).

    Authored in the lip-local frame: origin at lip center, Y along the lip
    axis.  ``reach_dir`` = -1 for front (blade toward -X), +1 for rear
    (blade toward +X).
    """
    half_w = PADDLE_W / 2.0

    # -- Hook: C-shaped annular sector extruded along Y ---------------------
    profile = _hook_profile(reach_dir)
    hook = (
        cq.Workplane("XZ")
        .polyline(profile)
        .close()
        .extrude(PADDLE_W)
    )
    # Workplane("XZ") extrudes along -Y; shift to center on Y = 0.
    hook = hook.translate((0, half_w, 0))

    # -- Blade: flat plate extending from the hook outward ------------------
    # Overlap the blade slightly into the hook wall for a clean boolean union.
    blade_overlap = PLATE_T
    blade_total_l = PADDLE_L + blade_overlap

    if reach_dir < 0:
        blade_cx = -(HOOK_OUTER_R + PADDLE_L / 2.0 - blade_overlap / 2.0)
    else:
        blade_cx = HOOK_OUTER_R + PADDLE_L / 2.0 - blade_overlap / 2.0

    blade = (
        cq.Workplane("XY")
        .box(blade_total_l, PADDLE_W, PLATE_T)
        .translate((blade_cx, 0, 0))
    )

    paddle = hook.union(blade)

    # -- Tip rounding: fillet the outer blade corners for a finished look ---
    # Select vertical edges (parallel to Z) farthest from the hook center.
    try:
        paddle = paddle.edges("|Z").fillet(PLATE_T * 1.5)
    except Exception:
        pass  # fillet is cosmetic; skip if geometry resists

    return paddle


# ---------------------------------------------------------------------------
# Object model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="binder_clip_paddle")

    steel_orange = Material(name="clip_orange", rgba=(0.86, 0.34, 0.13, 1.0))
    paddle_steel = Material(name="paddle_steel", rgba=(0.55, 0.55, 0.58, 1.0))

    # --- Root: folded triangular spring-steel body -------------------------
    body = model.part("clip_body")
    body.visual(_build_body_mesh(),
                material=steel_orange, name="body_shell")

    # --- Two paddle lever handles (shared helper, loop, uniform joints) ----
    for i, cfg in enumerate(HANDLE_CONFIGS):
        name = f"handle_{i}"
        handle = model.part(name)

        paddle_cq = _build_paddle_cq(cfg["reach_dir"])
        handle.visual(
            mesh_from_cadquery(paddle_cq, f"{name}_paddle",
                               tolerance=0.0004, angular_tolerance=0.2),
            material=paddle_steel,
            name=f"{name}_paddle",
        )

        model.articulation(
            f"{name}_pivot",
            ArticulationType.REVOLUTE,
            parent=body,
            child=handle,
            origin=Origin(xyz=(cfg["lip"][0], 0.0, cfg["lip"][1])),
            axis=cfg["axis"],
            motion_limits=MotionLimits(
                effort=0.4, velocity=4.0, lower=0.0, upper=2.0,
            ),
        )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("clip_body")
    handles = [object_model.get_part(f"handle_{i}") for i in range(2)]
    pivots  = [object_model.get_articulation(f"handle_{i}_pivot") for i in range(2)]

    # --- Mechanism type and axis claims ------------------------------------
    for i in range(2):
        ctx.check(
            f"handle_{i} is a revolute lever",
            pivots[i].joint_type == "revolute",
            details=f"got {pivots[i].joint_type}",
        )
        ctx.check(
            f"handle_{i} pivot axis is the lip (Y) axis",
            abs(pivots[i].axis[1]) > 0.99 and abs(pivots[i].axis[0]) < 1e-6,
            details=f"axis={pivots[i].axis}",
        )

    # Body is the single root; handles hang off it.
    roots = [p.name for p in object_model.root_parts()]
    ctx.check(
        "clip body is the sole root",
        roots == ["clip_body"],
        details=f"roots={roots}",
    )

    # --- Body proportion claims --------------------------------------------
    body_aabb = ctx.part_world_aabb(body)
    assert body_aabb is not None
    (bx0, by0, bz0), (bx1, by1, bz1) = body_aabb
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

    # --- Paddle geometry claims (flat stamped lever, not wire loop) --------
    for i in range(2):
        h_aabb = ctx.part_world_aabb(handles[i])
        assert h_aabb is not None
        h_dy = h_aabb[1][1] - h_aabb[0][1]   # Y span
        h_dz = h_aabb[1][2] - h_aabb[0][2]   # Z span (should be thin)

        ctx.check(
            f"handle_{i} paddle is wide in Y (> 0.015 m)",
            h_dy > 0.015,
            details=f"dy={h_dy:.4f}",
        )
        ctx.check(
            f"handle_{i} paddle is thin in Z (< 0.008 m)",
            h_dz < 0.008,
            details=f"dz={h_dz:.4f}",
        )

    # Handle_0 reaches outward past the front lip (-X).
    h0_aabb = ctx.part_world_aabb(handles[0])
    assert h0_aabb is not None
    ctx.check(
        "handle_0 paddle reaches past the front lip",
        h0_aabb[0][0] < FRONT_LIP[0] - 0.005,
        details=f"x_min={h0_aabb[0][0]:.4f}, lip_x={FRONT_LIP[0]:.4f}",
    )

    # Handle_1 reaches outward past the rear lip (+X).
    h1_aabb = ctx.part_world_aabb(handles[1])
    assert h1_aabb is not None
    ctx.check(
        "handle_1 paddle reaches past the rear lip",
        h1_aabb[1][0] > REAR_LIP[0] + 0.005,
        details=f"x_max={h1_aabb[1][0]:.4f}, lip_x={REAR_LIP[0]:.4f}",
    )

    # Each paddle hook captures its lip barrel (contact at the hook root).
    for i in range(2):
        ctx.expect_contact(
            handles[i], body,
            elem_a=f"handle_{i}_paddle", elem_b="body_shell",
            contact_tol=0.0015,
            name=f"handle_{i} hook captures the lip barrel",
        )

    # The hook root is an intentional capture nest on the barrel.
    for i in range(2):
        ctx.allow_overlap(
            handles[i], body,
            elem_a=f"handle_{i}_paddle", elem_b="body_shell",
            reason=(
                "The paddle hook root wraps around the rolled lip barrel; "
                "a small hook/lip nest is the real capture fit."
            ),
        )

    # --- Decisive motion check: squeezing the paddle lifts its free end ----
    for i in range(2):
        rest_aabb = ctx.part_world_aabb(handles[i])
        assert rest_aabb is not None
        rest_top = rest_aabb[1][2]

        with ctx.pose({pivots[i]: 1.6}):
            open_aabb = ctx.part_world_aabb(handles[i])
            assert open_aabb is not None
            open_top = open_aabb[1][2]

        ctx.check(
            f"raising handle_{i} lifts its free end upward",
            open_top > rest_top + 0.008,
            details=f"rest_top={rest_top:.4f}, open_top={open_top:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
