from __future__ import annotations

# Jade facial massage roller with flat paddle handle.
# Frame: handle axis along +Z (vertical), object centered on x=0, y=0.
#   - Static body = flat paddle stone handle (oval cross-section, wide in X,
#     thin in Y) + two polished metal collars + two U-shaped metal forks
#     (one at the top, one at the bottom).
#   - Top fork (+Z) holds a SMALL polished oval stone roller.
#   - Bottom fork (-Z) holds a LARGER polished oval stone roller.
# Articulations (both CONTINUOUS spin about the roller axle, axle along X):
#   - small_roller : continuous rotation in the top fork
#   - large_roller : continuous rotation in the bottom fork
# Each roller is an oblate ellipsoid (oval) and carries a tiny off-axis marker
# so its spin is unambiguously detectable by AABB spin tests.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    superellipse_profile,
    tube_from_spline_points,
)

# ---- key geometry constants (meters) ----
HANDLE_W = 0.026           # handle width along X (flat paddle, wide direction)
HANDLE_T = 0.007           # handle thickness along Y (flat paddle, thin direction)
HANDLE_HALF = 0.052        # handle half-length along Z  (handle spans ~0.104)
COLLAR_R = 0.0115          # metal collar/ferrule radius
COLLAR_H = 0.012           # metal collar height

# Fork geometry
FORK_ROD_R = 0.0018        # thin metal fork rod radius
TOP_AXLE_Z = 0.094         # z of the small roller axle (top)
BOT_AXLE_Z = -0.094        # z of the large roller axle (bottom)

SMALL_HALF_X = 0.0175      # small roller half-length along axle (X)  -> ~0.035 long
SMALL_RY = 0.0125          # small roller cross radius
LARGE_HALF_X = 0.025       # large roller half-length along axle (X)  -> ~0.050 long
LARGE_RY = 0.018           # large roller cross radius

SMALL_FORK_SPAN = SMALL_HALF_X + 0.004   # half-spacing of the top fork tips
LARGE_FORK_SPAN = LARGE_HALF_X + 0.004   # half-spacing of the bottom fork tips


def _u_fork(z_root: float, z_axle: float, span: float) -> "MeshGeometry":  # noqa: F821
    # A U-shaped metal fork: rises out of the collar end of the handle on the
    # axis, then splits into two arms that reach out to +/-X axle tips.
    # Built as one continuous bent tube down one arm, across, and up the other.
    direction = 1.0 if z_axle > z_root else -1.0
    z_tip = z_axle  # arms terminate at the axle height, holding the roller axle
    # Path: tip(+X) -> up toward root -> across the root yoke -> down -> tip(-X)
    pts = [
        (span, 0.0, z_tip),
        (span, 0.0, z_tip - direction * (z_tip - z_root) * 0.45),
        (span * 0.55, 0.0, z_root + direction * 0.010),
        (0.0, 0.0, z_root),
        (-span * 0.55, 0.0, z_root + direction * 0.010),
        (-span, 0.0, z_tip - direction * (z_tip - z_root) * 0.45),
        (-span, 0.0, z_tip),
    ]
    return tube_from_spline_points(
        pts,
        radius=FORK_ROD_R,
        samples_per_segment=16,
        radial_segments=14,
        cap_ends=True,
    )


def _oval_roller(half_x: float, ry: float, name: str):
    # Oblate ellipsoid (oval gemstone): unit sphere scaled long along the axle
    # (X) and round in cross section (Y/Z). Add a small off-axis marker bump so
    # the spin is detectable and to break any residual symmetry.
    from sdk import mesh_from_geometry  # local import to keep header tidy

    body = SphereGeometry(1.0, width_segments=36, height_segments=24)
    body.scale(half_x, ry, ry)
    # Small marker nub on the +Y rim, off the spin axis.
    marker = SphereGeometry(0.0028, width_segments=12, height_segments=8)
    marker.translate(0.0, ry * 0.96, 0.0)
    body.merge(marker)
    return mesh_from_geometry(body, name)


def build_object_model() -> ArticulatedObject:
    from sdk import mesh_from_geometry

    model = ArticulatedObject(name="jade_facial_roller")

    jade = model.material("marbled_jade", rgba=(0.94, 0.91, 0.80, 1.0))
    metal = model.material("satin_metal", rgba=(0.78, 0.77, 0.74, 1.0))

    # ================= static body =================
    body = model.part("body")

    # Marbled stone handle: flat paddle with oval cross-section (wide in X, thin in Y).
    handle_profile = superellipse_profile(HANDLE_W, HANDLE_T, exponent=2.0, segments=48)
    handle = ExtrudeGeometry(handle_profile, 2.0 * HANDLE_HALF, cap=True, center=True)
    body.visual(mesh_from_geometry(handle, "handle"), material=jade, name="handle")

    # Two polished metal collars/ferrules at each end of the handle.
    top_collar = CylinderGeometry(COLLAR_R, COLLAR_H, radial_segments=40)
    top_collar.translate(0.0, 0.0, HANDLE_HALF - COLLAR_H * 0.25)
    body.visual(mesh_from_geometry(top_collar, "top_collar"), material=metal, name="top_collar")

    bot_collar = CylinderGeometry(COLLAR_R, COLLAR_H, radial_segments=40)
    bot_collar.translate(0.0, 0.0, -(HANDLE_HALF - COLLAR_H * 0.25))
    body.visual(mesh_from_geometry(bot_collar, "bottom_collar"), material=metal, name="bottom_collar")

    # Two U-forks rising from the collar ends.
    top_fork = _u_fork(z_root=HANDLE_HALF + COLLAR_H * 0.4, z_axle=TOP_AXLE_Z, span=SMALL_FORK_SPAN)
    body.visual(mesh_from_geometry(top_fork, "top_fork"), material=metal, name="top_fork")

    bot_fork = _u_fork(
        z_root=-(HANDLE_HALF + COLLAR_H * 0.4), z_axle=BOT_AXLE_Z, span=LARGE_FORK_SPAN
    )
    body.visual(mesh_from_geometry(bot_fork, "bottom_fork"), material=metal, name="bottom_fork")

    body.inertial = Inertial.from_geometry(
        Box((HANDLE_W, HANDLE_T, 2.0 * HANDLE_HALF + 0.06)), mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # ================= small roller (top) =================
    small = model.part("small_roller")
    small.visual(_oval_roller(SMALL_HALF_X, SMALL_RY, "small_roller_stone"), material=jade,
                 name="small_roller_stone")
    # Thin metal axle through the roller, spanning the top fork tips.
    small_axle = CylinderGeometry(FORK_ROD_R * 0.85, 2.0 * SMALL_FORK_SPAN + 0.004,
                                  radial_segments=16).rotate_y(math.pi / 2.0)
    small.visual(mesh_from_geometry(small_axle, "small_axle"), material=metal, name="small_axle")
    small.inertial = Inertial.from_geometry(
        Box((2.0 * SMALL_HALF_X, 2.0 * SMALL_RY, 2.0 * SMALL_RY)), mass=0.02
    )
    model.articulation(
        "small_roller_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=small,
        origin=Origin(xyz=(0.0, 0.0, TOP_AXLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )

    # ================= large roller (bottom) =================
    large = model.part("large_roller")
    large.visual(_oval_roller(LARGE_HALF_X, LARGE_RY, "large_roller_stone"), material=jade,
                 name="large_roller_stone")
    large_axle = CylinderGeometry(FORK_ROD_R * 0.85, 2.0 * LARGE_FORK_SPAN + 0.004,
                                  radial_segments=16).rotate_y(math.pi / 2.0)
    large.visual(mesh_from_geometry(large_axle, "large_axle"), material=metal, name="large_axle")
    large.inertial = Inertial.from_geometry(
        Box((2.0 * LARGE_HALF_X, 2.0 * LARGE_RY, 2.0 * LARGE_RY)), mass=0.045
    )
    model.articulation(
        "large_roller_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=large,
        origin=Origin(xyz=(0.0, 0.0, BOT_AXLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=8.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    small = object_model.get_part("small_roller")
    large = object_model.get_part("large_roller")
    small_spin = object_model.get_articulation("small_roller_spin")
    large_spin = object_model.get_articulation("large_roller_spin")

    # --- rollers are held in the forks: axle ends seat in the fork tips ---
    ctx.allow_overlap(
        small, body,
        elem_a="small_axle", elem_b="top_fork",
        reason="The small roller axle ends are seated through the top fork tips.",
    )
    ctx.allow_overlap(
        large, body,
        elem_a="large_axle", elem_b="bottom_fork",
        reason="The large roller axle ends are seated through the bottom fork tips.",
    )
    ctx.expect_contact(small, body, name="small roller captured by top fork")
    ctx.expect_contact(large, body, name="large roller captured by bottom fork")

    # --- handle sits between the two forks/rollers (top roller above, bottom below) ---
    sp = ctx.part_world_position(small)
    lp = ctx.part_world_position(large)
    bp = ctx.part_world_position(body)
    ctx.check(
        "small roller is at the top, large at the bottom, handle between",
        sp is not None and lp is not None and sp[2] > bp[2] > lp[2],
        details=f"small_z={sp}, body_z={bp}, large_z={lp}",
    )

    # --- flat paddle handle: width (X) >> thickness (Y) ---
    handle_aabb = ctx.part_element_world_aabb(body, elem="handle")
    if handle_aabb is not None:
        hdx = handle_aabb[1][0] - handle_aabb[0][0]
        hdy = handle_aabb[1][1] - handle_aabb[0][1]
        hdz = handle_aabb[1][2] - handle_aabb[0][2]
        ctx.check(
            "handle is a flat paddle: width (X) at least 2x thickness (Y)",
            hdx > 2.0 * hdy,
            details=f"handle_dx={hdx:.4f}, handle_dy={hdy:.4f}",
        )
        ctx.check(
            "handle is longer (Z) than wide (X)",
            hdz > hdx,
            details=f"handle_dx={hdx:.4f}, handle_dz={hdz:.4f}",
        )

    # --- bottom roller is larger than the top roller ---
    small_ext = _ext(ctx.part_element_world_aabb(small, elem="small_roller_stone"))
    large_ext = _ext(ctx.part_element_world_aabb(large, elem="large_roller_stone"))
    small_len = small_ext[0]
    large_len = large_ext[0]
    ctx.check(
        "bottom roller longer than top roller along axle",
        large_len > small_len + 0.005,
        details=f"small_len={small_len}, large_len={large_len}",
    )
    ctx.check(
        "bottom roller wider in cross-section than top roller",
        large_ext[2] > small_ext[2] + 0.003,
        details=f"small_cross={small_ext[2]}, large_cross={large_ext[2]}",
    )

    # --- both rollers spin about their axle (axle along X): a quarter turn
    #     swaps the off-axis marker / oval profile between Y and Z extents. ---
    s0 = _ext(ctx.part_world_aabb(small))
    with ctx.pose({small_spin: math.pi / 2.0}):
        s90 = _ext(ctx.part_world_aabb(small))
    ctx.check(
        "small roller spin changes its YZ silhouette",
        abs(s90[1] - s0[1]) > 0.001 or abs(s90[2] - s0[2]) > 0.001,
        details=f"rest={s0}, quarter={s90}",
    )

    l0 = _ext(ctx.part_world_aabb(large))
    with ctx.pose({large_spin: math.pi / 2.0}):
        l90 = _ext(ctx.part_world_aabb(large))
    ctx.check(
        "large roller spin changes its YZ silhouette",
        abs(l90[1] - l0[1]) > 0.001 or abs(l90[2] - l0[2]) > 0.001,
        details=f"rest={l0}, quarter={l90}",
    )

    return ctx.report()


object_model = build_object_model()
