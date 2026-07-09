from __future__ import annotations

# Facial massage roller with metal handle and two spiky germanium massage balls.
# Frame: handle axis along +Z (vertical), object centered on x=0, y=0.
#   - Static body = marbled stone handle + two polished metal collars +
#     two U-shaped metal forks (one at the top, one at the bottom).
#   - Top fork (+Z) holds a SMALL spiky germanium massage ball.
#   - Bottom fork (-Z) holds a LARGER spiky germanium massage ball.
# Articulations (both CONTINUOUS spin about the roller axle, axle along X):
#   - small_roller_spin : continuous rotation in the top fork
#   - large_roller_spin : continuous rotation in the bottom fork
# Each spiky head is a near-spherical stone studded with a regular field of
# short rounded nubs placed via nested for-i-in-range loops at regular angular
# positions over the surface. The protruding nubs make spin obviously detectable.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    tube_from_spline_points,
)

# ---- key geometry constants (meters) ----
HANDLE_R = 0.0095          # handle radius (slim stone shaft)
HANDLE_HALF = 0.052        # handle half-length along Z  (handle spans ~0.104)
COLLAR_R = 0.0115          # metal collar/ferrule radius
COLLAR_H = 0.012           # metal collar height

# Fork geometry
FORK_ROD_R = 0.0018        # thin metal fork rod radius
TOP_AXLE_Z = 0.094         # z of the small roller axle (top)
BOT_AXLE_Z = -0.094        # z of the large roller axle (bottom)

# Spiky ball dimensions (near-spherical germanium stones)
SMALL_BALL_R = 0.013       # small ball core radius
LARGE_BALL_R = 0.018       # large ball core radius
SMALL_NUB_R = 0.0022       # small nub radius
LARGE_NUB_R = 0.0028       # large nub radius

# Fork tip half-spans (must exceed ball_r + nub protrusion along axle)
SMALL_FORK_SPAN = SMALL_BALL_R + 0.009   # half-spacing of top fork tips
LARGE_FORK_SPAN = LARGE_BALL_R + 0.012   # half-spacing of bottom fork tips


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


def _spiky_ball(ball_r: float, nub_r: float, n_lat: int, n_lon: int, name: str):
    """Build a near-spherical stone ball studded with rounded nubs.

    The core ball is built with SphereGeometry. A nested for-i-in-range loop
    places short rounded nubs at regular angular positions over the surface,
    using the spin axis (X) as the polar axis and skipping the poles where
    the axle passes through.
    """
    from sdk import mesh_from_geometry

    core = SphereGeometry(ball_r, width_segments=36, height_segments=24)

    # Nested loop: latitude bands (i) x longitude positions (j)
    for i in range(n_lat):
        # Polar angle from +X axis; avoid poles (keep theta in ~25°..155°)
        theta = math.pi * (0.14 + 0.72 * (i + 0.5) / n_lat)
        # Stagger alternate latitude rows for a more uniform triangular field
        phi_offset = (math.pi / n_lon) if (i % 2 == 1) else 0.0
        for j in range(n_lon):
            phi = 2.0 * math.pi * j / n_lon + phi_offset
            # Unit direction from center (polar axis = X)
            nx = math.cos(theta)
            ny = math.sin(theta) * math.cos(phi)
            nz = math.sin(theta) * math.sin(phi)
            # Nub center placed slightly above the ball surface so ~70% protrudes
            offset = ball_r + nub_r * 0.35
            cx = offset * nx
            cy = offset * ny
            cz = offset * nz
            nub = SphereGeometry(nub_r, width_segments=10, height_segments=8)
            nub.translate(cx, cy, cz)
            core.merge(nub)

    return mesh_from_geometry(core, name)


def build_object_model() -> ArticulatedObject:
    from sdk import mesh_from_geometry

    model = ArticulatedObject(name="facial_roller_spiky")

    jade = model.material("marbled_jade", rgba=(0.94, 0.91, 0.80, 1.0))
    metal = model.material("satin_metal", rgba=(0.78, 0.77, 0.74, 1.0))
    germanium = model.material("dark_germanium", rgba=(0.32, 0.30, 0.29, 1.0))

    # ================= static body =================
    body = model.part("body")

    # Marbled stone handle: slim cylinder with a slight barrel mid-bulge.
    handle = CylinderGeometry(HANDLE_R, 2.0 * HANDLE_HALF, radial_segments=40)
    bulge = SphereGeometry(1.0, width_segments=36, height_segments=20)
    bulge.scale(HANDLE_R * 1.07, HANDLE_R * 1.07, HANDLE_HALF * 0.62)
    handle.merge(bulge)
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
        Box((0.06, 0.025, 0.20)), mass=0.18, origin=Origin(xyz=(0.0, 0.0, 0.0))
    )

    # ================= spiky roller heads (outer for-i-in-range loop) =================
    # Each head: near-spherical germanium ball with nub field + metal axle.
    # Uniform joint policy: both heads share the same CONTINUOUS spin about X.
    roller_defs = [
        {
            "prefix": "small_roller",
            "ball_r": SMALL_BALL_R,
            "nub_r": SMALL_NUB_R,
            "n_lat": 5,
            "n_lon": 7,
            "axle_z": TOP_AXLE_Z,
            "fork_span": SMALL_FORK_SPAN,
            "mass": 0.02,
            "effort": 0.2,
        },
        {
            "prefix": "large_roller",
            "ball_r": LARGE_BALL_R,
            "nub_r": LARGE_NUB_R,
            "n_lat": 6,
            "n_lon": 9,
            "axle_z": BOT_AXLE_Z,
            "fork_span": LARGE_FORK_SPAN,
            "mass": 0.045,
            "effort": 0.3,
        },
    ]

    for i in range(2):
        d = roller_defs[i]
        roller = model.part(d["prefix"])

        # Spiky germanium massage ball (near-spherical with nub field)
        ball_mesh = _spiky_ball(
            d["ball_r"], d["nub_r"], d["n_lat"], d["n_lon"],
            f"{d['prefix']}_ball",
        )
        roller.visual(ball_mesh, material=germanium, name=f"{d['prefix']}_ball")

        # Thin metal axle through the roller, spanning the fork tips.
        axle = CylinderGeometry(
            FORK_ROD_R * 0.85, 2.0 * d["fork_span"] + 0.004, radial_segments=16
        ).rotate_y(math.pi / 2.0)
        roller.visual(
            mesh_from_geometry(axle, f"{d['prefix']}_axle"),
            material=metal,
            name=f"{d['prefix']}_axle",
        )

        roller.inertial = Inertial.from_geometry(
            Box((2.0 * d["ball_r"], 2.0 * d["ball_r"], 2.0 * d["ball_r"])),
            mass=d["mass"],
        )

        # Continuous spin joint about the axle (X axis), anchored at fork tip face.
        model.articulation(
            f"{d['prefix']}_spin",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=roller,
            origin=Origin(xyz=(0.0, 0.0, d["axle_z"])),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=d["effort"], velocity=8.0),
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
        elem_a="small_roller_axle", elem_b="top_fork",
        reason="The small roller axle ends are seated through the top fork tips.",
    )
    ctx.allow_overlap(
        large, body,
        elem_a="large_roller_axle", elem_b="bottom_fork",
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

    # --- bottom roller is larger than the top roller ---
    small_ext = _ext(ctx.part_element_world_aabb(small, elem="small_roller_ball"))
    large_ext = _ext(ctx.part_element_world_aabb(large, elem="large_roller_ball"))
    # Cross-section diameter (Y or Z extent) includes nub protrusion
    small_cross = max(small_ext[1], small_ext[2])
    large_cross = max(large_ext[1], large_ext[2])
    ctx.check(
        "bottom roller wider in cross-section than top roller",
        large_cross > small_cross + 0.004,
        details=f"small_cross={small_cross:.5f}, large_cross={large_cross:.5f}",
    )

    # --- spiky texture: nubs make the roller extent exceed the bare ball diameter ---
    # The Y extent should be noticeably larger than 2*ball_r due to nub protrusion.
    ctx.check(
        "small roller nubs protrude beyond core ball diameter",
        small_ext[1] > 2.0 * SMALL_BALL_R + SMALL_NUB_R * 0.5,
        details=f"small_y_ext={small_ext[1]:.5f}, expected_min={2.0 * SMALL_BALL_R + SMALL_NUB_R * 0.5:.5f}",
    )
    ctx.check(
        "large roller nubs protrude beyond core ball diameter",
        large_ext[1] > 2.0 * LARGE_BALL_R + LARGE_NUB_R * 0.5,
        details=f"large_y_ext={large_ext[1]:.5f}, expected_min={2.0 * LARGE_BALL_R + LARGE_NUB_R * 0.5:.5f}",
    )

    # --- both spiky rollers spin about their axle (X): a quarter turn
    #     swaps nub positions between Y and Z, changing the AABB silhouette.
    #     The nub field is nearly uniform, so the change is small but real. ---
    s0 = _ext(ctx.part_world_aabb(small))
    with ctx.pose({small_spin: math.pi / 2.0}):
        s90 = _ext(ctx.part_world_aabb(small))
    ctx.check(
        "small spiky roller spin changes its YZ silhouette",
        abs(s90[1] - s0[1]) > 0.0001 or abs(s90[2] - s0[2]) > 0.0001,
        details=f"rest={s0}, quarter={s90}",
    )

    l0 = _ext(ctx.part_world_aabb(large))
    with ctx.pose({large_spin: math.pi / 2.0}):
        l90 = _ext(ctx.part_world_aabb(large))
    ctx.check(
        "large spiky roller spin changes its YZ silhouette",
        abs(l90[1] - l0[1]) > 0.0001 or abs(l90[2] - l0[2]) > 0.0001,
        details=f"rest={l0}, quarter={l90}",
    )

    # --- near-spherical: ball cross-sections are close to equal on Y and Z ---
    ctx.check(
        "small roller ball is near-spherical (Y~Z)",
        abs(small_ext[1] - small_ext[2]) < 0.004,
        details=f"small_y={small_ext[1]:.5f}, small_z={small_ext[2]:.5f}",
    )
    ctx.check(
        "large roller ball is near-spherical (Y~Z)",
        abs(large_ext[1] - large_ext[2]) < 0.005,
        details=f"large_y={large_ext[1]:.5f}, large_z={large_ext[2]:.5f}",
    )

    return ctx.report()


object_model = build_object_model()
