from __future__ import annotations

# Jade facial massage roller – ridged-barrel variant.
# Frame: handle axis along +Z (vertical), object centered on x=0, y=0.
#   - Static body = marbled stone handle + two polished metal collars +
#     two U-shaped metal forks (one at the top, one at the bottom).
#   - Top fork (+Z) holds a SMALL ridged jade barrel roller.
#   - Bottom fork (-Z) holds a LARGER ridged jade barrel roller.
# Each roller is a lathe-revolved barrel with regular circumferential grooves
# and ridges running around the spin axis (the ribbed massage variant).
# Articulations (both CONTINUOUS spin about the roller axle, axle along X):
#   - small_roller_spin : continuous rotation in the top fork
#   - large_roller_spin : continuous rotation in the bottom fork
# Each ridged head carries a tiny off-axis marker nub so its spin is
# unambiguously detectable by AABB spin tests.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
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


def _ridged_roller(half_x: float, ry: float, num_ridges: int, name: str):
    """Build a ridged barrel roller via LatheGeometry.

    The profile is revolved around the lathe Z-axis to produce a barrel with
    regular circumferential grooves and ridges.  The result is then rotated
    so the spin axis aligns with X (the roller axle direction).

    A small off-axis marker nub is merged in so that spin is detectable via
    AABB changes under rotation.
    """
    ridge_depth = ry * 0.16           # groove depth as fraction of barrel radius
    n_samples = num_ridges * 14 + 1   # profile resolution (smooth waves)

    # Wavy profile: radius oscillates between ry (ridge crest) and ry-ridge_depth
    # (groove bottom).  End caps are closed via center points.
    profile = [(0.0, -half_x)]         # bottom cap center
    for j in range(n_samples):
        t = j / (n_samples - 1)        # 0..1 along the barrel length
        z = -half_x + t * 2.0 * half_x
        # Cosine wave: ridge crests where cos=1, groove bottoms where cos=-1
        r = ry - ridge_depth * 0.5 * (1.0 - math.cos(2.0 * math.pi * num_ridges * t))
        profile.append((max(r, 0.001), z))
    profile.append((0.0, half_x))      # top cap center

    barrel = LatheGeometry(profile, segments=48, closed=True)
    # Rotate from lathe Z-axis to the roller axle along X
    barrel.rotate_y(math.pi / 2.0)

    # Off-axis marker nub on the +Y rim for spin detectability
    marker = SphereGeometry(0.0028, width_segments=12, height_segments=8)
    marker.translate(0.0, ry * 0.92, 0.0)
    barrel.merge(marker)

    return mesh_from_geometry(barrel, name)


# Roller head configurations – differ only in size.
ROLLER_CONFIGS = [
    {
        "name": "small",
        "half_x": SMALL_HALF_X,
        "ry": SMALL_RY,
        "num_ridges": 8,
        "axle_z": TOP_AXLE_Z,
        "fork_span": SMALL_FORK_SPAN,
        "mass": 0.02,
        "effort": 0.2,
    },
    {
        "name": "large",
        "half_x": LARGE_HALF_X,
        "ry": LARGE_RY,
        "num_ridges": 10,
        "axle_z": BOT_AXLE_Z,
        "fork_span": LARGE_FORK_SPAN,
        "mass": 0.045,
        "effort": 0.3,
    },
]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="jade_facial_roller")

    jade = model.material("marbled_jade", rgba=(0.94, 0.91, 0.80, 1.0))
    metal = model.material("satin_metal", rgba=(0.78, 0.77, 0.74, 1.0))

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

    # ================= ridged roller heads (loop) =================
    for i, cfg in enumerate(ROLLER_CONFIGS):
        roller_part = model.part(f"{cfg['name']}_roller")

        # Ridged stone barrel
        stone_name = f"{cfg['name']}_roller_stone"
        roller_part.visual(
            _ridged_roller(cfg["half_x"], cfg["ry"], cfg["num_ridges"], stone_name),
            material=jade,
            name=stone_name,
        )

        # Thin metal axle through the roller, spanning the fork tips
        axle_name = f"{cfg['name']}_axle"
        axle_geom = CylinderGeometry(
            FORK_ROD_R * 0.85, 2.0 * cfg["fork_span"] + 0.004, radial_segments=16
        ).rotate_y(math.pi / 2.0)
        roller_part.visual(
            mesh_from_geometry(axle_geom, axle_name), material=metal, name=axle_name
        )

        roller_part.inertial = Inertial.from_geometry(
            Box((2.0 * cfg["half_x"], 2.0 * cfg["ry"], 2.0 * cfg["ry"])),
            mass=cfg["mass"],
        )

        model.articulation(
            f"{cfg['name']}_roller_spin",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=roller_part,
            origin=Origin(xyz=(0.0, 0.0, cfg["axle_z"])),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=cfg["effort"], velocity=8.0),
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

    # --- both ridged rollers are barrel-shaped: Y and Z extents are approximately
    #     equal (circular cross-section with circumferential ridges) ---
    for roller, stone_name, label in [
        (small, "small_roller_stone", "small"),
        (large, "large_roller_stone", "large"),
    ]:
        ext = _ext(ctx.part_element_world_aabb(roller, elem=stone_name))
        yz_ratio = ext[1] / max(ext[2], 1e-6)
        ctx.check(
            f"{label} ridged roller has barrel cross-section (Y≈Z)",
            0.75 < yz_ratio < 1.35,
            details=f"ext={ext}, Y/Z={yz_ratio:.3f}",
        )

    # --- both ridged rollers spin about their axle (axle along X): a quarter turn
    #     moves the off-axis marker nub between Y and Z, changing the silhouette. ---
    s0 = _ext(ctx.part_world_aabb(small))
    with ctx.pose({small_spin: math.pi / 2.0}):
        s90 = _ext(ctx.part_world_aabb(small))
    ctx.check(
        "small ridged roller spin changes its YZ silhouette",
        abs(s90[1] - s0[1]) > 0.001 or abs(s90[2] - s0[2]) > 0.001,
        details=f"rest={s0}, quarter={s90}",
    )

    l0 = _ext(ctx.part_world_aabb(large))
    with ctx.pose({large_spin: math.pi / 2.0}):
        l90 = _ext(ctx.part_world_aabb(large))
    ctx.check(
        "large ridged roller spin changes its YZ silhouette",
        abs(l90[1] - l0[1]) > 0.001 or abs(l90[2] - l0[2]) > 0.001,
        details=f"rest={l0}, quarter={l90}",
    )

    return ctx.report()


object_model = build_object_model()
