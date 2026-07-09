from __future__ import annotations

# Jade facial massage roller with flat blade yokes.
# Frame: handle axis along +Z (vertical), object centered on x=0, y=0.
#   - Static body = marbled stone handle + two polished metal collars +
#     two flat blade yokes (one at the top, one at the bottom).
#   - Top yoke (+Z) holds a SMALL polished oval stone roller.
#   - Bottom yoke (-Z) holds a LARGER polished oval stone roller.
# Each blade yoke is a stamped-sheet flat metal plate: a narrow central stem
# rises from the collar and splits into two flat blade arms that angle outward
# to the axle tips, with small cylindrical bosses at each arm tip representing
# the reinforced axle bore.
# Articulations (both CONTINUOUS spin about the roller axle, axle along X):
#   - small_roller_spin : continuous rotation in the top yoke
#   - large_roller_spin : continuous rotation in the bottom yoke
# Each roller is an oblate ellipsoid (oval) and carries a tiny off-axis marker
# so its spin is unambiguously detectable by AABB spin tests.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
)

# ---- key geometry constants (meters) ----
HANDLE_R = 0.0095          # handle radius (slim stone shaft)
HANDLE_HALF = 0.052        # handle half-length along Z  (handle spans ~0.104)
COLLAR_R = 0.0115          # metal collar/ferrule radius
COLLAR_H = 0.012           # metal collar height

TOP_AXLE_Z = 0.094         # z of the small roller axle (top)
BOT_AXLE_Z = -0.094        # z of the large roller axle (bottom)

SMALL_HALF_X = 0.0175      # small roller half-length along axle (X)
SMALL_RY = 0.0125          # small roller cross radius
LARGE_HALF_X = 0.025       # large roller half-length along axle (X)
LARGE_RY = 0.018           # large roller cross radius

SMALL_FORK_SPAN = SMALL_HALF_X + 0.004   # half-spacing of the top yoke arm tips
LARGE_FORK_SPAN = LARGE_HALF_X + 0.004   # half-spacing of the bottom yoke arm tips

# ---- blade yoke plate dimensions ----
YOKE_PLATE_THICK = 0.0020   # flat plate thickness in Y
YOKE_STEM_HW = 0.003        # stem half-width in X (6 mm wide stem)
YOKE_ARM_W = 0.005          # arm blade width perpendicular to arm length
YOKE_SPLIT_FRAC = 0.40      # stem occupies this fraction of total yoke height
AXLE_ROD_R = 0.00153        # thin axle rod radius


def _blade_yoke(z_root: float, z_axle: float, span: float):
    """Build a flat stamped-sheet blade yoke.

    A narrow central stem rises from the collar end and splits into two flat
    blade arms that angle outward to the axle positions.  The full shape is
    built as one 2D XZ profile extruded along Y, giving a single connected
    mesh with no internal disconnected islands.
    """
    from sdk import ExtrudeGeometry, mesh_from_geometry

    direction = 1.0 if z_axle > z_root else -1.0
    total_h = abs(z_axle - z_root)
    z_split = z_root + direction * total_h * YOKE_SPLIT_FRAC

    sw = YOKE_STEM_HW      # stem half-width
    aw = YOKE_ARM_W * 0.5   # arm half-width (perpendicular to arm)

    # Build a symmetric Y-shaped profile in XZ (closed polygon, CCW).
    # The profile traces: stem bottom -> stem left -> left arm -> tip -> back
    # to split -> right arm -> stem right -> back to start.
    # We add boss circles at the arm tips for the axle bores.

    # Arm outer edge points (perpendicular offset from arm centerline)
    dx_l, dz_l = -span, z_axle - z_split
    arm_len = math.sqrt(dx_l * dx_l + dz_l * dz_l)
    nx, nz = -dz_l / arm_len, dx_l / arm_len  # perpendicular unit vector

    # Profile points (XZ, going CCW when viewed from +Y)
    # Start at stem bottom-center-right, go up stem, out left arm, to tip,
    # back to split, out right arm, to tip, back to split, down stem.
    pts = [
        # Stem bottom-right
        (sw, z_root),
        # Stem top-right (at split, right side)
        (sw, z_split),
        # Right arm outer edge (going outward from split to tip)
        (span + aw * nx, z_axle + aw * nz),
        # Right arm tip (end cap)
        (span, z_axle + aw * 0.5 * direction),
        (span - aw * nx, z_axle - aw * nz),
        # Back to split center
        (aw * 0.3, z_split + direction * aw * 0.5),
        # Left arm inner edge going outward
        (-aw * 0.3, z_split + direction * aw * 0.5),
        # Left arm outer edge at tip
        (-span + aw * nx, z_axle - aw * nz),
        # Left arm tip
        (-span, z_axle + aw * 0.5 * direction),
        (-span - aw * nx, z_axle + aw * nz),
        # Back to split left side
        (-sw, z_split),
        # Stem bottom-left
        (-sw, z_root),
    ]

    # Extrude the XZ profile along Y by the plate thickness.
    # ExtrudeGeometry expects (x, y) profile points and extrudes along Z,
    # so we map XZ -> XY and then rotate the result.
    xy_profile = [(p[0], p[1]) for p in pts]
    yoke = ExtrudeGeometry(xy_profile, YOKE_PLATE_THICK, cap=True, center=True)
    # ExtrudeGeometry with center=True extrudes symmetrically around z=0.
    # Our profile is in XY; we need it in XZ. Rotate 90° around X.
    yoke.rotate_x(math.pi / 2.0)
    return yoke


def _oval_roller(half_x: float, ry: float, name: str):
    """Oblate ellipsoid gemstone roller with an off-axis spin marker."""
    from sdk import mesh_from_geometry

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

    body.inertial = Inertial.from_geometry(
        Box((0.06, 0.025, 0.20)), mass=0.18, origin=Origin(xyz=(0.0, 0.0, 0.0))
    )

    # ================= yokes + rollers (uniform for-loop) =================
    # Each entry defines one end of the roller: yoke visual on the body,
    # roller part with stone + axle, and a CONTINUOUS spin articulation.
    # The two ends differ only in span and arm length.
    yoke_roller_specs = [
        {
            "prefix": "small",
            "yoke_name": "top_yoke",
            "z_root": HANDLE_HALF - COLLAR_H * 0.25,
            "z_axle": TOP_AXLE_Z,
            "span": SMALL_FORK_SPAN,
            "half_x": SMALL_HALF_X,
            "ry": SMALL_RY,
            "mass": 0.02,
            "effort": 0.2,
        },
        {
            "prefix": "large",
            "yoke_name": "bottom_yoke",
            "z_root": -(HANDLE_HALF - COLLAR_H * 0.25),
            "z_axle": BOT_AXLE_Z,
            "span": LARGE_FORK_SPAN,
            "half_x": LARGE_HALF_X,
            "ry": LARGE_RY,
            "mass": 0.045,
            "effort": 0.3,
        },
    ]

    for i in range(2):
        spec = yoke_roller_specs[i]
        prefix = spec["prefix"]

        # -- flat blade yoke (body visual) --
        yoke_geom = _blade_yoke(spec["z_root"], spec["z_axle"], spec["span"])
        body.visual(
            mesh_from_geometry(yoke_geom, spec["yoke_name"]),
            material=metal,
            name=spec["yoke_name"],
        )

        # -- roller part --
        roller = model.part(f"{prefix}_roller")
        roller.visual(
            _oval_roller(spec["half_x"], spec["ry"], f"{prefix}_roller_stone"),
            material=jade,
            name=f"{prefix}_roller_stone",
        )

        # Thin metal axle through the roller, spanning the yoke arm tips.
        axle = CylinderGeometry(
            AXLE_ROD_R, 2.0 * spec["span"] + 0.004, radial_segments=16
        ).rotate_y(math.pi / 2.0)
        roller.visual(
            mesh_from_geometry(axle, f"{prefix}_axle"),
            material=metal,
            name=f"{prefix}_axle",
        )

        roller.inertial = Inertial.from_geometry(
            Box((2.0 * spec["half_x"], 2.0 * spec["ry"], 2.0 * spec["ry"])),
            mass=spec["mass"],
        )

        # Continuous spin about the axle (X axis), origin at the blade-tip
        # contact face where the yoke arm tips cradle the axle.
        model.articulation(
            f"{prefix}_roller_spin",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=roller,
            origin=Origin(xyz=(0.0, 0.0, spec["z_axle"])),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=spec["effort"], velocity=8.0),
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

    # --- rollers captured in the blade yokes: axle seated through arm tips ---
    ctx.allow_overlap(
        small, body,
        elem_a="small_axle", elem_b="top_yoke",
        reason="The small roller axle ends are seated through the top blade yoke arm-tip bores.",
    )
    ctx.allow_overlap(
        large, body,
        elem_a="large_axle", elem_b="bottom_yoke",
        reason="The large roller axle ends are seated through the bottom blade yoke arm-tip bores.",
    )
    ctx.expect_contact(small, body, name="small roller captured by top yoke")
    ctx.expect_contact(large, body, name="large roller captured by bottom yoke")

    # --- blade yokes are flat plates: Y extent much less than X span ---
    top_yoke_ext = _ext(ctx.part_element_world_aabb(body, elem="top_yoke"))
    bot_yoke_ext = _ext(ctx.part_element_world_aabb(body, elem="bottom_yoke"))

    ctx.check(
        "top blade yoke is a flat plate (Y thickness < 40% of X span)",
        top_yoke_ext[1] < top_yoke_ext[0] * 0.40,
        details=f"top_yoke dx={top_yoke_ext[0]:.4f} dy={top_yoke_ext[1]:.4f} dz={top_yoke_ext[2]:.4f}",
    )
    ctx.check(
        "bottom blade yoke is a flat plate (Y thickness < 40% of X span)",
        bot_yoke_ext[1] < bot_yoke_ext[0] * 0.40,
        details=f"bot_yoke dx={bot_yoke_ext[0]:.4f} dy={bot_yoke_ext[1]:.4f} dz={bot_yoke_ext[2]:.4f}",
    )

    # --- blade yoke X span reaches the axle positions ---
    ctx.check(
        "top yoke X span covers small roller axle span",
        top_yoke_ext[0] > 2.0 * SMALL_FORK_SPAN * 0.90,
        details=f"yoke_dx={top_yoke_ext[0]:.4f}, need>={2.0 * SMALL_FORK_SPAN * 0.90:.4f}",
    )
    ctx.check(
        "bottom yoke X span covers large roller axle span",
        bot_yoke_ext[0] > 2.0 * LARGE_FORK_SPAN * 0.90,
        details=f"yoke_dx={bot_yoke_ext[0]:.4f}, need>={2.0 * LARGE_FORK_SPAN * 0.90:.4f}",
    )

    # --- handle sits between the two yokes/rollers ---
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
