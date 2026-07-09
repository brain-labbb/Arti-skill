from __future__ import annotations

# Cylindrical drum playground merry-go-round (orbit spinner variant).
#
# A fixed white square steel post carries a spinning cylindrical drum cage of
# tubular steel hoops (drum ~1.8 m diameter, ~1.5 m tall): a top ring and
# bottom ring of equal radius joined by 12 straight vertical bars around the
# circumference. Two intermediate latitude rings (one sky blue equator ring and
# one bright yellow ring below center) add rigidity and grip. Six radial spoke
# arms at top and bottom connect the collar bearings to the drum rings. The
# vertical bars carry red-and-white candy-stripe paint. Rusty bracket clamps
# join bars to the top and bottom rings. The whole cage spins freely 360
# degrees about the vertical post axis (continuous revolute joint).

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Key dimensions (meters)
# ---------------------------------------------------------------------------
DRUM_R = 0.88  # drum cage radius (~1.76 m diameter)
DRUM_HALF_H = 0.75  # half-height of drum (1.5 m total)
TUBE_R = 0.020  # steel tube radius (0.04 m diameter)
STRIPE_R = 0.0215  # white stripe sleeve radius (slightly proud of red tube)

POST_W = 0.12  # square post width
POST_H = 2.20  # post height
CENTER_Z = 1.10  # drum center height on the post

COLLAR_INNER_R = 0.0895  # collar bore radius (light press onto the journal)
COLLAR_OUTER_R = 0.115  # collar outer radius
COLLAR_HALF_H = 0.070  # collar half height
JOURNAL_R = 0.0905  # round bearing journal radius on the post
JOURNAL_LEN = 0.18

N_VERTICAL_BARS = 12  # vertical bars around the drum circumference
N_STRIPE_SEGMENTS = 8  # alternating red/white bands per bar
N_SPOKE_ARMS = 6  # radial spoke arms at top and bottom

# Drum rings: (name, height above cage center, material key)
# All rings share the same DRUM_R radius (cylindrical drum).
DRUM_RINGS = (
    ("ring_top", DRUM_HALF_H, "sky_blue"),
    ("ring_middle", 0.0, "sky_blue"),
    ("ring_yellow", -0.25, "yellow"),
    ("ring_bottom", -DRUM_HALF_H, "sky_blue"),
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cylindrical_merry_go_round")

    white_paint = model.material("white_paint", rgba=(0.90, 0.90, 0.87, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.24, 0.25, 0.27, 1.0))
    sky_blue = model.material("sky_blue", rgba=(0.27, 0.60, 0.78, 1.0))
    worn_yellow = model.material("worn_yellow", rgba=(0.90, 0.76, 0.12, 1.0))
    candy_red = model.material("candy_red", rgba=(0.76, 0.13, 0.13, 1.0))
    stripe_white = model.material("stripe_white", rgba=(0.92, 0.90, 0.86, 1.0))
    rust = model.material("rust", rgba=(0.45, 0.27, 0.16, 1.0))

    # ------------------------------------------------------------------
    # Root: fixed white square steel post anchored to the ground
    # ------------------------------------------------------------------
    post = model.part("post")
    post.visual(
        Box((0.34, 0.34, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=steel_dark,
        name="base_plate",
    )
    post.visual(
        Box((POST_W, POST_W, POST_H)),
        origin=Origin(xyz=(0.0, 0.0, POST_H / 2.0)),
        material=white_paint,
        name="post_column",
    )
    post.visual(
        Box((0.14, 0.14, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, POST_H + 0.01)),
        material=white_paint,
        name="post_cap",
    )
    # Round bearing journals the cage collars ride on (upper / lower).
    for tag, sign in (("upper", 1.0), ("lower", -1.0)):
        post.visual(
            Cylinder(radius=JOURNAL_R, length=JOURNAL_LEN),
            origin=Origin(xyz=(0.0, 0.0, CENTER_Z + sign * DRUM_HALF_H)),
            material=steel_dark,
            name=f"journal_{tag}",
        )

    # ------------------------------------------------------------------
    # Child: rigid spinning drum cage (local frame at drum center)
    # ------------------------------------------------------------------
    cage = model.part("drum_cage")

    # Collar bearings: hollow sleeves wrapping the post journals.
    collar_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(COLLAR_OUTER_R, -COLLAR_HALF_H), (COLLAR_OUTER_R, COLLAR_HALF_H)],
            [(COLLAR_INNER_R, -COLLAR_HALF_H), (COLLAR_INNER_R, COLLAR_HALF_H)],
            segments=48,
        ),
        "bearing_collar",
    )
    cage.visual(
        collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, DRUM_HALF_H)),
        material=steel_dark,
        name="collar_upper",
    )
    cage.visual(
        collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, -DRUM_HALF_H)),
        material=steel_dark,
        name="collar_lower",
    )

    # Drum rings: horizontal tori all at the same DRUM_R radius.
    for ring_name, height, mat_key in DRUM_RINGS:
        ring_mesh = mesh_from_geometry(
            TorusGeometry(
                radius=DRUM_R, tube=TUBE_R, radial_segments=16, tubular_segments=72
            ),
            ring_name,
        )
        cage.visual(
            ring_mesh,
            origin=Origin(xyz=(0.0, 0.0, height)),
            material=sky_blue if mat_key == "sky_blue" else worn_yellow,
            name=ring_name,
        )

    # Vertical bars: straight tubes running from bottom ring to top ring.
    bar_pts = [
        (DRUM_R, 0.0, -DRUM_HALF_H),
        (DRUM_R, 0.0, DRUM_HALF_H),
    ]
    bar_mesh = mesh_from_geometry(
        tube_from_spline_points(
            bar_pts,
            radius=TUBE_R,
            samples_per_segment=4,
            radial_segments=14,
            cap_ends=True,
        ),
        "vertical_bar",
    )
    for i in range(N_VERTICAL_BARS):
        theta = i * 2.0 * math.pi / N_VERTICAL_BARS
        cage.visual(
            bar_mesh,
            origin=Origin(rpy=(0.0, 0.0, theta)),
            material=candy_red,
            name=f"vertical_bar_{i}",
        )

    # Candy-stripe white sleeves on alternating bands of each vertical bar.
    stripe_length = (2.0 * DRUM_HALF_H) / N_STRIPE_SEGMENTS
    stripe_meshes = []
    for j in range(1, N_STRIPE_SEGMENTS, 2):
        z_start = -DRUM_HALF_H + j * stripe_length
        z_end = z_start + stripe_length
        stripe_pts = [
            (DRUM_R, 0.0, z_start),
            (DRUM_R, 0.0, z_end),
        ]
        stripe_meshes.append(
            mesh_from_geometry(
                tube_from_spline_points(
                    stripe_pts,
                    radius=STRIPE_R,
                    samples_per_segment=2,
                    radial_segments=14,
                    cap_ends=True,
                ),
                f"bar_stripe_{j}",
            )
        )
    for i in range(N_VERTICAL_BARS):
        theta = i * 2.0 * math.pi / N_VERTICAL_BARS
        for s, stripe_mesh in enumerate(stripe_meshes):
            cage.visual(
                stripe_mesh,
                origin=Origin(rpy=(0.0, 0.0, theta)),
                material=stripe_white,
                name=f"bar_stripe_{i}_{s}",
            )

    # Radial spoke arms connecting collar bearings to drum rings.
    # Each spoke runs horizontally from the collar outer surface to the drum
    # ring at the top or bottom of the cage.
    spoke_pts = [
        (COLLAR_OUTER_R, 0.0, 0.0),
        (DRUM_R - TUBE_R, 0.0, 0.0),
    ]
    spoke_mesh = mesh_from_geometry(
        tube_from_spline_points(
            spoke_pts,
            radius=TUBE_R,
            samples_per_segment=4,
            radial_segments=14,
            cap_ends=True,
        ),
        "spoke_arm",
    )
    for i in range(N_SPOKE_ARMS):
        theta = i * 2.0 * math.pi / N_SPOKE_ARMS
        cage.visual(
            spoke_mesh,
            origin=Origin(xyz=(0.0, 0.0, DRUM_HALF_H), rpy=(0.0, 0.0, theta)),
            material=steel_dark,
            name=f"spoke_upper_{i}",
        )
        cage.visual(
            spoke_mesh,
            origin=Origin(xyz=(0.0, 0.0, -DRUM_HALF_H), rpy=(0.0, 0.0, theta)),
            material=steel_dark,
            name=f"spoke_lower_{i}",
        )

    # Rusty clamp brackets where vertical bars meet the top and bottom rings.
    for ring_tag, ring_h in (("top", DRUM_HALF_H), ("bot", -DRUM_HALF_H)):
        for i in range(N_VERTICAL_BARS):
            theta = i * 2.0 * math.pi / N_VERTICAL_BARS
            cage.visual(
                Box((0.055, 0.05, 0.05)),
                origin=Origin(
                    xyz=(
                        DRUM_R * math.cos(theta),
                        DRUM_R * math.sin(theta),
                        ring_h,
                    ),
                    rpy=(0.0, 0.0, theta),
                ),
                material=rust,
                name=f"clamp_{ring_tag}_{i}",
            )

    # ------------------------------------------------------------------
    # Articulation: free 360-degree spin about the vertical post axis
    # ------------------------------------------------------------------
    model.articulation(
        "cage_spin",
        ArticulationType.CONTINUOUS,
        parent=post,
        child=cage,
        origin=Origin(xyz=(0.0, 0.0, CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=200.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    post = object_model.get_part("post")
    cage = object_model.get_part("drum_cage")
    spin = object_model.get_articulation("cage_spin")

    # Joint identity: continuous spin about the vertical post axis.
    ctx.check(
        "cage spin is continuous about vertical axis",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )

    # Cage is concentric with the post.
    ctx.expect_origin_distance(
        cage, post, axes="xy", max_dist=0.002, name="cage centered on post axis"
    )

    # Bearing collars wrap the post journals: shaft-in-bushing fit.
    for tag in ("upper", "lower"):
        ctx.allow_overlap(
            cage,
            post,
            elem_a=f"collar_{tag}",
            elem_b=f"journal_{tag}",
            reason=(
                "The cage bearing collar is intentionally captured on the round "
                "post journal so the spinning drum reads as mounted on its "
                "bearing; the embed is a thin hidden ring inside the collar bore."
            ),
        )
    for tag in ("upper", "lower"):
        ctx.expect_within(
            post,
            cage,
            axes="xy",
            inner_elem=f"journal_{tag}",
            outer_elem=f"collar_{tag}",
            margin=0.0,
            name=f"{tag} journal sits inside its collar bore",
        )
        ctx.expect_contact(
            cage,
            post,
            elem_a=f"collar_{tag}",
            elem_b=f"journal_{tag}",
            contact_tol=0.004,
            name=f"{tag} collar rides its journal",
        )

    # Drum geometry: all rings share the same radius (cylindrical, not spherical).
    top = ctx.part_element_world_aabb(cage, elem="ring_top")
    bot = ctx.part_element_world_aabb(cage, elem="ring_bottom")
    mid = ctx.part_element_world_aabb(cage, elem="ring_middle")
    yel = ctx.part_element_world_aabb(cage, elem="ring_yellow")

    def _width(aabb):
        return aabb[1][0] - aabb[0][0]

    def _center_z(aabb):
        return (aabb[0][2] + aabb[1][2]) / 2.0

    ctx.check(
        "drum is about 1.8 m in diameter",
        top is not None and 1.65 <= _width(top) <= 1.95,
        details=f"top ring width={None if top is None else _width(top):.3f}",
    )
    ctx.check(
        "top and bottom rings have equal radius (cylindrical drum)",
        top is not None
        and bot is not None
        and abs(_width(top) - _width(bot)) < 0.02,
        details=f"top={_width(top):.3f} bot={_width(bot):.3f}",
    )
    ctx.check(
        "all drum rings share the same radius",
        top is not None
        and mid is not None
        and yel is not None
        and bot is not None
        and abs(_width(top) - _width(mid)) < 0.02
        and abs(_width(top) - _width(yel)) < 0.02
        and abs(_width(top) - _width(bot)) < 0.02,
        details=(
            f"top={_width(top):.2f} mid={_width(mid):.2f} "
            f"yel={_width(yel):.2f} bot={_width(bot):.2f}"
        ),
    )
    ctx.check(
        "yellow ring sits below the equator",
        yel is not None
        and mid is not None
        and _center_z(yel) < _center_z(mid) - 0.10,
        details=f"z yel={_center_z(yel):.2f} mid={_center_z(mid):.2f}",
    )

    # Vertical bars span from bottom ring to top ring.
    bar = ctx.part_element_world_aabb(cage, elem="vertical_bar_0")
    ctx.check(
        "vertical bar spans top to bottom ring",
        bar is not None and (bar[1][2] - bar[0][2]) > 1.40,
        details=f"bar z span={None if bar is None else bar[1][2] - bar[0][2]:.3f}",
    )

    # Candy stripes sleeve the vertical bars.
    stripe = ctx.part_element_world_aabb(cage, elem="bar_stripe_0_0")
    ctx.check(
        "candy stripe bands sleeve the vertical bars",
        stripe is not None,
        details="missing stripe element",
    )

    # Spoke arms connect collars to drum rings.
    spoke = ctx.part_element_world_aabb(cage, elem="spoke_upper_0")
    ctx.check(
        "spoke arms connect bearing collars to drum rings",
        spoke is not None,
        details="missing spoke element",
    )

    # Cage clears the ground; post stands ~2.2 m tall and tops out above cage.
    cage_aabb = ctx.part_world_aabb(cage)
    post_aabb = ctx.part_world_aabb(post)
    ctx.check(
        "cage clears the ground",
        cage_aabb is not None and cage_aabb[0][2] > 0.10,
        details=f"cage min z={None if cage_aabb is None else cage_aabb[0][2]:.3f}",
    )
    ctx.check(
        "post is about 2.2 m tall and tops out above the cage",
        post_aabb is not None
        and cage_aabb is not None
        and 2.15 <= post_aabb[1][2] <= 2.30
        and post_aabb[1][2] > cage_aabb[1][2],
        details=f"post top={None if post_aabb is None else post_aabb[1][2]:.3f}",
    )

    # Decisive spin pose: a clamp on the top ring at azimuth 0 swings from +X
    # to +Y when the cage rotates a quarter turn; the cage stays centered.
    before = ctx.part_element_world_aabb(cage, elem="clamp_top_0")
    with ctx.pose({spin: math.pi / 2.0}):
        ctx.expect_origin_distance(
            cage, post, axes="xy", max_dist=0.002, name="spinning cage stays centered"
        )
        after = ctx.part_element_world_aabb(cage, elem="clamp_top_0")

    def _center_xy(aabb):
        return (
            (aabb[0][0] + aabb[1][0]) / 2.0,
            (aabb[0][1] + aabb[1][1]) / 2.0,
        )

    ok = False
    details = "missing clamp element"
    if before is not None and after is not None:
        bx, by = _center_xy(before)
        ax, ay = _center_xy(after)
        ok = bx > 0.70 and abs(by) < 0.10 and abs(ax) < 0.10 and ay > 0.70
        details = f"before=({bx:.2f},{by:.2f}) after=({ax:.2f},{ay:.2f})"
    ctx.check(
        "quarter-turn spin carries the top clamp from +X to +Y", ok, details=details
    )

    return ctx.report()


object_model = build_object_model()
