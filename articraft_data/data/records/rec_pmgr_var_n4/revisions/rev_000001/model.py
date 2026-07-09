from __future__ import annotations

# Spherical playground merry-go-round (orbit spinner).
#
# A fixed white square steel post carries a spinning spherical cage of tubular
# steel hoops (sphere ~1.8 m diameter): three sky-blue latitude rings (largest
# at the equator), one bright yellow ring near the lower-middle, and four
# full vertical meridian hoop planes (eight half-arcs) painted in red-and-white
# candy stripes. The meridian arcs land on upper and lower collar bearings
# that ride round journals on the post. The whole cage spins freely 360
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
SPHERE_R = 0.90  # hoop sphere radius (1.8 m diameter)
TUBE_R = 0.020  # steel tube radius (0.04 m diameter)
STRIPE_R = 0.0215  # white stripe sleeve radius (slightly proud of red tube)

POST_W = 0.12  # square post width
POST_H = 2.20  # post height
CENTER_Z = 1.10  # sphere center height on the post

COLLAR_INNER_R = 0.0895  # collar bore radius (light press onto the journal)
COLLAR_OUTER_R = 0.115  # collar outer radius
COLLAR_HALF_H = 0.070  # collar half height
JOURNAL_R = 0.0905  # round bearing journal radius on the post
JOURNAL_LEN = 0.18

# Meridian arcs start/end where their centerline meets the collar wall.
ARC_PHI0 = math.asin(COLLAR_OUTER_R / SPHERE_R)  # polar angle of arc ends
COLLAR_Z = SPHERE_R * math.cos(ARC_PHI0)  # collar centers (local, +/-)

N_MERIDIAN_PLANES = 4  # full hoop planes -> 8 half arcs
N_STRIPE_SEGMENTS = 8  # alternating red/white bands per half arc

# Latitude rings: (name, height above sphere center, material key)
LATITUDE_RINGS = (
    ("ring_upper", 0.52, "sky_blue"),
    ("ring_equator", 0.0, "sky_blue"),
    ("ring_yellow", -0.34, "yellow"),
    ("ring_lower", -0.60, "sky_blue"),
)


def _arc_points(phi_start: float, phi_end: float, n: int) -> list[tuple[float, float, float]]:
    """Points along a meridian circle of radius SPHERE_R in the local XZ plane."""
    pts = []
    for i in range(n):
        phi = phi_start + (phi_end - phi_start) * i / (n - 1)
        pts.append((SPHERE_R * math.sin(phi), 0.0, SPHERE_R * math.cos(phi)))
    return pts


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spherical_merry_go_round")

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
    # Round bearing journals the cage collars ride on (upper / lower pole).
    for tag, sign in (("upper", 1.0), ("lower", -1.0)):
        post.visual(
            Cylinder(radius=JOURNAL_R, length=JOURNAL_LEN),
            origin=Origin(xyz=(0.0, 0.0, CENTER_Z + sign * COLLAR_Z)),
            material=steel_dark,
            name=f"journal_{tag}",
        )

    # ------------------------------------------------------------------
    # Child: rigid spinning hoop cage (local frame at sphere center)
    # ------------------------------------------------------------------
    cage = model.part("hoop_cage")

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
        origin=Origin(xyz=(0.0, 0.0, COLLAR_Z)),
        material=steel_dark,
        name="collar_upper",
    )
    cage.visual(
        collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, -COLLAR_Z)),
        material=steel_dark,
        name="collar_lower",
    )

    # Latitude rings (horizontal tori on the sphere surface).
    for ring_name, height, mat_key in LATITUDE_RINGS:
        ring_r = math.sqrt(SPHERE_R**2 - height**2)
        ring_mesh = mesh_from_geometry(
            TorusGeometry(radius=ring_r, tube=TUBE_R, radial_segments=16, tubular_segments=72),
            ring_name,
        )
        cage.visual(
            ring_mesh,
            origin=Origin(xyz=(0.0, 0.0, height)),
            material=sky_blue if mat_key == "sky_blue" else worn_yellow,
            name=ring_name,
        )

    # Meridian half-arcs (red base tube, pole collar to pole collar).
    arc_mesh = mesh_from_geometry(
        tube_from_spline_points(
            _arc_points(ARC_PHI0, math.pi - ARC_PHI0, 33),
            radius=TUBE_R,
            samples_per_segment=4,
            radial_segments=14,
            cap_ends=True,
        ),
        "meridian_arc",
    )
    # White stripe sleeves on alternating bands -> candy-stripe paint.
    delta = (math.pi - 2.0 * ARC_PHI0) / N_STRIPE_SEGMENTS
    stripe_meshes = []
    for j in range(1, N_STRIPE_SEGMENTS, 2):
        stripe_meshes.append(
            mesh_from_geometry(
                tube_from_spline_points(
                    _arc_points(ARC_PHI0 + j * delta, ARC_PHI0 + (j + 1) * delta, 7),
                    radius=STRIPE_R,
                    samples_per_segment=4,
                    radial_segments=14,
                    cap_ends=True,
                ),
                f"meridian_stripe_{j}",
            )
        )
    for k in range(2 * N_MERIDIAN_PLANES):
        yaw = k * math.pi / N_MERIDIAN_PLANES
        cage.visual(
            arc_mesh,
            origin=Origin(rpy=(0.0, 0.0, yaw)),
            material=candy_red,
            name=f"meridian_arc_{k}",
        )
        for s, stripe_mesh in enumerate(stripe_meshes):
            cage.visual(
                stripe_mesh,
                origin=Origin(rpy=(0.0, 0.0, yaw)),
                material=stripe_white,
                name=f"meridian_stripe_{k}_{s}",
            )

    # Rusty clamp brackets where meridians cross the latitude rings.
    ring_tags = {"ring_upper": "up", "ring_equator": "eq", "ring_yellow": "yel", "ring_lower": "low"}
    for ring_name, height, _mat in LATITUDE_RINGS:
        ring_r = math.sqrt(SPHERE_R**2 - height**2)
        tag = ring_tags[ring_name]
        for k in range(2 * N_MERIDIAN_PLANES):
            az = k * math.pi / N_MERIDIAN_PLANES
            cage.visual(
                Box((0.055, 0.05, 0.05)),
                origin=Origin(
                    xyz=(ring_r * math.cos(az), ring_r * math.sin(az), height),
                    rpy=(0.0, 0.0, az),
                ),
                material=rust,
                name=f"clamp_{tag}_{k}",
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
    cage = object_model.get_part("hoop_cage")
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

    # Bearing collars wrap the post journals: the collar bore is intentionally
    # captured on the journal (shaft-in-bushing fit), proven by the contact
    # checks below.
    for tag in ("upper", "lower"):
        ctx.allow_overlap(
            cage,
            post,
            elem_a=f"collar_{tag}",
            elem_b=f"journal_{tag}",
            reason=(
                "The cage bearing collar is intentionally captured on the round "
                "post journal so the spinning sphere reads as mounted on its "
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

    # Hero geometry: equator ring is the largest hoop; yellow ring lower-middle.
    eq = ctx.part_element_world_aabb(cage, elem="ring_equator")
    up = ctx.part_element_world_aabb(cage, elem="ring_upper")
    low = ctx.part_element_world_aabb(cage, elem="ring_lower")
    yel = ctx.part_element_world_aabb(cage, elem="ring_yellow")

    def _width(aabb):
        return aabb[1][0] - aabb[0][0]

    def _center_z(aabb):
        return (aabb[0][2] + aabb[1][2]) / 2.0

    ctx.check(
        "sphere is about 1.8 m in diameter",
        eq is not None and 1.70 <= _width(eq) <= 1.95,
        details=f"equator width={None if eq is None else _width(eq):.3f}",
    )
    ctx.check(
        "equator ring is the largest latitude ring",
        eq is not None
        and up is not None
        and low is not None
        and yel is not None
        and _width(eq) > _width(up) + 0.10
        and _width(eq) > _width(low) + 0.10
        and _width(eq) > _width(yel) + 0.05,
        details=f"widths eq={_width(eq):.2f} up={_width(up):.2f} low={_width(low):.2f} yel={_width(yel):.2f}",
    )
    ctx.check(
        "yellow ring sits in the lower-middle of the sphere",
        yel is not None and eq is not None and low is not None
        and _center_z(low) < _center_z(yel) < _center_z(eq) - 0.15,
        details=f"z yel={_center_z(yel):.2f} eq={_center_z(eq):.2f} low={_center_z(low):.2f}",
    )

    # Four meridian planes produce eight half-arcs (0..7).
    arc_names = [f"meridian_arc_{k}" for k in range(8)]
    arcs_present = all(
        ctx.part_element_world_aabb(cage, elem=n) is not None for n in arc_names
    )
    ctx.check(
        "four meridian planes yield eight half-arcs",
        arcs_present,
        details=f"missing one of {arc_names}",
    )
    # Eighth half-arc must NOT exist (no fifth plane).
    ctx.check(
        "no ninth half-arc (exactly four planes)",
        ctx.part_element_world_aabb(cage, elem="meridian_arc_8") is None,
        details="unexpected meridian_arc_8 found",
    )

    # Meridian hoops run pole to pole.
    arc = ctx.part_element_world_aabb(cage, elem="meridian_arc_0")
    ctx.check(
        "meridian hoop spans pole to pole",
        arc is not None and (arc[1][2] - arc[0][2]) > 1.70,
        details=f"arc z span={None if arc is None else arc[1][2] - arc[0][2]:.3f}",
    )
    stripe = ctx.part_element_world_aabb(cage, elem="meridian_stripe_0_0")
    ctx.check(
        "candy stripe bands sleeve the meridian hoop",
        stripe is not None,
        details="missing stripe element",
    )

    # Four planes are evenly spaced at 45-degree intervals.
    clamp_names = [f"clamp_eq_{k}" for k in range(8)]
    clamps_present = all(
        ctx.part_element_world_aabb(cage, elem=n) is not None for n in clamp_names
    )
    ctx.check(
        "eight equator clamps for four meridian planes",
        clamps_present,
        details=f"missing one of {clamp_names}",
    )

    # Cage spins freely above the ground; post stands ~2.2 m tall.
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

    # Decisive spin pose: a clamp on the equator at azimuth 0 swings from +X
    # to +Y when the cage rotates a quarter turn; the cage stays centered.
    before = ctx.part_element_world_aabb(cage, elem="clamp_eq_0")
    with ctx.pose({spin: math.pi / 2.0}):
        ctx.expect_origin_distance(
            cage, post, axes="xy", max_dist=0.002, name="spinning cage stays centered"
        )
        after = ctx.part_element_world_aabb(cage, elem="clamp_eq_0")

    def _center_xy(aabb):
        return ((aabb[0][0] + aabb[1][0]) / 2.0, (aabb[0][1] + aabb[1][1]) / 2.0)

    ok = False
    details = "missing clamp element"
    if before is not None and after is not None:
        bx, by = _center_xy(before)
        ax, ay = _center_xy(after)
        ok = bx > 0.70 and abs(by) < 0.05 and abs(ax) < 0.05 and ay > 0.70
        details = f"before=({bx:.2f},{by:.2f}) after=({ax:.2f},{ay:.2f})"
    ctx.check("quarter-turn spin carries the equator clamp from +X to +Y", ok, details=details)

    return ctx.report()


object_model = build_object_model()
