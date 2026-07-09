from __future__ import annotations

# Half-dome (hemisphere) playground merry-go-round (orbit spinner).
#
# A fixed white square steel post carries a spinning hemispherical cage of
# tubular steel hoops (sphere ~1.8 m diameter, upper hemisphere only): a floor
# base ring at the equator (~1.8 m diameter, sky blue), sky-blue latitude
# rings shrinking toward the top pole, one yellow accent ring in the upper
# middle, and six meridian arcs spanning from the top pole collar down to the
# floor base ring, painted in red-and-white candy stripes. The dome sits
# open-side-down. The meridian arcs converge at a collar bearing near the top
# pole that rides a round journal on the post. The whole cage spins freely
# 360 degrees about the vertical post axis (continuous revolute joint).

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
CENTER_Z = 0.90  # sphere center height (= equator / floor ring plane)

COLLAR_INNER_R = 0.0895  # collar bore radius (light press onto the journal)
COLLAR_OUTER_R = 0.115  # collar outer radius
COLLAR_HALF_H = 0.070  # collar half height
JOURNAL_R = 0.0905  # round bearing journal radius on the post
JOURNAL_LEN = 0.18

# Meridian arcs run from the top pole collar down to the floor ring (equator).
ARC_PHI0 = math.asin(COLLAR_OUTER_R / SPHERE_R)  # polar angle near top pole
ARC_PHI1 = math.pi / 2.0  # equator (floor ring plane)
COLLAR_Z = SPHERE_R * math.cos(ARC_PHI0)  # collar center z (local, above equator)

N_MERIDIAN_PLANES = 3  # 3 planes -> 6 meridian arcs (2 per plane)
N_STRIPE_SEGMENTS = 5  # alternating red/white bands per arc

# Latitude rings on the hemisphere (height above equator, material key).
# Rings shrink toward the top pole; the floor base ring is the largest.
LATITUDE_RINGS = (
    ("ring_base", 0.0, "sky_blue"),        # floor base ring (equator, largest)
    ("ring_mid", 0.30, "sky_blue"),         # mid-height
    ("ring_yellow", 0.52, "yellow"),        # yellow accent
    ("ring_upper", 0.72, "sky_blue"),       # near top pole (smallest)
)


def _arc_points(phi_start: float, phi_end: float, n: int) -> list[tuple[float, float, float]]:
    """Points along a meridian arc of radius SPHERE_R in the local XZ plane."""
    pts = []
    for i in range(n):
        phi = phi_start + (phi_end - phi_start) * i / (n - 1)
        pts.append((SPHERE_R * math.sin(phi), 0.0, SPHERE_R * math.cos(phi)))
    return pts


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hemisphere_merry_go_round")

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
    # Round bearing journal at the top pole for the dome collar to ride on.
    post.visual(
        Cylinder(radius=JOURNAL_R, length=JOURNAL_LEN),
        origin=Origin(xyz=(0.0, 0.0, CENTER_Z + COLLAR_Z)),
        material=steel_dark,
        name="journal_upper",
    )

    # ------------------------------------------------------------------
    # Child: rigid spinning hemisphere hoop cage (local frame at sphere center)
    # ------------------------------------------------------------------
    cage = model.part("hoop_cage")

    # Upper collar bearing: hollow sleeve wrapping the post journal at the dome apex.
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

    # Latitude rings (horizontal tori on the hemisphere surface, shrinking upward).
    for i, (ring_name, height, mat_key) in enumerate(LATITUDE_RINGS):
        ring_r = math.sqrt(SPHERE_R**2 - height**2)
        ring_mesh = mesh_from_geometry(
            TorusGeometry(radius=ring_r, tube=TUBE_R, radial_segments=16, tubular_segments=72),
            ring_name,
        )
        cage.visual(
            ring_mesh,
            origin=Origin(xyz=(0.0, 0.0, height)),
            material=sky_blue if mat_key == "sky_blue" else worn_yellow,
            name=f"latitude_ring_{i}",
        )

    # Meridian arcs (red base tube, top pole collar down to floor ring equator).
    arc_mesh = mesh_from_geometry(
        tube_from_spline_points(
            _arc_points(ARC_PHI0, ARC_PHI1, 25),
            radius=TUBE_R,
            samples_per_segment=4,
            radial_segments=14,
            cap_ends=True,
        ),
        "meridian_arc",
    )
    # White stripe sleeves on alternating bands -> candy-stripe paint.
    arc_span = ARC_PHI1 - ARC_PHI0
    delta = arc_span / N_STRIPE_SEGMENTS
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
    ring_tags = {0: "base", 1: "mid", 2: "yel", 3: "up"}
    for i, (ring_name, height, _mat) in enumerate(LATITUDE_RINGS):
        ring_r = math.sqrt(SPHERE_R**2 - height**2)
        tag = ring_tags[i]
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

    # Bearing collar wraps the post journal (shaft-in-bushing fit).
    ctx.allow_overlap(
        cage,
        post,
        elem_a="collar_upper",
        elem_b="journal_upper",
        reason=(
            "The cage bearing collar is intentionally captured on the round "
            "post journal so the spinning dome reads as mounted on its "
            "bearing; the embed is a thin hidden ring inside the collar bore."
        ),
    )
    ctx.expect_within(
        post,
        cage,
        axes="xy",
        inner_elem="journal_upper",
        outer_elem="collar_upper",
        margin=0.0,
        name="upper journal sits inside its collar bore",
    )
    ctx.expect_contact(
        cage,
        post,
        elem_a="collar_upper",
        elem_b="journal_upper",
        contact_tol=0.004,
        name="upper collar rides its journal",
    )

    # Hemisphere geometry: latitude rings shrink toward the top pole.
    base_aabb = ctx.part_element_world_aabb(cage, elem="latitude_ring_0")
    mid_aabb = ctx.part_element_world_aabb(cage, elem="latitude_ring_1")
    yel_aabb = ctx.part_element_world_aabb(cage, elem="latitude_ring_2")
    up_aabb = ctx.part_element_world_aabb(cage, elem="latitude_ring_3")

    def _width(aabb):
        return aabb[1][0] - aabb[0][0]

    def _center_z(aabb):
        return (aabb[0][2] + aabb[1][2]) / 2.0

    # Floor base ring is the largest latitude ring.
    ctx.check(
        "floor base ring is the largest latitude ring",
        base_aabb is not None and mid_aabb is not None
        and yel_aabb is not None and up_aabb is not None
        and _width(base_aabb) > _width(mid_aabb) + 0.02
        and _width(base_aabb) > _width(yel_aabb) + 0.05
        and _width(base_aabb) > _width(up_aabb) + 0.10,
        details=(
            f"widths base={_width(base_aabb):.2f} mid={_width(mid_aabb):.2f} "
            f"yel={_width(yel_aabb):.2f} up={_width(up_aabb):.2f}"
        ),
    )

    # Rings are stacked: base < mid < yellow < upper in height.
    ctx.check(
        "latitude rings stack upward from base to top pole",
        base_aabb is not None and mid_aabb is not None
        and yel_aabb is not None and up_aabb is not None
        and _center_z(base_aabb) < _center_z(mid_aabb)
        < _center_z(yel_aabb) < _center_z(up_aabb),
        details=(
            f"z base={_center_z(base_aabb):.2f} mid={_center_z(mid_aabb):.2f} "
            f"yel={_center_z(yel_aabb):.2f} up={_center_z(up_aabb):.2f}"
        ),
    )

    # Rings shrink monotonically toward the top.
    ctx.check(
        "latitude rings shrink toward the top pole",
        base_aabb is not None and mid_aabb is not None
        and yel_aabb is not None and up_aabb is not None
        and _width(base_aabb) > _width(mid_aabb) > _width(yel_aabb) > _width(up_aabb),
        details=(
            f"widths base={_width(base_aabb):.2f} mid={_width(mid_aabb):.2f} "
            f"yel={_width(yel_aabb):.2f} up={_width(up_aabb):.2f}"
        ),
    )

    # Hemisphere dome height: about 0.9 m (half the sphere diameter), not a full sphere.
    cage_aabb = ctx.part_world_aabb(cage)
    cage_height = cage_aabb[1][2] - cage_aabb[0][2] if cage_aabb else None
    ctx.check(
        "cage is a hemisphere (height ~0.9 m, not full 1.8 m sphere)",
        cage_height is not None and 0.75 <= cage_height <= 1.15,
        details=f"cage height={cage_height:.3f}" if cage_height else "missing cage",
    )

    # Floor base ring is about 1.8 m diameter.
    ctx.check(
        "floor base ring is about 1.8 m diameter",
        base_aabb is not None and 1.70 <= _width(base_aabb) <= 1.95,
        details=f"base ring width={None if base_aabb is None else _width(base_aabb):.3f}",
    )

    # Meridian arcs span from near the top pole down to the floor ring (not pole to pole).
    arc_aabb = ctx.part_element_world_aabb(cage, elem="meridian_arc_0")
    arc_span_z = arc_aabb[1][2] - arc_aabb[0][2] if arc_aabb else None
    ctx.check(
        "meridian arc spans pole to floor ring (not pole to pole)",
        arc_span_z is not None and 0.65 <= arc_span_z <= 1.10,
        details=f"arc z span={arc_span_z:.3f}" if arc_span_z else "missing arc",
    )

    # Candy stripes on the meridian arcs.
    stripe_aabb = ctx.part_element_world_aabb(cage, elem="meridian_stripe_0_0")
    ctx.check(
        "candy stripe bands sleeve the meridian arc",
        stripe_aabb is not None,
        details="missing stripe element",
    )

    # Cage clears the ground; post stands ~2.2 m tall and tops out above dome.
    post_aabb = ctx.part_world_aabb(post)
    ctx.check(
        "cage clears the ground",
        cage_aabb is not None and cage_aabb[0][2] > 0.10,
        details=f"cage min z={None if cage_aabb is None else cage_aabb[0][2]:.3f}",
    )
    ctx.check(
        "post is about 2.2 m tall and tops out above the dome",
        post_aabb is not None
        and cage_aabb is not None
        and 2.15 <= post_aabb[1][2] <= 2.30
        and post_aabb[1][2] > cage_aabb[1][2],
        details=f"post top={None if post_aabb is None else post_aabb[1][2]:.3f}",
    )

    # Open-side-down: no cage geometry significantly below the floor base ring.
    ctx.check(
        "dome is open-side-down (no cage geometry far below the base ring)",
        cage_aabb is not None and base_aabb is not None
        and cage_aabb[0][2] >= base_aabb[0][2] - 0.10,
        details=(
            f"cage min z={cage_aabb[0][2]:.3f}, "
            f"base min z={base_aabb[0][2]:.3f}"
        ),
    )

    # Decisive spin pose: a clamp on the base ring swings from +X to +Y on quarter turn.
    before = ctx.part_element_world_aabb(cage, elem="clamp_base_0")
    with ctx.pose({spin: math.pi / 2.0}):
        ctx.expect_origin_distance(
            cage, post, axes="xy", max_dist=0.002, name="spinning cage stays centered"
        )
        after = ctx.part_element_world_aabb(cage, elem="clamp_base_0")

    def _center_xy(aabb):
        return ((aabb[0][0] + aabb[1][0]) / 2.0, (aabb[0][1] + aabb[1][1]) / 2.0)

    ok = False
    details = "missing clamp element"
    if before is not None and after is not None:
        bx, by = _center_xy(before)
        ax, ay = _center_xy(after)
        ok = bx > 0.70 and abs(by) < 0.05 and abs(ax) < 0.05 and ay > 0.70
        details = f"before=({bx:.2f},{by:.2f}) after=({ax:.2f},{ay:.2f})"
    ctx.check(
        "quarter-turn spin carries the base ring clamp from +X to +Y", ok, details=details
    )

    return ctx.report()


object_model = build_object_model()
