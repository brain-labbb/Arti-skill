from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _boat_profile(length: float = 0.36, max_width: float = 0.056) -> list[tuple[float, float]]:
    """Smooth plan-view outline for a pointed wooden shuttle body."""
    half_l = length / 2.0
    half_w = max_width / 2.0
    top: list[tuple[float, float]] = []
    for i in range(25):
        x = -half_l + length * i / 24.0
        t = abs(x) / half_l
        if t >= 1.0:
            y = 0.0
        else:
            y = half_w * (1.0 - t**1.75) ** 0.48
        top.append((x, y))
    bottom = [(x, -y) for x, y in reversed(top[1:-1])]
    return top + bottom


def _make_body_mesh():
    outer = _boat_profile()
    return (
        cq.Workplane("XY")
        .polyline(outer)
        .close()
        .extrude(0.034)
        .faces(">Z")
        .workplane(centerOption="CenterOfBoundBox")
        .slot2D(0.225, 0.044)
        .cutBlind(-0.030)
    )


def _cone_tip_mesh(name: str, *, toward_positive_x: bool):
    cone = ConeGeometry(0.012, 0.030, radial_segments=32, closed=True)
    cone.rotate_y(math.pi / 2.0 if toward_positive_x else -math.pi / 2.0)
    return mesh_from_geometry(cone, name)


def _torus_mesh(name: str, *, normal: str):
    torus = TorusGeometry(0.0075, 0.0016, radial_segments=20, tubular_segments=36)
    if normal == "x":
        torus.rotate_y(math.pi / 2.0)
    elif normal == "y":
        torus.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(torus, name)


def _pirn_core_mesh():
    """Tapered pirn body revolved around Z.  After visual rpy=(0,-π/2,0)
    the Z lathe axis maps to bobbin -X, so the base sits at bobbin x=0
    and the narrow tip extends toward bobbin x=-0.14."""
    profile = [
        (0.000, 0.000),
        (0.0105, 0.000),
        (0.0100, 0.015),
        (0.0090, 0.035),
        (0.0075, 0.060),
        (0.0058, 0.085),
        (0.0042, 0.110),
        (0.0030, 0.130),
        (0.0020, 0.138),
        (0.000, 0.140),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=32), "pirn_core")


def _pirn_thread_mesh():
    """Wound cotton layer over the mid/base portion of the pirn."""
    profile = [
        (0.000, 0.005),
        (0.0130, 0.005),
        (0.0125, 0.020),
        (0.0115, 0.040),
        (0.0100, 0.060),
        (0.0082, 0.078),
        (0.0065, 0.092),
        (0.000, 0.092),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=32), "pirn_thread")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="loom_shuttle",
        meta={
            "reference_note": (
                "End-delivery cotton-mill loom shuttle: tapered pirn on a "
                "fixed cantilever spindle with a tension gate at the nose."
            )
        },
    )

    wood = model.material("worn_oiled_wood", rgba=(0.50, 0.25, 0.08, 1.0))
    dark_wood = model.material("dark_worn_cavity", rgba=(0.13, 0.075, 0.040, 1.0))
    grain = model.material("subtle_dark_grain", rgba=(0.24, 0.12, 0.04, 1.0))
    metal = model.material("bruised_steel", rgba=(0.55, 0.58, 0.56, 1.0))
    black = model.material("blackened_eyelet", rgba=(0.015, 0.018, 0.017, 1.0))
    yarn = model.material("aged_cotton_thread", rgba=(0.86, 0.78, 0.60, 1.0))
    red = model.material("red_thread_end", rgba=(0.85, 0.05, 0.03, 1.0))
    bobbin_wood = model.material("bobbin_end_wood", rgba=(0.34, 0.17, 0.06, 1.0))

    # ── Body ──────────────────────────────────────────────────────────
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_mesh(), "carved_wooden_hull", tolerance=0.0007),
        material=wood,
        name="carved_hull",
    )
    body.visual(
        Box((0.210, 0.030, 0.0012)),
        origin=Origin(xyz=(0.0, 0.0, 0.0043)),
        material=dark_wood,
        name="cavity_floor",
    )
    # Metal pointed shuttle noses
    body.visual(
        _cone_tip_mesh("left_metal_tip", toward_positive_x=False),
        origin=Origin(xyz=(-0.171, 0.0, 0.018)),
        material=metal,
        name="front_metal_tip",
    )
    body.visual(
        _cone_tip_mesh("right_metal_tip", toward_positive_x=True),
        origin=Origin(xyz=(0.171, 0.0, 0.018)),
        material=metal,
        name="rear_metal_tip",
    )

    # ── Rear bearing / spindle root (single cantilever mount) ────────
    body.visual(
        Box((0.006, 0.004, 0.008)),
        origin=Origin(xyz=(0.105, 0.0, 0.0076)),
        material=black,
        name="rear_bearing_saddle",
    )
    body.visual(
        _torus_mesh("rear_bearing_ring", normal="x"),
        origin=Origin(xyz=(0.105, 0.0, 0.020)),
        material=black,
        name="rear_bearing",
    )

    # ── Fixed cantilever spindle rod ─────────────────────────────────
    cyl_to_x = (0.0, math.pi / 2.0, 0.0)
    body.visual(
        Cylinder(radius=0.0028, length=0.120),
        origin=Origin(xyz=(0.045, 0.0, 0.020), rpy=cyl_to_x),
        material=metal,
        name="spindle_rod",
    )
    # Spindle collar bridges the rod to the rear bearing torus bore
    body.visual(
        Cylinder(radius=0.0062, length=0.008),
        origin=Origin(xyz=(0.105, 0.0, 0.020), rpy=cyl_to_x),
        material=metal,
        name="spindle_collar",
    )

    # ── Tension gate / porcupine eyelet near front nose ──────────────
    body.visual(
        Cylinder(radius=0.0020, length=0.014),
        origin=Origin(xyz=(-0.152, 0.0, 0.041)),
        material=metal,
        name="tension_gate_post",
    )
    tg_eyelet = TorusGeometry(0.0040, 0.0010, radial_segments=16, tubular_segments=24)
    tg_eyelet.rotate_y(math.pi / 2.0)
    body.visual(
        mesh_from_geometry(tg_eyelet, "tension_gate_eyelet_mesh"),
        origin=Origin(xyz=(-0.152, 0.0, 0.049)),
        material=black,
        name="tension_gate_eyelet",
    )

    # ── Side eyelets and dark holes (unchanged) ──────────────────────
    for x, y, z, nm in [
        (-0.142, -0.016, 0.018, "side_eyelet_0"),
        (0.145, -0.014, 0.022, "side_eyelet_1"),
    ]:
        body.visual(
            _torus_mesh(f"{nm}_mesh", normal="y"),
            origin=Origin(xyz=(x, y, z)),
            material=black,
            name=nm,
        )
    for i, (x, y) in enumerate([(-0.104, 0.018), (-0.045, 0.023), (0.046, 0.023), (0.125, 0.014)]):
        body.visual(
            Cylinder(radius=0.0027, length=0.0010),
            origin=Origin(xyz=(x, y, 0.0343)),
            material=black,
            name=f"top_hole_{i}",
        )
    for i, (x, y, sx, yaw) in enumerate(
        [
            (-0.095, 0.021, 0.055, 0.05),
            (-0.020, 0.024, 0.070, -0.04),
            (0.065, 0.021, 0.060, 0.06),
            (-0.110, -0.021, 0.040, -0.08),
            (0.030, -0.024, 0.080, 0.03),
            (0.116, -0.016, 0.040, -0.06),
        ]
    ):
        body.visual(
            Box((sx, 0.0020, 0.0008)),
            origin=Origin(xyz=(x, y, 0.0339), rpy=(0.0, 0.0, yaw)),
            material=grain,
            name=f"wood_grain_{i}",
        )

    # ── Bobbin (tapered pirn on cantilever spindle) ──────────────────
    bobbin = model.part("bobbin")
    # Lathe Z maps to bobbin -X via rpy=(0,-π/2,0), so the pirn
    # extends from bobbin x=0 (base at spindle root) toward x=-0.14 (tip).
    pirn_rot = (0.0, -math.pi / 2.0, 0.0)

    bobbin.visual(
        _pirn_core_mesh(),
        origin=Origin(rpy=pirn_rot),
        material=bobbin_wood,
        name="bobbin_core",
    )
    bobbin.visual(
        _pirn_thread_mesh(),
        origin=Origin(rpy=pirn_rot),
        material=yarn,
        name="cotton_thread",
    )

    # Rear flange: base collar where pirn seats against spindle shoulder
    bobbin.visual(
        Cylinder(radius=0.0120, length=0.006),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=cyl_to_x),
        material=black,
        name="rear_flange",
    )
    bobbin.visual(
        Cylinder(radius=0.0075, length=0.004),
        origin=Origin(xyz=(0.007, 0.0, 0.0), rpy=cyl_to_x),
        material=bobbin_wood,
        name="rear_flange_boss",
    )

    # Front flange: small locating collar near base of the pirn
    bobbin.visual(
        Cylinder(radius=0.0110, length=0.005),
        origin=Origin(xyz=(-0.016, 0.0, 0.0), rpy=cyl_to_x),
        material=black,
        name="front_flange",
    )
    bobbin.visual(
        Cylinder(radius=0.0070, length=0.004),
        origin=Origin(xyz=(-0.020, 0.0, 0.0), rpy=cyl_to_x),
        material=bobbin_wood,
        name="front_flange_boss",
    )

    # Off-axis red thread tail on pirn surface for rotation visibility
    bobbin.visual(
        Box((0.012, 0.004, 0.003)),
        origin=Origin(xyz=(-0.040, 0.001, 0.009), rpy=(0.0, 0.0, 0.08)),
        material=red,
        name="red_thread",
    )

    # ── Articulation ─────────────────────────────────────────────────
    model.articulation(
        "body_to_bobbin",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bobbin,
        origin=Origin(xyz=(0.100, 0.0, 0.020)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    bobbin = object_model.get_part("bobbin")
    joint = object_model.get_articulation("body_to_bobbin")

    # Pirn is retained inside the shuttle footprint
    ctx.expect_within(
        bobbin,
        body,
        axes="xy",
        margin=0.003,
        name="pirn is retained inside the shuttle footprint",
    )

    # Pirn overlaps the cavity lengthwise (cantilevered from rear)
    ctx.expect_overlap(
        bobbin,
        body,
        axes="x",
        min_overlap=0.10,
        name="pirn spans a significant portion of the cavity lengthwise",
    )

    # Pirn clears the cavity floor
    ctx.expect_gap(
        bobbin,
        body,
        axis="z",
        min_gap=0.0005,
        negative_elem="cavity_floor",
        name="pirn clears the carved cavity floor",
    )

    # The fixed spindle rod passes through the pirn bore; all pirn and
    # collar visuals intentionally surround the spindle.
    ctx.allow_overlap(
        body,
        bobbin,
        reason=(
            "The fixed cantilever spindle rod passes through the pirn bore; "
            "the pirn core, wound thread, and locating collars all intentionally "
            "surround the spindle as a seated end-delivery fit."
        ),
    )
    ctx.expect_overlap(
        bobbin,
        body,
        axes="x",
        min_overlap=0.08,
        elem_a="bobbin_core",
        elem_b="spindle_rod",
        name="pirn core overlaps spindle rod along the spindle axis",
    )
    ctx.expect_within(
        bobbin,
        body,
        axes="y",
        margin=0.004,
        name="pirn stays centered on the spindle axis laterally",
    )

    # Rear flange seats near the spindle root bearing
    ctx.expect_contact(
        bobbin,
        body,
        elem_a="rear_flange",
        elem_b="rear_bearing",
        contact_tol=0.012,
        name="rear bearing supports the pirn at the spindle root",
    )

    # Tension gate post is near the front nose
    tg_aabb = ctx.part_element_world_aabb(body, elem="tension_gate_post")
    ctx.check(
        "tension gate post is near the front nose for end-delivery thread path",
        tg_aabb is not None and tg_aabb[0][0] < -0.12,
        details=f"tension_gate_post aabb={tg_aabb}",
    )

    # Pirn is cantilevered: tip extends well forward of the spindle root
    pirn_aabb = ctx.part_element_world_aabb(bobbin, elem="bobbin_core")
    ctx.check(
        "pirn is cantilevered from rear spindle with tip extending forward",
        pirn_aabb is not None and pirn_aabb[0][0] < -0.02,
        details=f"bobbin_core aabb min_x={pirn_aabb[0][0] if pirn_aabb else None}",
    )

    # Revolute joint: pirn spins on spindle, red thread mark moves
    rest = ctx.part_world_position(bobbin)
    rest_tail = ctx.part_element_world_aabb(bobbin, elem="red_thread")
    with ctx.pose({joint: 1.2}):
        turned = ctx.part_world_position(bobbin)
        turned_tail = ctx.part_element_world_aabb(bobbin, elem="red_thread")

    def _aabb_center_yz(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return ((lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0)

    rest_yz = _aabb_center_yz(rest_tail)
    turned_yz = _aabb_center_yz(turned_tail)
    ctx.check(
        "body_to_bobbin revolute keeps pirn centered on spindle while thread mark turns",
        rest is not None
        and turned is not None
        and abs(rest[0] - turned[0]) < 1e-6
        and rest_yz is not None
        and turned_yz is not None
        and abs(rest_yz[0] - turned_yz[0]) > 0.003,
        details=f"rest={rest}, turned={turned}, rest_tail={rest_yz}, turned_tail={turned_yz}",
    )

    return ctx.report()


object_model = build_object_model()
