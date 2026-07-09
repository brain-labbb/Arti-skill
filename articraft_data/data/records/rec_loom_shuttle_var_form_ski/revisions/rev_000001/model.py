from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    AllowedOverlap,
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _ski_profile(length: float = 0.42, max_width: float = 0.038) -> list[tuple[float, float]]:
    """Long narrow pointed ski outline in XY for a ski-style loom shuttle."""
    half_l = length / 2.0
    half_w = max_width / 2.0
    top: list[tuple[float, float]] = []
    n = 36
    for i in range(n):
        x = -half_l + length * i / (n - 1)
        t = abs(x) / half_l
        if t >= 1.0:
            y = 0.0
        else:
            y = half_w * (1.0 - t ** 2.0) ** 0.55
        top.append((x, y))
    bottom = [(x, -y) for x, y in reversed(top[1:-1])]
    return top + bottom


def _ski_half_width_at(x: float, length: float = 0.42, max_width: float = 0.038) -> float:
    """Half-width of the ski plan at a given x position."""
    half_l = length / 2.0
    half_w = max_width / 2.0
    t = abs(x) / half_l
    if t >= 1.0:
        return 0.0
    return half_w * (1.0 - t ** 2.0) ** 0.55


def _make_body_mesh():
    """Ski shuttle hull: long thin slab with shallow open channel and upswept tips."""
    length = 0.42
    max_width = 0.038
    body_height = 0.022
    channel_depth = 0.007
    channel_length = 0.28
    channel_width = 0.026
    tip_lift = 0.014
    tip_region = 0.070

    half_l = length / 2.0
    plan = _ski_profile(length, max_width)

    # 1) Extrude the ski plan outline into a thin slab (z=0 to z=body_height)
    body = (
        cq.Workplane("XY")
        .polyline(plan)
        .close()
        .extrude(body_height)
    )

    # 2) Cut a shallow open running channel on the top face
    body = (
        body
        .faces(">Z")
        .workplane(centerOption="CenterOfBoundBox")
        .slot2D(channel_length, channel_width)
        .cutBlind(-channel_depth)
    )

    # 3) Cut upswept tips: remove material from the bottom at both ends,
    #    tapering from zero cut at the start of the tip region to tip_lift
    #    at the very end. This creates the characteristic ski-tip profile
    #    where the bottom rises at the pointed ends.
    for sign in (-1, 1):
        x_tip = sign * half_l
        x_start = sign * (half_l - tip_region)
        cutter = (
            cq.Workplane("XZ")
            .moveTo(x_start, -0.001)
            .lineTo(x_tip, -0.001)
            .lineTo(x_tip, tip_lift)
            .close()
            .extrude(max_width * 3.0)
            .translate((0.0, -max_width * 1.5, 0.0))
        )
        body = body.cut(cutter)

    # 4) Add upswept top ramp at each tip as a separate CadQuery union.
    #    Build each ramp as a narrow triangular prism that fits within the
    #    hull at the tip region, then union one at a time.
    for sign in (-1, 1):
        x_tip = sign * half_l
        x_start = sign * (half_l - tip_region)
        ramp_w = max_width * 0.6  # narrower than hull at tip start
        ramp_solid = (
            cq.Workplane("XZ")
            .moveTo(x_start, body_height)
            .lineTo(x_tip, body_height + tip_lift * 0.7)
            .lineTo(x_tip, body_height)
            .close()
            .extrude(ramp_w)
            .translate((0.0, -ramp_w / 2.0, 0.0))
        )
        try:
            body = body.union(ramp_solid)
        except Exception:
            pass  # cosmetic; skip if kernel rejects

    # 5) Soften the top edges for a worn-wood feel
    try:
        body = body.edges(">Z").fillet(0.0015)
    except Exception:
        pass

    return body


def _cone_tip_mesh(name: str, *, toward_positive_x: bool):
    cone = ConeGeometry(0.008, 0.024, radial_segments=32, closed=True)
    cone.rotate_y(math.pi / 2.0 if toward_positive_x else -math.pi / 2.0)
    return mesh_from_geometry(cone, name)


def _torus_mesh(name: str, *, normal: str):
    torus = TorusGeometry(0.0070, 0.0014, radial_segments=20, tubular_segments=36)
    if normal == "x":
        torus.rotate_y(math.pi / 2.0)
    elif normal == "y":
        torus.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(torus, name)


# ---------------------------------------------------------------------------
# Object model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="loom_shuttle",
        meta={
            "reference_note": (
                "Ski-style loom shuttle: long slender body with upswept pointed "
                "tips and a shallow open running channel carrying a rotating "
                "weft bobbin. Variant of the boat shuttle with ski-profile hull."
            )
        },
    )

    # ---- Materials ----
    wood = model.material("honey_oiled_wood", rgba=(0.56, 0.34, 0.12, 1.0))
    dark_wood = model.material("dark_worn_channel", rgba=(0.14, 0.08, 0.04, 1.0))
    dark_grain = model.material("dark_end_grain", rgba=(0.22, 0.11, 0.04, 1.0))
    grain = model.material("subtle_dark_grain", rgba=(0.24, 0.12, 0.04, 1.0))
    metal = model.material("bruised_steel", rgba=(0.55, 0.58, 0.56, 1.0))
    black = model.material("blackened_eyelet", rgba=(0.015, 0.018, 0.017, 1.0))
    yarn = model.material("aged_cotton_thread", rgba=(0.86, 0.78, 0.60, 1.0))
    red = model.material("red_thread_end", rgba=(0.85, 0.05, 0.03, 1.0))
    bobbin_wood = model.material("bobbin_end_wood", rgba=(0.34, 0.17, 0.06, 1.0))

    # ---- Key dimensions ----
    body_length = 0.42
    body_height = 0.022
    channel_depth = 0.007
    half_l = body_length / 2.0
    channel_floor_z = body_height - channel_depth  # 0.015

    # Bobbin sits high in the shallow open channel, mostly exposed (ski shuttle)
    bobbin_radius = 0.0061
    flange_radius = 0.0142
    # Flange bottom must clear cavity_floor top (channel_floor_z + 0.001 = 0.016)
    bobbin_z = channel_floor_z + flange_radius + 0.002  # 0.0312

    # ==== BODY (ski hull) ====
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_mesh(), "ski_shuttle_hull", tolerance=0.0007),
        material=wood,
        name="carved_hull",
    )

    # Dark channel floor — shallow open groove bottom visible from above
    body.visual(
        Box((0.250, 0.020, 0.001)),
        origin=Origin(xyz=(0.0, 0.0, channel_floor_z + 0.0005)),
        material=dark_wood,
        name="cavity_floor",
    )

    # Metal pointed ski tips, partly seated into the wooden ends
    tip_x = half_l - 0.012
    tip_z = body_height * 0.65  # mid-height region at upswept tip
    body.visual(
        _cone_tip_mesh("left_metal_tip", toward_positive_x=False),
        origin=Origin(xyz=(-tip_x, 0.0, tip_z)),
        material=metal,
        name="front_metal_tip",
    )
    body.visual(
        _cone_tip_mesh("right_metal_tip", toward_positive_x=True),
        origin=Origin(xyz=(tip_x, 0.0, tip_z)),
        material=metal,
        name="rear_metal_tip",
    )

    # Bearing eyelets at channel ends — support the bobbin axle
    bearing_x = 0.120
    saddle_h = bobbin_z - channel_floor_z  # saddle rises from floor to axle
    for x, nm in [(-bearing_x, "front_bearing"), (bearing_x, "rear_bearing")]:
        body.visual(
            Box((0.006, 0.004, saddle_h)),
            origin=Origin(xyz=(x, 0.0, channel_floor_z + saddle_h / 2.0)),
            material=black,
            name=f"{nm}_saddle",
        )
        body.visual(
            _torus_mesh(f"{nm}_ring", normal="x"),
            origin=Origin(xyz=(x, 0.0, bobbin_z)),
            material=black,
            name=nm,
        )

    # Side eyelets (thread guides on the ski hull flanks)
    for x, y, z, nm in [
        (-0.155, -0.012, body_height * 0.70, "side_eyelet_0"),
        (0.158, -0.011, body_height * 0.80, "side_eyelet_1"),
    ]:
        body.visual(
            _torus_mesh(f"{nm}_mesh", normal="y"),
            origin=Origin(xyz=(x, y, z)),
            material=black,
            name=nm,
        )

    # Decorative drilled holes on the top rim (within hull outline)
    for i, (x, y) in enumerate([
        (-0.100, 0.013), (-0.040, 0.015), (0.046, 0.015), (0.110, 0.011),
    ]):
        body.visual(
            Cylinder(radius=0.0025, length=0.0010),
            origin=Origin(xyz=(x, y, body_height + 0.0003)),
            material=black,
            name=f"top_hole_{i}",
        )

    # Wood grain streaks — positions kept within the ski hull outline
    grain_positions = [
        (-0.080, 0.014, 0.050, 0.05),
        (-0.020, 0.016, 0.065, -0.04),
        (0.060, 0.014, 0.055, 0.06),
        (-0.090, -0.013, 0.038, -0.08),
        (0.025, -0.015, 0.070, 0.03),
        (0.100, -0.012, 0.038, -0.06),
    ]
    for i, (x, y, sx, yaw) in enumerate(grain_positions):
        # Ensure grain is within hull outline
        hw = _ski_half_width_at(x)
        y_clamped = max(-hw + 0.002, min(hw - 0.002, y))
        body.visual(
            Box((sx, 0.0018, 0.0007)),
            origin=Origin(xyz=(x, y_clamped, body_height + 0.0001), rpy=(0.0, 0.0, yaw)),
            material=grain,
            name=f"wood_grain_{i}",
        )

    # Darker end-grain patches on top surface near the pointed tips
    for i, (x, sx) in enumerate([(-0.155, 0.026), (0.155, 0.024)]):
        body.visual(
            Box((sx, 0.007, 0.0008)),
            origin=Origin(xyz=(x, 0.0, body_height + 0.0002)),
            material=dark_grain,
            name=f"end_grain_{i}",
        )

    # ==== BOBBIN ====
    bobbin = model.part("bobbin")
    cyl_to_x = (0.0, math.pi / 2.0, 0.0)

    # Extended core so it reaches the bearing eyelets at ±0.120
    bobbin_core_length = 0.260
    bobbin.visual(
        Cylinder(radius=bobbin_radius, length=bobbin_core_length),
        origin=Origin(rpy=cyl_to_x),
        material=bobbin_wood,
        name="bobbin_core",
    )
    bobbin.visual(
        Cylinder(radius=0.0132, length=0.142),
        origin=Origin(rpy=cyl_to_x),
        material=yarn,
        name="cotton_thread",
    )
    for x, nm in [(-0.078, "front_flange"), (0.078, "rear_flange")]:
        bobbin.visual(
            Cylinder(radius=flange_radius, length=0.010),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=cyl_to_x),
            material=black,
            name=nm,
        )
        bobbin.visual(
            Cylinder(radius=0.0080, length=0.006),
            origin=Origin(xyz=(x * 1.03, 0.0, 0.0), rpy=cyl_to_x),
            material=bobbin_wood,
            name=f"{nm}_boss",
        )

    # Off-axis red thread tail makes rotation visible
    bobbin.visual(
        Box((0.014, 0.004, 0.003)),
        origin=Origin(xyz=(0.071, 0.001, 0.0138), rpy=(0.0, 0.0, 0.08)),
        material=red,
        name="red_thread",
    )

    # ==== ARTICULATION (one revolute bobbin joint, axis X) ====
    model.articulation(
        "body_to_bobbin",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bobbin,
        origin=Origin(xyz=(0.0, 0.0, bobbin_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4, velocity=8.0, lower=-math.pi, upper=math.pi,
        ),
    )
    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    bobbin = object_model.get_part("bobbin")
    joint = object_model.get_articulation("body_to_bobbin")

    # ---- Intentional overlap: bobbin sits in the shallow open channel ----
    # The cotton_thread cylinder partially intersects the carved_hull because
    # the ski shuttle channel is shallow and the bobbin is mostly exposed.
    ctx.allow_overlap(
        "bobbin", "body",
        elem_a="cotton_thread", elem_b="carved_hull",
        reason=(
            "The bobbin sits in a shallow open running channel on the ski shuttle. "
            "The thread cylinder partially overlaps the hull walls because the "
            "channel is only 7mm deep while the bobbin rides above it."
        ),
    )

    # ---- Bobbin retained inside the ski hull footprint ----
    ctx.expect_within(
        bobbin, body, axes="xy", margin=0.002,
        name="bobbin is retained inside the ski shuttle footprint",
    )
    ctx.expect_overlap(
        bobbin, body, axes="x", min_overlap=0.18,
        name="bobbin spans the central open channel lengthwise",
    )

    # ---- Bobbin flanges clear the shallow channel floor ----
    ctx.expect_gap(
        bobbin, body, axis="z",
        min_gap=-0.001,
        negative_elem="cavity_floor",
        name="bobbin clears the shallow channel floor",
    )

    # ---- Bearings support the bobbin axle ----
    ctx.expect_contact(
        bobbin, body,
        elem_a="bobbin_core", elem_b="front_bearing",
        contact_tol=0.004,
        name="front bearing supports the bobbin axle",
    )
    ctx.expect_contact(
        bobbin, body,
        elem_a="bobbin_core", elem_b="rear_bearing",
        contact_tol=0.004,
        name="rear bearing supports the bobbin axle",
    )

    # ---- Ski hull proportions: carved_hull is much longer than wide ----
    hull_aabb = ctx.part_element_world_aabb(body, elem="carved_hull")
    if hull_aabb is not None:
        lo, hi = hull_aabb
        dx = hi[0] - lo[0]
        dy = hi[1] - lo[1]
        dz = hi[2] - lo[2]
        ctx.check(
            "ski hull has elongated ski proportions (length >> width)",
            dx > dy * 6.0,
            details=f"hull dx={dx:.4f} dy={dy:.4f}",
        )
        ctx.check(
            "ski hull is thin and flat (length >> height)",
            dx > dz * 8.0,
            details=f"hull dx={dx:.4f} dz={dz:.4f}",
        )

    # ---- Ski shuttle: upswept tips verified by carved_hull geometry ----
    # The carved_hull AABB top at the tips should exceed body_height (0.022)
    # due to the upswept tip ramps.
    if hull_aabb is not None:
        lo, hi = hull_aabb
        hull_top = hi[2]
        ctx.check(
            "carved_hull shows upswept ski tips (top exceeds body_height)",
            hull_top > 0.024,
            details=f"hull top z={hull_top:.4f}, expected > 0.024",
        )

    # ---- Revolute joint keeps bobbin centered while thread mark turns ----
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
        "revolute joint keeps bobbin centered while thread mark turns",
        rest is not None
        and turned is not None
        and abs(rest[0] - turned[0]) < 1e-6
        and rest_yz is not None
        and turned_yz is not None
        and abs(rest_yz[0] - turned_yz[0]) > 0.004,
        details=f"rest={rest}, turned={turned}, rest_tail={rest_yz}, turned_tail={turned_yz}",
    )

    return ctx.report()


object_model = build_object_model()
