from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_rect_profile(
    length: float,
    width: float,
    radius: float,
    *,
    segments: int = 8,
) -> list[tuple[float, float]]:
    """Centered rounded rectangle in XY, with length along X."""
    r = min(radius, length / 2.0 - 1e-5, width / 2.0 - 1e-5)
    hx = length / 2.0 - r
    hy = width / 2.0 - r
    centers = [(hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy)]
    angle_ranges = [
        (0.0, math.pi / 2.0),
        (math.pi / 2.0, math.pi),
        (math.pi, 3.0 * math.pi / 2.0),
        (3.0 * math.pi / 2.0, 2.0 * math.pi),
    ]
    pts: list[tuple[float, float]] = []
    for (cx, cy), (a0, a1) in zip(centers, angle_ranges):
        for j in range(segments + 1):
            a = a0 + (a1 - a0) * j / segments
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


# ---------- body mesh: flat plank with channel + forked ends ----------

_PLANK_LENGTH = 0.36
_PLANK_WIDTH = 0.054
_PLANK_HEIGHT = 0.014
_CORNER_RADIUS = 0.007
_CHANNEL_LENGTH = 0.22
_CHANNEL_WIDTH = 0.036
_CHANNEL_DEPTH = 0.007
_NOTCH_DX = 0.058
_NOTCH_DY = 0.014
_NOTCH_DEPTH = 0.012
_NOTCH_X = 0.150


def _make_body_mesh():
    """Flat rectangular plank with a shallow open-top channel and forked end notches."""
    outline = _rounded_rect_profile(_PLANK_LENGTH, _PLANK_WIDTH, _CORNER_RADIUS)

    plank = (
        cq.Workplane("XY")
        .polyline(outline)
        .close()
        .extrude(_PLANK_HEIGHT)
    )

    # Shallow central channel (open-top slot for the bobbin thread).
    plank = (
        plank
        .faces(">Z")
        .workplane(centerOption="CenterOfBoundBox")
        .slot2D(_CHANNEL_LENGTH, _CHANNEL_WIDTH)
        .cutBlind(-_CHANNEL_DEPTH)
    )

    # Fork notches: blind U-slots from the top at each end.
    # Use a single stable workplane to avoid drift between successive face selections.
    plank = (
        plank
        .faces(">Z")
        .workplane(centerOption="CenterOfBoundBox")
        .pushPoints([(-_NOTCH_X, 0.0), (_NOTCH_X, 0.0)])
        .rect(_NOTCH_DX, _NOTCH_DY)
        .cutBlind(-_NOTCH_DEPTH)
    )

    return plank


def _torus_mesh(name: str, *, normal: str):
    torus = TorusGeometry(0.0075, 0.0016, radial_segments=20, tubular_segments=36)
    if normal == "x":
        torus.rotate_y(math.pi / 2.0)
    elif normal == "y":
        torus.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(torus, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="loom_shuttle",
        meta={
            "reference_note": (
                "Flat stick shuttle for rigid-heddle / frame looms: "
                "rectangular plank body with open-top channel and forked end notches, "
                "carrying an exposed weft bobbin on a revolute axle (axis X)."
            )
        },
    )

    wood = model.material("pale_oiled_pine", rgba=(0.68, 0.48, 0.22, 1.0))
    dark_wood = model.material("dark_channel_floor", rgba=(0.22, 0.14, 0.07, 1.0))
    grain = model.material("subtle_dark_grain", rgba=(0.32, 0.18, 0.06, 1.0))
    black = model.material("blackened_eyelet", rgba=(0.015, 0.018, 0.017, 1.0))
    yarn = model.material("aged_cotton_thread", rgba=(0.86, 0.78, 0.60, 1.0))
    red = model.material("red_thread_end", rgba=(0.85, 0.05, 0.03, 1.0))
    bobbin_wood = model.material("bobbin_end_wood", rgba=(0.38, 0.20, 0.08, 1.0))

    # ---- Body: flat plank with open channel and forked ends ----
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_mesh(), "flat_plank_body", tolerance=0.0007),
        material=wood,
        name="carved_hull",
    )
    # Dark worn floor of the open channel, slightly proud so the hollow reads.
    body.visual(
        Box((0.180, 0.028, 0.0008)),
        origin=Origin(xyz=(0.0, 0.0, 0.0074)),
        material=dark_wood,
        name="cavity_floor",
    )
    # Bearing saddles: small posts rising from the channel floor to cradle the axle.
    for x, nm in [(-0.100, "front_bearing"), (0.100, "rear_bearing")]:
        body.visual(
            Box((0.010, 0.020, 0.009)),
            origin=Origin(xyz=(x, 0.0, 0.0115)),
            material=wood,
            name=f"{nm}_saddle",
        )
        body.visual(
            _torus_mesh(f"{nm}_ring", normal="x"),
            origin=Origin(xyz=(x, 0.0, 0.023)),
            material=black,
            name=nm,
        )
    # Side eyelets on the plank long edges.
    for x, y, z, nm in [
        (-0.138, -0.027, 0.007, "side_eyelet_0"),
        (0.138, 0.027, 0.007, "side_eyelet_1"),
    ]:
        body.visual(
            _torus_mesh(f"{nm}_mesh", normal="y"),
            origin=Origin(xyz=(x, y, z)),
            material=black,
            name=nm,
        )
    # Small dark drilled holes on the plank top, outside the channel.
    for i, (x, y) in enumerate(
        [(-0.105, 0.022), (-0.040, 0.022), (0.040, 0.022), (0.105, 0.022)]
    ):
        body.visual(
            Cylinder(radius=0.0025, length=0.0010),
            origin=Origin(xyz=(x, y, 0.0142)),
            material=black,
            name=f"top_hole_{i}",
        )
    # Subtle wood-grain strips on the plank top, outside the channel.
    for i, (x, y, sx, yaw) in enumerate(
        [
            (-0.100, 0.024, 0.050, 0.04),
            (-0.020, 0.024, 0.065, -0.03),
            (0.060, 0.024, 0.055, 0.05),
            (-0.110, -0.024, 0.040, -0.06),
            (0.030, -0.024, 0.075, 0.03),
            (0.110, -0.024, 0.035, -0.05),
        ]
    ):
        body.visual(
            Box((sx, 0.0018, 0.0006)),
            origin=Origin(xyz=(x, y, 0.0141), rpy=(0.0, 0.0, yaw)),
            material=grain,
            name=f"wood_grain_{i}",
        )

    # ---- Bobbin: child frame on the axle centerline ----
    bobbin = model.part("bobbin")
    cyl_to_x = (0.0, math.pi / 2.0, 0.0)
    bobbin.visual(
        Cylinder(radius=0.0061, length=0.220),
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
            Cylinder(radius=0.0142, length=0.010),
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
    # Off-axis red thread tail makes bobbin rotation visually apparent.
    bobbin.visual(
        Box((0.014, 0.004, 0.003)),
        origin=Origin(xyz=(0.071, 0.001, 0.0138), rpy=(0.0, 0.0, 0.08)),
        material=red,
        name="red_thread",
    )

    # ---- Articulation: revolute axle along X ----
    model.articulation(
        "body_to_bobbin",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bobbin,
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4, velocity=8.0, lower=-math.pi, upper=math.pi
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    bobbin = object_model.get_part("bobbin")
    joint = object_model.get_articulation("body_to_bobbin")

    # ---- Variant-specific: body is a flat rectangular plank ----
    body_hull_aabb = ctx.part_element_world_aabb(body, elem="carved_hull")
    if body_hull_aabb is not None:
        lo, hi = body_hull_aabb
        dx = hi[0] - lo[0]
        dy = hi[1] - lo[1]
        dz = hi[2] - lo[2]
        ctx.check(
            "body is a flat rectangular plank (length >> width >> height)",
            dy > 3.0 * dz and dx > 15.0 * dz,
            details=f"plank dims: dx={dx:.4f}, dy={dy:.4f}, dz={dz:.4f}",
        )

    # Bobbin sits within the plank footprint on XY.
    ctx.expect_within(
        bobbin,
        body,
        axes="xy",
        margin=0.004,
        name="bobbin is retained inside the plank footprint",
    )
    # Bobbin spans the central channel lengthwise.
    ctx.expect_overlap(
        bobbin,
        body,
        axes="x",
        min_overlap=0.18,
        name="bobbin spans the central channel lengthwise",
    )
    # Bobbin flanges clear the channel floor.
    ctx.expect_gap(
        bobbin,
        body,
        axis="z",
        min_gap=0.0002,
        negative_elem="cavity_floor",
        name="bobbin clears the channel floor",
    )
    # Bearings support the bobbin axle.
    ctx.expect_contact(
        bobbin,
        body,
        elem_a="bobbin_core",
        elem_b="front_bearing",
        contact_tol=0.004,
        name="front bearing supports the bobbin axle",
    )
    ctx.expect_contact(
        bobbin,
        body,
        elem_a="bobbin_core",
        elem_b="rear_bearing",
        contact_tol=0.004,
        name="rear bearing supports the bobbin axle",
    )

    # Revolute joint rotation: bobbin stays centered, red thread mark moves.
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
        details=(
            f"rest={rest}, turned={turned}, "
            f"rest_tail={rest_yz}, turned_tail={turned_yz}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
