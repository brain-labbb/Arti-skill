from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _trapezoid_prism(
    inner_width: float,
    outer_width: float,
    height: float,
    depth: float,
    *,
    centered_z: bool = False,
) -> MeshGeometry:
    """Trapezoidal tooth prism.

    X is tooth thickness along the pitch/tangent direction, Y is face width,
    and Z is tooth height/radial depth.  ``inner_width`` is the root width.
    """

    z0, z1 = (-height / 2.0, height / 2.0) if centered_z else (0.0, height)
    y0, y1 = -depth / 2.0, depth / 2.0
    xi, xo = inner_width / 2.0, outer_width / 2.0

    geom = MeshGeometry()
    verts = [
        (-xi, y0, z0),
        (xi, y0, z0),
        (xo, y0, z1),
        (-xo, y0, z1),
        (-xi, y1, z0),
        (xi, y1, z0),
        (xo, y1, z1),
        (-xo, y1, z1),
    ]
    for x, y, z in verts:
        geom.add_vertex(x, y, z)
    for tri in (
        (0, 1, 2),
        (0, 2, 3),
        (4, 7, 6),
        (4, 6, 5),
        (0, 4, 5),
        (0, 5, 1),
        (3, 2, 6),
        (3, 6, 7),
        (0, 3, 7),
        (0, 7, 4),
        (1, 5, 6),
        (1, 6, 2),
    ):
        geom.add_face(*tri)
    return geom


def _build_pinion(pinion_part, gear_tooth_mesh, pinion_mat, brushed, dark, blue,
                  pinion_teeth, gear_root_radius, gear_tooth_depth, gear_width):
    """Populate a pinion part with root wheel, teeth, hub, axle, and caps."""
    pinion_part.visual(
        Cylinder(radius=gear_root_radius, length=gear_width),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pinion_mat,
        name="root_wheel",
    )
    for idx in range(pinion_teeth):
        theta = math.pi + idx * (2.0 * math.pi / pinion_teeth)
        radius = gear_root_radius + gear_tooth_depth / 2.0 - 0.0008
        pinion_part.visual(
            gear_tooth_mesh,
            origin=Origin(
                xyz=(radius * math.sin(theta), 0.0, radius * math.cos(theta)),
                rpy=(0.0, theta, 0.0),
            ),
            material=pinion_mat,
            name=f"pinion_tooth_{idx:02d}",
        )
    pinion_part.visual(
        Cylinder(radius=0.026, length=0.052),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="raised_hub",
    )
    pinion_part.visual(
        Cylinder(radius=0.0132, length=0.130),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="axle",
    )
    pinion_part.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.0, -0.061, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blue,
        name="axle_cap_0",
    )
    pinion_part.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.0, 0.061, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blue,
        name="axle_cap_1",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="rack_and_pinion_slider",
        meta={"category": "Robotics / Rack-and-pinion slider"},
    )

    brushed = model.material("brushed_aluminum", rgba=(0.72, 0.75, 0.76, 1.0))
    dark = model.material("dark_bearing_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    rack_mat = model.material("satin_steel_rack", rgba=(0.62, 0.64, 0.63, 1.0))
    pinion_mat = model.material("warm_machined_gear", rgba=(0.74, 0.68, 0.58, 1.0))
    blue = model.material("blue_hub_cap", rgba=(0.20, 0.36, 0.55, 1.0))

    # Shared rack-and-pinion tooth geometry.  The rack pitch matches the pinion
    # circular pitch at the pitch radius used by the mimic relationship.
    tooth_pitch = 0.019
    pinion_teeth = 22
    pitch_radius = tooth_pitch * pinion_teeth / (2.0 * math.pi)
    gear_root_radius = pitch_radius - 0.006
    gear_tooth_depth = 0.014
    gear_width = 0.034
    rack_width = 0.060
    rack_bar_height = 0.026
    rack_bar_top_z = 0.085
    rack_tooth_height = 0.017
    rack_tooth_tip_z = rack_bar_top_z + rack_tooth_height
    pinion_center_z = rack_tooth_tip_z + gear_root_radius + 0.001

    # Dual-pinion layout: two pinions on the same shaft line at symmetric
    # X offsets, both meshing the single rack for anti-backlash preload.
    num_pinions = 2
    pinion_spacing = 0.28
    pinion_x_offsets = [
        -pinion_spacing / 2.0 + k * pinion_spacing for k in range(num_pinions)
    ]

    rack_tooth_mesh = mesh_from_geometry(
        _trapezoid_prism(0.011, 0.006, rack_tooth_height, rack_width),
        "rack_tooth_profile",
    )
    gear_tooth_mesh = mesh_from_geometry(
        _trapezoid_prism(0.009, 0.006, gear_tooth_depth, gear_width, centered_z=True),
        "pinion_tooth_profile",
    )
    bearing_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.018, tube=0.0048, radial_segments=20, tubular_segments=36),
        "bearing_collar_ring",
    )

    # ── Guide frame (root) ──────────────────────────────────────────────
    frame = model.part("guide_frame")
    frame.visual(
        Box((0.86, 0.24, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=brushed,
        name="base_plate",
    )
    frame.visual(
        Box((0.74, 0.032, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=dark,
        name="guide_rail",
    )
    for idx, x in enumerate((-0.405, 0.405)):
        frame.visual(
            Box((0.028, 0.12, 0.040)),
            origin=Origin(xyz=(x, 0.0, 0.038)),
            material=brushed,
            name=f"rail_stop_{idx}",
        )
    for idx, (x, y) in enumerate(
        ((-0.36, -0.09), (-0.36, 0.09), (0.36, -0.09), (0.36, 0.09))
    ):
        frame.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x, y, 0.021)),
            material=dark,
            name=f"mount_bolt_{idx}",
        )

    # Bearing supports duplicated per pinion: each pinion has its own pair
    # of bearing webs, saddles, and collars at the pinion's X position.
    for k in range(num_pinions):
        px = pinion_x_offsets[k]
        for j, y in enumerate((-0.048, 0.048)):
            bidx = k * 2 + j
            web_height = pinion_center_z - 0.038
            frame.visual(
                Box((0.050, 0.014, web_height)),
                origin=Origin(xyz=(px, y, 0.018 + web_height / 2.0)),
                material=brushed,
                name=f"bearing_web_{bidx}",
            )
            frame.visual(
                Box((0.060, 0.014, 0.014)),
                origin=Origin(xyz=(px, y, pinion_center_z - 0.026)),
                material=brushed,
                name=f"bearing_saddle_{bidx}",
            )
            frame.visual(
                bearing_mesh,
                origin=Origin(
                    xyz=(px, y, pinion_center_z), rpy=(math.pi / 2.0, 0.0, 0.0)
                ),
                material=dark,
                name=f"bearing_collar_{bidx}",
            )

    # ── Rack carriage ───────────────────────────────────────────────────
    rack = model.part("rack_carriage")
    rack.visual(
        Box((0.250, 0.085, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
        material=brushed,
        name="carriage_bridge",
    )
    for idx, y in enumerate((-0.034, 0.034)):
        rack.visual(
            Box((0.250, 0.018, 0.024)),
            origin=Origin(xyz=(0.0, y, 0.035)),
            material=brushed,
            name=f"carriage_shoe_{idx}",
        )
    rack.visual(
        Box((0.620, rack_width, rack_bar_height)),
        origin=Origin(xyz=(0.0, 0.0, rack_bar_top_z - rack_bar_height / 2.0)),
        material=rack_mat,
        name="rack_bar",
    )
    for idx, x in enumerate((-0.305, 0.305)):
        rack.visual(
            Box((0.016, rack_width + 0.010, rack_bar_height + 0.010)),
            origin=Origin(xyz=(x, 0.0, rack_bar_top_z - rack_bar_height / 2.0 + 0.001)),
            material=rack_mat,
            name=f"rack_end_cap_{idx}",
        )
    for idx, i in enumerate(range(-15, 15)):
        # Half-pitch phasing leaves a clear valley directly below the pinion.
        x = (i + 0.5) * tooth_pitch
        rack.visual(
            rack_tooth_mesh,
            origin=Origin(xyz=(x, 0.0, rack_bar_top_z - 0.0005)),
            material=rack_mat,
            name=f"rack_tooth_{idx:02d}",
        )

    # ── Pinion sub-assemblies (loop) ────────────────────────────────────
    for k in range(num_pinions):
        px = pinion_x_offsets[k]
        pinion_k = model.part(f"pinion_{k}")
        _build_pinion(
            pinion_k, gear_tooth_mesh, pinion_mat, brushed, dark, blue,
            pinion_teeth, gear_root_radius, gear_tooth_depth, gear_width,
        )
        model.articulation(
            f"pinion_spin_{k}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=pinion_k,
            origin=Origin(xyz=(px, 0.0, pinion_center_z)),
            # Positive rotation advances the rack/carriage along +X at the
            # bottom mesh point.
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0, velocity=4.0, lower=-1.0, upper=1.0
            ),
        )

    # ── Rack prismatic slide (single joint, shared by both pinions) ─────
    model.articulation(
        "rack_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=rack,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=0.25,
            lower=-pitch_radius,
            upper=pitch_radius,
        ),
        meta={"pitch_radius_m": pitch_radius, "tooth_pitch_m": tooth_pitch},
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("guide_frame")
    rack = object_model.get_part("rack_carriage")
    slide = object_model.get_articulation("rack_slide")

    # Collect per-pinion references
    pinions = []
    spins = []
    for k in range(2):
        pinions.append(object_model.get_part(f"pinion_{k}"))
        spins.append(object_model.get_articulation(f"pinion_spin_{k}"))

    # ── Dual-pinion structural assertions ───────────────────────────────
    ctx.check(
        "two pinion parts exist",
        len(pinions) == 2,
        details=f"found {len(pinions)} pinion parts",
    )
    for k in range(2):
        ctx.check(
            f"pinion_spin_{k} is REVOLUTE",
            spins[k].articulation_type == ArticulationType.REVOLUTE,
            details=f"pinion_spin_{k} type={spins[k].articulation_type}",
        )
    ctx.check(
        "rack_slide is PRISMATIC",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"slide={slide.articulation_type}",
    )
    ctx.check(
        "rack pitch metadata matches pinion size",
        0.060 < slide.meta.get("pitch_radius_m", 0.0) < 0.075
        and 0.018 < slide.meta.get("tooth_pitch_m", 0.0) < 0.020,
        details=f"meta={slide.meta}",
    )

    # ── Bearing and mesh checks per pinion ──────────────────────────────
    for k in range(2):
        pinion_k = pinions[k]
        for j in range(2):
            bidx = k * 2 + j
            ctx.allow_overlap(
                frame,
                pinion_k,
                elem_a=f"bearing_collar_{bidx}",
                elem_b="axle",
                reason=(
                    "The rotating axle is intentionally captured in the bearing "
                    "collar as a close journal fit."
                ),
            )
            ctx.expect_within(
                pinion_k,
                frame,
                axes="xz",
                inner_elem="axle",
                outer_elem=f"bearing_collar_{bidx}",
                margin=0.001,
                name=f"pinion_{k} axle centered in bearing collar {bidx}",
            )
            ctx.expect_overlap(
                pinion_k,
                frame,
                axes="y",
                elem_a="axle",
                elem_b=f"bearing_collar_{bidx}",
                min_overlap=0.006,
                name=f"pinion_{k} axle retained through bearing collar {bidx}",
            )

        # Each pinion meshes the shared rack — tooth envelopes intentionally
        # interleave at the pitch line (gear meshing contact).
        ctx.allow_overlap(
            pinion_k,
            rack,
            reason=(
                f"pinion_{k} teeth intentionally mesh with rack teeth at the "
                "pitch line; tooth envelopes interleave as required by the "
                "rack-and-pinion drive."
            ),
        )
        ctx.expect_overlap(
            pinion_k,
            rack,
            axes="x",
            min_overlap=0.020,
            name=f"pinion_{k} centered over rack teeth",
        )
        ctx.expect_overlap(
            pinion_k,
            rack,
            axes="z",
            min_overlap=0.006,
            name=f"pinion_{k} tooth envelopes vertically intermesh with rack",
        )

    # ── Pinions separated along X ───────────────────────────────────────
    p0_center = ctx.part_world_position(pinions[0])
    p1_center = ctx.part_world_position(pinions[1])
    ctx.check(
        "pinions separated along X axis",
        p0_center is not None
        and p1_center is not None
        and abs(p1_center[0] - p0_center[0]) > 0.20,
        details=f"pinion_0={p0_center}, pinion_1={p1_center}",
    )

    # ── Rack guidance ───────────────────────────────────────────────────
    ctx.expect_within(
        rack,
        frame,
        axes="y",
        margin=0.002,
        name="rack carriage stays within guide width",
    )
    ctx.expect_gap(
        rack,
        frame,
        axis="z",
        positive_elem="rack_bar",
        negative_elem="guide_rail",
        min_gap=0.006,
        max_gap=0.020,
        name="rack is carried above guide rail",
    )

    # ── Pose: rack slides in +X, both pinions spin ──────────────────────
    rest_pos = ctx.part_world_position(rack)
    with ctx.pose({slide: 0.055, spins[0]: 0.8, spins[1]: 0.8}):
        moved_pos = ctx.part_world_position(rack)
        ctx.expect_within(
            rack,
            frame,
            axes="y",
            margin=0.002,
            name="sliding rack remains laterally guided",
        )
        ctx.expect_overlap(
            rack,
            frame,
            axes="x",
            min_overlap=0.45,
            name="extended rack remains supported on base",
        )
    ctx.check(
        "positive rack prismatic motion travels in +X",
        rest_pos is not None
        and moved_pos is not None
        and moved_pos[0] > rest_pos[0] + 0.045,
        details=f"rest={rest_pos}, moved={moved_pos}",
    )

    return ctx.report()


object_model = build_object_model()
