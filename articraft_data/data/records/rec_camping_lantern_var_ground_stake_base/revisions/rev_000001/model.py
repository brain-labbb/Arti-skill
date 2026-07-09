from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


SPIKE_LENGTH = 0.180
HUB_HEIGHT = 0.038
LANTERN_RADIUS = 0.064


def _ring_profile_mesh(
    name: str,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    *,
    segments: int = 96,
):
    """Thin-walled revolved shell with visible lips/ridges."""
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=segments,
            start_cap="flat",
            end_cap="flat",
            lip_samples=4,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="stake_mount_camping_lantern",
        meta={
            "run_notes": (
                "Stake-mount trail camping lantern variant: single central ground-stake "
                "spike replaces the flat ribbed canister base. Still a portable battery "
                "area lantern — category remains Camping Lantern."
            )
        },
    )

    # ── Materials ──
    black = model.material("satin_black_plastic", rgba=(0.005, 0.005, 0.004, 1.0))
    dark = model.material("dark_graphite_rubber", rgba=(0.020, 0.020, 0.018, 1.0))
    clear = model.material("slightly_smoked_clear_plastic", rgba=(0.70, 0.92, 1.0, 0.32))
    chrome = model.material("polished_wire_steel", rgba=(0.82, 0.86, 0.86, 1.0))
    white = model.material("white_led_carrier", rgba=(0.91, 0.93, 0.90, 1.0))
    led = model.material("warm_led_lenses", rgba=(1.0, 0.86, 0.42, 1.0))
    screw = model.material("small_silver_fasteners", rgba=(0.72, 0.74, 0.72, 1.0))
    blackened_steel = model.material("blackened_steel_spike", rgba=(0.06, 0.06, 0.058, 1.0))

    # ══════════════════════════════════════════════════════════════════
    # lower_base: ground-stake spike + seating hub
    # ══════════════════════════════════════════════════════════════════
    lower_base = model.part("lower_base")

    # Main spike shaft: long pointed cone tapering up to a cylindrical hub
    # with a small seating flange at the top. Solid closed lathe.
    lower_base.visual(
        mesh_from_geometry(
            LatheGeometry(
                [
                    (0.001, -SPIKE_LENGTH),             # near-point tip
                    (0.003, -SPIKE_LENGTH + 0.012),     # tip shoulder
                    (0.006, -SPIKE_LENGTH + 0.040),     # lower spike body
                    (0.009, -SPIKE_LENGTH + 0.080),     # mid-lower spike
                    (0.012, -SPIKE_LENGTH + 0.120),     # mid spike
                    (0.015, -0.040),                     # upper spike
                    (0.019, -0.020),                     # spike-to-hub transition
                    (0.023, -0.006),                     # hub base
                    (0.023, HUB_HEIGHT - 0.010),         # hub cylindrical body
                    (0.040, HUB_HEIGHT - 0.005),         # wide seating flange start
                    (0.040, HUB_HEIGHT),                 # flange top edge
                    (0.000, HUB_HEIGHT),                 # close at axis
                ],
                segments=96,
                closed=True,
            ),
            "ground_stake_spike",
        ),
        material=blackened_steel,
        name="spike_shaft",
    )



    # ══════════════════════════════════════════════════════════════════
    # upper_lantern: light chamber assembly (static seat on hub)
    # ══════════════════════════════════════════════════════════════════
    upper_lantern = model.part("upper_lantern")

    # Lower collar: transitions from the narrow hub to the wider clear chamber.
    # Inner radius clears the hub flange (0.030) by ~4mm.
    upper_lantern.visual(
        _ring_profile_mesh(
            "lower_light_chamber_collar",
            [(0.036, 0.000), (0.052, 0.006), (0.052, 0.015), (0.050, 0.019)],
            [(0.034, 0.000), (0.045, 0.019)],
            segments=96,
        ),
        material=black,
        name="lower_collar",
    )

    # Transparent cylindrical chamber, built as a thin hollow tube.
    upper_lantern.visual(
        mesh_from_geometry(
            LatheGeometry(
                [(0.052, 0.015), (0.052, 0.137), (0.049, 0.137), (0.049, 0.015)],
                segments=96,
                closed=True,
            ),
            "transparent_light_chamber_tube",
        ),
        material=clear,
        name="clear_chamber",
    )

    # Subtle vertical clear facets/guards on the transparent tube.
    for i in range(4):
        angle = 2.0 * math.pi * i / 4 + math.pi / 4
        r = 0.0525
        upper_lantern.visual(
            Box((0.006, 0.004, 0.118)),
            origin=Origin(
                xyz=(r * math.cos(angle), r * math.sin(angle), 0.077),
                rpy=(0.0, 0.0, angle),
            ),
            material=clear,
            name=f"clear_post_{i}",
        )

    # Central LED strip and warm lens dots inside the chamber.
    upper_lantern.visual(
        Box((0.030, 0.010, 0.112)),
        origin=Origin(xyz=(0.0, -0.014, 0.078)),
        material=white,
        name="led_board",
    )
    upper_lantern.visual(
        Box((0.022, 0.014, 0.112)),
        origin=Origin(xyz=(0.0, -0.004, 0.078)),
        material=white,
        name="inner_reflector",
    )
    for row in range(5):
        z = 0.035 + row * 0.020
        for col, x in enumerate((-0.009, 0.009)):
            upper_lantern.visual(
                Cylinder(radius=0.0043, length=0.0025),
                origin=Origin(
                    xyz=(x, -0.0184, z),
                    rpy=(math.pi / 2, 0.0, 0.0),
                ),
                material=led,
                name=f"led_lens_{row}_{col}",
            )
            upper_lantern.visual(
                Sphere(radius=0.0021),
                origin=Origin(xyz=(x, -0.0202, z)),
                material=screw,
                name=f"led_core_{row}_{col}",
            )

    # Stepped top cap shell.
    upper_lantern.visual(
        _ring_profile_mesh(
            "stepped_top_cap_shell",
            [
                (0.052, 0.132),
                (0.061, 0.138),
                (0.061, 0.154),
                (0.057, 0.160),
                (0.048, 0.165),
            ],
            [(0.020, 0.132), (0.020, 0.162), (0.000, 0.165)],
            segments=96,
        ),
        material=black,
        name="top_cap",
    )
    upper_lantern.visual(
        Cylinder(radius=0.038, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.1665)),
        material=dark,
        name="top_disc",
    )

    # Small raised tabs around the top cap.
    for i in range(6):
        angle = 2.0 * math.pi * i / 6
        r = 0.057
        upper_lantern.visual(
            Box((0.022, 0.026, 0.018)),
            origin=Origin(
                xyz=(r * math.cos(angle), r * math.sin(angle), 0.149),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark,
            name=f"cap_tab_{i}",
        )

    # Side hinge bosses for the wire bail.
    for side, x in enumerate((-0.0635, 0.0635)):
        upper_lantern.visual(
            Cylinder(radius=0.006, length=0.012),
            origin=Origin(xyz=(x, 0.0, 0.158), rpy=(0.0, math.pi / 2, 0.0)),
            material=chrome,
            name=f"handle_boss_{side}",
        )

    # ══════════════════════════════════════════════════════════════════
    # wire_handle: fold-up carry bail
    # ══════════════════════════════════════════════════════════════════
    handle = model.part("wire_handle")
    handle.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (-0.056, 0.0, 0.000),
                    (-0.055, 0.0, 0.045),
                    (-0.044, 0.0, 0.102),
                    (-0.020, 0.0, 0.128),
                    (0.000, 0.0, 0.132),
                    (0.020, 0.0, 0.128),
                    (0.044, 0.0, 0.102),
                    (0.055, 0.0, 0.045),
                    (0.056, 0.0, 0.000),
                ],
                radius=0.0022,
                samples_per_segment=10,
                radial_segments=18,
                cap_ends=True,
            ),
            "fold_up_wire_bail",
        ),
        material=chrome,
        name="wire_bail",
    )
    handle.visual(
        Box((0.052, 0.006, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, 0.132)),
        material=chrome,
        name="top_grip_flat",
    )

    # ══════════════════════════════════════════════════════════════════
    # Articulations
    # ══════════════════════════════════════════════════════════════════

    # Fixed seat: lantern body sits statically on the hub (no telescoping slide).
    model.articulation(
        "base_to_lantern_seat",
        ArticulationType.FIXED,
        parent=lower_base,
        child=upper_lantern,
        origin=Origin(xyz=(0.0, 0.0, HUB_HEIGHT)),
    )

    # Revolute hinge for the fold-up wire carry handle.
    # q=0 shows the handle upright; negative q folds it down.
    model.articulation(
        "lantern_to_handle_hinge",
        ArticulationType.REVOLUTE,
        parent=upper_lantern,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, 0.158)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-1.45, upper=0.20),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower_base = object_model.get_part("lower_base")
    upper_lantern = object_model.get_part("upper_lantern")
    handle = object_model.get_part("wire_handle")
    hinge = object_model.get_articulation("lantern_to_handle_hinge")

    # ── Spike-specific assertions ──

    # The ground-stake spike must extend well below the hub seating flange.
    spike_aabb = ctx.part_element_world_aabb(lower_base, elem="spike_shaft")
    collar_aabb = ctx.part_element_world_aabb(upper_lantern, elem="lower_collar")
    ctx.check(
        "spike_shaft extends below hub as a ground stake",
        spike_aabb is not None
        and collar_aabb is not None
        and spike_aabb[0][2] < collar_aabb[0][2] - 0.12,
        details=(
            f"spike_min_z={spike_aabb[0][2] if spike_aabb else None}, "
            f"collar_min_z={collar_aabb[0][2] if collar_aabb else None}"
        ),
    )

    # The spike must be roughly centered under the lantern body.
    ctx.expect_overlap(
        upper_lantern,
        lower_base,
        axes="xy",
        elem_a="clear_chamber",
        elem_b="spike_shaft",
        min_overlap=0.018,
        name="clear chamber is coaxial with ground-stake spike",
    )

    # Lower collar seats on the hub flange with small clearance.
    ctx.expect_gap(
        upper_lantern,
        lower_base,
        axis="z",
        positive_elem="lower_collar",
        negative_elem="spike_shaft",
        max_gap=0.003,
        max_penetration=0.001,
        name="lower collar seats on hub flange",
    )

    # ── Handle articulation ──

    upright_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({hinge: -1.20}):
        folded_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "wire handle folds down on hinge",
        upright_aabb is not None
        and folded_aabb is not None
        and folded_aabb[1][2] < upright_aabb[1][2] - 0.035,
        details=f"upright={upright_aabb}, folded={folded_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
