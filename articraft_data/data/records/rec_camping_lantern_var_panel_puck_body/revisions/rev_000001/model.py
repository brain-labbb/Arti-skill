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
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


BASE_HEIGHT = 0.080
PUCK_RADIUS = 0.082
PUCK_HEIGHT = 0.030
HINGE_Z_LOCAL = 0.020


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
        name="flat_puck_camping_lantern",
        meta={
            "run_notes": (
                "Flat disc/puck camping lantern variant: tall cylindrical light chamber "
                "collapsed into a short wide diffuser disc with downward-facing COB panel. "
                "Telescoping slide replaced with static seat; fold-up wire bail retained as "
                "the hang loop. Neutral gray-and-white colorway."
            )
        },
    )

    # ── Materials: neutral gray-and-white colorway ──
    gray = model.material("neutral_gray_housing", rgba=(0.40, 0.40, 0.42, 1.0))
    light_gray = model.material("light_gray_grip", rgba=(0.55, 0.55, 0.57, 1.0))
    dark_gray = model.material("dark_gray_cap", rgba=(0.25, 0.25, 0.27, 1.0))
    clear = model.material("frosted_diffuser", rgba=(0.92, 0.94, 0.92, 0.40))
    chrome = model.material("polished_wire_steel", rgba=(0.82, 0.86, 0.86, 1.0))
    white = model.material("white_pcb_carrier", rgba=(0.91, 0.93, 0.90, 1.0))
    cob_yellow = model.material("cob_phosphor_panel", rgba=(1.0, 0.88, 0.50, 1.0))
    screw = model.material("silver_fastener", rgba=(0.72, 0.74, 0.72, 1.0))

    # ── lower_base: ribbed cylindrical base ──
    lower_base = model.part("lower_base")
    lower_base.visual(
        _ring_profile_mesh(
            "ribbed_lower_base_shell",
            [
                (0.056, 0.000),
                (0.064, 0.006),
                (0.064, 0.017),
                (0.059, 0.023),
                (0.059, 0.035),
                (0.062, 0.039),
                (0.062, 0.045),
                (0.058, 0.049),
                (0.058, 0.075),
                (0.060, 0.078),
                (0.058, 0.080),
            ],
            [
                (0.047, 0.006),
                (0.052, 0.014),
                (0.054, 0.070),
                (0.054, 0.080),
            ],
        ),
        material=gray,
        name="base_shell",
    )
    # Vertical grip ribs on the base canister
    for i in range(18):
        angle = 2.0 * math.pi * i / 18
        r = 0.0625
        lower_base.visual(
            Box((0.006, 0.010, 0.038)),
            origin=Origin(
                xyz=(r * math.cos(angle), r * math.sin(angle), 0.048),
                rpy=(0.0, 0.0, angle),
            ),
            material=light_gray,
            name=f"base_rib_{i}",
        )
    # Bottom castellated grip pads
    for i in range(18):
        angle = 2.0 * math.pi * (i + 0.5) / 18
        r = 0.062
        lower_base.visual(
            Box((0.007, 0.014, 0.015)),
            origin=Origin(
                xyz=(r * math.cos(angle), r * math.sin(angle), 0.008),
                rpy=(0.0, 0.0, angle),
            ),
            material=light_gray,
            name=f"foot_lug_{i}",
        )

    # ── upper_lantern: flat puck body ──
    upper_lantern = model.part("upper_lantern")

    # Seating plate bridging the narrower base to the wider puck
    upper_lantern.visual(
        mesh_from_geometry(
            LatheGeometry(
                [
                    (0.054, -0.004),
                    (0.078, -0.004),
                    (0.078, 0.000),
                    (0.054, 0.000),
                ],
                segments=96,
                closed=True,
            ),
            "puck_seating_plate",
        ),
        material=gray,
        name="seat_lip",
    )

    # Flat diffuser disc (replaces tall transparent_light_chamber_tube)
    upper_lantern.visual(
        mesh_from_geometry(
            LatheGeometry(
                [
                    (0.010, -0.002),
                    (0.070, -0.002),
                    (0.070, 0.004),
                    (0.010, 0.004),
                ],
                segments=96,
                closed=True,
            ),
            "diffuser_disc",
        ),
        material=clear,
        name="clear_chamber",
    )

    # Puck housing shell — main body of the flat disc
    upper_lantern.visual(
        _ring_profile_mesh(
            "puck_housing_shell",
            [
                (0.076, 0.000),
                (0.082, 0.004),
                (0.082, 0.026),
                (0.076, PUCK_HEIGHT),
            ],
            [
                (0.070, 0.004),
                (0.070, 0.026),
                (0.020, PUCK_HEIGHT),
            ],
            segments=96,
        ),
        material=gray,
        name="puck_shell",
    )

    # Downward-facing COB panel (flat disc carrier)
    upper_lantern.visual(
        Cylinder(radius=0.070, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=white,
        name="led_board",
    )

    # COB phosphor layer on the bottom face of the carrier (visible from below)
    upper_lantern.visual(
        Cylinder(radius=0.058, length=0.001),
        origin=Origin(xyz=(0.0, 0.0, 0.0045)),
        material=cob_yellow,
        name="cob_phosphor",
    )

    # Reflector plate above the COB panel
    upper_lantern.visual(
        Cylinder(radius=0.070, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=screw,
        name="inner_reflector",
    )

    # Stepped top cap
    upper_lantern.visual(
        _ring_profile_mesh(
            "stepped_top_cap_shell",
            [
                (0.048, 0.025),
                (0.055, 0.028),
                (0.055, 0.034),
                (0.050, 0.037),
                (0.042, 0.040),
            ],
            [(0.018, 0.025), (0.018, 0.037), (0.000, 0.040)],
            segments=96,
        ),
        material=dark_gray,
        name="top_cap",
    )

    # Top disc on cap
    upper_lantern.visual(
        Cylinder(radius=0.035, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0425)),
        material=light_gray,
        name="top_disc",
    )

    # Raised tabs around the top cap
    for i in range(6):
        angle = 2.0 * math.pi * i / 6
        r = 0.052
        upper_lantern.visual(
            Box((0.018, 0.022, 0.012)),
            origin=Origin(
                xyz=(r * math.cos(angle), r * math.sin(angle), 0.031),
                rpy=(0.0, 0.0, angle),
            ),
            material=light_gray,
            name=f"cap_tab_{i}",
        )

    # Side hinge bosses for the wire bail
    for side, x in enumerate((-0.084, 0.084)):
        upper_lantern.visual(
            Cylinder(radius=0.006, length=0.014),
            origin=Origin(
                xyz=(x, 0.0, HINGE_Z_LOCAL), rpy=(0.0, math.pi / 2, 0.0)
            ),
            material=chrome,
            name=f"handle_boss_{side}",
        )

    # ── wire_handle: fold-up bail handle ──
    handle = model.part("wire_handle")
    handle.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (-0.078, 0.0, 0.000),
                    (-0.076, 0.0, 0.012),
                    (-0.064, 0.0, 0.035),
                    (-0.034, 0.0, 0.050),
                    (0.000, 0.0, 0.054),
                    (0.034, 0.0, 0.050),
                    (0.064, 0.0, 0.035),
                    (0.076, 0.0, 0.012),
                    (0.078, 0.0, 0.000),
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
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=chrome,
        name="top_grip_flat",
    )

    # ── Articulations ──

    # Fixed seat: puck sits statically on base (no telescoping slide)
    model.articulation(
        "base_to_lantern_seat",
        ArticulationType.FIXED,
        parent=lower_base,
        child=upper_lantern,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
    )

    # Fold-up wire bail hinge (primary moving joint)
    model.articulation(
        "lantern_to_handle_hinge",
        ArticulationType.REVOLUTE,
        parent=upper_lantern,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z_LOCAL)),
        axis=(1.0, 0.0, 0.0),
        # q=0 shows the carry handle upright; negative q folds it down.
        motion_limits=MotionLimits(
            effort=4.0, velocity=2.0, lower=-1.45, upper=0.20
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower_base = object_model.get_part("lower_base")
    upper_lantern = object_model.get_part("upper_lantern")
    handle = object_model.get_part("wire_handle")
    hinge = object_model.get_articulation("lantern_to_handle_hinge")

    # Category identity
    ctx.check(
        "lantern identity",
        "lantern" in object_model.name,
        details="Object should read as a camping lantern.",
    )

    # Flat puck form factor: upper_lantern width must dominate its height
    puck_aabb = ctx.part_world_aabb(upper_lantern)
    if puck_aabb is not None:
        dx = puck_aabb[1][0] - puck_aabb[0][0]
        dy = puck_aabb[1][1] - puck_aabb[0][1]
        dz = puck_aabb[1][2] - puck_aabb[0][2]
        ctx.check(
            "upper_lantern reads as flat puck (width >> height)",
            dx > 2.0 * dz and dy > 2.0 * dz,
            details=f"puck dims: dx={dx:.4f}, dy={dy:.4f}, dz={dz:.4f}",
        )

    # Clear chamber is a flat diffuser disc, not a tall tube
    diffuser_aabb = ctx.part_element_world_aabb(upper_lantern, elem="clear_chamber")
    if diffuser_aabb is not None:
        dd_width = diffuser_aabb[1][0] - diffuser_aabb[0][0]
        dd_height = diffuser_aabb[1][2] - diffuser_aabb[0][2]
        ctx.check(
            "clear_chamber is flat diffuser disc (wide, not tall)",
            dd_width > 3.0 * dd_height,
            details=f"diffuser width={dd_width:.4f}, height={dd_height:.4f}",
        )

    # COB panel (led_board) faces downward: below puck vertical center
    cob_aabb = ctx.part_element_world_aabb(upper_lantern, elem="led_board")
    if cob_aabb is not None and puck_aabb is not None:
        puck_mid_z = (puck_aabb[0][2] + puck_aabb[1][2]) / 2.0
        cob_center_z = (cob_aabb[0][2] + cob_aabb[1][2]) / 2.0
        ctx.check(
            "led_board COB panel is in lower half of puck (downward-facing)",
            cob_center_z < puck_mid_z,
            details=f"cob_center_z={cob_center_z:.4f}, puck_mid_z={puck_mid_z:.4f}",
        )

    # Puck seats on base (XY coaxial alignment)
    ctx.expect_overlap(
        upper_lantern,
        lower_base,
        axes="xy",
        elem_a="puck_shell",
        elem_b="base_shell",
        min_overlap=0.060,
        name="puck body overlaps base footprint in XY",
    )

    # Handle hinge folds down
    upright_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({hinge: -1.20}):
        folded_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "wire handle folds down on hinge",
        upright_aabb is not None
        and folded_aabb is not None
        and folded_aabb[1][2] < upright_aabb[1][2] - 0.020,
        details=f"upright_top={upright_aabb[1][2] if upright_aabb else None}, "
        f"folded_top={folded_aabb[1][2] if folded_aabb else None}",
    )

    # Handle upright pose extends above the puck body
    if upright_aabb is not None and puck_aabb is not None:
        ctx.check(
            "handle arches above puck when upright",
            upright_aabb[1][2] > puck_aabb[1][2] + 0.020,
            details=f"handle_top={upright_aabb[1][2]:.4f}, puck_top={puck_aabb[1][2]:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
