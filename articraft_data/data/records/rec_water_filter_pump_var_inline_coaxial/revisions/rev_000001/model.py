from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="inline_coaxial_water_filter_pump",
        meta={
            "run_notes": (
                "Inline coaxial hand-pump filter variant: pump barrel and plunger "
                "sit directly on top of and share the axis of the filter cartridge "
                "(single vertical column). Replaces the parent twin-column layout "
                "(offset pump_sleeve + bridges) with a stacked coaxial design. "
                "No classification conflict suspected."
            )
        },
    )

    olive = model.material("olive_green_plastic", rgba=(0.22, 0.38, 0.17, 1.0))
    dark_olive = model.material("dark_olive_rubber", rgba=(0.10, 0.20, 0.08, 1.0))
    light_olive = model.material("molded_highlight_green", rgba=(0.34, 0.52, 0.25, 1.0))
    metal = model.material("polished_stainless_rod", rgba=(0.78, 0.82, 0.78, 1.0))
    clear = model.material("clear_flexible_hose", rgba=(0.92, 0.98, 1.0, 0.43))
    grey = model.material("dark_gray_fitting", rgba=(0.20, 0.22, 0.20, 1.0))

    # ── Body (root part): single coaxial column ──────────────────────────

    body = model.part("body")

    # Coaxial axis X for the entire column
    CX = 0.018

    # Molded base cap (wider rubber foot)
    body.visual(
        Cylinder(radius=0.031, length=0.022),
        origin=Origin(xyz=(CX, 0.0, 0.016)),
        material=dark_olive,
        name="base_cap",
    )

    # Main filter cartridge body (slimmer per companion variation ⑤)
    body.visual(
        Cylinder(radius=0.028, length=0.200),
        origin=Origin(xyz=(CX, 0.0, 0.127)),
        material=olive,
        name="filter_body",
    )

    # Transition ring between filter cartridge and pump barrel
    transition_mesh = mesh_from_geometry(
        KnobGeometry(
            0.060,
            0.022,
            body_style="cylindrical",
            grip=KnobGrip(style="ribbed", count=28, depth=0.0014, width=0.002),
            edge_radius=0.001,
        ),
        "ribbed_filter_cap",
    )
    body.visual(
        transition_mesh,
        origin=Origin(xyz=(CX, 0.0, 0.238)),
        material=olive,
        name="ribbed_filter_cap",
    )

    # Coaxial pump barrel (upper cylinder, slightly narrower than filter)
    body.visual(
        Cylinder(radius=0.024, length=0.100),
        origin=Origin(xyz=(CX, 0.0, 0.299)),
        material=olive,
        name="pump_barrel",
    )

    # Top guide collar at the pump barrel entry
    collar_mesh = mesh_from_geometry(
        KnobGeometry(
            0.052,
            0.018,
            body_style="cylindrical",
            grip=KnobGrip(style="ribbed", count=24, depth=0.0012, width=0.0018),
            edge_radius=0.0008,
        ),
        "pump_top_collar",
    )
    body.visual(
        collar_mesh,
        origin=Origin(xyz=(CX, 0.0, 0.358)),
        material=olive,
        name="pump_top_collar",
    )

    # Recessed front grip/label panel on filter body
    body.visual(
        Box((0.013, 0.004, 0.110)),
        origin=Origin(xyz=(CX, -0.0285, 0.130)),
        material=light_olive,
        name="front_recess_panel",
    )

    # Side outlet barb near top of filter body
    body.visual(
        Cylinder(radius=0.0085, length=0.038),
        origin=Origin(xyz=(0.056, 0.0, 0.210), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=olive,
        name="outlet_barb",
    )

    # Small bleed/air-release cap on transition ring area
    body.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.043, 0.0, 0.245)),
        material=dark_olive,
        name="bleed_cap",
    )

    # Short clear curved hose from outlet downward
    hose_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.072, 0.0, 0.210),
                (0.112, 0.0, 0.200),
                (0.150, 0.0, 0.160),
                (0.180, 0.0, 0.105),
                (0.205, 0.0, 0.055),
            ],
            radius=0.0042,
            samples_per_segment=18,
            radial_segments=20,
            cap_ends=True,
            up_hint=(0.0, 1.0, 0.0),
        ),
        "curved_clear_hose",
    )
    body.visual(hose_mesh, material=clear, name="curved_clear_hose")

    # Hose end fitting
    body.visual(
        Cylinder(radius=0.0068, length=0.027),
        origin=Origin(xyz=(0.204, 0.0, 0.048), rpy=(0.0, 2.72, 0.0)),
        material=grey,
        name="hose_end_fitting",
    )

    # ── Plunger (child part): slides inside coaxial pump barrel ────────────

    plunger = model.part("plunger")

    # Articulation origin: top of pump barrel at the guide collar entry
    ARTIC_Z = 0.349

    # Stainless pump rod captured inside the coaxial barrel
    plunger.visual(
        Cylinder(radius=0.004, length=0.250),
        origin=Origin(xyz=(0.0, 0.0, -0.104)),
        material=metal,
        name="plunger_rod",
    )

    # Handle stem (olive plastic, connects rod to T-grip)
    plunger.visual(
        Cylinder(radius=0.0105, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=olive,
        name="handle_stem",
    )

    # T-handle grip (rotated capsule for ergonomic crossbar)
    grip_mesh = mesh_from_geometry(
        CapsuleGeometry(
            radius=0.012, length=0.108, radial_segments=24, height_segments=8
        ).rotate_y(math.pi / 2.0),
        "t_handle_grip",
    )
    plunger.visual(
        grip_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=olive,
        name="handle_grip",
    )

    # ── Articulation: prismatic pump stroke ────────────────────────────────

    model.articulation(
        "body_to_plunger",
        ArticulationType.PRISMATIC,
        parent=body,
        child=plunger,
        # Origin at the coaxial guide collar entry, on the shared column axis.
        origin=Origin(xyz=(CX, 0.0, ARTIC_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0, velocity=0.35, lower=0.0, upper=0.075
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("body_to_plunger")

    # ── Coaxial structural claim ──────────────────────────────────────────
    body_visual_names = {v.name for v in body.visuals}

    ctx.check(
        "coaxial pump_barrel replaces offset pump_sleeve",
        "pump_barrel" in body_visual_names
        and "pump_top_collar" in body_visual_names
        and "pump_sleeve" not in body_visual_names
        and "lower_bridge" not in body_visual_names
        and "upper_bridge" not in body_visual_names,
        details=f"body visuals={sorted(body_visual_names)}",
    )

    # ── Single prismatic joint preserved ──────────────────────────────────
    ctx.check(
        "single sliding pump mechanism",
        slide.motion_limits is not None
        and slide.motion_limits.lower == 0.0
        and slide.motion_limits.upper is not None
        and 0.06 <= slide.motion_limits.upper <= 0.09,
        details=f"limits={slide.motion_limits}",
    )

    # ── Required functional visuals present ───────────────────────────────
    ctx.check(
        "reference details present",
        all(
            name in body_visual_names
            for name in (
                "filter_body",
                "ribbed_filter_cap",
                "outlet_barb",
                "curved_clear_hose",
                "base_cap",
                "pump_barrel",
                "pump_top_collar",
            )
        ),
        details=f"body visuals={sorted(body_visual_names)}",
    )

    # ── Intentional overlap: rod captured inside coaxial column proxy ─────
    # The pump rod passes through the filter cartridge, transition ring,
    # pump barrel, and top collar in a single coaxial column.
    ctx.allow_overlap(
        body,
        plunger,
        elem_a="filter_body",
        elem_b="plunger_rod",
        reason="The pump rod extends into the filter cartridge to drive the inline pump piston.",
    )
    ctx.allow_overlap(
        body,
        plunger,
        elem_a="ribbed_filter_cap",
        elem_b="plunger_rod",
        reason="The transition ring surrounds and guides the rod between filter and pump sections.",
    )
    ctx.allow_overlap(
        body,
        plunger,
        elem_a="pump_barrel",
        elem_b="plunger_rod",
        reason="The pump rod is intentionally captured inside the coaxial pump barrel proxy.",
    )
    ctx.allow_overlap(
        body,
        plunger,
        elem_a="pump_top_collar",
        elem_b="plunger_rod",
        reason="The top guide collar surrounds and guides the sliding pump rod.",
    )

    # ── Rod centered and retained across the coaxial column ───────────────
    ctx.expect_within(
        plunger,
        body,
        axes="xy",
        inner_elem="plunger_rod",
        outer_elem="filter_body",
        margin=0.002,
        name="rod centered in filter body (coaxial column)",
    )
    ctx.expect_within(
        plunger,
        body,
        axes="xy",
        inner_elem="plunger_rod",
        outer_elem="pump_barrel",
        margin=0.002,
        name="rod centered in coaxial pump barrel",
    )
    ctx.expect_overlap(
        plunger,
        body,
        axes="z",
        elem_a="plunger_rod",
        elem_b="filter_body",
        min_overlap=0.050,
        name="rest pose keeps rod inserted in filter body",
    )
    ctx.expect_overlap(
        plunger,
        body,
        axes="z",
        elem_a="plunger_rod",
        elem_b="pump_barrel",
        min_overlap=0.040,
        name="rest pose keeps rod inserted in pump barrel",
    )

    # ── Handle clearance above collar ─────────────────────────────────────
    ctx.expect_gap(
        plunger,
        body,
        axis="z",
        min_gap=0.04,
        positive_elem="handle_grip",
        negative_elem="pump_top_collar",
        name="handle stands above pump top collar",
    )

    # ── Articulated pose: rod still captured, stroke moves upward ─────────
    rest_pos = ctx.part_world_position(plunger)
    with ctx.pose({slide: slide.motion_limits.upper}):
        ctx.expect_within(
            plunger,
            body,
            axes="xy",
            inner_elem="plunger_rod",
            outer_elem="pump_barrel",
            margin=0.002,
            name="extended rod remains centered in pump barrel",
        )
        ctx.expect_overlap(
            plunger,
            body,
            axes="z",
            elem_a="plunger_rod",
            elem_b="pump_barrel",
            min_overlap=0.010,
            name="extended pose retains insertion in pump barrel",
        )
        ctx.expect_overlap(
            plunger,
            body,
            axes="z",
            elem_a="plunger_rod",
            elem_b="filter_body",
            min_overlap=0.010,
            name="extended pose retains insertion in filter body",
        )
        extended_pos = ctx.part_world_position(plunger)

    ctx.check(
        "positive pump stroke moves upward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + 0.065,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
