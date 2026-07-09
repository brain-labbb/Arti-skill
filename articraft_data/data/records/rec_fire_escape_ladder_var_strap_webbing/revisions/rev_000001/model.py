from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _cyl_x(part, name: str, length: float, radius: float, xyz, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _cyl_y(part, name: str, length: float, radius: float, xyz, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _cyl_z(part, name: str, length: float, radius: float, xyz, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="portable_webbing_escape_ladder",
        meta={
            "run_notes": (
                "Fork variant: replaced rigid steel side rails with flat woven webbing straps "
                "and replaced wall-mount hardware/cage with a compact folding top hook pivot. "
                "Companion variation: high-visibility yellow webbing (palette only)."
            )
        },
    )

    # ── Materials ──
    yellow_webbing = model.material("hv_yellow_webbing", rgba=(0.93, 0.82, 0.08, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.72, 0.76, 0.75, 1.0))
    dark_metal = model.material("dark_metal_fittings", rgba=(0.14, 0.15, 0.15, 1.0))

    # ── Dimensions ──
    ladder_height = 4.50        # total ladder length (~15 ft portable escape ladder)
    strap_x = 0.150             # half-spacing between strap centres
    strap_width = 0.048         # webbing width (~50 mm, typical climbing webbing)
    strap_thickness = 0.004     # flat webbing thickness
    rung_radius = 0.013         # tubular rung outer radius (~26 mm OD)
    rung_length = 0.360         # rung extends past each strap
    rung_spacing = 0.300        # vertical pitch between rungs
    spacer_radius = 0.010       # rung standoff spacer radius
    spacer_length = 0.028       # spacer between strap inner face and rung shoulder

    # ════════════════════════════════════════════════════════
    # Root part: upper_ladder (webbing straps + tubular rungs)
    # ════════════════════════════════════════════════════════
    upper = model.part("upper_ladder")

    # Two flat webbing side straps (replacing rigid cylindrical rails)
    for side_index, x_sign in enumerate((-1.0, 1.0)):
        upper.visual(
            Box((strap_width, strap_thickness, ladder_height)),
            origin=Origin(xyz=(x_sign * strap_x, 0.0, ladder_height / 2.0)),
            material=yellow_webbing,
            name=f"side_strap_{side_index}",
        )

    # Tubular rungs threaded between the two straps, loop-emitted
    rung_z = 0.30
    rung_index = 0
    while rung_z < ladder_height - 0.15:
        # Main tubular rung (along X)
        _cyl_x(
            upper, f"rung_{rung_index}", rung_length, rung_radius,
            (0.0, 0.0, rung_z), galvanized,
        )
        # Short standoff spacers on each side (between strap and rung tube)
        for side_index, x_sign in enumerate((-1.0, 1.0)):
            sx = x_sign * (strap_x + spacer_length / 2.0)
            _cyl_x(
                upper, f"rung_spacer_{rung_index}_{side_index}",
                spacer_length, spacer_radius,
                (sx, 0.0, rung_z), dark_metal,
            )
        rung_z += rung_spacing
        rung_index += 1

    # Top reinforcement plate (where both straps terminate and the hook attaches)
    upper.visual(
        Box((2.0 * strap_x + strap_width + 0.020, 0.008, 0.100)),
        origin=Origin(xyz=(0.0, 0.0, ladder_height - 0.050)),
        material=galvanized,
        name="top_plate",
    )

    # Bottom reinforcement plate
    upper.visual(
        Box((2.0 * strap_x + strap_width + 0.020, 0.008, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=galvanized,
        name="bottom_plate",
    )

    # Pivot brackets on the upper ladder (where hook hinges attach)
    for side_index, x_sign in enumerate((-1.0, 1.0)):
        upper.visual(
            Box((0.022, 0.030, 0.065)),
            origin=Origin(xyz=(x_sign * 0.140, 0.0, ladder_height + 0.010)),
            material=galvanized,
            name=f"pivot_bracket_{side_index}",
        )

    # ════════════════════════════════════════════════════════
    # Hook part: folding standoff/hook for hanging over window sill
    # ════════════════════════════════════════════════════════
    hook = model.part("hook")

    # Hook arm — vertical plate from the pivot up to the bend
    hook.visual(
        Box((0.280, 0.006, 0.300)),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=galvanized,
        name="hook_arm",
    )

    # Hook crossbar — horizontal plate connecting arm top to the lip region
    hook.visual(
        Box((0.280, 0.086, 0.006)),
        origin=Origin(xyz=(0.0, -0.040, 0.300)),
        material=galvanized,
        name="hook_crossbar",
    )

    # Hook lip bar — cylindrical contact bar that hooks over the sill edge
    _cyl_x(hook, "hook_lip_bar", 0.280, 0.012,
           (0.0, -0.083, 0.300), galvanized)

    # Anti-slip pad under the lip bar
    hook.visual(
        Box((0.240, 0.004, 0.060)),
        origin=Origin(xyz=(0.0, -0.083, 0.268)),
        material=dark_metal,
        name="hook_pad",
    )

    # Hook pivot pin — visible hinge barrel (offset above pivot origin
    # so it clears the upper_ladder top_plate in the stowed pose)
    _cyl_x(hook, "hook_pin", 0.320, 0.010,
           (0.0, 0.0, 0.020), dark_metal)

    # ════════════════════════════════════════════════════════
    # Hook pivot articulation (REVOLUTE)
    # ════════════════════════════════════════════════════════
    # At q=0 the hook arm points upward along +Z (stowed / folded flat).
    # Positive q rotates around +X (right-hand rule): +Z → −Y, so the
    # hook swings toward −Y (the wall/sill side) to deploy.
    model.articulation(
        "hook_pivot",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=hook,
        origin=Origin(xyz=(0.0, 0.0, ladder_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=2.0,
            lower=0.0,
            upper=1.40,   # ~80° deployment
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    upper = object_model.get_part("upper_ladder")
    hook = object_model.get_part("hook")
    pivot = object_model.get_articulation("hook_pivot")

    # ── Structural topology: webbing straps replaced rigid rails ──
    ctx.check(
        "webbing side straps present",
        all(
            any(v.name == f"side_strap_{i}" for v in upper.visuals)
            for i in (0, 1)
        ),
        details="Two flat webbing side_strap_0/side_strap_1 must replace the rigid steel rails.",
    )

    # Straps should be thin (webbing profile, not round bar)
    for side_index in (0, 1):
        strap = upper.get_visual(f"side_strap_{side_index}")
        ctx.check(
            f"side_strap_{side_index} is flat webbing",
            strap is not None and strap.geometry.size[1] < 0.010,
            details="Webbing strap thickness must be thin (<10 mm), not a round bar.",
        )

    # ── Rungs remain loop-emitted ──
    rung_count = len([
        v for v in upper.visuals
        if v.name.startswith("rung_") and "spacer" not in v.name
    ])
    ctx.check(
        "tubular rungs via for-loop",
        rung_count >= 12,
        details=f"Expected at least 12 rungs from the indexed loop, got {rung_count}.",
    )

    # ── Hook pivot is a real non-fixed REVOLUTE joint ──
    ctx.check(
        "hook_pivot joint exists",
        pivot is not None,
        details="A REVOLUTE hook_pivot articulation must exist at the top attachment.",
    )

    # Hook deploys by swinging outward from stowed (q=0) position.
    # Measure the hook_arm visual AABB because the part origin does not
    # move under rotation around itself.
    rest_aabb = ctx.part_element_world_aabb(hook, elem="hook_arm")
    with ctx.pose({pivot: 1.2}):
        deployed_aabb = ctx.part_element_world_aabb(hook, elem="hook_arm")
    rest_y_min = rest_aabb[0][1] if rest_aabb is not None else None
    deployed_y_min = deployed_aabb[0][1] if deployed_aabb is not None else None
    ctx.check(
        "hook deploys from stowed position",
        rest_y_min is not None
        and deployed_y_min is not None
        and deployed_y_min < rest_y_min - 0.05,
        details=(
            f"Hook must swing outward (toward -Y) when hook_pivot is actuated. "
            f"rest_y_min={rest_y_min}, deployed_y_min={deployed_y_min}"
        ),
    )

    # ── Removed structures ──
    cage_visuals = [v for v in upper.visuals if v.name.startswith("cage_")]
    wall_plates = [v for v in upper.visuals if v.name.startswith("wall_plate_")]
    standoff_visuals = [v for v in upper.visuals if v.name.startswith("standoff_")]
    ctx.check(
        "no cage remnants",
        len(cage_visuals) == 0,
        details="Cage hoops and bars must be removed for the portable variant.",
    )
    ctx.check(
        "no wall-plate array",
        len(wall_plates) == 0 and len(standoff_visuals) == 0,
        details="Wall standoff brackets and mounting plates must be removed.",
    )

    # ── Intentional overlap: hook pivot passes through pivot brackets ──
    for side_index in (0, 1):
        ctx.allow_overlap(
            hook,
            upper,
            elem_a="hook_pin",
            elem_b=f"pivot_bracket_{side_index}",
            reason=(
                "The hook pivot pin is intentionally captured inside the pivot "
                "bracket to form a visible hinge barrel connection."
            ),
        )
        ctx.allow_overlap(
            hook,
            upper,
            elem_a="hook_arm",
            elem_b=f"pivot_bracket_{side_index}",
            reason=(
                "The hook arm passes through the pivot bracket cheek as part of "
                "the hinge assembly at the top attachment point."
            ),
        )
    ctx.expect_contact(
        hook,
        upper,
        elem_a="hook_pin",
        elem_b="pivot_bracket_0",
        contact_tol=0.005,
        name="hook pin contacts pivot bracket 0",
    )

    return ctx.report()


object_model = build_object_model()
