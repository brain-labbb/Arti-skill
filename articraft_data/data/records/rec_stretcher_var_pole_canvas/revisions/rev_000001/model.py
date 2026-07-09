from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


# ---------------------------------------------------------------------------
# Geometry helpers (preserved from parent: _tube, _curved_tube,
# _rounded_pad_mesh, tube_from_spline_points)
# ---------------------------------------------------------------------------


def _origin_for_cylinder_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    extend: float = 0.0,
) -> tuple[Origin, float]:
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1e-9:
        raise ValueError("tube endpoints must be separated")

    ux, uy, uz = dx / length, dy / length, dz / length
    sx -= ux * extend
    sy -= uy * extend
    sz -= uz * extend
    ex += ux * extend
    ey += uy * extend
    ez += uz * extend
    length += 2.0 * extend

    mx, my, mz = (sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5
    yaw = math.atan2(uy, ux)
    pitch = math.atan2(math.sqrt(ux * ux + uy * uy), uz)
    return Origin(xyz=(mx, my, mz), rpy=(0.0, pitch, yaw)), length


def _tube(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material: Material,
    name: str,
    *,
    extend: float = 0.002,
) -> None:
    origin, length = _origin_for_cylinder_between(start, end, extend=extend)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=origin,
        material=material,
        name=name,
    )


def _curved_tube(
    part,
    points: list[tuple[float, float, float]],
    radius: float,
    material: Material,
    name: str,
    *,
    samples_per_segment: int = 8,
    radial_segments: int = 18,
) -> None:
    part.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                points,
                radius=radius,
                samples_per_segment=samples_per_segment,
                radial_segments=radial_segments,
                cap_ends=True,
            ),
            name,
        ),
        material=material,
        name=name,
    )


def _rounded_pad_mesh(width: float, length: float, thickness: float, radius: float, name: str):
    profile = rounded_rect_profile(length, width, radius, corner_segments=8)
    return mesh_from_geometry(ExtrudeGeometry.centered(profile, thickness), name)


# ---------------------------------------------------------------------------
# Layout constants (metres)
# ---------------------------------------------------------------------------

_POLE_HALF_SPACING = 0.250       # poles at y = ±0.250
_POLE_HALF_LENGTH = 0.915        # poles from x = -0.915 to +0.915
_CANVAS_HALF_LENGTH = 0.640      # canvas from x = -0.640 to +0.640
_CANVAS_HALF_WIDTH = 0.220       # canvas from y = -0.220 to +0.220
_POLE_RADIUS = 0.014
_CANVAS_Z = 0.010                # canvas centre height
_CANVAS_THICKNESS = 0.005
_SPREADER_Z = -0.030             # spreader bars below poles
_HANDLE_START = 0.700            # where handle grips begin


# ---------------------------------------------------------------------------
# Pole-and-canvas stretcher geometry builders
# ---------------------------------------------------------------------------


def _add_pole_geometry(part, pole_mat, grip_mat, y: float, idx: int) -> None:
    """One carry pole with rubber handle grips and end caps."""
    _tube(
        part,
        (-_POLE_HALF_LENGTH, y, 0.0),
        (_POLE_HALF_LENGTH, y, 0.0),
        _POLE_RADIUS,
        pole_mat,
        f"pole_{idx}",
    )
    # Rubber grip sleeves at both handle ends
    for sign, suffix in ((-1, "head"), (1, "foot")):
        cx = sign * (_HANDLE_START + (_POLE_HALF_LENGTH - _HANDLE_START) * 0.5)
        half = (_POLE_HALF_LENGTH - _HANDLE_START) * 0.5
        _tube(
            part,
            (cx - half, y, 0.0),
            (cx + half, y, 0.0),
            0.019,
            grip_mat,
            f"grip_{suffix}_{idx}",
            extend=0.0,
        )
    # Rounded end caps
    for sign, tag in ((-1, "head"), (1, "foot")):
        part.visual(
            Sphere(0.016),
            origin=Origin(xyz=(sign * _POLE_HALF_LENGTH, y, 0.0)),
            material=pole_mat,
            name=f"end_cap_{tag}_{idx}",
        )


def _add_spreader_geometry(part, pole_mat, metal_mat) -> None:
    """Spreader bar in local frame: origin at hinge on the near pole,
    bar extends +Y across to the far pole.  Hinge and latch arms reach
    upward (+Z) to pole level and touch the bar tube."""
    # Main cross-bar tube — extends from just past hinge to just past latch
    _tube(
        part,
        (0.0, -0.005, 0.0),
        (0.0, 0.505, 0.0),
        0.011,
        pole_mat,
        "spreader_bar",
    )
    # Centre brace disc
    part.visual(
        Cylinder(radius=0.016, length=0.018),
        origin=Origin(xyz=(0.0, 0.250, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="centre_brace",
    )
    # Hinge arm — vertical plate from bar up toward the pole
    # Y range = [-0.004, 0.028], overlaps bar start at Y≈-0.003
    part.visual(
        Box((0.030, 0.032, 0.055)),
        origin=Origin(xyz=(0.0, 0.012, 0.028)),
        material=metal_mat,
        name="hinge_arm",
    )
    # Hinge pin (along X, at top of hinge arm — touches hinge arm box)
    part.visual(
        Cylinder(radius=0.005, length=0.038),
        origin=Origin(xyz=(0.0, 0.012, 0.052), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="hinge_pin",
    )
    # Latch arm at far end
    # Y range = [0.472, 0.504], overlaps bar end at Y≈0.503
    part.visual(
        Box((0.030, 0.032, 0.055)),
        origin=Origin(xyz=(0.0, 0.488, 0.028)),
        material=metal_mat,
        name="latch_arm",
    )
    # Latch hook pin
    part.visual(
        Cylinder(radius=0.005, length=0.034),
        origin=Origin(xyz=(0.0, 0.488, 0.052), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="latch_hook",
    )


def _add_backrest_geometry(part, pole_mat, canvas_mat) -> None:
    """Head-elevation prop frame.  Local origin at the hinge; the frame
    extends toward −X (head direction)."""
    # Two support arms
    _tube(part, (0.0, -0.180, 0.0), (-0.320, -0.180, 0.0), 0.008, pole_mat, "arm_0")
    _tube(part, (0.0, 0.180, 0.0), (-0.320, 0.180, 0.0), 0.008, pole_mat, "arm_1")
    # Cross bar at head end
    _tube(part, (-0.320, -0.180, 0.0), (-0.320, 0.180, 0.0), 0.008, pole_mat, "cross_bar")
    # Mid support slat
    _tube(part, (-0.160, -0.170, 0.0), (-0.160, 0.170, 0.0), 0.006, pole_mat, "mid_slat")
    # Canvas panel riding on the frame — sits close enough to touch the slat
    pad = _rounded_pad_mesh(0.340, 0.320, 0.004, 0.012, "backrest_canvas_pad")
    part.visual(
        pad,
        origin=Origin(xyz=(-0.160, 0.0, -0.004)),
        material=canvas_mat,
        name="backrest_canvas",
    )
    # Hinge brackets at pivot end — overlap with arm tubes for connectivity
    for i, y in enumerate((-0.180, 0.180)):
        part.visual(
            Box((0.030, 0.028, 0.024)),
            origin=Origin(xyz=(0.006, y, -0.004)),
            material=pole_mat,
            name=f"hinge_bracket_{i}",
        )


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="pole_canvas_field_stretcher",
        meta={
            "run_notes": (
                "Pole-and-canvas field stretcher fork: two parallel carry poles, "
                "hinged spreader cross-bars at head and foot, olive-drab canvas bed, "
                "four protruding pole-end carry handles.  Companion variation ⑥ "
                "(OD-green canvas, black poles).  The parent wheeled legs and "
                "casters are replaced by folding spreader-bar revolute hinges."
            )
        },
    )

    # --- Materials (companion variation ⑥) ---
    pole_black = model.material("pole_black", rgba=(0.04, 0.04, 0.04, 1.0))
    canvas_od = model.material("olive_drab_canvas", rgba=(0.42, 0.44, 0.24, 1.0))
    rubber_grip = model.material("rubber_grip", rgba=(0.06, 0.06, 0.05, 1.0))
    metal_hinge = model.material("zinc_hinge", rgba=(0.55, 0.55, 0.50, 1.0))
    stitch_dark = model.material("stitch_thread", rgba=(0.25, 0.26, 0.15, 1.0))

    # ================================================================
    # deck_frame  (root) — two carry poles + canvas bed
    # ================================================================
    deck = model.part("deck_frame")

    # Carry poles
    for i, y in enumerate((-_POLE_HALF_SPACING, _POLE_HALF_SPACING)):
        _add_pole_geometry(deck, pole_black, rubber_grip, y, i)

    # Main canvas bed
    main_canvas = _rounded_pad_mesh(
        2.0 * _CANVAS_HALF_WIDTH,
        2.0 * _CANVAS_HALF_LENGTH,
        _CANVAS_THICKNESS,
        0.022,
        "main_canvas_bed",
    )
    deck.visual(
        main_canvas,
        origin=Origin(xyz=(0.0, 0.0, _CANVAS_Z)),
        material=canvas_od,
        name="main_canvas",
    )

    # Canvas edge reinforcement strips — bridge from canvas body toward poles
    # Shortened to not extend into spreader mount areas at x=±0.500
    for i, y_sign in enumerate((-1, 1)):
        strip_cy = y_sign * 0.228
        deck.visual(
            Box((0.860, 0.030, 0.007)),
            origin=Origin(xyz=(0.0, strip_cy, _CANVAS_Z)),
            material=stitch_dark,
            name=f"canvas_edge_{i}",
        )

    # Cross-stitch reinforcement lines on canvas
    for i, x in enumerate((-0.38, 0.0, 0.38)):
        deck.visual(
            Box((0.007, 0.42, 0.006)),
            origin=Origin(xyz=(x, 0.0, _CANVAS_Z + 0.003)),
            material=stitch_dark,
            name=f"canvas_stitch_{i}",
        )

    # Canvas grommets at pole edges — positioned on the canvas surface
    for i, y in enumerate((-0.218, 0.218)):
        for j, x in enumerate((-0.40, -0.10, 0.10, 0.40)):
            deck.visual(
                Cylinder(radius=0.005, length=0.007),
                origin=Origin(xyz=(x, y, _CANVAS_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=metal_hinge,
                name=f"grommet_{i}_{j}",
            )

    # Spreader-bar mount brackets on poles (vertical plates that touch poles)
    for x_pos, prefix in ((-0.500, "head"), (0.500, "foot")):
        for i, y in enumerate((-_POLE_HALF_SPACING, _POLE_HALF_SPACING)):
            deck.visual(
                Box((0.038, 0.032, 0.050)),
                origin=Origin(xyz=(x_pos, y, _SPREADER_Z + 0.025)),
                material=metal_hinge,
                name=f"{prefix}_mount_{i}",
            )

    # Backrest hinge mount brackets + vertical support tubes
    backrest_hinge_z = 0.024
    for i, y in enumerate((-0.180, 0.180)):
        deck.visual(
            Box((0.036, 0.032, 0.024)),
            origin=Origin(xyz=(-0.340, y, backrest_hinge_z)),
            material=metal_hinge,
            name=f"backrest_mount_{i}",
        )
        # Vertical support tube from pole top to mount height
        _tube(
            deck,
            (-0.340, y, _POLE_RADIUS),
            (-0.340, y, backrest_hinge_z - 0.006),
            0.005,
            metal_hinge,
            f"backrest_support_{i}",
        )

    # Curved side-carry handle on left pole (mid-length grab handle)
    _curved_tube(
        deck,
        [
            (-0.100, -_POLE_HALF_SPACING, _POLE_RADIUS),
            (-0.050, -_POLE_HALF_SPACING, 0.036),
            (0.050, -_POLE_HALF_SPACING, 0.036),
            (0.100, -_POLE_HALF_SPACING, _POLE_RADIUS),
        ],
        0.006,
        rubber_grip,
        "carry_handle",
        samples_per_segment=6,
        radial_segments=12,
    )

    # ================================================================
    # head_spreader  — hinged cross-bar at head end
    # ================================================================
    head_spreader = model.part("head_spreader")
    _add_spreader_geometry(head_spreader, pole_black, metal_hinge)

    # ================================================================
    # foot_spreader  — hinged cross-bar at foot end
    # ================================================================
    foot_spreader = model.part("foot_spreader")
    _add_spreader_geometry(foot_spreader, pole_black, metal_hinge)

    # ================================================================
    # backrest  — head-elevation prop frame
    # ================================================================
    backrest = model.part("backrest")
    _add_backrest_geometry(backrest, pole_black, canvas_od)

    # ================================================================
    # Articulations
    # ================================================================

    # Head spreader hinge (replaces deck_to_head_leg)
    model.articulation(
        "deck_to_head_spreader",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=head_spreader,
        origin=Origin(xyz=(-0.500, -_POLE_HALF_SPACING, _SPREADER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.0, lower=0.0, upper=1.30),
    )

    # Foot spreader hinge (replaces deck_to_foot_leg)
    model.articulation(
        "deck_to_foot_spreader",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=foot_spreader,
        origin=Origin(xyz=(0.500, -_POLE_HALF_SPACING, _SPREADER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.0, lower=0.0, upper=1.30),
    )

    # Backrest tilt (head-elevation prop)
    model.articulation(
        "deck_to_backrest",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=backrest,
        origin=Origin(xyz=(-0.340, 0.0, backrest_hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.0, lower=0.0, upper=0.70),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck_frame")
    head_spreader = object_model.get_part("head_spreader")
    foot_spreader = object_model.get_part("foot_spreader")
    backrest = object_model.get_part("backrest")

    head_joint = object_model.get_articulation("deck_to_head_spreader")
    foot_joint = object_model.get_articulation("deck_to_foot_spreader")
    back_joint = object_model.get_articulation("deck_to_backrest")

    # ==================================================================
    # Intentional overlap allowances
    # ==================================================================

    # --- Spreader hinge/latch arms embed into deck mounts AND poles ---
    for spr_name, mount_prefix in (
        ("head_spreader", "head"),
        ("foot_spreader", "foot"),
    ):
        # Hinge arm nests inside the near mount bracket
        ctx.allow_overlap(
            deck,
            spr_name,
            elem_a=f"{mount_prefix}_mount_0",
            elem_b="hinge_arm",
            reason="Spreader hinge arm nests inside the deck pole-mount bracket at the pivot.",
        )
        # Hinge arm wraps around the near pole
        ctx.allow_overlap(
            deck,
            spr_name,
            elem_a="pole_0",
            elem_b="hinge_arm",
            reason="Spreader hinge arm wraps around the near carry pole at the pivot point.",
        )
        # Spreader bar passes through the near mount bracket
        ctx.allow_overlap(
            deck,
            spr_name,
            elem_a=f"{mount_prefix}_mount_0",
            elem_b="spreader_bar",
            reason="Spreader cross-bar tube passes through the near pole-mount bracket.",
        )
        # Latch arm seats into the far mount bracket
        ctx.allow_overlap(
            deck,
            spr_name,
            elem_a=f"{mount_prefix}_mount_1",
            elem_b="latch_arm",
            reason="Spreader latch arm seats into the opposite deck pole-mount bracket when deployed.",
        )
        # Latch arm wraps around the far pole
        ctx.allow_overlap(
            deck,
            spr_name,
            elem_a="pole_1",
            elem_b="latch_arm",
            reason="Spreader latch arm wraps around the far carry pole when latched.",
        )
        # Spreader bar passes through the far mount bracket
        ctx.allow_overlap(
            deck,
            spr_name,
            elem_a=f"{mount_prefix}_mount_1",
            elem_b="spreader_bar",
            reason="Spreader cross-bar tube passes through the far pole-mount bracket.",
        )

    # --- Backrest arms embed into deck mounts ---
    for i in range(2):
        ctx.allow_overlap(
            deck,
            "backrest",
            elem_a=f"backrest_mount_{i}",
            elem_b=f"arm_{i}",
            reason="Backrest arm tube passes through the deck mount bracket at the pivot.",
        )
        ctx.allow_overlap(
            deck,
            "backrest",
            elem_a=f"backrest_mount_{i}",
            elem_b=f"hinge_bracket_{i}",
            reason="Backrest hinge bracket is captured inside the deck mount at the pivot.",
        )
        ctx.allow_overlap(
            deck,
            "backrest",
            elem_a=f"backrest_mount_{i}",
            elem_b=f"backrest_support_{i}",
            reason="Deck backrest support tube nests inside the mount bracket at the pivot.",
        )
        ctx.allow_overlap(
            deck,
            "backrest",
            elem_a=f"backrest_support_{i}",
            elem_b=f"hinge_bracket_{i}",
            reason="Backrest hinge bracket contacts the deck support tube at the pivot point.",
        )

    # ==================================================================
    # Exact proof checks
    # ==================================================================

    # Spreader bars span between the poles (Y overlap proof)
    for spr, prefix in ((head_spreader, "head"), (foot_spreader, "foot")):
        ctx.expect_overlap(
            spr,
            deck,
            axes="y",
            elem_a="spreader_bar",
            elem_b="pole_0",
            min_overlap=0.010,
            name=f"{prefix} spreader bar overlaps left pole in Y",
        )
        ctx.expect_overlap(
            spr,
            deck,
            axes="y",
            elem_a="spreader_bar",
            elem_b="pole_1",
            min_overlap=0.010,
            name=f"{prefix} spreader bar overlaps right pole in Y",
        )

    # Canvas stays between the poles (signed gap checks)
    ctx.expect_gap(
        deck,
        deck,
        axis="y",
        positive_elem="main_canvas",
        negative_elem="pole_0",
        min_gap=0.0,
        name="canvas does not extend past left pole",
    )
    ctx.expect_gap(
        deck,
        deck,
        axis="y",
        positive_elem="pole_1",
        negative_elem="main_canvas",
        min_gap=0.0,
        name="canvas does not extend past right pole",
    )

    # Backrest arm overlap with mount (proof the hinge capture is real)
    for i in range(2):
        ctx.expect_overlap(
            backrest,
            deck,
            axes="y",
            elem_a=f"arm_{i}",
            elem_b=f"backrest_mount_{i}",
            min_overlap=0.015,
            name=f"backrest arm_{i} captured by mount bracket",
        )

    # ==================================================================
    # Pose checks
    # ==================================================================

    def _max_z(aabb):
        return aabb[1][2] if aabb is not None else None

    # Spreader bars fold upward when unlatched
    head_rest = ctx.part_world_aabb(head_spreader)
    foot_rest = ctx.part_world_aabb(foot_spreader)
    with ctx.pose({head_joint: 1.10, foot_joint: 1.10}):
        head_folded = ctx.part_world_aabb(head_spreader)
        foot_folded = ctx.part_world_aabb(foot_spreader)

    ctx.check(
        "head_spreader folds upward via deck_to_head_spreader revolute hinge",
        head_rest is not None
        and head_folded is not None
        and _max_z(head_folded) > _max_z(head_rest) + 0.15,
        details=f"rest_max_z={_max_z(head_rest)}, folded_max_z={_max_z(head_folded)}",
    )
    ctx.check(
        "foot_spreader folds upward via deck_to_foot_spreader revolute hinge",
        foot_rest is not None
        and foot_folded is not None
        and _max_z(foot_folded) > _max_z(foot_rest) + 0.15,
        details=f"rest_max_z={_max_z(foot_rest)}, folded_max_z={_max_z(foot_folded)}",
    )

    # Backrest tilts the head end upward
    flat_back = ctx.part_world_aabb(backrest)
    with ctx.pose({back_joint: 0.60}):
        raised_back = ctx.part_world_aabb(backrest)
    ctx.check(
        "backrest hinge tilts the head-elevation frame upward",
        flat_back is not None
        and raised_back is not None
        and _max_z(raised_back) > _max_z(flat_back) + 0.08,
        details=f"flat={_max_z(flat_back)}, raised={_max_z(raised_back)}",
    )

    # Structural identity: four carry-handle grips exist on the deck
    ctx.check(
        "deck has four handle grips (two poles × two ends)",
        all(
            deck.get_visual(name) is not None
            for name in ("grip_head_0", "grip_head_1", "grip_foot_0", "grip_foot_1")
        ),
    )

    # Classification note recorded
    ctx.check(
        "variant classification note recorded",
        "field stretcher" in str(object_model.meta.get("run_notes", "")),
    )

    return ctx.report()


object_model = build_object_model()
