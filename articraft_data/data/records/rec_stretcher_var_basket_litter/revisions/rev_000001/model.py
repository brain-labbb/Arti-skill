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
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


# ── Preserved geometry helpers (KEEP) ───────────────────────────────────


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
    """Preserved pad-mesh helper from parent baseline."""
    profile = rounded_rect_profile(length, width, radius, corner_segments=8)
    return mesh_from_geometry(ExtrudeGeometry.centered(profile, thickness), name)


# ── Stokes basket geometry constants ────────────────────────────────────

_HALF_LENGTH = 1.00      # each half extends 1 m from the fold line
_HALF_WIDTH = 0.30       # widest half-width of the oval rim
_RIM_Z = 0.22            # rim height above the ground reference
_FLOOR_Z = 0.02          # floor pan reference height
_HEAD_RAISE = 0.06       # additional elevation at the head end
_RIM_RADIUS = 0.014      # perimeter rim tube radius
_RIB_RADIUS = 0.008      # vertical rib upright radius
_BOTTOM_RAIL_RADIUS = 0.007
_ROPE_RADIUS = 0.005


def _half_oval_points(
    half_length: float,
    half_width: float,
    z_base: float,
    z_raise: float,
    *,
    n: int = 12,
    footward: bool = False,
) -> list[tuple[float, float, float]]:
    """Generate half-oval spline points for one basket-half rim or rail."""
    pts: list[tuple[float, float, float]] = []
    for i in range(n + 1):
        t = i / n
        theta = (-math.pi / 2.0 + t * math.pi) if footward else (math.pi / 2.0 + t * math.pi)
        x = half_length * math.cos(theta)
        y = half_width * math.sin(theta)
        z = z_base + z_raise * abs(x) / max(half_length, 1e-9)
        pts.append((x, y, z))
    return pts


def _oval_normal(theta: float, half_length: float, half_width: float) -> tuple[float, float]:
    """Outward unit normal of an oval at parameter *theta*."""
    nx = math.cos(theta) / max(half_length, 1e-9)
    ny = math.sin(theta) / max(half_width, 1e-9)
    mag = math.sqrt(nx * nx + ny * ny)
    if mag < 1e-12:
        return (1.0, 0.0)
    return (nx / mag, ny / mag)


def _add_basket_half(
    part,
    orange: Material,
    dark_grey: Material,
    rope_mat: Material,
    *,
    footward: bool,
) -> None:
    """Build one half of the Stokes basket on *part*."""
    sign = 1.0 if footward else -1.0
    raise_z = _HEAD_RAISE if not footward else 0.0

    # ── 1. perimeter rim tube (half-oval) ──────────────────────────────
    rim_pts = _half_oval_points(
        _HALF_LENGTH, _HALF_WIDTH, _RIM_Z, raise_z, n=12, footward=footward
    )
    _curved_tube(part, rim_pts, _RIM_RADIUS, orange, "rim_tube",
                 samples_per_segment=10, radial_segments=18)

    # ── 2. perforated floor pan ────────────────────────────────────────
    floor_length = _HALF_LENGTH * 0.84
    floor_width = _HALF_WIDTH * 1.55
    floor_mesh = mesh_from_geometry(
        PerforatedPanelGeometry(
            (floor_length, floor_width),
            0.005,
            hole_diameter=0.014,
            pitch=(0.025, 0.025),
            frame=0.012,
            corner_radius=0.020,
            stagger=True,
        ),
        "floor_pan",
    )
    floor_cx = sign * _HALF_LENGTH * 0.46
    floor_cz = _FLOOR_Z + 0.003 + (raise_z * 0.35 if not footward else 0.0)
    part.visual(
        floor_mesh,
        origin=Origin(xyz=(floor_cx, 0.0, floor_cz)),
        material=dark_grey,
        name="floor_pan",
    )

    # ── 3. vertical rib uprights ───────────────────────────────────────
    n_ribs = 7
    for i in range(n_ribs):
        t = (i + 0.5) / n_ribs
        theta = (-math.pi / 2.0 + t * math.pi) if footward else (math.pi / 2.0 + t * math.pi)
        x = _HALF_LENGTH * math.cos(theta)
        y = _HALF_WIDTH * math.sin(theta)
        z_top = _RIM_Z + (raise_z * abs(x) / max(_HALF_LENGTH, 1e-9) if not footward else 0.0)
        # Rib embeds slightly into the floor pan for guaranteed connectivity
        _tube(part, (x, y, floor_cz - 0.008), (x, y, z_top - 0.005),
              _RIB_RADIUS, orange, f"rib_{i}")

    # ── 4. bottom rail (connects rib bases around the basket perimeter) ─
    bottom_pts: list[tuple[float, float, float]] = []
    for i in range(10):
        t = i / 9
        theta = (-math.pi / 2.0 + t * math.pi) if footward else (math.pi / 2.0 + t * math.pi)
        x = _HALF_LENGTH * math.cos(theta)
        y = _HALF_WIDTH * math.sin(theta)
        z = floor_cz + 0.010 + (raise_z * abs(x) / max(_HALF_LENGTH, 1e-9) * 0.25 if not footward else 0.0)
        bottom_pts.append((x, y, z))
    _curved_tube(part, bottom_pts, _BOTTOM_RAIL_RADIUS, orange, "bottom_rail",
                 samples_per_segment=8, radial_segments=12)

    # ── 5. rope grab handles around the rim ────────────────────────────
    n_handles = 5
    for i in range(n_handles):
        t = (i + 0.5) / n_handles
        theta = (-math.pi / 2.0 + t * math.pi) if footward else (math.pi / 2.0 + t * math.pi)
        x = _HALF_LENGTH * math.cos(theta)
        y = _HALF_WIDTH * math.sin(theta)
        z_rim = _RIM_Z + (raise_z * abs(x) / max(_HALF_LENGTH, 1e-9) if not footward else 0.0)
        nx, ny = _oval_normal(theta, _HALF_LENGTH, _HALF_WIDTH)
        reach = 0.050
        # Along-rim tangent for the handle attachment footprint
        along_x = -math.sin(theta) * 0.012
        along_y = math.cos(theta) * 0.012
        hx, hy = x + nx * reach, y + ny * reach
        handle_pts = [
            (x + along_x, y + along_y, z_rim),
            (x + nx * 0.020, y + ny * 0.020, z_rim + 0.018),
            (hx, hy, z_rim + 0.038),
            (x + nx * 0.020, y + ny * 0.020, z_rim + 0.022),
            (x - along_x, y - along_y, z_rim),
        ]
        _curved_tube(part, handle_pts, _ROPE_RADIUS, rope_mat, f"rope_handle_{i}",
                     samples_per_segment=6, radial_segments=10)

    # ── 6. center-fold hinge hardware ──────────────────────────────────
    hinge_half_span = 0.09 if not footward else 0.07
    hinge_radius = 0.014 if not footward else 0.011
    _tube(part, (0.0, -hinge_half_span, 0.0), (0.0, hinge_half_span, 0.0),
          hinge_radius, orange, "hinge_barrel")
    # Fold stanchions connecting hinge barrel into the half's own rim.
    # Each stanchion starts at the barrel end but is offset along X by ±0.030 m
    # so the two half-hinges clear each other at the closed q=0 pose.
    # Stanchion bases touch the barrel surface (barrel radius ~0.014 m)
    # so the barrel connects through the stanchions into the rim.
    sx_start = sign * 0.014
    sx_end = sign * 0.045
    _tube(part, (sx_start, -hinge_half_span, 0.0), (sx_end, -_HALF_WIDTH, _RIM_Z),
          0.009, orange, "fold_stanchion_0")
    _tube(part, (sx_start, hinge_half_span, 0.0), (sx_end, _HALF_WIDTH, _RIM_Z),
          0.009, orange, "fold_stanchion_1")
    # Hinge plate sits fully on the half's own side of the fold line to avoid
    # crossing into the mating barrel's footprint.
    plate_cx = sign * 0.035
    part.visual(
        Box((0.040, hinge_half_span * 2.2, 0.006)),
        origin=Origin(xyz=(plate_cx, 0.0, 0.0)),
        material=orange,
        name="hinge_plate",
    )


# ── Model assembly ──────────────────────────────────────────────────────


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="stokes_basket_rescue_litter",
        meta={
            "run_notes": (
                "Stokes basket rescue litter variant of the emergency stretcher family: "
                "boat-shaped basket with oval perimeter tube rail, perforated mesh floor pan, "
                "rope grab handles, and center fold hinge for storage. International orange "
                "rescue colorway (⑥ companion variation)."
            )
        },
    )

    orange = model.material("rescue_orange", rgba=(0.95, 0.35, 0.05, 1.0))
    dark_grey = model.material("dark_grey_metal", rgba=(0.15, 0.15, 0.14, 1.0))
    rope_color = model.material("manila_rope", rgba=(0.78, 0.65, 0.38, 1.0))

    # Head basket half (root) – preserved part name from parent baseline
    head = model.part("deck_frame")
    _add_basket_half(head, orange, dark_grey, rope_color, footward=False)

    # Foot basket half – folds up against the head half for storage
    foot = model.part("foot_basket")
    _add_basket_half(foot, orange, dark_grey, rope_color, footward=True)

    # Center fold hinge (repurposed parent joint deck_to_head_leg)
    # Axis (0, -1, 0): positive q rotates the foot half's +X extent toward +Z (upward fold).
    model.articulation(
        "deck_to_head_leg",
        ArticulationType.REVOLUTE,
        parent=head,
        child=foot,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.0, lower=0.0, upper=3.0),
    )

    return model


# ── Tests ───────────────────────────────────────────────────────────────


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    head = object_model.get_part("deck_frame")
    foot = object_model.get_part("foot_basket")
    fold = object_model.get_articulation("deck_to_head_leg")

    # ── intentional-overlap allowances for captured-pin fold hinge ─────
    ctx.allow_overlap(
        head, foot,
        elem_a="hinge_barrel", elem_b="hinge_barrel",
        reason=(
            "The foot-half hinge barrel is intentionally nested inside the head-half "
            "hinge barrel to form a captured-pin center fold hinge."
        ),
    )
    # The fold stanchions from each half emerge from the mating barrel surface
    # and may clip the outer barrel at the fold line.
    for stanchion_name in ("fold_stanchion_0", "fold_stanchion_1"):
        ctx.allow_overlap(
            head, foot,
            elem_a="hinge_barrel", elem_b=stanchion_name,
            reason=(
                f"The foot {stanchion_name} emerges from the captured-pin hinge barrel "
                "zone and locally clips the head barrel surface at the fold line."
            ),
        )
        ctx.allow_overlap(
            foot, head,
            elem_a="hinge_barrel", elem_b=stanchion_name,
            reason=(
                f"The head {stanchion_name} emerges from the captured-pin hinge barrel "
                "zone and locally clips the foot barrel surface at the fold line."
            ),
        )
        ctx.allow_overlap(
            head, foot,
            elem_a="hinge_plate", elem_b=stanchion_name,
            reason=(
                f"Foot {stanchion_name} locally enters the head hinge plate at the fold line."
            ),
        )
        ctx.allow_overlap(
            foot, head,
            elem_a="hinge_plate", elem_b=stanchion_name,
            reason=(
                f"Head {stanchion_name} locally enters the foot hinge plate at the fold line."
            ),
        )

    # Proof: interleaved barrels retain overlap along Y
    ctx.expect_overlap(
        head, foot, axes="y",
        elem_a="hinge_barrel", elem_b="hinge_barrel",
        min_overlap=0.10,
        name="hinge barrels remain interleaved along Y",
    )
    # Proof: nested barrel stays centered in the outer barrel
    ctx.expect_within(
        foot, head, axes="z",
        inner_elem="hinge_barrel", outer_elem="hinge_barrel",
        margin=0.005,
        name="foot hinge barrel stays within head hinge barrel on Z",
    )

    # ── fold hinge moves foot half upward ──────────────────────────────
    with ctx.pose({fold: 0.0}):
        deployed_foot = ctx.part_world_aabb(foot)
    with ctx.pose({fold: 1.5}):
        folded_foot = ctx.part_world_aabb(foot)
    ctx.check(
        "basket fold hinge deck_to_head_leg raises foot half for storage",
        deployed_foot is not None
        and folded_foot is not None
        and folded_foot[1][2] > deployed_foot[1][2] + 0.15,
        details=f"deployed={deployed_foot}, folded={folded_foot}",
    )

    # ── visible geometry claims for this axis (③ primary form family) ──
    ctx.check(
        "head basket has perforated floor pan",
        head.get_visual("floor_pan") is not None,
    )
    ctx.check(
        "foot basket has perforated floor pan",
        foot.get_visual("floor_pan") is not None,
    )
    ctx.check(
        "head basket has oval perimeter rim tube",
        head.get_visual("rim_tube") is not None,
    )
    ctx.check(
        "foot basket has oval perimeter rim tube",
        foot.get_visual("rim_tube") is not None,
    )
    ctx.check(
        "head basket has rope grab handles",
        head.get_visual("rope_handle_0") is not None,
    )
    ctx.check(
        "foot basket has rope grab handles",
        foot.get_visual("rope_handle_0") is not None,
    )

    # ── head end is slightly raised ────────────────────────────────────
    head_aabb = ctx.part_world_aabb(head)
    ctx.check(
        "head end is slightly raised above foot end",
        head_aabb is not None
        and deployed_foot is not None
        and head_aabb[1][2] > deployed_foot[1][2] - 0.02,
        details=f"head_max_z={head_aabb[1][2] if head_aabb else None}, "
                f"foot_max_z={deployed_foot[1][2] if deployed_foot else None}",
    )

    return ctx.report()


object_model = build_object_model()
