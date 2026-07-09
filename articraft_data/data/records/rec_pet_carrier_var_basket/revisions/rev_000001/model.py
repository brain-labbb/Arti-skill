"""Woven wicker/rattan pet basket carrier.

A classic rounded-rectangle wicker basket body with horizontal weave bands
and vertical stakes, a domed woven top with a hatch opening, a hinged woven
lid, an arched carry handle, and a light-blue interior pet pad.

Long axis = X (front at +X). Ground at z = 0.
"""

from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    section_loft,
    tube_from_spline_points,
)

# ---------------------------------------------------------------- dimensions
BODY_L = 0.46   # x (long axis)
BODY_W = 0.27   # y
WALL_T = 0.013  # wicker wall thickness
TUB_TOP = 0.26  # z of basket rim
FLOOR_T = 0.014

CORNER_R = 0.032  # chamfer radius for rounded-rect sections

DOME_Z0 = 0.255   # dome bottom (embeds slightly into rim)
DOME_Z1 = 0.340   # dome flat top
DOME_TOP_HX = 0.16
DOME_TOP_HY = 0.10

# top hatch opening in the dome (world coords)
HATCH_X0, HATCH_X1 = -0.13, 0.07
HATCH_HY = 0.070

# weave parameters
BAND_RADIUS = 0.003    # horizontal weave band tube radius
STAKE_RADIUS = 0.0025  # vertical stake tube radius
N_BANDS_BODY = 6       # horizontal bands on the body
N_BANDS_DOME = 2       # horizontal bands on the dome
N_STAKES = 16          # vertical stakes around the perimeter

# materials - natural rattan palette
RATTAN_TAN = (0.76, 0.60, 0.42, 1.0)
RATTAN_DARK = (0.52, 0.38, 0.24, 1.0)
RATTAN_LIGHT = (0.84, 0.71, 0.51, 1.0)
RATTAN_WEAVE = (0.66, 0.50, 0.34, 1.0)
RATTAN_RIM = (0.58, 0.42, 0.26, 1.0)
PAD_BLUE = (0.47, 0.60, 0.70, 1.0)
HANDLE_BROWN = (0.48, 0.34, 0.20, 1.0)


# -------------------------------------------------------- geometry helpers

def _rect_loop(hx: float, hy: float, z: float, c: float, cx: float = 0.0):
    """Closed chamfered-rectangle loop (octagon) in a z-plane."""
    return [
        (cx - hx + c, -hy, z),
        (cx + hx - c, -hy, z),
        (cx + hx, -hy + c, z),
        (cx + hx, hy - c, z),
        (cx + hx - c, hy, z),
        (cx - hx + c, hy, z),
        (cx - hx, hy - c, z),
        (cx - hx, -hy + c, z),
    ]


def _distribute_perimeter(hx: float, hy: float, c: float, n: int):
    """Distribute n (x, y) points evenly around the chamfered-rectangle perimeter."""
    corners = _rect_loop(hx, hy, 0.0, c)
    edges = []
    total = 0.0
    for i in range(len(corners)):
        p0 = corners[i]
        p1 = corners[(i + 1) % len(corners)]
        length = math.sqrt((p1[0] - p0[0]) ** 2 + (p1[1] - p0[1]) ** 2)
        edges.append((p0, p1, length))
        total += length
    points: list[tuple[float, float]] = []
    for i in range(n):
        s = (i / n) * total
        acc = 0.0
        for p0, p1, length in edges:
            if acc + length >= s - 1e-9:
                t = min(1.0, max(0.0, (s - acc) / max(length, 1e-9)))
                points.append((p0[0] + t * (p1[0] - p0[0]), p0[1] + t * (p1[1] - p0[1])))
                break
            acc += length
    return points


def _build_basket_shell():
    """Hollow woven basket body: rounded-rect walls, solid floor, open top."""
    hx, hy = BODY_L / 2, BODY_W / 2
    c = CORNER_R
    mid_z = TUB_TOP * 0.5
    bulge = 0.004  # slight outward bulge at mid-height

    outer = section_loft([
        _rect_loop(hx, hy, 0.0, c),
        _rect_loop(hx + bulge, hy + bulge * 0.75, mid_z, c + bulge),
        _rect_loop(hx, hy, TUB_TOP, c),
    ])
    ic = max(c - WALL_T, 0.012)
    inner = section_loft([
        _rect_loop(hx - WALL_T, hy - WALL_T, FLOOR_T, ic),
        _rect_loop(hx + bulge - WALL_T, hy + bulge * 0.75 - WALL_T, mid_z, ic + bulge),
        _rect_loop(hx - WALL_T, hy - WALL_T, TUB_TOP + 0.01, ic),
    ])
    return boolean_difference(outer, inner)


def _build_dome_shell():
    """Domed woven top with hatch opening cut through."""
    dome = section_loft([
        _rect_loop(BODY_L / 2, BODY_W / 2, DOME_Z0, CORNER_R),
        _rect_loop(BODY_L / 2 - 0.008, BODY_W / 2 - 0.006, 0.300, CORNER_R + 0.012),
        _rect_loop(DOME_TOP_HX, DOME_TOP_HY, DOME_Z1, CORNER_R + 0.015),
    ])
    hole_cx = (HATCH_X0 + HATCH_X1) / 2.0
    hole_hx = (HATCH_X1 - HATCH_X0) / 2.0
    cutter = section_loft([
        _rect_loop(hole_hx, HATCH_HY, DOME_Z0 - 0.01, 0.012, cx=hole_cx),
        _rect_loop(hole_hx, HATCH_HY, DOME_Z1 + 0.01, 0.012, cx=hole_cx),
    ])
    return boolean_difference(dome, cutter)


def _weave_band_on_body(part, hx, hy, z, c, bulge, idx):
    """Emit a closed horizontal weave band tube on the body at height z."""
    band_hx = hx + bulge + 0.002
    band_hy = hy + bulge * 0.75 + 0.002
    band_c = c + bulge + 0.002
    path = _rect_loop(band_hx, band_hy, z, band_c)
    part.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                path,
                radius=BAND_RADIUS,
                closed_spline=True,
                radial_segments=8,
                samples_per_segment=4,
            ),
            f"weave_band_{idx}",
        ),
        origin=Origin(),
        material="rattan_weave",
        name=f"weave_band_{idx}",
    )


def _weave_stake(part, x, y, z0, z1, idx):
    """Emit a vertical weave stake tube."""
    part.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(x, y, z0), (x, y, (z0 + z1) / 2), (x, y, z1)],
                radius=STAKE_RADIUS,
                radial_segments=8,
                samples_per_segment=3,
                cap_ends=True,
            ),
            f"weave_stake_{idx}",
        ),
        origin=Origin(),
        material="rattan_dark",
        name=f"weave_stake_{idx}",
    )


# ----------------------------------------------------------- build model

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wicker_pet_basket")

    model.material("rattan_tan", rgba=RATTAN_TAN)
    model.material("rattan_dark", rgba=RATTAN_DARK)
    model.material("rattan_light", rgba=RATTAN_LIGHT)
    model.material("rattan_weave", rgba=RATTAN_WEAVE)
    model.material("rattan_rim", rgba=RATTAN_RIM)
    model.material("pad_blue", rgba=PAD_BLUE)
    model.material("handle_brown", rgba=HANDLE_BROWN)

    # --------------------------------------------------- carrier_body (root)
    body = model.part("carrier_body")

    # main hollow basket shell
    body.visual(
        mesh_from_geometry(_build_basket_shell(), "basket_shell"),
        origin=Origin(),
        material="rattan_tan",
        name="basket_shell",
    )

    # domed woven top with hatch opening
    body.visual(
        mesh_from_geometry(_build_dome_shell(), "dome_shell"),
        origin=Origin(),
        material="rattan_tan",
        name="dome_top_shell",
    )

    # ---- horizontal weave bands on body (loop-emitted) ----
    hx, hy = BODY_L / 2, BODY_W / 2
    band_span = TUB_TOP - FLOOR_T - 0.040
    for i in range(N_BANDS_BODY):
        z = FLOOR_T + 0.020 + i * band_span / max(N_BANDS_BODY - 1, 1)
        t = z / TUB_TOP
        bulge = 0.004 * math.sin(t * math.pi)
        _weave_band_on_body(body, hx, hy, z, CORNER_R, bulge, i)

    # ---- horizontal weave bands on dome ----
    for i in range(N_BANDS_DOME):
        t = (i + 1) / (N_BANDS_DOME + 1)
        z = DOME_Z0 + t * (DOME_Z1 - DOME_Z0)
        dome_hx = BODY_L / 2 - t * (BODY_L / 2 - DOME_TOP_HX)
        dome_hy = BODY_W / 2 - t * (BODY_W / 2 - DOME_TOP_HY)
        dome_c = CORNER_R + t * 0.015
        band_path = _rect_loop(dome_hx + 0.002, dome_hy + 0.002, z, dome_c + 0.002)
        idx = N_BANDS_BODY + i
        body.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    band_path,
                    radius=BAND_RADIUS,
                    closed_spline=True,
                    radial_segments=8,
                    samples_per_segment=4,
                ),
                f"weave_band_{idx}",
            ),
            origin=Origin(),
            material="rattan_weave",
            name=f"weave_band_{idx}",
        )

    # ---- vertical stakes (loop-emitted, shared helper) ----
    stake_xy = _distribute_perimeter(hx, hy, CORNER_R, N_STAKES)
    for i, (sx, sy) in enumerate(stake_xy):
        # slight outward offset so stakes sit proud of the shell
        dist = math.sqrt(sx * sx + sy * sy)
        ox = sx / dist * 0.002 if dist > 0 else 0.0
        oy = sy / dist * 0.002 if dist > 0 else 0.0
        _weave_stake(body, sx + ox, sy + oy, FLOOR_T + 0.006, TUB_TOP - 0.004, i)

    # ---- thick rim band at basket top ----
    rim_path = _rect_loop(hx + 0.003, hy + 0.003, TUB_TOP + 0.001, CORNER_R + 0.003)
    body.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                rim_path,
                radius=0.005,
                closed_spline=True,
                radial_segments=10,
                samples_per_segment=4,
            ),
            "basket_rim",
        ),
        origin=Origin(),
        material="rattan_rim",
        name="basket_rim",
    )

    # ---- dome rim band around hatch opening ----
    hatch_rim_path = _rect_loop(
        (HATCH_X1 - HATCH_X0) / 2 + 0.010,
        HATCH_HY + 0.010,
        DOME_Z1 + 0.001,
        0.016,
        cx=(HATCH_X0 + HATCH_X1) / 2,
    )
    body.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                hatch_rim_path,
                radius=0.004,
                closed_spline=True,
                radial_segments=8,
                samples_per_segment=4,
            ),
            "hatch_rim_band",
        ),
        origin=Origin(),
        material="rattan_rim",
        name="hatch_rim_band",
    )

    # ---- arched carry handle (Y-oriented bail, ahead of hatch) ----
    handle_pts = [
        (0.12, -0.072, DOME_Z1 - 0.004),
        (0.12, -0.048, DOME_Z1 + 0.038),
        (0.12, 0.0, DOME_Z1 + 0.054),
        (0.12, 0.048, DOME_Z1 + 0.038),
        (0.12, 0.072, DOME_Z1 - 0.004),
    ]
    body.visual(
        mesh_from_geometry(
            tube_from_spline_points(handle_pts, radius=0.007, radial_segments=12),
            "carry_handle",
        ),
        origin=Origin(),
        material="handle_brown",
        name="carry_handle_loop",
    )

    # ---- interior pet pad ----
    body.visual(
        Box((0.38, 0.21, 0.038)),
        origin=Origin(xyz=(-0.01, 0.0, FLOOR_T + 0.019)),
        material="pad_blue",
        name="pet_pad",
    )

    # ----------------------------------------- top_hatch (woven domed lid)
    hatch = model.part("top_hatch")

    lid_hx = (HATCH_X1 - HATCH_X0) / 2 + 0.008   # ~0.108
    lid_hy = HATCH_HY + 0.008                       # ~0.078

    # woven dome cap (low-profile lofted dome matching the opening)
    lid_cap = section_loft([
        _rect_loop(lid_hx, lid_hy, 0.001, 0.014, cx=lid_hx),
        _rect_loop(lid_hx * 0.88, lid_hy * 0.88, 0.010, 0.020, cx=lid_hx),
        _rect_loop(lid_hx * 0.65, lid_hy * 0.65, 0.020, 0.026, cx=lid_hx),
    ])
    hatch.visual(
        mesh_from_geometry(lid_cap, "lid_woven_cap"),
        origin=Origin(),
        material="rattan_tan",
        name="lid_woven_cap",
    )

    # lid rim band (thicker woven border around the lid edge)
    lid_rim_path = _rect_loop(lid_hx, lid_hy, 0.003, 0.014, cx=lid_hx)
    hatch.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                lid_rim_path,
                radius=0.004,
                closed_spline=True,
                radial_segments=8,
                samples_per_segment=4,
            ),
            "lid_rim_band",
        ),
        origin=Origin(),
        material="rattan_rim",
        name="lid_rim_band",
    )

    # small woven latch tab at the free (front) edge
    hatch.visual(
        Box((0.018, 0.024, 0.010)),
        origin=Origin(xyz=(2 * lid_hx - 0.006, 0.0, 0.010)),
        material="rattan_dark",
        name="lid_latch_tab",
    )

    # ---- articulation: hinge at rear of hatch opening ----
    model.articulation(
        "body_to_top_hatch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hatch,
        origin=Origin(xyz=(HATCH_X0 - 0.008, 0.0, DOME_Z1 - 0.001)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=2.4),
    )

    return model


# ----------------------------------------------------------- tests

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("carrier_body")
    hatch = object_model.get_part("top_hatch")
    hatch_joint = object_model.get_articulation("body_to_top_hatch")

    # closed woven lid seats on the dome rim (small overlap for seating)
    ctx.allow_overlap(
        hatch, body,
        reason="closed woven lid seats on the dome rim around the hatch opening",
    )

    # ---- woven basket body structure exists ----
    basket_vis = body.get_visual("basket_shell")
    ctx.check(
        "basket_shell_woven_body_exists",
        basket_vis is not None,
        details="carrier_body must have a basket_shell visual (rounded-rect woven body)",
    )

    dome_vis = body.get_visual("dome_top_shell")
    ctx.check(
        "dome_top_shell_woven_exists",
        dome_vis is not None,
        details="carrier_body must have a dome_top_shell visual (woven dome)",
    )

    band_vis = body.get_visual("weave_band_0")
    ctx.check(
        "weave_band_0_exists",
        band_vis is not None,
        details="carrier_body must have weave_band_0 (horizontal woven band)",
    )

    stake_vis = body.get_visual("weave_stake_0")
    ctx.check(
        "weave_stake_0_exists",
        stake_vis is not None,
        details="carrier_body must have weave_stake_0 (vertical woven stake)",
    )

    handle_vis = body.get_visual("carry_handle_loop")
    ctx.check(
        "carry_handle_loop_exists",
        handle_vis is not None,
        details="carrier_body must have carry_handle_loop",
    )

    pad_vis = body.get_visual("pet_pad")
    ctx.check(
        "pet_pad_exists",
        pad_vis is not None,
        details="carrier_body must have pet_pad",
    )

    # ---- closed pose: lid covers opening, contacts dome ----
    with ctx.pose({hatch_joint: 0.0}):
        ctx.expect_overlap(hatch, body, axes="xy", min_overlap=0.08)
        ctx.expect_contact(hatch, body)
        lid_aabb = ctx.part_world_aabb(hatch)
        ctx.check(
            "hatch_closed_covers_opening",
            lid_aabb is not None
            and lid_aabb[0][0] < HATCH_X0
            and lid_aabb[1][0] > HATCH_X1,
            details=f"lid aabb={lid_aabb}",
        )
        ctx.check(
            "hatch_closed_rests_on_dome_top",
            lid_aabb is not None
            and DOME_Z1 - 0.005 < lid_aabb[0][2] < DOME_Z1 + 0.008,
            details=f"lid aabb={lid_aabb}",
        )

    # ---- open lid: free edge swings up above the dome ----
    with ctx.pose({hatch_joint: 2.0}):
        lid_aabb = ctx.part_world_aabb(hatch)
        ctx.check(
            "hatch_open_rises_above_dome",
            lid_aabb is not None and lid_aabb[1][2] > DOME_Z1 + 0.10,
            details=f"open lid aabb={lid_aabb}",
        )

    # ---- pet pad inside on floor ----
    pad_aabb = ctx.part_element_world_aabb(body, elem="pet_pad")
    ctx.check(
        "pet_pad_inside_on_floor",
        pad_aabb is not None
        and pad_aabb[0][2] < FLOOR_T + 0.010
        and pad_aabb[1][2] > FLOOR_T
        and pad_aabb[1][0] < BODY_L / 2 - WALL_T,
        details=f"pad aabb={pad_aabb}",
    )

    # ---- hinge origin near real geometry ----
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)

    return ctx.report()


object_model = build_object_model()
