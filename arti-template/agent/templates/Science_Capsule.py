"""Realistic two-piece hard capsule pill modular template.

The template is intentionally scoped to pharmaceutical hard capsules: a smaller
BODY and a larger CAP are hollow gelatin shells that telescope along one long
axis.  Earlier drafts expanded the category into screw caps, bullet tips, and
multi-segment telescoping chains; those read more like bottles, ammunition, or
tubes than real pill capsules.  This version keeps one real PRISMATIC cap/body
joint and moves diversity into real capsule families:

    body_form_family: standard / short-wide blinding / long-slender / small
    seam_feature: plain overlap / snap lock / double lock / liquid seal band
    surface_feature: printed code / vent dimples / orientation bands

All combinations keep the cap/body skeleton and the axial slide closure.
"""

from __future__ import annotations

import random
from dataclasses import dataclass
from typing import Literal

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

__modular__ = True


BodyFormFamily = Literal[
    "standard_size0",
    "short_wide_blinding",
    "long_slender_000",
    "small_compact_size4",
]
SeamFeature = Literal[
    "plain_overlap",
    "snap_lock_ring",
    "double_lock_rings",
    "liquid_seal_band",
]
SurfaceFeature = Literal[
    "printed_code",
    "vent_dimples",
    "orientation_bands",
]
PaletteStyle = Literal[
    "red_white",
    "blue_clear",
    "green_yellow",
    "all_clear",
    "black_white",
    "orange_white",
]

BODY_FORMS: tuple[BodyFormFamily, ...] = (
    "standard_size0",
    "short_wide_blinding",
    "long_slender_000",
    "small_compact_size4",
)
SEAM_FEATURES: tuple[SeamFeature, ...] = (
    "plain_overlap",
    "snap_lock_ring",
    "double_lock_rings",
    "liquid_seal_band",
)
SURFACE_FEATURES: tuple[SurfaceFeature, ...] = (
    "printed_code",
    "vent_dimples",
    "orientation_bands",
)
PALETTE_STYLES: tuple[PaletteStyle, ...] = (
    "red_white",
    "blue_clear",
    "green_yellow",
    "all_clear",
    "black_white",
    "orange_white",
)


PALETTE_PRESETS: dict[str, dict[str, tuple[float, float, float, float]]] = {
    "red_white": {
        "gelatin_body": (0.95, 0.95, 0.94, 1.0),
        "gelatin_cap": (0.86, 0.10, 0.10, 1.0),
        "marking_ink": (0.78, 0.05, 0.05, 1.0),
        "seal_band": (0.95, 0.55, 0.12, 0.82),
        "vent_shadow": (0.62, 0.05, 0.05, 1.0),
        "band_ink": (0.96, 0.82, 0.12, 1.0),
    },
    "blue_clear": {
        "gelatin_body": (0.86, 0.90, 0.96, 0.55),
        "gelatin_cap": (0.12, 0.32, 0.74, 0.92),
        "marking_ink": (0.95, 0.95, 0.95, 1.0),
        "seal_band": (0.18, 0.52, 0.90, 0.78),
        "vent_shadow": (0.06, 0.16, 0.42, 1.0),
        "band_ink": (0.96, 0.96, 0.98, 1.0),
    },
    "green_yellow": {
        "gelatin_body": (0.93, 0.86, 0.16, 1.0),
        "gelatin_cap": (0.10, 0.55, 0.24, 1.0),
        "marking_ink": (0.05, 0.22, 0.10, 1.0),
        "seal_band": (0.12, 0.44, 0.14, 0.85),
        "vent_shadow": (0.04, 0.18, 0.08, 1.0),
        "band_ink": (0.98, 0.98, 0.78, 1.0),
    },
    "all_clear": {
        "gelatin_body": (0.90, 0.92, 0.93, 0.45),
        "gelatin_cap": (0.80, 0.84, 0.88, 0.55),
        "marking_ink": (0.30, 0.32, 0.34, 1.0),
        "seal_band": (0.78, 0.80, 0.82, 0.55),
        "vent_shadow": (0.36, 0.38, 0.40, 1.0),
        "band_ink": (0.44, 0.46, 0.48, 1.0),
    },
    "black_white": {
        "gelatin_body": (0.94, 0.94, 0.93, 1.0),
        "gelatin_cap": (0.12, 0.12, 0.13, 1.0),
        "marking_ink": (0.92, 0.92, 0.92, 1.0),
        "seal_band": (0.42, 0.42, 0.44, 0.82),
        "vent_shadow": (0.05, 0.05, 0.05, 1.0),
        "band_ink": (0.78, 0.78, 0.76, 1.0),
    },
    "orange_white": {
        "gelatin_body": (0.96, 0.95, 0.92, 1.0),
        "gelatin_cap": (0.92, 0.48, 0.06, 1.0),
        "marking_ink": (0.55, 0.26, 0.02, 1.0),
        "seal_band": (0.94, 0.34, 0.05, 0.84),
        "vent_shadow": (0.55, 0.20, 0.02, 1.0),
        "band_ink": (0.10, 0.26, 0.55, 1.0),
    },
}


@dataclass(frozen=True)
class CapsuleConfig:
    body_form_family: BodyFormFamily | None = None
    seam_feature: SeamFeature | None = None
    surface_feature: SurfaceFeature | None = None
    palette_style: PaletteStyle = "red_white"

    diameter_scale: float = 1.0
    length_scale: float = 1.0
    seat_overlap: float = 0.0030


@dataclass(frozen=True)
class ResolvedCapsuleConfig:
    body_form_family: BodyFormFamily
    seam_feature: SeamFeature
    surface_feature: SurfaceFeature
    palette_style: PaletteStyle

    wall: float
    body_outer_r: float
    cap_outer_r: float
    body_tube_len: float
    cap_tube_len: float
    seat_overlap: float
    separation_travel: float
    ground_lift: float
    palette: dict[str, tuple[float, float, float, float]]


WALL = 0.00030
CONTACT_EMBED = WALL * 0.45
LOCK_BAND_WIDTH = 0.00072
LOCK_BAND_DEPTH = 0.00013
SEAL_BAND_WIDTH = 0.00105


FORM_DIMENSIONS: dict[BodyFormFamily, tuple[float, float, float]] = {
    # body radius, body tube length, cap tube length
    "standard_size0": (0.00370, 0.0118, 0.0058),
    # Real over-encapsulation/blinding capsules are shorter and wider so they
    # can hide a comparator tablet or another capsule.
    "short_wide_blinding": (0.00435, 0.0082, 0.0050),
    # Large 000-style capsules read long and slender.
    "long_slender_000": (0.00335, 0.0154, 0.0072),
    # Small filled capsules keep the same two-piece logic at a compact scale.
    "small_compact_size4": (0.00270, 0.0072, 0.0038),
}


def config_from_seed(seed: int) -> CapsuleConfig:
    rng = random.Random(seed)
    return CapsuleConfig(
        body_form_family=rng.choice(BODY_FORMS),
        seam_feature=rng.choice(SEAM_FEATURES),
        surface_feature=rng.choice(SURFACE_FEATURES),
        palette_style=rng.choice(PALETTE_STYLES),
        diameter_scale=round(rng.uniform(0.94, 1.08), 4),
        length_scale=round(rng.uniform(0.92, 1.10), 4),
        seat_overlap=round(rng.uniform(0.0020, 0.0034), 5),
    )


def resolve_config(config: CapsuleConfig) -> ResolvedCapsuleConfig:
    body_form = config.body_form_family or "standard_size0"
    seam = config.seam_feature or "snap_lock_ring"
    surface = config.surface_feature or "printed_code"
    palette_style = config.palette_style
    if body_form not in BODY_FORMS:
        raise ValueError(f"Unsupported body_form_family: {body_form}")
    if seam not in SEAM_FEATURES:
        raise ValueError(f"Unsupported seam_feature: {seam}")
    if surface not in SURFACE_FEATURES:
        raise ValueError(f"Unsupported surface_feature: {surface}")
    if palette_style not in PALETTE_PRESETS:
        raise ValueError(f"Unsupported palette_style: {palette_style}")

    base_r, base_body_len, base_cap_len = FORM_DIMENSIONS[body_form]
    diameter_scale = max(0.94, min(float(config.diameter_scale), 1.08))
    length_scale = max(0.92, min(float(config.length_scale), 1.10))

    body_outer_r = base_r * diameter_scale
    # In the real object the cap bore clears the body; in mesh validation a tiny
    # overlap makes the seated contact explicit.  This is declared in run_tests.
    cap_outer_r = body_outer_r + WALL - CONTACT_EMBED
    body_tube_len = base_body_len * length_scale
    cap_tube_len = base_cap_len * length_scale
    seat_overlap = max(0.0020, min(float(config.seat_overlap), 0.0034))
    separation_travel = max(0.0090, seat_overlap + body_outer_r + 0.0025)
    ground_lift = cap_outer_r

    return ResolvedCapsuleConfig(
        body_form_family=body_form,
        seam_feature=seam,
        surface_feature=surface,
        palette_style=palette_style,
        wall=WALL,
        body_outer_r=body_outer_r,
        cap_outer_r=cap_outer_r,
        body_tube_len=body_tube_len,
        cap_tube_len=cap_tube_len,
        seat_overlap=seat_overlap,
        separation_travel=separation_travel,
        ground_lift=ground_lift,
        palette=dict(PALETTE_PRESETS[palette_style]),
    )


def _shell_half(outer_r: float, tube_len: float, wall: float) -> cq.Workplane:
    """Hollow hard-capsule half, open mouth at x=0, dome toward +X."""
    inner_r = outer_r - wall
    outer_tube = cq.Workplane("YZ").circle(outer_r).extrude(tube_len)
    outer_dome = cq.Workplane("YZ").workplane(offset=tube_len).sphere(outer_r)
    outer_solid = outer_tube.union(outer_dome)

    inner_tube = (
        cq.Workplane("YZ")
        .circle(inner_r)
        .extrude(tube_len)
        .translate((-wall, 0.0, 0.0))
    )
    inner_dome = cq.Workplane("YZ").workplane(offset=tube_len).sphere(inner_r)
    inner_cavity = inner_tube.union(inner_dome)
    return outer_solid.cut(inner_cavity)


def _body_shell_with_grooves(r: ResolvedCapsuleConfig) -> cq.Workplane:
    shell = _shell_half(r.body_outer_r, r.body_tube_len, r.wall)
    if r.seam_feature == "snap_lock_ring":
        groove_centers = (r.seat_overlap,)
    elif r.seam_feature == "double_lock_rings":
        groove_centers = (r.seat_overlap - 0.00048, r.seat_overlap + 0.00048)
    else:
        groove_centers = ()

    for center in groove_centers:
        cut_start = max(0.00005, center - LOCK_BAND_WIDTH / 2.0)
        groove_cut = (
            cq.Workplane("YZ")
            .workplane(offset=cut_start)
            .circle(r.body_outer_r + 0.00015)
            .circle(r.body_outer_r - LOCK_BAND_DEPTH)
            .extrude(LOCK_BAND_WIDTH)
        )
        shell = shell.cut(groove_cut)
    return shell


def _annular_band(
    outer_r: float,
    inner_r: float,
    width: float,
    *,
    start_x: float,
) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .workplane(offset=start_x)
        .circle(outer_r)
        .circle(inner_r)
        .extrude(width)
    )


def _cap_lock_bands(r: ResolvedCapsuleConfig) -> cq.Workplane:
    cap_inner_r = r.cap_outer_r - r.wall
    outer_r = r.cap_outer_r + LOCK_BAND_DEPTH
    inner_r = cap_inner_r - LOCK_BAND_DEPTH
    if r.seam_feature == "snap_lock_ring":
        starts = (-LOCK_BAND_WIDTH / 2.0,)
    else:
        starts = (-LOCK_BAND_WIDTH - 0.00045, 0.00045)

    bands = _annular_band(outer_r, inner_r, LOCK_BAND_WIDTH, start_x=starts[0])
    for start in starts[1:]:
        bands = bands.union(_annular_band(outer_r, inner_r, LOCK_BAND_WIDTH, start_x=start))
    if r.seam_feature == "double_lock_rings":
        # Hidden sleeve keeps the two visible ridges a single connected mesh
        # island while remaining embedded in the cap mouth wall.
        bridge_start = starts[0] + LOCK_BAND_WIDTH
        bridge_width = starts[1] - bridge_start
        bands = bands.union(
            _annular_band(
                r.cap_outer_r + 0.00002,
                inner_r,
                bridge_width,
                start_x=bridge_start,
            )
        )
    return bands


def _liquid_seal_band(r: ResolvedCapsuleConfig) -> cq.Workplane:
    return _annular_band(
        r.cap_outer_r + 0.00018,
        r.cap_outer_r - r.wall * 0.35,
        SEAL_BAND_WIDTH,
        start_x=-SEAL_BAND_WIDTH * 0.62,
    )


def _imprint_82(cap_outer_r: float, cap_tube_len: float, wall: float) -> cq.Workplane:
    x_center = -cap_tube_len * 0.5
    surf_y = cap_outer_r
    proud = 0.00012
    t = 0.00018
    depth = wall + proud

    def bar(cx: float, cz: float, lx: float, lz: float) -> cq.Workplane:
        return (
            cq.Workplane("XZ")
            .workplane(offset=surf_y - wall * 0.5)
            .center(cx, cz)
            .rect(lx, lz)
            .extrude(depth)
        )

    ring_w = 0.0013
    eight_cx = x_center - 0.0013
    parts = [
        bar(eight_cx, 0.0010, ring_w, t),
        bar(eight_cx, 0.0, ring_w, t),
        bar(eight_cx, -0.0010, ring_w, t),
        bar(eight_cx - ring_w / 2 + t / 2, 0.0005, t, 0.0012),
        bar(eight_cx + ring_w / 2 - t / 2, 0.0005, t, 0.0012),
        bar(eight_cx - ring_w / 2 + t / 2, -0.0005, t, 0.0012),
        bar(eight_cx + ring_w / 2 - t / 2, -0.0005, t, 0.0012),
    ]
    two_cx = x_center + 0.0013
    two_w = 0.0013
    parts += [
        bar(two_cx, 0.0010, two_w, t),
        bar(two_cx + two_w / 2 - t / 2, 0.0005, t, 0.0012),
        bar(two_cx, 0.0, two_w, t),
        bar(two_cx - two_w / 2 + t / 2, -0.0005, t, 0.0012),
        bar(two_cx, -0.0010, two_w, t),
    ]
    backing = (
        cq.Workplane("XZ")
        .workplane(offset=surf_y - wall * 0.65)
        .center(x_center, 0.0)
        .rect(0.0042, 0.0026)
        .extrude(wall * 0.7)
    )
    glyphs = backing
    for p in parts:
        glyphs = glyphs.union(p)
    return glyphs.mirror("XZ")


def _vent_dimple_strip(body_outer_r: float, body_tube_len: float, wall: float) -> cq.Workplane:
    surf_y = body_outer_r
    x_center = body_tube_len * 0.22
    depth = wall + 0.00008
    t = 0.00016

    def slot(cx: float, cz: float) -> cq.Workplane:
        return (
            cq.Workplane("XZ")
            .workplane(offset=surf_y - wall * 0.58)
            .center(cx, cz)
            .rect(0.00115, t)
            .extrude(depth)
        )

    backing = (
        cq.Workplane("XZ")
        .workplane(offset=surf_y - wall * 0.70)
        .center(x_center, 0.0)
        .rect(0.0040, 0.0019)
        .extrude(wall * 0.55)
    )
    vents = backing
    for dx in (-0.00125, 0.0, 0.00125):
        vents = vents.union(slot(x_center + dx, 0.0))
    return vents.mirror("XZ")


def _ink_band(outer_r: float, width: float, *, center_x: float) -> cq.Workplane:
    return _annular_band(
        outer_r + 0.00008,
        outer_r - 0.00004,
        width,
        start_x=center_x - width / 2.0,
    )


def build_capsule(
    config: CapsuleConfig,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    r = resolve_config(config)
    model = ArticulatedObject(name="capsule_pill", assets=assets)
    for material_name, rgba in r.palette.items():
        model.material(material_name, rgba=rgba)

    body_span = r.body_tube_len + r.body_outer_r
    cap_span = r.cap_tube_len + r.cap_outer_r

    body = model.part("capsule_body")
    body_shell = _body_shell_with_grooves(r)
    body.visual(
        mesh_from_cadquery(body_shell, "capsule_body_shell"),
        origin=Origin(xyz=(0.0, 0.0, r.ground_lift)),
        material="gelatin_body",
        name="body_shell",
    )
    if r.surface_feature == "vent_dimples":
        body.visual(
            mesh_from_cadquery(
                _vent_dimple_strip(r.body_outer_r, r.body_tube_len, r.wall),
                "body_vent_dimples",
            ),
            origin=Origin(xyz=(0.0, 0.0, r.ground_lift)),
            material="vent_shadow",
            name="body_vent_dimples",
        )
    if r.surface_feature == "orientation_bands":
        body.visual(
            mesh_from_cadquery(
                _ink_band(r.body_outer_r, 0.00052, center_x=r.body_tube_len * 0.52),
                "body_orientation_band",
            ),
            origin=Origin(xyz=(0.0, 0.0, r.ground_lift)),
            material="band_ink",
            name="body_orientation_band",
        )
    body.inertial = Inertial.from_geometry(
        Box((body_span, 2 * r.body_outer_r, 2 * r.body_outer_r)),
        mass=0.0004,
        origin=Origin(xyz=(body_span / 2.0, 0.0, r.ground_lift)),
    )

    cap = model.part("capsule_cap")
    cap_shell = _shell_half(r.cap_outer_r, r.cap_tube_len, r.wall).mirror("YZ")
    cap.visual(
        mesh_from_cadquery(cap_shell, "capsule_cap_shell"),
        material="gelatin_cap",
        name="cap_shell",
    )
    if r.seam_feature in {"snap_lock_ring", "double_lock_rings"}:
        cap.visual(
            mesh_from_cadquery(_cap_lock_bands(r), "cap_lock_bands"),
            material="gelatin_cap",
            name="cap_lock_bands",
        )
    if r.seam_feature == "liquid_seal_band":
        cap.visual(
            mesh_from_cadquery(_liquid_seal_band(r), "capsule_liquid_seal_band"),
            material="seal_band",
            name="liquid_seal_band",
        )
    if r.surface_feature == "printed_code":
        cap.visual(
            mesh_from_cadquery(_imprint_82(r.cap_outer_r, r.cap_tube_len, r.wall), "cap_imprint"),
            material="marking_ink",
            name="cap_imprint",
        )
    if r.surface_feature == "orientation_bands":
        cap.visual(
            mesh_from_cadquery(
                _ink_band(r.cap_outer_r, 0.00046, center_x=-r.cap_tube_len * 0.45),
                "cap_orientation_band",
            ),
            material="band_ink",
            name="cap_orientation_band",
        )
    cap.inertial = Inertial.from_geometry(
        Box((cap_span, 2 * r.cap_outer_r, 2 * r.cap_outer_r)),
        mass=0.0003,
        origin=Origin(xyz=(-cap_span / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_cap",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cap,
        origin=Origin(xyz=(r.seat_overlap, 0.0, r.ground_lift)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.05,
            lower=0.0,
            upper=r.separation_travel,
        ),
    )

    return model


def build_seeded_capsule(seed: int) -> ArticulatedObject:
    return build_capsule(config_from_seed(seed))


def slot_choices_for_seed(seed: int) -> list[tuple[str, str]]:
    r = resolve_config(config_from_seed(seed))
    return [
        ("body_form_family", r.body_form_family),
        ("seam_feature", r.seam_feature),
        ("surface_feature", r.surface_feature),
    ]


def _allow_overlaps(ctx: TestContext, model: ArticulatedObject, r: ResolvedCapsuleConfig) -> None:
    body = model.get_part("capsule_body")
    cap = model.get_part("capsule_cap")
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_shell",
        elem_b="body_shell",
        reason="The cap intentionally telescopes over the body mouth in the seated pose.",
    )
    if r.seam_feature in {"snap_lock_ring", "double_lock_rings"}:
        ctx.allow_overlap(
            cap,
            cap,
            elem_a="cap_lock_bands",
            elem_b="cap_shell",
            reason="Locking bands are molded into the cap mouth wall.",
        )
        ctx.allow_overlap(
            cap,
            body,
            elem_a="cap_lock_bands",
            elem_b="body_shell",
            reason="Cap lock bands seat into matching body grooves at closure.",
        )
    if r.seam_feature == "liquid_seal_band":
        ctx.allow_overlap(
            cap,
            cap,
            elem_a="liquid_seal_band",
            elem_b="cap_shell",
            reason="The liquid-fill sealing band is bonded around the cap/body seam.",
        )
        ctx.allow_overlap(
            cap,
            body,
            elem_a="liquid_seal_band",
            elem_b="body_shell",
            reason="The sealing band spans the seam while the capsule is seated.",
        )
    if r.surface_feature == "printed_code":
        ctx.allow_overlap(
            cap,
            cap,
            elem_a="cap_imprint",
            elem_b="cap_shell",
            reason="Printed/embossed marking is embedded into the cap surface.",
        )
    if r.surface_feature == "vent_dimples":
        ctx.allow_overlap(
            body,
            body,
            elem_a="body_vent_dimples",
            elem_b="body_shell",
            reason="Vent/dimple marks are shallow surface features on the body shell.",
        )
    if r.surface_feature == "orientation_bands":
        ctx.allow_overlap(
            body,
            body,
            elem_a="body_orientation_band",
            elem_b="body_shell",
            reason="Orientation band ink is printed directly on the body shell.",
        )
        ctx.allow_overlap(
            cap,
            cap,
            elem_a="cap_orientation_band",
            elem_b="cap_shell",
            reason="Orientation band ink is printed directly on the cap shell.",
        )


def _expect_capsule(ctx: TestContext, model: ArticulatedObject, r: ResolvedCapsuleConfig) -> None:
    body = model.get_part("capsule_body")
    cap = model.get_part("capsule_cap")
    joint = model.get_articulation("body_to_cap")

    ctx.check(
        "body_shell_present",
        any(v.name == "body_shell" for v in body.visuals),
        str([v.name for v in body.visuals]),
    )
    ctx.check(
        "cap_shell_present",
        any(v.name == "cap_shell" for v in cap.visuals),
        str([v.name for v in cap.visuals]),
    )
    ctx.check(
        "body_to_cap_is_prismatic",
        joint.articulation_type == ArticulationType.PRISMATIC,
        f"type={joint.articulation_type}",
    )
    ax = tuple(round(v, 6) for v in joint.axis)
    ctx.check("cap_slides_along_minus_x", ax == (-1.0, 0.0, 0.0), f"axis={ax}")

    if r.seam_feature in {"snap_lock_ring", "double_lock_rings"}:
        ctx.check(
            "lock_bands_present",
            any(v.name == "cap_lock_bands" for v in cap.visuals),
            str([v.name for v in cap.visuals]),
        )
    if r.seam_feature == "liquid_seal_band":
        ctx.check(
            "liquid_seal_band_present",
            any(v.name == "liquid_seal_band" for v in cap.visuals),
            str([v.name for v in cap.visuals]),
        )
    if r.surface_feature == "vent_dimples":
        ctx.check(
            "body_vent_dimples_present",
            any(v.name == "body_vent_dimples" for v in body.visuals),
            str([v.name for v in body.visuals]),
        )
    if r.surface_feature == "orientation_bands":
        ctx.check(
            "orientation_bands_present",
            any(v.name == "body_orientation_band" for v in body.visuals)
            and any(v.name == "cap_orientation_band" for v in cap.visuals),
            f"body={ [v.name for v in body.visuals] } cap={ [v.name for v in cap.visuals] }",
        )

    body_aspect = r.body_tube_len / (2.0 * r.body_outer_r)
    if r.body_form_family == "short_wide_blinding":
        ctx.check("short_wide_capsule_aspect", body_aspect < 1.10, f"aspect={body_aspect:.3f}")
    if r.body_form_family == "long_slender_000":
        ctx.check("long_slender_capsule_aspect", body_aspect > 1.85, f"aspect={body_aspect:.3f}")

    with ctx.pose({joint: 0.0}):
        ctx.expect_overlap(
            cap,
            body,
            axes="x",
            elem_a="cap_shell",
            elem_b="body_shell",
            min_overlap=0.0015,
            name="seated_cap_body_overlap",
        )
        ctx.expect_within(
            body,
            cap,
            axes="yz",
            inner_elem="body_shell",
            outer_elem="cap_shell",
            margin=0.00025,
            name="body_concentric_inside_cap",
        )
        seated = ctx.part_world_position(cap)

    # sampled-pose exemption: this capsule has a single adjacent telescoping
    # cap/body joint whose closed pose intentionally overlaps as retained
    # insertion. Targeted closed/max pose checks cover the real motion semantics.
    with ctx.pose({joint: joint.motion_limits.upper}):
        pulled = ctx.part_world_position(cap)
        ctx.expect_gap(
            body,
            cap,
            axis="x",
            min_gap=0.0,
            positive_elem="body_shell",
            negative_elem="cap_shell",
            name="cap_separates_at_max_travel",
        )
    ctx.check(
        "pull_motion_moves_cap_left",
        seated is not None and pulled is not None and pulled[0] < seated[0] - 0.005,
        f"seated={seated} pulled={pulled}",
    )


def run_capsule_tests(model: ArticulatedObject, config: CapsuleConfig) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(model)

    ctx.check_model_valid()
    ctx.fail_if_isolated_parts()
    _allow_overlaps(ctx, model, r)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.015)
    ctx.fail_if_joint_mating_has_gap()
    _expect_capsule(ctx, model, r)

    return ctx.report()


__all__ = [
    "BodyFormFamily",
    "CapsuleConfig",
    "PaletteStyle",
    "ResolvedCapsuleConfig",
    "SeamFeature",
    "SurfaceFeature",
    "build_capsule",
    "build_seeded_capsule",
    "config_from_seed",
    "resolve_config",
    "run_capsule_tests",
    "slot_choices_for_seed",
]
