from __future__ import annotations

# Wide-shoulder decanter / carafe with a crimped metal crown cap.
# Body-profile fork of the long-neck beer bottle parent:
#   - broad, low, bulbous round body with a wide rounded belly
#   - very wide sloping shoulder that necks down sharply
#   - comparatively long slim neck
#   - same pry-off crown closure and prismatic pop-off joint
# Frame: bottle axis along +Z (base at z=0, mouth/lip at the top), centered on
#   the z-axis. Scale ~0.23 m tall, ~0.11 m max body diameter.
# Articulations:
#   - crown cap: PRISMATIC, pops/lifts straight up (+Z) off the bottle mouth
#     (pry-off cap, no thread). Seated on the lip when down.

import math

import cadquery as cq

GLASS_RGBA = (0.18, 0.12, 0.07, 0.4)
CAP_RGBA = (0.62, 0.60, 0.58, 1.0)
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- Carafe profile heights (z, in metres) ----------------------------------
BASE_Z = 0.0
BELLY_TOP_Z = 0.100       # widest belly region ends
SHOULDER_START_Z = 0.108  # shoulder slope begins
SHOULDER_TOP_Z = 0.148    # shoulder blends into the slim neck
NECK_TOP_Z = 0.222        # top of the slim neck, where the lip starts
LIP_TOP_Z = 0.230         # top face of the mouth / lip

BODY_R = 0.055            # max belly radius (~0.11 m diameter)
NECK_R = 0.0115           # slim neck radius
LIP_R = 0.0135            # rolled lip is slightly fatter than the neck

WALL = 0.003              # glass wall thickness (hollow shell)
BORE_R = 0.0085           # inner bore radius at the mouth


def _profile_loft(sections, ruled: bool = True) -> cq.Workplane:
    # sections: list of (z, radius). Lofts circular sections stacked along +Z.
    # Ruled lofts give predictable straight segments between sections (no spline
    # overshoot); use many closely-spaced sections to approximate curves.
    wp = cq.Workplane("XY")
    prev = 0.0
    for i, (z, r) in enumerate(sections):
        off = z if i == 0 else z - prev
        if i > 0 or abs(off) > 1e-12:
            wp = wp.workplane(offset=off)
        wp = wp.circle(r)
        prev = z
    return wp.loft(ruled=ruled)


def _carafe_solid() -> cq.Workplane:
    # Outer silhouette: rounded base foot -> wide bulbous belly -> dramatic
    # sloping shoulder -> long slim neck -> slightly flared rolled lip.
    # Many closely-spaced sections approximate smooth curves under ruled loft.
    outer = _profile_loft(
        [
            (BASE_Z, 0.0240),        # flat-bottomed base disc
            (0.003, 0.0260),         # short heel
            (0.007, 0.0310),         # base flare
            (0.012, 0.0370),         # curving outward
            (0.018, 0.0425),         # belly curve building
            (0.026, 0.0475),         # approaching max width
            (0.036, 0.0515),         # belly rounding out
            (0.048, 0.0540),         # near max belly
            (0.060, 0.0550),         # max belly width
            (0.072, 0.0548),         # belly plateau
            (0.084, 0.0535),         # belly gently tapering
            (BELLY_TOP_Z, 0.0515),   # belly region ends
            (0.104, 0.0490),         # shoulder beginning
            (SHOULDER_START_Z, 0.0455),  # shoulder slope starts
            (0.115, 0.0405),         # shoulder narrowing
            (0.122, 0.0345),         # dramatic shoulder
            (0.128, 0.0280),         # shoulder rolls inward
            (0.134, 0.0225),         # approaching neck
            (0.140, 0.0180),         # shoulder-to-neck blend
            (SHOULDER_TOP_Z, 0.0145),  # shoulder ends, neck begins
            (0.158, 0.0125),         # neck settling
            (0.168, NECK_R),         # slim neck
            (NECK_TOP_Z, NECK_R),    # long slim neck continues
            (0.224, 0.0128),         # neck flares to the lip
            (0.227, LIP_R),          # flared lip
            (LIP_TOP_Z, LIP_R),      # flat mouth top
        ]
    )

    # Hollow it: subtract an inner cavity that opens at the mouth so the carafe
    # reads as a real open glass shell (interior bore visible from the top).
    inner = _profile_loft(
        [
            (0.010, 0.0190),         # cavity floor above a solid glass base
            (0.015, 0.0250),         # inner base flare
            (0.022, 0.0330),
            (0.032, 0.0395),
            (0.044, 0.0440),
            (0.058, 0.0475),
            (0.072, 0.0488),
            (0.084, 0.0478),
            (BELLY_TOP_Z, 0.0458),
            (0.104, 0.0432),
            (SHOULDER_START_Z, 0.0398),
            (0.115, 0.0348),
            (0.122, 0.0288),
            (0.128, 0.0225),
            (0.134, 0.0172),
            (0.140, 0.0130),
            (SHOULDER_TOP_Z, 0.0115),
            (0.158, 0.0095),
            (0.168, BORE_R),
            (NECK_TOP_Z, BORE_R),
            (LIP_TOP_Z + 0.004, BORE_R),  # poke out the top so mouth is open
        ]
    )
    return outer.cut(inner)


def _cap_mesh():
    # Crimped crown cap: a short shallow dome top with a fluted (crimped) skirt.
    # Top disk + small crowned center, then 21 crimp "teeth" around the skirt.
    cap_top_z = 0.0
    skirt_h = 0.0080
    top_h = 0.0035
    top_r = 0.0152  # cap sits over the lip
    skirt_r = 0.0150

    # Flat-ish crown top.
    top = (
        cq.Workplane("XY")
        .workplane(offset=cap_top_z)
        .circle(top_r)
        .workplane(offset=top_h)
        .circle(top_r * 0.86)
        .loft(ruled=True)
    )

    # Skirt ring hanging down from under the top rim.
    skirt = (
        cq.Workplane("XY")
        .workplane(offset=cap_top_z - skirt_h)
        .circle(skirt_r)
        .workplane(offset=skirt_h)
        .circle(top_r)
        .loft(ruled=True)
    )
    # Hollow the skirt so the cap clamps over the lip (open underside).
    skirt_bore = (
        cq.Workplane("XY")
        .workplane(offset=cap_top_z - skirt_h - 0.001)
        .circle(skirt_r - 0.0016)
        .workplane(offset=skirt_h + 0.0005)
        .circle(top_r - 0.0016)
        .loft(ruled=True)
    )
    skirt = skirt.cut(skirt_bore)

    cap = top.union(skirt)

    # Crimped flutes: 21 small radial tabs around the bottom skirt edge.
    n_flutes = 21
    flute_z = cap_top_z - skirt_h + 0.0030
    for i in range(n_flutes):
        ang = 2.0 * math.pi * i / n_flutes
        fx = (skirt_r + 0.0004) * math.cos(ang)
        fy = (skirt_r + 0.0004) * math.sin(ang)
        tab = (
            cq.Workplane("XY")
            .workplane(offset=flute_z)
            .center(fx, fy)
            .rect(0.0026, 0.0026)
            .extrude(0.0050)
        )
        cap = cap.union(tab)

    return mesh_from_cadquery(cap, "crown_cap")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="carafe")

    # Dark/brown glass: translucent (alpha < 1) so it reads as glass.
    glass = model.material("dark_glass", rgba=GLASS_RGBA)
    cap_metal = model.material("cap_metal", rgba=CAP_RGBA)

    # ---- carafe bottle (root): hollow dark-glass shell, wide carafe profile ----
    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_cadquery(_carafe_solid(), "bottle_glass"),
        material=glass,
        name="bottle_glass",
    )
    # Inertial from an equivalent solid cylinder spanning the body.
    bottle.inertial = Inertial.from_geometry(
        Cylinder(radius=BODY_R, length=LIP_TOP_Z),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, LIP_TOP_Z / 2.0)),
    )

    # ---- crown cap: crimped metal, pries straight up off the mouth ----
    cap = model.part("crown_cap")
    cap.visual(_cap_mesh(), material=cap_metal, name="crown_cap")
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0152, length=0.012),
        mass=0.004,
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
    )

    # The cap part frame is the top of the lip; the cap geometry hangs down over
    # the lip (skirt grips it). Positive q lifts the cap straight up off the
    # mouth (pry-off, no thread).
    model.articulation(
        "bottle_to_cap",
        ArticulationType.PRISMATIC,
        parent=bottle,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, LIP_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.2, lower=0.0, upper=0.03),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bottle = object_model.get_part("bottle")
    cap = object_model.get_part("crown_cap")
    cap_joint = object_model.get_articulation("bottle_to_cap")

    # --- Dark glass: translucent body material (alpha < 1). ---
    ctx.check(
        "bottle glass is translucent (alpha < 1)",
        GLASS_RGBA[3] < 1.0,
        details=f"glass rgba={GLASS_RGBA}",
    )

    # --- Carafe silhouette: wide squat body, not tall and thin. ---
    body_ext = _ext(ctx.part_world_aabb(bottle))
    body_width = max(body_ext[0], body_ext[1])
    ctx.check(
        "carafe has wide bulbous body (width > 0.08 m)",
        body_width > 0.08,
        details=f"bottle width={body_width:.4f}, extents={body_ext}",
    )
    ctx.check(
        "carafe is squat (height < 3x width)",
        body_ext[2] < 3.0 * body_width,
        details=f"height/width ratio={body_ext[2] / body_width:.2f}, extents={body_ext}",
    )

    # --- Dramatic shoulder: body much wider than neck region. ---
    # The neck should be notably narrower than the body.
    ctx.check(
        "carafe body is much wider than neck (ratio > 3)",
        body_width > 3.0 * (2.0 * NECK_R + 0.004),
        details=f"body_width={body_width:.4f}, neck_dia~{2.0 * NECK_R + 0.004:.4f}",
    )

    # --- Cap seated on the lip when down: cap skirt overlaps the bottle lip. ---
    ctx.allow_overlap(
        cap,
        bottle,
        elem_a="crown_cap",
        elem_b="bottle_glass",
        reason="Crown cap skirt is intentionally crimped down over the bottle lip when seated.",
    )
    ctx.expect_overlap(
        cap, bottle, axes="z", min_overlap=0.002, name="cap seated over the bottle lip"
    )

    # Cap is positioned at the top (mouth), not floating elsewhere.
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap mounted at the bottle mouth (top)",
        cap_pos is not None and cap_pos[2] > 0.20,
        details=f"cap origin={cap_pos}",
    )

    # --- Cap lifts straight up off the mouth (prismatic pop-off). ---
    rest_z = ctx.part_world_position(cap)[2]
    with ctx.pose({cap_joint: 0.03}):
        lifted_z = ctx.part_world_position(cap)[2]
        # When lifted, the cap clears the lip: no overlap with the bottle.
        ctx.expect_gap(cap, bottle, axis="z", max_gap=0.05, min_gap=0.001)
    ctx.check(
        "cap pops straight up off the mouth",
        lifted_z > rest_z + 0.025,
        details=f"rest_z={rest_z}, lifted_z={lifted_z}",
    )

    return ctx.report()


object_model = build_object_model()
