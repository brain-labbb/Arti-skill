from __future__ import annotations

# Wine bottle with a crimped metal crown cap.
# Wine-bottle profile family: tall straight-walled cylindrical body rising to a
# distinct high rounded shoulder that steps sharply into a short straight neck,
# with a punted concave base.
# Frame: bottle axis along +Z (base at z=0, mouth/lip at the top), centered on
#   the z-axis. Scale ~0.28 m tall, ~0.075 m body diameter.
# Articulations:
#   - crown cap: PRISMATIC, pops/lifts straight up (+Z) off the bottle mouth
#     (pry-off cap, no thread). Seated on the lip when down.

import math

import cadquery as cq

GLASS_RGBA = (0.06, 0.10, 0.05, 0.38)  # dark translucent green wine glass
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

# ---- Bottle profile heights (z, in metres) ----------------------------------
BASE_Z = 0.0
BODY_TOP_Z = 0.195       # straight body ends, shoulder begins
SHOULDER_TOP_Z = 0.240   # high rounded shoulder blends into the neck
NECK_TOP_Z = 0.275       # top of the short neck, where the lip starts
LIP_TOP_Z = 0.283        # top face of the mouth / lip

BODY_R = 0.0375          # body radius (~0.075 m diameter)
NECK_R = 0.0140          # neck radius (~0.028 m diameter)
LIP_R = 0.0155           # rolled lip slightly fatter than the neck

WALL = 0.003             # glass wall thickness (hollow shell)
BORE_R = 0.011           # inner bore radius at the mouth

PUNT_DEPTH = 0.015       # punt indentation depth from base
PUNT_SPHERE_R = 0.035    # sphere radius for the punt dome curvature


def _profile_loft(sections, ruled: bool = True) -> cq.Workplane:
    """Loft circular sections stacked along +Z.

    sections: list of (z, radius).  Ruled lofts give predictable straight
    segments between sections (no spline overshoot); use many closely-spaced
    sections to approximate curves.
    """
    wp = cq.Workplane("XY")
    prev = 0.0
    for i, (z, r) in enumerate(sections):
        off = z if i == 0 else z - prev
        if i > 0 or abs(off) > 1e-12:
            wp = wp.workplane(offset=off)
        wp = wp.circle(r)
        prev = z
    return wp.loft(ruled=ruled)


def _bottle_solid() -> cq.Workplane:
    """Build the hollow wine-bottle glass shell with a punted base."""
    # Outer silhouette: base rim -> straight cylindrical body -> high rounded
    # shoulder stepping sharply into a short straight neck -> flared lip.
    outer = _profile_loft(
        [
            (BASE_Z, 0.033),           # base rim
            (0.005, 0.035),            # heel flare
            (0.014, 0.0370),           # body flare
            (0.020, BODY_R),           # body full radius
            (BODY_TOP_Z, BODY_R),      # straight cylindrical body
            # High rounded shoulder — many sections for smooth curve
            (0.200, 0.0373),
            (0.206, 0.0368),
            (0.212, 0.0350),
            (0.218, 0.0320),
            (0.224, 0.0275),
            (0.230, 0.0220),
            (0.234, 0.0180),
            (SHOULDER_TOP_Z, NECK_R + 0.001),  # sharp step to neck
            # Short straight neck
            (0.245, NECK_R),
            (NECK_TOP_Z, NECK_R),
            # Lip
            (0.278, 0.0148),
            (0.280, LIP_R),
            (LIP_TOP_Z, LIP_R),
        ]
    )

    # Cut the punt: a concave dome indentation in the base.
    # Place a sphere below the base whose topmost point is at PUNT_DEPTH.
    # At z=0 the sphere cross-section has radius ~0.0287, fitting inside the
    # 0.033 base rim; at z=PUNT_DEPTH the sphere closes to a point.
    punt_center_z = -(PUNT_SPHERE_R - PUNT_DEPTH)
    punt_sphere = (
        cq.Workplane("XY")
        .sphere(PUNT_SPHERE_R)
        .translate((0, 0, punt_center_z))
    )
    outer = outer.cut(punt_sphere)

    # Hollow interior: inner cavity open at the mouth.
    # Floor starts above the punt — wine bottles have a thick glass base.
    inner = _profile_loft(
        [
            (0.022, 0.018),            # floor above punt (small bore)
            (0.028, 0.028),            # widening
            (0.035, 0.032),            # approaching body bore
            (BODY_TOP_Z - 0.005, BODY_R - WALL),  # body bore
            # Shoulder follows outer with ~WALL offset
            (0.200, 0.0343),
            (0.206, 0.0338),
            (0.212, 0.0320),
            (0.218, 0.0290),
            (0.224, 0.0245),
            (0.230, 0.0190),
            (0.234, 0.0150),
            (SHOULDER_TOP_Z, 0.0120),  # neck bore
            (0.245, BORE_R),
            (NECK_TOP_Z, BORE_R),
            (LIP_TOP_Z + 0.005, BORE_R),  # poke out the top so the mouth is open
        ]
    )
    return outer.cut(inner)


def _cap_mesh():
    """Crimped crown cap: short shallow dome top with 21 fluted teeth."""
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
    model = ArticulatedObject(name="wine_bottle")

    # Dark green glass: translucent (alpha < 1) so it reads as glass.
    glass = model.material("dark_glass", rgba=GLASS_RGBA)
    cap_metal = model.material("cap_metal", rgba=CAP_RGBA)

    # ---- bottle (root): hollow dark-glass wine-bottle shell ----
    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_cadquery(_bottle_solid(), "bottle_glass"),
        material=glass,
        name="bottle_glass",
    )
    # Inertial from an equivalent solid cylinder spanning the body.
    bottle.inertial = Inertial.from_geometry(
        Cylinder(radius=BODY_R, length=LIP_TOP_Z),
        mass=0.45,
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

    # --- Wine-bottle silhouette: tall body, relatively short neck. ---
    body_ext = _ext(ctx.part_world_aabb(bottle))
    ctx.check(
        "wine bottle is tall with cylindrical body",
        body_ext[2] > 0.25 and body_ext[2] > 3.0 * max(body_ext[0], body_ext[1]),
        details=f"bottle extents={body_ext}",
    )

    # --- Wine-bottle body is wider than a typical beer bottle (~75 mm). ---
    body_diameter = max(body_ext[0], body_ext[1])
    ctx.check(
        "wine bottle body diameter ~75mm",
        0.065 < body_diameter < 0.090,
        details=f"body diameter={body_diameter:.4f}",
    )

    # --- High shoulder with short neck (wine profile, not beer long-neck). ---
    total_h = body_ext[2]
    neck_fraction = (LIP_TOP_Z - SHOULDER_TOP_Z) / total_h if total_h > 0 else 0
    ctx.check(
        "wine bottle has high shoulder with short neck (<20% of height)",
        neck_fraction < 0.20,
        details=f"neck fraction={neck_fraction:.3f}",
    )

    # --- Punted base: bottle sits on the ground plane with concave base. ---
    bottle_aabb = ctx.part_world_aabb(bottle)
    ctx.check(
        "bottle base sits at ground level (z~0)",
        bottle_aabb is not None and bottle_aabb[0][2] < 0.003,
        details=f"bottle min_z={bottle_aabb[0][2] if bottle_aabb else None}",
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
        cap_pos is not None and cap_pos[2] > 0.25,
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
