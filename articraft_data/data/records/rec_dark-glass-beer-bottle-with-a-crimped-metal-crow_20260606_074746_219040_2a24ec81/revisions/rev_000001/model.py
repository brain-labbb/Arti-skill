from __future__ import annotations

# Dark glass beer bottle with a crimped metal crown cap.
# Frame: bottle axis along +Z (base at z=0, mouth/lip at the top), centered on
#   the z-axis. Scale ~0.24 m tall, ~0.065 m body diameter.
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

# ---- Bottle profile heights (z, in metres) ----------------------------------
BASE_Z = 0.0
BODY_TOP_Z = 0.120  # body cylinder ends, shoulder begins
SHOULDER_TOP_Z = 0.150  # shoulder blends into the neck
NECK_TOP_Z = 0.222  # top of the slim neck, where the lip starts
LIP_TOP_Z = 0.230  # top face of the mouth / lip

BODY_R = 0.0325  # body radius (~0.065 m diameter)
NECK_R = 0.0115  # slim long-neck radius
LIP_R = 0.0135  # rolled lip is slightly fatter than the neck

WALL = 0.0028  # glass wall thickness (hollow shell)
BORE_R = 0.0085  # inner bore radius at the mouth


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


def _bottle_solid() -> cq.Workplane:
    # Outer silhouette: rounded base -> straight body -> sloped shoulder ->
    # long slim neck -> slightly flared rolled lip.
    outer = _profile_loft(
        [
            (BASE_Z, 0.0235),  # flat-bottomed base disc
            (0.003, 0.0250),  # short straight heel
            (0.008, 0.0305),  # rounded base flare
            (0.013, 0.0322),
            (0.018, 0.0325),  # body full radius
            (BODY_TOP_Z, 0.0325),  # straight cylindrical body
            (0.128, 0.0316),  # shoulder begins
            (0.137, 0.0288),
            (SHOULDER_TOP_Z, 0.0205),  # sloped shoulder
            (0.160, 0.0150),
            (0.170, 0.0122),  # shoulder rolls into neck
            (NECK_TOP_Z, NECK_R),  # long slim neck
            (0.224, 0.0128),  # neck flares to the lip
            (0.227, LIP_R),  # flared lip
            (LIP_TOP_Z, LIP_R),  # flat mouth top
        ]
    )

    # Hollow it: subtract an inner cavity that opens at the mouth so the bottle
    # reads as a real open glass shell (interior bore visible from the top).
    inner = _profile_loft(
        [
            (0.010, 0.0200),  # cavity floor above a solid glass base
            (0.018, 0.0270),
            (0.024, 0.0297),
            (BODY_TOP_Z - 0.004, 0.0297),
            (0.128, 0.0288),
            (0.137, 0.0260),
            (SHOULDER_TOP_Z, 0.0177),
            (0.160, 0.0122),
            (0.170, 0.0094),
            (NECK_TOP_Z, BORE_R),
            (LIP_TOP_Z + 0.004, BORE_R),  # poke out the top so the mouth is open
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
    model = ArticulatedObject(name="beer_bottle")

    # Dark/brown glass: translucent (alpha < 1) so it reads as glass.
    glass = model.material("dark_glass", rgba=GLASS_RGBA)
    cap_metal = model.material("cap_metal", rgba=CAP_RGBA)

    # ---- bottle (root): hollow dark-glass shell ----
    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_cadquery(_bottle_solid(), "bottle_glass"),
        material=glass,
        name="bottle_glass",
    )
    # Inertial from an equivalent solid cylinder spanning the body.
    bottle.inertial = Inertial.from_geometry(
        Cylinder(radius=BODY_R, length=LIP_TOP_Z),
        mass=0.30,
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

    # --- Long neck: the bottle is tall, and the upper neck region is narrow. ---
    body_ext = _ext(ctx.part_world_aabb(bottle))
    ctx.check(
        "bottle is tall (long-neck silhouette)",
        body_ext[2] > 0.20 and body_ext[2] > 2.5 * max(body_ext[0], body_ext[1]),
        details=f"bottle extents={body_ext}",
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
