from __future__ import annotations

# Dark glass Rhine/Alsace flute beer bottle with a crimped metal crown cap.
# Frame: bottle axis along +Z (base at z=0, mouth/lip at the top), centered on
#   the z-axis. Scale ~0.30 m tall, ~0.054 m max body diameter.
# Profile: tall slim flute form - continuous smooth taper from body to neck
#   with no distinct shoulder break (Rhine/Alsace wine bottle silhouette).
# Articulations:
#   - crown cap: PRISMATIC, pops/lifts straight up (+Z) off the bottle mouth
#     (pry-off cap, no thread). Seated on the lip when down.

import math

import cadquery as cq

GLASS_RGBA = (0.12, 0.10, 0.06, 0.45)
CAP_RGBA = (0.55, 0.52, 0.18, 1.0)  # gold/brass-toned crown cap
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
NECK_TOP_Z = 0.290  # top of the slim neck, where the lip starts
LIP_TOP_Z = 0.298   # top face of the mouth / lip

MAX_BODY_R = 0.0270  # max body radius (~0.054 m diameter - slim flute)
NECK_R = 0.0110      # slim neck radius
LIP_R = 0.0130       # rolled lip is slightly fatter than the neck

WALL = 0.0025        # glass wall thickness (hollow shell)
BORE_R = 0.0082      # inner bore radius at the mouth


def _revolve_profile(profile_pts: list[tuple[float, float]]) -> cq.Workplane:
    """Revolve (z, radius) profile points around the Z axis to create a solid.

    Draws a closed half-section on the XZ plane (x=radius, y=z) and revolves
    360° around the Z axis (x=0 line).
    """
    wp = cq.Workplane("XZ")
    # Start on axis at the z of the first profile point
    wp = wp.moveTo(0.0, profile_pts[0][0])
    # Go out to first radius at first z
    wp = wp.lineTo(profile_pts[0][1], profile_pts[0][0])
    # Follow profile points (radius=x, z=y in workplane)
    for z, r in profile_pts[1:]:
        wp = wp.lineTo(r, z)
    # Back to axis at the z of the last point
    wp = wp.lineTo(0.0, profile_pts[-1][0])
    # Close wire (back along axis to start)
    wp = wp.close()
    # Revolve 360° around the Z axis (x=0 line in XZ workplane)
    return wp.revolve(360, (0, 0), (0, 1))


def _flute_outer_profile() -> list[tuple[float, float]]:
    """Smooth flute bottle outer silhouette as (z, radius) pairs.

    The flute form has a continuous smooth taper from the widest body point
    up to the neck - no shoulder break.
    """
    return [
        (BASE_Z, 0.0190),    # flat-bottomed base disc
        (0.004, 0.0215),     # short heel
        (0.010, 0.0245),     # base flare
        (0.018, 0.0262),     # approaching max body
        (0.028, MAX_BODY_R), # max body radius
        (0.050, 0.0268),     # upper body, taper starts
        (0.080, 0.0246),     # gentle taper
        (0.110, 0.0222),
        (0.140, 0.0198),
        (0.170, 0.0174),
        (0.200, 0.0152),
        (0.230, 0.0132),
        (0.265, 0.0115),     # taper ends, blends to neck
        (NECK_TOP_Z, NECK_R),  # straight neck section
        (0.293, 0.0120),     # lip flare begins
        (0.296, LIP_R),      # flared rolled lip
        (LIP_TOP_Z, LIP_R),  # flat mouth top
    ]


def _flute_inner_profile() -> list[tuple[float, float]]:
    """Inner cavity profile as (z, radius) pairs.

    Follows the outer wall with a constant-ish offset, opening
    at the mouth so the bottle reads as hollow glass.
    """
    return [
        (0.012, 0.0155),    # cavity floor above solid glass base
        (0.018, 0.0232),
        (0.028, 0.0242),
        (0.050, 0.0240),    # taper starts
        (0.080, 0.0218),
        (0.110, 0.0194),
        (0.140, 0.0170),
        (0.170, 0.0148),
        (0.200, 0.0126),
        (0.230, 0.0106),
        (0.265, 0.0088),    # taper ends
        (NECK_TOP_Z, BORE_R),
        (LIP_TOP_Z + 0.004, BORE_R),  # poke out top so mouth is open
    ]


def _bottle_solid() -> cq.Workplane:
    # Outer silhouette: rounded base -> slim body with continuous smooth taper
    # to neck (no shoulder break) -> slightly flared rolled lip.
    outer = _revolve_profile(_flute_outer_profile())

    # Hollow it: subtract an inner cavity that opens at the mouth so the bottle
    # reads as a real open glass shell (interior bore visible from the top).
    inner = _revolve_profile(_flute_inner_profile())
    return outer.cut(inner)


def _cap_mesh():
    # Crimped crown cap: a short shallow dome top with a fluted (crimped) skirt.
    # Top disk + small crowned center, then 21 crimp "teeth" around the skirt.
    cap_top_z = 0.0
    skirt_h = 0.0080
    top_h = 0.0035
    top_r = 0.0148  # cap sits over the lip
    skirt_r = 0.0146

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
        .circle(skirt_r - 0.0015)
        .workplane(offset=skirt_h + 0.0005)
        .circle(top_r - 0.0015)
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
            .rect(0.0024, 0.0024)
            .extrude(0.0050)
        )
        cap = cap.union(tab)

    return mesh_from_cadquery(cap, "crown_cap")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flute_beer_bottle")

    # Dark/amber glass: translucent (alpha < 1) so it reads as glass.
    glass = model.material("dark_glass", rgba=GLASS_RGBA)
    cap_metal = model.material("cap_metal", rgba=CAP_RGBA)

    # ---- bottle (root): hollow dark-glass flute shell ----
    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_cadquery(_bottle_solid(), "bottle_glass"),
        material=glass,
        name="bottle_glass",
    )
    # Inertial from an equivalent solid cylinder spanning the body.
    bottle.inertial = Inertial.from_geometry(
        Cylinder(radius=MAX_BODY_R, length=LIP_TOP_Z),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, LIP_TOP_Z / 2.0)),
    )

    # ---- crown cap: crimped metal, pries straight up off the mouth ----
    cap = model.part("crown_cap")
    cap.visual(_cap_mesh(), material=cap_metal, name="crown_cap")
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0148, length=0.012),
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

    # --- Flute silhouette: very tall and very slender with continuous taper. ---
    body_ext = _ext(ctx.part_world_aabb(bottle))
    # Must be tall (>= 0.28m) and very slender (height > 4x width)
    ctx.check(
        "flute bottle is very tall and slender (continuous taper silhouette)",
        body_ext[2] > 0.28 and body_ext[2] > 4.0 * max(body_ext[0], body_ext[1]),
        details=f"bottle extents={body_ext}",
    )

    # --- Slim body: max diameter well under the parent long-neck size. ---
    ctx.check(
        "flute body is slim (max width < 0.060 m)",
        max(body_ext[0], body_ext[1]) < 0.060,
        details=f"max lateral extent={max(body_ext[0], body_ext[1]):.4f}",
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
        cap_pos is not None and cap_pos[2] > 0.27,
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
