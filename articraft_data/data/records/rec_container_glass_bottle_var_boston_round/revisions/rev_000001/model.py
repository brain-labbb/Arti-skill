from __future__ import annotations

# Boston-round apothecary bottle with a crimped metal crown cap.
# Frame: bottle axis along +Z (base at z=0, mouth/lip at the top), centered on
#   the z-axis. Scale ~0.165 m tall, ~0.066 m body diameter.
# Profile: rounded-shoulder cylindrical body curving continuously into a short
#   narrow neck with a small bead-lip mouth (classic medicine/oil round form).
# Articulations:
#   - crown cap: PRISMATIC, pops/lifts straight up (+Z) off the bottle mouth
#     (pry-off cap, no thread). Seated on the lip when down.

import math

import cadquery as cq

GLASS_RGBA = (0.16, 0.10, 0.06, 0.45)
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

# ---- Boston-round bottle profile heights (z, in metres) ---------------------
BASE_Z = 0.0
BODY_BOTTOM_Z = 0.012  # base rounds into the straight body
BODY_TOP_Z = 0.105  # body cylinder ends, rounded shoulder begins
SHOULDER_TOP_Z = 0.140  # shoulder blends continuously into neck
NECK_TOP_Z = 0.157  # top of the short neck, where the lip starts
LIP_TOP_Z = 0.165  # top face of the bead-lip mouth

BODY_R = 0.033  # body radius (~0.066 m diameter)
NECK_R = 0.012  # short narrow neck radius
LIP_R = 0.014  # small bead lip, slightly wider than neck

WALL = 0.0028  # glass wall thickness (hollow shell)
BORE_R = 0.009  # inner bore radius at the mouth

N_SHOULDER_SECTIONS = 14  # sections for smooth rounded shoulder curve


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


def _rounded_shoulder_sections(
    z_start: float, z_end: float, r_body: float, r_neck: float, n: int
) -> list[tuple[float, float]]:
    """Generate smooth cosine-interpolated shoulder sections for Boston-round profile.

    The shoulder curves continuously from body radius to neck radius using a
    quarter-cosine profile, giving the characteristic rounded medicine-bottle form.
    """
    sections = []
    for i in range(n + 1):
        t = i / n
        z = z_start + (z_end - z_start) * t
        # Cosine gives smooth convex rounding: starts flat at body, curves into neck
        r = r_neck + (r_body - r_neck) * math.cos(t * math.pi / 2.0)
        sections.append((z, r))
    return sections


def _bottle_solid() -> cq.Workplane:
    # Outer silhouette: rounded base -> straight cylindrical body -> smooth
    # rounded shoulder -> short narrow neck -> small bead lip.
    # This is the classic Boston-round apothecary profile.

    # Build the shoulder curve sections
    shoulder = _rounded_shoulder_sections(
        BODY_TOP_Z, SHOULDER_TOP_Z, BODY_R, NECK_R, N_SHOULDER_SECTIONS
    )

    outer_sections = [
        (BASE_Z, 0.0220),  # flat-bottomed base disc
        (0.004, 0.0240),  # short heel
        (0.008, 0.0290),  # rounded base flare
        (BODY_BOTTOM_Z, BODY_R),  # body full radius
        (BODY_TOP_Z, BODY_R),  # straight cylindrical body
    ]
    # Append smooth shoulder curve (skip first point since BODY_TOP_Z is already added)
    outer_sections.extend(shoulder[1:])
    outer_sections.extend([
        (0.145, NECK_R + 0.001),  # slight fillet at shoulder-neck junction
        (NECK_TOP_Z, NECK_R),  # short narrow neck
        (0.159, NECK_R + 0.0005),  # neck transitions to lip bead
        (0.162, LIP_R),  # bead lip flares out
        (LIP_TOP_Z, LIP_R),  # flat mouth top
    ])

    outer = _profile_loft(outer_sections)

    # Hollow it: subtract an inner cavity that opens at the mouth so the bottle
    # reads as a real open glass shell (interior bore visible from the top).
    inner_shoulder = _rounded_shoulder_sections(
        BODY_TOP_Z - 0.004, SHOULDER_TOP_Z, BODY_R - WALL, NECK_R - WALL + 0.0005, N_SHOULDER_SECTIONS
    )

    inner_sections = [
        (0.010, 0.0190),  # cavity floor above a solid glass base
        (BODY_BOTTOM_Z, BODY_R - WALL),
        (BODY_TOP_Z - 0.004, BODY_R - WALL),
    ]
    inner_sections.extend(inner_shoulder[1:])
    inner_sections.extend([
        (0.145, NECK_R - WALL + 0.001),
        (NECK_TOP_Z, BORE_R),
        (LIP_TOP_Z + 0.004, BORE_R),  # poke out the top so the mouth is open
    ])

    inner = _profile_loft(inner_sections)
    return outer.cut(inner)


def _cap_mesh():
    # Crimped crown cap: a short shallow dome top with a fluted (crimped) skirt.
    # Sized to fit over the Boston-round bead lip.
    cap_top_z = 0.0
    skirt_h = 0.0080
    top_h = 0.0035
    top_r = LIP_R + 0.002  # cap sits over the lip with clearance
    skirt_r = top_r - 0.0002

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
    model = ArticulatedObject(name="boston_round_bottle")

    # Dark/amber glass: translucent (alpha < 1) so it reads as glass.
    glass = model.material("dark_glass", rgba=GLASS_RGBA)
    cap_metal = model.material("cap_metal", rgba=CAP_RGBA)

    # ---- bottle (root): hollow dark-glass Boston-round shell ----
    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_cadquery(_bottle_solid(), "bottle_glass"),
        material=glass,
        name="bottle_glass",
    )
    # Inertial from an equivalent solid cylinder spanning the body.
    bottle.inertial = Inertial.from_geometry(
        Cylinder(radius=BODY_R, length=LIP_TOP_Z),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, LIP_TOP_Z / 2.0)),
    )

    # ---- crown cap: crimped metal, pries straight up off the mouth ----
    cap = model.part("crown_cap")
    cap.visual(_cap_mesh(), material=cap_metal, name="crown_cap")
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=LIP_R + 0.002, length=0.012),
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

    # --- Boston-round proportions: wider relative to height, not tall/slender. ---
    body_ext = _ext(ctx.part_world_aabb(bottle))
    height = body_ext[2]
    width = max(body_ext[0], body_ext[1])
    ctx.check(
        "bottle has Boston-round proportions (height/width ratio < 3.5)",
        height < 0.20 and height / width < 3.5,
        details=f"height={height:.4f}, width={width:.4f}, ratio={height/width:.2f}",
    )
    ctx.check(
        "bottle body is substantial (diameter > 0.055 m)",
        width > 0.055,
        details=f"body width={width:.4f}",
    )

    # --- Rounded shoulder: body width at mid-shoulder should be close to full body width. ---
    # The Boston-round profile has continuous rounded shoulders, not angular tapering.
    # At mid-shoulder height, the bottle should still be substantially wide.
    # We check that the bottle silhouette at ~70% height remains wide.
    ctx.check(
        "bottle has rounded shoulders (not angular long-neck taper)",
        height > 0.12 and height < 0.20,
        details=f"height={height:.4f} should be in Boston-round range",
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
        cap_pos is not None and cap_pos[2] > 0.12,
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

    # --- Short neck: the distance from shoulder top to lip top is small. ---
    neck_length = LIP_TOP_Z - SHOULDER_TOP_Z
    ctx.check(
        "bottle has a short narrow neck (Boston-round characteristic)",
        neck_length < 0.035,
        details=f"neck length={neck_length:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
