from __future__ import annotations

# Dark glass beer bottle with a tapered cork stopper.
# Frame: bottle axis along +Z (base at z=0, mouth/lip at the top), centered on
#   the z-axis. Scale ~0.24 m tall, ~0.065 m body diameter.
# Articulations:
#   - cork stopper: PRISMATIC, pulls straight up (+Z) out of the bottle mouth.
#     Tapered plug seated in the bore when down, grip head above the lip.

import math

import cadquery as cq

GLASS_RGBA = (0.18, 0.12, 0.07, 0.4)
CORK_RGBA = (0.76, 0.60, 0.42, 1.0)
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


def _cork_mesh():
    # Tapered cork stopper: a cylindrical/slightly-tapered plug that seats in
    # the bottle bore, with a wider grip head protruding above the mouth.
    # The part frame sits at the lip top; cork extends DOWN into the bore.
    plug_length = 0.026  # total plug length inside the bore
    plug_top_r = 0.0090  # top of plug (interference fit, slightly > bore)
    plug_bot_r = 0.0078  # bottom of plug (tapered for insertion)
    head_h = 0.006  # grip head height above lip
    head_r = 0.0120  # grip head radius (wider for finger grip)
    chamfer = 0.0012  # small chamfer at bottom tip

    # Main tapered plug body (extends below the part frame into the bore).
    plug = (
        cq.Workplane("XY")
        .workplane(offset=-plug_length)
        .circle(plug_bot_r)
        .workplane(offset=plug_length)
        .circle(plug_top_r)
        .loft(ruled=True)
    )

    # Rounded bottom tip: a small sphere blended onto the plug bottom.
    tip = (
        cq.Workplane("XY")
        .workplane(offset=-plug_length)
        .sphere(plug_bot_r * 0.7)
    )
    plug = plug.union(tip)

    # Grip head: wider cylinder above the lip for finger grip, with a small
    # fillet/chamfer at the top edge for realism.
    head = (
        cq.Workplane("XY")
        .circle(head_r)
        .workplane(offset=head_h - chamfer)
        .circle(head_r)
        .workplane(offset=chamfer)
        .circle(head_r - chamfer)
        .loft(ruled=True)
    )
    cork = plug.union(head)

    # Surface detail: a few shallow circumferential grooves to suggest the
    # natural cork grain / compression rings.
    n_grooves = 3
    groove_depth = 0.0006
    for i in range(n_grooves):
        gz = -plug_length * (0.25 + 0.25 * i)
        # interpolate radius at this height
        t = (gz + plug_length) / plug_length
        gr = plug_bot_r + t * (plug_top_r - plug_bot_r)
        groove = (
            cq.Workplane("XY")
            .workplane(offset=gz - 0.0005)
            .circle(gr + 0.0002)
            .workplane(offset=0.001)
            .circle(gr + 0.0002)
            .loft(ruled=True)
        )
        inner = (
            cq.Workplane("XY")
            .workplane(offset=gz - 0.0006)
            .circle(gr - groove_depth)
            .workplane(offset=0.0012)
            .circle(gr - groove_depth)
            .loft(ruled=True)
        )
        groove_ring = groove.cut(inner)
        cork = cork.cut(groove_ring)

    return mesh_from_cadquery(cork, "cork_stopper")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="beer_bottle")

    # Dark/brown glass: translucent (alpha < 1) so it reads as glass.
    glass = model.material("dark_glass", rgba=GLASS_RGBA)
    cork_mat = model.material("cork", rgba=CORK_RGBA)

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

    # ---- cork stopper: tapered plug seated in the bore, pulls straight up ----
    cork = model.part("cork_stopper")
    cork.visual(_cork_mesh(), material=cork_mat, name="cork_stopper")
    cork.inertial = Inertial.from_geometry(
        Cylinder(radius=0.010, length=0.032),
        mass=0.006,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
    )

    # The cork part frame sits at the lip top surface. The plug extends
    # downward into the bore; the grip head extends upward above the lip.
    # Positive q pulls the cork straight up out of the mouth (prismatic).
    model.articulation(
        "bottle_to_cork",
        ArticulationType.PRISMATIC,
        parent=bottle,
        child=cork,
        origin=Origin(xyz=(0.0, 0.0, LIP_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.15, lower=0.0, upper=0.05),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bottle = object_model.get_part("bottle")
    cork = object_model.get_part("cork_stopper")
    cork_joint = object_model.get_articulation("bottle_to_cork")

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

    # --- Cork seated in the bore when down: plug overlaps the bottle on z-axis. ---
    ctx.allow_overlap(
        cork,
        bottle,
        elem_a="cork_stopper",
        elem_b="bottle_glass",
        reason="Cork plug is intentionally seated inside the bottle bore when at rest.",
    )
    ctx.expect_overlap(
        cork, bottle, axes="z", min_overlap=0.010, name="cork plug seated in the bottle bore"
    )

    # Cork is positioned at the top (mouth), not floating elsewhere.
    cork_pos = ctx.part_world_position(cork)
    ctx.check(
        "cork mounted at the bottle mouth (top)",
        cork_pos is not None and cork_pos[2] > 0.20,
        details=f"cork origin={cork_pos}",
    )

    # --- Cork pulls straight up out of the mouth (prismatic). ---
    rest_z = ctx.part_world_position(cork)[2]
    with ctx.pose({cork_joint: 0.05}):
        lifted_z = ctx.part_world_position(cork)[2]
        # When pulled, the cork clears the lip: gap between cork and bottle.
        ctx.expect_gap(cork, bottle, axis="z", max_gap=0.08, min_gap=0.002)
    ctx.check(
        "cork pulls straight up out of the mouth",
        lifted_z > rest_z + 0.045,
        details=f"rest_z={rest_z}, lifted_z={lifted_z}",
    )

    return ctx.report()


object_model = build_object_model()
