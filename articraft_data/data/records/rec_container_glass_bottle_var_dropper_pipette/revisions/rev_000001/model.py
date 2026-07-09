from __future__ import annotations

# Dark glass beer bottle with a dropper/pipette cap (family fork of crown-cap variant).
# Frame: bottle axis along +Z (base at z=0, mouth/lip at the top), centered on
#   the z-axis. Scale ~0.24 m tall, ~0.065 m body diameter.
# Articulations:
#   - dropper_cap: PRISMATIC, lifts straight up (+Z) out of the bottle mouth,
#     carrying the pipette tube and squeeze bulb with it.

import math

import cadquery as cq

GLASS_RGBA = (0.18, 0.12, 0.07, 0.4)
CAP_RGBA = (0.12, 0.12, 0.12, 1.0)       # dark plastic cap collar
PIPETTE_RGBA = (0.82, 0.85, 0.88, 0.55)  # translucent glass pipette
BULB_RGBA = (0.08, 0.06, 0.06, 1.0)      # dark rubber squeeze bulb

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
BODY_TOP_Z = 0.120   # body cylinder ends, shoulder begins
SHOULDER_TOP_Z = 0.150  # shoulder blends into the neck
NECK_TOP_Z = 0.222   # top of the slim neck, where the lip starts
LIP_TOP_Z = 0.230    # top face of the mouth / lip

BODY_R = 0.0325      # body radius (~0.065 m diameter)
NECK_R = 0.0115      # slim long-neck radius
LIP_R = 0.0135       # rolled lip is slightly fatter than the neck

WALL = 0.0028        # glass wall thickness (hollow shell)
BORE_R = 0.0085      # inner bore radius at the mouth


def _profile_loft(sections, ruled: bool = True) -> cq.Workplane:
    # sections: list of (z, radius). Lofts circular sections stacked along +Z.
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
            (BASE_Z, 0.0235),     # flat-bottomed base disc
            (0.003, 0.0250),      # short straight heel
            (0.008, 0.0305),      # rounded base flare
            (0.013, 0.0322),
            (0.018, 0.0325),      # body full radius
            (BODY_TOP_Z, 0.0325), # straight cylindrical body
            (0.128, 0.0316),      # shoulder begins
            (0.137, 0.0288),
            (SHOULDER_TOP_Z, 0.0205),  # sloped shoulder
            (0.160, 0.0150),
            (0.170, 0.0122),      # shoulder rolls into neck
            (NECK_TOP_Z, NECK_R), # long slim neck
            (0.224, 0.0128),      # neck flares to the lip
            (0.227, LIP_R),       # flared lip
            (LIP_TOP_Z, LIP_R),   # flat mouth top
        ]
    )

    # Hollow it: subtract an inner cavity that opens at the mouth so the bottle
    # reads as a real open glass shell (interior bore visible from the top).
    inner = _profile_loft(
        [
            (0.010, 0.0200),      # cavity floor above a solid glass base
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


# ---- Dropper cap geometry helpers -------------------------------------------

# Cap collar dimensions (in dropper_cap part frame, origin at lip top)
CAP_RING_OD = 0.030   # outer diameter of the screw-on collar
CAP_RING_ID = 0.024   # inner bore (clearance over the lip)
CAP_RING_H = 0.014    # collar height

# Pipette tube dimensions
PIPETTE_OD = 0.0040   # outer diameter of glass tube
PIPETTE_ID = 0.0028   # inner bore of tube
PIPETTE_LENGTH = 0.105  # extends down into the bottle body

# Squeeze bulb dimensions
BULB_RX = 0.0095      # horizontal radius
BULB_RZ = 0.012       # vertical radius (slightly taller than wide)


def _cap_collar() -> cq.Workplane:
    """Screw-on cap collar: a hollow cylinder with thread ridges inside."""
    # Main collar body
    outer = (
        cq.Workplane("XY")
        .circle(CAP_RING_OD / 2.0)
        .extrude(CAP_RING_H)
    )
    # Hollow bore so it fits over the lip
    bore = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .circle(CAP_RING_ID / 2.0)
        .extrude(CAP_RING_H + 0.002)
    )
    collar = outer.cut(bore)

    # Add thread ridges: 3 helical ribs on the inner wall (simplified as
    # small protruding tabs around the inside)
    n_threads = 3
    n_tabs_per_thread = 6
    for t in range(n_threads):
        phase = 2.0 * math.pi * t / n_threads
        for i in range(n_tabs_per_thread):
            frac = i / n_tabs_per_thread
            ang = phase + frac * (2.0 * math.pi / n_threads)
            z_pos = frac * CAP_RING_H
            r_mid = CAP_RING_ID / 2.0 + 0.0006
            tx = r_mid * math.cos(ang)
            ty = r_mid * math.sin(ang)
            tab = (
                cq.Workplane("XY")
                .workplane(offset=z_pos)
                .center(tx, ty)
                .rect(0.0020, 0.0016)
                .extrude(0.0018)
            )
            collar = collar.union(tab)

    # Chamfer the top edge with a small ring
    top_ring = (
        cq.Workplane("XY")
        .workplane(offset=CAP_RING_H - 0.001)
        .circle(CAP_RING_OD / 2.0)
        .workplane(offset=0.001)
        .circle(CAP_RING_OD / 2.0 - 0.001)
        .loft(ruled=True)
    )
    collar = collar.union(top_ring)

    return collar


def _pipette_tube():
    """Slim glass pipette tube extending downward from the cap collar.
    Built as a LatheGeometry to avoid internal disconnected mesh islands.
    Includes a mounting flange that bridges to the collar bore wall.
    """
    from sdk import LatheGeometry, mesh_from_geometry

    flange_h = 0.002
    flange_r = CAP_RING_ID / 2.0 + 0.001  # overlaps collar bore wall
    tube_r = PIPETTE_OD / 2.0
    bore_r = PIPETTE_ID / 2.0
    tip_r = 0.0008  # fine dispensing tip
    tip_len = 0.006
    tube_len = PIPETTE_LENGTH

    # Outer shell profile: (radius, z) going from top to bottom
    outer_profile = [
        (flange_r, 0.0),              # flange outer top
        (flange_r, -flange_h),        # flange outer bottom
        (tube_r, -flange_h),          # step to tube OD
        (tube_r, -tube_len),          # tube outer bottom
        (tip_r, -(tube_len + tip_len)),  # tapered tip
    ]

    # Inner bore profile: (radius, z) going from top to bottom
    inner_profile = [
        (bore_r, -flange_h),          # bore top (inside the flange)
        (bore_r, -tube_len),          # bore bottom
        (max(tip_r - 0.0002, 0.0002), -(tube_len + tip_len - 0.001)),  # bore tip
    ]

    geom = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=24,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(geom, "pipette_tube")


def _squeeze_bulb() -> cq.Workplane:
    """Rubber squeeze bulb on top of the cap: an oblate spheroid with a stem."""
    # Stem connecting bulb to cap top
    stem_h = 0.005
    stem_r = 0.004
    stem = (
        cq.Workplane("XY")
        .workplane(offset=CAP_RING_H)
        .circle(stem_r)
        .extrude(stem_h)
    )

    # Oblate bulb (ellipsoid of revolution)
    bulb_base_z = CAP_RING_H + stem_h
    n_sections = 12
    sections = []
    for i in range(n_sections + 1):
        t = i / n_sections  # 0..1
        theta = math.pi * t  # 0..pi
        z = bulb_base_z + BULB_RZ * (1.0 - math.cos(theta)) * 0.5
        r = BULB_RX * math.sin(theta)
        sections.append((z, max(r, 0.0005)))

    bulb = _profile_loft(sections, ruled=False)

    # Union stem and bulb
    result = stem.union(bulb)

    # Small neck ring where bulb meets stem (visual transition)
    neck_ring = (
        cq.Workplane("XY")
        .workplane(offset=bulb_base_z - 0.001)
        .circle(BULB_RX * 0.55)
        .workplane(offset=0.002)
        .circle(stem_r + 0.0005)
        .loft(ruled=True)
    )
    result = result.union(neck_ring)

    return result


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dropper_bottle")

    # Materials
    glass = model.material("dark_glass", rgba=GLASS_RGBA)
    cap_plastic = model.material("cap_plastic", rgba=CAP_RGBA)
    pipette_glass = model.material("pipette_glass", rgba=PIPETTE_RGBA)
    bulb_rubber = model.material("bulb_rubber", rgba=BULB_RGBA)

    # ---- bottle (root): hollow dark-glass shell (unchanged from parent) ----
    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_cadquery(_bottle_solid(), "bottle_glass"),
        material=glass,
        name="bottle_glass",
    )
    bottle.inertial = Inertial.from_geometry(
        Cylinder(radius=BODY_R, length=LIP_TOP_Z),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, LIP_TOP_Z / 2.0)),
    )

    # ---- dropper_cap: screw-on collar + pipette tube + squeeze bulb ----
    dropper = model.part("dropper_cap")

    # Cap collar (dark plastic)
    dropper.visual(
        mesh_from_cadquery(_cap_collar(), "cap_collar"),
        material=cap_plastic,
        name="cap_collar",
    )

    # Pipette tube (translucent glass, extends downward from cap)
    # _pipette_tube() returns a managed mesh via LatheGeometry
    dropper.visual(
        _pipette_tube(),
        material=pipette_glass,
        name="pipette_tube",
    )

    # Squeeze bulb (dark rubber, on top of cap)
    dropper.visual(
        mesh_from_cadquery(_squeeze_bulb(), "squeeze_bulb"),
        material=bulb_rubber,
        name="squeeze_bulb",
    )

    dropper.inertial = Inertial.from_geometry(
        Cylinder(radius=0.015, length=0.030),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.0, CAP_RING_H / 2.0)),
    )

    # Prismatic joint: cap lifts straight up off the bottle mouth
    # Origin at the top of the lip; axis +Z; positive q extracts the dropper.
    model.articulation(
        "bottle_to_dropper",
        ArticulationType.PRISMATIC,
        parent=bottle,
        child=dropper,
        origin=Origin(xyz=(0.0, 0.0, LIP_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=0.15, lower=0.0, upper=0.13
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bottle = object_model.get_part("bottle")
    dropper = object_model.get_part("dropper_cap")
    dropper_joint = object_model.get_articulation("bottle_to_dropper")

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

    # --- Dropper cap has three named visuals: collar, tube, bulb ---
    collar_vis = dropper.get_visual("cap_collar")
    tube_vis = dropper.get_visual("pipette_tube")
    bulb_vis = dropper.get_visual("squeeze_bulb")
    ctx.check(
        "dropper cap has collar, tube, and bulb visuals",
        collar_vis is not None and tube_vis is not None and bulb_vis is not None,
        details="missing one or more dropper sub-visuals",
    )

    # --- Cap seated on the lip when down: collar overlaps the lip in Z ---
    ctx.allow_overlap(
        dropper,
        bottle,
        elem_a="cap_collar",
        elem_b="bottle_glass",
        reason="Cap collar is intentionally seated over the bottle lip when closed.",
    )
    ctx.expect_overlap(
        dropper, bottle, axes="z", min_overlap=0.002,
        name="dropper cap seated over the bottle lip",
    )

    # --- Pipette tube extends into the bottle bore (intentional nesting) ---
    ctx.allow_overlap(
        dropper,
        bottle,
        elem_a="pipette_tube",
        elem_b="bottle_glass",
        reason="Pipette tube hangs down through the bottle bore into the liquid cavity.",
    )
    # The tube must overlap the bottle in Z (it extends deep inside)
    ctx.expect_overlap(
        dropper, bottle, axes="z",
        elem_a="pipette_tube", elem_b="bottle_glass",
        min_overlap=0.04,
        name="pipette tube extends well into the bottle body",
    )
    # The tube must stay within the bottle bore in XY
    ctx.expect_within(
        dropper, bottle, axes="xy",
        inner_elem="pipette_tube", outer_elem="bottle_glass",
        margin=0.005,
        name="pipette tube stays inside the bottle bore (XY)",
    )

    # --- Squeeze bulb sits above the cap collar ---
    ctx.expect_gap(
        dropper, dropper, axis="z",
        positive_elem="squeeze_bulb", negative_elem="cap_collar",
        min_gap=-0.003, max_gap=0.003,
        name="bulb is stacked on top of the collar",
    )

    # --- Cap mounted at the bottle mouth (top) ---
    cap_pos = ctx.part_world_position(dropper)
    ctx.check(
        "dropper cap mounted at the bottle mouth (top)",
        cap_pos is not None and cap_pos[2] > 0.20,
        details=f"dropper origin={cap_pos}",
    )

    # --- Dropper lifts straight up off the mouth (prismatic extraction) ---
    rest_z = ctx.part_world_position(dropper)[2]
    with ctx.pose({dropper_joint: 0.13}):
        lifted_z = ctx.part_world_position(dropper)[2]
        # When fully lifted, the pipette tube clears the bottle lip
        ctx.expect_gap(dropper, bottle, axis="z", min_gap=0.001,
                       name="fully lifted dropper clears the bottle lip")
    ctx.check(
        "dropper lifts straight up out of the mouth",
        lifted_z > rest_z + 0.12,
        details=f"rest_z={rest_z}, lifted_z={lifted_z}",
    )

    return ctx.report()


object_model = build_object_model()
