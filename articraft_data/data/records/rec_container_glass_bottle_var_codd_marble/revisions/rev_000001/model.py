from __future__ import annotations

# Codd-neck marble-stopper beer bottle.
# Fork of the dark-glass long-neck bottle: replaces the crimped crown cap with
# a captive glass marble stopper that seals against a rubber ring at the lip.
# Frame: bottle axis along +Z (base at z=0, mouth/lip at the top), centered on
#   the z-axis. Scale ~0.24 m tall, ~0.065 m body diameter.
# Articulations:
#   - marble stopper: PRISMATIC along -Z; positive q pushes the marble down
#     into the neck chamber to open, q=0 seats it against the lip ring (closed).

import math

import cadquery as cq

GLASS_RGBA = (0.18, 0.12, 0.07, 0.4)
MARBLE_RGBA = (0.82, 0.88, 0.86, 0.55)   # translucent clear glass marble
RUBBER_RGBA = (0.10, 0.08, 0.06, 1.0)     # dark rubber seal ring

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    SphereGeometry,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- Bottle profile heights (z, in metres) ----------------------------------
BASE_Z = 0.0
BODY_TOP_Z = 0.120          # body cylinder ends, shoulder begins
SHOULDER_TOP_Z = 0.150      # shoulder blends into the neck
NECK_TOP_Z = 0.222          # top of the slim neck, where the lip starts
LIP_TOP_Z = 0.230           # top face of the mouth / lip

BODY_R = 0.0325             # body radius (~0.065 m diameter)
NECK_R = 0.0115             # slim long-neck radius
LIP_R = 0.0135              # rolled lip is slightly fatter than the neck

WALL = 0.0028               # glass wall thickness (hollow shell)
BORE_R = 0.0085             # inner bore radius at the mouth

# Codd-neck marble stopper dimensions
MARBLE_R = 0.0075           # 15mm diameter captive glass marble
CHAMBER_R = 0.0092          # chamber bore radius (marble fits with clearance)
PINCH_R = 0.0058            # pinch ring inner radius (smaller than marble)
PINCH_Z = 0.172             # height of the pinch ring constriction
CHAMBER_BOTTOM_Z = 0.178    # where the chamber bore opens up above the pinch
CHAMBER_TOP_Z = 0.212       # where the chamber bore narrows back


def _profile_loft(sections, ruled: bool = True) -> cq.Workplane:
    """Loft circular sections stacked along +Z into a solid of revolution."""
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
    """Hollow dark-glass bottle shell with Codd-neck chamber and pinch ring."""
    # Outer silhouette: base -> body -> shoulder -> neck with visible chamber
    # bulge and pinch ridge -> flared lip.
    outer = _profile_loft(
        [
            (BASE_Z, 0.0235),          # flat-bottomed base disc
            (0.003, 0.0250),           # short straight heel
            (0.008, 0.0305),           # rounded base flare
            (0.013, 0.0322),
            (0.018, 0.0325),           # body full radius
            (BODY_TOP_Z, 0.0325),      # straight cylindrical body
            (0.128, 0.0316),           # shoulder begins
            (0.137, 0.0288),
            (SHOULDER_TOP_Z, 0.0205),  # sloped shoulder
            (0.160, 0.0150),
            (0.168, 0.0130),           # neck narrows above shoulder
            # Visible pinch-ring ridge on outer surface
            (0.170, 0.0140),           # slight outward bulge
            (0.174, 0.0130),           # back to neck
            # Marble chamber – slightly wider outer profile
            (0.180, 0.0152),           # chamber outer wall
            (0.206, 0.0152),           # chamber continues
            (0.212, 0.0130),           # upper neck narrows
            (NECK_TOP_Z, NECK_R),      # slim neck to lip
            (0.224, 0.0128),           # neck flares to the lip
            (0.227, LIP_R),            # flared lip
            (LIP_TOP_Z, LIP_R),        # flat mouth top
        ]
    )

    # Hollow cavity: same body bore but neck has pinch constriction and
    # wider marble chamber.
    inner = _profile_loft(
        [
            (0.010, 0.0200),           # cavity floor above solid glass base
            (0.018, 0.0270),
            (0.024, 0.0297),
            (BODY_TOP_Z - 0.004, 0.0297),
            (0.128, 0.0288),
            (0.137, 0.0260),
            (SHOULDER_TOP_Z, 0.0177),
            (0.160, 0.0122),
            (0.168, 0.0095),           # neck bore
            # Pinch constriction (retains marble from falling into bottle)
            (PINCH_Z, PINCH_R),
            (0.175, PINCH_R),
            # Marble chamber (wider bore for marble travel)
            (CHAMBER_BOTTOM_Z, CHAMBER_R),
            (CHAMBER_TOP_Z, CHAMBER_R),
            # Upper bore narrows back to standard
            (0.215, BORE_R),
            (NECK_TOP_Z, BORE_R),
            (LIP_TOP_Z + 0.004, BORE_R),  # poke out the top so mouth is open
        ]
    )
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="beer_bottle")

    glass = model.material("dark_glass", rgba=GLASS_RGBA)
    marble_glass = model.material("marble_glass", rgba=MARBLE_RGBA)
    rubber = model.material("rubber_seal", rgba=RUBBER_RGBA)

    # ---- bottle (root): hollow dark-glass shell with Codd-neck ----
    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_cadquery(_bottle_solid(), "bottle_glass"),
        material=glass,
        name="bottle_glass",
    )

    # Rubber sealing ring at the lip (non-moving inline decoration).
    # Sits just inside the lip bore where the marble contacts it to seal.
    rubber_ring = TorusGeometry(
        radius=BORE_R - 0.0012,   # torus major radius inside the bore
        tube=0.0014,              # tube cross-section radius
        radial_segments=16,
        tubular_segments=32,
    )
    bottle.visual(
        mesh_from_geometry(rubber_ring, "rubber_ring"),
        material=rubber,
        origin=Origin(xyz=(0.0, 0.0, LIP_TOP_Z - 0.0015)),
        name="rubber_ring",
    )

    bottle.inertial = Inertial.from_geometry(
        Cylinder(radius=BODY_R, length=LIP_TOP_Z),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, LIP_TOP_Z / 2.0)),
    )

    # ---- marble: captive glass stopper inside the Codd-neck ----
    marble = model.part("marble")
    marble.visual(
        Sphere(radius=MARBLE_R),
        material=marble_glass,
        name="marble",
    )
    marble.inertial = Inertial.from_geometry(
        Sphere(radius=MARBLE_R),
        mass=0.005,
    )

    # Marble part frame sits at the sealed position (center of marble when it
    # is held up by gas pressure against the rubber ring at the lip). At q=0
    # the marble seals the bottle (closed). Positive q pushes the marble
    # straight down into the neck chamber to open the bottle.
    # The marble is intentionally slightly smaller than the bore so it can
    # travel freely; retention comes from the pinch ring below the chamber.
    marble_seat_z = LIP_TOP_Z - MARBLE_R
    model.articulation(
        "bottle_to_marble",
        ArticulationType.PRISMATIC,
        parent=bottle,
        child=marble,
        origin=Origin(xyz=(0.0, 0.0, marble_seat_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=0.5, lower=0.0, upper=0.025,
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bottle = object_model.get_part("bottle")
    marble = object_model.get_part("marble")
    marble_joint = object_model.get_articulation("bottle_to_marble")

    # --- Dark glass: translucent body material (alpha < 1). ---
    ctx.check(
        "bottle glass is translucent (alpha < 1)",
        GLASS_RGBA[3] < 1.0,
        details=f"glass rgba={GLASS_RGBA}",
    )

    # --- Long neck bottle silhouette preserved from parent. ---
    body_ext = _ext(ctx.part_world_aabb(bottle))
    ctx.check(
        "bottle is tall (long-neck silhouette)",
        body_ext[2] > 0.20 and body_ext[2] > 2.5 * max(body_ext[0], body_ext[1]),
        details=f"bottle extents={body_ext}",
    )

    # --- Marble is captive inside the bottle neck (within bottle XY footprint). ---
    # The marble is intentionally smaller than the bore so it can travel freely
    # in the Codd-neck chamber. It is retained by the pinch ring below.
    ctx.allow_isolated_part(
        marble,
        reason="Marble is captive inside the Codd-neck chamber, retained by the pinch ring constriction below.",
    )
    ctx.allow_overlap(
        marble,
        bottle,
        elem_a="marble",
        elem_b="bottle_glass",
        reason="Marble is intentionally captive inside the Codd-neck chamber as a stopper.",
    )
    ctx.expect_within(
        marble, bottle, axes="xy", margin=0.003,
        name="marble stays centered in the bottle neck (XY)",
    )

    # --- Marble seated at the mouth when closed (q=0). ---
    marble_rest = ctx.part_world_position(marble)
    ctx.check(
        "marble seated at bottle mouth when closed",
        marble_rest is not None and marble_rest[2] > 0.20,
        details=f"marble rest position={marble_rest}",
    )

    # --- Marble pushes straight down into the chamber to open. ---
    rest_z = marble_rest[2]
    with ctx.pose({marble_joint: 0.025}):
        open_z = ctx.part_world_position(marble)[2]
        ctx.check(
            "marble moves down into chamber to open",
            open_z < rest_z - 0.020,
            details=f"rest_z={rest_z}, open_z={open_z}",
        )
        # Marble still within bottle neck when pushed down
        ctx.expect_within(
            marble, bottle, axes="xy", margin=0.003,
            name="marble stays centered in neck when pushed down",
        )
        # Marble still overlaps bottle in Z (still inside the neck, not escaped)
        ctx.expect_overlap(
            marble, bottle, axes="z", min_overlap=0.005,
            name="marble remains inside bottle neck when open",
        )

    # --- Rubber sealing ring visible at the lip. ---
    ring = bottle.get_visual("rubber_ring")
    ctx.check(
        "rubber sealing ring present at the lip",
        ring is not None,
        details="rubber_ring visual missing on bottle",
    )

    # --- Joint is prismatic (single-axis linear closure). ---
    ctx.check(
        "marble joint is prismatic",
        marble_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"joint type={marble_joint.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
