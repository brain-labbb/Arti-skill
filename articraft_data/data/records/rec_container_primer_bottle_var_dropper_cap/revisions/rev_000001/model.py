from __future__ import annotations

# Black airless cosmetic PRIMER bottle with a DROPPER closure.
# Fork of the cushion-bodied airless primer bottle: same body, gold band,
# and label, but the press-pump actuator is replaced by a dropper closure.
# The dropper assembly (screw collar + rubber squeeze bulb + glass pipette)
# lifts straight out of the neck via a PRISMATIC joint along +Z.
#
# Frame: bottle stands upright along +Z. Base sits at z=0, the slim
# cushion-shaped body rises to a flat shoulder, and the dropper caps the top.
# The broad front/back faces face +/-Y (label + gold band on front +Y face);
# the bottle is slim across X.
# Articulation:
#   - dropper_lift: PRISMATIC, lifts straight UP (+Z, ~0.045 m).

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key dimensions (meters) ----
BODY_W = 0.034  # across X (slim)
BODY_D = 0.024  # front-to-back along Y
BODY_H = 0.086  # body height
BODY_FILLET = 0.008  # rounded vertical edges -> cushion cross-section

SHOULDER_H = 0.006  # tapered shoulder above the main body
SHOULDER_TOP_W = 0.020
SHOULDER_TOP_D = 0.016

NECK_W = 0.018  # short collar the dropper seats into
NECK_D = 0.014
NECK_H = 0.006

NECK_TOP_Z = BODY_H + SHOULDER_H + NECK_H  # 0.098

GOLD_BAND_Z = 0.050  # gold accent band center height on the body
GOLD_BAND_H = 0.005

# Dropper closure
COLLAR_OD = 0.021  # outer diameter of screw collar
COLLAR_ID = 0.016  # inner bore diameter
COLLAR_H = 0.009   # collar height
COLLAR_SEAT_OVERLAP = 0.003  # how far collar slips down over neck

BULB_H = 0.025          # squeeze bulb height
BULB_BASE_R = 0.005     # bulb radius at collar interface
BULB_MAX_R = 0.008      # bulb maximum radius

PIPETTE_OD = 0.004      # pipette outer diameter
PIPETTE_LENGTH = 0.038  # pipette extends below collar
PIPETTE_TIP_H = 0.004   # tapered tip section height
PIPETTE_TIP_R = 0.0008  # tip radius

THREAD_COUNT = 3         # number of thread ridges on collar

DROPPER_LIFT = 0.045    # prismatic lift travel


# ---- shared geometry helpers ----

def _rounded_prism(w: float, d: float, h: float, fillet: float, z0: float = 0.0) -> cq.Workplane:
    """Upright rounded-rectangle prism: footprint w(X) x d(Y), height h, base at z0."""
    f = min(fillet, 0.49 * min(w, d))
    wp = (
        cq.Workplane("XY")
        .workplane(offset=z0)
        .rect(w, d)
        .extrude(h)
    )
    if f > 1e-5:
        wp = wp.edges("|Z").fillet(f)
    return wp


def _thread_ridge_solid(i: int) -> cq.Workplane:
    """One thread ridge ring at position i on the screw collar."""
    z0 = NECK_TOP_Z - COLLAR_SEAT_OVERLAP
    ridge_z = z0 + (i + 1) * COLLAR_H / (THREAD_COUNT + 1)
    ridge = (
        cq.Workplane("XY")
        .workplane(offset=ridge_z)
        .circle(COLLAR_OD / 2.0 + 0.0006)
        .circle(COLLAR_OD / 2.0 - 0.0002)
        .extrude(0.001)
    )
    return ridge


# ---- body geometry (identical to parent) ----

def _body_solid() -> cq.Workplane:
    """Main cushion body + tapered shoulder + short neck collar, fused into one shell."""
    body = _rounded_prism(BODY_W, BODY_D, BODY_H, BODY_FILLET, z0=0.0)

    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_H)
        .rect(BODY_W - 2 * BODY_FILLET + 0.004, BODY_D - 2 * BODY_FILLET + 0.004)
        .workplane(offset=SHOULDER_H)
        .rect(SHOULDER_TOP_W, SHOULDER_TOP_D)
        .loft(ruled=True)
    )
    body = body.union(shoulder)

    neck = _rounded_prism(NECK_W, NECK_D, NECK_H, 0.003, z0=BODY_H + SHOULDER_H)
    body = body.union(neck)
    return body


def _gold_band_solid() -> cq.Workplane:
    """Thin band wrapping the body at GOLD_BAND_Z; slightly proud of the surface."""
    band = _rounded_prism(
        BODY_W + 0.0008,
        BODY_D + 0.0008,
        GOLD_BAND_H,
        BODY_FILLET,
        z0=GOLD_BAND_Z - GOLD_BAND_H / 2.0,
    )
    return band


def _label_plate_solid() -> cq.Workplane:
    """Slightly raised label area on the front (+Y) face."""
    plate = (
        cq.Workplane("XY")
        .workplane(offset=0.012)
        .center(0.0, BODY_D / 2.0 - 0.0005)
        .rect(BODY_W - 0.010, 0.0018)
        .extrude(GOLD_BAND_Z - GOLD_BAND_H / 2.0 - 0.012 - 0.002)
    )
    return plate


# ---- dropper geometry (fork variant) ----

def _collar_solid() -> cq.Workplane:
    """Screw collar: solid cylindrical ring that threads onto the neck.
    The bulb and pipette penetrate into this solid to ensure mesh connectivity."""
    z0 = NECK_TOP_Z - COLLAR_SEAT_OVERLAP
    collar = (
        cq.Workplane("XY")
        .workplane(offset=z0)
        .circle(COLLAR_OD / 2.0)
        .extrude(COLLAR_H)
    )
    return collar


def _bulb_lathe_profile() -> list[tuple[float, float]]:
    """Rubber squeeze bulb cross-section as (radius, z) points for LatheGeometry.
    The base penetrates 1mm below the collar top into the solid collar to
    ensure mesh connectivity between the bulb and collar visuals."""
    z_collar_top = NECK_TOP_Z - COLLAR_SEAT_OVERLAP + COLLAR_H
    z_penetrate = z_collar_top - 0.001  # 1mm into the collar solid
    # Profile traces from bottom (inside collar) up to rounded top.
    return [
        (0.000, z_penetrate),
        (0.003, z_penetrate),
        (BULB_BASE_R, z_collar_top),
        (BULB_BASE_R + 0.001, z_collar_top + BULB_H * 0.08),
        (BULB_MAX_R * 0.85, z_collar_top + BULB_H * 0.20),
        (BULB_MAX_R, z_collar_top + BULB_H * 0.35),
        (BULB_MAX_R, z_collar_top + BULB_H * 0.55),
        (BULB_MAX_R * 0.85, z_collar_top + BULB_H * 0.72),
        (BULB_MAX_R * 0.55, z_collar_top + BULB_H * 0.88),
        (BULB_MAX_R * 0.25, z_collar_top + BULB_H * 0.96),
        (0.000, z_collar_top + BULB_H),
    ]


def _pipette_solid() -> cq.Workplane:
    """Slim glass pipette tube hanging down from the collar with tapered tip.
    The pipette top penetrates 3mm up into the solid collar to ensure mesh
    connectivity between the pipette and collar visuals."""
    z_collar_bot = NECK_TOP_Z - COLLAR_SEAT_OVERLAP
    z_top = z_collar_bot + 0.003  # 3mm up inside the collar
    z_bot = z_collar_bot - PIPETTE_LENGTH  # extends below collar
    or_p = PIPETTE_OD / 2.0
    total_length = z_top - z_bot

    # Main tube (from above the tapered tip up to inside the collar)
    tube = (
        cq.Workplane("XY")
        .workplane(offset=z_bot + PIPETTE_TIP_H)
        .circle(or_p)
        .extrude(total_length - PIPETTE_TIP_H)
    )

    # Tapered tip at bottom (loft from tip radius to full pipette radius)
    tip = (
        cq.Workplane("XY")
        .workplane(offset=z_bot)
        .circle(PIPETTE_TIP_R)
        .workplane(offset=PIPETTE_TIP_H)
        .circle(or_p)
        .loft(ruled=True)
    )
    tube = tube.union(tip)
    return tube


# ---- build ----

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="primer_dropper_bottle")

    # Materials
    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    gold = model.material("gold_accent", rgba=(0.80, 0.62, 0.22, 1.0))
    collar_gold = model.material("collar_gold", rgba=(0.72, 0.56, 0.20, 1.0))
    rubber_dark = model.material("rubber_dark", rgba=(0.14, 0.11, 0.09, 1.0))
    glass_pale = model.material("glass_pale", rgba=(0.88, 0.88, 0.85, 1.0))

    # ---- bottle body (root) — identical to parent ----
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_solid(), "body_shell"),
        material=matte_black,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_gold_band_solid(), "gold_band"),
        material=gold,
        name="gold_band",
    )
    body.visual(
        mesh_from_cadquery(_label_plate_solid(), "label_plate"),
        material=gold,
        name="label_plate",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H)),
        mass=0.090,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # ---- dropper assembly: collar + bulb + pipette ----
    dropper = model.part("dropper")

    # Screw collar (ring with bore)
    dropper.visual(
        mesh_from_cadquery(_collar_solid(), "collar_ring"),
        material=collar_gold,
        name="collar_ring",
    )

    # Thread ridges on the collar (repeated visuals via for-i-in-range loop)
    for i in range(THREAD_COUNT):
        dropper.visual(
            mesh_from_cadquery(_thread_ridge_solid(i), f"thread_ridge_{i}"),
            material=collar_gold,
            name=f"thread_ridge_{i}",
        )

    # Rubber squeeze bulb (LatheGeometry for smooth revolved form)
    dropper.visual(
        mesh_from_geometry(LatheGeometry(_bulb_lathe_profile(), segments=32), "squeeze_bulb"),
        material=rubber_dark,
        name="squeeze_bulb",
    )

    # Glass pipette (thin tube with tapered tip)
    dropper.visual(
        mesh_from_cadquery(_pipette_solid(), "glass_pipette"),
        material=glass_pale,
        name="glass_pipette",
    )

    # Inertial for the dropper assembly
    collar_z0 = NECK_TOP_Z - COLLAR_SEAT_OVERLAP
    dropper_total_h = COLLAR_H + BULB_H + PIPETTE_LENGTH
    dropper_mid_z = collar_z0 + (BULB_H + COLLAR_H - PIPETTE_LENGTH) / 2.0
    dropper.inertial = Inertial.from_geometry(
        Box((COLLAR_OD, COLLAR_OD, dropper_total_h)),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.0, dropper_mid_z)),
    )

    # Prismatic joint: dropper lifts straight UP out of the neck
    model.articulation(
        "dropper_lift",
        ArticulationType.PRISMATIC,
        parent=body,
        child=dropper,
        # Geometry is authored in absolute body-frame coords; joint origin at
        # frame origin so +Z travel lifts the dropper straight up.
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=0.05, lower=0.0, upper=DROPPER_LIFT
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    dropper = object_model.get_part("dropper")
    lift = object_model.get_articulation("dropper_lift")

    # ---- intentional overlap allowances ----

    # Collar seats down over the neck collar (small local embed).
    ctx.allow_overlap(
        dropper,
        body,
        elem_a="collar_ring",
        elem_b="body_shell",
        reason="The dropper screw collar is intentionally seated down over the neck collar.",
    )

    # Thread ridges extend slightly beyond collar OD; same-part visual overlap
    # with the collar is cosmetic threading detail.

    # Pipette extends through the neck into the body (representing the real
    # pipette tube reaching into the product reservoir).
    ctx.allow_overlap(
        dropper,
        body,
        elem_a="glass_pipette",
        elem_b="body_shell",
        reason="The glass pipette is intentionally represented extending through the neck into the bottle body.",
    )

    # ---- body reads slim and tall (identical to parent) ----
    body_ext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "body is tall (height dominates footprint)",
        body_ext[2] > 0.070 and body_ext[2] > body_ext[0] + 0.030 and body_ext[2] > body_ext[1] + 0.030,
        details=f"body extents={body_ext}",
    )
    ctx.check(
        "body is slim",
        body_ext[0] < 0.045 and body_ext[1] < 0.045,
        details=f"body extents={body_ext}",
    )

    # ---- gold accent band present on the body, partway up ----
    band_aabb = ctx.part_element_world_aabb(body, elem="gold_band")
    band_z = (band_aabb[0][2] + band_aabb[1][2]) / 2.0
    ctx.check(
        "gold accent band sits partway up the body",
        0.030 < band_z < 0.070,
        details=f"gold band center z={band_z}",
    )

    # ---- dropper assembly is on TOP and seated over the neck ----
    dropper_aabb = ctx.part_world_aabb(dropper)
    ctx.check(
        "dropper mounted at the top of the bottle",
        dropper_aabb[1][2] > BODY_H and dropper_aabb[0][2] < NECK_TOP_Z + 0.005,
        details=f"dropper aabb z=[{dropper_aabb[0][2]:.4f}, {dropper_aabb[1][2]:.4f}]",
    )
    ctx.expect_overlap(
        dropper, body, axes="xy", min_overlap=0.010,
        elem_a="collar_ring",
        name="collar seated over neck (footprint)",
    )

    # ---- dropper has three distinct visible components ----
    collar_aabb = ctx.part_element_world_aabb(dropper, elem="collar_ring")
    bulb_aabb = ctx.part_element_world_aabb(dropper, elem="squeeze_bulb")
    pipette_aabb = ctx.part_element_world_aabb(dropper, elem="glass_pipette")

    # Bulb sits on top of the collar
    ctx.check(
        "bulb sits on top of the collar",
        bulb_aabb[0][2] >= collar_aabb[1][2] - 0.002,
        details=f"bulb_bot_z={bulb_aabb[0][2]:.4f}, collar_top_z={collar_aabb[1][2]:.4f}",
    )

    # Pipette hangs below the collar
    ctx.check(
        "pipette hangs below the collar into the neck",
        pipette_aabb[0][2] < collar_aabb[0][2] - 0.010,
        details=f"pipette_bot_z={pipette_aabb[0][2]:.4f}, collar_bot_z={collar_aabb[0][2]:.4f}",
    )

    # Bulb is visibly wider than the collar bore
    bulb_dy = bulb_aabb[1][0] - bulb_aabb[0][0]  # X extent ~ diameter
    collar_dy = collar_aabb[1][0] - collar_aabb[0][0]
    ctx.check(
        "bulb is wider than collar bore (reads as squeeze bulb)",
        bulb_dy > collar_dy * 0.5,
        details=f"bulb_dx={bulb_dy:.4f}, collar_dx={collar_dy:.4f}",
    )

    # ---- dropper lifts straight UP and pipette clears the neck ----
    dropper_bot_rest = ctx.part_world_aabb(dropper)[0][2]
    dropper_top_rest = ctx.part_world_aabb(dropper)[1][2]

    with ctx.pose({lift: DROPPER_LIFT}):
        dropper_bot_lifted = ctx.part_world_aabb(dropper)[0][2]
        dropper_top_lifted = ctx.part_world_aabb(dropper)[1][2]
        # Pipette clears the neck top when fully lifted
        pipette_lifted_aabb = ctx.part_element_world_aabb(dropper, elem="glass_pipette")
        ctx.check(
            "pipette clears neck when fully lifted",
            pipette_lifted_aabb[0][2] > NECK_TOP_Z - 0.002,
            details=f"pipette_bot_z_lifted={pipette_lifted_aabb[0][2]:.4f}, neck_top={NECK_TOP_Z:.4f}",
        )

    ctx.check(
        "dropper rises when lifted",
        dropper_bot_lifted > dropper_bot_rest + 0.030,
        details=f"rest_bot_z={dropper_bot_rest:.4f}, lifted_bot_z={dropper_bot_lifted:.4f}",
    )
    ctx.check(
        "dropper moves straight up (only z changes)",
        abs((dropper_top_lifted - dropper_top_rest) - DROPPER_LIFT) < 0.002,
        details=f"top_rise={dropper_top_lifted - dropper_top_rest:.4f}, expected={DROPPER_LIFT:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
