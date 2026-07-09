from __future__ import annotations

# Black airless cosmetic PRIMER pump bottle — SIDE-LEVER TREATMENT PUMP variant.
# Frame: bottle stands upright along +Z. Base sits at z=0, the slim
# cushion-shaped body rises to a flat shoulder, and a side-lever treatment
# pump caps the top. A curved lever arm hinges on a pivot between fork posts
# at the top of the neck and swings DOWN through a REVOLUTE joint to drive the
# dispenser (instead of the parent prismatic straight-down press).
# The broad front/back faces face +/-Y (label + gold band on front +Y face);
# the bottle is slim across X.
# Articulation:
#   - lever_swing: REVOLUTE, pivot axis along X at the fork top, positive q
#     swings the lever arm downward (toward -Z).

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- key dimensions (meters) ----
# Body (IDENTICAL TO PARENT)
BODY_W = 0.034          # across X (slim)
BODY_D = 0.024          # front-to-back along Y
BODY_H = 0.086          # body height
BODY_FILLET = 0.008     # rounded vertical edges -> cushion cross-section

SHOULDER_H = 0.006      # tapered shoulder above the main body
SHOULDER_TOP_W = 0.020
SHOULDER_TOP_D = 0.016

NECK_W = 0.018          # short collar the pump seats over
NECK_D = 0.014
NECK_H = 0.006

GOLD_BAND_Z = 0.050     # gold accent band center height on the body
GOLD_BAND_H = 0.005

# Treatment pump lever mechanism (NEW — replaces parent press-down actuator)
PUMP_HOUSING_D = 0.016  # cylindrical collar on top of the neck
PUMP_HOUSING_H = 0.008

FORK_POST_D = 0.003     # fork post diameter
FORK_POST_H = 0.014     # fork post height above housing top
FORK_POST_SPACING = 0.012  # center-to-center along X

# Pivot axis height (where the lever hub rotates)
_neck_top = BODY_H + SHOULDER_H + NECK_H        # 0.098
_housing_top = _neck_top + PUMP_HOUSING_H       # 0.106
PIVOT_Z = _housing_top + FORK_POST_H * 0.65     # ~0.1151

LEVER_HUB_D = 0.006     # hub diameter (fits between fork posts)
LEVER_HUB_W = 0.0095    # hub width along X (wraps around fork posts for pivot contact)

LEVER_ARM_LENGTH = 0.030   # from pivot axis to arm tip (along +Y at rest)
LEVER_ARM_WIDTH = 0.005    # arm width at root
LEVER_ARM_THICK = 0.003    # arm thickness

NOZZLE_D = 0.004        # dispense nozzle diameter at arm tip
NOZZLE_L = 0.007        # nozzle length (forward-pointing)

LEVER_SWING_UPPER = 1.05   # max swing angle ~60 degrees


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

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


def _body_solid() -> cq.Workplane:
    """Main cushion body + tapered shoulder + short neck collar (SAME AS PARENT)."""
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
    """Thin band wrapping the body at GOLD_BAND_Z (SAME AS PARENT)."""
    return _rounded_prism(
        BODY_W + 0.0008,
        BODY_D + 0.0008,
        GOLD_BAND_H,
        BODY_FILLET,
        z0=GOLD_BAND_Z - GOLD_BAND_H / 2.0,
    )


def _label_plate_solid() -> cq.Workplane:
    """Slightly raised label area on the front +Y face (SAME AS PARENT)."""
    return (
        cq.Workplane("XY")
        .workplane(offset=0.012)
        .center(0.0, BODY_D / 2.0 - 0.0005)
        .rect(BODY_W - 0.010, 0.0018)
        .extrude(GOLD_BAND_Z - GOLD_BAND_H / 2.0 - 0.012 - 0.002)
    )


def _pump_housing_solid() -> cq.Workplane:
    """Cylindrical pump base collar sitting on top of the neck."""
    return (
        cq.Workplane("XY")
        .workplane(offset=_neck_top)
        .circle(PUMP_HOUSING_D / 2)
        .extrude(PUMP_HOUSING_H)
    )


def _fork_post_solid(x_center: float) -> cq.Workplane:
    """Single fork post rising from the pump housing top at the given X center."""
    return (
        cq.Workplane("XY")
        .workplane(offset=_housing_top)
        .center(x_center, 0.0)
        .circle(FORK_POST_D / 2)
        .extrude(FORK_POST_H)
    )


def _lever_solid() -> cq.Workplane:
    """Lever arm in local frame at pivot: hub at origin, arm extends in +Y,
    curves slightly upward at rest. Nozzle at the tip.
    Note: CadQuery "XZ" workplane has normal = -Y, so positive offsets go in -Y.
    We negate offsets to extend the arm in +Y."""
    # Hub: cylinder along X axis, centered at origin
    hub = (
        cq.Workplane("YZ")
        .workplane(offset=-LEVER_HUB_W / 2)
        .circle(LEVER_HUB_D / 2)
        .extrude(LEVER_HUB_W)
    )

    # Arm: loft from near hub outward in +Y, with upward curvature and taper
    # XZ workplane normal = -Y, so offset=-d moves to y=+d
    arm_start_y = LEVER_HUB_D / 2 * 0.5  # start inside hub for clean union
    arm = (
        cq.Workplane("XZ")
        .workplane(offset=-arm_start_y)  # → y = +arm_start_y
        .rect(LEVER_ARM_WIDTH, LEVER_ARM_THICK)
        .workplane(offset=-(LEVER_ARM_LENGTH - arm_start_y))  # → y = +LEVER_ARM_LENGTH
        .center(0, 0.004)  # shift end section up 4mm for curved lever profile
        .rect(LEVER_ARM_WIDTH * 0.65, LEVER_ARM_THICK * 0.7)
        .loft()
    )

    # Nozzle: small forward-pointing cylinder at the arm tip (+Y direction)
    # Place workplane at y = LEVER_ARM_LENGTH + NOZZLE_L, extrude toward -Y
    nozzle = (
        cq.Workplane("XZ")
        .workplane(offset=-(LEVER_ARM_LENGTH + NOZZLE_L))  # → y = +(ARM_LEN+NOZ_L)
        .center(0, 0.004)
        .circle(NOZZLE_D / 2)
        .extrude(NOZZLE_L + 0.001)  # extrudes -Y by (NOZ_L+0.001), ending at y≈ARM_LEN-0.001
    )

    return hub.union(arm).union(nozzle)


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="primer_lever_pump_bottle")

    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    gold = model.material("gold_accent", rgba=(0.80, 0.62, 0.22, 1.0))
    pump_black = model.material("pump_black", rgba=(0.05, 0.05, 0.06, 1.0))

    # ---- bottle body (root) — IDENTICAL to parent ----
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
    # Pump housing collar on top of neck (inline body visual)
    body.visual(
        mesh_from_cadquery(_pump_housing_solid(), "pump_housing"),
        material=pump_black,
        name="pump_housing",
    )
    # Fork posts (repeated pair — loop with name_i style naming)
    for i in range(2):
        x_sign = -1 if i == 0 else 1
        x_pos = x_sign * FORK_POST_SPACING / 2
        body.visual(
            mesh_from_cadquery(_fork_post_solid(x_pos), f"fork_post_{i}"),
            material=pump_black,
            name=f"fork_post_{i}",
        )

    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H)),
        mass=0.090,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # ---- lever arm: swings DOWN to dispense (REVOLUTE replaces parent PRISMATIC) ----
    lever = model.part("lever")
    lever.visual(
        mesh_from_cadquery(_lever_solid(), "lever_arm"),
        material=pump_black,
        name="lever_arm",
    )
    lever.inertial = Inertial.from_geometry(
        Box((LEVER_HUB_W, LEVER_ARM_LENGTH, LEVER_HUB_D)),
        mass=0.008,
        origin=Origin(xyz=(0.0, LEVER_ARM_LENGTH / 2, 0.0)),
    )

    model.articulation(
        "lever_swing",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        # Joint origin at the pivot axis center, on the real fork contact surface
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        # axis=(-1,0,0): right-hand rule around -X rotates +Y toward -Z,
        # so positive q swings the lever arm DOWNWARD.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=0.0, upper=LEVER_SWING_UPPER
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lever = object_model.get_part("lever")
    swing = object_model.get_articulation("lever_swing")

    # ---- intentional pivot overlap: hub wraps around fork posts at the pivot axis ----
    for i in range(2):
        ctx.allow_overlap(
            lever, body,
            elem_a="lever_arm",
            elem_b=f"fork_post_{i}",
            reason=f"Lever hub wraps around fork_post_{i} at the revolute pivot axis for captured pivot contact.",
        )

    # ---- body proportions (same cushion form as parent) ----
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

    # ---- front label plate present ----
    label_aabb = ctx.part_element_world_aabb(body, elem="label_plate")
    ctx.check(
        "label plate present on the front face",
        label_aabb[1][1] > BODY_D / 2 - 0.002,
        details=f"label max y={label_aabb[1][1]}",
    )

    # ---- lever is mounted at the top of the bottle ----
    lever_aabb = ctx.part_world_aabb(lever)
    lever_center_z = (lever_aabb[0][2] + lever_aabb[1][2]) / 2.0
    ctx.check(
        "lever mounted at the top of the bottle",
        lever_aabb[0][2] > BODY_H - 0.010 and lever_center_z > BODY_H,
        details=f"lever aabb z=[{lever_aabb[0][2]}, {lever_aabb[1][2]}]",
    )

    # ---- lever arm extends outward at rest (positive Y extent) ----
    ctx.check(
        "lever arm extends outward in +Y at rest",
        lever_aabb[1][1] > 0.015,
        details=f"lever max y={lever_aabb[1][1]}",
    )

    # ---- fork posts present on body at the top ----
    for i in range(2):
        fp_aabb = ctx.part_element_world_aabb(body, elem=f"fork_post_{i}")
        ctx.check(
            f"fork_post_{i} present at top of bottle",
            fp_aabb[0][2] > BODY_H,
            details=f"fork_post_{i} z=[{fp_aabb[0][2]}, {fp_aabb[1][2]}]",
        )

    # ---- pump housing present between neck and fork ----
    housing_aabb = ctx.part_element_world_aabb(body, elem="pump_housing")
    ctx.check(
        "pump housing sits on the neck",
        housing_aabb[0][2] > BODY_H and housing_aabb[1][2] < BODY_H + 0.030,
        details=f"pump_housing z=[{housing_aabb[0][2]}, {housing_aabb[1][2]}]",
    )

    # ---- lever hub centered between fork posts on X axis ----
    ctx.expect_within(
        lever, body, axes="x", margin=0.002,
        name="lever hub centered within fork post span on X",
    )

    # ---- lever swings DOWN on positive q (revolute downstroke) ----
    lever_bottom_rest = lever_aabb[0][2]
    lever_tip_y_rest = lever_aabb[1][1]

    with ctx.pose({swing: LEVER_SWING_UPPER}):
        lever_aabb_swing = ctx.part_world_aabb(lever)
        lever_bottom_swing = lever_aabb_swing[0][2]
        lever_tip_y_swing = lever_aabb_swing[1][1]

    ctx.check(
        "lever tip drops when swung (revolute downstroke)",
        lever_bottom_swing < lever_bottom_rest - 0.005,
        details=f"rest bottom z={lever_bottom_rest}, swing bottom z={lever_bottom_swing}",
    )
    ctx.check(
        "lever center drops when swung (net downward rotation of arm)",
        (lever_aabb_swing[0][2] + lever_aabb_swing[1][2]) / 2
        < (lever_aabb[0][2] + lever_aabb[1][2]) / 2 - 0.005,
        details=f"rest center z={(lever_aabb[0][2]+lever_aabb[1][2])/2}, swing center z={(lever_aabb_swing[0][2]+lever_aabb_swing[1][2])/2}",
    )
    ctx.check(
        "lever arm Y extent shrinks when swung (arm tilts downward)",
        lever_tip_y_swing < lever_tip_y_rest - 0.005,
        details=f"rest max_y={lever_tip_y_rest}, swing max_y={lever_tip_y_swing}",
    )

    # ---- lever footprint overlaps body at rest (pivot is seated) ----
    ctx.expect_overlap(
        lever, body, axes="xy", min_overlap=0.003,
        name="lever hub seated at fork pivot (XY footprint)",
    )

    return ctx.report()


object_model = build_object_model()
