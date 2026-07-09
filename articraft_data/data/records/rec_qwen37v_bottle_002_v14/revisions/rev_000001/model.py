from __future__ import annotations

# Swing-top bottle for soap/lotion:
#   hollow clear body with 3 molded volume bands, tapered shoulder,
#   raised spiral-like neck threads, hinge collar with two upright arms
#   and a pivot pin, and a white swing-top stopper cap that flips open.
# Frame: vertical axis along +Z, bottle standing on z=0, centerline on x=y=0.
# Articulation: cap_swing – REVOLUTE about Y at the pivot pin, 0 (closed)
#   to ~2.8 rad (fully swung open past vertical).

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key heights (m) along +Z ----
BODY_TOP_Z = 0.110
SHOULDER_TOP_Z = 0.156
NECK_TOP_Z = 0.176

BODY_R = 0.0275
NECK_R = 0.0125
NECK_BORE_R = 0.0098

# ---- hinge hardware ----
ARM_HEIGHT = 0.022                     # posts extend this far above neck top
PIVOT_Z = NECK_TOP_Z + ARM_HEIGHT     # 0.198
ARM_OFFSET = NECK_R + 0.004           # post Y offset from centre (0.0165)
ARM_POST_R = 0.0018
COLLAR_R = NECK_R + 0.005             # collar outer radius (0.0175)

# ---- swing cap ----
STOPPER_R = NECK_R + 0.002            # 0.0145
STOPPER_H = 0.006
ARM_DROP = ARM_HEIGHT                 # pivot-to-neck-top distance
STOPPER_LOCAL_Z = -(ARM_DROP - STOPPER_H / 2)  # -0.019

BAIL_Y = round(STOPPER_R * 0.78, 4)   # bail arm Y offset (~0.0113)
BAIL_W = 0.003                         # bail X thickness
BAIL_D = 0.003                         # bail Y depth

HANDLE_L = 0.016
HANDLE_W = 0.010
HANDLE_T = 0.003


# ------------------------------------------------------------------ geometry

def _profile_sections():
    """Outer wall: base → body with 3 volume bands → tapered shoulder → neck."""
    b = 0.0015  # band bump
    return [
        (0.000, 0.0150),
        (0.006, 0.0250),
        (0.014, 0.0273),
        # band 1
        (0.030, BODY_R),
        (0.031, BODY_R + b),
        (0.035, BODY_R + b),
        (0.036, BODY_R),
        # band 2
        (0.054, BODY_R),
        (0.055, BODY_R + b),
        (0.059, BODY_R + b),
        (0.060, BODY_R),
        # band 3
        (0.078, BODY_R),
        (0.079, BODY_R + b),
        (0.083, BODY_R + b),
        (0.084, BODY_R),
        (BODY_TOP_Z, BODY_R),
        # tapered shoulder
        (0.124, 0.0268),
        (0.138, 0.0228),
        (SHOULDER_TOP_Z, 0.0148),
        # neck
        (0.160, NECK_R),
        (NECK_TOP_Z, NECK_R),
    ]


def _bottle_solid() -> cq.Workplane:
    """Hollow bottle: revolved outer profile (with bands) minus smooth cavity."""
    pts = _profile_sections()
    wp = cq.Workplane("XZ").moveTo(0.0, pts[0][0])
    for r, z in [(r, z) for (z, r) in pts]:
        wp = wp.lineTo(r, z)
    wp = wp.lineTo(0.0, pts[-1][0]).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))

    wall = 0.0014
    inner_pts = [
        (0.010, 0.006),
        (0.0235, 0.012),
        (BODY_R - wall, 0.014),
        (BODY_R - wall, BODY_TOP_Z),
        (0.0254, 0.124),
        (0.0214, 0.138),
        (0.0134, SHOULDER_TOP_Z),
        (NECK_BORE_R, 0.160),
        (NECK_BORE_R, NECK_TOP_Z + 0.005),
    ]
    iwp = cq.Workplane("XZ").moveTo(0.0, inner_pts[0][1])
    for r, z in inner_pts:
        iwp = iwp.lineTo(r, z)
    iwp = iwp.lineTo(0.0, inner_pts[-1][1]).close()
    cavity = iwp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    return outer.cut(cavity)


def _neck_threads_mesh():
    """Raised ridges on the neck – closely-spaced torus rings."""
    g = None
    n = 7
    z0, z1 = 0.159, 0.174
    for i in range(n):
        z = z0 + (z1 - z0) * i / (n - 1)
        ring = TorusGeometry(
            NECK_R + 0.0003, 0.0008,
            radial_segments=8, tubular_segments=36,
        )
        ring.translate(0.0, 0.0, z)
        if g is None:
            g = ring
        else:
            g.merge(ring)
    return mesh_from_geometry(g, "neck_threads")


def _collar_posts_solid() -> cq.Workplane:
    """Hinge collar ring, two upright posts, and a horizontal pivot pin."""
    collar = (
        cq.Workplane("XY")
        .workplane(offset=NECK_TOP_Z - 0.004)
        .circle(COLLAR_R)
        .circle(NECK_R + 0.001)
        .extrude(0.004)
    )
    for sign in (-1, 1):
        post = (
            cq.Workplane("XY")
            .workplane(offset=NECK_TOP_Z)
            .center(0, sign * ARM_OFFSET)
            .circle(ARM_POST_R)
            .extrude(ARM_HEIGHT)
        )
        collar = collar.union(post)
    # pivot pin: horizontal cylinder along Y connecting the post tops
    pin = (
        cq.Workplane("XZ")
        .workplane(offset=-ARM_OFFSET)
        .center(0, PIVOT_Z)
        .circle(ARM_POST_R * 0.7)
        .extrude(ARM_OFFSET * 2)
    )
    collar = collar.union(pin)
    return collar


def _cap_solid() -> cq.Workplane:
    """Swing-top cap: stopper disc + two bail arms + flip handle tab."""
    sz = STOPPER_LOCAL_Z

    # stopper disc
    stopper = (
        cq.Workplane("XY")
        .workplane(offset=sz - STOPPER_H / 2)
        .circle(STOPPER_R)
        .extrude(STOPPER_H)
    )

    # bail arms – thin bars from stopper top up toward the pivot
    bail_bottom = sz + STOPPER_H / 2
    bail_top = -0.003
    bail_len = bail_top - bail_bottom
    for sign in (-1, 1):
        arm = (
            cq.Workplane("XY")
            .workplane(offset=bail_bottom)
            .center(0, sign * BAIL_Y)
            .rect(BAIL_W, BAIL_D)
            .extrude(bail_len)
        )
        stopper = stopper.union(arm)

    # flip handle tab extending in +X from the stopper
    hx = STOPPER_R + HANDLE_L / 2
    handle = (
        cq.Workplane("XY")
        .workplane(offset=sz - HANDLE_T / 2)
        .center(hx, 0)
        .rect(HANDLE_L, HANDLE_W)
        .extrude(HANDLE_T)
    )
    stopper = stopper.union(handle)
    return stopper


# ------------------------------------------------------------------ model

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_top_bottle")

    clear_body = model.material("clear_body", rgba=(0.88, 0.90, 0.86, 0.45))
    thread_mat = model.material("neck_thread", rgba=(0.82, 0.84, 0.80, 0.55))
    hw_mat = model.material("hinge_hw", rgba=(0.78, 0.78, 0.76, 1.0))
    cap_mat = model.material("cap_white", rgba=(0.94, 0.94, 0.92, 1.0))

    # ---- bottle body (root) ----
    body = model.part("bottle_body")
    body.visual(
        mesh_from_cadquery(_bottle_solid(), "bottle_shell"),
        material=clear_body, name="bottle_shell",
    )
    body.visual(
        _neck_threads_mesh(),
        material=thread_mat, name="neck_threads",
    )
    body.visual(
        mesh_from_cadquery(_collar_posts_solid(), "hinge_hardware"),
        material=hw_mat, name="hinge_hardware",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.028,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2)),
    )

    # ---- swing cap ----
    cap = model.part("swing_cap")
    cap.visual(
        mesh_from_cadquery(_cap_solid(), "cap_assembly"),
        material=cap_mat, name="cap_assembly",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(STOPPER_R, ARM_DROP + STOPPER_H),
        mass=0.006,
        origin=Origin(xyz=(0.0, 0.0, STOPPER_LOCAL_Z)),
    )

    # ---- articulation: swing hinge ----
    model.articulation(
        "cap_swing",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=2.8, effort=2.0, velocity=2.0,
        ),
    )

    return model


# ------------------------------------------------------------------ tests

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle_body")
    cap = object_model.get_part("swing_cap")
    swing = object_model.get_articulation("cap_swing")
    limits = swing.motion_limits

    # --- joint is REVOLUTE with proper bounded limits ---
    ctx.check(
        "cap_swing is revolute",
        swing.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={swing.articulation_type}",
    )
    ctx.check(
        "swing joint has finite lower/upper bounds",
        limits is not None and limits.lower is not None and limits.upper is not None
        and limits.upper > limits.lower + 1.0,
        details=f"lower={limits.lower}, upper={limits.upper}",
    )

    # --- volume bands: body AABB wider than 2*BODY_R ---
    body_aabb = ctx.part_world_aabb(body)
    body_dx = body_aabb[1][0] - body_aabb[0][0]
    ctx.check(
        "body has volume bands (wider than 2*BODY_R)",
        body_dx > 2 * BODY_R + 0.0008,
        details=f"body_dx={body_dx:.5f}, 2*BODY_R+tol={2*BODY_R+0.0008:.5f}",
    )

    # --- neck threads visual exists ---
    ctx.check(
        "neck threads visual present",
        body.get_visual("neck_threads") is not None,
    )

    # --- hinge hardware visual exists ---
    ctx.check(
        "hinge hardware visual present",
        body.get_visual("hinge_hardware") is not None,
    )

    # --- cap is mounted above the body ---
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "swing cap mounted near top of bottle",
        cap_pos is not None and cap_pos[2] > NECK_TOP_Z,
        details=f"cap origin z={cap_pos[2]:.4f}",
    )

    # --- intentional overlap: pivot pin captured by bail arms ---
    ctx.allow_overlap(
        body, cap,
        elem_a="hinge_hardware",
        elem_b="cap_assembly",
        reason="The bail arms of the swing cap clip around the pivot pin to form the hinge.",
    )
    ctx.expect_contact(
        body, cap,
        elem_a="hinge_hardware",
        elem_b="cap_assembly",
        name="bail arms contact pivot pin",
    )

    # --- at rest (q=0) cap is seated near the neck rim ---
    cap_aabb_rest = ctx.part_world_aabb(cap)
    cap_bottom_rest = cap_aabb_rest[0][2]
    ctx.check(
        "closed cap bottom is near neck rim",
        abs(cap_bottom_rest - NECK_TOP_Z) < 0.003,
        details=f"cap_bottom={cap_bottom_rest:.4f}, neck_top={NECK_TOP_Z}",
    )

    # --- at upper limit, cap centre has swung well above the neck ---
    upper = limits.upper if limits.upper is not None else 2.8

    def _cap_centre_z():
        mn, mx = ctx.part_world_aabb(cap)
        return (mn[2] + mx[2]) / 2.0

    cz_rest = _cap_centre_z()
    with ctx.pose({swing: upper}):
        cz_open = _cap_centre_z()
    ctx.check(
        "open cap centre swings above pivot height",
        cz_open > PIVOT_Z,
        details=f"rest centre z={cz_rest:.4f}, open centre z={cz_open:.4f}, pivot_z={PIVOT_Z}",
    )

    # --- cap AABB actually shifts when joint is actuated ---
    with ctx.pose({swing: 2.0}):
        cz_mid = _cap_centre_z()
    moved = abs(cz_mid - cz_rest)
    ctx.check(
        "cap centre moves when swing joint opens",
        moved > 0.005,
        details=f"rest cz={cz_rest:.4f}, mid cz={cz_mid:.4f}, moved={moved:.4f}",
    )

    # --- tapered shoulder still narrows toward the top ---
    ctx.check(
        "tapered shoulder narrows toward the top",
        NECK_R < BODY_R * 0.6,
        details=f"neck_r={NECK_R}, body_r={BODY_R}",
    )

    # --- bottle is taller than wide ---
    body_dz = body_aabb[1][2] - body_aabb[0][2]
    ctx.check(
        "bottle is tall (taller than wide)",
        body_dz > 2.5 * body_dx,
        details=f"body_dz={body_dz:.4f}, body_dx={body_dx:.4f}",
    )

    # --- bottle body is a clear/translucent material ---
    clear_mat = next(m for m in object_model.materials if m.name == "clear_body")
    a = clear_mat.rgba[3] if clear_mat.rgba is not None else 1.0
    ctx.check(
        "bottle shell is translucent",
        a < 1.0,
        details=f"clear_body alpha={a}",
    )

    return ctx.report()


object_model = build_object_model()
