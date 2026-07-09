from __future__ import annotations

# Automatic parking barrier (boom gate).
#
# Coordinate convention:
#   - up is +Z; the ground / footprint sits at z = 0.
#   - the motor housing post stands at the origin (its footprint centered on x=y=0).
#   - the boom arm extends out along local -X from a horizontal pivot at the top
#     of the post and, when closed, lies horizontal at the pivot height.
#
# Root structure: the housing post (base column + cap + access panel + pivot hub)
# is the root. The boom arm is the single moving part: it lifts from horizontal
# (closed, q=0) to nearly vertical (open, q~=85 deg) about a HORIZONTAL axis (+Y)
# at the top of the post. This is the primary user-facing articulation.
#
# Visual cues from the reference image:
#   - tan/orange housing column with a dark plastic crown cap on top.
#   - a dark recessed access/inspection panel on the front face of the column.
#   - a dark circular pivot hub / motor flange on the side of the column where
#     the boom attaches.
#   - a long boom arm with alternating black and orange diagonal stripe bands,
#     tipped with a dark end cap, tapering slightly toward the free end.

import math

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
)

# ---- key dimensions (meters) -------------------------------------------------
POST_W = 0.32          # housing column width (x)
POST_D = 0.26          # housing column depth (y)
POST_H = 1.02          # housing column height (z)
CAP_W = 0.36
CAP_D = 0.30
CAP_H = 0.08

PIVOT_Z = POST_H - 0.10      # boom pivot height (just below the cap)
HUB_R = 0.085               # pivot hub / motor flange radius
HUB_T = 0.05                # hub thickness along y
PIVOT_X_FACE = -POST_W * 0.5  # -X face of the column
PIVOT_X = PIVOT_X_FACE - 0.10  # pivot/hub center, proud of the column face

BOOM_LEN = 3.05             # full boom length
BOOM_ROOT_H = 0.11          # boom cross-section height at the root (z extent)
BOOM_ROOT_T = 0.055         # boom thickness (y extent) at the root
BOOM_TIP_H = 0.075          # boom cross-section height at the tip
BOOM_TIP_T = 0.04           # boom thickness at the tip
STRIPE_BANDS = 7            # number of stripe segments along the boom

OPEN_ANGLE = math.radians(85.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="parking_barrier")

    tan = model.material("tan_housing", rgba=(0.86, 0.62, 0.36, 1.0))
    dark = model.material("dark_plastic", rgba=(0.16, 0.16, 0.18, 1.0))
    steel = model.material("steel", rgba=(0.55, 0.57, 0.60, 1.0))
    boom_orange = model.material("boom_orange", rgba=(0.90, 0.55, 0.22, 1.0))
    boom_black = model.material("boom_black", rgba=(0.12, 0.12, 0.13, 1.0))

    # ---- housing post (root) -------------------------------------------------
    post = model.part("housing_post")
    # main tan column body
    post.visual(
        Box((POST_W, POST_D, POST_H)),
        origin=Origin(xyz=(0.0, 0.0, POST_H * 0.5)),
        material=tan,
    )
    # dark base trim / foot plate at z=0
    post.visual(
        Box((POST_W + 0.04, POST_D + 0.04, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark,
    )
    # dark crown cap on top
    post.visual(
        Box((CAP_W, CAP_D, CAP_H)),
        origin=Origin(xyz=(0.0, 0.0, POST_H + CAP_H * 0.5)),
        material=dark,
    )
    # recessed dark access/inspection panel on the front (-X) face
    post.visual(
        Box((0.015, POST_D - 0.10, POST_H - 0.22)),
        origin=Origin(xyz=(-POST_W * 0.5 + 0.004, 0.0, POST_H * 0.5 - 0.02)),
        material=dark,
        name="access_panel",
    )
    # dark pivot hub / motor flange protruding from the -X face of the column at
    # pivot height; the boom collar nests onto this. The hub straddles the face:
    # it reaches back into the column (connection) and out past it (collar seat).
    post.visual(
        Cylinder(radius=HUB_R, length=HUB_T),
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="pivot_hub",
    )
    # short steel shaft from the column face out to the hub, so it reads connected
    post.visual(
        Cylinder(radius=0.03, length=PIVOT_X_FACE - PIVOT_X + 0.04),
        origin=Origin(
            xyz=(0.5 * (PIVOT_X_FACE + PIVOT_X) + 0.02, 0.0, PIVOT_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="pivot_shaft",
    )
    post.inertial = Inertial.from_geometry(
        Box((POST_W, POST_D, POST_H)),
        mass=45.0,
        origin=Origin(xyz=(0.0, 0.0, POST_H * 0.5)),
    )

    # ---- boom arm (child) ----------------------------------------------------
    # The boom is authored in its own local frame with the pivot at local origin.
    # It extends along local -X. At q=0 (closed) this lies horizontal at the
    # pivot height. The cross-section tapers from root to tip.
    boom = model.part("boom_arm")

    # root mounting collar that hugs the pivot hub (so the arm reads attached).
    # Sized to reach from the hub out to the start of the striped beam.
    boom.visual(
        Cylinder(radius=HUB_R + 0.012, length=HUB_T + 0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="boom_collar",
    )

    # striped beam body: a row of tapered box segments alternating orange / black.
    # The beam begins just outboard of the hub so the only part inside the hub is
    # the collar; the beam itself stays clear of the column-side pivot assembly.
    BEAM_START = -(HUB_R + 0.01)
    beam_span = BOOM_LEN + BEAM_START   # total striped length from BEAM_START to -BOOM_LEN
    seg_len = beam_span / STRIPE_BANDS
    for i in range(STRIPE_BANDS):
        x_near = BEAM_START - i * seg_len           # closer to pivot
        x_far = BEAM_START - (i + 1) * seg_len      # farther from pivot
        x_mid = 0.5 * (x_near + x_far)
        # linear taper along the arm based on distance from pivot
        t_mid = (-x_mid) / BOOM_LEN
        h = BOOM_ROOT_H + (BOOM_TIP_H - BOOM_ROOT_H) * t_mid
        th = BOOM_ROOT_T + (BOOM_TIP_T - BOOM_ROOT_T) * t_mid
        mat = boom_orange if (i % 2 == 0) else boom_black
        boom.visual(
            Box((seg_len, th, h)),
            origin=Origin(xyz=(x_mid, 0.0, 0.0)),
            material=mat,
            name=f"stripe_{i}",
        )

    # dark end cap at the free tip
    boom.visual(
        Box((0.05, BOOM_TIP_T + 0.006, BOOM_TIP_H + 0.006)),
        origin=Origin(xyz=(-BOOM_LEN - 0.02, 0.0, 0.0)),
        material=dark,
        name="boom_tip_cap",
    )

    boom.inertial = Inertial.from_geometry(
        Box((BOOM_LEN, BOOM_ROOT_T, BOOM_ROOT_H)),
        mass=8.0,
        origin=Origin(xyz=(-BOOM_LEN * 0.5, 0.0, 0.0)),
    )

    # ---- articulation: boom lift ---------------------------------------------
    # Pivot at the top of the post, on the -X side at the hub center.
    # Axis is +Y (horizontal). Positive q lifts the -X-pointing arm tip upward:
    # for a point at (-L,0,0), rotation about +Y by +q gives z' = +L*sin(q) > 0.
    model.articulation(
        "post_to_boom",
        ArticulationType.REVOLUTE,
        parent=post,
        child=boom,
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=0.0, upper=OPEN_ANGLE),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    post = object_model.get_part("housing_post")
    boom = object_model.get_part("boom_arm")
    lift = object_model.get_articulation("post_to_boom")

    # --- primary joint: type / axis / limits ---
    ctx.check(
        "boom joint is revolute",
        str(lift.articulation_type).endswith("REVOLUTE"),
        details=f"type={lift.articulation_type}",
    )
    ctx.check(
        "boom pivot axis is horizontal (+Y)",
        tuple(round(a, 6) for a in lift.axis) == (0.0, 1.0, 0.0),
        details=f"axis={lift.axis}",
    )
    lim = lift.motion_limits
    ctx.check(
        "boom closed at horizontal (lower=0)",
        lim is not None and lim.lower is not None and abs(lim.lower) < 1e-6,
        details=f"lower={None if lim is None else lim.lower}",
    )
    ctx.check(
        "boom opens to roughly vertical (upper ~85 deg)",
        lim is not None and lim.upper is not None and math.radians(80.0) <= lim.upper <= math.radians(90.0),
        details=f"upper={None if lim is None else lim.upper}",
    )

    # --- base sits on the ground (z ~= 0) ---
    paabb = ctx.part_world_aabb(post)
    ctx.check(
        "housing post rests at z=0",
        paabb is not None and abs(paabb[0][2]) < 1e-3,
        details=f"post_min_z={None if paabb is None else paabb[0][2]}",
    )

    # --- boom is long and roughly horizontal when closed ---
    baabb = ctx.part_world_aabb(boom)
    if baabb is not None:
        span_x = baabb[1][0] - baabb[0][0]
        span_z = baabb[1][2] - baabb[0][2]
        ctx.check(
            "closed boom is long along X",
            span_x > 2.8,
            details=f"span_x={span_x}",
        )
        ctx.check(
            "closed boom is roughly horizontal (small z extent)",
            span_z < 0.2,
            details=f"span_z={span_z}",
        )

    # --- boom attaches at the top of the post, not floating ---
    ctx.expect_contact(
        post,
        boom,
        elem_a="pivot_hub",
        elem_b="boom_collar",
        contact_tol=0.01,
        name="boom collar meets the pivot hub",
    )

    # --- the pivot hub bridges back to the post column (not floating) ---
    ctx.expect_contact(
        post,
        post,
        elem_a="pivot_hub",
        elem_b="pivot_shaft",
        contact_tol=0.01,
        name="pivot hub joins the shaft back to the column",
    )

    # --- decisive open-pose check: free tip rises well above the pivot ---
    rest_tip = None
    open_tip = None
    rest = ctx.part_world_aabb(boom)
    if rest is not None:
        rest_tip = rest[1][2]
    with ctx.pose({lift: OPEN_ANGLE}):
        oa = ctx.part_world_aabb(boom)
        if oa is not None:
            open_tip = oa[1][2]
    ctx.check(
        "opening lifts the boom upward",
        rest_tip is not None and open_tip is not None and open_tip > rest_tip + 1.5,
        details=f"rest_top_z={rest_tip}, open_top_z={open_tip}",
    )

    # the collar nests onto the pivot hub / shaft by design (a captured pivot)
    ctx.allow_overlap(
        post,
        boom,
        elem_a="pivot_hub",
        elem_b="boom_collar",
        reason="The boom mounting collar intentionally seats over the pivot hub flange.",
    )
    ctx.allow_overlap(
        post,
        boom,
        elem_a="pivot_shaft",
        elem_b="boom_collar",
        reason="The boom collar is captured around the pivot shaft at the hinge axis.",
    )

    return ctx.report()


object_model = build_object_model()
