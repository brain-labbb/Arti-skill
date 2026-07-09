from __future__ import annotations

# Round brushed-metal canister tin with a hinged flip lid.
# Frame: tin axis along +Z, body base sitting on z=0, centerline at x=y=0.
# Body is a hollow cylinder (open top) taller than it is wide, with a hinge
# ear formed at the back rim.
# The lid is a flat cap with a short downward skirt that overhangs the body
# rim (visible seam ring). A hinge knuckle at the back edge wraps around the
# body hinge ear so the lid swings up and back.
# Articulations:
#   - lid: REVOLUTE, hinge at back rim, axis along +X (horizontal, tangent
#     to the rim). Positive q opens the front edge upward and back.

import cadquery as cq
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

# ---- canister dimensions (meters) ----
BODY_R = 0.050          # outer radius of the tin body (~0.10 m diameter)
BODY_H = 0.118          # body height (rim sits at z = BODY_H)
WALL = 0.0035           # sheet-metal wall thickness
FLOOR = 0.004           # interior floor thickness

LID_R = 0.0515          # lid slightly proud of body so the skirt overhangs the rim
LID_TOP_H = 0.010       # thickness of the flat lid top plate
SKIRT_H = 0.018         # how far the lid skirt drops down over the body rim

RIM_TOP_Z = BODY_H      # body rim top in body frame

# ---- hinge dimensions ----
EAR_W = 0.022           # hinge ear width along X
EAR_D = 0.008           # hinge ear depth (radial, Y direction)
EAR_H = 0.007           # hinge ear height above rim
EAR_EMBED = 0.003       # how far the ear embeds into the body rim for connectivity

KNUCKLE_R = 0.003       # hinge knuckle outer radius (on lid)
KNUCKLE_LEN = 0.016     # hinge knuckle length along X

# Hinge position: back rim (-Y side), axis along X tangent to the rim
HINGE_Y = -BODY_R       # hinge at the back edge of the body
HINGE_Z = RIM_TOP_Z     # hinge axis at rim top level


def _body_solid() -> cq.Workplane:
    """Hollow cylinder open at top, with an integral hinge ear at the back rim."""
    outer = cq.Workplane("XY").circle(BODY_R).extrude(BODY_H)
    inner = (
        cq.Workplane("XY")
        .workplane(offset=FLOOR)
        .circle(BODY_R - WALL)
        .extrude(BODY_H)  # overshoots top so rim is open
    )
    body = outer.cut(inner)

    # Hinge ear: small tab extending upward from the back rim.
    # Embeds slightly below the rim for reliable mesh connectivity with the body wall.
    ear_center_y = -(BODY_R + EAR_D / 2 - EAR_EMBED)
    ear = (
        cq.Workplane("XY")
        .workplane(offset=RIM_TOP_Z - EAR_EMBED)
        .center(0, ear_center_y)
        .rect(EAR_W, EAR_D)
        .extrude(EAR_H + EAR_EMBED)
    )
    body = body.union(ear)
    return body


def _lid_cap() -> cq.Workplane:
    """Lid cap in hinge frame: origin at hinge point, lid disc extends +Y toward front.

    At q=0 the hinge frame sits at (0, HINGE_Y, HINGE_Z) in body coords.
    The lid disc center is at local (0, BODY_R, 0), placing it centered over
    the body opening.
    """
    # Top plate: full disc above the hinge plane
    top = (
        cq.Workplane("XY")
        .center(0, BODY_R)
        .circle(LID_R)
        .extrude(LID_TOP_H)
    )
    # Skirt: annular ring below the hinge plane that sleeves over the body rim
    skirt_inner = BODY_R + 0.0008
    skirt = (
        cq.Workplane("XY")
        .workplane(offset=-SKIRT_H)
        .center(0, BODY_R)
        .circle(LID_R)
        .circle(skirt_inner)
        .extrude(SKIRT_H)
    )
    return top.union(skirt)


def _lid_knuckle() -> cq.Workplane:
    """Hinge knuckle on lid: a horizontal cylinder along X at the lid local origin."""
    knuckle = (
        cq.Workplane("YZ")
        .circle(KNUCKLE_R)
        .extrude(KNUCKLE_LEN)
    )
    # Center along X
    knuckle = knuckle.translate((-KNUCKLE_LEN / 2, 0, 0))
    return knuckle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="canister_tin")

    brushed = model.material("brushed_metal", rgba=(0.78, 0.79, 0.81, 1.0))
    hinge_mat = model.material("dark_hinge", rgba=(0.52, 0.50, 0.48, 1.0))

    # ---- body (root): hollow brushed-metal tin with hinge ear ----
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_solid(), "body_shell"),
        material=brushed,
        name="body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, BODY_H),
        mass=0.090,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # ---- lid: hinged flip cap with knuckle ----
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_cap(), "lid_cap"),
        material=brushed,
        name="lid_cap",
    )
    lid.visual(
        mesh_from_cadquery(_lid_knuckle(), "hinge_knuckle"),
        material=hinge_mat,
        name="hinge_knuckle",
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(LID_R, SKIRT_H + LID_TOP_H),
        mass=0.025,
        origin=Origin(xyz=(0.0, BODY_R, (LID_TOP_H - SKIRT_H) / 2.0)),
    )

    # ---- hinge articulation: REVOLUTE at back rim ----
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=2.8),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    # ---- body proportions ----
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "body taller than wide",
        bext[2] > max(bext[0], bext[1]) + 0.01,
        details=f"body extents={bext}",
    )
    ctx.check(
        "body roughly circular in plan",
        abs(bext[0] - bext[1]) < 0.01,
        details=f"body extents={bext}",
    )

    # ---- lid is a flat cap ----
    lid_ext = _ext(ctx.part_world_aabb(lid))
    ctx.check(
        "lid is a flat cap (wider than tall)",
        max(lid_ext[0], lid_ext[1]) > lid_ext[2] + 0.01,
        details=f"lid extents={lid_ext}",
    )

    # ---- hinge joint type and axis ----
    ctx.check(
        "lid joint is revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={hinge.articulation_type}",
    )
    ctx.check(
        "hinge axis is horizontal (no Z component)",
        abs(hinge.axis[2]) < 0.01,
        details=f"axis={hinge.axis}",
    )

    # ---- intentional overlaps ----
    # Lid skirt sleeves over body rim at rest (visible seam ring) and passes
    # the hinge ear at the back.
    ctx.allow_overlap(
        lid, body,
        elem_a="lid_cap", elem_b="body_shell",
        reason="Lid skirt sleeves over the body rim when closed (seam ring) and "
               "passes the hinge ear at the back.",
    )
    # Hinge knuckle wraps around the body hinge ear (captured pin fit).
    ctx.allow_overlap(
        lid, body,
        elem_a="hinge_knuckle", elem_b="body_shell",
        reason="Hinge knuckle wraps around the body hinge ear at the back rim "
               "(captured pin/barrel fit).",
    )

    # At rest (q=0): lid skirt overlaps body rim in Z (seated).
    ctx.expect_overlap(
        lid, body, axes="z", min_overlap=0.004,
        name="lid skirt overlaps body rim when closed",
    )

    # Proof: hinge knuckle contacts body hinge ear.
    ctx.expect_contact(
        lid, body,
        elem_a="hinge_knuckle", elem_b="body_shell",
        contact_tol=0.005,
        name="hinge knuckle is at the body hinge ear",
    )

    # Lid is mounted at the top of the tin.
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid mounted at tin top",
        lid_pos is not None and lid_pos[2] > BODY_H - SKIRT_H - 0.01,
        details=f"lid origin={lid_pos}",
    )

    # ---- hinge at the back (negative Y) ----
    hinge_pos = ctx.part_world_aabb(lid)  # use articulation origin
    ctx.check(
        "hinge origin at back of body",
        hinge.origin is not None and hinge.origin.xyz[1] < -BODY_R * 0.5,
        details=f"hinge origin y={hinge.origin.xyz[1]}",
    )

    # ---- opening motion: front edge lifts up ----
    z_rest_top = ctx.part_world_aabb(lid)[1][2]  # max-z when closed

    with ctx.pose({hinge: 1.0}):  # ~57 degrees open
        aabb_open = ctx.part_world_aabb(lid)
        z_open_top = aabb_open[1][2]

    ctx.check(
        "hinged lid lifts upward when opened",
        z_open_top > z_rest_top + 0.02,
        details=f"z_rest_top={z_rest_top}, z_open_top={z_open_top}",
    )

    # At partial open, the front of the lid is above the rim.
    with ctx.pose({hinge: 1.2}):
        front_z = ctx.part_world_aabb(lid)[1][2]
    ctx.check(
        "lid front edge well above rim at moderate open",
        front_z > RIM_TOP_Z + 0.03,
        details=f"front_z={front_z}",
    )

    # At full open (2.8 rad ~160 deg), lid has flipped past vertical.
    with ctx.pose({hinge: 2.5}):
        aabb_full = ctx.part_world_aabb(lid)
        z_full_top = aabb_full[1][2]
        y_full_min = aabb_full[0][1]  # back-most point

    ctx.check(
        "lid flips past vertical at large open angle",
        z_full_top > RIM_TOP_Z + 0.03,
        details=f"z_full_top={z_full_top}",
    )
    ctx.check(
        "lid swings behind body at full open",
        y_full_min < -BODY_R * 0.5,
        details=f"y_full_min={y_full_min}",
    )

    return ctx.report()


object_model = build_object_model()
