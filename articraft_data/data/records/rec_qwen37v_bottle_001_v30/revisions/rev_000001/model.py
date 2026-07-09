from __future__ import annotations

# Juice bottle variant: clear plastic body with molded volume bands,
# removable measuring-cup cap, and a straw spout that pivots up from the cap.
#
# Frame: bottle axis along +Z, base at z=0, neck/cap at top (+Z).
#
# Parts:
#   bottle        – root: thin-wall PET shell with 3 molded volume-band rings
#   cap_carrier   – massless carrier for cap rotation
#   measuring_cup – inverted cup that fits over the neck; graduation rings
#                   and a spout-pivot boss on top
#   spout         – straw tube that pivots from stowed (horizontal) to
#                   deployed (vertical upward)
#
# Joints:
#   cap_rotate  – CONTINUOUS, +Z axis, spins the cup
#   cap_slide   – PRISMATIC, +Z axis, lifts the cup off the neck
#   spout_pivot – REVOLUTE, −Y axis, positive q raises spout upward

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
    mesh_from_cadquery,
)

# ---- bottle body dimensions (meters) ----
BODY_R = 0.030
WALL = 0.0016
BASE_Z = 0.0
BARREL_TOP_Z = 0.108
SHOULDER_TOP_Z = 0.132
NECK_R = 0.0145
NECK_TOP_Z = 0.150

# ---- volume-band ridges on the barrel ----
BAND_ZS = [0.032, 0.062, 0.092]   # z-centers of the three bands
BAND_H = 0.0022                    # band height
BAND_BULGE = 0.0013                # outward protrusion

# ---- measuring-cup cap ----
CUP_R = 0.024
CUP_H = 0.034
CUP_WALL = 0.002
CUP_SKIRT = 0.014                  # how far the cup hangs below its origin
CUP_TOP_Z = CUP_H - CUP_SKIRT     # cup-local z of the top disc

# ---- spout-pivot boss on the cup top ----
BOSS_R = 0.005
BOSS_H = 0.006

# ---- straw spout ----
SPOUT_R = 0.003
SPOUT_BORE = 0.0018
SPOUT_LEN = 0.045


# ------------------------------------------------------------------ geometry
def _neck_thread_profile():
    pts = [(NECK_R, SHOULDER_TOP_Z)]
    z0 = SHOULDER_TOP_Z + 0.004
    ridge_r = NECK_R + 0.0016
    for k in range(3):
        zc = z0 + k * 0.0048
        pts.append((NECK_R, zc - 0.0016))
        pts.append((ridge_r, zc - 0.0006))
        pts.append((ridge_r, zc + 0.0006))
        pts.append((NECK_R, zc + 0.0016))
    pts.append((NECK_R, NECK_TOP_Z))
    return pts


def _bottle_shell():
    """Clear PET bottle with molded volume-band ridges on the barrel."""
    wp = (
        cq.Workplane("XZ")
        .moveTo(0.0, BASE_Z)
        .lineTo(BODY_R - 0.006, BASE_Z)
        .threePointArc((BODY_R, BASE_Z + 0.006), (BODY_R, BASE_Z + 0.012))
    )

    # barrel section with volume bands
    barrel_floor = BASE_Z + 0.012
    for bz in sorted(BAND_ZS):
        band_lo = bz - BAND_H / 2.0
        band_hi = bz + BAND_H / 2.0
        if band_lo > barrel_floor:
            wp = wp.lineTo(BODY_R, band_lo)
        wp = (
            wp.lineTo(BODY_R + BAND_BULGE, band_lo)
            .lineTo(BODY_R + BAND_BULGE, band_hi)
            .lineTo(BODY_R, band_hi)
        )
        barrel_floor = band_hi

    wp = wp.lineTo(BODY_R, BARREL_TOP_Z)

    # shoulder taper
    wp = wp.threePointArc(
        ((BODY_R + NECK_R) / 2.0, (BARREL_TOP_Z + SHOULDER_TOP_Z) / 2.0 + 0.004),
        (NECK_R, SHOULDER_TOP_Z),
    )
    # threaded neck
    for (r, z) in _neck_thread_profile()[1:]:
        wp = wp.lineTo(r, z)
    wp = wp.lineTo(0.0, NECK_TOP_Z).close()

    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    return outer.faces(">Z").shell(-WALL)


def _measuring_cup():
    """Inverted measuring-cup cap with graduation rings and spout boss."""
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -CUP_SKIRT))
        .circle(CUP_R)
        .extrude(CUP_H)
    )
    outer = outer.edges(">Z").fillet(0.003)

    # hollow interior (open at bottom); tight fit so thread ridges engage
    cavity = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -CUP_SKIRT - 0.001))
        .circle(NECK_R + 0.001)
        .extrude(CUP_H - 0.005)
    )
    cup = outer.cut(cavity)

    # graduation rings (thin ridges on outside)
    for frac in (0.28, 0.52, 0.76):
        rz = -CUP_SKIRT + frac * CUP_H
        ring = (
            cq.Workplane("XY")
            .transformed(offset=(0, 0, rz))
            .circle(CUP_R + 0.0007)
            .circle(CUP_R - 0.0001)
            .extrude(0.0009)
        )
        cup = cup.union(ring)

    # spout-pivot boss on cup top (center)
    boss = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, CUP_TOP_Z))
        .circle(BOSS_R)
        .extrude(BOSS_H)
    )
    cup = cup.union(boss)

    # two hinge ears flanking the boss (visual hinge bracket)
    ear_h = BOSS_H + 0.002
    ear_z = CUP_TOP_Z + ear_h / 2.0
    for dy in (-0.006, 0.006):
        ear = (
            cq.Workplane("XY")
            .transformed(offset=(0, dy, ear_z))
            .box(0.005, 0.002, ear_h)
        )
        cup = cup.union(ear)

    return cup


def _straw_spout():
    """Straw spout tube.  Local frame at pivot; tube extends along +X."""
    tube = (
        cq.Workplane("YZ")
        .circle(SPOUT_R)
        .extrude(SPOUT_LEN)
    )
    # bore starts past the pivot knuckle so it doesn't hollow the ball
    bore = (
        cq.Workplane("YZ")
        .transformed(offset=(SPOUT_R * 2.5, 0, 0))
        .circle(SPOUT_BORE)
        .extrude(SPOUT_LEN - SPOUT_R * 2.5)
    )
    spout = tube.cut(bore)
    # pivot knuckle ball at origin (hinge connection)
    knuckle = cq.Workplane("XY").sphere(SPOUT_R * 1.6)
    spout = spout.union(knuckle)
    return spout


# ------------------------------------------------------------------ model
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="juice_bottle_measuring_cup")

    clear = model.material("clear_pet", rgba=(0.80, 0.86, 0.84, 0.25))
    teal = model.material("cup_teal", rgba=(0.08, 0.32, 0.42, 1.0))
    white = model.material("spout_white", rgba=(0.92, 0.92, 0.90, 1.0))

    # ---- bottle body (root) ----
    body = model.part("bottle")
    shell = _bottle_shell()
    body.visual(
        mesh_from_cadquery(shell, "bottle_shell"),
        material=clear,
        name="bottle_shell",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.030,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- massless carrier ----
    carrier = model.part("cap_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.008, 0.008, 0.008)), mass=1e-4)

    # ---- measuring cup ----
    cup = model.part("measuring_cup")
    cup_geo = _measuring_cup()
    cup.visual(
        mesh_from_cadquery(cup_geo, "cup_shell"),
        material=teal,
        name="cup_shell",
    )
    cup.inertial = Inertial.from_geometry(
        Cylinder(CUP_R, CUP_H),
        mass=0.010,
        origin=Origin(xyz=(0.0, 0.0, (CUP_TOP_Z - CUP_SKIRT) / 2.0)),
    )

    # ---- straw spout ----
    spout = model.part("spout")
    spout_geo = _straw_spout()
    spout.visual(
        mesh_from_cadquery(spout_geo, "spout_shell"),
        material=white,
        name="spout_shell",
    )
    spout.inertial = Inertial.from_geometry(
        Cylinder(SPOUT_R, SPOUT_LEN),
        mass=0.004,
        origin=Origin(xyz=(SPOUT_LEN / 2.0, 0.0, 0.0)),
    )

    # ---- joints ----
    model.articulation(
        "cap_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )
    model.articulation(
        "cap_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=cup,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=CUP_H, effort=1.0, velocity=1.0),
    )
    # spout pivot: origin at the top of the boss on the cup,
    # axis −Y so positive q raises the +X-extending tube to +Z.
    spout_pivot_z_in_cup = CUP_TOP_Z + BOSS_H
    model.articulation(
        "spout_pivot",
        ArticulationType.REVOLUTE,
        parent=cup,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, spout_pivot_z_in_cup)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=math.pi / 2.0,
            effort=2.0,
            velocity=2.0,
        ),
    )

    return model


# ------------------------------------------------------------------ helpers
def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


# ------------------------------------------------------------------ tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle")
    cup = object_model.get_part("measuring_cup")
    spout = object_model.get_part("spout")
    rotate = object_model.get_articulation("cap_rotate")
    slide = object_model.get_articulation("cap_slide")
    pivot = object_model.get_articulation("spout_pivot")

    bottle_shell = body.get_visual("bottle_shell")
    cup_shell = cup.get_visual("cup_shell")
    spout_shell = spout.get_visual("spout_shell")

    # --- material checks ---
    ctx.check(
        "bottle material is tinted-transparent (alpha < 1)",
        bottle_shell.material.rgba is not None and bottle_shell.material.rgba[3] < 1.0,
        details=f"bottle rgba={bottle_shell.material.rgba}",
    )
    ctx.check(
        "measuring cup is opaque teal",
        cup_shell.material.rgba is not None
        and cup_shell.material.rgba[3] >= 0.99
        and cup_shell.material.rgba[1] > 0.2,
        details=f"cup rgba={cup_shell.material.rgba}",
    )
    ctx.check(
        "spout is opaque light-colored",
        spout_shell.material.rgba is not None
        and spout_shell.material.rgba[3] >= 0.99
        and min(spout_shell.material.rgba[:3]) > 0.8,
        details=f"spout rgba={spout_shell.material.rgba}",
    )

    # --- volume bands make the bottle wider than the plain barrel ---
    body_aabb = ctx.part_world_aabb(body)
    body_dx = body_aabb[1][0] - body_aabb[0][0]
    body_dy = body_aabb[1][1] - body_aabb[0][1]
    plain_dia = 2.0 * BODY_R
    ctx.check(
        "molded volume bands protrude beyond the plain barrel diameter",
        body_dx > plain_dia + BAND_BULGE * 1.5
        and body_dy > plain_dia + BAND_BULGE * 1.5,
        details=f"body_dx={body_dx:.5f}, body_dy={body_dy:.5f}, plain_dia={plain_dia:.5f}",
    )

    # --- cup sits on top of the neck ---
    cup_pos = ctx.part_world_position(cup)
    ctx.check(
        "measuring cup mounted above the barrel",
        cup_pos is not None and cup_pos[2] > BARREL_TOP_Z,
        details=f"cup origin={cup_pos}",
    )

    # intentional overlap: cup skirt wraps the threaded neck
    ctx.allow_overlap(
        cup,
        body,
        elem_a="cup_shell",
        elem_b="bottle_shell",
        reason="Measuring-cup skirt is intentionally seated over the threaded neck.",
    )

    # --- cap_rotate spins the cup ---
    with ctx.pose({rotate: 0.0}):
        aabb0 = ctx.part_world_aabb(cup)
    with ctx.pose({rotate: math.pi / 4.0}):
        aabb45 = ctx.part_world_aabb(cup)
    # the asymmetric hinge ears break symmetry, so a 45° rotation changes AABB
    e0 = _ext(aabb0)
    e45 = _ext(aabb45)
    ctx.check(
        "cap_rotate spins the measuring cup (AABB changes)",
        abs(e0[0] - e45[0]) > 1e-4 or abs(e0[1] - e45[1]) > 1e-4,
        details=f"rest={e0}, 45deg={e45}",
    )

    # --- cap_slide lifts the cup off the neck ---
    rest_z = ctx.part_world_position(cup)[2]
    with ctx.pose({slide: CUP_H}):
        lifted_z = ctx.part_world_position(cup)[2]
    ctx.check(
        "cap_slide lifts the measuring cup off the neck",
        lifted_z > rest_z + CUP_H * 0.8,
        details=f"rest_z={rest_z}, lifted_z={lifted_z}",
    )

    # --- spout_pivot raises the spout upward ---
    with ctx.pose({pivot: 0.0}):
        stowed_top = ctx.part_world_aabb(spout)[1][2]  # max z
    with ctx.pose({pivot: math.pi / 2.0}):
        deployed_top = ctx.part_world_aabb(spout)[1][2]
    ctx.check(
        "spout_pivot raises the spout tip upward",
        deployed_top > stowed_top + 0.015,
        details=f"stowed_top_z={stowed_top}, deployed_top_z={deployed_top}",
    )

    # prove the spout actually moves (non-fixed joint)
    ctx.check(
        "spout deployed position differs from stowed",
        deployed_top - stowed_top > 0.010,
        details=f"delta_z={deployed_top - stowed_top}",
    )

    # spout knuckle overlaps the cup boss (hinge pin in bracket) – intentional
    ctx.allow_overlap(
        spout,
        cup,
        elem_a="spout_shell",
        elem_b="cup_shell",
        reason="Spout pivot knuckle is intentionally nested in the cup hinge boss.",
    )

    return ctx.report()


object_model = build_object_model()
