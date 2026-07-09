from __future__ import annotations

# Clear plastic juice bottle with a black ribbed screw cap.
# Frame: bottle axis along +Z, base at z=0, neck/cap at the top (+Z).
# The body is a transparent thin-wall PET shell: rounded base + cylindrical
# barrel + shoulder taper + short threaded neck.
# The black ribbed cap rides on the neck and has two INDEPENDENT decoupled
# joints (share the vertical +Z cap axis) through a massless carrier link:
#   - cap_rotate: CONTINUOUS spin of the cap about +Z
#   - cap_slide:  PRISMATIC lift of the cap up off the neck
# A small off-axis marker rib on the cap makes the spin detectable.

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

# ---- key dimensions (meters) ----
BODY_R = 0.030          # outer barrel radius (~0.06 m dia)
WALL = 0.0016           # thin PET wall thickness
BASE_Z = 0.0            # bottom of the bottle
BARREL_TOP_Z = 0.108    # where the shoulder taper begins
SHOULDER_TOP_Z = 0.132  # top of the shoulder, base of the neck
NECK_R = 0.0145         # neck outer radius (under the threads)
NECK_TOP_Z = 0.150      # top rim of the neck (cap mounts here)
CAP_R = 0.0185          # cap outer radius (slips over the neck threads)
CAP_HEIGHT = 0.024      # cap height


# Cap-local Z layout: the rotate joint sits at z=NECK_TOP_Z. The cap skirt
# hangs DOWN past its own origin to wrap the neck (intentional seated overlap).
SKIRT_DROP = 0.012      # how far the skirt hangs below the cap origin
CAP_TOP_Z = CAP_HEIGHT - SKIRT_DROP  # cap-local top of the disc


def _neck_thread_profile():
    # Sawtooth ridge segments along the neck, expressed as profile points so the
    # threads are part of the single revolved solid (no fragile thin-solid
    # booleans). Returns a list of (radius, z) points from bottom to top of the
    # neck, ramping out and back for each ridge.
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
    # Transparent thin-wall bottle as one solid revolve, then shelled open at
    # the top so the neck reads hollow. The threaded neck ridges are part of the
    # same outer profile, so the whole bottle is one connected solid.
    wp = (
        cq.Workplane("XZ")
        .moveTo(0.0, BASE_Z)
        # rounded base corner
        .lineTo(BODY_R - 0.006, BASE_Z)
        .threePointArc((BODY_R, BASE_Z + 0.006), (BODY_R, BASE_Z + 0.012))
        # straight cylindrical barrel
        .lineTo(BODY_R, BARREL_TOP_Z)
        # shoulder taper up to the neck
        .threePointArc(
            ((BODY_R + NECK_R) / 2.0, (BARREL_TOP_Z + SHOULDER_TOP_Z) / 2.0 + 0.004),
            (NECK_R, SHOULDER_TOP_Z),
        )
    )
    # ridged neck (threads baked into the outline)
    for (r, z) in _neck_thread_profile()[1:]:
        wp = wp.lineTo(r, z)
    # close back along the axis
    wp = wp.lineTo(0.0, NECK_TOP_Z).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    # Hollow it: remove the top neck face and shell inward.
    return outer.faces(">Z").shell(-WALL)


def _cap_solid():
    # Black ribbed screw cap cup. Local frame: origin at the cap joint; the
    # solid disc sits above (0..CAP_TOP_Z) and the skirt hangs down to
    # z=-SKIRT_DROP, wrapping the neck.
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -SKIRT_DROP))
        .circle(CAP_R)
        .extrude(CAP_HEIGHT)
    )
    outer = outer.edges(">Z").fillet(0.0025)
    # hollow underside: cavity slips over the neck (open at the bottom)
    cavity = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -SKIRT_DROP - 0.001))
        .circle(NECK_R + 0.0016)
        .extrude(CAP_HEIGHT - 0.004)
    )
    cap = outer.cut(cavity)
    # Vertical knurl ribs around the skirt, fused onto the cap so it stays one
    # connected solid (the "ribbed" look).
    n = 24
    rib_h = CAP_HEIGHT - 0.004
    zc = -SKIRT_DROP + rib_h / 2.0
    for i in range(n):
        ang = 2.0 * math.pi * i / n
        x = (CAP_R - 0.0006) * math.cos(ang)
        y = (CAP_R - 0.0006) * math.sin(ang)
        rib = (
            cq.Workplane("XY")
            .transformed(offset=(x, y, zc), rotate=(0, 0, math.degrees(ang)))
            .box(0.0018, 0.0014, rib_h)
        )
        cap = cap.union(rib)
    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="juice_bottle")

    # Tinted-transparent clear PET (alpha ~0.25) and an opaque black cap.
    clear = model.material("clear_pet", rgba=(0.80, 0.86, 0.84, 0.25))
    black = model.material("cap_black", rgba=(0.06, 0.06, 0.07, 1.0))

    # ---- bottle body (root) ----
    body = model.part("bottle")
    shell = _bottle_shell()
    body.visual(mesh_from_cadquery(shell, "bottle_shell"), material=clear, name="bottle_shell")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.030,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- massless carrier (no visuals): carries the rotate joint ----
    carrier = model.part("cap_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.008, 0.008, 0.008)), mass=1e-4)

    # ---- black ribbed screw cap ----
    cap = model.part("cap")
    cap_geo = _cap_solid()
    cap.visual(mesh_from_cadquery(cap_geo, "cap_shell"), material=black, name="cap_shell")
    # Off-axis marker rib so the spin is detectable (a small tab on the skirt).
    cap.visual(
        Box((0.004, 0.006, 0.012)),
        origin=Origin(xyz=(CAP_R + 0.0015, 0.0, -SKIRT_DROP + 0.006)),
        material=black,
        name="cap_marker",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_HEIGHT),
        mass=0.006,
        origin=Origin(xyz=(0.0, 0.0, (CAP_TOP_Z - SKIRT_DROP) / 2.0)),
    )

    # ---- decoupled joints sharing the vertical +Z cap axis ----
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
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=CAP_HEIGHT, effort=1.0, velocity=1.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle")
    cap = object_model.get_part("cap")
    rotate = object_model.get_articulation("cap_rotate")
    slide = object_model.get_articulation("cap_slide")

    bottle_shell = body.get_visual("bottle_shell")
    cap_shell = cap.get_visual("cap_shell")

    # --- bottle is clear (alpha < 1), cap is opaque black ---
    ctx.check(
        "bottle material is tinted-transparent (alpha < 1)",
        bottle_shell.material.rgba is not None and bottle_shell.material.rgba[3] < 1.0,
        details=f"bottle rgba={bottle_shell.material.rgba}",
    )
    ctx.check(
        "cap material is opaque black",
        cap_shell.material.rgba is not None
        and cap_shell.material.rgba[3] >= 0.99
        and max(cap_shell.material.rgba[:3]) < 0.2,
        details=f"cap rgba={cap_shell.material.rgba}",
    )

    # --- cap sits on top of the neck ---
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap mounted on top of the neck",
        cap_pos is not None and cap_pos[2] > BARREL_TOP_Z,
        details=f"cap origin={cap_pos}",
    )

    # The cap skirt slips over the neck threads at rest -> intentional overlap.
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_shell",
        elem_b="bottle_shell",
        reason="Cap skirt is intentionally seated over the threaded neck.",
    )

    # --- rotating cap_rotate spins the cap: the off-axis marker moves ---
    marker0 = None
    marker90 = None
    with ctx.pose({rotate: 0.0}):
        marker0 = ctx.part_world_aabb(cap)
    with ctx.pose({rotate: math.pi / 2.0}):
        marker90 = ctx.part_world_aabb(cap)
    # The asymmetric marker rib swaps the bounding extents from X to Y on a
    # quarter turn.
    e0 = _ext(marker0)
    e90 = _ext(marker90)
    ctx.check(
        "cap rotation moves the off-axis marker (extents swap x<->y)",
        abs(e0[0] - e90[1]) < 0.002 and abs(e0[0] - e0[1]) > 0.0015,
        details=f"rest extents={e0}, quarter-turn extents={e90}",
    )

    # --- sliding cap_slide lifts the cap off the neck ---
    rest_z = ctx.part_world_position(cap)[2]
    with ctx.pose({slide: CAP_HEIGHT}):
        lifted_z = ctx.part_world_position(cap)[2]
    ctx.check(
        "cap_slide lifts the cap up off the neck",
        lifted_z > rest_z + CAP_HEIGHT * 0.8,
        details=f"rest_z={rest_z}, lifted_z={lifted_z}",
    )

    return ctx.report()


object_model = build_object_model()
