from __future__ import annotations

# Tall cylindrical clear plastic juice bottle with a black ribbed screw cap,
# a separate gasket ring, and a visible hollow mouth opening.
#
# Frame: bottle axis along +Z, base at z=0, neck/cap at the top (+Z).
# Body: tall thin-wall PET shell — cylindrical barrel, shoulder taper, narrow
#       threaded neck with a visible hollow bore at the mouth.
# Gasket: a separate rubber seal ring sitting on the neck rim below the cap.
# Cap carrier: massless link that decouples rotation from lift.
# Cap: black ribbed screw cap with two independent joints:
#   - cap_rotate: CONTINUOUS spin about +Z
#   - cap_slide:  PRISMATIC lift along +Z

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
BODY_R = 0.032          # outer barrel radius (~64 mm dia)
WALL = 0.0016           # thin PET wall thickness
BASE_Z = 0.0            # bottom of the bottle
BARREL_TOP_Z = 0.185    # where the shoulder taper begins (tall bottle)
SHOULDER_TOP_Z = 0.210  # top of shoulder, base of the neck
NECK_R = 0.013          # narrow neck outer radius (under the threads)
NECK_TOP_Z = 0.240      # top rim of the neck (mouth opening)
MOUTH_R = 0.010         # inner bore radius of the mouth opening
CAP_R = 0.017           # cap outer radius
CAP_HEIGHT = 0.022      # total cap height

# Gasket ring dimensions
GASKET_IR = 0.009       # gasket inner radius (inside the neck bore lip)
GASKET_OR = 0.016       # gasket outer radius (slightly wider than neck)
GASKET_H = 0.003        # gasket thickness (height)
GASKET_Z = NECK_TOP_Z   # gasket sits on top of the neck rim

# Cap-local Z layout: the rotate joint sits at z=NECK_TOP_Z + GASKET_H (on
# top of the gasket). The cap skirt hangs DOWN past its origin to wrap the
# neck + gasket (intentional seated overlap).
SKIRT_DROP = 0.012      # how far the skirt hangs below the cap origin
CAP_TOP_LOCAL = CAP_HEIGHT - SKIRT_DROP  # cap-local top of the disc


def _neck_thread_profile():
    """Sawtooth thread ridges on the narrow neck as profile points."""
    pts = [(NECK_R, SHOULDER_TOP_Z)]
    z0 = SHOULDER_TOP_Z + 0.004
    ridge_r = NECK_R + 0.0014
    for k in range(3):
        zc = z0 + k * 0.0048
        pts.append((NECK_R, zc - 0.0016))
        pts.append((ridge_r, zc - 0.0006))
        pts.append((ridge_r, zc + 0.0006))
        pts.append((NECK_R, zc + 0.0016))
    pts.append((NECK_R, NECK_TOP_Z))
    return pts


def _bottle_shell():
    """Transparent thin-wall tall bottle as one revolved solid, shelled open
    at the top so the narrow neck reads as a visible hollow mouth."""
    wp = (
        cq.Workplane("XZ")
        .moveTo(0.0, BASE_Z)
        # rounded base corner
        .lineTo(BODY_R - 0.006, BASE_Z)
        .threePointArc((BODY_R, BASE_Z + 0.006), (BODY_R, BASE_Z + 0.012))
        # straight tall cylindrical barrel
        .lineTo(BODY_R, BARREL_TOP_Z)
        # shoulder taper to the narrow neck
        .threePointArc(
            ((BODY_R + NECK_R) / 2.0, (BARREL_TOP_Z + SHOULDER_TOP_Z) / 2.0 + 0.004),
            (NECK_R, SHOULDER_TOP_Z),
        )
    )
    # ridged neck threads baked into the outer profile
    for (r, z) in _neck_thread_profile()[1:]:
        wp = wp.lineTo(r, z)
    # close back along the axis at the top of the neck
    wp = wp.lineTo(0.0, NECK_TOP_Z).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    # Shell: remove the top neck face and hollow inward — this opens the
    # mouth bore so the hollow interior is visible through the narrow neck.
    shelled = outer.faces(">Z").shell(-WALL)

    # Add a visible mouth rim/lip ring at the top of the neck so the
    # opening reads clearly. This is a small torus-like ring fused to the
    # neck top.
    rim = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, NECK_TOP_Z - 0.001))
        .circle(NECK_R + 0.002)
        .circle(NECK_R - 0.001)
        .extrude(0.003)
    )
    return shelled.union(rim)


def _gasket_ring():
    """Flat rubber gasket ring (washer). Built in local frame: centered at
    origin, extending from z=0 to z=GASKET_H. The FIXED joint origin places
    it at the correct world height."""
    ring = (
        cq.Workplane("XY")
        .circle(GASKET_OR)
        .circle(GASKET_IR)
        .extrude(GASKET_H)
    )
    return ring


def _cap_solid():
    """Black ribbed screw cap. Local frame: origin at the cap joint (top of
    gasket); disc above, skirt hanging down to wrap neck + gasket."""
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -SKIRT_DROP))
        .circle(CAP_R)
        .extrude(CAP_HEIGHT)
    )
    outer = outer.edges(">Z").fillet(0.0025)
    # hollow underside: cavity slips over neck + gasket (open at bottom)
    cavity = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -SKIRT_DROP - 0.001))
        .circle(NECK_R + 0.0014)
        .extrude(CAP_HEIGHT - 0.004)
    )
    cap = outer.cut(cavity)
    # Vertical knurl ribs around the skirt
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
    model = ArticulatedObject(name="tall_juice_bottle")

    # Materials
    clear = model.material("clear_pet", rgba=(0.80, 0.86, 0.84, 0.25))
    black = model.material("cap_black", rgba=(0.06, 0.06, 0.07, 1.0))
    rubber = model.material("gasket_rubber", rgba=(0.18, 0.18, 0.20, 1.0))

    # ---- bottle body (root) ----
    body = model.part("bottle")
    shell = _bottle_shell()
    body.visual(mesh_from_cadquery(shell, "bottle_shell"), material=clear, name="bottle_shell")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.035,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- gasket ring (fixed to bottle) ----
    gasket = model.part("gasket")
    gasket_geo = _gasket_ring()
    gasket.visual(mesh_from_cadquery(gasket_geo, "gasket_ring"), material=rubber, name="gasket_ring")
    gasket.inertial = Inertial.from_geometry(
        Cylinder(GASKET_OR, GASKET_H),
        mass=0.002,
        origin=Origin(xyz=(0.0, 0.0, GASKET_H / 2.0)),
    )

    # ---- massless carrier (no visuals): decouples rotate from slide ----
    carrier = model.part("cap_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.008, 0.008, 0.008)), mass=1e-4)

    # ---- black ribbed screw cap ----
    cap = model.part("cap")
    cap_geo = _cap_solid()
    cap.visual(mesh_from_cadquery(cap_geo, "cap_shell"), material=black, name="cap_shell")
    # Off-axis marker rib so the spin is detectable
    cap.visual(
        Box((0.004, 0.006, 0.012)),
        origin=Origin(xyz=(CAP_R + 0.0015, 0.0, -SKIRT_DROP + 0.006)),
        material=black,
        name="cap_marker",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_HEIGHT),
        mass=0.006,
        origin=Origin(xyz=(0.0, 0.0, (CAP_TOP_LOCAL - SKIRT_DROP) / 2.0)),
    )

    # ---- articulations ----
    # Gasket is fixed to the bottle body, placed at the neck rim
    model.articulation(
        "gasket_fixed",
        ArticulationType.FIXED,
        parent=body,
        child=gasket,
        origin=Origin(xyz=(0.0, 0.0, GASKET_Z)),
    )

    # Cap rotation: continuous spin about +Z at the top of the gasket
    cap_origin_z = GASKET_Z + GASKET_H
    model.articulation(
        "cap_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, cap_origin_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    # Cap slide: prismatic lift along +Z
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


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle")
    gasket = object_model.get_part("gasket")
    cap = object_model.get_part("cap")
    carrier = object_model.get_part("cap_carrier")
    rotate = object_model.get_articulation("cap_rotate")
    slide = object_model.get_articulation("cap_slide")
    gasket_joint = object_model.get_articulation("gasket_fixed")

    bottle_shell = body.get_visual("bottle_shell")
    cap_shell = cap.get_visual("cap_shell")
    gasket_ring = gasket.get_visual("gasket_ring")

    # --- bottle is clear transparent, cap is opaque black ---
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

    # --- bottle is tall: total height > 0.22 m ---
    bottle_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "bottle is tall (height > 0.22 m)",
        bottle_aabb is not None and (bottle_aabb[1][2] - bottle_aabb[0][2]) > 0.22,
        details=f"bottle AABB={bottle_aabb}",
    )

    # --- neck is clearly narrower than body ---
    # The gasket ring outer radius is a good proxy for neck-region width.
    # Check that gasket XY span is less than bottle XY span.
    gasket_aabb = ctx.part_world_aabb(gasket)
    ctx.check(
        "neck region is narrower than body",
        gasket_aabb is not None and bottle_aabb is not None
        and (gasket_aabb[1][0] - gasket_aabb[0][0]) < (bottle_aabb[1][0] - bottle_aabb[0][0]) * 0.7,
        details=f"gasket dx={gasket_aabb[1][0]-gasket_aabb[0][0]:.4f}, bottle dx={bottle_aabb[1][0]-bottle_aabb[0][0]:.4f}",
    )

    # --- gasket ring exists as a separate part between cap and bottle ---
    gasket_pos = ctx.part_world_position(gasket)
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "gasket sits between bottle top and cap",
        gasket_pos is not None and cap_pos is not None
        and gasket_pos[2] > BARREL_TOP_Z
        and gasket_pos[2] <= cap_pos[2],
        details=f"gasket_z={gasket_pos}, cap_z={cap_pos}",
    )

    # --- gasket is fixed to the bottle (FIXED joint) ---
    ctx.check(
        "gasket_fixed joint is FIXED type",
        gasket_joint.articulation_type == ArticulationType.FIXED,
        details=f"gasket_joint type={gasket_joint.articulation_type}",
    )

    # --- cap sits above the gasket ---
    ctx.check(
        "cap mounted above neck and gasket",
        cap_pos is not None and cap_pos[2] > SHOULDER_TOP_Z,
        details=f"cap origin z={cap_pos[2] if cap_pos else None}",
    )

    # The cap skirt slips over the neck + gasket at rest -> intentional overlap.
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_shell",
        elem_b="bottle_shell",
        reason="Cap skirt is intentionally seated over the threaded neck.",
    )
    ctx.allow_overlap(
        cap,
        gasket,
        elem_a="cap_shell",
        elem_b="gasket_ring",
        reason="Cap compresses the gasket ring when screwed on.",
    )

    # --- gasket ring contacts the bottle neck top ---
    ctx.expect_contact(
        gasket,
        body,
        elem_a="gasket_ring",
        elem_b="bottle_shell",
        name="gasket contacts bottle neck rim",
    )

    # --- cap_rotate is CONTINUOUS (non-fixed joint) ---
    ctx.check(
        "cap_rotate is a CONTINUOUS joint",
        rotate.articulation_type == ArticulationType.CONTINUOUS,
        details=f"rotate type={rotate.articulation_type}",
    )

    # --- cap_slide is PRISMATIC (non-fixed joint) ---
    ctx.check(
        "cap_slide is a PRISMATIC joint",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"slide type={slide.articulation_type}",
    )

    # --- rotating cap spins: off-axis marker moves ---
    with ctx.pose({rotate: 0.0}):
        marker0 = ctx.part_world_aabb(cap)
    with ctx.pose({rotate: math.pi / 2.0}):
        marker90 = ctx.part_world_aabb(cap)
    e0 = (marker0[1][0] - marker0[0][0], marker0[1][1] - marker0[0][1])
    e90 = (marker90[1][0] - marker90[0][0], marker90[1][1] - marker90[0][1])
    ctx.check(
        "cap rotation moves the off-axis marker (extents swap x<->y)",
        abs(e0[0] - e90[1]) < 0.002 and abs(e0[0] - e0[1]) > 0.0015,
        details=f"rest extents={e0}, quarter-turn extents={e90}",
    )

    # --- sliding cap lifts off the neck ---
    rest_z = ctx.part_world_position(cap)[2]
    with ctx.pose({slide: CAP_HEIGHT}):
        lifted_z = ctx.part_world_position(cap)[2]
    ctx.check(
        "cap_slide lifts the cap up off the neck",
        lifted_z > rest_z + CAP_HEIGHT * 0.8,
        details=f"rest_z={rest_z}, lifted_z={lifted_z}",
    )

    # --- hollow mouth opening: when cap is lifted, the bottle top is above
    # the shoulder and the bore region is within the bottle's XY footprint ---
    with ctx.pose({slide: CAP_HEIGHT}):
        # The mouth opening (at NECK_TOP_Z) should be exposed above the gasket
        cap_lifted_aabb = ctx.part_world_aabb(cap)
        bottle_aabb_check = ctx.part_world_aabb(body)
    ctx.check(
        "mouth opening is exposed when cap is lifted",
        cap_lifted_aabb is not None
        and cap_lifted_aabb[0][2] > NECK_TOP_Z + GASKET_H * 0.5,
        details=f"lifted cap min_z={cap_lifted_aabb[0][2] if cap_lifted_aabb else None}",
    )

    return ctx.report()


object_model = build_object_model()
