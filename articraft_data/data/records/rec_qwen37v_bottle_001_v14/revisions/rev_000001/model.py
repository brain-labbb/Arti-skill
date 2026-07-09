from __future__ import annotations

# Swing-top bottle for soap/lotion: clear body with molded volume bands,
# raised spiral-like neck threads, and a swing-top stopper that pivots on
# side hinge arms.
# Frame: bottle axis along +Z, base at z=0, neck/stopper at top (+Z).

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
BODY_R = 0.030          # outer barrel radius (~60 mm dia)
WALL = 0.0016           # thin PET/HDPE wall
BASE_Z = 0.0
BARREL_TOP_Z = 0.140    # where shoulder taper begins
SHOULDER_TOP_Z = 0.168  # top of shoulder / neck base
NECK_R = 0.012          # neck outer radius (under threads)
NECK_TOP_Z = 0.188      # top rim of neck

# Volume bands (molded ridges around the barrel)
BAND_ZS = [0.040, 0.075, 0.110]
BAND_BULGE = 0.0013     # outward protrusion from barrel surface
BAND_H = 0.005          # band height along Z

# Swing mechanism: collar + pivot lugs on bottle, bail arm + stopper on swing part
COLLAR_Z = SHOULDER_TOP_Z          # collar starts at shoulder/neck junction
COLLAR_H = 0.008                   # collar ring height
COLLAR_OR = NECK_R + 0.005         # collar outer radius = 0.017
PIVOT_X = COLLAR_OR                # pivot pin at collar outer edge = 0.017
PIVOT_Z = COLLAR_Z + COLLAR_H * 0.5  # pivot at collar mid-height = 0.172

# Bail arm (local frame: origin at pivot, arms go up in +Z)
WIRE_R = 0.0018         # wire cross-section radius
ARM_H = 0.030           # vertical arm length
ARM_DY = 0.012          # Y-spacing between the two parallel arms

# Stopper
STOP_R = NECK_R + 0.001  # 0.013, slightly wider than neck
STOP_H = 0.006           # stopper disc thickness


def _bottle_shell():
    """Hollow bottle: rounded base, banded barrel, shoulder, threaded neck."""
    wp = cq.Workplane("XZ")
    wp = wp.moveTo(0.0, BASE_Z)
    # Rounded base corner
    wp = wp.lineTo(BODY_R - 0.006, BASE_Z)
    wp = wp.threePointArc((BODY_R, BASE_Z + 0.006), (BODY_R, BASE_Z + 0.012))

    # Barrel with volume bands
    prev_z = BASE_Z + 0.012
    for bz in BAND_ZS:
        lo = bz - BAND_H / 2.0
        hi = bz + BAND_H / 2.0
        if lo > prev_z + 0.0001:
            wp = wp.lineTo(BODY_R, lo)
        wp = wp.lineTo(BODY_R + BAND_BULGE, bz - BAND_H * 0.3)
        wp = wp.lineTo(BODY_R + BAND_BULGE, bz + BAND_H * 0.3)
        wp = wp.lineTo(BODY_R, hi)
        prev_z = hi

    # Continue barrel to shoulder start
    wp = wp.lineTo(BODY_R, BARREL_TOP_Z)

    # Shoulder taper
    wp = wp.threePointArc(
        ((BODY_R + NECK_R) / 2.0, (BARREL_TOP_Z + SHOULDER_TOP_Z) / 2.0 + 0.003),
        (NECK_R, SHOULDER_TOP_Z),
    )

    # Threaded neck (raised spiral-like ridges baked into the profile)
    z0 = SHOULDER_TOP_Z + 0.003
    ridge_r = NECK_R + 0.0015
    for k in range(4):
        zc = z0 + k * 0.004
        wp = wp.lineTo(NECK_R, zc - 0.0014)
        wp = wp.lineTo(ridge_r, zc - 0.0004)
        wp = wp.lineTo(ridge_r, zc + 0.0004)
        wp = wp.lineTo(NECK_R, zc + 0.0014)

    wp = wp.lineTo(NECK_R, NECK_TOP_Z)
    # Close along axis
    wp = wp.lineTo(0.0, NECK_TOP_Z).close()

    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    # Shell open at the top so the neck reads hollow
    return outer.faces(">Z").shell(-WALL)


def _collar():
    """Collar ring with a pivot lug on the +X side, fused as one solid."""
    ring = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, COLLAR_Z))
        .circle(COLLAR_OR)
        .circle(NECK_R - 0.0005)   # wraps the neck (intentional seated overlap)
        .extrude(COLLAR_H)
    )
    # Pivot lug on +X side: a small block that the bail arms pass through
    lug_w_y = ARM_DY + 0.006       # wide enough to clear both arms
    lug_w_x = 0.006
    lug = (
        cq.Workplane("XY")
        .transformed(offset=(PIVOT_X, 0, PIVOT_Z))
        .box(lug_w_x, lug_w_y, COLLAR_H)
    )
    return ring.union(lug)


def _swing_bail():
    """Wire bail with stopper. Local frame origin = pivot point.
    At q=0 the arms go straight up (+Z local) and the bridge/stopper
    reach toward -X (bottle center)."""
    # Stopper position in local frame (at q=0 these land on the neck top)
    stop_x = -PIVOT_X                     # = -0.017
    stop_z = NECK_TOP_Z - PIVOT_Z        # = 0.016

    # Two parallel vertical wire arms
    arm1 = (
        cq.Workplane("XY")
        .transformed(offset=(0, ARM_DY / 2.0, 0))
        .circle(WIRE_R)
        .extrude(ARM_H)
    )
    arm2 = (
        cq.Workplane("XY")
        .transformed(offset=(0, -ARM_DY / 2.0, 0))
        .circle(WIRE_R)
        .extrude(ARM_H)
    )
    bail = arm1.union(arm2)

    # Horizontal bridge at top connecting the two arms and reaching toward -X
    bridge_len = abs(stop_x) + 0.004
    bridge_cx = stop_x / 2.0
    bridge = (
        cq.Workplane("XY")
        .transformed(offset=(bridge_cx, 0, ARM_H - WIRE_R * 1.5))
        .box(bridge_len, ARM_DY, WIRE_R * 3.0)
    )
    bail = bail.union(bridge)

    # Vertical stub from bridge down to stopper
    stub_top = ARM_H - WIRE_R * 1.5
    stub_bot = stop_z + STOP_H
    stub_h = stub_top - stub_bot
    if stub_h > 0.001:
        stub = (
            cq.Workplane("XY")
            .transformed(offset=(stop_x, 0, stub_bot))
            .circle(WIRE_R * 1.4)
            .extrude(stub_h)
        )
        bail = bail.union(stub)

    # Stopper disc at the bottom of the stub
    stopper = (
        cq.Workplane("XY")
        .transformed(offset=(stop_x, 0, stop_z))
        .circle(STOP_R)
        .extrude(STOP_H)
    )
    bail = bail.union(stopper)

    return bail


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_top_bottle")

    clear = model.material("clear_body", rgba=(0.78, 0.85, 0.82, 0.30))
    metal = model.material("steel_wire", rgba=(0.52, 0.54, 0.52, 1.0))
    cream = model.material("ceramic_stop", rgba=(0.93, 0.91, 0.86, 1.0))

    # ---- bottle body (root) ----
    body = model.part("bottle")
    shell = _bottle_shell()
    collar = _collar()
    body.visual(
        mesh_from_cadquery(shell, "bottle_shell"),
        material=clear,
        name="bottle_shell",
    )
    body.visual(
        mesh_from_cadquery(collar, "collar_ring"),
        material=metal,
        name="collar_ring",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.040,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- swing arm (bail + stopper) ----
    arm = model.part("swing_arm")
    bail = _swing_bail()
    arm.visual(
        mesh_from_cadquery(bail, "bail_assembly"),
        material=metal,
        name="bail_assembly",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.04, 0.02, 0.04)),
        mass=0.010,
        origin=Origin(xyz=(-0.01, 0.0, 0.015)),
    )

    # ---- swing joint ----
    # REVOLUTE at the pivot pin on the +X lug.
    # axis = +Y: right-hand rule rotates local +Z toward +X, so positive q
    # swings the arm (and stopper) away from the bottle center (opens).
    # At q=0 the stopper sits on the neck rim (closed).
    model.articulation(
        "cap_swing",
        ArticulationType.REVOLUTE,
        parent=body,
        child=arm,
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=2.4, effort=2.0, velocity=2.0
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle")
    arm = object_model.get_part("swing_arm")
    swing = object_model.get_articulation("cap_swing")

    bottle_shell = body.get_visual("bottle_shell")
    collar_ring = body.get_visual("collar_ring")
    bail = arm.get_visual("bail_assembly")

    # --- material checks ---
    ctx.check(
        "bottle body is translucent (alpha < 1)",
        bottle_shell.material.rgba is not None and bottle_shell.material.rgba[3] < 1.0,
        details=f"rgba={bottle_shell.material.rgba}",
    )
    ctx.check(
        "collar ring is opaque metal",
        collar_ring.material.rgba is not None and collar_ring.material.rgba[3] >= 0.99,
        details=f"rgba={collar_ring.material.rgba}",
    )

    # --- joint type and limits ---
    ctx.check(
        "cap_swing is REVOLUTE",
        swing.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={swing.articulation_type}",
    )
    limits = swing.motion_limits
    ctx.check(
        "cap_swing has finite lower/upper limits",
        limits is not None and limits.lower is not None and limits.upper is not None
        and limits.upper > limits.lower,
        details=f"lower={limits.lower}, upper={limits.upper}",
    )
    ctx.check(
        "cap_swing range is at least 1.5 rad (swing mechanism)",
        limits is not None and limits.upper is not None and limits.lower is not None
        and (limits.upper - limits.lower) >= 1.5,
        details=f"range={limits.upper - limits.lower}",
    )

    # --- closed pose: bail geometry spans the neck area ---
    closed_aabb = None
    with ctx.pose({swing: 0.0}):
        closed_aabb = ctx.part_world_aabb(arm)
    ctx.check(
        "swing arm geometry exists at closed pose",
        closed_aabb is not None,
    )
    if closed_aabb is not None:
        cmn, cmx = closed_aabb
        ctx.check(
            "closed bail top reaches above the neck rim",
            cmx[2] > NECK_TOP_Z,
            details=f"closed max z={cmx[2]}, neck top={NECK_TOP_Z}",
        )
        # The stopper center should be near x=0; the bail AABB is offset
        # by the arms on the +X side and stopper overhang on -X.
        bail_center_x = (cmn[0] + cmx[0]) / 2.0
        ctx.check(
            "closed bail spans the neck region in X",
            cmn[0] < 0.002 and cmx[0] > -0.002,
            details=f"closed min_x={cmn[0]}, max_x={cmx[0]}",
        )

    # --- open pose: bail geometry swings away from the neck ---
    upper = limits.upper if limits is not None and limits.upper is not None else 2.0
    open_aabb = None
    with ctx.pose({swing: upper}):
        open_aabb = ctx.part_world_aabb(arm)
    if closed_aabb is not None and open_aabb is not None:
        cmn, cmx = closed_aabb
        omn, omx = open_aabb
        # The stopper end (min X at closed) should move to the +X side when open
        ctx.check(
            "open pose swings the stopper away from bottle center",
            omn[0] > cmn[0] + 0.005,
            details=f"closed min_x={cmn[0]}, open min_x={omn[0]}",
        )

    # --- volume bands: bottle barrel shows outward protrusion ---
    # The bottle shell should have a wider extent than BODY_R*2 on at least one
    # horizontal axis (the bands protrude outward).
    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is not None:
        mn, mx = body_aabb
        dx = mx[0] - mn[0]
        dy = mx[1] - mn[1]
        band_extent = max(dx, dy)
        ctx.check(
            "volume bands protrude beyond barrel radius",
            band_extent > 2.0 * BODY_R + BAND_BULGE * 0.5,
            details=f"max horizontal extent={band_extent}, expected>{2*BODY_R + BAND_BULGE*0.5}",
        )

    # --- bottle height is taller than original juice bottle ---
    if body_aabb is not None:
        mn, mx = body_aabb
        bottle_height = mx[2] - mn[2]
        ctx.check(
            "bottle height reaches neck top (taller soap/lotion bottle)",
            bottle_height > NECK_TOP_Z * 0.95,
            details=f"height={bottle_height}",
        )

    # --- intentional overlaps ---
    # Collar wraps the neck (seated ring)
    ctx.allow_overlap(
        body, body,
        elem_a="collar_ring",
        elem_b="bottle_shell",
        reason="Collar ring wraps the neck for a seated clamp fit.",
    )
    # Bail arms pass through the pivot lug (pivot capture)
    ctx.allow_overlap(
        body, arm,
        elem_a="collar_ring",
        elem_b="bail_assembly",
        reason="Bail arms are captured by the pivot lug to form the hinge.",
    )
    # Stopper seats on the neck rim at closed pose
    ctx.allow_overlap(
        body, arm,
        elem_a="bottle_shell",
        elem_b="bail_assembly",
        reason="Stopper disc seats against the neck rim when closed.",
    )

    # Proof: at closed pose, bail overlaps the neck region in Z (stopper seated)
    with ctx.pose({swing: 0.0}):
        ctx.expect_overlap(
            arm, body,
            axes="z",
            elem_a="bail_assembly",
            elem_b="bottle_shell",
            min_overlap=STOP_H * 0.5,
            name="closed stopper overlaps the neck region in Z",
        )
        # And bail overlaps the neck in XY (stopper is above the neck opening)
        ctx.expect_overlap(
            arm, body,
            axes="xy",
            elem_a="bail_assembly",
            elem_b="bottle_shell",
            min_overlap=0.002,
            name="closed stopper centered over neck opening in XY",
        )

    return ctx.report()


object_model = build_object_model()
