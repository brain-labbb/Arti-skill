from __future__ import annotations

# Squat square-shouldered juice bottle with safety collar and volume bands.
# Frame: bottle axis along +Z, base at z=0, neck/cap at the top (+Z).
# The body is a transparent thin-wall PET shell: square barrel with filleted
# corners + flat square shoulder + cylindrical neck with thread ridges + 3
# molded volume bands around the barrel.
# A safety collar ring sits around the neck base on the shoulder, with its own
# REVOLUTE joint (tears/rotates around the neck).
# The black ribbed cap rides on the neck through decoupled joints via a carrier:
#   - cap_rotate: CONTINUOUS spin about +Z
#   - cap_slide:  PRISMATIC lift up off the neck

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
BODY_W = 0.068          # square body width
CORNER_R = 0.009        # corner fillet radius
WALL = 0.0016           # thin PET wall thickness
BARREL_H = 0.078        # barrel height (squat)
SHOULDER_H = 0.005      # shoulder plate thickness
NECK_R = 0.014          # neck outer radius
NECK_H = 0.020          # neck height

BARREL_TOP_Z = BARREL_H                         # 0.078
SHOULDER_TOP_Z = BARREL_H + SHOULDER_H          # 0.083
NECK_TOP_Z = SHOULDER_TOP_Z + NECK_H            # 0.103

# Volume bands (3 molded ridges around the barrel)
BAND_WIDTH = 0.0012     # band protrusion from body surface
BAND_HEIGHT = 0.003     # band vertical thickness
BAND_ZS = [0.020, 0.040, 0.060]

# Cap
CAP_R = 0.0185          # cap outer radius
CAP_HEIGHT = 0.024      # cap total height
SKIRT_DROP = 0.012      # how far the skirt hangs below the cap origin
CAP_TOP_Z = CAP_HEIGHT - SKIRT_DROP

# Safety collar (inner clips around the neck, intentional small overlap)
COLLAR_INNER_R = NECK_R - 0.0005   # clips around the neck body
COLLAR_OUTER_R = NECK_R + 0.006
COLLAR_H = 0.005


def _bottle_shell():
    """Squat square-shouldered hollow bottle with volume bands and thread ridges."""
    # Square barrel with filleted vertical edges and rounded base
    barrel = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_W, BARREL_H, centered=(True, True, False))
    )
    barrel = barrel.edges("|Z").fillet(CORNER_R)
    barrel = barrel.edges("<Z").fillet(0.004)

    # Flat square shoulder plate
    shoulder = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, BARREL_TOP_Z))
        .box(BODY_W, BODY_W, SHOULDER_H, centered=(True, True, False))
    )
    shoulder = shoulder.edges("|Z").fillet(CORNER_R)

    # Round neck cylinder
    neck = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, SHOULDER_TOP_Z))
        .circle(NECK_R)
        .extrude(NECK_H)
    )

    outer = barrel.union(shoulder).union(neck)

    # Molded volume bands: raised ridges around the barrel
    bw = BODY_W + 2 * BAND_WIDTH
    for z in BAND_ZS:
        band = (
            cq.Workplane("XY")
            .transformed(offset=(0, 0, z))
            .box(bw, bw, BAND_HEIGHT, centered=(True, True, False))
        )
        band = band.edges("|Z").fillet(CORNER_R + BAND_WIDTH)
        outer = outer.union(band)

    # Thread ridges on the neck (3 small torus-like rings)
    ridge_outer = NECK_R + 0.0015
    ridge_inner = NECK_R - 0.0005
    for k in range(3):
        zc = SHOULDER_TOP_Z + 0.007 + k * 0.005
        ridge = (
            cq.Workplane("XY")
            .transformed(offset=(0, 0, zc))
            .circle(ridge_outer)
            .circle(ridge_inner)
            .extrude(0.002)
        )
        outer = outer.union(ridge)

    # Inner cavity: hollow interior open at the neck top.
    # The barrel cavity hollows the square body. The neck cavity bores a round
    # hole up through the shoulder plate and neck, leaving the shoulder plate
    # solid around the neck base so the body stays one connected solid.
    iw = BODY_W - 2 * WALL
    ir = max(CORNER_R - WALL, 0.002)
    cavity_barrel = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, WALL))
        .box(iw, iw, BARREL_H - WALL, centered=(True, True, False))
    )
    cavity_barrel = cavity_barrel.edges("|Z").fillet(ir)

    cavity_neck = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, BARREL_H))
        .circle(NECK_R - WALL)
        .extrude(SHOULDER_H + NECK_H + 0.003)
    )

    cavity = cavity_barrel.union(cavity_neck)

    return outer.cut(cavity)


def _collar_solid():
    """Safety collar: ring with indicator tab. Local origin at ring bottom."""
    ring = (
        cq.Workplane("XY")
        .circle(COLLAR_OUTER_R)
        .circle(COLLAR_INNER_R)
        .extrude(COLLAR_H)
    )
    # Indicator tab extending radially outward (makes rotation visible)
    tab = (
        cq.Workplane("XY")
        .transformed(offset=(COLLAR_OUTER_R + 0.003, 0, COLLAR_H * 0.5))
        .box(0.006, 0.004, COLLAR_H * 0.8)
    )
    return ring.union(tab)


def _cap_solid():
    """Black ribbed screw cap. Local origin at the cap joint (neck top)."""
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -SKIRT_DROP))
        .circle(CAP_R)
        .extrude(CAP_HEIGHT)
    )
    outer = outer.edges(">Z").fillet(0.0025)

    # Hollow cavity for neck engagement (tight fit on thread ridges)
    cavity = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -SKIRT_DROP - 0.001))
        .circle(NECK_R + 0.0005)
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
    model = ArticulatedObject(name="squat_juice_bottle")

    # Materials
    clear = model.material("clear_pet", rgba=(0.80, 0.86, 0.84, 0.25))
    black = model.material("cap_black", rgba=(0.06, 0.06, 0.07, 1.0))
    collar_mat = model.material("collar_dark", rgba=(0.10, 0.10, 0.11, 1.0))

    # ---- bottle body (root) ----
    body = model.part("bottle")
    shell = _bottle_shell()
    body.visual(
        mesh_from_cadquery(shell, "bottle_shell"),
        material=clear,
        name="bottle_shell",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_W, NECK_TOP_Z)),
        mass=0.035,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- safety collar ----
    collar = model.part("collar")
    collar_geo = _collar_solid()
    collar.visual(
        mesh_from_cadquery(collar_geo, "collar_ring"),
        material=collar_mat,
        name="collar_ring",
    )
    collar.inertial = Inertial.from_geometry(
        Cylinder(COLLAR_OUTER_R, COLLAR_H),
        mass=0.002,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_H / 2.0)),
    )

    # ---- cap carrier (massless, no visuals) ----
    carrier = model.part("cap_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.008, 0.008, 0.008)), mass=1e-4)

    # ---- black ribbed screw cap ----
    cap = model.part("cap")
    cap_geo = _cap_solid()
    cap.visual(
        mesh_from_cadquery(cap_geo, "cap_shell"),
        material=black,
        name="cap_shell",
    )
    # Off-axis marker rib for detecting cap rotation
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

    # ---- articulations ----

    # Safety collar rotates around the neck base (tears/rotates)
    model.articulation(
        "collar_rotate",
        ArticulationType.REVOLUTE,
        parent=body,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=4.0, effort=1.0, velocity=2.0,
        ),
    )

    # Cap rotates continuously about the bottle axis
    model.articulation(
        "cap_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    # Cap slides up off the neck
    model.articulation(
        "cap_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=CAP_HEIGHT, effort=1.0, velocity=1.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle")
    collar = object_model.get_part("collar")
    cap = object_model.get_part("cap")
    collar_joint = object_model.get_articulation("collar_rotate")
    cap_rotate = object_model.get_articulation("cap_rotate")
    cap_slide = object_model.get_articulation("cap_slide")

    bottle_shell = body.get_visual("bottle_shell")
    cap_shell = cap.get_visual("cap_shell")

    # --- material checks ---
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

    # --- square body with volume bands ---
    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is not None:
        mn, mx = body_aabb
        dx = mx[0] - mn[0]
        dy = mx[1] - mn[1]
        expected_w = BODY_W + 2 * BAND_WIDTH
        ctx.check(
            "body has square cross-section (x ≈ y)",
            abs(dx - dy) < 0.004,
            details=f"dx={dx:.4f}, dy={dy:.4f}",
        )
        ctx.check(
            "molded volume bands protrude beyond body width",
            dx > BODY_W + 0.0005,
            details=f"body x-width={dx:.4f}, expected>{BODY_W + 0.0005:.4f}",
        )
        ctx.check(
            "body width matches expected (body + bands)",
            abs(dx - expected_w) < 0.003,
            details=f"dx={dx:.4f}, expected={expected_w:.4f}",
        )

    # --- squat proportion ---
    if body_aabb is not None:
        mn, mx = body_aabb
        dz = mx[2] - mn[2]
        ctx.check(
            "bottle has squat proportion (height reasonable)",
            0.085 < dz < 0.135,
            details=f"total height={dz:.4f}",
        )

    # --- safety collar positioned at neck base ---
    collar_pos = ctx.part_world_position(collar)
    ctx.check(
        "safety collar positioned at neck base",
        collar_pos is not None and abs(collar_pos[2] - SHOULDER_TOP_Z) < 0.006,
        details=f"collar z={collar_pos[2] if collar_pos else None}, expected≈{SHOULDER_TOP_Z}",
    )

    # --- collar rotates around neck (tab moves) ---
    with ctx.pose({collar_joint: 0.0}):
        collar_aabb_0 = ctx.part_world_aabb(collar)
    with ctx.pose({collar_joint: math.pi / 2.0}):
        collar_aabb_90 = ctx.part_world_aabb(collar)

    if collar_aabb_0 is not None and collar_aabb_90 is not None:
        mn0, mx0 = collar_aabb_0
        mn90, mx90 = collar_aabb_90
        dx0 = mx0[0] - mn0[0]
        dy0 = mx0[1] - mn0[1]
        dx90 = mx90[0] - mn90[0]
        dy90 = mx90[1] - mn90[1]
        # Tab at +X at rest → x-extent larger; after 90° → y-extent larger
        ctx.check(
            "collar rotation moves the indicator tab (extents swap)",
            abs(dx0 - dy90) < 0.004 and abs(dx0 - dx90) > 0.001,
            details=f"0°: dx={dx0:.4f} dy={dy0:.4f}, 90°: dx={dx90:.4f} dy={dy90:.4f}",
        )

    # --- collar_rotate is revolute (non-fixed) ---
    ctx.check(
        "collar_rotate is a revolute joint",
        collar_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={collar_joint.articulation_type}",
    )

    # --- cap sits on top of the neck ---
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap mounted on top of the neck",
        cap_pos is not None and cap_pos[2] > BARREL_TOP_Z,
        details=f"cap origin={cap_pos}",
    )

    # Cap skirt overlaps the neck threads (intentional seated overlap)
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_shell",
        elem_b="bottle_shell",
        reason="Cap skirt cavity engages the neck thread ridges for a seated screw fit.",
    )

    # Safety collar clips around the neck body (intentional small overlap)
    ctx.allow_overlap(
        collar,
        body,
        elem_a="collar_ring",
        elem_b="bottle_shell",
        reason="Safety collar ring clips around the neck body, retained below the thread ridges.",
    )
    # Proof: collar overlaps the bottle in XY (surrounds the neck)
    ctx.expect_overlap(
        collar,
        body,
        axes="xy",
        min_overlap=0.002,
        elem_a="collar_ring",
        elem_b="bottle_shell",
        name="collar ring surrounds the neck in XY",
    )

    # --- cap rotation moves the marker ---
    with ctx.pose({cap_rotate: 0.0}):
        marker0 = ctx.part_world_aabb(cap)
    with ctx.pose({cap_rotate: math.pi / 2.0}):
        marker90 = ctx.part_world_aabb(cap)

    if marker0 is not None and marker90 is not None:
        e0x = marker0[1][0] - marker0[0][0]
        e0y = marker0[1][1] - marker0[0][1]
        e90x = marker90[1][0] - marker90[0][0]
        e90y = marker90[1][1] - marker90[0][1]
        ctx.check(
            "cap rotation moves the off-axis marker (extents swap x↔y)",
            abs(e0x - e90y) < 0.003 and abs(e0x - e0y) > 0.001,
            details=f"rest=({e0x:.4f},{e0y:.4f}), 90°=({e90x:.4f},{e90y:.4f})",
        )

    # --- cap slide lifts the cap off the neck ---
    rest_z = ctx.part_world_position(cap)[2]
    with ctx.pose({cap_slide: CAP_HEIGHT}):
        lifted_z = ctx.part_world_position(cap)[2]
    ctx.check(
        "cap_slide lifts the cap up off the neck",
        lifted_z > rest_z + CAP_HEIGHT * 0.8,
        details=f"rest_z={rest_z}, lifted_z={lifted_z}",
    )

    return ctx.report()


object_model = build_object_model()
