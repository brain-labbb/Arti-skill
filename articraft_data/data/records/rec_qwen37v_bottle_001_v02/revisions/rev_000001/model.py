from __future__ import annotations

# Squat square-shouldered juice bottle with flip-up straw spout.
# Variant of the clear plastic juice bottle family.
# Frame: bottle axis along +Z, base at z=0, neck/cap at the top (+Z).
# Body: square cross-section with filleted corners, volume bands, shoulder taper,
# threaded neck. Black ribbed cap with a pivoting straw spout.
#
# Joint chain:
#   bottle -> cap_carrier: cap_rotate (CONTINUOUS, +Z)
#   cap_carrier -> cap:    cap_slide   (PRISMATIC, +Z)
#   cap -> spout:          spout_pivot (REVOLUTE, -Y axis)
#     At q=0 the spout lies flat on the cap top; positive q flips it upright.

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
BODY_W = 0.060           # body width and depth (square cross-section)
BODY_H = 0.085           # body height (squat)
BODY_FILLET = 0.008      # vertical-edge fillet radius
WALL = 0.0015            # PET wall thickness

BAND_COUNT = 3           # number of molded volume bands
BAND_WIDTH = 0.003       # height of each band ridge
BAND_PROTRUDE = 0.0012   # how far bands protrude from body surface

SHOULDER_H = 0.020       # shoulder transition height
SHOULDER_TOP_Z = BODY_H + SHOULDER_H  # 0.105

NECK_R = 0.013           # neck outer radius (under threads)
NECK_H = 0.018           # neck height
NECK_TOP_Z = SHOULDER_TOP_Z + NECK_H  # 0.123

CAP_R = 0.017            # cap outer radius
CAP_H = 0.022            # cap total height
SKIRT_DROP = 0.011       # how far skirt hangs below cap origin
CAP_TOP_Z = CAP_H - SKIRT_DROP  # cap-local top of disc

SPOUT_R = 0.003          # spout outer radius
SPOUT_LENGTH = 0.038     # spout straw length
SPOUT_BASE_W = 0.012     # hinge base width (along spout axis)
SPOUT_BASE_D = 0.010     # hinge base depth
SPOUT_BASE_H = 0.007     # hinge base height


def _bottle_solid():
    """Build the full bottle body: square barrel + volume bands + shoulder
    + threaded neck, all as one connected solid."""

    # --- main square body with filleted vertical edges ---
    body = (
        cq.Workplane("XY")
        .rect(BODY_W, BODY_W)
        .extrude(BODY_H)
    )
    body = body.edges("|Z").fillet(BODY_FILLET)
    # Slight fillet on bottom edges for a finished base
    body = body.edges("<Z").fillet(0.002)

    # --- volume bands: thin protruding frames around the body ---
    band_z_positions = [
        BODY_H * 0.28,
        BODY_H * 0.50,
        BODY_H * 0.72,
    ]
    for z in band_z_positions:
        outer_rect = (
            cq.Workplane("XY")
            .transformed(offset=(0, 0, z))
            .rect(BODY_W + 2 * BAND_PROTRUDE, BODY_W + 2 * BAND_PROTRUDE)
            .extrude(BAND_WIDTH)
        )
        inner_rect = (
            cq.Workplane("XY")
            .transformed(offset=(0, 0, z - 0.0001))
            .rect(BODY_W - 0.0002, BODY_W - 0.0002)
            .extrude(BAND_WIDTH + 0.0002)
        )
        band = outer_rect.cut(inner_rect)
        body = body.union(band)

    # --- shoulder: loft from inscribed circle to neck circle ---
    # Use a circle matching the body's inscribed diameter for a clean transition
    shoulder_bottom_r = BODY_W / 2.0 - BODY_FILLET * 0.3
    shoulder = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, BODY_H))
        .circle(shoulder_bottom_r)
        .workplane(offset=SHOULDER_H)
        .circle(NECK_R + 0.002)
        .loft()
    )
    body = body.union(shoulder)

    # --- neck cylinder ---
    neck = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, SHOULDER_TOP_Z))
        .circle(NECK_R)
        .extrude(NECK_H)
    )
    body = body.union(neck)

    # --- neck thread ridges: raised annular rings on the outside ---
    ridge_count = 4
    ridge_protrude = 0.0015
    ridge_thickness = 0.0012
    thread_zone_start = SHOULDER_TOP_Z + 0.003
    thread_zone_pitch = (NECK_H - 0.006) / max(ridge_count - 1, 1)
    for k in range(ridge_count):
        z = thread_zone_start + k * thread_zone_pitch
        ridge_outer = (
            cq.Workplane("XY")
            .transformed(offset=(0, 0, z))
            .circle(NECK_R + ridge_protrude)
            .extrude(ridge_thickness)
        )
        ridge_inner = (
            cq.Workplane("XY")
            .transformed(offset=(0, 0, z - 0.0001))
            .circle(NECK_R - 0.0002)
            .extrude(ridge_thickness + 0.0002)
        )
        ridge = ridge_outer.cut(ridge_inner)
        body = body.union(ridge)

    return body


def _cap_solid():
    """Black ribbed screw cap. Local frame: origin at the cap joint (top of
    neck). Disc sits above (0..CAP_TOP_Z), skirt hangs down to z=-SKIRT_DROP."""
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -SKIRT_DROP))
        .circle(CAP_R)
        .extrude(CAP_H)
    )
    outer = outer.edges(">Z").fillet(0.002)
    # Hollow cavity underneath to slip over the neck
    cavity = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -SKIRT_DROP - 0.001))
        .circle(NECK_R + 0.0015)
        .extrude(CAP_H - 0.004)
    )
    cap = outer.cut(cavity)
    # Vertical knurl ribs around the skirt
    n_ribs = 24
    rib_h = CAP_H - 0.004
    zc = -SKIRT_DROP + rib_h / 2.0
    for i in range(n_ribs):
        ang = 2.0 * math.pi * i / n_ribs
        x = (CAP_R - 0.0005) * math.cos(ang)
        y = (CAP_R - 0.0005) * math.sin(ang)
        rib = (
            cq.Workplane("XY")
            .transformed(offset=(x, y, zc), rotate=(0, 0, math.degrees(ang)))
            .box(0.0016, 0.0012, rib_h)
        )
        cap = cap.union(rib)

    # Spout hinge cradle: two small side walls on top of the cap that bracket
    # the spout base and provide a visible pivot mount.
    wall_w = 0.003
    wall_h = SPOUT_BASE_H + 0.004
    wall_d = 0.003
    # Walls straddle the spout base in Y, starting at the cap top surface
    for y_sign in (-1.0, 1.0):
        y_off = y_sign * (SPOUT_BASE_D / 2.0 + wall_d / 2.0)
        wall = (
            cq.Workplane("XY")
            .transformed(offset=(SPOUT_BASE_W * 0.3, y_off, CAP_TOP_Z + wall_h / 2.0))
            .box(SPOUT_BASE_W + 0.004, wall_d, wall_h)
        )
        cap = cap.union(wall)

    return cap


def _spout_solid():
    """Straw spout in local frame: pivot at origin, tube extends along +X.
    At q=0 the spout lies flat; positive q (axis=-Y) flips it upright."""
    # Hinge base block
    base = (
        cq.Workplane("XY")
        .transformed(offset=(SPOUT_BASE_W / 2.0, 0.0, SPOUT_BASE_H / 2.0))
        .box(SPOUT_BASE_W, SPOUT_BASE_D, SPOUT_BASE_H)
    )
    # Fillet the top edges slightly
    base = base.edges(">Z").fillet(0.001)

    # Straw tube: build along +Z, rotate to lie along +X
    tube = (
        cq.Workplane("XY")
        .circle(SPOUT_R)
        .extrude(SPOUT_LENGTH)
    )
    # Rotate from +Z to +X: Ry(+90°)
    tube = tube.rotate((0, 0, 0), (0, 1, 0), 90.0)
    # Translate so tube starts just past the hinge base
    tube = tube.translate((SPOUT_BASE_W, 0.0, SPOUT_BASE_H / 2.0))

    # Nozzle tip: small flare at the end
    tip = (
        cq.Workplane("XY")
        .circle(SPOUT_R + 0.001)
        .extrude(0.004)
    )
    tip = tip.rotate((0, 0, 0), (0, 1, 0), 90.0)
    tip = tip.translate((SPOUT_BASE_W + SPOUT_LENGTH, 0.0, SPOUT_BASE_H / 2.0))

    spout = base.union(tube).union(tip)
    return spout


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="squat_straw_bottle")

    # Materials
    clear = model.material("clear_pet", rgba=(0.78, 0.85, 0.82, 0.25))
    black = model.material("cap_black", rgba=(0.06, 0.06, 0.07, 1.0))
    spout_mat = model.material("spout_gray", rgba=(0.45, 0.45, 0.48, 1.0))

    # ---- bottle body (root) ----
    bottle = model.part("bottle")
    bottle_geo = _bottle_solid()
    bottle.visual(
        mesh_from_cadquery(bottle_geo, "bottle_shell"),
        material=clear,
        name="bottle_shell",
    )
    bottle.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_W, NECK_TOP_Z)),
        mass=0.035,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- massless carrier for decoupled rotate/slide ----
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
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_H),
        mass=0.007,
        origin=Origin(xyz=(0.0, 0.0, (CAP_TOP_Z - SKIRT_DROP) / 2.0)),
    )

    # ---- straw spout ----
    spout = model.part("spout")
    spout_geo = _spout_solid()
    spout.visual(
        mesh_from_cadquery(spout_geo, "spout_shell"),
        material=spout_mat,
        name="spout_shell",
    )
    spout.inertial = Inertial.from_geometry(
        Box((SPOUT_BASE_W + SPOUT_LENGTH, SPOUT_BASE_D, SPOUT_BASE_H)),
        mass=0.004,
        origin=Origin(xyz=((SPOUT_BASE_W + SPOUT_LENGTH) / 2.0, 0.0, SPOUT_BASE_H / 2.0)),
    )

    # ---- joints ----
    # cap_rotate: CONTINUOUS spin about +Z
    model.articulation(
        "cap_rotate",
        ArticulationType.CONTINUOUS,
        parent=bottle,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    # cap_slide: PRISMATIC lift along +Z
    model.articulation(
        "cap_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=CAP_H, effort=1.0, velocity=1.0),
    )

    # spout_pivot: REVOLUTE, axis=-Y, flips spout from flat to upright
    # Origin at the cap top surface where the spout base sits
    model.articulation(
        "spout_pivot",
        ArticulationType.REVOLUTE,
        parent=cap,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, CAP_TOP_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=2.0,
            lower=0.0, upper=1.5,  # 0 = flat, ~86° = upright
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bottle = object_model.get_part("bottle")
    cap = object_model.get_part("cap")
    spout = object_model.get_part("spout")
    rotate = object_model.get_articulation("cap_rotate")
    slide = object_model.get_articulation("cap_slide")
    pivot = object_model.get_articulation("spout_pivot")

    bottle_shell = bottle.get_visual("bottle_shell")
    cap_shell = cap.get_visual("cap_shell")
    spout_shell = spout.get_visual("spout_shell")

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

    # --- squat square-shouldered shape: body is wider than tall relative to parent ---
    bottle_aabb = ctx.part_world_aabb(bottle)
    if bottle_aabb is not None:
        mn, mx = bottle_aabb
        dx = mx[0] - mn[0]
        dy = mx[1] - mn[1]
        dz = mx[2] - mn[2]
        ctx.check(
            "bottle body is squat (height < 2.5x width)",
            dz < 2.5 * max(dx, dy),
            details=f"dx={dx:.4f}, dy={dy:.4f}, dz={dz:.4f}",
        )
        ctx.check(
            "bottle body is approximately square in XY",
            abs(dx - dy) < 0.008,
            details=f"dx={dx:.4f}, dy={dy:.4f}",
        )

    # --- cap sits on top of the neck ---
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap mounted on top of the neck",
        cap_pos is not None and cap_pos[2] > BODY_H + SHOULDER_H * 0.5,
        details=f"cap origin z={cap_pos}",
    )

    # The cap skirt slips over the neck threads at rest -> intentional overlap.
    ctx.allow_overlap(
        cap,
        bottle,
        elem_a="cap_shell",
        elem_b="bottle_shell",
        reason="Cap skirt is intentionally seated over the threaded neck.",
    )

    # The spout hinge base sits within the cap bracket walls -> intentional overlap.
    ctx.allow_overlap(
        cap,
        spout,
        elem_a="cap_shell",
        elem_b="spout_shell",
        reason="Spout hinge base is intentionally seated within the cap bracket walls at the pivot mount.",
    )

    # --- spout pivot: non-fixed revolute joint with correct limits ---
    pivot_limits = pivot.motion_limits
    ctx.check(
        "spout_pivot is REVOLUTE with bounded limits",
        pivot.articulation_type == ArticulationType.REVOLUTE
        and pivot_limits is not None
        and pivot_limits.lower is not None
        and pivot_limits.upper is not None
        and pivot_limits.upper > pivot_limits.lower + 0.5,
        details=f"type={pivot.articulation_type}, limits=({pivot_limits.lower}, {pivot_limits.upper})",
    )

    # --- spout pivot motion: at q=0 spout is low, at upper limit spout tip rises ---
    with ctx.pose({pivot: 0.0}):
        spout_closed_z = ctx.part_world_aabb(spout)
    with ctx.pose({pivot: pivot_limits.upper}):
        spout_open_z = ctx.part_world_aabb(spout)

    if spout_closed_z is not None and spout_open_z is not None:
        closed_max_z = spout_closed_z[1][2]
        open_max_z = spout_open_z[1][2]
        ctx.check(
            "spout pivot raises the tip when opened",
            open_max_z > closed_max_z + 0.010,
            details=f"closed_max_z={closed_max_z:.4f}, open_max_z={open_max_z:.4f}",
        )

    # --- spout is mounted on the cap (seated contact at pivot) ---
    ctx.expect_contact(
        spout,
        cap,
        contact_tol=0.002,
        name="spout hinge base contacts cap bracket (seated pivot)",
    )
    ctx.expect_overlap(
        spout,
        cap,
        axes="xy",
        min_overlap=0.002,
        name="spout overlaps cap in XY (mounted)",
    )

    # --- cap rotation moves the spout (proving cap_rotate drives spout) ---
    with ctx.pose({rotate: 0.0}):
        spout_aabb_0 = ctx.part_world_aabb(spout)
    with ctx.pose({rotate: math.pi / 2.0}):
        spout_aabb_90 = ctx.part_world_aabb(spout)
    if spout_aabb_0 is not None and spout_aabb_90 is not None:
        e0 = [spout_aabb_0[1][i] - spout_aabb_0[0][i] for i in range(3)]
        e90 = [spout_aabb_90[1][i] - spout_aabb_90[0][i] for i in range(3)]
        ctx.check(
            "cap rotation moves spout (extents differ)",
            abs(e0[0] - e90[0]) > 0.001 or abs(e0[1] - e90[1]) > 0.001,
            details=f"rest_extents={e0}, quarter_extents={e90}",
        )

    # --- cap_slide lifts cap + spout assembly ---
    rest_z = ctx.part_world_position(cap)
    with ctx.pose({slide: CAP_H}):
        lifted_z = ctx.part_world_position(cap)
    if rest_z is not None and lifted_z is not None:
        ctx.check(
            "cap_slide lifts the cap off the neck",
            lifted_z[2] > rest_z[2] + CAP_H * 0.8,
            details=f"rest_z={rest_z[2]:.4f}, lifted_z={lifted_z[2]:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
