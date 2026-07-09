from __future__ import annotations

# Squat square-shouldered bottle with a straw spout that pivots up from the cap.
# Frame: vertical axis along +Z, bottle standing on z=0, centerline on x=y=0.
#   - rounded-square body -> short square-to-round shoulder -> cylindrical neck
#     with raised spiral threads -> flat cap with knurled edge -> pivoting straw spout.
# Volume measurement bands are molded into the body walls.
# Articulations:
#   - cap_rotate:   CONTINUOUS spin of the cap about +Z (screw action).
#   - spout_pivot:  REVOLUTE pivot of the straw spout from stowed (flat along +Y)
#                   to deployed (upright along +Z), axis along +X, limits 0 to 1.5 rad.

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

# ---- key dimensions (m) ----
BODY_WIDTH = 0.062        # square body width (X)
BODY_DEPTH = 0.062        # square body depth (Y)
BODY_CORNER_R = 0.008     # corner fillet radius
BODY_HEIGHT = 0.070       # straight body height
SHOULDER_HEIGHT = 0.018   # shoulder taper height
NECK_R = 0.012            # neck outer radius
NECK_BORE_R = 0.009       # neck inner bore
NECK_HEIGHT = 0.016       # neck height above shoulder
WALL = 0.0014             # wall thickness

BODY_TOP_Z = BODY_HEIGHT                             # 0.070
SHOULDER_TOP_Z = BODY_TOP_Z + SHOULDER_HEIGHT        # 0.088
NECK_TOP_Z = SHOULDER_TOP_Z + NECK_HEIGHT            # 0.104

CAP_R = 0.015            # cap outer radius
CAP_HEIGHT = 0.016       # taller cap to cover the threaded neck
CAP_BORE_R = NECK_R - 0.002  # bore smaller than neck for solid overlap
CAP_MOUNT_Z = SHOULDER_TOP_Z + 0.002  # 0.090 - cap sits well down over the neck

# Spout dimensions
SPOUT_R = 0.003          # straw outer radius
SPOUT_LENGTH = 0.040     # straw length
SPOUT_BORE_R = 0.002     # straw inner bore


def _bottle_solid() -> cq.Workplane:
    """Squat square body with rounded corners, shoulder taper, and cylindrical neck.
    Hollow interior with open mouth at neck rim."""
    r = BODY_CORNER_R

    # Body: extruded rectangle with filleted edges
    body = (
        cq.Workplane("XY")
        .rect(BODY_WIDTH, BODY_DEPTH)
        .extrude(BODY_HEIGHT)
    )
    body = body.edges("|Z").fillet(r)
    body = body.edges("<Z").fillet(0.003)

    # Shoulder: loft from square top to circle at neck base
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP_Z)
        .rect(BODY_WIDTH, BODY_DEPTH)
        .workplane(offset=SHOULDER_HEIGHT)
        .circle(NECK_R + 0.002)
        .loft()
    )

    # Neck: cylinder on top of shoulder
    neck = (
        cq.Workplane("XY")
        .workplane(offset=SHOULDER_TOP_Z)
        .circle(NECK_R)
        .extrude(NECK_HEIGHT)
    )

    outer = body.union(shoulder).union(neck)

    # Hollow interior: cut cavity from inside
    iwall = WALL
    ir = max(r - iwall, 0.002)
    cavity_body = (
        cq.Workplane("XY")
        .workplane(offset=iwall)
        .rect(BODY_WIDTH - 2 * iwall, BODY_DEPTH - 2 * iwall)
        .extrude(BODY_HEIGHT - iwall)
    )
    cavity_body = cavity_body.edges("|Z").fillet(ir)

    # Cavity through shoulder and neck (cylindrical bore, open at top)
    cavity_neck = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP_Z)
        .circle(NECK_BORE_R)
        .extrude(SHOULDER_HEIGHT + NECK_HEIGHT + 0.005)
    )

    outer = outer.cut(cavity_body).cut(cavity_neck)
    return outer


def _bottle_with_bands() -> cq.Workplane:
    """Add molded volume bands (raised ridges) around the body."""
    bottle = _bottle_solid()

    band_heights = [0.020, 0.040, 0.058]
    band_thickness = 0.0008
    band_width = 0.002
    r = BODY_CORNER_R + band_thickness

    for z_center in band_heights:
        z_bot = z_center - band_width / 2.0
        band = (
            cq.Workplane("XY")
            .workplane(offset=z_bot)
            .rect(BODY_WIDTH + 2 * band_thickness, BODY_DEPTH + 2 * band_thickness)
            .extrude(band_width)
        )
        band = band.edges("|Z").fillet(r)
        inner = (
            cq.Workplane("XY")
            .workplane(offset=z_bot - 0.0001)
            .rect(BODY_WIDTH, BODY_DEPTH)
            .extrude(band_width + 0.0002)
        )
        inner = inner.edges("|Z").fillet(BODY_CORNER_R)
        band = band.cut(inner)
        bottle = bottle.union(band)

    return bottle


def _bottle_mesh():
    return mesh_from_cadquery(_bottle_with_bands(), "bottle_shell")


def _neck_threads():
    """Raised spiral-like thread ridges on the neck."""
    g = None
    thread_positions = [SHOULDER_TOP_Z + 0.003, SHOULDER_TOP_Z + 0.007, SHOULDER_TOP_Z + 0.011]
    for zt in thread_positions:
        ring = TorusGeometry(
            NECK_R - 0.0004,
            0.0010,
            radial_segments=8,
            tubular_segments=36,
        )
        ring.translate(0.0, 0.0, zt)
        if g is None:
            g = ring
        else:
            g.merge(ring)
    return mesh_from_geometry(g, "neck_threads")


def _cap_solid() -> cq.Workplane:
    """Flat cap with knurled edge and a spout channel slot."""
    cap = (
        cq.Workplane("XY")
        .circle(CAP_R)
        .extrude(CAP_HEIGHT)
    )
    # Hollow bore: smaller than neck so cap material overlaps neck solid
    bore = (
        cq.Workplane("XY")
        .circle(CAP_BORE_R)
        .extrude(CAP_HEIGHT - 0.002)
    )
    cap = cap.cut(bore)

    # Spout channel: slot in the top of the cap for the spout to nest in
    slot_width = SPOUT_R * 2.0 + 0.002
    slot_length = SPOUT_LENGTH * 0.5
    slot = (
        cq.Workplane("XY")
        .workplane(offset=CAP_HEIGHT - 0.003)
        .center(0.0, slot_length / 2.0)
        .rect(slot_width, slot_length)
        .extrude(0.004)
    )
    cap = cap.cut(slot)

    # Knurl grooves around the cap edge
    n = 20
    for i in range(n):
        a = 2.0 * math.pi * i / n
        groove = (
            cq.Workplane("XY")
            .center(CAP_R * math.cos(a), CAP_R * math.sin(a))
            .circle(0.0008)
            .extrude(CAP_HEIGHT)
        )
        cap = cap.cut(groove)

    return cap


def _cap_mesh():
    return mesh_from_cadquery(_cap_solid(), "cap_shell")


def _spout_solid() -> cq.Workplane:
    """Straw spout tube built extending along +Z from origin.
    After rotation by -90 deg around X, it extends along +Y (stowed flat).
    Pivot is at local origin. Pivot axis is +X."""
    # Build tube along +Z
    tube = (
        cq.Workplane("XY")
        .workplane(offset=SPOUT_R)  # start slightly above origin so pivot boss connects
        .circle(SPOUT_R)
        .extrude(SPOUT_LENGTH - SPOUT_R)
    )
    # Hollow bore
    bore = (
        cq.Workplane("XY")
        .workplane(offset=SPOUT_R)
        .circle(SPOUT_BORE_R)
        .extrude(SPOUT_LENGTH - SPOUT_R)
    )
    tube = tube.cut(bore)

    # Pivot ball at origin for hinge connection
    ball = cq.Workplane("XY").sphere(SPOUT_R + 0.001)
    tube = tube.union(ball)

    # Rotate from +Z to +Y: rotate -90 deg around X
    tube = tube.rotate((0, 0, 0), (1, 0, 0), -90)
    return tube


def _spout_mesh():
    return mesh_from_cadquery(_spout_solid(), "spout_tube")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_bottle")

    clear = model.material("clear_pet", rgba=(0.80, 0.88, 0.90, 0.30))
    clear_neck = model.material("clear_neck", rgba=(0.72, 0.80, 0.84, 0.35))
    cap_mat = model.material("cap_teal", rgba=(0.12, 0.45, 0.52, 1.0))
    spout_mat = model.material("spout_white", rgba=(0.92, 0.92, 0.90, 1.0))

    # ---- bottle body (root): squat square PET shell with bands and threads ----
    body = model.part("bottle_body")
    body.visual(_bottle_mesh(), material=clear, name="bottle_shell")
    body.visual(_neck_threads(), material=clear_neck, name="neck_threads")
    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, NECK_TOP_Z)),
        mass=0.035,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- cap ----
    cap = model.part("cap")
    cap.visual(_cap_mesh(), material=cap_mat, name="cap_shell")
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_HEIGHT),
        mass=0.004,
        origin=Origin(xyz=(0.0, 0.0, CAP_HEIGHT / 2.0)),
    )

    # ---- straw spout ----
    spout = model.part("straw_spout")
    spout.visual(_spout_mesh(), material=spout_mat, name="spout_tube")
    spout.inertial = Inertial.from_geometry(
        Cylinder(SPOUT_R, SPOUT_LENGTH),
        mass=0.002,
        origin=Origin(xyz=(0.0, SPOUT_LENGTH / 2.0, 0.0)),
    )

    # ---- cap_rotate: CONTINUOUS spin about +Z ----
    model.articulation(
        "cap_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, CAP_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    # ---- spout_pivot: REVOLUTE pivot from stowed (flat +Y) to deployed (upright +Z) ----
    # At q=0: spout lies flat along +Y (stowed on cap top).
    # Positive q rotates around +X: +Y -> +Z, so spout rises upright.
    # Pivot origin is at the cap top center.
    model.articulation(
        "spout_pivot",
        ArticulationType.REVOLUTE,
        parent=cap,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, CAP_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.5,
            effort=0.5,
            velocity=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle_body")
    cap = object_model.get_part("cap")
    spout = object_model.get_part("straw_spout")
    cap_rotate = object_model.get_articulation("cap_rotate")
    spout_pivot = object_model.get_articulation("spout_pivot")

    # --- bottle body is squat (height less than 2.5x width) ---
    body_aabb = ctx.part_world_aabb(body)
    body_ext = (
        body_aabb[1][0] - body_aabb[0][0],
        body_aabb[1][1] - body_aabb[0][1],
        body_aabb[1][2] - body_aabb[0][2],
    )
    ctx.check(
        "bottle is squat (height less than 2.5x width)",
        body_ext[2] < 2.5 * body_ext[0],
        details=f"body extents={body_ext}",
    )

    # --- body is roughly square in cross-section ---
    ctx.check(
        "body cross-section is roughly square",
        abs(body_ext[0] - body_ext[1]) < 0.005,
        details=f"width={body_ext[0]:.4f}, depth={body_ext[1]:.4f}",
    )

    # --- bottle is hollow (clear material with alpha < 1) ---
    clear_mat = next(m for m in object_model.materials if m.name == "clear_pet")
    a = clear_mat.rgba[3] if clear_mat.rgba is not None else 1.0
    ctx.check(
        "bottle shell is translucent",
        a < 1.0,
        details=f"clear_pet alpha={a}",
    )

    # --- neck threads exist ---
    threads_vis = body.get_visual("neck_threads")
    ctx.check(
        "neck threads visual exists",
        threads_vis is not None,
        details="neck_threads visual not found on bottle_body",
    )

    # --- cap is mounted at the top of the bottle ---
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap mounted at top of bottle",
        cap_pos is not None and cap_pos[2] > SHOULDER_TOP_Z,
        details=f"cap origin={cap_pos}",
    )

    # --- cap overlaps neck (screw-over capture) ---
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_shell",
        elem_b="bottle_shell",
        reason="The cap skirt intentionally screws down over the threaded neck.",
    )
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_shell",
        elem_b="neck_threads",
        reason="The cap skirt intentionally covers the neck threads when closed.",
    )

    # --- spout overlaps cap at rest (pivot boss seated in cap) ---
    ctx.allow_overlap(
        spout,
        cap,
        elem_a="spout_tube",
        elem_b="cap_shell",
        reason="The spout pivot boss and tube nest in the cap slot when stowed.",
    )

    # --- spout pivot is REVOLUTE with proper limits ---
    pivot_limits = spout_pivot.motion_limits
    ctx.check(
        "spout_pivot is REVOLUTE with bounded limits",
        spout_pivot.articulation_type == ArticulationType.REVOLUTE
        and pivot_limits is not None
        and pivot_limits.lower is not None
        and pivot_limits.upper is not None
        and pivot_limits.upper > pivot_limits.lower + 0.5,
        details=f"type={spout_pivot.articulation_type}, limits=({pivot_limits.lower}, {pivot_limits.upper})" if pivot_limits else "no limits",
    )

    # --- spout rises when pivoted ---
    spout_rest_aabb = ctx.part_world_aabb(spout)
    rest_max_z = spout_rest_aabb[1][2]
    with ctx.pose({spout_pivot: 1.2}):
        spout_up_aabb = ctx.part_world_aabb(spout)
        up_max_z = spout_up_aabb[1][2]
    ctx.check(
        "spout rises when pivoted (max Z increases)",
        up_max_z > rest_max_z + 0.008,
        details=f"rest max_z={rest_max_z:.4f}, pivoted max_z={up_max_z:.4f}",
    )

    # --- cap_rotate is CONTINUOUS ---
    ctx.check(
        "cap_rotate is CONTINUOUS",
        cap_rotate.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={cap_rotate.articulation_type}",
    )

    # --- body has molded volume bands (extents > base width) ---
    ctx.check(
        "body has molded volume bands (extents > base width)",
        body_ext[0] > BODY_WIDTH + 0.0005,
        details=f"body width={body_ext[0]:.4f}, base width={BODY_WIDTH}",
    )

    return ctx.report()


object_model = build_object_model()
