from __future__ import annotations

# Dark glass beer bottle with a swing-top (flip-top) stopper closure.
# Frame: bottle axis along +Z (base at z=0, mouth/lip at the top), centered on
#   the z-axis. Scale ~0.24 m tall, ~0.065 m body diameter.
# Articulations:
#   - stopper: REVOLUTE, wire-bail lever pivots on the neck collar to swing
#     the ceramic/rubber plug up and off the bottle mouth.

import math

import cadquery as cq

GLASS_RGBA = (0.18, 0.12, 0.07, 0.4)
WIRE_RGBA = (0.62, 0.60, 0.58, 1.0)
CERAMIC_RGBA = (0.92, 0.88, 0.80, 1.0)
RUBBER_RGBA = (0.35, 0.08, 0.06, 1.0)

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

# ---- Bottle profile heights (z, in metres) ----------------------------------
BASE_Z = 0.0
BODY_TOP_Z = 0.120
SHOULDER_TOP_Z = 0.150
NECK_TOP_Z = 0.222
LIP_TOP_Z = 0.230

BODY_R = 0.0325
NECK_R = 0.0115
LIP_R = 0.0135

WALL = 0.0028
BORE_R = 0.0085

# ---- Swing-top parameters ---------------------------------------------------
PIVOT_Z = 0.210          # pivot height on the neck (below lip flare)
WIRE_R = 0.0012          # bail/collar wire radius
PIVOT_Y = 0.018          # half-distance between pivot ears (Y offset from axis)
COLLAR_R = NECK_R + 0.0015  # collar ring center radius (clears the neck)

# Stopper local frame: origin at pivot, +Z up.
# Gasket sits exactly on the lip top surface; plug sits on gasket.
GASKET_LOCAL_Z = LIP_TOP_Z - PIVOT_Z          # 0.020  gasket bottom
PLUG_LOCAL_Z = GASKET_LOCAL_Z + 0.003         # 0.023  plug bottom (on gasket)
PLUG_R = 0.009
PLUG_H = 0.014
GASKET_R = 0.0105
GASKET_H = 0.003


# =============================================================================
# Geometry helpers
# =============================================================================

def _profile_loft(sections, ruled: bool = True) -> cq.Workplane:
    """Loft circular sections stacked along +Z."""
    wp = cq.Workplane("XY")
    prev = 0.0
    for i, (z, r) in enumerate(sections):
        off = z if i == 0 else z - prev
        if i > 0 or abs(off) > 1e-12:
            wp = wp.workplane(offset=off)
        wp = wp.circle(r)
        prev = z
    return wp.loft(ruled=ruled)


def _bottle_solid() -> cq.Workplane:
    """Hollow dark-glass bottle shell (identical to parent)."""
    outer = _profile_loft([
        (BASE_Z, 0.0235),
        (0.003, 0.0250),
        (0.008, 0.0305),
        (0.013, 0.0322),
        (0.018, 0.0325),
        (BODY_TOP_Z, 0.0325),
        (0.128, 0.0316),
        (0.137, 0.0288),
        (SHOULDER_TOP_Z, 0.0205),
        (0.160, 0.0150),
        (0.170, 0.0122),
        (NECK_TOP_Z, NECK_R),
        (0.224, 0.0128),
        (0.227, LIP_R),
        (LIP_TOP_Z, LIP_R),
    ])
    inner = _profile_loft([
        (0.010, 0.0200),
        (0.018, 0.0270),
        (0.024, 0.0297),
        (BODY_TOP_Z - 0.004, 0.0297),
        (0.128, 0.0288),
        (0.137, 0.0260),
        (SHOULDER_TOP_Z, 0.0177),
        (0.160, 0.0122),
        (0.170, 0.0094),
        (NECK_TOP_Z, BORE_R),
        (LIP_TOP_Z + 0.004, BORE_R),
    ])
    return outer.cut(inner)


def _rod(p1, p2, r) -> cq.Workplane:
    """Cylindrical rod from 3D point p1 to p2 with radius r."""
    dx, dy, dz = p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length < 1e-9:
        return cq.Workplane("XY").sphere(r)

    cyl = cq.Workplane("XY").circle(r).extrude(length)

    # Rotate from +Z to direction (dx, dy, dz)
    z_vec = cq.Vector(0, 0, 1)
    d_vec = cq.Vector(dx, dy, dz).normalized()
    cross = z_vec.cross(d_vec)
    dot = max(-1.0, min(1.0, z_vec.dot(d_vec)))

    if cross.Length > 1e-9:
        angle = math.degrees(math.acos(dot))
        ax = cross.normalized()
        cyl = cyl.rotate((0, 0, 0), (ax.x, ax.y, ax.z), angle)
    elif dot < 0:
        cyl = cyl.rotate((0, 0, 0), (1, 0, 0), 180.0)

    return cyl.translate(p1)


def _collar_mesh():
    """Wire collar ring with two pivot ears — clips around the bottle neck."""
    ring_h = 0.005
    ring_z0 = PIVOT_Z - ring_h / 2.0
    ring_outer = COLLAR_R + WIRE_R * 1.5  # ~0.0148
    ring_inner = COLLAR_R - WIRE_R * 1.5  # ~0.0112

    # Annular ring body (thick enough for robust connectivity)
    ring = (
        cq.Workplane("XY")
        .workplane(offset=ring_z0)
        .circle(ring_outer)
        .circle(ring_inner)
        .extrude(ring_h)
    )

    # Pivot ears: extend from inside the ring wall outward past the pivot point,
    # ensuring solid geometric overlap with the ring body.
    ear_half_x = WIRE_R * 1.8
    ear_inner_y = ring_outer - 0.002  # starts inside ring wall
    ear_outer_y = PIVOT_Y + 0.003    # extends past pivot point
    for sign in (1, -1):
        cy = sign * (ear_inner_y + ear_outer_y) / 2.0
        half_y = (ear_outer_y - ear_inner_y) / 2.0
        ear = (
            cq.Workplane("XY")
            .workplane(offset=ring_z0)
            .center(0.0, cy)
            .rect(ear_half_x * 2, half_y * 2)
            .extrude(ring_h)
        )
        ring = ring.union(ear)

    return mesh_from_cadquery(ring, "neck_collar")


def _bail_mesh():
    """Wire bail: two curved arms from pivot ears up to plug holder + cross-bar."""
    # Each arm is a polyline of 3D points in the stopper local frame.
    # The arms curve outward from the pivot ears, then sweep upward and inward
    # to cradle the plug from both sides.
    arm_top_y = 0.007
    arm_top_z = PLUG_LOCAL_Z + PLUG_H + 0.003

    arm_paths = [
        # +Y arm
        [
            (0.0, PIVOT_Y, 0.0),
            (0.0, PIVOT_Y * 0.88, 0.018),
            (0.0, PIVOT_Y * 0.45, 0.032),
            (0.0, arm_top_y, arm_top_z),
        ],
        # -Y arm
        [
            (0.0, -PIVOT_Y, 0.0),
            (0.0, -PIVOT_Y * 0.88, 0.018),
            (0.0, -PIVOT_Y * 0.45, 0.032),
            (0.0, -arm_top_y, arm_top_z),
        ],
    ]

    segments = []

    # Arm rod segments
    for path in arm_paths:
        for i in range(len(path) - 1):
            segments.append(_rod(path[i], path[i + 1], WIRE_R))
        # Smooth joint spheres at bend points
        for pt in path[1:-1]:
            segments.append(cq.Workplane("XY").sphere(WIRE_R * 1.3).translate(pt))

    # Upper cross-bar connecting the two arm tops
    segments.append(_rod((0.0, -arm_top_y, arm_top_z), (0.0, arm_top_y, arm_top_z), WIRE_R))

    # Pivot pins: short horizontal wire ends that insert into collar ear holes
    pin_inset = 0.005
    for sign in (1, -1):
        segments.append(
            _rod(
                (0.0, sign * PIVOT_Y, 0.0),
                (0.0, sign * (PIVOT_Y - pin_inset), 0.0),
                WIRE_R * 0.85,
            )
        )

    # Union all segments into one solid
    result = segments[0]
    for seg in segments[1:]:
        result = result.union(seg)

    return mesh_from_cadquery(result, "bail_wire")


def _plug_mesh():
    """Ceramic plug with domed top and base rim."""
    plug_z = PLUG_LOCAL_Z

    # Tapered body
    body = (
        cq.Workplane("XY")
        .workplane(offset=plug_z)
        .circle(PLUG_R)
        .workplane(offset=PLUG_H)
        .circle(PLUG_R * 0.92)
        .loft(ruled=True)
    )

    # Dome crown
    dome = (
        cq.Workplane("XY")
        .workplane(offset=plug_z + PLUG_H)
        .circle(PLUG_R * 0.92)
        .workplane(offset=0.005)
        .circle(PLUG_R * 0.30)
        .loft(ruled=True)
    )
    plug = body.union(dome)

    # Base rim ring
    rim = (
        cq.Workplane("XY")
        .workplane(offset=plug_z - 0.001)
        .circle(PLUG_R + 0.001)
        .circle(PLUG_R - 0.001)
        .extrude(0.002)
    )
    plug = plug.union(rim)

    return mesh_from_cadquery(plug, "plug_ceramic")


def _gasket_mesh():
    """Rubber gasket ring seated between plug and bottle mouth."""
    gasket = (
        cq.Workplane("XY")
        .workplane(offset=GASKET_LOCAL_Z)
        .circle(GASKET_R)
        .circle(GASKET_R - 0.003)
        .extrude(GASKET_H)
    )
    return mesh_from_cadquery(gasket, "plug_gasket")


# =============================================================================
# Object model
# =============================================================================

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="beer_bottle_swingtop")

    glass = model.material("dark_glass", rgba=GLASS_RGBA)
    wire_metal = model.material("wire_metal", rgba=WIRE_RGBA)
    ceramic = model.material("ceramic", rgba=CERAMIC_RGBA)
    rubber = model.material("rubber", rgba=RUBBER_RGBA)

    # ---- bottle (root): hollow dark-glass shell + fixed wire collar ----
    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_cadquery(_bottle_solid(), "bottle_glass"),
        material=glass,
        name="bottle_glass",
    )
    bottle.visual(
        _collar_mesh(),
        material=wire_metal,
        name="neck_collar",
    )
    bottle.inertial = Inertial.from_geometry(
        Cylinder(radius=BODY_R, length=LIP_TOP_Z),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, LIP_TOP_Z / 2.0)),
    )

    # ---- stopper: wire-bail lever + ceramic plug + rubber gasket ----
    stopper = model.part("stopper")
    stopper.visual(_bail_mesh(), material=wire_metal, name="bail_wire")
    stopper.visual(_plug_mesh(), material=ceramic, name="plug_ceramic")
    stopper.visual(_gasket_mesh(), material=rubber, name="plug_gasket")
    stopper.inertial = Inertial.from_geometry(
        Cylinder(radius=0.012, length=0.040),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    # REVOLUTE joint: bail pivots on the collar ears.
    # Origin at the pivot axis center on the neck.
    # Axis along +Y: right-hand rule makes positive q swing the plug toward +X
    # (away from the mouth, opening the bottle).
    # At q=0 the plug is seated; at q≈2.8 rad (~160°) the stopper is fully open.
    model.articulation(
        "bottle_to_stopper",
        ArticulationType.REVOLUTE,
        parent=bottle,
        child=stopper,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=4.0, lower=0.0, upper=2.8,
        ),
    )

    return model


# =============================================================================
# Tests
# =============================================================================

def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bottle = object_model.get_part("bottle")
    stopper = object_model.get_part("stopper")
    joint = object_model.get_articulation("bottle_to_stopper")

    # --- Dark glass: translucent body material (alpha < 1). ---
    ctx.check(
        "bottle glass is translucent (alpha < 1)",
        GLASS_RGBA[3] < 1.0,
        details=f"glass rgba={GLASS_RGBA}",
    )

    # --- Long-neck silhouette preserved from parent. ---
    body_ext = _ext(ctx.part_world_aabb(bottle))
    ctx.check(
        "bottle has long-neck silhouette",
        body_ext[2] > 0.20 and body_ext[2] > 2.5 * max(body_ext[0], body_ext[1]),
        details=f"bottle extents={body_ext}",
    )

    # --- Stopper part is mounted at the bottle mouth region. ---
    stopper_pos = ctx.part_world_position(stopper)
    ctx.check(
        "stopper mounted at the bottle mouth region",
        stopper_pos is not None and stopper_pos[2] > 0.19,
        details=f"stopper origin={stopper_pos}",
    )

    # --- Joint is revolute (swing-top mechanism, not prismatic pop-off). ---
    ctx.check(
        "stopper joint is revolute (swing-top mechanism)",
        joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"joint type={joint.articulation_type}",
    )

    # --- At q=0 the ceramic plug is seated on the mouth (Z overlap with lip). ---
    ctx.allow_overlap(
        stopper,
        bottle,
        elem_a="plug_ceramic",
        elem_b="bottle_glass",
        reason="Ceramic plug is seated on the bottle mouth lip when closed.",
    )
    ctx.allow_overlap(
        stopper,
        bottle,
        elem_a="plug_gasket",
        elem_b="bottle_glass",
        reason="Rubber gasket compresses against the bottle mouth rim when closed.",
    )
    ctx.allow_overlap(
        stopper,
        bottle,
        elem_a="bail_wire",
        elem_b="neck_collar",
        reason="Bail pivot pins insert into collar ear holes (captured pivot fit).",
    )

    # The plug is seated at the mouth: gasket contacts the lip surface.
    ctx.expect_contact(
        stopper,
        bottle,
        elem_a="plug_gasket",
        elem_b="bottle_glass",
        contact_tol=0.002,
        name="gasket contacts the bottle mouth rim when closed",
    )

    # Plug overlaps the bottle mouth region in XY (centered on the bore).
    ctx.expect_overlap(
        stopper,
        bottle,
        axes="xy",
        elem_a="plug_ceramic",
        elem_b="bottle_glass",
        min_overlap=0.005,
        name="plug centered over the bottle mouth when closed",
    )

    # --- Positive q swings the plug away from the mouth (opening). ---
    # Use element AABBs since part_world_position returns the fixed joint origin.
    plug_rest = ctx.part_element_world_aabb(stopper, elem="plug_ceramic")
    rest_center_z = (plug_rest[0][2] + plug_rest[1][2]) / 2.0

    with ctx.pose({joint: 2.8}):
        plug_open = ctx.part_element_world_aabb(stopper, elem="plug_ceramic")
        open_center_z = (plug_open[0][2] + plug_open[1][2]) / 2.0
        open_center_x = (plug_open[0][0] + plug_open[1][0]) / 2.0
        rest_center_x = (plug_rest[0][0] + plug_rest[1][0]) / 2.0

        # Plug swings laterally: significant X displacement.
        ctx.check(
            "stopper swings laterally when opened",
            abs(open_center_x - rest_center_x) > 0.010,
            details=f"rest_x={rest_center_x:.4f}, open_x={open_center_x:.4f}",
        )

        # Plug center drops below the mouth level when fully open.
        ctx.check(
            "plug drops below mouth level when swung fully open",
            open_center_z < LIP_TOP_Z,
            details=f"rest_z={rest_center_z:.4f}, open_z={open_center_z:.4f}, lip={LIP_TOP_Z}",
        )

    # --- Intermediate open pose (~80°) proves revolute swing. ---
    with ctx.pose({joint: 1.4}):
        plug_mid = ctx.part_element_world_aabb(stopper, elem="plug_ceramic")
        mid_center_x = (plug_mid[0][0] + plug_mid[1][0]) / 2.0
        ctx.check(
            "stopper mid-swing shows lateral displacement",
            abs(mid_center_x - rest_center_x) > 0.005,
            details=f"rest_x={rest_center_x:.4f}, mid_x={mid_center_x:.4f}",
        )

    # --- Ceramic plug is visibly present (material check). ---
    ctx.check(
        "ceramic plug material is opaque white/cream",
        CERAMIC_RGBA[3] >= 1.0 and CERAMIC_RGBA[0] > 0.8,
        details=f"ceramic rgba={CERAMIC_RGBA}",
    )

    # --- Rubber gasket is visibly present. ---
    ctx.check(
        "rubber gasket material is dark red",
        RUBBER_RGBA[3] >= 1.0 and RUBBER_RGBA[0] < 0.5 and RUBBER_RGBA[1] < 0.2,
        details=f"rubber rgba={RUBBER_RGBA}",
    )

    return ctx.report()


object_model = build_object_model()
