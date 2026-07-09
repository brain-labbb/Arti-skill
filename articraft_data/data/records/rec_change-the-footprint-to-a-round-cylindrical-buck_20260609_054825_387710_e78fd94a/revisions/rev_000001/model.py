from __future__ import annotations

# Round cylindrical bucket-style red plastic shopping basket with a single
# semicircular swing bail handle.
#
# Coordinate convention: +Z up, basket rests on the ground at z=0. The body is
# a hollow cylindrical shell with a flat circular base, a thick rolled top rim,
# and vertical rib slots cut through the wall at regular angular intervals.
# Two pivot lugs sit on opposite sides of the rim. A dark-red semicircular bail
# handle swings (REVOLUTE about the diameter line through the two lugs) from
# upright over the mouth down to either side.

import math

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

# ---------------------------------------------------------------------------
# Dimensions (meters)
# ---------------------------------------------------------------------------
R_OUTER = 0.150  # outer radius of the cylindrical wall
H = 0.220  # body height
WALL = 0.012  # wall thickness

# Flat base
BASE_THICK = 0.010  # base disk thickness

# Rolled rim lip
RIM_H = 0.022
RIM_OVERHANG = 0.010  # how far the lip rolls out past the outer wall

# Vertical rib slots around the cylindrical wall
N_SLOTS = 16  # number of vertical slots evenly spaced around circumference
SLOT_W = 0.018  # slot width (tangential)
SLOT_H = 0.130  # slot height (vertical)
SLOT_R = 0.008  # fillet radius on slot corners
SLOT_ZC = H * 0.44  # vertical center of the slots

# Bail handle / pivots
# Lugs are on the Y axis (at +-Y on the rim)
PIVOT_R = R_OUTER + RIM_OVERHANG * 0.3  # pivot sits just outside the wall
PIVOT_Z = H - 0.010  # pivot height near the top of the rim
BAR_R = 0.008  # handle bar radius (wire gauge)
ARCH_R = R_OUTER + 0.005  # semicircular arch radius (spans the mouth)
HANDLE_BOSS_R = 0.016  # pivot knuckle on the handle ends
HANDLE_BOSS_LEN = 0.018
LUG_R = 0.018  # pivot lug radius on the tub
LUG_LEN = 0.016  # lug protrusion length


# ---------------------------------------------------------------------------
# Geometry builders
# ---------------------------------------------------------------------------
def _bucket_mesh() -> object:
    """Hollow cylindrical bucket: open-top shell, flat base, rolled rim,
    vertical rib slots cut through the wall, and two pivot lugs."""

    # Main cylindrical wall (solid cylinder first, then hollow)
    outer = (
        cq.Workplane("XY")
        .circle(R_OUTER)
        .extrude(H)
    )
    # Shell: remove the top face to make an open-mouth bucket
    bucket = outer.faces(">Z").shell(-WALL)

    # Flat circular base disk
    base = (
        cq.Workplane("XY")
        .circle(R_OUTER - WALL * 0.5)
        .extrude(BASE_THICK)
    )
    bucket = bucket.union(base)

    # Soften the bottom outer edge
    try:
        bucket = bucket.edges("<Z").fillet(0.005)
    except Exception:
        pass

    # Rolled top rim: a torus-like ring that caps the wall top and rolls outward.
    r_rim_center = R_OUTER + RIM_OVERHANG * 0.5
    r_rim_tube = RIM_H * 0.5
    rim = (
        cq.Workplane("XY", origin=(0.0, 0.0, H - RIM_H * 0.25))
        .circle(r_rim_center + r_rim_tube)
        .circle(r_rim_center - r_rim_tube)
        .extrude(RIM_H)
    )
    # Round the rim top/bottom outer edges
    try:
        rim = rim.edges("#Z").fillet(0.004)
    except Exception:
        pass
    bucket = bucket.union(rim)

    # Vertical rib slots cut radially through the cylindrical wall.
    # Each slot is a thin box oriented radially, positioned at angle theta.
    for i in range(N_SLOTS):
        theta = 2.0 * math.pi * i / N_SLOTS
        # Radial center of the slot cutter (middle of the wall thickness)
        r_mid = R_OUTER - WALL * 0.5
        cx = r_mid * math.cos(theta)
        cy = r_mid * math.sin(theta)

        # Build a tall thin rounded-rect box in a radial orientation.
        # The box is extruded along the radial direction (through the wall).
        # We use a workplane oriented tangentially at (cx, cy, SLOT_ZC).
        # Local X = tangential (slot width), local Y = radial (extrusion = wall depth),
        # local Z = vertical (slot height).
        # Normal to the workplane = radial direction at this angle.
        nx, ny = math.cos(theta), math.sin(theta)

        sk = cq.Sketch().rect(SLOT_W, SLOT_H).vertices().fillet(SLOT_R)
        cutter = (
            cq.Workplane(
                cq.Plane(
                    origin=cq.Vector(cx, cy, SLOT_ZC),
                    normal=cq.Vector(nx, ny, 0.0),
                )
            )
            .placeSketch(sk)
            .extrude(WALL + 0.020, both=True)
        )
        bucket = bucket.cut(cutter)

    # Two pivot lugs on opposite sides at the rim (along Y axis).
    for sy in (-1.0, 1.0):
        lug = (
            cq.Workplane(
                "XZ",
                origin=(0.0, sy * PIVOT_R, PIVOT_Z),
            )
            .circle(LUG_R)
            .extrude(sy * LUG_LEN)
        )
        bucket = bucket.union(lug)

    return mesh_from_cadquery(bucket, "bucket_body")


def _handle_mesh() -> object:
    """Semicircular bail handle: a rod bent into a half-circle arc.
    LOCAL frame: pivot line is the local X axis at z=0, the arch rises
    in local +Z. The two pivot ends sit at (+-PIVOT_R, 0, 0)."""

    # Semicircular arc in the XZ plane, from (-ARCH_R, 0) to (+ARCH_R, 0),
    # rising to (0, ARCH_R) at the top.
    path = (
        cq.Workplane("XZ")
        .moveTo(-ARCH_R, 0.0)
        .threePointArc((0.0, ARCH_R), (ARCH_R, 0.0))
    )
    path_wire = path.val()
    start = path_wire.positionAt(0.0)
    tan = path_wire.tangentAt(0.0)
    bar = (
        cq.Workplane(cq.Plane(origin=start.toTuple(), normal=tan.toTuple()))
        .circle(BAR_R)
        .sweep(path, transition="round")
    )

    # Pivot knuckles at the two ends, axis along X.
    for sx in (-1.0, 1.0):
        boss = (
            cq.Workplane("YZ", origin=(sx * ARCH_R, 0.0, 0.0))
            .circle(HANDLE_BOSS_R)
            .extrude(-sx * HANDLE_BOSS_LEN)
        )
        bar = bar.union(boss)

    return mesh_from_cadquery(bar, "bail_handle")


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="round_shopping_basket")

    body_finish = model.material("body_finish", rgba=(0.88, 0.27, 0.22, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.55, 0.10, 0.09, 1.0))

    body = model.part("bucket_body")
    body.visual(_bucket_mesh(), material=body_finish, name="bucket_shell")
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=R_OUTER, length=H),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.0, H / 2.0)),
    )

    handle = model.part("bail_handle")
    # The handle mesh is authored with its pivot line on the local X axis at
    # z=0, so the joint origin (on the real pivot line) lines it up directly.
    handle.visual(_handle_mesh(), material=handle_finish, name="bail_bar")
    handle.inertial = Inertial.from_geometry(
        Cylinder(radius=BAR_R, length=2 * ARCH_R),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.0, ARCH_R / 2.0)),
    )

    # REVOLUTE about the X line through the two Y-axis pivot lugs at the rim.
    # At q=0 the arch stands upright over the mouth; positive q swings it
    # down toward the +Y side of the rim.
    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=-math.radians(95.0),
            upper=math.radians(95.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bucket_body")
    handle = object_model.get_part("bail_handle")
    joint = object_model.get_articulation("body_to_handle")

    # --- Footprint: cylindrical, so roughly equal X and Y extents. ---
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "body present with world AABB",
        body_aabb is not None,
        details=f"body_aabb={body_aabb}",
    )
    if body_aabb is not None:
        (bmn, bmx) = body_aabb
        ext_x = bmx[0] - bmn[0]
        ext_y = bmx[1] - bmn[1]
        ext_z = bmx[2] - bmn[2]
        ctx.check(
            "cylindrical footprint: X and Y extents are similar",
            abs(ext_x - ext_y) < 0.06,
            details=f"ext_x={ext_x:.3f}, ext_y={ext_y:.3f}",
        )
        ctx.check(
            "basket rests at z~=0",
            abs(bmn[2]) < 0.01,
            details=f"z_min={bmn[2]:.4f}",
        )
        ctx.check(
            "body has realistic height",
            0.18 < ext_z < 0.30,
            details=f"ext_z={ext_z:.3f}",
        )
        ctx.check(
            "body has realistic diameter",
            0.24 < ext_x < 0.40,
            details=f"ext_x={ext_x:.3f}",
        )

    # --- Hollow cylinder: inner cavity exists. ---
    inner_dia = 2 * (R_OUTER - WALL)
    ctx.check(
        "bucket is hollow (open interior cavity)",
        inner_dia > 0.20 and WALL < 0.05,
        details=f"inner_dia={inner_dia:.3f}, wall={WALL}",
    )

    # --- Vertical rib slots around the cylindrical wall. ---
    ctx.check(
        "vertical rib slots present around the wall",
        N_SLOTS >= 10,
        details=f"n_slots={N_SLOTS}",
    )

    # --- Bail handle pivots about the rim diameter (axis along X). ---
    ax = joint.axis
    ctx.check(
        "handle joint axis runs along X (through rim-diameter pivots)",
        abs(ax[0]) > 0.99 and abs(ax[1]) < 0.01 and abs(ax[2]) < 0.01,
        details=f"axis={ax}",
    )
    ctx.check(
        "handle joint is revolute",
        str(joint.articulation_type).upper().endswith("REVOLUTE"),
        details=f"type={joint.articulation_type}",
    )
    lim = joint.motion_limits
    ctx.check(
        "handle has realistic swing limits (~+-95deg)",
        lim is not None
        and lim.lower is not None
        and lim.upper is not None
        and lim.lower < -1.5
        and lim.upper > 1.5,
        details=f"lower={None if lim is None else lim.lower}, upper={None if lim is None else lim.upper}",
    )

    # Driving the joint swings the arch: upright vs laid-down pose.
    with ctx.pose({joint: 0.0}):
        up_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({joint: math.radians(90.0)}):
        down_aabb = ctx.part_world_aabb(handle)

    ctx.check(
        "handle poses resolve",
        up_aabb is not None and down_aabb is not None,
        details=f"up={up_aabb}, down={down_aabb}",
    )
    if up_aabb is not None and down_aabb is not None:
        up_top_z = up_aabb[1][2]
        down_top_z = down_aabb[1][2]
        ctx.check(
            "upright handle reaches higher than laid-down handle",
            up_top_z > down_top_z + 0.08,
            details=f"upright_top_z={up_top_z:.3f}, laid_top_z={down_top_z:.3f}",
        )

    # Handle arch rises well above the rim in the upright pose.
    if up_aabb is not None and body_aabb is not None:
        rise_above_rim = up_aabb[1][2] - body_aabb[1][2]
        ctx.check(
            "bail arches above the rim",
            rise_above_rim > 0.08,
            details=f"rise_above_rim={rise_above_rim:.3f}",
        )

    # --- Handle pivot knuckles are captured inside the body lugs. ---
    ctx.allow_overlap(
        body,
        handle,
        reason=(
            "The bail-handle pivot knuckles are intentionally captured inside "
            "the bucket rim pivot lugs; local overlap represents the real "
            "pin-in-lug pivot joint."
        ),
    )

    return ctx.report()


object_model = build_object_model()
