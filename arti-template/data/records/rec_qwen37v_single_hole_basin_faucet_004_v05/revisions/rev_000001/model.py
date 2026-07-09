from __future__ import annotations

"""Single-hole basin faucet with cylindrical column and gently curved downward spout.

Variant 05: fork from polished-chrome rectangular faucet into a compact
single-hole basin faucet with:
- cylindrical column (not rectangular)
- tubular spout sweeping forward and gently downward
- real hollow outlet at the spout mouth
- separate circular aerator insert that flips open on a tiny hinge

Layout (meters, +Z up, ground at z=0, spout extends along +X):
- Circular stepped base plate carries a slim cylindrical column.
- A tubular spout sweeps forward and gently downward from the column.
- A hollow outlet at the spout mouth with a separate circular aerator insert.
- The aerator flips open on a tiny hinge (revolute, 0..90 deg).
- A lever handle lifts for flow (revolute, 0..25 deg).
"""

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ----------------------------------------------------------------------------
# Key dimensions (meters)
# ----------------------------------------------------------------------------
BASE_LOWER_R = 0.045
BASE_LOWER_H = 0.006
BASE_UPPER_R = 0.034
BASE_UPPER_H = 0.012
BASE_TOP_Z = BASE_LOWER_H + BASE_UPPER_H  # 0.018

COLUMN_R = 0.018
COLUMN_TOP_Z = 0.235

# Spout tube
SPOUT_R = 0.013           # outer radius of tubular spout
SPOUT_WALL = 0.003        # wall thickness
SPOUT_START_Z = COLUMN_TOP_Z - 0.015  # 0.220 — spout emerges near column top
SPOUT_TIP_X = 0.170       # forward reach
SPOUT_TIP_Z = 0.180       # gentle downward curve endpoint

# Aerator
AERATOR_R = SPOUT_R - SPOUT_WALL - 0.001  # 0.009 — fits inside bore
AERATOR_THICK = 0.003

# Handle
POST_R = 0.012
POST_H = 0.015
POST_TOP_Z = COLUMN_TOP_Z + POST_H  # 0.250

HANDLE_LEN = 0.130
HANDLE_W = 0.038
HANDLE_T = 0.010
HANDLE_FLOAT = 0.004      # blade floats above post top

# Joint ranges
LIFT_RANGE = math.radians(25.0)
AERATOR_RANGE = math.radians(90.0)

# Hinge position: top of spout mouth
HINGE_X = SPOUT_TIP_X
HINGE_Z = SPOUT_TIP_Z + SPOUT_R  # 0.193


def _build_spout_path():
    """Create the spout sweep path as a gentle arc on the XZ plane."""
    return (
        cq.Workplane("XZ")
        .moveTo(0.0, SPOUT_START_Z)
        .threePointArc(
            ((SPOUT_TIP_X) * 0.5, SPOUT_START_Z - 0.010),  # midpoint bows slightly up
            (SPOUT_TIP_X, SPOUT_TIP_Z),                      # endpoint
        )
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    chrome = model.material("polished_chrome", rgba=(0.82, 0.84, 0.88, 1.0))
    dark = model.material("outlet_dark", rgba=(0.08, 0.08, 0.10, 1.0))
    mesh_mat = model.material("aerator_mesh", rgba=(0.45, 0.47, 0.50, 1.0))

    # ==================================================================
    # faucet_body (root): base, column, spout, post, hinge barrel
    # ==================================================================
    body = model.part("faucet_body")

    # --- Circular stepped base plate ---
    body.visual(
        Cylinder(radius=BASE_LOWER_R, length=BASE_LOWER_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_LOWER_H / 2.0)),
        material=chrome, name="base_lower",
    )
    body.visual(
        Cylinder(radius=BASE_UPPER_R, length=BASE_UPPER_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_LOWER_H + BASE_UPPER_H / 2.0)),
        material=chrome, name="base_upper",
    )

    # --- Cylindrical column ---
    col_h = COLUMN_TOP_Z - BASE_TOP_Z
    body.visual(
        Cylinder(radius=COLUMN_R, length=col_h),
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z + col_h / 2.0)),
        material=chrome, name="column",
    )
    # Column cap to close the top visually
    body.visual(
        Cylinder(radius=COLUMN_R, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_TOP_Z + 0.0015)),
        material=chrome, name="column_cap",
    )

    # --- Curved tubular spout (swept annular profile along spline path) ---
    path_wp = _build_spout_path()

    # Annular profile at path start (x=0, z=SPOUT_START_Z on YZ plane)
    spout_profile = (
        cq.Workplane("YZ")
        .center(0.0, SPOUT_START_Z)
        .circle(SPOUT_R)
        .circle(SPOUT_R - SPOUT_WALL)
    )
    spout_solid = spout_profile.sweep(path_wp)
    body.visual(
        mesh_from_cadquery(spout_solid, "spout_tube"),
        material=chrome, name="spout_tube",
    )

    # Spout collar at column junction (visual ring where spout emerges)
    collar = (
        cq.Workplane("YZ")
        .center(0.0, SPOUT_START_Z)
        .circle(SPOUT_R + 0.004)
        .circle(SPOUT_R)
        .extrude(0.008)
    )
    body.visual(
        mesh_from_cadquery(collar, "spout_collar"),
        material=chrome, name="spout_collar",
    )

    # Dark outlet ring seated inside the spout mouth (connects to inner tube wall)
    outlet_ring = (
        cq.Workplane("YZ")
        .center(0.0, SPOUT_TIP_Z)
        .circle(SPOUT_R - 0.001)  # 0.012 — embeds into tube wall for connectivity
        .circle(SPOUT_R - SPOUT_WALL - 0.003)
        .extrude(0.005)
    )
    outlet_ring = outlet_ring.translate((SPOUT_TIP_X - 0.006, 0.0, 0.0))
    body.visual(
        mesh_from_cadquery(outlet_ring, "outlet_ring"),
        material=dark, name="outlet_bore",
    )

    # --- Mounting post on column top ---
    body.visual(
        Cylinder(radius=POST_R, length=POST_H),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_TOP_Z + POST_H / 2.0)),
        material=chrome, name="mounting_post",
    )

    # --- Hinge barrel at spout mouth top (for aerator) ---
    # Barrel embeds slightly into the tube wall for connectivity
    hinge_barrel_len = 2.0 * (SPOUT_R - 0.002)
    barrel_embed_z = SPOUT_TIP_Z + SPOUT_R - 0.001  # 1mm below tube crown
    body.visual(
        Cylinder(radius=0.002, length=hinge_barrel_len),
        origin=Origin(
            xyz=(HINGE_X - 0.002, 0.0, barrel_embed_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=chrome, name="hinge_barrel",
    )

    # ==================================================================
    # lever_handle: flow control lever
    # ==================================================================
    handle = model.part("lever_handle")

    # Handle blade extends forward from pivot
    handle.visual(
        Box((HANDLE_LEN, HANDLE_W, HANDLE_T)),
        origin=Origin(xyz=(HANDLE_LEN / 2.0, 0.0, HANDLE_T / 2.0)),
        material=chrome, name="handle_blade",
    )
    # Pivot heel bridges from blade bottom down toward post top
    heel_h = HANDLE_FLOAT + 0.003
    handle.visual(
        Box((0.016, 0.026, heel_h)),
        origin=Origin(xyz=(0.008, 0.0, -heel_h / 2.0)),
        material=chrome, name="pivot_heel",
    )

    model.articulation(
        "handle_lift",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(-COLUMN_R, 0.0, POST_TOP_Z + HANDLE_FLOAT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0, velocity=3.0, lower=0.0, upper=LIFT_RANGE
        ),
    )

    # ==================================================================
    # aerator_insert: circular disc that flips on hinge
    # ==================================================================
    aerator = model.part("aerator_insert")

    # Aerator disc: thin cylinder perpendicular to spout axis (along X)
    # In part-local frame (origin at hinge), disc hangs down at z = -SPOUT_R
    aerator.visual(
        Cylinder(radius=AERATOR_R, length=AERATOR_THICK),
        origin=Origin(
            xyz=(0.0, 0.0, -SPOUT_R),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=mesh_mat, name="aerator_disc",
    )
    # Connecting arm from hinge knuckle down to the disc (bracket)
    arm_h = SPOUT_R
    aerator.visual(
        Box((0.004, 0.005, arm_h)),
        origin=Origin(xyz=(0.0, 0.0, -arm_h / 2.0)),
        material=chrome, name="hinge_arm",
    )
    # Hinge knuckle on aerator (mates with body hinge barrel)
    aerator.visual(
        Cylinder(radius=0.0018, length=0.006),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=chrome, name="hinge_knuckle",
    )

    model.articulation(
        "aerator_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=aerator,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        # axis (0, -1, 0): positive q flips disc forward/outward from mouth
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=2.0, lower=0.0, upper=AERATOR_RANGE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    handle = object_model.get_part("lever_handle")
    aerator = object_model.get_part("aerator_insert")
    lift = object_model.get_articulation("handle_lift")
    hinge = object_model.get_articulation("aerator_hinge")

    # --- Joint plan ---
    ctx.check(
        "aerator hinge is revolute 0..90 deg about horizontal Y axis",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and abs(hinge.axis[0]) < 1e-9
        and abs(abs(hinge.axis[1]) - 1.0) < 1e-9
        and abs(hinge.axis[2]) < 1e-9
        and hinge.motion_limits is not None
        and abs(hinge.motion_limits.lower) < 1e-9
        and abs(hinge.motion_limits.upper - math.radians(90.0)) < 1e-6,
        details=f"axis={hinge.axis}, limits={hinge.motion_limits}",
    )
    ctx.check(
        "handle lift is revolute 0..25 deg about horizontal Y axis",
        lift.articulation_type == ArticulationType.REVOLUTE
        and abs(lift.axis[0]) < 1e-9
        and abs(abs(lift.axis[1]) - 1.0) < 1e-9
        and abs(lift.axis[2]) < 1e-9
        and lift.motion_limits is not None
        and abs(lift.motion_limits.lower) < 1e-9
        and abs(lift.motion_limits.upper - math.radians(25.0)) < 1e-6,
        details=f"axis={lift.axis}, limits={lift.motion_limits}",
    )

    # --- Spout curves downward (variant-specific) ---
    spout_aabb = ctx.part_element_world_aabb(body, elem="spout_tube")
    col_cap_aabb = ctx.part_element_world_aabb(body, elem="column_cap")
    ctx.check(
        "spout tip is lower than column top (downward curve)",
        spout_aabb is not None
        and col_cap_aabb is not None
        and spout_aabb[0][2] < col_cap_aabb[0][2] - 0.02,
        details=f"spout_min_z={spout_aabb[0][2] if spout_aabb else None}, "
                f"col_cap_min_z={col_cap_aabb[0][2] if col_cap_aabb else None}",
    )

    # --- Hollow outlet at spout mouth (variant-specific) ---
    bore_aabb = ctx.part_element_world_aabb(body, elem="outlet_bore")
    ctx.check(
        "dark outlet bore exists inside the spout mouth region",
        bore_aabb is not None
        and bore_aabb[1][0] > SPOUT_TIP_X - 0.02
        and bore_aabb[0][0] < SPOUT_TIP_X,
        details=f"bore_aabb={bore_aabb}",
    )

    # --- Separate aerator insert part (variant-specific) ---
    aerator_disc_aabb = ctx.part_element_world_aabb(aerator, elem="aerator_disc")
    ctx.check(
        "aerator insert is a separate part with a visible disc",
        aerator.name != body.name
        and aerator_disc_aabb is not None,
        details=f"aerator={aerator.name}, body={body.name}",
    )

    # --- Grounding and proportions ---
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "base plate is grounded at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-6,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "spout cantilevers forward of the column",
        body_aabb is not None and body_aabb[1][0] > COLUMN_R + 0.10,
        details=f"body max x={body_aabb[1][0] if body_aabb else None}",
    )

    # --- Hinge barrel and knuckle share the hinge axis ---
    barrel_aabb = ctx.part_element_world_aabb(body, elem="hinge_barrel")
    knuckle_aabb = ctx.part_element_world_aabb(aerator, elem="hinge_knuckle")
    ctx.check(
        "hinge barrel and knuckle are co-located at the spout mouth",
        barrel_aabb is not None
        and knuckle_aabb is not None
        and abs(barrel_aabb[0][2] - knuckle_aabb[0][2]) < 0.005
        and abs(barrel_aabb[1][0] - knuckle_aabb[1][0]) < 0.005,
        details=f"barrel={barrel_aabb}, knuckle={knuckle_aabb}",
    )

    # --- Intentional overlap: hinge knuckle inside barrel ---
    ctx.allow_overlap(
        body, aerator,
        elem_a="hinge_barrel", elem_b="hinge_knuckle",
        reason="Hinge knuckle is intentionally nested inside the hinge barrel "
               "to represent the pivot pin joint.",
    )

    # --- Decisive pose: aerator hinge opens ---
    rest_aerator_aabb = ctx.part_world_aabb(aerator)
    rest_aerator_center_z = (
        (rest_aerator_aabb[0][2] + rest_aerator_aabb[1][2]) / 2.0
        if rest_aerator_aabb else None
    )
    with ctx.pose({hinge: AERATOR_RANGE}):
        open_aabb = ctx.part_world_aabb(aerator)
        open_center_z = (
            (open_aabb[0][2] + open_aabb[1][2]) / 2.0
            if open_aabb else None
        )
        ctx.check(
            "positive hinge angle flips aerator disc upward/outward",
            rest_aerator_center_z is not None
            and open_center_z is not None
            and open_center_z > rest_aerator_center_z + 0.003,
            details=f"rest_center_z={rest_aerator_center_z}, open_center_z={open_center_z}",
        )

    # --- Decisive pose: handle lifts ---
    rest_handle_aabb = ctx.part_world_aabb(handle)
    rest_handle_top = rest_handle_aabb[1][2] if rest_handle_aabb else None
    with ctx.pose({lift: LIFT_RANGE}):
        lifted_aabb = ctx.part_world_aabb(handle)
        ctx.check(
            "positive lift raises the handle grip upward",
            rest_handle_top is not None
            and lifted_aabb is not None
            and lifted_aabb[1][2] > rest_handle_top + 0.02,
            details=f"rest_top={rest_handle_top}, lifted_aabb={lifted_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
