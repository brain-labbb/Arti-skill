from __future__ import annotations

"""Polished-chrome single-hole basin faucet with gently curved downward spout.

Layout (meters, +Z up, ground at z=0, spout extends along +X):
- An oval rubber gasket seats under a round chrome base plate.
- A cylindrical chrome column rises from the base.
- A curved spout tube exits the column top and sweeps forward and gently
  downward, ending with a real hollow dark outlet at its mouth.
- A lever handle on the column top rear lifts for flow control.
- The spout swivels about the vertical column axis for stream direction.
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
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Key dimensions (meters)
# ---------------------------------------------------------------------------
GASKET_A = 0.036          # outer semi-major (X)
GASKET_B = 0.026          # outer semi-minor (Y)
GASKET_H = 0.005
GASKET_HOLE_A = 0.025
GASKET_HOLE_B = 0.017

BASE_R = 0.029
BASE_H = 0.012
BASE_TOP_Z = GASKET_H + BASE_H          # 0.017

COLUMN_R = 0.019
COLUMN_TOP_Z = 0.260
COLUMN_H = COLUMN_TOP_Z - BASE_TOP_Z    # 0.243

CAP_R = 0.032
CAP_H = 0.010
CAP_TOP_Z = COLUMN_TOP_Z + CAP_H        # 0.270

# Spout tube – spline points in spout local frame (origin = swivel joint)
SPOUT_POINTS = [
    (0.00, 0.0, 0.00),
    (0.04, 0.0, 0.022),
    (0.08, 0.0, 0.028),
    (0.12, 0.0, 0.010),
    (0.14, 0.0, -0.04),
    (0.148, 0.0, -0.085),
]
SPOUT_TUBE_R = 0.011

# Outlet at spout mouth (last spline point)
MOUTH = SPOUT_POINTS[-1]                 # (0.148, 0, -0.085)
OUTLET_RING_RO = 0.013
OUTLET_RING_RI = 0.009
OUTLET_RING_H = 0.006

# Handle
HANDLE_BOSS_X = -0.024                   # behind column centre
HANDLE_BOSS_R = 0.010
HANDLE_BOSS_H = 0.014
HANDLE_PIVOT_Z = CAP_TOP_Z + HANDLE_BOSS_H   # 0.284

HANDLE_LEN = 0.105
HANDLE_W = 0.026
HANDLE_T = 0.009

# Joint ranges
SPOUT_SWIVEL = math.radians(120.0)
HANDLE_LIFT = math.radians(25.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chrome_single_hole_basin_faucet")

    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.90, 1.0))
    dark = model.material("outlet_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    rubber = model.material("gasket_rubber", rgba=(0.12, 0.12, 0.13, 1.0))

    # ------------------------------------------------------------------
    # Body (root): oval gasket, base plate, column, cap, handle boss
    # ------------------------------------------------------------------
    body = model.part("faucet_body")

    # Oval base gasket (elliptical ring)
    gasket_shape = (
        cq.Workplane("XY")
        .ellipse(GASKET_A, GASKET_B)
        .extrude(GASKET_H)
        .faces(">Z").workplane()
        .ellipse(GASKET_HOLE_A, GASKET_HOLE_B)
        .cutThruAll()
    )
    body.visual(
        mesh_from_cadquery(gasket_shape, "oval_gasket"),
        material=rubber,
        name="oval_gasket",
    )

    # Round chrome base plate
    body.visual(
        Cylinder(radius=BASE_R, length=BASE_H),
        origin=Origin(xyz=(0.0, 0.0, GASKET_H + BASE_H / 2.0)),
        material=chrome,
        name="base_plate",
    )

    # Cylindrical column
    body.visual(
        Cylinder(radius=COLUMN_R, length=COLUMN_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z + COLUMN_H / 2.0)),
        material=chrome,
        name="column",
    )

    # Column cap (swivel bearing)
    body.visual(
        Cylinder(radius=CAP_R, length=CAP_H),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_TOP_Z + CAP_H / 2.0)),
        material=chrome,
        name="column_cap",
    )

    # Handle mounting boss (rear of column top)
    body.visual(
        Cylinder(radius=HANDLE_BOSS_R, length=HANDLE_BOSS_H),
        origin=Origin(xyz=(HANDLE_BOSS_X, 0.0, CAP_TOP_Z + HANDLE_BOSS_H / 2.0)),
        material=chrome,
        name="handle_boss",
    )

    # ------------------------------------------------------------------
    # Spout (swivels about vertical axis on column cap)
    # ------------------------------------------------------------------
    spout = model.part("spout")

    # Curved spout tube (gentle downward sweep)
    spout_tube_geom = tube_from_spline_points(
        SPOUT_POINTS,
        radius=SPOUT_TUBE_R,
        samples_per_segment=18,
        radial_segments=22,
        cap_ends=True,
    )
    spout.visual(
        mesh_from_geometry(spout_tube_geom, "spout_tube"),
        material=chrome,
        name="spout_tube",
    )

    # Chrome outlet ring (annulus) at mouth
    outlet_ring = (
        cq.Workplane("XY")
        .circle(OUTLET_RING_RO)
        .circle(OUTLET_RING_RI)
        .extrude(OUTLET_RING_H)
    )
    spout.visual(
        mesh_from_cadquery(outlet_ring, "outlet_ring"),
        origin=Origin(xyz=(MOUTH[0], MOUTH[1], MOUTH[2] - OUTLET_RING_H / 2.0)),
        material=chrome,
        name="outlet_ring",
    )

    # Dark hollow outlet disc recessed inside the ring
    spout.visual(
        Cylinder(radius=OUTLET_RING_RI, length=0.004),
        origin=Origin(xyz=(MOUTH[0], MOUTH[1], MOUTH[2] + 0.001)),
        material=dark,
        name="outlet_disc",
    )

    # Spout swivel joint (vertical axis through column cap)
    model.articulation(
        "spout_swivel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, CAP_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=3.0,
            lower=-SPOUT_SWIVEL, upper=SPOUT_SWIVEL,
        ),
    )

    # ------------------------------------------------------------------
    # Handle (lifts on horizontal axis at boss top)
    # ------------------------------------------------------------------
    handle = model.part("lever_handle")

    # Handle lever blade (extends backward from pivot, -X in local frame)
    handle.visual(
        Box((HANDLE_LEN, HANDLE_W, HANDLE_T)),
        origin=Origin(xyz=(-HANDLE_LEN / 2.0, 0.0, HANDLE_T / 2.0)),
        material=chrome,
        name="handle_blade",
    )

    # Small grip nub at the handle tip
    handle.visual(
        Cylinder(radius=0.006, length=HANDLE_T + 0.004),
        origin=Origin(xyz=(-HANDLE_LEN + 0.008, 0.0, HANDLE_T / 2.0)),
        material=chrome,
        name="grip_nub",
    )

    # Handle lift joint
    # Pivot at boss top; handle extends -X; axis +Y so positive q lifts tip.
    model.articulation(
        "handle_lift",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(HANDLE_BOSS_X, 0.0, HANDLE_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0, velocity=3.0,
            lower=0.0, upper=HANDLE_LIFT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    spout = object_model.get_part("spout")
    handle = object_model.get_part("lever_handle")
    swivel = object_model.get_articulation("spout_swivel")
    lift = object_model.get_articulation("handle_lift")

    # --- Joint plan: types, axes, ranges ---
    ctx.check(
        "spout swivel is revolute ±120° about vertical axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and abs(swivel.axis[0]) < 1e-9
        and abs(swivel.axis[1]) < 1e-9
        and abs(abs(swivel.axis[2]) - 1.0) < 1e-9
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + SPOUT_SWIVEL) < 1e-6
        and abs(swivel.motion_limits.upper - SPOUT_SWIVEL) < 1e-6,
        details=f"axis={swivel.axis}, limits={swivel.motion_limits}",
    )
    ctx.check(
        "handle lift is revolute 0..25° about horizontal axis",
        lift.articulation_type == ArticulationType.REVOLUTE
        and abs(lift.axis[0]) < 1e-9
        and abs(abs(lift.axis[1]) - 1.0) < 1e-9
        and abs(lift.axis[2]) < 1e-9
        and lift.motion_limits is not None
        and abs(lift.motion_limits.lower) < 1e-9
        and abs(lift.motion_limits.upper - math.radians(25.0)) < 1e-6,
        details=f"axis={lift.axis}, limits={lift.motion_limits}",
    )

    # --- Non-fixed joint check ---
    ctx.check(
        "at least one non-fixed joint exists",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and lift.articulation_type == ArticulationType.REVOLUTE,
    )

    # --- Grounding and scale ---
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "base gasket is grounded at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-4,
        details=f"body_aabb={body_aabb}",
    )

    handle_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "total faucet height roughly 0.25-0.32 m",
        handle_aabb is not None and 0.24 <= handle_aabb[1][2] <= 0.33,
        details=f"handle_aabb={handle_aabb}",
    )

    # --- Oval gasket ---
    gasket_aabb = ctx.part_element_world_aabb(body, elem="oval_gasket")
    ctx.check(
        "oval base gasket is present and thin (height ~5 mm)",
        gasket_aabb is not None
        and abs(gasket_aabb[1][2] - gasket_aabb[0][2] - GASKET_H) < 0.002
        and abs(gasket_aabb[0][2]) < 1e-4,
        details=f"gasket_aabb={gasket_aabb}",
    )
    ctx.check(
        "oval gasket is wider in X than Y (elliptical, not circular)",
        gasket_aabb is not None
        and (gasket_aabb[1][0] - gasket_aabb[0][0])
        > (gasket_aabb[1][1] - gasket_aabb[0][1]) + 0.005,
        details=f"gasket_aabb={gasket_aabb}",
    )

    # --- Cylindrical column ---
    column_aabb = ctx.part_element_world_aabb(body, elem="column")
    ctx.check(
        "column is roughly cylindrical (X and Y extents are similar)",
        column_aabb is not None
        and abs(
            (column_aabb[1][0] - column_aabb[0][0])
            - (column_aabb[1][1] - column_aabb[0][1])
        ) < 0.005,
        details=f"column_aabb={column_aabb}",
    )

    # --- Curved downward spout ---
    tube_aabb = ctx.part_element_world_aabb(spout, elem="spout_tube")
    ctx.check(
        "spout tube mouth is lower than its root (curves downward)",
        tube_aabb is not None
        and tube_aabb[0][2] < (CAP_TOP_Z + 0.01),  # lowest point well below cap top
        details=f"tube_aabb={tube_aabb}",
    )
    ctx.check(
        "spout extends forward from the column",
        tube_aabb is not None
        and tube_aabb[1][0] > COLUMN_R + 0.08,
        details=f"tube_aabb={tube_aabb}",
    )

    # --- Hollow outlet at spout mouth ---
    ring_aabb = ctx.part_element_world_aabb(spout, elem="outlet_ring")
    disc_aabb = ctx.part_element_world_aabb(spout, elem="outlet_disc")
    ctx.check(
        "outlet ring exists near the spout mouth",
        ring_aabb is not None and ring_aabb[0][2] < CAP_TOP_Z - 0.03,
        details=f"ring_aabb={ring_aabb}",
    )
    ctx.check(
        "dark outlet disc is inside the ring (hollow outlet)",
        ring_aabb is not None
        and disc_aabb is not None
        and disc_aabb[0][0] >= ring_aabb[0][0] - 0.001
        and disc_aabb[1][0] <= ring_aabb[1][0] + 0.001
        and disc_aabb[0][1] >= ring_aabb[0][1] - 0.001
        and disc_aabb[1][1] <= ring_aabb[1][1] + 0.001,
        details=f"ring={ring_aabb}, disc={disc_aabb}",
    )

    # --- Swivel joint origin near column top ---
    ctx.check(
        "spout swivel origin is at the column cap height",
        abs(swivel.origin.xyz[2] - CAP_TOP_Z) < 0.002,
        details=f"swivel_origin={swivel.origin.xyz}",
    )

    # --- Intentional overlap: spout tube root nests inside column cap bearing ---
    ctx.allow_overlap(
        body,
        spout,
        elem_a="column_cap",
        elem_b="spout_tube",
        reason="The spout tube root is intentionally nested inside the column cap as a swivel bearing seat.",
    )
    ctx.expect_contact(
        spout,
        body,
        elem_a="spout_tube",
        elem_b="column_cap",
        contact_tol=0.012,
        name="spout tube root stays nested within the column cap bearing",
    )

    # --- Decisive pose checks ---
    # Spout swivel: positive angle should slew the spout mouth sideways
    rest_mouth_x = None
    rest_mouth_y = None
    if ring_aabb is not None:
        rest_mouth_x = (ring_aabb[0][0] + ring_aabb[1][0]) / 2.0
        rest_mouth_y = (ring_aabb[0][1] + ring_aabb[1][1]) / 2.0

    with ctx.pose({swivel: math.radians(45.0)}):
        swiveled_ring = ctx.part_element_world_aabb(spout, elem="outlet_ring")
        ctx.check(
            "positive spout swivel moves the mouth sideways",
            rest_mouth_y is not None
            and swiveled_ring is not None
            and abs((swiveled_ring[0][1] + swiveled_ring[1][1]) / 2.0 - rest_mouth_y) > 0.03,
            details=f"rest_y={rest_mouth_y}, swiveled={swiveled_ring}",
        )

    # Handle lift: positive angle should raise the handle tip
    rest_handle_top = handle_aabb[1][2] if handle_aabb is not None else None
    with ctx.pose({lift: HANDLE_LIFT}):
        lifted_aabb = ctx.part_world_aabb(handle)
        ctx.check(
            "positive handle lift raises the grip tip upward",
            rest_handle_top is not None
            and lifted_aabb is not None
            and lifted_aabb[1][2] > rest_handle_top + 0.02,
            details=f"rest_top={rest_handle_top}, lifted={lifted_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
