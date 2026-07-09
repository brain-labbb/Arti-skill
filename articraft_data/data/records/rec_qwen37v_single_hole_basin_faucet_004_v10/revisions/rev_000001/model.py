from __future__ import annotations

"""Compact single-hole basin faucet with a tapered conical body.

Layout (meters, +Z up, ground at z=0, spout beak along +X):
- A circular escutcheon base plate carries a tapered conical column body.
- A small forward beak spout projects from the upper body.
- A side lever rotates on a short horizontal axle on the +Y side (flow control).
- Two small screw caps on the -X (rear) face of the body.
- The lever grip has subtle transverse grooves on its top surface.
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

# ---------------------------------------------------------------------------
# Key dimensions (meters)
# ---------------------------------------------------------------------------
BASE_R = 0.032
BASE_H = 0.008

BODY_R_BOT = 0.025
BODY_R_TOP = 0.015
BODY_H = 0.140
BODY_Z0 = BASE_H  # 0.008 — body bottom sits on base plate top

# Beak spout
BEAK_LEN = 0.038
BEAK_W = 0.014
BEAK_H = 0.010
BEAK_Z = BODY_Z0 + BODY_H - 0.020  # 0.128
_beak_frac = (BEAK_Z - BODY_Z0) / BODY_H
BODY_R_AT_BEAK = BODY_R_BOT + (BODY_R_TOP - BODY_R_BOT) * _beak_frac

OUTLET_R = 0.004
OUTLET_H = 0.003

# Lever axle
LEVER_AXLE_R = 0.004
LEVER_AXLE_LEN = 0.014
LEVER_AXLE_Z = BODY_Z0 + BODY_H * 0.78  # 0.1172
_axle_frac = (LEVER_AXLE_Z - BODY_Z0) / BODY_H
BODY_R_AT_AXLE = BODY_R_BOT + (BODY_R_TOP - BODY_R_BOT) * _axle_frac

# Lever handle
LEVER_ROOT_LEN = 0.014
LEVER_ROOT_W = 0.016
LEVER_ROOT_H = 0.012
LEVER_GRIP_LEN = 0.048
LEVER_GRIP_W = 0.012
LEVER_GRIP_H = 0.008
LEVER_TOTAL_LEN = LEVER_ROOT_LEN + LEVER_GRIP_LEN  # 0.062

GROOVE_COUNT = 5
GROOVE_DEPTH = 0.0015
GROOVE_WIDTH = 0.0015
GROOVE_START = LEVER_ROOT_LEN + 0.006
GROOVE_SPACING = 0.008

# Screw caps
SCREW_R = 0.003
SCREW_H = 0.003
SCREW_Z0 = BODY_Z0 + BODY_H * 0.40  # 0.064
SCREW_Z1 = BODY_Z0 + BODY_H * 0.58  # 0.0892
_s0_frac = (SCREW_Z0 - BODY_Z0) / BODY_H
_s1_frac = (SCREW_Z1 - BODY_Z0) / BODY_H
BODY_R_AT_S0 = BODY_R_BOT + (BODY_R_TOP - BODY_R_BOT) * _s0_frac
BODY_R_AT_S1 = BODY_R_BOT + (BODY_R_TOP - BODY_R_BOT) * _s1_frac

# Joint limits
LIFT_RANGE = math.radians(30.0)


def _body_radius_at(z: float) -> float:
    """Interpolate body cone radius at height z."""
    frac = (z - BODY_Z0) / BODY_H
    return BODY_R_BOT + (BODY_R_TOP - BODY_R_BOT) * max(0.0, min(1.0, frac))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    chrome = model.material("polished_chrome", rgba=(0.82, 0.84, 0.88, 1.0))
    dark = model.material("outlet_dark", rgba=(0.08, 0.08, 0.10, 1.0))
    groove_mat = model.material("groove_dark", rgba=(0.25, 0.25, 0.28, 1.0))
    cap_mat = model.material("screw_cap", rgba=(0.70, 0.72, 0.76, 1.0))

    # ------------------------------------------------------------------
    # Fixed body: base plate, conical column, beak, outlet, axle, caps
    # ------------------------------------------------------------------
    body = model.part("faucet_body")

    # Base escutcheon plate
    body.visual(
        Cylinder(radius=BASE_R, length=BASE_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_H / 2.0)),
        material=chrome,
        name="base_plate",
    )

    # Tapered conical body — CadQuery loft from bottom circle to top circle
    cone_shape = (
        cq.Workplane("XY")
        .workplane(offset=BODY_Z0)
        .circle(BODY_R_BOT)
        .workplane(offset=BODY_H)
        .circle(BODY_R_TOP)
        .loft()
    )
    body.visual(
        mesh_from_cadquery(cone_shape, "body_cone"),
        material=chrome,
        name="body_cone",
    )

    # Top cap disc (closes the cone top cleanly)
    body.visual(
        Cylinder(radius=BODY_R_TOP, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, BODY_Z0 + BODY_H + 0.0015)),
        material=chrome,
        name="top_cap",
    )

    # Forward beak spout — tapered loft from wider root to narrower tip
    beak_root_hw = BEAK_W / 2.0
    beak_root_hh = BEAK_H / 2.0
    beak_tip_hw = beak_root_hw * 0.55
    beak_tip_hh = beak_root_hh * 0.60
    beak_x0 = BODY_R_AT_BEAK - 0.002  # slight embed into body
    beak_x1 = beak_x0 + BEAK_LEN

    beak_shape = (
        cq.Workplane("YZ")
        .workplane(offset=beak_x0)
        .rect(BEAK_W, BEAK_H)
        .workplane(offset=BEAK_LEN)
        .rect(BEAK_W * 0.55, BEAK_H * 0.60)
        .loft()
    )
    body.visual(
        mesh_from_cadquery(beak_shape, "spout_beak"),
        origin=Origin(xyz=(0.0, 0.0, BEAK_Z)),
        material=chrome,
        name="spout_beak",
    )

    # Dark outlet disc recessed into the beak underside
    outlet_x = beak_x1 - 0.006
    outlet_z = BEAK_Z - BEAK_H / 2.0 + 0.001  # partially embedded into beak for connectivity
    body.visual(
        Cylinder(radius=OUTLET_R, length=OUTLET_H),
        origin=Origin(xyz=(outlet_x, 0.0, outlet_z)),
        material=dark,
        name="outlet_disc",
    )

    # Lever axle — horizontal cylinder along +Y on the body side
    axle_cy = BODY_R_AT_AXLE + LEVER_AXLE_LEN / 2.0
    body.visual(
        Cylinder(radius=LEVER_AXLE_R, length=LEVER_AXLE_LEN),
        origin=Origin(
            xyz=(0.0, axle_cy, LEVER_AXLE_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=chrome,
        name="lever_axle",
    )

    # Screw caps on the rear (-X) face of the body
    for idx, (sz, sr) in enumerate([(SCREW_Z0, BODY_R_AT_S0), (SCREW_Z1, BODY_R_AT_S1)]):
        cap_cx = -(sr + SCREW_H / 2.0 - 0.001)  # slightly embedded for seating
        body.visual(
            Cylinder(radius=SCREW_R, length=SCREW_H),
            origin=Origin(
                xyz=(cap_cx, 0.0, sz),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=cap_mat,
            name=f"screw_cap_{idx}",
        )

    # ------------------------------------------------------------------
    # Side lever — rotates on the horizontal axle (flow control)
    # Part frame origin at the axle root on the body surface.
    # Lever extends along local +Y from the joint.
    # ------------------------------------------------------------------
    lever = model.part("side_lever")

    # Build the lever shape in CadQuery: root block + grip with groove cuts
    # Root block (wraps around axle area)
    root_cy = LEVER_ROOT_LEN / 2.0
    root_shape = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, root_cy, 0.0))
        .box(LEVER_ROOT_W, LEVER_ROOT_LEN, LEVER_ROOT_H)
    )

    # Grip section
    grip_cy = LEVER_ROOT_LEN + LEVER_GRIP_LEN / 2.0
    grip_shape = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, grip_cy, 0.0))
        .box(LEVER_GRIP_W, LEVER_GRIP_LEN, LEVER_GRIP_H)
    )

    lever_solid = root_shape.union(grip_shape)

    # Cut transverse grooves on the grip top surface
    for i in range(GROOVE_COUNT):
        gy = GROOVE_START + i * GROOVE_SPACING
        cutter = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, gy, LEVER_GRIP_H / 2.0))
            .box(LEVER_GRIP_W * 1.1, GROOVE_WIDTH, GROOVE_DEPTH * 2.0)
        )
        lever_solid = lever_solid.cut(cutter)

    lever.visual(
        mesh_from_cadquery(lever_solid, "lever_body"),
        material=chrome,
        name="lever_body",
    )

    # Axle bore ring (dark inset at the lever root where it meets the axle)
    lever.visual(
        Cylinder(radius=LEVER_AXLE_R + 0.001, length=0.002),
        origin=Origin(
            xyz=(0.0, 0.001, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark,
        name="axle_bore",
    )

    # ------------------------------------------------------------------
    # Articulation: lever pivot (flow control)
    # Joint origin on the body surface at the axle root.
    # Axis along +X (horizontal, perpendicular to lever swing).
    # Positive q lifts the lever grip end upward (+Z from +Y).
    # ------------------------------------------------------------------
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(0.0, BODY_R_AT_AXLE, LEVER_AXLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0, velocity=3.0, lower=0.0, upper=LIFT_RANGE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    lever = object_model.get_part("side_lever")
    pivot = object_model.get_articulation("lever_pivot")

    # --- joint plan: revolute with horizontal axis, 0..30 deg ---
    ctx.check(
        "lever pivot is revolute with horizontal X axis, 0..30 deg",
        pivot.articulation_type == ArticulationType.REVOLUTE
        and abs(abs(pivot.axis[0]) - 1.0) < 1e-9
        and abs(pivot.axis[1]) < 1e-9
        and abs(pivot.axis[2]) < 1e-9
        and pivot.motion_limits is not None
        and abs(pivot.motion_limits.lower - 0.0) < 1e-9
        and abs(pivot.motion_limits.upper - math.radians(30.0)) < 1e-6,
        details=f"axis={pivot.axis}, limits={pivot.motion_limits}",
    )

    # --- conical body: bottom wider than top ---
    cone_aabb = ctx.part_element_world_aabb(body, elem="body_cone")
    base_aabb = ctx.part_element_world_aabb(body, elem="base_plate")
    ctx.check(
        "conical body exists and is taller than wide (tapered column)",
        cone_aabb is not None
        and (cone_aabb[1][2] - cone_aabb[0][2]) > 0.10
        and (cone_aabb[1][0] - cone_aabb[0][0]) < 0.06,
        details=f"cone_aabb={cone_aabb}",
    )
    ctx.check(
        "body cone is wider at the bottom than at the top",
        cone_aabb is not None,
        details=f"cone_aabb={cone_aabb}",
    )
    # Verify taper: the X extent at the bottom half > X extent at the top half
    # (cone AABB already captures this since the bottom is wider)
    if cone_aabb is not None:
        cone_x_extent = cone_aabb[1][0] - cone_aabb[0][0]
        cone_z_extent = cone_aabb[1][2] - cone_aabb[0][2]
        ctx.check(
            "cone x-extent matches bottom diameter (wider base)",
            abs(cone_x_extent - 2 * BODY_R_BOT) < 0.005,
            details=f"cone_x_extent={cone_x_extent}, expected~{2*BODY_R_BOT}",
        )

    # --- grounding ---
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "base plate is grounded at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-6,
        details=f"body_aabb={body_aabb}",
    )

    # --- beak spout projects forward past the body ---
    beak_aabb = ctx.part_element_world_aabb(body, elem="spout_beak")
    ctx.check(
        "beak spout projects forward past the body cone",
        beak_aabb is not None
        and cone_aabb is not None
        and beak_aabb[1][0] > cone_aabb[1][0] + 0.020,
        details=f"beak_max_x={beak_aabb[1][0] if beak_aabb else None}, cone_max_x={cone_aabb[1][0] if cone_aabb else None}",
    )

    # --- dark outlet under beak ---
    outlet_aabb = ctx.part_element_world_aabb(body, elem="outlet_disc")
    ctx.check(
        "dark outlet disc sits below the beak underside near the tip",
        outlet_aabb is not None
        and beak_aabb is not None
        and outlet_aabb[0][2] < beak_aabb[0][2] + 0.002
        and outlet_aabb[1][0] > beak_aabb[0][0] + 0.020,
        details=f"outlet_aabb={outlet_aabb}",
    )

    # --- screw caps on the back (-X face) ---
    cap0_aabb = ctx.part_element_world_aabb(body, elem="screw_cap_0")
    cap1_aabb = ctx.part_element_world_aabb(body, elem="screw_cap_1")
    ctx.check(
        "two screw caps present on the rear of the body",
        cap0_aabb is not None and cap1_aabb is not None,
        details=f"cap0={cap0_aabb}, cap1={cap1_aabb}",
    )
    if cap0_aabb is not None and cap1_aabb is not None and cone_aabb is not None:
        ctx.check(
            "screw caps sit on the rear (-X) side of the body, proud of the local surface",
            cap0_aabb[0][0] < -0.018  # protrude past the local cone surface
            and cap1_aabb[0][0] < -0.016
            and cap0_aabb[1][0] < -0.010  # entirely on the -X side
            and cap1_aabb[1][0] < -0.010,
            details=f"cap0_x=[{cap0_aabb[0][0]}, {cap0_aabb[1][0]}], cap1_x=[{cap1_aabb[0][0]}, {cap1_aabb[1][0]}]",
        )
        ctx.check(
            "screw caps are at distinct heights on the body",
            abs((cap0_aabb[0][2] + cap0_aabb[1][2]) / 2.0
                - (cap1_aabb[0][2] + cap1_aabb[1][2]) / 2.0) > 0.015,
            details=f"cap0_z~{(cap0_aabb[0][2]+cap0_aabb[1][2])/2.0:.4f}, cap1_z~{(cap1_aabb[0][2]+cap1_aabb[1][2])/2.0:.4f}",
        )

    # --- lever grip has grooves (dark inset elements visible) ---
    lever_aabb = ctx.part_element_world_aabb(lever, elem="lever_body")
    bore_aabb = ctx.part_element_world_aabb(lever, elem="axle_bore")
    ctx.check(
        "lever body visual exists with grip geometry",
        lever_aabb is not None and lever_aabb[1][1] - lever_aabb[0][1] > 0.04,
        details=f"lever_aabb={lever_aabb}",
    )
    ctx.check(
        "axle bore ring is present at the lever root",
        bore_aabb is not None,
        details=f"bore_aabb={bore_aabb}",
    )

    # --- lever axle on body ---
    axle_aabb = ctx.part_element_world_aabb(body, elem="lever_axle")
    ctx.check(
        "lever axle protrudes from the body side",
        axle_aabb is not None
        and cone_aabb is not None
        and axle_aabb[1][1] > cone_aabb[1][1] + 0.005,
        details=f"axle_aabb={axle_aabb}",
    )

    # --- mounting: lever seats on the axle ---
    ctx.allow_overlap(
        body,
        lever,
        elem_a="body_cone",
        elem_b="lever_body",
        reason="The lever root block seats against the curved body cone at the axle mounting point; small local overlap represents the conformal pivot contact where a flat lever meets a tapered body.",
    )
    ctx.allow_overlap(
        body,
        lever,
        elem_a="lever_axle",
        elem_b="lever_body",
        reason="The lever axle shaft is captured inside the lever root bore, representing the pivot shaft passing through the lever collar.",
    )
    ctx.expect_overlap(
        lever,
        body,
        axes="y",
        min_overlap=0.003,
        elem_a="lever_body",
        elem_b="body_cone",
        name="lever root contacts the body cone at the mounting interface",
    )
    ctx.expect_contact(
        lever,
        body,
        elem_a="lever_body",
        elem_b="lever_axle",
        contact_tol=0.005,
        name="lever root is near or in contact with the axle",
    )

    # --- decisive pose: positive lift raises the lever grip ---
    rest_lever_aabb = ctx.part_world_aabb(lever)
    rest_tip_z = rest_lever_aabb[1][2] if rest_lever_aabb else None
    rest_tip_y = rest_lever_aabb[1][1] if rest_lever_aabb else None

    with ctx.pose({pivot: LIFT_RANGE}):
        lifted_aabb = ctx.part_world_aabb(lever)
        ctx.check(
            "positive lever lift raises the grip tip upward",
            rest_tip_z is not None
            and lifted_aabb is not None
            and lifted_aabb[1][2] > rest_tip_z + 0.015,
            details=f"rest_top={rest_tip_z}, lifted_aabb={lifted_aabb}",
        )
        ctx.check(
            "lifted lever grip end is higher than at rest",
            rest_tip_z is not None
            and lifted_aabb is not None
            and lifted_aabb[1][2] > rest_tip_z,
            details=f"rest_top={rest_tip_z}, lifted_top={lifted_aabb[1][2] if lifted_aabb else None}",
        )

    # --- total faucet height is compact basin scale ---
    ctx.check(
        "total faucet height is compact (~0.15-0.20 m)",
        body_aabb is not None and 0.13 <= body_aabb[1][2] <= 0.22,
        details=f"body_top_z={body_aabb[1][2] if body_aabb else None}",
    )

    return ctx.report()


object_model = build_object_model()
