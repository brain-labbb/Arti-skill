from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# End-cutting nippers with slip-joint pivot and dual grip sleeves.
#
# Two forged steel halves cross at a pivot pin. Each half has a jaw head
# extending forward (+X) that is WIDER in Y than long in X (the hallmark
# of end-cutting nippers: the cutting edges run across the jaw face
# perpendicular to the handle axis). The lower half carries a round pivot
# hole; the upper half has a short elongated slot so the pin can translate
# along the handle axis (slip joint). A polished rivet pin with circular
# caps on both sides captures the assembly.
#
# Layout (closed-design local frame, before rest-pose yaw):
#   - Both jaw heads extend in +X from the pivot.
#   - Both handles extend in -X from the pivot.
#   - Lower half jaw cutting edge at y ≈ +EDGE_LAND.
#   - Upper half jaw cutting edge at y ≈ -EDGE_LAND.
#   - Rest pose (q=0): each half is yawed by ±HALF_OPEN so the cutting
#     edges are apart and the handles splay.
#   - Closing (positive q): upper jaw swings toward lower jaw, cutting
#     edges meet; handles scissor together.
#
# Articulation chain:
#   lower_half --(PRISMATIC along X)--> pivot_pin --(REVOLUTE Z)--> upper_half
# ---------------------------------------------------------------------------

HALF_OPEN = math.radians(12.5)
CLOSE_TRAVEL = 2.0 * HALF_OPEN

PLATE_T = 0.0038
ZL0, ZL1 = 0.0015, 0.0015 + PLATE_T  # lower plate: 0.0015..0.0053
ZU0, ZU1 = ZL1, ZL1 + PLATE_T         # upper plate: 0.0053..0.0091
FULL_Z0, FULL_Z1 = ZL0, ZU1           # full-height jaw region

EDGE_LAND = 0.0003

# Boss and pin
BOSS_R = 0.009
PIN_R = 0.0025
CAP_R = 0.0055
CAP_T = 0.0020
HOLE_R = PIN_R + 0.0003

# Slip-joint slot
SLOT_HALF_LEN = 0.004
SLOT_HALF_W = PIN_R + 0.0003
SLIP_TRAVEL = 0.006

# Jaw head: WIDER in Y than in X (end-cutter signature)
JAW_X_START = 0.006   # near boss
JAW_X_END = 0.016     # tip (short in X)
JAW_HALF_Y = 0.012    # half-width in Y (wide!)

# Grip sleeve boundaries (in local frame, before yaw)
GRIP_INNER_X0 = -0.026
GRIP_INNER_X1 = -0.050
GRIP_OUTER_X0 = -0.054
GRIP_OUTER_X1 = -0.088
GRIP_H = 0.011
GRIP_W = 0.014


def _mirror_y(pts):
    return [(x, -y) for (x, y) in pts]


def _poly_prism(pts, z0, z1):
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, z0))
        .polyline(pts)
        .close()
        .extrude(z1 - z0)
    )


def _spline_wire(pts, z, periodic=True, inset=0.0):
    edge = cq.Edge.makeSpline(
        [cq.Vector(x, y, z) for (x, y) in pts], periodic=periodic
    )
    wire = cq.Wire.assembleEdges([edge])
    if inset:
        wire = wire.offset2D(-inset)[0]
    return wire


def _soft_prism(pts, z0, z1, cap=0.0018, inset=0.0015):
    mid = cq.Solid.extrudeLinear(
        cq.Face.makeFromWires(_spline_wire(pts, z0 + cap)),
        cq.Vector(0.0, 0.0, (z1 - z0) - 2.0 * cap),
    )
    top = cq.Solid.makeLoft(
        [_spline_wire(pts, z1 - cap), _spline_wire(pts, z1, inset=inset)]
    )
    bot = cq.Solid.makeLoft(
        [_spline_wire(pts, z0, inset=inset), _spline_wire(pts, z0 + cap)]
    )
    return cq.Workplane(obj=mid.fuse(top).fuse(bot))


def _open_spline_prism(pts, z0, z1):
    """Non-periodic spline prism (no overshoot past endpoints)."""
    vecs = [cq.Vector(x, y, z0) for (x, y) in pts]
    edge = cq.Edge.makeSpline(vecs, periodic=False)
    wire = cq.Wire.assembleEdges([edge])
    face = cq.Face.makeFromWires(wire)
    return cq.Workplane(obj=cq.Solid.extrudeLinear(face, cq.Vector(0.0, 0.0, z1 - z0)))


# ---- Jaw head: wide block extending forward (+X), cutting edge near y=0 ----

def _jaw_head_pts(s):
    """Jaw head outline. s=+1 for lower (cutting edge at +EDGE_LAND),
    s=-1 for upper (cutting edge at -EDGE_LAND).
    The jaw is WIDER in Y (2*JAW_HALF_Y = 18mm) than long in X (14mm)."""
    inner_y = s * EDGE_LAND
    outer_y = s * JAW_HALF_Y
    return [
        (JAW_X_START, inner_y),
        (JAW_X_START, outer_y),
        (JAW_X_END - 0.002, outer_y),
        (JAW_X_END, outer_y - s * 0.002),
        (JAW_X_END, inner_y + s * 0.002),
        (JAW_X_END - 0.003, inner_y),
    ]


def _jaw_head(s, z0, z1):
    """Full-height jaw head with beveled cutting face."""
    pts = _jaw_head_pts(s)
    body = _poly_prism(pts, z0, z1)
    # Bevel the outer top/bottom edges for a softer look
    try:
        body = body.edges("|Z").fillet(0.0012)
    except Exception:
        pass
    return body


# ---- Neck/tang: connects boss to handle ----

def _neck_pts(s):
    """Steel neck from boss area to handle start."""
    y_inner = s * 0.002
    y_outer = s * 0.008
    return [
        (-0.002, y_inner),
        (0.004, y_inner),
        (0.004, y_outer * 0.6),
        (-0.002, y_outer),
        (-0.012, y_outer),
        (-0.020, y_outer * 0.9),
        (-0.022, y_outer * 0.7),
        (-0.022, y_inner * 1.5),
        (-0.012, y_inner * 1.2),
    ]


# ---- Handle steel core ----

def _handle_core_pts(s):
    """Steel tang under the grip sleeves."""
    y_off = s * 0.005
    return [
        (-0.020, y_off + 0.002),
        (-0.020, y_off - 0.004),
        (-0.040, y_off - 0.005),
        (-0.060, y_off - 0.006),
        (-0.080, y_off - 0.007),
        (-0.092, y_off - 0.006),
        (-0.094, y_off - 0.003),
        (-0.094, y_off + 0.000),
        (-0.092, y_off + 0.002),
        (-0.080, y_off + 0.003),
        (-0.060, y_off + 0.003),
        (-0.040, y_off + 0.003),
    ]


# ---- Grip sleeves: two distinct colored sections ----

def _grip_inner_pts(s):
    """Inner grip sleeve (closer to pivot): from GRIP_INNER_X0 to GRIP_INNER_X1."""
    y_off = s * 0.005
    w = GRIP_W / 2
    return [
        (GRIP_INNER_X0, y_off + w * 0.85),
        (GRIP_INNER_X0, y_off - w * 0.85),
        (GRIP_INNER_X0 - 0.002, y_off - w * 0.95),
        ((GRIP_INNER_X0 + GRIP_INNER_X1) / 2, y_off - w),
        (GRIP_INNER_X1, y_off - w * 0.95),
        (GRIP_INNER_X1 + 0.001, y_off - w * 0.70),
        (GRIP_INNER_X1 + 0.001, y_off + w * 0.70),
        ((GRIP_INNER_X0 + GRIP_INNER_X1) / 2, y_off + w * 0.92),
    ]


def _grip_outer_pts(s):
    """Outer grip sleeve (closer to handle tip): from GRIP_OUTER_X0 to GRIP_OUTER_X1."""
    y_off = s * 0.005
    w = GRIP_W / 2
    return [
        (GRIP_OUTER_X0, y_off + w * 0.85),
        (GRIP_OUTER_X0, y_off - w * 0.95),
        ((GRIP_OUTER_X0 + GRIP_OUTER_X1) / 2, y_off - w * 1.05),
        (GRIP_OUTER_X1 + 0.002, y_off - w * 0.95),
        (GRIP_OUTER_X1, y_off - w * 0.60),
        (GRIP_OUTER_X1, y_off - w * 0.20),
        (GRIP_OUTER_X1 + 0.002, y_off + w * 0.40),
        ((GRIP_OUTER_X0 + GRIP_OUTER_X1) / 2, y_off + w * 0.88),
    ]


# ---- Pivot boss ----

def _boss(z0, z1, with_slot=False, with_hole=False):
    boss = cq.Workplane("XY", origin=(0.0, 0.0, z0)).circle(BOSS_R).extrude(z1 - z0)
    if with_hole:
        hole = (
            cq.Workplane("XY", origin=(0.0, 0.0, z0 - 0.001))
            .circle(HOLE_R)
            .extrude((z1 - z0) + 0.002)
        )
        boss = boss.cut(hole)
    if with_slot:
        slot = (
            cq.Workplane("XY", origin=(0.0, 0.0, z0 - 0.001))
            .slot2D(SLOT_HALF_LEN * 2, SLOT_HALF_W * 2)
            .extrude((z1 - z0) + 0.002)
        )
        boss = boss.cut(slot)
    return boss


# ---- Pivot pin with caps on both sides ----

def _pin_shank():
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, ZL0 - 0.001))
        .circle(PIN_R)
        .extrude((ZU1 - ZL0) + 0.002)
    )


def _pin_cap_bottom():
    cap = (
        cq.Workplane("XY", origin=(0.0, 0.0, ZL0 - CAP_T - 0.001))
        .circle(CAP_R)
        .extrude(CAP_T)
    )
    try:
        cap = cap.edges("<Z").fillet(0.0008)
    except Exception:
        pass
    return cap


def _pin_cap_top():
    cap = (
        cq.Workplane("XY", origin=(0.0, 0.0, ZU1 + 0.001))
        .circle(CAP_R)
        .extrude(CAP_T)
    )
    try:
        cap = cap.edges(">Z").fillet(0.0008)
    except Exception:
        pass
    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="end_cutting_nippers")

    steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    polished = model.material("polished_steel", rgba=(0.85, 0.87, 0.90, 1.0))
    grip_blue = model.material("grip_blue", rgba=(0.12, 0.32, 0.62, 1.0))
    grip_yellow = model.material("grip_yellow", rgba=(0.95, 0.82, 0.15, 1.0))

    lower_pose = Origin(rpy=(0.0, 0.0, HALF_OPEN))

    # ---- lower half (base link) ----
    lower = model.part("lower_half")

    lower.visual(
        mesh_from_cadquery(_jaw_head(+1, FULL_Z0, FULL_Z1), "lower_jaw"),
        origin=lower_pose,
        material=steel,
        name="jaw_head",
    )
    lower.visual(
        mesh_from_cadquery(_boss(ZL0, ZL1, with_hole=True), "lower_boss"),
        origin=lower_pose,
        material=steel,
        name="pivot_boss",
    )
    lower.visual(
        mesh_from_cadquery(_poly_prism(_neck_pts(+1), ZL0, ZL1), "lower_neck"),
        origin=lower_pose,
        material=steel,
        name="neck",
    )
    lower.visual(
        mesh_from_cadquery(
            _poly_prism(_handle_core_pts(+1), ZL0, ZL1), "lower_handle"
        ),
        origin=lower_pose,
        material=steel,
        name="handle_core",
    )
    lower.visual(
        mesh_from_cadquery(
            _poly_prism(_grip_inner_pts(+1), ZL0, ZL1 - 0.0003),
            "lower_grip_inner",
        ),
        origin=lower_pose,
        material=grip_blue,
        name="grip_inner",
    )
    lower.visual(
        mesh_from_cadquery(
            _poly_prism(_grip_outer_pts(+1), ZL0, ZL1 - 0.0003),
            "lower_grip_outer",
        ),
        origin=lower_pose,
        material=grip_yellow,
        name="grip_outer",
    )

    # ---- pivot pin (sliding in slip-joint slot) ----
    pin = model.part("pivot_pin")
    pin.visual(
        mesh_from_cadquery(_pin_shank(), "pin_shank"),
        material=polished,
        name="shank",
    )
    pin.visual(
        mesh_from_cadquery(_pin_cap_bottom(), "pin_cap_bottom"),
        material=polished,
        name="cap_bottom",
    )
    pin.visual(
        mesh_from_cadquery(_pin_cap_top(), "pin_cap_top"),
        material=polished,
        name="cap_top",
    )

    # ---- upper half (moving link) ----
    upper = model.part("upper_half")

    upper.visual(
        mesh_from_cadquery(_jaw_head(-1, FULL_Z0, FULL_Z1), "upper_jaw"),
        material=steel,
        name="jaw_head",
    )
    upper.visual(
        mesh_from_cadquery(_boss(ZU0, ZU1, with_slot=True), "upper_boss"),
        material=steel,
        name="pivot_boss",
    )
    upper.visual(
        mesh_from_cadquery(_poly_prism(_neck_pts(-1), ZU0, ZU1), "upper_neck"),
        material=steel,
        name="neck",
    )
    upper.visual(
        mesh_from_cadquery(
            _poly_prism(_handle_core_pts(-1), ZU0, ZU1), "upper_handle"
        ),
        material=steel,
        name="handle_core",
    )
    upper.visual(
        mesh_from_cadquery(
            _poly_prism(_grip_inner_pts(-1), ZU0 + 0.0003, ZU1),
            "upper_grip_inner",
        ),
        material=grip_blue,
        name="grip_inner",
    )
    upper.visual(
        mesh_from_cadquery(
            _poly_prism(_grip_outer_pts(-1), ZU0 + 0.0003, ZU1),
            "upper_grip_outer",
        ),
        material=grip_yellow,
        name="grip_outer",
    )

    # ---- Articulations ----

    # Prismatic slip joint: pin slides along X in the upper-half slot
    model.articulation(
        "slip_joint",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=pin,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -HALF_OPEN)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0, velocity=0.1, lower=0.0, upper=SLIP_TRAVEL
        ),
    )

    # Revolute jaw pivot: upper half rotates around Z on the pin
    model.articulation(
        "jaw_pivot",
        ArticulationType.REVOLUTE,
        parent=pin,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0, velocity=4.0, lower=0.0, upper=CLOSE_TRAVEL
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_half")
    upper = object_model.get_part("upper_half")
    pin = object_model.get_part("pivot_pin")
    slip = object_model.get_articulation("slip_joint")
    pivot = object_model.get_articulation("jaw_pivot")

    # --- Jaws wider in Y than X (end-cutter signature: perpendicular to handles) ---
    for part_obj in (lower, upper):
        jaw = ctx.part_element_world_aabb(part_obj, elem="jaw_head")
        handle = ctx.part_element_world_aabb(part_obj, elem="handle_core")
        if jaw is not None:
            jaw_dx = jaw[1][0] - jaw[0][0]
            jaw_dy = jaw[1][1] - jaw[0][1]
            ctx.check(
                f"{part_obj.name} jaw is wider in Y than X (end-cutter shape)",
                jaw_dy > jaw_dx,
                details=f"jaw dx={jaw_dx:.4f}, dy={jaw_dy:.4f}",
            )
        if handle is not None:
            handle_dx = handle[1][0] - handle[0][0]
            handle_dy = handle[1][1] - handle[0][1]
            ctx.check(
                f"{part_obj.name} handle extends more in X than Y",
                handle_dx > handle_dy * 2.0,
                details=f"handle dx={handle_dx:.4f}, dy={handle_dy:.4f}",
            )

    # --- Slip joint: prismatic articulation exists ---
    ctx.check(
        "slip_joint is prismatic",
        slip.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slip.articulation_type}",
    )
    ctx.check(
        "slip_joint has positive travel range",
        slip.motion_limits is not None
        and slip.motion_limits.upper is not None
        and slip.motion_limits.upper > 0.003,
        details=f"limits={slip.motion_limits}",
    )

    # --- Jaw pivot: revolute articulation ---
    ctx.check(
        "jaw_pivot is revolute",
        pivot.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={pivot.articulation_type}",
    )
    ctx.check(
        "jaw_pivot has ~25 degree travel",
        pivot.motion_limits is not None
        and pivot.motion_limits.upper is not None
        and 0.38 <= pivot.motion_limits.upper <= 0.48,
        details=f"limits={pivot.motion_limits}",
    )

    # --- Rivet caps on both sides ---
    cap_top = ctx.part_element_world_aabb(pin, elem="cap_top")
    cap_bot = ctx.part_element_world_aabb(pin, elem="cap_bottom")
    upper_boss = ctx.part_element_world_aabb(upper, elem="pivot_boss")
    lower_boss = ctx.part_element_world_aabb(lower, elem="pivot_boss")
    ctx.check(
        "rivet cap above upper boss",
        cap_top is not None
        and upper_boss is not None
        and cap_top[1][2] > upper_boss[1][2] - 0.0001,
        details=f"cap_top={cap_top}, upper_boss={upper_boss}",
    )
    ctx.check(
        "rivet cap below lower boss",
        cap_bot is not None
        and lower_boss is not None
        and cap_bot[0][2] < lower_boss[0][2] + 0.0001,
        details=f"cap_bot={cap_bot}, lower_boss={lower_boss}",
    )

    # --- Color-separated grip sleeves ---
    for part_obj in (lower, upper):
        gi = ctx.part_element_world_aabb(part_obj, elem="grip_inner")
        go = ctx.part_element_world_aabb(part_obj, elem="grip_outer")
        ctx.check(
            f"{part_obj.name} has inner grip sleeve",
            gi is not None,
            details="grip_inner missing",
        )
        ctx.check(
            f"{part_obj.name} has outer grip sleeve",
            go is not None,
            details="grip_outer missing",
        )
        if gi is not None and go is not None:
            # Inner sleeve center should be closer to pivot (less negative X)
            gi_cx = (gi[0][0] + gi[1][0]) / 2
            go_cx = (go[0][0] + go[1][0]) / 2
            ctx.check(
                f"{part_obj.name} inner grip closer to pivot than outer grip",
                gi_cx > go_cx,
                details=f"inner_cx={gi_cx:.4f}, outer_cx={go_cx:.4f}",
            )

    # --- Pin passes through boss ---
    ctx.expect_overlap(
        lower,
        pin,
        axes="xy",
        elem_a="pivot_boss",
        elem_b="shank",
        min_overlap=0.004,
        name="pin shank passes through lower boss",
    )

    # --- Rest pose: cutting edges open ---
    ctx.expect_gap(
        lower,
        upper,
        axis="y",
        positive_elem="jaw_head",
        negative_elem="jaw_head",
        min_gap=0.002,
        name="jaw cutting edges are open at rest",
    )

    # --- Closing: upper jaw swings toward lower jaw ---
    open_jaw_center = None
    closed_jaw_center = None
    jaw_aabb = ctx.part_element_world_aabb(upper, elem="jaw_head")
    if jaw_aabb is not None:
        open_jaw_center = [(jaw_aabb[0][i] + jaw_aabb[1][i]) / 2 for i in range(3)]

    with ctx.pose({pivot: CLOSE_TRAVEL}):
        jaw_aabb_c = ctx.part_element_world_aabb(upper, elem="jaw_head")
        if jaw_aabb_c is not None:
            closed_jaw_center = [
                (jaw_aabb_c[0][i] + jaw_aabb_c[1][i]) / 2 for i in range(3)
            ]

    ctx.check(
        "closing swings upper jaw toward lower jaw (Y center moves up)",
        open_jaw_center is not None
        and closed_jaw_center is not None
        and closed_jaw_center[1] > open_jaw_center[1] + 0.003,
        details=f"open_center={open_jaw_center}, closed_center={closed_jaw_center}",
    )

    # --- Slip joint: pin translates when posed ---
    pin_rest = ctx.part_world_position(pin)
    with ctx.pose({slip: SLIP_TRAVEL}):
        pin_slid = ctx.part_world_position(pin)
    ctx.check(
        "slip joint translates pin",
        pin_rest is not None
        and pin_slid is not None
        and (
            abs(pin_slid[0] - pin_rest[0]) > 0.002
            or abs(pin_slid[1] - pin_rest[1]) > 0.002
        ),
        details=f"rest={pin_rest}, slid={pin_slid}",
    )

    return ctx.report()


object_model = build_object_model()
