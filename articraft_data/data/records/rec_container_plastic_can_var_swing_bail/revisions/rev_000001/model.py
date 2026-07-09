from __future__ import annotations

# Black plastic square jerrycan / chemical pail (HDPE), near-cubic footprint.
# Frame: base on z=0, body grows up +Z; recessed top deck at ~0.27 m.
#   - chunky rounded-edge square body (CadQuery: filleted box) modeled hollow
#     with a real through pour-mouth at the neck
#   - moulded recessed top deck
#   - SWING BAIL carry handle: U-shaped molded grab bar mounted on two small
#     lugs on opposite shoulder sides, REVOLUTE joint so the bail swings up
#     to carry and folds down flat against the shoulder
#   - moulded horizontal stacking ribs (one band near the base, one mid-body)
#     plus a slightly flared foot
#   - an offset round threaded neck rising from the deck, beside the bail
# Articulations:
#   - bail_swing: body -> bail_handle, REVOLUTE about Y, 0 (flat) to π
#   - cap_rotate: body -> cap_carrier, CONTINUOUS +Z
#   - cap_slide:  cap_carrier -> cap, PRISMATIC +Z

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# --- key dimensions (meters) ---
BODY_W = 0.250  # square footprint (X and Y)
BODY_H = 0.300  # overall height (deck reads ~0.27, lug top ~0.30)
DECK_Z = 0.268  # recessed deck base height
WALL_EDGE_R = 0.024  # softly rounded vertical edges

NECK_X = 0.050  # neck offset from center toward +X (beside the bail)
NECK_Y = -0.010
NECK_R = 0.030
NECK_TOP_Z = 0.300  # top of the neck where the cap seats

CAP_R = 0.036
CAP_H = 0.030
CAP_SLIDE = 0.045  # cap can lift this far off the neck

# --- bail handle geometry ---
BAIL_X = -0.045       # bail pivot X (offset away from cap, same as parent bridge)
LUG_HALF_Y = 0.060    # bail pivot half-span along Y (distance from center to each lug)
LUG_W_X = 0.018       # lug width along X
LUG_W_Y = 0.014       # lug thickness along Y
LUG_H = 0.018         # lug height above shoulder top
BAIL_ARM_LEN = 0.080  # arm length from pivot to grab bar crossbar
BAIL_RADIUS = 0.005   # tube radius (molded plastic bail, not thin wire)

# Derived shoulder and pivot heights
SH_TOP = DECK_Z + 0.014          # shoulder top surface = 0.282
BAIL_PIVOT_Z = SH_TOP + LUG_H / 2.0  # pivot center height = 0.291


def _body_solid() -> cq.Workplane:
    half = BODY_W / 2.0

    # Main body block (z = 0 .. DECK_Z), softly rounded vertical edges.
    body = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_W, DECK_Z, centered=(True, True, False))
        .edges("|Z")
        .fillet(WALL_EDGE_R)
    )
    # Soften the top horizontal edges of the body into the shoulder.
    body = body.edges(">Z").fillet(0.012)

    # Shoulder slab on top, slightly inset, rounded all around.
    sh_w = BODY_W - 0.014
    sh_z0 = DECK_Z - 0.020
    sh_top = DECK_Z + 0.014
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=sh_z0)
        .box(sh_w, sh_w, sh_top - sh_z0, centered=(True, True, False))
        .edges("|Z")
        .fillet(WALL_EDGE_R)
        .edges(">Z")
        .fillet(0.014)
    )
    body = body.union(shoulder)

    # Recessed top deck: carve a broad shallow pocket into the shoulder so the
    # deck reads as a moulded recess. We leave a rim around the perimeter.
    deck_floor = DECK_Z - 0.004
    pocket = (
        cq.Workplane("XY")
        .workplane(offset=deck_floor)
        .box(sh_w - 0.030, sh_w - 0.030, sh_top - deck_floor + 0.01, centered=(True, True, False))
    )
    body = body.cut(pocket)

    # --- Two small mounting lugs on the shoulder for the swing bail handle ---
    # Each lug is supported by a column that bridges through the deck pocket
    # void to the body below, ensuring the lugs stay connected to the root body.
    for sign in (-1, 1):
        # Support column from below the deck floor up through the pocket void
        # to slightly above the shoulder top, merging with both the shoulder
        # remnant and the lug above.
        col_bottom = deck_floor - 0.005
        col_h = sh_top - col_bottom + 0.008  # extends slightly above shoulder
        col = (
            cq.Workplane("XY")
            .workplane(offset=col_bottom)
            .center(BAIL_X, sign * LUG_HALF_Y)
            .box(LUG_W_X, LUG_W_Y, col_h, centered=(True, True, False))
        )
        body = body.union(col)
        # Lug boss on top of the shoulder with rounded top edges
        lug = (
            cq.Workplane("XY")
            .workplane(offset=sh_top)
            .center(BAIL_X, sign * LUG_HALF_Y)
            .box(LUG_W_X, LUG_W_Y, LUG_H, centered=(True, True, False))
            .edges(">Z")
            .fillet(0.003)
        )
        body = body.union(lug)

    # Offset round threaded neck rising from the deck (toward +X, beside the bail).
    neck_base_z = deck_floor - 0.030
    boss = (
        cq.Workplane("XY")
        .workplane(offset=neck_base_z)
        .center(NECK_X, NECK_Y)
        .circle(NECK_R + 0.010)
        .extrude(0.020)
    )
    body = body.union(boss)
    neck = (
        cq.Workplane("XY")
        .workplane(offset=neck_base_z)
        .center(NECK_X, NECK_Y)
        .circle(NECK_R)
        .extrude(NECK_TOP_Z - neck_base_z)
    )
    body = body.union(neck)
    # Open pour mouth: bore the neck axis deep into the body so the mouth reads
    # as a real through opening (hollow), not a shallow recess under the cap.
    bore = (
        cq.Workplane("XY")
        .workplane(offset=0.170)
        .center(NECK_X, NECK_Y)
        .circle(NECK_R - 0.013)
        .extrude((NECK_TOP_Z + 0.002) - 0.170)
    )
    body = body.cut(bore)

    # Mid-body and base moulded stacking ribs (raised bands around the body).
    for zb, th in ((0.020, 0.012), (0.150, 0.010)):
        ridge = (
            cq.Workplane("XY")
            .workplane(offset=zb)
            .box(BODY_W + 0.008, BODY_W + 0.008, th, centered=(True, True, False))
            .edges("|Z")
            .fillet(WALL_EDGE_R)
        )
        body = body.union(ridge)

    # Slightly flared foot: a short wider band at the very bottom.
    foot = (
        cq.Workplane("XY")
        .box(BODY_W + 0.012, BODY_W + 0.012, 0.010, centered=(True, True, False))
        .edges("|Z")
        .fillet(WALL_EDGE_R)
    )
    body = body.union(foot)

    return body


def _bail_mesh() -> "object":
    """U-shaped swing bail handle: two arms + crossbar as a smooth swept tube.

    In the bail's local frame (at q=0, bail lies flat extending in -X):
      - Pivot ends at (0, ±LUG_HALF_Y, 0)
      - Arms extend to (-BAIL_ARM_LEN, ±LUG_HALF_Y, 0)
      - Crossbar midpoint at (-BAIL_ARM_LEN, 0, 0)
    """
    arm = BAIL_ARM_LEN
    ly = LUG_HALF_Y

    # Smooth U path: pivot -> arm -> rounded corner -> crossbar -> corner -> arm -> pivot
    points = [
        (0.0, -ly, 0.0),
        (-arm * 0.50, -ly, 0.0),
        (-arm * 0.85, -ly, 0.0),
        (-arm, -ly * 0.75, 0.0),
        (-arm, 0.0, 0.0),
        (-arm, ly * 0.75, 0.0),
        (-arm * 0.85, ly, 0.0),
        (-arm * 0.50, ly, 0.0),
        (0.0, ly, 0.0),
    ]

    geom = tube_from_spline_points(
        points,
        radius=BAIL_RADIUS,
        samples_per_segment=16,
        radial_segments=20,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )
    return mesh_from_geometry(geom, "bail_handle")


def _cap_mesh() -> "object":
    # Round black ribbed/knurled screw cap: a short cylinder with an inset top
    # disc and a ring of vertical knurl ribs around the skirt.
    cap = CylinderGeometry(CAP_R, CAP_H, radial_segments=48)
    cap.translate(0.0, 0.0, CAP_H / 2.0)
    # Slightly inset top disc.
    top = CylinderGeometry(CAP_R - 0.004, 0.006, radial_segments=48)
    top.translate(0.0, 0.0, CAP_H + 0.002)
    cap.merge(top)
    # Vertical grip ribs (knurl) around the skirt.
    n = 28
    for i in range(n):
        a = 2.0 * math.pi * i / n
        rib = BoxGeometry((0.004, 0.006, CAP_H - 0.006))
        rib.rotate_z(a)
        rib.translate(CAP_R * math.cos(a), CAP_R * math.sin(a), CAP_H / 2.0)
        cap.merge(rib)
    return mesh_from_geometry(cap, "cap")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_jerrycan_bail")

    hdpe_black = model.material("hdpe_black", rgba=(0.12, 0.12, 0.13, 1.0))
    cap_black = model.material("cap_black", rgba=(0.08, 0.08, 0.09, 1.0))
    bail_gray = model.material("bail_gray", rgba=(0.20, 0.20, 0.21, 1.0))
    marker_red = model.material("marker_red", rgba=(0.80, 0.10, 0.10, 1.0))

    # ---- body (root): chunky square pail + recessed deck + lugs + neck + ribs ----
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_solid(), "body_shell"),
        material=hdpe_black,
        name="body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_W, BODY_H)), mass=2.0, origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0))
    )

    # ---- swing bail handle: U-shaped tube mounted on the shoulder lugs ----
    bail = model.part("bail_handle")
    bail.visual(_bail_mesh(), material=bail_gray, name="bail_tube")
    bail.inertial = Inertial.from_geometry(
        Box((BAIL_ARM_LEN, 2 * LUG_HALF_Y, 2 * BAIL_RADIUS)),
        mass=0.08,
        origin=Origin(xyz=(-BAIL_ARM_LEN / 2.0, 0.0, 0.0)),
    )

    # Bail pivot: REVOLUTE about Y axis between the two lugs.
    # At q=0 the bail extends in -X (flat on shoulder); positive q swings it
    # upward (+Z) for carrying via right-hand rule about +Y.
    model.articulation(
        "bail_swing",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bail,
        origin=Origin(xyz=(BAIL_X, 0.0, BAIL_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=math.pi,
            effort=3.0,
            velocity=2.0,
        ),
    )

    # ---- decoupled screw cap: massless carrier rotates, cap slides on the carrier ----
    carrier = model.part("cap_carrier")  # NO visuals (massless)
    carrier.inertial = Inertial.from_geometry(Box((0.01, 0.01, 0.01)), mass=1e-4)

    cap = model.part("cap")
    cap.visual(_cap_mesh(), material=cap_black, name="cap_shell")
    # Off-axis marker so spin is provable.
    cap.visual(
        Box((0.008, 0.006, 0.006)),
        origin=Origin(xyz=(CAP_R + 0.003, 0.0, CAP_H - 0.004)),
        material=marker_red,
        name="cap_marker",
    )
    cap.inertial = Inertial.from_geometry(
        Box((2 * CAP_R, 2 * CAP_R, CAP_H)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, CAP_H / 2.0)),
    )

    model.articulation(
        "cap_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(NECK_X, NECK_Y, NECK_TOP_Z)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )
    model.articulation(
        "cap_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=cap,
        origin=Origin(xyz=(0, 0, 0.0)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(lower=0.0, upper=CAP_SLIDE, effort=1.0, velocity=1.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    bail = object_model.get_part("bail_handle")
    cap = object_model.get_part("cap")
    bail_swing = object_model.get_articulation("bail_swing")
    rotate = object_model.get_articulation("cap_rotate")
    slide = object_model.get_articulation("cap_slide")

    # --- body is a roughly cubic square pail of the right scale ---
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "body footprint is square (width ~ depth)",
        abs(bext[0] - bext[1]) < 0.03 and bext[0] > 0.23 and bext[1] > 0.23,
        details=f"body extents={bext}",
    )
    ctx.check(
        "body is roughly cubic (height within ~20% of width, not wildly tall)",
        abs(bext[2] - bext[0]) < 0.20 * bext[0] and bext[2] < bext[0] * 1.25,
        details=f"body w={bext[0]:.3f} d={bext[1]:.3f} h={bext[2]:.3f}",
    )

    # --- bail handle: U-shaped tube that swings up and folds down ---
    bail_ext = _ext(ctx.part_world_aabb(bail))
    ctx.check(
        "bail spans across the shoulder (Y extent covers lug spacing)",
        bail_ext[1] > 2 * LUG_HALF_Y * 0.85,
        details=f"bail extents={bail_ext}",
    )

    # Bail at rest (q=0) lies flat on the shoulder - AABB top stays low
    bail_rest_aabb = ctx.part_world_aabb(bail)
    ctx.check(
        "bail lies flat at rest (top of tube stays near pivot height)",
        bail_rest_aabb[1][2] < BAIL_PIVOT_Z + BAIL_RADIUS + 0.005,
        details=f"bail top z={bail_rest_aabb[1][2]:.4f}, pivot_z={BAIL_PIVOT_Z:.3f}",
    )
    ctx.check(
        "flat bail extends in -X direction from pivot (arm reach)",
        bail_rest_aabb[0][0] < BAIL_X - BAIL_ARM_LEN * 0.7,
        details=f"bail min_x={bail_rest_aabb[0][0]:.4f}",
    )

    # Bail swung up (q=pi/2) is upright for carrying
    with ctx.pose({bail_swing: math.pi / 2.0}):
        bail_up_aabb = ctx.part_world_aabb(bail)
    ctx.check(
        "bail swings up for carrying (top rises well above pivot)",
        bail_up_aabb[1][2] > BAIL_PIVOT_Z + BAIL_ARM_LEN * 0.7,
        details=f"upright bail top z={bail_up_aabb[1][2]:.4f}, pivot_z={BAIL_PIVOT_Z:.3f}",
    )
    ctx.check(
        "upright bail top extends above the body top for hand clearance",
        bail_up_aabb[1][2] > BODY_H,
        details=f"bail top z={bail_up_aabb[1][2]:.3f}, body_h={BODY_H}",
    )

    # Bail folded past upright (q=pi) extends in +X direction (opposite side)
    with ctx.pose({bail_swing: math.pi}):
        bail_folded_aabb = ctx.part_world_aabb(bail)
    bail_folded_cx = (bail_folded_aabb[0][0] + bail_folded_aabb[1][0]) / 2.0
    ctx.check(
        "bail folds to opposite side at full swing (X center shifts past pivot)",
        bail_folded_cx > BAIL_X + 0.010,
        details=f"folded bail center_x={bail_folded_cx:.4f}, pivot_x={BAIL_X}",
    )

    # Bail tube ends are captured in the shoulder lugs as pivot bearings
    ctx.allow_overlap(
        bail, body,
        elem_a="bail_tube",
        elem_b="body_shell",
        reason="Bail tube ends are captured in the shoulder lugs as pivot bearings.",
    )

    # --- cap is round and seated on the offset neck at rest, near the top ---
    cext = _ext(ctx.part_world_aabb(cap))
    ctx.check(
        "cap is round (square XY bbox)",
        abs(cext[0] - cext[1]) < 0.006,
        details=f"cap extents={cext}",
    )
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap is offset on the top toward +X (not centered)",
        cap_pos is not None and cap_pos[0] > 0.03,
        details=f"cap origin={cap_pos}",
    )
    ctx.check(
        "cap sits near the top of the body",
        cap_pos is not None and cap_pos[2] > 0.26,
        details=f"cap z={cap_pos[2] if cap_pos else None}",
    )
    ctx.check(
        "cap is offset clear of the bail handle pivot",
        cap_pos is not None and (cap_pos[0] - BAIL_X) > 0.06,
        details=f"cap_x={cap_pos[0]:.3f}, bail_x={BAIL_X}",
    )

    # Cap rim seated over neck top (threaded engagement)
    ctx.allow_overlap(
        cap, body,
        elem_a="cap_shell",
        elem_b="body_shell",
        reason="Screw cap skirt is threaded down over the neck top (seated engagement).",
    )

    # --- cap rotates about +Z: the off-axis red marker swings around ---
    m_rest = ctx.part_element_world_aabb(cap, elem="cap_marker")
    m_rest_c = ((m_rest[0][0] + m_rest[1][0]) / 2.0, (m_rest[0][1] + m_rest[1][1]) / 2.0)
    with ctx.pose({rotate: math.pi / 2.0}):
        m_spun = ctx.part_element_world_aabb(cap, elem="cap_marker")
    m_spun_c = ((m_spun[0][0] + m_spun[1][0]) / 2.0, (m_spun[0][1] + m_spun[1][1]) / 2.0)
    ctx.check(
        "cap spins about +Z (marker swings from +X toward +Y)",
        m_spun_c[1] > m_rest_c[1] + 0.02 and abs(m_spun_c[0] - NECK_X) < abs(m_rest_c[0] - NECK_X),
        details=f"marker rest_center={m_rest_c}, spun_center={m_spun_c}, neck_x={NECK_X}",
    )

    # --- cap slides up off the neck along +Z (independent of rotation) ---
    z_rest = ctx.part_world_position(cap)[2]
    with ctx.pose({slide: CAP_SLIDE}):
        z_up = ctx.part_world_position(cap)[2]
    ctx.check(
        "cap slides up off the neck",
        z_up > z_rest + CAP_SLIDE * 0.8,
        details=f"z_rest={z_rest:.3f}, z_up={z_up:.3f}",
    )

    # --- joints are decoupled: rotating does not move it up ---
    with ctx.pose({rotate: math.pi}):
        z_after_spin = ctx.part_world_position(cap)[2]
    ctx.check(
        "rotation does not change cap height (decoupled)",
        abs(z_after_spin - z_rest) < 0.002,
        details=f"z_rest={z_rest:.3f}, z_after_spin={z_after_spin:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
