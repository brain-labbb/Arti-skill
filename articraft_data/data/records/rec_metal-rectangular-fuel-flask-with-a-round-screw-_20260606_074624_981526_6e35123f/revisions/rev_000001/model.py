from __future__ import annotations

# Metal rectangular fuel flask (flat hip-flask / jerry-tin canister).
# Frame: flask stands upright, height along +Z, width along +X, depth along +Y.
#   - body centerline at x=0, y=0; bottom rests at z=0, top of body at z~0.130.
#   - flat broad faces are the +Y (front, labelled) and -Y (back) sides.
#   - a short threaded neck rises from the top deck near the +X corner; the
#     round screw cap seats on it. A small folding bail ring sits beside the
#     cap on a low mount.
# Articulations:
#   - cap_rotate (CONTINUOUS) + cap_slide (PRISMATIC) about/along the +Z neck
#     axis through a massless carrier: the cap unscrews (spins) and lifts off.
#   - bail_flip (REVOLUTE): the carry ring flips up/down about its mount,
#     0 (lying flat) to ~90 deg (standing up).

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key dimensions (meters) ----
BODY_W = 0.130   # x extent
BODY_D = 0.060   # y extent
BODY_H = 0.130   # z extent of the main body
EDGE_R = 0.012   # rounded vertical-edge radius
TOP_R = 0.006    # rounded top/bottom edge radius (fillet)
WALL = 0.0035    # sheet-metal wall thickness
FLOOR = 0.004    # interior floor thickness
TOP_PLATE = 0.0035  # closed deck thickness below the neck

NECK_X = 0.040   # neck sits toward the +X side of the top deck
NECK_Y = 0.0
NECK_R = 0.0150
NECK_IR = 0.0105
NECK_H = 0.013
NECK_TOP_Z = BODY_H + NECK_H  # top plane of the neck where cap seats

CAP_R = 0.0185
CAP_H = 0.018

RING_X = -0.020  # bail mount sits toward the -X side of the top deck
RING_Y = 0.0
RING_TUBE = 0.0028
RING_RAD = 0.014


def _flask_body() -> cq.Workplane:
    # Flat rectangular canister with rounded vertical edges, sheet-metal wall
    # thickness, an internal cavity, and a real through-mouth under the neck.
    body = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(EDGE_R)
    )
    # Soften the top and bottom horizontal edges a touch.
    body = body.edges("|X or |Y").fillet(TOP_R)

    inner = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, FLOOR))
        .box(
            BODY_W - 2 * WALL,
            BODY_D - 2 * WALL,
            BODY_H - FLOOR - TOP_PLATE,
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(max(EDGE_R - WALL, 0.002))
    )
    mouth = (
        cq.Workplane("XY")
        .transformed(offset=(NECK_X, NECK_Y, BODY_H - TOP_PLATE - 0.001))
        .circle(NECK_IR)
        .extrude(TOP_PLATE + NECK_H + 0.004)
    )
    body = body.cut(inner).cut(mouth)
    return body


def _neck_solid() -> cq.Workplane:
    # Short threaded neck rising from the top deck, with an open bore that
    # aligns with the body mouth and exposes the hollow cavity below.
    neck = (
        cq.Workplane("XY")
        .transformed(offset=(NECK_X, NECK_Y, BODY_H))
        .circle(NECK_R)
        .circle(NECK_IR)
        .extrude(NECK_H)
    )
    return neck


def _cap_mesh():
    # Round knurled screw cap: a short cylinder with knurl ridges around the
    # rim and a small grip flange. Cap-local frame origin is its seating plane
    # (bottom of the cap), so it sits directly on top of the neck.
    cap = CylinderGeometry(CAP_R, CAP_H, radial_segments=48)
    cap.translate(0.0, 0.0, CAP_H / 2.0)
    # top dome lip
    lip = TorusGeometry(CAP_R - 0.002, 0.0022, radial_segments=12, tubular_segments=48)
    lip.translate(0.0, 0.0, CAP_H)
    cap.merge(lip)
    # knurl ridges around the side
    n = 20
    for i in range(n):
        a = 2.0 * math.pi * i / n
        ridge = BoxGeometry((0.0024, 0.0024, CAP_H * 0.8))
        ridge.translate(
            (CAP_R - 0.0008) * math.cos(a),
            (CAP_R - 0.0008) * math.sin(a),
            CAP_H / 2.0,
        )
        cap.merge(ridge)
    return mesh_from_geometry(cap, "cap_body")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fuel_flask")

    metal = model.material("brushed_metal", rgba=(0.62, 0.64, 0.67, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.40, 0.42, 0.45, 1.0))
    label_white = model.material("label_white", rgba=(0.93, 0.93, 0.90, 1.0))
    marker_red = model.material("marker_red", rgba=(0.82, 0.16, 0.14, 1.0))

    # ---- body (root): flask shell + neck + bail mount + front label ----
    body = model.part("body")

    shell = _flask_body().union(_neck_solid())
    body.visual(mesh_from_cadquery(shell, "flask_shell"), material=metal, name="flask_shell")

    # Bail mount: a small raised boss on the top deck that carries the ring pivot.
    body.visual(
        Box((0.020, 0.014, 0.008)),
        origin=Origin(xyz=(RING_X, RING_Y, BODY_H + 0.004)),
        material=dark_metal,
        name="bail_mount",
    )

    # Printed front label panel on the +Y broad face.
    body.visual(
        Box((0.058, 0.0015, 0.080)),
        origin=Origin(xyz=(0.0, BODY_D / 2.0 + 0.0006, 0.060)),
        material=label_white,
        name="front_label",
    )

    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H)), mass=0.55, origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0))
    )

    # ---- screw cap: decoupled rotate + slide through a massless carrier ----
    carrier = model.part("cap_carrier")  # NO visuals (massless)
    carrier.inertial = Inertial.from_geometry(Box((0.008, 0.008, 0.008)), mass=1e-4)

    cap = model.part("cap")
    cap.visual(_cap_mesh(), material=dark_metal, name="cap_body")
    # Off-axis marker so spin about +Z is observable.
    cap.visual(
        Box((0.004, 0.004, 0.004)),
        origin=Origin(xyz=(CAP_R - 0.003, 0.0, CAP_H + 0.002)),
        material=marker_red,
        name="cap_marker",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_H), mass=0.02, origin=Origin(xyz=(0.0, 0.0, CAP_H / 2.0))
    )

    model.articulation(
        "cap_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(NECK_X, NECK_Y, NECK_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )
    model.articulation(
        "cap_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=CAP_H, effort=1.0, velocity=1.0),
    )

    # ---- bail ring: folding carry ring that flips about its mount ----
    bail = model.part("bail_ring")
    # Open carry ring (torus) lying flat in the XY plane at rest, with its hinge
    # end at the pivot line (ring-local y=0) and the loop extending toward +Y.
    # Positive rotation about the +X pivot lifts the loop upright.
    ring = TorusGeometry(RING_RAD, RING_TUBE, radial_segments=12, tubular_segments=40)
    ring.translate(0.0, RING_RAD, 0.0)  # loop extends toward +Y from the pivot
    bail.visual(mesh_from_geometry(ring, "ring_loop"), material=dark_metal, name="ring_loop")
    bail.inertial = Inertial.from_geometry(
        Box((0.004, 2 * RING_RAD, 0.004)), mass=0.01,
        origin=Origin(xyz=(0.0, RING_RAD, 0.0)),
    )

    # Pivot axis is along +X. At q=0 the ring lies flat toward +Y on the top
    # deck; positive q swings the loop up toward +Z (standing upright).
    model.articulation(
        "bail_flip",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bail,
        origin=Origin(xyz=(RING_X, RING_Y, BODY_H + 0.008)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=0.0, upper=math.pi / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    carrier = object_model.get_part("cap_carrier")
    cap = object_model.get_part("cap")
    bail = object_model.get_part("bail_ring")
    cap_rotate = object_model.get_articulation("cap_rotate")
    cap_slide = object_model.get_articulation("cap_slide")
    bail_flip = object_model.get_articulation("bail_flip")

    def ext(part):
        mn, mx = ctx.part_world_aabb(part)
        return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])

    # ---- flask is flat and rectangular: width > depth, and reasonably tall. ----
    bx, by, bz = ext(body)
    ctx.check(
        "flask body is flat (width greater than depth)",
        bx > by + 0.03,
        details=f"body extents={bx, by, bz}",
    )
    ctx.check(
        "flask body is upright and tall",
        bz > 0.10,
        details=f"body extents={bx, by, bz}",
    )

    # ---- cap sits on top of the flask, above the body. ----
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap is mounted at the top of the flask",
        cap_pos is not None and cap_pos[2] > BODY_H - 0.001,
        details=f"cap origin={cap_pos}",
    )
    ctx.check(
        "neck bore penetrates the top deck into a hollow body cavity",
        0.0 < NECK_IR < NECK_R
        and BODY_H - FLOOR - TOP_PLATE > BODY_H * 0.80
        and TOP_PLATE < NECK_H,
        details=(
            f"neck_ir={NECK_IR}, neck_r={NECK_R}, "
            f"cavity_h={BODY_H - FLOOR - TOP_PLATE}, top_plate={TOP_PLATE}"
        ),
    )
    # Cap seats over the neck (intentional local overlap of cap skirt + neck).
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_body",
        elem_b="flask_shell",
        reason="The screw cap skirt is intentionally seated over the threaded neck.",
    )

    # ---- cap spins about +Z: the off-axis marker moves around the axis. ----
    m0 = ctx.part_element_world_aabb(cap, elem="cap_marker")
    mc0 = ((m0[0][0] + m0[1][0]) / 2.0, (m0[0][1] + m0[1][1]) / 2.0)
    with ctx.pose({cap_rotate: math.pi / 2.0}):
        m1 = ctx.part_element_world_aabb(cap, elem="cap_marker")
        mc1 = ((m1[0][0] + m1[1][0]) / 2.0, (m1[0][1] + m1[1][1]) / 2.0)
    moved = math.hypot(mc1[0] - mc0[0], mc1[1] - mc0[1])
    ctx.check(
        "cap marker sweeps around the neck axis when spun",
        moved > 0.01,
        details=f"marker rest={mc0}, spun={mc1}, moved={moved}",
    )

    # ---- cap slides up off the neck along +Z. ----
    rest_z = ctx.part_world_position(cap)[2]
    with ctx.pose({cap_slide: CAP_H}):
        lifted_z = ctx.part_world_position(cap)[2]
    ctx.check(
        "cap slides up off the neck",
        lifted_z > rest_z + CAP_H * 0.5,
        details=f"rest_z={rest_z}, lifted_z={lifted_z}",
    )

    # ---- bail ring flips up about its mount. ----
    r0 = ext(bail)
    z0 = ctx.part_world_aabb(bail)[1][2]  # top of ring at rest (flat)
    with ctx.pose({bail_flip: math.pi / 2.0}):
        z1 = ctx.part_world_aabb(bail)[1][2]  # top of ring when upright
    ctx.check(
        "bail ring flips up about its mount",
        z1 > z0 + 0.01,
        details=f"rest top_z={z0}, flipped top_z={z1}, rest extents={r0}",
    )
    # Ring pivot boss/ring overlap is intentional at the mount.
    ctx.allow_overlap(
        bail,
        body,
        elem_a="ring_loop",
        elem_b="bail_mount",
        reason="The bail ring's hinge end is intentionally captured by the mount boss.",
    )
    ctx.expect_contact(bail, body, name="bail ring attached to its mount")

    return ctx.report()


object_model = build_object_model()
