from __future__ import annotations

# Tall cylindrical pump bottle with a narrow neck, visible hollow mouth opening,
# transparent lip rim, and a pump-head actuator that slides down on a prismatic
# joint and rotates slightly on a revolute joint.
#
# Frame: vertical axis along +Z, bottle standing on z=0, centerline on x=y=0.
#   - rounded base -> tall straight cylindrical body -> short tapered shoulder
#     -> narrow neck -> transparent lip rim -> hollow mouth opening
#
# Articulations (two INDEPENDENT joints via a massless carrier):
#   - pump_rotate: REVOLUTE small-angle rotation of the pump head about +Z
#   - pump_slide:  PRISMATIC press-down of the pump head along +Z

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

# ---- key heights (m) along +Z ----
BASE_Z = 0.0
BODY_TOP_Z = 0.185       # end of straight cylindrical body
SHOULDER_TOP_Z = 0.205   # end of short taper
NECK_TOP_Z = 0.228       # top of neck tube
LIP_TOP_Z = 0.232        # lip rim extends slightly above neck

BODY_R = 0.030           # body outer radius (~60 mm dia)
NECK_R = 0.013           # neck outer radius
NECK_BORE_R = 0.010      # neck inner bore (mouth opening)
WALL = 0.0015            # wall thickness

# Pump dimensions
COLLAR_R = 0.016         # pump collar outer radius
COLLAR_H = 0.014         # collar height
STEM_R = 0.004           # pump stem radius
HEAD_R = 0.016           # pump head body radius
HEAD_H = 0.010           # pump head height
NOZZLE_LEN = 0.022       # nozzle spout length
NOZZLE_R = 0.003         # nozzle tube radius

# Pump mounts on top of collar, which sits at NECK_TOP_Z
COLLAR_BASE_Z = NECK_TOP_Z
PUMP_HEAD_BASE_Z = COLLAR_BASE_Z + COLLAR_H  # where the head assembly starts

# Slide travel (how far the pump head presses down)
SLIDE_TRAVEL = 0.012


def _bottle_solid() -> cq.Workplane:
    """Hollow bottle: rounded base -> tall cylinder -> short taper -> neck -> lip."""
    pts = [
        (0.000, 0.0140),   # rounded base bottom (tucked-in heel)
        (0.005, 0.0260),
        (0.012, 0.0295),
        (0.018, BODY_R),   # straight body starts
        (BODY_TOP_Z, BODY_R),  # tall straight cylindrical body
        (0.192, 0.0280),   # shoulder starts tapering
        (SHOULDER_TOP_Z, NECK_R + 0.002),  # short taper to neck
        (0.210, NECK_R),   # neck base
        (NECK_TOP_Z, NECK_R),  # neck tube
        (NECK_TOP_Z, NECK_R + 0.0025),  # lip flare outward
        (LIP_TOP_Z, NECK_R + 0.0025),   # lip top
    ]
    wp = cq.Workplane("XZ").moveTo(0.0, pts[0][0])
    for r, z in [(r, z) for (z, r) in pts]:
        wp = wp.lineTo(r, z)
    wp = wp.lineTo(0.0, pts[-1][0]).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))

    # Hollow cavity - open through the mouth at the top
    inner_pts = [
        (0.012, 0.005),
        (0.0245, 0.014),
        (BODY_R - WALL, 0.018),
        (BODY_R - WALL, BODY_TOP_Z),
        (0.0265, 0.192),
        (NECK_BORE_R + 0.001, SHOULDER_TOP_Z + 0.002),
        (NECK_BORE_R, 0.212),
        (NECK_BORE_R, LIP_TOP_Z + 0.005),  # open through the mouth rim
    ]
    iwp = cq.Workplane("XZ").moveTo(0.0, inner_pts[0][1])
    for r, z in inner_pts:
        iwp = iwp.lineTo(r, z)
    iwp = iwp.lineTo(0.0, inner_pts[-1][1]).close()
    cavity = iwp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    return outer.cut(cavity)


def _bottle_mesh():
    return mesh_from_cadquery(_bottle_solid(), "bottle_shell")


def _lip_ring():
    """Transparent lip rim as a separate visual at the mouth opening."""
    lip = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, NECK_TOP_Z))
        .circle(NECK_R + 0.0025)
        .circle(NECK_BORE_R)
        .extrude(LIP_TOP_Z - NECK_TOP_Z)
    )
    return mesh_from_cadquery(lip, "lip_rim")


def _pump_collar_solid() -> cq.Workplane:
    """Pump collar: threaded ring that sits around the neck."""
    collar = (
        cq.Workplane("XY")
        .circle(COLLAR_R)
        .extrude(COLLAR_H)
    )
    # Bore to fit over the neck
    bore = (
        cq.Workplane("XY")
        .circle(NECK_R + 0.001)
        .extrude(COLLAR_H)
    )
    collar = collar.cut(bore)
    # Add grip ribs around the outside
    n_ribs = 20
    for i in range(n_ribs):
        a = 2.0 * math.pi * i / n_ribs
        rib = (
            cq.Workplane("XY")
            .center(COLLAR_R * math.cos(a), COLLAR_R * math.sin(a))
            .circle(0.0008)
            .extrude(COLLAR_H)
        )
        collar = collar.cut(rib)
    return collar


def _pump_collar_mesh():
    return mesh_from_cadquery(_pump_collar_solid(), "pump_collar_shell")


def _pump_head_solid() -> cq.Workplane:
    """Pump head actuator with nozzle spout."""
    # Main cylindrical head body
    head = (
        cq.Workplane("XY")
        .circle(HEAD_R)
        .extrude(HEAD_H)
    )
    # Slightly domed top - flat disc with small raised center
    dome = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, HEAD_H))
        .circle(HEAD_R * 0.6)
        .extrude(0.002)
    )
    head = head.union(dome)

    # Nozzle spout extending in +X direction
    nozzle = (
        cq.Workplane("YZ")
        .transformed(offset=(HEAD_R - 0.002, 0, HEAD_H * 0.5))
        .circle(NOZZLE_R)
        .extrude(NOZZLE_LEN)
    )
    head = head.union(nozzle)

    # Stem going down (into the neck bore)
    stem = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -0.025))
        .circle(STEM_R)
        .extrude(0.025)
    )
    head = head.union(stem)

    # Bore through nozzle tip (small exit hole)
    nozzle_bore = (
        cq.Workplane("YZ")
        .transformed(offset=(HEAD_R + NOZZLE_LEN - 0.004, 0, HEAD_H * 0.5))
        .circle(NOZZLE_R * 0.5)
        .extrude(0.004)
    )
    head = head.cut(nozzle_bore)

    return head


def _pump_head_mesh():
    return mesh_from_cadquery(_pump_head_solid(), "pump_head_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pump_bottle")

    # Materials
    clear_body = model.material("clear_pet", rgba=(0.80, 0.88, 0.90, 0.22))
    clear_lip = model.material("clear_lip", rgba=(0.82, 0.90, 0.92, 0.30))
    white_collar = model.material("pump_collar_white", rgba=(0.92, 0.92, 0.90, 1.0))
    pump_head_mat = model.material("pump_head_white", rgba=(0.94, 0.94, 0.92, 1.0))
    nozzle_mat = model.material("nozzle_accent", rgba=(0.85, 0.85, 0.83, 1.0))

    # ---- bottle body (root): transparent hollow PET cylinder with neck ----
    body = model.part("bottle_body")
    body.visual(_bottle_mesh(), material=clear_body, name="bottle_shell")
    body.visual(_lip_ring(), material=clear_lip, name="lip_rim")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, LIP_TOP_Z),
        mass=0.028,
        origin=Origin(xyz=(0.0, 0.0, LIP_TOP_Z / 2.0)),
    )

    # ---- pump collar: fixed to the bottle neck (visual only, part of body) ----
    # We model it as a visual on the body part since it's fixed to the neck.
    body.visual(
        _pump_collar_mesh(),
        origin=Origin(xyz=(0.0, 0.0, COLLAR_BASE_Z)),
        material=white_collar,
        name="pump_collar",
    )

    # ---- massless carrier: decouples rotation (parent joint) from slide (child) ----
    carrier = model.part("pump_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.006, 0.006, 0.006)), mass=1e-4)

    # ---- pump head: actuator with nozzle ----
    pump_head = model.part("pump_head")
    pump_head.visual(
        _pump_head_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=pump_head_mat,
        name="pump_head_shell",
    )
    pump_head.inertial = Inertial.from_geometry(
        Cylinder(HEAD_R, HEAD_H + 0.025),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, HEAD_H / 2.0)),
    )

    # ---- Articulations ----
    # pump_rotate: REVOLUTE small-angle rotation about +Z
    # Carrier frame sits at the top of the collar where the head seats.
    model.articulation(
        "pump_rotate",
        ArticulationType.REVOLUTE,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, PUMP_HEAD_BASE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.5,
            lower=-0.26,  # ~-15 degrees
            upper=0.26,   # ~+15 degrees
        ),
    )

    # pump_slide: PRISMATIC press-down along -Z (positive q = head goes down)
    model.articulation(
        "pump_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=pump_head,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SLIDE_TRAVEL,
            effort=5.0,
            velocity=0.5,
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle_body")
    pump_head = object_model.get_part("pump_head")
    rotate = object_model.get_articulation("pump_rotate")
    slide = object_model.get_articulation("pump_slide")

    # --- bottle body is transparent ---
    clear_mat = next(m for m in object_model.materials if m.name == "clear_pet")
    a = clear_mat.rgba[3] if clear_mat.rgba is not None else 1.0
    ctx.check(
        "bottle shell is transparent",
        a < 1.0,
        details=f"clear_pet alpha={a}",
    )

    # --- lip rim is transparent ---
    lip_mat = next(m for m in object_model.materials if m.name == "clear_lip")
    a_lip = lip_mat.rgba[3] if lip_mat.rgba is not None else 1.0
    ctx.check(
        "lip rim is transparent",
        a_lip < 1.0,
        details=f"clear_lip alpha={a_lip}",
    )

    # --- tall cylindrical body: height >> width ---
    full = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "bottle is tall cylindrical (taller than 3x wide)",
        full[2] > 3.0 * full[0],
        details=f"body extents={full}",
    )

    # --- narrow neck: neck radius is much smaller than body radius ---
    ctx.check(
        "narrow neck (neck_r < body_r * 0.5)",
        NECK_R < BODY_R * 0.5,
        details=f"neck_r={NECK_R}, body_r={BODY_R}",
    )

    # --- mouth opening: lip_rim visual exists on the body ---
    lip_visual = body.get_visual("lip_rim")
    ctx.check(
        "visible lip rim at the mouth",
        lip_visual is not None,
        details="lip_rim visual not found on bottle_body",
    )

    # --- hollow mouth: the neck bore is smaller than the lip, proving a rim exists ---
    ctx.check(
        "mouth has wall thickness lip",
        (NECK_R + 0.0025) > NECK_BORE_R + WALL,
        details=f"lip outer={NECK_R + 0.0025}, bore={NECK_BORE_R}, wall={WALL}",
    )

    # --- pump head is mounted at the top ---
    head_pos = ctx.part_world_position(pump_head)
    ctx.check(
        "pump head mounted at top of bottle",
        head_pos is not None and head_pos[2] > 0.22,
        details=f"pump_head origin={head_pos}",
    )

    # --- pump head slides down: positive slide moves head downward ---
    z_rest = ctx.part_world_aabb(pump_head)[1][2]  # top of head at rest
    with ctx.pose({slide: SLIDE_TRAVEL}):
        z_pressed = ctx.part_world_aabb(pump_head)[1][2]  # top of head pressed
    ctx.check(
        "pump head slides down when pressed",
        z_pressed < z_rest - 0.005,
        details=f"head top z rest={z_rest:.4f}, pressed={z_pressed:.4f}",
    )

    # --- pump head rotates slightly: revolute joint has non-zero limits ---
    ctx.check(
        "pump head has rotation limits",
        rotate.motion_limits is not None
        and rotate.motion_limits.lower < 0.0
        and rotate.motion_limits.upper > 0.0,
        details=f"rotate limits={rotate.motion_limits}",
    )

    # --- rotation actually moves the nozzle off-axis ---
    head_aabb_rest = ctx.part_world_aabb(pump_head)
    center_rest = [(head_aabb_rest[0][i] + head_aabb_rest[1][i]) / 2.0 for i in range(3)]
    with ctx.pose({rotate: 0.26}):
        head_aabb_rot = ctx.part_world_aabb(pump_head)
        center_rot = [(head_aabb_rot[0][i] + head_aabb_rot[1][i]) / 2.0 for i in range(3)]
    xy_shift = math.hypot(center_rot[0] - center_rest[0], center_rot[1] - center_rest[1])
    ctx.check(
        "pump head rotation shifts the nozzle",
        xy_shift > 0.001,
        details=f"xy shift at max rotation={xy_shift:.4f}",
    )

    # --- joint is non-fixed: at least one joint has range > 0 ---
    slide_range = slide.motion_limits.upper - slide.motion_limits.lower
    rotate_range = rotate.motion_limits.upper - rotate.motion_limits.lower
    ctx.check(
        "at least one non-fixed joint exists",
        slide_range > 0.0 or rotate_range > 0.0,
        details=f"slide_range={slide_range}, rotate_range={rotate_range}",
    )

    # --- pump collar is present on the body ---
    collar_visual = body.get_visual("pump_collar")
    ctx.check(
        "pump collar visible on the neck",
        collar_visual is not None,
        details="pump_collar visual not found on bottle_body",
    )

    # --- pump stem passes through collar and neck bore (intentional) ---
    ctx.allow_overlap(
        pump_head,
        body,
        elem_a="pump_head_shell",
        elem_b="pump_collar",
        reason="The pump stem intentionally passes through the collar bore.",
    )
    ctx.allow_overlap(
        pump_head,
        body,
        elem_a="pump_head_shell",
        elem_b="bottle_shell",
        reason="The pump stem intentionally extends down into the neck bore.",
    )
    ctx.allow_overlap(
        pump_head,
        body,
        elem_a="pump_head_shell",
        elem_b="lip_rim",
        reason="The pump stem passes through the lip rim opening.",
    )

    # Prove the stem is inserted into the collar (Z overlap)
    ctx.expect_overlap(
        pump_head,
        body,
        axes="z",
        elem_a="pump_head_shell",
        elem_b="pump_collar",
        min_overlap=0.005,
        name="pump stem inserted into collar bore",
    )

    return ctx.report()


object_model = build_object_model()
