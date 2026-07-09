from __future__ import annotations

# Swing-top bottle: clear PET body with tapered shoulder, wire bail mechanism,
# hinged ceramic stopper.  The cap rotates continuously (carrier) and swings
# open on a revolute hinge.  Visible hollow mouth opening under the cap,
# transparent wall-thickness lip at the rim.

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

# ---- key heights (m) along +Z ----
BODY_TOP_Z = 0.110
SHOULDER_TOP_Z = 0.156
NECK_TOP_Z = 0.176

BODY_R = 0.0275
NECK_R = 0.0125
NECK_BORE_R = 0.0098

# Mouth lip (transparent wall-thickness ring at the rim)
LIP_MAJOR_R = NECK_R
LIP_MINOR_R = 0.002

# Stopper cap
CAP_R = 0.013
CAP_DISC_H = 0.006
CAP_PLUG_R = NECK_BORE_R - 0.001
CAP_PLUG_H = 0.010
CAP_KNOB_R = 0.004
CAP_KNOB_H = 0.004

# Hinge (at back edge of neck)
HINGE_Y = NECK_R + 0.003
HINGE_UPPER = 2.0  # ~115 degrees

# Wire bail
WIRE_R = 0.0008
BAIL_RING_R = NECK_R + 0.003
BAIL_RING_Z = SHOULDER_TOP_Z + 0.006
BAIL_ARM_TOP_Z = NECK_TOP_Z + 0.016


def _profile_sections():
    return [
        (0.000, 0.0150),
        (0.006, 0.0250),
        (0.014, 0.0273),
        (BODY_TOP_Z, BODY_R),
        (0.124, 0.0268),
        (0.138, 0.0228),
        (SHOULDER_TOP_Z, 0.0148),
        (0.160, NECK_R),
        (NECK_TOP_Z, NECK_R),
    ]


def _bottle_solid() -> cq.Workplane:
    """Hollow bottle shell: revolved profile with interior cavity open at the rim."""
    pts = _profile_sections()
    wp = cq.Workplane("XZ").moveTo(0.0, pts[0][0])
    for r, z in [(r, z) for (z, r) in pts]:
        wp = wp.lineTo(r, z)
    wp = wp.lineTo(0.0, pts[-1][0]).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))

    wall = 0.0014
    inner_pts = [
        (0.010, 0.006),
        (0.0235, 0.012),
        (BODY_R - wall, 0.014),
        (BODY_R - wall, BODY_TOP_Z),
        (0.0254, 0.124),
        (0.0214, 0.138),
        (0.0134, SHOULDER_TOP_Z),
        (NECK_BORE_R, 0.160),
        (NECK_BORE_R, NECK_TOP_Z + 0.005),  # open through the rim
    ]
    iwp = cq.Workplane("XZ").moveTo(0.0, inner_pts[0][1])
    for r, z in inner_pts:
        iwp = iwp.lineTo(r, z)
    iwp = iwp.lineTo(0.0, inner_pts[-1][1]).close()
    cavity = iwp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    return outer.cut(cavity)


def _bottle_mesh():
    return mesh_from_cadquery(_bottle_solid(), "bottle_shell")


def _neck_threads():
    g = None
    for zt in (0.163, 0.169):
        ring = TorusGeometry(NECK_R - 0.0006, 0.0012, radial_segments=10, tubular_segments=40)
        ring.translate(0.0, 0.0, zt)
        if g is None:
            g = ring
        else:
            g.merge(ring)
    return mesh_from_geometry(g, "neck_threads")


def _mouth_lip():
    """Transparent wall-thickness lip ring at the neck rim."""
    lip = TorusGeometry(LIP_MAJOR_R, LIP_MINOR_R, radial_segments=12, tubular_segments=48)
    lip.translate(0.0, 0.0, NECK_TOP_Z)
    return mesh_from_geometry(lip, "mouth_lip")


def _wire_bail_solid() -> cq.Workplane:
    """Wire bail: neck ring + two vertical arms + cross wire at top."""
    ring_r = BAIL_RING_R
    wire_r = WIRE_R

    # Torus ring around neck
    ring = (
        cq.Workplane("XZ")
        .moveTo(ring_r, 0)
        .circle(wire_r)
        .revolve(360, (0, 0), (0, 1))
        .translate((0, 0, BAIL_RING_Z))
    )

    # Two vertical arms (extra overlap at junctions for robust boolean)
    arm_h = BAIL_ARM_TOP_Z - BAIL_RING_Z + 4 * wire_r
    arm_l = (
        cq.Workplane("XY")
        .circle(wire_r)
        .extrude(arm_h)
        .translate((-ring_r, 0, BAIL_RING_Z - 2 * wire_r))
    )
    arm_r = (
        cq.Workplane("XY")
        .circle(wire_r)
        .extrude(arm_h)
        .translate((ring_r, 0, BAIL_RING_Z - 2 * wire_r))
    )

    # Cross wire at top connecting the two arms
    cross_len = 2 * ring_r + 4 * wire_r
    cross = (
        cq.Workplane("YZ")
        .circle(wire_r)
        .extrude(cross_len)
        .translate((-ring_r - 2 * wire_r, 0, BAIL_ARM_TOP_Z))
    )

    return ring.union(arm_l).union(arm_r).union(cross)


def _wire_bail_mesh():
    return mesh_from_cadquery(_wire_bail_solid(), "wire_bail")


def _stopper_solid() -> cq.Workplane:
    """Ceramic swing-top stopper: disc + plug + grip knob + front latch tab.

    Built in the cap part frame.  The hinge puts the cap origin at the back
    edge of the neck, so the stopper disc is offset by -HINGE_Y to sit
    centered over the mouth when closed.
    """
    dy = -HINGE_Y  # center the disc over the mouth

    # Main disc (sits on mouth rim, bottom at z=0)
    disc = (
        cq.Workplane("XY")
        .circle(CAP_R)
        .extrude(CAP_DISC_H)
        .translate((0, dy, 0))
    )

    # Plug that inserts into the mouth bore
    plug = (
        cq.Workplane("XY")
        .circle(CAP_PLUG_R)
        .extrude(CAP_PLUG_H)
        .translate((0, dy, -CAP_PLUG_H))
    )

    # Grip knob on top
    knob = (
        cq.Workplane("XY")
        .circle(CAP_KNOB_R)
        .extrude(CAP_KNOB_H)
        .translate((0, dy, CAP_DISC_H))
    )

    # Front latch tab: protrudes from the disc edge opposite the hinge,
    # providing a clip point for the wire bail.  This also breaks rotational
    # symmetry so continuous rotation is visually legible.
    tab_w = 0.006
    tab_h = CAP_DISC_H
    tab_y = dy - CAP_R - tab_w / 2  # front edge of disc, extending outward
    tab = (
        cq.Workplane("XY")
        .rect(tab_w, tab_w)
        .extrude(tab_h)
        .translate((0, tab_y, 0))
    )

    return disc.union(plug).union(knob).union(tab)


def _stopper_mesh():
    return mesh_from_cadquery(_stopper_solid(), "stopper_shell")


def _cap_gasket():
    """Red rubber gasket ring at the stopper disc bottom."""
    gasket = TorusGeometry(CAP_R - 0.003, 0.001, radial_segments=8, tubular_segments=24)
    gasket.translate(0, -HINGE_Y, 0)
    return mesh_from_geometry(gasket, "cap_gasket")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_top_bottle")

    # Materials
    clear = model.material("clear_pet", rgba=(0.78, 0.85, 0.88, 0.25))
    clear_lip = model.material("clear_lip", rgba=(0.82, 0.90, 0.92, 0.40))
    clear_neck = model.material("clear_neck", rgba=(0.72, 0.80, 0.84, 0.30))
    metal = model.material("bail_metal", rgba=(0.50, 0.50, 0.52, 1.0))
    ceramic = model.material("stopper_white", rgba=(0.93, 0.91, 0.86, 1.0))
    rubber = model.material("gasket_red", rgba=(0.65, 0.12, 0.10, 1.0))

    # ---- bottle body (root) ----
    body = model.part("bottle_body")
    body.visual(_bottle_mesh(), material=clear, name="bottle_shell")
    body.visual(_neck_threads(), material=clear_neck, name="neck_threads")
    body.visual(_mouth_lip(), material=clear_lip, name="mouth_lip")
    body.visual(_wire_bail_mesh(), material=metal, name="wire_bail")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, 0.176),
        mass=0.020,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
    )

    # ---- massless carrier for continuous rotation ----
    carrier = model.part("cap_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.008, 0.008, 0.008)), mass=1e-4)

    # ---- swing-top stopper cap ----
    cap = model.part("swing_cap")
    cap.visual(_stopper_mesh(), material=ceramic, name="stopper_shell")
    cap.visual(_cap_gasket(), material=rubber, name="cap_gasket")
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_DISC_H + CAP_PLUG_H),
        mass=0.005,
        origin=Origin(xyz=(0.0, -HINGE_Y, (CAP_DISC_H - CAP_PLUG_H) / 2.0)),
    )

    # cap_rotate: CONTINUOUS spin about +Z
    model.articulation(
        "cap_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    # cap_hinge: REVOLUTE swing open
    # Hinge at back (+Y) edge of neck.  axis=(-1,0,0) so positive q swings
    # the off-center stopper upward (away from the mouth).
    model.articulation(
        "cap_hinge",
        ArticulationType.REVOLUTE,
        parent=carrier,
        child=cap,
        origin=Origin(xyz=(0.0, HINGE_Y, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=HINGE_UPPER, effort=2.0, velocity=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle_body")
    cap = object_model.get_part("swing_cap")
    rotate = object_model.get_articulation("cap_rotate")
    hinge = object_model.get_articulation("cap_hinge")

    # --- transparent bottle shell ---
    clear_mat = next(m for m in object_model.materials if m.name == "clear_pet")
    a = clear_mat.rgba[3] if clear_mat.rgba is not None else 1.0
    ctx.check("bottle shell is transparent", a < 1.0, details=f"alpha={a}")

    # --- mouth lip visual exists ---
    lip_vis = body.get_visual("mouth_lip")
    ctx.check("mouth lip visual exists", lip_vis is not None, details="not found")

    # --- wire bail visual exists ---
    bail_vis = body.get_visual("wire_bail")
    ctx.check("wire bail visual exists", bail_vis is not None, details="not found")

    # --- intentional overlaps: stopper in mouth ---
    ctx.allow_overlap(
        cap, body,
        elem_a="stopper_shell", elem_b="bottle_shell",
        reason="Stopper plug intentionally inserts into the hollow mouth bore.",
    )
    ctx.allow_overlap(
        cap, body,
        elem_a="stopper_shell", elem_b="mouth_lip",
        reason="Stopper disc seats on the mouth lip when closed.",
    )
    ctx.allow_overlap(
        cap, body,
        elem_a="cap_gasket", elem_b="mouth_lip",
        reason="Rubber gasket compresses against the mouth lip.",
    )
    ctx.allow_overlap(
        cap, body,
        elem_a="cap_gasket", elem_b="bottle_shell",
        reason="Gasket seats on the neck rim when closed.",
    )

    # --- cap is at the top of the bottle ---
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap mounted at top of bottle",
        cap_pos is not None and cap_pos[2] > 0.15,
        details=f"cap origin={cap_pos}",
    )

    # --- hinge opens cap upward ---
    z_rest_top = ctx.part_world_aabb(cap)[1][2]
    with ctx.pose({hinge: HINGE_UPPER}):
        z_open_top = ctx.part_world_aabb(cap)[1][2]
    ctx.check(
        "hinge swings cap upward when opened",
        z_open_top > z_rest_top + 0.005,
        details=f"rest_top_z={z_rest_top:.4f}, open_top_z={z_open_top:.4f}",
    )

    # --- hinge has bounded revolute limits ---
    limits = hinge.motion_limits
    ctx.check(
        "hinge has lower and upper limits",
        limits is not None and limits.lower is not None and limits.upper is not None,
        details=f"limits={limits}",
    )
    if limits is not None and limits.upper is not None:
        ctx.check(
            "hinge upper limit < pi",
            limits.upper < math.pi,
            details=f"upper={limits.upper:.3f}",
        )

    # --- continuous rotate joint exists and type is correct ---
    ctx.check(
        "cap_rotate is CONTINUOUS type",
        rotate.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={rotate.articulation_type}",
    )

    # --- cap rotates (off-center hinge point moves when spinning) ---
    pos_rest = ctx.part_world_position(cap)
    with ctx.pose({rotate: math.pi / 2.0}):
        pos_rot = ctx.part_world_position(cap)
    moved = (
        math.hypot(pos_rot[0] - pos_rest[0], pos_rot[1] - pos_rest[1])
        if pos_rest is not None and pos_rot is not None
        else 0.0
    )
    ctx.check(
        "cap continuous rotation moves off-center hinge point",
        moved > 0.005,
        details=f"rest={pos_rest}, rot={pos_rot}, moved={moved:.4f}",
    )

    # --- bottle proportions: tall and tapered ---
    body_aabb = ctx.part_world_aabb(body)
    bw = body_aabb[1][0] - body_aabb[0][0]
    bh = body_aabb[1][2] - body_aabb[0][2]
    ctx.check("bottle is tall (h > 2.5*w)", bh > 2.5 * bw, details=f"h={bh:.3f}, w={bw:.3f}")
    ctx.check(
        "tapered shoulder narrows toward top",
        NECK_R < BODY_R * 0.6,
        details=f"neck_r={NECK_R}, body_r={BODY_R}",
    )

    return ctx.report()


object_model = build_object_model()
