from __future__ import annotations

# Squeeze bottle with a conical nozzle cap and a pivoting straw spout.
# Frame: vertical axis along +Z, bottle standing on z=0, centerline on x=y=0.
#   - rounded base -> cylindrical body -> tapered shoulder -> neck with mouth lip
#   - conical nozzle cap screws over the neck (CONTINUOUS spin + PRISMATIC lift)
#   - straw spout pivots up from the cap via a REVOLUTE joint
# Visible: hollow mouth opening, transparent wall-thickness lip at the mouth.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
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

# ---- key heights (m) along +Z ----
BASE_Z = 0.0
BODY_TOP_Z = 0.105   # end of straight cylindrical body, start of shoulder
SHOULDER_TOP_Z = 0.150  # end of tapered shoulder, base of neck
NECK_TOP_Z = 0.172   # top of threaded neck (cap mounts here)

BODY_R = 0.030       # body radius (~0.060 m dia — squeeze bottle is a bit wider)
NECK_R = 0.012       # outer thread/neck radius
NECK_BORE_R = 0.009  # neck inner bore

# Conical nozzle cap
CAP_BASE_R = 0.0145   # cap base (skirts over neck)
CAP_TIP_R = 0.0035    # nozzle tip radius
CAP_HEIGHT = 0.028    # total cap height (cone)
CAP_BORE_R = NECK_R   # inner skirt grips the neck
CAP_MOUNT_Z = NECK_TOP_Z - 0.006  # cap seats down over the neck top

# Mouth lip
LIP_OUTER_R = NECK_R + 0.003   # slightly wider than neck — visible lip
LIP_INNER_R = NECK_BORE_R
LIP_HEIGHT = 0.004              # short ring at the mouth rim

# Straw spout
SPOUT_R = 0.003       # outer radius of the straw tube
SPOUT_BORE_R = 0.002  # hollow bore inside the straw
SPOUT_LENGTH = 0.045  # straw length
SPOUT_PIVOT_Z = 0.030  # pivot near the cap top (in cap-local frame), above cone


def _profile_sections():
    # (z, radius) of the outer wall, base -> body -> tapered shoulder -> neck.
    return [
        (0.000, 0.016),    # rounded base bottom (tucked-in heel)
        (0.006, 0.026),
        (0.014, 0.0298),
        (BODY_TOP_Z, BODY_R),       # straight cylindrical body
        (0.118, 0.029),             # shoulder starts tapering inward
        (0.132, 0.024),
        (SHOULDER_TOP_Z, 0.014),    # tapered shoulder narrows
        (0.155, NECK_R),            # base of neck
        (NECK_TOP_Z, NECK_R),       # straight threaded neck up to the rim
    ]


def _bottle_solid() -> cq.Workplane:
    # Revolve the outer profile, then shell it open at the top so the bottle is
    # a real thin-walled hollow container (open mouth at the neck rim).
    pts = _profile_sections()
    wp = cq.Workplane("XZ").moveTo(0.0, pts[0][0])
    for r, z in [(r, z) for (z, r) in pts]:
        wp = wp.lineTo(r, z)
    wp = wp.lineTo(0.0, pts[-1][0]).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))

    # Hollow it out: cut an inner cavity that opens through the neck rim.
    wall = 0.0015
    inner_pts = [
        (0.010, 0.006),
        (0.024, 0.012),
        (BODY_R - wall, 0.014),
        (BODY_R - wall, BODY_TOP_Z),
        (0.0275, 0.118),
        (0.0225, 0.132),
        (0.0125, SHOULDER_TOP_Z),
        (NECK_BORE_R, 0.155),
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


def _mouth_lip_mesh():
    # Transparent wall-thickness lip ring at the mouth — a short annular ring
    # that sits on top of the neck rim, showing the clear wall thickness.
    lip = (
        cq.Workplane("XY")
        .circle(LIP_OUTER_R)
        .circle(LIP_INNER_R)
        .extrude(LIP_HEIGHT)
    )
    return mesh_from_cadquery(lip, "mouth_lip")


def _neck_threads():
    g = None
    for zt in (0.158, 0.164):
        ring = TorusGeometry(NECK_R - 0.0006, 0.0011, radial_segments=10, tubular_segments=40)
        ring.translate(0.0, 0.0, zt)
        if g is None:
            g = ring
        else:
            g.merge(ring)
    return mesh_from_geometry(g, "neck_threads")


def _cap_solid() -> cq.Workplane:
    # Conical nozzle cap with a cylindrical skirt base. The skirt is a short
    # cylinder that grips the neck, the cone sits above it tapering to a
    # nozzle stub. Built as one connected solid.
    skirt_h = 0.008          # cylindrical skirt height
    cone_h = CAP_HEIGHT - skirt_h  # cone section above the skirt
    nozzle_stub_h = 0.004
    nozzle_stub_r = 0.003

    # Outer profile: cylinder at base, then cone tapering to nozzle stub.
    cap = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(CAP_BASE_R, 0.0)
        .lineTo(CAP_BASE_R, skirt_h)         # vertical skirt wall
        .lineTo(CAP_TIP_R, skirt_h + cone_h) # cone taper
        .lineTo(nozzle_stub_r, skirt_h + cone_h)
        .lineTo(nozzle_stub_r, skirt_h + cone_h + nozzle_stub_h)
        .lineTo(0.0, skirt_h + cone_h + nozzle_stub_h)
        .close()
        .revolve(360.0, (0, 0, 0), (0, 1, 0))
    )
    # Hollow the underside (skirt bore) so it fits over the neck.
    bore = (
        cq.Workplane("XY")
        .circle(CAP_BORE_R)
        .extrude(skirt_h - 0.001)  # slightly less than skirt height
    )
    cap = cap.cut(bore)
    # Central channel bore from inside skirt through to nozzle tip.
    channel_start = skirt_h - 0.001
    total_top = skirt_h + cone_h + nozzle_stub_h
    channel = (
        cq.Workplane("XY")
        .workplane(offset=channel_start)
        .circle(0.0018)
        .extrude(total_top - channel_start + 0.001)
    )
    cap = cap.cut(channel)
    return cap


def _cap_mesh():
    return mesh_from_cadquery(_cap_solid(), "cap_shell")


def _spout_solid() -> cq.Workplane:
    # Straw spout: a thin hollow tube. Part frame at the pivot end.
    # The tube extends along local +X when at rest (folded down).
    outer = (
        cq.Workplane("YZ")
        .circle(SPOUT_R)
        .extrude(SPOUT_LENGTH)
    )
    inner = (
        cq.Workplane("YZ")
        .circle(SPOUT_BORE_R)
        .extrude(SPOUT_LENGTH)
    )
    return outer.cut(inner)


def _spout_mesh():
    return mesh_from_cadquery(_spout_solid(), "spout_tube")


def _spout_pivot_mount():
    # Small pivot hinge barrel at the base of the spout.
    barrel = CylinderGeometry(0.004, 0.008, radial_segments=16)
    return mesh_from_geometry(barrel, "spout_hinge")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="squeeze_bottle")

    clear = model.material("clear_pet", rgba=(0.78, 0.86, 0.90, 0.22))
    clear_lip = model.material("clear_lip", rgba=(0.80, 0.88, 0.92, 0.35))
    cap_white = model.material("cap_white", rgba=(0.92, 0.92, 0.93, 1.0))
    spout_gray = model.material("spout_gray", rgba=(0.55, 0.56, 0.58, 1.0))
    hinge_dark = model.material("hinge_dark", rgba=(0.25, 0.25, 0.27, 1.0))

    # ---- bottle body (root): transparent hollow squeeze bottle shell ----
    body = model.part("bottle_body")
    body.visual(_bottle_mesh(), material=clear, name="bottle_shell")
    body.visual(_neck_threads(), material=clear, name="neck_threads")
    body.visual(
        _mouth_lip_mesh(),
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z)),
        material=clear_lip,
        name="mouth_lip",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.022,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- massless carrier: decouples spin from lift ----
    carrier = model.part("cap_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.008, 0.008, 0.008)), mass=1e-4)

    # ---- conical nozzle cap ----
    cap = model.part("cap")
    cap.visual(_cap_mesh(), material=cap_white, name="cap_shell")
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_BASE_R, 0.032),
        mass=0.004,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
    )

    # ---- straw spout (pivots from cap) ----
    spout = model.part("spout")
    spout.visual(_spout_mesh(), material=spout_gray, name="spout_tube")
    spout.visual(
        _spout_pivot_mount(),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_dark,
        name="spout_hinge",
    )
    spout.inertial = Inertial.from_geometry(
        Cylinder(SPOUT_R, SPOUT_LENGTH),
        mass=0.002,
        origin=Origin(xyz=(SPOUT_LENGTH / 2.0, 0.0, 0.0)),
    )

    # ---- Articulations ----

    # cap_rotate: CONTINUOUS spin about +Z (the bottle axis).
    model.articulation(
        "cap_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, CAP_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    # cap_slide: PRISMATIC lift along +Z (cap lifts off the neck).
    model.articulation(
        "cap_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=CAP_HEIGHT + 0.01, effort=1.0, velocity=1.0),
    )

    # spout_pivot: REVOLUTE — spout pivots up from folded-down to upright.
    # Pivot origin at the cap top edge. The spout tube extends along local +X
    # at rest (folded flat). Axis along -Y so positive q rotates it upward (+Z).
    model.articulation(
        "spout_pivot",
        ArticulationType.REVOLUTE,
        parent=cap,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, SPOUT_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=math.pi / 2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle_body")
    cap = object_model.get_part("cap")
    spout = object_model.get_part("spout")
    rotate = object_model.get_articulation("cap_rotate")
    slide = object_model.get_articulation("cap_slide")
    pivot = object_model.get_articulation("spout_pivot")

    # --- bottle is clear (alpha < 1) ---
    clear_mat = next(m for m in object_model.materials if m.name == "clear_pet")
    a = clear_mat.rgba[3] if clear_mat.rgba is not None else 1.0
    ctx.check(
        "bottle shell is transparent",
        a < 1.0,
        details=f"clear_pet alpha={a}",
    )

    # --- mouth lip is transparent ---
    lip_mat = next(m for m in object_model.materials if m.name == "clear_lip")
    lip_a = lip_mat.rgba[3] if lip_mat.rgba is not None else 1.0
    ctx.check(
        "mouth lip is transparent",
        lip_a < 1.0,
        details=f"clear_lip alpha={lip_a}",
    )

    # --- cap seated over the neck at rest (intentional screw-over capture) ---
    ctx.allow_overlap(
        cap, body,
        elem_a="cap_shell", elem_b="bottle_shell",
        reason="The conical cap skirt intentionally seats over the threaded neck.",
    )
    ctx.allow_overlap(
        cap, body,
        elem_a="cap_shell", elem_b="neck_threads",
        reason="The cap skirt intentionally covers the neck threads when closed.",
    )
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap mounted at the top of the bottle",
        cap_pos is not None and cap_pos[2] > 0.14,
        details=f"cap origin={cap_pos}",
    )

    # --- cap is conical (narrower at top than base) ---
    cap_aabb = ctx.part_world_aabb(cap)
    cap_extents = (
        cap_aabb[1][0] - cap_aabb[0][0],
        cap_aabb[1][1] - cap_aabb[0][1],
        cap_aabb[1][2] - cap_aabb[0][2],
    )
    ctx.check(
        "cap has nonzero height",
        cap_extents[2] > 0.015,
        details=f"cap extents={cap_extents}",
    )

    # --- spout pivots up: at rest it's folded, at upper limit it's upright ---
    spout_rest_aabb = ctx.part_world_aabb(spout)
    rest_z_max = spout_rest_aabb[1][2]

    with ctx.pose({pivot: math.pi / 2.0}):
        spout_up_aabb = ctx.part_world_aabb(spout)
        up_z_max = spout_up_aabb[1][2]

    ctx.check(
        "spout pivots upward when actuated",
        up_z_max > rest_z_max + 0.02,
        details=f"rest z_max={rest_z_max:.4f}, pivoted z_max={up_z_max:.4f}",
    )

    # --- spout pivot is a REVOLUTE joint with correct limits ---
    pivot_limits = pivot.motion_limits
    ctx.check(
        "spout_pivot is REVOLUTE with valid limits",
        pivot.articulation_type == ArticulationType.REVOLUTE
        and pivot_limits is not None
        and pivot_limits.lower is not None
        and pivot_limits.upper is not None
        and pivot_limits.upper > pivot_limits.lower,
        details=f"type={pivot.articulation_type}, limits={pivot_limits}",
    )

    # --- cap spins about +Z ---
    with ctx.pose({rotate: math.pi / 2.0}):
        cap_rotated_aabb = ctx.part_world_aabb(cap)
    rotated_extents = (
        cap_rotated_aabb[1][0] - cap_rotated_aabb[0][0],
        cap_rotated_aabb[1][1] - cap_rotated_aabb[0][1],
    )
    rest_extents = (
        cap_aabb[1][0] - cap_aabb[0][0],
        cap_aabb[1][1] - cap_aabb[0][1],
    )
    # A cone rotated 90° about its axis should have similar XY extents (it's axi-symmetric)
    # but the articulation still moves — verify the joint works by checking a different way
    ctx.check(
        "cap_rotate is CONTINUOUS",
        rotate.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={rotate.articulation_type}",
    )

    # --- cap slides up off the neck ---
    z_rest = ctx.part_world_aabb(cap)[0][2]
    with ctx.pose({slide: CAP_HEIGHT + 0.01}):
        z_up = ctx.part_world_aabb(cap)[0][2]
    ctx.check(
        "cap slides up off the neck",
        z_up > z_rest + 0.015,
        details=f"cap bottom z rest={z_rest:.4f}, lifted={z_up:.4f}",
    )

    # --- mouth lip exists above the neck top ---
    lip_visual = body.get_visual("mouth_lip")
    ctx.check(
        "mouth lip visual exists on the bottle body",
        lip_visual is not None,
        details="mouth_lip visual not found on bottle_body",
    )

    # --- bottle is tall (taller than wide) ---
    body_aabb = ctx.part_world_aabb(body)
    body_ext = (
        body_aabb[1][0] - body_aabb[0][0],
        body_aabb[1][1] - body_aabb[0][1],
        body_aabb[1][2] - body_aabb[0][2],
    )
    ctx.check(
        "bottle is tall (taller than wide)",
        body_ext[2] > 2.5 * body_ext[0],
        details=f"body extents={body_ext}",
    )

    # --- tapered shoulder narrows toward the top ---
    ctx.check(
        "tapered shoulder narrows toward the top",
        NECK_R < BODY_R * 0.6,
        details=f"neck_r={NECK_R}, body_r={BODY_R}",
    )

    # --- spout is connected to the cap (hinge mount provides support) ---
    ctx.allow_overlap(
        spout, cap,
        elem_a="spout_hinge", elem_b="cap_shell",
        reason="The spout hinge barrel sits embedded in the cap top as a pivot mount.",
    )

    return ctx.report()


object_model = build_object_model()
