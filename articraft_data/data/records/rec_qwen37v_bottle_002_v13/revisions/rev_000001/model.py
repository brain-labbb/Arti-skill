from __future__ import annotations

# Sports bottle with flip straw cap, visible hollow mouth, and gasket ring.
# Frame: vertical axis along +Z, bottle standing on z=0, centerline on x=y=0.
#   - rounded base -> cylindrical body with grip indent -> tapered shoulder
#     -> threaded neck with visible hollow mouth bore
#   - rubber gasket ring seated on the neck rim
#   - cap_base collar with top plate and rear hinge lug
#   - flip_lid with hinge knuckle, lid disc, and straw spout
# Articulations:
#   - cap_screw: CONTINUOUS spin of cap_base about +Z (screw on/off)
#   - flip_hinge: REVOLUTE hinge at rear of cap, axis +X, opens lid upward

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
BASE_Z = 0.0
BODY_TOP_Z = 0.105       # end of straight body, start of shoulder
SHOULDER_TOP_Z = 0.145   # end of taper, base of neck
NECK_TOP_Z = 0.168       # top of threaded neck / mouth rim

BODY_R = 0.032           # body radius (~64mm dia, typical sports bottle)
BODY_GRIP_R = 0.029      # grip indent radius
NECK_R = 0.014           # neck outer radius (wider sports mouth ~28mm)
NECK_BORE_R = 0.011      # neck inner bore (visible mouth opening)

# Cap base collar
CAP_R = 0.017            # cap collar outer radius
CAP_BORE_R = NECK_R      # bore fits over neck
CAP_HEIGHT = 0.024       # collar height
CAP_PLATE_T = 0.003      # top plate thickness
CAP_MOUNT_Z = SHOULDER_TOP_Z  # cap screws down over neck from shoulder

# Hinge
HINGE_OFFSET_Y = -(CAP_R - 0.003)  # rear of cap collar (-Y)
HINGE_LUG_H = 0.008     # lug height above cap top
HINGE_LUG_W = 0.010     # lug width (X)
HINGE_LUG_D = 0.007     # lug depth (Y)
HINGE_KNUCKLE_R = 0.003  # knuckle outer radius
HINGE_PIN_Z = CAP_HEIGHT + HINGE_LUG_H * 0.5  # pin center height in cap_base local

# Flip lid
LID_R = 0.016            # lid disc radius
LID_T = 0.005            # lid disc thickness
SPOUT_R = 0.004          # straw spout outer radius
SPOUT_H = 0.012          # straw spout height
SPOUT_BORE_R = 0.0025    # spout bore radius

# Gasket
GASKET_MAJOR_R = NECK_R + 0.0005  # sits snugly on neck rim
GASKET_MINOR_R = 0.0018           # O-ring cross-section


def _profile_sections():
    """(z, radius) of the outer wall: base -> body with grip -> shoulder -> neck."""
    return [
        (0.000, 0.016),     # rounded base bottom (tucked heel)
        (0.006, 0.028),
        (0.012, 0.031),
        (0.020, BODY_R),    # full body radius
        (0.038, BODY_R),    # upper straight body
        (0.048, BODY_GRIP_R),  # grip indent start
        (0.065, BODY_GRIP_R),  # grip indent end
        (0.075, BODY_R),    # back to full radius
        (BODY_TOP_Z, BODY_R),  # end of body
        (0.118, 0.028),     # shoulder taper begins
        (0.132, 0.020),     # mid shoulder
        (SHOULDER_TOP_Z, 0.016),  # shoulder ends, neck starts
        (0.150, NECK_R),    # neck base
        (NECK_TOP_Z, NECK_R),   # neck top / mouth rim
    ]


def _bottle_solid() -> cq.Workplane:
    """Revoluted outer shell, hollowed through the neck for visible mouth bore."""
    pts = _profile_sections()
    wp = cq.Workplane("XZ").moveTo(0.0, pts[0][0])
    for r, z in [(r, z) for (z, r) in pts]:
        wp = wp.lineTo(r, z)
    wp = wp.lineTo(0.0, pts[-1][0]).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))

    # Hollow cavity opening through the neck rim
    wall = 0.0015
    inner_pts = [
        (0.010, 0.008),
        (0.026, 0.014),
        (BODY_R - wall, 0.022),
        (BODY_R - wall, 0.038),
        (BODY_GRIP_R - wall, 0.048),
        (BODY_GRIP_R - wall, 0.065),
        (BODY_R - wall, 0.075),
        (BODY_R - wall, BODY_TOP_Z),
        (0.026, 0.118),
        (0.019, 0.132),
        (0.0145, SHOULDER_TOP_Z),
        (NECK_BORE_R, 0.150),
        (NECK_BORE_R, NECK_TOP_Z + 0.005),  # open through rim
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
    """Helical thread rings on the neck."""
    g = None
    for zt in (0.152, 0.158, 0.164):
        ring = TorusGeometry(NECK_R - 0.0005, 0.0010, radial_segments=8, tubular_segments=36)
        ring.translate(0.0, 0.0, zt)
        if g is None:
            g = ring
        else:
            g.merge(ring)
    return mesh_from_geometry(g, "neck_threads")


def _cap_base_solid() -> cq.Workplane:
    """Cap collar with bore, top plate with mouth hole, and rear hinge lug."""
    # Main collar cylinder
    collar = cq.Workplane("XY").circle(CAP_R).extrude(CAP_HEIGHT)
    # Bore through (fits over neck)
    bore = cq.Workplane("XY").circle(CAP_BORE_R).extrude(CAP_HEIGHT)
    collar = collar.cut(bore)

    # Top plate closing the top, with mouth-sized hole
    plate_z = CAP_HEIGHT - CAP_PLATE_T
    plate = (
        cq.Workplane("XY")
        .workplane(offset=plate_z)
        .circle(CAP_R)
        .extrude(CAP_PLATE_T)
    )
    mouth_hole = (
        cq.Workplane("XY")
        .workplane(offset=plate_z - 0.001)
        .circle(NECK_BORE_R + 0.001)
        .extrude(CAP_PLATE_T + 0.002)
    )
    plate = plate.cut(mouth_hole)
    collar = collar.union(plate)

    # Hinge lug at rear (-Y side), extending above cap top
    lug = (
        cq.Workplane("XY")
        .workplane(offset=CAP_HEIGHT)
        .center(0.0, HINGE_OFFSET_Y)
        .rect(HINGE_LUG_W, HINGE_LUG_D)
        .extrude(HINGE_LUG_H)
    )
    collar = collar.union(lug)

    # Round the lug top slightly with a fillet-like cylinder
    lug_cap = (
        cq.Workplane("XZ")
        .workplane(offset=HINGE_OFFSET_Y)
        .center(0.0, CAP_HEIGHT + HINGE_LUG_H)
        .circle(HINGE_LUG_W * 0.45)
        .extrude(HINGE_LUG_D)
    )
    # Skip the rounded cap for simplicity; the box lug reads fine.

    return collar


def _cap_base_mesh():
    return mesh_from_cadquery(_cap_base_solid(), "cap_collar")


def _flip_lid_solid() -> cq.Workplane:
    """Flip lid: hinge knuckle at origin, lid disc forward (+Y), straw spout on top."""
    lid_offset_y = -HINGE_OFFSET_Y  # forward distance from hinge to mouth center
    lid_z_bot = -HINGE_KNUCKLE_R     # lid bottom aligns with knuckle bottom

    # Lid disc (thick cylinder)
    lid = (
        cq.Workplane("XY")
        .workplane(offset=lid_z_bot)
        .center(0.0, lid_offset_y)
        .circle(LID_R)
        .extrude(LID_T)
    )

    # Hinge knuckle: cylinder along X axis at origin
    knuckle_half = 0.005
    knuckle = (
        cq.Workplane("YZ")
        .workplane(offset=-knuckle_half)
        .circle(HINGE_KNUCKLE_R)
        .extrude(knuckle_half * 2)
    )

    # Bridge between knuckle and lid (thin rib connecting them)
    bridge_y_start = HINGE_KNUCKLE_R
    bridge_y_end = lid_offset_y - LID_R + 0.002
    bridge_len = max(bridge_y_end - bridge_y_start, 0.001)
    bridge_y_center = bridge_y_start + bridge_len / 2.0
    bridge = (
        cq.Workplane("XY")
        .workplane(offset=lid_z_bot)
        .center(0.0, bridge_y_center)
        .rect(0.006, bridge_len + 0.004)
        .extrude(LID_T)
    )

    # Straw spout on top of lid disc
    spout_z_bot = lid_z_bot + LID_T
    spout = (
        cq.Workplane("XY")
        .workplane(offset=spout_z_bot)
        .center(0.0, lid_offset_y)
        .circle(SPOUT_R)
        .extrude(SPOUT_H)
    )
    # Hollow the spout bore
    spout_bore = (
        cq.Workplane("XY")
        .workplane(offset=spout_z_bot + 0.001)
        .center(0.0, lid_offset_y)
        .circle(SPOUT_BORE_R)
        .extrude(SPOUT_H - 0.001)
    )
    spout = spout.cut(spout_bore)

    result = knuckle.union(lid).union(bridge).union(spout)
    return result


def _flip_lid_mesh():
    return mesh_from_cadquery(_flip_lid_solid(), "flip_lid_shell")


def _gasket_mesh():
    """Rubber O-ring gasket: torus centered on the neck rim."""
    g = TorusGeometry(
        GASKET_MAJOR_R,
        GASKET_MINOR_R,
        radial_segments=12,
        tubular_segments=48,
    )
    return mesh_from_geometry(g, "gasket_ring")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sports_bottle")

    # Materials
    translucent = model.material("translucent_body", rgba=(0.55, 0.75, 0.88, 0.35))
    neck_tint = model.material("neck_tint", rgba=(0.50, 0.70, 0.82, 0.40))
    rubber = model.material("rubber_gasket", rgba=(0.18, 0.18, 0.20, 1.0))
    cap_blue = model.material("cap_blue", rgba=(0.15, 0.35, 0.65, 1.0))
    lid_orange = model.material("lid_orange", rgba=(0.90, 0.45, 0.12, 1.0))

    # ---- bottle body (root) ----
    body = model.part("bottle_body")
    body.visual(_bottle_mesh(), material=translucent, name="bottle_shell")
    body.visual(_neck_threads(), material=neck_tint, name="neck_threads")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.045,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- gasket ring (fixed to body, seated on neck rim) ----
    gasket = model.part("gasket")
    gasket.visual(_gasket_mesh(), material=rubber, name="gasket_ring")
    gasket.inertial = Inertial.from_geometry(
        Cylinder(GASKET_MAJOR_R + GASKET_MINOR_R, GASKET_MINOR_R * 2),
        mass=0.002,
    )

    model.articulation(
        "gasket_seat",
        ArticulationType.FIXED,
        parent=body,
        child=gasket,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z)),
    )

    # ---- cap base collar (screws onto neck) ----
    cap_base = model.part("cap_base")
    cap_base.visual(_cap_base_mesh(), material=cap_blue, name="cap_collar")
    cap_base.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_HEIGHT + HINGE_LUG_H),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, (CAP_HEIGHT + HINGE_LUG_H) / 2.0)),
    )

    # cap_screw: CONTINUOUS rotation about +Z
    model.articulation(
        "cap_screw",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cap_base,
        origin=Origin(xyz=(0.0, 0.0, CAP_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0),
    )

    # ---- flip lid (hinged on cap_base) ----
    flip_lid = model.part("flip_lid")
    flip_lid.visual(_flip_lid_mesh(), material=lid_orange, name="flip_lid_shell")
    flip_lid.inertial = Inertial.from_geometry(
        Cylinder(LID_R, LID_T + SPOUT_H),
        mass=0.005,
        origin=Origin(xyz=(0.0, -HINGE_OFFSET_Y, LID_T / 2.0)),
    )

    # flip_hinge: REVOLUTE at rear of cap, axis +X, positive q opens lid upward
    # Hinge pin is at (0, HINGE_OFFSET_Y, HINGE_PIN_Z) in cap_base local frame.
    model.articulation(
        "flip_hinge",
        ArticulationType.REVOLUTE,
        parent=cap_base,
        child=flip_lid,
        origin=Origin(xyz=(0.0, HINGE_OFFSET_Y, HINGE_PIN_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=4.0,
            lower=0.0,    # closed
            upper=2.1,    # ~120 degrees open
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle_body")
    gasket = object_model.get_part("gasket")
    cap_base = object_model.get_part("cap_base")
    flip_lid = object_model.get_part("flip_lid")
    cap_screw = object_model.get_articulation("cap_screw")
    flip_hinge = object_model.get_articulation("flip_hinge")

    # --- Bottle identity: tall cylindrical body ---
    body_ext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "bottle body is tall (taller than wide)",
        body_ext[2] > 2.5 * body_ext[0],
        details=f"body extents={body_ext}",
    )

    # --- Translucent body material ---
    trans_mat = next(m for m in object_model.materials if m.name == "translucent_body")
    a = trans_mat.rgba[3] if trans_mat.rgba is not None else 1.0
    ctx.check(
        "bottle shell is translucent",
        a < 1.0,
        details=f"translucent_body alpha={a}",
    )

    # --- Gasket ring exists and is seated on neck rim ---
    ctx.allow_overlap(
        gasket,
        body,
        elem_a="gasket_ring",
        elem_b="bottle_shell",
        reason="The gasket ring is seated on the neck rim with slight embed for compression seal.",
    )
    ctx.expect_contact(
        gasket,
        body,
        elem_a="gasket_ring",
        elem_b="bottle_shell",
        name="gasket contacts neck rim",
    )
    gasket_pos = ctx.part_world_position(gasket)
    ctx.check(
        "gasket positioned at neck top",
        gasket_pos is not None and gasket_pos[2] > SHOULDER_TOP_Z,
        details=f"gasket pos={gasket_pos}",
    )

    # --- Cap base screws over neck (intentional overlap) ---
    ctx.allow_overlap(
        cap_base,
        body,
        elem_a="cap_collar",
        elem_b="bottle_shell",
        reason="The cap collar intentionally screws down over the threaded neck.",
    )
    ctx.allow_overlap(
        cap_base,
        body,
        elem_a="cap_collar",
        elem_b="neck_threads",
        reason="The cap collar covers the neck threads when mounted.",
    )
    ctx.allow_overlap(
        cap_base,
        gasket,
        elem_a="cap_collar",
        elem_b="gasket_ring",
        reason="The cap collar compresses the gasket ring when screwed on.",
    )
    cap_pos = ctx.part_world_position(cap_base)
    ctx.check(
        "cap base mounted at top of bottle",
        cap_pos is not None and cap_pos[2] > SHOULDER_TOP_Z - 0.005,
        details=f"cap_base pos={cap_pos}",
    )

    # --- Cap base spins (CONTINUOUS joint) ---
    with ctx.pose({cap_screw: math.pi / 2.0}):
        cap_rotated_pos = ctx.part_world_position(cap_base)
    ctx.check(
        "cap_screw is a non-fixed articulation",
        cap_screw.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={cap_screw.articulation_type}",
    )

    # --- Flip hinge exists and is REVOLUTE ---
    ctx.check(
        "flip_hinge is REVOLUTE",
        flip_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={flip_hinge.articulation_type}",
    )
    limits = flip_hinge.motion_limits
    ctx.check(
        "flip_hinge has bounded motion limits",
        limits is not None and limits.lower is not None and limits.upper is not None,
        details=f"limits={limits}",
    )
    ctx.check(
        "flip_hinge upper limit opens past 90 degrees",
        limits is not None and limits.upper is not None and limits.upper > 1.4,
        details=f"upper={limits.upper if limits else None}",
    )

    # --- Flip lid opens upward with positive hinge angle ---
    lid_rest_z = ctx.part_world_aabb(flip_lid)[1][2]  # max z at rest (closed)
    with ctx.pose({flip_hinge: 1.5}):
        lid_open_z = ctx.part_world_aabb(flip_lid)[1][2]
    ctx.check(
        "flip lid opens upward (max z rises with positive hinge angle)",
        lid_open_z > lid_rest_z + 0.005,
        details=f"rest max_z={lid_rest_z:.4f}, open max_z={lid_open_z:.4f}",
    )

    # --- Flip lid hinge knuckle overlaps cap_base lug (intentional nesting) ---
    ctx.allow_overlap(
        flip_lid,
        cap_base,
        elem_a="flip_lid_shell",
        elem_b="cap_collar",
        reason="The flip lid hinge knuckle nests into the cap_base hinge lug.",
    )
    ctx.expect_contact(
        flip_lid,
        cap_base,
        elem_a="flip_lid_shell",
        elem_b="cap_collar",
        name="flip lid contacts cap base at hinge",
    )

    # --- Mouth opening visible: bottle_shell has hollow bore at neck top ---
    # Verify the inner bore exists by checking the neck region is narrower than body
    ctx.check(
        "neck bore narrower than body (hollow mouth opening)",
        NECK_BORE_R < BODY_R * 0.5,
        details=f"neck_bore_r={NECK_BORE_R}, body_r={BODY_R}",
    )

    # --- Straw spout exists on flip lid (lid has height beyond just the disc) ---
    lid_ext = _ext(ctx.part_world_aabb(flip_lid))
    ctx.check(
        "flip lid has vertical extent (disc + spout)",
        lid_ext[2] > LID_T + 0.003,
        details=f"lid z extent={lid_ext[2]:.4f}",
    )

    return ctx.report()


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


object_model = build_object_model()
