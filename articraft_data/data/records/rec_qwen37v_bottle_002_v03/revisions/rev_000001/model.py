from __future__ import annotations

# Sports bottle with flip straw cap and swing-top stopper.
# Frame: vertical axis along +Z, bottle standing on z=0, centerline on x=y=0.
#   - rounded base -> cylindrical body -> tapered shoulder -> short neck -> mouth opening
# Parts:
#   - bottle_body: hollow clear PET sports bottle shell (root)
#   - cap_base: cap ring with hinge lugs on the +Y side (fixed to bottle neck)
#   - flip_lid: swing-top stopper that pivots on side hinge arms
# Articulation:
#   - body_to_cap: FIXED, cap base sits over the neck
#   - flip_hinge: REVOLUTE about +X axis at the side hinge pin.
#     positive q opens the lid upward/backward, limits 0 to ~2.4 rad (~140°).

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

# ---- key heights (m) along +Z (bottle body frame = world) ----
BODY_TOP_Z = 0.140       # end of straight cylindrical body
SHOULDER_TOP_Z = 0.195   # end of tapered shoulder, base of neck
NECK_TOP_Z = 0.215       # top of neck rim

BODY_R = 0.035           # body radius (~70mm dia, sports bottle)
NECK_R = 0.014           # outer neck radius
NECK_BORE_R = 0.011      # neck inner bore / mouth opening

# Cap base (wraps down over the neck for real contact)
CAP_BASE_R = 0.019       # cap base outer radius
CAP_BASE_HEIGHT = 0.018  # cap base ring height
# Cap base bottom in world Z: wraps down over upper neck
CAP_BASE_BOT_Z = NECK_TOP_Z - 0.008  # = 0.207; overlaps neck 0.207..0.215

# Hinge geometry (in cap_base local frame)
HINGE_ARM_WIDTH = 0.006
HINGE_ARM_THICK = 0.008  # lug thickness in Y (straddles ring wall)
HINGE_ARM_RISE = 0.014   # lug height above cap base top
HINGE_PIN_R = 0.0018
# Hinge pin center in cap_base local frame
HINGE_Y_LOCAL = CAP_BASE_R  # centered on ring outer surface
HINGE_Z_LOCAL = CAP_BASE_HEIGHT + HINGE_ARM_RISE / 2.0  # mid-height of lug


def _profile_sections():
    """(z, radius) of the outer wall: base -> body -> shoulder -> neck."""
    return [
        (0.000, 0.018),
        (0.008, 0.032),
        (0.018, 0.0348),
        (BODY_TOP_Z, BODY_R),
        (0.155, 0.034),
        (0.170, 0.028),
        (0.183, 0.020),
        (SHOULDER_TOP_Z, 0.015),
        (0.200, NECK_R),
        (NECK_TOP_Z, NECK_R),
    ]


def _bottle_solid() -> cq.Workplane:
    """Hollow sports bottle shell with open mouth at the neck rim."""
    pts = _profile_sections()
    wp = cq.Workplane("XZ").moveTo(0.0, pts[0][0])
    for r, z in [(r, z) for (z, r) in pts]:
        wp = wp.lineTo(r, z)
    wp = wp.lineTo(0.0, pts[-1][0]).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))

    wall = 0.0018
    inner_pts = [
        (0.012, 0.008),
        (0.030, 0.018),
        (BODY_R - wall, 0.020),
        (BODY_R - wall, BODY_TOP_Z),
        (0.032, 0.155),
        (0.026, 0.170),
        (0.018, 0.183),
        (0.013, SHOULDER_TOP_Z),
        (NECK_BORE_R, 0.200),
        (NECK_BORE_R, NECK_TOP_Z + 0.005),
    ]
    iwp = cq.Workplane("XZ").moveTo(0.0, inner_pts[0][1])
    for r, z in inner_pts:
        iwp = iwp.lineTo(r, z)
    iwp = iwp.lineTo(0.0, inner_pts[-1][1]).close()
    cavity = iwp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    return outer.cut(cavity)


def _cap_base_solid() -> cq.Workplane:
    """Cap base ring with two hinge lugs on the +Y side.

    Local frame: z=0 is the bottom of the ring, which in world is at
    CAP_BASE_BOT_Z. The ring wraps over the upper neck for contact.
    """
    # Main cap ring
    cap = (
        cq.Workplane("XY")
        .circle(CAP_BASE_R)
        .extrude(CAP_BASE_HEIGHT)
    )
    # Bore: tight fit on the neck outer wall (no clearance gap)
    bore = (
        cq.Workplane("XY")
        .circle(NECK_R)
        .extrude(CAP_BASE_HEIGHT)
    )
    cap = cap.cut(bore)

    # Two hinge lugs on +Y side, spaced apart along X.
    # Lugs start inside the cap ring (overlap into the solid) for mesh connectivity.
    lug_spacing = 0.014
    lug_embed = 0.006  # how far the lug extends down into the ring body
    for x_off in [-lug_spacing / 2.0, lug_spacing / 2.0]:
        lug = (
            cq.Workplane("XY")
            .workplane(offset=CAP_BASE_HEIGHT - lug_embed)
            .center(x_off, HINGE_Y_LOCAL)
            .rect(HINGE_ARM_WIDTH, HINGE_ARM_THICK)
            .extrude(HINGE_ARM_RISE + lug_embed)
        )
        cap = cap.union(lug)

    return cap


def _flip_lid_solid() -> cq.Workplane:
    """Swing-top stopper with straw nub.

    Part frame origin = hinge pin center. The lid disc extends from the
    hinge toward -Y and -Z so it covers the mouth when the hinge is at q=0.
    """
    lid_r = CAP_BASE_R - 0.002
    lid_thick = 0.006
    # Lid disc center offset from hinge pivot in local frame
    lid_cy = -HINGE_Y_LOCAL  # center the disc over the mouth
    lid_cz = -HINGE_Z_LOCAL + 0.004  # sit just above the neck rim

    lid = (
        cq.Workplane("XY")
        .workplane(offset=lid_cz)
        .center(0.0, lid_cy)
        .circle(lid_r)
        .extrude(lid_thick)
    )

    # Straw nub on top of the lid
    straw_r = 0.004
    straw_h = 0.016
    straw = (
        cq.Workplane("XY")
        .workplane(offset=lid_cz + lid_thick)
        .center(0.0, lid_cy - 0.006)
        .circle(straw_r)
        .extrude(straw_h)
    )
    lid = lid.union(straw)

    # Hollow straw bore
    straw_bore = (
        cq.Workplane("XY")
        .workplane(offset=lid_cz - 0.001)
        .center(0.0, lid_cy - 0.006)
        .circle(straw_r - 0.0012)
        .extrude(straw_h + 0.002)
    )
    lid = lid.cut(straw_bore)

    # Hinge barrel on the +Y side (connects to the cap base lugs)
    barrel_outer_r = HINGE_PIN_R + 0.0012
    barrel_half = 0.008
    barrel = (
        cq.Workplane("XZ")
        .workplane(offset=HINGE_Y_LOCAL)
        .circle(barrel_outer_r)
        .extrude(barrel_half, both=True)
    )
    lid = lid.union(barrel)

    # Connecting bridge from the lid disc edge to the hinge barrel.
    # This ensures the hinge barrel is mesh-connected to the lid body.
    bridge_y_start = lid_cy + lid_r - 0.002  # overlap into the lid disc
    bridge_y_end = HINGE_Y_LOCAL  # reach the barrel
    bridge_len_y = bridge_y_end - bridge_y_start
    bridge_cy = (bridge_y_start + bridge_y_end) / 2.0
    bridge_cz = lid_cz + lid_thick / 2.0  # center on lid thickness
    bridge = (
        cq.Workplane("XY")
        .workplane(offset=bridge_cz - 0.003)
        .center(0.0, bridge_cy)
        .rect(0.008, bridge_len_y + 0.004)
        .extrude(0.006)
    )
    lid = lid.union(bridge)

    return lid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sports_bottle")

    clear_body = model.material("clear_pet", rgba=(0.75, 0.88, 0.92, 0.30))
    dark_gray = model.material("cap_gray", rgba=(0.25, 0.28, 0.30, 1.0))
    lid_color = model.material("lid_blue", rgba=(0.15, 0.45, 0.75, 1.0))

    # ---- bottle body (root) ----
    body = model.part("bottle_body")
    body.visual(
        mesh_from_cadquery(_bottle_solid(), "bottle_shell"),
        material=clear_body,
        name="bottle_shell",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.035,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- cap base (fixed ring with hinge lugs) ----
    cap_base = model.part("cap_base")
    cap_base.visual(
        mesh_from_cadquery(_cap_base_solid(), "cap_base_shell"),
        material=dark_gray,
        name="cap_base_shell",
    )
    cap_base.inertial = Inertial.from_geometry(
        Cylinder(CAP_BASE_R, CAP_BASE_HEIGHT + HINGE_ARM_RISE),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, (CAP_BASE_HEIGHT + HINGE_ARM_RISE) / 2.0)),
    )

    # ---- flip lid (swing-top stopper) ----
    flip_lid = model.part("flip_lid")
    flip_lid.visual(
        mesh_from_cadquery(_flip_lid_solid(), "flip_lid_shell"),
        material=lid_color,
        name="flip_lid_shell",
    )
    flip_lid.inertial = Inertial.from_geometry(
        Cylinder(0.015, 0.024),
        mass=0.005,
        origin=Origin(xyz=(0.0, -HINGE_Y_LOCAL, -0.005)),
    )

    # Fixed joint: cap base ring over the bottle neck
    model.articulation(
        "body_to_cap",
        ArticulationType.FIXED,
        parent=body,
        child=cap_base,
        origin=Origin(xyz=(0.0, 0.0, CAP_BASE_BOT_Z)),
    )

    # Revolute hinge: flip lid pivots on side hinge arms.
    # Origin at the hinge pin center (in cap_base local frame).
    # The lid extends from the hinge toward -Y and -Z (covers the mouth at q=0).
    # axis=(1,0,0): positive q swings the lid open (backward, away from mouth).
    model.articulation(
        "flip_hinge",
        ArticulationType.REVOLUTE,
        parent=cap_base,
        child=flip_lid,
        origin=Origin(xyz=(0.0, HINGE_Y_LOCAL, HINGE_Z_LOCAL)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=2.4,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle_body")
    cap_base = object_model.get_part("cap_base")
    flip_lid = object_model.get_part("flip_lid")
    hinge = object_model.get_articulation("flip_hinge")

    # --- bottle is clear (transparent PET) ---
    clear_mat = next(m for m in object_model.materials if m.name == "clear_pet")
    a = clear_mat.rgba[3] if clear_mat.rgba is not None else 1.0
    ctx.check(
        "bottle shell is transparent",
        a < 1.0,
        details=f"clear_pet alpha={a}",
    )

    # --- bottle is tall and tapered ---
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
    ctx.check(
        "tapered shoulder narrows toward the top",
        NECK_R < BODY_R * 0.5,
        details=f"neck_r={NECK_R}, body_r={BODY_R}",
    )

    # --- cap base is at the top of the bottle ---
    cap_pos = ctx.part_world_position(cap_base)
    ctx.check(
        "cap base mounted at bottle top",
        cap_pos is not None and cap_pos[2] > 0.19,
        details=f"cap_base position={cap_pos}",
    )

    # --- cap base intentionally overlaps the neck (tight ring fit) ---
    ctx.allow_overlap(
        cap_base,
        body,
        elem_a="cap_base_shell",
        elem_b="bottle_shell",
        reason="The cap base ring intentionally press-fits over the neck outer wall.",
    )
    # Prove the cap base is seated at the neck
    ctx.expect_contact(
        cap_base,
        body,
        elem_a="cap_base_shell",
        elem_b="bottle_shell",
        contact_tol=0.002,
        name="cap base contacts bottle neck",
    )

    # --- flip lid exists at the top ---
    lid_pos = ctx.part_world_position(flip_lid)
    ctx.check(
        "flip lid is at the top of the bottle",
        lid_pos is not None and lid_pos[2] > 0.20,
        details=f"flip_lid position={lid_pos}",
    )

    # --- flip hinge is a non-fixed revolute joint ---
    ctx.check(
        "flip hinge is a revolute joint",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={hinge.articulation_type}",
    )
    limits = hinge.motion_limits
    ctx.check(
        "flip hinge has motion limits",
        limits is not None and limits.lower is not None and limits.upper is not None,
        details=f"limits={limits}",
    )
    if limits is not None and limits.upper is not None:
        ctx.check(
            "flip hinge opens more than 90 degrees",
            limits.upper > 1.5,
            details=f"upper={limits.upper}",
        )

    # --- flip lid opens when articulated ---
    lid_aabb_rest = ctx.part_world_aabb(flip_lid)
    lid_z_rest = lid_aabb_rest[0][2]
    with ctx.pose({hinge: 1.5}):
        lid_aabb_open = ctx.part_world_aabb(flip_lid)
        lid_z_open = lid_aabb_open[0][2]
    ctx.check(
        "flip lid moves when hinge opens",
        abs(lid_z_open - lid_z_rest) > 0.003,
        details=f"rest_bottom_z={lid_z_rest:.4f}, open_bottom_z={lid_z_open:.4f}",
    )

    # --- hinge barrel on the flip lid overlaps hinge region on cap base ---
    ctx.allow_overlap(
        flip_lid,
        cap_base,
        elem_a="flip_lid_shell",
        elem_b="cap_base_shell",
        reason="The flip lid hinge barrel wraps around the hinge pin between the cap base lugs.",
    )

    return ctx.report()


object_model = build_object_model()
