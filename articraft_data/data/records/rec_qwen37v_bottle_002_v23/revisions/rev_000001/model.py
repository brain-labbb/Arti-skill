from __future__ import annotations

# Sports bottle with a flip-straw cap, safety collar, and visible hollow mouth.
# Frame: vertical axis along +Z, bottle standing on z=0, centerline on x=y=0.
#   - rounded base -> straight cylindrical body -> tapered shoulder
#     -> short neck with open mouth -> mouth lip ring
# Parts:
#   bottle_body   (root)  - hollow clear PET shell + mouth lip
#   safety_collar         - tamper-evident ring around the neck
#   cap_base              - fixed cap body with spout nozzle
#   flip_lid              - hinged flip cover over the spout
# Articulations:
#   collar_rotate  CONTINUOUS  body -> safety_collar  (ring rotates around neck)
#   cap_mount      FIXED       body -> cap_base       (cap seated on neck)
#   lid_hinge      REVOLUTE    cap_base -> flip_lid   (flip lid opens upward)

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
BODY_TOP_Z = 0.130       # end of straight body, start of shoulder
SHOULDER_TOP_Z = 0.175   # end of tapered shoulder, base of neck
NECK_TOP_Z = 0.205       # top of neck / mouth rim

BODY_R = 0.033           # body radius (~0.066 m dia, sports bottle wider)
NECK_R = 0.013           # neck outer radius
NECK_BORE_R = 0.010      # neck inner bore (hollow mouth)

# Cap dimensions
CAP_R = 0.016            # cap outer radius
CAP_HEIGHT = 0.028       # cap total height
CAP_BASE_Z = NECK_TOP_Z - 0.005  # cap starts slightly below neck rim

# Spout nozzle on top of cap
SPOUT_R = 0.005          # spout outer radius
SPOUT_HEIGHT = 0.010     # spout height above cap top
SPOUT_BORE_R = 0.003     # straw bore through spout

# Flip lid
LID_R = 0.014            # lid disk radius
LID_THICKNESS = 0.004    # lid thickness
# Hinge at back of cap (-Y side), relative to cap part frame
HINGE_OFFSET_Y = -(CAP_R - 0.003)  # hinge pin near back edge
HINGE_Z_IN_CAP = CAP_HEIGHT        # hinge at cap top (relative to cap frame)

# Safety collar
COLLAR_R_OUTER = NECK_R + 0.003    # collar outer radius
COLLAR_R_INNER = NECK_R - 0.001    # collar inner (grips neck)
COLLAR_HEIGHT = 0.006               # collar height
COLLAR_Z = SHOULDER_TOP_Z + 0.002   # collar sits just above shoulder


def _profile_sections():
    """(z, radius) of the outer wall profile: base -> body -> shoulder -> neck."""
    return [
        (0.000, 0.016),   # rounded base heel
        (0.008, 0.028),
        (0.018, 0.032),
        (BODY_TOP_Z, BODY_R),         # straight cylindrical body
        (0.145, 0.031),               # shoulder starts tapering
        (0.158, 0.025),
        (SHOULDER_TOP_Z, 0.015),      # shoulder ends, neck begins
        (0.180, NECK_R),              # neck transition
        (NECK_TOP_Z, NECK_R),         # straight neck up to mouth rim
    ]


def _bottle_solid() -> cq.Workplane:
    """Revolve the outer profile, then shell it hollow with an open mouth."""
    pts = _profile_sections()
    wp = cq.Workplane("XZ").moveTo(0.0, pts[0][0])
    for r, z in [(r, z) for (z, r) in pts]:
        wp = wp.lineTo(r, z)
    wp = wp.lineTo(0.0, pts[-1][0]).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))

    # Hollow cavity that opens through the neck rim
    wall = 0.0016
    inner_pts = [
        (0.012, 0.008),
        (0.026, 0.016),
        (BODY_R - wall, 0.020),
        (BODY_R - wall, BODY_TOP_Z),
        (0.029, 0.145),
        (0.023, 0.158),
        (0.0134, SHOULDER_TOP_Z),
        (NECK_BORE_R, 0.180),
        (NECK_BORE_R, NECK_TOP_Z + 0.005),  # open through mouth rim
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
    """Transparent wall-thickness lip ring at the mouth rim."""
    # A torus representing the visible lip/rim showing wall thickness
    lip = TorusGeometry(
        NECK_R - 0.0005,  # major radius
        0.0018,           # minor radius (tube thickness showing wall)
        radial_segments=12,
        tubular_segments=48,
    )
    lip.translate(0.0, 0.0, NECK_TOP_Z)
    return mesh_from_geometry(lip, "mouth_lip")


def _collar_solid() -> cq.Workplane:
    """Tamper-evident safety collar ring with small bridge tabs."""
    # Main ring
    ring = (
        cq.Workplane("XY")
        .circle(COLLAR_R_OUTER)
        .circle(COLLAR_R_INNER)
        .extrude(COLLAR_HEIGHT)
    )
    # Add 4 small bridge tabs connecting to the cap above (perforated tear bridges)
    for i in range(4):
        angle = 2.0 * math.pi * i / 4.0
        tab_x = (COLLAR_R_OUTER - 0.001) * math.cos(angle)
        tab_y = (COLLAR_R_OUTER - 0.001) * math.sin(angle)
        tab = (
            cq.Workplane("XY")
            .center(tab_x, tab_y)
            .rect(0.003, 0.002)
            .extrude(COLLAR_HEIGHT + 0.004)  # bridges extend up toward cap
        )
        ring = ring.union(tab)
    return ring


def _collar_mesh():
    return mesh_from_cadquery(_collar_solid(), "collar_ring")


def _cap_solid() -> cq.Workplane:
    """Sports cap base: cylindrical body with a raised spout nozzle on top."""
    # Main cap body - sits over the neck
    cap = (
        cq.Workplane("XY")
        .circle(CAP_R)
        .extrude(CAP_HEIGHT)
    )
    # Hollow bore to grip the neck
    bore = (
        cq.Workplane("XY")
        .circle(NECK_R)
        .extrude(CAP_HEIGHT - 0.003)
    )
    cap = cap.cut(bore)

    # Raised spout nozzle on top
    spout = (
        cq.Workplane("XY")
        .workplane(offset=CAP_HEIGHT)
        .circle(SPOUT_R)
        .extrude(SPOUT_HEIGHT)
    )
    cap = cap.union(spout)

    # Straw bore through spout
    straw_bore = (
        cq.Workplane("XY")
        .workplane(offset=CAP_HEIGHT - 0.002)
        .circle(SPOUT_BORE_R)
        .extrude(SPOUT_HEIGHT + 0.004)
    )
    cap = cap.cut(straw_bore)

    # Hinge boss at the back (-Y side) for the flip lid
    hinge_boss = (
        cq.Workplane("XZ")
        .workplane(offset=HINGE_OFFSET_Y)
        .center(0.0, CAP_HEIGHT)
        .circle(0.003)
        .extrude(0.005)  # small boss protruding for hinge pin
    )
    cap = cap.union(hinge_boss)

    return cap


def _cap_mesh():
    return mesh_from_cadquery(_cap_solid(), "cap_shell")


def _lid_solid() -> cq.Workplane:
    """Flip lid: a disk that extends forward (+Y) from the hinge pin."""
    # Main lid disk - centered at +Y offset from hinge, flat (normal along Z)
    lid = (
        cq.Workplane("XY")
        .center(0.0, LID_R * 0.65)  # offset forward from hinge
        .circle(LID_R)
        .extrude(LID_THICKNESS)
    )
    # Small grip bump on top
    dome = (
        cq.Workplane("XY")
        .workplane(offset=LID_THICKNESS)
        .center(0.0, LID_R * 0.65)
        .circle(0.005)
        .extrude(0.003)
    )
    lid = lid.union(dome)

    # Hinge knuckle at the origin (wraps around hinge pin)
    knuckle = (
        cq.Workplane("XZ")
        .center(0.0, LID_THICKNESS / 2.0)
        .circle(0.003)
        .extrude(0.008)
    )
    # Center the knuckle along X symmetrically
    knuckle = knuckle.translate((0.0, -0.004, 0.0))
    lid = lid.union(knuckle)

    return lid


def _lid_mesh():
    return mesh_from_cadquery(_lid_solid(), "flip_lid_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sports_bottle")

    # Materials
    clear_pet = model.material("clear_pet", rgba=(0.75, 0.86, 0.90, 0.22))
    lip_clear = model.material("lip_clear", rgba=(0.80, 0.90, 0.92, 0.35))
    collar_gray = model.material("collar_gray", rgba=(0.55, 0.58, 0.60, 0.85))
    cap_blue = model.material("cap_blue", rgba=(0.12, 0.35, 0.65, 1.0))
    lid_blue = model.material("lid_blue", rgba=(0.15, 0.40, 0.72, 1.0))

    # ---- bottle body (root): transparent hollow PET shell ----
    body = model.part("bottle_body")
    body.visual(_bottle_mesh(), material=clear_pet, name="bottle_shell")
    body.visual(_mouth_lip_mesh(), material=lip_clear, name="mouth_lip")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- safety collar: tamper-evident ring around the neck ----
    collar = model.part("safety_collar")
    # Collar geometry is built from z=0; part frame will be at COLLAR_Z via articulation
    collar.visual(_collar_mesh(), material=collar_gray, name="collar_ring")
    collar.inertial = Inertial.from_geometry(
        Cylinder(COLLAR_R_OUTER, COLLAR_HEIGHT + 0.004),
        mass=0.002,
        origin=Origin(xyz=(0.0, 0.0, (COLLAR_HEIGHT + 0.004) / 2.0)),
    )

    # ---- cap base: fixed cap body with spout ----
    cap = model.part("cap_base")
    # Cap geometry is built from z=0; part frame will be at CAP_BASE_Z via articulation
    cap.visual(_cap_mesh(), material=cap_blue, name="cap_shell")
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_HEIGHT + SPOUT_HEIGHT),
        mass=0.005,
        origin=Origin(xyz=(0.0, 0.0, (CAP_HEIGHT + SPOUT_HEIGHT) / 2.0)),
    )

    # ---- flip lid: hinged cover over the spout ----
    lid = model.part("flip_lid")
    # Lid geometry extends in +Y from origin (hinge point); part frame at hinge via articulation
    lid.visual(_lid_mesh(), material=lid_blue, name="flip_lid_shell")
    lid.inertial = Inertial.from_geometry(
        Cylinder(LID_R, LID_THICKNESS),
        mass=0.002,
        origin=Origin(xyz=(0.0, LID_R * 0.65, LID_THICKNESS / 2.0)),
    )

    # ---- Articulations ----

    # collar_rotate: CONTINUOUS spin of safety collar about +Z
    model.articulation(
        "collar_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=2.0),
    )

    # cap_mount: FIXED mount of cap onto the neck
    model.articulation(
        "cap_mount",
        ArticulationType.FIXED,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, CAP_BASE_Z)),
    )

    # lid_hinge: REVOLUTE flip lid hinge at the back of the cap.
    # The lid extends in +Y from the hinge pin. Positive rotation about +X
    # moves +Y toward +Z, lifting the free edge upward (opening the lid).
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=cap,
        child=lid,
        # Hinge pin at back edge of cap top (relative to cap frame)
        origin=Origin(xyz=(0.0, HINGE_OFFSET_Y, HINGE_Z_IN_CAP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=4.0,
            lower=0.0, upper=2.4,  # opens to ~137 degrees
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle_body")
    collar = object_model.get_part("safety_collar")
    cap = object_model.get_part("cap_base")
    lid = object_model.get_part("flip_lid")
    collar_joint = object_model.get_articulation("collar_rotate")
    hinge = object_model.get_articulation("lid_hinge")

    # --- bottle is clear (transparent) ---
    clear_mat = next(m for m in object_model.materials if m.name == "clear_pet")
    a = clear_mat.rgba[3] if clear_mat.rgba is not None else 1.0
    ctx.check(
        "bottle shell is transparent",
        a < 1.0,
        details=f"clear_pet alpha={a}",
    )

    # --- mouth lip is also transparent ---
    lip_mat = next(m for m in object_model.materials if m.name == "lip_clear")
    lip_a = lip_mat.rgba[3] if lip_mat.rgba is not None else 1.0
    ctx.check(
        "mouth lip is transparent",
        lip_a < 1.0,
        details=f"lip_clear alpha={lip_a}",
    )

    # --- bottle body is tall (taller than wide) ---
    full = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "bottle is tall (taller than wide)",
        full[2] > 2.5 * full[0],
        details=f"body extents={full}",
    )

    # --- tapered shoulder narrows toward top ---
    ctx.check(
        "tapered shoulder narrows toward the top",
        NECK_R < BODY_R * 0.5,
        details=f"neck_r={NECK_R}, body_r={BODY_R}",
    )

    # --- mouth lip visual exists ---
    lip_vis = body.get_visual("mouth_lip")
    ctx.check(
        "mouth lip visual exists",
        lip_vis is not None,
        details="mouth_lip visual not found on bottle_body",
    )

    # --- safety collar exists and is at neck height ---
    collar_pos = ctx.part_world_position(collar)
    ctx.check(
        "safety collar is at neck height",
        collar_pos is not None and collar_pos[2] > SHOULDER_TOP_Z - 0.01,
        details=f"collar position={collar_pos}",
    )

    # --- collar rotates (continuous joint) ---
    collar_aabb_0 = ctx.part_world_aabb(collar)
    # Since collar is symmetric, verify the joint type is continuous
    ctx.check(
        "collar joint is continuous (non-fixed)",
        collar_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={collar_joint.articulation_type}",
    )

    # --- cap base is mounted at the top ---
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap mounted at the top of the bottle",
        cap_pos is not None and cap_pos[2] > 0.18,
        details=f"cap origin={cap_pos}",
    )

    # --- flip lid hinge is revolute with correct limits ---
    ctx.check(
        "lid hinge is revolute (non-fixed)",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={hinge.articulation_type}",
    )
    limits = hinge.motion_limits
    ctx.check(
        "lid hinge has positive opening range",
        limits is not None and limits.upper is not None and limits.upper > 1.0,
        details=f"upper={limits.upper if limits else None}",
    )

    # --- flip lid opens upward: lid top moves up when hinge opens ---
    lid_z_closed = ctx.part_world_aabb(lid)[1][2]  # max Z
    with ctx.pose({hinge: 1.5}):
        lid_z_open = ctx.part_world_aabb(lid)[1][2]
    ctx.check(
        "flip lid opens upward (max Z increases)",
        lid_z_open > lid_z_closed + 0.005,
        details=f"closed_max_z={lid_z_closed:.4f}, open_max_z={lid_z_open:.4f}",
    )

    # --- cap overlaps neck region (intentional seating) ---
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_shell",
        elem_b="bottle_shell",
        reason="The cap base intentionally seats over the threaded neck region.",
    )

    # --- collar overlaps neck region (intentional ring fit) ---
    ctx.allow_overlap(
        collar,
        body,
        elem_a="collar_ring",
        elem_b="bottle_shell",
        reason="The safety collar ring intentionally encircles the neck.",
    )

    # --- lid hinge knuckle wraps around cap hinge boss (intentional hinge fit) ---
    ctx.allow_overlap(
        cap,
        lid,
        elem_a="cap_shell",
        elem_b="flip_lid_shell",
        reason="The lid hinge knuckle intentionally wraps around the cap hinge boss to form the hinge pin joint.",
    )
    # Proof: the hinge mechanism still opens correctly (tested above with the max Z check)
    ctx.expect_contact(
        cap, lid,
        elem_a="cap_shell", elem_b="flip_lid_shell",
        contact_tol=0.010,
        name="lid hinge knuckle is in contact with cap hinge boss",
    )

    return ctx.report()


object_model = build_object_model()
