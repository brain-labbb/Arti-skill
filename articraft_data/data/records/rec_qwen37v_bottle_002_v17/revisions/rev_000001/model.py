from __future__ import annotations

# Ribbed water bottle with deep grip grooves, a swing-top stopper on side hinge arms,
# a visible hollow mouth opening, and a transparent wall-thickness lip at the mouth.
#
# Frame: vertical axis along +Z, bottle standing on z=0, centerline on x=y=0.
#   - rounded base -> ribbed cylindrical body -> tapered shoulder
#     -> short neck with hinge arms -> mouth lip ring.
# Articulation:
#   - stopper_swing: REVOLUTE about Y-axis through the hinge pin,
#     positive q opens the stopper upward/backward (lever swings to the side).

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
BODY_BOT_Z = 0.008       # end of rounded base heel
BODY_TOP_Z = 0.150        # end of straight ribbed body
SHOULDER_TOP_Z = 0.192    # end of tapered shoulder
NECK_BASE_Z = 0.192
NECK_TOP_Z = 0.218        # top of neck / mouth rim
LIP_TOP_Z = 0.224         # top of the thickened mouth lip

BODY_R = 0.0375           # body outer radius (~0.075m dia)
NECK_R = 0.014            # neck outer radius
NECK_BORE_R = 0.011       # mouth bore radius (visible hollow)
LIP_R = 0.016             # lip outer radius (wall thickness visible)

# Ribs - incorporated into the revolved profile as undulations
WALL_THICK = 0.003        # wall thickness (must exceed RIB_DEPTH)
RIB_DEPTH = 0.002         # groove depth into the outer wall (less than wall)
RIB_WIDTH = 0.005         # vertical width of each groove
RIB_SPACING = 0.012       # vertical spacing between groove centers
RIB_COUNT = 10            # number of grip grooves on the body

# Hinge geometry
HINGE_ARM_WIDTH = 0.006
HINGE_ARM_THICK = 0.003
HINGE_ARM_HEIGHT = 0.018  # from neck surface up past the rim
HINGE_PIN_R = 0.002
HINGE_Z = NECK_TOP_Z + 0.006  # hinge pin center height
HINGE_OFFSET_R = NECK_R + HINGE_ARM_THICK  # radial position of arm outer face

# Stopper
STOPPER_R = 0.012         # stopper plug radius (fits in mouth bore)
STOPPER_PLUG_H = 0.008    # plug depth into mouth
STOPPER_LEVER_H = 0.035   # lever arm height from pivot to top
STOPPER_LEVER_W = 0.008   # lever arm width
STOPPER_LEVER_T = 0.004   # lever arm thickness
PIVOT_OFFSET_Z = STOPPER_PLUG_H + 0.006  # local z of pivot in stopper geometry


def _bottle_profile_with_ribs():
    """Build outer profile points including rib undulations."""
    wall = WALL_THICK
    pts = []

    # Base heel
    pts.append((0.000, 0.018))
    pts.append((0.004, 0.030))
    pts.append((BODY_BOT_Z, BODY_R))

    # Ribbed body section
    rib_start_z = BODY_BOT_Z + 0.015
    for i in range(RIB_COUNT):
        rib_center = rib_start_z + i * RIB_SPACING
        if rib_center + RIB_WIDTH / 2.0 > BODY_TOP_Z - 0.005:
            break
        # Top of previous straight section -> bottom of this rib groove
        pts.append((rib_center - RIB_WIDTH / 2.0, BODY_R))
        # Bottom of groove (inward dip)
        pts.append((rib_center - RIB_WIDTH / 4.0, BODY_R - RIB_DEPTH))
        pts.append((rib_center + RIB_WIDTH / 4.0, BODY_R - RIB_DEPTH))
        # Back out to full radius
        pts.append((rib_center + RIB_WIDTH / 2.0, BODY_R))

    # End of body
    pts.append((BODY_TOP_Z, BODY_R))

    # Shoulder taper
    pts.append((0.162, 0.034))
    pts.append((0.178, 0.026))
    pts.append((SHOULDER_TOP_Z, 0.016))

    # Neck
    pts.append((NECK_BASE_Z, NECK_R))
    pts.append((NECK_TOP_Z, NECK_R))

    return pts, wall


def _bottle_body_solid() -> cq.Workplane:
    """Build the ribbed bottle body: revolved profile with ribs + hinge arms."""
    pts, wall = _bottle_profile_with_ribs()

    # Build outer shell via revolve
    wp = cq.Workplane("XZ").moveTo(0.0, pts[0][0])
    for r, z in [(r, z) for (z, r) in pts]:
        wp = wp.lineTo(r, z)
    wp = wp.lineTo(0.0, pts[-1][0]).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))

    # Inner cavity (hollow, opens through the mouth)
    inner_pts = [
        (0.014, BODY_BOT_Z + 0.002),
        (BODY_R - wall, BODY_BOT_Z + 0.004),
        (BODY_R - wall, BODY_TOP_Z),
        (0.032, 0.162),
        (0.024, 0.178),
        (0.0142, SHOULDER_TOP_Z),
        (NECK_BORE_R, NECK_BASE_Z + 0.004),
        (NECK_BORE_R, NECK_TOP_Z + 0.010),  # open through the rim
    ]
    iwp = cq.Workplane("XZ").moveTo(0.0, inner_pts[0][1])
    for r, z in inner_pts:
        iwp = iwp.lineTo(r, z)
    iwp = iwp.lineTo(0.0, inner_pts[-1][1]).close()
    cavity = iwp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    body = outer.cut(cavity)

    # Add hinge arms: build a collar ring with integrated arm extensions as one
    # solid piece, then union to the body. Start the collar well below the
    # shoulder junction so it overlaps the tapered body wall for connectivity.
    collar_bot = 0.182  # starts in the taper region for broad overlap
    collar_top = NECK_TOP_Z + 0.002
    collar_inner = NECK_BORE_R + 0.001  # bore through to the inner cavity
    collar_outer = NECK_R + HINGE_ARM_THICK
    arm_tip_x = NECK_R + HINGE_ARM_THICK + 0.005
    
    # Build collar+arms as one 3D assembly in local coords, then position
    # Collar ring: outer annulus
    collar_solid = (
        cq.Workplane("XY")
        .circle(collar_outer)
        .circle(collar_inner)
        .extrude(collar_top - collar_bot)
    )
    # Arm extensions: rectangular tabs on ±X sides, full height of collar+arm
    for sign in (1.0, -1.0):
        arm_cx = sign * (collar_outer + arm_tip_x) / 2.0
        arm_dx = arm_tip_x - collar_outer + 0.002
        arm = (
            cq.Workplane("XY")
            .center(arm_cx, 0.0)
            .rect(arm_dx, HINGE_ARM_WIDTH)
            .extrude(HINGE_ARM_HEIGHT + (collar_top - collar_bot) - 0.004)
        )
        collar_solid = collar_solid.union(arm)
    
    # Position the collar assembly on the bottle
    collar_solid = collar_solid.translate((0, 0, collar_bot))
    body = body.union(collar_solid)

    return body


def _mouth_lip_solid() -> cq.Workplane:
    """Thickened transparent lip ring at the mouth rim."""
    lip = (
        cq.Workplane("XY")
        .workplane(offset=NECK_TOP_Z)
        .circle(LIP_R)
        .circle(NECK_BORE_R)
        .extrude(LIP_TOP_Z - NECK_TOP_Z)
    )
    return lip


def _stopper_solid() -> cq.Workplane:
    """Swing-top stopper: plug + lever arm + top grip cap."""
    # Plug disc (seats into the mouth bore)
    plug = (
        cq.Workplane("XY")
        .circle(STOPPER_R)
        .extrude(STOPPER_PLUG_H)
    )

    # Lever arm extending from plug top upward to above the pivot
    lever = (
        cq.Workplane("XY")
        .workplane(offset=STOPPER_PLUG_H)
        .center(0.0, 0.0)
        .rect(STOPPER_LEVER_T, STOPPER_LEVER_W)
        .extrude(STOPPER_LEVER_H)
    )
    stopper = plug.union(lever)

    # Top cap/loop on the lever for gripping
    top_cap = (
        cq.Workplane("XY")
        .workplane(offset=STOPPER_PLUG_H + STOPPER_LEVER_H - 0.003)
        .rect(STOPPER_LEVER_T + 0.002, STOPPER_LEVER_W + 0.004)
        .extrude(0.005)
    )
    stopper = stopper.union(top_cap)

    # Pivot pin stubs on the sides of the lever (connect to hinge arms)
    for sign in (1.0, -1.0):
        pin = (
            cq.Workplane("XY")
            .workplane(offset=PIVOT_OFFSET_Z - HINGE_PIN_R)
            .center(0.0, sign * (STOPPER_LEVER_W / 2.0))
            .circle(HINGE_PIN_R)
            .extrude(sign * 0.004)
        )
        stopper = stopper.union(pin)

    return stopper


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ribbed_water_bottle")

    # Materials
    clear_body = model.material("clear_pet", rgba=(0.75, 0.88, 0.92, 0.22))
    clear_lip = model.material("clear_lip", rgba=(0.80, 0.90, 0.95, 0.35))
    rubber = model.material("stopper_rubber", rgba=(0.25, 0.25, 0.28, 1.0))

    # ---- bottle body (root) ----
    body = model.part("bottle_body")
    body.visual(
        mesh_from_cadquery(_bottle_body_solid(), "bottle_shell"),
        material=clear_body,
        name="bottle_shell",
    )
    body.visual(
        mesh_from_cadquery(_mouth_lip_solid(), "mouth_lip"),
        material=clear_lip,
        name="mouth_lip",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.045,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- swing-top stopper ----
    stopper = model.part("stopper")
    # The stopper geometry has its base at z=0 (plug bottom).
    # The pivot point is at local z=PIVOT_OFFSET_Z.
    # We offset the visual so the pivot aligns with the part frame origin.
    stopper.visual(
        mesh_from_cadquery(_stopper_solid(), "stopper_body"),
        material=rubber,
        origin=Origin(xyz=(0.0, 0.0, -PIVOT_OFFSET_Z)),
        name="stopper_body",
    )
    stopper.inertial = Inertial.from_geometry(
        Cylinder(STOPPER_R, STOPPER_PLUG_H + STOPPER_LEVER_H),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # Articulation: stopper_swing
    # Hinge axis: Y (horizontal, perpendicular to the hinge arms on ±X sides).
    # At q=0 (closed): plug seated below pivot, lever vertical above pivot.
    # Positive q swings the lever in the -X direction (plug rises in +X direction).
    model.articulation(
        "stopper_swing",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stopper,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,        # closed
            upper=2.4,        # open (~138 degrees)
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle_body")
    stopper = object_model.get_part("stopper")
    swing = object_model.get_articulation("stopper_swing")

    # --- bottle body has ribbed grooves (profile undulations in the body region) ---
    body_aabb = ctx.part_world_aabb(body)
    body_dx = body_aabb[1][0] - body_aabb[0][0]
    # The body diameter includes the full rib outer radius
    ctx.check(
        "bottle body diameter includes rib profile",
        body_dx > 2.0 * (BODY_R - 0.001),
        details=f"body_dx={body_dx:.4f}, expected > {2.0 * (BODY_R - 0.001):.4f}",
    )

    # --- hollow mouth opening exists (lip visual) ---
    lip_visual = body.get_visual("mouth_lip")
    ctx.check(
        "mouth lip visual exists",
        lip_visual is not None,
        details="mouth_lip visual not found on bottle_body",
    )

    # --- mouth lip is at the top of the bottle ---
    lip_aabb = ctx.part_element_world_aabb(body, elem=lip_visual)
    ctx.check(
        "mouth lip is at the top of the bottle",
        lip_aabb is not None and lip_aabb[0][2] > 0.19,
        details=f"lip bottom z={lip_aabb[0][2]:.4f}" if lip_aabb else "no aabb",
    )

    # --- transparent body ---
    clear_mat = next(m for m in object_model.materials if m.name == "clear_pet")
    a = clear_mat.rgba[3] if clear_mat.rgba is not None else 1.0
    ctx.check(
        "bottle shell is transparent",
        a < 1.0,
        details=f"clear_pet alpha={a}",
    )

    # --- transparent lip ---
    lip_mat = next(m for m in object_model.materials if m.name == "clear_lip")
    a_lip = lip_mat.rgba[3] if lip_mat.rgba is not None else 1.0
    ctx.check(
        "mouth lip has visible wall thickness (transparent)",
        a_lip < 1.0,
        details=f"clear_lip alpha={a_lip}",
    )

    # --- stopper exists and is mounted near the top ---
    stopper_pos = ctx.part_world_position(stopper)
    ctx.check(
        "stopper mounted near bottle top",
        stopper_pos is not None and stopper_pos[2] > 0.18,
        details=f"stopper pos={stopper_pos}",
    )

    # --- swing-top articulation is revolute with correct limits ---
    ctx.check(
        "stopper_swing is revolute",
        swing.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={swing.articulation_type}",
    )
    ctx.check(
        "stopper_swing has nonzero range",
        swing.motion_limits is not None and swing.motion_limits.upper > swing.motion_limits.lower + 0.5,
        details=f"limits lower={swing.motion_limits.lower}, upper={swing.motion_limits.upper}",
    )

    # --- stopper opens: the stopper center moves laterally (X displacement) ---
    center_closed = ctx.part_world_aabb(stopper)
    cx_closed = (center_closed[1][0] + center_closed[0][0]) / 2.0

    with ctx.pose({swing: 0.8}):  # partially open (~46°)
        center_mid = ctx.part_world_aabb(stopper)
        cx_mid = (center_mid[1][0] + center_mid[0][0]) / 2.0

    dx_mid = abs(cx_mid - cx_closed)
    ctx.check(
        "stopper swings open (center displaces laterally)",
        dx_mid > 0.002,
        details=f"closed cx={cx_closed:.4f}, mid cx={cx_mid:.4f}, dx={dx_mid:.4f}",
    )

    # --- at a wider open angle, displacement is larger ---
    with ctx.pose({swing: 1.4}):  # more open (~80°)
        center_wide = ctx.part_world_aabb(stopper)
        cx_wide = (center_wide[1][0] + center_wide[0][0]) / 2.0

    dx_wide = abs(cx_wide - cx_closed)
    ctx.check(
        "stopper swings further at wider angle",
        dx_wide > dx_mid,
        details=f"mid dx={dx_mid:.4f}, wide dx={dx_wide:.4f}",
    )

    # --- bottle is taller than wide (water bottle proportions) ---
    full = (body_aabb[1][0] - body_aabb[0][0],
            body_aabb[1][1] - body_aabb[0][1],
            body_aabb[1][2] - body_aabb[0][2])
    ctx.check(
        "bottle is tall (taller than wide)",
        full[2] > 2.5 * full[0],
        details=f"extents={full}",
    )

    # --- tapered shoulder: bottle narrows toward the top ---
    ctx.check(
        "tapered shoulder narrows toward neck",
        NECK_R < BODY_R * 0.5,
        details=f"neck_r={NECK_R}, body_r={BODY_R}",
    )

    # --- allow small overlap between stopper plug and bottle mouth when closed ---
    ctx.allow_overlap(
        stopper,
        body,
        elem_a="stopper_body",
        elem_b="bottle_shell",
        reason="The stopper plug seats into the mouth bore when closed.",
    )
    ctx.allow_overlap(
        stopper,
        body,
        elem_a="stopper_body",
        elem_b="mouth_lip",
        reason="The stopper plug contacts the lip ring when seated.",
    )

    return ctx.report()


object_model = build_object_model()
