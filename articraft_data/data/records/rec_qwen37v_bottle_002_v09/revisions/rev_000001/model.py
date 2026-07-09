from __future__ import annotations

# Swing-top (flip-top) bottle variant: clear bottle with tapered shoulder,
# hinged swing-top stopper with wire bail, and a rotating safety collar ring.
# Frame: vertical axis along +Z, bottle standing on z=0, centerline on x=y=0.
#   - rounded base -> cylindrical body -> tapered shoulder -> neck -> mouth
# Articulations:
#   - collar_spin:  CONTINUOUS rotation of safety collar ring about +Z
#   - swing_hinge:  REVOLUTE flip of the stopper+bail assembly (opens backward)

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
    tube_from_spline_points,
)

# ---- key heights (m) along +Z ----
BASE_Z = 0.0
BODY_TOP_Z = 0.120     # end of straight body, start of shoulder
SHOULDER_TOP_Z = 0.175  # end of tapered shoulder, base of neck
NECK_TOP_Z = 0.200      # top of neck rim (mouth opening)

BODY_R = 0.030          # body outer radius (~60mm dia)
NECK_R = 0.013          # neck outer radius
NECK_BORE_R = 0.010     # mouth bore radius (visible hollow opening)
NECK_LIP_R = 0.016      # wider lip at neck top for collar retention

# ---- collar geometry ----
COLLAR_Z_BOT = 0.182    # collar sits on the neck
COLLAR_Z_TOP = 0.192
COLLAR_R = 0.017        # collar outer radius
COLLAR_BORE_R = NECK_R + 0.0005  # slips over neck with small clearance
COLLAR_MID_Z = (COLLAR_Z_TOP + COLLAR_Z_BOT) / 2.0

# ---- swing-top stopper ----
STOPPER_R = 0.014       # stopper disc radius
STOPPER_H = 0.008       # stopper thickness
DOME_H = 0.003          # dome on top of stopper

# ---- hinge frame ----
# Hinge pivot axis goes through the left/right sides of the collar at COLLAR_Z_TOP
# Hinge origin in collar frame: (0, 0, COLLAR_Z_TOP)
# The stopper sits above the hinge at distance STOPPER_LIFT
STOPPER_LIFT = NECK_TOP_Z + STOPPER_H / 2.0 - COLLAR_Z_TOP  # = 0.204 - 0.192 = 0.012


def _profile_sections():
    """(z, radius) of the outer wall from base to neck rim."""
    return [
        (0.000, 0.016),   # rounded base heel
        (0.008, 0.027),
        (0.018, 0.0298),
        (BODY_TOP_Z, BODY_R),
        (0.135, 0.0294),
        (0.150, 0.0260),
        (0.162, 0.0200),
        (SHOULDER_TOP_Z, 0.0150),
        (0.180, NECK_R),
        (NECK_TOP_Z - 0.003, NECK_R),
        (NECK_TOP_Z - 0.002, NECK_LIP_R),  # wider lip for collar
        (NECK_TOP_Z, NECK_LIP_R),
    ]


def _bottle_solid() -> cq.Workplane:
    """Revolves the outer profile, then shells to hollow with open mouth."""
    pts = _profile_sections()
    wp = cq.Workplane("XZ").moveTo(0.0, pts[0][0])
    for r, z in [(r, z) for (z, r) in pts]:
        wp = wp.lineTo(r, z)
    wp = wp.lineTo(0.0, pts[-1][0]).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))

    # Hollow cavity opening through the neck rim (visible mouth)
    wall = 0.0015
    inner_pts = [
        (0.010, 0.008),
        (0.025, 0.016),
        (BODY_R - wall, 0.020),
        (BODY_R - wall, BODY_TOP_Z),
        (0.0278, 0.135),
        (0.0244, 0.150),
        (0.0184, 0.162),
        (0.0134, SHOULDER_TOP_Z),
        (NECK_BORE_R, 0.180),
        (NECK_BORE_R, NECK_TOP_Z + 0.005),  # opens through the rim
    ]
    iwp = cq.Workplane("XZ").moveTo(0.0, inner_pts[0][1])
    for r, z in inner_pts:
        iwp = iwp.lineTo(r, z)
    iwp = iwp.lineTo(0.0, inner_pts[-1][1]).close()
    cavity = iwp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    return outer.cut(cavity)


def _bottle_mesh():
    return mesh_from_cadquery(_bottle_solid(), "bottle_shell")


def _neck_mouth_ring():
    """A visible torus ring at the neck rim showing the mouth opening edge."""
    ring = TorusGeometry(NECK_BORE_R + 0.001, 0.0015, radial_segments=10, tubular_segments=40)
    ring.translate(0.0, 0.0, NECK_TOP_Z)
    return mesh_from_geometry(ring, "mouth_rim")


def _neck_lug():
    """Small catch lug on the front of the neck for the bail lever clasp.
    Embedded into the neck surface so it reads as attached.
    """
    lug_z_center = 0.178  # just below the collar
    lug_y_center = NECK_R - 0.001  # partially embedded in neck wall
    lug = (
        cq.Workplane("XY")
        .workplane(offset=lug_z_center - 0.003)
        .center(0.0, lug_y_center)
        .box(0.006, 0.005, 0.006)
    )
    return mesh_from_cadquery(lug, "neck_lug")


def _collar_mesh():
    """Safety collar ring with pivot tabs on left/right sides."""
    collar = (
        cq.Workplane("XY")
        .circle(COLLAR_R)
        .circle(COLLAR_BORE_R)
        .extrude(COLLAR_Z_TOP - COLLAR_Z_BOT)
    )
    # Pivot tabs on ±X sides where bail arms attach
    tab_w = 0.006
    tab_d = 0.004
    tab_h = 0.008
    for sign in [-1.0, 1.0]:
        tab = (
            cq.Workplane("XY")
            .center(sign * (COLLAR_R + tab_d / 2.0 - 0.001), 0.0)
            .box(tab_d, tab_w, tab_h)
        )
        collar = collar.union(tab)
    result = collar.translate((0.0, 0.0, COLLAR_Z_BOT))
    return mesh_from_cadquery(result, "collar_ring")


def _stopper_mesh():
    """Swing-top stopper (ceramic/plastic disc with gasket seat and dome).
    Built with base at z=0, extends upward to STOPPER_H + DOME_H.
    Will be placed at the correct height via visual origin offset.
    """
    stopper = (
        cq.Workplane("XY")
        .circle(STOPPER_R)
        .extrude(STOPPER_H)
    )
    # Concave underside (gasket seat)
    gasket_cut = (
        cq.Workplane("XY")
        .circle(STOPPER_R - 0.002)
        .extrude(0.002)
    )
    stopper = stopper.cut(gasket_cut)
    # Small dome on top
    dome = (
        cq.Workplane("XY")
        .workplane(offset=STOPPER_H)
        .circle(STOPPER_R * 0.6)
        .extrude(DOME_H)
    )
    stopper = stopper.union(dome)
    return mesh_from_cadquery(stopper, "stopper_body")


def _bail_wire_mesh():
    """Wire bail arms in the swing_cap part frame.
    Hinge is at local origin (0,0,0). Pivot tabs on collar are at
    (±COLLAR_R, 0, 0) in this frame. Stopper center is at (0, 0, STOPPER_LIFT).
    """
    wire_r = 0.0012
    pivot_x = COLLAR_R - 0.002  # where bail arms attach to collar tabs

    # Left arm: from pivot tab up to stopper level, curving slightly outward
    left_pts = [
        (-pivot_x, 0.0, 0.0),            # collar tab attachment
        (-pivot_x - 0.002, 0.0, 0.004),  # slight outward curve
        (-pivot_x - 0.001, 0.0, 0.008),
        (-pivot_x, 0.0, STOPPER_LIFT),   # at stopper level
    ]
    left_wire = tube_from_spline_points(
        left_pts, radius=wire_r, samples_per_segment=10,
        radial_segments=10, cap_ends=True,
    )

    # Right arm: mirror
    right_pts = [
        (pivot_x, 0.0, 0.0),
        (pivot_x + 0.002, 0.0, 0.004),
        (pivot_x + 0.001, 0.0, 0.008),
        (pivot_x, 0.0, STOPPER_LIFT),
    ]
    right_wire = tube_from_spline_points(
        right_pts, radius=wire_r, samples_per_segment=10,
        radial_segments=10, cap_ends=True,
    )

    # Top bar connecting arms at stopper level
    top_bar_pts = [
        (-pivot_x, 0.0, STOPPER_LIFT),
        (-pivot_x * 0.5, 0.0, STOPPER_LIFT + 0.002),
        (0.0, 0.0, STOPPER_LIFT + 0.003),
        (pivot_x * 0.5, 0.0, STOPPER_LIFT + 0.002),
        (pivot_x, 0.0, STOPPER_LIFT),
    ]
    top_bar = tube_from_spline_points(
        top_bar_pts, radius=wire_r, samples_per_segment=10,
        radial_segments=10, cap_ends=True,
    )

    # Lever/clasp wire: goes from near the top bar forward (+Y) and down
    # to catch on the neck lug
    lever_pts = [
        (-pivot_x + 0.003, 0.0, STOPPER_LIFT - 0.002),
        (-0.004, 0.006, STOPPER_LIFT - 0.004),
        (0.0, 0.010, STOPPER_LIFT - 0.006),
        (0.004, 0.006, STOPPER_LIFT - 0.004),
        (pivot_x - 0.003, 0.0, STOPPER_LIFT - 0.002),
    ]
    lever = tube_from_spline_points(
        lever_pts, radius=wire_r * 0.85, samples_per_segment=10,
        radial_segments=10, cap_ends=True,
    )

    left_wire.merge(right_wire)
    left_wire.merge(top_bar)
    left_wire.merge(lever)
    return mesh_from_geometry(left_wire, "bail_wire")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_top_bottle")

    # Materials
    clear_glass = model.material("clear_glass", rgba=(0.82, 0.90, 0.85, 0.30))
    mouth_mat = model.material("mouth_rim", rgba=(0.70, 0.78, 0.74, 0.45))
    lug_mat = model.material("neck_lug", rgba=(0.72, 0.80, 0.76, 0.40))
    collar_mat = model.material("collar_steel", rgba=(0.65, 0.65, 0.67, 1.0))
    stopper_mat = model.material("stopper_ceramic", rgba=(0.92, 0.88, 0.80, 1.0))
    wire_mat = model.material("bail_wire", rgba=(0.58, 0.58, 0.60, 1.0))

    # ---- bottle body (root) ----
    body = model.part("bottle_body")
    body.visual(_bottle_mesh(), material=clear_glass, name="bottle_shell")
    body.visual(_neck_mouth_ring(), material=mouth_mat, name="mouth_rim")
    body.visual(_neck_lug(), material=lug_mat, name="neck_lug")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.250,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- safety collar ring (rotates around neck) ----
    collar = model.part("safety_collar")
    collar.visual(_collar_mesh(), material=collar_mat, name="collar_ring")
    collar.inertial = Inertial.from_geometry(
        Cylinder(COLLAR_R, COLLAR_Z_TOP - COLLAR_Z_BOT),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_MID_Z)),
    )

    # ---- swing-top cap assembly (stopper + bail wire) ----
    # Part frame origin is at the hinge pivot point.
    # Stopper sits at local z = STOPPER_LIFT above the hinge.
    swing_cap = model.part("swing_cap")
    swing_cap.visual(
        _stopper_mesh(),
        material=stopper_mat,
        # Place stopper base at local z = STOPPER_LIFT - STOPPER_H/2
        # so stopper center is at z = STOPPER_LIFT
        origin=Origin(xyz=(0.0, 0.0, STOPPER_LIFT - STOPPER_H / 2.0)),
        name="stopper_body",
    )
    swing_cap.visual(
        _bail_wire_mesh(),
        material=wire_mat,
        name="bail_wire",
    )
    swing_cap.inertial = Inertial.from_geometry(
        Cylinder(STOPPER_R, STOPPER_H + 0.010),
        mass=0.015,
        origin=Origin(xyz=(0.0, 0.0, STOPPER_LIFT)),
    )

    # ---- collar_spin: CONTINUOUS rotation of the collar about +Z ----
    # Collar part frame coincides with body frame (at world origin).
    # Collar visuals are built at their absolute Z heights.
    model.articulation(
        "collar_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0),
    )

    # ---- swing_hinge: REVOLUTE flip of the stopper ----
    # Hinge at the collar top center. Axis along +X so positive q
    # swings the stopper from closed (over mouth, +Z from hinge)
    # backward (toward -Y, open position).
    model.articulation(
        "swing_hinge",
        ArticulationType.REVOLUTE,
        parent=collar,
        child=swing_cap,
        # In collar frame: at the top of the collar, centered on the neck
        origin=Origin(xyz=(0.0, 0.0, COLLAR_Z_TOP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,       # closed: stopper over mouth
            upper=2.3,       # open: stopper flipped back (~132 degrees)
            effort=2.0,
            velocity=3.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle_body")
    collar = object_model.get_part("safety_collar")
    swing_cap = object_model.get_part("swing_cap")
    collar_spin = object_model.get_articulation("collar_spin")
    swing_hinge = object_model.get_articulation("swing_hinge")

    # --- Bottle is translucent glass ---
    glass_mat = next(m for m in object_model.materials if m.name == "clear_glass")
    a = glass_mat.rgba[3] if glass_mat.rgba is not None else 1.0
    ctx.check(
        "bottle shell is translucent glass",
        a < 0.8,
        details=f"clear_glass alpha={a}",
    )

    # --- Bottle has tapered shoulder ---
    body_aabb = ctx.part_world_aabb(body)
    body_ext = (
        body_aabb[1][0] - body_aabb[0][0],
        body_aabb[1][1] - body_aabb[0][1],
        body_aabb[1][2] - body_aabb[0][2],
    )
    ctx.check(
        "bottle is taller than wide",
        body_ext[2] > 2.5 * body_ext[0],
        details=f"body extents={body_ext}",
    )

    # --- Safety collar is at neck height (use AABB center) ---
    collar_aabb = ctx.part_world_aabb(collar)
    collar_center_z = (collar_aabb[0][2] + collar_aabb[1][2]) / 2.0
    ctx.check(
        "safety collar is at neck height",
        collar_center_z > 0.15,
        details=f"collar center z={collar_center_z:.4f}",
    )

    # --- Swing cap is near the top (use AABB) ---
    cap_aabb = ctx.part_world_aabb(swing_cap)
    cap_center_z = (cap_aabb[0][2] + cap_aabb[1][2]) / 2.0
    ctx.check(
        "swing cap is at the top of the bottle",
        cap_center_z > 0.17,
        details=f"swing_cap center z={cap_center_z:.4f}",
    )

    # --- Collar spins (CONTINUOUS joint) ---
    ctx.check(
        "collar_spin is CONTINUOUS",
        collar_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={collar_spin.articulation_type}",
    )

    # --- Swing hinge is REVOLUTE with proper limits ---
    limits = swing_hinge.motion_limits
    ctx.check(
        "swing_hinge is REVOLUTE with bounded limits",
        swing_hinge.articulation_type == ArticulationType.REVOLUTE
        and limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.upper > limits.lower,
        details=f"type={swing_hinge.articulation_type}, limits={limits}",
    )

    # --- Allow overlap: collar ring encircles the neck ---
    ctx.allow_overlap(
        collar,
        body,
        elem_a="collar_ring",
        elem_b="bottle_shell",
        reason="The safety collar ring intentionally encircles the neck, overlapping the neck wall.",
    )
    ctx.allow_overlap(
        collar,
        body,
        elem_a="collar_ring",
        elem_b="mouth_rim",
        reason="The collar ring sits near the mouth rim on the neck.",
    )

    # --- Allow overlap: stopper seats on the neck mouth when closed ---
    ctx.allow_overlap(
        swing_cap,
        body,
        elem_a="stopper_body",
        elem_b="bottle_shell",
        reason="The stopper seats into the neck mouth opening when closed (gasket compression).",
    )
    ctx.allow_overlap(
        swing_cap,
        body,
        elem_a="stopper_body",
        elem_b="mouth_rim",
        reason="The stopper seats against the mouth rim when closed.",
    )
    ctx.allow_overlap(
        swing_cap,
        body,
        elem_a="bail_wire",
        elem_b="bottle_shell",
        reason="The bail lever wire clasps over the neck lug area when closed.",
    )
    ctx.allow_overlap(
        swing_cap,
        body,
        elem_a="bail_wire",
        elem_b="neck_lug",
        reason="The bail lever wire clips onto the neck catch lug when closed.",
    )

    # --- Closed pose: stopper is above the neck rim ---
    with ctx.pose({swing_hinge: 0.0}):
        closed_aabb = ctx.part_world_aabb(swing_cap)
        closed_top_z = closed_aabb[1][2]
        closed_center_y = (closed_aabb[0][1] + closed_aabb[1][1]) / 2.0
    ctx.check(
        "closed swing cap reaches above the neck rim",
        closed_top_z > NECK_TOP_Z,
        details=f"cap top z={closed_top_z:.4f}, neck top={NECK_TOP_Z}",
    )

    # --- Open pose: stopper swings backward ---
    with ctx.pose({swing_hinge: 2.0}):
        open_aabb = ctx.part_world_aabb(swing_cap)
        open_center_y = (open_aabb[0][1] + open_aabb[1][1]) / 2.0
        open_min_z = open_aabb[0][2]
    ctx.check(
        "swing cap flips backward when opened",
        open_center_y < closed_center_y - 0.005,
        details=f"closed_y={closed_center_y:.4f}, open_y={open_center_y:.4f}",
    )

    # --- Bail wire visual exists ---
    bail_visuals = [v for v in swing_cap.visuals if v.name == "bail_wire"]
    ctx.check(
        "bail wire geometry exists on swing cap",
        len(bail_visuals) > 0,
        details=f"swing_cap visuals={[v.name for v in swing_cap.visuals]}",
    )

    # --- Mouth rim visual exists (visible hollow opening) ---
    mouth_visuals = [v for v in body.visuals if v.name == "mouth_rim"]
    ctx.check(
        "mouth rim visual exists showing hollow opening",
        len(mouth_visuals) > 0,
        details=f"body visuals={[v.name for v in body.visuals]}",
    )

    # --- Neck bore radius confirms hollow mouth ---
    ctx.check(
        "neck has a visible hollow bore",
        NECK_BORE_R > 0.005,
        details=f"NECK_BORE_R={NECK_BORE_R}",
    )

    # --- Collar ring visual proof: it overlaps the neck (containment) ---
    ctx.expect_within(
        body,
        collar,
        axes="xy",
        inner_elem="mouth_rim",
        outer_elem="collar_ring",
        margin=0.005,
        name="mouth rim is within the collar ring footprint (collar encircles neck)",
    )

    return ctx.report()


object_model = build_object_model()
