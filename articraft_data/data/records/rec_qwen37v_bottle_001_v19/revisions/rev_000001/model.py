from __future__ import annotations

# Hinged swing-top bottle with wire bail, straw spout, gasket, and hollow mouth.
# Frame: bottle axis along +Z, base at z=0, neck/cap at the top (+Z).
# The body is a clear thin-wall PET shell with smooth neck and retaining lip.
# A rubber gasket ring sits on the neck rim. A wire bail clips around the neck
# and provides the swing hinge. A ceramic stopper cap swings open/closed on the
# hinge. A straw spout pivots up from the cap top.

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
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---- key dimensions (meters) ----
BODY_R = 0.030          # outer barrel radius (~60 mm dia)
WALL = 0.0016           # thin PET wall
BASE_Z = 0.0
BARREL_TOP_Z = 0.108
SHOULDER_TOP_Z = 0.132
NECK_R = 0.013          # neck outer wall radius
NECK_LIP_R = 0.0155     # retaining lip for wire bail
NECK_TOP_Z = 0.148      # top rim of neck

GASKET_H = 0.004        # gasket thickness
GASKET_OUTER_R = 0.017  # gasket outer radius
GASKET_INNER_R = 0.010  # gasket inner radius (covers mouth)

CAP_R = 0.016           # stopper disc radius
CAP_H = 0.007           # stopper disc height

STRAW_R = 0.0025        # straw tube radius
STRAW_LENGTH = 0.050    # straw deployed length

WIRE_R = 0.0012         # wire cross-section radius
RING_R = NECK_R + WIRE_R + 0.0005      # wire ring just outside neck wall
RING_Z = NECK_TOP_Z - 0.012            # wire ring below the lip

HINGE_OFFSET_Y = 0.022  # hinge point Y offset (behind neck, in -Y)
HINGE_Z = NECK_TOP_Z + 0.003       # hinge Z (slightly above neck top)

# Cap local frame: origin at hinge point. At q=0 the cap frame coincides with
# the articulation frame, so the disc must be offset in cap-local coords to sit
# centred over the mouth.
CAP_LOCAL_Y = HINGE_OFFSET_Y                       # brings disc to Y=0 in world
CAP_LOCAL_Z0 = NECK_TOP_Z + GASKET_H - HINGE_Z - 0.0015  # embed into gasket
CAP_LOCAL_ZC = CAP_LOCAL_Z0 + CAP_H / 2.0          # disc centre in cap-local Z


def _bottle_shell():
    """Transparent thin-wall bottle as one revolved solid, shelled open at
    the top.  Smooth neck with a small retaining lip for the wire bail."""
    wp = (
        cq.Workplane("XZ")
        .moveTo(0.0, BASE_Z)
        # rounded base corner
        .lineTo(BODY_R - 0.006, BASE_Z)
        .threePointArc((BODY_R, BASE_Z + 0.006), (BODY_R, BASE_Z + 0.012))
        # straight cylindrical barrel
        .lineTo(BODY_R, BARREL_TOP_Z)
        # shoulder taper
        .threePointArc(
            ((BODY_R + NECK_R) / 2.0,
             (BARREL_TOP_Z + SHOULDER_TOP_Z) / 2.0 + 0.004),
            (NECK_R, SHOULDER_TOP_Z),
        )
        # smooth neck up to lip
        .lineTo(NECK_R, NECK_TOP_Z - 0.008)
        # retaining lip (smooth bulge for wire bail to clip under)
        .threePointArc(
            (NECK_LIP_R, NECK_TOP_Z - 0.004),
            (NECK_R, NECK_TOP_Z),
        )
        # close along axis (top face will be removed by shell)
        .lineTo(0.0, NECK_TOP_Z)
        .close()
    )
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    # Hollow: remove the top neck face and shell inward → visible mouth opening
    return outer.faces(">Z").shell(-WALL)


def _gasket_solid():
    """Rubber gasket as a flat annular ring seated on the neck rim.
    Local frame = bottle frame (gasket is FIXED child at origin).
    The ring extends slightly below the neck top for a compression seat."""
    gasket_inner = NECK_R - WALL - 0.0005   # slightly inside neck inner wall
    gasket_outer = NECK_R + 0.003            # slightly outside neck outer wall
    gasket_z0 = NECK_TOP_Z - 0.001           # embed 1 mm below neck rim
    gasket = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, gasket_z0))
        .circle(gasket_outer)
        .circle(gasket_inner)
        .extrude(GASKET_H)
    )
    # Round outer and inner edges for a rubber look
    gasket = gasket.edges().fillet(0.0006)
    return gasket


def _cap_stopper():
    """Ceramic-style stopper disc + connecting bracket from hinge point.
    All coordinates in cap-local frame (origin at the hinge point)."""
    # Disc centred over the mouth
    disc = (
        cq.Workplane("XY")
        .transformed(offset=(0, CAP_LOCAL_Y, CAP_LOCAL_Z0))
        .circle(CAP_R)
        .extrude(CAP_H)
    )
    disc = disc.edges(">Z").fillet(0.0018)

    # Connecting bracket: two thin fork arms from hinge point to disc, leaving
    # a gap for the wire bail arms to pass between.
    arm_len_y = CAP_LOCAL_Y * 0.85
    arm_h = CAP_LOCAL_Z0 + CAP_H * 0.6
    arm_thickness = 0.0018
    arm_spacing = 0.005

    arm_l = (
        cq.Workplane("XY")
        .transformed(offset=(-arm_spacing, arm_len_y / 2, arm_h / 2))
        .box(arm_thickness, arm_len_y, arm_h)
    )
    arm_r = (
        cq.Workplane("XY")
        .transformed(offset=(arm_spacing, arm_len_y / 2, arm_h / 2))
        .box(arm_thickness, arm_len_y, arm_h)
    )
    # Cross-bar at the hinge end (gives the fork a solid base)
    crossbar = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0.002, arm_h / 2))
        .box(arm_spacing * 2 + arm_thickness, 0.004, arm_h)
    )
    cap = disc.union(arm_l).union(arm_r).union(crossbar)
    return cap


def _wire_bail_ring_mesh():
    """Wire ring that clips around the neck.  Returns MeshGeometry in bottle
    frame."""
    n = 36
    pts = [
        (RING_R * math.cos(2 * math.pi * i / n),
         RING_R * math.sin(2 * math.pi * i / n),
         RING_Z)
        for i in range(n)
    ]
    return tube_from_spline_points(
        pts,
        radius=WIRE_R,
        closed_spline=True,
        samples_per_segment=6,
        radial_segments=12,
        cap_ends=True,
    )


def _wire_bail_arms_mesh():
    """V-shaped bail arms from ring to hinge point.  Returns MeshGeometry in
    bottle frame.  Midpoints pushed well outside the neck and gasket to avoid
    interpenetration."""
    pts = [
        (-RING_R, 0.0, RING_Z),
        (-NECK_LIP_R * 1.6, -HINGE_OFFSET_Y * 0.55, RING_Z + 0.008),
        (0.0, -HINGE_OFFSET_Y, HINGE_Z),
        (NECK_LIP_R * 1.6, -HINGE_OFFSET_Y * 0.55, RING_Z + 0.008),
        (RING_R, 0.0, RING_Z),
    ]
    return tube_from_spline_points(
        pts,
        radius=WIRE_R,
        samples_per_segment=14,
        radial_segments=12,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_top_bottle")

    # ---- materials ----
    clear = model.material("clear_pet", rgba=(0.80, 0.86, 0.84, 0.25))
    ceramic = model.material("ceramic_white", rgba=(0.92, 0.90, 0.87, 1.0))
    rubber = model.material("rubber_gasket", rgba=(0.50, 0.12, 0.08, 1.0))
    steel = model.material("steel_wire", rgba=(0.55, 0.55, 0.58, 1.0))
    straw_mat = model.material("straw_plastic", rgba=(0.15, 0.55, 0.75, 0.70))

    # ---- bottle body (root) ----
    body = model.part("bottle")
    shell = _bottle_shell()
    body.visual(
        mesh_from_cadquery(shell, "bottle_shell"),
        material=clear,
        name="bottle_shell",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.030,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- gasket ring (FIXED child of bottle) ----
    gasket = model.part("gasket")
    gasket_geo = _gasket_solid()
    gasket.visual(
        mesh_from_cadquery(gasket_geo, "gasket_ring"),
        material=rubber,
        name="gasket_ring",
    )
    gasket.inertial = Inertial.from_geometry(
        Cylinder(GASKET_OUTER_R, GASKET_H),
        mass=0.003,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z + GASKET_H / 2.0)),
    )

    # ---- wire bail (FIXED child of bottle) ----
    wire_bail = model.part("wire_bail")
    wire_bail.visual(
        mesh_from_geometry(_wire_bail_ring_mesh(), "bail_ring"),
        material=steel,
        name="bail_ring",
    )
    wire_bail.visual(
        mesh_from_geometry(_wire_bail_arms_mesh(), "bail_arms"),
        material=steel,
        name="bail_arms",
    )

    # ---- swing-top cap / stopper ----
    cap = model.part("cap")
    cap_geo = _cap_stopper()
    cap.visual(
        mesh_from_cadquery(cap_geo, "cap_stopper"),
        material=ceramic,
        name="cap_stopper",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_H),
        mass=0.010,
        origin=Origin(xyz=(0.0, CAP_LOCAL_Y, CAP_LOCAL_ZC)),
    )

    # ---- straw spout ----
    straw = model.part("straw_spout")
    # Straw tube extends in +Y from origin (folded flat at q=0)
    straw.visual(
        Cylinder(STRAW_R, STRAW_LENGTH),
        origin=Origin(xyz=(0.0, STRAW_LENGTH / 2.0, 0.0),
                      rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=straw_mat,
        name="straw_tube",
    )
    # Small spout base at the pivot point
    straw.visual(
        Cylinder(0.005, 0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=straw_mat,
        name="straw_base",
    )

    # ---- articulations ----

    # Gasket: FIXED mount on neck rim
    model.articulation(
        "gasket_mount",
        ArticulationType.FIXED,
        parent=body,
        child=gasket,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # Wire bail: FIXED mount clipped around neck
    model.articulation(
        "bail_mount",
        ArticulationType.FIXED,
        parent=body,
        child=wire_bail,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # Swing hinge: REVOLUTE.  Hinge at back of neck, axis along +X so that
    # positive q swings the cap upward and backward (opening the bottle).
    model.articulation(
        "swing_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, -HINGE_OFFSET_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,     # closed: cap seated on gasket
            upper=2.2,     # open: cap swung back ~126°
        ),
    )

    # Straw pivot: REVOLUTE.  At top of cap disc, axis along +X so positive q
    # rotates the straw from folded-flat (+Y) toward deployed (+Z).
    model.articulation(
        "straw_pivot",
        ArticulationType.REVOLUTE,
        parent=cap,
        child=straw,
        origin=Origin(xyz=(0.0, CAP_LOCAL_Y, CAP_LOCAL_Z0 + CAP_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,     # folded flat along cap top
            upper=1.4,     # deployed ~80° upward
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle")
    gasket = object_model.get_part("gasket")
    wire_bail = object_model.get_part("wire_bail")
    cap = object_model.get_part("cap")
    straw = object_model.get_part("straw_spout")

    swing = object_model.get_articulation("swing_hinge")
    straw_pivot = object_model.get_articulation("straw_pivot")

    bottle_shell = body.get_visual("bottle_shell")
    gasket_vis = gasket.get_visual("gasket_ring")
    cap_vis = cap.get_visual("cap_stopper")
    straw_vis = straw.get_visual("straw_tube")

    # --- bottle material is tinted-transparent (alpha < 1) ---
    ctx.check(
        "bottle is clear plastic (alpha < 1)",
        bottle_shell.material.rgba is not None
        and bottle_shell.material.rgba[3] < 1.0,
        details=f"bottle rgba={bottle_shell.material.rgba}",
    )

    # --- gasket is opaque rubber-red ---
    ctx.check(
        "gasket is opaque rubber material",
        gasket_vis.material.rgba is not None
        and gasket_vis.material.rgba[3] >= 0.99,
        details=f"gasket rgba={gasket_vis.material.rgba}",
    )

    # --- cap is opaque ceramic white ---
    ctx.check(
        "cap is opaque ceramic white",
        cap_vis.material.rgba is not None
        and cap_vis.material.rgba[3] >= 0.99
        and min(cap_vis.material.rgba[:3]) > 0.7,
        details=f"cap rgba={cap_vis.material.rgba}",
    )

    # --- gasket sits on the neck (AABB z range near NECK_TOP_Z) ---
    gasket_aabb = ctx.part_world_aabb(gasket)
    ctx.check(
        "gasket positioned on the neck rim",
        gasket_aabb is not None
        and gasket_aabb[0][2] > SHOULDER_TOP_Z
        and gasket_aabb[0][2] < NECK_TOP_Z + 0.005
        and gasket_aabb[1][2] < NECK_TOP_Z + GASKET_H + 0.005,
        details=f"gasket aabb z=({gasket_aabb[0][2]:.4f}, {gasket_aabb[1][2]:.4f})" if gasket_aabb else "None",
    )

    # --- wire bail exists and has bail geometry ---
    bail_ring = wire_bail.get_visual("bail_ring")
    bail_arms = wire_bail.get_visual("bail_arms")
    ctx.check(
        "wire bail has ring and arms visuals",
        bail_ring is not None and bail_arms is not None,
        details="missing bail ring or arms",
    )

    # --- swing hinge is REVOLUTE (non-fixed joint) ---
    ctx.check(
        "swing hinge is revolute",
        swing.articulation_type == ArticulationType.REVOLUTE,
        details=f"swing type={swing.articulation_type}",
    )

    # --- straw pivot is REVOLUTE (non-fixed joint) ---
    ctx.check(
        "straw pivot is revolute",
        straw_pivot.articulation_type == ArticulationType.REVOLUTE,
        details=f"straw pivot type={straw_pivot.articulation_type}",
    )

    # --- swing hinge opens cap upward (use AABB since part origin is fixed
    # at the articulation origin for revolute joints) ---
    with ctx.pose({swing: 0.0}):
        cap_rest_aabb = ctx.part_world_aabb(cap)
    with ctx.pose({swing: 1.5}):
        cap_open_aabb = ctx.part_world_aabb(cap)
    ctx.check(
        "swing hinge raises cap when opened",
        cap_rest_aabb is not None
        and cap_open_aabb is not None
        and cap_open_aabb[1][2] > cap_rest_aabb[1][2] + 0.010,
        details=f"rest max_z={cap_rest_aabb[1][2]:.4f}, open max_z={cap_open_aabb[1][2]:.4f}",
    )

    # --- at closed pose, cap sits near gasket level ---
    with ctx.pose({swing: 0.0}):
        cap_closed = ctx.part_world_aabb(cap)
    ctx.check(
        "cap at rest sits near the neck top",
        cap_closed is not None
        and cap_closed[0][2] < NECK_TOP_Z + GASKET_H + 0.010
        and cap_closed[1][2] > NECK_TOP_Z - 0.005,
        details=f"cap closed aabb z=({cap_closed[0][2]:.4f}, {cap_closed[1][2]:.4f})",
    )

    # --- straw pivot deploys straw upward ---
    with ctx.pose({straw_pivot: 0.0}):
        straw_folded = ctx.part_world_aabb(straw)
    with ctx.pose({straw_pivot: 1.2}):
        straw_deployed = ctx.part_world_aabb(straw)
    ctx.check(
        "straw pivot raises straw when deployed",
        straw_folded is not None
        and straw_deployed is not None
        and straw_deployed[1][2] > straw_folded[1][2] + 0.015,
        details=f"folded max_z={straw_folded[1][2]:.4f}, deployed max_z={straw_deployed[1][2]:.4f}",
    )

    # --- straw exists with tube visual ---
    ctx.check(
        "straw spout has tube visual",
        straw_vis is not None,
        details="no straw_tube visual found",
    )

    # --- hollow mouth: neck is shelled open at top (bottle inner is hollow) ---
    # The bottle shell AABB should show the bottle extends to NECK_TOP_Z
    bottle_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "bottle neck extends to mouth height",
        bottle_aabb is not None
        and bottle_aabb[1][2] > NECK_TOP_Z - 0.005,
        details=f"bottle max_z={bottle_aabb[1][2]:.4f}",
    )

    # ---- intentional overlap allowances (seated / clipped parts) ----

    # Wire bail is a clip-on part that sits just outside the neck without
    # penetrating it.  Allow it to remain isolated from the grounded body.
    ctx.allow_isolated_part(
        wire_bail,
        reason="Wire bail clips around the bottle neck as a separate spring-steel assembly.",
    )

    # Gasket is seated on the neck rim with slight compression embed.
    ctx.allow_overlap(
        gasket, body,
        elem_a="gasket_ring",
        elem_b="bottle_shell",
        reason="Gasket ring is intentionally seated with slight embed on the neck rim for compression seal.",
    )
    ctx.expect_contact(
        gasket, body,
        elem_a="gasket_ring",
        elem_b="bottle_shell",
        name="gasket contacts the neck rim",
    )

    # Cap seats on the gasket with slight embed.
    ctx.allow_overlap(
        cap, gasket,
        elem_a="cap_stopper",
        elem_b="gasket_ring",
        reason="Cap stopper seats on the gasket ring with slight compression embed.",
    )
    ctx.expect_contact(
        cap, gasket,
        elem_a="cap_stopper",
        elem_b="gasket_ring",
        name="cap contacts the gasket when closed",
    )

    return ctx.report()


object_model = build_object_model()
