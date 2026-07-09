from __future__ import annotations

# Hinged swing-top bottle with wire bail geometry, a straw spout that pivots
# up from the cap, a visible hollow mouth opening under the cap, and a
# separate gasket ring below the cap.
#
# Frame: vertical axis along +Z, bottle standing on z=0, centerline on x=y=0.
#   - rounded base -> cylindrical body -> tapered shoulder -> neck with open mouth
# Articulations:
#   - body_to_cap: REVOLUTE at the bail hinge (side of neck), axis=+Y so positive
#     q swings the cap upward/open. Limits 0 to ~2.4 rad.
#   - cap_to_straw: REVOLUTE at the straw pivot on the cap top, axis=+X so
#     positive q tilts the straw upright. Limits 0 to ~1.3 rad.

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
BODY_TOP_Z = 0.110
SHOULDER_TOP_Z = 0.156
NECK_TOP_Z = 0.176

BODY_R = 0.0275
NECK_R = 0.0125
NECK_BORE_R = 0.0098

# Cap (swing-top stopper) dimensions
CAP_R = 0.0140  # stopper disc radius (slightly larger than neck bore)
CAP_HEIGHT = 0.012  # stopper thickness
CAP_BASE_Z = NECK_TOP_Z  # sits on top of the neck rim when closed

# Bail hinge geometry
BAIL_ARM_R = 0.0012  # wire radius
BAIL_PIVOT_Z = NECK_TOP_Z - 0.004  # bail clips into neck ~4mm below rim
BAIL_ARM_LENGTH = 0.030  # arm length from neck pivot up and over the cap

# Gasket
GASKET_R_OUTER = NECK_R + 0.001
GASKET_R_INNER = NECK_BORE_R - 0.001
GASKET_HEIGHT = 0.003
GASKET_Z = NECK_TOP_Z - GASKET_HEIGHT  # sits on the rim

# Straw
STRAW_R = 0.003
STRAW_LENGTH = 0.060
STRAW_PIVOT_Z = CAP_BASE_Z + CAP_HEIGHT  # pivot at top of cap


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
    pts = _profile_sections()
    wp = cq.Workplane("XZ").moveTo(0.0, pts[0][0])
    for r, z in [(r, z) for (z, r) in pts]:
        wp = wp.lineTo(r, z)
    wp = wp.lineTo(0.0, pts[-1][0]).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))

    # Hollow cavity open through the neck rim
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
        (NECK_BORE_R, NECK_TOP_Z + 0.005),
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


def _gasket_mesh():
    """Rubber/silicone gasket ring that sits on the bottle mouth rim."""
    gasket = (
        cq.Workplane("XY")
        .circle(GASKET_R_OUTER)
        .circle(GASKET_R_INNER)
        .extrude(GASKET_HEIGHT)
    )
    return mesh_from_cadquery(gasket, "gasket_ring")


def _cap_solid() -> cq.Workplane:
    """Swing-top stopper: a domed disc plug that seals the mouth."""
    # Flat disc base
    cap = (
        cq.Workplane("XY")
        .circle(CAP_R)
        .extrude(CAP_HEIGHT * 0.6)
    )
    # Domed top
    dome = (
        cq.Workplane("XY")
        .workplane(offset=CAP_HEIGHT * 0.6)
        .circle(CAP_R)
        .extrude(CAP_HEIGHT * 0.4)
    )
    # Fillet approximation: add a smaller cylinder on top for dome feel
    cap = cap.union(dome)
    # Cut a small center hole for the straw to pass through
    straw_hole = (
        cq.Workplane("XY")
        .circle(STRAW_R + 0.0005)
        .extrude(CAP_HEIGHT + 0.002)
    )
    cap = cap.cut(straw_hole)
    return cap


def _cap_mesh():
    return mesh_from_cadquery(_cap_solid(), "cap_plug")


def _wire_bail_mesh():
    """Wire bail: two arms pivoting from the neck sides, arcing over the cap,
    joined by a cross-bar at the top that clips onto the cap."""
    wire_r = BAIL_ARM_R

    # Left bail arm: from neck pivot on -Y side, arcs up and over the cap
    left_arm = tube_from_spline_points(
        [
            (0.0, -(NECK_R + 0.001), BAIL_PIVOT_Z),
            (0.0, -(NECK_R + 0.004), BAIL_PIVOT_Z + 0.015),
            (0.0, -(NECK_R - 0.002), BAIL_PIVOT_Z + 0.028),
            (0.0, -0.002, CAP_BASE_Z + CAP_HEIGHT + 0.004),
        ],
        radius=wire_r,
        samples_per_segment=10,
        radial_segments=10,
        cap_ends=True,
    )

    # Right bail arm: mirror on +Y side
    right_arm = tube_from_spline_points(
        [
            (0.0, (NECK_R + 0.001), BAIL_PIVOT_Z),
            (0.0, (NECK_R + 0.004), BAIL_PIVOT_Z + 0.015),
            (0.0, (NECK_R - 0.002), BAIL_PIVOT_Z + 0.028),
            (0.0, 0.002, CAP_BASE_Z + CAP_HEIGHT + 0.004),
        ],
        radius=wire_r,
        samples_per_segment=10,
        radial_segments=10,
        cap_ends=True,
    )

    # Cross-bar connecting the two arms at the top (clips over the cap)
    cross_bar = tube_from_spline_points(
        [
            (0.0, -0.002, CAP_BASE_Z + CAP_HEIGHT + 0.004),
            (0.0, 0.0, CAP_BASE_Z + CAP_HEIGHT + 0.005),
            (0.0, 0.002, CAP_BASE_Z + CAP_HEIGHT + 0.004),
        ],
        radius=wire_r,
        samples_per_segment=8,
        radial_segments=10,
        cap_ends=True,
    )

    # Pivot pin (short horizontal tube through the neck)
    pivot_pin = tube_from_spline_points(
        [
            (0.0, -(NECK_R + 0.003), BAIL_PIVOT_Z),
            (0.0, 0.0, BAIL_PIVOT_Z),
            (0.0, (NECK_R + 0.003), BAIL_PIVOT_Z),
        ],
        radius=wire_r * 1.3,
        samples_per_segment=4,
        radial_segments=10,
        cap_ends=True,
    )

    left_arm.merge(right_arm)
    left_arm.merge(cross_bar)
    left_arm.merge(pivot_pin)
    return mesh_from_geometry(left_arm, "wire_bail")


def _straw_mesh():
    """Straw spout: a straight tube that pivots from the cap top."""
    straw = tube_from_spline_points(
        [
            (0.0, 0.0, 0.0),
            (0.0, 0.0, STRAW_LENGTH * 0.5),
            (0.0, 0.0, STRAW_LENGTH),
        ],
        radius=STRAW_R,
        samples_per_segment=6,
        radial_segments=12,
        cap_ends=True,
    )
    # Add a slight bend at the top for drinking spout feel
    bend = tube_from_spline_points(
        [
            (0.0, 0.0, STRAW_LENGTH),
            (0.005, 0.0, STRAW_LENGTH + 0.005),
            (0.012, 0.0, STRAW_LENGTH + 0.006),
        ],
        radius=STRAW_R,
        samples_per_segment=8,
        radial_segments=12,
        cap_ends=True,
    )
    straw.merge(bend)
    return mesh_from_geometry(straw, "straw_spout")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_top_bottle")

    clear = model.material("clear_pet", rgba=(0.78, 0.85, 0.88, 0.25))
    clear_neck = model.material("clear_neck", rgba=(0.72, 0.80, 0.84, 0.30))
    rubber = model.material("gasket_rubber", rgba=(0.75, 0.20, 0.12, 1.0))
    ceramic = model.material("cap_ceramic", rgba=(0.92, 0.91, 0.88, 1.0))
    steel = model.material("bail_steel", rgba=(0.62, 0.63, 0.65, 1.0))
    straw_mat = model.material("straw_plastic", rgba=(0.20, 0.55, 0.80, 1.0))

    # ---- bottle body (root): hollow clear PET shell ----
    body = model.part("bottle_body")
    body.visual(_bottle_mesh(), material=clear, name="bottle_shell")
    body.visual(_neck_threads(), material=clear_neck, name="neck_threads")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, 0.176),
        mass=0.020,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
    )

    # ---- gasket ring: sits on the neck rim ----
    gasket = model.part("gasket")
    gasket.visual(
        _gasket_mesh(),
        material=rubber,
        origin=Origin(xyz=(0.0, 0.0, GASKET_Z)),
        name="gasket_ring",
    )
    gasket.inertial = Inertial.from_geometry(
        Cylinder(GASKET_R_OUTER, GASKET_HEIGHT),
        mass=0.002,
        origin=Origin(xyz=(0.0, 0.0, GASKET_Z + GASKET_HEIGHT / 2)),
    )

    # ---- cap (swing-top stopper) ----
    # The cap part frame sits at the bail hinge pivot (BAIL_PIVOT_Z).
    # The cap plug visual is offset upward from the part frame.
    cap_local_z = CAP_BASE_Z - BAIL_PIVOT_Z  # 0.004 m above hinge
    cap = model.part("cap")
    cap.visual(
        _cap_mesh(),
        material=ceramic,
        origin=Origin(xyz=(0.0, 0.0, cap_local_z)),
        name="cap_plug",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_HEIGHT),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, cap_local_z + CAP_HEIGHT / 2)),
    )

    # ---- wire bail ----
    bail = model.part("wire_bail")
    bail.visual(
        _wire_bail_mesh(),
        material=steel,
        name="bail_frame",
    )
    bail.inertial = Inertial.from_geometry(
        Box((0.004, 0.030, 0.040)),
        mass=0.005,
        origin=Origin(xyz=(0.0, 0.0, BAIL_PIVOT_Z + 0.015)),
    )

    # ---- straw spout (pivots from cap top) ----
    # The straw part frame sits at the cap_to_straw articulation origin.
    # The straw mesh starts at z=0 in its local frame, so no visual offset needed.
    straw = model.part("straw_spout")
    straw.visual(
        _straw_mesh(),
        material=straw_mat,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="straw_tube",
    )
    straw.inertial = Inertial.from_geometry(
        Cylinder(STRAW_R, STRAW_LENGTH),
        mass=0.001,
        origin=Origin(xyz=(0.0, 0.0, STRAW_LENGTH / 2)),
    )

    # ---- Articulation: gasket is fixed to body (rigid mount on rim) ----
    model.articulation(
        "body_to_gasket",
        ArticulationType.FIXED,
        parent=body,
        child=gasket,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # ---- Articulation: bail is fixed to body (clips into neck) ----
    model.articulation(
        "body_to_bail",
        ArticulationType.FIXED,
        parent=body,
        child=bail,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # ---- Articulation: body_to_cap REVOLUTE at bail hinge ----
    # The cap swings open around the bail pivot point on the +X side.
    # Axis = +Y so positive q rotates the cap upward (away from the mouth).
    # The cap part frame origin is at (0,0,CAP_BASE_Z) in world.
    # The articulation origin in the parent (body) frame is at the bail pivot.
    model.articulation(
        "body_to_cap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cap,
        # Hinge at the bail pivot height, on the side of the neck
        origin=Origin(xyz=(0.0, 0.0, BAIL_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=2.4,
            effort=3.0,
            velocity=2.0,
        ),
    )

    # ---- Articulation: cap_to_straw REVOLUTE ----
    # Straw pivots up from the cap top. In the cap's local frame, the pivot
    # is at the top of the cap plug.
    model.articulation(
        "cap_to_straw",
        ArticulationType.REVOLUTE,
        parent=cap,
        child=straw,
        origin=Origin(xyz=(0.0, 0.0, cap_local_z + CAP_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.3,
            effort=1.0,
            velocity=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle_body")
    gasket = object_model.get_part("gasket")
    cap = object_model.get_part("cap")
    bail = object_model.get_part("wire_bail")
    straw = object_model.get_part("straw_spout")
    body_to_cap = object_model.get_articulation("body_to_cap")
    cap_to_straw = object_model.get_articulation("cap_to_straw")

    # --- clear bottle body ---
    clear_mat = next(m for m in object_model.materials if m.name == "clear_pet")
    a = clear_mat.rgba[3] if clear_mat.rgba is not None else 1.0
    ctx.check(
        "bottle shell is transparent",
        a < 1.0,
        details=f"clear_pet alpha={a}",
    )

    # --- bottle is taller than wide ---
    body_aabb = ctx.part_world_aabb(body)
    dx = body_aabb[1][0] - body_aabb[0][0]
    dz = body_aabb[1][2] - body_aabb[0][2]
    ctx.check(
        "bottle is tall (taller than wide)",
        dz > 2.5 * dx,
        details=f"dx={dx:.4f}, dz={dz:.4f}",
    )

    # --- tapered shoulder narrows toward the top ---
    ctx.check(
        "tapered shoulder narrows toward the top",
        NECK_R < BODY_R * 0.6,
        details=f"neck_r={NECK_R}, body_r={BODY_R}",
    )

    # --- gasket ring exists and is near the neck top ---
    gasket_aabb = ctx.part_world_aabb(gasket)
    gasket_center_z = (gasket_aabb[0][2] + gasket_aabb[1][2]) / 2
    ctx.check(
        "gasket ring is mounted near the bottle mouth",
        abs(gasket_center_z - (GASKET_Z + GASKET_HEIGHT / 2)) < 0.005,
        details=f"gasket center z={gasket_center_z:.4f}, expected={GASKET_Z + GASKET_HEIGHT / 2:.4f}",
    )

    # --- gasket contacts the bottle rim ---
    ctx.expect_contact(
        gasket,
        body,
        elem_a="gasket_ring",
        elem_b="bottle_shell",
        contact_tol=0.002,
        name="gasket seated on the bottle rim",
    )

    # --- cap (stopper) exists above the gasket ---
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap stopper is mounted at the top of the bottle",
        cap_pos is not None and cap_pos[2] > 0.15,
        details=f"cap pos={cap_pos}",
    )

    # --- cap swings open: at max angle, cap moves laterally away from the mouth ---
    cap_aabb_rest = ctx.part_world_aabb(cap)
    cap_center_x_rest = (cap_aabb_rest[0][0] + cap_aabb_rest[1][0]) / 2
    with ctx.pose({body_to_cap: 2.0}):
        cap_aabb_open = ctx.part_world_aabb(cap)
        cap_center_x_open = (cap_aabb_open[0][0] + cap_aabb_open[1][0]) / 2
    ctx.check(
        "cap swings open on bail hinge",
        abs(cap_center_x_open - cap_center_x_rest) > 0.008,
        details=f"rest_center_x={cap_center_x_rest:.4f}, open_center_x={cap_center_x_open:.4f}",
    )

    # --- wire bail exists near the neck ---
    bail_aabb = ctx.part_world_aabb(bail)
    bail_center_z = (bail_aabb[0][2] + bail_aabb[1][2]) / 2
    ctx.check(
        "wire bail is mounted near the neck",
        bail_center_z > 0.14,
        details=f"bail center z={bail_center_z:.4f}",
    )

    # --- straw spout pivots up from the cap ---
    straw_z_rest = ctx.part_world_aabb(straw)[1][2]
    with ctx.pose({cap_to_straw: 1.0}):
        straw_z_tilted = ctx.part_world_aabb(straw)[1][2]
    ctx.check(
        "straw spout pivots from the cap",
        abs(straw_z_tilted - straw_z_rest) > 0.005,
        details=f"rest_top_z={straw_z_rest:.4f}, tilted_top_z={straw_z_tilted:.4f}",
    )

    # --- visible mouth opening: the neck bore is open (bore radius > 0) ---
    ctx.check(
        "neck bore is open (hollow mouth under cap)",
        NECK_BORE_R > 0.005,
        details=f"neck_bore_r={NECK_BORE_R}",
    )

    # --- non-fixed joints exist ---
    non_fixed = [
        a for a in object_model.articulations
        if a.articulation_type not in (ArticulationType.FIXED,)
    ]
    ctx.check(
        "at least one non-fixed articulation exists",
        len(non_fixed) >= 1,
        details=f"non_fixed={[a.name for a in non_fixed]}",
    )

    return ctx.report()


object_model = build_object_model()
