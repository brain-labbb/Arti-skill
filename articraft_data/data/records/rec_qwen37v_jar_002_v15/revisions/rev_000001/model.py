from __future__ import annotations

# Wide apothecary jar with a domed glass stopper.
# Frame: jar stands on +XY ground plane, central axis along +Z (up).
#   - jar_body: round wide-mouth apothecary jar, hollow interior, thick glass
#     walls, a short wide neck/mouth rim at the top. (root)
#   - stopper: domed glass stopper with a short cylindrical plug that inserts
#     into the wide mouth. Modeled via two INDEPENDENT decoupled joints:
#       stopper_rotate (CONTINUOUS, body->carrier): stopper spins about +Z
#       stopper_lift   (PRISMATIC, carrier->stopper): stopper lifts up/out
# The jar is wider than tall (apothecary proportions) with a visible wide
# hollow opening at the mouth.

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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key dimensions (meters) ----
# Apothecary jar: wide round body, slightly bulging
BODY_R = 0.052              # outer body radius
BODY_BOTTOM_Z = 0.0         # base sits on ground
BODY_TOP_Z = 0.078          # top of main body cylinder
BODY_BULGE_Z = 0.042        # z of maximum bulge
BODY_BULGE_R = 0.058        # radius at the widest bulge point
WALL = 0.004                # glass wall thickness

# Foot: a slightly wider base ring
FOOT_R = 0.056
FOOT_HEIGHT = 0.006

# Neck/mouth: wide opening
MOUTH_R_OUTER = 0.038       # outer radius of the mouth rim
MOUTH_R_INNER = 0.032       # inner radius (the actual opening)
MOUTH_HEIGHT = 0.014        # height of the raised rim above body top
MOUTH_BOTTOM_Z = BODY_TOP_Z
MOUTH_TOP_Z = MOUTH_BOTTOM_Z + MOUTH_HEIGHT

# Stopper: domed top + cylindrical plug
PLUG_R = 0.030              # plug radius (fits inside mouth)
PLUG_HEIGHT = 0.018         # plug length that inserts into mouth
DOME_R = 0.036              # dome radius
DOME_BASE_Z = PLUG_HEIGHT   # dome sits on top of plug in local frame
STOPPER_TOTAL_H = PLUG_HEIGHT + DOME_R  # approximate total height

# Stopper mount: plug inserts into mouth, plug bottom at mouth inner bottom
# The stopper local origin is at the plug bottom.
# At rest, plug bottom = MOUTH_BOTTOM_Z - small insertion
STOPPER_MOUNT_Z = MOUTH_BOTTOM_Z - 0.004  # plug inserts 4mm below rim top


def _jar_body_solid() -> cq.Workplane:
    """Hollow wide apothecary jar with bulging profile and wide mouth rim."""
    # Build outer shell as union of lofted/extruded sections.
    # 1) Foot ring
    foot = (
        cq.Workplane("XY")
        .circle(FOOT_R)
        .extrude(FOOT_HEIGHT)
    )
    # 2) Lower body: foot top -> bulge
    lower_body = (
        cq.Workplane("XY")
        .workplane(offset=FOOT_HEIGHT)
        .circle(BODY_R)
        .workplane(offset=BODY_BULGE_Z - FOOT_HEIGHT)
        .circle(BODY_BULGE_R)
        .loft(ruled=False)
    )
    # 3) Upper body: bulge -> body top
    upper_body = (
        cq.Workplane("XY")
        .workplane(offset=BODY_BULGE_Z)
        .circle(BODY_BULGE_R)
        .workplane(offset=BODY_TOP_Z - BODY_BULGE_Z)
        .circle(MOUTH_R_OUTER)
        .loft(ruled=False)
    )
    # 4) Mouth rim: raised ring at top
    mouth_rim = (
        cq.Workplane("XY")
        .workplane(offset=MOUTH_BOTTOM_Z)
        .circle(MOUTH_R_OUTER)
        .extrude(MOUTH_HEIGHT)
    )

    outer = foot.union(lower_body).union(upper_body).union(mouth_rim)

    # Build inner cavity (hollow interior, open at mouth top)
    # Inner cavity starts at WALL above the base floor
    inner_lower = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .circle(BODY_R - WALL)
        .workplane(offset=BODY_BULGE_Z - WALL - 0.001)
        .circle(BODY_BULGE_R - WALL)
        .loft(ruled=False)
    )
    inner_upper = (
        cq.Workplane("XY")
        .workplane(offset=BODY_BULGE_Z - 0.001)
        .circle(BODY_BULGE_R - WALL)
        .workplane(offset=BODY_TOP_Z - BODY_BULGE_Z + 0.001)
        .circle(MOUTH_R_INNER)
        .loft(ruled=False)
    )
    inner_mouth = (
        cq.Workplane("XY")
        .workplane(offset=MOUTH_BOTTOM_Z)
        .circle(MOUTH_R_INNER)
        .extrude(MOUTH_HEIGHT + 0.003)  # extends slightly above rim
    )
    cavity = inner_lower.union(inner_upper).union(inner_mouth)

    jar = outer.cut(cavity)
    return jar


def _jar_body_mesh():
    return mesh_from_cadquery(_jar_body_solid(), "jar_glass")


def _stopper_solid() -> cq.Workplane:
    """Domed glass stopper: cylindrical plug + hemisphere dome on top."""
    # Cylindrical plug (local z=0 at plug bottom, extends up to PLUG_HEIGHT)
    plug = (
        cq.Workplane("XY")
        .circle(PLUG_R)
        .extrude(PLUG_HEIGHT)
    )

    # Dome: hemisphere sitting on top of plug
    dome = (
        cq.Workplane("XY")
        .workplane(offset=PLUG_HEIGHT)
        .circle(DOME_R)
        .extrude(DOME_R)  # full cylinder to cut from
    )
    # Make a proper hemisphere using sphere and cut
    sphere = (
        cq.Workplane("XY")
        .workplane(offset=PLUG_HEIGHT)
        .sphere(DOME_R)
    )
    # Cut sphere in half - remove below the plug top plane
    cut_below = (
        cq.Workplane("XY")
        .workplane(offset=PLUG_HEIGHT - DOME_R - 0.001)
        .circle(DOME_R + 0.01)
        .extrude(DOME_R + 0.001)
    )
    hemisphere = sphere.cut(cut_below)

    # Add a small finial/knob on top of the dome for grip
    finial = (
        cq.Workplane("XY")
        .workplane(offset=PLUG_HEIGHT + DOME_R - 0.002)
        .circle(0.008)
        .extrude(0.010)
    )
    finial_top = (
        cq.Workplane("XY")
        .workplane(offset=PLUG_HEIGHT + DOME_R + 0.006)
        .sphere(0.008)
    )

    stopper = plug.union(hemisphere).union(finial).union(finial_top)

    # Add a small ring ridge on the plug for visual thread/grip detail
    ring = (
        cq.Workplane("XY")
        .workplane(offset=PLUG_HEIGHT * 0.4)
        .circle(PLUG_R + 0.001)
        .circle(PLUG_R - 0.001)
        .extrude(0.003)
    )
    stopper = stopper.union(ring)

    return stopper


def _stopper_mesh():
    return mesh_from_cadquery(_stopper_solid(), "stopper_glass")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_apothecary_jar")

    glass = model.material("clear_glass", rgba=(0.80, 0.88, 0.90, 0.30))
    glass_stopper = model.material("stopper_glass", rgba=(0.75, 0.82, 0.85, 0.35))
    glass_dark = model.material("glass_dark", rgba=(0.60, 0.70, 0.72, 0.50))

    # ---- jar body (root): round wide apothecary jar with hollow interior ----
    body = model.part("jar_body")
    body.visual(_jar_body_mesh(), material=glass, name="jar_glass")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_BULGE_R, MOUTH_TOP_Z),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, MOUTH_TOP_Z / 2.0)),
    )

    # ---- massless carrier (NO visuals): routes the spin joint ----
    carrier = model.part("stopper_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.008, 0.008, 0.008)), mass=1e-4)

    # ---- domed stopper: plug + dome + finial ----
    stopper = model.part("stopper")
    stopper.visual(_stopper_mesh(), material=glass_stopper, name="stopper_glass")
    # Off-axis marker so rotation is observable in tests
    marker = CylinderGeometry(0.003, 0.006).translate(DOME_R - 0.005, 0.0, PLUG_HEIGHT + DOME_R * 0.5)
    stopper.visual(
        mesh_from_geometry(marker, "stopper_marker"),
        material=glass_dark,
        name="stopper_marker",
    )
    stopper.inertial = Inertial.from_geometry(
        Cylinder(DOME_R, STOPPER_TOTAL_H),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, STOPPER_TOTAL_H / 2.0)),
    )

    # ---- two INDEPENDENT decoupled joints sharing +Z, through the carrier ----
    model.articulation(
        "stopper_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, STOPPER_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )
    model.articulation(
        "stopper_lift",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=stopper,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=PLUG_HEIGHT + MOUTH_HEIGHT + 0.01,
            effort=1.0,
            velocity=1.0,
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("jar_body")
    stopper = object_model.get_part("stopper")
    rotate = object_model.get_articulation("stopper_rotate")
    lift = object_model.get_articulation("stopper_lift")

    # The stopper plug is intentionally seated inside the wide mouth (capture fit).
    ctx.allow_overlap(
        stopper,
        body,
        elem_a="stopper_glass",
        elem_b="jar_glass",
        reason="The domed stopper plug is intentionally inserted into the wide mouth opening.",
    )

    # --- jar body is round and wider-than-tall (apothecary proportions) ---
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jar body is round (x ≈ y extent)",
        abs(bext[0] - bext[1]) < 0.008,
        details=f"x={bext[0]:.4f}, y={bext[1]:.4f}",
    )
    ctx.check(
        "jar is wider than tall (apothecary proportions)",
        bext[0] > bext[2] * 0.7,
        details=f"width={bext[0]:.4f}, height={bext[2]:.4f}",
    )

    # --- wide mouth: opening at top of jar ---
    # The mouth rim extends above the main body top
    body_pos = ctx.part_world_position(body)
    ctx.check(
        "jar mouth rim extends above body",
        MOUTH_TOP_Z > BODY_TOP_Z,
        details=f"mouth_top={MOUTH_TOP_Z:.4f}, body_top={BODY_TOP_Z:.4f}",
    )
    # Mouth inner diameter is wide (> 0.05m)
    ctx.check(
        "jar has a wide mouth opening",
        2.0 * MOUTH_R_INNER > 0.050,
        details=f"mouth_inner_diameter={2.0 * MOUTH_R_INNER:.4f} m",
    )

    # --- domed stopper sits on top of the jar ---
    sext = _ext(ctx.part_world_aabb(stopper))
    stopper_pos = ctx.part_world_position(stopper)
    ctx.check(
        "stopper sits at the top of the jar",
        stopper_pos is not None and stopper_pos[2] > BODY_TOP_Z - 0.01,
        details=f"stopper z={stopper_pos[2] if stopper_pos else None}",
    )
    # Stopper is seated in the mouth footprint
    ctx.expect_overlap(
        stopper, body, axes="xy", min_overlap=0.02,
        name="stopper seated in mouth footprint",
    )

    # --- stopper has a dome shape (taller than just a flat disc) ---
    ctx.check(
        "stopper has dome height (not flat)",
        sext[2] > 0.03,
        details=f"stopper height={sext[2]:.4f}",
    )

    # --- stopper_rotate spins the stopper (continuous joint about +Z) ---
    m0 = ctx.part_element_world_aabb(stopper, elem="stopper_marker")
    m0c = ((m0[0][0] + m0[1][0]) / 2.0, (m0[0][1] + m0[1][1]) / 2.0)
    with ctx.pose({rotate: math.pi / 2.0}):
        m1 = ctx.part_element_world_aabb(stopper, elem="stopper_marker")
        m1c = ((m1[0][0] + m1[1][0]) / 2.0, (m1[0][1] + m1[1][1]) / 2.0)
    marker_shift = math.hypot(m1c[0] - m0c[0], m1c[1] - m0c[1])
    ctx.check(
        "stopper_rotate spins the stopper (marker moves)",
        marker_shift > 0.008,
        details=f"marker moved {marker_shift:.4f} m on quarter turn",
    )

    # --- stopper_lift raises the stopper out of the mouth ---
    z_rest = ctx.part_world_position(stopper)[2]
    lift_max = PLUG_HEIGHT + MOUTH_HEIGHT + 0.01
    with ctx.pose({lift: lift_max}):
        z_lift = ctx.part_world_position(stopper)[2]
    ctx.check(
        "stopper_lift raises the stopper out of the mouth",
        z_lift > z_rest + 0.015,
        details=f"rest z={z_rest:.4f}, lifted z={z_lift:.4f}",
    )

    # --- joint types and axes ---
    ctx.check(
        "stopper_rotate is continuous about +Z",
        rotate.axis == (0.0, 0.0, 1.0) and rotate.articulation_type == ArticulationType.CONTINUOUS,
        details=f"axis={rotate.axis}, type={rotate.articulation_type}",
    )
    ctx.check(
        "stopper_lift is prismatic along +Z",
        lift.axis == (0.0, 0.0, 1.0) and lift.articulation_type == ArticulationType.PRISMATIC,
        details=f"axis={lift.axis}, type={lift.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
