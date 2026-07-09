from __future__ import annotations

# Wide apothecary jar with a domed stopper and rotating shaker insert.
# Frame: jar stands on +XY ground plane, central axis along +Z (up).
#   - jar_body: wide round glass body with hollow interior, short rim/neck at
#     top forming a wide mouth opening. Gasket ring on the rim. (root)
#   - lid: domed glass stopper with an inner skirt that seats into the mouth.
#     PRISMATIC lid_lift joint lifts it straight up off the jar.
#   - shaker_insert: perforated disc inside the lid cavity, CONTINUOUS rotation.
# Two non-fixed joints: lid_lift (PRISMATIC +Z) and shaker_rotate (CONTINUOUS +Z).

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    DomeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key dimensions (meters) ----
# Jar body: wide round cylinder
BODY_R = 0.050           # body outer radius (100mm diameter)
BODY_H = 0.078           # body cylinder height
WALL = 0.004             # glass wall thickness
BOTTOM_THICK = 0.006     # thicker base

# Rim / neck: short raised ring at top forming the wide mouth
RIM_R_OUTER = 0.044      # outer radius of the rim
RIM_R_INNER = 0.037      # inner radius = mouth opening (74mm wide mouth)
RIM_H = 0.013            # rim height above body top
BODY_TOP = BODY_H        # z where body ends and rim begins
RIM_TOP = BODY_TOP + RIM_H  # z at top of rim = 0.091

# Gasket ring: sits on the rim top
GASKET_MAIN_R = (RIM_R_OUTER + RIM_R_INNER) / 2.0  # center of the ring
GASKET_TUBE_R = (RIM_R_OUTER - RIM_R_INNER) / 2.0 - 0.0005  # tube radius
GASKET_Z = RIM_TOP       # sits on top of rim

# Lid / stopper
LID_SKIRT_R = RIM_R_INNER - 0.001   # skirt fits inside the mouth
LID_SKIRT_H = 0.010                  # skirt drops into the mouth
LID_BASE_R = RIM_R_OUTER + 0.001    # base disc covers the rim
LID_BASE_H = 0.005                   # thin disc above the skirt
LID_DOME_R = RIM_R_OUTER - 0.002    # dome radius (slightly less than rim)
# In lid local frame: origin at skirt bottom (z=0)
#   skirt: z=0 to LID_SKIRT_H
#   base disc: z=LID_SKIRT_H to LID_SKIRT_H+LID_BASE_H
#   dome: base at z=LID_SKIRT_H+LID_BASE_H, extends upward

# Shaker insert: perforated disc inside the lid cavity
SHAKER_R = LID_SKIRT_R - 0.003      # fits inside the skirt bore
SHAKER_THICK = 0.002
SHAKER_Z_IN_LID = LID_SKIRT_H + 0.001  # just above skirt bottom inside
SHAKER_HOLE_R = 0.003               # hole radius
SHAKER_HOLE_N = 8                    # number of holes in outer ring
SHAKER_HOLE_RING_R = SHAKER_R * 0.6  # radius of the hole ring pattern

# Joint: lid_lift is PRISMATIC, lifts lid up along +Z
# At q=0 lid is seated: skirt bottom is at BODY_TOP (just above body, inside rim)
LID_SEAT_Z = BODY_TOP  # skirt bottom sits at body top level (inside rim)
LID_LIFT_MAX = 0.060   # max lift height


def _jar_body_solid() -> cq.Workplane:
    """Wide round glass jar: cylindrical body + rim with wide open mouth."""
    # Main body cylinder
    body = (
        cq.Workplane("XY")
        .circle(BODY_R)
        .extrude(BODY_H)
    )

    # Slight fillet on bottom edge for a finished base
    try:
        body = body.faces("<Z").edges().fillet(0.003)
    except Exception:
        pass

    # Rim / neck: a short ring on top of the body
    rim = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP)
        .circle(RIM_R_OUTER)
        .extrude(RIM_H)
    )

    # Smooth shoulder transition: body top circle to rim base
    # Since rim is narrower than body, add a tapered ring
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP - 0.001)
        .circle(BODY_R)
        .workplane(offset=0.001 + RIM_H * 0.3)
        .circle(RIM_R_OUTER + 0.001)
        .loft(ruled=False)
    )

    solid = body.union(shoulder).union(rim)

    # Hollow cavity: open at the top (wide mouth)
    # Inner body cavity
    inner_body = (
        cq.Workplane("XY")
        .workplane(offset=BOTTOM_THICK)
        .circle(BODY_R - WALL)
        .extrude(BODY_H - BOTTOM_THICK)
    )

    # Inner rim bore: cut through the rim to form the mouth opening
    inner_rim = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP - 0.001)
        .circle(RIM_R_INNER)
        .extrude(RIM_H + 0.002)  # cut through rim top
    )

    # Shoulder transition for inner cavity
    inner_shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP - 0.002)
        .circle(BODY_R - WALL)
        .workplane(offset=0.003)
        .circle(RIM_R_INNER)
        .loft(ruled=False)
    )

    cavity = inner_body.union(inner_shoulder).union(inner_rim)
    return solid.cut(cavity)


def _jar_body_mesh():
    return mesh_from_cadquery(_jar_body_solid(), "jar_glass")


def _gasket_mesh():
    """Rubber gasket ring sitting on the jar rim."""
    gasket = TorusGeometry(GASKET_MAIN_R, GASKET_TUBE_R, radial_segments=12, tubular_segments=32)
    gasket.translate(0.0, 0.0, GASKET_Z)
    return mesh_from_geometry(gasket, "gasket_ring")


def _lid_cup_solid() -> cq.Workplane:
    """Hollow cup: skirt + base disc, open at the bottom."""
    # Skirt: drops into the mouth
    skirt = (
        cq.Workplane("XY")
        .circle(LID_SKIRT_R)
        .extrude(LID_SKIRT_H)
    )

    # Base disc on top of the skirt
    base_z = LID_SKIRT_H
    base = (
        cq.Workplane("XY")
        .workplane(offset=base_z)
        .circle(LID_BASE_R)
        .extrude(LID_BASE_H)
    )

    cup = skirt.union(base)

    # Hollow out from BELOW: bore extends from below bottom to inside the base.
    # This creates a clean open-bottom cup without internal disconnected shells.
    bore = (
        cq.Workplane("XY")
        .workplane(offset=-0.002)
        .circle(LID_SKIRT_R - WALL)
        .extrude(LID_SKIRT_H + LID_BASE_H - 0.001)
    )
    cup = cup.cut(bore)
    return cup


def _lid_mesh():
    """Stopper: hollow cup (skirt+base) + dome as separate visuals."""
    return mesh_from_cadquery(_lid_cup_solid(), "stopper_cup")


def _dome_mesh():
    """Half-sphere dome on top of the stopper base."""
    dome_base_z = LID_SKIRT_H + LID_BASE_H
    dome = DomeGeometry(LID_DOME_R, radial_segments=24, height_segments=12, closed=True)
    dome.translate(0.0, 0.0, dome_base_z)
    return mesh_from_geometry(dome, "stopper_dome")


def _shaker_mesh():
    """Perforated disc: shaker insert with holes."""
    # Start with a solid disc
    disc = (
        cq.Workplane("XY")
        .circle(SHAKER_R)
        .extrude(SHAKER_THICK)
    )

    # Cut center hole
    center_hole = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .circle(SHAKER_HOLE_R * 0.8)
        .extrude(SHAKER_THICK + 0.002)
    )
    disc = disc.cut(center_hole)

    # Cut ring of holes
    for k in range(SHAKER_HOLE_N):
        ang = 2.0 * math.pi * k / SHAKER_HOLE_N
        hx = SHAKER_HOLE_RING_R * math.cos(ang)
        hy = SHAKER_HOLE_RING_R * math.sin(ang)
        hole = (
            cq.Workplane("XY")
            .workplane(offset=-0.001)
            .center(hx, hy)
            .circle(SHAKER_HOLE_R)
            .extrude(SHAKER_THICK + 0.002)
        )
        disc = disc.cut(hole)

    # Add an off-axis marker nub for rotation observability in tests
    marker_x = SHAKER_R - 0.004
    marker = (
        cq.Workplane("XY")
        .workplane(offset=LID_SKIRT_H + SHAKER_Z_IN_LID)  # not needed, will place via origin
        .center(marker_x, 0.0)
        .circle(0.002)
        .extrude(SHAKER_THICK)
    )
    # Actually, let me just make the marker as a separate visual on the part
    # Return just the disc
    return mesh_from_cadquery(disc, "shaker_disc")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_apothecary_jar")

    # Materials
    glass = model.material("clear_glass", rgba=(0.78, 0.84, 0.86, 0.30))
    glass_stop = model.material("stopper_glass", rgba=(0.85, 0.88, 0.90, 0.35))
    rubber = model.material("rubber_gasket", rgba=(0.25, 0.22, 0.20, 1.0))
    metal = model.material("shaker_metal", rgba=(0.70, 0.70, 0.68, 1.0))
    marker_mat = model.material("marker_red", rgba=(0.85, 0.20, 0.15, 1.0))

    # ---- jar body (root): wide round glass jar + gasket on rim ----
    body = model.part("jar_body")
    body.visual(_jar_body_mesh(), material=glass, name="jar_glass")
    body.visual(_gasket_mesh(), material=rubber, name="gasket_ring")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, RIM_TOP),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP / 2.0)),
    )

    # ---- lid / domed stopper ----
    lid = model.part("stopper")
    lid.visual(_lid_mesh(), material=glass_stop, name="stopper_cup")
    lid.visual(_dome_mesh(), material=glass_stop, name="stopper_dome")
    lid.inertial = Inertial.from_geometry(
        Cylinder(LID_BASE_R, LID_SKIRT_H + LID_BASE_H + LID_DOME_R),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, (LID_SKIRT_H + LID_BASE_H + LID_DOME_R) / 2.0)),
    )

    # ---- shaker insert: perforated disc inside the lid ----
    shaker = model.part("shaker_insert")
    shaker.visual(_shaker_mesh(), material=metal, name="shaker_disc")
    # Off-axis marker for rotation observability
    marker_geom = CylinderGeometry(0.002, SHAKER_THICK).translate(
        SHAKER_R - 0.004, 0.0, SHAKER_THICK / 2.0
    )
    shaker.visual(
        mesh_from_geometry(marker_geom, "shaker_marker"),
        material=marker_mat,
        name="shaker_marker",
    )
    shaker.inertial = Inertial.from_geometry(
        Cylinder(SHAKER_R, SHAKER_THICK),
        mass=0.01,
        origin=Origin(xyz=(0.0, 0.0, SHAKER_THICK / 2.0)),
    )

    # ---- Articulation 1: lid_lift (PRISMATIC, jar_body -> stopper) ----
    # Lifts the stopper straight up off the jar along +Z.
    # At q=0 the lid origin is at LID_SEAT_Z (skirt bottom inside the rim).
    model.articulation(
        "lid_lift",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, LID_SEAT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=LID_LIFT_MAX,
            effort=2.0,
            velocity=0.5,
        ),
    )

    # ---- Articulation 2: shaker_rotate (CONTINUOUS, stopper -> shaker_insert) ----
    # The shaker disc rotates inside the lid cavity about +Z.
    # In lid local frame, the shaker sits at z = SHAKER_Z_IN_LID.
    model.articulation(
        "shaker_rotate",
        ArticulationType.CONTINUOUS,
        parent=lid,
        child=shaker,
        origin=Origin(xyz=(0.0, 0.0, SHAKER_Z_IN_LID)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=2.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("jar_body")
    lid = object_model.get_part("stopper")
    shaker = object_model.get_part("shaker_insert")
    lift = object_model.get_articulation("lid_lift")
    rotate = object_model.get_articulation("shaker_rotate")

    # Allow the lid skirt to overlap the jar rim (seated fit inside mouth)
    ctx.allow_overlap(
        lid,
        body,
        elem_a="stopper_cup",
        elem_b="jar_glass",
        reason="The stopper skirt is intentionally seated inside the jar mouth rim.",
    )

    # Allow gasket to overlap jar body (it sits on the rim surface)
    ctx.allow_overlap(
        body,
        body,
        elem_a="gasket_ring",
        elem_b="jar_glass",
        reason="The gasket ring sits on the jar rim surface as a seated seal.",
    )

    # Allow gasket to overlap stopper (gasket is compressed between rim and stopper base)
    ctx.allow_overlap(
        body,
        lid,
        elem_a="gasket_ring",
        elem_b="stopper_cup",
        reason="The gasket ring is compressed between the jar rim and the stopper base disc.",
    )

    # Allow shaker insert to overlap the lid cavity (it sits inside)
    ctx.allow_overlap(
        shaker,
        lid,
        elem_a="shaker_disc",
        elem_b="stopper_cup",
        reason="The shaker insert sits inside the hollow stopper cavity.",
    )

    # --- Jar body is round and wide (diameter >= height) ---
    bext = _ext(ctx.part_world_aabb(body))
    body_diam = (bext[0] + bext[1]) / 2.0
    ctx.check(
        "jar body is round in cross-section",
        abs(bext[0] - bext[1]) < 0.008,
        details=f"x={bext[0]:.4f}, y={bext[1]:.4f}",
    )
    ctx.check(
        "jar is wide (diameter comparable to height)",
        body_diam > 0.08,
        details=f"avg diameter={body_diam:.4f}",
    )

    # --- Wide mouth opening: the rim inner diameter is large ---
    ctx.check(
        "wide mouth: rim inner diameter > 60mm",
        2.0 * RIM_R_INNER > 0.060,
        details=f"mouth ID={2*RIM_R_INNER:.4f}m",
    )

    # --- Gasket ring exists and is on the rim ---
    gasket_aabb = ctx.part_element_world_aabb(body, elem="gasket_ring")
    ctx.check(
        "gasket ring exists on the jar",
        gasket_aabb is not None,
        details="gasket_ring visual not found",
    )
    if gasket_aabb is not None:
        gasket_z_min = gasket_aabb[0][2]
        ctx.check(
            "gasket ring sits at or above the rim top",
            gasket_z_min > BODY_TOP - 0.005,
            details=f"gasket z_min={gasket_z_min:.4f}, body_top={BODY_TOP:.4f}",
        )

    # --- Stopper is seated on the jar at rest ---
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "stopper seated at jar top",
        lid_pos is not None and lid_pos[2] >= BODY_TOP - 0.005,
        details=f"lid z={lid_pos[2] if lid_pos else None}",
    )
    ctx.expect_overlap(
        lid, body, axes="xy", min_overlap=0.02,
        name="stopper overlaps jar rim footprint when seated",
    )

    # --- Stopper has a dome shape (taller than just a flat disc) ---
    lext = _ext(ctx.part_world_aabb(lid))
    dome_visual = lid.get_visual("stopper_dome")
    ctx.check(
        "stopper has dome visual and height",
        dome_visual is not None and lext[2] > LID_SKIRT_H + LID_BASE_H + 0.005,
        details=f"lid height={lext[2]:.4f}",
    )

    # --- lid_lift raises the stopper ---
    z_rest = ctx.part_world_position(lid)[2]
    with ctx.pose({lift: LID_LIFT_MAX}):
        z_lift = ctx.part_world_position(lid)[2]
    ctx.check(
        "lid_lift raises the stopper off the jar",
        z_lift > z_rest + 0.03,
        details=f"rest z={z_rest:.4f}, lifted z={z_lift:.4f}",
    )

    # --- lid_lift is prismatic along +Z ---
    ctx.check(
        "lid_lift is prismatic about +Z",
        lift.axis == (0.0, 0.0, 1.0) and lift.articulation_type == ArticulationType.PRISMATIC,
        details=f"axis={lift.axis}, type={lift.articulation_type}",
    )

    # --- shaker_rotate spins the shaker (marker moves) ---
    m0 = ctx.part_element_world_aabb(shaker, elem="shaker_marker")
    m0c = ((m0[0][0] + m0[1][0]) / 2.0, (m0[0][1] + m0[1][1]) / 2.0)
    with ctx.pose({rotate: math.pi / 2.0}):
        m1 = ctx.part_element_world_aabb(shaker, elem="shaker_marker")
        m1c = ((m1[0][0] + m1[1][0]) / 2.0, (m1[0][1] + m1[1][1]) / 2.0)
    marker_shift = math.hypot(m1c[0] - m0c[0], m1c[1] - m0c[1])
    ctx.check(
        "shaker_rotate spins the insert (marker moves)",
        marker_shift > 0.005,
        details=f"marker shifted {marker_shift:.4f} m on quarter turn",
    )

    # --- shaker_rotate is continuous about +Z ---
    ctx.check(
        "shaker_rotate is continuous about +Z",
        rotate.axis == (0.0, 0.0, 1.0) and rotate.articulation_type == ArticulationType.CONTINUOUS,
        details=f"axis={rotate.axis}, type={rotate.articulation_type}",
    )

    # --- Shaker insert is inside the lid footprint ---
    ctx.expect_within(
        shaker, lid, axes="xy",
        inner_elem="shaker_disc", outer_elem="stopper_cup",
        margin=0.005,
        name="shaker insert fits within stopper footprint",
    )

    # --- Proof checks for gasket seating ---
    # Gasket sits on the rim: its bottom is near the rim top (small embed from torus shape)
    ctx.expect_gap(
        body, body,
        axis="z",
        positive_elem="gasket_ring",
        negative_elem="jar_glass",
        max_penetration=0.004,
        name="gasket ring seated on jar rim",
    )

    # Stopper and gasket overlap in Z (the stopper base disc compresses the gasket)
    ctx.expect_overlap(
        lid, body,
        axes="z",
        elem_a="stopper_cup",
        elem_b="gasket_ring",
        min_overlap=0.001,
        name="stopper base compresses gasket ring",
    )

    return ctx.report()


object_model = build_object_model()
