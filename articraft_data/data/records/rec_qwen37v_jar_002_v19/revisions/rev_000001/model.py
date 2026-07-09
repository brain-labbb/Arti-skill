from __future__ import annotations

# Spice jar variant: square glass jar with a flip lid on a rear revolute hinge,
# a rotating perforated shaker insert, a visible wide-mouth hollow opening,
# and a gasket ring under the lid.
#
# Frame: jar stands on +XY ground plane, central axis along +Z (up).
#   - jar_body (root): square-section clear glass shell with rounded vertical
#     edges, hollow interior, wide-mouth opening at top with a short rim lip.
#   - gasket: silicone ring seated on the jar rim.
#   - shaker: perforated disc insert that rotates inside the mouth (CONTINUOUS).
#   - lid: flip cap that opens on a rear revolute hinge (REVOLUTE).

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
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key dimensions (meters) ----
BODY_HALF = 0.035          # half-width of square section (0.070 m square)
BODY_FILLET = 0.010        # rounded vertical-edge radius
WALL = 0.003               # glass wall thickness
BODY_TOP = 0.090           # top of the square body section
RIM_HEIGHT = 0.006         # short rim lip above body
RIM_TOP = BODY_TOP + RIM_HEIGHT  # top of rim = mouth opening

MOUTH_R = BODY_HALF - WALL - 0.002  # inner mouth radius (approx circular)

LID_HALF = BODY_HALF + 0.002  # lid slightly wider than body for overlap
LID_THICK = 0.006             # lid cap thickness

GASKET_OR = BODY_HALF - 0.001   # gasket outer radius (sits on rim)
GASKET_IR = MOUTH_R + 0.001    # gasket inner radius
GASKET_THICK = 0.003            # gasket thickness

SHAKER_R = BODY_HALF - WALL + 0.001  # shaker disc radius (snap-fit contact with inner walls)
SHAKER_THICK = 0.002            # shaker plate thickness
SHAKER_Z = BODY_TOP - 0.005    # shaker sits just below the rim top

# Hinge location: rear edge of jar top (-Y side)
HINGE_Y = -(BODY_HALF + 0.001)
HINGE_Z = RIM_TOP


def _jar_body_solid() -> cq.Workplane:
    """Hollow square glass jar with rounded edges and wide-mouth opening."""
    # Outer square prism with filleted vertical edges
    outer = (
        cq.Workplane("XY")
        .box(2 * BODY_HALF, 2 * BODY_HALF, BODY_TOP, centered=(True, True, False))
        .edges("|Z")
        .fillet(BODY_FILLET)
    )

    # Short rim lip on top (slightly inset from outer wall)
    rim = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP)
        .box(
            2 * (BODY_HALF - 0.001),
            2 * (BODY_HALF - 0.001),
            RIM_HEIGHT,
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(max(BODY_FILLET - 0.001, 0.002))
    )

    solid = outer.union(rim)

    # Hollow interior: cut cavity from bottom wall up through the mouth
    inner_half = BODY_HALF - WALL
    inner = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .box(
            2 * inner_half,
            2 * inner_half,
            BODY_TOP - WALL + RIM_HEIGHT + 0.001,
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(max(BODY_FILLET - WALL, 0.001))
    )

    return solid.cut(inner)


def _jar_body_mesh():
    return mesh_from_cadquery(_jar_body_solid(), "jar_glass")


def _gasket_solid() -> cq.Workplane:
    """Flat ring gasket that sits on the jar rim."""
    outer = (
        cq.Workplane("XY")
        .circle(GASKET_OR)
        .extrude(GASKET_THICK)
    )
    inner_cut = (
        cq.Workplane("XY")
        .circle(GASKET_IR)
        .extrude(GASKET_THICK + 0.001)
    )
    return outer.cut(inner_cut)


def _gasket_mesh():
    return mesh_from_cadquery(_gasket_solid(), "gasket_ring")


def _shaker_solid() -> cq.Workplane:
    """Perforated disc shaker insert with radial hole pattern."""
    disc = (
        cq.Workplane("XY")
        .circle(SHAKER_R)
        .extrude(SHAKER_THICK)
    )

    # Central hole
    center_hole = (
        cq.Workplane("XY")
        .circle(0.004)
        .extrude(SHAKER_THICK + 0.001)
    )
    disc = disc.cut(center_hole)

    # Ring of perforation holes
    hole_r = 0.002
    ring_r = SHAKER_R * 0.55
    n_holes = 12
    for i in range(n_holes):
        ang = 2.0 * math.pi * i / n_holes
        hx = ring_r * math.cos(ang)
        hy = ring_r * math.sin(ang)
        hole = (
            cq.Workplane("XY")
            .center(hx, hy)
            .circle(hole_r)
            .extrude(SHAKER_THICK + 0.001)
        )
        disc = disc.cut(hole)

    # Outer ring of smaller holes
    ring_r2 = SHAKER_R * 0.82
    n_holes2 = 18
    for i in range(n_holes2):
        ang = 2.0 * math.pi * i / n_holes2
        hx = ring_r2 * math.cos(ang)
        hy = ring_r2 * math.sin(ang)
        hole = (
            cq.Workplane("XY")
            .center(hx, hy)
            .circle(0.0015)
            .extrude(SHAKER_THICK + 0.001)
        )
        disc = disc.cut(hole)

    return disc


def _shaker_mesh():
    return mesh_from_cadquery(_shaker_solid(), "shaker_disc")


def _lid_solid() -> cq.Workplane:
    """Flat flip lid cap with a rear hinge tab."""
    # Main cap: square with rounded corners
    cap = (
        cq.Workplane("XY")
        .box(2 * LID_HALF, 2 * LID_HALF, LID_THICK, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.006)
    )

    # Hinge tab: small protrusion at the rear (-Y) for the hinge connection
    tab = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .center(0.0, -LID_HALF)
        .box(0.016, 0.010, LID_THICK, centered=(True, True, False))
    )
    cap = cap.union(tab)

    # Small lip on underside for sealing (inner ring)
    lip = (
        cq.Workplane("XY")
        .circle(GASKET_IR - 0.001)
        .extrude(-0.003)
    )
    cap = cap.union(lip)

    return cap


def _lid_mesh():
    return mesh_from_cadquery(_lid_solid(), "lid_cap")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spice_jar_shaker")

    glass = model.material("clear_glass", rgba=(0.82, 0.88, 0.85, 0.30))
    silicone = model.material("silicone_gasket", rgba=(0.85, 0.35, 0.25, 1.0))
    plastic_white = model.material("plastic_white", rgba=(0.92, 0.91, 0.88, 1.0))
    plastic_lid = model.material("plastic_lid", rgba=(0.88, 0.86, 0.82, 1.0))

    # ---- jar body (root): hollow square glass with wide mouth ----
    body = model.part("jar_body")
    body.visual(_jar_body_mesh(), material=glass, name="jar_glass")
    body.inertial = Inertial.from_geometry(
        Box((2 * BODY_HALF, 2 * BODY_HALF, RIM_TOP)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP / 2.0)),
    )

    # ---- gasket ring: seated on rim ----
    gasket = model.part("gasket")
    gasket.visual(
        _gasket_mesh(),
        material=silicone,
        name="gasket_ring",
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP)),
    )
    gasket.inertial = Inertial.from_geometry(
        Cylinder(GASKET_OR, GASKET_THICK),
        mass=0.005,
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP + GASKET_THICK / 2.0)),
    )

    # ---- shaker insert: perforated disc that rotates inside the mouth ----
    shaker = model.part("shaker")
    shaker.visual(
        _shaker_mesh(),
        material=plastic_white,
        name="shaker_disc",
        origin=Origin(xyz=(0.0, 0.0, SHAKER_Z)),
    )
    # Off-axis marker so rotation is observable
    marker = CylinderGeometry(0.002, SHAKER_THICK).translate(SHAKER_R - 0.005, 0.0, SHAKER_Z)
    shaker.visual(
        mesh_from_geometry(marker, "shaker_marker"),
        material=silicone,
        name="shaker_marker",
    )
    shaker.inertial = Inertial.from_geometry(
        Cylinder(SHAKER_R, SHAKER_THICK),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, SHAKER_Z + SHAKER_THICK / 2.0)),
    )

    # ---- lid: flip cap on rear hinge ----
    lid = model.part("lid")
    # Lid part frame is at the hinge line; geometry extends along +Y (forward)
    lid.visual(
        _lid_mesh(),
        material=plastic_lid,
        name="lid_cap",
        origin=Origin(xyz=(0.0, LID_HALF, 0.0)),
    )
    lid.inertial = Inertial.from_geometry(
        Box((2 * LID_HALF, 2 * LID_HALF, LID_THICK)),
        mass=0.015,
        origin=Origin(xyz=(0.0, LID_HALF, LID_THICK / 2.0)),
    )

    # ---- articulation: gasket is fixed to jar body ----
    model.articulation(
        "body_to_gasket",
        ArticulationType.FIXED,
        parent=body,
        child=gasket,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # ---- articulation: shaker rotates about +Z (continuous) ----
    model.articulation(
        "shaker_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=shaker,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=2.0),
    )

    # ---- articulation: lid flip hinge at rear (revolute) ----
    # Hinge at rear edge (-Y) of jar rim. Lid extends along +Y from hinge.
    # axis=(1,0,0): right-hand rule curls +Y toward +Z, so positive q opens lid up.
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0, lower=0.0, upper=2.2
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("jar_body")
    gasket = object_model.get_part("gasket")
    shaker = object_model.get_part("shaker")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("lid_hinge")
    shaker_rot = object_model.get_articulation("shaker_rotate")

    # --- Allowances for intentional overlaps ---
    # Gasket is seated on the jar rim (compression fit)
    ctx.allow_overlap(
        gasket,
        body,
        elem_a="gasket_ring",
        elem_b="jar_glass",
        reason="The gasket ring is seated on the jar rim as a compression seal.",
    )

    # Shaker disc sits inside the jar mouth cavity
    ctx.allow_overlap(
        shaker,
        body,
        elem_a="shaker_disc",
        elem_b="jar_glass",
        reason="The shaker insert sits inside the jar mouth cavity.",
    )

    # Lid cap overlaps the gasket area when closed
    ctx.allow_overlap(
        lid,
        gasket,
        elem_a="lid_cap",
        elem_b="gasket_ring",
        reason="The closed lid cap presses against the gasket seal.",
    )

    # --- jar body is square section ---
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jar body is square in cross-section",
        abs(bext[0] - bext[1]) < 0.008,
        details=f"x={bext[0]:.4f}, y={bext[1]:.4f}",
    )
    ctx.check(
        "jar is taller than wide",
        bext[2] > bext[0] + 0.02,
        details=f"extents={bext}",
    )

    # --- wide mouth: jar body has a hollow opening at top ---
    # The inner cavity should be wide relative to the body
    ctx.check(
        "wide mouth opening (inner width > 60% of body width)",
        (2 * MOUTH_R) > 0.6 * (2 * BODY_HALF),
        details=f"mouth_dia={2*MOUTH_R:.4f}, body_width={2*BODY_HALF:.4f}",
    )

    # --- gasket ring is seated on jar rim ---
    gasket_aabb = ctx.part_world_aabb(gasket)
    gasket_zmin = gasket_aabb[0][2] if gasket_aabb else 0.0
    gasket_zmax = gasket_aabb[1][2] if gasket_aabb else 0.0
    ctx.check(
        "gasket sits at top of jar body",
        gasket_aabb is not None and gasket_zmin > BODY_TOP - 0.005,
        details=f"gasket z range=[{gasket_zmin:.4f}, {gasket_zmax:.4f}]",
    )
    ctx.expect_overlap(
        gasket, body, axes="xy", min_overlap=0.01,
        name="gasket footprint overlaps jar body",
    )

    # --- shaker insert is inside the jar mouth ---
    shaker_aabb = ctx.part_world_aabb(shaker)
    shaker_zmin = shaker_aabb[0][2] if shaker_aabb else 0.0
    shaker_zmax = shaker_aabb[1][2] if shaker_aabb else 0.0
    ctx.check(
        "shaker sits inside the jar mouth",
        shaker_aabb is not None and shaker_zmin > BODY_TOP * 0.8,
        details=f"shaker z range=[{shaker_zmin:.4f}, {shaker_zmax:.4f}]",
    )
    ctx.expect_within(
        shaker, body, axes="xy",
        inner_elem="shaker_disc",
        outer_elem="jar_glass",
        margin=0.005,
        name="shaker disc stays within jar body footprint",
    )

    # --- shaker rotates about +Z (continuous joint) ---
    ctx.check(
        "shaker_rotate is continuous about +Z",
        shaker_rot.axis == (0.0, 0.0, 1.0),
        details=f"axis={shaker_rot.axis}, type={shaker_rot.articulation_type}",
    )
    # Marker moves when shaker rotates
    m0 = ctx.part_element_world_aabb(shaker, elem="shaker_marker")
    m0c = ((m0[0][0] + m0[1][0]) / 2.0, (m0[0][1] + m0[1][1]) / 2.0)
    with ctx.pose({shaker_rot: math.pi / 2.0}):
        m1 = ctx.part_element_world_aabb(shaker, elem="shaker_marker")
        m1c = ((m1[0][0] + m1[1][0]) / 2.0, (m1[0][1] + m1[1][1]) / 2.0)
    marker_shift = math.hypot(m1c[0] - m0c[0], m1c[1] - m0c[1])
    ctx.check(
        "shaker rotation moves the marker",
        marker_shift > 0.005,
        details=f"marker moved {marker_shift:.4f} m on quarter turn",
    )

    # --- lid hinge: revolute, opens upward ---
    ctx.check(
        "lid_hinge is revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={hinge.articulation_type}",
    )
    ctx.check(
        "lid_hinge axis is along X (rear hinge line)",
        hinge.axis == (1.0, 0.0, 0.0),
        details=f"axis={hinge.axis}",
    )

    # Lid closed position: lid is at the top of the jar
    lid_pos_closed = ctx.part_world_position(lid)
    ctx.check(
        "lid sits at top of jar when closed",
        lid_pos_closed is not None and lid_pos_closed[2] > BODY_TOP - 0.01,
        details=f"lid z={lid_pos_closed[2] if lid_pos_closed else None}",
    )

    # Lid open position: lid moves upward when hinge is opened
    lid_center_closed = ctx.part_world_aabb(lid)
    z_closed_max = lid_center_closed[1][2]
    with ctx.pose({hinge: 1.5}):
        lid_center_open = ctx.part_world_aabb(lid)
        z_open_max = lid_center_open[1][2]
    ctx.check(
        "lid opens upward on hinge (positive q raises lid top)",
        z_open_max > z_closed_max + 0.02,
        details=f"closed top z={z_closed_max:.4f}, open top z={z_open_max:.4f}",
    )

    # Hinge limits are sensible
    limits = hinge.motion_limits
    ctx.check(
        "lid hinge has sensible limits (0 to ~2.2 rad)",
        limits is not None and limits.lower is not None and limits.upper is not None
        and limits.lower >= 0.0 and limits.upper > 1.0,
        details=f"lower={limits.lower if limits else None}, upper={limits.upper if limits else None}",
    )

    return ctx.report()


object_model = build_object_model()
