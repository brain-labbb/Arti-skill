from __future__ import annotations

# WIDE APOTHECARY JAR WITH DOMED STOPPER.
# Frame: jar axis along +Z, base resting on z=0, jar centered on (x=0, y=0).
# Amber glass apothecary jar with a wide mouth, thick walls, and a domed glass
# stopper that seats into the mouth on a short cylindrical plug.
#
# Articulations (screw stopper pattern, both share the vertical +Z axis,
# decoupled through a massless carrier link):
#   - stopper_rotate: CONTINUOUS spin of the carrier about +Z at the rim top
#   - stopper_slide:  PRISMATIC lift of the stopper relative to the carrier along +Z

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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- key dimensions (meters) ----
BODY_OUTER_R = 0.050          # outer radius of the apothecary body (~100mm dia)
BODY_H = 0.065                # height of the main body cylinder
WALL = 0.005                  # thick glass wall
BASE_THICK = 0.008            # thick flat base

# Shoulder and mouth geometry
SHOULDER_H = 0.008            # height of the curved shoulder transition
MOUTH_OUTER_R = 0.042         # outer radius of the mouth rim
MOUTH_INNER_R = 0.038         # inner radius (wide mouth opening)
RIM_H = 0.010                 # height of the raised mouth rim/lip
RIM_TOP_Z = BODY_H + SHOULDER_H + RIM_H  # top of the rim where stopper seats

# Stopper geometry (in stopper-local frame, origin at rim top)
PLUG_R = MOUTH_INNER_R - 0.001  # plug slightly smaller than mouth for clearance
PLUG_H = 0.012                  # plug insertion depth
FLANGE_R = MOUTH_OUTER_R + 0.002  # flange wider than mouth outer to seat on rim
FLANGE_H = 0.005                # flange thickness
DOME_R = 0.032                  # dome base radius (narrower than flange for elegance)
DOME_H = 0.048                  # dome height (tall prominent apothecary dome)

# Stopper total height from plug bottom to dome top
STOPPER_TOTAL_H = PLUG_H + FLANGE_H + DOME_H


def _jar_body_solid() -> cq.Workplane:
    """Hollow thick-walled apothecary jar body.

    Revolved half-profile in XZ plane: traces outer wall up through shoulder
    into the raised mouth rim, then back down the inner wall to form a real
    open-topped cavity with a wide mouth.
    """
    pts = [
        (0.0, 0.0),                                    # center of base
        (BODY_OUTER_R, 0.0),                           # outer base edge
        (BODY_OUTER_R, BODY_H - 0.003),                # outer wall up (slight taper at top)
        (BODY_OUTER_R - 0.003, BODY_H + SHOULDER_H * 0.5),  # rounded shoulder start
        (MOUTH_OUTER_R + 0.003, BODY_H + SHOULDER_H),  # shoulder to rim base
        (MOUTH_OUTER_R + 0.003, RIM_TOP_Z),            # outer rim wall up
        (MOUTH_OUTER_R, RIM_TOP_Z),                    # across rim top outer edge
        (MOUTH_OUTER_R, RIM_TOP_Z - 0.003),            # slight lip step down
        (MOUTH_INNER_R, RIM_TOP_Z - 0.003),            # across to inner rim
        (MOUTH_INNER_R, BODY_H + SHOULDER_H - 0.002),  # inner mouth wall down
        (BODY_OUTER_R - WALL, BODY_H + SHOULDER_H * 0.4),  # inner shoulder
        (BODY_OUTER_R - WALL, BASE_THICK),             # inner body wall down
        (0.0, BASE_THICK),                             # across inner base
        (0.0, 0.0),                                    # close back to center
    ]
    profile = cq.Workplane("XZ").polyline(pts).close()
    body = profile.revolve(360.0, (0, 0, 0), (0, 1, 0))
    return body


def _body_decoration_ring() -> cq.Workplane:
    """A raised decorative ring embossed around the body near the shoulder."""
    ring_z = BODY_H - 0.008
    ring = (
        cq.Workplane("XY")
        .workplane(offset=ring_z)
        .circle(BODY_OUTER_R + 0.002)
        .circle(BODY_OUTER_R - 0.001)
        .extrude(0.004)
    )
    return ring


def _cream_fill_mesh():
    """Cream/ointment filling the jar interior, visible through the wide mouth."""
    inner_r = BODY_OUTER_R - WALL - 0.001
    fill_h = BODY_H - BASE_THICK - 0.015  # cream level below the rim
    cream = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICK)
        .circle(inner_r)
        .extrude(fill_h)
    )
    # Slight domed top surface of the cream
    dome = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICK + fill_h)
        .circle(inner_r)
        .workplane(offset=0.006)
        .circle(inner_r * 0.6)
        .loft(ruled=False)
    )
    result = cream.union(dome)
    return mesh_from_cadquery(result, "cream_fill")


def _stopper_solid() -> cq.Workplane:
    """Domed glass stopper: plug + flange + dome, authored in stopper-local frame.

    Origin is at the rim top (world z=RIM_TOP_Z at rest). The plug extends
    downward (-Z in local) into the mouth, the flange sits at z=0, and the
    dome rises above.
    """
    # Plug: short cylinder that inserts into the mouth
    plug = (
        cq.Workplane("XY")
        .workplane(offset=-PLUG_H)
        .circle(PLUG_R)
        .extrude(PLUG_H)
    )
    # Flange: wider disc that seats on the rim
    flange = (
        cq.Workplane("XY")
        .circle(FLANGE_R)
        .extrude(FLANGE_H)
    )
    # Dome: elegant curved top built as a loft from flange radius to apex
    dome_base = (
        cq.Workplane("XY")
        .workplane(offset=FLANGE_H)
        .circle(DOME_R)
    )
    dome_mid = (
        cq.Workplane("XY")
        .workplane(offset=FLANGE_H + DOME_H * 0.55)
        .circle(DOME_R * 0.55)
    )
    dome_top = (
        cq.Workplane("XY")
        .workplane(offset=FLANGE_H + DOME_H * 0.85)
        .circle(DOME_R * 0.20)
    )
    # Build dome as stacked lofts for smooth curvature
    dome_lower = (
        cq.Workplane("XY")
        .workplane(offset=FLANGE_H)
        .circle(DOME_R)
        .workplane(offset=DOME_H * 0.55)
        .circle(DOME_R * 0.55)
        .loft(ruled=False)
    )
    dome_upper = (
        cq.Workplane("XY")
        .workplane(offset=FLANGE_H + DOME_H * 0.55)
        .circle(DOME_R * 0.55)
        .workplane(offset=DOME_H * 0.30)
        .circle(DOME_R * 0.20)
        .loft(ruled=False)
    )
    dome_cap = (
        cq.Workplane("XY")
        .workplane(offset=FLANGE_H + DOME_H * 0.85)
        .circle(DOME_R * 0.20)
        .workplane(offset=DOME_H * 0.15)
        .circle(0.002)
        .loft(ruled=False)
    )
    stopper = plug.union(flange).union(dome_lower).union(dome_upper).union(dome_cap)
    return stopper


def _stopper_grip_mesh():
    """Small grip knob at the very top of the dome for pulling the stopper."""
    knob = (
        cq.Workplane("XY")
        .workplane(offset=FLANGE_H + DOME_H - 0.002)
        .circle(0.006)
        .workplane(offset=0.008)
        .circle(0.004)
        .loft(ruled=False)
    )
    return mesh_from_cadquery(knob, "stopper_grip")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="apothecary_jar")

    # Materials
    glass_amber = model.material("glass_amber", rgba=(0.55, 0.32, 0.10, 0.60))
    glass_dark = model.material("glass_dark", rgba=(0.18, 0.12, 0.06, 0.85))
    cream_white = model.material("cream_white", rgba=(0.95, 0.92, 0.82, 1.0))
    ring_gold = model.material("ring_gold", rgba=(0.72, 0.58, 0.22, 1.0))

    # ---- jar body (root): glass shell + decoration ring + cream fill ----
    body = model.part("body")

    jar_shell = _jar_body_solid().union(_body_decoration_ring())
    body.visual(
        mesh_from_cadquery(jar_shell, "jar_glass"),
        material=glass_amber,
        name="jar_glass",
    )

    # Decorative ring colored differently
    body.visual(
        mesh_from_cadquery(_body_decoration_ring(), "deco_ring"),
        material=ring_gold,
        name="deco_ring",
    )

    # Cream filling visible through the wide mouth
    body.visual(_cream_fill_mesh(), material=cream_white, name="cream_fill")

    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_OUTER_R, BODY_H + SHOULDER_H + RIM_H),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, (BODY_H + SHOULDER_H + RIM_H) * 0.5)),
    )

    # ---- massless carrier (no visuals): rotates about +Z at the rim top ----
    carrier = model.part("stopper_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.008, 0.008, 0.008)), mass=1e-4)
    model.articulation(
        "stopper_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    # ---- domed stopper: plugs into the mouth, slides up off the carrier ----
    stopper = model.part("stopper")
    stopper.visual(
        mesh_from_cadquery(_stopper_solid(), "stopper_shell"),
        material=glass_dark,
        name="stopper_shell",
    )
    stopper.visual(_stopper_grip_mesh(), material=glass_dark, name="stopper_grip")
    # Small off-axis marker plate on the flange so rotation is visible.
    # Slightly embedded into the flange top for guaranteed mesh connectivity.
    stopper.visual(
        Box((0.006, 0.006, 0.002)),
        origin=Origin(xyz=(FLANGE_R - 0.008, 0.0, FLANGE_H)),
        material=ring_gold,
        name="stopper_marker",
    )
    stopper.inertial = Inertial.from_geometry(
        Cylinder(FLANGE_R, STOPPER_TOTAL_H),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, -PLUG_H + STOPPER_TOTAL_H * 0.5)),
    )
    model.articulation(
        "stopper_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=stopper,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=PLUG_H + FLANGE_H + 0.01, effort=1.0, velocity=1.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    carrier = object_model.get_part("stopper_carrier")
    stopper = object_model.get_part("stopper")
    rotate = object_model.get_articulation("stopper_rotate")
    slide = object_model.get_articulation("stopper_slide")

    # The stopper plug is intentionally inserted into the jar mouth.
    ctx.allow_overlap(
        stopper,
        body,
        elem_a="stopper_shell",
        elem_b="jar_glass",
        reason="The stopper plug is intentionally inserted into the wide mouth cavity.",
    )

    # ---- jar body has a wide mouth: mouth inner diameter > 60% of body diameter ----
    body_ext = _ext(ctx.part_world_aabb(body))
    body_diameter = max(body_ext[0], body_ext[1])
    mouth_ratio = (2.0 * MOUTH_INNER_R) / body_diameter
    ctx.check(
        "jar has a wide mouth (>= 60% of body diameter)",
        mouth_ratio >= 0.60,
        details=f"mouth_ratio={mouth_ratio:.2f}, body_diameter={body_diameter:.4f}",
    )

    # ---- jar body is wider than it is tall (apothecary proportions) ----
    ctx.check(
        "jar is wider than tall (apothecary proportions)",
        body_ext[0] > body_ext[2] and body_ext[1] > body_ext[2],
        details=f"body_extents={body_ext}",
    )

    # ---- stopper sits on top of the jar ----
    body_pos = ctx.part_world_position(body)
    stopper_pos = ctx.part_world_position(stopper)
    ctx.check(
        "stopper is on top of the jar",
        stopper_pos is not None and body_pos is not None and stopper_pos[2] > RIM_TOP_Z - 0.005,
        details=f"stopper_pos={stopper_pos}, rim_top={RIM_TOP_Z}",
    )

    # ---- stopper footprint overlaps the mouth (seated in the rim) ----
    ctx.expect_overlap(
        stopper, body, axes="xy", min_overlap=0.02,
        name="stopper seats in the mouth",
    )

    # ---- dome rises prominently above the flange (domed stopper shape) ----
    stopper_ext = _ext(ctx.part_world_aabb(stopper))
    # The dome should contribute significant height relative to the assembly width
    ctx.check(
        "stopper has a prominent dome (height > 65% of width)",
        stopper_ext[2] > stopper_ext[0] * 0.65,
        details=f"stopper_extents={stopper_ext}",
    )

    # ---- stopper_rotate spins the stopper (marker moves) ----
    marker0 = ctx.part_element_world_aabb(stopper, elem="stopper_marker")
    m0 = ((marker0[0][0] + marker0[1][0]) * 0.5,
          (marker0[0][1] + marker0[1][1]) * 0.5)
    with ctx.pose({rotate: math.pi / 2.0}):
        marker1 = ctx.part_element_world_aabb(stopper, elem="stopper_marker")
        m1 = ((marker1[0][0] + marker1[1][0]) * 0.5,
              (marker1[0][1] + marker1[1][1]) * 0.5)
    moved = math.hypot(m1[0] - m0[0], m1[1] - m0[1])
    ctx.check(
        "stopper_rotate spins the stopper (marker moves)",
        moved > 0.01,
        details=f"marker rest={m0}, quarter-turn={m1}, moved={moved}",
    )

    # ---- stopper_slide lifts the stopper off the jar ----
    rest_z = ctx.part_world_position(stopper)[2]
    lift_amount = PLUG_H + FLANGE_H + 0.01
    with ctx.pose({slide: lift_amount}):
        lifted_z = ctx.part_world_position(stopper)[2]
        # When fully lifted, stopper clears the rim along +Z
        ctx.expect_gap(
            stopper, body, axis="z", min_gap=0.0,
            positive_elem="stopper_shell", negative_elem="jar_glass",
            name="lifted stopper clears the rim",
        )
    ctx.check(
        "stopper_slide lifts the stopper off the jar",
        lifted_z > rest_z + lift_amount * 0.5,
        details=f"rest_z={rest_z}, lifted_z={lifted_z}",
    )

    # ---- carrier is massless / has no visuals ----
    ctx.check(
        "carrier link has no visuals",
        len(carrier.visuals) == 0,
        details=f"carrier visuals={len(carrier.visuals)}",
    )

    # ---- hollow interior: cream fill is within the jar body ----
    cream = object_model.get_part("body")
    ctx.expect_within(
        body, body, axes="xy",
        inner_elem="cream_fill", outer_elem="jar_glass",
        margin=0.002,
        name="cream fill is within the jar walls",
    )

    return ctx.report()


object_model = build_object_model()
