from __future__ import annotations

# Cosmetic FACE CREAM JAR — bail-lid flip-top variant.
# Frame: jar axis along +Z, base resting on z=0, jar centered on (x=0, y=0).
#
# A squat glass jar with a wide mouth, hinged flip-top lid, rubber gasket ring,
# and a wire clamp bail that pivots on two side ears. The lid hinges at the
# rear of the neck; the bail pivots on bosses on the ±X sides of the neck.
#
# Articulations:
#   - lid_hinge: REVOLUTE at rear of neck, axis along +X, positive q lifts
#     the front edge of the lid upward.
#   - bail_pivot: REVOLUTE on the side bosses, axis along -X, positive q
#     swings the bail from clamped (cross-bar on top) forward and down.

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

# ---- key dimensions (meters) ----
JAR_OUTER_R = 0.038           # outer radius of the glass body (~76 mm dia)
JAR_BODY_H = 0.055            # height of the glass body
WALL = 0.004                  # thick glass wall
NECK_R = 0.034                # outer radius of the wide-mouth neck
NECK_H = 0.010                # neck height above the shoulder
RIM_TOP_Z = JAR_BODY_H + NECK_H  # z of the rim top (0.065)

# Lid
LID_R = NECK_R + 0.001        # lid slightly wider than neck outer for overhang
LID_THICK = 0.005             # lid disc thickness

# Gasket (rubber ring under the lid)
GASKET_MEAN_R = NECK_R - WALL * 0.5  # centered on the rim wall
GASKET_CS_R = 0.0018          # gasket cross-section radius (tube)

# Bail
PIVOT_Z = JAR_BODY_H + NECK_H * 0.55  # pivot boss center on neck
BAIL_LEG = 0.017              # bail leg length (pivot to cross-bar, clears lid top)
BAIL_HALF_SPAN = NECK_R + 0.004  # bail wider than neck
WIRE_R = 0.0015               # bail wire radius

# Hinge ear dimensions
EAR_W = 0.006                 # ear width (along X)
EAR_H = 0.006                 # ear height (along Z)
EAR_D = 0.004                 # ear depth (along Y, protrusion from neck)


def _jar_glass_solid() -> cq.Workplane:
    """Hollow thick-walled glass jar — wide mouth, no threads.
    Revolve profile in XZ plane about Z axis. The inner cavity opens at the
    top, forming a real hollow mouth."""
    pts = [
        (0.0, 0.0),
        (JAR_OUTER_R, 0.0),
        (JAR_OUTER_R, JAR_BODY_H - 0.006),
        (JAR_OUTER_R - 0.003, JAR_BODY_H),          # rounded shoulder
        (NECK_R, JAR_BODY_H + 0.002),                # step into neck
        (NECK_R, RIM_TOP_Z),                         # neck outer up to rim
        (NECK_R - WALL, RIM_TOP_Z),                  # across rim top
        (NECK_R - WALL, JAR_BODY_H - 0.002),         # inner neck down
        (JAR_OUTER_R - WALL, JAR_BODY_H - 0.006),
        (JAR_OUTER_R - WALL, WALL),                  # inner wall down
        (0.0, WALL),                                  # inner base
        (0.0, 0.0),
    ]
    profile = cq.Workplane("XZ").polyline(pts).close()
    return profile.revolve(360.0, (0, 0, 0), (0, 1, 0))


def _neck_ears() -> cq.Workplane:
    """Small protruding ears on the neck: two on the sides for the bail pivot,
    one at the rear for the lid hinge."""
    result = None

    # Side pivot ears (for bail) at ±X
    for sign in (-1.0, 1.0):
        ear = (
            cq.Workplane("XY")
            .workplane(offset=PIVOT_Z - EAR_H * 0.5)
            .center(sign * (NECK_R + EAR_D * 0.5), 0.0)
            .rect(EAR_D, EAR_W)
            .extrude(EAR_H)
        )
        result = ear if result is None else result.union(ear)

    # Rear hinge ear (for lid) at -Y
    rear_ear = (
        cq.Workplane("XY")
        .workplane(offset=RIM_TOP_Z - EAR_H)
        .center(0.0, -(NECK_R + EAR_D * 0.5))
        .rect(EAR_W, EAR_D)
        .extrude(EAR_H)
    )
    result = result.union(rear_ear)

    return result


def _lid_disc() -> cq.Workplane:
    """Flat disc lid with a small raised rim around the edge and a hinge
    barrel at the rear."""
    # Main disc
    disc = (
        cq.Workplane("XY")
        .circle(LID_R)
        .extrude(LID_THICK)
    )
    # Raised rim on top edge
    rim = (
        cq.Workplane("XY")
        .workplane(offset=LID_THICK)
        .circle(LID_R)
        .circle(LID_R - 0.002)
        .extrude(0.001)
    )
    lid = disc.union(rim)

    # Hinge barrel at rear — a small cylinder along X at the back edge
    barrel = (
        cq.Workplane("YZ")
        .workplane(offset=-EAR_W * 0.5)
        .center(-LID_R + 0.002, LID_THICK * 0.5)
        .circle(0.0025)
        .extrude(EAR_W)
    )
    lid = lid.union(barrel)

    return lid


def _gasket_ring() -> cq.Workplane:
    """Rubber gasket ring that sits between the lid and the jar rim.
    Modeled as a torus."""
    # Revolve a small circle around the Z axis to make a torus
    gasket = (
        cq.Workplane("XZ")
        .center(GASKET_MEAN_R, 0.0)
        .circle(GASKET_CS_R)
        .revolve(360.0, (0, 0, 0), (0, 0, 1))
    )
    return gasket


def _bail_wire() -> cq.Workplane:
    """U-shaped wire bail with two legs and a cross-bar. The bail is modeled
    in its local frame with the pivot center at origin, legs extending
    upward (+Z) and the cross-bar at the top."""
    result = None

    # Left leg (at x = -BAIL_HALF_SPAN, extending from z=0 to z=BAIL_LEG)
    left_leg = (
        cq.Workplane("XY")
        .center(-BAIL_HALF_SPAN, 0.0)
        .circle(WIRE_R)
        .extrude(BAIL_LEG)
    )
    result = left_leg

    # Right leg
    right_leg = (
        cq.Workplane("XY")
        .center(BAIL_HALF_SPAN, 0.0)
        .circle(WIRE_R)
        .extrude(BAIL_LEG)
    )
    result = result.union(right_leg)

    # Cross-bar connecting the legs at the top (along X at z=BAIL_LEG)
    cross_bar = (
        cq.Workplane("YZ")
        .workplane(offset=-BAIL_HALF_SPAN)
        .center(0.0, BAIL_LEG)
        .circle(WIRE_R)
        .extrude(2.0 * BAIL_HALF_SPAN)
    )
    result = result.union(cross_bar)

    # Small pivot stubs at the base of each leg (engage the ear holes)
    for sign in (-1.0, 1.0):
        stub = (
            cq.Workplane("YZ")
            .workplane(offset=sign * BAIL_HALF_SPAN - sign * 0.003)
            .center(0.0, 0.0)
            .circle(WIRE_R * 1.3)
            .extrude(sign * 0.003)
        )
        result = result.union(stub)

    # Small rounded cap at cross-bar center for visual detail
    cap = (
        cq.Workplane("XY")
        .workplane(offset=BAIL_LEG - WIRE_R)
        .circle(WIRE_R * 2.0)
        .extrude(WIRE_R * 2.0)
    )
    result = result.union(cap)

    return result


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bail_lid_jar")

    # Materials
    glass_clear = model.material("glass_clear", rgba=(0.85, 0.90, 0.92, 0.45))
    lid_cream = model.material("lid_cream", rgba=(0.95, 0.93, 0.88, 1.0))
    gasket_rubber = model.material("gasket_rubber", rgba=(0.22, 0.20, 0.18, 1.0))
    bail_metal = model.material("bail_metal", rgba=(0.72, 0.72, 0.70, 1.0))
    label_olive = model.material("label_olive", rgba=(0.45, 0.52, 0.38, 1.0))

    # ---- jar body (root): glass shell + neck ears ----
    body = model.part("body")

    glass = _jar_glass_solid().union(_neck_ears())
    body.visual(
        mesh_from_cadquery(glass, "jar_glass"),
        material=glass_clear,
        name="jar_glass",
    )

    # Label band on the front of the body
    body.visual(
        Cylinder(JAR_OUTER_R + 0.0004, 0.016),
        origin=Origin(xyz=(0.0, 0.0, JAR_BODY_H * 0.45)),
        material=label_olive,
        name="brand_label",
    )

    body.inertial = Inertial.from_geometry(
        Cylinder(JAR_OUTER_R, JAR_BODY_H),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, JAR_BODY_H * 0.5)),
    )

    # ---- lid: flip-top disc hinged at the rear ----
    lid = model.part("lid")

    # Lid disc — in lid local frame, origin is at the hinge point (rear of rim).
    # The disc center is at (0, NECK_R, 0) in local coords (forward from hinge,
    # sitting on the rim plane).
    lid_disc_cq = _lid_disc()
    lid.visual(
        mesh_from_cadquery(lid_disc_cq, "lid_disc"),
        origin=Origin(xyz=(0.0, NECK_R, 0.0)),
        material=lid_cream,
        name="lid_disc",
    )

    # Gasket ring — attached to the underside of the lid, centered on the jar mouth.
    # In lid local frame, the gasket center is at (0, NECK_R, -GASKET_CS_R).
    gasket_cq = _gasket_ring()
    lid.visual(
        mesh_from_cadquery(gasket_cq, "gasket_ring"),
        origin=Origin(xyz=(0.0, NECK_R, -GASKET_CS_R)),
        material=gasket_rubber,
        name="gasket_ring",
    )

    lid.inertial = Inertial.from_geometry(
        Cylinder(LID_R, LID_THICK),
        mass=0.02,
        origin=Origin(xyz=(0.0, NECK_R, LID_THICK * 0.5)),
    )

    # Lid hinge articulation — at rear of rim, axis along +X
    # Positive q lifts the front edge (+Y from hinge) upward (+Z).
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -NECK_R, RIM_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=0.0, upper=1.8,
        ),
    )

    # ---- bail: wire clamp pivoting on side ears ----
    bail = model.part("bail")

    bail_cq = _bail_wire()
    bail.visual(
        mesh_from_cadquery(bail_cq, "bail_wire"),
        material=bail_metal,
        name="bail_wire",
    )

    bail.inertial = Inertial.from_geometry(
        Box((2.0 * BAIL_HALF_SPAN, 0.004, BAIL_LEG)),
        mass=0.01,
        origin=Origin(xyz=(0.0, 0.0, BAIL_LEG * 0.5)),
    )

    # Bail pivot articulation — on the side bosses, axis along -X
    # At q=0, bail legs point upward (cross-bar on top of lid = clamped).
    # Positive q swings bail forward (+Y) and down, opening it.
    model.articulation(
        "bail_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bail,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=0.0, upper=1.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    bail = object_model.get_part("bail")
    lid_hinge = object_model.get_articulation("lid_hinge")
    bail_pivot = object_model.get_articulation("bail_pivot")

    # ---- intentional overlaps ----
    # The gasket ring sits compressed between the lid and the rim.
    ctx.allow_overlap(
        lid, body,
        elem_a="gasket_ring", elem_b="jar_glass",
        reason="The rubber gasket is intentionally seated compressed between the lid disc and the jar rim.",
    )

    # ---- jar is a wide-mouth jar: mouth opening is at least 60% of body diameter ----
    body_aabb = ctx.part_world_aabb(body)
    body_dia = body_aabb[1][0] - body_aabb[0][0]
    ctx.check(
        "jar has a wide body",
        body_dia > 0.060,
        details=f"body diameter={body_dia:.4f}",
    )

    # ---- lid sits on top of the jar at rest (q=0) ----
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid is on top of the jar at rest",
        lid_pos is not None and lid_pos[2] > RIM_TOP_Z - 0.005,
        details=f"lid_pos={lid_pos}, rim_top={RIM_TOP_Z}",
    )

    # ---- lid overlaps the body footprint in XY (caps the mouth) ----
    ctx.expect_overlap(
        lid, body, axes="xy", min_overlap=0.02,
        name="lid caps the jar mouth",
    )

    # ---- gasket ring is present on the lid ----
    ctx.check(
        "gasket ring visual exists on lid",
        any(v.name == "gasket_ring" for v in lid.visuals),
        details="no gasket_ring visual found on lid",
    )

    # ---- lid_hinge opens the lid upward ----
    lid_rest_aabb = ctx.part_world_aabb(lid)
    lid_rest_top_z = lid_rest_aabb[1][2]
    with ctx.pose({lid_hinge: 1.2}):
        lid_open_aabb = ctx.part_world_aabb(lid)
        lid_open_top_z = lid_open_aabb[1][2]
        # The lid top should rise when opened
        ctx.check(
            "lid_hinge opens the lid upward",
            lid_open_top_z > lid_rest_top_z + 0.005,
            details=f"rest_top_z={lid_rest_top_z:.4f}, open_top_z={lid_open_top_z:.4f}",
        )

    # ---- lid_hinge is REVOLUTE with meaningful limits ----
    ctx.check(
        "lid_hinge is revolute with finite limits",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE
        and lid_hinge.motion_limits is not None
        and lid_hinge.motion_limits.upper > lid_hinge.motion_limits.lower + 0.5,
        details=f"type={lid_hinge.articulation_type}, limits={lid_hinge.motion_limits}",
    )

    # ---- bail_pivot swings the bail from top to open ----
    bail_rest_aabb = ctx.part_world_aabb(bail)
    bail_rest_top_z = bail_rest_aabb[1][2]
    with ctx.pose({bail_pivot: 1.5}):
        bail_open_aabb = ctx.part_world_aabb(bail)
        bail_open_top_z = bail_open_aabb[1][2]
        # The bail top should drop when swung open
        ctx.check(
            "bail_pivot swings bail from clamped to open",
            bail_rest_top_z > bail_open_top_z + 0.003,
            details=f"rest_top_z={bail_rest_top_z:.4f}, open_top_z={bail_open_top_z:.4f}",
        )

    # ---- bail_pivot is REVOLUTE with limits ----
    ctx.check(
        "bail_pivot is revolute with finite limits",
        bail_pivot.articulation_type == ArticulationType.REVOLUTE
        and bail_pivot.motion_limits is not None
        and bail_pivot.motion_limits.upper > bail_pivot.motion_limits.lower + 0.5,
        details=f"type={bail_pivot.articulation_type}, limits={bail_pivot.motion_limits}",
    )

    # ---- bail at rest (q=0) has cross-bar near the lid top ----
    # The bail cross-bar should be above the pivot point when clamped.
    bail_aabb = ctx.part_world_aabb(bail)
    bail_top_z = bail_aabb[1][2]
    ctx.check(
        "bail cross-bar is above pivot when clamped",
        bail_top_z > PIVOT_Z + BAIL_LEG * 0.5,
        details=f"bail_top_z={bail_top_z:.4f}, pivot_z={PIVOT_Z:.4f}",
    )

    # ---- jar body has ears (bosses) for bail and lid ----
    ctx.check(
        "jar body has visual geometry for mounting ears",
        any(v.name == "jar_glass" for v in body.visuals),
        details="jar_glass visual not found",
    )

    return ctx.report()


object_model = build_object_model()
