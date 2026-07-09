from __future__ import annotations

# Round brushed-metal canister tin with a threaded screw cap.
# Fork of the lift-off lid variant: the straight-lift press lid is replaced
# by a short threaded neck rising from the body mouth plus a knurled screw
# cap that spins (CONTINUOUS) and lifts off (PRISMATIC through a carrier).
# Body footprint is identical to the parent canister.
#
# Articulations:
#   - body_to_carrier: PRISMATIC (+Z), lifts the cap assembly off the neck
#   - carrier_to_cap: CONTINUOUS (Z axis), spins the cap for threading

import math
import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    KnobGeometry,
    KnobBore,
    KnobGrip,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- canister body dimensions (meters, identical to parent) ----
BODY_R = 0.050       # outer radius (~0.10 m diameter)
BODY_H = 0.118       # body height (rim sits at z = BODY_H)
WALL = 0.0035        # sheet-metal wall thickness
FLOOR = 0.004        # interior floor thickness

# ---- threaded neck (rises from body mouth via shoulder) ----
NECK_R = 0.030       # neck outer radius
NECK_IR = 0.0265     # neck inner bore radius
NECK_H = 0.020       # neck height above the shoulder
SHOULDER_H = 0.004   # shoulder plate thickness (connects body wall to neck)

# Thread grooves cut into the neck exterior
THREAD_PITCH = 0.004     # vertical spacing between thread grooves
THREAD_DEPTH = 0.0012    # groove radial depth
THREAD_GROOVE_H = 0.0015 # groove axial height
THREAD_COUNT = 4         # number of thread grooves

# ---- screw cap ----
CAP_OD = 0.072       # cap outer diameter
CAP_H = 0.028        # cap total height (skirt + top plate)
KNURL_COUNT = 36     # knurl ridges on cap exterior
CAP_BORE_D = 0.062   # cap bore diameter (clears neck outer)

# ---- tamper-evident ring (carrier visual) ----
RING_OR = 0.037      # ring outer radius (slightly larger than cap bore)
RING_IR = 0.031      # ring inner radius (clears neck outer)
RING_H = 0.003       # ring height

# Derived heights
SHOULDER_Z = BODY_H                              # shoulder bottom
NECK_BASE_Z = BODY_H + SHOULDER_H                # neck base (shoulder top)
NECK_TOP_Z = NECK_BASE_Z + NECK_H                # neck top


def _body_solid() -> cq.Workplane:
    """Hollow cylinder body with shoulder plate and threaded neck."""
    # Main body: hollow cylinder open at the top
    outer = cq.Workplane("XY").circle(BODY_R).extrude(BODY_H)
    inner = (
        cq.Workplane("XY")
        .workplane(offset=FLOOR)
        .circle(BODY_R - WALL)
        .extrude(BODY_H)  # overshoots top so rim is open
    )
    body = outer.cut(inner)

    # Shoulder plate: connects body wall top to neck base
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=SHOULDER_Z)
        .circle(BODY_R)
        .circle(NECK_IR)
        .extrude(SHOULDER_H)
    )
    body = body.union(shoulder)

    # Neck: hollow ring rising from the shoulder
    neck_outer = (
        cq.Workplane("XY")
        .workplane(offset=NECK_BASE_Z)
        .circle(NECK_R)
        .extrude(NECK_H)
    )
    neck_bore = (
        cq.Workplane("XY")
        .workplane(offset=NECK_BASE_Z - 0.001)
        .circle(NECK_IR)
        .extrude(NECK_H + 0.002)  # through-cut
    )
    neck = neck_outer.cut(neck_bore)
    body = body.union(neck)

    # Thread grooves: ring cuts on the neck exterior (repeated feature)
    for i in range(THREAD_COUNT):
        groove_z = NECK_BASE_Z + 0.003 + i * THREAD_PITCH
        groove = (
            cq.Workplane("XY")
            .workplane(offset=groove_z)
            .circle(NECK_R + 0.001)
            .circle(NECK_R - THREAD_DEPTH)
            .extrude(THREAD_GROOVE_H)
        )
        body = body.cut(groove)

    return body


def _ring_solid() -> cq.Workplane:
    """Tamper-evident ring: thin annular collar at the cap base."""
    return (
        cq.Workplane("XY")
        .circle(RING_OR)
        .circle(RING_IR)
        .extrude(RING_H)
    )


def _cap_knob() -> KnobGeometry:
    """Knurled screw cap built via KnobGeometry with bore for neck."""
    return KnobGeometry(
        CAP_OD,
        CAP_H,
        body_style="cylindrical",
        grip=KnobGrip(style="knurled", count=KNURL_COUNT, depth=0.0009, helix_angle_deg=25.0),
        bore=KnobBore(style="round", diameter=CAP_BORE_D),
        center=False,  # mounting face at z=0
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="canister_tin_screw_cap")

    brushed = model.material("brushed_metal", rgba=(0.78, 0.79, 0.81, 1.0))
    dark_ring = model.material("dark_brass", rgba=(0.55, 0.48, 0.30, 1.0))

    # ---- body (root): hollow cylinder with threaded neck ----
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_solid(), "body_shell"),
        material=brushed,
        name="body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, BODY_H + SHOULDER_H + NECK_H),
        mass=0.095,
        origin=Origin(xyz=(0.0, 0.0, (BODY_H + SHOULDER_H + NECK_H) / 2.0)),
    )

    # ---- cap_carrier: tamper-evident ring, PRISMATIC child ----
    # The ring sits on the shoulder around the neck base at rest.
    carrier = model.part("cap_carrier")
    carrier.visual(
        mesh_from_cadquery(_ring_solid(), "carrier_ring"),
        material=dark_ring,
        name="carrier_ring",
    )

    # ---- screw_cap: knurled cap with internal bore ----
    screw_cap = model.part("screw_cap")
    screw_cap.visual(
        mesh_from_geometry(_cap_knob(), "screw_cap_shell"),
        material=brushed,
        name="screw_cap_shell",
    )
    screw_cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_OD / 2.0, CAP_H),
        mass=0.020,
        origin=Origin(xyz=(0.0, 0.0, CAP_H / 2.0)),
    )

    # ---- articulations ----

    # body_to_carrier: PRISMATIC, lifts cap assembly off the neck
    # Origin at the shoulder surface (where the ring/cap seat)
    model.articulation(
        "body_to_carrier",
        ArticulationType.PRISMATIC,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, NECK_BASE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.1, lower=0.0, upper=0.050),
    )

    # carrier_to_cap: CONTINUOUS, spins the cap about the body axis
    # Origin at the ring top surface (cap base sits on the ring)
    model.articulation(
        "carrier_to_cap",
        ArticulationType.CONTINUOUS,
        parent=carrier,
        child=screw_cap,
        origin=Origin(xyz=(0.0, 0.0, RING_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    carrier = object_model.get_part("cap_carrier")
    cap = object_model.get_part("screw_cap")
    prismatic = object_model.get_articulation("body_to_carrier")
    continuous = object_model.get_articulation("carrier_to_cap")

    # ---- body shape: cylinder taller than wide, circular plan ----
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "body taller than wide",
        bext[2] > max(bext[0], bext[1]) + 0.01,
        details=f"body extents={bext}",
    )
    ctx.check(
        "body roughly circular in plan",
        abs(bext[0] - bext[1]) < 0.01,
        details=f"body extents={bext}",
    )

    # ---- neck narrows from body (body wider than neck region) ----
    # At the top of the body, the neck is narrower than the body diameter
    ctx.check(
        "neck narrower than body",
        NECK_R * 2 < BODY_R * 2 - 0.01,
        details=f"neck_d={NECK_R*2}, body_d={BODY_R*2}",
    )

    # ---- cap shape: roughly circular, with knurling ----
    cap_ext = _ext(ctx.part_world_aabb(cap))
    ctx.check(
        "cap roughly circular",
        abs(cap_ext[0] - cap_ext[1]) < 0.006,
        details=f"cap extents={cap_ext}",
    )

    # ---- cap seated on neck at rest ----
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap mounted at neck height",
        cap_pos is not None and cap_pos[2] >= NECK_BASE_Z - 0.005,
        details=f"cap origin z={cap_pos[2] if cap_pos else None}, neck_base={NECK_BASE_Z}",
    )

    # Cap overlaps with neck region in Z when seated
    ctx.expect_overlap(
        cap, body, axes="z", min_overlap=0.005,
        name="cap overlaps neck region when seated",
    )

    # Carrier ring sits at the neck base
    ctx.expect_overlap(
        carrier, body, axes="xy", min_overlap=0.005,
        name="carrier ring shares footprint with neck region",
    )

    # ---- intentional overlaps: cap bore wraps around neck ----
    ctx.allow_overlap(
        cap, body,
        elem_a="screw_cap_shell",
        elem_b="body_shell",
        reason="Screw cap bore sleeves over the threaded neck when seated (thread engagement).",
    )
    # Cap footprint stays within body footprint (cap narrower than body)
    ctx.expect_within(
        cap, body, axes="xy",
        elem_a="screw_cap_shell",
        elem_b="body_shell",
        margin=0.002,
        name="cap stays within body footprint",
    )

    # Carrier ring around the neck base
    ctx.allow_overlap(
        carrier, body,
        elem_a="carrier_ring",
        elem_b="body_shell",
        reason="Tamper-evident ring sits on the shoulder around the neck base.",
    )

    # ---- PRISMATIC: cap lifts off when extended ----
    z_rest_min = ctx.part_world_aabb(cap)[0][2]
    with ctx.pose({prismatic: 0.050}):
        z_lift_min = ctx.part_world_aabb(cap)[0][2]
        lifted_ext = _ext(ctx.part_world_aabb(cap))

    ctx.check(
        "cap lifts off when prismatic extended",
        z_lift_min > z_rest_min + 0.04,
        details=f"z_rest_min={z_rest_min}, z_lift_min={z_lift_min}",
    )

    # Fully lifted cap clears the neck top
    ctx.check(
        "lifted cap clears neck top",
        z_lift_min > NECK_TOP_Z - 0.005,
        details=f"z_lift_min={z_lift_min}, neck_top={NECK_TOP_Z}",
    )

    # Cap retains shape when lifted (still a cylinder)
    ctx.check(
        "cap stays cylindrical when lifted",
        abs(lifted_ext[0] - lifted_ext[1]) < 0.006,
        details=f"lifted cap extents={lifted_ext}",
    )

    # ---- CONTINUOUS: cap can spin freely ----
    # At a non-zero rotation, the cap is still at the same height
    with ctx.pose({continuous: 1.57}):
        z_spun = ctx.part_world_aabb(cap)[0][2]
    ctx.check(
        "spinning cap stays at same height",
        abs(z_spun - z_rest_min) < 0.002,
        details=f"z_rest_min={z_rest_min}, z_spun={z_spun}",
    )

    return ctx.report()


object_model = build_object_model()
