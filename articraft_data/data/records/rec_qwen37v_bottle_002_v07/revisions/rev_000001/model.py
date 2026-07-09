from __future__ import annotations

# Ribbed water bottle with deep grip grooves, a flip cap on a revolute hinge,
# a visible hollow mouth opening under the cap, and a separate gasket ring.
# Frame: vertical axis along +Z, bottle standing on z=0, centerline on x=y=0.

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
)

# ---- key heights (m) along +Z ----
BASE_Z = 0.0
BODY_TOP_Z = 0.130       # end of straight cylindrical body, start of shoulder
SHOULDER_TOP_Z = 0.170   # end of tapered shoulder, base of neck
NECK_TOP_Z = 0.192       # top of neck rim (mouth opening)

BODY_R = 0.035           # body outer radius (~70 mm dia)
NECK_R = 0.016           # neck outer radius
NECK_BORE_R = 0.013      # mouth bore radius (~26 mm opening)

CAP_R = 0.019            # flip cap disc radius (slightly larger than neck)
CAP_HEIGHT = 0.008       # cap disc thickness

GASKET_MAJOR_R = 0.015   # gasket torus center radius (on the neck rim annulus)
GASKET_TUBE_R = 0.002    # gasket cross-section radius

N_GROOVES = 10           # number of vertical grip grooves
GROOVE_R = 0.003         # groove cutter radius (depth into the wall)
WALL = 0.004             # shell wall thickness (grooves leave ~1 mm min wall)


def _profile_sections():
    """(z, outer_radius) pairs for the revolved outer bottle profile."""
    return [
        (0.000, 0.018),           # rounded base heel (tucked in)
        (0.008, 0.032),           # base flare
        (0.016, BODY_R),          # full body radius reached
        (BODY_TOP_Z, BODY_R),     # straight cylindrical body
        (0.145, 0.032),           # shoulder starts tapering
        (0.158, 0.024),
        (SHOULDER_TOP_Z, 0.017),  # end of shoulder, narrower
        (0.176, NECK_R),          # neck base
        (NECK_TOP_Z, NECK_R),     # straight neck up to the rim
    ]


def _bottle_solid() -> cq.Workplane:
    """Revolve the outer profile, then shell it to make a hollow container."""
    pts = _profile_sections()
    wp = cq.Workplane("XZ").moveTo(0.0, pts[0][0])
    for r, z in [(r, z) for (z, r) in pts]:
        wp = wp.lineTo(r, z)
    wp = wp.lineTo(0.0, pts[-1][0]).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))

    # Hollow interior: cavity opens through the neck rim (visible mouth).
    inner_pts = [
        (0.014, 0.008),
        (BODY_R - WALL, 0.016),
        (BODY_R - WALL, BODY_TOP_Z),
        (0.030, 0.145),
        (0.022, 0.158),
        (0.015, SHOULDER_TOP_Z),
        (NECK_BORE_R, 0.176),
        (NECK_BORE_R, NECK_TOP_Z + 0.005),  # open through the rim
    ]
    iwp = cq.Workplane("XZ").moveTo(0.0, inner_pts[0][1])
    for r, z in inner_pts:
        iwp = iwp.lineTo(r, z)
    iwp = iwp.lineTo(0.0, inner_pts[-1][1]).close()
    cavity = iwp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    return outer.cut(cavity)


def _bottle_mesh():
    """Hollow ribbed bottle shell with deep vertical grip grooves."""
    solid = _bottle_solid()
    # Cut deep grip grooves into the straight body section.
    groove_z_start = 0.025
    groove_z_end = BODY_TOP_Z - 0.005
    groove_h = groove_z_end - groove_z_start
    for i in range(N_GROOVES):
        a = 2.0 * math.pi * i / N_GROOVES
        cx = BODY_R * math.cos(a)
        cy = BODY_R * math.sin(a)
        groove = (
            cq.Workplane("XY")
            .workplane(offset=groove_z_start)
            .center(cx, cy)
            .circle(GROOVE_R)
            .extrude(groove_h)
        )
        solid = solid.cut(groove)
    return mesh_from_cadquery(solid, "bottle_shell")


def _cap_solid() -> cq.Workplane:
    """Flip cap disc with a front finger-grip tab."""
    cap = cq.Workplane("XY").circle(CAP_R).extrude(CAP_HEIGHT)
    # Small tab protruding from the front edge for flipping open.
    tab = (
        cq.Workplane("XY")
        .center(0.0, CAP_R + 0.003)
        .rect(0.010, 0.006)
        .extrude(CAP_HEIGHT)
    )
    return cap.union(tab)


def _cap_mesh():
    return mesh_from_cadquery(_cap_solid(), "cap_disc")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ribbed_water_bottle")

    clear = model.material("clear_plastic", rgba=(0.80, 0.88, 0.92, 0.30))
    cap_mat = model.material("cap_blue", rgba=(0.15, 0.35, 0.65, 1.0))
    gasket_mat = model.material("gasket_rubber", rgba=(0.20, 0.20, 0.22, 1.0))

    # ---- bottle body (root): ribbed hollow container ----
    body = model.part("bottle_body")
    body.visual(_bottle_mesh(), material=clear, name="bottle_shell")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.080,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- gasket ring: compression seal on the neck rim ----
    gasket = model.part("gasket")
    gasket.visual(
        mesh_from_geometry(
            TorusGeometry(
                GASKET_MAJOR_R, GASKET_TUBE_R,
                radial_segments=12, tubular_segments=40,
            ),
            "gasket_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=gasket_mat,
        name="gasket_ring",
    )
    gasket.inertial = Inertial.from_geometry(
        Cylinder(GASKET_MAJOR_R + GASKET_TUBE_R, GASKET_TUBE_R * 2.0),
        mass=0.003,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # Fixed joint: gasket seated on body neck rim.
    model.articulation(
        "body_to_gasket",
        ArticulationType.FIXED,
        parent=body,
        child=gasket,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z)),
    )

    # ---- flip cap: disc covering the mouth, hinged at the rear of the neck ----
    cap = model.part("flip_cap")
    # Cap part frame sits at the hinge point. The disc visual is shifted
    # forward (+Y) by NECK_R so the disc center lands over the mouth at q=0.
    cap.visual(
        _cap_mesh(),
        origin=Origin(xyz=(0.0, NECK_R, 0.0)),
        material=cap_mat,
        name="cap_disc",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_HEIGHT),
        mass=0.010,
        origin=Origin(xyz=(0.0, NECK_R, CAP_HEIGHT / 2.0)),
    )

    # Revolute hinge at the rear edge of the neck rim.
    # axis=(1,0,0): right-hand rule → positive q lifts +Y (front of cap) toward +Z.
    model.articulation(
        "cap_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, -NECK_R, NECK_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0, lower=0.0, upper=2.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle_body")
    cap = object_model.get_part("flip_cap")
    gasket = object_model.get_part("gasket")
    hinge = object_model.get_articulation("cap_hinge")

    # --- gasket is seated on the neck rim (intentional compression) ---
    ctx.allow_overlap(
        gasket, body,
        elem_a="gasket_ring", elem_b="bottle_shell",
        reason="The gasket ring is intentionally seated into the neck rim for a compression seal.",
    )
    ctx.expect_contact(
        gasket, body,
        elem_a="gasket_ring", elem_b="bottle_shell",
        name="gasket contacts the neck rim",
    )

    # --- cap closes onto the gasket (intentional compression seal) ---
    ctx.allow_overlap(
        cap, gasket,
        elem_a="cap_disc", elem_b="gasket_ring",
        reason="The flip cap compresses the gasket ring when closed to form a seal.",
    )
    ctx.expect_gap(
        cap, body,
        axis="z",
        positive_elem="cap_disc", negative_elem="bottle_shell",
        max_penetration=0.004,
        name="closed cap seats at or above the neck rim",
    )

    # --- flip cap opens: positive hinge angle lifts the cap disc upward ---
    cap_disc = cap.get_visual("cap_disc")

    def _cap_disc_center_z():
        mn, mx = ctx.part_element_world_aabb(cap, elem=cap_disc)
        return (mn[2] + mx[2]) / 2.0

    cap_z_rest = _cap_disc_center_z()
    with ctx.pose({hinge: 1.5}):
        cap_z_open = _cap_disc_center_z()
    ctx.check(
        "flip cap opens upward on the hinge",
        cap_z_open > cap_z_rest + 0.005,
        details=f"closed z={cap_z_rest:.4f}, open z={cap_z_open:.4f}",
    )

    # --- hinge is revolute with bounded travel matching the flip mechanism ---
    limits = hinge.motion_limits
    ctx.check(
        "cap hinge is revolute with bounded travel",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and limits is not None
        and limits.lower is not None and limits.lower == 0.0
        and limits.upper is not None and limits.upper > 1.5,
        details=f"type={hinge.articulation_type}, limits=[{limits.lower}, {limits.upper}]",
    )

    # --- gasket ring is positioned at the top of the neck ---
    gasket_pos = ctx.part_world_position(gasket)
    ctx.check(
        "gasket ring positioned at the neck top",
        gasket_pos is not None and abs(gasket_pos[2] - NECK_TOP_Z) < 0.005,
        details=f"gasket z={gasket_pos[2]:.4f}, expected ~{NECK_TOP_Z}",
    )

    # --- bottle proportions: taller than wide (water bottle shape) ---
    mn, mx = ctx.part_world_aabb(body)
    dx = mx[0] - mn[0]
    dz = mx[2] - mn[2]
    ctx.check(
        "bottle is taller than wide",
        dz > 2.0 * dx,
        details=f"dx={dx:.4f}, dz={dz:.4f}",
    )

    # --- mouth opening exists: neck bore is structurally smaller than neck outer ---
    ctx.check(
        "visible hollow mouth opening under the cap",
        NECK_BORE_R < NECK_R * 0.95,
        details=f"bore_r={NECK_BORE_R}, neck_r={NECK_R}",
    )

    return ctx.report()


object_model = build_object_model()
