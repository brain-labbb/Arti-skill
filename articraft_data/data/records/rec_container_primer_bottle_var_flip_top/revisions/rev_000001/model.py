from __future__ import annotations

# Black airless cosmetic PRIMER bottle with hinged snap flip-top disc cap.
# Frame: bottle stands upright along +Z. Base sits at z=0, the slim
# cushion-shaped body rises to a flat shoulder, and a low cylindrical cap
# shell seats over the neck. A circular flip disc lid hinges open about a
# horizontal axis (+X) along the back edge via a REVOLUTE joint (living-hinge
# disc that flips up to reveal the dispense orifice).
# The broad front/back faces face +/-Y (label + gold band on +Y face).
# Articulation:
#   - cap_hinge: REVOLUTE, horizontal +X axis at back edge of cap top,
#     positive q opens the disc upward to reveal the orifice.

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
BODY_W = 0.034  # across X (slim)
BODY_D = 0.024  # front-to-back along Y
BODY_H = 0.086  # body height
BODY_FILLET = 0.008  # rounded vertical edges -> cushion cross-section

SHOULDER_H = 0.006  # tapered shoulder above the main body
SHOULDER_TOP_W = 0.020
SHOULDER_TOP_D = 0.016

NECK_W = 0.018  # short collar the cap seats over
NECK_D = 0.014
NECK_H = 0.006

GOLD_BAND_Z = 0.050  # gold accent band center height on the body
GOLD_BAND_H = 0.005

# Cap base (low shell seating over the neck, inline on body)
CAP_RADIUS = 0.012  # outer radius of cylindrical cap shell
CAP_H = 0.007  # cap base total height
CAP_SEAT_OVERLAP = 0.003  # how far the cap skirt slips down over the neck

# Flip disc cap
FLIP_RADIUS = 0.011  # flip disc radius (slightly smaller than cap)
FLIP_THICKNESS = 0.0025  # flip disc thickness
FLIP_OPEN_ANGLE = 1.8  # ~103 degrees max open

# Derived positions
NECK_TOP_Z = BODY_H + SHOULDER_H + NECK_H  # 0.098
CAP_BASE_Z0 = NECK_TOP_Z - CAP_SEAT_OVERLAP  # 0.095
CAP_TOP_Z = CAP_BASE_Z0 + CAP_H  # 0.102


def _rounded_prism(w: float, d: float, h: float, fillet: float, z0: float = 0.0) -> cq.Workplane:
    """Upright rounded-rectangle prism: footprint w(X) x d(Y), height h, base at z0."""
    f = min(fillet, 0.49 * min(w, d))
    wp = (
        cq.Workplane("XY")
        .workplane(offset=z0)
        .rect(w, d)
        .extrude(h)
    )
    if f > 1e-5:
        wp = wp.edges("|Z").fillet(f)
    return wp


def _body_solid() -> cq.Workplane:
    """Main cushion body + tapered shoulder + short neck collar, fused."""
    body = _rounded_prism(BODY_W, BODY_D, BODY_H, BODY_FILLET, z0=0.0)
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_H)
        .rect(BODY_W - 2 * BODY_FILLET + 0.004, BODY_D - 2 * BODY_FILLET + 0.004)
        .workplane(offset=SHOULDER_H)
        .rect(SHOULDER_TOP_W, SHOULDER_TOP_D)
        .loft(ruled=True)
    )
    body = body.union(shoulder)
    neck = _rounded_prism(NECK_W, NECK_D, NECK_H, 0.003, z0=BODY_H + SHOULDER_H)
    body = body.union(neck)
    return body


def _gold_band_solid() -> cq.Workplane:
    """Thin band wrapping the body at GOLD_BAND_Z; slightly proud."""
    return _rounded_prism(
        BODY_W + 0.0008,
        BODY_D + 0.0008,
        GOLD_BAND_H,
        BODY_FILLET,
        z0=GOLD_BAND_Z - GOLD_BAND_H / 2.0,
    )


def _label_plate_solid() -> cq.Workplane:
    """Slightly raised label area on the front (+Y) face."""
    return (
        cq.Workplane("XY")
        .workplane(offset=0.012)
        .center(0.0, BODY_D / 2.0 - 0.0005)
        .rect(BODY_W - 0.010, 0.0018)
        .extrude(GOLD_BAND_Z - GOLD_BAND_H / 2.0 - 0.012 - 0.002)
    )


def _cap_base_solid() -> cq.Workplane:
    """Low cylindrical cap shell seating over the neck with a small dispense
    orifice well on the flat top plate."""
    # Outer cylindrical shell
    shell = (
        cq.Workplane("XY")
        .workplane(offset=CAP_BASE_Z0)
        .circle(CAP_RADIUS)
        .extrude(CAP_H)
    )
    # Thin snap-fit ridge ring around the lower exterior
    ridge = (
        cq.Workplane("XY")
        .workplane(offset=CAP_BASE_Z0 + 0.001)
        .circle(CAP_RADIUS + 0.0006)
        .circle(CAP_RADIUS)
        .extrude(0.0012)
    )
    shell = shell.union(ridge)
    # Small dispense orifice well cut into the top plate
    well = (
        cq.Workplane("XY")
        .workplane(offset=CAP_TOP_Z - 0.002)
        .circle(0.0025)
        .extrude(0.003)
    )
    shell = shell.cut(well)
    return shell


def _flip_disc_solid() -> cq.Workplane:
    """Circular flip disc lid in the flip_cap part frame.

    Part frame origin is at the hinge point (back edge of cap top).
    The disc extends forward (+Y) from the hinge and lies flat at q=0.
    """
    # Disc center in local frame: forward by CAP_RADIUS from hinge,
    # sitting on the cap top surface
    disc_cy = CAP_RADIUS
    disc = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .center(0.0, disc_cy)
        .circle(FLIP_RADIUS)
        .extrude(FLIP_THICKNESS)
    )
    # Thumb-lift tab at the front edge
    tab_cy = disc_cy + FLIP_RADIUS - 0.002
    tab = (
        cq.Workplane("XY")
        .workplane(offset=-0.0004)
        .center(0.0, tab_cy)
        .rect(0.005, 0.005)
        .extrude(FLIP_THICKNESS + 0.0008)
    )
    disc = disc.union(tab)
    # Small sealing plug on the underside (fits into the orifice well)
    plug = (
        cq.Workplane("XY")
        .workplane(offset=-0.0015)
        .center(0.0, disc_cy)
        .circle(0.0022)
        .extrude(0.0018)
    )
    disc = disc.union(plug)
    return disc


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="primer_fliptop_bottle")

    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    gold = model.material("gold_accent", rgba=(0.80, 0.62, 0.22, 1.0))
    cap_black = model.material("cap_black", rgba=(0.06, 0.06, 0.07, 1.0))
    disc_black = model.material("disc_black", rgba=(0.04, 0.04, 0.05, 1.0))

    # ---- bottle body (root) ----
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_solid(), "body_shell"),
        material=matte_black,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_gold_band_solid(), "gold_band"),
        material=gold,
        name="gold_band",
    )
    body.visual(
        mesh_from_cadquery(_label_plate_solid(), "label_plate"),
        material=gold,
        name="label_plate",
    )
    # Cap base is a non-moving decoration seated over the neck (inline on body)
    body.visual(
        mesh_from_cadquery(_cap_base_solid(), "cap_base"),
        material=cap_black,
        name="cap_base",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H)),
        mass=0.090,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # ---- flip disc cap: hinges open about horizontal axis at back edge ----
    flip_cap = model.part("flip_cap")
    flip_cap.visual(
        mesh_from_cadquery(_flip_disc_solid(), "flip_disc"),
        material=disc_black,
        name="flip_disc",
    )
    flip_cap.inertial = Inertial.from_geometry(
        Cylinder(radius=FLIP_RADIUS, length=FLIP_THICKNESS),
        mass=0.005,
        origin=Origin(xyz=(0.0, CAP_RADIUS, FLIP_THICKNESS / 2.0)),
    )

    model.articulation(
        "cap_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flip_cap,
        # Hinge at back edge of cap top surface
        origin=Origin(xyz=(0.0, -CAP_RADIUS, CAP_TOP_Z)),
        axis=(1.0, 0.0, 0.0),  # horizontal +X axis
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0, lower=0.0, upper=FLIP_OPEN_ANGLE
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    flip_cap = object_model.get_part("flip_cap")
    hinge = object_model.get_articulation("cap_hinge")

    # ---- body still reads slim and tall (same cushion body as parent) ----
    body_ext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "body is tall (height dominates footprint)",
        body_ext[2] > 0.070 and body_ext[2] > body_ext[0] + 0.030 and body_ext[2] > body_ext[1] + 0.030,
        details=f"body extents={body_ext}",
    )
    ctx.check(
        "body is slim",
        body_ext[0] < 0.045 and body_ext[1] < 0.045,
        details=f"body extents={body_ext}",
    )

    # ---- gold accent band still present ----
    band_aabb = ctx.part_element_world_aabb(body, elem="gold_band")
    band_z = (band_aabb[0][2] + band_aabb[1][2]) / 2.0
    ctx.check(
        "gold accent band sits partway up the body",
        0.030 < band_z < 0.070,
        details=f"gold band center z={band_z}",
    )

    # ---- cap base is present at the top of the bottle ----
    cap_aabb = ctx.part_element_world_aabb(body, elem="cap_base")
    ctx.check(
        "cap base mounted at the top of the bottle",
        cap_aabb[0][2] > BODY_H - 0.010,
        details=f"cap_base aabb z=[{cap_aabb[0][2]:.4f}, {cap_aabb[1][2]:.4f}]",
    )

    # ---- flip disc is circular and mounted on top ----
    flip_aabb = ctx.part_world_aabb(flip_cap)
    flip_dx = flip_aabb[1][0] - flip_aabb[0][0]
    flip_dy = flip_aabb[1][1] - flip_aabb[0][1]
    ctx.check(
        "flip disc is roughly circular (aspect near 1)",
        abs(flip_dx - flip_dy) < 0.010,
        details=f"flip dx={flip_dx:.4f}, dy={flip_dy:.4f}",
    )
    ctx.check(
        "flip disc is mounted at top of bottle",
        flip_aabb[0][2] > BODY_H,
        details=f"flip aabb z=[{flip_aabb[0][2]:.4f}, {flip_aabb[1][2]:.4f}]",
    )

    # ---- hinge axis is horizontal (along X) ----
    ctx.check(
        "hinge axis is horizontal (+X)",
        abs(hinge.axis[0]) > 0.9 and abs(hinge.axis[2]) < 0.1,
        details=f"hinge axis={hinge.axis}",
    )

    # ---- flip cap opens upward with positive q ----
    flip_center_rest = (flip_aabb[0][2] + flip_aabb[1][2]) / 2.0
    flip_front_rest = flip_aabb[1][1]  # front (+Y) edge at rest

    with ctx.pose({hinge: FLIP_OPEN_ANGLE}):
        flip_aabb_open = ctx.part_world_aabb(flip_cap)
        flip_center_open = (flip_aabb_open[0][2] + flip_aabb_open[1][2]) / 2.0
        flip_top_open = flip_aabb_open[1][2]

    ctx.check(
        "flip cap opens upward (center rises at open pose)",
        flip_center_open > flip_center_rest + 0.003,
        details=f"rest center z={flip_center_rest:.4f}, open center z={flip_center_open:.4f}",
    )
    ctx.check(
        "flip cap top rises significantly when opened",
        flip_top_open > flip_aabb[1][2] + 0.005,
        details=f"rest top z={flip_aabb[1][2]:.4f}, open top z={flip_top_open:.4f}",
    )

    # ---- at rest, flip disc covers the cap base orifice area (XY overlap) ----
    ctx.expect_overlap(
        flip_cap, body, axes="xy", min_overlap=0.008,
        elem_a="flip_disc", elem_b="cap_base",
        name="flip disc covers cap base when closed",
    )

    # ---- sealing plug seats into the cap base (small intentional embed) ----
    ctx.allow_overlap(
        flip_cap,
        body,
        elem_a="flip_disc",
        elem_b="cap_base",
        reason="The flip disc sealing plug intentionally seats into the cap base orifice well.",
    )
    # Prove the plug is seated (Z contact at the interface)
    ctx.expect_gap(
        flip_cap, body, axis="z",
        positive_elem="flip_disc", negative_elem="cap_base",
        max_penetration=0.003, max_gap=0.001,
        name="flip disc sits on cap base (seated contact)",
    )

    return ctx.report()


object_model = build_object_model()
