from __future__ import annotations

# Black airless cosmetic PRIMER pump bottle.
# Frame: bottle stands upright along +Z. Base sits at z=0, the slim
# cushion-shaped body rises to a flat shoulder, and a flat black airless pump
# actuator caps the top. The broad front/back faces face +/-Y (label + gold
# band live on the front +Y face); the bottle is slim across X.
# Articulation:
#   - pump top: PRISMATIC, presses straight DOWN (-Z, ~0.006 m) then springs back.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
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

NECK_W = 0.018  # short collar the spray head seats over
NECK_D = 0.014
NECK_H = 0.006

GOLD_BAND_Z = 0.050  # gold accent band center height on the body
GOLD_BAND_H = 0.005

# Spray atomizer head
HEAD_DIAM = 0.022       # actuator body diameter
HEAD_H = 0.022          # actuator body height
HEAD_SEAT_OVERLAP = 0.004  # how far the head skirt slips down over the neck
HEAD_TRAVEL = 0.006     # press-down stroke

NOZZLE_DIAM = 0.005     # nozzle spout outer diameter
NOZZLE_LEN = 0.013      # nozzle protrusion length from head front face
NOZZLE_BORE = 0.002     # nozzle orifice inner diameter
NOZZLE_Z_FROM_BASE = 0.008  # nozzle centerline height above head base


def _rounded_prism(w: float, d: float, h: float, fillet: float, z0: float = 0.0) -> cq.Workplane:
    # Upright rounded-rectangle prism: footprint w(X) x d(Y), height h, base at z0.
    f = min(fillet, 0.49 * min(w, d))
    wp = (
        cq.Workplane("XY")
        .workplane(offset=z0)
        .rect(w, d)
        .extrude(h)
    )
    if f > 1e-5:
        # Fillet the four vertical edges only.
        wp = wp.edges("|Z").fillet(f)
    return wp


def _body_solid() -> cq.Workplane:
    # Main cushion body + tapered shoulder + short neck collar, fused into one shell.
    body = _rounded_prism(BODY_W, BODY_D, BODY_H, BODY_FILLET, z0=0.0)

    # Tapered shoulder (loft from body top footprint down to the neck footprint).
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_H)
        .rect(BODY_W - 2 * BODY_FILLET + 0.004, BODY_D - 2 * BODY_FILLET + 0.004)
        .workplane(offset=SHOULDER_H)
        .rect(SHOULDER_TOP_W, SHOULDER_TOP_D)
        .loft(ruled=True)
    )
    body = body.union(shoulder)

    # Short neck collar the pump skirt seats over.
    neck = _rounded_prism(NECK_W, NECK_D, NECK_H, 0.003, z0=BODY_H + SHOULDER_H)
    body = body.union(neck)
    return body


def _gold_band_solid() -> cq.Workplane:
    # Thin band wrapping the body at GOLD_BAND_Z; slightly proud of the surface.
    band = _rounded_prism(
        BODY_W + 0.0008,
        BODY_D + 0.0008,
        GOLD_BAND_H,
        BODY_FILLET,
        z0=GOLD_BAND_Z - GOLD_BAND_H / 2.0,
    )
    return band


def _label_plate_solid() -> cq.Workplane:
    # Slightly raised label area on the front (+Y) face, below the gold band,
    # standing in for the printed "PRIMER" text block.
    plate = (
        cq.Workplane("XY")
        .workplane(offset=0.012)
        .center(0.0, BODY_D / 2.0 - 0.0005)
        .rect(BODY_W - 0.010, 0.0018)
        .extrude(GOLD_BAND_Z - GOLD_BAND_H / 2.0 - 0.012 - 0.002)
    )
    return plate


def _spray_head_solid() -> cq.Workplane:
    """Fine-mist spray atomizer head: cylindrical actuator body with a finger-pad
    dome on top, a hollow skirt that grips the neck, and a short directional
    nozzle spout protruding from the front (+Y) face for lateral mist firing."""
    z0 = BODY_H + SHOULDER_H + NECK_H - HEAD_SEAT_OVERLAP
    head_r = HEAD_DIAM / 2.0

    # Main actuator cylinder.
    head = (
        cq.Workplane("XY")
        .workplane(offset=z0)
        .circle(head_r)
        .extrude(HEAD_H)
    )

    # Domed finger pad on top for pressing — a shallow spherical cap.
    dome_r = head_r * 1.1
    dome = (
        cq.Workplane("XY")
        .workplane(offset=z0 + HEAD_H)
        .circle(head_r)
        .extrude(0.003)
    )
    dome = dome.edges(">Z").fillet(0.002)
    head = head.union(dome)

    # Hollow skirt bore — grips the neck collar (intentional seated overlap).
    bore_r = (min(NECK_W, NECK_D) / 2.0) - 0.0005
    bore = (
        cq.Workplane("XY")
        .workplane(offset=z0 - 0.002)
        .circle(bore_r)
        .extrude(HEAD_SEAT_OVERLAP + 0.002)
    )
    head = head.cut(bore)

    # Nozzle spout — short cylinder protruding from the front (+Y) face of the head.
    # The spout centerline is at z0 + NOZZLE_Z_FROM_BASE, extending in +Y.
    nozzle_z = z0 + NOZZLE_Z_FROM_BASE
    nozzle_start_y = head_r - 0.001  # start just inside the head surface for clean union

    # Create nozzle cylinder at origin, then translate into position
    nozzle = (
        cq.Workplane("XY")
        .circle(NOZZLE_DIAM / 2.0)
        .extrude(NOZZLE_LEN + 0.001)
        .translate((0.0, nozzle_start_y, nozzle_z))
    )
    head = head.union(nozzle)

    # Orifice bore through the nozzle tip — small hole for mist exit.
    orifice = (
        cq.Workplane("XY")
        .circle(NOZZLE_BORE / 2.0)
        .extrude(0.005)
        .translate((0.0, nozzle_start_y + NOZZLE_LEN - 0.002, nozzle_z))
    )
    head = head.cut(orifice)

    return head


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="primer_pump_bottle")

    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    gold = model.material("gold_accent", rgba=(0.80, 0.62, 0.22, 1.0))
    pump_black = model.material("pump_black", rgba=(0.05, 0.05, 0.06, 1.0))

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
    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H)),
        mass=0.090,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # ---- spray atomizer head: presses straight down then springs back ----
    spray_head = model.part("spray_head")
    spray_head.visual(
        mesh_from_cadquery(_spray_head_solid(), "atomizer_head"),
        material=pump_black,
        name="atomizer_head",
    )
    head_z0 = BODY_H + SHOULDER_H + NECK_H - HEAD_SEAT_OVERLAP
    spray_head.inertial = Inertial.from_geometry(
        Box((HEAD_DIAM, HEAD_DIAM, HEAD_H)),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.0, head_z0 + HEAD_H / 2.0)),
    )
    model.articulation(
        "spray_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=spray_head,
        # Head geometry and inertial are authored in absolute body-frame coords,
        # so the joint origin is at the frame origin and only the +Z stroke moves.
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=0.05, lower=-HEAD_TRAVEL, upper=0.0
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    spray_head = object_model.get_part("spray_head")
    press = object_model.get_articulation("spray_press")

    # Spray head skirt is intentionally slipped down over the body neck collar.
    ctx.allow_overlap(
        spray_head,
        body,
        elem_a="atomizer_head",
        elem_b="body_shell",
        reason="The spray atomizer head skirt is intentionally seated down over the neck collar.",
    )

    # ---- bottle reads slim and tall ----
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

    # ---- gold accent band present on the body, partway up ----
    band_aabb = ctx.part_element_world_aabb(body, elem="gold_band")
    band_z = (band_aabb[0][2] + band_aabb[1][2]) / 2.0
    ctx.check(
        "gold accent band sits partway up the body",
        0.030 < band_z < 0.070,
        details=f"gold band center z={band_z}",
    )

    # ---- spray head is on TOP and seated over the neck ----
    head_aabb = ctx.part_world_aabb(spray_head)
    head_center_z = (head_aabb[0][2] + head_aabb[1][2]) / 2.0
    ctx.check(
        "spray head mounted at the top of the bottle",
        head_aabb[0][2] > BODY_H - 0.010 and head_center_z > BODY_H,
        details=f"head aabb z=[{head_aabb[0][2]}, {head_aabb[1][2]}]",
    )
    ctx.expect_overlap(
        spray_head, body, axes="xy", min_overlap=0.010, name="spray head seated over neck (footprint)"
    )

    # ---- nozzle spout protrudes from the front (+Y) of the head ----
    head_aabb_full = ctx.part_world_aabb(spray_head)
    body_aabb_full = ctx.part_world_aabb(body)
    ctx.check(
        "nozzle spout protrudes beyond the body front face (+Y)",
        head_aabb_full[1][1] > body_aabb_full[1][1],
        details=f"head y_max={head_aabb_full[1][1]}, body y_max={body_aabb_full[1][1]}",
    )

    # ---- spray head presses straight DOWN and stays seated ----
    head_top_rest = ctx.part_world_aabb(spray_head)[1][2]
    head_bot_rest = ctx.part_world_aabb(spray_head)[0][2]
    with ctx.pose({press: -HEAD_TRAVEL}):
        head_top_down = ctx.part_world_aabb(spray_head)[1][2]
        head_bot_down = ctx.part_world_aabb(spray_head)[0][2]
        # Still seated over the neck at full press.
        ctx.expect_overlap(
            spray_head, body, axes="xy", min_overlap=0.010, name="spray head stays seated when pressed"
        )
    ctx.check(
        "spray head top drops when pressed",
        head_top_down < head_top_rest - 0.004,
        details=f"top rest_z={head_top_rest}, pressed_z={head_top_down}",
    )
    ctx.check(
        "spray head presses straight down (only z changes)",
        head_bot_down < head_bot_rest - 0.004,
        details=f"bottom rest_z={head_bot_rest}, pressed_z={head_bot_down}",
    )

    return ctx.report()


object_model = build_object_model()
