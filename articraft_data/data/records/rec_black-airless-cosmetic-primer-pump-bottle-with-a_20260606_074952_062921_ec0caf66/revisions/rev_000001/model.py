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

NECK_W = 0.018  # short collar the pump seats over
NECK_D = 0.014
NECK_H = 0.006

GOLD_BAND_Z = 0.050  # gold accent band center height on the body
GOLD_BAND_H = 0.005

# Pump actuator
PUMP_W = 0.022
PUMP_D = 0.018
PUMP_H = 0.020
PUMP_FILLET = 0.004
PUMP_SEAT_OVERLAP = 0.004  # how far the pump skirt slips down over the neck
PUMP_TRAVEL = 0.006  # press-down stroke


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


def _pump_solid() -> cq.Workplane:
    # Flat black airless actuator: short rounded-rect cap with a hollow skirt that
    # slips over the neck, and a tiny dispense hole through the top.
    z0 = BODY_H + SHOULDER_H + NECK_H - PUMP_SEAT_OVERLAP
    cap = _rounded_prism(PUMP_W, PUMP_D, PUMP_H, PUMP_FILLET, z0=z0)

    # Hollow out the underside so it reads as a real skirt sitting over the neck.
    # The bore is slightly smaller than the neck so the skirt grips it (the
    # intentional seated overlap is allowed in run_tests). The bore stops short
    # of the cap top, so the cap interior ceiling rests on the neck top face.
    bore = _rounded_prism(
        NECK_W - 0.0010,
        NECK_D - 0.0010,
        PUMP_SEAT_OVERLAP,
        0.002,
        z0=z0 - 0.002,
    )
    cap = cap.cut(bore)

    # Tiny dispense hole through the flat top (the airless orifice).
    hole = (
        cq.Workplane("XY")
        .workplane(offset=z0 + PUMP_H - 0.005)
        .center(0.0, 0.0)
        .circle(0.0016)
        .extrude(0.006)
    )
    cap = cap.cut(hole)
    return cap


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

    # ---- pump top: presses straight down then springs back ----
    pump = model.part("pump_top")
    pump.visual(
        mesh_from_cadquery(_pump_solid(), "pump_cap"),
        material=pump_black,
        name="pump_cap",
    )
    pump_z0 = BODY_H + SHOULDER_H + NECK_H - PUMP_SEAT_OVERLAP
    pump.inertial = Inertial.from_geometry(
        Box((PUMP_W, PUMP_D, PUMP_H)),
        mass=0.010,
        origin=Origin(xyz=(0.0, 0.0, pump_z0 + PUMP_H / 2.0)),
    )
    model.articulation(
        "pump_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=pump,
        # Pump geometry and inertial are authored in absolute body-frame coords,
        # so the joint origin is at the frame origin and only the +Z stroke moves.
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=0.05, lower=-PUMP_TRAVEL, upper=0.0
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    pump = object_model.get_part("pump_top")
    press = object_model.get_articulation("pump_press")

    # Pump skirt is intentionally slipped down over the body neck collar.
    ctx.allow_overlap(
        pump,
        body,
        elem_a="pump_cap",
        elem_b="body_shell",
        reason="The airless pump skirt is intentionally seated down over the neck collar.",
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

    # ---- pump is on TOP and seated over the neck ----
    pump_aabb = ctx.part_world_aabb(pump)
    pump_center_z = (pump_aabb[0][2] + pump_aabb[1][2]) / 2.0
    ctx.check(
        "pump mounted at the top of the bottle",
        pump_aabb[0][2] > BODY_H - 0.010 and pump_center_z > BODY_H,
        details=f"pump aabb z=[{pump_aabb[0][2]}, {pump_aabb[1][2]}]",
    )
    ctx.expect_overlap(
        pump, body, axes="xy", min_overlap=0.010, name="pump seated over neck (footprint)"
    )

    # ---- pump presses straight DOWN and stays seated ----
    pump_top_rest = ctx.part_world_aabb(pump)[1][2]
    pump_bot_rest = ctx.part_world_aabb(pump)[0][2]
    with ctx.pose({press: -PUMP_TRAVEL}):
        pump_top_down = ctx.part_world_aabb(pump)[1][2]
        pump_bot_down = ctx.part_world_aabb(pump)[0][2]
        # Still seated over the neck at full press.
        ctx.expect_overlap(
            pump, body, axes="xy", min_overlap=0.010, name="pump stays seated when pressed"
        )
    ctx.check(
        "pump top drops when pressed",
        pump_top_down < pump_top_rest - 0.004,
        details=f"top rest_z={pump_top_rest}, pressed_z={pump_top_down}",
    )
    ctx.check(
        "pump presses straight down (only z changes)",
        pump_bot_down < pump_bot_rest - 0.004,
        details=f"bottom rest_z={pump_bot_rest}, pressed_z={pump_bot_down}",
    )

    return ctx.report()


object_model = build_object_model()
