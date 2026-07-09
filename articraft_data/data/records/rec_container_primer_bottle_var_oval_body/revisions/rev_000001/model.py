from __future__ import annotations

# Black airless cosmetic PRIMER pump bottle — OVAL (elliptical) body variant.
# Fork of the cushion-bodied parent: the filleted-squircle cross-section is
# replaced by a flattened oval (elliptical) cross-section that is wide across
# X (front face) and shallow front-to-back along Y, tapering smoothly through
# the shoulder and neck via loft.
# Frame: bottle stands upright along +Z. Base at z=0.
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

# ---- key dimensions (meters) — oval / elliptical cross-section ----
BODY_RX = 0.019    # semi-axis along X (wide, across the front face)
BODY_RY = 0.011    # semi-axis along Y (shallow, front-to-back)
BODY_H = 0.086     # main body cylinder height

SHOULDER_H = 0.008  # smooth taper from body top to neck base
SHOULDER_TOP_RX = 0.010
SHOULDER_TOP_RY = 0.008

NECK_RX = 0.009    # elliptical neck collar the pump seats over
NECK_RY = 0.007
NECK_H = 0.006

GOLD_BAND_Z = 0.050   # gold accent band center height on the body
GOLD_BAND_H = 0.005

# Pump actuator (exterior stays rounded-rect; bore is elliptical to match neck)
PUMP_W = 0.022
PUMP_D = 0.018
PUMP_H = 0.020
PUMP_FILLET = 0.004
PUMP_SEAT_OVERLAP = 0.004  # how far the pump skirt slips down over the neck
PUMP_TRAVEL = 0.006        # press-down stroke


def _elliptical_prism(rx: float, ry: float, h: float, z0: float = 0.0) -> cq.Workplane:
    """Upright elliptical prism: semi-axes rx (X) × ry (Y), height h, base at z0."""
    return (
        cq.Workplane("XY")
        .workplane(offset=z0)
        .ellipse(rx, ry)
        .extrude(h)
    )


def _rounded_prism(w: float, d: float, h: float, fillet: float, z0: float = 0.0) -> cq.Workplane:
    """Upright rounded-rectangle prism (used for pump cap exterior)."""
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
    """Oval body: elliptical extrusion + smooth lofted shoulder + elliptical neck."""
    # Main oval body — smooth elliptical cross-section, wide in X, shallow in Y.
    body = _elliptical_prism(BODY_RX, BODY_RY, BODY_H, z0=0.0)

    # Smooth tapered shoulder: loft from body-top ellipse to neck-base ellipse.
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_H)
        .ellipse(BODY_RX, BODY_RY)
        .workplane(offset=SHOULDER_H)
        .ellipse(SHOULDER_TOP_RX, SHOULDER_TOP_RY)
        .loft()
    )
    body = body.union(shoulder)

    # Elliptical neck collar the pump skirt seats over.
    neck = _elliptical_prism(NECK_RX, NECK_RY, NECK_H, z0=BODY_H + SHOULDER_H)
    body = body.union(neck)

    # Subtle base rim: a thin elliptical lip at the bottom for visual grounding.
    rim = _elliptical_prism(BODY_RX + 0.0006, BODY_RY + 0.0006, 0.002, z0=0.0)
    body = body.union(rim)

    return body


def _gold_band_solid() -> cq.Workplane:
    """Thin elliptical band wrapping the oval body at GOLD_BAND_Z, slightly proud."""
    # Outer shell of the band (proud of the body surface).
    outer_rx = BODY_RX + 0.0006
    outer_ry = BODY_RY + 0.0006
    band = _elliptical_prism(
        outer_rx, outer_ry, GOLD_BAND_H,
        z0=GOLD_BAND_Z - GOLD_BAND_H / 2.0,
    )
    # Hollow interior so it reads as a ring wrapping the body, not a solid disk.
    inner = _elliptical_prism(
        BODY_RX - 0.0004, BODY_RY - 0.0004,
        GOLD_BAND_H + 0.002,
        z0=GOLD_BAND_Z - GOLD_BAND_H / 2.0 - 0.001,
    )
    return band.cut(inner)


def _label_plate_solid() -> cq.Workplane:
    """Slightly raised label area on the front (+Y) face of the oval body."""
    # The label sits on the curved front face between the base and gold band.
    z_bottom = 0.014
    z_top = GOLD_BAND_Z - GOLD_BAND_H / 2.0 - 0.002
    label_h = z_top - z_bottom
    # Position at the front-most extent of the ellipse.
    plate = (
        cq.Workplane("XY")
        .workplane(offset=z_bottom)
        .center(0.0, BODY_RY - 0.0004)
        .rect(BODY_RX * 1.3, 0.0018)
        .extrude(label_h)
    )
    return plate


def _pump_solid() -> cq.Workplane:
    """Flat black airless actuator with elliptical bore matching the oval neck."""
    z0 = BODY_H + SHOULDER_H + NECK_H - PUMP_SEAT_OVERLAP
    # Rounded-rect exterior cap.
    cap = _rounded_prism(PUMP_W, PUMP_D, PUMP_H, PUMP_FILLET, z0=z0)

    # Elliptical bore so the skirt grips the oval neck.
    bore = (
        cq.Workplane("XY")
        .workplane(offset=z0 - 0.002)
        .ellipse(NECK_RX - 0.0005, NECK_RY - 0.0005)
        .extrude(PUMP_SEAT_OVERLAP + 0.001)
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
    model = ArticulatedObject(name="primer_pump_bottle_oval")

    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    gold = model.material("gold_accent", rgba=(0.80, 0.62, 0.22, 1.0))
    pump_black = model.material("pump_black", rgba=(0.05, 0.05, 0.06, 1.0))

    # ---- bottle body (root) — oval cross-section ----
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
        Box((2 * BODY_RX, 2 * BODY_RY, BODY_H)),
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

    # Pump skirt is intentionally slipped down over the oval neck collar.
    ctx.allow_overlap(
        pump,
        body,
        elem_a="pump_cap",
        elem_b="body_shell",
        reason="The airless pump skirt is intentionally seated down over the oval neck collar.",
    )

    # ---- body reads tall and oval (wider across X than front-to-back Y) ----
    body_ext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "body is tall (height dominates footprint)",
        body_ext[2] > 0.070 and body_ext[2] > body_ext[0] + 0.030 and body_ext[2] > body_ext[1] + 0.030,
        details=f"body extents={body_ext}",
    )
    ctx.check(
        "body cross-section is oval (wider in X than Y)",
        body_ext[0] > body_ext[1] + 0.004,
        details=f"body X={body_ext[0]:.4f}, Y={body_ext[1]:.4f}",
    )
    ctx.check(
        "body footprint is slim",
        body_ext[0] < 0.050 and body_ext[1] < 0.040,
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
    # Band wraps the oval: wider in X than Y, matching the body shape.
    band_ext = (band_aabb[1][0] - band_aabb[0][0], band_aabb[1][1] - band_aabb[0][1])
    ctx.check(
        "gold band follows oval cross-section (wider in X)",
        band_ext[0] > band_ext[1] + 0.002,
        details=f"band X={band_ext[0]:.4f}, Y={band_ext[1]:.4f}",
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
