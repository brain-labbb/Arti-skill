from __future__ import annotations

# Black airless cosmetic PRIMER bottle with a tall treatment/lotion pump head.
# Fork of the flat-puck airless bottle: same cushion body, gold band, and label,
# but the closure is now a tall gooseneck-spout pump head.
# Frame: bottle stands upright along +Z. Base at z=0, cushion body rises to a
# flat shoulder, short neck collar. A cylindrical actuator collar seats over
# the neck, a vertical stem rises from it, and a long curved gooseneck spout
# bends out to the side (+X) and over, ending in a downturned nozzle tip.
# Articulation:
#   - pump_press: PRISMATIC, presses straight DOWN (-Z, ~0.006 m) then springs back.

import math

import cadquery as cq
from cadquery.func import (
    circle as func_circle,
    face as func_face,
    spline as func_spline,
    sweep as func_sweep,
)
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

# ---- body dimensions (meters) — identical to parent ----
BODY_W = 0.034
BODY_D = 0.024
BODY_H = 0.086
BODY_FILLET = 0.008

SHOULDER_H = 0.006
SHOULDER_TOP_W = 0.020
SHOULDER_TOP_D = 0.016

NECK_W = 0.018
NECK_D = 0.014
NECK_H = 0.006

GOLD_BAND_Z = 0.050
GOLD_BAND_H = 0.005

# ---- pump-head dimensions (treatment / lotion gooseneck pump) ----
COLLAR_R = 0.010        # collar outer radius
COLLAR_H = 0.012        # collar height
COLLAR_BORE_R = 0.007   # bore radius (grips neck)
STEM_R = 0.005          # vertical stem radius
STEM_H = 0.018          # stem height above collar top
SPOUT_R = 0.003         # gooseneck tube radius
NOZZLE_R = 0.0028       # nozzle tip outer radius
NOZZLE_H = 0.007        # nozzle tip length (downward)

PUMP_SEAT_OVERLAP = 0.004   # collar slips down over neck
PUMP_TRAVEL = 0.006         # press-down stroke

# ---- derived absolute heights ----
NECK_TOP_Z = BODY_H + SHOULDER_H + NECK_H     # 0.098
COLLAR_Z0 = NECK_TOP_Z - PUMP_SEAT_OVERLAP    # 0.094
COLLAR_TOP_Z = COLLAR_Z0 + COLLAR_H           # 0.106
STEM_TOP_Z = COLLAR_TOP_Z + STEM_H            # 0.124


def _rounded_prism(w: float, d: float, h: float, fillet: float, z0: float = 0.0) -> cq.Workplane:
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
    return _rounded_prism(
        BODY_W + 0.0008,
        BODY_D + 0.0008,
        GOLD_BAND_H,
        BODY_FILLET,
        z0=GOLD_BAND_Z - GOLD_BAND_H / 2.0,
    )


def _label_plate_solid() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .workplane(offset=0.012)
        .center(0.0, BODY_D / 2.0 - 0.0005)
        .rect(BODY_W - 0.010, 0.0018)
        .extrude(GOLD_BAND_Z - GOLD_BAND_H / 2.0 - 0.012 - 0.002)
    )


def _pump_head_solid() -> cq.Workplane:
    """Tall treatment pump head: collar + stem + gooseneck spout + nozzle tip."""

    # --- actuator collar with neck bore ---
    collar = (
        cq.Workplane("XY")
        .workplane(offset=COLLAR_Z0)
        .circle(COLLAR_R)
        .extrude(COLLAR_H)
    )
    # Hollow bore so the collar grips the neck collar
    bore = (
        cq.Workplane("XY")
        .workplane(offset=COLLAR_Z0 - 0.002)
        .circle(COLLAR_BORE_R)
        .extrude(PUMP_SEAT_OVERLAP + 0.004)
    )
    collar = collar.cut(bore)

    # Small chamfer ring at the collar top for realism
    collar_top_ring = (
        cq.Workplane("XY")
        .workplane(offset=COLLAR_TOP_Z - 0.001)
        .circle(COLLAR_R + 0.001)
        .circle(COLLAR_R - 0.001)
        .extrude(0.002)
    )
    collar = collar.union(collar_top_ring)

    # --- vertical stem rising from collar ---
    stem = (
        cq.Workplane("XY")
        .workplane(offset=COLLAR_TOP_Z)
        .circle(STEM_R)
        .extrude(STEM_H)
    )
    collar = collar.union(stem)

    # --- gooseneck spout: swept tube along a spline path ---
    # Path arches out to +X and curves back down to a dispensing position.
    path_points = [
        (0.000, 0.0, STEM_TOP_Z),
        (0.004, 0.0, STEM_TOP_Z + 0.008),
        (0.014, 0.0, STEM_TOP_Z + 0.013),
        (0.025, 0.0, STEM_TOP_Z + 0.008),
        (0.031, 0.0, STEM_TOP_Z - 0.002),
        (0.032, 0.0, STEM_TOP_Z - 0.016),
    ]
    path_tangents = [
        (0.0, 0.0, 1.0),    # start tangent: upward
        (0.0, 0.0, -1.0),   # end tangent: downward
    ]
    path_wire = func_spline(path_points, path_tangents)

    # Circular profile at path start, swept into a solid tube
    profile = func_face(func_circle(SPOUT_R)).moved(z=STEM_TOP_Z)
    gooseneck_shape = func_sweep(profile, path_wire)
    collar = collar.union(cq.Workplane("XY").newObject([gooseneck_shape]))

    # --- downturned nozzle tip at gooseneck end ---
    nozzle_end = path_points[-1]  # (0.032, 0.0, STEM_TOP_Z - 0.016)
    nozzle = (
        cq.Workplane("XY")
        .workplane(offset=nozzle_end[2] - NOZZLE_H)
        .center(nozzle_end[0], nozzle_end[1])
        .circle(NOZZLE_R)
        .extrude(NOZZLE_H + 0.001)  # slight overlap with gooseneck end
    )
    collar = collar.union(nozzle)

    # Small dispensing orifice (hole) through the nozzle bottom face
    orifice = (
        cq.Workplane("XY")
        .workplane(offset=nozzle_end[2] - NOZZLE_H - 0.001)
        .center(nozzle_end[0], nozzle_end[1])
        .circle(0.0012)
        .extrude(0.004)
    )
    collar = collar.cut(orifice)

    return collar


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="primer_gooseneck_pump_bottle")

    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    gold = model.material("gold_accent", rgba=(0.80, 0.62, 0.22, 1.0))
    pump_black = model.material("pump_black", rgba=(0.05, 0.05, 0.06, 1.0))

    # ---- bottle body (root) — identical to parent ----
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

    # ---- pump head: tall gooseneck treatment pump, presses straight down ----
    pump_head = model.part("pump_head")
    pump_head.visual(
        mesh_from_cadquery(_pump_head_solid(), "pump_head_shell"),
        material=pump_black,
        name="pump_head_shell",
    )
    pump_head.inertial = Inertial.from_geometry(
        Cylinder(radius=COLLAR_R, length=COLLAR_H + STEM_H + 0.020),
        mass=0.015,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_Z0 + (COLLAR_H + STEM_H) / 2.0)),
    )

    model.articulation(
        "pump_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=pump_head,
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
    pump = object_model.get_part("pump_head")
    press = object_model.get_articulation("pump_press")

    # Pump collar is intentionally seated down over the body neck collar.
    ctx.allow_overlap(
        pump,
        body,
        elem_a="pump_head_shell",
        elem_b="body_shell",
        reason="The pump actuator collar is intentionally seated over the neck collar.",
    )

    # ---- bottle body reads slim and tall ----
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

    # ---- gold accent band present on the body ----
    band_aabb = ctx.part_element_world_aabb(body, elem="gold_band")
    band_z = (band_aabb[0][2] + band_aabb[1][2]) / 2.0
    ctx.check(
        "gold accent band sits partway up the body",
        0.030 < band_z < 0.070,
        details=f"gold band center z={band_z}",
    )

    # ---- pump head is tall (taller than the flat puck parent) ----
    pump_aabb = ctx.part_world_aabb(pump)
    pump_height = pump_aabb[1][2] - pump_aabb[0][2]
    ctx.check(
        "pump head is tall (gooseneck extends well above neck)",
        pump_height > 0.030,
        details=f"pump head height={pump_height:.4f}",
    )

    # ---- pump head mounted at the top of the bottle ----
    pump_center_z = (pump_aabb[0][2] + pump_aabb[1][2]) / 2.0
    ctx.check(
        "pump head mounted at the top of the bottle",
        pump_aabb[0][2] > BODY_H - 0.010 and pump_center_z > BODY_H,
        details=f"pump aabb z=[{pump_aabb[0][2]:.4f}, {pump_aabb[1][2]:.4f}]",
    )

    # ---- gooseneck spout extends laterally from the stem ----
    pump_ext = _ext(pump_aabb)
    ctx.check(
        "gooseneck spout extends laterally (X span exceeds collar diameter)",
        pump_ext[0] > 0.028,
        details=f"pump X span={pump_ext[0]:.4f}",
    )

    # ---- pump seated over neck (footprint overlap) ----
    ctx.expect_overlap(
        pump, body, axes="xy", min_overlap=0.010,
        name="pump collar seated over neck (footprint)",
    )

    # ---- pump presses straight DOWN and stays seated ----
    pump_top_rest = ctx.part_world_aabb(pump)[1][2]
    pump_bot_rest = ctx.part_world_aabb(pump)[0][2]
    with ctx.pose({press: -PUMP_TRAVEL}):
        pump_top_down = ctx.part_world_aabb(pump)[1][2]
        pump_bot_down = ctx.part_world_aabb(pump)[0][2]
        ctx.expect_overlap(
            pump, body, axes="xy", min_overlap=0.010,
            name="pump stays seated when pressed",
        )
    ctx.check(
        "pump top drops when pressed",
        pump_top_down < pump_top_rest - 0.004,
        details=f"top rest_z={pump_top_rest:.4f}, pressed_z={pump_top_down:.4f}",
    )
    ctx.check(
        "pump presses straight down (only z changes)",
        pump_bot_down < pump_bot_rest - 0.004,
        details=f"bottom rest_z={pump_bot_rest:.4f}, pressed_z={pump_bot_down:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
