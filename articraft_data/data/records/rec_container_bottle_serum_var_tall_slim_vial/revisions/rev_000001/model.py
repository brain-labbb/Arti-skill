from __future__ import annotations

# Tall slim amber-glass serum vial with a white dropper cap.
# Fork of the round squat bottle: same dropper closure family, but the body
# is now a high-aspect-ratio narrow straight glass tube (ampoule/test-tube
# style) with a minimal shoulder rising directly into the neck.
# Frame: bottle axis along +Z, bottle base at z=0, dropper rising at +Z.
# Construction:
#   - body (root): tall slim amber-glass tube + minimal shoulder + neck (hollow
#     shell), wrapped by a white paper label band around the middle.
#   - dropper assembly (ONE rigid part): white collar that grips the neck, a
#     rounded white rubber squeeze bulb on top, and a thin clear-glass pipette
#     that runs down through the neck into the vial.
# Articulation:
#   - dropper assembly: PRISMATIC, pulls straight UP out of the neck, lifting
#     the pipette clear of the vial mouth.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- tall slim vial body dimensions ----
BODY_R = 0.0065         # body outer radius (~13 mm diameter, slim vial)
BODY_Z0 = 0.0           # base
BODY_TOP = 0.098        # top of the long straight tube wall
SHOULDER_TOP = 0.103    # top of the minimal shoulder taper
NECK_R = 0.0052         # neck outer radius (narrow, just under body)
NECK_TOP = 0.115        # top of the neck (vial mouth)
WALL = 0.0012           # glass wall thickness
BORE_R = NECK_R - WALL  # inner bore radius at the neck

# ---- label band ----
LABEL_Z0 = 0.022
LABEL_Z1 = 0.062
LABEL_R = BODY_R + 0.0004

# ---- dropper assembly dimensions ----
COLLAR_R = 0.0078       # white collar grips over the neck
COLLAR_Z0 = 0.101       # collar bottom (overlaps the shoulder/neck slightly)
COLLAR_TOP = 0.124      # collar top
BULB_R = 0.0080         # squeeze bulb radius
BULB_CZ = 0.140         # bulb center height
PIPETTE_R = 0.0018      # thin glass pipette radius
PIPETTE_BOTTOM = 0.016  # pipette tip height at rest (deep in the vial)

DROPPER_TRAVEL = 0.105  # prismatic pull-up distance (lifts pipette clear of mouth)


def _body_glass_mesh():
    # Hollow amber-glass shell: tall slim straight tube, minimal shoulder taper,
    # short neck, with an internal bore so the vial reads as a real open
    # container.
    # Long straight tube section
    tube = (
        cq.Workplane("XY")
        .circle(BODY_R)
        .extrude(BODY_TOP)
    )
    # Minimal shoulder: a short smooth loft from body radius to neck radius
    # (ampoule / test-tube style, almost no pronounced shoulder)
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP)
        .circle(BODY_R)
        .workplane(offset=(SHOULDER_TOP - BODY_TOP))
        .circle(NECK_R + 0.0008)
        .loft(ruled=False)
    )
    # Short neck rising from the shoulder to the mouth
    neck = (
        cq.Workplane("XY")
        .workplane(offset=SHOULDER_TOP)
        .circle(NECK_R)
        .extrude(NECK_TOP - SHOULDER_TOP)
    )
    solid = tube.union(shoulder).union(neck)
    # Interior cavity: a single tapered bore staying one wall-thickness inside
    # the outer profile at every height, open at the mouth.
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .circle(BODY_R - WALL)               # z = WALL, tube interior
        .workplane(offset=(BODY_TOP - WALL))
        .circle(BODY_R - WALL)               # z = BODY_TOP, tube interior
        .workplane(offset=(SHOULDER_TOP - BODY_TOP))
        .circle(BORE_R)                      # z = SHOULDER_TOP, into neck
        .workplane(offset=(NECK_TOP - SHOULDER_TOP + 0.001))
        .circle(BORE_R)                      # through the mouth (open top)
        .loft(ruled=False)
    )
    solid = solid.cut(cavity)
    # Subtle bottom rounding: a small fillet-like inset so the base reads as
    # finished glass rather than a sharp cut plane.
    base_round = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .circle(BODY_R - 0.001)
        .extrude(0.001)
    )
    solid = solid.cut(base_round)
    return mesh_from_cadquery(solid, "body_glass")


def _collar_mesh():
    # White collar: an outer ring that grips the neck, with a small bore that
    # the pipette passes through, plus a thin top cap the bulb sits on.
    outer = (
        cq.Workplane("XY")
        .workplane(offset=COLLAR_Z0)
        .circle(COLLAR_R)
        .extrude(COLLAR_TOP - COLLAR_Z0)
    )
    # hollow it out so it slips over the neck (open at the bottom)
    inner = (
        cq.Workplane("XY")
        .workplane(offset=COLLAR_Z0 - 0.001)
        .circle(NECK_R + 0.0004)
        .extrude((COLLAR_TOP - 0.002) - (COLLAR_Z0 - 0.001))
    )
    collar = outer.cut(inner)
    return mesh_from_cadquery(collar, "collar")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="serum_vial")

    amber = model.material("amber_glass", rgba=(0.45, 0.24, 0.06, 0.5))
    white = model.material("white_plastic", rgba=(0.95, 0.95, 0.94, 1.0))
    label_white = model.material("label_white", rgba=(0.97, 0.97, 0.96, 1.0))
    clear = model.material("clear_glass", rgba=(0.85, 0.90, 0.92, 0.35))

    # ---- body (root) ----
    body = model.part("body")
    body.visual(_body_glass_mesh(), material=amber, name="body_glass")

    # white paper label band wrapping the middle of the tube (thin shell ring)
    label = (
        cq.Workplane("XY")
        .workplane(offset=LABEL_Z0)
        .circle(LABEL_R)
        .circle(BODY_R - 0.0002)
        .extrude(LABEL_Z1 - LABEL_Z0)
    )
    body.visual(mesh_from_cadquery(label, "label_band"), material=label_white, name="label_band")

    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, BODY_TOP),
        mass=0.028,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP / 2.0)),
    )

    # ---- dropper assembly (collar + bulb + pipette as ONE rigid part) ----
    dropper = model.part("dropper")

    # white collar
    dropper.visual(_collar_mesh(), material=white, name="collar")

    # rounded white rubber squeeze bulb on top of the collar
    dropper.visual(
        Sphere(BULB_R),
        origin=Origin(xyz=(0.0, 0.0, BULB_CZ)),
        material=white,
        name="bulb",
    )
    # short neck connecting the collar top to the bulb so they read as one piece
    dropper.visual(
        Cylinder(0.0040, (BULB_CZ - BULB_R) - COLLAR_TOP + 0.004),
        origin=Origin(xyz=(0.0, 0.0, (COLLAR_TOP + (BULB_CZ - BULB_R)) / 2.0)),
        material=white,
        name="bulb_stem",
    )

    # thin clear-glass pipette running down from inside the collar into the body
    pip_len = COLLAR_TOP - PIPETTE_BOTTOM
    dropper.visual(
        Cylinder(PIPETTE_R, pip_len),
        origin=Origin(xyz=(0.0, 0.0, PIPETTE_BOTTOM + pip_len / 2.0)),
        material=clear,
        name="pipette",
    )

    dropper.inertial = Inertial.from_geometry(
        Cylinder(COLLAR_R, COLLAR_TOP - PIPETTE_BOTTOM),
        mass=0.007,
        origin=Origin(xyz=(0.0, 0.0, (PIPETTE_BOTTOM + BULB_CZ) / 2.0)),
    )

    model.articulation(
        "body_to_dropper",
        ArticulationType.PRISMATIC,
        parent=body,
        child=dropper,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=DROPPER_TRAVEL,
            effort=5.0,
            velocity=0.1,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    dropper = object_model.get_part("dropper")
    pull = object_model.get_articulation("body_to_dropper")

    # --- vial body is TALL and SLIM (high-aspect-ratio narrow tube) ---
    body_aabb = ctx.part_world_aabb(body)
    bmn, bmx = body_aabb
    body_h = bmx[2] - bmn[2]
    body_dia = max(bmx[0] - bmn[0], bmx[1] - bmn[1])
    ctx.check(
        "vial body is tall — height over 0.10 m",
        body_h > 0.100,
        details=f"body height={body_h:.4f}",
    )
    ctx.check(
        "vial body is slim — diameter under 0.018 m",
        body_dia < 0.018,
        details=f"body dia={body_dia:.4f}",
    )
    aspect = body_h / body_dia if body_dia > 0 else 0.0
    ctx.check(
        "vial body is high-aspect-ratio (tall slim tube, ratio > 6)",
        aspect > 6.0,
        details=f"height/dia aspect ratio={aspect:.1f}",
    )
    body_glass = body.get_visual("body_glass")
    mat = body_glass.material
    rgba = getattr(mat, "rgba", None)
    ctx.check(
        "body glass is amber-tinted and translucent",
        rgba is not None and rgba[0] > rgba[2] and rgba[3] < 0.95,
        details=f"amber_glass rgba={rgba}",
    )

    # --- white label band wraps the slim tube ---
    label_aabb = ctx.part_element_world_aabb(body, elem="label_band")
    lmn, lmx = label_aabb
    ctx.check(
        "label band is on the body wall, around the middle",
        lmn[2] > 0.008 and lmx[2] < SHOULDER_TOP,
        details=f"label z=[{lmn[2]:.4f},{lmx[2]:.4f}]",
    )

    # --- dropper seated in the neck at rest, pipette inserted into the vial ---
    pip_rest = ctx.part_element_world_aabb(dropper, elem="pipette")
    ctx.check(
        "pipette tip is seated deep inside the vial at rest",
        pip_rest[0][2] < SHOULDER_TOP - 0.010,
        details=f"pipette bottom z(rest)={pip_rest[0][2]:.4f}",
    )
    # collar grips the neck (projected overlap with the body along z)
    ctx.allow_overlap(
        dropper,
        body,
        elem_a="collar",
        elem_b="body_glass",
        reason="The white collar intentionally slips over the vial neck (seated grip).",
    )
    ctx.allow_overlap(
        dropper,
        body,
        elem_a="pipette",
        elem_b="body_glass",
        reason="The thin glass pipette is intentionally inserted down through the neck into the vial.",
    )
    ctx.expect_overlap(
        dropper,
        body,
        axes="z",
        elem_a="collar",
        elem_b="body_glass",
        min_overlap=0.002,
        name="collar seated over the neck at rest",
    )

    # --- dropper pulls straight UP and the pipette clears the neck/mouth ---
    rest_pos = ctx.part_world_position(dropper)
    with ctx.pose({pull: DROPPER_TRAVEL}):
        up_pos = ctx.part_world_position(dropper)
        pip_up = ctx.part_element_world_aabb(dropper, elem="pipette")
    ctx.check(
        "dropper translates straight up when pulled (no lateral shift)",
        up_pos[2] > rest_pos[2] + 0.05
        and abs(up_pos[0] - rest_pos[0]) < 1e-4
        and abs(up_pos[1] - rest_pos[1]) < 1e-4,
        details=f"rest={rest_pos}, up={up_pos}",
    )
    ctx.check(
        "pipette tip clears the vial mouth at full extension",
        pip_up[0][2] >= NECK_TOP - 0.001,
        details=f"pipette bottom z(extended)={pip_up[0][2]:.4f}, mouth={NECK_TOP:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
