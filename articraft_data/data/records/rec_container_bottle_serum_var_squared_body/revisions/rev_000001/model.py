from __future__ import annotations

# Small amber-glass serum bottle with a SQUARED FACETED body and white dropper cap.
# Fork of the round-cylinder parent: only the body footprint/shape changes.
# Frame: bottle axis along +Z, bottle base at z=0, dropper rising at +Z.
# Construction:
#   - body (root): squared faceted amber-glass shell (flat sides, rounded
#     vertical edges) rising to a shoulder, then a round neck (hollow open
#     mouth), wrapped by a white label band on the flat-sided middle.
#   - dropper assembly (ONE rigid part): white collar that grips the neck, a
#     rounded white rubber squeeze bulb on top, and a thin clear-glass pipette
#     that runs down through the neck into the bottle.
# Articulation:
#   - dropper assembly: PRISMATIC, pulls straight UP out of the neck, lifting
#     the pipette clear of the bottle mouth.

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

# ---- squared body dimensions ----
BODY_W = 0.028           # body width across flats (square cross-section)
CORNER_R = 0.003         # vertical edge fillet radius (rounded corners)
BODY_Z0 = 0.0            # base
BODY_TOP = 0.060         # top of the straight faceted wall
SHOULDER_TOP = 0.070     # top of the shoulder transition
NECK_R = 0.0080          # neck outer radius (round)
NECK_TOP = 0.0820        # top of the neck (bottle mouth)
WALL = 0.0018            # glass wall thickness
BORE_R = NECK_R - WALL   # inner bore radius at the neck

# ---- label band ----
LABEL_Z0 = 0.012
LABEL_Z1 = 0.044
LABEL_OFFSET = 0.0006    # label sits proud of the body flats
LABEL_W = BODY_W + 2.0 * LABEL_OFFSET
LABEL_CR = CORNER_R + LABEL_OFFSET

# ---- dropper assembly dimensions (identical to parent) ----
COLLAR_R = 0.0105        # white collar grips over the neck
COLLAR_Z0 = 0.066        # collar bottom (overlaps the shoulder/neck slightly)
COLLAR_TOP = 0.090       # collar top
BULB_R = 0.0095          # squeeze bulb radius
BULB_CZ = 0.1075         # bulb center height
PIPETTE_R = 0.0024       # thin glass pipette radius
PIPETTE_BOTTOM = 0.018   # pipette tip height at rest (deep in the bottle)

DROPPER_TRAVEL = 0.068   # prismatic pull-up distance


def _rounded_rect_solid(w, corner_r, z_start, height):
    """Build an extruded rounded-rectangle solid centered at origin."""
    return (
        cq.Workplane("XY")
        .workplane(offset=z_start)
        .rect(w, w)
        .extrude(height)
        .edges("|Z")
        .fillet(min(corner_r, w / 2.0 - 0.0001))
    )


def _body_glass_mesh():
    """Squared faceted amber-glass shell with shoulder, neck, and interior bore."""
    # --- outer shell ---
    # Main body: extruded square with filleted vertical edges
    body_outer = (
        cq.Workplane("XY")
        .rect(BODY_W, BODY_W)
        .extrude(BODY_TOP)
        .edges("|Z")
        .fillet(CORNER_R)
    )
    # Shoulder: loft from the square body top to the round neck base
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP)
        .rect(BODY_W, BODY_W)
        .workplane(offset=(SHOULDER_TOP - BODY_TOP))
        .circle(NECK_R + 0.0015)
        .loft(ruled=True)
    )
    # Neck: round cylinder
    neck = (
        cq.Workplane("XY")
        .workplane(offset=SHOULDER_TOP)
        .circle(NECK_R)
        .extrude(NECK_TOP - SHOULDER_TOP)
    )
    solid = body_outer.union(shoulder).union(neck)

    # --- interior cavity ---
    # Hollow bore that follows the outer profile offset by WALL thickness.
    inner_w = BODY_W - 2.0 * WALL
    inner_cr = max(CORNER_R - WALL, 0.0005)

    cavity_body = _rounded_rect_solid(inner_w, inner_cr, WALL, BODY_TOP - WALL)

    cavity_shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP)
        .rect(inner_w, inner_w)
        .workplane(offset=(SHOULDER_TOP - BODY_TOP))
        .circle(BORE_R)
        .loft(ruled=True)
    )
    cavity_neck = (
        cq.Workplane("XY")
        .workplane(offset=SHOULDER_TOP)
        .circle(BORE_R)
        .extrude(NECK_TOP - SHOULDER_TOP + 0.001)
    )
    cavity = cavity_body.union(cavity_shoulder).union(cavity_neck)
    solid = solid.cut(cavity)

    return mesh_from_cadquery(solid, "body_glass")


def _label_band_mesh():
    """White label band: a thin rounded-rect shell wrapping the squared body."""
    outer = _rounded_rect_solid(LABEL_W, LABEL_CR, LABEL_Z0, LABEL_Z1 - LABEL_Z0)
    # inner cutout slightly smaller than body so the label penetrates the body
    # surface — this ensures the label mesh contacts the body glass (connected
    # geometry within the body part), matching the parent's approach.
    inner_cut = _rounded_rect_solid(
        BODY_W - 0.0004, max(CORNER_R - 0.0002, 0.0005),
        LABEL_Z0 - 0.001, (LABEL_Z1 - LABEL_Z0) + 0.002,
    )
    label = outer.cut(inner_cut)
    return mesh_from_cadquery(label, "label_band")


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
        .circle(NECK_R + 0.0006)
        .extrude((COLLAR_TOP - 0.002) - (COLLAR_Z0 - 0.001))
    )
    collar = outer.cut(inner)
    return mesh_from_cadquery(collar, "collar")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="serum_bottle")

    amber = model.material("amber_glass", rgba=(0.45, 0.24, 0.06, 0.5))
    white = model.material("white_plastic", rgba=(0.95, 0.95, 0.94, 1.0))
    label_white = model.material("label_white", rgba=(0.97, 0.97, 0.96, 1.0))
    clear = model.material("clear_glass", rgba=(0.85, 0.90, 0.92, 0.35))

    # ---- body (root) ----
    body = model.part("body")
    body.visual(_body_glass_mesh(), material=amber, name="body_glass")
    body.visual(_label_band_mesh(), material=label_white, name="label_band")

    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_W, BODY_TOP)),
        mass=0.040,
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
        Cylinder(0.0045, (BULB_CZ - BULB_R) - COLLAR_TOP + 0.004),
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
        mass=0.008,
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

    # --- squared body shape verification ---
    body_aabb = ctx.part_world_aabb(body)
    bmn, bmx = body_aabb
    body_h = bmx[2] - bmn[2]
    x_width = bmx[0] - bmn[0]
    y_width = bmx[1] - bmn[1]

    ctx.check(
        "bottle body is short — height under ~0.09 m",
        body_h < 0.090,
        details=f"body height={body_h:.4f}",
    )
    ctx.check(
        "body has squared cross-section (X and Y widths approximately equal)",
        abs(x_width - y_width) < 0.004,
        details=f"x_width={x_width:.4f}, y_width={y_width:.4f}",
    )
    ctx.check(
        "body width across flats is ~28mm (squared, not round 30mm)",
        0.024 < x_width < 0.034,
        details=f"x_width={x_width:.4f}",
    )

    # amber glass material
    body_glass = body.get_visual("body_glass")
    mat = body_glass.material
    rgba = getattr(mat, "rgba", None)
    ctx.check(
        "body glass is amber-tinted and translucent",
        rgba is not None and rgba[0] > rgba[2] and rgba[3] < 0.95,
        details=f"amber_glass rgba={rgba}",
    )

    # --- white label band wraps the squared body ---
    label_aabb = ctx.part_element_world_aabb(body, elem="label_band")
    lmn, lmx = label_aabb
    ctx.check(
        "label band is on the body wall, around the middle",
        lmn[2] > 0.005 and lmx[2] < SHOULDER_TOP,
        details=f"label z=[{lmn[2]:.4f},{lmx[2]:.4f}]",
    )
    # label width should match squared body (not round)
    label_xw = lmx[0] - lmn[0]
    ctx.check(
        "label band follows squared body footprint",
        abs(label_xw - LABEL_W) < 0.003,
        details=f"label x_width={label_xw:.4f}, expected~{LABEL_W:.4f}",
    )

    # --- dropper seated in the neck at rest, pipette inserted into the bottle ---
    pip_rest = ctx.part_element_world_aabb(dropper, elem="pipette")
    ctx.check(
        "pipette tip is seated deep inside the bottle at rest",
        pip_rest[0][2] < SHOULDER_TOP - 0.010,
        details=f"pipette bottom z(rest)={pip_rest[0][2]:.4f}",
    )
    # collar grips the neck (projected overlap with the body along z)
    ctx.allow_overlap(
        dropper,
        body,
        elem_a="collar",
        elem_b="body_glass",
        reason="The white collar intentionally slips over the bottle neck (seated grip).",
    )
    ctx.allow_overlap(
        dropper,
        body,
        elem_a="pipette",
        elem_b="body_glass",
        reason="The thin glass pipette is intentionally inserted down through the neck into the bottle.",
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
        "pipette tip clears the bottle mouth at full extension",
        pip_up[0][2] >= NECK_TOP - 0.001,
        details=f"pipette bottom z(extended)={pip_up[0][2]:.4f}, mouth={NECK_TOP:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
