from __future__ import annotations

# Boston-round amber-glass serum bottle with a white dropper cap.
# Variant: taller Boston-round body with a bulbous lower shoulder curving
# into a distinct narrower waisted neck, replacing the parent's squat
# straight-cylinder body. Dropper closure, hollow open-mouth shell, and
# white label band identical in function to the parent.
# Frame: bottle axis along +Z, base at z=0, dropper rising at +Z.
# Construction:
#   - body (root): Boston-round amber-glass hollow shell (lathe revolved
#     spline profile) + white paper label band around the belly.
#   - dropper assembly (ONE rigid part): white collar gripping the neck,
#     rounded white rubber squeeze bulb on top, thin clear-glass pipette
#     running down through the neck into the bottle.
# Articulation:
#   - body_to_dropper: PRISMATIC, pulls straight UP out of the neck.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
)

# ---- Boston-round body dimensions ----
WALL = 0.0018            # glass wall thickness

# Key z-heights
Z_BASE = 0.0
Z_BELLY_LOW = 0.016      # lower belly starts reaching max width
Z_BELLY_MID = 0.034      # belly midpoint
Z_BELLY_TOP = 0.050      # upper belly begins tapering
Z_SHOULDER = 0.062       # shoulder curving inward
Z_NECK_TRANS = 0.076     # transition to straight neck
Z_NECK_TOP = 0.098       # top of neck (mouth)

# Radii
BODY_MAX_R = 0.019       # max outer radius at belly (~38mm dia)
BODY_BASE_R = 0.011      # base footprint radius
NECK_R = 0.0075          # neck outer radius (narrower waisted neck)
BORE_R = NECK_R - WALL   # inner bore at neck

# ---- label band ----
LABEL_Z0 = 0.018
LABEL_Z1 = 0.048
LABEL_R = BODY_MAX_R + 0.0005

# ---- dropper assembly (same function as parent, adjusted for new neck) ----
COLLAR_R = 0.0100        # white collar grips over the narrower neck
COLLAR_Z0 = 0.082        # collar bottom (over the neck)
COLLAR_TOP = 0.106       # collar top
BULB_R = 0.0095          # squeeze bulb radius
BULB_CZ = 0.1240         # bulb center height
PIPETTE_R = 0.0024       # thin glass pipette radius
PIPETTE_BOTTOM = 0.026   # pipette tip height at rest (deep in bottle)

DROPPER_TRAVEL = 0.072   # prismatic pull-up distance


def _outer_profile():
    """Outer half-profile (radius, z) for the Boston-round body.

    Traces from base center outward, bulges to max radius at the belly,
    then curves smoothly inward through the shoulder into the narrower
    waisted neck.
    """
    pts = [
        (0.000, Z_BASE),
        (BODY_BASE_R, Z_BASE + 0.001),
        (0.015, 0.006),
        (BODY_MAX_R - 0.002, 0.010),
        (BODY_MAX_R, Z_BELLY_LOW),
        (BODY_MAX_R, Z_BELLY_TOP),
        (BODY_MAX_R - 0.003, Z_SHOULDER),
        (0.012, 0.068),
        (NECK_R + 0.002, Z_NECK_TRANS),
        (NECK_R, Z_NECK_TRANS + 0.004),
        (NECK_R, Z_NECK_TOP),
    ]
    return sample_catmull_rom_spline_2d(pts, samples_per_segment=8)


def _inner_profile():
    """Inner cavity profile (radius, z), offset inward by WALL thickness.

    Follows the outer profile shape to create a uniform-thickness shell.
    Ends at the mouth so the bottle reads as an open container.
    """
    pts = [
        (0.000, Z_BASE + WALL),
        (BODY_BASE_R - WALL, Z_BASE + WALL + 0.001),
        (0.015 - WALL, 0.006 + WALL * 0.5),
        (BODY_MAX_R - WALL - 0.002, 0.010 + WALL * 0.3),
        (BODY_MAX_R - WALL, Z_BELLY_LOW + WALL * 0.3),
        (BODY_MAX_R - WALL, Z_BELLY_TOP),
        (BODY_MAX_R - WALL - 0.003, Z_SHOULDER),
        (0.012 - WALL, 0.068),
        (BORE_R, Z_NECK_TRANS + 0.002),
        (BORE_R, Z_NECK_TOP),
    ]
    return sample_catmull_rom_spline_2d(pts, samples_per_segment=8)


def _body_glass_mesh():
    """Hollow Boston-round bottle shell via lathe revolution."""
    outer = _outer_profile()
    inner = _inner_profile()
    geom = LatheGeometry.from_shell_profiles(
        outer, inner,
        segments=48,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    return mesh_from_geometry(geom, "body_glass")


def _collar_mesh():
    """White collar: ring that grips the neck, hollowed to slip over it."""
    outer = (
        cq.Workplane("XY")
        .workplane(offset=COLLAR_Z0)
        .circle(COLLAR_R)
        .extrude(COLLAR_TOP - COLLAR_Z0)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=COLLAR_Z0 - 0.001)
        .circle(NECK_R - 0.0001)
        .extrude((COLLAR_TOP - 0.002) - (COLLAR_Z0 - 0.001))
    )
    collar = outer.cut(inner)
    return mesh_from_cadquery(collar, "collar")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="serum_bottle_boston_round")

    amber = model.material("amber_glass", rgba=(0.45, 0.24, 0.06, 0.5))
    white = model.material("white_plastic", rgba=(0.95, 0.95, 0.94, 1.0))
    label_white = model.material("label_white", rgba=(0.97, 0.97, 0.96, 1.0))
    clear = model.material("clear_glass", rgba=(0.85, 0.90, 0.92, 0.35))

    # ---- body (root) ----
    body = model.part("body")
    body.visual(_body_glass_mesh(), material=amber, name="body_glass")

    # white paper label band wrapping the belly of the body
    label = (
        cq.Workplane("XY")
        .workplane(offset=LABEL_Z0)
        .circle(LABEL_R)
        .circle(BODY_MAX_R - 0.0002)
        .extrude(LABEL_Z1 - LABEL_Z0)
    )
    body.visual(
        mesh_from_cadquery(label, "label_band"),
        material=label_white,
        name="label_band",
    )

    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_MAX_R, Z_NECK_TOP),
        mass=0.042,
        origin=Origin(xyz=(0.0, 0.0, Z_NECK_TOP / 2.0)),
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
    # short stem connecting collar top to bulb
    stem_bottom = COLLAR_TOP
    stem_top = BULB_CZ - BULB_R
    dropper.visual(
        Cylinder(0.0045, stem_top - stem_bottom + 0.004),
        origin=Origin(xyz=(0.0, 0.0, (stem_bottom + stem_top) / 2.0)),
        material=white,
        name="bulb_stem",
    )

    # thin clear-glass pipette running from inside collar down into the body
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

    # --- Boston-round body is taller and has a bulbous belly ---
    body_aabb = ctx.part_world_aabb(body)
    bmn, bmx = body_aabb
    body_h = bmx[2] - bmn[2]
    body_dia = max(bmx[0] - bmn[0], bmx[1] - bmn[1])
    ctx.check(
        "body is taller than squat parent — height over 0.088 m",
        body_h > 0.088,
        details=f"body height={body_h:.4f}",
    )
    ctx.check(
        "body belly is bulbous — diameter wider than 0.032 m",
        body_dia > 0.032,
        details=f"body max dia={body_dia:.4f}",
    )

    # --- neck is distinctly narrower than the belly (waisted) ---
    neck_dia = 2.0 * NECK_R
    ctx.check(
        "neck is distinctly narrower than belly (Boston-round waist)",
        neck_dia < body_dia * 0.55,
        details=f"neck dia={neck_dia:.4f}, body dia={body_dia:.4f}, ratio={neck_dia/body_dia:.2f}",
    )

    # --- body glass is amber-tinted and translucent ---
    body_glass = body.get_visual("body_glass")
    mat = body_glass.material
    rgba = getattr(mat, "rgba", None)
    ctx.check(
        "body glass is amber-tinted and translucent",
        rgba is not None and rgba[0] > rgba[2] and rgba[3] < 0.95,
        details=f"amber_glass rgba={rgba}",
    )

    # --- white label band wraps the belly ---
    label_aabb = ctx.part_element_world_aabb(body, elem="label_band")
    lmn, lmx = label_aabb
    ctx.check(
        "label band is on the body belly",
        lmn[2] > 0.005 and lmx[2] < Z_SHOULDER,
        details=f"label z=[{lmn[2]:.4f},{lmx[2]:.4f}]",
    )

    # --- dropper seated in the neck at rest, pipette inserted into bottle ---
    pip_rest = ctx.part_element_world_aabb(dropper, elem="pipette")
    ctx.check(
        "pipette tip is seated deep inside the bottle at rest",
        pip_rest[0][2] < Z_SHOULDER - 0.010,
        details=f"pipette bottom z(rest)={pip_rest[0][2]:.4f}",
    )
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

    # --- dropper pulls straight UP and pipette clears the mouth ---
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
        pip_up[0][2] >= Z_NECK_TOP - 0.001,
        details=f"pipette bottom z(extended)={pip_up[0][2]:.4f}, mouth={Z_NECK_TOP:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
