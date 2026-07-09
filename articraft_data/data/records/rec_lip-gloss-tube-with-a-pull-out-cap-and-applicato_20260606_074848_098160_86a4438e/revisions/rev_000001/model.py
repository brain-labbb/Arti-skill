from __future__ import annotations

# Lip gloss tube with a pull-out cap and doe-foot applicator wand.
# Frame: tube axis along +Z (vertical). z=0 at the bottom of the clear base,
# the gold faceted body rises above it, and a short neck sits at the top.
# Articulation:
#   - cap + applicator wand (one rigid part): PRISMATIC, pulls straight UP (+Z)
#     out of the tube. At rest the long white wand hangs down through the body
#     with its pink doe-foot tip seated in the gloss near the base. At full
#     extension the cap lifts off the neck and the tip clears the tube mouth.

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

# ---- Tube body geometry (axis = +Z) ----------------------------------------
BASE_Z0 = 0.0           # bottom of clear base
BASE_TOP = 0.024        # top of the clear base section
BODY_TOP = 0.082        # top of the gold faceted body (below neck)
NECK_TOP = 0.094        # top of the threaded neck / tube mouth
BODY_FLAT = 0.0105      # octagon "radius" (center to flat) of the body
BASE_FLAT = 0.0095      # clear base octagon half-width
NECK_R = 0.0066         # neck (threaded mouth) radius
BORE_R = 0.0072         # inner bore radius the wand travels through

# ---- Cap + wand geometry ----------------------------------------------------
CAP_FLAT = 0.0118       # cap octagon "radius" (slightly proud of the body)
CAP_H = 0.030           # cap shell height
WAND_R = 0.0019         # thin doe-foot stem radius
TIP_R = 0.0045          # soft applicator tip radius
# At rest the cap sits over the neck; its top is here.
CAP_REST_TOP = NECK_TOP + 0.012
PULL = 0.072            # prismatic travel that lifts the wand tip clear of the mouth


def _octa_prism(half_flat: float, z0: float, z1: float) -> cq.Workplane:
    # Regular octagonal prism between z0 and z1, centered on the Z axis.
    # half_flat is the apothem (center-to-flat distance).
    r = half_flat / math.cos(math.pi / 8.0)  # circumradius for a regular octagon
    pts = []
    for i in range(8):
        a = math.pi / 8.0 + i * math.pi / 4.0
        pts.append((r * math.cos(a), r * math.sin(a)))
    wp = (
        cq.Workplane("XY")
        .workplane(offset=z0)
        .polyline(pts)
        .close()
        .extrude(z1 - z0)
    )
    return wp


def _tube_body_mesh() -> cq.Workplane:
    # Gold faceted (octagonal) body with a hollow bore, plus a round threaded
    # neck on top. The bore is open at the top so the wand can travel through it.
    body = _octa_prism(BODY_FLAT, BASE_TOP - 0.001, BODY_TOP)
    # Round threaded neck.
    neck = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP - 0.002)
        .circle(NECK_R)
        .extrude(NECK_TOP - BODY_TOP + 0.002)
    )
    body = body.union(neck)
    # Hollow bore (open at the top, stops above the gloss reservoir).
    bore = (
        cq.Workplane("XY")
        .workplane(offset=BASE_TOP + 0.002)
        .circle(BORE_R)
        .extrude(NECK_TOP - BASE_TOP)
    )
    body = body.cut(bore)
    body = body.edges("|Z").fillet(0.0006)
    return body


def _clear_base_mesh() -> cq.Workplane:
    # Clear octagonal base section that shows the gloss, hollow inside.
    outer = _octa_prism(BASE_FLAT, BASE_Z0, BASE_TOP)
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=0.0015)
        .circle(BASE_FLAT - 0.0012)
        .extrude(BASE_TOP)
    )
    return outer.cut(cavity)


def _gloss_mesh() -> cq.Workplane:
    # Pink gloss liquid filling the clear base and a little of the lower body.
    gloss = (
        cq.Workplane("XY")
        .workplane(offset=0.002)
        .circle(BASE_FLAT - 0.0016)
        .extrude(BASE_TOP - 0.003)
    )
    # A thin column of gloss continues up into the bore where the wand dips in.
    column = (
        cq.Workplane("XY")
        .workplane(offset=BASE_TOP - 0.001)
        .circle(BORE_R - 0.0006)
        .extrude(0.012)
    )
    return gloss.union(column)


def _cap_mesh() -> cq.Workplane:
    # Gold octagonal cap shell, hollow underneath so it slips over the neck.
    z0 = CAP_REST_TOP - CAP_H
    outer = _octa_prism(CAP_FLAT, z0, CAP_REST_TOP)
    socket = (
        cq.Workplane("XY")
        .workplane(offset=z0 - 0.001)
        .circle(NECK_R + 0.0012)
        .extrude(CAP_H - 0.006)
    )
    cap = outer.cut(socket)
    cap = cap.edges("|Z").fillet(0.0008)
    return cap


def _wand_mesh() -> cq.Workplane:
    # White doe-foot stem hanging down from inside the cap, ending just above
    # the gloss reservoir at rest. Long enough that pulling the cap up lifts the
    # whole stem and tip out through the mouth.
    # Stem reaches up to the inner roof of the cap socket so the cap and wand
    # fuse into one connected solid.
    cap_under = CAP_REST_TOP - 0.005          # stem top buried in the cap crown
    tip_center_z = BASE_TOP + 0.006           # doe-foot tip sits in the gloss column
    stem_bottom = tip_center_z + TIP_R * 0.5
    stem = (
        cq.Workplane("XY")
        .workplane(offset=stem_bottom)
        .circle(WAND_R)
        .extrude(cap_under - stem_bottom)
    )
    return stem


def _tip_mesh() -> cq.Workplane:
    # Soft pink doe-foot applicator tip at the bottom of the wand stem.
    tip_center_z = BASE_TOP + 0.006
    # Tapered tip: a short cylinder capped with spheres.
    body = (
        cq.Workplane("XY")
        .workplane(offset=tip_center_z)
        .circle(TIP_R)
        .extrude(0.008)
    )
    nose = cq.Workplane("XY").sphere(TIP_R).translate((0.0, 0.0, tip_center_z))
    top = cq.Workplane("XY").sphere(TIP_R * 0.85).translate((0.0, 0.0, tip_center_z + 0.008))
    return body.union(nose).union(top)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lip_gloss_tube")

    gold = model.material("gold", rgba=(0.78, 0.62, 0.18, 1.0))
    clear = model.material("clear_base", rgba=(0.86, 0.88, 0.90, 0.32))
    pink = model.material("pink_gloss", rgba=(0.93, 0.55, 0.66, 1.0))
    white = model.material("wand_white", rgba=(0.95, 0.95, 0.95, 1.0))
    pink_tip = model.material("tip_pink", rgba=(0.86, 0.18, 0.45, 1.0))

    # ---- tube (root): gold faceted body + clear base + pink gloss ----
    tube = model.part("tube")
    tube.visual(mesh_from_cadquery(_tube_body_mesh(), "tube_body"), material=gold, name="tube_body")
    tube.visual(mesh_from_cadquery(_clear_base_mesh(), "clear_base"), material=clear, name="clear_base")
    tube.visual(mesh_from_cadquery(_gloss_mesh(), "gloss"), material=pink, name="gloss")
    tube.inertial = Inertial.from_geometry(
        Cylinder(radius=BODY_FLAT, length=NECK_TOP),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP / 2.0)),
    )

    # ---- cap + applicator wand: one rigid part, pulls straight up ----
    cap = model.part("cap_applicator")
    cap.visual(mesh_from_cadquery(_cap_mesh(), "cap"), material=gold, name="cap")
    cap.visual(mesh_from_cadquery(_wand_mesh(), "wand_stem"), material=white, name="wand_stem")
    cap.visual(mesh_from_cadquery(_tip_mesh(), "doe_foot_tip"), material=pink_tip, name="doe_foot_tip")
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=CAP_FLAT, length=CAP_REST_TOP - BASE_TOP),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.0, (CAP_REST_TOP + BASE_TOP) / 2.0)),
    )

    model.articulation(
        "tube_to_cap",
        ArticulationType.PRISMATIC,
        parent=tube,
        child=cap,
        # Cap/wand geometry is authored in the same absolute frame as the tube,
        # so the joint frame is coincident with the parent frame at rest (q=0)
        # and positive prismatic q lifts the whole part straight up the +Z axis.
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.2, lower=0.0, upper=PULL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tube = object_model.get_part("tube")
    cap = object_model.get_part("cap_applicator")
    joint = object_model.get_articulation("tube_to_cap")

    # --- Intentional nested overlaps (cap-over-neck, wand/tip inside bore) ---
    ctx.allow_overlap(
        cap,
        tube,
        elem_a="cap",
        elem_b="tube_body",
        reason="The gold cap socket is intentionally slipped over the threaded neck at rest.",
    )
    ctx.allow_overlap(
        cap,
        tube,
        elem_a="wand_stem",
        elem_b="tube_body",
        reason="The long applicator wand intentionally hangs down inside the tube bore.",
    )
    ctx.allow_overlap(
        cap,
        tube,
        elem_a="doe_foot_tip",
        elem_b="gloss",
        reason="The doe-foot tip is intentionally seated in the pink gloss reservoir at rest.",
    )
    ctx.allow_overlap(
        cap,
        tube,
        elem_a="doe_foot_tip",
        elem_b="tube_body",
        reason="The doe-foot tip dips into the lower reservoir of the tube body at rest, "
        "the same gloss-filled cavity it withdraws from when the cap is pulled up.",
    )

    # --- Hero geometry: tube is tall and faceted, gloss is pink ---
    body_aabb = ctx.part_element_world_aabb(tube, elem="tube_body")
    bmn, bmx = body_aabb
    height = bmx[2] - bmn[2]
    width = max(bmx[0] - bmn[0], bmx[1] - bmn[1])
    ctx.check(
        "tube body is tall (height >> width)",
        height > 0.05 and height > 2.5 * width,
        details=f"height={height:.4f}, width={width:.4f}",
    )

    gloss_v = tube.get_visual("gloss")
    r, g, b, _a = gloss_v.material.rgba
    ctx.check(
        "gloss is pink",
        r > 0.7 and g < 0.75 and b > 0.45 and r > b,
        details=f"gloss rgba={gloss_v.material.rgba}",
    )

    # --- Cap + wand seated at rest: tip down inside the tube, near the base ---
    tip_rest = ctx.part_element_world_aabb(cap, elem="doe_foot_tip")
    tip_rest_top = tip_rest[1][2]
    ctx.check(
        "wand tip seated inside the tube at rest",
        tip_rest_top < BODY_TOP,
        details=f"rest tip top z={tip_rest_top:.4f}, body top={BODY_TOP:.4f}",
    )
    # Wand reaches deep toward the gloss.
    stem_rest = ctx.part_element_world_aabb(cap, elem="wand_stem")
    ctx.check(
        "wand stem extends down into the lower tube at rest",
        stem_rest[0][2] < BASE_TOP + 0.02,
        details=f"rest stem bottom z={stem_rest[0][2]:.4f}",
    )

    # Cap socket overlaps the neck at rest (seated over the mouth).
    ctx.expect_overlap(
        cap, tube, axes="z", elem_a="cap", elem_b="tube_body",
        min_overlap=0.004, name="cap seated over the neck at rest",
    )
    # Doe-foot tip is seated in the gloss reservoir at rest.
    ctx.expect_contact(
        cap, tube, elem_a="doe_foot_tip", elem_b="gloss",
        name="doe-foot tip seated in the gloss at rest",
    )
    # Wand stays centered in the bore (non-motion axes).
    ctx.expect_within(
        cap, tube, axes="xy", inner_elem="wand_stem", outer_elem="tube_body",
        margin=0.001, name="wand centered in the tube bore",
    )

    rest_cap_z = ctx.part_world_position(cap)[2]

    # --- Full extension: cap lifts up and the tip clears the tube mouth ---
    with ctx.pose({joint: joint.motion_limits.upper}):
        ext_cap_z = ctx.part_world_position(cap)[2]
        tip_ext = ctx.part_element_world_aabb(cap, elem="doe_foot_tip")
        tip_ext_bottom = tip_ext[0][2]
        ctx.check(
            "wand tip clears the tube mouth at full extension",
            tip_ext_bottom > NECK_TOP,
            details=f"extended tip bottom z={tip_ext_bottom:.4f}, mouth top={NECK_TOP:.4f}",
        )

    ctx.check(
        "cap pulls straight up out of the tube",
        ext_cap_z > rest_cap_z + 0.05,
        details=f"rest cap z={rest_cap_z:.4f}, extended cap z={ext_cap_z:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
