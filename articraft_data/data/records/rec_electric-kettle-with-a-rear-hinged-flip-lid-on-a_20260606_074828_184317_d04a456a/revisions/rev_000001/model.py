from __future__ import annotations

# Electric kettle on a separate round power base.
# Frame: Z up (kettle axis vertical), kettle centerline at x=y=0.
#   - front (pour spout) at +X, rear (handle + lid hinge) at -X.
# Root: power_base (the black round mains base sitting on the table).
# Articulations (INDEPENDENT user-facing mechanisms):
#   - body_lift: PRISMATIC, kettle body lifts straight up off the base (+Z ~0.04m).
#   - lid_hinge: REVOLUTE, rear-hinged flip lid; positive q lifts the front
#     edge of the lid up and back about the rear hinge line (~0..70 deg).

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key dimensions (meters) --------------------------------------------------
BODY_R = 0.066  # kettle outer radius (~0.13 m dia)
BODY_BASE_Z = 0.012  # bottom of the kettle shell above the base seating ring
BODY_TOP_Z = 0.196  # top rim of the kettle shell
COLLAR_Z = 0.156  # where the black top collar begins
RIM_Z = BODY_TOP_Z  # mouth rim plane
WALL = 0.006

BASE_R = 0.080  # power base radius (~0.16 m dia)
BASE_H = 0.018  # power base disc height
LIFT = 0.040  # how far the body lifts off the base

HINGE_X = -BODY_R + 0.004  # rear hinge line x (just inside rear wall)
HINGE_Z = RIM_Z + 0.006  # hinge pin height (just above the rim)


def _loft(sections) -> cq.Workplane:
    # sections: list of ("circle", z, r) stacked along +Z (XY planes).
    wp = cq.Workplane("XY")
    prev = 0.0
    for i, s in enumerate(sections):
        z = s[1]
        off = z if i == 0 else z - prev
        if i > 0 or abs(off) > 1e-12:
            wp = wp.workplane(offset=off)
        wp = wp.circle(s[2])
        prev = z
    return wp.loft(ruled=False)


def _body_shell() -> cq.Workplane:
    # Hollow stainless-steel barrel: a gently waisted cylinder, open at the top
    # mouth.  Outer loft minus a slightly smaller inner loft that pokes out the
    # top so the kettle reads as a real open vessel with walls.
    outer = _loft(
        [
            ("circle", BODY_BASE_Z, BODY_R - 0.004),
            ("circle", BODY_BASE_Z + 0.010, BODY_R),
            ("circle", 0.090, BODY_R - 0.002),
            ("circle", COLLAR_Z, BODY_R - 0.001),
            ("circle", BODY_TOP_Z, BODY_R - 0.003),
        ]
    )
    inner = _loft(
        [
            ("circle", BODY_BASE_Z + WALL, BODY_R - 0.004 - WALL),
            ("circle", 0.090, BODY_R - 0.002 - WALL),
            ("circle", COLLAR_Z, BODY_R - 0.001 - WALL),
            ("circle", BODY_TOP_Z + 0.012, BODY_R - 0.003 - WALL),
        ]
    )
    return outer.cut(inner)


def _spout() -> cq.Workplane:
    # Front pour-spout lip: a small open trough that rises from the front rim
    # and tips slightly forward/up, breaking the circular mouth at +X.
    outer = (
        cq.Workplane("YZ")
        .workplane(offset=BODY_R - 0.012)
        .moveTo(0.0, COLLAR_Z + 0.006)
        .ellipse(0.020, 0.012)
        .workplane(offset=0.020)
        .moveTo(0.0, COLLAR_Z + 0.024)
        .ellipse(0.016, 0.010)
        .workplane(offset=0.014)
        .moveTo(0.0, COLLAR_Z + 0.040)
        .ellipse(0.011, 0.007)
        .loft(ruled=False)
    )
    inner = (
        cq.Workplane("YZ")
        .workplane(offset=BODY_R - 0.018)
        .moveTo(0.0, COLLAR_Z + 0.006)
        .ellipse(0.015, 0.009)
        .workplane(offset=0.022)
        .moveTo(0.0, COLLAR_Z + 0.024)
        .ellipse(0.012, 0.007)
        .workplane(offset=0.020)
        .moveTo(0.0, COLLAR_Z + 0.044)
        .ellipse(0.008, 0.005)
        .loft(ruled=False)
    )
    return outer.cut(inner)


def _pour_cut() -> cq.Workplane:
    # Front (+X) channel that breaks the rim/collar and upper wall so the pour
    # spout opens straight through into the kettle cavity (mouth runs through to
    # the interior, not a blind trough stuck on the outside).
    return (
        cq.Workplane("XY")
        .workplane(offset=COLLAR_Z + 0.002)
        .center(BODY_R - 0.006, 0.0)
        .rect(0.060, 0.020)
        .extrude(BODY_TOP_Z - COLLAR_Z + 0.030)
    )


def _handle() -> cq.Workplane:
    # Black C-handle on the rear (-X): two struts off the body joined by a long
    # vertical grip, leaving an open hand gap between grip and body.
    grip_x = -BODY_R - 0.040
    top_z = COLLAR_Z + 0.030
    bot_z = 0.050
    grip = (
        cq.Workplane("XY")
        .workplane(offset=bot_z)
        .center(grip_x, 0.0)
        .rect(0.016, 0.026)
        .workplane(offset=(top_z - bot_z) * 0.5)
        .center(0.004, 0.0)
        .rect(0.016, 0.028)
        .workplane(offset=(top_z - bot_z) * 0.5)
        .center(-0.004, 0.0)
        .rect(0.016, 0.026)
        .loft(ruled=False)
    )
    # Upper strut from grip top to the collar.
    upper = (
        cq.Workplane("XY")
        .workplane(offset=top_z - 0.013)
        .center(grip_x, 0.0)
        .rect(0.014, 0.024)
        .workplane(offset=0.0)
        .center((grip_x - (-BODY_R + 0.006)) * -0.5, 0.0)
        .rect(0.014, 0.024)
        .loft(ruled=False)
    )
    # Build struts as straight bridges using boxes lofted toward the body.
    upper_bridge = _strut(grip_x, top_z - 0.004, -BODY_R + 0.004, COLLAR_Z + 0.006)
    lower_bridge = _strut(grip_x, bot_z + 0.006, -BODY_R + 0.004, 0.072)
    return grip.union(upper).union(upper_bridge).union(lower_bridge)


def _strut(x0, z0, x1, z1) -> cq.Workplane:
    # A slim rounded bar bridging (x0,z0) -> (x1,z1) in the XZ plane at y=0.
    dx = x1 - x0
    dz = z1 - z0
    length = math.hypot(dx, dz)
    ang = math.atan2(dz, dx)
    bar = (
        cq.Workplane("XY")
        .box(length, 0.024, 0.012)
        .edges("|X")
        .fillet(0.004)
    )
    # Lay it in the XZ plane: rotate about Y so its long axis (local X) tilts.
    bar = bar.rotate((0, 0, 0), (0, 1, 0), -math.degrees(ang))
    bar = bar.translate(((x0 + x1) * 0.5, 0.0, (z0 + z1) * 0.5))
    return bar


def _base_solid() -> cq.Workplane:
    # Round black power base: a low disc with a slightly domed top and a small
    # central seating boss the kettle sits on.
    disc = (
        cq.Workplane("XY")
        .circle(BASE_R)
        .extrude(BASE_H)
        .edges(">Z")
        .fillet(0.006)
    )
    boss_wp = (
        cq.Workplane("XY")
        .workplane(offset=BASE_H)
        .circle(0.030)
        .extrude(0.006)
    )
    return disc.union(boss_wp)


def _lid_solid() -> cq.Workplane:
    # Round flip lid: a shallow domed disc that caps the mouth, with a low knob.
    lid_r = BODY_R - 0.006
    disc = (
        cq.Workplane("XY")
        .circle(lid_r)
        .extrude(0.008)
        .edges(">Z")
        .fillet(0.003)
    )
    knob = (
        cq.Workplane("XY")
        .workplane(offset=0.008)
        .circle(0.012)
        .extrude(0.010)
        .edges(">Z")
        .fillet(0.004)
    )
    return disc.union(knob)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="electric_kettle")

    steel = model.material("brushed_steel", rgba=(0.66, 0.67, 0.69, 1.0))
    black = model.material("black_plastic", rgba=(0.10, 0.10, 0.11, 1.0))
    dark = model.material("dark_gray", rgba=(0.18, 0.18, 0.20, 1.0))
    glass = model.material("glass_blue", rgba=(0.30, 0.40, 0.48, 0.85))

    # ============================ POWER BASE (root) ============================
    base = model.part("power_base")
    base.visual(mesh_from_cadquery(_base_solid(), "base_disc"), material=black, name="base_disc")
    # Small illuminated control pad on the front rim of the base.
    base.visual(
        Box((0.050, 0.034, 0.004)),
        origin=Origin(xyz=(BASE_R - 0.030, 0.0, BASE_H + 0.001)),
        material=dark,
        name="control_pad",
    )
    base.visual(
        mesh_from_geometry(
            CylinderGeometry(0.006, 0.003).translate(BASE_R - 0.030, 0.0, BASE_H + 0.004),
            "power_button",
        ),
        material=glass,
        name="power_button",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(BASE_R, BASE_H), mass=0.45, origin=Origin(xyz=(0.0, 0.0, BASE_H * 0.5))
    )

    # ============================ KETTLE BODY =================================
    body = model.part("kettle_body")

    pour_cut = _pour_cut()
    body.visual(
        mesh_from_cadquery(_body_shell().cut(pour_cut), "body_shell"),
        material=steel,
        name="body_shell",
    )

    # Black top collar ring (the dark band around the mouth in the image), with
    # the front pour channel cut through so the spout opens into the cavity.
    collar = (
        cq.Workplane("XY")
        .workplane(offset=COLLAR_Z - 0.002)
        .circle(BODY_R + 0.001)
        .extrude(BODY_TOP_Z - COLLAR_Z + 0.004)
        .faces(">Z")
        .workplane()
        .circle(BODY_R - WALL)
        .cutThruAll()
    ).cut(pour_cut)
    body.visual(mesh_from_cadquery(collar, "top_collar"), material=black, name="top_collar")

    # Black bottom heel ring / heating plate skirt.
    heel = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .circle(BODY_R - 0.002)
        .extrude(0.014)
        .edges(">Z")
        .fillet(0.004)
    )
    body.visual(mesh_from_cadquery(heel, "base_collar"), material=black, name="base_collar")

    body.visual(mesh_from_cadquery(_spout(), "spout"), material=steel, name="spout")
    body.visual(mesh_from_cadquery(_handle(), "handle"), material=black, name="handle")

    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, BODY_TOP_Z),
        mass=0.9,
        origin=Origin(xyz=(-0.01, 0.0, BODY_TOP_Z * 0.5)),
    )

    model.articulation(
        "body_lift",
        ArticulationType.PRISMATIC,
        parent=base,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, BASE_H + 0.005)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.2, lower=0.0, upper=LIFT),
    )

    # ============================ FLIP LID ===================================
    # The lid geometry is authored about the hinge frame: the hinge line is at
    # the rear, and the lid disc extends forward (+X) so positive revolute q
    # lifts the front edge up.  Build it centered on the body, then shift so the
    # rear edge sits on the hinge origin.
    lid = model.part("lid")
    lid_solid = _lid_solid().translate((BODY_R - 0.006, 0.0, 0.0))  # disc center forward of hinge
    lid.visual(mesh_from_cadquery(lid_solid, "lid_disc"), material=dark, name="lid_disc")
    # Rear hinge knuckle that wraps the pin (sits at the hinge origin).
    knuckle = TorusGeometry(0.006, 0.004, radial_segments=12, tubular_segments=24).rotate_x(
        math.pi / 2.0
    )
    lid.visual(mesh_from_geometry(knuckle, "hinge_knuckle"), material=black, name="hinge_knuckle")
    lid.inertial = Inertial.from_geometry(
        Cylinder(BODY_R - 0.006, 0.018),
        mass=0.05,
        origin=Origin(xyz=(BODY_R - 0.006, 0.0, 0.004)),
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        # Lid disc extends along +X from the hinge; -Y lifts the free (+X) edge up.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.22),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("power_base")
    body = object_model.get_part("kettle_body")
    lid = object_model.get_part("lid")
    body_lift = object_model.get_articulation("body_lift")
    lid_hinge = object_model.get_articulation("lid_hinge")

    # ---- kettle shell is centered over the power base and seats on it ----
    ctx.expect_within(
        body,
        base,
        axes="xy",
        inner_elem="body_shell",
        outer_elem="base_disc",
        margin=0.003,
        name="body shell centered over base",
    )
    # The kettle heel seats on the base's central boss (tiny intentional capture).
    ctx.allow_overlap(
        body,
        base,
        elem_a="base_collar",
        elem_b="base_disc",
        reason="Kettle heel ring seats onto the base's central seating boss when docked.",
    )
    ctx.expect_contact(
        body,
        base,
        elem_a="base_collar",
        elem_b="base_disc",
        name="kettle heel rests on the power base",
    )
    base_pos = ctx.part_world_position(base)
    body_pos = ctx.part_world_position(body)
    ctx.check(
        "kettle body sits above the power base",
        base_pos is not None and body_pos is not None and body_pos[2] >= base_pos[2],
        details=f"base={base_pos}, body={body_pos}",
    )

    # ---- the body lifts straight up off the base ----
    rest = ctx.part_world_position(body)
    with ctx.pose({body_lift: LIFT}):
        lifted = ctx.part_world_position(body)
    ctx.check(
        "kettle body lifts up off the base",
        lifted[2] > rest[2] + 0.03,
        details=f"rest_z={rest[2]:.4f}, lifted_z={lifted[2]:.4f}",
    )

    # ---- pour spout is at the front-top (+X), above the collar ----
    spout_box = ctx.part_element_world_aabb(body, elem="spout")
    body_box = ctx.part_world_aabb(body)
    ctx.check(
        "pour spout breaks the front of the mouth (+X)",
        spout_box is not None and spout_box[1][0] >= body_box[1][0] - 0.002,
        details=f"spout_max_x={spout_box[1][0]:.4f}, body_max_x={body_box[1][0]:.4f}",
    )
    ctx.check(
        "pour spout sits high on the body",
        spout_box[0][2] > COLLAR_Z - 0.01,
        details=f"spout_min_z={spout_box[0][2]:.4f}",
    )

    # ---- C-handle is on the rear side (-X), opposite the spout ----
    handle_box = ctx.part_element_world_aabb(body, elem="handle")
    ctx.check(
        "C-handle is on the rear (-X) side",
        handle_box is not None and handle_box[0][0] < -BODY_R,
        details=f"handle_min_x={handle_box[0][0]:.4f}",
    )

    # ---- lid caps the mouth at rest; flips open about the rear hinge ----
    front_rest = ctx.part_world_aabb(lid)[1]  # max corner
    front_z_rest = front_rest[2]
    front_x_rest = front_rest[0]
    with ctx.pose({lid_hinge: 1.22}):
        lid_box_open = ctx.part_world_aabb(lid)
    # Opening lifts the front edge upward (max-Z grows) and pulls it rearward.
    ctx.check(
        "flip lid front edge lifts up when opened",
        lid_box_open[1][2] > front_z_rest + 0.03,
        details=f"rest_top_z={front_z_rest:.4f}, open_top_z={lid_box_open[1][2]:.4f}",
    )
    ctx.check(
        "flip lid swings back about the rear hinge",
        lid_box_open[1][0] < front_x_rest - 0.02,
        details=f"rest_front_x={front_x_rest:.4f}, open_front_x={lid_box_open[1][0]:.4f}",
    )

    # The lid rim and the body mouth collar share the rim plane: tiny seated
    # overlap at the hinge knuckle / collar is intentional capture.
    ctx.allow_overlap(
        lid,
        body,
        elem_a="hinge_knuckle",
        elem_b="top_collar",
        reason="Rear hinge knuckle is intentionally captured at the body collar/rim.",
    )
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_disc",
        elem_b="top_collar",
        reason="Closed lid rim seats a hair into the mouth collar to read as a sealed cap.",
    )

    return ctx.report()


object_model = build_object_model()
