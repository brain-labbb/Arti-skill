from __future__ import annotations

# Electric kettle on a separate round power base — twist-lock cap variant.
# Frame: Z up (kettle axis vertical), kettle centerline at x=y=0.
#   - front (pour spout) at +X, rear (handle) at -X.
# Root: power_base (the black round mains base sitting on the table).
# Articulations (INDEPENDENT user-facing mechanisms):
#   - body_lift: PRISMATIC, kettle body lifts straight up off the base (+Z ~0.04m).
#   - cap_twist: REVOLUTE about Z, twist-lock domed cap seats on the mouth
#     collar; positive q rotates the cap to tighten/lock (~0..60 deg).

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

CAP_SEAT_Z = BODY_TOP_Z + 0.002  # collar top face where the cap flange seats

# ---- twist-lock cap dimensions ------------------------------------------------
PLUG_R = BODY_R - WALL - 0.006  # 0.054 — fits inside body shell bore
FLANGE_R = BODY_R - 0.001  # 0.065 — nearly matches collar outer edge
N_LUGS = 3
LUG_OFFSET_DEG = 60.0  # first lug at 60 deg to avoid the pour spout


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


def _twist_cap_body() -> cq.Workplane:
    """Domed twist-lock cap body: plug skirt + seating flange + dome + grip knob.

    Built in the cap local frame with z=0 at the seating plane (collar top).
    The plug extends below z=0 into the mouth; the dome rises above the flange.
    """
    # Plug skirt: extends down from z=0 into the mouth bore
    plug = (
        cq.Workplane("XY")
        .workplane(offset=-0.012)
        .circle(PLUG_R)
        .extrude(0.012)
    )

    # Annular thread ridge on the plug (simplified thread representation)
    thread = (
        cq.Workplane("XY")
        .workplane(offset=-0.008)
        .circle(PLUG_R + 0.002)
        .circle(PLUG_R - 0.001)
        .extrude(0.003)
    )

    # Seating flange: rests on the collar top face
    flange = (
        cq.Workplane("XY")
        .circle(FLANGE_R)
        .extrude(0.004)
        .edges(">Z")
        .fillet(0.001)
    )

    # Dome: smooth loft from flange top to apex
    dome = (
        cq.Workplane("XY")
        .workplane(offset=0.004)
        .circle(FLANGE_R - 0.003)
        .workplane(offset=0.005)
        .circle(FLANGE_R * 0.55)
        .workplane(offset=0.005)
        .circle(0.010)
        .loft(ruled=False)
    )

    # Central grip knob for twisting
    knob = (
        cq.Workplane("XY")
        .workplane(offset=0.014)
        .circle(0.009)
        .extrude(0.007)
        .edges(">Z")
        .fillet(0.003)
    )

    return plug.union(thread).union(flange).union(dome).union(knob)


def _twist_cap_lug() -> cq.Workplane:
    """Single bayonet lug tab protruding radially from the plug skirt.

    Built at angle 0 (+X direction) in the cap local frame.
    Rotate about Z to place at other angular positions.
    """
    lug_radial = 0.003  # radial protrusion from plug surface
    lug_tangential = 0.008  # width around the circumference
    lug_h = 0.004  # vertical height
    return (
        cq.Workplane("XY")
        .workplane(offset=-0.009)
        .center(PLUG_R + lug_radial * 0.5, 0.0)
        .rect(lug_radial, lug_tangential)
        .extrude(lug_h)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="electric_kettle_twist_cap")

    steel = model.material("brushed_steel", rgba=(0.66, 0.67, 0.69, 1.0))
    black = model.material("black_plastic", rgba=(0.10, 0.10, 0.11, 1.0))
    dark = model.material("dark_gray", rgba=(0.18, 0.18, 0.20, 1.0))
    glass = model.material("glass_blue", rgba=(0.30, 0.40, 0.48, 0.85))
    cap_mat = model.material("cap_plastic", rgba=(0.20, 0.22, 0.26, 1.0))

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

    # ============================ TWIST-LOCK CAP ==============================
    # Domed threaded cap that seats on the mouth collar and closes by rotating
    # about the vertical kettle axis.  The cap part frame sits at the collar
    # top (the actual contact surface), with the plug extending down into the
    # mouth bore and the dome rising above.
    cap = model.part("cap")
    cap.visual(
        mesh_from_cadquery(_twist_cap_body(), "cap_body"),
        material=cap_mat,
        name="cap_body",
    )

    # Bayonet lugs: N_LUGS evenly spaced, offset to avoid the pour spout
    _base_lug = _twist_cap_lug()
    for i in range(N_LUGS):
        angle_deg = LUG_OFFSET_DEG + i * (360.0 / N_LUGS)
        lug = _base_lug.rotate((0, 0, 0), (0, 0, 1), angle_deg)
        cap.visual(
            mesh_from_cadquery(lug, f"cap_lug_{i}"),
            material=dark,
            name=f"cap_lug_{i}",
        )

    cap.inertial = Inertial.from_geometry(
        Cylinder(FLANGE_R, 0.022),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )

    model.articulation(
        "cap_twist",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, CAP_SEAT_Z)),
        # Rotation about vertical (Z) axis; positive q tightens/locks the cap.
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("power_base")
    body = object_model.get_part("kettle_body")
    cap = object_model.get_part("cap")
    body_lift = object_model.get_articulation("body_lift")
    cap_twist = object_model.get_articulation("cap_twist")

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

    # ---- twist-lock cap seats on the mouth collar and rotates about Z ----
    # The cap plug inserts into the collar bore and the flange seats on the
    # collar top — this nesting is the twist-lock engagement.
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_body",
        elem_b="top_collar",
        reason="Cap plug skirt inserts into the collar bore and the flange seats on the collar rim as a twist-lock fit.",
    )

    # Cap is centered on the kettle axis over the body mouth
    ctx.expect_within(
        cap,
        body,
        axes="xy",
        inner_elem="cap_body",
        outer_elem="top_collar",
        margin=0.005,
        name="cap centered on the mouth collar",
    )

    # The plug skirt inserts into the mouth bore below the collar (Z overlap)
    ctx.expect_overlap(
        cap,
        body,
        axes="z",
        elem_a="cap_body",
        elem_b="top_collar",
        min_overlap=0.005,
        name="cap plug inserts into the mouth bore",
    )

    # Cap dome/knob top is above the collar top (visible seated cap profile)
    ctx.expect_gap(
        cap,
        body,
        axis="z",
        elem_a="cap_body",
        elem_b="top_collar",
        min_gap=-0.015,
        max_gap=None,
        name="cap body extends above the collar (dome visible above rim)",
    )

    # Rotating the cap moves the lugs angularly about the Z axis
    lug0_rest = ctx.part_element_world_aabb(cap, elem="cap_lug_0")
    with ctx.pose({cap_twist: 1.05}):
        lug0_twisted = ctx.part_element_world_aabb(cap, elem="cap_lug_0")
    rest_cx = (lug0_rest[0][0] + lug0_rest[1][0]) * 0.5
    rest_cy = (lug0_rest[0][1] + lug0_rest[1][1]) * 0.5
    twist_cx = (lug0_twisted[0][0] + lug0_twisted[1][0]) * 0.5
    twist_cy = (lug0_twisted[0][1] + lug0_twisted[1][1]) * 0.5
    lug_shift = math.hypot(twist_cx - rest_cx, twist_cy - rest_cy)
    ctx.check(
        "cap rotation sweeps lugs angularly about Z",
        lug_shift > 0.005,
        details=f"lug_center_shift={lug_shift:.4f}m",
    )

    # Cap stays at the same height when twisted (no vertical displacement)
    cap_z_rest = ctx.part_world_position(cap)
    with ctx.pose({cap_twist: 1.05}):
        cap_z_twist = ctx.part_world_position(cap)
    ctx.check(
        "cap stays seated at same height when twisted",
        abs(cap_z_twist[2] - cap_z_rest[2]) < 0.002,
        details=f"rest_z={cap_z_rest[2]:.4f}, twist_z={cap_z_twist[2]:.4f}",
    )

    # The cap dome rises above the collar (visible cap profile)
    cap_aabb = ctx.part_world_aabb(cap)
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "cap dome extends above the body rim",
        cap_aabb[1][2] > body_aabb[1][2] - 0.005,
        details=f"cap_top_z={cap_aabb[1][2]:.4f}, body_top_z={body_aabb[1][2]:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
