from __future__ import annotations

# Electric kettle with a SQUAT POT-BELLY body on a power base.
# Fork variant: body shell is a short, wide, rounded pot-belly (oblate ovoid)
# that bulges at mid-height and tucks back in at the rim.
# Frame: Z up, kettle centerline at x=y=0.
#   - front (pour spout) at +X, rear (handle + lid hinge) at -X.
# Root: power_base.
# Articulations:
#   - body_lift: PRISMATIC, kettle lifts off base (+Z ~0.04m).
#   - lid_hinge: REVOLUTE, rear-hinged flip lid.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
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
# Pot-belly body: squat, wide, bulging at mid-height, tucking in at the rim.
BODY_BOTTOM_R = 0.050   # bottom radius (seats on base)
BODY_BELLY_R = 0.095    # max bulge radius at mid-height
BODY_RIM_R = 0.052      # rim/mouth radius (tucked back in)
BODY_BASE_Z = 0.012     # bottom of kettle shell above base
BODY_TOP_Z = 0.125      # top rim plane
BELLY_Z = 0.065         # height of max bulge
COLLAR_Z = 0.100        # where the black top collar begins
RIM_Z = BODY_TOP_Z
WALL = 0.006

BASE_R = 0.080          # power base radius
BASE_H = 0.018          # power base height
LIFT = 0.040            # body lift travel

HINGE_X = -BODY_RIM_R + 0.004   # rear hinge line (just inside rim)
HINGE_Z = RIM_Z + 0.006         # hinge pin height (just above rim)

# Outer body profile: (z, radius) pairs for the lathe-like loft.
_OUTER_PROFILE = [
    (BODY_BASE_Z, BODY_BOTTOM_R),
    (0.020, 0.062),
    (0.035, 0.080),
    (0.050, 0.090),
    (BELLY_Z, BODY_BELLY_R),
    (0.080, 0.090),
    (0.095, 0.075),
    (COLLAR_Z, 0.058),
    (BODY_TOP_Z, BODY_RIM_R),
]


def _body_r(z: float) -> float:
    """Linearly interpolated outer body radius at height z."""
    if z <= _OUTER_PROFILE[0][0]:
        return _OUTER_PROFILE[0][1]
    if z >= _OUTER_PROFILE[-1][0]:
        return _OUTER_PROFILE[-1][1]
    for i in range(len(_OUTER_PROFILE) - 1):
        z0, r0 = _OUTER_PROFILE[i]
        z1, r1 = _OUTER_PROFILE[i + 1]
        if z0 <= z <= z1:
            t = (z - z0) / (z1 - z0)
            return r0 + t * (r1 - r0)
    return BODY_BELLY_R


def _loft(sections) -> cq.Workplane:
    """Stack circular sections along +Z to create a smooth lofted solid."""
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
    """Hollow pot-belly shell: wide ovoid bulge, open at the top mouth."""
    outer = _loft([("circle", z, r) for z, r in _OUTER_PROFILE])
    inner = _loft(
        [
            ("circle", BODY_BASE_Z + WALL, BODY_BOTTOM_R - WALL),
            ("circle", 0.035, 0.080 - WALL),
            ("circle", 0.050, 0.090 - WALL),
            ("circle", BELLY_Z, BODY_BELLY_R - WALL),
            ("circle", 0.080, 0.090 - WALL),
            ("circle", 0.095, 0.075 - WALL),
            ("circle", COLLAR_Z, 0.058 - WALL),
            ("circle", BODY_TOP_Z + 0.012, BODY_RIM_R - WALL),
        ]
    )
    return outer.cut(inner)


def _spout() -> cq.Workplane:
    """Front pour-spout lip rising from the collar/rim area."""
    r_collar = _body_r(COLLAR_Z + 0.006)
    outer = (
        cq.Workplane("YZ")
        .workplane(offset=r_collar - 0.010)
        .moveTo(0.0, COLLAR_Z + 0.004)
        .ellipse(0.018, 0.012)
        .workplane(offset=0.018)
        .moveTo(0.0, COLLAR_Z + 0.020)
        .ellipse(0.014, 0.010)
        .workplane(offset=0.014)
        .moveTo(0.0, COLLAR_Z + 0.036)
        .ellipse(0.010, 0.007)
        .loft(ruled=False)
    )
    inner = (
        cq.Workplane("YZ")
        .workplane(offset=r_collar - 0.016)
        .moveTo(0.0, COLLAR_Z + 0.004)
        .ellipse(0.013, 0.009)
        .workplane(offset=0.020)
        .moveTo(0.0, COLLAR_Z + 0.020)
        .ellipse(0.010, 0.007)
        .workplane(offset=0.018)
        .moveTo(0.0, COLLAR_Z + 0.040)
        .ellipse(0.007, 0.005)
        .loft(ruled=False)
    )
    return outer.cut(inner)


def _pour_cut() -> cq.Workplane:
    """Front channel cutting through collar/upper wall for the pour spout."""
    r_at_cut = _body_r(COLLAR_Z + 0.004)
    return (
        cq.Workplane("XY")
        .workplane(offset=COLLAR_Z)
        .center(r_at_cut - 0.004, 0.0)
        .rect(0.050, 0.018)
        .extrude(BODY_TOP_Z - COLLAR_Z + 0.028)
    )


def _handle() -> cq.Workplane:
    """Black C-handle on the rear (-X), clearing the belly bulge."""
    grip_x = -BODY_BELLY_R - 0.038
    top_z = BODY_TOP_Z - 0.008
    bot_z = 0.028

    # Vertical grip bar lofted through three cross-sections.
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

    # Upper strut from grip top to body near collar.
    r_upper = _body_r(COLLAR_Z + 0.006)
    upper_bridge = _strut(grip_x, top_z - 0.004, -r_upper + 0.004, COLLAR_Z + 0.006)

    # Lower strut from grip bottom to body below the belly.
    r_lower = _body_r(0.038)
    lower_bridge = _strut(grip_x, bot_z + 0.006, -r_lower + 0.004, 0.042)

    return grip.union(upper_bridge).union(lower_bridge)


def _strut(x0, z0, x1, z1) -> cq.Workplane:
    """Slim rounded bar bridging two points in the XZ plane at y=0."""
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
    bar = bar.rotate((0, 0, 0), (0, 1, 0), -math.degrees(ang))
    bar = bar.translate(((x0 + x1) * 0.5, 0.0, (z0 + z1) * 0.5))
    return bar


def _base_solid() -> cq.Workplane:
    """Round power base disc with a central seating boss."""
    disc = (
        cq.Workplane("XY")
        .circle(BASE_R)
        .extrude(BASE_H)
        .edges(">Z")
        .fillet(0.006)
    )
    boss = (
        cq.Workplane("XY")
        .workplane(offset=BASE_H)
        .circle(0.030)
        .extrude(0.006)
    )
    return disc.union(boss)


def _lid_solid() -> cq.Workplane:
    """Shallow domed flip-lid disc with a low knob."""
    lid_r = BODY_RIM_R - 0.006
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
    model = ArticulatedObject(name="electric_kettle_potbelly")

    steel = model.material("brushed_steel", rgba=(0.66, 0.67, 0.69, 1.0))
    black = model.material("black_plastic", rgba=(0.10, 0.10, 0.11, 1.0))
    dark = model.material("dark_gray", rgba=(0.18, 0.18, 0.20, 1.0))
    glass = model.material("glass_blue", rgba=(0.30, 0.40, 0.48, 0.85))

    # ============================ POWER BASE (root) ============================
    base = model.part("power_base")
    base.visual(mesh_from_cadquery(_base_solid(), "base_disc"), material=black, name="base_disc")
    # Control pad on the front rim of the base.
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

    # ============================ KETTLE BODY (pot-belly) ======================
    body = model.part("kettle_body")

    pour_cut = _pour_cut()
    body.visual(
        mesh_from_cadquery(_body_shell().cut(pour_cut), "body_shell"),
        material=steel,
        name="body_shell",
    )

    # Black top collar ring around the mouth, pour channel cut through.
    collar_outer = _body_r(COLLAR_Z) + 0.002
    collar = (
        cq.Workplane("XY")
        .workplane(offset=COLLAR_Z - 0.002)
        .circle(collar_outer)
        .extrude(BODY_TOP_Z - COLLAR_Z + 0.004)
        .faces(">Z")
        .workplane()
        .circle(_body_r(COLLAR_Z) - WALL)
        .cutThruAll()
    ).cut(pour_cut)
    body.visual(mesh_from_cadquery(collar, "top_collar"), material=black, name="top_collar")

    # Black bottom heel ring / heating plate skirt.
    heel = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .circle(BODY_BOTTOM_R + 0.002)
        .extrude(0.014)
        .edges(">Z")
        .fillet(0.004)
    )
    body.visual(mesh_from_cadquery(heel, "base_collar"), material=black, name="base_collar")

    body.visual(mesh_from_cadquery(_spout(), "spout"), material=steel, name="spout")
    body.visual(mesh_from_cadquery(_handle(), "handle"), material=black, name="handle")

    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_BELLY_R, BODY_TOP_Z),
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
    lid = model.part("lid")
    # Shift lid disc forward from hinge so positive revolute q lifts front edge.
    lid_solid = _lid_solid().translate((BODY_RIM_R - 0.006, 0.0, 0.0))
    lid.visual(mesh_from_cadquery(lid_solid, "lid_disc"), material=dark, name="lid_disc")
    # Rear hinge knuckle wrapping the pin.
    knuckle = TorusGeometry(0.006, 0.004, radial_segments=12, tubular_segments=24).rotate_x(
        math.pi / 2.0
    )
    lid.visual(mesh_from_geometry(knuckle, "hinge_knuckle"), material=black, name="hinge_knuckle")
    lid.inertial = Inertial.from_geometry(
        Cylinder(BODY_RIM_R - 0.006, 0.018),
        mass=0.05,
        origin=Origin(xyz=(BODY_RIM_R - 0.006, 0.0, 0.004)),
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("power_base")
    body = object_model.get_part("kettle_body")
    lid = object_model.get_part("lid")
    body_lift = object_model.get_articulation("body_lift")
    lid_hinge = object_model.get_articulation("lid_hinge")

    # ---- pot-belly shape: body is clearly wider than tall ----
    body_aabb = ctx.part_world_aabb(body)
    body_dx = body_aabb[1][0] - body_aabb[0][0]
    body_dy = body_aabb[1][1] - body_aabb[0][1]
    body_dz = body_aabb[1][2] - body_aabb[0][2]
    body_width = max(body_dx, body_dy)
    ctx.check(
        "pot-belly body is wider than tall",
        body_width > body_dz * 1.2,
        details=f"width={body_width:.4f}, height={body_dz:.4f}",
    )

    # ---- belly bulges out wider than 0.16m diameter ----
    shell_aabb = ctx.part_element_world_aabb(body, elem="body_shell")
    shell_dx = shell_aabb[1][0] - shell_aabb[0][0]
    ctx.check(
        "belly bulges wider than 0.16m",
        shell_dx > 0.16,
        details=f"shell_width={shell_dx:.4f}",
    )

    # ---- body heel seats on the base (heel is within base footprint) ----
    ctx.expect_within(
        body,
        base,
        axes="xy",
        inner_elem="base_collar",
        outer_elem="base_disc",
        margin=0.003,
        name="body heel centered over base",
    )
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

    # ---- body lifts straight up off the base ----
    rest = ctx.part_world_position(body)
    with ctx.pose({body_lift: LIFT}):
        lifted = ctx.part_world_position(body)
    ctx.check(
        "kettle body lifts up off the base",
        lifted[2] > rest[2] + 0.03,
        details=f"rest_z={rest[2]:.4f}, lifted_z={lifted[2]:.4f}",
    )

    # ---- pour spout at front-top (+X), above the collar ----
    spout_box = ctx.part_element_world_aabb(body, elem="spout")
    # The belly is wider than the spout at a lower Z; the spout extends
    # past the narrower rim/collar area at the front, not past the belly.
    collar_box = ctx.part_element_world_aabb(body, elem="top_collar")
    ctx.check(
        "pour spout extends past the collar at front (+X)",
        spout_box is not None and collar_box is not None
        and spout_box[1][0] > collar_box[1][0] - 0.002,
        details=f"spout_max_x={spout_box[1][0]:.4f}, collar_max_x={collar_box[1][0]:.4f}",
    )
    ctx.check(
        "pour spout sits high on the body (above collar)",
        spout_box[0][2] > COLLAR_Z - 0.01,
        details=f"spout_min_z={spout_box[0][2]:.4f}",
    )

    # ---- C-handle on rear (-X), extending past the belly ----
    handle_box = ctx.part_element_world_aabb(body, elem="handle")
    ctx.check(
        "C-handle is on the rear (-X) side, past the belly",
        handle_box is not None and handle_box[0][0] < -BODY_BELLY_R,
        details=f"handle_min_x={handle_box[0][0]:.4f}",
    )

    # ---- lid caps mouth at rest; flips open about rear hinge ----
    front_rest = ctx.part_world_aabb(lid)[1]
    front_z_rest = front_rest[2]
    front_x_rest = front_rest[0]
    with ctx.pose({lid_hinge: 1.22}):
        lid_box_open = ctx.part_world_aabb(lid)
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

    # Hinge knuckle and lid disc seated into collar: intentional capture.
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
