from __future__ import annotations

# Electric kettle on a separate round power base, with a side-mounted swinging
# carry loop (D-shaped grip ring hinged to two lugs on the rear quarter).
# Frame: Z up (kettle axis vertical), kettle centerline at x=y=0.
#   - front (pour spout) at +X, rear (lid hinge) at -X.
#   - carry loop lugs on the +Y rear quarter (~135 deg from +X).
# Root: power_base (the black round mains base sitting on the table).
# Articulations (INDEPENDENT user-facing mechanisms):
#   - body_lift: PRISMATIC, kettle body lifts straight up off the base (+Z ~0.04m).
#   - lid_hinge: REVOLUTE, rear-hinged flip lid; positive q lifts the front
#     edge of the lid up and back about the rear hinge line (~0..70 deg).
#   - loop_pivot: REVOLUTE, D-ring carry loop swings from folded flat against
#     the body side (q=0) to deployed outward for carrying (q ~ pi/2).

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
    tube_from_spline_points,
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

# ---- carry loop dimensions ---------------------------------------------------
LUG_ANGLE = math.radians(135)  # rear quarter on +Y side (135 deg from +X)
LUG_Z_UPPER = 0.155  # upper lug height (just below collar)
LUG_Z_LOWER = 0.085  # lower lug height (mid-body)
LUG_MID_Z = (LUG_Z_UPPER + LUG_Z_LOWER) / 2.0  # pivot midpoint z = 0.120
HALF_SPAN = (LUG_Z_UPPER - LUG_Z_LOWER) / 2.0  # half lug spacing = 0.035
LUG_HEIGHT = 0.010  # lug protrusion from body surface
LUG_R_OUTER = BODY_R + LUG_HEIGHT  # radial distance of lug outer face
LUG_RADIUS = 0.009  # lug boss cross-section radius
TUBE_R = 0.005  # D-ring tube cross-section radius
PIN_R = 0.003  # pivot pin radius
PIN_LENGTH = 0.005  # pivot pin protrusion into lug

# Pivot point: outer face of lugs, midpoint between upper and lower
PIVOT_X = LUG_R_OUTER * math.cos(LUG_ANGLE)
PIVOT_Y = LUG_R_OUTER * math.sin(LUG_ANGLE)
PIVOT_Z = LUG_MID_Z


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
    # spout opens straight through into the kettle cavity.
    return (
        cq.Workplane("XY")
        .workplane(offset=COLLAR_Z + 0.002)
        .center(BODY_R - 0.006, 0.0)
        .rect(0.060, 0.020)
        .extrude(BODY_TOP_Z - COLLAR_Z + 0.030)
    )


def _lug_solid(z_center: float) -> cq.Workplane:
    """Single cylindrical lug boss protruding radially from the body surface.

    Built as a cylinder along +X at the given Z, then rotated to the lug angle.
    The lug base overlaps slightly with the body shell to ensure connectivity.
    """
    # Start the lug slightly inside the body (at BODY_R - 0.004) to ensure
    # geometric contact with the body shell for connectivity
    lug_start = BODY_R - 0.004
    lug_length = LUG_HEIGHT + 0.004  # extend further to compensate for overlap

    # Cylinder in YZ plane, centered at (y=0, z=z_center), extruded along +X
    lug = (
        cq.Workplane("YZ")
        .center(0.0, z_center)
        .circle(LUG_RADIUS)
        .extrude(lug_length)
        .translate((lug_start, 0.0, 0.0))
    )
    # Fillet the outer face edges for a rounded boss look
    lug = lug.edges(">X").fillet(0.002)
    # Rotate from +X to the lug radial direction (angle measured from +X)
    lug = lug.rotate((0, 0, 0), (0, 0, 1), math.degrees(LUG_ANGLE))
    return lug


def _carry_loop_solid():
    """D-shaped carry ring in its local part frame.

    Local frame: origin at pivot midpoint (between the two lugs),
    +Z along pivot axis (vertical), +X outward (curve direction),
    +Y perpendicular.

    The D-ring consists of:
    - Straight bar along Z from -HALF_SPAN to +HALF_SPAN (the flat side of D)
    - Semicircular curve in XZ plane from (0,0,+HALF_SPAN) through
      (+HALF_SPAN,0,0) to (0,0,-HALF_SPAN) (the curved side of D)
    - Pivot pins at top and bottom extending along +/-Z into the lugs

    Returns a MeshGeometry that can be used with mesh_from_geometry.
    """
    # Build the semicircular curve using tube_from_spline_points
    # Sample points along the semicircle in the XZ plane (Y=0)
    n_points = 9
    curve_points = []
    for i in range(n_points):
        angle = (i / (n_points - 1)) * math.pi  # 0 to π
        x = HALF_SPAN * math.sin(angle)
        z = HALF_SPAN * math.cos(angle)
        curve_points.append((x, 0.0, z))

    # Build the curved tube section
    curved = tube_from_spline_points(
        curve_points,
        radius=TUBE_R,
        samples_per_segment=8,
        radial_segments=12,
        cap_ends=True,
        up_hint=(0.0, 1.0, 0.0),  # Y is "up" for the curve frame
    )

    # Build the straight bar along Z
    bar = CylinderGeometry(TUBE_R, HALF_SPAN * 2, radial_segments=12, closed=True)
    bar.translate(0.0, 0.0, 0.0)  # centered at origin, spans -HALF_SPAN to +HALF_SPAN

    # Build pivot pins at top and bottom
    upper_pin = CylinderGeometry(PIN_R, PIN_LENGTH, radial_segments=8, closed=True)
    upper_pin.translate(0.0, 0.0, HALF_SPAN + PIN_LENGTH / 2)

    lower_pin = CylinderGeometry(PIN_R, PIN_LENGTH, radial_segments=8, closed=True)
    lower_pin.translate(0.0, 0.0, -HALF_SPAN - PIN_LENGTH / 2)

    # Merge all parts into one mesh
    result = curved.merge(bar)
    result = result.merge(upper_pin)
    result = result.merge(lower_pin)

    return result


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

    # Black top collar ring (the dark band around the mouth), with the front
    # pour channel cut through so the spout opens into the cavity.
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

    # Carry loop mounting lugs: two cylindrical bosses on the rear quarter of
    # the body (+Y side, at 135 deg from +X). Each lug protrudes radially
    # outward from the body surface to receive the D-ring pivot pins.
    lug_z_positions = [LUG_Z_LOWER, LUG_Z_UPPER]
    for i in range(2):
        body.visual(
            mesh_from_cadquery(_lug_solid(lug_z_positions[i]), f"lug_{i}"),
            material=black,
            name=f"lug_{i}",
        )

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

    # ============================ CARRY LOOP ==================================
    # D-shaped grip ring hinged to the two lugs on the rear quarter of the body.
    # At q=0: loop folded flat against the body side (curve tangent to body).
    # At q=pi/2: loop deployed outward (curve extends radially from body).
    # The articulation frame is rotated by yaw=pi/4 so that local +X aligns
    # with the body tangent direction at the lug position (135 deg from +X).
    carry_loop = model.part("carry_loop")
    carry_loop.visual(
        mesh_from_geometry(_carry_loop_solid(), "d_ring"),
        material=black,
        name="d_ring",
    )
    carry_loop.inertial = Inertial.from_geometry(
        Cylinder(TUBE_R * 3, HALF_SPAN * 2),
        mass=0.04,
        origin=Origin(xyz=(HALF_SPAN * 0.5, 0.0, 0.0)),
    )

    model.articulation(
        "loop_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=carry_loop,
        origin=Origin(
            xyz=(PIVOT_X, PIVOT_Y, PIVOT_Z),
            rpy=(0.0, 0.0, math.pi / 4.0),
        ),
        # Pivot axis is vertical (Z); positive q swings the D-ring outward.
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=0.0, upper=math.pi / 2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("power_base")
    body = object_model.get_part("kettle_body")
    lid = object_model.get_part("lid")
    carry_loop = object_model.get_part("carry_loop")
    body_lift = object_model.get_articulation("body_lift")
    lid_hinge = object_model.get_articulation("lid_hinge")
    loop_pivot = object_model.get_articulation("loop_pivot")

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

    # ---- carry loop lugs are on the rear quarter (-X, +Y side) ----
    for i in range(2):
        lug_box = ctx.part_element_world_aabb(body, elem=f"lug_{i}")
        ctx.check(
            f"lug_{i} is on the rear quarter",
            lug_box is not None and lug_box[0][0] < 0.0 and lug_box[1][1] > 0.0,
            details=f"lug_{i} min_x={lug_box[0][0]:.4f}, max_y={lug_box[1][1]:.4f}",
        )

    # ---- carry loop is a separate articulated part (not fixed) ----
    ctx.check(
        "carry loop has a revolute pivot joint",
        loop_pivot is not None
        and loop_pivot.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={loop_pivot.articulation_type if loop_pivot else None}",
    )

    # ---- at q=0 (folded), loop lies near the body ----
    loop_rest_center = ctx.part_world_position(carry_loop)
    body_center = ctx.part_world_position(body)
    ctx.check(
        "carry loop folds near the body at rest (q=0)",
        loop_rest_center is not None
        and body_center is not None
        and abs(loop_rest_center[0] - body_center[0]) < BODY_R + 0.05
        and abs(loop_rest_center[1] - body_center[1]) < BODY_R + 0.05,
        details=f"loop={loop_rest_center}, body={body_center}",
    )

    # ---- at q=upper (deployed), loop extends further outward from the body ----
    rest_aabb = ctx.part_world_aabb(carry_loop)
    with ctx.pose({loop_pivot: math.pi / 2.0}):
        deployed_aabb = ctx.part_world_aabb(carry_loop)

    # Measure the maximum radial extent from the body centerline (z-axis)
    def _max_radial_extent(aabb):
        corners = [
            (aabb[0][0], aabb[0][1]),
            (aabb[0][0], aabb[1][1]),
            (aabb[1][0], aabb[0][1]),
            (aabb[1][0], aabb[1][1]),
        ]
        return max(math.sqrt(x * x + y * y) for x, y in corners)

    rest_extent = _max_radial_extent(rest_aabb)
    deployed_extent = _max_radial_extent(deployed_aabb)
    ctx.check(
        "carry loop extends further outward when deployed",
        deployed_extent > rest_extent + 0.005,
        details=f"rest_radial={rest_extent:.4f}, deployed_radial={deployed_extent:.4f}",
    )

    # ---- pivot pins seat into the lugs (intentional capture) ----
    ctx.allow_overlap(
        carry_loop,
        body,
        elem_a="d_ring",
        elem_b="lug_0",
        reason="Lower pivot pin is intentionally captured in the lower lug bore.",
    )
    ctx.allow_overlap(
        carry_loop,
        body,
        elem_a="d_ring",
        elem_b="lug_1",
        reason="Upper pivot pin is intentionally captured in the upper lug bore.",
    )
    # Prove pins contact the lugs at the pivot
    ctx.expect_contact(
        carry_loop,
        body,
        elem_a="d_ring",
        elem_b="lug_0",
        name="carry loop lower pin contacts lower lug",
    )
    ctx.expect_contact(
        carry_loop,
        body,
        elem_a="d_ring",
        elem_b="lug_1",
        name="carry loop upper pin contacts upper lug",
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
