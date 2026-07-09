from __future__ import annotations

# Brass & leather refractor spyglass telescope on a tall wooden tripod.
#
# Frame: optical axis along +X. The wide brass OBJECTIVE ring is at the front
# (+X); the thin brass eyepiece DRAW-TUBE is at the back (-X). The optical
# centerline sits at tube height z = TUBE_Z above the floor (z = 0).
#
# Parts / articulations:
#   - tripod (root): three slim dark-wood legs from a brass hub, brass-tipped
#     feet on the floor, a triangular brass spreader, and the fixed brass
#     pedestal collar that carries the rotating head.
#   - azimuth_head: brass yoke base + dark iron trunnion fork. CONTINUOUS rotation
#     about the vertical (Z) axis at the hub -> tube swings left/right.
#   - tube: leather-wrapped tapering body, brass objective ring + lens, brass
#     rear collar. REVOLUTE tilt about the horizontal yoke axis (Y), -30..+45deg.
#   - draw_tube: thin brass eyepiece draw-tube + eyecup. PRISMATIC extend
#     (~0.05 m) out the back of the tube along -X.

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
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---- key dimensions (meters) --------------------------------------------------
TUBE_Z = 0.60  # optical centerline height above the floor (world)
HUB_Z = 0.500  # brass leg hub / azimuth bearing height (world)
AZ_JOINT_Z = 0.530  # world height of the azimuth bearing face (head local z=0)
PIVOT_LOCAL_Z = TUBE_Z - AZ_JOINT_Z  # tilt pivot height in head-local frame (=0.07)
TUBE_FRONT_X = 0.225  # objective end of the leather body
TUBE_BACK_X = -0.205  # rear collar end of the leather body
PIVOT_X = -0.02  # yoke trunnion sits just behind the tube center of mass


def _loft(sections) -> cq.Workplane:
    # sections: list of ("circle", x, r) along +X (YZ planes).
    wp = cq.Workplane("YZ")
    prev = 0.0
    for i, s in enumerate(sections):
        x = s[1]
        off = x if i == 0 else x - prev
        if i > 0 or abs(off) > 1e-12:
            wp = wp.workplane(offset=off)
        wp = wp.circle(s[2])
        prev = x
    return wp.loft(ruled=False)


def _leather_body() -> cq.Workplane:
    # Tapered hollow body: wide at the objective (+X), narrowing toward the rear.
    # A short rear neck mates with the brass rear collar.
    outer = _loft(
        [
            ("circle", TUBE_BACK_X, 0.026),
            ("circle", TUBE_BACK_X + 0.030, 0.029),
            ("circle", -0.10, 0.033),
            ("circle", 0.02, 0.039),
            ("circle", 0.14, 0.044),
            ("circle", TUBE_FRONT_X, 0.046),
        ]
    )
    inner = _loft(
        [
            # Rear bore widened so the brass draw-tube nests inside the body cleanly.
            ("circle", TUBE_BACK_X - 0.02, 0.0225),
            ("circle", -0.10, 0.027),
            ("circle", 0.02, 0.034),
            ("circle", 0.14, 0.039),
            ("circle", TUBE_FRONT_X + 0.01, 0.041),
        ]
    )
    return outer.cut(inner)


def _objective_ring() -> cq.Workplane:
    # Heavy polished brass dew-shield / objective bezel that caps the wide front.
    ring = _loft(
        [
            ("circle", TUBE_FRONT_X - 0.012, 0.047),
            ("circle", TUBE_FRONT_X + 0.006, 0.052),
            ("circle", TUBE_FRONT_X + 0.030, 0.052),
            ("circle", TUBE_FRONT_X + 0.034, 0.049),
        ]
    )
    bore = _loft(
        [
            ("circle", TUBE_FRONT_X - 0.02, 0.043),
            ("circle", TUBE_FRONT_X + 0.040, 0.047),
        ]
    )
    return ring.cut(bore)


def _rear_collar() -> cq.Workplane:
    # Brass band where the leather body ends and the draw-tube emerges.
    band = _loft(
        [
            ("circle", TUBE_BACK_X - 0.012, 0.024),
            ("circle", TUBE_BACK_X, 0.027),
            ("circle", TUBE_BACK_X + 0.022, 0.027),
            ("circle", TUBE_BACK_X + 0.026, 0.023),
        ]
    )
    bore = _loft(
        [
            ("circle", TUBE_BACK_X - 0.02, 0.019),
            ("circle", TUBE_BACK_X + 0.04, 0.019),
        ]
    )
    return band.cut(bore)


def _draw_tube_solid() -> cq.Workplane:
    # Thin brass draw-tube (slides inside the rear collar) ending in an eyecup.
    # Built along +X in its own frame; the back-most ring stays captured inside
    # the body even at full extension.
    x0 = 0.0  # deepest captured end (inside the body)
    body = _loft(
        [
            ("circle", x0 + 0.020, 0.0205),  # captured shoulder rides the collar bore
            ("circle", x0, 0.0185),
            ("circle", x0 - 0.150, 0.0185),
            ("circle", x0 - 0.165, 0.020),
            ("circle", x0 - 0.180, 0.024),  # eyecup flare
            ("circle", x0 - 0.190, 0.022),
        ]
    )
    bore = _loft(
        [
            ("circle", x0 + 0.030, 0.014),
            ("circle", x0 - 0.200, 0.016),
        ]
    )
    return body.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="brass_leather_spyglass_telescope")

    brass = model.material("polished_brass", rgba=(0.80, 0.62, 0.22, 1.0))
    leather = model.material("brown_leather", rgba=(0.46, 0.20, 0.12, 1.0))
    wood = model.material("dark_wood", rgba=(0.27, 0.16, 0.11, 1.0))
    iron = model.material("dark_iron", rgba=(0.16, 0.16, 0.18, 1.0))
    glass = model.material("objective_glass", rgba=(0.55, 0.68, 0.74, 0.55))

    # =====================================================================
    # TRIPOD (root): brass hub, three dark-wood legs, brass feet, spreader,
    # and the fixed brass pedestal collar that the head rotates on.
    # =====================================================================
    tripod = model.part("tripod")

    # Brass hub at the top of the legs.
    tripod.visual(
        Cylinder(radius=0.030, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, HUB_Z + 0.000)),
        material=brass,
        name="leg_hub",
    )
    # Fixed brass pedestal collar (lower race of the azimuth bearing), topping out
    # at the azimuth bearing face (AZ_JOINT_Z).
    tripod.visual(
        Cylinder(radius=0.034, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, AZ_JOINT_Z - 0.012)),
        material=brass,
        name="pedestal_collar",
    )

    # Three slim wooden legs splaying from just under the hub to the floor.
    leg_angles = (
        math.pi / 2.0,
        math.pi / 2.0 + 2.0 * math.pi / 3.0,
        math.pi / 2.0 + 4.0 * math.pi / 3.0,
    )
    foot_r = 0.165  # horizontal footprint radius (widest at the feet)
    # Leg waypoint at the spreader height (used to anchor spreader endpoints onto
    # the actual leg centerline so the triangle stays connected to the legs).
    spread_r = 0.105
    spread_z = HUB_Z * 0.40
    for index, angle in enumerate(leg_angles):
        c = math.cos(angle)
        s = math.sin(angle)
        leg_mesh = tube_from_spline_points(
            [
                (0.020 * c, 0.020 * s, HUB_Z + 0.005),
                (0.060 * c, 0.060 * s, HUB_Z * 0.70),
                (spread_r * c, spread_r * s, spread_z),
                (foot_r * c, foot_r * s, 0.012),
            ],
            radius=0.0095,
            samples_per_segment=18,
            radial_segments=16,
            cap_ends=True,
        )
        tripod.visual(
            mesh_from_geometry(leg_mesh, f"tripod_leg_{index}"),
            material=wood,
            name=f"leg_{index}",
        )
        # Brass foot tip resting on the floor.
        tripod.visual(
            Cylinder(radius=0.0125, length=0.026),
            origin=Origin(xyz=(foot_r * c, foot_r * s, 0.013)),
            material=brass,
            name=f"foot_{index}",
        )

    # Triangular brass spreader linking the three legs at the spreader height.
    # Endpoints sit on the leg centerline (the leg passes through spread_r/spread_z).
    pts = []
    for angle in leg_angles:
        pts.append((spread_r * math.cos(angle), spread_r * math.sin(angle), spread_z))
    for index in range(3):
        a = pts[index]
        b = pts[(index + 1) % 3]
        bar = tube_from_spline_points(
            [a, ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0, spread_z), b],
            radius=0.0050,
            samples_per_segment=10,
            radial_segments=12,
            cap_ends=True,
        )
        tripod.visual(
            mesh_from_geometry(bar, f"spreader_bar_{index}"),
            material=brass,
            name=f"spreader_{index}",
        )

    tripod.inertial = Inertial.from_geometry(
        Box((0.40, 0.40, HUB_Z + 0.05)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, HUB_Z * 0.45)),
    )

    # =====================================================================
    # AZIMUTH HEAD: brass yoke base + dark iron trunnion fork. Rotates about Z.
    # =====================================================================
    azimuth_head = model.part("azimuth_head")

    # Brass rotating ring riding on the pedestal collar (head-local z=0 = bearing
    # face). Its top reaches up to meet the brass riser block.
    azimuth_head.visual(
        Cylinder(radius=0.032, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=brass,
        name="azimuth_ring",
    )
    # Brass riser block carrying the trunnion fork. Wide enough in Y to seat the
    # two outboard fork plates that straddle the tube.
    PLATE_Y = 0.050  # fork plates sit outboard of the tube body (radius ~0.046)
    azimuth_head.visual(
        Box((0.034, 2.0 * PLATE_Y + 0.008, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=brass,
        name="yoke_block",
    )
    # Two tall dark iron trunnion plates forming the fork that grips the tube
    # pivot from the sides (outboard of the leather body, rising to the axle).
    plate_bottom = 0.024
    plate_h = (PIVOT_LOCAL_Z + 0.006) - plate_bottom
    for side, yy in (("0", PLATE_Y), ("1", -PLATE_Y)):
        azimuth_head.visual(
            Box((0.024, 0.007, plate_h)),
            origin=Origin(xyz=(PIVOT_X, yy, plate_bottom + plate_h / 2.0)),
            material=iron,
            name=f"trunnion_plate_{side}",
        )
    # Brass pivot bolt / tilt axle spanning between the fork plates at the tilt axis.
    azimuth_head.visual(
        Cylinder(radius=0.007, length=2.0 * PLATE_Y + 0.008),
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_LOCAL_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pivot_axle",
    )

    azimuth_head.inertial = Inertial.from_geometry(
        Box((0.07, 0.07, 0.10)),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
    )

    # =====================================================================
    # TUBE: leather body + brass objective ring/lens + brass rear collar.
    # Tilts about the horizontal yoke (Y) axis. Authored in tube-local frame
    # with the pivot at the origin (z up = yoke height).
    # =====================================================================
    tube = model.part("tube")

    tube.visual(
        mesh_from_cadquery(_leather_body(), "leather_body"),
        material=leather,
        name="leather_body",
    )
    tube.visual(
        mesh_from_cadquery(_objective_ring(), "objective_ring"),
        material=brass,
        name="objective_ring",
    )
    # Objective lens disc seated inside the brass front ring.
    tube.visual(
        mesh_from_geometry(
            CylinderGeometry(0.0455, 0.008, radial_segments=48).rotate_y(math.pi / 2.0),
            "objective_lens",
        ),
        origin=Origin(xyz=(TUBE_FRONT_X + 0.008, 0.0, 0.0)),
        material=glass,
        name="objective_lens",
    )
    tube.visual(
        mesh_from_cadquery(_rear_collar(), "rear_collar"),
        material=brass,
        name="rear_collar",
    )
    # A pair of dark iron saddle ears protruding from the tube sides at the pivot
    # axis; the trunnion fork plates grip them and the axle bolt passes through.
    for side, yy in (("0", 0.041), ("1", -0.041)):
        tube.visual(
            Box((0.024, 0.024, 0.022)),
            origin=Origin(xyz=(PIVOT_X, yy, 0.0)),
            material=iron,
            name=f"saddle_lug_{side}",
        )
    # Small off-axis brass focus knob so azimuth spin is visibly detectable.
    tube.visual(
        Sphere(radius=0.0085),
        origin=Origin(xyz=(0.10, 0.0, 0.050)),
        material=brass,
        name="focus_knob",
    )

    tube.inertial = Inertial.from_geometry(
        Box((0.43, 0.10, 0.10)),
        mass=0.90,
        origin=Origin(xyz=(0.01, 0.0, 0.0)),
    )

    # =====================================================================
    # DRAW-TUBE: thin brass eyepiece tube that slides out the back along -X.
    # Authored so x=0 (its captured end) lands at the rear collar bore.
    # =====================================================================
    draw_tube = model.part("draw_tube")
    draw_tube.visual(
        mesh_from_cadquery(_draw_tube_solid(), "draw_tube"),
        material=brass,
        name="draw_tube",
    )
    # Brass eyecup ring at the very back.
    draw_tube.visual(
        mesh_from_geometry(
            TorusGeometry(
                radius=0.020, tube=0.005, radial_segments=14, tubular_segments=40
            ).rotate_y(math.pi / 2.0),
            "eyecup",
        ),
        origin=Origin(xyz=(-0.192, 0.0, 0.0)),
        material=brass,
        name="eyecup",
    )
    draw_tube.inertial = Inertial.from_geometry(
        Box((0.21, 0.05, 0.05)),
        mass=0.10,
        origin=Origin(xyz=(-0.10, 0.0, 0.0)),
    )

    # =====================================================================
    # ARTICULATIONS
    # =====================================================================
    # Azimuth: continuous spin about vertical Z at the hub.
    model.articulation(
        "azimuth_rotation",
        ArticulationType.CONTINUOUS,
        parent=tripod,
        child=azimuth_head,
        origin=Origin(xyz=(0.0, 0.0, AZ_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0),
    )
    # Altitude tilt: revolute about the horizontal yoke axis (Y). +angle about
    # -Y raises the +X (objective) end of the tube.
    model.articulation(
        "altitude_tilt",
        ArticulationType.REVOLUTE,
        parent=azimuth_head,
        child=tube,
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_LOCAL_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=-math.radians(30.0),
            upper=math.radians(45.0),
        ),
    )
    # Draw-tube: prismatic extension out the back (-X) of the tube.
    model.articulation(
        "drawtube_extend",
        ArticulationType.PRISMATIC,
        parent=tube,
        child=draw_tube,
        # x=0 (draw-tube shoulder) seats inside the rear brass collar bore.
        origin=Origin(xyz=(TUBE_BACK_X - 0.005, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.1, lower=0.0, upper=0.050),
    )

    return model


def _elem_center(ctx, part, elem):
    aabb = ctx.part_element_world_aabb(part, elem=elem)
    if aabb is None:
        return None
    mn, mx = aabb
    return ((mn[0] + mx[0]) / 2.0, (mn[1] + mx[1]) / 2.0, (mn[2] + mx[2]) / 2.0)


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tripod = object_model.get_part("tripod")
    head = object_model.get_part("azimuth_head")
    tube = object_model.get_part("tube")
    draw = object_model.get_part("draw_tube")

    azimuth = object_model.get_articulation("azimuth_rotation")
    tilt = object_model.get_articulation("altitude_tilt")
    extend = object_model.get_articulation("drawtube_extend")

    # ---- Tripod: three feet on the floor, widest footprint at the bottom. ----
    for i in range(3):
        foot_pos = _elem_center(ctx, tripod, f"foot_{i}")
        ctx.check(
            f"foot_{i} rests near the floor",
            foot_pos is not None and foot_pos[2] < 0.05,
            details=f"foot_{i}={foot_pos}",
        )
    foot0 = _elem_center(ctx, tripod, "foot_0")
    foot_radius = math.hypot(foot0[0], foot0[1]) if foot0 is not None else 0.0
    ctx.check(
        "feet splay out to the widest footprint",
        foot_radius > 0.12,
        details=f"foot radius={foot_radius}",
    )

    # ---- Tube is held in the yoke, up at optical height (not floating). ----
    ctx.expect_contact(head, tripod, name="head sits on tripod pedestal")
    tube_pos = ctx.part_world_position(tube)
    ctx.check(
        "tube held at the yoke height",
        tube_pos is not None and tube_pos[2] > 0.55,
        details=f"tube origin={tube_pos}",
    )
    # The trunnion fork grips the tube saddle lugs (intentional capture).
    ctx.allow_overlap(
        tube,
        head,
        elem_a="saddle_lug_0",
        elem_b="trunnion_plate_0",
        reason="Tube saddle lugs are captured between the trunnion fork plates at the pivot.",
    )
    ctx.allow_overlap(
        tube,
        head,
        elem_a="saddle_lug_1",
        elem_b="trunnion_plate_1",
        reason="Tube saddle lugs are captured between the trunnion fork plates at the pivot.",
    )
    # The brass trunnion bolt passes through the tube saddle at the optical axis.
    ctx.allow_overlap(
        head,
        tube,
        elem_a="pivot_axle",
        elem_b="leather_body",
        reason="The trunnion pivot bolt runs through the tube saddle/body on the tilt axis.",
    )
    ctx.expect_overlap(
        head,
        tube,
        axes="y",
        min_overlap=0.01,
        elem_a="pivot_axle",
        elem_b="leather_body",
        name="pivot bolt passes through the tube saddle",
    )

    # ---- Altitude tilt: front of the tube rises when tilted up. ----
    front_rest = _elem_center(ctx, tube, "objective_ring")
    with ctx.pose({tilt: math.radians(45.0)}):
        front_up = _elem_center(ctx, tube, "objective_ring")
    ctx.check(
        "tilting up raises the objective end",
        front_up[2] > front_rest[2] + 0.05,
        details=f"rest_z={front_rest[2]}, up_z={front_up[2]}",
    )

    # ---- Azimuth: spinning the head swings the off-axis focus knob sideways. ----
    knob_rest = _elem_center(ctx, tube, "focus_knob")
    with ctx.pose({azimuth: math.pi / 2.0}):
        knob_spun = _elem_center(ctx, tube, "focus_knob")
    swing = math.hypot(knob_spun[0] - knob_rest[0], knob_spun[1] - knob_rest[1])
    ctx.check(
        "azimuth rotation swings the head to a new heading",
        swing > 0.05,
        details=f"rest={knob_rest}, spun={knob_spun}, swing={swing}",
    )

    # ---- Draw-tube: extending grows the overall tube length along -X. ----
    ctx.allow_overlap(
        draw,
        tube,
        elem_a="draw_tube",
        elem_b="rear_collar",
        reason="Draw-tube stays captured inside the rear brass collar bore as it slides.",
    )
    ctx.allow_overlap(
        draw,
        tube,
        elem_a="draw_tube",
        elem_b="leather_body",
        reason="Draw-tube nests inside the main tube's rear bore (telescoping fit).",
    )
    eye_rest = _elem_center(ctx, draw, "eyecup")
    with ctx.pose({extend: 0.050}):
        eye_out = _elem_center(ctx, draw, "eyecup")
    ctx.check(
        "draw-tube extends out the back of the tube",
        (eye_rest[0] - eye_out[0]) > 0.045,
        details=f"rest_x={eye_rest[0]}, extended_x={eye_out[0]}",
    )

    return ctx.report()


object_model = build_object_model()
