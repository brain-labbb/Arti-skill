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


# ---- draw-tube segment parameters (N=3 nested telescoping segments) ----------
N_DRAW_SEGMENTS = 3
# Outer radii: each segment nests inside the previous (or the rear collar bore=0.019).
_SEG_OUTER_R = (0.0180, 0.0140, 0.0105)
_SEG_BORE_R = (0.0150, 0.0110, 0.0080)
_SEG_LENGTH = (0.060, 0.055, 0.050)  # total tube length per segment
_SEG_CAPTURE = 0.020  # forward end inside parent bore at q=0
_SEG_MAX_EXTEND = (0.035, 0.030, 0.025)  # max prismatic travel per segment


def _seg_free_x(i: int) -> float:
    """Back (free) end x in segment i's local frame."""
    return -(_SEG_LENGTH[i] - _SEG_CAPTURE)


def _draw_segment_mesh(i: int) -> cq.Workplane:
    """Hollow brass draw-tube segment i, authored along +X in its own frame.

    x = +_SEG_CAPTURE is the forward captured shoulder (seats inside parent bore).
    x = _seg_free_x(i) is the back free end with a small stop-ring flare that
    retains the next smaller segment.
    """
    outer_r = _SEG_OUTER_R[i]
    bore_r = _SEG_BORE_R[i]
    x_cap = _SEG_CAPTURE
    x_free = _seg_free_x(i)
    # Parent bore radius (for stop ring sizing)
    parent_bore_r = 0.019 if i == 0 else _SEG_BORE_R[i - 1]
    # Stop ring matches parent bore radius for contact/connectivity
    stop_ring_r = parent_bore_r

    sections = [
        ("circle", x_cap, outer_r * 0.92),  # captured shoulder (rides parent bore)
        ("circle", x_cap - 0.008, outer_r),  # main tube OD
        ("circle", x_free + 0.012, outer_r),
        ("circle", x_free, stop_ring_r),  # rear stop ring (contacts parent bore)
    ]
    # Innermost segment: add rear stub connecting to eyecup position
    if i == N_DRAW_SEGMENTS - 1:
        sections.append(("circle", x_free - 0.018, outer_r * 0.8))

    body = _loft(sections)
    bore = _loft(
        [
            ("circle", x_cap + 0.010, bore_r),
            ("circle", x_free - 0.005, bore_r),
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
    # DRAW-SEGMENT CHAIN (N=3 nested telescoping brass tubes).
    # Segment 0 slides out of the rear collar bore; segment i+1 slides out of
    # segment i; the eyecup rides on the innermost segment (segment N-1).
    # Each segment's frame origin is at its captured shoulder (forward end),
    # which seats inside its parent bore at q=0.
    # =====================================================================
    segments = []
    for i in range(N_DRAW_SEGMENTS):
        seg = model.part(f"draw_segment_{i}")
        seg.visual(
            mesh_from_cadquery(_draw_segment_mesh(i), f"draw_segment_{i}"),
            material=brass,
            name=f"draw_segment_{i}",
        )
        # Innermost segment carries the eyecup just past its rear stop ring.
        if i == N_DRAW_SEGMENTS - 1:
            seg.visual(
                mesh_from_geometry(
                    TorusGeometry(
                        radius=_SEG_OUTER_R[i] * 1.05,
                        tube=_SEG_OUTER_R[i] * 0.28,
                        radial_segments=12,
                        tubular_segments=36,
                    ).rotate_y(math.pi / 2.0),
                    "eyecup",
                ),
                origin=Origin(xyz=(_seg_free_x(i) - 0.018, 0.0, 0.0)),
                material=brass,
                name="eyecup",
            )
        seg.inertial = Inertial.from_geometry(
            Box((_SEG_LENGTH[i] + 0.02, 2.0 * _SEG_OUTER_R[i] + 0.01, 2.0 * _SEG_OUTER_R[i] + 0.01)),
            mass=0.040 - 0.010 * i,
            origin=Origin(xyz=(_seg_free_x(i) / 2.0, 0.0, 0.0)),
        )
        segments.append(seg)

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
    # Draw-segment chain: each segment slides out the back (-X) of its parent.
    # Segment 0 parent = tube (slides out of the rear collar bore).
    # Segment i (i>0) parent = segment i-1 (slides out of segment i-1's bore).
    _draw_parents = [tube] + segments[:-1]
    _draw_joints = []
    for i in range(N_DRAW_SEGMENTS):
        parent_part = _draw_parents[i]
        child_part = segments[i]
        # Origin: where the child segment's local frame sits at q=0.
        if i == 0:
            # Seg0 shoulder (local x=0.020) seats inside the rear collar bore.
            # Offset so the stop ring sits inside the bore wall region for contact.
            joint_origin = (TUBE_BACK_X + 0.035, 0.0, 0.0)
        else:
            # Seg i shoulder sits at the forward entrance of parent seg i-1's bore.
            joint_origin = (_SEG_CAPTURE - 0.010, 0.0, 0.0)
        jname = f"draw_segment_{i}_extend"
        model.articulation(
            jname,
            ArticulationType.PRISMATIC,
            parent=parent_part,
            child=child_part,
            origin=Origin(xyz=joint_origin),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.1,
                lower=0.0,
                upper=_SEG_MAX_EXTEND[i],
            ),
        )
        _draw_joints.append(jname)

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
    draw_segs = [object_model.get_part(f"draw_segment_{i}") for i in range(N_DRAW_SEGMENTS)]
    draw_joints = [object_model.get_articulation(f"draw_segment_{i}_extend") for i in range(N_DRAW_SEGMENTS)]

    azimuth = object_model.get_articulation("azimuth_rotation")
    tilt = object_model.get_articulation("altitude_tilt")

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

    # ---- Draw-segment chain: N nested telescoping brass tubes. ----
    # Each segment nests inside its parent (telescoping fit, intentional overlap).
    # Segment 0 nests inside the tube's rear collar bore.
    ctx.allow_overlap(
        draw_segs[0],
        tube,
        elem_a="draw_segment_0",
        elem_b="rear_collar",
        reason="Outermost draw segment nests inside the rear brass collar bore (telescoping fit).",
    )
    ctx.allow_overlap(
        draw_segs[0],
        tube,
        elem_a="draw_segment_0",
        elem_b="leather_body",
        reason="Outermost draw segment extends through the leather body bore when retracted.",
    )
    # Segments i>0 nest inside segment i-1's bore.
    for i in range(1, N_DRAW_SEGMENTS):
        ctx.allow_overlap(
            draw_segs[i],
            draw_segs[i - 1],
            elem_a=f"draw_segment_{i}",
            elem_b=f"draw_segment_{i - 1}",
            reason=f"Draw segment {i} nests inside segment {i - 1}'s bore (telescoping fit).",
        )

    # Extending all segments: eyecup on the innermost segment moves further out.
    last_seg = draw_segs[N_DRAW_SEGMENTS - 1]
    eye_rest = _elem_center(ctx, last_seg, "eyecup")
    # Extend all segments to their max simultaneously.
    ext_pose = {draw_joints[i]: _SEG_MAX_EXTEND[i] for i in range(N_DRAW_SEGMENTS)}
    with ctx.pose(ext_pose):
        eye_out = _elem_center(ctx, last_seg, "eyecup")
    total_travel = sum(_SEG_MAX_EXTEND)
    ctx.check(
        "draw-segment chain extends the eyecup out the back",
        eye_rest is not None
        and eye_out is not None
        and (eye_rest[0] - eye_out[0]) > total_travel * 0.85,
        details=f"rest_x={eye_rest}, extended_x={eye_out}, expected_travel>={total_travel * 0.85:.3f}",
    )

    # Each segment exists and is nested behind the previous at rest.
    for i in range(N_DRAW_SEGMENTS):
        seg_pos = ctx.part_world_position(draw_segs[i])
        ctx.check(
            f"draw_segment_{i} exists at tube height",
            seg_pos is not None and seg_pos[2] > 0.40,
            details=f"draw_segment_{i} pos={seg_pos}",
        )

    # Innermost segment is behind (more -X than) the outermost at rest.
    rest_positions = [ctx.part_world_position(seg) for seg in draw_segs]
    ctx.check(
        "draw segments are nested (inner behind outer along -X)",
        all(
            rest_positions[i][0] <= rest_positions[i - 1][0] + 0.01
            for i in range(1, N_DRAW_SEGMENTS)
            if rest_positions[i] is not None and rest_positions[i - 1] is not None
        ),
        details=f"positions={rest_positions}",
    )

    return ctx.report()


object_model = build_object_model()
