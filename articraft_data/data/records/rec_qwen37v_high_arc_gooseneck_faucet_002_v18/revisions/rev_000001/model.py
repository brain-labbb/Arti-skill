from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    LatheGeometry,
    MeshGeometry,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# High-arc gooseneck pre-rinse kitchen faucet, ~0.45 m tall.
#
# Variant of the brushed-gold single-hole faucet with:
# - A slim commercial pre-rinse helical spring wrapping the gooseneck arc
# - Shallow ribbing on the pull-down spray head
# - A single side lever on a revolute joint
#
# Layout (world frame, deck plane at z = 0):
# - +X is the front (direction the gooseneck reaches), +Z is up.
# - Tapered column rises on Z to z = 0.307.
# - Gooseneck spout swivels about the vertical column axis (+/-60 deg).
# - Pre-rinse spring coils around the arc portion of the gooseneck.
# - Pull-down spray head with shallow ribbing hangs at spout tip.
# - Side lever on right (-Y) side at mid-height, revolute +/-45 deg.
# - Control pod with dial on front face.
# ---------------------------------------------------------------------------

# Column
COLUMN_BASE_R = 0.030
COLUMN_MID_R = 0.020
COLUMN_TOP_R = 0.0155
COLUMN_MID_Z = 0.18
COLUMN_TOP_Z = 0.295
COLLAR_R = 0.0175
COLLAR_LEN = 0.012
SWIVEL_Z = 0.307

# Gooseneck (spout-local coordinates, frame at the top of the collar)
TUBE_R = 0.012
RISER_TOP = 0.053
ARC_R = 0.085
REACH_X = 2.0 * ARC_R  # 0.17 m
DROP_END = 0.003

# Pre-rinse spring
SPRING_WIRE_R = 0.0018
SPRING_COIL_R = TUBE_R + SPRING_WIRE_R * 0.3  # seats against the tube surface
SPRING_COILS = 10
SPRING_SAMPLES_PER_COIL = 20

# Pull-down stages
STAGE_TRAVEL = 0.060
SLEEVE_R = 0.0075
SLEEVE_LEN = 0.072
INNER_HOSE_R = 0.0048

# Spray head (ribbed)
HEAD_LEN = 0.100
NOZZLE_R = 0.0095
HEAD_RIB_COUNT = 10
HEAD_RIB_DEPTH = 0.0008

# Valve + lever
VALVE_Z = 0.14
VALVE_R = 0.013
VALVE_LEN = 0.055
VALVE_Y_CENTER = -0.0455
LEVER_JOINT_Y = -0.070
LEVER_PIN_LEN = 0.102

# Control pod + dial
POD_Z = 0.085
POD_R = 0.016
POD_LEN = 0.075
POD_X = 0.034
DIAL_JOINT_Y = -0.036
DIAL_D = 0.030
DIAL_H = 0.012


def _column_shape() -> cq.Workplane:
    """Tapered conical column, 0.06 m diameter at the deck."""
    return (
        cq.Workplane("XY")
        .circle(COLUMN_BASE_R)
        .workplane(offset=COLUMN_MID_Z)
        .circle(COLUMN_MID_R)
        .workplane(offset=COLUMN_TOP_Z - COLUMN_MID_Z)
        .circle(COLUMN_TOP_R)
        .loft()
    )


def _gooseneck_shape() -> cq.Workplane:
    """Slim tube: straight riser, high inverted-U arc, short drop leg."""
    path = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, RISER_TOP)
        .threePointArc((ARC_R, RISER_TOP + ARC_R), (REACH_X, RISER_TOP))
        .lineTo(REACH_X, DROP_END)
    )
    return cq.Workplane("XY").circle(TUBE_R).sweep(path)


def _spring_mesh() -> MeshGeometry:
    """Build a pre-rinse helical spring mesh around the gooseneck arc.

    The spring wraps around the semicircular arc portion of the gooseneck.
    Arc center is at (ARC_R, 0, RISER_TOP) in spout-local XZ coordinates.
    Arc angle theta goes from pi (start of arc) to 0 (end of arc).

    Built as a procedural tube mesh: a circle swept along the helical path.
    """
    n_total = SPRING_COILS * SPRING_SAMPLES_PER_COIL
    radial = 10  # segments around the wire cross-section

    # Generate helix centerline points
    path_pts: list[tuple[float, float, float]] = []
    for i in range(n_total + 1):
        t = i / n_total
        theta = math.pi * (1.0 - t)

        # Base point on the arc centerline
        bx = ARC_R + ARC_R * math.cos(theta)
        by = 0.0
        bz = RISER_TOP + ARC_R * math.sin(theta)

        # Helix phase
        phi = SPRING_COILS * 2.0 * math.pi * t

        # Outward normal and binormal
        nx = math.cos(theta)
        nz = math.sin(theta)

        ox = SPRING_COIL_R * math.cos(phi) * nx
        oy = SPRING_COIL_R * math.sin(phi)
        oz = SPRING_COIL_R * math.cos(phi) * nz

        path_pts.append((bx + ox, by + oy, bz + oz))

    # Build tube mesh by placing circles perpendicular to the path
    geom = MeshGeometry()
    n_pts = len(path_pts)

    # Compute frames along the path
    rings: list[list[int]] = []
    for i in range(n_pts):
        # Tangent direction (finite difference)
        if i == 0:
            tx = path_pts[1][0] - path_pts[0][0]
            ty = path_pts[1][1] - path_pts[0][1]
            tz = path_pts[1][2] - path_pts[0][2]
        elif i == n_pts - 1:
            tx = path_pts[-1][0] - path_pts[-2][0]
            ty = path_pts[-1][1] - path_pts[-2][1]
            tz = path_pts[-1][2] - path_pts[-2][2]
        else:
            tx = path_pts[i + 1][0] - path_pts[i - 1][0]
            ty = path_pts[i + 1][1] - path_pts[i - 1][1]
            tz = path_pts[i + 1][2] - path_pts[i - 1][2]

        # Normalize tangent
        tl = math.sqrt(tx * tx + ty * ty + tz * tz)
        if tl < 1e-12:
            tl = 1e-12
        tx, ty, tz = tx / tl, ty / tl, tz / tl

        # Build a perpendicular frame (parallel transport approximation)
        # Start with a reference up vector
        if abs(ty) < 0.9:
            ux, uy, uz = 0.0, 1.0, 0.0
        else:
            ux, uy, uz = 1.0, 0.0, 0.0

        # Normal = cross(tangent, up), normalized
        nx2 = ty * uz - tz * uy
        ny2 = tz * ux - tx * uz
        nz2 = tx * uy - ty * ux
        nl = math.sqrt(nx2 * nx2 + ny2 * ny2 + nz2 * nz2)
        if nl < 1e-12:
            nl = 1e-12
        nx2, ny2, nz2 = nx2 / nl, ny2 / nl, nz2 / nl

        # Binormal = cross(normal, tangent)
        bx2 = ny2 * tz - nz2 * ty
        by2 = nz2 * tx - nx2 * tz
        bz2 = nx2 * ty - ny2 * tx

        # Place radial vertices around the path point
        cx, cy, cz = path_pts[i]
        ring: list[int] = []
        for j in range(radial):
            angle = 2.0 * math.pi * j / radial
            ca = math.cos(angle)
            sa = math.sin(angle)
            # Offset from center in the normal-binormal plane
            dx = SPRING_WIRE_R * (ca * nx2 + sa * bx2)
            dy = SPRING_WIRE_R * (ca * ny2 + sa * by2)
            dz = SPRING_WIRE_R * (ca * nz2 + sa * bz2)
            idx = geom.add_vertex(cx + dx, cy + dy, cz + dz)
            ring.append(idx)
        rings.append(ring)

    # Connect rings with triangles
    for i in range(n_pts - 1):
        for j in range(radial):
            j_next = (j + 1) % radial
            a = rings[i][j]
            b = rings[i][j_next]
            c = rings[i + 1][j]
            d = rings[i + 1][j_next]
            geom.add_face(a, c, b)
            geom.add_face(b, c, d)

    # Cap ends
    center_start = geom.add_vertex(*path_pts[0])
    center_end = geom.add_vertex(*path_pts[-1])
    for j in range(radial):
        j_next = (j + 1) % radial
        geom.add_face(center_start, rings[0][j_next], rings[0][j])
        geom.add_face(center_end, rings[-1][j], rings[-1][j_next])

    return geom


def _ribbed_head_profile() -> list[tuple[float, float]]:
    """Generate a lathe profile for the spray head with shallow ribbing.

    The profile tapers from ~0.0125 at the top to ~0.0105 at the nozzle end,
    with periodic radial oscillations for shallow ribs.
    """
    n_segments = 120
    profile: list[tuple[float, float]] = []

    for i in range(n_segments + 1):
        t = i / n_segments  # 0 to 1
        z = -t * HEAD_LEN  # head extends downward

        # Base profile: smooth taper matching original head loft
        if t < 0.10:
            # Top transition (connects to hose)
            r = 0.0125 + (0.0140 - 0.0125) * (t / 0.10)
        elif t < 0.35:
            # Upper body widens
            r = 0.0140 + (0.0160 - 0.0140) * ((t - 0.10) / 0.25)
        elif t < 0.65:
            # Mid body (widest)
            r = 0.0160 + (0.0165 - 0.0160) * ((t - 0.35) / 0.30)
        elif t < 0.85:
            # Lower taper
            r = 0.0165 + (0.0140 - 0.0165) * ((t - 0.65) / 0.20)
        else:
            # Nozzle taper
            r = 0.0140 + (0.0105 - 0.0140) * ((t - 0.85) / 0.15)

        # Superimpose shallow ribbing (sinusoidal grooves)
        rib_phase = t * HEAD_RIB_COUNT * 2.0 * math.pi
        r += HEAD_RIB_DEPTH * math.sin(rib_phase)

        profile.append((max(r, 0.001), z))

    return profile


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_arc_gooseneck_prerinse_faucet")

    gold = model.material("brushed_gold", rgba=(0.78, 0.62, 0.28, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.75, 0.76, 0.78, 1.0))
    black = model.material("onyx_black", rgba=(0.05, 0.05, 0.05, 1.0))
    hose_gray = model.material("hose_gray", rgba=(0.20, 0.20, 0.20, 1.0))
    red = model.material("indicator_red", rgba=(0.85, 0.13, 0.10, 1.0))
    blue = model.material("indicator_blue", rgba=(0.20, 0.45, 0.90, 1.0))

    # ------------------------------------------------------------------ column
    column = model.part("body_column")
    column.visual(
        mesh_from_cadquery(_column_shape(), "tapered_column"),
        material=gold,
        name="tapered_column",
    )
    column.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_TOP_Z + COLLAR_LEN / 2.0)),
        material=gold,
        name="swivel_collar",
    )
    # Horizontal valve body on the right (-Y) side, mid-column height.
    column.visual(
        Cylinder(radius=VALVE_R, length=VALVE_LEN),
        origin=Origin(xyz=(0.0, VALVE_Y_CENTER, VALVE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="valve_body",
    )
    # Horizontal control pod on the front face of the column.
    column.visual(
        Cylinder(radius=POD_R, length=POD_LEN),
        origin=Origin(xyz=(POD_X, 0.0, POD_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="control_pod",
    )
    # Black touch display, slightly proud of the pod front surface.
    column.visual(
        Box((0.005, 0.048, 0.020)),
        origin=Origin(xyz=(POD_X + 0.0155, 0.004, POD_Z)),
        material=black,
        name="touch_display",
    )
    # Red (hot) and blue (cold) temperature icons proud of the display glass.
    column.visual(
        Cylinder(radius=0.0028, length=0.003),
        origin=Origin(xyz=(POD_X + 0.0192, -0.008, POD_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=red,
        name="hot_icon",
    )
    column.visual(
        Cylinder(radius=0.0028, length=0.003),
        origin=Origin(xyz=(POD_X + 0.0192, 0.014, POD_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blue,
        name="cold_icon",
    )

    # --------------------------------------------------------------- gooseneck
    spout = model.part("gooseneck_spout")
    spout.visual(
        mesh_from_cadquery(_gooseneck_shape(), "gooseneck_tube"),
        material=gold,
        name="gooseneck_tube",
    )
    # Pre-rinse helical spring wrapping the arc portion of the gooseneck.
    spout.visual(
        mesh_from_geometry(_spring_mesh(), "prerinse_spring"),
        material=chrome,
        name="prerinse_spring",
    )
    model.articulation(
        "spout_swivel",
        ArticulationType.REVOLUTE,
        parent=column,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=1.5, lower=-math.pi / 3.0, upper=math.pi / 3.0
        ),
    )

    # ------------------------------------------------- pull-down stage 1: hose
    hose = model.part("hose_stem")
    hose.visual(
        Cylinder(radius=SLEEVE_R, length=SLEEVE_LEN),
        origin=Origin(xyz=(0.0, 0.0, SLEEVE_LEN / 2.0)),
        material=hose_gray,
        name="hose_sleeve",
    )

    # ------------------------------------------------- pull-down stage 2: head
    head = model.part("spray_head")
    # Ribbed spray head body (lathe profile with shallow ribbing).
    head_profile = _ribbed_head_profile()
    head.visual(
        mesh_from_geometry(
            LatheGeometry(head_profile, segments=32),
            "ribbed_head_body",
        ),
        material=gold,
        name="ribbed_head_body",
    )
    # Nozzle ring at the bottom of the head, embedded into the head body bottom.
    head.visual(
        Cylinder(radius=NOZZLE_R, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -(HEAD_LEN - 0.004))),
        material=black,
        name="nozzle_ring",
    )
    # Spray mode button on the side.
    head.visual(
        Cylinder(radius=0.004, length=0.005),
        origin=Origin(xyz=(0.0175, 0.0, -0.045), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="spray_button",
    )
    # Hidden inner hose: extends deep into the head body to bridge to the nozzle ring,
    # and upward to keep engagement with the sleeve at full pull.
    head.visual(
        Cylinder(radius=INNER_HOSE_R, length=0.164),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=hose_gray,
        name="inner_hose",
    )

    model.articulation(
        "spray_pulldown",
        ArticulationType.PRISMATIC,
        parent=hose,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=0.3, lower=0.0, upper=STAGE_TRAVEL),
    )
    model.articulation(
        "hose_slide",
        ArticulationType.PRISMATIC,
        parent=spout,
        child=hose,
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=0.3, lower=0.0, upper=STAGE_TRAVEL),
        mimic=Mimic("spray_pulldown", multiplier=1.0),
    )

    # ------------------------------------------------------------------- lever
    lever = model.part("pin_lever")
    lever.visual(
        Cylinder(radius=0.0135, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="lever_collar",
    )
    lever.visual(
        Cylinder(radius=0.004, length=LEVER_PIN_LEN),
        origin=Origin(xyz=(0.0, 0.0, 0.010 + LEVER_PIN_LEN / 2.0)),
        material=gold,
        name="lever_pin",
    )
    lever.visual(
        Cylinder(radius=0.005, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.010 + LEVER_PIN_LEN + 0.0025)),
        material=gold,
        name="lever_tip",
    )
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=column,
        child=lever,
        origin=Origin(xyz=(0.0, LEVER_JOINT_Y, VALVE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=-math.pi / 4.0, upper=math.pi / 4.0
        ),
    )

    # -------------------------------------------------------------------- dial
    dial = model.part("dial_cap")
    dial_geo = KnobGeometry(
        DIAL_D,
        DIAL_H,
        body_style="cylindrical",
        grip=KnobGrip(style="fluted", count=24, depth=0.0008),
        center=False,
    )
    dial.visual(
        mesh_from_geometry(dial_geo, "dial_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="dial_body",
    )
    # Off-axis red dot on the dial face.
    dial.visual(
        Cylinder(radius=0.002, length=0.002),
        origin=Origin(xyz=(0.008, -(DIAL_H + 0.0005), 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=red,
        name="dial_dot",
    )
    model.articulation(
        "dial_knob",
        ArticulationType.REVOLUTE,
        parent=column,
        child=dial,
        origin=Origin(xyz=(POD_X, DIAL_JOINT_Y, POD_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0, lower=-2.3562, upper=2.3562
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    column = object_model.get_part("body_column")
    spout = object_model.get_part("gooseneck_spout")
    hose = object_model.get_part("hose_stem")
    head = object_model.get_part("spray_head")
    lever = object_model.get_part("pin_lever")
    dial = object_model.get_part("dial_cap")

    swivel = object_model.get_articulation("spout_swivel")
    pulldown = object_model.get_articulation("spray_pulldown")
    hose_slide = object_model.get_articulation("hose_slide")
    lever_pivot = object_model.get_articulation("lever_pivot")
    dial_knob = object_model.get_articulation("dial_knob")

    # Intentional nested telescoping fits and captured rotating collars.
    ctx.allow_overlap(
        hose,
        spout,
        elem_a="hose_sleeve",
        elem_b="gooseneck_tube",
        reason="Pull-down hose sleeve intentionally nests inside the solid spout tube proxy.",
    )
    ctx.allow_overlap(
        head,
        hose,
        elem_a="inner_hose",
        elem_b="hose_sleeve",
        reason="Inner hose intentionally telescopes inside the hose sleeve proxy.",
    )
    ctx.allow_overlap(
        head,
        spout,
        elem_a="inner_hose",
        elem_b="gooseneck_tube",
        reason="Hidden inner hose sits inside the solid spout tube at the rest pose.",
    )
    ctx.allow_overlap(
        lever,
        column,
        elem_a="lever_collar",
        elem_b="valve_body",
        reason="Rotating lever collar is captured on the valve body end (seated insertion).",
    )
    ctx.allow_overlap(
        dial,
        column,
        elem_a="dial_body",
        elem_b="control_pod",
        reason="Dial cap base embeds 1.5 mm into the pod end so it reads seated.",
    )

    # ----- scale, grounding, proportions
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "column grounded at deck plane",
        col_aabb is not None and abs(col_aabb[0][2]) <= 0.002,
        details=f"column aabb={col_aabb}",
    )
    base_aabb = ctx.part_element_world_aabb(column, elem="tapered_column")
    ctx.check(
        "column base diameter ~0.06 m",
        base_aabb is not None and 0.056 <= (base_aabb[1][0] - base_aabb[0][0]) <= 0.064,
        details=f"column element aabb={base_aabb}",
    )
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "gooseneck apex near 0.45 m",
        spout_aabb is not None and 0.43 <= spout_aabb[1][2] <= 0.48,
        details=f"spout aabb={spout_aabb}",
    )
    head_aabb = ctx.part_world_aabb(head)
    ctx.check(
        "spray head tip hangs around z=0.20 pointing down",
        head_aabb is not None and 0.16 <= head_aabb[0][2] <= 0.23,
        details=f"head aabb={head_aabb}",
    )

    # ----- variant: pre-rinse spring around the arc
    spring_aabb = ctx.part_element_world_aabb(spout, elem="prerinse_spring")
    ctx.check(
        "pre-rinse spring wraps the gooseneck arc",
        spring_aabb is not None
        and spring_aabb[1][2] > SWIVEL_Z + RISER_TOP + 0.02  # extends above arc start
        and spring_aabb[1][0] > 0.05  # reaches outward along arc
        and (spring_aabb[1][1] - spring_aabb[0][1]) > 0.020  # has Y-spread from coiling
        ,
        details=f"spring aabb={spring_aabb}",
    )

    # ----- variant: ribbed spray head
    ribbed_aabb = ctx.part_element_world_aabb(head, elem="ribbed_head_body")
    ctx.check(
        "ribbed spray head body exists with correct length",
        ribbed_aabb is not None
        and 0.085 <= (ribbed_aabb[1][2] - ribbed_aabb[0][2]) <= 0.110,
        details=f"ribbed head aabb={ribbed_aabb}",
    )
    ctx.check(
        "ribbed head diameter wider than smooth due to ribs",
        ribbed_aabb is not None
        and (ribbed_aabb[1][0] - ribbed_aabb[0][0]) > 0.025,
        details=f"ribbed head x-width={ribbed_aabb[1][0] - ribbed_aabb[0][0] if ribbed_aabb else None}",
    )

    # ----- joint plan: types and ranges
    ctx.check(
        "spout swivel is revolute +/-60 deg about vertical axis",
        swivel is not None
        and swivel.articulation_type == ArticulationType.REVOLUTE
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + math.pi / 3.0) < 1e-6
        and abs(swivel.motion_limits.upper - math.pi / 3.0) < 1e-6
        and tuple(swivel.axis) == (0.0, 0.0, 1.0),
    )
    ctx.check(
        "lever pivot is revolute +/-45 deg about valve horizontal axis",
        lever_pivot.articulation_type == ArticulationType.REVOLUTE
        and lever_pivot.motion_limits is not None
        and abs(lever_pivot.motion_limits.lower + math.pi / 4.0) < 1e-6
        and abs(lever_pivot.motion_limits.upper - math.pi / 4.0) < 1e-6,
    )
    ctx.check(
        "dial knob is revolute +/-135 deg",
        dial_knob.articulation_type == ArticulationType.REVOLUTE
        and dial_knob.motion_limits is not None
        and abs(dial_knob.motion_limits.lower + 2.3562) < 1e-4
        and abs(dial_knob.motion_limits.upper - 2.3562) < 1e-4,
    )
    total_travel = (pulldown.motion_limits.upper or 0.0) + (hose_slide.motion_limits.upper or 0.0)
    ctx.check(
        "pull-down spray head total prismatic travel is 0.12 m downward",
        pulldown.articulation_type == ArticulationType.PRISMATIC
        and hose_slide.articulation_type == ArticulationType.PRISMATIC
        and abs(total_travel - 0.12) < 1e-9
        and hose_slide.mimic is not None
        and hose_slide.mimic.joint == "spray_pulldown",
        details=f"total_travel={total_travel}, mimic={hose_slide.mimic}",
    )

    # ----- seating and retained insertion at rest
    ctx.expect_contact(
        head,
        spout,
        elem_a="ribbed_head_body",
        elem_b="gooseneck_tube",
        contact_tol=0.005,
        name="spray head seats near the spout tip",
    )
    ctx.expect_overlap(
        hose,
        spout,
        axes="z",
        elem_a="hose_sleeve",
        elem_b="gooseneck_tube",
        min_overlap=0.05,
        name="hose sleeve hidden inside the spout tube at rest",
    )
    ctx.expect_overlap(
        head,
        hose,
        axes="z",
        elem_a="inner_hose",
        elem_b="hose_sleeve",
        min_overlap=0.05,
        name="inner hose hidden inside the sleeve at rest",
    )
    ctx.expect_within(
        head,
        hose,
        axes="xy",
        inner_elem="inner_hose",
        outer_elem="hose_sleeve",
        margin=0.001,
        name="inner hose stays centered in the sleeve",
    )
    ctx.expect_overlap(
        lever,
        column,
        axes="y",
        elem_a="lever_collar",
        elem_b="valve_body",
        min_overlap=0.002,
        name="lever collar captured on the valve body",
    )
    ctx.expect_overlap(
        dial,
        column,
        axes="y",
        elem_a="dial_body",
        elem_b="control_pod",
        min_overlap=0.0005,
        name="dial cap seated on the pod end",
    )

    # ----- pull-down pose: head drops 0.12 m
    rest_head = ctx.part_world_position(head)
    with ctx.pose({pulldown: STAGE_TRAVEL}):
        ext_head = ctx.part_world_position(head)
        ctx.expect_overlap(
            hose,
            spout,
            axes="z",
            elem_a="hose_sleeve",
            elem_b="gooseneck_tube",
            min_overlap=0.008,
            name="sleeve retains insertion in spout at full pull-down",
        )
        ctx.expect_overlap(
            head,
            hose,
            axes="z",
            elem_a="inner_hose",
            elem_b="hose_sleeve",
            min_overlap=0.008,
            name="inner hose retains insertion in sleeve at full pull-down",
        )
    ctx.check(
        "pull-down lowers the spray head by 0.12 m along the spout-tip axis",
        rest_head is not None
        and ext_head is not None
        and abs((rest_head[2] - ext_head[2]) - 0.12) < 1e-6
        and abs(rest_head[0] - ext_head[0]) < 1e-9,
        details=f"rest={rest_head}, extended={ext_head}",
    )

    # ----- swivel pose: spray head carried sideways about the column axis
    with ctx.pose({swivel: 1.0}):
        sw_head = ctx.part_world_position(head)
    ctx.check(
        "spout swivel carries the spray head sideways about the column axis",
        sw_head is not None and sw_head[1] > 0.10 and rest_head is not None
        and abs(rest_head[1]) < 1e-9,
        details=f"rest={rest_head}, swiveled={sw_head}",
    )

    # ----- lever pose: pin sweeps fore/aft about the valve axis
    rest_lever = ctx.part_world_aabb(lever)
    ctx.check(
        "pin lever is ~0.10 m long above the valve body",
        rest_lever is not None and 0.095 <= (rest_lever[1][2] - VALVE_Z) <= 0.130,
        details=f"lever aabb={rest_lever}",
    )
    with ctx.pose({lever_pivot: math.pi / 4.0}):
        tilted_lever = ctx.part_world_aabb(lever)
    ctx.check(
        "lever pin sweeps in X when rotated about the valve axis",
        rest_lever is not None
        and tilted_lever is not None
        and tilted_lever[1][0] > rest_lever[1][0] + 0.05,
        details=f"rest={rest_lever}, tilted={tilted_lever}",
    )

    # ----- dial pose: off-axis dot orbits the pod axis
    rest_dot = ctx.part_element_world_aabb(dial, elem="dial_dot")
    with ctx.pose({dial_knob: math.pi / 2.0}):
        turned_dot = ctx.part_element_world_aabb(dial, elem="dial_dot")
    ctx.check(
        "off-axis dial dot orbits the dial axis",
        rest_dot is not None
        and turned_dot is not None
        and abs(0.5 * (turned_dot[0][2] + turned_dot[1][2]) - 0.5 * (rest_dot[0][2] + rest_dot[1][2]))
        > 0.005,
        details=f"rest={rest_dot}, turned={turned_dot}",
    )

    # ----- control pod display details
    disp = ctx.part_element_world_aabb(column, elem="touch_display")
    hot = ctx.part_element_world_aabb(column, elem="hot_icon")
    cold = ctx.part_element_world_aabb(column, elem="cold_icon")
    pod = ctx.part_element_world_aabb(column, elem="control_pod")
    ctx.check(
        "touch display sits proud of the pod front surface",
        disp is not None and pod is not None and disp[1][0] > pod[1][0],
        details=f"display={disp}, pod={pod}",
    )
    ctx.check(
        "red and blue temperature icons sit proud of the display",
        disp is not None
        and hot is not None
        and cold is not None
        and hot[1][0] > disp[1][0]
        and cold[1][0] > disp[1][0],
        details=f"display={disp}, hot={hot}, cold={cold}",
    )

    return ctx.report()


object_model = build_object_model()
