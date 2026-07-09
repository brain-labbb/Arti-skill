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
    KnobIndicator,
    KnobTopFeature,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# High-arc gooseneck kitchen faucet variant 08, ~0.45 m tall.
#
# Changes from parent:
# - Slim commercial pre-rinse style spring around the gooseneck arc
# - Small top flow knob that rotates independently at the arc apex
# - Shallow ribbing on the spray head
# - Thin seam at the swivel collar
#
# Layout (world frame, deck plane at z = 0):
# - +X is front (direction the gooseneck reaches), +Z is up.
# - Tapered column rises to z = 0.307.
# - Gooseneck spout swivels about vertical column axis (+/-60 deg).
# - Pull-down spray head hangs at spout tip, 0.12 m travel.
# - Horizontal valve body on the right (-Y) side carries pin lever.
# - Control pod with touch display and dial cap on front face.
# - Pre-rinse spring coils around the gooseneck arc.
# - Flow knob at arc apex rotates independently.
# ---------------------------------------------------------------------------

# Column
COLUMN_BASE_R = 0.030
COLUMN_MID_R = 0.020
COLUMN_TOP_R = 0.0155
COLUMN_MID_Z = 0.18
COLUMN_TOP_Z = 0.295
COLLAR_R = 0.0175
COLLAR_LEN = 0.012
SWIVEL_Z = 0.307  # top of the swivel collar = base of the gooseneck riser

# Gooseneck (spout-local coordinates, frame at the top of the collar)
TUBE_R = 0.012
RISER_TOP = 0.053  # straight riser before the arc
ARC_R = 0.085
REACH_X = 2.0 * ARC_R  # 0.17 m horizontal reach
DROP_END = 0.003  # spout-local z of the open tube tip

# Pre-rinse spring
SPRING_COIL_R = TUBE_R + 0.004  # spring sits just outside the tube
SPRING_WIRE_R = 0.0012
SPRING_N_COILS = 10
SPRING_N_SAMPLES = 160

# Pull-down stages
STAGE_TRAVEL = 0.060  # per stage; total head travel = 0.12 m
SLEEVE_R = 0.0075
SLEEVE_LEN = 0.072
INNER_HOSE_R = 0.0048

# Spray head (head-local, frame at the tube tip seam, body extends -Z)
HEAD_LEN = 0.100
NOZZLE_R = 0.0095

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

# Flow knob (at arc apex in spout-local frame)
FLOW_KNOB_X = ARC_R  # x position at arc apex
FLOW_KNOB_Z = RISER_TOP + ARC_R + TUBE_R  # z position on top of tube at arc apex
FLOW_KNOB_D = 0.020
FLOW_KNOB_H = 0.012

# Seam ring
SEAM_RING_R = COLLAR_R + 0.001
SEAM_RING_THICK = 0.0015


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


def _spring_points() -> list[tuple[float, float, float]]:
    """Generate helical points wrapping around the gooseneck arc."""
    points: list[tuple[float, float, float]] = []
    for i in range(SPRING_N_SAMPLES + 1):
        t = i / SPRING_N_SAMPLES  # 0 to 1
        theta = math.pi * (1.0 - t)  # π to 0 along the arc
        phi = SPRING_N_COILS * 2.0 * math.pi * t  # coil angle

        # Arc center point
        cx = ARC_R + ARC_R * math.cos(theta)
        cz = RISER_TOP + ARC_R * math.sin(theta)

        # Normal (outward from arc center in XZ plane)
        nx = math.cos(theta)
        nz = math.sin(theta)

        # Spring point: offset from arc center by coil radius
        x = cx + SPRING_COIL_R * (math.cos(phi) * nx)
        y = SPRING_COIL_R * math.sin(phi)
        z = cz + SPRING_COIL_R * (math.cos(phi) * nz)

        points.append((x, y, z))
    return points


def _ribbed_head_shape() -> cq.Workplane:
    """Tapered pull-down spray head with shallow horizontal ribbing."""
    # Sections: (z_offset, radius) — z is negative (body extends -Z from origin)
    sections = [
        (0.000, 0.0125),   # top connector
        (-0.008, 0.0145),  # upper shoulder
        # Rib section 1
        (-0.014, 0.0168),  # rib crest
        (-0.018, 0.0158),  # rib valley
        # Rib section 2
        (-0.024, 0.0168),  # rib crest
        (-0.028, 0.0158),  # rib valley
        # Rib section 3
        (-0.034, 0.0168),  # rib crest
        (-0.038, 0.0158),  # rib valley
        # Rib section 4
        (-0.044, 0.0165),  # rib crest
        (-0.048, 0.0155),  # rib valley
        # Lower body taper
        (-0.058, 0.0158),
        (-0.070, 0.0150),
        (-0.082, 0.0138),
        (-0.090, 0.0120),
        (-0.100, 0.0105),  # tip
    ]

    wp = cq.Workplane("XY").circle(sections[0][1])
    prev_z = sections[0][0]
    for z_off, r in sections[1:]:
        wp = wp.workplane(offset=z_off - prev_z).circle(r)
        prev_z = z_off
    return wp.loft()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_arc_gooseneck_faucet_v08")

    gold = model.material("brushed_gold", rgba=(0.78, 0.62, 0.28, 1.0))
    black = model.material("onyx_black", rgba=(0.05, 0.05, 0.05, 1.0))
    hose_gray = model.material("hose_gray", rgba=(0.20, 0.20, 0.20, 1.0))
    chrome = model.material("spring_chrome", rgba=(0.72, 0.72, 0.74, 1.0))
    red = model.material("indicator_red", rgba=(0.85, 0.13, 0.10, 1.0))
    blue = model.material("indicator_blue", rgba=(0.20, 0.45, 0.90, 1.0))
    seam_dark = model.material("seam_dark", rgba=(0.12, 0.10, 0.06, 1.0))

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
    # Thin seam ring at the swivel collar base
    column.visual(
        Cylinder(radius=SEAM_RING_R, length=SEAM_RING_THICK),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_TOP_Z + SEAM_RING_THICK / 2.0)),
        material=seam_dark,
        name="collar_seam",
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
    # Pre-rinse spring coiled around the gooseneck arc
    spring_mesh = tube_from_spline_points(
        _spring_points(),
        radius=SPRING_WIRE_R,
        samples_per_segment=3,
        radial_segments=10,
        cap_ends=True,
        spline="catmull_rom",
    )
    spout.visual(
        mesh_from_geometry(spring_mesh, "prerinse_spring"),
        material=chrome,
        name="prerinse_spring",
    )
    # Spring anchor clips connecting the spring to the tube at start and end
    spout.visual(
        Cylinder(radius=0.002, length=0.008),
        origin=Origin(xyz=(-0.014, 0.0, RISER_TOP), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="spring_clip_start",
    )
    spout.visual(
        Cylinder(radius=0.002, length=0.008),
        origin=Origin(xyz=(REACH_X + 0.014, 0.0, RISER_TOP), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="spring_clip_end",
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
    head.visual(
        mesh_from_cadquery(_ribbed_head_shape(), "head_body"),
        material=gold,
        name="head_body",
    )
    head.visual(
        Cylinder(radius=NOZZLE_R, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -(HEAD_LEN + 0.003))),
        material=black,
        name="nozzle_ring",
    )
    head.visual(
        Cylinder(radius=0.004, length=0.005),
        origin=Origin(xyz=(0.0155, 0.0, -0.045), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="spray_button",
    )
    # Hidden inner hose: keeps the head engaged with the sleeve at full pull.
    head.visual(
        Cylinder(radius=INNER_HOSE_R, length=0.078),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
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
    # Off-axis red dot on the dial face
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

    # -------------------------------------------------------------- flow knob
    # Small top flow knob at the arc apex, rotates independently about Z.
    flow_knob = model.part("flow_knob")
    flow_knob_geo = KnobGeometry(
        FLOW_KNOB_D,
        FLOW_KNOB_H,
        body_style="domed",
        grip=KnobGrip(style="ribbed", count=16, depth=0.0006),
        indicator=KnobIndicator(style="line", mode="raised", depth=0.0005),
        top_feature=KnobTopFeature(style="recess", diameter=0.006, height=0.001),
        center=False,
    )
    flow_knob.visual(
        mesh_from_geometry(flow_knob_geo, "flow_knob_body"),
        material=gold,
        name="flow_knob_body",
    )
    # Small stem connecting knob to the spout arc
    flow_knob.visual(
        Cylinder(radius=0.004, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=gold,
        name="flow_knob_stem",
    )
    # Off-center indicator dot for rotation proof
    flow_knob.visual(
        Cylinder(radius=0.0015, length=0.002),
        origin=Origin(xyz=(0.007, 0.0, FLOW_KNOB_H + 0.0005)),
        material=black,
        name="flow_knob_dot",
    )
    model.articulation(
        "flow_knob_rotate",
        ArticulationType.REVOLUTE,
        parent=spout,
        child=flow_knob,
        origin=Origin(xyz=(FLOW_KNOB_X, 0.0, FLOW_KNOB_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0, lower=-math.pi, upper=math.pi
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
    flow_knob = object_model.get_part("flow_knob")

    swivel = object_model.get_articulation("spout_swivel")
    pulldown = object_model.get_articulation("spray_pulldown")
    hose_slide = object_model.get_articulation("hose_slide")
    lever_pivot = object_model.get_articulation("lever_pivot")
    dial_knob = object_model.get_articulation("dial_knob")
    flow_knob_rot = object_model.get_articulation("flow_knob_rotate")

    # Intentional nested telescoping fits and captured rotating collars
    ctx.allow_overlap(
        hose, spout,
        elem_a="hose_sleeve", elem_b="gooseneck_tube",
        reason="Pull-down hose sleeve intentionally nests inside the solid spout tube proxy.",
    )
    ctx.allow_overlap(
        head, hose,
        elem_a="inner_hose", elem_b="hose_sleeve",
        reason="Inner hose intentionally telescopes inside the hose sleeve proxy.",
    )
    ctx.allow_overlap(
        head, spout,
        elem_a="inner_hose", elem_b="gooseneck_tube",
        reason="Hidden inner hose sits inside the solid spout tube at the rest pose.",
    )
    ctx.allow_overlap(
        lever, column,
        elem_a="lever_collar", elem_b="valve_body",
        reason="Rotating lever collar is captured on the valve body end (seated insertion).",
    )
    ctx.allow_overlap(
        dial, column,
        elem_a="dial_body", elem_b="control_pod",
        reason="Dial cap base embeds 1.5 mm into the pod end so it reads seated.",
    )
    ctx.allow_overlap(
        flow_knob, spout,
        elem_a="flow_knob_stem", elem_b="gooseneck_tube",
        reason="Flow knob stem embeds into the spout tube at the arc apex for mounting.",
    )
    ctx.allow_overlap(
        flow_knob, spout,
        elem_a="flow_knob_body", elem_b="prerinse_spring",
        reason="Flow knob sits at the arc apex where spring coils pass around it; small local nesting overlap.",
    )
    ctx.allow_overlap(
        head, spout,
        elem_a="head_body", elem_b="gooseneck_tube",
        reason="Ribbed spray head seats against the tube tip with small local loft overshoot at the interface.",
    )

    # ----- variant 08: pre-rinse spring present on gooseneck arc
    spring_aabb = ctx.part_element_world_aabb(spout, elem="prerinse_spring")
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "pre-rinse spring present around the gooseneck arc",
        spring_aabb is not None
        and spout_aabb is not None
        and spring_aabb[1][2] > 0.40  # spring reaches near the arc apex
        and spring_aabb[0][2] < 0.40  # spring extends below the apex
        and (spring_aabb[1][0] - spring_aabb[0][0]) > 0.05,  # spans arc width
        details=f"spring_aabb={spring_aabb}",
    )

    # ----- variant 08: collar seam ring visible
    seam_aabb = ctx.part_element_world_aabb(column, elem="collar_seam")
    collar_aabb = ctx.part_element_world_aabb(column, elem="swivel_collar")
    ctx.check(
        "thin seam ring present at the swivel collar",
        seam_aabb is not None
        and collar_aabb is not None
        and abs(seam_aabb[0][2] - collar_aabb[0][2]) < 0.005,
        details=f"seam={seam_aabb}, collar={collar_aabb}",
    )

    # ----- variant 08: flow knob at arc apex with independent rotation
    flow_knob_aabb = ctx.part_world_aabb(flow_knob)
    ctx.check(
        "flow knob positioned at arc apex near top of faucet",
        flow_knob_aabb is not None
        and flow_knob_aabb[1][2] > 0.42  # near the top
        and flow_knob_aabb[0][0] > 0.04,  # forward on the arc
        details=f"flow_knob_aabb={flow_knob_aabb}",
    )
    ctx.check(
        "flow knob is revolute about vertical axis with full rotation range",
        flow_knob_rot.articulation_type == ArticulationType.REVOLUTE
        and flow_knob_rot.motion_limits is not None
        and abs(flow_knob_rot.motion_limits.lower + math.pi) < 1e-6
        and abs(flow_knob_rot.motion_limits.upper - math.pi) < 1e-6
        and tuple(flow_knob_rot.axis) == (0.0, 0.0, 1.0),
    )

    # Prove flow knob rotates independently (off-center dot orbits the axis)
    rest_knob_dot = ctx.part_element_world_aabb(flow_knob, elem="flow_knob_dot")
    with ctx.pose({flow_knob_rot: math.pi / 2.0}):
        turned_knob_dot = ctx.part_element_world_aabb(flow_knob, elem="flow_knob_dot")
    ctx.check(
        "flow knob off-center dot orbits the knob axis (independent rotation proof)",
        rest_knob_dot is not None
        and turned_knob_dot is not None
        and abs(0.5 * (turned_knob_dot[0][1] + turned_knob_dot[1][1])
              - 0.5 * (rest_knob_dot[0][1] + rest_knob_dot[1][1])) > 0.003,
        details=f"rest={rest_knob_dot}, turned={turned_knob_dot}",
    )

    # ----- variant 08: ribbed spray head (wider than smooth would be)
    head_elem_aabb = ctx.part_element_world_aabb(head, elem="head_body")
    ctx.check(
        "spray head has ribbed profile (wider than smooth taper)",
        head_elem_aabb is not None
        and (head_elem_aabb[1][0] - head_elem_aabb[0][0]) > 0.030,
        details=f"head_body_aabb={head_elem_aabb}",
    )

    # ----- proof checks for new variant 08 allowances
    ctx.expect_overlap(
        flow_knob, spout,
        axes="z", elem_a="flow_knob_body", elem_b="prerinse_spring",
        min_overlap=0.005,
        name="flow knob nested at arc apex within the spring coil",
    )
    ctx.expect_contact(
        head, spout,
        elem_a="head_body", elem_b="gooseneck_tube",
        contact_tol=0.008,
        name="ribbed spray head seats against the tube tip",
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
    ctx.check(
        "gooseneck apex near 0.45 m",
        spout_aabb is not None and 0.43 <= spout_aabb[1][2] <= 0.48,
        details=f"spout aabb={spout_aabb}",
    )
    head_aabb = ctx.part_world_aabb(head)
    ctx.check(
        "spray head tip hangs around z=0.20 pointing down",
        head_aabb is not None and 0.18 <= head_aabb[0][2] <= 0.23,
        details=f"head aabb={head_aabb}",
    )

    # ----- joint plan: types and ranges
    ctx.check(
        "spout swivel is revolute +/-60 deg about vertical axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
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
        head, spout,
        elem_a="head_body", elem_b="gooseneck_tube",
        contact_tol=0.002,
        name="spray head seats flush against the spout tip",
    )
    ctx.expect_overlap(
        hose, spout,
        axes="z", elem_a="hose_sleeve", elem_b="gooseneck_tube",
        min_overlap=0.05,
        name="hose sleeve hidden inside the spout tube at rest",
    )
    ctx.expect_overlap(
        head, hose,
        axes="z", elem_a="inner_hose", elem_b="hose_sleeve",
        min_overlap=0.05,
        name="inner hose hidden inside the sleeve at rest",
    )
    ctx.expect_within(
        head, hose,
        axes="xy", inner_elem="inner_hose", outer_elem="hose_sleeve",
        margin=0.001,
        name="inner hose stays centered in the sleeve",
    )
    ctx.expect_overlap(
        lever, column,
        axes="y", elem_a="lever_collar", elem_b="valve_body",
        min_overlap=0.002,
        name="lever collar captured on the valve body",
    )
    ctx.expect_overlap(
        dial, column,
        axes="y", elem_a="dial_body", elem_b="control_pod",
        min_overlap=0.0005,
        name="dial cap seated on the pod end",
    )

    # ----- pull-down pose
    rest_head = ctx.part_world_position(head)
    with ctx.pose({pulldown: STAGE_TRAVEL}):
        ext_head = ctx.part_world_position(head)
        ctx.expect_overlap(
            hose, spout,
            axes="z", elem_a="hose_sleeve", elem_b="gooseneck_tube",
            min_overlap=0.008,
            name="sleeve retains insertion in spout at full pull-down",
        )
        ctx.expect_overlap(
            head, hose,
            axes="z", elem_a="inner_hose", elem_b="hose_sleeve",
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

    # ----- swivel pose
    with ctx.pose({swivel: 1.0}):
        sw_head = ctx.part_world_position(head)
    ctx.check(
        "spout swivel carries the spray head sideways about the column axis",
        sw_head is not None and sw_head[1] > 0.10 and rest_head is not None
        and abs(rest_head[1]) < 1e-9,
        details=f"rest={rest_head}, swiveled={sw_head}",
    )

    # ----- lever pose
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

    # ----- dial pose
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
