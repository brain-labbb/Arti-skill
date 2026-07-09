from __future__ import annotations

"""Wall-mounted mini-split air conditioner indoor unit (clamshell variant).

Glossy white rounded horizontal body ~0.90 m wide x 0.30 m tall x 0.22 m deep,
back flat against the wall plane (y=0), bottom at z=0. The lower front is a
quarter-round curve carrying three long slim louver slots, each with its own
independently pivoting airflow vane (revolute about the unit's width axis).
The front service cover is split into two side-by-side clamshell leaves, each
hinged along its own top edge and opening upward independently to reveal a
shallow filter cavity with a removable-look filter panel. The interior behind
the louver slots is a hollow dark plenum.

Coordinate convention:
- X: unit width (left-right along the wall)
- Y: depth, +Y points away from the wall (front)
- Z: up, bottom of the unit at z = 0
"""

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ----------------------------------------------------------------------------
# Master dimensions (meters)
# ----------------------------------------------------------------------------
BODY_W = 0.90  # width along X (x in [-0.45, 0.45])
BODY_D = 0.22  # overall depth target along Y
BODY_H = 0.30  # height along Z (z in [0, 0.30])

# Side profile control points (y = depth from wall, z = height from bottom).
ARC_CY, ARC_CZ, ARC_R = 0.085, 0.13, 0.13  # bottom-front quarter-round
FRONT_LO = (0.215, 0.13)  # bottom of the leaning front face
FRONT_HI = (0.205, 0.285)  # top of the leaning front face
TOP_FRONT = (0.185, 0.30)  # front edge of the flat top (small bevel)

# Front service-panel zone (chassis is recessed behind the hinged panel).
PANEL_RECESS_Y = 0.198  # recessed chassis face behind the panel
PANEL_ZONE_Z0 = 0.142  # panel zone starts above the lower curve lip

# Hinged front panel.
PANEL_W = 0.85
PANEL_T = 0.013
PANEL_H = 0.1465
PANEL_HINGE_Y = 0.1995
PANEL_HINGE_Z = 0.290
PANEL_BORDER = 0.025
PANEL_OPEN_MAX = math.radians(60.0)

# Clamshell variant: two side-by-side leaves sharing the panel zone.
NUM_LEAVES = 2
LEAF_GAP = 0.005  # center gap between the two leaves
LEAF_W = (PANEL_W - LEAF_GAP) / float(NUM_LEAVES)
# X centers for leaf 0 (left) and leaf 1 (right).
LEAF_CX = tuple(
    (i - (NUM_LEAVES - 1) / 2.0) * (LEAF_W + LEAF_GAP) for i in range(NUM_LEAVES)
)

# Filter cavity + filter behind the front panel.
POCKET_W, POCKET_Y0, POCKET_Y1 = 0.74, 0.15, 0.21
POCKET_Z0, POCKET_Z1 = 0.185, 0.272

# Louver slots / vanes on the lower front curve, parameterized by the arc
# angle theta measured from straight-down (0 deg = bottom, 90 deg = front).
VANE_SPECS = (
    ("lower", math.radians(35.0)),
    ("middle", math.radians(55.0)),
    ("upper", math.radians(75.0)),
)
SLOT_LEN = 0.78  # slot cut length along X
SLOT_OPEN = 0.024  # slot opening measured along the surface tangent
SLOT_DEPTH = 0.12  # cut depth along the surface normal (pierces into plenum)
VANE_LEN = 0.74
VANE_CHORD = 0.018
VANE_T = 0.0045
VANE_SWING = math.radians(45.0)

# Hollow dark plenum behind the louver slots (cross-flow blower bay).
PLENUM_R = 0.08
PLENUM_HALF_LEN = 0.41
LINER_R = 0.075
LINER_HALF_LEN = 0.42  # ends embed into the chassis side walls


def _arc_point(theta: float) -> tuple[float, float]:
    """(y, z) point on the bottom-front quarter-round at arc angle theta."""
    return (ARC_CY + ARC_R * math.sin(theta), ARC_CZ - ARC_R * math.cos(theta))


def _housing_shape() -> cq.Workplane:
    mid = _arc_point(math.radians(45.0))
    body = (
        cq.Workplane("YZ")
        .moveTo(0.0, 0.0)
        .lineTo(ARC_CY, 0.0)
        .threePointArc(mid, FRONT_LO)
        .lineTo(*FRONT_HI)
        .lineTo(*TOP_FRONT)
        .lineTo(0.0, BODY_H)
        .close()
        .extrude(BODY_W / 2.0, both=True)
    )

    # Recess the front wall behind the hinged service panel.
    body = body.cut(
        cq.Workplane("XY")
        .box(BODY_W + 0.02, 0.102, 0.208)
        .translate((0.0, PANEL_RECESS_Y + 0.051, PANEL_ZONE_Z0 + 0.104))
    )
    # Shallow framed inset on the top face (air-intake panel look).
    body = body.cut(
        cq.Workplane("XY").box(0.80, 0.141, 0.013).translate((0.0, 0.0925, 0.3035))
    )
    # Filter cavity pocket behind the front panel.
    body = body.cut(
        cq.Workplane("XY")
        .box(POCKET_W, POCKET_Y1 - POCKET_Y0, POCKET_Z1 - POCKET_Z0)
        .translate(
            (0.0, (POCKET_Y0 + POCKET_Y1) / 2.0, (POCKET_Z0 + POCKET_Z1) / 2.0)
        )
    )
    # Hollow blower plenum behind the louver slots, capped by a back wall.
    plenum = (
        cq.Workplane("YZ")
        .center(ARC_CY, ARC_CZ)
        .circle(PLENUM_R)
        .extrude(PLENUM_HALF_LEN, both=True)
        .intersect(
            cq.Workplane("XY").box(0.82, 0.19, 0.30).translate((0.0, 0.11, 0.15))
        )
    )
    body = body.cut(plenum)
    # Three long louver slots through the curved lower-front wall.
    for _, theta in VANE_SPECS:
        ay, az = _arc_point(theta)
        slot = (
            cq.Workplane("XY")
            .box(SLOT_LEN, SLOT_DEPTH, SLOT_OPEN)
            .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), math.degrees(theta) - 90.0)
            .translate((0.0, ay, az))
        )
        body = body.cut(slot)
    return body


def _front_panel_shape(leaf_w: float) -> cq.Workplane:
    """One clamshell leaf plate in its joint-local frame.

    Local frame sits on the hinge line (top edge): the plate hangs along -Z
    and its thickness extends along +Y (away from the chassis).
    ``leaf_w`` is the width of this single leaf.
    """
    plate = (
        cq.Workplane("XY")
        .box(leaf_w, PANEL_T, PANEL_H)
        .translate((0.0, PANEL_T / 2.0, -PANEL_H / 2.0))
    )
    # Shallow face recess leaving a thin raised border frame.
    recess = (
        cq.Workplane("XY")
        .box(leaf_w - 2.0 * PANEL_BORDER, 0.006, PANEL_H - 2.0 * PANEL_BORDER)
        .translate((0.0, PANEL_T, -PANEL_H / 2.0))
    )
    return plate.cut(recess)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mini_split_indoor_unit")

    model.material("shell_white", rgba=(0.93, 0.94, 0.95, 1.0))
    model.material("panel_white", rgba=(0.96, 0.965, 0.97, 1.0))
    model.material("vane_white", rgba=(0.88, 0.895, 0.91, 1.0))
    model.material("cavity_dark", rgba=(0.07, 0.07, 0.08, 1.0))
    model.material("filter_gray", rgba=(0.80, 0.81, 0.82, 1.0))
    model.material("mesh_gray", rgba=(0.30, 0.32, 0.34, 1.0))

    # ------------------------------------------------------------------ body
    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shape(), "housing_shell"),
        material="shell_white",
        name="housing_shell",
    )
    # Dark blower-bay liner: visible through the louver slots, ends embedded
    # in the chassis side walls so it reads as the hollow dark interior.
    housing.visual(
        Cylinder(radius=LINER_R, length=2.0 * LINER_HALF_LEN),
        origin=Origin(xyz=(0.0, ARC_CY, ARC_CZ), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="cavity_dark",
        name="plenum_liner",
    )
    # Removable-look filter panel seated against the back of the filter
    # cavity (frame slightly embedded into the cavity back wall).
    housing.visual(
        Box((0.70, 0.005, 0.082)),
        origin=Origin(xyz=(0.0, 0.152, 0.2285)),
        material="filter_gray",
        name="filter_frame",
    )
    housing.visual(
        Box((0.66, 0.004, 0.072)),
        origin=Origin(xyz=(0.0, 0.1555, 0.2285)),
        material="mesh_gray",
        name="filter_mesh",
    )

    # ------------------------------------------- clamshell service panels
    # Two side-by-side leaves, each hinged along its own top edge and
    # opening upward like a clamshell (uniform hinge policy via loop).
    for i in range(NUM_LEAVES):
        leaf = model.part(f"panel_{i}")
        leaf.visual(
            mesh_from_cadquery(_front_panel_shape(LEAF_W), f"panel_{i}_plate"),
            material="panel_white",
            name=f"panel_{i}_plate",
        )
        # Hinge knuckles along the top back edge of this leaf: seated into
        # the recessed chassis wall to carry the cover (intentional embed).
        for kdx, kx_off in enumerate((-LEAF_W * 0.35, LEAF_W * 0.35)):
            leaf.visual(
                Box((0.04, 0.006, 0.012)),
                origin=Origin(xyz=(kx_off, -0.00125, -0.004)),
                material="panel_white",
                name=f"panel_{i}_hinge_knuckle_{kdx}",
            )
        model.articulation(
            f"panel_{i}_hinge",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=leaf,
            origin=Origin(xyz=(LEAF_CX[i], PANEL_HINGE_Y, PANEL_HINGE_Z)),
            # Hinge along each leaf's top edge; the closed plate hangs
            # along local -Z, so positive rotation about +X swings the
            # free bottom edge outward (+Y) and up: clamshell opens.
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0, velocity=1.5, lower=0.0, upper=PANEL_OPEN_MAX
            ),
        )

    # --------------------------------------------------------- louver vanes
    for label, theta in VANE_SPECS:
        vane = model.part(f"{label}_louver_vane")
        # Blade lies in the slot's tangent plane at q=0: local +Y is the
        # outward surface normal, local +Z the surface tangent (chord).
        vane.visual(
            Box((VANE_LEN, VANE_T, VANE_CHORD)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material="vane_white",
            name=f"{label}_vane_blade",
        )
        # End pivot pins on the blade axis, seated into the slot end walls.
        for idx, px in enumerate((-0.385, 0.385)):
            vane.visual(
                Cylinder(radius=0.003, length=0.04),
                origin=Origin(xyz=(px, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
                material="vane_white",
                name=f"{label}_pivot_pin_{idx}",
            )
        ay, az = _arc_point(theta)
        model.articulation(
            f"{label}_louver_pivot",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=vane,
            # Joint frame rotated about X so local +Y matches the outward
            # normal of the curved front at this slot.
            origin=Origin(
                xyz=(0.0, ay, az), rpy=(theta - math.pi / 2.0, 0.0, 0.0)
            ),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0, velocity=2.0, lower=-VANE_SWING, upper=VANE_SWING
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    leaves = [object_model.get_part(f"panel_{i}") for i in range(NUM_LEAVES)]
    hinges = [object_model.get_articulation(f"panel_{i}_hinge") for i in range(NUM_LEAVES)]
    vanes = [object_model.get_part(f"{label}_louver_vane") for label, _ in VANE_SPECS]
    pivots = [
        object_model.get_articulation(f"{label}_louver_pivot") for label, _ in VANE_SPECS
    ]

    # ---- real-world scale and grounding -----------------------------------
    hb = ctx.part_world_aabb(housing)
    assert hb is not None
    ctx.check(
        "housing width ~0.90 m",
        abs((hb[1][0] - hb[0][0]) - BODY_W) < 0.02,
        details=f"width={hb[1][0] - hb[0][0]:.4f}",
    )
    ctx.check(
        "housing height ~0.30 m",
        abs((hb[1][2] - hb[0][2]) - BODY_H) < 0.02,
        details=f"height={hb[1][2] - hb[0][2]:.4f}",
    )
    ctx.check(
        "housing depth ~0.22 m",
        0.20 <= (hb[1][1] - hb[0][1]) <= 0.23,
        details=f"depth={hb[1][1] - hb[0][1]:.4f}",
    )
    ctx.check(
        "unit bottom sits at z=0 with flat back at the wall plane y=0",
        abs(hb[0][2]) < 0.005 and abs(hb[0][1]) < 0.005,
        details=f"min={hb[0]}",
    )

    # ---- intentional hinge/pivot embeds ------------------------------------
    for i in range(NUM_LEAVES):
        leaf = leaves[i]
        for kdx in range(2):
            ctx.allow_overlap(
                leaf,
                housing,
                elem_a=f"panel_{i}_hinge_knuckle_{kdx}",
                elem_b="housing_shell",
                reason="Hinge knuckle is intentionally seated into the recessed chassis wall to carry the clamshell leaf.",
            )
            ctx.expect_contact(
                leaf,
                housing,
                elem_a=f"panel_{i}_hinge_knuckle_{kdx}",
                elem_b="housing_shell",
                name=f"panel_{i} hinge knuckle {kdx} is seated on the chassis",
            )
    for (label, _), vane in zip(VANE_SPECS, vanes):
        for idx in range(2):
            ctx.allow_overlap(
                vane,
                housing,
                elem_a=f"{label}_pivot_pin_{idx}",
                elem_b="housing_shell",
                reason="Vane end pivot pin is intentionally captured in the louver slot end wall.",
            )
            ctx.expect_contact(
                vane,
                housing,
                elem_a=f"{label}_pivot_pin_{idx}",
                elem_b="housing_shell",
                name=f"{label} vane pin {idx} is captured in the slot end",
            )

    # ---- clamshell service panels: closed seating and independent hinges ---
    closed_aabbs = []
    for i in range(NUM_LEAVES):
        leaf = leaves[i]
        hinge = hinges[i]

        ctx.expect_within(
            leaf, housing, axes="x", margin=0.002,
            name=f"panel_{i} within body width",
        )
        ctx.expect_overlap(
            leaf, housing, axes="xz", min_overlap=0.02,
            name=f"closed panel_{i} covers its half of the front face",
        )
        limits = hinge.motion_limits
        ctx.check(
            f"panel_{i} hinge opens upward 0..~60 deg about the width axis",
            limits is not None
            and abs(limits.lower) < 1e-6
            and abs(limits.upper - PANEL_OPEN_MAX) < 0.02
            and tuple(hinge.axis) == (1.0, 0.0, 0.0),
            details=f"axis={hinge.axis}, limits=({limits.lower}, {limits.upper})",
        )
        pb_closed = ctx.part_world_aabb(leaf)
        plate_closed = ctx.part_element_world_aabb(leaf, elem=f"panel_{i}_plate")
        assert pb_closed is not None and plate_closed is not None
        closed_aabbs.append(pb_closed)
        ctx.check(
            f"closed panel_{i} plate sits proud of the recessed chassis face",
            plate_closed[0][1] > PANEL_RECESS_Y + 0.0005,
            details=f"plate min y={plate_closed[0][1]:.4f}",
        )

    # Two leaves are side-by-side with distinct X centers.
    l0_pos = ctx.part_world_position(leaves[0])
    l1_pos = ctx.part_world_position(leaves[1])
    assert l0_pos is not None and l1_pos is not None
    ctx.check(
        "clamshell leaves are side-by-side with left leaf at negative X",
        l0_pos[0] < -0.05 and l1_pos[0] > 0.05,
        details=f"leaf0 x={l0_pos[0]:.4f}, leaf1 x={l1_pos[0]:.4f}",
    )

    # Each leaf opens independently: open only leaf 0, verify leaf 1 stays closed.
    with ctx.pose({hinges[0]: 1.0}):
        pb_open_0 = ctx.part_world_aabb(leaves[0])
        pb_still_1 = ctx.part_world_aabb(leaves[1])
        assert pb_open_0 is not None and pb_still_1 is not None
        ctx.check(
            "panel_0 swings outward when its hinge is actuated",
            pb_open_0[1][1] > hb[1][1] + 0.06 and pb_open_0[0][2] > closed_aabbs[0][0][2] + 0.04,
            details=f"open aabb={pb_open_0}",
        )
        ctx.check(
            "panel_1 stays closed when only panel_0 is opened",
            abs(pb_still_1[0][2] - closed_aabbs[1][0][2]) < 0.005
            and abs(pb_still_1[1][1] - closed_aabbs[1][1][1]) < 0.005,
            details=f"leaf1 closed aabb={pb_still_1}",
        )
        # Filter cavity contents are revealed behind the lifted cover.
        filt = ctx.part_element_world_aabb(housing, elem="filter_mesh")
        assert filt is not None
        ctx.check(
            "filter panel sits inside the cavity behind the covers",
            POCKET_Y0 - 0.001 <= filt[0][1]
            and filt[1][1] <= POCKET_Y1
            and POCKET_Z0 <= filt[0][2]
            and filt[1][2] <= POCKET_Z1,
            details=f"filter aabb={filt}",
        )

    # ---- louver vanes: placement, independence, motion ---------------------
    prev_z = -1.0
    prev_y = -1.0
    for (label, theta), vane, pivot in zip(VANE_SPECS, vanes, pivots):
        vb = ctx.part_world_aabb(vane)
        assert vb is not None
        cz = 0.5 * (vb[0][2] + vb[1][2])
        cy = 0.5 * (vb[0][1] + vb[1][1])
        ay, az = _arc_point(theta)
        ctx.check(
            f"{label} vane is centered in its slot on the lower front curve",
            abs(cz - az) < 0.004 and abs(cy - ay) < 0.004,
            details=f"center=({cy:.4f},{cz:.4f}), slot=({ay:.4f},{az:.4f})",
        )
        ctx.check(
            f"{label} vane stacks above the previous vane and further forward",
            cz > prev_z and cy > prev_y,
            details=f"cy={cy:.4f}, cz={cz:.4f}",
        )
        prev_z, prev_y = cz, cy
        ctx.expect_within(vane, housing, axes="x", margin=0.001, name=f"{label} vane within slot length")
        vlimits = pivot.motion_limits
        ctx.check(
            f"{label} vane pivots +/-45 deg about the width axis",
            vlimits is not None
            and abs(vlimits.lower + VANE_SWING) < 0.02
            and abs(vlimits.upper - VANE_SWING) < 0.02
            and tuple(pivot.axis) == (1.0, 0.0, 0.0),
            details=f"axis={pivot.axis}, limits=({vlimits.lower}, {vlimits.upper})",
        )
        # Decisive motion check: tilting the blade toward the airflow-down
        # extreme makes its world z-extent grow versus the opposite extreme,
        # proving rotation happens about the X (width) axis at the slot.
        with ctx.pose({pivot: VANE_SWING}):
            up = ctx.part_world_aabb(vane)
        with ctx.pose({pivot: -VANE_SWING}):
            dn = ctx.part_world_aabb(vane)
        assert up is not None and dn is not None
        ext_up = up[1][2] - up[0][2]
        ext_dn = dn[1][2] - dn[0][2]
        ctx.check(
            f"{label} vane blade actually tilts in its slot",
            ext_up > ext_dn + 0.003,
            details=f"z-extent +45deg={ext_up:.4f}, -45deg={ext_dn:.4f}",
        )

    # ---- hollow dark interior behind the slots -----------------------------
    liner = ctx.part_element_world_aabb(housing, elem="plenum_liner")
    assert liner is not None
    ctx.check(
        "dark plenum liner spans behind all three louver slots",
        liner[0][0] < -0.40
        and liner[1][0] > 0.40
        and liner[0][2] < 0.06
        and liner[1][2] > 0.19,
        details=f"liner aabb={liner}",
    )

    return ctx.report()


object_model = build_object_model()
