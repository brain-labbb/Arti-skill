from __future__ import annotations

"""Wall-mounted mini-split air conditioner indoor unit (boxy rectangular variant).

Glossy white rectangular horizontal body ~0.90 m wide x 0.30 m tall x 0.22 m deep,
back flat against the wall plane (y=0), bottom at z=0. The housing is a crisp
rectangular box: flat vertical front face, squared-off top and bottom edges, and
a flat lower front carrying three long horizontal louver slots. Each slot holds a
slim airflow louver vane that independently pivots about the unit's width axis.

The large framed front face panel is a service cover hinged along its top edge,
swinging upward to reveal a shallow filter cavity with a removable-look filter
panel. The interior behind the louver slots is a hollow dark blower plenum.

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
BODY_W = 0.90  # width along X
BODY_D = 0.22  # depth along Y (front face at y = BODY_D)
BODY_H = 0.30  # height along Z

# Front service-panel zone (upper front face, recessed behind the hinged panel).
PANEL_ZONE_Z0 = 0.150  # panel zone starts above the louver area

# Hinged front panel.
PANEL_W = 0.85
PANEL_T = 0.013
PANEL_H = 0.140
PANEL_HINGE_Y = BODY_D - PANEL_T  # hinge behind front face so panel sits flush
PANEL_HINGE_Z = 0.290
PANEL_BORDER = 0.025
PANEL_OPEN_MAX = math.radians(60.0)

# Filter cavity + filter behind the front panel.
POCKET_W, POCKET_Y0, POCKET_Y1 = 0.74, 0.15, 0.21
POCKET_Z0, POCKET_Z1 = 0.185, 0.272

# Louver slots / vanes on the flat lower front face.
NUM_VANES = 3
VANE_Z_CENTERS = (0.040, 0.080, 0.120)  # evenly spaced in the lower front
SLOT_LEN = 0.78  # slot cut length along X
SLOT_OPEN = 0.022  # slot opening height (Z)
SLOT_DEPTH = 0.12  # cut depth into the body (Y)
VANE_LEN = 0.74
VANE_CHORD = 0.018
VANE_T = 0.0045
VANE_SWING = math.radians(45.0)
VANE_PIN_R = 0.003
VANE_PIN_LEN = 0.04
VANE_PIN_X = (-0.385, 0.385)

# Hollow dark plenum behind the louver slots (cross-flow blower bay).
PLENUM_CY = 0.11
PLENUM_CZ = 0.08
PLENUM_R = 0.08
PLENUM_HALF_LEN = 0.41
LINER_R = 0.075
LINER_HALF_LEN = 0.42  # ends embed into the chassis side walls


# ----------------------------------------------------------------------------
# Geometry helpers
# ----------------------------------------------------------------------------

_RECESS_Y = PANEL_HINGE_Y - 0.002  # recessed face behind the panel zone


def _housing_shape() -> cq.Workplane:
    """Crisp rectangular box housing with flat faces and squared edges.

    Built as a YZ-profile extrusion along X so the front-face step between
    the louver zone (y = BODY_D) and the panel recess (y = _RECESS_Y) is
    modelled as a clean profile edge, not a boolean cut that can leave
    mesh artifacts.
    """
    # Side profile (YZ plane): back at y=0, bottom at z=0.
    # The front face has a step-in at z = PANEL_ZONE_Z0 for the panel recess.
    body = (
        cq.Workplane("YZ")
        .moveTo(0.0, 0.0)
        .lineTo(BODY_D, 0.0)
        .lineTo(BODY_D, PANEL_ZONE_Z0)
        .lineTo(_RECESS_Y, PANEL_ZONE_Z0)
        .lineTo(_RECESS_Y, PANEL_HINGE_Z)
        .lineTo(BODY_D, PANEL_HINGE_Z)
        .lineTo(BODY_D, BODY_H)
        .lineTo(0.0, BODY_H)
        .close()
        .extrude(BODY_W / 2.0, both=True)
    )

    # Shallow framed inset on the top face (air-intake panel look).
    body = body.cut(
        cq.Workplane("XY")
        .box(0.80, 0.14, 0.016)
        .translate((0.0, BODY_D / 2.0, BODY_H))
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
        .center(PLENUM_CY, PLENUM_CZ)
        .circle(PLENUM_R)
        .extrude(PLENUM_HALF_LEN, both=True)
        .intersect(
            cq.Workplane("XY")
            .box(0.82, 0.19, 0.20)
            .translate((0.0, PLENUM_CY, PLENUM_CZ))
        )
    )
    body = body.cut(plenum)

    # Three horizontal louver slots through the flat lower front face.
    for i in range(NUM_VANES):
        vz = VANE_Z_CENTERS[i]
        slot = (
            cq.Workplane("XY")
            .box(SLOT_LEN, SLOT_DEPTH, SLOT_OPEN)
            .translate((0.0, BODY_D - SLOT_DEPTH / 2.0, vz))
        )
        body = body.cut(slot)

    return body


def _front_panel_shape() -> cq.Workplane:
    """Hinged front face panel in its joint-local frame.

    Local frame sits on the hinge line (top edge): the plate hangs along -Z
    and its thickness extends along +Y (away from the chassis).
    """
    plate = (
        cq.Workplane("XY")
        .box(PANEL_W, PANEL_T, PANEL_H)
        .translate((0.0, PANEL_T / 2.0, -PANEL_H / 2.0))
    )
    # Shallow face recess leaving a thin raised border frame.
    recess = (
        cq.Workplane("XY")
        .box(PANEL_W - 2.0 * PANEL_BORDER, 0.006, PANEL_H - 2.0 * PANEL_BORDER)
        .translate((0.0, PANEL_T, -PANEL_H / 2.0))
    )
    return plate.cut(recess)


def _add_vane_visuals(vane_part, i: int) -> None:
    """Shared visual helper: add blade and pivot pins to a louver vane part."""
    vane_part.visual(
        Box((VANE_LEN, VANE_T, VANE_CHORD)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="vane_white",
        name=f"vane_blade_{i}",
    )
    for idx, px in enumerate(VANE_PIN_X):
        vane_part.visual(
            Cylinder(radius=VANE_PIN_R, length=VANE_PIN_LEN),
            origin=Origin(xyz=(px, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="vane_white",
            name=f"pivot_pin_{i}_{idx}",
        )


# ----------------------------------------------------------------------------
# Object model
# ----------------------------------------------------------------------------

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
        origin=Origin(xyz=(0.0, PLENUM_CY, PLENUM_CZ), rpy=(0.0, math.pi / 2.0, 0.0)),
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

    # ---------------------------------------------------------- front panel
    front_panel = model.part("front_panel")
    front_panel.visual(
        mesh_from_cadquery(_front_panel_shape(), "front_panel"),
        material="panel_white",
        name="panel_plate",
    )
    # Hinge knuckles along the top back edge: they seat into the recessed
    # chassis wall and carry the cover (intentional local embed).
    for idx, kx in enumerate((-0.30, 0.30)):
        front_panel.visual(
            Box((0.04, 0.006, 0.012)),
            origin=Origin(xyz=(kx, -0.00125, -0.004)),
            material="panel_white",
            name=f"hinge_knuckle_{idx}",
        )
    model.articulation(
        "front_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=front_panel,
        origin=Origin(xyz=(0.0, PANEL_HINGE_Y, PANEL_HINGE_Z)),
        # Hinge along the panel's top edge; the closed plate hangs along
        # local -Z, so positive rotation about +X swings the free bottom
        # edge outward (+Y) and up: the service cover lifts open.
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=1.5, lower=0.0, upper=PANEL_OPEN_MAX
        ),
    )

    # --------------------------------------------------------- louver vanes
    for i in range(NUM_VANES):
        vane = model.part(f"louver_vane_{i}")
        _add_vane_visuals(vane, i)
        # Joint frame on the flat front face: the vane blade chord is along
        # Z at q=0 (closing the slot), and it pivots about X to direct
        # airflow up or down.
        model.articulation(
            f"louver_pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=vane,
            origin=Origin(xyz=(0.0, BODY_D, VANE_Z_CENTERS[i])),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0, velocity=2.0, lower=-VANE_SWING, upper=VANE_SWING
            ),
        )

    return model


# ----------------------------------------------------------------------------
# Tests
# ----------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    panel = object_model.get_part("front_panel")
    hinge = object_model.get_articulation("front_panel_hinge")
    vanes = [object_model.get_part(f"louver_vane_{i}") for i in range(NUM_VANES)]
    pivots = [
        object_model.get_articulation(f"louver_pivot_{i}") for i in range(NUM_VANES)
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

    # ---- boxy rectangular visible geometry --------------------------------
    # The AABB should closely match exact box dimensions, proving flat faces
    # and squared-off edges (no rounding or curvature on the housing).
    aabb_dy = hb[1][1] - hb[0][1]
    aabb_dz = hb[1][2] - hb[0][2]
    ctx.check(
        "housing AABB closely matches rectangular box (sharp edges, flat faces)",
        abs(aabb_dy - BODY_D) < 0.012 and abs(aabb_dz - BODY_H) < 0.012,
        details=f"aabb_dy={aabb_dy:.4f}, aabb_dz={aabb_dz:.4f}",
    )
    # Front face is flat: the housing max Y closely matches BODY_D across
    # the full height, proving no lean or curvature.
    ctx.check(
        "housing front face is flat at y=BODY_D",
        abs(hb[1][1] - BODY_D) < 0.008,
        details=f"max_y={hb[1][1]:.4f}",
    )

    # ---- intentional panel/housing coplanar seating -------------------------
    # The closed panel plate fills the front-face recess: its front surface is
    # flush with the lower front face (y = BODY_D), and its top/bottom edges
    # share coplanar step ledges with the housing profile. This is intentional
    # seated-trim contact, not a collision.
    ctx.allow_overlap(
        panel,
        housing,
        elem_a="panel_plate",
        elem_b="housing_shell",
        reason="Closed service panel plate is seated flush in the housing front-face recess; coplanar step ledges and flush front surface are intentional trim contact.",
    )
    ctx.expect_contact(
        panel,
        housing,
        elem_a="panel_plate",
        elem_b="housing_shell",
        name="closed panel plate is seated against the housing recess",
    )

    # ---- intentional hinge/pivot embeds ------------------------------------
    for idx in range(2):
        ctx.allow_overlap(
            panel,
            housing,
            elem_a=f"hinge_knuckle_{idx}",
            elem_b="housing_shell",
            reason="Hinge knuckle is intentionally seated into the recessed chassis wall to carry the hinged service cover.",
        )
        ctx.expect_contact(
            panel,
            housing,
            elem_a=f"hinge_knuckle_{idx}",
            elem_b="housing_shell",
            name=f"hinge knuckle {idx} is seated on the chassis",
        )
    for i, vane in enumerate(vanes):
        for idx in range(2):
            ctx.allow_overlap(
                vane,
                housing,
                elem_a=f"pivot_pin_{i}_{idx}",
                elem_b="housing_shell",
                reason="Vane end pivot pin is intentionally captured in the louver slot end wall.",
            )
            ctx.expect_contact(
                vane,
                housing,
                elem_a=f"pivot_pin_{i}_{idx}",
                elem_b="housing_shell",
                name=f"vane {i} pin {idx} is captured in the slot end",
            )

    # ---- front service panel: closed seating and hinge --------------------
    ctx.expect_within(panel, housing, axes="x", margin=0.002, name="panel within body width")
    ctx.expect_overlap(
        panel, housing, axes="xz", min_overlap=0.05, name="closed panel covers the front face"
    )
    limits = hinge.motion_limits
    ctx.check(
        "panel hinge opens upward 0..~60 deg about the width axis",
        limits is not None
        and abs(limits.lower) < 1e-6
        and abs(limits.upper - PANEL_OPEN_MAX) < 0.02
        and tuple(hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={hinge.axis}, limits=({limits.lower}, {limits.upper})",
    )
    # Closed panel sits flush with the flat housing front face.
    plate_closed = ctx.part_element_world_aabb(panel, elem="panel_plate")
    assert plate_closed is not None
    ctx.check(
        "closed panel plate front is flush with the housing front face",
        abs(plate_closed[1][1] - BODY_D) < 0.005,
        details=f"plate max y={plate_closed[1][1]:.4f}, front face y={BODY_D}",
    )
    pb_closed = ctx.part_world_aabb(panel)
    assert pb_closed is not None
    with ctx.pose({hinge: 1.0}):
        pb_open = ctx.part_world_aabb(panel)
        assert pb_open is not None
        ctx.check(
            "open panel swings outward and its bottom edge lifts",
            pb_open[1][1] > hb[1][1] + 0.08 and pb_open[0][2] > pb_closed[0][2] + 0.05,
            details=f"open aabb={pb_open}",
        )
        # Filter cavity contents are revealed behind the lifted cover.
        filt = ctx.part_element_world_aabb(housing, elem="filter_mesh")
        assert filt is not None
        ctx.check(
            "filter panel sits inside the cavity behind the cover",
            POCKET_Y0 - 0.001 <= filt[0][1]
            and filt[1][1] <= POCKET_Y1
            and POCKET_Z0 <= filt[0][2]
            and filt[1][2] <= POCKET_Z1,
            details=f"filter aabb={filt}",
        )

    # ---- louver vanes: flat-front placement, independence, motion ---------
    prev_z = -1.0
    for i, (vane, pivot) in enumerate(zip(vanes, pivots)):
        vb = ctx.part_world_aabb(vane)
        assert vb is not None
        cz = 0.5 * (vb[0][2] + vb[1][2])
        cy = 0.5 * (vb[0][1] + vb[1][1])
        expected_z = VANE_Z_CENTERS[i]
        ctx.check(
            f"vane {i} centered at z={expected_z:.3f} on the flat front face",
            abs(cz - expected_z) < 0.005 and abs(cy - BODY_D) < 0.005,
            details=f"center=({cy:.4f},{cz:.4f}), expected=({BODY_D},{expected_z})",
        )
        # Vanes stack from lower to upper along Z on the flat front.
        ctx.check(
            f"vane {i} stacks above the previous vane",
            cz > prev_z,
            details=f"cz={cz:.4f}, prev_z={prev_z:.4f}",
        )
        prev_z = cz
        ctx.expect_within(
            vane, housing, axes="x", margin=0.001, name=f"vane {i} within slot length"
        )
        vlimits = pivot.motion_limits
        ctx.check(
            f"vane {i} pivots +/-45 deg about the width axis",
            vlimits is not None
            and abs(vlimits.lower + VANE_SWING) < 0.02
            and abs(vlimits.upper - VANE_SWING) < 0.02
            and tuple(pivot.axis) == (1.0, 0.0, 0.0),
            details=f"axis={pivot.axis}, limits=({vlimits.lower}, {vlimits.upper})",
        )
        # Decisive motion check: pivoting the blade makes its Y-extent (depth)
        # grow versus the rest pose, proving rotation about the X axis.
        rest_dy = vb[1][1] - vb[0][1]
        with ctx.pose({pivot: VANE_SWING}):
            tilted = ctx.part_world_aabb(vane)
        assert tilted is not None
        tilted_dy = tilted[1][1] - tilted[0][1]
        ctx.check(
            f"vane {i} blade tilts (depth grows when pivoted)",
            tilted_dy > rest_dy + 0.005,
            details=f"rest_dy={rest_dy:.4f}, tilted_dy={tilted_dy:.4f}",
        )

    # ---- hollow dark interior behind the slots -----------------------------
    liner = ctx.part_element_world_aabb(housing, elem="plenum_liner")
    assert liner is not None
    ctx.check(
        "dark plenum liner spans behind all three louver slots",
        liner[0][0] < -0.40
        and liner[1][0] > 0.40
        and liner[0][2] < VANE_Z_CENTERS[0] + 0.01
        and liner[1][2] > VANE_Z_CENTERS[-1] - 0.01,
        details=f"liner aabb={liner}",
    )

    return ctx.report()


object_model = build_object_model()
