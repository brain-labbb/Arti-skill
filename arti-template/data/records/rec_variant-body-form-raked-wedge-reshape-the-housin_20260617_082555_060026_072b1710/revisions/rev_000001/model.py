from __future__ import annotations

"""Wall-mounted mini-split air conditioner indoor unit — raked wedge variant.

The housing side profile is a wedge: a strongly raked front face that leans far
forward at the bottom and recedes toward the top, giving a sharp angular
silhouette instead of the parent's near-vertical gently curved front.
All other functional layers (louver vanes, service panel, filter cavity, dark
plenum) are preserved from the parent.

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
BODY_H = 0.30  # height along Z (z in [0, 0.30])

# ----------------------------------------------------------------------------
# Wedge side-profile (y = depth from wall, z = height from bottom).
# The front face rakes strongly: far forward at the bottom, receding at top.
# ----------------------------------------------------------------------------
WEDGE_LO_Y = 0.26   # bottom-front corner (far forward)
WEDGE_HI_Y = 0.10   # top-front corner (receded toward the wall)

# Derived raked-face geometry.
FACE_DY = WEDGE_HI_Y - WEDGE_LO_Y   # -0.16
FACE_DZ = BODY_H                     # 0.30
FACE_LEN = math.sqrt(FACE_DY * FACE_DY + FACE_DZ * FACE_DZ)  # ~0.34
# Outward unit normal of the flat raked face (in the YZ plane).
FACE_NY = FACE_DZ / FACE_LEN          # ~0.882
FACE_NZ = -FACE_DY / FACE_LEN         # ~0.471
# Angle of the outward normal from the +Y axis (rotation about X).
FACE_NORMAL_ANGLE = math.atan2(FACE_NZ, FACE_NY)  # ~0.49 rad ≈ 28°


def _face_point(fraction: float) -> tuple[float, float]:
    """(y, z) point on the raked front face at a fraction from the bottom."""
    return (
        WEDGE_LO_Y + fraction * FACE_DY,
        fraction * FACE_DZ,
    )


# ----------------------------------------------------------------------------
# Front service-panel zone (hinged cover over the upper raked face)
# ----------------------------------------------------------------------------
PANEL_W = 0.85
PANEL_T = 0.013
PANEL_H = 0.1465
PANEL_BORDER = 0.025
PANEL_OPEN_MAX = math.radians(60.0)
# Hinge at the top-front edge of the wedge.
PANEL_HINGE_Y = WEDGE_HI_Y + 0.005   # ~0.105
PANEL_HINGE_Z = 0.290

# ----------------------------------------------------------------------------
# Filter cavity + filter behind the front panel
# ----------------------------------------------------------------------------
POCKET_W = 0.74
POCKET_Y0 = 0.04
POCKET_Y1 = 0.115
POCKET_Z0 = 0.185
POCKET_Z1 = 0.272

# ----------------------------------------------------------------------------
# Louver vanes on the lower raked face, parameterised by fraction from bottom.
# ----------------------------------------------------------------------------
VANE_SPECS = (
    ("lower", 0.10),
    ("middle", 0.22),
    ("upper", 0.34),
)
SLOT_LEN = 0.78       # slot cut length along X
SLOT_OPEN = 0.024     # slot opening along the face tangent
SLOT_DEPTH = 0.12     # cut depth along the face normal (into plenum)
VANE_LEN = 0.74
VANE_CHORD = 0.018
VANE_T = 0.0045
VANE_SWING = math.radians(45.0)

# ----------------------------------------------------------------------------
# Hollow dark plenum behind the louver slots (cross-flow blower bay)
# ----------------------------------------------------------------------------
PLENUM_CY = 0.12
PLENUM_CZ = 0.08
PLENUM_R = 0.08
PLENUM_HALF_LEN = 0.41
LINER_R = 0.075
LINER_HALF_LEN = 0.42  # ends embed into the chassis side walls


def _housing_shape() -> cq.Workplane:
    """Wedge-profile housing body.

    Side profile is a simple quadrilateral:
      (0, 0) → (WEDGE_LO_Y, 0) → (WEDGE_HI_Y, BODY_H) → (0, BODY_H)
    extruded symmetrically along X to the full body width.
    """
    body = (
        cq.Workplane("YZ")
        .moveTo(0.0, 0.0)
        .lineTo(WEDGE_LO_Y, 0.0)
        .lineTo(WEDGE_HI_Y, BODY_H)
        .lineTo(0.0, BODY_H)
        .close()
        .extrude(BODY_W / 2.0, both=True)
    )

    # Recess the front wall behind the hinged service panel.
    # A thin tilted slab cut removes ~2 mm of face material (in the normal
    # direction) across the panel zone, creating a step that the hinged
    # cover seats against.  The slab is aligned with the raked face.
    _panel_frac_center = 0.725
    _pcy, _pcz = _face_point(_panel_frac_center)
    _PANEL_ZONE_H = 0.168  # panel zone height along the face tangent
    _RECESS_SLAB_T = 0.006  # 6 mm slab → 3 mm behind face, 3 mm in front
    body = body.cut(
        cq.Workplane("XY")
        .box(BODY_W - 0.04, _RECESS_SLAB_T, _PANEL_ZONE_H)
        .rotate(
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            math.degrees(FACE_NORMAL_ANGLE),
        )
        .translate((0.0, _pcy, _pcz))
    )

    # Shallow framed inset on the top face (air-intake grille look).
    body = body.cut(
        cq.Workplane("XY")
        .box(0.80, 0.06, 0.013)
        .translate((0.0, 0.04, BODY_H - 0.003))
    )

    # Filter cavity pocket behind the front panel zone.
    body = body.cut(
        cq.Workplane("XY")
        .box(POCKET_W, POCKET_Y1 - POCKET_Y0, POCKET_Z1 - POCKET_Z0)
        .translate(
            (0.0, (POCKET_Y0 + POCKET_Y1) / 2.0, (POCKET_Z0 + POCKET_Z1) / 2.0)
        )
    )

    # Hollow blower plenum behind the louver slots, clipped to body interior.
    plenum = (
        cq.Workplane("YZ")
        .center(PLENUM_CY, PLENUM_CZ)
        .circle(PLENUM_R)
        .extrude(PLENUM_HALF_LEN, both=True)
        .intersect(
            cq.Workplane("XY")
            .box(0.82, 0.20, 0.20)
            .translate((0.0, 0.10, 0.10))
        )
    )
    body = body.cut(plenum)

    # Three long louver slots through the raked front face.
    for _, fraction in VANE_SPECS:
        ay, az = _face_point(fraction)
        slot = (
            cq.Workplane("XY")
            .box(SLOT_LEN, SLOT_DEPTH, SLOT_OPEN)
            .rotate(
                (0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
                math.degrees(FACE_NORMAL_ANGLE),
            )
            .translate((0.0, ay, az))
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
    # Removable-look filter panel seated against the back wall of the filter
    # cavity (frame slightly embedded into the cavity back wall for
    # connectivity — realistic seated filter placement).
    housing.visual(
        Box((0.70, 0.005, 0.082)),
        origin=Origin(xyz=(0.0, POCKET_Y0 - 0.001, 0.2285)),
        material="filter_gray",
        name="filter_frame",
    )
    housing.visual(
        Box((0.66, 0.004, 0.072)),
        origin=Origin(xyz=(0.0, POCKET_Y0 + 0.002, 0.2285)),
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
        # The joint frame is rotated about X so local +Y aligns with the
        # raked face outward normal; the panel plate then follows the rake.
        origin=Origin(
            xyz=(0.0, PANEL_HINGE_Y, PANEL_HINGE_Z),
            rpy=(FACE_NORMAL_ANGLE, 0.0, 0.0),
        ),
        # Hinge along the panel's top edge (width axis); positive rotation
        # swings the free bottom edge outward (+Y) and up: cover lifts open.
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=1.5, lower=0.0, upper=PANEL_OPEN_MAX
        ),
    )

    # --------------------------------------------------------- louver vanes
    for label, fraction in VANE_SPECS:
        vane = model.part(f"{label}_louver_vane")
        # Blade lies in the slot's tangent plane at q=0: local +Y is the
        # outward face normal, local +Z the face tangent (chord direction).
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
        ay, az = _face_point(fraction)
        model.articulation(
            f"{label}_louver_pivot",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=vane,
            # Joint frame rotated about X so local +Y matches the outward
            # normal of the raked face at this slot height.
            origin=Origin(
                xyz=(0.0, ay, az),
                rpy=(FACE_NORMAL_ANGLE, 0.0, 0.0),
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
    panel = object_model.get_part("front_panel")
    hinge = object_model.get_articulation("front_panel_hinge")
    vanes = [
        object_model.get_part(f"{label}_louver_vane") for label, _ in VANE_SPECS
    ]
    pivots = [
        object_model.get_articulation(f"{label}_louver_pivot")
        for label, _ in VANE_SPECS
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
        "housing max depth matches wedge bottom front (~0.26 m)",
        0.24 <= (hb[1][1] - hb[0][1]) <= 0.28,
        details=f"depth={hb[1][1] - hb[0][1]:.4f}",
    )
    ctx.check(
        "unit bottom sits at z=0 with flat back at the wall plane y=0",
        abs(hb[0][2]) < 0.005 and abs(hb[0][1]) < 0.005,
        details=f"min={hb[0]}",
    )

    # ---- wedge profile: raked face recedes toward top ---------------------
    ctx.check(
        "wedge profile: top front recedes far behind the bottom front",
        PANEL_HINGE_Y < hb[1][1] - 0.10,
        details=f"hinge_y={PANEL_HINGE_Y:.3f}, max_y={hb[1][1]:.3f}",
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

    # ---- front service panel: closed seating and hinge --------------------
    ctx.expect_within(
        panel, housing, axes="x", margin=0.002, name="panel within body width"
    )
    ctx.expect_overlap(
        panel,
        housing,
        axes="xz",
        min_overlap=0.05,
        name="closed panel covers the front face",
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
    pb_closed = ctx.part_world_aabb(panel)
    plate_closed = ctx.part_element_world_aabb(panel, elem="panel_plate")
    assert pb_closed is not None and plate_closed is not None
    ctx.check(
        "closed panel plate sits at or ahead of the receded top face",
        plate_closed[1][1] > WEDGE_HI_Y - 0.01,
        details=f"plate max y={plate_closed[1][1]:.4f}",
    )
    with ctx.pose({hinge: 1.0}):
        pb_open = ctx.part_world_aabb(panel)
        assert pb_open is not None
        ctx.check(
            "open panel bottom edge lifts significantly above closed position",
            pb_open[0][2] > pb_closed[0][2] + 0.05,
            details=f"closed_z0={pb_closed[0][2]:.4f}, open_z0={pb_open[0][2]:.4f}",
        )
        # Filter cavity contents are revealed behind the lifted cover.
        filt = ctx.part_element_world_aabb(housing, elem="filter_mesh")
        assert filt is not None
        ctx.check(
            "filter panel sits inside the cavity behind the cover",
            filt[0][1] >= POCKET_Y0 - 0.008
            and filt[1][1] <= POCKET_Y1 + 0.005
            and POCKET_Z0 <= filt[0][2]
            and filt[1][2] <= POCKET_Z1,
            details=f"filter aabb={filt}",
        )

    # ---- louver vanes: placement, independence, motion ---------------------
    prev_z = -1.0
    prev_y = 999.0
    for (label, fraction), vane, pivot in zip(VANE_SPECS, vanes, pivots):
        vb = ctx.part_world_aabb(vane)
        assert vb is not None
        cz = 0.5 * (vb[0][2] + vb[1][2])
        cy = 0.5 * (vb[0][1] + vb[1][1])
        ay, az = _face_point(fraction)
        ctx.check(
            f"{label} vane is centered on its slot on the raked face",
            abs(cz - az) < 0.006 and abs(cy - ay) < 0.006,
            details=f"center=({cy:.4f},{cz:.4f}), slot=({ay:.4f},{az:.4f})",
        )
        ctx.check(
            f"{label} vane stacks above the previous vane and recedes toward the wall",
            cz > prev_z and cy < prev_y,
            details=f"cy={cy:.4f}, cz={cz:.4f}, prev_y={prev_y:.4f}",
        )
        prev_z, prev_y = cz, cy
        ctx.expect_within(
            vane,
            housing,
            axes="x",
            margin=0.001,
            name=f"{label} vane within slot length",
        )
        vlimits = pivot.motion_limits
        ctx.check(
            f"{label} vane pivots +/-45 deg about the width axis",
            vlimits is not None
            and abs(vlimits.lower + VANE_SWING) < 0.02
            and abs(vlimits.upper - VANE_SWING) < 0.02
            and tuple(pivot.axis) == (1.0, 0.0, 0.0),
            details=f"axis={pivot.axis}, limits=({vlimits.lower}, {vlimits.upper})",
        )
        # Decisive motion check: tilting the blade toward one extreme makes
        # its world z-extent grow versus the opposite extreme, proving
        # rotation happens about the X (width) axis at the slot.
        with ctx.pose({pivot: VANE_SWING}):
            up = ctx.part_world_aabb(vane)
        with ctx.pose({pivot: -VANE_SWING}):
            dn = ctx.part_world_aabb(vane)
        assert up is not None and dn is not None
        ext_up = up[1][2] - up[0][2]
        ext_dn = dn[1][2] - dn[0][2]
        ctx.check(
            f"{label} vane blade actually tilts in its slot",
            abs(ext_up - ext_dn) > 0.002,
            details=f"z-extent +45deg={ext_up:.4f}, -45deg={ext_dn:.4f}",
        )

    # ---- hollow dark interior behind the slots -----------------------------
    liner = ctx.part_element_world_aabb(housing, elem="plenum_liner")
    assert liner is not None
    ctx.check(
        "dark plenum liner spans behind all three louver slots",
        liner[0][0] < -0.40
        and liner[1][0] > 0.40
        and liner[0][2] < 0.10
        and liner[1][2] > 0.05,
        details=f"liner aabb={liner}",
    )

    return ctx.report()


object_model = build_object_model()
