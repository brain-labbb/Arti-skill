"""Black tactical bi-fold riot shield (free-standing A-frame barrier).

Faithful to the reference image: a matte-black main panel with white POLICE
lettering high on the front, an angled top deflector bend, a forward-flared
lower kick plate, two steel clamp hinge brackets bolted across the top bend,
and a folding rear support panel with a perforated steel vision window. The
rear panel folds toward the main panel through the top hinge pins.
"""

from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------- dimensions
PANEL_T = 0.012           # panel wall thickness
FRONT_W = 0.55            # front panel width (Y)
REAR_W = 0.52             # rear support panel width (Y)

MID_Z0, MID_Z1 = 0.16, 0.88          # front mid section height span
BEND_ANGLE = math.radians(25.0)      # top deflector bend (leans rearward)
BEND_LEN = 0.22
BEND_START_Z = 0.85                  # overlaps the mid plate for a solid knee
FLARE_ANGLE = math.radians(30.0)     # lower kick plate (flares forward)
FLARE_LEN = 0.22
FLARE_START_Z = 0.19

HINGE_XYZ = (-0.115, 0.0, 1.02)      # fold hinge axis (root frame)
DEPLOY_TILT = 0.42                   # rad, rear panel lean at rest (A-frame)
BRACKET_Y = 0.155                    # clamp bracket centerline offset

REAR_LEN = 1.11                      # rear panel plate length
WINDOW_W, WINDOW_H = 0.20, 0.18      # vision window cutout (Y x panel-length)
WINDOW_ZC = -0.42                    # window center along rear panel (local)


def _front_panel_solid() -> cq.Workplane:
    """One bent plate: mid section + top deflector bend + lower kick flare."""
    mid = (
        cq.Workplane("XY")
        .box(PANEL_T, FRONT_W, MID_Z1 - MID_Z0)
        .translate((0.0, 0.0, (MID_Z0 + MID_Z1) / 2.0))
    )

    # Top deflector bend, leaning toward the rear (-X).
    d_bend = (-math.sin(BEND_ANGLE), 0.0, math.cos(BEND_ANGLE))
    bend_c = (
        d_bend[0] * BEND_LEN / 2.0,
        0.0,
        BEND_START_Z + d_bend[2] * BEND_LEN / 2.0,
    )
    bend = (
        cq.Workplane("XY")
        .box(PANEL_T, FRONT_W, BEND_LEN)
        .rotate((0, 0, 0), (0, 1, 0), -math.degrees(BEND_ANGLE))
        .translate(bend_c)
    )

    # Lower kick plate, flaring toward the front (+X) down to the ground.
    d_flare = (math.sin(FLARE_ANGLE), 0.0, -math.cos(FLARE_ANGLE))
    flare_c = (
        d_flare[0] * FLARE_LEN / 2.0,
        0.0,
        FLARE_START_Z + d_flare[2] * FLARE_LEN / 2.0,
    )
    flare = (
        cq.Workplane("XY")
        .box(PANEL_T, FRONT_W, FLARE_LEN)
        .rotate((0, 0, 0), (0, 1, 0), 180.0 - math.degrees(FLARE_ANGLE))
        .translate(flare_c)
    )

    panel = mid.union(bend).union(flare)

    # Molded horizontal stiffening ribs on the lower front face.
    for k in range(3):
        rib = (
            cq.Workplane("XY")
            .box(0.006, 0.50, 0.012)
            .translate((PANEL_T / 2.0 - 0.001, 0.0, 0.24 + 0.06 * k))
        )
        panel = panel.union(rib)

    return panel


def _rear_panel_solid() -> cq.Workplane:
    """Rear support panel: plate + hinge tongues, window/notch/pin-bore cuts.

    Local frame: hinge axis is +Y through the origin; the plate hangs along -Z.
    """
    plate = (
        cq.Workplane("XY")
        .box(PANEL_T, REAR_W, REAR_LEN)
        .translate((-0.004, 0.0, -0.012 - REAR_LEN / 2.0))
    )

    # Vision window cutout (the perforated mesh screen mounts behind it).
    window = (
        cq.Workplane("XY")
        .box(0.06, WINDOW_W, WINDOW_H)
        .translate((-0.004, 0.0, WINDOW_ZC))
    )
    plate = plate.cut(window)

    # Top-edge notches so the clamp bracket cheeks clear the folding panel.
    for sy in (-1.0, 1.0):
        notch = (
            cq.Workplane("XY")
            .box(0.06, 0.06, 0.11)
            .translate((-0.004, sy * BRACKET_Y, -0.035))
        )
        plate = plate.cut(notch)

    # Hinge tongues rising into the bracket clevis slots.
    for sy in (-1.0, 1.0):
        tongue = (
            cq.Workplane("XY")
            .box(0.016, 0.028, 0.118)
            .translate((-0.004, sy * BRACKET_Y, -0.041))
        )
        plate = plate.union(tongue)

    # Pin bores through the tongues. The bore is slightly undersized so the
    # tongues genuinely capture the hinge pins (scoped allowance in tests).
    bore = (
        cq.Workplane("XZ")
        .circle(0.0075)
        .extrude(REAR_W / 2.0 + 0.02, both=True)
    )
    plate = plate.cut(bore)

    return plate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bifold_police_riot_shield")

    matte_black = model.material("matte_black", rgba=(0.07, 0.07, 0.08, 1.0))
    lettering_white = model.material("lettering_white", rgba=(0.93, 0.93, 0.95, 1.0))
    steel = model.material("bracket_steel", rgba=(0.42, 0.43, 0.46, 1.0))
    gunmetal = model.material("pin_gunmetal", rgba=(0.30, 0.31, 0.33, 1.0))
    mesh_gray = model.material("mesh_gray", rgba=(0.22, 0.22, 0.24, 1.0))

    # ------------------------------------------------------------ front panel
    front = model.part("front_panel")
    front.visual(
        mesh_from_cadquery(_front_panel_solid(), "front_panel_body"),
        material=matte_black,
        name="front_panel_body",
    )

    # White POLICE lettering, proud of the upper front face.
    text_solid = cq.Workplane("XY").text("POLICE", 0.10, 0.005)
    front.visual(
        mesh_from_cadquery(text_solid, "police_lettering"),
        # Map text (x=read dir, y=up, z=relief) onto the panel face (+X normal).
        origin=Origin(xyz=(0.004, 0.0, 0.79), rpy=(math.pi / 2, 0.0, math.pi / 2)),
        material=lettering_white,
        name="police_lettering",
    )

    # Steel clamp hinge brackets bolted over the top deflector bend.
    bend_dir = (-math.sin(BEND_ANGLE), 0.0, math.cos(BEND_ANGLE))
    bend_nrm = (math.cos(BEND_ANGLE), 0.0, math.sin(BEND_ANGLE))
    for i, sy in enumerate((-1.0, 1.0)):
        yc = sy * BRACKET_Y
        for side, sc in (("inner", -1.0), ("outer", 1.0)):
            front.visual(
                Box((0.16, 0.006, 0.10)),
                origin=Origin(xyz=(-0.06, yc + sc * 0.021, 1.00)),
                material=steel,
                name=f"bracket_{i}_{side}_cheek",
            )
        front.visual(
            Box((0.05, 0.048, 0.008)),
            origin=Origin(xyz=(-0.06, yc, 1.0515)),
            material=steel,
            name=f"bracket_{i}_strap",
        )
        front.visual(
            Cylinder(radius=0.008, length=0.056),
            origin=Origin(
                xyz=(HINGE_XYZ[0], yc, HINGE_XYZ[2]),
                rpy=(math.pi / 2, 0.0, 0.0),
            ),
            material=gunmetal,
            name=f"hinge_pin_{i}",
        )
        # Clamp through-bolt heads on the bend front face.
        for j, s in enumerate((0.05, 0.13)):
            base = (
                s * bend_dir[0] + (PANEL_T / 2.0) * bend_nrm[0],
                yc,
                BEND_START_Z + s * bend_dir[2] + (PANEL_T / 2.0) * bend_nrm[2],
            )
            front.visual(
                Cylinder(radius=0.007, length=0.012),
                origin=Origin(
                    xyz=(
                        base[0] + 0.002 * bend_nrm[0],
                        base[1],
                        base[2] + 0.002 * bend_nrm[2],
                    ),
                    rpy=(0.0, math.pi / 2 - BEND_ANGLE, 0.0),
                ),
                material=gunmetal,
                name=f"bracket_{i}_bolt_{j}",
            )

    # ------------------------------------------------------------- rear panel
    rear = model.part("rear_panel")
    rear.visual(
        mesh_from_cadquery(_rear_panel_solid(), "rear_panel_body"),
        material=matte_black,
        name="rear_panel_body",
    )

    # Perforated steel vision screen mounted behind the window cutout.
    perf = PerforatedPanelGeometry(
        (0.24, 0.26),
        0.005,
        hole_diameter=0.009,
        pitch=0.016,
        frame=0.012,
        stagger=True,
    )
    rear.visual(
        mesh_from_geometry(perf, "vision_mesh"),
        origin=Origin(xyz=(-0.0105, 0.0, WINDOW_ZC), rpy=(0.0, math.pi / 2, 0.0)),
        material=mesh_gray,
        name="vision_mesh",
    )

    # ------------------------------------------------------------- fold hinge
    # At q=0 the rear panel hangs rearward at the deployed A-frame tilt.
    # Negative q folds it toward the back of the front panel (stowed).
    model.articulation(
        "fold_hinge",
        ArticulationType.REVOLUTE,
        parent=front,
        child=rear,
        origin=Origin(xyz=HINGE_XYZ, rpy=(0.0, DEPLOY_TILT, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.0, lower=-0.50, upper=0.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_panel")
    rear = object_model.get_part("rear_panel")
    hinge = object_model.get_articulation("fold_hinge")

    limits = hinge.motion_limits
    ctx.check(
        "fold hinge is revolute with a stow range",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and limits is not None
        and limits.lower is not None
        and limits.lower <= -0.45,
        details=f"type={hinge.articulation_type}, limits={limits}",
    )

    # Both wings share the shield width.
    ctx.expect_overlap(front, rear, axes="y", min_overlap=0.45, name="wings share width")

    # Deployed A-frame stance: both panels reach the ground plane.
    fa = ctx.part_world_aabb(front)
    ra = ctx.part_world_aabb(rear)
    ctx.check(
        "front panel kick plate reaches the ground",
        fa is not None and fa[0][2] < 0.03,
        details=f"front aabb={fa}",
    )
    ctx.check(
        "rear panel foot reaches the ground",
        ra is not None and ra[0][2] < 0.03,
        details=f"rear aabb={ra}",
    )
    ctx.check(
        "rear panel splays rearward while the kick plate flares forward",
        fa is not None and ra is not None and ra[0][0] < -0.35 and fa[1][0] > 0.08,
        details=f"front aabb={fa}, rear aabb={ra}",
    )

    # POLICE lettering sits proud of the upper front face.
    txt = ctx.part_element_world_aabb(front, elem="police_lettering")
    ctx.check(
        "POLICE lettering proud on the upper front face",
        txt is not None
        and txt[1][0] > PANEL_T / 2.0 + 0.001
        and 0.72 < (txt[0][2] + txt[1][2]) / 2.0 < 0.88,
        details=f"lettering aabb={txt}",
    )

    # Perforated vision screen stays behind the rear panel window.
    ctx.expect_within(
        rear,
        rear,
        axes="y",
        inner_elem="vision_mesh",
        outer_elem="rear_panel_body",
        margin=0.0,
        name="vision mesh stays within the rear panel width",
    )

    # Hinge pins are captured inside the rear panel tongue bores.
    for i in range(2):
        ctx.allow_overlap(
            front,
            rear,
            elem_a=f"hinge_pin_{i}",
            elem_b="rear_panel_body",
            reason=(
                "The bracket hinge pin is intentionally captured inside the "
                "rear panel tongue bore (seated pivot fit)."
            ),
        )
        ctx.expect_contact(
            front,
            rear,
            elem_a=f"hinge_pin_{i}",
            elem_b="rear_panel_body",
            name=f"hinge pin {i} seats in the rear panel tongue bore",
        )

    # Folding: at the stow limit the rear panel swings up toward the front
    # panel without cutting through it.
    assert limits is not None and limits.lower is not None
    with ctx.pose({hinge: limits.lower}):
        folded = ctx.part_world_aabb(rear)
        ctx.check(
            "rear panel folds toward the front panel",
            folded is not None and ra is not None and folded[0][0] > ra[0][0] + 0.25,
            details=f"deployed aabb={ra}, folded aabb={folded}",
        )
        ctx.check(
            "stowed rear panel stays behind the front panel back face",
            folded is not None and folded[1][0] < -0.01,
            details=f"folded aabb={folded}",
        )

    return ctx.report()


object_model = build_object_model()
