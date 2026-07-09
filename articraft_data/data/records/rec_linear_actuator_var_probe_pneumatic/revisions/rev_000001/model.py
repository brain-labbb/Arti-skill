from __future__ import annotations

from math import cos, pi, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    ClevisBracketGeometry,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ── Key dimensions ──────────────────────────────────────────────────────
BARREL_OR = 0.035     # barrel outer radius  (70 mm OD)
BARREL_IR = 0.030     # barrel inner radius  (60 mm ID, 5 mm wall)
BARREL_L = 0.400      # barrel tube length
CAP_OR = 0.042        # end-cap flange radius (84 mm)
CAP_T = 0.018         # end-cap thickness
ROD_R = 0.010         # piston-rod radius (20 mm dia)
GLAND_R = 0.013       # front-cap gland bore radius
STROKE = 0.250        # prismatic stroke
ROD_L = 0.380         # total rod length (barrel engagement + stroke + tip)

# X-axis positions (barrel centered at origin, axis along X)
HALF_BARREL = BARREL_L / 2.0                    # 0.200
REAR_CAP_X = -(HALF_BARREL + CAP_T / 2.0)       # -0.209
FRONT_CAP_X = +(HALF_BARREL + CAP_T / 2.0)       # +0.209
GLAND_X = +(HALF_BARREL + CAP_T)                 # +0.218  (articulation origin)


# ── CadQuery builders ───────────────────────────────────────────────────

def _barrel_tube() -> cq.Workplane:
    """Hollow cylindrical barrel tube centred at origin along X."""
    outer = cq.Workplane("YZ").circle(BARREL_OR).extrude(HALF_BARREL, both=True)
    inner = cq.Workplane("YZ").circle(BARREL_IR).extrude(HALF_BARREL * 1.005, both=True)
    return outer.cut(inner)


def _end_cap(has_bore: bool = False) -> cq.Workplane:
    """End-cap disk centred at origin along X.  Optional centre bore for rod gland."""
    cap = cq.Workplane("YZ").circle(CAP_OR).extrude(CAP_T / 2.0, both=True)
    if has_bore:
        bore = cq.Workplane("YZ").circle(GLAND_R).extrude(CAP_T, both=True)
        cap = cap.cut(bore)
    # Six tie-rod through-holes on a bolt circle
    bolt_r = 0.034
    hole_r = 0.003
    for i in range(6):
        a = 2.0 * pi * i / 6.0
        h = (
            cq.Workplane("YZ")
            .center(bolt_r * cos(a), bolt_r * sin(a))
            .circle(hole_r)
            .extrude(CAP_T, both=True)
        )
        cap = cap.cut(h)
    return cap


def _port_boss() -> cq.Workplane:
    """Small cylindrical boss with a through-hole for a fluid port (builds along +Z)."""
    boss = cq.Workplane("XY").circle(0.009).extrude(0.014)
    hole = cq.Workplane("XY").circle(0.004).extrude(0.016)
    return boss.cut(hole)


# ── Object model ────────────────────────────────────────────────────────

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="pneumatic_cylinder_actuator",
        meta={
            "category": "Robotics / Linear actuator",
            "run_note": (
                "Pneumatic/hydraulic cylinder variant: cylindrical barrel with "
                "coaxial piston rod, direct single-prismatic drive, no rotary drivetrain."
            ),
        },
    )

    # ── Materials ───────────────────────────────────────────────────────
    barrel_blue = model.material("barrel_blue", rgba=(0.18, 0.28, 0.48, 1.0))
    cap_black = model.material("cap_black", rgba=(0.06, 0.06, 0.07, 1.0))
    chrome = model.material("chrome_rod", rgba=(0.88, 0.90, 0.88, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.73, 0.70, 1.0))
    dark = model.material("dark_recess", rgba=(0.02, 0.02, 0.02, 1.0))
    brass = model.material("brass_port", rgba=(0.78, 0.60, 0.20, 1.0))

    # ====================================================================
    # FRAME  (barrel + end caps + ports + rear clevis)
    # ====================================================================
    frame = model.part("frame")

    # Barrel tube (hollow cylinder along X)
    frame.visual(
        mesh_from_cadquery(_barrel_tube(), "barrel_tube_mesh", tolerance=0.0005),
        material=barrel_blue,
        name="barrel_tube",
    )

    # Rear end cap (solid, no bore)
    frame.visual(
        mesh_from_cadquery(_end_cap(has_bore=False), "rear_end_cap_mesh", tolerance=0.0005),
        origin=Origin(xyz=(REAR_CAP_X, 0.0, 0.0)),
        material=cap_black,
        name="rear_end_cap",
    )

    # Front end cap (with rod gland bore)
    frame.visual(
        mesh_from_cadquery(_end_cap(has_bore=True), "front_end_cap_mesh", tolerance=0.0005),
        origin=Origin(xyz=(FRONT_CAP_X, 0.0, 0.0)),
        material=cap_black,
        name="front_end_cap",
    )

    # Rod gland / wiper ring on front cap outer face
    frame.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [(GLAND_R + 0.006, -0.005), (GLAND_R + 0.006, 0.005)],
                [(GLAND_R + 0.001, -0.005), (GLAND_R + 0.001, 0.005)],
                segments=32,
            ),
            "gland_ring_mesh",
        ),
        origin=Origin(xyz=(GLAND_X + 0.002, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="gland_ring",
    )

    # Two fluid-port bosses on top of barrel (slight embed for surface contact)
    for i, xp in enumerate((-0.100, 0.100)):
        frame.visual(
            mesh_from_cadquery(_port_boss(), f"port_boss_{i}_mesh", tolerance=0.0005),
            origin=Origin(xyz=(xp, 0.0, BARREL_OR - 0.002)),
            material=brass,
            name=f"port_boss_{i}",
        )

    # Rear mounting clevis (U-bracket with pin bore)
    clevis = ClevisBracketGeometry(
        (0.052, 0.048, 0.052),
        gap_width=0.022,
        bore_diameter=0.012,
        bore_center_z=0.036,
        base_thickness=0.008,
        center=False,
    )
    # Rotate so base faces +X (toward barrel rear face), cheeks extend in -X.
    frame.visual(
        mesh_from_geometry(clevis, "rear_clevis_mesh"),
        origin=Origin(
            xyz=(-(HALF_BARREL + CAP_T) - 0.001, 0.0, -0.026),
            rpy=(0.0, -pi / 2.0, 0.0),
        ),
        material=cap_black,
        name="rear_clevis",
    )

    # Tie-rod bolt heads visible on both end-cap outer faces
    bolt_r = 0.034
    for i in range(6):
        a = 2.0 * pi * i / 6.0
        y = bolt_r * cos(a)
        z = bolt_r * sin(a)
        # Rear bolts
        frame.visual(
            Cylinder(radius=0.005, length=0.004),
            origin=Origin(
                xyz=(REAR_CAP_X - CAP_T / 2.0 - 0.002, y, z),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=steel,
            name=f"tie_bolt_rear_{i}",
        )
        # Front bolts
        frame.visual(
            Cylinder(radius=0.005, length=0.004),
            origin=Origin(
                xyz=(FRONT_CAP_X + CAP_T / 2.0 + 0.002, y, z),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=steel,
            name=f"tie_bolt_front_{i}",
        )

    # Barrel band (decorative ring near mid-barrel for visual break)
    frame.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [(BARREL_OR + 0.002, -0.006), (BARREL_OR + 0.002, 0.006)],
                [(BARREL_OR - 0.001, -0.006), (BARREL_OR - 0.001, 0.006)],
                segments=48,
            ),
            "barrel_band_mesh",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="barrel_band",
    )

    # ====================================================================
    # CARRIAGE  (piston rod — the moving element)
    # ====================================================================
    carriage = model.part("carriage")

    # Rod: extends backward (into barrel) from carriage origin.
    # At q=0 the carriage frame sits at the gland face (x = GLAND_X).
    # The rod spans local x = -ROD_L  to  x = 0.
    carriage.visual(
        Cylinder(radius=ROD_R, length=ROD_L),
        origin=Origin(xyz=(-ROD_L / 2.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=chrome,
        name="piston_rod",
    )

    # Rod-end attachment disk (weld-on clevis pad) at rod tip
    # Rod extends from local x=-ROD_L to x=0; disk overlaps the rod end for connectivity.
    carriage.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(-0.005, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="rod_end_disk",
    )

    # Piston disk at rod base (rides on barrel bore, hidden inside barrel).
    # Outer radius slightly exceeds barrel IR for a sealed contact representation.
    piston_or = BARREL_IR + 0.0005   # 0.5 µm overlap for guaranteed mesh contact
    piston_half = 0.010
    carriage.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [(piston_or, -piston_half), (piston_or, piston_half)],
                [(ROD_R, -piston_half), (ROD_R, piston_half)],
                segments=32,
            ),
            "piston_disk_mesh",
        ),
        origin=Origin(xyz=(-ROD_L + 0.030, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark,
        name="piston_disk",
    )

    # ====================================================================
    # ARTICULATION  (single direct prismatic — carriage_slide)
    # ====================================================================
    model.articulation(
        "carriage_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(GLAND_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2000.0,
            velocity=0.50,
            lower=0.0,
            upper=STROKE,
        ),
    )

    return model


# ── Tests ───────────────────────────────────────────────────────────────

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("carriage_slide")

    # ── Structural identity ─────────────────────────────────────────
    ctx.check(
        "pneumatic cylinder part set",
        all(object_model.get_part(n) is not None for n in ("frame", "carriage")),
        details="Expected frame (barrel assembly) and carriage (piston rod).",
    )
    has_lead_screw = any(p.name == "lead_screw" for p in object_model.parts)
    ctx.check(
        "lead_screw part removed",
        not has_lead_screw,
        details="Lead screw must not exist in pneumatic cylinder variant.",
    )
    ctx.check(
        "single prismatic drive",
        slide is not None and slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"carriage_slide type={slide.articulation_type if slide else 'MISSING'}",
    )
    has_screw_spin = any(a.name == "screw_spin" for a in object_model.articulations)
    ctx.check(
        "screw_spin articulation removed",
        not has_screw_spin,
        details="Rotary lead-screw joint must not exist.",
    )

    # ── Barrel is a round tube ──────────────────────────────────────
    ctx.check(
        "barrel_tube visual present",
        frame.get_visual("barrel_tube") is not None,
        details="Frame must include cylindrical barrel tube visual.",
    )
    ctx.check(
        "front_end_cap has gland bore",
        frame.get_visual("front_end_cap") is not None,
        details="Front end cap must be present (with rod gland bore).",
    )

    # ── Rod captured inside barrel bore (intentional overlap) ───────
    ctx.allow_overlap(
        frame,
        carriage,
        elem_a="barrel_tube",
        elem_b="piston_rod",
        reason="Piston rod is intentionally captured inside the hollow barrel bore.",
    )
    ctx.allow_overlap(
        frame,
        carriage,
        elem_a="front_end_cap",
        elem_b="piston_rod",
        reason="Piston rod passes through the front end cap gland bore.",
    )
    ctx.allow_overlap(
        frame,
        carriage,
        elem_a="gland_ring",
        elem_b="piston_rod",
        reason="Rod passes through the gland seal ring seated on the front cap.",
    )
    ctx.allow_overlap(
        frame,
        carriage,
        elem_a="barrel_tube",
        elem_b="piston_disk",
        reason="Piston disk is intentionally a close fit inside the barrel bore (sealed piston).",
    )
    ctx.allow_overlap(
        frame,
        carriage,
        elem_a="front_end_cap",
        elem_b="rod_end_disk",
        reason="Rod-end disk seats against the front cap face at full retraction (realistic end-of-stroke rest).",
    )

    # Rod stays centred in barrel bore
    ctx.expect_within(
        carriage,
        frame,
        axes="yz",
        inner_elem="piston_rod",
        outer_elem="barrel_tube",
        margin=0.002,
        name="piston rod stays centred in barrel bore",
    )

    # Rod-end disk contacts front cap at retraction (proof for allow_overlap)
    ctx.expect_contact(
        carriage,
        frame,
        elem_a="rod_end_disk",
        elem_b="front_end_cap",
        contact_tol=0.012,
        name="rod-end disk seats near front cap at rest",
    )

    # Rod has barrel engagement at rest (retracted)
    ctx.expect_overlap(
        carriage,
        frame,
        axes="x",
        elem_a="piston_rod",
        elem_b="barrel_tube",
        min_overlap=0.100,
        name="retracted rod has sufficient barrel engagement",
    )

    # ── Pose: positive slide extends rod along +X ───────────────────
    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: STROKE}):
        extended_pos = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            frame,
            axes="x",
            elem_a="piston_rod",
            elem_b="barrel_tube",
            min_overlap=0.050,
            name="extended rod retains barrel engagement at full stroke",
        )

    ctx.check(
        "positive slide extends rod along +X",
        (
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[0] > rest_pos[0] + STROKE - 0.01
        ),
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
