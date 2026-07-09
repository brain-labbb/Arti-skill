"""Round EVA airlock hatch on a white pressurised module.

Identity features from the reference picture:
- white cylindrical pressurised airlock (crew lock) with a circular top
  opening surrounded by a raised hatch collar and a tan pressure seal
- a large round hatch lid, tan thermal-cover faces, swung wide open on a
  side hinge bracket with visible lugs and pin
- an inner locking handwheel with spokes and rim on the lid's inner face
- radial latch dogs around the lid rim and matching strike plates on the
  collar, plus EVA grab handrails on the hull

Articulation: hatch_hinge revolute (rest pose = open as pictured; negative
motion closes the lid onto the seal) and handwheel_spin revolute.
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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------- layout ----
HULL_R = 1.00            # airlock hull outer radius
HULL_WALL = 0.12
HULL_TOP = 1.00          # hull tube top
PLATE_TOP = 1.06         # end plate top
COLLAR_RO = 0.60         # hatch collar outer radius
COLLAR_RI = 0.50         # hatch opening radius
COLLAR_TOP = 1.24
SEAL_TOP = 1.245         # tan seal top face

HINGE_X = -0.68          # hinge pin x (on the collar side)
HINGE_Z = 1.28           # hinge pin height
LID_R = 0.60
LID_OPEN = 1.9           # rest open angle (rad), as pictured


def _ring(ro: float, ri: float, h: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(ro).circle(ri).extrude(h)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="eva_airlock_hatch_module")

    hull_white = model.material("hull_white", rgba=(0.88, 0.88, 0.85, 1.0))
    plate_grey = model.material("plate_grey", rgba=(0.74, 0.74, 0.72, 1.0))
    seal_tan = model.material("seal_tan", rgba=(0.78, 0.64, 0.42, 1.0))
    lid_tan = model.material("lid_tan", rgba=(0.76, 0.68, 0.52, 1.0))
    metal_grey = model.material("metal_grey", rgba=(0.55, 0.56, 0.58, 1.0))
    silver = model.material("silver", rgba=(0.75, 0.76, 0.78, 1.0))
    rail_gold = model.material("rail_gold", rgba=(0.82, 0.70, 0.42, 1.0))
    hardware = model.material("dark_hardware", rgba=(0.30, 0.30, 0.32, 1.0))

    # ----------------------------------------------------------- airlock shell
    shell = model.part("airlock_shell")
    hull_tube = mesh_from_cadquery(_ring(HULL_R, HULL_R - HULL_WALL, HULL_TOP), "hull_tube")
    shell.visual(hull_tube, origin=Origin(), material=hull_white, name="hull_tube")
    shell.visual(
        Cylinder(radius=HULL_R, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=hull_white,
        name="hull_base_cap",
    )
    # Circumferential seam band on the hull.
    seam_mesh = mesh_from_cadquery(_ring(HULL_R + 0.015, HULL_R - 0.01, 0.05), "hull_seam")
    shell.visual(seam_mesh, origin=Origin(xyz=(0.0, 0.0, 0.55)), material=plate_grey,
                 name="hull_seam")
    # Top end plate with the circular hatch opening.
    end_plate = mesh_from_cadquery(_ring(HULL_R, COLLAR_RI, PLATE_TOP - HULL_TOP),
                                   "end_plate")
    shell.visual(end_plate, origin=Origin(xyz=(0.0, 0.0, HULL_TOP)), material=plate_grey,
                 name="end_plate")
    # Raised hatch collar (tunnel ring) and tan pressure seal.
    collar_mesh = mesh_from_cadquery(_ring(COLLAR_RO, COLLAR_RI, COLLAR_TOP - PLATE_TOP),
                                     "hatch_collar")
    shell.visual(collar_mesh, origin=Origin(xyz=(0.0, 0.0, PLATE_TOP)), material=hull_white,
                 name="hatch_collar")
    seal_ring = mesh_from_cadquery(_ring(COLLAR_RO, 0.51, SEAL_TOP - COLLAR_TOP),
                                   "seal_ring")
    shell.visual(seal_ring, origin=Origin(xyz=(0.0, 0.0, COLLAR_TOP)), material=seal_tan,
                 name="seal_ring")
    # Bolt ring on the end plate around the collar.
    for i in range(12):
        a = math.radians(30.0 * i)
        shell.visual(
            Cylinder(radius=0.020, length=0.03),
            origin=Origin(xyz=(0.80 * math.cos(a), 0.80 * math.sin(a), PLATE_TOP + 0.01)),
            material=metal_grey,
            name=f"plate_bolt_{i}",
        )
    # Latch strike plates around the collar exterior.
    for i in range(8):
        a = math.radians(45.0 * i + 22.5)
        shell.visual(
            Box((0.05, 0.08, 0.10)),
            origin=Origin(
                xyz=(0.615 * math.cos(a), 0.615 * math.sin(a), 1.16),
                rpy=(0.0, 0.0, a),
            ),
            material=metal_grey,
            name=f"latch_strike_{i}",
        )
    # Hinge bracket: two lugs rising from the end plate to the pin.
    for k, ly in enumerate((-0.11, 0.11)):
        shell.visual(
            Box((0.08, 0.05, 0.24)),
            origin=Origin(xyz=(HINGE_X, ly, 1.17)),
            material=metal_grey,
            name=f"hinge_bracket_lug_{k}",
        )
    shell.visual(
        Cylinder(radius=0.020, length=0.34),
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="hinge_pin",
    )
    # EVA grab handrails on the hull exterior.
    for i, a_deg in enumerate((130.0, 230.0)):
        a = math.radians(a_deg)
        ux, uy = math.cos(a), math.sin(a)
        for k, pz in enumerate((0.40, 0.80)):
            shell.visual(
                Cylinder(radius=0.016, length=0.08),
                origin=Origin(
                    xyz=((HULL_R + 0.03) * ux, (HULL_R + 0.03) * uy, pz),
                    rpy=(0.0, math.pi / 2.0, a),
                ),
                material=rail_gold,
                name=f"rail_post_{i}_{k}",
            )
        shell.visual(
            Cylinder(radius=0.018, length=0.52),
            origin=Origin(xyz=((HULL_R + 0.07) * ux, (HULL_R + 0.07) * uy, 0.60)),
            material=rail_gold,
            name=f"handrail_{i}",
        )

    # ------------------------------------------------------------- hatch lid
    # Lid local frame sits on the hinge axis; the lid extends along local -x
    # and its outward (space-side) face is +z. Rest pose is open as pictured.
    lid = model.part("hatch_lid")
    lid.visual(
        Cylinder(radius=LID_R, length=0.05),
        origin=Origin(xyz=(-0.68, 0.0, 0.015)),
        material=lid_tan,
        name="lid_disc",
    )
    lid.visual(
        Cylinder(radius=0.50, length=0.03),
        origin=Origin(xyz=(-0.68, 0.0, 0.055)),
        material=lid_tan,
        name="lid_dome_step",
    )
    lid.visual(
        Cylinder(radius=0.20, length=0.03),
        origin=Origin(xyz=(-0.68, 0.0, 0.085)),
        material=plate_grey,
        name="lid_hub_boss",
    )
    lid_seal = mesh_from_cadquery(_ring(LID_R, 0.50, 0.02), "lid_seal_ring")
    lid.visual(lid_seal, origin=Origin(xyz=(-0.68, 0.0, -0.03)), material=seal_tan,
               name="lid_seal_ring")
    # Radial latch dogs around the lid rim.
    for j in range(8):
        a = math.radians(45.0 * j + 22.5)
        lid.visual(
            Box((0.09, 0.06, 0.04)),
            origin=Origin(
                xyz=(-0.68 + 0.63 * math.cos(a), 0.63 * math.sin(a), 0.0),
                rpy=(0.0, 0.0, a),
            ),
            material=metal_grey,
            name=f"latch_dog_{j}",
        )
    # Lid-side hinge lugs wrapping the shell pin.
    for k, ly in enumerate((-0.05, 0.05)):
        lid.visual(
            Box((0.09, 0.04, 0.09)),
            origin=Origin(xyz=(0.0, ly, 0.0)),
            material=metal_grey,
            name=f"lid_hinge_lug_{k}",
        )
    lid.visual(
        Box((0.16, 0.12, 0.035)),
        origin=Origin(xyz=(-0.10, 0.0, 0.0)),
        material=metal_grey,
        name="hinge_strap",
    )
    model.articulation(
        "hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z), rpy=(0.0, LID_OPEN, math.pi)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.0, lower=-LID_OPEN, upper=0.08),
    )

    # ------------------------------------------------------- lock handwheel
    wheel = model.part("lock_handwheel")
    wheel.visual(
        Cylinder(radius=0.05, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=silver,
        name="wheel_hub",
    )
    for j in range(3):
        a = math.radians(120.0 * j)
        wheel.visual(
            Box((0.66, 0.03, 0.025)),
            origin=Origin(xyz=(0.0, 0.0, -0.028), rpy=(0.0, 0.0, a)),
            material=silver,
            name=f"wheel_spoke_{j}",
        )
    wheel_rim = mesh_from_cadquery(_ring(0.34, 0.30, 0.03), "wheel_rim")
    wheel.visual(wheel_rim, origin=Origin(xyz=(0.0, 0.0, -0.05)), material=hardware,
                 name="wheel_rim")
    model.articulation(
        "handwheel_spin",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=wheel,
        origin=Origin(xyz=(-0.68, 0.0, -0.03)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=-2.6, upper=2.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("airlock_shell")
    lid = object_model.get_part("hatch_lid")
    wheel = object_model.get_part("lock_handwheel")
    hinge = object_model.get_articulation("hatch_hinge")
    spin = object_model.get_articulation("handwheel_spin")

    # ---------------------------------------------- intentional embeddings --
    for k in range(2):
        ctx.allow_overlap(
            "airlock_shell",
            "hatch_lid",
            elem_a="hinge_pin",
            elem_b=f"lid_hinge_lug_{k}",
            reason="The hinge pin is captured inside the lid hinge lugs.",
        )
        ctx.allow_overlap(
            "airlock_shell",
            "hatch_lid",
            elem_a=f"hinge_bracket_lug_{k}",
            elem_b="hinge_strap",
            reason="The lid hinge strap nests between the bracket lugs at the pin.",
        )
    ctx.allow_overlap(
        "hatch_lid",
        "lock_handwheel",
        elem_a="lid_disc",
        elem_b="wheel_hub",
        reason="The handwheel shaft hub is seated through the lid disc.",
    )

    def _center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) / 2.0 for i in range(3))

    # -------------------------------------------------- structural checks --
    collar = ctx.part_element_world_aabb(shell, elem="hatch_collar")
    ctx.check(
        "raised hatch collar rings a ~1 m opening",
        collar is not None
        and (collar[1][0] - collar[0][0]) > 1.15
        and collar[1][2] > 1.2,
        details=f"collar={collar}",
    )
    seal = ctx.part_element_world_aabb(shell, elem="seal_ring")
    ctx.check("tan pressure seal caps the collar", seal is not None
              and abs(seal[1][2] - SEAL_TOP) < 0.01, details=f"seal={seal}")
    dogs = [v for v in lid.visuals if v.name and v.name.startswith("latch_dog_")]
    ctx.check("eight latch dogs around the lid rim", len(dogs) == 8, details=str(len(dogs)))
    strikes = [v for v in shell.visuals if v.name and v.name.startswith("latch_strike_")]
    ctx.check("eight strike plates on the collar", len(strikes) == 8,
              details=str(len(strikes)))
    rails = [v for v in shell.visuals if v.name and v.name.startswith("handrail_")]
    ctx.check("EVA handrails on the hull", len(rails) == 2, details=str(len(rails)))

    # ------------------------------------------------------ hinge checks --
    ctx.check(
        "hatch hinge is revolute and closes through -1.9 rad",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and hinge.motion_limits is not None
        and abs(hinge.motion_limits.lower + LID_OPEN) < 1e-6,
        details=str(hinge.motion_limits),
    )
    # Rest pose: the lid is swung wide open beside the opening, leaving it clear.
    lid_disc_open = ctx.part_element_world_aabb(lid, elem="lid_disc")
    lo = _center(lid_disc_open) if lid_disc_open is not None else None
    ctx.check(
        "lid rests swung open beside the hatch opening",
        lo is not None and lo[0] < -0.75 and lo[2] > 1.5
        and lid_disc_open[1][0] < -0.45,
        details=f"lid_disc={lid_disc_open}",
    )
    # Closed pose: the lid becomes concentric with the opening just above the seal.
    with ctx.pose({hinge: -LID_OPEN}):
        lid_disc_closed = ctx.part_element_world_aabb(lid, elem="lid_disc")
        seal_closed = ctx.part_element_world_aabb(lid, elem="lid_seal_ring")
    lc = _center(lid_disc_closed) if lid_disc_closed is not None else None
    ctx.check(
        "closed lid covers the opening concentrically",
        lc is not None and math.hypot(lc[0], lc[1]) < 0.05,
        details=f"closed_center={lc}",
    )
    ctx.check(
        "closed lid seal ring hovers just above the collar seal",
        seal_closed is not None and 0.0 < seal_closed[0][2] - SEAL_TOP < 0.02,
        details=f"lid_seal={seal_closed}, collar_seal_top={SEAL_TOP}",
    )

    # -------------------------------------------------- handwheel checks --
    ctx.check(
        "handwheel spins about the lid normal",
        spin.articulation_type == ArticulationType.REVOLUTE
        and spin.parent == "hatch_lid",
        details=f"parent={spin.parent}",
    )
    spoke = ctx.part_element_world_aabb(wheel, elem="wheel_spoke_0")
    with ctx.pose({spin: 1.0}):
        spoke_turned = ctx.part_element_world_aabb(wheel, elem="wheel_spoke_0")
    s0 = _center(spoke) if spoke is not None else None
    s1 = _center(spoke_turned) if spoke_turned is not None else None
    dims_changed = (
        spoke is not None
        and spoke_turned is not None
        and abs((spoke[1][0] - spoke[0][0]) - (spoke_turned[1][0] - spoke_turned[0][0]))
        > 0.05
    )
    ctx.check(
        "turning the handwheel sweeps the spokes",
        s0 is not None and s1 is not None and dims_changed,
        details=f"rest={spoke}, turned={spoke_turned}",
    )

    return ctx.report()


object_model = build_object_model()
