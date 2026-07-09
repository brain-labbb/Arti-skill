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

# ---------------------------------------------------------------------------
# World frame: +X toward the muzzle, +Y to the shooter's left, +Z up.
# The revolver stands on the flat of its grip butt at z = 0.
# ---------------------------------------------------------------------------

MESH_TOL = 0.0002
MESH_ANG_TOL = 0.2

# Barrel / bore
BORE_Z = 0.108            # barrel bore axis height
BARREL_R = 0.011
BARREL_X0 = -0.030        # forcing cone tucked into the frame window
MUZZLE_X = 0.126          # 6 in = 0.152 m measured from the frame front (-0.026)
FRAME_FRONT = -0.026

# Cylinder
CYL_Z = 0.095             # cylinder axis height (chamber offset 0.013 below bore)
CYL_R = 0.0205
CYL_X0, CYL_X1 = -0.0745, -0.0325
CYL_CX = 0.5 * (CYL_X0 + CYL_X1)
CYL_LEN = CYL_X1 - CYL_X0
CHAMBER_CIRCLE_R = 0.013
CHAMBER_R = 0.0052
CYL_BORE_R = 0.0056

# Frame
FRAME_HALF_W = 0.012
RAIL_Z0, RAIL_Z1 = 0.064, 0.070
STRAP_Z0, STRAP_Z1 = 0.118, 0.126
FRAME_REAR_BOT = -0.118

# Underlug / rib
LUG_Z0, LUG_Z1 = 0.083, 0.100
LUG_HALF_W = 0.010
LUG_X1 = 0.122
ROD_BORE_R = 0.0055

# Articulation frames
CRANE_PIVOT = (-0.0305, 0.010, 0.062)   # axis parallel to the barrel (X)
ROD_ORIGIN = (-0.0735, 0.0, CYL_Z)
TRIGGER_PIVOT = (-0.0630, 0.0, 0.067)
HAMMER_PIVOT = (-0.0960, 0.0, 0.088)

CRANE_OPEN = 0.785        # 45 deg swing-out
TRIGGER_PULL = 0.44       # ~25 deg
HAMMER_COCK = 0.52        # ~30 deg
ROD_TRAVEL = 0.020

GUARD_C = (-0.070, 0.047)
GUARD_OUTER_R = 0.021
GUARD_INNER_R = 0.0165


def _xcyl(x0: float, x1: float, r: float, y: float = 0.0, z: float = 0.0) -> cq.Workplane:
    """Solid cylinder whose long axis runs along world +X from x0 to x1."""
    return (
        cq.Workplane("YZ", origin=(x0, 0.0, 0.0))
        .center(y, z)
        .circle(r)
        .extrude(x1 - x0)
    )


def _bbox(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float) -> cq.Workplane:
    return cq.Workplane(
        "XY", origin=(0.5 * (x0 + x1), 0.5 * (y0 + y1), 0.5 * (z0 + z1))
    ).box(x1 - x0, y1 - y0, z1 - z0)


def _frame_body_solid() -> cq.Workplane:
    profile = [
        (FRAME_FRONT, RAIL_Z0),
        (FRAME_REAR_BOT, RAIL_Z0),
        (FRAME_REAR_BOT, 0.082),
        (-0.108, 0.100),
        (-0.105, STRAP_Z1),
        (FRAME_FRONT, STRAP_Z1),
    ]
    body = cq.Workplane("XZ").polyline(profile).close().extrude(FRAME_HALF_W, both=True)
    # Cylinder window (open at the front below the top strap; the crane fills it).
    body = body.cut(_bbox(-0.078, -0.024, -0.020, 0.020, RAIL_Z1, STRAP_Z0))
    # Notch in the bottom-rail front-left corner for the crane knuckle and lug.
    body = body.cut(_bbox(-0.032, -0.0255, 0.0045, 0.0140, 0.054, 0.0705))
    # Trigger slot through the bottom rail.
    body = body.cut(_bbox(-0.072, -0.056, -0.0035, 0.0035, 0.060, 0.0705))
    # Hammer slot through the standing breech and rear face.
    body = body.cut(_bbox(-0.115, -0.085, -0.0045, 0.0045, 0.076, 0.127))
    return body


def _barrel_solid() -> cq.Workplane:
    barrel = _xcyl(BARREL_X0, MUZZLE_X, BARREL_R, 0.0, BORE_Z)
    # Full-length underlug shrouding the ejector rod.
    barrel = barrel.union(_bbox(FRAME_FRONT, LUG_X1, -LUG_HALF_W, LUG_HALF_W, LUG_Z0, LUG_Z1))
    # Vented rib on top.
    barrel = barrel.union(_bbox(BARREL_X0, MUZZLE_X, -0.0045, 0.0045, 0.117, 0.1255))
    # Raised ramp-style front sight with tall blade.
    # Ramp base sitting on the rib top.
    ramp = _bbox(0.104, 0.124, -0.005, 0.005, 0.1255, 0.131)
    # Tall serrated blade rising above the ramp.
    front_blade = _bbox(0.113, 0.120, -0.003, 0.003, 0.131, 0.141)
    barrel = barrel.union(ramp).union(front_blade)
    # Elongated vent slots through the rib web.
    for k in range(4):
        cx = 0.012 + 0.028 * k
        barrel = barrel.cut(_bbox(cx - 0.010, cx + 0.010, -0.015, 0.015, 0.120, 0.1235))
    # Muzzle bore.
    barrel = barrel.cut(_xcyl(0.118, 0.127, 0.0045, 0.0, BORE_Z))
    # Ejector-rod channel through the underlug (and barrel-root underside).
    # Wide enough that the crane arbor and rod never touch the shroud.
    barrel = barrel.cut(_xcyl(-0.0315, 0.058, 0.0062, 0.0, CYL_Z))
    return barrel


def _guard_solid() -> cq.Workplane:
    guard = (
        cq.Workplane("XZ", origin=(GUARD_C[0], 0.0, GUARD_C[1]))
        .circle(GUARD_OUTER_R)
        .circle(GUARD_INNER_R)
        .extrude(0.005, both=True)
    )
    # Clear the trigger slot through the guard root (deeper than the frame
    # slot so the blade never grazes the ring web).
    guard = guard.cut(_bbox(-0.072, -0.056, -0.0035, 0.0035, 0.056, 0.0705))
    return guard


def _rear_sight_solid() -> cq.Workplane:
    """Tall fully adjustable rear sight assembly with elevated notch blade,
    windage knob, and elevation screw — replacing the low fixed sight."""
    # Dovetail base seated on the top strap (z ≈ 0.126).
    base = _bbox(-0.107, -0.085, -0.0075, 0.0075, 0.1255, 0.1295)
    # Windage slider bridge spanning the base (overlaps base in z for connectivity).
    bridge = _bbox(-0.104, -0.088, -0.006, 0.006, 0.1285, 0.133)
    # Tall blade housing rising above the bridge.
    housing = _bbox(-0.102, -0.090, -0.0045, 0.0045, 0.133, 0.141)
    sight = base.union(bridge).union(housing)
    # Windage adjustment knob protruding from the right side of the bridge.
    # Stem starts well inside the bridge (y=0) for guaranteed mesh connectivity.
    windage_stem = (
        cq.Workplane("XZ", origin=(-0.096, 0.0, 0.1315))
        .circle(0.0025)
        .extrude(0.012)
    )
    # Knob head (wider disc at the tip).
    windage_head = (
        cq.Workplane("XZ", origin=(-0.096, 0.009, 0.1315))
        .circle(0.004)
        .extrude(0.003)
    )
    sight = sight.union(windage_stem).union(windage_head)
    # Elevation screw head on top of the housing (overlaps housing top).
    elev_screw = (
        cq.Workplane("XY", origin=(-0.096, 0.0, 0.1405))
        .circle(0.0025)
        .extrude(0.003)
    )
    sight = sight.union(elev_screw)
    # Rear sight notch cut through the blade.
    sight = sight.cut(_bbox(-0.104, -0.088, -0.0013, 0.0013, 0.137, 0.145))
    return sight


def _crane_solid() -> cq.Workplane:
    px, py, pz = CRANE_PIVOT
    knuckle = _xcyl(-0.0305, -0.014, 0.005, py, pz)
    lug = _bbox(-0.0305, -0.0265, 0.0055, 0.0135, 0.060, 0.075)
    # Block seats on the bottom-rail top (z=0.070) with a 0.5 mm embed so the
    # closed crane is positively supported by the frame.
    block = _bbox(-0.0305, -0.0265, -0.0115, 0.0115, 0.0695, 0.0965)
    # Arbor runs a light press fit inside the cylinder bore (r 0.0056).
    arbor = _xcyl(-0.058, -0.0295, 0.0059, 0.0, CYL_Z)
    crane = knuckle.union(lug).union(block).union(arbor)
    # Bore for the ejector rod through block and arbor.
    crane = crane.cut(_xcyl(-0.060, -0.025, 0.0040, 0.0, CYL_Z))
    return crane.translate((-px, -py, -pz))


def _cylinder_solid() -> cq.Workplane:
    half = CYL_LEN / 2.0
    cyl = cq.Workplane("YZ", origin=(-half, 0.0, 0.0)).circle(CYL_R).extrude(CYL_LEN)
    # Center bore for the crane arbor / ejector rod.
    cyl = cyl.cut(_xcyl(-half - 0.001, half + 0.001, CYL_BORE_R))
    # Six chambers; the top chamber aligns with the barrel bore at q = 0.
    for k in range(6):
        a = math.radians(90.0 + 60.0 * k)
        cy = CHAMBER_CIRCLE_R * math.cos(a)
        cz = CHAMBER_CIRCLE_R * math.sin(a)
        cyl = cyl.cut(_xcyl(-half - 0.001, half + 0.001, CHAMBER_R, cy, cz))
    # Rear recess for the ejector star.
    cyl = cyl.cut(_xcyl(-half - 0.001, -half + 0.0025, 0.0065))
    # Six flutes between the chambers.
    for k in range(6):
        a = math.radians(120.0 + 60.0 * k)
        fy = 0.0215 * math.cos(a)
        fz = 0.0215 * math.sin(a)
        cyl = cyl.cut(_xcyl(-0.016, 0.013, 0.005, fy, fz))
    return cyl


def _rod_solid() -> cq.Workplane:
    rx, ry, rz = ROD_ORIGIN
    # Shaft runs a light press fit inside the crane arbor bore (r 0.0040).
    shaft = _xcyl(-0.0735, 0.052, 0.0043, 0.0, CYL_Z)
    star = (
        cq.Workplane("YZ", origin=(-0.0744, 0.0, 0.0))
        .center(0.0, CYL_Z)
        .polygon(6, 0.0116)
        .extrude(0.0020)
    )
    rod = shaft.union(star)
    return rod.translate((-rx, -ry, -rz))


def _trigger_solid() -> cq.Workplane:
    pts = [
        (-0.0598, 0.0690),
        (-0.0592, 0.0600),
        (-0.0610, 0.0500),
        (-0.0652, 0.0420),
        (-0.0688, 0.0395),
        (-0.0703, 0.0420),
        (-0.0676, 0.0512),
        (-0.0658, 0.0610),
        (-0.0663, 0.0690),
    ]
    blade = cq.Workplane("XZ").polyline(pts).close().extrude(0.003, both=True)
    px, py, pz = TRIGGER_PIVOT
    pin_hole = (
        cq.Workplane("XZ", origin=(px, 0.0, pz)).circle(0.0018).extrude(0.01, both=True)
    )
    blade = blade.cut(pin_hole)
    return blade.translate((-px, -py, -pz))


def _hammer_solid() -> cq.Workplane:
    pts = [
        (-0.0865, 0.0840),
        (-0.0865, 0.0935),
        (-0.0960, 0.1000),
        (-0.1010, 0.1060),
        (-0.1040, 0.1160),
        (-0.1100, 0.1205),
        (-0.1210, 0.1240),
        (-0.1225, 0.1195),
        (-0.1130, 0.1135),
        (-0.1065, 0.1040),
        (-0.1045, 0.0950),
        (-0.1040, 0.0840),
        (-0.0960, 0.0790),
    ]
    body = cq.Workplane("XZ").polyline(pts).close().extrude(0.004, both=True)
    # Checkered thumb pad on the spur, slightly wider than the blade.
    pad = _bbox(-0.1225, -0.110, -0.0055, 0.0055, 0.1175, 0.1245)
    body = body.union(pad)
    px, py, pz = HAMMER_PIVOT
    pin_hole = (
        cq.Workplane("XZ", origin=(px, 0.0, pz)).circle(0.0026).extrude(0.012, both=True)
    )
    body = body.cut(pin_hole)
    return body.translate((-px, -py, -pz))


def _grip_solid() -> cq.Workplane:
    pts = [
        (-0.096, 0.0645),
        (-0.103, 0.0420),
        (-0.111, 0.0180),
        (-0.118, 0.0050),
        (-0.124, 0.0000),
        (-0.150, 0.0000),
        (-0.156, 0.0060),
        (-0.158, 0.0160),
        (-0.1545, 0.0300),
        (-0.146, 0.0440),
        (-0.135, 0.0560),
        (-0.124, 0.0625),
        (-0.118, 0.0645),
    ]
    grip = cq.Workplane("XZ").polyline(pts).close().extrude(0.017, both=True)
    try:
        grip = grip.edges(">Y or <Y").fillet(0.0035)
    except Exception:
        pass
    return grip


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="colt_python_double_action_revolver")

    stainless = model.material("stainless_steel", rgba=(0.78, 0.79, 0.81, 1.0))
    stainless_mid = model.material("stainless_mid", rgba=(0.71, 0.72, 0.74, 1.0))
    stainless_bright = model.material("stainless_bright", rgba=(0.83, 0.84, 0.86, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.16, 0.16, 0.18, 1.0))
    bore_black = model.material("bore_black", rgba=(0.08, 0.08, 0.09, 1.0))
    walnut = model.material("walnut", rgba=(0.45, 0.28, 0.15, 1.0))
    walnut_dark = model.material("walnut_dark", rgba=(0.28, 0.16, 0.09, 1.0))

    # ---------------------------------------------------------------- frame
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(
            _frame_body_solid(), "frame_body", tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL
        ),
        material=stainless,
        name="frame_body",
    )
    frame.visual(
        mesh_from_cadquery(
            _barrel_solid(), "barrel_assembly", tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL
        ),
        material=stainless,
        name="barrel_assembly",
    )
    frame.visual(
        mesh_from_cadquery(
            _guard_solid(), "trigger_guard", tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL
        ),
        material=stainless,
        name="trigger_guard",
    )
    frame.visual(
        mesh_from_cadquery(
            _rear_sight_solid(), "rear_sight", tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL
        ),
        material=dark_steel,
        name="rear_sight",
    )
    # Colt-style cylinder release latch on the left of the recoil shield.
    frame.visual(
        Box((0.018, 0.0025, 0.010)),
        origin=Origin(xyz=(-0.091, 0.01325, 0.101)),
        material=stainless_mid,
        name="cylinder_latch",
    )
    # Visible trigger and hammer pivot pins spanning the frame slots.
    frame.visual(
        Cylinder(radius=0.0022, length=0.0080),
        origin=Origin(xyz=(TRIGGER_PIVOT[0], 0.0, TRIGGER_PIVOT[2]), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless_mid,
        name="trigger_pin",
    )
    frame.visual(
        Cylinder(radius=0.0030, length=0.0100),
        origin=Origin(xyz=(HAMMER_PIVOT[0], 0.0, HAMMER_PIVOT[2]), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless_mid,
        name="hammer_pin",
    )

    # ---------------------------------------------------------------- crane
    crane = model.part("crane")
    crane.visual(
        mesh_from_cadquery(
            _crane_solid(), "crane_body", tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL
        ),
        material=stainless,
        name="crane_body",
    )
    model.articulation(
        "crane_swing",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=crane,
        origin=Origin(xyz=CRANE_PIVOT),
        # Pivot axis runs parallel to the barrel; -X makes +q swing the
        # cylinder out to the shooter's left.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=CRANE_OPEN),
    )

    # ------------------------------------------------------------- cylinder
    cylinder = model.part("cylinder")
    cylinder.visual(
        mesh_from_cadquery(
            _cylinder_solid(), "cylinder_body", tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL
        ),
        material=stainless_mid,
        name="cylinder_body",
    )
    # Dark chamber liners (slightly embedded into the chamber walls); the top
    # liner is the off-axis witness for the continuous spin.
    for k in range(6):
        a = math.radians(90.0 + 60.0 * k)
        cy = CHAMBER_CIRCLE_R * math.cos(a)
        cz = CHAMBER_CIRCLE_R * math.sin(a)
        cylinder.visual(
            Cylinder(radius=0.0055, length=0.0405),
            origin=Origin(xyz=(0.0, cy, cz), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bore_black,
            name=f"chamber_liner_{k}",
        )
    model.articulation(
        "cylinder_spin",
        ArticulationType.CONTINUOUS,
        parent=crane,
        child=cylinder,
        origin=Origin(
            xyz=(
                CYL_CX - CRANE_PIVOT[0],
                0.0 - CRANE_PIVOT[1],
                CYL_Z - CRANE_PIVOT[2],
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0),
    )

    # ----------------------------------------------------------- ejector rod
    rod = model.part("ejector_rod")
    rod.visual(
        mesh_from_cadquery(
            _rod_solid(), "ejector_rod_body", tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL
        ),
        material=stainless_bright,
        name="ejector_rod_body",
    )
    model.articulation(
        "ejector_push",
        ArticulationType.PRISMATIC,
        parent=crane,
        child=rod,
        origin=Origin(
            xyz=(
                ROD_ORIGIN[0] - CRANE_PIVOT[0],
                ROD_ORIGIN[1] - CRANE_PIVOT[1],
                ROD_ORIGIN[2] - CRANE_PIVOT[2],
            )
        ),
        # +q pushes the rod (and ejector star) rearward along the cylinder axis.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.5, lower=0.0, upper=ROD_TRAVEL),
    )

    # --------------------------------------------------------------- trigger
    trigger = model.part("trigger")
    trigger.visual(
        mesh_from_cadquery(
            _trigger_solid(), "trigger_blade", tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL
        ),
        material=stainless_bright,
        name="trigger_blade",
    )
    model.articulation(
        "trigger_pull",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=trigger,
        origin=Origin(xyz=TRIGGER_PIVOT),
        # Left-right pin; +q about +Y swings the blade tip rearward.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=0.0, upper=TRIGGER_PULL),
    )

    # ---------------------------------------------------------------- hammer
    hammer = model.part("hammer")
    hammer.visual(
        mesh_from_cadquery(
            _hammer_solid(), "hammer_body", tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL
        ),
        material=stainless,
        name="hammer_body",
    )
    model.articulation(
        "hammer_cock",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=hammer,
        origin=Origin(xyz=HAMMER_PIVOT),
        # Left-right pin; -Y makes +q rotate the spur rearward/down (cocking).
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=0.0, upper=HAMMER_COCK),
    )

    # ------------------------------------------------------------------ grip
    grip = model.part("grip")
    grip.visual(
        mesh_from_cadquery(
            _grip_solid(), "grip_body", tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL
        ),
        material=walnut,
        name="grip_body",
    )
    for side, sy in (("left", 1.0), ("right", -1.0)):
        grip.visual(
            Box((0.024, 0.0008, 0.044)),
            origin=Origin(xyz=(-0.131, sy * 0.0172, 0.0315), rpy=(0.0, 0.50, 0.0)),
            material=walnut_dark,
            name=f"{side}_grip_panel",
        )
        grip.visual(
            Cylinder(radius=0.0025, length=0.0012),
            origin=Origin(xyz=(-0.131, sy * 0.01765, 0.0315), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"{side}_grip_screw",
        )
    model.articulation(
        "grip_mount",
        ArticulationType.FIXED,
        parent=frame,
        child=grip,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    crane = object_model.get_part("crane")
    cylinder = object_model.get_part("cylinder")
    rod = object_model.get_part("ejector_rod")
    trigger = object_model.get_part("trigger")
    hammer = object_model.get_part("hammer")
    grip = object_model.get_part("grip")

    crane_swing = object_model.get_articulation("crane_swing")
    cylinder_spin = object_model.get_articulation("cylinder_spin")
    ejector_push = object_model.get_articulation("ejector_push")
    trigger_pull = object_model.get_articulation("trigger_pull")
    hammer_cock = object_model.get_articulation("hammer_cock")

    # ----------------------------------------------------- joint plan checks
    ctx.check(
        "crane is revolute with ~45 deg swing",
        crane_swing.articulation_type == ArticulationType.REVOLUTE
        and crane_swing.motion_limits is not None
        and abs(crane_swing.motion_limits.lower) < 1e-9
        and 0.52 <= crane_swing.motion_limits.upper <= 0.80
        and crane_swing.axis == (-1.0, 0.0, 0.0),
    )
    ctx.check(
        "cylinder spin is continuous about the longitudinal axis",
        cylinder_spin.articulation_type == ArticulationType.CONTINUOUS
        and cylinder_spin.motion_limits is not None
        and cylinder_spin.motion_limits.lower is None
        and cylinder_spin.motion_limits.upper is None
        and cylinder_spin.axis == (1.0, 0.0, 0.0),
    )
    ctx.check(
        "ejector rod is prismatic with 0.02 m rearward travel",
        ejector_push.articulation_type == ArticulationType.PRISMATIC
        and ejector_push.motion_limits is not None
        and abs(ejector_push.motion_limits.upper - 0.020) < 1e-9
        and ejector_push.axis == (-1.0, 0.0, 0.0),
    )
    ctx.check(
        "trigger pivots ~25 deg on a left-right pin",
        trigger_pull.articulation_type == ArticulationType.REVOLUTE
        and trigger_pull.motion_limits is not None
        and 0.40 <= trigger_pull.motion_limits.upper <= 0.48
        and trigger_pull.axis == (0.0, 1.0, 0.0),
    )
    ctx.check(
        "hammer cocks ~30 deg on a left-right pin",
        hammer_cock.articulation_type == ArticulationType.REVOLUTE
        and hammer_cock.motion_limits is not None
        and 0.48 <= hammer_cock.motion_limits.upper <= 0.56
        and hammer_cock.axis == (0.0, -1.0, 0.0),
    )

    # ------------------------------------------------------ scale / grounding
    parts = [frame, crane, cylinder, rod, trigger, hammer, grip]
    lo = [math.inf] * 3
    hi = [-math.inf] * 3
    for p in parts:
        aabb = ctx.part_world_aabb(p)
        if aabb is None:
            continue
        for i in range(3):
            lo[i] = min(lo[i], aabb[0][i])
            hi[i] = max(hi[i], aabb[1][i])
    length = hi[0] - lo[0]
    width = hi[1] - lo[1]
    height = hi[2] - lo[2]
    ctx.check(
        "overall length ~0.29 m", 0.27 <= length <= 0.30, details=f"length={length:.4f}"
    )
    ctx.check("overall width ~0.04 m", 0.034 <= width <= 0.046, details=f"width={width:.4f}")
    ctx.check(
        "overall height ~0.13-0.14 m", 0.125 <= height <= 0.145, details=f"height={height:.4f}"
    )
    ctx.check("grip butt grounded at z=0", abs(lo[2]) <= 0.002, details=f"zmin={lo[2]:.4f}")

    # 6-inch barrel: barrel assembly reaches the muzzle 0.152 m ahead of the frame front.
    barrel_aabb = ctx.part_element_world_aabb(frame, elem="barrel_assembly")
    ctx.check(
        "6-inch barrel reaches x~0.126",
        barrel_aabb is not None and abs(barrel_aabb[1][0] - MUZZLE_X) < 0.004,
        details=f"barrel_aabb={barrel_aabb}",
    )

    # ------------------------------------------------- adjustable sight checks
    rear_sight_aabb = ctx.part_element_world_aabb(frame, elem="rear_sight")
    ctx.check(
        "rear sight is tall adjustable assembly (blade tip above z=0.137)",
        rear_sight_aabb is not None and rear_sight_aabb[1][2] > 0.137,
        details=f"rear_sight_aabb={rear_sight_aabb}",
    )
    ctx.check(
        "rear sight base seated on top strap (z_min near 0.126)",
        rear_sight_aabb is not None and abs(rear_sight_aabb[0][2] - 0.1255) < 0.002,
        details=f"rear_sight_zmin={rear_sight_aabb[0][2] if rear_sight_aabb else None}",
    )
    barrel_assembly_aabb = ctx.part_element_world_aabb(frame, elem="barrel_assembly")
    ctx.check(
        "front sight is raised ramp+blade (tip above z=0.138)",
        barrel_assembly_aabb is not None and barrel_assembly_aabb[1][2] > 0.138,
        details=f"barrel_assembly_aabb={barrel_assembly_aabb}",
    )
    # Sight line: both sights should reach similar heights for proper alignment.
    if rear_sight_aabb is not None and barrel_assembly_aabb is not None:
        rear_tip = rear_sight_aabb[1][2]
        front_tip = barrel_assembly_aabb[1][2]
        ctx.check(
            "front and rear sight tips at similar height for sight alignment",
            abs(rear_tip - front_tip) < 0.006,
            details=f"rear_tip={rear_tip:.4f}, front_tip={front_tip:.4f}",
        )

    # ----------------------------------------------------- rest-pose seating
    # Intentional press fits: the crane arbor is captured inside the cylinder
    # center bore, and the ejector rod is captured inside the crane arbor bore.
    ctx.allow_overlap(
        crane,
        cylinder,
        elem_a="crane_body",
        elem_b="cylinder_body",
        reason=(
            "The crane arbor is a light press fit inside the cylinder center "
            "bore so the swing-out cylinder stays retained on the crane."
        ),
    )
    ctx.allow_overlap(
        crane,
        rod,
        elem_a="crane_body",
        elem_b="ejector_rod_body",
        reason=(
            "The ejector rod shaft is a light press fit inside the crane arbor "
            "bore so the rod stays captured while it slides."
        ),
    )
    ctx.allow_overlap(
        crane,
        frame,
        elem_a="crane_body",
        elem_b="frame_body",
        reason=(
            "The closed crane block intentionally seats 0.5 mm into the top of "
            "the frame bottom rail so the latched crane is positively supported."
        ),
    )
    ctx.expect_within(rod, crane, axes="y", margin=0.0, name="ejector rod centered in the crane arbor bore")
    ctx.expect_overlap(
        rod, crane, axes="x", min_overlap=0.025, name="ejector rod captured inside the crane arbor"
    )
    ctx.expect_contact(
        crane, frame, contact_tol=0.0008, name="closed crane block seats on the frame bottom rail"
    )
    ctx.expect_within(cylinder, frame, axes="x", margin=0.0, name="cylinder seated in frame window")
    ctx.expect_within(rod, cylinder, axes="yz", margin=0.0, name="ejector rod runs on the cylinder axis")
    ctx.expect_overlap(
        rod, cylinder, axes="x", min_overlap=0.030, name="ejector rod threaded through the cylinder"
    )
    ctx.expect_overlap(
        crane, cylinder, axes="x", min_overlap=0.015, name="crane arbor retained inside the cylinder"
    )
    ctx.expect_contact(grip, frame, contact_tol=0.0008, name="walnut grip seats against the frame")
    ctx.expect_within(trigger, frame, axes="xz", margin=0.0, name="trigger inside the guard opening")

    # Six chambers present.
    liners = [v for v in cylinder.visuals if v.name and v.name.startswith("chamber_liner_")]
    ctx.check("six-shot cylinder has 6 chambers", len(liners) == 6, details=f"n={len(liners)}")

    # ------------------------------------------------- crane swing-out pose
    rest_cyl = ctx.part_world_position(cylinder)
    with ctx.pose({crane_swing: CRANE_OPEN}):
        open_cyl = ctx.part_world_position(cylinder)
        ctx.check(
            "open crane swings the cylinder out to the left",
            rest_cyl is not None
            and open_cyl is not None
            and open_cyl[1] > rest_cyl[1] + 0.020,
            details=f"rest={rest_cyl}, open={open_cyl}",
        )
    rest_rod = ctx.part_world_position(rod)
    with ctx.pose({crane_swing: CRANE_OPEN, ejector_push: ROD_TRAVEL}):
        pushed_rod = ctx.part_world_position(rod)
        ctx.check(
            "ejector rod pushes 0.02 m rearward along the cylinder axis",
            rest_rod is not None
            and pushed_rod is not None
            and abs((rest_rod[0] - pushed_rod[0]) - ROD_TRAVEL) < 0.001
            and pushed_rod[1] > 0.015,
            details=f"rest={rest_rod}, pushed={pushed_rod}",
        )
        ctx.expect_overlap(
            rod,
            cylinder,
            axes="x",
            min_overlap=0.020,
            name="pushed ejector rod stays threaded through the cylinder",
        )

    # ----------------------------------------------- hammer / trigger poses
    rest_hammer = ctx.part_world_aabb(hammer)
    with ctx.pose({hammer_cock: HAMMER_COCK}):
        cocked = ctx.part_world_aabb(hammer)
        ctx.check(
            "cocking rotates the hammer spur rearward",
            rest_hammer is not None
            and cocked is not None
            and cocked[0][0] < rest_hammer[0][0] - 0.002
            and cocked[0][2] < rest_hammer[0][2] - 0.002,
            details=f"rest={rest_hammer}, cocked={cocked}",
        )
    rest_trigger = ctx.part_world_aabb(trigger)
    with ctx.pose({trigger_pull: TRIGGER_PULL}):
        pulled = ctx.part_world_aabb(trigger)
        ctx.check(
            "pulling swings the trigger tip rearward",
            rest_trigger is not None
            and pulled is not None
            and pulled[0][0] < rest_trigger[0][0] - 0.006,
            details=f"rest={rest_trigger}, pulled={pulled}",
        )

    # ------------------------------- off-axis proof of the continuous spin
    liner_rest = ctx.part_element_world_aabb(cylinder, elem="chamber_liner_0")
    with ctx.pose({cylinder_spin: math.pi}):
        liner_spun = ctx.part_element_world_aabb(cylinder, elem="chamber_liner_0")
        rest_z = 0.5 * (liner_rest[0][2] + liner_rest[1][2]) if liner_rest else None
        spun_z = 0.5 * (liner_spun[0][2] + liner_spun[1][2]) if liner_spun else None
        ctx.check(
            "half-turn spin carries the top chamber to the bottom",
            rest_z is not None and spun_z is not None and (rest_z - spun_z) > 0.020,
            details=f"rest_z={rest_z}, spun_z={spun_z}",
        )

    return ctx.report()


object_model = build_object_model()
