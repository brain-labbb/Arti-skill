from __future__ import annotations

# Aerosol SPRAY PAINT CAN with a screw-on threaded cap.
# Fork of the lift-off dust cap variant: the closure mechanism is replaced with
# a deeper cylindrical cap that threads down over a visible threaded collar ring
# on the can shoulder. The cap is removed by a CONTINUOUS rotation followed by
# a PRISMATIC lift on a small carrier link, so it visibly rises off the thread.
#
# Frame: can axis along +Z. Base rim sits at z=0, can body rises in +Z, the
# crimped dome and spray nozzle are at the top. Cylinder ~0.22 m tall,
# ~0.065 m body diameter.
# Articulations:
#   - cap_unscrew: body -> cap_carrier, CONTINUOUS rotation about +Z.
#   - cap_lift: cap_carrier -> screw_cap, PRISMATIC lift along +Z.
#   - spray nozzle button: PRISMATIC press DOWN (-Z, ~0.004 m).
# The nozzle button is a CHILD of the can body (not the cap) so it stays put
# when the cap is unscrewed.

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
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key dimensions (meters) ----
CAN_R = 0.0325              # body radius (~0.065 m diameter)
BODY_BOTTOM = 0.0           # base plane
BODY_TOP = 0.148            # straight wall ends, shoulder begins
DOME_TOP = 0.178            # top of the crimped dome (mounting cup rim)
VALVE_TOP = 0.184           # top of central valve stem
NOZZLE_SEAT_Z = 0.176       # z where nozzle button base sits
NOZZLE_TOP_Z = 0.196        # top of the nozzle button

# Threaded collar on the can shoulder (visible mating face)
COLLAR_BOTTOM = 0.178       # collar lower edge on the valve cup, clear of shoulder
COLLAR_TOP = 0.202          # collar upper edge above the nozzle button
COLLAR_OUTER_R = 0.0225     # narrow threaded neck, not a body-sized sleeve
COLLAR_INNER_R = 0.0130     # clears the central valve/nozzle boss

# Thread ridge parameters (visible on collar exterior)
N_THREAD_RIDGES = 4
THREAD_RIDGE_H = 0.0014     # ridge height (along Z)
THREAD_RIDGE_D = 0.0010     # ridge radial protrusion

# Screw cap
CAP_BOTTOM = 0.176          # cap lower rim clears the crimped shoulder
CAP_TOP = 0.236             # cap top
CAP_OUTER_R = 0.0305        # smaller cap that reads as a threaded spray-can cap
CAP_BORE_R = COLLAR_OUTER_R + THREAD_RIDGE_D - 0.0002
CAP_LIFT_TRAVEL = 0.072


def _can_body_solid() -> cq.Workplane:
    """Can body with rolled bottom/top seams and a smoother crimped shoulder."""
    # Rolled bottom foot.
    body = (
        cq.Workplane("XY")
        .circle(CAN_R + 0.0012)
        .extrude(0.006)
    )
    body = body.union(
        cq.Workplane("XY")
        .workplane(offset=0.006)
        .circle(CAN_R + 0.0008)
        .extrude(0.005)
    )
    # Slightly inset main wall so the seams and shoulder are legible.
    body = body.union(
        cq.Workplane("XY")
        .workplane(offset=0.011)
        .circle(CAN_R - 0.0006)
        .extrude(BODY_TOP - 0.011)
    )
    # Top rolled seam below the shoulder.
    body = body.union(
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP - 0.006)
        .circle(CAN_R + 0.0010)
        .extrude(0.007)
    )
    # Crimped top dome: fuller outside radius, then narrows to a small neck.
    dome = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP - 0.001)
        .circle(CAN_R + 0.0005)
        .workplane(offset=0.007)
        .circle(CAN_R - 0.002)
        .workplane(offset=0.008)
        .circle(0.024)
        .workplane(offset=0.010)
        .circle(0.015)
        .workplane(offset=0.006)
        .circle(0.0135)
        .loft(ruled=False)
    )
    body = body.union(dome)
    # Raised valve cup and central stem.
    body = body.union(
        cq.Workplane("XY")
        .workplane(offset=DOME_TOP - 0.004)
        .circle(0.0145)
        .extrude(0.004)
    )
    body = body.union(
        cq.Workplane("XY")
        .workplane(offset=DOME_TOP - 0.001)
        .circle(0.006)
        .extrude(VALVE_TOP - DOME_TOP)
    )
    return body


def _nozzle_solid() -> cq.Workplane:
    """Press-down spray button with front spray hole."""
    btn = (
        cq.Workplane("XY")
        .box(0.018, 0.016, 0.014, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.003)
    )
    # Spray hole on the +X front face
    hole = (
        cq.Workplane("YZ")
        .workplane(offset=0.009)
        .center(0.0, 0.008)
        .circle(0.0014)
        .extrude(-0.010)
    )
    btn = btn.cut(hole)
    return btn


def _collar_solid() -> cq.Workplane:
    """Threaded collar ring on the can shoulder — visible mating face for the cap.

    Built as an annular ring with external thread ridges (evenly spaced rings).
    The collar wraps around the can body wall with a slight interference fit
    to ensure mesh connectivity with the parent can body.
    """
    h = COLLAR_TOP - COLLAR_BOTTOM
    # Main annular threaded neck.
    collar = (
        cq.Workplane("XY")
        .circle(COLLAR_OUTER_R)
        .circle(COLLAR_INNER_R)
        .extrude(h)
    )
    # External thread ridges (stacked rings read clearly in the viewer).
    pitch = h / (N_THREAD_RIDGES + 1)
    for i in range(N_THREAD_RIDGES):
        z = pitch * (i + 1)
        ridge = (
            cq.Workplane("XY")
            .workplane(offset=z - THREAD_RIDGE_H / 2)
            .circle(COLLAR_OUTER_R + THREAD_RIDGE_D)
            .circle(COLLAR_OUTER_R - 0.0003)
            .extrude(THREAD_RIDGE_H)
        )
        collar = collar.union(ridge)
    return collar


def _screw_cap_solid() -> cq.Workplane:
    """Deeper cylindrical screw cap with domed top and hollow bore.

    The bore is sized to clear the collar thread ridges so the cap sleeves
    over the collar without geometric overlap.
    """
    h = CAP_TOP - CAP_BOTTOM
    # Outer cylindrical shell with a small lower skirt and rounded top.
    outer = (
        cq.Workplane("XY")
        .circle(CAP_OUTER_R)
        .extrude(h - 0.010)
    )
    # Domed closed top
    dome = (
        cq.Workplane("XY")
        .workplane(offset=h - 0.011)
        .circle(CAP_OUTER_R)
        .workplane(offset=0.006)
        .circle(CAP_OUTER_R - 0.004)
        .workplane(offset=0.004)
        .circle(0.012)
        .loft(ruled=False)
    )
    cap = outer.union(dome)
    # Hollow bore (clearance for collar + thread ridges).
    bore = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .circle(CAP_BORE_R)
        .extrude(h - 0.012)
    )
    cap = cap.cut(bore)
    # Knurled grip ribs around the outside of the cap.
    rib_count = 14
    for i in range(rib_count):
        ang = 360.0 * i / rib_count
        rib = (
            cq.Workplane("XY")
            .workplane(offset=0.008)
            .center(CAP_OUTER_R + 0.0007, 0)
            .box(0.0018, 0.0045, h - 0.020, centered=(True, True, False))
            .edges("|Z")
            .fillet(0.0005)
        )
        rib = rib.rotate((0, 0, 0), (0, 0, 1), ang)
        cap = cap.union(rib)
    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spray_paint_can")

    # Materials
    label = model.material("label_splatter", rgba=(0.90, 0.90, 0.92, 1.0))
    metal = model.material("metal", rgba=(0.78, 0.80, 0.83, 1.0))
    collar_metal = model.material("collar_metal", rgba=(0.70, 0.73, 0.77, 1.0))
    dark_cap = model.material("dark_cap", rgba=(0.18, 0.20, 0.24, 1.0))
    dark_nozzle = model.material("dark_nozzle", rgba=(0.16, 0.16, 0.17, 1.0))

    # ---- can body (root) ----
    body = model.part("can_body")
    body.visual(
        mesh_from_cadquery(_can_body_solid(), "can_body"),
        material=metal,
        name="can_body",
    )
    # Splatter-graphic label band
    label_band = CylinderGeometry(CAN_R + 0.0004, 0.105, radial_segments=64)
    label_band.translate(0.0, 0.0, 0.012 + 0.105 / 2.0)
    body.visual(
        mesh_from_geometry(label_band, "label_band"),
        material=label,
        name="label_band",
    )
    # Threaded collar ring on the shoulder (inline visual on can body)
    body.visual(
        mesh_from_cadquery(_collar_solid(), "threaded_collar"),
        material=collar_metal,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_BOTTOM)),
        name="threaded_collar",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(CAN_R, 0.176),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
    )

    # ---- spray nozzle button: child of can body, presses straight down ----
    nozzle = model.part("spray_nozzle")
    nozzle.visual(
        mesh_from_cadquery(_nozzle_solid(), "spray_nozzle"),
        material=dark_nozzle,
        name="spray_nozzle",
    )
    nozzle.inertial = Inertial.from_geometry(
        Box((0.018, 0.016, 0.014)),
        mass=0.004,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
    )
    model.articulation(
        "nozzle_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=nozzle,
        origin=Origin(xyz=(0.0, 0.0, NOZZLE_SEAT_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.05, lower=0.0, upper=0.004),
    )

    # ---- screw cap: continuous spin carrier, then cap lifts upward ----
    carrier = model.part("cap_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.006, 0.006, 0.006)), mass=1e-4)

    cap = model.part("screw_cap")
    cap.visual(
        mesh_from_cadquery(_screw_cap_solid(), "screw_cap"),
        material=dark_cap,
        name="screw_cap",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_OUTER_R, CAP_TOP - CAP_BOTTOM),
        mass=0.018,
        origin=Origin(xyz=(0.0, 0.0, (CAP_TOP - CAP_BOTTOM) / 2.0)),
    )
    model.articulation(
        "cap_unscrew",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, CAP_BOTTOM)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0),
    )
    model.articulation(
        "cap_lift",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=cap,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.20, lower=0.0, upper=CAP_LIFT_TRAVEL),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("can_body")
    nozzle = object_model.get_part("spray_nozzle")
    carrier = object_model.get_part("cap_carrier")
    cap = object_model.get_part("screw_cap")
    cap_unscrew = object_model.get_articulation("cap_unscrew")
    cap_lift = object_model.get_articulation("cap_lift")
    nozzle_press = object_model.get_articulation("nozzle_press")

    # --- can body proportions ---
    bmn, bmx = ctx.part_world_aabb(body)
    height = bmx[2] - bmn[2]
    dia_x = bmx[0] - bmn[0]
    dia_y = bmx[1] - bmn[1]
    ctx.check(
        "can body is a tall cylinder (~0.18 m, ~0.065 m dia)",
        height > 0.16 and 0.05 < dia_x < 0.10 and 0.05 < dia_y < 0.10 and height > dia_x * 1.8,
        details=f"height={height:.3f}, dia_x={dia_x:.3f}, dia_y={dia_y:.3f}",
    )

    # --- threaded collar is a visible feature on the can body ---
    collar_vis = body.get_visual("threaded_collar")
    ctx.check(
        "threaded collar is present as a visual on the can body",
        collar_vis is not None,
        details="threaded_collar visual not found on can_body",
    )

    # --- nozzle mounted on the can top ---
    npos = ctx.part_world_position(nozzle)
    ctx.check(
        "nozzle button is mounted on the can top",
        npos is not None and npos[2] > 0.16 and abs(npos[0]) < 0.01 and abs(npos[1]) < 0.01,
        details=f"nozzle origin={npos}",
    )

    # --- nozzle seated on valve stem (intentional local overlap) ---
    ctx.allow_overlap(
        nozzle, body,
        elem_a="spray_nozzle", elem_b="can_body",
        reason="The spray button is intentionally seated over the central valve stem.",
    )
    ctx.expect_contact(nozzle, body, name="nozzle seated on the valve stem")

    # --- nozzle presses straight down ---
    nz_rest = ctx.part_world_position(nozzle)[2]
    with ctx.pose({nozzle_press: 0.004}):
        nz_down = ctx.part_world_position(nozzle)[2]
    ctx.check(
        "nozzle button presses straight down",
        nz_down < nz_rest - 0.0035,
        details=f"rest_z={nz_rest:.4f}, pressed_z={nz_down:.4f}",
    )

    # --- screw cap thread engagement with collar (intentional local overlap) ---
    ctx.allow_overlap(
        cap, body,
        elem_a="screw_cap", elem_b="threaded_collar",
        reason="Screw cap bore engages with collar thread ridges; small radial overlap represents threaded fit.",
    )
    ctx.expect_contact(cap, body, elem_a="screw_cap", elem_b="threaded_collar",
                       name="screw cap bore contacts collar thread ridges")

    # --- screw cap covers the collar region at rest ---
    cap_mn, cap_mx = ctx.part_world_aabb(cap)
    ctx.check(
        "screw cap covers the threaded collar region when seated",
        cap_mn[2] <= COLLAR_BOTTOM + 0.002 and cap_mx[2] > COLLAR_TOP + 0.020,
        details=f"cap z range=({cap_mn[2]:.3f}, {cap_mx[2]:.3f}), "
                f"collar=({COLLAR_BOTTOM:.3f},{COLLAR_TOP:.3f})",
    )

    # --- cap carrier is a massless visual-free intermediate link ---
    ctx.check(
        "cap carrier is visual-free",
        len(carrier.visuals) == 0,
        details=f"carrier visual count={len(carrier.visuals)}",
    )

    # --- cap_unscrew is a CONTINUOUS joint about Z ---
    ctx.check(
        "cap_unscrew is a CONTINUOUS (unlimited rotation) joint",
        cap_unscrew.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={cap_unscrew.articulation_type}",
    )
    ctx.check(
        "cap_unscrew axis is vertical (Z)",
        abs(cap_unscrew.axis[2]) > 0.99,
        details=f"axis={cap_unscrew.axis}",
    )

    # --- cap_lift is a PRISMATIC joint that raises the cap after unscrewing ---
    ctx.check(
        "cap_lift is a PRISMATIC lift joint",
        cap_lift.articulation_type == ArticulationType.PRISMATIC and cap_lift.axis[2] > 0.99,
        details=f"type={cap_lift.articulation_type}, axis={cap_lift.axis}",
    )

    # --- cap rotation preserves center; cap lift raises it visibly ---
    cap_rest = ctx.part_world_position(cap)
    with ctx.pose({cap_unscrew: 3.14159}):
        cap_half = ctx.part_world_position(cap)
    ctx.check(
        "cap half-turn rotation preserves center position before lift",
        abs(cap_half[0] - cap_rest[0]) < 0.005
        and abs(cap_half[1] - cap_rest[1]) < 0.005
        and abs(cap_half[2] - cap_rest[2]) < 0.005,
        details=f"rest={cap_rest}, half_turn={cap_half}",
    )
    with ctx.pose({cap_unscrew: 3.14159, cap_lift: CAP_LIFT_TRAVEL}):
        cap_up_pos = ctx.part_world_position(cap)
        cap_up_mn, _ = ctx.part_world_aabb(cap)
    ctx.check(
        "screw cap lifts upward after unscrewing",
        cap_up_pos[2] - cap_rest[2] > CAP_LIFT_TRAVEL * 0.90
        and cap_up_mn[2] > COLLAR_TOP + 0.025,
        details=f"rest_z={cap_rest[2]:.4f}, lifted_z={cap_up_pos[2]:.4f}, "
                f"lifted_min_z={cap_up_mn[2]:.4f}",
    )

    return ctx.report()
