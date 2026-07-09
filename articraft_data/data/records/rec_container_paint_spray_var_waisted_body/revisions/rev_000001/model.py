from __future__ import annotations

# WAIsted AEROSOL SPRAY PAINT CAN with a lift-off dust cap.
# Frame: can axis along +Z. Base rim sits at z=0, can body rises in +Z, the
# body waists inward at the upper third into a pronounced narrowed shoulder,
# then a crimped dome and spray nozzle at the top.
# Built as a lathe-revolved profile for a real hourglass-style contour.
# Articulations (two INDEPENDENT prismatic joints):
#   - dust cap: PRISMATIC lift straight UP (+Z) by ~0.09 m, revealing nozzle.
#   - spray nozzle button: PRISMATIC press straight DOWN (-Z, ~0.004 m).

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
CAN_R = 0.0325            # lower body radius (~0.065 m diameter)
SHOULDER_R = 0.024        # narrowed upper shoulder radius (~0.048 m dia)
WAIST_START_Z = 0.090     # where the concave waist curve begins
WAIST_END_Z = 0.115       # where the narrow upper wall starts
DOME_START_Z = 0.148      # where the crimped dome begins
DOME_TOP = 0.176          # top of the crimped dome (mounting cup rim)
VALVE_TOP = 0.182         # top of the central valve stem
NOZZLE_SEAT_Z = 0.176     # z where the nozzle button base sits
CAP_BOTTOM = 0.140        # dust cap lower rim (on the narrow shoulder)
CAP_TOP = 0.210           # dust cap top
CAP_R = SHOULDER_R + 0.003   # cap outer radius (fits narrow section)
CAP_BORE_R = SHOULDER_R - 0.0005  # cap bore (snap fit over narrow body)


def _can_body_solid() -> cq.Workplane:
    """Waisted aerosol can body: revolved half-profile on XZ plane.

    Full-diameter lower cylinder, concave waist at upper third narrowing to
    SHOULDER_R, crimped dome curving inward to the mounting cup rim, and a
    short central valve stem the nozzle seats on.
    """
    R = CAN_R
    Rr = SHOULDER_R
    Rrim = R + 0.0012  # bottom rim (rolled seam)

    # Draw the closed half-profile on the XZ workplane (x=radius, z=height).
    profile = (
        cq.Workplane("XZ")
        # Start at axis, bottom
        .moveTo(0.0, 0.0)
        # Bottom rim: slightly wider ring for rolled seam
        .lineTo(Rrim, 0.0)
        .lineTo(Rrim, 0.006)
        .lineTo(R, 0.007)
        # Straight lower wall up to waist start
        .lineTo(R, WAIST_START_Z)
        # Concave waist spline: body narrows from full radius to shoulder
        .spline(
            [
                (R - 0.002, WAIST_START_Z + 0.005),
                (R - 0.005, WAIST_START_Z + 0.010),
                (Rr + 0.002, WAIST_START_Z + 0.018),
                (Rr, WAIST_END_Z),
            ],
            includeCurrent=True,
            periodic=False,
            tangents=[(0.0, 1.0), (0.0, 1.0)],
        )
        # Narrow upper wall
        .lineTo(Rr, DOME_START_Z)
        # Dome spline: curves inward from shoulder to cup rim
        .spline(
            [
                (Rr - 0.002, DOME_START_Z + 0.005),
                (0.019, DOME_START_Z + 0.012),
                (0.016, DOME_START_Z + 0.020),
                (0.014, DOME_TOP - 0.001),
            ],
            includeCurrent=True,
            periodic=False,
            tangents=[(0.0, 1.0), (-0.5, 1.0)],
        )
        # Cup rim step to valve stem
        .lineTo(0.014, DOME_TOP)
        .lineTo(0.006, DOME_TOP)
        # Valve stem
        .lineTo(0.006, VALVE_TOP)
        # Close along axis back to origin
        .lineTo(0.0, VALVE_TOP)
        .close()
    )
    # Revolve the closed profile 360° around the Z axis.
    return profile.revolve(360, (0.0, 0.0), (0.0, 1.0))


def _nozzle_solid() -> cq.Workplane:
    """Press-down spray button: rounded block with a spray hole."""
    btn = (
        cq.Workplane("XY")
        .box(0.018, 0.016, 0.014, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.003)
    )
    # Tiny spray hole on the +X front face.
    hole = (
        cq.Workplane("YZ")
        .workplane(offset=0.009)
        .center(0.0, 0.008)
        .circle(0.0014)
        .extrude(-0.010)
    )
    btn = btn.cut(hole)
    return btn


def _dust_cap_solid() -> cq.Workplane:
    """Dust cap: hollow cylindrical shell with domed top, fits narrow shoulder."""
    h = CAP_TOP - CAP_BOTTOM
    outer = (
        cq.Workplane("XY")
        .circle(CAP_R)
        .extrude(h - 0.010)
    )
    # Rounded closed top.
    top = (
        cq.Workplane("XY")
        .workplane(offset=h - 0.011)
        .circle(CAP_R)
        .workplane(offset=0.007)
        .circle(CAP_R - 0.007)
        .workplane(offset=0.004)
        .circle(0.008)
        .loft(ruled=False)
    )
    cap = outer.union(top)
    # Hollow the inside so it slips over the narrow can shoulder.
    bore = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .circle(CAP_BORE_R)
        .extrude(h - 0.014)
    )
    cap = cap.cut(bore)
    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spray_paint_can")

    label = model.material("label_splatter", rgba=(0.88, 0.89, 0.91, 1.0))
    metal = model.material("metal", rgba=(0.78, 0.80, 0.83, 1.0))
    dark_cap = model.material("dark_grey_cap", rgba=(0.20, 0.21, 0.20, 1.0))
    dark_nozzle = model.material("dark_nozzle", rgba=(0.16, 0.16, 0.17, 1.0))

    # ---- can body (root) ----
    body = model.part("can_body")
    body.visual(
        mesh_from_cadquery(_can_body_solid(), "can_body"),
        material=metal,
        name="can_body",
    )
    # Label band on the straight lower wall (below the waist transition).
    label_height = WAIST_START_Z - 0.012  # fits on straight section
    label_band = CylinderGeometry(CAN_R + 0.0004, label_height, radial_segments=64)
    label_band.translate(0.0, 0.0, 0.012 + label_height / 2.0)
    body.visual(mesh_from_geometry(label_band, "label_band"), material=label, name="label_band")
    body.inertial = Inertial.from_geometry(
        Cylinder(CAN_R, DOME_TOP),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, DOME_TOP / 2.0)),
    )

    # ---- spray nozzle button: child of the can body, presses straight down ----
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

    # ---- dust cap: child of the can body, lifts straight up ----
    cap = model.part("dust_cap")
    cap.visual(
        mesh_from_cadquery(_dust_cap_solid(), "dust_cap"),
        material=dark_cap,
        name="dust_cap",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_TOP - CAP_BOTTOM),
        mass=0.015,
        origin=Origin(xyz=(0.0, 0.0, (CAP_TOP - CAP_BOTTOM) / 2.0)),
    )
    model.articulation(
        "cap_lift",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, CAP_BOTTOM)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.2, lower=0.0, upper=0.090),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("can_body")
    nozzle = object_model.get_part("spray_nozzle")
    cap = object_model.get_part("dust_cap")
    cap_lift = object_model.get_articulation("cap_lift")
    nozzle_press = object_model.get_articulation("nozzle_press")

    # --- can body overall dimensions ---
    bmn, bmx = ctx.part_world_aabb(body)
    height = bmx[2] - bmn[2]
    dia_x = bmx[0] - bmn[0]
    dia_y = bmx[1] - bmn[1]
    ctx.check(
        "waisted can body is tall (~0.18 m) with ~0.065 m base diameter",
        height > 0.16 and 0.055 < dia_x < 0.080 and 0.055 < dia_y < 0.080 and height > dia_x * 2.0,
        details=f"height={height:.3f}, dia_x={dia_x:.3f}, dia_y={dia_y:.3f}",
    )

    # --- waisted profile: dust cap (on narrow shoulder) is visibly narrower than body base ---
    cap_mn, cap_mx = ctx.part_world_aabb(cap)
    cap_dia_x = cap_mx[0] - cap_mn[0]
    cap_dia_y = cap_mx[1] - cap_mn[1]
    ctx.check(
        "dust cap is narrower than can body base (waisted hourglass profile)",
        cap_dia_x < dia_x - 0.008 and cap_dia_y < dia_y - 0.008,
        details=f"body_dia_x={dia_x:.4f}, cap_dia_x={cap_dia_x:.4f}, "
                f"body_dia_y={dia_y:.4f}, cap_dia_y={cap_dia_y:.4f}",
    )

    # --- nozzle sits on the can top, near the central axis ---
    npos = ctx.part_world_position(nozzle)
    ctx.check(
        "nozzle button is mounted on the can top",
        npos is not None and npos[2] > 0.16 and abs(npos[0]) < 0.01 and abs(npos[1]) < 0.01,
        details=f"nozzle origin={npos}",
    )

    # --- nozzle seats on the valve stem (intentional local overlap) ---
    ctx.allow_overlap(
        nozzle,
        body,
        elem_a="spray_nozzle",
        elem_b="can_body",
        reason="The spray button is intentionally seated over the central valve stem.",
    )
    ctx.expect_contact(nozzle, body, name="nozzle seated on the valve stem")

    # --- nozzle presses straight DOWN ---
    nz_rest = ctx.part_world_position(nozzle)[2]
    with ctx.pose({nozzle_press: 0.004}):
        nz_down = ctx.part_world_position(nozzle)[2]
    ctx.check(
        "nozzle button presses straight down",
        nz_down < nz_rest - 0.0035,
        details=f"rest_z={nz_rest:.4f}, pressed_z={nz_down:.4f}",
    )

    # --- dust cap sleeves over the narrow can shoulder (intentional overlap) ---
    ctx.allow_overlap(
        cap,
        body,
        elem_a="dust_cap",
        elem_b="can_body",
        reason="Dust cap intentionally sleeves down over the can's narrow shoulder when seated.",
    )
    cap_rest_mn, cap_rest_mx = ctx.part_world_aabb(cap)
    ctx.check(
        "dust cap is seated over the can top at rest",
        cap_rest_mn[2] < 0.15 and cap_rest_mx[2] > 0.19,
        details=f"cap z range at rest=({cap_rest_mn[2]:.3f}, {cap_rest_mx[2]:.3f})",
    )

    # --- dust cap lifts straight UP a LARGE amount, clearing the nozzle ---
    nmn, nmx = ctx.part_world_aabb(nozzle)
    nozzle_top = nmx[2]
    cap_rest_pos_z = ctx.part_world_position(cap)[2]
    with ctx.pose({cap_lift: 0.090}):
        cap_up_mn, cap_up_mx = ctx.part_world_aabb(cap)
        cap_up_pos = ctx.part_world_position(cap)
    ctx.check(
        "dust cap lifts straight up by a large amount (~0.09 m)",
        cap_up_pos[2] - cap_rest_pos_z > 0.085,
        details=f"cap rose from z={cap_rest_pos_z:.3f} to z={cap_up_pos[2]:.3f}",
    )
    ctx.check(
        "lifted cap bottom clears the nozzle top (nozzle revealed)",
        cap_up_mn[2] > nozzle_top + 0.002,
        details=f"lifted cap bottom={cap_up_mn[2]:.3f}, nozzle top={nozzle_top:.3f}",
    )
    ctx.check(
        "cap lift is purely vertical (no lateral drift)",
        abs(cap_up_pos[0]) < 0.005 and abs(cap_up_pos[1]) < 0.005,
        details=f"lifted cap xy=({cap_up_pos[0]:.4f}, {cap_up_pos[1]:.4f})",
    )

    return ctx.report()
