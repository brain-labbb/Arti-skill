from __future__ import annotations

# FLAT-OVAL AEROSOL SPRAY PAINT CAN with a lift-off dust cap.
# Fork of the straight-cylinder parent: body footprint is a flat-oval
# (elliptical) cross-section rather than a perfect circle, like a slim flat
# spray can. The crimped dome adapts to the oval rim, transitioning to a
# round mounting cup at the top. The dust cap rim follows the oval.
# Press-down nozzle button and lift-off cap mechanisms are identical.
#
# Frame: can axis along +Z. Base rim sits at z=0, can body rises in +Z, the
# crimped dome and spray nozzle are at the top.

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key dimensions (meters) ----
CAN_RX = 0.035            # body half-width (X axis, ~70 mm wide)
CAN_RY = 0.020            # body half-depth (Y axis, ~40 mm deep, flat oval)
BODY_BOTTOM = 0.0         # base plane
BODY_TOP = 0.150          # where the straight wall ends and the dome begins
DOME_TOP = 0.176          # top of the crimped dome (mounting cup rim)
VALVE_TOP = 0.182         # top of the central valve stem the nozzle seats on
NOZZLE_SEAT_Z = 0.176     # z where the nozzle button base sits
NOZZLE_TOP_Z = 0.196      # top of the nozzle button
CAP_BOTTOM = 0.150        # dust cap lower rim (sleeves down over the body shoulder)
CAP_TOP = 0.210           # dust cap top

# Rim offset for the rolled bottom seam
RIM_EXTRA = 0.0012


def _oval_profile(rx: float, ry: float, z_offset: float = 0.0) -> cq.Workplane:
    """Shared helper: create an elliptical wire on a workplane at z_offset.

    When rx ≈ ry the profile is nearly round; otherwise it reads as a flat oval.
    """
    wp = cq.Workplane("XY")
    if z_offset != 0.0:
        wp = wp.workplane(offset=z_offset)
    return wp.ellipse(rx, ry)


def _can_body_solid() -> cq.Workplane:
    # Flat-oval aerosol can body: bottom concave rim, straight oval wall, a
    # crimped shoulder dome transitioning from oval to the round mounting cup
    # rim, and a short central valve stem.
    # Build the full body as one lofted solid from base to dome top for clean
    # mesh connectivity, then union the valve stem and bottom rim.

    # Full body loft: bottom rim → straight wall → crimped dome.
    body = (
        cq.Workplane("XY")
        .ellipse(CAN_RX + RIM_EXTRA, CAN_RY + RIM_EXTRA)   # bottom rim (z=0)
        .workplane(offset=0.006)
        .ellipse(CAN_RX, CAN_RY)                             # wall start (z=0.006)
        .workplane(offset=BODY_TOP - 0.006 - 0.001)
        .ellipse(CAN_RX, CAN_RY)                             # wall end (~z=BODY_TOP)
        .workplane(offset=0.006)
        .ellipse(CAN_RX - 0.001, CAN_RY - 0.001)            # crimp lip ring
        .workplane(offset=0.004)
        .ellipse(CAN_RX - 0.006, CAN_RY - 0.006)            # shoulder narrowing
        .workplane(offset=0.017)                              # rise to DOME_TOP
        .circle(0.014)                                        # round mounting-cup rim
        .loft(ruled=False)
    )

    # Central valve stem boss the nozzle seats over (stays circular).
    # Extend 2mm into the dome for solid mesh connectivity.
    stem = (
        cq.Workplane("XY")
        .workplane(offset=DOME_TOP - 0.004)
        .circle(0.006)
        .extrude(VALVE_TOP - DOME_TOP + 0.004)
    )
    body = body.union(stem)
    return body


def _label_band_solid() -> cq.Workplane:
    # Thin oval shell sleeve that wraps the straight wall as a label band.
    h = 0.105
    z_base = 0.012
    outer = (
        cq.Workplane("XY")
        .workplane(offset=z_base)
        .ellipse(CAN_RX + 0.0004, CAN_RY + 0.0004)
        .extrude(h)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=z_base - 0.001)
        .ellipse(CAN_RX - 0.0002, CAN_RY - 0.0002)
        .extrude(h + 0.002)
    )
    return outer.cut(inner)


def _nozzle_solid() -> cq.Workplane:
    # Press-down spray button: small rounded block seated on the valve stem,
    # with a tiny spray hole drilled in the front face. (Identical to parent.)
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
    # Dark grey dust cap: a hollow oval shell, closed domed top, that sleeves
    # down over the can's oval top shoulder. The rim follows the oval body.
    h = CAP_TOP - CAP_BOTTOM
    cap_rx = CAN_RX + 0.0035
    cap_ry = CAN_RY + 0.0035

    outer = (
        cq.Workplane("XY")
        .ellipse(cap_rx, cap_ry)
        .extrude(h - 0.010)
    )

    # Rounded closed top: loft from oval to a small round apex.
    top = (
        cq.Workplane("XY")
        .workplane(offset=h - 0.011)
        .ellipse(cap_rx, cap_ry)
        .workplane(offset=0.007)
        .ellipse(CAN_RX - 0.004, CAN_RY - 0.004)
        .workplane(offset=0.004)
        .circle(0.010)
        .loft(ruled=False)
    )
    cap = outer.union(top)

    # Hollow the inside so it slips over the oval can top.
    bore = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .ellipse(CAN_RX - 0.0006, CAN_RY - 0.0006)
        .extrude(h - 0.014)
    )
    cap = cap.cut(bore)
    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="oval_spray_paint_can")

    label = model.material("label_splatter", rgba=(0.12, 0.55, 0.82, 1.0))
    metal = model.material("metal", rgba=(0.78, 0.80, 0.83, 1.0))
    dark_cap = model.material("dark_grey_cap", rgba=(0.18, 0.18, 0.22, 1.0))
    dark_nozzle = model.material("dark_nozzle", rgba=(0.16, 0.16, 0.17, 1.0))

    # ---- can body (root) ----
    body = model.part("can_body")
    body.visual(
        mesh_from_cadquery(_can_body_solid(), "can_body"),
        material=metal,
        name="can_body",
    )
    # Oval label band wrapped on the straight wall (thin sleeve).
    body.visual(
        mesh_from_cadquery(_label_band_solid(), "label_band"),
        material=label,
        name="label_band",
    )
    body.inertial = Inertial.from_geometry(
        Box((2.0 * CAN_RX, 2.0 * CAN_RY, 0.176)),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
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

    # ---- dust cap: child of the can body, lifts straight up a LARGE amount ----
    cap = model.part("dust_cap")
    cap.visual(
        mesh_from_cadquery(_dust_cap_solid(), "dust_cap"),
        material=dark_cap,
        name="dust_cap",
    )
    cap.inertial = Inertial.from_geometry(
        Box((2.0 * (CAN_RX + 0.0035), 2.0 * (CAN_RY + 0.0035), CAP_TOP - CAP_BOTTOM)),
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

    # --- can body is a tall flat-oval shape (wider in X than Y) ---
    bmn, bmx = ctx.part_world_aabb(body)
    height = bmx[2] - bmn[2]
    dia_x = bmx[0] - bmn[0]
    dia_y = bmx[1] - bmn[1]
    ctx.check(
        "can body is tall with flat-oval cross-section",
        height > 0.16 and dia_x > 0.055 and dia_y > 0.030 and dia_x > dia_y * 1.3,
        details=f"height={height:.3f}, dia_x={dia_x:.3f}, dia_y={dia_y:.3f}",
    )
    ctx.check(
        "body X diameter ~70 mm, Y diameter ~40 mm (flat oval)",
        0.060 < dia_x < 0.080 and 0.033 < dia_y < 0.050,
        details=f"dia_x={dia_x:.3f}, dia_y={dia_y:.3f}",
    )

    # --- dust cap also has oval cross-section matching the body ---
    cmn, cmx = ctx.part_world_aabb(cap)
    cap_dx = cmx[0] - cmn[0]
    cap_dy = cmx[1] - cmn[1]
    ctx.check(
        "dust cap has oval cross-section matching body oval",
        cap_dx > cap_dy * 1.2 and cap_dx > 0.060 and cap_dy > 0.035,
        details=f"cap_dx={cap_dx:.3f}, cap_dy={cap_dy:.3f}",
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

    # --- dust cap sleeves over the oval can top (intentional overlap) ---
    ctx.allow_overlap(
        cap,
        body,
        elem_a="dust_cap",
        elem_b="can_body",
        reason="Dust cap intentionally sleeves down over the can's oval top shoulder when seated.",
    )
    cap_rest_mn, cap_rest_mx = ctx.part_world_aabb(cap)
    ctx.check(
        "dust cap is seated over the can top at rest",
        cap_rest_mn[2] < 0.16 and cap_rest_mx[2] > 0.20,
        details=f"cap z range at rest=({cap_rest_mn[2]:.3f}, {cap_rest_mx[2]:.3f})",
    )

    # --- dust cap lifts straight UP a LARGE amount, ending clear above the nozzle ---
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
    # Lift is purely vertical: cap stays centered on the axis.
    ctx.check(
        "cap lift is purely vertical (no lateral drift)",
        abs(cap_up_pos[0]) < 0.005 and abs(cap_up_pos[1]) < 0.005,
        details=f"lifted cap xy=({cap_up_pos[0]:.4f}, {cap_up_pos[1]:.4f})",
    )

    return ctx.report()
