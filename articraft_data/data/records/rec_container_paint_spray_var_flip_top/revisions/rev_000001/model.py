from __future__ import annotations

# Aerosol SPRAY PAINT CAN with a hinged flip-top cap.
# Fork of the lift-off dust cap parent: the can body, crimped dome, and press-down
# spray nozzle button are identical. The closure is now a one-piece flip-top lid
# hinged at the rear top shoulder of the can, swinging open on a REVOLUTE joint
# (axis horizontal, tangent to the rim) to expose the nozzle below.
#
# Frame: can axis along +Z. Base rim at z=0, body rises in +Z.
# Articulations:
#   - cap_hinge: REVOLUTE at rear shoulder, axis along +X (tangent to rim),
#     positive q opens the cap upward/backward.
#   - nozzle_press: PRISMATIC press straight DOWN (-Z), identical to parent.

import math

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
CAN_R = 0.0325            # body radius (~0.065 m diameter)
BODY_BOTTOM = 0.0         # base plane
BODY_TOP = 0.150          # where the straight wall ends and the dome begins
DOME_TOP = 0.176          # top of the crimped dome (mounting cup rim)
VALVE_TOP = 0.182         # top of the central valve stem the nozzle seats on
NOZZLE_SEAT_Z = 0.176     # z where the nozzle button base sits
NOZZLE_TOP_Z = 0.196      # top of the nozzle button

# Flip-top cap dimensions
CAP_H = 0.052             # cap wall height
CAP_SKIRT = 0.003         # skirt drop below hinge
CAP_WALL = 0.003          # shell wall thickness
HINGE_OFFSET = 0.005      # hinge pin offset outside can wall for rotation clearance
HINGE_Y = -(CAN_R + HINGE_OFFSET)  # hinge Y in world (rear of can)
HINGE_Z = BODY_TOP        # hinge Z in world (shoulder height)
CAP_R = CAN_R + 0.002     # cap outer radius (slight lip over can)
CAP_CENTER_Y = -HINGE_Y   # can center Y in cap local frame

# Hinge hardware layout
EAR_COUNT = 2
EAR_SPACING = 0.026       # center-to-center along X
EAR_W = 0.005             # ear width along X
EAR_D = 0.014             # ear depth along Y (overlaps body wall for connectivity)
EAR_H = 0.012             # ear height along Z
MOUNT_WIDTH = 0.020       # hinge mount width along X on the cap
MOUNT_DEPTH = 0.014       # hinge mount depth along Y
MOUNT_HEIGHT = 0.014      # hinge mount height above skirt bottom


# ---- geometry helpers ----

def _can_body_solid() -> cq.Workplane:
    """Tall cylindrical can body with bottom rim, straight wall, crimped shoulder
    dome, and central valve stem. Identical to parent."""
    body = (
        cq.Workplane("XY")
        .circle(CAN_R + 0.0012)
        .extrude(0.006)
    )
    body = body.union(
        cq.Workplane("XY")
        .workplane(offset=0.005)
        .circle(CAN_R)
        .extrude(BODY_TOP - 0.005)
    )
    dome = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP - 0.001)
        .circle(CAN_R)
        .workplane(offset=0.006)
        .circle(CAN_R - 0.001)
        .workplane(offset=0.004)
        .circle(CAN_R - 0.006)
        .workplane(offset=0.017)
        .circle(0.014)
        .loft(ruled=False)
    )
    body = body.union(dome)
    body = body.union(
        cq.Workplane("XY")
        .workplane(offset=DOME_TOP - 0.001)
        .circle(0.006)
        .extrude(VALVE_TOP - DOME_TOP)
    )
    return body


def _nozzle_solid() -> cq.Workplane:
    """Press-down spray button with spray hole on front face. Identical to parent."""
    btn = (
        cq.Workplane("XY")
        .box(0.018, 0.016, 0.014, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.003)
    )
    hole = (
        cq.Workplane("YZ")
        .workplane(offset=0.009)
        .center(0.0, 0.008)
        .circle(0.0014)
        .extrude(-0.010)
    )
    btn = btn.cut(hole)
    return btn


def _flip_cap_solid() -> cq.Workplane:
    """One-piece flip-top cap shell with integrated hinge mount.
    Local origin at the hinge pin center. At q=0 (closed), the cap extends
    forward (+Y) and upward (+Z), covering the can dome and nozzle."""
    cy = CAP_CENTER_Y  # can center Y in local frame

    # Outer cylindrical wall
    outer = (
        cq.Workplane("XY")
        .workplane(offset=-CAP_SKIRT)
        .center(0, cy)
        .circle(CAP_R)
        .extrude(CAP_H + CAP_SKIRT)
    )
    # Rounded dome top via fillet (single connected solid)
    outer = outer.edges(">Z").fillet(0.004)

    # Hinge mount: reinforced block at the rear, added before bore cut so
    # the bore does not sever the mount-shell connection.
    mount_cy = 0.002  # mount center Y (straddles the shell rear wall at y≈0.003)
    mount = (
        cq.Workplane("XY")
        .workplane(offset=-CAP_SKIRT)
        .center(0, mount_cy)
        .box(MOUNT_WIDTH, MOUNT_DEPTH, MOUNT_HEIGHT, centered=(True, True, False))
    )
    outer = outer.union(mount)

    # Hollow bore: starts above the skirt base so the lower cap wall stays
    # solid, preserving a robust mount-shell connection through the base.
    bore_start = 0.010  # higher start for better mesh connectivity at base
    bore_top = CAP_H - 0.004
    bore = (
        cq.Workplane("XY")
        .workplane(offset=bore_start)
        .center(0, cy)
        .circle(CAP_R - CAP_WALL)
        .extrude(bore_top - bore_start)
    )
    cap = outer.cut(bore)

    return cap


def _hinge_ear_solid() -> cq.Workplane:
    """Small hinge ear tab. Local origin at base center, extends upward (+Z).
    Deep enough to overlap the can body wall for connectivity."""
    ear = (
        cq.Workplane("XY")
        .box(EAR_W, EAR_D, EAR_H, centered=(True, True, False))
    )
    ear = ear.edges(">Z").fillet(0.0018)
    return ear


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spray_paint_can")

    label = model.material("label_splatter", rgba=(0.90, 0.90, 0.92, 1.0))
    metal = model.material("metal", rgba=(0.78, 0.80, 0.83, 1.0))
    dark_cap = model.material("dark_grey_cap", rgba=(0.22, 0.24, 0.22, 1.0))
    dark_nozzle = model.material("dark_nozzle", rgba=(0.16, 0.16, 0.17, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.65, 0.67, 0.70, 1.0))

    # ---- can body (root) ---- identical to parent
    body = model.part("can_body")
    body.visual(
        mesh_from_cadquery(_can_body_solid(), "can_body"),
        material=metal,
        name="can_body",
    )
    label_band = CylinderGeometry(CAN_R + 0.0004, 0.105, radial_segments=64)
    label_band.translate(0.0, 0.0, 0.012 + 0.105 / 2.0)
    body.visual(
        mesh_from_geometry(label_band, "label_band"),
        material=label,
        name="label_band",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(CAN_R, 0.176),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
    )

    # Hinge ear tabs on the can body rear rim (repeated sub-parts via loop)
    for i in range(EAR_COUNT):
        x_off = -EAR_SPACING / 2.0 + i * EAR_SPACING
        body.visual(
            mesh_from_cadquery(_hinge_ear_solid(), f"hinge_ear_{i}"),
            material=hinge_metal,
            origin=Origin(xyz=(x_off, HINGE_Y, HINGE_Z)),
            name=f"hinge_ear_{i}",
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

    # ---- flip-top cap: child of can body, hinged at rear shoulder ----
    cap = model.part("flip_cap")
    cap.visual(
        mesh_from_cadquery(_flip_cap_solid(), "flip_cap"),
        material=dark_cap,
        name="flip_cap",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_H),
        mass=0.015,
        origin=Origin(xyz=(0.0, CAP_CENTER_Y, CAP_H / 2.0)),
    )
    model.articulation(
        "cap_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=0.0, upper=2.6,
        ),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("can_body")
    nozzle = object_model.get_part("spray_nozzle")
    cap = object_model.get_part("flip_cap")
    cap_hinge = object_model.get_articulation("cap_hinge")
    nozzle_press = object_model.get_articulation("nozzle_press")

    # --- can body is a tall cylinder (identical to parent) ---
    bmn, bmx = ctx.part_world_aabb(body)
    height = bmx[2] - bmn[2]
    dia_x = bmx[0] - bmn[0]
    dia_y = bmx[1] - bmn[1]
    ctx.check(
        "can body is a tall cylinder (~0.18 m, ~0.065 m dia)",
        height > 0.16 and 0.05 < dia_x < 0.09 and 0.05 < dia_y < 0.09 and height > dia_x * 2.0,
        details=f"height={height:.3f}, dia_x={dia_x:.3f}, dia_y={dia_y:.3f}",
    )

    # --- nozzle sits on the can top near center ---
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

    # --- flip cap is a REVOLUTE joint (not prismatic) ---
    ctx.check(
        "cap_hinge is a REVOLUTE articulation",
        cap_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={cap_hinge.articulation_type}",
    )

    # --- cap at rest covers the nozzle area (closed position) ---
    cap_rest_mn, cap_rest_mx = ctx.part_world_aabb(cap)
    nmn, nmx = ctx.part_world_aabb(nozzle)
    ctx.check(
        "closed cap covers the nozzle height range",
        cap_rest_mn[2] < nmn[2] + 0.005 and cap_rest_mx[2] > nmx[2] - 0.005,
        details=f"cap_z=({cap_rest_mn[2]:.3f}, {cap_rest_mx[2]:.3f}), "
                f"nozzle_z=({nmn[2]:.3f}, {nmx[2]:.3f})",
    )

    # --- cap skirt sleeves over the can shoulder at rest (intentional overlap) ---
    ctx.allow_overlap(
        cap,
        body,
        elem_a="flip_cap",
        elem_b="can_body",
        reason="Flip-top cap skirt intentionally sleeves over the can shoulder when closed.",
    )
    ctx.expect_overlap(
        cap, body, axes="z", min_overlap=0.001,
        name="cap skirt overlaps the shoulder at rest",
    )

    # --- hinge ears embed through the cap skirt to pivot on the body ---
    for i in range(EAR_COUNT):
        ctx.allow_overlap(
            body,
            cap,
            elem_a=f"hinge_ear_{i}",
            elem_b="flip_cap",
            reason=(
                f"Hinge ear {i} passes through the cap skirt rear wall to serve "
                "as a pivot post for the flip-top mechanism."
            ),
        )
    ctx.expect_contact(
        body, cap,
        elem_a="hinge_ear_0", elem_b="flip_cap",
        name="hinge ear contacts the cap at the pivot",
    )

    # --- cap opens by rotating: AABB center rises and max_z increases ---
    rest_center_z = (cap_rest_mn[2] + cap_rest_mx[2]) / 2.0
    rest_max_z = cap_rest_mx[2]
    with ctx.pose({cap_hinge: 1.2}):
        cap_open_mn, cap_open_mx = ctx.part_world_aabb(cap)
    open_center_z = (cap_open_mn[2] + cap_open_mx[2]) / 2.0
    open_max_z = cap_open_mx[2]

    ctx.check(
        "cap opens upward (AABB center rises when hinge opens)",
        open_center_z > rest_center_z + 0.005,
        details=f"rest_center_z={rest_center_z:.4f}, open_center_z={open_center_z:.4f}",
    )
    ctx.check(
        "cap opens upward (AABB max_z increases)",
        open_max_z > rest_max_z + 0.010,
        details=f"rest_max_z={rest_max_z:.4f}, open_max_z={open_max_z:.4f}",
    )

    # --- open cap exposes the nozzle: Y footprint clears the nozzle ---
    with ctx.pose({cap_hinge: 1.5}):
        cap_wide_mn, cap_wide_mx = ctx.part_world_aabb(cap)
    ctx.check(
        "open cap exposes nozzle (cap Y range clears nozzle Y range)",
        cap_wide_mx[1] < nmn[1] - 0.001 or cap_wide_mn[1] > nmx[1] + 0.001,
        details=f"cap_y=({cap_wide_mn[1]:.4f}, {cap_wide_mx[1]:.4f}), "
                f"nozzle_y=({nmn[1]:.4f}, {nmx[1]:.4f})",
    )

    # --- hinge axis is horizontal (tangent to rim, along X) ---
    ctx.check(
        "hinge axis is along +X (horizontal, tangent to rear rim)",
        abs(cap_hinge.axis[0]) > 0.9 and abs(cap_hinge.axis[1]) < 0.1 and abs(cap_hinge.axis[2]) < 0.1,
        details=f"axis={cap_hinge.axis}",
    )

    # --- hinge origin is at the rear top shoulder ---
    hinge_origin = cap_hinge.origin
    ctx.check(
        "hinge origin is at the rear top shoulder of the can",
        hinge_origin is not None
        and hinge_origin.xyz[1] < -CAN_R + 0.001
        and abs(hinge_origin.xyz[2] - BODY_TOP) < 0.005,
        details=f"hinge origin={hinge_origin.xyz}",
    )

    # --- hinge ears visible on the body ---
    body_visual_names = [v.name for v in body.visuals]
    ear_names = [n for n in body_visual_names if n.startswith("hinge_ear_")]
    ctx.check(
        "body has hinge ear tabs",
        len(ear_names) >= 2,
        details=f"ear visuals={ear_names}",
    )

    return ctx.report()
