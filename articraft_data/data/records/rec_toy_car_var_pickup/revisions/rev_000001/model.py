from __future__ import annotations

# Wooden push toy pickup truck with a bead-maze wire arch.
# Frame: truck length along +X (cab at +X front), width along Y, up +Z.
#   Body rests on four red wheels (ground z=0).
# Articulations:
#   - 4 wheels: each CONTINUOUS roll about its Y axle.
#   - 3 beads on the wire arch: each CONTINUOUS spin about its local wire segment.

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
    SphereGeometry,
    TestContext,
    TestReport,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---- key dimensions (meters) ----
WHEEL_R = 0.025          # wheel radius (~0.05 m dia)
WHEEL_W = 0.016          # wheel width
AXLE_Z = WHEEL_R         # axle height above ground
AXLE_FX = 0.044          # front axle X
AXLE_RX = -0.044         # rear axle X
AXLE_Y = 0.052           # half track (axle ends out to +/- Y, wheels clear body)

BODY_LEN = 0.150
BODY_BOT_Z = 0.022       # underside of the wooden body, clears ground

# Truck-specific dimensions
BASE_H = 0.022           # base chassis slab height
CAB_H = 0.050            # cab block height above base
CAB_XC = 0.038           # cab center X
CAB_LEN = 0.065          # cab length (X extent)
CAB_W = 0.078            # cab width (Y extent)
CAB_TOP_Z = BODY_BOT_Z + BASE_H + CAB_H   # 0.094

BED_WALL_H = 0.026       # cargo bed wall height above base
BED_LEN = 0.080          # cargo bed outer length
BED_W = 0.082            # cargo bed outer width
BED_XC = -0.030          # cargo bed center X
BED_TOP_Z = BODY_BOT_Z + BASE_H + BED_WALL_H  # 0.070

WALL_T = 0.007           # cargo bed wall thickness
WELL_R = 0.016           # cockpit well radius
WELL_DEPTH = 0.020       # cockpit well depth into cab


def _truck_body() -> cq.Workplane:
    """Build a chunky wooden pickup truck body as one CadQuery solid.

    The body is carved as a single rounded piece:
    - Lower base chassis slab spanning the full length
    - Rounded front cab block with a cockpit well on top
    - Rear cargo bed with short raised side walls and a tailgate (hollow interior)
    """
    base_zc = BODY_BOT_Z + BASE_H / 2
    cab_zc = BODY_BOT_Z + BASE_H + CAB_H / 2
    cab_top_z = CAB_TOP_Z
    wall_zc = BODY_BOT_Z + BASE_H + BED_WALL_H / 2

    # 1. Base chassis slab - rounded platform
    result = (
        cq.Workplane("XY")
        .box(BODY_LEN, 0.084, BASE_H)
        .translate((0, 0, base_zc))
    )
    try:
        result = result.edges("|Z").fillet(0.008)
    except Exception:
        pass

    # 2. Cab block - rounded box on the front half
    cab = (
        cq.Workplane("XY")
        .box(CAB_LEN, CAB_W, CAB_H)
        .translate((CAB_XC, 0, cab_zc))
    )
    try:
        cab = cab.edges("|Z").fillet(0.008)
    except Exception:
        pass
    result = result.union(cab)

    # 3. Cargo bed outer walls - rounded box on the rear half
    bed_outer = (
        cq.Workplane("XY")
        .box(BED_LEN, BED_W, BED_WALL_H)
        .translate((BED_XC, 0, wall_zc))
    )
    try:
        bed_outer = bed_outer.edges("|Z").fillet(0.006)
    except Exception:
        pass
    result = result.union(bed_outer)

    # 4. Cut cargo bed interior cavity (leaves floor, tailgate, and side walls)
    cavity_l = BED_LEN - WALL_T          # shorter at rear to leave tailgate
    cavity_w = BED_W - 2 * WALL_T        # narrower to leave side walls
    cavity_h = BED_WALL_H + 0.004        # extends above walls for clean top cut
    cavity_xc = BED_XC + WALL_T / 2      # offset forward to leave rear tailgate
    cavity_zc = BODY_BOT_Z + BASE_H + (cavity_h / 2) + 0.002  # leaves thin floor
    cavity = (
        cq.Workplane("XY")
        .box(cavity_l, cavity_w, cavity_h)
        .translate((cavity_xc, 0, cavity_zc))
    )
    result = result.cut(cavity)

    # 5. Cut cockpit well from cab top (open cab for the driver)
    well = (
        cq.Workplane("XY")
        .circle(WELL_R)
        .extrude(WELL_DEPTH + 0.005)
        .translate((CAB_XC, 0, cab_top_z - WELL_DEPTH))
    )
    result = result.cut(well)

    # 6. Round the top edges of the finished body
    try:
        result = result.edges(">Z").fillet(0.004)
    except Exception:
        pass

    return result


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wooden_push_toy_truck")

    wood = model.material("natural_wood", rgba=(0.80, 0.62, 0.36, 1.0))
    wood_dark = model.material("wood_dark", rgba=(0.55, 0.40, 0.22, 1.0))
    wood_light = model.material("wood_light", rgba=(0.90, 0.76, 0.50, 1.0))
    red = model.material("toy_red", rgba=(0.85, 0.13, 0.12, 1.0))
    hub_light = model.material("hub_light", rgba=(0.88, 0.86, 0.82, 1.0))
    black = model.material("face_black", rgba=(0.10, 0.10, 0.11, 1.0))
    metal = model.material("wire_metal", rgba=(0.35, 0.62, 0.80, 1.0))  # blue-ish wire
    bead_red = model.material("bead_red", rgba=(0.88, 0.18, 0.16, 1.0))
    bead_orange = model.material("bead_orange", rgba=(0.93, 0.55, 0.10, 1.0))
    bead_green = model.material("bead_green", rgba=(0.30, 0.66, 0.26, 1.0))

    # ==================== BODY (root) ====================
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_truck_body(), "truck_body"),
        material=wood,
        name="truck_body",
    )

    # Driver neck/stem connecting head to the cockpit well floor.
    # This ensures the driver assembly is physically connected to the body.
    stem_bot_z = CAB_TOP_Z - WELL_DEPTH - 0.003   # embeds into cab below well
    stem_top_z = CAB_TOP_Z - 0.002                 # overlaps into head sphere
    stem_h = stem_top_z - stem_bot_z
    stem = CylinderGeometry(0.006, stem_h, radial_segments=14)
    stem.translate(CAB_XC, 0.0, stem_bot_z + stem_h / 2)
    body.visual(
        mesh_from_geometry(stem, "driver_stem"),
        material=wood_light,
        name="driver_stem",
    )

    # Small round wooden DRIVER HEAD poking up from the open cab cockpit.
    head_r = 0.014
    head = SphereGeometry(head_r, width_segments=28, height_segments=20)
    head.translate(CAB_XC, 0.0, CAB_TOP_Z)
    body.visual(
        mesh_from_geometry(head, "driver_head"),
        material=wood_light,
        name="driver_head",
    )

    # Two small driver eyes on the front face of the head (embedded into sphere).
    for i, ey in enumerate((-0.006, 0.006)):
        eye = CylinderGeometry(0.0025, 0.004, radial_segments=12).rotate_y(
            math.pi / 2.0
        )
        eye.translate(CAB_XC + head_r - 0.005, ey, CAB_TOP_Z + 0.004)
        body.visual(
            mesh_from_geometry(eye, f"driver_eye_{i}"),
            material=black,
            name=f"driver_eye_{i}",
        )

    # Two axle pegs (wooden) spanning the body so wheels read as mounted on axles.
    for i, ax in enumerate((AXLE_FX, AXLE_RX)):
        peg = CylinderGeometry(0.005, 2 * AXLE_Y + 0.010, radial_segments=16).rotate_x(
            math.pi / 2.0
        )
        peg.translate(ax, 0.0, AXLE_Z)
        body.visual(
            mesh_from_geometry(peg, f"axle_{i}"),
            material=wood_dark,
            name=f"axle_{i}",
        )

    body.inertial = Inertial.from_geometry(
        Box((BODY_LEN, 0.085, 0.065)),
        mass=0.30,
        origin=Origin(xyz=(0.01, 0.0, 0.055)),
    )

    # ==================== WIRE ARCH (fixed to body) ====================
    # Bent metal wire looping from the cargo bed wall up over to the cab roof.
    arch_pts = [
        (-0.058, -0.037, BED_TOP_Z - 0.003),   # rear-left foot (on left bed wall)
        (-0.042, -0.030, 0.094),
        (-0.010, -0.006, 0.118),                # apex (rises well above body)
        (0.018, 0.014, 0.114),
        (0.036, 0.026, 0.102),
        (0.042, 0.030, CAB_TOP_Z - 0.003),      # front-right foot (on cab roof)
    ]
    wire = tube_from_spline_points(
        arch_pts,
        radius=0.0022,
        samples_per_segment=18,
        radial_segments=14,
        cap_ends=True,
    )
    arch = model.part("wire_arch")
    arch.visual(
        mesh_from_geometry(wire, "arch_wire"),
        material=metal,
        name="arch_wire",
    )
    arch.inertial = Inertial.from_geometry(
        Box((0.11, 0.07, 0.06)), mass=0.02, origin=Origin(xyz=(0.0, 0.0, 0.100))
    )
    model.articulation(
        "body_to_arch",
        ArticulationType.FIXED,
        parent=body,
        child=arch,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # ==================== BEADS (spin about the wire) ====================
    bead_specs = [
        ((-0.034, -0.022, 0.106), (0.50, 0.25, 0.80), bead_red, "bead_0"),
        ((0.002, 0.004, 0.117), (0.80, 0.50, -0.10), bead_orange, "bead_1"),
        ((0.030, 0.022, 0.106), (0.65, 0.45, -0.55), bead_green, "bead_2"),
    ]

    def _rot_to_axis(ax):
        n = math.sqrt(ax[0] ** 2 + ax[1] ** 2 + ax[2] ** 2)
        return (ax[0] / n, ax[1] / n, ax[2] / n)

    for center, tangent, mat, nm in bead_specs:
        ax = _rot_to_axis(tangent)
        disc = CylinderGeometry(0.011, 0.012, radial_segments=22).rotate_y(
            math.pi / 2.0
        )
        bead = model.part(nm)
        yaw = math.atan2(ax[1], ax[0])
        pitch = math.asin(max(-1.0, min(1.0, ax[2])))
        disc.rotate_y(-pitch)
        disc.rotate_z(yaw)
        bead.visual(
            mesh_from_geometry(disc, "bead_disc"),
            material=mat,
            name="bead_disc",
        )
        bead.inertial = Inertial.from_geometry(
            Cylinder(0.011, 0.012), mass=0.004
        )
        model.articulation(
            f"arch_to_{nm}",
            ArticulationType.CONTINUOUS,
            parent=arch,
            child=bead,
            origin=Origin(xyz=center),
            axis=ax,
            motion_limits=MotionLimits(effort=0.2, velocity=10.0),
        )

    # ==================== WHEELS (roll about axles) ====================
    def _wheel_mesh():
        wheel = WheelGeometry(
            WHEEL_R,
            WHEEL_W,
            rim=WheelRim(
                inner_radius=0.018, flange_height=0.004, flange_thickness=0.003
            ),
            hub=WheelHub(radius=0.009, width=WHEEL_W * 0.6, cap_style="recessed"),
            face=WheelFace(dish_depth=0.004, front_inset=0.002),
            bore=WheelBore(style="round", diameter=0.006),
        )
        wheel.rotate_z(math.pi / 2.0)
        return mesh_from_geometry(wheel, "wheel_body")

    wheel_specs = [
        (AXLE_FX, 1, "wheel_front_left"),
        (AXLE_FX, -1, "wheel_front_right"),
        (AXLE_RX, 1, "wheel_rear_left"),
        (AXLE_RX, -1, "wheel_rear_right"),
    ]
    for ax_x, ysign, nm in wheel_specs:
        w = model.part(nm)
        wm = _wheel_mesh()
        w.visual(wm, material=red, name="wheel_body")
        hub = CylinderGeometry(0.010, 0.008, radial_segments=20).rotate_x(
            math.pi / 2.0
        )
        hub.translate(0.0, ysign * (WHEEL_W / 2.0 - 0.002), 0.0)
        w.visual(
            mesh_from_geometry(hub, "hub_cap"),
            material=hub_light,
            name="hub_cap",
        )
        marker = CylinderGeometry(0.0022, 0.010, radial_segments=10).rotate_x(
            math.pi / 2.0
        )
        marker.translate(0.013, ysign * (WHEEL_W / 2.0 - 0.003), 0.0)
        w.visual(
            mesh_from_geometry(marker, "spin_marker"),
            material=black,
            name="spin_marker",
        )

        w.inertial = Inertial.from_geometry(
            Box((2 * WHEEL_R, WHEEL_W, 2 * WHEEL_R)), mass=0.02
        )
        model.articulation(
            f"axle_{nm}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=w,
            origin=Origin(xyz=(ax_x, ysign * AXLE_Y, AXLE_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.5, velocity=20.0),
        )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    arch = object_model.get_part("wire_arch")

    # ---- truck body: cab is tall in front, bed walls lower in rear ----
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "truck body has substantial length",
        body_aabb is not None and (body_aabb[1][0] - body_aabb[0][0]) > 0.12,
        details=f"body length={body_aabb[1][0] - body_aabb[0][0]:.4f}" if body_aabb else "no aabb",
    )
    ctx.check(
        "cab reaches truck cab height",
        body_aabb is not None and body_aabb[1][2] > CAB_TOP_Z - 0.006,
        details=f"body max_z={body_aabb[1][2] if body_aabb else None}, cab_top={CAB_TOP_Z}",
    )

    # ---- driver head pokes up above the cab, in the front portion ----
    head_aabb = ctx.part_element_world_aabb(body, elem="driver_head")
    ctx.check(
        "driver head pokes above cab roof",
        head_aabb is not None
        and (head_aabb[0][2] + head_aabb[1][2]) / 2.0 > CAB_TOP_Z - 0.002,
        details=f"head center_z={(head_aabb[0][2] + head_aabb[1][2]) / 2.0 if head_aabb else None}",
    )
    ctx.check(
        "driver head is in the front half of the truck",
        head_aabb is not None
        and (head_aabb[0][0] + head_aabb[1][0]) / 2.0 > 0.02,
        details=f"head center_x={(head_aabb[0][0] + head_aabb[1][0]) / 2.0 if head_aabb else None}",
    )

    # ---- wire arch rises above the truck body ----
    arch_aabb = ctx.part_world_aabb(arch)
    ctx.check(
        "wire arch rises above the truck body",
        arch_aabb is not None and arch_aabb[1][2] > CAB_TOP_Z + 0.02,
        details=f"arch top_z={arch_aabb[1][2] if arch_aabb else None}, cab_top={CAB_TOP_Z}",
    )
    # arch feet embed into truck body (cab roof + bed wall)
    ctx.allow_overlap(
        arch,
        body,
        elem_a="arch_wire",
        elem_b="truck_body",
        reason="Wire-arch feet are intentionally seated into the truck cab roof and bed wall.",
    )
    ctx.expect_contact(arch, body, name="wire arch attached to body")

    # ---- car rests on 4 wheels ----
    for nm in (
        "wheel_front_left",
        "wheel_front_right",
        "wheel_rear_left",
        "wheel_rear_right",
    ):
        w = object_model.get_part(nm)
        w_aabb = ctx.part_world_aabb(w)
        ctx.check(
            f"{nm} reaches the ground",
            w_aabb is not None and w_aabb[0][2] <= 0.004,
            details=f"{nm} min_z={w_aabb[0][2] if w_aabb else None}",
        )
        peg = "axle_0" if "front" in nm else "axle_1"
        ctx.allow_overlap(
            w,
            body,
            elem_a="wheel_body",
            elem_b=peg,
            reason="Wheel hub bore is intentionally seated over the body axle peg.",
        )

    # ---- all 4 wheels roll about their axles ----
    for nm in (
        "wheel_front_left",
        "wheel_front_right",
        "wheel_rear_left",
        "wheel_rear_right",
    ):
        w = object_model.get_part(nm)
        joint = object_model.get_articulation(f"axle_{nm}")
        m0 = ctx.part_element_world_aabb(w, elem="spin_marker")
        with ctx.pose({joint: math.pi / 2.0}):
            m1 = ctx.part_element_world_aabb(w, elem="spin_marker")
        c0 = ((m0[0][0] + m0[1][0]) / 2.0, (m0[0][2] + m0[1][2]) / 2.0)
        c1 = ((m1[0][0] + m1[1][0]) / 2.0, (m1[0][2] + m1[1][2]) / 2.0)
        moved = math.hypot(c1[0] - c0[0], c1[1] - c0[1])
        ctx.check(
            f"{nm} rolls about its axle (marker moves)",
            moved > 0.008,
            details=f"{nm} marker moved={moved:.4f}",
        )

    # ---- beads are threaded on the wire ----
    for nm in ("bead_0", "bead_1", "bead_2"):
        bd = object_model.get_part(nm)
        ctx.allow_overlap(
            bd,
            arch,
            elem_a="bead_disc",
            elem_b="arch_wire",
            reason="Bead is threaded onto the wire; disc bore intentionally overlaps the wire.",
        )
        ctx.expect_contact(bd, arch, name=f"{nm} threaded on the wire")

    # ---- a sampled bead spins about the wire ----
    bead = object_model.get_part("bead_1")
    bjoint = object_model.get_articulation("arch_to_bead_1")
    b0 = ctx.part_world_aabb(bead)
    with ctx.pose({bjoint: math.pi / 2.0}):
        b1 = ctx.part_world_aabb(bead)
    d0 = _ext(b0)
    d1 = _ext(b1)
    delta = abs(d0[1] - d1[1]) + abs(d0[2] - d1[2]) + abs(d0[0] - d1[0])
    ctx.check(
        "bead spins about the wire (extents change)",
        delta > 0.003,
        details=f"rest_ext={d0}, spun_ext={d1}, delta={delta:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
