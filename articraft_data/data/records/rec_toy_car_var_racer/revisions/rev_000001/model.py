from __future__ import annotations

# Wooden push toy car: chunky open-racer body with driver, bead-maze wire arch, and four red wheels.
# Frame: car length along +X (pointed nose at +X front), width along Y, up +Z.
#   Body centerline at z so the car rests on four red wheels (ground z=0).
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
    TorusGeometry,
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
BODY_BOT_Z = 0.022       # underside of the wooden body, clears the ground
BODY_TOP_Z = 0.070       # top of the wooden deck

# Cockpit (open round pocket carved into the body top)
COCKPIT_X = -0.005       # cockpit center X (slightly behind body center)
COCKPIT_R = 0.020        # cockpit radius
COCKPIT_DEPTH = 0.018    # pocket depth below deck surface

# Driver head (small round wooden head sitting up in the cockpit)
DRIVER_R = 0.015
DRIVER_X = COCKPIT_X
DRIVER_CENTER_Z = BODY_TOP_Z + DRIVER_R * 0.55  # head center above deck


def _loft(sections) -> cq.Workplane:
    # sections: list of ("rect", x, w, h) or ("circle", x, r) along +X (YZ planes).
    wp = cq.Workplane("YZ")
    prev = 0.0
    for i, s in enumerate(sections):
        x = s[1]
        off = x if i == 0 else x - prev
        if i > 0 or abs(off) > 1e-12:
            wp = wp.workplane(offset=off)
        if s[0] == "circle":
            wp = wp.circle(s[2])
        else:
            wp = wp.rect(s[2], s[3])
        prev = x
    return wp.loft(ruled=False)


def _racer_body_solid() -> cq.Workplane:
    """Chunky wooden open-racer hull: low tapered body from rounded tail to pointed nose,
    with an open round cockpit pocket carved into the deck."""
    zc = (BODY_BOT_Z + BODY_TOP_Z) / 2.0
    h = BODY_TOP_Z - BODY_BOT_Z  # ~0.048

    # Hull cross-sections along +X (YZ planes), rear to front.
    # The rear sections are wide (rounded tail), tapering to a narrow pointed nose.
    body = _loft([
        ("rect", -0.070, 0.040, h * 0.76),   # rear tail: small, rounds after fillet
        ("rect", -0.054, 0.080, h * 0.92),   # widening
        ("rect", -0.018, 0.084, h),           # widest section (near cockpit)
        ("rect", 0.024, 0.068, h * 0.88),    # narrowing toward nose
        ("rect", 0.054, 0.036, h * 0.62),    # tapering
        ("rect", 0.074, 0.014, h * 0.36),    # pointed nose (small rounded tip)
    ])

    # Fillet long horizontal edges for a rounded toy look.
    try:
        body = body.edges("|X").fillet(0.006)
    except Exception:
        pass

    # Lift loft (built centered at z=0) up to the deck height.
    body = body.translate((0.0, 0.0, zc))

    # Cut the open cockpit pocket: a cylinder that penetrates the deck from above.
    cutter_bottom = BODY_TOP_Z - COCKPIT_DEPTH - 0.002
    cutter_height = COCKPIT_DEPTH + 0.009  # extends above body top for clean cut
    cockpit_cutter = (
        cq.Workplane("XY")
        .workplane(offset=cutter_bottom)
        .center(COCKPIT_X, 0.0)
        .circle(COCKPIT_R)
        .extrude(cutter_height)
    )
    body = body.cut(cockpit_cutter)

    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wooden_push_toy_car")

    wood = model.material("natural_wood", rgba=(0.80, 0.62, 0.36, 1.0))
    wood_dark = model.material("wood_dark", rgba=(0.66, 0.47, 0.26, 1.0))
    red = model.material("toy_red", rgba=(0.85, 0.13, 0.12, 1.0))
    hub_light = model.material("hub_light", rgba=(0.88, 0.86, 0.82, 1.0))
    black = model.material("face_black", rgba=(0.10, 0.10, 0.11, 1.0))
    metal = model.material("wire_metal", rgba=(0.35, 0.62, 0.80, 1.0))
    bead_red = model.material("bead_red", rgba=(0.88, 0.18, 0.16, 1.0))
    bead_orange = model.material("bead_orange", rgba=(0.93, 0.55, 0.10, 1.0))
    bead_green = model.material("bead_green", rgba=(0.30, 0.66, 0.26, 1.0))
    driver_skin = model.material("driver_wood", rgba=(0.88, 0.72, 0.46, 1.0))

    # ==================== BODY (root) ====================
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_racer_body_solid(), "racer_hull"),
        material=wood,
        name="racer_hull",
    )

    # ---- Driver head: small round wooden head sitting up in the cockpit ----
    driver = SphereGeometry(DRIVER_R, width_segments=28, height_segments=20)
    driver.translate(DRIVER_X, 0.0, DRIVER_CENTER_Z)
    body.visual(
        mesh_from_geometry(driver, "driver_head"),
        material=driver_skin,
        name="driver_head",
    )

    # Two dot eyes on the driver (facing forward +X, in the cockpit).
    eye_fwd = DRIVER_X + DRIVER_R * 0.82   # front face of head
    eye_z = DRIVER_CENTER_Z + DRIVER_R * 0.15
    for i, ey in enumerate((-0.006, 0.006)):
        eye = CylinderGeometry(0.003, 0.005, radial_segments=16).rotate_y(math.pi / 2.0)
        eye.translate(eye_fwd, ey, eye_z)
        body.visual(
            mesh_from_geometry(eye, f"driver_eye_{i}"),
            material=black,
            name=f"driver_eye_{i}",
        )

    # Smiling mouth on the driver (partial torus, below the eyes, facing +X).
    smile_z = DRIVER_CENTER_Z - DRIVER_R * 0.28
    smile = TorusGeometry(0.008, 0.0013, radial_segments=10, tubular_segments=24)
    smile.rotate_x(math.pi / 2.0)
    smile.rotate_z(math.pi / 2.0)
    smile.translate(DRIVER_X + DRIVER_R * 0.78, 0.0, smile_z)
    body.visual(
        mesh_from_geometry(smile, "driver_smile"),
        material=black,
        name="driver_smile",
    )

    # Two axle pegs (wooden) spanning the body so wheels read as mounted on axles.
    for i, ax in enumerate((AXLE_FX, AXLE_RX)):
        peg = CylinderGeometry(0.005, 2 * AXLE_Y + 0.010, radial_segments=16).rotate_x(
            math.pi / 2.0
        )
        peg.translate(ax, 0.0, AXLE_Z)
        body.visual(mesh_from_geometry(peg, f"axle_{i}"), material=wood_dark, name=f"axle_{i}")

    body.inertial = Inertial.from_geometry(
        Box((BODY_LEN, 0.085, 0.050)),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, (BODY_BOT_Z + BODY_TOP_Z) / 2.0)),
    )

    # ==================== WIRE ARCH (fixed to body) ====================
    # A bent metal wire that loops up over the deck; beads thread onto it.
    arch_pts = [
        (-0.050, -0.030, BODY_TOP_Z - 0.004),  # rear-left foot (into the deck)
        (-0.040, -0.028, 0.092),
        (-0.010, -0.010, 0.112),               # apex (rises above the body)
        (0.020, 0.012, 0.108),
        (0.046, 0.026, 0.086),
        (0.052, 0.030, BODY_TOP_Z - 0.004),    # front-right foot (into the deck)
    ]
    wire = tube_from_spline_points(
        arch_pts,
        radius=0.0022,
        samples_per_segment=18,
        radial_segments=14,
        cap_ends=True,
    )
    arch = model.part("wire_arch")
    arch.visual(mesh_from_geometry(wire, "arch_wire"), material=metal, name="arch_wire")
    arch.inertial = Inertial.from_geometry(
        Box((0.11, 0.07, 0.05)), mass=0.02, origin=Origin(xyz=(0.0, 0.0, 0.095))
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
        ((-0.030, -0.020, 0.101), (0.55, 0.22, 0.45), bead_red, "bead_0"),
        ((0.004, -0.001, 0.111), (0.80, 0.40, -0.08), bead_orange, "bead_1"),
        ((0.034, 0.020, 0.097), (0.55, 0.35, -0.62), bead_green, "bead_2"),
    ]

    def _rot_to_axis(geom_local, axis):
        ax = axis
        n = math.sqrt(ax[0] ** 2 + ax[1] ** 2 + ax[2] ** 2)
        ax = (ax[0] / n, ax[1] / n, ax[2] / n)
        return ax

    for center, tangent, mat, nm in bead_specs:
        ax = _rot_to_axis(None, tangent)
        disc = CylinderGeometry(0.011, 0.012, radial_segments=22).rotate_y(math.pi / 2.0)
        bead = model.part(nm)
        yaw = math.atan2(ax[1], ax[0])
        pitch = math.asin(max(-1.0, min(1.0, ax[2])))
        disc.rotate_y(-pitch)
        disc.rotate_z(yaw)
        bead.visual(mesh_from_geometry(disc, "bead_disc"), material=mat, name="bead_disc")
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
            rim=WheelRim(inner_radius=0.018, flange_height=0.004, flange_thickness=0.003),
            hub=WheelHub(radius=0.009, width=WHEEL_W * 0.6, cap_style="recessed"),
            face=WheelFace(dish_depth=0.004, front_inset=0.002),
            bore=WheelBore(style="round", diameter=0.006),
        )
        # WheelGeometry spins about local X; rotate so local +X aligns to world +Y.
        wheel.rotate_z(math.pi / 2.0)
        return mesh_from_geometry(wheel, "wheel_body")

    # wheel positions: (x, y_sign, name)
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
        # light-colored hub cap straddling the outer face (stays connected to rim).
        hub = CylinderGeometry(0.010, 0.008, radial_segments=20).rotate_x(math.pi / 2.0)
        hub.translate(0.0, ysign * (WHEEL_W / 2.0 - 0.002), 0.0)
        w.visual(mesh_from_geometry(hub, "hub_cap"), material=hub_light, name="hub_cap")
        # small off-axis marker dot so wheel rotation is observable.
        marker = CylinderGeometry(0.0022, 0.010, radial_segments=10).rotate_x(math.pi / 2.0)
        marker.translate(0.013, ysign * (WHEEL_W / 2.0 - 0.003), 0.0)
        w.visual(mesh_from_geometry(marker, "spin_marker"), material=black, name="spin_marker")

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

    # ---- racer hull tapers toward the pointed nose at front (+X) ----
    hull_aabb = ctx.part_element_world_aabb(body, elem="racer_hull")
    ctx.check(
        "racer hull Y span is narrower than a full-width block",
        hull_aabb is not None and (hull_aabb[1][1] - hull_aabb[0][1]) < 0.092,
        details=f"hull Y span={hull_aabb[1][1] - hull_aabb[0][1]:.4f}" if hull_aabb else "no hull",
    )

    # The front of the hull should be noticeably narrower than the rear.
    # We check the hull reaches forward to a pointed nose (max_x near body front).
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "hull nose reaches near the body front",
        hull_aabb is not None and hull_aabb[1][0] >= 0.065,
        details=f"hull max_x={hull_aabb[1][0]:.4f}" if hull_aabb else "no hull",
    )

    # ---- driver head sits up in the cockpit area ----
    driver_aabb = ctx.part_element_world_aabb(body, elem="driver_head")
    ctx.check(
        "driver head pokes up above the deck",
        driver_aabb is not None and driver_aabb[1][2] > BODY_TOP_Z,
        details=f"driver top_z={driver_aabb[1][2]:.4f}, deck={BODY_TOP_Z}" if driver_aabb else "no driver",
    )
    ctx.check(
        "driver head is near body center in X (cockpit area)",
        driver_aabb is not None
        and abs((driver_aabb[0][0] + driver_aabb[1][0]) / 2.0 - COCKPIT_X) < 0.020,
        details=f"driver center_x={(driver_aabb[0][0] + driver_aabb[1][0]) / 2.0:.4f}" if driver_aabb else "no driver",
    )

    # ---- wire arch rises above the body deck ----
    arch_aabb = ctx.part_world_aabb(arch)
    ctx.check(
        "wire arch rises above the wooden body",
        arch_aabb is not None and arch_aabb[1][2] > BODY_TOP_Z + 0.02,
        details=f"arch top_z={arch_aabb[1][2]:.4f}, body top={BODY_TOP_Z}",
    )
    # arch feet plug into the deck (intentional embed).
    ctx.allow_overlap(
        arch,
        body,
        elem_a="arch_wire",
        elem_b="racer_hull",
        reason="Wire-arch feet are intentionally seated into the wooden deck.",
    )
    ctx.expect_contact(arch, body, name="wire arch attached to body")

    # ---- car rests on 4 wheels (wheels reach the ground plane z~0) ----
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
            details=f"{nm} min_z={w_aabb[0][2]:.4f}" if w_aabb else f"no {nm}",
        )
        # each wheel hub bore is seated over its body axle peg (intentional capture).
        peg = "axle_0" if "front" in nm else "axle_1"
        ctx.allow_overlap(
            w,
            body,
            elem_a="wheel_body",
            elem_b=peg,
            reason="Wheel hub bore is intentionally seated over the body axle peg.",
        )

    # ---- all 4 wheels roll about their axles (marker sweeps in X/Z) ----
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
        # marker center should move when the wheel rolls about Y (changes X and Z)
        c0 = ((m0[0][0] + m0[1][0]) / 2.0, (m0[0][2] + m0[1][2]) / 2.0)
        c1 = ((m1[0][0] + m1[1][0]) / 2.0, (m1[0][2] + m1[1][2]) / 2.0)
        moved = math.hypot(c1[0] - c0[0], c1[1] - c0[1])
        ctx.check(
            f"{nm} rolls about its axle (marker moves)",
            moved > 0.008,
            details=f"{nm} marker moved={moved:.4f}",
        )

    # ---- beads are threaded on the wire (intentional disc-bore overlap) ----
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
    # spinning the disc about the wire changes its world AABB extents
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
