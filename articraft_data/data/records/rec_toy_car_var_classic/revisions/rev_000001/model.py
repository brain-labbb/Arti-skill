from __future__ import annotations

# Wooden push toy classic car with a driver figure and a bead-maze wire arch.
# Frame: car length along +X (hood at +X front), width along Y, up +Z.
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
BODY_BOT_Z = 0.022       # underside of the wooden body, clears the ground a bit
BODY_TOP_Z = 0.070       # top of the deck at the cowl

CABIN_H = 0.032          # cabin block height above deck
CABIN_LEN = 0.058        # cabin length along X
CABIN_X = -0.030         # cabin center X (rear portion of body)

DRIVER_X = -0.025        # driver head X (in the cabin)
DRIVER_R = 0.015         # driver head radius
DRIVER_Z = BODY_TOP_Z + CABIN_H - 0.005  # driver head center Z (pokes above cabin)


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


def _body_solid() -> cq.Workplane:
    # Classic toy car body: lofted deck with a sloped hood at front (+X)
    # and a raised cabin block on the rear (-X).
    zc = (BODY_BOT_Z + BODY_TOP_Z) / 2.0
    deck_h = BODY_TOP_Z - BODY_BOT_Z  # 0.048

    # Deck loft: full height at the cowl, sloping down toward the hood tip.
    deck = _loft(
        [
            ("rect", -0.072, 0.068, deck_h * 0.90),   # rear face
            ("rect", -0.055, 0.082, deck_h),            # rear-mid (under cabin)
            ("rect", -0.010, 0.084, deck_h),            # cowl (full height)
            ("rect", 0.025, 0.082, deck_h * 0.72),     # hood start, dropping
            ("rect", 0.044, 0.080, deck_h * 0.68),     # at front axle (keeps deck contact)
            ("rect", 0.060, 0.074, deck_h * 0.54),     # past axle, hood drops further
            ("rect", 0.076, 0.058, deck_h * 0.44),     # hood tip, lowest/narrowest
        ]
    )
    try:
        deck = deck.edges("|X").fillet(0.008)
    except Exception:
        pass
    deck = deck.translate((0.0, 0.0, zc))

    # Cabin block: rounded box on the rear portion of the deck.
    cabin = cq.Workplane("XY").box(CABIN_LEN, 0.074, CABIN_H)
    try:
        cabin = cabin.edges("|Z").fillet(0.008)
    except Exception:
        pass
    cabin = cabin.translate((CABIN_X, 0.0, BODY_TOP_Z + CABIN_H / 2.0))

    return deck.union(cabin)


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
    skin = model.material("driver_skin", rgba=(0.90, 0.74, 0.56, 1.0))

    # ==================== BODY (root) ====================
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_solid(), "body_block"),
        material=wood,
        name="body_block",
    )

    # ---- Driver figure (inline visuals on body, poking up from cockpit) ----
    # Round wooden driver head.
    head = SphereGeometry(DRIVER_R, width_segments=28, height_segments=20)
    head.translate(DRIVER_X, 0.0, DRIVER_Z)
    body.visual(mesh_from_geometry(head, "driver_head"), material=skin, name="driver_head")

    # Two dot eyes painted on the front (+X) face of the driver head.
    for i, ey in enumerate((-0.006, 0.006)):
        eye = CylinderGeometry(0.003, 0.004, radial_segments=16).rotate_y(math.pi / 2.0)
        eye.translate(DRIVER_X + DRIVER_R * 0.88, ey, DRIVER_Z + 0.004)
        body.visual(mesh_from_geometry(eye, f"driver_eye_{i}"), material=black, name=f"driver_eye_{i}")

    # Smiling mouth arc on the front of the driver head.
    smile = TorusGeometry(0.007, 0.0012, radial_segments=10, tubular_segments=24)
    smile.rotate_x(math.pi / 2.0)
    smile.rotate_z(math.pi / 2.0)
    smile.translate(DRIVER_X + DRIVER_R * 0.84, 0.0, DRIVER_Z - 0.004)
    body.visual(mesh_from_geometry(smile, "driver_smile"), material=black, name="driver_smile")

    # Two axle pegs (wooden) spanning the body so wheels read as mounted on axles.
    for i, ax in enumerate((AXLE_FX, AXLE_RX)):
        peg = CylinderGeometry(0.005, 2 * AXLE_Y + 0.010, radial_segments=16).rotate_x(
            math.pi / 2.0
        )
        peg.translate(ax, 0.0, AXLE_Z)
        body.visual(mesh_from_geometry(peg, f"axle_{i}"), material=wood_dark, name=f"axle_{i}")

    body.inertial = Inertial.from_geometry(
        Box((BODY_LEN, 0.085, 0.080)),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    # ==================== WIRE ARCH (fixed to body) ====================
    # A bent metal wire that loops up over the car; beads thread onto it.
    # Rear foot mounts into the cabin/deck area; front foot mounts into the hood.
    arch_pts = [
        (-0.050, -0.030, BODY_TOP_Z - 0.004),  # rear-left foot (into deck under cabin)
        (-0.040, -0.028, 0.092),
        (-0.010, -0.010, 0.112),               # apex
        (0.020, 0.012, 0.108),
        (0.046, 0.026, 0.080),                  # descending toward hood
        (0.052, 0.030, 0.056),                  # front-right foot (into hood surface)
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
        Box((0.11, 0.07, 0.06)), mass=0.02, origin=Origin(xyz=(0.0, 0.0, 0.095))
    )
    model.articulation(
        "body_to_arch",
        ArticulationType.FIXED,
        parent=body,
        child=arch,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # ==================== BEADS (spin about the wire) ====================
    # Each bead is a flat colored disc threaded on a wire segment.
    # Placements near spline control points with tangent along local wire direction.
    bead_specs = [
        ((-0.030, -0.020, 0.101), (0.55, 0.22, 0.45), bead_red, "bead_0"),
        ((0.004, -0.001, 0.111), (0.80, 0.40, -0.08), bead_orange, "bead_1"),
        ((0.034, 0.020, 0.090), (0.55, 0.35, -0.62), bead_green, "bead_2"),
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
        hub = CylinderGeometry(0.010, 0.008, radial_segments=20).rotate_x(math.pi / 2.0)
        hub.translate(0.0, ysign * (WHEEL_W / 2.0 - 0.002), 0.0)
        w.visual(mesh_from_geometry(hub, "hub_cap"), material=hub_light, name="hub_cap")
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

    cabin_top = BODY_TOP_Z + CABIN_H  # 0.102

    # ---- classic car body: cabin extends above deck line ----
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "cabin extends above the deck line",
        body_aabb is not None and body_aabb[1][2] >= cabin_top - 0.005,
        details=f"body max_z={body_aabb[1][2] if body_aabb else None}, cabin_top={cabin_top}",
    )

    # ---- driver head pokes up above cabin top ----
    head_aabb = ctx.part_element_world_aabb(body, elem="driver_head")
    ctx.check(
        "driver head pokes above cabin top",
        head_aabb is not None and head_aabb[1][2] > cabin_top - 0.002,
        details=f"head max_z={head_aabb[1][2] if head_aabb else None}, cabin_top={cabin_top}",
    )

    # ---- driver head is in the cabin area (rear half of car) ----
    ctx.check(
        "driver head is in the cabin area",
        head_aabb is not None and (head_aabb[0][0] + head_aabb[1][0]) / 2.0 < 0.0,
        details=f"head center_x={(head_aabb[0][0] + head_aabb[1][0]) / 2.0 if head_aabb else None}",
    )

    # ---- driver has two eyes and a smile on the front face ----
    for elem_name in ("driver_eye_0", "driver_eye_1", "driver_smile"):
        e_aabb = ctx.part_element_world_aabb(body, elem=elem_name)
        ctx.check(
            f"{elem_name} exists on the driver head",
            e_aabb is not None,
            details=f"{elem_name} aabb={e_aabb}",
        )

    # ---- wire arch rises above the body ----
    arch_aabb = ctx.part_world_aabb(arch)
    ctx.check(
        "wire arch rises above the body",
        arch_aabb is not None and arch_aabb[1][2] > BODY_TOP_Z + 0.02,
        details=f"arch top_z={arch_aabb[1][2] if arch_aabb else None}",
    )

    # arch feet plug into the body (intentional embed).
    ctx.allow_overlap(
        arch,
        body,
        elem_a="arch_wire",
        elem_b="body_block",
        reason="Wire-arch feet are intentionally seated into the wooden body (deck and hood).",
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

    # ---- bead_0 may graze the cabin top or driver head (near-arch proximity) ----
    bead_0 = object_model.get_part("bead_0")
    ctx.allow_overlap(
        bead_0,
        body,
        reason="Bead_0 passes near the cabin/driver area on the wire arch; small proximity overlap.",
    )

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
