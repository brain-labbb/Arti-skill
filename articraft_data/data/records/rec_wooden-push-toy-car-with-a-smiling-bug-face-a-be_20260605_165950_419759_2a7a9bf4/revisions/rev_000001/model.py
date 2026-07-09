from __future__ import annotations

# Wooden push toy car with a smiling bug face and a bead-maze wire arch.
# Frame: car length along +X (bug face at +X front), width along Y, up +Z.
#   Body centerline at z so the car rests on four red wheels (ground z=0).
# Articulations:
#   - 4 wheels: each CONTINUOUS roll about its Y axle.
#   - 3 beads on the wire arch: each CONTINUOUS spin about its local wire segment.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
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
BODY_TOP_Z = 0.070       # top of the chunky wood deck


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
    # Chunky natural-wood car body: a rounded block that tapers up toward the
    # nose, with the deck centered around z = (BODY_BOT_Z+BODY_TOP_Z)/2.
    zc = (BODY_BOT_Z + BODY_TOP_Z) / 2.0
    h = BODY_TOP_Z - BODY_BOT_Z
    body = _loft(
        [
            ("rect", -0.072, 0.066, h * 0.86),   # rear face (slightly tucked)
            ("rect", -0.060, 0.080, h),
            ("rect", 0.020, 0.084, h),
            ("rect", 0.058, 0.080, h * 0.92),
            ("rect", 0.078, 0.060, h * 0.74),    # front, narrows toward the bug
        ]
    )
    # round the long horizontal edges before translating
    try:
        body = body.edges("|X").fillet(0.009)
    except Exception:
        pass
    # lift loft (built centered at z=0) up to the deck height
    body = body.translate((0.0, 0.0, zc))
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wooden_push_toy_car")

    wood = model.material("natural_wood", rgba=(0.80, 0.62, 0.36, 1.0))
    wood_dark = model.material("wood_dark", rgba=(0.66, 0.47, 0.26, 1.0))
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
        mesh_from_cadquery(_body_solid(), "body_block"),
        material=wood,
        name="body_block",
    )

    # Rounded wooden bug HEAD on the front, sitting up off the deck.
    head_x = 0.082
    head_z = 0.060
    head_r = 0.026
    head = SphereGeometry(head_r, width_segments=32, height_segments=24)
    head.translate(head_x, 0.0, head_z)
    body.visual(mesh_from_geometry(head, "bug_head"), material=wood, name="bug_head")

    # Two eyes (small black discs on the front face of the head).
    for i, ey in enumerate((-0.010, 0.010)):
        eye = CylinderGeometry(0.0045, 0.006, radial_segments=18).rotate_y(math.pi / 2.0)
        eye.translate(head_x + 0.022, ey, head_z + 0.006)
        body.visual(mesh_from_geometry(eye, f"eye_{i}"), material=black, name=f"eye_{i}")

    # Smiling mouth: a thin black arc (partial torus) on the lower front of head.
    smile = TorusGeometry(0.011, 0.0016, radial_segments=10, tubular_segments=28)
    # keep the bottom half of the ring -> a smile; rotate so the ring faces +X.
    smile.rotate_x(math.pi / 2.0)        # ring now in XZ-ish plane
    smile.rotate_z(math.pi / 2.0)
    smile.translate(head_x + 0.0215, 0.0, head_z - 0.004)
    body.visual(mesh_from_geometry(smile, "smile"), material=black, name="smile")

    # Small red nose (round button) below the eyes, in front of the head.
    nose = SphereGeometry(0.007, width_segments=20, height_segments=14)
    nose.translate(head_x + 0.0205, 0.0, head_z - 0.006)
    body.visual(mesh_from_geometry(nose, "nose"), material=red, name="nose")

    # Two short antennae rising from the top of the head (wire + ball tip).
    for i, ay in enumerate((-0.011, 0.011)):
        stalk = tube_from_spline_points(
            [
                (head_x - 0.004 * (1 - 2 * i), ay * 0.6, head_z + 0.020),
                (head_x - 0.006, ay, head_z + 0.034),
                (head_x - 0.012, ay * 1.3, head_z + 0.044),
            ],
            radius=0.0013,
            samples_per_segment=12,
            radial_segments=10,
        )
        tip = SphereGeometry(0.0035, width_segments=14, height_segments=10)
        tip.translate(head_x - 0.012, ay * 1.3, head_z + 0.044)
        stalk.merge(tip)
        body.visual(mesh_from_geometry(stalk, f"antenna_{i}"), material=black, name=f"antenna_{i}")

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
    # Path goes from a mount near the rear-left, arcs up over the top, and
    # comes back down to a mount near the front-right.
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
    # Each bead is a flat colored disc threaded on a wire segment. We place it
    # at a sample point on the arch and orient its rotation axis along the local
    # wire tangent so it spins around the wire.
    def _tangent(pts, t):
        # central difference of catmull-ish sampling: use neighboring path pts.
        return None

    # Hand-picked bead placements: (center, tangent_dir, material, name)
    bead_specs = [
        ((-0.030, -0.020, 0.101), (0.55, 0.22, 0.45), bead_red, "bead_0"),
        ((0.004, -0.001, 0.111), (0.80, 0.40, -0.08), bead_orange, "bead_1"),
        ((0.034, 0.020, 0.097), (0.55, 0.35, -0.62), bead_green, "bead_2"),
    ]

    def _rot_to_axis(geom_local, axis):
        # geom_local is built spinning about its local +X (cylinder rotated to X).
        # Rotate it so local +X aligns to `axis`, then return rotation that maps.
        ax = axis
        n = math.sqrt(ax[0] ** 2 + ax[1] ** 2 + ax[2] ** 2)
        ax = (ax[0] / n, ax[1] / n, ax[2] / n)
        return ax

    for center, tangent, mat, nm in bead_specs:
        ax = _rot_to_axis(None, tangent)
        # Build a chunky flat disc (bead) whose spin axis is +X locally.
        disc = CylinderGeometry(0.011, 0.012, radial_segments=22).rotate_y(math.pi / 2.0)
        # central hole channel suggestion: a thin dark ring on the rim
        bead = model.part(nm)
        # Orient disc's local +X onto the world tangent via two rotations.
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

    # ---- bug face is on the front (+X) ----
    head_aabb = ctx.part_element_world_aabb(body, elem="bug_head")
    nose_aabb = ctx.part_element_world_aabb(body, elem="nose")
    body_aabb = ctx.part_world_aabb(body)
    # head sits in the front quarter of the car (forward of body center).
    ctx.check(
        "bug head is on the front of the car",
        head_aabb is not None and (head_aabb[0][0] + head_aabb[1][0]) / 2.0 > 0.045,
        details=f"head center_x={(head_aabb[0][0] + head_aabb[1][0]) / 2.0 if head_aabb else None}",
    )
    # red nose is the frontmost facial feature, reaching near the body front.
    ctx.check(
        "red nose sits at the very front",
        nose_aabb is not None and nose_aabb[1][0] >= body_aabb[1][0] - 0.012,
        details=f"nose max_x={nose_aabb[1][0] if nose_aabb else None}, body max_x={body_aabb[1][0]}",
    )

    # ---- wire arch rises above the body deck ----
    arch_aabb = ctx.part_world_aabb(arch)
    ctx.check(
        "wire arch rises above the wooden body",
        arch_aabb is not None and arch_aabb[1][2] > BODY_TOP_Z + 0.02,
        details=f"arch top_z={arch_aabb[1][2] if arch_aabb else None}, body top={BODY_TOP_Z}",
    )
    # arch is fixed to the body; its feet plug into the deck (intentional embed).
    ctx.allow_overlap(
        arch,
        body,
        elem_a="arch_wire",
        elem_b="body_block",
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
            details=f"{nm} min_z={w_aabb[0][2] if w_aabb else None}",
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
