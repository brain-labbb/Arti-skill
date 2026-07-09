from __future__ import annotations

# Wooden push toy car with a smiling bug face and a rooftop spinner pinwheel.
# Frame: car length along +X (bug face at +X front), width along Y, up +Z.
#   Body centerline at z so the car rests on four red wheels (ground z=0).
# Articulations:
#   - 4 wheels: each CONTINUOUS roll about its Y axle.
#   - spinner disc: CONTINUOUS revolute about the vertical post axis (Z).

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
BODY_TOP_Z = 0.070       # top of the chunky wood deck

# Spinner dimensions
POST_R = 0.006           # post radius
POST_H = 0.038           # post height above collar
COLLAR_R = 0.010         # collar radius (wider base ring)
COLLAR_H = 0.006         # collar height
POST_X = -0.010          # slightly rear of deck center
POST_Y = 0.0             # centered laterally
POST_TOP_Z = BODY_TOP_Z + COLLAR_H + POST_H  # top of post (joint origin Z)

DISC_HUB_R = 0.010       # central hub radius
DISC_HUB_H = 0.006       # central hub height
BLADE_LEN = 0.030        # blade length (radial extent of each paddle)
BLADE_W = 0.015          # blade width (tangential)
BLADE_T = 0.004          # blade thickness
BLADE_REACH = 0.022      # blade center distance from disc axis
NUM_BLADES = 4


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


def _spinner_disc_solid() -> cq.Workplane:
    """Pinwheel disc: central hub + radiating paddle blades, centered at z=0."""
    # Central hub cylinder, centered at origin
    hub = (
        cq.Workplane("XY")
        .circle(DISC_HUB_R)
        .extrude(DISC_HUB_H)
        .translate((0, 0, -DISC_HUB_H / 2.0))
    )
    # Four paddle blades radiating outward, each overlapping the hub for union
    for i in range(NUM_BLADES):
        angle_deg = i * (360.0 / NUM_BLADES)
        blade = (
            cq.Workplane("XY")
            .transformed(rotate=(0, 0, angle_deg))
            .transformed(offset=(BLADE_REACH, 0, 0))
            .box(BLADE_LEN, BLADE_W, BLADE_T)
        )
        hub = hub.union(blade)
    # Soften blade outer edges for a toy-like feel
    try:
        hub = hub.edges("|Z").fillet(0.0015)
    except Exception:
        pass
    return hub


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wooden_push_toy_car")

    wood = model.material("natural_wood", rgba=(0.80, 0.62, 0.36, 1.0))
    wood_dark = model.material("wood_dark", rgba=(0.66, 0.47, 0.26, 1.0))
    red = model.material("toy_red", rgba=(0.85, 0.13, 0.12, 1.0))
    hub_light = model.material("hub_light", rgba=(0.88, 0.86, 0.82, 1.0))
    black = model.material("face_black", rgba=(0.10, 0.10, 0.11, 1.0))
    # Spinner blade colors (colorful toy pinwheel)
    blade_red = model.material("blade_red", rgba=(0.88, 0.18, 0.16, 1.0))
    blade_blue = model.material("blade_blue", rgba=(0.20, 0.40, 0.80, 1.0))
    blade_yellow = model.material("blade_yellow", rgba=(0.93, 0.80, 0.20, 1.0))
    blade_green = model.material("blade_green", rgba=(0.30, 0.66, 0.26, 1.0))
    blade_colors = [blade_red, blade_blue, blade_yellow, blade_green]

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

    # ==================== SPINNER POST (fixed decoration on body deck) ====================
    # Collar ring at deck surface (visible mounting base)
    collar = CylinderGeometry(COLLAR_R, COLLAR_H, radial_segments=22)
    collar.translate(POST_X, POST_Y, BODY_TOP_Z + COLLAR_H / 2.0)
    body.visual(mesh_from_geometry(collar, "post_collar"), material=wood_dark, name="post_collar")

    # Short wooden post rising from collar
    post = CylinderGeometry(POST_R, POST_H, radial_segments=16)
    post.translate(POST_X, POST_Y, BODY_TOP_Z + COLLAR_H + POST_H / 2.0)
    body.visual(mesh_from_geometry(post, "spinner_post"), material=wood_dark, name="spinner_post")

    body.inertial = Inertial.from_geometry(
        Box((BODY_LEN, 0.085, 0.050)),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, (BODY_BOT_Z + BODY_TOP_Z) / 2.0)),
    )

    # ==================== SPINNER DISC (continuous revolute about Z) ====================
    spinner = model.part("spinner_disc")

    # Pinwheel disc body (CadQuery: hub + 4 paddle blades)
    spinner.visual(
        mesh_from_cadquery(_spinner_disc_solid(), "disc_body"),
        material=wood,
        name="disc_body",
    )

    # Colored paddle dots on each blade (rotation markers and color variety)
    for i in range(NUM_BLADES):
        angle = i * (2.0 * math.pi / NUM_BLADES)
        mx = BLADE_REACH * math.cos(angle)
        my = BLADE_REACH * math.sin(angle)
        dot = CylinderGeometry(0.004, 0.005, radial_segments=14)
        dot.translate(mx, my, BLADE_T / 2.0 + 0.0025)
        spinner.visual(
            mesh_from_geometry(dot, f"blade_dot_{i}"),
            material=blade_colors[i],
            name=f"blade_dot_{i}",
        )

    # Dark bore cap on hub top (suggests post passes through)
    bore_cap = CylinderGeometry(0.007, 0.002, radial_segments=18)
    bore_cap.translate(0.0, 0.0, DISC_HUB_H / 2.0 + 0.001)
    spinner.visual(
        mesh_from_geometry(bore_cap, "hub_bore_cap"),
        material=black,
        name="hub_bore_cap",
    )

    spinner.inertial = Inertial.from_geometry(
        Cylinder(0.048, 0.008), mass=0.015
    )

    # Continuous revolute joint: spinner spins about vertical post axis.
    # Joint origin at post top contact surface.
    model.articulation(
        "body_to_spinner",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=spinner,
        origin=Origin(xyz=(POST_X, POST_Y, POST_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=15.0),
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
    spinner = object_model.get_part("spinner_disc")

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

    # ---- spinner post rises above the body deck ----
    post_aabb = ctx.part_element_world_aabb(body, elem="spinner_post")
    ctx.check(
        "spinner post rises above the wooden body deck",
        post_aabb is not None and post_aabb[1][2] > BODY_TOP_Z + 0.02,
        details=f"post top_z={post_aabb[1][2] if post_aabb else None}, deck top={BODY_TOP_Z}",
    )

    # ---- spinner disc sits at/above the post top ----
    disc_aabb = ctx.part_world_aabb(spinner)
    ctx.check(
        "spinner disc sits at the post top",
        disc_aabb is not None and disc_aabb[0][2] >= POST_TOP_Z - DISC_HUB_H / 2.0 - 0.002,
        details=f"disc min_z={disc_aabb[0][2] if disc_aabb else None}, post_top={POST_TOP_Z}",
    )

    # ---- spinner hub seated on post (intentional small overlap) ----
    ctx.allow_overlap(
        spinner,
        body,
        elem_a="disc_body",
        elem_b="spinner_post",
        reason="Spinner disc hub is intentionally seated around the post top for a realistic pivot mount.",
    )
    ctx.expect_contact(
        spinner,
        body,
        elem_a="disc_body",
        elem_b="spinner_post",
        name="spinner disc contacts the post top",
    )

    # ---- spinner rotates about the vertical axis (blade dot sweeps in XY) ----
    spinner_joint = object_model.get_articulation("body_to_spinner")
    d0 = ctx.part_element_world_aabb(spinner, elem="blade_dot_0")
    with ctx.pose({spinner_joint: math.pi / 2.0}):
        d1 = ctx.part_element_world_aabb(spinner, elem="blade_dot_0")
    c0 = ((d0[0][0] + d0[1][0]) / 2.0, (d0[0][1] + d0[1][1]) / 2.0)
    c1 = ((d1[0][0] + d1[1][0]) / 2.0, (d1[0][1] + d1[1][1]) / 2.0)
    moved = math.hypot(c1[0] - c0[0], c1[1] - c0[1])
    ctx.check(
        "spinner disc rotates about the post (blade dot moves in XY)",
        moved > 0.010,
        details=f"rest_xy={c0}, spun_xy={c1}, moved={moved:.4f}",
    )

    # ---- spinner disc has colorful blade markers ----
    for i in range(NUM_BLADES):
        dot_aabb = ctx.part_element_world_aabb(spinner, elem=f"blade_dot_{i}")
        ctx.check(
            f"blade_dot_{i} exists on the disc",
            dot_aabb is not None,
            details=f"blade_dot_{i} AABB is None",
        )

    # ---- car rests on 4 wheels (wheels reach the ground plane z~0) ----
    wheel_names = (
        "wheel_front_left",
        "wheel_front_right",
        "wheel_rear_left",
        "wheel_rear_right",
    )
    for nm in wheel_names:
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
    for nm in wheel_names:
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

    return ctx.report()


object_model = build_object_model()
