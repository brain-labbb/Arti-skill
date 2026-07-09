from __future__ import annotations

# Wooden push toy car with a smiling bug face and a chunky steering wheel
# on a tilted column (replaces the bead-maze arch from the parent variant).
# Frame: car length along +X (bug face at +X front), width along Y, up +Z.
#   Body centerline at z so the car rests on four red wheels (ground z=0).
# Articulations:
#   - 4 wheels: each CONTINUOUS roll about its Y axle.
#   - steering wheel: CONTINUOUS spin about the tilted column axis.

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

# ---- steering column ----
TILT = math.radians(30)           # column tilt back from vertical
COL_BASE_X = 0.020                # column base X on deck
COL_LEN = 0.042                   # column length along axis
COL_R = 0.006                     # column post radius
BOSS_R = 0.010                    # column boss radius on deck
BOSS_H = 0.005                    # column boss height

_COS_T = math.cos(TILT)
_SIN_T = math.sin(TILT)
COL_TOP_X = COL_BASE_X - _SIN_T * COL_LEN    # top of column X
COL_TOP_Z = BODY_TOP_Z + _COS_T * COL_LEN    # top of column Z
COL_MID_X = (COL_BASE_X + COL_TOP_X) / 2.0
COL_MID_Z = (BODY_TOP_Z + COL_TOP_Z) / 2.0


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


def _steering_wheel_mesh():
    """Chunky wooden steering wheel: rim + hub + 4 spokes, flat in XY, spin about Z."""
    rim_r = 0.026       # rim center radius
    tube_r = 0.004      # rim tube radius
    hub_r = 0.007       # hub radius
    hub_h = 0.010       # hub height (along Z)
    spoke_r = 0.003     # spoke cross-section radius
    spoke_len = rim_r - hub_r            # 0.019
    spoke_offset = hub_r + spoke_len / 2.0  # center of spoke from origin

    # Rim: torus ring in XY plane (around Z axis)
    wheel = TorusGeometry(rim_r, tube_r, radial_segments=14, tubular_segments=32)

    # Hub: short cylinder along Z at center
    hub = CylinderGeometry(hub_r, hub_h, radial_segments=18)
    wheel.merge(hub)

    # 4 spokes radiating from hub to rim in XY plane
    for i in range(4):
        angle = i * math.pi / 2.0
        spoke = CylinderGeometry(spoke_r, spoke_len, radial_segments=10)
        spoke.rotate_y(math.pi / 2.0)    # align along X
        spoke.rotate_z(angle)             # rotate to i-th direction
        dx = spoke_offset * math.cos(angle)
        dy = spoke_offset * math.sin(angle)
        spoke.translate(dx, dy, 0.0)
        wheel.merge(spoke)

    return mesh_from_geometry(wheel, "steering_wheel_body")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wooden_push_toy_car")

    wood = model.material("natural_wood", rgba=(0.80, 0.62, 0.36, 1.0))
    wood_dark = model.material("wood_dark", rgba=(0.66, 0.47, 0.26, 1.0))
    red = model.material("toy_red", rgba=(0.85, 0.13, 0.12, 1.0))
    hub_light = model.material("hub_light", rgba=(0.88, 0.86, 0.82, 1.0))
    black = model.material("face_black", rgba=(0.10, 0.10, 0.11, 1.0))
    steer_wood = model.material("steer_wood", rgba=(0.74, 0.52, 0.28, 1.0))

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

    # ---- Steering column (inline body visuals) ----
    # Column boss: raised ring on deck where column emerges
    boss = CylinderGeometry(BOSS_R, BOSS_H, radial_segments=18)
    boss.translate(COL_BASE_X, 0.0, BODY_TOP_Z + BOSS_H / 2.0)
    body.visual(mesh_from_geometry(boss, "column_boss"), material=wood_dark, name="column_boss")

    # Column post: tilted wooden cylinder from deck to wheel
    post = CylinderGeometry(COL_R, COL_LEN, radial_segments=14)
    post.rotate_y(-TILT)
    post.translate(COL_MID_X, 0.0, COL_MID_Z)
    body.visual(mesh_from_geometry(post, "column_post"), material=wood_dark, name="column_post")

    body.inertial = Inertial.from_geometry(
        Box((BODY_LEN, 0.085, 0.050)),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, (BODY_BOT_Z + BODY_TOP_Z) / 2.0)),
    )

    # ==================== STEERING WHEEL ====================
    sw = model.part("steering_wheel")
    sw.visual(
        _steering_wheel_mesh(),
        material=steer_wood,
        name="steering_wheel_body",
    )
    # Marker dot on the rim (+X side) so rotation is visually observable
    marker = CylinderGeometry(0.003, 0.006, radial_segments=10)
    marker.translate(0.026, 0.0, 0.0)
    sw.visual(mesh_from_geometry(marker, "steer_marker"), material=red, name="steer_marker")

    sw.inertial = Inertial.from_geometry(
        Cylinder(0.056, 0.012), mass=0.015,
    )

    # Continuous revolute about the tilted column axis.
    # Joint frame: origin at column top, rpy tilts local Z to match column direction.
    # Axis (0,0,1) in joint frame = column direction in world.
    model.articulation(
        "body_to_steering",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=sw,
        origin=Origin(xyz=(COL_TOP_X, 0.0, COL_TOP_Z), rpy=(0.0, -TILT, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=10.0),
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
    sw = object_model.get_part("steering_wheel")

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

    # ---- steering column rises above the deck ----
    post_aabb = ctx.part_element_world_aabb(body, elem="column_post")
    ctx.check(
        "steering column post rises above the deck",
        post_aabb is not None and post_aabb[1][2] > BODY_TOP_Z + 0.02,
        details=f"post top_z={post_aabb[1][2] if post_aabb else None}, deck top={BODY_TOP_Z}",
    )
    boss_aabb = ctx.part_element_world_aabb(body, elem="column_boss")
    ctx.check(
        "column boss sits on the deck",
        boss_aabb is not None and boss_aabb[0][2] <= BODY_TOP_Z + 0.006,
        details=f"boss min_z={boss_aabb[0][2] if boss_aabb else None}",
    )

    # ---- steering wheel is above the deck, mounted on the column ----
    sw_aabb = ctx.part_world_aabb(sw)
    ctx.check(
        "steering wheel is above the deck",
        sw_aabb is not None and sw_aabb[0][2] > BODY_TOP_Z + 0.01,
        details=f"wheel min_z={sw_aabb[0][2] if sw_aabb else None}",
    )
    # Hub is intentionally seated onto the column post top (small local embed).
    ctx.allow_overlap(
        sw,
        body,
        elem_a="steering_wheel_body",
        elem_b="column_post",
        reason="Steering wheel hub is intentionally seated onto the column post top.",
    )
    ctx.expect_contact(
        sw, body,
        elem_a="steering_wheel_body",
        elem_b="column_post",
        name="steering wheel contacts column post",
    )

    # ---- steering wheel turns about the column axis (marker moves) ----
    steer_joint = object_model.get_articulation("body_to_steering")
    m0 = ctx.part_element_world_aabb(sw, elem="steer_marker")
    with ctx.pose({steer_joint: math.pi / 2.0}):
        m1 = ctx.part_element_world_aabb(sw, elem="steer_marker")
    c0 = ((m0[0][0] + m0[1][0]) / 2.0, (m0[0][1] + m0[1][1]) / 2.0, (m0[0][2] + m0[1][2]) / 2.0)
    c1 = ((m1[0][0] + m1[1][0]) / 2.0, (m1[0][1] + m1[1][1]) / 2.0, (m1[0][2] + m1[1][2]) / 2.0)
    moved = math.sqrt((c1[0] - c0[0])**2 + (c1[1] - c0[1])**2 + (c1[2] - c0[2])**2)
    ctx.check(
        "steering wheel turns about column axis (marker moves)",
        moved > 0.010,
        details=f"rest_center={c0}, turned_center={c1}, moved={moved:.4f}",
    )

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

    return ctx.report()


object_model = build_object_model()
