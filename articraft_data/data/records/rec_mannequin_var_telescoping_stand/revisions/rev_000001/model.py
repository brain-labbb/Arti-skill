from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    LoftGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

BLACK = "mannequin_black"
STEEL = "stand_steel"
GLASS = "base_glass"


# ----------------------------------------------------------------------
# Mesh helpers for organic, tapered, sculpted body forms.
# ----------------------------------------------------------------------
def lathe_mesh(profile: list[tuple[float, float]], name: str, segments: int = 28):
    """Revolve a (radius, z) profile around the z-axis. Profiles start and end
    on the axis (radius 0) so the solid is watertight with rounded/tapered caps."""
    return mesh_from_geometry(LatheGeometry(profile, segments=segments), name)


def loft_mesh(rings: list[list[tuple[float, float, float]]], name: str):
    return mesh_from_geometry(LoftGeometry(rings, cap=True, closed=True), name)


def ring_xy(z: float, a: float, b: float, n: int = 28, e: float = 2.4) -> list[tuple[float, float, float]]:
    """Superellipse (rounded-rectangle) ring in the X-Y plane at height z.
    a = half-width (x), b = half-depth (y), e = corner sharpness."""
    pts = []
    for i in range(n):
        t = 2.0 * math.pi * i / n
        c, s = math.cos(t), math.sin(t)
        x = a * (abs(c) ** (2.0 / e)) * (1.0 if c >= 0 else -1.0)
        y = b * (abs(s) ** (2.0 / e)) * (1.0 if s >= 0 else -1.0)
        pts.append((x, y, z))
    return pts


def ring_foot(zc: float, xh: float, ry: float, sole: float = 0.045, n: int = 24) -> list[tuple[float, float, float]]:
    """Foot cross-section authored in the X-Y plane at constant z (=foot length).
    The sole is pinned at a constant y so the lofted sole stays flat; the whole
    mesh is laid toe-forward and sole-down by an rpy origin rotation at placement.
    xh = half-width, ry = half-thickness."""
    cy = sole - ry
    pts = []
    for i in range(n):
        t = 2.0 * math.pi * i / n
        pts.append((xh * math.cos(t), cy + ry * math.sin(t), zc))
    return pts


def build_object_model() -> ArticulatedObject:
    """Matte-black poseable artist mannequin (ref 003.png), sculpted with organic
    tapered limbs, an anatomical lofted torso, an ovoid head and contoured feet.
    Leans on a wheeled steel stand (square glass plate + 4 casters + rear lean-rod).
    Every articulation retains a visible ball joint like the real poseable figure."""
    model = ArticulatedObject(name="poseable_drawing_mannequin_003")
    model.material(BLACK, rgba=(0.06, 0.06, 0.07, 1.0))
    model.material(STEEL, rgba=(0.60, 0.60, 0.63, 1.0))
    model.material(GLASS, rgba=(0.10, 0.10, 0.12, 1.0))

    # ------------------------------------------------------------------
    # Base: square rolling glass plate + 4 corner casters + rear lean-rod.
    # ------------------------------------------------------------------
    base = model.part("base")
    base.visual(
        Box((0.40, 0.40, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=GLASS,
        name="base_plate",
    )
    for i, (cx, cy) in enumerate(
        [(0.16, 0.16), (0.16, -0.16), (-0.16, 0.16), (-0.16, -0.16)]
    ):
        base.visual(
            Cylinder(radius=0.020, length=0.028),
            origin=Origin(xyz=(cx, cy, 0.036)),
            material=STEEL,
            name=f"caster_yoke_{i}",
        )
        base.visual(
            Cylinder(radius=0.016, length=0.024),
            origin=Origin(xyz=(cx, cy, 0.014), rpy=(math.pi / 2, 0.0, 0.0)),
            material=STEEL,
            name=f"caster_{i}",
        )
    # Outer sleeve (fixed to base): wider hollow tube from base plate upward.
    base.visual(
        Cylinder(radius=0.015, length=0.63),
        origin=Origin(xyz=(0.0, -0.155, 0.385)),
        material=STEEL,
        name="outer_sleeve",
    )

    # Inner rod + saddle: height-adjustable telescoping member.
    height_rod = model.part("height_rod")
    height_rod.visual(
        Cylinder(radius=0.009, length=0.895),
        origin=Origin(xyz=(0.0, 0.0, -0.103)),
        material=STEEL,
        name="inner_rod_member",
    )
    height_rod.visual(
        Box((0.06, 0.13, 0.03)),
        origin=Origin(xyz=(0.0, 0.045, 0.345)),
        material=STEEL,
        name="lean_saddle",
    )
    model.articulation(
        "sleeve_to_rod",
        ArticulationType.PRISMATIC,
        parent=base,
        child=height_rod,
        origin=Origin(xyz=(0.0, -0.155, 0.70)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.5, lower=0.0, upper=0.30),
    )

    # ------------------------------------------------------------------
    # Torso: three blended lofted regions (pelvis / waist / chest) that share
    # cross-sections so they read as one sculpted body, plus a tapered neck.
    # ------------------------------------------------------------------
    torso = model.part("torso")
    torso.visual(
        loft_mesh([
            ring_xy(1.00, 0.115, 0.090),
            ring_xy(1.06, 0.145, 0.105),  # hip flare (widest)
            ring_xy(1.13, 0.125, 0.095),
        ], "torso_pelvis"),
        material=BLACK,
        name="pelvis",
    )
    torso.visual(
        loft_mesh([
            ring_xy(1.12, 0.125, 0.095),
            ring_xy(1.22, 0.108, 0.083),  # narrow waist
            ring_xy(1.33, 0.138, 0.097),
        ], "torso_waist"),
        material=BLACK,
        name="waist",
    )
    torso.visual(
        loft_mesh([
            ring_xy(1.32, 0.130, 0.093),
            ring_xy(1.40, 0.140, 0.106),  # pectoral swell (deeper)
            ring_xy(1.48, 0.132, 0.095),  # upper chest
            ring_xy(1.55, 0.098, 0.082),  # taper toward neck
            ring_xy(1.60, 0.068, 0.064),
        ], "torso_chest"),
        material=BLACK,
        name="chest",
    )
    # Deltoid / clavicle caps: extend the shoulders outboard so the arms socket
    # into a proper shoulder yoke instead of clipping the ribcage.
    torso.visual(
        Sphere(radius=0.055),
        origin=Origin(xyz=(0.155, 0.0, 1.50)),
        material=BLACK,
        name="deltoid_left",
    )
    torso.visual(
        Sphere(radius=0.055),
        origin=Origin(xyz=(-0.155, 0.0, 1.50)),
        material=BLACK,
        name="deltoid_right",
    )
    torso.visual(
        lathe_mesh([
            (0.000, 0.000), (0.055, 0.006), (0.050, 0.050),
            (0.046, 0.095), (0.040, 0.108), (0.000, 0.112),
        ], "torso_neck"),
        origin=Origin(xyz=(0.0, 0.0, 1.52)),
        material=BLACK,
        name="neck",
    )
    model.articulation(
        "base_to_torso",
        ArticulationType.FIXED,
        parent=base,
        child=torso,
    )

    # ------------------------------------------------------------------
    # Head: smooth featureless ovoid + a jaw/chin lobe that seats over the neck.
    # ------------------------------------------------------------------
    head = model.part("head")
    head.visual(
        lathe_mesh([
            (0.000, 0.000), (0.052, 0.012), (0.086, 0.045),
            (0.100, 0.090), (0.099, 0.125), (0.086, 0.165),
            (0.055, 0.200), (0.000, 0.212),
        ], "head_ovoid"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=BLACK,
        name="head_shell",
    )
    head.visual(
        Sphere(radius=0.060),
        origin=Origin(xyz=(0.0, 0.025, 0.052)),
        material=BLACK,
        name="head_chin",
    )
    model.articulation(
        "neck_joint",
        ArticulationType.REVOLUTE,
        parent=torso,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 1.63)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=-0.8, upper=0.8),
    )

    # ------------------------------------------------------------------
    # Arms: shoulder / elbow / wrist chains, mirrored. Rounded deltoid ball +
    # tapered bicep/forearm lathes + a flattened mitten hand.
    # ------------------------------------------------------------------
    for side, sx in (("left", 1.0), ("right", -1.0)):
        upper = model.part(f"{side}_upper_arm")
        upper.visual(
            Sphere(radius=0.050),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=BLACK,
            name="shoulder_ball",
        )
        upper.visual(
            lathe_mesh([
                (0.000, 0.000), (0.035, -0.006), (0.048, -0.035),
                (0.050, -0.080), (0.046, -0.150), (0.040, -0.240),
                (0.036, -0.285), (0.000, -0.292),
            ], f"{side}_upper_arm_seg"),
            material=BLACK,
            name="upper_arm_seg",
        )
        model.articulation(
            f"{side}_shoulder",
            ArticulationType.REVOLUTE,
            parent=torso,
            child=upper,
            origin=Origin(xyz=(sx * 0.205, 0.0, 1.49)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=10.0, velocity=2.0, lower=-2.8, upper=2.8
            ),
        )

        forearm = model.part(f"{side}_forearm")
        forearm.visual(
            Sphere(radius=0.045),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=BLACK,
            name="elbow_ball",
        )
        forearm.visual(
            lathe_mesh([
                (0.000, 0.000), (0.038, -0.006), (0.045, -0.030),
                (0.041, -0.120), (0.035, -0.200), (0.031, -0.245),
                (0.000, -0.255),
            ], f"{side}_forearm_seg"),
            material=BLACK,
            name="forearm_seg",
        )
        model.articulation(
            f"{side}_elbow",
            ArticulationType.REVOLUTE,
            parent=upper,
            child=forearm,
            origin=Origin(xyz=(0.0, 0.0, -0.30)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=2.6),
        )

        hand = model.part(f"{side}_hand")
        hand.visual(
            Sphere(radius=0.035),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=BLACK,
            name="wrist_ball",
        )
        hand.visual(
            loft_mesh([
                ring_xy(-0.020, 0.022, 0.036, n=20, e=2.2),
                ring_xy(-0.060, 0.028, 0.052, n=20, e=2.2),  # palm
                ring_xy(-0.120, 0.026, 0.050, n=20, e=2.2),
                ring_xy(-0.160, 0.017, 0.032, n=20, e=2.2),  # fingertips
                ring_xy(-0.175, 0.006, 0.012, n=20, e=2.2),
            ], f"{side}_hand_paddle"),
            material=BLACK,
            name="hand_paddle",
        )
        model.articulation(
            f"{side}_wrist",
            ArticulationType.REVOLUTE,
            parent=forearm,
            child=hand,
            origin=Origin(xyz=(0.0, 0.0, -0.26)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-1.0, upper=1.0),
        )

    # ------------------------------------------------------------------
    # Legs: hip / knee / ankle chains, mirrored. Tapered muscular thigh/calf
    # lathes + a contoured lofted foot.
    # ------------------------------------------------------------------
    for side, sx in (("left", 1.0), ("right", -1.0)):
        thigh = model.part(f"{side}_thigh")
        thigh.visual(
            Sphere(radius=0.062),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=BLACK,
            name="hip_ball",
        )
        thigh.visual(
            lathe_mesh([
                (0.000, 0.000), (0.048, -0.006), (0.062, -0.040),
                (0.066, -0.120), (0.062, -0.240), (0.055, -0.340),
                (0.048, -0.372), (0.000, -0.378),
            ], f"{side}_thigh_seg"),
            material=BLACK,
            name="thigh_seg",
        )
        model.articulation(
            f"{side}_hip",
            ArticulationType.REVOLUTE,
            parent=torso,
            child=thigh,
            origin=Origin(xyz=(sx * 0.09, 0.0, 0.99)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=-0.6, upper=1.8),
        )

        shin = model.part(f"{side}_shin")
        shin.visual(
            Sphere(radius=0.058),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=BLACK,
            name="knee_ball",
        )
        shin.visual(
            lathe_mesh([
                (0.000, 0.000), (0.046, -0.010), (0.056, -0.060),
                (0.052, -0.140), (0.043, -0.260), (0.036, -0.350),
                (0.000, -0.380),
            ], f"{side}_shin_seg"),
            material=BLACK,
            name="shin_seg",
        )
        model.articulation(
            f"{side}_knee",
            ArticulationType.REVOLUTE,
            parent=thigh,
            child=shin,
            origin=Origin(xyz=(0.0, 0.0, -0.42)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=0.0, upper=2.2),
        )

        foot = model.part(f"{side}_foot")
        foot.visual(
            Sphere(radius=0.045),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=BLACK,
            name="ankle_ball",
        )
        # Authored in XY (length along +z, sole pinned flat), then laid
        # toe-forward (+y) and sole-down (-z) by the rpy origin rotation.
        foot.visual(
            loft_mesh([
                ring_foot(0.000, 0.033, 0.045),  # heel
                ring_foot(0.035, 0.042, 0.047),
                ring_foot(0.105, 0.040, 0.045),  # mid-foot
                ring_foot(0.165, 0.033, 0.038),
                ring_foot(0.205, 0.020, 0.028),  # rounded toe
            ], f"{side}_foot_wedge"),
            origin=Origin(xyz=(0.0, -0.055, -0.070), rpy=(-math.pi / 2, 0.0, 0.0)),
            material=BLACK,
            name="foot_wedge",
        )
        model.articulation(
            f"{side}_ankle",
            ArticulationType.REVOLUTE,
            parent=shin,
            child=foot,
            origin=Origin(xyz=(0.0, 0.0, -0.385)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.6, upper=0.6),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    torso = object_model.get_part("torso")
    head = object_model.get_part("head")

    # --- Intentional joint-ball / socket embeds and stand seating. ---
    height_rod = object_model.get_part("height_rod")
    ctx.allow_overlap(height_rod, torso, elem_a="lean_saddle", elem_b="pelvis",
                      reason="Telescoping stand saddle seats against the pelvis back.")
    ctx.allow_overlap(base, height_rod, elem_a="outer_sleeve", elem_b="inner_rod_member",
                      reason="Inner rod telescopes inside the outer sleeve as a height-adjustable prismatic fit.")
    ctx.allow_overlap(head, torso, elem_a="head_chin", elem_b="neck",
                      reason="Chin lobe seats over the neck stub.")
    for side in ("left", "right"):
        ctx.allow_overlap(f"{side}_upper_arm", torso,
                          elem_a="shoulder_ball", elem_b=f"deltoid_{side}",
                          reason="Shoulder ball nests in the deltoid shoulder cap.")
        ctx.allow_overlap(f"{side}_upper_arm", torso,
                          elem_a="upper_arm_seg", elem_b=f"deltoid_{side}",
                          reason="Upper-arm top seats into the deltoid shoulder cap.")
        ctx.allow_overlap(f"{side}_forearm", f"{side}_upper_arm",
                          elem_a="elbow_ball", elem_b="upper_arm_seg",
                          reason="Elbow ball nests in the upper-arm end.")
        ctx.allow_overlap(f"{side}_hand", f"{side}_forearm",
                          elem_a="wrist_ball", elem_b="forearm_seg",
                          reason="Wrist ball nests in the forearm end.")
        ctx.allow_overlap(f"{side}_thigh", torso,
                          elem_a="hip_ball", elem_b="pelvis",
                          reason="Hip ball nests into the pelvis socket so the leg is visibly seated.")
        ctx.allow_overlap(f"{side}_thigh", torso,
                          elem_a="thigh_seg", elem_b="pelvis",
                          reason="Thigh seats into the pelvis hip socket.")
        ctx.allow_overlap(f"{side}_shin", f"{side}_thigh",
                          elem_a="knee_ball", elem_b="thigh_seg",
                          reason="Knee ball nests in the thigh end.")
        ctx.allow_overlap(f"{side}_foot", f"{side}_shin",
                          elem_a="ankle_ball", elem_b="shin_seg",
                          reason="Ankle ball nests in the shin end.")

    # --- Stand reads as a rear lean-support behind the figure. ---
    sleeve = base.get_visual("outer_sleeve")
    ctx.check(
        "outer_sleeve stands behind the pelvis",
        sleeve.origin.xyz[1] < -0.05,
        details=f"sleeve_y={sleeve.origin.xyz[1]}",
    )

    # --- Prismatic telescoping joint raises/lowers the saddle. ---
    sleeve_joint = object_model.get_articulation("sleeve_to_rod")
    rest_saddle_z = ctx.part_element_world_aabb(height_rod, elem="lean_saddle")[1][2]
    with ctx.pose({sleeve_joint: 0.25}):
        raised_saddle_z = ctx.part_element_world_aabb(height_rod, elem="lean_saddle")[1][2]
    ctx.check(
        "sleeve_to_rod prismatic raises the lean_saddle vertically",
        raised_saddle_z > rest_saddle_z + 0.20,
        details=f"rest_saddle_top_z={rest_saddle_z:.4f}, raised_saddle_top_z={raised_saddle_z:.4f}",
    )
    # Inner rod stays centered inside the outer sleeve on non-motion axes.
    ctx.expect_within(
        height_rod, base,
        axes="xy",
        inner_elem="inner_rod_member",
        outer_elem="outer_sleeve",
        margin=0.008,
        name="inner rod stays centered in outer sleeve",
    )

    # --- Feet rest on and stay over the square base plate. ---
    for side in ("left", "right"):
        foot = object_model.get_part(f"{side}_foot")
        ctx.expect_gap(
            foot,
            base,
            axis="z",
            positive_elem="foot_wedge",
            negative_elem="base_plate",
            min_gap=-0.004,
            max_gap=0.006,
            name=f"{side} foot seats on base plate",
        )
        ctx.expect_within(
            foot,
            base,
            axes="xy",
            outer_elem="base_plate",
            margin=0.01,
            name=f"{side} foot stays over the base plate",
        )

    # --- Head seats above the torso neck. ---
    ctx.expect_gap(head, torso, axis="z", max_penetration=0.06, max_gap=0.03,
                   negative_elem="neck", name="head seats on the neck")

    # --- Contemplative pose is reachable: fold the right arm across the chest. ---
    r_shoulder = object_model.get_articulation("right_shoulder")
    r_elbow = object_model.get_articulation("right_elbow")
    r_hand = object_model.get_part("right_hand")
    rest_rh = ctx.part_world_position(r_hand)
    with ctx.pose({r_shoulder: 1.4, r_elbow: 1.9}):
        folded = ctx.part_world_position(r_hand)
    ctx.check(
        "folding the right arm brings the hand up and inward across the chest",
        rest_rh is not None and folded is not None
        and folded[2] > rest_rh[2] + 0.25 and abs(folded[0]) < abs(rest_rh[0]),
        details=f"rest={rest_rh}, folded={folded}",
    )

    # --- Left hand can rise toward the chin (thinking gesture). ---
    l_shoulder = object_model.get_articulation("left_shoulder")
    l_elbow = object_model.get_articulation("left_elbow")
    l_hand = object_model.get_part("left_hand")
    rest_lh = ctx.part_world_position(l_hand)
    with ctx.pose({l_shoulder: 2.0, l_elbow: 2.4}):
        raised = ctx.part_world_position(l_hand)
    ctx.check(
        "raising the left arm lifts the hand toward the head/chin",
        rest_lh is not None and raised is not None and raised[2] > rest_lh[2] + 0.35,
        details=f"rest={rest_lh}, raised={raised}",
    )

    # --- Knee bend lifts the foot like a forward-braced leg. ---
    knee = object_model.get_articulation("left_knee")
    foot = object_model.get_part("left_foot")
    rest_f = ctx.part_world_position(foot)
    with ctx.pose({knee: 1.6}):
        bent = ctx.part_world_position(foot)
    ctx.check(
        "bending the left knee raises the foot",
        rest_f is not None and bent is not None and bent[2] > rest_f[2] + 0.10,
        details=f"rest={rest_f}, bent={bent}",
    )

    return ctx.report()


object_model = build_object_model()
