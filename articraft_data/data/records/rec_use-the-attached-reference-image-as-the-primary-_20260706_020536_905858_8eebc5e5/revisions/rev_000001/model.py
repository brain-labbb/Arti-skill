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

WOOD = "mannequin_wood"
WOOD_BASE = "base_wood"
STEEL = "stand_steel"


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
    """Superellipse (rounded-rectangle) ring in the X-Y plane at height z."""
    pts = []
    for i in range(n):
        t = 2.0 * math.pi * i / n
        c, s = math.cos(t), math.sin(t)
        x = a * (abs(c) ** (2.0 / e)) * (1.0 if c >= 0 else -1.0)
        y = b * (abs(s) ** (2.0 / e)) * (1.0 if s >= 0 else -1.0)
        pts.append((x, y, z))
    return pts


def ring_foot(zc: float, xh: float, ry: float, sole: float = 0.045, n: int = 24) -> list[tuple[float, float, float]]:
    """Foot cross-section authored in X-Y (length along z, sole pinned flat),
    laid toe-forward and sole-down by an rpy origin rotation at placement."""
    cy = sole - ry
    pts = []
    for i in range(n):
        t = 2.0 * math.pi * i / n
        pts.append((xh * math.cos(t), cy + ry * math.sin(t), zc))
    return pts


def build_object_model() -> ArticulatedObject:
    """Classic light-wood poseable artist drawing mannequin (ref 001.png):
    natural maple body sculpted with rounded barrel torso, tapered limbs, an
    ovoid head, paddle hands and flat oval feet, with a visible ball joint at
    every articulation. Held aloft by a thin steel pin on a round wooden disc
    base attached to the lower back."""
    model = ArticulatedObject(name="poseable_drawing_mannequin_001")
    model.material(WOOD, rgba=(0.82, 0.68, 0.45, 1.0))
    model.material(WOOD_BASE, rgba=(0.73, 0.57, 0.35, 1.0))
    model.material(STEEL, rgba=(0.58, 0.58, 0.61, 1.0))

    # ------------------------------------------------------------------
    # Base: round wooden disc + thin steel pin rising to the lower back.
    # ------------------------------------------------------------------
    base = model.part("base")
    base.visual(
        Cylinder(radius=0.105, length=0.030),
        origin=Origin(xyz=(0.0, -0.20, 0.015)),
        material=WOOD_BASE,
        name="base_disc",
    )
    # Thin steel display pin (rises vertically from the disc).
    base.visual(
        Cylinder(radius=0.0065, length=1.030),
        origin=Origin(xyz=(0.0, -0.13, 0.545)),
        material=STEEL,
        name="support_pin",
    )
    # Small saddle bracket bridging the pin top to the pelvis lower back so the
    # figure is structurally connected where the pin holds it.
    base.visual(
        Box((0.05, 0.11, 0.028)),
        origin=Origin(xyz=(0.0, -0.105, 1.040)),
        material=STEEL,
        name="pin_saddle",
    )

    # ------------------------------------------------------------------
    # Torso: rounded barrel chest, waist, hips (blended lofts) + deltoid caps
    # and a tapered neck. Fixed to the base through the rear pin.
    # ------------------------------------------------------------------
    torso = model.part("torso")
    torso.visual(
        loft_mesh([
            ring_xy(0.97, 0.116, 0.092),  # low pelvis / crotch (envelops thigh tops)
            ring_xy(1.05, 0.146, 0.106),  # hip flare (widest)
            ring_xy(1.13, 0.124, 0.094),
        ], "torso_pelvis"),
        material=WOOD,
        name="pelvis",
    )
    torso.visual(
        loft_mesh([
            ring_xy(1.12, 0.124, 0.094),
            ring_xy(1.22, 0.110, 0.086),  # waist
            ring_xy(1.33, 0.140, 0.100),
        ], "torso_waist"),
        material=WOOD,
        name="waist",
    )
    torso.visual(
        loft_mesh([
            ring_xy(1.32, 0.140, 0.100),
            ring_xy(1.40, 0.146, 0.110),  # barrel chest
            ring_xy(1.48, 0.134, 0.098),  # upper chest
            ring_xy(1.55, 0.098, 0.082),  # taper toward neck
            ring_xy(1.60, 0.068, 0.064),
        ], "torso_chest"),
        material=WOOD,
        name="chest",
    )
    # Deltoid / clavicle caps so the arms socket into a shoulder yoke.
    torso.visual(
        Sphere(radius=0.055),
        origin=Origin(xyz=(0.155, 0.0, 1.50)),
        material=WOOD,
        name="deltoid_left",
    )
    torso.visual(
        Sphere(radius=0.055),
        origin=Origin(xyz=(-0.155, 0.0, 1.50)),
        material=WOOD,
        name="deltoid_right",
    )
    torso.visual(
        lathe_mesh([
            (0.000, 0.000), (0.055, 0.006), (0.050, 0.050),
            (0.046, 0.095), (0.040, 0.108), (0.000, 0.112),
        ], "torso_neck"),
        origin=Origin(xyz=(0.0, 0.0, 1.52)),
        material=WOOD,
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
    # One clean smooth egg (solid of revolution) -- no separate chin lobe -- like
    # the featureless wooden mannequin head. Lowered so it seats over the neck.
    head = model.part("head")
    head.visual(
        lathe_mesh([
            (0.000, 0.000), (0.042, 0.018), (0.064, 0.052),
            (0.079, 0.095), (0.088, 0.140), (0.092, 0.185),
            (0.093, 0.222), (0.089, 0.248), (0.076, 0.267),
            (0.048, 0.280), (0.000, 0.286),
        ], "head_ovoid"),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=WOOD,
        name="head_shell",
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
    # Arms: shoulder / elbow / wrist chains, mirrored. Tapered bicep/forearm
    # lathes + a flattened paddle hand, with visible joint balls.
    # ------------------------------------------------------------------
    for side, sx in (("left", 1.0), ("right", -1.0)):
        upper = model.part(f"{side}_upper_arm")
        upper.visual(
            Sphere(radius=0.050),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=WOOD,
            name="shoulder_ball",
        )
        upper.visual(
            lathe_mesh([
                (0.000, 0.000), (0.035, -0.006), (0.048, -0.035),
                (0.050, -0.080), (0.046, -0.150), (0.040, -0.240),
                (0.036, -0.285), (0.000, -0.292),
            ], f"{side}_upper_arm_seg"),
            material=WOOD,
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
            material=WOOD,
            name="elbow_ball",
        )
        forearm.visual(
            lathe_mesh([
                (0.000, 0.000), (0.038, -0.006), (0.045, -0.030),
                (0.041, -0.120), (0.035, -0.200), (0.031, -0.245),
                (0.000, -0.255),
            ], f"{side}_forearm_seg"),
            material=WOOD,
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
            material=WOOD,
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
            material=WOOD,
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
    # Legs: hip / knee / ankle chains, mirrored. Tapered thigh/calf lathes +
    # a flat oval lofted foot, with visible joint balls.
    # ------------------------------------------------------------------
    for side, sx in (("left", 1.0), ("right", -1.0)):
        thigh = model.part(f"{side}_thigh")
        thigh.visual(
            lathe_mesh([
                (0.000, 0.000), (0.050, -0.006), (0.066, -0.040),
                (0.070, -0.120), (0.066, -0.240), (0.058, -0.340),
                (0.050, -0.372), (0.000, -0.378),
            ], f"{side}_thigh_seg"),
            material=WOOD,
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
            material=WOOD,
            name="knee_ball",
        )
        shin.visual(
            lathe_mesh([
                (0.000, 0.000), (0.046, -0.010), (0.056, -0.060),
                (0.052, -0.140), (0.043, -0.260), (0.036, -0.350),
                (0.000, -0.380),
            ], f"{side}_shin_seg"),
            material=WOOD,
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
            material=WOOD,
            name="ankle_ball",
        )
        # Flat oval wooden foot, authored in XY then laid toe-forward / sole-down.
        foot.visual(
            loft_mesh([
                ring_foot(0.000, 0.034, 0.038),  # heel
                ring_foot(0.035, 0.043, 0.040),
                ring_foot(0.105, 0.041, 0.038),  # mid-foot
                ring_foot(0.165, 0.034, 0.032),
                ring_foot(0.205, 0.021, 0.022),  # rounded toe
            ], f"{side}_foot_wedge"),
            origin=Origin(xyz=(0.0, -0.055, -0.070), rpy=(-math.pi / 2, 0.0, 0.0)),
            material=WOOD,
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

    # --- Intentional joint-ball / socket embeds and pin seating. ---
    ctx.allow_overlap(base, torso, elem_a="pin_saddle", elem_b="pelvis",
                      reason="Rear display-pin saddle seats against the pelvis back.")
    ctx.allow_overlap(head, torso, elem_a="head_shell", elem_b="neck",
                      reason="Egg head seats over and envelops the neck top.")
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
                          elem_a="thigh_seg", elem_b="pelvis",
                          reason="Thigh seats into the pelvis hip socket.")
        ctx.allow_overlap(f"{side}_shin", f"{side}_thigh",
                          elem_a="knee_ball", elem_b="thigh_seg",
                          reason="Knee ball nests in the thigh end.")
        ctx.allow_overlap(f"{side}_foot", f"{side}_shin",
                          elem_a="ankle_ball", elem_b="shin_seg",
                          reason="Ankle ball nests in the shin end.")

    # --- Stand reads as a rear display pin on a round disc behind the figure. ---
    disc = base.get_visual("base_disc")
    pin = base.get_visual("support_pin")
    ctx.check(
        "wooden disc base sits behind the figure",
        disc.origin.xyz[1] < -0.1,
        details=f"disc_y={disc.origin.xyz[1]}",
    )
    ctx.check(
        "support pin is thin and tall",
        pin.geometry.radius < 0.01 and pin.geometry.length > 0.8,
        details=f"pin_r={pin.geometry.radius}, pin_len={pin.geometry.length}",
    )

    # --- Head seats above the torso neck. ---
    ctx.expect_gap(head, torso, axis="z", max_penetration=0.06, max_gap=0.03,
                   negative_elem="neck", name="head seats on the neck")

    # --- Poseable: fold the right arm across the chest. ---
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

    # --- Left arm can raise (waving gesture like the image). ---
    l_shoulder = object_model.get_articulation("left_shoulder")
    l_elbow = object_model.get_articulation("left_elbow")
    l_hand = object_model.get_part("left_hand")
    rest_lh = ctx.part_world_position(l_hand)
    with ctx.pose({l_shoulder: 2.0, l_elbow: 1.2}):
        raised = ctx.part_world_position(l_hand)
    ctx.check(
        "raising the left arm lifts the hand upward",
        rest_lh is not None and raised is not None and raised[2] > rest_lh[2] + 0.35,
        details=f"rest={rest_lh}, raised={raised}",
    )

    # --- Knee bend lifts the foot (mid-stride like the walking figure). ---
    knee = object_model.get_articulation("left_knee")
    foot = object_model.get_part("left_foot")
    rest_f = ctx.part_world_position(foot)
    with ctx.pose({knee: 1.6}):
        bent = ctx.part_world_position(foot)
    ctx.check(
        "bending the left knee lifts the foot for a stride",
        rest_f is not None and bent is not None and bent[2] > rest_f[2] + 0.10,
        details=f"rest={rest_f}, bent={bent}",
    )

    return ctx.report()


object_model = build_object_model()
