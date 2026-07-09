from __future__ import annotations

"""Classic 20x50 Porro-prism binoculars with twist-up eyecups.

Layout (world frame):
- +X is the viewing direction: objective lenses face +X, eyecups face -X.
- +Z is up; the binoculars rest on their objective tubes near z = 0.
- The central hinge bridge axle runs along X at y = 0, z = HINGE_Z.

Kinematic tree:
- hinge_bridge (root): central axle + end caps.
- left_barrel / right_barrel: revolute about the longitudinal axle axis
  (interpupillary fold, +/- 25 deg each).
- focus_wheel: continuous rotation about the same central axis.
- diopter_ring: revolute about the right eyepiece viewing axis (+/- 60 deg).
- eyecup_collar_0 / eyecup_collar_1: prismatic along each eyepiece viewing
  axis (twist-up helical extension, 0 to 8 mm travel).
"""

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

# --- Shared layout constants -------------------------------------------------

HINGE_Z = 0.031  # world height of the central hinge axle axis
OBJ_Y = 0.065  # lateral offset of each objective tube axis from the hinge
EYE_Y = 0.032  # lateral offset of each eyepiece axis (IPD ~64 mm)
OBJ_TUBE_R = 0.0305  # objective bell outer radius (~0.06 m diameter)
EYE_TUBE_R = 0.0135  # slim eyepiece tube radius
FOLD_LIMIT = math.radians(25.0)
DIOPTER_LIMIT = math.radians(60.0)
FOCUS_WHEEL_X = -0.059
DIOPTER_X = -0.048

# Twist-up eyecup collar constants
EYECUP_TRAVEL = 0.008  # prismatic travel for twist-up extension (m)
EYECUP_MOUNT_X = -0.054  # barrel-local X of the collar joint origin
EYECUP_COLLAR_LEN = 0.035  # collar length along the viewing axis
EYECUP_BORE_R = 0.0153  # inner bore radius (captured fit over eyepiece ring)
EYECUP_OUTER_R = 0.0172  # collar outer radius

# Visual rpy that maps a +Z-aligned cylinder/lathe/knob onto the +X axis.
ROT_Z_TO_PX = (0.0, math.pi / 2.0, 0.0)
# Same, but the lathe's open end (profile +Z) faces -X instead.
ROT_Z_TO_NX = (0.0, -math.pi / 2.0, 0.0)

# Hinge-lug axial stations (sleeve centers along the axle), interleaved so the
# left and right barrel lugs alternate like a real Porro center hinge.
LEFT_SLEEVE_XS = (0.031, -0.015)
RIGHT_SLEEVE_XS = (0.008, -0.038)
SLEEVE_LEN = 0.018
SLEEVE_R = 0.0095
AXLE_R = 0.006


def _objective_tube_mesh(tag: str):
    """Hollow tapered objective bell, axis +Z, rear at z=0, open mouth at z=0.08."""
    outer = [
        (0.0265, 0.000),
        (0.0265, 0.010),
        (0.0290, 0.042),
        (OBJ_TUBE_R, 0.070),
        (OBJ_TUBE_R, 0.080),
    ]
    inner = [
        (0.0000, 0.060),
        (0.0240, 0.062),
        (0.0250, 0.066),
        (0.0265, 0.080),
    ]
    geom = LatheGeometry.from_shell_profiles(outer, inner, segments=48)
    return mesh_from_geometry(geom, f"{tag}_objective_tube")


def _twist_up_eyecup_collar_mesh(tag: str):
    """Rigid cylindrical twist-up eyecup collar, axis +Z, mount face at z=0.

    The collar has helical grip grooves on the outer surface for twist-up
    action and a thin internal bore that captures the eyepiece ring.
    """
    outer = [
        (EYECUP_BORE_R + 0.0004, 0.000),  # base skirt
        (EYECUP_OUTER_R, 0.001),
        (EYECUP_OUTER_R, 0.005),
        (EYECUP_OUTER_R - 0.0008, 0.006),  # grip groove
        (EYECUP_OUTER_R, 0.007),
        (EYECUP_OUTER_R, 0.013),
        (EYECUP_OUTER_R - 0.0008, 0.014),  # grip groove
        (EYECUP_OUTER_R, 0.015),
        (EYECUP_OUTER_R, 0.021),
        (EYECUP_OUTER_R - 0.0008, 0.022),  # grip groove
        (EYECUP_OUTER_R, 0.023),
        (EYECUP_OUTER_R, 0.029),
        (EYECUP_OUTER_R - 0.0008, 0.030),  # grip groove
        (EYECUP_OUTER_R, 0.031),
        (EYECUP_OUTER_R + 0.0005, 0.034),
        (EYECUP_OUTER_R + 0.0005, EYECUP_COLLAR_LEN),  # eye-end lip
    ]
    inner = [
        (EYECUP_BORE_R, 0.000),
        (EYECUP_BORE_R, EYECUP_COLLAR_LEN),
    ]
    geom = LatheGeometry.from_shell_profiles(outer, inner, segments=40)
    return mesh_from_geometry(geom, f"{tag}_collar")


def _housing_mesh(tag: str):
    """Rounded prism-housing block, extruded along +Z (rotated onto X at placement)."""
    profile = rounded_rect_profile(0.054, 0.078, 0.012)
    geom = ExtrudeGeometry(profile, 0.070)
    return mesh_from_geometry(geom, f"{tag}_prism_housing")


def _add_barrel(model: ArticulatedObject, name: str, side: int, sleeve_xs, mats: dict) -> object:
    """Build one barrel. `side` is +1 (left, +Y) or -1 (right, -Y).

    The barrel part frame sits on the central hinge axis (y=0, z=HINGE_Z world)
    so the fold articulation rotates it about its own local +X.
    """
    armor = mats["rubber_armor"]
    metal = mats["hinge_metal"]
    lens_amber = mats["lens_amber"]
    lens_dark = mats["ocular_glass"]

    barrel = model.part(name)

    # Wide tapered objective tube, hollow with a recessed amber lens disc.
    barrel.visual(
        _objective_tube_mesh(name),
        origin=Origin(xyz=(0.020, side * OBJ_Y, 0.0), rpy=ROT_Z_TO_PX),
        material=armor,
        name="objective_tube",
    )
    barrel.visual(
        Cylinder(radius=0.0235, length=0.006),
        origin=Origin(xyz=(0.0832, side * OBJ_Y, 0.0), rpy=ROT_Z_TO_PX),
        material=lens_amber,
        name="objective_lens",
    )

    # Stepped prism housing spanning from the objective axis to the eyepiece axis.
    barrel.visual(
        _housing_mesh(name),
        origin=Origin(xyz=(0.0, side * 0.055, 0.0), rpy=ROT_Z_TO_PX),
        material=armor,
        name="prism_housing",
    )

    # Slim eyepiece tube with a dark metal ring and a body connecting to the ocular.
    barrel.visual(
        Cylinder(radius=EYE_TUBE_R, length=0.044),
        origin=Origin(xyz=(-0.052, side * EYE_Y, 0.0), rpy=ROT_Z_TO_PX),
        material=metal,
        name="eyepiece_tube",
    )
    barrel.visual(
        Cylinder(radius=0.0155, length=0.008),
        origin=Origin(xyz=(-0.062, side * EYE_Y, 0.0), rpy=ROT_Z_TO_PX),
        material=metal,
        name="eyepiece_ring",
    )
    # Eyepiece body connecting the ring to the ocular lens (structural support).
    barrel.visual(
        Cylinder(radius=0.014, length=0.015),
        origin=Origin(xyz=(-0.0735, side * EYE_Y, 0.0), rpy=ROT_Z_TO_PX),
        material=metal,
        name="eyepiece_body",
    )
    barrel.visual(
        Cylinder(radius=0.0125, length=0.005),
        origin=Origin(xyz=(-0.0835, side * EYE_Y, 0.0), rpy=ROT_Z_TO_PX),
        material=lens_dark,
        name="ocular_lens",
    )

    # Hinge lugs: a sleeve captured on the central axle plus an arm into the housing.
    sleeve_names = ("front_hinge_sleeve", "rear_hinge_sleeve")
    arm_names = ("front_hinge_arm", "rear_hinge_arm")
    for sx, sleeve_name, arm_name in zip(sleeve_xs, sleeve_names, arm_names):
        barrel.visual(
            Cylinder(radius=SLEEVE_R, length=SLEEVE_LEN),
            origin=Origin(xyz=(sx, 0.0, 0.0), rpy=ROT_Z_TO_PX),
            material=metal,
            name=sleeve_name,
        )
        barrel.visual(
            Box((SLEEVE_LEN, 0.0135, 0.019)),
            origin=Origin(xyz=(sx, side * 0.01525, 0.0)),
            material=metal,
            name=arm_name,
        )

    return barrel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="porro_prism_binoculars_20x50_twistup")

    mats = {
        "rubber_armor": model.material("rubber_armor", rgba=(0.100, 0.100, 0.105, 1.0)),
        "hinge_metal": model.material("hinge_metal", rgba=(0.300, 0.310, 0.330, 1.0)),
        "eyecup_collar": model.material("eyecup_collar", rgba=(0.120, 0.120, 0.130, 1.0)),
        "lens_amber": model.material("lens_amber", rgba=(0.520, 0.160, 0.080, 1.0)),
        "ocular_glass": model.material("ocular_glass", rgba=(0.110, 0.080, 0.070, 1.0)),
        "wheel_black": model.material("wheel_black", rgba=(0.080, 0.080, 0.090, 1.0)),
    }
    metal = mats["hinge_metal"]

    # --- Root: central hinge bridge -----------------------------------------
    bridge = model.part("hinge_bridge")
    bridge.visual(
        Cylinder(radius=AXLE_R, length=0.119),
        origin=Origin(xyz=(-0.0125, 0.0, HINGE_Z), rpy=ROT_Z_TO_PX),
        material=metal,
        name="hinge_axle",
    )
    bridge.visual(
        Cylinder(radius=0.0095, length=0.006),
        origin=Origin(xyz=(0.0485, 0.0, HINGE_Z), rpy=ROT_Z_TO_PX),
        material=metal,
        name="front_hinge_cap",
    )
    bridge.visual(
        Cylinder(radius=0.0095, length=0.006),
        origin=Origin(xyz=(-0.074, 0.0, HINGE_Z), rpy=ROT_Z_TO_PX),
        material=metal,
        name="rear_hinge_cap",
    )

    # --- Mirrored offset barrels ---------------------------------------------
    barrel_defs = [
        ("left_barrel", +1, LEFT_SLEEVE_XS),
        ("right_barrel", -1, RIGHT_SLEEVE_XS),
    ]
    barrel_objs = []
    for barrel_name, side, sleeve_xs in barrel_defs:
        barrel = _add_barrel(model, barrel_name, side, sleeve_xs, mats)
        barrel_objs.append((barrel, barrel_name, side))

    left_barrel = barrel_objs[0][0]
    right_barrel = barrel_objs[1][0]

    model.articulation(
        "bridge_to_left_barrel",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=left_barrel,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-FOLD_LIMIT, upper=FOLD_LIMIT),
    )
    model.articulation(
        "bridge_to_right_barrel",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=right_barrel,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-FOLD_LIMIT, upper=FOLD_LIMIT),
    )

    # --- Knurled center focus wheel ------------------------------------------
    focus_wheel = model.part("focus_wheel")
    wheel_geom = KnobGeometry(
        0.027,
        0.020,
        body_style="cylindrical",
        grip=KnobGrip(style="knurled", count=36, depth=0.0008, helix_angle_deg=0.0),
    )
    focus_wheel.visual(
        mesh_from_geometry(wheel_geom, "focus_wheel"),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=ROT_Z_TO_PX),
        material=mats["wheel_black"],
        name="focus_wheel_knurl",
    )
    model.articulation(
        "bridge_to_focus_wheel",
        ArticulationType.CONTINUOUS,
        parent=bridge,
        child=focus_wheel,
        origin=Origin(xyz=(FOCUS_WHEEL_X, 0.0, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=6.0),
    )

    # --- Right eyepiece diopter ring ------------------------------------------
    diopter_ring = model.part("diopter_ring")
    diopter_geom = KnobGeometry(
        0.033,
        0.012,
        body_style="cylindrical",
        grip=KnobGrip(style="knurled", count=30, depth=0.0008, helix_angle_deg=0.0),
    )
    diopter_ring.visual(
        mesh_from_geometry(diopter_geom, "diopter_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=ROT_Z_TO_PX),
        material=mats["wheel_black"],
        name="diopter_knurl",
    )
    model.articulation(
        "right_barrel_to_diopter_ring",
        ArticulationType.REVOLUTE,
        parent=right_barrel,
        child=diopter_ring,
        origin=Origin(xyz=(DIOPTER_X, -EYE_Y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0, velocity=4.0, lower=-DIOPTER_LIMIT, upper=DIOPTER_LIMIT
        ),
    )

    # --- Twist-up eyecup collars (prismatic, shared geometry) -----------------
    for i, (barrel, barrel_name, side) in enumerate(barrel_objs):
        collar_name = f"eyecup_collar_{i}"
        collar = model.part(collar_name)
        collar.visual(
            _twist_up_eyecup_collar_mesh(collar_name),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=ROT_Z_TO_NX),
            material=mats["eyecup_collar"],
            name="collar",
        )
        # Prismatic joint: collar slides along the eyepiece viewing axis (-X).
        # Joint origin on the eyepiece tube where the collar bore captures the ring.
        # Positive q extends the collar outward (twisted up for eyeglass wearers).
        model.articulation(
            f"{barrel_name}_to_{collar_name}",
            ArticulationType.PRISMATIC,
            parent=barrel,
            child=collar,
            origin=Origin(xyz=(EYECUP_MOUNT_X, side * EYE_Y, 0.0)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0, velocity=0.5, lower=0.0, upper=EYECUP_TRAVEL
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bridge = object_model.get_part("hinge_bridge")
    left = object_model.get_part("left_barrel")
    right = object_model.get_part("right_barrel")
    wheel = object_model.get_part("focus_wheel")
    diopter = object_model.get_part("diopter_ring")
    left_collar = object_model.get_part("eyecup_collar_0")
    right_collar = object_model.get_part("eyecup_collar_1")
    left_hinge = object_model.get_articulation("bridge_to_left_barrel")
    right_hinge = object_model.get_articulation("bridge_to_right_barrel")
    focus_joint = object_model.get_articulation("bridge_to_focus_wheel")
    diopter_joint = object_model.get_articulation("right_barrel_to_diopter_ring")
    left_collar_joint = object_model.get_articulation("left_barrel_to_eyecup_collar_0")
    right_collar_joint = object_model.get_articulation("right_barrel_to_eyecup_collar_1")

    # --- Intentional captured-shaft overlaps ---------------------------------
    for barrel in (left, right):
        for sleeve in ("front_hinge_sleeve", "rear_hinge_sleeve"):
            ctx.allow_overlap(
                barrel,
                bridge,
                elem_a=sleeve,
                elem_b="hinge_axle",
                reason="Hinge lug sleeve is intentionally captured around the central axle.",
            )
    ctx.allow_overlap(
        wheel,
        bridge,
        elem_a="focus_wheel_knurl",
        elem_b="hinge_axle",
        reason="Center focus wheel is intentionally captured on the central axle.",
    )
    ctx.allow_overlap(
        diopter,
        right,
        elem_a="diopter_knurl",
        elem_b="eyepiece_tube",
        reason="Diopter ring is intentionally captured around the right eyepiece tube.",
    )

    # Twist-up collar bore captures the eyepiece ring (sliding fit).
    for collar, barrel in ((left_collar, left), (right_collar, right)):
        ctx.allow_overlap(
            collar,
            barrel,
            elem_a="collar",
            elem_b="eyepiece_ring",
            reason="Twist-up eyecup collar bore intentionally captures the eyepiece ring for sliding fit.",
        )

    # --- Overall envelope matches the 0.20 x 0.19 x 0.06 m spec --------------
    all_parts = [bridge, left, right, wheel, diopter, left_collar, right_collar]
    aabbs = [ctx.part_world_aabb(p) for p in all_parts]
    lo = [min(a[0][i] for a in aabbs) for i in range(3)]
    hi = [max(a[1][i] for a in aabbs) for i in range(3)]
    ctx.check(
        "overall length ~0.20 m along viewing axis",
        0.185 <= hi[0] - lo[0] <= 0.215,
        details=f"length={hi[0] - lo[0]:.4f}",
    )
    ctx.check(
        "overall width ~0.19 m with barrels open",
        0.182 <= hi[1] - lo[1] <= 0.200,
        details=f"width={hi[1] - lo[1]:.4f}",
    )
    ctx.check(
        "overall height ~0.06 m",
        0.052 <= hi[2] - lo[2] <= 0.068,
        details=f"height={hi[2] - lo[2]:.4f}",
    )
    ctx.check(
        "binoculars rest on the ground plane",
        -0.003 <= lo[2] <= 0.004,
        details=f"min_z={lo[2]:.4f}",
    )

    # --- Mirrored offset barrel structure ------------------------------------
    for barrel, side in ((left, +1), (right, -1)):
        obj_bb = ctx.part_element_world_aabb(barrel, elem="objective_tube")
        i = 0 if side > 0 else 1
        collar = object_model.get_part(f"eyecup_collar_{i}")
        eye_bb = ctx.part_element_world_aabb(collar, elem="collar")
        obj_cy = 0.5 * (obj_bb[0][1] + obj_bb[1][1])
        eye_cy = 0.5 * (eye_bb[0][1] + eye_bb[1][1])
        ctx.check(
            f"{barrel.name} objective axis sits outboard of its eyepiece axis",
            side * obj_cy > side * eye_cy + 0.020,
            details=f"objective_y={obj_cy:.4f}, eyecup_y={eye_cy:.4f}",
        )
        ctx.check(
            f"{barrel.name} eyepiece axis near +/-32 mm (IPD ~64 mm)",
            abs(eye_cy - side * EYE_Y) < 0.004,
            details=f"eyecup_y={eye_cy:.4f}",
        )
        ctx.check(
            f"{barrel.name} eyecup at the rear, objective at the front",
            eye_bb[1][0] < obj_bb[0][0],
            details=f"eyecup_max_x={eye_bb[1][0]:.4f}, objective_min_x={obj_bb[0][0]:.4f}",
        )

    # --- Recessed lens discs at both ends -------------------------------------
    for barrel, side in ((left, +1), (right, -1)):
        tube_bb = ctx.part_element_world_aabb(barrel, elem="objective_tube")
        lens_bb = ctx.part_element_world_aabb(barrel, elem="objective_lens")
        ctx.check(
            f"{barrel.name} objective lens recessed behind the front rim",
            lens_bb[1][0] <= tube_bb[1][0] - 0.008,
            details=f"lens_max_x={lens_bb[1][0]:.4f}, tube_max_x={tube_bb[1][0]:.4f}",
        )
        ctx.check(
            f"{barrel.name} objective lens inside the tube bore (yz)",
            lens_bb[0][1] >= tube_bb[0][1]
            and lens_bb[1][1] <= tube_bb[1][1]
            and lens_bb[0][2] >= tube_bb[0][2]
            and lens_bb[1][2] <= tube_bb[1][2],
            details=f"lens_bb={lens_bb}, tube_bb={tube_bb}",
        )
        i = 0 if side > 0 else 1
        collar = object_model.get_part(f"eyecup_collar_{i}")
        cup_bb = ctx.part_element_world_aabb(collar, elem="collar")
        ocu_bb = ctx.part_element_world_aabb(barrel, elem="ocular_lens")
        ctx.check(
            f"{barrel.name} ocular lens recessed inside the eyecup collar",
            ocu_bb[0][0] >= cup_bb[0][0] + 0.001 and ocu_bb[1][0] <= cup_bb[1][0],
            details=f"ocular_bb_x=({ocu_bb[0][0]:.4f},{ocu_bb[1][0]:.4f}), "
            f"collar_bb_x=({cup_bb[0][0]:.4f},{cup_bb[1][0]:.4f})",
        )

    # --- Hinge lugs are seated on the bridge axle ------------------------------
    for barrel in (left, right):
        ctx.expect_contact(barrel, bridge, name=f"{barrel.name} hinge lugs touch the bridge")
        for sleeve in ("front_hinge_sleeve", "rear_hinge_sleeve"):
            ctx.expect_overlap(
                barrel,
                bridge,
                axes="x",
                elem_a=sleeve,
                elem_b="hinge_axle",
                min_overlap=0.012,
                name=f"{barrel.name} {sleeve} captured on the axle",
            )

    # --- Center focus wheel: on-axis, between the eyepieces, continuous --------
    ctx.expect_overlap(
        wheel,
        bridge,
        axes="x",
        elem_a="focus_wheel_knurl",
        elem_b="hinge_axle",
        min_overlap=0.015,
        name="focus wheel seated on the central axle",
    )
    wheel_bb = ctx.part_element_world_aabb(wheel, elem="focus_wheel_knurl")
    wheel_cy = 0.5 * (wheel_bb[0][1] + wheel_bb[1][1])
    wheel_cz = 0.5 * (wheel_bb[0][2] + wheel_bb[1][2])
    ctx.check(
        "focus wheel centered on the hinge axis",
        abs(wheel_cy) < 0.002 and abs(wheel_cz - HINGE_Z) < 0.002,
        details=f"wheel_center=({wheel_cy:.4f},{wheel_cz:.4f})",
    )
    ctx.check(
        "focus wheel joint is continuous about the longitudinal axis",
        focus_joint.articulation_type == ArticulationType.CONTINUOUS
        and abs(focus_joint.axis[0]) > 0.99,
        details=f"type={focus_joint.articulation_type}, axis={focus_joint.axis}",
    )

    # --- Diopter ring on the right eyepiece ------------------------------------
    ctx.expect_overlap(
        diopter,
        right,
        axes="x",
        elem_a="diopter_knurl",
        elem_b="eyepiece_tube",
        min_overlap=0.008,
        name="diopter ring wraps the right eyepiece tube",
    )
    dio_bb = ctx.part_element_world_aabb(diopter, elem="diopter_knurl")
    dio_cy = 0.5 * (dio_bb[0][1] + dio_bb[1][1])
    dio_cz = 0.5 * (dio_bb[0][2] + dio_bb[1][2])
    ctx.check(
        "diopter ring centered on the right eyepiece axis",
        abs(dio_cy + EYE_Y) < 0.002 and abs(dio_cz - HINGE_Z) < 0.002,
        details=f"diopter_center=({dio_cy:.4f},{dio_cz:.4f})",
    )
    dl = diopter_joint.motion_limits
    ctx.check(
        "diopter ring travel is about +/- 60 deg about its own axis",
        dl is not None
        and dl.lower is not None
        and dl.upper is not None
        and abs(dl.lower + DIOPTER_LIMIT) < 0.02
        and abs(dl.upper - DIOPTER_LIMIT) < 0.02
        and abs(diopter_joint.axis[0]) > 0.99,
        details=f"limits=({dl.lower},{dl.upper}), axis={diopter_joint.axis}",
    )

    # --- Twist-up eyecup collars: prismatic mechanism -------------------------
    collar_pairs = [
        (left, "left_barrel", +1, left_collar, left_collar_joint),
        (right, "right_barrel", -1, right_collar, right_collar_joint),
    ]
    for barrel, barrel_name, side, collar, collar_joint in collar_pairs:
        # Joint type and axis
        ctx.check(
            f"{collar.name} joint is prismatic",
            collar_joint.articulation_type == ArticulationType.PRISMATIC,
            details=f"type={collar_joint.articulation_type}",
        )
        ctx.check(
            f"{collar.name} joint axis along viewing direction (-X)",
            abs(collar_joint.axis[0] + 1.0) < 0.01
            and abs(collar_joint.axis[1]) < 0.01
            and abs(collar_joint.axis[2]) < 0.01,
            details=f"axis={collar_joint.axis}",
        )
        # Travel limits
        cl = collar_joint.motion_limits
        ctx.check(
            f"{collar.name} travel is 0 to ~{EYECUP_TRAVEL} m",
            cl is not None
            and cl.lower is not None
            and cl.upper is not None
            and abs(cl.lower) < 0.001
            and abs(cl.upper - EYECUP_TRAVEL) < 0.001,
            details=f"limits=({cl.lower},{cl.upper})",
        )
        # Collar is centered on its eyepiece axis
        collar_bb = ctx.part_element_world_aabb(collar, elem="collar")
        collar_cy = 0.5 * (collar_bb[0][1] + collar_bb[1][1])
        collar_cz = 0.5 * (collar_bb[0][2] + collar_bb[1][2])
        ctx.check(
            f"{collar.name} centered on the eyepiece axis",
            abs(collar_cy - side * EYE_Y) < 0.002 and abs(collar_cz - HINGE_Z) < 0.002,
            details=f"collar_center=({collar_cy:.4f},{collar_cz:.4f})",
        )
        # Collar wraps around the eyepiece ring at rest (overlap along x)
        ctx.expect_overlap(
            collar,
            barrel,
            axes="x",
            elem_a="collar",
            elem_b="eyepiece_ring",
            min_overlap=0.005,
            name=f"{collar.name} wraps the eyepiece ring at rest",
        )
        # Collar stays on the eyepiece axis (yz containment)
        ctx.expect_within(
            collar,
            barrel,
            axes="y",
            inner_elem="collar",
            outer_elem="eyepiece_ring",
            margin=0.003,
            name=f"{collar.name} stays centered on the ring in Y",
        )

    # --- Eyecup extension: collar moves outward when twisted up ---------------
    rest_left_pos = ctx.part_world_position(left_collar)
    rest_right_pos = ctx.part_world_position(right_collar)
    with ctx.pose({left_collar_joint: EYECUP_TRAVEL, right_collar_joint: EYECUP_TRAVEL}):
        ext_left_pos = ctx.part_world_position(left_collar)
        ext_right_pos = ctx.part_world_position(right_collar)
        ctx.check(
            "left eyecup collar extends outward (more negative X) when twisted up",
            rest_left_pos is not None
            and ext_left_pos is not None
            and ext_left_pos[0] < rest_left_pos[0] - 0.005,
            details=f"rest_x={rest_left_pos[0]:.4f}, extended_x={ext_left_pos[0]:.4f}",
        )
        ctx.check(
            "right eyecup collar extends outward (more negative X) when twisted up",
            rest_right_pos is not None
            and ext_right_pos is not None
            and ext_right_pos[0] < rest_right_pos[0] - 0.005,
            details=f"rest_x={rest_right_pos[0]:.4f}, extended_x={ext_right_pos[0]:.4f}",
        )
        # Collar retains engagement with the ring at max extension
        for barrel, collar in ((left, left_collar), (right, right_collar)):
            ctx.expect_overlap(
                collar,
                barrel,
                axes="x",
                elem_a="collar",
                elem_b="eyepiece_ring",
                min_overlap=0.002,
                name=f"{collar.name} retains ring engagement at max extension",
            )

    # --- Interpupillary fold: barrels rotate toward each other -----------------
    rest_left = ctx.part_element_world_aabb(left_collar, elem="collar")
    rest_right = ctx.part_element_world_aabb(right_collar, elem="collar")
    rest_left_cy = 0.5 * (rest_left[0][1] + rest_left[1][1])
    rest_right_cy = 0.5 * (rest_right[0][1] + rest_right[1][1])
    rest_left_cz = 0.5 * (rest_left[0][2] + rest_left[1][2])
    with ctx.pose({left_hinge: -FOLD_LIMIT, right_hinge: FOLD_LIMIT}):
        fold_left = ctx.part_element_world_aabb(left_collar, elem="collar")
        fold_right = ctx.part_element_world_aabb(right_collar, elem="collar")
        fold_left_cy = 0.5 * (fold_left[0][1] + fold_left[1][1])
        fold_right_cy = 0.5 * (fold_right[0][1] + fold_right[1][1])
        fold_left_cz = 0.5 * (fold_left[0][2] + fold_left[1][2])
        ctx.check(
            "folded eyepieces move toward each other (narrower IPD)",
            (rest_left_cy - rest_right_cy) - (fold_left_cy - fold_right_cy) > 0.003,
            details=f"rest_ipd={rest_left_cy - rest_right_cy:.4f}, "
            f"fold_ipd={fold_left_cy - fold_right_cy:.4f}",
        )
        ctx.check(
            "folded barrels swing about the longitudinal hinge (eyecups drop)",
            fold_left_cz < rest_left_cz - 0.008,
            details=f"rest_z={rest_left_cz:.4f}, fold_z={fold_left_cz:.4f}",
        )
        ctx.expect_gap(
            left,
            right,
            axis="y",
            min_gap=0.001,
            positive_elem="prism_housing",
            negative_elem="prism_housing",
            name="fully folded barrel housings keep clearance",
        )

    return ctx.report()


object_model = build_object_model()
