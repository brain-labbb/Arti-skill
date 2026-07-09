from __future__ import annotations

"""Compact reverse-Porro binoculars.

Layout (world frame):
- +X is the viewing direction: objective lenses face +X, eyecups face -X.
- +Z is up; the binoculars rest on their objective tubes near z = 0.
- The central hinge bridge axle runs along X at y = 0, z = HINGE_Z.

Reverse-Porro geometry:
- Wide objective tubes sit INBOARD (lateral offset ~0.026 m from hinge).
- Eyepieces splay OUTBOARD (lateral offset ~0.038 m from hinge).
- The stepped prism housing bridges from the inboard objective axis to
  the outboard eyepiece axis.

Kinematic tree:
- hinge_bridge (root): central axle + end caps.
- left_barrel / right_barrel: revolute about the longitudinal axle axis
  (interpupillary fold, +/- 25 deg each).
- focus_wheel: continuous rotation about the same central axis.
- diopter_ring: revolute about the right eyepiece viewing axis (+/- 60 deg).
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

HINGE_Z = 0.025       # world height of the central hinge axle axis
OBJ_Y = 0.026         # lateral offset of each objective tube axis (INBOARD)
EYE_Y = 0.038         # lateral offset of each eyepiece axis (OUTBOARD)
OBJ_TUBE_R = 0.024    # objective bell outer radius (~0.048 m diameter)
EYE_TUBE_R = 0.011    # slim eyepiece tube radius
FOLD_LIMIT = math.radians(25.0)
DIOPTER_LIMIT = math.radians(60.0)
FOCUS_WHEEL_X = -0.032
DIOPTER_X = -0.038

# Visual rpy that maps a +Z-aligned cylinder/lathe/knob onto the +X axis.
ROT_Z_TO_PX = (0.0, math.pi / 2.0, 0.0)
# Same, but the lathe's open end (profile +Z) faces -X instead.
ROT_Z_TO_NX = (0.0, -math.pi / 2.0, 0.0)

# Hinge-lug axial stations (sleeve centers along the axle), interleaved so the
# left and right barrel lugs alternate like a real center hinge.
LEFT_SLEEVE_XS = (0.020, 0.000)
RIGHT_SLEEVE_XS = (0.010, -0.010)
SLEEVE_LEN = 0.014
SLEEVE_R = 0.0075
AXLE_R = 0.005


def _objective_tube_mesh(tag: str):
    """Hollow tapered objective bell, axis +Z, rear at z=0, open mouth at z=0.055."""
    outer = [
        (0.0180, 0.000),
        (0.0180, 0.008),
        (0.0210, 0.030),
        (OBJ_TUBE_R, 0.048),
        (OBJ_TUBE_R, 0.055),
    ]
    inner = [
        (0.0000, 0.040),
        (0.0160, 0.042),
        (0.0180, 0.046),
        (0.0200, 0.055),
    ]
    geom = LatheGeometry.from_shell_profiles(outer, inner, segments=48)
    return mesh_from_geometry(geom, f"{tag}_objective_tube")


def _eyecup_mesh(tag: str):
    """Soft rubber eyecup shell, axis +Z, mount face at z=0, open mouth at z=0.020."""
    outer = [
        (0.0125, 0.000),
        (0.0140, 0.003),
        (0.0140, 0.014),
        (0.0145, 0.018),
        (0.0145, 0.020),
    ]
    inner = [
        (0.0000, 0.008),
        (0.0105, 0.009),
        (0.0115, 0.014),
        (0.0120, 0.020),
    ]
    geom = LatheGeometry.from_shell_profiles(outer, inner, segments=40)
    return mesh_from_geometry(geom, f"{tag}_eyecup")


def _housing_mesh(tag: str):
    """Rounded prism-housing block, extruded along +Z (rotated onto X at placement).

    For reverse-Porro: the housing bridges from the inboard objective axis to the
    outboard eyepiece axis. Its Y-extent covers OBJ_Y to EYE_Y on one side.

    ExtrudeGeometry centers the extrusion at z=0 (±depth/2), so the mesh
    extends ±0.020 in local Z → ±0.020 in world X after rotation.
    """
    # Profile X → height after rotation, profile Y → lateral (barrel-local Y)
    profile = rounded_rect_profile(0.040, 0.046, 0.008)
    geom = ExtrudeGeometry(profile, 0.040)
    return mesh_from_geometry(geom, f"{tag}_prism_housing")


def _add_barrel(model: ArticulatedObject, name: str, side: int, sleeve_xs, mats: dict) -> object:
    """Build one barrel. `side` is +1 (left, +Y) or -1 (right, -Y).

    The barrel part frame sits on the central hinge axis (y=0, z=HINGE_Z world)
    so the fold articulation rotates it about its own local +X.

    Reverse-Porro layout: objective tube INBOARD, eyepiece OUTBOARD.
    """
    armor = mats["rubber_armor"]
    metal = mats["hinge_metal"]
    rubber = mats["eyecup_rubber"]
    lens_amber = mats["lens_amber"]
    lens_dark = mats["ocular_glass"]

    barrel = model.part(name)

    # --- Wide tapered objective tube (INBOARD, near hinge centerline) ---------
    # Rear face coplanar with housing front face at X=0.025.
    barrel.visual(
        _objective_tube_mesh(name),
        origin=Origin(xyz=(0.025, side * OBJ_Y, 0.0), rpy=ROT_Z_TO_PX),
        material=armor,
        name="objective_tube",
    )
    barrel.visual(
        Cylinder(radius=0.019, length=0.005),
        origin=Origin(xyz=(0.065, side * OBJ_Y, 0.0), rpy=ROT_Z_TO_PX),
        material=lens_amber,
        name="objective_lens",
    )

    # --- Stepped prism housing bridging inboard objective to outboard eyepiece -
    # ExtrudeGeometry centers at z=0, so ±0.020 around origin_x=0.005
    # gives housing from X=-0.015 to X=0.025.
    housing_cy = (OBJ_Y + EYE_Y) / 2.0  # 0.032
    barrel.visual(
        _housing_mesh(name),
        origin=Origin(xyz=(0.005, side * housing_cy, 0.0), rpy=ROT_Z_TO_PX),
        material=armor,
        name="prism_housing",
    )

    # --- Slim eyepiece tube (OUTBOARD, farther from hinge) --------------------
    # Front face coplanar with housing rear face at X=-0.015.
    barrel.visual(
        Cylinder(radius=EYE_TUBE_R, length=0.034),
        origin=Origin(xyz=(-0.032, side * EYE_Y, 0.0), rpy=ROT_Z_TO_PX),
        material=metal,
        name="eyepiece_tube",
    )
    barrel.visual(
        Cylinder(radius=0.013, length=0.007),
        origin=Origin(xyz=(-0.044, side * EYE_Y, 0.0), rpy=ROT_Z_TO_PX),
        material=metal,
        name="eyepiece_ring",
    )
    barrel.visual(
        _eyecup_mesh(name),
        origin=Origin(xyz=(-0.047, side * EYE_Y, 0.0), rpy=ROT_Z_TO_NX),
        material=rubber,
        name="eyecup",
    )
    barrel.visual(
        Cylinder(radius=0.0115, length=0.004),
        origin=Origin(xyz=(-0.056, side * EYE_Y, 0.0), rpy=ROT_Z_TO_PX),
        material=lens_dark,
        name="ocular_lens",
    )

    # --- Hinge lugs: sleeve captured on the axle + arm into housing -----------
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
            Box((SLEEVE_LEN, 0.016, 0.016)),
            origin=Origin(xyz=(sx, side * 0.013, 0.0)),
            material=metal,
            name=arm_name,
        )

    return barrel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="reverse_porro_compact_binoculars")

    mats = {
        "rubber_armor": model.material("rubber_armor", rgba=(0.090, 0.095, 0.100, 1.0)),
        "hinge_metal": model.material("hinge_metal", rgba=(0.280, 0.290, 0.310, 1.0)),
        "eyecup_rubber": model.material("eyecup_rubber", rgba=(0.120, 0.120, 0.130, 1.0)),
        "lens_amber": model.material("lens_amber", rgba=(0.500, 0.150, 0.070, 1.0)),
        "ocular_glass": model.material("ocular_glass", rgba=(0.100, 0.075, 0.065, 1.0)),
        "wheel_black": model.material("wheel_black", rgba=(0.070, 0.070, 0.080, 1.0)),
    }
    metal = mats["hinge_metal"]

    # --- Root: central hinge bridge -----------------------------------------
    bridge = model.part("hinge_bridge")
    bridge.visual(
        Cylinder(radius=AXLE_R, length=0.080),
        origin=Origin(xyz=(-0.008, 0.0, HINGE_Z), rpy=ROT_Z_TO_PX),
        material=metal,
        name="hinge_axle",
    )
    bridge.visual(
        Cylinder(radius=0.0075, length=0.005),
        origin=Origin(xyz=(0.034, 0.0, HINGE_Z), rpy=ROT_Z_TO_PX),
        material=metal,
        name="front_hinge_cap",
    )
    bridge.visual(
        Cylinder(radius=0.0075, length=0.005),
        origin=Origin(xyz=(-0.050, 0.0, HINGE_Z), rpy=ROT_Z_TO_PX),
        material=metal,
        name="rear_hinge_cap",
    )

    # --- Mirrored reverse-Porro barrels -------------------------------------
    left_barrel = _add_barrel(model, "left_barrel", +1, LEFT_SLEEVE_XS, mats)
    right_barrel = _add_barrel(model, "right_barrel", -1, RIGHT_SLEEVE_XS, mats)

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
        0.022,
        0.016,
        body_style="cylindrical",
        grip=KnobGrip(style="knurled", count=30, depth=0.0007, helix_angle_deg=0.0),
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
        0.027,
        0.010,
        body_style="cylindrical",
        grip=KnobGrip(style="knurled", count=26, depth=0.0007, helix_angle_deg=0.0),
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

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bridge = object_model.get_part("hinge_bridge")
    left = object_model.get_part("left_barrel")
    right = object_model.get_part("right_barrel")
    wheel = object_model.get_part("focus_wheel")
    diopter = object_model.get_part("diopter_ring")
    left_hinge = object_model.get_articulation("bridge_to_left_barrel")
    right_hinge = object_model.get_articulation("bridge_to_right_barrel")
    focus_joint = object_model.get_articulation("bridge_to_focus_wheel")
    diopter_joint = object_model.get_articulation("right_barrel_to_diopter_ring")

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

    # --- Compact overall envelope (~0.15 x 0.12 x 0.05 m) --------------------
    aabbs = [ctx.part_world_aabb(p) for p in (bridge, left, right, wheel, diopter)]
    lo = [min(a[0][i] for a in aabbs) for i in range(3)]
    hi = [max(a[1][i] for a in aabbs) for i in range(3)]
    ctx.check(
        "compact overall length ~0.13-0.16 m along viewing axis",
        0.115 <= hi[0] - lo[0] <= 0.170,
        details=f"length={hi[0] - lo[0]:.4f}",
    )
    ctx.check(
        "compact overall width ~0.10-0.14 m with barrels open",
        0.090 <= hi[1] - lo[1] <= 0.150,
        details=f"width={hi[1] - lo[1]:.4f}",
    )
    ctx.check(
        "compact overall height ~0.04-0.06 m",
        0.038 <= hi[2] - lo[2] <= 0.060,
        details=f"height={hi[2] - lo[2]:.4f}",
    )
    ctx.check(
        "binoculars rest on the ground plane",
        -0.003 <= lo[2] <= 0.004,
        details=f"min_z={lo[2]:.4f}",
    )

    # --- Reverse-Porro layout: objectives INBOARD, eyepieces OUTBOARD ---------
    for barrel, side in ((left, +1), (right, -1)):
        obj_bb = ctx.part_element_world_aabb(barrel, elem="objective_tube")
        eye_bb = ctx.part_element_world_aabb(barrel, elem="eyecup")
        obj_cy = 0.5 * (obj_bb[0][1] + obj_bb[1][1])
        eye_cy = 0.5 * (eye_bb[0][1] + eye_bb[1][1])
        ctx.check(
            f"{barrel.name} objective axis sits INBOARD of its eyepiece axis (reverse-Porro)",
            side * eye_cy > side * obj_cy + 0.006,
            details=f"objective_y={obj_cy:.4f}, eyecup_y={eye_cy:.4f}",
        )
        ctx.check(
            f"{barrel.name} objective axis near +/-{OBJ_Y*1000:.0f} mm (inboard)",
            abs(obj_cy - side * OBJ_Y) < 0.005,
            details=f"objective_y={obj_cy:.4f}, expected={side * OBJ_Y:.4f}",
        )
        ctx.check(
            f"{barrel.name} eyepiece axis near +/-{EYE_Y*1000:.0f} mm (outboard)",
            abs(eye_cy - side * EYE_Y) < 0.005,
            details=f"eyecup_y={eye_cy:.4f}, expected={side * EYE_Y:.4f}",
        )
        ctx.check(
            f"{barrel.name} eyecup at the rear, objective at the front",
            eye_bb[1][0] < obj_bb[0][0],
            details=f"eyecup_max_x={eye_bb[1][0]:.4f}, objective_min_x={obj_bb[0][0]:.4f}",
        )

    # --- Recessed lens discs at both ends -------------------------------------
    for barrel in (left, right):
        tube_bb = ctx.part_element_world_aabb(barrel, elem="objective_tube")
        lens_bb = ctx.part_element_world_aabb(barrel, elem="objective_lens")
        ctx.check(
            f"{barrel.name} objective lens recessed behind the front rim",
            lens_bb[1][0] <= tube_bb[1][0] - 0.005,
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
        cup_bb = ctx.part_element_world_aabb(barrel, elem="eyecup")
        ocu_bb = ctx.part_element_world_aabb(barrel, elem="ocular_lens")
        ctx.check(
            f"{barrel.name} ocular lens recessed inside the eyecup",
            ocu_bb[0][0] >= cup_bb[0][0] + 0.003 and ocu_bb[1][0] <= cup_bb[1][0],
            details=f"ocular_bb_x=({ocu_bb[0][0]:.4f},{ocu_bb[1][0]:.4f}), "
            f"eyecup_bb_x=({cup_bb[0][0]:.4f},{cup_bb[1][0]:.4f})",
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
                min_overlap=0.010,
                name=f"{barrel.name} {sleeve} captured on the axle",
            )

    # --- Center focus wheel: on-axis, between the eyepieces, continuous --------
    ctx.expect_overlap(
        wheel,
        bridge,
        axes="x",
        elem_a="focus_wheel_knurl",
        elem_b="hinge_axle",
        min_overlap=0.012,
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
        min_overlap=0.006,
        name="diopter ring wraps the right eyepiece tube",
    )
    dio_bb = ctx.part_element_world_aabb(diopter, elem="diopter_knurl")
    dio_cy = 0.5 * (dio_bb[0][1] + dio_bb[1][1])
    dio_cz = 0.5 * (dio_bb[0][2] + dio_bb[1][2])
    ctx.check(
        "diopter ring centered on the right eyepiece axis (outboard)",
        abs(dio_cy + EYE_Y) < 0.003 and abs(dio_cz - HINGE_Z) < 0.003,
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

    # --- Interpupillary fold: barrels rotate toward each other -----------------
    rest_left = ctx.part_element_world_aabb(left, elem="eyecup")
    rest_right = ctx.part_element_world_aabb(right, elem="eyecup")
    rest_left_cy = 0.5 * (rest_left[0][1] + rest_left[1][1])
    rest_right_cy = 0.5 * (rest_right[0][1] + rest_right[1][1])
    rest_left_cz = 0.5 * (rest_left[0][2] + rest_left[1][2])
    with ctx.pose({left_hinge: -FOLD_LIMIT, right_hinge: FOLD_LIMIT}):
        fold_left = ctx.part_element_world_aabb(left, elem="eyecup")
        fold_right = ctx.part_element_world_aabb(right, elem="eyecup")
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
            fold_left_cz < rest_left_cz - 0.005,
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