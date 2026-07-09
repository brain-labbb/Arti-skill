from __future__ import annotations

"""Classic 20x50 Porro-prism binoculars — Individual Focus (IF) variant.

Layout (world frame):
- +X is the viewing direction: objective lenses face +X, eyecups face -X.
- +Z is up; the binoculars rest on their objective tubes near z = 0.
- The central hinge bridge axle runs along X at y = 0, z = HINGE_Z.

Kinematic tree:
- hinge_bridge (root): central axle + end caps.
- left_barrel / right_barrel: revolute about the longitudinal axle axis
  (interpupillary fold, +/- 25 deg each).
- focus_ring_0 / focus_ring_1: individual knurled focus rings, one per
  eyepiece, revolute about each eyepiece's own viewing axis (+/- 60 deg).
"""

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobBore,
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
FOCUS_RING_LIMIT = math.radians(60.0)
FOCUS_RING_X = -0.056  # x position of each focus ring on its eyepiece tube

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


def _eyecup_mesh(tag: str):
    """Soft rubber eyecup shell, axis +Z, mount face at z=0, open mouth at z=0.024."""
    outer = [
        (0.0150, 0.000),
        (0.0170, 0.004),
        (0.0170, 0.016),
        (0.0175, 0.021),
        (0.0175, 0.024),
    ]
    inner = [
        (0.0000, 0.0100),
        (0.0130, 0.0115),
        (0.0140, 0.0160),
        (0.0145, 0.0240),
    ]
    geom = LatheGeometry.from_shell_profiles(outer, inner, segments=40)
    return mesh_from_geometry(geom, f"{tag}_eyecup")


def _housing_mesh(tag: str):
    """Rounded prism-housing block, extruded along +Z (rotated onto X at placement)."""
    profile = rounded_rect_profile(0.054, 0.078, 0.012)
    geom = ExtrudeGeometry(profile, 0.070)
    return mesh_from_geometry(geom, f"{tag}_prism_housing")


def _focus_ring_mesh(tag: str):
    """Knurled individual-focus ring for IF binocular eyepieces, axis +Z.

    Annular ring with knurled grip and a central bore sized for the
    eyepiece tube clearance fit.
    """
    geom = KnobGeometry(
        0.036,
        0.016,
        body_style="cylindrical",
        grip=KnobGrip(style="knurled", count=30, depth=0.0008, helix_angle_deg=0.0),
        bore=KnobBore(style="round", diameter=0.028),
    )
    return mesh_from_geometry(geom, f"{tag}_knurl")


def _add_barrel(model: ArticulatedObject, name: str, side: int, sleeve_xs, mats: dict) -> object:
    """Build one barrel. `side` is +1 (left, +Y) or -1 (right, -Y).

    The barrel part frame sits on the central hinge axis (y=0, z=HINGE_Z world)
    so the fold articulation rotates it about its own local +X.
    """
    armor = mats["rubber_armor"]
    metal = mats["hinge_metal"]
    rubber = mats["eyecup_rubber"]
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

    # Slim eyepiece tube with a dark metal ring and a soft rubber eyecup.
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
    barrel.visual(
        _eyecup_mesh(name),
        origin=Origin(xyz=(-0.071, side * EYE_Y, 0.0), rpy=ROT_Z_TO_NX),
        material=rubber,
        name="eyecup",
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
    model = ArticulatedObject(name="porro_prism_binoculars_20x50_if")

    mats = {
        "rubber_armor": model.material("rubber_armor", rgba=(0.100, 0.100, 0.105, 1.0)),
        "hinge_metal": model.material("hinge_metal", rgba=(0.300, 0.310, 0.330, 1.0)),
        "eyecup_rubber": model.material("eyecup_rubber", rgba=(0.130, 0.130, 0.140, 1.0)),
        "lens_amber": model.material("lens_amber", rgba=(0.520, 0.160, 0.080, 1.0)),
        "ocular_glass": model.material("ocular_glass", rgba=(0.110, 0.080, 0.070, 1.0)),
        "ring_black": model.material("ring_black", rgba=(0.080, 0.080, 0.090, 1.0)),
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
    left_barrel = _add_barrel(model, "left_barrel", +1, LEFT_SLEEVE_XS, mats)
    right_barrel = _add_barrel(model, "right_barrel", -1, RIGHT_SLEEVE_XS, mats)

    model.articulation(
        "bridge_to_left_barrel",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=left_barrel,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        # Negative q swings the left (+Y) barrel down-and-inward toward the
        # right barrel; +/- 25 deg of interpupillary fold from the open pose.
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-FOLD_LIMIT, upper=FOLD_LIMIT),
    )
    model.articulation(
        "bridge_to_right_barrel",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=right_barrel,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        # Positive q swings the right (-Y) barrel down-and-inward (mirror fold).
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-FOLD_LIMIT, upper=FOLD_LIMIT),
    )

    # --- Individual focus rings (IF design) ----------------------------------
    # Each eyepiece gets its own knurled focus ring that rotates about
    # that eyepiece's viewing axis for independent focusing.
    sides = (+1, -1)
    barrels = (left_barrel, right_barrel)
    for i in range(2):
        side = sides[i]
        barrel = barrels[i]
        ring = model.part(f"focus_ring_{i}")
        ring.visual(
            _focus_ring_mesh(f"focus_ring_{i}"),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=ROT_Z_TO_PX),
            material=mats["ring_black"],
            name="focus_ring_knurl",
        )
        model.articulation(
            f"barrel_to_focus_ring_{i}",
            ArticulationType.REVOLUTE,
            parent=barrel,
            child=ring,
            # Ring sits on the eyepiece tube, centered on the viewing axis.
            origin=Origin(xyz=(FOCUS_RING_X, side * EYE_Y, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=4.0,
                lower=-FOCUS_RING_LIMIT,
                upper=FOCUS_RING_LIMIT,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bridge = object_model.get_part("hinge_bridge")
    left = object_model.get_part("left_barrel")
    right = object_model.get_part("right_barrel")
    left_hinge = object_model.get_articulation("bridge_to_left_barrel")
    right_hinge = object_model.get_articulation("bridge_to_right_barrel")

    focus_rings = [object_model.get_part(f"focus_ring_{i}") for i in range(2)]
    focus_joints = [object_model.get_articulation(f"barrel_to_focus_ring_{i}") for i in range(2)]

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

    # Each focus ring wraps around its eyepiece tube (clearance-fit ring)
    # and sits adjacent the static eyepiece ring on the same tube.
    parent_barrels = (left, right)
    for i in range(2):
        ctx.allow_overlap(
            focus_rings[i],
            parent_barrels[i],
            elem_a="focus_ring_knurl",
            elem_b="eyepiece_tube",
            reason=f"Focus ring {i} is intentionally captured around the eyepiece tube.",
        )
        ctx.allow_overlap(
            focus_rings[i],
            parent_barrels[i],
            elem_a="focus_ring_knurl",
            elem_b="eyepiece_ring",
            reason=f"Focus ring {i} seats against the static eyepiece ring on the tube.",
        )

    # --- Overall envelope matches the 0.20 x 0.19 x 0.06 m spec --------------
    all_parts = [bridge, left, right] + focus_rings
    aabbs = [ctx.part_world_aabb(p) for p in all_parts]
    lo = [min(a[0][i] for a in aabbs) for i in range(3)]
    hi = [max(a[1][i] for a in aabbs) for i in range(3)]
    ctx.check(
        "overall length ~0.20 m along viewing axis",
        0.185 <= hi[0] - lo[0] <= 0.210,
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
        eye_bb = ctx.part_element_world_aabb(barrel, elem="eyecup")
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
    for barrel in (left, right):
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
        cup_bb = ctx.part_element_world_aabb(barrel, elem="eyecup")
        ocu_bb = ctx.part_element_world_aabb(barrel, elem="ocular_lens")
        ctx.check(
            f"{barrel.name} ocular lens recessed inside the eyecup",
            ocu_bb[0][0] >= cup_bb[0][0] + 0.004 and ocu_bb[1][0] <= cup_bb[1][0],
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
                min_overlap=0.012,
                name=f"{barrel.name} {sleeve} captured on the axle",
            )

    # --- Individual focus rings: IF design checks ----------------------------
    # Verify each focus ring is on its respective eyepiece axis with correct
    # joint type, axis, and travel limits.
    ring_sides = (+1, -1)
    ring_barrels = (left, right)
    for i in range(2):
        ring = focus_rings[i]
        joint = focus_joints[i]
        barrel = ring_barrels[i]
        side = ring_sides[i]
        side_label = "left" if side > 0 else "right"

        # Ring wraps the eyepiece tube (overlap along viewing axis).
        ctx.expect_overlap(
            ring,
            barrel,
            axes="x",
            elem_a="focus_ring_knurl",
            elem_b="eyepiece_tube",
            min_overlap=0.008,
            name=f"focus_ring_{i} wraps the {side_label} eyepiece tube",
        )

        # Ring centered on the eyepiece viewing axis.
        ring_bb = ctx.part_element_world_aabb(ring, elem="focus_ring_knurl")
        ring_cy = 0.5 * (ring_bb[0][1] + ring_bb[1][1])
        ring_cz = 0.5 * (ring_bb[0][2] + ring_bb[1][2])
        ctx.check(
            f"focus_ring_{i} centered on the {side_label} eyepiece axis",
            abs(ring_cy - side * EYE_Y) < 0.002 and abs(ring_cz - HINGE_Z) < 0.002,
            details=f"ring_center=({ring_cy:.4f},{ring_cz:.4f}), "
            f"expected_y={side * EYE_Y:.4f}, expected_z={HINGE_Z:.4f}",
        )

        # Joint is revolute about the viewing (X) axis.
        ctx.check(
            f"focus_ring_{i} joint is revolute about the viewing axis",
            joint.articulation_type == ArticulationType.REVOLUTE
            and abs(joint.axis[0]) > 0.99,
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )

        # Travel is about +/- 60 degrees.
        ml = joint.motion_limits
        ctx.check(
            f"focus_ring_{i} travel is about +/- 60 deg",
            ml is not None
            and ml.lower is not None
            and ml.upper is not None
            and abs(ml.lower + FOCUS_RING_LIMIT) < 0.02
            and abs(ml.upper - FOCUS_RING_LIMIT) < 0.02,
            details=f"limits=({ml.lower},{ml.upper})",
        )

    # Confirm there is NO center focus wheel (IF design removed it).
    all_part_names = [p.name for p in object_model.parts]
    ctx.check(
        "no center focus wheel (IF design)",
        "focus_wheel" not in all_part_names,
        details=f"parts={all_part_names}",
    )

    # Confirm both rings are independent (each on its own barrel parent).
    ctx.check(
        "focus ring 0 parent is left barrel",
        focus_joints[0].parent == "left_barrel",
        details=f"parent={focus_joints[0].parent}",
    )
    ctx.check(
        "focus ring 1 parent is right barrel",
        focus_joints[1].parent == "right_barrel",
        details=f"parent={focus_joints[1].parent}",
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
            fold_left_cz < rest_left_cz - 0.008,
            details=f"rest_z={rest_left_cz:.4f}, fold_z={fold_left_cz:.4f}",
        )
        # The interleaved hinge sleeves straddle y=0 by design, so scope the
        # fold clearance check to the barrel bodies (prism housings).
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
