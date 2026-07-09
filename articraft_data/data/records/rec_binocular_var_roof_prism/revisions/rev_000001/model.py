from __future__ import annotations

"""Modern compact roof-prism binoculars, 20x50 style.

Layout (world frame):
- +X is the viewing direction: objective lenses face +X, eyecups face -X.
- +Z is up; the binoculars rest on their objective bells near z = 0.
- The central hinge bridge axle runs along X at y = 0, z = HINGE_Z.

Kinematic tree:
- hinge_bridge (root): central axle and end caps.
- left_barrel / right_barrel: straight roof-prism tubes (objective and eyepiece
  share one common optical axis), revolute about the longitudinal axle axis
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
    KnobGeometry,
    KnobGrip,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# --- Shared layout constants -------------------------------------------------

HINGE_Z = 0.030        # world height of the central hinge axle axis
IPD_HALF = 0.032       # lateral offset of each barrel axis (IPD ~64 mm)
BARREL_R = 0.021       # main tube outer radius (~42 mm diameter)
OBJ_BELL_R = 0.028     # objective bell outer radius (~56 mm diameter)
BARREL_LENGTH = 0.170  # total barrel body length
FOLD_LIMIT = math.radians(25.0)
DIOPTER_LIMIT = math.radians(60.0)
FOCUS_WHEEL_X = -0.007
DIOPTER_X = -0.068

# Visual rpy that maps a +Z-aligned cylinder/lathe/knob onto the +X axis.
ROT_Z_TO_PX = (0.0, math.pi / 2.0, 0.0)
# Same, but the lathe's open end (profile +Z) faces -X instead.
ROT_Z_TO_NX = (0.0, -math.pi / 2.0, 0.0)

# Hinge-lug axial stations interleaved so left and right alternate on the axle.
# A gap between the inner pair accommodates the center focus wheel.
LEFT_SLEEVE_XS = (0.025, -0.025)
RIGHT_SLEEVE_XS = (0.011, -0.039)
SLEEVE_LEN = 0.014
SLEEVE_R = 0.008
AXLE_R = 0.005
AXLE_LEN = 0.082


def _barrel_body_mesh(tag: str):
    """Hollow straight roof-prism barrel tube.

    Axis +Z, rear (eyepiece) at z=0, front (objective) at z=BARREL_LENGTH.
    The objective and eyepiece share one common optical axis — no lateral step.
    """
    BL = BARREL_LENGTH
    outer = [
        (0.0185, 0.000),      # rear face outer edge
        (0.0185, 0.012),      # eyepiece section
        (0.0200, 0.020),      # transition
        (BARREL_R, 0.028),    # main tube start
        (BARREL_R, 0.132),    # main tube end
        (0.024, 0.145),       # objective bell transition
        (0.027, 0.158),       # objective bell widening
        (OBJ_BELL_R, 0.165),  # objective bell max
        (OBJ_BELL_R, BL),     # front rim
    ]
    inner = [
        (0.0155, 0.000),      # rear bore
        (0.0155, 0.012),      # eyepiece bore
        (0.0170, 0.020),      # transition bore
        (0.0180, 0.028),      # main bore start
        (0.0180, 0.150),      # main bore end
        (0.0240, 0.160),      # objective bore opens
        (0.0250, BL),         # objective front opening
    ]
    geom = LatheGeometry.from_shell_profiles(outer, inner, segments=48)
    return mesh_from_geometry(geom, f"{tag}_barrel_body")


def _eyecup_mesh(tag: str):
    """Soft rubber eyecup shell, axis +Z, mount face at z=0, mouth at z=0.024."""
    outer = [
        (0.0170, 0.000),      # base wider than barrel bore for mesh contact
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


def _add_barrel(model: ArticulatedObject, name: str, side: int,
                sleeve_xs, mats: dict) -> object:
    """Build one straight roof-prism barrel.

    `side` is +1 (left, +Y) or -1 (right, -Y).
    The barrel part frame sits on the central hinge axis (y=0, z=HINGE_Z world)
    so the fold articulation rotates it about its own local +X.
    """
    armor = mats["rubber_armor"]
    metal = mats["hinge_metal"]
    rubber = mats["eyecup_rubber"]
    lens_amber = mats["lens_amber"]
    lens_dark = mats["ocular_glass"]

    barrel = model.part(name)

    x_rear = -BARREL_LENGTH / 2   # -0.085
    barrel_y = side * IPD_HALF    # +/- 0.032

    # --- Straight hollow barrel body (one optical axis, no offset) ----------
    barrel.visual(
        _barrel_body_mesh(name),
        origin=Origin(xyz=(x_rear, barrel_y, 0.0), rpy=ROT_Z_TO_PX),
        material=armor,
        name="barrel_body",
    )

    # --- Objective lens: amber disc recessed inside the front bore ----------
    barrel.visual(
        Cylinder(radius=0.024, length=0.005),
        origin=Origin(xyz=(x_rear + BARREL_LENGTH - 0.012, barrel_y, 0.0),
                      rpy=ROT_Z_TO_PX),
        material=lens_amber,
        name="objective_lens",
    )

    # --- Eyepiece ring: dark metal ring near the rear -----------------------
    barrel.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=Origin(xyz=(x_rear + 0.010, barrel_y, 0.0), rpy=ROT_Z_TO_PX),
        material=metal,
        name="eyepiece_ring",
    )

    # --- Eyecup: soft rubber cup at the rear --------------------------------
    # Shifted 2 mm forward into the barrel bore so the cup base overlaps the
    # barrel body rear section for mesh connectivity.
    barrel.visual(
        _eyecup_mesh(name),
        origin=Origin(xyz=(x_rear + 0.002, barrel_y, 0.0), rpy=ROT_Z_TO_NX),
        material=rubber,
        name="eyecup",
    )

    # --- Ocular lens: dark disc recessed inside the eyecup ------------------
    # Radius slightly exceeds eyecup bore (~0.013 at this station) so the disc
    # contacts the cup wall for mesh connectivity.
    barrel.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(x_rear - 0.012, barrel_y, 0.0), rpy=ROT_Z_TO_PX),
        material=lens_dark,
        name="ocular_lens",
    )

    # --- Hinge lugs: sleeves on the axle + arms to the barrel body ----------
    arm_y_half = (IPD_HALF - BARREL_R + 0.002) / 2
    sleeve_names = ("front_hinge_sleeve", "rear_hinge_sleeve")
    arm_names = ("front_hinge_arm", "rear_hinge_arm")
    for i in range(len(sleeve_xs)):
        sx = sleeve_xs[i]
        barrel.visual(
            Cylinder(radius=SLEEVE_R, length=SLEEVE_LEN),
            origin=Origin(xyz=(sx, 0.0, 0.0), rpy=ROT_Z_TO_PX),
            material=metal,
            name=sleeve_names[i],
        )
        barrel.visual(
            Box((SLEEVE_LEN, IPD_HALF - BARREL_R + 0.002, 0.016)),
            origin=Origin(xyz=(sx, side * arm_y_half, 0.0)),
            material=metal,
            name=arm_names[i],
        )

    return barrel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_prism_binoculars_20x50")

    mats = {
        "rubber_armor": model.material(
            "rubber_armor", rgba=(0.100, 0.100, 0.105, 1.0)),
        "hinge_metal": model.material(
            "hinge_metal", rgba=(0.300, 0.310, 0.330, 1.0)),
        "eyecup_rubber": model.material(
            "eyecup_rubber", rgba=(0.130, 0.130, 0.140, 1.0)),
        "lens_amber": model.material(
            "lens_amber", rgba=(0.520, 0.160, 0.080, 1.0)),
        "ocular_glass": model.material(
            "ocular_glass", rgba=(0.110, 0.080, 0.070, 1.0)),
        "wheel_black": model.material(
            "wheel_black", rgba=(0.080, 0.080, 0.090, 1.0)),
    }
    metal = mats["hinge_metal"]

    # --- Root: central hinge bridge -----------------------------------------
    bridge = model.part("hinge_bridge")
    axle_cx = -0.007  # axle centred between the inner lug pair
    bridge.visual(
        Cylinder(radius=AXLE_R, length=AXLE_LEN),
        origin=Origin(xyz=(axle_cx, 0.0, HINGE_Z), rpy=ROT_Z_TO_PX),
        material=metal,
        name="hinge_axle",
    )
    cap_x_front = axle_cx + AXLE_LEN / 2
    cap_x_rear = axle_cx - AXLE_LEN / 2
    bridge.visual(
        Cylinder(radius=0.009, length=0.005),
        origin=Origin(xyz=(cap_x_front, 0.0, HINGE_Z), rpy=ROT_Z_TO_PX),
        material=metal,
        name="front_hinge_cap",
    )
    bridge.visual(
        Cylinder(radius=0.009, length=0.005),
        origin=Origin(xyz=(cap_x_rear, 0.0, HINGE_Z), rpy=ROT_Z_TO_PX),
        material=metal,
        name="rear_hinge_cap",
    )

    # --- Mirrored straight barrels -------------------------------------------
    left_barrel = _add_barrel(model, "left_barrel", +1, LEFT_SLEEVE_XS, mats)
    right_barrel = _add_barrel(model, "right_barrel", -1, RIGHT_SLEEVE_XS, mats)

    model.articulation(
        "bridge_to_left_barrel",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=left_barrel,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0,
            lower=-FOLD_LIMIT, upper=FOLD_LIMIT),
    )
    model.articulation(
        "bridge_to_right_barrel",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=right_barrel,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0,
            lower=-FOLD_LIMIT, upper=FOLD_LIMIT),
    )

    # --- Knurled centre focus wheel ------------------------------------------
    focus_wheel = model.part("focus_wheel")
    wheel_geom = KnobGeometry(
        0.010,
        0.018,
        body_style="cylindrical",
        grip=KnobGrip(
            style="knurled", count=30, depth=0.0006, helix_angle_deg=0.0),
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

    # --- Right eyepiece diopter ring ----------------------------------------
    diopter_ring = model.part("diopter_ring")
    diopter_geom = KnobGeometry(
        0.023,
        0.012,
        body_style="cylindrical",
        grip=KnobGrip(
            style="knurled", count=30, depth=0.0008, helix_angle_deg=0.0),
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
        origin=Origin(xyz=(DIOPTER_X, -IPD_HALF, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0, velocity=4.0,
            lower=-DIOPTER_LIMIT, upper=DIOPTER_LIMIT),
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
                barrel, bridge,
                elem_a=sleeve, elem_b="hinge_axle",
                reason="Hinge lug sleeve is intentionally captured around "
                       "the central axle.",
            )
    ctx.allow_overlap(
        wheel, bridge,
        elem_a="focus_wheel_knurl", elem_b="hinge_axle",
        reason="Centre focus wheel is intentionally captured on the "
               "central axle.",
    )
    ctx.allow_overlap(
        diopter, right,
        elem_a="diopter_knurl", elem_b="barrel_body",
        reason="Diopter ring is intentionally captured around the right "
               "barrel eyepiece section.",
    )

    # --- Roof-prism straight-axis claim: objective & eyepiece share one axis -
    for barrel, side in ((left, +1), (right, -1)):
        obj_bb = ctx.part_element_world_aabb(barrel, elem="objective_lens")
        cup_bb = ctx.part_element_world_aabb(barrel, elem="eyecup")
        obj_cy = 0.5 * (obj_bb[0][1] + obj_bb[1][1])
        cup_cy = 0.5 * (cup_bb[0][1] + cup_bb[1][1])
        obj_cz = 0.5 * (obj_bb[0][2] + obj_bb[1][2])
        cup_cz = 0.5 * (cup_bb[0][2] + cup_bb[1][2])
        ctx.check(
            f"{barrel.name} objective and eyepiece share the same Y axis "
            f"(roof-prism straight tube)",
            abs(obj_cy - cup_cy) < 0.004,
            details=f"obj_y={obj_cy:.4f}, eyecup_y={cup_cy:.4f}",
        )
        ctx.check(
            f"{barrel.name} objective and eyepiece share the same Z axis "
            f"(roof-prism straight tube)",
            abs(obj_cz - cup_cz) < 0.004,
            details=f"obj_z={obj_cz:.4f}, eyecup_z={cup_cz:.4f}",
        )

    # --- IPD ~64 mm centre-to-centre ----------------------------------------
    left_bb = ctx.part_element_world_aabb(left, elem="barrel_body")
    right_bb = ctx.part_element_world_aabb(right, elem="barrel_body")
    left_cy = 0.5 * (left_bb[0][1] + left_bb[1][1])
    right_cy = 0.5 * (right_bb[0][1] + right_bb[1][1])
    ipd = left_cy - right_cy
    ctx.check(
        "interpupillary distance ~64 mm centre-to-centre",
        0.060 <= ipd <= 0.068,
        details=f"IPD={ipd:.4f}",
    )

    # --- Barrel diameter ~42 mm (main tube) ---------------------------------
    for barrel in (left, right):
        bb = ctx.part_element_world_aabb(barrel, elem="barrel_body")
        dy = bb[1][1] - bb[0][1]
        dz = bb[1][2] - bb[0][2]
        max_dia = max(dy, dz)
        ctx.check(
            f"{barrel.name} body max diameter includes 42 mm main tube",
            max_dia >= 0.040,
            details=f"max_diameter={max_dia:.4f}",
        )

    # --- Eyecup at rear, objective at front ----------------------------------
    for barrel in (left, right):
        obj_bb = ctx.part_element_world_aabb(barrel, elem="objective_lens")
        cup_bb = ctx.part_element_world_aabb(barrel, elem="eyecup")
        ctx.check(
            f"{barrel.name} eyecup at rear, objective at front",
            cup_bb[1][0] < obj_bb[0][0],
            details=f"eyecup_max_x={cup_bb[1][0]:.4f}, "
                    f"objective_min_x={obj_bb[0][0]:.4f}",
        )

    # --- Recessed lens discs at both ends ------------------------------------
    for barrel in (left, right):
        tube_bb = ctx.part_element_world_aabb(barrel, elem="barrel_body")
        lens_bb = ctx.part_element_world_aabb(barrel, elem="objective_lens")
        ctx.check(
            f"{barrel.name} objective lens recessed behind the front rim",
            lens_bb[1][0] <= tube_bb[1][0] - 0.006,
            details=f"lens_max_x={lens_bb[1][0]:.4f}, "
                    f"tube_max_x={tube_bb[1][0]:.4f}",
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
            ocu_bb[0][0] >= cup_bb[0][0] + 0.003
            and ocu_bb[1][0] <= cup_bb[1][0],
            details=f"ocular_bb_x=({ocu_bb[0][0]:.4f},{ocu_bb[1][0]:.4f}), "
                    f"eyecup_bb_x=({cup_bb[0][0]:.4f},{cup_bb[1][0]:.4f})",
        )

    # --- Hinge lugs seated on the bridge axle --------------------------------
    for barrel in (left, right):
        ctx.expect_contact(
            barrel, bridge,
            name=f"{barrel.name} hinge lugs touch the bridge",
        )
        for sleeve in ("front_hinge_sleeve", "rear_hinge_sleeve"):
            ctx.expect_overlap(
                barrel, bridge,
                axes="x",
                elem_a=sleeve, elem_b="hinge_axle",
                min_overlap=0.010,
                name=f"{barrel.name} {sleeve} captured on the axle",
            )

    # --- Centre focus wheel: on-axis, continuous -----------------------------
    ctx.expect_overlap(
        wheel, bridge,
        axes="x",
        elem_a="focus_wheel_knurl", elem_b="hinge_axle",
        min_overlap=0.014,
        name="focus wheel seated on the central axle",
    )
    wheel_bb = ctx.part_element_world_aabb(wheel, elem="focus_wheel_knurl")
    wheel_cy = 0.5 * (wheel_bb[0][1] + wheel_bb[1][1])
    wheel_cz = 0.5 * (wheel_bb[0][2] + wheel_bb[1][2])
    ctx.check(
        "focus wheel centred on the hinge axis",
        abs(wheel_cy) < 0.003 and abs(wheel_cz - HINGE_Z) < 0.003,
        details=f"wheel_centre=({wheel_cy:.4f},{wheel_cz:.4f})",
    )
    ctx.check(
        "focus wheel joint is continuous about the longitudinal axis",
        focus_joint.articulation_type == ArticulationType.CONTINUOUS
        and abs(focus_joint.axis[0]) > 0.99,
        details=f"type={focus_joint.articulation_type}, "
                f"axis={focus_joint.axis}",
    )

    # --- Diopter ring on the right barrel eyepiece ---------------------------
    ctx.expect_overlap(
        diopter, right,
        axes="x",
        elem_a="diopter_knurl", elem_b="barrel_body",
        min_overlap=0.008,
        name="diopter ring wraps the right barrel eyepiece section",
    )
    dio_bb = ctx.part_element_world_aabb(diopter, elem="diopter_knurl")
    dio_cy = 0.5 * (dio_bb[0][1] + dio_bb[1][1])
    dio_cz = 0.5 * (dio_bb[0][2] + dio_bb[1][2])
    ctx.check(
        "diopter ring centred on the right barrel axis",
        abs(dio_cy + IPD_HALF) < 0.003 and abs(dio_cz - HINGE_Z) < 0.003,
        details=f"diopter_centre=({dio_cy:.4f},{dio_cz:.4f})",
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
        details=f"limits=({dl.lower},{dl.upper}), "
                f"axis={diopter_joint.axis}",
    )

    # --- Interpupillary fold: barrels rotate toward each other ---------------
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
            (rest_left_cy - rest_right_cy)
            - (fold_left_cy - fold_right_cy) > 0.003,
            details=f"rest_ipd={rest_left_cy - rest_right_cy:.4f}, "
                    f"fold_ipd={fold_left_cy - fold_right_cy:.4f}",
        )
        ctx.check(
            "folded barrels swing about the longitudinal hinge (eyecups drop)",
            fold_left_cz < rest_left_cz - 0.006,
            details=f"rest_z={rest_left_cz:.4f}, fold_z={fold_left_cz:.4f}",
        )
        ctx.expect_gap(
            left, right,
            axis="y",
            min_gap=0.000,
            positive_elem="barrel_body",
            negative_elem="barrel_body",
            name="fully folded barrel bodies keep clearance",
        )

    # --- Overall envelope ----------------------------------------------------
    aabbs = [ctx.part_world_aabb(p)
             for p in (bridge, left, right, wheel, diopter)]
    lo = [min(a[0][i] for a in aabbs) for i in range(3)]
    hi = [max(a[1][i] for a in aabbs) for i in range(3)]
    ctx.check(
        "overall length ~0.19 m along viewing axis",
        0.175 <= hi[0] - lo[0] <= 0.210,
        details=f"length={hi[0] - lo[0]:.4f}",
    )
    ctx.check(
        "overall width ~0.12 m (compact roof-prism layout)",
        0.105 <= hi[1] - lo[1] <= 0.135,
        details=f"width={hi[1] - lo[1]:.4f}",
    )
    ctx.check(
        "overall height ~0.06 m",
        0.048 <= hi[2] - lo[2] <= 0.068,
        details=f"height={hi[2] - lo[2]:.4f}",
    )
    ctx.check(
        "binoculars rest near the ground plane",
        -0.004 <= lo[2] <= 0.006,
        details=f"min_z={lo[2]:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
