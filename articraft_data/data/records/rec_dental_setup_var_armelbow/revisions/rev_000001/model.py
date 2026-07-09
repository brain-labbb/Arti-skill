from __future__ import annotations

# Dental treatment unit (operatory setup), modeled from picture/Science/Dental setup/001.png.
#
# Layout (world frame, Z up):
#   +X  : toward the patient's feet (the long, low end of the chair)
#   -X  : toward the patient's head / headrest
#   +Y  : the operator side that carries the delivery column and overhead light post
#   Z   : vertical
#
# Real movable parts that are articulated here:
#   - chair_backrest      REVOLUTE : the patient back/seat reclines about a lateral pivot
#   - upper_arm           REVOLUTE : the first segment of the overhead light arm swings
#                                    horizontally on a vertical post
#   - forearm             REVOLUTE : the second segment folds at the elbow joint
#   - light_head          REVOLUTE : the round reflector head tilts on its yoke
#
# Everything else (floor pedestal, delivery column with monitors / tray / cuspidor,
# clinician stool on a caster base, headrest) is fixed support structure that keeps
# every part mounted and non-floating.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _mesh(name, geometry):
    return mesh_from_geometry(geometry, name)


def _cushion_loft(name, length, half_width, thickness, *, taper=0.7, crown=0.012):
    """Build a soft rounded upholstery slab as a lofted shell.

    The slab runs along local +X, is `length` long, `2*half_width` wide, and
    `thickness` tall. Ends and the top are rounded so it reads as padded foam,
    not a flat box. Returns a managed Mesh.
    """
    n = 5
    sections = []
    for i in range(n):
        t = i / (n - 1)
        x = -length / 2.0 + t * length
        # taper the width slightly toward the ends so corners read rounded
        edge = math.sin(math.pi * t)  # 0 at ends, 1 in middle
        w = half_width * (taper + (1.0 - taper) * edge)
        h_top = thickness * (0.85 + crown / thickness * edge)
        loop = [
            (x, -w, 0.0),
            (x, w, 0.0),
            (x, w * 0.96, h_top),
            (x, -w * 0.96, h_top),
        ]
        sections.append(loop)
    return _mesh(name, section_loft(sections))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dental_treatment_unit")

    # ---- materials -------------------------------------------------------
    upholstery = model.material("upholstery", rgba=(0.30, 0.30, 0.46, 1.0))  # navy/indigo pads
    upholstery_dark = model.material("upholstery_dark", rgba=(0.24, 0.24, 0.38, 1.0))
    cream = model.material("cream", rgba=(0.86, 0.82, 0.66, 1.0))  # tan/cream arms & column
    cream_dark = model.material("cream_dark", rgba=(0.72, 0.67, 0.52, 1.0))
    metal = model.material("metal", rgba=(0.55, 0.56, 0.58, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.23, 0.25, 1.0))
    screen = model.material("screen", rgba=(0.36, 0.55, 0.66, 1.0))
    glass = model.material("glass", rgba=(0.70, 0.82, 0.88, 1.0))
    ceramic = model.material("ceramic", rgba=(0.90, 0.91, 0.92, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.83, 1.0))

    # =====================================================================
    # ROOT: floor pedestal that carries the whole chair + light post.
    # =====================================================================
    base = model.part("base")
    # Operatory floor pad. Spans the whole setup so the chair pedestal,
    # delivery column, and stool all physically rest on (and connect through)
    # one grounded body. Thin so it reads as a floor mat, not a riser.
    base.visual(
        Box((1.30, 1.40, 0.02)),
        origin=Origin(xyz=(0.05, -0.05, 0.01)),
        material=dark_metal,
        name="floor_pad",
    )
    # tapered chair pedestal column rising from the pad
    pedestal_profile = [
        (0.0, 0.02),
        (0.18, 0.02),
        (0.17, 0.26),
        (0.15, 0.40),
        (0.13, 0.46),
        (0.0, 0.46),
    ]
    base.visual(
        _mesh("pedestal", LatheGeometry(pedestal_profile, segments=40)),
        origin=Origin(xyz=(0.06, 0.0, 0.0)),
        material=cream,
        name="chair_pedestal",
    )
    # horizontal tilt-mechanism housing that the seat sits on
    base.visual(
        Box((0.70, 0.34, 0.16)),
        origin=Origin(xyz=(0.06, 0.0, 0.50)),
        material=cream_dark,
        name="seat_carrier",
    )

    # Vertical post (rises from the operator-side rear of the pad) that the
    # overhead light arm pivots on. Modeled as part of the fixed base.
    # Placed clear of the delivery column footprint.
    post_x, post_y = -0.55, 0.32
    base.visual(
        Cylinder(radius=0.045, length=1.78),
        origin=Origin(xyz=(post_x, post_y, 0.91)),
        material=cream,
        name="light_post",
    )
    # foot flange that ties the post down to the floor pad
    base.visual(
        Cylinder(radius=0.075, length=0.06),
        origin=Origin(xyz=(post_x, post_y, 0.04)),
        material=cream_dark,
        name="light_post_foot",
    )

    # =====================================================================
    # PATIENT CHAIR BACKREST  (reclining seat + back as one moving body)
    #   The seat/back reclines about a lateral (Y) pivot at the seat hinge.
    #   Local frame: +X toward feet, headrest stack toward -X.
    # =====================================================================
    backrest = model.part("backrest")
    # main seat + back slab (long padded body)
    backrest.visual(
        _cushion_loft("seat_pad", length=1.30, half_width=0.21, thickness=0.085),
        origin=Origin(xyz=(0.25, 0.0, 0.045)),
        material=upholstery,
        name="seat_pad",
    )
    # raised side bolsters along the seat edges
    backrest.visual(
        _cushion_loft("bolster_l", length=1.20, half_width=0.035, thickness=0.055, taper=0.85),
        origin=Origin(xyz=(0.22, 0.205, 0.07)),
        material=upholstery_dark,
        name="bolster_left",
    )
    backrest.visual(
        _cushion_loft("bolster_r", length=1.20, half_width=0.035, thickness=0.055, taper=0.85),
        origin=Origin(xyz=(0.22, -0.205, 0.07)),
        material=upholstery_dark,
        name="bolster_right",
    )
    # rigid under-shell / backing board the foam is built on (runs the full
    # length so it underlies the headrest neck too)
    backrest.visual(
        Box((1.48, 0.40, 0.075)),
        origin=Origin(xyz=(0.23, 0.0, 0.0075)),
        material=cream_dark,
        name="back_shell",
    )
    # neck/headrest stalk bridging the back shell up to the headrest cushion
    backrest.visual(
        Cylinder(radius=0.020, length=0.16),
        origin=Origin(xyz=(-0.47, 0.0, 0.07), rpy=(0.0, 0.6, 0.0)),
        material=chrome,
        name="headrest_stalk",
    )
    backrest.visual(
        _cushion_loft("headrest_pad", length=0.22, half_width=0.13, thickness=0.07, taper=0.6),
        origin=Origin(xyz=(-0.54, 0.0, 0.12)),
        material=upholstery,
        name="headrest_pad",
    )

    # =====================================================================
    # UPPER ARM  (first segment of the two-segment overhead light arm)
    #   Child of base via a vertical-axis revolute at the post top.
    #   Extends along local +X/-Y from the pivot toward the patient.
    # =====================================================================
    upper_arm = model.part("upper_arm")
    # pivot collar that hugs the post at the swing bearing
    upper_arm.visual(
        Cylinder(radius=0.052, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=cream_dark,
        name="arm_pivot_collar",
    )
    # upper arm tube: a stout cream swept profile from post top to the elbow
    # End point reaches into the knuckle cylinder for connectivity.
    upper_arm_pts = [
        (0.0, 0.0, 0.0),
        (0.10, -0.03, 0.0),
        (0.20, -0.07, -0.003),
        (0.31, -0.12, -0.007),
    ]
    upper_arm.visual(
        _mesh(
            "upper_arm_tube",
            sweep_profile_along_spline(
                upper_arm_pts,
                profile=rounded_rect_profile(0.075, 0.055, radius=0.018),
                samples_per_segment=12,
                cap_profile=True,
            ),
        ),
        material=cream,
        name="upper_arm_tube",
    )
    # elbow knuckle: cylindrical pivot at the far end of the upper arm
    upper_arm.visual(
        Cylinder(radius=0.040, length=0.08),
        origin=Origin(xyz=(0.34, -0.14, -0.01), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="elbow_knuckle",
    )

    # =====================================================================
    # FOREARM  (second segment, folds at the elbow joint)
    #   Child of upper_arm via a lateral-axis revolute at the elbow.
    #   Extends from the elbow to the light-head drop link.
    # =====================================================================
    forearm = model.part("forearm")
    # elbow hub: cylindrical socket at the forearm root that captures the
    # upper-arm elbow knuckle
    forearm.visual(
        Cylinder(radius=0.044, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="elbow_hub",
    )
    # forearm tube: cream swept profile from the elbow hub to the end knuckle
    # Start point reaches into the hub cylinder for connectivity.
    forearm_pts = [
        (0.03, -0.015, -0.002),
        (0.12, -0.07, -0.018),
        (0.23, -0.14, -0.04),
        (0.32, -0.18, -0.05),
    ]
    forearm.visual(
        _mesh(
            "forearm_tube",
            sweep_profile_along_spline(
                forearm_pts,
                profile=rounded_rect_profile(0.065, 0.048, radius=0.016),
                samples_per_segment=12,
                cap_profile=True,
            ),
        ),
        material=cream,
        name="forearm_tube",
    )
    # end knuckle at the far end where the head yoke hangs
    forearm.visual(
        Cylinder(radius=0.038, length=0.07),
        origin=Origin(xyz=(0.32, -0.18, -0.05), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="end_knuckle",
    )
    # short drop link from the end knuckle down to the light head yoke
    forearm.visual(
        Cylinder(radius=0.022, length=0.14),
        origin=Origin(xyz=(0.36, -0.18, -0.12)),
        material=metal,
        name="head_drop_link",
    )

    # =====================================================================
    # LIGHT HEAD  (round reflector that tilts on the yoke)
    #   Child of forearm via a lateral-axis revolute at the drop link.
    #   Local frame: dish opens downward (-Z); handle ring around it.
    # =====================================================================
    light_head = model.part("light_head")
    # reflector housing: shallow domed shell, opening down
    refl_profile = [
        (0.0, 0.060),
        (0.075, 0.055),
        (0.130, 0.035),
        (0.165, 0.008),
        (0.175, -0.010),
        (0.150, -0.010),
        (0.110, -0.005),
        (0.060, 0.012),
        (0.0, 0.020),
    ]
    light_head.visual(
        _mesh("reflector_shell", LatheGeometry(refl_profile, segments=56)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=cream,
        name="reflector_shell",
    )
    # glass lens disc filling the downward opening
    light_head.visual(
        Cylinder(radius=0.160, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=glass,
        name="light_lens",
    )
    # chrome trim ring around the rim
    light_head.visual(
        _mesh(
            "head_rim",
            TorusGeometry(radius=0.166, tube=0.010, radial_segments=14, tubular_segments=64),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome,
        name="head_rim",
    )
    # two side grab handles for the clinician to reposition the light.
    # Inner ends start on the rim (r~0.166) so they connect to the head.
    handle_pts_r = [
        (0.12, -0.118, 0.005),
        (0.19, -0.135, -0.01),
        (0.23, -0.105, -0.02),
        (0.21, -0.055, -0.02),
    ]
    light_head.visual(
        _mesh("handle_right", tube_from_spline_points(handle_pts_r, radius=0.009, radial_segments=12)),
        material=dark_metal,
        name="handle_right",
    )
    handle_pts_l = [(p[0], -p[1], p[2]) for p in handle_pts_r]
    light_head.visual(
        _mesh("handle_left", tube_from_spline_points(handle_pts_l, radius=0.009, radial_segments=12)),
        material=dark_metal,
        name="handle_left",
    )
    # yoke trunnion stub on top that the drop link captures (mount geometry)
    light_head.visual(
        Cylinder(radius=0.020, length=0.06, ),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=metal,
        name="head_yoke_stub",
    )

    # =====================================================================
    # DELIVERY COLUMN  (operator-side cabinet: monitors, tray, cuspidor)
    #   Fixed support. Stands beside the chair on the +Y operator side.
    # =====================================================================
    column = model.part("delivery_column")
    # Column stands at the head/operator side, clear of the chair (seat edge is
    # near y=0.25). Cabinet rests on the floor pad (z=0.02).
    cx, cy = -0.20, 0.58
    column.visual(
        Box((0.34, 0.30, 0.74)),
        origin=Origin(xyz=(cx, cy, 0.39)),
        material=cream,
        name="column_cabinet",
    )
    column.visual(
        Box((0.40, 0.36, 0.05)),
        origin=Origin(xyz=(cx, cy, 0.785)),
        material=cream_dark,
        name="column_top",
    )
    # instrument delivery head sitting on top of the cabinet (patient side)
    column.visual(
        Box((0.30, 0.18, 0.10)),
        origin=Origin(xyz=(cx, cy - 0.12, 0.86)),
        material=cream,
        name="delivery_head",
    )
    # row of instrument holders on the delivery head, hanging toward patient
    for i in range(5):
        hx = cx - 0.12 + i * 0.06
        column.visual(
            Cylinder(radius=0.010, length=0.06),
            origin=Origin(xyz=(hx, cy - 0.20, 0.84), rpy=(0.45, 0.0, 0.0)),
            material=chrome,
            name=f"instrument_{i}",
        )
    # swivel monitor on a short arm above the cabinet (rear/operator side)
    column.visual(
        Cylinder(radius=0.016, length=0.26),
        origin=Origin(xyz=(cx + 0.10, cy + 0.09, 0.90)),
        material=metal,
        name="monitor_arm",
    )
    column.visual(
        Box((0.02, 0.22, 0.16)),
        origin=Origin(xyz=(cx + 0.11, cy + 0.11, 1.00), rpy=(0.0, 0.0, -0.5)),
        material=screen,
        name="monitor_panel",
    )
    # cuspidor (rinse bowl) bracketed off the patient-side of the cabinet.
    # Kept at y >= 0.30 so it clears the reclining chair seat.
    cusp_profile = [
        (0.0, 0.0),
        (0.085, 0.0),
        (0.095, 0.02),
        (0.090, 0.05),
        (0.070, 0.06),
        (0.050, 0.04),
        (0.045, 0.02),
        (0.0, 0.02),
    ]
    column.visual(
        Box((0.10, 0.14, 0.05)),
        origin=Origin(xyz=(cx + 0.02, cy - 0.16, 0.62)),
        material=cream_dark,
        name="cuspidor_bracket",
    )
    column.visual(
        _mesh("cuspidor_bowl", LatheGeometry(cusp_profile, segments=40)),
        origin=Origin(xyz=(cx + 0.02, cy - 0.24, 0.64)),
        material=ceramic,
        name="cuspidor_bowl",
    )
    # tray arm + tray reaching toward the patient, staying clear of the seat
    column.visual(
        Cylinder(radius=0.012, length=0.26),
        origin=Origin(xyz=(cx + 0.18, cy - 0.16, 0.82), rpy=(0.0, math.pi / 2.0, -0.7)),
        material=metal,
        name="tray_arm",
    )
    column.visual(
        Box((0.22, 0.16, 0.012)),
        origin=Origin(xyz=(cx + 0.30, cy - 0.26, 0.82)),
        material=metal,
        name="instrument_tray",
    )

    # =====================================================================
    # CLINICIAN STOOL  (round seat on a 5-star caster base)
    #   Fixed support placed beside the chair on the -Y side.
    # =====================================================================
    stool = model.part("stool")
    sx, sy, sz = 0.30, -0.55, 0.020  # sz lifts the casters onto the floor pad
    # 5-star caster base
    for i in range(5):
        ang = i * (2.0 * math.pi / 5.0)
        leg_pts = [
            (sx, sy, sz + 0.05),
            (sx + 0.13 * math.cos(ang), sy + 0.13 * math.sin(ang), sz + 0.045),
            (sx + 0.24 * math.cos(ang), sy + 0.24 * math.sin(ang), sz + 0.035),
        ]
        stool.visual(
            _mesh(f"stool_leg_{i}", tube_from_spline_points(leg_pts, radius=0.018, radial_segments=12)),
            material=dark_metal,
            name=f"stool_leg_{i}",
        )
        stool.visual(
            Cylinder(radius=0.022, length=0.03),
            origin=Origin(
                xyz=(sx + 0.24 * math.cos(ang), sy + 0.24 * math.sin(ang), sz + 0.018),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=metal,
            name=f"stool_caster_{i}",
        )
    # gas-lift column
    stool.visual(
        Cylinder(radius=0.022, length=0.34),
        origin=Origin(xyz=(sx, sy, sz + 0.22)),
        material=chrome,
        name="stool_column",
    )
    # padded round seat
    stool.visual(
        _mesh(
            "stool_seat",
            LatheGeometry(
                [(0.0, 0.0), (0.16, 0.0), (0.17, 0.03), (0.15, 0.055), (0.0, 0.06)],
                segments=44,
            ),
        ),
        origin=Origin(xyz=(sx, sy, sz + 0.39)),
        material=upholstery,
        name="stool_seat",
    )
    # short curved support post + backrest cushion behind the stool seat
    # support post rises from the seat and leans toward -X to carry the cushion
    stool.visual(
        Cylinder(radius=0.012, length=0.16),
        origin=Origin(xyz=(sx - 0.12, sy, sz + 0.45), rpy=(0.0, -0.5, 0.0)),
        material=chrome,
        name="stool_back_post",
    )
    stool.visual(
        _cushion_loft("stool_back", length=0.24, half_width=0.06, thickness=0.05, taper=0.7),
        origin=Origin(xyz=(sx - 0.17, sy, sz + 0.50), rpy=(0.0, -1.1, 0.0)),
        material=upholstery,
        name="stool_back",
    )

    # =====================================================================
    # ARTICULATIONS
    # =====================================================================
    # Backrest recline: pivot about a lateral (Y) axis at the seat hinge,
    # located at the rear of the seat carrier. Positive q reclines the back
    # (lowers the head end / raises the foot end). Geometry extends +X (feet)
    # and the headrest is at -X. Reclining should lower the head/back end.
    # About axis -Y, positive q rotates the -X end downward (right-hand rule),
    # so the headrest drops as the chair reclines.
    model.articulation(
        "backrest_recline",
        ArticulationType.REVOLUTE,
        parent=base,
        child=backrest,
        origin=Origin(xyz=(0.06, 0.0, 0.61)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=400.0, velocity=0.4, lower=-0.35, upper=0.55),
    )

    # Overhead light swing: vertical-axis revolute at the top of the post.
    # The upper arm is the first segment; positive q swings the arm toward +Y/-Y.
    model.articulation(
        "light_arm_swing",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(-0.55, 0.32, 1.79)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=-1.9, upper=1.9),
    )

    # Elbow fold: lateral-axis revolute at the elbow knuckle. The forearm
    # folds up/down in the vertical plane. About axis -Y, positive q lifts
    # the forearm +X end upward (right-hand rule: +X → +Z).
    # Origin is in upper_arm's local frame, at the elbow knuckle location.
    model.articulation(
        "elbow_fold",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.34, -0.14, -0.01)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2, lower=-0.4, upper=1.6),
    )

    # Light head tilt: lateral-axis revolute at the forearm drop link.
    # The head hangs below the forearm end; tilt aims the beam.
    # Origin is in forearm's local frame, below the drop link bottom.
    model.articulation(
        "light_head_tilt",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=light_head,
        origin=Origin(xyz=(0.36, -0.18, -0.26)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=-0.9, upper=0.9),
    )

    # Fixed mounts for the standalone furniture pieces onto the floor base.
    model.articulation(
        "column_mount",
        ArticulationType.FIXED,
        parent=base,
        child=column,
    )
    model.articulation(
        "stool_mount",
        ArticulationType.FIXED,
        parent=base,
        child=stool,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    backrest = object_model.get_part("backrest")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    light_head = object_model.get_part("light_head")
    column = object_model.get_part("delivery_column")
    stool = object_model.get_part("stool")

    recline = object_model.get_articulation("backrest_recline")
    swing = object_model.get_articulation("light_arm_swing")
    elbow = object_model.get_articulation("elbow_fold")
    tilt = object_model.get_articulation("light_head_tilt")

    # ---- intentional capture fits ---------------------------------------
    # The swing-arm pivot collar wraps the fixed light post (the bearing).
    ctx.allow_overlap(
        upper_arm,
        base,
        elem_a="arm_pivot_collar",
        elem_b="light_post",
        reason="The arm pivot collar is captured around the light post at the swing bearing.",
    )
    # The upper arm tube root also meets the post at the same bearing.
    ctx.allow_overlap(
        upper_arm,
        base,
        elem_a="upper_arm_tube",
        elem_b="light_post",
        reason="The upper arm tube root joins the post at the swing bearing collar.",
    )
    # The forearm elbow hub captures the upper arm elbow knuckle.
    ctx.allow_overlap(
        forearm,
        upper_arm,
        elem_a="elbow_hub",
        elem_b="elbow_knuckle",
        reason="The forearm elbow hub captures the upper arm elbow knuckle at the fold joint.",
    )
    # The elbow hub also contacts the upper arm tube root at the joint.
    ctx.allow_overlap(
        forearm,
        upper_arm,
        elem_a="elbow_hub",
        elem_b="upper_arm_tube",
        reason="The forearm elbow hub seats against the upper arm tube at the fold bearing.",
    )
    # The forearm tube root contacts the upper arm elbow knuckle at the joint.
    ctx.allow_overlap(
        forearm,
        upper_arm,
        elem_a="forearm_tube",
        elem_b="elbow_knuckle",
        reason="The forearm tube root is mounted on the upper arm elbow knuckle at the fold bearing.",
    )
    # The light-head yoke stub is captured inside the forearm drop link.
    ctx.allow_overlap(
        light_head,
        forearm,
        elem_a="head_yoke_stub",
        elem_b="head_drop_link",
        reason="The head yoke stub nests inside the drop link at the tilt joint.",
    )

    # ---- joint type/axis contract ---------------------------------------
    ctx.check(
        "backrest is a lateral-axis revolute recline",
        str(recline.articulation_type).endswith("REVOLUTE")
        and abs(recline.axis[1]) > 0.9
        and abs(recline.axis[0]) < 1e-6
        and abs(recline.axis[2]) < 1e-6,
        details=f"type={recline.articulation_type}, axis={recline.axis}",
    )
    ctx.check(
        "upper arm swings on a vertical axis",
        str(swing.articulation_type).endswith("REVOLUTE") and abs(swing.axis[2]) > 0.9,
        details=f"type={swing.articulation_type}, axis={swing.axis}",
    )
    ctx.check(
        "elbow fold is a lateral-axis revolute",
        str(elbow.articulation_type).endswith("REVOLUTE") and abs(elbow.axis[1]) > 0.9,
        details=f"type={elbow.articulation_type}, axis={elbow.axis}",
    )
    ctx.check(
        "light head tilts on a lateral axis",
        str(tilt.articulation_type).endswith("REVOLUTE") and abs(tilt.axis[1]) > 0.9,
        details=f"type={tilt.articulation_type}, axis={tilt.axis}",
    )

    # ---- two-segment arm structure --------------------------------------
    ctx.check(
        "upper arm has an elbow knuckle",
        upper_arm.get_visual("elbow_knuckle") is not None,
        details="upper arm elbow knuckle missing",
    )
    ctx.check(
        "forearm has an elbow hub and drop link",
        forearm.get_visual("elbow_hub") is not None
        and forearm.get_visual("head_drop_link") is not None,
        details="forearm elbow hub or drop link missing",
    )

    # ---- hero geometry present ------------------------------------------
    ctx.check(
        "reflector head has a glass lens and a rim",
        light_head.get_visual("light_lens") is not None
        and light_head.get_visual("head_rim") is not None,
        details="reflector lens/rim missing",
    )
    ctx.check(
        "delivery column carries a cuspidor and instrument tray",
        column.get_visual("cuspidor_bowl") is not None
        and column.get_visual("instrument_tray") is not None,
        details="column instruments missing",
    )
    ctx.check(
        "chair has a headrest pad",
        backrest.get_visual("headrest_pad") is not None,
        details="headrest missing",
    )

    # ---- spatial sanity at rest -----------------------------------------
    # The overhead light head is above the patient chair seat, and the head
    # hangs below the arm on its drop link.
    with ctx.pose({recline: 0.0, swing: 0.0, elbow: 0.0, tilt: 0.0}):
        head_aabb = ctx.part_world_aabb(light_head)
        seat_aabb = ctx.part_world_aabb(backrest)
        upper_pos = ctx.part_world_position(upper_arm)
        head_pos = ctx.part_world_position(light_head)
        ctx.check(
            "light head hangs above the chair",
            head_aabb is not None
            and seat_aabb is not None
            and head_aabb[0][2] > seat_aabb[1][2] + 0.5,
            details=f"head_z={head_aabb}, seat_z={seat_aabb}",
        )
        ctx.check(
            "light head hangs below the upper arm pivot",
            upper_pos is not None
            and head_pos is not None
            and head_pos[2] < upper_pos[2] - 0.15,
            details=f"upper={upper_pos}, head={head_pos}",
        )
        # head reflector sits roughly over the seat (within the chair footprint)
        ctx.check(
            "light head is positioned over the chair seat",
            head_aabb is not None
            and seat_aabb is not None
            and head_aabb[1][0] > seat_aabb[0][0]
            and head_aabb[0][0] < seat_aabb[1][0],
            details=f"head_x={head_aabb}, seat_x={seat_aabb}",
        )

    # ---- the swing actually moves the head laterally --------------------
    with ctx.pose({swing: 0.0, elbow: 0.0}):
        head_rest = ctx.part_world_position(light_head)
    with ctx.pose({swing: 1.2, elbow: 0.0}):
        head_swung = ctx.part_world_position(light_head)
    ctx.check(
        "swinging the arm moves the light head sideways",
        head_rest is not None
        and head_swung is not None
        and abs(head_swung[1] - head_rest[1]) > 0.2,
        details=f"rest={head_rest}, swung={head_swung}",
    )

    # ---- the elbow fold actually moves the forearm end vertically --------
    with ctx.pose({swing: 0.0, elbow: 0.0, tilt: 0.0}):
        end_rest = ctx.part_element_world_aabb(forearm, elem="end_knuckle")
        head_elbow_rest = ctx.part_world_position(light_head)
    with ctx.pose({swing: 0.0, elbow: 1.2, tilt: 0.0}):
        end_folded = ctx.part_element_world_aabb(forearm, elem="end_knuckle")
        head_elbow_folded = ctx.part_world_position(light_head)
    ctx.check(
        "folding the elbow moves the forearm end vertically",
        end_rest is not None
        and end_folded is not None
        and end_folded[1][2] > end_rest[1][2] + 0.05,
        details=f"rest={end_rest}, folded={end_folded}",
    )
    ctx.check(
        "folding the elbow raises the light head",
        head_elbow_rest is not None
        and head_elbow_folded is not None
        and head_elbow_folded[2] > head_elbow_rest[2] + 0.05,
        details=f"rest={head_elbow_rest}, folded={head_elbow_folded}",
    )

    # ---- recline actually lowers the head end of the chair --------------
    with ctx.pose({recline: 0.0}):
        rest = ctx.part_element_world_aabb(backrest, elem="headrest_pad")
    with ctx.pose({recline: 0.5}):
        reclined = ctx.part_element_world_aabb(backrest, elem="headrest_pad")
    ctx.check(
        "reclining lowers the headrest end",
        rest is not None
        and reclined is not None
        and reclined[0][2] < rest[0][2] - 0.03,
        details=f"rest={rest}, reclined={reclined}",
    )

    # ---- tilt aims the reflector ----------------------------------------
    with ctx.pose({tilt: 0.0}):
        lens_rest = ctx.part_element_world_aabb(light_head, elem="light_lens")
    with ctx.pose({tilt: 0.7}):
        lens_tilt = ctx.part_element_world_aabb(light_head, elem="light_lens")
    ctx.check(
        "tilting the head reorients the lens",
        lens_rest is not None
        and lens_tilt is not None
        and abs(lens_tilt[1][0] - lens_rest[1][0]) > 0.02,
        details=f"rest={lens_rest}, tilt={lens_tilt}",
    )

    # ---- furniture sits on the floor, not floating ----------------------
    stool_aabb = ctx.part_world_aabb(stool)
    ctx.check(
        "stool casters rest near the floor",
        stool_aabb is not None and stool_aabb[0][2] < 0.05,
        details=f"stool_aabb={stool_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
