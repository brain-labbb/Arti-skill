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
#   - light_arm           REVOLUTE : the overhead operatory light swings horizontally on
#                                    a vertical post (the primary user-facing mechanism)
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


def _handpiece_body(name):
    """Dental handpiece as a lathe-profiled pen-like body.

    Axis along local Z: bur tip at z=0 (bottom), hose barb at z≈0.148 (top).
    Sized to a realistic high-speed handpiece (~148 mm overall length, ~20 mm
    max grip diameter).
    """
    profile = [
        (0.0, 0.0),         # center bottom
        (0.002, 0.0),       # bur tip
        (0.002, 0.005),     # bur
        (0.004, 0.008),     # bur-to-head transition
        (0.005, 0.012),     # head
        (0.005, 0.022),     # neck
        (0.008, 0.038),     # body widens toward grip
        (0.010, 0.055),     # grip section start
        (0.010, 0.112),     # grip section end
        (0.009, 0.122),     # taper back
        (0.006, 0.134),     # barb neck
        (0.007, 0.138),     # barb ridge
        (0.006, 0.145),     # hose connection
        (0.0, 0.148),       # close top
    ]
    return _mesh(name, LatheGeometry(profile, segments=28))


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
    # OVERHEAD LIGHT ARM  (horizontal swing arm on the vertical post)
    #   Child of base via a vertical-axis revolute at the post top.
    #   The arm extends along local +X from the pivot toward the patient.
    # =====================================================================
    light_arm = model.part("light_arm")
    # upper rigid arm: a stout cream tube that reaches out over the patient and
    # curves toward -Y so the head lands over the chair centerline.
    arm_pts = [
        (0.0, 0.0, 0.0),
        (0.22, -0.08, 0.0),
        (0.46, -0.20, -0.02),
        (0.66, -0.32, -0.06),
    ]
    light_arm.visual(
        _mesh(
            "upper_arm",
            sweep_profile_along_spline(
                arm_pts,
                profile=rounded_rect_profile(0.075, 0.055, radius=0.018),
                samples_per_segment=12,
                cap_profile=True,
            ),
        ),
        material=cream,
        name="upper_arm",
    )
    # pivot collar that hugs the post at the joint
    light_arm.visual(
        Cylinder(radius=0.052, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=cream_dark,
        name="arm_pivot_collar",
    )
    # spring/tension knuckle at the far end where the head yoke hangs
    light_arm.visual(
        Cylinder(radius=0.040, length=0.07),
        origin=Origin(xyz=(0.66, -0.32, -0.06), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="arm_knuckle",
    )
    # short drop link from the knuckle down to the light head yoke
    light_arm.visual(
        Cylinder(radius=0.022, length=0.14),
        origin=Origin(xyz=(0.70, -0.32, -0.13)),
        material=metal,
        name="head_drop_link",
    )

    # =====================================================================
    # LIGHT HEAD  (round reflector that tilts on the yoke)
    #   Child of light_arm via a lateral-axis revolute at the yoke knuckle.
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
    # row of three handpieces on the delivery head, each with a draped hose
    _hp_count = 3
    _hp_spacing = 0.08
    for i in range(_hp_count):
        hx = cx - (_hp_count - 1) * _hp_spacing / 2.0 + i * _hp_spacing
        # handpiece hangs from the delivery head, tilted slightly forward
        hp_y = cy - 0.20
        hp_z = 0.68
        column.visual(
            _handpiece_body(f"handpiece_{i}"),
            origin=Origin(xyz=(hx, hp_y, hp_z), rpy=(0.35, 0.0, 0.0)),
            material=chrome,
            name=f"handpiece_{i}",
        )
        # draped hose from delivery head top down to the handpiece connection
        hose_start = (hx, cy - 0.04, 0.91)
        hose_end = (
            hx,
            hp_y - 0.148 * math.sin(0.35),
            hp_z + 0.148 * math.cos(0.35),
        )
        mid_y = (hose_start[1] + hose_end[1]) / 2.0
        sag_z = min(hose_start[2], hose_end[2]) - 0.06
        hose_pts = [
            hose_start,
            (hx, hose_start[1] - 0.04, sag_z + 0.06),
            (hx, mid_y, sag_z),
            (hx, hose_end[1] + 0.03, sag_z + 0.04),
            hose_end,
        ]
        column.visual(
            _mesh(
                f"hose_{i}",
                tube_from_spline_points(hose_pts, radius=0.005, radial_segments=10),
            ),
            material=dark_metal,
            name=f"hose_{i}",
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
    # (lowers the head end / raises the foot end). Geometry extends +X (feet),
    # so axis +Y rotates the head (-X) end downward as q increases.
    # Geometry extends +X (feet) and the headrest is at -X. Reclining should
    # lower the head/back end. About axis -Y, positive q rotates the -X end
    # downward (right-hand rule), so the headrest drops as the chair reclines.
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
    # Positive q swings the arm (which reaches out along +X) toward +Y/-Y.
    model.articulation(
        "light_arm_swing",
        ArticulationType.REVOLUTE,
        parent=base,
        child=light_arm,
        origin=Origin(xyz=(-0.55, 0.32, 1.79)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=-1.9, upper=1.9),
    )

    # Light head tilt: lateral-axis revolute at the yoke drop link.
    # The head hangs ~0.135 below the arm's far end; tilt aims the beam.
    model.articulation(
        "light_head_tilt",
        ArticulationType.REVOLUTE,
        parent=light_arm,
        child=light_head,
        origin=Origin(xyz=(0.70, -0.32, -0.265)),
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
    light_arm = object_model.get_part("light_arm")
    light_head = object_model.get_part("light_head")
    column = object_model.get_part("delivery_column")
    stool = object_model.get_part("stool")

    recline = object_model.get_articulation("backrest_recline")
    swing = object_model.get_articulation("light_arm_swing")
    tilt = object_model.get_articulation("light_head_tilt")

    # ---- intentional capture fits ---------------------------------------
    # The swing-arm pivot collar wraps the fixed light post (the bearing).
    ctx.allow_overlap(
        light_arm,
        base,
        elem_a="arm_pivot_collar",
        elem_b="light_post",
        reason="The arm pivot collar is captured around the light post at the swing bearing.",
    )
    # The arm root also meets the post at the same bearing.
    ctx.allow_overlap(
        light_arm,
        base,
        elem_a="upper_arm",
        elem_b="light_post",
        reason="The arm root joins the post at the swing bearing collar.",
    )
    # The light-head yoke stub is captured inside the arm drop link.
    ctx.allow_overlap(
        light_head,
        light_arm,
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
        "light arm swings on a vertical axis",
        str(swing.articulation_type).endswith("REVOLUTE") and abs(swing.axis[2]) > 0.9,
        details=f"type={swing.articulation_type}, axis={swing.axis}",
    )
    ctx.check(
        "light head tilts on a lateral axis",
        str(tilt.articulation_type).endswith("REVOLUTE") and abs(tilt.axis[1]) > 0.9,
        details=f"type={tilt.articulation_type}, axis={tilt.axis}",
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

    # ---- three handpieces with hoses on the delivery head ---------------
    col_visual_names = {v.name for v in column.visuals}
    hp_names = [f"handpiece_{i}" for i in range(3)]
    hose_names = [f"hose_{i}" for i in range(3)]
    ctx.check(
        "delivery head carries three handpieces",
        all(n in col_visual_names for n in hp_names),
        details="one or more handpiece visuals missing",
    )
    ctx.check(
        "each handpiece has a draped hose",
        all(n in col_visual_names for n in hose_names),
        details="one or more hose visuals missing",
    )
    ctx.check(
        "only three handpieces (no fourth)",
        "handpiece_3" not in col_visual_names,
        details="unexpected fourth handpiece found",
    )
    # regular X spacing between handpieces
    hp_centers = []
    for i in range(3):
        aabb = ctx.part_element_world_aabb(column, elem=f"handpiece_{i}")
        if aabb is not None:
            hp_centers.append((aabb[0][0] + aabb[1][0]) / 2.0)
    if len(hp_centers) == 3:
        gap_01 = hp_centers[1] - hp_centers[0]
        gap_12 = hp_centers[2] - hp_centers[1]
        ctx.check(
            "handpieces are regularly spaced along X",
            abs(gap_01 - gap_12) < 0.01 and gap_01 > 0.04,
            details=f"gaps={gap_01:.4f}, {gap_12:.4f}",
        )
    # handpieces hang below the delivery head
    dh_aabb = ctx.part_element_world_aabb(column, elem="delivery_head")
    hp0_aabb = ctx.part_element_world_aabb(column, elem="handpiece_0")
    if dh_aabb is not None and hp0_aabb is not None:
        ctx.check(
            "handpieces hang below the delivery head",
            hp0_aabb[0][2] < dh_aabb[0][2],
            details=f"handpiece_min_z={hp0_aabb[0][2]:.3f}, delivery_head_min_z={dh_aabb[0][2]:.3f}",
        )
    ctx.check(
        "chair has a headrest pad",
        backrest.get_visual("headrest_pad") is not None,
        details="headrest missing",
    )

    # ---- spatial sanity at rest -----------------------------------------
    # The overhead light head is above the patient chair seat, and the head
    # hangs below the swing arm on its drop link.
    with ctx.pose({recline: 0.0, swing: 0.0, tilt: 0.0}):
        head_aabb = ctx.part_world_aabb(light_head)
        seat_aabb = ctx.part_world_aabb(backrest)
        arm_pos = ctx.part_world_position(light_arm)
        head_pos = ctx.part_world_position(light_head)
        ctx.check(
            "light head hangs above the chair",
            head_aabb is not None
            and seat_aabb is not None
            and head_aabb[0][2] > seat_aabb[1][2] + 0.5,
            details=f"head_z={head_aabb}, seat_z={seat_aabb}",
        )
        ctx.check(
            "light head hangs below the swing arm pivot",
            arm_pos is not None
            and head_pos is not None
            and head_pos[2] < arm_pos[2] - 0.15,
            details=f"arm={arm_pos}, head={head_pos}",
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
    with ctx.pose({swing: 0.0}):
        head_rest = ctx.part_world_position(light_head)
    with ctx.pose({swing: 1.2}):
        head_swung = ctx.part_world_position(light_head)
    ctx.check(
        "swinging the arm moves the light head sideways",
        head_rest is not None
        and head_swung is not None
        and abs(head_swung[1] - head_rest[1]) > 0.2,
        details=f"rest={head_rest}, swung={head_swung}",
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
