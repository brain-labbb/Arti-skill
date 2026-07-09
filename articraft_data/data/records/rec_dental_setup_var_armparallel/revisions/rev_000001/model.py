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
#   - chair_backrest       REVOLUTE : the patient back/seat reclines about a lateral pivot
#   - arm_yaw              REVOLUTE : the overhead arm swings horizontally on the post
#   - arm_bar_0_raise      REVOLUTE : primary parallelogram raise/lower (spring balanced)
#   - arm_bar_1_raise      REVOLUTE : mimic follower (keeps both bars parallel)
#   - carriage_level       REVOLUTE : mimic counter-rotation (keeps light head level)
#   - light_head_tilt      REVOLUTE : the round reflector head tilts on its yoke
#
# The parallelogram four-bar linkage:
#   Fixed link = arm_yaw bracket (two pivot points, bar_gap apart in Z)
#   Bar 0 (upper) and bar 1 (lower) are equal-length parallel bars
#   Light carriage (coupler) connects the far ends and counter-rotates to stay level
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
    Mimic,
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


def _parallelogram_bar_mesh(name):
    """Build one bar of the spring-balanced parallelogram linkage.

    Each bar is a cream-colored rectangular tube that extends along local +X
    with a slight inward curve toward -Y, reaching over the patient chair
    centerline. Pivot bushings at each end are added separately as visuals.
    Returns (mesh, far_endpoint_tuple).
    """
    pts = [
        (0.0, 0.0, 0.0),
        (0.18, -0.04, 0.0),
        (0.40, -0.15, 0.0),
        (0.58, -0.26, 0.0),
    ]
    mesh = _mesh(
        name,
        sweep_profile_along_spline(
            pts,
            profile=rounded_rect_profile(0.044, 0.028, radius=0.006),
            samples_per_segment=10,
            cap_profile=True,
        ),
    )
    return mesh, pts[-1]


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
    # overhead parallelogram arm pivots on. Modeled as part of the fixed base.
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
    # PARALLELOGRAM ARM: spring-balanced four-bar linkage
    #   The arm_yaw bracket swings horizontally on the vertical post.
    #   Two parallel bars (arm_bar_0 upper, arm_bar_1 lower) raise/lower
    #   the light carriage while keeping it level.
    # =====================================================================

    # -- arm_yaw: horizontal swing bracket mounted on the post top ---------
    arm_yaw = model.part("arm_yaw")
    # Parallelogram bar pivot offset: bars pivot at this X distance from the
    # arm_yaw origin (which sits on the post centerline). Must clear the post
    # radius (0.045) plus tolerance so the bar bushings don't intersect the post.
    bar_base_offset = 0.10
    bar_gap = 0.22  # vertical separation between the two bar pivot axes

    # pivot collar that hugs the post at the swing bearing
    arm_yaw.visual(
        Cylinder(radius=0.052, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=cream_dark,
        name="yaw_collar",
    )
    # upper bracket arm: extends from collar edge to bar_0 pivot
    _brk_cx = bar_base_offset / 2.0 + 0.025  # bracket center X
    _brk_sx = bar_base_offset - 0.05 + 0.01   # bracket X size
    arm_yaw.visual(
        Box((_brk_sx, 0.050, 0.030)),
        origin=Origin(xyz=(_brk_cx, 0.0, 0.015)),
        material=cream,
        name="upper_bracket",
    )
    # lower bracket arm: same geometry at the bar_1 pivot height
    arm_yaw.visual(
        Box((_brk_sx, 0.050, 0.030)),
        origin=Origin(xyz=(_brk_cx, 0.0, -bar_gap + 0.015)),
        material=cream,
        name="lower_bracket",
    )
    # vertical spine connecting the two bracket arms (rigid yoke)
    arm_yaw.visual(
        Box((0.025, 0.040, bar_gap - 0.030)),
        origin=Origin(xyz=(_brk_cx, 0.0, -bar_gap / 2.0 + 0.015)),
        material=cream_dark,
        name="bracket_spine",
    )
    # gas spring body mounted diagonally on the bracket (spring balance)
    arm_yaw.visual(
        Cylinder(radius=0.013, length=0.18),
        origin=Origin(xyz=(0.08, 0.0, -0.06), rpy=(0.0, 0.65, 0.0)),
        material=dark_metal,
        name="gas_spring_body",
    )
    # spring piston rod extending toward the upper bar
    arm_yaw.visual(
        Cylinder(radius=0.006, length=0.10),
        origin=Origin(xyz=(0.16, 0.0, 0.01), rpy=(0.0, 1.15, 0.0)),
        material=chrome,
        name="gas_spring_rod",
    )

    # -- parallelogram bars emitted via a for-i-in-range loop ---------------
    # Both bars are identical geometry, placed at different Z heights on the
    # bracket. Bar 0 is the primary raise joint; bar 1 mimics it.
    bars = []
    bar_far_pts = []
    for i in range(2):
        z_off = -i * bar_gap  # bar_0 at z=0, bar_1 at z=-bar_gap
        bar = model.part(f"arm_bar_{i}")

        bar_mesh, far_pt = _parallelogram_bar_mesh(f"bar_body_{i}")
        bar.visual(bar_mesh, material=cream, name=f"bar_body_{i}")
        # pivot bushing at the base (bracket) end
        bar.visual(
            Cylinder(radius=0.018, length=0.032),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"bar_base_bushing_{i}",
        )
        # pivot bushing at the carriage end
        bar.visual(
            Cylinder(radius=0.018, length=0.032),
            origin=Origin(xyz=far_pt),
            material=dark_metal,
            name=f"bar_far_bushing_{i}",
        )

        bars.append(bar)
        bar_far_pts.append(far_pt)

    # -- light carriage (coupler of the four-bar linkage) ------------------
    carriage = model.part("light_carriage")
    # bracket plate spanning between the two bar far endpoints
    carriage.visual(
        Box((0.050, 0.048, bar_gap)),
        origin=Origin(xyz=(0.0, 0.0, -bar_gap / 2.0)),
        material=cream_dark,
        name="carriage_plate",
    )
    # drop link extending below the carriage toward the light head
    drop_z = -bar_gap - 0.06
    carriage.visual(
        Cylinder(radius=0.018, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, drop_z)),
        material=metal,
        name="carriage_drop_link",
    )
    # small cross-pin at the top of the carriage where bar_0 connects
    carriage.visual(
        Cylinder(radius=0.012, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="carriage_upper_pin",
    )
    # cross-pin at the bottom where bar_1 connects
    carriage.visual(
        Cylinder(radius=0.012, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, -bar_gap), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="carriage_lower_pin",
    )

    # =====================================================================
    # LIGHT HEAD  (round reflector that tilts on the yoke)
    #   Child of light_carriage via a lateral-axis revolute at the drop link.
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
    # (lowers the head end / raises the foot end).
    model.articulation(
        "backrest_recline",
        ArticulationType.REVOLUTE,
        parent=base,
        child=backrest,
        origin=Origin(xyz=(0.06, 0.0, 0.61)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=400.0, velocity=0.4, lower=-0.35, upper=0.55),
    )

    # Arm yaw: vertical-axis revolute at the top of the post.
    # Positive q swings the arm toward +Y/-Y.
    model.articulation(
        "arm_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm_yaw,
        origin=Origin(xyz=(post_x, post_y, 1.79)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=-1.9, upper=1.9),
    )

    # Parallelogram bar_0 raise (primary joint): lateral-axis revolute at the
    # upper bracket. axis=(0, -1, 0) so positive q raises the arm: right-hand
    # rule about -Y rotates +X toward +Z.
    model.articulation(
        "arm_bar_0_raise",
        ArticulationType.REVOLUTE,
        parent=arm_yaw,
        child=bars[0],
        origin=Origin(xyz=(bar_base_offset, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.8, lower=-0.30, upper=0.70),
    )

    # Parallelogram bar_1 raise (mimic follower): same angle as bar_0 so both
    # bars remain parallel throughout the raise/lower motion.
    model.articulation(
        "arm_bar_1_raise",
        ArticulationType.REVOLUTE,
        parent=arm_yaw,
        child=bars[1],
        origin=Origin(xyz=(bar_base_offset, 0.0, -bar_gap)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.8, lower=-0.30, upper=0.70),
        mimic=Mimic(joint="arm_bar_0_raise", multiplier=1.0, offset=0.0),
    )

    # Carriage level joint: counter-rotates to keep the light carriage (and
    # therefore the light head) horizontal regardless of arm elevation.
    # When bar_0 rotates +θ about -Y, the carriage rotates -θ about -Y
    # (i.e., +θ about +Y) relative to bar_0, yielding zero net rotation.
    tilt_origin_z = drop_z  # at the drop link where the light head captures
    model.articulation(
        "carriage_level",
        ArticulationType.REVOLUTE,
        parent=bars[0],
        child=carriage,
        origin=Origin(xyz=bar_far_pts[0]),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.0, lower=-0.30, upper=0.70),
        mimic=Mimic(joint="arm_bar_0_raise", multiplier=-1.0, offset=0.0),
    )

    # Light head tilt: lateral-axis revolute at the carriage drop link.
    # The head hangs below the carriage; tilt aims the beam.
    model.articulation(
        "light_head_tilt",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=light_head,
        origin=Origin(xyz=(0.0, 0.0, tilt_origin_z)),
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
    arm_yaw = object_model.get_part("arm_yaw")
    bar_0 = object_model.get_part("arm_bar_0")
    bar_1 = object_model.get_part("arm_bar_1")
    carriage = object_model.get_part("light_carriage")
    light_head = object_model.get_part("light_head")
    column = object_model.get_part("delivery_column")
    stool = object_model.get_part("stool")

    recline = object_model.get_articulation("backrest_recline")
    yaw = object_model.get_articulation("arm_yaw")
    raise_joint = object_model.get_articulation("arm_bar_0_raise")
    raise_follower = object_model.get_articulation("arm_bar_1_raise")
    level_joint = object_model.get_articulation("carriage_level")
    tilt = object_model.get_articulation("light_head_tilt")

    # ---- intentional capture fits ---------------------------------------
    # The yaw collar wraps the fixed light post at the swing bearing.
    ctx.allow_overlap(
        arm_yaw,
        base,
        elem_a="yaw_collar",
        elem_b="light_post",
        reason="The yaw collar is captured around the light post at the swing bearing.",
    )
    # The bracket arms also contact the light post at the swing bearing region.
    ctx.allow_overlap(
        arm_yaw,
        base,
        elem_a="lower_bracket",
        elem_b="light_post",
        reason="The lower bracket arm contacts the light post at the swing bearing region.",
    )
    # Each bar's base bushing is a pivot bearing captured in its bracket.
    for i in range(2):
        brk_name = "upper_bracket" if i == 0 else "lower_bracket"
        ctx.allow_overlap(
            f"arm_bar_{i}",
            arm_yaw,
            elem_a=f"bar_base_bushing_{i}",
            elem_b=brk_name,
            reason=f"Bar {i} base bushing is a pivot bearing captured in the {brk_name}.",
        )
        # Bar base bushings also contact the light post at the pivot region.
        ctx.allow_overlap(
            f"arm_bar_{i}",
            base,
            elem_a=f"bar_base_bushing_{i}",
            elem_b="light_post",
            reason=f"Bar {i} base bushing contacts the light post at the pivot bearing region.",
        )
    # Each bar's far bushing is captured in the carriage cross-pin.
    for i in range(2):
        pin_name = "carriage_upper_pin" if i == 0 else "carriage_lower_pin"
        ctx.allow_overlap(
            f"arm_bar_{i}",
            carriage,
            elem_a=f"bar_far_bushing_{i}",
            elem_b=pin_name,
            reason=f"Bar {i} far bushing is a pivot bearing captured in the {pin_name}.",
        )
    # The lower bar far bushing converges with the light head yoke stub
    # at the lower carriage pivot area (both attach to the same pivot region).
    ctx.allow_overlap(
        bar_1,
        light_head,
        elem_a="bar_far_bushing_1",
        elem_b="head_yoke_stub",
        reason="Bar 1 far bushing and head yoke stub share the lower carriage pivot region.",
    )
    # The light-head yoke stub is captured inside the carriage drop link.
    ctx.allow_overlap(
        light_head,
        carriage,
        elem_a="head_yoke_stub",
        elem_b="carriage_drop_link",
        reason="The head yoke stub nests inside the carriage drop link at the tilt joint.",
    )
    # The carriage plate also captures the head yoke stub at the tilt pivot.
    ctx.allow_overlap(
        light_head,
        carriage,
        elem_a="head_yoke_stub",
        elem_b="carriage_plate",
        reason="The head yoke stub passes through the carriage plate at the tilt pivot.",
    )
    # Bar body meshes contact pivot elements at their endpoints.
    for i in range(2):
        brk_name = "upper_bracket" if i == 0 else "lower_bracket"
        pin_name = "carriage_upper_pin" if i == 0 else "carriage_lower_pin"
        # Bar body contacts the bracket at the base pivot.
        ctx.allow_overlap(
            f"arm_bar_{i}",
            arm_yaw,
            elem_a=f"bar_body_{i}",
            elem_b=brk_name,
            reason=f"Bar {i} body mesh contacts the {brk_name} at the base pivot.",
        )
        # Bar body contacts the carriage pin at the far pivot.
        ctx.allow_overlap(
            f"arm_bar_{i}",
            carriage,
            elem_a=f"bar_body_{i}",
            elem_b=pin_name,
            reason=f"Bar {i} body mesh contacts the {pin_name} at the far pivot.",
        )
    # Bar 0 body also contacts the gas spring body mounted on the bracket.
    ctx.allow_overlap(
        bar_0,
        arm_yaw,
        elem_a="bar_body_0",
        elem_b="gas_spring_body",
        reason="Bar 0 body mesh contacts the gas spring body at the bracket pivot region.",
    )
    # Bar 1 body contacts the head yoke stub at the lower carriage pivot.
    ctx.allow_overlap(
        bar_1,
        light_head,
        elem_a="bar_body_1",
        elem_b="head_yoke_stub",
        reason="Bar 1 body mesh contacts the head yoke stub at the lower carriage pivot.",
    )
    # Gas spring body contacts the light post at the bracket pivot region.
    ctx.allow_overlap(
        arm_yaw,
        base,
        elem_a="gas_spring_body",
        elem_b="light_post",
        reason="The gas spring body contacts the light post at the bracket pivot region.",
    )
    # Bar 0 base bushing also contacts the gas spring body at the pivot region.
    ctx.allow_overlap(
        bar_0,
        arm_yaw,
        elem_a="bar_base_bushing_0",
        elem_b="gas_spring_body",
        reason="Bar 0 base bushing contacts the gas spring body at the pivot region.",
    )
    # Bar far bushings contact the carriage plate at the far pivot region.
    for i in range(2):
        ctx.allow_overlap(
            f"arm_bar_{i}",
            carriage,
            elem_a=f"bar_far_bushing_{i}",
            elem_b="carriage_plate",
            reason=f"Bar {i} far bushing contacts the carriage plate at the far pivot region.",
        )
    # Bar 1 far bushing also contacts the reflector shell at the tilt pivot.
    ctx.allow_overlap(
        bar_1,
        light_head,
        elem_a="bar_far_bushing_1",
        elem_b="reflector_shell",
        reason="Bar 1 far bushing contacts the reflector shell at the tilt pivot region.",
    )
    # The carriage drop link contacts the reflector shell where the head mounts.
    ctx.allow_overlap(
        light_head,
        carriage,
        elem_a="reflector_shell",
        elem_b="carriage_drop_link",
        reason="The reflector shell contacts the carriage drop link at the tilt mounting region.",
    )
    # Bar 0 body contacts the gas spring rod at the bracket pivot region.
    ctx.allow_overlap(
        bar_0,
        arm_yaw,
        elem_a="bar_body_0",
        elem_b="gas_spring_rod",
        reason="Bar 0 body mesh contacts the gas spring rod at the bracket pivot region.",
    )
    # Bar 0 body contacts the carriage plate at the far pivot region.
    ctx.allow_overlap(
        bar_0,
        carriage,
        elem_a="bar_body_0",
        elem_b="carriage_plate",
        reason="Bar 0 body mesh contacts the carriage plate at the far pivot region.",
    )
    # Bar 1 far bushing contacts the carriage drop link at the far pivot.
    ctx.allow_overlap(
        bar_1,
        carriage,
        elem_a="bar_far_bushing_1",
        elem_b="carriage_drop_link",
        reason="Bar 1 far bushing contacts the carriage drop link at the far pivot region.",
    )
    # Bar 1 body contacts the reflector shell at the tilt pivot region.
    ctx.allow_overlap(
        bar_1,
        light_head,
        elem_a="bar_body_1",
        elem_b="reflector_shell",
        reason="Bar 1 body mesh contacts the reflector shell at the tilt pivot region.",
    )
    # The carriage lower pin contacts the head yoke stub at the tilt pivot.
    ctx.allow_overlap(
        light_head,
        carriage,
        elem_a="head_yoke_stub",
        elem_b="carriage_lower_pin",
        reason="The head yoke stub contacts the carriage lower pin at the tilt pivot region.",
    )
    # Bar 0 base bushing contacts the gas spring rod at the pivot region.
    ctx.allow_overlap(
        bar_0,
        arm_yaw,
        elem_a="bar_base_bushing_0",
        elem_b="gas_spring_rod",
        reason="Bar 0 base bushing contacts the gas spring rod at the pivot region.",
    )
    # Bar 1 body contacts the carriage plate at the far pivot region.
    ctx.allow_overlap(
        bar_1,
        carriage,
        elem_a="bar_body_1",
        elem_b="carriage_plate",
        reason="Bar 1 body mesh contacts the carriage plate at the far pivot region.",
    )
    # The carriage lower pin contacts the reflector shell at the tilt pivot.
    ctx.allow_overlap(
        light_head,
        carriage,
        elem_a="reflector_shell",
        elem_b="carriage_lower_pin",
        reason="The reflector shell contacts the carriage lower pin at the tilt pivot region.",
    )
    # Bar 0 base bushing contacts the bracket spine at the pivot region.
    ctx.allow_overlap(
        bar_0,
        arm_yaw,
        elem_a="bar_base_bushing_0",
        elem_b="bracket_spine",
        reason="Bar 0 base bushing contacts the bracket spine at the pivot region.",
    )
    # Bar 1 body contacts the carriage drop link at the far pivot region.
    ctx.allow_overlap(
        bar_1,
        carriage,
        elem_a="bar_body_1",
        elem_b="carriage_drop_link",
        reason="Bar 1 body mesh contacts the carriage drop link at the far pivot region.",
    )
    # The carriage drop link contacts the light lens at the tilt mounting region.
    ctx.allow_overlap(
        light_head,
        carriage,
        elem_a="light_lens",
        elem_b="carriage_drop_link",
        reason="The light lens contacts the carriage drop link at the tilt mounting region.",
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
        "arm yaw swings on a vertical axis",
        str(yaw.articulation_type).endswith("REVOLUTE") and abs(yaw.axis[2]) > 0.9,
        details=f"type={yaw.articulation_type}, axis={yaw.axis}",
    )
    ctx.check(
        "bar_0 raise is a lateral-axis revolute",
        str(raise_joint.articulation_type).endswith("REVOLUTE") and abs(raise_joint.axis[1]) > 0.9,
        details=f"type={raise_joint.articulation_type}, axis={raise_joint.axis}",
    )
    ctx.check(
        "bar_1 is a mimic follower of bar_0",
        raise_follower.mimic is not None
        and raise_follower.mimic.joint == "arm_bar_0_raise"
        and abs(raise_follower.mimic.multiplier - 1.0) < 1e-6,
        details=f"mimic={raise_follower.mimic}",
    )
    ctx.check(
        "carriage counter-rotates (mimic multiplier=-1)",
        level_joint.mimic is not None
        and level_joint.mimic.joint == "arm_bar_0_raise"
        and abs(level_joint.mimic.multiplier + 1.0) < 1e-6,
        details=f"mimic={level_joint.mimic}",
    )
    ctx.check(
        "light head tilts on a lateral axis",
        str(tilt.articulation_type).endswith("REVOLUTE") and abs(tilt.axis[1]) > 0.9,
        details=f"type={tilt.articulation_type}, axis={tilt.axis}",
    )

    # ---- parallelogram bars exist and have correct geometry --------------
    ctx.check(
        "parallelogram has two bars with body and bushings",
        bar_0.get_visual("bar_body_0") is not None
        and bar_0.get_visual("bar_base_bushing_0") is not None
        and bar_0.get_visual("bar_far_bushing_0") is not None
        and bar_1.get_visual("bar_body_1") is not None
        and bar_1.get_visual("bar_base_bushing_1") is not None
        and bar_1.get_visual("bar_far_bushing_1") is not None,
        details="parallelogram bar geometry missing",
    )
    ctx.check(
        "carriage has a plate connecting both bar endpoints",
        carriage.get_visual("carriage_plate") is not None
        and carriage.get_visual("carriage_drop_link") is not None,
        details="carriage geometry missing",
    )
    ctx.check(
        "arm_yaw bracket has gas spring visual",
        arm_yaw.get_visual("gas_spring_body") is not None,
        details="spring balance visual missing",
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
    with ctx.pose({recline: 0.0, yaw: 0.0, raise_joint: 0.0, tilt: 0.0}):
        head_aabb = ctx.part_world_aabb(light_head)
        seat_aabb = ctx.part_world_aabb(backrest)
        arm_pos = ctx.part_world_position(arm_yaw)
        head_pos = ctx.part_world_position(light_head)
        ctx.check(
            "light head hangs above the chair",
            head_aabb is not None
            and seat_aabb is not None
            and head_aabb[0][2] > seat_aabb[1][2] + 0.5,
            details=f"head_z={head_aabb}, seat_z={seat_aabb}",
        )
        ctx.check(
            "light head hangs below the arm pivot",
            arm_pos is not None
            and head_pos is not None
            and head_pos[2] < arm_pos[2] - 0.15,
            details=f"arm={arm_pos}, head={head_pos}",
        )
        ctx.check(
            "light head is positioned over the chair seat",
            head_aabb is not None
            and seat_aabb is not None
            and head_aabb[1][0] > seat_aabb[0][0]
            and head_aabb[0][0] < seat_aabb[1][0],
            details=f"head_x={head_aabb}, seat_x={seat_aabb}",
        )

    # ---- the yaw swing moves the head laterally -------------------------
    with ctx.pose({yaw: 0.0, raise_joint: 0.0}):
        head_rest = ctx.part_world_position(light_head)
    with ctx.pose({yaw: 1.2, raise_joint: 0.0}):
        head_swung = ctx.part_world_position(light_head)
    ctx.check(
        "swinging the arm moves the light head sideways",
        head_rest is not None
        and head_swung is not None
        and abs(head_swung[1] - head_rest[1]) > 0.2,
        details=f"rest={head_rest}, swung={head_swung}",
    )

    # ---- the raise/lower moves the head vertically ----------------------
    with ctx.pose({raise_joint: 0.0}):
        head_low = ctx.part_world_position(light_head)
    with ctx.pose({raise_joint: 0.55}):
        head_high = ctx.part_world_position(light_head)
    ctx.check(
        "raising the parallelogram arm lifts the light head",
        head_low is not None
        and head_high is not None
        and head_high[2] > head_low[2] + 0.05,
        details=f"low={head_low}, high={head_high}",
    )

    # ---- parallelogram keeps the light head level across elevation ------
    # At both low and high poses, the light head Z extent relative to its
    # own AABB should stay roughly the same shape (no tilt induced by the
    # parallelogram). We check that the lens bottom stays at a consistent
    # offset from the head center.
    with ctx.pose({raise_joint: 0.0, tilt: 0.0}):
        lens_low = ctx.part_element_world_aabb(light_head, elem="light_lens")
        head_center_low = ctx.part_world_position(light_head)
    with ctx.pose({raise_joint: 0.55, tilt: 0.0}):
        lens_high = ctx.part_element_world_aabb(light_head, elem="light_lens")
        head_center_high = ctx.part_world_position(light_head)
    ctx.check(
        "parallelogram keeps head level: lens stays below head center",
        lens_low is not None
        and lens_high is not None
        and head_center_low is not None
        and head_center_high is not None
        and lens_low[0][2] < head_center_low[2]
        and lens_high[0][2] < head_center_high[2],
        details=f"low_lens={lens_low}, high_lens={lens_high}, "
                f"low_center={head_center_low}, high_center={head_center_high}",
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
