from __future__ import annotations

# Outdoor wooden canopy swing chair (dark-stained hardwood) — single-person variant.
#
# Frame convention:
#   X = fore/aft depth (chair swings in X; sitter faces +X)
#   Y = left/right width (top beam and canopy ridge run along Y)
#   Z = up
#
# Root part = fixed stand: two A-shaped side supports (square-section legs)
# with a side cross rail each, joined by a top beam; a small flat tray shelf
# on the outside of each A-frame at armrest height; above the beam a lightly
# pitched green canopy (ridge bar, eave rails, rafters, two slope panels)
# with a thin scalloped fabric skirt strip hanging around all four edges.
# Four wooden swing arms hang from collinear revolute pivots on the top beam
# (one driver + three mimic followers, multiplier 1.0), and the single-person
# contoured slatted seat with a curved backrest hangs from the arms and
# swings as a pendulum between the A-frames.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)

# --- Stand dimensions (meters) -------------------------------------------------
LEG_W = 0.065          # square leg section
FOOT_X = 0.55          # leg foot center x at ground
LEG_TOP_X = 0.05       # leg axis x at the top reference height
LEG_TOP_Z = 1.66       # top reference height for the leg axis line
LEG_LEN = 1.74         # leg length along its axis
LEG_Y = 0.78           # A-frame center planes at y = +/-LEG_Y

BEAM_Z = 1.62          # top beam center height
BEAM_W = 0.07          # beam section
BEAM_LEN = 1.78

RAIL_Z = 0.57          # side cross rail height (tray support)
TRAY_Y = 0.90          # tray shelf center plane (outside the A-frame)

# --- Canopy ---------------------------------------------------------------------
RIDGE_Z = 1.815        # ridge bar center height
EAVE_X = 0.60          # eave rail x position
EAVE_Z = 1.74          # eave rail center height
CANOPY_PITCH = math.atan2(0.075, EAVE_X)  # gentle pitch (~7 deg)
PANEL_W = 1.76         # canopy width along Y
SKIRT_H = 0.13         # scalloped skirt height
SKIRT_TOP_Z = 1.775
SKIRT_T = 0.012

# --- Swing ----------------------------------------------------------------------
PIVOT_Z = 1.60         # swing pivot height (bolts through the top beam)
ARM_Y = 0.24           # swing arm center planes at y = +/-ARM_Y (single chair)
ARM_ATTACH_Z = -0.97   # arm/chair attach height relative to the pivot
ARM_FRONT_X = 0.20     # front arm lower-end x (chair frame)
ARM_REAR_X = -0.22     # rear arm lower-end x (chair frame)
SWING_LIMIT = 0.4      # rad

# --- Single-person swing chair (origin under the pivot line) --------------------
SEAT_TOP = -1.10       # seat slat top (world ~0.50)
SEAT_W = 0.44          # seat slat span along Y (single person)
SEAT_DEPTH = 0.36      # seat rail span in X (front to rear)
BACK_RECLINE = math.radians(12.0)

# Curved backrest
BACK_WIDTH = 0.50      # backrest width along Y
BACK_HEIGHT = 0.44     # backrest height (along the recline direction)
BACK_RADIUS = 0.65     # horizontal curvature radius
BACK_THICKNESS = 0.022 # panel thickness

DRIVE_JOINT = "swing_pivot_0"


def _scalloped_skirt(length: float, name: str, peak: float = 0.0):
    """Thin vertical fabric strip with a scalloped (wavy) lower edge.

    Built in the local XY plane: x = along the strip, y = up (bottom of the
    scallop lobes at y=0, straight top edge at y=SKIRT_H, optional triangular
    peak above for the pitched gable ends), extruded SKIRT_T thick and
    centered on local z=0.
    """
    n = max(2, int(round(length / 0.11)))
    r = length / (2.0 * n)
    solid = (
        cq.Workplane("XY")
        .center(0.0, (SKIRT_H + r) / 2.0)
        .rect(length, SKIRT_H - r)
        .extrude(SKIRT_T)
    )
    for k in range(n):
        cx = -length / 2.0 + (2 * k + 1) * r
        solid = solid.union(cq.Workplane("XY").center(cx, r).circle(r).extrude(SKIRT_T))
    if peak > 0.0:
        solid = solid.union(
            cq.Workplane("XY")
            .polyline(
                [
                    (-length / 2.0, SKIRT_H),
                    (0.0, SKIRT_H + peak),
                    (length / 2.0, SKIRT_H),
                ]
            )
            .close()
            .extrude(SKIRT_T)
        )
    solid = solid.translate((0.0, 0.0, -SKIRT_T / 2.0))
    return mesh_from_cadquery(solid, name)


def _curved_back_panel(width: float, height: float, radius: float,
                       thickness: float, name: str):
    """CadQuery annular-sector curved panel for the backrest.

    Built in local coords: x = depth offset (0 at the outer surface center,
    increasing toward the sitter), y = across, z = up from 0 to height.
    The sitter-facing inner surface is concave toward +X.
    """
    half_w = width / 2.0
    R = radius
    t = thickness
    alpha = math.asin(min(0.999, half_w / R))

    # Outer arc endpoints (radius R from center at (R, 0))
    ob = (R * (1.0 - math.cos(alpha)), -half_w)
    om = (0.0, 0.0)
    ot = (R * (1.0 - math.cos(alpha)), half_w)

    # Inner arc endpoints (radius R-t from the same center)
    Ri = R - t
    ib = (R - Ri * math.cos(alpha), -Ri * math.sin(alpha))
    im = (t, 0.0)
    it = (R - Ri * math.cos(alpha), Ri * math.sin(alpha))

    panel = (
        cq.Workplane("XY")
        .moveTo(ob[0], ob[1])
        .threePointArc(om, ot)
        .lineTo(it[0], it[1])
        .threePointArc(im, ib)
        .close()
        .extrude(height)
    )
    return mesh_from_cadquery(panel, name)


def _arc_path_points(width: float, radius: float, z_height: float,
                     n: int = 10):
    """3D points along a horizontal circular arc at given z height.

    Returns points in the same local frame as _curved_back_panel.
    """
    half_w = width / 2.0
    pts = []
    for i in range(n + 1):
        y = -half_w + width * i / n
        theta = math.asin(max(-0.999, min(0.999, y / radius)))
        x = radius * (1.0 - math.cos(theta))
        pts.append((x, y, z_height))
    return pts


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wooden_canopy_swing_chair")
    model.material("wood_dark", rgba=(0.28, 0.165, 0.09, 1.0))
    model.material("wood", rgba=(0.37, 0.225, 0.12, 1.0))
    model.material("canvas_green", rgba=(0.10, 0.36, 0.18, 1.0))
    model.material("canvas_green_dark", rgba=(0.075, 0.27, 0.135, 1.0))
    model.material("steel", rgba=(0.45, 0.46, 0.48, 1.0))

    frame = model.part("a_frame_stand")

    # ------------------------------------------------------ A-frame legs
    tilt = math.atan2(FOOT_X - LEG_TOP_X, LEG_TOP_Z)
    slope = (FOOT_X - LEG_TOP_X) / LEG_TOP_Z  # dx per dz toward the center
    # Lift so the lowest tilted-box corner rests exactly on the ground.
    leg_cz = (LEG_LEN / 2.0) * math.cos(tilt) + (LEG_W / 2.0) * math.sin(tilt)
    leg_cx = FOOT_X - slope * leg_cz
    for s, ly in enumerate((-LEG_Y, LEG_Y)):
        for tag, sgn in (("front", 1.0), ("rear", -1.0)):
            frame.visual(
                Box((LEG_W, LEG_W, LEG_LEN)),
                origin=Origin(xyz=(sgn * leg_cx, ly, leg_cz), rpy=(0.0, -sgn * tilt, 0.0)),
                material="wood_dark",
                name=f"leg_{tag}_{s}",
            )
        # Side cross rail between the two legs of this A (tray support).
        frame.visual(
            Box((0.83, 0.05, 0.06)),
            origin=Origin(xyz=(0.0, ly, RAIL_Z)),
            material="wood_dark",
            name=f"side_rail_{s}",
        )

    # Small flat tray shelves on the OUTSIDE of each A-frame at armrest height.
    for s, sgn in enumerate((-1.0, 1.0)):
        for b, bx in enumerate((-0.20, 0.20)):
            frame.visual(
                Box((0.05, 0.30, 0.03)),
                origin=Origin(xyz=(bx, sgn * (TRAY_Y - 0.005), 0.585)),
                material="wood_dark",
                name=f"tray_bearer_{s}_{b}",
            )
        frame.visual(
            Box((0.58, 0.30, 0.025)),
            origin=Origin(xyz=(0.0, sgn * TRAY_Y, 0.6125)),
            material="wood",
            name=f"tray_shelf_{s}",
        )

    # ------------------------------------------------------ top beam
    frame.visual(
        Box((BEAM_W, BEAM_LEN, BEAM_W)),
        origin=Origin(xyz=(0.0, 0.0, BEAM_Z)),
        material="wood_dark",
        name="top_beam",
    )

    # ------------------------------------------------------ canopy frame
    # Struts sit outboard of the swing-arm planes (arms at |y| <= ~0.35).
    for s, sy in enumerate((-0.70, 0.70)):
        frame.visual(
            Box((0.05, 0.05, 0.16)),
            origin=Origin(xyz=(0.0, sy, 1.735)),
            material="wood_dark",
            name=f"ridge_strut_{s}",
        )
    frame.visual(
        Box((0.05, 1.72, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, RIDGE_Z)),
        material="wood_dark",
        name="ridge_bar",
    )
    for tag, sgn in (("front", 1.0), ("rear", -1.0)):
        frame.visual(
            Box((0.04, 1.72, 0.04)),
            origin=Origin(xyz=(sgn * EAVE_X, 0.0, EAVE_Z)),
            material="wood_dark",
            name=f"eave_rail_{tag}",
        )
        for s, ry in enumerate((-0.86, 0.86)):
            frame.visual(
                Box((0.645, 0.04, 0.045)),
                origin=Origin(
                    xyz=(sgn * 0.30, ry, 1.7775),
                    rpy=(0.0, sgn * CANOPY_PITCH, 0.0),
                ),
                material="wood_dark",
                name=f"rafter_{tag}_{s}",
            )

    # ------------------------------------------------------ canopy panels + skirt
    for tag, sgn in (("front", 1.0), ("rear", -1.0)):
        frame.visual(
            Box((0.68, PANEL_W, 0.012)),
            origin=Origin(
                xyz=(sgn * 0.325, 0.0, 1.797),
                rpy=(0.0, sgn * CANOPY_PITCH, 0.0),
            ),
            material="canvas_green",
            name=f"canopy_panel_{tag}",
        )
    frame.visual(
        Box((0.10, PANEL_W, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 1.852)),
        material="canvas_green_dark",
        name="canopy_ridge_cap",
    )
    # Scalloped skirt strips around the canopy edge. Eave strips: local x ->
    # world Y, local y -> world Z, thickness -> world X (rpy = (pi/2, 0, pi/2)).
    for tag, sgn in (("front", 1.0), ("rear", -1.0)):
        frame.visual(
            _scalloped_skirt(PANEL_W, f"skirt_{tag}_mesh"),
            origin=Origin(
                xyz=(sgn * 0.66, 0.0, SKIRT_TOP_Z - SKIRT_H),
                rpy=(math.pi / 2.0, 0.0, math.pi / 2.0),
            ),
            material="canvas_green",
            name=f"canopy_skirt_{tag}",
        )
    # Gable-end strips with a peaked top following the pitch: local x ->
    # world X, local y -> world Z, thickness -> world Y (rpy = (pi/2, 0, 0)).
    for s, sgn in enumerate((-1.0, 1.0)):
        frame.visual(
            _scalloped_skirt(1.34, f"skirt_side_{s}_mesh", peak=0.065),
            origin=Origin(
                xyz=(0.0, sgn * 0.883, SKIRT_TOP_Z - SKIRT_H),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="canvas_green",
            name=f"canopy_skirt_side_{s}",
        )

    # ------------------------------------------------------ four wooden swing arms
    # Each arm hangs from its own revolute pivot on the top beam; the four
    # pivots are collinear on the beam axis. Front/rear arms at each side
    # share a pivot bolt and fan out (slight V) to the chair armrest.
    def _arm_part(name: str, lower_x: float, with_bolt: bool):
        part = model.part(name)
        theta = math.atan2(abs(lower_x), -ARM_ATTACH_Z)
        sgn = 1.0 if lower_x > 0.0 else -1.0
        reach = math.hypot(lower_x, ARM_ATTACH_Z)
        t0, t1 = -0.06, reach + 0.045  # extend above the pivot and past the attach
        length = t1 - t0
        tc = (t0 + t1) / 2.0
        dx, dz = sgn * math.sin(theta), -math.cos(theta)
        part.visual(
            Box((0.055, 0.035, length)),
            origin=Origin(
                xyz=(dx * tc, 0.0, dz * tc),
                rpy=(0.0, -sgn * theta, 0.0),
            ),
            material="wood",
            name="bar",
        )
        if with_bolt:
            part.visual(
                Cylinder(radius=0.011, length=0.09),
                origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material="steel",
                name="pivot_bolt",
            )
        return part

    # Arm definitions: (part_name, lower_x, y_pos, with_bolt, mimic)
    arm_configs = [
        ("swing_arm_0", ARM_FRONT_X, -ARM_Y, True, None),
        ("swing_arm_1", ARM_FRONT_X, ARM_Y, True,
         Mimic(joint=DRIVE_JOINT, multiplier=1.0)),
        ("swing_arm_2", ARM_REAR_X, -ARM_Y, False,
         Mimic(joint=DRIVE_JOINT, multiplier=1.0)),
        ("swing_arm_3", ARM_REAR_X, ARM_Y, False,
         Mimic(joint=DRIVE_JOINT, multiplier=1.0)),
    ]
    arms = []
    for i, (aname, lower_x, jy, bolt, mimic) in enumerate(arm_configs):
        arm = _arm_part(aname, lower_x, with_bolt=bolt)
        arms.append(arm)
        model.articulation(
            f"swing_pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=arm,
            origin=Origin(xyz=(0.0, jy, PIVOT_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=120.0, velocity=2.5,
                lower=-SWING_LIMIT, upper=SWING_LIMIT,
            ),
            mimic=mimic,
        )

    # ------------------------------------------------------ single-person swing chair
    chair = model.part("swing_chair")

    # Seat frame: front/rear rails + end rails (narrow for one person).
    # Frame width reaches the arm post positions so the whole frame is connected.
    half_depth = SEAT_DEPTH / 2.0
    FRAME_W = 2.0 * ARM_Y + 0.04   # front/rear rail span in Y
    for tag, rx in (("front", half_depth), ("rear", -half_depth)):
        chair.visual(
            Box((0.05, FRAME_W, 0.05)),
            origin=Origin(xyz=(rx, 0.0, -1.15)),
            material="wood_dark",
            name=f"seat_{tag}_rail",
        )
    for i in range(2):
        ey = (-1.0, 1.0)[i] * ARM_Y
        chair.visual(
            Box((SEAT_DEPTH, 0.05, 0.05)),
            origin=Origin(xyz=(0.0, ey, -1.15)),
            material="wood_dark",
            name=f"seat_end_rail_{i}",
        )

    # Four contoured seat slats with a gentle dish (center lower than edges).
    # Slat span matches the frame width so slats contact the end rails.
    n_seat_slats = 4
    slat_span = 2.0 * ARM_Y
    slat_spacing = SEAT_DEPTH * 0.85 / (n_seat_slats - 1)
    for i in range(n_seat_slats):
        sx = -SEAT_DEPTH * 0.425 + i * slat_spacing
        # Parabolic dish: max 5mm depression at center
        norm = sx / (SEAT_DEPTH * 0.425) if SEAT_DEPTH > 0 else 0.0
        dish = -0.005 * (1.0 - norm * norm)
        chair.visual(
            Box((0.075, slat_span, 0.025)),
            origin=Origin(xyz=(sx, 0.0, SEAT_TOP - 0.0125 + dish)),
            material="wood",
            name=f"seat_slat_{i}",
        )

    # ------------------------------------------------------ curved backrest
    # The backrest base sits at the rear seat rail, reclined 12° from vertical.
    # bz0 is lowered so the back bottom rail contacts the seat rear rail.
    bx0 = -half_depth - 0.025   # back plane base x (just behind the rear rail)
    bz0 = -1.12                 # back plane base z (contacting seat rail top)

    # Curved panel (CadQuery annular sector, extruded)
    chair.visual(
        _curved_back_panel(BACK_WIDTH, BACK_HEIGHT, BACK_RADIUS,
                           BACK_THICKNESS, "back_panel_mesh"),
        origin=Origin(xyz=(bx0, 0.0, bz0), rpy=(0.0, -BACK_RECLINE, 0.0)),
        material="wood",
        name="back_panel",
    )

    # Curved bottom rail (swept along the arc at z=0 in back-local frame)
    bottom_pts = _arc_path_points(BACK_WIDTH, BACK_RADIUS, 0.0, n=10)
    bottom_rail_geom = sweep_profile_along_spline(
        bottom_pts,
        profile=rounded_rect_profile(0.048, 0.042, 0.006),
        up_hint=(0.0, 0.0, 1.0),
    )
    chair.visual(
        mesh_from_geometry(bottom_rail_geom, "back_bottom_rail_mesh"),
        origin=Origin(xyz=(bx0, 0.0, bz0), rpy=(0.0, -BACK_RECLINE, 0.0)),
        material="wood_dark",
        name="back_bottom_rail",
    )

    # Curved top rail (swept along the arc at z=BACK_HEIGHT)
    top_pts = _arc_path_points(BACK_WIDTH, BACK_RADIUS, BACK_HEIGHT, n=10)
    top_rail_geom = sweep_profile_along_spline(
        top_pts,
        profile=rounded_rect_profile(0.052, 0.045, 0.006),
        up_hint=(0.0, 0.0, 1.0),
    )
    chair.visual(
        mesh_from_geometry(top_rail_geom, "back_top_rail_mesh"),
        origin=Origin(xyz=(bx0, 0.0, bz0), rpy=(0.0, -BACK_RECLINE, 0.0)),
        material="wood_dark",
        name="back_top_rail",
    )

    # Two vertical support battens on the outside of the curved panel
    for i in range(2):
        by = (-0.13, 0.13)[i]
        theta_b = math.asin(by / BACK_RADIUS)
        bx_outer = BACK_RADIUS * (1.0 - math.cos(theta_b))
        # Batten centered in back-local frame at (bx_outer - 0.012, by, height/2)
        chair.visual(
            Box((0.024, 0.04, BACK_HEIGHT - 0.06)),
            origin=Origin(
                xyz=(bx0 + bx_outer * math.cos(BACK_RECLINE) - 0.014,
                     by,
                     bz0 + BACK_HEIGHT / 2.0 * math.cos(BACK_RECLINE)),
                rpy=(0.0, -BACK_RECLINE, theta_b),
            ),
            material="wood_dark",
            name=f"back_batten_{i}",
        )

    # Two rear uprights connecting the seat frame to the backrest.
    # These bridge from the seat rear rail up into the backrest region,
    # contacting both the seat end rails and the curved back bottom rail.
    for i in range(2):
        uy = (-1.0, 1.0)[i] * (BACK_WIDTH / 2.0 - 0.03)
        # Upright from seat rail bottom to above the back bottom rail
        up_bot = -1.175
        up_top = bz0 + BACK_HEIGHT * 0.30 * math.cos(BACK_RECLINE)
        up_h = up_top - up_bot
        up_cz = (up_bot + up_top) / 2.0
        chair.visual(
            Box((0.04, 0.04, up_h)),
            origin=Origin(xyz=(-half_depth, uy, up_cz)),
            material="wood_dark",
            name=f"back_upright_{i}",
        )

    # ------------------------------------------------------ armrests
    # Board from the backrest to a front post on the seat rail, at each arm Y.
    for i in range(2):
        ay = (-1.0, 1.0)[i] * ARM_Y
        # Arm post at the front of the seat
        chair.visual(
            Box((0.045, 0.045, 0.17)),
            origin=Origin(xyz=(half_depth, ay, -1.07)),
            material="wood_dark",
            name=f"arm_post_{i}",
        )
        # Armrest board spanning from rear to front
        chair.visual(
            Box((SEAT_DEPTH + 0.06, 0.08, 0.03)),
            origin=Origin(xyz=(-0.01, ay, -0.985)),
            material="wood",
            name=f"armrest_{i}",
        )

    # ------------------------------------------------------ chair mount (fixed to driver arm)
    model.articulation(
        "chair_mount",
        ArticulationType.FIXED,
        parent=arms[0],
        child=chair,
        origin=Origin(xyz=(0.0, ARM_Y, 0.0)),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("a_frame_stand")
    chair = object_model.get_part("swing_chair")
    arms = [object_model.get_part(f"swing_arm_{i}") for i in range(4)]
    driver = object_model.get_articulation(DRIVE_JOINT)
    followers = [object_model.get_articulation(f"swing_pivot_{i}") for i in range(1, 4)]

    # --- intentional overlaps ---------------------------------------------
    for arm in arms:
        ctx.allow_overlap(
            frame, arm,
            elem_a="top_beam", elem_b="bar",
            reason="Arm top is captured on a pivot bolt passing through the beam.",
        )
    for arm in (arms[0], arms[1]):
        ctx.allow_overlap(
            frame, arm,
            elem_a="top_beam", elem_b="pivot_bolt",
            reason="Pivot bolt passes through the top beam.",
        )
    for fr, rr in ((arms[0], arms[2]), (arms[1], arms[3])):
        ctx.allow_overlap(
            fr, rr,
            elem_a="bar", elem_b="bar",
            reason="Front and rear hanger share the pivot boss; bars stack on one bolt.",
        )
        ctx.allow_overlap(
            fr, rr,
            elem_a="pivot_bolt", elem_b="bar",
            reason="Shared pivot bolt passes through both hanger bars.",
        )
    for i, arm in enumerate(arms):
        s = i % 2  # armrest index: left=0, right=1
        ctx.allow_overlap(
            chair, arm,
            elem_a=f"armrest_{s}", elem_b="bar",
            reason="Arm lower end is bolted onto the chair armrest side.",
        )
        ctx.allow_overlap(
            chair, arm,
            elem_a=f"arm_post_{s}", elem_b="bar",
            reason="Hanger bar passes through the arm post at the chair mounting point.",
        )
    # Rear hanger bars pass through/near the curved back panel and uprights
    # where they attach to the chair frame at the backrest mounting region.
    for arm in (arms[2], arms[3]):
        ctx.allow_overlap(
            chair, arm,
            elem_a="back_panel", elem_b="bar",
            reason="Rear hanger bar reaches into the chair frame near the backrest mounting region.",
        )
        ctx.allow_overlap(
            chair, arm,
            elem_a="back_upright_0", elem_b="bar",
            reason="Rear hanger bar contacts the back upright at the chair rear mounting point.",
        )
        ctx.allow_overlap(
            chair, arm,
            elem_a="back_upright_1", elem_b="bar",
            reason="Rear hanger bar contacts the back upright at the chair rear mounting point.",
        )

    # Arms are seated on the beam and bolted to the armrests/arm posts.
    ctx.expect_contact(
        frame, arms[0],
        elem_a="top_beam", elem_b="bar",
        name="swing arm hangs from the top beam",
    )
    ctx.expect_overlap(
        chair, arms[0],
        axes="y",
        elem_a="arm_post_0", elem_b="bar",
        min_overlap=0.01,
        name="hanger bar reaches the chair arm post",
    )
    ctx.expect_overlap(
        chair, arms[2],
        axes="y",
        elem_a="back_panel", elem_b="bar",
        min_overlap=0.01,
        name="rear hanger bar reaches the chair back region",
    )

    def count(part, prefix: str) -> int:
        return sum(1 for v in part.visuals if v.name and v.name.startswith(prefix))

    # --- four swing arms with a mimic chain --------------------------------
    n_arm_parts = sum(1 for p in object_model.parts if p.name.startswith("swing_arm_"))
    ctx.check("four wooden swing arms", n_arm_parts == 4, details=f"{n_arm_parts}")
    ctx.check("driver pivot is not a mimic", driver.mimic is None)
    for f in followers:
        ok = (
            f.mimic is not None
            and f.mimic.joint == DRIVE_JOINT
            and abs(f.mimic.multiplier - 1.0) < 1e-9
        )
        ctx.check(f"{f.name} mimics the driver with multiplier 1.0", ok)
    ctx.check(
        "swing limits are about -0.4..0.4 rad",
        abs(driver.motion_limits.lower + SWING_LIMIT) < 1e-9
        and abs(driver.motion_limits.upper - SWING_LIMIT) < 1e-9,
    )

    # --- frame: A-frames, beam, trays on both sides -------------------------
    ctx.check("four A-frame legs", count(frame, "leg_") == 4)
    ctx.check("side cross rails on both A-frames", count(frame, "side_rail_") == 2)
    ctx.check("top beam present", frame.get_visual("top_beam") is not None)
    fb = ctx.part_world_aabb(frame)
    if fb is not None:
        ctx.check("stand rests on the ground", abs(fb[0][2]) < 0.01, details=f"zmin={fb[0][2]:.4f}")
    for s, sgn in ((0, -1.0), (1, 1.0)):
        tray = ctx.part_element_world_aabb(frame, elem=f"tray_shelf_{s}")
        leg = ctx.part_element_world_aabb(frame, elem=f"leg_front_{s}")
        if tray is not None and leg is not None:
            cy = (tray[0][1] + tray[1][1]) / 2.0
            leg_cy = (leg[0][1] + leg[1][1]) / 2.0
            ctx.check(
                f"tray shelf {s} sits outboard of its A-frame at armrest height",
                sgn * cy > sgn * leg_cy + 0.08 and 0.55 < tray[1][2] < 0.70,
                details=f"tray_cy={cy:.3f} leg_cy={leg_cy:.3f} top={tray[1][2]:.3f}",
            )

    # --- canopy above the chair, scalloped skirt below the canopy edge ------
    panel = ctx.part_element_world_aabb(frame, elem="canopy_panel_front")
    cb = ctx.part_world_aabb(chair)
    if panel is not None and cb is not None:
        ctx.check(
            "green canopy is above the chair",
            panel[0][2] > cb[1][2] + 0.5,
            details=f"panel_zmin={panel[0][2]:.2f} chair_zmax={cb[1][2]:.2f}",
        )
    skirt_names = (
        "canopy_skirt_front",
        "canopy_skirt_rear",
        "canopy_skirt_side_0",
        "canopy_skirt_side_1",
    )
    for sk in skirt_names:
        skirt = ctx.part_element_world_aabb(frame, elem=sk)
        ok = skirt is not None
        details = "missing"
        if skirt is not None and panel is not None:
            h = skirt[1][2] - skirt[0][2]
            ok = skirt[0][2] < panel[0][2] - 0.05 and skirt[1][2] > panel[0][2] and 0.10 < h < 0.25
            details = f"zmin={skirt[0][2]:.3f} zmax={skirt[1][2]:.3f} h={h:.3f}"
        ctx.check(f"scalloped skirt {sk} hangs below the canopy edge", ok, details=details)

    # --- single-person chair: narrow width, slats, curved back ---------------
    n_seat = count(chair, "seat_slat_")
    ctx.check("seat has four slats (single person)", n_seat == 4, details=f"{n_seat}")

    # Chair width should be narrow (~0.5m, not the 1.1m two-seat bench)
    if cb is not None:
        chair_width = cb[1][1] - cb[0][1]
        ctx.check(
            "single-person chair width (~0.5 m)",
            0.42 < chair_width < 0.72,
            details=f"w={chair_width:.2f}",
        )

    # Curved backrest: panel + curved rails present
    ctx.check("curved back panel present", chair.get_visual("back_panel") is not None)
    ctx.check("curved bottom rail present", chair.get_visual("back_bottom_rail") is not None)
    ctx.check("curved top rail present", chair.get_visual("back_top_rail") is not None)

    # Back panel Y-width should match single-person scale
    back_aabb = ctx.part_element_world_aabb(chair, elem="back_panel")
    if back_aabb is not None:
        back_w = back_aabb[1][1] - back_aabb[0][1]
        ctx.check(
            "backrest width sized for one (~0.42 m)",
            0.35 < back_w < 0.55,
            details=f"back_w={back_w:.3f}",
        )
        # Curved panel has meaningful X-depth beyond just recline thickness
        back_dx = back_aabb[1][0] - back_aabb[0][0]
        ctx.check(
            "back panel shows curvature (depth > flat panel)",
            back_dx > 0.10,
            details=f"dx={back_dx:.3f}",
        )

    # Contoured seat: center slats lower than edge slats
    slat_0 = ctx.part_element_world_aabb(chair, elem="seat_slat_0")
    slat_1 = ctx.part_element_world_aabb(chair, elem="seat_slat_1")
    if slat_0 is not None and slat_1 is not None:
        ctx.check(
            "seat slats show contour (edge higher than center)",
            slat_0[1][2] > slat_1[1][2] + 0.001,
            details=f"slat0_top={slat_0[1][2]:.4f} slat1_top={slat_1[1][2]:.4f}",
        )

    # Two armrests
    ctx.check("two armrests", count(chair, "armrest_") == 2)

    # Seat at sitting height
    seat = ctx.part_element_world_aabb(chair, elem="seat_slat_0")
    if seat is not None:
        ctx.check(
            "seat at sitting height (~0.50 m)",
            0.44 < seat[1][2] < 0.57,
            details=f"seat_top={seat[1][2]:.3f}",
        )

    # --- swing clearance at the limits --------------------------------------
    leg_inner = LEG_Y - 0.0325

    def _swing_aabb():
        boxes = [ctx.part_world_aabb(chair)] + [ctx.part_world_aabb(a) for a in arms]
        boxes = [b for b in boxes if b is not None]
        if not boxes:
            return None
        mn = [min(b[0][i] for b in boxes) for i in range(3)]
        mx = [max(b[1][i] for b in boxes) for i in range(3)]
        return (tuple(mn), tuple(mx))

    centers = {}
    for q in (-SWING_LIMIT, 0.0, SWING_LIMIT):
        with ctx.pose({driver: q}):
            sw = _swing_aabb()
            if sw is None:
                continue
            ctx.check(
                f"swing assembly clears the ground at q={q:+.1f}",
                sw[0][2] > 0.05,
                details=f"zmin={sw[0][2]:.3f}",
            )
            ctx.check(
                f"swing assembly clears the A-frame legs sideways at q={q:+.1f}",
                sw[1][1] < leg_inner - 0.10 and sw[0][1] > -(leg_inner - 0.10),
                details=f"ymax={sw[1][1]:.3f} limit={leg_inner - 0.10:.3f}",
            )
            cb_q = ctx.part_world_aabb(chair)
            if cb_q is not None:
                centers[q] = (cb_q[0][0] + cb_q[1][0]) / 2.0
                ctx.check(
                    f"chair stays below the top beam at q={q:+.1f}",
                    cb_q[1][2] < BEAM_Z - BEAM_W / 2.0 - 0.05,
                    details=f"chair_zmax={cb_q[1][2]:.3f}",
                )
    if -SWING_LIMIT in centers and SWING_LIMIT in centers:
        travel = centers[-SWING_LIMIT] - centers[SWING_LIMIT]
        ctx.check(
            "chair swings fore/aft on the pivots",
            travel > 0.35,
            details=f"travel={travel:.3f}",
        )

    return ctx.report()
