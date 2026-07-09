from __future__ import annotations

# Realistic articulated clothes peg (wooden spring clothespin).
#
# Articraft brief:
# - Object: classic wooden spring clothes peg, ~72 mm long, ~8.5 mm wide,
#   ~13 mm tall at the pivot. Two mirror-image wooden halves pivot against
#   each other around a coiled steel torsion spring.
# - Pose: the peg LIES FLAT on its side on the ground (Z-up). The peg's long
#   axis runs along world X, the two halves sit side by side along world Y
#   (lower_half at +Y, upper_half at -Y), and the peg width spans world Z
#   from 0 (ground) to HALF_W.
# - Root/support: lower_half is the fixed root and rests flat on the ground.
#   It carries the spring (coil seated in the carved pivot notch) and the
#   upper half pivots on the shared spring-barrel axis (vertical in world).
# - Parts: lower_half (root wooden leg), upper_half (moving wooden leg),
#   spring (one-piece steel torsion spring: central coil + two straight legs
#   lying against the relieved inner tail faces of the two halves).
# - Articulation: pivot, REVOLUTE. The joint child frame is the "peg frame"
#   (long axis X, halves stacked along peg-frame Z); axis (0,-1,0) in that
#   frame so positive q opens the gripping jaws while the back finger tails
#   squeeze toward each other. q=0 is the closed rest pose (jaws nearly
#   touching, tails splayed apart by the relief angle).
# - Geometry guarantees: the two halves are exact mirror images about the
#   parting mid-plane. Their inner faces are flat and parallel (small GAP)
#   from the nose back to the pivot fulcrum, then ramp AWAY from each other
#   behind the pivot (TAIL_ANGLE relief), so across the entire joint range
#   [0, PIVOT_MAX] the halves never interpenetrate: the limit is chosen
#   below the tail-contact angle computed from the relief geometry.
# - Spring: the coil is centered exactly on the pivot axis (captured in the
#   half-round seats carved into both halves); its two legs run back along
#   the relieved inner faces of the two tails.
# - Intentional overlaps: the steel spring coil nests inside the carved
#   wooden pivot seats of both halves (captured spring).
# - Tests: both wooden halves + spring present; pivot is revolute about the
#   peg-frame -Y (world vertical) axis; opening swings the upper nose away
#   from the lower nose; closed jaws nearly touch without penetration; the
#   peg rests on the ground (z_min ~ 0).
# - Back finger ends: concave thumb-dished pressing pads (not flat flared
#   pads). A spherical depression is carved into the outer face of each back
#   pad, giving the user a concave thumb-rest for pinching the peg open.
# - Assumptions: generic flat-leg clothespin proportions, weathered wood +
#   dark steel spring matching the reference image.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ----------------------------------------------------------------------------
# Dimensions (meters)
# ----------------------------------------------------------------------------
LEG_LEN = 0.072            # full peg length along X
HALF_W = 0.0085            # width of one wooden leg (peg-frame Y / world Z)
GAP = 0.0006               # closed gap between the two flat jaw faces

PIVOT_X = -0.004           # X of the pivot/fulcrum (slightly back of center)

NOSE_X = LEG_LEN / 2.0     # front gripping tip
BACK_X = -LEG_LEN / 2.0    # back finger end

# Tail relief: behind the pivot the inner face of each half ramps away from
# the parting plane so the finger tails are splayed apart at rest and can be
# squeezed together to open the jaws (like a real peg).
TAIL_ANGLE = 0.08                                   # rad, relief per half
TAIL_RISE = (PIVOT_X - BACK_X) * math.tan(TAIL_ANGLE)  # ~2.6 mm at the back end

SPRING_R = 0.0042          # coil centerline radius of the torsion spring
WIRE_R = 0.00055           # steel wire radius
SEAT_R = 0.0052            # carved half-round spring seat radius (per half)

# Back finger end profile (local part frame, z from parting plane).
Z_BACK = 0.0095            # base height at the finger end

# Concave thumb-dish pressing pad dimensions (carved into the outer face at the
# back finger end of each half, replacing the old flat flared pad).
DISH_R = 0.006             # curvature radius of the concave dish (6 mm)
DISH_DEPTH = 0.0015        # depth of the thumb depression
PAD_BULGE = 0.002          # extra height added to the back pad for dish stock

# Joint range: tails (lever ~ PIVOT_X-BACK_X) collide when
#   sin(q) ~ 2*tan(TAIL_ANGLE) + GAP/(PIVOT_X-BACK_X) ~ 0.179  (q ~ 0.180 rad)
# Keep the upper limit below that with margin so the halves NEVER touch.
PIVOT_MAX = 0.16           # rad, max jaw opening (~6.4 mm extra at the nose)


def _wood_half() -> cq.Workplane:
    """Build one wooden clothespin leg as a CadQuery solid.

    Authored in the local part frame: long axis +X, width +/-Y, the parting
    (gripping) face on z=0 from the pivot forward, body growing toward +Z.
    Behind the pivot the bottom face ramps up by TAIL_ANGLE (tail relief).
    """
    # Side silhouette in the XZ plane (z measured up from the parting plane).
    z_back = Z_BACK          # base height at the finger end
    z_pad = z_back + PAD_BULGE  # raised pressing pad top (stock for dish)
    z_pivot = 0.0115         # tallest at the pivot bulge
    z_mid = 0.0070
    z_nose = 0.0042          # thin gripping nose

    # Outer profile loop (XZ). z>=0 grows away from the parting plane.
    # The back finger end is reshaped as a rounded pressing pad (bulged up for
    # the concave thumb dish) instead of the old flat flared pad.
    pts = [
        (BACK_X, TAIL_RISE),           # relieved tail end (splayed inner face)
        (PIVOT_X, 0.0),                # fulcrum: relief ramp meets flat face
        (NOSE_X, 0.0),                 # flat parting face from pivot to nose
        (NOSE_X, z_nose),              # nose tip
        (NOSE_X - 0.010, z_nose + 0.0006),
        (PIVOT_X + 0.012, z_mid),
        (PIVOT_X, z_pivot),            # pivot bulge (tallest)
        (PIVOT_X - 0.010, z_mid + 0.0010),
        (BACK_X + 0.012, z_back + 0.001),   # shoulder into pressing pad
        (BACK_X + 0.006, z_pad + 0.0005),   # pressing pad crest (rounded top)
        (BACK_X + 0.001, z_pad),            # back edge of pressing pad
    ]

    leg = (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
        .extrude(HALF_W)              # extrude along the XZ normal (-Y) for width
    )
    # The XZ extrude lands width in Y=(-HALF_W, 0); recenter on Y=0 so the part
    # frame Y=0 is the leg centerline.
    leg = leg.translate((0.0, HALF_W / 2.0, 0.0))

    # Round the long top/back edges so it reads as a worn wooden peg.
    leg = leg.edges("|Y").fillet(0.0010)

    # --- Clothesline groove behind the gripping tip ---
    # A half-round notch carved into the parting face a few mm behind the
    # nose, so the flat tip pad still touches its mirror twin when closed
    # (as in the reference photo) while the groove grips the line.
    groove_w = HALF_W + 0.002
    groove = (
        cq.Workplane("XZ")
        .center(NOSE_X - 0.009, 0.0)
        .circle(0.0022)
        .extrude(groove_w)
        .translate((0.0, groove_w / 2.0, 0.0))
    )
    leg = leg.cut(groove)

    # --- Round spring seat (barrel notch) carved at the pivot ---
    # A half-round recess carved INTO the parting face, centered on the pivot
    # axis, so the steel coil nests inside the wood of both halves while the
    # wood still bridges over the top of the barrel (z_pivot >> SEAT_R).
    seat = (
        cq.Workplane("XZ")
        .center(PIVOT_X, 0.0)
        .circle(SEAT_R)
        .extrude(HALF_W + 0.004)
        .translate((0.0, (HALF_W + 0.004) / 2.0, 0.0))
    )
    leg = leg.cut(seat)

    # --- Concave thumb-dish pressing pad at the back finger end ---
    # A spherical depression carved into the outer (+Z) face of the back pad,
    # replacing the old flat flared pad. The dish gives the user's thumb a
    # concave resting surface for pinching the peg open. The sphere center is
    # positioned above the pad so only the lower cap cuts into the wood.
    dish_x = BACK_X + 0.005           # centered on the pressing pad
    dish_center_z = z_pad + DISH_R - DISH_DEPTH  # sphere center above surface
    dish_sphere = (
        cq.Workplane("XY")
        .center(dish_x, 0.0)
        .sphere(DISH_R)
        .translate((0.0, 0.0, dish_center_z))
    )
    leg = leg.cut(dish_sphere)

    return leg


def _spring_mesh():
    """One-piece steel torsion spring, authored in the peg frame.

    Central coil wound around +Y, centered EXACTLY on the pivot axis
    (PIVOT_X, *, 0). Two straight legs run backward from the coil, each lying
    against the relieved inner tail face of one half (lower face at
    z = -GAP/2 - d*tan(TAIL_ANGLE), upper face mirrored)."""
    tan_a = math.tan(TAIL_ANGLE)

    def lower_face_z(d: float) -> float:
        return -GAP / 2.0 - d * tan_a

    def upper_face_z(d: float) -> float:
        return GAP / 2.0 + d * tan_a

    y_lo = -0.0036
    y_hi = 0.0036

    pts: list[tuple[float, float, float]] = []

    # Straight leg resting ON the lower half's relieved tail face.
    for d in (0.024, 0.012):
        pts.append((PIVOT_X - d, y_lo, lower_face_z(d) + WIRE_R))

    # Coil: just under 2 turns around +Y, centerline exactly on the pivot axis.
    a0 = math.pi + 0.12
    sweep = 2.0 * (2.0 * math.pi) - 0.24   # ends at pi - 0.12 (mod 2*pi)
    n = 64
    for i in range(n + 1):
        t = i / n
        a = a0 + sweep * t
        y = y_lo + (y_hi - y_lo) * t
        x = PIVOT_X + SPRING_R * math.cos(a)
        z = SPRING_R * math.sin(a)
        pts.append((x, y, z))

    # Straight leg resting AGAINST the upper half's relieved tail face.
    for d in (0.012, 0.024):
        pts.append((PIVOT_X - d, y_hi, upper_face_z(d) - WIRE_R))

    spring = tube_from_spline_points(
        pts,
        radius=WIRE_R,
        samples_per_segment=8,
        radial_segments=12,
        cap_ends=True,
    )
    return mesh_from_geometry(spring, "spring")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wooden_clothes_peg")

    wood = model.material("worn_wood", rgba=(0.42, 0.29, 0.17, 1.0))
    steel = model.material("dark_steel", rgba=(0.16, 0.16, 0.18, 1.0))

    half_solid = _wood_half()

    # World placement: the peg lies flat on its side. The "peg frame" (long
    # axis X, halves stacked along peg Z) is rotated by roll=+90deg into the
    # world and lifted by HALF_W/2 so the flat side face rests on z=0:
    #   world = Trans(0,0,HALF_W/2) * Rx(+90) * peg
    # Peg-frame +Z maps to world -Y; peg-frame +/-Y (width) maps to world Z.

    # --- Lower half (root, fixed, at world +Y). ---
    # In the peg frame it is the authored leg flipped by roll=pi (parting face
    # up at z=-GAP/2, body below). Composed with the world roll=+90deg this is
    # a net roll of -90deg and an offset of (0, GAP/2, HALF_W/2).
    lower = model.part("lower_half")
    lower.visual(
        mesh_from_cadquery(half_solid, "lower_half"),
        origin=Origin(
            xyz=(0.0, GAP / 2.0, HALF_W / 2.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=wood,
        name="lower_half",
    )

    # --- Pivot: upper half rotates about the spring-barrel axis. ---
    # The joint frame at the fulcrum is the peg frame (rpy roll=+90deg), so
    # the child (upper half) is authored in plain peg coordinates. Axis
    # (0,-1,0) in the joint frame = world -Z (vertical): positive q opens the
    # front jaws while the back tails squeeze together. Upper limit stays
    # below the tail-contact angle (~0.180 rad) so the halves never touch.
    upper = model.part("upper_half")
    upper.visual(
        mesh_from_cadquery(half_solid, "upper_half"),
        # Child/joint frame sits at (PIVOT_X, 0, 0) of the peg frame; place
        # the authored mesh back at the peg origin, parting face at +GAP/2.
        origin=Origin(xyz=(-PIVOT_X, 0.0, GAP / 2.0), rpy=(0.0, 0.0, 0.0)),
        material=wood,
        name="upper_half",
    )

    # --- Spring: rides with the lower/root half at the pivot. ---
    # Authored in the peg frame with the coil centerline exactly on the pivot
    # axis; mapped into the world with the same roll=+90deg + lift transform.
    spring = model.part("spring")
    spring.visual(
        _spring_mesh(),
        origin=Origin(
            xyz=(0.0, 0.0, HALF_W / 2.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
    )
    # Rigidly mount the spring to the lower half (its coil is captured in the
    # carved pivot seat and its lower leg lies on the lower tail face).
    model.articulation(
        "lower_to_spring",
        ArticulationType.FIXED,
        parent=lower,
        child=spring,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(
            xyz=(PIVOT_X, 0.0, HALF_W / 2.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=4.0, lower=0.0, upper=PIVOT_MAX
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_half")
    upper = object_model.get_part("upper_half")
    spring = object_model.get_part("spring")
    pivot = object_model.get_articulation("pivot")
    fixed = object_model.get_articulation("lower_to_spring")

    # The steel spring intentionally nests inside the carved pivot seats of
    # both halves (it is a captured torsion spring).
    ctx.allow_overlap(
        spring,
        lower,
        reason="Torsion spring coil is captured/seated inside the lower half pivot seat.",
    )
    ctx.allow_overlap(
        spring,
        upper,
        reason="Torsion spring coil is captured/seated inside the upper half pivot seat.",
    )

    # --- Joint topology / type ---
    ctx.check(
        "pivot is revolute",
        pivot.joint_type == "revolute",
        details=f"joint_type={pivot.joint_type}",
    )
    ctx.check(
        "pivot axis is -Y of the joint frame (spring barrel, world vertical)",
        tuple(round(a, 6) for a in pivot.axis) == (0.0, -1.0, 0.0),
        details=f"axis={pivot.axis}",
    )
    ctx.check(
        "spring is fixed to lower half",
        fixed.joint_type == "fixed",
        details=f"joint_type={fixed.joint_type}",
    )

    # --- Hero parts present and placed ---
    # Spring coil sits at the pivot (its AABB brackets the pivot X), with the
    # spring legs trailing back toward the finger ends.
    spring_box = ctx.part_world_aabb(spring)
    ctx.check(
        "spring located at the pivot",
        spring_box is not None
        and spring_box[0][0] < PIVOT_X
        and spring_box[1][0] >= PIVOT_X,
        details=f"spring_aabb={spring_box}, pivot_x={PIVOT_X}",
    )
    # The coil straddles the parting mid-plane (world y=0) between the halves.
    ctx.check(
        "spring coil straddles the parting plane",
        spring_box is not None
        and spring_box[0][1] < -0.001
        and spring_box[1][1] > 0.001,
        details=f"spring_y=({spring_box[0][1]:.4f},{spring_box[1][1]:.4f})",
    )

    # Both wooden legs span roughly the full peg length along X.
    lower_box = ctx.part_world_aabb(lower)
    upper_box = ctx.part_world_aabb(upper)
    lower_len = None if lower_box is None else lower_box[1][0] - lower_box[0][0]
    upper_len = None if upper_box is None else upper_box[1][0] - upper_box[0][0]
    ctx.check(
        "lower leg spans the peg length",
        lower_len is not None and lower_len > LEG_LEN * 0.9,
        details=f"len_x={lower_len}",
    )
    ctx.check(
        "upper leg spans the peg length",
        upper_len is not None and upper_len > LEG_LEN * 0.9,
        details=f"len_x={upper_len}",
    )

    # The peg lies flat and rests on the ground plane (z ~ 0).
    ctx.check(
        "peg rests on the ground",
        lower_box is not None and abs(lower_box[0][2]) < 0.0005,
        details=f"lower_zmin={lower_box[0][2]:.5f}",
    )

    # --- Closed rest pose: mirror halves nearly touch at the jaws. ---
    with ctx.pose({pivot: 0.0}):
        lb = ctx.part_world_aabb(lower)
        ub = ctx.part_world_aabb(upper)
        ctx.check(
            "closed: lower half at +Y, upper half at -Y of the parting plane",
            lb is not None and ub is not None
            and lb[1][1] > 0.002 and ub[0][1] < -0.002,
            details=f"lower_ymax={lb[1][1]:.4f}, upper_ymin={ub[0][1]:.4f}",
        )
        # The flat jaw faces are close together but never interpenetrate.
        # Lower half is at +Y, upper half at -Y of the parting plane.
        ctx.expect_gap(
            lower,
            upper,
            axis="y",
            max_gap=0.004,
            max_penetration=0.0002,
            name="closed jaws nearly touch at parting plane",
        )

    # --- Opened pose: actuating the pivot swings the upper nose away. ---
    # At the open pose the back tails swing toward each other (this is how a
    # real clothespin works — the user squeezes the dished pressing pads
    # together to open the front jaws). The front jaws must separate.
    with ctx.pose({pivot: 0.0}):
        upper_closed = ctx.part_world_aabb(upper)
    with ctx.pose({pivot: PIVOT_MAX}):
        upper_open = ctx.part_world_aabb(upper)
    ctx.check(
        "opening the pivot swings the upper jaw open (>3 mm)",
        upper_closed is not None and upper_open is not None
        and upper_open[0][1] < upper_closed[0][1] - 0.003,
        details=(
            f"upper_ymin_closed={upper_closed[0][1]:.5f}, "
            f"upper_ymin_open={upper_open[0][1]:.5f}"
        ),
    )

    # Sanity: the lower leg reaches the gripping nose tip.
    nose_tip = ctx.part_element_world_aabb(lower, elem="lower_half")
    lower_max_x = None if nose_tip is None else nose_tip[1][0]
    ctx.check(
        "lower leg reaches the gripping nose",
        lower_max_x is not None and lower_max_x >= NOSE_X - 0.001,
        details=f"lower_max_x={lower_max_x}",
    )

    # --- Concave thumb-dished pressing pads at the back finger ends ---
    # The back finger ends are reshaped as raised pressing pads with concave
    # thumb dishes (not flat flared pads). The pad crest extends above the old
    # flat pad level (z_back + 0.001 = 0.0105 in local → world y ~ 0.0108).
    # The raised pad with dish has its outer face at world y > 0.011.
    pad_threshold = GAP / 2.0 + Z_BACK + 0.001  # above old flat pad level
    ctx.check(
        "lower back pad is raised for thumb dish (not flat flared pad)",
        lower_box is not None and lower_box[1][1] > pad_threshold,
        details=(
            f"lower_ymax={lower_box[1][1]:.5f}, "
            f"old_flat_threshold={pad_threshold:.5f}"
        ),
    )
    ctx.check(
        "upper back pad is raised for thumb dish (not flat flared pad)",
        upper_box is not None and abs(upper_box[0][1]) > pad_threshold,
        details=(
            f"upper_ymin={upper_box[0][1]:.5f}, "
            f"old_flat_threshold={pad_threshold:.5f}"
        ),
    )
    # Both halves have identical pressing pad geometry (same shared mesh).
    ctx.check(
        "both halves span the same Y extent (symmetric pressing pads)",
        lower_box is not None and upper_box is not None
        and abs((lower_box[1][1] - lower_box[0][1]) - (upper_box[1][1] - upper_box[0][1])) < 0.0002,
        details=(
            f"lower_dy={lower_box[1][1] - lower_box[0][1]:.5f}, "
            f"upper_dy={upper_box[1][1] - upper_box[0][1]:.5f}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
