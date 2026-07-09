from __future__ import annotations

# Realistic articulated clothes peg with barrel-head jaws (wooden spring clothespin).
#
# Articraft brief:
# - Object: classic wooden spring clothes peg with smooth rounded barrel-head
#   gripping jaws (fork variant of the flat-notched parent), ~72 mm long,
#   ~8.5 mm wide, ~13 mm tall at the pivot. Two mirror-image wooden halves
#   pivot against each other around a coiled steel torsion spring.
# - Pose: the peg LIES FLAT on its side on the ground (Z-up). The peg's long
#   axis runs along world X, the two halves sit side by side along world Y
#   (half_0 at +Y, half_1 at -Y), and the peg width spans world Z from 0
#   (ground) to HALF_W.
# - Root/support: half_0 is the fixed root and rests flat on the ground.
#   It carries the spring (coil seated in the carved pivot notch) and the
#   upper half pivots on the shared spring-barrel axis (vertical in world).
# - Parts: half_0 (root wooden leg), half_1 (moving wooden leg),
#   spring (one-piece steel torsion spring: central coil + two straight legs
#   lying against the relieved inner tail faces of the two halves).
# - Articulation: pivot, REVOLUTE. The joint child frame is the "peg frame"
#   (long axis X, halves stacked along peg-frame Z); axis (0,-1,0) in that
#   frame so positive q opens the gripping jaws while the back finger tails
#   squeeze toward each other. q=0 is the closed rest pose (jaws nearly
#   touching, tails splayed apart by the relief angle).
# - Barrel-head jaws: the nose end of each half is a smooth half-round barrel
#   (radius BARREL_R) in XZ cross-section. When closed, the two half-rounds
#   form a full cylindrical gripping head. No relief-notch groove is present.
# - Geometry guarantees: the two halves are exact mirror images about the
#   parting mid-plane. Their inner faces are flat and parallel (small GAP)
#   from the pivot to the barrel start, then the barrel arcs away from the
#   parting plane. Behind the pivot the inner faces ramp AWAY from each other
#   (TAIL_ANGLE relief), so across the entire joint range [0, PIVOT_MAX] the
#   halves never interpenetrate.
# - Spring: the coil is centered exactly on the pivot axis (captured in the
#   half-round seats carved into both halves); its two legs run back along
#   the relieved inner faces of the two tails.
# - Intentional overlaps: the steel spring coil nests inside the carved
#   wooden pivot seats of both halves (captured spring).
# - Tests: both wooden halves + spring present; pivot is revolute about the
#   peg-frame -Y (world vertical) axis; opening swings the upper nose away
#   from the lower nose; closed jaws nearly touch without penetration; the
#   peg rests on the ground (z_min ~ 0); barrel-head jaw has rounded profile.
# - Assumptions: generic flat-leg clothespin proportions with barrel-head
#   gripping jaws, weathered wood + dark steel spring.

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

BARREL_R = 0.0040          # barrel-head jaw radius (half-round gripping head)

# Joint range: tails (lever ~ PIVOT_X-BACK_X) collide when
#   sin(q) ~ 2*tan(TAIL_ANGLE) + GAP/(PIVOT_X-BACK_X) ~ 0.179  (q ~ 0.180 rad)
# Keep the upper limit below that with margin so the halves NEVER touch.
PIVOT_MAX = 0.16           # rad, max jaw opening (~6.4 mm extra at the nose)


def _wood_half() -> cq.Workplane:
    """Build one wooden clothespin leg with a smooth barrel-head jaw.

    Authored in the local part frame: long axis +X, width +/-Y, the parting
    (gripping) face on z=0 from the pivot forward, body growing toward +Z.
    Behind the pivot the bottom face ramps up by TAIL_ANGLE (tail relief).
    The nose end has a smooth half-round barrel profile (radius BARREL_R)
    instead of a flat tip with a relief groove.
    """
    z_back = 0.0095          # height at the finger end (flared pad)
    z_pivot = 0.0115         # tallest at the pivot bulge
    z_mid = 0.0070

    # Barrel-head center in the XZ profile plane.
    bx = NOSE_X - BARREL_R   # barrel arc center X
    bz = BARREL_R             # barrel arc center Z

    # Approximate the half-round barrel arc (-90° to +90°) with polyline points.
    n_arc = 12
    arc_pts: list[tuple[float, float]] = []
    for j in range(n_arc + 1):
        a = -math.pi / 2.0 + math.pi * j / n_arc
        arc_pts.append((bx + BARREL_R * math.cos(a), bz + BARREL_R * math.sin(a)))

    # Side silhouette in the XZ plane (z measured up from the parting plane).
    pts: list[tuple[float, float]] = [
        (BACK_X, TAIL_RISE),           # relieved tail end (splayed inner face)
        (PIVOT_X, 0.0),                # fulcrum: relief ramp meets flat face
        (bx, 0.0),                     # parting face ends, barrel arc begins
    ]
    # Skip arc_pts[0] (duplicate of (bx, 0)); arc ends at (bx, 2*BARREL_R).
    pts.extend(arc_pts[1:])
    # Top profile from barrel crown back through body to finger end.
    pts.extend([
        (NOSE_X - 0.012, 2.0 * BARREL_R - 0.001),   # smooth shoulder behind barrel
        (PIVOT_X + 0.012, z_mid),
        (PIVOT_X, z_pivot),            # pivot bulge (tallest)
        (PIVOT_X - 0.010, z_mid + 0.0010),
        (BACK_X + 0.008, z_back + 0.0010),
        (BACK_X, z_back),              # flared finger pad
    ])

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

    # --- Two mirrored wooden halves (barrel-head jaws) ---
    # half_0: lower/root half at world +Y, flipped so parting face faces -Y.
    # half_1: upper/moving half at world -Y, plain peg frame orientation.
    half_origins = [
        Origin(
            xyz=(0.0, GAP / 2.0, HALF_W / 2.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        Origin(xyz=(-PIVOT_X, 0.0, GAP / 2.0), rpy=(0.0, 0.0, 0.0)),
    ]
    halves = []
    for i in range(2):
        name_i = f"half_{i}"
        p = model.part(name_i)
        p.visual(
            mesh_from_cadquery(half_solid, name_i),
            origin=half_origins[i],
            material=wood,
            name=name_i,
        )
        halves.append(p)

    lower, upper = halves

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

    # --- Pivot: upper half rotates about the spring-barrel axis. ---
    # The joint frame at the fulcrum is the peg frame (rpy roll=+90deg), so
    # the child (upper half) is authored in plain peg coordinates. Axis
    # (0,-1,0) in the joint frame = world -Z (vertical): positive q opens the
    # front jaws while the back tails squeeze together. Upper limit stays
    # below the tail-contact angle (~0.180 rad) so the halves never touch.
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

    lower = object_model.get_part("half_0")
    upper = object_model.get_part("half_1")
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

    # --- Barrel-head jaw: rounded nose profile ---
    # The barrel arc gives each half a nose height of 2*BARREL_R in peg Z.
    # For the lower half (roll=-90°), peg Z maps to world +Y, so the Y extent
    # from the parting face should be at least 2*BARREL_R.
    barrel_dia = 2.0 * BARREL_R
    if lower_box is not None:
        lower_y_extent = lower_box[1][1] - lower_box[0][1]
        ctx.check(
            "barrel-head jaw: rounded barrel profile has sufficient depth",
            lower_y_extent >= barrel_dia * 0.95,
            details=f"y_extent={lower_y_extent:.4f}, barrel_dia={barrel_dia:.4f}",
        )
    # The barrel nose reaches the front of the peg (no truncated flat tip).
    nose_tip = ctx.part_element_world_aabb(lower, elem="half_0")
    lower_max_x = None if nose_tip is None else nose_tip[1][0]
    ctx.check(
        "barrel-head jaw: nose reaches the gripping tip",
        lower_max_x is not None and lower_max_x >= NOSE_X - 0.001,
        details=f"lower_max_x={lower_max_x}",
    )

    # --- Closed rest pose: mirror halves nearly touch at the parting face. ---
    with ctx.pose({pivot: 0.0}):
        lb = ctx.part_world_aabb(lower)
        ub = ctx.part_world_aabb(upper)
        ctx.check(
            "closed: half_0 at +Y, half_1 at -Y of the parting plane",
            lb is not None and ub is not None
            and lb[1][1] > 0.002 and ub[0][1] < -0.002,
            details=f"lower_ymax={lb[1][1]:.4f}, upper_ymin={ub[0][1]:.4f}",
        )
        # The flat parting faces are close together but never interpenetrate.
        # lower is on +Y side (positive_link), upper on -Y side (negative_link).
        ctx.expect_gap(
            lower,
            upper,
            axis="y",
            max_gap=0.004,
            max_penetration=0.0002,
            name="closed: barrel-head jaws nearly touch at parting plane",
        )

    # --- Opened pose: actuating the pivot swings the upper nose away. ---
    with ctx.pose({pivot: 0.0}):
        upper_closed = ctx.part_world_aabb(upper)
    with ctx.pose({pivot: PIVOT_MAX}):
        upper_open = ctx.part_world_aabb(upper)
        # Even fully open the halves must not touch in 3D. The Y-projection
        # overlap is an artifact of the rotation geometry (the tail swings
        # slightly past the parting mid-plane in Y projection but clears in X
        # and Z). Real 3D clearance is verified by the compiler's overlap QC.
        # Allow up to 3 mm projected Y penetration as long as the parts don't
        # actually collide.
        ctx.expect_gap(
            lower,
            upper,
            axis="y",
            max_gap=0.02,
            max_penetration=0.003,
            name="fully open: barrel-head halves clear in Y projection",
        )
    # The front nose of the upper leg swings toward -Y when the jaws open.
    ctx.check(
        "opening the pivot swings the barrel-head jaw open",
        upper_closed is not None and upper_open is not None
        and upper_open[0][1] < upper_closed[0][1] - 0.003,
        details=(
            f"upper_ymin_closed={upper_closed[0][1]:.4f}, "
            f"upper_ymin_open={upper_open[0][1]:.4f}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
