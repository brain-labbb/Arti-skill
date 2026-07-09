from __future__ import annotations

# Hinged swing-top bottle with wire bail geometry.
# Variant of the clear plastic juice bottle: Grolsch-style flip-top.
# Frame: bottle axis along +Z, base at z=0, neck/mouth at the top (+Z).
#
# Parts:
#   bottle (root): transparent PET shell with rounded base, cylindrical barrel,
#     shoulder taper, short neck with raised lip ring, and hollow mouth opening.
#   bail: wire bail frame that clips onto the neck lip and extends upward;
#     modelled as a single smooth tube swept through spline points.
#   cap: ceramic-style swing stopper with rubber gasket, carried by the bail
#     via a continuous-rotation joint.
#
# Articulations:
#   swing_hinge: REVOLUTE, horizontal axis at the neck lip; positive q swings
#     the bail+cap assembly open (away from the mouth).
#   cap_rotate: CONTINUOUS, vertical axis through the stopper centre.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ─── key dimensions (metres) ────────────────────────────────────────────────
BODY_R = 0.030          # outer barrel radius
WALL = 0.0016           # thin PET wall
BASE_Z = 0.0
BARREL_TOP_Z = 0.108
SHOULDER_TOP_Z = 0.132
NECK_R = 0.013          # neck outer radius (smooth, no threads)
NECK_TOP_Z = 0.150      # top rim of the neck / mouth plane

# Lip ring (raised ridge where the bail clips on)
LIP_R = 0.017           # lip ring outer radius
LIP_Z_CENTRE = 0.140    # vertical centre of the lip ring
LIP_H = 0.005           # lip ring height

# Mouth rim (thin transparent wall-thickness ring at the opening)
MOUTH_RIM_R = NECK_R + 0.001   # slightly proud of the neck OD
MOUTH_RIM_H = 0.003

# Bail wire
WIRE_R = 0.0012         # wire cross-section radius (~2.4 mm dia)
ARM_TOP = 0.035         # bail arch peak above the hinge origin
NECK_OUT = NECK_R + 0.003  # wire sits just outside the neck

# Hinge sits at the lip ring centre height
HINGE_Z = LIP_Z_CENTRE + LIP_H / 2.0  # ≈ 0.1425

# Cap / stopper
STOPPER_R = 0.012
STOPPER_H = 0.006       # stopper disc thickness
GASKET_R = 0.0135
GASKET_H = 0.002
STEM_R = 0.002          # thin stem connecting bail arch to stopper

# Cap hangs from the bail arch: the cap_rotate origin is at the arch peak
# in bail-local coords.  The stopper disc sits ON TOP of the mouth plane
# and only the thin gasket compresses slightly into the rim.
# Cap origin world Z = HINGE_Z + ARM_TOP
_CAP_ORIGIN_Z = HINGE_Z + ARM_TOP
# Gasket bottom: 1 mm below the mouth for a seated seal
GASKET_BOT_LOCAL = (NECK_TOP_Z - 0.001) - _CAP_ORIGIN_Z     # ≈ -0.0285
GASKET_TOP_LOCAL = GASKET_BOT_LOCAL + GASKET_H              # ≈ -0.0265
DISC_BOT_LOCAL = GASKET_TOP_LOCAL                           # ≈ -0.0265
DISC_TOP_LOCAL = DISC_BOT_LOCAL + STOPPER_H                 # ≈ -0.0205
STEM_LEN = abs(DISC_TOP_LOCAL)                               # ≈ 0.0205


# ─── geometry builders ──────────────────────────────────────────────────────

def _bottle_shell():
    """Transparent thin-wall bottle as one revolved solid, shelled open at top."""
    wp = (
        cq.Workplane("XZ")
        .moveTo(0.0, BASE_Z)
        .lineTo(BODY_R - 0.006, BASE_Z)
        .threePointArc((BODY_R, BASE_Z + 0.006), (BODY_R, BASE_Z + 0.012))
        .lineTo(BODY_R, BARREL_TOP_Z)
        .threePointArc(
            ((BODY_R + NECK_R) / 2.0, (BARREL_TOP_Z + SHOULDER_TOP_Z) / 2.0 + 0.004),
            (NECK_R, SHOULDER_TOP_Z),
        )
    )
    # Smooth neck up to lip ring
    wp = wp.lineTo(NECK_R, LIP_Z_CENTRE - LIP_H / 2.0)
    # Lip ring ridge (raised bump on the neck profile)
    wp = wp.lineTo(LIP_R, LIP_Z_CENTRE - LIP_H / 2.0)
    wp = wp.lineTo(LIP_R, LIP_Z_CENTRE + LIP_H / 2.0)
    wp = wp.lineTo(NECK_R, LIP_Z_CENTRE + LIP_H / 2.0)
    # Continue neck up to mouth
    wp = wp.lineTo(NECK_R, NECK_TOP_Z)
    # Close along axis
    wp = wp.lineTo(0.0, NECK_TOP_Z).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    # Shell open at the top so the mouth reads hollow
    return outer.faces(">Z").shell(-WALL)


def _mouth_rim():
    """Thin transparent ring at the mouth showing wall thickness."""
    rim = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, NECK_TOP_Z - MOUTH_RIM_H))
        .circle(MOUTH_RIM_R)
        .circle(NECK_R - WALL)
        .extrude(MOUTH_RIM_H)
    )
    return rim


def _bail_wire_mesh():
    """Wire bail as a smooth tube swept through spline points, plus a cross-wire.

    Path is in bail-local coordinates (origin at the hinge point).
    The main U-shaped arch clips onto the neck lip; a cross-wire wraps
    around the front of the neck connecting the two hook tips for 3D depth.
    """
    hook_drop = 0.008
    points = [
        (0.0, -NECK_OUT, -hook_drop),                   # left hook tip
        (0.0, -NECK_OUT, -0.003),                       # left hook bend
        (0.0, -NECK_OUT, 0.002),                        # left side near hinge
        (0.0, -NECK_OUT * 0.75, ARM_TOP * 0.55),        # left arm mid
        (0.0, -NECK_OUT * 0.35, ARM_TOP * 0.9),         # left arm upper
        (0.0, 0.0, ARM_TOP),                            # arch peak
        (0.0, NECK_OUT * 0.35, ARM_TOP * 0.9),          # right arm upper
        (0.0, NECK_OUT * 0.75, ARM_TOP * 0.55),         # right arm mid
        (0.0, NECK_OUT, 0.002),                         # right side near hinge
        (0.0, NECK_OUT, -0.003),                        # right hook bend
        (0.0, NECK_OUT, -hook_drop),                    # right hook tip
    ]
    main_tube = tube_from_spline_points(
        points,
        radius=WIRE_R,
        samples_per_segment=16,
        radial_segments=12,
        cap_ends=True,
    )

    # Cross-wire wraps around front of neck connecting the two hook tips
    cross_points = [
        (0.0, -NECK_OUT, -hook_drop),                   # left hook tip (shared)
        (NECK_OUT * 0.7, -NECK_OUT * 0.3, -hook_drop - 0.002),
        (NECK_OUT * 0.9, 0.0, -hook_drop - 0.003),     # front centre
        (NECK_OUT * 0.7, NECK_OUT * 0.3, -hook_drop - 0.002),
        (0.0, NECK_OUT, -hook_drop),                    # right hook tip (shared)
    ]
    cross_tube = tube_from_spline_points(
        cross_points,
        radius=WIRE_R,
        samples_per_segment=14,
        radial_segments=12,
        cap_ends=True,
    )

    return main_tube.merge(cross_tube)


def _cap_stopper():
    """Ceramic-style swing stopper with stem.

    Cap-local origin is at the cap_rotate articulation frame (at the bail arch
    peak). Geometry extends downward: stem, then disc, then gasket.
    """
    # Stem from arch down to disc top
    stem = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, DISC_TOP_LOCAL))
        .circle(STEM_R)
        .extrude(STEM_LEN)
    )

    # Stopper disc
    disc = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, DISC_BOT_LOCAL))
        .circle(STOPPER_R)
        .extrude(STOPPER_H)
    )
    # Fillet bottom edge for a seated look
    disc = disc.edges("<Z").fillet(0.001)

    return disc.union(stem)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_top_bottle")

    # Materials
    clear = model.material("clear_pet", rgba=(0.80, 0.88, 0.85, 0.25))
    ceramic = model.material("stopper_ceramic", rgba=(0.92, 0.88, 0.82, 1.0))
    rubber = model.material("gasket_rubber", rgba=(0.15, 0.10, 0.08, 1.0))
    steel = model.material("bail_steel", rgba=(0.55, 0.56, 0.58, 1.0))

    # ─── bottle body (root) ──────────────────────────────────────────────
    body = model.part("bottle")
    shell = _bottle_shell()
    body.visual(
        mesh_from_cadquery(shell, "bottle_shell"),
        material=clear,
        name="bottle_shell",
    )
    # Mouth rim: thin transparent ring showing wall thickness at the opening
    rim = _mouth_rim()
    body.visual(
        mesh_from_cadquery(rim, "mouth_rim"),
        material=clear,
        name="mouth_rim",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.032,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ─── bail (wire frame) ──────────────────────────────────────────────
    bail = model.part("bail")
    bail_mesh = _bail_wire_mesh()
    bail.visual(
        mesh_from_geometry(bail_mesh, "bail_wire"),
        material=steel,
        name="bail_wire",
    )
    bail.inertial = Inertial.from_geometry(
        Box((0.02, 0.04, 0.05)),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, ARM_TOP / 2.0)),
    )

    # ─── cap (swing stopper) ────────────────────────────────────────────
    cap = model.part("cap")
    cap_geo = _cap_stopper()
    cap.visual(
        mesh_from_cadquery(cap_geo, "stopper_body"),
        material=ceramic,
        name="stopper_body",
    )
    # Gasket as a separate named visual for testing
    gasket_geo = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, GASKET_BOT_LOCAL))
        .circle(GASKET_R)
        .extrude(GASKET_H)
    )
    cap.visual(
        mesh_from_cadquery(gasket_geo, "gasket_ring"),
        material=rubber,
        name="gasket_ring",
    )
    # Off-axis marker tab on the disc edge (makes rotation detectable)
    marker_z = DISC_BOT_LOCAL + STOPPER_H * 0.5  # at disc mid-height
    cap.visual(
        Box((0.005, 0.004, 0.004)),
        origin=Origin(xyz=(STOPPER_R + 0.002, 0.0, marker_z)),
        material=ceramic,
        name="cap_marker",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(STOPPER_R, STOPPER_H + GASKET_H),
        mass=0.010,
        origin=Origin(xyz=(0.0, 0.0, DISC_BOT_LOCAL + STOPPER_H / 2.0)),
    )

    # ─── articulations ──────────────────────────────────────────────────

    # Swing hinge: bail swings open on a horizontal axis at the neck lip.
    # Positive q opens the cap away from the mouth.
    # The bail arch extends along local +Z from the hinge; using +X axis
    # makes positive q tip the arch toward -Y (backward / open).
    model.articulation(
        "swing_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bail,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=2.0,
        ),
    )

    # Cap rotate: stopper spins on its vertical axis (continuous).
    # Origin at the bail arch peak in bail-local coords.
    model.articulation(
        "cap_rotate",
        ArticulationType.CONTINUOUS,
        parent=bail,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, ARM_TOP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle")
    bail = object_model.get_part("bail")
    cap = object_model.get_part("cap")
    swing = object_model.get_articulation("swing_hinge")
    rotate = object_model.get_articulation("cap_rotate")

    bottle_shell = body.get_visual("bottle_shell")
    mouth_rim = body.get_visual("mouth_rim")
    bail_wire = bail.get_visual("bail_wire")
    stopper = cap.get_visual("stopper_body")
    gasket = cap.get_visual("gasket_ring")

    # ─── bottle is transparent ──────────────────────────────────────────
    ctx.check(
        "bottle material is tinted-transparent (alpha < 1)",
        bottle_shell.material.rgba is not None and bottle_shell.material.rgba[3] < 1.0,
        details=f"bottle rgba={bottle_shell.material.rgba}",
    )

    # ─── mouth rim exists and is transparent ────────────────────────────
    ctx.check(
        "mouth rim is transparent (wall thickness visible)",
        mouth_rim.material.rgba is not None and mouth_rim.material.rgba[3] < 1.0,
        details=f"rim rgba={mouth_rim.material.rgba}",
    )

    # ─── bail wire exists and is opaque metal ──────────────────────────
    ctx.check(
        "bail wire is opaque metal",
        bail_wire.material.rgba is not None and bail_wire.material.rgba[3] >= 0.99,
        details=f"bail rgba={bail_wire.material.rgba}",
    )

    # ─── cap stopper exists ─────────────────────────────────────────────
    ctx.check(
        "stopper body exists with ceramic material",
        stopper is not None and stopper.material.rgba is not None,
        details=f"stopper material={stopper.material.rgba if stopper else None}",
    )

    # ─── gasket ring exists ─────────────────────────────────────────────
    ctx.check(
        "gasket ring exists with rubber material",
        gasket is not None,
        details="gasket visual missing",
    )

    # ─── swing hinge is REVOLUTE with limits ────────────────────────────
    ctx.check(
        "swing_hinge is revolute with finite limits",
        swing.articulation_type == ArticulationType.REVOLUTE
        and swing.motion_limits is not None
        and swing.motion_limits.lower is not None
        and swing.motion_limits.upper is not None
        and swing.motion_limits.upper > swing.motion_limits.lower,
        details=f"swing type={swing.articulation_type}, limits={swing.motion_limits}",
    )

    # ─── cap_rotate is CONTINUOUS ───────────────────────────────────────
    ctx.check(
        "cap_rotate is continuous",
        rotate.articulation_type == ArticulationType.CONTINUOUS,
        details=f"rotate type={rotate.articulation_type}",
    )

    # ─── swing hinge opens: cap moves away from mouth ───────────────────
    rest_z = ctx.part_world_position(cap)
    with ctx.pose({swing: 1.0}):
        open_z = ctx.part_world_position(cap)
    ctx.check(
        "swing hinge moves the cap away from the rest position",
        rest_z is not None and open_z is not None
        and (abs(open_z[0] - rest_z[0]) > 0.005
             or abs(open_z[1] - rest_z[1]) > 0.005
             or abs(open_z[2] - rest_z[2]) > 0.005),
        details=f"rest={rest_z}, open={open_z}",
    )

    # ─── cap rotation moves the off-axis marker ────────────────────────
    def _ext(ab):
        mn, mx = ab
        return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])

    aabb0 = None
    aabb90 = None
    with ctx.pose({rotate: 0.0}):
        aabb0 = ctx.part_world_aabb(cap)
    with ctx.pose({rotate: math.pi / 2.0}):
        aabb90 = ctx.part_world_aabb(cap)
    e0 = _ext(aabb0)
    e90 = _ext(aabb90)
    ctx.check(
        "cap rotation moves the off-axis marker (extents swap x<->y)",
        aabb0 is not None and aabb90 is not None
        and abs(e0[0] - e90[1]) < 0.003
        and abs(e0[0] - e0[1]) > 0.001,
        details=f"rest extents={e0}, quarter-turn extents={e90}",
    )

    # ─── bail wire is mounted near the bottle neck ──────────────────────
    bail_pos = ctx.part_world_position(bail)
    ctx.check(
        "bail mounted at the neck lip area",
        bail_pos is not None and bail_pos[2] > SHOULDER_TOP_Z - 0.01,
        details=f"bail origin={bail_pos}",
    )

    # ─── cap seated near the mouth at rest ──────────────────────────────
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap near mouth level when closed",
        cap_pos is not None and abs(cap_pos[2] - NECK_TOP_Z) < 0.040,
        details=f"cap_z={cap_pos[2] if cap_pos else None}, mouth_z={NECK_TOP_Z}",
    )

    # Allow bail hooks to overlap the bottle neck lip (seated clip)
    ctx.allow_overlap(
        bail,
        body,
        elem_a="bail_wire",
        elem_b="bottle_shell",
        reason="Bail wire hooks clip over the neck lip ring — intentional seated overlap.",
    )

    # Allow cap stopper to overlap the bottle shell at the mouth (seated seal)
    # The stopper disc sits just above the mouth; only the gasket penetrates slightly.
    ctx.allow_overlap(
        cap,
        body,
        elem_a="gasket_ring",
        elem_b="bottle_shell",
        reason="Rubber gasket compresses 1mm into the mouth rim as a seated seal.",
    )

    # Also allow the gasket to overlap the mouth rim visual
    ctx.allow_overlap(
        cap,
        body,
        elem_a="gasket_ring",
        elem_b="mouth_rim",
        reason="Gasket wraps around the mouth rim for sealing contact.",
    )

    # Prove the gasket is seated at the mouth
    ctx.expect_overlap(
        cap,
        body,
        axes="z",
        elem_a="gasket_ring",
        elem_b="bottle_shell",
        min_overlap=0.0005,
        name="gasket overlaps bottle shell at the mouth zone",
    )

    # Prove the stopper disc sits just above the mouth (small gap)
    ctx.expect_gap(
        cap,
        body,
        axis="z",
        positive_elem="stopper_body",
        negative_elem="bottle_shell",
        min_gap=-0.002,
        max_gap=0.005,
        name="stopper disc near mouth plane",
    )

    return ctx.report()


object_model = build_object_model()
