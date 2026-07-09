from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Locking pliers (Vise-Grip style) with rear adjustment screw and serrated
# jaws.  Two forged-steel halves cross at a central pivot rivet.  Each half
# is one rigid link: thick serrated jaw -> slim steel neck -> curved over-
# molded handle.  The lower half is the base link and carries a rear
# adjustment thumb-screw at the handle end.  The upper half rotates about
# the central rivet axis (Z, perpendicular to the flat plane of the tool).
#
# Geometry is authored per half in a "closed-design" local frame in which
# the two halves are exactly mirrored about the XZ plane and the inner jaw
# gripping faces sit at y = +/-GRIP_FACE.  The rest pose (q=0) splays each
# half by HALF_OPEN via visual/joint-frame yaw, so q in [0, CLOSE_TRAVEL]
# closes the jaws while the handles scissor together in opposition.
# ---------------------------------------------------------------------------

HALF_OPEN = math.radians(12.0)
CLOSE_TRAVEL = 2.0 * HALF_OPEN

PLATE_T = 0.004
ZL0, ZL1 = 0.002, 0.002 + PLATE_T  # lower plate: 0.002..0.006
ZU0, ZU1 = ZL1, ZL1 + PLATE_T       # upper plate: 0.006..0.010

JAW_Z0, JAW_Z1 = ZL0, ZU1  # full-height jaw region spans both plates
JAW_X_MIN = 0.008

BOSS_R = 0.008
HOLE_R = 0.003
SHANK_R = 0.0026
HEAD_R = 0.0045

GRIP_FACE = 0.0015  # inner jaw gripping face offset from shear-plane center

# Handle over-mold z extents per half.
GRIP_H = 0.012
GRIP_LZ0, GRIP_LZ1 = 0.0, GRIP_H          # lower grip 0.0..0.012
GRIP_UZ0 = GRIP_LZ0 + (ZU0 - ZL0)         # upper grip 0.004..0.016
GRIP_UZ1 = GRIP_UZ0 + GRIP_H

# Stripe inset from handle top face.
STRIPE_EMBED = 0.0003
STRIPE_T = 0.0007

# ---- outline profiles (lower half, jaw at +y, handle at -y) ---------------

# Thick jaw profile (flat inner face near y = GRIP_FACE for serrated grip).
JAW_PTS = [
    (0.0000, 0.0120),
    (0.0080, 0.0128),
    (0.0180, 0.0125),
    (0.0300, 0.0112),
    (0.0400, 0.0088),
    (0.0470, 0.0055),
    (0.0490, 0.0035),
    (0.0465, GRIP_FACE),
    (0.0080, GRIP_FACE),
    (0.0020, 0.0065),
]

# Slim steel neck/tang from the boss back to the handle.
TANG_PTS = [
    (0.0020, -0.0035),
    (-0.0120, -0.0048),
    (-0.0250, -0.0058),
    (-0.0350, -0.0068),
    (-0.0350, -0.0120),
    (-0.0250, -0.0105),
    (-0.0120, -0.0088),
    (0.0008, -0.0078),
]

# Curved over-molded handle outline (periodic spline; widens, rounded tip).
GRIP_PTS = [
    (-0.0280, -0.0042),
    (-0.0480, -0.0055),
    (-0.0680, -0.0068),
    (-0.0880, -0.0082),
    (-0.1050, -0.0095),
    (-0.1150, -0.0108),
    (-0.1120, -0.0140),
    (-0.0960, -0.0158),
    (-0.0760, -0.0165),
    (-0.0560, -0.0152),
    (-0.0380, -0.0128),
    (-0.0290, -0.0100),
    (-0.0260, -0.0078),
]

# Accent stripe along the top face of the handle.
STRIPE_PTS = [
    (-0.0340, -0.0065),
    (-0.0520, -0.0078),
    (-0.0720, -0.0092),
    (-0.0880, -0.0100),
    (-0.0980, -0.0106),
    (-0.0930, -0.0116),
    (-0.0760, -0.0122),
    (-0.0580, -0.0115),
    (-0.0420, -0.0100),
    (-0.0350, -0.0085),
]

# Adjustment screw centre (in lower-half local frame).
SCREW_SX = -0.114
SCREW_SY = -0.0120


# ---- small geometry helpers ------------------------------------------------

def _mirror(pts, s):
    return [(x, s * y) for (x, y) in pts]


def _poly_prism(pts, z0, z1):
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, z0))
        .polyline(pts)
        .close()
        .extrude(z1 - z0)
    )


def _spline_wire(pts, z, inset=0.0):
    edge = cq.Edge.makeSpline([cq.Vector(x, y, z) for (x, y) in pts], periodic=True)
    wire = cq.Wire.assembleEdges([edge])
    if inset:
        wire = wire.offset2D(-inset)[0]
    return wire


def _soft_prism(pts, z0, z1, cap=0.0020, inset=0.0018):
    mid = cq.Solid.extrudeLinear(
        cq.Face.makeFromWires(_spline_wire(pts, z0 + cap)),
        cq.Vector(0.0, 0.0, (z1 - z0) - 2.0 * cap),
    )
    top = cq.Solid.makeLoft(
        [_spline_wire(pts, z1 - cap), _spline_wire(pts, z1, inset=inset)]
    )
    bot = cq.Solid.makeLoft(
        [_spline_wire(pts, z0, inset=inset), _spline_wire(pts, z0 + cap)]
    )
    return cq.Workplane(obj=mid.fuse(top).fuse(bot))


def _spline_prism(pts, z0, z1):
    face = cq.Face.makeFromWires(_spline_wire(pts, z0))
    return cq.Workplane(obj=cq.Solid.extrudeLinear(face, cq.Vector(0.0, 0.0, z1 - z0)))


# ---- jaw, teeth, boss, rivet, screw ----------------------------------------

def _jaw_clip_box():
    """Box that selects the full-height jaw region forward of the boss."""
    return cq.Workplane(
        "XY", origin=(JAW_X_MIN + 0.022, 0.0, (JAW_Z0 + JAW_Z1) / 2)
    ).box(0.065, 0.060, 0.020)


def _jaw_body(s, own_z0, own_z1):
    """Thick locking-plier jaw: full-height forward, own-plate rear."""
    prof = _mirror(JAW_PTS, s)
    full = _poly_prism(prof, JAW_Z0, JAW_Z1).intersect(_jaw_clip_box())
    rear = _poly_prism(prof, own_z0, own_z1)
    return full.union(rear)


def _jaw_teeth(s, z0, z1):
    """Serrated teeth: thin block with grooves cut to form individual ridges."""
    face_y = s * GRIP_FACE
    tooth_d = 0.0012
    h = z1 - z0
    z_mid = (z0 + z1) / 2.0
    start_x = 0.012
    end_x = 0.044
    total_length = end_x - start_x

    # Base protruding block from jaw face toward centre
    cy = face_y - s * (tooth_d / 2.0)
    base = (
        cq.Workplane("XY", origin=(start_x + total_length / 2.0, cy, z_mid))
        .box(total_length, tooth_d, h)
    )

    # Cut grooves to create individual tooth ridges
    tooth_pitch = 0.003
    groove_w = 0.0015
    x = start_x + (tooth_pitch - groove_w)
    while x < end_x - (tooth_pitch - groove_w) / 2.0:
        groove = (
            cq.Workplane("XY", origin=(x + groove_w / 2.0, cy, z_mid))
            .box(groove_w, tooth_d * 1.5, h * 1.5)
        )
        base = base.cut(groove)
        x += tooth_pitch

    return base


def _half_boss(own_z0, own_z1, with_hole):
    boss = (
        cq.Workplane("XY", origin=(0.0, 0.0, own_z0))
        .circle(BOSS_R)
        .extrude(own_z1 - own_z0)
    )
    if with_hole:
        hole = (
            cq.Workplane("XY", origin=(0.0, 0.0, own_z0 - 0.001))
            .circle(HOLE_R)
            .extrude((own_z1 - own_z0) + 0.002)
        )
        boss = boss.cut(hole)
    return boss


def _rivet():
    lower_head = (
        cq.Workplane("XY", origin=(0.0, 0.0, ZL0 - 0.0015))
        .circle(HEAD_R)
        .extrude(0.0018)
    )
    shank = (
        cq.Workplane("XY", origin=(0.0, 0.0, ZL0 - 0.001))
        .circle(SHANK_R)
        .extrude((ZU1 - ZL0) + 0.002)
    )
    upper_head = (
        cq.Workplane("XY", origin=(0.0, 0.0, ZU1 + 0.0001))
        .circle(HEAD_R)
        .extrude(0.0014)
    )
    rivet = lower_head.union(shank).union(upper_head)
    try:
        rivet = rivet.edges(">Z").fillet(0.0009)
    except Exception:
        pass
    return rivet


def _adjustment_screw():
    """Rear thumb-screw: flange + shaft + head, built along +Z then rotated to -X."""
    shaft_r = 0.0022
    shaft_len = 0.014
    head_r = 0.0045
    head_len = 0.004
    flange_r = shaft_r + 0.0012
    flange_len = 0.003
    sz = (GRIP_LZ0 + GRIP_LZ1) / 2.0

    # Build along +Z at origin
    flange = cq.Workplane("XY").circle(flange_r).extrude(flange_len)
    shaft = cq.Workplane("XY").circle(shaft_r).extrude(shaft_len)
    head = (
        cq.Workplane("XY", origin=(0.0, 0.0, shaft_len))
        .circle(head_r)
        .extrude(head_len)
    )
    screw = flange.union(shaft).union(head)

    # Rotate from +Z axis to -X axis: rotate -90° about Y through origin
    screw = screw.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -90.0)
    # After rotation: flange near origin extending in -X, head far in -X.
    # Translate so flange sits at handle end.
    screw = screw.translate((SCREW_SX, SCREW_SY, sz))

    return screw


# ---- model -----------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="locking_pliers")

    steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    polished = model.material("polished_steel", rgba=(0.86, 0.87, 0.90, 1.0))
    dark_steel = model.material("tooth_steel", rgba=(0.52, 0.53, 0.56, 1.0))
    blue = model.material("blue_grip", rgba=(0.12, 0.35, 0.72, 1.0))
    yellow = model.material("yellow_accent", rgba=(0.95, 0.82, 0.10, 1.0))

    # ---- lower half (base link): jaw at +y, handle at -y ------------------
    lower = model.part("lower_half")
    lower_pose = Origin(rpy=(0.0, 0.0, HALF_OPEN))

    lower.visual(
        mesh_from_cadquery(_jaw_body(+1.0, ZL0, ZL1), "lower_jaw"),
        origin=lower_pose, material=steel, name="jaw_body",
    )
    lower.visual(
        mesh_from_cadquery(_jaw_teeth(+1.0, JAW_Z0, JAW_Z1), "lower_teeth"),
        origin=lower_pose, material=dark_steel, name="jaw_teeth",
    )
    lower.visual(
        mesh_from_cadquery(_half_boss(ZL0, ZL1, with_hole=False), "lower_boss"),
        origin=lower_pose, material=steel, name="pivot_boss",
    )
    lower.visual(
        mesh_from_cadquery(
            _poly_prism(_mirror(TANG_PTS, +1.0), ZL0, ZL1), "lower_tang"
        ),
        origin=lower_pose, material=steel, name="neck_tang",
    )
    lower.visual(
        mesh_from_cadquery(
            _spline_prism(_mirror(GRIP_PTS, +1.0), GRIP_LZ0, GRIP_LZ1), "lower_grip"
        ),
        origin=lower_pose, material=blue, name="grip",
    )
    lower.visual(
        mesh_from_cadquery(
            _spline_prism(
                _mirror(STRIPE_PTS, +1.0),
                GRIP_LZ1 - STRIPE_EMBED,
                GRIP_LZ1 - STRIPE_EMBED + STRIPE_T,
            ),
            "lower_stripe",
        ),
        origin=lower_pose, material=yellow, name="grip_stripe",
    )
    lower.visual(
        mesh_from_cadquery(_rivet(), "rivet"),
        origin=lower_pose, material=polished, name="rivet",
    )
    lower.visual(
        mesh_from_cadquery(_adjustment_screw(), "adj_screw"),
        origin=lower_pose, material=dark_steel, name="adj_screw",
    )

    # ---- upper half (moving link): mirrored, upper steel layer -------------
    upper = model.part("upper_half")

    upper.visual(
        mesh_from_cadquery(_jaw_body(-1.0, ZU0, ZU1), "upper_jaw"),
        material=steel, name="jaw_body",
    )
    upper.visual(
        mesh_from_cadquery(_jaw_teeth(-1.0, JAW_Z0, JAW_Z1), "upper_teeth"),
        material=dark_steel, name="jaw_teeth",
    )
    upper.visual(
        mesh_from_cadquery(_half_boss(ZU0, ZU1, with_hole=True), "upper_boss"),
        material=steel, name="pivot_boss",
    )
    upper.visual(
        mesh_from_cadquery(
            _poly_prism(_mirror(TANG_PTS, -1.0), ZU0, ZU1), "upper_tang"
        ),
        material=steel, name="neck_tang",
    )
    upper.visual(
        mesh_from_cadquery(
            _spline_prism(_mirror(GRIP_PTS, -1.0), GRIP_UZ0, GRIP_UZ1), "upper_grip"
        ),
        material=blue, name="grip",
    )
    upper.visual(
        mesh_from_cadquery(
            _spline_prism(
                _mirror(STRIPE_PTS, -1.0),
                GRIP_UZ1 - STRIPE_EMBED,
                GRIP_UZ1 - STRIPE_EMBED + STRIPE_T,
            ),
            "upper_stripe",
        ),
        material=yellow, name="grip_stripe",
    )

    # ---- single revolute pivot at the rivet --------------------------------
    pivot = model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -HALF_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0, velocity=4.0, lower=0.0, upper=CLOSE_TRAVEL
        ),
    )

    return model


# ---- tests -----------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_half")
    upper = object_model.get_part("upper_half")
    pivot = object_model.get_articulation("pivot")

    # --- pivot stack: bosses coaxial, upper sits above lower ----------------
    ctx.expect_overlap(
        lower, upper, axes="xy",
        elem_a="pivot_boss", elem_b="pivot_boss",
        min_overlap=0.012,
        name="pivot bosses stack coaxially",
    )
    ctx.expect_gap(
        upper, lower, axis="z",
        positive_elem="pivot_boss", negative_elem="pivot_boss",
        min_gap=-0.00002, max_gap=0.0006,
        name="upper boss sits slightly above lower boss",
    )
    ctx.expect_contact(
        lower, upper,
        elem_a="pivot_boss", elem_b="pivot_boss",
        contact_tol=0.0001,
        name="upper boss rides on the lower boss face",
    )

    # --- rivet passes through boss stack and caps it ------------------------
    ctx.expect_within(
        lower, upper, axes="xy",
        inner_elem="rivet", outer_elem="pivot_boss",
        margin=0.0002,
        name="rivet centered through the boss bore",
    )
    rivet_aabb = ctx.part_element_world_aabb(lower, elem="rivet")
    uboss_aabb = ctx.part_element_world_aabb(upper, elem="pivot_boss")
    ctx.check(
        "rivet head caps the upper boss",
        rivet_aabb is not None
        and uboss_aabb is not None
        and rivet_aabb[1][2] > uboss_aabb[1][2] + 0.0008,
        details=f"rivet={rivet_aabb}, upper_boss={uboss_aabb}",
    )

    # --- serrated teeth on both jaw halves ----------------------------------
    for pname in ("lower_half", "upper_half"):
        p = object_model.get_part(pname)
        t = ctx.part_element_world_aabb(p, elem="jaw_teeth")
        ctx.check(
            f"{pname} has serrated teeth geometry",
            t is not None,
            details="jaw_teeth visual missing",
        )

    # --- teeth protrude inward from the jaw body face -----------------------
    lt = ctx.part_element_world_aabb(lower, elem="jaw_teeth")
    lj = ctx.part_element_world_aabb(lower, elem="jaw_body")
    if lt is not None and lj is not None:
        ctx.check(
            "lower teeth protrude inward from jaw face",
            lt[0][1] < lj[1][1] - 0.0005,
            details=f"teeth_min_y={lt[0][1]:.4f}, jaw_max_y={lj[1][1]:.4f}",
        )

    ut = ctx.part_element_world_aabb(upper, elem="jaw_teeth")
    uj = ctx.part_element_world_aabb(upper, elem="jaw_body")
    if ut is not None and uj is not None:
        ctx.check(
            "upper teeth protrude inward from jaw face",
            ut[1][1] > uj[0][1] + 0.0005,
            details=f"teeth_max_y={ut[1][1]:.4f}, jaw_min_y={uj[0][1]:.4f}",
        )

    # --- teeth span the forward jaw region ----------------------------------
    for p in (lower, upper):
        t = ctx.part_element_world_aabb(p, elem="jaw_teeth")
        ctx.check(
            f"{p.name} teeth span jaw height",
            t is not None and t[0][2] <= JAW_Z0 + 0.0004 and t[1][2] >= JAW_Z1 - 0.0004,
            details=f"teeth={t}",
        )

    # --- adjustment screw at the rear of the lower handle -------------------
    screw_aabb = ctx.part_element_world_aabb(lower, elem="adj_screw")
    grip_aabb = ctx.part_element_world_aabb(lower, elem="grip")
    ctx.check(
        "adjustment screw exists on lower half",
        screw_aabb is not None,
        details="adj_screw visual missing",
    )
    if screw_aabb is not None and grip_aabb is not None:
        ctx.check(
            "screw positioned at or behind handle rear",
            screw_aabb[0][0] < grip_aabb[0][0] + 0.010,
            details=f"screw_min_x={screw_aabb[0][0]:.4f}, grip_min_x={grip_aabb[0][0]:.4f}",
        )
        ctx.check(
            "screw head protrudes beyond handle end",
            screw_aabb[0][0] < grip_aabb[0][0] - 0.005,
            details=f"screw_min_x={screw_aabb[0][0]:.4f}, grip_min_x={grip_aabb[0][0]:.4f}",
        )

    # --- rest pose: jaws open, handles splayed apart ------------------------
    ctx.expect_gap(
        lower, upper, axis="y",
        positive_elem="jaw_body", negative_elem="jaw_body",
        min_gap=0.002,
        name="jaw faces open at rest",
    )
    ctx.expect_gap(
        upper, lower, axis="y",
        positive_elem="grip", negative_elem="grip",
        min_gap=0.010,
        name="handles splay apart at rest",
    )

    # --- jaws span the full forged head height ------------------------------
    for p in (lower, upper):
        jaw = ctx.part_element_world_aabb(p, elem="jaw_body")
        ctx.check(
            f"{p.name} jaw spans both plate layers",
            jaw is not None and jaw[0][2] <= JAW_Z0 + 0.0003 and jaw[1][2] >= JAW_Z1 - 0.0003,
            details=f"jaw={jaw}",
        )

    # --- overall proportions ------------------------------------------------
    la = ctx.part_world_aabb(lower)
    ua = ctx.part_world_aabb(upper)
    if la is not None and ua is not None:
        length = max(la[1][0], ua[1][0]) - min(la[0][0], ua[0][0])
        width = max(la[1][1], ua[1][1]) - min(la[0][1], ua[0][1])
        height = max(la[1][2], ua[1][2]) - min(la[0][2], ua[0][2])
        ctx.check(
            "overall length ~0.17-0.20 m",
            0.155 <= length <= 0.210,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "splayed handle width ~0.05-0.08 m",
            0.045 <= width <= 0.090,
            details=f"width={width:.4f}",
        )
        ctx.check(
            "tool thickness ~0.014-0.020 m",
            0.012 <= height <= 0.022,
            details=f"height={height:.4f}",
        )
    else:
        ctx.fail("overall proportions", "missing part AABBs")

    # --- articulation: positive q closes jaws, handles scissor opposite -----
    limits = pivot.motion_limits
    ctx.check(
        "pivot travel is roughly 0..24 degrees",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower) < 1e-9
        and 0.36 <= limits.upper <= 0.48,
        details=f"limits={limits}",
    )

    open_jaw = ctx.part_element_world_aabb(upper, elem="jaw_body")
    open_grip = ctx.part_element_world_aabb(upper, elem="grip")
    with ctx.pose({pivot: CLOSE_TRAVEL}):
        closed_jaw = ctx.part_element_world_aabb(upper, elem="jaw_body")
        closed_grip = ctx.part_element_world_aabb(upper, elem="grip")

    ctx.check(
        "closing swings upper jaw toward lower jaw",
        open_jaw is not None
        and closed_jaw is not None
        and closed_jaw[1][1] > open_jaw[1][1] + 0.003,
        details=f"open={open_jaw}, closed={closed_jaw}",
    )
    ctx.check(
        "handles scissor opposite to the jaws",
        open_grip is not None
        and closed_grip is not None
        and closed_grip[0][1] < open_grip[0][1] - 0.003,
        details=f"open={open_grip}, closed={closed_grip}",
    )

    return ctx.report()


object_model = build_object_model()
