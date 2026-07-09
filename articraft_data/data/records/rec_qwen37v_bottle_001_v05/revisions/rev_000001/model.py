from __future__ import annotations

# Squeeze bottle with conical nozzle cap and pivoting straw spout.
# Variant of a clear plastic juice bottle. Frame: bottle axis along +Z,
# base at z=0, nozzle/spout at the top (+Z).
#
# Body: transparent thin-wall PET squeeze bottle shell — soft-taper barrel,
#   shoulder, short threaded neck, and a visible thickened transparent lip
#   ring at the mouth opening. The hollow interior is visible through the
#   clear walls and open mouth.
#
# Cap: black conical nozzle cap that rotates on the neck (CONTINUOUS joint).
#   A small nozzle tip at the top of the cone is the dispensing opening.
#
# Spout: a thin straw/spout tube that pivots from a hinge mount on the cap
#   side (REVOLUTE joint). At q=0 the spout lies folded down alongside the
#   cap; at the upper limit it pivots up to a near-vertical drinking position.

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
)

# ---- key dimensions (meters) ----
BODY_R = 0.028           # outer barrel radius (~0.056 m dia, squeeze-friendly)
WALL = 0.0015            # thin PET wall thickness
BASE_Z = 0.0             # bottom of the bottle
BARREL_TOP_Z = 0.115     # where the shoulder taper begins
SHOULDER_TOP_Z = 0.138   # top of the shoulder, base of the neck
NECK_R = 0.013           # neck outer radius (under the threads)
NECK_TOP_Z = 0.155       # top rim of the neck
LIP_HEIGHT = 0.004       # thickened mouth lip height
LIP_OUTER_R = NECK_R + 0.003  # lip outer radius (wider than neck)
LIP_BORE_R = NECK_R - 0.002   # lip inner bore (visible hollow opening)

# Cap dimensions (conical nozzle)
CAP_BASE_R = 0.017       # cap base radius (fits over neck + threads)
CAP_BASE_H = 0.008       # cylindrical base section height
CAP_CONE_TOP_R = 0.005   # nozzle tip radius
CAP_CONE_H = 0.022       # cone section height
CAP_TOTAL_H = CAP_BASE_H + CAP_CONE_H  # total cap height
NOZZLE_BORE_R = 0.002    # nozzle dispensing hole radius

# Spout dimensions
SPOUT_R = 0.0025         # spout tube outer radius
SPOUT_BORE_R = 0.0015    # spout tube inner bore
SPOUT_LENGTH = 0.050     # spout tube length
SPOUT_HINGE_OFFSET_X = CAP_BASE_R + 0.001  # hinge on cap side (+X)
SPOUT_HINGE_Z = CAP_BASE_H  # hinge height on cap (at base/cone junction)


def _neck_thread_profile():
    """Sawtooth ridge segments along the neck for thread detail."""
    pts = [(NECK_R, SHOULDER_TOP_Z)]
    z0 = SHOULDER_TOP_Z + 0.004
    ridge_r = NECK_R + 0.0014
    for k in range(3):
        zc = z0 + k * 0.0045
        pts.append((NECK_R, zc - 0.0015))
        pts.append((ridge_r, zc - 0.0005))
        pts.append((ridge_r, zc + 0.0005))
        pts.append((NECK_R, zc + 0.0015))
    pts.append((NECK_R, NECK_TOP_Z - LIP_HEIGHT))
    return pts


def _bottle_shell():
    """Transparent squeeze bottle shell with thickened mouth lip.

    One revolved solid with threads baked into the outer profile and a
    thickened lip ring at the top. The interior is hollowed open at the top
    so the mouth opening and wall thickness lip are clearly visible.
    """
    wp = (
        cq.Workplane("XZ")
        .moveTo(0.0, BASE_Z)
        # rounded base corner
        .lineTo(BODY_R - 0.005, BASE_Z)
        .threePointArc((BODY_R, BASE_Z + 0.005), (BODY_R, BASE_Z + 0.010))
        # straight cylindrical barrel with slight squeeze-bottle taper
        .lineTo(BODY_R, BARREL_TOP_Z - 0.020)
        .lineTo(BODY_R - 0.002, BARREL_TOP_Z)
        # shoulder taper up to the neck
        .threePointArc(
            ((BODY_R + NECK_R) / 2.0, (BARREL_TOP_Z + SHOULDER_TOP_Z) / 2.0 + 0.003),
            (NECK_R, SHOULDER_TOP_Z),
        )
    )
    # ridged neck
    for (r, z) in _neck_thread_profile()[1:]:
        wp = wp.lineTo(r, z)
    # mouth lip: widen out then back to form a thickened rim
    lip_base_z = NECK_TOP_Z - LIP_HEIGHT
    wp = (
        wp
        .lineTo(LIP_OUTER_R, lip_base_z)
        .lineTo(LIP_OUTER_R, NECK_TOP_Z)
        .lineTo(LIP_BORE_R, NECK_TOP_Z)
    )
    # close back along the axis
    wp = wp.lineTo(0.0, NECK_TOP_Z).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    # Hollow it: remove the top face and shell inward
    return outer.faces(">Z").shell(-WALL)


def _cap_solid():
    """Black conical nozzle cap. Local frame origin at the cap joint (base
    center). The cylindrical base sits above origin and the cone tapers up."""
    # Cylindrical base (sits on the neck)
    base = (
        cq.Workplane("XY")
        .circle(CAP_BASE_R)
        .extrude(CAP_BASE_H)
    )
    # Conical nozzle taper
    cone = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, CAP_BASE_H))
        .circle(CAP_BASE_R)
        .workplane(offset=CAP_CONE_H)
        .circle(CAP_CONE_TOP_R)
        .loft()
    )
    cap = base.union(cone)
    # Hollow bore through cap for the nozzle opening
    bore = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -0.001))
        .circle(NECK_R + 0.001)
        .extrude(CAP_BASE_H + 0.002)
    )
    cap = cap.cut(bore)
    # Nozzle tip bore (small hole at top of cone)
    nozzle_bore = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, CAP_TOTAL_H - 0.004))
        .circle(NOZZLE_BORE_R)
        .extrude(0.005)
    )
    cap = cap.cut(nozzle_bore)
    # Fillet the base bottom edge for a finished look
    cap = cap.edges("<Z").fillet(0.001)
    # Add a hinge boss on the side for the spout pivot mount
    boss = (
        cq.Workplane("XZ")
        .transformed(offset=(SPOUT_HINGE_OFFSET_X, 0, SPOUT_HINGE_Z))
        .circle(0.004)
        .extrude(0.006)  # along +Y (boss thickness)
    )
    # Center the boss on the hinge point
    boss = (
        cq.Workplane("XY")
        .transformed(offset=(SPOUT_HINGE_OFFSET_X, 0.0, SPOUT_HINGE_Z))
        .box(0.008, 0.008, 0.008)
    )
    cap = cap.union(boss)
    return cap


def _spout_tube():
    """Straw spout tube. Local frame origin at the pivot hinge point.
    The tube extends along +X from the pivot when at rest (folded down)."""
    # Outer tube
    outer = (
        cq.Workplane("YZ")
        .transformed(offset=(SPOUT_R + 0.002, 0, 0))
        .circle(SPOUT_R)
        .extrude(SPOUT_LENGTH)
    )
    # Inner bore
    bore = (
        cq.Workplane("YZ")
        .transformed(offset=(SPOUT_R + 0.001, 0, 0))
        .circle(SPOUT_BORE_R)
        .extrude(SPOUT_LENGTH + 0.002)
    )
    tube = outer.cut(bore)
    # Hinge knuckle at the pivot end (connects to cap boss)
    knuckle = (
        cq.Workplane("XY")
        .circle(0.0035)
        .extrude(0.007)
    )
    # Position knuckle centered on origin
    knuckle = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -0.0035))
        .circle(0.0035)
        .extrude(0.007)
    )
    tube = tube.union(knuckle)
    return tube


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="squeeze_bottle")

    # Materials
    clear = model.material("clear_pet", rgba=(0.82, 0.88, 0.86, 0.22))
    black = model.material("cap_black", rgba=(0.05, 0.05, 0.06, 1.0))
    grey = model.material("spout_grey", rgba=(0.35, 0.35, 0.38, 1.0))

    # ---- bottle body (root) ----
    body = model.part("bottle")
    shell = _bottle_shell()
    body.visual(
        mesh_from_cadquery(shell, "bottle_shell"),
        material=clear,
        name="bottle_shell",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.028,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- conical nozzle cap ----
    cap = model.part("cap")
    cap_geo = _cap_solid()
    cap.visual(
        mesh_from_cadquery(cap_geo, "cap_shell"),
        material=black,
        name="cap_shell",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_BASE_R, CAP_TOTAL_H),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, CAP_TOTAL_H / 2.0)),
    )

    # ---- pivoting straw spout ----
    spout = model.part("spout")
    spout_geo = _spout_tube()
    spout.visual(
        mesh_from_cadquery(spout_geo, "spout_tube"),
        material=grey,
        name="spout_tube",
    )
    spout.inertial = Inertial.from_geometry(
        Cylinder(SPOUT_R, SPOUT_LENGTH),
        mass=0.003,
        origin=Origin(xyz=(SPOUT_LENGTH / 2.0 + SPOUT_R, 0.0, 0.0)),
    )

    # ---- joints ----
    # cap_rotate: CONTINUOUS spin of the cap about the bottle +Z axis
    model.articulation(
        "cap_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    # spout_pivot: REVOLUTE pivot of the straw spout.
    # Origin at the hinge boss on the cap side. Axis along Y so positive
    # rotation takes the spout from pointing in +X (folded) toward +Z (upright).
    model.articulation(
        "spout_pivot",
        ArticulationType.REVOLUTE,
        parent=cap,
        child=spout,
        origin=Origin(xyz=(SPOUT_HINGE_OFFSET_X, 0.0, SPOUT_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=math.radians(110),
            effort=0.5,
            velocity=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle")
    cap = object_model.get_part("cap")
    spout = object_model.get_part("spout")
    cap_rotate = object_model.get_articulation("cap_rotate")
    spout_pivot = object_model.get_articulation("spout_pivot")

    bottle_shell = body.get_visual("bottle_shell")
    cap_shell = cap.get_visual("cap_shell")
    spout_tube = spout.get_visual("spout_tube")

    # --- bottle is clear (alpha < 1), cap is opaque black ---
    ctx.check(
        "bottle material is tinted-transparent (alpha < 1)",
        bottle_shell.material.rgba is not None and bottle_shell.material.rgba[3] < 1.0,
        details=f"bottle rgba={bottle_shell.material.rgba}",
    )
    ctx.check(
        "cap material is opaque black",
        cap_shell.material.rgba is not None
        and cap_shell.material.rgba[3] >= 0.99
        and max(cap_shell.material.rgba[:3]) < 0.2,
        details=f"cap rgba={cap_shell.material.rgba}",
    )

    # --- cap sits on top of the neck ---
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap mounted on top of the neck",
        cap_pos is not None and cap_pos[2] > BARREL_TOP_Z,
        details=f"cap origin={cap_pos}",
    )

    # --- spout is mounted on the cap ---
    spout_pos = ctx.part_world_position(spout)
    ctx.check(
        "spout mounted on the cap",
        spout_pos is not None and spout_pos[2] > SHOULDER_TOP_Z,
        details=f"spout origin={spout_pos}",
    )

    # Cap skirt/hollow sits over neck threads at rest -> intentional overlap
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_shell",
        elem_b="bottle_shell",
        reason="Cap base cavity is intentionally seated over the threaded neck.",
    )

    # Spout knuckle nests into the cap hinge boss -> small local overlap
    ctx.allow_overlap(
        spout,
        cap,
        elem_a="spout_tube",
        elem_b="cap_shell",
        reason="Spout hinge knuckle is intentionally nested in the cap hinge boss.",
    )

    # --- spout pivot: folded at q=0, raised at upper limit ---
    # The spout origin is at the hinge (joint origin), so it doesn't translate.
    # Check the AABB max Z to see the spout tube tip swing upward.
    rest_aabb = ctx.part_world_aabb(spout)
    with ctx.pose({spout_pivot: math.radians(110)}):
        raised_aabb = ctx.part_world_aabb(spout)

    rest_max_z = rest_aabb[1][2] if rest_aabb else 0.0
    raised_max_z = raised_aabb[1][2] if raised_aabb else 0.0
    ctx.check(
        "spout_pivot raises the spout tip upward",
        raised_max_z > rest_max_z + 0.01,
        details=f"rest_max_z={rest_max_z}, raised_max_z={raised_max_z}",
    )

    # --- spout pivot is revolute with proper limits ---
    ctx.check(
        "spout_pivot is a revolute joint",
        spout_pivot.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={spout_pivot.articulation_type}",
    )
    ctx.check(
        "spout_pivot has non-zero motion range",
        spout_pivot.motion_limits.upper > spout_pivot.motion_limits.lower + 0.1,
        details=f"lower={spout_pivot.motion_limits.lower}, upper={spout_pivot.motion_limits.upper}",
    )

    # --- cap_rotate is continuous ---
    ctx.check(
        "cap_rotate is a continuous joint",
        cap_rotate.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={cap_rotate.articulation_type}",
    )

    # --- hollow mouth: bottle shell is hollowed (has inner bore visible) ---
    body_dims = ctx.part_world_aabb(body)
    ctx.check(
        "bottle has substantial height (squeeze bottle proportions)",
        body_dims is not None and (body_dims[1][2] - body_dims[0][2]) > 0.12,
        details=f"bottle aabb={body_dims}",
    )

    return ctx.report()


object_model = build_object_model()
