from __future__ import annotations

# Tall cylindrical flip-cap bottle variant.
# Frame: bottle axis along +Z, base at z=0, neck/cap at the top (+Z).
# Body: tall slim cylindrical barrel, shoulder taper, narrow neck with hollow
#       open top (visible mouth opening when cap is open).
# Cap: flip cap on a REVOLUTE hinge at the back of the neck top.
#       The cap disc covers the mouth; a hinge strap bridges to the hinge pin,
#       a pull tab at the front, and a sealing plug on the underside.
# Hinge: axis along +X at the rear of the neck.  Positive q lifts the front
#        edge upward (opening).  Limits 0 (closed) to ~2.0 rad (~115°).

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
BODY_R = 0.030          # barrel outer radius (~60 mm dia)
WALL = 0.0016           # thin PET wall
BASE_Z = 0.0
BARREL_TOP_Z = 0.200    # where the shoulder taper begins
SHOULDER_TOP_Z = 0.218  # top of shoulder, base of the neck
NECK_R = 0.013          # neck outer radius (narrow)
NECK_TOP_Z = 0.240      # top rim of the neck (mouth)

# Cap
CAP_DISC_R = NECK_R + 0.001          # disc covers the mouth
CAP_THICKNESS = 0.004                # disc thickness
HINGE_Y_OFFSET = NECK_R + 0.003     # hinge pin behind neck centre
HINGE_LOWER = 0.0                    # closed
HINGE_UPPER = 2.0                    # ~115° open


def _bottle_shell():
    """Thin-wall PET bottle as one revolved solid, shelled open at the top."""
    wp = (
        cq.Workplane("XZ")
        .moveTo(0.0, BASE_Z)
        # rounded base corner
        .lineTo(BODY_R - 0.006, BASE_Z)
        .threePointArc((BODY_R, BASE_Z + 0.006), (BODY_R, BASE_Z + 0.012))
        # straight cylindrical barrel
        .lineTo(BODY_R, BARREL_TOP_Z)
        # shoulder taper up to the neck
        .threePointArc(
            ((BODY_R + NECK_R) / 2.0, (BARREL_TOP_Z + SHOULDER_TOP_Z) / 2.0 + 0.004),
            (NECK_R, SHOULDER_TOP_Z),
        )
        # straight narrow neck up to the mouth
        .lineTo(NECK_R, NECK_TOP_Z)
        # close back along the axis
        .lineTo(0.0, NECK_TOP_Z)
        .close()
    )
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    # Hollow it: remove the top face and shell inward -> open mouth at the top.
    return outer.faces(">Z").shell(-WALL)


def _neck_rim():
    """Small raised ring at the mouth that frames the hollow opening."""
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, NECK_TOP_Z - 0.001))
        .circle(NECK_R + 0.002)
        .extrude(0.004)
    )
    inner = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, NECK_TOP_Z - 0.002))
        .circle(NECK_R - 0.0005)   # slight embed into neck wall for connectivity
        .extrude(0.006)            # cut through the outer ring
    )
    return outer.cut(inner)


def _cap_solid():
    """Flip cap: disc + hinge strap + pull tab + sealing plug.

    Local frame: origin at the hinge pin.  At q=0 the cap is closed; the disc
    extends in +Y from the hinge to cover the mouth opening.
    """
    disc_cy = HINGE_Y_OFFSET   # disc centre Y in cap-local frame

    # --- main disc ---
    disc = (
        cq.Workplane("XY")
        .transformed(offset=(0, disc_cy, 0))
        .circle(CAP_DISC_R)
        .extrude(CAP_THICKNESS)
    )
    # Fillet the top edge of the disc (skip if geometry is fragile)
    try:
        disc = disc.edges(">Z").fillet(0.0008)
    except Exception:
        pass

    # --- hinge strap: bridges from hinge origin to disc back edge ---
    back_edge_y = disc_cy - CAP_DISC_R           # disc back edge in local Y
    strap_span = abs(back_edge_y) + 0.006        # reach past origin + overlap
    strap_cy = back_edge_y / 2.0 + 0.001         # centre so it bridges
    strap = (
        cq.Workplane("XY")
        .transformed(offset=(0, strap_cy, 0))
        .box(0.008, strap_span, CAP_THICKNESS)
    )

    cap = disc.union(strap)

    # --- front pull tab ---
    tab_y = disc_cy + CAP_DISC_R + 0.004
    tab = (
        cq.Workplane("XY")
        .transformed(offset=(0, tab_y, CAP_THICKNESS / 2.0))
        .box(0.006, 0.008, CAP_THICKNESS + 0.002)
    )
    cap = cap.union(tab)

    # --- underside sealing plug (fits into the neck bore) ---
    plug_r = NECK_R - WALL - 0.001               # clearance inside neck
    plug = (
        cq.Workplane("XY")
        .transformed(offset=(0, disc_cy, -0.006))
        .circle(plug_r)
        .extrude(0.006)
    )
    cap = cap.union(plug)

    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flip_cap_bottle")

    # Tinted-transparent clear PET and an opaque blue cap.
    clear = model.material("clear_pet", rgba=(0.80, 0.86, 0.84, 0.25))
    blue = model.material("cap_blue", rgba=(0.15, 0.35, 0.65, 1.0))

    # ---- bottle body (root) ----
    body = model.part("bottle")
    shell = _bottle_shell()
    body.visual(
        mesh_from_cadquery(shell, "bottle_shell"),
        material=clear, name="bottle_shell",
    )

    # Raised rim ring at the mouth – frames the hollow opening.
    rim = _neck_rim()
    body.visual(
        mesh_from_cadquery(rim, "neck_rim"),
        material=clear, name="neck_rim",
    )

    # Hinge-mount ear on the back of the neck (visual hinge anchor).
    body.visual(
        Box((0.010, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, -(NECK_R + 0.001), NECK_TOP_Z - 0.004)),
        material=clear, name="hinge_mount",
    )

    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.032,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- flip cap ----
    cap = model.part("cap")
    cap_geo = _cap_solid()
    cap.visual(
        mesh_from_cadquery(cap_geo, "cap_shell"),
        material=blue, name="cap_shell",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_DISC_R, CAP_THICKNESS),
        mass=0.008,
        origin=Origin(xyz=(0.0, HINGE_Y_OFFSET, CAP_THICKNESS / 2.0)),
    )

    # ---- revolute hinge ----
    model.articulation(
        "cap_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, -HINGE_Y_OFFSET, NECK_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0,
            lower=HINGE_LOWER, upper=HINGE_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle")
    cap = object_model.get_part("cap")
    hinge = object_model.get_articulation("cap_hinge")

    bottle_shell = body.get_visual("bottle_shell")
    cap_shell = cap.get_visual("cap_shell")

    # --- bottle is clear transparent PET ---
    ctx.check(
        "bottle material is tinted-transparent (alpha < 1)",
        bottle_shell.material.rgba is not None and bottle_shell.material.rgba[3] < 1.0,
        details=f"bottle rgba={bottle_shell.material.rgba}",
    )

    # --- cap is opaque coloured ---
    ctx.check(
        "cap material is opaque",
        cap_shell.material.rgba is not None and cap_shell.material.rgba[3] >= 0.99,
        details=f"cap rgba={cap_shell.material.rgba}",
    )

    # --- hinge is REVOLUTE with finite limits ---
    ctx.check(
        "cap_hinge is revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={hinge.articulation_type}",
    )
    limits = hinge.motion_limits
    ctx.check(
        "cap_hinge has bounded limits (lower < upper)",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.upper > limits.lower,
        details=f"limits lower={getattr(limits, 'lower', None)}, upper={getattr(limits, 'upper', None)}",
    )

    # --- cap sits at the neck top at rest ---
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap mounted at neck top",
        cap_pos is not None and cap_pos[2] > SHOULDER_TOP_Z,
        details=f"cap origin z={cap_pos}",
    )

    # --- intentional overlaps ---
    ctx.allow_overlap(
        cap, body,
        elem_a="cap_shell", elem_b="bottle_shell",
        reason="Cap hinge strap wraps around the neck back for the hinge connection, "
               "and the sealing plug inserts into the neck bore.",
    )
    ctx.allow_overlap(
        cap, body,
        elem_a="cap_shell", elem_b="neck_rim",
        reason="Cap disc seats against the neck rim ring when closed.",
    )
    ctx.allow_overlap(
        cap, body,
        elem_a="cap_shell", elem_b="hinge_mount",
        reason="Cap hinge strap wraps around the hinge mount at the back of the neck.",
    )

    # --- hinge opens the cap: front edge lifts upward ---
    # Use the AABB z-max to track the highest point on the cap (the front edge
    # lifts while the hinge origin stays fixed).
    rest_aabb = ctx.part_world_aabb(cap)
    rest_zmax = rest_aabb[1][2] if rest_aabb is not None else 0.0
    with ctx.pose({hinge: 1.0}):
        open_aabb = ctx.part_world_aabb(cap)
        open_zmax = open_aabb[1][2] if open_aabb is not None else 0.0
    ctx.check(
        "hinge opens cap upward (cap top rises)",
        open_zmax > rest_zmax + 0.005,
        details=f"rest_zmax={rest_zmax:.4f}, open_zmax(1 rad)={open_zmax:.4f}",
    )

    # --- fully open: cap clears the mouth so opening is visible ---
    with ctx.pose({hinge: HINGE_UPPER}):
        full_aabb = ctx.part_world_aabb(cap)
        full_zmax = full_aabb[1][2] if full_aabb is not None else 0.0
    ctx.check(
        "fully open cap is well above the neck top",
        full_zmax > NECK_TOP_Z + 0.010,
        details=f"full_open_zmax={full_zmax:.4f}, neck_top={NECK_TOP_Z}",
    )

    # --- bottle is tall and cylindrical (height > 3.5x width) ---
    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is not None:
        mn, mx = body_aabb
        height = mx[2] - mn[2]
        width = max(mx[0] - mn[0], mx[1] - mn[1])
        ctx.check(
            "bottle is tall and cylindrical (height > 3.5 × width)",
            height > 3.5 * width,
            details=f"height={height:.4f}, width={width:.4f}",
        )

    # --- neck is narrower than body ---
    ctx.check(
        "neck is narrower than body",
        NECK_R < BODY_R * 0.6,
        details=f"neck_r={NECK_R}, body_r={BODY_R}",
    )

    # --- neck_rim visual exists (frames mouth opening) ---
    neck_rim_vis = body.get_visual("neck_rim")
    ctx.check(
        "neck_rim visual exists",
        neck_rim_vis is not None,
        details="neck_rim visual not found on bottle part",
    )

    return ctx.report()


object_model = build_object_model()
