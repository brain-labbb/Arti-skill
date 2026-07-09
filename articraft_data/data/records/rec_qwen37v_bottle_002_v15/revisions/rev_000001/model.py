from __future__ import annotations

# Squeeze bottle with a conical nozzle cap.
# Frame: vertical axis along +Z, bottle standing on z=0, centerline on x=y=0.
#   - rounded base -> cylindrical squeezable body -> sharp tapered shoulder
#     -> short threaded neck with visible hollow mouth opening -> conical nozzle cap.
# Articulations (two INDEPENDENT, decoupled joints sharing +Z, via a massless carrier):
#   - cap_rotate:  CONTINUOUS spin of the nozzle cap about +Z.
#   - cap_slide:   PRISMATIC lift of the nozzle cap straight up off the neck.

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
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key heights (m) along +Z ----
BASE_Z = 0.0
BODY_TOP_Z = 0.105  # end of straight cylindrical body, start of shoulder
SHOULDER_TOP_Z = 0.145  # end of tapered shoulder, base of neck
NECK_TOP_Z = 0.165  # top of threaded neck (cap mounts here)

BODY_R = 0.032  # body radius (~0.064 m dia, wider squeeze bottle)
NECK_R = 0.011  # outer thread/neck radius
NECK_BORE_R = 0.008  # neck inner bore (visible hollow mouth opening)

# Conical nozzle cap dimensions
CAP_BASE_R = 0.013  # cap base ring outer radius (slightly larger than neck)
CAP_BASE_H = 0.008  # threaded base ring height
NOZZLE_BASE_R = 0.012  # nozzle cone base radius (at top of base ring)
NOZZLE_TIP_R = 0.003  # nozzle tip radius (small opening)
NOZZLE_H = 0.028  # conical section height
CAP_TOTAL_H = CAP_BASE_H + NOZZLE_H  # total cap height

# Mount so closed cap base ring screws fully down over the threaded neck.
CAP_MOUNT_Z = NECK_TOP_Z - CAP_BASE_H


def _profile_sections():
    # (z, radius) of the outer wall, base -> body -> tapered shoulder -> neck.
    # Squeeze bottle: wider body, sharper shoulder taper than a regular bottle.
    return [
        (0.000, 0.016),  # rounded base bottom (tucked-in heel)
        (0.005, 0.026),
        (0.012, 0.031),
        (0.030, BODY_R),  # full body radius reached
        (BODY_TOP_Z, BODY_R),  # straight cylindrical body (squeezable section)
        (0.115, 0.030),  # shoulder starts tapering sharply
        (0.128, 0.022),
        (0.138, 0.015),
        (SHOULDER_TOP_Z, 0.013),  # sharp taper to neck
        (0.148, NECK_R),  # base of neck
        (NECK_TOP_Z, NECK_R),  # straight threaded neck up to the rim
    ]


def _bottle_solid() -> cq.Workplane:
    # Revolve the outer profile, then shell it open at the top so the bottle is
    # a real thin-walled hollow container with a visible hollow mouth opening
    # at the neck rim.
    pts = _profile_sections()
    wp = cq.Workplane("XZ").moveTo(0.0, pts[0][0])
    for r, z in [(r, z) for (z, r) in pts]:
        wp = wp.lineTo(r, z)
    # close the profile back down the axis
    wp = wp.lineTo(0.0, pts[-1][0]).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))

    # Hollow it out: cut an inner cavity that opens through the neck rim.
    # The visible hollow mouth opening is the bore at the top of the neck.
    wall = 0.0015
    inner_pts = [
        (0.010, 0.005),
        (0.024, 0.011),
        (BODY_R - wall, 0.014),
        (BODY_R - wall, 0.030),
        (BODY_R - wall, BODY_TOP_Z),
        (0.029, 0.115),
        (0.021, 0.128),
        (0.014, 0.138),
        (NECK_BORE_R + 0.001, SHOULDER_TOP_Z),
        (NECK_BORE_R, 0.148),
        (NECK_BORE_R, NECK_TOP_Z + 0.005),  # open through the rim = visible mouth
    ]
    iwp = cq.Workplane("XZ").moveTo(0.0, inner_pts[0][1])
    for r, z in inner_pts:
        iwp = iwp.lineTo(r, z)
    iwp = iwp.lineTo(0.0, inner_pts[-1][1]).close()
    cavity = iwp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    return outer.cut(cavity)


def _bottle_mesh():
    return mesh_from_cadquery(_bottle_solid(), "bottle_shell")


def _neck_threads():
    # Helical-ish thread rings on the neck as torus rings.
    g = None
    for zt in (0.150, 0.156):
        ring = TorusGeometry(NECK_R - 0.0005, 0.0010, radial_segments=10, tubular_segments=40)
        ring.translate(0.0, 0.0, zt)
        if g is None:
            g = ring
        else:
            g.merge(ring)
    return mesh_from_geometry(g, "neck_threads")


def _mouth_rim():
    # A visible rim ring at the top of the neck to highlight the hollow mouth opening.
    rim = TorusGeometry(NECK_BORE_R + 0.001, 0.0012, radial_segments=10, tubular_segments=40)
    rim.translate(0.0, 0.0, NECK_TOP_Z)
    return mesh_from_geometry(rim, "mouth_rim")


def _nozzle_cap_solid() -> cq.Workplane:
    # Conical nozzle cap: threaded base ring + conical nozzle tapering to a small tip.
    # Base ring (cylinder with bore for threading onto neck)
    base = (
        cq.Workplane("XY")
        .circle(CAP_BASE_R)
        .extrude(CAP_BASE_H)
    )
    # Hollow bore in the base so it screws over the neck
    bore = (
        cq.Workplane("XY")
        .circle(NECK_R)
        .extrude(CAP_BASE_H - 0.001)
    )
    base = base.cut(bore)

    # Conical nozzle section on top of the base ring
    # Build as a revolved profile: wide at base, narrow at tip
    nozzle_profile = (
        cq.Workplane("XZ")
        .moveTo(0.0, CAP_BASE_H)
        .lineTo(NOZZLE_BASE_R, CAP_BASE_H)
        .lineTo(NOZZLE_TIP_R, CAP_BASE_H + NOZZLE_H)
        .lineTo(0.0, CAP_BASE_H + NOZZLE_H)
        .close()
    )
    nozzle = nozzle_profile.revolve(360.0, (0, 0, 0), (0, 1, 0))
    cap = base.union(nozzle)

    # Small bore through the nozzle tip (the dispensing opening)
    tip_bore = (
        cq.Workplane("XY")
        .workplane(offset=CAP_BASE_H + NOZZLE_H - 0.006)
        .circle(NOZZLE_TIP_R * 0.5)
        .extrude(0.006)
    )
    cap = cap.cut(tip_bore)

    return cap


def _nozzle_cap_mesh():
    return mesh_from_cadquery(_nozzle_cap_solid(), "nozzle_cap_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="squeeze_bottle")

    # Semi-transparent squeeze bottle body (HDPE-ish white/translucent)
    body_mat = model.material("squeeze_hdpe", rgba=(0.92, 0.93, 0.91, 0.40))
    neck_mat = model.material("neck_hdpe", rgba=(0.88, 0.90, 0.88, 0.50))
    rim_mat = model.material("mouth_rim", rgba=(0.85, 0.87, 0.85, 0.70))
    # White conical nozzle cap (like condiment bottles)
    cap_mat = model.material("cap_white", rgba=(0.95, 0.95, 0.96, 1.0))
    marker = model.material("cap_marker", rgba=(0.85, 0.15, 0.12, 1.0))

    # ---- bottle body (root): translucent hollow squeeze bottle shell ----
    body = model.part("bottle_body")
    body.visual(_bottle_mesh(), material=body_mat, name="bottle_shell")
    body.visual(_neck_threads(), material=neck_mat, name="neck_threads")
    body.visual(_mouth_rim(), material=rim_mat, name="mouth_rim")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- massless carrier: decouples spin (parent joint) from lift (child joint) ----
    carrier = model.part("cap_carrier")  # NO visuals (massless)
    carrier.inertial = Inertial.from_geometry(Box((0.006, 0.006, 0.006)), mass=1e-4)

    # ---- white conical nozzle cap ----
    cap = model.part("nozzle_cap")
    cap.visual(_nozzle_cap_mesh(), material=cap_mat, name="nozzle_cap_shell")
    # Small off-axis marker so the spin is legible
    cap.visual(
        Cylinder(0.0015, 0.003),
        origin=Origin(xyz=(CAP_BASE_R - 0.003, 0.0, CAP_BASE_H * 0.5)),
        material=marker,
        name="cap_marker",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_BASE_R, CAP_TOTAL_H),
        mass=0.004,
        origin=Origin(xyz=(0.0, 0.0, CAP_TOTAL_H / 2.0)),
    )

    # cap_rotate: CONTINUOUS spin about +Z.
    model.articulation(
        "cap_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, CAP_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )
    # cap_slide: PRISMATIC lift along +Z, carrier -> cap (independent of spin).
    model.articulation(
        "cap_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=CAP_BASE_H + 0.015, effort=1.0, velocity=1.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle_body")
    cap = object_model.get_part("nozzle_cap")
    rotate = object_model.get_articulation("cap_rotate")
    slide = object_model.get_articulation("cap_slide")

    # --- bottle body is translucent (alpha < 1) ---
    body_mat = next(m for m in object_model.materials if m.name == "squeeze_hdpe")
    a = body_mat.rgba[3] if body_mat.rgba is not None else 1.0
    ctx.check(
        "bottle shell is translucent",
        a < 1.0,
        details=f"squeeze_hdpe alpha={a}",
    )

    # --- conical nozzle cap geometry: wider at base, narrower at top ---
    cap_aabb = ctx.part_world_aabb(cap)
    cap_ext = _ext(cap_aabb)
    ctx.check(
        "nozzle cap is taller than wide (conical shape)",
        cap_ext[2] > cap_ext[0] * 1.2,
        details=f"cap extents={cap_ext}",
    )

    # --- cap seated over the neck at rest (intentional screw-over capture) ---
    ctx.allow_overlap(
        cap,
        body,
        elem_a="nozzle_cap_shell",
        elem_b="bottle_shell",
        reason="The nozzle cap base ring intentionally screws down over the threaded neck.",
    )
    ctx.allow_overlap(
        cap,
        body,
        elem_a="nozzle_cap_shell",
        elem_b="neck_threads",
        reason="The nozzle cap base ring covers the neck threads when closed.",
    )
    ctx.allow_overlap(
        cap,
        body,
        elem_a="nozzle_cap_shell",
        elem_b="mouth_rim",
        reason="The nozzle cap base ring sits over the mouth rim when closed.",
    )
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "nozzle cap mounted at the top of the bottle",
        cap_pos is not None and cap_pos[2] > 0.14,
        details=f"cap origin={cap_pos}",
    )

    # --- hollow mouth opening exists: mouth_rim visual is at the neck top ---
    mouth_rim_vis = body.get_visual("mouth_rim")
    ctx.check(
        "visible mouth rim exists on bottle body",
        mouth_rim_vis is not None,
        details="mouth_rim visual not found",
    )

    # --- cap spins about +Z: the off-axis marker swings around ---
    cap_marker = cap.get_visual("cap_marker")

    def _marker_xy():
        mn, mx = ctx.part_element_world_aabb(cap, elem=cap_marker)
        return ((mn[0] + mx[0]) / 2.0, (mn[1] + mx[1]) / 2.0)

    mk0 = _marker_xy()
    with ctx.pose({rotate: math.pi / 2.0}):
        mk90 = _marker_xy()
    moved = math.hypot(mk90[0] - mk0[0], mk90[1] - mk0[1])
    ctx.check(
        "nozzle cap spins about +Z (marker swings around)",
        moved > 0.004,
        details=f"marker rest={mk0}, quarter-turn={mk90}, moved={moved:.4f}",
    )

    # --- cap slides straight up off the neck, exposing the hollow mouth ---
    z_rest = ctx.part_world_aabb(cap)[0][2]
    with ctx.pose({slide: CAP_BASE_H + 0.015}):
        z_up = ctx.part_world_aabb(cap)[0][2]
    ctx.check(
        "nozzle cap slides up off the neck",
        z_up > z_rest + 0.012,
        details=f"cap bottom z rest={z_rest:.4f}, lifted={z_up:.4f}",
    )

    # --- when cap is lifted, mouth opening is exposed (gap between cap bottom and neck top) ---
    with ctx.pose({slide: CAP_BASE_H + 0.015}):
        cap_bottom_z = ctx.part_world_aabb(cap)[0][2]
    ctx.check(
        "mouth opening exposed when cap lifted",
        cap_bottom_z > NECK_TOP_Z + 0.005,
        details=f"cap_bottom_z={cap_bottom_z:.4f}, neck_top_z={NECK_TOP_Z}",
    )

    # --- squeeze bottle proportions: body wider than neck, taller than wide ---
    full = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "bottle is tall (taller than wide)",
        full[2] > 2.5 * full[0],
        details=f"body extents={full}",
    )
    ctx.check(
        "squeeze bottle has wide body relative to neck",
        BODY_R > NECK_R * 2.5,
        details=f"body_r={BODY_R}, neck_r={NECK_R}",
    )

    # --- continuous joint exists and is non-fixed ---
    ctx.check(
        "cap_rotate is a continuous joint",
        rotate.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={rotate.articulation_type}",
    )

    # --- prismatic joint has non-zero range ---
    ctx.check(
        "cap_slide has non-zero travel range",
        slide.motion_limits.upper > slide.motion_limits.lower + 0.005,
        details=f"lower={slide.motion_limits.lower}, upper={slide.motion_limits.upper}",
    )

    return ctx.report()


object_model = build_object_model()
