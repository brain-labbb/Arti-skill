from __future__ import annotations

# Waisted (hourglass) round lipstick tube — fork of the slim cylinder parent.
# Frame: tube axis along +Z, base seated on z=0, cap on top.
# Construction:
#   - tube_base (root): waisted lathe-revolved hollow body with silver center band.
#   - cap: glossy white cap that PULLS STRAIGHT UP off the base (PRISMATIC).
#   - bullet_carrier: massless screw carrier; CONTINUOUS rotate about +Z.
#   - bullet: red lipstick bullet on a silver carrier cup; PRISMATIC rise about +Z.
# The twist-up screw chain (continuous rotate + prismatic rise) is decoupled and
# shares the vertical +Z axis with the cap pull-off joint, but the two are independent.
#
# CHANGE vs parent: the body cross-section is now a swept lathe profile that pinches
# inward at the mid parting line (waist) and flares toward the base and top, instead
# of a constant-radius cylinder. All mechanism parts remain identical.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    mesh_from_cadquery,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
)

# --- key dimensions (meters) ---
TUBE_R_TOP = 0.0105       # outer radius at the top rim (matches parent TUBE_R)
TUBE_R_BASE = 0.0125      # wider base flare
TUBE_R_WAIST = 0.0100     # narrow waist at mid parting line
TUBE_WALL = 0.0016
TUBE_BOTTOM_Z = 0.0
BASE_TOP_Z = 0.050        # top rim of the body
BAND_Z = 0.0455           # center silver band height (near top)
INNER_R = TUBE_R_TOP - TUBE_WALL  # bore radius (must fit inside waist: 0.0089 < 0.0100)

CAP_R = TUBE_R_TOP + 0.0012   # cap slips over the body top, slightly larger
CAP_LEN = 0.046               # cap height
CAP_REST_Z = BAND_Z           # cap bottom rim sits at the silver band when closed

BULLET_R = 0.0072         # red bullet radius (a bit narrower than the bore)
CUP_R = INNER_R - 0.0006  # silver carrier cup, slides inside the bore
CUP_LEN = 0.012
BULLET_LEN = 0.020        # straight portion of bullet below the angled tip
RISE_HEIGHT = 0.040       # twist-up travel

# Profile control heights for the waisted body
_Z_BASE_FLAT = 0.004      # short flat base section / bore floor
_Z_WAIST = 0.025          # waist center height


def _waisted_body_mesh() -> LatheGeometry:
    """Hollow waisted body mesh built via SDK LatheGeometry.

    The full cross-section profile (outer wall + bore + floor) is traced as one
    closed loop of (radius, z) points and revolved to produce a single connected
    hollow mesh with no boolean-cut artifacts.
    """
    # Outer-wall control points (radius, z) — smooth hourglass curve.
    outer_ctrl = [
        (TUBE_R_BASE, TUBE_BOTTOM_Z),      # 0.0125, 0.000 — base bottom edge
        (TUBE_R_BASE, _Z_BASE_FLAT),       # 0.0125, 0.004 — base wall top
        (0.0122, 0.008),                    # begin narrowing
        (0.0115, 0.014),
        (TUBE_R_WAIST, 0.020),              # 0.0100, 0.020 — approach waist
        (TUBE_R_WAIST, _Z_WAIST),           # 0.0100, 0.025 — waist center
        (0.0102, 0.030),                    # start flaring
        (0.0106, 0.036),
        (0.0110, 0.040),
        (TUBE_R_TOP, 0.044),               # 0.0105, 0.044 — back to top radius
        (TUBE_R_TOP, BASE_TOP_Z),          # 0.0105, 0.050 — top rim
    ]
    # Smooth the outer wall with a Catmull-Rom spline for a continuous curve.
    outer_smooth = sample_catmull_rom_spline_2d(
        outer_ctrl, samples_per_segment=8, closed=False,
    )

    # Full cross-section: axis → outer wall up → top rim → bore → floor → axis.
    profile: list[tuple[float, float]] = []
    profile.append((0.0, TUBE_BOTTOM_Z))           # start at axis bottom
    profile.extend(outer_smooth)                    # outer wall (smoothed)
    profile.append((INNER_R, BASE_TOP_Z))           # step inward to bore at top
    profile.append((INNER_R, _Z_BASE_FLAT))         # bore wall down to floor
    profile.append((0.0, _Z_BASE_FLAT))             # bore floor to axis
    # close() connects (0, 0.004) back to (0, 0.0) along the axis

    return LatheGeometry(profile, segments=48, closed=True)


def _waisted_cap_mesh() -> LatheGeometry:
    """Cap shell mesh with a subtle taper, built via SDK LatheGeometry.

    Full cross-section traces outer wall → dome top → inner bore wall → close,
    producing one connected hollow mesh.
    """
    cap_r_bottom = CAP_R                        # matches body top + clearance
    cap_r_mid = CAP_R - 0.0006                  # subtle inward taper
    cap_bore_r = CAP_R - 0.0014                 # inner bore radius
    cap_top_z = CAP_LEN                         # total cap height
    cap_top_inner_z = cap_top_z - 0.004         # bore stops 4mm below top

    # Outer wall control points for spline smoothing
    outer_ctrl = [
        (cap_r_bottom, 0.0),
        (cap_r_bottom - 0.0001, cap_top_z * 0.25),
        (cap_r_bottom - 0.0003, cap_top_z * 0.50),
        (cap_r_mid, cap_top_z * 0.85),
        (cap_r_mid, cap_top_z - 0.003),
        (0.004, cap_top_z),                     # dome peak
    ]
    outer_smooth = sample_catmull_rom_spline_2d(
        outer_ctrl, samples_per_segment=8, closed=False,
    )

    # Full cross-section: outer bottom → outer wall up → dome → inner → bore → close
    profile: list[tuple[float, float]] = []
    profile.extend(outer_smooth)                        # outer wall + dome
    profile.append((0.004, cap_top_inner_z))            # inner dome surface
    profile.append((cap_bore_r, cap_top_inner_z))       # bore ceiling
    profile.append((cap_bore_r, 0.0))                   # bore wall down
    # closed=True connects (cap_bore_r, 0) back to (cap_r_bottom, 0) via flat bottom

    return LatheGeometry(profile, segments=48, closed=True)


def _bullet_cup_solid() -> cq.Workplane:
    # Silver carrier cup: short slightly-narrower cylinder that rides in the bore.
    cup = (
        cq.Workplane("XY")
        .circle(CUP_R)
        .extrude(CUP_LEN)
    )
    return cup


def _bullet_red_solid() -> cq.Workplane:
    # Red lipstick column with an angled (slanted) cut tip, sitting on the cup.
    col_bottom = CUP_LEN
    column = (
        cq.Workplane("XY")
        .workplane(offset=col_bottom)
        .circle(BULLET_R)
        .extrude(BULLET_LEN)
    )
    # Tip stock above the column.
    tip = (
        cq.Workplane("XY")
        .workplane(offset=col_bottom + BULLET_LEN)
        .circle(BULLET_R)
        .extrude(0.010)
    )
    # Cut the tip with a slanted plane to get the classic angled lipstick nose.
    cutter = (
        cq.Workplane("XY")
        .workplane(offset=col_bottom + BULLET_LEN)
        .transformed(rotate=(20.0, 0.0, 0.0))
        .rect(0.05, 0.05)
        .extrude(0.03)
        .translate((0.0, 0.0, 0.004))
    )
    tip = tip.cut(cutter)
    return column.union(tip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="waisted_lipstick_tube")

    white = model.material("tube_white", rgba=(0.93, 0.93, 0.94, 1.0))
    silver = model.material("silver", rgba=(0.74, 0.75, 0.78, 1.0))
    red = model.material("lipstick_red", rgba=(0.86, 0.10, 0.10, 1.0))
    dark = model.material("marker_dark", rgba=(0.15, 0.15, 0.16, 1.0))

    # ---- tube_base (root): waisted hollow body + silver center band ----
    base = model.part("tube_base")
    base.visual(
        mesh_from_geometry(_waisted_body_mesh(), "tube_body"),
        material=white,
        name="tube_body",
    )
    # Silver center band wrapping the tube near the parting line (at top where r=TUBE_R_TOP).
    base.visual(
        Cylinder(radius=TUBE_R_TOP + 0.0006, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, BAND_Z)),
        material=silver,
        name="center_band",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=TUBE_R_BASE, length=BASE_TOP_Z),
        mass=0.030,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z / 2.0)),
    )

    # ---- cap: pulls straight UP off the base (PRISMATIC, no thread) ----
    cap = model.part("cap")
    cap.visual(
        mesh_from_geometry(_waisted_cap_mesh(), "cap_shell"),
        material=white,
        name="cap_shell",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=CAP_R, length=CAP_LEN),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.0, CAP_LEN / 2.0)),
    )
    model.articulation(
        "cap_pull",
        ArticulationType.PRISMATIC,
        parent=base,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, CAP_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.2, lower=0.0, upper=0.050),
    )

    # ---- bullet_carrier: massless screw carrier, CONTINUOUS rotate about +Z ----
    carrier = model.part("bullet_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.004, 0.004, 0.004)), mass=1e-4)
    model.articulation(
        "bullet_twist",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),  # bottom of the inner bore
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    # ---- bullet: red lipstick on a silver carrier cup, PRISMATIC rise about +Z ----
    bullet = model.part("bullet")
    bullet.visual(
        mesh_from_cadquery(_bullet_cup_solid(), "carrier_cup"),
        material=silver,
        name="carrier_cup",
    )
    bullet.visual(
        mesh_from_cadquery(_bullet_red_solid(), "bullet_red"),
        material=red,
        name="bullet_red",
    )
    # Small off-axis marker on the cup so a twist is detectable.
    bullet.visual(
        Box((0.0016, 0.0016, 0.004)),
        origin=Origin(xyz=(CUP_R - 0.0006, 0.0, CUP_LEN / 2.0)),
        material=dark,
        name="twist_marker",
    )
    bullet.inertial = Inertial.from_geometry(
        Cylinder(radius=BULLET_R, length=CUP_LEN + BULLET_LEN),
        mass=0.004,
        origin=Origin(xyz=(0.0, 0.0, (CUP_LEN + BULLET_LEN) / 2.0)),
    )
    model.articulation(
        "bullet_rise",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=bullet,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=0.0, upper=RISE_HEIGHT),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests():
    from sdk import TestContext

    ctx = TestContext(object_model)

    base = object_model.get_part("tube_base")
    cap = object_model.get_part("cap")
    carrier = object_model.get_part("bullet_carrier")
    bullet = object_model.get_part("bullet")
    cap_pull = object_model.get_articulation("cap_pull")
    twist = object_model.get_articulation("bullet_twist")
    rise = object_model.get_articulation("bullet_rise")

    # --- material/identity claims ---
    bullet_red = bullet.get_visual("bullet_red")
    tube_body = base.get_visual("tube_body")
    band = base.get_visual("center_band")
    ctx.check(
        "bullet is red",
        tuple(bullet_red.material.rgba[:3]) == (0.86, 0.10, 0.10),
        details=f"bullet rgba={bullet_red.material.rgba}",
    )
    ctx.check(
        "tube body is white/silver (light)",
        min(tube_body.material.rgba[:3]) > 0.7,
        details=f"tube rgba={tube_body.material.rgba}",
    )
    ctx.check(
        "center band is silver",
        abs(band.material.rgba[0] - 0.74) < 0.05,
        details=f"band rgba={band.material.rgba}",
    )

    # --- waisted body profile: body is wider at base than at waist ---
    body_aabb = ctx.part_element_world_aabb(base, elem="tube_body")
    body_dx = body_aabb[1][0] - body_aabb[0][0]  # full X extent = 2 * max_radius
    # The AABB reflects the widest part (base flare). Verify it's wider than the top.
    ctx.check(
        "body base is wider than a straight TUBE_R_TOP cylinder",
        body_dx > 2.0 * TUBE_R_TOP + 0.001,
        details=f"body X extent={body_dx:.6f}, 2*TUBE_R_TOP={2*TUBE_R_TOP:.6f}",
    )

    # --- cap slips over the base; justify the seated overlaps ---
    ctx.allow_overlap(
        cap,
        base,
        elem_a="cap_shell",
        elem_b="tube_body",
        reason="The cap is intentionally slipped over the tube base top (seated pull-off cap).",
    )
    ctx.allow_overlap(
        cap,
        base,
        elem_a="cap_shell",
        elem_b="center_band",
        reason="The cap rim seats down onto the silver center band when closed.",
    )
    # Bullet/carrier cup intentionally starts down inside the tube bore.
    ctx.allow_overlap(
        bullet,
        base,
        elem_a="carrier_cup",
        elem_b="tube_body",
        reason="The carrier cup intentionally rides inside the tube bore (twist-up mechanism).",
    )
    ctx.allow_overlap(
        bullet,
        base,
        elem_a="bullet_red",
        elem_b="tube_body",
        reason="The red bullet starts nested inside the tube before being twisted up.",
    )
    # Cap surrounds the bullet when closed.
    ctx.allow_overlap(
        cap,
        bullet,
        reason="The closed cap intentionally encloses the retracted bullet.",
    )

    # cap sits on top of the base in the rest pose
    cap_rest = ctx.part_world_position(cap)
    ctx.check(
        "cap is on top of the tube",
        cap_rest is not None and cap_rest[2] > BAND_Z - 0.001,
        details=f"cap origin={cap_rest}",
    )

    # --- cap pulls straight UP off the base ---
    with ctx.pose({cap_pull: 0.045}):
        cap_up = ctx.part_world_position(cap)
        cap_bottom = ctx.part_world_aabb(cap)[0][2]
    ctx.check(
        "cap pulls straight up off the tube",
        cap_up is not None and cap_up[2] > cap_rest[2] + 0.040,
        details=f"rest={cap_rest}, pulled={cap_up}",
    )
    ctx.check(
        "pulled cap lifts toward the base top",
        cap_bottom > BASE_TOP_Z - 0.006,
        details=f"cap bottom when pulled={cap_bottom}",
    )

    # --- twisting bullet_twist spins the bullet (off-axis marker moves) ---
    marker0 = ctx.part_element_world_aabb(bullet, elem="twist_marker")
    mc0 = ((marker0[0][0] + marker0[1][0]) / 2.0, (marker0[0][1] + marker0[1][1]) / 2.0)
    with ctx.pose({twist: math.pi / 2.0}):
        marker1 = ctx.part_element_world_aabb(bullet, elem="twist_marker")
        mc1 = ((marker1[0][0] + marker1[1][0]) / 2.0, (marker1[0][1] + marker1[1][1]) / 2.0)
    moved = math.hypot(mc1[0] - mc0[0], mc1[1] - mc0[1])
    ctx.check(
        "twist spins the bullet (marker moves)",
        moved > 0.003,
        details=f"marker rest={mc0}, twisted={mc1}, moved={moved}",
    )

    # --- bullet_rise raises the bullet up out of the tube ---
    bull_rest = ctx.part_world_position(bullet)
    with ctx.pose({rise: RISE_HEIGHT}):
        bull_top_aabb = ctx.part_world_aabb(bullet)
        bull_up = ctx.part_world_position(bullet)
        bull_top_z = bull_top_aabb[1][2]
    ctx.check(
        "twist-up raises the bullet",
        bull_up is not None and bull_up[2] > bull_rest[2] + 0.030,
        details=f"rest={bull_rest}, raised={bull_up}",
    )
    ctx.check(
        "raised bullet emerges above the tube top",
        bull_top_z > BASE_TOP_Z + 0.005,
        details=f"bullet top z when raised={bull_top_z}",
    )

    return ctx.report()


object_model = build_object_model()
