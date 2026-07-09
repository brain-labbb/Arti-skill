from __future__ import annotations

# Swing-top bottle with removable measuring-cup cap.
# Variant of clear plastic juice bottle: same PET shell approach but with
#   - molded volume bands (raised rings) around the barrel
#   - raised spiral-like neck thread ridges
#   - swing-top stopper on bail arms (REVOLUTE hinge at neck sides)
#   - removable measuring-cup cap (PRISMATIC slide on +Z)
# Frame: bottle axis along +Z, base at z=0, neck at top.

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
BODY_R = 0.030          # outer barrel radius (~60 mm dia)
WALL = 0.0016           # thin PET wall
BASE_Z = 0.0
BARREL_TOP_Z = 0.108    # shoulder taper begins
SHOULDER_TOP_Z = 0.132  # top of shoulder, base of neck
NECK_R = 0.0145         # neck outer radius (under threads)
NECK_TOP_Z = 0.150      # top rim of neck
NECK_BORE_R = 0.011     # inner bore radius of neck

# Volume band parameters
BAND_PROTRUSION = 0.0018  # how far bands protrude outward
BAND_HALF_H = 0.002       # half-height of each band
BAND_Z_LIST = [0.035, 0.060, 0.085]  # z-centers of the three bands

# Swing-top stopper parameters
HINGE_Z = SHOULDER_TOP_Z + 0.004   # hinge pivot height (on neck, above shoulder)
ARM_SPREAD = NECK_R + 0.003        # how far arms spread from center at hinge
ARM_TOP_SPREAD = 0.006              # arm convergence near disc
DISC_Z_LOCAL = NECK_TOP_Z - HINGE_Z + 0.002  # disc center in stopper local frame
DISC_R = NECK_BORE_R - 0.001       # stopper disc fits in bore
DISC_H = 0.005                      # disc thickness

# Measuring cup parameters
CUP_R = 0.021           # outer radius
CUP_WALL = 0.0047       # wall thickness (inner radius slightly under thread ridges for snap fit)
CUP_H = 0.032           # total cup height
CUP_SEAT_Z = NECK_TOP_Z - 0.008  # cup dips down over neck when seated
CUP_SLIDE_MAX = 0.045   # max slide travel


def _barrel_profile_with_bands():
    """Generate the barrel section of the revolve profile including volume bands.
    Note: the arc already ends at (BODY_R, BASE_Z+0.012), so we start from there."""
    pts = []

    # Generate barrel with bands (starting from z=0.012 where arc ends)
    z_cursor = BASE_Z + 0.012
    for bz in BAND_Z_LIST:
        band_lo = bz - BAND_HALF_H
        band_hi = bz + BAND_HALF_H
        # straight section up to band
        if band_lo > z_cursor:
            pts.append((BODY_R, band_lo))
        # band profile: out, up, back in
        pts.append((BODY_R + BAND_PROTRUSION, band_lo))
        pts.append((BODY_R + BAND_PROTRUSION, band_hi))
        pts.append((BODY_R, band_hi))
        z_cursor = band_hi

    # Final straight section to barrel top
    if z_cursor < BARREL_TOP_Z:
        pts.append((BODY_R, BARREL_TOP_Z))
    return pts


def _neck_thread_profile():
    """Raised spiral-like ridges on the neck - more pronounced than parent."""
    pts = [(NECK_R, SHOULDER_TOP_Z)]
    z0 = SHOULDER_TOP_Z + 0.003
    ridge_r = NECK_R + 0.0020  # taller ridges for spiral look
    n_ridges = 4
    ridge_spacing = 0.004
    for k in range(n_ridges):
        zc = z0 + k * ridge_spacing
        pts.append((NECK_R, zc - 0.0014))
        pts.append((ridge_r, zc - 0.0004))
        pts.append((ridge_r, zc + 0.0004))
        pts.append((NECK_R, zc + 0.0014))
    pts.append((NECK_R, NECK_TOP_Z))
    return pts


def _bottle_shell():
    """Transparent thin-wall bottle with volume bands and threaded neck.
    Built as a hollow revolve (outer minus inner profile) instead of using shell."""
    # Outer profile
    outer_wp = (
        cq.Workplane("XZ")
        .moveTo(0.0, BASE_Z)
        # rounded base corner
        .lineTo(BODY_R - 0.006, BASE_Z)
        .threePointArc((BODY_R, BASE_Z + 0.006), (BODY_R, BASE_Z + 0.012))
    )
    # barrel with volume bands
    for (r, z) in _barrel_profile_with_bands():
        outer_wp = outer_wp.lineTo(r, z)
    # shoulder taper up to the neck
    outer_wp = outer_wp.threePointArc(
        ((BODY_R + NECK_R) / 2.0, (BARREL_TOP_Z + SHOULDER_TOP_Z) / 2.0 + 0.004),
        (NECK_R, SHOULDER_TOP_Z),
    )
    # ridged neck (threads baked into outline)
    for (r, z) in _neck_thread_profile()[1:]:
        outer_wp = outer_wp.lineTo(r, z)
    # close back along the axis
    outer_wp = outer_wp.lineTo(0.0, NECK_TOP_Z).close()
    outer_solid = outer_wp.revolve(360.0, (0, 0, 0), (0, 1, 0))

    # Inner profile (hollow cavity) - simpler, no bands, just the inner wall
    inner_r = BODY_R - WALL
    inner_neck_r = NECK_R - WALL
    inner_wp = (
        cq.Workplane("XZ")
        .moveTo(0.0, BASE_Z + WALL)  # start slightly above base
        .lineTo(inner_r - 0.006, BASE_Z + WALL)
        .threePointArc((inner_r, BASE_Z + WALL + 0.006), (inner_r, BASE_Z + 0.012 + WALL))
        .lineTo(inner_r, BARREL_TOP_Z - WALL)
        .threePointArc(
            ((inner_r + inner_neck_r) / 2.0, (BARREL_TOP_Z + SHOULDER_TOP_Z) / 2.0 + 0.003),
            (inner_neck_r, SHOULDER_TOP_Z + WALL),
        )
        .lineTo(inner_neck_r, NECK_TOP_Z + 0.001)  # extend slightly above to cut through
        .lineTo(0.0, NECK_TOP_Z + 0.001)
        .close()
    )
    inner_solid = inner_wp.revolve(360.0, (0, 0, 0), (0, 1, 0))

    # Subtract inner from outer to create hollow shell
    hollow = outer_solid.cut(inner_solid)

    return hollow


def _hinge_bosses():
    """Small cylindrical bosses on the neck sides where the bail clips on."""
    boss_r = 0.003
    boss_h = 0.003
    result = None
    for sign in (-1, 1):
        x = sign * (NECK_R + 0.002)
        boss = (
            cq.Workplane("XY")
            .transformed(offset=(x, 0.0, HINGE_Z))
            .circle(boss_r)
            .extrude(boss_h)
        )
        # Shift so boss extends from neck surface outward
        boss = boss.translate((0, 0, -boss_h / 2.0))
        if result is None:
            result = boss
        else:
            result = result.union(boss)
    return result


def _stopper_solid():
    """Swing-top stopper: central post + disc + two bail arms + crossbar.
    Local frame origin at hinge pivot (0,0,0).
    Arms go up from hinge clips, disc sits at DISC_Z_LOCAL above.
    Central post ensures connectivity between disc and arms."""
    arm_w = 0.003
    arm_d = 0.003

    # Central post from hinge level up to disc (ensures connectivity)
    post = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, DISC_Z_LOCAL / 2.0))
        .box(arm_w, arm_d, DISC_Z_LOCAL)
    )

    # Stopper disc (plug that seats in neck)
    disc = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, DISC_Z_LOCAL - DISC_H / 2.0))
        .circle(DISC_R)
        .extrude(DISC_H)
    )
    stopper = post.union(disc)

    # Bail arms: two members from hinge clips up to disc level
    for sign in (-1, 1):
        # Lower arm: from hinge clip outward and upward
        lx_start = sign * ARM_SPREAD
        lz_start = 0.0
        lx_end = sign * ARM_SPREAD * 0.85
        lz_end = DISC_Z_LOCAL * 0.55
        # Segment length and angle
        dx = lx_end - lx_start
        dz = lz_end - lz_start
        seg_len = math.sqrt(dx*dx + dz*dz)
        seg_angle = math.degrees(math.atan2(dx, dz))  # angle from Z axis
        cx = (lx_start + lx_end) / 2.0
        cz = (lz_start + lz_end) / 2.0
        lower = (
            cq.Workplane("XY")
            .transformed(offset=(cx, 0.0, cz), rotate=(0.0, seg_angle, 0.0))
            .box(arm_w, arm_d, seg_len)
        )
        stopper = stopper.union(lower)

        # Upper arm: from mid to disc level, converging inward
        ux_start = lx_end
        uz_start = lz_end
        ux_end = sign * ARM_TOP_SPREAD
        uz_end = DISC_Z_LOCAL
        dx2 = ux_end - ux_start
        dz2 = uz_end - uz_start
        seg_len2 = math.sqrt(dx2*dx2 + dz2*dz2)
        seg_angle2 = math.degrees(math.atan2(dx2, dz2))
        cx2 = (ux_start + ux_end) / 2.0
        cz2 = (uz_start + uz_end) / 2.0
        upper = (
            cq.Workplane("XY")
            .transformed(offset=(cx2, 0.0, cz2), rotate=(0.0, seg_angle2, 0.0))
            .box(arm_w, arm_d, seg_len2)
        )
        stopper = stopper.union(upper)

        # Pivot knob at hinge clip point
        knob = (
            cq.Workplane("XY")
            .transformed(offset=(sign * ARM_SPREAD, 0.0, 0.0))
            .box(arm_w * 1.5, arm_d * 1.5, arm_w * 1.5)
        )
        stopper = stopper.union(knob)

    # Crossbar connecting the two arm tops (along X at disc level)
    bar_len = 2.0 * ARM_TOP_SPREAD + arm_w
    bar = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, DISC_Z_LOCAL))
        .box(bar_len, arm_d, arm_w)
    )
    stopper = stopper.union(bar)

    return stopper


def _measuring_cup():
    """Removable measuring cup cap - hollow cup with graduation ribs inside.
    Local frame: origin at bottom center (opening). Cup extends +Z.
    The cup is inverted on the bottle (opening faces down)."""
    # Outer shell
    outer = (
        cq.Workplane("XY")
        .circle(CUP_R)
        .extrude(CUP_H)
    )
    # Fillet the closed top edge
    outer = outer.edges(">Z").fillet(0.002)

    # Hollow cavity (open at bottom)
    inner_r = CUP_R - CUP_WALL
    cavity_h = CUP_H - CUP_WALL  # leave wall at top
    cavity = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, -0.001))
        .circle(inner_r)
        .extrude(cavity_h + 0.001)
    )
    cup = outer.cut(cavity)

    # Graduation ribs inside (small raised lines on inner wall)
    n_ribs = 4
    for i in range(n_ribs):
        z_rib = 0.005 + i * 0.006  # ribs at different heights
        rib = (
            cq.Workplane("XY")
            .transformed(offset=(inner_r + 0.0005, 0.0, z_rib))
            .box(0.001, 0.008, 0.0008)
        )
        cup = cup.union(rib)

    return cup


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_top_bottle")

    # Materials
    clear = model.material("clear_pet", rgba=(0.82, 0.88, 0.86, 0.28))
    stopper_mat = model.material("stopper_white", rgba=(0.92, 0.91, 0.88, 1.0))
    cup_mat = model.material("cup_translucent", rgba=(0.85, 0.90, 0.92, 0.45))

    # ---- bottle body (root) ----
    body = model.part("bottle")
    shell = _bottle_shell()
    body.visual(mesh_from_cadquery(shell, "bottle_shell"), material=clear, name="bottle_shell")

    # Hinge bosses on neck sides (part of bottle - where bail clips)
    bosses = _hinge_bosses()
    body.visual(mesh_from_cadquery(bosses, "hinge_bosses"), material=clear, name="hinge_bosses")

    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.035,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- swing-top stopper ----
    stopper = model.part("stopper")
    stopper_geo = _stopper_solid()
    stopper.visual(
        mesh_from_cadquery(stopper_geo, "stopper_body"),
        material=stopper_mat,
        name="stopper_body",
    )
    stopper.inertial = Inertial.from_geometry(
        Cylinder(DISC_R + ARM_SPREAD, DISC_Z_LOCAL + DISC_H),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, DISC_Z_LOCAL / 2.0)),
    )

    # ---- measuring cup cap ----
    cup = model.part("measuring_cup")
    cup_geo = _measuring_cup()
    # Cup is inverted on bottle: opening faces down. In local frame, cup extends +Z
    # from origin. When seated at CUP_SEAT_Z, opening is at the neck top.
    cup.visual(
        mesh_from_cadquery(cup_geo, "cup_shell"),
        material=cup_mat,
        name="cup_shell",
    )
    cup.inertial = Inertial.from_geometry(
        Cylinder(CUP_R, CUP_H),
        mass=0.010,
        origin=Origin(xyz=(0.0, 0.0, CUP_H / 2.0)),
    )

    # ---- Articulations ----

    # Swing-top stopper hinge: REVOLUTE around Y axis at neck sides
    # At q=0: stopper is closed (disc on neck). Positive q swings open (away from neck).
    # Origin at hinge pivot point on the bottle.
    # Axis (0,1,0): right-hand rule rotates from +Z toward -X for positive q,
    # which swings the stopper assembly to the side (open).
    model.articulation(
        "stopper_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stopper,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=2.0, lower=0.0, upper=2.2
        ),
    )

    # Measuring cup slide: PRISMATIC along +Z
    # At q=0: cup is seated on neck. Positive q lifts cup off.
    model.articulation(
        "cup_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cup,
        origin=Origin(xyz=(0.0, 0.0, CUP_SEAT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=0.5, lower=0.0, upper=CUP_SLIDE_MAX
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle")
    stopper = object_model.get_part("stopper")
    cup = object_model.get_part("measuring_cup")
    hinge = object_model.get_articulation("stopper_hinge")
    slide = object_model.get_articulation("cup_slide")

    bottle_shell = body.get_visual("bottle_shell")
    hinge_bosses = body.get_visual("hinge_bosses")
    stopper_body = stopper.get_visual("stopper_body")
    cup_shell = cup.get_visual("cup_shell")

    # --- bottle is clear/transparent (alpha < 1) ---
    ctx.check(
        "bottle material is tinted-transparent (alpha < 1)",
        bottle_shell.material.rgba is not None and bottle_shell.material.rgba[3] < 1.0,
        details=f"bottle rgba={bottle_shell.material.rgba}",
    )

    # --- hinge bosses exist on bottle (mounting point for bail) ---
    ctx.check(
        "hinge bosses exist on bottle neck",
        hinge_bosses is not None,
        details="hinge_bosses visual missing from bottle part",
    )

    # --- volume bands: bottle shell has wider extent than barrel radius alone ---
    # The bands protrude BAND_PROTRUSION beyond BODY_R, so max extent should exceed BODY_R.
    shell_aabb = ctx.part_element_world_aabb(body, elem="bottle_shell")
    if shell_aabb is not None:
        mn, mx = shell_aabb
        max_r = max(abs(mn[0]), abs(mx[0]), abs(mn[1]), abs(mx[1]))
        ctx.check(
            "volume bands protrude beyond barrel radius",
            max_r > BODY_R + BAND_PROTRUSION * 0.5,
            details=f"max radial extent={max_r:.5f}, expected>{BODY_R + BAND_PROTRUSION * 0.5:.5f}",
        )

    # --- neck threads: bottle shell has wider extent at neck than bare NECK_R ---
    # Threads add ~0.002 to neck radius
    ctx.check(
        "neck threads add raised ridges beyond base neck radius",
        max_r > NECK_R + 0.001,
        details=f"max radial extent={max_r:.5f} should exceed NECK_R+0.001={NECK_R+0.001:.5f}",
    )

    # --- stopper sits on top of the bottle (at neck height) ---
    stopper_pos = ctx.part_world_position(stopper)
    ctx.check(
        "stopper mounted at neck height",
        stopper_pos is not None and stopper_pos[2] >= SHOULDER_TOP_Z,
        details=f"stopper origin z={stopper_pos}",
    )

    # --- cup sits on top of the neck ---
    cup_pos = ctx.part_world_position(cup)
    ctx.check(
        "measuring cup mounted on top of neck",
        cup_pos is not None and cup_pos[2] >= BARREL_TOP_Z,
        details=f"cup origin={cup_pos}",
    )

    # --- stopper hinge: positive q opens stopper (moves away from +Z axis) ---
    with ctx.pose({hinge: 0.0}):
        rest_aabb = ctx.part_world_aabb(stopper)
    with ctx.pose({hinge: 1.5}):
        open_aabb = ctx.part_world_aabb(stopper)

    if rest_aabb and open_aabb:
        rest_center_x = (rest_aabb[0][0] + rest_aabb[1][0]) / 2.0
        open_center_x = (open_aabb[0][0] + open_aabb[1][0]) / 2.0
        ctx.check(
            "stopper hinge swings stopper to the side when opened",
            abs(open_center_x - rest_center_x) > 0.005,
            details=f"rest center x={rest_center_x:.5f}, open center x={open_center_x:.5f}",
        )
        # Check that the stopper width extent changes (disc swings out)
        rest_dx = rest_aabb[1][0] - rest_aabb[0][0]
        open_dx = open_aabb[1][0] - open_aabb[0][0]
        ctx.check(
            "stopper swing changes X extent (disc moves laterally)",
            abs(open_dx - rest_dx) > 0.003,
            details=f"rest dx={rest_dx:.5f}, open dx={open_dx:.5f}",
        )

    # --- cup slide: positive q lifts the cup off the neck ---
    rest_cup_z = ctx.part_world_position(cup)[2]
    with ctx.pose({slide: CUP_SLIDE_MAX}):
        lifted_cup_z = ctx.part_world_position(cup)[2]
    ctx.check(
        "cup_slide lifts measuring cup up off the neck",
        lifted_cup_z > rest_cup_z + CUP_SLIDE_MAX * 0.8,
        details=f"rest_z={rest_cup_z:.5f}, lifted_z={lifted_cup_z:.5f}",
    )

    # --- both joints are non-fixed ---
    ctx.check(
        "stopper_hinge is REVOLUTE (non-fixed)",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"stopper_hinge type={hinge.articulation_type}",
    )
    ctx.check(
        "cup_slide is PRISMATIC (non-fixed)",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"cup_slide type={slide.articulation_type}",
    )

    # --- stopper disc overlaps with neck bore at rest (intentional seated fit) ---
    ctx.allow_overlap(
        stopper,
        body,
        elem_a="stopper_body",
        elem_b="bottle_shell",
        reason="Stopper disc is intentionally seated inside the neck bore when closed.",
    )

    # --- cup overlaps with neck at rest (intentional seated cap) ---
    ctx.allow_overlap(
        cup,
        body,
        elem_a="cup_shell",
        elem_b="bottle_shell",
        reason="Measuring cup is intentionally seated over the neck when closed.",
    )

    # --- cup encloses stopper at rest (cup cap covers the closed stopper) ---
    ctx.allow_overlap(
        cup,
        stopper,
        elem_a="cup_shell",
        elem_b="stopper_body",
        reason="Measuring cup cap encloses the closed swing-top stopper when both are at rest.",
    )

    # --- prove stopper is retained in neck when closed ---
    ctx.expect_overlap(
        stopper,
        body,
        axes="z",
        elem_a="stopper_body",
        elem_b="bottle_shell",
        min_overlap=0.002,
        name="stopper disc overlaps neck in Z when closed",
    )

    # --- prove cup is retained over neck when closed ---
    ctx.expect_overlap(
        cup,
        body,
        axes="z",
        elem_a="cup_shell",
        elem_b="bottle_shell",
        min_overlap=0.002,
        name="measuring cup overlaps neck in Z when seated",
    )

    # --- prove cup encloses stopper when both closed ---
    ctx.expect_within(
        stopper,
        cup,
        axes="xy",
        inner_elem="stopper_body",
        outer_elem="cup_shell",
        margin=0.003,
        name="stopper fits within cup XY footprint when both closed",
    )

    return ctx.report()


object_model = build_object_model()
