from __future__ import annotations

# Exterior wall lantern sconce (conical-roof variant).
#
# A cast-iron octagonal wall backplate carries two bolted mounting brackets and
# a forged hook arm. A cylindrical amber-glass lantern hangs from a chain loop
# over that hook. The lantern is a single swinging assembly (conical aged-copper
# roof with finial + flared collar ring, translucent glass cylinder, two wrought
# cage bands, internal socket stem, and a glowing LED bulb). The primary
# mechanism is the lantern swinging on its hook: a REVOLUTE joint whose axis is
# parallel to the wall so the lantern body can rock toward and away from the
# facade.

from math import pi

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
# Key dimensions (meters)
# ---------------------------------------------------------------------------

# Wall backplate (octagonal cast-iron shield)
PLATE_W = 0.150
PLATE_H = 0.300
PLATE_T = 0.014
PLATE_CHAMFER = 0.034

# Hook arm: forged member that projects from the plate, curving up then over
# so its eye sits forward of the wall. The lantern loop hangs on the eye.
HOOK_REACH = 0.085  # how far the eye sits forward of the plate face (+X)
HOOK_EYE_Z = 0.064  # eye height above plate center
HOOK_EYE_R = 0.014  # outer radius of the forged eye
HOOK_BAR_R = 0.0065  # arm bar radius

# Lantern body
LANT_GLASS_R = 0.052
LANT_GLASS_H = 0.190
LANT_WALL_T = 0.0035
CAP_R = 0.060  # flared collar ring radius
CONE_HEIGHT = 0.035  # conical roof rise from collar top to apex
CONE_APEX_R = 0.005  # small flat at the cone apex (under the finial)
SOCKET_R = 0.013
SOCKET_H = 0.040
BULB_R = 0.030

# Vertical drop from the hook eye down to the top of the lantern cap.
# The lantern part frame is placed at the hook eye so the joint pivot is there
# (lantern-local origin == eye center). Must be large enough that the cap dome
# stays clear of the forged hook/eye above it.
LOOP_DROP = 0.052  # chain loop + spacing between eye and cap collar top


def _build_backplate_shape() -> cq.Workplane:
    """Octagonal cast-iron wall plate with chamfered corners and a center boss."""
    half_w = PLATE_W / 2.0
    half_h = PLATE_H / 2.0
    c = PLATE_CHAMFER
    # Octagon outline in the wall plane (Y horizontal, Z vertical).
    pts = [
        (-half_w + c, -half_h),
        (half_w - c, -half_h),
        (half_w, -half_h + c),
        (half_w, half_h - c),
        (half_w - c, half_h),
        (-half_w + c, half_h),
        (-half_w, half_h - c),
        (-half_w, -half_h + c),
    ]
    plate = (
        cq.Workplane("YZ")
        .polyline(pts)
        .close()
        .extrude(PLATE_T)  # extrude along +X (away from wall)
    )
    # Raised central spine boss running vertically (cast detail).
    spine = (
        cq.Workplane("YZ")
        .workplane(offset=PLATE_T)
        .rect(0.034, PLATE_H * 0.78)
        .extrude(0.006)
        .edges("|X")
        .fillet(0.004)
    )
    return plate.union(spine)


def _build_bracket_shape(z_center: float) -> cq.Workplane:
    """A bolted iron mounting tab standing proud of the plate face.

    Two of these (upper and lower) clamp the hook arm to the plate.
    """
    tab = (
        cq.Workplane("YZ")
        .workplane(offset=PLATE_T)
        .center(0.0, z_center)
        .rect(0.072, 0.030)
        .extrude(0.012)
        .edges("|X")
        .fillet(0.004)
    )
    # Two bolt heads (left/right) reading as cast bolts.
    for dy in (-0.024, 0.024):
        bolt = (
            cq.Workplane("YZ")
            .workplane(offset=PLATE_T + 0.012)
            .center(dy, z_center)
            .circle(0.0055)
            .extrude(0.004)
        )
        tab = tab.union(bolt)
    return tab


def _build_hook_arm_shape() -> cq.Workplane:
    """Forged arm: rises from the lower bracket, curves forward and up, and ends
    in a closed eye that sits forward of the wall. The lantern loop hangs here.
    """
    # Centerline control points of the arm in XZ (Y=0). The forged arm springs
    # from the UPPER bracket and reaches FORWARD (and slightly up) to the eye,
    # staying high near the eye height. Keeping it high clears the wide lantern
    # cap that hangs below. Sampled densely and built as overlapping spheres at
    # each sample joined by cylinders, so the result is one solid bar.
    ctrl = [
        (PLATE_T, 0.066),  # at the upper bracket face
        (PLATE_T + 0.022, 0.062),
        (PLATE_T + 0.044, 0.058),
        (HOOK_REACH, HOOK_EYE_Z + HOOK_EYE_R + 0.002),  # join eye top
    ]
    samples = _sample_polyline_xz(ctrl, n=40)

    arm = None
    for x, z in samples:
        ball = (
            cq.Workplane("YZ")
            .workplane(offset=x)
            .center(0.0, z)
            .sphere(HOOK_BAR_R)
        )
        arm = ball if arm is None else arm.union(ball)
    # Solid cylinders between consecutive samples guarantee connectivity.
    for (x0, z0), (x1, z1) in zip(samples, samples[1:]):
        length = ((x1 - x0) ** 2 + (z1 - z0) ** 2) ** 0.5
        if length < 1e-6:
            continue
        cyl = cq.Solid.makeCylinder(
            HOOK_BAR_R,
            length,
            pnt=cq.Vector(x0, 0.0, z0),
            dir=cq.Vector(x1 - x0, 0.0, z1 - z0),
        )
        arm = arm.union(cq.Workplane(obj=cyl))

    # Closed forged eye (a torus) at the top end. Its plane is XZ (axis along
    # +Y) so the hole faces sideways and a vertical chain loop can hang through
    # it. It overlaps the last bar sample so it stays one solid.
    eye_torus = cq.Solid.makeTorus(
        HOOK_EYE_R, HOOK_BAR_R, pnt=(0, 0, 0), dir=(0, 1, 0)
    )
    eye_wp = cq.Workplane(obj=eye_torus).translate((HOOK_REACH, 0.0, HOOK_EYE_Z))
    arm = arm.union(eye_wp)
    return arm


def _sample_polyline_xz(pts, n: int):
    """Resample a polyline given as (x, z) into n+1 points along its length."""
    flat = [(p[0], p[1]) for p in pts]
    segs = []
    total = 0.0
    for a, b in zip(flat, flat[1:]):
        d = ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5
        segs.append((a, b, d))
        total += d
    out = []
    for i in range(n + 1):
        target = total * i / n
        acc = 0.0
        for a, b, d in segs:
            if acc + d >= target or (a, b, d) is segs[-1]:
                t = 0.0 if d == 0 else (target - acc) / d
                out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
                break
            acc += d
    return out


# ---------------------------------------------------------------------------
# Lantern parts. The lantern part frame is at the hook-eye pivot (origin of the
# articulation). Geometry hangs DOWN (-Z) from there: a chain loop, the cap,
# the glass cylinder, the cage bands, the socket, and the bulb.
# ---------------------------------------------------------------------------

CAP_TOP_Z = -LOOP_DROP  # top of cap collar relative to the pivot
COLLAR_H = 0.022
GLASS_TOP_Z = CAP_TOP_Z - COLLAR_H
GLASS_BOT_Z = GLASS_TOP_Z - LANT_GLASS_H
BAND_R = LANT_GLASS_R + 0.004


def _build_loop_shape() -> cq.Workplane:
    """The hanging chain loop that drapes over the hook eye. A vertical ring in
    the XZ plane (hole along Y) whose upper arc captures the hook eye and whose
    lower arc meets the cap finial.

    Built in the lantern part frame (pivot at z=0), so its center sits a little
    below the pivot/eye. The ring radius (0.020) spans from the eye down to the
    finial top (z ~= +0.006)."""
    # The loop is authored in the LANTERN-LOCAL frame whose origin coincides
    # with the eye center (the joint pivot). Ring axis +X => it lies in the YZ
    # plane (hangs vertically) and drapes over the eye (whose plane is XZ).
    # Center slightly below the pivot: the loop's TOP arc rests on the eye's
    # upper tube (a small intentional overlap), and its BOTTOM arc meets the cap
    # finial below, keeping the cap clear of the hook.
    loop = cq.Solid.makeTorus(0.020, 0.0035, pnt=(0, 0, 0), dir=(1, 0, 0))
    loop_wp = cq.Workplane(obj=loop).translate((0.0, 0.0, -0.010))
    return loop_wp


def _build_cap_shape() -> cq.Workplane:
    """Conical aged-copper roof: a flared collar ring topped by a straight-sided
    cone with a turned finial at the apex.

    Built as one revolved (lathe) profile so the collar and cone are a single
    connected solid, then topped by a decorative finial (stem + ball) that
    extends up through the chain loop.
    """
    collar_z0 = GLASS_TOP_Z
    collar_z1 = GLASS_TOP_Z + COLLAR_H
    cone_apex_z = collar_z1 + CONE_HEIGHT
    # Revolved profile in (radius, z): collar wall, then a straight conical
    # slope from the full collar radius up to the small apex flat.
    # Add a subtle shoulder bead at the collar-to-cone transition.
    bead_r = CAP_R + 0.004
    bead_h = 0.005
    prof = [
        (0.0, collar_z0),
        (CAP_R, collar_z0),
        (CAP_R, collar_z1 - 0.002),
        (bead_r, collar_z1),           # bead outer
        (bead_r, collar_z1 + bead_h),  # bead top
        (CAP_R - 0.002, collar_z1 + bead_h + 0.002),  # cone start
        (CONE_APEX_R, cone_apex_z),    # cone apex
        (0.0, cone_apex_z),
    ]
    cap = (
        cq.Workplane("XZ")
        .polyline([(r, z) for r, z in prof])
        .close()
        .revolve(360.0, (0, 0, 0), (0, 1, 0))
    )
    # Turned finial: a stem passing through the cone apex and the chain loop,
    # topped by a small decorative ball. The stem extends down past the apex to
    # overlap the loop's bottom arc, keeping the loop and cap connected within
    # the lantern part. The ball sits inside the loop hole, well below the hook
    # eye, so it never collides with the forged hook above.
    stem_bot_z = cone_apex_z - 0.010  # reaches down into the loop bottom arc
    stem_top_z = cone_apex_z + 0.012  # rises above the apex
    stem = (
        cq.Workplane("XY")
        .workplane(offset=stem_bot_z)
        .circle(0.003)
        .extrude(stem_top_z - stem_bot_z)
    )
    ball_center_z = stem_top_z + 0.004
    ball = (
        cq.Workplane("XY")
        .workplane(offset=ball_center_z)
        .sphere(0.004)
    )
    return cap.union(stem).union(ball)


def _build_glass_shape() -> cq.Workplane:
    """Hollow translucent glass cylinder (open-ish tube with thin walls)."""
    outer = (
        cq.Workplane("XY")
        .workplane(offset=GLASS_BOT_Z)
        .circle(LANT_GLASS_R)
        .extrude(LANT_GLASS_H)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=GLASS_BOT_Z - 0.001)
        .circle(LANT_GLASS_R - LANT_WALL_T)
        .extrude(LANT_GLASS_H + 0.002)
    )
    return outer.cut(inner)


def _build_base_ring_shape() -> cq.Workplane:
    """Iron base ring that closes the bottom of the lantern and seats the glass."""
    ring = (
        cq.Workplane("XY")
        .workplane(offset=GLASS_BOT_Z - 0.012)
        .circle(CAP_R)
        .extrude(0.014)
        .edges("<Z")
        .chamfer(0.004)
    )
    return ring


def _build_cage_shape() -> cq.Workplane:
    """Two wrought-iron horizontal cage bands plus four vertical straps that
    join the cap collar to the base ring, protecting the glass."""
    cage = None
    # Horizontal bands at two heights.
    for frac in (0.32, 0.70):
        z = GLASS_BOT_Z + LANT_GLASS_H * frac
        band_outer = (
            cq.Workplane("XY")
            .workplane(offset=z - 0.004)
            .circle(BAND_R)
            .extrude(0.008)
        )
        band_inner = (
            cq.Workplane("XY")
            .workplane(offset=z - 0.005)
            .circle(BAND_R - 0.004)
            .extrude(0.010)
        )
        band = band_outer.cut(band_inner)
        cage = band if cage is None else cage.union(band)
    # Vertical straps tying cap collar to base ring.
    strap_top = GLASS_TOP_Z + 0.004
    strap_bot = GLASS_BOT_Z - 0.006
    strap_h = strap_top - strap_bot
    for k in range(4):
        ang = pi / 4 + k * (pi / 2)
        sx = (BAND_R) * 1.0 * __import__("math").cos(ang)
        sy = (BAND_R) * 1.0 * __import__("math").sin(ang)
        strap = (
            cq.Workplane("XY")
            .workplane(offset=strap_bot)
            .center(sx, sy)
            .rect(0.010, 0.005)
            .extrude(strap_h)
        )
        cage = cage.union(strap)
    return cage


def _build_socket_shape() -> cq.Workplane:
    """Lamp socket stem hanging from the cap dome, holding the bulb."""
    # Stem starts up inside the cap collar so the socket connects to the cap.
    stem = (
        cq.Workplane("XY")
        .workplane(offset=GLASS_TOP_Z + 0.006)
        .circle(0.010)
        .extrude(-0.038)  # downward, into the lantern
    )
    socket = (
        cq.Workplane("XY")
        .workplane(offset=GLASS_TOP_Z - 0.032)
        .circle(SOCKET_R)
        .extrude(-SOCKET_H)
    )
    return stem.union(socket)


def _build_bulb_shape() -> cq.Workplane:
    """A standard A-shape LED bulb screwed into the socket, glowing."""
    socket_bottom_z = GLASS_TOP_Z - 0.032 - SOCKET_H
    # Place the bulb just below the socket, with its neck overlapping the socket.
    bulb_center_z = socket_bottom_z - BULB_R + 0.008
    bulb = (
        cq.Workplane("XY")
        .workplane(offset=bulb_center_z)
        .sphere(BULB_R)
    )
    # Screw-base neck reaching up into the socket bottom (keeps bulb connected).
    neck = (
        cq.Workplane("XY")
        .workplane(offset=bulb_center_z + BULB_R - 0.006)
        .circle(0.013)
        .extrude(0.020)
    )
    return bulb.union(neck)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="exterior_wall_lantern_conical_roof")

    model.material("cast_iron", rgba=(0.16, 0.17, 0.18, 1.0))
    model.material("wrought_iron", rgba=(0.22, 0.23, 0.24, 1.0))
    model.material("aged_copper", rgba=(0.42, 0.28, 0.18, 1.0))
    model.material("amber_glass", rgba=(0.74, 0.58, 0.30, 0.45))
    model.material("warm_bulb", rgba=(1.0, 0.96, 0.82, 1.0))
    model.material("socket_brass", rgba=(0.55, 0.45, 0.22, 1.0))

    # ---- Root: wall backplate + brackets + hook arm ----
    backplate = model.part("wall_backplate")
    backplate.visual(
        mesh_from_cadquery(_build_backplate_shape(), "backplate"),
        material="cast_iron",
        name="backplate_shield",
    )
    backplate.visual(
        mesh_from_cadquery(_build_bracket_shape(0.070), "upper_bracket"),
        material="cast_iron",
        name="upper_bracket",
    )
    backplate.visual(
        mesh_from_cadquery(_build_bracket_shape(-0.034), "lower_bracket"),
        material="cast_iron",
        name="lower_bracket",
    )
    backplate.visual(
        mesh_from_cadquery(_build_hook_arm_shape(), "hook_arm"),
        material="wrought_iron",
        name="hook_arm",
    )

    # ---- Lantern: swinging assembly hung at the hook eye ----
    lantern = model.part("lantern")
    lantern.visual(
        mesh_from_cadquery(_build_loop_shape(), "chain_loop"),
        material="wrought_iron",
        name="chain_loop",
    )
    lantern.visual(
        mesh_from_cadquery(_build_cap_shape(), "conical_roof"),
        material="aged_copper",
        name="conical_roof",
    )
    lantern.visual(
        mesh_from_cadquery(_build_glass_shape(), "lantern_glass"),
        material="amber_glass",
        name="lantern_glass",
    )
    lantern.visual(
        mesh_from_cadquery(_build_base_ring_shape(), "base_ring"),
        material="cast_iron",
        name="base_ring",
    )
    lantern.visual(
        mesh_from_cadquery(_build_cage_shape(), "lantern_cage"),
        material="wrought_iron",
        name="lantern_cage",
    )
    lantern.visual(
        mesh_from_cadquery(_build_socket_shape(), "lamp_socket"),
        material="socket_brass",
        name="lamp_socket",
    )
    lantern.visual(
        mesh_from_cadquery(_build_bulb_shape(), "led_bulb"),
        material="warm_bulb",
        name="led_bulb",
    )

    # The lantern hangs from the hook eye. Pivot at the eye; axis is parallel to
    # the wall (Y). The lantern body sits below the pivot (local -Z) and forward
    # (+X). With axis=(0,-1,0), positive q rotates points below the pivot toward
    # +X, i.e. the lantern bottom swings OUTWARD away from the facade, like a
    # gimbal-hung lantern rocking in the wind.
    model.articulation(
        "lantern_swing",
        ArticulationType.REVOLUTE,
        parent=backplate,
        child=lantern,
        origin=Origin(xyz=(HOOK_REACH, 0.0, HOOK_EYE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.35, upper=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    backplate = object_model.get_part("wall_backplate")
    lantern = object_model.get_part("lantern")
    swing = object_model.get_articulation("lantern_swing")

    # --- Structure / root ---
    ctx.check(
        "backplate is the single root",
        len(object_model.root_parts()) == 1
        and object_model.root_parts()[0].name == "wall_backplate",
        details=f"roots={[p.name for p in object_model.root_parts()]}",
    )

    # --- Joint type / axis (revolute hung pivot, axis parallel to the wall) ---
    ctx.check(
        "lantern swing is revolute about the wall-parallel Y axis",
        swing.articulation_type == ArticulationType.REVOLUTE
        and abs(swing.axis[1]) > 0.99
        and abs(swing.axis[0]) < 0.01
        and abs(swing.axis[2]) < 0.01,
        details=f"type={swing.articulation_type}, axis={swing.axis}",
    )

    # --- The hook eye sits forward of the wall and the lantern hangs below it ---
    plate_box = ctx.part_element_world_aabb(backplate, elem="backplate_shield")
    hook_box = ctx.part_element_world_aabb(backplate, elem="hook_arm")
    glass_box = ctx.part_element_world_aabb(lantern, elem="lantern_glass")
    ctx.check(
        "hook reaches forward of the wall plate",
        hook_box is not None and plate_box is not None and hook_box[1][0] > plate_box[1][0] + 0.03,
        details=f"hook_max_x={hook_box[1][0] if hook_box else None}, plate_max_x={plate_box[1][0] if plate_box else None}",
    )
    ctx.check(
        "glass hangs below the hook eye",
        glass_box is not None and hook_box is not None and glass_box[1][2] < hook_box[1][2],
        details=f"glass_top_z={glass_box[1][2] if glass_box else None}, hook_top_z={hook_box[1][2] if hook_box else None}",
    )

    # --- Conical roof: wider at the base than tall, proving it tapers to an apex ---
    roof_box = ctx.part_element_world_aabb(lantern, elem="conical_roof")
    ctx.check(
        "conical roof is present as a named visual",
        roof_box is not None,
        details="conical_roof visual not found on lantern part",
    )
    if roof_box is not None:
        roof_dx = roof_box[1][0] - roof_box[0][0]
        roof_dy = roof_box[1][1] - roof_box[0][1]
        roof_dz = roof_box[1][2] - roof_box[0][2]
        roof_xy_span = max(roof_dx, roof_dy)
        ctx.check(
            "conical roof XY span exceeds its height (tapering cone)",
            roof_xy_span > roof_dz + 0.005,
            details=f"xy_span={roof_xy_span:.4f}, z_span={roof_dz:.4f}",
        )
    # Verify the roof uses the aged-copper material (not cast iron like the parent dome).
    roof_visual = lantern.get_visual("conical_roof")
    ctx.check(
        "conical roof uses aged_copper material",
        roof_visual is not None and roof_visual.material == "aged_copper",
        details=f"material={roof_visual.material if roof_visual else 'missing'}",
    )

    # --- Glass is below the roof and above the base ring (the lantern envelope) ---
    ctx.expect_gap(
        lantern,
        lantern,
        axis="z",
        max_penetration=0.008,
        positive_elem="conical_roof",
        negative_elem="lantern_glass",
        name="conical roof seats on top of the glass",
    )
    # --- Cage straps span the glass: cage encloses the glass cylinder in plan ---
    ctx.expect_within(
        lantern,
        lantern,
        axes="xy",
        margin=0.0,
        inner_elem="lantern_glass",
        outer_elem="lantern_cage",
        name="cage encloses the glass cylinder",
    )
    # --- Bulb is inside the glass envelope (plan view) ---
    ctx.expect_within(
        lantern,
        lantern,
        axes="xy",
        margin=0.0,
        inner_elem="led_bulb",
        outer_elem="lantern_glass",
        name="bulb sits inside the glass cylinder",
    )
    # --- Bulb sits inside the vertical span of the glass too ---
    ctx.expect_overlap(
        lantern,
        lantern,
        axes="z",
        min_overlap=0.02,
        elem_a="led_bulb",
        elem_b="lantern_glass",
        name="bulb is enclosed within the glass height",
    )

    # --- Loop/eye capture: the chain loop overlaps the hook eye (hung joint) ---
    ctx.allow_overlap(
        lantern,
        backplate,
        elem_a="chain_loop",
        elem_b="hook_arm",
        reason="The hanging chain loop is captured over the forged hook eye; the loop and eye interlock as a hung pivot.",
    )
    ctx.expect_overlap(
        lantern,
        backplate,
        axes="xz",
        min_overlap=0.001,
        elem_a="chain_loop",
        elem_b="hook_arm",
        name="chain loop is captured on the hook eye",
    )

    # --- Mechanism: positive swing rocks the lantern outward (+X) at rest pose vs posed ---
    rest_pos = ctx.part_world_position(lantern)
    bottom_rest = ctx.part_element_world_aabb(lantern, elem="base_ring")
    with ctx.pose({swing: 0.30}):
        bottom_swung = ctx.part_element_world_aabb(lantern, elem="base_ring")
    # Rotating +0.30 about +Y pushes points below the pivot toward +X.
    ctx.check(
        "positive swing moves the lantern bottom outward (+X)",
        bottom_rest is not None
        and bottom_swung is not None
        and bottom_swung[0][0] > bottom_rest[0][0] + 0.01,
        details=f"rest_min_x={bottom_rest[0][0] if bottom_rest else None}, swung_min_x={bottom_swung[0][0] if bottom_swung else None}",
    )
    ctx.check(
        "lantern origin is at the hook eye pivot",
        rest_pos is not None and abs(rest_pos[0] - HOOK_REACH) < 1e-6 and abs(rest_pos[2] - HOOK_EYE_Z) < 1e-6,
        details=f"lantern_origin={rest_pos}",
    )

    return ctx.report()


object_model = build_object_model()
