from __future__ import annotations

# Rooftop yagi / TV antenna on a tall vertical mast — X-shaped dual-boom variant.
#
# Coordinate convention:
#   - up is +Z; the mast foot sits on the roof at z = 0.
#   - the yagi booms nominally point along the head's local +X (toward the
#     broadcast tower). Two booms splay at ±boom_splay_angle from +X, crossing
#     at a central hub to form an X shape when viewed from above.
#   - the wide flat reflector grid sits at the rear (-X) of the assembly; the
#     graduated dipole/director rods straddle each boom left/right and get
#     shorter toward the front.
#
# Structure / articulation:
#   - mast (root, static): tall weathered metal pole standing on the roof, with
#     two small standoff mounting brackets partway down.
#   - antenna_head (REVOLUTE about +Z, primary AZIMUTH): a rotation collar /
#     bearing hub clamped to the mast top.
#   - yagi_boom (REVOLUTE about +Y, secondary ELEVATION tilt): the X-shaped
#     dual-boom assembly carrying the reflector grid, the director rod elements
#     on each boom, and two small junction / balun boxes.

import math

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
    mesh_from_geometry,
    tube_from_spline_points,
)


# ---- shared geometry helpers -------------------------------------------------

def _rod(name: str, length: float, radius: float = 0.0035):
    """A straight thin rod centered at its midpoint, lying along local +X."""
    return mesh_from_geometry(
        tube_from_spline_points(
            [(-length / 2.0, 0.0, 0.0), (0.0, 0.0, 0.0), (length / 2.0, 0.0, 0.0)],
            radius=radius,
            samples_per_segment=2,
            radial_segments=8,
            cap_ends=True,
        ),
        name,
    )


def _boom_point(x_local: float, boom_angle: float, boom_z: float = 0.0):
    """Convert a position along a splayed boom's local axis to world XYZ."""
    return (
        x_local * math.cos(boom_angle),
        x_local * math.sin(boom_angle),
        boom_z,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_yagi_antenna")

    weathered_metal = model.material("weathered_metal", rgba=(0.82, 0.81, 0.80, 1.0))
    bright_alloy = model.material("bright_alloy", rgba=(0.88, 0.89, 0.91, 1.0))
    dark_box = model.material("dark_box", rgba=(0.16, 0.17, 0.19, 1.0))
    clamp_gray = model.material("clamp_gray", rgba=(0.55, 0.57, 0.60, 1.0))

    # ------------------------------------------------------------------ scales
    mast_len = 3.40          # visible mast height
    mast_r = 0.016           # ~32 mm OD steel mast
    head_z = mast_len - 0.05  # the rotation collar sits just below the mast top

    # ====================================================================== MAST
    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=mast_r, length=mast_len),
        origin=Origin(xyz=(0.0, 0.0, mast_len / 2.0)),
        material=weathered_metal,
        name="mast_pole",
    )
    # small foot plate so the mast reads as planted on the roof, not floating
    mast.visual(
        Cylinder(radius=0.060, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=clamp_gray,
        name="foot_plate",
    )
    # two small standoff mounting brackets partway down the mast (visible tabs)
    for i, bz in enumerate((0.95, 1.95)):
        mast.visual(
            Box((0.14, 0.022, 0.014)),
            origin=Origin(xyz=(0.085, 0.0, bz)),
            material=clamp_gray,
            name=f"standoff_bracket_{i}",
        )
        # the little wall pad at the outboard end of each standoff
        mast.visual(
            Box((0.018, 0.040, 0.040)),
            origin=Origin(xyz=(0.150, 0.0, bz)),
            material=clamp_gray,
            name=f"standoff_pad_{i}",
        )
    mast.inertial = Inertial.from_geometry(
        Cylinder(radius=mast_r, length=mast_len),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, mast_len / 2.0)),
    )

    # ============================================================= ANTENNA HEAD
    # The azimuth rotation collar/bearing that clamps onto the mast top. Its part
    # frame is centered on the mast axis at head_z so azimuth spin is clean.
    antenna_head = model.part("antenna_head")
    antenna_head.visual(
        Cylinder(radius=0.030, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=clamp_gray,
        name="rotation_collar",
    )
    # U-bolt clamp blocks gripping the mast
    for j, zoff in enumerate((-0.035, 0.035)):
        antenna_head.visual(
            Box((0.052, 0.064, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, zoff)),
            material=dark_box,
            name=f"clamp_block_{j}",
        )
    # short stub that lifts the elevation pivot just above the collar
    antenna_head.visual(
        Box((0.040, 0.030, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=clamp_gray,
        name="elevation_post",
    )
    antenna_head.inertial = Inertial.from_geometry(
        Cylinder(radius=0.05, length=0.20),
        mass=0.6,
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
    )

    # ============================================================ X-SHAPED BOOM
    # Two booms cross at a central hub and splay forward at ±boom_splay_angle
    # from the nominal +X direction. Each boom carries its own row of graduated
    # transverse director rods. The shared reflector grid sits at the rear.
    yagi_boom = model.part("yagi_boom")

    boom_splay = math.radians(14.0)   # each boom splays ±14° from centerline
    boom_len = 1.55
    boom_back = -0.45        # rear end (along each boom's local axis)
    boom_front = boom_len + boom_back  # front end = 1.10
    boom_mid = (boom_front + boom_back) / 2.0
    boom_z = 0.0             # boom centerline at the pivot height

    # ---- central hub where the two booms cross ------------------------------
    yagi_boom.visual(
        Cylinder(radius=0.042, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, boom_z)),
        material=clamp_gray,
        name="hub",
    )

    # ---- two splayed boom spines + director elements ------------------------
    n_directors = 8
    # Element length taper: longest near rear, shortest at front
    elem_max_len = 0.74
    elem_min_len = 0.26
    # Element positions along each boom's local axis
    elem_x_start = boom_back + 0.12
    elem_x_end = boom_front - 0.08

    for b, sign in enumerate((+1, -1)):
        angle = sign * boom_splay

        # boom spine: a slim square-section aluminium tube
        cx, cy, cz = _boom_point(boom_mid, angle, boom_z)
        yagi_boom.visual(
            Box((boom_len, 0.024, 0.024)),
            origin=Origin(xyz=(cx, cy, cz), rpy=(0.0, 0.0, angle)),
            material=bright_alloy,
            name=f"boom_spine_{b}",
        )

        # graduated transverse director rods on this boom
        for i in range(n_directors):
            frac = i / (n_directors - 1)
            ex_local = elem_x_start + frac * (elem_x_end - elem_x_start)
            elen = elem_max_len + frac * (elem_min_len - elem_max_len)
            # world position along the boom axis
            wx, wy, wz = _boom_point(ex_local, angle, boom_z)
            # rod perpendicular to the boom: rotate local +X by angle + 90°
            rod_angle = angle + math.pi / 2.0
            yagi_boom.visual(
                _rod(f"element_{b}_{i}", elen, radius=0.0035),
                origin=Origin(xyz=(wx, wy, wz), rpy=(0.0, 0.0, rod_angle)),
                material=bright_alloy,
                name=f"element_{b}_{i}",
            )

    # ---- two small junction / balun boxes, one on each boom -----------------
    for b, sign in enumerate((+1, -1)):
        angle = sign * boom_splay
        box_x_local = boom_back + 0.24
        bx, by, bz = _boom_point(box_x_local, angle, boom_z)
        yagi_boom.visual(
            Box((0.065, 0.048, 0.042)),
            origin=Origin(xyz=(bx, by, boom_z + 0.028), rpy=(0.0, 0.0, angle)),
            material=dark_box,
            name=f"balun_box_{b}",
        )

    # ---- wide flat reflector grid at the rear of the boom assembly ----------
    # The reflector is centered on the assembly centerline (Y=0) behind the hub.
    refl_x = boom_back * math.cos(boom_splay) - 0.02
    refl_half_h = 0.62               # screen total height ~1.24 m
    refl_half_w = 0.36               # screen half-width in Y
    n_grid = 17

    # two vertical end stiles (along local Z), one at each Y extreme
    for s, ys in enumerate((-refl_half_w, refl_half_w)):
        stile = _rod(f"reflector_stile_{s}", 2.0 * refl_half_h, radius=0.0045)
        yagi_boom.visual(
            stile,
            origin=Origin(xyz=(refl_x, ys, boom_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=weathered_metal,
            name=f"reflector_stile_{s}",
        )
    # horizontal grid rods spanning the full width in Y, stacked along Z
    for g in range(n_grid):
        frac = g / (n_grid - 1)
        gz = boom_z - refl_half_h + frac * (2.0 * refl_half_h)
        grid_rod = _rod(f"reflector_grid_{g:02d}", 2.0 * refl_half_w, radius=0.0028)
        yagi_boom.visual(
            grid_rod,
            origin=Origin(xyz=(refl_x, 0.0, gz), rpy=(0.0, 0.0, math.pi / 2.0)),
            material=weathered_metal,
            name=f"reflector_grid_{g:02d}",
        )

    # ---- connecting struts from each boom rear to the reflector frame -------
    # Each strut bridges the gap from a splayed boom's rear end to the centered
    # reflector frame, ensuring the reflector reads as supported by both booms.
    for b, sign in enumerate((+1, -1)):
        angle = sign * boom_splay
        # rear end of this boom in world coords
        rear_x, rear_y, _ = _boom_point(boom_back, angle, boom_z)
        # reflector frame attachment point (centerline side)
        attach_y = sign * refl_half_w * 0.3  # attach partway along the stiles
        # strut runs from boom rear to reflector frame
        dx = refl_x - rear_x
        dy = attach_y - rear_y
        strut_len = math.sqrt(dx * dx + dy * dy)
        strut_angle = math.atan2(dy, dx)
        mid_x = (rear_x + refl_x) / 2.0
        mid_y = (rear_y + attach_y) / 2.0
        yagi_boom.visual(
            Box((strut_len, 0.018, 0.018)),
            origin=Origin(xyz=(mid_x, mid_y, boom_z), rpy=(0.0, 0.0, strut_angle)),
            material=bright_alloy,
            name=f"reflector_strut_{b}",
        )
    # a short central spine piece connecting the hub area to the reflector plane
    # Start just behind the hub (x = -0.05) to clear the elevation post pivot zone
    center_strut_front = -0.05
    center_strut_len = abs(refl_x - center_strut_front)
    yagi_boom.visual(
        Box((center_strut_len, 0.020, 0.020)),
        origin=Origin(xyz=((center_strut_front + refl_x) / 2.0, 0.0, boom_z)),
        material=bright_alloy,
        name="reflector_center_strut",
    )

    yagi_boom.inertial = Inertial.from_geometry(
        Box((boom_len, 1.00, 1.30)),
        mass=2.8,
        origin=Origin(xyz=(boom_mid * math.cos(boom_splay), 0.0, boom_z)),
    )

    # ============================================================ ARTICULATIONS
    # PRIMARY: azimuth rotation of the whole head about the vertical mast axis.
    model.articulation(
        "azimuth_rotation",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=antenna_head,
        origin=Origin(xyz=(0.0, 0.0, head_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.2,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    # SECONDARY: elevation tilt of the boom assembly about the horizontal -Y axis.
    # Boom extends along local +X from the pivot; -Y axis lifts the front (+X)
    # end upward for positive q, so the antenna can be aimed slightly up/down.
    model.articulation(
        "elevation_tilt",
        ArticulationType.REVOLUTE,
        parent=antenna_head,
        child=yagi_boom,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.0,
            lower=-0.35,
            upper=0.35,
        ),
    )

    return model


def _aabb_center(aabb):
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    mast = object_model.get_part("mast")
    head = object_model.get_part("antenna_head")
    boom = object_model.get_part("yagi_boom")
    azimuth = object_model.get_articulation("azimuth_rotation")
    elevation = object_model.get_articulation("elevation_tilt")

    # --- intentional joint-fit overlaps --------------------------------------
    # The rotation collar deliberately wraps around the mast top (a real antenna
    # rotor/collar clamps over the pole), and the elevation post is the boom's
    # pivot boss seated into the hub. Both are genuine mechanical fits.
    ctx.allow_overlap(
        head,
        mast,
        elem_a="rotation_collar",
        elem_b="mast_pole",
        reason="The azimuth rotation collar clamps around the mast top; the bearing fit is intentional.",
    )
    for cb in ("clamp_block_0", "clamp_block_1"):
        ctx.allow_overlap(
            head,
            mast,
            elem_a=cb,
            elem_b="mast_pole",
            reason="U-bolt clamp blocks grip around the mast pole to hold the rotor collar.",
        )
    ctx.allow_overlap(
        head,
        boom,
        elem_a="elevation_post",
        elem_b="hub",
        reason="The elevation pivot post is seated into the boom hub at the tilt joint.",
    )
    for bs in ("boom_spine_0", "boom_spine_1"):
        ctx.allow_overlap(
            head,
            boom,
            elem_a="elevation_post",
            elem_b=bs,
            reason="The elevation post supports the X-boom at the hub crossing; local contact at the pivot is intentional.",
        )

    # --- base sits on the roof at z ~ 0 (not buried, not floating) ------------
    mast_aabb = ctx.part_world_aabb(mast)
    ctx.check(
        "mast foot rests on the roof at z~0",
        mast_aabb is not None and abs(mast_aabb[0][2]) < 0.02,
        details=f"mast world aabb min z = {None if mast_aabb is None else mast_aabb[0][2]}",
    )

    # --- mast is a tall vertical pole ----------------------------------------
    ctx.check(
        "mast is tall (>3 m)",
        mast_aabb is not None and (mast_aabb[1][2] - mast_aabb[0][2]) > 3.0,
        details=f"mast height = {None if mast_aabb is None else mast_aabb[1][2] - mast_aabb[0][2]}",
    )

    # --- the head/boom assembly sits at the mast top -------------------------
    head_pos = ctx.part_world_position(head)
    ctx.check(
        "antenna head mounted near the mast top",
        head_pos is not None and head_pos[2] > 3.0,
        details=f"head world z = {None if head_pos is None else head_pos[2]}",
    )

    # --- X-SHAPED DUAL BOOM: two splayed boom spines exist -------------------
    spine_0 = ctx.part_element_world_aabb(boom, elem="boom_spine_0")
    spine_1 = ctx.part_element_world_aabb(boom, elem="boom_spine_1")
    ctx.check(
        "dual boom: both boom spines exist",
        spine_0 is not None and spine_1 is not None,
        details="boom_spine_0 and boom_spine_1 must exist",
    )
    # The two boom front ends should be spread apart in Y (X-shape splay)
    # Check that the front tip elements on boom 0 and boom 1 are separated in Y
    tip_0 = ctx.part_element_world_aabb(boom, elem="element_0_7")
    tip_1 = ctx.part_element_world_aabb(boom, elem="element_1_7")
    if tip_0 is not None and tip_1 is not None:
        tip_0_center = _aabb_center(tip_0)
        tip_1_center = _aabb_center(tip_1)
        y_sep = abs(tip_0_center[1] - tip_1_center[1])
        ctx.check(
            "X-boom: front tips are splayed apart in Y",
            y_sep > 0.15,
            details=f"front tip Y separation = {y_sep:.4f} m",
        )
    # The rear element ends should also be spread apart (X-shape)
    rear_0 = ctx.part_element_world_aabb(boom, elem="element_0_0")
    rear_1 = ctx.part_element_world_aabb(boom, elem="element_1_0")
    if rear_0 is not None and rear_1 is not None:
        rear_0_center = _aabb_center(rear_0)
        rear_1_center = _aabb_center(rear_1)
        y_sep_rear = abs(rear_0_center[1] - rear_1_center[1])
        ctx.check(
            "X-boom: rear elements are splayed apart in Y",
            y_sep_rear > 0.05,
            details=f"rear element Y separation = {y_sep_rear:.4f} m",
        )

    # --- PRIMARY: azimuth is a vertical-axis revolute joint ------------------
    ctx.check(
        "azimuth joint is revolute about +Z",
        azimuth.articulation_type == ArticulationType.REVOLUTE
        and tuple(azimuth.axis) == (0.0, 0.0, 1.0),
        details=f"type={azimuth.articulation_type}, axis={azimuth.axis}",
    )
    # spinning azimuth should swing the (off-axis) reflector grid around in X/Y
    refl_rest = _aabb_center(ctx.part_element_world_aabb(boom, elem="reflector_grid_08"))
    with ctx.pose({azimuth: math.pi / 2.0}):
        refl_spun = _aabb_center(ctx.part_element_world_aabb(boom, elem="reflector_grid_08"))
    ctx.check(
        "azimuth rotation swings the antenna head horizontally",
        abs(refl_spun[0] - refl_rest[0]) > 0.05 or abs(refl_spun[1] - refl_rest[1]) > 0.05,
        details=f"rest={refl_rest}, spun={refl_spun}",
    )

    # --- SECONDARY: elevation is a horizontal-axis revolute joint ------------
    ctx.check(
        "elevation joint is revolute about the horizontal Y axis",
        elevation.articulation_type == ArticulationType.REVOLUTE
        and tuple(elevation.axis) in ((0.0, 1.0, 0.0), (0.0, -1.0, 0.0)),
        details=f"type={elevation.articulation_type}, axis={elevation.axis}",
    )
    # tilting elevation up should raise the front director tip
    front_rest = _aabb_center(ctx.part_element_world_aabb(boom, elem="element_0_7"))
    with ctx.pose({elevation: 0.35}):
        front_up = _aabb_center(ctx.part_element_world_aabb(boom, elem="element_0_7"))
    ctx.check(
        "elevation tilt raises the front of the boom",
        front_up[2] > front_rest[2] + 0.02,
        details=f"rest tip z={front_rest[2]}, tilted tip z={front_up[2]}",
    )

    # --- reflector grid reads as a wide flat screen at the rear --------------
    grid_lo = ctx.part_element_world_aabb(boom, elem="reflector_grid_00")
    grid_hi = ctx.part_element_world_aabb(boom, elem="reflector_grid_16")
    ctx.check(
        "reflector grid spans a tall screen",
        grid_lo is not None and grid_hi is not None
        and (_aabb_center(grid_hi)[2] - _aabb_center(grid_lo)[2]) > 0.8,
        details="grid vertical span",
    )
    # the reflector is behind (-X of) the front director element
    refl_strut_aabb = ctx.part_element_world_aabb(boom, elem="reflector_center_strut")
    front_dir_aabb = ctx.part_element_world_aabb(boom, elem="element_0_7")
    ctx.check(
        "reflector grid is at the rear of the boom",
        _aabb_center(refl_strut_aabb)[0] < _aabb_center(front_dir_aabb)[0],
        details=f"reflector x={_aabb_center(refl_strut_aabb)[0]}, front x={_aabb_center(front_dir_aabb)[0]}",
    )

    # --- director elements taper shorter toward the front on each boom -------
    for b in range(2):
        rear_el = ctx.part_element_world_aabb(boom, elem=f"element_{b}_0")
        front_el = ctx.part_element_world_aabb(boom, elem=f"element_{b}_7")
        if rear_el is not None and front_el is not None:
            # Measure the span perpendicular to the boom (Y-extent for a splayed boom)
            rear_span = max(
                rear_el[1][0] - rear_el[0][0],
                rear_el[1][1] - rear_el[0][1],
            )
            front_span = max(
                front_el[1][0] - front_el[0][0],
                front_el[1][1] - front_el[0][1],
            )
            ctx.check(
                f"boom {b}: front director shorter than rear element",
                front_span < rear_span - 0.1,
                details=f"rear span={rear_span:.3f}, front span={front_span:.3f}",
            )

    return ctx.report()


object_model = build_object_model()
