from __future__ import annotations

# Rooftop yagi / TV antenna on a tall vertical mast.
#
# Coordinate convention:
#   - up is +Z; the mast foot sits on the roof at z = 0.
#   - the yagi boom runs along the head's local +X (the long horizontal element-
#     carrying spine that points toward the broadcast tower).
#   - the wide flat reflector grid sits at the rear (-X) of the boom; the
#     graduated dipole/director rods straddle the boom left/right along +/-Y and
#     get shorter toward the front.
#
# Structure / articulation:
#   - mast (root, static): tall weathered metal pole standing on the roof, with
#     two small standoff mounting brackets partway down.
#   - antenna_head (REVOLUTE about +Z, primary AZIMUTH): a rotation collar /
#     bearing hub clamped to the mast top. Aiming the antenna means swinging the
#     whole head around the vertical mast axis.
#   - yagi_boom (REVOLUTE about +Y, secondary ELEVATION tilt): the horizontal
#     boom carrying the reflector grid, the dipole + director rod elements, and
#     two small junction / balun boxes. It can be tilted up/down a little to
#     trim the aim.

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

    # ================================================================ YAGI BOOM
    # Boom + every rod element + reflector grid + junction boxes ride together as
    # the elevation child. The boom long axis is local +X. The boom local origin
    # is the elevation pivot (at the elevation_post top, slightly behind the
    # boom's geometric midpoint toward the reflector so the long front overhangs).
    yagi_boom = model.part("yagi_boom")

    boom_len = 1.55
    boom_back = -0.45        # reflector end (rear)
    boom_front = boom_len + boom_back  # director end (front), = 1.10
    boom_mid = (boom_front + boom_back) / 2.0
    boom_z = 0.0             # boom centerline at the pivot height

    # main boom: a slim square-section aluminium spine
    yagi_boom.visual(
        Box((boom_len, 0.026, 0.026)),
        origin=Origin(xyz=(boom_mid, 0.0, boom_z)),
        material=bright_alloy,
        name="boom_spine",
    )

    # ---- graduated transverse rod elements (driven dipole + directors) -------
    # Real yagi: a long reflector area at the back, then a driven element, then
    # directors that shorten toward the front. We place them as full-width rods
    # crossing the boom along +/-Y. Lengths taper linearly down toward the front.
    n_elements = 14
    element_start_x = boom_back + 0.10      # first element near the reflector
    element_end_x = boom_back + 1.42        # last element near the boom front
    longest_element = 0.86                  # rear element full length
    shortest_element = 0.28                 # front director full length
    for i in range(n_elements):
        frac = i / (n_elements - 1)
        ex = element_start_x + frac * (element_end_x - element_start_x)
        elen = longest_element - frac * (longest_element - shortest_element)
        rod = _rod(f"element_rod_{i:02d}", elen, radius=0.0035)
        yagi_boom.visual(
            rod,
            # rods cross the boom: rotate the local-+X rod onto +/-Y
            origin=Origin(xyz=(ex, 0.0, boom_z), rpy=(0.0, 0.0, math.pi / 2.0)),
            material=bright_alloy,
            name=f"element_rod_{i:02d}",
        )

    # ---- two small junction / balun boxes seated on the boom -----------------
    yagi_boom.visual(
        Box((0.070, 0.050, 0.045)),
        origin=Origin(xyz=(boom_back + 0.20, 0.0, boom_z + 0.030), rpy=(0.0, 0.0, 0.0)),
        material=dark_box,
        name="balun_box_rear",
    )
    yagi_boom.visual(
        Box((0.060, 0.045, 0.040)),
        origin=Origin(xyz=(boom_back + 0.50, 0.0, boom_z + 0.028)),
        material=dark_box,
        name="junction_box_front",
    )

    # ---- wide flat reflector grid at the rear of the boom --------------------
    # A vertical screen of many parallel thin horizontal rods held by two short
    # vertical end stiles. It sits at the boom rear and reads as a flat panel.
    refl_x = boom_back - 0.02         # just behind the rearmost element
    refl_half_h = 0.62               # screen total height ~1.24 m
    refl_half_w = 0.36               # screen half-width in Y (each side of boom)
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
    # a short connecting strut tying the reflector screen back to the boom end.
    # It must physically bridge the boom spine (front) and the grid plane (rear)
    # so the screen is not a floating island.
    strut_front = boom_back + 0.06     # overlaps the boom spine rear end
    strut_rear = refl_x - 0.004        # reaches just past the grid plane
    yagi_boom.visual(
        Box((strut_front - strut_rear, 0.022, 0.022)),
        origin=Origin(xyz=((strut_front + strut_rear) / 2.0, 0.0, boom_z)),
        material=bright_alloy,
        name="reflector_strut",
    )

    yagi_boom.inertial = Inertial.from_geometry(
        Box((boom_len, 0.90, 1.30)),
        mass=2.4,
        origin=Origin(xyz=(boom_mid, 0.0, boom_z)),
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
    # SECONDARY: elevation tilt of the boom about the horizontal -Y axis.
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
    # pivot boss seated into the boom spine. Both are genuine mechanical fits.
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
        elem_b="boom_spine",
        reason="The elevation pivot post is seated into the boom spine at the tilt joint.",
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

    # --- dense director array: 14 transverse rod elements on the boom ---------
    element_names = [f"element_rod_{i:02d}" for i in range(14)]
    element_count = sum(
        1 for v in boom.visuals if v.name.startswith("element_rod_")
    )
    ctx.check(
        "boom carries 14 transverse dipole/director rod elements",
        element_count == 14,
        details=f"found {element_count} element_rod visuals",
    )

    # --- SECONDARY: elevation is a horizontal-axis revolute joint ------------
    ctx.check(
        "elevation joint is revolute about the horizontal Y axis",
        elevation.articulation_type == ArticulationType.REVOLUTE
        and tuple(elevation.axis) in ((0.0, 1.0, 0.0), (0.0, -1.0, 0.0)),
        details=f"type={elevation.articulation_type}, axis={elevation.axis}",
    )
    # tilting elevation up should raise the front director tip
    front_rest = _aabb_center(ctx.part_element_world_aabb(boom, elem="element_rod_13"))
    with ctx.pose({elevation: 0.35}):
        front_up = _aabb_center(ctx.part_element_world_aabb(boom, elem="element_rod_13"))
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
    refl_strut_aabb = ctx.part_element_world_aabb(boom, elem="reflector_strut")
    front_dir_aabb = ctx.part_element_world_aabb(boom, elem="element_rod_13")
    ctx.check(
        "reflector grid is at the rear of the boom",
        _aabb_center(refl_strut_aabb)[0] < _aabb_center(front_dir_aabb)[0],
        details=f"reflector x={_aabb_center(refl_strut_aabb)[0]}, front x={_aabb_center(front_dir_aabb)[0]}",
    )

    # --- director elements taper shorter toward the front --------------------
    rear_el = ctx.part_element_world_aabb(boom, elem="element_rod_00")  # rear (longest)
    front_el = ctx.part_element_world_aabb(boom, elem="element_rod_13")  # front (shortest)
    rear_span = rear_el[1][1] - rear_el[0][1]
    front_span = front_el[1][1] - front_el[0][1]
    ctx.check(
        "front director element is shorter than rear elements",
        front_span < rear_span - 0.1,
        details=f"rear span={rear_span}, front span={front_span}",
    )

    return ctx.report()


object_model = build_object_model()
