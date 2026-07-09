from __future__ import annotations

# Rooftop dipole + whip antenna on a tall vertical mast.
#
# Coordinate convention:
#   - up is +Z; the mast foot sits on the roof at z = 0.
#   - the dipole crossbar runs along the head's local +X axis.
#   - the vertical whip rod rises along +Z from the center hub.
#   - a small circular loop ring of evenly spaced thin rods surrounds
#     the hub in the horizontal plane.
#
# Structure / articulation:
#   - mast (root, static): tall weathered metal pole standing on the roof,
#     with two small standoff mounting brackets partway down.
#   - antenna_head (REVOLUTE about +Z, primary AZIMUTH): a rotation collar /
#     bearing hub clamped to the mast top. Aiming the antenna means swinging
#     the whole head around the vertical mast axis.
#   - dipole_assembly (REVOLUTE about -Y, secondary ELEVATION tilt): the
#     center hub carrying the horizontal dipole crossbar, the vertical whip
#     rod, the loop ring, and a small feedpoint balun box. It can be tilted
#     up/down a little to trim the aim.

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


def _vertical_rod(name: str, length: float, radius: float = 0.0035):
    """A straight thin rod centered at its midpoint, lying along local +Z."""
    return mesh_from_geometry(
        tube_from_spline_points(
            [(0.0, 0.0, -length / 2.0), (0.0, 0.0, 0.0), (0.0, 0.0, length / 2.0)],
            radius=radius,
            samples_per_segment=2,
            radial_segments=8,
            cap_ends=True,
        ),
        name,
    )


def _loop_segment(name: str, seg_len: float, radius: float = 0.003):
    """A short thin rod segment for the circular loop ring, along local +X."""
    return _rod(name, seg_len, radius=radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_dipole_whip_antenna")

    weathered_metal = model.material("weathered_metal", rgba=(0.82, 0.81, 0.80, 1.0))
    bright_alloy = model.material("bright_alloy", rgba=(0.88, 0.89, 0.91, 1.0))
    dark_box = model.material("dark_box", rgba=(0.16, 0.17, 0.19, 1.0))
    clamp_gray = model.material("clamp_gray", rgba=(0.55, 0.57, 0.60, 1.0))
    copper_tone = model.material("copper_tone", rgba=(0.72, 0.55, 0.38, 1.0))

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
    # two small standoff mounting brackets partway down the mast
    for i, bz in enumerate((0.95, 1.95)):
        mast.visual(
            Box((0.14, 0.022, 0.014)),
            origin=Origin(xyz=(0.085, 0.0, bz)),
            material=clamp_gray,
            name=f"standoff_bracket_{i}",
        )
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
    # The azimuth rotation collar/bearing that clamps onto the mast top.
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

    # ========================================================= DIPOLE ASSEMBLY
    # The elevation-tilt child: center hub, horizontal dipole crossbar,
    # vertical whip rod, small loop ring, and a feedpoint balun box.
    dipole_assembly = model.part("dipole_assembly")

    # center hub - small junction cylinder where elements meet
    dipole_assembly.visual(
        Cylinder(radius=0.022, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_box,
        name="center_hub",
    )

    # horizontal dipole crossbar along local +X (pitch axis about Y)
    dipole_len = 0.60
    dipole_assembly.visual(
        _rod("dipole_crossbar", dipole_len, radius=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=bright_alloy,
        name="dipole_crossbar",
    )

    # tall thin vertical whip rod rising from the hub center
    whip_len = 0.85
    dipole_assembly.visual(
        _vertical_rod("vertical_whip", whip_len, radius=0.003),
        origin=Origin(xyz=(0.0, 0.0, whip_len / 2.0)),
        material=copper_tone,
        name="vertical_whip",
    )

    # small feedpoint balun box, seated on top of the hub above the post
    dipole_assembly.visual(
        Box((0.040, 0.035, 0.025)),
        origin=Origin(xyz=(0.030, 0.0, 0.015)),
        material=dark_box,
        name="balun_box",
    )

    # small loop ring of evenly spaced thin rods in the horizontal plane
    # Each rod is a chord between two adjacent octagon vertices on the ring.
    loop_n = 8
    loop_radius = 0.14
    chord_len = 2.0 * loop_radius * math.sin(math.pi / loop_n)
    chord_mid_r = loop_radius * math.cos(math.pi / loop_n)
    # extend segments slightly so adjacent rods overlap at shared vertices
    seg_len = chord_len * 1.08
    for i in range(loop_n):
        mid_angle = 2.0 * math.pi * i / loop_n + math.pi / loop_n
        mx = chord_mid_r * math.cos(mid_angle)
        my = chord_mid_r * math.sin(mid_angle)
        # rod is along local +X; rotate to tangent direction at midpoint
        tang_angle = mid_angle + math.pi / 2.0
        dipole_assembly.visual(
            _loop_segment(f"loop_rod_{i}", seg_len, radius=0.003),
            origin=Origin(xyz=(mx, my, 0.0), rpy=(0.0, 0.0, tang_angle)),
            material=weathered_metal,
            name=f"loop_rod_{i}",
        )

    # thin radial support spokes connecting hub to the loop ring
    # (placed at every other loop-rod angle so each spoke crosses its rod)
    spoke_inner = 0.010   # well inside the hub radius (0.022)
    spoke_outer = 0.135   # past the loop rod chord center
    spoke_len = spoke_outer - spoke_inner
    spoke_mid_r = (spoke_inner + spoke_outer) / 2.0
    for k in range(4):
        rod_i = 2 * k
        spoke_angle = 2.0 * math.pi * rod_i / loop_n + math.pi / loop_n
        sx = spoke_mid_r * math.cos(spoke_angle)
        sy = spoke_mid_r * math.sin(spoke_angle)
        dipole_assembly.visual(
            _rod(f"loop_support_{k}", spoke_len, radius=0.0025),
            origin=Origin(xyz=(sx, sy, 0.0), rpy=(0.0, 0.0, spoke_angle)),
            material=clamp_gray,
            name=f"loop_support_{k}",
        )

    dipole_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=0.20, length=0.90),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
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
    # SECONDARY: elevation tilt of the dipole assembly about the horizontal -Y
    # axis. The crossbar extends along +X from the pivot; -Y axis lifts the +X
    # end upward for positive q.
    model.articulation(
        "elevation_tilt",
        ArticulationType.REVOLUTE,
        parent=antenna_head,
        child=dipole_assembly,
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
    dipole = object_model.get_part("dipole_assembly")
    azimuth = object_model.get_articulation("azimuth_rotation")
    elevation = object_model.get_articulation("elevation_tilt")

    # --- intentional joint-fit overlaps --------------------------------------
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
        dipole,
        elem_a="elevation_post",
        elem_b="center_hub",
        reason="The elevation pivot post seats into the center hub at the tilt joint.",
    )

    # --- base sits on the roof at z ~ 0 (not buried, not floating) -----------
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

    # --- the head assembly sits at the mast top ------------------------------
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
    # spinning azimuth swings the off-axis balun box around in X/Y
    balun_rest = _aabb_center(ctx.part_element_world_aabb(dipole, elem="balun_box"))
    with ctx.pose({azimuth: math.pi / 2.0}):
        balun_spun = _aabb_center(ctx.part_element_world_aabb(dipole, elem="balun_box"))
    ctx.check(
        "azimuth rotation swings the antenna head horizontally",
        abs(balun_spun[0] - balun_rest[0]) > 0.02 or abs(balun_spun[1] - balun_rest[1]) > 0.02,
        details=f"rest={balun_rest}, spun={balun_spun}",
    )

    # --- SECONDARY: elevation is a horizontal-axis revolute joint ------------
    ctx.check(
        "elevation joint is revolute about the horizontal Y axis",
        elevation.articulation_type == ArticulationType.REVOLUTE
        and tuple(elevation.axis) in ((0.0, 1.0, 0.0), (0.0, -1.0, 0.0)),
        details=f"type={elevation.articulation_type}, axis={elevation.axis}",
    )
    # tilting elevation raises the front (+X) end; use the offset balun box
    # as an asymmetric tracking point (crossbar is symmetric about pivot)
    balun_el_rest = _aabb_center(ctx.part_element_world_aabb(dipole, elem="balun_box"))
    with ctx.pose({elevation: 0.35}):
        balun_el_tilted = _aabb_center(ctx.part_element_world_aabb(dipole, elem="balun_box"))
    ctx.check(
        "elevation tilt raises the front of the dipole assembly",
        balun_el_tilted[2] > balun_el_rest[2] + 0.005,
        details=f"rest z={balun_el_rest[2]}, tilted z={balun_el_tilted[2]}",
    )

    # --- dipole crossbar is a single short horizontal element ----------------
    dipole_aabb = ctx.part_element_world_aabb(dipole, elem="dipole_crossbar")
    dipole_span = dipole_aabb[1][0] - dipole_aabb[0][0]  # X span
    ctx.check(
        "dipole crossbar spans ~0.5-0.7m horizontally",
        0.40 < dipole_span < 0.80,
        details=f"dipole span = {dipole_span}",
    )

    # --- vertical whip is tall and thin --------------------------------------
    whip_aabb = ctx.part_element_world_aabb(dipole, elem="vertical_whip")
    whip_height = whip_aabb[1][2] - whip_aabb[0][2]
    ctx.check(
        "vertical whip is tall (>0.6m)",
        whip_height > 0.60,
        details=f"whip height = {whip_height}",
    )

    # --- loop ring of 8 evenly spaced thin rods exists -----------------------
    for i in range(8):
        rod_aabb = ctx.part_element_world_aabb(dipole, elem=f"loop_rod_{i}")
        ctx.check(
            f"loop rod {i} exists in the assembly",
            rod_aabb is not None,
            details=f"loop_rod_{i} aabb = {rod_aabb}",
        )

    # --- the loop ring is roughly circular (spans similar in X and Y) --------
    # check that loop_rod_0 and loop_rod_2 (opposite sides) span the diameter
    rod0_aabb = ctx.part_element_world_aabb(dipole, elem="loop_rod_0")
    rod4_aabb = ctx.part_element_world_aabb(dipole, elem="loop_rod_4")
    if rod0_aabb is not None and rod4_aabb is not None:
        ring_span_x = max(rod0_aabb[1][0], rod4_aabb[1][0]) - min(rod0_aabb[0][0], rod4_aabb[0][0])
        ctx.check(
            "loop ring spans ~0.25m diameter",
            0.18 < ring_span_x < 0.40,
            details=f"ring x span = {ring_span_x}",
        )

    # --- no yagi boom or reflector grid (replaced by dipole+whip) ------------
    visual_names = [v.name for v in dipole.visuals]
    ctx.check(
        "no reflector grid rods in dipole assembly",
        not any("reflector" in n for n in visual_names),
        details=f"visual names = {visual_names}",
    )
    ctx.check(
        "no graduated element rods in dipole assembly",
        not any("element_rod" in n for n in visual_names),
        details=f"visual names = {visual_names}",
    )

    return ctx.report()


object_model = build_object_model()
