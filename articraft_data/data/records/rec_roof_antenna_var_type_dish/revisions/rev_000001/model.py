from __future__ import annotations

# Rooftop parabolic-dish antenna on a tall vertical mast.
#
# Coordinate convention:
#   - up is +Z; the mast foot sits on the roof at z = 0.
#   - the dish face points along the head's local +X (the aiming direction).
#   - the concave parabolic reflector opens toward +X; the feed horn on tripod
#     struts stands off in front of the dish center toward +X.
#
# Structure / articulation:
#   - mast (root, static): tall weathered metal pole standing on the roof, with
#     two small standoff mounting brackets partway down.
#   - antenna_head (REVOLUTE about +Z, primary AZIMUTH): a rotation collar /
#     bearing hub clamped to the mast top. Aiming the antenna means swinging the
#     whole head around the vertical mast axis.
#   - dish_assembly (REVOLUTE about -Y, secondary ELEVATION tilt): the dish
#     reflector with feed horn and stub arm. Tilts up/down to trim elevation.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _parabolic_dish_shell(
    radius: float,
    focal_length: float,
    thickness: float,
    *,
    segments: int = 48,
    n_samples: int = 24,
):
    """Build a thin concave parabolic dish shell via LatheGeometry.

    The lathe axis is Z; the concave opening faces +Z.  Rotate the returned
    mesh to aim the dish along the desired direction.
    """
    outer = []  # back / convex surface (slightly below the parabola)
    inner = []  # front / concave surface (on the parabola)
    for i in range(n_samples + 1):
        r = radius * i / n_samples
        z = r * r / (4.0 * focal_length)
        inner.append((r, z))
        outer.append((r, z - thickness))
    return LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _strut_mesh(name: str, start, end, *, radius: float = 0.003):
    """Thin straight tube between two 3-D points."""
    return mesh_from_geometry(
        tube_from_spline_points(
            [tuple(start), tuple(end)],
            radius=radius,
            samples_per_segment=2,
            radial_segments=8,
            cap_ends=True,
        ),
        name,
    )


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_dish_antenna")

    weathered_metal = model.material("weathered_metal", rgba=(0.82, 0.81, 0.80, 1.0))
    bright_alloy = model.material("bright_alloy", rgba=(0.88, 0.89, 0.91, 1.0))
    dark_box = model.material("dark_box", rgba=(0.16, 0.17, 0.19, 1.0))
    clamp_gray = model.material("clamp_gray", rgba=(0.55, 0.57, 0.60, 1.0))
    dish_white = model.material("dish_white", rgba=(0.90, 0.91, 0.89, 1.0))
    feed_dark = model.material("feed_dark", rgba=(0.20, 0.21, 0.23, 1.0))

    # ------------------------------------------------------------------ scales
    mast_len = 3.40          # visible mast height
    mast_r = 0.016           # ~32 mm OD steel mast
    head_z = mast_len - 0.05  # the rotation collar sits just below the mast top

    # Dish parameters
    dish_radius = 0.38       # 76 cm diameter dish
    focal_length = 0.22      # focal length
    dish_depth = dish_radius * dish_radius / (4.0 * focal_length)
    shell_t = 0.003          # 3 mm shell thickness
    stub_arm_len = 0.14      # horizontal arm from pivot to dish back

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

    # =========================================================== DISH ASSEMBLY
    # The dish reflector + feed horn + stub arm ride together as the elevation
    # child. The dish face points along local +X. The part origin is the
    # elevation pivot point.
    dish_assembly = model.part("dish_assembly")

    # ---- stub arm: horizontal bar from elevation pivot to dish back ----------
    dish_assembly.visual(
        Box((stub_arm_len, 0.026, 0.026)),
        origin=Origin(xyz=(stub_arm_len / 2.0, 0.0, 0.0)),
        material=bright_alloy,
        name="stub_arm",
    )
    # small pivot bracket wrapping around the elevation post top
    dish_assembly.visual(
        Box((0.044, 0.034, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=clamp_gray,
        name="pivot_bracket",
    )

    # ---- parabolic dish reflector (lathe shell) -----------------------------
    dish_shell = _parabolic_dish_shell(
        dish_radius, focal_length, shell_t, segments=48, n_samples=24,
    )
    # The lathe axis is Z with the opening facing +Z.
    # Rotate +90° around Y so the opening faces +X (the aiming direction).
    dish_shell.rotate((0.0, 1.0, 0.0), math.pi / 2.0)
    # Position so the dish vertex (back) sits at the end of the stub arm.
    dish_shell.translate(stub_arm_len, 0.0, 0.0)
    dish_assembly.visual(
        mesh_from_geometry(dish_shell, "dish_reflector"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dish_white,
        name="dish_reflector",
    )

    # ---- rim ring around the dish edge --------------------------------------
    # After rotation, the rim circle lies in the YZ plane at x = stub_arm_len + dish_depth.
    rim_x = stub_arm_len + dish_depth
    rim_geom = TorusGeometry(
        radius=dish_radius, tube=0.005,
        radial_segments=8, tubular_segments=48,
    )
    # Torus ring is in XY plane by default; rotate so ring is in YZ plane.
    rim_geom.rotate((0.0, 1.0, 0.0), math.pi / 2.0)
    rim_geom.translate(rim_x, 0.0, 0.0)
    dish_assembly.visual(
        mesh_from_geometry(rim_geom, "dish_rim"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=bright_alloy,
        name="dish_rim",
    )

    # ---- feed horn at focal point -------------------------------------------
    # The focal point is at x = stub_arm_len + focal_length, centered on axis.
    focal_x = stub_arm_len + focal_length
    feed_horn_len = 0.045
    feed_horn_r = 0.016
    # Feed horn is a small cylinder pointing back toward the dish center.
    # CylinderGeometry is centered, extends along local Z.
    feed_geom = CylinderGeometry(radius=feed_horn_r, height=feed_horn_len, radial_segments=16, closed=True)
    # Rotate so the cylinder axis aligns with X (pointing along the dish axis).
    feed_geom.rotate((0.0, 1.0, 0.0), math.pi / 2.0)
    feed_geom.translate(focal_x, 0.0, 0.0)
    dish_assembly.visual(
        mesh_from_geometry(feed_geom, "feed_horn"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=feed_dark,
        name="feed_horn",
    )

    # ---- tripod support struts from dish rim to feed horn -------------------
    # Three struts at 120° spacing, each running from a rim point to the
    # feed horn center.
    for i in range(3):
        theta = 2.0 * math.pi * i / 3.0
        # Rim point in dish-local coords (after rotation to +X):
        #   x = rim_x,  y = dish_radius * sin(theta),  z = -dish_radius * cos(theta)
        rim_pt = (
            rim_x,
            dish_radius * math.sin(theta),
            -dish_radius * math.cos(theta),
        )
        focal_pt = (focal_x, 0.0, 0.0)
        strut = _strut_mesh(
            f"support_strut_{i}",
            rim_pt,
            focal_pt,
            radius=0.003,
        )
        dish_assembly.visual(
            strut,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=bright_alloy,
            name=f"support_strut_{i}",
        )

    # ---- small junction box on the stub arm ---------------------------------
    dish_assembly.visual(
        Box((0.055, 0.040, 0.035)),
        origin=Origin(xyz=(stub_arm_len * 0.4, 0.0, -0.025)),
        material=dark_box,
        name="junction_box",
    )

    dish_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=dish_radius, length=dish_depth + focal_length + stub_arm_len),
        mass=3.5,
        origin=Origin(xyz=(stub_arm_len + dish_depth / 2.0, 0.0, 0.0)),
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
    # SECONDARY: elevation tilt of the dish assembly about the horizontal -Y axis.
    # Dish extends along local +X from the pivot; -Y axis lifts the front (+X)
    # end upward for positive q, so the dish can be aimed slightly up/down.
    model.articulation(
        "elevation_tilt",
        ArticulationType.REVOLUTE,
        parent=antenna_head,
        child=dish_assembly,
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


# ---------------------------------------------------------------------------
# Helpers for tests
# ---------------------------------------------------------------------------

def _aabb_center(aabb):
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    mast = object_model.get_part("mast")
    head = object_model.get_part("antenna_head")
    dish = object_model.get_part("dish_assembly")
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
        dish,
        elem_a="elevation_post",
        elem_b="pivot_bracket",
        reason="The elevation pivot post seats into the pivot bracket at the tilt joint.",
    )
    ctx.allow_overlap(
        head,
        dish,
        elem_a="elevation_post",
        elem_b="stub_arm",
        reason="The stub arm rests on the elevation pivot post at the tilt joint; the seated fit is intentional.",
    )
    # Proof: the stub arm contacts the elevation post at the pivot
    ctx.expect_contact(
        head,
        dish,
        elem_a="elevation_post",
        elem_b="stub_arm",
        contact_tol=0.015,
        name="stub arm seated on elevation post at pivot",
    )

    # --- base sits on the roof at z ~ 0 --------------------------------------
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

    # --- the head/dish assembly sits at the mast top -------------------------
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
    # spinning azimuth should swing the dish around in X/Y
    dish_rest = _aabb_center(ctx.part_element_world_aabb(dish, elem="dish_reflector"))
    with ctx.pose({azimuth: math.pi / 2.0}):
        dish_spun = _aabb_center(ctx.part_element_world_aabb(dish, elem="dish_reflector"))
    ctx.check(
        "azimuth rotation swings the dish head horizontally",
        abs(dish_spun[0] - dish_rest[0]) > 0.05 or abs(dish_spun[1] - dish_rest[1]) > 0.05,
        details=f"rest={dish_rest}, spun={dish_spun}",
    )

    # --- SECONDARY: elevation is a horizontal-axis revolute joint ------------
    ctx.check(
        "elevation joint is revolute about the horizontal Y axis",
        elevation.articulation_type == ArticulationType.REVOLUTE
        and tuple(elevation.axis) in ((0.0, 1.0, 0.0), (0.0, -1.0, 0.0)),
        details=f"type={elevation.articulation_type}, axis={elevation.axis}",
    )
    # tilting elevation up should raise the dish front (feed horn)
    feed_rest = _aabb_center(ctx.part_element_world_aabb(dish, elem="feed_horn"))
    with ctx.pose({elevation: 0.35}):
        feed_up = _aabb_center(ctx.part_element_world_aabb(dish, elem="feed_horn"))
    ctx.check(
        "elevation tilt raises the front of the dish",
        feed_up[2] > feed_rest[2] + 0.02,
        details=f"rest feed z={feed_rest[2]}, tilted feed z={feed_up[2]}",
    )

    # --- dish reflector is a curved surface (not flat box) -------------------
    dish_aabb = ctx.part_element_world_aabb(dish, elem="dish_reflector")
    ctx.check(
        "dish reflector has significant size",
        dish_aabb is not None
        and (dish_aabb[1][1] - dish_aabb[0][1]) > 0.5
        and (dish_aabb[1][2] - dish_aabb[0][2]) > 0.5,
        details="dish Y and Z spans should exceed 0.5 m (76 cm diameter)",
    )

    # --- feed horn is in front of the dish (+X direction) -------------------
    dish_center = _aabb_center(ctx.part_element_world_aabb(dish, elem="dish_reflector"))
    feed_center = _aabb_center(ctx.part_element_world_aabb(dish, elem="feed_horn"))
    ctx.check(
        "feed horn is in front of the dish along the aiming axis",
        feed_center[0] > dish_center[0] + 0.05,
        details=f"dish center x={dish_center[0]}, feed center x={feed_center[0]}",
    )

    # --- support struts connect the rim to the feed horn ---------------------
    for i in range(3):
        strut_aabb = ctx.part_element_world_aabb(dish, elem=f"support_strut_{i}")
        ctx.check(
            f"support_strut_{i} exists and has length",
            strut_aabb is not None
            and max(
                strut_aabb[1][0] - strut_aabb[0][0],
                strut_aabb[1][1] - strut_aabb[0][1],
                strut_aabb[1][2] - strut_aabb[0][2],
            ) > 0.10,
            details=f"strut_{i} max span should exceed 0.10 m",
        )

    return ctx.report()


object_model = build_object_model()
