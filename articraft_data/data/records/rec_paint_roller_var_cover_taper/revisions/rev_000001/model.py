from __future__ import annotations

# Articraft model: a small/mini paint roller.
#
# Real object (from reference image):
#   - A cream/white cylindrical roller cover (foam over a hard core), with
#     visibly hollow open ends.
#   - A gray bent-wire steel frame ("crank cage"): one long axle segment runs
#     through the roller core, bends down ~90deg at the handle end, then runs
#     out to a pink plastic handle.
#   - A pink/coral molded plastic grip handle that the wire socket plugs into.
#
# Primary mechanism: the roller cover free-spins on the wire axle. That is a
# CONTINUOUS revolute joint about the roller's long axis (here world +X).
#
# Frame convention: roller long axis = X, roller centered at x=0. The roller's
# free (far) end is toward -X. The wire bends down at the +X end and the pink
# handle extends further in +X, away from the roller -- matching the image.
# The wire frame and the pink handle form one rigid root body. The roller cover
# is the single moving child, captured on the axle.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    CylinderGeometry,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ----------------------------------------------------------------------------
# Real-world dimensions (meters)
# ----------------------------------------------------------------------------
ROLLER_LEN = 0.180          # roller cover cylindrical length (along X)
ROLLER_OUTER_R = 0.0190     # roller cover outer radius (~38 mm dia)
ROLLER_BORE_R = 0.0078      # inner core bore radius (visible hollow ends)
TAPER_LEN = 0.025           # length of the feather taper at each end
TIP_R = 0.005               # small rounded radius at each feathered tip

AXLE_R = 0.0028             # steel wire radius (~5.6 mm wire)

ROLLER_X_MIN = -ROLLER_LEN / 2.0   # free (far) end, toward -X
ROLLER_X_MAX = ROLLER_LEN / 2.0    # handle-side end, toward +X

# The crank geometry. At the +X end the axle clears the roller, bends down by
# FRAME_DROP, then the handle continues further out in +X.
FRAME_DROP = 0.055          # vertical drop of the crank (Z)
HANDLE_LEN = 0.130          # pink grip length (along X)

AXLE_Z = 0.0                       # roller axle line height
HANDLE_Z = AXLE_Z - FRAME_DROP     # handle axis height (below the axle)

# Key X stations of the crank.
AXLE_FAR_X = ROLLER_X_MIN - 0.006          # capped stub past the far end
AXLE_NEAR_X = ROLLER_X_MAX + 0.012         # axle clears the +X roller face
BEND_X = AXLE_NEAR_X + 0.006               # top corner of the crank
HANDLE_TOP_X = BEND_X + 0.030              # where the wire meets the grip top
SOCKET_X = HANDLE_TOP_X                    # grip top (collar) X

CREAM = Material(name="roller_cover_cream", rgba=(0.93, 0.91, 0.84, 1.0))
STEEL = Material(name="frame_steel", rgba=(0.62, 0.63, 0.65, 1.0))
CORAL = Material(name="handle_coral", rgba=(0.86, 0.45, 0.43, 1.0))
END_CAP = Material(name="roller_endcap", rgba=(0.80, 0.78, 0.72, 1.0))


def _smoothstep(t: float) -> float:
    """Hermite smoothstep: f(0)=0 f'(0)=0, f(1)=1 f'(1)=0."""
    return t * t * (3.0 - 2.0 * t)


def _roller_cover_shape():
    """Edge-roller style cover: a lathed body whose cylindrical middle narrows
    to rounded coned (feathered) tips at both ends.  Built around the lathe Z
    axis, then rotated so the roller long axis aligns with world +X."""
    R = ROLLER_OUTER_R
    z_min = -ROLLER_LEN / 2.0          # left (far-end) tip
    z_max = ROLLER_LEN / 2.0           # right (handle-end) tip
    z_sl = z_min + TAPER_LEN           # left shoulder
    z_sr = z_max - TAPER_LEN           # right shoulder

    # --- Lathe profile: (radius, z) ----------------------------------------
    profile: list[tuple[float, float]] = []

    # Left feathered tip → shoulder (radius grows from TIP_R to R)
    n_taper = 12
    profile.append((TIP_R, z_min))
    for i in range(1, n_taper + 1):
        t = i / n_taper
        z = z_min + t * TAPER_LEN
        r = TIP_R + _smoothstep(t) * (R - TIP_R)
        profile.append((r, z))

    # Cylindrical middle (single midpoint keeps the lathe well-conditioned)
    profile.append((R, (z_sl + z_sr) / 2.0))

    # Right shoulder → feathered tip (mirror of left)
    for i in range(n_taper):
        t = i / n_taper
        z = z_sr + t * TAPER_LEN
        r = TIP_R + _smoothstep(1.0 - t) * (R - TIP_R)
        profile.append((r, z))
    profile.append((TIP_R, z_max))

    outer = LatheGeometry(profile, segments=56)

    # --- Bore: narrow channel full-length + wide bore in the middle -----------
    # Narrow bore through full length so the axle wire clears the solid tips.
    wire_clearance_r = AXLE_R + 0.0008      # 0.0036 < TIP_R (0.005)
    narrow_bore = CylinderGeometry(
        radius=wire_clearance_r, height=ROLLER_LEN + 0.004,
    )
    cover = boolean_difference(outer, narrow_bore)

    # Wide bore through the cylindrical middle (core tube seating).
    wide_bore_h = (z_sr - z_sl) + 0.008     # slight overshoot past shoulders
    wide_bore = CylinderGeometry(radius=ROLLER_BORE_R, height=wide_bore_h)
    cover = boolean_difference(cover, wide_bore)

    cover.rotate_y(math.pi / 2.0)      # lathe Z → world X
    return mesh_from_geometry(cover, "roller_cover")


def _core_tube_shape():
    """Thin rigid inner core (hard plastic sleeve) bonded inside the cover
    bore.  Its outer wall contacts the cover bore (so it is not a floating
    island) and its inner bore is the journal that runs on the steel axle.
    Shorter than the full roller because the feathered tips are solid."""
    core_outer_r = ROLLER_BORE_R + 0.0003   # press fit into the cover bore
    core_inner_r = AXLE_R - 0.0006          # captures the steel axle
    core_h = ROLLER_LEN - 2.0 * TAPER_LEN - 0.010   # fits inside the bore

    outer = CylinderGeometry(radius=core_outer_r, height=core_h)
    inner = CylinderGeometry(radius=core_inner_r, height=core_h + 0.004)
    core = boolean_difference(outer, inner)
    core.rotate_y(math.pi / 2.0)           # Z → X
    return mesh_from_geometry(core, "roller_core")


def _frame_wire_path() -> list[tuple[float, float, float]]:
    """Centerline of the bent steel wire, world coords: from the far axle stub,
    through the roller, up over the +X end, down the crank, into the handle."""
    return [
        (AXLE_FAR_X, 0.0, AXLE_Z),                 # capped stub, far end
        (ROLLER_X_MIN, 0.0, AXLE_Z),
        (0.0, 0.0, AXLE_Z),                        # mid axle (through roller)
        (ROLLER_X_MAX, 0.0, AXLE_Z),
        (AXLE_NEAR_X, 0.0, AXLE_Z),                # axle clears the +X face
        (BEND_X, 0.0, AXLE_Z - 0.006),             # round into the down bend
        (BEND_X + 0.006, 0.0, AXLE_Z - 0.030),     # diagonal down the crank
        (HANDLE_TOP_X - 0.012, 0.0, HANDLE_Z + 0.004),
        (HANDLE_TOP_X, 0.0, HANDLE_Z),             # arrives on the handle axis
        (HANDLE_TOP_X + 0.022, 0.0, HANDLE_Z),     # plunges into the grip socket
    ]


def _frame_wire_mesh():
    """Build the bent wire as one smooth swept tube along its 3D path."""
    geom = tube_from_spline_points(
        _frame_wire_path(),
        radius=AXLE_R,
        samples_per_segment=14,
        radial_segments=16,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, "wire_frame")


def _handle_shape() -> cq.Workplane:
    """Pink molded grip: a rounded, gently barrel-tapered body, revolved about
    the handle axis (X). The wire enters the collar (+X end of the assembly is
    the grip top); the grip tapers down to a rounded toe further out in +X."""
    x0 = SOCKET_X                 # collar / top, where the wire plugs in
    x1 = SOCKET_X + HANDLE_LEN    # rounded toe, far from the roller
    pts = [
        (x0, 0.0),
        (x0, 0.0085),             # collar neck (wire socket)
        (x0 + 0.006, 0.0115),     # flare below the collar
        (x0 + 0.020, 0.0135),
        (x0 + 0.058, 0.0140),     # belly
        (x0 + 0.100, 0.0122),
        (x1 - 0.012, 0.0095),
        (x1 - 0.004, 0.0052),     # rounded toe
        (x1, 0.0),
    ]
    prof = cq.Workplane("XZ").polyline(pts).close()
    handle = prof.revolve(360.0, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    handle = handle.translate((0.0, 0.0, HANDLE_Z))
    return handle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="paint_roller")
    model.materials.extend([CREAM, STEEL, CORAL, END_CAP])

    # --- Root: pink handle + fixed steel wire frame/cage ---------------------
    frame = model.part("handle_frame")
    frame.visual(
        mesh_from_cadquery(_handle_shape(), "handle_grip"),
        material=CORAL,
        name="handle_grip",
    )
    frame.visual(
        _frame_wire_mesh(),
        material=STEEL,
        name="wire_frame",
    )

    # --- Moving child: roller cover (spins on the axle) ----------------------
    roller = model.part("roller_cover")
    roller.visual(
        _roller_cover_shape(),
        material=CREAM,
        name="roller_cover",
    )
    roller.visual(
        _core_tube_shape(),
        material=END_CAP,
        name="roller_core",
    )

    # The roller spins freely about the axle (world +X). Joint frame at the
    # roller center, on the axle line.
    model.articulation(
        "frame_to_roller",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=roller,
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("handle_frame")
    roller = object_model.get_part("roller_cover")
    spin = object_model.get_articulation("frame_to_roller")

    # The axle wire passes through the roller core bore: an intentional nested
    # capture fit (axle journaled inside the rotating core). This is also the
    # real support path that keeps the spinning roller attached to the grounded
    # frame, so scope the overlap allowance to exactly that element pair.
    ctx.allow_overlap(
        frame,
        roller,
        elem_a="wire_frame",
        elem_b="roller_core",
        reason=(
            "The steel axle wire is intentionally captured inside the roller "
            "core bore; the cover spins on it as a journal bearing."
        ),
    )

    # Prove the capture: the axle wire and the roller core share the axle line
    # (this is the allowed overlap and the real support path).
    ctx.expect_overlap(
        frame,
        roller,
        axes="x",
        elem_a="wire_frame",
        elem_b="roller_core",
        min_overlap=0.10,
        name="axle wire runs through the roller core",
    )

    # --- Joint type / axis is the real spin mechanism ------------------------
    ctx.check(
        "roller joint is continuous",
        spin.joint_type == ArticulationType.CONTINUOUS,
        details=f"got {spin.joint_type}",
    )
    ctx.check(
        "roller spins about its long X axis",
        abs(spin.axis[0]) > 0.99
        and abs(spin.axis[1]) < 0.01
        and abs(spin.axis[2]) < 0.01,
        details=f"axis={spin.axis}",
    )

    # --- Roller cover is a long tapered body centered on the axle ------------
    cover = roller.get_visual("roller_cover")
    aabb = ctx.part_element_world_aabb(roller, elem=cover)
    assert aabb is not None
    (xmin, ymin, zmin), (xmax, ymax, zmax) = aabb
    length_x = xmax - xmin
    dia_y = ymax - ymin
    dia_z = zmax - zmin
    ctx.check(
        "roller cover reads as a long tapered body",
        length_x > 0.16 and 0.034 < dia_y < 0.042 and 0.034 < dia_z < 0.042,
        details=f"len_x={length_x:.3f}, dia_y={dia_y:.3f}, dia_z={dia_z:.3f}",
    )
    ctx.check(
        "roller cover has feathered taper (bore < outer radius)",
        ROLLER_BORE_R < ROLLER_OUTER_R - 0.006 and TAPER_LEN > 0.010,
        details=f"outer_r={ROLLER_OUTER_R}, bore_r={ROLLER_BORE_R}, taper={TAPER_LEN}",
    )

    # Core sits inside the cover bore and contacts the cover wall (so it is not
    # a floating island within the roller part).
    ctx.expect_contact(
        roller,
        roller,
        elem_a="roller_core",
        elem_b="roller_cover",
        contact_tol=0.0006,
        name="roller core seated in cover bore",
    )

    # --- Wire frame spans the roller and drops to the handle -----------------
    wire = frame.get_visual("wire_frame")
    wbb = ctx.part_element_world_aabb(frame, elem=wire)
    assert wbb is not None
    (wxmin, _wy0, wzmin), (wxmax, _wy1, wzmax) = wbb
    ctx.check(
        "wire frame spans the roller and drops to the handle",
        wxmin <= ROLLER_X_MIN + 0.002
        and wxmax >= ROLLER_X_MAX
        and (wzmax - wzmin) > FRAME_DROP * 0.7,
        details=f"wire x[{wxmin:.3f},{wxmax:.3f}] z[{wzmin:.3f},{wzmax:.3f}]",
    )

    # --- Handle grip is below the axle, away from the roller, connected ------
    grip = frame.get_visual("handle_grip")
    gbb = ctx.part_element_world_aabb(frame, elem=grip)
    assert gbb is not None
    (gxmin, _gy0, gzmin), (gxmax, _gy1, gzmax) = gbb
    ctx.check(
        "handle grip sits below the roller axle",
        gzmax < AXLE_Z - 0.02,
        details=f"grip z[{gzmin:.3f},{gzmax:.3f}], axle_z={AXLE_Z}",
    )
    ctx.check(
        "handle grip extends out beyond the roller (+X side)",
        gxmin > ROLLER_X_MAX,
        details=f"grip xmin={gxmin:.3f}, roller xmax={ROLLER_X_MAX:.3f}",
    )
    # The wire and grip are one rigid body; the wire socket must reach into the
    # grip so the grip is not a floating island.
    ctx.expect_contact(
        frame,
        frame,
        elem_a="wire_frame",
        elem_b="handle_grip",
        contact_tol=1e-6,
        name="wire frame plugs into handle grip",
    )

    # --- The roller actually spins: the joint is poseable and stays centered -
    with ctx.pose({spin: 0.0}):
        aabb0 = ctx.part_world_aabb(roller)
    with ctx.pose({spin: math.pi / 2.0}):
        aabb1 = ctx.part_world_aabb(roller)
    assert aabb0 is not None and aabb1 is not None
    centered = (
        abs((aabb1[0][1] + aabb1[1][1]) / 2.0) < 0.002
        and abs((aabb1[0][2] + aabb1[1][2]) / 2.0 - AXLE_Z) < 0.002
    )
    ctx.check(
        "roller stays centered on the axle through a quarter spin",
        centered,
        details=f"spun aabb={aabb1}",
    )

    # Roller cover must clear the pink handle grip (no collision between them).
    ctx.expect_gap(
        frame,
        roller,
        axis="x",
        positive_elem="handle_grip",
        negative_elem="roller_cover",
        min_gap=0.0,
        name="handle grip clears the roller cover",
    )

    return ctx.report()


object_model = build_object_model()
