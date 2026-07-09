from __future__ import annotations

# Articraft model: a small/mini paint roller — extension-pole socket variant.
#
# Real object (from reference image):
#   - A cream/white cylindrical roller cover (foam over a hard core), with
#     visibly hollow open ends.
#   - A gray bent-wire steel frame ("crank cage"): one long axle segment runs
#     through the roller core, bends down ~90deg at the handle end, then runs
#     out to a pink plastic handle.
#   - A pink/coral molded plastic grip handle that the wire socket plugs into.
#
# VARIANT: the grip butt end has an open female-threaded socket for an
# extension pole. The same molded handle but with a visible internal-thread
# receptacle bored into the far (+X) end.
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
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
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

# --- Extension pole socket (butt end of grip) --------------------------------
SOCKET_X1 = SOCKET_X + HANDLE_LEN          # butt-end face X
BORE_R = 0.0080                            # bore radius (~16 mm dia, std pole)
BORE_DEPTH = 0.022                         # bore depth into the grip
THREAD_TUBE_R = 0.0009                     # thread ridge cross-section radius
THREAD_MAJOR_R = BORE_R - THREAD_TUBE_R * 0.5  # ridge center radius (overlap w/ wall)
THREAD_PITCH = 0.0035                      # axial advance per turn
NUM_THREAD_TURNS = 6                       # visible thread turns
THREAD_ARC_DEG = 340.0                     # partial turn arc (shows helix gap)

CREAM = Material(name="roller_cover_cream", rgba=(0.93, 0.91, 0.84, 1.0))
STEEL = Material(name="frame_steel", rgba=(0.62, 0.63, 0.65, 1.0))
CORAL = Material(name="handle_coral", rgba=(0.86, 0.45, 0.43, 1.0))
THREAD_CORAL = Material(name="thread_coral", rgba=(0.79, 0.40, 0.38, 1.0))
END_CAP = Material(name="roller_endcap", rgba=(0.80, 0.78, 0.72, 1.0))


def _roller_cover_shape() -> cq.Workplane:
    """Hollow cream roller cover: an open-ended tube with a real bore so the
    ends read as hollow."""
    outer = (
        cq.Workplane("YZ")
        .circle(ROLLER_OUTER_R)
        .extrude(ROLLER_LEN)
        .translate((ROLLER_X_MIN, 0.0, 0.0))
    )
    bore = (
        cq.Workplane("YZ")
        .circle(ROLLER_BORE_R)
        .extrude(ROLLER_LEN + 0.006)
        .translate((ROLLER_X_MIN - 0.003, 0.0, 0.0))
    )
    cover = outer.cut(bore)
    # Soften the outer rim edges so the fabric ends read rounded.
    try:
        cover = cover.edges("%CIRCLE").fillet(0.0020)
    except Exception:
        pass
    return cover


def _core_tube_shape() -> cq.Workplane:
    """Thin rigid inner core (hard plastic sleeve) bonded inside the cover bore.
    Its outer wall contacts the cover bore (so it is not a floating island) and
    its inner bore is the journal that runs on the steel axle."""
    core_outer_r = ROLLER_BORE_R + 0.0003  # slight press fit into the cover bore
    # Bore slightly smaller than the axle so the journal positively captures the
    # steel axle (the intentional, allowed wire<->core overlap). This contact is
    # also the support path that keeps the spinning roller attached.
    core_inner_r = AXLE_R - 0.0006
    outer = (
        cq.Workplane("YZ")
        .circle(core_outer_r)
        .extrude(ROLLER_LEN - 0.008)
        .translate((ROLLER_X_MIN + 0.004, 0.0, 0.0))
    )
    bore = (
        cq.Workplane("YZ")
        .circle(core_inner_r)
        .extrude(ROLLER_LEN)
        .translate((ROLLER_X_MIN - 0.001, 0.0, 0.0))
    )
    return outer.cut(bore)


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
    the grip top); the grip tapers down to a rounded toe further out in +X.

    VARIANT: the butt end (far +X) has a flat face with a cylindrical bore
    for an extension pole socket, replacing the rounded toe.
    """
    x0 = SOCKET_X                 # collar / top, where the wire plugs in
    x1 = SOCKET_X1               # butt end, flat face with bore
    pts = [
        (x0, 0.0),
        (x0, 0.0085),             # collar neck (wire socket)
        (x0 + 0.006, 0.0115),     # flare below the collar
        (x0 + 0.020, 0.0135),
        (x0 + 0.058, 0.0140),     # belly
        (x0 + 0.088, 0.0136),     # slight taper after belly
        (x0 + 0.105, 0.0144),     # socket boss widens for bore wall
        (x1 - 0.003, 0.0150),     # end rim
        (x1, 0.0150),             # flat end face (outer edge)
        (x1, 0.0),                # flat end face (center, on revolve axis)
    ]
    prof = cq.Workplane("XZ").polyline(pts).close()
    handle = prof.revolve(360.0, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    handle = handle.translate((0.0, 0.0, HANDLE_Z))

    # Cut the extension-pole bore into the butt end.
    bore_cutter = (
        cq.Workplane("YZ")
        .circle(BORE_R)
        .extrude(BORE_DEPTH + 0.002)
        .translate((x1 - BORE_DEPTH - 0.001, 0.0, HANDLE_Z))
    )
    handle = handle.cut(bore_cutter)

    # Soften bore entrance edge.
    try:
        handle = handle.edges(
            cq.selectors.NearestToPointSelector((x1, BORE_R, HANDLE_Z))
        ).chamfer(0.0010)
    except Exception:
        pass

    return handle


def _thread_turn_points(
    x_pos: float,
    rotation_deg: float,
    major_r: float,
    arc_deg: float,
    n_segments: int = 24,
) -> list[tuple[float, float, float]]:
    """3D centerline points for one partial turn of internal thread.

    The arc lies in a plane perpendicular to X at *x_pos*, centered on the
    handle axis (y=0, z=HANDLE_Z). *rotation_deg* rotates the arc gap around
    the X axis so successive turns show the helical offset.
    """
    rot = math.radians(rotation_deg)
    cos_r, sin_r = math.cos(rot), math.sin(rot)
    points: list[tuple[float, float, float]] = []
    for j in range(n_segments + 1):
        a = math.radians(j / n_segments * arc_deg)
        y = major_r * math.cos(a)
        z = major_r * math.sin(a)
        yr = y * cos_r - z * sin_r
        zr = y * sin_r + z * cos_r
        points.append((x_pos, yr, zr + HANDLE_Z))
    return points


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="paint_roller")
    model.materials.extend([CREAM, STEEL, CORAL, THREAD_CORAL, END_CAP])

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

    # --- Extension pole socket threads (inline decorations on the root) ------
    x1 = SOCKET_X1
    for i in range(NUM_THREAD_TURNS):
        x_pos = x1 - THREAD_PITCH * (i + 0.5)
        rot_deg = i * (360.0 / NUM_THREAD_TURNS)
        pts = _thread_turn_points(x_pos, rot_deg, THREAD_MAJOR_R, THREAD_ARC_DEG)
        geom = tube_from_spline_points(
            pts,
            radius=THREAD_TUBE_R,
            samples_per_segment=6,
            radial_segments=8,
            cap_ends=True,
        )
        frame.visual(
            mesh_from_geometry(geom, f"thread_turn_{i}"),
            material=THREAD_CORAL,
            name=f"thread_turn_{i}",
        )

    # --- Moving child: roller cover (spins on the axle) ----------------------
    roller = model.part("roller_cover")
    roller.visual(
        mesh_from_cadquery(_roller_cover_shape(), "roller_cover"),
        material=CREAM,
        name="roller_cover",
    )
    roller.visual(
        mesh_from_cadquery(_core_tube_shape(), "roller_core"),
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
        min_overlap=0.15,
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

    # --- Roller cover is a long hollow cylinder centered on the axle ---------
    cover = roller.get_visual("roller_cover")
    aabb = ctx.part_element_world_aabb(roller, elem=cover)
    assert aabb is not None
    (xmin, ymin, zmin), (xmax, ymax, zmax) = aabb
    length_x = xmax - xmin
    dia_y = ymax - ymin
    dia_z = zmax - zmin
    ctx.check(
        "roller cover reads as a long cylinder",
        length_x > 0.16 and 0.034 < dia_y < 0.042 and 0.034 < dia_z < 0.042,
        details=f"len_x={length_x:.3f}, dia_y={dia_y:.3f}, dia_z={dia_z:.3f}",
    )
    ctx.check(
        "roller cover is hollow-walled (real bore present)",
        ROLLER_BORE_R < ROLLER_OUTER_R - 0.006,
        details=f"outer_r={ROLLER_OUTER_R}, bore_r={ROLLER_BORE_R}",
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

    # --- Extension pole socket: flat butt end with bore and threads ----------
    ctx.check(
        "handle butt end is flat (socket face present)",
        gxmax > SOCKET_X1 - 0.003,
        details=f"grip xmax={gxmax:.4f}, socket_x1={SOCKET_X1:.4f}",
    )
    ctx.check(
        "handle socket boss wide enough for bore",
        (gzmax - gzmin) > 2.0 * BORE_R + 0.004,
        details=f"grip dz={gzmax - gzmin:.4f}, 2*bore_r+4mm={2*BORE_R+0.004:.4f}",
    )

    # Verify thread turns exist and are positioned within the bore region.
    for i in range(NUM_THREAD_TURNS):
        tname = f"thread_turn_{i}"
        tbb = ctx.part_element_world_aabb(frame, elem=frame.get_visual(tname))
        assert tbb is not None
        (tx0, _ty0, _tz0), (tx1, _ty1, _tz1) = tbb
        expected_x = SOCKET_X1 - THREAD_PITCH * (i + 0.5)
        ctx.check(
            f"{tname} is within the bore",
            abs(tx0 + tx1 - 2.0 * expected_x) / 2.0 < THREAD_PITCH,
            details=f"center_x={(tx0+tx1)/2:.4f}, expected≈{expected_x:.4f}",
        )

    # Thread turns contact the handle bore wall (connectivity within the part).
    ctx.expect_contact(
        frame,
        frame,
        elem_a="thread_turn_0",
        elem_b="handle_grip",
        contact_tol=0.001,
        name="thread turn 0 contacts bore wall",
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
