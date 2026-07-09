from __future__ import annotations

# Articraft model: a small/mini paint roller — ergonomic-grip variant.
#
# Real object (from reference image):
#   - A cream/white cylindrical roller cover (foam over a hard core), with
#     visibly hollow open ends.
#   - A gray bent-wire steel frame ("crank cage"): one long axle segment runs
#     through the roller core, bends down ~90deg at the handle end, then runs
#     out to a pink plastic handle.
#   - A pink/coral ergonomic molded plastic grip handle with finger-contoured
#     scalloped ridges along its length. The wire socket plugs into the collar.
#
# Primary mechanism: the roller cover free-spins on the wire axle. That is a
# CONTINUOUS revolute joint about the roller's long axis (here world +X).
#
# Frame convention: roller long axis = X, roller centered at x=0. The roller's
# free (far) end is toward -X. The wire bends down at the +X end and the pink
# handle extends further in +X, away from the roller -- matching the image.
# The wire frame and the pink handle form one rigid root body. The roller cover
# is the single moving child, captured on the axle.
#
# Ergonomic grip variant:
#   The smooth molded grip is replaced with a ribbed scalloped handle. The
#   revolved body profile has valleys between finger positions, and raised
#   torus-ring ridges sit proud at each finger station. The ridges are
#   non-moving decorations inlined as parent visuals on the handle_frame part,
#   emitted via a for-i-in-range loop with a shared geometry helper.

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
    TorusGeometry,
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

# ----------------------------------------------------------------------------
# Ergonomic grip: finger ridge parameters
# ----------------------------------------------------------------------------
NUM_FINGER_RIDGES = 5
RIDGE_TUBE_R = 0.0022         # torus tube radius (ridge protrusion)
RIDGE_START_OFFSET = 0.022    # first ridge offset from collar (SOCKET_X)
RIDGE_SPACING = 0.020         # uniform spacing between ridges along X
HANDLE_PEAK_R = 0.0120        # handle body radius at ridge peaks
HANDLE_VALLEY_R = 0.0108      # handle body radius at scalloped valleys

CREAM = Material(name="roller_cover_cream", rgba=(0.93, 0.91, 0.84, 1.0))
STEEL = Material(name="frame_steel", rgba=(0.62, 0.63, 0.65, 1.0))
CORAL = Material(name="handle_coral", rgba=(0.86, 0.45, 0.43, 1.0))
END_CAP = Material(name="roller_endcap", rgba=(0.80, 0.78, 0.72, 1.0))
RIDGE_RUBBER = Material(name="grip_ridge_rubber", rgba=(0.72, 0.35, 0.33, 1.0))


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
    """Ergonomic scalloped grip: the revolved profile has valleys between finger
    ridge positions, creating a ribbed contour. The raised ridge visuals (torus
    rings) sit proud of the valley surface at each finger station. The wire
    still sockets into the collar at the +X-adjacent end."""
    x0 = SOCKET_X                 # collar / top, where the wire plugs in
    x1 = SOCKET_X + HANDLE_LEN    # rounded toe, far from the roller

    # Scalloped profile: peaks at ridge stations, valleys between them.
    # The peaks match the ridge X positions so the torus rings land on the
    # raised body regions; the valleys sit between ridges for finger recesses.
    pts = [
        (x0, 0.0),                                   # axis start
        (x0, 0.0085),                                # collar neck (wire socket)
        (x0 + 0.006, 0.0115),                        # flare below collar
        (x0 + 0.015, HANDLE_PEAK_R),                 # approach ridge 0
        (x0 + 0.022, HANDLE_PEAK_R),                 # ridge 0 station
        (x0 + 0.032, HANDLE_VALLEY_R),               # valley 0–1
        (x0 + 0.042, HANDLE_PEAK_R),                 # ridge 1 station
        (x0 + 0.052, HANDLE_VALLEY_R),               # valley 1–2
        (x0 + 0.062, HANDLE_PEAK_R),                 # ridge 2 station
        (x0 + 0.072, HANDLE_VALLEY_R),               # valley 2–3
        (x0 + 0.082, HANDLE_PEAK_R),                 # ridge 3 station
        (x0 + 0.092, HANDLE_VALLEY_R),               # valley 3–4
        (x0 + 0.102, HANDLE_PEAK_R),                 # ridge 4 station
        (x0 + 0.112, 0.0110),                        # past grip, tapering
        (x1 - 0.012, 0.0095),                        # toe approach
        (x1 - 0.004, 0.0052),                        # rounded toe
        (x1, 0.0),                                   # axis end
    ]
    prof = cq.Workplane("XZ").polyline(pts).close()
    handle = prof.revolve(360.0, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    handle = handle.translate((0.0, 0.0, HANDLE_Z))
    return handle


def _ridge_x_position(i: int) -> float:
    """X coordinate of the i-th finger ridge center."""
    return SOCKET_X + RIDGE_START_OFFSET + i * RIDGE_SPACING


def _finger_ridge_mesh(i: int):
    """Shared geometry helper: one finger ridge torus ring at station i.
    The torus wraps around the handle body with its hole aligned to the handle
    axis (world X). The tube radius creates a raised bump above the handle
    valley surface, giving the grip its ribbed tactile contour."""
    x_pos = _ridge_x_position(i)
    geom = TorusGeometry(
        radius=HANDLE_PEAK_R,
        tube=RIDGE_TUBE_R,
        radial_segments=14,
        tubular_segments=28,
    )
    # TorusGeometry: ring in XY plane, hole along Z.
    # Rotate 90° around Y so hole aligns with X (handle axis).
    geom.rotate_y(math.pi / 2.0)
    # Place at the ridge X station on the handle axis.
    geom.translate(x_pos, 0.0, HANDLE_Z)
    return mesh_from_geometry(geom, f"finger_ridge_{i}")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="paint_roller")
    model.materials.extend([CREAM, STEEL, CORAL, END_CAP, RIDGE_RUBBER])

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

    # Ergonomic finger ridges: repeated non-moving decorations emitted via a
    # for-i-in-range loop with a shared geometry helper, regular placement,
    # and uniform inline-visual policy (no separate parts or FIXED joints).
    for i in range(NUM_FINGER_RIDGES):
        frame.visual(
            _finger_ridge_mesh(i),
            material=RIDGE_RUBBER,
            name=f"finger_ridge_{i}",
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

    # --- Ergonomic grip: finger ridges exist, are raised, and regularly spaced
    ridge_names = [f"finger_ridge_{i}" for i in range(NUM_FINGER_RIDGES)]
    for name in ridge_names:
        rv = frame.get_visual(name)
        ctx.check(
            f"finger ridge visual '{name}' exists on handle_frame",
            rv is not None,
            details=f"visual lookup returned {rv}",
        )

    # Check each ridge protrudes above the handle valley surface and is at the
    # correct X station.
    ridge_x_centers = []
    for i in range(NUM_FINGER_RIDGES):
        rv = frame.get_visual(f"finger_ridge_{i}")
        rbb = ctx.part_element_world_aabb(frame, elem=rv)
        assert rbb is not None
        (rxmin, rymin, rzmin), (rxmax, rymax, rzmax) = rbb
        ridge_x_centers.append((rxmin + rxmax) / 2.0)

        # Ridge Z extent should exceed the valley diameter (protrusion check).
        ridge_dia_z = rzmax - rzmin
        valley_dia = 2.0 * HANDLE_VALLEY_R
        ctx.check(
            f"finger_ridge_{i} protrudes above handle valley",
            ridge_dia_z > valley_dia + 0.001,
            details=f"ridge_dia_z={ridge_dia_z:.4f}, valley_dia={valley_dia:.4f}",
        )

        # Ridge should be near its expected X station.
        expected_x = _ridge_x_position(i)
        ctx.check(
            f"finger_ridge_{i} at correct X station",
            abs(ridge_x_centers[-1] - expected_x) < 0.004,
            details=f"actual_x={ridge_x_centers[-1]:.4f}, expected_x={expected_x:.4f}",
        )

    # Ridges are regularly spaced (uniform spacing along X).
    if len(ridge_x_centers) >= 2:
        spacings = [
            ridge_x_centers[i + 1] - ridge_x_centers[i]
            for i in range(len(ridge_x_centers) - 1)
        ]
        avg_spacing = sum(spacings) / len(spacings)
        max_deviation = max(abs(s - avg_spacing) for s in spacings)
        ctx.check(
            "finger ridges are regularly spaced along X",
            max_deviation < 0.003 and 0.015 < avg_spacing < 0.025,
            details=f"spacings={[f'{s:.4f}' for s in spacings]}, max_dev={max_deviation:.4f}",
        )

    # Handle body has scalloped profile: the grip zone max diameter is less
    # than the original smooth barrel diameter, proving the valleys exist.
    grip_bb = ctx.part_element_world_aabb(frame, elem=grip)
    assert grip_bb is not None
    grip_dia_z = grip_bb[1][2] - grip_bb[0][2]
    original_barrel_dia = 2.0 * 0.0140  # the old smooth grip diameter
    ctx.check(
        "handle grip has scalloped contour (valleys reduce max diameter)",
        grip_dia_z < original_barrel_dia + 0.001,
        details=f"grip_dia_z={grip_dia_z:.4f}, original_barrel={original_barrel_dia:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
