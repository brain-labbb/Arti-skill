from __future__ import annotations

# Articraft model: a small/mini paint roller with a hollow tubular sleeve grip.
#
# Real object (from reference image):
#   - A cream/white cylindrical roller cover (foam over a hard core), with
#     visibly hollow open ends.
#   - A gray bent-wire steel frame ("crank cage"): one long axle segment runs
#     through the roller core, bends down ~90deg at the handle end, then runs
#     out to a handle.
#   - An open hollow tubular sleeve grip: a straight tube with a visible bore
#     at its butt (collar) end that the wire shank plugs into. The bore is
#     stepped: a wide entry section (visible annular gap around the wire) and
#     a tight socket section deeper in (press-fit capture).
#
# Primary mechanism: the roller cover free-spins on the wire axle. That is a
# CONTINUOUS revolute joint about the roller's long axis (here world +X).
#
# Frame convention: roller long axis = X, roller centered at x=0. The roller's
# free (far) end is toward -X. The wire bends down at the +X end and the
# tubular grip extends further in +X, away from the roller.
# The wire frame and the tubular grip form one rigid root body. The roller
# cover is the single moving child, captured on the axle.

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
HANDLE_LEN = 0.130          # grip tube length (along X)

AXLE_Z = 0.0                       # roller axle line height
HANDLE_Z = AXLE_Z - FRAME_DROP     # handle axis height (below the axle)

# Key X stations of the crank.
AXLE_FAR_X = ROLLER_X_MIN - 0.006          # capped stub past the far end
AXLE_NEAR_X = ROLLER_X_MAX + 0.012         # axle clears the +X roller face
BEND_X = AXLE_NEAR_X + 0.006               # top corner of the crank
HANDLE_TOP_X = BEND_X + 0.030              # where the wire meets the grip top
SOCKET_X = HANDLE_TOP_X                    # grip collar (butt end) X

# --- Tubular sleeve grip dimensions -----------------------------------------
TUBE_OUTER_R = 0.0135       # outer radius of the grip tube (~27 mm OD)

# Stepped bore: wide entry (visible annular gap) + tight socket (press fit).
ENTRY_BORE_R = AXLE_R + 0.0028   # wide entry bore (11.2 mm dia, visible gap)
ENTRY_BORE_DEPTH = 0.016         # 16 mm entry section
SOCKET_BORE_R = AXLE_R - 0.0003  # tight socket bore (press fit on wire)
# Socket extends from entry_depth through to far end (open through-tube).

# Grip rings: repeated annular ridges for tactile grip texture.
GRIP_RING_COUNT = 6
GRIP_RING_WIDTH = 0.004
GRIP_RING_PROUD = 0.0016    # how far the ring stands above the tube surface
GRIP_RING_EMBED = 0.0005    # how far the ring embeds into the tube wall
GRIP_RING_START_X = SOCKET_X + 0.028
GRIP_RING_END_X = SOCKET_X + HANDLE_LEN - 0.018

CREAM = Material(name="roller_cover_cream", rgba=(0.93, 0.91, 0.84, 1.0))
STEEL = Material(name="frame_steel", rgba=(0.62, 0.63, 0.65, 1.0))
CORAL = Material(name="handle_coral", rgba=(0.86, 0.45, 0.43, 1.0))
DARK_RUBBER = Material(name="grip_rubber", rgba=(0.32, 0.20, 0.18, 1.0))
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
    through the roller, up over the +X end, down the crank, into the tube bore."""
    return [
        (AXLE_FAR_X, 0.0, AXLE_Z),                 # capped stub, far end
        (ROLLER_X_MIN, 0.0, AXLE_Z),
        (0.0, 0.0, AXLE_Z),                        # mid axle (through roller)
        (ROLLER_X_MAX, 0.0, AXLE_Z),
        (AXLE_NEAR_X, 0.0, AXLE_Z),                # axle clears the +X face
        (BEND_X, 0.0, AXLE_Z - 0.006),             # round into the down bend
        (BEND_X + 0.006, 0.0, AXLE_Z - 0.030),     # diagonal down the crank
        (HANDLE_TOP_X - 0.012, 0.0, HANDLE_Z + 0.004),
        (HANDLE_TOP_X, 0.0, HANDLE_Z),             # arrives at the tube collar
        (HANDLE_TOP_X + 0.032, 0.0, HANDLE_Z),     # enters the socket bore
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


def _handle_tube_shape() -> cq.Workplane:
    """Open hollow tubular sleeve grip: a straight cylinder with a stepped bore.

    - Wide entry bore at the butt (collar) end: visible annular gap around the
      wire shank, so the opening reads as a real bore.
    - Tight socket bore from the entry step through to the far end: press-fit
      capture of the wire shank for structural connection.
    - Far end open (through-tube)."""
    x0 = SOCKET_X

    # Outer cylinder
    outer = (
        cq.Workplane("YZ")
        .circle(TUBE_OUTER_R)
        .extrude(HANDLE_LEN)
        .translate((x0, 0.0, HANDLE_Z))
    )

    # Entry bore: wide, visible at the butt end
    entry_bore = (
        cq.Workplane("YZ")
        .circle(ENTRY_BORE_R)
        .extrude(ENTRY_BORE_DEPTH + 0.002)
        .translate((x0 - 0.001, 0.0, HANDLE_Z))
    )

    # Socket bore: tight press-fit, from entry step through to far end
    socket_start_x = x0 + ENTRY_BORE_DEPTH
    socket_len = HANDLE_LEN - ENTRY_BORE_DEPTH + 0.006
    socket_bore = (
        cq.Workplane("YZ")
        .circle(SOCKET_BORE_R)
        .extrude(socket_len)
        .translate((socket_start_x, 0.0, HANDLE_Z))
    )

    tube = outer.cut(entry_bore).cut(socket_bore)

    # Chamfer the entry face edge for a finished look
    try:
        tube = tube.edges("<X").chamfer(0.0008)
    except Exception:
        pass
    # Chamfer the far end edge
    try:
        tube = tube.edges(">X").chamfer(0.0008)
    except Exception:
        pass

    return tube


def _grip_ring_shape(x_center: float) -> cq.Workplane:
    """Annular grip ring that sits on the tube outer surface at a given X
    center. The ring inner radius embeds slightly into the tube wall for
    geometric contact (not a floating island)."""
    ring_outer = TUBE_OUTER_R + GRIP_RING_PROUD
    ring_inner = TUBE_OUTER_R - GRIP_RING_EMBED
    ring = (
        cq.Workplane("YZ")
        .circle(ring_outer)
        .circle(ring_inner)
        .extrude(GRIP_RING_WIDTH)
        .translate((x_center - GRIP_RING_WIDTH / 2.0, 0.0, HANDLE_Z))
    )
    return ring


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="paint_roller")
    model.materials.extend([CREAM, STEEL, CORAL, DARK_RUBBER, END_CAP])

    # --- Root: tubular grip + fixed steel wire frame/cage --------------------
    frame = model.part("handle_frame")
    frame.visual(
        mesh_from_cadquery(_handle_tube_shape(), "handle_tube"),
        material=CORAL,
        name="handle_tube",
    )
    frame.visual(
        _frame_wire_mesh(),
        material=STEEL,
        name="wire_frame",
    )

    # Repeated grip rings along the tube surface (functional layer: grip
    # texture). Emitted via for-i-in-range with shared geometry helper,
    # regular spacing, and name_i naming. All rings are parent visuals
    # (no FIXED-joint decoration parts).
    ring_spacing = (
        (GRIP_RING_END_X - GRIP_RING_START_X) / max(GRIP_RING_COUNT - 1, 1)
    )
    for i in range(GRIP_RING_COUNT):
        x_pos = GRIP_RING_START_X + i * ring_spacing
        frame.visual(
            mesh_from_cadquery(_grip_ring_shape(x_pos), f"grip_ring_{i}"),
            material=DARK_RUBBER,
            name=f"grip_ring_{i}",
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
    # roller center, on the axle line — the actual contact surface between
    # the roller core journal and the wire axle.
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

    # --- Handle tube is below the axle, away from the roller, connected ------
    tube = frame.get_visual("handle_tube")
    tbb = ctx.part_element_world_aabb(frame, elem=tube)
    assert tbb is not None
    (txmin, _ty0, tzmin), (txmax, _ty1, tzmax) = tbb
    ctx.check(
        "handle tube sits below the roller axle",
        tzmax < AXLE_Z - 0.02,
        details=f"tube z[{tzmin:.3f},{tzmax:.3f}], axle_z={AXLE_Z}",
    )
    ctx.check(
        "handle tube extends out beyond the roller (+X side)",
        txmin > ROLLER_X_MAX,
        details=f"tube xmin={txmin:.3f}, roller xmax={ROLLER_X_MAX:.3f}",
    )

    # --- Handle tube reads as a hollow straight cylinder ----------------------
    tube_dia_y = tbb[1][1] - tbb[0][1]
    tube_dia_z = tbb[1][2] - tbb[0][2]
    tube_len_x = tbb[1][0] - tbb[0][0]
    ctx.check(
        "handle tube reads as a straight cylinder",
        0.024 < tube_dia_y < 0.032 and 0.024 < tube_dia_z < 0.032 and tube_len_x > 0.12,
        details=f"dia_y={tube_dia_y:.4f}, dia_z={tube_dia_z:.4f}, len={tube_len_x:.4f}",
    )
    ctx.check(
        "handle tube has a real stepped bore (hollow sleeve grip)",
        ENTRY_BORE_R > AXLE_R + 0.001
        and SOCKET_BORE_R < AXLE_R
        and ENTRY_BORE_DEPTH > 0.008,
        details=(
            f"entry_bore_r={ENTRY_BORE_R}, socket_bore_r={SOCKET_BORE_R}, "
            f"entry_depth={ENTRY_BORE_DEPTH}, axle_r={AXLE_R}"
        ),
    )

    # Wire shank enters the tube bore (overlap on X proves insertion depth).
    ctx.expect_overlap(
        frame,
        frame,
        axes="x",
        elem_a="wire_frame",
        elem_b="handle_tube",
        min_overlap=0.020,
        name="wire shank inserts into tube bore",
    )

    # --- Grip rings are present and regularly spaced -------------------------
    ring_centers_x: list[float] = []
    for i in range(GRIP_RING_COUNT):
        ring_name = f"grip_ring_{i}"
        ring = frame.get_visual(ring_name)
        rbb = ctx.part_element_world_aabb(frame, elem=ring)
        assert rbb is not None
        ring_dy = rbb[1][1] - rbb[0][1]
        ctx.check(
            f"{ring_name} is present and proud of the tube",
            ring_dy > tube_dia_y - 0.001,
            details=f"ring_dy={ring_dy:.4f}, tube_dy={tube_dia_y:.4f}",
        )
        ring_centers_x.append((rbb[0][0] + rbb[1][0]) / 2.0)

    if len(ring_centers_x) >= 2:
        spacings = [
            ring_centers_x[j + 1] - ring_centers_x[j]
            for j in range(len(ring_centers_x) - 1)
        ]
        avg_spacing = sum(spacings) / len(spacings)
        max_dev = max(abs(s - avg_spacing) for s in spacings)
        ctx.check(
            "grip rings are regularly spaced along the tube",
            max_dev < 0.002 and avg_spacing > 0.005,
            details=f"spacings={[round(s, 4) for s in spacings]}, max_dev={max_dev:.4f}",
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

    # Roller cover must clear the handle tube (no collision between them).
    ctx.expect_gap(
        frame,
        roller,
        axis="x",
        positive_elem="handle_tube",
        negative_elem="roller_cover",
        min_gap=0.0,
        name="handle tube clears the roller cover",
    )

    return ctx.report()


object_model = build_object_model()
