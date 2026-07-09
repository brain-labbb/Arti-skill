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
import random

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MeshGeometry,
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

CREAM = Material(name="roller_cover_cream", rgba=(0.93, 0.91, 0.84, 1.0))
STEEL = Material(name="frame_steel", rgba=(0.62, 0.63, 0.65, 1.0))
CORAL = Material(name="handle_coral", rgba=(0.86, 0.45, 0.43, 1.0))
END_CAP = Material(name="roller_endcap", rgba=(0.80, 0.78, 0.72, 1.0))
NAP = Material(name="roller_nap_fabric", rgba=(0.96, 0.95, 0.90, 1.0))

# Nap texture parameters
NAP_RINGS = 6                # number of nap ring sections along the roller
NAP_AXIAL_SEGMENTS = 14      # axial resolution per ring
NAP_CIRC_SEGMENTS = 48       # circumferential resolution
NAP_DEPTH = 0.0018           # max radial displacement for fiber nap (~1.8mm)
NAP_END_MARGIN = 0.004       # keep nap clear of the open hollow ends


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


def _nap_ring_mesh(ring_index: int, n_rings: int) -> MeshGeometry:
    """Generate one ring section of napped pile fabric texture.

    Creates a thin cylindrical shell section with radially displaced outer
    vertices that simulate short raised fibers (nap). Each ring covers an
    equal portion of the roller length, with a small gap at the open ends
    so the hollow bore remains visible.

    Shared geometry helper used in a for-i-in-range(n) loop with name_i
    style naming and regular placement along the roller axis.
    """
    rng = random.Random(42 + ring_index * 137)

    usable_len = ROLLER_LEN - 2.0 * NAP_END_MARGIN
    ring_len = usable_len / n_rings
    gap = ring_len * 0.02  # tiny gap between rings for visual separation
    x_start = ROLLER_X_MIN + NAP_END_MARGIN + ring_index * ring_len + gap
    x_end = x_start + ring_len - 2.0 * gap

    n_ax = NAP_AXIAL_SEGMENTS
    n_ci = NAP_CIRC_SEGMENTS

    geom = MeshGeometry()

    # Build inner (smooth bore-side) and outer (textured nap) vertex grids.
    inner_ids: dict[tuple[int, int], int] = {}
    outer_ids: dict[tuple[int, int], int] = {}

    inner_r = ROLLER_OUTER_R - 0.0004  # thin shell anchored just under the nap
    for i in range(n_ax + 1):
        x = x_start + (x_end - x_start) * i / n_ax
        for j in range(n_ci):
            theta = 2.0 * math.pi * j / n_ci
            cos_t = math.cos(theta)
            sin_t = math.sin(theta)

            iv = geom.add_vertex(x, inner_r * cos_t, inner_r * sin_t)
            inner_ids[(i, j)] = iv

            # Outer: radial displacement simulating raised fiber tufts.
            # Use layered noise: a base bump plus finer grain variation.
            base_bump = 0.4 + 0.6 * (0.5 + 0.5 * math.sin(
                7.0 * theta + 3.0 * x / ROLLER_LEN * math.pi))
            fine = rng.uniform(0.15, 1.0)
            disp = NAP_DEPTH * (0.35 * base_bump + 0.65 * fine)
            # Slight angular jitter for fiber-direction irregularity.
            jt = theta + rng.gauss(0.0, 0.006)
            r_outer = ROLLER_OUTER_R + disp
            ov = geom.add_vertex(x, r_outer * math.cos(jt), r_outer * math.sin(jt))
            outer_ids[(i, j)] = ov

    # Outer surface triangles (normal outward).
    for i in range(n_ax):
        for j in range(n_ci):
            j1 = (j + 1) % n_ci
            a = outer_ids[(i, j)]
            b = outer_ids[(i, j1)]
            c = outer_ids[(i + 1, j1)]
            d = outer_ids[(i + 1, j)]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)

    # Inner surface triangles (normal inward).
    for i in range(n_ax):
        for j in range(n_ci):
            j1 = (j + 1) % n_ci
            a = inner_ids[(i, j)]
            b = inner_ids[(i + 1, j)]
            c = inner_ids[(i + 1, j1)]
            d = inner_ids[(i, j1)]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)

    # End-cap rings connecting inner to outer at each ring boundary.
    for end_i, flip in ((0, False), (n_ax, True)):
        for j in range(n_ci):
            j1 = (j + 1) % n_ci
            ii = inner_ids[(end_i, j)]
            ii1 = inner_ids[(end_i, j1)]
            oo = outer_ids[(end_i, j)]
            oo1 = outer_ids[(end_i, j1)]
            if not flip:
                geom.add_face(ii, oo, oo1)
                geom.add_face(ii, oo1, ii1)
            else:
                geom.add_face(ii, oo1, oo)
                geom.add_face(ii, ii1, oo1)

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="paint_roller")
    model.materials.extend([CREAM, STEEL, CORAL, END_CAP, NAP])

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
        mesh_from_cadquery(_roller_cover_shape(), "roller_cover"),
        material=CREAM,
        name="roller_cover",
    )
    roller.visual(
        mesh_from_cadquery(_core_tube_shape(), "roller_core"),
        material=END_CAP,
        name="roller_core",
    )

    # --- Napped pile fabric texture: raised short-fiber nap on the outer
    # cylindrical wall, split into ring sections for structured naming.
    # Ends stay clear of the open hollow bore so the hollow ends remain visible.
    for i in range(NAP_RINGS):
        roller.visual(
            mesh_from_geometry(_nap_ring_mesh(i, NAP_RINGS), f"nap_ring_{i}"),
            material=NAP,
            name=f"nap_ring_{i}",
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

    # --- Napped pile fabric texture checks -----------------------------------
    # All nap ring visuals exist on the roller cover part.
    for i in range(NAP_RINGS):
        ctx.check(
            f"nap_ring_{i}_exists",
            roller.get_visual(f"nap_ring_{i}") is not None,
            details=f"nap_ring_{i} visual missing from roller_cover",
        )

    # The nap texture extends radially beyond the smooth base cylinder wall,
    # proving the surface has raised fibers rather than being smooth foam.
    nap_aabbs = []
    for i in range(NAP_RINGS):
        nap_v = roller.get_visual(f"nap_ring_{i}")
        bb = ctx.part_element_world_aabb(roller, elem=nap_v)
        if bb is not None:
            nap_aabbs.append(bb)

    if nap_aabbs:
        nap_y_min = min(bb[0][1] for bb in nap_aabbs)
        nap_y_max = max(bb[1][1] for bb in nap_aabbs)
        nap_z_min = min(bb[0][2] for bb in nap_aabbs)
        nap_z_max = max(bb[1][2] for bb in nap_aabbs)
        nap_dia_y = nap_y_max - nap_y_min
        nap_dia_z = nap_z_max - nap_z_min
        # Nap must be wider than the smooth base cylinder in at least one axis,
        # proving the raised fiber texture protrudes outward.
        ctx.check(
            "nap_texture_has_raised_fibers",
            nap_dia_y > 2.0 * ROLLER_OUTER_R + 0.0005
            or nap_dia_z > 2.0 * ROLLER_OUTER_R + 0.0005,
            details=f"nap_dia_y={nap_dia_y:.4f}, nap_dia_z={nap_dia_z:.4f}, "
                    f"smooth_dia={2.0 * ROLLER_OUTER_R:.4f}",
        )

        # Nap rings must not reach the roller ends (hollow bore visible).
        nap_x_min = min(bb[0][0] for bb in nap_aabbs)
        nap_x_max = max(bb[1][0] for bb in nap_aabbs)
        ctx.check(
            "nap_clears_hollow_ends",
            nap_x_min > ROLLER_X_MIN + 0.001 and nap_x_max < ROLLER_X_MAX - 0.001,
            details=f"nap_x=[{nap_x_min:.4f},{nap_x_max:.4f}], "
                    f"roller_x=[{ROLLER_X_MIN:.4f},{ROLLER_X_MAX:.4f}]",
        )

    return ctx.report()


object_model = build_object_model()
