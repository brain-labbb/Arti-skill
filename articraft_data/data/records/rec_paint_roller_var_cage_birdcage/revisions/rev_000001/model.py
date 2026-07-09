from __future__ import annotations

# Articraft model: paint roller with birdcage spider frame.
#
# Real object (Handtools/Paint roller, birdcage fork variant):
#   - Cream/white cylindrical roller cover (foam over a hard core), with
#     visibly hollow open ends.
#   - Steel wire frame: a straight axle segment runs through the roller core
#     bore; at the handle-side (+X) end a birdcage spider cage of 6 wire
#     spokes radiates from a small hub end-cap retainer, forming a retention
#     cage that holds the cover on the axle while it still free-spins; a
#     handle stem drops from the hub down to the pink plastic grip.
#   - Pink/coral molded plastic grip handle.
#
# Primary mechanism: the roller cover free-spins on the wire axle. This is a
# CONTINUOUS revolute joint about the roller's long axis (world +X).
#
# Frame convention: roller long axis = X, roller centered at x=0. The roller's
# free (far) end is toward -X. The birdcage spider cage is at the +X end, and
# the handle extends further in +X, below the axle line.
# The wire frame (axle + stem + cage + hub cap) and the pink handle form one
# rigid root body. The roller cover is the single moving child.

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

FRAME_DROP = 0.055          # vertical drop of the handle stem (Z)
HANDLE_LEN = 0.130          # pink grip length (along X)

AXLE_Z = 0.0                       # roller axle line height
HANDLE_Z = AXLE_Z - FRAME_DROP     # handle axis height (below the axle)

# Key X stations.
AXLE_FAR_X = ROLLER_X_MIN - 0.006          # capped stub past the far end
HUB_X = ROLLER_X_MAX + 0.005               # birdcage hub center (just past +X face)
HANDLE_TOP_X = HUB_X + 0.035               # where the stem meets the grip collar
SOCKET_X = HANDLE_TOP_X                    # grip top (collar) X

# Birdcage spider parameters.
N_SPOKES = 6
HUB_R = 0.006                    # hub end-cap outer radius
HUB_THICKNESS = 0.004            # hub disk thickness

CREAM = Material(name="roller_cover_cream", rgba=(0.93, 0.91, 0.84, 1.0))
STEEL = Material(name="frame_steel", rgba=(0.62, 0.63, 0.65, 1.0))
CORAL = Material(name="handle_coral", rgba=(0.86, 0.45, 0.43, 1.0))
END_CAP = Material(name="roller_endcap", rgba=(0.80, 0.78, 0.72, 1.0))
HUB_MAT = Material(name="hub_retainer", rgba=(0.55, 0.56, 0.58, 1.0))


# ----------------------------------------------------------------------------
# Geometry helpers
# ----------------------------------------------------------------------------

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
    try:
        cover = cover.edges("%CIRCLE").fillet(0.0020)
    except Exception:
        pass
    return cover


def _core_tube_shape() -> cq.Workplane:
    """Thin rigid inner core (hard plastic sleeve) bonded inside the cover bore.
    Its outer wall contacts the cover bore and its inner bore is the journal
    that runs on the steel axle."""
    core_outer_r = ROLLER_BORE_R + 0.0003
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


def _wire_tube_mesh(path: list[tuple[float, float, float]], name: str):
    """Shared geometry helper: swept wire tube from a 3D centerline path."""
    geom = tube_from_spline_points(
        path,
        radius=AXLE_R,
        samples_per_segment=14,
        radial_segments=16,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, name)


def _axle_wire_path() -> list[tuple[float, float, float]]:
    """Straight axle wire from far-end stub, through the roller bore, to the
    birdcage hub."""
    return [
        (AXLE_FAR_X, 0.0, AXLE_Z),
        (ROLLER_X_MIN, 0.0, AXLE_Z),
        (0.0, 0.0, AXLE_Z),
        (ROLLER_X_MAX, 0.0, AXLE_Z),
        (HUB_X, 0.0, AXLE_Z),
    ]


def _handle_stem_path() -> list[tuple[float, float, float]]:
    """Wire stem from the birdcage hub, dropping down to the handle grip socket."""
    return [
        (HUB_X, 0.0, AXLE_Z),
        (HUB_X + 0.006, 0.0, AXLE_Z - 0.008),
        (HUB_X + 0.018, 0.0, AXLE_Z - 0.030),
        (HANDLE_TOP_X - 0.010, 0.0, HANDLE_Z + 0.006),
        (HANDLE_TOP_X, 0.0, HANDLE_Z),
        (HANDLE_TOP_X + 0.022, 0.0, HANDLE_Z),
    ]


def _cage_spoke_path(i: int, n_spokes: int = N_SPOKES) -> list[tuple[float, float, float]]:
    """One birdcage spoke: curves from the hub outward to near the roller OD at
    the handle-side roller end, forming a retention cage. Each spoke seats
    against the roller cover end face (small intentional overlap)."""
    angle = 2.0 * math.pi * i / n_spokes
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)

    r0 = HUB_R                                              # at hub edge
    r1 = HUB_R + (ROLLER_OUTER_R - HUB_R) * 0.35            # 35% outward
    r2 = HUB_R + (ROLLER_OUTER_R - HUB_R) * 0.70            # 70% outward
    r3 = ROLLER_OUTER_R - 0.002                              # near roller OD

    x0 = HUB_X
    x1 = HUB_X - 0.001
    x2 = ROLLER_X_MAX + 0.003
    x3 = ROLLER_X_MAX + 0.001

    return [
        (x0, r0 * cos_a, AXLE_Z + r0 * sin_a),
        (x1, r1 * cos_a, AXLE_Z + r1 * sin_a),
        (x2, r2 * cos_a, AXLE_Z + r2 * sin_a),
        (x3, r3 * cos_a, AXLE_Z + r3 * sin_a),
    ]


def _hub_cap_shape() -> cq.Workplane:
    """End-cap retainer disk at the birdcage hub center, with a bore for the
    axle. This is the central piece the spokes radiate from."""
    outer = (
        cq.Workplane("YZ")
        .circle(HUB_R)
        .extrude(HUB_THICKNESS)
        .translate((HUB_X - HUB_THICKNESS, 0.0, AXLE_Z))
    )
    bore = (
        cq.Workplane("YZ")
        .circle(AXLE_R + 0.0005)
        .extrude(HUB_THICKNESS + 0.002)
        .translate((HUB_X - HUB_THICKNESS - 0.001, 0.0, AXLE_Z))
    )
    return outer.cut(bore)


def _handle_shape() -> cq.Workplane:
    """Pink molded grip: a rounded, gently barrel-tapered body, revolved about
    the handle axis (X)."""
    x0 = SOCKET_X
    x1 = SOCKET_X + HANDLE_LEN
    pts = [
        (x0, 0.0),
        (x0, 0.0085),
        (x0 + 0.006, 0.0115),
        (x0 + 0.020, 0.0135),
        (x0 + 0.058, 0.0140),
        (x0 + 0.100, 0.0122),
        (x1 - 0.012, 0.0095),
        (x1 - 0.004, 0.0052),
        (x1, 0.0),
    ]
    prof = cq.Workplane("XZ").polyline(pts).close()
    handle = prof.revolve(360.0, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    handle = handle.translate((0.0, 0.0, HANDLE_Z))
    return handle


# ----------------------------------------------------------------------------
# Model
# ----------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="paint_roller")
    model.materials.extend([CREAM, STEEL, CORAL, END_CAP, HUB_MAT])

    # --- Root: handle frame with birdcage spider cage -----------------------
    frame = model.part("handle_frame")

    # Pink grip handle
    frame.visual(
        mesh_from_cadquery(_handle_shape(), "handle_grip"),
        material=CORAL,
        name="handle_grip",
    )

    # Straight axle wire (through roller bore to the hub)
    frame.visual(
        _wire_tube_mesh(_axle_wire_path(), "wire_axle"),
        material=STEEL,
        name="wire_axle",
    )

    # Handle stem (wire from hub, dropping down to grip socket)
    frame.visual(
        _wire_tube_mesh(_handle_stem_path(), "handle_stem"),
        material=STEEL,
        name="handle_stem",
    )

    # Birdcage hub end-cap retainer (disk with axle bore)
    frame.visual(
        mesh_from_cadquery(_hub_cap_shape(), "hub_cap"),
        material=HUB_MAT,
        name="hub_cap",
    )

    # Birdcage cage spokes — repeated sub-parts via for-i-in-range loop,
    # shared geometry helper (_wire_tube_mesh / _cage_spoke_path), regular
    # angular placement (2π/N_SPOKES), uniform static-on-frame joint policy.
    for i in range(N_SPOKES):
        frame.visual(
            _wire_tube_mesh(_cage_spoke_path(i), f"cage_spoke_{i}"),
            material=STEEL,
            name=f"cage_spoke_{i}",
        )

    # --- Moving child: roller cover (spins on axle) ------------------------
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

    # Roller spins freely about the axle (world +X). Joint origin at the
    # roller center on the axle line — the actual bearing contact surface.
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


# ----------------------------------------------------------------------------
# Tests
# ----------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("handle_frame")
    roller = object_model.get_part("roller_cover")
    spin = object_model.get_articulation("frame_to_roller")

    # --- Axle / roller journal bearing overlap allowance --------------------
    ctx.allow_overlap(
        frame,
        roller,
        elem_a="wire_axle",
        elem_b="roller_core",
        reason=(
            "The steel axle wire is intentionally captured inside the roller "
            "core bore; the cover spins on it as a journal bearing."
        ),
    )

    # --- Birdcage spoke / roller cover seat allowances ----------------------
    for i in range(N_SPOKES):
        ctx.allow_overlap(
            frame,
            roller,
            elem_a=f"cage_spoke_{i}",
            elem_b="roller_cover",
            reason=(
                "Birdcage spoke seats against the roller cover end face; the "
                "cage retains the cover on the axle while it free-spins."
            ),
        )

    # Prove the axle capture: wire and core share the axle line.
    ctx.expect_overlap(
        frame,
        roller,
        axes="x",
        elem_a="wire_axle",
        elem_b="roller_core",
        min_overlap=0.15,
        name="axle wire runs through the roller core",
    )

    # --- Joint type / axis is the real spin mechanism -----------------------
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

    # --- Roller cover is a long hollow cylinder centered on the axle --------
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

    # Core seated in cover bore (not a floating island within the roller part).
    ctx.expect_contact(
        roller,
        roller,
        elem_a="roller_core",
        elem_b="roller_cover",
        contact_tol=0.0006,
        name="roller core seated in cover bore",
    )

    # --- Birdcage spider structure ------------------------------------------
    # Hub cap exists at the handle-side roller end.
    hub = frame.get_visual("hub_cap")
    hub_bb = ctx.part_element_world_aabb(frame, elem=hub)
    assert hub_bb is not None
    (hx0, _, _), (hx1, _, _) = hub_bb
    ctx.check(
        "hub cap sits at handle-side roller end",
        hx0 > ROLLER_X_MAX - 0.005 and hx1 < ROLLER_X_MAX + 0.015,
        details=f"hub x[{hx0:.4f},{hx1:.4f}]",
    )

    # All 6 spokes exist, span outward from hub, and seat near roller face.
    for i in range(N_SPOKES):
        spoke = frame.get_visual(f"cage_spoke_{i}")
        sbb = ctx.part_element_world_aabb(frame, elem=spoke)
        assert sbb is not None
        (sx0, sy0, sz0), (sx1, sy1, sz1) = sbb
        span_y = sy1 - sy0
        span_z = sz1 - sz0
        ctx.check(
            f"cage_spoke_{i} spans outward from hub",
            span_y > 0.006 or span_z > 0.006,
            details=f"span_y={span_y:.4f}, span_z={span_z:.4f}",
        )
        # Spoke retains roller cover end face (contact or small embed).
        ctx.expect_contact(
            frame,
            roller,
            elem_a=f"cage_spoke_{i}",
            elem_b="roller_cover",
            contact_tol=0.006,
            name=f"cage_spoke_{i} retains roller cover end",
        )

    # Birdcage cage is at the handle-side end (+X).
    spoke0 = frame.get_visual("cage_spoke_0")
    s0bb = ctx.part_element_world_aabb(frame, elem=spoke0)
    assert s0bb is not None
    ctx.check(
        "birdcage cage is at the handle-side roller end",
        s0bb[0][0] > ROLLER_X_MAX - 0.010,
        details=f"spoke_0 xmin={s0bb[0][0]:.4f}",
    )

    # The 6 spokes form a roughly symmetric radial cage. Spoke 1 (angle=60°)
    # extends above the axle in +Z; spoke 4 (angle=240°) extends below in -Z.
    spoke_top = frame.get_visual("cage_spoke_1")
    spoke_bot = frame.get_visual("cage_spoke_4")
    top_bb = ctx.part_element_world_aabb(frame, elem=spoke_top)
    bot_bb = ctx.part_element_world_aabb(frame, elem=spoke_bot)
    assert top_bb is not None and bot_bb is not None
    top_z_max = top_bb[1][2]
    bot_z_min = bot_bb[0][2]
    ctx.check(
        "birdcage spokes span above and below the axle (cage profile)",
        top_z_max > AXLE_Z + 0.004 and bot_z_min < AXLE_Z - 0.004,
        details=f"top_z_max={top_z_max:.4f}, bot_z_min={bot_z_min:.4f}",
    )

    # --- Handle stem connects hub to grip -----------------------------------
    ctx.expect_contact(
        frame,
        frame,
        elem_a="handle_stem",
        elem_b="handle_grip",
        contact_tol=1e-6,
        name="handle stem plugs into handle grip",
    )

    # Handle grip is below the axle, extends past the roller.
    grip = frame.get_visual("handle_grip")
    gbb = ctx.part_element_world_aabb(frame, elem=grip)
    assert gbb is not None
    (gxmin, _, gzmin), (gxmax, _, gzmax) = gbb
    ctx.check(
        "handle grip sits below the roller axle",
        gzmax < AXLE_Z - 0.02,
        details=f"grip z[{gzmin:.3f},{gzmax:.3f}]",
    )
    ctx.check(
        "handle grip extends out beyond the roller (+X side)",
        gxmin > ROLLER_X_MAX,
        details=f"grip xmin={gxmin:.3f}, roller xmax={ROLLER_X_MAX:.3f}",
    )

    # --- Roller actually spins: poseable and stays centered -----------------
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

    # Roller cover clears the pink handle grip.
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
