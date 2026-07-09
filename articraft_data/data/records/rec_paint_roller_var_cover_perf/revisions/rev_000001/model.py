from __future__ import annotations

# Articraft model: a small/mini paint roller with perforated open-cage core.
#
# Real object (from reference image):
#   - A cream/white cylindrical roller cover (foam over a lattice core), with
#     visibly hollow open ends showing the cage structure inside.
#   - A gray bent-wire steel frame ("crank cage"): one long axle segment runs
#     through the roller core, bends down ~90deg at the handle end, then runs
#     out to a pink plastic handle.
#   - A pink/coral molded plastic grip handle that the wire socket plugs into.
#
# Lattice core variant:
#   The solid inner core tube is replaced by a visible open-cage lattice:
#   - 8 longitudinal ribs on the bore surface (thin rods along X)
#   - 5 circumferential hoop rings connecting the ribs
#   - 2 end spiders (hub + radial arms) that journal the axle
#   Looking through the hollow ends reveals the cage framework.
#
# Primary mechanism: the roller cover free-spins on the wire axle. That is a
# CONTINUOUS revolute joint about the roller's long axis (here world +X).
#
# Frame convention: roller long axis = X, roller centered at x=0. The roller's
# free (far) end is toward -X. The wire bends down at the +X end and the pink
# handle extends further in +X, away from the roller.

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

# ---------------------------------------------------------------------------
# Real-world dimensions (meters)
# ---------------------------------------------------------------------------
ROLLER_LEN = 0.180          # roller cover cylindrical length (along X)
ROLLER_OUTER_R = 0.0190     # roller cover outer radius (~38 mm dia)
ROLLER_BORE_R = 0.0078      # inner core bore radius (visible hollow ends)

AXLE_R = 0.0028             # steel wire radius (~5.6 mm wire)

ROLLER_X_MIN = -ROLLER_LEN / 2.0   # free (far) end, toward -X
ROLLER_X_MAX = ROLLER_LEN / 2.0    # handle-side end, toward +X

FRAME_DROP = 0.055          # vertical drop of the crank (Z)
HANDLE_LEN = 0.130          # pink grip length (along X)

AXLE_Z = 0.0                       # roller axle line height
HANDLE_Z = AXLE_Z - FRAME_DROP     # handle axis height (below the axle)

AXLE_FAR_X = ROLLER_X_MIN - 0.006          # capped stub past the far end
AXLE_NEAR_X = ROLLER_X_MAX + 0.012         # axle clears the +X roller face
BEND_X = AXLE_NEAR_X + 0.006               # top corner of the crank
HANDLE_TOP_X = BEND_X + 0.030              # where the wire meets the grip top
SOCKET_X = HANDLE_TOP_X                    # grip top (collar) X

# ---------------------------------------------------------------------------
# Lattice open-cage core parameters
# ---------------------------------------------------------------------------
N_RIBS = 8                   # longitudinal ribs around the bore
N_HOOPS = 5                  # circumferential hoop rings
N_SPIDER_ARMS = 4            # radial arms per end spider

LATTICE_RIB_R = 0.0008       # rib cross-section radius (1.6 mm dia rods)
LATTICE_CAGE_R = ROLLER_BORE_R - LATTICE_RIB_R  # rib centerline radius

LATTICE_HOOP_WALL = 0.0015   # hoop radial wall thickness
LATTICE_HOOP_WIDTH = 0.0015  # hoop axial width

SPIDER_HUB_R = 0.0045        # end spider hub outer radius
SPIDER_HUB_BORE_R = AXLE_R - 0.0006  # spider bore captures the axle
SPIDER_ARM_WIDTH = 0.0020    # spider arm transverse width
SPIDER_THICKNESS = 0.0025    # spider axial thickness

# Hoop X positions (evenly spaced, inset from roller ends)
HOOP_MARGIN = 0.012
HOOP_SPAN = ROLLER_LEN - 2.0 * HOOP_MARGIN
HOOP_POSITIONS = [
    ROLLER_X_MIN + HOOP_MARGIN + i * HOOP_SPAN / (N_HOOPS - 1)
    for i in range(N_HOOPS)
]

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------
CREAM = Material(name="roller_cover_cream", rgba=(0.93, 0.91, 0.84, 1.0))
STEEL = Material(name="frame_steel", rgba=(0.62, 0.63, 0.65, 1.0))
CORAL = Material(name="handle_coral", rgba=(0.86, 0.45, 0.43, 1.0))
LATTICE_MAT = Material(name="lattice_core_gray", rgba=(0.76, 0.76, 0.74, 1.0))


# ===================================================================
# Geometry helpers — roller cover (foam shell)
# ===================================================================

def _roller_cover_shape() -> cq.Workplane:
    """Hollow cream roller cover: an open-ended tube with a real bore so the
    ends read as hollow and reveal the lattice cage inside."""
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


# ===================================================================
# Geometry helpers — lattice open-cage core
# ===================================================================

def _lattice_rib_shape(angle_rad: float) -> cq.Workplane:
    """One longitudinal rib: a thin rod along X at the cage radius."""
    y_off = LATTICE_CAGE_R * math.cos(angle_rad)
    z_off = LATTICE_CAGE_R * math.sin(angle_rad)
    rib_len = ROLLER_LEN - 0.006
    return (
        cq.Workplane("YZ")
        .center(y_off, z_off)
        .circle(LATTICE_RIB_R)
        .extrude(rib_len)
        .translate((ROLLER_X_MIN + 0.003, 0.0, 0.0))
    )


def _lattice_hoop_shape(x_pos: float) -> cq.Workplane:
    """One circumferential hoop ring: a thin annular disk at the bore radius."""
    outer_r = ROLLER_BORE_R
    inner_r = outer_r - LATTICE_HOOP_WALL
    w = LATTICE_HOOP_WIDTH
    outer = (
        cq.Workplane("YZ")
        .circle(outer_r)
        .extrude(w)
        .translate((x_pos - w / 2.0, 0.0, 0.0))
    )
    bore = (
        cq.Workplane("YZ")
        .circle(inner_r)
        .extrude(w + 0.002)
        .translate((x_pos - w / 2.0 - 0.001, 0.0, 0.0))
    )
    return outer.cut(bore)


def _end_spider_shape(x_center: float) -> cq.Workplane:
    """End spider: a central hub ring (axle journal bearing) with radial arms
    connecting to the lattice cage ring at the bore wall."""
    # Hub ring with bore for the axle
    hub = (
        cq.Workplane("YZ")
        .circle(SPIDER_HUB_R)
        .extrude(SPIDER_THICKNESS)
        .translate((x_center - SPIDER_THICKNESS / 2.0, 0.0, 0.0))
    )
    hub_bore = (
        cq.Workplane("YZ")
        .circle(SPIDER_HUB_BORE_R)
        .extrude(SPIDER_THICKNESS + 0.002)
        .translate((x_center - SPIDER_THICKNESS / 2.0 - 0.001, 0.0, 0.0))
    )
    result = hub.cut(hub_bore)

    # Radial arms — each extends from inside the hub to the bore wall,
    # aligned with every (N_RIBS / N_SPIDER_ARMS)-th rib for connectivity.
    arm_len = ROLLER_BORE_R - SPIDER_HUB_R + 0.001  # overlap into hub
    r_mid = SPIDER_HUB_R - 0.001 + arm_len / 2.0

    for i in range(N_SPIDER_ARMS):
        angle_deg = 360.0 * i / N_SPIDER_ARMS
        arm = (
            cq.Workplane("YZ")
            .center(r_mid, 0.0)
            .rect(arm_len, SPIDER_ARM_WIDTH)
            .extrude(SPIDER_THICKNESS)
            .translate((x_center - SPIDER_THICKNESS / 2.0, 0.0, 0.0))
        )
        if abs(angle_deg) > 0.1:
            arm = arm.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), angle_deg)
        result = result.union(arm)

    return result


# ===================================================================
# Geometry helpers — frame and handle
# ===================================================================

def _frame_wire_path() -> list[tuple[float, float, float]]:
    """Centerline of the bent steel wire, world coords."""
    return [
        (AXLE_FAR_X, 0.0, AXLE_Z),
        (ROLLER_X_MIN, 0.0, AXLE_Z),
        (0.0, 0.0, AXLE_Z),
        (ROLLER_X_MAX, 0.0, AXLE_Z),
        (AXLE_NEAR_X, 0.0, AXLE_Z),
        (BEND_X, 0.0, AXLE_Z - 0.006),
        (BEND_X + 0.006, 0.0, AXLE_Z - 0.030),
        (HANDLE_TOP_X - 0.012, 0.0, HANDLE_Z + 0.004),
        (HANDLE_TOP_X, 0.0, HANDLE_Z),
        (HANDLE_TOP_X + 0.022, 0.0, HANDLE_Z),
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
    """Pink molded grip: a rounded, gently barrel-tapered body revolved about
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


# ===================================================================
# Build
# ===================================================================

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="paint_roller")
    model.materials.extend([CREAM, STEEL, CORAL, LATTICE_MAT])

    # --- Root: pink handle + fixed steel wire frame/cage -----------------
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

    # --- Moving child: roller cover with lattice open-cage core ----------
    roller = model.part("roller_cover")

    # Foam cover shell (outer visible surface)
    roller.visual(
        mesh_from_cadquery(_roller_cover_shape(), "roller_cover"),
        material=CREAM,
        name="roller_cover",
    )

    # Lattice longitudinal ribs (repeated sub-parts, loop + name_i)
    for i in range(N_RIBS):
        angle = 2.0 * math.pi * i / N_RIBS
        roller.visual(
            mesh_from_cadquery(_lattice_rib_shape(angle), f"lattice_rib_{i}"),
            material=LATTICE_MAT,
            name=f"lattice_rib_{i}",
        )

    # Lattice circumferential hoops (repeated sub-parts, loop + name_i)
    for i in range(N_HOOPS):
        roller.visual(
            mesh_from_cadquery(_lattice_hoop_shape(HOOP_POSITIONS[i]), f"lattice_hoop_{i}"),
            material=LATTICE_MAT,
            name=f"lattice_hoop_{i}",
        )

    # End spiders — axle journal bearings with radial arms (loop + name_i)
    spider_x = [HOOP_POSITIONS[0], HOOP_POSITIONS[-1]]
    for i in range(2):
        roller.visual(
            mesh_from_cadquery(_end_spider_shape(spider_x[i]), f"end_spider_{i}"),
            material=LATTICE_MAT,
            name=f"end_spider_{i}",
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


# ===================================================================
# Tests
# ===================================================================

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("handle_frame")
    roller = object_model.get_part("roller_cover")
    spin = object_model.get_articulation("frame_to_roller")

    # --- Axle captured by end spider journal bearings (intentional overlap) --
    for i in range(2):
        ctx.allow_overlap(
            frame,
            roller,
            elem_a="wire_frame",
            elem_b=f"end_spider_{i}",
            reason=(
                f"End spider {i} hub bore intentionally captures the steel axle "
                f"wire as a journal bearing; the cover spins on it."
            ),
        )

    for i in range(2):
        ctx.expect_overlap(
            frame,
            roller,
            axes="x",
            elem_a="wire_frame",
            elem_b=f"end_spider_{i}",
            min_overlap=0.001,
            name=f"axle wire passes through end spider {i} hub",
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

    # --- Lattice is an open cage (visible through hollow ends) -------------
    rib_dia = 2.0 * LATTICE_RIB_R
    bore_dia = 2.0 * ROLLER_BORE_R
    angular_gap = 2.0 * math.pi / N_RIBS
    chord_gap = 2.0 * LATTICE_CAGE_R * math.sin(angular_gap / 2.0)
    ctx.check(
        "lattice is an open cage visible through hollow ends",
        rib_dia < bore_dia * 0.20 and chord_gap > rib_dia * 2.0,
        details=(
            f"rib_dia={rib_dia:.4f}, bore_dia={bore_dia:.4f}, "
            f"chord_gap={chord_gap:.4f}"
        ),
    )

    # Lattice ribs span the roller length
    rib0 = roller.get_visual("lattice_rib_0")
    rib_aabb = ctx.part_element_world_aabb(roller, elem=rib0)
    assert rib_aabb is not None
    rib_len = rib_aabb[1][0] - rib_aabb[0][0]
    ctx.check(
        "lattice ribs span most of the roller length",
        rib_len > ROLLER_LEN * 0.88,
        details=f"rib_len={rib_len:.4f}, roller_len={ROLLER_LEN}",
    )

    # Lattice elements sit within the roller cover bore
    ctx.expect_within(
        roller,
        roller,
        axes="yz",
        inner_elem="lattice_rib_0",
        outer_elem="roller_cover",
        margin=0.001,
        name="lattice rib within roller cover bore",
    )
    ctx.expect_within(
        roller,
        roller,
        axes="yz",
        inner_elem="lattice_hoop_2",
        outer_elem="roller_cover",
        margin=0.001,
        name="lattice hoop within roller cover bore",
    )

    # Lattice contacts cover bore wall (not floating islands)
    ctx.expect_contact(
        roller,
        roller,
        elem_a="lattice_hoop_2",
        elem_b="roller_cover",
        contact_tol=0.001,
        name="lattice hoop seated against cover bore",
    )
    ctx.expect_contact(
        roller,
        roller,
        elem_a="lattice_rib_0",
        elem_b="roller_cover",
        contact_tol=0.001,
        name="lattice rib seated against cover bore",
    )

    # End spiders contact the lattice hoops at co-located positions
    ctx.expect_contact(
        roller,
        roller,
        elem_a="end_spider_0",
        elem_b="lattice_hoop_0",
        contact_tol=0.003,
        name="end spider 0 contacts nearest hoop",
    )
    ctx.expect_contact(
        roller,
        roller,
        elem_a="end_spider_1",
        elem_b=f"lattice_hoop_{N_HOOPS - 1}",
        contact_tol=0.003,
        name="end spider 1 contacts nearest hoop",
    )

    # --- Wire frame spans the roller and drops to the handle ----------------
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

    # --- Handle grip is below the axle, away from the roller, connected -----
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
    ctx.expect_contact(
        frame,
        frame,
        elem_a="wire_frame",
        elem_b="handle_grip",
        contact_tol=1e-6,
        name="wire frame plugs into handle grip",
    )

    # --- The roller actually spins: joint is poseable and stays centered -----
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

    # Roller cover must clear the pink handle grip
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
