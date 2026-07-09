from __future__ import annotations

# Articraft model: a straight-shank paint roller (inline fork variant).
#
# Fork of the original Z-crank paint roller. The bent wire frame is replaced
# with a straight in-line shank that runs from the roller axle directly into a
# collar adapter on the grip, with no downward crank bend. The roller still
# free-spins on the shank as a journal bearing.
#
# Structure:
#   Root  (handle_frame): straight steel shank + zinc collar adapter + coral grip
#   Child (roller_cover): cream hollow cover + core tube + 2 end-cap rings
#   Joint: CONTINUOUS revolute about +X (roller spins on the shank axle)
#
# Frame convention: roller long axis = X, roller centered at x=0.
# The shank exits the +X roller face and continues straight into the collar
# and grip, all on the same axle height (AXLE_Z = 0).

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
)

# ---------------------------------------------------------------------------
# Real-world dimensions (meters)
# ---------------------------------------------------------------------------
ROLLER_LEN = 0.180          # roller cover cylindrical length (along X)
ROLLER_OUTER_R = 0.0190     # roller cover outer radius (~38 mm dia)
ROLLER_BORE_R = 0.0078      # inner core bore radius (visible hollow ends)

AXLE_R = 0.0028             # steel shank wire radius (~5.6 mm)

ROLLER_X_MIN = -ROLLER_LEN / 2.0   # free (far) end, toward -X
ROLLER_X_MAX = ROLLER_LEN / 2.0    # handle-side end, toward +X

AXLE_Z = 0.0                       # roller axle line height

# Straight in-line shank stations (no Z drop)
AXLE_FAR_X = ROLLER_X_MIN - 0.006          # capped stub past the far end
AXLE_NEAR_X = ROLLER_X_MAX + 0.012         # shank clears the +X roller face
COLLAR_X_MIN = AXLE_NEAR_X                 # collar adapter start
COLLAR_LEN = 0.020                         # collar body length
COLLAR_R = 0.006                           # collar outer radius
SOCKET_X = COLLAR_X_MIN + COLLAR_LEN       # grip start (collar end)

HANDLE_LEN = 0.130          # coral grip length (along X)

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------
CREAM = Material(name="roller_cover_cream", rgba=(0.93, 0.91, 0.84, 1.0))
STEEL = Material(name="frame_steel", rgba=(0.62, 0.63, 0.65, 1.0))
CORAL = Material(name="handle_coral", rgba=(0.86, 0.45, 0.43, 1.0))
END_CAP = Material(name="roller_endcap", rgba=(0.75, 0.73, 0.68, 1.0))
ZINC = Material(name="collar_zinc", rgba=(0.50, 0.52, 0.54, 1.0))


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
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
    that runs on the steel shank."""
    core_outer_r = ROLLER_BORE_R + 0.0003  # press fit into the cover bore
    core_inner_r = AXLE_R - 0.0006         # captures the shank (journal fit)
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


def _straight_shank_shape() -> cq.Workplane:
    """Straight steel shank: a cylinder running from the far stub, through the
    roller bore, and into the collar adapter -- all on one axis (no crank)."""
    length = (COLLAR_X_MIN + 0.010) - AXLE_FAR_X  # extends 10 mm into collar
    return (
        cq.Workplane("YZ")
        .circle(AXLE_R)
        .extrude(length)
        .translate((AXLE_FAR_X, 0.0, AXLE_Z))
    )


def _collar_shape() -> cq.Workplane:
    """Zinc collar/ferrule adapter: a solid cylinder that the shank plunges
    into on one side and the grip receives on the other. Extends 2 mm past
    SOCKET_X into the grip for a seated structural connection."""
    return (
        cq.Workplane("YZ")
        .circle(COLLAR_R)
        .extrude(COLLAR_LEN + 0.002)
        .translate((COLLAR_X_MIN, 0.0, AXLE_Z))
    )


def _handle_shape() -> cq.Workplane:
    """Coral molded grip: a rounded, gently barrel-tapered body, revolved
    about the X axis at the axle height (inline with the roller)."""
    x0 = SOCKET_X
    x1 = SOCKET_X + HANDLE_LEN
    pts = [
        (x0, 0.0),
        (x0, 0.0085),             # collar neck (receives the collar)
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
    handle = handle.translate((0.0, 0.0, AXLE_Z))
    return handle


def _end_cap_ring_shape(x_end: float) -> cq.Workplane:
    """Small plastic end-cap ring flush with a roller end face, extending
    inward into the cover wall for a seated connection."""
    outer_r = ROLLER_OUTER_R - 0.002   # recessed from the cover OD
    inner_r = AXLE_R + 0.001           # axle clearance
    thickness = 0.004
    # Ring extends inward from the end face
    x_start = x_end if x_end < 0 else x_end - thickness
    return (
        cq.Workplane("YZ")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(thickness)
        .translate((x_start, 0.0, AXLE_Z))
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="paint_roller")
    model.materials.extend([CREAM, STEEL, CORAL, END_CAP, ZINC])

    # --- Root: straight shank + collar adapter + handle grip ----------------
    frame = model.part("handle_frame")
    frame.visual(
        mesh_from_cadquery(_straight_shank_shape(), "shank"),
        material=STEEL,
        name="shank",
    )
    frame.visual(
        mesh_from_cadquery(_collar_shape(), "collar"),
        material=ZINC,
        name="collar",
    )
    frame.visual(
        mesh_from_cadquery(_handle_shape(), "handle_grip"),
        material=CORAL,
        name="handle_grip",
    )

    # --- Moving child: roller cover (spins on the shank) --------------------
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
    # Repeated end-cap rings at each roller end (shared helper, for-i loop)
    for i in range(2):
        x_pos = ROLLER_X_MIN if i == 0 else ROLLER_X_MAX
        roller.visual(
            mesh_from_cadquery(_end_cap_ring_shape(x_pos), f"end_cap_{i}"),
            material=END_CAP,
            name=f"end_cap_{i}",
        )

    # The roller spins freely about the axle (world +X). Joint frame at the
    # roller center on the axle line -- the actual bearing contact surface.
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


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("handle_frame")
    roller = object_model.get_part("roller_cover")
    spin = object_model.get_articulation("frame_to_roller")

    # --- Shank journaled inside roller core (journal bearing) ---------------
    ctx.allow_overlap(
        frame,
        roller,
        elem_a="shank",
        elem_b="roller_core",
        reason=(
            "The straight shank is intentionally captured inside the roller "
            "core bore; the cover spins on it as a journal bearing."
        ),
    )

    ctx.expect_overlap(
        frame,
        roller,
        axes="x",
        elem_a="shank",
        elem_b="roller_core",
        min_overlap=0.15,
        name="shank runs through the roller core",
    )

    # --- Joint type and axis ------------------------------------------------
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

    # --- THE KEY CHANGE: straight shank, no Z crank drop --------------------
    shank_bb = ctx.part_element_world_aabb(frame, elem="shank")
    assert shank_bb is not None
    shank_z_span = shank_bb[1][2] - shank_bb[0][2]
    ctx.check(
        "shank is straight with no Z crank drop",
        shank_z_span < 0.010,
        details=f"shank z_span={shank_z_span:.4f}",
    )

    # --- Handle is inline with the roller axle (no drop) --------------------
    grip_bb = ctx.part_element_world_aabb(frame, elem="handle_grip")
    assert grip_bb is not None
    handle_center_z = (grip_bb[0][2] + grip_bb[1][2]) / 2.0
    ctx.check(
        "handle grip is inline with roller axle (no drop)",
        abs(handle_center_z - AXLE_Z) < 0.005,
        details=f"handle_center_z={handle_center_z:.4f}, axle_z={AXLE_Z}",
    )

    # --- Collar adapter sits between roller and handle ----------------------
    collar_bb = ctx.part_element_world_aabb(frame, elem="collar")
    assert collar_bb is not None
    ctx.check(
        "collar adapter sits between roller and handle",
        collar_bb[0][0] >= ROLLER_X_MAX - 0.002
        and collar_bb[1][0] <= grip_bb[1][0] + 0.002,
        details=f"collar x[{collar_bb[0][0]:.3f},{collar_bb[1][0]:.3f}]",
    )

    # --- Shank-collar and collar-handle connectivity ------------------------
    ctx.expect_contact(
        frame,
        frame,
        elem_a="shank",
        elem_b="collar",
        contact_tol=0.002,
        name="shank enters collar adapter",
    )
    ctx.expect_contact(
        frame,
        frame,
        elem_a="collar",
        elem_b="handle_grip",
        contact_tol=0.002,
        name="collar adapter contacts handle grip",
    )

    # --- Roller cover is a long hollow cylinder -----------------------------
    cover = roller.get_visual("roller_cover")
    cover_bb = ctx.part_element_world_aabb(roller, elem=cover)
    assert cover_bb is not None
    length_x = cover_bb[1][0] - cover_bb[0][0]
    dia_y = cover_bb[1][1] - cover_bb[0][1]
    dia_z = cover_bb[1][2] - cover_bb[0][2]
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

    # --- Core seated in cover bore (not a floating island) ------------------
    ctx.expect_contact(
        roller,
        roller,
        elem_a="roller_core",
        elem_b="roller_cover",
        contact_tol=0.0006,
        name="roller core seated in cover bore",
    )

    # --- End-cap rings at roller ends ---------------------------------------
    for i in range(2):
        cap_bb = ctx.part_element_world_aabb(roller, elem=f"end_cap_{i}")
        assert cap_bb is not None
        expected_x = ROLLER_X_MIN if i == 0 else ROLLER_X_MAX
        cap_center_x = (cap_bb[0][0] + cap_bb[1][0]) / 2.0
        ctx.check(
            f"end_cap_{i} at roller end",
            abs(cap_center_x - expected_x) < 0.005,
            details=f"cap_center_x={cap_center_x:.4f}, expected={expected_x:.4f}",
        )
        ctx.expect_contact(
            roller,
            roller,
            elem_a=f"end_cap_{i}",
            elem_b="roller_cover",
            contact_tol=0.003,
            name=f"end_cap_{i} contacts roller cover",
        )

    # --- Roller actually spins: stays centered on the axle ------------------
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

    # --- Handle grip clears the roller cover --------------------------------
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
