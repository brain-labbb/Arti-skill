from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Dining-leg bar stool — tan leather seat with curved wrap-around low
# backrest, on four fixed splayed tapered brushed-gold legs with box
# stretchers and a rectangular seat apron with central swivel hub.
# The seat swivels on the leg frame via a continuous Z-axis joint.
# ---------------------------------------------------------------------------
# Real-world scale: seat top ≈ 0.70 m. +Z is up; legs sit on floor at z=0.
# Root (fixed):  4 tapered splayed legs + apron frame + 4 stretchers.
# Child (swivels about Z): padded tan leather seat + tub/barrel backrest.
# ---------------------------------------------------------------------------

# --- Frame: legs ---

LEG_TOP_Z = 0.62           # top of legs / apron top (same as parent pedestal top)
LEG_TOP_R = 0.016          # leg radius at top
LEG_BOTTOM_R = 0.012       # leg radius at floor (slightly tapered)

# Leg corner positions: (top_xy, bottom_xy) for each of 4 legs.
# Order: 0=back-left, 1=back-right, 2=front-right, 3=front-left.
# Legs splay outward from top to bottom for stability.
LEG_CORNERS: list[tuple[tuple[float, float], tuple[float, float]]] = [
    ((-0.140, -0.120), (-0.185, -0.160)),   # back-left
    (( 0.140, -0.120), ( 0.185, -0.160)),   # back-right
    (( 0.140,  0.120), ( 0.185,  0.160)),   # front-right
    ((-0.140,  0.120), (-0.185,  0.160)),   # front-left
]

# --- Frame: apron ---

APRON_HW = 0.140           # half-width X (matches leg-top X spacing)
APRON_HD = 0.120           # half-depth Y (matches leg-top Y spacing)
APRON_THICK = 0.012        # board thickness
APRON_HEIGHT = 0.048       # board height (vertical extent)

# --- Frame: stretchers ---

STRETCHER_Z = 0.20         # stretcher centre-line height
STRETCHER_R = 0.007        # stretcher tube radius

# --- Seat (unchanged from parent) ---

SEAT_HALF_W = 0.205
SEAT_HALF_D = 0.200
SEAT_THICK = 0.080
SEAT_BOTTOM_Z = LEG_TOP_Z + 0.015
SEAT_TOP_Z = SEAT_BOTTOM_Z + SEAT_THICK

# --- Backrest (unchanged from parent) ---

BACK_WRAP_R = 0.215
BACK_THICK = 0.058
BACK_HEIGHT = 0.235


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _leg_pos_at_z(index: int, z: float) -> tuple[float, float, float]:
    """Interpolate leg centre position at a given height."""
    top_xy, bottom_xy = LEG_CORNERS[index]
    frac = z / LEG_TOP_Z
    x = bottom_xy[0] + frac * (top_xy[0] - bottom_xy[0])
    y = bottom_xy[1] + frac * (top_xy[1] - bottom_xy[1])
    return (x, y, z)


def _leg_mesh(index: int) -> object:
    """Tapered splayed leg via CadQuery loft between two circles."""
    top_xy, bottom_xy = LEG_CORNERS[index]
    bx, by = bottom_xy
    tx, ty = top_xy

    leg = (
        cq.Workplane("XY")
        .center(bx, by)
        .circle(LEG_BOTTOM_R)
        .workplane(offset=LEG_TOP_Z)
        .center(tx - bx, ty - by)
        .circle(LEG_TOP_R)
        .loft()
    )
    return mesh_from_cadquery(leg, f"leg_{index}")


def _apron_frame_mesh() -> object:
    """
    Rectangular seat apron: four boards forming a hollow rectangular frame,
    two cross-members (X and Y) connecting opposite boards at the top, and
    a central cylindrical hub for the swivel bearing mount.
    All built as one CadQuery solid for clean connectivity.
    """
    hw = APRON_HW
    hd = APRON_HD
    t = APRON_THICK
    h = APRON_HEIGHT

    # Rectangular hollow frame (outer rect minus inner rect, extruded)
    frame = (
        cq.Workplane("XY")
        .workplane(offset=LEG_TOP_Z - h)
        .rect(2 * hw, 2 * hd)
        .rect(2 * (hw - t), 2 * (hd - t))
        .extrude(h)
    )

    # Cross-member in X direction — extends slightly into the side boards
    cm_dim = 0.016                       # square cross-section
    cm_z = LEG_TOP_Z - cm_dim           # top flush with apron top
    cm_x = (
        cq.Workplane("XY")
        .workplane(offset=cm_z)
        .rect(2 * (hw - t * 0.4), cm_dim)
        .extrude(cm_dim)
    )
    frame = frame.union(cm_x)

    # Cross-member in Y direction
    cm_y = (
        cq.Workplane("XY")
        .workplane(offset=cm_z)
        .rect(cm_dim, 2 * (hd - t * 0.4))
        .extrude(cm_dim)
    )
    frame = frame.union(cm_y)

    # Central hub for swivel bearing — cylinder rising from cross-members
    hub_r = 0.032
    hub_h = 0.022
    hub = (
        cq.Workplane("XY")
        .workplane(offset=LEG_TOP_Z - hub_h)
        .circle(hub_r)
        .extrude(hub_h)
    )
    frame = frame.union(hub)

    return mesh_from_cadquery(frame, "apron_frame")


def _stretcher_mesh(index: int) -> object:
    """
    Box-pattern stretcher tube connecting adjacent legs at STRETCHER_Z.
    0: back (leg 0 → 1), 1: right (1 → 2), 2: front (2 → 3), 3: left (3 → 0).
    """
    pairs = [(0, 1), (1, 2), (2, 3), (3, 0)]
    i0, i1 = pairs[index]
    p0 = _leg_pos_at_z(i0, STRETCHER_Z)
    p1 = _leg_pos_at_z(i1, STRETCHER_Z)

    geom = tube_from_spline_points(
        [p0, p1],
        radius=STRETCHER_R,
        samples_per_segment=4,
        radial_segments=10,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, f"stretcher_{index}")


def _seat_cushion_mesh() -> object:
    """Padded tan leather seat cushion: rounded-square slab with dome top."""
    w = SEAT_HALF_W * 2.0
    d = SEAT_HALF_D * 2.0
    t = SEAT_THICK
    result = (
        cq.Workplane("XY")
        .box(w, d, t, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.045)
        .edges(">Z")
        .fillet(t * 0.25)
    )
    return mesh_from_cadquery(result, "seat_cushion")


def _backrest_mesh() -> object:
    """
    Curved wrap-around tub/barrel low backrest in tan leather.
    Wraps ~220° around the back and sides of the seat (opens toward +Y front).
    """
    half_deg = 110.0
    half_rad = math.radians(half_deg)
    n = 36

    pts = []
    for i in range(n + 1):
        frac = i / n
        a = half_rad - 2.0 * half_rad * frac
        cx, cy = 0.0, -BACK_WRAP_R * 0.05
        x = cx + BACK_WRAP_R * math.sin(a)
        y = cy - BACK_WRAP_R * math.cos(a)
        pts.append((x, y, BACK_HEIGHT / 2.0))

    profile = rounded_rect_profile(BACK_THICK, BACK_HEIGHT, radius=0.018)

    geom = sweep_profile_along_spline(
        pts,
        profile=profile,
        samples_per_segment=4,
        cap_profile=True,
        up_hint=(0.0, 0.0, 1.0),
    )
    return mesh_from_geometry(geom, "backrest")


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dining_leg_bar_stool")

    model.material("gold", rgba=(0.85, 0.70, 0.30, 1.0))
    model.material("gold_dark", rgba=(0.68, 0.54, 0.22, 1.0))
    model.material("leather_tan", rgba=(0.73, 0.51, 0.36, 1.0))
    model.material("steel", rgba=(0.30, 0.31, 0.33, 1.0))

    # -------------------------------------------------------------------
    # Root part: fixed leg frame (4 legs + apron + 4 stretchers)
    # -------------------------------------------------------------------
    frame = model.part("leg_frame")

    for i in range(4):
        frame.visual(
            _leg_mesh(i),
            material="gold",
            name=f"leg_{i}",
        )

    frame.visual(
        _apron_frame_mesh(),
        material="gold_dark",
        name="apron_frame",
    )

    for i in range(4):
        frame.visual(
            _stretcher_mesh(i),
            material="gold_dark",
            name=f"stretcher_{i}",
        )

    # -------------------------------------------------------------------
    # Child part: swiveling seat + wrap-around backrest.
    # child_local_z = world_z - LEG_TOP_Z
    # -------------------------------------------------------------------
    seat = model.part("seat_backrest")

    def sz(world_z: float) -> float:
        return world_z - LEG_TOP_Z

    # Gold swivel collar wraps around the apron hub
    seat.visual(
        Cylinder(radius=0.048, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, sz(LEG_TOP_Z + 0.003))),
        material="gold",
        name="swivel_collar",
    )

    # Thin steel seat-mount plate under the cushion
    seat.visual(
        Cylinder(radius=0.105, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, sz(SEAT_BOTTOM_Z - 0.006))),
        material="steel",
        name="seat_plate",
    )

    # Padded tan leather seat cushion
    seat.visual(
        _seat_cushion_mesh(),
        origin=Origin(xyz=(0.0, 0.0, sz(SEAT_BOTTOM_Z))),
        material="leather_tan",
        name="cushion",
    )

    # Curved wrap-around low backrest
    # Raise bottom above the leg tops (z=0.62) to avoid corner-leg overlap.
    back_local_z = 0.005
    seat.visual(
        _backrest_mesh(),
        origin=Origin(xyz=(0.0, 0.0, back_local_z)),
        material="leather_tan",
        name="backrest",
    )

    # -------------------------------------------------------------------
    # Articulation: continuous swivel about +Z at the apron-top hub.
    # -------------------------------------------------------------------
    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, LEG_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=10.0),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("leg_frame")
    seat = object_model.get_part("seat_backrest")
    swivel = object_model.get_articulation("seat_swivel")

    # --- Intentional overlap: swivel collar wraps around the apron hub ---
    ctx.allow_overlap(
        frame, seat,
        elem_a="apron_frame", elem_b="swivel_collar",
        reason="Swivel collar captures around the apron hub for the bearing mount.",
    )

    # --- Four legs present and reaching the floor ---
    for i in range(4):
        leg_aabb = ctx.part_element_world_aabb(frame, elem=f"leg_{i}")
        ctx.check(
            f"leg_{i} present and on floor",
            leg_aabb is not None and leg_aabb[0][2] < 0.005,
            details=f"leg_{i} z_min={leg_aabb[0][2]:.4f}" if leg_aabb else "missing",
        )

    # --- Legs are splayed: overall frame wider at bottom than top ---
    leg0_aabb = ctx.part_element_world_aabb(frame, elem="leg_0")
    leg2_aabb = ctx.part_element_world_aabb(frame, elem="leg_2")
    if leg0_aabb and leg2_aabb:
        # Diagonal span (back-left to front-right) at full extent
        diag_span_x = leg2_aabb[1][0] - leg0_aabb[0][0]
        ctx.check(
            "legs splay outward (diagonal span > 0.34 m)",
            diag_span_x > 0.34,
            details=f"diagonal_x_span={diag_span_x:.3f}",
        )

    # --- Four stretchers present at mid-height ---
    for i in range(4):
        s_aabb = ctx.part_element_world_aabb(frame, elem=f"stretcher_{i}")
        ctx.check(
            f"stretcher_{i} present",
            s_aabb is not None,
            details=f"stretcher_{i} aabb={s_aabb}",
        )
        if s_aabb is not None:
            s_z = (s_aabb[0][2] + s_aabb[1][2]) / 2.0
            ctx.check(
                f"stretcher_{i} at mid-height",
                0.10 < s_z < 0.35,
                details=f"stretcher_{i} z_centre={s_z:.3f}",
            )

    # --- Apron frame present ---
    apron_aabb = ctx.part_element_world_aabb(frame, elem="apron_frame")
    ctx.check(
        "apron frame present",
        apron_aabb is not None,
        details=f"apron aabb={apron_aabb}",
    )

    # --- Seat at bar-stool height ---
    cushion_aabb = ctx.part_element_world_aabb(seat, elem="cushion")
    assert cushion_aabb is not None
    seat_top = cushion_aabb[1][2]
    ctx.check(
        "seat at bar height",
        0.66 <= seat_top <= 0.80,
        details=f"seat_top={seat_top:.3f}",
    )

    # --- Cushion is roughly square and padded ---
    sx = cushion_aabb[1][0] - cushion_aabb[0][0]
    sy = cushion_aabb[1][1] - cushion_aabb[0][1]
    ctx.check(
        "seat cushion is square-ish and wide",
        sx > 0.36 and sy > 0.36 and abs(sx - sy) < 0.04,
        details=f"sx={sx:.3f}, sy={sy:.3f}",
    )

    # --- Backrest rises above seat ---
    back_aabb = ctx.part_element_world_aabb(seat, elem="backrest")
    assert back_aabb is not None
    back_top = back_aabb[1][2]
    back_w = back_aabb[1][0] - back_aabb[0][0]
    ctx.check(
        "backrest rises above seat",
        back_top > seat_top + 0.06 and back_top < seat_top + 0.32,
        details=f"back_top={back_top:.3f}, seat_top={seat_top:.3f}",
    )
    ctx.check(
        "backrest is wide (wrap-around)",
        back_w > 0.28,
        details=f"back_width={back_w:.3f}",
    )
    ctx.check(
        "backrest bottom near seat level",
        back_aabb[0][2] <= seat_top + 0.02,
        details=f"back_bottom={back_aabb[0][2]:.3f}, seat_top={seat_top:.3f}",
    )

    # --- Swivel joint: continuous about Z ---
    ctx.check(
        "seat_swivel is continuous about Z",
        swivel.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(c, 3) for c in swivel.axis) == (0.0, 0.0, 1.0),
        details=f"type={swivel.articulation_type}, axis={swivel.axis}",
    )

    # --- Swiveling moves the backrest ---
    back_before = ctx.part_element_world_aabb(seat, elem="backrest")
    with ctx.pose({swivel: math.pi / 2.0}):
        back_after = ctx.part_element_world_aabb(seat, elem="backrest")
    assert back_before is not None and back_after is not None
    moved = (
        abs(back_after[0][1] - back_before[0][1]) > 0.03
        or abs(back_after[0][0] - back_before[0][0]) > 0.03
    )
    ctx.check(
        "swivelling moves the backrest",
        moved,
        details=f"before_min={back_before[0]}, after_min={back_after[0]}",
    )

    # --- Legs stay fixed under swivel ---
    leg0_before = ctx.part_element_world_aabb(frame, elem="leg_0")
    with ctx.pose({swivel: math.pi / 2.0}):
        leg0_after = ctx.part_element_world_aabb(frame, elem="leg_0")
    assert leg0_before is not None and leg0_after is not None
    ctx.check(
        "legs stay fixed under swivel",
        abs(leg0_after[0][0] - leg0_before[0][0]) < 1e-5,
        details="legs are on the fixed root frame",
    )

    # --- Fork verification: no pedestal, disc base, or footrest ring ---
    parent_elements = ("pedestal", "disc_base", "footrest_ring")
    for elem_name in parent_elements:
        try:
            aabb = ctx.part_element_world_aabb(frame, elem=elem_name)
            present = aabb is not None
        except Exception:
            present = False
        ctx.check(
            f"no {elem_name} (fork: legs replace pedestal base)",
            not present,
            details=f"{elem_name} should not exist in the leg variant",
        )

    # --- Seat gap above apron: seat plate clears the apron top ---
    ctx.expect_gap(
        seat, frame,
        axis="z",
        positive_elem="seat_plate",
        negative_elem="apron_frame",
        max_penetration=0.004,
        name="seat plate clears the apron frame top",
    )

    return ctx.report()


object_model = build_object_model()
