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
    WirePath,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Sled-base bar stool — tan leather seat with curved wrap-around low backrest,
# on a continuous bent sled base made from two U-shaped side runners,
# with a round gold footrest ring and brushed-gold frame.
# ---------------------------------------------------------------------------
# Fork of swivel bar stool: base changed from pedestal+disc to sled runners.
# Seat, backrest, footrest ring, and swivel joint preserved.
# ---------------------------------------------------------------------------
# Real-world scale: seat top ~ 0.73 m. +Z is up; base sits on floor at z=0.
# Root (fixed):  sled base frame + footrest ring + struts.
# Child (swivels about Z): padded tan leather seat + tub/barrel backrest.
# ---------------------------------------------------------------------------

# Sled base frame
SIDE_OFFSET = 0.16          # Y offset of each side runner from center
FRONT_TOP_X = 0.10          # X position of front leg tops
BACK_TOP_X = 0.10           # X position of back leg tops
FRONT_FOOT_X = 0.18         # X position of front floor contacts
BACK_FOOT_X = 0.18          # X position of back floor contacts
RUNNER_TUBE_R = 0.014       # runner tube radius
CROSS_TUBE_R = 0.012        # cross-bar tube radius
FLOOR_Z = RUNNER_TUBE_R     # spline Z so tube bottom touches floor

PEDESTAL_TOP_Z = 0.62       # top of sled frame / swivel joint height

# Hub on the sled frame (swivel bearing housing)
HUB_RADIUS = 0.035
HUB_LENGTH = 0.012          # short, top flush with cross-bar tops

# Footrest
FOOTREST_Z = 0.23
FOOTREST_RING_RADIUS = 0.145
FOOTREST_TUBE_R = 0.009

# Seat cushion: square-ish, padded
SEAT_HALF_W = 0.205
SEAT_HALF_D = 0.200
SEAT_THICK = 0.080
SUPPORT_TOP_Z = PEDESTAL_TOP_Z + CROSS_TUBE_R   # 0.632 — top of cross-bars
PLATE_CENTER_Z = SUPPORT_TOP_Z + 0.009           # 0.641
PLATE_LENGTH = 0.016
SEAT_BOTTOM_Z = PLATE_CENTER_Z + PLATE_LENGTH / 2.0 + 0.002  # 0.651
SEAT_TOP_Z = SEAT_BOTTOM_Z + SEAT_THICK           # 0.731

# Swivel collar on the seat (captures around hub and center bar)
COLLAR_RADIUS = 0.048
COLLAR_LENGTH = 0.050
COLLAR_CENTER_WORLD_Z = PEDESTAL_TOP_Z + 0.010   # 0.630

# Backrest: tub/barrel form wrapping around back and sides
BACK_WRAP_R = 0.215
BACK_THICK = 0.058
BACK_HEIGHT = 0.235


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _sled_runner_points(side_y: float) -> list[tuple[float, float, float]]:
    """
    U-shaped sled runner control points for one side.
    Uses WirePath with Bezier curves for smooth bends.
    Floor runner goes front-to-back; two legs rise to seat-support height.
    """
    ftx, btx = FRONT_TOP_X, BACK_TOP_X
    ffx, bfx = FRONT_FOOT_X, BACK_FOOT_X
    fz = FLOOR_Z
    sh = PEDESTAL_TOP_Z
    sy = side_y

    wp = WirePath((ftx, sy, sh))
    # Front leg: straight down with slight outward splay
    wp.bezier_to(
        (ftx, sy, sh * 0.55),
        (ffx - 0.04, sy, 0.12),
        (ffx - 0.01, sy, fz + 0.04),
        samples=18,
    )
    # Front bend: curve from leg to floor
    wp.bezier_to(
        (ffx, sy, fz + 0.01),
        (ffx, sy, fz),
        (ffx * 0.6, sy, fz),
        samples=12,
    )
    # Floor section: straight runner front-to-back
    wp.line_to((-bfx * 0.6, sy, fz))
    # Back bend: curve from floor to leg
    wp.bezier_to(
        (-bfx, sy, fz),
        (-bfx, sy, fz + 0.01),
        (-bfx + 0.01, sy, fz + 0.04),
        samples=12,
    )
    # Back leg: up with slight outward splay
    wp.bezier_to(
        (-ffx + 0.04, sy, 0.12),
        (-btx, sy, sh * 0.55),
        (-btx, sy, sh),
        samples=18,
    )
    return wp.to_points()


def _sled_runner_mesh(side_y: float, name: str) -> object:
    """Bent tubular U-shaped sled runner for one side."""
    pts = _sled_runner_points(side_y)
    geom = tube_from_spline_points(
        pts,
        radius=RUNNER_TUBE_R,
        samples_per_segment=8,
        radial_segments=18,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, name)


def _cross_bar_mesh(x_pos: float, name: str) -> object:
    """Horizontal cross-bar connecting left and right runners."""
    pts = [
        (x_pos, -SIDE_OFFSET, PEDESTAL_TOP_Z),
        (x_pos, SIDE_OFFSET, PEDESTAL_TOP_Z),
    ]
    geom = tube_from_spline_points(
        pts,
        radius=CROSS_TUBE_R,
        samples_per_segment=4,
        radial_segments=12,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, name)


def _center_bar_mesh() -> object:
    """Longitudinal center bar connecting front and back cross-bars."""
    pts = [
        (FRONT_TOP_X, 0.0, PEDESTAL_TOP_Z),
        (-BACK_TOP_X, 0.0, PEDESTAL_TOP_Z),
    ]
    geom = tube_from_spline_points(
        pts,
        radius=CROSS_TUBE_R,
        samples_per_segment=4,
        radial_segments=12,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, "center_bar")


def _footrest_ring_mesh() -> object:
    """Gold torus-ring at footrest height."""
    seg = 80
    pts = []
    for i in range(seg + 1):
        a = 2.0 * math.pi * i / seg
        pts.append(
            (FOOTREST_RING_RADIUS * math.cos(a), FOOTREST_RING_RADIUS * math.sin(a), 0.0)
        )
    geom = tube_from_spline_points(
        pts,
        radius=FOOTREST_TUBE_R,
        closed_spline=True,
        radial_segments=16,
        cap_ends=False,
    )
    return mesh_from_geometry(geom, "footrest_ring")


def _footrest_strut_mesh(leg_x: float, leg_y: float, name: str) -> object:
    """Short diagonal strut from a runner leg to the footrest ring."""
    angle = math.atan2(leg_y, leg_x)
    ring_x = FOOTREST_RING_RADIUS * math.cos(angle)
    ring_y = FOOTREST_RING_RADIUS * math.sin(angle)
    p0 = (leg_x, leg_y, FOOTREST_Z)
    p1 = (ring_x, ring_y, FOOTREST_Z)
    geom = tube_from_spline_points(
        [p0, p1],
        radius=0.007,
        samples_per_segment=4,
        radial_segments=10,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, name)


def _seat_cushion_mesh() -> object:
    """
    Padded tan leather seat cushion: rounded-square slab, dome top.
    CadQuery box with vertical-edge and top-face fillets.
    Mesh spans z=0 to SEAT_THICK locally.
    """
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
    Built as a sweep of a rounded-rect cross-section along the arc path.
    Mesh is built in local space with bottom at z=0, top at z=BACK_HEIGHT.
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
# Build
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sled_bar_stool")

    model.material("gold", rgba=(0.85, 0.70, 0.30, 1.0))
    model.material("gold_dark", rgba=(0.68, 0.54, 0.22, 1.0))
    model.material("leather_tan", rgba=(0.73, 0.51, 0.36, 1.0))
    model.material("steel", rgba=(0.30, 0.31, 0.33, 1.0))

    # -------------------------------------------------------------------
    # Root part: sled base frame + footrest ring + struts
    # -------------------------------------------------------------------
    base = model.part("sled_base")

    # Two U-shaped side runners (right = 0, left = 1)
    for i in range(2):
        sy = SIDE_OFFSET if i == 0 else -SIDE_OFFSET
        base.visual(
            _sled_runner_mesh(sy, f"runner_{i}"),
            material="gold",
            name=f"runner_{i}",
        )

    # Front and back cross-bars connecting the runners at seat-support height
    base.visual(
        _cross_bar_mesh(FRONT_TOP_X, "front_crossbar"),
        material="gold",
        name="front_crossbar",
    )
    base.visual(
        _cross_bar_mesh(-BACK_TOP_X, "back_crossbar"),
        material="gold",
        name="back_crossbar",
    )

    # Longitudinal center bar for swivel support
    base.visual(
        _center_bar_mesh(),
        material="gold",
        name="center_bar",
    )

    # Swivel hub (short bearing housing)
    base.visual(
        Cylinder(radius=HUB_RADIUS, length=HUB_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_TOP_Z + HUB_LENGTH / 2.0)),
        material="gold_dark",
        name="hub",
    )

    # Footrest ring — gold torus at footrest height
    base.visual(
        _footrest_ring_mesh(),
        origin=Origin(xyz=(0.0, 0.0, FOOTREST_Z)),
        material="gold",
        name="footrest_ring",
    )

    # Four struts from runner legs to the footrest ring
    # Approximate leg X at footrest height (between top and foot positions)
    strut_leg_x = 0.14
    strut_configs = [
        (strut_leg_x, SIDE_OFFSET),      # 0: front-right
        (strut_leg_x, -SIDE_OFFSET),     # 1: front-left
        (-strut_leg_x, SIDE_OFFSET),     # 2: back-right
        (-strut_leg_x, -SIDE_OFFSET),    # 3: back-left
    ]
    for i, (lx, ly) in enumerate(strut_configs):
        base.visual(
            _footrest_strut_mesh(lx, ly, f"footrest_strut_{i}"),
            material="gold_dark",
            name=f"footrest_strut_{i}",
        )

    # -------------------------------------------------------------------
    # Child part: swiveling seat cushion + wrap-around barrel backrest.
    # child_local_z = world_z - PEDESTAL_TOP_Z
    # -------------------------------------------------------------------
    seat = model.part("seat_backrest")

    def sz(world_z: float) -> float:
        return world_z - PEDESTAL_TOP_Z

    # Gold swivel collar captures around the hub and center bar
    seat.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, sz(COLLAR_CENTER_WORLD_Z))),
        material="gold",
        name="swivel_collar",
    )

    # Thin steel seat-mount plate under the cushion
    seat.visual(
        Cylinder(radius=0.105, length=PLATE_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, sz(PLATE_CENTER_Z))),
        material="steel",
        name="seat_plate",
    )

    # Padded tan leather seat cushion (CadQuery rounded box)
    seat.visual(
        _seat_cushion_mesh(),
        origin=Origin(xyz=(0.0, 0.0, sz(SEAT_BOTTOM_Z))),
        material="leather_tan",
        name="cushion",
    )

    # Curved wrap-around low backrest — raised slightly above seat center
    # to clear the sled runners at the sides (fork-specific placement).
    back_local_z = sz(SEAT_TOP_Z + 0.020 - BACK_HEIGHT / 2.0)
    seat.visual(
        _backrest_mesh(),
        origin=Origin(xyz=(0.0, 0.0, back_local_z)),
        material="leather_tan",
        name="backrest",
    )

    # -------------------------------------------------------------------
    # Articulation: continuous swivel about +Z at the sled frame top.
    # Joint origin at the hub/center-bar contact surface.
    # -------------------------------------------------------------------
    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=10.0),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("sled_base")
    seat = object_model.get_part("seat_backrest")
    swivel = object_model.get_articulation("seat_swivel")

    # --- Intentional overlaps: swivel capture fit ---
    ctx.allow_overlap(
        seat, base,
        elem_a="swivel_collar", elem_b="hub",
        reason="Swivel collar captures around the hub bearing for rotation.",
    )
    ctx.allow_overlap(
        seat, base,
        elem_a="swivel_collar", elem_b="center_bar",
        reason="Swivel collar wraps around the longitudinal center bar.",
    )

    # Proof: hub is radially inside the collar
    ctx.expect_within(
        base, seat,
        axes="xy",
        inner_elem="hub",
        outer_elem="swivel_collar",
        margin=0.001,
        name="hub sits inside swivel collar radially",
    )

    # Proof: collar and center bar have significant XY overlap at joint
    ctx.expect_overlap(
        seat, base,
        axes="xy",
        elem_a="swivel_collar", elem_b="center_bar",
        min_overlap=0.020,
        name="collar captures center bar at swivel joint",
    )

    # --- Two U-shaped runners present ---
    for i in range(2):
        runner_aabb = ctx.part_element_world_aabb(base, elem=f"runner_{i}")
        assert runner_aabb is not None, f"runner_{i} missing"
        r_zmin = runner_aabb[0][2]
        r_zmax = runner_aabb[1][2]
        r_span = r_zmax - r_zmin
        ctx.check(
            f"runner_{i} touches floor",
            r_zmin < 0.025,
            details=f"zmin={r_zmin:.4f}",
        )
        ctx.check(
            f"runner_{i} reaches seat-support height",
            r_zmax > PEDESTAL_TOP_Z - 0.02,
            details=f"zmax={r_zmax:.4f}",
        )
        ctx.check(
            f"runner_{i} is tall (U-shaped)",
            r_span > 0.50,
            details=f"span={r_span:.3f}",
        )

    # Runners are on opposite sides (Y separation)
    r0_aabb = ctx.part_element_world_aabb(base, elem="runner_0")
    r1_aabb = ctx.part_element_world_aabb(base, elem="runner_1")
    assert r0_aabb is not None and r1_aabb is not None
    r0_y_center = (r0_aabb[0][1] + r0_aabb[1][1]) / 2.0
    r1_y_center = (r1_aabb[0][1] + r1_aabb[1][1]) / 2.0
    ctx.check(
        "runners are on opposite sides",
        abs(r0_y_center - r1_y_center) > 0.20,
        details=f"r0_y={r0_y_center:.3f}, r1_y={r1_y_center:.3f}",
    )

    # Floor contact span (sled extends front-to-back)
    r0_x_span = r0_aabb[1][0] - r0_aabb[0][0]
    ctx.check(
        "sled runners have significant floor contact length",
        r0_x_span > 0.20,
        details=f"runner_x_span={r0_x_span:.3f}",
    )

    # --- Cross-bars present ---
    fc_aabb = ctx.part_element_world_aabb(base, elem="front_crossbar")
    bc_aabb = ctx.part_element_world_aabb(base, elem="back_crossbar")
    ctx.check(
        "front crossbar present at seat height",
        fc_aabb is not None and fc_aabb[0][2] > PEDESTAL_TOP_Z - 0.02,
        details=f"fc_zmin={fc_aabb[0][2]:.4f}" if fc_aabb else "missing",
    )
    ctx.check(
        "back crossbar present at seat height",
        bc_aabb is not None and bc_aabb[0][2] > PEDESTAL_TOP_Z - 0.02,
        details=f"bc_zmin={bc_aabb[0][2]:.4f}" if bc_aabb else "missing",
    )

    # Cross-bars span between runners (Y width)
    if fc_aabb:
        fc_y_span = fc_aabb[1][1] - fc_aabb[0][1]
        ctx.check(
            "front crossbar spans between runners",
            fc_y_span > 0.25,
            details=f"fc_y_span={fc_y_span:.3f}",
        )

    # --- Hub present at center ---
    hub_aabb = ctx.part_element_world_aabb(base, elem="hub")
    ctx.check(
        "hub present at swivel center",
        hub_aabb is not None and abs(hub_aabb[0][2] - PEDESTAL_TOP_Z) < 0.01,
        details=f"hub_zmin={hub_aabb[0][2]:.4f}" if hub_aabb else "missing",
    )

    # --- Footrest ring: at mid-height, large diameter ---
    ring_aabb = ctx.part_element_world_aabb(base, elem="footrest_ring")
    assert ring_aabb is not None
    ring_w = ring_aabb[1][0] - ring_aabb[0][0]
    ring_z = (ring_aabb[0][2] + ring_aabb[1][2]) / 2.0
    ctx.check(
        "footrest ring is wide and at mid-height",
        ring_w > 0.26 and 0.12 < ring_z < 0.38,
        details=f"ring_width={ring_w:.3f}, ring_z={ring_z:.3f}",
    )

    # --- Four struts connect runners to ring ---
    for i in range(4):
        strut_aabb = ctx.part_element_world_aabb(base, elem=f"footrest_strut_{i}")
        ctx.check(
            f"footrest strut {i} present",
            strut_aabb is not None,
            details=f"strut_{i} aabb={strut_aabb}",
        )

    # --- Seat sits at bar-stool height (~0.70 m+) ---
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

    # --- Wrap-around backrest ---
    back_aabb = ctx.part_element_world_aabb(seat, elem="backrest")
    assert back_aabb is not None
    back_top = back_aabb[1][2]
    back_w = back_aabb[1][0] - back_aabb[0][0]
    ctx.check(
        "backrest rises above seat (low back tub form)",
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

    # Rotating the swivel must move the backrest but not the ring.
    back_before = ctx.part_element_world_aabb(seat, elem="backrest")
    with ctx.pose({swivel: math.pi / 2.0}):
        back_after = ctx.part_element_world_aabb(seat, elem="backrest")
        ring_after = ctx.part_element_world_aabb(base, elem="footrest_ring")
    assert back_before is not None and back_after is not None and ring_after is not None
    moved = (
        abs(back_after[0][1] - back_before[0][1]) > 0.03
        or abs(back_after[0][0] - back_before[0][0]) > 0.03
    )
    ctx.check(
        "swiveling moves the backrest (seat rotates)",
        moved,
        details=f"before_min={back_before[0]}, after_min={back_after[0]}",
    )
    ctx.check(
        "footrest ring stays fixed under swivel",
        abs(ring_after[0][0] - ring_aabb[0][0]) < 1e-5,
        details="ring is on the fixed root frame",
    )

    # --- Sled base is the visible support (no pedestal or disc base) ---
    visual_names = [v.name for v in base.visuals]
    ctx.check(
        "no pedestal in sled base variant",
        "pedestal" not in visual_names,
        details=f"visuals={visual_names}",
    )
    ctx.check(
        "no disc base in sled base variant",
        "disc_base" not in visual_names,
        details=f"visuals={visual_names}",
    )

    return ctx.report()


object_model = build_object_model()
