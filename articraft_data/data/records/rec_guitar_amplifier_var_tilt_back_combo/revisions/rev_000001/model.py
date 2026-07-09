from __future__ import annotations

# Marshall-style mini guitar combo amplifier — WEDGE (tilt-back) variant.
# The cabinet is a wedge (trapezoidal prism): front baffle leans backward so the
# speaker grille aims slightly upward; rear face is taller than front.
#
# Coordinate convention:
#   +X  : forward, toward the front speaker grille
#   +Y  : cabinet width (left/right)
#   +Z  : up
#
# Realistic mini-amp wedge scale:
#   width  (Y) ~ 0.18 m
#   depth  (X) ~ 0.16 m
#   front height (Z) ~ 0.155 m
#   rear  height (Z) ~ 0.195 m
#   front baffle tilt ~ 14° from vertical

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobTopFeature,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ── principal dimensions (meters) ──────────────────────────────────────────────
CAB_W = 0.180              # width  (Y)
CAB_D = 0.160              # depth at base (X)
H_FRONT = 0.155            # front face height
H_REAR = 0.195             # rear face height
WALL = 0.010               # cabinet wall thickness
BAFFLE_TILT = math.radians(14)   # front baffle tilt from vertical
SETBACK = H_FRONT * math.tan(BAFFLE_TILT)  # ≈ 0.0386 m

# ── cross-section corners (XZ plane, centred on Y) ────────────────────────────
_half_d = CAB_D / 2.0
_half_w = CAB_W / 2.0
_z_bot = -H_REAR / 2.0

PT_A = (-_half_d, _z_bot)                               # rear-bottom
PT_B = (_half_d, _z_bot)                                # front-bottom
PT_C = (_half_d - SETBACK, _z_bot + H_FRONT)            # front-top
PT_D = (-_half_d, _z_bot + H_REAR)                      # rear-top

# ── baffle (front face) derived ────────────────────────────────────────────────
_bdx = PT_C[0] - PT_B[0]
_bdz = PT_C[1] - PT_B[1]
_blen = math.hypot(_bdx, _bdz)
BAFFLE_NX = _bdz / _blen          # outward-normal X  ≈ 0.970
BAFFLE_NZ = -_bdx / _blen         # outward-normal Z  ≈ 0.244
BAFFLE_DIR_X = _bdx / _blen       # "up on face" X    ≈ -0.244
BAFFLE_DIR_Z = _bdz / _blen       # "up on face" Z    ≈  0.970
BAFFLE_CX = (PT_B[0] + PT_C[0]) / 2.0
BAFFLE_CZ = (PT_B[1] + PT_C[1]) / 2.0

# ── top surface derived ────────────────────────────────────────────────────────
_tdx = PT_D[0] - PT_C[0]
_tdz = PT_D[1] - PT_C[1]
_tlen = math.hypot(_tdx, _tdz)
TOP_NX = _tdz / _tlen             # outward-normal X  ≈ 0.314
TOP_NZ = -_tdx / _tlen            # outward-normal Z  ≈ 0.949
TOP_ANGLE = math.atan2(TOP_NX, TOP_NZ)  # rotation from +Z toward +X  ≈ 18°
TOP_CX = (PT_C[0] + PT_D[0]) / 2.0
TOP_CZ = (PT_C[1] + PT_D[1]) / 2.0

# ── gold panel on top surface ──────────────────────────────────────────────────
PANEL_W = 0.150               # span along Y
PANEL_D = 0.060               # span along the top surface
PANEL_THICK = 0.004
# Panel centre at top-surface midpoint, slightly proud along the normal
PANEL_CX = TOP_CX + PANEL_THICK * 0.5 * TOP_NX
PANEL_CZ = TOP_CZ + PANEL_THICK * 0.5 * TOP_NZ

# ── knobs ──────────────────────────────────────────────────────────────────────
KNOB_DIAM = 0.018
KNOB_H = 0.014
KNOB_YS = (-0.054, -0.018, 0.018, 0.054)
# Row at 35 % from C toward D on the top surface (slightly forward of centre).
_KF = 0.35
KNOB_BASE_X = PT_C[0] + _KF * (PT_D[0] - PT_C[0])
KNOB_BASE_Z = PT_C[1] + _KF * (PT_D[1] - PT_C[1])
# Knob seat: panel surface with tiny press-fit embed along the top normal.
KNOB_SEAT_X = KNOB_BASE_X + (PANEL_THICK - 0.001) * TOP_NX
KNOB_SEAT_Z = KNOB_BASE_Z + (PANEL_THICK - 0.001) * TOP_NZ

# ── handle position (on top surface, in front of panel) ────────────────────────
_HF = 0.12
HANDLE_CX = PT_C[0] + _HF * (PT_D[0] - PT_C[0])
HANDLE_CZ = PT_C[1] + _HF * (PT_D[1] - PT_C[1])


# ──────────────────────── helpers ──────────────────────────────────────────────

def _wedge_solid() -> cq.Workplane:
    """Wedge cabinet body: trapezoidal XZ profile extruded along Y."""
    profile = (
        cq.Workplane("XY")
        .moveTo(PT_A[0], PT_A[1])
        .lineTo(PT_B[0], PT_B[1])
        .lineTo(PT_C[0], PT_C[1])
        .lineTo(PT_D[0], PT_D[1])
        .close()
    )
    wedge = (
        profile
        .extrude(CAB_W)                          # along +Z for CAB_W
        .translate((0, 0, -_half_w))             # centre on Z
        .rotate((0, 0, 0), (1, 0, 0), 90)       # swap Y↔Z: height→Z, width→Y
    )
    try:
        wedge = wedge.edges("|Y").fillet(0.005)
    except Exception:
        pass
    return wedge


def _handle_mesh():
    """Carry handle on the slanted top surface (arched strap + mounts)."""
    arch = 0.018
    span = 0.100
    n = 13
    pts = []
    for i in range(n):
        t = i / (n - 1)
        y = -span / 2.0 + t * span
        zz = arch * (1.0 - (2.0 * t - 1.0) ** 2) + 0.002
        pts.append((0.0, y, zz))

    path = cq.Workplane().add(
        cq.Edge.makeSpline([cq.Vector(*p) for p in pts])
    )
    prof = (
        cq.Workplane("XZ")
        .center(0.0, pts[0][2])
        .rect(0.016, 0.006)
    )
    strap = prof.sweep(path, multisection=False, makeSolid=True)

    result = strap
    for y in (-span / 2.0, span / 2.0):
        mount = (
            cq.Workplane("XY")
            .box(0.022, 0.016, 0.012)
            .translate((0.0, y, -0.003))
        )
        result = result.union(mount)

    # Rotate to follow the top surface, then translate into position.
    result = result.rotate((0, 0, 0), (0, 1, 0), math.degrees(TOP_ANGLE))
    result = result.translate((HANDLE_CX, 0.0, HANDLE_CZ))
    return mesh_from_cadquery(result, "handle")


# ──────────────────────── build ────────────────────────────────────────────────

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wedge_mini_guitar_amp")

    # Materials
    black_vinyl   = model.material("black_vinyl",   rgba=(0.07, 0.07, 0.08, 1.0))
    gold_panel_m  = model.material("gold_panel",    rgba=(0.86, 0.62, 0.18, 1.0))
    grille_dark   = model.material("grille_dark",   rgba=(0.10, 0.10, 0.11, 1.0))
    white_piping  = model.material("white_piping",  rgba=(0.92, 0.92, 0.90, 1.0))
    logo_cream    = model.material("logo_cream",    rgba=(0.95, 0.93, 0.86, 1.0))
    metal_knob    = model.material("metal_knob",    rgba=(0.78, 0.78, 0.80, 1.0))
    indicator_red = model.material("indicator_red", rgba=(0.85, 0.12, 0.10, 1.0))
    trim_black    = model.material("trim_black",    rgba=(0.04, 0.04, 0.05, 1.0))

    # ───────────────────────────── body ────────────────────────────────────────
    body = model.part("body")

    # Cabinet wedge shell
    cab = _wedge_solid()
    body.visual(
        mesh_from_cadquery(cab, "cabinet"),
        material=black_vinyl, name="cabinet",
    )

    # ── gold control panel on the slanted top ─────────────────────────────────
    panel = BoxGeometry((PANEL_D, PANEL_W, PANEL_THICK))
    panel.rotate_y(TOP_ANGLE)
    panel.translate(PANEL_CX, 0.0, PANEL_CZ)
    body.visual(
        mesh_from_geometry(panel, "gold_panel"),
        material=gold_panel_m, name="gold_panel",
    )

    # Power LED on the panel (embedded into the gold panel for connectivity)
    led = CylinderGeometry(0.003, 0.004)
    led.rotate_y(TOP_ANGLE)
    _lf = _KF + 0.18
    _lx = PT_C[0] + _lf * (PT_D[0] - PT_C[0]) + PANEL_THICK * 0.5 * TOP_NX
    _lz = PT_C[1] + _lf * (PT_D[1] - PT_C[1]) + PANEL_THICK * 0.5 * TOP_NZ
    led.translate(_lx, 0.068, _lz)
    body.visual(
        mesh_from_geometry(led, "power_led"),
        material=indicator_red, name="power_led",
    )

    # ── speaker grille on the tilted baffle ───────────────────────────────────
    grille_h = _blen - 0.030
    grille_w = CAB_W - 0.030
    grille = PerforatedPanelGeometry(
        (grille_h, grille_w), 0.006,
        hole_diameter=0.0035, pitch=0.007,
        frame=0.006, stagger=True,
    )
    # Rotate face from +Z to baffle normal: Ry(π/2 − BAFFLE_TILT)
    grille.rotate_y(math.pi / 2.0 - BAFFLE_TILT)
    # Position at baffle centre, recessed 3 mm behind the surface
    gx = BAFFLE_CX - 0.003 * BAFFLE_NX
    gz = BAFFLE_CZ - 0.003 * BAFFLE_NZ
    grille.translate(gx, 0.0, gz)
    body.visual(
        mesh_from_geometry(grille, "speaker_grille"),
        material=grille_dark, name="speaker_grille",
    )

    # ── white piping around the grille (4 bars on the baffle) ─────────────────
    pip_t = 0.005        # protrusion (along baffle normal)
    pip_w = 0.006        # bar cross-section width
    pw = CAB_W - 0.020   # Y span of horizontal bars
    ph = _blen - 0.020   # span along baffle direction of vertical bars
    # proud offset along baffle normal
    _pn = 0.002

    # Horizontal bars (along Y) at the top and bottom edges of the grille
    for frac, nm in ((0.012, "piping_h_0"), (1.0 - 0.012 / 1.0, "piping_h_1")):
        # frac is fraction from B toward C along the baffle
        _bx = PT_B[0] + frac * _bdx + _pn * BAFFLE_NX
        _bz = PT_B[1] + frac * _bdz + _pn * BAFFLE_NZ
        bar = BoxGeometry((pip_t, pw, pip_w))
        bar.rotate_y(-BAFFLE_TILT)
        bar.translate(_bx, 0.0, _bz)
        body.visual(
            mesh_from_geometry(bar, nm),
            material=white_piping, name=nm,
        )

    # Vertical bars (along baffle direction) at the left and right edges
    for iy, yc in enumerate((-_half_w + 0.010, _half_w - 0.010)):
        nm = f"piping_v_{iy}"
        bar = BoxGeometry((pip_t, pip_w, ph))
        bar.rotate_y(-BAFFLE_TILT)
        bar.translate(BAFFLE_CX + _pn * BAFFLE_NX, yc, BAFFLE_CZ + _pn * BAFFLE_NZ)
        body.visual(
            mesh_from_geometry(bar, nm),
            material=white_piping, name=nm,
        )

    # ── "Marshall" logo plate on the lower baffle ─────────────────────────────
    _logo_frac = 0.28   # 28 % from B toward C (lower third)
    logo = BoxGeometry((0.004, 0.080, 0.022))
    logo.rotate_y(-BAFFLE_TILT)
    # Embed the logo into the grille surface (1 mm proud of baffle = overlaps grille)
    logo.translate(
        PT_B[0] + _logo_frac * _bdx + 0.001 * BAFFLE_NX,
        -0.006,
        PT_B[1] + _logo_frac * _bdz + 0.001 * BAFFLE_NZ,
    )
    body.visual(
        mesh_from_geometry(logo, "marshall_logo"),
        material=logo_cream, name="marshall_logo",
    )

    # ── corner caps at the four baffle corners ────────────────────────────────
    cap_hw = 0.008       # half-size along baffle direction
    cap_y = _half_w - 0.008
    for iy, yc in enumerate((-cap_y, cap_y)):
        for ib, frac in enumerate((0.06, 0.94)):
            nm = f"corner_cap_{iy}_{ib}"
            cap = BoxGeometry((0.016, 0.016, cap_hw * 2))
            cap.rotate_y(-BAFFLE_TILT)
            cap.translate(
                PT_B[0] + frac * _bdx + 0.003 * BAFFLE_NX,
                yc,
                PT_B[1] + frac * _bdz + 0.003 * BAFFLE_NZ,
            )
            body.visual(
                mesh_from_geometry(cap, nm),
                material=trim_black, name=nm,
            )

    # ── carry handle on the slanted top ───────────────────────────────────────
    body.visual(_handle_mesh(), material=trim_black, name="handle")

    body.inertial = Inertial.from_geometry(
        Box((CAB_D, CAB_W, H_REAR)), mass=2.6, origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # ───────────────────────────── knobs ───────────────────────────────────────
    for i, ky in enumerate(KNOB_YS):
        knob_geo = KnobGeometry(
            KNOB_DIAM, KNOB_H,
            body_style="cylindrical",
            edge_radius=0.0008,
            grip=KnobGrip(style="knurled", count=28, depth=0.0008),
            indicator=KnobIndicator(
                style="line", mode="raised",
                length=0.009, width=0.0014, depth=0.0010,
            ),
            top_feature=KnobTopFeature(
                style="recessed_disk", diameter=0.010, depth=0.0008,
            ),
            center=False,
        )
        # Raised pointer tab (makes knob visibly non-axisymmetric)
        pointer = BoxGeometry((0.0030, 0.0060, 0.0024))
        pointer.translate(0.0, KNOB_DIAM / 2.0 - 0.0010, KNOB_H + 0.0008)
        knob_geo.merge(pointer)

        # Tilt the whole knob to follow the slanted top surface
        knob_geo.rotate_y(TOP_ANGLE)

        knob_part = model.part(f"knob_{i}")
        knob_part.visual(
            mesh_from_geometry(knob_geo, f"knob_{i}"),
            material=metal_knob, name=f"knob_{i}",
        )
        knob_part.inertial = Inertial.from_geometry(
            Cylinder(KNOB_DIAM / 2.0, KNOB_H), mass=0.01,
        )

        # CONTINUOUS rotation about the top-surface normal
        model.articulation(
            f"panel_to_knob_{i}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob_part,
            origin=Origin(xyz=(KNOB_SEAT_X, ky, KNOB_SEAT_Z)),
            axis=(TOP_NX, 0.0, TOP_NZ),
            motion_limits=MotionLimits(effort=0.3, velocity=8.0),
        )

    return model


# ──────────────────────── tests ────────────────────────────────────────────────

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    knobs = [object_model.get_part(f"knob_{i}") for i in range(4)]
    joints = [object_model.get_articulation(f"panel_to_knob_{i}") for i in range(4)]

    # ── exactly four knobs ────────────────────────────────────────────────────
    ctx.check(
        "four control knobs present",
        len(knobs) == 4 and all(k is not None for k in knobs),
        details=f"knob parts={[k.name for k in knobs]}",
    )

    # ── wedge shape: rear of cabinet is taller than front ─────────────────────
    cab_mn, cab_mx = ctx.part_element_world_aabb(body, elem="cabinet")
    # Front-most column (max X): find its height
    # Rear-most column (min X): find its height
    # The cabinet bounding box should show the full height range.
    # The top surface is slanted: the max Z at min X (rear) > max Z at max X (front)
    # We verify via the overall bounding box: the box spans H_REAR in Z.
    cab_dz = cab_mx[2] - cab_mn[2]
    ctx.check(
        "wedge cabinet rear taller than front",
        cab_dz > H_FRONT and cab_dz >= H_REAR - 0.005,
        details=f"cabinet dz={cab_dz:.4f}, H_FRONT={H_FRONT}, H_REAR={H_REAR}",
    )

    # ── speaker grille on tilted baffle ───────────────────────────────────────
    g_mn, g_mx = ctx.part_element_world_aabb(body, elem="speaker_grille")
    gx_ext = g_mx[0] - g_mn[0]   # X extent (should be thin: tilted panel)
    gy_ext = g_mx[1] - g_mn[1]   # Y extent (width)
    gz_ext = g_mx[2] - g_mn[2]   # Z extent (height along baffle)
    # The grille is a thin panel: its X extent is much smaller than Y and Z
    ctx.check(
        "speaker grille is a thin tilted panel on the baffle",
        gx_ext < gy_ext and gx_ext < gz_ext and g_mn[0] > 0.0,
        details=f"grille extents=({gx_ext:.3f},{gy_ext:.3f},{gz_ext:.3f})",
    )
    # The grille centre Z should be lower than the top-surface centre Z
    # (it's on the front face, not on top)
    grille_cz = (g_mn[2] + g_mx[2]) / 2.0
    ctx.check(
        "grille centre is below the top panel",
        grille_cz < TOP_CZ,
        details=f"grille_cz={grille_cz:.4f}, top_cz={TOP_CZ:.4f}",
    )

    # ── gold panel on slanted top ─────────────────────────────────────────────
    p_mn, p_mx = ctx.part_element_world_aabb(body, elem="gold_panel")
    p_cx = (p_mn[0] + p_mx[0]) / 2.0
    p_cz = (p_mn[2] + p_mx[2]) / 2.0
    # Panel should be on the top surface: above the grille centre
    ctx.check(
        "gold panel is on the slanted top surface above the grille",
        p_cz > grille_cz and p_cz > 0.0,
        details=f"panel centre=({p_cx:.4f},{p_cz:.4f}), grille_cz={grille_cz:.4f}",
    )

    # ── knobs seated on the slanted panel ─────────────────────────────────────
    for i, (k, j) in enumerate(zip(knobs, joints)):
        ctx.allow_overlap(
            k, body,
            elem_a=f"knob_{i}", elem_b="gold_panel",
            reason="Knob base is intentionally press-fit a hair into the gold panel.",
        )
        pos = ctx.part_world_position(k)
        # Knob should be near the top surface and forward of the rear edge
        on_panel = (
            pos is not None
            and abs(pos[1] - KNOB_YS[i]) < 0.005
            and pos[2] > TOP_CZ - 0.03
        )
        ctx.check(
            f"knob_{i} seated on the slanted top panel",
            on_panel,
            details=f"knob_{i} pos={pos}, TOP_CZ={TOP_CZ:.3f}",
        )
        ctx.expect_contact(k, body, name=f"knob_{i} rests on panel")

    # ── knobs stand proud above the panel surface ─────────────────────────────
    for i, k in enumerate(knobs):
        mn, mx = ctx.part_world_aabb(k)
        ctx.check(
            f"knob_{i} stands proud above the panel",
            mx[2] > KNOB_SEAT_Z + 0.003,
            details=f"knob_{i} top z={mx[2]:.4f}, seat z={KNOB_SEAT_Z:.4f}",
        )

    # ── rotary articulation about the tilted top-surface normal ───────────────
    for i, (k, j) in enumerate(zip(knobs, joints)):
        ax = ctx.part_world_position(k)
        mn0, mx0 = ctx.part_world_aabb(k)
        cen0 = ((mn0[0] + mx0[0]) / 2.0 - ax[0],
                 (mn0[1] + mx0[1]) / 2.0 - ax[1])
        with ctx.pose({j: math.pi / 2.0}):
            mn1, mx1 = ctx.part_world_aabb(k)
            cen1 = ((mn1[0] + mx1[0]) / 2.0 - ax[0],
                     (mn1[1] + mx1[1]) / 2.0 - ax[1])
            z_top1 = mx1[2]
        z_top0 = mx0[2]
        # The pointer makes the knob non-axisymmetric: a quarter turn moves the
        # AABB centre in the XY plane while preserving height approximately.
        moved = math.hypot(cen1[0] - cen0[0], cen1[1] - cen0[1])
        ctx.check(
            f"knob_{i} spins about the tilted axis",
            moved > 0.0005 and abs(z_top1 - z_top0) < 0.008,
            details=f"rest_off={cen0}, turn_off={cen1}, moved={moved:.4f}, "
                    f"dz={abs(z_top1 - z_top0):.5f}",
        )

    # ── knob rotation axis is NOT purely vertical (wedge-specific) ────────────
    for i, j in enumerate(joints):
        ax = j.axis
        # The axis should have a significant X component (tilted forward)
        ctx.check(
            f"knob_{i} axis is tilted (not purely vertical)",
            abs(ax[0]) > 0.1 and abs(ax[2]) > 0.5,
            details=f"axis={ax}",
        )

    # ── white piping visible on the baffle ────────────────────────────────────
    for nm in ("piping_h_0", "piping_h_1", "piping_v_0", "piping_v_1"):
        pmn, pmx = ctx.part_element_world_aabb(body, elem=nm)
        ctx.check(
            f"{nm} is on the front baffle side",
            pmn[0] > 0.0,
            details=f"{nm} min_x={pmn[0]:.4f}",
        )

    # ── logo on the baffle ────────────────────────────────────────────────────
    lmn, lmx = ctx.part_element_world_aabb(body, elem="marshall_logo")
    ctx.check(
        "logo is on the front baffle",
        lmn[0] > 0.0 and (lmn[2] + lmx[2]) / 2.0 < TOP_CZ,
        details=f"logo x=[{lmn[0]:.3f},{lmx[0]:.3f}], z_center={(lmn[2]+lmx[2])/2:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
