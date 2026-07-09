from __future__ import annotations

# Round-puck conference speakerphone (light warm-gray colorway).
# Body: circular disc with chamfered rim band, shallow dome top, small feet.
# Center: round perforated speaker grille with raised trim ring.
# Front sector (-Y): 12 dial keys on two concentric arcs (6 per arc),
#   call_key (green) and end_key (red) at outer arc ends.
# Rear sector (+Y): rectangular LCD with bezel, 4 soft keys flanking.
# Rim at ±90°: volume up/down keys.
# Rim at 120° spacing: 3 mic hole clusters.
# All keys: prismatic press (-Z, 1mm travel), seated 1mm into dome deck.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- Body dimensions (round puck) -------------------------------------------
BODY_R = 0.090            # body radius
BODY_H = 0.040            # main cylinder height
DOME_H = 0.004            # dome rise above BODY_H at center
CHAMFER = 0.004           # bottom-edge fillet
FOOT_R = 0.006            # foot radius
FOOT_H = 0.003            # foot height below z=0
N_FEET = 4

# ---- Speaker grille ---------------------------------------------------------
GRILLE_R = 0.032          # grille radius (diameter 64mm = 0.356 * body_dia < 0.38)
TRIM_R_OUTER = 0.036      # trim ring outer radius
TRIM_R_INNER = 0.033      # trim ring inner radius
TRIM_H = 0.002            # trim ring height
ANNULAR_CLEARANCE_MIN = 0.006  # min clearance from trim ring to any key

# ---- Key arcs (front sector, centered on 270° = -Y) -------------------------
INNER_ARC_R = 0.050
OUTER_ARC_R = 0.065
ARC_START_DEG = 225.0     # leftmost key on arcs
ARC_END_DEG = 315.0       # rightmost key on arcs
N_KEYS_PER_ARC = 6
CALL_ANGLE_DEG = 213.0    # call key (left end of outer arc)
END_ANGLE_DEG = 327.0     # end key (right end of outer arc)

# ---- Soft keys (rear sector, near LCD) --------------------------------------
SOFT_ARC_R = 0.052
SOFT_ANGLES_DEG = (65.0, 80.0, 100.0, 115.0)

# ---- LCD (rear sector) ------------------------------------------------------
LCD_R = 0.064
LCD_ANGLE_DEG = 90.0
LCD_W = 0.050
LCD_D = 0.018

# ---- Volume keys (rim, ±90° from front) -------------------------------------
VOL_R = 0.082
VOL_UP_DEG = 0.0          # +X side
VOL_DN_DEG = 180.0        # -X side

# ---- Mic clusters (rim, 120° spacing) ---------------------------------------
MIC_R = 0.085
MIC_ANGLES_DEG = (30.0, 150.0, 270.0)
MIC_HOLE_R = 0.004

# ---- Keycap sizes -----------------------------------------------------------
KEY_W, KEY_D, KEY_H = 0.012, 0.010, 0.005
SOFT_W, SOFT_D, SOFT_H = 0.011, 0.008, 0.004
CC_W, CC_D, CC_H = 0.014, 0.011, 0.005
VOL_W, VOL_D, VOL_H = 0.016, 0.008, 0.004

PRESS_TRAVEL = 0.001
CAP_EMBED = 0.001


# ---- Helpers ----------------------------------------------------------------

def _dome_z(r: float) -> float:
    """Approximate dome surface height at radial distance r from center."""
    r_inner = BODY_R * 0.50
    if r <= r_inner:
        return BODY_H + DOME_H
    if r >= BODY_R:
        return BODY_H
    t = (BODY_R - r) / (BODY_R - r_inner)
    return BODY_H + DOME_H * t * t


def _polar(deg: float, r: float) -> tuple[float, float]:
    """Polar (degrees CCW from +X, radius) → (x, y)."""
    a = math.radians(deg)
    return (r * math.cos(a), r * math.sin(a))


def _body_solid() -> cq.Workplane:
    """Round puck body: cylinder + shallow dome loft + bottom fillet + feet."""
    body = cq.Workplane("XY").circle(BODY_R).extrude(BODY_H)

    # Shallow dome: loft from near-full body radius at deck level to a smaller
    # circle at the dome peak.  The base circle is 98 % of BODY_R so the loft
    # overlaps the cylinder wall and unions cleanly.
    dome = (
        cq.Workplane("XY")
        .workplane(offset=BODY_H - 0.001)
        .circle(BODY_R * 0.98)
        .workplane(offset=DOME_H + 0.001)
        .circle(BODY_R * 0.50)
        .loft(ruled=False)
    )
    body = body.union(dome)

    # Soft chamfer on the bottom edge.
    try:
        body = body.faces("<Z").edges().fillet(CHAMFER)
    except Exception:
        pass

    # Small rubber feet on the underside.
    for i in range(N_FEET):
        ang = math.radians(i * 360.0 / N_FEET + 45.0)
        fx = (BODY_R * 0.65) * math.cos(ang)
        fy = (BODY_R * 0.65) * math.sin(ang)
        foot = (
            cq.Workplane("XY")
            .center(fx, fy)
            .workplane(offset=-FOOT_H)
            .circle(FOOT_R)
            .extrude(FOOT_H + 0.001)
        )
        body = body.union(foot)
    return body


def _speaker_grille_mesh():
    panel = PerforatedPanelGeometry(
        (2 * GRILLE_R + 0.004, 2 * GRILLE_R + 0.004),
        0.003,
        hole_diameter=0.0035,
        pitch=(0.007, 0.007),
        frame=0.005,
        corner_radius=GRILLE_R,
        stagger=True,
        center=True,
    )
    return mesh_from_geometry(panel, "speaker_grille")


def _trim_ring_mesh():
    ring = (
        cq.Workplane("XY")
        .circle(TRIM_R_OUTER)
        .circle(TRIM_R_INNER)
        .extrude(TRIM_H)
    )
    return mesh_from_cadquery(ring, "speaker_trim")


def _mic_cluster_mesh(name: str):
    panel = PerforatedPanelGeometry(
        (2 * MIC_HOLE_R + 0.002, 2 * MIC_HOLE_R + 0.002),
        0.002,
        hole_diameter=0.0015,
        pitch=(0.003, 0.003),
        frame=0.002,
        corner_radius=MIC_HOLE_R,
        stagger=True,
        center=True,
    )
    return mesh_from_geometry(panel, name)


def _keycap_mesh(name: str, w: float, d: float, h: float):
    cap = (
        cq.Workplane("XY")
        .rect(w, d)
        .workplane(offset=h * 0.6)
        .rect(w * 0.94, d * 0.94)
        .workplane(offset=h * 0.4)
        .rect(w * 0.82, d * 0.82)
        .loft(ruled=False)
    )
    try:
        cap = cap.edges(">Z").fillet(min(w, d) * 0.12)
    except Exception:
        pass
    return mesh_from_cadquery(cap, name)


# ---- Build ------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="conference_speakerphone")

    # Materials – light warm-gray body colorway.
    body_gray = model.material("body_gray", rgba=(0.68, 0.66, 0.63, 1.0))
    grille_dark = model.material("grille_dark", rgba=(0.13, 0.13, 0.14, 1.0))
    lcd_dark = model.material("lcd_dark", rgba=(0.10, 0.16, 0.14, 1.0))
    key_gray = model.material("key_gray", rgba=(0.55, 0.57, 0.60, 1.0))
    soft_gray = model.material("soft_gray", rgba=(0.48, 0.50, 0.52, 1.0))
    call_green = model.material("call_green", rgba=(0.15, 0.62, 0.28, 1.0))
    end_red = model.material("end_red", rgba=(0.72, 0.16, 0.16, 1.0))
    bezel = model.material("bezel", rgba=(0.18, 0.19, 0.21, 1.0))
    trim_mat = model.material("trim_mat", rgba=(0.25, 0.25, 0.27, 1.0))

    # ---- BODY (root) --------------------------------------------------------
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_solid(), "body_shell"),
        material=body_gray,
        name="body_shell",
    )

    # Central speaker grille (recessed slightly into dome peak).
    grille_z = _dome_z(0.0) - 0.001
    body.visual(
        _speaker_grille_mesh(),
        origin=Origin(xyz=(0.0, 0.0, grille_z)),
        material=grille_dark,
        name="speaker_grille",
    )

    # Raised trim ring around grille.
    trim_z = _dome_z(GRILLE_R)
    body.visual(
        _trim_ring_mesh(),
        origin=Origin(xyz=(0.0, 0.0, trim_z)),
        material=trim_mat,
        name="speaker_trim",
    )

    # LCD display in the rear sector (+Y).
    lx, ly = _polar(LCD_ANGLE_DEG, LCD_R)
    lz = _dome_z(LCD_R) - 0.001
    body.visual(
        Box((LCD_W, LCD_D, 0.003)),
        origin=Origin(xyz=(lx, ly, lz)),
        material=lcd_dark,
        name="lcd_display",
    )
    body.visual(
        Box((LCD_W + 0.008, LCD_D + 0.006, 0.004)),
        origin=Origin(xyz=(lx, ly, lz - 0.001)),
        material=bezel,
        name="lcd_bezel",
    )

    # Mic clusters on the rim band at 120° spacing.
    for i, ang_deg in enumerate(MIC_ANGLES_DEG):
        mx, my = _polar(ang_deg, MIC_R)
        mz = _dome_z(MIC_R) - 0.001
        body.visual(
            _mic_cluster_mesh(f"mic_cluster_{i}"),
            origin=Origin(xyz=(mx, my, mz)),
            material=grille_dark,
            name=f"mic_cluster_{i}",
        )

    body.inertial = Inertial.from_geometry(
        Cylinder(radius=BODY_R, length=BODY_H + DOME_H),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.0, (BODY_H + DOME_H) * 0.5)),
    )

    # ---- KEYS (prismatic press, -Z) ----------------------------------------
    press_limit = MotionLimits(effort=4.0, velocity=0.05, lower=0.0, upper=PRESS_TRAVEL)

    # Collect (name, x, y, z, w, d, cap_h, material, yaw_deg) for all keys.
    key_specs: list[tuple[str, float, float, float, float, float, float, object, float]] = []

    # Arc angle distribution (6 keys, evenly spaced).
    arc_span = ARC_END_DEG - ARC_START_DEG
    arc_step = arc_span / (N_KEYS_PER_ARC - 1)
    arc_angles = [ARC_START_DEG + j * arc_step for j in range(N_KEYS_PER_ARC)]

    # Inner arc: keypad rows 0-1 (keys 1-6).
    inner_labels = [
        "keypad_0_0", "keypad_0_1", "keypad_0_2",
        "keypad_1_0", "keypad_1_1", "keypad_1_2",
    ]
    for j, kname in enumerate(inner_labels):
        kx, ky = _polar(arc_angles[j], INNER_ARC_R)
        kz = _dome_z(INNER_ARC_R) - CAP_EMBED
        yaw = arc_angles[j] + 90.0
        key_specs.append((kname, kx, ky, kz, KEY_W, KEY_D, KEY_H, key_gray, yaw))

    # Outer arc: keypad rows 2-3 (keys 7-#).
    outer_labels = [
        "keypad_2_0", "keypad_2_1", "keypad_2_2",
        "keypad_3_0", "keypad_3_1", "keypad_3_2",
    ]
    for j, kname in enumerate(outer_labels):
        kx, ky = _polar(arc_angles[j], OUTER_ARC_R)
        kz = _dome_z(OUTER_ARC_R) - CAP_EMBED
        yaw = arc_angles[j] + 90.0
        key_specs.append((kname, kx, ky, kz, KEY_W, KEY_D, KEY_H, key_gray, yaw))

    # Call key (green) and end key (red) at outer arc extremities.
    for kname, ang_deg, mat in [
        ("call_key", CALL_ANGLE_DEG, call_green),
        ("end_key", END_ANGLE_DEG, end_red),
    ]:
        kx, ky = _polar(ang_deg, OUTER_ARC_R)
        kz = _dome_z(OUTER_ARC_R) - CAP_EMBED
        yaw = ang_deg + 90.0
        key_specs.append((kname, kx, ky, kz, CC_W, CC_D, CC_H, mat, yaw))

    # Soft keys: 4 keys in rear sector near the LCD.
    for i, ang_deg in enumerate(SOFT_ANGLES_DEG):
        sx, sy = _polar(ang_deg, SOFT_ARC_R)
        sz = _dome_z(SOFT_ARC_R) - CAP_EMBED
        yaw = ang_deg + 90.0
        key_specs.append((f"soft_key_{i}", sx, sy, sz, SOFT_W, SOFT_D, SOFT_H, soft_gray, yaw))

    # Volume keys on rim at ±90° from front.
    for kname, ang_deg in [("volume_up_key", VOL_UP_DEG), ("volume_down_key", VOL_DN_DEG)]:
        vx, vy = _polar(ang_deg, VOL_R)
        vz = _dome_z(VOL_R) - CAP_EMBED
        yaw = ang_deg + 90.0
        key_specs.append((kname, vx, vy, vz, VOL_W, VOL_D, VOL_H, soft_gray, yaw))

    # Build each key as a separate part with a prismatic press joint.
    for name, kx, ky, kz, w, d, cap_h, mat, yaw_deg in key_specs:
        kp = model.part(name)
        kp.visual(
            _keycap_mesh(f"{name}_cap", w, d, cap_h),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=mat,
            name=f"{name}_cap",
        )
        kp.inertial = Inertial.from_geometry(
            Box((w, d, cap_h)), mass=0.004, origin=Origin(xyz=(0.0, 0.0, cap_h * 0.5))
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=kp,
            origin=Origin(xyz=(kx, ky, kz), rpy=(0.0, 0.0, math.radians(yaw_deg))),
            axis=(0.0, 0.0, -1.0),
            motion_limits=press_limit,
        )

    return model


# ---- Test constants ---------------------------------------------------------

KEY_NAMES: list[str] = (
    [f"keypad_{r}_{c}" for r in range(4) for c in range(3)]
    + [f"soft_key_{i}" for i in range(4)]
    + ["call_key", "end_key", "volume_up_key", "volume_down_key"]
)

DIAL_KEY_NAMES: list[str] = [f"keypad_{r}_{c}" for r in range(4) for c in range(3)]


# ---- Tests ------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")

    # -- Allow seated key overlap (each cap sinks 1 mm into the dome deck). --
    for kname in KEY_NAMES:
        kpart = object_model.get_part(kname)
        ctx.allow_overlap(
            kpart,
            body,
            elem_a=f"{kname}_cap",
            elem_b="body_shell",
            reason="Keycap base is intentionally seated 1 mm into its dome deck well.",
        )

    # ---- body_shell is a round disc ----
    body_aabb = ctx.part_world_aabb(body)
    bmn, bmx = body_aabb
    bdx = bmx[0] - bmn[0]
    bdy = bmx[1] - bmn[1]
    bdz = bmx[2] - bmn[2]
    ctx.check(
        "body_shell is a round disc (X≈Y span, wide vs tall)",
        abs(bdx - bdy) < 0.020 and bdx > 0.15 and bdz < bdx * 0.45,
        details=f"dx={bdx:.4f} dy={bdy:.4f} dz={bdz:.4f}",
    )

    # ---- Speaker grille centered and within diameter limit ----
    ga = ctx.part_element_world_aabb(body, elem="speaker_grille")
    gmn, gmx = ga
    gcx = (gmn[0] + gmx[0]) * 0.5
    gcy = (gmn[1] + gmx[1]) * 0.5
    gdia = max(gmx[0] - gmn[0], gmx[1] - gmn[1])
    body_dia = max(bdx, bdy)
    ctx.check(
        "speaker_grille centered on body",
        abs(gcx) < 0.010 and abs(gcy) < 0.010,
        details=f"grille_center=({gcx:.4f},{gcy:.4f})",
    )
    ctx.check(
        "speaker_grille diameter ≤ 0.38 × body diameter",
        gdia <= 0.38 * body_dia + 0.005,
        details=f"grille_dia={gdia:.4f} body_dia={body_dia:.4f}",
    )

    # ---- Dial keys lie on two concentric arcs ----
    inner_rs: list[float] = []
    outer_rs: list[float] = []
    min_clearance = float("inf")
    trim_outer = TRIM_R_OUTER

    for j in range(6):
        r_idx = j // 3
        c_idx = j % 3
        kpart = object_model.get_part(f"keypad_{r_idx}_{c_idx}")
        pos = ctx.part_world_position(kpart)
        r = math.sqrt(pos[0] ** 2 + pos[1] ** 2)
        inner_rs.append(r)
        min_clearance = min(min_clearance, r - trim_outer)

    for j in range(6):
        r_idx = 2 + j // 3
        c_idx = j % 3
        kpart = object_model.get_part(f"keypad_{r_idx}_{c_idx}")
        pos = ctx.part_world_position(kpart)
        r = math.sqrt(pos[0] ** 2 + pos[1] ** 2)
        outer_rs.append(r)
        min_clearance = min(min_clearance, r - trim_outer)

    inner_mean = sum(inner_rs) / len(inner_rs)
    inner_spread = max(inner_rs) - min(inner_rs)
    outer_mean = sum(outer_rs) / len(outer_rs)
    outer_spread = max(outer_rs) - min(outer_rs)

    ctx.check(
        "inner arc dial keys at consistent radius (~0.050 m)",
        abs(inner_mean - INNER_ARC_R) < 0.005 and inner_spread < 0.004,
        details=f"inner_mean={inner_mean:.4f} spread={inner_spread:.4f}",
    )
    ctx.check(
        "outer arc dial keys at consistent radius (~0.065 m)",
        abs(outer_mean - OUTER_ARC_R) < 0.005 and outer_spread < 0.004,
        details=f"outer_mean={outer_mean:.4f} spread={outer_spread:.4f}",
    )

    # Annular clearance: no dial key inside the grille annulus (≥ 6 mm).
    ctx.check(
        "all dial keys clear the grille trim annulus ≥ 6 mm",
        min_clearance >= ANNULAR_CLEARANCE_MIN,
        details=f"min_clearance={min_clearance:.4f} m",
    )

    # ---- LCD is rectangular (landscape, clearly non-square) ----
    lcd_aabb = ctx.part_element_world_aabb(body, elem="lcd_display")
    lmn, lmx = lcd_aabb
    ldx = lmx[0] - lmn[0]
    ldy = lmx[1] - lmn[1]
    ctx.check(
        "lcd_display is rectangular (non-square, both dims positive)",
        abs(ldx - ldy) > 0.015 and min(ldx, ldy) > 0.008 and max(ldx, ldy) > 0.030,
        details=f"lcd dx={ldx:.4f} dy={ldy:.4f}",
    )

    # ---- Mic clusters at 120° spacing on rim ----
    mic_positions: list[tuple[float, float]] = []
    for i in range(3):
        ma = ctx.part_element_world_aabb(body, elem=f"mic_cluster_{i}")
        mmn, mmx = ma
        mcx = (mmn[0] + mmx[0]) * 0.5
        mcy = (mmn[1] + mmx[1]) * 0.5
        mic_positions.append((mcx, mcy))

    mic_ang = sorted(math.atan2(p[1], p[0]) % (2 * math.pi) for p in mic_positions)
    gaps = [math.degrees((mic_ang[(i + 1) % 3] - mic_ang[i]) % (2 * math.pi)) for i in range(3)]
    gaps_sorted = sorted(gaps)
    ctx.check(
        "mic clusters at ~120° spacing on rim",
        all(abs(g - 120.0) < 5.0 for g in gaps_sorted),
        details=f"angular_gaps={gaps_sorted}",
    )

    # ---- Sample key press (straight down, XY stable) ----
    sample_keys = [
        "keypad_0_1", "keypad_2_1", "call_key",
        "end_key", "soft_key_1", "volume_up_key",
    ]
    for kname in sample_keys:
        kpart = object_model.get_part(kname)
        joint = object_model.get_articulation(f"body_to_{kname}")
        rest = ctx.part_world_position(kpart)
        with ctx.pose({joint: PRESS_TRAVEL}):
            pressed = ctx.part_world_position(kpart)
        ctx.check(
            f"{kname} presses straight down (-Z)",
            rest is not None
            and pressed is not None
            and pressed[2] < rest[2] - 0.0005
            and abs(pressed[0] - rest[0]) < 1e-4
            and abs(pressed[1] - rest[1]) < 1e-4,
            details=f"rest={rest} pressed={pressed}",
        )

    return ctx.report()


object_model = build_object_model()
