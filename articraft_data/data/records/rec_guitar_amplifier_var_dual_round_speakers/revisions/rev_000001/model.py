from __future__ import annotations

# Marshall-style mini guitar combo amplifier (black) — dual speaker variant.
# Two visible round speaker drivers behind a coarse open bar grille.
# Coordinate convention:
#   - +X  : forward, toward the front speaker grille (grille face at +X).
#   - +Y  : cabinet width (left/right of the front face).
#   - +Z  : up; the recessed gold control panel sits on the top (+Z) face and
#           the four knobs point UP out of the panel (knob axis = +Z).
# Realistic mini-amp scale:
#   - width  (Y) ~ 0.18 m
#   - depth  (X) ~ 0.10 m
#   - height (Z) ~ 0.18 m
#
# Static body  : black vinyl cabinet (rounded, shelled at top for the recessed
#                gold panel), two lathed speaker drivers on the front baffle
#                behind a coarse open bar grille framed by white piping,
#                white cursive "Marshall" logo plate, black corner caps,
#                and a top carry handle.
# Articulations: FOUR knurled control knobs on the gold panel, each a CONTINUOUS
#                rotary joint about the vertical (+Z) axis.

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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# --- principal dimensions (meters) ---
CAB_W = 0.180          # width  (Y)
CAB_D = 0.100          # depth  (X)
CAB_H = 0.180          # height (Z)
WALL = 0.010           # cabinet wall thickness

# Top recessed gold panel (in the top face), spanning most of the width.
PANEL_TOP_Z = CAB_H / 2.0          # cabinet top surface
PANEL_RECESS = 0.012               # how deep the gold panel sits below the top
PANEL_W = 0.150                    # along Y
PANEL_D = 0.060                    # along X
PANEL_X = 0.012                    # panel center X (slightly toward the rear)
PANEL_GOLD_Z = PANEL_TOP_Z - PANEL_RECESS  # gold surface height

# Front grille opening.
FRONT_X = CAB_D / 2.0              # front face plane

# Four knobs evenly spaced across the gold panel.
KNOB_DIAM = 0.018
KNOB_H = 0.014
KNOB_YS = (-0.054, -0.018, 0.018, 0.054)
KNOB_X = PANEL_X + 0.004           # row sits a touch forward of panel center

# Speaker layout (two round drivers side by side on the front baffle).
SPEAKER_RADIUS = 0.033             # 66 mm driver (frame outer)
SPEAKER_DEPTH = 0.028              # total lathed depth (incl. magnet housing)
SPEAKER_YS = (-0.037, 0.037)       # Y centres of the two drivers
SPEAKER_X = FRONT_X - 0.012        # speaker front face X (behind the grille bars)
# Baffle board: front face coplanar with the grille pocket inner wall so it
# registers as connected to the cabinet shell.
POCKET_INNER_X = FRONT_X - 0.006   # pocket back wall
BAFFLE_THICK = 0.008
BAFFLE_X = POCKET_INNER_X - BAFFLE_THICK / 2.0  # centre of baffle plate
POCKET_W = CAB_W - 0.024           # pocket opening width  (Y)
POCKET_H = CAB_H - 0.024           # pocket opening height (Z)


def _rounded_box(w_y: float, d_x: float, h_z: float, fillet: float) -> cq.Workplane:
    # Box centered at origin, length d_x along X, width w_y along Y, height h_z
    # along Z, with vertical edges filleted (rounded cabinet corners).
    wp = cq.Workplane("XY").box(d_x, w_y, h_z)
    try:
        wp = wp.edges("|Z").fillet(fillet)
    except Exception:
        pass
    return wp


def _cabinet_solid() -> cq.Workplane:
    # Outer black vinyl cabinet, with a rectangular gold-panel recess cut into
    # the top face. The recess is the seat for the (separate) gold panel plate.
    outer = _rounded_box(CAB_W, CAB_D, CAB_H, fillet=0.010)

    recess = (
        cq.Workplane("XY")
        .box(PANEL_D, PANEL_W, PANEL_RECESS * 2.2)
        .translate((PANEL_X, 0.0, PANEL_TOP_Z))
    )
    body = outer.cut(recess)

    # Shallow recess on the front face where the grille seats (frame of cabinet
    # left proud all around as the piping mount).
    grille_pocket = (
        cq.Workplane("XY")
        .box(0.012, CAB_W - 0.024, CAB_H - 0.024)
        .translate((FRONT_X, 0.0, 0.0))
    )
    body = body.cut(grille_pocket)
    return body


def _handle_mesh():
    # Top carry handle: an arched strap spanning Y over the cabinet top, in front
    # of the gold panel, anchored by two end mounts. Built by sweeping a small
    # rectangular profile along a parabolic-arch spline path.
    strap_x = PANEL_X - PANEL_D / 2.0 - 0.016
    z0 = PANEL_TOP_Z
    arch = 0.020
    span = 0.104

    n = 13
    path_pts = []
    for i in range(n):
        t = i / (n - 1)
        y = -span / 2.0 + t * span
        zz = z0 + arch * (1.0 - (2.0 * t - 1.0) ** 2) + 0.002
        path_pts.append((strap_x, y, zz))

    path = cq.Workplane().add(cq.Edge.makeSpline([cq.Vector(*p) for p in path_pts]))
    profile = (
        cq.Workplane("XZ")
        .center(strap_x, path_pts[0][2])
        .rect(0.018, 0.007)
    )
    # Sweep the strap profile along the arched spline.
    strap = profile.sweep(path, multisection=False, makeSolid=True)

    # End mounts anchoring the strap ends to the cabinet top.
    result = strap
    for y in (-span / 2.0, span / 2.0):
        mount = (
            cq.Workplane("XY")
            .box(0.024, 0.018, 0.014)
            .translate((strap_x, y, z0 - 0.004))
        )
        result = result.union(mount)
    return mesh_from_cadquery(result, "handle")


def _speaker_driver():
    """Build one speaker driver mesh using LatheGeometry.

    Axis of revolution is Z. Front face at z=0 (dust cap apex), depth extends
    along +Z to the flat rear. The revolved profile traces:

      dust-cap dome → cone slope → surround half-roll → frame ring → rear plate

    Returns a MeshGeometry that can be rotated and placed on the baffle.
    """
    profile = [
        # Dust cap dome (from centre apex outward)
        (0.000, 0.000),   # apex (front-most point)
        (0.003, 0.001),   # dome curve
        (0.006, 0.002),   # dome mid
        (0.009, 0.004),   # dome edge

        # Cone surface (sloping backward and outward)
        (0.011, 0.005),   # cone inner
        (0.017, 0.008),   # cone mid
        (0.023, 0.011),   # cone outer
        (0.027, 0.014),   # cone edge

        # Surround half-roll (forward bulge at the rim)
        (0.029, 0.013),   # surround inner lip
        (0.031, 0.010),   # surround forward curve
        (0.032, 0.007),   # surround peak (most forward at rim)
        (0.031, 0.011),   # surround back curve
        (0.030, 0.015),   # surround rear lip

        # Frame / basket ring
        (0.033, 0.015),   # frame front outer
        (0.033, 0.022),   # frame back
        (0.020, 0.022),   # rear plate inner
        (0.020, 0.028),   # magnet housing depth
        (0.000, 0.028),   # close at back centre
    ]
    return LatheGeometry(profile, segments=36)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mini_guitar_amp")

    black_vinyl = model.material("black_vinyl", rgba=(0.07, 0.07, 0.08, 1.0))
    gold_panel = model.material("gold_panel", rgba=(0.86, 0.62, 0.18, 1.0))
    grille_dark = model.material("grille_dark", rgba=(0.10, 0.10, 0.11, 1.0))
    white_piping = model.material("white_piping", rgba=(0.92, 0.92, 0.90, 1.0))
    logo_cream = model.material("logo_cream", rgba=(0.95, 0.93, 0.86, 1.0))
    metal_knob = model.material("metal_knob", rgba=(0.78, 0.78, 0.80, 1.0))
    indicator_red = model.material("indicator_red", rgba=(0.85, 0.12, 0.10, 1.0))
    trim_black = model.material("trim_black", rgba=(0.04, 0.04, 0.05, 1.0))
    speaker_mat = model.material("speaker_cone", rgba=(0.14, 0.13, 0.12, 1.0))
    baffle_mat = model.material("baffle", rgba=(0.06, 0.06, 0.07, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("body")

    cab = _cabinet_solid()
    body.visual(mesh_from_cadquery(cab, "cabinet"), material=black_vinyl, name="cabinet")

    # Gold control panel plate seated in the top recess.
    panel = BoxGeometry((PANEL_D, PANEL_W, 0.004))
    panel.translate(PANEL_X, 0.0, PANEL_GOLD_Z + 0.002)
    body.visual(mesh_from_geometry(panel, "gold_panel"), material=gold_panel, name="gold_panel")

    # Small red power indicator on the gold panel, beside the knob row.
    led = CylinderGeometry(0.0035, 0.004)
    led.translate(PANEL_X - 0.004, 0.072, PANEL_GOLD_Z + 0.004)
    body.visual(mesh_from_geometry(led, "power_led"), material=indicator_red, name="power_led")

    # ---- baffle board (dark panel behind the speakers that they mount to) ----
    # Sized to match the grille pocket opening so its front face and edges are
    # coplanar with the pocket walls, establishing mesh connectivity to the
    # cabinet shell.
    baffle = BoxGeometry((BAFFLE_THICK, POCKET_W, POCKET_H))
    baffle.translate(BAFFLE_X, 0.0, 0.0)
    body.visual(mesh_from_geometry(baffle, "baffle_board"),
                material=baffle_mat, name="baffle_board")

    # ---- two round speaker drivers on the front baffle (for-i-in-range) ----
    for i, sy in enumerate(SPEAKER_YS):
        spk = _speaker_driver()
        # Rotate so the front face (z=0 in lathe) faces +X.
        # rotate_y(-pi/2): lathe +Z → world -X, so front (z=0) stays at x=0
        # and the depth extends along -X (behind the front face).
        spk.rotate_y(-math.pi / 2.0)
        spk.translate(SPEAKER_X, sy, 0.0)
        body.visual(mesh_from_geometry(spk, f"speaker_{i}"),
                    material=speaker_mat, name=f"speaker_{i}")

    # ---- coarse open grille: horizontal bars + vertical supports ----
    grille_opening_h = CAB_H - 0.030
    bar_thickness = 0.004       # X depth of each bar
    bar_h = 0.004               # Z height of each horizontal bar
    n_bars = 10
    bar_pitch = grille_opening_h / n_bars
    # Bars span the full pocket width so their ends touch the pocket side walls,
    # and their back face is coplanar with the pocket inner wall at x=POCKET_INNER_X.
    bar_x = POCKET_INNER_X + bar_thickness / 2.0   # bar back at POCKET_INNER_X
    bar_span = POCKET_W                             # ends touch pocket side walls

    for i in range(n_bars):
        z_pos = -grille_opening_h / 2.0 + bar_pitch * (i + 0.5)
        bar = BoxGeometry((bar_thickness, bar_span, bar_h))
        bar.translate(bar_x, 0.0, z_pos)
        body.visual(mesh_from_geometry(bar, f"grille_bar_{i}"),
                    material=grille_dark, name=f"grille_bar_{i}")

    # Vertical support bar (centre, between the two speakers).
    vert_w = 0.004              # Y width of the vertical bar
    vbar = BoxGeometry((bar_thickness, vert_w, grille_opening_h))
    vbar.translate(bar_x, 0.0, 0.0)
    body.visual(mesh_from_geometry(vbar, "grille_vert_0"),
                material=grille_dark, name="grille_vert_0")

    # White piping frame around the grille (four thin bars on the front face).
    pip_t = 0.005          # bar thickness (X protrusion)
    pip_w = 0.006          # bar cross width
    fx = FRONT_X - 0.001
    fw = CAB_W - 0.020     # outer span of piping
    fh = CAB_H - 0.020
    # top & bottom bars (run along Y)
    for zc in (fh / 2.0, -fh / 2.0):
        bar = BoxGeometry((pip_t, fw, pip_w))
        bar.translate(fx, 0.0, zc)
        body.visual(mesh_from_geometry(bar, f"piping_h_{1 if zc > 0 else 0}"),
                    material=white_piping, name=f"piping_h_{1 if zc > 0 else 0}")
    # left & right bars (run along Z)
    for yc in (fw / 2.0, -fw / 2.0):
        bar = BoxGeometry((pip_t, pip_w, fh))
        bar.translate(fx, yc, 0.0)
        body.visual(mesh_from_geometry(bar, f"piping_v_{1 if yc > 0 else 0}"),
                    material=white_piping, name=f"piping_v_{1 if yc > 0 else 0}")

    # "Marshall" cursive logo plate (thin raised cream plate on the lower grille).
    # Back face coplanar with grille bar front face for mesh connectivity.
    logo = BoxGeometry((0.004, 0.080, 0.022))
    logo.translate(FRONT_X, -0.006, -0.046)
    body.visual(mesh_from_geometry(logo, "marshall_logo"), material=logo_cream, name="marshall_logo")

    # Black corner caps on the four front vertical corners (protective trim).
    cap_y = (CAB_W / 2.0) - 0.006
    cap_z = (CAB_H / 2.0) - 0.006
    for iy, yc in enumerate((-cap_y, cap_y)):
        for iz, zc in enumerate((-cap_z, cap_z)):
            cap = BoxGeometry((0.018, 0.016, 0.016))
            cap.translate(FRONT_X - 0.006, yc, zc)
            body.visual(mesh_from_geometry(cap, f"corner_cap_{iy}_{iz}"),
                        material=trim_black, name=f"corner_cap_{iy}_{iz}")

    # Top carry handle.
    body.visual(_handle_mesh(), material=trim_black, name="handle")

    body.inertial = Inertial.from_geometry(
        Box((CAB_D, CAB_W, CAB_H)), mass=2.4, origin=Origin(xyz=(0.0, 0.0, 0.0))
    )

    # --------------------------------------------------------------- knobs
    # Each knob: knurled cylinder with an engraved pointer line, mounting face on
    # z=0 so it sits ON the gold panel and points UP (+Z). CONTINUOUS rotation.
    for i, ky in enumerate(KNOB_YS):
        knob_geo = KnobGeometry(
            KNOB_DIAM,
            KNOB_H,
            body_style="cylindrical",
            edge_radius=0.0008,
            grip=KnobGrip(style="knurled", count=28, depth=0.0008),
            indicator=KnobIndicator(style="line", mode="raised",
                                    length=0.009, width=0.0014, depth=0.0010),
            top_feature=KnobTopFeature(style="recessed_disk", diameter=0.010, depth=0.0008),
            center=False,  # mounting face at z=0
        )
        # Raised pointer tab near the rim (the position marker). Sitting off the
        # rotation axis, it makes the knob clearly non-axisymmetric so its spin is
        # observable, and reads as the indicator dot/pointer of the knob.
        pointer = BoxGeometry((0.0030, 0.0060, 0.0024))
        pointer.translate(0.0, KNOB_DIAM / 2.0 - 0.0010, KNOB_H + 0.0008)
        knob_geo.merge(pointer)
        knob_part = model.part(f"knob_{i}")
        knob_part.visual(mesh_from_geometry(knob_geo, f"knob_{i}"),
                         material=metal_knob, name=f"knob_{i}")
        knob_part.inertial = Inertial.from_geometry(
            Cylinder(KNOB_DIAM / 2.0, KNOB_H), mass=0.01
        )
        # Seat the knob base slightly INTO the gold panel surface for a real
        # press-fit look (justified overlap in tests).
        seat_z = PANEL_GOLD_Z + 0.003 - 0.002
        model.articulation(
            f"panel_to_knob_{i}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob_part,
            origin=Origin(xyz=(KNOB_X, ky, seat_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.3, velocity=8.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    knobs = [object_model.get_part(f"knob_{i}") for i in range(4)]
    joints = [object_model.get_articulation(f"panel_to_knob_{i}") for i in range(4)]

    # --- exactly four knobs on the top gold panel ---
    ctx.check(
        "four control knobs present",
        len(knobs) == 4 and all(k is not None for k in knobs),
        details=f"knob parts={[k.name for k in knobs]}",
    )

    # Each knob sits on the gold panel: base near the gold surface, above it,
    # and within the panel footprint in X/Y. Allow the small press-fit overlap.
    for i, (k, j) in enumerate(zip(knobs, joints)):
        ctx.allow_overlap(
            k,
            body,
            elem_a=f"knob_{i}",
            elem_b="gold_panel",
            reason="Knob base is intentionally press-fit a hair into the gold panel surface.",
        )
        pos = ctx.part_world_position(k)
        on_panel = (
            pos is not None
            and abs(pos[0] - PANEL_X) <= PANEL_D / 2.0 + 0.006
            and abs(pos[1]) <= PANEL_W / 2.0
            and pos[2] > PANEL_GOLD_Z - 0.001
        )
        ctx.check(
            f"knob_{i} seated on the gold panel",
            on_panel,
            details=f"knob_{i} world pos={pos}, gold_z={PANEL_GOLD_Z:.3f}",
        )
        ctx.expect_contact(k, body, name=f"knob_{i} rests on panel")

    # Knobs point UP: each knob's vertical (Z) extent is its dominant axis-ish,
    # and the top is above the gold surface (it stands proud of the panel).
    for i, k in enumerate(knobs):
        mn, mx = ctx.part_world_aabb(k)
        ctx.check(
            f"knob_{i} stands proud above the panel",
            mx[2] > PANEL_GOLD_Z + 0.004,
            details=f"knob_{i} top z={mx[2]:.4f}",
        )

    # --- rotary articulation about the vertical axis ---
    for i, (k, j) in enumerate(zip(knobs, joints)):
        ax = ctx.part_world_position(k)
        mn0, mx0 = ctx.part_world_aabb(k)
        z_top0 = mx0[2]
        cen0 = ((mn0[0] + mx0[0]) / 2.0 - ax[0], (mn0[1] + mx0[1]) / 2.0 - ax[1])
        with ctx.pose({j: math.pi / 2.0}):
            mn1, mx1 = ctx.part_world_aabb(k)
            z_top1 = mx1[2]
            cen1 = ((mn1[0] + mx1[0]) / 2.0 - ax[0], (mn1[1] + mx1[1]) / 2.0 - ax[1])
        rotated = abs(cen0[1]) > abs(cen0[0]) and abs(cen1[0]) > abs(cen1[1])
        moved = math.hypot(cen1[0] - cen0[0], cen1[1] - cen0[1])
        ctx.check(
            f"knob_{i} spins about the vertical axis",
            rotated and moved > 0.0008 and abs(z_top1 - z_top0) < 0.0015,
            details=f"rest_offset={cen0}, turn_offset={cen1}, moved={moved:.4f}, "
                    f"dz_top={abs(z_top1 - z_top0):.5f}",
        )

    # --- two round speaker drivers on the front baffle ---
    speaker_elems = []
    for i in range(2):
        ename = f"speaker_{i}"
        s_mn, s_mx = ctx.part_element_world_aabb(body, elem=ename)
        speaker_elems.append((ename, s_mn, s_mx))
        sx_ext = s_mx[0] - s_mn[0]
        sy_ext = s_mx[1] - s_mn[1]
        sz_ext = s_mx[2] - s_mn[2]
        # Speaker is a round driver: thin in X (depth), roughly equal Y and Z extents.
        ctx.check(
            f"speaker_{i} is a round driver (thin in X, round in YZ)",
            sx_ext < sy_ext and sx_ext < sz_ext and abs(sy_ext - sz_ext) < 0.015,
            details=f"speaker_{i} extents=({sx_ext:.3f},{sy_ext:.3f},{sz_ext:.3f})",
        )
        # Speaker is near the front face.
        sx_center = (s_mn[0] + s_mx[0]) / 2.0
        ctx.check(
            f"speaker_{i} located near the front face",
            sx_center > 0.02 and sx_center < FRONT_X,
            details=f"speaker_{i} center x={sx_center:.3f}",
        )

    # Speakers are side by side: different Y centres, similar Z centres.
    if len(speaker_elems) == 2:
        _, mn0, mx0 = speaker_elems[0]
        _, mn1, mx1 = speaker_elems[1]
        cy0 = (mn0[1] + mx0[1]) / 2.0
        cy1 = (mn1[1] + mx1[1]) / 2.0
        cz0 = (mn0[2] + mx0[2]) / 2.0
        cz1 = (mn1[2] + mx1[2]) / 2.0
        ctx.check(
            "two speakers side by side (Y-separated, Z-aligned)",
            abs(cy1 - cy0) > 0.040 and abs(cz1 - cz0) < 0.010,
            details=f"speaker Y centres=({cy0:.3f},{cy1:.3f}), Z centres=({cz0:.3f},{cz1:.3f})",
        )

    # Speakers are behind the grille bars (lower X than grille bar positions).
    grille_bar_mn, grille_bar_mx = ctx.part_element_world_aabb(body, elem="grille_bar_0")
    bar_x_center = (grille_bar_mn[0] + grille_bar_mx[0]) / 2.0
    for ename, s_mn, s_mx in speaker_elems:
        spk_x_center = (s_mn[0] + s_mx[0]) / 2.0
        ctx.check(
            f"{ename} is behind the grille bars",
            spk_x_center < bar_x_center - 0.003,
            details=f"speaker x={spk_x_center:.3f}, bar x={bar_x_center:.3f}",
        )

    # --- open grille: bars are thin front-facing panels ---
    bar0_mn, bar0_mx = ctx.part_element_world_aabb(body, elem="grille_bar_0")
    bx_ext = bar0_mx[0] - bar0_mn[0]
    by_ext = bar0_mx[1] - bar0_mn[1]
    bz_ext = bar0_mx[2] - bar0_mn[2]
    ctx.check(
        "grille bars are thin front-facing horizontal bars",
        bx_ext < 0.010 and by_ext > 0.10 and bz_ext < 0.010,
        details=f"bar_0 extents=({bx_ext:.3f},{by_ext:.3f},{bz_ext:.3f})",
    )

    return ctx.report()


object_model = build_object_model()
