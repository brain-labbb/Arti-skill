from __future__ import annotations

# Marshall-style mini guitar combo amplifier (black) — FRONT-PANEL variant.
# Coordinate convention:
#   - +X  : forward, toward the front speaker grille (grille face at +X).
#   - +Y  : cabinet width (left/right of the front face).
#   - +Z  : up; the top of the cabinet is plain vinyl with a carry handle.
# The gold control panel is now a horizontal strip on the FRONT face ABOVE the
# speaker grille, with four knurled knobs pointing FORWARD (knob axis = +X).
# Realistic mini-amp scale:
#   - width  (Y) ~ 0.18 m
#   - depth  (X) ~ 0.10 m
#   - height (Z) ~ 0.18 m
#
# Static body  : black vinyl cabinet (rounded), plain top with carry handle,
#                front-mounted gold control strip, dark perforated speaker grille
#                framed by white piping, white cursive "Marshall" logo plate,
#                and black corner caps.
# Articulations: FOUR knurled control knobs on the front gold panel strip, each
#                a CONTINUOUS rotary joint about the forward (+X) axis.

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

# --- principal dimensions (meters) ---
CAB_W = 0.180          # width  (Y)
CAB_D = 0.100          # depth  (X)
CAB_H = 0.180          # height (Z)
WALL = 0.010           # cabinet wall thickness

# Front face plane.
FRONT_X = CAB_D / 2.0  # = 0.050

# Gold control panel strip — mounted on the front face, above the grille.
PANEL_STRIP_W = 0.150      # width along Y
PANEL_STRIP_H = 0.030      # height along Z
PANEL_STRIP_Z = 0.063      # center Z (near the top of the front face)
PANEL_STRIP_THICK = 0.004  # thickness along X
# Panel front surface (where knobs mount).
PANEL_FRONT_X = FRONT_X - 0.002 + PANEL_STRIP_THICK / 2.0

# Speaker grille area — below the panel strip on the front face.
GRILLE_GAP = 0.008         # gap between panel strip bottom and grille top
GRILLE_TOP = PANEL_STRIP_Z - PANEL_STRIP_H / 2.0 - GRILLE_GAP
GRILLE_BOT = -(CAB_H / 2.0 - 0.015)
GRILLE_H = GRILLE_TOP - GRILLE_BOT
GRILLE_CENTER_Z = (GRILLE_TOP + GRILLE_BOT) / 2.0
GRILLE_W = CAB_W - 0.030

# Four knobs evenly spaced across the gold panel strip.
KNOB_DIAM = 0.018
KNOB_H = 0.014
KNOB_YS = (-0.054, -0.018, 0.018, 0.054)
KNOB_Z = PANEL_STRIP_Z
# Knob seat: slightly embedded into the panel front surface for press-fit look.
KNOB_SEAT_X = PANEL_FRONT_X - 0.002


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
    # Outer black vinyl cabinet. The top is plain (no recess). The front face
    # has a shallow pocket where the grille and panel strip seat.
    outer = _rounded_box(CAB_W, CAB_D, CAB_H, fillet=0.010)

    # Front face pocket: spans from below the grille to above the panel strip.
    pocket_top = PANEL_STRIP_Z + PANEL_STRIP_H / 2.0 + 0.004
    pocket_bot = GRILLE_BOT - 0.004
    pocket_h = pocket_top - pocket_bot
    pocket_cz = (pocket_top + pocket_bot) / 2.0
    grille_pocket = (
        cq.Workplane("XY")
        .box(0.012, CAB_W - 0.024, pocket_h)
        .translate((FRONT_X, 0.0, pocket_cz))
    )
    body = outer.cut(grille_pocket)
    return body


def _handle_mesh():
    # Top carry handle: an arched strap spanning Y over the plain cabinet top,
    # anchored by two end mounts. Built by sweeping a small rectangular profile
    # along a parabolic-arch spline path.
    strap_x = 0.0  # centered on the plain top
    z0 = CAB_H / 2.0  # cabinet top surface
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


def _build_knob_geometry_forward_facing(i: int):
    """Build a knurled knob whose axis points along +X (forward-facing).

    KnobGeometry builds with axis along +Z and base at z=0 (center=False).
    We rotate the whole assembly by +90 deg about Y so that +Z maps to +X:
    the base sits at x=0 and the knob protrudes in +X.
    """
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
    # Raised pointer tab near the rim (position marker). Before rotation this
    # sits near the top (+Z) face offset in +Y; after rotate_y(+pi/2) it ends
    # up at the front tip of the knob offset in +Y — clearly visible and
    # non-axisymmetric so rotation is observable.
    pointer = BoxGeometry((0.0030, 0.0060, 0.0024))
    pointer.translate(0.0, KNOB_DIAM / 2.0 - 0.0010, KNOB_H + 0.0008)
    knob_geo.merge(pointer)

    # Rotate: +Z -> +X so the knob axis points forward.
    knob_geo.rotate_y(math.pi / 2.0)
    return knob_geo


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

    # ------------------------------------------------------------------ body
    body = model.part("body")

    cab = _cabinet_solid()
    body.visual(mesh_from_cadquery(cab, "cabinet"), material=black_vinyl, name="cabinet")

    # Gold control panel strip on the front face, above the grille.
    panel = BoxGeometry((PANEL_STRIP_THICK, PANEL_STRIP_W, PANEL_STRIP_H))
    panel.translate(FRONT_X - 0.002, 0.0, PANEL_STRIP_Z)
    body.visual(mesh_from_geometry(panel, "gold_panel"), material=gold_panel, name="gold_panel")

    # Small red power indicator on the gold panel strip.
    led = BoxGeometry((0.004, 0.007, 0.007))
    led.translate(FRONT_X + 0.001, 0.068, PANEL_STRIP_Z)
    body.visual(mesh_from_geometry(led, "power_led"), material=indicator_red, name="power_led")

    # Front speaker grille: dark perforated panel set into the front pocket,
    # below the gold panel strip.
    # PerforatedPanelGeometry((a, b), thickness): after rotate_y(pi/2),
    # the first param 'a' becomes the Z extent and 'b' becomes the Y extent.
    grille = PerforatedPanelGeometry(
        (GRILLE_H, GRILLE_W),
        0.006,
        hole_diameter=0.004,
        pitch=0.009,
        frame=0.006,
        stagger=True,
    )
    grille.rotate_y(math.pi / 2.0)  # thickness Z -> X
    grille.translate(FRONT_X - 0.004, 0.0, GRILLE_CENTER_Z)
    body.visual(mesh_from_geometry(grille, "speaker_grille"), material=grille_dark, name="speaker_grille")

    # White piping frame around the grille (four thin bars on the front face).
    pip_t = 0.005          # bar thickness (X protrusion)
    pip_w = 0.006          # bar cross width
    fx = FRONT_X - 0.001
    pip_fw = GRILLE_W + 0.012  # piping outer span in Y
    pip_fh = GRILLE_H + 0.012  # piping outer span in Z
    pip_cz = GRILLE_CENTER_Z
    # top & bottom bars (run along Y)
    for idx, dz in enumerate((pip_fh / 2.0, -pip_fh / 2.0)):
        zc = pip_cz + dz
        bar = BoxGeometry((pip_t, pip_fw, pip_w))
        bar.translate(fx, 0.0, zc)
        body.visual(mesh_from_geometry(bar, f"piping_h_{idx}"),
                    material=white_piping, name=f"piping_h_{idx}")
    # left & right bars (run along Z)
    for idx, dy in enumerate((pip_fw / 2.0, -pip_fw / 2.0)):
        bar = BoxGeometry((pip_t, pip_w, pip_fh))
        bar.translate(fx, dy, pip_cz)
        body.visual(mesh_from_geometry(bar, f"piping_v_{idx}"),
                    material=white_piping, name=f"piping_v_{idx}")

    # "Marshall" cursive logo plate (thin raised cream plate on the lower grille).
    logo = BoxGeometry((0.004, 0.080, 0.022))
    logo.translate(FRONT_X + 0.001, -0.006, GRILLE_CENTER_Z - GRILLE_H * 0.30)
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

    # Top carry handle (plain top, no gold panel recess).
    body.visual(_handle_mesh(), material=trim_black, name="handle")

    body.inertial = Inertial.from_geometry(
        Box((CAB_D, CAB_W, CAB_H)), mass=2.4, origin=Origin(xyz=(0.0, 0.0, 0.0))
    )

    # --------------------------------------------------------------- knobs
    # Each knob: knurled cylinder with an engraved pointer, rotated so its axis
    # points along +X (forward). CONTINUOUS rotation about the forward axis.
    for i in range(4):
        ky = KNOB_YS[i]
        knob_geo = _build_knob_geometry_forward_facing(i)
        knob_part = model.part(f"knob_{i}")
        knob_part.visual(mesh_from_geometry(knob_geo, f"knob_{i}"),
                         material=metal_knob, name=f"knob_{i}")
        knob_part.inertial = Inertial.from_geometry(
            Cylinder(KNOB_DIAM / 2.0, KNOB_H), mass=0.01
        )
        # Articulation: knob spins about the forward (+X) axis on the panel
        # strip front surface. The seat is slightly embedded into the panel
        # for a real press-fit look (justified overlap in tests).
        model.articulation(
            f"panel_to_knob_{i}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob_part,
            origin=Origin(xyz=(KNOB_SEAT_X, ky, KNOB_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.3, velocity=8.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    knobs = [object_model.get_part(f"knob_{i}") for i in range(4)]
    joints = [object_model.get_articulation(f"panel_to_knob_{i}") for i in range(4)]

    # --- exactly four knobs on the front gold panel strip ---
    ctx.check(
        "four control knobs present",
        len(knobs) == 4 and all(k is not None for k in knobs),
        details=f"knob parts={[k.name for k in knobs]}",
    )

    # Each knob is on the front faceplate: base near the panel front surface,
    # protruding forward (+X), and within the panel strip footprint in Y/Z.
    for i, (k, j) in enumerate(zip(knobs, joints)):
        ctx.allow_overlap(
            k,
            body,
            elem_a=f"knob_{i}",
            elem_b="gold_panel",
            reason="Knob base is intentionally press-fit a hair into the gold panel strip surface.",
        )
        pos = ctx.part_world_position(k)
        on_panel = (
            pos is not None
            and pos[0] > FRONT_X - 0.010  # near the front face
            and abs(pos[1] - KNOB_YS[i]) <= 0.006
            and abs(pos[2] - PANEL_STRIP_Z) <= PANEL_STRIP_H / 2.0 + 0.006
        )
        ctx.check(
            f"knob_{i} seated on the front panel strip",
            on_panel,
            details=f"knob_{i} world pos={pos}, panel_strip_z={PANEL_STRIP_Z:.3f}",
        )
        ctx.expect_contact(k, body, name=f"knob_{i} rests on panel strip")

    # Knobs face FORWARD: each knob protrudes in +X from the panel surface,
    # and the forward (X) extent is the dominant protrusion direction.
    for i, k in enumerate(knobs):
        mn, mx = ctx.part_world_aabb(k)
        ctx.check(
            f"knob_{i} protrudes forward from the panel strip",
            mx[0] > PANEL_FRONT_X + 0.004,
            details=f"knob_{i} max x={mx[0]:.4f}, panel_front_x={PANEL_FRONT_X:.4f}",
        )

    # --- rotary articulation about the forward (+X) axis ---
    # The off-axis raised pointer makes the knob non-axisymmetric. At rest the
    # pointer sits toward +Y from the rotation axis. A quarter turn about +X
    # swings that Y offset into +Z. The forward (X) protrusion stays constant.
    for i, (k, j) in enumerate(zip(knobs, joints)):
        ax = ctx.part_world_position(k)
        mn0, mx0 = ctx.part_world_aabb(k)
        x_tip0 = mx0[0]
        cen0_y = (mn0[1] + mx0[1]) / 2.0 - ax[1]
        cen0_z = (mn0[2] + mx0[2]) / 2.0 - ax[2]
        with ctx.pose({j: math.pi / 2.0}):
            mn1, mx1 = ctx.part_world_aabb(k)
            x_tip1 = mx1[0]
            cen1_y = (mn1[1] + mx1[1]) / 2.0 - ax[1]
            cen1_z = (mn1[2] + mx1[2]) / 2.0 - ax[2]
        # At rest the offset is mostly in Y; after a quarter turn mostly in Z.
        rotated = abs(cen0_y) > abs(cen0_z) and abs(cen1_z) > abs(cen1_y)
        moved = math.hypot(cen1_y - cen0_y, cen1_z - cen0_z)
        ctx.check(
            f"knob_{i} spins about the forward axis",
            rotated and moved > 0.0008 and abs(x_tip1 - x_tip0) < 0.0015,
            details=f"rest_offset_y={cen0_y:.4f} z={cen0_z:.4f}, "
                    f"turn_offset_y={cen1_y:.4f} z={cen1_z:.4f}, moved={moved:.4f}, "
                    f"dx_tip={abs(x_tip1 - x_tip0):.5f}",
        )

    # --- gold panel strip is on the front face, above the grille ---
    p_mn, p_mx = ctx.part_element_world_aabb(body, elem="gold_panel")
    px = (p_mn[0] + p_mx[0]) / 2.0
    px_ext = p_mx[0] - p_mn[0]
    py_ext = p_mx[1] - p_mn[1]
    pz_ext = p_mx[2] - p_mn[2]
    ctx.check(
        "gold panel is a thin front-facing strip on the front face",
        px > 0.03 and px_ext < py_ext and px_ext < pz_ext,
        details=f"panel center x={px:.3f}, extents=({px_ext:.3f},{py_ext:.3f},{pz_ext:.3f})",
    )
    # Panel is above the grille in Z.
    g_mn, g_mx = ctx.part_element_world_aabb(body, elem="speaker_grille")
    ctx.check(
        "gold panel strip is above the speaker grille",
        p_mn[2] > g_mx[2] - 0.005,
        details=f"panel_min_z={p_mn[2]:.3f}, grille_max_z={g_mx[2]:.3f}",
    )

    # --- grille faces the front (+X): thin panel on the front face ---
    gx = (g_mn[0] + g_mx[0]) / 2.0
    gx_ext = g_mx[0] - g_mn[0]
    gy_ext = g_mx[1] - g_mn[1]
    gz_ext = g_mx[2] - g_mn[2]
    ctx.check(
        "speaker grille is a thin front-facing panel near +X",
        gx > 0.03 and gx_ext < gy_ext and gx_ext < gz_ext,
        details=f"grille center x={gx:.3f}, extents=({gx_ext:.3f},{gy_ext:.3f},{gz_ext:.3f})",
    )

    # --- top is plain (no gold panel on top): cabinet top Z extent should
    # not have the gold panel near it.
    ctx.check(
        "cabinet top is plain (no gold panel on top surface)",
        p_mx[2] < CAB_H / 2.0 - 0.010,
        details=f"panel max z={p_mx[2]:.3f}, cabinet top z={CAB_H/2.0:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
