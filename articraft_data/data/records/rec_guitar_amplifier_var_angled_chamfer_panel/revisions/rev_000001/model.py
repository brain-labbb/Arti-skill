from __future__ import annotations

# Marshall-style mini guitar combo amplifier (black) — angled-facet variant.
# The control panel sits on a chamfered front-top facet that faces up and
# forward toward the player, with knobs spinning about the facet normal.
#
# Coordinate convention:
#   +X : forward, toward the front speaker grille (grille face at +X).
#   +Y : cabinet width (left/right of the front face).
#   +Z : up.
#
# Realistic mini-amp scale:
#   width  (Y) ~ 0.18 m
#   depth  (X) ~ 0.10 m
#   height (Z) ~ 0.18 m
#
# Chamfer facet geometry:
#   The top-front edge of the cabinet is bevelled into a 45-degree slanted
#   facet.  The facet outward normal points up-and-forward at
#   (sin 45, 0, cos 45) ≈ (0.707, 0, 0.707).
#   The gold control panel and four knurled knobs sit on this angled surface,
#   with each knob's CONTINUOUS rotation axis aligned to the facet normal.
#
# Static body  : black vinyl cabinet with chamfered top-front edge, gold panel
#                recessed into the facet, dark perforated speaker grille framed
#                by white piping, white cursive logo plate, black corner caps,
#                and a carry handle on the remaining flat top behind the chamfer.
# Articulations: FOUR knurled control knobs, each CONTINUOUS about the facet
#                normal axis.

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

# --- principal cabinet dimensions (meters) ---
CAB_W = 0.180          # width  (Y)
CAB_D = 0.100          # depth  (X)
CAB_H = 0.180          # height (Z)
WALL = 0.010           # cabinet wall thickness

PANEL_TOP_Z = CAB_H / 2.0          # cabinet top surface z
FRONT_X = CAB_D / 2.0              # front face plane x

# --- chamfer facet geometry ---
CHAMFER_DX = 0.030   # chamfer extent along X (from front face backward)
CHAMFER_DZ = 0.030   # chamfer extent along Z (from top face downward)
FACET_ANGLE = math.atan2(CHAMFER_DZ, CHAMFER_DX)  # π/4 for equal values
FACET_LENGTH = math.hypot(CHAMFER_DX, CHAMFER_DZ)  # surface length ~0.0424

# Facet center (midpoint of the chamfer surface)
FACET_CX = FRONT_X - CHAMFER_DX / 2.0   # 0.035
FACET_CZ = PANEL_TOP_Z - CHAMFER_DZ / 2.0  # 0.075

# Facet outward normal (pointing up-and-forward toward the player)
FACET_NX = math.sin(FACET_ANGLE)  # ≈ 0.7071
FACET_NZ = math.cos(FACET_ANGLE)  # ≈ 0.7071

# --- gold panel on the angled facet ---
PANEL_ALONG_FACET = FACET_LENGTH * 0.85  # ~0.036 along the facet surface
PANEL_W = 0.150                          # across the cabinet width (Y)
PANEL_THICKNESS = 0.004                  # thin plate
PANEL_RECESS_DEPTH = 0.005               # pocket depth into the facet

# --- front grille region (flat face below the chamfer) ---
FRONT_FACE_H = CAB_H - CHAMFER_DZ       # height of flat front face
FRONT_FACE_ZC = -CHAMFER_DZ / 2.0       # center z of flat front face

# --- knobs (evenly spaced across the facet) ---
KNOB_DIAM = 0.018
KNOB_H = 0.014
KNOB_YS = (-0.054, -0.018, 0.018, 0.054)
KNOB_EMBED = 0.001  # press-fit embed depth into facet surface


def _rounded_box(w_y: float, d_x: float, h_z: float, fillet: float) -> cq.Workplane:
    """Box centered at origin with vertical edges filleted."""
    wp = cq.Workplane("XY").box(d_x, w_y, h_z)
    try:
        wp = wp.edges("|Z").fillet(fillet)
    except Exception:
        pass
    return wp


def _cabinet_solid() -> cq.Workplane:
    """Outer cabinet with chamfered top-front edge and front grille pocket."""
    outer = _rounded_box(CAB_W, CAB_D, CAB_H, fillet=0.010)

    # --- chamfer wedge: cut the triangular prism from the top-front edge ---
    # Triangle in XZ (viewed from +Y):
    #   A = (FRONT_X - CHAMFER_DX, PANEL_TOP_Z)    on top face where chamfer starts
    #   B = (FRONT_X + margin, PANEL_TOP_Z + margin)  above/forward of original corner
    #   C = (FRONT_X + margin, PANEL_TOP_Z - CHAMFER_DZ)  on front face
    pad = 0.005
    wedge = (
        cq.Workplane("XZ")
        .moveTo(FRONT_X - CHAMFER_DX, PANEL_TOP_Z)
        .lineTo(FRONT_X + pad, PANEL_TOP_Z + pad)
        .lineTo(FRONT_X + pad, PANEL_TOP_Z - CHAMFER_DZ)
        .close()
        .extrude(CAB_W, both=True)
    )
    body = outer.cut(wedge)

    # --- shallow recess lip for gold panel on the angled facet ---
    # Cut a thin border around the panel seat so the panel reads as recessed.
    # The recess is just a shallow scoring around the panel perimeter; the
    # central seat is left intact so the panel contacts the facet floor.
    recess_outer = (
        cq.Workplane("XY")
        .box(0.003, PANEL_W + 0.008, PANEL_ALONG_FACET + 0.008)
        .rotate((0, 0, 0), (0, 1, 0), -math.degrees(FACET_ANGLE))
        .translate((FACET_CX + 0.001 * FACET_NX, 0.0, FACET_CZ + 0.001 * FACET_NZ))
    )
    recess_inner = (
        cq.Workplane("XY")
        .box(0.004, PANEL_W - 0.002, PANEL_ALONG_FACET - 0.002)
        .rotate((0, 0, 0), (0, 1, 0), -math.degrees(FACET_ANGLE))
        .translate((FACET_CX + 0.001 * FACET_NX, 0.0, FACET_CZ + 0.001 * FACET_NZ))
    )
    # Only cut the border ring (outer minus inner leaves a thin recess groove)
    recess_ring = recess_outer.cut(recess_inner)
    body = body.cut(recess_ring)

    # --- front grille pocket (shallow recess for the grille to seat in) ---
    grille_h = FRONT_FACE_H - 0.024
    grille_pocket = (
        cq.Workplane("XY")
        .box(0.012, CAB_W - 0.024, grille_h)
        .translate((FRONT_X, 0.0, FRONT_FACE_ZC))
    )
    body = body.cut(grille_pocket)
    return body


def _handle_mesh():
    """Top carry handle on the flat area behind the chamfer facet."""
    strap_x = FRONT_X - CHAMFER_DX - 0.020   # on flat top, behind chamfer
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
    strap = profile.sweep(path, multisection=False, makeSolid=True)

    result = strap
    for y in (-span / 2.0, span / 2.0):
        mount = (
            cq.Workplane("XY")
            .box(0.024, 0.018, 0.014)
            .translate((strap_x, y, z0 - 0.004))
        )
        result = result.union(mount)
    return mesh_from_cadquery(result, "handle")


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

    # --- gold control panel on the angled facet ---
    # Build as a thin plate: X = thickness (along normal), Y = width, Z = along facet.
    # rotate_y(-FACET_ANGLE) maps:
    #   local X → facet normal (thickness direction)
    #   local Z → facet surface direction (along-slope)
    # Shift outward by half-thickness minus a small embed so the panel inner face
    # presses into the facet surface (ensures geometric contact with the cabinet).
    PANEL_OUTSET = PANEL_THICKNESS / 2.0 - 0.0005  # embed 0.5 mm into facet
    panel = BoxGeometry((PANEL_THICKNESS, PANEL_W, PANEL_ALONG_FACET))
    panel.rotate_y(-FACET_ANGLE)
    panel.translate(FACET_CX + PANEL_OUTSET * FACET_NX, 0.0,
                    FACET_CZ + PANEL_OUTSET * FACET_NZ)
    body.visual(mesh_from_geometry(panel, "gold_panel"), material=gold_panel, name="gold_panel")

    # --- power LED on the facet (to the right of the knob row) ---
    # Sits on the gold panel outer surface, slightly embedded for contact.
    LED_H = 0.004
    led = CylinderGeometry(0.0035, LED_H)
    # Rotate so cylinder axis (Z) aligns with facet normal
    led.rotate_y(FACET_ANGLE)
    # Panel outer face is at FACET + PANEL_OUTSET*normal + PANEL_THICKNESS/2*normal
    # = FACET + (PANEL_OUTSET + PANEL_THICKNESS/2)*normal
    panel_outer_offset = PANEL_OUTSET + PANEL_THICKNESS / 2.0
    led_seat_offset = panel_outer_offset + LED_H / 2.0 - 0.0005  # embed 0.5mm
    led_x = FACET_CX + led_seat_offset * FACET_NX
    led_z = FACET_CZ + led_seat_offset * FACET_NZ
    led.translate(led_x, 0.072, led_z)
    body.visual(mesh_from_geometry(led, "power_led"), material=indicator_red, name="power_led")

    # --- front speaker grille: dark perforated panel on the flat front face ---
    grille_h = FRONT_FACE_H - 0.030
    grille_w = CAB_W - 0.030
    grille = PerforatedPanelGeometry(
        (grille_w, grille_h),
        0.006,
        hole_diameter=0.0035,
        pitch=0.007,
        frame=0.006,
        stagger=True,
    )
    grille.rotate_y(math.pi / 2.0)  # thickness Z → X
    grille.translate(FRONT_X - 0.004, 0.0, FRONT_FACE_ZC)
    body.visual(mesh_from_geometry(grille, "speaker_grille"), material=grille_dark, name="speaker_grille")

    # --- white piping frame around the grille (four thin bars on the front face) ---
    pip_t = 0.005          # bar thickness (X protrusion)
    pip_w = 0.006          # bar cross width
    fx = FRONT_X - 0.001
    fw = CAB_W - 0.020     # outer span of piping in Y
    fh = FRONT_FACE_H - 0.020  # outer span of piping in Z
    # top & bottom bars (run along Y)
    for i, zc in enumerate((FRONT_FACE_ZC + fh / 2.0, FRONT_FACE_ZC - fh / 2.0)):
        bar = BoxGeometry((pip_t, fw, pip_w))
        bar.translate(fx, 0.0, zc)
        body.visual(mesh_from_geometry(bar, f"piping_h_{i}"),
                    material=white_piping, name=f"piping_h_{i}")
    # left & right bars (run along Z)
    for i, yc in enumerate((fw / 2.0, -fw / 2.0)):
        bar = BoxGeometry((pip_t, pip_w, fh))
        bar.translate(fx, yc, FRONT_FACE_ZC)
        body.visual(mesh_from_geometry(bar, f"piping_v_{i}"),
                    material=white_piping, name=f"piping_v_{i}")

    # --- "Marshall" cursive logo plate (thin raised cream plate on lower grille) ---
    logo = BoxGeometry((0.004, 0.080, 0.022))
    logo.translate(FRONT_X + 0.001, -0.006, FRONT_FACE_ZC - grille_h / 2.0 + 0.020)
    body.visual(mesh_from_geometry(logo, "marshall_logo"), material=logo_cream, name="marshall_logo")

    # --- black corner caps on the four front vertical corners ---
    cap_y = (CAB_W / 2.0) - 0.006
    cap_z_top = (PANEL_TOP_Z - CHAMFER_DZ) - 0.006   # top-front corner (below chamfer)
    cap_z_bot = -(CAB_H / 2.0) + 0.006                # bottom-front corner
    for iy, yc in enumerate((-cap_y, cap_y)):
        for iz, zc in enumerate((cap_z_bot, cap_z_top)):
            cap = BoxGeometry((0.018, 0.016, 0.016))
            cap.translate(FRONT_X - 0.006, yc, zc)
            body.visual(mesh_from_geometry(cap, f"corner_cap_{iy}_{iz}"),
                        material=trim_black, name=f"corner_cap_{iy}_{iz}")

    # --- top carry handle (on flat top behind the chamfer) ---
    body.visual(_handle_mesh(), material=trim_black, name="handle")

    body.inertial = Inertial.from_geometry(
        Box((CAB_D, CAB_W, CAB_H)), mass=2.4, origin=Origin(xyz=(0.0, 0.0, 0.0))
    )

    # --------------------------------------------------------------- knobs
    # Each knob: knurled cylinder built along +Z with base at z=0.
    # After rotate_y(+FACET_ANGLE), the knob extends along the facet normal.
    # CONTINUOUS rotation about the facet normal axis.
    for i in range(len(KNOB_YS)):
        ky = KNOB_YS[i]
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
        # Raised pointer tab near the rim (position marker).
        pointer = BoxGeometry((0.0030, 0.0060, 0.0024))
        pointer.translate(0.0, KNOB_DIAM / 2.0 - 0.0010, KNOB_H + 0.0008)
        knob_geo.merge(pointer)

        # Rotate the knob so its local +Z (extension direction) aligns with
        # the facet normal: rotate_y(+FACET_ANGLE) maps (0,0,1) → (sin θ, 0, cos θ).
        knob_geo.rotate_y(FACET_ANGLE)

        knob_part = model.part(f"knob_{i}")
        knob_part.visual(mesh_from_geometry(knob_geo, f"knob_{i}"),
                         material=metal_knob, name=f"knob_{i}")
        knob_part.inertial = Inertial.from_geometry(
            Cylinder(KNOB_DIAM / 2.0, KNOB_H), mass=0.01
        )

        # Joint origin on the gold panel outer surface, embedded slightly for press-fit.
        # Panel outer face is at FACET + (PANEL_OUTSET + PANEL_THICKNESS/2) * normal.
        panel_outer = PANEL_OUTSET + PANEL_THICKNESS / 2.0
        seat_offset = panel_outer - KNOB_EMBED
        seat_x = FACET_CX + seat_offset * FACET_NX
        seat_z = FACET_CZ + seat_offset * FACET_NZ

        model.articulation(
            f"panel_to_knob_{i}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob_part,
            origin=Origin(xyz=(seat_x, ky, seat_z)),
            axis=(FACET_NX, 0.0, FACET_NZ),
            motion_limits=MotionLimits(effort=0.3, velocity=8.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    knobs = [object_model.get_part(f"knob_{i}") for i in range(4)]
    joints = [object_model.get_articulation(f"panel_to_knob_{i}") for i in range(4)]

    # --- gold panel press-fit into the facet surface ---
    ctx.allow_overlap(
        body,
        body,
        elem_a="gold_panel",
        elem_b="cabinet",
        reason="Gold panel is intentionally embedded 0.5mm into the facet surface for a seated fit.",
    )
    ctx.allow_overlap(
        body,
        body,
        elem_a="power_led",
        elem_b="gold_panel",
        reason="Power LED is intentionally embedded 0.5mm into the gold panel surface.",
    )
    ctx.expect_contact(
        body, body,
        elem_a="gold_panel", elem_b="cabinet",
        name="gold panel contacts the facet surface",
    )

    # --- exactly four knobs on the angled facet ---
    ctx.check(
        "four control knobs present",
        len(knobs) == 4 and all(k is not None for k in knobs),
        details=f"knob parts={[k.name for k in knobs]}",
    )

    # Each knob is seated on the angled facet: world position near the facet
    # center, above the front-face region, and within the panel Y span.
    for i, (k, j) in enumerate(zip(knobs, joints)):
        ctx.allow_overlap(
            k,
            body,
            elem_a=f"knob_{i}",
            elem_b="gold_panel",
            reason="Knob base is intentionally press-fit into the angled facet surface.",
        )
        pos = ctx.part_world_position(k)
        on_facet = (
            pos is not None
            and abs(pos[0] - FACET_CX) <= CHAMFER_DX / 2.0 + 0.006
            and abs(pos[1]) <= PANEL_W / 2.0
            and pos[2] > PANEL_TOP_Z - CHAMFER_DZ - 0.005
            and pos[2] < PANEL_TOP_Z + 0.010
        )
        ctx.check(
            f"knob_{i} seated on the angled facet",
            on_facet,
            details=f"knob_{i} world pos={pos}, facet_center=({FACET_CX:.3f},{FACET_CZ:.3f})",
        )

    # --- knobs tilt forward (not vertical): top of each knob extends forward
    # of its base position (the facet normal has a +X component).
    for i, k in enumerate(knobs):
        pos = ctx.part_world_position(k)
        mn, mx = ctx.part_world_aabb(k)
        knob_top_x = (mn[0] + mx[0]) / 2.0  # approximate center x of knob
        extends_forward = knob_top_x > pos[0] - 0.002
        ctx.check(
            f"knob_{i} tilts forward along facet normal",
            extends_forward,
            details=f"knob_{i} base_x={pos[0]:.4f}, aabb_center_x={knob_top_x:.4f}",
        )

    # --- rotary articulation about the facet normal axis ---
    # The off-axis pointer makes the knob non-axisymmetric. After a half-turn
    # (π radians) about the facet normal, the knob AABB should approximately
    # return to its rest shape (pointer swings to opposite side, but extents
    # are similar). After a quarter turn, the Y extent should change.
    for i, (k, j) in enumerate(zip(knobs, joints)):
        mn0, mx0 = ctx.part_world_aabb(k)
        y_ext_0 = mx0[1] - mn0[1]
        z_ext_0 = mx0[2] - mn0[2]
        pos0 = ctx.part_world_position(k)

        # Quarter turn: the Y-extent should change as the pointer swings
        with ctx.pose({j: math.pi / 2.0}):
            mn1, mx1 = ctx.part_world_aabb(k)
            y_ext_1 = mx1[1] - mn1[1]
            pos1 = ctx.part_world_position(k)

        # The knob position should stay constant (rotation about axis through origin)
        pos_stable = (
            pos0 is not None and pos1 is not None
            and math.sqrt(sum((a - b) ** 2 for a, b in zip(pos0, pos1))) < 0.002
        )
        # The Y extent should change (pointer rotates out of Y)
        y_changed = abs(y_ext_1 - y_ext_0) > 0.0005
        ctx.check(
            f"knob_{i} spins about the facet normal axis",
            pos_stable and y_changed,
            details=f"y_ext_rest={y_ext_0:.4f}, y_ext_quarter={y_ext_1:.4f}, "
                    f"pos_rest={pos0}, pos_quarter={pos1}",
        )

    # --- gold panel is on the angled facet (not on the flat top) ---
    g_mn, g_mx = ctx.part_element_world_aabb(body, elem="gold_panel")
    panel_cx = (g_mn[0] + g_mx[0]) / 2.0
    panel_cz = (g_mn[2] + g_mx[2]) / 2.0
    # Panel center should be near the facet center and below the cabinet top
    ctx.check(
        "gold panel is on the angled facet (below cabinet top, near front)",
        panel_cx > 0.02 and panel_cz < PANEL_TOP_Z - 0.005,
        details=f"panel center=({panel_cx:.3f}, {panel_cz:.3f}), "
                f"facet_center=({FACET_CX:.3f}, {FACET_CZ:.3f})",
    )

    # --- speaker grille faces the front (+X): thin panel near the front face ---
    gr_mn, gr_mx = ctx.part_element_world_aabb(body, elem="speaker_grille")
    gx = (gr_mn[0] + gr_mx[0]) / 2.0
    gx_ext = gr_mx[0] - gr_mn[0]
    gy_ext = gr_mx[1] - gr_mn[1]
    gz_ext = gr_mx[2] - gr_mn[2]
    ctx.check(
        "speaker grille is a thin front-facing panel near +X",
        gx > 0.03 and gx_ext < gy_ext and gx_ext < gz_ext,
        details=f"grille center x={gx:.3f}, extents=({gx_ext:.3f},{gy_ext:.3f},{gz_ext:.3f})",
    )

    return ctx.report()


object_model = build_object_model()
