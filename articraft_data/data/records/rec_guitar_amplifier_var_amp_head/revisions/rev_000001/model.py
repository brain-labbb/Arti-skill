from __future__ import annotations

# Marshall-style mini guitar amplifier HEAD unit (black).
# Variant of the combo amp: electronics-only cabinet with NO speaker section.
# Low, wide, and shallow proportions like a tabletop amp head.
#
# Coordinate convention:
#   +X  : forward, toward the front face (front face at +X).
#   +Y  : cabinet width (left/right).
#   +Z  : up; recessed gold control panel on top (+Z), knobs point UP.
#
# Head-unit scale (realistic mini amp head):
#   width  (Y) ~ 0.24 m
#   depth  (X) ~ 0.14 m
#   height (Z) ~ 0.095 m
#
# Front face: plain black vinyl with logo plate and small ventilation slots.
# Top: recessed gold control panel with 4 knurled rotary knobs (CONTINUOUS).
# White piping trim, corner caps, and carry handle as on the combo.

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
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# --- principal dimensions (meters) ---
CAB_W = 0.240          # width  (Y)
CAB_D = 0.140          # depth  (X)
CAB_H = 0.095          # height (Z)
WALL = 0.010           # cabinet wall thickness

# Top recessed gold panel (in the top face), spanning most of the width.
PANEL_TOP_Z = CAB_H / 2.0
PANEL_RECESS = 0.012
PANEL_W = 0.200                    # along Y (wider for head unit)
PANEL_D = 0.060                    # along X
PANEL_X = 0.020                    # panel center X (slightly toward rear)
PANEL_GOLD_Z = PANEL_TOP_Z - PANEL_RECESS

# Front face plane.
FRONT_X = CAB_D / 2.0

# Four knobs evenly spaced across the gold panel.
KNOB_DIAM = 0.018
KNOB_H = 0.014
KNOB_YS = (-0.072, -0.024, 0.024, 0.072)
KNOB_X = PANEL_X + 0.004


def _rounded_box(w_y: float, d_x: float, h_z: float, fillet: float) -> cq.Workplane:
    """Box centered at origin with vertical edges filleted."""
    wp = cq.Workplane("XY").box(d_x, w_y, h_z)
    try:
        wp = wp.edges("|Z").fillet(fillet)
    except Exception:
        pass
    return wp


def _cabinet_solid() -> cq.Workplane:
    """Outer black vinyl cabinet with a rectangular recess cut into the top face
    for the gold control panel. No speaker grille pocket (head unit variant)."""
    outer = _rounded_box(CAB_W, CAB_D, CAB_H, fillet=0.010)

    recess = (
        cq.Workplane("XY")
        .box(PANEL_D, PANEL_W, PANEL_RECESS * 2.2)
        .translate((PANEL_X, 0.0, PANEL_TOP_Z))
    )
    body = outer.cut(recess)

    # Small ventilation cutouts on the front face (upper region).
    # These are shallow recesses, not through-holes; the vent grille sits in them.
    vent_w = 0.100   # along Y
    vent_h = 0.020   # along Z
    vent_depth = 0.006
    vent_x = FRONT_X - vent_depth / 2.0
    vent_z = CAB_H / 2.0 - 0.022   # near top, below panel
    vent = (
        cq.Workplane("XY")
        .box(vent_depth, vent_w, vent_h)
        .translate((vent_x, 0.0, vent_z))
    )
    body = body.cut(vent)

    return body


def _handle_mesh():
    """Top carry handle: arched strap spanning Y, in front of the gold panel."""
    strap_x = PANEL_X - PANEL_D / 2.0 - 0.016
    z0 = PANEL_TOP_Z
    arch = 0.018
    span = 0.140  # wider for head unit

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
    model = ArticulatedObject(name="mini_guitar_amp_head")

    black_vinyl = model.material("black_vinyl", rgba=(0.07, 0.07, 0.08, 1.0))
    gold_panel = model.material("gold_panel", rgba=(0.86, 0.62, 0.18, 1.0))
    vent_dark = model.material("vent_dark", rgba=(0.06, 0.06, 0.07, 1.0))
    white_piping = model.material("white_piping", rgba=(0.92, 0.92, 0.90, 1.0))
    logo_cream = model.material("logo_cream", rgba=(0.95, 0.93, 0.86, 1.0))
    metal_knob = model.material("metal_knob", rgba=(0.78, 0.78, 0.80, 1.0))
    indicator_red = model.material("indicator_red", rgba=(0.85, 0.12, 0.10, 1.0))
    trim_black = model.material("trim_black", rgba=(0.04, 0.04, 0.05, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("body")

    cab = _cabinet_solid()
    body.visual(mesh_from_cadquery(cab, "cabinet"), material=black_vinyl, name="cabinet")

    # Gold control panel plate seated in the top recess.
    panel = BoxGeometry((PANEL_D, PANEL_W, 0.004))
    panel.translate(PANEL_X, 0.0, PANEL_GOLD_Z + 0.002)
    body.visual(mesh_from_geometry(panel, "gold_panel"), material=gold_panel, name="gold_panel")

    # Small red power indicator LED on the gold panel, beside the knob row.
    led = CylinderGeometry(0.0035, 0.004)
    led.translate(PANEL_X - 0.004, 0.096, PANEL_GOLD_Z + 0.004)
    body.visual(mesh_from_geometry(led, "power_led"), material=indicator_red, name="power_led")

    # --- Front face ventilation slots ---
    # Small slotted panel in the upper front region (sits in the vent recess).
    vent_z = CAB_H / 2.0 - 0.022
    vent = SlotPatternPanelGeometry(
        (0.096, 0.018),
        0.004,
        slot_size=(0.030, 0.003),
        pitch=(0.036, 0.006),
        frame=0.003,
        stagger=False,
    )
    vent.rotate_y(math.pi / 2.0)  # thickness Z -> X
    vent.translate(FRONT_X - 0.004, 0.0, vent_z)
    body.visual(mesh_from_geometry(vent, "front_vent_slots"), material=vent_dark, name="front_vent_slots")

    # --- Logo plate ---
    # Thin cream plate with cursive script logo, centered on the lower front face.
    logo = BoxGeometry((0.004, 0.080, 0.018))
    logo.translate(FRONT_X + 0.001, 0.0, -0.012)
    body.visual(mesh_from_geometry(logo, "marshall_logo"), material=logo_cream, name="marshall_logo")

    # --- White piping trim around front face edges ---
    pip_t = 0.005          # bar thickness (X protrusion)
    pip_w = 0.006          # bar cross width
    fx = FRONT_X - 0.001
    fw = CAB_W - 0.020     # outer span along Y
    fh = CAB_H - 0.020     # outer span along Z
    # Top & bottom horizontal bars (run along Y)
    for i, zc in enumerate((fh / 2.0, -fh / 2.0)):
        bar = BoxGeometry((pip_t, fw, pip_w))
        bar.translate(fx, 0.0, zc)
        body.visual(mesh_from_geometry(bar, f"piping_h_{i}"),
                    material=white_piping, name=f"piping_h_{i}")
    # Left & right vertical bars (run along Z)
    for i, yc in enumerate((fw / 2.0, -fw / 2.0)):
        bar = BoxGeometry((pip_t, pip_w, fh))
        bar.translate(fx, yc, 0.0)
        body.visual(mesh_from_geometry(bar, f"piping_v_{i}"),
                    material=white_piping, name=f"piping_v_{i}")

    # --- Black corner caps on the four front vertical corners ---
    cap_y = (CAB_W / 2.0) - 0.006
    cap_z = (CAB_H / 2.0) - 0.006
    for iy, yc in enumerate((-cap_y, cap_y)):
        for iz, zc in enumerate((-cap_z, cap_z)):
            cap = BoxGeometry((0.018, 0.016, 0.016))
            cap.translate(FRONT_X - 0.006, yc, zc)
            body.visual(mesh_from_geometry(cap, f"corner_cap_{iy}_{iz}"),
                        material=trim_black, name=f"corner_cap_{iy}_{iz}")

    # --- Top carry handle ---
    body.visual(_handle_mesh(), material=trim_black, name="handle")

    body.inertial = Inertial.from_geometry(
        Box((CAB_D, CAB_W, CAB_H)), mass=1.8, origin=Origin(xyz=(0.0, 0.0, 0.0))
    )

    # --------------------------------------------------------------- knobs
    # Each knob: knurled cylinder with engraved pointer line, mounting face on
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
            center=False,
        )
        # Raised pointer tab near the rim for visible non-axisymmetry.
        pointer = BoxGeometry((0.0030, 0.0060, 0.0024))
        pointer.translate(0.0, KNOB_DIAM / 2.0 - 0.0010, KNOB_H + 0.0008)
        knob_geo.merge(pointer)

        knob_part = model.part(f"knob_{i}")
        knob_part.visual(mesh_from_geometry(knob_geo, f"knob_{i}"),
                         material=metal_knob, name=f"knob_{i}")
        knob_part.inertial = Inertial.from_geometry(
            Cylinder(KNOB_DIAM / 2.0, KNOB_H), mass=0.01
        )

        # Seat the knob base slightly into the gold panel surface (press-fit).
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

    # --- Head unit proportions: wider than tall ---
    body_mn, body_mx = ctx.part_world_aabb(body)
    dx = body_mx[0] - body_mn[0]
    dy = body_mx[1] - body_mn[1]
    dz = body_mx[2] - body_mn[2]
    ctx.check(
        "head unit is wider than tall",
        dy > dz * 1.5,
        details=f"cabinet extents: dx={dx:.3f} dy={dy:.3f} dz={dz:.3f}",
    )
    ctx.check(
        "head unit is short (cabinet body under 0.10m, total with handle under 0.14m)",
        dz < 0.14,
        details=f"height={dz:.3f}",
    )

    # --- No speaker grille: front face should NOT have a large perforated panel ---
    body_vis_names = [v.name for v in body.visuals]
    ctx.check(
        "no speaker grille on the front face",
        "speaker_grille" not in body_vis_names,
        details=f"visual names: {body_vis_names}",
    )

    # --- Ventilation slots present on front face ---
    ctx.check(
        "ventilation slots on the front face",
        "front_vent_slots" in body_vis_names,
        details=f"visual names: {body_vis_names}",
    )
    vent_mn, vent_mx = ctx.part_element_world_aabb(body, elem="front_vent_slots")
    vent_x_center = (vent_mn[0] + vent_mx[0]) / 2.0
    ctx.check(
        "vent slots are near the front face (+X)",
        vent_x_center > 0.04,
        details=f"vent center x={vent_x_center:.3f}",
    )

    # --- Logo plate on front face ---
    ctx.check(
        "logo plate present on front",
        "marshall_logo" in body_vis_names,
        details=f"visual names: {body_vis_names}",
    )

    # --- Four knobs on the gold panel ---
    ctx.check(
        "four control knobs present",
        len(knobs) == 4 and all(k is not None for k in knobs),
        details=f"knob parts={[k.name for k in knobs]}",
    )

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

    # Knobs point up above the gold surface.
    for i, k in enumerate(knobs):
        mn, mx = ctx.part_world_aabb(k)
        ctx.check(
            f"knob_{i} stands proud above the panel",
            mx[2] > PANEL_GOLD_Z + 0.004,
            details=f"knob_{i} top z={mx[2]:.4f}",
        )

    # --- Rotary articulation: knob spins about the vertical axis ---
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

    # --- White piping trim present ---
    for name in ("piping_h_0", "piping_h_1", "piping_v_0", "piping_v_1"):
        ctx.check(
            f"{name} present on front",
            name in body_vis_names,
            details=f"visual names: {body_vis_names}",
        )

    return ctx.report()


object_model = build_object_model()
