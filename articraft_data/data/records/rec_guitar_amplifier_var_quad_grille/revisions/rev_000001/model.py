from __future__ import annotations

# Marshall-style mini guitar combo amplifier (black).
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
#                gold panel), dark perforated speaker grille framed by white
#                piping, white cursive "Marshall" logo plate, black corner caps,
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

# 2×2 grille cell grid (replaces the single full-face grille).
CELL_GAP = 0.010                   # gap / divider rib width between cells
GRILLE_SPAN_Y = CAB_W - 0.024     # total available Y inside the front pocket
GRILLE_SPAN_Z = CAB_H - 0.024     # total available Z inside the front pocket
CELL_W = (GRILLE_SPAN_Y - CELL_GAP) / 2.0   # each cell width  (Y): ~0.073
CELL_H = (GRILLE_SPAN_Z - CELL_GAP) / 2.0   # each cell height (Z): ~0.073


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


def _add_grille_cell(body, idx: int, cy: float, cz: float,
                     cell_w: float, cell_h: float,
                     grille_mat, piping_mat):
    """Add one perforated grille cell with its thin piping border to *body*.

    The cell faces +X (the amp front).  *cy*, *cz* are the cell centre in Y/Z.
    """
    # Perforated speaker panel (lies in local XY; rotate so thickness → X).
    cell = PerforatedPanelGeometry(
        (cell_w, cell_h),
        0.005,
        hole_diameter=0.003,
        pitch=0.006,
        frame=0.005,
        stagger=True,
    )
    cell.rotate_y(math.pi / 2.0)          # thickness Z → X
    cell.translate(FRONT_X - 0.004, cy, cz)
    body.visual(mesh_from_geometry(cell, f"grille_cell_{idx}"),
                material=grille_mat, name=f"grille_cell_{idx}")

    # Thin piping border around this cell (four bars forming a rectangle).
    fx = FRONT_X - 0.001
    pt, pw = 0.004, 0.004                  # bar thickness, cross-width
    # Horizontal bars (top, bottom of cell)
    for k, dz in enumerate((cell_h / 2.0, -cell_h / 2.0)):
        bar = BoxGeometry((pt, cell_w + pw, pw))
        bar.translate(fx, cy, cz + dz)
        body.visual(mesh_from_geometry(bar, f"cell_pipe_{idx}_h{k}"),
                    material=piping_mat, name=f"cell_pipe_{idx}_h{k}")
    # Vertical bars (left, right of cell)
    for k, dy in enumerate((-cell_w / 2.0, cell_w / 2.0)):
        bar = BoxGeometry((pt, pw, cell_h + pw))
        bar.translate(fx, cy + dy, cz)
        body.visual(mesh_from_geometry(bar, f"cell_pipe_{idx}_v{k}"),
                    material=piping_mat, name=f"cell_pipe_{idx}_v{k}")


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

    # Gold control panel plate seated in the top recess.
    panel = BoxGeometry((PANEL_D, PANEL_W, 0.004))
    panel.translate(PANEL_X, 0.0, PANEL_GOLD_Z + 0.002)
    body.visual(mesh_from_geometry(panel, "gold_panel"), material=gold_panel, name="gold_panel")

    # Small red power indicator on the gold panel, beside the knob row.
    led = CylinderGeometry(0.0035, 0.004)
    led.translate(PANEL_X - 0.004, 0.072, PANEL_GOLD_Z + 0.004)
    body.visual(mesh_from_geometry(led, "power_led"), material=indicator_red, name="power_led")

    # Front speaker grille: 2×2 grid of four perforated cells, each with its
    # own thin piping border (compact 4-speaker baffle layout).
    for i in range(4):
        row, col = divmod(i, 2)                # row 0=bottom, 1=top; col 0=left, 1=right
        cy = (col - 0.5) * (CELL_W + CELL_GAP)
        cz = (row - 0.5) * (CELL_H + CELL_GAP)
        _add_grille_cell(body, i, cy, cz, CELL_W, CELL_H,
                         grille_dark, white_piping)

    # Cross-divider ribs between the four grille cells (visible black cabinet
    # material in the + shaped gap, seating each cell in its own compartment).
    rib_d = 0.010                               # depth into pocket (X)
    rib_x = FRONT_X - 0.005                     # centered in the pocket
    # Horizontal rib at Z=0
    h_rib = BoxGeometry((rib_d, GRILLE_SPAN_Y, CELL_GAP))
    h_rib.translate(rib_x, 0.0, 0.0)
    body.visual(mesh_from_geometry(h_rib, "grille_rib_h"),
                material=black_vinyl, name="grille_rib_h")
    # Vertical rib at Y=0
    v_rib = BoxGeometry((rib_d, CELL_GAP, GRILLE_SPAN_Z))
    v_rib.translate(rib_x, 0.0, 0.0)
    body.visual(mesh_from_geometry(v_rib, "grille_rib_v"),
                material=black_vinyl, name="grille_rib_v")

    # "Marshall" cursive logo plate (thin raised cream plate on the lower grille).
    logo = BoxGeometry((0.004, 0.080, 0.022))
    logo.translate(FRONT_X + 0.001, -0.006, -0.046)
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


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


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
    # The off-axis raised pointer makes the knob non-axisymmetric. At rest the
    # pointer sits toward +Y, so the knob AABB center is offset from the rotation
    # axis in +Y. A quarter turn about +Z swings that offset into +X. We confirm
    # the in-plane offset rotates ~90 deg while the top Z height is preserved.
    for i, (k, j) in enumerate(zip(knobs, joints)):
        ax = ctx.part_world_position(k)  # joint origin == knob axis (x,y)
        mn0, mx0 = ctx.part_world_aabb(k)
        z_top0 = mx0[2]
        cen0 = ((mn0[0] + mx0[0]) / 2.0 - ax[0], (mn0[1] + mx0[1]) / 2.0 - ax[1])
        with ctx.pose({j: math.pi / 2.0}):
            mn1, mx1 = ctx.part_world_aabb(k)
            z_top1 = mx1[2]
            cen1 = ((mn1[0] + mx1[0]) / 2.0 - ax[0], (mn1[1] + mx1[1]) / 2.0 - ax[1])
        # At rest the offset is mostly in Y; after a quarter turn mostly in X.
        rotated = abs(cen0[1]) > abs(cen0[0]) and abs(cen1[0]) > abs(cen1[1])
        moved = math.hypot(cen1[0] - cen0[0], cen1[1] - cen0[1])
        ctx.check(
            f"knob_{i} spins about the vertical axis",
            rotated and moved > 0.0008 and abs(z_top1 - z_top0) < 0.0015,
            details=f"rest_offset={cen0}, turn_offset={cen1}, moved={moved:.4f}, "
                    f"dz_top={abs(z_top1 - z_top0):.5f}",
        )

    # --- four grille cells in a 2×2 grid on the front face ---
    cell_aabbs = []
    for i in range(4):
        g_mn, g_mx = ctx.part_element_world_aabb(body, elem=f"grille_cell_{i}")
        gx = (g_mn[0] + g_mx[0]) / 2.0
        gx_ext = g_mx[0] - g_mn[0]
        gy_ext = g_mx[1] - g_mn[1]
        gz_ext = g_mx[2] - g_mn[2]
        cell_aabbs.append((g_mn, g_mx))
        ctx.check(
            f"grille_cell_{i} is a thin front-facing panel near +X",
            gx > 0.03 and gx_ext < gy_ext and gx_ext < gz_ext,
            details=f"cell_{i} center x={gx:.3f}, extents=({gx_ext:.3f},{gy_ext:.3f},{gz_ext:.3f})",
        )

    # Verify the 2×2 grid arrangement: cells 0,1 are bottom row; 2,3 are top row.
    # Cells 0,2 are left column; cells 1,3 are right column.
    centers_y = [(aabb[0][1] + aabb[1][1]) / 2.0 for aabb in cell_aabbs]
    centers_z = [(aabb[0][2] + aabb[1][2]) / 2.0 for aabb in cell_aabbs]
    ctx.check(
        "grille cells form a 2×2 grid",
        centers_z[0] < centers_z[2] and centers_z[1] < centers_z[3]
        and centers_y[0] < centers_y[1] and centers_y[2] < centers_y[3],
        details=f"centers_y={[f'{c:.3f}' for c in centers_y]}, centers_z={[f'{c:.3f}' for c in centers_z]}",
    )

    # Verify each cell has its own piping border (4 bars per cell).
    for i in range(4):
        piping_count = sum(
            1 for v in body.visuals
            if v.name and v.name.startswith(f"cell_pipe_{i}_")
        )
        ctx.check(
            f"grille_cell_{i} has piping border",
            piping_count == 4,
            details=f"cell_{i} piping bars={piping_count}",
        )

    return ctx.report()


object_model = build_object_model()
