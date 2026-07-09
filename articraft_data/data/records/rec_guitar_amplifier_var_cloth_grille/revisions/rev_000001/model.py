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
    MeshGeometry,
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


# --- Woven cloth grille ---------------------------------------------------
# The speaker baffle is covered by a fabric grille cloth with a diagonal
# basket-weave texture.  The weave is modelled as real surface relief:
# two families of thin raised fabric ribs cross at ±45° in the YZ plane
# (grille faces +X).  At every crossing one rib passes "over" the other,
# alternating in a checkerboard pattern to produce the classic over-under
# basket-weave look.

# Grille cloth dimensions (match the front pocket opening).
CLOTH_W = CAB_W - 0.030          # span in Y
CLOTH_H = CAB_H - 0.030          # span in Z
CLOTH_T = 0.002                  # backing thickness (X)
CLOTH_X = FRONT_X - 0.004        # front face of backing (world X)

# Rib parameters for the basket-weave surface relief.
RIB_WIDTH = 0.0035               # width of each fabric rib
RIB_RELIEF = 0.0016              # how far a rib stands proud of the backing
RIB_PITCH = 0.022                # spacing between parallel ribs (perpendicular)
RIB_SEG_FRAC = 0.94              # segment length as fraction of pitch (tiny gap)


def _add_rotated_box(mesh: MeshGeometry,
                     x_c: float, y_c: float, z_c: float,
                     angle: float,
                     sx: float, sy: float, sz: float) -> None:
    """Add a box to *mesh* directly, rotated about X by *angle* (rad) and
    translated to (x_c, y_c, z_c).  Avoids the O(n²) merge pattern."""
    c, s = math.cos(angle), math.sin(angle)
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
    # Eight corners of a box centred at origin, then rotated about X and shifted.
    corners = []
    for dx in (-hx, hx):
        for dy in (-hy, hy):
            for dz in (-hz, hz):
                # rotate about X:  y' = y*c - z*s,  z' = y*s + z*c
                ry = dy * c - dz * s
                rz = dy * s + dz * c
                corners.append((x_c + dx, y_c + ry, z_c + rz))
    base = len(mesh.vertices)
    for v in corners:
        mesh.add_vertex(*v)
    # 12 triangles (2 per face).  Corner ordering:
    # 0: (-x,-y,-z)  1: (-x,-y,+z)  2: (-x,+y,-z)  3: (-x,+y,+z)
    # 4: (+x,-y,-z)  5: (+x,-y,+z)  6: (+x,+y,-z)  7: (+x,+y,+z)
    tris = [
        (0, 2, 1), (1, 2, 3),  # -X face
        (4, 5, 6), (5, 7, 6),  # +X face
        (0, 1, 4), (1, 5, 4),  # -Y face (original)
        (2, 6, 3), (3, 6, 7),  # +Y face (original)
        (0, 4, 2), (2, 4, 6),  # -Z face (original)
        (1, 3, 5), (3, 7, 5),  # +Z face (original)
    ]
    for a, b, cc in tris:
        mesh.add_face(base + a, base + b, base + cc)


def _woven_grille_mesh() -> MeshGeometry:
    """Build the full basket-weave grille cloth (backing + two rib families).

    Family A ribs run at +45° in YZ; family B at -45°.
    Crossing (i, j) of A-rib *i* and B-rib *j* lies at:
        y = (j - i) * pitch / √2,   z = (i + j) * pitch / √2
    When (i + j) is even, family A passes *over* B (raised); when odd, *under*.
    """
    sq2 = math.sqrt(2.0)
    half_y = CLOTH_W / 2.0
    half_z = CLOTH_H / 2.0

    # Start with an empty mesh and add geometry directly.
    mesh = MeshGeometry()

    # Cloth backing plate.
    _add_rotated_box(mesh,
                     CLOTH_X - CLOTH_T / 2.0, 0.0, 0.0,
                     0.0, CLOTH_T, CLOTH_W, CLOTH_H)

    # Index range for ribs (covers the full grille rectangle).
    v_max = (half_y + half_z) / sq2
    n_ribs = int(v_max / RIB_PITCH) + 1
    seg_len = RIB_PITCH * RIB_SEG_FRAC

    # X-centre offsets for "over" (proud) and "under" (flush) segments.
    x_over = CLOTH_X + RIB_RELIEF / 2.0
    x_under = CLOTH_X - RIB_RELIEF / 2.0

    # Clip margin: ribs should not protrude past the cloth backing bounds.
    clip_y = half_y - RIB_WIDTH
    clip_z = half_z - RIB_WIDTH

    # ---- Family A: +45° ribs (long axis ↗ in YZ) -------------------------
    for i in range(-n_ribs, n_ribs + 1):
        for j in range(-n_ribs, n_ribs):
            yc = ((j + 0.5) - i) * RIB_PITCH / sq2
            zc = (i + (j + 0.5)) * RIB_PITCH / sq2
            if abs(yc) > clip_y or abs(zc) > clip_z:
                continue
            high = ((i + j) % 2 == 0)
            xc = x_over if high else x_under
            # Box dims: thickness=X(relief), length=Y(seg), width=Z(rib_width)
            # After rotate_x(π/4) the Y axis tilts to +45° in YZ.
            _add_rotated_box(mesh, xc, yc, zc,
                             math.pi / 4.0,
                             RIB_RELIEF, seg_len, RIB_WIDTH)

    # ---- Family B: −45° ribs (long axis ↘ in YZ) -------------------------
    for j in range(-n_ribs, n_ribs + 1):
        for i in range(-n_ribs, n_ribs):
            yc = (j - (i + 0.5)) * RIB_PITCH / sq2
            zc = ((i + 0.5) + j) * RIB_PITCH / sq2
            if abs(yc) > clip_y or abs(zc) > clip_z:
                continue
            high = ((i + j) % 2 != 0)
            xc = x_over if high else x_under
            _add_rotated_box(mesh, xc, yc, zc,
                             -math.pi / 4.0,
                             RIB_RELIEF, seg_len, RIB_WIDTH)

    return mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mini_guitar_amp")

    black_vinyl = model.material("black_vinyl", rgba=(0.07, 0.07, 0.08, 1.0))
    gold_panel = model.material("gold_panel", rgba=(0.86, 0.62, 0.18, 1.0))
    grille_cloth = model.material("grille_cloth", rgba=(0.09, 0.08, 0.07, 1.0))
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

    # Front speaker grille: woven basket-weave cloth with real surface relief.
    # The cloth backing plus two diagonal families of raised fabric ribs form
    # the over-under basket-weave texture.  Framed by the white piping.
    body.visual(mesh_from_geometry(_woven_grille_mesh(), "speaker_grille"),
                material=grille_cloth, name="speaker_grille")

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
    # Seated on the cloth grille front face (backing + over-rib relief).
    _logo_front = CLOTH_X + RIB_RELIEF + 0.002
    logo = BoxGeometry((0.004, 0.080, 0.022))
    logo.translate(_logo_front, -0.006, -0.046)
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

    # --- woven cloth grille at the front (+X) with real surface relief ---
    g_mn, g_mx = ctx.part_element_world_aabb(body, elem="speaker_grille")
    gx = (g_mn[0] + g_mx[0]) / 2.0
    gx_ext = g_mx[0] - g_mn[0]
    gy_ext = g_mx[1] - g_mn[1]
    gz_ext = g_mx[2] - g_mn[2]
    ctx.check(
        "speaker grille is a front-facing cloth panel near +X",
        gx > 0.03 and gx_ext < gy_ext and gx_ext < gz_ext,
        details=f"grille center x={gx:.3f}, extents=({gx_ext:.3f},{gy_ext:.3f},{gz_ext:.3f})",
    )
    # The cloth has surface relief: the basket-weave ribs protrude beyond the
    # flat backing thickness (CLOTH_T).  Total X extent must exceed the bare
    # backing thickness, proving the raised ribs exist.
    ctx.check(
        "woven cloth grille has surface relief beyond the backing",
        gx_ext > CLOTH_T + 0.0005,
        details=f"x_extent={gx_ext:.4f}, backing_t={CLOTH_T:.4f}",
    )
    # The grille cloth spans most of the cabinet width and height.
    ctx.check(
        "woven cloth grille covers the baffle opening",
        gy_ext > CLOTH_W * 0.80 and gz_ext > CLOTH_H * 0.80,
        details=f"y_extent={gy_ext:.3f} (expect>{CLOTH_W * 0.80:.3f}), "
                f"z_extent={gz_ext:.3f} (expect>{CLOTH_H * 0.80:.3f})",
    )

    return ctx.report()


object_model = build_object_model()
