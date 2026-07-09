from __future__ import annotations

# Chrome wire-basket shopping cart (supermarket trolley) -- high fidelity.
#
# World frame: Z up. The BACK (handle / push end) is +X, the FRONT (the small,
# low, nesting end) is -X. Width is Y. The four wheels touch z ~ 0.
#
# Reading the reference (005.png):
#   - A tapered CHROME WIRE basket: narrow/low at the front, wide/tall at the
#     back, open top. The cross-section is a rounded RECTANGLE (near-vertical
#     flat walls with slightly rounded corners). The mesh is DENSE: many vertical
#     wires per face plus several longitudinal rails and a fine floor grid.
#   - A RED plastic push handle bar across the top-back with RED end caps, and
#     RED plastic corner bumper caps at the front top corners.
#   - A splayed chrome tubular underframe carrying four rubber-tired swivel
#     casters tucked directly under the leg bottoms.
#   - A lower wire TRAY shelf between the legs.
#   - A fold-down CHILD-SEAT flap at the back interior.
#
# Articulations:
#   - PRIMARY: four swivel casters: each YAWs about a vertical kingpin
#     (continuous) and ROLLs about its axle (continuous).
#   - SECONDARY: child-seat flap is REVOLUTE about a transverse (Y) hinge.
#
# Root part = basket. Underframe, handle, bumpers FIXED to the basket. Wheels
# attach through the caster yokes.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TireGeometry,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)

WIRE_R = 0.0028  # basket lattice wire radius (thin)
RAIL_R = 0.0050  # perimeter rail / longitudinal rail radius (thicker)
FRAME_R = 0.0110  # chrome underframe tube radius

# ---- basket envelope (tapered rounded-rectangle) ----
# back (handle) end is +X, front (nesting) end is -X
B_BACK_X = 0.32
B_FRONT_X = -0.40
B_BACK_TOP_Z = 0.66
B_FRONT_TOP_Z = 0.575
B_BACK_BOT_Z = 0.435
B_FRONT_BOT_Z = 0.405
B_BACK_HALF_Y = 0.240  # wide at back
B_FRONT_HALF_Y = 0.170  # narrow at front
B_BOT_BACK_HALF_Y = 0.210
B_BOT_FRONT_HALF_Y = 0.140
CORNER_FRAC = 0.16  # rounded-corner radius as fraction of half-width

WHEEL_RADIUS = 0.050
WHEEL_WIDTH = 0.026
FORK_OFFSET = 0.028

# underframe geometry
REAR_AXLE_X = 0.255
FRONT_AXLE_X = -0.345
TRACK_HALF_Y = 0.205  # wheel track half-width
CHASSIS_Z = 0.150  # caster mount height (kingpin top)


def _tube(points, name, *, radius=WIRE_R, sps=4, rs=6):
    if len(points) == 2:
        sps = 1
    geom = tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=sps,
        radial_segments=rs,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, name)


def _straight_tube_geom(p0, p1, *, radius=WIRE_R, rs=8):
    return tube_from_spline_points(
        [p0, p1],
        radius=radius,
        samples_per_segment=1,
        radial_segments=rs,
        cap_ends=True,
    )


def _straight_path_mesh(points, name, *, radius=WIRE_R, rs=8, closed=False):
    geom = MeshGeometry()
    pairs = list(zip(points, points[1:]))
    if closed and len(points) > 2:
        pairs.append((points[-1], points[0]))
    for p0, p1 in pairs:
        geom.merge(_straight_tube_geom(p0, p1, radius=radius, rs=rs))
    return mesh_from_geometry(geom, name)


def _lerp(a, b, t):
    return a + (b - a) * t


# ---------------------------------------------------------------------------
# Cross-section geometry.
#
# At longitudinal fraction t (0=front, 1=back) and height fraction h (0=bottom,
# 1=top), the wall traces a rounded rectangle of half-width hy(t,h) (in Y) and
# the wall sits at world x(t) and z(t,h). We parameterize the perimeter by a
# fraction u in [0,1) going around the loop, and return the (x,y,z) world point.
# ---------------------------------------------------------------------------


def _xt(t):
    return _lerp(B_FRONT_X, B_BACK_X, t)


def _half_y(t, h):
    top = _lerp(B_FRONT_HALF_Y, B_BACK_HALF_Y, t)
    bot = _lerp(B_BOT_FRONT_HALF_Y, B_BOT_BACK_HALF_Y, t)
    return _lerp(bot, top, h)


def _z(t, h):
    top = _lerp(B_FRONT_TOP_Z, B_BACK_TOP_Z, t)
    bot = _lerp(B_FRONT_BOT_Z, B_BACK_BOT_Z, t)
    return _lerp(bot, top, h)


# The basket spans t in [0,1] (front..back). The end walls are flat planes at
# t=0 and t=1. The two long side walls run along Y = +/- half_y.
#
# We represent a wall point by an (x, y, z) directly. Side walls: fixed sy, vary
# t and h. End walls: fixed t, vary "fy" across the width (-1..1) and h.


def _side_pt(t, sy, h):
    return (_xt(t), sy * _half_y(t, h), _z(t, h))


def _end_pt(t, fy, h):
    # fy in [-1, 1] across the (tapered) full width at this end.
    return (_xt(t), fy * _half_y(t, h), _z(t, h))


def _floor_pt(t, fy):
    # bottom-face point; floor is slightly bowed? keep flat at h=0.
    return (_xt(t), fy * _half_y(t, 0.0), _z(t, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wire_shopping_cart")

    chrome = model.material("chrome", rgba=(0.82, 0.84, 0.87, 1.0))
    chrome_rail = model.material("chrome_rail", rgba=(0.76, 0.78, 0.82, 1.0))
    chrome_frame = model.material("chrome_frame", rgba=(0.72, 0.74, 0.78, 1.0))
    red = model.material("red_plastic", rgba=(0.80, 0.09, 0.09, 1.0))
    rubber = model.material("wheel_tread", rgba=(0.15, 0.15, 0.17, 1.0))
    steel = model.material("caster_steel", rgba=(0.54, 0.55, 0.57, 1.0))
    rim_silver = model.material("rim_silver", rgba=(0.80, 0.81, 0.83, 1.0))

    # =====================================================================
    # BASKET (root)
    # =====================================================================
    basket = model.part("basket")

    # Lattice station tables (shared so rails pass exactly through every wire).
    n_side = 18
    side_ts = [(i + 0.5) / n_side for i in range(n_side)]
    n_end = 9
    end_fys = [-1.0 + 2.0 * (i + 0.5) / n_end for i in range(n_end)]

    # ---- perimeter rails at several heights (closed rounded-rect loops) ----
    # A rail at height fraction h traces the full perimeter passing through every
    # side-wire station and end-wire station, so each vertical wire lands exactly
    # on the rim/rail (guaranteed connectivity, no floating wires). Built as one
    # closed tube => a single connected ring per height.
    def perimeter_loop(h, nm, *, radius=RAIL_R):
        pts = []
        # front end wall: left..right at t=0 (include the two corners explicitly)
        pts.append(_end_pt(0.0, -1.0, h))
        for fy in end_fys:
            pts.append(_end_pt(0.0, fy, h))
        pts.append(_end_pt(0.0, 1.0, h))
        # +Y side wall front..back through every side station
        for t in side_ts:
            pts.append(_side_pt(t, 1.0, h))
        pts.append(_side_pt(1.0, 1.0, h))
        # back end wall: right..left at t=1
        for fy in reversed(end_fys):
            pts.append(_end_pt(1.0, fy, h))
        pts.append(_end_pt(1.0, -1.0, h))
        # -Y side wall back..front through every side station
        for t in reversed(side_ts):
            pts.append(_side_pt(t, -1.0, h))
        basket.visual(
            _straight_path_mesh(pts, nm, radius=radius, rs=8, closed=True),
            material=chrome_rail,
            name=nm,
        )

    # top rim (thickest), bottom rim, plus longitudinal mid rails per face.
    perimeter_loop(1.00, "top_rim", radius=RAIL_R + 0.0010)
    perimeter_loop(0.00, "bottom_rim", radius=RAIL_R + 0.0008)
    for i, h in enumerate((0.22, 0.46, 0.70, 0.88)):
        perimeter_loop(h, f"long_rail_{i}", radius=RAIL_R - 0.0008)

    # ---- side-wall vertical wires (DENSE): top rim -> bottom rim ----
    for sy in (-1.0, 1.0):
        side = "p" if sy > 0 else "n"
        for i, t in enumerate(side_ts):
            top = _side_pt(t, sy, 1.0)
            bot = _side_pt(t, sy, 0.0)
            basket.visual(
                _tube([top, bot], f"side_vwire_{side}_{i}", radius=WIRE_R),
                material=chrome,
                name=f"side_vwire_{side}_{i}",
            )

    # ---- end-wall vertical wires (DENSE) at front and back ----
    for nm_end, t in (("front", 0.0), ("back", 1.0)):
        for i, fy in enumerate(end_fys):
            top = _end_pt(t, fy, 1.0)
            bot = _end_pt(t, fy, 0.0)
            basket.visual(
                _tube([top, bot], f"{nm_end}_vwire_{i}", radius=WIRE_R),
                material=chrome,
                name=f"{nm_end}_vwire_{i}",
            )

    # ---- basket floor grid (longitudinal + transverse wires) ----
    # longitudinal wires run front..back at constant width fraction.
    n_floor_l = 9
    for i in range(n_floor_l):
        fy = -1.0 + 2.0 * (i + 0.5) / n_floor_l
        a = _floor_pt(0.0, fy)
        b = _floor_pt(1.0, fy)
        basket.visual(
            _tube([a, b], f"floor_lwire_{i}", radius=WIRE_R),
            material=chrome,
            name=f"floor_lwire_{i}",
        )
    # transverse wires run across the width at length stations.
    n_floor_t = 14
    for i in range(n_floor_t):
        t = (i + 0.5) / n_floor_t
        a = _floor_pt(t, -1.0)
        b = _floor_pt(t, 1.0)
        basket.visual(
            _tube([a, b], f"floor_twire_{i}", radius=WIRE_R),
            material=chrome,
            name=f"floor_twire_{i}",
        )

    # =====================================================================
    # FRONT ADVERTISING / NAMEPLATE PANEL (fixed visuals on basket).
    #
    # A prominent molded-plastic panel clamped onto the front wire wall.
    # Rounded corners, a raised border frame, shallow embossed stripes,
    # a logo placeholder, and four clamp tabs that grip the top/bottom
    # rim wires.  All visuals live on the basket part (not separate
    # moving parts) so they cannot interfere with casters or child seat.
    # =====================================================================
    panel_plastic = model.material("panel_blue", rgba=(0.12, 0.32, 0.68, 1.0))
    panel_accent = model.material("panel_accent", rgba=(0.85, 0.85, 0.88, 1.0))

    _pw = 0.260   # panel width (Y)
    _ph = 0.145   # panel height (Z)
    _pt = 0.004   # panel plate thickness (X)
    _pr = 0.015   # corner radius
    # Back face flush with the wire plane at x = B_FRONT_X.
    _px = B_FRONT_X - _pt / 2.0
    _pz = (B_FRONT_BOT_Z + B_FRONT_TOP_Z) / 2.0

    # --- main panel plate ---
    _prof = rounded_rect_profile(_ph, _pw, _pr)
    _body = ExtrudeGeometry(_prof, _pt, center=True, closed=True)
    _body.rotate_y(-math.pi / 2.0)
    _body.translate(_px, 0.0, _pz)
    basket.visual(
        mesh_from_geometry(_body, "front_panel_body"),
        material=panel_plastic,
        name="front_panel_body",
    )

    # --- raised border frame (proud of the outer face) ---
    _bw = 0.008   # border width
    _bp = 0.002   # border proud thickness
    _outer = rounded_rect_profile(_ph, _pw, _pr)
    _inner = rounded_rect_profile(
        _ph - 2 * _bw, _pw - 2 * _bw, max(_pr - _bw, 0.003)
    )
    _frame = ExtrudeWithHolesGeometry(
        _outer, [_inner], _bp, center=True, closed=True
    )
    _frame.rotate_y(-math.pi / 2.0)
    _frame.translate(_px - _pt / 2.0 - _bp / 2.0, 0.0, _pz)
    basket.visual(
        mesh_from_geometry(_frame, "front_panel_border"),
        material=panel_plastic,
        name="front_panel_border",
    )

    # --- embossed horizontal stripes (inside the border field) ---
    _sw = _pw - 2 * _bw - 0.020   # stripe width
    _sh = 0.006                    # stripe height (Z)
    _st = 0.0015                   # stripe proud thickness
    for i, dz in enumerate((-0.032, 0.032)):
        sg = BoxGeometry((_st, _sw, _sh))
        sg.translate(_px - _pt / 2.0 - _st / 2.0, 0.0, _pz + dz)
        basket.visual(
            mesh_from_geometry(sg, f"front_panel_stripe_{i}"),
            material=panel_accent,
            name=f"front_panel_stripe_{i}",
        )

    # --- logo placeholder (raised rectangle at centre) ---
    _lw, _lh, _lt = 0.050, 0.022, 0.0015
    _lg = BoxGeometry((_lt, _lw, _lh))
    _lg.translate(_px - _pt / 2.0 - _lt / 2.0, 0.0, _pz)
    basket.visual(
        mesh_from_geometry(_lg, "front_panel_logo"),
        material=panel_accent,
        name="front_panel_logo",
    )

    # --- clamp tabs (bridge from panel edges to rim wires) ---
    _tw = 0.018   # tab width (Y)
    _td = 0.012   # tab depth (X, into the basket)
    _tab_positions = (
        (-0.055, 1.0),   # top-left
        (0.055, 1.0),    # top-right
        (-0.055, -1.0),  # bottom-left
        (0.055, -1.0),   # bottom-right
    )
    for i, (y_off, z_sign) in enumerate(_tab_positions):
        if z_sign > 0:
            _th = (B_FRONT_TOP_Z + 0.004) - (_pz + _ph / 2.0)
            _tz = (_pz + _ph / 2.0 + B_FRONT_TOP_Z + 0.004) / 2.0
        else:
            _th = (_pz - _ph / 2.0) - (B_FRONT_BOT_Z - 0.004)
            _tz = (B_FRONT_BOT_Z - 0.004 + _pz - _ph / 2.0) / 2.0
        tg = BoxGeometry((_td, _tw, max(_th, 0.008)))
        tg.translate(_px + _td / 2.0 - 0.001, y_off, _tz)
        basket.visual(
            mesh_from_geometry(tg, f"front_panel_tab_{i}"),
            material=panel_plastic,
            name=f"front_panel_tab_{i}",
        )

    basket.inertial = Inertial.from_geometry(
        Box((B_BACK_X - B_FRONT_X, 2.0 * B_BACK_HALF_Y, B_BACK_TOP_Z - B_BACK_BOT_Z)),
        mass=6.0,
        origin=Origin(
            xyz=(
                (B_BACK_X + B_FRONT_X) / 2.0,
                0.0,
                (B_BACK_TOP_Z + B_BACK_BOT_Z) / 2.0,
            )
        ),
    )

    # =====================================================================
    # RED push handle bar across the top-back (FIXED), with red end caps.
    # =====================================================================
    handle = model.part("push_handle")
    htz = B_BACK_TOP_Z + 0.028
    hx = B_BACK_X + 0.030
    bar_half = B_BACK_HALF_Y + 0.006
    handle.visual(
        Cylinder(radius=0.0145, length=2.0 * bar_half - 0.04),
        origin=Origin(xyz=(hx, 0.0, htz), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=red,
        name="handle_bar",
    )
    # red rounded end caps (slightly fatter, at each end)
    for sy in (-1.0, 1.0):
        side = "p" if sy > 0 else "n"
        handle.visual(
            Cylinder(radius=0.018, length=0.040),
            origin=Origin(
                xyz=(hx, sy * (bar_half - 0.024), htz), rpy=(math.pi / 2.0, 0.0, 0.0)
            ),
            material=red,
            name=f"handle_cap_{side}",
        )
        # short uprights linking the handle bar to the back rim corners
        tc = _side_pt(1.0, sy, 1.0)
        handle.visual(
            _tube(
                [(tc[0], tc[1], tc[2]), (hx, sy * (bar_half - 0.024), htz)],
                f"handle_post_{side}",
                radius=0.0075,
                rs=8,
            ),
            material=red,
            name=f"handle_post_{side}",
        )
    handle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0145, length=2.0 * bar_half),
        mass=0.4,
        origin=Origin(xyz=(hx, 0.0, htz), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    model.articulation(
        "basket_to_handle", ArticulationType.FIXED, parent=basket, child=handle
    )

    # =====================================================================
    # RED plastic front corner bumpers (one part each, FIXED).
    # =====================================================================
    for sy in (-1.0, 1.0):
        side = "p" if sy > 0 else "n"
        tc = _side_pt(0.0, sy, 1.0)
        bmp = model.part(f"front_bumper_{side}")
        bmp.visual(
            Box((0.052, 0.046, 0.058)),
            origin=Origin(xyz=(tc[0] + 0.006, tc[1], tc[2] - 0.010)),
            material=red,
            name="bumper",
        )
        bmp.inertial = Inertial.from_geometry(Box((0.05, 0.05, 0.05)), mass=0.1)
        model.articulation(
            f"basket_to_bumper_{side}",
            ArticulationType.FIXED,
            parent=basket,
            child=bmp,
        )

    # =====================================================================
    # CHROME underframe (FIXED to basket): splayed legs + lower tray.
    # =====================================================================
    frame = model.part("underframe")

    # Side longitudinal chassis rails running just above the wheels.
    for sy in (-1.0, 1.0):
        side = "p" if sy > 0 else "n"
        rail = [
            (FRONT_AXLE_X - 0.02, sy * TRACK_HALF_Y, CHASSIS_Z),
            (REAR_AXLE_X + 0.02, sy * TRACK_HALF_Y, CHASSIS_Z),
        ]
        frame.visual(
            _tube(rail, f"chassis_rail_{side}", radius=FRAME_R, rs=10),
            material=chrome_frame,
            name=f"chassis_rail_{side}",
        )
    # transverse cross tubes tying the rails at the axle lines.
    for i, x in enumerate((FRONT_AXLE_X, REAR_AXLE_X)):
        frame.visual(
            Cylinder(radius=FRAME_R, length=2.0 * TRACK_HALF_Y),
            origin=Origin(xyz=(x, 0.0, CHASSIS_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=chrome_frame,
            name=f"cross_tube_{i}",
        )

    # splayed struts from chassis up to the basket bottom rim (front + rear).
    for sy in (-1.0, 1.0):
        side = "p" if sy > 0 else "n"
        bc_back = _side_pt(1.0, sy, 0.0)
        frame.visual(
            _tube(
                [
                    (REAR_AXLE_X, sy * TRACK_HALF_Y, CHASSIS_Z),
                    (bc_back[0] - 0.02, bc_back[1], bc_back[2] + 0.01),
                ],
                f"rear_strut_{side}",
                radius=FRAME_R,
                rs=10,
            ),
            material=chrome_frame,
            name=f"rear_strut_{side}",
        )
        bc_front = _side_pt(0.0, sy, 0.0)
        frame.visual(
            _tube(
                [
                    (FRONT_AXLE_X, sy * TRACK_HALF_Y, CHASSIS_Z),
                    (bc_front[0] + 0.02, bc_front[1], bc_front[2] + 0.01),
                ],
                f"front_strut_{side}",
                radius=FRAME_R,
                rs=10,
            ),
            material=chrome_frame,
            name=f"front_strut_{side}",
        )

    # ---- lower wire TRAY shelf between the legs (a wire grid platform) ----
    tray_z = CHASSIS_Z + 0.012
    tray_back_x = REAR_AXLE_X - 0.01
    tray_front_x = FRONT_AXLE_X + 0.01
    tray_half_y = TRACK_HALF_Y - 0.018
    # perimeter frame of the tray (closed loop). Include edge midpoints so the
    # catmull-rom does not bow the rectangle into a lens shape.
    def _edge(a, b, n):
        return [
            (
                _lerp(a[0], b[0], (k + 1) / (n + 1)),
                _lerp(a[1], b[1], (k + 1) / (n + 1)),
                tray_z,
            )
            for k in range(n)
        ]

    c_fn = (tray_front_x, -tray_half_y, tray_z)
    c_fp = (tray_front_x, tray_half_y, tray_z)
    c_bp = (tray_back_x, tray_half_y, tray_z)
    c_bn = (tray_back_x, -tray_half_y, tray_z)
    tray_loop = (
        [c_fn]
        + _edge(c_fn, c_fp, 1)
        + [c_fp]
        + _edge(c_fp, c_bp, 5)
        + [c_bp]
        + _edge(c_bp, c_bn, 1)
        + [c_bn]
        + _edge(c_bn, c_fn, 5)
    )
    frame.visual(
        _straight_path_mesh(tray_loop, "tray_rim", radius=RAIL_R, rs=8, closed=True),
        material=chrome_frame,
        name="tray_rim",
    )
    # longitudinal tray wires
    n_tray_l = 7
    for i in range(n_tray_l):
        y = _lerp(-tray_half_y, tray_half_y, (i + 0.5) / n_tray_l)
        frame.visual(
            _tube(
                [(tray_front_x, y, tray_z), (tray_back_x, y, tray_z)],
                f"tray_lwire_{i}",
                radius=WIRE_R,
            ),
            material=chrome_frame,
            name=f"tray_lwire_{i}",
        )
    # transverse tray wires
    n_tray_t = 11
    for i in range(n_tray_t):
        x = _lerp(tray_front_x, tray_back_x, (i + 0.5) / n_tray_t)
        frame.visual(
            _tube(
                [(x, -tray_half_y, tray_z), (x, tray_half_y, tray_z)],
                f"tray_twire_{i}",
                radius=WIRE_R,
            ),
            material=chrome_frame,
            name=f"tray_twire_{i}",
        )

    frame.inertial = Inertial.from_geometry(
        Box((REAR_AXLE_X - FRONT_AXLE_X + 0.1, 2.0 * TRACK_HALF_Y, 0.3)),
        mass=5.0,
        origin=Origin(
            xyz=((REAR_AXLE_X + FRONT_AXLE_X) / 2.0, 0.0, CHASSIS_Z + 0.12)
        ),
    )
    model.articulation(
        "basket_to_underframe", ArticulationType.FIXED, parent=basket, child=frame
    )

    # =====================================================================
    # CHILD-SEAT flap (REVOLUTE, transverse hinge at the back interior).
    # At q=0 the flap stands UP (+Z). Positive q folds it DOWN/forward to a seat.
    # =====================================================================
    flap = model.part("child_seat_flap")
    flap_w = 2.0 * B_BACK_HALF_Y - 0.10
    flap_len = 0.175
    flap.visual(
        Box((0.012, flap_w, flap_len)),
        origin=Origin(xyz=(0.0, 0.0, flap_len / 2.0)),
        material=chrome,
        name="seat_panel",
    )
    for i, fy in enumerate((-0.66, 0.0, 0.66)):
        flap.visual(
            Box((0.016, 0.006, flap_len)),
            origin=Origin(xyz=(0.0, fy * flap_w / 2.0, flap_len / 2.0)),
            material=chrome,
            name=f"seat_wire_{i}",
        )
    flap.inertial = Inertial.from_geometry(
        Box((0.012, flap_w, flap_len)),
        mass=0.3,
        origin=Origin(xyz=(0.0, 0.0, flap_len / 2.0)),
    )
    hinge_x = B_BACK_X - 0.006
    hinge_z = 0.500
    model.articulation(
        "basket_to_seat_flap",
        ArticulationType.REVOLUTE,
        parent=basket,
        child=flap,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.5),
    )

    # =====================================================================
    # WHEELS: four swivel casters mounted under the leg bottoms.
    # =====================================================================
    leg_half_y = WHEEL_WIDTH / 2.0 + 0.009

    def add_caster(idx, cx, cy, mount_z):
        yoke = model.part(f"caster_yoke_{idx}")
        # local frame: origin at the kingpin top (chassis underside, z=mount_z).
        lb = -(mount_z - WHEEL_RADIUS)  # wheel-center z in local frame
        lt = lb + WHEEL_RADIUS + 0.010
        yoke.visual(
            Box((0.044, 0.044, 0.009)),
            origin=Origin(xyz=(0.0, 0.0, -0.0045)),
            material=steel,
            name="swivel_plate",
        )
        yoke.visual(
            Cylinder(radius=0.011, length=0.038),
            origin=Origin(xyz=(0.0, 0.0, -0.023)),
            material=steel,
            name="kingpin",
        )
        yoke.visual(
            Box((FORK_OFFSET + 0.024, 0.034, 0.012)),
            origin=Origin(xyz=(-FORK_OFFSET / 2.0, 0.0, -0.040)),
            material=steel,
            name="offset_bracket",
        )
        yoke.visual(
            Box((0.028, leg_half_y * 2.0 + 0.012, 0.012)),
            origin=Origin(xyz=(-FORK_OFFSET, 0.0, lt)),
            material=steel,
            name="fork_crown",
        )
        leg_h = abs(lb - lt) + 0.014
        for sy in (-1.0, 1.0):
            yoke.visual(
                Box((0.013, 0.011, leg_h)),
                origin=Origin(
                    xyz=(-FORK_OFFSET, sy * leg_half_y, (lt + lb) / 2.0)
                ),
                material=steel,
                name=f"fork_leg_{'p' if sy > 0 else 'n'}",
            )
        yoke.visual(
            Cylinder(radius=0.005, length=leg_half_y * 2.0 + 0.016),
            origin=Origin(
                xyz=(-FORK_OFFSET, 0.0, lb), rpy=(math.pi / 2.0, 0.0, 0.0)
            ),
            material=steel,
            name="axle",
        )
        yoke.inertial = Inertial.from_geometry(
            Box((0.05, 0.05, 0.10)),
            mass=0.4,
            origin=Origin(xyz=(-FORK_OFFSET, 0.0, -0.05)),
        )

        wheel = model.part(f"caster_wheel_{idx}")
        rim_geom = WheelGeometry(
            WHEEL_RADIUS * 0.66,
            WHEEL_WIDTH * 0.7,
            rim=WheelRim(inner_radius=WHEEL_RADIUS * 0.55, flange_height=0.003),
            hub=WheelHub(
                radius=WHEEL_RADIUS * 0.45, width=WHEEL_WIDTH * 0.8, cap_style="flat"
            ),
            face=WheelFace(dish_depth=0.0),
            spokes=WheelSpokes(style="disc"),
            bore=WheelBore(style="round", diameter=0.006),
        )
        rim_geom.rotate_z(math.pi / 2.0)
        wheel.visual(
            mesh_from_geometry(rim_geom, f"caster_rim_{idx}"),
            material=rim_silver,
            name="rim",
        )
        tire_geom = TireGeometry(
            WHEEL_RADIUS,
            WHEEL_WIDTH,
            inner_radius=WHEEL_RADIUS * 0.64,
            tread=TireTread(style="circumferential", depth=0.0015, count=1),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
        )
        tire_geom.rotate_z(math.pi / 2.0)
        wheel.visual(
            mesh_from_geometry(tire_geom, f"caster_tire_{idx}"),
            material=rubber,
            name="tire",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
            mass=0.3,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        )

        model.articulation(
            f"frame_to_caster_yoke_{idx}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=yoke,
            origin=Origin(xyz=(cx, cy, mount_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=6.0, velocity=12.0),
        )
        model.articulation(
            f"caster_spin_{idx}",
            ArticulationType.CONTINUOUS,
            parent=yoke,
            child=wheel,
            origin=Origin(xyz=(-FORK_OFFSET, 0.0, lb)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=40.0),
        )

    # four casters mount to the chassis directly under the leg bottoms.
    add_caster(0, REAR_AXLE_X, TRACK_HALF_Y, CHASSIS_Z)
    add_caster(1, REAR_AXLE_X, -TRACK_HALF_Y, CHASSIS_Z)
    add_caster(2, FRONT_AXLE_X, TRACK_HALF_Y, CHASSIS_Z)
    add_caster(3, FRONT_AXLE_X, -TRACK_HALF_Y, CHASSIS_Z)

    return model


def run_tests():
    from sdk import TestContext

    ctx = TestContext(object_model)
    basket = object_model.get_part("basket")
    frame = object_model.get_part("underframe")
    handle = object_model.get_part("push_handle")
    flap = object_model.get_part("child_seat_flap")
    ctx.check("basket_present", basket is not None, "missing")

    # captured axle in each wheel hub: scoped overlap
    for i in range(4):
        ctx.allow_overlap(
            object_model.get_part(f"caster_yoke_{i}"),
            object_model.get_part(f"caster_wheel_{i}"),
            elem_a="axle",
            elem_b="rim",
            reason="The fork axle is captured inside the wheel hub bore.",
        )

    # Each caster bolts onto the chassis at the mount interface: scoped overlap.
    for i in range(4):
        yoke = object_model.get_part(f"caster_yoke_{i}")
        ctx.allow_overlap(
            yoke,
            frame,
            reason="The caster swivel plate/kingpin bolts onto the chassis frame.",
        )

    # Mounted attachments seat onto the basket frame: scoped intentional overlaps.
    ctx.allow_overlap(handle, basket, reason="The handle posts clamp onto the back rim.")
    for side in ("p", "n"):
        ctx.allow_overlap(
            object_model.get_part(f"front_bumper_{side}"),
            basket,
            reason="The bumper caps over the front rim corner.",
        )
    ctx.allow_overlap(
        frame, basket, reason="The underframe struts weld to the basket bottom rim."
    )
    ctx.allow_overlap(
        flap, basket, reason="The seat-flap hinge edge attaches to the back wires."
    )

    # ---- Basket is tapered: back wider and taller than front. ----
    fc_top = _side_pt(0.0, 1.0, 1.0)
    bc_top = _side_pt(1.0, 1.0, 1.0)
    ctx.check(
        "basket_wider_at_back",
        bc_top[1] > fc_top[1] + 0.04,
        f"front_halfY={fc_top[1]:.3f} back_halfY={bc_top[1]:.3f}",
    )
    ctx.check(
        "basket_taller_at_back",
        bc_top[2] > fc_top[2],
        f"front_topZ={fc_top[2]:.3f} back_topZ={bc_top[2]:.3f}",
    )
    ba = ctx.part_world_aabb(basket)
    if ba is not None:
        ctx.check(
            "basket_above_ground", ba[0][2] >= 0.30, f"basket_bottom={ba[0][2]:.3f}"
        )

    # ---- Dense mesh present: count the lattice wires actually authored. ----
    vis_names = [v.name for v in basket.visuals]
    n_side_v = sum(1 for n in vis_names if n.startswith("side_vwire_"))
    n_end_v = sum(1 for n in vis_names if "_vwire_" in n and ("front" in n or "back" in n))
    n_floor = sum(1 for n in vis_names if n.startswith("floor_"))
    n_rail = sum(1 for n in vis_names if n.startswith("long_rail_") or n in ("top_rim", "bottom_rim"))
    ctx.check("dense_side_wires", n_side_v >= 30, f"side vwires={n_side_v}")
    ctx.check("dense_end_wires", n_end_v >= 14, f"end vwires={n_end_v}")
    ctx.check("dense_floor_grid", n_floor >= 18, f"floor wires={n_floor}")
    ctx.check("longitudinal_rails", n_rail >= 4, f"rails={n_rail}")

    # ---- Lower wire tray present between the legs. ----
    frame_names = [v.name for v in frame.visuals]
    n_tray = sum(1 for n in frame_names if n.startswith("tray_"))
    ctx.check("lower_tray_present", n_tray >= 10, f"tray wires={n_tray}")
    tray_rim = next((v for v in frame.visuals if v.name == "tray_rim"), None)
    ctx.check("tray_rim_present", tray_rim is not None, "missing tray_rim")

    # ---- Wheels touch the floor. ----
    lows = []
    for i in range(4):
        wa = ctx.part_world_aabb(object_model.get_part(f"caster_wheel_{i}"))
        if wa is not None:
            lows.append(wa[0][2])
    ctx.check("four_wheels", len(lows) == 4, f"found {len(lows)}")
    if lows:
        ctx.check(
            "wheels_touch_floor",
            all(abs(z) <= 0.012 for z in lows),
            f"lows={['%.3f' % z for z in lows]}",
        )

    # ---- Casters tucked under the leg bottoms (wheel center under chassis). ----
    for i in range(4):
        yoke = object_model.get_part(f"caster_yoke_{i}")
        ya = ctx.part_world_aabb(yoke)
        if ya is not None:
            ctx.check(
                f"caster_{i}_under_frame",
                ya[1][2] <= CHASSIS_Z + 0.02,
                f"yoke_top={ya[1][2]:.3f}",
            )

    # ---- Red handle sits at the top-back, above the basket back rim. ----
    ha = ctx.part_world_aabb(handle)
    if ha is not None and ba is not None:
        ctx.check(
            "handle_at_back_top",
            ha[1][0] > 0.25 and ha[1][2] >= ba[1][2] - 0.02,
            f"handle_xmax={ha[1][0]:.3f} handle_top={ha[1][2]:.3f}",
        )

    # ---- Joint inventory: 4 spin + 4 swivel continuous, 1 revolute seat. ----
    for i in range(4):
        sw = object_model.get_articulation(f"frame_to_caster_yoke_{i}")
        sp = object_model.get_articulation(f"caster_spin_{i}")
        ctx.check(
            f"swivel_{i}_continuous_z",
            sw.articulation_type == ArticulationType.CONTINUOUS
            and tuple(sw.axis) == (0.0, 0.0, 1.0),
            f"axis={sw.axis}",
        )
        ctx.check(
            f"spin_{i}_continuous_y",
            sp.articulation_type == ArticulationType.CONTINUOUS
            and tuple(sp.axis) == (0.0, 1.0, 0.0),
            f"axis={sp.axis}",
        )
    seat = object_model.get_articulation("basket_to_seat_flap")
    ctx.check(
        "seat_flap_revolute_y",
        seat.articulation_type == ArticulationType.REVOLUTE
        and abs(tuple(seat.axis)[1]) == 1.0
        and tuple(seat.axis)[0] == 0.0
        and tuple(seat.axis)[2] == 0.0,
        f"axis={seat.axis}",
    )

    # ---- Decisive pose: wheel spins in place (center fixed). ----
    wheel0 = object_model.get_part("caster_wheel_0")
    spin0 = object_model.get_articulation("caster_spin_0")
    rest = ctx.part_world_position(wheel0)
    with ctx.pose({spin0: 0.6}):
        turned = ctx.part_world_position(wheel0)
    if rest is not None and turned is not None:
        moved = sum((turned[k] - rest[k]) ** 2 for k in range(3)) ** 0.5
        ctx.check("wheel_spins_in_place", moved < 1e-4, f"moved={moved:.5f}")

    # ---- Decisive pose: the seat flap folds DOWN/forward. ----
    rest_flap = ctx.part_world_aabb(flap)
    with ctx.pose({seat: 1.4}):
        down_flap = ctx.part_world_aabb(flap)
    if rest_flap is not None and down_flap is not None:
        top_dropped = down_flap[1][2] < rest_flap[1][2] - 0.08
        reaches_forward = down_flap[0][0] < rest_flap[0][0] - 0.08
        ctx.check(
            "seat_flap_folds_down",
            top_dropped and reaches_forward,
            f"rest_top={rest_flap[1][2]:.3f} down_top={down_flap[1][2]:.3f} "
            f"rest_xmin={rest_flap[0][0]:.3f} down_xmin={down_flap[0][0]:.3f}",
        )

    # =================================================================
    # FRONT ADVERTISING PANEL checks
    # =================================================================
    # --- Panel visual names exist on the basket ---
    panel_names = (
        "front_panel_body",
        "front_panel_border",
        "front_panel_logo",
    )
    for nm in panel_names:
        ctx.check(
            f"panel_visual_{nm}",
            nm in vis_names,
            f"missing visual '{nm}' on basket",
        )
    n_stripes = sum(1 for n in vis_names if n.startswith("front_panel_stripe_"))
    n_tabs = sum(1 for n in vis_names if n.startswith("front_panel_tab_"))
    ctx.check("panel_stripes_present", n_stripes >= 2, f"stripes={n_stripes}")
    ctx.check("panel_tabs_present", n_tabs >= 4, f"tabs={n_tabs}")

    # --- Panel body sits at the front of the cart (correct X position) ---
    panel_body_aabb = ctx.part_element_world_aabb(basket, elem="front_panel_body")
    if panel_body_aabb is not None:
        ctx.check(
            "panel_at_front_x",
            panel_body_aabb[1][0] <= B_FRONT_X + 0.005,
            f"panel_xmax={panel_body_aabb[1][0]:.4f} expected<={B_FRONT_X + 0.005:.4f}",
        )
        ctx.check(
            "panel_centered_z",
            abs(
                (panel_body_aabb[0][2] + panel_body_aabb[1][2]) / 2.0
                - (B_FRONT_BOT_Z + B_FRONT_TOP_Z) / 2.0
            )
            < 0.01,
            f"panel_z_center={(panel_body_aabb[0][2] + panel_body_aabb[1][2]) / 2:.4f}",
        )

    # --- Panel does not interfere with caster wheels (Z separation) ---
    if panel_body_aabb is not None:
        for i in range(4):
            wa = ctx.part_world_aabb(object_model.get_part(f"caster_wheel_{i}"))
            if wa is not None:
                ctx.check(
                    f"panel_clear_wheel_{i}",
                    panel_body_aabb[0][2] > wa[1][2] + 0.10,
                    f"panel_bot_z={panel_body_aabb[0][2]:.3f} wheel_top_z={wa[1][2]:.3f}",
                )

    # --- Panel does not interfere with child seat flap at max fold ---
    if panel_body_aabb is not None:
        with ctx.pose({seat: 1.5}):
            flap_fold_aabb = ctx.part_world_aabb(flap)
        if flap_fold_aabb is not None:
            ctx.check(
                "panel_clear_seat_flap_folded",
                flap_fold_aabb[0][0] > panel_body_aabb[1][0] + 0.05,
                f"flap_xmin={flap_fold_aabb[0][0]:.3f} panel_xmax={panel_body_aabb[1][0]:.3f}",
            )

    return ctx.report()


object_model = build_object_model()
