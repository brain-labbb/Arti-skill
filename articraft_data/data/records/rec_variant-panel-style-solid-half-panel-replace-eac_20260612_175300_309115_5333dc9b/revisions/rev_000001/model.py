from __future__ import annotations

# Cascade crowd-control barrier (interlocking steel fence panels).
#
# Real object: a painted steel "cascade" / crowd-control barrier of the type
# linked end-to-end at road works, events, and fairgrounds. Each panel is a
# rounded tubular outer frame with a dense run of thin vertical pickets between
# a top rail and a bottom rail, carried on two splayed A-frame feet. The right
# end of every panel exposes two vertical coupler EYES (top and bottom); the
# left end exposes two vertical coupler PINS. To cascade, the pins of one panel
# drop down through the eyes of its neighbour, forming a vertical hinge so the
# next panel can swing about that coupler line. That coupler hinge is the
# articulated mechanism modelled here: two panels linked by one vertical-axis
# REVOLUTE joint.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ----------------------------------------------------------------------------
# Real-world dimensions (meters).
# ----------------------------------------------------------------------------
PANEL_LEN = 2.00          # overall panel width (X)
PANEL_HEIGHT = 1.10       # frame height (top rail centre above bottom rail centre)
TUBE_R = 0.019            # outer-frame tube radius (~38 mm OD)
CORNER_R = 0.10           # rounded top corner radius of the frame
PICKET_R = 0.0055         # vertical picket radius (~11 mm)
N_PICKETS = 21            # number of vertical infill bars
RAIL_R = 0.014            # top/bottom internal rail tube radius
FRAME_BOTTOM_Z = 0.16     # height of the bottom frame rail centre above ground
FRAME_TOP_Z = FRAME_BOTTOM_Z + PANEL_HEIGHT  # top rail centre height

# Coupler (cascade hinge) geometry, located just past the +X / -X frame edge.
EYE_OUTER_R = 0.024
EYE_INNER_R = 0.013
EYE_THICK = 0.024         # axial (vertical) thickness of one eye ring
PIN_R = 0.011
PIN_LEN = 0.150
COUPLER_DX = 0.055        # how far the coupler centre sits beyond the frame edge

# Coupler heights chosen in the STRAIGHT-post region (below the top corner arc
# and above the foot apex) so the connecting neck reliably fuses into the post.
COUPLER_TOP_Z = FRAME_TOP_Z - 0.18
COUPLER_BOT_Z = FRAME_BOTTOM_Z + 0.18

# A-frame foot geometry.
FOOT_TUBE_R = 0.016
FOOT_SPREAD = 0.32        # half-length of each foot leg along Y at the ground
FOOT_LONG = 0.13          # how far the foot reaches along X from the post

# Colors / materials (dark blue-grey painted steel, per the reference photo).
STEEL_BLUE = (0.27, 0.31, 0.37, 1.0)     # painted dark blue-grey steel frame/feet
STEEL_DARK = (0.21, 0.24, 0.29, 1.0)     # slightly darker pickets
PLATE_STEEL = (0.24, 0.28, 0.34, 1.0)    # solid sheet-metal infill plate
COUPLER_GREY = (0.33, 0.36, 0.41, 1.0)   # coupler eyes/pins


def _strut(p0, p1, radius: float) -> cq.Workplane:
    """A straight tube from p0 to p1."""
    x0, y0, z0 = p0
    x1, y1, z1 = p1
    dx, dy, dz = (x1 - x0, y1 - y0, z1 - z0)
    cyl = cq.Solid.makeCylinder(
        radius,
        math.sqrt(dx * dx + dy * dy + dz * dz),
        cq.Vector(x0, y0, z0),
        cq.Vector(dx, dy, dz),
    )
    return cq.Workplane("XY").add(cyl)


def _tube_chain(pts, radius: float) -> cq.Workplane:
    """A connected tube following a polyline of 3D centreline points.

    Each segment is a straight circular tube; consecutive segments overlap at
    the shared vertex so the chain reads as one continuous bent tube (thin in
    the unused axis). This avoids CadQuery sweep section-orientation blowups.
    """
    chain = None
    for p0, p1 in zip(pts[:-1], pts[1:]):
        seg = _strut(p0, p1, radius)
        chain = seg if chain is None else chain.union(seg)
    return chain


def _frame_solid() -> cq.Workplane:
    """Rounded tubular outer frame: bottom rail, two posts, arched top corners.

    Built from a chain of straight tube segments following the frame centreline
    in the X-Z plane (the panel is thin along Y), matching the photo's rounded
    tube barrier.
    """
    half = PANEL_LEN / 2.0
    z0 = FRAME_BOTTOM_Z
    z1 = FRAME_TOP_Z
    r = CORNER_R

    pts = []
    # Bottom rail, left to right (start at left post base).
    pts.append((-half, 0.0, z0))
    pts.append((half, 0.0, z0))
    # Up the right post to the start of the top-right arc.
    pts.append((half, 0.0, z1 - r))
    # Top-right rounded corner (quarter arc, post -> top rail), centre offset -X.
    cx, cz = (half - r, z1 - r)
    for i in range(1, 9):
        a = (math.pi / 2.0) * (i / 8.0)  # 0 -> pi/2 (post -> top rail)
        pts.append((cx + r * math.cos(a), 0.0, cz + r * math.sin(a)))
    # Top rail, right to left.
    pts.append((-half + r, 0.0, z1))
    # Top-left rounded corner (top rail -> post).
    cx, cz = (-half + r, z1 - r)
    for i in range(1, 9):
        a = (math.pi / 2.0) + (math.pi / 2.0) * (i / 8.0)  # pi/2 -> pi
        pts.append((cx + r * math.cos(a), 0.0, cz + r * math.sin(a)))
    # Down the left post back to the bottom rail start.
    pts.append((-half, 0.0, z0))

    return _tube_chain(pts, TUBE_R)


PLATE_THICK = 0.003       # sheet-metal infill plate thickness (Y)
UPPER_PICKET_FRAC = 0.38  # fraction of infill height reserved for upper pickets


def _lower_plate_solid() -> cq.Workplane:
    """Lower-half solid sheet-metal infill plate with bounding rails.

    The plate fills the lower portion of the panel between the outer frame posts,
    bounded by a bottom internal rail and a mid-height transition rail.
    """
    half = PANEL_LEN / 2.0
    inner_left = -half + TUBE_R + 0.02
    inner_right = half - TUBE_R - 0.02
    span = inner_right - inner_left

    z_bot = FRAME_BOTTOM_Z + 0.045
    z_top = FRAME_TOP_Z - 0.045
    full_h = z_top - z_bot
    z_mid = z_bot + full_h * (1.0 - UPPER_PICKET_FRAC)
    plate_h = z_mid - z_bot

    # Bottom internal rail.
    bottom_rail = (
        cq.Workplane("XY")
        .transformed(rotate=(0.0, 90.0, 0.0))
        .circle(RAIL_R)
        .extrude(span / 2.0 + 0.02, both=True)
        .translate((0.0, 0.0, z_bot))
    )
    # Mid-height transition rail (ties plate top to upper pickets).
    mid_rail = (
        cq.Workplane("XY")
        .transformed(rotate=(0.0, 90.0, 0.0))
        .circle(RAIL_R)
        .extrude(span / 2.0 + 0.02, both=True)
        .translate((0.0, 0.0, z_mid))
    )
    # Solid sheet-metal plate spanning between the rails.
    plate = (
        cq.Workplane("XY")
        .box(span, PLATE_THICK, plate_h, centered=(True, True, False))
        .translate((0.0, 0.0, z_bot))
    )
    return bottom_rail.union(mid_rail).union(plate)


def _upper_pickets_solid() -> cq.Workplane:
    """Short upper row of vertical pickets with a top rail.

    The pickets occupy the upper fraction of the infill zone, running from the
    mid-height transition rail up to the top internal rail.
    """
    half = PANEL_LEN / 2.0
    inner_left = -half + TUBE_R + 0.02
    inner_right = half - TUBE_R - 0.02
    span = inner_right - inner_left

    z_bot = FRAME_BOTTOM_Z + 0.045
    z_top = FRAME_TOP_Z - 0.045
    full_h = z_top - z_bot
    z_mid = z_bot + full_h * (1.0 - UPPER_PICKET_FRAC)
    picket_h = z_top - z_mid

    # Top internal rail.
    top_rail = (
        cq.Workplane("XY")
        .transformed(rotate=(0.0, 90.0, 0.0))
        .circle(RAIL_R)
        .extrude(span / 2.0 + 0.02, both=True)
        .translate((0.0, 0.0, z_top))
    )
    infill = top_rail

    n_upper = max(8, int(N_PICKETS * UPPER_PICKET_FRAC) + 1)
    for i in range(n_upper):
        t = i / (n_upper - 1)
        x = inner_left + t * span
        picket = (
            cq.Workplane("XY")
            .circle(PICKET_R)
            .extrude(picket_h)
            .translate((x, 0.0, z_mid))
        )
        infill = infill.union(picket)
    return infill


def _eye_solid(z: float, x_dir: float) -> cq.Workplane:
    """One coupler eye ring (vertical-axis annulus) on a frame end, with neck.

    The neck starts well inside the straight post so the eye fuses solidly into
    the panel mass (single connected island).
    """
    half = PANEL_LEN / 2.0
    cx = x_dir * (half + COUPLER_DX)
    ring = (
        cq.Workplane("XY")
        .circle(EYE_OUTER_R)
        .circle(EYE_INNER_R)
        .extrude(EYE_THICK / 2.0, both=True)
        .translate((cx, 0.0, z))
    )
    inner_x = x_dir * (half - 0.03)   # bite deep into the post tube
    outer_x = x_dir * (half + COUPLER_DX)
    neck = _strut((inner_x, 0.0, z), (outer_x, 0.0, z), 0.010)
    return ring.union(neck)


def _pin_solid(z: float, x_dir: float) -> cq.Workplane:
    """One vertical coupler pin on a frame end (drops into neighbour eyes)."""
    half = PANEL_LEN / 2.0
    cx = x_dir * (half + COUPLER_DX)
    pin = (
        cq.Workplane("XY")
        .circle(PIN_R)
        .extrude(PIN_LEN / 2.0, both=True)
        .translate((cx, 0.0, z))
    )
    inner_x = x_dir * (half - 0.03)   # bite deep into the post tube
    outer_x = x_dir * (half + COUPLER_DX)
    neck = _strut((inner_x, 0.0, z), (outer_x, 0.0, z), 0.010)
    return pin.union(neck)


def _feet_solid() -> cq.Workplane:
    """Two splayed A-frame feet (one under each post) for free-standing support."""
    half = PANEL_LEN / 2.0
    z_apex = FRAME_BOTTOM_Z + 0.02  # foot apex just under the bottom rail
    # Toe centrelines sit one tube radius above the ground so the tube surface
    # rests ON z=0 instead of sinking a full radius below it.
    z_toe = FOOT_TUBE_R
    foot = None
    for x_dir in (-1.0, 1.0):
        post_x = x_dir * (half - 0.10)
        apex = (post_x, 0.0, z_apex)
        # Two splayed legs (inverted V across Y) plus a forward X stabiliser,
        # reproducing the photo's angled tripod feet.
        for y_dir in (-1.0, 1.0):
            toe = (post_x + x_dir * 0.02, y_dir * FOOT_SPREAD, z_toe)
            leg = _strut(apex, toe, FOOT_TUBE_R)
            foot = leg if foot is None else foot.union(leg)
        toe_f = (post_x + x_dir * FOOT_LONG, 0.0, z_toe)
        foot = foot.union(_strut(apex, toe_f, FOOT_TUBE_R))
    return foot


def _build_panel(x_shift: float):
    """Return CadQuery solids for one barrier panel, shifted +X by ``x_shift``.

    Local panel construction is centred at X=0 (eyes on +X end, pins on -X end).
    ``x_shift`` lets the linked panel be authored so its -X coupler line sits on
    its own part origin, so the cascade hinge can rotate the whole panel about
    that seam.
    """
    solids = {
        "frame": _frame_solid(),
        "lower_plate": _lower_plate_solid(),
        "upper_pickets": _upper_pickets_solid(),
        "feet": _feet_solid(),
        "eye_top": _eye_solid(COUPLER_TOP_Z, +1.0),
        "eye_bottom": _eye_solid(COUPLER_BOT_Z, +1.0),
        "pin_top": _pin_solid(COUPLER_TOP_Z, -1.0),
        "pin_bottom": _pin_solid(COUPLER_BOT_Z, -1.0),
    }
    if x_shift != 0.0:
        solids = {k: v.translate((x_shift, 0.0, 0.0)) for k, v in solids.items()}
    return solids


def _add_panel_visuals(part, solids, tag, m_frame, m_plate, m_picket, m_coupler):
    part.visual(mesh_from_cadquery(solids["frame"], f"frame_{tag}"), material=m_frame, name="frame")
    part.visual(
        mesh_from_cadquery(solids["lower_plate"], f"lower_plate_{tag}"),
        material=m_plate,
        name="lower_plate",
    )
    part.visual(
        mesh_from_cadquery(solids["upper_pickets"], f"upper_pickets_{tag}"),
        material=m_picket,
        name="upper_pickets",
    )
    part.visual(mesh_from_cadquery(solids["feet"], f"feet_{tag}"), material=m_frame, name="feet")
    part.visual(
        mesh_from_cadquery(solids["eye_top"], f"eye_top_{tag}"), material=m_coupler, name="eye_top"
    )
    part.visual(
        mesh_from_cadquery(solids["eye_bottom"], f"eye_bottom_{tag}"),
        material=m_coupler,
        name="eye_bottom",
    )
    part.visual(
        mesh_from_cadquery(solids["pin_top"], f"pin_top_{tag}"), material=m_coupler, name="pin_top"
    )
    part.visual(
        mesh_from_cadquery(solids["pin_bottom"], f"pin_bottom_{tag}"),
        material=m_coupler,
        name="pin_bottom",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cascade_barrier_fence")

    m_frame = model.material("painted_steel", rgba=STEEL_BLUE)
    m_plate = model.material("plate_steel", rgba=PLATE_STEEL)
    m_picket = model.material("picket_steel", rgba=STEEL_DARK)
    m_coupler = model.material("coupler_steel", rgba=COUPLER_GREY)

    half = PANEL_LEN / 2.0
    # The seam: A's +X coupler line and B's -X coupler line share this X.
    seam_x = half + COUPLER_DX

    # ---- Panel A (root), centred at X=0 ------------------------------------
    panel_a = model.part("barrier_panel")
    _add_panel_visuals(panel_a, _build_panel(0.0), "a", m_frame, m_plate, m_picket, m_coupler)

    # ---- Panel B (hinged neighbour) ----------------------------------------
    # B is authored shifted +X so its -X coupler line lands on B's part origin
    # (B's geometry then extends along +X). The cascade hinge frame sits at the
    # seam in A's frame; at q=0 B's origin coincides there, so B sits to A's +X.
    panel_b = model.part("linked_panel")
    b_shift = seam_x  # moves B's -X coupler line to B-local X=0
    _add_panel_visuals(panel_b, _build_panel(b_shift), "b", m_frame, m_plate, m_picket, m_coupler)

    # Cascade coupler hinge: vertical (Z) axis through the shared coupler line.
    # B's coupler pins drop into A's coupler eyes; the whole next panel swings
    # about that vertical pin line.
    model.articulation(
        "cascade_coupler",
        ArticulationType.REVOLUTE,
        parent=panel_a,
        child=panel_b,
        origin=Origin(xyz=(seam_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.5, lower=-1.6, upper=1.6),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    panel_a = object_model.get_part("barrier_panel")
    panel_b = object_model.get_part("linked_panel")
    coupler = object_model.get_articulation("cascade_coupler")

    # ---- Intentional coupler engagement ------------------------------------
    # The cascade hinge is formed by B's coupler pins dropping into A's coupler
    # eyes. That nested pin-in-eye fit is a real, intended interpenetration.
    ctx.allow_overlap(
        panel_a,
        panel_b,
        elem_a="eye_top",
        elem_b="pin_top",
        reason="Linked panel's top coupler pin is seated inside this panel's top eye ring.",
    )
    ctx.allow_overlap(
        panel_a,
        panel_b,
        elem_a="eye_bottom",
        elem_b="pin_bottom",
        reason="Linked panel's bottom coupler pin is seated inside this panel's bottom eye ring.",
    )

    # ---- Joint contract: vertical-axis revolute cascade hinge --------------
    ctx.check(
        "coupler is a vertical-axis revolute hinge",
        coupler.joint_type == "revolute"
        and abs(coupler.axis[2]) > 0.99
        and abs(coupler.axis[0]) < 1e-6
        and abs(coupler.axis[1]) < 1e-6,
        details=f"type={coupler.joint_type} axis={coupler.axis}",
    )

    # ---- Hero geometry present and correctly scaled (panel A) --------------
    aabb_a = ctx.part_world_aabb(panel_a)
    assert aabb_a is not None
    (axmin, aymin, azmin), (axmax, aymax, azmax) = aabb_a
    width_a = axmax - axmin
    height_a = azmax - azmin
    ctx.check(
        "panel A reads as a ~2 m wide, ~1.3 m tall barrier",
        1.9 < width_a < 2.6 and 1.1 < height_a < 1.6,
        details=f"width={width_a:.3f} height={height_a:.3f}",
    )
    ctx.check(
        "panel A stands on the ground (feet near z=0)",
        azmin < 0.02,
        details=f"z_min={azmin:.4f}",
    )

    # Lower solid sheet-metal plate spans the panel width in the lower portion.
    lower_plate = panel_a.get_visual("lower_plate")
    lp_aabb = ctx.part_element_world_aabb(panel_a, elem=lower_plate)
    assert lp_aabb is not None
    (lx0, _, lz0), (lx1, _, lz1) = lp_aabb
    plate_w = lx1 - lx0
    plate_h = lz1 - lz0
    ctx.check(
        "lower solid plate spans the panel width",
        plate_w > 1.6,
        details=f"plate_w={plate_w:.3f}",
    )
    ctx.check(
        "lower plate occupies the lower portion of the panel",
        plate_h > 0.30 and lz0 < 0.40,
        details=f"plate_h={plate_h:.3f} plate_z_min={lz0:.3f}",
    )

    # Upper pickets span the panel width in the upper portion.
    upper_pickets = panel_a.get_visual("upper_pickets")
    up_aabb = ctx.part_element_world_aabb(panel_a, elem=upper_pickets)
    assert up_aabb is not None
    (ux0, _, uz0), (ux1, _, uz1) = up_aabb
    pickets_w = ux1 - ux0
    pickets_h = uz1 - uz0
    ctx.check(
        "upper pickets span the panel width",
        pickets_w > 1.6,
        details=f"pickets_w={pickets_w:.3f}",
    )
    ctx.check(
        "upper pickets occupy the upper portion of the panel",
        pickets_h > 0.20 and uz1 > 1.0,
        details=f"pickets_h={pickets_h:.3f} pickets_z_max={uz1:.3f}",
    )

    # The lower plate sits below the upper pickets (solid-half-panel layout).
    ctx.check(
        "lower plate is below upper pickets (solid half-panel variant)",
        lz1 <= uz0 + 0.02,
        details=f"plate_z_max={lz1:.3f} pickets_z_min={uz0:.3f}",
    )

    # Coupler eyes on +X end, pins on -X end of panel A.
    eye_top = panel_a.get_visual("eye_top")
    pin_top = panel_a.get_visual("pin_top")
    eye_aabb = ctx.part_element_world_aabb(panel_a, elem=eye_top)
    pin_aabb = ctx.part_element_world_aabb(panel_a, elem=pin_top)
    assert eye_aabb is not None and pin_aabb is not None
    eye_cx = (eye_aabb[0][0] + eye_aabb[1][0]) / 2.0
    pin_cx = (pin_aabb[0][0] + pin_aabb[1][0]) / 2.0
    ctx.check(
        "coupler eye on +X end, coupler pin on -X end",
        eye_cx > 0.9 and pin_cx < -0.9,
        details=f"eye_cx={eye_cx:.3f} pin_cx={pin_cx:.3f}",
    )

    # Cascade engagement at rest: B's top coupler pin seats inside A's top eye.
    ctx.expect_overlap(
        panel_a,
        panel_b,
        axes="xy",
        elem_a="eye_top",
        elem_b="pin_top",
        min_overlap=0.008,
        name="linked pin seated inside coupler eye",
    )

    # ---- Two cascaded panels: B sits to the +X side of A -------------------
    aabb_b = ctx.part_world_aabb(panel_b)
    assert aabb_b is not None
    (bxmin, _, bzmin), (bxmax, _, _) = aabb_b
    ctx.check(
        "linked panel B is the neighbour off A's +X coupler end",
        bxmin > axmax - 0.30 and bxmax > axmax + 1.5,
        details=f"A_xmax={axmax:.3f} B_xmin={bxmin:.3f} B_xmax={bxmax:.3f}",
    )
    ctx.check(
        "linked panel B also stands on the ground",
        bzmin < 0.02,
        details=f"B_z_min={bzmin:.4f}",
    )

    # ---- Actuating the hinge swings panel B about the vertical coupler ------
    # Track the AABB centre (an off-axis point) and the far (+X) coupler eye of
    # panel B; the part origin itself lies on the hinge axis so it cannot move.
    def _aabb_center(aabb):
        (xmn, ymn, zmn), (xmx, ymx, zmx) = aabb
        return ((xmn + xmx) / 2.0, (ymn + ymx) / 2.0, (zmn + zmx) / 2.0)

    rest_aabb = ctx.part_world_aabb(panel_b)
    assert rest_aabb is not None
    rest_center = _aabb_center(rest_aabb)
    rest_y_span = rest_aabb[1][1] - rest_aabb[0][1]
    with ctx.pose({coupler: 1.2}):
        swung_aabb = ctx.part_world_aabb(panel_b)
        assert swung_aabb is not None
        swung_center = _aabb_center(swung_aabb)
        swung_y_span = swung_aabb[1][1] - swung_aabb[0][1]
    # Positive rotation about +Z swings B's body (extending along +X from the
    # coupler seam) toward +Y: its AABB centre moves toward -X and +Y, and its
    # Y-footprint grows from a thin panel to most of the panel length.
    ctx.check(
        "rotating the coupler swings panel B about the vertical hinge",
        swung_center[0] < rest_center[0] - 0.3
        and swung_center[1] > rest_center[1] + 0.3
        and swung_y_span > rest_y_span + 0.5,
        details=(
            f"rest_center={tuple(round(v, 3) for v in rest_center)} "
            f"swung_center={tuple(round(v, 3) for v in swung_center)} "
            f"rest_y={rest_y_span:.3f} swung_y={swung_y_span:.3f}"
        ),
    )

    return ctx.report()
