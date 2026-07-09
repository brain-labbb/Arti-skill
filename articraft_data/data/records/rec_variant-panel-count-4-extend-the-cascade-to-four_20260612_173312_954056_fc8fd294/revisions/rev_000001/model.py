from __future__ import annotations

# Cascade crowds-control barrier: four interlinked steel fence panels.
#
# Real object: a painted steel "cascade" / crowd-control barrier of the type
# linked end-to-end at road works, events, and fairgrounds. Each panel is a
# rounded tubular outer frame with a dense run of thin vertical pickets between
# a top rail and a bottom rail, carried on two splayed A-frame feet. The right
# end of every panel exposes two vertical coupler EYES (top and bottom); the
# left end exposes two vertical coupler PINS. To cascade, the pins of one panel
# drop down through the eyes of its neighbour, forming a vertical hinge so the
# next panel can swing about that coupler line. That coupler hinge is the
# articulated mechanism modelled here: four panels chained by three
# vertical-axis REVOLUTE joints.

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

# Number of panels in the cascade.
N_PANELS = 4

# Colors / materials (dark blue-grey painted steel, per the reference photo).
STEEL_BLUE = (0.27, 0.31, 0.37, 1.0)     # painted dark blue-grey steel frame/feet
STEEL_DARK = (0.21, 0.24, 0.29, 1.0)     # slightly darker pickets
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


def _infill_solid() -> cq.Workplane:
    """Top + bottom internal rails plus the dense vertical pickets."""
    half = PANEL_LEN / 2.0
    inner_left = -half + TUBE_R + 0.02
    inner_right = half - TUBE_R - 0.02
    span = inner_right - inner_left

    z_bot = FRAME_BOTTOM_Z + 0.045
    z_top = FRAME_TOP_Z - 0.045
    picket_h = z_top - z_bot

    # Horizontal internal rails (cylinders along X) tie the picket ends together.
    bottom_rail = (
        cq.Workplane("XY")
        .transformed(rotate=(0.0, 90.0, 0.0))
        .circle(RAIL_R)
        .extrude(span / 2.0 + 0.02, both=True)
        .translate((0.0, 0.0, z_bot))
    )
    top_rail = (
        cq.Workplane("XY")
        .transformed(rotate=(0.0, 90.0, 0.0))
        .circle(RAIL_R)
        .extrude(span / 2.0 + 0.02, both=True)
        .translate((0.0, 0.0, z_top))
    )
    infill = bottom_rail.union(top_rail)

    for i in range(N_PICKETS):
        t = i / (N_PICKETS - 1)
        x = inner_left + t * span
        picket = (
            cq.Workplane("XY")
            .circle(PICKET_R)
            .extrude(picket_h)
            .translate((x, 0.0, z_bot))
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
        "infill": _infill_solid(),
        "feet": _feet_solid(),
        "eye_top": _eye_solid(COUPLER_TOP_Z, +1.0),
        "eye_bottom": _eye_solid(COUPLER_BOT_Z, +1.0),
        "pin_top": _pin_solid(COUPLER_TOP_Z, -1.0),
        "pin_bottom": _pin_solid(COUPLER_BOT_Z, -1.0),
    }
    if x_shift != 0.0:
        solids = {k: v.translate((x_shift, 0.0, 0.0)) for k, v in solids.items()}
    return solids


def _add_panel_visuals(part, solids, tag, m_frame, m_picket, m_coupler):
    part.visual(mesh_from_cadquery(solids["frame"], f"frame_{tag}"), material=m_frame, name="frame")
    part.visual(
        mesh_from_cadquery(solids["infill"], f"infill_{tag}"), material=m_picket, name="infill"
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
    m_picket = model.material("picket_steel", rgba=STEEL_DARK)
    m_coupler = model.material("coupler_steel", rgba=COUPLER_GREY)

    half = PANEL_LEN / 2.0
    seam_x = half + COUPLER_DX  # distance from panel centre to coupler line

    # ---- Panel 0 (root), centred at X=0 ------------------------------------
    panels = [model.part("panel_0")]
    _add_panel_visuals(panels[0], _build_panel(0.0), "0", m_frame, m_picket, m_coupler)

    # ---- Linked panels 1..3, chained via vertical coupler hinges -----------
    # Each linked panel is shifted so its -X pin line sits at its part origin
    # (local x=0). The panel body then extends along +X from the hinge seam.
    # The articulation origin is in the parent's frame: for the root panel it
    # is at seam_x (the +X coupler line); for all subsequent parents it is at
    # 2*seam_x (the distance from their origin to their +X coupler line).
    hinges = []
    linked_shift = seam_x  # moves each linked panel's -X pins to local x=0

    for i in range(1, N_PANELS):
        panel = model.part(f"panel_{i}")
        _add_panel_visuals(panel, _build_panel(linked_shift), str(i), m_frame, m_picket, m_coupler)
        panels.append(panel)

        # Articulation origin in parent frame: root panel uses seam_x,
        # all linked parents use 2*seam_x (their +X coupler offset).
        parent_origin_x = seam_x if i == 1 else 2.0 * seam_x
        hinge = model.articulation(
            f"hinge_{i-1}_{i}",
            ArticulationType.REVOLUTE,
            parent=panels[i - 1],
            child=panel,
            origin=Origin(xyz=(parent_origin_x, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=60.0, velocity=1.5, lower=-1.6, upper=1.6),
        )
        hinges.append(hinge)

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    panels = [object_model.get_part(f"panel_{i}") for i in range(N_PANELS)]
    hinges = [object_model.get_articulation(f"hinge_{i}_{i+1}") for i in range(N_PANELS - 1)]

    # ---- Intentional coupler engagement at each seam -----------------------
    # The cascade hinge is formed by child panel's coupler pins dropping into
    # parent panel's coupler eyes. That nested pin-in-eye fit is a real,
    # intended interpenetration at every seam.
    for i in range(N_PANELS - 1):
        parent_panel = panels[i]
        child_panel = panels[i + 1]
        ctx.allow_overlap(
            parent_panel,
            child_panel,
            elem_a="eye_top",
            elem_b="pin_top",
            reason=f"Panel {i+1}'s top coupler pin is seated inside panel {i}'s top eye ring.",
        )
        ctx.allow_overlap(
            parent_panel,
            child_panel,
            elem_a="eye_bottom",
            elem_b="pin_bottom",
            reason=f"Panel {i+1}'s bottom coupler pin is seated inside panel {i}'s bottom eye ring.",
        )

    # ---- Joint contract: all hinges are vertical-axis revolute -------------
    for i, hinge in enumerate(hinges):
        ctx.check(
            f"hinge_{i}_{i+1} is a vertical-axis revolute joint",
            hinge.joint_type == "revolute"
            and abs(hinge.axis[2]) > 0.99
            and abs(hinge.axis[0]) < 1e-6
            and abs(hinge.axis[1]) < 1e-6,
            details=f"type={hinge.joint_type} axis={hinge.axis}",
        )

    # ---- Hero geometry present and correctly scaled (panel 0) --------------
    aabb_0 = ctx.part_world_aabb(panels[0])
    assert aabb_0 is not None
    (axmin, aymin, azmin), (axmax, aymax, azmax) = aabb_0
    width_0 = axmax - axmin
    height_0 = azmax - azmin
    ctx.check(
        "panel 0 reads as a ~2 m wide, ~1.3 m tall barrier",
        1.9 < width_0 < 2.6 and 1.1 < height_0 < 1.6,
        details=f"width={width_0:.3f} height={height_0:.3f}",
    )
    ctx.check(
        "panel 0 stands on the ground (feet near z=0)",
        azmin < 0.02,
        details=f"z_min={azmin:.4f}",
    )

    # Dense vertical pickets: the infill visual spans most of the width/height.
    infill = panels[0].get_visual("infill")
    inf_aabb = ctx.part_element_world_aabb(panels[0], elem=infill)
    assert inf_aabb is not None
    (ix0, _, iz0), (ix1, _, iz1) = inf_aabb
    ctx.check(
        "picket infill spans the panel width and most of its height",
        (ix1 - ix0) > 1.6 and (iz1 - iz0) > 0.8,
        details=f"infill_w={ix1 - ix0:.3f} infill_h={iz1 - iz0:.3f}",
    )

    # Coupler eyes on +X end, pins on -X end of panel 0.
    eye_top = panels[0].get_visual("eye_top")
    pin_top = panels[0].get_visual("pin_top")
    eye_aabb = ctx.part_element_world_aabb(panels[0], elem=eye_top)
    pin_aabb = ctx.part_element_world_aabb(panels[0], elem=pin_top)
    assert eye_aabb is not None and pin_aabb is not None
    eye_cx = (eye_aabb[0][0] + eye_aabb[1][0]) / 2.0
    pin_cx = (pin_aabb[0][0] + pin_aabb[1][0]) / 2.0
    ctx.check(
        "coupler eye on +X end, coupler pin on -X end",
        eye_cx > 0.9 and pin_cx < -0.9,
        details=f"eye_cx={eye_cx:.3f} pin_cx={pin_cx:.3f}",
    )

    # ---- Cascade engagement: each child's pin seated in parent's eye -------
    for i in range(N_PANELS - 1):
        ctx.expect_overlap(
            panels[i],
            panels[i + 1],
            axes="xy",
            elem_a="eye_top",
            elem_b="pin_top",
            min_overlap=0.008,
            name=f"hinge {i}->{i+1} pin seated inside coupler eye",
        )

    # ---- All four panels present and standing on ground ---------------------
    ctx.check(
        "cascade has exactly four panels",
        len(panels) == 4,
        details=f"panel_count={len(panels)}",
    )

    prev_xmax = axmax
    for i in range(1, N_PANELS):
        aabb_i = ctx.part_world_aabb(panels[i])
        assert aabb_i is not None
        (ixmin, _, izmin), (ixmax, _, _) = aabb_i
        ctx.check(
            f"panel {i} is the neighbour off panel {i-1}'s +X coupler end",
            ixmin > prev_xmax - 0.30 and ixmax > prev_xmax + 1.5,
            details=f"prev_xmax={prev_xmax:.3f} panel_{i}_xmin={ixmin:.3f} panel_{i}_xmax={ixmax:.3f}",
        )
        ctx.check(
            f"panel {i} stands on the ground",
            izmin < 0.02,
            details=f"panel_{i}_z_min={izmin:.4f}",
        )
        prev_xmax = ixmax

    # ---- Actuating the first hinge swings panel 1 about the vertical ------
    def _aabb_center(aabb):
        (xmn, ymn, zmn), (xmx, ymx, zmx) = aabb
        return ((xmn + xmx) / 2.0, (ymn + ymx) / 2.0, (zmn + zmx) / 2.0)

    rest_aabb = ctx.part_world_aabb(panels[1])
    assert rest_aabb is not None
    rest_center = _aabb_center(rest_aabb)
    rest_y_span = rest_aabb[1][1] - rest_aabb[0][1]
    with ctx.pose({hinges[0]: 1.2}):
        swung_aabb = ctx.part_world_aabb(panels[1])
        assert swung_aabb is not None
        swung_center = _aabb_center(swung_aabb)
        swung_y_span = swung_aabb[1][1] - swung_aabb[0][1]
    ctx.check(
        "rotating hinge 0->1 swings panel 1 about the vertical coupler",
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
