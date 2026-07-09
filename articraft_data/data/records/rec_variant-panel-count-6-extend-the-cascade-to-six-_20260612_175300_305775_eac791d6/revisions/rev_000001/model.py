from __future__ import annotations

# Cascade crowd-control barrier – six interlocking steel fence panels.
#
# Real object: a painted steel "cascade" / crowd-control barrier of the type
# linked end-to-end at road works, events, and fairgrounds. Each panel is a
# rounded tubular outer frame with a dense run of thin vertical pickets between
# a top rail and a bottom rail, carried on two splayed A-frame feet. The right
# end of every panel exposes two vertical coupler EYES (top and bottom); the
# left end exposes two vertical coupler PINS. To cascade, the pins of one panel
# drop down through the eyes of its neighbour, forming a vertical hinge so the
# next panel can swing about that coupler line.
#
# This variant chains six identical panels in a line. Panel 0 is the root;
# panels 1..5 are each hinged to the previous panel by a vertical-axis revolute
# coupler joint.

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
N_PANELS = 6

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

# Derived seam spacing.
HALF = PANEL_LEN / 2.0
SEAM_X = HALF + COUPLER_DX  # distance from panel centre to its coupler line

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
    """A connected tube following a polyline of 3D centreline points."""
    chain = None
    for p0, p1 in zip(pts[:-1], pts[1:]):
        seg = _strut(p0, p1, radius)
        chain = seg if chain is None else chain.union(seg)
    return chain


def _frame_solid() -> cq.Workplane:
    """Rounded tubular outer frame: bottom rail, two posts, arched top corners."""
    half = PANEL_LEN / 2.0
    z0 = FRAME_BOTTOM_Z
    z1 = FRAME_TOP_Z
    r = CORNER_R

    pts = []
    pts.append((-half, 0.0, z0))
    pts.append((half, 0.0, z0))
    pts.append((half, 0.0, z1 - r))
    cx, cz = (half - r, z1 - r)
    for i in range(1, 9):
        a = (math.pi / 2.0) * (i / 8.0)
        pts.append((cx + r * math.cos(a), 0.0, cz + r * math.sin(a)))
    pts.append((-half + r, 0.0, z1))
    cx, cz = (-half + r, z1 - r)
    for i in range(1, 9):
        a = (math.pi / 2.0) + (math.pi / 2.0) * (i / 8.0)
        pts.append((cx + r * math.cos(a), 0.0, cz + r * math.sin(a)))
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
    """One coupler eye ring (vertical-axis annulus) on a frame end, with neck."""
    half = PANEL_LEN / 2.0
    cx = x_dir * (half + COUPLER_DX)
    ring = (
        cq.Workplane("XY")
        .circle(EYE_OUTER_R)
        .circle(EYE_INNER_R)
        .extrude(EYE_THICK / 2.0, both=True)
        .translate((cx, 0.0, z))
    )
    inner_x = x_dir * (half - 0.03)
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
    inner_x = x_dir * (half - 0.03)
    outer_x = x_dir * (half + COUPLER_DX)
    neck = _strut((inner_x, 0.0, z), (outer_x, 0.0, z), 0.010)
    return pin.union(neck)


def _feet_solid() -> cq.Workplane:
    """Two splayed A-frame feet (one under each post) for free-standing support."""
    half = PANEL_LEN / 2.0
    z_apex = FRAME_BOTTOM_Z + 0.02
    z_toe = FOOT_TUBE_R
    foot = None
    for x_dir in (-1.0, 1.0):
        post_x = x_dir * (half - 0.10)
        apex = (post_x, 0.0, z_apex)
        for y_dir in (-1.0, 1.0):
            toe = (post_x + x_dir * 0.02, y_dir * FOOT_SPREAD, z_toe)
            leg = _strut(apex, toe, FOOT_TUBE_R)
            foot = leg if foot is None else foot.union(leg)
        toe_f = (post_x + x_dir * FOOT_LONG, 0.0, z_toe)
        foot = foot.union(_strut(apex, toe_f, FOOT_TUBE_R))
    return foot


def _build_panel(x_shift: float) -> dict[str, cq.Workplane]:
    """Return CadQuery solids for one barrier panel, shifted +X by ``x_shift``.

    Local panel construction is centred at X=0 (eyes on +X end, pins on -X end).
    ``x_shift`` positions the panel so its -X coupler line lands at local X=0
    (for non-root panels), letting the cascade hinge rotate the whole panel
    about that seam.
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
    """Attach all visuals for one panel part."""
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
    model = ArticulatedObject(name="cascade_barrier_fence_6")

    m_frame = model.material("painted_steel", rgba=STEEL_BLUE)
    m_picket = model.material("picket_steel", rgba=STEEL_DARK)
    m_coupler = model.material("coupler_steel", rgba=COUPLER_GREY)

    panels = []
    joints = []

    for i in range(N_PANELS):
        part_name = f"panel_{i}"
        panel = model.part(part_name)

        # Root panel (i=0) is centred at its part origin.
        # Child panels (i>0) are shifted so their -X coupler line sits on their
        # part origin (the seam with the previous panel).
        x_shift = 0.0 if i == 0 else SEAM_X
        _add_panel_visuals(panel, _build_panel(x_shift), f"p{i}", m_frame, m_picket, m_coupler)
        panels.append(panel)

    # Chain the panels with vertical-axis revolute coupler hinges.
    # Joint i connects panel_i (parent) to panel_{i+1} (child).
    for i in range(N_PANELS - 1):
        # The joint origin is at the +X coupler line of the parent panel, in
        # the parent's local frame.
        # - Root panel (i=0): +X eyes at local X = SEAM_X
        # - Child panels (i>0): +X eyes at local X = SEAM_X + SEAM_X = 2*SEAM_X
        joint_x = SEAM_X if i == 0 else 2.0 * SEAM_X

        joint = model.articulation(
            f"coupler_{i}",
            ArticulationType.REVOLUTE,
            parent=panels[i],
            child=panels[i + 1],
            origin=Origin(xyz=(joint_x, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=60.0, velocity=1.5, lower=-1.6, upper=1.6),
        )
        joints.append(joint)

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    panels = [object_model.get_part(f"panel_{i}") for i in range(N_PANELS)]
    couplers = [object_model.get_articulation(f"coupler_{i}") for i in range(N_PANELS - 1)]

    # ---- Intentional coupler engagement at every seam ----------------------
    # Each child panel's coupler pins drop into the parent panel's coupler eyes.
    for i in range(N_PANELS - 1):
        ctx.allow_overlap(
            panels[i],
            panels[i + 1],
            elem_a="eye_top",
            elem_b="pin_top",
            reason=f"Coupler {i}: child top pin seated inside parent top eye ring.",
        )
        ctx.allow_overlap(
            panels[i],
            panels[i + 1],
            elem_a="eye_bottom",
            elem_b="pin_bottom",
            reason=f"Coupler {i}: child bottom pin seated inside parent bottom eye ring.",
        )

    # ---- All coupler joints are vertical-axis revolute ---------------------
    for i, jnt in enumerate(couplers):
        ctx.check(
            f"coupler_{i} is a vertical-axis revolute hinge",
            jnt.joint_type == "revolute"
            and abs(jnt.axis[2]) > 0.99
            and abs(jnt.axis[0]) < 1e-6
            and abs(jnt.axis[1]) < 1e-6,
            details=f"type={jnt.joint_type} axis={jnt.axis}",
        )

    # ---- Six panels present and correctly scaled ----------------------------
    ctx.check(
        "six cascade panels exist",
        len(panels) == N_PANELS,
        details=f"found {len(panels)} panels",
    )

    # Root panel geometry check.
    aabb_0 = ctx.part_world_aabb(panels[0])
    assert aabb_0 is not None
    (x0min, y0min, z0min), (x0max, y0max, z0max) = aabb_0
    width_0 = x0max - x0min
    height_0 = z0max - z0min
    ctx.check(
        "panel 0 reads as a ~2 m wide, ~1.3 m tall barrier",
        1.9 < width_0 < 2.6 and 1.1 < height_0 < 1.6,
        details=f"width={width_0:.3f} height={height_0:.3f}",
    )
    ctx.check(
        "panel 0 stands on the ground (feet near z=0)",
        z0min < 0.02,
        details=f"z_min={z0min:.4f}",
    )

    # Picket infill spans the panel.
    infill_0 = panels[0].get_visual("infill")
    inf_aabb = ctx.part_element_world_aabb(panels[0], elem=infill_0)
    assert inf_aabb is not None
    (ix0, _, iz0), (ix1, _, iz1) = inf_aabb
    ctx.check(
        "picket infill spans the panel width and most of its height",
        (ix1 - ix0) > 1.6 and (iz1 - iz0) > 0.8,
        details=f"infill_w={ix1 - ix0:.3f} infill_h={iz1 - iz0:.3f}",
    )

    # Coupler eyes on +X end, pins on -X end of root panel.
    eye_aabb = ctx.part_element_world_aabb(panels[0], elem=panels[0].get_visual("eye_top"))
    pin_aabb = ctx.part_element_world_aabb(panels[0], elem=panels[0].get_visual("pin_top"))
    assert eye_aabb is not None and pin_aabb is not None
    eye_cx = (eye_aabb[0][0] + eye_aabb[1][0]) / 2.0
    pin_cx = (pin_aabb[0][0] + pin_aabb[1][0]) / 2.0
    ctx.check(
        "coupler eye on +X end, coupler pin on -X end",
        eye_cx > 0.9 and pin_cx < -0.9,
        details=f"eye_cx={eye_cx:.3f} pin_cx={pin_cx:.3f}",
    )

    # ---- Cascade chain extends along +X from root --------------------------
    # Each successive panel should be positioned further along +X.
    prev_xmax = x0max
    for i in range(1, N_PANELS):
        aabb_i = ctx.part_world_aabb(panels[i])
        assert aabb_i is not None
        (xmin_i, _, zmin_i), (xmax_i, _, _) = aabb_i

        ctx.check(
            f"panel_{i} sits to the +X side of panel_{i-1}",
            xmin_i > prev_xmax - 0.30 and xmax_i > prev_xmax + 1.5,
            details=f"prev_xmax={prev_xmax:.3f} panel_{i}_xmin={xmin_i:.3f} xmax={xmax_i:.3f}",
        )
        ctx.check(
            f"panel_{i} stands on the ground",
            zmin_i < 0.02,
            details=f"z_min={zmin_i:.4f}",
        )
        prev_xmax = xmax_i

    # ---- Pin-in-eye engagement at first seam --------------------------------
    ctx.expect_overlap(
        panels[0],
        panels[1],
        axes="xy",
        elem_a="eye_top",
        elem_b="pin_top",
        min_overlap=0.008,
        name="coupler 0: linked pin seated inside coupler eye",
    )

    # ---- Actuating the first coupler swings panel 1 about the vertical hinge
    def _aabb_center(aabb):
        (xmn, ymn, zmn), (xmx, ymx, zmx) = aabb
        return ((xmn + xmx) / 2.0, (ymn + ymx) / 2.0, (zmn + zmx) / 2.0)

    rest_aabb = ctx.part_world_aabb(panels[1])
    assert rest_aabb is not None
    rest_center = _aabb_center(rest_aabb)
    rest_y_span = rest_aabb[1][1] - rest_aabb[0][1]

    with ctx.pose({couplers[0]: 1.2}):
        swung_aabb = ctx.part_world_aabb(panels[1])
        assert swung_aabb is not None
        swung_center = _aabb_center(swung_aabb)
        swung_y_span = swung_aabb[1][1] - swung_aabb[0][1]

    ctx.check(
        "rotating coupler 0 swings panel 1 about the vertical hinge",
        swung_center[0] < rest_center[0] - 0.3
        and swung_center[1] > rest_center[1] + 0.3
        and swung_y_span > rest_y_span + 0.5,
        details=(
            f"rest_center={tuple(round(v, 3) for v in rest_center)} "
            f"swung_center={tuple(round(v, 3) for v in swung_center)} "
            f"rest_y={rest_y_span:.3f} swung_y={swung_y_span:.3f}"
        ),
    )

    # ---- Five coupler joints exist -----------------------------------------
    ctx.check(
        "five cascade coupler joints chain the six panels",
        len(couplers) == N_PANELS - 1,
        details=f"found {len(couplers)} couplers",
    )

    return ctx.report()
