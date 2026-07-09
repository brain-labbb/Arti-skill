from __future__ import annotations

# Cascade / crowd-control pedestrian barriers (galvanized steel).
#
# Reference image: two galvanized "cascade" barriers. Each barrier is a
# rectangular tubular frame filled with vertical pickets, standing on two
# bridge/A feet. Adjacent barriers interlock: one upright end carries downward
# hooks, the neighbour's upright end carries loop eyes. The hooks drop into the
# loops and form a VERTICAL HINGE, so one barrier swings relative to the other
# like a gate. We model two interlocked panels: a fixed left panel (root,
# standing on its feet) and a right panel that pivots about the shared
# hook/loop column (the hinged gate mechanism).

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

# ---------------------------------------------------------------------------
# Real-world dimensions (meters)
# ---------------------------------------------------------------------------
PANEL_LEN = 2.00          # frame length (along X)
FRAME_TOP_Z = 1.10        # top rail center height
FRAME_BOT_Z = 0.20        # bottom rail center height
TUBE_R = 0.020            # outer frame tube radius (~40 mm OD)
PICKET_R = 0.0065         # vertical picket radius (~13 mm)
N_PICKETS = 23            # vertical pickets per panel
UPRIGHT_R = 0.018         # end upright tube radius

FEET_OFFSET = 0.55        # distance of each foot from the panel ends
FOOT_HALF_SPREAD = 0.34   # half the foot leg spread (along Y)
FOOT_TOP_Z = FRAME_BOT_Z  # feet meet the bottom rail
FOOT_TUBE_R = 0.016

# Hook / loop interlock column geometry (the hinge)
HOOK_LOOP_R = 0.008       # wire radius of hooks and loops
LOOP_RADIUS = 0.024       # eyelet ring radius
N_HOOKS = 3               # number of hook/loop pairs up the column
# The loop column sits on the hinge axis (x=0) and its rings reach out to
# +/- LOOP_RADIUS in X. The right panel's frame upright must start clear of the
# rings, so the half-gap exceeds the ring reach plus the upright radius.
HINGE_GAP = 2.0 * (LOOP_RADIUS + UPRIGHT_R + 0.006)

SEG = 14                  # spline samples per segment
RAD = 12                  # tube radial segments


# ---------------------------------------------------------------------------
# CadQuery geometry builders. Each returns a solid in its PART-LOCAL frame.
# A "panel" local frame has X along the panel length, the picket plane in XZ,
# and Y as the (small) tube thickness / foot-spread direction.
# ---------------------------------------------------------------------------
def _straight_tube(p0, p1, radius: float):
    """Capped cylinder between two 3D points."""
    x0, y0, z0 = p0
    x1, y1, z1 = p1
    dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    mid = ((x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0)
    # Direction angles to orient the +Z cylinder onto the segment.
    if length < 1e-9:
        return cq.Workplane("XY").sphere(radius)
    ux, uy, uz = dx / length, dy / length, dz / length
    # Rotation: rotate +Z onto (ux,uy,uz). Use yaw about Z then pitch about Y.
    yaw = math.degrees(math.atan2(uy, ux))
    pitch = math.degrees(math.acos(max(-1.0, min(1.0, uz))))
    cyl = (
        cq.Workplane("XY")
        .cylinder(length, radius)
        .rotate((0, 0, 0), (0, 1, 0), pitch)
        .rotate((0, 0, 0), (0, 0, 1), yaw)
        .translate(mid)
    )
    return cyl


def _frame_and_pickets():
    """Rectangular tubular frame filled with vertical pickets, X-spanning panel."""
    x0 = -PANEL_LEN / 2.0
    x1 = PANEL_LEN / 2.0
    res = None

    def _add(solid):
        nonlocal res
        res = solid if res is None else res.union(solid)

    # Top and bottom rails (along X).
    _add(_straight_tube((x0, 0, FRAME_TOP_Z), (x1, 0, FRAME_TOP_Z), TUBE_R))
    _add(_straight_tube((x0, 0, FRAME_BOT_Z), (x1, 0, FRAME_BOT_Z), TUBE_R))
    # End uprights connecting the rails.
    _add(_straight_tube((x0, 0, FRAME_BOT_Z), (x0, 0, FRAME_TOP_Z), UPRIGHT_R))
    _add(_straight_tube((x1, 0, FRAME_BOT_Z), (x1, 0, FRAME_TOP_Z), UPRIGHT_R))

    # Vertical pickets evenly distributed between the uprights.
    margin = 0.075
    span = (PANEL_LEN - 2.0 * margin)
    for i in range(N_PICKETS):
        t = i / (N_PICKETS - 1)
        px = x0 + margin + t * span
        _add(_straight_tube((px, 0, FRAME_BOT_Z), (px, 0, FRAME_TOP_Z), PICKET_R))

    return res


def _bridge_foot():
    """One bridge/A foot: an inverted-V of tube that meets the bottom rail.

    Local frame: foot apex near Z = FOOT_TOP_Z at y=0, splayed feet on the
    ground. Built from short capped tubes so booleans stay clean. Each ground
    tip gets a small pad for stable, non-floating ground contact.
    """
    apex_z = FOOT_TOP_Z + 0.01
    # Tube tip centrelines sit on top of the ground pads (z = pad top) so the
    # slanted tube rims stay at/above z=0 instead of poking below the ground.
    left_g = (0.0, -FOOT_HALF_SPREAD, 0.012)
    right_g = (0.0, FOOT_HALF_SPREAD, 0.012)
    apex_l = (0.0, -FOOT_HALF_SPREAD * 0.40, apex_z)
    apex_r = (0.0, FOOT_HALF_SPREAD * 0.40, apex_z)
    res = _straight_tube(left_g, apex_l, FOOT_TUBE_R)
    res = res.union(_straight_tube(apex_l, apex_r, FOOT_TUBE_R))
    res = res.union(_straight_tube(apex_r, right_g, FOOT_TUBE_R))
    # Small ground pads under each foot tip.
    pad = cq.Workplane("XY").box(0.06, 0.05, 0.012)
    res = res.union(pad.translate((0.0, -FOOT_HALF_SPREAD, 0.006)))
    res = res.union(pad.translate((0.0, FOOT_HALF_SPREAD, 0.006)))
    return res


def _panel_with_feet():
    """Full standing barrier panel: frame + pickets + two bridge feet."""
    res = _frame_and_pickets()
    foot_x = [-PANEL_LEN / 2.0 + FEET_OFFSET, PANEL_LEN / 2.0 - FEET_OFFSET]
    for fx in foot_x:
        foot = _bridge_foot().translate((fx, 0, 0))
        res = res.union(foot)
    return res


def _loop_column():
    """Vertical end upright carrying loop eyes (the hinge socket side).

    Built in a LOCAL frame centered on the hinge axis (x=0, y=0). The eyes are
    torus rings whose centers sit on the hinge axis so hooks can drop through.
    """
    res = _straight_tube((0, 0, FRAME_BOT_Z - 0.02), (0, 0, FRAME_TOP_Z + 0.02), UPRIGHT_R)
    zs = [FRAME_BOT_Z + 0.12 + i * ((FRAME_TOP_Z - FRAME_BOT_Z - 0.24) / (N_HOOKS - 1))
          for i in range(N_HOOKS)]
    for z in zs:
        res = res.union(_ring(center=(0, 0, z), ring_radius=LOOP_RADIUS,
                              tube_radius=HOOK_LOOP_R))
    # Horizontal tie arms anchor the loop column to the LEFT panel's frame
    # upright (at x = -HINGE_GAP/2) so the column and its eyelet rings are
    # solidly attached to the fixed panel instead of barely grazing it. The
    # arm heights sit midway between the hook/loop levels so the right panel's
    # hooks and connector arms sweep clear of them when the hinge swings.
    frame_upright_x = -HINGE_GAP / 2.0
    for z in ((zs[0] + zs[1]) / 2.0, (zs[1] + zs[2]) / 2.0):
        res = res.union(
            _straight_tube((0.0, 0.0, z), (frame_upright_x, 0.0, z), HOOK_LOOP_R + 0.001)
        )
    return res, zs


def _ring(center, ring_radius: float, tube_radius: float, segments: int = 18):
    """A circular eyelet ring lying in the XZ plane (hole along local Y).

    Built from short capped tube segments so it unions cleanly without a torus
    primitive. The ring is centered on the hinge axis so a hook can drop through.
    """
    cx, cy, cz = center
    pts = []
    for i in range(segments + 1):
        a = 2.0 * math.pi * i / segments
        pts.append((cx + ring_radius * math.cos(a), cy, cz + ring_radius * math.sin(a)))
    res = None
    for p0, p1 in zip(pts[:-1], pts[1:]):
        seg = _straight_tube(p0, p1, tube_radius)
        res = seg if res is None else res.union(seg)
    return res


def _hook_column(zs):
    """Vertical end upright carrying downward hooks that capture the loop eyes.

    Built in a LOCAL frame centered on the hinge axis. Each hook is a small
    C-shaped tube that curls down and around through the loop eye on the axis.
    """
    res = _straight_tube((0, 0, FRAME_BOT_Z - 0.02), (0, 0, FRAME_TOP_Z + 0.02), UPRIGHT_R)

    # Horizontal connector arms bridge the hinge-axis upright (x=0) out to the
    # right panel's frame upright (at x = +HINGE_GAP/2) so the hook column is
    # structurally attached to the panel, not a floating island.
    frame_upright_x = HINGE_GAP / 2.0
    for z in (FRAME_TOP_Z - 0.04, FRAME_BOT_Z + 0.04):
        res = res.union(
            _straight_tube((0.0, 0.0, z), (frame_upright_x, 0.0, z), HOOK_LOOP_R + 0.001)
        )

    for z in zs:
        # Hook curls from the upright, over the top, and down through the eye.
        # Built from short capped tubes in CadQuery for a clean boolean union.
        pts = [
            (UPRIGHT_R, 0.0, z + 0.018),
            (UPRIGHT_R + 0.010, 0.0, z + 0.030),
            (0.004, 0.0, z + 0.034),
            (0.0, 0.0, z + 0.016),
            (0.0, 0.0, z - 0.004),
        ]
        hook_solid = None
        for a, b in zip(pts[:-1], pts[1:]):
            t = _straight_tube(a, b, HOOK_LOOP_R)
            hook_solid = t if hook_solid is None else hook_solid.union(t)
        res = res.union(hook_solid)
    return res


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cascade_barrier_pair")
    galv = model.material("galvanized_steel", rgba=(0.66, 0.68, 0.70, 1.0))
    galv_dark = model.material("galvanized_dark", rgba=(0.55, 0.57, 0.60, 1.0))

    # Loop column z-positions, shared between loop and hook columns.
    _, loop_zs = _loop_column()

    # --- Fixed (root) left panel ---------------------------------------
    # Its right end sits at x = -HINGE_GAP/2; the loop column lives on the
    # hinge axis at world x=0.
    left_panel = model.part("left_panel")
    left_shape = _panel_with_feet().translate((-PANEL_LEN / 2.0 - HINGE_GAP / 2.0, 0, 0))
    left_panel.visual(
        mesh_from_cadquery(left_shape, "left_panel"),
        material=galv,
        name="left_panel_body",
    )
    loop_shape, _ = _loop_column()
    left_panel.visual(
        mesh_from_cadquery(loop_shape, "loop_column"),
        material=galv_dark,
        name="loop_column",
    )

    # --- Hinged (gate) right panel -------------------------------------
    # Authored in a LOCAL frame whose origin is ON the hinge axis: the panel
    # extends along +X from x=0, and the hook column lives at x=0.
    right_panel = model.part("right_panel")
    right_shape = _panel_with_feet().translate((PANEL_LEN / 2.0 + HINGE_GAP / 2.0, 0, 0))
    right_panel.visual(
        mesh_from_cadquery(right_shape, "right_panel"),
        material=galv,
        name="right_panel_body",
    )
    hook_shape = _hook_column(loop_zs)
    right_panel.visual(
        mesh_from_cadquery(hook_shape, "hook_column"),
        material=galv_dark,
        name="hook_column",
    )

    # --- Vertical hinge: hooks dropped into loops -----------------------
    # Hinge axis is vertical (world Z) at world x=0, y=0. The right panel
    # extends along +X from this axis, so positive q swings it toward +Y
    # (opening like a gate around the corner).
    model.articulation(
        "panels_hinge",
        ArticulationType.REVOLUTE,
        parent=left_panel,
        child=right_panel,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0, velocity=1.5, lower=-2.4, upper=2.4
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    left = object_model.get_part("left_panel")
    right = object_model.get_part("right_panel")
    hinge = object_model.get_articulation("panels_hinge")

    # --- Mechanism is a vertical revolute hinge ------------------------
    ctx.check(
        "hinge is revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"got {hinge.articulation_type}",
    )
    ax = hinge.axis
    ctx.check(
        "hinge axis is vertical Z",
        abs(ax[0]) < 1e-6 and abs(ax[1]) < 1e-6 and abs(abs(ax[2]) - 1.0) < 1e-6,
        details=f"axis={ax}",
    )

    # --- Hero parts present: each panel has body + interlock column -----
    ctx.check(
        "left panel has frame body and loop column",
        {v.name for v in left.visuals} >= {"left_panel_body", "loop_column"},
        details=f"{[v.name for v in left.visuals]}",
    )
    ctx.check(
        "right panel has frame body and hook column",
        {v.name for v in right.visuals} >= {"right_panel_body", "hook_column"},
        details=f"{[v.name for v in right.visuals]}",
    )

    # --- Interlock columns meet at the hinge axis (not floating apart) --
    # In the closed pose the hook and loop columns overlap vertically and
    # nearly touch in plan around the hinge axis at x=0.
    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            left,
            right,
            axes="z",
            elem_a="loop_column",
            elem_b="hook_column",
            min_overlap=0.40,
            name="loop and hook columns overlap vertically at the hinge",
        )
        ctx.expect_contact(
            left,
            right,
            elem_a="loop_column",
            elem_b="hook_column",
            contact_tol=0.02,
            name="hooks seat in the loop eyes",
        )
        # Right panel extends to +X of the hinge axis (panel, not a stub).
        right_aabb = ctx.part_world_aabb(right)
        ctx.check(
            "right panel spans away from hinge along +X",
            right_aabb is not None and right_aabb[1][0] > 1.5,
            details=f"right_aabb={right_aabb}",
        )

    # --- The hinge actually swings the gate panel ----------------------
    closed_pos = None
    opened_pos = None
    with ctx.pose({hinge: 0.0}):
        closed_aabb = ctx.part_world_aabb(right)
        # Free far end of the gate panel in the closed pose.
        closed_pos = (closed_aabb[1][0], closed_aabb[1][1])
    with ctx.pose({hinge: 1.2}):
        opened_aabb = ctx.part_world_aabb(right)
        opened_pos = (opened_aabb[1][0], opened_aabb[1][1])
    # Swinging about +Z by +1.2 rad pulls the far end toward +Y and back in X.
    ctx.check(
        "opening the hinge swings the gate toward +Y",
        opened_pos is not None and closed_pos is not None
        and opened_pos[1] > closed_pos[1] + 0.5,
        details=f"closed={closed_pos}, opened={opened_pos}",
    )
    ctx.check(
        "opening the hinge reduces the gate +X reach",
        opened_pos is not None and closed_pos is not None
        and opened_pos[0] < closed_pos[0] - 0.3,
        details=f"closed={closed_pos}, opened={opened_pos}",
    )

    # The hook/loop interlock is an intentional capture fit (small nesting).
    ctx.allow_overlap(
        left,
        right,
        elem_a="loop_column",
        elem_b="hook_column",
        reason="Hooks are intentionally captured inside the loop eyes to form the hinge.",
    )

    return ctx.report()


object_model = build_object_model()
