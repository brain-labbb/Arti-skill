from __future__ import annotations

# Industrial yellow gas/oil pipeline section with an inline gate valve.
#
# The reference image shows a glossy yellow pipe running horizontally with a
# 90-degree elbow that turns downward on the left end. Two bolted flange joints
# bracket a central gate-valve body. A steel bonnet rises from the valve body,
# carrying a packing/gland nut, a rising threaded stem, and a chrome crossed
# tee-bar handle on top.
#
# Articulated mechanism: the CROSS HANDLE is the operator control. It spins
# continuously about the vertical valve stem (right-hand rule about +Z). In a
# real rising-stem gate valve this rotation drives the gate up/down; here we
# articulate the directly-visible rotary control (the cross handle) about the stem.

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

TOL = 0.0012
ATOL = 0.2

# ---------------------------------------------------------------------------
# Real-world dimensions (meters)
# ---------------------------------------------------------------------------
PIPE_OR = 0.060          # pipe outer radius (~120 mm OD line)
PIPE_WALL = 0.010        # pipe wall thickness
PIPE_IR = PIPE_OR - PIPE_WALL

ELBOW_BEND_R = 0.110     # elbow centerline bend radius
RUN_RIGHT_LEN = 0.150    # horizontal run from elbow to first flange
VALVE_RUN_LEN = 0.330    # length of the central valve-body region
RUN_END_LEN = 0.150      # horizontal run after the valve to the open end
DROP_LEN = 0.230         # vertical drop of the elbow leg

FLANGE_R = 0.085         # bolted flange outer radius
FLANGE_T = 0.013         # flange half-thickness (disk spans +-FLANGE_T)
BOLT_CIRCLE_R = 0.066    # bolt circle radius
BOLT_HEAD_R = 0.0075
BOLT_HEAD_H = 0.006
N_BOLTS = 12

VALVE_BODY_R = 0.078     # central valve body bulge radius
VALVE_BODY_LEN = 0.150

BONNET_BASE_R = 0.058
BONNET_TOP_R = 0.034
BONNET_H = 0.072         # bonnet rises above the pipe top
GLAND_R = 0.030
GLAND_H = 0.026

STEM_R = 0.0085
STEM_LEN = 0.090         # exposed stem from bonnet top up into hub

HUB_R = 0.022
HUB_H = 0.026
HANDLE_BAR_LEN = 0.150   # straight bar length between the rounded end knobs
HANDLE_BAR_R = 0.0065
HANDLE_END_R = 0.010
HANDLE_BAR_Z = 0.016     # bars pass through the upper half of the hub

# World layout: pipe centerline along +X at height Z0. The elbow is on the
# left (low X) and turns down. The cross handle sits on top at the valve center.
Z0 = DROP_LEN + PIPE_OR + 0.02       # centerline height above ground
X_ELBOW = 0.0                        # x of vertical leg centerline
X_VALVE_CENTER = X_ELBOW + ELBOW_BEND_R + RUN_RIGHT_LEN + VALVE_RUN_LEN / 2.0


# ---------------------------------------------------------------------------
# Geometry builders (CadQuery)
# ---------------------------------------------------------------------------
def _pipe_tube(length: float) -> cq.Workplane:
    """A hollow straight pipe segment of given length, axis along +X, starting at x=0."""
    outer = cq.Workplane("YZ").circle(PIPE_OR).extrude(length)
    inner = cq.Workplane("YZ").circle(PIPE_IR).extrude(length)
    return outer.cut(inner)


def _pipeline_run() -> cq.Workplane:
    """Yellow pipe: straight vertical drop leg + 90-deg elbow + long horizontal
    run, all hollow.

    Built from clean primitives (two cylinders + a quarter-torus elbow) instead
    of a swept spline, so the horizontal run stays perfectly straight like the
    reference image rather than oscillating.
    """
    z_bend_start = Z0 - ELBOW_BEND_R
    z_bottom = z_bend_start - DROP_LEN
    cx = X_ELBOW + ELBOW_BEND_R          # bend center x == horizontal leg start
    cz = Z0 - ELBOW_BEND_R               # bend center z
    x_end = X_ELBOW + ELBOW_BEND_R + RUN_RIGHT_LEN + VALVE_RUN_LEN + RUN_END_LEN

    eps = 0.012  # small overlap so unions stay continuous at the joints

    def _cyl(r: float, origin, normal, length: float) -> cq.Workplane:
        return cq.Workplane(cq.Plane(origin=origin, normal=normal)).circle(r).extrude(length)

    def _tube_cyl(origin, normal, length: float) -> cq.Workplane:
        return (
            cq.Workplane(cq.Plane(origin=origin, normal=normal))
            .circle(PIPE_OR)
            .circle(PIPE_IR)
            .extrude(length)
        )

    def _quarter_torus(minor: float) -> cq.Workplane:
        """Upper-left quadrant of a torus: connects the vertical leg to the run."""
        full = cq.Workplane().add(
            cq.Solid.makeTorus(
                ELBOW_BEND_R, minor,
                pnt=cq.Vector(cx, 0.0, cz), dir=cq.Vector(0, 1, 0),
            )
        )
        bx_lo, bx_hi = cx - ELBOW_BEND_R - PIPE_OR - 0.01, cx + 0.01
        bz_lo, bz_hi = cz - 0.01, cz + ELBOW_BEND_R + PIPE_OR + 0.01
        box = cq.Workplane(
            cq.Plane(origin=((bx_lo + bx_hi) / 2.0, 0.0, (bz_lo + bz_hi) / 2.0))
        ).box(bx_hi - bx_lo, 4.0 * PIPE_OR, bz_hi - bz_lo)
        return full.intersect(box)

    # Straight legs are authored as true annular extrusions, not solid cylinders
    # post-cut by a bore. This keeps exported open ends from triangulating as
    # yellow cap disks in the viewer.
    v_shell = _tube_cyl((X_ELBOW, 0.0, z_bottom), (0, 0, 1), DROP_LEN + eps)
    h_shell = _tube_cyl((cx - eps, 0.0, Z0), (1, 0, 0), (x_end - cx) + eps)
    # Quarter-torus elbow, also hollow.
    e_out = _quarter_torus(PIPE_OR)
    e_in = _quarter_torus(PIPE_IR)
    e_shell = e_out.cut(e_in)

    return v_shell.union(e_shell).union(h_shell)


def _flange(x_center: float) -> cq.Workplane:
    """Bolted raised-face flange (annular steel collar) at the given x."""
    plane = cq.Plane(origin=(x_center, 0.0, Z0), normal=(1, 0, 0))
    disk = cq.Workplane(plane).circle(FLANGE_R).extrude(FLANGE_T, both=True)
    bore = cq.Workplane(plane).circle(PIPE_IR).extrude(FLANGE_T + 0.004, both=True)
    flange = disk.cut(bore)
    # Bolt heads spaced around the bolt circle, protruding on both faces.
    for i in range(N_BOLTS):
        ang = 2.0 * math.pi * i / N_BOLTS
        by = BOLT_CIRCLE_R * math.cos(ang)
        bz = BOLT_CIRCLE_R * math.sin(ang)
        bolt = (
            cq.Workplane(cq.Plane(origin=(x_center, by, Z0 + bz), normal=(1, 0, 0)))
            .polygon(6, BOLT_HEAD_R * 2.0)
            .extrude(FLANGE_T + BOLT_HEAD_H, both=True)
        )
        flange = flange.union(bolt)
    return flange


def _flange_pair() -> cq.Workplane:
    """All four flanges: a bolted pair at the inlet joint and at the outlet joint.

    Each joint is two flange disks bolted face-to-face around the pipe; the span
    between the joints stays bare yellow pipe (plus the yellow valve body),
    matching the reference rather than wrapping it in a steel sleeve.
    """
    x_in = X_ELBOW + ELBOW_BEND_R + RUN_RIGHT_LEN
    x_out = x_in + VALVE_RUN_LEN
    f = _flange(x_in - FLANGE_T - 0.001)
    f = f.union(_flange(x_in + FLANGE_T + 0.001))
    f = f.union(_flange(x_out - FLANGE_T - 0.001))
    f = f.union(_flange(x_out + FLANGE_T + 0.001))
    return f


def _valve_body() -> cq.Workplane:
    """Central cast valve body: a fatter barrel with the pipe bore cut through."""
    plane = cq.Plane(origin=(X_VALVE_CENTER, 0.0, Z0), normal=(1, 0, 0))
    body = cq.Workplane(plane).circle(VALVE_BODY_R).extrude(VALVE_BODY_LEN / 2.0, both=True)
    bore = cq.Workplane(plane).circle(PIPE_IR).extrude(VALVE_BODY_LEN / 2.0 + 0.004, both=True)
    return body.cut(bore)


def _bonnet_stack() -> cq.Workplane:
    """Steel bonnet neck, packing gland nut, and gland flange above the valve."""
    z_top = Z0 + PIPE_OR  # top of the valve body roughly
    # Tapered bonnet neck rising from the valve body top.
    bonnet = (
        cq.Workplane("XY")
        .workplane(offset=z_top)
        .circle(BONNET_BASE_R)
        .workplane(offset=BONNET_H)
        .circle(BONNET_TOP_R)
        .loft(combine=True)
        .translate((X_VALVE_CENTER, 0.0, 0.0))
    )
    # Hexagonal gland/packing nut on top of the bonnet.
    gland = (
        cq.Workplane(cq.Plane(origin=(X_VALVE_CENTER, 0.0, z_top + BONNET_H), normal=(0, 0, 1)))
        .polygon(6, GLAND_R * 2.0)
        .extrude(GLAND_H)
    )
    return bonnet.union(gland)


def _stem() -> cq.Workplane:
    """Rising threaded valve stem, vertical along +Z up to the cross-handle hub."""
    z_base = Z0 + PIPE_OR + BONNET_H + GLAND_H - 0.006
    stem = (
        cq.Workplane(cq.Plane(origin=(X_VALVE_CENTER, 0.0, z_base), normal=(0, 0, 1)))
        .circle(STEM_R)
        .extrude(STEM_LEN)
    )
    return stem


def _handle_hub() -> cq.Workplane:
    """Solid chrome collar for the cross handle.

    Local frame: the part origin is the seating/contact point on top of the
    visible stem. The hub rises along +Z from that contact surface so the joint
    origin is on a real parent face rather than hidden at the center of the part.
    """
    return cq.Workplane("XY").circle(HUB_R).extrude(HUB_H)


def _handle_bar(index: int) -> cq.Workplane:
    """One straight chrome tee-bar with rounded grip ends.

    Two instances, rotated 90 degrees apart, pass through the hub to make the
    requested crossed tee-bar/cross handle instead of the parent three-spoke rim.
    """
    angle = 90.0 * index
    bar = (
        cq.Workplane(cq.Plane(origin=(-HANDLE_BAR_LEN / 2.0, 0.0, HANDLE_BAR_Z), normal=(1, 0, 0)))
        .circle(HANDLE_BAR_R)
        .extrude(HANDLE_BAR_LEN)
    )
    for x in (-HANDLE_BAR_LEN / 2.0, HANDLE_BAR_LEN / 2.0):
        cap = cq.Workplane("XY").sphere(HANDLE_END_R).translate((x, 0.0, HANDLE_BAR_Z))
        bar = bar.union(cap)
    return bar.rotate((0, 0, 0), (0, 0, 1), angle)


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gate_valve_pipeline")

    model.material("pipe_yellow", rgba=(0.96, 0.78, 0.05, 1.0))
    model.material("valve_steel", rgba=(0.52, 0.54, 0.57, 1.0))
    model.material("bolt_steel", rgba=(0.40, 0.41, 0.43, 1.0))
    model.material("chrome", rgba=(0.78, 0.80, 0.83, 1.0))

    # ---- ROOT: the whole stationary pipeline + valve body + bonnet + stem ----
    body = model.part("pipeline_body")
    body.visual(
        mesh_from_cadquery(_pipeline_run(), "pipeline_run", tolerance=TOL, angular_tolerance=ATOL),
        material="pipe_yellow",
        name="pipe_yellow",
    )
    body.visual(
        mesh_from_cadquery(_valve_body(), "valve_body", tolerance=TOL, angular_tolerance=ATOL),
        material="pipe_yellow",
        name="valve_body",
    )
    body.visual(
        mesh_from_cadquery(_flange_pair(), "flanges", tolerance=TOL, angular_tolerance=ATOL),
        material="bolt_steel",
        name="flanges",
    )
    body.visual(
        mesh_from_cadquery(_bonnet_stack(), "bonnet_stack", tolerance=TOL, angular_tolerance=ATOL),
        material="valve_steel",
        name="bonnet",
    )
    body.visual(
        mesh_from_cadquery(_stem(), "valve_stem", tolerance=TOL, angular_tolerance=ATOL),
        material="chrome",
        name="valve_stem",
    )

    # ---- CROSS HANDLE: the rotary operator control ----
    handle = model.part("cross_handle")
    handle.visual(
        mesh_from_cadquery(_handle_hub(), "handle_hub", tolerance=TOL, angular_tolerance=ATOL),
        material="chrome",
        name="handle_hub",
    )
    for i in range(2):
        handle.visual(
            mesh_from_cadquery(_handle_bar(i), f"handle_bar_{i}", tolerance=TOL, angular_tolerance=ATOL),
            material="chrome",
            name=f"handle_bar_{i}",
        )

    # The cross handle spins about the vertical stem. Its local part frame is
    # the lower hub contact face, so the continuous joint origin sits on the
    # visible top face of the valve stem.
    z_stem_top = Z0 + PIPE_OR + BONNET_H + GLAND_H - 0.006 + STEM_LEN
    model.articulation(
        "valve_handle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=handle,
        origin=Origin(xyz=(X_VALVE_CENTER, 0.0, z_stem_top)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("pipeline_body")
    handle = object_model.get_part("cross_handle")
    joint = object_model.get_articulation("valve_handle")

    # --- Joint is the rotary cross-handle control about the vertical stem ---
    ctx.check(
        "cross handle joint is continuous",
        joint.joint_type == "continuous",
        details=f"joint_type={joint.joint_type}",
    )
    ax = tuple(round(c, 6) for c in joint.axis)
    ctx.check(
        "cross handle spin axis is vertical (+Z)",
        ax == (0.0, 0.0, 1.0),
        details=f"axis={ax}",
    )

    # --- Cross handle sits on top, above the bonnet/valve, centered over the stem ---
    handle_aabb = ctx.part_world_aabb(handle)
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "cross handle is the highest assembly (on top of the valve)",
        handle_aabb is not None
        and body_aabb is not None
        and handle_aabb[1][2] >= body_aabb[1][2] - 0.001,
        details=f"handle_top={handle_aabb[1][2]:.3f}, body_top={body_aabb[1][2]:.3f}",
    )
    handle_pos = ctx.part_world_position(handle)
    ctx.check(
        "cross handle centered over the valve stem",
        handle_pos is not None
        and abs(handle_pos[0] - X_VALVE_CENTER) < 0.02
        and abs(handle_pos[1]) < 0.02,
        details=f"handle_pos={handle_pos}, stem_x={X_VALVE_CENTER:.3f}",
    )

    # --- The changed operator is a crossed tee-bar: two straight bars through the hub ---
    visual_names = {visual.name for visual in handle.visuals}
    ctx.check(
        "cross handle has hub plus exactly two bar visuals",
        visual_names == {"handle_hub", "handle_bar_0", "handle_bar_1"},
        details=f"visual_names={sorted(visual_names)}",
    )
    bar_0 = ctx.part_element_world_aabb(handle, elem="handle_bar_0")
    bar_1 = ctx.part_element_world_aabb(handle, elem="handle_bar_1")
    hub_aabb = ctx.part_element_world_aabb(handle, elem="handle_hub")
    ctx.check(
        "cross handle bars are long straight perpendicular members",
        bar_0 is not None
        and bar_1 is not None
        and (bar_0[1][0] - bar_0[0][0]) > HANDLE_BAR_LEN - 0.004
        and (bar_0[1][1] - bar_0[0][1]) < 2.0 * HANDLE_END_R + 0.006
        and (bar_1[1][1] - bar_1[0][1]) > HANDLE_BAR_LEN - 0.004
        and (bar_1[1][0] - bar_1[0][0]) < 2.0 * HANDLE_END_R + 0.006,
        details=f"bar_0={bar_0}, bar_1={bar_1}",
    )
    ctx.check(
        "cross handle replaces the parent three-spoke rim",
        "handwheel" not in visual_names
        and handle_aabb is not None
        and hub_aabb is not None
        and (handle_aabb[1][0] - handle_aabb[0][0]) > 2.0 * HUB_R
        and (handle_aabb[1][1] - handle_aabb[0][1]) > 2.0 * HUB_R,
        details=f"handle_aabb={handle_aabb}, hub_aabb={hub_aabb}, visual_names={visual_names}",
    )

    # --- Rotating the handle keeps the hub center fixed on the stem axis ---
    hub_rest = ctx.part_world_position(handle)
    with ctx.pose({joint: math.pi / 2.0}):
        hub_turned = ctx.part_world_position(handle)
    ctx.check(
        "turning the cross handle keeps the hub center on the stem axis",
        hub_rest is not None
        and hub_turned is not None
        and abs(hub_turned[0] - hub_rest[0]) < 1e-4
        and abs(hub_turned[1] - hub_rest[1]) < 1e-4
        and abs(hub_turned[2] - hub_rest[2]) < 1e-4,
        details=f"rest={hub_rest}, turned={hub_turned}",
    )

    # --- Pipe is hollow at the open right end (gas line, not a solid rod) ---
    pipe_aabb = ctx.part_element_world_aabb(body, elem="pipe_yellow")
    ctx.check(
        "yellow pipe spans the full horizontal run with the elbow drop",
        pipe_aabb is not None
        and (pipe_aabb[1][0] - pipe_aabb[0][0]) > 0.55
        and (pipe_aabb[1][2] - pipe_aabb[0][2]) > DROP_LEN,
        details=f"pipe_aabb={pipe_aabb}",
    )

    # --- Bonnet rises between the pipe and the cross handle (real supporting stack) ---
    bonnet_aabb = ctx.part_element_world_aabb(body, elem="bonnet")
    stem_aabb = ctx.part_element_world_aabb(body, elem="valve_stem")
    ctx.check(
        "bonnet + stem form the support stack reaching up toward the cross handle",
        bonnet_aabb is not None
        and stem_aabb is not None
        and stem_aabb[1][2] > bonnet_aabb[1][2]
        and stem_aabb[1][2] >= handle_aabb[0][2] - 0.005,
        details=f"bonnet_top={bonnet_aabb[1][2]:.3f}, stem_top={stem_aabb[1][2]:.3f}, "
        f"handle_bottom={handle_aabb[0][2]:.3f}",
    )

    # --- Flanges present and seated around the valve body ---
    flange_aabb = ctx.part_element_world_aabb(body, elem="flanges")
    ctx.check(
        "bolted flanges present and taller/wider than the bare pipe",
        flange_aabb is not None
        and (flange_aabb[1][1] - flange_aabb[0][1]) > 2.0 * PIPE_OR + 0.02,
        details=f"flange_aabb={flange_aabb}",
    )

    # The cross-handle hub is seated on the visible top of the rising stem.
    ctx.expect_contact(
        handle, body,
        elem_a="handle_hub", elem_b="valve_stem",
        contact_tol=0.002,
        name="cross handle hub seats on the valve stem top face",
    )
    ctx.expect_overlap(
        handle, body, axes="xy",
        elem_a="handle_hub", elem_b="valve_stem",
        min_overlap=2.0 * STEM_R - 0.002,
        name="cross handle hub is centered over the valve stem",
    )

    return ctx.report()


object_model = build_object_model()
