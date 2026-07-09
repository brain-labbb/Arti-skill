from __future__ import annotations

# Industrial yellow gas/oil pipeline section with an inline gate valve.
#
# The reference image shows a glossy yellow pipe running horizontally with a
# 90-degree elbow that turns downward on the left end. Two smooth socket/bell
# coupling collars bracket a central gate-valve body. A steel bonnet rises from the valve body,
# carrying a packing/gland nut, a rising threaded stem, and a chrome three-spoke
# handwheel on top.
#
# Articulated mechanism: the HANDWHEEL is the operator control. It spins
# continuously about the vertical valve stem (right-hand rule about +Z). In a
# real rising-stem gate valve this rotation drives the gate up/down; here we
# articulate the directly-visible rotary control (the handwheel) about the stem.

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

SOCKET_R = 0.073         # smooth bell coupling outer radius
SOCKET_LEN = 0.074       # full length of one push-fit/socket-weld collar
SOCKET_SHOULDER = 0.016  # tapered bell-mouth length at each end

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
WHEEL_RIM_R = 0.082      # handwheel outer radius
WHEEL_TUBE_R = 0.0085    # torus tube radius of the rim
SPOKE_R = 0.0055
N_SPOKES = 3

# World layout: pipe centerline along +X at height Z0. The elbow is on the
# left (low X) and turns down. The handwheel sits on top at the valve center.
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


def _socket_collar(x_center: float) -> cq.Workplane:
    """Plain socket-weld / push-fit bell collar at a pipe-to-valve joint.

    The collar is a single smooth annular sleeve with tapered bell mouths. It
    intentionally has no flange disks, no split flange pair, and no bolt ring.
    """
    x0 = x_center - SOCKET_LEN / 2.0
    sleeve_len = SOCKET_LEN - 2.0 * SOCKET_SHOULDER
    plane = cq.Plane(origin=(x0, 0.0, Z0), normal=(1, 0, 0))

    outer = (
        cq.Workplane(plane)
        .circle(PIPE_OR + 0.002)
        .workplane(offset=SOCKET_SHOULDER)
        .circle(SOCKET_R)
        .workplane(offset=sleeve_len)
        .circle(SOCKET_R)
        .workplane(offset=SOCKET_SHOULDER)
        .circle(PIPE_OR + 0.002)
        .loft(combine=True)
    )
    bore = (
        cq.Workplane(plane)
        .circle(PIPE_IR)
        .extrude(SOCKET_LEN + 0.004)
        .translate((-0.002, 0.0, 0.0))
    )
    return outer.cut(bore)


def _socket_joint_centers() -> tuple[float, float]:
    """Locations of the two pipe-to-valve coupling joints."""
    x_in = X_ELBOW + ELBOW_BEND_R + RUN_RIGHT_LEN
    x_out = x_in + VALVE_RUN_LEN
    return (x_in, x_out)


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
    """Rising threaded valve stem, vertical along +Z up into the handwheel hub."""
    z_base = Z0 + PIPE_OR + BONNET_H + GLAND_H - 0.006
    stem = (
        cq.Workplane(cq.Plane(origin=(X_VALVE_CENTER, 0.0, z_base), normal=(0, 0, 1)))
        .circle(STEM_R)
        .extrude(STEM_LEN)
    )
    return stem


def _handwheel() -> cq.Workplane:
    """Chrome three-spoke handwheel built around its own center at local origin.

    Local frame: wheel lies in the XY plane, spin axis = +Z, hub at origin.
    """
    # Outer rim: a torus.
    rim = (
        cq.Workplane("XY")
        .add(
            cq.Solid.makeTorus(
                WHEEL_RIM_R - WHEEL_TUBE_R,
                WHEEL_TUBE_R,
            )
        )
    )
    wheel = rim
    # Central hub (short collar around the stem axis).
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HUB_H, both=True)
    wheel = wheel.union(hub)
    # Three spokes from hub to rim.
    for i in range(N_SPOKES):
        ang = 2.0 * math.pi * i / N_SPOKES
        # spoke is a cylinder lying in the wheel plane, pointing outward at angle.
        spoke = (
            cq.Workplane("XY")
            .center(0.0, 0.0)
            .circle(SPOKE_R)
            .extrude(WHEEL_RIM_R - WHEEL_TUBE_R)
        )
        # The default extrude is along +Z; rotate so the spoke lies radially in XY.
        spoke = spoke.rotate((0, 0, 0), (0, 1, 0), 90.0)  # now along +X
        spoke = spoke.rotate((0, 0, 0), (0, 0, 1), math.degrees(ang))
        wheel = wheel.union(spoke)
    return wheel


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gate_valve_pipeline")

    model.material("pipe_yellow", rgba=(0.96, 0.78, 0.05, 1.0))
    model.material("joint_yellow", rgba=(0.90, 0.68, 0.03, 1.0))
    model.material("valve_steel", rgba=(0.52, 0.54, 0.57, 1.0))
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
    for i, x_joint in enumerate(_socket_joint_centers()):
        body.visual(
            mesh_from_cadquery(
                _socket_collar(x_joint),
                f"socket_collar_{i}",
                tolerance=TOL,
                angular_tolerance=ATOL,
            ),
            material="joint_yellow",
            name=f"socket_collar_{i}",
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

    # ---- HANDWHEEL: the rotary operator control ----
    wheel = model.part("handwheel")
    wheel.visual(
        mesh_from_cadquery(_handwheel(), "handwheel", tolerance=TOL, angular_tolerance=ATOL),
        material="chrome",
        name="handwheel",
    )

    # Handwheel spins about the vertical stem. Its local frame has the hub at the
    # origin and the spin axis along +Z, so we place the joint at the stem top.
    z_wheel = Z0 + PIPE_OR + BONNET_H + GLAND_H + STEM_LEN - 0.014
    model.articulation(
        "valve_handwheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wheel,
        origin=Origin(xyz=(X_VALVE_CENTER, 0.0, z_wheel)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("pipeline_body")
    wheel = object_model.get_part("handwheel")
    joint = object_model.get_articulation("valve_handwheel")

    # --- Joint is the rotary handwheel control about the vertical stem ---
    ctx.check(
        "handwheel joint is continuous",
        joint.joint_type == "continuous",
        details=f"joint_type={joint.joint_type}",
    )
    ax = tuple(round(c, 6) for c in joint.axis)
    ctx.check(
        "handwheel spin axis is vertical (+Z)",
        ax == (0.0, 0.0, 1.0),
        details=f"axis={ax}",
    )

    # --- Handwheel sits on top, above the bonnet/valve, centered over the stem ---
    wheel_aabb = ctx.part_world_aabb(wheel)
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "handwheel is the highest assembly (on top of the valve)",
        wheel_aabb is not None
        and body_aabb is not None
        and wheel_aabb[1][2] >= body_aabb[1][2] - 0.001,
        details=f"wheel_top={wheel_aabb[1][2]:.3f}, body_top={body_aabb[1][2]:.3f}",
    )
    wheel_pos = ctx.part_world_position(wheel)
    ctx.check(
        "handwheel centered over the valve stem",
        wheel_pos is not None
        and abs(wheel_pos[0] - X_VALVE_CENTER) < 0.02
        and abs(wheel_pos[1]) < 0.02,
        details=f"wheel_pos={wheel_pos}, stem_x={X_VALVE_CENTER:.3f}",
    )

    # --- Rotating the handwheel spins the rim but keeps the hub center fixed ---
    rim_rest = ctx.part_element_world_aabb(wheel, elem="handwheel")
    hub_rest = ctx.part_world_position(wheel)
    with ctx.pose({joint: math.pi / 2.0}):
        hub_turned = ctx.part_world_position(wheel)
    ctx.check(
        "turning the handwheel keeps the hub center on the stem axis",
        hub_rest is not None
        and hub_turned is not None
        and abs(hub_turned[0] - hub_rest[0]) < 1e-4
        and abs(hub_turned[1] - hub_rest[1]) < 1e-4
        and abs(hub_turned[2] - hub_rest[2]) < 1e-4,
        details=f"rest={hub_rest}, turned={hub_turned}",
    )
    ctx.check(
        "handwheel rim has real radial extent (it is a spoked wheel, not a disk)",
        rim_rest is not None
        and (rim_rest[1][0] - rim_rest[0][0]) > 2.0 * (WHEEL_RIM_R - 0.01)
        and (rim_rest[1][1] - rim_rest[0][1]) > 2.0 * (WHEEL_RIM_R - 0.01),
        details=f"rim_aabb={rim_rest}",
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

    # --- Bonnet rises between the pipe and the handwheel (real supporting stack) ---
    bonnet_aabb = ctx.part_element_world_aabb(body, elem="bonnet")
    stem_aabb = ctx.part_element_world_aabb(body, elem="valve_stem")
    ctx.check(
        "bonnet + stem form the support stack reaching up toward the handwheel",
        bonnet_aabb is not None
        and stem_aabb is not None
        and stem_aabb[1][2] > bonnet_aabb[1][2]
        and stem_aabb[1][2] >= wheel_aabb[0][2] - 0.005,
        details=f"bonnet_top={bonnet_aabb[1][2]:.3f}, stem_top={stem_aabb[1][2]:.3f}, "
        f"wheel_bottom={wheel_aabb[0][2]:.3f}",
    )

    # --- Requested fork: plain socket/bell collars, no bolted flange pairs ---
    visual_names = tuple(v.name for v in body.visuals)
    ctx.check(
        "bolted flange and bolt-ring visuals have been removed",
        "flanges" not in visual_names
        and all(("flange" not in (name or "")) and ("bolt" not in (name or "")) for name in visual_names),
        details=f"body_visuals={visual_names}",
    )
    collar_aabbs = [
        ctx.part_element_world_aabb(body, elem=f"socket_collar_{i}")
        for i in range(2)
    ]
    ctx.check(
        "two smooth socket collars replace the original flange pairs",
        all(aabb is not None for aabb in collar_aabbs),
        details=f"collar_aabbs={collar_aabbs}",
    )
    centers = _socket_joint_centers()
    for i, (aabb, expected_x) in enumerate(zip(collar_aabbs, centers)):
        if aabb is None:
            continue
        dx = aabb[1][0] - aabb[0][0]
        dy = aabb[1][1] - aabb[0][1]
        dz = aabb[1][2] - aabb[0][2]
        cx = (aabb[0][0] + aabb[1][0]) / 2.0
        ctx.check(
            f"socket_collar_{i} is a modest bell sleeve, not a flange disk",
            0.060 <= dx <= 0.090
            and 2.0 * PIPE_OR + 0.015 <= dy <= 2.0 * SOCKET_R + 0.010
            and 2.0 * PIPE_OR + 0.015 <= dz <= 2.0 * SOCKET_R + 0.010
            and abs(cx - expected_x) < 0.005,
            details=f"aabb={aabb}, dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}, cx={cx:.3f}, expected_x={expected_x:.3f}",
        )
        ctx.expect_overlap(
            body, body,
            axes="x",
            elem_a=f"socket_collar_{i}",
            elem_b="pipe_yellow",
            min_overlap=0.055,
            name=f"socket_collar_{i} is seated over the pipe run",
        )

    # The handwheel hub is intentionally seated over the captured rising stem.
    ctx.allow_overlap(
        wheel,
        body,
        elem_a="handwheel",
        elem_b="valve_stem",
        reason="The handwheel hub is seated over the captured rising valve stem (nested shaft fit).",
    )
    ctx.expect_overlap(
        wheel, body, axes="xy",
        elem_a="handwheel", elem_b="valve_stem",
        min_overlap=2.0 * STEM_R - 0.002,
        name="handwheel hub engages the valve stem",
    )

    return ctx.report()


object_model = build_object_model()
