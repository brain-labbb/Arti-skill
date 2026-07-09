from __future__ import annotations

# Industrial yellow gas/oil pipeline section with an L-shaped angle valve.
#
# The reference image shows a glossy yellow pipe running horizontally with a
# 90-degree valve casting: one flanged leg points downward and the other points
# horizontally. A steel bonnet rises directly from the elbow corner, carrying a
# packing/gland nut, a rising threaded stem, and a chrome three-spoke handwheel
# on top.
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
RUN_RIGHT_LEN = 0.150    # horizontal run from elbow corner to outlet flange
RUN_END_LEN = 0.150      # horizontal run after the outlet flange to the open end
DROP_LEN = 0.230         # vertical drop from bend tangent to inlet open end
INLET_FLANGE_DROP = 0.105  # vertical run from bend tangent down to inlet flange

FLANGE_R = 0.085         # bolted flange outer radius
FLANGE_T = 0.013         # flange half-thickness (disk spans +-FLANGE_T)
BOLT_CIRCLE_R = 0.066    # bolt circle radius
BOLT_HEAD_R = 0.0075
BOLT_HEAD_H = 0.006
N_BOLTS = 12

VALVE_BODY_R = 0.078     # central valve body bulge radius
VALVE_BODY_LEN = 0.150   # retained as the casting socket length scale

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

# World layout: vertical inlet leg rises at X_ELBOW and turns 90 degrees into a
# horizontal outlet leg at height Z0. The bonnet and handwheel are centered over
# the elbow outlet/corner, not over an inline body.
Z0 = DROP_LEN + ELBOW_BEND_R + PIPE_OR + 0.02  # outlet centerline height
X_ELBOW = 0.0                                  # x of vertical inlet centerline
X_VALVE_CENTER = X_ELBOW + ELBOW_BEND_R        # bonnet over the angle corner
Z_VERTICAL_TANGENT = Z0 - ELBOW_BEND_R
INLET_FLANGE_Z = Z_VERTICAL_TANGENT - INLET_FLANGE_DROP
OUTLET_FLANGE_X = X_VALVE_CENTER + RUN_RIGHT_LEN


# ---------------------------------------------------------------------------
# Geometry builders (CadQuery)
# ---------------------------------------------------------------------------
def _pipe_tube(length: float) -> cq.Workplane:
    """A hollow straight pipe segment of given length, axis along +X, starting at x=0."""
    outer = cq.Workplane("YZ").circle(PIPE_OR).extrude(length)
    inner = cq.Workplane("YZ").circle(PIPE_IR).extrude(length)
    return outer.cut(inner)


def _angle_quarter_torus(minor: float) -> cq.Workplane:
    """Upper-left quadrant of a torus: vertical inlet to horizontal outlet."""
    cx = X_ELBOW + ELBOW_BEND_R
    cz = Z0 - ELBOW_BEND_R
    full = cq.Workplane().add(
        cq.Solid.makeTorus(
            ELBOW_BEND_R, minor,
            pnt=cq.Vector(cx, 0.0, cz), dir=cq.Vector(0, 1, 0),
        )
    )
    # Keep only the upper-left elbow quadrant, with a slight extension past the
    # two tangent planes so the straight socket tubes have positive overlap.
    blend_eps = 0.014
    bx_lo, bx_hi = cx - ELBOW_BEND_R - minor - 0.01, cx + blend_eps
    bz_lo, bz_hi = cz - blend_eps, cz + ELBOW_BEND_R + minor + 0.01
    box = cq.Workplane(
        cq.Plane(origin=((bx_lo + bx_hi) / 2.0, 0.0, (bz_lo + bz_hi) / 2.0))
    ).box(bx_hi - bx_lo, 4.0 * max(PIPE_OR, VALVE_BODY_R), bz_hi - bz_lo)
    return full.intersect(box)


def _annular_cylinder(origin, normal, length: float, outer_r: float, inner_r: float) -> cq.Workplane:
    """Open-ended annular cylinder on an arbitrary axis."""
    return (
        cq.Workplane(cq.Plane(origin=origin, normal=normal))
        .circle(outer_r)
        .circle(inner_r)
        .extrude(length)
    )


def _pipeline_run() -> cq.Workplane:
    """Yellow pipe: straight vertical inlet + 90-deg elbow + horizontal outlet.

    The centerline itself is now the angle-valve path: inlet and outlet legs are
    perpendicular, with the bonnet carried by the elbow corner.
    """
    z_bend_start = Z0 - ELBOW_BEND_R
    z_bottom = z_bend_start - DROP_LEN
    cx = X_ELBOW + ELBOW_BEND_R          # bend center x == horizontal leg start
    cz = Z0 - ELBOW_BEND_R               # bend center z
    x_end = OUTLET_FLANGE_X + 2.0 * FLANGE_T + RUN_END_LEN

    eps = 0.012  # small overlap so unions stay continuous at the joints

    def _tube_cyl(origin, normal, length: float) -> cq.Workplane:
        return _annular_cylinder(origin, normal, length, PIPE_OR, PIPE_IR)

    # Straight legs are authored as true annular extrusions, not solid cylinders
    # post-cut by a bore. This keeps exported open ends from triangulating as
    # yellow cap disks in the viewer.
    v_shell = _tube_cyl((X_ELBOW, 0.0, z_bottom), (0, 0, 1), DROP_LEN + eps)
    h_shell = _tube_cyl((cx - eps, 0.0, Z0), (1, 0, 0), (x_end - cx) + eps)
    # Quarter-torus elbow, also hollow.
    e_out = _angle_quarter_torus(PIPE_OR)
    e_in = _angle_quarter_torus(PIPE_IR)
    e_shell = e_out.cut(e_in)

    return v_shell.union(e_shell).union(h_shell)


def _flange(center, axis: str) -> cq.Workplane:
    """Bolted raised-face flange (annular collar) perpendicular to a pipe leg."""
    if axis == "x":
        normal = (1, 0, 0)
        def bolt_xyz(ang: float):
            return (center[0], BOLT_CIRCLE_R * math.cos(ang), center[2] + BOLT_CIRCLE_R * math.sin(ang))
    elif axis == "z":
        normal = (0, 0, 1)
        def bolt_xyz(ang: float):
            return (center[0] + BOLT_CIRCLE_R * math.cos(ang), BOLT_CIRCLE_R * math.sin(ang), center[2])
    else:
        raise ValueError(f"unsupported flange axis {axis}")

    plane = cq.Plane(origin=center, normal=normal)
    disk = cq.Workplane(plane).circle(FLANGE_R).extrude(FLANGE_T, both=True)
    bore = cq.Workplane(plane).circle(PIPE_IR).extrude(FLANGE_T + 0.004, both=True)
    flange = disk.cut(bore)
    # Bolt heads spaced around the bolt circle, protruding on both faces.
    for i in range(N_BOLTS):
        ang = 2.0 * math.pi * i / N_BOLTS
        bolt = (
            cq.Workplane(cq.Plane(origin=bolt_xyz(ang), normal=normal))
            .polygon(6, BOLT_HEAD_R * 2.0)
            .extrude(FLANGE_T + BOLT_HEAD_H, both=True)
        )
        flange = flange.union(bolt)
    return flange


def _flange_pair(center, axis: str) -> cq.Workplane:
    """Two mating flange disks at one pipe joint, centered on the requested leg."""
    sep = FLANGE_T + 0.001
    if axis == "x":
        c0 = (center[0] - sep, center[1], center[2])
        c1 = (center[0] + sep, center[1], center[2])
    elif axis == "z":
        c0 = (center[0], center[1], center[2] - sep)
        c1 = (center[0], center[1], center[2] + sep)
    else:
        raise ValueError(f"unsupported flange axis {axis}")
    return _flange(c0, axis).union(_flange(c1, axis))


def _inlet_flange_pair() -> cq.Workplane:
    """Bolted inlet flange pair on the vertical leg of the angle valve."""
    return _flange_pair((X_ELBOW, 0.0, INLET_FLANGE_Z), "z")


def _outlet_flange_pair() -> cq.Workplane:
    """Bolted outlet flange pair on the horizontal leg, perpendicular to inlet."""
    return _flange_pair((OUTLET_FLANGE_X, 0.0, Z0), "x")


def _valve_body() -> cq.Workplane:
    """L-shaped angle-valve elbow casting around the 90-degree flow path."""
    z_bend_start = Z0 - ELBOW_BEND_R
    # Enlarged elbow shell around the same quarter-turn bore as the pipe.
    casting = _angle_quarter_torus(VALVE_BODY_R).cut(_angle_quarter_torus(PIPE_IR))

    # Short enlarged sockets blend the casting into the two flanged legs.
    h_socket = _annular_cylinder(
        (X_VALVE_CENTER - 0.035, 0.0, Z0),
        (1, 0, 0),
        min(VALVE_BODY_LEN, RUN_RIGHT_LEN + 0.035),
        VALVE_BODY_R,
        PIPE_IR,
    )
    v_socket = _annular_cylinder(
        (X_ELBOW, 0.0, z_bend_start - VALVE_BODY_LEN * 0.55),
        (0, 0, 1),
        VALVE_BODY_LEN * 0.72,
        VALVE_BODY_R,
        PIPE_IR,
    )
    # A raised top pad gives the bonnet a real visible seat on the elbow corner.
    bonnet_pad = (
        cq.Workplane(cq.Plane(origin=(X_VALVE_CENTER, 0.0, Z0 + PIPE_OR - 0.006), normal=(0, 0, 1)))
        .circle(BONNET_BASE_R * 1.08)
        .extrude(0.020)
    )
    return casting.union(h_socket).union(v_socket).union(bonnet_pad)


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
        mesh_from_cadquery(_inlet_flange_pair(), "inlet_flanges", tolerance=TOL, angular_tolerance=ATOL),
        material="bolt_steel",
        name="inlet_flanges",
    )
    body.visual(
        mesh_from_cadquery(_outlet_flange_pair(), "outlet_flanges", tolerance=TOL, angular_tolerance=ATOL),
        material="bolt_steel",
        name="outlet_flanges",
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
        "yellow pipe forms a hollow L-shaped run with vertical inlet and horizontal outlet",
        pipe_aabb is not None
        and (pipe_aabb[1][0] - pipe_aabb[0][0]) > 0.42
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

    # --- Flanges present on perpendicular inlet/outlet legs of the angle valve ---
    inlet_flange_aabb = ctx.part_element_world_aabb(body, elem="inlet_flanges")
    outlet_flange_aabb = ctx.part_element_world_aabb(body, elem="outlet_flanges")
    def _dims(aabb):
        return None if aabb is None else tuple(aabb[1][i] - aabb[0][i] for i in range(3))
    def _center(aabb):
        return None if aabb is None else tuple((aabb[0][i] + aabb[1][i]) / 2.0 for i in range(3))
    inlet_dims = _dims(inlet_flange_aabb)
    outlet_dims = _dims(outlet_flange_aabb)
    inlet_center = _center(inlet_flange_aabb)
    outlet_center = _center(outlet_flange_aabb)
    ctx.check(
        "inlet flange is on the vertical leg while outlet flange is on the horizontal leg",
        inlet_center is not None
        and outlet_center is not None
        and outlet_center[0] > inlet_center[0] + 0.12
        and outlet_center[2] > inlet_center[2] + 0.08,
        details=f"inlet_center={inlet_center}, outlet_center={outlet_center}",
    )
    ctx.check(
        "inlet and outlet flanges are perpendicular 90-degree collars",
        inlet_dims is not None
        and outlet_dims is not None
        and inlet_dims[2] < inlet_dims[0] * 0.45
        and outlet_dims[0] < outlet_dims[2] * 0.45
        and inlet_dims[0] > 2.0 * PIPE_OR + 0.02
        and outlet_dims[2] > 2.0 * PIPE_OR + 0.02,
        details=f"inlet_dims={inlet_dims}, outlet_dims={outlet_dims}",
    )
    valve_aabb = ctx.part_element_world_aabb(body, elem="valve_body")
    ctx.check(
        "bonnet rises from the elbow casting at the 90-degree corner",
        valve_aabb is not None
        and bonnet_aabb is not None
        and valve_aabb[0][0] < X_VALVE_CENTER < valve_aabb[1][0]
        and valve_aabb[0][2] < Z0 < valve_aabb[1][2]
        and abs(((bonnet_aabb[0][0] + bonnet_aabb[1][0]) / 2.0) - X_VALVE_CENTER) < 0.02,
        details=f"valve_aabb={valve_aabb}, bonnet_aabb={bonnet_aabb}, corner={(X_VALVE_CENTER, Z0)}",
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
