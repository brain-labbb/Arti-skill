from __future__ import annotations

# Double clear-plastic CD jewel case with two side-by-side disc trays.
#
# Coordinate convention:
#   - The case lies flat in the XY plane, "up" is +Z.
#   - X is the case width (~0.270 m for two discs), Y is the case depth (~0.125 m).
#   - The rear hinge runs left-right along the back +Y edge; the lid swings up
#     about that hinge (axis parallel to X).
#   - The front finger-notches are on the -Y edge.
#   - z=0 is the underside of the base frame; the assembly stacks upward in +Z.
#
# Parts / articulations:
#   - base (root): clear base frame + two fixed dark inner tray sections,
#     each with raised hub, rosette teeth, and front finger notches. STATIC.
#   - lid: clear hinged transparent cover. REVOLUTE about the rear hinge
#     (axis ~ +X), opens 0 -> ~70 deg.
#   - disc_0, disc_1: silver CDs. Each CONTINUOUS spin about its hub axis (+Z).

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    CylinderGeometry,
    Inertial,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key dimensions (meters) -------------------------------------------------
N_DISCS = 2
CASE_W = 0.270  # X extent (widened for two side-by-side discs)
CASE_D = 0.125  # Y extent
BASE_T = 0.006  # base frame thickness (Z)
WALL = 0.004  # frame wall thickness
LID_T = 0.005  # lid shell thickness (Z)
LID_WALL = 0.0016  # lid shell wall thickness

TRAY_SECTION_W = (CASE_W - 2 * WALL) / N_DISCS  # width of each tray section

HINGE_Y = CASE_D / 2.0  # rear hinge line at +Y edge
HINGE_Z = BASE_T  # hinge axis height (top of base frame)

DISC_R = 0.060  # CD outer radius (120 mm disc)
DISC_T = 0.0012  # CD thickness
DISC_HOLE_R = 0.0075  # CD center hole radius

HUB_R = 0.009  # raised tray hub radius
HUB_H = 0.006  # hub height above tray floor

TRAY_FLOOR_Z = BASE_T  # tray floor sits at top of base frame
FLOOR_T = 0.0022  # tray floor thickness
DISC_Z = TRAY_FLOOR_Z + FLOOR_T + HUB_H * 0.7  # disc seating height (on rosette)


def disc_cx(i: int) -> float:
    """X center of disc/tray section i (evenly spaced across case width)."""
    return (i - (N_DISCS - 1) / 2.0) * TRAY_SECTION_W


# ---- shared geometry helpers ------------------------------------------------


def _shallow_shell(w: float, d: float, wall: float, depth: float) -> cq.Workplane:
    """Open-topped shallow rectangular shell with thin walls and floor."""
    outer = cq.Workplane("XY").box(w, d, depth, centered=(True, True, False))
    inner = (
        cq.Workplane("XY")
        .workplane(offset=wall)
        .box(w - 2 * wall, d - 2 * wall, depth, centered=(True, True, False))
    )
    return outer.cut(inner)


def _base_frame() -> cq.Workplane:
    """Clear base frame: a shallow rectangular shell holding the trays."""
    return _shallow_shell(CASE_W, CASE_D, WALL, BASE_T)


def _tray_section(cx: float) -> cq.Workplane:
    """One tray section at X=cx: floor + hub + rosette teeth + finger notches."""
    tw = TRAY_SECTION_W + 0.0008  # slight overlap into frame walls for connectivity
    td = CASE_D - 2 * WALL + 0.0008

    # Rectangular floor section
    tray = (
        cq.Workplane("XY")
        .workplane(offset=TRAY_FLOOR_Z)
        .moveTo(cx, 0)
        .rect(tw, td)
        .extrude(FLOOR_T)
    )

    # Raised circular hub boss at section center
    hub = (
        cq.Workplane("XY")
        .workplane(offset=TRAY_FLOOR_Z + FLOOR_T)
        .moveTo(cx, 0)
        .circle(HUB_R)
        .extrude(HUB_H)
    )
    tray = tray.union(hub)

    # Rosette: radial flexure teeth ring around the hub
    teeth = cq.Workplane("XY").workplane(offset=TRAY_FLOOR_Z + FLOOR_T)
    n_teeth = 8
    for j in range(n_teeth):
        a = 2 * math.pi * j / n_teeth
        tx = cx + (HUB_R + 0.0015) * math.cos(a)
        ty = (HUB_R + 0.0015) * math.sin(a)
        teeth = teeth.moveTo(tx, ty).circle(0.0016).extrude(HUB_H * 0.7)
    tray = tray.union(teeth)

    # Front finger-notch cutouts: two scoops on the -Y edge for disc removal
    notch = cq.Workplane("XY").workplane(offset=TRAY_FLOOR_Z - 0.001)
    for nx_off in (-0.026, 0.026):
        notch = (
            notch.moveTo(cx + nx_off, -td / 2.0)
            .circle(0.013)
            .extrude(FLOOR_T + 0.003)
        )
    tray = tray.cut(notch)

    return tray


def _lid_shell() -> cq.Workplane:
    """Clear hinged cover: shallow shell matching the widened base."""
    w = CASE_W
    d = CASE_D
    outer = (
        cq.Workplane("XY")
        .workplane(offset=-LID_T)
        .box(w, d, LID_T, centered=(True, True, False))
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=-LID_T)
        .box(w - 2 * LID_WALL, d - 2 * LID_WALL, LID_T - LID_WALL, centered=(True, True, False))
    )
    return outer.cut(inner)


def _disc_solid() -> cq.Workplane:
    """Silver CD: thin cylinder with central hole."""
    disc = (
        cq.Workplane("XY")
        .circle(DISC_R)
        .extrude(DISC_T)
        .faces(">Z")
        .workplane()
        .hole(DISC_HOLE_R * 2.0)
    )
    return disc


# ---- assembly ---------------------------------------------------------------


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cd_jewel_case_dual")

    clear = model.material("clear_plastic", rgba=(0.55, 0.60, 0.68, 0.30))
    tray_dark = model.material("tray_dark", rgba=(0.10, 0.10, 0.12, 1.0))
    silver = model.material("disc_silver", rgba=(0.80, 0.82, 0.85, 1.0))

    # ---- base (root): clear frame + two dark tray sections ----
    base = model.part("base")

    base.visual(
        mesh_from_cadquery(_base_frame(), "base_frame"),
        material=clear,
        name="base_frame",
    )

    # Two tray sections emitted by loop — each is a parent visual on base
    for i in range(N_DISCS):
        cx = disc_cx(i)
        base.visual(
            mesh_from_cadquery(_tray_section(cx), f"tray_{i}"),
            material=tray_dark,
            name=f"tray_{i}",
        )

    base.inertial = Inertial.from_geometry(
        Box((CASE_W, CASE_D, BASE_T + HUB_H)),
        mass=0.080,
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
    )

    # ---- lid: clear hinged transparent cover, revolute about rear hinge ----
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell(), "lid_shell"),
        material=clear,
        name="lid_shell",
    )
    lid.inertial = Inertial.from_geometry(
        Box((CASE_W, CASE_D, LID_T)),
        mass=0.040,
        origin=Origin(xyz=(0.0, -CASE_D / 2.0, -LID_T / 2.0)),
    )
    # Hinge frame at rear +Y edge, top of base frame. Axis +X so positive q
    # lifts the front (-Y) edge upward in +Z.
    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=math.radians(70.0)),
    )

    # ---- discs: two silver CDs, each with independent continuous spin ----
    for i in range(N_DISCS):
        cx = disc_cx(i)
        disc_part = model.part(f"disc_{i}")
        disc_part.visual(
            mesh_from_cadquery(_disc_solid(), f"disc_{i}_body"),
            material=silver,
            name=f"disc_{i}_body",
        )
        # Off-center label marker so rotation is visually/numerically detectable
        disc_part.visual(
            Cylinder(radius=0.004, length=DISC_T),
            origin=Origin(xyz=(0.030, 0.0, DISC_T / 2.0)),
            material=tray_dark,
            name=f"disc_{i}_marker",
        )
        disc_part.inertial = Inertial.from_geometry(
            Cylinder(radius=DISC_R, length=DISC_T),
            mass=0.016,
            origin=Origin(xyz=(0.0, 0.0, DISC_T / 2.0)),
        )
        # Disc center hole drops over the hub; joint origin at disc seating surface
        model.articulation(
            f"hub_to_disc_{i}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=disc_part,
            origin=Origin(xyz=(cx, 0.0, DISC_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.2, velocity=8.0),
        )

    return model


# ---- tests ------------------------------------------------------------------


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    lid_joint = object_model.get_articulation("base_to_lid")

    # --- transparency checks ---
    lid_mat = lid.get_visual("lid_shell").material
    alpha = lid_mat.rgba[3] if lid_mat.rgba is not None else 1.0
    ctx.check("lid is transparent (alpha < 1)", alpha < 0.6, details=f"lid alpha={alpha}")

    base_mat = base.get_visual("base_frame").material
    base_alpha = base_mat.rgba[3] if base_mat.rgba is not None else 1.0
    ctx.check("base frame is clear plastic (alpha < 1)", base_alpha < 0.6,
              details=f"base alpha={base_alpha}")

    # --- lid hinges about the rear edge: opening lifts the front edge up ---
    closed_top = ctx.part_world_aabb(lid)[1][2]
    closed_front_y = ctx.part_world_aabb(lid)[0][1]
    with ctx.pose({lid_joint: math.radians(60.0)}):
        open_aabb = ctx.part_world_aabb(lid)
        open_top = open_aabb[1][2]
        open_front_y = open_aabb[0][1]
    ctx.check("lid swings open and lifts upward", open_top > closed_top + 0.03,
              details=f"closed_top={closed_top}, open_top={open_top}")
    ctx.check("opening swings the free edge toward the hinge (front y moves rearward)",
              open_front_y > closed_front_y + 0.02,
              details=f"closed_front_y={closed_front_y}, open_front_y={open_front_y}")
    ctx.check("hinge axis runs along X (left-right)", abs(lid_joint.axis[0]) > 0.9,
              details=f"axis={lid_joint.axis}")

    # --- two tray sections exist as distinct visuals on base ---
    for i in range(N_DISCS):
        ctx.check(f"tray_{i} visual exists on base",
                  base.get_visual(f"tray_{i}") is not None,
                  details=f"tray_{i} not found on base")

    # --- two discs, each with independent continuous spin ---
    disc_parts = [object_model.get_part(f"disc_{i}") for i in range(N_DISCS)]
    disc_joints = [object_model.get_articulation(f"hub_to_disc_{i}") for i in range(N_DISCS)]

    for i in range(N_DISCS):
        disc_i = disc_parts[i]
        joint_i = disc_joints[i]
        cx = disc_cx(i)

        # Spin axis is vertical
        ctx.check(f"disc_{i} spin axis is vertical (+Z)", abs(joint_i.axis[2]) > 0.9,
                  details=f"axis={joint_i.axis}")

        # Spin moves the off-center marker (quarter turn shifts X/Y)
        marker_rest = ctx.part_element_world_aabb(disc_i, elem=f"disc_{i}_marker")
        rest_mx = 0.5 * (marker_rest[0][0] + marker_rest[1][0])
        rest_my = 0.5 * (marker_rest[0][1] + marker_rest[1][1])
        with ctx.pose({joint_i: math.radians(90.0)}):
            marker_q = ctx.part_element_world_aabb(disc_i, elem=f"disc_{i}_marker")
            q_mx = 0.5 * (marker_q[0][0] + marker_q[1][0])
            q_my = 0.5 * (marker_q[0][1] + marker_q[1][1])
        ctx.check(f"disc_{i} spin moves the off-center marker around the hub axis",
                  abs(q_mx - rest_mx) > 0.02 and abs(q_my - rest_my) > 0.02,
                  details=f"rest=({rest_mx:.4f},{rest_my:.4f}) q=({q_mx:.4f},{q_my:.4f})")

        # Disc centered at its hub X position
        disc_pos = ctx.part_world_position(disc_i)
        ctx.check(f"disc_{i} is centered over hub_{i} position",
                  abs(disc_pos[0] - cx) < 0.005 and abs(disc_pos[1]) < 0.005,
                  details=f"disc origin={disc_pos}, expected cx={cx:.4f}")

        # Disc within case footprint
        ctx.expect_within(disc_i, base, axes="xy",
                          inner_elem=f"disc_{i}_body", outer_elem="base_frame",
                          margin=0.002,
                          name=f"disc_{i} sits within the case footprint")

        # Disc rests on hub above tray floor
        disc_bottom = ctx.part_world_aabb(disc_i)[0][2]
        ctx.check(f"disc_{i} rests on tray hub above tray floor",
                  TRAY_FLOOR_Z - 0.001 < disc_bottom < TRAY_FLOOR_Z + HUB_H + 0.002,
                  details=f"disc_bottom={disc_bottom}, floor={TRAY_FLOOR_Z}")

        # Allow hub-through-disc-bore overlap (hub captures the disc)
        ctx.allow_overlap(
            disc_i, base,
            elem_a=f"disc_{i}_body", elem_b=f"tray_{i}",
            reason=f"disc_{i} center hole drops over tray_{i} raised hub/rosette; "
                   f"the hub intentionally pokes through the disc bore to capture it.",
        )

    # --- co-planarity: both discs seat at the same Z height ---
    disc0_bottom = ctx.part_world_aabb(disc_parts[0])[0][2]
    disc1_bottom = ctx.part_world_aabb(disc_parts[1])[0][2]
    ctx.check("both discs are co-planar (same seating height)",
              abs(disc0_bottom - disc1_bottom) < 0.001,
              details=f"disc_0_bottom={disc0_bottom:.5f}, disc_1_bottom={disc1_bottom:.5f}")

    # --- tray sections seated in the base frame ---
    for i in range(N_DISCS):
        ctx.allow_overlap(
            base, base,
            elem_a=f"tray_{i}", elem_b="base_frame",
            reason=f"tray_{i} is press-fit into the clear base frame; floor edges "
                   f"seat a fraction of a millimeter into the frame walls.",
        )
        ctx.expect_within(
            base, base, axes="xy",
            inner_elem=f"tray_{i}", outer_elem="base_frame",
            margin=0.001,
            name=f"tray_{i} nests within the base frame footprint",
        )

    return ctx.report()


object_model = build_object_model()
