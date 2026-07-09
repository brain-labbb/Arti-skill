from __future__ import annotations

# Double-wide clear-plastic CD jewel case (two-disc variant, lying open).
#
# This is a double-wide fork of the standard single-disc jewel case. The base
# frame, tray, and lid are roughly twice the standard width (~0.284 m vs the
# usual ~0.142 m) while depth and thickness remain standard. The dark inner
# tray has two hub/rosette disc positions (one per half), a center divider
# ridge, and paired front finger-notches. A single silver CD sits on one hub.
#
# Coordinate convention:
#   - The case lies flat in the XY plane, "up" is +Z.
#   - X is the case width (~0.284 m), Y is the case depth (~0.125 m).
#   - The rear hinge runs left-right along the back +Y edge; the lid swings up
#     about that hinge (axis parallel to X).
#   - The front finger-notches are on the -Y edge.
#   - z=0 is the underside of the base frame; the assembly stacks upward in +Z.
#
# Parts / articulations:
#   - base (root): clear base frame + dark inner tray floor with center divider
#     + two hub/rosette assemblies (hub_rosette_0, hub_rosette_1). STATIC.
#   - lid: clear hinged transparent cover spanning the full widened body.
#     REVOLUTE about the rear hinge (axis ~ +X), opens 0 -> ~70 deg.
#   - disc: silver CD on one hub. CONTINUOUS spin about the vertical hub axis.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
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
CASE_W = 0.284        # X extent (double-wide, ~2× standard 0.142)
CASE_D = 0.125        # Y extent (standard depth, unchanged)
BASE_T = 0.006        # base frame thickness (Z)
WALL = 0.004          # frame wall thickness
LID_T = 0.005         # lid shell thickness (Z)
LID_WALL = 0.0016     # lid shell wall thickness

HINGE_Y = CASE_D / 2.0   # rear hinge line at +Y edge
HINGE_Z = BASE_T          # hinge axis height (top of base frame)

DISC_R = 0.060         # CD outer radius (120 mm disc)
DISC_T = 0.0012        # CD thickness
DISC_HOLE_R = 0.0075   # CD center hole radius

HUB_R = 0.009          # raised tray hub radius
HUB_H = 0.006          # hub height above tray floor

TRAY_FLOOR_Z = BASE_T        # tray floor sits at top of base frame
TRAY_FLOOR_T = 0.0022        # tray floor plate thickness

# Two disc-slot hub X positions: each centred in its half of the double case.
N_DISC_SLOTS = 2
HUB_X = [-(CASE_W / 4.0), +(CASE_W / 4.0)]

# Which slot holds the disc (0 = left, 1 = right).
DISC_SLOT = 1


# ---------------------------------------------------------------------------
# Shared geometry helpers
# ---------------------------------------------------------------------------

def _shallow_shell(w: float, d: float, wall: float, depth: float) -> cq.Workplane:
    """Open-topped rectangular shell with thin walls and floor."""
    outer = cq.Workplane("XY").box(w, d, depth, centered=(True, True, False))
    inner = (
        cq.Workplane("XY")
        .workplane(offset=wall)
        .box(w - 2 * wall, d - 2 * wall, depth, centered=(True, True, False))
    )
    return outer.cut(inner)


def _base_frame() -> cq.Workplane:
    """Clear base frame: shallow rectangular shell (open top)."""
    return _shallow_shell(CASE_W, CASE_D, WALL, BASE_T)


def _hub_rosette(x_offset: float) -> cq.Workplane:
    """Shared helper: one raised hub with 8 rosette teeth at *x_offset*.

    The hub base embeds ~0.5 mm below the tray-floor top surface so it is
    geometrically connected (press-fit boss) to the tray floor visual.
    """
    base_z = TRAY_FLOOR_Z + TRAY_FLOOR_T
    embed = 0.0005  # press-fit embed depth

    # Hub cylinder (starts slightly below floor top for overlap connectivity).
    hub = (
        cq.Workplane("XY")
        .workplane(offset=base_z - embed)
        .circle(HUB_R)
        .extrude(HUB_H + embed)
        .translate((x_offset, 0.0, 0.0))
    )

    # Rosette: 8 radial flexure teeth around the hub.
    n_teeth = 8
    for i in range(n_teeth):
        a = 2 * math.pi * i / n_teeth
        tx = x_offset + (HUB_R + 0.0015) * math.cos(a)
        ty = (HUB_R + 0.0015) * math.sin(a)
        tooth = (
            cq.Workplane("XY")
            .workplane(offset=base_z)
            .circle(0.0016)
            .extrude(HUB_H * 0.7)
            .translate((tx, ty, 0.0))
        )
        hub = hub.union(tooth)

    return hub


def _tray_floor() -> cq.Workplane:
    """Dark tray floor plate with a centre divider ridge and finger-notch cuts.

    One pair of front-edge finger-notches per disc slot, emitted in a loop.
    """
    tw = CASE_W - 2 * WALL + 0.0008
    td = CASE_D - 2 * WALL + 0.0008

    # Main floor plate.
    floor = (
        cq.Workplane("XY")
        .workplane(offset=TRAY_FLOOR_Z)
        .box(tw, td, TRAY_FLOOR_T, centered=(True, True, False))
    )

    # Centre divider ridge (thin raised wall running front-to-back).
    divider = (
        cq.Workplane("XY")
        .workplane(offset=TRAY_FLOOR_Z + TRAY_FLOOR_T)
        .box(0.003, td - 0.010, 0.003, centered=(True, True, False))
    )
    floor = floor.union(divider)

    # Finger-notch cutouts: one pair per disc slot on the -Y front edge.
    for slot in range(N_DISC_SLOTS):
        hx = HUB_X[slot]
        for dx in (-0.026, 0.026):
            cut = (
                cq.Workplane("XY")
                .workplane(offset=TRAY_FLOOR_Z - 0.001)
                .circle(0.013)
                .extrude(TRAY_FLOOR_T + 0.003)
                .translate((hx + dx, -td / 2.0, 0.0))
            )
            floor = floor.cut(cut)

    return floor


def _lid_shell() -> cq.Workplane:
    """Clear hinged cover spanning the full widened body."""
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
    """Silver CD: thin cylinder with a central hole."""
    return (
        cq.Workplane("XY")
        .circle(DISC_R)
        .extrude(DISC_T)
        .faces(">Z")
        .workplane()
        .hole(DISC_HOLE_R * 2.0)
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cd_jewel_case_double")

    clear = model.material("clear_plastic", rgba=(0.55, 0.60, 0.68, 0.30))
    tray_dark = model.material("tray_dark", rgba=(0.10, 0.10, 0.12, 1.0))
    silver = model.material("disc_silver", rgba=(0.80, 0.82, 0.85, 1.0))

    # ---- base (root): clear frame + dark tray floor + two hub/rosettes ----
    base = model.part("base")

    base.visual(
        mesh_from_cadquery(_base_frame(), "base_frame"),
        material=clear,
        name="base_frame",
    )
    base.visual(
        mesh_from_cadquery(_tray_floor(), "tray_floor"),
        material=tray_dark,
        name="tray_floor",
    )

    # Hub+rosette assemblies: emitted via for-loop with name_i naming,
    # shared geometry helper (_hub_rosette), and regular X placement.
    for i in range(N_DISC_SLOTS):
        base.visual(
            mesh_from_cadquery(_hub_rosette(HUB_X[i]), f"hub_rosette_{i}"),
            material=tray_dark,
            name=f"hub_rosette_{i}",
        )

    base.inertial = Inertial.from_geometry(
        Box((CASE_W, CASE_D, BASE_T + HUB_H)),
        mass=0.095,
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
    )

    # ---- lid: clear hinged transparent cover spanning the full widened body ----
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell(), "lid_shell"),
        material=clear,
        name="lid_shell",
    )
    lid.inertial = Inertial.from_geometry(
        Box((CASE_W, CASE_D, LID_T)),
        mass=0.055,
        origin=Origin(xyz=(0.0, -CASE_D / 2.0, -LID_T / 2.0)),
    )

    # Single rear lid hinge: axis +X so positive q lifts the front edge up.
    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0,
            lower=0.0, upper=math.radians(70.0),
        ),
    )

    # ---- disc: silver CD on one hub (DISC_SLOT side), continuous spin ----
    disc = model.part("disc")
    disc.visual(
        mesh_from_cadquery(_disc_solid(), "disc_body"),
        material=silver,
        name="disc_body",
    )
    # Small off-centre label marker so rotation is visually/numerically detectable.
    disc.visual(
        Cylinder(radius=0.004, length=DISC_T),
        origin=Origin(xyz=(0.030, 0.0, DISC_T / 2.0)),
        material=tray_dark,
        name="disc_marker",
    )
    disc.inertial = Inertial.from_geometry(
        Cylinder(radius=DISC_R, length=DISC_T),
        mass=0.016,
        origin=Origin(xyz=(0.0, 0.0, DISC_T / 2.0)),
    )

    # Disc centre hole drops over the hub; it rests on the rosette/hub top.
    disc_z = TRAY_FLOOR_Z + TRAY_FLOOR_T + HUB_H * 0.7
    disc_hub_x = HUB_X[DISC_SLOT]
    model.articulation(
        "hub_to_disc",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=disc,
        origin=Origin(xyz=(disc_hub_x, 0.0, disc_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    disc = object_model.get_part("disc")
    lid_joint = object_model.get_articulation("base_to_lid")
    disc_joint = object_model.get_articulation("hub_to_disc")

    # --- prompt-critical: double-wide case ----------------------------------
    base_aabb = ctx.part_world_aabb(base)
    base_width = base_aabb[1][0] - base_aabb[0][0]
    ctx.check(
        "base is double-wide (width >= 0.25 m, roughly 2× standard 0.142)",
        base_width >= 0.25,
        details=f"base_width={base_width:.4f}",
    )
    # Depth unchanged from standard.
    base_depth = base_aabb[1][1] - base_aabb[0][1]
    ctx.check(
        "case depth is standard (~0.12–0.13 m, not widened)",
        0.10 < base_depth < 0.15,
        details=f"base_depth={base_depth:.4f}",
    )

    # --- two hub positions exist (loop-emitted visuals) ---------------------
    for i in range(N_DISC_SLOTS):
        ctx.check(
            f"hub_rosette_{i} visual exists",
            base.get_visual(f"hub_rosette_{i}") is not None,
            details=f"hub_rosette_{i} must be present on the base",
        )

    # The two hubs are separated across the width (one per half).
    hub0_aabb = ctx.part_element_world_aabb(base, elem="hub_rosette_0")
    hub1_aabb = ctx.part_element_world_aabb(base, elem="hub_rosette_1")
    hub0_cx = 0.5 * (hub0_aabb[0][0] + hub1_aabb[0][0])  # intentional: hub0 vs hub1
    hub_sep = abs(
        0.5 * (hub1_aabb[0][0] + hub1_aabb[1][0])
        - 0.5 * (hub0_aabb[0][0] + hub0_aabb[1][0])
    )
    ctx.check(
        "two hubs are separated across the case width (>= 0.10 m apart)",
        hub_sep >= 0.10,
        details=f"hub_separation={hub_sep:.4f}",
    )

    # Both hubs sit within the base frame footprint.
    for i in range(N_DISC_SLOTS):
        ctx.expect_within(
            base, base,
            axes="xy",
            inner_elem=f"hub_rosette_{i}",
            outer_elem="base_frame",
            margin=0.002,
            name=f"hub_rosette_{i} within base footprint",
        )

    # --- clear/transparent materials ----------------------------------------
    lid_mat = lid.get_visual("lid_shell").material
    alpha = lid_mat.rgba[3] if lid_mat.rgba is not None else 1.0
    ctx.check("lid is transparent (alpha < 1)", alpha < 0.6, details=f"lid alpha={alpha}")

    base_mat = base.get_visual("base_frame").material
    base_alpha = base_mat.rgba[3] if base_mat.rgba is not None else 1.0
    ctx.check(
        "base frame is clear plastic (alpha < 1)",
        base_alpha < 0.6,
        details=f"base alpha={base_alpha}",
    )

    # --- lid hinges about the rear edge: opening lifts the front edge up ----
    closed_top = ctx.part_world_aabb(lid)[1][2]
    closed_front_y = ctx.part_world_aabb(lid)[0][1]
    with ctx.pose({lid_joint: math.radians(60.0)}):
        open_aabb = ctx.part_world_aabb(lid)
        open_top = open_aabb[1][2]
        open_front_y = open_aabb[0][1]
    ctx.check(
        "lid swings open and lifts upward",
        open_top > closed_top + 0.03,
        details=f"closed_top={closed_top}, open_top={open_top}",
    )
    ctx.check(
        "opening swings the free edge toward the hinge (front y moves rearward)",
        open_front_y > closed_front_y + 0.02,
        details=f"closed_front_y={closed_front_y}, open_front_y={open_front_y}",
    )
    ctx.check(
        "hinge axis runs along X (left-right)",
        abs(lid_joint.axis[0]) > 0.9,
        details=f"axis={lid_joint.axis}",
    )

    # --- lid spans the full widened body (XY overlap with base) -------------
    ctx.expect_overlap(
        lid, base,
        axes="x",
        elem_a="lid_shell",
        elem_b="base_frame",
        min_overlap=CASE_W * 0.80,
        name="lid spans at least 80% of the widened base width",
    )

    # --- disc spins about the vertical hub axis -----------------------------
    marker_rest = ctx.part_element_world_aabb(disc, elem="disc_marker")
    rest_cx = 0.5 * (marker_rest[0][0] + marker_rest[1][0])
    rest_cy = 0.5 * (marker_rest[0][1] + marker_rest[1][1])
    with ctx.pose({disc_joint: math.radians(90.0)}):
        marker_q = ctx.part_element_world_aabb(disc, elem="disc_marker")
        q_cx = 0.5 * (marker_q[0][0] + marker_q[1][0])
        q_cy = 0.5 * (marker_q[0][1] + marker_q[1][1])
    ctx.check(
        "disc spin moves the off-center marker around the hub axis",
        abs(q_cx - rest_cx) > 0.02 and abs(q_cy - rest_cy) > 0.02,
        details=f"rest=({rest_cx:.4f},{rest_cy:.4f}) quarter=({q_cx:.4f},{q_cy:.4f})",
    )
    ctx.check(
        "disc spin axis is vertical (+Z)",
        abs(disc_joint.axis[2]) > 0.9,
        details=f"axis={disc_joint.axis}",
    )

    # --- disc is on one side (not centred in the case) ----------------------
    disc_pos = ctx.part_world_position(disc)
    expected_x = HUB_X[DISC_SLOT]
    ctx.check(
        "disc is offset to one side of the widened case",
        abs(disc_pos[0] - expected_x) < 0.005,
        details=f"disc_x={disc_pos[0]:.4f}, expected_x={expected_x:.4f}",
    )
    ctx.check(
        "disc is NOT centred in the double-wide case",
        abs(disc_pos[0]) > CASE_W / 6.0,
        details=f"disc_x={disc_pos[0]:.4f}, half_width={CASE_W/2:.4f}",
    )

    # --- disc sits on the tray hub (captured by the hub bore) ---------------
    ctx.allow_overlap(
        disc, base,
        elem_a="disc_body",
        elem_b=f"hub_rosette_{DISC_SLOT}",
        reason="The CD centre hole drops over the raised tray hub/rosette; the hub "
               "intentionally pokes through the disc bore so the disc is captured "
               "and seated on its slot.",
    )
    ctx.expect_within(
        disc, base,
        axes="xy",
        inner_elem="disc_body",
        outer_elem="base_frame",
        margin=0.002,
        name="disc sits within the case footprint",
    )
    disc_bottom = ctx.part_world_aabb(disc)[0][2]
    ctx.check(
        "disc rests on the tray hub above the tray floor",
        TRAY_FLOOR_Z - 0.001 < disc_bottom < TRAY_FLOOR_Z + HUB_H + 0.003,
        details=f"disc_bottom={disc_bottom}, floor={TRAY_FLOOR_Z}",
    )

    # --- tray floor seated in the base frame --------------------------------
    ctx.allow_overlap(
        base, base,
        elem_a="tray_floor",
        elem_b="base_frame",
        reason="The dark inner tray floor is press-fit into the clear base frame; "
               "its edges intentionally seat a fraction of a millimeter into the "
               "frame walls.",
    )
    ctx.expect_within(
        base, base,
        axes="xy",
        inner_elem="tray_floor",
        outer_elem="base_frame",
        margin=0.001,
        name="tray floor nests within the base frame footprint",
    )

    # --- hub rosettes press-fit into the tray floor -------------------------
    for i in range(N_DISC_SLOTS):
        ctx.allow_overlap(
            base, base,
            elem_a=f"hub_rosette_{i}",
            elem_b="tray_floor",
            reason=f"Hub rosette {i} is a press-fit boss that embeds slightly "
                   "into the tray floor for structural connectivity.",
        )

    return ctx.report()


object_model = build_object_model()
