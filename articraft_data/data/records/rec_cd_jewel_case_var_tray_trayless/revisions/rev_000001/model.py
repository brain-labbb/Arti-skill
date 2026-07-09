from __future__ import annotations

# Sleeve-pocket CD jewel case variant (lying open).
#
# Coordinate convention:
#   - The case lies flat in the XY plane, "up" is +Z.
#   - X is the case width (~0.142 m), Y is the case depth (~0.125 m).
#   - The rear hinge runs left-right along the back +Y edge; the lid swings up
#     about that hinge (axis parallel to X).
#   - The front (-Y) edge is the pocket opening where the CD slides in.
#   - z=0 is the underside of the base frame; the assembly stacks upward in +Z.
#
# Parts / articulations:
#   - base (root): clear base frame + paper/fabric pocket sleeve + static
#     silver CD resting flat inside the pocket. No raised hub, no dark tray,
#     no finger notches. STATIC.
#   - lid: clear hinged transparent cover. REVOLUTE about the rear hinge
#     (axis ~ +X), opens 0 -> ~70 deg.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- key dimensions (meters) -------------------------------------------------
CASE_W = 0.142  # X extent
CASE_D = 0.125  # Y extent
BASE_T = 0.006  # base frame thickness (Z)
WALL = 0.004  # frame wall thickness
LID_T = 0.005  # lid shell thickness (Z)
LID_WALL = 0.0016  # lid shell wall thickness

HINGE_Y = CASE_D / 2.0  # rear hinge line at +Y edge
HINGE_Z = BASE_T  # hinge axis height (top of base frame)

DISC_R = 0.060  # CD outer radius (120 mm disc)
DISC_T = 0.0012  # CD thickness
DISC_HOLE_R = 0.0075  # CD center hole radius

TRAY_FLOOR_Z = BASE_T  # pocket bottom sits at top of base frame

SLEEVE_THICK = 0.0008  # paper/fabric sheet thickness
SLEEVE_INTERIOR = DISC_T + 0.0003  # interior pocket gap (clearance for disc)


# ---- geometry helpers --------------------------------------------------------

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
    """Clear base frame: shallow rectangular shell that holds the pocket."""
    return _shallow_shell(CASE_W, CASE_D, WALL, BASE_T)


def _sleeve_pocket() -> cq.Workplane:
    """Flat paper/fabric pocket sleeve: hollow shell open on the -Y front edge.

    The pocket is a thin rectangular envelope with:
    - a bottom sheet (against the base floor)
    - a top sheet with a circular window (to view the disc label)
    - sealed side walls and back wall
    - an open front (-Y) edge where the CD slides in
    """
    sw = CASE_W - 2 * WALL + 0.006  # slightly overlaps base walls for connectivity
    sd = CASE_D - 2 * WALL + 0.006
    st = SLEEVE_THICK
    total_h = st * 2 + SLEEVE_INTERIOR  # bottom + gap + top

    # Outer solid shell
    outer = (
        cq.Workplane("XY")
        .workplane(offset=TRAY_FLOOR_Z)
        .box(sw, sd, total_h, centered=(True, True, False))
    )

    # Interior cavity: shifted toward -Y to break through the front face,
    # leaving side walls (st thick) and a back wall (st thick).
    cav_w = sw - 2 * st
    cav_d = sd  # full depth; Y-shift breaks front, leaves back wall
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=TRAY_FLOOR_Z + st)
        .center(0, -st)
        .box(cav_w, cav_d, SLEEVE_INTERIOR, centered=(True, True, False))
    )
    pocket = outer.cut(cavity)

    # Circular window in the top sheet to show the disc label area.
    window_z = TRAY_FLOOR_Z + st + SLEEVE_INTERIOR - 0.0001
    window = (
        cq.Workplane("XY")
        .workplane(offset=window_z)
        .circle(DISC_R * 0.50)
        .extrude(st + 0.0002)
    )
    pocket = pocket.cut(window)

    return pocket


def _lid_shell() -> cq.Workplane:
    """Clear hinged cover: shallow shell mirroring the base footprint."""
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
    disc = (
        cq.Workplane("XY")
        .circle(DISC_R)
        .extrude(DISC_T)
        .faces(">Z")
        .workplane()
        .hole(DISC_HOLE_R * 2.0)
    )
    return disc


# ---- model -------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cd_sleeve_case")

    clear = model.material("clear_plastic", rgba=(0.55, 0.60, 0.68, 0.30))
    paper = model.material("sleeve_paper", rgba=(0.92, 0.88, 0.82, 1.0))
    silver = model.material("disc_silver", rgba=(0.80, 0.82, 0.85, 1.0))

    # ---- base (root): clear frame + paper pocket sleeve + static disc ----
    base = model.part("base")

    base.visual(
        mesh_from_cadquery(_base_frame(), "base_frame"),
        material=clear,
        name="base_frame",
    )
    base.visual(
        mesh_from_cadquery(_sleeve_pocket(), "sleeve_pocket"),
        material=paper,
        name="sleeve_pocket",
    )
    # Silver CD resting flat inside the pocket cavity (no hub, no spin).
    disc_z = TRAY_FLOOR_Z + SLEEVE_THICK
    base.visual(
        mesh_from_cadquery(_disc_solid(), "disc_body"),
        origin=Origin(xyz=(0.0, 0.0, disc_z)),
        material=silver,
        name="disc_body",
    )
    base.inertial = Inertial.from_geometry(
        Box((CASE_W, CASE_D, BASE_T + SLEEVE_THICK * 2 + SLEEVE_INTERIOR)),
        mass=0.070,
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
    )

    # ---- lid: clear hinged transparent cover, revolute about the rear hinge ----
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell(), "lid_shell"),
        material=clear,
        name="lid_shell",
    )
    lid.inertial = Inertial.from_geometry(
        Box((CASE_W, CASE_D, LID_T)),
        mass=0.035,
        origin=Origin(xyz=(0.0, -CASE_D / 2.0, -LID_T / 2.0)),
    )
    # Hinge frame at the rear +Y edge, at the top of the base frame. The lid
    # local origin sits on the hinge line; its body extends to local -Y. Axis +X
    # so positive q lifts the front (-Y) edge up in +Z.
    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0, lower=0.0, upper=math.radians(70.0)
        ),
    )

    return model


# ---- tests -------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    lid_joint = object_model.get_articulation("base_to_lid")

    # --- lid is clear/transparent (material alpha < 1) ---
    lid_mat = lid.get_visual("lid_shell").material
    alpha = lid_mat.rgba[3] if lid_mat.rgba is not None else 1.0
    ctx.check(
        "lid is transparent (alpha < 1)",
        alpha < 0.6,
        details=f"lid alpha={alpha}",
    )
    base_mat = base.get_visual("base_frame").material
    base_alpha = base_mat.rgba[3] if base_mat.rgba is not None else 1.0
    ctx.check(
        "base frame is clear plastic (alpha < 1)",
        base_alpha < 0.6,
        details=f"base alpha={base_alpha}",
    )

    # --- lid hinges about the rear edge: opening lifts the front edge up ---
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

    # Hinge axis runs along X (left-right).
    ctx.check(
        "hinge axis runs along X (left-right)",
        abs(lid_joint.axis[0]) > 0.9,
        details=f"axis={lid_joint.axis}",
    )

    # --- no dark inner tray / hub / rosette (trayless sleeve variant) ---
    tray_visuals = [v for v in base.visuals if v.name == "inner_tray"]
    ctx.check(
        "no dark inner tray present (trayless variant)",
        len(tray_visuals) == 0,
        details=f"found {len(tray_visuals)} inner_tray visuals",
    )

    # --- sleeve pocket exists and is opaque paper/fabric material ---
    sleeve = base.get_visual("sleeve_pocket")
    sleeve_mat = sleeve.material
    sleeve_alpha = sleeve_mat.rgba[3] if sleeve_mat.rgba is not None else 1.0
    ctx.check(
        "sleeve pocket is opaque paper/fabric (alpha >= 0.8)",
        sleeve_alpha >= 0.8,
        details=f"sleeve alpha={sleeve_alpha}",
    )

    # --- disc is static: no separate disc part, no continuous articulation ---
    continuous_joints = [
        a for a in object_model.articulations
        if a.articulation_type == ArticulationType.CONTINUOUS
    ]
    ctx.check(
        "no continuous spin articulation (disc is static in sleeve)",
        len(continuous_joints) == 0,
        details=f"found {len(continuous_joints)} continuous articulations",
    )

    # Disc is inlined as a base visual (not a separate part).
    disc_visuals = [v for v in base.visuals if v.name == "disc_body"]
    ctx.check(
        "disc is inlined as a base visual",
        len(disc_visuals) == 1,
        details=f"found {len(disc_visuals)} disc_body visuals on base",
    )
    # Only two parts exist: base and lid.
    ctx.check(
        "exactly two parts (base and lid)",
        len(object_model.parts) == 2,
        details=f"parts={[p.name for p in object_model.parts]}",
    )

    # --- disc rests flat in the sleeve (Z position between pocket sheets) ---
    disc_aabb = ctx.part_element_world_aabb(base, elem="disc_body")
    disc_bottom = disc_aabb[0][2]
    disc_top = disc_aabb[1][2]
    expected_bottom = TRAY_FLOOR_Z + SLEEVE_THICK
    ctx.check(
        "disc rests on sleeve bottom sheet",
        abs(disc_bottom - expected_bottom) < 0.002,
        details=f"disc_bottom={disc_bottom:.4f}, expected={expected_bottom:.4f}",
    )
    ctx.check(
        "disc is flat (thickness ~= CD spec)",
        abs((disc_top - disc_bottom) - DISC_T) < 0.001,
        details=f"measured={disc_top - disc_bottom:.4f}, expected={DISC_T}",
    )

    # --- disc sits within the case footprint ---
    ctx.expect_within(
        base,
        base,
        axes="xy",
        inner_elem="disc_body",
        outer_elem="base_frame",
        margin=0.004,
        name="disc within case footprint",
    )

    # --- sleeve pocket seats in the base frame ---
    ctx.allow_overlap(
        base,
        base,
        elem_a="sleeve_pocket",
        elem_b="base_frame",
        reason="The paper sleeve pocket seats inside the clear base frame; its outer "
        "edges intentionally overlap the frame walls for a snug nested fit.",
    )
    ctx.expect_within(
        base,
        base,
        axes="xy",
        inner_elem="sleeve_pocket",
        outer_elem="base_frame",
        margin=0.004,
        name="sleeve pocket nests within base frame footprint",
    )

    # --- disc is nested inside the sleeve pocket cavity ---
    ctx.allow_overlap(
        base,
        base,
        elem_a="disc_body",
        elem_b="sleeve_pocket",
        reason="The CD slides flat into the pocket sleeve cavity; it is intentionally "
        "nested between the pocket bottom and top sheets.",
    )
    ctx.expect_overlap(
        base,
        base,
        axes="xy",
        elem_a="disc_body",
        elem_b="sleeve_pocket",
        min_overlap=0.08,
        name="disc overlaps sleeve pocket in XY (nested inside)",
    )

    return ctx.report()


object_model = build_object_model()
