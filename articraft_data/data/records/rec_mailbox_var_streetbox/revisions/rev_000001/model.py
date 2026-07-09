from __future__ import annotations

# Rounded "streetbox" US Mail collection box.
#
# A curbside rounded-front street mail collection box whose body is a smooth
# rounded cabinet: the front face and top are blended into one continuous convex
# curved sheet-metal shell (a rounded-shoulder dome that sweeps from the
# vertical front face up and over the crown).  The shell is genuinely hollow
# with an outer skin, dark interior, side walls, floor and back wall.  A
# pull-down hopper deposit flap on a bottom hinge (REVOLUTE) opens to reveal
# the dark chute interior.
#
# Coordinate convention:
#   +Z up, box rests on the floor at z = 0
#   +X front (deposit flap side), -X back
#   Y centred at y = 0, width along Y
#
# Root structure: body (root) → cap (FIXED crown ridge) and flap (REVOLUTE).

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- key dimensions (metres) ------------------------------------------------
BODY_W = 0.40               # width  (Y)
BODY_D = 0.34               # depth  (X)
WALL_T = 0.014              # shell wall thickness
HALF_W = BODY_W / 2.0

FRONT_X = BODY_D / 2.0      #  0.17
BACK_X = -BODY_D / 2.0      # -0.17

# dome profile
Z_SHOULDER = 0.55           # vertical wall ends, dome curve begins
Z_CROWN = 0.85              # top of dome

# deposit flap
LOWER_TOP = 0.30
FLAP_BOT = LOWER_TOP + 0.02
FLAP_H = 0.22
FLAP_TOP = FLAP_BOT + FLAP_H
HEADER_BOT = FLAP_TOP + 0.005
HEADER_H = 0.075
FLAP_W = 0.30
FLAP_T = 0.020
JAMB_W = (BODY_W - FLAP_W) / 2.0


# ---------------------------------------------------------------------------
# CadQuery geometry helpers
# ---------------------------------------------------------------------------

def _build_shell() -> object:
    """Hollow rounded streetbox shell with deposit opening cut."""
    # Outer profile in XZ at y=0, extruded in -Y direction, then centered.
    outer = (
        cq.Workplane("XZ")
        .moveTo(FRONT_X, 0.0)
        .lineTo(FRONT_X, Z_SHOULDER)
        .threePointArc((0.0, Z_CROWN), (BACK_X, Z_SHOULDER))
        .lineTo(BACK_X, 0.0)
        .close()
        .extrude(BODY_W)
        .translate((0.0, HALF_W, 0.0))  # center: y from -HALF_W to +HALF_W
    )

    # Inner cavity (offset inward by WALL_T).
    inner = (
        cq.Workplane("XZ")
        .moveTo(FRONT_X - WALL_T, WALL_T)
        .lineTo(FRONT_X - WALL_T, Z_SHOULDER)
        .threePointArc((0.0, Z_CROWN - WALL_T), (BACK_X + WALL_T, Z_SHOULDER))
        .lineTo(BACK_X + WALL_T, WALL_T)
        .close()
        .extrude(BODY_W - 2.0 * WALL_T)
        .translate((0.0, HALF_W - WALL_T, 0.0))  # center inner cavity
    )

    shell = outer.cut(inner)

    # Cut deposit opening in the front wall.
    op_z = (FLAP_BOT + FLAP_TOP) / 2.0
    cutter = (
        cq.Workplane("XY")
        .transformed(offset=(FRONT_X, 0.0, op_z))
        .box(WALL_T * 5.0, FLAP_W, FLAP_H)
    )
    shell = shell.cut(cutter)

    return shell


def _build_cap_finial() -> object:
    """Small decorative ridge along the dome crown."""
    ridge = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, Z_CROWN + 0.004))
        .box(0.05, BODY_W * 0.88, 0.016)
    )
    return ridge


# ---------------------------------------------------------------------------
# Object model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rounded_streetbox")

    blue = model.material("blue", rgba=(0.30, 0.42, 0.55, 1.0))
    blue_dk = model.material("blue_dk", rgba=(0.22, 0.33, 0.45, 1.0))
    rust = model.material("rust", rgba=(0.55, 0.40, 0.30, 1.0))
    rust_dk = model.material("rust_dk", rgba=(0.45, 0.31, 0.22, 1.0))
    cap_blue = model.material("cap_blue", rgba=(0.27, 0.38, 0.50, 1.0))
    interior = model.material("interior", rgba=(0.06, 0.07, 0.10, 1.0))
    card = model.material("card", rgba=(0.82, 0.78, 0.66, 1.0))
    brass = model.material("brass", rgba=(0.62, 0.55, 0.32, 1.0))
    letters_mat = model.material("letters", rgba=(0.18, 0.22, 0.28, 1.0))

    # -------------------------------------------------------- BODY (root) ----
    body = model.part("body")

    # Main hollow curved shell
    shell_cq = _build_shell()
    body.visual(
        mesh_from_cadquery(shell_cq, "body_shell"),
        material=blue,
        name="body_shell",
    )

    # Dark interior back liner (visible through deposit opening)
    inner_w = BODY_W - 2.0 * WALL_T
    inner_h = Z_SHOULDER - 2.0 * WALL_T
    body.visual(
        Box((0.004, inner_w, inner_h)),
        origin=Origin(xyz=(BACK_X + WALL_T + 0.002, 0.0, WALL_T + inner_h / 2.0)),
        material=interior,
        name="interior_back_liner",
    )
    # Interior floor liner
    body.visual(
        Box((BODY_D - 2.0 * WALL_T, inner_w, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, WALL_T + 0.002)),
        material=interior,
        name="interior_floor",
    )

    # Base skirt / foot ring
    body.visual(
        Box((BODY_D + 0.03, BODY_W + 0.03, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=blue_dk,
        name="base_skirt",
    )

    # Front jamb pillars flanking the deposit opening
    for i, sgn in enumerate((1.0, -1.0)):
        body.visual(
            Box((WALL_T + 0.004, JAMB_W, FLAP_H)),
            origin=Origin(
                xyz=(
                    FRONT_X - WALL_T / 2.0 + 0.002,
                    sgn * (BODY_W / 2.0 - JAMB_W / 2.0),
                    (FLAP_BOT + FLAP_TOP) / 2.0,
                )
            ),
            material=blue_dk,
            name=f"front_jamb_{i}",
        )

    # Front lower panel with raised frame + schedule-card holder
    body.visual(
        Box((0.012, FLAP_W + 0.03, LOWER_TOP - 0.06)),
        origin=Origin(xyz=(FRONT_X + 0.004, 0.0, (LOWER_TOP - 0.06) / 2.0 + 0.05)),
        material=blue_dk,
        name="lower_panel_frame",
    )
    body.visual(
        Box((0.006, 0.13, 0.10)),
        origin=Origin(xyz=(FRONT_X + 0.010, 0.0, 0.18)),
        material=card,
        name="schedule_card",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(FRONT_X + 0.012, 0.0, 0.085), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="lower_knob",
    )

    # LETTERS header band above the flap opening
    band_back = FRONT_X - 0.004
    band_front = FRONT_X + 0.010
    body.visual(
        Box((band_front - band_back, FLAP_W + 0.03, HEADER_H)),
        origin=Origin(
            xyz=((band_back + band_front) / 2.0, 0.0, HEADER_BOT + HEADER_H / 2.0)
        ),
        material=rust,
        name="letters_band",
    )
    body.visual(
        Box((0.006, FLAP_W - 0.02, 0.028)),
        origin=Origin(xyz=(band_front + 0.002, 0.0, HEADER_BOT + HEADER_H / 2.0)),
        material=letters_mat,
        name="letters_text",
    )

    # Side "U.S. MAIL" raised plate (mounted on shell side, slightly inset)
    body.visual(
        Box((0.10, 0.006, 0.16)),
        origin=Origin(xyz=(0.04, HALF_W + 0.001, 0.40)),
        material=blue_dk,
        name="side_plate",
    )

    # Back reinforcing ribs
    for i, yy in enumerate((-0.11, 0.0, 0.11)):
        body.visual(
            Box((0.012, 0.03, Z_SHOULDER * 0.7)),
            origin=Origin(xyz=(BACK_X - 0.004, yy, Z_SHOULDER * 0.40)),
            material=blue_dk,
            name=f"back_rib_{i}",
        )

    # --------------------------------------------------------- CAP (ridge) ---
    cap = model.part("cap")
    cap_cq = _build_cap_finial()
    cap.visual(
        mesh_from_cadquery(cap_cq, "cap_ridge"),
        material=cap_blue,
        name="cap_ridge",
    )

    # ------------------------------------------------------------- FLAP -----
    flap = model.part("flap")
    flap.visual(
        Box((FLAP_T, FLAP_W, FLAP_H)),
        origin=Origin(xyz=(0.0, 0.0, FLAP_H / 2.0)),
        material=rust_dk,
        name="flap_panel",
    )
    flap.visual(
        Box((0.006, FLAP_W + 0.02, 0.018)),
        origin=Origin(xyz=(FLAP_T / 2.0 + 0.003, 0.0, FLAP_H - 0.012)),
        material=rust,
        name="flap_top_rim",
    )
    flap.visual(
        Cylinder(radius=0.011, length=FLAP_W * 0.5),
        origin=Origin(
            xyz=(FLAP_T / 2.0 + 0.012, 0.0, FLAP_H - 0.03),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="flap_handle",
    )

    # ------------------------------------------------------- ARTICULATIONS --
    model.articulation(
        "body_to_cap",
        ArticulationType.FIXED,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(FRONT_X + FLAP_T / 2.0, 0.0, FLAP_BOT)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=0.0, upper=1.3),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    cap = object_model.get_part("cap")
    flap = object_model.get_part("flap")
    flap_joint = object_model.get_articulation("body_to_flap")

    # Interior liner bonded to inner face of back wall.
    ctx.allow_overlap(
        body, body,
        elem_a="interior_back_liner", elem_b="body_shell",
        reason="Interior liner sits just inside the shell back wall.",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="interior_floor", elem_b="body_shell",
        reason="Interior floor liner sits just inside the shell floor.",
    )
    
    # Cap ridge nests into the dome crown.
    ctx.allow_overlap(
        cap, body,
        elem_a="cap_ridge", elem_b="body_shell",
        reason="Cap ridge sits on and nests into the dome crown surface.",
    )
    
    # Front decorative elements (jambs, panels, LETTERS band) are mounted on the shell.
    for elem in ["front_jamb_0", "front_jamb_1", "lower_panel_frame", "schedule_card", 
                  "lower_knob", "letters_band", "letters_text"]:
        ctx.allow_overlap(
            body, body,
            elem_a=elem, elem_b="body_shell",
            reason=f"{elem} is mounted on and overlaps with the shell surface.",
        )
    
    # Side plate mounted on shell side.
    ctx.allow_overlap(
        body, body,
        elem_a="side_plate", elem_b="body_shell",
        reason="Side plate is mounted on the shell side surface.",
    )
    
    # Flap sits inside the deposit opening when closed.
    ctx.allow_overlap(
        body, flap,
        elem_a="body_shell", elem_b="flap_panel",
        reason="Flap panel nests inside the deposit opening cut in the shell.",
    )
    ctx.expect_overlap(
        flap, body, axes="yz", min_overlap=0.10,
        name="flap is within the body footprint (seated in opening)",
    )

    # --- flap joint is bottom-hinge revolute about horizontal Y ---------------
    ctx.check(
        "flap joint is revolute",
        str(flap_joint.articulation_type).upper().endswith("REVOLUTE"),
        details=f"type={flap_joint.articulation_type}",
    )
    ax = flap_joint.axis
    ctx.check(
        "flap hinge axis is horizontal Y",
        abs(abs(ax[1]) - 1.0) < 1e-6 and abs(ax[0]) < 1e-6 and abs(ax[2]) < 1e-6,
        details=f"axis={ax}",
    )

    # --- box rests on the floor at z ~ 0 -------------------------------------
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "box base rests on floor",
        body_aabb is not None and abs(body_aabb[0][2]) < 0.01,
        details=f"min_z={None if body_aabb is None else body_aabb[0][2]}",
    )

    # --- realistic proportions -----------------------------------------------
    if body_aabb is not None:
        bh = body_aabb[1][2] - body_aabb[0][2]
        bw = body_aabb[1][1] - body_aabb[0][1]
        ctx.check(
            "body height is realistic (dome ≥ 0.80 m)",
            bh > 0.80,
            details=f"height={bh}",
        )
        ctx.check(
            "body width is realistic (~0.40 m)",
            0.34 < bw < 0.50,
            details=f"width={bw}",
        )

    # --- ROUNDED DOME: body extends above shoulder height at the crown -------
    body_shell_aabb = ctx.part_element_world_aabb(body, elem="body_shell")
    if body_shell_aabb is not None:
        ctx.check(
            "dome crown extends well above shoulder height",
            body_shell_aabb[1][2] > Z_SHOULDER + 0.20,
            details=f"shell_max_z={body_shell_aabb[1][2]}",
        )

    # --- HOLLOW INTERIOR visible through deposit opening ---------------------
    back_aabb = ctx.part_element_world_aabb(body, elem="interior_back_liner")
    if back_aabb is not None:
        depth = FRONT_X - back_aabb[1][0]
        ctx.check(
            "interior chute is deep (hollow, not solid)",
            depth > 0.22,
            details=f"front_to_liner_depth={depth}",
        )

    # --- cap ridge sits at the dome crown ------------------------------------
    cap_aabb = ctx.part_world_aabb(cap)
    if cap_aabb is not None and body_shell_aabb is not None:
        ctx.check(
            "cap ridge is at the dome crown",
            cap_aabb[1][2] >= body_shell_aabb[1][2] - 0.02,
            details=f"cap_top={cap_aabb[1][2]}, shell_top={body_shell_aabb[1][2]}",
        )
    ctx.expect_overlap(
        cap, body, axes="xy", min_overlap=0.04,
        name="cap ridge seats over the body footprint",
    )

    # --- closed flap covers the deposit opening ------------------------------
    ctx.expect_overlap(
        flap, body, axes="yz", min_overlap=0.10,
        name="closed flap covers the deposit opening",
    )
    ctx.expect_gap(
        flap, body, axis="x",
        positive_elem="flap_panel", negative_elem="body_shell",
        min_gap=-0.030, max_gap=0.025,
        name="closed flap seats within the deposit opening",
    )

    # --- LETTERS header is above the flap ------------------------------------
    band_aabb = ctx.part_element_world_aabb(body, elem="letters_band")
    flap_aabb = ctx.part_world_aabb(flap)
    if band_aabb is not None and flap_aabb is not None:
        ctx.check(
            "LETTERS band sits above the deposit flap",
            band_aabb[0][2] >= flap_aabb[1][2] - 0.02,
            details=f"band_bot={band_aabb[0][2]}, flap_top={flap_aabb[1][2]}",
        )

    # --- opened flap swings outward (+X) and free edge drops -----------------
    rest_flap_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({flap_joint: 1.2}):
        open_flap_aabb = ctx.part_world_aabb(flap)
    ctx.check(
        "flap opens outward in +X",
        rest_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][0] > rest_flap_aabb[1][0] + 0.04,
        details=f"rest_maxX={rest_flap_aabb[1][0] if rest_flap_aabb else None}, "
                f"open_maxX={open_flap_aabb[1][0] if open_flap_aabb else None}",
    )
    ctx.check(
        "flap free edge drops when open",
        rest_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][2] < rest_flap_aabb[1][2] - 0.04,
        details=f"rest_topZ={rest_flap_aabb[1][2] if rest_flap_aabb else None}, "
                f"open_topZ={open_flap_aabb[1][2] if open_flap_aabb else None}",
    )

    return ctx.report()
