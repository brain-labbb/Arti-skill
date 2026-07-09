from __future__ import annotations

# Red British K6 telephone booth (phone box) — bi-fold door variant.
#
# Coordinate convention:
#   - up is +Z; the base plinth sits on the ground at z = 0.
#   - the booth is (nearly) square in plan, centered on x = y = 0.
#   - the ENTRANCE is on the front face (+X). The other three faces (+Y, -Y, -X)
#     are fixed glazed windows with a red mullion grid.
#
# Root structure: the kiosk body (plinth + four corner pilasters + three glazed
# window walls + lower kick panels + the frieze band carrying the TELEPHONE
# signs + the domed crown roof with crown emblems) is the root.
#
# Bi-fold door: the front opening is split into two narrower glazed leaves that
# fold together. The outer leaf is hinged on a vertical axis at the +Y front
# corner. The inner leaf is hinged to the outer leaf at the meeting edge
# (bi-fold joint) with a mimic coupling (multiplier = -2) so the inner leaf's
# free edge stays approximately on the front face, mimicking a track-guided
# bi-fold. Both leaves carry the same glazing-bar grid.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

# ---- key dimensions (meters) -------------------------------------------------
BOX_W = 0.92            # plan width  (x, front-back footprint)
BOX_D = 0.92            # plan depth  (y, side-side footprint)
PLINTH_H = 0.10         # black base plinth height
WALL_BOTTOM = PLINTH_H
BODY_TOP = 2.00         # top of the glazed body / underside of frieze
FRIEZE_H = 0.20         # TELEPHONE frieze band height
FRIEZE_TOP = BODY_TOP + FRIEZE_H
ROOF_BASE = FRIEZE_TOP
ROOF_H = 0.36           # domed crown roof height
TOTAL_H = ROOF_BASE + ROOF_H

POST = 0.085            # corner pilaster square size
WINDOW_BOTTOM = 0.62    # bottom of the glazed area (top of lower kick panel)
GLASS_INSET = 0.022     # glass set in from the outer face
MULLION_T = 0.020       # glazing-bar (mullion) thickness on the face
MULLION_D = 0.018       # glazing-bar depth (proud of the glass)

COLS = 3                # panes across each fixed glazed face
ROWS = 6                # panes tall on each glazed face
LEAF_COLS = 2           # panes across each bi-fold leaf (narrower)

DOOR_GAP = 0.005
OPEN_ANGLE = math.radians(95.0)


def _glazed_grid(part, *, face, span, z0, z1, cols, rows, red, glass, name_prefix):
    """Translucent glass pane (inset) plus a proud red mullion grid on one wall
    face. The glass plane and the bars sit at different depths to avoid coplanar
    z-fighting."""
    height = z1 - z0
    zc = 0.5 * (z0 + z1)
    if face in ("+x", "-x"):
        sign = 1.0 if face == "+x" else -1.0
        glass_x = sign * (BOX_W * 0.5 - GLASS_INSET)
        part.visual(
            Box((0.005, span, height)),
            origin=Origin(xyz=(glass_x, 0.0, zc)),
            material=glass,
            name=f"{name_prefix}_glass",
        )
        bar_x = sign * (BOX_W * 0.5 - GLASS_INSET + MULLION_D * 0.5 + 0.004)
        for c in range(cols + 1):
            y = -span * 0.5 + span * c / cols
            part.visual(
                Box((MULLION_D, MULLION_T, height)),
                origin=Origin(xyz=(bar_x, y, zc)),
                material=red,
                name=f"{name_prefix}_vbar_{c}",
            )
        for r in range(rows + 1):
            z = z0 + height * r / rows
            part.visual(
                Box((MULLION_D, span, MULLION_T)),
                origin=Origin(xyz=(bar_x, 0.0, z)),
                material=red,
                name=f"{name_prefix}_hbar_{r}",
            )
    else:
        sign = 1.0 if face == "+y" else -1.0
        glass_y = sign * (BOX_D * 0.5 - GLASS_INSET)
        part.visual(
            Box((span, 0.005, height)),
            origin=Origin(xyz=(0.0, glass_y, zc)),
            material=glass,
            name=f"{name_prefix}_glass",
        )
        bar_y = sign * (BOX_D * 0.5 - GLASS_INSET + MULLION_D * 0.5 + 0.004)
        for c in range(cols + 1):
            x = -span * 0.5 + span * c / cols
            part.visual(
                Box((MULLION_T, MULLION_D, height)),
                origin=Origin(xyz=(x, bar_y, zc)),
                material=red,
                name=f"{name_prefix}_vbar_{c}",
            )
        for r in range(rows + 1):
            z = z0 + height * r / rows
            part.visual(
                Box((span, MULLION_D, MULLION_T)),
                origin=Origin(xyz=(0.0, bar_y, z)),
                material=red,
                name=f"{name_prefix}_hbar_{r}",
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="k6_telephone_booth")

    red = model.material("kiosk_red", rgba=(0.62, 0.09, 0.10, 1.0))
    red_dark = model.material("kiosk_red_dark", rgba=(0.48, 0.07, 0.08, 1.0))
    black = model.material("plinth_black", rgba=(0.10, 0.10, 0.11, 1.0))
    glass = model.material("glass", rgba=(0.72, 0.80, 0.82, 0.28))
    sign_white = model.material("sign_white", rgba=(0.93, 0.93, 0.90, 1.0))
    sign_text = model.material("sign_text", rgba=(0.13, 0.13, 0.14, 1.0))
    steel = model.material("steel", rgba=(0.18, 0.18, 0.20, 1.0))
    gold = model.material("crown_gold", rgba=(0.78, 0.62, 0.22, 1.0))

    # ===================== BODY (root) ==================================
    body = model.part("kiosk_body")

    # --- black base plinth ---
    body.visual(
        Box((BOX_W + 0.06, BOX_D + 0.06, PLINTH_H)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_H * 0.5)),
        material=black,
        name="plinth",
    )

    # --- four corner pilasters (full body height) ---
    corners = [
        (BOX_W * 0.5 - POST * 0.5, BOX_D * 0.5 - POST * 0.5),
        (BOX_W * 0.5 - POST * 0.5, -BOX_D * 0.5 + POST * 0.5),
        (-BOX_W * 0.5 + POST * 0.5, BOX_D * 0.5 - POST * 0.5),
        (-BOX_W * 0.5 + POST * 0.5, -BOX_D * 0.5 + POST * 0.5),
    ]
    post_h = BODY_TOP - WALL_BOTTOM
    for i, (cx, cy) in enumerate(corners):
        body.visual(
            Box((POST, POST, post_h)),
            origin=Origin(xyz=(cx, cy, WALL_BOTTOM + post_h * 0.5)),
            material=red,
            name=f"pilaster_{i}",
        )

    # --- lower solid kick panels (between plinth and window bottom) ---
    kick_h = WINDOW_BOTTOM - WALL_BOTTOM
    kick_zc = WALL_BOTTOM + kick_h * 0.5
    inner_w = BOX_W - 2 * POST
    inner_d = BOX_D - 2 * POST
    for sy in (1.0, -1.0):
        body.visual(
            Box((inner_w, 0.05, kick_h)),
            origin=Origin(xyz=(0.0, sy * (BOX_D * 0.5 - 0.025), kick_zc)),
            material=red,
            name=f"kick_y_{'p' if sy > 0 else 'm'}",
        )
    body.visual(
        Box((0.05, inner_d, kick_h)),
        origin=Origin(xyz=(-(BOX_W * 0.5 - 0.025), 0.0, kick_zc)),
        material=red,
        name="kick_back",
    )

    # --- glazed window walls on +Y, -Y, -X (front +X is the bi-fold door) ---
    _glazed_grid(body, face="+y", span=inner_w, z0=WINDOW_BOTTOM, z1=BODY_TOP,
                 cols=COLS, rows=ROWS, red=red, glass=glass, name_prefix="win_left")
    _glazed_grid(body, face="-y", span=inner_w, z0=WINDOW_BOTTOM, z1=BODY_TOP,
                 cols=COLS, rows=ROWS, red=red, glass=glass, name_prefix="win_right")
    _glazed_grid(body, face="-x", span=inner_d, z0=WINDOW_BOTTOM, z1=BODY_TOP,
                 cols=COLS, rows=ROWS, red=red, glass=glass, name_prefix="win_back")

    # --- frieze band carrying the TELEPHONE signs (wraps all four faces) ---
    body.visual(
        Box((BOX_W + 0.02, BOX_D + 0.02, FRIEZE_H)),
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP + FRIEZE_H * 0.5)),
        material=red,
        name="frieze_band",
    )
    sign_zc = BODY_TOP + FRIEZE_H * 0.5
    face_specs = [
        ("+x", (BOX_W * 0.5 + 0.012, 0.0), (0.006, 0.56, 0.12)),
        ("-x", (-(BOX_W * 0.5 + 0.012), 0.0), (0.006, 0.56, 0.12)),
        ("+y", (0.0, BOX_D * 0.5 + 0.012), (0.56, 0.006, 0.12)),
        ("-y", (0.0, -(BOX_D * 0.5 + 0.012)), (0.56, 0.006, 0.12)),
    ]
    for fc, (px, py), size in face_specs:
        body.visual(
            Box(size),
            origin=Origin(xyz=(px, py, sign_zc)),
            material=sign_white,
            name=f"sign_{fc}",
        )
        if fc in ("+x", "-x"):
            sgn = 1.0 if fc == "+x" else -1.0
            body.visual(
                Box((0.004, 0.50, 0.045)),
                origin=Origin(xyz=(px + sgn * 0.004, py, sign_zc)),
                material=sign_text,
                name=f"signtext_{fc}",
            )
        else:
            sgn = 1.0 if fc == "+y" else -1.0
            body.visual(
                Box((0.50, 0.004, 0.045)),
                origin=Origin(xyz=(px, py + sgn * 0.004, sign_zc)),
                material=sign_text,
                name=f"signtext_{fc}",
            )

    # --- domed (barrel-vault) crown roof: stacked tapering rounded boxes ---
    dome_steps = 11
    body.visual(
        Box((BOX_W + 0.07, BOX_D + 0.07, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, ROOF_BASE + 0.025)),
        material=red,
        name="roof_cornice",
    )
    for s in range(dome_steps):
        t0 = s / dome_steps
        t1 = (s + 1) / dome_steps
        zc = ROOF_BASE + 0.05 + (ROOF_H - 0.05) * 0.5 * (t0 + t1)
        f = math.cos(0.5 * (t0 + t1) * math.pi * 0.5)
        w = BOX_W * (0.42 + 0.58 * f)
        d = BOX_D * (0.42 + 0.58 * f)
        h = (ROOF_H - 0.05) / dome_steps
        body.visual(
            Box((w, d, h * 1.02)),
            origin=Origin(xyz=(0.0, 0.0, zc)),
            material=red if s % 2 == 0 else red_dark,
            name=f"roof_step_{s}",
        )
    # crown emblem greeble on each of the four dome faces
    emblem_z = ROOF_BASE + ROOF_H * 0.55
    emblem_specs = [
        ("+x", (BOX_W * 0.30, 0.0), (0.05, 0.10, 0.05)),
        ("-x", (-BOX_W * 0.30, 0.0), (0.05, 0.10, 0.05)),
        ("+y", (0.0, BOX_D * 0.30), (0.10, 0.05, 0.05)),
        ("-y", (0.0, -BOX_D * 0.30), (0.10, 0.05, 0.05)),
    ]
    for fc, (px, py), size in emblem_specs:
        body.visual(
            Box(size),
            origin=Origin(xyz=(px, py, emblem_z)),
            material=gold,
            name=f"crown_emblem_{fc}",
        )

    body.inertial = Inertial.from_geometry(
        Box((BOX_W, BOX_D, TOTAL_H)),
        mass=250.0,
        origin=Origin(xyz=(0.0, 0.0, TOTAL_H * 0.5)),
    )

    # ===================== BI-FOLD DOOR ================================
    # Front (+X) face split into two narrower glazed leaves that fold together.
    # The outer leaf is hinged on a vertical axis at the +Y front corner.
    # The inner leaf is hinged to the outer leaf at the meeting edge (bi-fold
    # joint) with a mimic coupling so both leaves fold in concert.
    #
    # Each leaf is authored in a local frame with origin at its hinge axis;
    # the leaf extends along -Y from that origin.

    leaf_w = (inner_d - 3 * DOOR_GAP) / 2  # gap at each end + gap between leaves

    leaf_z0 = WALL_BOTTOM + DOOR_GAP
    leaf_z1 = BODY_TOP - DOOR_GAP
    leaf_h = leaf_z1 - leaf_z0
    leaf_zc = 0.5 * (leaf_z0 + leaf_z1)

    hinge_x = BOX_W * 0.5 - 0.03    # body-frame x of the hinge axis
    hinge_y = BOX_D * 0.5 - POST - DOOR_GAP  # body-frame y of the +Y hinge corner

    leaf_kick_top = WINDOW_BOTTOM - DOOR_GAP
    dg_z0 = leaf_kick_top
    dg_z1 = leaf_z1
    dg_h = dg_z1 - dg_z0
    dg_zc = 0.5 * (dg_z0 + dg_z1)
    leaf_glass_w = leaf_w - 0.04
    d_rows = ROWS - 1

    def _build_leaf(leaf_part, *, prefix, has_handle):
        """Add frame, kick panel, glass, and mullion grid to one bi-fold leaf.
        The leaf is authored with origin at its hinge axis, extending along -Y."""
        cy = -leaf_w * 0.5  # center Y of the leaf in local frame

        # Red frame (slightly recessed from the face)
        leaf_part.visual(
            Box((0.030, leaf_w, leaf_h)),
            origin=Origin(xyz=(-0.012, cy, leaf_zc)),
            material=red,
            name=f"{prefix}_frame",
        )
        # Lower solid kick panel
        leaf_part.visual(
            Box((0.040, leaf_w - 0.02, leaf_kick_top - leaf_z0)),
            origin=Origin(xyz=(0.0, cy, 0.5 * (leaf_z0 + leaf_kick_top))),
            material=red,
            name=f"{prefix}_kick_panel",
        )
        # Translucent glass pane
        leaf_part.visual(
            Box((0.005, leaf_glass_w, dg_h)),
            origin=Origin(xyz=(-0.004, cy, dg_zc)),
            material=glass,
            name=f"{prefix}_glass",
        )
        # Mullion grid (vertical bars + horizontal bars)
        bar_x = 0.012
        for c in range(LEAF_COLS + 1):
            y = cy - leaf_glass_w * 0.5 + leaf_glass_w * c / LEAF_COLS
            leaf_part.visual(
                Box((MULLION_D, MULLION_T, dg_h)),
                origin=Origin(xyz=(bar_x, y, dg_zc)),
                material=red,
                name=f"{prefix}_vbar_{c}",
            )
        for r in range(d_rows + 1):
            z = dg_z0 + dg_h * r / d_rows
            leaf_part.visual(
                Box((MULLION_D, leaf_glass_w, MULLION_T)),
                origin=Origin(xyz=(bar_x, cy, z)),
                material=red,
                name=f"{prefix}_hbar_{r}",
            )
        # Dark steel pull handle on the free (-Y) edge
        if has_handle:
            leaf_part.visual(
                Box((0.05, 0.025, 0.16)),
                origin=Origin(xyz=(0.03, -leaf_w + 0.02, WINDOW_BOTTOM + 0.18)),
                material=steel,
                name=f"{prefix}_handle",
            )

    # --- Outer leaf (hinged at the +Y front corner) ---
    outer_leaf = model.part("outer_leaf")
    _build_leaf(outer_leaf, prefix="outer", has_handle=False)
    outer_leaf.inertial = Inertial.from_geometry(
        Box((0.04, leaf_w, leaf_h)),
        mass=9.0,
        origin=Origin(xyz=(0.0, -leaf_w * 0.5, leaf_zc)),
    )

    # --- Inner leaf (hinged to outer leaf at the meeting edge) ---
    inner_leaf = model.part("inner_leaf")
    _build_leaf(inner_leaf, prefix="inner", has_handle=True)
    inner_leaf.inertial = Inertial.from_geometry(
        Box((0.04, leaf_w, leaf_h)),
        mass=9.0,
        origin=Origin(xyz=(0.0, -leaf_w * 0.5, leaf_zc)),
    )

    # ===================== ARTICULATIONS ==================================
    # Outer leaf: swings open about a vertical hinge at the +Y front corner.
    # Leaf extends along -Y; axis +Z so positive q swings the free edge outward (+X).
    model.articulation(
        "body_to_outer_leaf",
        ArticulationType.REVOLUTE,
        parent=body,
        child=outer_leaf,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=0.0, upper=OPEN_ANGLE),
    )

    # Inner leaf: bi-fold joint at the outer leaf's free edge.
    # Mimic with multiplier = -2 makes the inner leaf fold so its free edge
    # stays approximately on the front face (track-guided bi-fold behavior).
    # At q=0 both leaves are flush; as the outer opens by θ, the inner rotates
    # by -2θ relative to the outer, producing a V-fold with inner absolute = -θ.
    model.articulation(
        "outer_to_inner_leaf",
        ArticulationType.REVOLUTE,
        parent=outer_leaf,
        child=inner_leaf,
        origin=Origin(xyz=(0.0, -leaf_w, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0, velocity=1.5,
            lower=-2.0 * OPEN_ANGLE, upper=0.0,
        ),
        mimic=Mimic(joint="body_to_outer_leaf", multiplier=-2.0, offset=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("kiosk_body")
    outer_leaf = object_model.get_part("outer_leaf")
    inner_leaf = object_model.get_part("inner_leaf")
    main_hinge = object_model.get_articulation("body_to_outer_leaf")
    fold_hinge = object_model.get_articulation("outer_to_inner_leaf")

    # --- joint type and axis checks ---
    ctx.check(
        "outer leaf joint is revolute",
        str(main_hinge.articulation_type).endswith("REVOLUTE"),
        details=f"type={main_hinge.articulation_type}",
    )
    ctx.check(
        "outer leaf hinge axis is vertical (Z)",
        abs(main_hinge.axis[0]) < 1e-6 and abs(main_hinge.axis[1]) < 1e-6 and abs(abs(main_hinge.axis[2]) - 1.0) < 1e-6,
        details=f"axis={main_hinge.axis}",
    )
    lim = main_hinge.motion_limits
    ctx.check(
        "outer leaf closed at q=0 and opens to ~95 deg",
        lim is not None and abs(lim.lower) < 1e-6 and math.radians(85) <= lim.upper <= math.radians(105),
        details=f"lower={None if lim is None else lim.lower}, upper={None if lim is None else lim.upper}",
    )

    ctx.check(
        "bi-fold joint is a mimic follower",
        fold_hinge.mimic is not None and fold_hinge.mimic.joint == "body_to_outer_leaf",
        details=f"mimic={fold_hinge.mimic}",
    )
    ctx.check(
        "bi-fold mimic multiplier is -2 (track-guided fold)",
        fold_hinge.mimic is not None and abs(fold_hinge.mimic.multiplier - (-2.0)) < 1e-6,
        details=f"multiplier={None if fold_hinge.mimic is None else fold_hinge.mimic.multiplier}",
    )

    # --- booth scale checks ---
    baabb = ctx.part_world_aabb(body)
    ctx.check(
        "kiosk base rests at z=0",
        baabb is not None and abs(baabb[0][2]) < 1e-3,
        details=f"body_min_z={None if baabb is None else baabb[0][2]}",
    )
    ctx.check(
        "kiosk is roughly 2.5 m tall",
        baabb is not None and 2.4 < baabb[1][2] < 2.8,
        details=f"body_top_z={None if baabb is None else baabb[1][2]}",
    )

    # --- closed position: both leaves on front face, in contact with body ---
    ctx.expect_contact(
        body,
        outer_leaf,
        contact_tol=0.02,
        name="closed outer leaf meets the body",
    )
    ctx.expect_contact(
        body,
        inner_leaf,
        contact_tol=0.02,
        name="closed inner leaf meets the body",
    )

    outer_aabb = ctx.part_world_aabb(outer_leaf)
    inner_aabb = ctx.part_world_aabb(inner_leaf)
    if outer_aabb is not None:
        ctx.check(
            "closed outer leaf is on the front (+X) side",
            outer_aabb[1][0] > BOX_W * 0.5 - 0.10,
            details=f"outer_max_x={outer_aabb[1][0]}",
        )
        ctx.check(
            "closed outer leaf is tall (mullioned glazing present)",
            (outer_aabb[1][2] - outer_aabb[0][2]) > 1.0,
            details=f"outer_height={outer_aabb[1][2] - outer_aabb[0][2]}",
        )
    if inner_aabb is not None:
        ctx.check(
            "closed inner leaf is on the front (+X) side",
            inner_aabb[1][0] > BOX_W * 0.5 - 0.10,
            details=f"inner_max_x={inner_aabb[1][0]}",
        )
        ctx.check(
            "closed inner leaf is tall (mullioned glazing present)",
            (inner_aabb[1][2] - inner_aabb[0][2]) > 1.0,
            details=f"inner_height={inner_aabb[1][2] - inner_aabb[0][2]}",
        )

    # --- decisive open-pose check: outer leaf swings outward (+X) ---
    rest_outer_max_x = outer_aabb[1][0] if outer_aabb else None
    with ctx.pose({main_hinge: OPEN_ANGLE}):
        open_outer = ctx.part_world_aabb(outer_leaf)
        open_outer_max_x = open_outer[1][0] if open_outer else None
    ctx.check(
        "opening swings the outer leaf outward (+X)",
        rest_outer_max_x is not None and open_outer_max_x is not None and open_outer_max_x > rest_outer_max_x + 0.10,
        details=f"rest_max_x={rest_outer_max_x}, open_max_x={open_outer_max_x}",
    )

    # --- both leaves seat into the front opening when closed (small jamb embed) ---
    ctx.allow_overlap(
        body,
        outer_leaf,
        reason="The closed outer leaf seats into the front opening between the corner pilasters, a small intentional seating embed at the jamb.",
    )
    ctx.allow_overlap(
        body,
        inner_leaf,
        reason="The closed inner leaf seats into the front opening between the corner pilasters, a small intentional seating embed at the jamb.",
    )
    # The two leaves meet at the bi-fold joint with a small embed at the meeting stile.
    ctx.allow_overlap(
        outer_leaf,
        inner_leaf,
        reason="The inner leaf is hinged to the outer leaf at the bi-fold meeting stile; a small local embed at the shared hinge edge is intentional.",
    )

    return ctx.report()


object_model = build_object_model()
