from __future__ import annotations

# Red British K6 telephone booth — OPEN-SIDE variant.
#
# Coordinate convention:
#   - up is +Z; the base plinth sits on the ground at z = 0.
#   - the booth is (nearly) square in plan, centered on x = y = 0.
#   - the FRONT (+X) face is a permanently open walk-in doorway framed by
#     the corner pilasters — no door or fixed panel.
#   - the LEFT (+Y) face carries a single hinged glazed access/maintenance
#     panel that swings outward on a vertical hinge at its rear (-X) edge.
#   - the RIGHT (-Y) and BACK (-X) faces are fixed glazed window walls with
#     a red mullion grid.
#
# Root structure: the kiosk body (plinth + four corner pilasters + two fixed
# glazed walls + lower kick panels + frieze/TELEPHONE signs + crown roof) is
# the root. The single moving part is the +Y side access panel, hinged on a
# vertical axis.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
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

COLS = 3                # panes across each glazed face
ROWS = 6                # panes tall on each glazed face

PANEL_GAP = 0.005
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

    # --- lower solid kick panels ---
    # Front (+X) is the open doorway — no kick panel.
    # +Y face carries the hinged access panel — its kick panel is on that part.
    # Only -Y and -X get body kick panels.
    kick_h = WINDOW_BOTTOM - WALL_BOTTOM
    kick_zc = WALL_BOTTOM + kick_h * 0.5
    inner_w = BOX_W - 2 * POST
    inner_d = BOX_D - 2 * POST

    # -Y kick panel
    body.visual(
        Box((inner_w, 0.05, kick_h)),
        origin=Origin(xyz=(0.0, -(BOX_D * 0.5 - 0.025), kick_zc)),
        material=red,
        name="kick_y_m",
    )
    # -X kick panel (back)
    body.visual(
        Box((0.05, inner_d, kick_h)),
        origin=Origin(xyz=(-(BOX_W * 0.5 - 0.025), 0.0, kick_zc)),
        material=red,
        name="kick_back",
    )

    # --- fixed glazed window walls on -Y and -X only ---
    # (+Y is the access panel; +X is the open entrance)
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

    # ===================== ACCESS PANEL (child) ==============================
    # The +Y side face carries a single hinged glazed access/maintenance panel.
    # Hinged on a vertical axis at its rear (-X) edge; swings outward (+Y).
    # Part frame origin is at the hinge line; the leaf extends along +X.
    panel = model.part("access_panel")
    panel_w = inner_w - 2 * PANEL_GAP  # span along X
    panel_z0 = WALL_BOTTOM + PANEL_GAP
    panel_z1 = BODY_TOP - PANEL_GAP

    # Hinge at local origin; leaf extends along +X from hinge.
    leaf_cx = panel_w * 0.5  # center of leaf in local +X

    # Panel frame (thin red box, slightly behind the glass plane)
    panel.visual(
        Box((panel_w, 0.030, panel_z1 - panel_z0)),
        origin=Origin(xyz=(leaf_cx, -0.012, 0.5 * (panel_z0 + panel_z1))),
        material=red,
        name="panel_frame",
    )

    # Kick panel (lower solid section)
    panel_kick_top = WINDOW_BOTTOM - PANEL_GAP
    panel.visual(
        Box((panel_w - 0.02, 0.040, panel_kick_top - panel_z0)),
        origin=Origin(xyz=(leaf_cx, -0.005, 0.5 * (panel_z0 + panel_kick_top))),
        material=red,
        name="panel_kick",
    )

    # Glass + mullion grid (upper glazed section)
    pg_z0 = panel_kick_top
    pg_z1 = panel_z1
    pg_h = pg_z1 - pg_z0
    pg_zc = 0.5 * (pg_z0 + pg_z1)
    glass_span = panel_w - 0.04

    panel.visual(
        Box((glass_span, 0.005, pg_h)),
        origin=Origin(xyz=(leaf_cx, 0.0, pg_zc)),
        material=glass,
        name="panel_glass",
    )

    p_cols = COLS
    p_rows = ROWS - 1
    bar_y = MULLION_D * 0.5 + 0.004  # bars proud of the outer face (+Y side)
    for c in range(p_cols + 1):
        x = leaf_cx - glass_span * 0.5 + glass_span * c / p_cols
        panel.visual(
            Box((MULLION_T, MULLION_D, pg_h)),
            origin=Origin(xyz=(x, bar_y, pg_zc)),
            material=red,
            name=f"panel_vbar_{c}",
        )
    for r in range(p_rows + 1):
        z = pg_z0 + pg_h * r / p_rows
        panel.visual(
            Box((glass_span, MULLION_D, MULLION_T)),
            origin=Origin(xyz=(leaf_cx, bar_y, z)),
            material=red,
            name=f"panel_hbar_{r}",
        )

    # Dark steel pull handle on the free (+X) edge
    handle_x = panel_w - 0.02
    panel.visual(
        Box((0.025, 0.05, 0.16)),
        origin=Origin(xyz=(handle_x, 0.03, WINDOW_BOTTOM + 0.18)),
        material=steel,
        name="panel_handle",
    )

    panel.inertial = Inertial.from_geometry(
        Box((panel_w, 0.04, panel_z1 - panel_z0)),
        mass=18.0,
        origin=Origin(xyz=(leaf_cx, 0.0, 0.5 * (panel_z0 + panel_z1))),
    )

    # ===================== ARTICULATION ==================================
    # Access panel swings outward about a vertical hinge at the -X rear corner
    # of the +Y face. Leaf extends along +X from hinge; axis +Z so positive q
    # swings the free (+X) edge outward (+Y).
    hinge_x = -(BOX_W * 0.5 - POST - PANEL_GAP)
    hinge_y = BOX_D * 0.5 - GLASS_INSET  # at the glass plane
    model.articulation(
        "body_to_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=panel,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=0.0, upper=OPEN_ANGLE),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("kiosk_body")
    panel = object_model.get_part("access_panel")
    hinge = object_model.get_articulation("body_to_panel")

    ctx.check(
        "panel joint is revolute",
        str(hinge.articulation_type).endswith("REVOLUTE"),
        details=f"type={hinge.articulation_type}",
    )
    ctx.check(
        "panel hinge axis is vertical (Z)",
        abs(hinge.axis[0]) < 1e-6 and abs(hinge.axis[1]) < 1e-6 and abs(abs(hinge.axis[2]) - 1.0) < 1e-6,
        details=f"axis={hinge.axis}",
    )
    lim = hinge.motion_limits
    ctx.check(
        "panel closed at q=0 and opens to ~95 deg",
        lim is not None and abs(lim.lower) < 1e-6 and math.radians(85) <= lim.upper <= math.radians(105),
        details=f"lower={None if lim is None else lim.lower}, upper={None if lim is None else lim.upper}",
    )

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

    # Panel connected to the body at the hinge (not floating)
    ctx.expect_contact(
        body,
        panel,
        contact_tol=0.02,
        name="closed panel meets the body",
    )

    paabb = ctx.part_world_aabb(panel)
    if paabb is not None:
        ctx.check(
            "closed panel is on the +Y side",
            paabb[1][1] > BOX_D * 0.5 - 0.10,
            details=f"panel_max_y={paabb[1][1]}",
        )
        ctx.check(
            "closed panel is tall (mullioned glazing present)",
            (paabb[1][2] - paabb[0][2]) > 1.4,
            details=f"panel_height={paabb[1][2] - paabb[0][2]}",
        )

    # Decisive open-pose check: the free edge swings outward in +Y
    rest = ctx.part_world_aabb(panel)
    rest_max_y = rest[1][1] if rest else None
    with ctx.pose({hinge: OPEN_ANGLE}):
        oa = ctx.part_world_aabb(panel)
        open_max_y = oa[1][1] if oa else None
    ctx.check(
        "opening swings the panel leaf outward (+Y)",
        rest_max_y is not None and open_max_y is not None and open_max_y > rest_max_y + 0.18,
        details=f"rest_max_y={rest_max_y}, open_max_y={open_max_y}",
    )

    # The panel seats into the +Y side opening when closed (small jamb embed)
    ctx.allow_overlap(
        body,
        panel,
        reason="The closed access panel seats into the +Y side opening between the corner pilasters and frieze, a small intentional seating embed at the jamb.",
    )

    return ctx.report()


object_model = build_object_model()
