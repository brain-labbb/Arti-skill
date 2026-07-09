from __future__ import annotations

# Red British K6 telephone booth – modern wide variant.
#
# Coordinate convention:
#   - up is +Z; the base plinth sits on the ground at z = 0.
#   - the booth is rectangular in plan, centered on x = y = 0.
#   - the DOOR is on the front face (+X). The other three faces (+Y, -Y, -X)
#     are fixed glazed windows with a red mullion grid.
#
# Changes from parent K6:
#   - FLAT modern slab roof (overhanging cap) instead of domed crown.
#   - DENSE 4-by-8 fine-pane glazing grid, loop-emitted as individual panes.
#   - WIDER two-person footprint (1.50 m side-to-side, 1.10 m front-back).
#
# Root structure: the kiosk body (plinth + four corner pilasters + three glazed
# window walls + lower kick panels + the frieze band carrying the TELEPHONE
# signs + the flat overhanging slab roof) is the root. The single moving part
# is the glazed door, hinged on a VERTICAL axis (+Z) at one front corner.

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
BOX_W = 1.10            # plan depth  (x, front-back footprint)
BOX_D = 1.50            # plan width  (y, side-side footprint, wider for two)
PLINTH_H = 0.10         # black base plinth height
WALL_BOTTOM = PLINTH_H
BODY_TOP = 2.20         # top of the glazed body / underside of frieze
FRIEZE_H = 0.20         # TELEPHONE frieze band height
FRIEZE_TOP = BODY_TOP + FRIEZE_H
ROOF_BASE = FRIEZE_TOP
ROOF_H = 0.12           # flat slab roof thickness
TOTAL_H = ROOF_BASE + ROOF_H  # 2.52 m

POST = 0.085            # corner pilaster square size
WINDOW_BOTTOM = 0.62    # bottom of the glazed area (top of lower kick panel)
GLASS_INSET = 0.022     # glass set in from the outer face
MULLION_T = 0.020       # glazing-bar (mullion) thickness on the face
MULLION_D = 0.018       # glazing-bar depth (proud of the glass)

COLS = 4                # panes across each glazed face (dense)
ROWS = 8                # panes tall on each glazed face (dense)

DOOR_GAP = 0.005
OPEN_ANGLE = math.radians(95.0)


def _glazed_grid(part, *, face, span, z0, z1, cols, rows, red, glass, name_prefix):
    """Translucent individual glass panes (inset) plus a proud red mullion grid
    on one wall face. Each pane is emitted in a loop as pane_{idx}. Mullion bars
    are emitted as vbar_{c} and hbar_{r}."""
    height = z1 - z0
    pane_w = span / cols - MULLION_T
    pane_h = height / rows - MULLION_T

    if face in ("+x", "-x"):
        sign = 1.0 if face == "+x" else -1.0
        glass_x = sign * (BOX_W * 0.5 - GLASS_INSET)
        bar_x = sign * (BOX_W * 0.5 - GLASS_INSET + MULLION_D * 0.5 + 0.004)

        # individual panes
        idx = 0
        for c in range(cols):
            y = -span * 0.5 + span * (c + 0.5) / cols
            for r in range(rows):
                z = z0 + height * (r + 0.5) / rows
                part.visual(
                    Box((0.005, pane_w, pane_h)),
                    origin=Origin(xyz=(glass_x, y, z)),
                    material=glass,
                    name=f"{name_prefix}_pane_{idx}",
                )
                idx += 1

        # vertical mullion bars
        for c in range(cols + 1):
            y = -span * 0.5 + span * c / cols
            part.visual(
                Box((MULLION_D, MULLION_T, height)),
                origin=Origin(xyz=(bar_x, y, 0.5 * (z0 + z1))),
                material=red,
                name=f"{name_prefix}_vbar_{c}",
            )
        # horizontal mullion bars
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
        bar_y = sign * (BOX_D * 0.5 - GLASS_INSET + MULLION_D * 0.5 + 0.004)

        # individual panes
        idx = 0
        for c in range(cols):
            x = -span * 0.5 + span * (c + 0.5) / cols
            for r in range(rows):
                z = z0 + height * (r + 0.5) / rows
                part.visual(
                    Box((pane_w, 0.005, pane_h)),
                    origin=Origin(xyz=(x, glass_y, z)),
                    material=glass,
                    name=f"{name_prefix}_pane_{idx}",
                )
                idx += 1

        # vertical mullion bars
        for c in range(cols + 1):
            x = -span * 0.5 + span * c / cols
            part.visual(
                Box((MULLION_T, MULLION_D, height)),
                origin=Origin(xyz=(x, bar_y, 0.5 * (z0 + z1))),
                material=red,
                name=f"{name_prefix}_vbar_{c}",
            )
        # horizontal mullion bars
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

    # --- glazed window walls on +Y, -Y, -X (front +X is the door) ---
    # Dense 4x8 grid, loop-emitted individual panes
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
    # Scale sign widths to the wider booth faces
    sign_w_x = inner_d * 0.75   # sign width on +X/-X faces (spans Y)
    sign_w_y = inner_w * 0.75   # sign width on +Y/-Y faces (spans X)
    text_w_x = sign_w_x * 0.90
    text_w_y = sign_w_y * 0.90
    face_specs = [
        ("+x", (BOX_W * 0.5 + 0.012, 0.0), (0.006, sign_w_x, 0.12)),
        ("-x", (-(BOX_W * 0.5 + 0.012), 0.0), (0.006, sign_w_x, 0.12)),
        ("+y", (0.0, BOX_D * 0.5 + 0.012), (sign_w_y, 0.006, 0.12)),
        ("-y", (0.0, -(BOX_D * 0.5 + 0.012)), (sign_w_y, 0.006, 0.12)),
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
                Box((0.004, text_w_x, 0.045)),
                origin=Origin(xyz=(px + sgn * 0.004, py, sign_zc)),
                material=sign_text,
                name=f"signtext_{fc}",
            )
        else:
            sgn = 1.0 if fc == "+y" else -1.0
            body.visual(
                Box((text_w_y, 0.004, 0.045)),
                origin=Origin(xyz=(px, py + sgn * 0.004, sign_zc)),
                material=sign_text,
                name=f"signtext_{fc}",
            )

    # --- FLAT modern slab roof: overhanging cap ---
    roof_overhang = 0.06   # overhang beyond body on each side
    body.visual(
        Box((BOX_W + 2 * roof_overhang, BOX_D + 2 * roof_overhang, ROOF_H)),
        origin=Origin(xyz=(0.0, 0.0, ROOF_BASE + ROOF_H * 0.5)),
        material=red_dark,
        name="roof_slab",
    )
    # thin fascia edge strip (slightly darker outline at bottom of slab)
    body.visual(
        Box((BOX_W + 2 * roof_overhang + 0.01, BOX_D + 2 * roof_overhang + 0.01, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, ROOF_BASE + 0.0125)),
        material=red,
        name="roof_fascia",
    )

    body.inertial = Inertial.from_geometry(
        Box((BOX_W, BOX_D, TOTAL_H)),
        mass=320.0,
        origin=Origin(xyz=(0.0, 0.0, TOTAL_H * 0.5)),
    )

    # ===================== DOOR (child) ==================================
    # Front (+X) face, hinged on a vertical axis at the +Y front corner. The
    # door is authored in a local frame with the hinge at local origin and the
    # leaf extending along -Y across the front opening.
    door = model.part("door")
    door_w = inner_d - 2 * DOOR_GAP
    door_z0 = WALL_BOTTOM + DOOR_GAP
    door_z1 = BODY_TOP - DOOR_GAP
    hinge_x = BOX_W * 0.5 - 0.03
    door_face_x = 0.0
    leaf_cy = -door_w * 0.5

    # door frame (red)
    door.visual(
        Box((0.030, door_w, door_z1 - door_z0)),
        origin=Origin(xyz=(door_face_x - 0.012, leaf_cy, 0.5 * (door_z0 + door_z1))),
        material=red,
        name="door_frame",
    )
    # door kick panel (solid red lower section)
    door_kick_top = WINDOW_BOTTOM - DOOR_GAP
    door.visual(
        Box((0.040, door_w - 0.02, door_kick_top - door_z0)),
        origin=Origin(xyz=(door_face_x, leaf_cy, 0.5 * (door_z0 + door_kick_top))),
        material=red,
        name="door_kick_panel",
    )
    # door glazing: individual panes + mullion grid
    dg_z0 = door_kick_top
    dg_z1 = door_z1
    dg_h = dg_z1 - dg_z0
    dg_zc = 0.5 * (dg_z0 + dg_z1)
    glass_w = door_w - 0.04
    bar_x = door_face_x + 0.012

    d_cols = COLS
    d_rows = ROWS
    d_pane_w = glass_w / d_cols - MULLION_T
    d_pane_h = dg_h / d_rows - MULLION_T

    # individual door panes (loop-emitted)
    d_idx = 0
    for c in range(d_cols):
        y = leaf_cy - glass_w * 0.5 + glass_w * (c + 0.5) / d_cols
        for r in range(d_rows):
            z = dg_z0 + dg_h * (r + 0.5) / d_rows
            door.visual(
                Box((0.005, d_pane_w, d_pane_h)),
                origin=Origin(xyz=(door_face_x - 0.004, y, z)),
                material=glass,
                name=f"door_pane_{d_idx}",
            )
            d_idx += 1

    # door vertical mullion bars
    for c in range(d_cols + 1):
        y = leaf_cy - glass_w * 0.5 + glass_w * c / d_cols
        door.visual(
            Box((MULLION_D, MULLION_T, dg_h)),
            origin=Origin(xyz=(bar_x, y, dg_zc)),
            material=red,
            name=f"door_vbar_{c}",
        )
    # door horizontal mullion bars
    for r in range(d_rows + 1):
        z = dg_z0 + dg_h * r / d_rows
        door.visual(
            Box((MULLION_D, glass_w, MULLION_T)),
            origin=Origin(xyz=(bar_x, leaf_cy, z)),
            material=red,
            name=f"door_hbar_{r}",
        )

    # dark steel pull handle on the latch (free, -Y) edge
    handle_y = -door_w + 0.02
    door.visual(
        Box((0.05, 0.025, 0.16)),
        origin=Origin(xyz=(door_face_x + 0.03, handle_y, WINDOW_BOTTOM + 0.18)),
        material=steel,
        name="door_handle",
    )

    door.inertial = Inertial.from_geometry(
        Box((0.04, door_w, door_z1 - door_z0)),
        mass=22.0,
        origin=Origin(xyz=(door_face_x, leaf_cy, 0.5 * (door_z0 + door_z1))),
    )

    # ===================== ARTICULATION ==================================
    # Door swings open about a vertical hinge at the +Y front corner. Leaf
    # extends along -Y; axis +Z so positive q swings the free edge outward (+X).
    hinge_y = BOX_D * 0.5 - POST - DOOR_GAP
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=0.0, upper=OPEN_ANGLE),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("kiosk_body")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("body_to_door")

    ctx.check(
        "door joint is revolute",
        str(hinge.articulation_type).endswith("REVOLUTE"),
        details=f"type={hinge.articulation_type}",
    )
    ctx.check(
        "door hinge axis is vertical (Z)",
        abs(hinge.axis[0]) < 1e-6 and abs(hinge.axis[1]) < 1e-6 and abs(abs(hinge.axis[2]) - 1.0) < 1e-6,
        details=f"axis={hinge.axis}",
    )
    lim = hinge.motion_limits
    ctx.check(
        "door closed at q=0 and opens to ~95 deg",
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
        "kiosk is roughly 2.4-2.8 m tall",
        baabb is not None and 2.4 < baabb[1][2] < 2.8,
        details=f"body_top_z={None if baabb is None else baabb[1][2]}",
    )

    # Wider footprint check: side-to-side (Y) should be > 1.2 m
    ctx.check(
        "kiosk has wide two-person footprint (Y width > 1.2 m)",
        baabb is not None and (baabb[1][1] - baabb[0][1]) > 1.2,
        details=f"y_width={None if baabb is None else baabb[1][1] - baabb[0][1]}",
    )

    # door connected to the body at the hinge (not floating)
    ctx.expect_contact(
        body,
        door,
        contact_tol=0.02,
        name="closed door meets the body",
    )

    daabb = ctx.part_world_aabb(door)
    if daabb is not None:
        ctx.check(
            "closed door is on the front (+X) side",
            daabb[1][0] > BOX_W * 0.5 - 0.10,
            details=f"door_max_x={daabb[1][0]}",
        )
        ctx.check(
            "closed door is tall (mullioned glazing present)",
            (daabb[1][2] - daabb[0][2]) > 1.4,
            details=f"door_height={daabb[1][2] - daabb[0][2]}",
        )

    # decisive open-pose check: the free edge swings outward in +X
    rest = ctx.part_world_aabb(door)
    rest_max_x = rest[1][0] if rest else None
    with ctx.pose({hinge: OPEN_ANGLE}):
        oa = ctx.part_world_aabb(door)
        open_max_x = oa[1][0] if oa else None
    ctx.check(
        "opening swings the door leaf outward (+X)",
        rest_max_x is not None and open_max_x is not None and open_max_x > rest_max_x + 0.18,
        details=f"rest_max_x={rest_max_x}, open_max_x={open_max_x}",
    )

    # the door seats into the front opening when closed (small jamb embed)
    ctx.allow_overlap(
        body,
        door,
        reason="The closed door seats into the front opening between the corner pilasters and frieze, a small intentional seating embed at the jamb.",
    )

    return ctx.report()


object_model = build_object_model()
