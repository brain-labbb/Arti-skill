from __future__ import annotations

# Red British K6 telephone booth — temple-pediment bi-fold variant.
#
# Coordinate convention:
#   - up is +Z; the base plinth sits on the ground at z = 0.
#   - the booth is (nearly) square in plan, centered on x = y = 0.
#   - the BI-FOLD DOOR is on the front face (+X). The other three faces
#     (+Y, -Y, -X) are fixed glazed windows with a sparse 2-by-4 mullion grid.
#
# Changes from the standard K6 parent:
#   - TRIANGULAR PEDIMENT gable roof (temple front) replaces the domed crown.
#   - SPARSE 2-by-4 large-pane glazing grid (loop-emitted).
#   - BI-FOLD front entrance: two leaves hinged on vertical axes. The jamb
#     leaf swings outward; the outer leaf folds back via a mimic joint.
#
# Root: kiosk_body (plinth + pilasters + 3 glazed walls + kick panels +
# frieze + pediment). Children: leaf_jamb, leaf_outer.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---- key dimensions (meters) ------------------------------------------------
BOX_W = 0.92            # plan width  (X, front-back)
BOX_D = 0.92            # plan depth  (Y, side-side)
PLINTH_H = 0.10         # black base plinth height
WALL_BOTTOM = PLINTH_H
BODY_TOP = 2.00         # top of the glazed body / underside of frieze
FRIEZE_H = 0.20         # TELEPHONE frieze band height
FRIEZE_TOP = BODY_TOP + FRIEZE_H
ROOF_BASE = FRIEZE_TOP
CORNICE_H = 0.05        # cornice slab under the pediment
GABLE_H = 0.31          # triangular pediment height above cornice
ROOF_H = CORNICE_H + GABLE_H  # total roof zone height (0.36)
TOTAL_H = ROOF_BASE + ROOF_H  # overall booth height (~2.56 m)

POST = 0.085            # corner pilaster square size
WINDOW_BOTTOM = 0.62    # bottom of glazed area (top of kick panel)
GLASS_INSET = 0.022     # glass set-back from outer face
MULLION_T = 0.020       # glazing-bar thickness on face
MULLION_D = 0.018       # glazing-bar depth (proud of glass)

COLS = 2                # sparse 2-by-4 grid
ROWS = 4

DOOR_GAP = 0.005
OPEN_ANGLE = math.radians(80.0)

# Derived spans
INNER_W = BOX_W - 2 * POST
INNER_D = BOX_D - 2 * POST
LEAF_W = (INNER_D - 2 * DOOR_GAP) / 2.0  # each bi-fold leaf width


# ---- shared helpers ---------------------------------------------------------

def _glazed_grid(part, *, face, span, z0, z1, cols, rows, red, glass,
                 name_prefix):
    """Translucent glass pane plus a proud red mullion grid on one wall face."""
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


def _build_door_leaf(part, *, leaf_w, z0, z1, kick_top, red, glass, steel,
                     name_prefix, add_handle=False):
    """One bi-fold leaf: frame, kick panel, glass pane, mullion grid, handle."""
    face_x = 0.0
    leaf_cy = -leaf_w * 0.5
    h = z1 - z0

    # Red frame panel
    part.visual(
        Box((0.030, leaf_w, h)),
        origin=Origin(xyz=(face_x - 0.012, leaf_cy, 0.5 * (z0 + z1))),
        material=red,
        name=f"{name_prefix}_frame",
    )

    # Lower kick panel
    kick_h = kick_top - z0
    part.visual(
        Box((0.040, leaf_w - 0.02, kick_h)),
        origin=Origin(xyz=(face_x, leaf_cy, z0 + kick_h * 0.5)),
        material=red,
        name=f"{name_prefix}_kick",
    )

    # Glass pane
    glass_h = z1 - kick_top
    glass_w = leaf_w - 0.04
    glass_zc = kick_top + glass_h * 0.5
    part.visual(
        Box((0.005, glass_w, glass_h)),
        origin=Origin(xyz=(face_x - 0.004, leaf_cy, glass_zc)),
        material=glass,
        name=f"{name_prefix}_glass",
    )

    # Mullion grid: 1 column, (ROWS-1) = 3 rows per leaf
    d_cols = 1
    d_rows = ROWS - 1
    bar_x = face_x + 0.012
    for c in range(d_cols + 1):
        y = leaf_cy - glass_w * 0.5 + glass_w * c / d_cols
        part.visual(
            Box((MULLION_D, MULLION_T, glass_h)),
            origin=Origin(xyz=(bar_x, y, glass_zc)),
            material=red,
            name=f"{name_prefix}_vbar_{c}",
        )
    for r in range(d_rows + 1):
        z = kick_top + glass_h * r / d_rows
        part.visual(
            Box((MULLION_D, glass_w, MULLION_T)),
            origin=Origin(xyz=(bar_x, leaf_cy, z)),
            material=red,
            name=f"{name_prefix}_hbar_{r}",
        )

    if add_handle:
        handle_y = -leaf_w + 0.03
        part.visual(
            Box((0.05, 0.025, 0.16)),
            origin=Origin(xyz=(face_x + 0.03, handle_y, WINDOW_BOTTOM + 0.18)),
            material=steel,
            name=f"{name_prefix}_handle",
        )


# ---- build ------------------------------------------------------------------

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

    # ===================== BODY (root) ===================================
    body = model.part("kiosk_body")

    # --- black base plinth ---
    body.visual(
        Box((BOX_W + 0.06, BOX_D + 0.06, PLINTH_H)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_H * 0.5)),
        material=black,
        name="plinth",
    )

    # --- four corner pilasters ---
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

    # --- lower solid kick panels (+Y, -Y, -X; front +X is door opening) ---
    kick_h = WINDOW_BOTTOM - WALL_BOTTOM
    kick_zc = WALL_BOTTOM + kick_h * 0.5
    for sy in (1.0, -1.0):
        body.visual(
            Box((INNER_W, 0.05, kick_h)),
            origin=Origin(xyz=(0.0, sy * (BOX_D * 0.5 - 0.025), kick_zc)),
            material=red,
            name=f"kick_y_{'p' if sy > 0 else 'm'}",
        )
    body.visual(
        Box((0.05, INNER_D, kick_h)),
        origin=Origin(xyz=(-(BOX_W * 0.5 - 0.025), 0.0, kick_zc)),
        material=red,
        name="kick_back",
    )

    # --- glazed window walls on +Y, -Y, -X (2-by-4 grid) ---
    _glazed_grid(body, face="+y", span=INNER_W, z0=WINDOW_BOTTOM, z1=BODY_TOP,
                 cols=COLS, rows=ROWS, red=red, glass=glass,
                 name_prefix="win_left")
    _glazed_grid(body, face="-y", span=INNER_W, z0=WINDOW_BOTTOM, z1=BODY_TOP,
                 cols=COLS, rows=ROWS, red=red, glass=glass,
                 name_prefix="win_right")
    _glazed_grid(body, face="-x", span=INNER_D, z0=WINDOW_BOTTOM, z1=BODY_TOP,
                 cols=COLS, rows=ROWS, red=red, glass=glass,
                 name_prefix="win_back")

    # --- frieze band with TELEPHONE signs on all four faces ---
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

    # --- TRIANGULAR PEDIMENT gable roof ---
    # Cornice slab (flat slab at the top of the frieze)
    body.visual(
        Box((BOX_W + 0.07, BOX_D + 0.07, CORNICE_H)),
        origin=Origin(xyz=(0.0, 0.0, ROOF_BASE + CORNICE_H * 0.5)),
        material=red,
        name="roof_cornice",
    )

    # Gable prism: triangular cross-section in YZ, extruded along X.
    # ExtrudeGeometry makes a triangle in XY extruded along Z (centered).
    # rotate_y(-pi/2) maps (x,y,z) -> (-z, y, x), placing the triangle in YZ
    # and the extrusion along X.
    half_span = (BOX_D + 0.07) * 0.5
    gable_length = BOX_W + 0.07
    triangle_profile = [
        (0.0, -half_span),
        (0.0, half_span),
        (GABLE_H, 0.0),
    ]
    gable_mesh = ExtrudeGeometry(triangle_profile, gable_length,
                                 cap=True, center=True)
    gable_mesh.rotate_y(-math.pi / 2.0)
    body.visual(
        mesh_from_geometry(gable_mesh, "gable_roof"),
        origin=Origin(xyz=(0.0, 0.0, ROOF_BASE + CORNICE_H)),
        material=red,
        name="gable_roof",
    )

    # Ridge cap along the peak line
    body.visual(
        Box((gable_length + 0.02, 0.06, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, ROOF_BASE + CORNICE_H + GABLE_H + 0.015)),
        material=red_dark,
        name="ridge_cap",
    )

    # Tympanum emblems (gold medallions on front and back pediment faces)
    emblem_x = gable_length * 0.5 + 0.006
    emblem_z = ROOF_BASE + CORNICE_H + GABLE_H * 0.35
    for sx in (1.0, -1.0):
        body.visual(
            Box((0.012, 0.22, 0.14)),
            origin=Origin(xyz=(sx * emblem_x, 0.0, emblem_z)),
            material=gold,
            name=f"pediment_emblem_{'p' if sx > 0 else 'm'}x",
        )

    body.inertial = Inertial.from_geometry(
        Box((BOX_W, BOX_D, TOTAL_H)),
        mass=250.0,
        origin=Origin(xyz=(0.0, 0.0, TOTAL_H * 0.5)),
    )

    # ===================== BI-FOLD DOOR LEAVES ============================
    hinge_x = BOX_W * 0.5 - 0.03
    hinge_y = BOX_D * 0.5 - POST - DOOR_GAP
    door_z0 = WALL_BOTTOM + DOOR_GAP
    door_z1 = BODY_TOP - DOOR_GAP
    door_kick_top = WINDOW_BOTTOM - DOOR_GAP

    # --- Jamb leaf (inner leaf, hinged to body at +Y front corner) ---
    leaf_jamb = model.part("leaf_jamb")
    _build_door_leaf(
        leaf_jamb,
        leaf_w=LEAF_W,
        z0=door_z0,
        z1=door_z1,
        kick_top=door_kick_top,
        red=red,
        glass=glass,
        steel=steel,
        name_prefix="jamb",
        add_handle=True,
    )
    leaf_jamb.inertial = Inertial.from_geometry(
        Box((0.04, LEAF_W, door_z1 - door_z0)),
        mass=9.0,
        origin=Origin(xyz=(0.0, -LEAF_W * 0.5, 0.5 * (door_z0 + door_z1))),
    )

    # --- Outer leaf (hinged to jamb leaf at the meeting stile) ---
    leaf_outer = model.part("leaf_outer")
    _build_door_leaf(
        leaf_outer,
        leaf_w=LEAF_W,
        z0=door_z0,
        z1=door_z1,
        kick_top=door_kick_top,
        red=red,
        glass=glass,
        steel=steel,
        name_prefix="outer",
        add_handle=False,
    )
    leaf_outer.inertial = Inertial.from_geometry(
        Box((0.04, LEAF_W, door_z1 - door_z0)),
        mass=9.0,
        origin=Origin(xyz=(0.0, -LEAF_W * 0.5, 0.5 * (door_z0 + door_z1))),
    )

    # ===================== ARTICULATIONS ==================================
    # Jamb leaf swings outward on vertical hinge at +Y front corner.
    # Leaf extends along -Y; axis +Z so positive q swings free edge in +X.
    model.articulation(
        "body_to_jamb",
        ArticulationType.REVOLUTE,
        parent=body,
        child=leaf_jamb,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0, velocity=1.5, lower=0.0, upper=OPEN_ANGLE,
        ),
    )

    # Outer leaf folds back against the jamb leaf.
    # Hinge at the free edge of the jamb leaf (local y = -LEAF_W).
    # Mimic: q_outer = -2 * q_jamb, so at full open the outer leaf folds flat.
    model.articulation(
        "jamb_to_outer",
        ArticulationType.REVOLUTE,
        parent=leaf_jamb,
        child=leaf_outer,
        origin=Origin(xyz=(0.0, -LEAF_W, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=-2.0 * OPEN_ANGLE,
            upper=0.0,
        ),
        mimic=Mimic(joint="body_to_jamb", multiplier=-2.0, offset=0.0),
    )

    return model


# ---- tests ------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("kiosk_body")
    jamb = object_model.get_part("leaf_jamb")
    outer = object_model.get_part("leaf_outer")
    hinge = object_model.get_articulation("body_to_jamb")
    fold = object_model.get_articulation("jamb_to_outer")

    # --- joint structure ---
    ctx.check(
        "jamb joint is revolute",
        str(hinge.articulation_type).endswith("REVOLUTE"),
        details=f"type={hinge.articulation_type}",
    )
    ctx.check(
        "jamb hinge axis is vertical (Z)",
        abs(hinge.axis[0]) < 1e-6
        and abs(hinge.axis[1]) < 1e-6
        and abs(abs(hinge.axis[2]) - 1.0) < 1e-6,
        details=f"axis={hinge.axis}",
    )
    ctx.check(
        "fold joint is revolute with mimic",
        str(fold.articulation_type).endswith("REVOLUTE")
        and fold.mimic is not None,
        details=f"type={fold.articulation_type}, mimic={fold.mimic}",
    )

    lim = hinge.motion_limits
    ctx.check(
        "jamb leaf closed at q=0 and opens to ~80 deg",
        lim is not None
        and abs(lim.lower) < 1e-6
        and math.radians(70) <= lim.upper <= math.radians(90),
        details=f"lower={None if lim is None else lim.lower}, "
                f"upper={None if lim is None else lim.upper}",
    )

    # --- booth dimensions ---
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

    # --- connectivity ---
    ctx.expect_contact(
        body, jamb, contact_tol=0.02,
        name="closed jamb leaf meets the body",
    )
    ctx.expect_contact(
        jamb, outer, contact_tol=0.02,
        name="closed outer leaf meets the jamb leaf",
    )

    # --- closed pose: leaves on the front (+X) side ---
    jaabb = ctx.part_world_aabb(jamb)
    if jaabb is not None:
        ctx.check(
            "closed jamb leaf is on the front (+X) side",
            jaabb[1][0] > BOX_W * 0.5 - 0.10,
            details=f"jamb_max_x={jaabb[1][0]}",
        )

    # --- decisive open-pose check: jamb leaf swings outward in +X ---
    rest = ctx.part_world_aabb(jamb)
    rest_max_x = rest[1][0] if rest else None
    with ctx.pose({hinge: OPEN_ANGLE}):
        oa = ctx.part_world_aabb(jamb)
        open_max_x = oa[1][0] if oa else None
    ctx.check(
        "opening swings the jamb leaf outward (+X)",
        rest_max_x is not None
        and open_max_x is not None
        and open_max_x > rest_max_x + 0.15,
        details=f"rest_max_x={rest_max_x}, open_max_x={open_max_x}",
    )

    # --- outer leaf folds back (its min_x reaches toward the booth) ---
    with ctx.pose({hinge: OPEN_ANGLE}):
        outer_oa = ctx.part_world_aabb(outer)
        outer_min_x = outer_oa[0][0] if outer_oa else None
    ctx.check(
        "outer leaf folds back toward the booth body",
        open_max_x is not None
        and outer_min_x is not None
        and (open_max_x - outer_min_x) > 0.20,
        details=f"open_jamb_max_x={open_max_x}, "
                f"folded_outer_min_x={outer_min_x}",
    )

    # --- scoped overlap allowances for closed-door seating embed ---
    ctx.allow_overlap(
        body, jamb,
        reason="Closed jamb leaf seats into the front opening between the "
               "corner pilasters and frieze, a small intentional seating embed.",
    )
    ctx.allow_overlap(
        body, outer,
        reason="Closed outer leaf seats into the front opening between the "
               "corner pilasters and frieze, a small intentional seating embed.",
    )

    return ctx.report()


object_model = build_object_model()
