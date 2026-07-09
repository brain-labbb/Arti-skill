from __future__ import annotations

# Street electrical utility cabinet with vertical roller shutter.
#
# Reference: a free-standing roadside distribution cabinet, brushed grey steel,
# sitting on a poured concrete pedestal. The single front service door is
# replaced by a vertical roller shutter: a corrugated steel curtain of
# horizontal slats that travels UP and DOWN on a PRISMATIC joint over the front
# opening. When lowered (q=0) the curtain seals the front face; when raised
# (q=upper) the curtain retracts into a head-box housing at the top.
#
# Coordinate convention (Z-up world):
#   +X : cabinet WIDTH  (~0.62 m)
#   +Y : cabinet DEPTH  (~0.40 m); front face at -Y
#   +Z : HEIGHT; concrete plinth base at z=0, cabinet top ~1.05 m
#
# Parts / articulations:
#   - plinth (ROOT)  : concrete pedestal, base at z=0, wider than the body
#   - body           : hollow grey-steel cabinet shell (open front), drip cap,
#                      head box, guide rails, side vents, warning plate
#   - shutter        : corrugated curtain (N slats + bottom rail + edge rails),
#                      PRISMATIC along +Z
#   Static detail on body: conduit stubs at plinth, louvered side vents.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---- overall dimensions (metres) ----
W = 0.62           # width (X)
D = 0.40           # depth (Y)
BODY_H = 0.85      # cabinet shell height above plinth
PLINTH_H = 0.20    # concrete pedestal height
PLINTH_OVER = 0.06 # plinth overhang beyond body on each side
WALL = 0.012

FRONT_Y = -D / 2.0

# ---- shutter dimensions ----
N_SLATS = 22
SLAT_PITCH = 0.035       # vertical spacing (center-to-center)
SLAT_THICK = 0.032       # actual slat height (pitch - gap)
SLAT_GAP = SLAT_PITCH - SLAT_THICK  # 0.003 m visible gap between slats
SLAT_DEPTH = 0.012       # slat depth (Y direction)
GUIDE_W = 0.028          # guide rail width
SLAT_W = W - 2 * GUIDE_W - 0.006  # slat width fits between guide rails
CURTAIN_H = N_SLATS * SLAT_PITCH  # ~0.770 m

BOTTOM_RAIL_H = 0.040
BOTTOM_RAIL_DEPTH = SLAT_DEPTH + 0.006

# Edge rail dimensions (vertical strips connecting slats)
EDGE_RAIL_W = 0.016
EDGE_RAIL_H = CURTAIN_H + BOTTOM_RAIL_H

# Head box (housing at top front for retracted curtain)
HEAD_BOX_H = 0.14
HEAD_BOX_D = 0.16

# Guide rail height (full opening)
GUIDE_H = BODY_H - 2 * WALL

# Shutter travel distance (prismatic upper limit)
SHUTTER_TRAVEL = 0.72


def _hollow_box(outer, wall, *, open_face):
    """Return wall pieces for a hollow box open on one face."""
    sx, sy, sz = outer
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
    pieces = []
    pieces.append((Box((sx, wall, sz)), Origin(xyz=(0.0, hy - wall / 2.0, 0.0))))   # back
    pieces.append((Box((wall, sy, sz)), Origin(xyz=(hx - wall / 2.0, 0.0, 0.0))))   # +X
    pieces.append((Box((wall, sy, sz)), Origin(xyz=(-hx + wall / 2.0, 0.0, 0.0))))  # -X
    pieces.append((Box((sx, sy, wall)), Origin(xyz=(0.0, 0.0, hz - wall / 2.0))))   # top
    pieces.append((Box((sx, sy, wall)), Origin(xyz=(0.0, 0.0, -hz + wall / 2.0))))  # bottom
    if open_face != "-y":
        pieces.append((Box((sx, wall, sz)), Origin(xyz=(0.0, -hy + wall / 2.0, 0.0))))
    return pieces


def _make_shutter_slat(i: int):
    """Return (geometry, local_origin) for shutter slat i."""
    z_center = BOTTOM_RAIL_H + i * SLAT_PITCH + SLAT_THICK / 2.0
    return Box((SLAT_W, SLAT_DEPTH, SLAT_THICK)), Origin(xyz=(0.0, 0.0, z_center))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="distribution_cabinet_shutter")

    # ---- materials ----
    steel = model.material("grey_steel", rgba=(0.66, 0.67, 0.69, 1.0))
    steel_shutter = model.material("shutter_steel", rgba=(0.72, 0.73, 0.76, 1.0))
    concrete = model.material("concrete", rgba=(0.55, 0.54, 0.51, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.34, 0.35, 0.37, 1.0))
    label_white = model.material("label_plate", rgba=(0.90, 0.90, 0.88, 1.0))
    hazard_yellow = model.material("hazard_yellow", rgba=(0.92, 0.78, 0.10, 1.0))

    # ---- plinth (ROOT) : concrete pedestal, base at z=0 ----
    plinth = model.part("plinth")
    pw = W + 2 * PLINTH_OVER
    pd = D + 2 * PLINTH_OVER
    plinth.visual(
        Box((pw, pd, PLINTH_H)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_H / 2.0)),
        material=concrete,
        name="concrete_pedestal",
    )
    # conduit cable entries at the base
    for c, cx in enumerate((-0.14, 0.14)):
        plinth.visual(
            Cylinder(radius=0.022, length=0.06),
            origin=Origin(xyz=(cx, 0.08, PLINTH_H + 0.02)),
            material=dark_metal,
            name=f"conduit_{c}",
        )
    plinth.inertial = Inertial.from_geometry(
        Box((pw, pd, PLINTH_H)), mass=60.0,
        origin=Origin(xyz=(0.0, 0.0, PLINTH_H / 2.0)),
    )

    # ---- body : hollow grey-steel shell, open at the front ----
    body = model.part("body")
    body_cz = PLINTH_H + BODY_H / 2.0
    for i, (geom, org) in enumerate(_hollow_box((W, D, BODY_H), WALL, open_face="-y")):
        body.visual(
            geom,
            origin=Origin(xyz=(org.xyz[0], org.xyz[1], org.xyz[2] + body_cz)),
            material=steel,
            name=f"shell_{i}",
        )

    # drip cap / roof cap
    body.visual(
        Box((W + 0.04, D + 0.04, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_H + BODY_H + 0.008)),
        material=steel,
        name="drip_cap",
    )

    # head box at top front (houses retracted shutter curtain)
    hb_cz = PLINTH_H + BODY_H - HEAD_BOX_H / 2.0
    hb_cy = FRONT_Y - HEAD_BOX_D / 2.0 + WALL / 2.0
    body.visual(
        Box((W, HEAD_BOX_D, HEAD_BOX_H)),
        origin=Origin(xyz=(0.0, hb_cy, hb_cz)),
        material=steel,
        name="head_box",
    )
    # head box front face lip (visible bottom edge of the housing)
    body.visual(
        Box((W - 0.02, 0.008, 0.02)),
        origin=Origin(xyz=(0.0, FRONT_Y - HEAD_BOX_D + 0.004, PLINTH_H + BODY_H - HEAD_BOX_H + 0.01)),
        material=dark_metal,
        name="head_box_lip",
    )

    # guide rails on front vertical edges
    guide_cz = PLINTH_H + BODY_H / 2.0
    guide_cy = FRONT_Y - GUIDE_W / 2.0
    for side, sx in enumerate((-1.0, 1.0)):
        gx = sx * (W / 2.0 - GUIDE_W / 2.0)
        body.visual(
            Box((GUIDE_W, GUIDE_W, GUIDE_H)),
            origin=Origin(xyz=(gx, guide_cy, guide_cz)),
            material=dark_metal,
            name=f"guide_rail_{side}",
        )

    # side louvered vents on +X and -X walls
    vent_size = (0.18, 0.12)
    for side, sx in enumerate((-1.0, 1.0)):
        vent = SlotPatternPanelGeometry(
            vent_size,
            0.006,
            slot_size=(0.14, 0.006),
            pitch=(0.18, 0.014),
            frame=0.012,
            corner_radius=0.004,
        )
        vx = sx * (W / 2.0)
        vz = PLINTH_H + 0.25
        # Panel is in local XY; rotate so it faces ±X direction
        if sx > 0:
            rpy = (0.0, math.pi / 2.0, 0.0)
        else:
            rpy = (0.0, -math.pi / 2.0, 0.0)
        body.visual(
            mesh_from_geometry(vent, f"side_vent_{side}"),
            origin=Origin(xyz=(vx, 0.0, vz), rpy=rpy),
            material=dark_metal,
            name=f"side_vent_{side}",
        )

    # warning label plate on head box front face
    hb_front_y = FRONT_Y - HEAD_BOX_D + WALL / 2.0
    body.visual(
        Box((0.16, 0.004, 0.05)),
        origin=Origin(xyz=(0.0, hb_front_y - 0.002, hb_cz + 0.02)),
        material=label_white,
        name="warning_label",
    )
    # hazard symbol (yellow plate with implied bolt)
    body.visual(
        Box((0.09, 0.004, 0.09)),
        origin=Origin(xyz=(0.0, hb_front_y - 0.002, hb_cz - 0.02)),
        material=hazard_yellow,
        name="hazard_symbol",
    )

    body.inertial = Inertial.from_geometry(
        Box((W, D, BODY_H)), mass=40.0,
        origin=Origin(xyz=(0.0, 0.0, body_cz)),
    )

    # ---- shutter : corrugated curtain, PRISMATIC along +Z ----
    shutter = model.part("shutter")

    # bottom rail (thicker bar at curtain bottom, seals threshold)
    shutter.visual(
        Box((SLAT_W, BOTTOM_RAIL_DEPTH, BOTTOM_RAIL_H)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_RAIL_H / 2.0)),
        material=dark_metal,
        name="bottom_rail",
    )

    # corrugated slats emitted via for-i-in-range loop
    for i in range(N_SLATS):
        geom, org = _make_shutter_slat(i)
        shutter.visual(
            geom,
            origin=org,
            material=steel_shutter,
            name=f"shutter_slat_{i}",
        )

    # edge rails (vertical strips connecting all slats, ensure connectivity)
    for side, sx in enumerate((0, 1)):
        ex = sx * (SLAT_W / 2.0 - EDGE_RAIL_W / 2.0) - (SLAT_W / 2.0 - EDGE_RAIL_W / 2.0) * (1 - 2 * sx)
        # Simpler: left rail at -SLAT_W/2 + EDGE_RAIL_W/2, right at +SLAT_W/2 - EDGE_RAIL_W/2
        if sx == 0:
            ex = -SLAT_W / 2.0 + EDGE_RAIL_W / 2.0
        else:
            ex = SLAT_W / 2.0 - EDGE_RAIL_W / 2.0
        shutter.visual(
            Box((EDGE_RAIL_W, SLAT_DEPTH, EDGE_RAIL_H)),
            origin=Origin(xyz=(ex, 0.0, EDGE_RAIL_H / 2.0)),
            material=dark_metal,
            name=f"edge_rail_{side}",
        )

    shutter.inertial = Inertial.from_geometry(
        Box((SLAT_W, SLAT_DEPTH, EDGE_RAIL_H)), mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, EDGE_RAIL_H / 2.0)),
    )

    # ---- articulations ----
    # PRISMATIC: shutter travels vertically (+Z) over the front opening.
    # At q=0 curtain is fully lowered (closed); at q=upper curtain retracts up.
    # Joint origin at bottom-center of the front opening, at the front face plane.
    model.articulation(
        "body_to_shutter",
        ArticulationType.PRISMATIC,
        parent=body,
        child=shutter,
        origin=Origin(xyz=(0.0, FRONT_Y, PLINTH_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=SHUTTER_TRAVEL, effort=80.0, velocity=0.08
        ),
    )
    model.articulation(
        "plinth_to_body",
        ArticulationType.FIXED,
        parent=plinth,
        child=body,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    plinth = object_model.get_part("plinth")
    body = object_model.get_part("body")
    shutter = object_model.get_part("shutter")
    shutter_joint = object_model.get_articulation("body_to_shutter")

    # ---- plinth base at z=0 and wider than body ----
    pmn, pmx = ctx.part_world_aabb(plinth)
    ctx.check(
        "plinth base at z=0",
        abs(pmn[2]) < 1e-3,
        details=f"plinth min z={pmn[2]:.4f}",
    )
    bmn, bmx = ctx.part_world_aabb(body)
    ctx.check(
        "plinth footprint wider than body",
        (pmx[0] - pmn[0]) > (bmx[0] - bmn[0]) + 0.05,
        details=f"plinth_x={(pmx[0]-pmn[0]):.3f}, body_x={(bmx[0]-bmn[0]):.3f}",
    )

    # ---- shutter joint is PRISMATIC along vertical (+Z) axis ----
    ax = shutter_joint.axis
    ctx.check(
        "shutter slide axis is vertical (Z)",
        abs(ax[2]) > 0.99 and abs(ax[0]) < 1e-6 and abs(ax[1]) < 1e-6,
        details=f"axis={ax}",
    )
    ctx.check(
        "shutter joint is prismatic",
        str(shutter_joint.articulation_type).upper().endswith("PRISMATIC"),
        details=f"type={shutter_joint.articulation_type}",
    )

    # ---- shutter curtain spans most of the cabinet width and height ----
    shutter_ext = _ext(ctx.part_world_aabb(shutter))
    ctx.check(
        "shutter spans most of cabinet width",
        shutter_ext[0] > W * 0.75,
        details=f"shutter width extent={shutter_ext[0]:.3f}",
    )
    ctx.check(
        "shutter spans most of cabinet height (closed)",
        shutter_ext[2] > BODY_H * 0.70,
        details=f"shutter height extent={shutter_ext[2]:.3f}",
    )

    # ---- shutter slats present (corrugated curtain) ----
    slat_count = sum(
        1 for v in shutter.visuals if v.name and v.name.startswith("shutter_slat_")
    )
    ctx.check(
        "shutter has corrugated slats",
        slat_count >= N_SLATS,
        details=f"found {slat_count} slats, expected {N_SLATS}",
    )

    # ---- positive q raises shutter upward (opens the front) ----
    closed_pos = ctx.part_world_position(shutter)
    with ctx.pose({shutter_joint: SHUTTER_TRAVEL}):
        open_pos = ctx.part_world_position(shutter)
    ctx.check(
        "shutter retracts upward when opened",
        closed_pos is not None and open_pos is not None
        and open_pos[2] > closed_pos[2] + SHUTTER_TRAVEL * 0.9,
        details=f"closed_z={closed_pos[2] if closed_pos else None:.3f}, "
                f"open_z={open_pos[2] if open_pos else None:.3f}",
    )

    # ---- at closed pose, shutter covers the front opening ----
    closed_aabb = ctx.part_world_aabb(shutter)
    ctx.check(
        "closed shutter reaches plinth top (seals bottom)",
        closed_aabb[0][2] < PLINTH_H + 0.02,
        details=f"shutter min z={closed_aabb[0][2]:.4f}",
    )

    # ---- head box present on body ----
    ctx.check(
        "head box present",
        body.get_visual("head_box") is not None,
        details="body exposes head_box",
    )

    # ---- guide rails present ----
    for rn in ("guide_rail_0", "guide_rail_1"):
        ctx.check(
            f"{rn} present",
            body.get_visual(rn) is not None,
            details=f"body exposes {rn}",
        )

    # ---- side vents present ----
    for vn in ("side_vent_0", "side_vent_1"):
        ctx.check(
            f"{vn} present",
            body.get_visual(vn) is not None,
            details=f"body exposes {vn}",
        )

    # ---- conduit stubs penetrate cabinet floor (cable entry) ----
    for c in (0, 1):
        ctx.allow_overlap(
            plinth, body,
            elem_a=f"conduit_{c}",
            elem_b="shell_4",
            reason="Conduit cable stub enters the cabinet through its bottom wall.",
        )
    ctx.expect_overlap(
        plinth, body,
        axes="xy",
        elem_a="conduit_0",
        elem_b="shell_4",
        min_overlap=0.01,
        name="conduit enters cabinet floor footprint",
    )

    # ---- shutter curtain nests in the front opening and guide channels ----
    ctx.allow_overlap(
        shutter, body,
        reason="Shutter curtain slides within the guide channels and retracts "
               "into the head box housing at the top of the front opening.",
    )
    ctx.expect_overlap(
        shutter, body,
        axes="x",
        min_overlap=W * 0.5,
        name="shutter centered in body width",
    )

    return ctx.report()


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


object_model = build_object_model()
