from __future__ import annotations

# Hexagonal street/park trash can with swing-lid.
#
# A faceted six-sided prism body (six flat panels at 60-degree intervals),
# a hexagonal floor plate, and a silver hexagonal hood/cap on top. The hood
# has a genuine open oval hole through it -- the trash mouth is a real void
# into the hollow bin, not a sealed face. An oval swing flap with a pivot
# rod rocks in the opening on a horizontal Y axis (REVOLUTE).
#
# Coordinate convention:
#   +Z up, body footprint on z=0.
#   Hex vertices at 0, 60, 120, ... degrees from +X.
#   Panel outward normals at 30, 90, 150, ... degrees.
#
# Root structure: body is root; hood is FIXED to body; flap is REVOLUTE on hood.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---- key dimensions (meters) -------------------------------------------------
HEX_R = 0.19               # body circumradius (center to vertex)
BODY_H = 0.70              # body height (mouth rim z)
WALL_T = 0.005             # panel wall thickness
FLOOR_T = 0.008            # floor plate thickness

HEX_APOTHEM = HEX_R * math.cos(math.pi / 6.0)
HEX_SIDE = HEX_R            # regular hexagon: side length = circumradius

HOOD_R = HEX_R + 0.010     # hood circumradius (slight overhang)
HOOD_T = 0.010             # hood plate thickness
SKIRT_H = 0.025            # hood skirt height (sleeves over body)
SKIRT_INNER_R = HEX_R + 0.004  # skirt inner hex (clearance over body panels)

FLAP_A = 0.070              # flap semi-axis along X
FLAP_B = 0.045              # flap semi-axis along Y
FLAP_T = 0.006              # flap panel thickness
HOLE_A = FLAP_A + 0.003    # hood opening semi-axis X (3 mm clearance)
HOLE_B = FLAP_B + 0.003    # hood opening semi-axis Y

PIVOT_ROD_R = 0.003         # pivot rod radius
PIVOT_ROD_HALF = HOLE_B + 0.008  # rod half-length (extends into hood plate)


# ---- profile helpers ---------------------------------------------------------

def _hex_profile(R):
    """Regular hexagon 2D profile, vertices at 0, 60, 120, ... degrees."""
    return [
        (R * math.cos(2.0 * math.pi * i / 6.0),
         R * math.sin(2.0 * math.pi * i / 6.0))
        for i in range(6)
    ]


def _oval_profile(a, b, n=32):
    """Ellipse 2D profile with semi-axes *a* (X) and *b* (Y)."""
    return [
        (a * math.cos(2.0 * math.pi * i / n),
         b * math.sin(2.0 * math.pi * i / n))
        for i in range(n)
    ]


# ---- shared panel geometry helpers -------------------------------------------

def _panel_dimensions():
    """Shared geometry helper: one body panel box (width, depth, height)."""
    return (HEX_SIDE, WALL_T, BODY_H)


def _panel_origin(i):
    """Shared geometry helper: placement origin for body panel *i* (0..5).

    Each panel is centred at the hex apothem distance, rotated so its thin
    axis is radial and its wide axis is tangent.
    """
    theta = (i * 60 + 30) * math.pi / 180.0
    cx = HEX_APOTHEM * math.cos(theta)
    cy = HEX_APOTHEM * math.sin(theta)
    return Origin(
        xyz=(cx, cy, BODY_H / 2.0),
        rpy=(0.0, 0.0, theta + math.pi / 2.0),
    )


# ---- build -------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hex_swing_lid_trash_can")

    black = model.material("bin_black", rgba=(0.08, 0.08, 0.09, 1.0))
    silver = model.material("hood_silver", rgba=(0.72, 0.74, 0.77, 1.0))
    flap_metal = model.material("flap_metal", rgba=(0.58, 0.60, 0.63, 1.0))

    # ---- body (root): 6 panels + hexagonal floor ----------------------------
    body = model.part("body")

    panel_size = _panel_dimensions()
    for i in range(6):
        body.visual(
            Box(panel_size),
            origin=_panel_origin(i),
            material=black,
            name=f"panel_{i}",
        )

    # Hexagonal floor plate (closes the bottom of the hollow body)
    floor_geo = ExtrudeGeometry.from_z0(
        _hex_profile(HEX_R), FLOOR_T, cap=True,
    )
    body.visual(
        mesh_from_geometry(floor_geo, "floor_plate"),
        material=black,
        name="floor_plate",
    )

    # ---- hood (FIXED to body top) -------------------------------------------
    hood = model.part("hood")

    # Hood plate: hexagonal flat plate with a genuine oval through-hole.
    # ExtrudeWithHolesGeometry cuts a real void -- no back-face quad seals
    # the trash mouth.
    plate_geo = ExtrudeWithHolesGeometry(
        _hex_profile(HOOD_R),
        [_oval_profile(HOLE_A, HOLE_B, 32)],
        HOOD_T,
        cap=True,
        center=False,
        closed=True,
    )
    hood.visual(
        mesh_from_geometry(plate_geo, "hood_plate"),
        material=silver,
        name="hood_plate",
        origin=Origin(xyz=(0.0, 0.0, BODY_H)),
    )

    # Hood skirt: short hexagonal ring that sleeves down over the body rim,
    # as a real bin cap seats on the can.
    skirt_geo = ExtrudeWithHolesGeometry(
        _hex_profile(HOOD_R),
        [_hex_profile(SKIRT_INNER_R)],
        SKIRT_H,
        cap=True,
        center=False,
        closed=True,
    )
    hood.visual(
        mesh_from_geometry(skirt_geo, "hood_skirt"),
        material=silver,
        name="hood_skirt",
        origin=Origin(xyz=(0.0, 0.0, BODY_H - SKIRT_H)),
    )

    model.articulation(
        "body_to_hood",
        ArticulationType.FIXED,
        parent=body,
        child=hood,
        origin=Origin(),
    )

    # ---- swing flap (REVOLUTE on hood) --------------------------------------
    flap = model.part("flap")

    # Oval flap panel, centred at local origin.
    flap_geo = ExtrudeGeometry.centered(
        _oval_profile(FLAP_A, FLAP_B, 32),
        FLAP_T,
        cap=True,
    )
    flap.visual(
        mesh_from_geometry(flap_geo, "flap_panel"),
        material=flap_metal,
        name="flap_panel",
    )

    # Pivot rod along Y through the flap centre, spanning the hood opening
    # and embedding into the hood plate on both sides (real hinge axis).
    rod_len = 2.0 * PIVOT_ROD_HALF
    flap.visual(
        Cylinder(radius=PIVOT_ROD_R, length=rod_len),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=flap_metal,
        name="pivot_rod",
    )

    # REVOLUTE articulation: pivot at hood mid-plane, axis along Y.
    # Positive q tips the +X edge down into the bin (opens the mouth).
    pivot_z = BODY_H + HOOD_T / 2.0
    model.articulation(
        "hood_to_flap",
        ArticulationType.REVOLUTE,
        parent=hood,
        child=flap,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=3.0,
            lower=-1.5,
            upper=1.5,
        ),
    )

    return model


# ---- tests -------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    hood = object_model.get_part("hood")
    flap = object_model.get_part("flap")
    flap_joint = object_model.get_articulation("hood_to_flap")

    # --- six body panels emitted via loop ------------------------------------
    for i in range(6):
        ctx.check(
            f"panel_{i} exists on body",
            body.get_visual(f"panel_{i}") is not None,
            details=f"visual panel_{i} not found",
        )

    # --- hexagonal body proportions ------------------------------------------
    baabb = ctx.part_world_aabb(body)
    bext = (
        baabb[1][0] - baabb[0][0],
        baabb[1][1] - baabb[0][1],
        baabb[1][2] - baabb[0][2],
    )
    ctx.check(
        "body height ~0.70 m",
        abs(bext[2] - BODY_H) < 0.02,
        details=f"body_z={bext[2]:.3f}",
    )
    ctx.check(
        "hex body ~0.33-0.42 m across",
        0.30 < bext[0] < 0.44 and 0.30 < bext[1] < 0.44,
        details=f"body_xy=({bext[0]:.3f}, {bext[1]:.3f})",
    )
    ctx.check(
        "body base sits at z ~ 0",
        abs(baabb[0][2]) < 0.012,
        details=f"min_z={baabb[0][2]:.4f}",
    )

    # --- hood sits on body top -----------------------------------------------
    haabb = ctx.part_world_aabb(hood)
    h_z_ext = haabb[1][2] - haabb[0][2]
    ctx.check(
        "hood bottom at body top minus skirt",
        abs(haabb[0][2] - (BODY_H - SKIRT_H)) < 0.015,
        details=f"hood_min_z={haabb[0][2]:.4f}",
    )
    ctx.check(
        "hood total Z = plate + skirt",
        abs(h_z_ext - (HOOD_T + SKIRT_H)) < 0.008,
        details=f"hood_z={h_z_ext:.4f}",
    )

    # Hood skirt sleeves over body (intentional overlap)
    ctx.allow_overlap(
        body, hood,
        reason="Hood skirt sleeves down over the body rim as a real bin cap seats on the can.",
    )
    ctx.expect_overlap(
        hood, body, axes="xy",
        min_overlap=0.25,
        name="hood covers body mouth footprint",
    )

    # --- genuine open mouth: hood plate is thin, frame extends past flap -----
    h_x_ext = haabb[1][0] - haabb[0][0]
    faabb = ctx.part_world_aabb(flap)
    fext = (faabb[1][0] - faabb[0][0], faabb[1][1] - faabb[0][1])
    ctx.check(
        "hood frame extends well beyond flap (opening frame exists)",
        h_x_ext > fext[0] + 0.10,
        details=f"hood_x={h_x_ext:.3f}, flap_x={fext[0]:.3f}",
    )

    # --- swing flap ----------------------------------------------------------
    ctx.check(
        "flap X dimension ~0.14 m",
        abs(fext[0] - 2.0 * FLAP_A) < 0.02,
        details=f"flap_x={fext[0]:.3f}",
    )
    ctx.check(
        "flap centred on hood opening",
        abs((faabb[0][0] + faabb[1][0]) / 2.0) < 0.01
        and abs((faabb[0][1] + faabb[1][1]) / 2.0) < 0.01,
        details="flap off-centre",
    )

    # Pivot rod embeds into hood plate (intentional hinge overlap)
    ctx.allow_overlap(
        flap, hood,
        elem_a="pivot_rod",
        elem_b="hood_plate",
        reason="Pivot rod ends embed into the hood plate as a real swing-flap hinge.",
    )
    ctx.expect_overlap(
        flap, hood, axes="y",
        elem_a="pivot_rod",
        elem_b="hood_plate",
        min_overlap=0.080,
        name="pivot rod spans the hood opening",
    )

    # --- primary articulation: revolute swing --------------------------------
    ctx.check(
        "flap joint is REVOLUTE",
        str(flap_joint.articulation_type).endswith("REVOLUTE"),
        details=f"type={flap_joint.articulation_type}",
    )
    ctx.check(
        "swing axis is lateral (Y)",
        abs(flap_joint.axis[1]) > 0.99,
        details=f"axis={flap_joint.axis}",
    )
    lim = flap_joint.motion_limits
    ctx.check(
        "swing travel >= ~85 deg each way",
        lim.upper >= 1.4 and lim.lower <= -1.4,
        details=f"lower={lim.lower}, upper={lim.upper}",
    )

    # Rocker behaviour: positive q tips one edge down into the bin
    rest_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({flap_joint: 1.2}):
        open_aabb = ctx.part_world_aabb(flap)
    ctx.check(
        "swing opens: flap edge drops below closed position",
        open_aabb[0][2] < rest_aabb[0][2] - 0.02,
        details=f"rest_min_z={rest_aabb[0][2]:.4f}, open_min_z={open_aabb[0][2]:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
