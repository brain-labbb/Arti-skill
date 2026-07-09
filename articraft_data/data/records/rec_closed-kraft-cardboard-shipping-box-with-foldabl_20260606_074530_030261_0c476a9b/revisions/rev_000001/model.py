from __future__ import annotations

# Closed kraft cardboard shipping box (corrugated cube, ~0.25 m).
# Frame: box centered on the Z axis, footprint centered at x=y=0, bottom at z=0,
#   top rim at z=S. Walls face +X (east), -X (west), +Y (north), -Y (south).
# Articulations (four top flaps, each REVOLUTE about its top edge, fold open ~0-90 deg):
#   - flap_n / flap_s : the two LONG outer flaps, hinged on the north / south top
#     edges, hinge axis along X. Closed, they lie flat reaching toward center (-Y / +Y).
#   - flap_e / flap_w : the two SHORT inner flaps, hinged on the east / west top
#     edges, hinge axis along Y. Closed, they lie flat reaching toward center (-X / +X).
# Printed "FRAGIL"/handling icons rendered as a subtle darker decal panel on a side wall.

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

S = 0.25  # outer cube side
WALL = 0.004  # corrugated cardboard wall thickness
INNER = S - 2.0 * WALL  # inner cavity side
FLAP_T = 0.0035  # flap slab thickness
SEAM = 0.0015  # tiny seam gap between meeting flaps when closed

# Flaps are hinged at the INNER top edge of each wall and reach inward to the box
# center, so opposite flaps nearly meet in the middle when closed.
HINGE_INSET = INNER / 2.0  # inner wall face distance from center (= hinge line)
FLAP_REACH = INNER / 2.0 - SEAM  # how far a flap extends inward toward the center


def _box_shell() -> cq.Workplane:
    # Open-topped hollow shell: 4 side walls + bottom, cavity open at the top.
    outer = cq.Workplane("XY").box(S, S, S, centered=(True, True, False))
    # Cut the interior, leaving the bottom and four walls. The cut pokes out the
    # top (z > S) so the top is fully open.
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .box(INNER, INNER, S, centered=(True, True, False))
    )
    return outer.cut(cavity)


def _flap_slab(length_along_hinge: float, reach: float) -> cq.Workplane:
    # A thin rectangular flap. Local frame: hinge line on the local -Y edge at the
    # origin, slab lies in the XY plane and reaches toward +Y by `reach`, thickness
    # along Z (top surface near z=0, so the slab hangs just below the rim when closed).
    return (
        cq.Workplane("XY")
        .center(0.0, reach / 2.0)
        .box(length_along_hinge, reach, FLAP_T, centered=(True, True, False))
        .translate((0.0, 0.0, -FLAP_T))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="shipping_box")

    kraft = model.material("kraft_cardboard", rgba=(0.78, 0.66, 0.45, 1.0))
    kraft_flap = model.material("kraft_flap", rgba=(0.80, 0.69, 0.49, 1.0))
    ink = model.material("print_ink", rgba=(0.20, 0.18, 0.15, 1.0))

    # ---- box shell (root) ----
    box = model.part("box_shell")
    box.visual(mesh_from_cadquery(_box_shell(), "box_shell"), material=kraft, name="box_shell")

    # Printed handling-icon decal panel on the south (-Y) wall, lower-middle area.
    # A thin dark slab proud of the wall outer face, reading as the printed icons.
    decal = Box((0.105, 0.0008, 0.045))
    box.visual(
        decal,
        origin=Origin(xyz=(0.0, -(S / 2.0 + 0.0004), 0.085)),
        material=ink,
        name="print_decal",
    )

    box.inertial = Inertial.from_geometry(
        Box((S, S, S)), mass=0.35, origin=Origin(xyz=(0.0, 0.0, S / 2.0))
    )

    # ---- four top flaps ----
    # Long outer flaps (north / south): span the full inner X width, hinge axis X.
    long_len = INNER
    # Short inner flaps (east / west): span a little under the inner Y width so they
    # tuck between the long flaps, hinge axis Y.
    short_len = INNER - 2.0 * FLAP_T - 2.0 * SEAM

    # Inner (short E/W) flaps fold down first and sit one layer lower; the outer
    # (long N/S) flaps fold on top of them, flush with the rim.
    z_outer = S  # top surface of long flaps at the rim
    z_inner = S - FLAP_T  # short flaps a layer below

    flap_defs = [
        # name, hinge origin (xyz), axis, local rotation about Z so the slab's
        # +Y reach points toward box center, length along hinge.
        # flap_n: hinge on inner north edge (+Y), reaches toward -Y (center). Slab
        # local +Y must map to world -Y -> rotate local frame 180 deg about Z.
        # Axis -X so positive q lifts the -Y-reaching free edge up and out.
        ("flap_n", (0.0, HINGE_INSET, z_outer), (-1.0, 0.0, 0.0), 180.0, long_len),
        # flap_s: hinge on inner south edge (-Y), reaches toward +Y. No rotation.
        # Axis +X so positive q lifts the +Y-reaching free edge up and out.
        ("flap_s", (0.0, -HINGE_INSET, z_outer), (1.0, 0.0, 0.0), 0.0, long_len),
        # flap_e: hinge on inner east edge (+X), reaches toward -X. Local +Y -> world
        # -X -> rotate +90 deg about Z. Hinge axis +Y lifts the -X free edge up.
        ("flap_e", (HINGE_INSET, 0.0, z_inner), (0.0, 1.0, 0.0), 90.0, short_len),
        # flap_w: hinge on inner west edge (-X), reaches toward +X. Local +Y -> world
        # +X -> rotate -90 deg about Z. Hinge axis -Y lifts the +X free edge up.
        ("flap_w", (-HINGE_INSET, 0.0, z_inner), (0.0, -1.0, 0.0), -90.0, short_len),
    ]

    for name, hinge_xyz, axis, rot_deg_z, hinge_len in flap_defs:
        part = model.part(name)
        slab = _flap_slab(hinge_len, FLAP_REACH)
        part.visual(
            mesh_from_cadquery(slab, name),
            origin=Origin(rpy=(0.0, 0.0, _deg(rot_deg_z))),
            material=kraft_flap,
            name=name,
        )
        part.inertial = Inertial.from_geometry(
            Box((hinge_len, FLAP_REACH, FLAP_T)),
            mass=0.03,
            origin=Origin(
                xyz=_rot_z((0.0, FLAP_REACH / 2.0, -FLAP_T / 2.0), rot_deg_z)
            ),
        )
        model.articulation(
            f"box_to_{name}",
            ArticulationType.REVOLUTE,
            parent=box,
            child=part,
            origin=Origin(xyz=hinge_xyz),
            axis=axis,
            # Positive q swings the free (inner) edge up and outward (open).
            motion_limits=MotionLimits(lower=0.0, upper=1.5708, effort=2.0, velocity=2.0),
        )

    return model


def _deg(d: float) -> float:
    import math

    return math.radians(d)


def _rot_z(xyz, deg: float):
    import math

    a = math.radians(deg)
    x, y, z = xyz
    return (x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a), z)


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    import math

    ctx = TestContext(object_model)

    box = object_model.get_part("box_shell")
    flap_n = object_model.get_part("flap_n")
    flap_s = object_model.get_part("flap_s")
    flap_e = object_model.get_part("flap_e")
    flap_w = object_model.get_part("flap_w")
    j_n = object_model.get_articulation("box_to_flap_n")
    j_s = object_model.get_articulation("box_to_flap_s")
    j_e = object_model.get_articulation("box_to_flap_e")
    j_w = object_model.get_articulation("box_to_flap_w")

    # --- box is a cube ---
    bext = _ext(ctx.part_world_aabb(box))
    ctx.check(
        "box footprint/height read as a cube",
        abs(bext[0] - S) < 0.01 and abs(bext[1] - S) < 0.01 and abs(bext[2] - S) < 0.01,
        details=f"box extents={bext}",
    )

    # --- flaps meet near the top center when closed (opposite flaps overlap in plan) ---
    ctx.allow_overlap(
        flap_n,
        flap_e,
        reason="Closed top flaps lie in adjacent thin layers and abut near the box center.",
    )
    ctx.allow_overlap(
        flap_s,
        flap_e,
        reason="Closed top flaps lie in adjacent thin layers and abut near the box center.",
    )
    ctx.allow_overlap(
        flap_n,
        flap_w,
        reason="Closed top flaps lie in adjacent thin layers and abut near the box center.",
    )
    ctx.allow_overlap(
        flap_s,
        flap_w,
        reason="Closed top flaps lie in adjacent thin layers and abut near the box center.",
    )

    # Each closed flap sits at the top of the box (just below the rim).
    for fl, nm in ((flap_n, "flap_n"), (flap_s, "flap_s"), (flap_e, "flap_e"), (flap_w, "flap_w")):
        fmn, fmx = ctx.part_world_aabb(fl)
        ctx.check(
            f"{nm} lies flat across the box top when closed",
            fmx[2] <= S + 0.002 and fmn[2] >= S - 2.0 * FLAP_T - 0.002,
            details=f"{nm} z-range=({fmn[2]}, {fmx[2]})",
        )

    # Opposite long flaps reach toward each other and (nearly) meet at center.
    ns_mn = ctx.part_world_aabb(flap_n)[0][1]  # north flap min-y (its inner free edge)
    ctx.check(
        "north flap reaches in toward the box center",
        ns_mn < 0.01,
        details=f"north flap inner-edge y={ns_mn}",
    )

    # --- each flap opens by swinging its free edge UP about its top edge ---
    # When opened ~90 deg the flap stands vertical: its free (inner) edge lifts well
    # above the rim, and the flat slab's vertical extent grows to ~ its reach length.
    stand = 0.7 * FLAP_REACH  # expected vertical rise of a flap standing up

    for fl, j, nm in (
        (flap_n, j_n, "flap_n"),
        (flap_s, j_s, "flap_s"),
        (flap_e, j_e, "flap_e"),
        (flap_w, j_w, "flap_w"),
    ):
        rest = ctx.part_world_aabb(fl)
        with ctx.pose({j: math.pi / 2.0}):
            opened = ctx.part_world_aabb(fl)
        rest_h = rest[1][2] - rest[0][2]
        open_h = opened[1][2] - opened[0][2]
        ctx.check(
            f"{nm} free edge swings up when opened",
            opened[1][2] > rest[1][2] + 0.05,
            details=f"{nm} max-z rest={rest[1][2]:.4f} open={opened[1][2]:.4f}",
        )
        ctx.check(
            f"{nm} stands vertical (free edge lifts off the rim) when opened",
            open_h > rest_h + stand and opened[1][2] > S + stand - 0.01,
            details=f"{nm} z-extent rest={rest_h:.4f} open={open_h:.4f} top={opened[1][2]:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
