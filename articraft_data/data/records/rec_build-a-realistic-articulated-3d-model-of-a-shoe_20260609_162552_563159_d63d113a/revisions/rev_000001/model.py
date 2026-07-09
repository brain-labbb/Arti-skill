from __future__ import annotations

# Articraft model: Nike two-piece shoe box.
#
# Real object: a classic red corrugated-cardboard shoe box (Nike). It is a
# two-piece box: a shallow tray-shaped BASE that holds the shoes, and a
# lift-off LID whose deep side walls (skirt) overlap the OUTSIDE of the base
# rim. The lid is the one movable part.
#
# Mechanism: a real two-piece shoe box opens by LIFTING the lid straight up off
# the base. We model this as a PRISMATIC joint along +Z. At rest the lid sits
# down over the base with its deep skirt wrapping the outside of the base
# walls; raising the joint lifts the whole lid vertically off the box.
#
# Frames:
# - Base part frame: centered on the box footprint, z=0 at the table.
# - Lid part frame: centered in X/Y on the footprint, with the lid top panel
#   just above z=0 and the skirt hanging down. The joint origin raises that
#   frame to the rest (seated) height; positive prismatic q lifts it upward.
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# --- Real-world dimensions (meters) ---------------------------------------
BOX_LEN = 0.340  # X: long dimension of the box
BOX_WID = 0.205  # Y: depth / width
BASE_H = 0.100  # Z: height of the base tray (walls)
WALL = 0.0045  # cardboard wall thickness

LID_OVERLAP = 0.010  # clearance between lid inner skirt wall and base outer wall
LID_SKIRT = 0.045  # how far the lid side walls hang down over the base
LID_TOP_T = 0.0045  # lid top panel thickness

# Lid outer footprint sits outside the base footprint so the skirt wraps it.
LID_LEN = BOX_LEN + 2.0 * (WALL + LID_OVERLAP)
LID_WID = BOX_WID + 2.0 * (WALL + LID_OVERLAP)

# Rest geometry: the lid top panel sits LID_TOP_T above the base rim, with the
# skirt dropping LID_SKIRT down over the outside of the base wall.
LID_SEAT_Z = BASE_H + LID_TOP_T  # world z of the lid local origin when seated
LIFT_TRAVEL = 0.120  # how far the lid can be lifted straight up

# --- Materials -------------------------------------------------------------
NIKE_RED = (0.851, 0.118, 0.137, 1.0)  # Nike box red
WHITE = (0.945, 0.945, 0.945, 1.0)
GREY = (0.62, 0.63, 0.64, 1.0)  # swoosh / logo grey on the front
LABEL_BG = (0.97, 0.97, 0.96, 1.0)
LABEL_INK = (0.20, 0.20, 0.22, 1.0)
LOGO_DECAL_T = 0.0012


def _sample_cubic(
    p0: tuple[float, float],
    p1: tuple[float, float],
    p2: tuple[float, float],
    p3: tuple[float, float],
    steps: int,
) -> list[tuple[float, float]]:
    pts: list[tuple[float, float]] = []
    for i in range(steps + 1):
        t = i / steps
        u = 1.0 - t
        pts.append(
            (
                u**3 * p0[0] + 3.0 * u**2 * t * p1[0] + 3.0 * u * t**2 * p2[0] + t**3 * p3[0],
                u**3 * p0[1] + 3.0 * u**2 * t * p1[1] + 3.0 * u * t**2 * p2[1] + t**3 * p3[1],
            )
        )
    return pts


def _swoosh_profile(length: float, height: float) -> list[tuple[float, float]]:
    """A smooth, tapered Nike-swoosh-like 2D profile, centered near origin."""
    top = _sample_cubic(
        (-0.55, -0.02),
        (-0.22, 0.03),
        (0.30, 0.18),
        (0.60, 0.31),
        18,
    )
    belly = _sample_cubic(
        (0.60, 0.31),
        (0.20, 0.06),
        (-0.18, -0.54),
        (-0.55, -0.20),
        22,
    )
    return [(x * length, y * height) for x, y in (top + belly[1:])]


def _poly_plate(points: list[tuple[float, float]], thickness: float = LOGO_DECAL_T) -> cq.Workplane:
    return cq.Workplane("XY").polyline(points).close().extrude(thickness)


def _stroke_between(
    start: tuple[float, float],
    end: tuple[float, float],
    width: float,
    thickness: float = LOGO_DECAL_T,
) -> cq.Workplane:
    x0, y0 = start
    x1, y1 = end
    dx = x1 - x0
    dy = y1 - y0
    length = math.hypot(dx, dy)
    nx = -dy / length * width / 2.0
    ny = dx / length * width / 2.0
    return _poly_plate(
        [
            (x0 + nx, y0 + ny),
            (x0 - nx, y0 - ny),
            (x1 - nx, y1 - ny),
            (x1 + nx, y1 + ny),
        ],
        thickness,
    )


def _slanted_rect(
    center: tuple[float, float],
    width: float,
    height: float,
    slant: float,
    thickness: float = LOGO_DECAL_T,
) -> cq.Workplane:
    cx, cy = center
    return _poly_plate(
        [
            (cx - width / 2.0 - slant / 2.0, cy - height / 2.0),
            (cx + width / 2.0 - slant / 2.0, cy - height / 2.0),
            (cx + width / 2.0 + slant / 2.0, cy + height / 2.0),
            (cx - width / 2.0 + slant / 2.0, cy + height / 2.0),
        ],
        thickness,
    )


def _union_plates(plates: list[cq.Workplane]) -> cq.Workplane:
    result = plates[0]
    for plate in plates[1:]:
        result = result.union(plate)
    return result


def _nike_wordmark_decal(length: float, height: float) -> cq.Workplane:
    """Blocky italic NIKE wordmark made from separate raised strokes."""
    stroke = height * 0.18
    slant = height * 0.26
    plates = [
        # N
        _slanted_rect((-0.43 * length, 0.0), stroke, height, slant),
        _stroke_between((-0.46 * length, 0.47 * height), (-0.30 * length, -0.47 * height), stroke),
        _slanted_rect((-0.28 * length, 0.0), stroke, height, slant),
        # I
        _slanted_rect((-0.13 * length, 0.0), stroke, height, slant),
        # K
        _slanted_rect((0.02 * length, 0.0), stroke, height, slant),
        _stroke_between((0.05 * length, 0.02 * height), (0.21 * length, 0.48 * height), stroke),
        _stroke_between((0.05 * length, -0.02 * height), (0.23 * length, -0.48 * height), stroke),
        # E
        _slanted_rect((0.31 * length, 0.0), stroke, height, slant),
        _slanted_rect((0.42 * length, 0.40 * height), 0.19 * length, stroke, slant * 0.35),
        _slanted_rect((0.41 * length, 0.02 * height), 0.15 * length, stroke, slant * 0.25),
        _slanted_rect((0.42 * length, -0.40 * height), 0.20 * length, stroke, slant * 0.35),
    ]
    return _union_plates(plates)


def _nike_swoosh_decal(
    length: float, height: float, thickness: float = LOGO_DECAL_T
) -> cq.Workplane:
    return cq.Workplane("XY").polyline(_swoosh_profile(length, height)).close().extrude(thickness)


def _build_base_solid() -> cq.Workplane:
    """Hollow tray: floor + four walls, open top. Centered footprint, z in [0, BASE_H]."""
    outer = cq.Workplane("XY").box(BOX_LEN, BOX_WID, BASE_H, centered=(True, True, False))
    inner = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .box(
            BOX_LEN - 2.0 * WALL,
            BOX_WID - 2.0 * WALL,
            BASE_H,  # tall enough to cut through the open top
            centered=(True, True, False),
        )
    )
    tray = outer.cut(inner)
    tray = tray.edges("|Z").fillet(0.004)
    return tray


def _build_lid_solid() -> cq.Workplane:
    """Inverted shallow tray (the lid) in its own centered local frame.

    Local frame: centered in X/Y. The lid top panel occupies z in
    [-LID_TOP_T, 0] and the skirt hangs down to z = -(LID_TOP_T + LID_SKIRT).
    """
    bottom_z = -(LID_TOP_T + LID_SKIRT)
    outer = (
        cq.Workplane("XY")
        .workplane(offset=bottom_z)
        .box(LID_LEN, LID_WID, LID_TOP_T + LID_SKIRT, centered=(True, True, False))
    )
    # Hollow the underside: leave the top panel and the four skirt walls.
    inner = (
        cq.Workplane("XY")
        .workplane(offset=bottom_z)
        .box(
            LID_LEN - 2.0 * WALL,
            LID_WID - 2.0 * WALL,
            LID_SKIRT,  # leaves a LID_TOP_T-thick top panel above
            centered=(True, True, False),
        )
    )
    lid = outer.cut(inner)
    lid = lid.edges("|Z").fillet(0.004)
    return lid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nike_shoe_box")

    # ---- Materials ------------------------------------------------------
    model.material("box_red", rgba=NIKE_RED)
    model.material("white", rgba=WHITE)
    model.material("logo_grey", rgba=GREY)
    model.material("label_bg", rgba=LABEL_BG)
    model.material("label_ink", rgba=LABEL_INK)

    # ---- BASE (root): hollow red tray -----------------------------------
    base = model.part("base")
    base_solid = _build_base_solid()
    base.visual(
        mesh_from_cadquery(base_solid, "base_tray"),
        material="box_red",
        name="base_tray",
    )

    # Front-face NIKE wordmark + swoosh decals (thin proud plates on the -Y wall).
    front_y = -BOX_WID / 2.0 - 0.0005  # used by the existing label geometry below
    front_print_y = -BOX_WID / 2.0 + 0.0003
    base.visual(
        mesh_from_cadquery(_nike_wordmark_decal(0.150, 0.038), "front_wordmark"),
        origin=Origin(
            xyz=(0.032, front_print_y, BASE_H * 0.61),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="logo_grey",
        name="front_wordmark",
    )
    base.visual(
        mesh_from_cadquery(_nike_swoosh_decal(0.185, 0.070), "front_swoosh"),
        origin=Origin(
            xyz=(0.046, front_print_y, BASE_H * 0.34),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="logo_grey",
        name="front_swoosh",
    )
    label = (
        cq.Workplane("XY")
        .box(0.085, 0.0010, 0.055, centered=(True, True, True))
        .translate((-BOX_LEN / 2.0 + 0.055, front_y, BASE_H * 0.42))
    )
    base.visual(
        mesh_from_cadquery(label, "front_label"),
        material="label_bg",
        name="front_label",
    )
    for i, dz in enumerate((0.014, 0.000, -0.014)):
        line = (
            cq.Workplane("XY")
            .box(0.060, 0.0012, 0.0035, centered=(True, True, True))
            .translate((-BOX_LEN / 2.0 + 0.055, front_y - 0.0002, BASE_H * 0.42 + dz))
        )
        base.visual(
            mesh_from_cadquery(line, f"label_line_{i}"),
            material="label_ink",
            name=f"label_line_{i}",
        )

    # ---- LID: hollow red cap with deep overlapping skirt ----------------
    lid = model.part("lid")
    lid_solid = _build_lid_solid()
    lid.visual(
        mesh_from_cadquery(lid_solid, "lid_cap"),
        material="box_red",
        name="lid_cap",
    )
    # NIKE wordmark + swoosh on the lid TOP panel (top surface at local z=0).
    top_z = 0.0005  # just proud of the lid top surface
    lid.visual(
        mesh_from_cadquery(_nike_wordmark_decal(0.195, 0.046), "lid_wordmark"),
        origin=Origin(xyz=(-0.005, BOX_WID * 0.16, top_z - 0.0009)),
        material="white",
        name="lid_wordmark",
    )
    lid_swoosh = (
        cq.Workplane("XY")
        .polyline(_swoosh_profile(0.215, 0.078))
        .close()
        # Extrude down into the lid top panel so the decal stays bonded to it
        # (no floating island) while still standing proud of the top surface.
        .extrude(-0.0020)
        .translate((0.025, -BOX_WID * 0.12, top_z + 0.0010))
    )
    lid.visual(
        mesh_from_cadquery(lid_swoosh, "lid_swoosh"),
        material="white",
        name="lid_swoosh",
    )

    # ---- Articulation: lift-off lid (prismatic, straight up) ------------
    # At q=0 the lid is seated over the base. Positive q lifts it up along +Z.
    model.articulation(
        "lid_lift",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, LID_SEAT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=0.3, lower=0.0, upper=LIFT_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    lift = object_model.get_articulation("lid_lift")

    # --- Mechanism: correct joint type and axis --------------------------
    ctx.check(
        "lid lift is prismatic",
        str(lift.articulation_type).endswith("PRISMATIC"),
        details=f"type={lift.articulation_type}",
    )
    ctx.check(
        "lift axis is +Z (straight up)",
        abs(lift.axis[2]) > 0.99 and abs(lift.axis[0]) < 0.01 and abs(lift.axis[1]) < 0.01,
        details=f"axis={lift.axis}",
    )

    # --- Hero geometry present and placed --------------------------------
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base footprint reads as a shoe box",
        base_aabb is not None
        and (base_aabb[1][0] - base_aabb[0][0]) > 0.30
        and (base_aabb[1][1] - base_aabb[0][1]) > 0.18
        and (base_aabb[1][2] - base_aabb[0][2]) > 0.08,
        details=f"base_aabb={base_aabb}",
    )
    for nm in ("front_wordmark", "front_swoosh", "front_label", "label_line_0"):
        ctx.check(f"base has {nm}", base.get_visual(nm) is not None)
    for nm in ("lid_wordmark", "lid_swoosh"):
        ctx.check(f"lid has {nm}", lid.get_visual(nm) is not None)

    # The deep lid skirt intentionally wraps the OUTSIDE of the base rim. The
    # actual solids do not interpenetrate (the skirt inner wall clears the base
    # outer wall by ~10 mm), but bounding-box overlap QC reports the seated
    # capture as an overlap. Allow exactly that lid-cap / base-tray capture fit.
    ctx.allow_overlap(
        lid,
        base,
        elem_a="lid_cap",
        elem_b="base_tray",
        reason=(
            "Two-piece shoe box: the lid's deep skirt is seated down over the "
            "outside of the base rim (capture fit). Skirt inner wall clears the "
            "base outer wall; only the seated-capture bounding overlap remains."
        ),
    )

    # --- Closed pose: lid seated, deep overlapping skirt -----------------
    with ctx.pose({lift: 0.0}):
        # Lid wraps the base footprint in plan (deep overlapping lid). The box
        # is ~0.205 m deep, so require meaningful overlap on both plan axes.
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            min_overlap=0.18,
            elem_a="lid_cap",
            elem_b="base_tray",
            name="closed lid wraps the base footprint",
        )
        # Skirt drops down over the outside of the base wall (retained overlap).
        ctx.expect_overlap(
            lid,
            base,
            axes="z",
            min_overlap=0.030,
            elem_a="lid_cap",
            elem_b="base_tray",
            name="seated lid skirt overlaps the base wall vertically",
        )
        # The lid top panel reads as a cap sitting on/above the base rim, not
        # sunk deep into the cavity: its top surface is at or above the rim.
        lid_top_z = ctx.part_element_world_aabb(lid, elem="lid_cap")[1][2]
        ctx.check(
            "lid top panel caps the box at the rim",
            lid_top_z >= BASE_H - 0.001,
            details=f"lid_top_z={lid_top_z}, base_rim={BASE_H}",
        )
        seated_lid_min_z = ctx.part_world_aabb(lid)[0][2]
        ctx.check(
            "seated lid skirt drops below the base rim",
            seated_lid_min_z < BASE_H - 0.020,
            details=f"lid_min_z={seated_lid_min_z}, base_rim={BASE_H}",
        )
        seated_top = ctx.part_world_aabb(lid)[1][2]

    # --- Open pose: lifting the joint raises the lid straight up ---------
    with ctx.pose({lift: LIFT_TRAVEL}):
        open_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "lifting the lid raises it clear of the base",
            open_aabb is not None and open_aabb[1][2] > seated_top + LIFT_TRAVEL - 0.001,
            details=f"open_top={open_aabb[1][2] if open_aabb else None}, seated_top={seated_top}",
        )
        ctx.check(
            "fully lifted lid clears the base rim",
            open_aabb is not None and open_aabb[0][2] > BASE_H + 0.010,
            details=f"open_min_z={open_aabb[0][2] if open_aabb else None}, base_rim={BASE_H}",
        )

    _ = math  # available for geometry tuning

    return ctx.report()


object_model = build_object_model()
