from __future__ import annotations

"""Cellular honeycomb pleated shade.

Fork of the horizontal wooden venetian blind: same headrail, bottom rail, and
lift mechanism, but the slat/panel layer is replaced by repeated horizontal
honeycomb cells. Each cell is a hexagonal-cross-section fabric tube extruded
across the blind width. The cells gather from the bottom upward when the lift
cord is pulled, compressing into a tight stack beneath the headrail.

Articulations:
- lift (PRISMATIC driver on the bottom rail) + 30 per-cell mimic lift joints:
  pulling the lift cord raises the bottom rail, and the cells themselves rise
  and gather into a tight stack under the headrail. Each cell rides its own
  prismatic carrier that mimics `lift`; the multiplier grows toward the bottom,
  so the lowest cells travel farthest and the gather is led from the bottom
  upward, collapsing the open ~38 mm cell pitch down to a ~8 mm stacked pitch.

No tilt mechanism: cellular shades do not tilt. The tilt wand and ladder tapes
from the venetian-blind parent are removed. Thin cord-guide strings at two
stations replace the ladder tapes as the visible vertical support elements.

Support strategy: the cord guides are thin cylinders owned by the headrail,
running the full drop from inside the headrail to the bottom rail. Each cell
carrier is connected to the headrail via its own prismatic lift joint. The
cells pass around the cord guides, giving visual continuity.
"""

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---------------------------------------------------------------- dimensions
BLIND_WIDTH = 0.80  # headrail width (X)
CELL_WIDTH = 0.78
CELL_DEPTH = 0.035   # front-to-back depth of each honeycomb cell (Y)
CELL_HEIGHT = 0.028  # expanded cell height (Z)
CELL_COUNT = 30
CELL_PITCH = 0.038   # open (lowered) cell-to-cell pitch
TOP_CELL_Z = 1.215

HEADRAIL_SIZE = (BLIND_WIDTH, 0.062, 0.060)
HEADRAIL_Z = 1.27   # center; spans 1.24 .. 1.30

BOTTOM_RAIL_SIZE = (CELL_WIDTH, 0.055, 0.022)
BOTTOM_RAIL_Z = 0.068  # center; spans 0.057 .. 0.079

CORD_STATIONS = (-0.20, 0.20)   # cord guide X positions
CORD_GUIDE_RADIUS = 0.002       # 2 mm cord

# ---------------------------------------------------------------- gather (lift)
STACK_PITCH = 0.008   # compressed cell pitch when fully raised
STACK_TOP_Z = TOP_CELL_Z + 0.005  # gathered stack top; top cell lifts ~5 mm
RAIL_GATHER_GAP = 0.006

# ---------------------------------------------------------------- materials
DARK_WOOD = Material("dark_wood", rgba=(0.23, 0.155, 0.105, 1.0))
DARK_WOOD_RAIL = Material("dark_wood_rail", rgba=(0.20, 0.135, 0.09, 1.0))
FABRIC_CREAM = Material("fabric_cream", rgba=(0.91, 0.86, 0.76, 1.0))
CORD_TAN = Material("cord_tan", rgba=(0.66, 0.55, 0.40, 1.0))


def cell_down_z(index: int) -> float:
    """Headrail-frame Z of a cell center when lowered (open). 1-based from top."""
    return TOP_CELL_Z - (index - 1) * CELL_PITCH


def cell_up_z(index: int) -> float:
    """Headrail-frame Z of a cell center when fully gathered under the headrail."""
    return STACK_TOP_Z - (index - 1) * STACK_PITCH


def cell_displacement(index: int) -> float:
    """How far a cell rises between lowered and fully gathered (always > 0)."""
    return cell_up_z(index) - cell_down_z(index)


# Bottom rail sits just under the lowest gathered cell at full lift; its travel
# is the master lift stroke that every cell carrier mimics.
RAIL_UP_Z = cell_up_z(CELL_COUNT) - (0.5 * CELL_HEIGHT + RAIL_GATHER_GAP + 0.5 * BOTTOM_RAIL_SIZE[2])
RAIL_TRAVEL = RAIL_UP_Z - BOTTOM_RAIL_Z


# --------------------------------------------------------- geometry helper
def make_honeycomb_cell(width: float, depth: float, height: float):
    """Build a honeycomb cell mesh: hexagonal-cross-section prism along X.

    The hex profile is defined in local XY of ExtrudeGeometry (extrusion along
    local Z), then rotated so that:
      - extrusion Z -> world X  (cell width along the blind)
      - profile  X  -> world Y  (cell depth, front-to-back)
      - profile  Y  -> world Z  (cell height, vertical)
    """
    hw = depth * 0.30   # half-width of flat top/bottom pleats
    hd = depth * 0.50   # half-depth at the widest point
    hh = height * 0.50  # half-height

    profile = [
        (-hw,  hh),
        ( hw,  hh),
        ( hd, 0.0),
        ( hw, -hh),
        (-hw, -hh),
        (-hd, 0.0),
    ]
    geom = ExtrudeGeometry(profile, width, cap=True, center=True)
    # 120-degree rotation about (1,1,1) maps (x,y,z) -> (z,x,y)
    geom.rotate(axis=(1.0, 1.0, 1.0), angle=2.0 * math.pi / 3.0)
    return geom


# --------------------------------------------------------- build
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cellular_honeycomb_shade")

    # --------------------------------------------------------- headrail
    headrail = model.part("headrail")
    headrail.visual(
        Box(HEADRAIL_SIZE),
        origin=Origin(xyz=(0.0, 0.0, HEADRAIL_Z)),
        material=DARK_WOOD_RAIL,
        name="headrail_box",
    )

    # Cord guides: thin vertical strings at two stations, headrail -> bottom rail.
    cord_guide_len = HEADRAIL_Z - BOTTOM_RAIL_Z + 0.04
    cord_guide_mid_z = 0.5 * (HEADRAIL_Z + BOTTOM_RAIL_Z)
    for station_x, side in zip(CORD_STATIONS, ("a", "b")):
        headrail.visual(
            Cylinder(radius=CORD_GUIDE_RADIUS, length=cord_guide_len),
            origin=Origin(xyz=(station_x, 0.0, cord_guide_mid_z)),
            material=CORD_TAN,
            name=f"cord_guide_{side}",
        )

    # Cord lock on the front-right of the headrail; pull cords exit here.
    headrail.visual(
        Box((0.07, 0.014, 0.030)),
        origin=Origin(xyz=(0.32, 0.036, 1.252)),
        material=DARK_WOOD_RAIL,
        name="cord_lock",
    )
    for cord_x, idx in ((0.305, 0), (0.335, 1)):
        cord_top = 1.260
        cord_bottom = 0.500
        headrail.visual(
            Cylinder(radius=0.0022, length=cord_top - cord_bottom),
            origin=Origin(xyz=(cord_x, 0.038, 0.5 * (cord_top + cord_bottom))),
            material=CORD_TAN,
            name=f"lift_cord_{idx}",
        )
        headrail.visual(
            Cylinder(radius=0.008, length=0.055),
            origin=Origin(xyz=(cord_x, 0.038, 0.475)),
            material=DARK_WOOD,
            name=f"cord_tassel_{idx}",
        )

    # ------------------------------------------------------- bottom rail
    bottom_rail = model.part("bottom_rail")
    bottom_rail.visual(
        Box(BOTTOM_RAIL_SIZE),
        origin=Origin(),
        material=DARK_WOOD_RAIL,
        name="bottom_rail_bar",
    )
    model.articulation(
        "lift",
        ArticulationType.PRISMATIC,
        parent=headrail,
        child=bottom_rail,
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_RAIL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.5, lower=0.0, upper=RAIL_TRAVEL),
    )

    # --------------------------------------------------- honeycomb cells
    # Shared mesh asset: one hex-cell geometry reused by all 30 cell visuals.
    cell_mesh = mesh_from_geometry(
        make_honeycomb_cell(CELL_WIDTH, CELL_DEPTH, CELL_HEIGHT),
        "honeycomb_cell",
    )

    for i in range(1, CELL_COUNT + 1):
        cell = model.part(f"cell_{i:02d}")
        cell.visual(
            cell_mesh,
            origin=Origin(),
            material=FABRIC_CREAM,
            name=f"cell_{i:02d}_body",
        )

        disp = cell_displacement(i)
        lift_mult = disp / RAIL_TRAVEL
        model.articulation(
            f"cell_lift_{i:02d}",
            ArticulationType.PRISMATIC,
            parent=headrail,
            child=cell,
            origin=Origin(xyz=(0.0, 0.0, cell_down_z(i))),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=20.0, velocity=0.5, lower=0.0, upper=disp),
            mimic=Mimic(joint="lift", multiplier=lift_mult, offset=0.0),
        )

    return model


# --------------------------------------------------------- tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    headrail = object_model.get_part("headrail")
    bottom_rail = object_model.get_part("bottom_rail")
    cells = [object_model.get_part(f"cell_{i:02d}") for i in range(1, CELL_COUNT + 1)]
    lift = object_model.get_articulation("lift")

    # 1. Cell count is 30.
    cell_parts = [p for p in object_model.parts if p.name.startswith("cell_")]
    ctx.check(
        "cell_count_is_30",
        len(cell_parts) == CELL_COUNT,
        f"expected {CELL_COUNT} cells, found {len(cell_parts)}",
    )

    # 2. No tilt articulation exists (cellular shades do not tilt).
    tilt_joints = [a for a in object_model.articulations if "tilt" in a.name]
    ctx.check(
        "no_tilt_articulation",
        len(tilt_joints) == 0,
        f"found tilt joints: {[a.name for a in tilt_joints]}",
    )

    # 3. Lift driver: a single prismatic on the bottom rail, vertical, positive stroke.
    ctx.check(
        "lift_driver_axis_is_vertical_z",
        tuple(lift.axis) == (0.0, 0.0, 1.0)
        and lift.articulation_type == ArticulationType.PRISMATIC
        and lift.mimic is None,
        f"lift axis={lift.axis} type={lift.articulation_type} mimic={lift.mimic}",
    )
    ctx.check(
        "lift_stroke_positive",
        lift.motion_limits is not None and lift.motion_limits.upper > 0.3,
        f"lift travel={RAIL_TRAVEL:.3f} m",
    )

    # 4. Per-cell lift mimic chain: 30 carriers mimic `lift`, multipliers strictly
    #    increase from the top cell to the bottom cell (the bottom leads the gather).
    lift_followers = [
        object_model.get_articulation(f"cell_lift_{i:02d}") for i in range(1, CELL_COUNT + 1)
    ]
    mults = [a.mimic.multiplier for a in lift_followers if a.mimic is not None]
    ok_lift_chain = (
        len(mults) == CELL_COUNT
        and all(a.mimic is not None and a.mimic.joint == "lift" for a in lift_followers)
        and all(0.0 < mm <= 1.0 for mm in mults)
        and all(mults[k] < mults[k + 1] for k in range(len(mults) - 1))
    )
    ctx.check(
        "lift_followers_mimic_lift_bottom_leads",
        ok_lift_chain,
        f"lift multipliers (top->bottom)={[round(mm, 3) for mm in mults]}",
    )

    # 5. Cord guides at two stations.
    for side in ("a", "b"):
        guide = headrail.get_visual(f"cord_guide_{side}")
        ctx.check(
            f"cord_guide_{side}_exists",
            guide is not None,
            "missing cord guide",
        )

    # 6. Cells have visible depth and height (hexagonal cross-section, not flat).
    cell_aabb = ctx.part_world_aabb(cells[0])
    assert cell_aabb is not None
    cell_dy = cell_aabb[1][1] - cell_aabb[0][1]
    cell_dz = cell_aabb[1][2] - cell_aabb[0][2]
    ctx.check(
        "cell_has_hex_cross_section_depth",
        cell_dy > 0.025,
        f"cell Y extent={cell_dy:.4f} m (expected >0.025 for hex profile)",
    )
    ctx.check(
        "cell_has_hex_cross_section_height",
        cell_dz > 0.020,
        f"cell Z extent={cell_dz:.4f} m (expected >0.020 for hex profile)",
    )

    # 7. Cells span both cord-guide stations.
    guide_xs = set()
    for side in ("a", "b"):
        g = headrail.get_visual(f"cord_guide_{side}")
        g_aabb = ctx.part_element_world_aabb(headrail, elem=g)
        assert g_aabb is not None
        guide_xs.add(round(0.5 * (g_aabb[0][0] + g_aabb[1][0]), 4))
    ctx.check(
        "cells_span_both_cord_stations",
        all(cell_aabb[0][0] < x < cell_aabb[1][0] for x in guide_xs),
        f"cell x-range=({cell_aabb[0][0]:.3f},{cell_aabb[1][0]:.3f}), stations={sorted(guide_xs)}",
    )

    headrail_box = headrail.get_visual("headrail_box")
    headrail_box_aabb = ctx.part_element_world_aabb(headrail, elem=headrail_box)
    assert headrail_box_aabb is not None

    # 8. Lowered pose: bottom rail sits below the lowest cell with a clear gap,
    #    headrail sits above the top cell, and cord guides reach the bottom rail.
    with ctx.pose({lift: 0.0}):
        ctx.expect_gap(
            cells[-1],
            bottom_rail,
            axis="z",
            min_gap=0.01,
            name="bottom_rail_below_lowest_cell",
        )
        ctx.expect_gap(
            headrail,
            cells[0],
            axis="z",
            min_gap=0.01,
            positive_elem=headrail_box,
            name="headrail_above_top_cell",
        )
        cord_guide_a = headrail.get_visual("cord_guide_a")
        ctx.expect_contact(
            headrail,
            bottom_rail,
            elem_a=cord_guide_a,
            name="cord_guide_reaches_bottom_rail",
        )

    # 9. Raised pose: cells rise and gather into a tight stack under the headrail.
    travel = float(lift.motion_limits.upper)
    z_lowered = [ctx.part_world_position(c)[2] for c in cells]
    with ctx.pose({lift: travel}):
        z_raised = [ctx.part_world_position(c)[2] for c in cells]
        rises = [zr - zl for zr, zl in zip(z_raised, z_lowered)]

        # Every cell actually rises.
        ctx.check(
            "all_cells_rise_on_lift",
            all(r > 1e-4 for r in rises),
            f"min rise={min(rises):.4f} m",
        )
        # Bottom-led gather: each cell rises more than the one above it.
        ctx.check(
            "lower_cells_lead_the_gather",
            all(rises[k] < rises[k + 1] for k in range(len(rises) - 1)),
            f"rises top->bottom={[round(r, 3) for r in rises]}",
        )
        # Gathered stack is tight.
        stack_span = z_raised[0] - z_raised[-1]
        expected_span = (CELL_COUNT - 1) * STACK_PITCH
        ctx.check(
            "gathered_stack_is_tight",
            abs(stack_span - expected_span) < 0.02,
            f"stack span={stack_span:.3f} m, expected ~{expected_span:.3f} m",
        )
        # Stack hangs under the headrail.
        top_cell_aabb = ctx.part_world_aabb(cells[0])
        assert top_cell_aabb is not None
        ctx.check(
            "gathered_stack_under_headrail",
            top_cell_aabb[1][2] < headrail_box_aabb[0][2],
            f"top cell top={top_cell_aabb[1][2]:.3f}, headrail bottom={headrail_box_aabb[0][2]:.3f}",
        )
        # Bottom rail rose to just beneath the gathered stack.
        ctx.expect_gap(
            cells[-1],
            bottom_rail,
            axis="z",
            min_gap=0.002,
            name="bottom_rail_under_gathered_stack",
        )

    # 10. Pull cords + tassels hang at the front, below the headrail.
    for idx in (0, 1):
        tassel = headrail.get_visual(f"cord_tassel_{idx}")
        aabb = ctx.part_element_world_aabb(headrail, elem=tassel)
        assert aabb is not None
        ctx.check(
            f"tassel_{idx}_hangs_below_headrail_front",
            aabb[1][2] < headrail_box_aabb[0][2] - 0.5 and aabb[0][1] > 0.0,
            f"tassel aabb={aabb}",
        )

    return ctx.report()


object_model = build_object_model()
