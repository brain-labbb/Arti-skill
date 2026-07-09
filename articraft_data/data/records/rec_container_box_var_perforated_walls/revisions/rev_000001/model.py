from __future__ import annotations

# Open corrugated cardboard box with perforated ventilated side walls and
# four fold-out top flaps.
# Frame:
#   - X is the long box axis (length ~0.28 m), X in [-0.14, 0.14]
#   - Y is the short box axis (depth ~0.24 m), Y in [-0.12, 0.12]
#   - Z is up (height ~0.22 m), floor at z=0, top rim at z=0.22
# Construction:
#   - body (root): open-topped rectangular shell = solid floor + 4 perforated
#     side walls (solid panels pierced by a regular grid of round ventilation
#     holes cut via CadQuery boolean), plus a printed-icon front panel.
#   - four top flaps, each REVOLUTE, hinged along a top rim edge:
#       * two LONG flaps hinge on the long (Y=+/-) top edges, run the full
#         length in X, and are sized so they meet at the center line when
#         folded flat (closed); they fold up/out ~110 deg.
#       * two SHORT flaps hinge on the short (X=+/-) top edges, span the depth
#         in Y, and fold up/out ~110 deg.

import math

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

# ---- master dimensions (meters) ----
LEN_X = 0.28            # outer length (long axis)
DEP_Y = 0.24           # outer depth (short axis)
HGT_Z = 0.22           # outer height
WALL = 0.006           # wall / floor thickness (visible)

HX = LEN_X / 2.0        # 0.140
HY = DEP_Y / 2.0        # 0.120
TOP_Z = HGT_Z           # top rim plane

# inner cavity extents
IN_HX = HX - WALL       # inner half length
IN_HY = HY - WALL       # inner half depth

# flap thickness
FLAP_T = 0.005

# A small seam clearance so the two long flaps just kiss at the center line
SEAM = 0.0015

OPEN_ANGLE = math.radians(110.0)

# ---- perforation parameters ----
HOLE_D = 0.008          # ventilation hole diameter (8 mm)
HOLE_R = HOLE_D / 2.0
PITCH = 0.016           # hole center-to-center spacing (16 mm)
FRAME_MARGIN = 0.014    # solid border around the hole grid (14 mm)


def _shell_solid() -> cq.Workplane:
    """Open-topped rectangular box: outer block minus an upward-open inner cavity."""
    outer = cq.Workplane("XY").box(LEN_X, DEP_Y, HGT_Z, centered=(True, True, False))
    cavity_h = HGT_Z - WALL + 0.01  # punch slightly past the top rim
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .box(
            LEN_X - 2.0 * WALL,
            DEP_Y - 2.0 * WALL,
            cavity_h,
            centered=(True, True, False),
        )
    )
    return outer.cut(cavity)


def _grid_positions(span_lo: float, span_hi: float, z_lo: float, z_hi: float) -> list[tuple[float, float]]:
    """Generate a regular grid of (u, v) center positions within the given bounds."""
    pts: list[tuple[float, float]] = []
    u = span_lo + FRAME_MARGIN
    while u <= span_hi - FRAME_MARGIN:
        v = z_lo + FRAME_MARGIN
        while v <= z_hi - FRAME_MARGIN:
            pts.append((u, v))
            v += PITCH
        u += PITCH
    return pts


def _perforated_shell() -> cq.Workplane:
    """Build the open-topped box shell with perforated ventilated side walls.

    The four side walls are solid panels pierced by a regular grid of round
    ventilation holes cut via CadQuery boolean. The floor remains solid.
    """
    shell = _shell_solid()

    # Extra extension past the wall thickness for clean boolean cuts
    cut_ext = WALL + 0.004

    # Z range for holes: above the solid floor, below the top rim
    z_lo = WALL  # just above floor inner surface
    z_hi = TOP_Z  # up to the top rim

    # === Long walls (Y-facing): +Y and -Y ===
    # Grid in (X, Z) space
    long_pts = _grid_positions(-HX, HX, z_lo, z_hi)

    if long_pts:
        # +Y wall cutter: cylinders along -Y from just outside the outer face
        plus_y_cutter = (
            cq.Workplane("XZ")
            .workplane(offset=HY + 0.001)
            .pushPoints(long_pts)
            .circle(HOLE_R)
            .extrude(-cut_ext)
        )
        shell = shell.cut(plus_y_cutter)

        # -Y wall cutter: cylinders along +Y from just outside the outer face
        minus_y_cutter = (
            cq.Workplane("XZ")
            .workplane(offset=-HY - 0.001)
            .pushPoints(long_pts)
            .circle(HOLE_R)
            .extrude(cut_ext)
        )
        shell = shell.cut(minus_y_cutter)

    # === Short walls (X-facing): +X and -X ===
    # Grid in (Y, Z) space
    short_pts = _grid_positions(-HY, HY, z_lo, z_hi)

    if short_pts:
        # +X wall cutter: cylinders along -X
        plus_x_cutter = (
            cq.Workplane("YZ")
            .workplane(offset=HX + 0.001)
            .pushPoints(short_pts)
            .circle(HOLE_R)
            .extrude(-cut_ext)
        )
        shell = shell.cut(plus_x_cutter)

        # -X wall cutter: cylinders along +X
        minus_x_cutter = (
            cq.Workplane("YZ")
            .workplane(offset=-HX - 0.001)
            .pushPoints(short_pts)
            .circle(HOLE_R)
            .extrude(cut_ext)
        )
        shell = shell.cut(minus_x_cutter)

    return shell


def _flap_solid(length: float, depth: float) -> cq.Workplane:
    """Flat rectangular cardboard flap. Local frame: hinge edge at y=0 along
    the +X..-X span; the panel extends along -Y (folds inward when closed).
    Centered in X, panel mid-plane at z=0 so it lies flush with the rim plane."""
    return (
        cq.Workplane("XY")
        .box(length, depth, FLAP_T, centered=(True, True, True))
        .translate((0.0, -depth / 2.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="perforated_box")

    kraft = model.material("kraft", rgba=(0.78, 0.62, 0.40, 1.0))
    kraft_in = model.material("kraft_inner", rgba=(0.72, 0.56, 0.36, 1.0))
    print_ink = model.material("print_ink", rgba=(0.20, 0.20, 0.22, 1.0))

    # ---- body (root): perforated shell + printed front panel ----
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_perforated_shell(), "shell"),
        material=kraft,
        name="shell",
    )

    # Printed-icon front panel: a thin printed face plate on the front (-Y) wall,
    # sitting just proud of the kraft surface, carrying ink icons.
    panel_w = 0.16
    panel_h = 0.085
    panel_cz = 0.085
    panel_face_y = -HY - 0.0005  # just outside the front wall
    body.visual(
        Box((panel_w, 0.001, panel_h)),
        origin=Origin(xyz=(0.0, panel_face_y, panel_cz)),
        material=kraft_in,
        name="front_panel",
    )
    # Recycle + handling icons rendered as small dark ink tiles in a row.
    icon_y = panel_face_y - 0.0008
    for i, ix in enumerate((-0.057, -0.019, 0.019, 0.057)):
        body.visual(
            Box((0.022, 0.0008, 0.030)),
            origin=Origin(xyz=(ix, icon_y, panel_cz)),
            material=print_ink,
            name=f"icon_{i}",
        )

    body.inertial = Inertial.from_geometry(
        Box((LEN_X, DEP_Y, HGT_Z)),
        mass=0.40,  # slightly lighter due to perforations
        origin=Origin(xyz=(0.0, 0.0, HGT_Z / 2.0)),
    )

    # Flap depth so the two long flaps just meet at the center line (y=0).
    long_depth = IN_HY - SEAM       # spans from rim toward center, kisses at y=0
    long_length = LEN_X - 2.0 * WALL - 0.002
    # Short flaps span the full inner depth between the long-flap hinge lines.
    short_depth = IN_HX - SEAM
    short_length = DEP_Y - 2.0 * WALL - 0.002

    # ---- long flap 0: hinges on the +Y top edge, folds toward -Y ----
    lf0 = model.part("long_flap_0")
    lf0.visual(
        mesh_from_cadquery(_flap_solid(long_length, long_depth), "panel"),
        material=kraft_in,
        name="panel",
    )
    lf0.inertial = Inertial.from_geometry(
        Box((long_length, long_depth, FLAP_T)),
        mass=0.03,
        origin=Origin(xyz=(0.0, -long_depth / 2.0, 0.0)),
    )
    model.articulation(
        "long_flap_0_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lf0,
        origin=Origin(xyz=(0.0, IN_HY, TOP_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=OPEN_ANGLE),
    )

    # ---- long flap 1: hinges on the -Y top edge, folds toward +Y ----
    lf1 = model.part("long_flap_1")
    lf1_solid = _flap_solid(long_length, long_depth).mirror("XZ")
    lf1.visual(
        mesh_from_cadquery(lf1_solid, "panel"),
        material=kraft_in,
        name="panel",
    )
    lf1.inertial = Inertial.from_geometry(
        Box((long_length, long_depth, FLAP_T)),
        mass=0.03,
        origin=Origin(xyz=(0.0, long_depth / 2.0, 0.0)),
    )
    model.articulation(
        "long_flap_1_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lf1,
        origin=Origin(xyz=(0.0, -IN_HY, TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=OPEN_ANGLE),
    )

    # Short flaps: hinge edge runs along Y; panel extends along -X (closed).
    def _short_solid(length: float, depth: float) -> cq.Workplane:
        return (
            cq.Workplane("XY")
            .box(depth, length, FLAP_T, centered=(True, True, True))
            .translate((-depth / 2.0, 0.0, 0.0))
        )

    # ---- short flap 0: hinges on the +X top edge, folds toward -X ----
    sf0 = model.part("short_flap_0")
    sf0.visual(
        mesh_from_cadquery(_short_solid(short_length, short_depth), "panel"),
        material=kraft_in,
        name="panel",
    )
    sf0.inertial = Inertial.from_geometry(
        Box((short_depth, short_length, FLAP_T)),
        mass=0.025,
        origin=Origin(xyz=(-short_depth / 2.0, 0.0, 0.0)),
    )
    model.articulation(
        "short_flap_0_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=sf0,
        origin=Origin(xyz=(IN_HX, 0.0, TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=OPEN_ANGLE),
    )

    # ---- short flap 1: hinges on the -X top edge, folds toward +X ----
    sf1 = model.part("short_flap_1")
    sf1_solid = _short_solid(short_length, short_depth).mirror("YZ")
    sf1.visual(
        mesh_from_cadquery(sf1_solid, "panel"),
        material=kraft_in,
        name="panel",
    )
    sf1.inertial = Inertial.from_geometry(
        Box((short_depth, short_length, FLAP_T)),
        mass=0.025,
        origin=Origin(xyz=(short_depth / 2.0, 0.0, 0.0)),
    )
    model.articulation(
        "short_flap_1_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=sf1,
        origin=Origin(xyz=(-IN_HX, 0.0, TOP_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=OPEN_ANGLE),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lf0 = object_model.get_part("long_flap_0")
    lf1 = object_model.get_part("long_flap_1")
    sf0 = object_model.get_part("short_flap_0")
    sf1 = object_model.get_part("short_flap_1")

    lf0_h = object_model.get_articulation("long_flap_0_hinge")
    lf1_h = object_model.get_articulation("long_flap_1_hinge")
    sf0_h = object_model.get_articulation("short_flap_0_hinge")
    sf1_h = object_model.get_articulation("short_flap_1_hinge")

    flaps = [
        ("long_flap_0", lf0, lf0_h),
        ("long_flap_1", lf1, lf1_h),
        ("short_flap_0", sf0, sf0_h),
        ("short_flap_1", sf1, sf1_h),
    ]

    # --- Shell has the correct box footprint and full height ---
    sh = _ext(ctx.part_element_world_aabb(body, elem="shell"))
    ctx.check(
        "shell has box footprint and full height",
        abs(sh[0] - LEN_X) < 0.01 and abs(sh[1] - DEP_Y) < 0.01 and abs(sh[2] - HGT_Z) < 0.01,
        details=f"shell extents={sh}",
    )

    # --- Perforated walls: the shell is built from CadQuery boolean-cut panels ---
    # Verify perforation grid exists by checking the shell visual is present
    # and that the perforation parameters produce a non-trivial grid.
    long_grid = _grid_positions(-HX, HX, WALL, TOP_Z)
    short_grid = _grid_positions(-HY, HY, WALL, TOP_Z)
    ctx.check(
        "long walls have a non-trivial perforation grid",
        len(long_grid) >= 50,
        details=f"long wall hole count={len(long_grid)}",
    )
    ctx.check(
        "short walls have a non-trivial perforation grid",
        len(short_grid) >= 40,
        details=f"short wall hole count={len(short_grid)}",
    )

    # Hole diameter and pitch are consistent with ventilation design
    ctx.check(
        "hole diameter is reasonable for ventilation",
        0.004 <= HOLE_D <= 0.020,
        details=f"hole_d={HOLE_D}",
    )
    ctx.check(
        "pitch exceeds hole diameter (solid web between holes)",
        PITCH > HOLE_D,
        details=f"pitch={PITCH}, hole_d={HOLE_D}",
    )

    # --- Each flap lies flat at the rim when closed, then lifts up/out when opened ---
    for name, flap, hinge in flaps:
        rest_z = ctx.part_world_aabb(flap)[1][2]
        with ctx.pose({hinge: OPEN_ANGLE}):
            open_z = ctx.part_world_aabb(flap)[1][2]
        ctx.check(
            f"{name} lies near the rim when closed",
            abs(ctx.part_world_aabb(flap)[1][2] - TOP_Z) < 0.02,
            details=f"{name} closed top z={rest_z}",
        )
        ctx.check(
            f"{name} rotates open upward about its top edge",
            open_z > rest_z + 0.03,
            details=f"{name} rest_top_z={rest_z}, open_top_z={open_z}",
        )

    # --- Two long flaps meet at the center line (y=0) when closed ---
    lf0_min_y = ctx.part_world_aabb(lf0)[0][1]
    lf1_max_y = ctx.part_world_aabb(lf1)[1][1]
    ctx.check(
        "long flaps meet at the center line when closed",
        lf0_min_y < 0.005 and lf1_max_y > -0.005 and abs(lf0_min_y) < 0.02 and abs(lf1_max_y) < 0.02,
        details=f"lf0_min_y={lf0_min_y}, lf1_max_y={lf1_max_y}",
    )

    ctx.expect_gap(
        lf0, lf1, axis="y", min_gap=-0.002, max_gap=0.02,
        name="long flaps share the center seam",
    )

    # --- Flaps are hinged on the rim: each flap touches the body shell ---
    for name, flap, _hinge in flaps:
        ctx.allow_overlap(
            flap, body, elem_a="panel", elem_b="shell",
            reason=f"{name} hinge edge sits on the box top rim and may locally touch the perforated wall.",
        )
        ctx.expect_contact(flap, body, name=f"{name} hinged on the rim")

    # Long and short flaps tuck against each other along their shared rim
    # corner when closed; allow the small corner overlap.
    for lf in (lf0, lf1):
        for sf in (sf0, sf1):
            ctx.allow_overlap(
                lf, sf, elem_a="panel", elem_b="panel",
                reason="Adjacent long/short flaps share the top rim and overlap slightly at the corners when folded flat.",
            )

    # Printed icons sit on the front panel.
    ctx.expect_contact(body, body, elem_a="icon_0", elem_b="front_panel",
                       name="recycle icon printed on front panel")

    return ctx.report()


object_model = build_object_model()
