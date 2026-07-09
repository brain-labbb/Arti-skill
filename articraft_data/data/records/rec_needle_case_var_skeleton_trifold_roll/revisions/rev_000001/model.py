"""Three-panel roll-up leather knitting needle organizer.

Layout (world frame, case lying open on the XY plane, +Z up):
- needle_panel (root, x in [0, 0.21]): carries stitched channel strip with
  4 tunnels holding 8 bamboo DPNs and a hinged tip retaining flap.
- middle_panel (x in [-0.21, 0]): connected by fold_hinge at x=0. Carries
  its own channel strip with 2 tunnels holding 4 bamboo DPNs.
- pocket_panel (x in [-0.42, -0.21]): connected by fold_hinge_2 at x=-0.21.
  Carries flat pocket, cable pocket with coiled cables, and snap straps.
- wrap_tie: long leather strap hinged on the +X outer edge of needle_panel,
  replaces the snap cover flap for tying the roll closed.
"""

from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ----------------------------------------------------------------------------
# Dimensional constants (meters)
# ----------------------------------------------------------------------------
LEATHER_T = 0.003  # main panel leather thickness
PANEL_W = 0.21  # each panel width along X
PANEL_H = 0.22  # panel height along Y (y in [-0.11, 0.11])
PANEL_TOP = LEATHER_T  # needle_panel top face z (shifted up by LEATHER_T/2)
CHILD_PANEL_TOP = LEATHER_T / 2.0  # child panel top face in local frame
EMBED = 0.0003  # small same-part embed so mounted visuals stay connected

# Needle channels (on the root needle_panel, local frame)
CHANNEL_XS = (0.05, 0.09, 0.13, 0.17)
CHANNEL_SLOT_W = 0.013
CHANNEL_SLOT_H = 0.0045  # slot height above the panel top
STRIP_X_MIN, STRIP_X_MAX = 0.025, 0.20
STRIP_Y_MIN, STRIP_Y_MAX = -0.075, 0.005
STRIP_TOP_OFFSET = 0.0055  # strip top above panel top

# Middle-panel channels (local frame, x extends along -X)
M_CHANNEL_XS = (-0.07, -0.15)
M_STRIP_X_MIN, M_STRIP_X_MAX = -0.19, -0.02

# Bamboo needles
NEEDLE_R = 0.002
NEEDLE_LEN = 0.17  # double-pointed needles
NEEDLE_OFFSETS = (-0.003, 0.003)  # two needles per channel
NEEDLE_TRAVEL = 0.095  # slide-out travel toward -Y

# Pocket-side geometry (in the pocket_panel frame: z = 0 at panel mid-plane)
PP_TOP = LEATHER_T / 2.0  # panel top in child panel frames
POCKET_WALL_H = 0.003
POCKET_SHEET_T = 0.0025
POCKET_SHEET_TOP = PP_TOP + POCKET_WALL_H + POCKET_SHEET_T

FLAT_POCKET_X = (-0.195, -0.105)
FLAT_POCKET_Y = (-0.065, 0.045)
CABLE_POCKET_X = (-0.085, -0.015)
CABLE_POCKET_Y = (-0.055, 0.055)

COIL_CENTER = (-0.05, 0.015)
COIL_R = (0.028, 0.033)  # two coiled circular-needle cables
COIL_TUBE_R = 0.0016
COIL_Z = POCKET_SHEET_TOP + COIL_TUBE_R - EMBED  # seated on the pocket sheet

# Wrap tie strap
WRAP_TIE_LEN = 0.22
WRAP_TIE_W = 0.016

STRAP_T = 0.003


# ----------------------------------------------------------------------------
# CadQuery helpers
# ----------------------------------------------------------------------------
def _rounded_panel(x_center: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(PANEL_W, PANEL_H, LEATHER_T)
        .edges("|Z")
        .fillet(0.008)
        .translate((x_center, 0.0, 0.0))
    )


def _channel_strip_at(
    channel_xs: tuple[float, ...],
    strip_x_min: float,
    strip_x_max: float,
    strip_y_min: float,
    strip_y_max: float,
    panel_top_z: float,
) -> cq.Workplane:
    """Stitched leather channel strip with cut-through needle tunnels."""
    cx = (strip_x_min + strip_x_max) / 2.0
    cy = (strip_y_min + strip_y_max) / 2.0
    strip_bot = panel_top_z - EMBED
    height = STRIP_TOP_OFFSET + EMBED
    strip = (
        cq.Workplane("XY")
        .box(strip_x_max - strip_x_min, strip_y_max - strip_y_min, height)
        .translate((cx, cy, strip_bot + height / 2.0))
    )
    slot_bot = panel_top_z - 0.001
    slot_h = CHANNEL_SLOT_H + 0.001
    slot_len = (strip_y_max - strip_y_min) + 0.01
    for ch_x in channel_xs:
        slot = (
            cq.Workplane("XY")
            .box(CHANNEL_SLOT_W, slot_len, slot_h)
            .translate((ch_x, cy, slot_bot + slot_h / 2.0))
        )
        strip = strip.cut(slot)
    return strip


def _pocket(x_range: tuple[float, float], y_range: tuple[float, float]) -> cq.Workplane:
    """Open-top leather pocket: raised sheet plus bottom and side gusset walls."""
    x0, x1 = x_range
    y0, y1 = y_range
    wall_t = 0.004
    wall_z0 = PP_TOP - EMBED
    wall_z1 = PP_TOP + POCKET_WALL_H + EMBED
    sheet = (
        cq.Workplane("XY")
        .box(x1 - x0, y1 - y0, POCKET_SHEET_T)
        .translate(((x0 + x1) / 2.0, (y0 + y1) / 2.0, POCKET_SHEET_TOP - POCKET_SHEET_T / 2.0))
    )
    for wx0, wx1, wy0, wy1 in (
        (x0, x0 + wall_t, y0, y1),
        (x1 - wall_t, x1, y0, y1),
        (x0, x1, y0, y0 + wall_t),
    ):
        wall = (
            cq.Workplane("XY")
            .box(wx1 - wx0, wy1 - wy0, wall_z1 - wall_z0)
            .translate(((wx0 + wx1) / 2.0, (wy0 + wy1) / 2.0, (wall_z0 + wall_z1) / 2.0))
        )
        sheet = sheet.union(wall)
    return sheet


def _cable_coil() -> cq.Shape:
    coil = cq.Solid.makeTorus(COIL_R[0], COIL_TUBE_R)
    return coil.fuse(cq.Solid.makeTorus(COIL_R[1], COIL_TUBE_R))


def _wrap_tie_strap() -> cq.Workplane:
    """Long narrow leather tie strap extending along +X from its hinge."""
    return (
        cq.Workplane("XY")
        .box(WRAP_TIE_LEN, WRAP_TIE_W, LEATHER_T)
        .edges("|Z")
        .fillet(0.004)
        .translate((WRAP_TIE_LEN / 2.0, 0.0, 0.0))
    )


def _strap_segment(
    y0: float,
    z0: float,
    y1: float,
    z1: float,
    thickness: float,
    width: float,
) -> cq.Workplane:
    """One convex constant-thickness leather segment from a YZ side profile."""
    pts = [(y0, z0), (y1, z1), (y1, z1 + thickness), (y0, z0 + thickness)]
    return (
        cq.Workplane("YZ")
        .polyline(pts)
        .close()
        .extrude(width)
        .translate((-width / 2.0, 0.0, 0.0))
    )


def _add_segmented_strap(
    part,
    knots: list[tuple[float, float]],
    seg_names: list[str],
    width: float,
    material,
) -> None:
    for (y0, z0), (y1, z1), seg_name in zip(knots[:-1], knots[1:], seg_names):
        part.visual(
            mesh_from_cadquery(
                _strap_segment(y0, z0, y1, z1, STRAP_T, width),
                f"{part.name}_{seg_name}",
            ),
            material=material,
            name=seg_name,
        )


# Strap knot presets (unchanged from parent)
POCKET_STRAP_KNOTS = [(0.0, 0.0), (-0.006, 0.0), (-0.012, 0.0055), (-0.092, 0.0055)]
POCKET_STRAP_SEGS = ["hinge_tab", "ramp", "snap_run"]

CABLE_STRAP_KNOTS = [
    (0.0, 0.0),
    (-0.002, 0.0),
    (-0.006, 0.0092),
    (-0.0255, 0.0092),
    (-0.0295, 0.0055),
    (-0.0415, 0.0055),
]
CABLE_STRAP_SEGS = ["hinge_tab", "rise", "cable_bridge", "descent", "snap_tab"]

TIP_FLAP_KNOTS = [(0.0, 0.0), (-0.010, 0.0), (-0.016, 0.0045), (-0.052, 0.0045)]
TIP_FLAP_SEGS = ["stitch_tab", "ramp", "tip_tongue"]


def _add_snap_stud(part, x: float, y: float, base_top_z: float, brass, tag: str) -> None:
    """Two stacked brass discs reading as a snap fastener."""
    base_h, cap_h = 0.0016, 0.0014
    part.visual(
        Cylinder(radius=0.0055, length=base_h),
        origin=Origin(xyz=(x, y, base_top_z - EMBED + base_h / 2.0)),
        material=brass,
        name=f"snap_base_{tag}",
    )
    part.visual(
        Cylinder(radius=0.0038, length=cap_h),
        origin=Origin(xyz=(x, y, base_top_z - EMBED + base_h - 0.0002 + cap_h / 2.0)),
        material=brass,
        name=f"snap_cap_{tag}",
    )


# ----------------------------------------------------------------------------
# Shared panel and needle-bed helpers
# ----------------------------------------------------------------------------
def _add_panel_sheet(panel, x_center: float, leather_mat) -> None:
    """Add the rounded leather sheet visual to a panel part.

    Root panels use an upward shift so their bottom sits at z=0.
    Child panels (hinged) stay centered at local z=0.
    """
    panel.visual(
        mesh_from_cadquery(
            _rounded_panel(x_center).translate((0.0, 0.0, LEATHER_T / 2.0)),
            f"{panel.name}_sheet",
        ),
        material=leather_mat,
        name=f"{panel.name}_sheet",
    )


def _add_panel_sheet_centered(panel, x_center: float, leather_mat) -> None:
    """Child panel sheet centered at local z=0 (top at LEATHER_T/2)."""
    panel.visual(
        mesh_from_cadquery(
            _rounded_panel(x_center),
            f"{panel.name}_sheet",
        ),
        material=leather_mat,
        name=f"{panel.name}_sheet",
    )


def _add_needle_bed(
    model,
    parent_panel,
    channel_xs: tuple[float, ...],
    strip_x_range: tuple[float, float],
    strip_y_range: tuple[float, float],
    panel_top_z: float,
    leather_light,
    bamboo,
    ci_offset: int = 0,
) -> list:
    """Add a channel strip visual and prismatic slide needles to a panel.

    Returns the list of needle parts created.
    """
    strip_x_min, strip_x_max = strip_x_range
    strip_y_min, strip_y_max = strip_y_range

    parent_panel.visual(
        mesh_from_cadquery(
            _channel_strip_at(channel_xs, strip_x_min, strip_x_max, strip_y_min, strip_y_max, panel_top_z),
            f"{parent_panel.name}_channel_strip",
        ),
        material=leather_light,
        name="channel_strip",
    )

    needles = []
    for ci, ch_x in enumerate(channel_xs):
        for si, off in enumerate(NEEDLE_OFFSETS):
            name = f"needle_{ci + ci_offset}_{si}"
            needle = model.part(name)
            needle.visual(
                Cylinder(radius=NEEDLE_R, length=NEEDLE_LEN),
                origin=Origin(xyz=(0.0, 0.075, 0.0), rpy=(1.5707963, 0.0, 0.0)),
                material=bamboo,
                name="needle_shaft",
            )
            model.articulation(
                f"{name}_slide",
                ArticulationType.PRISMATIC,
                parent=parent_panel,
                child=needle,
                origin=Origin(xyz=(ch_x + off, strip_y_min, panel_top_z + NEEDLE_R)),
                axis=(0.0, -1.0, 0.0),
                motion_limits=MotionLimits(
                    effort=2.0, velocity=0.5, lower=0.0, upper=NEEDLE_TRAVEL
                ),
            )
            needles.append(needle)
    return needles


# ----------------------------------------------------------------------------
# Model
# ----------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="leather_needle_case_roll")

    leather_tan = model.material("leather_tan", rgba=(0.70, 0.42, 0.19, 1.0))
    leather_light = model.material("leather_light", rgba=(0.79, 0.55, 0.32, 1.0))
    bamboo = model.material("bamboo", rgba=(0.83, 0.70, 0.45, 1.0))
    brass = model.material("brass", rgba=(0.62, 0.51, 0.27, 1.0))
    steel = model.material("steel_cable", rgba=(0.72, 0.73, 0.75, 1.0))

    # --- Root: needle-side panel -----------------------------------------
    needle_panel = model.part("needle_panel")
    _add_panel_sheet(needle_panel, PANEL_W / 2.0, leather_tan)

    # Needle bed on needle_panel: 4 channels, 8 needles (ci 0-3)
    _add_needle_bed(
        model, needle_panel,
        channel_xs=CHANNEL_XS,
        strip_x_range=(STRIP_X_MIN, STRIP_X_MAX),
        strip_y_range=(STRIP_Y_MIN, STRIP_Y_MAX),
        panel_top_z=PANEL_TOP,
        leather_light=leather_light,
        bamboo=bamboo,
        ci_offset=0,
    )

    # --- Retaining flap over the needle tips ------------------------------
    tip_flap = model.part("tip_flap")
    _add_segmented_strap(tip_flap, TIP_FLAP_KNOTS, TIP_FLAP_SEGS, 0.16, leather_light)
    model.articulation(
        "tip_flap_hinge",
        ArticulationType.REVOLUTE,
        parent=needle_panel,
        child=tip_flap,
        origin=Origin(xyz=(0.1125, 0.105, PANEL_TOP)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=0.0, upper=1.9),
    )

    # --- Wrap tie strap (replaces cover_flap) on +X outer edge -----------
    wrap_tie = model.part("wrap_tie")
    wrap_tie.visual(
        mesh_from_cadquery(_wrap_tie_strap(), "wrap_tie_strap"),
        material=leather_tan,
        name="wrap_tie_strap",
    )
    model.articulation(
        "wrap_tie_hinge",
        ArticulationType.REVOLUTE,
        parent=needle_panel,
        child=wrap_tie,
        origin=Origin(xyz=(PANEL_W, 0.0, LEATHER_T / 2.0)),
        # Strap extends along local +X; positive q swings it up and over.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=3.0),
    )

    # --- Middle panel on fold_hinge (first spine fold) -------------------
    middle_panel = model.part("middle_panel")
    _add_panel_sheet_centered(middle_panel, -PANEL_W / 2.0, leather_tan)

    # Needle bed on middle_panel: 2 channels, 4 needles (ci 4-5)
    _add_needle_bed(
        model, middle_panel,
        channel_xs=M_CHANNEL_XS,
        strip_x_range=(M_STRIP_X_MIN, M_STRIP_X_MAX),
        strip_y_range=(STRIP_Y_MIN, STRIP_Y_MAX),
        panel_top_z=CHILD_PANEL_TOP,
        leather_light=leather_light,
        bamboo=bamboo,
        ci_offset=4,
    )

    model.articulation(
        "fold_hinge",
        ArticulationType.REVOLUTE,
        parent=needle_panel,
        child=middle_panel,
        origin=Origin(xyz=(0.0, 0.0, LEATHER_T / 2.0)),
        # Middle panel extends along local -X; positive q folds it up.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=2.1),
    )

    # --- Pocket panel on fold_hinge_2 (second spine fold) ----------------
    pocket_panel = model.part("pocket_panel")
    _add_panel_sheet_centered(pocket_panel, -PANEL_W / 2.0, leather_tan)
    pocket_panel.visual(
        mesh_from_cadquery(_pocket(FLAT_POCKET_X, FLAT_POCKET_Y), "flat_pocket"),
        material=leather_light,
        name="flat_pocket",
    )
    pocket_panel.visual(
        mesh_from_cadquery(_pocket(CABLE_POCKET_X, CABLE_POCKET_Y), "cable_pocket"),
        material=leather_light,
        name="cable_pocket",
    )
    pocket_panel.visual(
        mesh_from_cadquery(_cable_coil(), "cable_coil"),
        origin=Origin(xyz=(COIL_CENTER[0], COIL_CENTER[1], COIL_Z)),
        material=steel,
        name="cable_coil",
    )

    model.articulation(
        "fold_hinge_2",
        ArticulationType.REVOLUTE,
        parent=middle_panel,
        child=pocket_panel,
        origin=Origin(xyz=(-PANEL_W, 0.0, 0.0)),
        # Pocket panel extends along local -X; positive q folds it up.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=2.1),
    )

    # --- Flat pocket snap strap ------------------------------------------
    pocket_strap = model.part("pocket_strap")
    _add_segmented_strap(pocket_strap, POCKET_STRAP_KNOTS, POCKET_STRAP_SEGS, 0.03, leather_light)
    _add_snap_stud(pocket_strap, 0.0, -0.082, 0.0085, brass, tag="0")
    model.articulation(
        "pocket_strap_hinge",
        ArticulationType.REVOLUTE,
        parent=pocket_panel,
        child=pocket_strap,
        origin=Origin(xyz=(-0.15, 0.06, PP_TOP)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=0.0, upper=1.7),
    )

    # --- Cable pocket snap strap -----------------------------------------
    cable_strap = model.part("cable_strap")
    _add_segmented_strap(cable_strap, CABLE_STRAP_KNOTS, CABLE_STRAP_SEGS, 0.024, leather_light)
    _add_snap_stud(cable_strap, 0.0, -0.036, 0.0085, brass, tag="0")
    model.articulation(
        "cable_strap_hinge",
        ArticulationType.REVOLUTE,
        parent=pocket_panel,
        child=cable_strap,
        origin=Origin(xyz=(COIL_CENTER[0], 0.062, PP_TOP)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=0.0, upper=1.7),
    )

    return model


# ----------------------------------------------------------------------------
# Tests
# ----------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    needle_panel = object_model.get_part("needle_panel")
    middle_panel = object_model.get_part("middle_panel")
    pocket_panel = object_model.get_part("pocket_panel")
    tip_flap = object_model.get_part("tip_flap")
    wrap_tie = object_model.get_part("wrap_tie")
    pocket_strap = object_model.get_part("pocket_strap")
    cable_strap = object_model.get_part("cable_strap")
    needle = object_model.get_part("needle_0_0")
    m_needle = object_model.get_part("needle_4_0")

    # --- Three-panel chain structure ---
    fold_hinge = object_model.get_articulation("fold_hinge")
    fold_hinge_2 = object_model.get_articulation("fold_hinge_2")

    ctx.check(
        "fold_hinge connects needle_panel to middle_panel",
        fold_hinge.parent == "needle_panel" and fold_hinge.child == "middle_panel",
        details=f"parent={fold_hinge.parent}, child={fold_hinge.child}",
    )
    ctx.check(
        "fold_hinge_2 connects middle_panel to pocket_panel",
        fold_hinge_2.parent == "middle_panel" and fold_hinge_2.child == "pocket_panel",
        details=f"parent={fold_hinge_2.parent}, child={fold_hinge_2.child}",
    )

    # Twelve needles total: 8 on needle_panel (ci 0-3) + 4 on middle_panel (ci 4-5)
    needle_names = [
        p.name
        for p in object_model.parts
        if p.name.startswith("needle_") and p.name != "needle_panel"
    ]
    ctx.check(
        "twelve double-pointed needles across two panel beds",
        len(needle_names) == 12,
        details=f"found {len(needle_names)}: {sorted(needle_names)}",
    )

    # Middle panel carries its own needle bed (channel_strip visual exists)
    m_strip = middle_panel.get_visual("channel_strip")
    ctx.check(
        "middle_panel has a channel_strip needle bed",
        m_strip is not None,
        details="channel_strip visual not found on middle_panel",
    )

    # Open rest pose: needle_panel meets middle_panel edge to edge at x=0
    ctx.expect_gap(
        needle_panel,
        middle_panel,
        axis="x",
        min_gap=-0.0005,
        max_gap=0.0010,
        positive_elem="needle_panel_sheet",
        negative_elem="middle_panel_sheet",
        name="needle_panel meets middle_panel at the fold line",
    )

    # Middle panel meets pocket_panel edge to edge at x=-0.21
    ctx.expect_gap(
        middle_panel,
        pocket_panel,
        axis="x",
        min_gap=-0.0005,
        max_gap=0.0010,
        positive_elem="middle_panel_sheet",
        negative_elem="pocket_panel_sheet",
        name="middle_panel meets pocket_panel at the second fold line",
    )

    # Needles rest on the panel and pass through the channel strip
    ctx.expect_gap(
        needle,
        needle_panel,
        axis="z",
        min_gap=-0.0002,
        max_gap=0.0006,
        negative_elem="needle_panel_sheet",
        name="needle rests on the leather panel",
    )
    ctx.expect_overlap(
        needle,
        needle_panel,
        axes="y",
        elem_b="channel_strip",
        min_overlap=0.05,
        name="needle is threaded through the channel strip",
    )

    # Middle-panel needle rests on the middle panel
    ctx.expect_gap(
        m_needle,
        middle_panel,
        axis="z",
        min_gap=-0.0002,
        max_gap=0.0006,
        negative_elem="middle_panel_sheet",
        name="middle-panel needle rests on the panel",
    )

    # Retaining flap covers the needle tips
    ctx.expect_overlap(
        tip_flap,
        needle,
        axes="y",
        min_overlap=0.02,
        name="retaining flap overlaps the needle tips",
    )

    # Straps rest in contact with the pockets
    ctx.expect_contact(
        pocket_strap,
        pocket_panel,
        contact_tol=2e-4,
        name="flat pocket strap lies against the pocket",
    )
    ctx.expect_contact(
        cable_strap,
        pocket_panel,
        contact_tol=2e-4,
        name="cable strap lies against the cable pocket",
    )

    # Cable coil seated on the cable pocket sheet
    coil_aabb = ctx.part_element_world_aabb(pocket_panel, elem="cable_coil")
    sheet_aabb = ctx.part_element_world_aabb(pocket_panel, elem="cable_pocket")
    ctx.check(
        "cable coil is seated on the cable pocket sheet",
        coil_aabb is not None
        and sheet_aabb is not None
        and coil_aabb[0][2] <= sheet_aabb[1][2] + 1e-4
        and coil_aabb[1][2] > sheet_aabb[1][2],
        details=f"coil={coil_aabb}, sheet={sheet_aabb}",
    )

    # Wrap tie extends from the outer edge at rest
    ctx.expect_gap(
        wrap_tie,
        needle_panel,
        axis="x",
        min_gap=-0.0005,
        max_gap=0.0010,
        positive_elem="wrap_tie_strap",
        negative_elem="needle_panel_sheet",
        name="wrap tie meets needle_panel at the outer edge",
    )

    # fold_hinge lifts middle_panel upward when folded
    rest_mid = ctx.part_world_aabb(middle_panel)
    with ctx.pose({fold_hinge: 1.2}):
        folded_mid = ctx.part_world_aabb(middle_panel)
    ctx.check(
        "fold_hinge lifts middle_panel upward",
        rest_mid is not None
        and folded_mid is not None
        and folded_mid[1][2] > 0.15
        and rest_mid[1][2] < 0.02,
        details=f"rest={rest_mid}, folded={folded_mid}",
    )

    # fold_hinge_2 lifts pocket_panel upward relative to middle_panel
    rest_pock = ctx.part_world_aabb(pocket_panel)
    with ctx.pose({fold_hinge_2: 1.2}):
        folded_pock = ctx.part_world_aabb(pocket_panel)
    ctx.check(
        "fold_hinge_2 lifts pocket_panel upward",
        rest_pock is not None
        and folded_pock is not None
        and folded_pock[1][2] > 0.15
        and rest_pock[1][2] < 0.02,
        details=f"rest={rest_pock}, folded={folded_pock}",
    )

    # Needle slide: pulls needle out toward -Y
    slide = object_model.get_articulation("needle_0_0_slide")
    rest_pos = ctx.part_world_position(needle)
    with ctx.pose({slide: NEEDLE_TRAVEL}):
        out_pos = ctx.part_world_position(needle)
        ctx.expect_overlap(
            needle,
            needle_panel,
            axes="y",
            elem_b="channel_strip",
            min_overlap=0.03,
            name="extended needle stays engaged in the channel strip",
        )
    ctx.check(
        "needle slides out toward -Y",
        rest_pos is not None and out_pos is not None and out_pos[1] < rest_pos[1] - 0.05,
        details=f"rest={rest_pos}, out={out_pos}",
    )

    # Hinged flaps and straps lift when opened
    for joint_name, flap_part, min_top in (
        ("tip_flap_hinge", tip_flap, 0.02),
        ("wrap_tie_hinge", wrap_tie, 0.03),
        ("pocket_strap_hinge", pocket_strap, 0.05),
        ("cable_strap_hinge", cable_strap, 0.02),
    ):
        jnt = object_model.get_articulation(joint_name)
        with ctx.pose({jnt: 1.2}):
            open_aabb = ctx.part_world_aabb(flap_part)
        ctx.check(
            f"{joint_name} lifts {flap_part.name} upward",
            open_aabb is not None and open_aabb[1][2] > min_top,
            details=f"open={open_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
