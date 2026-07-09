"""Open tan leather knitting needle case with articulated fold, straps, flaps, and needles.

Layout (world frame, case lying open on the XY plane, +Z up):
- The needle-side half panel (x in [0, 0.21]) is the root. It carries a stitched
  leather channel strip with six open-bottom tunnels holding twelve bamboo
  double-pointed needles (prismatic, pull out toward -Y) and a hinged retaining
  flap over the needle tips.
- The pocket-side half panel (x in [-0.21, 0]) hangs on the central fold hinge.
  It carries a flat leather pocket with a hinged snap strap, and a cable pocket
  holding two coiled circular-needle steel cables pinned by a hinged snap strap.
- A snap cover flap is hinged along the outer +X edge of the needle panel.
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
PANEL_W = 0.21  # each half panel width along X
PANEL_H = 0.22  # panel height along Y (y in [-0.11, 0.11])
PANEL_TOP = LEATHER_T  # world z of the panel top face
EMBED = 0.0003  # small same-part embed so mounted visuals stay connected

# Needle channels (on the root needle panel)
CHANNEL_COUNT = 6
CHANNEL_PITCH = 0.030  # center-to-center spacing between tunnels
CHANNEL_X0 = 0.035  # x of the first tunnel center
CHANNEL_XS = tuple(round(CHANNEL_X0 + i * CHANNEL_PITCH, 4) for i in range(CHANNEL_COUNT))
CHANNEL_SLOT_W = 0.013
CHANNEL_SLOT_H = 0.0045  # slot height above the panel top
STRIP_X_MIN, STRIP_X_MAX = 0.020, 0.200
STRIP_Y_MIN, STRIP_Y_MAX = -0.075, 0.005
STRIP_TOP = PANEL_TOP + 0.0055

# Bamboo needles
NEEDLE_R = 0.002
NEEDLE_LEN = 0.17  # double-pointed needles, y in [-0.085, 0.085] at rest
NEEDLE_AXIS_Z = PANEL_TOP + NEEDLE_R  # resting on the panel top
NEEDLE_OFFSETS = (-0.003, 0.003)  # two needles per channel
NEEDLE_TRAVEL = 0.095  # slide-out travel toward -Y, tunnel stays engaged

# Pocket-side geometry (in the pocket_panel frame: z = 0 at panel mid-plane)
PP_TOP = LEATHER_T / 2.0  # panel top in the pocket_panel frame
POCKET_WALL_H = 0.003
POCKET_SHEET_T = 0.0025
POCKET_SHEET_TOP = PP_TOP + POCKET_WALL_H + POCKET_SHEET_T  # 0.007

FLAT_POCKET_X = (-0.195, -0.105)
FLAT_POCKET_Y = (-0.065, 0.045)
CABLE_POCKET_X = (-0.085, -0.015)
CABLE_POCKET_Y = (-0.055, 0.055)

COIL_CENTER = (-0.05, 0.015)
COIL_R = (0.028, 0.033)  # two coiled circular-needle cables
COIL_TUBE_R = 0.0016
COIL_Z = POCKET_SHEET_TOP + COIL_TUBE_R - EMBED  # seated on the pocket sheet

# Cover flap (outer +X edge)
COVER_FLAP_W = 0.06

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


def _channel_strip() -> cq.Workplane:
    cx = (STRIP_X_MIN + STRIP_X_MAX) / 2.0
    cy = (STRIP_Y_MIN + STRIP_Y_MAX) / 2.0
    height = STRIP_TOP - (PANEL_TOP - EMBED)
    strip = (
        cq.Workplane("XY")
        .box(STRIP_X_MAX - STRIP_X_MIN, STRIP_Y_MAX - STRIP_Y_MIN, height)
        .translate((cx, cy, (PANEL_TOP - EMBED) + height / 2.0))
    )
    for ch_x in CHANNEL_XS:
        slot = (
            cq.Workplane("XY")
            .box(CHANNEL_SLOT_W, (STRIP_Y_MAX - STRIP_Y_MIN) + 0.01, CHANNEL_SLOT_H + 0.001)
            .translate((ch_x, cy, PANEL_TOP + (CHANNEL_SLOT_H + 0.001) / 2.0 - 0.001))
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
        (x0, x0 + wall_t, y0, y1),  # one side gusset
        (x1 - wall_t, x1, y0, y1),  # other side gusset
        (x0, x1, y0, y0 + wall_t),  # bottom gusset (opening stays at high Y)
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


def _strap_segment(
    y0: float,
    z0: float,
    y1: float,
    z1: float,
    thickness: float,
    width: float,
) -> cq.Workplane:
    """One convex constant-thickness leather segment from a YZ side profile.

    Straps and flaps are authored as chains of convex segments (tab, ramp, run)
    so element-level overlap QC sees tight convex hulls instead of one concave
    bridge shape whose hull would falsely fill the space it spans over.
    """
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


# Hinge tab on the panel, ramp up before the pocket edge, long run on the sheet.
POCKET_STRAP_KNOTS = [(0.0, 0.0), (-0.006, 0.0), (-0.012, 0.0055), (-0.092, 0.0055)]
POCKET_STRAP_SEGS = ["hinge_tab", "ramp", "snap_run"]

# Hinge tab on the panel, hump bridging over the coiled cables, tab on the sheet.
CABLE_STRAP_KNOTS = [
    (0.0, 0.0),
    (-0.002, 0.0),
    (-0.006, 0.0092),
    (-0.0255, 0.0092),
    (-0.0295, 0.0055),
    (-0.0415, 0.0055),
]
CABLE_STRAP_SEGS = ["hinge_tab", "rise", "cable_bridge", "descent", "snap_tab"]

# Stitched tab on the panel near the top edge, raised tongue over the needle tips.
TIP_FLAP_KNOTS = [(0.0, 0.0), (-0.010, 0.0), (-0.016, 0.0045), (-0.052, 0.0045)]
TIP_FLAP_SEGS = ["stitch_tab", "ramp", "tip_tongue"]


def _cover_flap() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(COVER_FLAP_W, PANEL_H, LEATHER_T)
        .edges("|Z")
        .fillet(0.008)
        .translate((COVER_FLAP_W / 2.0, 0.0, 0.0))
    )


def _add_snap_stud(part, x: float, y: float, base_top_z: float, brass, tag: str) -> None:
    """Two stacked brass discs reading as a snap fastener, embedded into the leather."""
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
# Model
# ----------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="leather_needle_case")

    leather_tan = model.material("leather_tan", rgba=(0.70, 0.42, 0.19, 1.0))
    leather_light = model.material("leather_light", rgba=(0.79, 0.55, 0.32, 1.0))
    bamboo = model.material("bamboo", rgba=(0.83, 0.70, 0.45, 1.0))
    brass = model.material("brass", rgba=(0.62, 0.51, 0.27, 1.0))
    steel = model.material("steel_cable", rgba=(0.72, 0.73, 0.75, 1.0))

    # --- Root: needle-side half panel -------------------------------------
    needle_panel = model.part("needle_panel")
    needle_panel.visual(
        mesh_from_cadquery(
            _rounded_panel(PANEL_W / 2.0).translate((0.0, 0.0, LEATHER_T / 2.0)),
            "needle_panel_sheet",
        ),
        material=leather_tan,
        name="needle_panel_sheet",
    )
    needle_panel.visual(
        mesh_from_cadquery(_channel_strip(), "channel_strip"),
        material=leather_light,
        name="channel_strip",
    )

    # --- Bamboo double-pointed needles (prismatic, pull out toward -Y) ----
    for ci, ch_x in enumerate(CHANNEL_XS):
        for si, off in enumerate(NEEDLE_OFFSETS):
            needle = model.part(f"needle_{ci}_{si}")
            needle.visual(
                Cylinder(radius=NEEDLE_R, length=NEEDLE_LEN),
                origin=Origin(xyz=(0.0, 0.075, 0.0), rpy=(1.5707963, 0.0, 0.0)),
                material=bamboo,
                name="needle_shaft",
            )
            model.articulation(
                f"needle_{ci}_{si}_slide",
                ArticulationType.PRISMATIC,
                parent=needle_panel,
                child=needle,
                # Joint frame at the channel tunnel entry on the pull-out side.
                origin=Origin(xyz=(ch_x + off, STRIP_Y_MIN, NEEDLE_AXIS_Z)),
                axis=(0.0, -1.0, 0.0),
                motion_limits=MotionLimits(
                    effort=2.0, velocity=0.5, lower=0.0, upper=NEEDLE_TRAVEL
                ),
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
        # Flap extends along local -Y; positive q lifts the free edge upward.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=0.0, upper=1.9),
    )

    # --- Snap cover flap on the outer +X edge ------------------------------
    cover_flap = model.part("cover_flap")
    cover_flap.visual(
        mesh_from_cadquery(_cover_flap(), "cover_flap"),
        material=leather_tan,
        name="cover_flap_sheet",
    )
    for i, sy in enumerate((-0.055, 0.055)):
        _add_snap_stud(cover_flap, 0.043, sy, LEATHER_T / 2.0, brass, tag=str(i))
    model.articulation(
        "cover_flap_hinge",
        ArticulationType.REVOLUTE,
        parent=needle_panel,
        child=cover_flap,
        origin=Origin(xyz=(PANEL_W, 0.0, LEATHER_T / 2.0)),
        # Flap extends along local +X; positive q folds it up and over.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=2.6),
    )

    # --- Pocket-side half panel on the central fold hinge ------------------
    pocket_panel = model.part("pocket_panel")
    pocket_panel.visual(
        mesh_from_cadquery(_rounded_panel(-PANEL_W / 2.0), "pocket_panel_sheet"),
        material=leather_tan,
        name="pocket_panel_sheet",
    )
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
        "fold_hinge",
        ArticulationType.REVOLUTE,
        parent=needle_panel,
        child=pocket_panel,
        origin=Origin(xyz=(0.0, 0.0, LEATHER_T / 2.0)),
        # Pocket panel extends along local -X; positive q folds it up toward closed.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=2.1),
    )

    # --- Flat pocket snap strap --------------------------------------------
    pocket_strap = model.part("pocket_strap")
    _add_segmented_strap(pocket_strap, POCKET_STRAP_KNOTS, POCKET_STRAP_SEGS, 0.03, leather_light)
    _add_snap_stud(pocket_strap, 0.0, -0.082, 0.0085, brass, tag="0")
    model.articulation(
        "pocket_strap_hinge",
        ArticulationType.REVOLUTE,
        parent=pocket_panel,
        child=pocket_strap,
        origin=Origin(xyz=(-0.15, 0.06, PP_TOP)),
        # Strap extends along local -Y; positive q lifts the snap end upward.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=0.0, upper=1.7),
    )

    # --- Cable pocket snap strap (bridges over the coiled cables) ----------
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
    pocket_panel = object_model.get_part("pocket_panel")
    tip_flap = object_model.get_part("tip_flap")
    cover_flap = object_model.get_part("cover_flap")
    pocket_strap = object_model.get_part("pocket_strap")
    cable_strap = object_model.get_part("cable_strap")
    needle = object_model.get_part("needle_0_0")

    # Twelve bamboo needles exist, two per stitched channel (6 tunnels).
    needle_names = [
        p.name
        for p in object_model.parts
        if p.name.startswith("needle_") and p.name != "needle_panel"
    ]
    ctx.check(
        "twelve double-pointed needles are present (6 channels x 2 needles)",
        len(needle_names) == 12,
        details=f"found {sorted(needle_names)}",
    )

    # All 6 channel tunnels emit exactly two prismatic-slide needle joints.
    channel_slide_names = [
        a.name
        for a in object_model.articulations
        if a.name.endswith("_slide")
    ]
    ctx.check(
        "six channel tunnels each have two needle slide joints",
        len(channel_slide_names) == 12,
        details=f"found {sorted(channel_slide_names)}",
    )
    # Verify tunnels span 0..5 along the channel index axis.
    channel_indices = sorted(
        {int(n.split("_")[1]) for n in channel_slide_names}
    )
    ctx.check(
        "channel indices run from 0 to 5 (6 evenly spaced tunnels)",
        channel_indices == list(range(CHANNEL_COUNT)),
        details=f"indices={channel_indices}",
    )

    # Open rest pose: the two half panels lie coplanar and meet at the fold line.
    ctx.expect_gap(
        needle_panel,
        pocket_panel,
        axis="x",
        min_gap=-0.0005,
        max_gap=0.0010,
        positive_elem="needle_panel_sheet",
        negative_elem="pocket_panel_sheet",
        name="half panels meet edge to edge at the fold line",
    )

    # Needles rest on the panel and pass through the stitched channel strip.
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

    # Retaining flap covers the needle tips with clearance at rest.
    ctx.expect_overlap(
        tip_flap,
        needle,
        axes="y",
        min_overlap=0.02,
        name="retaining flap overlaps the needle tips",
    )

    # Straps rest in contact with the pockets they close over.
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

    # Coiled cables sit seated on the cable pocket sheet.
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

    # Cover flap continues flush from the outer panel edge at rest.
    ctx.expect_gap(
        cover_flap,
        needle_panel,
        axis="x",
        min_gap=-0.0005,
        max_gap=0.0010,
        positive_elem="cover_flap_sheet",
        negative_elem="needle_panel_sheet",
        name="cover flap meets the outer panel edge",
    )

    # Fold hinge: folding lifts the pocket panel free edge up and over.
    fold = object_model.get_articulation("fold_hinge")
    rest_aabb = ctx.part_world_aabb(pocket_panel)
    with ctx.pose({fold: 1.2}):
        folded_aabb = ctx.part_world_aabb(pocket_panel)
    ctx.check(
        "fold hinge lifts the pocket panel upward",
        rest_aabb is not None
        and folded_aabb is not None
        and folded_aabb[1][2] > 0.15
        and rest_aabb[1][2] < 0.02,
        details=f"rest={rest_aabb}, folded={folded_aabb}",
    )

    # Needle slide: pulls the needle out toward -Y while staying in its channel.
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

    # Hinged flaps and straps all lift their free edges when opened.
    for joint_name, part, min_top in (
        ("tip_flap_hinge", tip_flap, 0.02),
        ("cover_flap_hinge", cover_flap, 0.03),
        ("pocket_strap_hinge", pocket_strap, 0.05),
        ("cable_strap_hinge", cable_strap, 0.02),
    ):
        joint = object_model.get_articulation(joint_name)
        with ctx.pose({joint: 1.2}):
            open_aabb = ctx.part_world_aabb(part)
        ctx.check(
            f"{joint_name} lifts {part.name} upward",
            open_aabb is not None and open_aabb[1][2] > min_top,
            details=f"open={open_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
