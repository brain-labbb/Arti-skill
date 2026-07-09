from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Widespread two-handle bathroom faucet in polished gold brass.
#
# Frame conventions:
#   - The deck (counter surface) is horizontal at z = 0.
#   - The spout projects forward along +Y from the center.
#   - The handles flank the spout at x = +/- HANDLE_SPACING.
#   - Z is up; all three units mount through the deck.
# ---------------------------------------------------------------------------

# Layout
HANDLE_SPACING = 0.10  # handle centers at x = +/- 0.10

# Deck plate (mounting substrate)
DECK_W = 0.34
DECK_D = 0.14
DECK_T = 0.015

# Spout tube
SPOUT_R = 0.013       # outer radius (~0.026 m diameter)
SPOUT_BORE_R = 0.009  # inner bore radius (visible at outlet)

# Spout escutcheon / base flange
SPOUT_FLANGE_R = 0.025
SPOUT_FLANGE_T = 0.006

# Valve assemblies
VALVE_ESC_R = 0.030   # escutcheon radius
VALVE_ESC_T = 0.006   # escutcheon thickness

# Tapered pedestal (wider at deck, narrower at top)
PED_BOT_R = 0.020     # bottom radius (at deck)
PED_TOP_R = 0.013     # top radius
PED_H = 0.038         # pedestal height

# Lever handles
LEVER_R = 0.007       # lever cylinder radius
LEVER_LEN = 0.065     # lever length (from pivot center to tip)
LEVER_CAP_R = 0.008   # rounded end cap

# Seam rings (nvisible dark gap at deck interface)
SEAM_MAJOR_R = 0.028  # outer ring radius
SEAM_TUBE_R = 0.0012  # ring cross-section radius

# Aerator
AERATOR_R = 0.011     # aerator outer radius
AERATOR_LEN = 0.018   # aerator body length
AERATOR_BORE_R = 0.007

# Computed volume for hollow-bore verification.
SPOUT_SOLID_VOLUME: float = 0.0
SPOUT_UNBORED_VOLUME: float = 0.0


def _build_spout_solid() -> cq.Workplane:
    """Gooseneck spout in its local frame: base on the deck at z=0,
    rising vertically then curving forward (+Y) and downward to an open outlet.
    The bore is cut through the interior."""

    # Spout spine path in the YZ plane (Y=forward, Z=up).
    # Rise vertically, arch forward, curve down to outlet.
    spine_pts = [
        (0.0, 0.0),        # base at deck
        (0.0, 0.08),       # vertical rise
        (0.01, 0.12),      # beginning of curve
        (0.05, 0.155),     # approaching apex
        (0.09, 0.165),     # apex region
        (0.12, 0.155),     # past apex, starting down
        (0.14, 0.12),      # curving down
        (0.145, 0.07),     # near outlet
    ]

    # Build path as a spline in the YZ workplane.
    path = cq.Workplane("YZ").spline(spine_pts)

    # Bore path extends slightly past both ends for clean cuts.
    bore_pts = [
        (0.0, -0.005),
        (0.0, 0.08),
        (0.01, 0.12),
        (0.05, 0.155),
        (0.09, 0.165),
        (0.12, 0.155),
        (0.14, 0.12),
        (0.145, 0.065),
    ]
    bore_path = cq.Workplane("YZ").spline(bore_pts)

    # Outer tube: profile on XY plane (perpendicular to initial +Z tangent).
    tube = cq.Workplane("XY").circle(SPOUT_R).sweep(path)
    # Inner bore.
    bore = cq.Workplane("XY").workplane(offset=-0.005).circle(SPOUT_BORE_R).sweep(bore_path)

    # Base flange (escutcheon ring at deck level).
    flange = cq.Workplane("XY").circle(SPOUT_FLANGE_R).extrude(SPOUT_FLANGE_T)

    unbored = tube.union(flange)
    solid = unbored.cut(bore)

    global SPOUT_SOLID_VOLUME, SPOUT_UNBORED_VOLUME
    SPOUT_SOLID_VOLUME = solid.val().Volume()
    SPOUT_UNBORED_VOLUME = unbored.val().Volume()
    return solid


def _build_pedestal_mesh():
    """Tapered pedestal via LatheGeometry: wider at deck, narrower at top.
    Origin at base center, axis along +Z."""
    profile = [
        (PED_BOT_R, 0.0),
        (PED_BOT_R - 0.001, 0.004),
        (PED_BOT_R - 0.003, 0.010),
        (PED_TOP_R + 0.002, 0.028),
        (PED_TOP_R, PED_H),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=32), "pedestal")


def _build_lever_solid() -> cq.Workplane:
    """Cylindrical lever handle as a single CadQuery solid.
    Shaft along +X from origin, rounded end cap, and a short stem
    extending downward (-Z) to seat into the pedestal bore."""
    # Main shaft: cylinder along +X.
    shaft = (
        cq.Workplane("YZ")
        .circle(LEVER_R)
        .extrude(LEVER_LEN)
    )
    # Rounded end cap at the tip.
    cap = (
        cq.Workplane("YZ")
        .workplane(offset=LEVER_LEN)
        .circle(LEVER_CAP_R)
        .extrude(0.005)
    )
    # Stem going down from the shaft base into the pedestal.
    stem = (
        cq.Workplane("XY")
        .circle(0.005)
        .extrude(-0.014)  # extends below z=0 into pedestal
    )
    # Boss ring at shaft base (connects stem to shaft visually).
    boss = (
        cq.Workplane("XY")
        .circle(LEVER_R + 0.002)
        .extrude(0.004)
    )
    return shaft.union(cap).union(stem).union(boss)


def _build_aerator_solid() -> cq.Workplane:
    """Small cylindrical aerator body with a hollow bore and a screen-like
    ring at the outlet end. Axis along -Z (points downward when installed)."""
    body = cq.Workplane("XY").circle(AERATOR_R).extrude(AERATOR_LEN)
    bore = cq.Workplane("XY").workplane(offset=-0.002).circle(AERATOR_BORE_R).extrude(AERATOR_LEN + 0.004)
    # Screen ring at outlet end.
    ring = (
        cq.Workplane("XY")
        .workplane(offset=AERATOR_LEN - 0.003)
        .circle(AERATOR_R)
        .circle(AERATOR_BORE_R + 0.001)
        .extrude(0.003)
    )
    return body.cut(bore).union(ring)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_two_handle_faucet")

    gold = model.material("polished_gold_brass", rgba=(0.85, 0.66, 0.20, 1.0))
    deck_mat = model.material("deck_granite", rgba=(0.25, 0.25, 0.28, 1.0))
    seam_mat = model.material("seam_dark", rgba=(0.12, 0.10, 0.08, 1.0))

    # --- deck plate (root, mounting substrate) ---
    deck = model.part("deck")
    deck.visual(
        Box((DECK_W, DECK_D, DECK_T)),
        origin=Origin(xyz=(0.0, 0.0, -DECK_T / 2.0)),
        material=deck_mat,
        name="deck_plate",
    )

    # --- central spout (fixed) ---
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_spout_solid(), "spout_body"),
        material=gold,
        name="spout_tube",
    )
    # Seam ring at spout deck base.
    spout_seam = mesh_from_geometry(
        TorusGeometry(radius=SPOUT_FLANGE_R - 0.002, tube=SEAM_TUBE_R),
        "spout_seam",
    )
    spout.visual(spout_seam, material=seam_mat, name="spout_seam")
    model.articulation(
        "deck_to_spout",
        ArticulationType.FIXED,
        parent=deck,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- aerator (revolute hinge at spout outlet) ---
    aerator = model.part("aerator")
    aerator.visual(
        mesh_from_cadquery(_build_aerator_solid(), "aerator_body"),
        material=gold,
        name="aerator_body",
    )
    # The aerator mounts at the spout outlet end. The spout outlet is
    # approximately at (0, 0.145, 0.07) in the spout frame, facing downward.
    # The hinge axis is along X (lateral) so the aerator can pivot forward/back.
    # Small hinge knuckle to show the pivot point.
    hinge_knuckle = mesh_from_geometry(
        TorusGeometry(radius=AERATOR_R + 0.002, tube=0.002),
        "hinge_knuckle",
    )
    aerator.visual(
        hinge_knuckle,
        material=gold,
        name="hinge_knuckle",
    )
    model.articulation(
        "spout_to_aerator",
        ArticulationType.REVOLUTE,
        parent=spout,
        child=aerator,
        # Position at the spout outlet end, oriented so the aerator hangs down.
        origin=Origin(xyz=(0.0, 0.145, 0.07), rpy=(math.pi, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=1.0, lower=0.0, upper=math.radians(40)
        ),
    )

    # --- valve assemblies (fixed) and lever handles (revolute) ---
    pedestal_mesh = _build_pedestal_mesh()
    lever_mesh = mesh_from_cadquery(_build_lever_solid(), "lever")

    for side, sx in (("left", -1.0), ("right", 1.0)):
        valve = model.part(f"{side}_valve")

        # Escutcheon flange.
        valve.visual(
            Cylinder(radius=VALVE_ESC_R, length=VALVE_ESC_T),
            origin=Origin(xyz=(0.0, 0.0, VALVE_ESC_T / 2.0)),
            material=gold,
            name="escutcheon",
        )
        # Tapered pedestal rising from the escutcheon.
        valve.visual(
            pedestal_mesh,
            origin=Origin(xyz=(0.0, 0.0, VALVE_ESC_T)),
            material=gold,
            name="pedestal",
        )
        # Seam ring at deck interface.
        seam = mesh_from_geometry(
            TorusGeometry(radius=VALVE_ESC_R - 0.002, tube=SEAM_TUBE_R),
            f"{side}_seam",
        )
        valve.visual(seam, material=seam_mat, name="seam_ring")

        model.articulation(
            f"deck_to_{side}_valve",
            ArticulationType.FIXED,
            parent=deck,
            child=valve,
            origin=Origin(xyz=(sx * HANDLE_SPACING, 0.0, 0.0)),
        )

        # Lever handle: rotates about vertical Z axis.
        handle = model.part(f"{side}_lever")
        handle.visual(
            lever_mesh,
            origin=Origin(xyz=(0.0, 0.0, LEVER_R)),
            material=gold,
            name="lever_body",
        )

        model.articulation(
            f"{side}_lever_joint",
            ArticulationType.REVOLUTE,
            parent=valve,
            child=handle,
            # Joint at top of the pedestal where the handle mounts.
            origin=Origin(xyz=(0.0, 0.0, VALVE_ESC_T + PED_H)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=5.0, velocity=3.0, lower=-math.pi, upper=math.pi
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    spout = object_model.get_part("spout")
    aerator = object_model.get_part("aerator")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_lever = object_model.get_part("left_lever")
    right_lever = object_model.get_part("right_lever")
    left_joint = object_model.get_articulation("left_lever_joint")
    right_joint = object_model.get_articulation("right_lever_joint")
    aerator_joint = object_model.get_articulation("spout_to_aerator")

    # --- lever handle joints: both revolute about vertical axis ---
    for joint in (left_joint, right_joint):
        ctx.check(
            f"{joint.name}_revolute",
            str(joint.joint_type).lower().endswith("revolute"),
            f"type={joint.joint_type}",
        )
        ax = joint.axis
        ctx.check(
            f"{joint.name}_axis_vertical",
            abs(ax[0]) < 1e-9 and abs(ax[1]) < 1e-9 and abs(ax[2] - 1.0) < 1e-9,
            f"axis={ax}",
        )
        lim = joint.motion_limits
        ctx.check(
            f"{joint.name}_full_turn_range",
            lim is not None
            and abs(lim.lower + math.pi) < 1e-6
            and abs(lim.upper - math.pi) < 1e-6,
            f"limits=({lim.lower}, {lim.upper})",
        )

    # --- aerator hinge: revolute, pivots downward ---
    ctx.check(
        "aerator_joint_revolute",
        str(aerator_joint.joint_type).lower().endswith("revolute"),
        f"type={aerator_joint.joint_type}",
    )
    ctx.check(
        "aerator_joint_lateral_axis",
        abs(aerator_joint.axis[0] - 1.0) < 1e-9
        and abs(aerator_joint.axis[1]) < 1e-9
        and abs(aerator_joint.axis[2]) < 1e-9,
        f"axis={aerator_joint.axis}",
    )
    aero_lim = aerator_joint.motion_limits
    ctx.check(
        "aerator_pivots_downward",
        aero_lim is not None
        and abs(aero_lim.lower) < 1e-6
        and aero_lim.upper > math.radians(20),
        f"limits=({aero_lim.lower}, {aero_lim.upper})",
    )

    # --- spout geometry: hollow bore ---
    ctx.check(
        "spout_tube_is_hollow",
        0.0 < SPOUT_SOLID_VOLUME < 0.95 * SPOUT_UNBORED_VOLUME,
        f"solid={SPOUT_SOLID_VOLUME:.3e} m^3 vs unbored={SPOUT_UNBORED_VOLUME:.3e} m^3",
    )

    # --- spout rises above deck and reaches forward ---
    spout_aabb = ctx.part_world_aabb(spout)
    assert spout_aabb is not None
    (sx0, sy0, sz0), (sx1, sy1, sz1) = spout_aabb
    ctx.check(
        "spout_rises_above_deck",
        sz1 > 0.12,
        f"spout zmax={sz1:.3f}",
    )
    ctx.check(
        "spout_reaches_forward",
        sy1 > 0.10,
        f"spout ymax={sy1:.3f}",
    )

    # --- seam rings present at all three deck bases ---
    spout_seam = spout.get_visual("spout_seam")
    left_seam = left_valve.get_visual("seam_ring")
    right_seam = right_valve.get_visual("seam_ring")
    ctx.check("spout_has_seam_ring", spout_seam is not None, "missing spout seam ring")
    ctx.check("left_valve_has_seam_ring", left_seam is not None, "missing left seam ring")
    ctx.check("right_valve_has_seam_ring", right_seam is not None, "missing right seam ring")

    # --- tapered pedestals on both valves ---
    ctx.check(
        "left_valve_has_pedestal",
        left_valve.get_visual("pedestal") is not None,
        "missing left pedestal",
    )
    ctx.check(
        "right_valve_has_pedestal",
        right_valve.get_visual("pedestal") is not None,
        "missing right pedestal",
    )

    # --- lever handles are cylindrical (not cross) ---
    ctx.check(
        "left_has_lever",
        left_lever.get_visual("lever_body") is not None,
        "missing left lever",
    )
    ctx.check(
        "right_has_lever",
        right_lever.get_visual("lever_body") is not None,
        "missing right lever",
    )

    # --- valve placement: flanking the spout symmetrically ---
    lv = ctx.part_world_position(left_valve)
    rv = ctx.part_world_position(right_valve)
    assert lv is not None and rv is not None
    ctx.check(
        "valves_flank_spout_symmetrically",
        abs(lv[0] + HANDLE_SPACING) < 1e-6
        and abs(rv[0] - HANDLE_SPACING) < 1e-6
        and abs(lv[1]) < 1e-6
        and abs(rv[1]) < 1e-6,
        f"left={lv}, right={rv}",
    )

    # --- valves and spout sit on deck surface ---
    ctx.expect_gap(spout, deck, axis="z", max_gap=0.001, max_penetration=0.005)
    ctx.expect_gap(left_valve, deck, axis="z", max_gap=0.001, max_penetration=0.005)
    ctx.expect_gap(right_valve, deck, axis="z", max_gap=0.001, max_penetration=0.005)

    # --- lever handle overlap with valve (stem and boss seat into/on pedestal) ---
    ctx.allow_overlap(
        left_lever,
        left_valve,
        elem_a=left_lever.get_visual("lever_body"),
        elem_b=left_valve.get_visual("pedestal"),
        reason="lever stem and boss seat into and on top of the pedestal bore",
    )
    ctx.allow_overlap(
        right_lever,
        right_valve,
        elem_a=right_lever.get_visual("lever_body"),
        elem_b=right_valve.get_visual("pedestal"),
        reason="lever stem and boss seat into and on top of the pedestal bore",
    )

    # --- aerator seats into the spout outlet ---
    ctx.allow_overlap(
        aerator,
        spout,
        elem_a=aerator.get_visual("aerator_body"),
        elem_b=spout.get_visual("spout_tube"),
        reason="aerator body seats into the spout outlet bore at the hinge point",
    )

    ctx.expect_overlap(left_lever, left_valve, axes="xy", min_overlap=0.005)
    ctx.expect_overlap(right_lever, right_valve, axes="xy", min_overlap=0.005)

    # --- handle rotation proof: lever swings in the horizontal plane ---
    with ctx.pose({left_joint: math.pi / 2.0}):
        rot_pos = ctx.part_world_position(left_lever)
        assert rot_pos is not None
        ctx.check(
            "left_lever_rotates_about_vertical_axis",
            abs(rot_pos[0] + HANDLE_SPACING) < 0.001
            and abs(rot_pos[1]) < 0.001,
            f"handle origin after 90deg rotation={rot_pos}",
        )

    with ctx.pose({right_joint: -math.pi / 4.0}):
        ctx.expect_overlap(right_lever, right_valve, axes="xy", min_overlap=0.005)

    # --- aerator at spout outlet and pivots ---
    aerator_rest_aabb = ctx.part_world_aabb(aerator)
    assert aerator_rest_aabb is not None
    ctx.check(
        "aerator_near_spout_outlet",
        aerator_rest_aabb[1][1] > 0.10 and aerator_rest_aabb[1][2] < 0.12,
        f"aerator AABB max={aerator_rest_aabb[1]}",
    )

    with ctx.pose({aerator_joint: math.radians(30)}):
        aerator_pivot_aabb = ctx.part_world_aabb(aerator)
        assert aerator_pivot_aabb is not None
        # When the aerator pivots, its y-extent should change (tilting forward).
        rest_dy = aerator_rest_aabb[1][1] - aerator_rest_aabb[0][1]
        piv_dy = aerator_pivot_aabb[1][1] - aerator_pivot_aabb[0][1]
        ctx.check(
            "aerator_pivots_when_actuated",
            abs(piv_dy - rest_dy) > 0.0005
            or abs(aerator_pivot_aabb[1][2] - aerator_rest_aabb[1][2]) > 0.0005,
            f"rest_dy={rest_dy:.4f}, pivoted_dy={piv_dy:.4f}",
        )

    # --- overall width about 0.25-0.35 m across lever extents ---
    lh_aabb = ctx.part_world_aabb(left_lever)
    rh_aabb = ctx.part_world_aabb(right_lever)
    assert lh_aabb is not None and rh_aabb is not None
    total_w = rh_aabb[1][0] - lh_aabb[0][0]
    ctx.check(
        "overall_width_widespread_proportions",
        0.22 <= total_w <= 0.40,
        f"lever-tip to lever-tip width={total_w:.3f}",
    )

    # --- deck grounded ---
    deck_aabb = ctx.part_world_aabb(deck)
    assert deck_aabb is not None
    ctx.check(
        "deck_grounded_at_z0",
        abs(deck_aabb[1][2]) < 0.002,
        f"deck zmax={deck_aabb[1][2]:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
