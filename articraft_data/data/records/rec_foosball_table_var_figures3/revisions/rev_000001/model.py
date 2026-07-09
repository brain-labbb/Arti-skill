from __future__ import annotations

# Foosball (table football) table — uniform-three-figure variant:
# black cabinet with rounded-corner side panels and a lighter gray top rim, four
# white splayed rectangular legs, green pitch with raised white markings, goal
# slots in both end walls, bead score counters on rails above each end, and
# EIGHT steel player rods crossing the cabinet width. Each rod passes through
# clearance holes bored in the side panels, carries a colored handle grip on its
# team's side and molded red/blue player figures, and has TWO DOF: a prismatic
# slide along its axis (massless carrier link) plus a continuous spin for
# kicking. Every rod carries exactly FIGURES_PER_ROD (= 3) evenly spaced molded
# figures, demonstrating a constant figures-per-rod count. Team red (handles on
# -Y) defends -X goal; team blue (handles on +Y) defends +X goal.
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------- dimensions
CAB_L = 1.20  # X, table length
CAB_W = 0.70  # Y, table width
WALL_T = 0.025
BASE_BOT = 0.60  # cabinet underside height
BASE_TOP = 0.616  # black base panel top
PITCH_TOP = 0.622  # green pitch playing surface
WALL_TOP = 0.90
RIM_TOP = 0.914

SIDE_Y = CAB_W / 2.0 - WALL_T / 2.0  # 0.3375 side panel center
INNER_Y = CAB_W / 2.0 - WALL_T  # 0.325 interior half width
END_X = 0.57  # end wall center
END_IN_X = END_X - WALL_T / 2.0  # 0.5575 inner face of end wall

GOAL_HALF_W = 0.10
GOAL_TOP = PITCH_TOP + 0.103  # lintel underside

ROD_R = 0.008
ROD_Z = PITCH_TOP + 0.082  # 0.704 rod axis height
# bearing bore in the side panels: slight interference fit so the rod reads as
# captured/supported by the wall bushings (overlap is explicitly allowed)
HOLE_R = 0.0072

MARK_T = 0.0012  # marking strip thickness (raised decal)
MARK_ZC = PITCH_TOP + MARK_T / 2.0

LEG_TILT = 0.13  # splay angle about Y (outward along table length)
LEG_LEN = 0.6116
LEG_SX, LEG_SY = 0.06, 0.095

# figure layout (local to rod frame, figure hangs along -Z at q=0)
FIG_TORSO = (0.024, 0.032, 0.052)
FIG_TORSO_Z = -0.010
FIG_HEAD_R = 0.0125
FIG_HEAD_Z = 0.0265
FIG_LEG = (0.017, 0.020, 0.028)
FIG_LEG_Z = -0.048
FIG_FOOT = (0.018, 0.022, 0.012)
FIG_FOOT_Z = -0.0625
FIG_FOOT_X = 0.003
FIG_FOOT_TILT = 0.25

# Uniform figures-per-rod parameter: every rod carries exactly this many figures.
FIGURES_PER_ROD = 3

# rod order from -X end: (index, x, team, n_figures, figure spacing, slide travel)
# teams: red defends the -X goal, blue defends the +X goal
# All rods now carry FIGURES_PER_ROD (= 3) evenly spaced figures with uniform
# spacing and travel, demonstrating a constant figures-per-rod count.
ROD_CONFIGS = (
    (1, -0.525, "red", FIGURES_PER_ROD, 0.185, 0.110),  # goalkeeper red
    (2, -0.375, "red", FIGURES_PER_ROD, 0.185, 0.110),  # defense red
    (3, -0.225, "blue", FIGURES_PER_ROD, 0.185, 0.110),  # attack blue
    (4, -0.075, "red", FIGURES_PER_ROD, 0.185, 0.110),  # midfield red
    (5, 0.075, "blue", FIGURES_PER_ROD, 0.185, 0.110),  # midfield blue
    (6, 0.225, "red", FIGURES_PER_ROD, 0.185, 0.110),  # attack red
    (7, 0.375, "blue", FIGURES_PER_ROD, 0.185, 0.110),  # defense blue
    (8, 0.525, "blue", FIGURES_PER_ROD, 0.185, 0.110),  # goalkeeper blue
)

PROT_MARGIN = 0.06  # rod still protrudes this far beyond the wall at extreme slide


def rod_protrusion(travel: float) -> float:
    return travel + PROT_MARGIN


def figure_swing_radius() -> float:
    """Max radial distance of any figure point from the rod axis (XZ plane)."""
    c, s = math.cos(FIG_FOOT_TILT), math.sin(FIG_FOOT_TILT)
    hx, hz = FIG_FOOT[0] / 2.0, FIG_FOOT[2] / 2.0
    r_max = 0.0
    for sx in (-1.0, 1.0):
        for sz in (-1.0, 1.0):
            x = FIG_FOOT_X + sx * hx * c + sz * hz * s
            z = FIG_FOOT_Z - sx * hx * s + sz * hz * c
            r_max = max(r_max, math.hypot(x, z))
    r_max = max(r_max, abs(FIG_HEAD_Z) + FIG_HEAD_R)
    r_max = max(r_max, math.hypot(abs(FIG_TORSO_Z) + FIG_TORSO[2] / 2.0, FIG_TORSO[0] / 2.0))
    r_max = max(r_max, math.hypot(abs(FIG_LEG_Z) + FIG_LEG[2] / 2.0, FIG_LEG[0] / 2.0))
    return r_max


def _side_panel_mesh(name: str):
    """Black side panel with rounded corners and 8 rod clearance holes.

    Authored centered in X/Z on the panel mid-height; the solid spans local
    y in [-WALL_T, 0] (Workplane("XZ") extrudes toward -Y). The panel reaches
    down to the base-panel top so it embeds into the floor layer (connected).
    """
    height = WALL_TOP - BASE_TOP
    zc = (WALL_TOP + BASE_TOP) / 2.0
    panel = cq.Workplane("XZ").rect(CAB_L, height).extrude(WALL_T).edges("|Y").fillet(0.045)
    pts = [(cfg[1], ROD_Z - zc) for cfg in ROD_CONFIGS]
    holes = cq.Workplane("XZ").pushPoints(pts).circle(HOLE_R).extrude(WALL_T * 3.0, both=True)
    return mesh_from_cadquery(panel.cut(holes), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="foosball_table")

    black = Material("cabinet_black", rgba=(0.05, 0.05, 0.06, 1.0))
    gray = Material("rim_gray", rgba=(0.62, 0.63, 0.65, 1.0))
    white = Material("leg_white", rgba=(0.92, 0.92, 0.93, 1.0))
    green = Material("pitch_green", rgba=(0.10, 0.45, 0.16, 1.0))
    mark = Material("marking_white", rgba=(0.95, 0.95, 0.95, 1.0))
    steel = Material("rod_steel", rgba=(0.68, 0.70, 0.72, 1.0))
    red = Material("team_red", rgba=(0.78, 0.10, 0.08, 1.0))
    blue = Material("team_blue", rgba=(0.10, 0.22, 0.68, 1.0))
    team_mat = {"red": red, "blue": blue}

    wall_zc = (WALL_TOP + PITCH_TOP) / 2.0
    wall_h = WALL_TOP - PITCH_TOP

    cab = model.part("cabinet")

    # ---- base panel (black) + green pitch floor ----------------------------
    cab.visual(
        Box((CAB_L, CAB_W, BASE_TOP - BASE_BOT)),
        origin=Origin(xyz=(0.0, 0.0, (BASE_TOP + BASE_BOT) / 2.0)),
        material=black,
        name="cabinet_base",
    )
    cab.visual(
        Box((2.0 * END_X + WALL_T, 0.66, PITCH_TOP - BASE_TOP)),
        origin=Origin(xyz=(0.0, 0.0, (PITCH_TOP + BASE_TOP) / 2.0)),
        material=green,
        name="pitch",
    )

    # ---- side panels with rounded corners and rod holes --------------------
    # panel solid spans local y in [-WALL_T, 0]; outer faces land at +/-0.35
    panel_zc = (WALL_TOP + BASE_TOP) / 2.0
    cab.visual(
        _side_panel_mesh("side_panel_mesh_0"),
        origin=Origin(xyz=(0.0, -(CAB_W / 2.0 - WALL_T), panel_zc)),
        material=black,
        name="side_panel_0",
    )
    cab.visual(
        _side_panel_mesh("side_panel_mesh_1"),
        origin=Origin(xyz=(0.0, CAB_W / 2.0, panel_zc)),
        material=black,
        name="side_panel_1",
    )

    # ---- end walls with goal slots ------------------------------------------
    post_w = INNER_Y - GOAL_HALF_W  # 0.225 wall segment beside each goal mouth
    lintel_h = WALL_TOP - GOAL_TOP
    for e, end in ((-1.0, "0"), (1.0, "1")):
        for s, seg in ((-1.0, "a"), (1.0, "b")):
            cab.visual(
                Box((WALL_T, post_w, wall_h)),
                origin=Origin(xyz=(e * END_X, s * (GOAL_HALF_W + post_w / 2.0), wall_zc)),
                material=black,
                name=f"end_wall_{end}_post_{seg}",
            )
        cab.visual(
            Box((WALL_T, 2.0 * GOAL_HALF_W, lintel_h)),
            origin=Origin(xyz=(e * END_X, 0.0, (WALL_TOP + GOAL_TOP) / 2.0)),
            material=black,
            name=f"end_wall_{end}_lintel",
        )

    # ---- lighter gray top rim (sunk 2 mm into the wall tops for a solid seat) ----
    rim_bot = WALL_TOP - 0.002
    rim_t = RIM_TOP - rim_bot
    for s, idx in ((-1.0, "0"), (1.0, "1")):
        cab.visual(
            Box((1.10, 0.05, rim_t)),
            origin=Origin(xyz=(0.0, s * SIDE_Y, rim_bot + rim_t / 2.0)),
            material=gray,
            name=f"side_rim_{idx}",
        )
    for e, idx in ((-1.0, "0"), (1.0, "1")):
        cab.visual(
            Box((0.05, 2.0 * INNER_Y, rim_t)),
            origin=Origin(xyz=(e * END_X, 0.0, rim_bot + rim_t / 2.0)),
            material=gray,
            name=f"end_rim_{idx}",
        )

    # ---- pitch markings (raised ~1.2 mm decal strips) ---------------------------
    cab.visual(
        Box((0.012, 2.0 * INNER_Y, MARK_T)),
        origin=Origin(xyz=(0.0, 0.0, MARK_ZC)),
        material=mark,
        name="halfway_line",
    )
    # ring built with CadQuery so the hole is genuinely open (green shows inside)
    ring = cq.Workplane("XY").circle(0.105).circle(0.092).extrude(MARK_T)
    cab.visual(
        mesh_from_cadquery(ring, "center_circle_mesh"),
        origin=Origin(xyz=(0.0, 0.0, PITCH_TOP)),
        material=mark,
        name="center_circle",
    )
    cab.visual(
        Cylinder(radius=0.018, length=MARK_T),
        origin=Origin(xyz=(0.0, 0.0, MARK_ZC)),
        material=mark,
        name="center_spot",
    )
    box_depth = 0.16
    box_half_w = 0.18
    for e, end in ((-1.0, "0"), (1.0, "1")):
        cab.visual(
            Box((0.012, 2.0 * box_half_w, MARK_T)),
            origin=Origin(xyz=(e * (END_IN_X - box_depth), 0.0, MARK_ZC)),
            material=mark,
            name=f"goal_box_{end}_front",
        )
        for s, seg in ((-1.0, "a"), (1.0, "b")):
            cab.visual(
                Box((box_depth, 0.012, MARK_T)),
                origin=Origin(xyz=(e * (END_IN_X - box_depth / 2.0), s * box_half_w, MARK_ZC)),
                material=mark,
                name=f"goal_box_{end}_side_{seg}",
            )

    # ---- four white splayed legs ------------------------------------------------
    half = LEG_LEN / 2.0
    for i, (sx, sy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        cab.visual(
            Box((LEG_SX, LEG_SY, LEG_LEN)),
            origin=Origin(
                xyz=(
                    sx * (0.50 + half * math.sin(LEG_TILT)),
                    sy * 0.27,
                    0.610 - half * math.cos(LEG_TILT),
                ),
                rpy=(0.0, -sx * LEG_TILT, 0.0),
            ),
            material=white,
            name=f"leg_{i}",
        )

    # ---- bead score counters above each end ----------------------------------------
    for e, end, team in ((-1.0, "0", red), (1.0, "1", blue)):
        for s, seg in ((-1.0, "a"), (1.0, "b")):
            cab.visual(
                Box((0.024, 0.024, 0.05)),
                origin=Origin(xyz=(e * END_X, s * 0.20, RIM_TOP + 0.025)),
                material=black,
                name=f"score_post_{end}_{seg}",
            )
        cab.visual(
            Cylinder(radius=0.0055, length=0.44),
            origin=Origin(xyz=(e * END_X, 0.0, 0.952), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"score_rail_{end}",
        )
        for b in range(10):
            cab.visual(
                Cylinder(radius=0.0125, length=0.019),
                origin=Origin(
                    xyz=(e * END_X, -0.185 + b * 0.0205, 0.952),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=team,
                name=f"score_bead_{end}_{b}",
            )

    # ---- eight player rods: prismatic carrier + continuous spin ----------------------
    for idx, x, team, n_fig, spacing, travel in ROD_CONFIGS:
        prot = rod_protrusion(travel)
        rod_len = CAB_W + 2.0 * prot
        side = -1.0 if team == "red" else 1.0  # handle side of the table
        end_y = CAB_W / 2.0 + prot  # rod tip distance from center

        carrier = model.part(f"rod_{idx}_carrier")
        carrier.inertial = Inertial.from_geometry(Box((0.01, 0.01, 0.01)), mass=1e-4)

        rod = model.part(f"rod_{idx}")
        rod.visual(
            Cylinder(radius=ROD_R, length=rod_len),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="shaft",
        )
        # colored handle grip on the team side, slipped over the rod end
        rod.visual(
            Cylinder(radius=0.0175, length=0.115),
            origin=Origin(
                xyz=(0.0, side * (end_y + 0.0375), 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=team_mat[team],
            name="handle",
        )
        # safety end cap on the opposite end
        rod.visual(
            Cylinder(radius=0.0115, length=0.018),
            origin=Origin(
                xyz=(0.0, -side * (end_y + 0.004), 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=black,
            name="end_cap",
        )
        # molded player figures, hanging along -Z at q=0
        for j in range(n_fig):
            yj = (j - (n_fig - 1) / 2.0) * spacing
            p = f"player_{j + 1}"
            rod.visual(
                Box(FIG_TORSO),
                origin=Origin(xyz=(0.0, yj, FIG_TORSO_Z)),
                material=team_mat[team],
                name=f"{p}_torso",
            )
            rod.visual(
                Sphere(FIG_HEAD_R),
                origin=Origin(xyz=(0.0, yj, FIG_HEAD_Z)),
                material=team_mat[team],
                name=f"{p}_head",
            )
            rod.visual(
                Box(FIG_LEG),
                origin=Origin(xyz=(0.0, yj, FIG_LEG_Z)),
                material=team_mat[team],
                name=f"{p}_legs",
            )
            rod.visual(
                Box(FIG_FOOT),
                origin=Origin(
                    xyz=(FIG_FOOT_X, yj, FIG_FOOT_Z),
                    rpy=(0.0, FIG_FOOT_TILT, 0.0),
                ),
                material=team_mat[team],
                name=f"{p}_foot",
            )

        model.articulation(
            f"rod_{idx}_slide",
            ArticulationType.PRISMATIC,
            parent=cab,
            child=carrier,
            origin=Origin(xyz=(x, 0.0, ROD_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=30.0, velocity=2.0, lower=-travel, upper=travel),
        )
        model.articulation(
            f"rod_{idx}_spin",
            ArticulationType.CONTINUOUS,
            parent=carrier,
            child=rod,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=10.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cab = object_model.get_part("cabinet")

    # ---- figure geometry clears the pitch and the neighbor rods ---------------
    swing_r = figure_swing_radius()
    ctx.check(
        "figure swing radius clears the pitch",
        swing_r < (ROD_Z - PITCH_TOP) - 0.002,
        details=f"swing radius {swing_r:.4f} m vs rod height above pitch {ROD_Z - PITCH_TOP:.4f} m",
    )
    ctx.check(
        "adjacent spinning rods cannot clash",
        2.0 * swing_r < 0.15,
        details=f"2*swing={2.0 * swing_r:.4f} vs rod spacing 0.15",
    )

    # ---- 8 rods, each with prismatic slide + continuous spin ------------------
    for idx, x, team, n_fig, spacing, travel in ROD_CONFIGS:
        rod = object_model.get_part(f"rod_{idx}")
        slide = object_model.get_articulation(f"rod_{idx}_slide")
        spin = object_model.get_articulation(f"rod_{idx}_spin")

        ctx.check(
            f"rod_{idx}_slide is prismatic with +/-{travel:.3f} m travel",
            str(slide.articulation_type).lower().endswith("prismatic")
            and slide.motion_limits is not None
            and abs(slide.motion_limits.lower + travel) < 1e-9
            and abs(slide.motion_limits.upper - travel) < 1e-9,
            details=f"type={slide.articulation_type}, limits={slide.motion_limits}",
        )
        ctx.check(
            f"rod_{idx}_spin is continuous",
            str(spin.articulation_type).lower().endswith("continuous"),
            details=f"type={spin.articulation_type}",
        )

        torsos = [v for v in rod.visuals if v.name and v.name.endswith("_torso")]
        ctx.check(
            f"rod_{idx} carries {n_fig} player figures",
            len(torsos) == n_fig,
            details=f"found {len(torsos)} torsos",
        )
        
        # Uniform figures-per-rod: every rod carries exactly FIGURES_PER_ROD figures
        ctx.check(
            f"rod_{idx} carries uniform FIGURES_PER_ROD (= {FIGURES_PER_ROD})",
            len(torsos) == FIGURES_PER_ROD,
            details=f"found {len(torsos)} torsos, expected {FIGURES_PER_ROD}",
        )
        
        # Even lateral spacing: figures are evenly distributed along the rod
        if n_fig >= 2:
            # Collect Y positions of all figure torsos
            y_positions = []
            for t in torsos:
                aabb = ctx.part_element_world_aabb(rod, elem=t.name)
                if aabb is not None:
                    y_center = (aabb[0][1] + aabb[1][1]) / 2.0
                    y_positions.append(y_center)
            
            if len(y_positions) >= 2:
                y_positions.sort()
                gaps = [y_positions[i+1] - y_positions[i] for i in range(len(y_positions)-1)]
                avg_gap = sum(gaps) / len(gaps)
                max_deviation = max(abs(g - avg_gap) for g in gaps)
                
                ctx.check(
                    f"rod_{idx} figures are evenly spaced (max deviation {max_deviation:.4f} m)",
                    max_deviation < 0.005,  # tolerance for even spacing
                    details=f"gaps={gaps}, avg={avg_gap:.4f}, max_dev={max_deviation:.4f}",
                )

        # rod shaft runs through bearing bores in both side panels (slight
        # interference fit stands in for the wall bushings that carry the rod)
        for panel in ("side_panel_0", "side_panel_1"):
            ctx.allow_overlap(
                rod,
                cab,
                elem_a="shaft",
                elem_b=panel,
                reason=(
                    "The steel rod intentionally runs through a bearing bore in the "
                    "side panel; the slight interference represents the wall bushing "
                    "that supports the rod while it slides and spins."
                ),
            )
            ctx.expect_contact(
                rod,
                cab,
                elem_a="shaft",
                elem_b=panel,
                name=f"rod_{idx} shaft is seated in {panel} bearing bore",
            )

        # figure feet hang above the pitch decals at rest pose
        ctx.expect_gap(
            rod,
            cab,
            axis="z",
            positive_elem="player_1_foot",
            negative_elem="pitch",
            min_gap=0.003,
            name=f"rod_{idx} figure feet hang above the pitch",
        )

        # handle sits on the team side, fully outside the cabinet wall
        h_aabb = ctx.part_element_world_aabb(rod, elem="handle")
        side = -1.0 if team == "red" else 1.0
        ok_handle = h_aabb is not None and (
            (side < 0 and h_aabb[1][1] < -CAB_W / 2.0) or (side > 0 and h_aabb[0][1] > CAB_W / 2.0)
        )
        ctx.check(
            f"rod_{idx} handle is outside the {'-Y' if side < 0 else '+Y'} wall ({team} team)",
            ok_handle,
            details=f"handle aabb={h_aabb}",
        )

        # slide limits keep every figure inside the interior walls
        for q in (-travel, travel):
            with ctx.pose({slide: q}):
                for elem in ("player_1_torso", f"player_{n_fig}_torso"):
                    aabb = ctx.part_element_world_aabb(rod, elem=elem)
                    ok = (
                        aabb is not None
                        and aabb[0][1] >= -INNER_Y + 0.002
                        and aabb[1][1] <= INNER_Y - 0.002
                    )
                    ctx.check(
                        f"rod_{idx} {elem} stays inside walls at slide q={q:+.3f}",
                        ok,
                        details=f"aabb={aabb}, interior |y|<={INNER_Y}",
                    )

    # ---- uniform figures-per-rod invariant ----------------------------------------
    # Verify that every rod carries exactly FIGURES_PER_ROD figures, demonstrating
    # the constant figures-per-rod count instead of the parent mix of 1, 2, 3, 5.
    figure_counts = []
    for idx in range(1, 9):
        rod = object_model.get_part(f"rod_{idx}")
        torsos = [v for v in rod.visuals if v.name and v.name.endswith("_torso")]
        figure_counts.append(len(torsos))
    
    ctx.check(
        f"all eight rods carry exactly {FIGURES_PER_ROD} figures (uniform count)",
        all(count == FIGURES_PER_ROD for count in figure_counts),
        details=f"figure counts per rod: {figure_counts}, expected all {FIGURES_PER_ROD}",
    )
    
    ctx.check(
        "figure counts are constant (not the parent mix of 1, 2, 3, 5)",
        len(set(figure_counts)) == 1,
        details=f"unique counts: {set(figure_counts)}, expected {{3}}",
    )

    # retained insertion: the longest-travel rod still spans both wall bores at
    # both slide extremes
    rod2 = object_model.get_part("rod_2")
    slide2 = object_model.get_articulation("rod_2_slide")
    for q in (-0.175, 0.175):
        with ctx.pose({slide2: q}):
            for panel in ("side_panel_0", "side_panel_1"):
                ctx.expect_overlap(
                    rod2,
                    cab,
                    axes="y",
                    elem_a="shaft",
                    elem_b=panel,
                    min_overlap=WALL_T * 0.8,
                    name=f"rod_2 shaft stays inserted through {panel} at q={q:+.3f}",
                )

    # decisive spin poses on one rod per row type: feet never touch the pitch
    for idx in (1, 2, 4, 6):
        rod = object_model.get_part(f"rod_{idx}")
        spin = object_model.get_articulation(f"rod_{idx}_spin")
        for ang in (math.pi / 4.0, math.pi / 2.0, math.pi, -math.pi / 4.0):
            with ctx.pose({spin: ang}):
                aabb = ctx.part_element_world_aabb(rod, elem="player_1_foot")
                ctx.check(
                    f"rod_{idx} foot clears pitch at spin {ang:+.2f} rad",
                    aabb is not None and aabb[0][2] > PITCH_TOP + 0.001,
                    details=f"foot aabb={aabb}, pitch top={PITCH_TOP}",
                )

    # ---- goal openings in both end walls ----------------------------------------
    for end in ("0", "1"):
        a = ctx.part_element_world_aabb(cab, elem=f"end_wall_{end}_post_a")
        b = ctx.part_element_world_aabb(cab, elem=f"end_wall_{end}_post_b")
        lintel = ctx.part_element_world_aabb(cab, elem=f"end_wall_{end}_lintel")
        ok = (
            a is not None
            and b is not None
            and lintel is not None
            and (b[0][1] - a[1][1]) > 0.18  # clear width between goal-side segments
            and (lintel[0][2] - PITCH_TOP) > 0.09  # clear height under the lintel
        )
        ctx.check(
            f"end wall {end} has an open goal slot",
            ok,
            details=f"post_a={a}, post_b={b}, lintel={lintel}",
        )

    # ---- legs reach the ground ------------------------------------------------------
    for i in range(4):
        aabb = ctx.part_element_world_aabb(cab, elem=f"leg_{i}")
        ctx.check(
            f"leg_{i} touches the ground",
            aabb is not None and -0.003 <= aabb[0][2] <= 0.004,
            details=f"leg aabb={aabb}",
        )

    # ---- score counters above each end ------------------------------------------------
    for end in ("0", "1"):
        rail = ctx.part_element_world_aabb(cab, elem=f"score_rail_{end}")
        bead = ctx.part_element_world_aabb(cab, elem=f"score_bead_{end}_0")
        ctx.check(
            f"score counter rail {end} sits above the end wall",
            rail is not None and bead is not None and rail[0][2] > RIM_TOP,
            details=f"rail={rail}, bead={bead}",
        )

    return ctx.report()


object_model = build_object_model()
