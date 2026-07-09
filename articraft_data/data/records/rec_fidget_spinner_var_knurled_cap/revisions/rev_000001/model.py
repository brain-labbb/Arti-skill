from __future__ import annotations

# Red three-arm fidget spinner with knurled metal spin caps.
# Frame: spinner lies flat in the XY plane, thickness along +Z, centered on the
# origin. The central vertical axis is +Z.
# Construction:
#   - center_cap (ROOT / held part): the central pinch assembly you hold between
#     two fingers. A silver hub barrel through the middle, a TALL knurled metal
#     spin cap on the top face (+Z, standing proud above the body), and a
#     matching LOW knurled metal cap on the bottom face (-Z). Knurl ridges are
#     fine vertical ribs emitted as inline visuals around each cap rim via a
#     shared CadQuery ridge helper in a for-i-in-range loop. This is the part
#     that stays still while the body spins.
#   - spinner_body: flat glossy red tri-lobe plate (3 round lobes at 120deg
#     fused to a central hub), with a circular bearing pocket bored through each
#     lobe. CONTINUOUS spin about the central +Z axis relative to the cap. Its
#     3-lobe silhouette is off-axis, so the spin is detectable by AABB tests.
#   - bearing_0/1/2: each lobe holds a press-fit skateboard-style bearing: a
#     black rubber ring (outer race), a silver inner race ring, and the open
#     center hole. Each spins CONTINUOUSLY about its own lobe axis (+Z). A tiny
#     off-axis silver marker tab on the inner race makes each ring's spin
#     detectable by AABB spin tests.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key dimensions (meters) ----
BODY_THICK = 0.011            # tri-lobe plate thickness
HALF_T = BODY_THICK / 2.0

LOBE_R = 0.0150               # radius of each round lobe
LOBE_DIST = 0.0250           # center-to-lobe-center distance
HUB_R = 0.0098               # central hub radius of the body

BEARING_POCKET_R = 0.0116    # bored pocket radius in each lobe
BEARING_OUTER_R = 0.0119     # black rubber ring outer radius (press fit, slight interference)
BEARING_INNER_R = 0.0072     # boundary between black ring and silver race
RACE_INNER_R = 0.0042        # silver inner race inner radius (the hole)
BEARING_THICK = 0.010        # bearing total thickness

# central bearing / knurled spin caps
CAP_HUB_R = 0.0072           # silver hub barrel radius at the very center
TOP_CAP_R = 0.0110           # top spin cap radius (22mm diameter)
TOP_CAP_T = 0.0140           # top spin cap height (tall, stands proud)
BOTTOM_CAP_R = 0.0110        # bottom spin cap radius (matching)
BOTTOM_CAP_T = 0.0070        # bottom spin cap height (low profile)
KNURL_COUNT = 24             # number of vertical knurl ridges per cap
KNURL_WIDTH = 0.0010         # tangential width of each ridge
KNURL_DEPTH = 0.0008         # radial protrusion of each ridge
# Hub barrel spans from bottom of bottom cap to top of top cap.
CAP_HUB_LEN = BODY_THICK + TOP_CAP_T + BOTTOM_CAP_T

LOBE_ANGLES = (math.pi / 2.0, math.pi / 2.0 + 2.0 * math.pi / 3.0,
               math.pi / 2.0 + 4.0 * math.pi / 3.0)


def _tri_lobe_body() -> cq.Workplane:
    """Flat red tri-lobe plate: central hub disc fused with 3 lobe discs and
    the connecting webs, then a bearing pocket bored through each lobe and a
    hub bore through the center for the cap."""
    # Central hub disc.
    body = cq.Workplane("XY").circle(HUB_R).extrude(BODY_THICK)

    for ang in LOBE_ANGLES:
        cx = LOBE_DIST * math.cos(ang)
        cy = LOBE_DIST * math.sin(ang)
        # Lobe disc.
        lobe = cq.Workplane("XY").center(cx, cy).circle(LOBE_R).extrude(BODY_THICK)
        body = body.union(lobe)
        # Connecting web (tapered bridge) between hub and lobe.
        web_len = LOBE_DIST
        web = (
            cq.Workplane("XY")
            .center(cx / 2.0, cy / 2.0)
            .transformed(rotate=(0.0, 0.0, math.degrees(ang)))
            .rect(web_len, 2.0 * (HUB_R * 0.92))
            .extrude(BODY_THICK)
        )
        body = body.union(web)

    # Bore a bearing pocket through each lobe.
    for ang in LOBE_ANGLES:
        cx = LOBE_DIST * math.cos(ang)
        cy = LOBE_DIST * math.sin(ang)
        pocket = (
            cq.Workplane("XY")
            .center(cx, cy)
            .circle(BEARING_POCKET_R)
            .extrude(BODY_THICK + 0.004)
            .translate((0.0, 0.0, -0.002))
        )
        body = body.cut(pocket)

    # Bore the central hub hole for the cap barrel.
    center_bore = (
        cq.Workplane("XY")
        .circle(CAP_HUB_R + 0.0004)
        .extrude(BODY_THICK + 0.004)
        .translate((0.0, 0.0, -0.002))
    )
    body = body.cut(center_bore)

    # Round the flat top/bottom edges of the lobes for a glossy molded look.
    body = body.edges("|Z").fillet(0.0012)
    return body


def _bearing_ring_mesh(name: str):
    """Black rubber outer ring (a thick torus-like ring) for one bearing."""
    outer = cq.Workplane("XY").circle(BEARING_OUTER_R).extrude(BEARING_THICK)
    hole = cq.Workplane("XY").circle(BEARING_INNER_R).extrude(BEARING_THICK + 0.004).translate(
        (0.0, 0.0, -0.002)
    )
    ring = outer.cut(hole)
    return mesh_from_cadquery(ring, name)


def _bearing_race_mesh(name: str):
    """Silver inner race ring + a tiny off-axis marker tab so spin is visible."""
    race = cq.Workplane("XY").circle(BEARING_INNER_R).extrude(BEARING_THICK)
    hole = cq.Workplane("XY").circle(RACE_INNER_R).extrude(BEARING_THICK + 0.004).translate(
        (0.0, 0.0, -0.002)
    )
    race = race.cut(hole)
    # Tiny off-axis notch/marker on the race rim so an AABB spin test can detect
    # rotation of the otherwise rotationally-symmetric ring.
    marker = (
        cq.Workplane("XY")
        .center(BEARING_INNER_R - 0.0008, 0.0)
        .box(0.0026, 0.0014, BEARING_THICK + 0.0010, centered=(True, True, True))
        .translate((0.0, 0.0, BEARING_THICK / 2.0))
    )
    race = race.union(marker)
    return mesh_from_cadquery(race, name)


def _spin_cap_body(name: str, radius: float, height: float):
    """Cylindrical spin cap body built with CadQuery.
    A solid cylinder with a small chamfer on the outer rim edges for a
    machined-finish look."""
    cap = cq.Workplane("XY").circle(radius).extrude(height)
    # Chamfer the top and bottom outer rim circles for a finished machined look.
    cap = cap.edges(">Z or <Z").chamfer(0.0005)
    return mesh_from_cadquery(cap, name)


def _knurl_ridge(name: str, width: float, depth: float, height: float):
    """A single vertical knurling ridge: a thin radial rib built with CadQuery.
    The ridge is a tall thin box with its long axis along Z, narrow in X (radial
    depth) and Y (tangential width). Bottom face sits at z=0."""
    ridge = (
        cq.Workplane("XY")
        .box(depth, width, height, centered=(True, True, False))
    )
    return mesh_from_cadquery(ridge, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fidget_spinner")

    red = model.material("glossy_red", rgba=(0.82, 0.07, 0.09, 1.0))
    black = model.material("rubber_black", rgba=(0.06, 0.06, 0.07, 1.0))
    silver = model.material("silver_steel", rgba=(0.78, 0.80, 0.83, 1.0))
    gunmetal = model.material("gunmetal_cap", rgba=(0.38, 0.40, 0.43, 1.0))
    knurl_steel = model.material("knurl_steel", rgba=(0.50, 0.52, 0.55, 1.0))

    # ---- center_cap (ROOT / held part) ----
    # Silver hub barrel + tall knurled metal top cap + low knurled metal bottom cap.
    cap = model.part("center_cap")

    # The spinner body is a child with articulation origin at HALF_T, so in
    # world coords the body occupies z = [HALF_T, HALF_T + BODY_THICK].
    # Caps must seat just outside that range.
    body_top_z = HALF_T + BODY_THICK   # 0.0165
    body_bot_z = HALF_T                 # 0.0055

    # Silver hub barrel spanning the full cap-to-cap height.
    hub_barrel = CylinderGeometry(CAP_HUB_R, CAP_HUB_LEN, radial_segments=48)
    hub_center_z = (body_top_z + TOP_CAP_T + (body_bot_z - BOTTOM_CAP_T)) / 2.0
    cap.visual(
        mesh_from_geometry(hub_barrel, "cap_hub"),
        origin=Origin(xyz=(0.0, 0.0, hub_center_z)),
        material=silver,
        name="cap_hub",
    )

    # Tall knurled metal spin cap on the top face (standing proud above body).
    cap.visual(
        _spin_cap_body("top_cap_body", TOP_CAP_R, TOP_CAP_T),
        origin=Origin(xyz=(0.0, 0.0, body_top_z)),
        material=gunmetal,
        name="top_cap_body",
    )

    # Low knurled metal cap on the bottom face.
    cap.visual(
        _spin_cap_body("bottom_cap_body", BOTTOM_CAP_R, BOTTOM_CAP_T),
        origin=Origin(xyz=(0.0, 0.0, body_bot_z - BOTTOM_CAP_T)),
        material=gunmetal,
        name="bottom_cap_body",
    )

    # Knurl ridges around the top cap rim (inline visuals, shared helper, loop).
    # Each ridge straddles the cap surface: half embedded in the cap body for
    # mesh connectivity, half protruding outward for visible grip texture.
    top_ridge_mesh = _knurl_ridge("top_knurl_ridge", KNURL_WIDTH, KNURL_DEPTH, TOP_CAP_T)
    for i in range(KNURL_COUNT):
        angle = 2.0 * math.pi * i / KNURL_COUNT
        rx = TOP_CAP_R * math.cos(angle)
        ry = TOP_CAP_R * math.sin(angle)
        cap.visual(
            top_ridge_mesh,
            origin=Origin(xyz=(rx, ry, body_top_z), rpy=(0.0, 0.0, angle)),
            material=knurl_steel,
            name=f"top_ridge_{i}",
        )

    # Knurl ridges around the bottom cap rim (same helper, matching loop).
    bottom_ridge_mesh = _knurl_ridge("bottom_knurl_ridge", KNURL_WIDTH, KNURL_DEPTH, BOTTOM_CAP_T)
    for i in range(KNURL_COUNT):
        angle = 2.0 * math.pi * i / KNURL_COUNT
        rx = BOTTOM_CAP_R * math.cos(angle)
        ry = BOTTOM_CAP_R * math.sin(angle)
        cap.visual(
            bottom_ridge_mesh,
            origin=Origin(xyz=(rx, ry, body_bot_z - BOTTOM_CAP_T), rpy=(0.0, 0.0, angle)),
            material=knurl_steel,
            name=f"bottom_ridge_{i}",
        )

    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=TOP_CAP_R, length=CAP_HUB_LEN),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, hub_center_z)),
    )

    # ---- spinner_body: tri-lobe red plate, spins about +Z relative to cap ----
    body = model.part("spinner_body")
    body.visual(
        mesh_from_cadquery(_tri_lobe_body(), "tri_lobe_body"),
        material=red,
        name="tri_lobe_body",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=LOBE_DIST + LOBE_R, length=BODY_THICK),
        mass=0.045,
        origin=Origin(xyz=(0.0, 0.0, HALF_T)),
    )
    model.articulation(
        "cap_to_body",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, HALF_T)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=80.0),
    )

    # ---- 3 ring bearings, one per lobe, each spinning about its lobe axis ----
    bearing_z = HALF_T  # center the bearing in the plate thickness
    for i, ang in enumerate(LOBE_ANGLES):
        cx = LOBE_DIST * math.cos(ang)
        cy = LOBE_DIST * math.sin(ang)
        name = f"bearing_{i}"
        b = model.part(name)
        # Black rubber outer ring.
        b.visual(
            _bearing_ring_mesh(f"{name}_ring"),
            origin=Origin(xyz=(0.0, 0.0, -BEARING_THICK / 2.0)),
            material=black,
            name=f"{name}_ring",
        )
        # Silver inner race + marker.
        b.visual(
            _bearing_race_mesh(f"{name}_race"),
            origin=Origin(xyz=(0.0, 0.0, -BEARING_THICK / 2.0)),
            material=silver,
            name=f"{name}_race",
        )
        b.inertial = Inertial.from_geometry(
            Cylinder(radius=BEARING_OUTER_R, length=BEARING_THICK),
            mass=0.006,
        )
        # Bearing spins about the lobe's own vertical axis, relative to the body.
        model.articulation(
            f"body_to_{name}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=b,
            origin=Origin(xyz=(cx, cy, bearing_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.1, velocity=80.0),
        )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cap = object_model.get_part("center_cap")
    body = object_model.get_part("spinner_body")
    bearings = [object_model.get_part(f"bearing_{i}") for i in range(3)]
    spin = object_model.get_articulation("cap_to_body")

    # --- Cap is on the central axis and is the held root. ---
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "center cap on the central axis",
        cap_pos is not None and abs(cap_pos[0]) < 1e-4 and abs(cap_pos[1]) < 1e-4,
        details=f"cap origin={cap_pos}",
    )

    # --- Top spin cap stands proud above the spinner body. ---
    top_cap_aabb = ctx.part_element_world_aabb(cap, elem="top_cap_body")
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "top spin cap stands proud above body",
        top_cap_aabb is not None and body_aabb is not None
        and top_cap_aabb[1][2] > body_aabb[1][2] + 0.005,
        details=f"top_cap_max_z={top_cap_aabb[1][2] if top_cap_aabb else None}, "
                f"body_max_z={body_aabb[1][2] if body_aabb else None}",
    )

    # --- Top cap is tall (height >= 0.012m). ---
    top_cap_dims = _ext(top_cap_aabb) if top_cap_aabb else None
    ctx.check(
        "top spin cap is tall (>= 12mm)",
        top_cap_dims is not None and top_cap_dims[2] >= 0.012,
        details=f"top_cap_dims={top_cap_dims}",
    )

    # --- Bottom cap is low profile (height < top cap height). ---
    bottom_cap_aabb = ctx.part_element_world_aabb(cap, elem="bottom_cap_body")
    bottom_cap_dims = _ext(bottom_cap_aabb) if bottom_cap_aabb else None
    ctx.check(
        "bottom cap is lower than top cap",
        bottom_cap_dims is not None and top_cap_dims is not None
        and bottom_cap_dims[2] < top_cap_dims[2],
        details=f"bottom_cap_dims={bottom_cap_dims}, top_cap_dims={top_cap_dims}",
    )

    # --- Knurl ridges exist on both caps (24 each). ---
    top_ridge_count = sum(
        1 for v in cap.visuals if v.name.startswith("top_ridge_")
    )
    bottom_ridge_count = sum(
        1 for v in cap.visuals if v.name.startswith("bottom_ridge_")
    )
    ctx.check(
        "top cap has 24 knurl ridges",
        top_ridge_count == KNURL_COUNT,
        details=f"found {top_ridge_count}",
    )
    ctx.check(
        "bottom cap has 24 knurl ridges",
        bottom_ridge_count == KNURL_COUNT,
        details=f"found {bottom_ridge_count}",
    )

    # --- Knurl ridges are arranged at equal angular pitch around the cap rim. ---
    # Check that ridge centers form a circle at the expected radius.
    top_ridge_positions = []
    for i in range(KNURL_COUNT):
        vaabb = ctx.part_element_world_aabb(cap, elem=f"top_ridge_{i}")
        if vaabb is not None:
            cx = (vaabb[0][0] + vaabb[1][0]) / 2.0
            cy = (vaabb[0][1] + vaabb[1][1]) / 2.0
            top_ridge_positions.append((cx, cy))
    if len(top_ridge_positions) == KNURL_COUNT:
        radii = [math.hypot(p[0], p[1]) for p in top_ridge_positions]
        mean_r = sum(radii) / len(radii)
        ctx.check(
            "top knurl ridges are at uniform radius around cap rim",
            all(abs(r - mean_r) < 0.002 for r in radii),
            details=f"radii={radii}, mean={mean_r}",
        )
        # Check angular spacing: sort by angle and verify ~equal spacing.
        angles = sorted(math.atan2(p[1], p[0]) for p in top_ridge_positions)
        gaps = [(angles[(j + 1) % KNURL_COUNT] - angles[j]) % (2.0 * math.pi)
                for j in range(KNURL_COUNT)]
        expected_gap = 2.0 * math.pi / KNURL_COUNT
        ctx.check(
            "top knurl ridges at equal angular pitch",
            all(abs(g - expected_gap) < 0.15 for g in gaps),
            details=f"gaps_deg={[math.degrees(g) for g in gaps]}",
        )

    # --- Three lobes are arranged at ~120deg around the center. ---
    angs = []
    for i, b in enumerate(bearings):
        p = ctx.part_world_position(b)
        r = math.hypot(p[0], p[1])
        ctx.check(
            f"bearing_{i} sits out on a lobe",
            abs(r - LOBE_DIST) < 0.002,
            details=f"radius={r}",
        )
        angs.append(math.atan2(p[1], p[0]))
    def sep(a, b):
        d = abs((a - b) % (2.0 * math.pi))
        return min(d, 2.0 * math.pi - d)
    ctx.check(
        "three lobes spaced ~120deg apart",
        all(abs(sep(angs[i], angs[j]) - 2.0 * math.pi / 3.0) < 0.10
            for i, j in ((0, 1), (1, 2), (2, 0))),
        details=f"angles={[math.degrees(a) for a in angs]}",
    )

    # --- The tri-lobe body spins about the center relative to the cap. ---
    rest = ctx.part_world_position(bearings[0])
    with ctx.pose({spin: math.pi / 3.0}):
        turned = ctx.part_world_position(bearings[0])
    moved = math.hypot(turned[0] - rest[0], turned[1] - rest[1])
    ctx.check(
        "spinning the body swings a lobe around the center",
        moved > 0.010,
        details=f"rest={rest}, turned={turned}, moved={moved}",
    )
    # Cap must NOT move when the body spins (it is the held root).
    with ctx.pose({spin: math.pi / 3.0}):
        cap_turned = ctx.part_world_position(cap)
    ctx.check(
        "center cap stays put while body spins",
        cap_turned is not None and math.hypot(cap_turned[0], cap_turned[1]) < 1e-4,
        details=f"cap_turned={cap_turned}",
    )

    # --- Each ring bearing spins about its own lobe axis (marker reorients). ---
    for i in range(3):
        joint = object_model.get_articulation(f"body_to_bearing_{i}")
        ext0 = _ext(ctx.part_element_world_aabb(bearings[i], elem=f"bearing_{i}_race"))
        with ctx.pose({joint: math.pi / 2.0}):
            ext90 = _ext(ctx.part_element_world_aabb(bearings[i], elem=f"bearing_{i}_race"))
        ctx.check(
            f"bearing_{i} ring spins about its lobe axis",
            abs(ext0[0] - ext90[1]) < 1e-3 and abs(ext0[0] - ext0[1]) > 1e-4,
            details=f"ext0={ext0}, ext90={ext90}",
        )

    # --- Each bearing is seated in its lobe pocket (intentional press-fit). ---
    for i in range(3):
        ctx.allow_overlap(
            bearings[i],
            body,
            elem_a=f"bearing_{i}_ring",
            elem_b="tri_lobe_body",
            reason="Black rubber bearing ring is press-fit into the lobe pocket.",
        )
        ctx.expect_overlap(
            bearings[i], body, axes="z", min_overlap=0.004,
            name=f"bearing_{i} seated in lobe pocket (thickness)",
        )

    # --- Cap hub seats through the central body bore (intentional). ---
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_hub",
        elem_b="tri_lobe_body",
        reason="Silver cap hub passes through the central bore of the spinner body.",
    )
    ctx.expect_overlap(
        cap, body, axes="z", min_overlap=0.006,
        name="cap hub passes through body center",
    )

    # --- Cap spin joint is the active non-fixed axisymmetric joint. ---
    ctx.check(
        "cap_to_body is a non-fixed continuous spin joint",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={spin.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
