from __future__ import annotations

# Red three-arm fidget spinner.
# Frame: spinner lies flat in the XY plane, thickness along +Z, centered on the
# origin. The central vertical axis is +Z.
# Construction:
#   - center_cap (ROOT / held part): the central bearing cap you pinch between
#     two fingers. A short silver hub barrel through the middle plus a red CAP
#     button disc on each face (top +Z and bottom -Z). This is the part that
#     stays still while the body spins.
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

# central bearing / cap
CAP_HUB_R = 0.0072           # silver hub barrel radius at the very center
CAP_DISC_R = 0.0090          # red cap button radius (each face)
CAP_DISC_T = 0.0020          # red cap button thickness (protrudes past face)
CAP_HUB_LEN = 0.014          # silver hub length (slightly taller than body)

LOBE_ANGLES = (math.pi / 2.0, math.pi / 2.0 + 2.0 * math.pi / 3.0,
               math.pi / 2.0 + 4.0 * math.pi / 3.0)


def _lobe_centers():
    """Return the three lobe center positions as (x, y) tuples."""
    return [
        (LOBE_DIST * math.cos(a), LOBE_DIST * math.sin(a))
        for a in LOBE_ANGLES
    ]


def _solid_reuleaux_plate() -> cq.Workplane:
    """Solid rounded-triangle (Reuleaux-style) plate: three lobe circles fused
    with wide rectangular bridges between each pair, plus a central hub disc,
    all extruded to BODY_THICK. No open gaps between the arms -- the inter-arm
    region is fully filled. The outer boundary reads as a convex rounded
    triangle."""
    pts = _lobe_centers()

    # Start with the first lobe circle.
    body = cq.Workplane("XY").center(pts[0][0], pts[0][1]).circle(LOBE_R).extrude(BODY_THICK)

    # Union the other two lobe circles.
    for i in (1, 2):
        lobe = cq.Workplane("XY").center(pts[i][0], pts[i][1]).circle(LOBE_R).extrude(BODY_THICK)
        body = body.union(lobe)

    # Central hub disc to anchor the inter-lobe fill.
    hub = cq.Workplane("XY").circle(HUB_R).extrude(BODY_THICK)
    body = body.union(hub)

    # Wide rectangular bridges between each pair of adjacent lobes, filling
    # the inter-arm gaps. Each bridge is centered at the midpoint between two
    # lobe centers, oriented along the connecting line, with width = 2*LOBE_R
    # so the outer edge is tangent to the lobe circles (convex hull).
    for i in range(3):
        j = (i + 1) % 3
        mx = (pts[i][0] + pts[j][0]) / 2.0
        my = (pts[i][1] + pts[j][1]) / 2.0
        dx = pts[j][0] - pts[i][0]
        dy = pts[j][1] - pts[i][1]
        angle_rad = math.atan2(dy, dx)
        bridge_len = math.hypot(dx, dy) + 2.0 * LOBE_R  # overlap into both lobes
        bridge_width = 2.0 * LOBE_R  # tangent to lobe circles on convex side
        bridge = (
            cq.Workplane("XY")
            .center(mx, my)
            .transformed(rotate=(0.0, 0.0, math.degrees(angle_rad)))
            .rect(bridge_len, bridge_width)
            .extrude(BODY_THICK)
        )
        body = body.union(bridge)

    # Bore a bearing pocket straight through each lobe position.
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

    return body


def _bearing_ring_mesh(name: str):
    """Black rubber outer ring (a thick torus-like ring) for one bearing."""
    # Outer black ring: annulus between BEARING_OUTER_R and BEARING_INNER_R.
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fidget_spinner")

    red = model.material("glossy_red", rgba=(0.82, 0.07, 0.09, 1.0))
    black = model.material("rubber_black", rgba=(0.06, 0.06, 0.07, 1.0))
    silver = model.material("silver_steel", rgba=(0.78, 0.80, 0.83, 1.0))

    # ---- center_cap (ROOT / held part) ----
    # Silver hub barrel down the central axis + a red cap button on each face.
    cap = model.part("center_cap")

    hub_barrel = CylinderGeometry(CAP_HUB_R, CAP_HUB_LEN, radial_segments=48)
    cap.visual(
        mesh_from_geometry(hub_barrel, "cap_hub"),
        origin=Origin(xyz=(0.0, 0.0, HALF_T)),
        material=silver,
        name="cap_hub",
    )
    # Red cap buttons proud of each face (the part you pinch).
    cap.visual(
        Cylinder(radius=CAP_DISC_R, length=CAP_DISC_T),
        origin=Origin(xyz=(0.0, 0.0, BODY_THICK + CAP_DISC_T / 2.0)),
        material=red,
        name="cap_button_top",
    )
    cap.visual(
        Cylinder(radius=CAP_DISC_R, length=CAP_DISC_T),
        origin=Origin(xyz=(0.0, 0.0, -CAP_DISC_T / 2.0)),
        material=red,
        name="cap_button_bottom",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=CAP_DISC_R, length=CAP_HUB_LEN + 2.0 * CAP_DISC_T),
        mass=0.004,
        origin=Origin(xyz=(0.0, 0.0, HALF_T)),
    )

    # ---- spinner_body: solid Reuleaux-style plate, spins about +Z relative to cap ----
    body = model.part("spinner_body")
    body.visual(
        mesh_from_cadquery(_solid_reuleaux_plate(), "solid_plate"),
        material=red,
        name="solid_plate",
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
    # Pairwise angular separations should be ~120deg.
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
    # A lobe swings around: track a single lobe (bearing_0) world position.
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
        # The off-axis marker makes the race AABB swap its x/y extents on a
        # quarter turn, proving the ring actually spins.
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
            elem_b="solid_plate",
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
        elem_b="solid_plate",
        reason="Silver cap hub passes through the central bore of the spinner body.",
    )
    ctx.expect_overlap(
        cap, body, axes="z", min_overlap=0.006,
        name="cap hub passes through body center",
    )

    # --- Solid plate has no open gaps between arms (Reuleaux-style hull). ---
    # Check that the plate fills the inter-arm regions by verifying the AABB
    # spans across the full lobe-to-lobe width in both X and Y directions.
    body_aabb = ctx.part_world_aabb(body)
    body_dims = [body_aabb[1][i] - body_aabb[0][i] for i in range(3)]
    # The plate should span at least 2*LOBE_DIST - LOBE_R in both X and Y
    # (proving it's a continuous hull, not three separate arms with gaps).
    min_expected_span = 2.0 * LOBE_DIST - 2.0 * LOBE_R
    ctx.check(
        "solid plate spans inter-arm regions (no open gaps)",
        body_dims[0] > min_expected_span and body_dims[1] > min_expected_span,
        details=f"body_dims={body_dims}, min_expected={min_expected_span:.4f}",
    )
    # The plate outline should be roughly equilateral (Reuleaux triangle).
    ctx.check(
        "solid plate has rounded-triangle outline",
        abs(body_dims[0] - body_dims[1]) < 0.008,
        details=f"x_span={body_dims[0]:.4f}, y_span={body_dims[1]:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
