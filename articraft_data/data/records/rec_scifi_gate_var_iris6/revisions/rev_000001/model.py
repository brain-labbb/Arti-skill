from __future__ import annotations

# Futuristic hexagonal-framed sci-fi iris-aperture blast door.
#
# Articraft brief:
# - Object: a wall-mounted sci-fi gate, ~1.86 m wide x ~2.05 m tall x ~0.22 m
#   deep, standing on the floor (geometry from z=0 upward). Faces +Y; width
#   along X. The chamfered-hexagon opening is a true through-opening (no back
#   panel), so the open gate reveals empty space behind it.
# - Root/support: the grey frame slab with a hex-beveled opening, the dark
#   top/bottom guide bars spanning the opening, the cyan light-strip assemblies
#   flanking the opening, the recessed top band, chamfer trim strips, frame
#   bolts, the two dark clamp/latch blocks at mid-height, and six pivot
#   bracket-hub assemblies anchoring the iris petals to the frame jamb.
#   All FIXED.
# - Parts: frame (root); petal_0..petal_5, six triangular armoured iris petals
#   arranged radially around the hex opening center, each on its own revolute
#   joint whose axis is tangent to the iris rim.
# - Articulations: petal_0_joint (REVOLUTE, axis tangent to rim so positive q
#   swings the petal forward +Y to dilate the aperture); petal_1..petal_5 are
#   mimic-coupled with multiplier=1 so all six petals open together uniformly.
# - Visible geometry: hex opening cut into the frame, glowing twin cyan strips
#   on dark backing channels, six triangular armoured petals with reinforcing
#   spines and pivot knuckles that converge at the aperture center when closed,
#   dark bracket-hub assemblies at each petal mounting point, dark guide bars,
#   clamp blocks with status lamps, chamfer trim, and bolts.
# - Support/fit: every petal pivots on a visible bracket-hub mounted on the
#   frame jamb; the bracket arm connects the cylindrical hub to the frame slab
#   through the hex opening edge.
# - Intentional overlaps: adjacent petals overlap along their shared sector
#   edges when closed (intentional iris overlap, allowed + proven); all petals
#   converge and overlap near the aperture center when closed (allowed +
#   proven); each petal knuckle seats inside its pivot hub (small local
#   overlap, allowed + proven).
# - Tests: closed petals cover the aperture center, open petals swing forward
#   (+Y), mimic joints move all petals together, frame/strips/clamps stay
#   fixed and flanking, frame stands on the floor, doorway window clears when
#   open.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Absolute dimensions (meters)
# ---------------------------------------------------------------------------
FRAME_W = 1.86          # overall width along X
FRAME_H = 2.05          # overall height along Z (stands on z=0)
FRAME_D = 0.22          # frame slab depth along Y
FRAME_FACE_Y = FRAME_D / 2.0  # front face plane (+Y)

# Hexagonal opening cut through the frame (chamfered top/bottom corners)
OPEN_W = 1.18           # opening width (flat-side to flat-side) along X
OPEN_H = 1.74           # opening height along Z
OPEN_CHAMFER = 0.30     # how far the angled corner cuts run
OPEN_CY = 1.04          # opening vertical center (z)
OPEN_HW = OPEN_W / 2.0  # 0.59
OPEN_TOP = OPEN_CY + OPEN_H / 2.0
OPEN_BOT = OPEN_CY - OPEN_H / 2.0

# Fixed guide bars (chunky dark bars across the opening).
BAR_HALF_LEN = 0.90
BAR_Y_LO = -0.08
BAR_Y_HI = 0.075
TOP_BAR_Z = (1.78, 1.92)
BOT_BAR_Z = (0.16, 0.30)

# ---------------------------------------------------------------------------
# Iris aperture parameters
# ---------------------------------------------------------------------------
N_PETALS = 6
IRIS_R = 0.50            # iris radius (pivot circle) from opening center
PETAL_THICK = 0.022      # armoured plate thickness along Y
PIVOT_Y = 0.055          # Y position of pivot points (within frame depth)
IRIS_OPEN_ANGLE = 1.05   # max opening angle (~60 deg); positive q opens +Y
HALF_SPAN_ANGLE_DEG = 33.0  # half-angle of each petal sector (>30 for overlap)


# ---------------------------------------------------------------------------
# Mesh helpers
# ---------------------------------------------------------------------------

def _hex_opening_profile(w: float, h: float, chamf: float) -> list[tuple[float, float]]:
    """Return a centered elongated-hexagon outline (chamfered top/bottom)."""
    hw = w / 2.0
    hh = h / 2.0
    cz = h / 2.0 - chamf
    return [
        (-hw, -cz),
        (-hw, cz),
        (-hw + chamf, hh),
        (hw - chamf, hh),
        (hw, cz),
        (hw, -cz),
        (hw - chamf, -hh),
        (-hw + chamf, -hh),
    ]


def _frame_mesh() -> cq.Workplane:
    """Grey frame slab with a chamfered-hexagon opening cut through it."""
    slab = cq.Workplane("XY").box(FRAME_W, FRAME_D, FRAME_H, centered=(True, True, False))
    pts = _hex_opening_profile(OPEN_W, OPEN_H, OPEN_CHAMFER)
    cutter = (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
        .extrude(FRAME_D * 2.0, both=True)
        .translate((0.0, 0.0, OPEN_CY))
    )
    frame = slab.cut(cutter)
    frame = frame.edges("|Y").edges(">Y").chamfer(0.012)
    return frame


def _guide_bar_mesh(z_lo: float, z_hi: float) -> cq.Workplane:
    """A fixed dark guide bar spanning the opening."""
    bar = (
        cq.Workplane("XY")
        .box(2.0 * BAR_HALF_LEN, BAR_Y_HI - BAR_Y_LO, z_hi - z_lo, centered=(True, True, True))
        .translate((0.0, (BAR_Y_LO + BAR_Y_HI) / 2.0, (z_lo + z_hi) / 2.0))
    )
    return bar


def _top_recess_mesh() -> cq.Workplane:
    """Dark recessed band across the top of the frame face."""
    band_lo = OPEN_TOP + 0.02
    band_hi = FRAME_H - 0.04
    cz = (band_lo + band_hi) / 2.0
    h = band_hi - band_lo
    band = (
        cq.Workplane("XY")
        .box(OPEN_W - 0.10, 0.05, h, centered=(True, True, True))
        .translate((0.0, FRAME_FACE_Y - 0.024, cz))
    )
    return band


def _clamp_block_mesh(sign: float) -> cq.Workplane:
    """Dark mechanical clamp/latch block at mid-height on one side."""
    x = sign * (OPEN_HW + 0.085)
    body = (
        cq.Workplane("XY")
        .box(0.18, 0.16, 0.26, centered=(True, True, True))
        .edges("|Y").fillet(0.02)
        .translate((x, FRAME_FACE_Y + 0.02, OPEN_CY))
    )
    pin = (
        cq.Workplane("XY")
        .cylinder(0.07, 0.04, centered=(True, True, True))
        .rotate((0, 0, 0), (1, 0, 0), 90.0)
        .translate((x, FRAME_FACE_Y + 0.10, OPEN_CY))
    )
    return body.union(pin)


def _bracket_hub_mesh(theta: float) -> cq.Workplane:
    """Bracket arm + cylindrical pivot hub at one iris petal mounting point.

    The bracket extends radially outward from the iris rim to the frame jamb,
    connecting the cylindrical pivot hub to the frame slab.  Built directly
    in world coordinates so the bracket end overlaps the frame solid for a
    grounded structural connection.
    """
    hub_r = 0.032
    hub_len = 0.072
    bracket_w = 0.024     # width along tangent (Z in local)
    bracket_h = 0.030     # height along Y
    # Bracket length: from hub center to well past the hex jamb.
    bracket_len = 0.42

    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    px = IRIS_R * cos_t
    pz = OPEN_CY + IRIS_R * sin_t

    # Hub: cylinder along Z, then rotate so Z aligns with tangent direction.
    # Tangent at theta = (-sin(theta), 0, cos(theta)) = Ry(-theta) * (0,0,1).
    hub = (
        cq.Workplane("XY")
        .cylinder(hub_len, hub_r, centered=(True, True, True))
        .rotate((0, 0, 0), (0, 1, 0), -math.degrees(theta))
        .translate((px, PIVOT_Y, pz))
    )

    # Bracket: box from hub center extending radially outward.
    # In local frame: +X = radial outward = (cos_t, 0, sin_t) in world.
    # Build along world +X then rotate into position.
    mid_r = IRIS_R + bracket_len / 2.0
    mid_x = mid_r * cos_t
    mid_z = OPEN_CY + mid_r * sin_t
    bracket = (
        cq.Workplane("XY")
        .box(bracket_len, bracket_h, bracket_w, centered=(True, True, True))
        .rotate((0, 0, 0), (0, 1, 0), -math.degrees(theta))
        .translate((mid_x, PIVOT_Y, mid_z))
    )

    return hub.union(bracket)


def _petal_mesh() -> cq.Workplane:
    """Triangular armoured iris petal in the petal's local frame.

    Local frame (matches the articulation frame at q=0):
    - Origin at pivot vertex (rim point)
    - -X points radially inward toward the aperture center
    - Y is perpendicular to the door face (thickness direction)
    - Z is tangent to the iris rim (the rotation axis)

    The petal is a triangular plate with a reinforcing spine along the
    center line and a short barrel knuckle at the pivot end that wraps
    around the frame's pivot hub.
    """
    half_span = IRIS_R * math.tan(math.radians(HALF_SPAN_ANGLE_DEG))
    petal_len = IRIS_R  # base vertices reach the aperture center

    # Main triangular plate (vertices in XZ workplane: x, z coords).
    pts = [
        (0.0, 0.0),                     # pivot vertex
        (-petal_len, -half_span),        # base left
        (-petal_len, half_span),         # base right
    ]
    plate = (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
        .extrude(PETAL_THICK)
        .translate((0.0, -PETAL_THICK / 2.0, 0.0))
    )

    # Reinforcing spine along the petal center line (protrudes on both faces).
    spine_w = 0.034
    spine_extra = PETAL_THICK * 0.55
    spine_len = petal_len * 0.60
    spine_x_mid = -(0.08 + spine_len / 2.0)
    spine_total_h = PETAL_THICK + 2.0 * spine_extra
    spine = (
        cq.Workplane("XZ")
        .center(spine_x_mid, 0.0)
        .rect(spine_len, spine_w)
        .extrude(spine_total_h)
        .translate((0.0, -spine_total_h / 2.0, 0.0))
    )
    body = plate.union(spine)

    # Short barrel knuckle at the pivot end (cylinder along local Z).
    # Slightly smaller than the frame hub so it nests inside.
    knuckle_r = 0.028
    knuckle_len = spine_total_h + 0.008
    knuckle = (
        cq.Workplane("XY")
        .cylinder(knuckle_len, knuckle_r, centered=(True, True, True))
    )
    body = body.union(knuckle)

    # Small edge ribs near the base for armoured panel detail.
    rib_w = 0.018
    rib_extra = PETAL_THICK * 0.35
    rib_total_h = PETAL_THICK + 2.0 * rib_extra
    for z_off in (-half_span * 0.55, half_span * 0.55):
        rib = (
            cq.Workplane("XZ")
            .center(-petal_len * 0.50, z_off)
            .rect(petal_len * 0.55, rib_w)
            .extrude(rib_total_h)
            .translate((0.0, -rib_total_h / 2.0, 0.0))
        )
        body = body.union(rib)

    return body


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="scifi_hex_iris_door")

    grey = model.material("frame_grey", rgba=(0.85, 0.86, 0.88, 1.0))
    cyan = model.material("glow_cyan", rgba=(0.45, 0.98, 1.0, 1.0))
    clamp_dark = model.material("clamp_charcoal", rgba=(0.09, 0.10, 0.11, 1.0))
    petal_gunmetal = model.material("petal_gunmetal", rgba=(0.20, 0.22, 0.26, 1.0))
    status_red = model.material("status_red", rgba=(0.85, 0.16, 0.12, 1.0))

    # ---- Fixed root: frame + guide bars + light strips + clamps + trim ----
    frame = model.part("frame")
    frame.visual(mesh_from_cadquery(_frame_mesh(), "frame_slab"), material=grey, name="frame_slab")
    frame.visual(
        mesh_from_cadquery(_top_recess_mesh(), "top_recess"),
        material=clamp_dark,
        name="top_recess",
    )
    frame.visual(
        mesh_from_cadquery(_guide_bar_mesh(*TOP_BAR_Z), "top_track"),
        material=clamp_dark,
        name="top_track",
    )
    frame.visual(
        mesh_from_cadquery(_guide_bar_mesh(*BOT_BAR_Z), "bottom_track"),
        material=clamp_dark,
        name="bottom_track",
    )
    frame.visual(
        mesh_from_cadquery(_clamp_block_mesh(-1.0), "clamp_0"),
        material=clamp_dark,
        name="clamp_0",
    )
    frame.visual(
        mesh_from_cadquery(_clamp_block_mesh(1.0), "clamp_1"),
        material=clamp_dark,
        name="clamp_1",
    )
    # Red status lamps on clamp blocks.
    for s, lamp in ((-1.0, "clamp_lamp_0"), (1.0, "clamp_lamp_1")):
        frame.visual(
            Box((0.05, 0.012, 0.03)),
            origin=Origin(xyz=(s * (OPEN_HW + 0.085), FRAME_FACE_Y + 0.106, OPEN_CY + 0.09)),
            material=status_red,
            name=lamp,
        )

    # Twin glowing cyan strips on dark backing channels flanking the opening.
    strip_h = 1.16
    backing_w = 0.055
    backing_cx = OPEN_HW + 0.01 - backing_w / 2.0
    for s, tag in ((-1.0, "0"), (1.0, "1")):
        frame.visual(
            Box((backing_w, 0.023, strip_h)),
            origin=Origin(xyz=(s * backing_cx, 0.0835, OPEN_CY)),
            material=clamp_dark,
            name=f"strip_backing_{tag}",
        )
        frame.visual(
            Box((0.018, 0.012, strip_h - 0.06)),
            origin=Origin(xyz=(s * (backing_cx - 0.0155), 0.101, OPEN_CY)),
            material=cyan,
            name=f"light_strip_{tag}",
        )
        frame.visual(
            Box((0.018, 0.012, strip_h - 0.06)),
            origin=Origin(xyz=(s * (backing_cx + 0.0125), 0.101, OPEN_CY)),
            material=cyan,
            name=f"light_strip_{tag}_inner",
        )

    # Dark trim strips along the four opening chamfers.
    chamf_mid_x = OPEN_HW - OPEN_CHAMFER / 2.0
    chamf_top_z = OPEN_TOP - OPEN_CHAMFER / 2.0
    chamf_bot_z = OPEN_BOT + OPEN_CHAMFER / 2.0
    chamf_len = OPEN_CHAMFER * math.sqrt(2.0) - 0.02
    for sx in (-1.0, 1.0):
        for cz, sz, vtag in ((chamf_top_z, 1.0, "t"), (chamf_bot_z, -1.0, "b")):
            pitch = sx * sz * math.pi / 4.0
            frame.visual(
                Box((chamf_len, 0.012, 0.05)),
                origin=Origin(
                    xyz=(sx * (chamf_mid_x + 0.035), FRAME_FACE_Y + 0.006, cz + sz * 0.035),
                    rpy=(0.0, pitch, 0.0),
                ),
                material=clamp_dark,
                name=f"chamfer_trim_{'l' if sx < 0 else 'r'}{vtag}",
            )

    # Bolt studs on the frame front face.
    for sx in (-1.0, 1.0):
        for bz, btag in ((0.30, "lo"), (1.04, "mid"), (1.78, "hi")):
            frame.visual(
                Box((0.03, 0.012, 0.03)),
                origin=Origin(xyz=(sx * 0.80, FRAME_FACE_Y + 0.006, bz)),
                material=clamp_dark,
                name=f"bolt_{'l' if sx < 0 else 'r'}_{btag}",
            )

    # Pivot bracket-hub assemblies at each petal mounting point.
    for i in range(N_PETALS):
        theta = i * 2.0 * math.pi / N_PETALS
        frame.visual(
            mesh_from_cadquery(_bracket_hub_mesh(theta), f"pivot_hub_{i}"),
            material=clamp_dark,
            name=f"pivot_hub_{i}",
        )

    # ---- Iris aperture petals (shared geometry, loop-emitted) ----
    petal_geom = mesh_from_cadquery(_petal_mesh(), "iris_petal")

    petals = []
    for i in range(N_PETALS):
        p = model.part(f"petal_{i}")
        p.visual(petal_geom, material=petal_gunmetal, name="plate")
        petals.append(p)

    # ---- Articulations: revolute iris joints (driver + mimics) ----
    driver_joint = None
    for i in range(N_PETALS):
        theta = i * 2.0 * math.pi / N_PETALS
        px = IRIS_R * math.cos(theta)
        pz = OPEN_CY + IRIS_R * math.sin(theta)

        # Orient the articulation frame so that local +Z aligns with the
        # tangent direction at this rim point: (-sin(theta), 0, cos(theta)).
        # Ry(-theta) maps +Z to exactly that direction.
        artic_origin = Origin(
            xyz=(px, PIVOT_Y, pz),
            rpy=(0.0, -theta, 0.0),
        )

        # Axis (0, 0, -1) in the articulation frame = negative tangent in
        # world. Positive rotation about -Z swings the -X end (toward center)
        # toward +Y (forward), dilating the aperture.
        limits = MotionLimits(
            effort=600.0, velocity=0.8, lower=0.0, upper=IRIS_OPEN_ANGLE,
        )

        if i == 0:
            driver_joint = model.articulation(
                "petal_0_joint",
                ArticulationType.REVOLUTE,
                parent=frame,
                child=petals[0],
                origin=artic_origin,
                axis=(0.0, 0.0, -1.0),
                motion_limits=limits,
            )
        else:
            model.articulation(
                f"petal_{i}_joint",
                ArticulationType.REVOLUTE,
                parent=frame,
                child=petals[i],
                origin=artic_origin,
                axis=(0.0, 0.0, -1.0),
                motion_limits=limits,
                mimic=Mimic(joint=driver_joint.name, multiplier=1.0, offset=0.0),
            )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    petals = [object_model.get_part(f"petal_{i}") for i in range(N_PETALS)]
    driver = object_model.get_articulation("petal_0_joint")

    # --- Intentional local overlaps ---

    # Adjacent petals overlap along shared sector edges when closed.
    for i in range(N_PETALS):
        j = (i + 1) % N_PETALS
        ctx.allow_overlap(
            petals[i],
            petals[j],
            elem_a="plate",
            elem_b="plate",
            reason=(
                f"Iris petals {i} and {j} overlap along their shared sector "
                "edges when the aperture is closed (33-deg half-span exceeds "
                "the 30-deg sector boundary for complete coverage)."
            ),
        )

    # Non-adjacent petals converge and overlap near the aperture center.
    for i in range(N_PETALS):
        for j in range(i + 2, N_PETALS):
            if j - i == N_PETALS - 1:
                continue  # already covered by adjacent pair
            ctx.allow_overlap(
                petals[i],
                petals[j],
                elem_a="plate",
                elem_b="plate",
                reason=(
                    f"Iris petals {i} and {j} converge and overlap near the "
                    "aperture center when the aperture is closed."
                ),
            )

    # Each petal knuckle seats inside its pivot hub on the frame.
    for i in range(N_PETALS):
        ctx.allow_overlap(
            petals[i],
            frame,
            elem_a="plate",
            elem_b=f"pivot_hub_{i}",
            reason=(
                f"Petal {i} knuckle wraps around the pivot hub; the barrel "
                "and hub are concentric cylinders at the mounting point."
            ),
        )

    # --- Hero geometry present and placed ---

    frame_aabb = ctx.part_world_aabb(frame)
    ctx.check(
        "frame stands on the floor and reaches full height",
        frame_aabb is not None
        and abs(frame_aabb[0][2]) < 0.02
        and frame_aabb[1][2] > FRAME_H - 0.05,
        details=f"frame aabb={frame_aabb}",
    )

    # Cyan light strips flank the opening on the +Y face.
    strip0 = ctx.part_element_world_aabb(frame, elem="light_strip_0")
    strip1 = ctx.part_element_world_aabb(frame, elem="light_strip_1")
    ctx.check(
        "cyan light strips flank the opening left and right",
        strip0 is not None
        and strip1 is not None
        and strip0[1][0] < 0.0 < strip1[0][0],
        details=f"strip0={strip0}, strip1={strip1}",
    )

    # Clamp blocks sit at mid-height, proud of the frame front face.
    clamp0 = ctx.part_element_world_aabb(frame, elem="clamp_0")
    ctx.check(
        "clamp block sits at mid-height proud of the front face",
        clamp0 is not None
        and clamp0[1][1] > FRAME_FACE_Y
        and clamp0[0][2] < OPEN_CY < clamp0[1][2],
        details=f"clamp0={clamp0}",
    )

    # --- 6 petals exist and are radially arranged ---
    ctx.check(
        "all 6 iris petals exist",
        len(petals) == N_PETALS and all(p is not None for p in petals),
    )

    # Verify radial arrangement: all petal origins are at approximately
    # the iris radius from the opening center axis.
    radii = []
    for p in petals:
        pos = ctx.part_world_position(p)
        assert pos is not None
        r = math.sqrt(pos[0] ** 2 + (pos[2] - OPEN_CY) ** 2)
        radii.append(r)
    avg_r = sum(radii) / len(radii)
    ctx.check(
        "petals are arranged at approximately equal radius from opening center",
        all(abs(r - avg_r) < 0.05 for r in radii) and abs(avg_r - IRIS_R) < 0.05,
        details=f"radii={[round(r, 3) for r in radii]}, avg={avg_r:.3f}, expected={IRIS_R}",
    )

    # --- Closed pose: petals cover the aperture ---
    closed_aabbs = {}
    with ctx.pose({driver: 0.0}):
        for i, p in enumerate(petals):
            aabb = ctx.part_world_aabb(p)
            assert aabb is not None
            closed_aabbs[i] = aabb

        # Adjacent petals overlap in Z (their sector edges interleave).
        for i in range(N_PETALS):
            j = (i + 1) % N_PETALS
            ctx.expect_overlap(
                petals[i],
                petals[j],
                axes="z",
                elem_a="plate",
                elem_b="plate",
                min_overlap=0.10,
                name=f"adjacent petals {i} and {j} overlap in Z when closed",
            )

        # Each petal's plate reaches close to the aperture center in XY
        # projection (covering the opening when all are closed).
        for i, p in enumerate(petals):
            aabb = ctx.part_element_world_aabb(p, elem="plate")
            assert aabb is not None
            # The plate AABB should include points within 0.08 m of the
            # opening center in X.
            ctx.check(
                f"petal_{i} plate extends near aperture center when closed",
                aabb[0][0] < 0.08 and aabb[1][0] > -0.08,
                details=f"petal_{i} plate x=({aabb[0][0]:.3f}, {aabb[1][0]:.3f})",
            )

        # Petals are near the aperture plane (not swung forward).
        for i, p in enumerate(petals):
            aabb = ctx.part_world_aabb(p)
            assert aabb is not None
            max_y = aabb[1][1]
            ctx.check(
                f"petal_{i} is near the aperture plane when closed",
                max_y < PIVOT_Y + 0.08,
                details=f"petal_{i} max_y={max_y:.3f}",
            )

    # --- Open pose: petals swing forward (+Y) to dilate ---
    with ctx.pose({driver: driver.motion_limits.upper}):
        for i, p in enumerate(petals):
            aabb = ctx.part_world_aabb(p)
            assert aabb is not None
            max_y = aabb[1][1]
            center_y = (aabb[0][1] + aabb[1][1]) / 2.0
            # The petal's forward-most point should be well past the frame
            # face when the iris is fully open.
            ctx.check(
                f"petal_{i} swings forward (+Y) when iris opens",
                max_y > FRAME_FACE_Y + 0.15,
                details=f"petal_{i} max_y={max_y:.3f}, center_y={center_y:.3f}",
            )
            # The AABB center should move forward compared to the closed pose.
            closed_center_y = (closed_aabbs[i][0][1] + closed_aabbs[i][1][1]) / 2.0
            ctx.check(
                f"petal_{i} AABB center moves forward when opening",
                center_y > closed_center_y + 0.08,
                details=f"closed_center_y={closed_center_y:.3f}, open_center_y={center_y:.3f}",
            )

    # --- Doorway window: no fixed frame furniture blocks the central passage ---
    window_x = 0.35
    window_z = (0.55, 1.50)
    for elem in ("top_recess", "top_track", "bottom_track", "strip_backing_0", "strip_backing_1"):
        aabb = ctx.part_element_world_aabb(frame, elem=elem)
        assert aabb is not None
        blocks = (
            aabb[0][0] < window_x
            and aabb[1][0] > -window_x
            and aabb[0][2] < window_z[1]
            and aabb[1][2] > window_z[0]
        )
        ctx.check(
            f"frame '{elem}' stays clear of the open doorway window",
            not blocks,
            details=f"{elem} aabb={aabb}",
        )

    return ctx.report()


object_model = build_object_model()
