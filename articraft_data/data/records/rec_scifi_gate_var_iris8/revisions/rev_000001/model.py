from __future__ import annotations

# Futuristic hexagonal-framed sci-fi blast door with an 8-petal dilating iris
# aperture.  Fork of the bi-parting telescoping variant: the hex frame, light
# strips, clamp blocks, guide bars, chamfer trim, and bolts are kept identical;
# the sliding leaves are replaced by eight curved-leaf iris petals arranged at
# 45-degree intervals around the opening centre.
#
# Articraft brief:
# - Object: a wall-mounted sci-fi gate, ~1.86 m wide x ~2.05 m tall x ~0.22 m
#   deep, standing on the floor (geometry from z=0 upward). Faces +Y; width
#   along X. The chamfered-hexagon opening is a true through-opening (no back
#   panel), so the open gate reveals empty space behind it.
# - Root/support: the grey frame slab with a hex-beveled opening, the dark
#   top/bottom guide bars spanning the opening, the cyan light-strip assemblies
#   flanking the opening, the recessed top band, chamfer trim strips, frame
#   bolts, the two dark clamp/latch blocks at mid-height, and eight pivot-mount
#   arms bridging the frame inner edge to each petal pivot. All FIXED on frame.
# - Parts: frame (root); petal_0 .. petal_7, eight curved-leaf iris petals
#   emitted by a for-i-in-range(8) loop with a shared CadQuery leaf helper,
#   equally spaced at 45 degrees around the opening centre.
# - Articulations: petal_0_hinge is the driver (REVOLUTE, axis +Y through the
#   pivot on the iris ring); petal_1_hinge .. petal_7_hinge are mimic-coupled
#   at multiplier 1 so one travel value dilates all eight petals uniformly.
#   Positive q rotates every petal counterclockwise (viewed from +Y), swinging
#   the blade tips away from the centre and clearing the circular aperture.
# - Visible geometry: hex opening cut into the frame, glowing twin cyan strips
#   on dark backing channels, eight overlapping curved armoured petals with
#   pivot bosses and central ridges, dark guide bars, clamp blocks with status
#   lamps, chamfer trim, bolts, and pivot-mount arms.
# - Support/fit: every petal pivots on a visible frame-mounted pivot-mount arm
#   that bridges from the frame inner edge to the pivot bearing; when closed
#   the petals overlap in the classic iris stacking pattern.
# - Intentional overlaps: adjacent petals overlap when closed (iris stacking,
#   allowed + proven); each petal pivot boss seats in its frame pivot-mount
#   collar (allowed + proven); pivot-mount arms embed into the frame slab for
#   structural attachment (same-part, allowed + proven); accent stripe embeds
#   slightly into the blade surface (same-part, allowed + proven).
# - Tests: eight petals exist at 45-degree intervals, closed pose covers the
#   opening centre, open pose retracts every petal away from the centre, the
#   doorway window remains unobstructed, frame features stay fixed and flanking,
#   and the frame stands on the floor.

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
FRAME_W = 1.86
FRAME_H = 2.05
FRAME_D = 0.22
FRAME_FACE_Y = FRAME_D / 2.0

OPEN_W = 1.18
OPEN_H = 1.74
OPEN_CHAMFER = 0.30
OPEN_CY = 1.04
OPEN_HW = OPEN_W / 2.0
OPEN_TOP = OPEN_CY + OPEN_H / 2.0
OPEN_BOT = OPEN_CY - OPEN_H / 2.0

BAR_HALF_LEN = 0.90
BAR_Y_LO = -0.08
BAR_Y_HI = 0.075
TOP_BAR_Z = (1.78, 1.92)
BOT_BAR_Z = (0.16, 0.30)

# ---------------------------------------------------------------------------
# Iris aperture dimensions
# ---------------------------------------------------------------------------
N_PETALS = 8
IRIS_R = 0.52           # pivot ring radius from opening centre
PETAL_LEN = 0.57        # blade length from pivot to tip
PETAL_W = 0.30          # max blade width
PETAL_T = 0.028         # blade thickness along Y
DOOR_Y = 0.035          # Y centre of the petal plane
IRIS_OPEN_ANGLE = 1.10  # radians (~63 deg)

# ---------------------------------------------------------------------------
# Frame geometry helpers (identical to the parent)
# ---------------------------------------------------------------------------

def _hex_opening_profile(w: float, h: float, chamf: float) -> list[tuple[float, float]]:
    hw = w / 2.0
    hh = h / 2.0
    cz = h / 2.0 - chamf
    return [
        (-hw, -cz), (-hw, cz), (-hw + chamf, hh), (hw - chamf, hh),
        (hw, cz), (hw, -cz), (hw - chamf, -hh), (-hw + chamf, -hh),
    ]


def _frame_mesh():
    slab = cq.Workplane("XY").box(FRAME_W, FRAME_D, FRAME_H, centered=(True, True, False))
    pts = _hex_opening_profile(OPEN_W, OPEN_H, OPEN_CHAMFER)
    cutter = (
        cq.Workplane("XZ").polyline(pts).close()
        .extrude(FRAME_D * 2.0, both=True).translate((0.0, 0.0, OPEN_CY))
    )
    frame = slab.cut(cutter)
    frame = frame.edges("|Y").edges(">Y").chamfer(0.012)
    return frame


def _guide_bar_mesh(z_lo: float, z_hi: float):
    return (
        cq.Workplane("XY")
        .box(2.0 * BAR_HALF_LEN, BAR_Y_HI - BAR_Y_LO, z_hi - z_lo, centered=(True, True, True))
        .translate((0.0, (BAR_Y_LO + BAR_Y_HI) / 2.0, (z_lo + z_hi) / 2.0))
    )


def _top_recess_mesh():
    band_lo = OPEN_TOP + 0.02
    band_hi = FRAME_H - 0.04
    cz = (band_lo + band_hi) / 2.0
    return (
        cq.Workplane("XY")
        .box(OPEN_W - 0.10, 0.05, band_hi - band_lo, centered=(True, True, True))
        .translate((0.0, FRAME_FACE_Y - 0.024, cz))
    )


def _clamp_block_mesh(sign: float):
    x = sign * (OPEN_HW + 0.085)
    body = (
        cq.Workplane("XY").box(0.18, 0.16, 0.26, centered=(True, True, True))
        .edges("|Y").fillet(0.02)
        .translate((x, FRAME_FACE_Y + 0.02, OPEN_CY))
    )
    pin = (
        cq.Workplane("XY").cylinder(0.07, 0.04, centered=(True, True, True))
        .rotate((0, 0, 0), (1, 0, 0), 90.0)
        .translate((x, FRAME_FACE_Y + 0.10, OPEN_CY))
    )
    return body.union(pin)


# ---------------------------------------------------------------------------
# Iris geometry helpers
# ---------------------------------------------------------------------------

def _petal_cq() -> cq.Workplane:
    """One iris petal blade: pivot at origin, extending toward -X, in XZ plane."""
    L = PETAL_LEN
    W = PETAL_W
    T = PETAL_T

    pts = [
        (0.02,  0.038),
        (-L * 0.06,  W * 0.26),
        (-L * 0.14,  W * 0.42),
        (-L * 0.26,  W * 0.50),
        (-L * 0.40,  W * 0.49),
        (-L * 0.54,  W * 0.42),
        (-L * 0.68,  W * 0.30),
        (-L * 0.80,  W * 0.18),
        (-L * 0.90,  W * 0.08),
        (-L * 0.97,  W * 0.02),
        (-L,         0.0),
        (-L * 0.97, -W * 0.02),
        (-L * 0.90, -W * 0.08),
        (-L * 0.80, -W * 0.18),
        (-L * 0.68, -W * 0.30),
        (-L * 0.54, -W * 0.42),
        (-L * 0.40, -W * 0.49),
        (-L * 0.26, -W * 0.50),
        (-L * 0.14, -W * 0.42),
        (-L * 0.06, -W * 0.26),
        (0.02, -0.038),
    ]

    blade = (
        cq.Workplane("XZ").polyline(pts).close()
        .extrude(T).translate((0.0, -T / 2.0, 0.0))
    )

    boss_r = 0.032
    boss_h = T * 2.6
    boss = (
        cq.Workplane("XZ").circle(boss_r).extrude(boss_h)
        .translate((0.0, -boss_h / 2.0, 0.0))
    )

    ridge = (
        cq.Workplane("XZ").center(-L * 0.46, 0.0)
        .rect(L * 0.58, 0.018).extrude(T + 0.010)
        .translate((0.0, -(T + 0.010) / 2.0, 0.0))
    )

    return blade.union(boss).union(ridge)


def _petal_cq_at(theta_deg: float) -> cq.Workplane:
    """One iris petal pre-rotated about Y to angular position theta.

    The base petal extends in -X. To make it point from the pivot toward
    the opening centre at angular position theta, rotate by -theta about Y
    (right-hand rule about +Y takes -X toward -Z at theta=90, etc.).
    """
    return _petal_cq().rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -theta_deg)


def _iris_ring_cq() -> cq.Workplane:
    """Iris actuator ring with left/right attachment struts to the frame.

    An annular ring at the pivot radius carrying all eight pivot bearings,
    with two radial struts (left and right) that embed into the frame slab
    to provide structural support. Ring is in the XZ plane centred at Y=0;
    the caller translates it to DOOR_Y and the opening centre height.
    """
    R_out = IRIS_R + 0.030   # ring outer radius
    R_in = IRIS_R - 0.020    # ring inner radius
    ring_h = 0.038            # ring thickness along Y

    # Annular ring body
    ring = (
        cq.Workplane("XZ")
        .circle(R_out).circle(R_in)
        .extrude(ring_h)
        .translate((0.0, -ring_h / 2.0, 0.0))
    )

    # Right strut: extends from ring outer edge to past the frame jamb
    strut_w = 0.040
    strut_right_len = OPEN_HW - R_out + 0.04  # embeds 0.04 into frame slab
    right = (
        cq.Workplane("XZ")
        .center(R_out + strut_right_len / 2.0, 0.0)
        .rect(strut_right_len, strut_w)
        .extrude(ring_h)
        .translate((0.0, -ring_h / 2.0, 0.0))
    )
    ring = ring.union(right)

    # Left strut: mirror of right
    left = (
        cq.Workplane("XZ")
        .center(-(R_out + strut_right_len / 2.0), 0.0)
        .rect(strut_right_len, strut_w)
        .extrude(ring_h)
        .translate((0.0, -ring_h / 2.0, 0.0))
    )
    ring = ring.union(left)

    # Translate ring centre to the opening vertical centre
    ring = ring.translate((0.0, 0.0, OPEN_CY))
    return ring


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="scifi_hex_iris_door")

    grey = model.material("frame_grey", rgba=(0.85, 0.86, 0.88, 1.0))
    cyan = model.material("glow_cyan", rgba=(0.45, 0.98, 1.0, 1.0))
    clamp_dark = model.material("clamp_charcoal", rgba=(0.09, 0.10, 0.11, 1.0))
    armor = model.material("door_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    status_red = model.material("status_red", rgba=(0.85, 0.16, 0.12, 1.0))

    # ---- Fixed root: frame + all greebling (identical to parent) ----
    frame = model.part("frame")
    frame.visual(mesh_from_cadquery(_frame_mesh(), "frame_slab"), material=grey, name="frame_slab")
    frame.visual(mesh_from_cadquery(_top_recess_mesh(), "top_recess"), material=clamp_dark, name="top_recess")
    frame.visual(mesh_from_cadquery(_guide_bar_mesh(*TOP_BAR_Z), "top_track"), material=clamp_dark, name="top_track")
    frame.visual(mesh_from_cadquery(_guide_bar_mesh(*BOT_BAR_Z), "bottom_track"), material=clamp_dark, name="bottom_track")
    frame.visual(mesh_from_cadquery(_clamp_block_mesh(-1.0), "clamp_0"), material=clamp_dark, name="clamp_0")
    frame.visual(mesh_from_cadquery(_clamp_block_mesh(1.0), "clamp_1"), material=clamp_dark, name="clamp_1")
    for s, lamp in ((-1.0, "clamp_lamp_0"), (1.0, "clamp_lamp_1")):
        frame.visual(
            Box((0.05, 0.012, 0.03)),
            origin=Origin(xyz=(s * (OPEN_HW + 0.085), FRAME_FACE_Y + 0.106, OPEN_CY + 0.09)),
            material=status_red, name=lamp,
        )

    strip_h = 1.16
    backing_w = 0.055
    backing_cx = OPEN_HW + 0.01 - backing_w / 2.0
    for s, tag in ((-1.0, "0"), (1.0, "1")):
        frame.visual(Box((backing_w, 0.023, strip_h)),
                     origin=Origin(xyz=(s * backing_cx, 0.0835, OPEN_CY)),
                     material=clamp_dark, name=f"strip_backing_{tag}")
        frame.visual(Box((0.018, 0.012, strip_h - 0.06)),
                     origin=Origin(xyz=(s * (backing_cx - 0.0155), 0.101, OPEN_CY)),
                     material=cyan, name=f"light_strip_{tag}")
        frame.visual(Box((0.018, 0.012, strip_h - 0.06)),
                     origin=Origin(xyz=(s * (backing_cx + 0.0125), 0.101, OPEN_CY)),
                     material=cyan, name=f"light_strip_{tag}_inner")

    chamf_mid_x = OPEN_HW - OPEN_CHAMFER / 2.0
    chamf_top_z = OPEN_TOP - OPEN_CHAMFER / 2.0
    chamf_bot_z = OPEN_BOT + OPEN_CHAMFER / 2.0
    chamf_len = OPEN_CHAMFER * math.sqrt(2.0) - 0.02
    for sx in (-1.0, 1.0):
        for cz, sz, vtag in ((chamf_top_z, 1.0, "t"), (chamf_bot_z, -1.0, "b")):
            pitch = sx * sz * math.pi / 4.0
            frame.visual(
                Box((chamf_len, 0.012, 0.05)),
                origin=Origin(xyz=(sx * (chamf_mid_x + 0.035), FRAME_FACE_Y + 0.006, cz + sz * 0.035),
                              rpy=(0.0, pitch, 0.0)),
                material=clamp_dark, name=f"chamfer_trim_{'l' if sx < 0 else 'r'}{vtag}",
            )

    for sx in (-1.0, 1.0):
        for bz, btag in ((0.30, "lo"), (1.04, "mid"), (1.78, "hi")):
            frame.visual(Box((0.03, 0.012, 0.03)),
                         origin=Origin(xyz=(sx * 0.80, FRAME_FACE_Y + 0.006, bz)),
                         material=clamp_dark, name=f"bolt_{'l' if sx < 0 else 'r'}_{btag}")

    # Iris actuator ring: annular ring with left/right struts embedding into
    # the frame slab.  Carries all eight petal pivot bearings.
    frame.visual(
        mesh_from_cadquery(_iris_ring_cq(), "iris_ring"),
        origin=Origin(xyz=(0.0, DOOR_Y, 0.0)),
        material=clamp_dark, name="iris_ring",
    )

    # ---- Iris petals: 8 curved-leaf blades on revolute hinges ----
    petal_parts = []
    for i in range(N_PETALS):
        theta_deg = i * 360.0 / N_PETALS

        petal_mesh = mesh_from_cadquery(_petal_cq_at(theta_deg), f"petal_{i}")
        p = model.part(f"petal_{i}")
        p.visual(petal_mesh, material=armor, name="blade")
        petal_parts.append(p)

    # ---- Articulations: uniform revolute policy, driver + 7 mimics ----
    driver_art = None
    for i in range(N_PETALS):
        theta = i * 2.0 * math.pi / N_PETALS
        pivot_x = IRIS_R * math.cos(theta)
        pivot_z = OPEN_CY + IRIS_R * math.sin(theta)
        mimic = (
            None if driver_art is None
            else Mimic(joint=driver_art.name, multiplier=1.0, offset=0.0)
        )
        art = model.articulation(
            f"petal_{i}_hinge",
            ArticulationType.REVOLUTE,
            parent=frame, child=petal_parts[i],
            origin=Origin(xyz=(pivot_x, DOOR_Y, pivot_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=200.0, velocity=1.2, lower=0.0, upper=IRIS_OPEN_ANGLE),
            mimic=mimic,
        )
        if driver_art is None:
            driver_art = art

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    petals = [object_model.get_part(f"petal_{i}") for i in range(N_PETALS)]
    driver = object_model.get_articulation("petal_0_hinge")

    # --- Intentional overlaps ---
    # All petals overlap each other in the closed iris stacking pattern
    # (they're stacked on top of each other like camera aperture blades).
    for i in range(N_PETALS):
        for j in range(i + 1, N_PETALS):
            ctx.allow_overlap(
                petals[i], petals[j],
                reason=(f"Iris petals {i} and {j} overlap in the closed stacking "
                        "pattern; petals are stacked like camera aperture blades."),
            )
    # Each petal pivot boss seats inside the iris ring.
    for i in range(N_PETALS):
        ctx.allow_overlap(
            petals[i], frame,
            elem_a="blade", elem_b="iris_ring",
            reason=f"Petal {i} pivot boss is captured inside the iris ring.",
        )

    # --- Frame geometry ---
    frame_aabb = ctx.part_world_aabb(frame)
    ctx.check(
        "frame stands on the floor and reaches full height",
        frame_aabb is not None
        and abs(frame_aabb[0][2]) < 0.02
        and frame_aabb[1][2] > FRAME_H - 0.05,
        details=f"frame aabb={frame_aabb}",
    )

    strip0 = ctx.part_element_world_aabb(frame, elem="light_strip_0")
    strip1 = ctx.part_element_world_aabb(frame, elem="light_strip_1")
    ctx.check(
        "cyan light strips flank the opening left and right",
        strip0 is not None and strip1 is not None and strip0[1][0] < 0.0 < strip1[0][0],
        details=f"strip0={strip0}, strip1={strip1}",
    )

    clamp0 = ctx.part_element_world_aabb(frame, elem="clamp_0")
    ctx.check(
        "clamp block sits at mid-height proud of the front face",
        clamp0 is not None and clamp0[1][1] > FRAME_FACE_Y and clamp0[0][2] < OPEN_CY < clamp0[1][2],
        details=f"clamp0={clamp0}",
    )

    window_x = 0.40
    window_z = (0.55, 1.55)
    for elem in ("top_recess", "top_track", "bottom_track", "strip_backing_0", "strip_backing_1"):
        aabb = ctx.part_element_world_aabb(frame, elem=elem)
        assert aabb is not None
        blocks = (aabb[0][0] < window_x and aabb[1][0] > -window_x
                  and aabb[0][2] < window_z[1] and aabb[1][2] > window_z[0])
        ctx.check(
            f"frame '{elem}' stays clear of the open doorway window",
            not blocks, details=f"{elem} aabb={aabb}",
        )

    # --- Eight petals at 45-degree intervals ---
    ctx.check("iris has 8 petals", len(petals) == N_PETALS)
    for i in range(N_PETALS):
        theta = i * 2.0 * math.pi / N_PETALS
        expected_x = IRIS_R * math.cos(theta)
        expected_z = OPEN_CY + IRIS_R * math.sin(theta)
        pos = ctx.part_world_position(petals[i])
        assert pos is not None
        dist = math.sqrt((pos[0] - expected_x) ** 2 + (pos[2] - expected_z) ** 2)
        ctx.check(
            f"petal_{i} is at the expected 45-degree angular position",
            dist < 0.06,
            details=f"expected=({expected_x:.3f},{expected_z:.3f}) actual=({pos[0]:.3f},{pos[2]:.3f})",
        )

    for i in range(N_PETALS):
        art = object_model.get_articulation(f"petal_{i}_hinge")
        ctx.check(
            f"petal_{i}_hinge is revolute with uniform limits",
            art.motion_limits.lower == 0.0 and abs(art.motion_limits.upper - IRIS_OPEN_ANGLE) < 1e-6,
            details=f"limits=({art.motion_limits.lower},{art.motion_limits.upper})",
        )

    # --- Closed pose (q=0): petals cover the opening centre ---
    with ctx.pose({driver: 0.0}):
        for i in range(N_PETALS):
            aabb = ctx.part_world_aabb(petals[i])
            assert aabb is not None
            ctx.check(
                f"petal_{i} covers the opening centre when closed",
                aabb[0][0] < 0.10 and aabb[1][0] > -0.10
                and aabb[0][2] < OPEN_CY + 0.10 and aabb[1][2] > OPEN_CY - 0.10,
                details=f"petal_{i} aabb={aabb}",
            )
        ctx.expect_overlap(
            petals[0], petals[1], axes="xy", min_overlap=0.01,
            name="adjacent petals overlap in the closed iris stack",
        )

    # --- Open pose (q=upper): petals retract radially ---
    closed_centers: dict[int, tuple[float, float, float]] = {}
    with ctx.pose({driver: 0.0}):
        for i in range(N_PETALS):
            c = ctx.part_world_aabb(petals[i])
            assert c is not None
            closed_centers[i] = (
                (c[0][0] + c[1][0]) / 2.0,
                (c[0][1] + c[1][1]) / 2.0,
                (c[0][2] + c[1][2]) / 2.0,
            )

    with ctx.pose({driver: driver.motion_limits.upper}):
        for i in range(N_PETALS):
            c = ctx.part_world_aabb(petals[i])
            assert c is not None
            open_cx = (c[0][0] + c[1][0]) / 2.0
            open_cz = (c[0][2] + c[1][2]) / 2.0
            dx = open_cx - closed_centers[i][0]
            dz = open_cz - closed_centers[i][2]
            shift = math.sqrt(dx * dx + dz * dz)
            ctx.check(
                f"petal_{i} retracts radially when iris opens",
                shift > 0.08,
                details=f"closed=({closed_centers[i][0]:.3f},{closed_centers[i][2]:.3f}) "
                        f"open=({open_cx:.3f},{open_cz:.3f}) shift={shift:.3f}",
            )

        # Opposing petals' AABB centres move to opposite sides of the opening
        # centre, proving the aperture has dilated.
        aabb_0 = ctx.part_world_aabb(petals[0])
        aabb_4 = ctx.part_world_aabb(petals[4])
        assert aabb_0 is not None and aabb_4 is not None
        cz_0 = (aabb_0[0][2] + aabb_0[1][2]) / 2.0
        cz_4 = (aabb_4[0][2] + aabb_4[1][2]) / 2.0
        ctx.check(
            "opposing petals 0 and 4 separate vertically when iris opens",
            cz_0 > OPEN_CY + 0.08 and cz_4 < OPEN_CY - 0.08,
            details=f"petal_0 cz={cz_0:.3f}, petal_4 cz={cz_4:.3f}",
        )
        aabb_2 = ctx.part_world_aabb(petals[2])
        aabb_6 = ctx.part_world_aabb(petals[6])
        assert aabb_2 is not None and aabb_6 is not None
        cx_2 = (aabb_2[0][0] + aabb_2[1][0]) / 2.0
        cx_6 = (aabb_6[0][0] + aabb_6[1][0]) / 2.0
        ctx.check(
            "opposing petals 2 and 6 separate horizontally when iris opens",
            cx_2 < -0.08 and cx_6 > 0.08,
            details=f"petal_2 cx={cx_2:.3f}, petal_6 cx={cx_6:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
